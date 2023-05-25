// Copyright 2020, Steve Macenski
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/qos.hpp"
#include "ros2_ouster/ouster_driver.hpp"
#include "ros2_ouster/exception.hpp"
#include "ros2_ouster/interfaces/lifecycle_interface.hpp"
#include "ros2_ouster/interfaces/sensor_interface.hpp"
#include "ros2_ouster/OS1/processor_factories.hpp"

namespace ros2_ouster
{

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;
using namespace std::chrono_literals;

OusterDriver::OusterDriver(
  std::unique_ptr<SensorInterface> sensor,
  const rclcpp::NodeOptions & options)
: LifecycleInterface("OusterDriver", options), _sensor{std::move(sensor)}
{
  this->declare_parameter("lidar_ip");
  this->declare_parameter("computer_ip");
  this->declare_parameter("imu_port", 7503);
  this->declare_parameter("lidar_port", 7502);
  this->declare_parameter("lidar_mode", std::string("512x10"));
  this->declare_parameter(
    "timestamp_mode", std::string("TIME_FROM_INTERNAL_OSC"));
  this->declare_parameter("sensor_frame", std::string("laser_sensor_frame"));
  this->declare_parameter("laser_frame", std::string("laser_data_frame"));
  this->declare_parameter("imu_frame", std::string("imu_data_frame"));
  this->declare_parameter("udp_profile_lidar", "");
  this->declare_parameter("use_system_default_qos", false);
  this->declare_parameter("os1_proc_mask", std::string("IMG|PCL|IMU|SCAN"));
}

OusterDriver::~OusterDriver() = default;

void OusterDriver::onConfigure()
{
  ros2_ouster::Configuration lidar_config;
  try {
    lidar_config.lidar_ip = get_parameter("lidar_ip").as_string();
    lidar_config.computer_ip = get_parameter("computer_ip").as_string();
  } catch (...) {
    RCLCPP_FATAL(
      this->get_logger(),
      "Failed to get lidar or IMU IP address or "
      "hostname. An IP address for both are required!");
    exit(-1);
  }

  lidar_config.imu_port = get_parameter("imu_port").as_int();
  lidar_config.lidar_port = get_parameter("lidar_port").as_int();
  lidar_config.lidar_mode = get_parameter("lidar_mode").as_string();
  lidar_config.timestamp_mode = get_parameter("timestamp_mode").as_string();
  if (lidar_config.timestamp_mode == "TIME_FROM_ROS_RECEPTION") {
    RCLCPP_WARN(
      this->get_logger(),
      "Using TIME_FROM_ROS_RECEPTION to stamp data with ROS time on "
      "reception. This has unmodelled latency!");
    _use_ros_time = true;
  } else {
    _use_ros_time = false;
  }

  _laser_sensor_frame = get_parameter("sensor_frame").as_string();
  _laser_data_frame = get_parameter("laser_frame").as_string();
  _imu_data_frame = get_parameter("imu_frame").as_string();
  _use_system_default_qos = get_parameter("use_system_default_qos").as_bool();

  _os1_proc_mask =
    ros2_ouster::toProcMask(get_parameter("os1_proc_mask").as_string());

  RCLCPP_INFO(
    this->get_logger(),
    "Connecting to sensor at %s.", lidar_config.lidar_ip.c_str());
  RCLCPP_INFO(
    this->get_logger(),
    "Sending data from sensor to %s.", lidar_config.computer_ip.c_str());

  _reset_srv = this->create_service<std_srvs::srv::Empty>(
    "~/reset", std::bind(&OusterDriver::resetService, this, _1, _2, _3));
  _metadata_srv = this->create_service<ouster_msgs::srv::GetMetadata>(
    "~/get_metadata", std::bind(&OusterDriver::getMetadata, this, _1, _2, _3));

  try {
    _sensor->configure(lidar_config);
  } catch (const OusterDriverException & e) {
    RCLCPP_FATAL(this->get_logger(), "Exception thrown: (%s)", e.what());
    exit(-1);
  }

  _sensor->updateConfigAndMetadata();

  if (_use_system_default_qos) {
    RCLCPP_INFO(
      this->get_logger(), "Using system defaults QoS for sensor data");
    _data_processors = ros2_ouster::createProcessors(
      shared_from_this(), _sensor->getMetadata(), _imu_data_frame, _laser_data_frame,
      rclcpp::SystemDefaultsQoS(), _os1_proc_mask);
  } else {
    _data_processors = ros2_ouster::createProcessors(
      shared_from_this(), _sensor->getMetadata(), _imu_data_frame, _laser_data_frame,
      rclcpp::SensorDataQoS(), _os1_proc_mask);
  }

  _tf_b = std::make_unique<tf2_ros::StaticTransformBroadcaster>(
    shared_from_this());
  broadcastStaticTransforms(_sensor->getSensorInfo());
}

void OusterDriver::onActivate()
{
  _sensor->allocateBuffers();

  DataProcessorMapIt it;
  for (it = _data_processors.begin(); it != _data_processors.end(); ++it) {
    it->second->onActivate();
  }

  // Speed of the lidar is 1280 hz. We fire our timer event at 2x that rate to
  // ensure we can process all of the incoming data in a timely manner.
  // See: https://github.com/SteveMacenski/ros2_ouster_drivers/issues/55
  _process_timer = this->create_wall_timer(
    390625ns, std::bind(&OusterDriver::processData, this));
}

void OusterDriver::onError()
{
}

void OusterDriver::onDeactivate()
{
  _process_timer->cancel();
  _process_timer.reset();

  DataProcessorMapIt it;
  for (it = _data_processors.begin(); it != _data_processors.end(); ++it) {
    it->second->onDeactivate();
  }
}

void OusterDriver::onCleanup()
{
  _data_processors.clear();
  _tf_b.reset();
  _reset_srv.reset();
  _metadata_srv.reset();
}

void OusterDriver::onShutdown()
{
  _process_timer->cancel();
  _process_timer.reset();
  _tf_b.reset();

  DataProcessorMapIt it;
  for (it = _data_processors.begin(); it != _data_processors.end(); ++it) {
    delete it->second;
  }
  _data_processors.clear();
}

void OusterDriver::broadcastStaticTransforms(
  const OS1::sensor_info & info_data)
{
  if (_tf_b) {
    std::vector<geometry_msgs::msg::TransformStamped> transforms;
    transforms.push_back(
      toMsg(
        info_data.imu_to_sensor_transform,
        _laser_sensor_frame, _imu_data_frame, this->now()));
    transforms.push_back(
      toMsg(
        info_data.lidar_to_sensor_transform,
        _laser_sensor_frame, _laser_data_frame, this->now()));
    _tf_b->sendTransform(transforms);
  }
}

void OusterDriver::processData()
{
  try {
    State state = _sensor->poll();
    RCLCPP_DEBUG(
      this->get_logger(),
      "Packet with state: %s",
      ros2_ouster::toString(state).c_str());

    if (state == State::EXIT) {
      handlePollError();
      return;
    }

    uint8_t * packet_data = handlePacket(state);

    if (packet_data) {
      std::pair<DataProcessorMapIt, DataProcessorMapIt> key_its;
      key_its = _data_processors.equal_range(state);
      uint64_t override_ts =
        this->_use_ros_time ? this->now().nanoseconds() : 0;

      for (auto it = key_its.first; it != key_its.second; it++) {
        it->second->process(packet_data, override_ts);
      }
    }
  } catch (const OusterDriverException & e) {
    RCLCPP_WARN(
      this->get_logger(),
      "Failed to process packet with exception %s.", e.what());
  }
}

void OusterDriver::handlePollError() {
  RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 100,
                       "sensor::poll_client()) returned error");
  // in case error continues for a while attempt to recover by
  // performing sensor reset
  if (++_poll_error_count > _max_poll_error_count) {
    RCLCPP_ERROR_STREAM(
            get_logger(),
            "maximum number of allowed errors from "
            "sensor::poll_client() reached, performing self reset...");
    _poll_error_count = 0;
    _reset_srv.reset();
  }
}

uint8_t * OusterDriver::handlePacket(const ros2_ouster::State state) {
  uint8_t * packet_data = _sensor->readPacket();

  if (!packet_data) {
    if (++_packet_error_count > _max_packet_error_count) {
      RCLCPP_ERROR_STREAM(
              get_logger(),
              "maximum number of allowed errors from "
              "sensor::read_lidar_packet() reached, reactivating...");
      _packet_error_count = 0;
      //TODO(reset): perform only a reactivation instead of a full reset
      _reset_srv.reset();
    }
    return nullptr;
  }

  _packet_error_count = 0;
  if (_sensor->shouldReset(packet_data)) {
    // TODO: short circut reset if no breaking changes occured?
    RCLCPP_WARN(get_logger(),
                "sensor init_id has changed! reactivating..");
    //TODO(reset): perform only a reactivation instead of a full reset
    _reset_srv.reset();
    return nullptr;
  }

  return packet_data;
}

void OusterDriver::resetService(
  const std::shared_ptr<rmw_request_id_t>/*request_header*/,
  const std::shared_ptr<std_srvs::srv::Empty::Request> request,
  std::shared_ptr<std_srvs::srv::Empty::Response> response)
{
  if (!this->isActive()) {
    return;
  }

  ros2_ouster::Configuration lidar_config;
  lidar_config.lidar_ip = get_parameter("lidar_ip").as_string();
  lidar_config.computer_ip = get_parameter("computer_ip").as_string();
  lidar_config.imu_port = get_parameter("imu_port").as_int();
  lidar_config.lidar_port = get_parameter("lidar_port").as_int();
  lidar_config.lidar_mode = get_parameter("lidar_mode").as_string();
  lidar_config.timestamp_mode = get_parameter("timestamp_mode").as_string();
  _sensor->reset(lidar_config);
}

void OusterDriver::getMetadata(
  const std::shared_ptr<rmw_request_id_t>/*request_header*/,
  const std::shared_ptr<ouster_msgs::srv::GetMetadata::Request> request,
  std::shared_ptr<ouster_msgs::srv::GetMetadata::Response> response)
{
  if (!this->isActive()) {
    return;
  }
  response->metadata = toMsg(_sensor->getMetadata());
}

}  // namespace ros2_ouster
