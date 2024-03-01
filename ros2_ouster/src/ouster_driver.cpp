// Copyright 2021, Steve Macenski
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

#include <utility>
#include <vector>

#include "rclcpp/qos.hpp"
#include "ros2_ouster/exception.hpp"
#include "ros2_ouster/interfaces/lifecycle_interface.hpp"
#include "ros2_ouster/interfaces/sensor_interface.hpp"
#include "ros2_ouster/ouster_driver.hpp"
#include "ros2_ouster/processors/processor_factories.hpp"
#include "ros2_ouster/client/types.h"
#include "ros2_ouster/sensor.hpp"
#include "ros2_ouster/sensor_tins.hpp"

using namespace std::string_literals;

namespace ros2_ouster
{

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;
using namespace std::chrono_literals;

OusterDriver::OusterDriver(
  std::unique_ptr<SensorInterface> sensor,
  const rclcpp::NodeOptions & options)
: LifecycleInterface("OusterDriver", options), _sensor{std::move(sensor)},
      change_state_client{
              create_client<ChangeState>(get_name() + "/change_state"s)}
{
  // Declare parameters for configuring the _driver_
  this->declare_parameter("sensor_frame", std::string("laser_sensor_frame"));
  this->declare_parameter("laser_frame", std::string("laser_data_frame"));
  this->declare_parameter("imu_frame", std::string("imu_data_frame"));
  this->declare_parameter("use_system_default_qos", false);
  this->declare_parameter("proc_mask", std::string("IMG|PCL|IMU|SCAN"));
  this->declare_parameter("lidar_udp_profile", std::string("RNG19_RFL8_SIG16_NIR16"));

  // Declare parameters used across ALL _sensor_ implementations
  this->declare_parameter<std::string>("lidar_ip", "");
  this->declare_parameter<std::string>("computer_ip","");
  this->declare_parameter("imu_port", 7503);
  this->declare_parameter("lidar_port", 7502);
  this->declare_parameter("lidar_mode", std::string("512x10"));
  this->declare_parameter("timestamp_mode", std::string("TIME_FROM_INTERNAL_OSC"));
}

OusterDriver::~OusterDriver() = default;

void OusterDriver::onConfigure()
{
  // Get parameters for configuring the _driver_
  _laser_sensor_frame = get_parameter("sensor_frame").as_string();
  _laser_data_frame = get_parameter("laser_frame").as_string();
  _imu_data_frame = get_parameter("imu_frame").as_string();
  _use_system_default_qos = get_parameter("use_system_default_qos").as_bool();
  _proc_mask = ros2_ouster::toProcMask(get_parameter("proc_mask").as_string());

  // Get parameters used across ALL _sensor_ implementations. Parameters unique
  // a specific Sensor implementation are "getted" in the configure() function
  // for that sensor.
  ros2_ouster::Configuration lidar_config;
  lidar_config.imu_port = this->get_parameter("imu_port").as_int();
  lidar_config.lidar_port = this->get_parameter("lidar_port").as_int();
  lidar_config.lidar_mode = this->get_parameter("lidar_mode").as_string();
  lidar_config.timestamp_mode = this->get_parameter("timestamp_mode").as_string();
  lidar_config.lidar_udp_profile = get_parameter("lidar_udp_profile").as_string();

  // Deliberately retrieve the IP parameters in a try block without defaults, as
  // we cannot estimate a reasonable default IP address for the LiDAR/computer.
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

  if (lidar_config.timestamp_mode == "TIME_FROM_ROS_RECEPTION") {
    RCLCPP_WARN(
      this->get_logger(),
      "Using TIME_FROM_ROS_RECEPTION to stamp data with ROS time on "
      "reception. This has unmodelled latency!");
    lidar_config.timestamp_mode = "TIME_FROM_INTERNAL_OSC";
    _use_ros_time = true;
  } else {
    _use_ros_time = false;
  }

  // Configure the driver and sensor
  try {
    _sensor->configure(lidar_config, shared_from_this());
  } catch (const OusterDriverException & e) {
    RCLCPP_FATAL(this->get_logger(), "Exception thrown: (%s)", e.what());
    exit(-1);
  }

  _reset_srv = this->create_service<std_srvs::srv::Empty>(
    "~/reset", std::bind(&OusterDriver::resetService, this, _1, _2, _3));
  _metadata_srv = this->create_service<ouster_msgs::srv::GetMetadata>(
    "~/get_metadata", std::bind(&OusterDriver::getMetadata, this, _1, _2, _3));

  _full_rotation_accumulator = std::make_shared<sensor::FullRotationAccumulator>(
    _sensor->getMetadata(), _sensor->getPacketFormat());

  if (_use_system_default_qos) {
    RCLCPP_INFO(
      this->get_logger(), "Using system defaults QoS for sensor data");
    _data_processors = ros2_ouster::createProcessors(
      shared_from_this(), _sensor->getMetadata(), _imu_data_frame, _laser_data_frame,
      rclcpp::SystemDefaultsQoS(),
      _sensor->getPacketFormat(), _full_rotation_accumulator, _proc_mask);
  } else {
    _data_processors = ros2_ouster::createProcessors(
      shared_from_this(), _sensor->getMetadata(), _imu_data_frame, _laser_data_frame,
      rclcpp::SensorDataQoS(), _sensor->getPacketFormat(), _full_rotation_accumulator, _proc_mask);
  }

  _tf_b = std::make_unique<tf2_ros::StaticTransformBroadcaster>(
    shared_from_this());
  broadcastStaticTransforms(_sensor->getMetadata());
}

void OusterDriver::onActivate()
{
  DataProcessorMapIt it;
  for (it = _data_processors.begin(); it != _data_processors.end(); ++it) {
    it->second->onActivate();
  }

  _lidar_packet_buf = std::make_unique<RingBuffer>(
    _sensor->getPacketFormat().lidar_packet_size, 1024);
  _imu_packet_buf = std::make_unique<RingBuffer>(
    _sensor->getPacketFormat().imu_packet_size, 1024);

  _processing_active = true;
  _process_thread = std::thread(std::bind(&OusterDriver::processData, this));
  _recv_thread = std::thread(std::bind(&OusterDriver::receiveData, this));
}

void OusterDriver::onError()
{
}

void OusterDriver::onDeactivate()
{
  _processing_active = false;

  if (_recv_thread.joinable()) {
    _recv_thread.join();
  }

  _process_cond.notify_all();
  if (_process_thread.joinable()) {
    _process_thread.join();
  }

  for (auto & _data_processor : _data_processors) {
    _data_processor.second->onDeactivate();
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
  _tf_b.reset();

  for (auto & _data_processor : _data_processors) {
    _data_processor.second.reset();
  }
  _data_processors.clear();
}

void OusterDriver::broadcastStaticTransforms(
  const ouster::sensor::sensor_info & mdata)
{
  if (_tf_b) {
    std::vector<geometry_msgs::msg::TransformStamped> transforms;
    transforms.push_back(
      toMsg(
        mdata.imu_to_sensor_transform,
        _laser_sensor_frame, _imu_data_frame, this->now()));
    transforms.push_back(
      toMsg(
        mdata.lidar_to_sensor_transform,
        _laser_sensor_frame, _laser_data_frame, this->now()));
    _tf_b->sendTransform(transforms);
  }
}

void OusterDriver::processData() {
  std::pair<DataProcessorMapIt, DataProcessorMapIt> key_lidar_its =
    _data_processors.equal_range(ouster::sensor::client_state::LIDAR_DATA);
  std::pair<DataProcessorMapIt, DataProcessorMapIt> key_imu_its =
    _data_processors.equal_range(ouster::sensor::client_state::IMU_DATA);

  std::unique_lock<std::mutex> ringbuffer_guard(_ringbuffer_mutex);
  auto printed = std::chrono::high_resolution_clock::now();
  while (_processing_active.load()) {
    // Wait for data in either the lidar or imu ringbuffer
    _process_cond.wait(
      ringbuffer_guard, [this] () {
        return (!_lidar_packet_buf->empty() ||
                !_imu_packet_buf->empty() ||
                !_processing_active.load());
    });
    ringbuffer_guard.unlock();

    uint64_t override_ts = this->_use_ros_time ? this->now().nanoseconds() : 0;

    // If we have data in the lidar buffer, process it
    if (!_lidar_packet_buf->empty() && _processing_active.load()) {
      _full_rotation_accumulator->accumulate(_lidar_packet_buf->head(), override_ts);
      for (auto it = key_lidar_its.first; it != key_lidar_its.second; it++) {
        it->second->process(_lidar_packet_buf->head(), override_ts);
      }
      _lidar_packet_buf->pop();

      ++readCounter;
      uint64_t counterMax = UINT64_MAX;
      readCounter.compare_exchange_strong(counterMax, 0);
      const uint64_t counterDiff = writeCounter.load() - readCounter.load();
//      if ((counterDiff > 0) && ((std::chrono::high_resolution_clock::now() - printed) > std::chrono::milliseconds(20))) {
//        std::cout << "unprocessed packets: " << counterDiff << " (written: " << writeCounter.load() << ", read: " << readCounter.load() << ")" << std::endl;
//        std::cout << "buffer active items: " << _lidar_packet_buf->size() << std::endl;
//        printed = std::chrono::high_resolution_clock::now();
//      }
    }

    // If we have data in the imu buffer, process it
    if (!_imu_packet_buf->empty() && _processing_active.load()) {
      for (auto it = key_imu_its.first; it != key_imu_its.second; it++) {
        it->second->process(_imu_packet_buf->head(), override_ts);
      }
      _imu_packet_buf->pop();
    }

    ringbuffer_guard.lock();
  }
}

void OusterDriver::receiveData()
{
  while (_processing_active.load()) {
    try {
      // Receive raw sensor data from the network.
      // This blocks for some time until either data is received or timeout
      ouster::sensor::client_state state = _sensor->get();

      if (state == ouster::sensor::client_state::EXIT) {
        handlePollError();
        return;
      }

      bool got_lidar = handleLidarPacket(state);
      bool got_imu = handleImuPacket(state);

      // If we got some data, push to ringbuffer and signal processing thread
      if (got_lidar || got_imu) {
        if (got_lidar) {
          // If the ringbuffer is full, this means the processing thread is running too slow to
          // process all frames. Therefore, we push to it anyway, discarding all (old) data
          // in the buffer and emit a warning.
          if (_lidar_packet_buf->full()) {
            RCLCPP_WARN(this->get_logger(), "Lidar buffer overrun!");
          }
          _lidar_packet_buf->push();
        }

        if (got_imu) {
          if (_imu_packet_buf->full()) {
            RCLCPP_WARN(this->get_logger(), "IMU buffer overrun!");
          }
          _imu_packet_buf->push();
        }
        _process_cond.notify_all();
      }

      RCLCPP_DEBUG(
        this->get_logger(),
        "Retrieved packet with state: %s",
        ros2_ouster::toString(state).c_str());
    } catch (const OusterDriverException & e) {
      RCLCPP_WARN(
        this->get_logger(),
        "Failed to process packet with exception %s.", e.what());
    }
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
    reset_sensor(true);
  }
}

bool OusterDriver::handleLidarPacket(const ouster::sensor::client_state & state)
{
  if (!(state & ouster::sensor::client_state::LIDAR_DATA)) {
    return false;
  }

  const auto timeStamp = std::chrono::high_resolution_clock::now();
  const auto timeDiff = (timeStamp - lastTimeStamp).count();
  const auto epochStamp = timeStamp.time_since_epoch().count();
  uint8_t* buf = _lidar_packet_buf->tail();

  if (!_sensor->readLidarPacket(state, buf)) {
    if (++_lidar_packet_error_count > _max_lidar_packet_error_count) {
      RCLCPP_ERROR_STREAM(
              get_logger(),
              "maximum number of allowed errors from "
              "sensor::read_lidar_packet() reached, reactivating...");
      _lidar_packet_error_count = 0;
      reactivate_sensor(true);
    }
    std::cout << "[" << epochStamp << "] ERROR reading lidar packet! (time diff: " << std::setw(10) << timeDiff << std::setw(0) << " ns)" << std::endl;
    return false;
  }

  _lidar_packet_error_count = 0;
  if (_sensor->shouldReset(state, _lidar_packet_buf->tail())) {
    // TODO: short circut reset if no breaking changes occured?
    RCLCPP_WARN(get_logger(),
                "sensor init_id has changed! reactivating..");
    reactivate_sensor(false);
    return false;
  }

  const auto pf = _sensor->getPacketFormat();
  const uint16_t f_id = pf.frame_id(buf);
  ++writeCounter;
  uint64_t counterMax = UINT64_MAX;
  writeCounter.compare_exchange_strong(counterMax, 0);
  const uint16_t fIDDiff = f_id - lastFrameID;
  if (fIDDiff > 1) {
    std::cout << "[" << epochStamp << "] missing " << (fIDDiff - 1) << " whole frames (last f_id: " << lastFrameID << ", new f_id: " << f_id << ", time diff: " << std::setw(10) << timeDiff << std::setw(0) << " ns)" << std::endl;
  }
  for (int icol = 0; icol < pf.columns_per_packet; icol++) {
    const uint8_t* col_buf = pf.nth_col(icol, buf);
    const uint16_t m_id = pf.col_measurement_id(col_buf);
    if (f_id == lastFrameID) {
      const uint16_t mIDDiff = m_id - lastMeasID;
      if (mIDDiff > 1) {
        std::cout << "[" << epochStamp << "] missing " << (mIDDiff + 1) / 16 << " packets (last m_id: " << lastMeasID << ", new m_id: " << m_id << ", time diff: " << std::setw(10) << timeDiff << std::setw(0) << " ns)" << std::endl;
      }
      if (mIDDiff < 1) {
        std::cout << "[" << epochStamp << "] got the same packet again, (last m_id: " << lastMeasID << ", new m_id: " << m_id << ", time diff: " << std::setw(10) << timeDiff << std::setw(0) << " ns)" << std::endl;
      }
    }
    lastMeasID = m_id;
  }
  lastFrameID = f_id;
  lastTimeStamp = timeStamp;
  //    std::cout << "time diff: " << std::setw(10) << timeDiff << std::setw(0) << " ns" << std::endl;

  return true;
}

bool OusterDriver::handleImuPacket(const ouster::sensor::client_state & state)
{
  if (!(state & ouster::sensor::client_state::IMU_DATA)) return false;

  if (!_sensor->readImuPacket(state, _imu_packet_buf->tail())) {
    if (++_imu_packet_error_count > _max_imu_packet_error_count) {
      RCLCPP_ERROR_STREAM(
              get_logger(),
              "maximum number of allowed errors from "
              "sensor::read_lidar_packet() reached, reactivating...");
      _imu_packet_error_count = 0;
      reactivate_sensor(true);
    }
    return false;
  }

  _imu_packet_error_count = 0;
  return true;
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
  _sensor->reset(lidar_config, shared_from_this());
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

  // Save the metadata to file ONLY if the user specifies a filepath
  if (!request->metadata_filepath.empty()) {
    std::string json_config = ouster::sensor::to_string(_sensor->getMetadata());
    std::ofstream ofs;
    ofs.open(request->metadata_filepath);
    ofs << json_config << std::endl;
    ofs.close();
    if (!ofs) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Failed to save metadata to: %s.",
        request->metadata_filepath.c_str());
    } else {
      RCLCPP_INFO(
        this->get_logger(),
        "Saving metadata to a .json file specifed here: %s",
        request->metadata_filepath.c_str());
    }
  }
}

std::string OusterDriver::transition_id_to_string(uint8_t transition_id) {
  switch (transition_id) {
    case lifecycle_msgs::msg::Transition::TRANSITION_CREATE:
      return "create"s;
    case lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE:
      return "configure"s;
    case lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP:
      return "cleanup"s;
    case lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE:
      return "activate";
    case lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE:
      return "deactivate"s;
    case lifecycle_msgs::msg::Transition::TRANSITION_DESTROY:
      return "destroy"s;
    default:
      return "unknown"s;
  }
}

template <typename CallbackT, typename... CallbackT_Args>
bool OusterDriver::change_state(std::uint8_t transition_id, CallbackT callback,
                          CallbackT_Args... callback_args,
                          std::chrono::seconds time_out) {
  if (!change_state_client->wait_for_service(time_out)) {
    RCLCPP_ERROR_STREAM(
            get_logger(), "Service " << change_state_client->get_service_name()
                                     << "is not available.");
    return false;
  }

  auto request = std::make_shared<ChangeState::Request>();
  request->transition.id = transition_id;
  // send an async request to perform the transition
  change_state_client->async_send_request(
          request, [callback,
                    callback_args...](rclcpp::Client<ChangeState>::SharedFuture) {
            callback(callback_args...);
          });
  return true;
}

void OusterDriver::execute_transitions_sequence(
        std::vector<uint8_t> transitions_sequence, size_t at) {
  assert(at < transitions_sequence.size() &&
         "at index exceeds the number of transitions");
  auto transition_id = transitions_sequence[at];
  RCLCPP_DEBUG_STREAM(
          get_logger(), "transition: [" << transition_id_to_string(transition_id)
                                        << "] started");
  change_state(transition_id, [this, transitions_sequence, at]() {
    RCLCPP_DEBUG_STREAM(
            get_logger(),
            "transition: [" << transition_id_to_string(transitions_sequence[at])
                            << "] completed");
    if (at < transitions_sequence.size() - 1) {
      execute_transitions_sequence(transitions_sequence, at + 1);
    } else {
      RCLCPP_DEBUG_STREAM(get_logger(), "transitions sequence completed");
    }
  });
}

// param init_id_reset is overriden to true when force_reinit is true
void OusterDriver::reset_sensor(bool force_reinit, bool init_id_reset) {
  if (!_processing_active.load()) {
    RCLCPP_WARN(get_logger(),
                "sensor reset is invoked but sensor connection is not "
                "active, ignoring call!");
    return;
  }

  _sensor->reset_sensor(force_reinit, init_id_reset);
  auto request_transitions = std::vector<uint8_t>{
          lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE,
          lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP,
          lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE,
          lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE};
  execute_transitions_sequence(request_transitions, 0);
}

// TODO: need to notify dependent node(s) of the update
void OusterDriver::reactivate_sensor(bool init_id_reset) {
  if (!_processing_active.load()) {
    //     This may indicate that we are in the process of re-activation
    RCLCPP_WARN(get_logger(),
                "sensor reactivate is invoked but sensor connection is "
                "not active, ignoring call!");
    return;
  }

  _sensor->reactivate_sensor(init_id_reset);
  _sensor->update_metadata();
//  publish_metadata();
//  save_metadata();
  auto request_transitions = std::vector<uint8_t>{
          lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE,
          lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE};
  execute_transitions_sequence(request_transitions, 0);
}

}  // namespace ros2_ouster
