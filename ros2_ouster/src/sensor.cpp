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

#include "ros2_ouster/sensor.hpp"

#include <string>
#include <sstream>

#include "ros2_ouster/client/logging.h"
#include "ros2_ouster/client/client.h"
#include "ros2_ouster/exception.hpp"
#include "ros2_ouster/interfaces/metadata.hpp"

namespace sensor
{

Sensor::Sensor()
: SensorInterface() {}

Sensor::~Sensor()
{
  _ouster_client.reset();
}

void Sensor::reset(
  ros2_ouster::Configuration & config,
  rclcpp_lifecycle::LifecycleNode::SharedPtr node)
{
  _ouster_client.reset();

  configure(config, node);
}

void Sensor::configure(
  ros2_ouster::Configuration & config,
  rclcpp_lifecycle::LifecycleNode::SharedPtr node)
{
  RCLCPP_INFO(
    node->get_logger(), 
    "Configuring Ouster driver node.");

  // Check the validity of some of the retrieved parameters
  if (!ouster::sensor::lidar_mode_of_string(config.lidar_mode)) {
    throw ros2_ouster::OusterDriverException(
            "Invalid lidar mode: " + config.lidar_mode);
  }

  if (!ouster::sensor::timestamp_mode_of_string(config.timestamp_mode)) {
    throw ros2_ouster::OusterDriverException(
            "Invalid timestamp mode: " + config.timestamp_mode);
  }

  // Report to the user whether automatic address detection is being used, and 
  // what the source / destination IPs are
  if (config.lidar_ip.empty()) {
    throw ros2_ouster::OusterDriverException(
            "Cannot connect to sensor, lidar ip was not provided.");
  }
  RCLCPP_INFO(
    node->get_logger(),
    "Connecting to sensor at %s.", config.lidar_ip.c_str());
  if (config.computer_ip.empty()) {
    RCLCPP_INFO(
      node->get_logger(),
      "Sending data from sensor to computer using automatic address detection");
  }  else {
    RCLCPP_INFO(
      node->get_logger(),
      "Sending data from sensor to %s.", config.computer_ip.c_str());
  }

  _ouster_client = configure_and_initialize_sensor(config);

  if (!_ouster_client) {
    throw ros2_ouster::OusterDriverException(
            std::string("Failed to create connection to lidar."));
  }

  setMetadata(config.lidar_port, config.imu_port, config.timestamp_mode);

  display_lidar_info(_metadata);
}

bool Sensor::shouldReset(const ouster::sensor::client_state & state, const uint8_t * packet)
{
  return (state & ouster::sensor::client_state::LIDAR_DATA) &&
         is_non_legacy_lidar_profile(getMetadata()) &&
         init_id_changed(getPacketFormat(), packet);
}

std::shared_ptr<ouster::sensor::client> Sensor::configure_and_initialize_sensor(
        const ros2_ouster::Configuration &config)
{
  ouster::sensor::sensor_config sensor_config{};
  if (!config.computer_ip.empty()) sensor_config.udp_dest = config.computer_ip;
  sensor_config.udp_port_imu = config.imu_port;
  sensor_config.udp_port_lidar = config.lidar_port;
  sensor_config.ld_mode = ouster::sensor::lidar_mode_of_string(config.lidar_mode);
  sensor_config.ts_mode = ouster::sensor::timestamp_mode_of_string(config.timestamp_mode);
  sensor_config.udp_profile_lidar =
          ouster::sensor::udp_profile_lidar_of_string(config.lidar_udp_profile);

  uint8_t config_flags = compose_config_flags(sensor_config);
  if (!set_config(config.lidar_ip, sensor_config, config_flags)) {
    throw std::runtime_error("Error connecting to sensor " + config.lidar_ip);
  }

  ouster::sensor::logger().info("Sensor {} configured successfully, initializing client.", config.lidar_ip);

  return ouster::sensor::init_client(config.lidar_ip, sensor_config.udp_dest.value_or(""),
                          sensor_config.ld_mode.value(),
                          sensor_config.ts_mode.value(),
                          sensor_config.udp_port_lidar.value(),
                          sensor_config.udp_port_imu.value());
}

ouster::sensor::client_state Sensor::get()
{
  const ouster::sensor::client_state state = ouster::sensor::poll_client(*_ouster_client);

  if (state == ouster::sensor::client_state::EXIT) {
    throw ros2_ouster::OusterDriverException(
            std::string(
              "Failed to get valid sensor data "
              "information from lidar, returned exit!"));
  } else if (state & ouster::sensor::client_state::CLIENT_ERROR) {
    throw ros2_ouster::OusterDriverException(
            std::string(
              "Failed to get valid sensor data "
              "information from lidar, returned error!"));
  }
  return state;
}

bool Sensor::readLidarPacket(const ouster::sensor::client_state & state, uint8_t * buf)
{
  if (state & ouster::sensor::client_state::LIDAR_DATA)
  {
    const auto pf = this->getPacketFormat();
    const bool success = ouster::sensor::read_lidar_packet(*_ouster_client, buf, pf);

    if (!success) std::cout << "ERROR reading lidar packet!" << std::endl;
    const uint16_t f_id = pf.frame_id(buf);
    const uint16_t fIDDiff = f_id - lastFrameID;
    if (fIDDiff > 1) {
      std::cout << "missing " << (fIDDiff - 1) << " whole frames (last f_id: " << lastFrameID << ", new f_id: " << f_id << ")" << std::endl;
    }
    for (int icol = 0; icol < pf.columns_per_packet; icol++) {
      const uint8_t* col_buf = pf.nth_col(icol, buf);
      const uint16_t m_id = pf.col_measurement_id(col_buf);
      if (f_id == lastFrameID) {
        const uint16_t mIDDiff = m_id - lastMeasID;
        if (mIDDiff > 1) {
          std::cout << "missing " << (mIDDiff + 1) / 16 << " packets (last m_id: " << lastMeasID << ", new m_id: " << m_id << ")" << std::endl;
        }
        if (mIDDiff < 1) {
          std::cout << "got the same packet again, (last m_id: " << lastMeasID << ", new m_id: " << m_id << ")" << std::endl;
        }
      }
      lastMeasID = m_id;
    }
    lastFrameID = f_id;

    if (success)
      return true;
  }
  std::cout << "INVALID CALL TO readLidarPacket() !!!" << std::endl;
  return false;
}

bool Sensor::readImuPacket(const ouster::sensor::client_state & state, uint8_t * buf)
{
  if (state & ouster::sensor::client_state::IMU_DATA &&
    ouster::sensor::read_imu_packet(
      *_ouster_client, buf,
      this->getPacketFormat()))
  {
    return true;
  }
  return false;
}

void Sensor::setMetadata(
  int lidar_port, int imu_port,
  const std::string & timestamp_mode)
{
  if (_ouster_client) {
    _metadata = ros2_ouster::Metadata(
      ouster::sensor::parse_metadata(
        ouster::sensor::get_metadata(*_ouster_client)),
      imu_port, lidar_port, timestamp_mode);
  }
  ros2_ouster::populate_missing_metadata_defaults(_metadata, ouster::sensor::MODE_UNSPEC);
}

ros2_ouster::Metadata Sensor::getMetadata()
{
  return _metadata;
}

ouster::sensor::packet_format Sensor::getPacketFormat()
{
  return ouster::sensor::get_format(getMetadata());
}

uint8_t Sensor::compose_config_flags(const ouster::sensor::sensor_config &config)
{
  uint8_t config_flags = 0;
  if (config.udp_dest) {
    ouster::sensor::logger().info("Will send UDP data to {}", config.udp_dest.value());
  }
  else {
    ouster::sensor::logger().info("Will use automatic UDP destination");
    config_flags |= ouster::sensor::CONFIG_UDP_DEST_AUTO;
  }

  if (force_sensor_reinit) {
    force_sensor_reinit = false;
    ouster::sensor::logger().info("Forcing sensor to reinitialize");
    config_flags |= ouster::sensor::CONFIG_FORCE_REINIT;
  }

  return config_flags;
}

void Sensor::reset_sensor(bool force_reinit, bool init_id_reset) {
  force_sensor_reinit = force_reinit;
  reset_last_init_id = force_reinit || init_id_reset;
}

void Sensor::reactivate_sensor(bool init_id_reset) {
  reset_last_init_id = init_id_reset;
}

void Sensor::update_metadata() {
  const auto old_metadata = _metadata;
  try {
    _metadata = ros2_ouster::Metadata(
            ouster::sensor::parse_metadata(
                    ouster::sensor::get_metadata(*_ouster_client, 60)),
            old_metadata.udp_port_imu, old_metadata.udp_port_lidar, old_metadata.timestamp_mode);
  } catch (const std::exception& e) {
    ouster::sensor::logger().error("sensor::get_metadata exception: {}", e.what());
    _metadata = {};
  }

  ros2_ouster::populate_missing_metadata_defaults(_metadata, ouster::sensor::MODE_UNSPEC);

  //    publish_metadata();
  //    save_metadata();
  display_lidar_info(_metadata);
}

void Sensor::display_lidar_info(const ros2_ouster::Metadata& meta) {
    ouster::sensor::logger().info(
          "sensor name: {}\n"
          "\t\tproduct: {}, sn: {}, firmware rev: {}\n"
          "\t\tlidar mode: {}, lidar udp profile: {}\n"
          /*"\t\touster client version: {}"*/,
          meta.name,
          meta.prod_line, meta.sn, meta.fw_rev,
          ouster::sensor::to_string(meta.mode), ouster::sensor::to_string(meta.format.udp_profile_lidar)/*,
          ouster::SDK_VERSION_FULL*/);
}

}  // namespace sensor
