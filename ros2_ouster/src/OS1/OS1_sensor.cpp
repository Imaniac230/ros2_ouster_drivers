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

#include <string>

#include "ros2_ouster/OS1/OS1_sensor.hpp"
#include "ros2_ouster/conversions.hpp"
#include "ros2_ouster/exception.hpp"
#include "ros2_ouster/interfaces/common.hpp"

namespace OS1
{

OS1Sensor::OS1Sensor() : SensorInterface() {}

OS1Sensor::~OS1Sensor()
{
  _ouster_client.reset();
  _lidar_packet.clear();
  _imu_packet.clear();
}

void OS1Sensor::reset(const ros2_ouster::Configuration &config)
{
  _ouster_client.reset();
  configure(config);
}

void OS1Sensor::configure(const ros2_ouster::Configuration &config)
{
  if (!OS1::lidar_mode_of_string(config.lidar_mode)) {
    throw ros2_ouster::OusterDriverException(
            std::string("Invalid lidar mode %s!", config.lidar_mode.c_str()));
    exit(-1);
  }

  if (!OS1::timestamp_mode_of_string(config.timestamp_mode)) {
    throw ros2_ouster::OusterDriverException(std::string(
            "Invalid timestamp mode %s!", config.timestamp_mode.c_str()));
    exit(-1);
  }

  _ouster_client =
          OS1::init_client(config.lidar_ip, config.computer_ip,
                           OS1::lidar_mode_of_string(config.lidar_mode),
                           OS1::timestamp_mode_of_string(config.timestamp_mode),
                           config.lidar_port, config.imu_port);

  if (!_ouster_client) {
    throw ros2_ouster::OusterDriverException(
            std::string("Failed to create connection to lidar."));
  }
}

void OS1Sensor::allocateBuffers()
{
  _lidar_packet.resize(_packet_format->lidar_packet_size + 1);
  _imu_packet.resize(_packet_format->imu_packet_size + 1);
}

ros2_ouster::State OS1Sensor::poll()
{
  _state = poll_client(*_ouster_client);

  if (_state == client_state::EXIT) {
    std::cerr << "poll_client: caught signal, exiting!" << std::endl;
  }
  else if (_state == client_state::CLIENT_ERROR) {
    std::cerr << "Failed to get valid sensor data "
                 "information from lidar, returned error!"
              << std::endl;
  }

  return as_ouster_state(_state);
}

uint8_t *OS1Sensor::readPacket()
{
  if ((_state & client_state::LIDAR_DATA) &&
      read_lidar_packet(*_ouster_client, _lidar_packet.data(),
                        *_packet_format)) {
    std::cout << "got lidar packet size: " << _lidar_packet.size() << std::endl;
    return _lidar_packet.data();
  }
  if ((_state & client_state::IMU_DATA) &&
      read_imu_packet(*_ouster_client, _imu_packet.data(), *_packet_format)) {
    return _imu_packet.data();
  }

  return nullptr;
}

void OS1Sensor::updateConfigAndMetadata()
{
  if (_ouster_client) {
    if (!get_config(_ouster_client->hostname, _config)) {
      std::cerr << "Failed to collect sensor config" << std::endl;
      _config = {};
      _metadata.clear();
      _info = {};
      _packet_format.reset();
      return;
    }

    try {
      _metadata = OS1::get_metadata(*_ouster_client, 60, false);
    }
    catch (const std::exception &e) {
      std::cerr << "sensor::get_metadata exception: " << e.what() << std::endl;
      _metadata.clear();
    }
    //TODO(get-metadata): can the methods still return empty strings if no exceptions?
    if (_metadata.empty()) {
      std::cerr << "Failed to collect sensor metadata" << std::endl;
      return;
    }

    _info = OS1::parse_metadata(_metadata);
    // TODO: revist when *min_version* is changed
    populate_metadata_defaults(_info, OS1::MODE_UNSPEC);
    _packet_format = std::make_unique<packet_format>(OS1::get_format(_info));
  }
  else {
    _config = {};
    _metadata.clear();
    _info = {};
    _packet_format.reset();
  }
}

// fill in values that could not be parsed from metadata
void OS1Sensor::populate_metadata_defaults(sensor_info &info,
                                           lidar_mode specified_lidar_mode)
{
  if (info.name.empty()) info.name = "UNKNOWN";

  if (info.sn.empty()) info.sn = "UNKNOWN";

  OS1::version v = OS1::version_of_string(info.fw_rev);
  if (v == OS1::invalid_version) {
    std::cerr << "Unknown sensor firmware version; output may not be reliable"
              << std::endl;
  }
  else if (v < OS1::min_version) {
    std::cerr << "Firmware < " << to_string(OS1::min_version).c_str()
              << " not supported; output may not be reliable" << std::endl;
  }

  if (!info.mode) {
    std::cerr << "Lidar mode not found in metadata; output may not be reliable"
              << std::endl;
    info.mode = specified_lidar_mode;
  }

  if (info.prod_line.empty()) info.prod_line = "UNKNOWN";

  if (info.beam_azimuth_angles.empty() || info.beam_altitude_angles.empty()) {
    std::cerr << "Beam angles not found in metadata; using design values"
              << std::endl;
    info.beam_azimuth_angles = OS1::gen1_azimuth_angles;
    info.beam_altitude_angles = OS1::gen1_altitude_angles;
  }
}

}// namespace OS1
