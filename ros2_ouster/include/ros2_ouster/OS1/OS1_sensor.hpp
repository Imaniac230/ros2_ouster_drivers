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

#ifndef ROS2_OUSTER__OS1__OS1_SENSOR_HPP_
#define ROS2_OUSTER__OS1__OS1_SENSOR_HPP_

#include <memory>
#include <optional>
#include <vector>

#include "ros2_ouster/OS1/processor_factories.hpp"

#include "ros2_ouster/OS1/OS1_client.hpp"
#include "ros2_ouster/interfaces/data_processor_interface.hpp"
#include "ros2_ouster/interfaces/sensor_interface.hpp"

namespace OS1
{

inline ros2_ouster::State as_ouster_state(const client_state & right)
{
  switch (right) {
    case client_state::TIMEOUT:
      return ros2_ouster::State::TIMEOUT;
    case CLIENT_ERROR:
      return ros2_ouster::State::ERROR;
    case LIDAR_DATA:
      return ros2_ouster::State::LIDAR_DATA;
    case IMU_DATA:
      return ros2_ouster::State::IMU_DATA;
    case EXIT:
      return ros2_ouster::State::EXIT;
    default:
      return ros2_ouster::State::ERROR;
  }
}

class OS1Sensor : public ros2_ouster::SensorInterface
{
  public:
  OS1Sensor();

  ~OS1Sensor() override;

  /**
   * @brief Reset lidar sensor
   * @param configuration file to use
   */
  void reset(const ros2_ouster::Configuration & config) override;

  /**
   * @brief Configure lidar sensor
   * @param configuration file to use
   */
  void configure(const ros2_ouster::Configuration & config) override;

  /**
   * @brief Allocate sensor data buffers based on the active configuration
   */
  void allocateBuffers() override;

  /**
   * @brief Update and store the currently active sensor config and metadata
   */
  void updateConfigAndMetadata() override;

  /**
   * @brief Get lidar sensor's metadata
   * @return sensor metadata struct
   */
  [[nodiscard]] inline const std::string & getMetadata() const override
  {
    return _metadata;
  }

  [[nodiscard]] inline const OS1::sensor_info & getSensorInfo() const override
  {
    return _info;
  }

  /**
   * @brief Ask sensor to get its current state for data collection
   * @return the state enum value
   */
  ros2_ouster::State poll() override;

  /**
   * @brief reading the packet corresponding to the sensor state
   * @param state of the sensor
   * @return the packet of data
   */
  uint8_t * readPacket() override;

  /**
   * @brief Indicate whether a reactivation operation is required
   * @return sensor metadata struct
   */
  inline bool shouldReset(const uint8_t * packet) override
  {
    return (_state == client_state::LIDAR_DATA) &&
           is_non_legacy_lidar_profile(_info) &&
           init_id_changed(*_packet_format, packet);
  }

  private:
  [[nodiscard]] std::shared_ptr<client>
  configure_and_initialize_sensor(const ros2_ouster::Configuration & config);

  uint8_t compose_config_flags(const sensor_config & config);

  inline bool init_id_changed(const packet_format & pf,
                              const uint8_t * lidar_buf)
  {
    uint32_t current_init_id = pf.init_id(lidar_buf);
    if (!last_init_id_initialized) {
      last_init_id = current_init_id + 1;
      last_init_id_initialized = true;
    }
    if (reset_last_init_id && last_init_id != current_init_id) {
      last_init_id = current_init_id;
      reset_last_init_id = false;
      return false;
    }
    if (last_init_id == current_init_id) return false;
    last_init_id = current_init_id;
    return true;
  }

  inline static bool is_non_legacy_lidar_profile(const sensor_info & info)
  {
    return info.format.udp_profile_lidar !=
           UDPProfileLidar::PROFILE_LIDAR_LEGACY;
  }

  static void populate_metadata_defaults(sensor_info & info,
                                         lidar_mode specified_lidar_mode);

  std::shared_ptr<client> _ouster_client;
  std::vector<uint8_t> _lidar_packet;
  std::vector<uint8_t> _imu_packet;
  sensor_config _config{};
  client_state _state{};
  std::string _metadata;
  sensor_info _info{};
  std::unique_ptr<packet_format> _packet_format;

  bool force_sensor_reinit = false;
  bool reset_last_init_id = true;
  bool last_init_id_initialized = false;
  uint32_t last_init_id{};
};

}// namespace OS1

#endif// ROS2_OUSTER__OS1__OS1_SENSOR_HPP_
