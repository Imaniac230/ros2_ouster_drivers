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

#ifndef ROS2_OUSTER__OS1__PROCESSORS__SCAN_PROCESSOR_HPP_
#define ROS2_OUSTER__OS1__PROCESSORS__SCAN_PROCESSOR_HPP_

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/qos.hpp"

#include "ros2_ouster/conversions.hpp"

#include "sensor_msgs/msg/laser_scan.hpp"

#include "ros2_ouster/OS1/OS1_client.hpp"
#include "ros2_ouster/OS1/OS1_util.hpp"
#include "ros2_ouster/interfaces/data_processor_interface.hpp"

namespace OS1
{
/**
 * @class OS1::ScanProcessor
 * @brief A data processor interface implementation of a processor
 * for creating Scans in the
 * driver in ROS2.
 */
class ScanProcessor : public ros2_ouster::DataProcessorInterface
{
  public:
  using OSScan = std::vector<scan_os::ScanOS>;
  using OSScanIt = OSScan::iterator;

  /**
   * @brief A constructor for OS1::ScanProcessor
   * @param node Node for creating interfaces
   * @param mdata metadata about the sensor
   * @param frame frame_id to use for messages
   */
  ScanProcessor(const rclcpp_lifecycle::LifecycleNode::SharedPtr node,
                const std::string &mdata, const std::string &frame,
                const rclcpp::QoS &qos)
      : DataProcessorInterface(), _node(node), _frame(frame),
        _info(OS1::parse_metadata(mdata)), _pf(OS1::get_format(_info))
  {
    _pub = _node->create_publisher<sensor_msgs::msg::LaserScan>("scan", qos);
    _height = _pf.pixels_per_column;
    _width = OS1::n_cols_of_lidar_mode(_info.mode);
    _xyz_lut = OS1::make_xyz_lut(_width, _height, _info.beam_azimuth_angles,
                                 _info.beam_altitude_angles);
    _aggregated_scans.resize(_width * _height);

    double zero_angle = 9999.0;
    _ring = 0;
    for (uint i = 0; i != _info.beam_altitude_angles.size(); i++) {
      if (fabs(_info.beam_altitude_angles[i]) < zero_angle) {
        _ring = static_cast<uint8_t>(i);
        zero_angle = fabs(_info.beam_altitude_angles[i]);
      }
    }

    _batch_and_publish = OS1::batch_to_iter<OSScanIt>(
            _xyz_lut, _width, _height, _pf, {}, &scan_os::ScanOS::make,
            [&](uint64_t scan_ts) mutable {
              if (_pub->get_subscription_count() > 0 && _pub->is_activated()) {
                auto msg_ptr = std::make_unique<sensor_msgs::msg::LaserScan>(
                        std::move(ros2_ouster::toMsg(
                                _aggregated_scans,
                                std::chrono::nanoseconds(scan_ts), _frame,
                                mdata, _ring)));
                _pub->publish(std::move(msg_ptr));
              }
            });
  }

  /**
   * @brief A destructor clearing memory allocated
   */
  ~ScanProcessor() { _pub.reset(); }

  /**
   * @brief Process method to create scan
   * @param data the packet data
   */
  bool process(uint8_t *data, uint64_t override_ts) override
  {
    OSScanIt it = _aggregated_scans.begin();
    _batch_and_publish(data, it, override_ts);
    return true;
  }

  /**
   * @brief Activating processor from lifecycle state transitions
   */
  void onActivate() override { _pub->on_activate(); }

  /**
   * @brief Deactivating processor from lifecycle state transitions
   */
  void onDeactivate() override { _pub->on_deactivate(); }

  private:
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::LaserScan>::SharedPtr
          _pub;
  std::function<void(const uint8_t *, OSScanIt, uint64_t)> _batch_and_publish;
  rclcpp_lifecycle::LifecycleNode::SharedPtr _node;
  std::vector<double> _xyz_lut;
  OSScan _aggregated_scans;
  std::string _frame;
  uint32_t _height;
  uint32_t _width;
  uint8_t _ring;
  //TODO(OS1-data): abstract this away to make ti independent of the OS1 structs?
  OS1::sensor_info _info;
  OS1::packet_format _pf;
};

}// namespace OS1

#endif// ROS2_OUSTER__OS1__PROCESSORS__SCAN_PROCESSOR_HPP_
