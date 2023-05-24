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

#ifndef ROS2_OUSTER__OS1__PROCESSORS__POINTCLOUD_PROCESSOR_HPP_
#define ROS2_OUSTER__OS1__PROCESSORS__POINTCLOUD_PROCESSOR_HPP_

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/qos.hpp"

#include "ros2_ouster/conversions.hpp"

#include "sensor_msgs/msg/point_cloud2.hpp"

#include "ros2_ouster/OS1/OS1_client.hpp"
#include "ros2_ouster/OS1/OS1_util.hpp"
#include "ros2_ouster/interfaces/data_processor_interface.hpp"

namespace OS1
{
/**
 * @class OS1::PointcloudProcessor
 * @brief A data processor interface implementation of a processor
 * for creating Pointclouds in the
 * driver in ROS2.
 */
class PointcloudProcessor : public ros2_ouster::DataProcessorInterface
{
  public:
  using OSCloud = pcl::PointCloud<point_os::PointOS>;
  using OSCloudIt = OSCloud::iterator;
  /**
   * @brief A constructor for OS1::PointcloudProcessor
   * @param node Node for creating interfaces
   * @param mdata metadata about the sensor
   * @param frame frame_id to use for messages
   */
  PointcloudProcessor(const rclcpp_lifecycle::LifecycleNode::SharedPtr node,
                      const std::string &mdata, const std::string &frame,
                      const rclcpp::QoS &qos)
      : DataProcessorInterface(), _node(node), _frame(frame),
        _info(OS1::parse_metadata(mdata)), _pf(OS1::get_format(_info))
  {
    _height = _pf.pixels_per_column;
    _width = OS1::n_cols_of_lidar_mode(_info.mode);
    _xyz_lut = OS1::make_xyz_lut(_width, _height, _info.beam_azimuth_angles,
                                 _info.beam_altitude_angles);
    _cloud = std::make_shared<pcl::PointCloud<point_os::PointOS>>(_width,
                                                                  _height);
    _pub = _node->create_publisher<sensor_msgs::msg::PointCloud2>("points",
                                                                  qos);

    _batch_and_publish =
            OS1::batch_to_iter<pcl::PointCloud<point_os::PointOS>::iterator>(
                    _xyz_lut, _width, _height, {}, &point_os::PointOS::make,
                    [&](uint64_t scan_ts) mutable {
                      if (_pub->get_subscription_count() > 0 &&
                          _pub->is_activated()) {
                        auto msg_ptr =
                                std::make_unique<sensor_msgs::msg::PointCloud2>(
                                        std::move(ros2_ouster::toMsg(
                                                *_cloud,
                                                std::chrono::nanoseconds(
                                                        scan_ts),
                                                _frame)));
                        _pub->publish(std::move(msg_ptr));
                      }
                    });
  }

  /**
   * @brief A destructor clearing memory allocated
   */
  ~PointcloudProcessor() { _pub.reset(); }

  /**
   * @brief Process method to create pointcloud
   * @param data the packet data
   */
  bool process(uint8_t *data, uint64_t override_ts) override
  {
    OSCloudIt it = _cloud->begin();
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
  std::function<void(const uint8_t *, OSCloudIt, uint64_t)> _batch_and_publish;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud2>::SharedPtr
          _pub;
  std::shared_ptr<OSCloud> _cloud;
  rclcpp_lifecycle::LifecycleNode::SharedPtr _node;
  std::vector<double> _xyz_lut;
  std::string _frame;
  uint32_t _height;
  uint32_t _width;
  //TODO(OS1-data): abstract this away to make ti independent of the OS1 structs?
  OS1::sensor_info _info;
  OS1::packet_format _pf;
};

}// namespace OS1

#endif// ROS2_OUSTER__OS1__PROCESSORS__POINTCLOUD_PROCESSOR_HPP_
