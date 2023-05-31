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
#include "ros2_ouster/OS1/OS1_lidar_scan.hpp"
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
      : DataProcessorInterface(mdata), _node(node), _frame(frame)
  {
    _height = _info.format.pixels_per_column;
    _width = _info.format.columns_per_frame;
    _cloud = std::make_shared<OSCloud>(_width, _height);
    //FIXME(publishers): we're not yet handling dual data mode publishing
    _pub = _node->create_publisher<sensor_msgs::msg::PointCloud2>("points",
                                                                  qos);

    const auto xyz_lut = OS1::make_xyz_lut(_info);
    _lut_direction = xyz_lut.direction.cast<float>();
    _lut_offset = xyz_lut.offset.cast<float>();
    _points = OS1::PointsF(_lut_direction.rows(), _lut_offset.cols());
    _scan_batcher = std::make_shared<OS1::ScanBatcher>(_info);
    _lidar_scan = std::make_shared<OS1::LidarScan>(
            _width, _height, _info.format.udp_profile_lidar);
    compute_scan_ts = [this](const auto &ts_v) {
      return compute_scan_ts_0(ts_v);
    };

    _batch_and_publish = OS1::batch_to_iter<OSCloudIt>(
            _width, _height, _pf, {}, &point_os::PointOS::make,
            [&](uint64_t scan_ts) mutable {
              if (_pub->get_subscription_count() > 0 && _pub->is_activated()) {
                const auto used_scan_ts =
                        (scan_ts == 0)
                                ? compute_scan_ts(_lidar_scan->timestamp())
                                : scan_ts;
                scan_to_cloud_f(_points, _lut_direction, _lut_offset,
                                used_scan_ts, *_lidar_scan, _cloud, 0);
                auto msg_ptr = std::make_unique<sensor_msgs::msg::PointCloud2>(
                        std::move(ros2_ouster::toMsg(
                                *_cloud, std::chrono::nanoseconds(used_scan_ts),
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
    _batch_and_publish(data, it, override_ts, _scan_batcher, _lidar_scan);
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
  //TODO(helpers): all of these private methods came from ouster_ros os_ros.cpp, decide where to actually put them
  void scan_to_cloud_f(OS1::PointsF &points, const OS1::PointsF &lut_direction,
                       const OS1::PointsF &lut_offset, uint64_t scan_ts,
                       const OS1::LidarScan &ls,
                       const std::shared_ptr<OSCloud> &cloud, int return_index)
  {
    bool second = (return_index == 1);

    assert(cloud->width == static_cast<std::uint32_t>(ls.w) &&
           cloud->height == static_cast<std::uint32_t>(ls.h) &&
           "point cloud and lidar scan size mismatch");

    // across supported lidar profiles range is always 32-bit
    auto range_channel_field =
            second ? OS1::ChanField::RANGE2 : OS1::ChanField::RANGE;
    OS1::img_t<uint32_t> range = ls.field<uint32_t>(range_channel_field);

    OS1::img_t<uint16_t> reflectivity = get_or_fill_zero<uint16_t>(
            suitable_return(OS1::ChanField::REFLECTIVITY, second), ls);

    OS1::img_t<uint32_t> signal = get_or_fill_zero<uint32_t>(
            suitable_return(OS1::ChanField::SIGNAL, second), ls);

    OS1::img_t<uint16_t> near_ir = get_or_fill_zero<uint16_t>(
            suitable_return(OS1::ChanField::NEAR_IR, second), ls);

    OS1::cartesianT(points, range, lut_direction, lut_offset);

    copy_scan_to_cloud(cloud, ls, scan_ts, points, range, reflectivity, near_ir,
                       signal);
  }

  template<typename PointT, typename RangeT, typename ReflectivityT,
           typename NearIrT, typename SignalT>
  void copy_scan_to_cloud(const std::shared_ptr<OSCloud> &cloud,
                          const OS1::LidarScan &ls, uint64_t scan_ts,
                          const PointT &points, const OS1::img_t<RangeT> &range,
                          const OS1::img_t<ReflectivityT> &reflectivity,
                          const OS1::img_t<NearIrT> &near_ir,
                          const OS1::img_t<SignalT> &signal)
  {
    auto timestamp = ls.timestamp();

    const auto rg = range.data();
    const auto rf = reflectivity.data();
    const auto nr = near_ir.data();
    const auto sg = signal.data();

    for (auto u = 0; u < ls.h; u++) {
      for (auto v = 0; v < ls.w; v++) {
        const auto col_ts = timestamp[v];
        const auto ts = col_ts > scan_ts ? col_ts - scan_ts : 0UL;
        const auto idx = u * ls.w + v;
        const auto xyz = points.row(idx);
        cloud->points[idx] = point_os::PointOS{
                {static_cast<float>(xyz(0)), static_cast<float>(xyz(1)),
                 static_cast<float>(xyz(2)), 1.0f},
                static_cast<float>(sg[idx]),
                static_cast<uint32_t>(ts),
                static_cast<uint16_t>(rf[idx]),
                static_cast<uint16_t>(u),
                static_cast<uint16_t>(nr[idx]),
                static_cast<uint32_t>(rg[idx]),
        };
      }
    }
  }

  static OS1::ChanField suitable_return(OS1::ChanField input_field, bool second)
  {
    switch (input_field) {
      case OS1::ChanField::RANGE:
      case OS1::ChanField::RANGE2:
        return second ? OS1::ChanField::RANGE2 : OS1::ChanField::RANGE;
      case OS1::ChanField::SIGNAL:
      case OS1::ChanField::SIGNAL2:
        return second ? OS1::ChanField::SIGNAL2 : OS1::ChanField::SIGNAL;
      case OS1::ChanField::REFLECTIVITY:
      case OS1::ChanField::REFLECTIVITY2:
        return second ? OS1::ChanField::REFLECTIVITY2
                      : OS1::ChanField::REFLECTIVITY;
      case OS1::ChanField::NEAR_IR:
        return OS1::ChanField::NEAR_IR;
      default:
        throw std::runtime_error("Unreachable");
    }
  }

  template<typename T>
  inline OS1::img_t<T> get_or_fill_zero(OS1::ChanField f,
                                        const OS1::LidarScan &ls)
  {
    if (!ls.field_type(f)) { return OS1::img_t<T>::Zero(ls.h, ls.w); }

    OS1::img_t<T> result{ls.h, ls.w};
    OS1::visit_field(ls, f, read_and_cast(), result);
    return result;
  }

  /*
   * from ouster_ros os_cloud_node.cpp
   * */
  template<typename T, typename UnaryPredicate>
  int find_if_reverse(const Eigen::Array<T, -1, 1> &array,
                      UnaryPredicate predicate)
  {
    auto p = array.data() + array.size() - 1;
    do {
      if (predicate(*p)) return p - array.data();
    } while (p-- != array.data());
    return -1;
  }

  uint64_t linear_interpolate(int x0, uint64_t y0, int x1, uint64_t y1, int x)
  {
    uint64_t min_v, max_v;
    double sign;
    if (y1 > y0) {
      min_v = y0;
      max_v = y1;
      sign = +1;
    }
    else {
      min_v = y1;
      max_v = y0;
      sign = -1;
    }
    return y0 + (x - x0) * sign * (max_v - min_v) / (x1 - x0);
  }

  template<typename T>
  uint64_t ulround(T value)
  {
    T rounded_value = std::round(value);
    if (rounded_value < 0) return 0ULL;
    if (rounded_value > ULLONG_MAX) return ULLONG_MAX;
    return static_cast<uint64_t>(rounded_value);
  }

  uint64_t impute_value(int last_scan_last_nonzero_idx,
                        uint64_t last_scan_last_nonzero_value,
                        int curr_scan_first_nonzero_idx,
                        uint64_t curr_scan_first_nonzero_value, int scan_width)
  {
    assert(scan_width + curr_scan_first_nonzero_idx >
           last_scan_last_nonzero_idx);
    double interpolated_value = linear_interpolate(
            last_scan_last_nonzero_idx, last_scan_last_nonzero_value,
            scan_width + curr_scan_first_nonzero_idx,
            curr_scan_first_nonzero_value, scan_width);
    return ulround(interpolated_value);
  }

  uint64_t extrapolate_value(int curr_scan_first_nonzero_idx,
                             uint64_t curr_scan_first_nonzero_value)
  {
    double extrapolated_value =
            curr_scan_first_nonzero_value -
            scan_col_ts_spacing_ns * curr_scan_first_nonzero_idx;
    return ulround(extrapolated_value);
  }

  // compute_scan_ts_0 for first scan
  uint64_t compute_scan_ts_0(const OS1::LidarScan::Header<uint64_t> &ts_v)
  {
    auto idx = std::find_if(ts_v.data(), ts_v.data() + ts_v.size(),
                            [](uint64_t h) { return h != 0; });
    assert(idx != ts_v.data() + ts_v.size());// should never happen
    int curr_scan_first_nonzero_idx = idx - ts_v.data();
    uint64_t curr_scan_first_nonzero_value = *idx;

    uint64_t scan_ns =
            curr_scan_first_nonzero_idx == 0
                    ? curr_scan_first_nonzero_value
                    : extrapolate_value(curr_scan_first_nonzero_idx,
                                        curr_scan_first_nonzero_value);

    last_scan_last_nonzero_idx =
            find_if_reverse(ts_v, [](uint64_t h) { return h != 0; });
    assert(last_scan_last_nonzero_idx >= 0);// should never happen
    last_scan_last_nonzero_value = ts_v(last_scan_last_nonzero_idx);
    compute_scan_ts = [this](const auto &ts_v) {
      return compute_scan_ts_n(ts_v);
    };
    return scan_ns;
  }

  // compute_scan_ts_n applied to all subsequent scans except first one
  uint64_t compute_scan_ts_n(const OS1::LidarScan::Header<uint64_t> &ts_v)
  {
    auto idx = std::find_if(ts_v.data(), ts_v.data() + ts_v.size(),
                            [](uint64_t h) { return h != 0; });
    assert(idx != ts_v.data() + ts_v.size());// should never happen
    int curr_scan_first_nonzero_idx = idx - ts_v.data();
    uint64_t curr_scan_first_nonzero_value = *idx;

    uint64_t scan_ns = curr_scan_first_nonzero_idx == 0
                               ? curr_scan_first_nonzero_value
                               : impute_value(last_scan_last_nonzero_idx,
                                              last_scan_last_nonzero_value,
                                              curr_scan_first_nonzero_idx,
                                              curr_scan_first_nonzero_value,
                                              static_cast<int>(ts_v.size()));

    last_scan_last_nonzero_idx =
            find_if_reverse(ts_v, [](uint64_t h) { return h != 0; });
    assert(last_scan_last_nonzero_idx >= 0);// should never happen
    last_scan_last_nonzero_value = ts_v(last_scan_last_nonzero_idx);
    return scan_ns;
  }

  std::function<void(const uint8_t *, OSCloudIt, uint64_t,
                     std::optional<std::shared_ptr<OS1::ScanBatcher>>,
                     std::optional<std::shared_ptr<OS1::LidarScan>>)>
          _batch_and_publish;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud2>::SharedPtr
          _pub;
  std::shared_ptr<OSCloud> _cloud;
  rclcpp_lifecycle::LifecycleNode::SharedPtr _node;
  std::string _frame;
  uint32_t _height;
  uint32_t _width;

  OS1::PointsF _lut_direction;
  OS1::PointsF _lut_offset;
  OS1::PointsF _points;
  std::shared_ptr<OS1::LidarScan> _lidar_scan;
  std::shared_ptr<OS1::ScanBatcher> _scan_batcher;
  int last_scan_last_nonzero_idx = -1;
  uint64_t last_scan_last_nonzero_value = 0;
  std::function<uint64_t(const OS1::LidarScan::Header<uint64_t> &)>
          compute_scan_ts;
  double scan_col_ts_spacing_ns;// interval or spacing between columns of a
                                // scan
};

}// namespace OS1

#endif// ROS2_OUSTER__OS1__PROCESSORS__POINTCLOUD_PROCESSOR_HPP_
