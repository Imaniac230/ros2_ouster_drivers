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

#ifndef ROS2_OUSTER__OS1__OS1_UTIL_HPP_
#define ROS2_OUSTER__OS1__OS1_UTIL_HPP_


#include <algorithm>
#include <functional>
#include <iterator>
#include <optional>
#include <vector>

#include "ros2_ouster/OS1/OS1_lidar_scan.hpp"
#include <ros2_ouster/OS1/OS1_types.hpp>

namespace OS1
{
/**
 * Generate a table of pixel offsets based on the scan width (512, 1024, or 2048
 * columns). These can be used to create a de-staggered range image where each
 * column of pixels has the same azimuth angle from raw sensor output.
 * The offset is the starting column of each row in the de-staggered lidar scan.
 * @param W number of columns in the lidar scan. One of 512, 1024, or 2048.
 * @return vector of H pixel offsets
 */
inline std::vector<int> get_px_offset(int lidar_mode)
{
  auto repeat = [](int n, const std::vector<int> &v) {
    std::vector<int> res{};
    for (int i = 0; i < n; i++) { res.insert(res.end(), v.begin(), v.end()); }
    return res;
  };

  switch (lidar_mode) {
    case 512:
      return repeat(16, {0, 3, 6, 9});
    case 1024:
      return repeat(16, {0, 6, 12, 18});
    case 2048:
      return repeat(16, {0, 12, 24, 36});
    default:
      return std::vector<int>{64, 0};
  }
}

/**
 * Make a function that batches a single scan (revolution) of data to a
 * random-access iterator. The callback f() is invoked with the timestamp of the
 * first column in the scan before adding data from a new scan. Timestamps for
 * each column are ns relative to the scan timestamp. XYZ coordinates in meters
 * are computed using the provided lookup table.
 *
 * The value type is assumed to be constructed from 9 values: x, y, z,
 * (padding), intensity, ts, reflectivity, noise, range (in mm) and
 * default-constructible. It should be compatible with PointOS1 in the
 * ouster_ros package.
 *
 * @param xyz_lut a lookup table generated from make_xyz_lut, above
 * @param W number of columns in the lidar scan. One of 512, 1024, or 2048.
 * @param H number of rows in the lidar scan. 64 for the OS1 family of sensors.
 * @param empty value to insert for mossing data
 * @param c function to construct a value from x, y, z (m), intensity, ts, reflectivity,
 * ring, column, noise, range (mm). Needed to use with Eigen datatypes.
 * @param f callback invoked when batching a scan is done.
 * @return a function taking a lidar packet buffer and random-access iterator to
 * which data is added for every point in the scan.
 */
template<typename iterator_type, typename F, typename C>
std::function<void(const uint8_t *, iterator_type it, uint64_t,
                   std::optional<std::shared_ptr<OS1::ScanBatcher>>,
                   std::optional<std::shared_ptr<OS1::LidarScan>>)>
batch_to_iter(int W, int H, const OS1::packet_format &pf,
              const typename iterator_type::value_type &empty, C &&c, F &&f)
{
  int next_m_id{W};
  int32_t cur_f_id{-1};

  int64_t scan_ts{-1L};

  return [=](const uint8_t *packet_buf, iterator_type it, uint64_t override_ts,
             std::optional<std::shared_ptr<OS1::ScanBatcher>> scan_batcher =
                     std::nullopt,
             std::optional<std::shared_ptr<OS1::LidarScan>> lidar_scan =
                     std::nullopt) mutable {
    bool should_publish = false;
    if (scan_batcher.has_value() && lidar_scan.has_value()) {
      should_publish = (*scan_batcher.value())(packet_buf, lidar_scan.value());
    }

    //FIXME(processor-publish): this will always call only the pointcloud publishing callback anyway,
    //  we must inline the lidar_scan to pointcloud conversion either way
    if (should_publish) f(override_ts);
  };
}

struct version {
  uint16_t major;///< Major version number
  uint16_t minor;///< Minor version number
  uint16_t patch;///< Patch(or revision) version number
};
const version invalid_version = {0, 0, 0};

/** \defgroup ouster_client_version_operators Ouster Client version.h Operators
 * @{
 */
/**
 * Equality operation for version structs.
 *
 * @param[in] u The first version to compare.
 * @param[in] v The second version to compare.
 *
 * @return If the versions are the same.
 */
inline bool operator==(const version &u, const version &v)
{
  return u.major == v.major && u.minor == v.minor && u.patch == v.patch;
}

/**
 * Less than operation for version structs.
 *
 * @param[in] u The first version to compare.
 * @param[in] v The second version to compare.
 *
 * @return If the first version is less than the second version.
 */
inline bool operator<(const version &u, const version &v)
{
  return (u.major < v.major) || (u.major == v.major && u.minor < v.minor) ||
         (u.major == v.major && u.minor == v.minor && u.patch < v.patch);
}

/**
 * Less than or equal to operation for version structs.
 *
 * @param[in] u The first version to compare.
 * @param[in] v The second version to compare.
 *
 * @return If the first version is less than or equal to the second version.
 */
inline bool operator<=(const version &u, const version &v)
{
  return u < v || u == v;
}

/**
 * In-equality operation for version structs.
 *
 * @param[in] u The first version to compare.
 * @param[in] v The second version to compare.
 *
 * @return If the versions are not the same.
 */
inline bool operator!=(const version &u, const version &v) { return !(u == v); }

/**
 * Greater than or equal to operation for version structs.
 *
 * @param[in] u The first version to compare.
 * @param[in] v The second version to compare.
 *
 * @return If the first version is greater than or equal to the second version.
 */
inline bool operator>=(const version &u, const version &v) { return !(u < v); }

/**
 * Greater than operation for version structs.
 *
 * @param[in] u The first version to compare.
 * @param[in] v The second version to compare.
 *
 * @return If the first version is greater than the second version.
 */
inline bool operator>(const version &u, const version &v) { return !(u <= v); }
/** @}*/

/**
 * Get string representation of a version.
 *
 * @param[in] v version.
 *
 * @return string representation of the version.
 */
inline std::string to_string(const version &v)
{
  if (v == invalid_version) { return "UNKNOWN"; }

  std::stringstream ss{};
  ss << "v" << v.major << "." << v.minor << "." << v.patch;
  return ss.str();
}

/**
 * Get version from string.
 *
 * @param[in] s string.
 *
 * @return version corresponding to the string, or invalid_version on error.
 */
inline version version_of_string(const std::string &s)
{
  std::istringstream is{s};
  char c1, c2, c3;
  version v{};

  is >> c1 >> v.major >> c2 >> v.minor >> c3 >> v.patch;

  if (is && c1 == 'v' && c2 == '.' && c3 == '.') return v;
  else
    return invalid_version;
}

/*
 * From ouster_ros cartesian.hpp
 * */
template<typename T>
using PointsT = Eigen::Array<T, -1, 3>;
using PointsD = PointsT<double>;
using PointsF = PointsT<float>;

/**
 * Converts a staggered range image to Cartesian points.
 *
 * @param[in, out] points The resulting point cloud, should be pre-allocated and
 * have the same dimensions as the direction array.
 * @param[in] range a range image in the same format as the RANGE field of a
 * LidarScan.
 * @param[in] direction the direction of an xyz lut.
 * @param[in] offset the offset of an xyz lut.
 *
 * @return Cartesian points where ith row is a 3D point which corresponds
 *         to ith pixel in LidarScan where i = row * w + col.
 */
template<typename T>
void cartesianT(PointsT<T> &points,
                const Eigen::Ref<const img_t<uint32_t>> &range,
                const PointsT<T> &direction, const PointsT<T> &offset)
{
  assert(points.rows() == direction.rows() &&
         "points & direction row count mismatch");
  assert(points.rows() == offset.rows() &&
         "points & offset row count mismatch");
  assert(points.rows() == range.size() &&
         "points and range image size mismatch");

  const auto pts = points.data();
  const auto *const rng = range.data();
  const auto *const dir = direction.data();
  const auto *const ofs = offset.data();

  const auto N = range.size();
  const auto col_x = 0 * N;// 1st column of points (x)
  const auto col_y = 1 * N;// 2nd column of points (y)
  const auto col_z = 2 * N;// 3rd column of points (z)

  for (auto i = 0; i < N; ++i) {
    const auto r = rng[i];
    const auto idx_x = col_x + i;
    const auto idx_y = col_y + i;
    const auto idx_z = col_z + i;
    if (r == 0) { pts[idx_x] = pts[idx_y] = pts[idx_z] = static_cast<T>(0.0); }
    else {
      pts[idx_x] = r * dir[idx_x] + ofs[idx_x];
      pts[idx_y] = r * dir[idx_y] + ofs[idx_y];
      pts[idx_z] = r * dir[idx_z] + ofs[idx_z];
    }
  }
}

}// namespace OS1

#endif// ROS2_OUSTER__OS1__OS1_UTIL_HPP_
