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
#include <vector>

#include <ros2_ouster/OS1/OS1_types.hpp>

namespace OS1
{
/**
 * Generate a matrix of unit vectors pointing radially outwards, useful for
 * efficiently computing cartesian coordinates from ranges.  The result is a n x
 * 3 array of doubles stored in row-major order where each row is the unit
 * vector corresponding to the nth point in a lidar scan, with 0 <= n < H*W. The
 * index into the lidar scan of a point can be obtained by H * j + i (where i is
 * the index to nth_px, and j is the measurement_id of the column, when reading
 * from a packet).
 * @param W number of columns in the lidar scan. One of 512, 1024, or 2048.
 * @param H number of rows in the lidar scan. 64 for the OS1 family of sensors.
 * @param beam_azimuth_angles azimuth offsets in degrees for each of H beams
 * @param beam_altitude_angles altitude in degrees for each of H beams
 * @return xyz direction unit vectors for each point in the lidar scan
 */
inline std::vector<double>
make_xyz_lut(int W, int H, const std::vector<double> &azimuth_angles,
             const std::vector<double> &altitude_angles)
{
  const int n = W * H;
  std::vector<double> xyz = std::vector<double>(3 * n, 0);

  for (int icol = 0; icol < W; icol++) {
    double h_angle_0 = 2.0 * M_PI * icol / W;
    for (int ipx = 0; ipx < H; ipx++) {
      int ind = 3 * (icol * H + ipx);
      double h_angle = (azimuth_angles.at(ipx) * 2 * M_PI / 360.0) + h_angle_0;

      xyz[ind + 0] = std::cos(altitude_angles[ipx] * 2 * M_PI / 360.0) *
                     std::cos(h_angle);
      xyz[ind + 1] = -std::cos(altitude_angles[ipx] * 2 * M_PI / 360.0) *
                     std::sin(h_angle);
      xyz[ind + 2] = std::sin(altitude_angles[ipx] * 2 * M_PI / 360.0);
    }
  }
  return xyz;
}
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
std::function<void(const uint8_t *, iterator_type it, uint64_t)>
batch_to_iter(const std::vector<double> &xyz_lut, int W, int H,
              const OS1::packet_format &pf,
              const typename iterator_type::value_type &empty, C &&c, F &&f)
{
  int next_m_id{W};
  int32_t cur_f_id{-1};

  int64_t scan_ts{-1L};

  return [=](const uint8_t *packet_buf, iterator_type it,
             uint64_t override_ts) mutable {
    for (int icol = 0; icol < pf.columns_per_packet; icol++) {
      const uint8_t *col_buf = pf.nth_col(icol, packet_buf);
      const uint16_t m_id = pf.col_measurement_id(col_buf);
      const uint16_t f_id = pf.frame_id(col_buf);
      const uint64_t ts = pf.col_timestamp(col_buf);
      const bool valid = (pf.col_status(col_buf) & 0x01);

      // drop invalid / out-of-bounds data in case of misconfiguration
      if (!valid || m_id >= W || f_id + 1 == cur_f_id) { continue; }

      if (f_id != cur_f_id) {
        // if not initializing with first packet
        if (scan_ts != -1) {
          // zero out remaining missing columns
          std::fill(it + (H * next_m_id), it + (H * W), empty);
          f(override_ts == 0 ? scan_ts : override_ts);
        }

        // start new frame
        scan_ts = ts;
        next_m_id = 0;
        cur_f_id = f_id;
      }

      // zero out missing columns if we jumped forward
      if (m_id >= next_m_id) {
        std::fill(it + (H * next_m_id), it + (H * m_id), empty);
        next_m_id = m_id + 1;
      }

      // index of the first point in current packet
      const int idx = H * m_id;

      for (uint8_t ipx = 0; ipx < H; ipx++) {
        const uint8_t *px_buf = pf.nth_px(ipx, col_buf);
        uint32_t r = pf.px_range(px_buf);
        int ind = 3 * (idx + ipx);

        // x, y, z(m), intensity, ts, reflectivity, ring, column,
        // noise, range (mm)
        it[idx + ipx] =
                c(r * 0.001f * xyz_lut[ind + 0], r * 0.001f * xyz_lut[ind + 1],
                  r * 0.001f * xyz_lut[ind + 2], pf.px_signal(px_buf),
                  ts - scan_ts, pf.px_reflectivity(px_buf), ipx, m_id,
                  pf.px_ambient(px_buf), r);
      }
    }
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

}// namespace OS1

#endif// ROS2_OUSTER__OS1__OS1_UTIL_HPP_
