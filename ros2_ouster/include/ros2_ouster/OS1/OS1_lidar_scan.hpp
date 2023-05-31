/**
 * Copyright (c) 2018, Ouster, Inc.  All rights reserved.  @file
 * @brief Holds lidar data by field in row-major order
 */

#ifndef ROS2_OUSTER__OS1__OS1_LIDAR_SCAN_HPP_
#define ROS2_OUSTER__OS1__OS1_LIDAR_SCAN_HPP_

#include <Eigen/Core>
#include <chrono>
#include <cstddef>
#include <map>
#include <stdexcept>
#include <type_traits>
#include <utility>
#include <vector>

#include "ros2_ouster/OS1/OS1_types.hpp"

namespace OS1
{

template<typename T>
struct FieldTag;

template<>
struct FieldTag<uint8_t> {
  static constexpr ChanFieldType tag = ChanFieldType::UINT8;
};

template<>
struct FieldTag<uint16_t> {
  static constexpr ChanFieldType tag = ChanFieldType::UINT16;
};

template<>
struct FieldTag<uint32_t> {
  static constexpr ChanFieldType tag = ChanFieldType::UINT32;
};

template<>
struct FieldTag<uint64_t> {
  static constexpr ChanFieldType tag = ChanFieldType::UINT64;
};

/*
 * Tagged union for LidarScan fields
 */
struct FieldSlot {
  ChanFieldType tag;
  union
  {
    img_t<uint8_t> f8;
    img_t<uint16_t> f16;
    img_t<uint32_t> f32;
    img_t<uint64_t> f64;
  };

  FieldSlot(ChanFieldType t, size_t w, size_t h) : tag{t}
  {
    switch (t) {
      case ChanFieldType::VOID:
        break;
      case ChanFieldType::UINT8:
        new (&f8) img_t<uint8_t>{h, w};
        f8.setZero();
        break;
      case ChanFieldType::UINT16:
        new (&f16) img_t<uint16_t>{h, w};
        f16.setZero();
        break;
      case ChanFieldType::UINT32:
        new (&f32) img_t<uint32_t>{h, w};
        f32.setZero();
        break;
      case ChanFieldType::UINT64:
        new (&f64) img_t<uint64_t>{h, w};
        f64.setZero();
        break;
    }
  }

  FieldSlot() : FieldSlot{ChanFieldType::VOID, 0, 0} {};

  ~FieldSlot() { clear(); }

  FieldSlot(const FieldSlot &other)
  {
    switch (other.tag) {
      case ChanFieldType::VOID:
        break;
      case ChanFieldType::UINT8:
        new (&f8) img_t<uint8_t>{other.f8};
        break;
      case ChanFieldType::UINT16:
        new (&f16) img_t<uint16_t>{other.f16};
        break;
      case ChanFieldType::UINT32:
        new (&f32) img_t<uint32_t>{other.f32};
        break;
      case ChanFieldType::UINT64:
        new (&f64) img_t<uint64_t>{other.f64};
        break;
    }
    tag = other.tag;
  }

  FieldSlot(FieldSlot &&other) { set_from(other); }

  FieldSlot &operator=(FieldSlot other)
  {
    clear();
    set_from(other);
    return *this;
  }

  template<typename T>
  Eigen::Ref<img_t<T>> get()
  {
    if (tag == FieldTag<T>::tag) return get_unsafe<T>();
    else
      throw std::invalid_argument("Accessed field at wrong type");
  }

  template<typename T>
  Eigen::Ref<const img_t<T>> get() const
  {
    if (tag == FieldTag<T>::tag) return get_unsafe<T>();
    else
      throw std::invalid_argument("Accessed field at wrong type");
  }

  friend bool operator==(const FieldSlot &l, const FieldSlot &r)
  {
    if (l.tag != r.tag) return false;
    switch (l.tag) {
      case ChanFieldType::VOID:
        return true;
      case ChanFieldType::UINT8:
        return (l.f8 == r.f8).all();
      case ChanFieldType::UINT16:
        return (l.f16 == r.f16).all();
      case ChanFieldType::UINT32:
        return (l.f32 == r.f32).all();
      case ChanFieldType::UINT64:
        return (l.f64 == r.f64).all();
      default:
        assert(false);
    }
    // unreachable, appease older gcc
    return false;
  }

  private:
  void set_from(FieldSlot &other)
  {
    switch (other.tag) {
      case ChanFieldType::VOID:
        break;
      case ChanFieldType::UINT8:
        new (&f8) img_t<uint8_t>{std::move(other.f8)};
        break;
      case ChanFieldType::UINT16:
        new (&f16) img_t<uint16_t>{std::move(other.f16)};
        break;
      case ChanFieldType::UINT32:
        new (&f32) img_t<uint32_t>{std::move(other.f32)};
        break;
      case ChanFieldType::UINT64:
        new (&f64) img_t<uint64_t>{std::move(other.f64)};
        break;
    }
    tag = other.tag;
    other.clear();
  }

  void clear()
  {
    switch (tag) {
      case ChanFieldType::VOID:
        break;
      case ChanFieldType::UINT8:
        f8.~img_t<uint8_t>();
        break;
      case ChanFieldType::UINT16:
        f16.~img_t<uint16_t>();
        break;
      case ChanFieldType::UINT32:
        f32.~img_t<uint32_t>();
        break;
      case ChanFieldType::UINT64:
        f64.~img_t<uint64_t>();
        break;
    }
    tag = ChanFieldType::VOID;
  }

  template<typename T>
  Eigen::Ref<img_t<T>> get_unsafe();

  template<typename T>
  Eigen::Ref<const img_t<T>> get_unsafe() const;
};

template<>
inline Eigen::Ref<img_t<uint8_t>> FieldSlot::get_unsafe()
{
  return f8;
}

template<>
inline Eigen::Ref<img_t<uint16_t>> FieldSlot::get_unsafe()
{
  return f16;
}

template<>
inline Eigen::Ref<img_t<uint32_t>> FieldSlot::get_unsafe()
{
  return f32;
}

template<>
inline Eigen::Ref<img_t<uint64_t>> FieldSlot::get_unsafe()
{
  return f64;
}

template<>
inline Eigen::Ref<const img_t<uint8_t>> FieldSlot::get_unsafe() const
{
  return f8;
}

template<>
inline Eigen::Ref<const img_t<uint16_t>> FieldSlot::get_unsafe() const
{
  return f16;
}

template<>
inline Eigen::Ref<const img_t<uint32_t>> FieldSlot::get_unsafe() const
{
  return f32;
}

template<>
inline Eigen::Ref<const img_t<uint64_t>> FieldSlot::get_unsafe() const
{
  return f64;
}

/*
 * Call a generic operation op<T>(f, Args..) with the type parameter T having
 * the correct (dynamic) field type for the LidarScan channel field f
 * Example code for the operation<T>:
 * \code
 * struct print_field_size {
 *   template <typename T>
 *   void operator()(Eigen::Ref<img_t<T>> field) {
 *       std::cout << "Rows: " + field.rows() << std::endl;
 *       std::cout << "Cols: " + field.cols() << std::endl;
 *   }
 * };
 * \endcode
 */
template<typename SCAN, typename OP, typename... Args>
void visit_field(SCAN &&ls, OS1::ChanField f, OP &&op, Args &&...args)
{
  switch (ls.field_type(f)) {
    case OS1::ChanFieldType::UINT8:
      op.template operator()(ls.template field<uint8_t>(f),
                             std::forward<Args>(args)...);
      break;
    case OS1::ChanFieldType::UINT16:
      op.template operator()(ls.template field<uint16_t>(f),
                             std::forward<Args>(args)...);
      break;
    case OS1::ChanFieldType::UINT32:
      op.template operator()(ls.template field<uint32_t>(f),
                             std::forward<Args>(args)...);
      break;
    case OS1::ChanFieldType::UINT64:
      op.template operator()(ls.template field<uint64_t>(f),
                             std::forward<Args>(args)...);
      break;
    default:
      throw std::invalid_argument("Invalid field for LidarScan");
  }
}

/*
 * Call a generic operation op<T>(f, Args...) for each field of the lidar scan
 * with type parameter T having the correct field type
 */
template<typename SCAN, typename OP, typename... Args>
void foreach_field(SCAN &&ls, OP &&op, Args &&...args)
{
  for (const auto &ft: ls)
    visit_field(std::forward<SCAN>(ls), ft.first, std::forward<OP>(op),
                ft.first, std::forward<Args>(args)...);
}

// Read LidarScan field and cast to the destination
struct read_and_cast {
  template<typename T, typename U>
  void operator()(Eigen::Ref<const img_t<T>> src, Eigen::Ref<img_t<U>> dest)
  {
    dest = src.template cast<U>();
  }
  template<typename T, typename U>
  void operator()(Eigen::Ref<img_t<T>> src, Eigen::Ref<img_t<U>> dest)
  {
    dest = src.template cast<U>();
  }
  template<typename T, typename U>
  void operator()(Eigen::Ref<img_t<T>> src, img_t<U> &dest)
  {
    dest = src.template cast<U>();
  }
  template<typename T, typename U>
  void operator()(Eigen::Ref<const img_t<T>> src, img_t<U> &dest)
  {
    dest = src.template cast<U>();
  }
};

/**
 * Alias for the lidar scan field types
 */
using LidarScanFieldTypes =
        std::vector<std::pair<OS1::ChanField, OS1::ChanFieldType>>;

/**
 * Data structure for efficient operations on aggregated lidar data.
 *
 * Stores each field (range, intensity, etc.) contiguously as a H x W block of
 * 4-byte unsigned integers, where H is the number of beams and W is the
 * horizontal resolution (e.g. 512, 1024, 2048).
 *
 * Note: this is the "staggered" representation where each column corresponds
 * to a single measurement in time. Use the destagger() function to create an
 * image where columns correspond to a single azimuth angle.
 */
class LidarScan
{
  public:
  template<typename T>
  using Header = Eigen::Array<T, Eigen::Dynamic, 1>;///< Header typedef

  /** XYZ coordinates with dimensions arranged contiguously in columns. */
  using Points = Eigen::Array<double, Eigen::Dynamic, 3>;

  private:
  Header<uint64_t> timestamp_;
  Header<uint16_t> measurement_id_;
  Header<uint32_t> status_;
  std::map<OS1::ChanField, FieldSlot> fields_;
  LidarScanFieldTypes field_types_;

  LidarScan(size_t w, size_t h, LidarScanFieldTypes field_types);

  public:
  /**
     * Pointer offsets to deal with strides.
     *
     * @warning Members variables: use with caution, some of these will become
     * private.
     */
  std::ptrdiff_t w{0};

  /**
     * Pointer offsets to deal with strides.
     *
     * @warning Members variables: use with caution, some of these will become
     * private.
     */
  std::ptrdiff_t h{0};

  /**
     * Frame status - information from the packet header which corresponds to a
     * frame
     *
     * @warning Member variables: use with caution, some of these will become
     * private.
     */
  uint64_t frame_status{0};

  /**
     * The current frame ID.
     *
     * @warning Members variables: use with caution, some of these will become
     * private.
     */
  int32_t frame_id{-1};

  using FieldIter =
          decltype(field_types_)::const_iterator;///< An STL Iterator of the
                                                 ///< field types

  /** The default constructor creates an invalid 0 x 0 scan. */
  LidarScan();

  /**
     * Initialize a scan with fields configured for the LEGACY udp profile.
     *
     * @param[in] w horizontal resoulution, i.e. the number of measurements per
     * scan.
     * @param[in] h vertical resolution, i.e. the number of channels.
     */
  LidarScan(size_t w, size_t h);

  /**
     * Initialize a scan with the default fields for a particular udp profile.
     *
     * @param[in] w horizontal resoulution, i.e. the number of measurements per
     * scan.
     * @param[in] h vertical resolution, i.e. the number of channels.
     * @param[in] profile udp profile.
     */
  LidarScan(size_t w, size_t h, OS1::UDPProfileLidar profile);

  /**
     * Initialize a scan with a custom set of fields.
     *
     * @tparam Iterator A standard template iterator for the custom fields.
     *
     * @param[in] w horizontal resoulution, i.e. the number of measurements per
     * scan.
     * @param[in] h vertical resolution, i.e. the number of channels.
     * @param[in] begin begin iterator of pairs of channel fields and types.
     * @param[in] end end iterator of pairs of channel fields and types.
     */
  template<typename Iterator>
  LidarScan(size_t w, size_t h, Iterator begin, Iterator end)
      : LidarScan(w, h, {begin, end}){};

  /**
     * Initialize a lidar scan from another lidar scan.
     *
     * @param[in] other The other lidar scan to initialize from.
     */
  LidarScan(const LidarScan &other);

  /** @copydoc LidarScan(const LidarScan& other) */
  LidarScan(LidarScan &&other);

  /**
     * Copy via Move semantic.
     *
     * @param[in] other The lidar scan to copy from.
     */
  LidarScan &operator=(const LidarScan &other);

  /** @copydoc operator=(const LidarScan& other) */
  LidarScan &operator=(LidarScan &&other);

  /**
     * Lidar scan destructor.
     */
  ~LidarScan();

  /**
     * Get frame shot limiting status
     */
  OS1::ShotLimitingStatus shot_limiting() const;

  /**
     * Get frame thermal shutdown status
     */
  OS1::ThermalShutdownStatus thermal_shutdown() const;

  /**
     * Access a lidar data field.
     *
     * @throw std::invalid_argument if T does not match the runtime field type.
     *
     * @tparam T The type parameter T must match the dynamic type of the field.
     * See the constructor documentation for expected field types or query
     * dynamically for generic operations.
     *
     * @param[in] f the field to view.
     *
     * @return a view of the field data.
     */
  template<typename T = uint32_t,
           typename std::enable_if<std::is_unsigned<T>::value, T>::type = 0>
  Eigen::Ref<img_t<T>> field(OS1::ChanField f);

  /** @copydoc field(Field f) */
  template<typename T = uint32_t,
           typename std::enable_if<std::is_unsigned<T>::value, T>::type = 0>
  Eigen::Ref<const img_t<T>> field(OS1::ChanField f) const;

  /**
     * Get the type of the specified field.
     *
     * @param[in] f the field to query.
     *
     * @return the type tag associated with the field.
     */
  OS1::ChanFieldType field_type(OS1::ChanField f) const;

  /** A const forward iterator over field / type pairs. */
  FieldIter begin() const;

  /** @copydoc begin() */
  FieldIter end() const;

  /**
     * Access the measurement timestamp headers.
     *
     * @return a view of timestamp as a w-element vector.
     */
  Eigen::Ref<Header<uint64_t>> timestamp();

  /**
     * @copydoc timestamp()
     */
  Eigen::Ref<const Header<uint64_t>> timestamp() const;

  /**
     * Access the measurement id headers.
     *
     * @return a view of measurement ids as a w-element vector.
     */
  Eigen::Ref<Header<uint16_t>> measurement_id();

  /** @copydoc measurement_id() */
  Eigen::Ref<const Header<uint16_t>> measurement_id() const;

  /**
     * Access the measurement status headers.
     *
     * @return a view of measurement statuses as a w-element vector.
     */
  Eigen::Ref<Header<uint32_t>> status();

  /** @copydoc status() */
  Eigen::Ref<const Header<uint32_t>> status() const;

  /**
     * Assess completeness of scan.
     * @param[in] window The column window to use for validity assessment
     * @return whether all columns within given column window were valid
     */
  bool complete(OS1::ColumnWindow window) const;

  friend bool operator==(const LidarScan &a, const LidarScan &b);
};

// Copy fields from `ls_source` LidarScan to `field_dest` img with casting
// to the img_t<T> type of `field_dest`.
struct copy_and_cast {
  template<typename T>
  void operator()(Eigen::Ref<img_t<T>> field_dest, const LidarScan &ls_source,
                  OS1::ChanField ls_source_field)
  {
    visit_field(ls_source, ls_source_field, read_and_cast(), field_dest);
  }
};

/** \defgroup ouster_client_destagger Ouster Client lidar_scan.h
 * @{
 */
/**
 * Generate a destaggered version of a channel field.
 *
 * In the default staggered representation, each column corresponds to a single
 * timestamp. In the destaggered representation, each column corresponds to a
 * single azimuth angle, compensating for the azimuth offset of each beam.
 *
 * Destaggering is used for visualizing lidar data as an image or for algorithms
 * that exploit the structure of the lidar data, such as beam_uniformity in
 * ouster_viz, or computer vision algorithms.
 *
 * @tparam T the datatype of the channel field.
 *
 * @param[in] img the channel field.
 * @param[in] pixel_shift_by_row offsets, usually queried from the sensor.
 * @param[in] inverse perform the inverse operation.
 *
 * @return destaggered version of the image.
 */
template<typename T>
inline img_t<T> destagger(const Eigen::Ref<const img_t<T>> &img,
                          const std::vector<int> &pixel_shift_by_row,
                          bool inverse = false)
{
  const size_t h = img.rows();
  const size_t w = img.cols();

  if (pixel_shift_by_row.size() != h)
    throw std::invalid_argument{"image height does not match shifts size"};

  img_t<T> destaggered{h, w};
  for (size_t u = 0; u < h; u++) {
    const std::ptrdiff_t offset =
            ((inverse ? -1 : 1) * pixel_shift_by_row[u] + w) % w;

    destaggered.row(u).segment(offset, w - offset) =
            img.row(u).segment(0, w - offset);
    destaggered.row(u).segment(0, offset) =
            img.row(u).segment(w - offset, offset);
  }
  return destaggered;
}

/**
 * Get string representation of lidar scan field types.
 *
 * @param[in] field_types The field types to get the string representation of.
 *
 * @return string representation of the lidar scan field types.
 */
std::string to_string(const LidarScanFieldTypes &field_types);

/**
 * Get the lidar scan field types from a lidar scan
 *
 * @param[in] ls The lidar scan to get the lidar scan field types from.
 *
 * @return The lidar scan field types
 */
LidarScanFieldTypes get_field_types(const LidarScan &ls);

/**
 * Get the lidar scan field types from sensor info
 *
 * @param[in] info The sensor info to get the lidar scan field types from.
 *
 * @return The lidar scan field types
 */
LidarScanFieldTypes get_field_types(const OS1::sensor_info &info);

/**
 * Get string representation of a lidar scan.
 *
 * @param[in] ls The lidar scan to get the string representation of.
 *
 * @return string representation of the lidar scan.
 */
std::string to_string(const LidarScan &ls);

/** \defgroup ouster_client_lidar_scan_operators Ouster Client lidar_scan.h
 * Operators
 * @{
 */

/**
 * Equality for scans.
 *
 * @param[in] a The first scan to compare.
 * @param[in] b The second scan to compare.
 *
 * @return if a == b.
 */
bool operator==(const LidarScan &a, const LidarScan &b);

/**
 * NOT Equality for scans.
 *
 * @param[in] a The first scan to compare.
 * @param[in] b The second scan to compare.
 *
 * @return if a != b.
 */
inline bool operator!=(const LidarScan &a, const LidarScan &b)
{
  return !(a == b);
}
/** @}*/

/** Lookup table of beam directions and offsets. */
struct XYZLut {
  LidarScan::Points direction;///< Lookup table of beam directions
  LidarScan::Points offset;   ///< Lookup table of beam offsets
};

/**
 * Generate a set of lookup tables useful for computing Cartesian coordinates
 * from ranges.
 *
 * The lookup tables are:
 * - direction: a matrix of unit vectors pointing radially outwards.
 * - offset: a matrix of offsets dependent on beam origin distance from lidar
 *           origin.
 *
 * Each table is an n x 3 array of doubles stored in column-major order where
 * each row corresponds to the nth point in a lidar scan, with 0 <= n < h*w.
 *
 * Projections to XYZ made with this XYZLut will be in the coordinate frame
 * defined by transform*beam_to_lidar_transform.
 *
 * @param[in] w number of columns in the lidar scan. e.g. 512, 1024, or 2048.
 * @param[in] h number of rows in the lidar scan.
 * @param[in] range_unit the unit, in meters, of the range,  e.g.
 * sensor::range_unit.
 * @param[in] beam_to_lidar_transform transform between beams and
 * lidar origin. Translation portion is in millimeters.
 * @param[in] transform additional transformation to apply to resulting points.
 * @param[in] azimuth_angles_deg azimuth offsets in degrees for each of h beams.
 * @param[in] altitude_angles_deg altitude in degrees for each of h beams.
 *
 * @return xyz direction and offset vectors for each point in the lidar scan.
 */
XYZLut make_xyz_lut(size_t w, size_t h, double range_unit,
                    const mat4d &beam_to_lidar_transform,
                    const mat4d &transform,
                    const std::vector<double> &azimuth_angles_deg,
                    const std::vector<double> &altitude_angles_deg);

/**
 * Convenient overload that uses parameters from the supplied sensor_info.
 * Projections to XYZ made with this XYZLut will be in the sensor coordinate
 * frame defined in the sensor documentation.
 *
 * @param[in] sensor metadata returned from the client.
 *
 * @return xyz direction and offset vectors for each point in the lidar scan.
 */
inline XYZLut make_xyz_lut(const OS1::sensor_info &sensor)
{
  return make_xyz_lut(sensor.format.columns_per_frame,
                      sensor.format.pixels_per_column, OS1::range_unit,
                      sensor.beam_to_lidar_transform,
                      sensor.lidar_to_sensor_transform,
                      sensor.beam_azimuth_angles, sensor.beam_altitude_angles);
}

/** \defgroup ouster_client_lidar_scan_cartesian Ouster Client lidar_scan.h
 * XYZLut related items.
 * @{
 */
/**
 * Convert LidarScan to Cartesian points.
 *
 * @param[in] scan a LidarScan.
 * @param[in] lut lookup tables generated by make_xyz_lut.
 *
 * @return Cartesian points where ith row is a 3D point which corresponds
 *         to ith pixel in LidarScan where i = row * w + col.
 */
LidarScan::Points cartesian(const LidarScan &scan, const XYZLut &lut);

/**
 * Convert a staggered range image to Cartesian points.
 *
 * @param[in] range a range image in the same format as the RANGE field of a
 * LidarScan.
 * @param[in] lut lookup tables generated by make_xyz_lut.
 *
 * @return Cartesian points where ith row is a 3D point which corresponds
 *         to ith pixel in LidarScan where i = row * w + col.
 */
LidarScan::Points cartesian(const Eigen::Ref<const img_t<uint32_t>> &range,
                            const XYZLut &lut);
/** @}*/

/**
 * Generate a staggered version of a channel field.
 *
 * @tparam T the datatype of the channel field.
 *
 * @param[in] img the channel field.
 * @param[in] pixel_shift_by_row offsets, usually queried from the sensor.
 *
 * @return staggered version of the image.
 */
template<typename T>
inline img_t<T> stagger(const Eigen::Ref<const img_t<T>> &img,
                        const std::vector<int> &pixel_shift_by_row)
{
  return destagger(img, pixel_shift_by_row, true);
}
/** @}*/
/**
 * Parse lidar packets into a LidarScan.
 *
 * Make a function that batches a single scan (revolution) of data to a
 * LidarScan.
 */
class ScanBatcher
{
  std::ptrdiff_t w;
  std::ptrdiff_t h;
  uint16_t next_valid_m_id;
  uint16_t next_headers_m_id;
  std::vector<uint8_t> cache;
  bool cached_packet = false;

  uint16_t packet_count = 0;

  public:
  OS1::packet_format pf;///< The packet format object used for decoding

  /**
     * Create a batcher given information about the scan and packet format.
     *
     * @param[in] w number of columns in the lidar scan. One of 512, 1024, or
     * 2048.
     * @param[in] pf expected format of the incoming packets used for parsing.
     */
  ScanBatcher(size_t w, const OS1::packet_format &pf);

  /**
     * Create a batcher given information about the scan and packet format.
     *
     * @param[in] info sensor metadata returned from the client.
     */
  ScanBatcher(const OS1::sensor_info &info);

  /**
     * Add a packet to the scan.
     *
     * @param[in] packet_buf the lidar packet.
     * @param[in] ls lidar scan to populate.
     *
     * @return true when the provided lidar scan is ready to use.
     */
  bool operator()(const uint8_t *packet_buf,
                  const std::shared_ptr<LidarScan> &ls);
};

/**
 * Imu Data
 */
struct Imu {
  union
  {
    std::array<double, 3> angular_vel;
    struct {
      double wx, wy, wz;
    };
  };
  union
  {
    std::array<double, 3> linear_accel;
    struct {
      double ax, ay, az;
    };
  };
  union
  {
    std::array<uint64_t, 3> ts;
    struct {
      uint64_t sys_ts, accel_ts, gyro_ts;
    };
  };
};

/** Equality for Imu */
inline bool operator==(const Imu &a, const Imu &b)
{
  return a.angular_vel == b.angular_vel && a.linear_accel == b.linear_accel &&
         a.ts == b.ts;
};

/** Not Equality for Imu */
inline bool operator!=(const Imu &a, const Imu &b) { return !(a == b); };

std::string to_string(const Imu &imu);

/// Reconstructs buf with UDP imu_packet to osf::Imu object
void packet_to_imu(const uint8_t *buf, const OS1::packet_format &pf, Imu &imu);

}// namespace OS1

#endif// ROS2_OUSTER__OS1__OS1_LIDAR_SCAN_HPP_