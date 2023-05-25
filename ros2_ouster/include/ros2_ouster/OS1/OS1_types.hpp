/**
 * Copyright (c) 2018, Ouster, Inc.
 * All rights reserved.
 *
 * @file
 * @brief Ouster client datatypes and constants
 */

#ifndef ROS2_OUSTER__OS1__OS1_TYPES_HPP_
#define ROS2_OUSTER__OS1__OS1_TYPES_HPP_

#include <Eigen/Core>
#include <array>
#include <cstdint>
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

#include "json/json.h"

namespace OS1
{

/**
 * For image operations.
 *
 * @tparam T The data type for the array.
 */
template<typename T>
using img_t = Eigen::Array<T, -1, -1, Eigen::RowMajor>;

/** Used for transformations. */
using mat4d = Eigen::Matrix<double, 4, 4, Eigen::DontAlign>;

/** Unit of range from sensor packet, in meters. */
static constexpr double range_unit = 0.001;

/** Design values for altitude and azimuth offset angles for gen1 sensors. */
static const std::vector<double> gen1_altitude_angles = {
        16.611,  16.084,  15.557,  15.029,  14.502,  13.975,  13.447,  12.920,
        12.393,  11.865,  11.338,  10.811,  10.283,  9.756,   9.229,   8.701,
        8.174,   7.646,   7.119,   6.592,   6.064,   5.537,   5.010,   4.482,
        3.955,   3.428,   2.900,   2.373,   1.846,   1.318,   0.791,   0.264,
        -0.264,  -0.791,  -1.318,  -1.846,  -2.373,  -2.900,  -3.428,  -3.955,
        -4.482,  -5.010,  -5.537,  -6.064,  -6.592,  -7.119,  -7.646,  -8.174,
        -8.701,  -9.229,  -9.756,  -10.283, -10.811, -11.338, -11.865, -12.393,
        -12.920, -13.447, -13.975, -14.502, -15.029, -15.557, -16.084, -16.611,
};

/** Design values for altitude and azimuth offset angles for gen1 sensors. */
static const std::vector<double> gen1_azimuth_angles = {
        3.164, 1.055, -1.055, -3.164, 3.164, 1.055, -1.055, -3.164,
        3.164, 1.055, -1.055, -3.164, 3.164, 1.055, -1.055, -3.164,
        3.164, 1.055, -1.055, -3.164, 3.164, 1.055, -1.055, -3.164,
        3.164, 1.055, -1.055, -3.164, 3.164, 1.055, -1.055, -3.164,
        3.164, 1.055, -1.055, -3.164, 3.164, 1.055, -1.055, -3.164,
        3.164, 1.055, -1.055, -3.164, 3.164, 1.055, -1.055, -3.164,
        3.164, 1.055, -1.055, -3.164, 3.164, 1.055, -1.055, -3.164,
        3.164, 1.055, -1.055, -3.164, 3.164, 1.055, -1.055, -3.164,
};

/** Design values for imu and lidar to sensor-frame transforms. */
static const mat4d default_imu_to_sensor_transform =
        (mat4d() << 1, 0, 0, 6.253, 0, 1, 0, -11.775, 0, 0, 1, 7.645, 0, 0, 0,
         1)
                .finished();

/** Design values for imu and lidar to sensor-frame transforms. */
static const mat4d default_lidar_to_sensor_transform =
        (mat4d() << -1, 0, 0, 0, 0, -1, 0, 0, 0, 0, 1, 36.18, 0, 0, 0, 1)
                .finished();

/**
 * Constants used for configuration. Refer to the sensor documentation for the
 * meaning of each option.
 */
enum lidar_mode
{
  MODE_UNSPEC = 0,///< lidar mode: unspecified
  MODE_512x10,    ///< lidar mode: 10 scans of 512 columns per second
  MODE_512x20,    ///< lidar mode: 20 scans of 512 columns per second
  MODE_1024x10,   ///< lidar mode: 10 scans of 1024 columns per second
  MODE_1024x20,   ///< lidar mode: 20 scans of 1024 columsn per second
  MODE_2048x10,   ///< lidar mode: 10 scans of 2048 columns per second
  MODE_4096x5     ///< lidar mode: 5 scans of 4096 columns per second. Only
                  ///< available on select sensors

};

/**
 * Mode controlling timestamp method. Refer to the sensor documentation for the
 * meaning of each option.
 */
enum timestamp_mode
{
  /**
     * Timestamp mode unspecified.
     */
  TIME_FROM_UNSPEC = 0,

  /**
     * Use the internal clock.
     */
  TIME_FROM_INTERNAL_OSC,

  /**
     * A free running counter synced to the SYNC_PULSE_IN input
     * counts seconds (# of pulses) and nanoseconds since sensor turn
     * on.
     */
  TIME_FROM_SYNC_PULSE_IN,

  /** Synchronize with an external PTP master. */
  TIME_FROM_PTP_1588
};

/**
 * Mode controlling sensor operation. Refer to the sensor documentation for the
 * meaning of each option.
 */
enum OperatingMode
{
  OPERATING_NORMAL = 1,///< Normal sensor operation
  OPERATING_STANDBY    ///< Standby
};

/**
 * Mode controlling ways to input timesync information. Refer to the sensor
 * documentation for the meaning of each option.
 */
enum MultipurposeIOMode
{

  MULTIPURPOSE_OFF = 1,///< Multipurpose IO is turned off (default)

  /**
     * Used in conjunction with timestamp_mode::TIME_FROM_SYNC_PULSE_IN
     * to enable time pulses in on the multipurpose io input.
     */
  MULTIPURPOSE_INPUT_NMEA_UART,

  /**
     * Output a SYNC_PULSE_OUT signal synchronized with
     * the internal clock.
     */
  MULTIPURPOSE_OUTPUT_FROM_INTERNAL_OSC,

  /**
     * Output a SYNC_PULSE_OUT signal synchronized with
     * a SYNC_PULSE_IN provided to the unit.
     */
  MULTIPURPOSE_OUTPUT_FROM_SYNC_PULSE_IN,

  /**
     * Output a SYNC_PULSE_OUT signal synchronized with
     * an external PTP IEEE 1588 master.
     */
  MULTIPURPOSE_OUTPUT_FROM_PTP_1588,

  /**
     * Output a SYNC_PULSE_OUT signal with a user defined
     * rate in an integer number of degrees.
     */
  MULTIPURPOSE_OUTPUT_FROM_ENCODER_ANGLE
};

/**
 * Polarity represents polarity of NMEA UART and SYNC_PULSE inputs and outputs.
 * See sensor docs for more details.
 */
enum Polarity
{
  POLARITY_ACTIVE_LOW = 1,///< ACTIVE_LOW
  POLARITY_ACTIVE_HIGH    ///< ACTIVE_HIGH
};

/**
 * Baud rate the sensor attempts for NMEA UART input $GPRMC messages
 * See sensor docs for more details.
 */
enum NMEABaudRate
{
  BAUD_9600 = 1,///< 9600 bits per second UART baud rate
  BAUD_115200   ///< 115200 bits per second UART baud rate
};

/** Profile indicating packet format of lidar data. */
enum UDPProfileLidar
{
  /** Legacy lidar data */
  PROFILE_LIDAR_LEGACY = 1,

  /** Dual Returns data */
  PROFILE_RNG19_RFL8_SIG16_NIR16_DUAL,

  /** Single Returns data */
  PROFILE_RNG19_RFL8_SIG16_NIR16,

  /** Single Returns Low Data Rate */
  PROFILE_RNG15_RFL8_NIR8,

  /** Five Word Profile */
  PROFILE_FIVE_WORD_PIXEL,
};

/** Profile indicating packet format of IMU data. */
enum UDPProfileIMU
{
  PROFILE_IMU_LEGACY = 1///< Legacy IMU data
};

/** Thermal Shutdown status. */
enum ThermalShutdownStatus
{
  THERMAL_SHUTDOWN_NORMAL = 0x00,  ///< Normal operation
  THERMAL_SHUTDOWN_IMMINENT = 0x01,///< Thermal Shutdown imminent
};

/** Shot Limiting status. */
enum ShotLimitingStatus
{
  SHOT_LIMITING_NORMAL = 0x00,         ///< Normal operation
  SHOT_LIMITING_IMMINENT = 0x01,       ///< Shot limiting imminent
  SHOT_LIMITING_REDUCTION_0_10 = 0x02, //< Shot limiting reduction by 0 to 10%
  SHOT_LIMITING_REDUCTION_10_20 = 0x03,///< Shot limiting reduction by 10 to 20%
  SHOT_LIMITING_REDUCTION_20_30 = 0x04,///< Shot limiting reduction by 20 to 30%
  SHOT_LIMITING_REDUCTION_30_40 = 0x05,///< Shot limiting reduction by 30 to 40%
  SHOT_LIMITING_REDUCTION_40_50 = 0x06,///< Shot limiting reduction by 40 to 50%
  SHOT_LIMITING_REDUCTION_50_60 = 0x07,///< Shot limiting reduction by 50 to 60%
  SHOT_LIMITING_REDUCTION_60_70 = 0x08,///< Shot limiting reduction by 60 to 70%
  SHOT_LIMITING_REDUCTION_70_75 = 0x09,///< Shot limiting reduction by 70 to 80%
};

/**
 * Convenience type alias for azimuth windows, the window over which the
 * sensor fires in millidegrees.
 */
using AzimuthWindow = std::pair<int, int>;
/**
 * Convenience type alias for column windows, the window over which the
 * sensor fires in columns.
 */
using ColumnWindow = std::pair<int, int>;

/**
 * Struct for sensor configuration parameters.
 */
struct sensor_config {
  std::optional<std::string> udp_dest;///< The destination address for the
                                      ///< lidar/imu data to be sent to
  std::optional<int> udp_port_lidar;  ///< The destination port for the lidar
                                      ///< data to be sent to
  std::optional<int> udp_port_imu;    ///< The destination port for the imu data
                                      ///< to be sent to

  // TODO: replace ts_mode and ld_mode when timestamp_mode and
  // lidar_mode get changed to CapsCase
  /**
     * The timestamp mode for the sensor to use.
     * Refer to timestamp_mode for more details.
     */
  std::optional<timestamp_mode> ts_mode;

  /**
     * The lidar mode for the sensor to use.
     * Refer to lidar_mode for more details.
     */
  std::optional<lidar_mode> ld_mode;

  /**
     * The operating mode for the sensor to use.
     * Refer to OperatingMode for more details.
     */
  std::optional<OperatingMode> operating_mode;

  /**
     * The multipurpose io mode for the sensor to use.
     * Refer to MultipurposeIOMode for more details.
     */
  std::optional<MultipurposeIOMode> multipurpose_io_mode;

  /**
     * The azimuth window for the sensor to use.
     * Refer to AzimuthWindow for more details.
     */
  std::optional<AzimuthWindow> azimuth_window;

  /**
     * Multiplier for signal strength of sensor. See the sensor docs for
     * more details on usage.
     */
  std::optional<double> signal_multiplier;

  /**
     * The nmea polarity for the sensor to use.
     * Refer to Polarity for more details.
     */
  std::optional<Polarity> nmea_in_polarity;

  /**
     * Whether NMEA UART input $GPRMC messages should be ignored.
     * Refer to the sensor docs for more details.
     */
  std::optional<bool> nmea_ignore_valid_char;

  /**
     * The nmea baud rate for the sensor to use.
     * Refer to Polarity> for more details.
     */
  std::optional<NMEABaudRate> nmea_baud_rate;

  /**
     * Number of leap seconds added to UDP timestamp.
     * See the sensor docs for more details.
     */
  std::optional<int> nmea_leap_seconds;

  /**
     * Polarity of SYNC_PULSE_IN input.
     * See Polarity for more details.
     */
  std::optional<Polarity> sync_pulse_in_polarity;

  /**
     * Polarity of SYNC_PULSE_OUT output.
     * See Polarity for more details.
     */
  std::optional<Polarity> sync_pulse_out_polarity;

  /**
     * Angle in degrees that sensor traverses between each SYNC_PULSE_OUT
     * pulse. See senor docs for more details.
     */
  std::optional<int> sync_pulse_out_angle;

  /**
     * Width of SYNC_PULSE_OUT pulse in ms.
     * See sensor docs for more details.
     */
  std::optional<int> sync_pulse_out_pulse_width;

  /**
     * Frequency of SYNC_PULSE_OUT pulse in Hz.
     * See sensor docs for more details.
     */
  std::optional<int> sync_pulse_out_frequency;

  /**
     * Whether phase locking is enabled.
     * See sensor docs for more details.
     */
  std::optional<bool> phase_lock_enable;

  /**
     * Angle that sensors are locked to in millidegrees.
     * See sensor docs for more details.
     */
  std::optional<int> phase_lock_offset;

  /**
     * Columns per packet.
     * See sensor docs for more details.
     */
  std::optional<int> columns_per_packet;

  /**
     * The lidar profile for the sensor to use.
     * Refer to UDPProfileLidar for more details.
     */
  std::optional<UDPProfileLidar> udp_profile_lidar;

  /**
     * The imu profile for the sensor to use.
     * Refer to UDPProfileIMU for more details.
     */
  std::optional<UDPProfileIMU> udp_profile_imu;
};

/** Stores data format information. */
struct data_format {
  uint32_t pixels_per_column; ///< pixels per column
  uint32_t columns_per_packet;///< columns per packet
  uint32_t
          columns_per_frame;///< columns per frame, should match with lidar mode
  std::vector<int>
          pixel_shift_by_row;///< shift of pixels by row to enable destagger
  ColumnWindow column_window;///< window of columns over which sensor fires
  UDPProfileLidar udp_profile_lidar;///< profile of lidar packet
  UDPProfileIMU udp_profile_imu;    ///< profile of imu packet
  uint16_t fps;                     ///< frames per second
};

/** Stores necessary information from sensor to parse and project sensor data.
 */
struct sensor_info {
  [[deprecated("Will be removed in the next version")]] std::string
          name;         ///< @deprecated Will be removed in the next version
  std::string sn;       ///< sensor serial number
  std::string fw_rev;   ///< fw revision
  lidar_mode mode;      ///< lidar mode of sensor
  std::string prod_line;///< prod line
  data_format format;   ///< data format of sensor
  std::vector<double>
          beam_azimuth_angles;///< beam azimuth angles for 3D projection
  std::vector<double>
          beam_altitude_angles;///< beam altitude angles for 3D projection
  double lidar_origin_to_beam_origin_mm;///< distance between lidar origin
                                        ///< and beam origin in mm
  mat4d beam_to_lidar_transform;  ///< transform between beam and lidar frame
  mat4d imu_to_sensor_transform;  ///< transform between sensor coordinate
                                  ///< frame and imu
  mat4d lidar_to_sensor_transform;///< transform between lidar and sensor
                                  ///< coordinate frames
  mat4d extrinsic;                ///< extrinsic matrix
  uint32_t init_id;               ///< initialization ID updated every reinit
  uint16_t udp_port_lidar;        ///< the lidar destination port
  uint16_t udp_port_imu;          ///< the imu destination port
};

// clang-format off
/** Tag to identitify a paricular value reported in the sensor channel data
 * block. */
    enum ChanField {
        RANGE = 1,            ///< 1st return range in mm
        RANGE2 = 2,           ///< 2nd return range in mm
        INTENSITY = 3,        ///< @deprecated Use SIGNAL instead
        SIGNAL = 3,           ///< 1st return signal in photons
        SIGNAL2 = 4,          ///< 2nd return signal in photons
        REFLECTIVITY = 5,     ///< 1st return reflectivity, calibrated by range and sensor
        ///< sensitivity in FW 2.1+. See sensor docs for more details
        REFLECTIVITY2 = 6,    ///< 2nd return reflectivity, calibrated by range and sensor
        ///< sensitivity in FW 2.1+. See sensor docs for more details
        AMBIENT = 7,          ///< @deprecated Use NEAR_IR instead
        NEAR_IR = 7,          ///< near_ir in photons
        FLAGS = 8,            ///< 1st return flags
        FLAGS2 = 9,           ///< 2nd return flags
        RAW_HEADERS = 40,     ///< raw headers for packet/footer/column for dev use
        RAW32_WORD5 = 45,     ///< raw word access to packet for dev use
        RAW32_WORD6 = 46,     ///< raw word access to packet for dev use
        RAW32_WORD7 = 47,     ///< raw word access to packet for dev use
        RAW32_WORD8 = 48,     ///< raw word access to packet for dev use
        RAW32_WORD9 = 49,     ///< raw word access to packet for dev use
        CUSTOM0 = 50,         ///< custom user field
        CUSTOM1 = 51,         ///< custom user field
        CUSTOM2 = 52,         ///< custom user field
        CUSTOM3 = 53,         ///< custom user field
        CUSTOM4 = 54,         ///< custom user field
        CUSTOM5 = 55,         ///< custom user field
        CUSTOM6 = 56,         ///< custom user field
        CUSTOM7 = 57,         ///< custom user field
        CUSTOM8 = 58,         ///< custom user field
        CUSTOM9 = 59,         ///< custom user field
        RAW32_WORD1 = 60,     ///< raw word access to packet for dev use
        RAW32_WORD2 = 61,     ///< raw word access to packet for dev use
        RAW32_WORD3 = 62,     ///< raw word access to packet for dev use
        RAW32_WORD4 = 63,     ///< raw word access to packet for dev use
        CHAN_FIELD_MAX = 64,  ///< max which allows us to introduce future fields
    };
// clang-format on

template<typename K, typename V, size_t N>
using Table = std::array<std::pair<K, V>, N>;

static constexpr Table<lidar_mode, const char *, 7> lidar_mode_strings{
        {{MODE_UNSPEC, "UNKNOWN"},
         {MODE_512x10, "512x10"},
         {MODE_512x20, "512x20"},
         {MODE_1024x10, "1024x10"},
         {MODE_1024x20, "1024x20"},
         {MODE_2048x10, "2048x10"},
         {MODE_4096x5, "4096x5"}}};

static constexpr Table<timestamp_mode, const char *, 4> timestamp_mode_strings{
        {{TIME_FROM_UNSPEC, "UNKNOWN"},
         {TIME_FROM_INTERNAL_OSC, "TIME_FROM_INTERNAL_OSC"},
         {TIME_FROM_SYNC_PULSE_IN, "TIME_FROM_SYNC_PULSE_IN"},
         {TIME_FROM_PTP_1588, "TIME_FROM_PTP_1588"}}};

static constexpr Table<OperatingMode, const char *, 2> operating_mode_strings{
        {{OPERATING_NORMAL, "NORMAL"}, {OPERATING_STANDBY, "STANDBY"}}};

static constexpr Table<MultipurposeIOMode, const char *, 6>
        multipurpose_io_mode_strings{
                {{MULTIPURPOSE_OFF, "OFF"},
                 {MULTIPURPOSE_INPUT_NMEA_UART, "INPUT_NMEA_UART"},
                 {MULTIPURPOSE_OUTPUT_FROM_INTERNAL_OSC,
                  "OUTPUT_FROM_INTERNAL_OSC"},
                 {MULTIPURPOSE_OUTPUT_FROM_SYNC_PULSE_IN,
                  "OUTPUT_FROM_SYNC_PULSE_IN"},
                 {MULTIPURPOSE_OUTPUT_FROM_PTP_1588, "OUTPUT_FROM_PTP_1588"},
                 {MULTIPURPOSE_OUTPUT_FROM_ENCODER_ANGLE,
                  "OUTPUT_FROM_ENCODER_ANGLE"}}};

static constexpr Table<Polarity, const char *, 2> polarity_strings{
        {{POLARITY_ACTIVE_LOW, "ACTIVE_LOW"},
         {POLARITY_ACTIVE_HIGH, "ACTIVE_HIGH"}}};

static constexpr Table<NMEABaudRate, const char *, 2> nmea_baud_rate_strings{
        {{BAUD_9600, "BAUD_9600"}, {BAUD_115200, "BAUD_115200"}}};

static constexpr Table<ChanField, const char *, 29> chanfield_strings{{
        {ChanField::RANGE, "RANGE"},
        {ChanField::RANGE2, "RANGE2"},
        {ChanField::SIGNAL, "SIGNAL"},
        {ChanField::SIGNAL2, "SIGNAL2"},
        {ChanField::REFLECTIVITY, "REFLECTIVITY"},
        {ChanField::REFLECTIVITY2, "REFLECTIVITY2"},
        {ChanField::NEAR_IR, "NEAR_IR"},
        {ChanField::FLAGS, "FLAGS"},
        {ChanField::FLAGS2, "FLAGS2"},
        {ChanField::RAW_HEADERS, "RAW_HEADERS"},
        {ChanField::CUSTOM0, "CUSTOM0"},
        {ChanField::CUSTOM1, "CUSTOM1"},
        {ChanField::CUSTOM2, "CUSTOM2"},
        {ChanField::CUSTOM3, "CUSTOM3"},
        {ChanField::CUSTOM4, "CUSTOM4"},
        {ChanField::CUSTOM5, "CUSTOM5"},
        {ChanField::CUSTOM6, "CUSTOM6"},
        {ChanField::CUSTOM7, "CUSTOM7"},
        {ChanField::CUSTOM8, "CUSTOM8"},
        {ChanField::CUSTOM9, "CUSTOM9"},
        {ChanField::RAW32_WORD1, "RAW32_WORD1"},
        {ChanField::RAW32_WORD2, "RAW32_WORD2"},
        {ChanField::RAW32_WORD3, "RAW32_WORD3"},
        {ChanField::RAW32_WORD4, "RAW32_WORD4"},
        {ChanField::RAW32_WORD5, "RAW32_WORD5"},
        {ChanField::RAW32_WORD6, "RAW32_WORD6"},
        {ChanField::RAW32_WORD7, "RAW32_WORD7"},
        {ChanField::RAW32_WORD8, "RAW32_WORD8"},
        {ChanField::RAW32_WORD9, "RAW32_WORD9"},
}};

static constexpr Table<UDPProfileLidar, const char *, 5>
        udp_profile_lidar_strings{{
                {PROFILE_LIDAR_LEGACY, "LEGACY"},
                {PROFILE_RNG19_RFL8_SIG16_NIR16_DUAL,
                 "RNG19_RFL8_SIG16_NIR16_DUAL"},
                {PROFILE_RNG19_RFL8_SIG16_NIR16, "RNG19_RFL8_SIG16_NIR16"},
                {PROFILE_RNG15_RFL8_NIR8, "RNG15_RFL8_NIR8"},
                {PROFILE_FIVE_WORD_PIXEL, "FIVE_WORD_PIXEL"},
        }};

static constexpr Table<UDPProfileIMU, const char *, 1> udp_profile_imu_strings{{
        {PROFILE_IMU_LEGACY, "LEGACY"},
}};

// TODO: should we name them something better? feel like the most important is
// SHOT_LIMITING_NORMAL
static constexpr Table<ShotLimitingStatus, const char *, 10>
        shot_limiting_status_strings{{
                {SHOT_LIMITING_NORMAL, "SHOT_LIMITING_NORMAL"},
                {SHOT_LIMITING_IMMINENT, "SHOT_LIMITING_IMMINENT"},
                {SHOT_LIMITING_REDUCTION_0_10, "SHOT_LIMITING_REDUCTION_0_10"},
                {SHOT_LIMITING_REDUCTION_10_20,
                 "SHOT_LIMITING_REDUCTION_10_20"},
                {SHOT_LIMITING_REDUCTION_20_30,
                 "SHOT_LIMITING_REDUCTION_20_30"},
                {SHOT_LIMITING_REDUCTION_30_40,
                 "SHOT_LIMITING_REDUCTION_30_40"},
                {SHOT_LIMITING_REDUCTION_40_50,
                 "SHOT_LIMITING_REDUCTION_40_50"},
                {SHOT_LIMITING_REDUCTION_50_60,
                 "SHOT_LIMITING_REDUCTION_50_60"},
                {SHOT_LIMITING_REDUCTION_60_70,
                 "SHOT_LIMITING_REDUCTION_60_70"},
                {SHOT_LIMITING_REDUCTION_70_75,
                 "SHOT_LIMITING_REDUCTION_70_75"},
        }};

// TODO: do we want these? do we like the names?
static constexpr Table<ThermalShutdownStatus, const char *, 2>
        thermal_shutdown_status_strings{{
                {THERMAL_SHUTDOWN_NORMAL, "THERMAL_SHUTDOWN_NORMAL"},
                {THERMAL_SHUTDOWN_IMMINENT, "THERMAL_SHUTDOWN_IMMINENT"},
        }};

// bool represents whether it is an object (true) or just a member (false)
// NOTE: lidar_data_format and calibration_status should be objects but as they
// were introduced earlier, non-legacy formats for FW version do not include
// them
// TODO parse metadata by FW version specified ?
static const std::map<std::string, bool> nonlegacy_metadata_fields = {
        {"sensor_info", true},        {"beam_intrinsics", true},
        {"imu_intrinsics", true},     {"lidar_intrinsics", true},
        {"config_params", true},      {"lidar_data_format", false},
        {"calibration_status", false}};

/**
 * Equality for data_format.
 *
 * @param[in] lhs The first object to compare.
 * @param[in] rhs The second object to compare.
 *
 * @return lhs == rhs
 */
bool operator==(const data_format &lhs, const data_format &rhs);

/**
 * Not-Equality for data_format.
 *
 * @param[in] lhs The first object to compare.
 * @param[in] rhs The second object to compare.
 *
 * @return lhs != rhs
 */
bool operator!=(const data_format &lhs, const data_format &rhs);

/**
 * Equality for sensor_info.
 *
 * @param[in] lhs The first object to compare.
 * @param[in] rhs The second object to compare.
 *
 * @return lhs == rhs
 */
bool operator==(const sensor_info &lhs, const sensor_info &rhs);

/**
 * Not-Equality for sensor_info.
 *
 * @param[in] lhs The first object to compare.
 * @param[in] rhs The second object to compare.
 *
 * @return lhs != rhs
 */
bool operator!=(const sensor_info &lhs, const sensor_info &rhs);

/**
 * Equality for sensor config.
 *
 * @param[in] lhs The first object to compare.
 * @param[in] rhs The second object to compare.
 *
 * @return lhs == rhs
 */
bool operator==(const sensor_config &lhs, const sensor_config &rhs);

/**
 * Not-Equality for sensor config.
 *
 * @param[in] lhs The first object to compare.
 * @param[in] rhs The second object to compare.
 *
 * @return lhs != rhs
 */
bool operator!=(const sensor_config &lhs, const sensor_config &rhs);

/**
 * Get a default sensor_info for the given lidar mode.
 *
 * @param[in] mode lidar mode to generate default sensor_info for.
 *
 * @return default sensor_info for the OS1-64.
 */
sensor_info default_sensor_info(lidar_mode mode);

/* String conversion */

template<typename K, typename V, size_t N>
inline static std::optional<V> lookup(const Table<K, V, N> table, const K &k)
{
  auto end = table.end();
  auto res = std::find_if(table.begin(), end, [&](const std::pair<K, V> &p) {
    return p.first == k;
  });

  return res == end ? std::nullopt : std::make_optional<V>(res->second);
}

template<typename K, size_t N>
inline static std::optional<K> rlookup(const Table<K, const char *, N> table,
                                       const char *v)
{
  auto end = table.end();
  auto res = std::find_if(table.begin(), end,
                          [&](const std::pair<K, const char *> &p) {
                            return std::strcmp(p.second, v) == 0;
                          });

  return res == end ? std::nullopt : std::make_optional<K>(res->first);
}

/**
 * Get string representation of a lidar mode.
 *
 * @param[in] mode lidar_mode to get the string representation for.
 *
 * @return string representation of the lidar mode, or "UNKNOWN".
 */
inline std::string to_string(lidar_mode mode)
{
  auto res = lookup(lidar_mode_strings, mode);
  return res ? res.value() : "UNKNOWN";
}

/**
 * Get lidar mode from string.
 *
 * @param[in] s String to decode.
 *
 * @return lidar mode corresponding to the string, or 0 on error.
 */
inline lidar_mode lidar_mode_of_string(const std::string &s)
{
  auto res = rlookup(lidar_mode_strings, s.c_str());
  return res ? res.value() : lidar_mode::MODE_UNSPEC;
}

/**
 * Get number of columns in a scan for a lidar mode.
 *
 * @param[in] mode lidar_mode to get the number of columns for.
 *
 * @return number of columns per rotation for the mode.
 */
inline uint32_t n_cols_of_lidar_mode(lidar_mode mode)
{
  switch (mode) {
    case MODE_512x10:
    case MODE_512x20:
      return 512;
    case MODE_1024x10:
    case MODE_1024x20:
      return 1024;
    case MODE_2048x10:
      return 2048;
    case MODE_4096x5:
      return 4096;
    default:
      throw std::invalid_argument{"n_cols_of_lidar_mode"};
  }
}

/**
 * Get the lidar rotation frequency from lidar mode.
 *
 * @param[in] mode Lidar mode to get the rotation frequency from.
 *
 * @return lidar rotation frequency in Hz.
 */
inline int frequency_of_lidar_mode(lidar_mode mode)
{
  switch (mode) {
    case MODE_4096x5:
      return 5;
    case MODE_512x10:
    case MODE_1024x10:
    case MODE_2048x10:
      return 10;
    case MODE_512x20:
    case MODE_1024x20:
      return 20;
    default:
      throw std::invalid_argument{"frequency_of_lidar_mode"};
  }
}

/**
 * Get string representation of a timestamp mode.
 *
 * @param[in] mode timestamp_mode to get the string representation for.
 *
 * @return string representation of the timestamp mode, or "UNKNOWN".
 */
inline std::string to_string(timestamp_mode mode)
{
  auto res = lookup(timestamp_mode_strings, mode);
  return res ? res.value() : "UNKNOWN";
}

/**
 * Get timestamp mode from string.
 *
 * @param[in] s String to decode into a timestamp mode.
 *
 * @return timestamp mode corresponding to the string, or 0 on error.
 */
inline timestamp_mode timestamp_mode_of_string(const std::string &s)
{
  auto res = rlookup(timestamp_mode_strings, s.c_str());
  return res ? res.value() : timestamp_mode::TIME_FROM_UNSPEC;
}

/**
 * Get string representation of an operating mode.
 *
 * @param[in] mode Operating mode to get the string representation from.
 *
 * @return string representation of the operating mode, or "UNKNOWN".
 */
inline std::string to_string(OperatingMode mode)
{
  auto res = lookup(operating_mode_strings, mode);
  return res ? res.value() : "UNKNOWN";
}

/**
 * Get operating mode from string.
 *
 * @param s String to get the operating mode from.
 *
 * @return operating mode corresponding to the string, or 0 on error.
 */
inline std::optional<OperatingMode>
operating_mode_of_string(const std::string &s)
{
  return rlookup(operating_mode_strings, s.c_str());
}

/**
 * Get string representation of a multipurpose io mode.
 *
 * @param[in] mode Multipurpose io mode to get a string representation from.
 *
 * @return string representation of the multipurpose io mode, or "UNKNOWN".
 */
inline std::string to_string(MultipurposeIOMode mode)
{
  auto res = lookup(multipurpose_io_mode_strings, mode);
  return res ? res.value() : "UNKNOWN";
}

/**
 * Get multipurpose io mode from string.
 *
 * @param[in] s String to decode into a multipurpose io mode.
 *
 * @return multipurpose io mode corresponding to the string, or 0 on error.
 */
inline std::optional<MultipurposeIOMode>
multipurpose_io_mode_of_string(const std::string &s)
{
  return rlookup(multipurpose_io_mode_strings, s.c_str());
}

/**
 * Get string representation of a polarity.
 *
 * @param[in] polarity The polarity to get the string representation of.
 *
 * @return string representation of the polarity, or "UNKNOWN".
 */
inline std::string to_string(Polarity polarity)
{
  auto res = lookup(polarity_strings, polarity);
  return res ? res.value() : "UNKNOWN";
}

/**
 * Get polarity from string.
 *
 * @param[in] s The string to decode into a polarity.
 *
 * @return polarity corresponding to the string, or 0 on error.
 */
inline std::optional<Polarity> polarity_of_string(const std::string &s)
{
  return rlookup(polarity_strings, s.c_str());
}

/**
 * Get string representation of a NMEA Baud Rate.
 *
 * @param[in] rate The NNEABaudRate to get the string representation of.
 *
 * @return string representation of the NMEA baud rate, or "UNKNOWN".
 */
inline std::string to_string(NMEABaudRate rate)
{
  auto res = lookup(nmea_baud_rate_strings, rate);
  return res ? res.value() : "UNKNOWN";
}

/**
 * Get nmea baud rate from string.
 *
 * @param[in] s The string to decode into a NMEA baud rate.
 *
 * @return nmea baud rate corresponding to the string, or 0 on error.
 */
inline std::optional<NMEABaudRate>
nmea_baud_rate_of_string(const std::string &s)
{
  return rlookup(nmea_baud_rate_strings, s.c_str());
}

/**
 * Get string representation of an Azimuth Window.
 *
 * @param[in] azimuth_window The azimuth window to get the string
representation. of
 *
 * @return string representation of the azimuth window.
 */
inline std::string to_string(AzimuthWindow azimuth_window)
{
  std::stringstream ss;
  ss << "[" << azimuth_window.first << ", " << azimuth_window.second << "]";
  return ss.str();
}

/**
 * Get string representation of a lidar profile.
 *
 * @param[in] profile The profile to get the string representation of.
 *
 * @return string representation of the lidar profile.
 */
inline std::string to_string(UDPProfileLidar profile)
{
  auto res = lookup(udp_profile_lidar_strings, profile);
  return res ? res.value() : "UNKNOWN";
}

/**
 * Get lidar profile from string.
 *
 * @param[in] s The string to decode into a lidar profile.
 *
 * @return lidar profile corresponding to the string, or nullopt on error.
 */
inline std::optional<UDPProfileLidar>
udp_profile_lidar_of_string(const std::string &s)
{
  return rlookup(udp_profile_lidar_strings, s.c_str());
}

/**
 * Get string representation of an IMU profile.
 *
 * @param[in] profile The profile to get the string representation of.
 *
 * @return string representation of the lidar profile.
 */
inline std::string to_string(UDPProfileIMU profile)
{
  auto res = lookup(udp_profile_imu_strings, profile);
  return res ? res.value() : "UNKNOWN";
}

/**
 * Get imu profile from string
 *
 * @param[in] s The string to decode into an imu profile.
 *
 * @return imu profile corresponding to the string, or nullopt on error.
 */
inline std::optional<UDPProfileIMU>
udp_profile_imu_of_string(const std::string &s)
{
  return rlookup(udp_profile_imu_strings, s.c_str());
}

/**
 * Get string representation of a Shot Limiting Status.
 *
 * @param[in] shot_limiting_status The shot limiting status to get the string
 * representation of.
 *
 * @return string representation of the shot limiting status.
 */
inline std::string to_string(ShotLimitingStatus shot_limiting_status)
{
  auto res = lookup(shot_limiting_status_strings, shot_limiting_status);
  return res ? res.value() : "UNKNOWN";
}

/**
 * Get string representation of Thermal Shutdown Status.
 *
 * @param[in] thermal_shutdown_status The thermal shutdown status to get the
 * string representation of.
 *
 * @return string representation of thermal shutdown status.
 */
inline std::string to_string(ThermalShutdownStatus thermal_shutdown_status)
{
  auto res = lookup(thermal_shutdown_status_strings, thermal_shutdown_status);
  return res ? res.value() : "UNKNOWN";
}

/**
 * Determine validity of provided signal multiplier value
 *
 * @param[in] signal_multiplier Signal multiplier value.
 */
void check_signal_multiplier(double signal_multiplier);

/**
 * Parse metadata text blob from the sensor into a sensor_info struct.
 *
 * String and vector fields will have size 0 if the parameter cannot
 * be found or parsed, while lidar_mode will be set to 0 (invalid).
 *
 * @throw runtime_error if the text is not valid json
 *
 * @param[in] metadata a text blob returned by get_metadata from client.h.
 *
 * @return a sensor_info struct populated with a subset of the metadata.
 */
sensor_info parse_metadata(const std::string &metadata);

/**
 * Parse metadata given path to a json file.
 *
 * @throw runtime_error if json file does not exist or is malformed.
 *
 * @param[in] json_file path to a json file containing sensor metadata.
 *
 * @return a sensor_info struct populated with a subset of the metadata.
 */
sensor_info metadata_from_json(const std::string &json_file);

/**
 * Get a string representation of the sensor_info. All fields included. Not
 * equivalent or interchangeable with metadata from sensor.
 *
 * @param[in] info sensor_info struct
 *
 * @return a json metadata string
 */
std::string to_string(const sensor_info &info);

/**
 * Parse config text blob from the sensor into a sensor_config struct.
 *
 * All fields are optional, and will only be set if found.
 *
 * @throw runtime_error if the text is not valid json.
 *
 * @param[in] config a text blob given by get_config from client.h.
 *
 * @return a sensor_config struct populated with the sensor config.
 * parameters.
 */
sensor_config parse_config(const std::string &config);

/**
 * Get a string representation of sensor config. Only set fields will be
 * represented.
 *
 * @param[in] config a struct of sensor config.
 *
 * @return a json sensor config string.
 */
std::string to_string(const sensor_config &config);

/**
 * Convert non-legacy string representation of metadata to legacy.
 *
 * @param[in] metadata non-legacy string representation of metadata.
 *
 * @return legacy string representation of metadata.
 */
std::string convert_to_legacy(const std::string &metadata);

/**
 * Get client version.
 *
 * @return client version string
 */
inline std::string client_version() { return {"ouster_client_undefined"}; }

/**
 * Get string representation of a channel field.
 *
 * @param[in] field The field to get the string representation of.
 *
 * @return string representation of the channel field.
 */
inline std::string to_string(ChanField field)
{
  auto res = lookup(chanfield_strings, field);
  return res ? res.value() : "UNKNOWN";
}

/**
 * Types of channel fields.
 */
enum ChanFieldType
{
  VOID = 0,
  UINT8,
  UINT16,
  UINT32,
  UINT64
};

/**
 * Get the size of the ChanFieldType in bytes.
 *
 * @param[in] ft the field type
 *
 * @return size of the field type in bytes
 */
inline size_t field_type_size(ChanFieldType ft)
{
  switch (ft) {
    case ChanFieldType::UINT8:
      return 1;
    case ChanFieldType::UINT16:
      return 2;
    case ChanFieldType::UINT32:
      return 4;
    case ChanFieldType::UINT64:
      return 8;
    default:
      return 0;
  }
}

/**
 * Get string representation of a channel field.
 *
 * @param[in] ft The field type to get the string representation of.
 *
 * @return string representation of the channel field type.
 */
inline std::string to_string(ChanFieldType ft)
{
  switch (ft) {
    case ChanFieldType::VOID:
      return "VOID";
    case ChanFieldType::UINT8:
      return "UINT8";
    case ChanFieldType::UINT16:
      return "UINT16";
    case ChanFieldType::UINT32:
      return "UINT32";
    case ChanFieldType::UINT64:
      return "UINT64";
    default:
      return "UNKNOWN";
  }
}

/**
 * Table of accessors for extracting data from imu and lidar packets.
 *
 * In the user guide, refer to section 9 for the lidar packet format and section
 * 10 for imu packets.
 *
 * For 0 <= n < columns_per_packet, nth_col(n, packet_buf) returns a pointer to
 * the nth measurement block. For 0 <= m < pixels_per_column, nth_px(m, col_buf)
 * returns the mth channel data block.
 *
 * Use imu_la_{x,y,z} to access the acceleration in the corresponding
 * direction. Use imu_av_{x,y,z} to read the angular velocity.
 */
class packet_format final
{
  template<typename T>
  T px_field(const uint8_t *px_buf, ChanField i) const;

  struct Impl;
  std::shared_ptr<const Impl> impl_;

  std::vector<std::pair<ChanField, ChanFieldType>> field_types_;

  public:
  explicit packet_format(
          const sensor_info &info);//< create packet_format from sensor_info

  using FieldIter =
          decltype(field_types_)::const_iterator;///< iterator over field types
                                                 ///< of packet

  const UDPProfileLidar
          udp_profile_lidar;     ///< udp lidar profile of packet format
  const size_t lidar_packet_size;///< lidar packet size
  const size_t imu_packet_size;  ///< imu packet size
  const int columns_per_packet;  ///< columns per lidar packet
  const int pixels_per_column;   ///< pixels per column for lidar
  [[deprecated]] const int encoder_ticks_per_rev;///< @deprecated

  const size_t packet_header_size;
  const size_t col_header_size;
  const size_t col_footer_size;
  const size_t col_size;
  const size_t packet_footer_size;

  /**
     * Read the packet type packet header.
     *
     * @param[in] lidar_buf the lidar buf.
     *
     * @return the packet type.
     */
  uint16_t packet_type(const uint8_t *lidar_buf) const;

  /**
     * Read the frame_id packet header.
     *
     * @param[in] lidar_buf the lidar buf.
     *
     * @return the frame id.
     */
  uint16_t frame_id(const uint8_t *lidar_buf) const;

  /**
     * Read the initialization id packet header.
     *
     * @param[in] lidar_buf the lidar buf.
     *
     * @return the init id.
     */
  uint32_t init_id(const uint8_t *lidar_buf) const;

  /**
     * Read the packet serial number header.
     *
     * @param[in] lidar_buf the lidar buf.
     *
     * @return the serial number.
     */
  uint64_t prod_sn(const uint8_t *lidar_buf) const;

  /**
     * Read the packet thermal shutdown countdown
     *
     * @param[in] lidar_buf the lidar buf.
     *
     * @return the thermal shutdown countdown.
     */
  uint16_t countdown_thermal_shutdown(const uint8_t *lidar_buf) const;

  /**
     * Read the packet shot limiting countdown
     *
     * @param[in] lidar_buf the lidar buf.
     *
     * @return the shot limiting countdown.
     */
  uint16_t countdown_shot_limiting(const uint8_t *lidar_buf) const;

  /**
     * Read the packet thermal shutdown header.
     *
     * @param[in] lidar_buf the lidar buf.
     *
     * @return the thermal shutdown status
     */
  uint8_t thermal_shutdown(const uint8_t *lidar_buf) const;

  /**
     * Read the packet shot limiting header.
     *
     * @param[in] lidar_buf the lidar buf.
     *
     * @return the shot limiting status
     */
  uint8_t shot_limiting(const uint8_t *lidar_buf) const;

  /**
     * Get the bit width of the specified channel field.
     *
     * @param[in] f the channel field to query.
     *
     * @return a type tag specifying the bitwidth of the requested field or
     * ChannelFieldType::VOID if it is not supported by the packet format.
     */
  ChanFieldType field_type(ChanField f) const;

  /**
     * A const forward iterator over field / type pairs.
     */
  FieldIter begin() const;

  /**
     * A const forward iterator over field / type pairs.
     */
  FieldIter end() const;

  /**
     * Get pointer to the packet footer of a lidar buffer.
     *
     * @param[in] lidar_buf the lidar buffer.
     *
     * @return pointer to packet footer of lidar buffer, can be nullptr if
     * packet format doesn't have packet footer.
     */
  const uint8_t *footer(const uint8_t *lidar_buf) const;

  // Measurement block accessors
  /**
     * Get pointer to nth column of a lidar buffer.
     *
     * @param[in] n which column.
     * @param[in] lidar_buf the lidar buffer.
     *
     * @return pointer to nth column of lidar buffer.
     */
  const uint8_t *nth_col(int n, const uint8_t *lidar_buf) const;

  /**
     * Read column timestamp from column buffer.
     *
     * @param[in] col_buf the column buffer.
     *
     * @return column timestamp.
     */
  uint64_t col_timestamp(const uint8_t *col_buf) const;

  /**
     * Read measurement id from column buffer.
     *
     * @param[in] col_buf the column buffer.
     *
     * @return column measurement id.
     */
  uint16_t col_measurement_id(const uint8_t *col_buf) const;

  /**
     * Read column status from column buffer.
     *
     * @param[in] col_buf the column buffer.
     *
     * @return column status.
     */
  uint32_t col_status(const uint8_t *col_buf) const;

  [[deprecated("Use col_measurement_id instead")]] uint32_t
  col_encoder(const uint8_t *col_buf)
          const;///< @deprecated Encoder count is deprecated as it is redundant
                ///< with measurement id, barring a multiplication factor which
                ///< varies by lidar mode. Use col_measurement_id instead
  [[deprecated("Use frame_id instead")]] uint16_t col_frame_id(
          const uint8_t *col_buf) const;///< @deprecated Use frame_id instead

  /**
     * Copy the specified channel field out of a packet measurement block.
     *
     * @tparam T T should be an unsigned integer type large enough to store
     * values of the specified field. Otherwise, data will be truncated.
     *
     * @param[in] col_buf a measurement block pointer returned by `nth_col()`.
     * @param[in] f the channel field to copy.
     * @param[out] dst destination array of size pixels_per_column * dst_stride.
     * @param[in] dst_stride stride for writing to the destination array.
     */
  template<typename T,
           typename std::enable_if<std::is_unsigned<T>::value, T>::type = 0>
  void col_field(const uint8_t *col_buf, ChanField f, T *dst,
                 int dst_stride = 1) const;

  // Per-pixel channel data block accessors
  /**
     * Get pointer to nth pixel of a column buffer.
     *
     * @param[in] n which pixel.
     * @param[in] col_buf the column buffer.
     *
     * @return pointer to nth pixel of a column buffer.
     */
  const uint8_t *nth_px(int n, const uint8_t *col_buf) const;

  /**
     * Read range from pixel buffer.
     *
     * @param[in] px_buf the pixel buffer.
     *
     * @return range from pixel buffer.
     */
  uint32_t px_range(const uint8_t *px_buf) const;

  /**
     * Read reflectivity from pixel buffer.
     *
     * @param[in] px_buf the pixel buffer.
     *
     * @return reflectivity from pixel buffer.
     */
  uint16_t px_reflectivity(const uint8_t *px_buf) const;

  /**
     * Read signal from pixel buffer.
     *
     * @param[in] px_buf the pixel buffer.
     *
     * @return signal from pixel buffer.
     */
  uint16_t px_signal(const uint8_t *px_buf) const;

  // TODO switch to px_near_ir
  /**
     * Read ambient from pixel buffer.
     *
     * @param[in] px_buf the pixel buffer.
     *
     * @return ambient from pixel buffer.
     */
  uint16_t px_ambient(const uint8_t *px_buf) const;

  // IMU packet accessors
  /**
     * Read sys ts from imu packet buffer.
     * @param[in] imu_buf the imu packet buffer.
     *
     * @return sys ts from imu pacet buffer.
     */
  uint64_t imu_sys_ts(const uint8_t *imu_buf) const;

  /**
     * Read acceleration timestamp.
     *
     * @param[in] imu_buf the imu packet buffer.
     *
     * @return acceleration ts from imu packet buffer.
     */
  uint64_t imu_accel_ts(const uint8_t *imu_buf) const;

  /**
     * Read gyro timestamp.
     *
     * @param[in] imu_buf the imu packet buffer.
     *
     * @return gyro ts from imu packet buffer.
     */
  uint64_t imu_gyro_ts(const uint8_t *imu_buf) const;

  /**
     * Read acceleration in x.
     *
     * @param[in] imu_buf the imu packet buffer.
     *
     * @return acceleration in x.
     */
  float imu_la_x(const uint8_t *imu_buf) const;

  /**
     * Read acceleration in y.
     *
     * @param[in] imu_buf the imu packet buffer.
     *
     * @return acceleration in y.
     */
  float imu_la_y(const uint8_t *imu_buf) const;

  /**
     * Read acceleration in z.
     *
     * @param[in] imu_buf the imu packet buffer.
     *
     * @return acceleration in z.
     */
  float imu_la_z(const uint8_t *imu_buf) const;

  /**
     * Read angular velocity in x.
     *
     * @param[in] imu_buf the imu packet buffer.
     *
     * @return angular velocity in x.
     */
  float imu_av_x(const uint8_t *imu_buf) const;

  /**
     * Read angular velocity in y.
     *
     * @param[in] imu_buf the imu packet buffer.
     *
     * @return angular velocity in y.
     */
  float imu_av_y(const uint8_t *imu_buf) const;

  /**
     * Read angular velocity in z.
     *
     * @param[in] imu_buf the imu packet buffer.
     *
     * @return angular velocity in z.
     */
  float imu_av_z(const uint8_t *imu_buf) const;

  /** Declare get_format as friend. */
  friend const packet_format &get_format(const sensor_info &);
};

/**
 * Get a packet parser for a particular data format.
 *
 * @param[in] info parameters provided by the sensor.
 *
 * @return a packet_format suitable for parsing UDP packets sent by the sensor.
 */
const packet_format &get_format(const sensor_info &info);

Json::Value to_json(const sensor_config &config);

}// namespace OS1

#endif//ROS2_OUSTER__OS1__OS1_TYPES_HPP_