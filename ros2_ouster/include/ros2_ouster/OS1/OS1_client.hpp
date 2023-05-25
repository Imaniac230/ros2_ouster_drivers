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

#ifndef ROS2_OUSTER__OS1__OS1_CLIENT_HPP_
#define ROS2_OUSTER__OS1__OS1_CLIENT_HPP_

#include <fcntl.h>
#include <netdb.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "arpa/inet.h"
#include "sys/socket.h"
#include "sys/types.h"

#include "ros2_ouster/OS1/OS1_base_factory.hpp"
#include "ros2_ouster/OS1/OS1_base_interface.hpp"
#include "ros2_ouster/OS1/OS1_util.hpp"
#include "ros2_ouster/interfaces/common.hpp"
#include "json/json.h"

namespace OS1
{

struct client {
  int lidar_fd{7502};
  int imu_fd{7503};
  std::string hostname;
  Json::Value meta;
  ~client()
  {
    close(lidar_fd);
    close(imu_fd);
  }
};

/**
 * Minimum supported version
 */
//const version min_version = {1, 9, 0};
const version min_version = {1, 12, 0};

/** Returned by poll_client. */
enum client_state
{
  TIMEOUT = 0,     ///< Client has timed out
  CLIENT_ERROR = 1,///< Client has reported an error
  LIDAR_DATA = 2,  ///< New lidar data available
  IMU_DATA = 4,    ///< New IMU data available
  EXIT = 8         ///< Client has exited
};

/**
 * Flags for set_config()
 */
enum config_flags : uint8_t
{
  CONFIG_UDP_DEST_AUTO = (1 << 0),///< Set udp_dest automatically
  CONFIG_PERSIST = (1 << 1),      ///< Make configuration persistent
  CONFIG_FORCE_REINIT = (1 << 2)  ///< Forces the sensor to re-init during
                                  ///< set_config even when config params
                                  ///< have not changed
};

/**
 * Set sensor config on sensor.
 *
 * @throw runtime_error on failure to communcate with the sensor.
 * @throw invalid_argument when config parameters fail validation.
 *
 * @param[in] hostname sensor hostname.
 * @param[in] config sensor config.
 * @param[in] config_flags flags to pass in.
 *
 * @return true if config params successfuly set on sensor.
 */
inline bool set_config(const std::string &hostname, const sensor_config &config,
                       uint8_t config_flags = 0)
{
  auto sensor_http = BaseInterface::create(hostname);

  // reset staged config to avoid spurious errors
  auto config_params = sensor_http->active_config_params();
  Json::Value config_params_copy = config_params;

  // set all desired config parameters
  Json::Value config_json = to_json(config);
  for (const auto &key: config_json.getMemberNames()) {
    config_params[key] = config_json[key];
  }

  if (config_json.isMember("operating_mode") &&
      config_params.isMember("auto_start_flag")) {
    // we're setting operating mode and this sensor has a FW with
    // auto_start_flag
    config_params["auto_start_flag"] =
            config_json["operating_mode"] == "NORMAL" ? 1 : 0;
  }

  // Signal multiplier changed from int to double for FW 3.0/2.5+, with
  // corresponding change to config.signal_multiplier.
  // Change values 1, 2, 3 back to ints to support older FWs
  if (config_json.isMember("signal_multiplier")) {
    check_signal_multiplier(config_params["signal_multiplier"].asDouble());
    if (config_params["signal_multiplier"].asDouble() != 0.25 &&
        config_params["signal_multiplier"].asDouble() != 0.5) {
      config_params["signal_multiplier"] =
              config_params["signal_multiplier"].asInt();
    }
  }

  // set automatic udp dest, if flag specified
  if (config_flags & CONFIG_UDP_DEST_AUTO) {
    if (config.udp_dest)
      throw std::invalid_argument(
              "UDP_DEST_AUTO flag set but provided config has udp_dest");
    sensor_http->set_udp_dest_auto();

    auto staged = sensor_http->staged_config_params();

    // now we set config_params according to the staged udp_dest from the
    // sensor
    if (staged.isMember("udp_ip")) {// means the FW version carries udp_ip
      config_params["udp_ip"] = staged["udp_ip"];
      config_params["udp_dest"] = staged["udp_ip"];
    }
    else {// don't need to worry about udp_ip
      config_params["udp_dest"] = staged["udp_dest"];
    }
  }

  // if configuration didn't change then skip applying the params
  // note: comparison will fail if config_params contains newer config params
  // introduced after the verison of FW the sensor is on
  if (config_flags & CONFIG_FORCE_REINIT ||
      config_params_copy != config_params) {
    Json::StreamWriterBuilder builder;
    builder["indentation"] = "";
    // send full string -- depends on older FWs not rejecting a blob even
    // when it contains unknown keys
    auto config_params_str = Json::writeString(builder, config_params);
    sensor_http->set_config_param(".", config_params_str);
    // reinitialize to make all staged parameters effective
    sensor_http->reinitialize();
  }

  // save if indicated
  if (config_flags & CONFIG_PERSIST) { sensor_http->save_config_params(); }

  return true;
}

/**
 * Connect to and configure the sensor and start listening for data
 * @param port port on which the sensor will receive lidar data
 * @return int of socket address
 */
//TODO(udp_socket): decide if porting the full ipv6/ipv4 ouster_ros impl makes sense
inline int udp_data_socket(int port)
{
  struct addrinfo hints {
  }, *info_start, *ai;

  memset(&hints, 0, sizeof hints);
  hints.ai_family = AF_INET6;
  hints.ai_socktype = SOCK_DGRAM;
  hints.ai_flags = AI_PASSIVE;

  auto port_s = std::to_string(port);

  int ret = getaddrinfo(nullptr, port_s.c_str(), &hints, &info_start);
  if (ret != 0) {
    std::cerr << "getaddrinfo(): " << gai_strerror(ret) << std::endl;
    return -1;
  }
  if (info_start == nullptr) {
    std::cerr << "getaddrinfo: empty result" << std::endl;
    return -1;
  }

  int sock_fd;
  for (ai = info_start; ai != nullptr; ai = ai->ai_next) {
    sock_fd = socket(ai->ai_family, ai->ai_socktype, ai->ai_protocol);
    if (sock_fd < 0) {
      std::cerr << "udp socket(): " << std::strerror(errno) << std::endl;
      continue;
    }

    if (bind(sock_fd, ai->ai_addr, ai->ai_addrlen) < 0) {
      close(sock_fd);
      std::cerr << "udp bind(): " << std::strerror(errno) << std::endl;
      continue;
    }

    break;
  }

  freeaddrinfo(info_start);
  if (ai == nullptr) {
    close(sock_fd);
    return -1;
  }

  if (fcntl(sock_fd, F_SETFL, fcntl(sock_fd, F_GETFL, 0) | O_NONBLOCK) < 0) {
    std::cerr << "udp fcntl(): " << std::strerror(errno) << std::endl;
    return -1;
  }

  return sock_fd;
}

inline Json::Value collect_metadata(const std::string &hostname,
                                    const int timeout_sec)
{
  auto sensor_http = BaseInterface::create(hostname);
  auto timeout_time =
          std::chrono::steady_clock::now() + std::chrono::seconds{timeout_sec};
  std::string status;

  // TODO: can remove this loop when we drop support for FW 2.4
  do {
    if (std::chrono::steady_clock::now() >= timeout_time) return false;
    std::this_thread::sleep_for(std::chrono::seconds(1));
    status = sensor_http->sensor_info()["status"].asString();
  } while (status == "INITIALIZING");

  // not all metadata available when sensor isn't RUNNING
  if (status != "RUNNING") {
    throw std::runtime_error(
            "Cannot obtain full metadata with sensor status: " + status +
            ". Please ensure that sensor is not in a STANDBY, UNCONFIGURED, "
            "WARMUP, or ERROR state");
  }

  auto metadata = sensor_http->metadata();
  // merge extra info into metadata
  metadata["client_version"] = client_version();
  return metadata;
}

/**
 * Connect to and configure the sensor and start listening for data
 * @param lidar_port port on which the sensor will send lidar data
 * @param imu_port port on which the sensor will send imu data
 * @return pointer owning the resources associated with the connection
 */
inline std::shared_ptr<client> init_client(const std::string &hostname,
                                           const int lidar_port,
                                           const int imu_port)
{
  std::cout << "initializing sensor: " << hostname
            << " with lidar port/imu port: " << lidar_port << "/" << imu_port
            << "" << std::endl;

  auto cli = std::make_shared<client>();
  cli->hostname = hostname;
  cli->lidar_fd = udp_data_socket(lidar_port);
  cli->imu_fd = udp_data_socket(imu_port);

  if ((cli->lidar_fd < 0) || (cli->imu_fd < 0)) return {};

  return cli;
}

/**
 * Connect to and configure the sensor and start listening for data
 * @param hostname hostname or ip of the sensor
 * @param udp_dest_host hostname or ip where the sensor should send data
 * @param mode lidar_mode defining azimuth resolution and rotation rate
 * @param ts_mode method used to timestamp lidar measurements
 * @param lidar_port port on which the sensor will send lidar data
 * @param imu_port port on which the sensor will send imu data
 * @return pointer owning the resources associated with the connection
 */
inline std::shared_ptr<client>
init_client(const std::string &hostname, const std::string &udp_dest_host,
            lidar_mode ld_mode = MODE_1024x10,
            timestamp_mode ts_mode = TIME_FROM_INTERNAL_OSC,
            int lidar_port = 7502, int imu_port = 7503, int timeout_sec = 60)
{
  auto cli = init_client(hostname, lidar_port, imu_port);
  if (!cli) return {};

  try {
    OS1::sensor_config config;
    uint8_t config_flags = 0;
    if (udp_dest_host.empty()) config_flags |= CONFIG_UDP_DEST_AUTO;
    else
      config.udp_dest = udp_dest_host;
    if (ld_mode) config.ld_mode = ld_mode;
    if (ts_mode) config.ts_mode = ts_mode;
    if (lidar_port) config.udp_port_lidar = lidar_port;
    if (imu_port) config.udp_port_imu = imu_port;
    config.operating_mode = OPERATING_NORMAL;
    set_config(hostname, config, config_flags);

    // will block until no longer INITIALIZING
    cli->meta = collect_metadata(hostname, timeout_sec);
    // check for sensor error states
    auto status = cli->meta["sensor_info"]["status"].asString();
    if (status == "ERROR" || status == "UNCONFIGURED") return {};
  }
  catch (const std::runtime_error &e) {
    // log error message
    std::cerr << "init_client(): " << e.what() << std::endl;
    return {};
  }

  return cli;
}


/**
 * Block for up to timeout_sec until either data is ready or an error occurs.
 *
 * NOTE: will return immediately if LIDAR_DATA or IMU_DATA are set and not
 * cleared by read_lidar_data() and read_imu_data() before the next call.
 *
 * @param[in] cli client returned by init_client associated with the connection.
 * @param[in] timeout_sec seconds to block while waiting for data.
 *
 * @return client_state s where (s & ERROR) is true if an error occured, (s &
 * LIDAR_DATA) is true if lidar data is ready to read, and (s & IMU_DATA) is
 * true if imu data is ready to read.
 */
inline client_state poll_client(const client &cli, const int timeout_sec = 1)
{
  fd_set rfds;
  FD_ZERO(&rfds);
  FD_SET(cli.lidar_fd, &rfds);
  FD_SET(cli.imu_fd, &rfds);

  timeval tv{};
  tv.tv_sec = timeout_sec;
  tv.tv_usec = 0;

  int max_fd = std::max(cli.lidar_fd, cli.imu_fd);

  int retval =
          select(static_cast<int>(max_fd) + 1, &rfds, nullptr, nullptr, &tv);

  if (retval == -1 && errno == EINTR) return client_state::EXIT;
  if (retval == -1) {
    std::cerr << "select: " << std::strerror(errno) << std::endl;
    return client_state::CLIENT_ERROR;
  }

  //TODO(enum-construct): copied from sdk, test if this actually does what it is expected to do,
  //  this driver implementation does not process both lidar and imu in one shot (uses switch instead of if)
  auto res = client_state::TIMEOUT;
  if (retval) {
    if (FD_ISSET(cli.lidar_fd, &rfds))
      res = client_state(res | client_state::LIDAR_DATA);
    if (FD_ISSET(cli.imu_fd, &rfds))
      res = client_state(res | client_state::IMU_DATA);
  }

  return res;
}

/**
 * Read lidar data from the sensor. Will not block.
 * @param fd Socket connection for sensor data
 * @param buf buffer to which to write lidar data. Must be at least
 * lidar_packet_bytes + 1 bytes
 * @param len length of packet
 * @return true if a lidar packet was successfully read
 */
static bool recv_fixed(int fd, void *buf, size_t len)
{
  // Have to read longer than len because you need to know if the packet is
  // too large
  ssize_t bytes_read = recv(fd, buf, len + 1, 0);
  if (bytes_read == static_cast<ssize_t>(len)) { return true; }
  else if (bytes_read == -1) {
    std::cerr << "recvfrom: " << std::strerror(errno) << std::endl;
  }
  else {
    std::cerr << "Unexpected udp packet length: " << bytes_read << std::endl;
  }
  return false;
}

/**
 * Read lidar data from the sensor. Will not block.
 *
 * @param[in] cli client returned by init_client associated with the connection.
 * @param[out] buf buffer to which to write lidar data. Must be at least
 * lidar_packet_bytes + 1 bytes.
 * @param[in] pf The packet format.
 *
 * @return true if a lidar packet was successfully read.
 */
inline bool read_lidar_packet(const client &cli, uint8_t *buf,
                              const packet_format &pf)
{
  return recv_fixed(cli.lidar_fd, buf, pf.lidar_packet_size);
}

/**
 * Read imu data from the sensor. Will not block.
 *
 * @param[in] cli client returned by init_client associated with the connection.
 * @param[out] buf buffer to which to write imu data. Must be at least
 * imu_packet_bytes + 1 bytes.
 * @param[in] pf The packet format.
 *
 * @return true if an imu packet was successfully read.
 */
inline bool read_imu_packet(const client &cli, uint8_t *buf,
                            const packet_format &pf)
{
  return recv_fixed(cli.imu_fd, buf, pf.imu_packet_size);
}

/**
 * Get sensor config from the sensor.
 *
 * Populates passed in config with the results of get_config.
 *
 * @param[in] hostname sensor hostname.
 * @param[out] config sensor config to populate.
 * @param[in] active whether to pull active or passive configs.
 *
 * @return true if sensor config successfully populated.
 */
inline bool get_config(const std::string &hostname, sensor_config &config,
                       bool active = true)
{
  auto sensor_http = BaseInterface::create(hostname);
  auto res = sensor_http->get_config_params(active);
  config = parse_config(res);
  return true;
}

/**
 * Get metadata text blob from the sensor.
 *
 * Will attempt to fetch from the network if not already populated.
 *
 * @throw runtime_error if the sensor is in ERROR state, the firmware version
 * used to initialize the HTTP or TCP client is invalid, the metadata could
 * not be retrieved from the sensor, or the response could not be parsed.
 *
 * @param[in] cli client returned by init_client associated with the connection.
 * @param[in] timeout_sec how long to wait for the sensor to initialize.
 * @param[in] legacy_format whether to use legacy format of metadata output.
 *
 * @return a text blob of metadata parseable into a sensor_info struct.
 */
inline std::string get_metadata(client &cli, const int timeout_sec = 60,
                                const bool legacy_format = false)
{
  try {
    cli.meta = collect_metadata(cli.hostname, timeout_sec);
  }
  catch (const std::exception &e) {
    std::cerr << "Unable to retrieve sensor metadata: " << e.what()
              << std::endl;
    throw;
  }

  Json::StreamWriterBuilder builder;
  builder["enableYAMLCompatibility"] = true;
  builder["precision"] = 6;
  builder["indentation"] = "    ";
  auto metadata_string = Json::writeString(builder, cli.meta);
  if (legacy_format) {
    std::cout
            << "The SDK will soon output the non-legacy metadata format by "
               "default.  If you parse the metadata directly instead of using "
               "the "
               "SDK (which will continue to read both legacy and non-legacy "
               "formats), please be advised that on the next release you will "
               "either have to update your parsing or specify legacy_format = "
               "true to the get_metadata function."
            << std::endl;
  }

  // We can't insert this logic into the light init_client since its advantage
  // is that it doesn't make netowrk calls but we need it to run every time
  // there is a valid connection to the sensor So we insert it here
  // TODO: remove after release of FW 3.2/3.3 (sufficient warning)
  sensor_config config;
  get_config(cli.hostname, config);
  auto fw_version = BaseInterface::firmware_version(cli.hostname);
  // only warn for people on the latest FW, as people on older FWs may not
  // care
  if (fw_version.major >= 3 &&
      config.udp_profile_lidar == UDPProfileLidar::PROFILE_LIDAR_LEGACY) {
    std::cout
            << "Please note that the Legacy Lidar Profile will be deprecated "
               "in the sensor FW soon. If you plan to upgrade your FW, we "
               "recommend using the Single Return Profile instead. For users "
               "sticking with older FWs, the Ouster SDK will continue to parse "
               "the legacy lidar profile."
            << std::endl;
  }
  return legacy_format ? convert_to_legacy(metadata_string) : metadata_string;
}

}// namespace OS1

#endif// ROS2_OUSTER__OS1__OS1_CLIENT_HPP_
