#include "ros2_ouster/client/client.h"

#include <json/json.h>

#include <algorithm>
#include <chrono>
#include <cstring>
#include <iostream>
#include <stdexcept>
#include <thread>
#include <vector>

#include "ros2_ouster/client/logging.h"
//#include "ouster/build.h"
#include "ros2_ouster/client/impl/netcompat.h"
#include "ros2_ouster/client/client_factory.hpp"

namespace ouster
{
namespace sensor
{

using namespace std::chrono_literals;
namespace chrono = std::chrono;

struct client
{
  SOCKET lidar_fd{};
  SOCKET imu_fd{};
  std::string hostname;
  Json::Value meta;
  ~client()
  {
    impl::socket_close(lidar_fd);
    impl::socket_close(imu_fd);
  }
};

namespace
{

// default udp receive buffer size on windows is very low -- use 256K
const int RCVBUF_SIZE = 256 * 1024;

int32_t get_sock_port(SOCKET sock_fd)
{
  struct sockaddr_storage ss{};
  socklen_t addrlen = sizeof ss;

  if (!impl::socket_valid(
      getsockname(sock_fd, (struct sockaddr *)&ss, &addrlen)))
  {
    logger().error("udp getsockname(): {}", impl::socket_get_error());
    return SOCKET_ERROR;
  }

  if (ss.ss_family == AF_INET) {
    return ntohs(((struct sockaddr_in *)&ss)->sin_port);
  } else if (ss.ss_family == AF_INET6) {
    return ntohs(((struct sockaddr_in6 *)&ss)->sin6_port);
  } else {
    return SOCKET_ERROR;
  }
}

SOCKET udp_data_socket(int port)
{
  struct addrinfo hints{}, * info_start, * ai;

  memset(&hints, 0, sizeof hints);
  hints.ai_family = AF_UNSPEC;
  hints.ai_socktype = SOCK_DGRAM;
  hints.ai_flags = AI_PASSIVE;

  auto port_s = std::to_string(port);

  int ret = getaddrinfo(nullptr, port_s.c_str(), &hints, &info_start);
  if (ret != 0) {
    logger().error("udp getaddrinfo(): {}", gai_strerror(ret));
    return SOCKET_ERROR;
  }
  if (info_start == nullptr) {
    logger().error("udp getaddrinfo(): empty result");
    return SOCKET_ERROR;
  }

  // try to bind a dual-stack ipv6 socket, but fall back to ipv4 only if that
  // fails (when ipv6 is disabled via kernel parameters). Use two passes to
  // deal with glibc addrinfo ordering:
  // https://sourceware.org/bugzilla/show_bug.cgi?id=9981
  for (auto preferred_af : {AF_INET6, AF_INET}) {
    for (ai = info_start; ai != nullptr; ai = ai->ai_next) {
      if (ai->ai_family != preferred_af) continue;

      // choose first addrinfo where bind() succeeds
      SOCKET sock_fd =
              socket(ai->ai_family, ai->ai_socktype, ai->ai_protocol);
      if (!impl::socket_valid(sock_fd)) {
        logger().warn("udp socket(): {}", impl::socket_get_error());
        continue;
      }

      int off = 0;
      if (ai->ai_family == AF_INET6 &&
          setsockopt(sock_fd, IPPROTO_IPV6, IPV6_V6ONLY, (char*)&off,
                     sizeof(off))) {
        logger().warn("udp setsockopt(): {}", impl::socket_get_error());
        impl::socket_close(sock_fd);
        continue;
      }

      if (impl::socket_set_reuse(sock_fd)) {
        logger().warn("udp socket_set_reuse(): {}",
                      impl::socket_get_error());
      }

      if (::bind(sock_fd, ai->ai_addr, (socklen_t)ai->ai_addrlen)) {
        logger().warn("udp bind(): {}", impl::socket_get_error());
        impl::socket_close(sock_fd);
        continue;
      }

      // bind() succeeded; set some options and return
      if (impl::socket_set_non_blocking(sock_fd)) {
        logger().warn("udp fcntl(): {}", impl::socket_get_error());
        impl::socket_close(sock_fd);
        continue;
      }

      if (setsockopt(sock_fd, SOL_SOCKET, SO_RCVBUF, (char*)&RCVBUF_SIZE,
                     sizeof(RCVBUF_SIZE))) {
        logger().warn("udp setsockopt(): {}", impl::socket_get_error());
        impl::socket_close(sock_fd);
        continue;
      }

      freeaddrinfo(info_start);
      return sock_fd;
    }
  }

  // could not bind() a UDP server socket
  freeaddrinfo(info_start);
  logger().error("failed to bind udp socket");
  return SOCKET_ERROR;
}

Json::Value collect_metadata(const std::string& hostname, int timeout_sec) {
  auto net_client = util::ClientInterface::create(hostname);
  auto timeout_time =
          chrono::steady_clock::now() + chrono::seconds{timeout_sec};
  std::string status;

  do {
    if (chrono::steady_clock::now() >= timeout_time) return false;
    std::this_thread::sleep_for(1s);
    status = net_client->sensor_info()["status"].asString();
  } while (status == "INITIALIZING");

  // not all metadata available when sensor isn't RUNNING
  if (status != "RUNNING") {
    throw std::runtime_error(
            "Cannot obtain full metadata with sensor status: " + status +
            ". Please ensure that sensor is not in a STANDBY, UNCONFIGURED, "
            "WARMUP, or ERROR state");
  }

  auto metadata = net_client->metadata();
  // merge extra info into metadata
//  metadata["client_version"] = client_version();
  return metadata;
}

}  // namespace

bool get_config(
  const std::string& hostname,
  sensor_config& config,
  bool active)
{
  auto net_client = util::ClientInterface::create(hostname);
  auto res = net_client->get_config_params(active);
  config = parse_config(res);
  return true;
}

bool set_config(
  const std::string& hostname,
  const sensor_config& config,
  uint8_t config_flags)
{
  auto net_client = util::ClientInterface::create(hostname);

  // reset staged config to avoid spurious errors
  auto config_params = net_client->active_config_params();
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
      net_client->set_udp_dest_auto();

      auto staged = net_client->staged_config_params();

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
      net_client->set_config_param(".", config_params_str);
      // reinitialize to make all staged parameters effective
      net_client->reinitialize();
  }

  // save if indicated
  if (config_flags & CONFIG_PERSIST) { net_client->save_config_params(); }

  return true;
}

std::string get_metadata(client& cli, int timeout_sec, bool legacy_format) {
  try {
      cli.meta = collect_metadata(cli.hostname, timeout_sec);
  } catch (const std::exception& e) {
      logger().warn(std::string("Unable to retrieve sensor metadata: ") + e.what());
      throw;
  }

  Json::StreamWriterBuilder builder;
  builder["enableYAMLCompatibility"] = true;
  builder["precision"] = 6;
  builder["indentation"] = "    ";
  auto metadata_string = Json::writeString(builder, cli.meta);
  if (legacy_format) {
      logger().warn(
              "The SDK will soon output the non-legacy metadata format by "
              "default.  If you parse the metadata directly instead of using the "
              "SDK (which will continue to read both legacy and non-legacy "
              "formats), please be advised that on the next release you will "
              "either have to update your parsing or specify legacy_format = "
              "true to the get_metadata function.");
  }

  // We can't insert this logic into the light init_client since its advantage
  // is that it doesn't make netowrk calls but we need it to run every time
  // there is a valid connection to the sensor So we insert it here
  // TODO: remove after release of FW 3.2/3.3 (sufficient warning)
  sensor_config config;
  get_config(cli.hostname, config);
  auto fw_version = util::ClientInterface::firmware_version(cli.hostname);
  // only warn for people on the latest FW, as people on older FWs may not
  // care
  if (fw_version.major >= 3 &&
      config.udp_profile_lidar == UDPProfileLidar::PROFILE_LIDAR_LEGACY) {
      logger().warn(
              "Please note that the Legacy Lidar Profile will be deprecated "
              "in the sensor FW soon. If you plan to upgrade your FW, we "
              "recommend using the Single Return Profile instead. For users "
              "sticking with older FWs, the Ouster SDK will continue to parse "
              "the legacy lidar profile.");
  }
  return legacy_format ? convert_to_legacy(metadata_string) : metadata_string;
}

std::shared_ptr<client> init_client(
  const std::string & hostname, int lidar_port,
  int imu_port)
{
  logger().info("initializing sensor: {} with lidar port/imu port: {}/{}",
                hostname, lidar_port, imu_port);

  auto cli = std::make_shared<client>();
  cli->hostname = hostname;

  cli->lidar_fd = udp_data_socket(lidar_port);
  cli->imu_fd = udp_data_socket(imu_port);

  if (!impl::socket_valid(cli->lidar_fd) || !impl::socket_valid(cli->imu_fd)) {
    return {};
  }

  return cli;
}

std::shared_ptr<client> init_client(
  const std::string & hostname,
  const std::string & udp_dest_host,
  lidar_mode ld_mode, timestamp_mode ts_mode,
  int lidar_port, int imu_port,
  int timeout_sec)
{
  auto cli = init_client(hostname, lidar_port, imu_port);
  if (!cli) {return {};}

  // update requested ports to actual bound ports
  lidar_port = get_sock_port(cli->lidar_fd);
  imu_port = get_sock_port(cli->imu_fd);
  if (!impl::socket_valid(lidar_port) || !impl::socket_valid(imu_port)) {
    return {};
  }

  try {
    sensor::sensor_config config;
    uint8_t config_flags = 0;
    if (udp_dest_host.empty())
          config_flags |= CONFIG_UDP_DEST_AUTO;
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
    if (status == "ERROR" || status == "UNCONFIGURED")
          return {};
  } catch (const std::runtime_error& e) {
    // log error message
    logger().error("init_client(): {}", e.what());
    return {};
  }

  return cli;
}

client_state poll_client(const client & c, const int timeout_sec)
{
  fd_set rfds;
  FD_ZERO(&rfds);
  FD_SET(c.lidar_fd, &rfds);
  FD_SET(c.imu_fd, &rfds);

  timeval tv{};
  tv.tv_sec = timeout_sec;
  tv.tv_usec = 0;

  SOCKET max_fd = std::max(c.lidar_fd, c.imu_fd);

  SOCKET retval = select((int)max_fd + 1, &rfds, nullptr, nullptr, &tv);

  auto res = client_state(0);

  if (!impl::socket_valid(retval) && impl::socket_exit()) {
    res = EXIT;
  } else if (!impl::socket_valid(retval)) {
    std::cerr << "select: " << impl::socket_get_error() << std::endl;
    res = client_state(res | CLIENT_ERROR);
  } else if (retval) {
    if (FD_ISSET(c.lidar_fd, &rfds)) {res = client_state(res | LIDAR_DATA);}
    if (FD_ISSET(c.imu_fd, &rfds)) {res = client_state(res | IMU_DATA);}
  }

  return res;
}

static bool recv_fixed(SOCKET fd, void * buf, int64_t len)
{
  // Have to read longer than len because you need to know if the packet is
  // too large
  int64_t bytes_read = recv(fd, (char*)buf, len + 1, 0);

  if (bytes_read == len) {
    return true;
  } else if (bytes_read == -1) {
    logger().error("recvfrom: {}", impl::socket_get_error());
  } else {
    logger().warn("Unexpected udp packet length: {}, expected: {}", bytes_read, len);
  }
  return false;
}

bool read_lidar_packet(
  const client & cli, uint8_t * buf,
  const packet_format & pf)
{
  return recv_fixed(cli.lidar_fd, buf, pf.lidar_packet_size);
}

bool read_imu_packet(const client & cli, uint8_t * buf, const packet_format & pf)
{
  return recv_fixed(cli.imu_fd, buf, pf.imu_packet_size);
}

int get_lidar_port(client& cli) { return get_sock_port(cli.lidar_fd); }

int get_imu_port(client& cli) { return get_sock_port(cli.imu_fd); }

/**
 * Return the socket file descriptor used to listen for lidar UDP data.
 *
 * @param[in] cli client returned by init_client associated with the connection.
 *
 * @return the socket file descriptor.
 */
extern SOCKET get_lidar_socket_fd(client& cli) { return cli.lidar_fd; }

/**
 * Return the socket file descriptor used to listen for imu UDP data.
 *
 * @param[in] cli client returned by init_client associated with the connection.
 *
 * @return the socket file descriptor.
 */
extern SOCKET get_imu_socket_fd(client& cli) { return cli.imu_fd; }

}  // namespace sensor
}  // namespace ouster