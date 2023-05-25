//
// Created by user on 23/05/23.
//

#ifndef ROS2_OUSTER__OS1__OS1_BASE_FACTORY_HPP_
#define ROS2_OUSTER__OS1__OS1_BASE_FACTORY_HPP_

#include "ros2_ouster/OS1/OS1_base_interface.hpp"
#include "ros2_ouster/OS1/OS1_http_impl.hpp"
#include "ros2_ouster/OS1/OS1_tcp_impl.hpp"

namespace OS1
{

/**
     * Creates an instance of the SensorHttp interface.
     *
     * @param[in] hostname hostname of the sensor to communicate with.
     */
inline std::unique_ptr<BaseInterface>
BaseInterface::create(const std::string &hostname)
{
  auto fw = firmware_version(hostname);

  if (fw == invalid_version || fw.major < 2) {
    throw std::runtime_error(
            "SensorHttp:: create firmware version information unavailable or "
            "not fully supported version. Please upgrade your sensor to FW "
            "2.0 or later.");
  }

  if (fw.major == 2) {
    switch (fw.minor) {
      case 0:
        // FW 2.0 doesn't work properly with http
        return std::make_unique<TcpImpl>(hostname);
        //      case 1:
        //        return std::make_unique<SensorHttpImp_2_1>(hostname);
        //      case 2:
        //        return std::make_unique<SensorHttpImp_2_2>(hostname);
    }
  }

  return std::make_unique<HttpImpl>(hostname);
}

}// namespace OS1

#endif//ROS2_OUSTER_OS1_BASE_FACTORY_H
