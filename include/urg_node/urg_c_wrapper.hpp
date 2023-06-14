/*
 * Copyright (c) 2013, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * Author: Chad Rockey
 */

#ifndef URG_NODE__URG_C_WRAPPER_HPP_
#define URG_NODE__URG_C_WRAPPER_HPP_

#include <netinet/in.h>
#include <netinet/tcp.h>

#include <chrono>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/multi_echo_laser_scan.hpp"
#include "urg_c/urg_sensor.h"
#include "urg_c/urg_utils.h"

namespace urg_node
{

struct EthernetConnection
{
  std::string ip_address;
  int ip_port;
};

class URGCWrapper
{
public:
  URGCWrapper(
    const EthernetConnection & connection,
    const rclcpp::Logger & logger = rclcpp::get_logger("urg_c_wrapper"),
    bool disable_linger = false);

  ~URGCWrapper();

  bool connect();

  bool reboot();

  bool grabScan(sensor_msgs::msg::LaserScan & msg);

  bool start_measurement();

  bool stop_measurement();

  bool isStarted() const;

  std::string getDeviceID();

  bool setAngleLimitsAndCluster(double & angle_min, double & angle_max, int cluster);

  void setSkip(int skip);

  double getRangeMin() const;

  double getRangeMax() const;

  double getAngleMin() const;

  double getAngleMax() const;

  double getAngleMinLimit() const;

  double getAngleMaxLimit() const;

  double getAngleIncrement() const;

  double getScanPeriod() const;

  double getTimeIncrement() const;

  rclcpp::Duration getAngularTimeOffset() const;

private:
  std::string ip_address_;
  int ip_port_;
  urg_t urg_;
  std::vector<long> data_;                 // NOLINT
  std::vector<unsigned short> intensity_;  // NOLINT
  urg_measurement_type_t measurement_type_;
  int first_step_;
  int last_step_;
  int cluster_;
  int skip_;
  rclcpp::Logger logger_;
  bool disable_linger_;
};
}  // namespace urg_node

#endif  // URG_NODE__URG_C_WRAPPER_HPP_
