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
 * Author: Chad Rockey, Mike O'Driscoll
 */

#include <chrono>
#include <cinttypes>
#include <limits>
#include <memory>
#include <string>
#include <urg_node/urg_c_wrapper.hpp>
#include <vector>

#include "boost/crc.hpp"

namespace urg_node
{

URGCWrapper::URGCWrapper(
  const EthernetConnection & connection, const rclcpp::Logger & logger, bool disable_linger)
: ip_address_(connection.ip_address),
  ip_port_(connection.ip_port),
  logger_(logger),
  disable_linger_(disable_linger)
{
  RCLCPP_INFO(logger_, "Created Ethernet URGCWrapper at %s:%d", ip_address_.c_str(), ip_port_);
}

bool URGCWrapper::connect()
{
  int result = 0;
  if (!ip_address_.empty()) {
    RCLCPP_INFO(logger_, "Connecting to Hokuyo at %s:%d", ip_address_.c_str(), ip_port_);
    result = urg_open(&urg_, URG_ETHERNET, ip_address_.c_str(), ip_port_);
  } else {
    RCLCPP_ERROR(logger_, "No connection information provided");
    return false;
  }

  if (result < 0) {
    std::string error(urg_error(&urg_));
    RCLCPP_ERROR(logger_, "Could connect to Hokuyo: %s", error.c_str());
    return false;
  }

  int urg_data_size = urg_max_data_size(&urg_);

  if (urg_data_size < 0) {
    std::string error(urg_error(&urg_));
    RCLCPP_ERROR(logger_, "Could not initialise: %s", error.c_str());
    return false;
  }

  if (urg_data_size > 5000) {
    urg_data_size = 5000;
  }
  data_.resize(urg_data_size * URG_MAX_ECHO);
  intensity_.resize(urg_data_size * URG_MAX_ECHO);

  first_step_ = 0;
  last_step_ = 0;
  cluster_ = 1;
  skip_ = 0;

  measurement_type_ = URG_DISTANCE_INTENSITY;
  return true;
}

bool URGCWrapper::start_measurement()
{
  int result = urg_start_measurement(&urg_, measurement_type_, 0, skip_);
  if (result < 0) {
    std::string error(urg_error(&urg_));
    RCLCPP_ERROR(logger_, "Could not start measurement: %s", error.c_str());
    return false;
  }
  return true;
}

bool URGCWrapper::stop_measurement()
{
  int result = urg_stop_measurement(&urg_);
  if (result < 0) {
    std::string error(urg_error(&urg_));
    RCLCPP_ERROR(logger_, "Could not stop measurement: %s", error.c_str());
    return false;
  }
  return true;
}

URGCWrapper::~URGCWrapper()
{
  RCLCPP_INFO(logger_, "URGCWrapper destructor called.");
  int sock = urg_.connection.tcpclient.sock_desc;
  if (sock != -1) {
    if (disable_linger_) {
      // Disable SO_LINGER option
      struct linger linger_opt;
      linger_opt.l_onoff = 1;   // Disable SO_LINGER
      linger_opt.l_linger = 0;  // Not used when l_onoff is 0

      if (setsockopt(sock, SOL_SOCKET, SO_LINGER, &linger_opt, sizeof(linger_opt)) == -1) {
        RCLCPP_ERROR(logger_, "Could not set SO_LINGER off on socket: %s", strerror(errno));
      } else {
        RCLCPP_INFO(logger_, "Disabled SO_LINGER");
      }
    }
    // TODO(richardw347): This is a bit exterme to always ensure the
    // socket is closed on destruction. However this is necessary
    // at the moment to ensure the sensor can alawys be restarted
    // stop();
    // urg_close(&urg_);
    RCLCPP_INFO(logger_, "Closing socket.");
    close(sock);
  } else {
    RCLCPP_INFO(logger_, "Socket already closed.");
  }
}

bool URGCWrapper::reboot()
{
  if (urg_reboot(&urg_) != 0) {
    RCLCPP_ERROR(logger_, "Could not reboot Hokuyo: %s", urg_error(&urg_));
    return false;
  }
  RCLCPP_ERROR(logger_, "Rebooting Hokuyo");
  return true;
}

bool URGCWrapper::grabScan(sensor_msgs::msg::LaserScan & msg)
{
  // Grab scan
  long time_stamp = 0;                       // NOLINT
  unsigned long long system_time_stamp = 0;  // NOLINT

  int num_beams =
    urg_get_distance_intensity(&urg_, &data_[0], &intensity_[0], &time_stamp, &system_time_stamp);

  if (num_beams <= 0) {
    std::string error(urg_error(&urg_));
    RCLCPP_WARN(logger_, "Error grabbing scan: %s streaming data stopped", error.c_str());
    return false;
  }

  // Fill scan
  builtin_interfaces::msg::Time stampTime =
    rclcpp::Time(static_cast<int64_t>(system_time_stamp)) + getAngularTimeOffset();
  msg.header.stamp = stampTime;
  msg.ranges.resize(num_beams);
  msg.intensities.resize(num_beams);

  for (int i = 0; i < num_beams; i++) {
    if (data_[(i) + 0] != 0) {
      msg.ranges[i] = static_cast<float>(data_[i]) / 1000.0;
      msg.intensities[i] = intensity_[i];
    } else {
      msg.ranges[i] = std::numeric_limits<float>::quiet_NaN();
      continue;
    }
  }
  return true;
}

bool URGCWrapper::isStarted() const {return urg_.is_laser_on == 1;}

double URGCWrapper::getRangeMin() const
{
  long minr;  // NOLINT
  long maxr;  // NOLINT
  urg_distance_min_max(&urg_, &minr, &maxr);
  return static_cast<double>(minr) / 1000.0;
}

double URGCWrapper::getRangeMax() const
{
  long minr;  // NOLINT
  long maxr;  // NOLINT
  urg_distance_min_max(&urg_, &minr, &maxr);
  return static_cast<double>(maxr) / 1000.0;
}

double URGCWrapper::getAngleMin() const {return urg_step2rad(&urg_, first_step_);}

double URGCWrapper::getAngleMax() const {return urg_step2rad(&urg_, last_step_);}

double URGCWrapper::getAngleMinLimit() const
{
  int min_step;
  int max_step;
  urg_step_min_max(&urg_, &min_step, &max_step);
  return urg_step2rad(&urg_, min_step);
}

double URGCWrapper::getAngleMaxLimit() const
{
  int min_step;
  int max_step;
  urg_step_min_max(&urg_, &min_step, &max_step);
  return urg_step2rad(&urg_, max_step);
}

double URGCWrapper::getAngleIncrement() const
{
  double angle_min = getAngleMin();
  double angle_max = getAngleMax();
  return cluster_ * (angle_max - angle_min) / static_cast<double>(last_step_ - first_step_);
}

double URGCWrapper::getScanPeriod() const
{
  long scan_usec = urg_scan_usec(&urg_);  // NOLINT
  return 1.e-6 * static_cast<double>(scan_usec);
}

double URGCWrapper::getTimeIncrement() const
{
  int min_step;
  int max_step;
  urg_step_min_max(&urg_, &min_step, &max_step);
  double scan_period = getScanPeriod();
  double circle_fraction = (getAngleMaxLimit() - getAngleMinLimit()) / (2.0 * 3.141592);
  return cluster_ * circle_fraction * scan_period / static_cast<double>(max_step - min_step);
}

std::string URGCWrapper::getDeviceID() {return std::string(urg_sensor_serial_id(&urg_));}

// Must be called before urg_start
bool URGCWrapper::setAngleLimitsAndCluster(double & angle_min, double & angle_max, int cluster)
{
  // Set step limits
  first_step_ = urg_rad2step(&urg_, angle_min);
  last_step_ = urg_rad2step(&urg_, angle_max);
  cluster_ = cluster;

  // Make sure step limits are not the same
  if (first_step_ == last_step_) {
    // Make sure we're not at a limit
    int min_step;
    int max_step;
    urg_step_min_max(&urg_, &min_step, &max_step);
    if (first_step_ == min_step) {  // At beginning of range
      last_step_ = first_step_ + 1;
    } else {  // At end of range (or all other cases)
      first_step_ = last_step_ - 1;
    }
  }

  // Make sure angle_max is greater than angle_min (should check this after end limits)
  if (last_step_ < first_step_) {
    double temp = first_step_;
    first_step_ = last_step_;
    last_step_ = temp;
  }

  angle_min = urg_step2rad(&urg_, first_step_);
  angle_max = urg_step2rad(&urg_, last_step_);
  int result = urg_set_scanning_parameter(&urg_, first_step_, last_step_, cluster);
  if (result < 0) {
    return false;
  }
  return true;
}

rclcpp::Duration URGCWrapper::getAngularTimeOffset() const
{
  // Adjust value for Hokuyo's timestamps
  // Hokuyo's timestamps start from the rear center of the device (at Pi according to ROS standards)
  double circle_fraction = 0.0;
  if (first_step_ == 0 && last_step_ == 0) {
    circle_fraction = (getAngleMinLimit() + 3.141592) / (2.0 * 3.141592);
  } else {
    circle_fraction = (getAngleMin() + 3.141592) / (2.0 * 3.141592);
  }
  return rclcpp::Duration(std::chrono::duration<double>(circle_fraction * getScanPeriod()));
}

void URGCWrapper::setSkip(int skip) {skip_ = skip;}

}  // namespace urg_node
