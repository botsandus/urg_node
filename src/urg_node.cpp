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
 * Author: Chad Rockey, Michael Carroll, Mike O'Driscoll
 */

#include "urg_node/urg_node.hpp"

#include <memory>
#include <string>
#include <vector>

namespace urg_node
{

UrgNode::UrgNode(const rclcpp::NodeOptions & node_options)
: Node("urg_node", node_options),
  error_count_(0),
  error_limit_(4),
  close_scan_(true),
  ip_address_(""),
  ip_port_(10940),
  angle_min_(-3.14),
  angle_max_(3.14),
  cluster_(1),
  skip_(0),
  laser_frame_id_("laser"),
  reconn_delay_(0.5),
  disable_linger_(false),
  error_reset_period_(3.0),
  last_error_(0)
{
  // Declare parameters so we can change these later.
  ip_address_ = this->declare_parameter<std::string>("ip_address", ip_address_);
  ip_port_ = this->declare_parameter<int>("ip_port", ip_port_);
  laser_frame_id_ = this->declare_parameter<std::string>("laser_frame_id", laser_frame_id_);
  error_limit_ = this->declare_parameter<int>("error_limit", error_limit_);
  angle_min_ = this->declare_parameter<double>("angle_min", angle_min_);
  angle_max_ = this->declare_parameter<double>("angle_max", angle_max_);
  skip_ = this->declare_parameter<int>("skip", skip_);
  cluster_ = this->declare_parameter<int>("cluster", cluster_);
  reconn_delay_ = declare_parameter<double>("reconnect_delay", reconn_delay_);
  disable_linger_ = declare_parameter<bool>("disable_linger", disable_linger_);
  error_reset_period_ = declare_parameter<double>("error_reset_period", error_reset_period_);

  last_error_ = this->now();

  laser_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("scan", 20);

  reboot_triggered_ = false;
  reboot_service_ = this->create_service<std_srvs::srv::Trigger>(
    std::string(get_name()) + "/reboot_lidar",
    std::bind(
      &UrgNode::rebootCallback, this, std::placeholders::_1, std::placeholders::_2,
      std::placeholders::_3));

  // Put this in a separate thread since it might take a while to connect
  run_thread_ = std::thread(std::bind(&UrgNode::run, this));

  scan_.header.frame_id = laser_frame_id_;
}

UrgNode::~UrgNode()
{
  if (run_thread_.joinable()) {
    run_thread_.join();
  }
  if (scan_thread_.joinable()) {
    close_scan_ = true;
    scan_thread_.join();
  }
}

void UrgNode::rebootCallback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std_srvs::srv::Trigger::Request::SharedPtr req,
  const std_srvs::srv::Trigger::Response::SharedPtr res)
{
  (void)request_header;
  (void)req;
  reboot_triggered_ = true;
  res->success = true;
}

bool UrgNode::connect()
{
  try {
    urg_.reset();  // Clear any previous connections();
    EthernetConnection connection{ip_address_, ip_port_};
    urg_.reset(new urg_node::URGCWrapper(connection, this->get_logger(), disable_linger_));

    if (!urg_->connect()) {
      urg_.reset();
      return false;
    }
    RCLCPP_INFO(get_logger(), "Connected to Hokuyo with ID: %s", urg_->getDeviceID().c_str());

    // The publish frequency changes based on the number of skipped scans.
    // Update accordingly here.
    freq_min_ = 1.0 / (urg_->getScanPeriod() * (skip_ + 1));

    urg_->setAngleLimitsAndCluster(angle_min_, angle_max_, cluster_);
    urg_->setSkip(skip_);

    scan_.angle_min = angle_min_;
    scan_.angle_max = angle_max_;
    scan_.angle_increment = urg_->getAngleIncrement();
    scan_.scan_time = urg_->getScanPeriod();
    scan_.time_increment = urg_->getTimeIncrement();
    scan_.range_min = urg_->getRangeMin();
    scan_.range_max = urg_->getRangeMax();

    return true;
  } catch (const std::runtime_error & e) {
    RCLCPP_ERROR(this->get_logger(), "Runtime error connecting to Hokuyo: %s", e.what());
    urg_.reset();
    return false;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Unknown error connecting to Hokuyo: %s", e.what());
    urg_.reset();
    return false;
  }

  return false;
}

void UrgNode::scanThread()
{
  while (!close_scan_) {
    if (!urg_) {
      if (!connect()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        continue;  // Connect failed, sleep, try again.
      }
    }

    if (!urg_ || !rclcpp::ok()) {
      continue;
    }

    try {
      // If the connection failed, don't try and connect
      // pointer is invalid.
      if (!urg_) {
        continue;  // Return to top of main loop, not connected.
      }
      urg_->start_measurement();
      RCLCPP_INFO(this->get_logger(), "Streaming data.");
      // Clear the error count.
      error_count_ = 0;
    } catch (const std::runtime_error & e) {
      RCLCPP_ERROR(this->get_logger(), "Error starting Hokuyo: %s", e.what());
      urg_.reset();
      rclcpp::sleep_for(std::chrono::seconds(1));
      continue;  // Return to top of main loop
    } catch (...) {
      RCLCPP_ERROR(this->get_logger(), "Unknown error starting Hokuyo");
      urg_.reset();
      rclcpp::sleep_for(std::chrono::seconds(1));
      continue;  // Return to top of main loop
    }
    rclcpp::Time last_status_update = this->now();

    while (!close_scan_) {
      // Don't allow external access during grabbing the scan.
      try {
        if (urg_->grabScan(scan_)) {
          laser_pub_->publish(scan_);
        } else {
          RCLCPP_INFO(this->get_logger(), "Stream stopped due to error, restarting");
          urg_->start_measurement();
          std::this_thread::sleep_for(std::chrono::milliseconds(100));
          error_count_++;
          last_error_ = this->now();
          RCLCPP_INFO(this->get_logger(), "Error count: %d", error_count_);
        }
      } catch (...) {
        RCLCPP_WARN(this->get_logger(), "Unknown error grabbing Hokuyo scan.");
        error_count_++;
      }

      if (reboot_triggered_) {
        reboot_triggered_ = false;
        if (urg_->reboot()) {
          RCLCPP_INFO(this->get_logger(), "Rebooting Hokuyo....");
          urg_.reset();
          std::this_thread::sleep_for(std::chrono::milliseconds(1000));
          break;  // Return to top of main loop
        } else {
          RCLCPP_ERROR(this->get_logger(), "Failed to reboot Hokuyo.");
        }
      }

      // Reestablish connection if things seem to have gone wrong.
      if (error_count_ > error_limit_) {
        RCLCPP_ERROR(this->get_logger(), "Error count exceeded limit, reconnecting.");
        urg_.reset();
        rclcpp::sleep_for(std::chrono::milliseconds(static_cast<uint64_t>(reconn_delay_ * 1000)));
        break;  // Return to top of main loop
      } else {
        rclcpp::Duration period = this->now() - last_error_;
        if (error_count_ > 0 && period.seconds() >= error_reset_period_) {
          RCLCPP_INFO(this->get_logger(), "Error count reset.");
          error_count_ = 0;
        }
      }
    }
  }
}

void UrgNode::run()
{
  // Setup initial connection
  connect();

  // Start scanning now that everything is configured.
  close_scan_ = false;
  scan_thread_ = std::thread(std::bind(&UrgNode::scanThread, this));
}
}  // namespace urg_node

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(urg_node::UrgNode)
