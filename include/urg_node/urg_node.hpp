/*
 * Copyright (c) 2013, Willow Garage, Inc.
 * Copyright (c) 2017, Clearpath Robotics, Inc.
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
 * Author: Mike O'Driscoll
 */

#ifndef URG_NODE__URG_NODE_HPP_
#define URG_NODE__URG_NODE_HPP_

#include <atomic>
#include <chrono>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "urg_node/urg_c_wrapper.hpp"

namespace urg_node
{
class UrgNode : public rclcpp::Node
{
public:
  explicit UrgNode(const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions());

  virtual ~UrgNode();

  /**
   * @brief Start's the nodes threads to run the lidar.
   */
  void run();

private:
  bool connect();

  void scanThread();

  void rebootCallback(
    const std::shared_ptr<rmw_request_id_t> requestHeader,
    const std_srvs::srv::Trigger::Request::SharedPtr req,
    const std_srvs::srv::Trigger::Response::SharedPtr res);

  std::thread run_thread_;
  std::thread scan_thread_;

  std::unique_ptr<urg_node::URGCWrapper> urg_;

  int error_count_;
  int error_limit_;
  double freq_min_;
  bool close_scan_;
  std::string ip_address_;
  int ip_port_;
  double angle_min_;
  double angle_max_;
  /**
   * Divide the number of rays per scan by cluster_ (if cluster_ == 10, you get 1/10 ray per scan)
   * */
  int cluster_;  // default : 1, range : 1 to 100
  /** Reduce the rate of scans */
  int skip_;  // default : 0, range : 0 to 9

  /** The laser tf frame id. */
  std::string laser_frame_id_;

  /** how long to wait to reconnect **/
  double reconn_delay_;

  bool disable_linger_;

  std::atomic_bool reboot_triggered_;

  double error_reset_period_;

  rclcpp::Time last_error_;

  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reboot_service_;

  sensor_msgs::msg::LaserScan scan_;
};
}  // namespace urg_node

#endif  // URG_NODE__URG_NODE_HPP_
