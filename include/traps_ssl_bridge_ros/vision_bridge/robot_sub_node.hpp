// Copyright 2024 TRAPS
//
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

#ifndef TRAPS_SSL_BRIDGE_ROS__VISION_BRIDGE__ROBOT_SUB_NODE_HPP_
#define TRAPS_SSL_BRIDGE_ROS__VISION_BRIDGE__ROBOT_SUB_NODE_HPP_

#include <optional>
#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

#include "./ssl_vision_detection.pb.h"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "rclcpp/node.hpp"

namespace traps_ssl_bridge_ros::vision_bridge
{

class RobotSubNode
{
public:
  explicit RobotSubNode(rclcpp::Node::SharedPtr robot_sub_node);

  void bridge_pose(
    const builtin_interfaces::msg::Time capture_time, const SSL_DetectionRobot & robot);

private:
  using PoseMsg = geometry_msgs::msg::PoseWithCovarianceStamped;

  rclcpp::Node::SharedPtr sub_node_;
  rclcpp::Publisher<PoseMsg>::SharedPtr pose_publisher_;
  PoseMsg pose_msg_;
};

}  // namespace traps_ssl_bridge_ros::vision_bridge

#endif  // TRAPS_SSL_BRIDGE_ROS__VISION_BRIDGE__ROBOT_SUB_NODE_HPP_