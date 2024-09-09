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

#include "traps_ssl_bridge_ros/vision_bridge/robot_sub_node.hpp"

#include "traps_ssl_bridge_ros/dynamic_qos.hpp"

namespace traps_ssl_bridge_ros::vision_bridge
{

RobotSubNode::RobotSubNode(rclcpp::Node::SharedPtr sub_node)
: sub_node_(std::move(sub_node)),
  pose_publisher_(sub_node_->create_publisher<PoseMsg>("vision/pose", dynamic_qos()))
{
  pose_msg_.header.frame_id = sub_node_->has_parameter("frame_id") ?
    sub_node_->get_parameter("frame_id").as_string() :
    sub_node_->declare_parameter("frame_id", "map");
}

void RobotSubNode::bridge_pose(
  const builtin_interfaces::msg::Time capture_time, const SSL_DetectionRobot & robot)
{
  pose_msg_.header.stamp = capture_time;
  pose_msg_.pose.covariance = {
    0.25, 0.00, 0.00, 0.00, 0.00, 0.00,  //
    0.00, 0.25, 0.00, 0.00, 0.00, 0.00,  //
    0.00, 0.00, 0.00, 0.00, 0.00, 0.00,  //
    0.00, 0.00, 0.00, 0.00, 0.00, 0.00,  //
    0.00, 0.00, 0.00, 0.00, 0.00, 0.00,  //
    0.00, 0.00, 0.00, 0.00, 0.00, 0.25,  //
  };
  pose_msg_.pose.pose.position.x = robot.x() * 1e-3;
  pose_msg_.pose.pose.position.y = robot.y() * 1e-3;
  const auto yaw_harf = robot.orientation() * 0.5;
  pose_msg_.pose.pose.orientation.z = std::sin(yaw_harf);
  pose_msg_.pose.pose.orientation.w = std::cos(yaw_harf);
  pose_publisher_->publish(pose_msg_);
}

}  // namespace traps_ssl_bridge_ros::vision_bridge
