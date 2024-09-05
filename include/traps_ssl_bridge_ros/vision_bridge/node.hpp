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

#ifndef TRAPS_SSL_BRIDGE_ROS__VISION_BRIDGE__NODE_HPP_
#define TRAPS_SSL_BRIDGE_ROS__VISION_BRIDGE__NODE_HPP_

#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include "./ssl_vision_geometry.pb.h"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/node.hpp"
#include "traps_ssl_bridge_ros/msg/serial.hpp"
#include "traps_ssl_bridge_ros/static_qos.hpp"
#include "traps_ssl_bridge_ros/visibility.hpp"
#include "traps_ssl_bridge_ros/vision_bridge/robot_sub_node.hpp"

namespace traps_ssl_bridge_ros::vision_bridge
{

class Node : public rclcpp::Node
{
public:
  static constexpr auto default_node_name() noexcept {return "vision_bridge";}

  TRAPS_SSL_BRIDGE_ROS_PUBLIC
  Node(
    const std::string & node_name, const std::string & node_namespace,
    const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions());

  TRAPS_SSL_BRIDGE_ROS_PUBLIC
  explicit inline Node(
    const std::string & node_name, const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions())
  : Node(node_name, "", node_options)
  {
  }

  TRAPS_SSL_BRIDGE_ROS_PUBLIC
  explicit inline Node(const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions())
  : Node(this->default_node_name(), "", node_options)
  {
  }

private:
  using MapMsg = nav_msgs::msg::OccupancyGrid;
  using SerialMsg = traps_ssl_bridge_ros::msg::Serial;

  void receive(const SerialMsg::ConstSharedPtr udp_buffer_msg);

  std::vector<std::optional<RobotSubNode>> blue_robot_sub_nodes_, yellow_robot_sub_nodes_;
  rclcpp::Publisher<MapMsg>::SharedPtr map_publisher_;
  rclcpp::Subscription<SerialMsg>::SharedPtr udp_buffer_subscription_;

  MapMsg map_msg_;
  double map_resolution_inv_;
  std::size_t wall_thickness_, goal_width_, goal_height_harf_;

  MapMsg::_info_type map_info_last_;
};

}  // namespace traps_ssl_bridge_ros::vision_bridge

#endif  // TRAPS_SSL_BRIDGE_ROS__VISION_BRIDGE__NODE_HPP_