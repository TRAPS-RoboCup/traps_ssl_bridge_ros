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

#ifndef TRAPS_SSL_BRIDGE_ROS__CREATE_ROBOT_SUB_NODES_HPP_
#define TRAPS_SSL_BRIDGE_ROS__CREATE_ROBOT_SUB_NODES_HPP_

#include <string>
#include <vector>

#include "fmt/core.h"
#include "rclcpp/node.hpp"

namespace traps_ssl_bridge_ros
{
namespace
{

template<class SubNode = rclcpp::Node::SharedPtr>
inline auto create_robot_sub_nodes(rclcpp::Node * node, const std::vector<std::string> & robot_names)
{
  auto robot_subnodes = std::vector<SubNode>();
  for (const auto & robot_name : robot_names) {
    if (robot_name == "-") {
      robot_subnodes.emplace_back();
    } else if (robot_name == "/") {
      robot_subnodes.emplace_back(node->shared_from_this());
    } else {
      robot_subnodes.emplace_back(node->create_sub_node(robot_name));
    }
  }
  return robot_subnodes;
}

}  // namespace
}  // namespace traps_ssl_bridge_ros

#endif  // TRAPS_SSL_BRIDGE_ROS__CREATE_ROBOT_SUB_NODES_HPP_