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

#ifndef TRAPS_SSL_BRIDGE_ROS__DYNAMIC_QOS_HPP_
#define TRAPS_SSL_BRIDGE_ROS__DYNAMIC_QOS_HPP_

#include "rclcpp/qos.hpp"

namespace traps_ssl_bridge_ros
{

namespace
{

inline auto dynamic_qos(std::size_t qos_depth = 1) noexcept
{
  return rclcpp::QoS(qos_depth).best_effort().durability_volatile();
}

}  // namespace

}  // namespace traps_ssl_bridge_ros

#endif  // TRAPS_SSL_BRIDGE_ROS__DYNAMIC_QOS_HPP_
