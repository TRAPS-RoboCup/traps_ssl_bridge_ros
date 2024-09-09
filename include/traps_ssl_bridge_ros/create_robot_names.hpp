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

#ifndef TRAPS_SSL_BRIDGE_ROS__CREATE_ROBOT_NAMES_HPP_
#define TRAPS_SSL_BRIDGE_ROS__CREATE_ROBOT_NAMES_HPP_

#include <string>
#include <string_view>
#include <vector>

#include "fmt/core.h"

namespace traps_ssl_bridge_ros
{
namespace
{

inline auto create_robot_names(const std::string_view name_prefix, const std::size_t count)
{
  std::vector<std::string> robot_names;
  robot_names.reserve(count);
  for (std::size_t index = 0; index < count; ++index) {
    robot_names.emplace_back(fmt::format("{}{:02}", name_prefix, index));
  }
  return robot_names;
}

}  // namespace
}  // namespace traps_ssl_bridge_ros

#endif  // TRAPS_SSL_BRIDGE_ROS__CREATE_ROBOT_NAMES_HPP_
