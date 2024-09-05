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

#ifndef TRAPS_SSL_BRIDGE_ROS__FILL_MAP_HPP_
#define TRAPS_SSL_BRIDGE_ROS__FILL_MAP_HPP_

#include <string>
#include <vector>

#include "fmt/core.h"
#include "rclcpp/node.hpp"

namespace traps_ssl_bridge_ros
{
namespace
{

inline auto fill_map(
  const std::pair<std::pair<std::size_t, std::size_t>, std::pair<std::size_t, std::size_t>> & range,
  std::size_t width, std::vector<std::int8_t> && grid)
{
  const auto fill_width = range.second.first - range.first.first;
  const auto fill_height = range.second.second - range.first.second;
  const auto first_offset = range.first.second * width + range.first.first;
  // std::cout << fill_width << ", " << fill_height << ", " << first_offset << std::endl;
  for (auto itr_left = std::next(std::begin(grid), first_offset),
    end = std::next(itr_left, width * fill_height);
    itr_left != end; itr_left = std::next(itr_left, width))
  {
    for (auto itr = itr_left, end = std::next(itr + fill_width); itr != end; itr = std::next(itr)) {
      *itr = 100;
    }
  }
  return std::move(grid);
}

inline auto fill_map_mirror(
  const std::pair<std::pair<std::size_t, std::size_t>, std::pair<std::size_t, std::size_t>> & range,
  std::size_t width, std::vector<std::int8_t> && grid)
{
  grid = fill_map(range, width, std::move(grid));
  grid = fill_map(
    {{width - range.second.first, range.first.second},
      {width - range.first.first, range.second.second}},
    width, std::move(grid));
  return std::move(grid);
}

}  // namespace
}  // namespace traps_ssl_bridge_ros

#endif  // TRAPS_SSL_BRIDGE_ROS__FILL_MAP_HPP_
