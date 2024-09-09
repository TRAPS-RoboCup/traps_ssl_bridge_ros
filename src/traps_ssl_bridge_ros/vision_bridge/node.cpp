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

#include "traps_ssl_bridge_ros/vision_bridge/node.hpp"

#include <deque>

#include "./ssl_vision_wrapper.pb.h"
#include "asio/ip/multicast.hpp"
#include "fmt/core.h"
#include "fmt/ranges.h"
#include "traps_ssl_bridge_ros/create_robot_names.hpp"
#include "traps_ssl_bridge_ros/create_robot_sub_nodes.hpp"
#include "traps_ssl_bridge_ros/dynamic_qos.hpp"
#include "traps_ssl_bridge_ros/fill_map.hpp"

namespace traps_ssl_bridge_ros::vision_bridge
{

Node::Node(
  const std::string & node_name, const std::string & node_namespace,
  const rclcpp::NodeOptions & node_options)
: rclcpp::Node(node_name, node_namespace, node_options),
  blue_robot_sub_nodes_(create_robot_sub_nodes<std::optional<RobotSubNode>>(
      this, this->declare_parameter(
        "blue_robots", create_robot_names(
          this->declare_parameter("blue_robots.prefix", "blue"),
          this->declare_parameter("blue_robots.count", 11))))),
  yellow_robot_sub_nodes_(create_robot_sub_nodes<std::optional<RobotSubNode>>(
      this, this->declare_parameter(
        "yellow_robots", create_robot_names(
          this->declare_parameter("yellow_robots.prefix", "yellow"),
          this->declare_parameter("yellow_robots.count", 11))))),
  ball_publisher_(this->create_publisher<PoseMsg>("ball/pose", dynamic_qos())),
  map_publisher_(this->create_publisher<MapMsg>("map", static_qos())),
  udp_buffer_subscription_(this->create_subscription<SerialMsg>("udp_buffer/ssl_vision", dynamic_qos(), [this](const SerialMsg::ConstSharedPtr udp_buffer_msg){ this->receive(std::move(udp_buffer_msg)); }))
{
  // frame_id初期化
  ball_pose_msg_.header.frame_id = map_msg_.header.frame_id = this->has_parameter("frame_id") ?
    this->get_parameter("frame_id").as_string() :
    this->declare_parameter("frame_id", "map");

  // ball_pose_msgの初期化
  ball_pose_msg_.pose.covariance = {
    0.25, 0.00, 0.00, 0.00, 0.00, 0.00,  //
    0.00, 0.25, 0.00, 0.00, 0.00, 0.00,  //
    0.00, 0.00, 0.00, 0.00, 0.00, 0.00,  //
    0.00, 0.00, 0.00, 0.00, 0.00, 0.00,  //
    0.00, 0.00, 0.00, 0.00, 0.00, 0.00,  //
    0.00, 0.00, 0.00, 0.00, 0.00, 0.25,  //
  };
  ball_pose_msg_.pose.pose.orientation.y = -std::sqrt(0.5);
  ball_pose_msg_.pose.pose.orientation.w = +std::sqrt(0.5);

  // map_msgと関連変数の初期化
  map_msg_.info.resolution = this->declare_parameter("map.resolution", 0.005);
  map_resolution_inv_ = 1.0 / map_msg_.info.resolution;
  wall_thickness_ = 0.02 * map_resolution_inv_;
  goal_width_ = 0.18 * map_resolution_inv_;
  goal_height_harf_ = 1.8 / 2 * map_resolution_inv_;
}

void Node::receive(const SerialMsg::ConstSharedPtr udp_buffer_msg)
{
  // 受信時刻設定
  const auto now = this->now();

  // protobufへ変換
  SSL_WrapperPacket packet;
  if (!packet.MergeFromString({udp_buffer_msg->data.begin(), udp_buffer_msg->data.end()})) {
    RCLCPP_ERROR(this->get_logger(), "failed to purse buffer");
    return;
  }

  // detentionのpublish
  if (packet.has_detection()) {
    const auto & detection = packet.detection();

    // キャプチャ遅延処理
    const std::uint64_t delay_ns = (detection.t_sent() - detection.t_capture()) * 1e9;
    const auto stamp = now - std::chrono::nanoseconds(delay_ns);

    // publish
    const auto publish_robots = [&stamp](auto & subnodes, const auto & robots) {
        for (const auto & robot : robots) {
          if (robot.robot_id() >= subnodes.size()) {
            continue;
          }
          auto & subnode = subnodes[robot.robot_id()];
          if (!subnode) {
            continue;
          }

          subnode->bridge_pose(stamp, robot);
        }
      };
    publish_robots(yellow_robot_sub_nodes_, detection.robots_yellow());
    publish_robots(blue_robot_sub_nodes_, detection.robots_blue());

    for (const auto & ball : detection.balls()) {
      ball_pose_msg_.header.stamp = stamp;
      ball_pose_msg_.pose.pose.position.x = ball.x();
      ball_pose_msg_.pose.pose.position.y = ball.y();
      ball_pose_msg_.pose.pose.position.z = ball.z();
      ball_publisher_->publish(ball_pose_msg_);
    }
  }

  // geometryのpublish
  if (packet.has_geometry() && packet.geometry().has_field()) {
    const auto field = packet.geometry().field();

    // map_msg.infoの設定
    const auto field_offset =
      wall_thickness_ +
      static_cast<std::size_t>(1e-3 * field.boundary_width() * map_resolution_inv_);
    const auto with_harf =
      field_offset +
      static_cast<std::size_t>((1e-3 / 2) * field.field_length() * map_resolution_inv_);
    const auto height_harf =
      field_offset +
      static_cast<std::size_t>((1e-3 / 2) * field.field_width() * map_resolution_inv_);
    map_msg_.info.width = 2 * with_harf;
    map_msg_.info.height = 2 * height_harf;
    map_msg_.info.origin.position.x = (1e-3 / -2) * field.field_length();
    map_msg_.info.origin.position.y = (1e-3 / -2) * field.field_width();

    // 前回publishしたものと同じならpublishしない
    if (map_msg_.info != map_info_last_) {
      map_msg_.header.stamp = map_msg_.info.map_load_time = now;
      map_msg_.data = decltype(map_msg_.data)(map_msg_.info.height * map_msg_.info.width);

      // フィールドの壁を塗りつぶす
      map_msg_.data = fill_map(
        {{0, 0}, {map_msg_.info.width, wall_thickness_}}, map_msg_.info.width,
        std::move(map_msg_.data));
      map_msg_.data = fill_map(
        {{0, map_msg_.info.height - wall_thickness_}, {map_msg_.info.width, map_msg_.info.height}},
        map_msg_.info.width, std::move(map_msg_.data));
      map_msg_.data = fill_map_mirror(
        {{0, 0}, {wall_thickness_, map_msg_.info.height}}, map_msg_.info.width,
        std::move(map_msg_.data));

      // ゴールを塗りつぶす
      map_msg_.data = fill_map_mirror(
        {{field_offset - goal_width_ - wall_thickness_, height_harf + goal_height_harf_},
          {field_offset, height_harf + goal_height_harf_ + wall_thickness_}},
        map_msg_.info.width, std::move(map_msg_.data));
      map_msg_.data = fill_map_mirror(
        {{field_offset - goal_width_ - wall_thickness_,
          height_harf - goal_height_harf_ - wall_thickness_},
          {field_offset, height_harf - goal_height_harf_}},
        map_msg_.info.width, std::move(map_msg_.data));
      map_msg_.data = fill_map_mirror(
        {{field_offset - goal_width_ - wall_thickness_, height_harf - goal_height_harf_},
          {field_offset - goal_width_, height_harf + goal_height_harf_}},
        map_msg_.info.width, std::move(map_msg_.data));
      map_publisher_->publish(map_msg_);

      // publishした内容を記録
      map_info_last_ = map_msg_.info;
    }
  }
}

}  // namespace traps_ssl_bridge_ros::vision_bridge

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(traps_ssl_bridge_ros::vision_bridge::Node)
