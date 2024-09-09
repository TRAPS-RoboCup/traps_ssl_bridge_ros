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

#include "traps_ssl_bridge_ros/udp_receiver/node.hpp"

#include <charconv>
#include <string>
#include <vector>

#include "asio/ip/address.hpp"
#include "asio/ip/multicast.hpp"
#include "asio/ip/udp.hpp"
#include "fmt/core.h"
#include "traps_ssl_bridge_ros/dynamic_qos.hpp"

namespace traps_ssl_bridge_ros::udp_receiver
{

Node::Node(
  const std::string & node_name, const std::string & node_namespace,
  const rclcpp::NodeOptions & node_options)
: rclcpp::Node(node_name, node_namespace, node_options),
  receiving_timer_(this->create_wall_timer(std::chrono::seconds(0), [this] {this->receive();})),
  io_service_()
{
  // アドレスの取得
  const auto address_strs = this->declare_parameter("addresses", std::vector<std::string>());

  // アドレスからsocketとpublisherを構築
  socket_and_publishers_.reserve(address_strs.size());
  for (const auto & address_str : address_strs) {
    try {
      // ':'で区切られたipアドレスとポートを取得(できなければエラー)
      const auto format_error_msg_getter = [](const auto & address_str) {
          return fmt::format(
            "address \"{}\" is invalid format. (format must be \"[alias]:[ip address]:[port]\")",
            address_str);
        };
      const auto sepalate_pos_0 = address_str.find(':');
      if (sepalate_pos_0 == std::string::npos) {
        RCLCPP_ERROR(this->get_logger(), format_error_msg_getter(address_str).c_str());
        continue;
      }
      const auto sepalate_pos_1 = address_str.substr(sepalate_pos_0 + 1).find(':');
      if (sepalate_pos_0 == std::string::npos) {
        RCLCPP_ERROR(this->get_logger(), format_error_msg_getter(address_str).c_str());
        continue;
      }
      const auto alias = address_str.substr(0, sepalate_pos_0);
      const auto ip_address_str = address_str.substr(sepalate_pos_0 + 1, sepalate_pos_1);
      const auto ip_address = asio::ip::address::from_string(ip_address_str);
      const auto port_str = address_str.substr(sepalate_pos_0 + sepalate_pos_1 + 2);
      int port;
      [[maybe_unused]] const auto [port_ptr, port_ec] =
        std::from_chars(port_str.data(), port_str.data() + port_str.size(), port);
      if (port_ec != std::errc{}) {
        const auto error_str = fmt::format("port \"{}\" must be int.", port_str);
        RCLCPP_ERROR(this->get_logger(), error_str.c_str());
        continue;
      }

      // ソケットの作成
      const auto endpoint = asio::ip::udp::endpoint(
        ip_address.is_v6() ? asio::ip::udp::v6() : asio::ip::udp::v4(), port);
      auto socket = asio::ip::udp::socket(io_service_, endpoint);

      // マルチキャストグループへの追加
      if (ip_address.is_multicast()) {
        socket.set_option(asio::ip::multicast::join_group(ip_address));
      } else if (!ip_address.is_unspecified()) {
        // ipアドレスがからでもマルチキャストでもなければ警告
        const auto error_str = fmt::format(
          "ip address \"{}\" is not multicast. ip address will be ignored.", ip_address_str);
        RCLCPP_WARN(this->get_logger(), error_str.c_str());
      }

      // publicher to socket wo risutonituika
      socket_and_publishers_.emplace_back(
        SocketAndPublisher{
          std::move(socket),
          this->create_publisher<SerialMsg>(fmt::format("udp_buffer/{}", alias), dynamic_qos())});
    }
    // asio
    catch (const asio::system_error::exception & e) {
      const auto error_str = fmt::format("asio error: {}", e.what());
      RCLCPP_ERROR(this->get_logger(), error_str.c_str());
      continue;
    } catch (...) {
      RCLCPP_ERROR(this->get_logger(), "unknown error");
      continue;
    }
  }
}

void Node::receive()
{
  for (auto & socket_and_publisher : socket_and_publishers_) {
    // zyusinnsitanagasawosyutoku
    const auto received_length = socket_and_publisher.socket.available();
    if (!received_length) {
      continue;
    }

    // zyusinnsyori
    SerialMsg buffer_msg;
    buffer_msg.data.resize(received_length);
    asio::error_code ec;

    socket_and_publisher.socket.receive(asio::buffer(buffer_msg.data), 0, ec);

    // publish
    socket_and_publisher.buffer_publisher->publish(buffer_msg);
  }
}

}  // namespace traps_ssl_bridge_ros::udp_receiver

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(traps_ssl_bridge_ros::udp_receiver::Node)
