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

#ifndef TRAPS_SSL_BRIDGE_ROS__UDP_RECEIVER__NODE_HPP_
#define TRAPS_SSL_BRIDGE_ROS__UDP_RECEIVER__NODE_HPP_

#include <string>
#include <vector>

#include "asio/io_service.hpp"
#include "asio/ip/address.hpp"
#include "asio/ip/udp.hpp"
#include "rclcpp/node.hpp"
#include "traps_ssl_bridge_ros/msg/serial.hpp"
#include "traps_ssl_bridge_ros/visibility.hpp"

namespace traps_ssl_bridge_ros::udp_receiver
{

class Node : public rclcpp::Node
{
public:
  static constexpr auto default_node_name() noexcept {return "udp_receiver";}

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
  using SerialMsg = traps_ssl_bridge_ros::msg::Serial;

  void receive();

  struct SocketAndPublisher
  {
    asio::ip::udp::socket socket;
    rclcpp::Publisher<SerialMsg>::SharedPtr buffer_publisher;
  };

  std::vector<SocketAndPublisher> socket_and_publishers_;
  rclcpp::TimerBase::SharedPtr receiving_timer_;

  asio::io_service io_service_;
  asio::error_code ec_;
};

}  // namespace traps_ssl_bridge_ros::udp_receiver

#endif  // TRAPS_SSL_BRIDGE_ROS__UDP_RECEIVER__NODE_HPP_
