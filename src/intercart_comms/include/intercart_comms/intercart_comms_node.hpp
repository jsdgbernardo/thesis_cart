#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/int32.hpp>

#include "intercart_comms/cart_state.hpp"
#include "intercart_comms/conflict_resolver.hpp"

class IntercartCommNode : public rclcpp::Node
{
public:
  IntercartCommNode();

private:
  void neighbor_callback(const geometry_msgs::msg::Pose::SharedPtr msg);
  void self_velocity_callback(const geometry_msgs::msg::Twist::SharedPtr msg);

  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr intent_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr neighbor_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr vel_sub_;

  std::vector<CartState> neighbors_;
  ConflictResolver resolver_;
};
