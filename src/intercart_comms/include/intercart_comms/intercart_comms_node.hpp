#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/int32.hpp>

#include <map>

#include "intercart_comms/cart_state.hpp"
#include "intercart_comms/conflict_resolver.hpp"
#include "intercart_comms/msg/cart_state.hpp"

#include <gazebo_msgs/msg/model_states.hpp>
#include <string>

class IntercartCommNode : public rclcpp::Node
{
public:
  IntercartCommNode();

private:

  // Receive the state of another cart
  void neighbor_callback(
    const intercart_comms::msg::CartState::SharedPtr msg);

  // Receive this cart's velocity
  void self_velocity_callback(
    const geometry_msgs::msg::Twist::SharedPtr msg);

  // Publish this cart's own state
  void publish_self_state();

  void gazebo_model_states_callback(
    const gazebo_msgs::msg::ModelStates::SharedPtr msg);

  // ROS 2 publishers/subscribers
  rclcpp::Publisher<intercart_comms::msg::CartState>::SharedPtr state_pub_;

  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr intent_pub_;

  rclcpp::Subscription<intercart_comms::msg::CartState>::SharedPtr neighbor_sub_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr vel_sub_;

  rclcpp::Subscription<gazebo_msgs::msg::ModelStates>::SharedPtr
    gazebo_states_sub_;

  std::string gazebo_model_name_;

  // Command velocity sent to this cart's Gazebo model
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr
  gazebo_cmd_vel_pub_;

  // // Timer for periodic state broadcasting
  // rclcpp::TimerBase::SharedPtr state_timer_;
  // Timer for periodic simulation and state broadcasting
  rclcpp::TimerBase::SharedPtr state_timer_;
  // Update simulated position
  void update_simulation();

  // Store the most recent state of every neighboring cart
  std::map<int, CartState> neighbors_;

  // This cart's own state
  CartState self_state_;

  // Velocity requested by the cart/navigation system
  geometry_msgs::msg::Twist desired_velocity_;

  // Velocity allowed after inter-cart safety checks
  geometry_msgs::msg::Twist safe_velocity_;

  // Collision/conflict decision system
  ConflictResolver resolver_;
};