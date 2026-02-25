#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include "collision_avoidance/obstacle_detector.hpp"
#include "collision_avoidance/avoidance_planner.hpp"

class CollisionAvoidanceNode : public rclcpp::Node
{
public:
  CollisionAvoidanceNode();

private:
  // --- Callbacks ---
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void cmd_callback(const geometry_msgs::msg::Twist::SharedPtr msg);

  // --- ROS Interfaces ---
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;

  // --- Internal State ---
  geometry_msgs::msg::Twist last_cmd_;
  bool obstacle_detected_ = false;

  // --- Subsystem Components ---
  ObstacleDetector detector_;
  AvoidancePlanner planner_;
};



// #pragma once

// #include <rclcpp/rclcpp.hpp>
// #include <sensor_msgs/msg/laser_scan.hpp>
// #include <geometry_msgs/msg/twist.hpp>

// #include "collision_avoidance/obstacle_detector.hpp"
// #include "collision_avoidance/avoidance_planner.hpp"

// class CollisionAvoidanceNode : public rclcpp::Node
// {
// public:
//   CollisionAvoidanceNode();

// private:
//   void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

//   rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
//   rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;

//   ObstacleDetector detector_;
//   AvoidancePlanner planner_;
// };
