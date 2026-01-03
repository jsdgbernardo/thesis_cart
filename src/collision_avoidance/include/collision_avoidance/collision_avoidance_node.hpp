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
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;

  ObstacleDetector detector_;
  AvoidancePlanner planner_;
};
