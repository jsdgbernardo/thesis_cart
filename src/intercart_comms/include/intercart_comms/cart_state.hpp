#pragma once

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>

struct CartState
{
  int cart_id;
  geometry_msgs::msg::Pose pose;
  geometry_msgs::msg::Twist velocity;
  int intent;
};