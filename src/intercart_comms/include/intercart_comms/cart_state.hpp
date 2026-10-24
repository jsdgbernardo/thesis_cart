#pragma once

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>

struct CartState
{
  int cart_id;

  geometry_msgs::msg::Pose pose;
  geometry_msgs::msg::Twist velocity;

  // 0 = PROCEED
  // 1 = SLOW
  // 2 = STOP
  // 3 = AVOID
  int intent;

  // 0 = NORMAL
  // 1 = PRIORITY USER
  int priority_class;

  // Time at which this cart entered
  // the current conflict negotiation.
  // -1 means not currently negotiating.
  double arrival_time;
};