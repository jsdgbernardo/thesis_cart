#include "collision_avoidance/avoidance_planner.hpp"

geometry_msgs::msg::Twist AvoidancePlanner::compute_cmd(
  float front, float left, float right)
{
  geometry_msgs::msg::Twist cmd;

  if (front < safe_dist_) {
    // STOP + TURN
    cmd.linear.x = 0.0;
    cmd.angular.z = (left > right) ? turn_speed_ : -turn_speed_;
  }
  else if (front < slow_dist_) {
    // SLOW + TURN
    cmd.linear.x = base_speed_ * 0.5;
    cmd.angular.z = (left > right) ? turn_speed_ * 0.5 : -turn_speed_ * 0.5;
  }
  else {
    // CLEAR PATH
    cmd.linear.x = base_speed_;
    cmd.angular.z = 0.0;
  }

  return cmd;
}
