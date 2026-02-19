#include "collision_avoidance/avoidance_planner.hpp"

AvoidancePlanner::AvoidancePlanner(float safe_dist, float slow_dist,
                                   float base_speed, float turn_speed)
  : safe_dist_(safe_dist), slow_dist_(slow_dist),
    base_speed_(base_speed), turn_speed_(turn_speed)
{
}

geometry_msgs::msg::Twist AvoidancePlanner::compute_cmd(
  float front, float left, float right)
{
  geometry_msgs::msg::Twist cmd;

  // dynammic obstacles will follow the same
  // limitation: dynamic obstacles moving too fast

  // TO DO: account for coordanates of the shelves/permanent items in the map
  // TO DO: finetune safe distance/speed parameters based on irl dimensions

  if (front < safe_dist_) { // CA-3
    // STOP + TURN
    cmd.linear.x = 0.0;
    cmd.angular.z = (left > right) ? turn_speed_ : -turn_speed_; // turn towards the direction with more space

    // TO DO: needs to be finetuned to curve around the obstacle but ultimately return to global path
  }
  else if (front < slow_dist_) { // CA-1, CA-2
    // SLOW + TURN
    cmd.linear.x = base_speed_ * 0.5;
    cmd.angular.z = (left > right) ? turn_speed_ * 0.5 : -turn_speed_ * 0.5;

    // TO DO: needs to return back to global path
  }
  else {
    // CLEAR PATH

    // directions will be taken from global planner
    // TO DO: subscribe to global path
    cmd.linear.x = base_speed_;
    cmd.angular.z = 0.0;
  }

  return cmd;
}
