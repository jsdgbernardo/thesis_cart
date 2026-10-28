// #pragma once

#include <geometry_msgs/msg/twist.hpp>
#include <string>
#include <rclcpp/rclcpp.hpp>  // for logging

// class AvoidancePlanner
// {
// public:
//   AvoidancePlanner(float safe_dist, float slow_dist,
//                    float base_speed, float turn_speed);

//   geometry_msgs::msg::Twist compute_cmd(
//     float front, float left, float right);

//   // --- Getter for safe_dist_ ---
//   float get_safe_dist() const { return safe_dist_; }

// private:
//   float safe_dist_;
//   float slow_dist_;
//   float base_speed_;
//   float turn_speed_;

//   std::string last_state_ = "CLEAR";  // NEW: track last logged state
// };


#pragma once

class AvoidancePlanner
{
public:
    // Only a preliminary turning-direction heuristic, not collision-free path planning.
    enum class TurnDirection { NONE = 0, LEFT = 1, RIGHT = -1 };

    explicit AvoidancePlanner(double side_clearance_threshold);

    TurnDirection choose_turn_direction(
        float left_distance, bool left_valid,
        float right_distance, bool right_valid) const;

private:
    double side_clearance_threshold_;
};