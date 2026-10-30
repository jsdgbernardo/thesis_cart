// // #include "collision_avoidance/avoidance_planner.hpp"

// // AvoidancePlanner::AvoidancePlanner(
// //     float safe_dist,
// //     float slow_dist,
// //     float base_speed,
// //     float turn_speed)
// //     : safe_dist_(safe_dist),
// //       slow_dist_(slow_dist),
// //       base_speed_(base_speed),
// //       turn_speed_(turn_speed),
// //       last_state_("CLEAR")
// // {
// // }

// // geometry_msgs::msg::Twist AvoidancePlanner::compute_cmd(
// //     float front,
// //     float left,
// //     float right)
// // {
// //     geometry_msgs::msg::Twist cmd;
// //     std::string state;

// //     if (front < safe_dist_) {

// //         cmd.linear.x = 0.0;

// //         // Initial directional check, not yet a full
// //         // cart-footprint or swept-volume clearance test.
// //         if (left < safe_dist_ && right < safe_dist_) {

// //             cmd.angular.z = 0.0;
// //             state = "BLOCKED";

// //         } else {

// //             cmd.angular.z =
// //                 (left > right) ? turn_speed_ : -turn_speed_;

// //             state = "AVOIDING";
// //         }

// //     } else if (front < slow_dist_) {

// //         cmd.linear.x = base_speed_ * 0.5;

// //         if (left < safe_dist_ && right < safe_dist_) {

// //             cmd.linear.x = 0.0;
// //             cmd.angular.z = 0.0;
// //             state = "BLOCKED";

// //         } else {

// //             cmd.angular.z =
// //                 (left > right) ?
// //                 turn_speed_ * 0.5 :
// //                 -turn_speed_ * 0.5;

// //             state = "SLOWING";
// //         }

// //     } else {

// //         cmd.linear.x = base_speed_;
// //         cmd.angular.z = 0.0;
// //         state = "CLEAR";
// //     }

// //     if (state != last_state_) {
// //         RCLCPP_INFO(
// //             rclcpp::get_logger("avoidance_planner"),
// //             "State: %s",
// //             state.c_str()
// //         );

// //         last_state_ = state;
// //     }

// //     return cmd;
// // }




// #include "collision_avoidance/avoidance_planner.hpp"

// AvoidancePlanner::AvoidancePlanner(double side_clearance_threshold)
//     : side_clearance_threshold_(side_clearance_threshold)
// {
// }

// AvoidancePlanner::TurnDirection AvoidancePlanner::choose_turn_direction(
//     float left_distance, bool left_valid,
//     float right_distance, bool right_valid) const
// {
//     const bool left_candidate = left_valid && left_distance > side_clearance_threshold_;
//     const bool right_candidate = right_valid && right_distance > side_clearance_threshold_;

//     if (left_candidate && right_candidate) {
//         return (left_distance >= right_distance) ? TurnDirection::LEFT : TurnDirection::RIGHT;
//     }
//     if (left_candidate) {
//         return TurnDirection::LEFT;
//     }
//     if (right_candidate) {
//         return TurnDirection::RIGHT;
//     }
//     return TurnDirection::NONE;
// }


#include "collision_avoidance/avoidance_planner.hpp"

AvoidancePlanner::AvoidancePlanner(double side_clearance_threshold)
    : side_clearance_threshold_(side_clearance_threshold)
{
}

AvoidancePlanner::TurnDirection AvoidancePlanner::choose_turn_direction(
    float left_distance, bool left_valid,
    float right_distance, bool right_valid) const
{
    const bool left_candidate = left_valid && left_distance > side_clearance_threshold_;
    const bool right_candidate = right_valid && right_distance > side_clearance_threshold_;

    if (left_candidate && right_candidate) {
        return (left_distance >= right_distance) ? TurnDirection::LEFT : TurnDirection::RIGHT;
    }
    if (left_candidate) {
        return TurnDirection::LEFT;
    }
    if (right_candidate) {
        return TurnDirection::RIGHT;
    }
    return TurnDirection::NONE;
}
