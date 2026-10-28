
// #include "collision_avoidance/collision_avoidance_node.hpp"

// #include <algorithm>
// #include <chrono>
// #include <functional>

// CollisionAvoidanceNode::CollisionAvoidanceNode()
//     : Node("collision_avoidance_node"),
//       detector_(15.0, 45.0),
//       planner_(
//           this->declare_parameter("safe_distance", 0.5),
//           this->declare_parameter("slow_distance", 1.0),
//           this->declare_parameter("base_speed", 0.4),
//           this->declare_parameter("turn_speed", 0.8)
//       )
// {
//     // LiDAR data
//     scan_sub_ =
//         this->create_subscription<sensor_msgs::msg::LaserScan>(
//             "/scan",
//             rclcpp::SensorDataQoS(),
//             std::bind(
//                 &CollisionAvoidanceNode::scan_callback,
//                 this,
//                 std::placeholders::_1
//             )
//         );

//     // Test movement command
//     cmd_sub_ =
//         this->create_subscription<geometry_msgs::msg::Twist>(
//             "/cmd_vel_manual",
//             10,
//             std::bind(
//                 &CollisionAvoidanceNode::cmd_callback,
//                 this,
//                 std::placeholders::_1
//             )
//         );

//     // Filtered output
//     cmd_pub_ =
//         this->create_publisher<geometry_msgs::msg::Twist>(
//             "/cmd_vel_collision", 10
//         );

//     // Diagnostic state
//     state_pub_ =
//         this->create_publisher<std_msgs::msg::String>(
//             "/cart_state", 10
//         );

//     // Evaluate sensor and commands at 20 Hz
//     control_timer_ = this->create_wall_timer(
//         std::chrono::milliseconds(50),
//         std::bind(
//             &CollisionAvoidanceNode::control_callback,
//             this
//         )
//     );

//     RCLCPP_INFO(
//         this->get_logger(),
//         "Collision Avoidance Node started"
//     );
// }

// void CollisionAvoidanceNode::scan_callback(
//     const sensor_msgs::msg::LaserScan::SharedPtr msg)
// {
//     detector_.update(*msg);

//     last_scan_time_ = this->now();
//     have_scan_ = true;
// }

// void CollisionAvoidanceNode::cmd_callback(
//     const geometry_msgs::msg::Twist::SharedPtr msg)
// {
//     last_cmd_ = *msg;

//     last_cmd_time_ = this->now();
//     have_cmd_ = true;
// }

// void CollisionAvoidanceNode::control_callback()
// {
//     geometry_msgs::msg::Twist safe_cmd;
//     std_msgs::msg::String state_msg;

//     auto current_time = this->now();

//     bool scan_fresh =
//         have_scan_ &&
//         (current_time - last_scan_time_).seconds() < 0.5;

//     bool cmd_fresh =
//         have_cmd_ &&
//         (current_time - last_cmd_time_).seconds() < 0.5;

//     if (!scan_fresh || !detector_.has_valid_front_data()) {

//         // Zero velocity if LiDAR data is unavailable
//         state_msg.data = "SENSOR_UNAVAILABLE";

//     } else if (!cmd_fresh) {

//         // Zero velocity if no current command
//         state_msg.data = "IDLE";

//     } else if (last_cmd_.linear.x <= 0.0) {

//         // Initial tests permit forward requests only.
//         // Reverse/turn-only movements require
//         // additional clearance checks.
//         state_msg.data = "IDLE";

//     } else {

//         float front = detector_.front_distance();
//         float left = detector_.left_distance();
//         float right = detector_.right_distance();

//         float safe_dist =
//             this->get_parameter("safe_distance").as_double();

//         float slow_dist =
//             this->get_parameter("slow_distance").as_double();

//         if (front < safe_dist) {

//             safe_cmd = planner_.compute_cmd(
//                 front, left, right
//             );

//             state_msg.data =
//                 (safe_cmd.angular.z == 0.0) ?
//                 "BLOCKED" : "AVOIDING";

//         } else if (front < slow_dist) {

//             safe_cmd = planner_.compute_cmd(
//                 front, left, right
//             );

//             // Do not exceed requested forward speed
//             safe_cmd.linear.x = std::min(
//                 safe_cmd.linear.x,
//                 last_cmd_.linear.x
//             );

//             state_msg.data =
//                 (safe_cmd.linear.x == 0.0) ?
//                 "BLOCKED" : "SLOWING";

//         } else {

//             // Clear: pass through the requested command
//             safe_cmd = last_cmd_;
//             state_msg.data = "CLEAR";
//         }
//     }




//     // // TEMPORARY: Straight-line floor testing only.
//     // // Do not execute autonomous avoidance turns yet.

//     // if (state_msg.data == "CLEAR") {

//     //     // Allow forward motion, but no turning.
//     //     safe_cmd.angular.z = 0.0;

//     // } else {

//     //     // Stop for any obstacle, invalid sensor data,
//     //     // expired command, or other non-clear state.
//     //     safe_cmd = geometry_msgs::msg::Twist();
//     // }



//     // TEMPORARY: Forward-only obstacle approach testing.
//     // Allow normal movement, slow movement, and stopping.
//     // Autonomous avoidance turning remains disabled.

//     if (state_msg.data == "CLEAR") {

//         // Normal forward speed, capped at 0.07 m/s
//         safe_cmd.linear.x = std::min(
//             std::max(last_cmd_.linear.x, 0.0),
//             0.07
//         );

//         safe_cmd.angular.z = 0.0;

//     } else if (state_msg.data == "SLOWING") {

//         // Reduce forward speed to at most 0.05 m/s.
//         // Preserve any lower limit imposed by the planner.
//         safe_cmd.linear.x = std::max(
//             0.0,
//             std::min(
//                 0.05,
//                 std::min(
//                     last_cmd_.linear.x,
//                     safe_cmd.linear.x
//                 )
//             )
//         );

//         // Do not perform automatic turning yet.
//         safe_cmd.angular.z = 0.0;

//     } else {

//         // STOP for AVOIDING, BLOCKED, IDLE,
//         // SENSOR_UNAVAILABLE, or unexpected states.
//         safe_cmd = geometry_msgs::msg::Twist();
//     }




//     cmd_pub_->publish(safe_cmd);
//     state_pub_->publish(state_msg);

//     if (state_msg.data != last_state_) {

//         RCLCPP_INFO(
//             this->get_logger(),
//             "State: %s",
//             state_msg.data.c_str()
//         );

//         last_state_ = state_msg.data;
//     }
// }

// int main(int argc, char *argv[])
// {
//     rclcpp::init(argc, argv);

//     rclcpp::spin(
//         std::make_shared<CollisionAvoidanceNode>()
//     );

//     rclcpp::shutdown();
//     return 0;
// }




#include "collision_avoidance/collision_avoidance_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>

namespace {
constexpr double PI = 3.14159265358979323846;
}

CollisionAvoidanceNode::CollisionAvoidanceNode()
    : Node("collision_avoidance_node"),
      safe_distance_(this->declare_parameter<double>("safe_distance", 0.55)),
      slow_distance_(this->declare_parameter<double>("slow_distance", 1.00)),
      clear_speed_limit_(this->declare_parameter<double>("clear_speed_limit", 0.07)),
      slow_speed_limit_(this->declare_parameter<double>("slow_speed_limit", 0.05)),
      initial_turn_angle_rad_(this->declare_parameter<double>("initial_turn_angle_deg", 15.0) * PI / 180.0),
      initial_turn_rate_(this->declare_parameter<double>("initial_turn_rate", 0.15)),
      turn_timeout_sec_(this->declare_parameter<double>("turn_timeout_sec", 4.0)),
      detector_(15.0f, 45.0f),
      planner_(this->declare_parameter<double>("side_clearance_threshold", 0.70))
{
    this->declare_parameter<bool>("enable_initial_turn", false);

    // Keep existing YAML keys valid, even though the CA-1B test does not use
    // the old planner's base_speed / turn_speed values.
    this->declare_parameter<double>("base_speed", 0.4);
    this->declare_parameter<double>("turn_speed", 0.8);

    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", rclcpp::SensorDataQoS(),
        std::bind(&CollisionAvoidanceNode::scan_callback, this, std::placeholders::_1));

    // Preserve your existing launch-file remaps from /cmd_vel_manual.
    cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel_manual", 10,
        std::bind(&CollisionAvoidanceNode::cmd_callback, this, std::placeholders::_1));

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/diff_drive/odom", 10,
        std::bind(&CollisionAvoidanceNode::odom_callback, this, std::placeholders::_1));

    // Preserve your existing launch-file remaps to diff_drive controller.
    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel_collision", 10);
    state_pub_ = this->create_publisher<std_msgs::msg::String>("/cart_state", 10);

    control_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(50),
        std::bind(&CollisionAvoidanceNode::control_callback, this));

    RCLCPP_WARN(this->get_logger(),
        "CA-1B initial-turn TEST ONLY; turn enabled: %s. Do not run powered turns on floor.",
        this->get_parameter("enable_initial_turn").as_bool() ? "true" : "false");
}

void CollisionAvoidanceNode::scan_callback(
    const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    detector_.update(*msg);
    last_scan_time_ = this->now();
    have_scan_ = true;
}

void CollisionAvoidanceNode::cmd_callback(
    const geometry_msgs::msg::Twist::SharedPtr msg)
{
    last_cmd_ = *msg;
    last_cmd_time_ = this->now();
    have_cmd_ = true;
}

void CollisionAvoidanceNode::odom_callback(
    const nav_msgs::msg::Odometry::SharedPtr msg)
{
    const auto & q = msg->pose.pose.orientation;
    current_yaw_ = std::atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    last_odom_time_ = this->now();
    have_odom_ = true;
}

double CollisionAvoidanceNode::angle_difference(double current, double start)
{
    return std::atan2(std::sin(current - start), std::cos(current - start));
}

void CollisionAvoidanceNode::reset_turn()
{
    turn_active_ = false;
    turn_complete_ = false;
    turn_direction_ = 0;
}

void CollisionAvoidanceNode::abort_turn()
{
    turn_active_ = false;
    turn_complete_ = true;  // Requires withdrawal of command to re-arm.
}

void CollisionAvoidanceNode::control_callback()
{
    geometry_msgs::msg::Twist safe_cmd;  // Zero by default, fail closed.
    std_msgs::msg::String state_msg;
    const auto now = this->now();

    const bool scan_fresh = have_scan_ &&
        (now - last_scan_time_).seconds() >= 0.0 &&
        (now - last_scan_time_).seconds() < 0.5;
    const bool cmd_fresh = have_cmd_ &&
        (now - last_cmd_time_).seconds() >= 0.0 &&
        (now - last_cmd_time_).seconds() < 0.5;
    const bool odom_fresh = have_odom_ &&
        (now - last_odom_time_).seconds() >= 0.0 &&
        (now - last_odom_time_).seconds() < 0.5;

    // Deliberate opt-in, not enabled by normal M7 launch parameters.
    const bool turn_enabled = this->get_parameter("enable_initial_turn").as_bool();

    if (!scan_fresh || !detector_.has_valid_front_data()) {
        if (turn_active_) abort_turn();
        state_msg.data = "SENSOR_UNAVAILABLE";
    } else if (!cmd_fresh || last_cmd_.linear.x <= 0.0) {
        reset_turn();
        state_msg.data = "IDLE";
    } else if (turn_complete_) {
        state_msg.data = "TURN_COMPLETE_HOLD";
    } else if (turn_active_) {
        if (!turn_enabled || !odom_fresh) {
            abort_turn();
            state_msg.data = !odom_fresh ? "ODOM_UNAVAILABLE" : "TURN_DISABLED";
        } else if ((now - turn_start_time_).seconds() > turn_timeout_sec_) {
            abort_turn();
            state_msg.data = "TURN_TIMEOUT_HOLD";
        } else {
            const double progress = turn_direction_ * angle_difference(current_yaw_, start_yaw_);
            if (progress >= initial_turn_angle_rad_) {
                abort_turn();
                state_msg.data = "TURN_COMPLETE_HOLD";
            } else {
                // TEST ONLY: rotation on fully elevated drive wheels.
                safe_cmd.angular.z = turn_direction_ * initial_turn_rate_;
                state_msg.data = turn_direction_ > 0 ? "TURNING_LEFT" : "TURNING_RIGHT";
            }
        }
    } else {
        const double front = detector_.front_distance();
        if (front >= slow_distance_) {
            safe_cmd.linear.x = std::min(std::max(last_cmd_.linear.x, 0.0), clear_speed_limit_);
            state_msg.data = "CLEAR";
        } else if (front >= safe_distance_) {
            // Preserve the original conservative side-blocking rule in M7.
            const bool left_clear = detector_.has_valid_left_data() &&
                detector_.left_distance() >= safe_distance_;
            const bool right_clear = detector_.has_valid_right_data() &&
                detector_.right_distance() >= safe_distance_;
            if (!left_clear && !right_clear) {
                state_msg.data = "BLOCKED";
            } else {
                safe_cmd.linear.x = std::min(std::max(last_cmd_.linear.x, 0.0), slow_speed_limit_);
                state_msg.data = "SLOWING";
            }
        } else {
            const auto direction = planner_.choose_turn_direction(
                detector_.left_distance(), detector_.has_valid_left_data(),
                detector_.right_distance(), detector_.has_valid_right_data());

            if (direction == AvoidancePlanner::TurnDirection::NONE) {
                state_msg.data = "BLOCKED";
            } else if (!turn_enabled) {
                // Preserve M7's physical stop when obstacle is within 0.55m.
                state_msg.data = "AVOIDING";
            } else if (!odom_fresh) {
                state_msg.data = "ODOM_UNAVAILABLE";
            } else {
                turn_direction_ = static_cast<int>(direction);
                turn_active_ = true;
                start_yaw_ = current_yaw_;
                turn_start_time_ = now;
                // Output remains zero for this first control cycle.
                state_msg.data = "TURN_PREPARE";
            }
        }
    }

    cmd_pub_->publish(safe_cmd);
    state_pub_->publish(state_msg);

    if (state_msg.data != last_state_) {
        RCLCPP_INFO(this->get_logger(), "State: %s", state_msg.data.c_str());
        last_state_ = state_msg.data;
    }
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CollisionAvoidanceNode>());
    rclcpp::shutdown();
    return 0;
}