// // #include "collision_avoidance/collision_avoidance_node.hpp"

// // #include <algorithm>
// // #include <chrono>
// // #include <functional>

// // CollisionAvoidanceNode::CollisionAvoidanceNode()
// //     : Node("collision_avoidance_node"),
// //       detector_(15.0, 45.0),
// //       planner_(
// //           this->declare_parameter("safe_distance", 0.5),
// //           this->declare_parameter("slow_distance", 1.0),
// //           this->declare_parameter("base_speed", 0.4),
// //           this->declare_parameter("turn_speed", 0.8)
// //       )
// // {
// //     // LiDAR data
// //     scan_sub_ =
// //         this->create_subscription<sensor_msgs::msg::LaserScan>(
// //             "/scan",
// //             rclcpp::SensorDataQoS(),
// //             std::bind(
// //                 &CollisionAvoidanceNode::scan_callback,
// //                 this,
// //                 std::placeholders::_1
// //             )
// //         );

// //     // Test movement command
// //     cmd_sub_ =
// //         this->create_subscription<geometry_msgs::msg::Twist>(
// //             "/cmd_vel_manual",
// //             10,
// //             std::bind(
// //                 &CollisionAvoidanceNode::cmd_callback,
// //                 this,
// //                 std::placeholders::_1
// //             )
// //         );

// //     // Filtered output
// //     cmd_pub_ =
// //         this->create_publisher<geometry_msgs::msg::Twist>(
// //             "/cmd_vel_collision", 10
// //         );

// //     // Diagnostic state
// //     state_pub_ =
// //         this->create_publisher<std_msgs::msg::String>(
// //             "/cart_state", 10
// //         );

// //     // Evaluate sensor and commands at 20 Hz
// //     control_timer_ = this->create_wall_timer(
// //         std::chrono::milliseconds(50),
// //         std::bind(
// //             &CollisionAvoidanceNode::control_callback,
// //             this
// //         )
// //     );

// //     RCLCPP_INFO(
// //         this->get_logger(),
// //         "Collision Avoidance Node started"
// //     );
// // }

// // void CollisionAvoidanceNode::scan_callback(
// //     const sensor_msgs::msg::LaserScan::SharedPtr msg)
// // {
// //     detector_.update(*msg);

// //     last_scan_time_ = this->now();
// //     have_scan_ = true;
// // }

// // void CollisionAvoidanceNode::cmd_callback(
// //     const geometry_msgs::msg::Twist::SharedPtr msg)
// // {
// //     last_cmd_ = *msg;

// //     last_cmd_time_ = this->now();
// //     have_cmd_ = true;
// // }

// // void CollisionAvoidanceNode::control_callback()
// // {
// //     geometry_msgs::msg::Twist safe_cmd;
// //     std_msgs::msg::String state_msg;

// //     auto current_time = this->now();

// //     bool scan_fresh =
// //         have_scan_ &&
// //         (current_time - last_scan_time_).seconds() < 0.5;

// //     bool cmd_fresh =
// //         have_cmd_ &&
// //         (current_time - last_cmd_time_).seconds() < 0.5;

// //     if (!scan_fresh || !detector_.has_valid_front_data()) {

// //         // Zero velocity if LiDAR data is unavailable
// //         state_msg.data = "SENSOR_UNAVAILABLE";

// //     } else if (!cmd_fresh) {

// //         // Zero velocity if no current command
// //         state_msg.data = "IDLE";

// //     } else if (last_cmd_.linear.x <= 0.0) {

// //         // Initial tests permit forward requests only.
// //         // Reverse/turn-only movements require
// //         // additional clearance checks.
// //         state_msg.data = "IDLE";

// //     } else {

// //         float front = detector_.front_distance();
// //         float left = detector_.left_distance();
// //         float right = detector_.right_distance();

// //         float safe_dist =
// //             this->get_parameter("safe_distance").as_double();

// //         float slow_dist =
// //             this->get_parameter("slow_distance").as_double();

// //         if (front < safe_dist) {

// //             safe_cmd = planner_.compute_cmd(
// //                 front, left, right
// //             );

// //             state_msg.data =
// //                 (safe_cmd.angular.z == 0.0) ?
// //                 "BLOCKED" : "AVOIDING";

// //         } else if (front < slow_dist) {

// //             safe_cmd = planner_.compute_cmd(
// //                 front, left, right
// //             );

// //             // Do not exceed requested forward speed
// //             safe_cmd.linear.x = std::min(
// //                 safe_cmd.linear.x,
// //                 last_cmd_.linear.x
// //             );

// //             state_msg.data =
// //                 (safe_cmd.linear.x == 0.0) ?
// //                 "BLOCKED" : "SLOWING";

// //         } else {

// //             // Clear: pass through the requested command
// //             safe_cmd = last_cmd_;
// //             state_msg.data = "CLEAR";
// //         }
// //     }




// //     // // TEMPORARY: Straight-line floor testing only.
// //     // // Do not execute autonomous avoidance turns yet.

// //     // if (state_msg.data == "CLEAR") {

// //     //     // Allow forward motion, but no turning.
// //     //     safe_cmd.angular.z = 0.0;

// //     // } else {

// //     //     // Stop for any obstacle, invalid sensor data,
// //     //     // expired command, or other non-clear state.
// //     //     safe_cmd = geometry_msgs::msg::Twist();
// //     // }



// //     // TEMPORARY: Forward-only obstacle approach testing.
// //     // Allow normal movement, slow movement, and stopping.
// //     // Autonomous avoidance turning remains disabled.

// //     if (state_msg.data == "CLEAR") {

// //         // Normal forward speed, capped at 0.07 m/s
// //         safe_cmd.linear.x = std::min(
// //             std::max(last_cmd_.linear.x, 0.0),
// //             0.07
// //         );

// //         safe_cmd.angular.z = 0.0;

// //     } else if (state_msg.data == "SLOWING") {

// //         // Reduce forward speed to at most 0.05 m/s.
// //         // Preserve any lower limit imposed by the planner.
// //         safe_cmd.linear.x = std::max(
// //             0.0,
// //             std::min(
// //                 0.05,
// //                 std::min(
// //                     last_cmd_.linear.x,
// //                     safe_cmd.linear.x
// //                 )
// //             )
// //         );

// //         // Do not perform automatic turning yet.
// //         safe_cmd.angular.z = 0.0;

// //     } else {

// //         // STOP for AVOIDING, BLOCKED, IDLE,
// //         // SENSOR_UNAVAILABLE, or unexpected states.
// //         safe_cmd = geometry_msgs::msg::Twist();
// //     }




// //     cmd_pub_->publish(safe_cmd);
// //     state_pub_->publish(state_msg);

// //     if (state_msg.data != last_state_) {

// //         RCLCPP_INFO(
// //             this->get_logger(),
// //             "State: %s",
// //             state_msg.data.c_str()
// //         );

// //         last_state_ = state_msg.data;
// //     }
// // }

// // int main(int argc, char *argv[])
// // {
// //     rclcpp::init(argc, argv);

// //     rclcpp::spin(
// //         std::make_shared<CollisionAvoidanceNode>()
// //     );

// //     rclcpp::shutdown();
// //     return 0;
// // }




// #include "collision_avoidance/collision_avoidance_node.hpp"

// #include <algorithm>
// #include <chrono>
// #include <cmath>
// #include <functional>

// namespace {
// constexpr double PI = 3.14159265358979323846;
// }

// CollisionAvoidanceNode::CollisionAvoidanceNode()
//     : Node("collision_avoidance_node"),
//       safe_distance_(this->declare_parameter<double>("safe_distance", 0.55)),
//       slow_distance_(this->declare_parameter<double>("slow_distance", 1.00)),
//       clear_speed_limit_(this->declare_parameter<double>("clear_speed_limit", 0.07)),
//       slow_speed_limit_(this->declare_parameter<double>("slow_speed_limit", 0.05)),
//       initial_turn_angle_rad_(this->declare_parameter<double>("initial_turn_angle_deg", 15.0) * PI / 180.0),
//       initial_turn_rate_(this->declare_parameter<double>("initial_turn_rate", 0.15)),
//       turn_timeout_sec_(this->declare_parameter<double>("turn_timeout_sec", 4.0)),
//       detector_(15.0f, 45.0f),
//       planner_(this->declare_parameter<double>("side_clearance_threshold", 0.70))
// {
//     this->declare_parameter<bool>("enable_initial_turn", false);

//     // Keep existing YAML keys valid, even though the CA-1B test does not use
//     // the old planner's base_speed / turn_speed values.
//     this->declare_parameter<double>("base_speed", 0.4);
//     this->declare_parameter<double>("turn_speed", 0.8);

//     scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
//         "/scan", rclcpp::SensorDataQoS(),
//         std::bind(&CollisionAvoidanceNode::scan_callback, this, std::placeholders::_1));

//     // Preserve your existing launch-file remaps from /cmd_vel_manual.
//     cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
//         "/cmd_vel_manual", 10,
//         std::bind(&CollisionAvoidanceNode::cmd_callback, this, std::placeholders::_1));

//     odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
//         "/diff_drive/odom", 10,
//         std::bind(&CollisionAvoidanceNode::odom_callback, this, std::placeholders::_1));

//     // Preserve your existing launch-file remaps to diff_drive controller.
//     cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel_collision", 10);
//     state_pub_ = this->create_publisher<std_msgs::msg::String>("/cart_state", 10);

//     control_timer_ = this->create_wall_timer(
//         std::chrono::milliseconds(50),
//         std::bind(&CollisionAvoidanceNode::control_callback, this));

//     RCLCPP_WARN(this->get_logger(),
//         "CA-1B initial-turn TEST ONLY; turn enabled: %s. Do not run powered turns on floor.",
//         this->get_parameter("enable_initial_turn").as_bool() ? "true" : "false");
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

// void CollisionAvoidanceNode::odom_callback(
//     const nav_msgs::msg::Odometry::SharedPtr msg)
// {
//     const auto & q = msg->pose.pose.orientation;
//     current_yaw_ = std::atan2(
//         2.0 * (q.w * q.z + q.x * q.y),
//         1.0 - 2.0 * (q.y * q.y + q.z * q.z));
//     last_odom_time_ = this->now();
//     have_odom_ = true;
// }

// double CollisionAvoidanceNode::angle_difference(double current, double start)
// {
//     return std::atan2(std::sin(current - start), std::cos(current - start));
// }

// void CollisionAvoidanceNode::reset_turn()
// {
//     turn_active_ = false;
//     turn_complete_ = false;
//     turn_direction_ = 0;
// }

// void CollisionAvoidanceNode::abort_turn()
// {
//     turn_active_ = false;
//     turn_complete_ = true;  // Requires withdrawal of command to re-arm.
// }

// void CollisionAvoidanceNode::control_callback()
// {
//     geometry_msgs::msg::Twist safe_cmd;  // Zero by default, fail closed.
//     std_msgs::msg::String state_msg;
//     const auto now = this->now();

//     const bool scan_fresh = have_scan_ &&
//         (now - last_scan_time_).seconds() >= 0.0 &&
//         (now - last_scan_time_).seconds() < 0.5;
//     const bool cmd_fresh = have_cmd_ &&
//         (now - last_cmd_time_).seconds() >= 0.0 &&
//         (now - last_cmd_time_).seconds() < 0.5;
//     const bool odom_fresh = have_odom_ &&
//         (now - last_odom_time_).seconds() >= 0.0 &&
//         (now - last_odom_time_).seconds() < 0.5;

//     // Deliberate opt-in, not enabled by normal M7 launch parameters.
//     const bool turn_enabled = this->get_parameter("enable_initial_turn").as_bool();

//     if (!scan_fresh || !detector_.has_valid_front_data()) {
//         if (turn_active_) abort_turn();
//         state_msg.data = "SENSOR_UNAVAILABLE";
//     } else if (!cmd_fresh || last_cmd_.linear.x <= 0.0) {
//         reset_turn();
//         state_msg.data = "IDLE";
//     } else if (turn_complete_) {
//         state_msg.data = "TURN_COMPLETE_HOLD";
//     } else if (turn_active_) {
//         if (!turn_enabled || !odom_fresh) {
//             abort_turn();
//             state_msg.data = !odom_fresh ? "ODOM_UNAVAILABLE" : "TURN_DISABLED";
//         } else if ((now - turn_start_time_).seconds() > turn_timeout_sec_) {
//             abort_turn();
//             state_msg.data = "TURN_TIMEOUT_HOLD";
//         } else {
//             const double progress = turn_direction_ * angle_difference(current_yaw_, start_yaw_);
//             if (progress >= initial_turn_angle_rad_) {
//                 abort_turn();
//                 state_msg.data = "TURN_COMPLETE_HOLD";
//             } else {
//                 // TEST ONLY: rotation on fully elevated drive wheels.
//                 safe_cmd.angular.z = turn_direction_ * initial_turn_rate_;
//                 state_msg.data = turn_direction_ > 0 ? "TURNING_LEFT" : "TURNING_RIGHT";
//             }
//         }
//     } else {
//         const double front = detector_.front_distance();
//         if (front >= slow_distance_) {
//             safe_cmd.linear.x = std::min(std::max(last_cmd_.linear.x, 0.0), clear_speed_limit_);
//             state_msg.data = "CLEAR";
//         } else if (front >= safe_distance_) {
//             // Preserve the original conservative side-blocking rule in M7.
//             const bool left_clear = detector_.has_valid_left_data() &&
//                 detector_.left_distance() >= safe_distance_;
//             const bool right_clear = detector_.has_valid_right_data() &&
//                 detector_.right_distance() >= safe_distance_;
//             if (!left_clear && !right_clear) {
//                 state_msg.data = "BLOCKED";
//             } else {
//                 safe_cmd.linear.x = std::min(std::max(last_cmd_.linear.x, 0.0), slow_speed_limit_);
//                 state_msg.data = "SLOWING";
//             }
//         } else {
//             const auto direction = planner_.choose_turn_direction(
//                 detector_.left_distance(), detector_.has_valid_left_data(),
//                 detector_.right_distance(), detector_.has_valid_right_data());

//             if (direction == AvoidancePlanner::TurnDirection::NONE) {
//                 state_msg.data = "BLOCKED";
//             } else if (!turn_enabled) {
//                 // Preserve M7's physical stop when obstacle is within 0.55m.
//                 state_msg.data = "AVOIDING";
//             } else if (!odom_fresh) {
//                 state_msg.data = "ODOM_UNAVAILABLE";
//             } else {
//                 turn_direction_ = static_cast<int>(direction);
//                 turn_active_ = true;
//                 start_yaw_ = current_yaw_;
//                 turn_start_time_ = now;
//                 // Output remains zero for this first control cycle.
//                 state_msg.data = "TURN_PREPARE";
//             }
//         }
//     }

//     cmd_pub_->publish(safe_cmd);
//     state_pub_->publish(state_msg);

//     if (state_msg.data != last_state_) {
//         RCLCPP_INFO(this->get_logger(), "State: %s", state_msg.data.c_str());
//         last_state_ = state_msg.data;
//     }
// }

// int main(int argc, char * argv[])
// {
//     rclcpp::init(argc, argv);
//     rclcpp::spin(std::make_shared<CollisionAvoidanceNode>());
//     rclcpp::shutdown();
//     return 0;
// }


#include "collision_avoidance/collision_avoidance_node.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <cmath>
#include <functional>
#include <limits>
#include <queue>
#include <stdexcept>
#include <tuple>

namespace {
constexpr double PI = 3.14159265358979323846;

void laser_to_base(double range, double angle, double lx, double ly,
                   double laser_yaw, double &x, double &y)
{
    const double a = angle + laser_yaw;
    x = lx + range * std::cos(a);
    y = ly + range * std::sin(a);
}

struct Grid {
    int nx, ny;
    double resolution, xmin, ymin;
    std::vector<int8_t> value;  // -1 unknown, 0 observed free, 1 inflated obstacle
    Grid(double xlow, double xhigh, double yhalf, double res)
        : nx(static_cast<int>(std::ceil((xhigh - xlow) / res)) + 1),
          ny(static_cast<int>(std::ceil(2.0 * yhalf / res)) + 1),
          resolution(res), xmin(xlow), ymin(-yhalf),
          value(static_cast<size_t>(nx * ny), -1) {}
    bool inside(int ix, int iy) const { return ix >= 0 && ix < nx && iy >= 0 && iy < ny; }
    int id(int ix, int iy) const { return iy * nx + ix; }
    int gx(double x) const { return static_cast<int>(std::floor((x - xmin) / resolution + 0.5)); }
    int gy(double y) const { return static_cast<int>(std::floor((y - ymin) / resolution + 0.5)); }
    double wx(int ix) const { return xmin + ix * resolution; }
    double wy(int iy) const { return ymin + iy * resolution; }
    bool free(int ix, int iy) const {
        return inside(ix, iy) && value[static_cast<size_t>(id(ix, iy))] == 0;
    }
};
}

CollisionAvoidanceNode::CollisionAvoidanceNode()
    : Node("collision_avoidance_node"),
      safe_distance_(declare_parameter<double>("safe_distance", 0.55)),
      slow_distance_(declare_parameter<double>("slow_distance", 1.00)),
      avoid_trigger_distance_(declare_parameter<double>("avoid_trigger_distance", 0.90)),
      clear_speed_limit_(declare_parameter<double>("clear_speed_limit", 0.07)),
      slow_speed_limit_(declare_parameter<double>("slow_speed_limit", 0.05)),
      detour_speed_limit_(declare_parameter<double>("detour_speed_limit", 0.05)),
      max_turn_rate_(declare_parameter<double>("max_turn_rate", 0.18)),
      heading_gain_(declare_parameter<double>("heading_gain", 0.75)),
      heading_turn_threshold_(declare_parameter<double>("heading_turn_threshold_deg", 20.0) * PI / 180.0),
      goal_forward_distance_(declare_parameter<double>("goal_forward_distance", 1.50)),
      completion_radius_(declare_parameter<double>("completion_radius", 0.12)),
      resume_distance_(declare_parameter<double>("resume_distance", 0.15)),
      detour_timeout_sec_(declare_parameter<double>("detour_timeout_sec", 90.0)),
      lidar_x_(declare_parameter<double>("lidar_x", 0.0)),
      lidar_y_(declare_parameter<double>("lidar_y", 0.0)),
      lidar_yaw_rad_(declare_parameter<double>("lidar_yaw_deg", 0.0) * PI / 180.0),
      cart_front_extent_(declare_parameter<double>("cart_front_extent", 0.19)),
      cart_rear_extent_(declare_parameter<double>("cart_rear_extent", 0.19)),
      cart_half_width_(declare_parameter<double>("cart_half_width", 0.135)),
      safety_margin_(declare_parameter<double>("safety_margin", 0.10)),
      lookahead_(declare_parameter<double>("path_lookahead", 0.35)),
      grid_resolution_(declare_parameter<double>("grid_resolution", 0.08)),
      grid_forward_(declare_parameter<double>("grid_forward", 2.50)),
      grid_rear_(declare_parameter<double>("grid_rear", 1.0)),
      grid_half_width_(declare_parameter<double>("grid_half_width", 1.80)),
      enable_full_detour_(declare_parameter<bool>("enable_full_detour", false))
{
    // Existing configuration keys are accepted so the previous YAML works.
    declare_parameter<double>("base_speed", 0.4);
    declare_parameter<double>("turn_speed", 0.8);
    declare_parameter<double>("side_clearance_threshold", 0.7);
    declare_parameter<bool>("enable_initial_turn", false);
    declare_parameter<bool>("enable_bypass_segment", false);

    if (!(safe_distance_ > 0.0 && slow_distance_ > safe_distance_ &&
          avoid_trigger_distance_ > safe_distance_ &&
          avoid_trigger_distance_ < slow_distance_ &&
          clear_speed_limit_ > 0.0 && slow_speed_limit_ > 0.0 &&
          detour_speed_limit_ > 0.0 && max_turn_rate_ > 0.0 &&
          goal_forward_distance_ > 0.0 && resume_distance_ >= 0.0 &&
          detour_timeout_sec_ > 0.0 &&
          cart_front_extent_ > 0.0 && cart_rear_extent_ > 0.0 &&
          cart_half_width_ > 0.0 && safety_margin_ >= 0.0 &&
          grid_resolution_ > 0.03 && grid_resolution_ <= 0.12 &&
          grid_half_width_ > 1.0 && grid_forward_ > goal_forward_distance_ + 0.3)) {
        throw std::runtime_error("Invalid local-detour parameters");
    }

    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", rclcpp::SensorDataQoS(),
        std::bind(&CollisionAvoidanceNode::scan_callback, this, std::placeholders::_1));
    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel_manual", 10,
        std::bind(&CollisionAvoidanceNode::cmd_callback, this, std::placeholders::_1));
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "/diff_drive/odom", 10,
        std::bind(&CollisionAvoidanceNode::odom_callback, this, std::placeholders::_1));
    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel_collision", 10);
    state_pub_ = create_publisher<std_msgs::msg::String>("/cart_state", 10);
    timer_ = create_wall_timer(std::chrono::milliseconds(50),
        std::bind(&CollisionAvoidanceNode::control_callback, this));
    RCLCPP_WARN(get_logger(),
        "MINIMUM DETOUR DEMO (experimental), enable_full_detour=%s; NOT certified for unattended floor use",
        enable_full_detour_ ? "true" : "false");
}

void CollisionAvoidanceNode::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    scan_ = msg;
    detector_.update(*msg);
    scan_received_ = now();
    have_scan_ = true;
}

void CollisionAvoidanceNode::cmd_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    last_cmd_ = *msg;
    cmd_received_ = now();
    have_cmd_ = true;
}

void CollisionAvoidanceNode::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    const auto &q = msg->pose.pose.orientation;
    yaw_ = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    x_ = msg->pose.pose.position.x;
    y_ = msg->pose.pose.position.y;
    odom_received_ = now();
    have_odom_ = true;
}

double CollisionAvoidanceNode::angle_difference(double current, double reference)
{
    return std::atan2(std::sin(current - reference), std::cos(current - reference));
}

void CollisionAvoidanceNode::hold(const std::string &reason)
{
    phase_ = Phase::HOLD;
    hold_reason_ = reason;
    path_.clear();
}

void CollisionAvoidanceNode::reset()
{
    phase_ = Phase::APPROACH;
    hold_reason_.clear();
    path_.clear();
    last_plan_sec_ = -100.0;
}

std::pair<double, double> CollisionAvoidanceNode::goal_in_base() const
{
    const double dx = goal_x_ - x_;
    const double dy = goal_y_ - y_;
    return {std::cos(yaw_) * dx + std::sin(yaw_) * dy,
            -std::sin(yaw_) * dx + std::cos(yaw_) * dy};
}

bool CollisionAvoidanceNode::usable_scan() const
{
    if (!scan_ || scan_->ranges.size() < 180 ||
        !std::isfinite(scan_->angle_increment) ||
        std::abs(scan_->angle_increment) < 1.e-7 ||
        !std::isfinite(scan_->range_min) || !std::isfinite(scan_->range_max) ||
        scan_->range_min <= 0.0 || scan_->range_max <= scan_->range_min) return false;
    // Require samples with finite range in all angular octants.
    std::array<int, 8> seen{};
    for (size_t i = 0; i < scan_->ranges.size(); ++i) {
        const double r = scan_->ranges[i];
        if (!std::isfinite(r) || r < scan_->range_min || r > scan_->range_max) continue;
        const double angle = scan_->angle_min + i * scan_->angle_increment;
        const double a = std::atan2(std::sin(angle), std::cos(angle));
        int sector = static_cast<int>(std::floor((a + PI) / (PI / 4.0)));
        sector = std::max(0, std::min(7, sector));
        ++seen[static_cast<size_t>(sector)];
    }
    return std::all_of(seen.begin(), seen.end(), [](int n) { return n >= 4; });
}

bool CollisionAvoidanceNode::turn_sweep_clear() const
{
    if (!usable_scan()) return false;
    const double radius = std::hypot(std::max(cart_front_extent_, cart_rear_extent_),
                                     cart_half_width_) + safety_margin_;
    for (size_t i = 0; i < scan_->ranges.size(); ++i) {
        const double r = scan_->ranges[i];
        if (!std::isfinite(r) || r < scan_->range_min || r > scan_->range_max) continue;
        double x, y;
        laser_to_base(r, scan_->angle_min + i * scan_->angle_increment,
                      lidar_x_, lidar_y_, lidar_yaw_rad_, x, y);
        if (std::hypot(x, y) <= radius) return false;
    }
    return true;
}

bool CollisionAvoidanceNode::forward_corridor_clear(double speed) const
{
    if (!usable_scan()) return false;
    const double half_width = cart_half_width_ + safety_margin_;
    // Provisional lookahead: current front bumper plus at least 0.30 m.
    const double look = std::max(0.30, speed * 6.0);
    const double xmax = cart_front_extent_ + look;
    int forward_samples = 0;
    for (size_t i = 0; i < scan_->ranges.size(); ++i) {
        const double r = scan_->ranges[i];
        if (!std::isfinite(r) || r < scan_->range_min || r > scan_->range_max) continue;
        double x, y;
        laser_to_base(r, scan_->angle_min + i * scan_->angle_increment,
                      lidar_x_, lidar_y_, lidar_yaw_rad_, x, y);
        if (x > 0.0 && std::abs(y) < 0.50) ++forward_samples;
        if (x >= -cart_rear_extent_ - safety_margin_ && x <= xmax &&
            std::abs(y) <= half_width) return false;
    }
    return forward_samples >= 10;
}

// Local, current-scan A* planner. Unknown cells are NOT traversable. A valid
// beam establishes observed free cells along its length; endpoints are inflated
// by the robot's conservative circular footprint radius. No claim of global
// mapping or reliable 3D obstacle clearance is made.
bool CollisionAvoidanceNode::compute_path(
    std::vector<std::pair<double, double>> &path) const
{
    path.clear();
    if (!usable_scan()) return false;
    Grid g(-grid_rear_, grid_forward_, grid_half_width_, grid_resolution_);
    std::vector<std::pair<int,int>> endpoints;
    endpoints.reserve(scan_->ranges.size());

    const double ray_limit = std::hypot(grid_forward_ + std::abs(lidar_x_),
                                       grid_half_width_ + std::abs(lidar_y_)) + 0.2;
    // Trace all observed finite returns. No finite return = unknown, not clear.
    for (size_t i = 0; i < scan_->ranges.size(); ++i) {
        const double r = scan_->ranges[i];
        if (!std::isfinite(r) || r < scan_->range_min || r > scan_->range_max) continue;
        const bool hit_in_range = r <= ray_limit;
        const double distance = std::min(r, ray_limit);
        const double angle = scan_->angle_min + i * scan_->angle_increment;
        double end_x, end_y;
        laser_to_base(distance, angle, lidar_x_, lidar_y_, lidar_yaw_rad_, end_x, end_y);
        const int steps = std::max(1, static_cast<int>(std::ceil(distance / (g.resolution * 0.45))));
        for (int k = 0; k < steps; ++k) {
            const double t = static_cast<double>(k) / steps;
            const double px = lidar_x_ + (end_x - lidar_x_) * t;
            const double py = lidar_y_ + (end_y - lidar_y_) * t;
            const int ix = g.gx(px), iy = g.gy(py);
            if (g.inside(ix, iy)) g.value[static_cast<size_t>(g.id(ix, iy))] = 0;
        }
        if (hit_in_range) {
            const int ix = g.gx(end_x), iy = g.gy(end_y);
            if (g.inside(ix, iy)) endpoints.emplace_back(ix, iy);
        }
    }

    // Inflate obstacles by the full circular swept-footprint bound.
    const double inflation = std::hypot(
        std::max(cart_front_extent_, cart_rear_extent_), cart_half_width_) + safety_margin_;
    const int rad_cells = static_cast<int>(std::ceil(inflation / g.resolution));
    for (const auto &p : endpoints) {
        for (int dy = -rad_cells; dy <= rad_cells; ++dy) {
            for (int dx = -rad_cells; dx <= rad_cells; ++dx) {
                if (std::hypot(dx * g.resolution, dy * g.resolution) > inflation) continue;
                const int ix = p.first + dx, iy = p.second + dy;
                if (g.inside(ix, iy)) g.value[static_cast<size_t>(g.id(ix, iy))] = 1;
            }
        }
    }

    const auto goal = goal_in_base();
    const int sx = g.gx(0.0), sy = g.gy(0.0);
    const int gx = g.gx(goal.first), gy = g.gy(goal.second);
    if (!g.free(sx, sy) || !g.inside(gx, gy)) return false;

    const int start_id = g.id(sx, sy), goal_id = g.id(gx, gy);
    const size_t n = g.value.size();
    std::vector<double> cost(n, std::numeric_limits<double>::infinity());
    std::vector<int> parent(n, -1);
    using Entry = std::pair<double, int>;
    std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> open;
    cost[static_cast<size_t>(start_id)] = 0.0;
    open.emplace(0.0, start_id);

    constexpr std::array<std::pair<int,int>, 8> neighbors{{
        {1,0}, {-1,0}, {0,1}, {0,-1}, {1,1}, {1,-1}, {-1,1}, {-1,-1}
    }};
    while (!open.empty()) {
        const auto top = open.top(); open.pop();
        const int id = top.second, ix = id % g.nx, iy = id / g.nx;
        const double h_here = std::hypot(gx - ix, gy - iy) * g.resolution;
        if (top.first > cost[static_cast<size_t>(id)] + h_here + 1e-6) continue;
        for (const auto &d : neighbors) {
            const int xx = ix + d.first, yy = iy + d.second;
            if (!g.free(xx, yy)) continue;
            if (d.first != 0 && d.second != 0 &&
                (!g.free(ix + d.first, iy) || !g.free(ix, iy + d.second))) continue;
            const int nid = g.id(xx, yy);
            const double step = g.resolution * std::hypot(d.first, d.second);
            // Prefer returning to original route and avoid backwards detours.
            const double lateral_cost = 0.05 * std::abs(g.wy(yy)) * step;
            const double reverse_cost = (d.first < 0 ? 0.12 : 0.0) * step;
            const double new_cost = cost[static_cast<size_t>(id)] + step + lateral_cost + reverse_cost;
            if (new_cost < cost[static_cast<size_t>(nid)]) {
                cost[static_cast<size_t>(nid)] = new_cost;
                parent[static_cast<size_t>(nid)] = id;
                const double heuristic = std::hypot(gx - xx, gy - yy) * g.resolution;
                open.emplace(new_cost + heuristic, nid);
            }
        }
    }
    // The goal may be hidden directly behind the obstacle. A one-scan map
    // cannot mark occluded space FREE. When necessary, select a reachable
    // visible intermediate point closer to the goal, and replan on new scans.
    int selected = -1;
    if (g.free(gx, gy) && std::isfinite(cost[static_cast<size_t>(goal_id)])) {
        selected = goal_id;
    } else {
        double best = std::numeric_limits<double>::infinity();
        const double distance_to_goal = std::hypot(goal.first, goal.second);
        for (int iy = 0; iy < g.ny; ++iy) {
            for (int ix = 0; ix < g.nx; ++ix) {
                if (!g.free(ix, iy)) continue;
                const int id = g.id(ix, iy);
                if (!std::isfinite(cost[static_cast<size_t>(id)])) continue;
                const double xx = g.wx(ix), yy = g.wy(iy);
                const double dist = std::hypot(goal.first - xx, goal.second - yy);
                // Require meaningful progress; do not inch into an impassable wall.
                if (xx < std::min(0.45, distance_to_goal * 0.4) ||
                    dist > distance_to_goal - 0.15) continue;
                const double score = dist + 0.20 * cost[static_cast<size_t>(id)];
                if (score < best) { best = score; selected = id; }
            }
        }
    }
    if (selected < 0) return false;

    std::vector<std::pair<double,double>> backward;
    for (int id = selected; id != -1; id = parent[static_cast<size_t>(id)]) {
        backward.emplace_back(g.wx(id % g.nx), g.wy(id / g.nx));
        if (id == start_id) break;
    }
    if (backward.empty() ||
        std::hypot(backward.back().first, backward.back().second) > 2 * g.resolution)
        return false;
    path.assign(backward.rbegin(), backward.rend());
    return true;
}

void CollisionAvoidanceNode::control_callback()
{
    geometry_msgs::msg::Twist out;  // always zero unless explicitly permitted
    std_msgs::msg::String state;
    const auto t = now();
    const auto fresh = [&t](bool have, const rclcpp::Time &last) {
        const double dt = (t - last).seconds();
        return have && dt >= 0.0 && dt < 0.5;
    };
    const bool cmd_ok = fresh(have_cmd_, cmd_received_);
    const bool scan_ok = fresh(have_scan_, scan_received_) && usable_scan();
    const bool odom_ok = fresh(have_odom_, odom_received_);

    // Withdraw the movement request to clear any hold and rearm.
    if (!cmd_ok || last_cmd_.linear.x <= 0.0) {
        reset();
        state.data = "IDLE";
    } else if (!scan_ok) {
        hold("SENSOR_UNAVAILABLE_HOLD");
        state.data = hold_reason_;
    } else if (phase_ == Phase::HOLD) {
        state.data = hold_reason_;
    } else if (phase_ == Phase::APPROACH) {
        const double front = detector_.front_distance();
        if (front < safe_distance_) {
            state.data = "STOP_OBSTACLE";
        } else if (front < avoid_trigger_distance_ && enable_full_detour_) {
            if (!odom_ok) {
                state.data = "ODOM_UNAVAILABLE";
            } else {
                start_x_ = x_; start_y_ = y_; start_yaw_ = yaw_;
                goal_x_ = x_ + goal_forward_distance_ * std::cos(yaw_);
                goal_y_ = y_ + goal_forward_distance_ * std::sin(yaw_);
                detour_started_sec_ = t.seconds();
                last_plan_sec_ = -100.0;
                path_.clear();
                phase_ = Phase::DETOUR;
                state.data = "DETOUR_PREPARE";
            }
        } else if (front < slow_distance_) {
            if (forward_corridor_clear(slow_speed_limit_)) {
                out.linear.x = std::min(last_cmd_.linear.x, slow_speed_limit_);
                state.data = "SLOWING";
            } else {
                state.data = "STOP_CORRIDOR";
            }
        } else if (forward_corridor_clear(clear_speed_limit_)) {
            out.linear.x = std::min(last_cmd_.linear.x, clear_speed_limit_);
            state.data = "CLEAR";
        } else {
            state.data = "STOP_CORRIDOR";
        }
    } else if (phase_ == Phase::RESUME) {
        if (!odom_ok) {
            hold("ODOM_UNAVAILABLE_HOLD");
            state.data = hold_reason_;
        } else if (t.seconds() - detour_started_sec_ > detour_timeout_sec_) {
            hold("DETOUR_TIMEOUT_HOLD");
            state.data = hold_reason_;
        } else {
            const double progress = (x_ - resume_start_x_) * std::cos(start_yaw_) +
                                    (y_ - resume_start_y_) * std::sin(start_yaw_);
            const double err = angle_difference(start_yaw_, yaw_);
            if (progress >= resume_distance_) {
                hold("DETOUR_COMPLETE_HOLD");
                state.data = hold_reason_;
            } else if (std::abs(err) > 0.17) {
                if (turn_sweep_clear()) {
                    out.angular.z = std::clamp(heading_gain_ * err,
                                               -max_turn_rate_, max_turn_rate_);
                    state.data = "RECOVER_HEADING";
                } else {
                    state.data = "STOP_TURN_CLEARANCE";
                }
            } else if (!forward_corridor_clear(detour_speed_limit_) ||
                       detector_.front_distance() < slow_distance_) {
                state.data = "STOP_RESUME_CORRIDOR";
            } else {
                out.linear.x = std::min(last_cmd_.linear.x, detour_speed_limit_);
                out.angular.z = std::clamp(heading_gain_ * err,
                                           -max_turn_rate_, max_turn_rate_);
                state.data = "RESUMING_PATH";
            }
        }
    } else {  // Phase::DETOUR
        if (!odom_ok) {
            hold("ODOM_UNAVAILABLE_HOLD");
            state.data = hold_reason_;
        } else if (t.seconds() - detour_started_sec_ > detour_timeout_sec_) {
            hold("DETOUR_TIMEOUT_HOLD");
            state.data = hold_reason_;
        } else {
            const auto goal = goal_in_base();
            if (std::hypot(goal.first, goal.second) <= completion_radius_) {
                const double yaw_error = angle_difference(start_yaw_, yaw_);
                if (std::abs(yaw_error) < 0.17) {
                    phase_ = Phase::RESUME;
                    resume_start_x_ = x_;
                    resume_start_y_ = y_;
                    state.data = "RESUME_PREPARE";
                } else if (turn_sweep_clear()) {
                    out.angular.z = std::clamp(heading_gain_ * yaw_error,
                                               -max_turn_rate_, max_turn_rate_);
                    state.data = "RECOVER_HEADING";
                } else {
                    state.data = "STOP_TURN_CLEARANCE";
                }
            } else {
                if (t.seconds() - last_plan_sec_ > 0.30) {
                    last_plan_sec_ = t.seconds();
                    std::vector<std::pair<double,double>> candidate;
                    if (compute_path(candidate)) path_ = std::move(candidate);
                    else path_.clear();
                }
                if (path_.empty()) {
                    state.data = "NO_SAFE_PATH";
                } else {
                    // Select a waypoint along the current shortest obstacle-free path.
                    std::pair<double,double> waypoint = path_.back();
                    for (const auto &p : path_) {
                        if (std::hypot(p.first, p.second) >= lookahead_) {
                            waypoint = p;
                            break;
                        }
                    }
                    const double heading = std::atan2(waypoint.second, waypoint.first);
                    if (std::abs(heading) > heading_turn_threshold_) {
                        if (turn_sweep_clear()) {
                            out.angular.z = std::clamp(heading_gain_ * heading,
                                                       -max_turn_rate_, max_turn_rate_);
                            state.data = "DETOUR_TURNING";
                        } else {
                            state.data = "STOP_TURN_CLEARANCE";
                        }
                    } else if (!forward_corridor_clear(detour_speed_limit_) ||
                               detector_.front_distance() < safe_distance_) {
                        state.data = "STOP_REPLAN";
                    } else {
                        out.linear.x = std::min(last_cmd_.linear.x, detour_speed_limit_);
                        out.angular.z = std::clamp(heading_gain_ * heading,
                                                   -max_turn_rate_, max_turn_rate_);
                        state.data = "DETOUR_MOVING";
                    }
                }
            }
        }
    }

    cmd_pub_->publish(out);
    state_pub_->publish(state);
    if (state.data != last_state_) {
        RCLCPP_INFO(get_logger(), "State: %s", state.data.c_str());
        last_state_ = state.data;
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CollisionAvoidanceNode>());
    rclcpp::shutdown();
    return 0;
}
