// // #pragma once

// // #include <rclcpp/rclcpp.hpp>
// // #include <sensor_msgs/msg/laser_scan.hpp>
// // #include <geometry_msgs/msg/twist.hpp>
// // #include <std_msgs/msg/string.hpp>

// // #include "collision_avoidance/obstacle_detector.hpp"
// // #include "collision_avoidance/avoidance_planner.hpp"

// // class CollisionAvoidanceNode : public rclcpp::Node
// // {
// // public:
// //     CollisionAvoidanceNode();

// // private:
// //     void scan_callback(
// //         const sensor_msgs::msg::LaserScan::SharedPtr msg);

// //     void cmd_callback(
// //         const geometry_msgs::msg::Twist::SharedPtr msg);

// //     void control_callback();

// //     rclcpp::Subscription<
// //         sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;

// //     rclcpp::Subscription<
// //         geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;

// //     rclcpp::Publisher<
// //         geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;

// //     rclcpp::Publisher<
// //         std_msgs::msg::String>::SharedPtr state_pub_;

// //     rclcpp::TimerBase::SharedPtr control_timer_;

// //     geometry_msgs::msg::Twist last_cmd_;

// //     bool have_scan_ = false;
// //     bool have_cmd_ = false;

// //     rclcpp::Time last_scan_time_{0, 0, RCL_ROS_TIME};
// //     rclcpp::Time last_cmd_time_{0, 0, RCL_ROS_TIME};

// //     std::string last_state_ = "";

// //     ObstacleDetector detector_;
// //     AvoidancePlanner planner_;
// // };




// #pragma once

// #include <rclcpp/rclcpp.hpp>
// #include <sensor_msgs/msg/laser_scan.hpp>
// #include <geometry_msgs/msg/twist.hpp>
// #include <nav_msgs/msg/odometry.hpp>
// #include <std_msgs/msg/string.hpp>
// #include <string>

// #include "collision_avoidance/obstacle_detector.hpp"
// #include "collision_avoidance/avoidance_planner.hpp"

// class CollisionAvoidanceNode : public rclcpp::Node
// {
// public:
//     CollisionAvoidanceNode();

// private:
//     void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
//     void cmd_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
//     void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
//     void control_callback();
//     void abort_turn();
//     void reset_turn();
//     static double angle_difference(double current, double start);

//     rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
//     rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
//     rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
//     rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
//     rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
//     rclcpp::TimerBase::SharedPtr control_timer_;

//     geometry_msgs::msg::Twist last_cmd_;
//     bool have_scan_ = false;
//     bool have_cmd_ = false;
//     bool have_odom_ = false;
//     rclcpp::Time last_scan_time_{0, 0, RCL_ROS_TIME};
//     rclcpp::Time last_cmd_time_{0, 0, RCL_ROS_TIME};
//     rclcpp::Time last_odom_time_{0, 0, RCL_ROS_TIME};
//     std::string last_state_;
//     double current_yaw_ = 0.0;
//     double start_yaw_ = 0.0;
//     rclcpp::Time turn_start_time_{0, 0, RCL_ROS_TIME};
//     int turn_direction_ = 0;
//     bool turn_active_ = false;
//     bool turn_complete_ = false;

//     double safe_distance_;
//     double slow_distance_;
//     double clear_speed_limit_;
//     double slow_speed_limit_;
//     double initial_turn_angle_rad_;
//     double initial_turn_rate_;
//     double turn_timeout_sec_;

//     ObstacleDetector detector_;
//     AvoidancePlanner planner_;
// };


#pragma once

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/string.hpp>

#include <chrono>
#include <string>
#include <utility>
#include <vector>

#include "collision_avoidance/obstacle_detector.hpp"

class CollisionAvoidanceNode : public rclcpp::Node
{
public:
    CollisionAvoidanceNode();

private:
    enum class Phase { APPROACH, DETOUR, RESUME, HOLD };

    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void cmd_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void control_callback();
    void hold(const std::string & reason);
    void reset();

    bool usable_scan() const;
    bool turn_sweep_clear() const;
    bool forward_corridor_clear(double speed) const;
    bool compute_path(std::vector<std::pair<double, double>> & path) const;
    std::pair<double, double> goal_in_base() const;
    static double angle_difference(double current, double reference);

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    sensor_msgs::msg::LaserScan::SharedPtr scan_;
    geometry_msgs::msg::Twist last_cmd_;
    ObstacleDetector detector_{15.0f, 45.0f};
    bool have_scan_ = false;
    bool have_cmd_ = false;
    bool have_odom_ = false;
    rclcpp::Time scan_received_{0, 0, RCL_ROS_TIME};
    rclcpp::Time cmd_received_{0, 0, RCL_ROS_TIME};
    rclcpp::Time odom_received_{0, 0, RCL_ROS_TIME};

    Phase phase_ = Phase::APPROACH;
    std::string hold_reason_;
    std::string last_state_;
    double x_ = 0.0, y_ = 0.0, yaw_ = 0.0;
    double start_x_ = 0.0, start_y_ = 0.0, start_yaw_ = 0.0;
    double goal_x_ = 0.0, goal_y_ = 0.0;
    double resume_start_x_ = 0.0, resume_start_y_ = 0.0;
    double detour_started_sec_ = 0.0;
    double last_plan_sec_ = -100.0;
    std::vector<std::pair<double, double>> path_;

    double safe_distance_, slow_distance_, avoid_trigger_distance_;
    double clear_speed_limit_, slow_speed_limit_, detour_speed_limit_;
    double max_turn_rate_, heading_gain_, heading_turn_threshold_;
    double goal_forward_distance_, completion_radius_, resume_distance_, detour_timeout_sec_;
    double lidar_x_, lidar_y_, lidar_yaw_rad_;
    double cart_front_extent_, cart_rear_extent_, cart_half_width_, safety_margin_;
    double lookahead_, grid_resolution_, grid_forward_, grid_rear_, grid_half_width_;
    bool enable_full_detour_;
};
