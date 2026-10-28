
#include "collision_avoidance/collision_avoidance_node.hpp"

#include <algorithm>
#include <chrono>
#include <functional>

CollisionAvoidanceNode::CollisionAvoidanceNode()
    : Node("collision_avoidance_node"),
      detector_(15.0, 45.0),
      planner_(
          this->declare_parameter("safe_distance", 0.5),
          this->declare_parameter("slow_distance", 1.0),
          this->declare_parameter("base_speed", 0.4),
          this->declare_parameter("turn_speed", 0.8)
      )
{
    // LiDAR data
    scan_sub_ =
        this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan",
            rclcpp::SensorDataQoS(),
            std::bind(
                &CollisionAvoidanceNode::scan_callback,
                this,
                std::placeholders::_1
            )
        );

    // Test movement command
    cmd_sub_ =
        this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel_manual",
            10,
            std::bind(
                &CollisionAvoidanceNode::cmd_callback,
                this,
                std::placeholders::_1
            )
        );

    // Filtered output
    cmd_pub_ =
        this->create_publisher<geometry_msgs::msg::Twist>(
            "/cmd_vel_collision", 10
        );

    // Diagnostic state
    state_pub_ =
        this->create_publisher<std_msgs::msg::String>(
            "/cart_state", 10
        );

    // Evaluate sensor and commands at 20 Hz
    control_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(50),
        std::bind(
            &CollisionAvoidanceNode::control_callback,
            this
        )
    );

    RCLCPP_INFO(
        this->get_logger(),
        "Collision Avoidance Node started"
    );
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

void CollisionAvoidanceNode::control_callback()
{
    geometry_msgs::msg::Twist safe_cmd;
    std_msgs::msg::String state_msg;

    auto current_time = this->now();

    bool scan_fresh =
        have_scan_ &&
        (current_time - last_scan_time_).seconds() < 0.5;

    bool cmd_fresh =
        have_cmd_ &&
        (current_time - last_cmd_time_).seconds() < 0.5;

    if (!scan_fresh || !detector_.has_valid_front_data()) {

        // Zero velocity if LiDAR data is unavailable
        state_msg.data = "SENSOR_UNAVAILABLE";

    } else if (!cmd_fresh) {

        // Zero velocity if no current command
        state_msg.data = "IDLE";

    } else if (last_cmd_.linear.x <= 0.0) {

        // Initial tests permit forward requests only.
        // Reverse/turn-only movements require
        // additional clearance checks.
        state_msg.data = "IDLE";

    } else {

        float front = detector_.front_distance();
        float left = detector_.left_distance();
        float right = detector_.right_distance();

        float safe_dist =
            this->get_parameter("safe_distance").as_double();

        float slow_dist =
            this->get_parameter("slow_distance").as_double();

        if (front < safe_dist) {

            safe_cmd = planner_.compute_cmd(
                front, left, right
            );

            state_msg.data =
                (safe_cmd.angular.z == 0.0) ?
                "BLOCKED" : "AVOIDING";

        } else if (front < slow_dist) {

            safe_cmd = planner_.compute_cmd(
                front, left, right
            );

            // Do not exceed requested forward speed
            safe_cmd.linear.x = std::min(
                safe_cmd.linear.x,
                last_cmd_.linear.x
            );

            state_msg.data =
                (safe_cmd.linear.x == 0.0) ?
                "BLOCKED" : "SLOWING";

        } else {

            // Clear: pass through the requested command
            safe_cmd = last_cmd_;
            state_msg.data = "CLEAR";
        }
    }




    // // TEMPORARY: Straight-line floor testing only.
    // // Do not execute autonomous avoidance turns yet.

    // if (state_msg.data == "CLEAR") {

    //     // Allow forward motion, but no turning.
    //     safe_cmd.angular.z = 0.0;

    // } else {

    //     // Stop for any obstacle, invalid sensor data,
    //     // expired command, or other non-clear state.
    //     safe_cmd = geometry_msgs::msg::Twist();
    // }



    // TEMPORARY: Forward-only obstacle approach testing.
    // Allow normal movement, slow movement, and stopping.
    // Autonomous avoidance turning remains disabled.

    if (state_msg.data == "CLEAR") {

        // Normal forward speed, capped at 0.07 m/s
        safe_cmd.linear.x = std::min(
            std::max(last_cmd_.linear.x, 0.0),
            0.07
        );

        safe_cmd.angular.z = 0.0;

    } else if (state_msg.data == "SLOWING") {

        // Reduce forward speed to at most 0.05 m/s.
        // Preserve any lower limit imposed by the planner.
        safe_cmd.linear.x = std::max(
            0.0,
            std::min(
                0.05,
                std::min(
                    last_cmd_.linear.x,
                    safe_cmd.linear.x
                )
            )
        );

        // Do not perform automatic turning yet.
        safe_cmd.angular.z = 0.0;

    } else {

        // STOP for AVOIDING, BLOCKED, IDLE,
        // SENSOR_UNAVAILABLE, or unexpected states.
        safe_cmd = geometry_msgs::msg::Twist();
    }




    cmd_pub_->publish(safe_cmd);
    state_pub_->publish(state_msg);

    if (state_msg.data != last_state_) {

        RCLCPP_INFO(
            this->get_logger(),
            "State: %s",
            state_msg.data.c_str()
        );

        last_state_ = state_msg.data;
    }
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::spin(
        std::make_shared<CollisionAvoidanceNode>()
    );

    rclcpp::shutdown();
    return 0;
}
