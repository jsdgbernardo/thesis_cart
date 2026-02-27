#include "collision_avoidance/collision_avoidance_node.hpp"
#include <std_msgs/msg/string.hpp> 

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
  // LiDAR subscription
  scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "/scan", 10,
    std::bind(&CollisionAvoidanceNode::scan_callback, this, std::placeholders::_1)
  );

  // Manual velocity subscription
  cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "/cmd_vel_manual", 10,
    std::bind(&CollisionAvoidanceNode::cmd_callback, this, std::placeholders::_1)
  );

  // Output publishers
  cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
    "/cmd_vel_collision", 10);

  state_pub_ = this->create_publisher<std_msgs::msg::String>(
    "/cart_state", 10);   // NEW

  RCLCPP_INFO(this->get_logger(), "Collision Avoidance Node started");
}

void CollisionAvoidanceNode::scan_callback(
  const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  detector_.update(*msg);

  // Update obstacle flag based on front distance
  if (detector_.front_distance() < 0.5)  // threshold
    obstacle_detected_ = true;
  else
    obstacle_detected_ = false;
    // obstacle_detected_ = true; // for testing
}

// void CollisionAvoidanceNode::cmd_callback(
//   const geometry_msgs::msg::Twist::SharedPtr msg)
// {
//   last_cmd_ = *msg;

//   geometry_msgs::msg::Twist safe_cmd = last_cmd_;

//   if (obstacle_detected_)
//   {
//     // Use planner to generate avoidance command
//     safe_cmd = planner_.compute_cmd(
//       detector_.front_distance(),
//       detector_.left_distance(),
//       detector_.right_distance()
//     );

//     std::string state = (detector_.front_distance() < planner_.get_safe_dist()) ? "AVOIDING" : "CLEAR";

//     if (state != last_state_) {
//       RCLCPP_INFO(this->get_logger(), "State: %s", state.c_str());
//       last_state_ = state;
//     }

//     RCLCPP_WARN(this->get_logger(), "Obstacle detected! Avoiding...");
//   }
//   else
//   {
//     std::string state = "CLEAR";
//     if (state != last_state_) {
//       RCLCPP_INFO(this->get_logger(), "State: %s", state.c_str());
//       last_state_ = state;
//     }
//   }

//   cmd_pub_->publish(safe_cmd);
// }

void CollisionAvoidanceNode::cmd_callback(
  const geometry_msgs::msg::Twist::SharedPtr msg)
{
  last_cmd_ = *msg;

  geometry_msgs::msg::Twist safe_cmd;
  std_msgs::msg::String state_msg;

  float front = detector_.front_distance();
  float left  = detector_.left_distance();
  float right = detector_.right_distance();

  if (front < planner_.get_safe_dist()) {
    safe_cmd = planner_.compute_cmd(front, left, right);
    state_msg.data = "AVOIDING";
  } else {
    safe_cmd = last_cmd_;
    state_msg.data = "CLEAR";
  }

  state_pub_->publish(state_msg);
  cmd_pub_->publish(safe_cmd);
}




int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CollisionAvoidanceNode>());
  rclcpp::shutdown();
  return 0;
}