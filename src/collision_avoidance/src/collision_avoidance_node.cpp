#include "collision_avoidance/collision_avoidance_node.hpp"

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
  scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "/scan", 10,
    std::bind(&CollisionAvoidanceNode::scan_callback, this, std::placeholders::_1)
  );

  // cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
  //   "/cmd_vel_safe", 10);
  cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
    "/cmd_vel_collision", 10);


  RCLCPP_INFO(this->get_logger(), "Collision Avoidance Node started");
}

void CollisionAvoidanceNode::scan_callback(
  const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  detector_.update(*msg);

  auto cmd = planner_.compute_cmd(
    detector_.front_distance(),
    detector_.left_distance(),
    detector_.right_distance()
  );

  cmd_pub_->publish(cmd);
}
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CollisionAvoidanceNode>());
  rclcpp::shutdown();
  return 0;
}