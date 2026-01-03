#include "intercart_comms/intercart_comms_node.hpp"

IntercartCommNode::IntercartCommNode()
: Node("intercart_comm_node"),
  resolver_(
    this->declare_parameter("safety_radius", 1.0),
    this->declare_parameter("cart_id", 1)
  )
{
  neighbor_sub_ = this->create_subscription<geometry_msgs::msg::Pose>(
    "/neighbor_cart_pose", 10,
    std::bind(&IntercartCommNode::neighbor_callback, this, std::placeholders::_1)
  );

  vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "/cmd_vel_safe", 10,
    std::bind(&IntercartCommNode::self_velocity_callback, this, std::placeholders::_1)
  );

  intent_pub_ = this->create_publisher<std_msgs::msg::Int32>(
    "/cart_intent", 10);

  RCLCPP_INFO(this->get_logger(), "Inter-cart communication node started");
}

void IntercartCommNode::neighbor_callback(
  const geometry_msgs::msg::Pose::SharedPtr msg)
{
  CartState cart;
  cart.cart_id = 0;  // placeholder (expand later)
  cart.pose = *msg;

  neighbors_.push_back(cart);

  int decision = resolver_.resolve(neighbors_);

  std_msgs::msg::Int32 intent_msg;
  intent_msg.data = decision;
  intent_pub_->publish(intent_msg);
}

void IntercartCommNode::self_velocity_callback(
  const geometry_msgs::msg::Twist::SharedPtr)
{
  // Reserved for future intent estimation
}
