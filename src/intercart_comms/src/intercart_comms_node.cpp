#include "intercart_comms/intercart_comms_node.hpp"

#include <functional>
#include <vector>

IntercartCommNode::IntercartCommNode()
: Node("intercart_comm_node"),
  resolver_(
    this->declare_parameter("safety_radius", 1.2),
    this->declare_parameter("cart_id", 1)
  )
{
  // Store this cart's ID
  self_state_.cart_id =
    this->get_parameter("cart_id").as_int();

  // // Initialize this cart's state
  // self_state_.pose.position.x = 0.0;
  // self_state_.pose.position.y = 0.0;
  // self_state_.pose.position.z = 0.0;

  // self_state_.velocity.linear.x = 0.0;
  // self_state_.velocity.linear.y = 0.0;
  // self_state_.velocity.linear.z = 0.0;

  // Read simulation parameters
  double initial_x =
    this->declare_parameter("initial_x", 0.0);

  double initial_y =
    this->declare_parameter("initial_y", 0.0);

  double velocity_x =
    this->declare_parameter("velocity_x", 0.0);

  double velocity_y =
    this->declare_parameter("velocity_y", 0.0);

  // Initialize this cart's position
  self_state_.pose.position.x = initial_x;
  self_state_.pose.position.y = initial_y;
  self_state_.pose.position.z = 0.0;

  // Initialize this cart's velocity
  self_state_.velocity.linear.x = velocity_x;
  self_state_.velocity.linear.y = velocity_y;
  self_state_.velocity.linear.z = 0.0;

  self_state_.intent = 0;

  // --------------------------------------------------
  // Publisher: broadcast this cart's state
  // --------------------------------------------------

  state_pub_ =
    this->create_publisher<intercart_comms::msg::CartState>(
      "/cart_states",
      10
    );

  // --------------------------------------------------
  // Subscriber: receive other carts' states
  // --------------------------------------------------

  neighbor_sub_ =
    this->create_subscription<intercart_comms::msg::CartState>(
      "/cart_states",
      10,
      std::bind(
        &IntercartCommNode::neighbor_callback,
        this,
        std::placeholders::_1
      )
    );

  // --------------------------------------------------
  // Subscriber: receive this cart's velocity
  // --------------------------------------------------

  vel_sub_ =
    this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel_safe",
      10,
      std::bind(
        &IntercartCommNode::self_velocity_callback,
        this,
        std::placeholders::_1
      )
    );

  // --------------------------------------------------
  // Publisher: publish this cart's current intent
  // --------------------------------------------------

  intent_pub_ =
    this->create_publisher<std_msgs::msg::Int32>(
      "/cart_intent",
      10
    );

  // --------------------------------------------------
  // Timer: periodically broadcast cart state
  // --------------------------------------------------

  // state_timer_ =
  //   this->create_wall_timer(
  //     std::chrono::milliseconds(100),
  //     std::bind(
  //       &IntercartCommNode::publish_self_state,
  //       this
  //     )
  //   );
  state_timer_ =
  this->create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(
      &IntercartCommNode::update_simulation,
      this
    )
  );

  RCLCPP_INFO(
    this->get_logger(),
    "Inter-cart communication node started. Cart ID: %d",
    self_state_.cart_id
  );
}


// =====================================================
// Receive another cart's state
// =====================================================

void IntercartCommNode::neighbor_callback(
  const intercart_comms::msg::CartState::SharedPtr msg)
{
  // Ignore our own broadcast
  if (msg->cart_id == self_state_.cart_id)
  {
    return;
  }

  // Convert ROS message into our internal CartState
  CartState cart;

  cart.cart_id = msg->cart_id;
  cart.pose = msg->pose;
  cart.velocity = msg->velocity;
  cart.intent = msg->intent;

  // Store or update the latest state of this cart
  neighbors_[cart.cart_id] = cart;

  RCLCPP_INFO_THROTTLE(
    this->get_logger(),
    *this->get_clock(),
    1000,
    "Received state from Cart %d: x=%.2f y=%.2f",
    cart.cart_id,
    cart.pose.position.x,
    cart.pose.position.y
  );

  // Convert map into vector for ConflictResolver
  std::vector<CartState> neighbor_vector;

  for (const auto &entry : neighbors_)
  {
    neighbor_vector.push_back(entry.second);
  }

  // Run conflict resolution
  int decision =
    resolver_.resolve(
      self_state_,
      neighbor_vector
    );

  // Publish the decision
  std_msgs::msg::Int32 intent_msg;

  intent_msg.data = decision;

  intent_pub_->publish(intent_msg);
}


// =====================================================
// Receive this cart's velocity
// =====================================================

void IntercartCommNode::self_velocity_callback(
  const geometry_msgs::msg::Twist::SharedPtr msg)
{
  self_state_.velocity = *msg;
}


// =====================================================
// Update simulated cart position
// =====================================================

void IntercartCommNode::update_simulation()
{
  // Simulation timestep: 0.1 seconds
  const double dt = 0.1;

  // Update position using velocity
  self_state_.pose.position.x +=
    self_state_.velocity.linear.x * dt;

  self_state_.pose.position.y +=
    self_state_.velocity.linear.y * dt;

  // Broadcast updated state
  publish_self_state();
}


// =====================================================
// Broadcast this cart's state
// =====================================================

void IntercartCommNode::publish_self_state()
{
  intercart_comms::msg::CartState msg;

  msg.cart_id = self_state_.cart_id;
  msg.pose = self_state_.pose;
  msg.velocity = self_state_.velocity;
  msg.intent = self_state_.intent;

  state_pub_->publish(msg);
}


// =====================================================
// Main
// =====================================================

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::spin(
    std::make_shared<IntercartCommNode>()
  );

  rclcpp::shutdown();

  return 0;
}