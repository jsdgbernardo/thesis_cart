#include "intercart_comms/intercart_comms_node.hpp"

#include <functional>
#include <vector>


IntercartCommNode::IntercartCommNode()
: Node("intercart_comm_node"),
  resolver_(
    this->declare_parameter("safety_radius", 1.2),
    this->declare_parameter("cart_id", 1),
    this->declare_parameter("prediction_horizon", 3.0)
  )
{
  // =====================================================
  // 1. CART ID
  // =====================================================

  self_state_.cart_id =
    this->get_parameter("cart_id").as_int();


  // =====================================================
  // 2. GAZEBO MODEL NAME
  // ADD THE NEW PARAMETER HERE
  // =====================================================

  gazebo_model_name_ =
    this->declare_parameter(
      "gazebo_model_name",
      "cart_1"
    );

  std::string gazebo_cmd_topic =
    "/" + gazebo_model_name_ + "/cmd_vel";


  // =====================================================
  // 3. SIMULATION PARAMETERS
  // =====================================================

  double initial_x =
    this->declare_parameter("initial_x", 0.0);

  double initial_y =
    this->declare_parameter("initial_y", 0.0);

  double velocity_x =
    this->declare_parameter("velocity_x", 0.0);

  double velocity_y =
    this->declare_parameter("velocity_y", 0.0);


  // =====================================================
  // 4. INITIAL POSITION
  // =====================================================

  self_state_.pose.position.x = initial_x;
  self_state_.pose.position.y = initial_y;
  self_state_.pose.position.z = 0.0;


  // =====================================================
  // 5. DESIRED VELOCITY
  // =====================================================

  desired_velocity_.linear.x = velocity_x;
  desired_velocity_.linear.y = velocity_y;
  desired_velocity_.linear.z = 0.0;

  desired_velocity_.angular.x = 0.0;
  desired_velocity_.angular.y = 0.0;
  desired_velocity_.angular.z = 0.0;

  safe_velocity_ = desired_velocity_;

  self_state_.velocity = safe_velocity_;

  self_state_.intent = 0;


  // =====================================================
  // 6. PUBLISH THIS CART'S STATE
  // =====================================================

  state_pub_ =
    this->create_publisher<intercart_comms::msg::CartState>(
      "/cart_states",
      10
    );


  gazebo_cmd_vel_pub_ =
    this->create_publisher<geometry_msgs::msg::Twist>(
      gazebo_cmd_topic,
      10
    );


  // =====================================================
  // 7. RECEIVE OTHER CARTS' STATES
  // =====================================================

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


  // =====================================================
  // 8. RECEIVE DESIRED CART VELOCITY
  // =====================================================

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


  // =====================================================
  // 9. ADD THE NEW GAZEBO SUBSCRIPTION HERE
  // =====================================================

  gazebo_states_sub_ =
    this->create_subscription<gazebo_msgs::msg::ModelStates>(
      "/gazebo/model_states",
      10,
      std::bind(
        &IntercartCommNode::gazebo_model_states_callback,
        this,
        std::placeholders::_1
      )
    );


  // =====================================================
  // 10. PUBLISH INTENT
  // =====================================================

  intent_pub_ =
    this->create_publisher<std_msgs::msg::Int32>(
      "/cart_intent",
      10
    );


  // =====================================================
  // 11. TIMER
  // =====================================================

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
    "Inter-cart communication node started. Cart ID: %d, Gazebo model: %s",
    self_state_.cart_id,
    gazebo_model_name_.c_str()
  );
}


// =====================================================
// NEIGHBOR CALLBACK
// =====================================================

void IntercartCommNode::neighbor_callback(
  const intercart_comms::msg::CartState::SharedPtr msg)
{
  if (msg->cart_id == self_state_.cart_id)
  {
    return;
  }

  CartState cart;

  cart.cart_id = msg->cart_id;
  cart.pose = msg->pose;
  cart.velocity = msg->velocity;
  cart.intent = msg->intent;

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

  std::vector<CartState> neighbor_vector;

  for (const auto &entry : neighbors_)
  {
    neighbor_vector.push_back(entry.second);
  }

  int decision =
    resolver_.resolve(
      self_state_,
      neighbor_vector
    );

  self_state_.intent = decision;


  if (decision == 2)
  {
    safe_velocity_.linear.x = 0.0;
    safe_velocity_.linear.y = 0.0;
    safe_velocity_.linear.z = 0.0;

    safe_velocity_.angular.x = 0.0;
    safe_velocity_.angular.y = 0.0;
    safe_velocity_.angular.z = 0.0;
  }
  else if (decision == 1)
  {
    safe_velocity_.linear.x =
      desired_velocity_.linear.x * 0.5;

    safe_velocity_.linear.y =
      desired_velocity_.linear.y * 0.5;

    safe_velocity_.linear.z =
      desired_velocity_.linear.z * 0.5;

    safe_velocity_.angular =
      desired_velocity_.angular;
  }
  else
  {
    safe_velocity_ = desired_velocity_;
  }

  // self_state_.velocity = safe_velocity_;


  std_msgs::msg::Int32 intent_msg;
  intent_msg.data = decision;
  intent_pub_->publish(intent_msg);


  if (decision == 2)
  {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      1000,
      "Cart %d decision: STOP",
      self_state_.cart_id
    );
  }
  else
  {
    RCLCPP_INFO_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      1000,
      "Cart %d decision: PROCEED",
      self_state_.cart_id
    );
  }
}


// =====================================================
// VELOCITY CALLBACK
// =====================================================

void IntercartCommNode::self_velocity_callback(
  const geometry_msgs::msg::Twist::SharedPtr msg)
{
  desired_velocity_ = *msg;
}


// =====================================================
// ADD THE NEW GAZEBO CALLBACK HERE
//
// Put this AFTER self_velocity_callback()
// and BEFORE update_simulation().
// =====================================================

void IntercartCommNode::gazebo_model_states_callback(
  const gazebo_msgs::msg::ModelStates::SharedPtr msg)
{
  for (std::size_t i = 0; i < msg->name.size(); ++i)
  {
    if (msg->name[i] == gazebo_model_name_)
    {
      self_state_.pose = msg->pose[i];
      self_state_.velocity = msg->twist[i];

      return;
    }
  }
}


// =====================================================
// CURRENT INTERNAL SIMULATION
//
// KEEP THIS FOR NOW.
// We will remove the fake position update later.
// =====================================================

void IntercartCommNode::update_simulation()
{
  // Send the currently allowed velocity to Gazebo
  gazebo_cmd_vel_pub_->publish(
    safe_velocity_
  );

  // Broadcast the latest state received from Gazebo
  publish_self_state();
}


// =====================================================
// PUBLISH SELF STATE
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
// MAIN
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