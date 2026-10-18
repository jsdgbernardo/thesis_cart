#include "intercart_comms/intercart_comms_node.hpp"

#include <functional>
#include <vector>
#include <cmath>


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


  avoidance_state_ = AvoidanceState::NORMAL;

  original_heading_ = 0.0;
  original_y_ = 0.0;

  avoid_direction_ = -1;

  avoidance_neighbor_id_ = -1;

  //For now: avoid_direction_ = -1 means the cart will bias to the right side of its original path. We can later make this choose whichever side is safer.


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

  if (
    decision == 3 &&
    avoidance_state_ == AvoidanceState::NORMAL
  )
  {
    original_heading_ =
      get_yaw(self_state_.pose.orientation);

    original_y_ =
      self_state_.pose.position.y;

    avoidance_state_ =
      AvoidanceState::TURN_OUT;

    RCLCPP_WARN(
      this->get_logger(),
      "Cart %d starting avoidance maneuver",
      self_state_.cart_id
    );
  }


  if (avoidance_state_ == AvoidanceState::NORMAL)
  {
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
  }

  // self_state_.velocity = safe_velocity_;


  std_msgs::msg::Int32 intent_msg;
  intent_msg.data = decision;
  intent_pub_->publish(intent_msg);


  if (decision == 3)
  {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      1000,
      "Cart %d decision: AVOID",
      self_state_.cart_id
    );
  }
  else if (decision == 2)
  {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      1000,
      "Cart %d decision: STOP",
      self_state_.cart_id
    );
  }
  else if (decision == 1)
  {
    RCLCPP_INFO_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      1000,
      "Cart %d decision: SLOW",
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




// AVOIDANCE STATE MACHINE
void IntercartCommNode::update_avoidance()
{
  double current_yaw =
    get_yaw(self_state_.pose.orientation);

  double heading_error =
    normalize_angle(
      current_yaw -
      original_heading_
    );

  double lateral_offset =
    self_state_.pose.position.y -
    original_y_;

  RCLCPP_INFO_THROTTLE(
    this->get_logger(),
    *this->get_clock(),
    500,
    "Avoid state=%d | y=%.2f | original_y=%.2f | offset=%.2f | yaw=%.2f | original_yaw=%.2f",
    static_cast<int>(avoidance_state_),
    self_state_.pose.position.y,
    original_y_,
    lateral_offset,
    current_yaw,
    original_heading_
  );


  switch (avoidance_state_)
  {
    case AvoidanceState::NORMAL:
    {
      return;
    }


    case AvoidanceState::TURN_OUT:
    {
      safe_velocity_.linear.x = 0.20;
      safe_velocity_.linear.y = 0.0;

      safe_velocity_.angular.z =
        avoid_direction_ * 0.6;

      // Turn about 40 degrees
      if (std::abs(heading_error) >= 0.70)
      {
        avoidance_state_ =
          AvoidanceState::PASS;

        RCLCPP_INFO(
          this->get_logger(),
          "Cart %d avoidance: PASS",
          self_state_.cart_id
        );
      }

      break;
    }


    case AvoidanceState::PASS:
    {
      safe_velocity_.linear.x = 0.30;
      safe_velocity_.linear.y = 0.0;
      safe_velocity_.angular.z = 0.0;

      // Move sideways enough to clear the other cart
      if (std::abs(lateral_offset) >= 1.0)
      {
        avoidance_state_ =
          AvoidanceState::TURN_BACK;

        RCLCPP_INFO(
          this->get_logger(),
          "Cart %d avoidance: TURN_BACK",
          self_state_.cart_id
        );
      }

      break;
    }


    case AvoidanceState::TURN_BACK:
    {
      safe_velocity_.linear.x = 0.20;
      safe_velocity_.linear.y = 0.0;

      // Desired heading is the cart's original heading
      double yaw_error =
        normalize_angle(
          original_heading_ - current_yaw
        );

      // Proportional steering toward original heading
      safe_velocity_.angular.z =
        1.5 * yaw_error;

      // Clamp angular velocity
      if (safe_velocity_.angular.z > 0.6)
      {
        safe_velocity_.angular.z = 0.6;
      }

      if (safe_velocity_.angular.z < -0.6)
      {
        safe_velocity_.angular.z = -0.6;
      }

      // Once roughly parallel to original path,
      // start returning toward the path itself
      if (std::abs(yaw_error) < 0.15)
      {
        safe_velocity_.angular.z = 0.0;

        avoidance_state_ =
          AvoidanceState::RETURN_TO_PATH;

        RCLCPP_INFO(
          this->get_logger(),
          "Cart %d avoidance: RETURN_TO_PATH",
          self_state_.cart_id
        );
      }

      break;
    }


    case AvoidanceState::RETURN_TO_PATH:
    {
      // Lateral error relative to the original straight path
      double path_error =
        original_y_ -
        self_state_.pose.position.y;

      // =====================================================
      // A. Keep moving toward the original path
      //    until we are VERY close to it.
      // =====================================================

      if (std::abs(path_error) >= 0.03)
      {
        double correction_angle =
          1.20 * path_error;

        // Allow a stronger temporary heading toward the path
        if (correction_angle > 0.70)
        {
          correction_angle = 0.70;
        }

        if (correction_angle < -0.70)
        {
          correction_angle = -0.70;
        }

        // Avoid weak corrections when still off-path
        if (std::abs(correction_angle) < 0.15)
        {
          correction_angle =
            (path_error > 0.0) ? 0.15 : -0.15;
        }

        double target_heading =
          normalize_angle(
            original_heading_ +
            correction_angle
          );

        double yaw_error =
          normalize_angle(
            target_heading -
            current_yaw
          );

        // Faster translation back toward path
        safe_velocity_.linear.x = 0.45;
        safe_velocity_.linear.y = 0.0;

        // Stronger steering response
        safe_velocity_.angular.z =
          3.0 * yaw_error;

        if (safe_velocity_.angular.z > 0.9)
        {
          safe_velocity_.angular.z = 0.9;
        }

        if (safe_velocity_.angular.z < -0.9)
        {
          safe_velocity_.angular.z = -0.9;
        }
      }

      
      // =====================================================
      // B. Now we are almost exactly on the old path.
      //    Stop and straighten the heading.
      // =====================================================

      else
      {
        double yaw_error =
          normalize_angle(
            original_heading_ -
            current_yaw
          );

        safe_velocity_.linear.x = 0.0;
        safe_velocity_.linear.y = 0.0;
        safe_velocity_.linear.z = 0.0;

        // Fast final alignment
        safe_velocity_.angular.z =
          4.0 * yaw_error;

        if (safe_velocity_.angular.z > 0.9)
        {
          safe_velocity_.angular.z = 0.9;
        }

        if (safe_velocity_.angular.z < -0.9)
        {
          safe_velocity_.angular.z = -0.9;
        }

        if (
          std::abs(yaw_error) > 0.02 &&
          std::abs(safe_velocity_.angular.z) < 0.18
        )
        {
          safe_velocity_.angular.z =
            (yaw_error > 0.0) ? 0.18 : -0.18;
        }

        // Only finish when BOTH:
        // 1. very close to original path
        // 2. very close to original heading
        if (
          std::abs(path_error) < 0.03 &&
          std::abs(yaw_error) < 0.02
        )
        {
          safe_velocity_.angular.z = 0.0;

          avoidance_state_ =
            AvoidanceState::NORMAL;

          self_state_.intent = 0;

          safe_velocity_ =
            desired_velocity_;

          safe_velocity_.angular.z = 0.0;

          RCLCPP_INFO(
            this->get_logger(),
            "Cart %d avoidance complete -- path error %.3f m, yaw error %.3f rad",
            self_state_.cart_id,
            path_error,
            yaw_error
          );
        }
      }

      break;
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
  if (
    avoidance_state_ !=
    AvoidanceState::NORMAL
  )
  {
    update_avoidance();
  }

  gazebo_cmd_vel_pub_->publish(
    safe_velocity_
  );

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




double IntercartCommNode::get_yaw(
  const geometry_msgs::msg::Quaternion &q)
{
  double siny_cosp =
    2.0 * (
      q.w * q.z +
      q.x * q.y
    );

  double cosy_cosp =
    1.0 -
    2.0 * (
      q.y * q.y +
      q.z * q.z
    );

  return std::atan2(
    siny_cosp,
    cosy_cosp
  );
}

double IntercartCommNode::normalize_angle(double angle)
{
  while (angle > M_PI)
  {
    angle -= 2.0 * M_PI;
  }

  while (angle < -M_PI)
  {
    angle += 2.0 * M_PI;
  }

  return angle;
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