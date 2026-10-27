#include "intercart_comms/intercart_comms_node.hpp"

#include <functional>
#include <vector>
#include <cmath>
#include <algorithm>


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



  priority_class_ =
    this->declare_parameter(
      "priority_class",
      0
    );

  conflict_arrival_time_ = -1.0;

  conflict_active_ = false;




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


  self_state_.priority_class =
  priority_class_;

  self_state_.arrival_time =
    -1.0;


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
  original_x_ = 0.0;
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
  // --------------------------------------------------
  // Ignore our own broadcast
  // --------------------------------------------------

  if (msg->cart_id == self_state_.cart_id)
  {
    return;
  }


  // --------------------------------------------------
  // Convert received ROS message to internal CartState
  // --------------------------------------------------

  CartState cart;

  cart.cart_id = msg->cart_id;
  cart.pose = msg->pose;
  cart.velocity = msg->velocity;
  cart.intent = msg->intent;

  cart.priority_class =
    msg->priority_class;

  cart.arrival_time =
    msg->arrival_time;


  // --------------------------------------------------
  // Store/update this neighbor
  // --------------------------------------------------

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


  // --------------------------------------------------
  // Convert neighbor map to vector
  // --------------------------------------------------

  std::vector<CartState> neighbor_vector;

  for (const auto &entry : neighbors_)
  {
    neighbor_vector.push_back(entry.second);
  }


  // ==================================================
  // NEW CODE STARTS HERE
  // ==================================================

  bool conflict_now =
    resolver_.hasConflict(
      self_state_,
      neighbor_vector
    );


  if (
    conflict_now &&
    !conflict_active_
  )
  {
    conflict_active_ = true;

    conflict_arrival_time_ =
      this->get_clock()->now().seconds();

    self_state_.arrival_time =
      conflict_arrival_time_;

    RCLCPP_INFO(
      this->get_logger(),
      "Cart %d entered negotiation at %.3f",
      self_state_.cart_id,
      conflict_arrival_time_
    );
  }


  if (
    !conflict_now &&
    conflict_active_ &&
    avoidance_state_ == AvoidanceState::NORMAL
  )
  {
    conflict_active_ = false;

    conflict_arrival_time_ = -1.0;

    self_state_.arrival_time = -1.0;

    RCLCPP_INFO(
      this->get_logger(),
      "Cart %d conflict cleared",
      self_state_.cart_id
    );
  }


  // ==================================================
  // NEW CODE ENDS HERE
  // ==================================================


  // --------------------------------------------------
  // Right-of-way / collision decision
  // --------------------------------------------------

  int decision =
    resolver_.resolve(
      self_state_,
      neighbor_vector
    );


  // --------------------------------------------------
  // Store intent
  // --------------------------------------------------

  // While an avoidance maneuver is active,
  // keep broadcasting AVOID regardless of the
  // latest instantaneous resolver result.
  if (avoidance_state_ != AvoidanceState::NORMAL)
  {
    self_state_.intent = 3;   // AVOID
  }
  else
  {
    self_state_.intent = decision;
  }


  // --------------------------------------------------
  // Start avoidance maneuver if needed
  // --------------------------------------------------

  if (
    decision == 3 &&
    avoidance_state_ == AvoidanceState::NORMAL
  )
  {
    original_heading_ =
      get_yaw(self_state_.pose.orientation);

    original_x_ =
      self_state_.pose.position.x;

    original_y_ =
      self_state_.pose.position.y;

    // Dynamically select LEFT or RIGHT
    // based on surrounding carts.
    avoid_direction_ =
      choose_avoid_direction(
        neighbor_vector
      );


    // =====================================================
    // FIND THE NEAREST CART IN FRONT OF US.
    //
    // This becomes the primary cart that we must completely
    // pass before returning to our original path.
    // =====================================================

    avoidance_neighbor_id_ = -1;

    double nearest_forward_distance = 999.0;

    for (const auto &neighbor : neighbor_vector)
    {
      double dx =
        neighbor.pose.position.x -
        self_state_.pose.position.x;

      double dy =
        neighbor.pose.position.y -
        self_state_.pose.position.y;

      // Position of neighbor along our original travel direction
      double longitudinal =
        std::cos(original_heading_) * dx +
        std::sin(original_heading_) * dy;

      // Only consider carts in front of us
      if (
        longitudinal > 0.0 &&
        longitudinal < nearest_forward_distance
      )
      {
        nearest_forward_distance =
          longitudinal;

        avoidance_neighbor_id_ =
          neighbor.cart_id;
      }
    }

    RCLCPP_WARN(
      this->get_logger(),
      "Cart %d avoiding Cart %d",
      self_state_.cart_id,
      avoidance_neighbor_id_
    );

    

    avoidance_state_ =
      AvoidanceState::TURN_OUT;

    RCLCPP_WARN(
      this->get_logger(),
      "Cart %d starting avoidance maneuver",
      self_state_.cart_id
    );
  }


  // --------------------------------------------------
  // Normal intent-to-velocity logic
  // --------------------------------------------------

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
      safe_velocity_ =
        desired_velocity_;
    }
  }


  // --------------------------------------------------
  // Publish intent
  // --------------------------------------------------

  std_msgs::msg::Int32 intent_msg;
  // intent_msg.data = decision;
  intent_msg.data = self_state_.intent;

  intent_pub_->publish(intent_msg);


  // --------------------------------------------------
  // Readable logging
  // --------------------------------------------------

  if (self_state_.intent == 3)
  {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      1000,
      "Cart %d decision: AVOID",
      self_state_.cart_id
    );
  }
  else if (self_state_.intent == 2)
  {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      1000,
      "Cart %d decision: STOP",
      self_state_.cart_id
    );
  }
  else if (self_state_.intent == 1)
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

// void IntercartCommNode::neighbor_callback(
//   const intercart_comms::msg::CartState::SharedPtr msg)
// {
//   if (msg->cart_id == self_state_.cart_id)
//   {
//     return;
//   }

//   CartState cart;

//   cart.cart_id = msg->cart_id;
//   cart.pose = msg->pose;
//   cart.velocity = msg->velocity;
//   cart.intent = msg->intent;

//   cart.priority_class =
//     msg->priority_class;

//   cart.arrival_time =
//     msg->arrival_time;

//   neighbors_[cart.cart_id] = cart;

//   RCLCPP_INFO_THROTTLE(
//     this->get_logger(),
//     *this->get_clock(),
//     1000,
//     "Received state from Cart %d: x=%.2f y=%.2f",
//     cart.cart_id,
//     cart.pose.position.x,
//     cart.pose.position.y
//   );

//   std::vector<CartState> neighbor_vector;

//   for (const auto &entry : neighbors_)
//   {
//     neighbor_vector.push_back(entry.second);
//   }

//   int decision =
//     resolver_.resolve(
//       self_state_,
//       neighbor_vector
//     );

//   if (
//     decision != 0 &&
//     !conflict_active_
//   )
//   {
//     conflict_active_ = true;

//     conflict_arrival_time_ =
//       this->get_clock()->now().seconds();

//     self_state_.arrival_time =
//       conflict_arrival_time_;

//     RCLCPP_INFO(
//       this->get_logger(),
//       "Cart %d entered conflict negotiation at %.3f",
//       self_state_.cart_id,
//       conflict_arrival_time_
//     );
//   }

//   self_state_.intent = decision;

//   if (
//     decision == 3 &&
//     avoidance_state_ == AvoidanceState::NORMAL
//   )
//   {
//     original_heading_ =
//       get_yaw(self_state_.pose.orientation);

//     original_y_ =
//       self_state_.pose.position.y;

//     avoidance_state_ =
//       AvoidanceState::TURN_OUT;

//     RCLCPP_WARN(
//       this->get_logger(),
//       "Cart %d starting avoidance maneuver",
//       self_state_.cart_id
//     );
//   }


//   if (avoidance_state_ == AvoidanceState::NORMAL)
//   {
//     if (decision == 2)
//     {
//       safe_velocity_.linear.x = 0.0;
//       safe_velocity_.linear.y = 0.0;
//       safe_velocity_.linear.z = 0.0;

//       safe_velocity_.angular.x = 0.0;
//       safe_velocity_.angular.y = 0.0;
//       safe_velocity_.angular.z = 0.0;
//     }
//     else if (decision == 1)
//     {
//       safe_velocity_.linear.x =
//         desired_velocity_.linear.x * 0.5;

//       safe_velocity_.linear.y =
//         desired_velocity_.linear.y * 0.5;

//       safe_velocity_.linear.z =
//         desired_velocity_.linear.z * 0.5;

//       safe_velocity_.angular =
//         desired_velocity_.angular;
//     }
//     else
//     {
//       safe_velocity_ = desired_velocity_;
//     }
//   }

//   // self_state_.velocity = safe_velocity_;


//   std_msgs::msg::Int32 intent_msg;
//   intent_msg.data = decision;
//   intent_pub_->publish(intent_msg);


//   if (decision == 3)
//   {
//     RCLCPP_WARN_THROTTLE(
//       this->get_logger(),
//       *this->get_clock(),
//       1000,
//       "Cart %d decision: AVOID",
//       self_state_.cart_id
//     );
//   }
//   else if (decision == 2)
//   {
//     RCLCPP_WARN_THROTTLE(
//       this->get_logger(),
//       *this->get_clock(),
//       1000,
//       "Cart %d decision: STOP",
//       self_state_.cart_id
//     );
//   }
//   else if (decision == 1)
//   {
//     RCLCPP_INFO_THROTTLE(
//       this->get_logger(),
//       *this->get_clock(),
//       1000,
//       "Cart %d decision: SLOW",
//       self_state_.cart_id
//     );
//   }
//   else
//   {
//     RCLCPP_INFO_THROTTLE(
//       this->get_logger(),
//       *this->get_clock(),
//       1000,
//       "Cart %d decision: PROCEED",
//       self_state_.cart_id
//     );
//   }
// }


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



int IntercartCommNode::choose_avoid_direction(
  const std::vector<CartState> &neighbors)
{
  // +1 = LEFT
  // -1 = RIGHT

  double left_nearest = 999.0;
  double right_nearest = 999.0;

  for (const auto &cart : neighbors)
  {
    double dx =
      cart.pose.position.x -
      self_state_.pose.position.x;

    double dy =
      cart.pose.position.y -
      self_state_.pose.position.y;


    // Convert neighbor position into the cart's
    // local lateral direction.
    //
    // positive lateral = LEFT
    // negative lateral = RIGHT
    double lateral =
      -std::sin(original_heading_) * dx
      +
       std::cos(original_heading_) * dy;


    double distance =
      std::sqrt(
        dx * dx +
        dy * dy
      );


    // Ignore carts almost directly ahead/behind
    // when deciding which SIDE has more space.
    if (lateral > 0.15)
    {
      // Neighbor is on LEFT side
      if (distance < left_nearest)
      {
        left_nearest = distance;
      }
    }
    else if (lateral < -0.15)
    {
      // Neighbor is on RIGHT side
      if (distance < right_nearest)
      {
        right_nearest = distance;
      }
    }
  }


  RCLCPP_INFO(
    this->get_logger(),
    "Cart %d side clearance: LEFT=%.2f m RIGHT=%.2f m",
    self_state_.cart_id,
    left_nearest,
    right_nearest
  );


  // Choose whichever side has the farther nearest cart.
  if (left_nearest > right_nearest)
  {
    RCLCPP_WARN(
      this->get_logger(),
      "Cart %d choosing LEFT avoidance",
      self_state_.cart_id
    );

    return 1;
  }

  if (right_nearest > left_nearest)
  {
    RCLCPP_WARN(
      this->get_logger(),
      "Cart %d choosing RIGHT avoidance",
      self_state_.cart_id
    );

    return -1;
  }


  // Both sides appear equally free.
  // Use LEFT as the deterministic default.
  RCLCPP_WARN(
    this->get_logger(),
    "Cart %d sides equal -- defaulting to LEFT",
    self_state_.cart_id
  );

  return 1;
}





// AVOIDANCE STATE MACHINE
void IntercartCommNode::update_avoidance()
{

  // =====================================================
  // EMERGENCY SAFETY CHECK DURING AVOIDANCE
  //
  // If another cart becomes dangerously close while this
  // cart is maneuvering, STOP rather than continuing to
  // push/turn into it.
  // =====================================================

  const double emergency_stop_distance = 0.80;

  for (const auto &entry : neighbors_)
  {
    const CartState &cart = entry.second;

    double dx =
      cart.pose.position.x -
      self_state_.pose.position.x;

    double dy =
      cart.pose.position.y -
      self_state_.pose.position.y;

    double distance =
      std::sqrt(
        dx * dx +
        dy * dy
      );

    if (distance < emergency_stop_distance)
    {
      safe_velocity_.linear.x = 0.0;
      safe_velocity_.linear.y = 0.0;
      safe_velocity_.linear.z = 0.0;

      safe_velocity_.angular.x = 0.0;
      safe_velocity_.angular.y = 0.0;
      safe_velocity_.angular.z = 0.0;

      RCLCPP_ERROR_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        500,
        "Cart %d EMERGENCY STOP during avoidance: Cart %d is only %.2f m away",
        self_state_.cart_id,
        cart.cart_id,
        distance
      );

      return;
    }
  }


  double current_yaw =
    get_yaw(self_state_.pose.orientation);

  double heading_error =
    normalize_angle(
      current_yaw -
      original_heading_
    );


  // =====================================================
  // POSITION RELATIVE TO ORIGINAL PATH START
  // =====================================================

  double dx_from_original =
    self_state_.pose.position.x -
    original_x_;

  double dy_from_original =
    self_state_.pose.position.y -
    original_y_;


  // =====================================================
  // SIGNED LATERAL OFFSET FROM ORIGINAL PATH
  //
  // This works regardless of whether the original cart
  // direction was:
  //   0       -> east
  //   pi      -> west
  //   pi/2    -> north
  //   -pi/2   -> south
  // =====================================================

  double lateral_offset =
    -std::sin(original_heading_) * dx_from_original
    +
    std::cos(original_heading_) * dy_from_original;


  // Error used to steer BACK toward the path
  double path_error =
    -lateral_offset;

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
      // Continue forward beside the original path.
      safe_velocity_.linear.x = 0.40;
      safe_velocity_.linear.y = 0.0;
      safe_velocity_.angular.z = 0.0;


      bool passed_target_cart = false;

      double target_longitudinal = 999.0;


      // =====================================================
      // FIND THE CART WE ARE CURRENTLY PASSING
      // =====================================================

      auto it =
        neighbors_.find(
          avoidance_neighbor_id_
        );

      if (it != neighbors_.end())
      {
        const CartState &target =
          it->second;

        double dx =
          target.pose.position.x -
          self_state_.pose.position.x;

        double dy =
          target.pose.position.y -
          self_state_.pose.position.y;


        // Position of target relative to our ORIGINAL
        // forward direction.
        //
        // positive = target still ahead
        // zero     = approximately alongside
        // negative = target is now behind us
        target_longitudinal =
          std::cos(original_heading_) * dx +
          std::sin(original_heading_) * dy;


        // Require the avoided cart to be at least
        // 0.8 m BEHIND us before turning back.
        if (target_longitudinal < -0.8)
        {
          passed_target_cart = true;
        }
      }


      RCLCPP_INFO_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        500,
        "Cart %d PASS: lateral=%.2f m target_longitudinal=%.2f m passed=%s",
        self_state_.cart_id,
        lateral_offset,
        target_longitudinal,
        passed_target_cart ? "YES" : "NO"
      );


      // =====================================================
      // DO NOT TURN BACK UNTIL:
      //
      // 1. We have enough sideways clearance, AND
      // 2. We have completely passed the target cart.
      // =====================================================

      if (
        std::abs(lateral_offset) >= 0.8 &&
        passed_target_cart
      )
      {
        avoidance_state_ =
          AvoidanceState::TURN_BACK;

        RCLCPP_INFO(
          this->get_logger(),
          "Cart %d avoidance: target cleared, TURN_BACK",
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
      // // Lateral error relative to the original straight path
      // double path_error =
      //   original_y_ -
      //   self_state_.pose.position.y;

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

          avoidance_neighbor_id_ = -1;

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
// CROSS-AISLE SAFETY CHECK
//
// Valid for the test world:
// - 1.8 m wide corridors
// - Corridor centerlines at x=0 and y=0
// - Shelves occupy the four corner quadrants
//
// Uses a conservative circular footprint around the
// cart, including 0.10 m of wall clearance.
// =====================================================

static bool is_inside_aisle(double x, double y)
{
  constexpr double aisle_half_width = 0.90;

  constexpr double cart_half_length = 0.38 / 2.0;
  constexpr double cart_half_width  = 0.27 / 2.0;

  constexpr double wall_margin = 0.10;

  const double footprint_radius =
    std::hypot(
      cart_half_length,
      cart_half_width
    ) + wall_margin;

  const double allowable_offset =
    aisle_half_width - footprint_radius;

  // These two conditions describe the crossing
  // horizontal and vertical open corridors.
  bool inside_horizontal_aisle =
    std::abs(y) <= allowable_offset;

  bool inside_vertical_aisle =
    std::abs(x) <= allowable_offset;

  // Limit motion to the modeled 12 m x 12 m area.
  bool inside_world =
    std::abs(x) <= 6.0 - footprint_radius &&
    std::abs(y) <= 6.0 - footprint_radius;

  return inside_world &&
    (inside_horizontal_aisle ||
     inside_vertical_aisle);
}


// =====================================================
// PREDICT WHETHER A COMMAND WILL LEAVE THE AISLE
// =====================================================

static bool command_stays_inside_aisle(
  const geometry_msgs::msg::Pose &pose,
  const geometry_msgs::msg::Twist &cmd)
{
  double x = pose.position.x;
  double y = pose.position.y;

  const auto &q = pose.orientation;

  double yaw = std::atan2(
    2.0 * (q.w * q.z + q.x * q.y),
    1.0 - 2.0 * (q.y * q.y + q.z * q.z)
  );

  // Check the next 0.6 seconds at 0.05 s intervals.
  constexpr double dt = 0.05;
  constexpr int steps = 12;

  for (int i = 0; i < steps; ++i)
  {
    // Body-frame planar velocity converted to world frame.
    double vx =
      std::cos(yaw) * cmd.linear.x -
      std::sin(yaw) * cmd.linear.y;

    double vy =
      std::sin(yaw) * cmd.linear.x +
      std::cos(yaw) * cmd.linear.y;

    x += vx * dt;
    y += vy * dt;

    yaw += cmd.angular.z * dt;

    if (!is_inside_aisle(x, y))
    {
      return false;
    }
  }

  return true;
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


  // =====================================================
  // FINAL ENVIRONMENT SAFETY GATE
  // =====================================================

  if (!command_stays_inside_aisle(
        self_state_.pose,
        safe_velocity_))
  {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      1000,
      "Cart %d: movement blocked by aisle boundary",
      self_state_.cart_id
    );

    // Do not execute a movement that leaves the aisle.
    safe_velocity_ = geometry_msgs::msg::Twist();
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

  msg.priority_class =
    self_state_.priority_class;

  msg.arrival_time =
    self_state_.arrival_time;

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