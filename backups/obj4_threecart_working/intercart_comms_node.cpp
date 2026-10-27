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

  this->declare_parameter("aisle_width", 1.8);

  this->declare_parameter("world_half_width", 6.0);

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
      // Passing-lane offset for the 2.8 m aisle.
      const double target_offset = 0.75;

      const double signed_offset =
        avoid_direction_ * lateral_offset;

      // =============================================
      // CHECK WHETHER WE ALREADY PASSED OUR TARGET
      // =============================================

      auto it = neighbors_.find(avoidance_neighbor_id_);

      if (it == neighbors_.end())
      {
        // We cannot confirm the target's position.
        safe_velocity_ = geometry_msgs::msg::Twist();

        RCLCPP_WARN_THROTTLE(
          this->get_logger(),
          *this->get_clock(),
          1000,
          "Cart %d: avoidance target state unavailable",
          self_state_.cart_id
        );

        break;
      }

      const CartState &target = it->second;

      double target_dx =
        target.pose.position.x -
        self_state_.pose.position.x;

      double target_dy =
        target.pose.position.y -
        self_state_.pose.position.y;

      // Positive = target ahead.
      // Negative = target behind.
      double target_longitudinal =
        std::cos(original_heading_) * target_dx +
        std::sin(original_heading_) * target_dy;

      double target_distance =
        std::hypot(target_dx, target_dy);

      // =============================================
      // IF TARGET IS ALREADY SAFELY BEHIND US,
      // STOP MOVING OUTWARD AND BEGIN RETURNING
      // =============================================

      if (
        target_longitudinal < -1.60 &&
        target_distance > 1.80
      )
      {
        safe_velocity_ = geometry_msgs::msg::Twist();

        double heading_difference =
          normalize_angle(
            original_heading_ - current_yaw
          );

        if (std::abs(heading_difference) < 0.15)
        {
          avoidance_state_ =
            AvoidanceState::RETURN_TO_PATH;
        }
        else
        {
          avoidance_state_ =
            AvoidanceState::TURN_BACK;
        }

        RCLCPP_INFO(
          this->get_logger(),
          "Cart %d: target already passed. "
          "Starting return, offset=%.2f",
          self_state_.cart_id,
          lateral_offset
        );

        break;
      }

      // =============================================
      // OTHERWISE, MOVE INTO THE PASSING LANE
      // =============================================

      double offset_error =
        target_offset - signed_offset;

      // Gradually straighten as we reach the lane.
      double outward_angle =
        std::clamp(
          1.5 * offset_error,
          -0.20,
          0.55
        );

      double desired_heading =
        normalize_angle(
          original_heading_ +
          avoid_direction_ * outward_angle
        );

      double yaw_error =
        normalize_angle(
          desired_heading - current_yaw
        );

      safe_velocity_ = geometry_msgs::msg::Twist();

      safe_velocity_.linear.x = 0.34;

      safe_velocity_.angular.z =
        std::clamp(
          2.7 * yaw_error,
          -0.75,
          0.75
        );

      // =============================================
      // PASS WHEN ALIGNED WITH THE PASSING LANE
      // =============================================

      double original_heading_error =
        normalize_angle(
          original_heading_ - current_yaw
        );

      if (
        signed_offset >= 0.69 &&
        std::abs(original_heading_error) < 0.20
      )
      {
        avoidance_state_ = AvoidanceState::PASS;

        RCLCPP_INFO(
          this->get_logger(),
          "Cart %d: passing lane reached, offset=%.2f",
          self_state_.cart_id,
          lateral_offset
        );
      }

      break;
    }


    case AvoidanceState::PASS:
    {
      // Keep the cart approximately parallel to its
      // original path while maintaining the passing offset.
      const double target_lateral =
        avoid_direction_ * 0.75;

      double offset_error =
        target_lateral - lateral_offset;

      double heading_correction =
        std::clamp(
          1.2 * offset_error,
          -0.25,
          0.25
        );

      double target_heading =
        normalize_angle(
          original_heading_ + heading_correction
        );

      double yaw_error =
        normalize_angle(
          target_heading - current_yaw
        );

      safe_velocity_ = geometry_msgs::msg::Twist();

      safe_velocity_.linear.x = 0.36;

      safe_velocity_.angular.z =
        std::clamp(
          2.5 * yaw_error,
          -0.60,
          0.60
        );

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

        
        double target_distance =
          std::hypot(dx, dy);


        // Require the avoided cart to be at least
        // 0.8 m BEHIND us before turning back.
        if (
          target_longitudinal < -1.60 &&
          target_distance > 1.80
        )
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

      if (passed_target_cart)
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

        // =====================================================
        // CHECK TARGET CLEARANCE BEFORE RETURNING TO PATH
        // =====================================================

        auto target_it =
          neighbors_.find(avoidance_neighbor_id_);

        if (target_it == neighbors_.end())
        {
          // Cannot verify clearance if the target is unknown.
          safe_velocity_ = geometry_msgs::msg::Twist();

          RCLCPP_WARN_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            1000,
            "Cart %d: cannot return; target state unavailable",
            self_state_.cart_id
          );

          break;
        }

        const CartState &target = target_it->second;

        double target_dx =
          target.pose.position.x -
          self_state_.pose.position.x;

        double target_dy =
          target.pose.position.y -
          self_state_.pose.position.y;

        double target_longitudinal =
          std::cos(original_heading_) * target_dx +
          std::sin(original_heading_) * target_dy;

        double target_distance =
          std::hypot(target_dx, target_dy);

        // Return is permitted only after the target is
        // sufficiently behind us AND physically separated.
        bool safe_to_return =
          target_longitudinal < -1.60 &&
          target_distance > 1.80;

        if (!safe_to_return)
        {
          // Return was attempted prematurely.
          // Go back to PASS rather than steering into the cart.
          avoidance_state_ = AvoidanceState::PASS;

          safe_velocity_ = geometry_msgs::msg::Twist();

          RCLCPP_WARN_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            1000,
            "Cart %d: return delayed; longitudinal=%.2f "
            "distance=%.2f",
            self_state_.cart_id,
            target_longitudinal,
            target_distance
          );

          break;
        }



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

static bool is_inside_aisle(
  double x,
  double y,
  double aisle_half_width,
  double world_half_width)
{

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
    std::abs(x) <= world_half_width - footprint_radius &&
    std::abs(y) <= world_half_width - footprint_radius;

  return inside_world &&
    (inside_horizontal_aisle ||
     inside_vertical_aisle);
}


// =====================================================
// PREDICT WHETHER A COMMAND WILL LEAVE THE AISLE
// =====================================================

static bool command_stays_inside_aisle(
  const geometry_msgs::msg::Pose &pose,
  const geometry_msgs::msg::Twist &cmd,
  double aisle_half_width,
  double world_half_width)
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

    if (!is_inside_aisle(
          x,
          y,
          aisle_half_width,
          world_half_width))
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
  // =====================================================
  // 1. UPDATE THE AVOIDANCE CONTROLLER
  // =====================================================

  if (avoidance_state_ != AvoidanceState::NORMAL)
  {
    update_avoidance();
  }

  // =====================================================
  // 2. CART-TO-CART PREDICTIVE SAFETY CHECK
  //
  // Actual prototype footprint:
  // Length = 0.38 m
  // Width  = 0.27 m
  //
  // Use an enclosing circular footprint with a
  // 0.15 m margin between the two cart envelopes.
  // =====================================================

  const double cart_radius =
    std::hypot(0.38 / 2.0, 0.27 / 2.0);

  const double minimum_separation =
    2.0 * cart_radius + 0.15;

  // Predict the next 0.4 seconds in 0.04 s steps.
  const double dt = 0.04;
  const int prediction_steps = 10;

  bool unsafe_cart_motion = false;
  int closest_cart_id = -1;
  double minimum_predicted_distance = 999.0;

  // Check the proposed command against EVERY
  // neighboring cart before sending it to Gazebo.

  for (const auto &entry : neighbors_)
  {
    const CartState &other = entry.second;

    // Start from our current measured pose.
    double predicted_x = self_state_.pose.position.x;
    double predicted_y = self_state_.pose.position.y;

    double predicted_yaw =
      get_yaw(self_state_.pose.orientation);

    for (int step = 0; step <= prediction_steps; ++step)
    {
      double t = step * dt;

      // Gazebo model-state velocities are in
      // world coordinates.
      double other_x =
        other.pose.position.x +
        other.velocity.linear.x * t;

      double other_y =
        other.pose.position.y +
        other.velocity.linear.y * t;

      double distance =
        std::hypot(
          predicted_x - other_x,
          predicted_y - other_y
        );

      if (distance < minimum_predicted_distance)
      {
        minimum_predicted_distance = distance;
        closest_cart_id = other.cart_id;
      }

      if (distance < minimum_separation)
      {
        unsafe_cart_motion = true;
      }

      // Integrate our currently proposed body-frame
      // velocity command for the next sample.
      if (step < prediction_steps)
      {
        double world_vx =
          std::cos(predicted_yaw) * safe_velocity_.linear.x -
          std::sin(predicted_yaw) * safe_velocity_.linear.y;

        double world_vy =
          std::sin(predicted_yaw) * safe_velocity_.linear.x +
          std::cos(predicted_yaw) * safe_velocity_.linear.y;

        predicted_x += world_vx * dt;
        predicted_y += world_vy * dt;

        predicted_yaw += safe_velocity_.angular.z * dt;
      }
    }
  }

  // Stop only if the proposed motion is predicted
  // to violate the protected cart-to-cart clearance.
  if (unsafe_cart_motion)
  {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      500,
      "Cart %d: predicted clearance STOP "
      "near Cart %d; predicted distance=%.3f m, "
      "minimum=%.3f m, avoidance_state=%d",
      self_state_.cart_id,
      closest_cart_id,
      minimum_predicted_distance,
      minimum_separation,
      static_cast<int>(avoidance_state_)
    );

    safe_velocity_ = geometry_msgs::msg::Twist();
  }

  // =====================================================
  // 3. EXISTING AISLE-BOUNDARY SAFETY CHECK
  // =====================================================

  if (!command_stays_inside_aisle(
        self_state_.pose,
        safe_velocity_,
        this->get_parameter("aisle_width").as_double() / 2.0,
        this->get_parameter("world_half_width").as_double()))
  {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      1000,
      "Cart %d: movement blocked by aisle boundary",
      self_state_.cart_id
    );

    safe_velocity_ = geometry_msgs::msg::Twist();
  }

  // =====================================================
  // 4. PUBLISH FINAL SAFE VELOCITY
  // =====================================================

  gazebo_cmd_vel_pub_->publish(safe_velocity_);

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