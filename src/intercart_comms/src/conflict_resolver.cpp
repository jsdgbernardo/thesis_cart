// #include "intercart_comms/conflict_resolver.hpp"
// #include <cmath>

// ConflictResolver::ConflictResolver(float safety_radius, int self_id)
// : safety_radius_(safety_radius), self_id_(self_id)
// {
// }

// int ConflictResolver::resolve(const std::vector<CartState> &neighbors)
// {
//   for (const auto &cart : neighbors) {

//     float dx = cart.pose.position.x;
//     float dy = cart.pose.position.y;
//     float distance = std::sqrt(dx*dx + dy*dy);

//     // get the cart's own position
//     // continuously monitor positions of other carts
//     // project their trajectories based on current velocieties and global paths

//     // if on collision course
//     // safety radius could be used for head-on collisions
//     if (distance < safety_radius_) {
//       // Simple deterministic rule:
//       // lower cart ID gets priority
//         // how to decide priority (first come, first serve)
//         // can follow AROW algorithm, but simpler
//       if (cart.cart_id < self_id_) {
//         return 2; // STOP
//       }
//     }
//   }

//   return 0; // PROCEED
// }


#include "intercart_comms/conflict_resolver.hpp"

#include <algorithm>
#include <cmath>

ConflictResolver::ConflictResolver(
  float safety_radius,
  int self_id,
  double prediction_horizon)
: safety_radius_(safety_radius),
  self_id_(self_id),
  prediction_horizon_(prediction_horizon)
{
}


bool ConflictResolver::hasConflict(
  const CartState &self,
  const std::vector<CartState> &neighbors)
{
  for (const auto &cart : neighbors)
  {
    double dx =
      cart.pose.position.x -
      self.pose.position.x;

    double dy =
      cart.pose.position.y -
      self.pose.position.y;

    double dvx =
      cart.velocity.linear.x -
      self.velocity.linear.x;

    double dvy =
      cart.velocity.linear.y -
      self.velocity.linear.y;

    double current_distance =
      std::sqrt(
        dx * dx +
        dy * dy
      );

    if (current_distance < safety_radius_)
    {
      return true;
    }

    double relative_speed_squared =
      dvx * dvx +
      dvy * dvy;

    if (relative_speed_squared < 0.000001)
    {
      continue;
    }

    double time_to_closest =
      -(
        dx * dvx +
        dy * dvy
      ) / relative_speed_squared;

    time_to_closest =
      std::clamp(
        time_to_closest,
        0.0,
        prediction_horizon_
      );

    double predicted_dx =
      dx +
      dvx * time_to_closest;

    double predicted_dy =
      dy +
      dvy * time_to_closest;

    double predicted_distance =
      std::sqrt(
        predicted_dx * predicted_dx +
        predicted_dy * predicted_dy
      );

    if (predicted_distance < safety_radius_)
    {
      return true;
    }
  }

  return false;
}



// int ConflictResolver::resolve(
//   const CartState &self,
//   const std::vector<CartState> &neighbors)
// {
//   for (const auto &cart : neighbors)
//   {
//     // -------------------------------------------------
//     // 1. Relative position
//     // -------------------------------------------------

//     double dx =
//       cart.pose.position.x -
//       self.pose.position.x;

//     double dy =
//       cart.pose.position.y -
//       self.pose.position.y;


//     // -------------------------------------------------
//     // 2. Relative velocity
//     // -------------------------------------------------

//     double dvx =
//       cart.velocity.linear.x -
//       self.velocity.linear.x;

//     double dvy =
//       cart.velocity.linear.y -
//       self.velocity.linear.y;


//     // -------------------------------------------------
//     // 3. Current distance
//     // -------------------------------------------------

//     double current_distance =
//       std::sqrt(
//         dx * dx +
//         dy * dy
//       );


//     // -------------------------------------------------
//     // 4. Check whether carts are already too close
//     // -------------------------------------------------

//     bool current_conflict =
//       current_distance < safety_radius_;


//     // -------------------------------------------------
//     // 5. Calculate time of closest approach
//     // -------------------------------------------------

//     double relative_speed_squared =
//       dvx * dvx +
//       dvy * dvy;

//     double time_to_closest = 0.0;


//     // If the carts have some relative motion
//     if (relative_speed_squared > 0.000001)
//     {
//       time_to_closest =
//         -(
//           dx * dvx +
//           dy * dvy
//         ) / relative_speed_squared;
//     }


//     // We only care about the future.
//     //
//     // Clamp prediction between:
//     //
//     // 0 seconds
//     //
//     // and
//     //
//     // prediction_horizon_ seconds.

//     time_to_closest =
//       std::clamp(
//         time_to_closest,
//         0.0,
//         prediction_horizon_
//       );


//     // -------------------------------------------------
//     // 6. Predict positions at closest approach
//     // -------------------------------------------------

//     double predicted_dx =
//       dx +
//       dvx * time_to_closest;

//     double predicted_dy =
//       dy +
//       dvy * time_to_closest;


//     // -------------------------------------------------
//     // 7. Predicted minimum separation
//     // -------------------------------------------------

//     double predicted_distance =
//       std::sqrt(
//         predicted_dx * predicted_dx +
//         predicted_dy * predicted_dy
//       );


//     // -------------------------------------------------
//     // 8. Determine whether a future conflict exists
//     // -------------------------------------------------

//     bool predicted_conflict =
//       predicted_distance < safety_radius_;


//     // -------------------------------------------------
//     // 9. Apply temporary right-of-way rule
//     // -------------------------------------------------

//     if (current_conflict || predicted_conflict)
//     {

//       bool self_has_priority = false;
//       bool other_has_priority = false;

//       // -------------------------------------------------
//       // Determine whether the carts are approximately
//       // travelling in opposite directions.
//       // -------------------------------------------------

//       double self_vx = self.velocity.linear.x;
//       double self_vy = self.velocity.linear.y;

//       double other_vx = cart.velocity.linear.x;
//       double other_vy = cart.velocity.linear.y;

//       double dot_product =
//         self_vx * other_vx +
//         self_vy * other_vy;


//       // Negative dot product means that the velocity
//       // vectors generally point in opposite directions.
//       bool opposite_direction =
//         dot_product < -0.01;


//       // -------------------------------------------------
//       // Temporary right-of-way rule:
//       //
//       // lower cart ID = higher priority
//       // -------------------------------------------------

//       if (cart.cart_id < self_id_)
//       {
//         // Another cart has higher priority.
//         return 2;   // STOP
//       }
//       else
//       {
//         // This cart has higher priority.

//         if (opposite_direction)
//         {
//           return 3;   // AVOID
//         }

//         return 0;     // PROCEED
//       }
//     }
//   }


//   return 0;   // PROCEED
// }



int ConflictResolver::resolve(
  const CartState &self,
  const std::vector<CartState> &neighbors)
{
  for (const auto &cart : neighbors)
  {
    // =====================================================
    // 1. RELATIVE POSITION
    // =====================================================

    double dx =
      cart.pose.position.x -
      self.pose.position.x;

    double dy =
      cart.pose.position.y -
      self.pose.position.y;


    // =====================================================
    // 2. RELATIVE VELOCITY
    // =====================================================

    double dvx =
      cart.velocity.linear.x -
      self.velocity.linear.x;

    double dvy =
      cart.velocity.linear.y -
      self.velocity.linear.y;


    // =====================================================
    // 3. CURRENT DISTANCE
    // =====================================================

    double current_distance =
      std::sqrt(
        dx * dx +
        dy * dy
      );

    bool current_conflict =
      current_distance < safety_radius_;


    // =====================================================
    // 4. PREDICT CLOSEST APPROACH
    // =====================================================

    double relative_speed_squared =
      dvx * dvx +
      dvy * dvy;

    double time_to_closest = 0.0;

    if (relative_speed_squared > 0.000001)
    {
      time_to_closest =
        -(
          dx * dvx +
          dy * dvy
        ) / relative_speed_squared;
    }

    time_to_closest =
      std::clamp(
        time_to_closest,
        0.0,
        prediction_horizon_
      );


    // =====================================================
    // 5. PREDICTED MINIMUM DISTANCE
    // =====================================================

    double predicted_dx =
      dx +
      dvx * time_to_closest;

    double predicted_dy =
      dy +
      dvy * time_to_closest;

    double predicted_distance =
      std::sqrt(
        predicted_dx * predicted_dx +
        predicted_dy * predicted_dy
      );

    bool predicted_conflict =
      predicted_distance < safety_radius_;


    // =====================================================
    // 6. IF NO CONFLICT WITH THIS CART, CHECK NEXT CART
    // =====================================================

    if (
      !current_conflict &&
      !predicted_conflict
    )
    {
      continue;
    }


    // =====================================================
    // 7. DETERMINE WHETHER THIS IS HEAD-ON
    // =====================================================

    double self_vx =
      self.velocity.linear.x;

    double self_vy =
      self.velocity.linear.y;

    double other_vx =
      cart.velocity.linear.x;

    double other_vy =
      cart.velocity.linear.y;

    double dot_product =
      self_vx * other_vx +
      self_vy * other_vy;

    bool opposite_direction =
      dot_product < -0.01;


    // =====================================================
    // 8. DETERMINE RIGHT-OF-WAY
    //
    // Priority order:
    //
    // 1. Priority class
    // 2. Earlier conflict arrival time
    // 3. Lower cart ID
    // =====================================================

    bool self_has_priority = false;
    bool other_has_priority = false;


    // -----------------------------------------------------
    // 8A. PRIORITY CLASS
    // -----------------------------------------------------

    if (
      self.priority_class >
      cart.priority_class
    )
    {
      self_has_priority = true;
    }
    else if (
      self.priority_class <
      cart.priority_class
    )
    {
      other_has_priority = true;
    }


    // -----------------------------------------------------
    // 8B. ARRIVAL TIME
    // Only checked if priority classes are equal.
    // -----------------------------------------------------

    else if (
      self.arrival_time >= 0.0 &&
      cart.arrival_time >= 0.0
    )
    {
      double arrival_difference =
        self.arrival_time -
        cart.arrival_time;

      // Earlier cart wins.
      //
      // Times within 0.1 seconds are treated as a tie.
      if (arrival_difference < -0.1)
      {
        self_has_priority = true;
      }
      else if (arrival_difference > 0.1)
      {
        other_has_priority = true;
      }
    }


    // -----------------------------------------------------
    // 8C. CART ID TIE-BREAKER
    //
    // This runs if:
    // - both priority classes are equal, AND
    // - arrival times are unavailable or effectively tied.
    // -----------------------------------------------------

    if (
      !self_has_priority &&
      !other_has_priority
    )
    {
      if (self.cart_id < cart.cart_id)
      {
        self_has_priority = true;
      }
      else
      {
        other_has_priority = true;
      }
    }


    // =====================================================
    // 9. CONVERT RIGHT-OF-WAY INTO AN INTENT
    // =====================================================

    if (other_has_priority)
    {
      return 2;  // STOP
    }


    if (self_has_priority)
    {
      // If this is head-on, the winning cart
      // must physically maneuver around the other cart.
      if (opposite_direction)
      {
        return 3;  // AVOID
      }

      // For an intersection/crossing conflict,
      // the winner proceeds normally.
      return 0;  // PROCEED
    }
  }


  // No conflicts detected with any neighboring cart
  return 0;  // PROCEED
}