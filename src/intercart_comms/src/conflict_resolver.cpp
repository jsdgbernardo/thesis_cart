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


int ConflictResolver::resolve(
  const CartState &self,
  const std::vector<CartState> &neighbors)
{
  for (const auto &cart : neighbors)
  {
    // -------------------------------------------------
    // 1. Relative position
    // -------------------------------------------------

    double dx =
      cart.pose.position.x -
      self.pose.position.x;

    double dy =
      cart.pose.position.y -
      self.pose.position.y;


    // -------------------------------------------------
    // 2. Relative velocity
    // -------------------------------------------------

    double dvx =
      cart.velocity.linear.x -
      self.velocity.linear.x;

    double dvy =
      cart.velocity.linear.y -
      self.velocity.linear.y;


    // -------------------------------------------------
    // 3. Current distance
    // -------------------------------------------------

    double current_distance =
      std::sqrt(
        dx * dx +
        dy * dy
      );


    // -------------------------------------------------
    // 4. Check whether carts are already too close
    // -------------------------------------------------

    bool current_conflict =
      current_distance < safety_radius_;


    // -------------------------------------------------
    // 5. Calculate time of closest approach
    // -------------------------------------------------

    double relative_speed_squared =
      dvx * dvx +
      dvy * dvy;

    double time_to_closest = 0.0;


    // If the carts have some relative motion
    if (relative_speed_squared > 0.000001)
    {
      time_to_closest =
        -(
          dx * dvx +
          dy * dvy
        ) / relative_speed_squared;
    }


    // We only care about the future.
    //
    // Clamp prediction between:
    //
    // 0 seconds
    //
    // and
    //
    // prediction_horizon_ seconds.

    time_to_closest =
      std::clamp(
        time_to_closest,
        0.0,
        prediction_horizon_
      );


    // -------------------------------------------------
    // 6. Predict positions at closest approach
    // -------------------------------------------------

    double predicted_dx =
      dx +
      dvx * time_to_closest;

    double predicted_dy =
      dy +
      dvy * time_to_closest;


    // -------------------------------------------------
    // 7. Predicted minimum separation
    // -------------------------------------------------

    double predicted_distance =
      std::sqrt(
        predicted_dx * predicted_dx +
        predicted_dy * predicted_dy
      );


    // -------------------------------------------------
    // 8. Determine whether a future conflict exists
    // -------------------------------------------------

    bool predicted_conflict =
      predicted_distance < safety_radius_;


    // -------------------------------------------------
    // 9. Apply temporary right-of-way rule
    // -------------------------------------------------

    if (current_conflict || predicted_conflict)
    {
      // TEMPORARY rule:
      //
      // Lower cart ID gets priority.
      //
      // Example:
      //
      // Cart 1 vs Cart 2
      //
      // Cart 1 -> PROCEED
      // Cart 2 -> STOP

      if (cart.cart_id < self_id_)
      {
        return 2;   // STOP
      }
    }
  }


  return 0;   // PROCEED
}