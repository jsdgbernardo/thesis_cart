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

#include <cmath>

ConflictResolver::ConflictResolver(
  float safety_radius,
  int self_id)
: safety_radius_(safety_radius),
  self_id_(self_id)
{
}

int ConflictResolver::resolve(
  const CartState &self,
  const std::vector<CartState> &neighbors)
{
  for (const auto &cart : neighbors)
  {
    // Calculate the difference in position
    float dx =
      cart.pose.position.x - self.pose.position.x;

    float dy =
      cart.pose.position.y - self.pose.position.y;

    // Calculate distance between the two carts
    float distance =
      std::sqrt(dx * dx + dy * dy);

    // Check whether the carts are inside
    // the defined safety radius.
    if (distance < safety_radius_)
    {
      // Temporary deterministic rule:
      // the cart with the lower ID gets priority.
      //
      // This is ONLY a placeholder.
      // We will replace this with your
      // finalized collision-avoidance algorithm.

      if (cart.cart_id < self_id_)
      {
        return 2;  // STOP
      }
    }
  }

  return 0;  // PROCEED
}