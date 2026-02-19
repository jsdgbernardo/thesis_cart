#include "intercart_comms/conflict_resolver.hpp"
#include <cmath>

ConflictResolver::ConflictResolver(float safety_radius, int self_id)
: safety_radius_(safety_radius), self_id_(self_id)
{
}

int ConflictResolver::resolve(const std::vector<CartState> &neighbors)
{
  for (const auto &cart : neighbors) {

    float dx = cart.pose.position.x;
    float dy = cart.pose.position.y;
    float distance = std::sqrt(dx*dx + dy*dy);

    // get the cart's own position
    // continuously monitor positions of other carts
    // project their trajectories based on current velocieties and global paths

    // if on collision course
    // safety radius could be used for head-on collisions
    if (distance < safety_radius_) {
      // Simple deterministic rule:
      // lower cart ID gets priority
        // how to decide priority (first come, first serve)
        // can follow AROW algorithm, but simpler
      if (cart.cart_id < self_id_) {
        return 2; // STOP
      }
    }
  }

  return 0; // PROCEED
}
