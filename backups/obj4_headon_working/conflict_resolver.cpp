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
    // =====================================================
    // ACTIVE AVOIDANCE RIGHT-OF-WAY LOCK
    //
    // If another cart is currently performing an avoidance
    // maneuver and it has equal or higher priority than us,
    // keep yielding until it has safely moved away.
    // =====================================================

    double lock_dx =
      cart.pose.position.x -
      self.pose.position.x;

    double lock_dy =
      cart.pose.position.y -
      self.pose.position.y;

    double lock_distance =
      std::sqrt(
        lock_dx * lock_dx +
        lock_dy * lock_dy
      );


    // Neighbor is actively performing avoidance
    if (cart.intent == 3)
    {
      bool neighbor_has_priority = false;


      // Priority-class comparison
      if (cart.priority_class > self.priority_class)
      {
        neighbor_has_priority = true;
      }

      // Same priority class: use arrival time / ID
      else if (cart.priority_class == self.priority_class)
      {
        if (
          cart.arrival_time >= 0.0 &&
          self.arrival_time >= 0.0
        )
        {
          double arrival_difference =
            cart.arrival_time -
            self.arrival_time;

          // Neighbor arrived clearly earlier
          if (arrival_difference < -0.1)
          {
            neighbor_has_priority = true;
          }

          // Effectively tied: lower ID wins
          else if (
            std::abs(arrival_difference) <= 0.1 &&
            cart.cart_id < self.cart_id
          )
          {
            neighbor_has_priority = true;
          }
        }

        // If timestamps are unavailable,
        // fall back to cart ID.
        else if (cart.cart_id < self.cart_id)
        {
          neighbor_has_priority = true;
        }
      }


      // Keep yielding while the avoiding cart
      // is still close enough to the conflict area.
      if (
        neighbor_has_priority &&
        lock_distance < 3.0
      )
      {
        return 2;  // STOP
      }
    }


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



int ConflictResolver::resolve(
  const CartState &self,
  const std::vector<CartState> &neighbors)
{
  // =====================================================
  // FIRST PASS
  // CHECK ALL CARTS FOR ACTIVE HIGHER-PRIORITY AVOIDANCE
  //
  // This is checked BEFORE normal collision prediction.
  // If another higher-priority cart is actively avoiding
  // nearby, this cart must keep yielding.
  // =====================================================

  for (const auto &cart : neighbors)
  {
    // Only care about carts currently broadcasting AVOID.
    if (cart.intent != 3)
    {
      continue;
    }

    double dx =
      cart.pose.position.x -
      self.pose.position.x;

    double dy =
      cart.pose.position.y -
      self.pose.position.y;

    double distance =
      std::sqrt(
        dx * dx +
        dy * dy
      );


    bool neighbor_has_priority = false;


    // -----------------------------------------------------
    // 1. PRIORITY CLASS
    // -----------------------------------------------------

    if (
      cart.priority_class >
      self.priority_class
    )
    {
      neighbor_has_priority = true;
    }


    // -----------------------------------------------------
    // 2. SAME PRIORITY CLASS
    //    Compare arrival time
    // -----------------------------------------------------

    else if (
      cart.priority_class ==
      self.priority_class
    )
    {
      if (
        cart.arrival_time >= 0.0 &&
        self.arrival_time >= 0.0
      )
      {
        double difference =
          cart.arrival_time -
          self.arrival_time;

        // Neighbor arrived earlier
        if (difference < -0.1)
        {
          neighbor_has_priority = true;
        }

        // Nearly simultaneous:
        // lower cart ID wins
        else if (
          std::abs(difference) <= 0.1 &&
          cart.cart_id < self.cart_id
        )
        {
          neighbor_has_priority = true;
        }
      }

      // If arrival times are unavailable,
      // use cart ID as deterministic fallback.
      else if (
        cart.cart_id <
        self.cart_id
      )
      {
        neighbor_has_priority = true;
      }
    }


    // -----------------------------------------------------
    // ACTIVE AVOIDANCE LOCK
    // -----------------------------------------------------

    // =============================================
    // CHECK WHETHER THE TWO CARTS HAVE ALREADY
    // SAFELY PASSED ONE ANOTHER
    // =============================================

    const auto &self_q = self.pose.orientation;
    const auto &other_q = cart.pose.orientation;

    double self_yaw = std::atan2(
      2.0 * (self_q.w * self_q.z + self_q.x * self_q.y),
      1.0 - 2.0 * (self_q.y * self_q.y + self_q.z * self_q.z)
    );

    double other_yaw = std::atan2(
      2.0 * (other_q.w * other_q.z + other_q.x * other_q.y),
      1.0 - 2.0 * (other_q.y * other_q.y + other_q.z * other_q.z)
    );

    // Other cart's location relative to my heading.
    double other_along_self =
      std::cos(self_yaw) * dx +
      std::sin(self_yaw) * dy;

    // My location relative to the other cart's heading.
    double self_along_other =
      std::cos(other_yaw) * (-dx) +
      std::sin(other_yaw) * (-dy);

    // Both carts must be behind one another in their
    // respective current forward directions.
    // This is appropriate for the head-on pass case.
    bool passed_each_other =
      other_along_self < -1.20 &&
      self_along_other < -1.20;

    // Require additional physical separation.
    bool enough_clearance =
      distance > 1.80;

    bool safely_cleared =
      passed_each_other && enough_clearance;

    // Keep yielding only while the priority cart has
    // NOT clearly completed the pass.
    if (
      neighbor_has_priority &&
      distance < 5.0 &&
      !safely_cleared
    )
    {
      return 2;  // STOP
    }
  }



    // =====================================================
    // WAITING-ORDER LOCK
    //
    // If another cart is currently STOPPED and has a
    // higher right-of-way rank than us, remain stopped.
    //
    // This preserves the queue/order after a higher
    // priority cart has cleared the conflict.
    // =====================================================

    for (const auto &cart : neighbors)
    {
      // Only evaluate carts that are currently waiting.
      if (cart.intent != 2)
      {
        continue;
      }


      double dx =
        cart.pose.position.x -
        self.pose.position.x;

      double dy =
        cart.pose.position.y -
        self.pose.position.y;

      double distance =
        std::sqrt(
          dx * dx +
          dy * dy
        );


      // Only coordinate with reasonably nearby waiting carts.
      if (distance > 4.0)
      {
        continue;
      }


      bool other_goes_first = false;


      // =====================================================
      // 1. PRIORITY CLASS
      // =====================================================

      if (
        cart.priority_class >
        self.priority_class
      )
      {
        other_goes_first = true;
      }


      // =====================================================
      // 2. SAME PRIORITY CLASS:
      //    earlier arrival time wins
      // =====================================================

      else if (
        cart.priority_class ==
        self.priority_class
      )
      {
        if (
          cart.arrival_time >= 0.0 &&
          self.arrival_time >= 0.0
        )
        {
          double difference =
            cart.arrival_time -
            self.arrival_time;


          // Other cart arrived earlier
          if (difference < -0.1)
          {
            other_goes_first = true;
          }


          // Arrival times effectively tied:
          // lower cart ID wins
          else if (
            std::abs(difference) <= 0.1 &&
            cart.cart_id < self.cart_id
          )
          {
            other_goes_first = true;
          }
        }


        // If arrival timestamps are not available,
        // use cart ID as deterministic fallback.
        else if (
          cart.cart_id <
          self.cart_id
        )
        {
          other_goes_first = true;
        }
      }


      // Higher-ranked waiting cart gets released first.
      if (other_goes_first)
      {
        return 2;  // remain STOPPED
      }
    }




  // =====================================================
  // SECOND PASS
  // NORMAL COLLISION PREDICTION AND RIGHT-OF-WAY
  // =====================================================

  bool avoidance_required = false;

  for (const auto &cart : neighbors)
  {
    // -----------------------------------------------------
    // Relative position
    // -----------------------------------------------------

    double dx =
      cart.pose.position.x -
      self.pose.position.x;

    double dy =
      cart.pose.position.y -
      self.pose.position.y;


    // -----------------------------------------------------
    // Relative velocity
    // -----------------------------------------------------

    double dvx =
      cart.velocity.linear.x -
      self.velocity.linear.x;

    double dvy =
      cart.velocity.linear.y -
      self.velocity.linear.y;


    // -----------------------------------------------------
    // Current distance
    // -----------------------------------------------------

    double current_distance =
      std::sqrt(
        dx * dx +
        dy * dy
      );

    bool current_conflict =
      current_distance < safety_radius_;


    // -----------------------------------------------------
    // Predict time of closest approach
    // -----------------------------------------------------

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


    // -----------------------------------------------------
    // Predicted minimum distance
    // -----------------------------------------------------

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


    // -----------------------------------------------------
    // No conflict with this cart?
    // Check the next cart.
    // -----------------------------------------------------

    if (
      !current_conflict &&
      !predicted_conflict
    )
    {
      continue;
    }


    // -----------------------------------------------------
    // Determine whether this is head-on
    // -----------------------------------------------------

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
    // RIGHT-OF-WAY
    // =====================================================

    bool self_has_priority = false;
    bool other_has_priority = false;


    // -----------------------------------------------------
    // 1. Priority class
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
    // 2. Arrival time
    // -----------------------------------------------------

    else if (
      self.arrival_time >= 0.0 &&
      cart.arrival_time >= 0.0
    )
    {
      double arrival_difference =
        self.arrival_time -
        cart.arrival_time;

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
    // 3. Cart ID tie-break
    // -----------------------------------------------------

    if (
      !self_has_priority &&
      !other_has_priority
    )
    {
      if (
        self.cart_id <
        cart.cart_id
      )
      {
        self_has_priority = true;
      }
      else
      {
        other_has_priority = true;
      }
    }


    // =====================================================
    // TURN PRIORITY INTO AN INTENT
    // =====================================================

    if (other_has_priority)
    {
      // Any higher-ranked conflicting cart requires
      // us to yield, even if we won against another cart.
      return 2;
    }

    if (self_has_priority && opposite_direction)
    {
      // Record the need to avoid, but continue checking
      // ALL remaining neighboring carts.
      avoidance_required = true;
    }
  }


  // No conflict with any cart
  return avoidance_required ? 3 : 0;
}