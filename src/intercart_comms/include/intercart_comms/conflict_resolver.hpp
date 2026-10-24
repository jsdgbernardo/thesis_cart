#pragma once

#include <vector>

#include "intercart_comms/cart_state.hpp"

class ConflictResolver
{
public:

  ConflictResolver(
    float safety_radius,
    int self_id,
    double prediction_horizon);

  int resolve(
    const CartState &self,
    const std::vector<CartState> &neighbors);

  bool hasConflict(
    const CartState &self,
    const std::vector<CartState> &neighbors);

private:

  float safety_radius_;
  int self_id_;
  double prediction_horizon_;
};