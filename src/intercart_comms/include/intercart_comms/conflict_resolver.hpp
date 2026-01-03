#pragma once

#include <vector>
#include "intercart_comms/cart_state.hpp"

class ConflictResolver
{
public:
  ConflictResolver(float safety_radius, int self_id);

  int resolve(const std::vector<CartState> &neighbors);

private:
  float safety_radius_;
  int self_id_;
};
