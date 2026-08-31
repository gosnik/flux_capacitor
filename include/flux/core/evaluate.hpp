#pragma once

#include "flux/core/connectivity.hpp"
#include "flux/core/drc.hpp"
#include "flux/core/validation.hpp"

#include <compare>
#include <cstddef>
#include <cstdint>

namespace flux {

struct ObjectiveVector {
  std::size_t hard_placement_violations{0};
  std::size_t routing_rule_violations{0};
  std::size_t open_connections{0};
  std::size_t unrouted_nets{0};
  std::size_t via_count{0};
  std::uint64_t routed_length_nm{0};

  [[nodiscard]] std::strong_ordering operator<=>(const ObjectiveVector& rhs) const;
  [[nodiscard]] bool operator==(const ObjectiveVector& rhs) const = default;
};

struct EvaluationResult {
  ValidationReport validation;
  ConnectivityReport connectivity;
  CopperDrcReport copper_drc;
  ObjectiveVector objective;
};

[[nodiscard]] EvaluationResult evaluate_board(const Board& board);

} // namespace flux
