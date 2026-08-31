#pragma once

#include "flux/core/board.hpp"

namespace flux {

struct ResolvedNetRoutingRules {
  Nanometres clearance;
  Nanometres track_width;
  Nanometres via_diameter;
  Nanometres via_drill;
  Nanometres differential_pair_track_width;
  Nanometres differential_pair_gap;
  Nanometres differential_pair_via_gap;

  bool operator==(const ResolvedNetRoutingRules&) const = default;
};

// Resolves independently inherited class properties. Invalid or unknown IDs fall back to board
// defaults; validate_board is the hard gate that rejects such input before solver use.
[[nodiscard]] ResolvedNetRoutingRules resolve_net_routing_rules(const Board& board,
                                                                const NetId& net);

// KiCad-style pairwise clearance: the larger effective clearance controls unlike-net spacing.
[[nodiscard]] Nanometres resolve_pairwise_clearance(const Board& board, const NetId& lhs,
                                                    const NetId& rhs);

} // namespace flux
