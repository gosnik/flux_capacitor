#pragma once

#include "flux/core/board.hpp"

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace flux {

enum class CopperViolationKind {
  track_clearance,
  track_via_clearance,
  via_clearance,
  track_pad_clearance,
  via_pad_clearance,
  pad_clearance,
  track_zone_clearance,
  via_zone_clearance,
  pad_zone_clearance,
  zone_clearance,
  differential_pair_gap,
  differential_pair_skew,
  track_width,
  via_diameter,
  via_drill,
  via_annular_width,
  hole_clearance,
  hole_to_hole_clearance,
  copper_edge_clearance,
  copper_outside_board,
  keepout
};

struct CopperViolation {
  CopperViolationKind kind{CopperViolationKind::track_clearance};
  std::string code;
  std::vector<std::string> object_ids;
  Nanometres required_clearance;
  long double actual_distance_nm{0.0L};
};

struct CopperDrcReport {
  std::vector<CopperViolation> violations;

  [[nodiscard]] bool ok() const { return violations.empty(); }
  [[nodiscard]] std::size_t violation_count() const { return violations.size(); }
};

// Checks currently represented copper using exact supported-pad geometry and keepout
// constraints. KiCad DRC is still the final external oracle for generated KiCad output.
[[nodiscard]] CopperDrcReport check_copper_rules(const Board& board);

// Exact bounded DRC for dynamic copper proposal objects. Every reported violation involves at
// least one selected track, arc, or via, so callers can cheaply reject a local geometry proposal
// without repeatedly rechecking immutable source-only relationships. Differential-pair rules for
// a touched net remain included. Whole-board validation, connectivity, and check_copper_rules()
// remain mandatory before a transaction is committed.
[[nodiscard]] CopperDrcReport
check_dynamic_copper_rules(const Board& board, const std::vector<ObjectId>& dynamic_objects);

// Saturating sum of represented straight and circular track centerline lengths for one net.
// This is the shared spatial-length primitive used by DRC and post-route tuning.
[[nodiscard]] std::uint64_t routed_net_centerline_length_nm(const Board& board, const NetId& net);

} // namespace flux
