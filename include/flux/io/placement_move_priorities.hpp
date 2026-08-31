#pragma once

#include "flux/core/board.hpp"
#include "flux/core/placement_move_priority.hpp"
#include "flux/io/orientation_constraints.hpp"

#include <filesystem>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace flux::io {

struct PlacementMovePriorityResult {
  std::optional<PlacementMovePriorityPolicy> policy;
  std::vector<ConstraintDiagnostic> diagnostics;

  [[nodiscard]] bool has_errors() const;
  [[nodiscard]] bool ok() const { return policy.has_value() && !has_errors(); }
};

// Loads a complete optimizer-only move policy bound to the immutable source revision and every
// footprint's stable ID/reference. It does not mutate the canonical board.
[[nodiscard]] PlacementMovePriorityResult load_placement_move_priorities(const Board& source,
                                                                         std::string_view contents);

[[nodiscard]] PlacementMovePriorityResult
load_placement_move_priorities_file(const Board& source, const std::filesystem::path& path);

// Builds the same complete source-bound prefix suggestion used by the template. This may guide a
// preview-only optimizer run, but U-prefixed MCU/SoC classification still requires human review.
[[nodiscard]] PlacementMovePriorityPolicy
make_suggested_placement_move_priorities(const Board& source);

// Produces a complete deterministic suggestion. Prefixes R/C/Q/J/P are useful defaults, U is
// deliberately only misc_ic, and all remaining prefixes are placed in the explicit other tier.
[[nodiscard]] std::string make_placement_move_priorities_template(const Board& source);

} // namespace flux::io
