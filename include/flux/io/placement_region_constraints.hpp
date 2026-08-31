#pragma once

#include "flux/core/board.hpp"
#include "flux/io/orientation_constraints.hpp"

#include <filesystem>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace flux::io {

struct PlacementRegionConstraintResult {
  std::optional<Board> board;
  std::vector<PlacementRegionId> loaded_regions;
  std::vector<ConstraintDiagnostic> diagnostics;

  [[nodiscard]] bool has_errors() const;
  [[nodiscard]] bool ok() const { return board.has_value() && !has_errors(); }
};

// Replaces optimizer-only placement regions on an immutable board snapshot. World polygons and
// membership are explicit; no region or member is inferred from current placement or connectivity.
[[nodiscard]] PlacementRegionConstraintResult
apply_placement_region_constraints(const Board& source, std::string_view contents);

[[nodiscard]] PlacementRegionConstraintResult
apply_placement_region_constraints_file(const Board& source, const std::filesystem::path& path);

// Produces a deterministic empty document. Designers add explicit regions and members.
[[nodiscard]] std::string make_placement_region_constraints_template(const Board& source);

} // namespace flux::io
