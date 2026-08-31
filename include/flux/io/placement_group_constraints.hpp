#pragma once

#include "flux/core/board.hpp"
#include "flux/io/orientation_constraints.hpp"

#include <filesystem>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace flux::io {

struct PlacementGroupConstraintResult {
  std::optional<Board> board;
  std::vector<PlacementGroupId> loaded_groups;
  std::vector<ConstraintDiagnostic> diagnostics;

  [[nodiscard]] bool has_errors() const;
  [[nodiscard]] bool ok() const { return board.has_value() && !has_errors(); }
};

// Replaces optimizer-only placement groups on an immutable board snapshot. Every member is bound
// by stable object ID and current human-readable reference; no group is inferred.
[[nodiscard]] PlacementGroupConstraintResult
apply_placement_group_constraints(const Board& source, std::string_view contents);

[[nodiscard]] PlacementGroupConstraintResult
apply_placement_group_constraints_file(const Board& source, const std::filesystem::path& path);

// Produces a deterministic empty document. Designers add explicit group declarations.
[[nodiscard]] std::string make_placement_group_constraints_template(const Board& source);

} // namespace flux::io
