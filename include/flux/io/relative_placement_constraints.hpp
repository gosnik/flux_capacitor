#pragma once

#include "flux/core/board.hpp"
#include "flux/io/orientation_constraints.hpp"

#include <filesystem>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace flux::io {

struct RelativePlacementConstraintResult {
  std::optional<Board> board;
  std::vector<RelativePlacementConstraintId> loaded_constraints;
  std::vector<ConstraintDiagnostic> diagnostics;

  [[nodiscard]] bool has_errors() const;
  [[nodiscard]] bool ok() const { return board.has_value() && !has_errors(); }
};

[[nodiscard]] RelativePlacementConstraintResult
apply_relative_placement_constraints(const Board& source, std::string_view contents);

[[nodiscard]] RelativePlacementConstraintResult
apply_relative_placement_constraints_file(const Board& source, const std::filesystem::path& path);

// Produces a deterministic empty document; proximity or connectivity never implies a relationship.
[[nodiscard]] std::string make_relative_placement_constraints_template(const Board& source);

} // namespace flux::io
