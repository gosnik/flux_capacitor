#pragma once

#include "flux/core/board.hpp"
#include "flux/io/orientation_constraints.hpp"

#include <filesystem>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace flux::io {

struct PlacementEnvelopeConstraintResult {
  std::optional<Board> board;
  std::vector<ObjectId> constrained_footprints;
  std::vector<ConstraintDiagnostic> diagnostics;

  [[nodiscard]] bool has_errors() const;
  [[nodiscard]] bool ok() const { return board.has_value() && !has_errors(); }
};

[[nodiscard]] PlacementEnvelopeConstraintResult
apply_placement_envelope_constraints(const Board& source, std::string_view contents);

[[nodiscard]] PlacementEnvelopeConstraintResult
apply_placement_envelope_constraints_file(const Board& source, const std::filesystem::path& path);

// Suggestions are deliberately review-required and cannot be applied until every `approved no`
// entry is checked and changed to `approved yes` by the design owner.
[[nodiscard]] std::string make_placement_envelope_constraints_template(const Board& source);

} // namespace flux::io
