#pragma once

#include "flux/core/board.hpp"
#include "flux/io/orientation_constraints.hpp"

#include <filesystem>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace flux::io {

struct DifferentialPairConstraintResult {
  std::optional<Board> board;
  std::vector<DifferentialPairId> constrained_pairs;
  std::vector<ConstraintDiagnostic> diagnostics;

  [[nodiscard]] bool has_errors() const;
  [[nodiscard]] bool ok() const { return board.has_value() && !has_errors(); }
};

// Applies explicit hard pair constraints to inferred canonical pair identity. Every current pair
// must appear, even when both values are `none`, and the exact source revision/names must match.
[[nodiscard]] DifferentialPairConstraintResult
apply_differential_pair_constraints(const Board& source, std::string_view contents);

[[nodiscard]] DifferentialPairConstraintResult
apply_differential_pair_constraints_file(const Board& source, const std::filesystem::path& path);

// Produces a source-bound, deterministic template without inventing hard constraints.
[[nodiscard]] std::string make_differential_pair_constraints_template(const Board& source);

} // namespace flux::io
