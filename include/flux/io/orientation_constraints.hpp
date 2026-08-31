#pragma once

#include "flux/core/board.hpp"

#include <filesystem>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace flux::io {

enum class ConstraintSeverity { warning, error };

struct ConstraintDiagnostic {
  ConstraintSeverity severity{ConstraintSeverity::error};
  std::string code;
  std::string message;
  std::size_t line{1};
  std::size_t column{1};
  std::vector<std::string> object_ids;
};

struct OrientationConstraintResult {
  std::optional<Board> board;
  std::vector<ObjectId> constrained_footprints;
  std::vector<ConstraintDiagnostic> diagnostics;

  [[nodiscard]] bool has_errors() const;
  [[nodiscard]] bool ok() const { return board.has_value() && !has_errors(); }
};

// Applies a source-bound sidecar to an immutable board snapshot. The sidecar is rejected if the
// board fingerprint, footprint IDs/references, current orientation, or grammar do not match.
[[nodiscard]] OrientationConstraintResult apply_orientation_constraints(const Board& source,
                                                                        std::string_view contents);

[[nodiscard]] OrientationConstraintResult
apply_orientation_constraints_file(const Board& source, const std::filesystem::path& path);

// Produces a deterministic current-only template. Callers explicitly add legal alternatives.
[[nodiscard]] std::string make_orientation_constraints_template(const Board& source);

} // namespace flux::io
