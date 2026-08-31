#pragma once

#include "flux/core/board.hpp"

#include <cstddef>
#include <string>
#include <vector>

namespace flux {

enum class IssueSeverity { warning, error };

struct ValidationIssue {
  IssueSeverity severity{IssueSeverity::error};
  std::string code;
  std::string message;
  std::vector<std::string> object_ids;
};

struct ValidationReport {
  std::vector<ValidationIssue> issues;

  [[nodiscard]] std::size_t error_count() const;
  [[nodiscard]] std::size_t warning_count() const;
  [[nodiscard]] bool ok() const { return error_count() == 0; }
};

struct CandidateValidationResult;

class BoardValidationBaseline {
public:
  [[nodiscard]] const ValidationReport& source_validation() const { return source_validation_; }

private:
  friend BoardValidationBaseline make_board_validation_baseline(const Board& source);
  friend struct CandidateValidationResult;
  friend CandidateValidationResult
  validate_board_candidate(const Board& candidate, const BoardValidationBaseline& baseline);

  ValidationReport source_validation_;
  Polygon outline_;
  std::vector<Polygon> cutouts_;
  std::vector<CopperZone> zones_;
  bool static_zone_geometry_certified_{false};
};

struct CandidateValidationResult {
  ValidationReport validation;
  bool reused_static_zone_geometry{false};
};

[[nodiscard]] ValidationReport validate_board(const Board& board);

// Performs one complete canonical validation and retains exact immutable geometry evidence for
// repeated candidate checks. The baseline is derived state and never changes Board semantics.
[[nodiscard]] BoardValidationBaseline make_board_validation_baseline(const Board& source);

// Reuses static zone-topology evidence only when outline, cutouts, and complete zone objects remain
// exactly equal to the fully checked source baseline. All other canonical rules are revalidated.
[[nodiscard]] CandidateValidationResult
validate_board_candidate(const Board& candidate, const BoardValidationBaseline& baseline);

} // namespace flux
