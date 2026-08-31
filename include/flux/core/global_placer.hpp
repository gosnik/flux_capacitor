#pragma once

#include "flux/core/board.hpp"
#include "flux/core/validation.hpp"

#include <cstddef>
#include <cstdint>
#include <optional>
#include <stop_token>
#include <string>
#include <vector>

namespace flux {

struct GlobalPlacementGuide {
  ObjectId footprint;
  Point target;
  std::size_t weight{1U};

  bool operator==(const GlobalPlacementGuide&) const = default;
};

struct GlobalPlacerConfig {
  std::size_t iterations{40};
  long double attraction_strength{0.35L};
  long double density_strength{0.60L};
  long double source_anchor_weight{1.0L};
  long double routing_guidance_weight{1.0L};
  Nanometres placement_grid{Nanometres::from_mm(1.0)};
  std::size_t max_pairwise_checks{2'000'000};
  std::size_t max_legalization_candidates_per_footprint{20'000};
  bool enable_assignment_legalization{false};
  std::size_t max_assignment_components{8};
  std::size_t max_assignment_candidates_per_component{64};
  std::size_t max_assignment_search_states{100'000};
  // Permit the legalizer to start from overlap/outside/keepout defects only when every
  // non-authoritative envelope is explicitly tentative and subject to later human review.
  bool allow_tentative_placement_envelopes{false};
};

struct PlacementProxyMetrics {
  std::uint64_t hpwl_nm{0};
  std::uint64_t displacement_nm{0};
  std::uint64_t analytical_target_distance_nm{0};
};

enum class GlobalPlacerStopReason { complete, invalid_input, infeasible, cancelled };

struct GlobalPlacementResult {
  std::optional<Board> candidate;
  ValidationReport validation;
  PlacementProxyMetrics before;
  PlacementProxyMetrics after;
  std::size_t iterations{0};
  std::size_t moved_footprints{0};
  std::size_t constraint_components{0};
  std::size_t immobilized_constraint_components{0};
  std::size_t atomically_legalized_components{0};
  std::size_t moved_components{0};
  std::size_t routing_guides{0};
  std::size_t routing_guided_footprints{0};
  std::size_t legalization_attempts{0};
  bool assignment_legalization_attempted{false};
  bool assignment_legalization_withheld{false};
  bool assignment_legalization_accepted{false};
  bool assignment_search_budget_exhausted{false};
  bool repairable_tentative_source_accepted{false};
  std::size_t repairable_tentative_source_errors{0};
  std::size_t assignment_components{0};
  std::size_t assignment_candidate_edges{0};
  std::size_t assignment_search_states{0};
  GlobalPlacerStopReason stop_reason{GlobalPlacerStopReason::invalid_input};
  std::string diagnostic;

  [[nodiscard]] bool complete() const { return stop_reason == GlobalPlacerStopReason::complete; }
};

[[nodiscard]] std::uint64_t placement_hpwl_nm(const Board& board);

// Produces a legal deterministic placement proposal. Proxy improvement is not acceptance;
// the integrated optimizer must route and DRC the proposal before committing it.
[[nodiscard]] GlobalPlacementResult
place_global(const Board& source, const GlobalPlacerConfig& config = {},
             std::stop_token stop_token = {},
             const std::vector<GlobalPlacementGuide>& guidance = {});

} // namespace flux
