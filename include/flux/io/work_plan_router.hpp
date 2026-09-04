#pragma once

#include "flux/core/routing_preset.hpp"
#include "flux/io/work_plan.hpp"

#include <cstddef>
#include <optional>
#include <string>
#include <vector>

namespace flux::io {

struct CompiledWorkPlanPlacementOptions {
  Nanometres move_step{Nanometres::from_mm(5.0)};
  Nanometres routing_feedback_margin{Nanometres::from_mm(1.0)};
  std::size_t max_passes{4U};
  std::size_t max_candidate_evaluations{200U};
  std::size_t max_candidate_evaluation_workers{6U};
  std::size_t max_full_routing_candidates_per_wave{4U};
  std::size_t candidate_evaluation_offset{0U};
  std::size_t max_routing_guided_targets_per_footprint{16U};
  std::size_t max_swap_partners_per_footprint{8U};
  std::size_t max_group_translation_proposals_per_group{24U};
  std::size_t max_relative_component_translation_proposals{24U};
  std::size_t max_global_routing_guides{256U};
  std::size_t max_orientation_proposals_per_footprint{8U};
  std::size_t full_routing_verification_interval{8U};
  std::size_t max_pareto_checkpoints{8U};
  bool allow_diagonal_moves{true};
  bool enable_global_placement{false};
  bool enable_global_routing_guidance{false};
  bool enable_routing_guided_moves{true};
  bool enable_swap_moves{false};
  bool enable_group_moves{false};
  bool enable_relative_component_moves{false};
  bool enable_incremental_routing{true};
  bool optimize_routed_quality{false};
  bool orientation_repair{false};
  bool side_flip_repair{false};
  bool assignment_global_placement{false};
  bool allow_tentative_placement_envelopes{false};
  bool suggested_placement_move_priorities{false};

  bool operator==(const CompiledWorkPlanPlacementOptions&) const = default;
};

struct CompiledWorkPlanRouteStep {
  std::string id;
  WorkPlanStepKind kind{WorkPlanStepKind::route};
  AutorouteV1Configuration configuration;
  std::vector<NetId> selected_nets;
  std::vector<ObjectId> selected_footprints;
  std::vector<NetId> priority_nets;
  std::vector<std::string> applied_options;
  // Bounded exact-copper rejection feedback after global/regional checkpoint selection.
  std::size_t max_final_routability_feedback_passes{3U};
  bool post_negotiation_sampled_conflict_repair{false};
  std::size_t max_post_negotiation_sampled_conflict_repair_cones{2U};
  bool refillable_zone_fills{false};
  bool retain_incomplete_diagnostic_preview{false};
  std::optional<CompiledWorkPlanPlacementOptions> placement;

  bool operator==(const CompiledWorkPlanRouteStep&) const = default;
};

struct WorkPlanRouteCompileResult {
  WorkPlanExpansionResult expansion;
  std::vector<CompiledWorkPlanRouteStep> route_steps;
  std::vector<WorkPlanDiagnostic> diagnostics;

  [[nodiscard]] bool has_errors() const;
  [[nodiscard]] bool ok() const { return expansion.ok() && !has_errors(); }
};

// Compiles supported route-like and integrated-placement steps into the same autoroute_v1
// configuration used by the direct CLI. Placement adds bounded optimizer policy and selected
// footprint scope. Differential-pair and bus steps retain their declared atomic group semantics.
// Unknown options and type/range mismatches fail closed. No search or board mutation occurs here.
[[nodiscard]] WorkPlanRouteCompileResult compile_flux_work_plan_routes(const Board& source,
                                                                       const FluxWorkPlan& plan);

} // namespace flux::io
