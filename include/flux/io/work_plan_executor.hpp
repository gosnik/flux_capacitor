#pragma once

#include "flux/core/router_telemetry.hpp"
#include "flux/io/work_plan_router.hpp"

#include <cstddef>
#include <optional>
#include <stop_token>
#include <string>
#include <vector>

namespace flux::io {

enum class WorkPlanStepExecutionStatus { completed, failed, cancelled, skipped };

struct WorkPlanAcceptanceState {
  bool structural_validation_ok{false};
  std::size_t structural_issue_count{0U};
  std::size_t open_connections{0U};
  std::size_t unrouted_nets{0U};
  std::size_t copper_violations{0U};
  bool exact_internal_acceptance{false};

  bool operator==(const WorkPlanAcceptanceState&) const = default;
};

struct WorkPlanStepExecutionResult {
  std::string id;
  WorkPlanStepKind kind{WorkPlanStepKind::validation};
  WorkPlanStepExecutionStatus status{WorkPlanStepExecutionStatus::skipped};
  std::vector<NetId> selected_nets;
  std::vector<ObjectId> selected_footprints;
  std::vector<NetId> replacement_nets;
  std::size_t global_unrouted_nets{0U};
  std::size_t global_overflow{0U};
  std::uint64_t global_cost{0U};
  bool post_negotiation_sampled_conflict_repair_enabled{false};
  std::size_t post_negotiation_cone_repair_attempts{0U};
  std::size_t post_negotiation_cone_repairs_accepted{0U};
  std::size_t post_negotiation_cone_repair_nets_attempted{0U};
  bool coarse_polyline_fallback_attempted{false};
  bool coarse_polyline_fallback_accepted{false};
  bool sampled_fallback_attempted{false};
  bool sampled_fallback_accepted{false};
  bool final_routability_feedback_attempted{false};
  bool final_routability_feedback_selected{false};
  std::size_t final_routability_feedback_passes{0U};
  std::size_t final_routability_feedback_improvements{0U};
  std::size_t final_routability_diagnoses_consumed{0U};
  bool provisional_zone_refill_required{false};
  bool incomplete_diagnostic_preview_attempted{false};
  bool incomplete_diagnostic_preview_available{false};
  std::size_t incomplete_diagnostic_preview_paths{0U};
  bool placement_attempted{false};
  std::size_t placement_candidate_evaluations{0U};
  std::size_t placement_full_route_evaluations{0U};
  std::size_t placement_moves_accepted{0U};
  std::size_t placement_priority_tiers_unlocked{0U};
  bool placement_move_priorities_configured{false};
  bool placement_human_review_required{false};
  std::string placement_stop_reason;
  WorkPlanAcceptanceState acceptance;
  std::string diagnostic;

  bool operator==(const WorkPlanStepExecutionResult&) const = default;
};

struct WorkPlanExecutionResult {
  std::string plan_sha256;
  WorkPlanMode mode{WorkPlanMode::preview};
  bool preview{true};
  bool internally_complete{false};
  bool whole_board_accepted{false};
  bool candidate_ready_for_export{false};
  bool candidate_ready_for_zone_refill{false};
  bool zone_refill_required{false};
  bool candidate_is_incomplete_diagnostic_preview{false};
  bool zone_refill_run{false};
  bool zone_refill_passed{false};
  int zone_refill_exit_code{-1};
  std::string zone_refill_executable;
  std::string zone_refill_diagnostic;
  bool external_kicad_drc_run{false};
  bool external_kicad_drc_passed{false};
  int external_kicad_drc_exit_code{-1};
  std::string external_kicad_drc_executable;
  std::string external_kicad_drc_diagnostic;
  std::optional<Board> candidate;
  WorkPlanAcceptanceState final_acceptance;
  std::vector<WorkPlanStepExecutionResult> steps;
  std::vector<WorkPlanDiagnostic> diagnostics;

  [[nodiscard]] bool ok() const { return internally_complete; }
};

// Executes currently supported route, differential-pair, bus, placement, and validation steps on
// private board copies. Preview never authorizes export. Every route-like or placement step is
// committed to the next step only after detailed exact acceptance; the sole provisional exception
// is an explicitly terminal refill-required route that can authorize only the external refill
// transaction. A failed/cancelled step leaves the prior checkpoint intact and skips dependants.
[[nodiscard]] WorkPlanExecutionResult
execute_flux_work_plan(const Board& source, const FluxWorkPlan& plan,
                       std::stop_token stop_token = {}, RouterTelemetrySink* telemetry = nullptr);

// Executes a file-loaded plan against the exact policy-applied board produced by its one-read
// identity/application boundary. Raw execution above deliberately rejects non-empty policy lists.
[[nodiscard]] WorkPlanExecutionResult
execute_loaded_flux_work_plan(const WorkPlanLoadResult& loaded, std::stop_token stop_token = {},
                              RouterTelemetrySink* telemetry = nullptr);

[[nodiscard]] std::string_view
work_plan_step_execution_status_name(WorkPlanStepExecutionStatus status);

// Deterministic diagnostic summary. It deliberately excludes board geometry; the candidate uses
// the ordinary source-preserving KiCad exporter and remains subject to external KiCad DRC.
[[nodiscard]] std::string
serialize_flux_work_plan_execution_summary(const WorkPlanExecutionResult& result);

} // namespace flux::io
