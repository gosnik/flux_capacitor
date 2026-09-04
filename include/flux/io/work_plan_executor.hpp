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
  std::vector<NetId> replacement_nets;
  std::size_t global_unrouted_nets{0U};
  std::size_t global_overflow{0U};
  std::uint64_t global_cost{0U};
  bool sampled_fallback_attempted{false};
  bool sampled_fallback_accepted{false};
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
  bool external_kicad_drc_run{false};
  std::optional<Board> candidate;
  WorkPlanAcceptanceState final_acceptance;
  std::vector<WorkPlanStepExecutionResult> steps;
  std::vector<WorkPlanDiagnostic> diagnostics;

  [[nodiscard]] bool ok() const { return internally_complete; }
};

// Executes only currently supported route and validation steps on private board copies. Preview
// never authorizes export. Every route is committed to the next step only after detailed exact
// acceptance; a failed/cancelled step leaves the prior checkpoint intact and skips dependants.
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
