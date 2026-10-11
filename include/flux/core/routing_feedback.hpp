#pragma once

#include "flux/core/detailed_router.hpp"
#include "flux/core/routing_engine.hpp"
#include "flux/core/regional_router.hpp"
#include "flux/core/router_telemetry.hpp"

#include <cstddef>
#include <stop_token>
#include <vector>

namespace flux {

// Runs ordinary global negotiation, then applies bounded attributed spatial-cone repair only when
// sampled/physical resource overflow remains on an otherwise connected checkpoint.
[[nodiscard]] GlobalRouteResult route_global_with_sampled_conflict_repair(
    const Board& board, const GlobalRouterConfig& config, std::size_t max_repair_cones,
    std::stop_token stop_token = {}, RouterTelemetrySink* telemetry = nullptr);

// Builds the corresponding regional plan without invoking the ordinary outer planner loop. This
// preserves the explicit repair mode's semantics while sharing it between CLI and WorkPlan paths.
[[nodiscard]] RoutingPlanResult plan_routes_with_sampled_conflict_repair(
    const Board& board, const RoutingPlannerConfig& config, std::size_t max_repair_cones,
    std::stop_token stop_token = {}, RouterTelemetrySink* telemetry = nullptr);

struct MaterializationFeedbackResult {
  RoutingPlanResult routing;
  DetailedRouteResult detailed;
  bool coarse_polyline_fallback_attempted{false};
  bool coarse_polyline_fallback_accepted{false};
  enum class Scope { local_geometry, regional, global };
  struct Pass {
    std::size_t index{0U};
    Scope scope{Scope::global};
    std::size_t diagnoses{0U};
    std::size_t affected_nets{0U};
    bool repeated_failure_escalation{false};
    bool incremental_global{false};
    bool incremental_detailed{false};
    bool plan_complete{false};
    bool exact_complete{false};
    bool selected{false};

    bool operator==(const Pass&) const = default;
  };
  std::vector<Pass> passes;
  RoutingEngineOutcome engine_outcome{RoutingEngineOutcome::progressed};
  std::vector<RoutingEngineAttempt> engine_attempts;
  std::size_t engine_tightening_actions{0U};
  std::size_t engine_tightening_improvements{0U};
  std::size_t engine_decision_points{0U};
  std::size_t engine_regional_action_switches{0U};
  std::size_t engine_regional_continuations_resumed{0U};
  RoutingEngineProgressBudget engine_regional_work_budget{};
  RoutingEngineProgressBudget engine_preparation_work_budget{};
  RoutingEngineProgressBudget engine_global_retry_budget{};
  RoutingEngineProgressBudget engine_changed_context_retry_budget{};
  std::size_t engine_physical_context_changes{0U};
  std::size_t engine_preparation_budget_exhaustions{0U};
  std::size_t engine_global_retry_credit_events{0U};
  std::size_t engine_regional_work_credit_events{0U};
  std::size_t engine_regional_work_budget_exhaustions{0U};
  std::size_t engine_experiment_initial_explorations{0U};
  std::size_t engine_experiment_state_suppressions{0U};
  std::size_t engine_experiment_started{0U};
  std::size_t engine_experiment_suspended{0U};
  std::size_t engine_experiment_resumed{0U};
  std::size_t engine_experiment_invalidated{0U};
  std::size_t engine_experiment_retired{0U};
  std::size_t engine_experiment_memory_evictions{0U};
  std::uint64_t engine_experiment_active_us{0U};
  std::size_t engine_preparation_reuses{0U};
  std::size_t engine_speculative_preparations_started{0U};
  std::size_t engine_speculative_preparations_used{0U};
  std::size_t engine_speculative_preparations_discarded{0U};

};

struct PersistentRoutingSessionConfig {
  RoutingPlannerConfig planner;
  DetailedRouterConfig detailed = [] {
    DetailedRouterConfig config;
    config.circular_arc_refinement = true;
    return config;
  }();
  std::size_t max_final_feedback_passes{3U};
  bool post_negotiation_sampled_conflict_repair{false};
  std::size_t max_post_negotiation_sampled_conflict_repair_cones{2U};
  // Planner/detail above remain authoritative; this config controls orchestration only.
  RoutingEngineConfig engine;
  RoutingEngineStepBudget step_budget;

  bool operator==(const PersistentRoutingSessionConfig&) const = default;
};

// Owns one complete planning-to-copper attempt.  The selected topology, regional transaction,
// routability knowledge, and exact copper checkpoint remain live across feedback passes; callers
// no longer have to reconstruct the global/regional/detail sequence independently.
[[nodiscard]] MaterializationFeedbackResult
run_persistent_routing_session(const Board& planning_board, const Board& materialization_source,
                               const PersistentRoutingSessionConfig& config = {},
                               std::stop_token stop_token = {},
                               RouterTelemetrySink* telemetry = nullptr);

// Chooses the nearest authoritative loop capable of acting on a diagnosis.  A repeated local or
// regional failure may be escalated by the session coordinator, but quantization/PH failures do
// not trigger global PathFinder work on their first observation.
[[nodiscard]] MaterializationFeedbackResult::Scope
materialization_feedback_scope(const RoutabilityDiagnosis& diagnosis);

// Reuses exact materialization failures as cost-only routability knowledge. The global planning
// board may omit unlocked source copper covered by an explicit replacement transaction, while the
// materialization source retains that copper until the complete candidate is accepted. Only a
// strictly better exact detailed result replaces the caller's checkpoint.
[[nodiscard]] MaterializationFeedbackResult refine_materialization_with_routability_feedback(
    const Board& planning_board, const Board& materialization_source,
    RoutingPlanResult initial_routing, DetailedRouteResult initial_detailed,
    const RoutingPlannerConfig& planner_config, const DetailedRouterConfig& detailed_config,
    std::size_t max_feedback_passes, std::stop_token stop_token = {},
    RouterTelemetrySink* telemetry = nullptr);

} // namespace flux
