#pragma once

#include "flux/core/detailed_router.hpp"
#include "flux/core/routing_engine_policy.hpp"
#include "flux/core/live_copper_terrain.hpp"
#include "flux/core/copper_scaffold.hpp"
#include "flux/core/regional_router.hpp"
#include "flux/core/routing_engine_policy.hpp"

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <map>
#include <optional>
#include <stop_token>
#include <string>
#include <string_view>
#include <vector>

namespace flux {

class RouterTelemetrySink;

// Evidence-driven coordinator with stack-preserving cooperative kernel continuations. Each
// advance either yields inside the current transaction or commits one finished action. Inspection
// is safe between advances; concurrent calls on the same engine are unsupported.
enum class RoutingEnginePlanningMode { split_stages, legacy_plan_routes };
enum class RoutingEngineAction {
  none,
  global_allocation,
  regional_realization,
  probe_quantized_copper,
  construct_copper_scaffold,
  materialize_copper,
  tighten_routes,
  confined_erosion_repair,
  xpbd_repair,
  usped_global_replan,
  feedback_global_replan
};
enum class RoutingEngineOutcome {
  progressed,
  yielded,
  awaiting_materialization,
  awaiting_external_validation,
  complete,
  exhausted,
  timed_out,
  cancelled
};

struct RoutingEngineStepBudget {
  // Cooperative continuation budget passed to RoutingKernel.  A zero work count uses 256 and a
  // non-positive duration uses 50 ms.
  std::size_t work_units{256U};
  std::chrono::milliseconds time_slice{50};
  // Zero means retain the corresponding configuration bound.  These are kernel input bounds,
  // not a false promise of an interruptible inner continuation.
  std::size_t global_iterations{0U};
  std::size_t regional_bundle_states{0U};
  std::size_t confined_erosion_checks{0U};
  std::size_t xpbd_iterations{0U};
  bool operator==(const RoutingEngineStepBudget&) const = default;
};

// Legacy counter representation retained for source/API compatibility. The engine records work
// consumed but does not use allowance/earned/exhausted as scheduling permission.
struct RoutingEngineProgressBudget {
  std::size_t allowance{0U};
  std::size_t consumed{0U};
  std::size_t earned{0U};

  // These operations saturate rather than wrapping; callers can safely use externally supplied
  // or accumulated kernel counters in diagnostics and tests.
  void consume(std::size_t work_units) noexcept;
  [[nodiscard]] std::size_t earn(std::size_t credit, std::size_t ceiling) noexcept;
  // Quality rank zero records evidence/completion state, so it cannot earn work by itself.
  // Every physical rank (opens, DRC, malformed geometry, bends) must be no
  // worse and at least one must improve; the supplied gate carries connectivity/topology proof.
  [[nodiscard]] static bool strict_physical_improvement(
      const RoutingQuality& previous, const RoutingQuality& candidate,
      bool connectivity_and_hard_gates_preserved) noexcept;
  [[nodiscard]] bool exhausted() const noexcept { return consumed >= allowance; }
  bool operator==(const RoutingEngineProgressBudget&) const = default;
};

struct RoutingEngineConfig {
  RoutingPlannerConfig planner;
  CopperScaffoldConfig copper_scaffold;
  DetailedRouterConfig detailed = [] {
    DetailedRouterConfig config;
    config.circular_arc_refinement = true;
    return config;
  }();
  RoutingEnginePlanningMode planning_mode{RoutingEnginePlanningMode::split_stages};
  bool materialize{true};
  bool enable_confined_erosion{true};
  bool enable_xpbd{true};
  bool enable_route_tightening{true};
  bool enable_usped_replan{true};
  bool bounded_regional_decisions{true};
  bool speculative_preparation{true};
  // Deprecated credit knobs retained for source compatibility. Scheduler experiments and the
  // shared deadline now bound production dispatch; these values do not gate eligibility.
  // Deprecated source-compatible work-credit configuration. Deadline-bounded experiment
  // scheduling controls dispatch eligibility; these fields remain telemetry/input compatibility.
  std::size_t preparation_work_units{8'000'000U};
  // Deprecated source-compatible retry-credit configuration; shared deadline and experiment
  // scheduling now bound retries.
  std::size_t initial_global_retry_attempts{2U};
  std::size_t global_retry_attempts_per_improvement{1U};
  std::size_t max_global_retry_attempts{8U};
  std::size_t max_changed_context_retries{2U};
  std::size_t initial_regional_work_units{1'000'000U};
  std::size_t regional_work_units_per_improvement{1'000'000U};
  std::size_t max_regional_work_units{30'000'000U};
  // Reconsider a regional continuation after this many completed spline candidates/lane batches without a
  // better evaluated scene. This is a work bound, not a hard wall-clock guarantee.
  std::size_t max_regional_decisions_without_improvement{32U};
  std::size_t max_decision_points{4096U};
  bool post_negotiation_sampled_conflict_repair{false};
  std::size_t max_post_negotiation_sampled_conflict_repair_cones{2U};
  // Absolute safety bound; shared deadline and physical stagnation normally stop first.
  // Bound each frozen-scene job; untouched demands remain in the canonical domain.
  std::size_t max_physical_region_requests{6U};
  std::size_t max_physical_failure_records{32U};
  std::size_t max_actions{256U};
  std::size_t max_actions_without_physical_progress{16U};
  // Deprecated: exact state-key suppression is handled by RoutingExperimentScheduleState.
  std::size_t max_repeated_action_attempts{2U};
  std::chrono::milliseconds experiment_quantum{500};
  // Applied to bounded diagnosis/repair experiments only; initial global allocation and exact
  // materialization remain governed by their own work limits/shared deadline.
  // Optional per-experiment safety cap. By default the shared deadline and completed-search
  // stagnation decide retirement; expensive preparation does not lose permission to finish.
  std::chrono::milliseconds max_experiment_active_time{0};
  std::size_t max_parked_experiments{2U};
  // Cooperative process RSS ceiling on Linux; count bound remains portable. Zero disables it.
  std::size_t max_parked_experiment_bytes{2ULL * 1024U * 1024U * 1024U};
  // A zero duration disables the engine-owned deadline.  The source is shared with every kernel
  // invocation and a timer requests cooperative cancellation without changing any legality gate.
  std::chrono::milliseconds shared_deadline{0};
  bool operator==(const RoutingEngineConfig&) const = default;
};

struct RoutingEngineAttempt {
  RoutingEngineAction action{RoutingEngineAction::none};
  std::string evidence_key;
  std::string physical_context_key;
  std::string experiment_state_key;
  std::size_t ordinal{0U};
  std::size_t work_units{0U};
  std::size_t quality_before{0U};
  RoutingQuality physical_before{};
  RoutingQuality physical_after{};
  std::size_t quality_after{0U};
  RoutingPhysicalEvidence measured_before;
  RoutingPhysicalEvidence measured_after;
  std::uint64_t elapsed_microseconds{0U};
  std::size_t live_copper_terrain_revision{0U};
  std::size_t live_copper_terrain_features{0U};
  bool live_copper_terrain_provisional{false};
  bool accepted{false};
  bool physical_improvement{false};
  bool evidence_recorded{false};
  bool cycle_suppressed{false};
  bool interrupted_at_decision{false};
  std::string diagnostic;
  bool operator==(const RoutingEngineAttempt&) const = default;
};

// Working checkpoints may be worse than the accepted checkpoint; this preserves exploration
// evidence while making exact copper selection monotonic.  accepted_detailed is set only after
// the existing complete-board validation, connectivity, and copper DRC gate succeeds.
struct RoutingPhysicalRepairEvidence {
  std::string physical_key;
  std::vector<std::size_t> request_indices;
  std::vector<RoutabilityDiagnosis> diagnoses;
  std::size_t packing_rejections{0U};
  std::size_t no_good_rejections{0U};
  std::size_t no_good_unknown_withheld{0U};
  std::size_t regional_fragments{0U};
  std::size_t regional_windows{0U};
  std::size_t regional_bundle_searches{0U};
  std::size_t regional_physical_requests{0U};
  std::size_t regional_admitted_physical_requests{0U};
  std::size_t regional_configured_blocker_objects{0U};
  std::size_t regional_blocker_objects{0U};
  bool scene_evaluated{false};
  bool candidate_available{false};
  std::string diagnostic;
  bool operator==(const RoutingPhysicalRepairEvidence&) const = default;
};

// Pure, deterministic fair selection of a causal missing-demand component. A bounded search
// failure only supplies adjacency; it never establishes an infeasible assignment.
[[nodiscard]] std::vector<std::size_t> select_physical_repair_requests(
    const CopperScaffoldResult&, const std::vector<std::size_t>& attempts, std::size_t limit);

// One canonical blocker token per request index. Satisfied or invalid indices have an empty
// token, so an evidence-only update can reopen only the missing demands whose scope changed.
[[nodiscard]] std::vector<std::string> physical_repair_evidence_tokens(
    const CopperScaffoldResult&);

// Reopen only missing requests whose canonical blocker scope changed. A retained-scene geometry
// change invalidates every prior attempt regardless of the evidence tokens.
void reconcile_physical_repair_attempts(bool geometry_changed,
                                        const std::vector<std::string>& previous_tokens,
                                        const std::vector<std::string>& current_tokens,
                                        std::vector<std::size_t>& attempts);

// Canonical evidence for only unresolved scaffold demands. This changes when an attributed
// physical blocker changes, and is independent of bounded-search termination, incidental
// evidence ordering and counters.
[[nodiscard]] std::string physical_repair_evidence_token(const CopperScaffoldResult&);

struct RoutingEngineState {
  // Exact-checked incremental copper survives unsuccessful topology/curve proposals. It is
  // provisional until full connectivity, curved qualification and external KiCad validation.
  std::shared_ptr<const CopperScaffoldResult> legal_scaffold;
  std::optional<RoutingPlanResult> working_plan;
  std::optional<DetailedRouteResult> working_detailed;
  std::optional<RoutingPlanResult> accepted_plan;
  std::optional<DetailedRouteResult> accepted_detailed;
  // Best feasibility-ranked partial evidence is distinct from an accepted exact transaction.
  std::optional<RoutingPlanResult> best_partial_plan;
  std::optional<DetailedRouteResult> best_partial_detailed;
  std::optional<RoutingPlanResult> best_detailed_plan;
  std::vector<RoutabilityDiagnosis> pending_diagnoses;
  std::vector<RoutingEngineAttempt> attempts;
  RoutingEngineAction next_action{RoutingEngineAction::global_allocation};
  RoutingEngineOutcome outcome{RoutingEngineOutcome::progressed};
  std::size_t generation{0U};
  std::size_t global_iterations_used{0U};
  LiveCopperTerrainSnapshot live_copper_terrain{};
  std::size_t live_copper_terrain_revision{0U};
  std::size_t live_copper_terrain_snapshot_changes{0U};
  std::size_t live_copper_terrain_features{0U};
  std::size_t live_copper_terrain_provisional_replacements{0U};
  std::size_t live_copper_terrain_exact_replacements{0U};
  bool external_validation_required{false};
  std::size_t tightening_actions{0U};
  std::size_t tightening_improvements{0U};
  std::size_t decision_points{0U};
  std::size_t regional_action_switches{0U};
  std::size_t regional_continuations_resumed{0U};
  RoutingEngineProgressBudget regional_work_budget{};
  RoutingEngineProgressBudget preparation_work_budget{};
  RoutingEngineProgressBudget global_retry_budget{}; // Attempts, not kernel work units.
  RoutingEngineProgressBudget changed_context_retry_budget{};
  std::string physical_context_key;
  std::size_t physical_context_changes{0U};
  std::size_t preparation_budget_exhaustions{0U};
  std::size_t global_retry_credit_events{0U};
  // Best measured regional/oracle physical scene across actions.
  std::optional<RoutingQuality> best_evaluated_physical_scene;
  std::size_t regional_work_credit_events{0U};
  std::size_t regional_work_budget_exhaustions{0U};
  std::size_t preparation_reuses{0U};
  std::size_t speculative_preparations_started{0U};
  std::size_t speculative_preparations_used{0U};
  std::size_t speculative_preparations_discarded{0U};
  // Bounded diagnosis-experiment selection is kept independently of legacy credit counters so
  // telemetry can distinguish scheduling suppression from legality/acceptance decisions.
  RoutingExperimentScheduleState experiment_schedule{};
  std::uint64_t experiment_dispatch_sequence{0U};
  std::size_t experiment_initial_explorations{0U};
  std::size_t experiment_state_suppressions{0U};
  std::size_t experiment_started{0U};
  std::size_t scaffold_regional_actions{0U};
  std::size_t physical_preparation_reuses{0U};
  std::vector<RoutingPhysicalRepairEvidence> physical_repair_evidence;
  std::optional<QuantizedCopperOracleResult> scaffold_oracle;
  std::string scaffold_oracle_key;
  std::size_t scaffold_regional_missing_requests{0U};
  // Request-local physical terminal rebound is deliberately narrower than a net rewrite. These
  // counters/proofs describe only demands whose noncopper abstract endpoint was replaced after
  // a zero-attachment failure by a pad proved reachable through another selected coarse branch.
  std::size_t scaffold_terminal_rebinds{0U};
  std::vector<std::string> scaffold_terminal_rebind_proofs;
  std::vector<std::string> scaffold_terminal_rebind_attempted_proofs;
  std::size_t experiment_suspended{0U};
  std::size_t experiment_resumed{0U};
  std::size_t experiment_invalidated{0U};
  std::size_t experiment_retired{0U};
  std::size_t experiment_memory_evictions{0U};
  std::uint64_t experiment_active_us{0U};
  std::string regional_phase;
  bool operator==(const RoutingEngineState&) const = default;
};

struct RoutingEngineStepResult {
  RoutingEngineOutcome outcome{RoutingEngineOutcome::progressed};
  RoutingEngineAction action{RoutingEngineAction::none};
  std::size_t work_units{0U};
  std::size_t generation{0U};
  bool accepted_checkpoint_changed{false};
  bool decision_point{false};
  std::string diagnostic;
  bool operator==(const RoutingEngineStepResult&) const = default;
};

[[nodiscard]] const char* to_string(RoutingEngineAction action);
[[nodiscard]] const char* to_string(RoutingEngineOutcome outcome);
[[nodiscard]] bool routing_engine_action_uses_physical_continuation(
    RoutingEngineAction action) noexcept;
// Pure continuation dependency key used by engine tests and parked-work admission. Regional
// actions use only physical solver inputs; other actions retain the supplied full scheduler key.
[[nodiscard]] std::string routing_engine_continuation_dependency_key(
    RoutingEngineAction action, const RoutingPlanResult* plan,
    const LiveCopperTerrainSnapshot& terrain, std::string_view experiment_state_key);
[[nodiscard]] std::string routing_engine_negotiation_dependency_key(
    const RoutabilityKnowledge& knowledge);
void retain_current_routability_knowledge(RoutabilityKnowledge& checkpoint,
                                          const RoutabilityKnowledge& current,
                                          std::size_t maximum_entries);

class RoutingEngine {
public:
  RoutingEngine(Board planning_board, Board materialization_source, RoutingEngineConfig config = {},
                RouterTelemetrySink* telemetry = nullptr);
  ~RoutingEngine();
  RoutingEngine(const RoutingEngine&) = delete;
  RoutingEngine& operator=(const RoutingEngine&) = delete;
  RoutingEngine(RoutingEngine&&) = delete;
  RoutingEngine& operator=(RoutingEngine&&) = delete;

  [[nodiscard]] RoutingEngineStepResult advance(RoutingEngineStepBudget budget = {},
                                                std::stop_token stop_token = {});
  // External KiCad DRC belongs outside the core.  A successful internal transaction remains
  // pending until its caller supplies this acknowledgement.
  void acknowledge_external_validation(std::size_t generation, bool passed,
                                       std::vector<RoutabilityDiagnosis> diagnoses = {},
                                       std::string diagnostic = {});
  [[nodiscard]] const RoutingEngineState& state() const noexcept { return state_; }
  [[nodiscard]] const RoutingPlanResult* selected_plan() const noexcept;
  [[nodiscard]] const DetailedRouteResult* selected_detailed() const noexcept;
  [[nodiscard]] bool terminal() const noexcept;

private:
  struct Deadline;
  struct Pending;
  struct Speculation;
  [[nodiscard]] RoutingEngineAction select_action(RoutingEngineAction excluded = RoutingEngineAction::none);
  [[nodiscard]] std::string experiment_state_key() const;
  [[nodiscard]] std::string continuation_key(RoutingEngineAction action) const;
  std::string physical_dependency_key(bool retained_scene = false,
                                      bool include_evidence = true) const;
  void record_experiment(const RoutingEngineAttempt& attempt, bool finished = true);
  bool yield_experiment(bool force = false);
  void prune_parked_experiments();
  void release_pending();
  [[nodiscard]] std::string evidence_key(RoutingEngineAction action) const;
  [[nodiscard]] std::size_t working_quality() const;
  void collect_diagnoses();
  void schedule_after_plan();
  [[nodiscard]] bool needs_tightening() const;
  void schedule_after_detailed();
  void replace_live_copper_terrain(LiveCopperTerrainSnapshot snapshot);
  void publish_retained_copper(bool force = false);
  void start_speculative_preparation();
  void stop_speculative_preparation();
  void acquire_regional_preparation();

  Board planning_board_;
  Board materialization_source_;
  RoutingEngineConfig config_;
  RouterTelemetrySink* telemetry_{nullptr};
  std::weak_ptr<const CopperScaffoldResult> published_scaffold_;
  std::chrono::steady_clock::time_point last_copper_publication_{};
  RoutingEngineState state_;
  bool working_plan_tightened_{false};
  std::size_t actions_without_physical_progress_{0U};
  std::unique_ptr<Deadline> deadline_;
  std::unique_ptr<Pending> pending_;
  std::vector<std::unique_ptr<Pending>> parked_;
  mutable std::map<std::string, std::size_t> experiment_keys_;
  mutable std::map<std::string, std::size_t> physical_keys_;
  mutable std::map<std::string, std::size_t> physical_records_;
  mutable std::size_t experiment_key_bytes_{0U};
  mutable bool experiment_metadata_exhausted_{false};
  std::shared_ptr<RegionalRoutingPreparation> regional_preparation_;
  std::shared_ptr<RegionalRoutingPreparation> physical_preparation_;
  std::string physical_preparation_key_;
  std::string physical_request_attempt_key_;
  std::string physical_request_geometry_key_;
  std::vector<std::string> physical_request_evidence_tokens_;
  std::vector<std::size_t> physical_request_attempts_;
  std::unique_ptr<Speculation> speculation_;
};

} // namespace flux
