#include "flux/core/routing_engine.hpp"
#include "flux/core/routing_feedback.hpp"
#include "flux/core/router_telemetry.hpp"
#include "../../src/scaffold_terminal_rebind.hpp"
#include "../../src/scaffold_physical_demands.hpp"
#include "../../src/scaffold_access.hpp"

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <stop_token>
#include <set>
#include <string>
#include <thread>

namespace {
using namespace flux;

[[noreturn]] void fail(const char* message) {
  std::cerr << message << '\n';
  std::exit(1);
}
void require(bool value, const char* message) {
  if (!value) fail(message);
}
class CopperSnapshotSink final : public RouterTelemetrySink {
public:
  bool accepts(RouterTelemetryLevel) const noexcept override { return true; }
  bool try_emit(RouterTelemetryEvent event) noexcept override {
    if (event.kind == RouterTelemetryEventKind::search_summary) {
      if (event.progress_phase == "copper-scaffold / begin") ++scaffold_begins;
      if (event.progress_phase == "regional / begin") ++regional_begins;
      if (event.progress_phase == "regional / end") ++regional_ends;
    }
    return true;
  }
  void publish_generated_copper(const Board& board, std::string) override {
    latest = board;
    ++publications;
  }
  std::optional<Board> latest;
  std::size_t publications{0U};
  std::size_t scaffold_begins{0U};
  std::size_t regional_begins{0U};
  std::size_t regional_ends{0U};
};

[[nodiscard]] Point mm(double x, double y) {
  return {Nanometres::from_mm(x), Nanometres::from_mm(y)};
}

[[nodiscard]] Board two_pad_board() {
  const LayerId front{"F.Cu"};
  const LayerId back{"B.Cu"};
  const NetId net{"engine-experiment-net"};
  Board board;
  board.source_revision = "routing-engine-experiment-fixture";
  board.outline = Polygon{{mm(0.0, 0.0), mm(100.0, 0.0), mm(100.0, 80.0), mm(0.0, 80.0)}};
  board.layers = {{front, "F.Cu", LayerKind::copper, 0U},
                  {back, "B.Cu", LayerKind::copper, 1U}};
  board.nets = {{net, "SIGNAL", 0U}};
  const auto footprint = [&](std::string id, Point position) {
    Footprint value;
    value.id = ObjectId{id};
    value.reference = id;
    value.position = position;
    value.courtyard = Polygon{{mm(-4.0, -3.0), mm(4.0, -3.0), mm(4.0, 3.0), mm(-4.0, 3.0)}};
    value.pads = {{ObjectId{id + "-pad"}, net, Point{}, {front}}};
    return value;
  };
  board.footprints = {footprint("U1", mm(20.0, 40.0)), footprint("U2", mm(80.0, 40.0))};
  return board;
}

[[nodiscard]] RoutingEngineConfig experiment_config() {
  RoutingEngineConfig config;
  // These fixtures specifically exercise the legacy regional continuation scheduler.
  config.copper_scaffold.enabled = false;
  config.speculative_preparation = false;
  config.materialize = false;
  config.planner.global.grid_pitch = Nanometres::from_mm(5.0);
  config.planner.global.max_iterations = 1U;
  config.planner.global.max_pin_escape_grid_radius = 2U;
  config.planner.global.max_pin_access_nodes_per_terminal = 32U;
  config.planner.max_routability_feedback_retries = 0U;
  config.max_actions = 8U;
  return config;
}

void physical_regions_are_causal_bounded_and_fair() {
  const RoutingEngineConfig defaults;
  require(defaults.scaffold_failed_requests_per_regional_handoff == 4U &&
              defaults.max_physical_region_requests == 6U,
          "large scaffold sweeps must batch a small bounded failure prefix by default");
  CopperScaffoldResult scaffold;
  for (std::size_t i = 0; i < 4; ++i) {
    GlobalRoutePath request;
    request.net = NetId{std::to_string(i)};
    scaffold.requests.push_back(request);
    scaffold.missing_indices.push_back(i);
    CopperScaffoldRequestEvidence evidence;
    evidence.blocker_nets = {NetId{i < 3 ? "shared" : "unrelated"}};
    evidence.termination = CopperScaffoldSearchTermination::expansion_budget_exhausted;
    scaffold.request_evidence.push_back(evidence);
  }
  require(select_physical_repair_requests(scaffold, {}, 2U) == std::vector<std::size_t>{0U, 1U},
          "causal region must obey its hard demand bound");
  require(select_physical_repair_requests(scaffold, {1U, 1U, 0U, 0U}, 2U) ==
              std::vector<std::size_t>{2U},
          "previously attempted neighbours must not starve an untouched demand");
  require(select_physical_repair_requests(scaffold, {1U, 1U, 1U, 0U}, 2U) ==
              std::vector<std::size_t>{3U},
          "an unrelated unresolved component must receive its own job");
  require(select_physical_repair_requests(scaffold, {0U, 0U, 3U, 0U}, 1U, {2U}) ==
              std::vector<std::size_t>{2U},
          "new failure evidence must receive its handoff before older failures are retried");
  scaffold.sweep_in_progress = true;
  scaffold.failed_request_indices = {2U};
  require(select_physical_repair_requests(scaffold, {}, 3U) == std::vector<std::size_t>{2U},
          "an ongoing scaffold must not admit deferred requests as failed regional demands");
  scaffold.sweep_in_progress = false;
  require(select_physical_repair_requests(scaffold, {}, 0U).empty(),
          "zero regional request allowance must not dispatch work");
}

void physical_repair_evidence_token_is_canonical_and_causal() {
  CopperScaffoldResult scaffold;
  scaffold.requests = {{.net = NetId{"missing-net"}, .connection_index = 7U}};
  scaffold.missing_indices = {0U};
  CopperScaffoldRequestEvidence evidence;
  evidence.termination = CopperScaffoldSearchTermination::expansion_budget_exhausted;
  evidence.blocker_objects = {ObjectId{"blocker-b"}, ObjectId{"blocker-a"}, ObjectId{"blocker-b"}};
  scaffold.request_evidence = {evidence};
  const auto canonical = physical_repair_evidence_token(scaffold);

  scaffold.request_evidence.front().blocker_objects =
      {ObjectId{"blocker-a"}, ObjectId{"blocker-b"}};
  require(physical_repair_evidence_token(scaffold) == canonical,
          "blocker ordering and duplicates must not invalidate physical repair state");

  scaffold.request_evidence.front().blocker_objects.push_back(ObjectId{"blocker-c"});
  require(physical_repair_evidence_token(scaffold) != canonical,
          "new attributed blocker geometry must reopen its missing repair demand");

  scaffold.request_evidence.front().blocker_objects = {ObjectId{"blocker-a"}, ObjectId{"blocker-b"}};
  scaffold.request_evidence.front().termination = CopperScaffoldSearchTermination::search_exhausted;
  require(physical_repair_evidence_token(scaffold) == canonical,
          "termination changes without new blockers must not reopen the same repair demand");
}

void physical_repair_evidence_reopens_only_changed_missing_demand() {
  CopperScaffoldResult scaffold;
  scaffold.requests = {{.net = NetId{"low"}, .connection_index = 0U},
                       {.net = NetId{"high"}, .connection_index = 1U}};
  scaffold.missing_indices = {0U, 1U};
  scaffold.request_evidence.resize(2U);
  scaffold.request_evidence[0].blocker_objects = {ObjectId{"low-blocker"}};
  scaffold.request_evidence[1].blocker_objects = {ObjectId{"high-blocker"}};
  const auto prior_tokens = physical_repair_evidence_tokens(scaffold);
  std::vector<std::size_t> attempts{3U, 3U};

  scaffold.request_evidence[1].blocker_objects = {ObjectId{"new-high-blocker"}};
  const auto current_tokens = physical_repair_evidence_tokens(scaffold);
  reconcile_physical_repair_attempts(false, prior_tokens, current_tokens, attempts);

  require(attempts == std::vector<std::size_t>{3U, 0U},
          "an evidence-only update must preserve attempts for unchanged missing demands");
  require(select_physical_repair_requests(scaffold, attempts, 1U) == std::vector<std::size_t>{1U},
          "a changed higher-index demand must be reopened ahead of already-attempted demands");

  attempts = {3U, 3U};
  reconcile_physical_repair_attempts(true, current_tokens, current_tokens, attempts);
  require(attempts == std::vector<std::size_t>{0U, 0U},
          "a retained-scene geometry change must reset every physical repair attempt");
}

void strict_scaffold_coverage_progress_excludes_numeric_and_metadata_changes() {
  CopperScaffoldResult incumbent;
  incumbent.request_count = 2U;
  incumbent.request_identities = {"coverage-a", "coverage-b"};
  incumbent.satisfied_indices = {0U};
  auto numeric_only = incumbent;
  numeric_only.counters.expansions = 99U;
  numeric_only.diagnostic = "new search telemetry";
  numeric_only.request_evidence.resize(2U);
  numeric_only.request_evidence[1].termination = CopperScaffoldSearchTermination::search_exhausted;
  require(!routing_scaffold_strict_coverage_improvement(numeric_only, incumbent),
          "numeric counters and negative evidence must not reset scaffold stagnation");
  auto lost_cover = incumbent;
  lost_cover.satisfied_indices.clear();
  require(!routing_scaffold_strict_coverage_improvement(lost_cover, incumbent),
          "a checkpoint that drops checked coverage must not reset scaffold stagnation");
  auto gained_cover = incumbent;
  gained_cover.satisfied_indices.push_back(1U);
  require(routing_scaffold_strict_coverage_improvement(gained_cover, incumbent),
          "preserved same-domain checked coverage gain must reset scaffold stagnation");
  gained_cover.request_identities[1] = "reconstructed-domain";
  require(!routing_scaffold_strict_coverage_improvement(gained_cover, incumbent),
          "coverage in a reconstructed request domain must not reset scaffold stagnation");
}

void parked_engine_resumes_without_committing_partial_state() {
  const auto board = two_pad_board();
  auto config = experiment_config();
  config.progressive_global_preparation_connection_threshold = 1U;
  RoutingEngine engine(board, board, config);
  const auto first = engine.advance({.work_units = 1U, .time_slice = std::chrono::milliseconds{20}});
  require(first.outcome == RoutingEngineOutcome::yielded && engine.state().attempts.empty() &&
              !engine.selected_plan(),
          "a short first quantum must park the transaction without publishing partial routing");

  const auto active_before_pause = engine.state().experiment_active_us;
  std::this_thread::sleep_for(std::chrono::milliseconds{100});
  const auto after_pause = engine.advance(
      {.work_units = 1U, .time_slice = std::chrono::milliseconds{1}});
  const auto active_after_pause = engine.state().experiment_active_us;
  require(after_pause.outcome == RoutingEngineOutcome::yielded &&
              active_after_pause >= active_before_pause &&
              active_after_pause - active_before_pause < 40'000U,
          "caller pause time must not be charged to a parked experiment's active-time quantum");

  bool resumed = after_pause.outcome == RoutingEngineOutcome::yielded;
  for (std::size_t step = 0U; step < 20'000U && !engine.terminal(); ++step) {
    const auto result = engine.advance({.work_units = 1U, .time_slice = std::chrono::milliseconds{20}});
    resumed = resumed || result.outcome == RoutingEngineOutcome::yielded;
  }
  require(resumed && engine.state().regional_continuations_resumed != 0U &&
              engine.state().outcome == RoutingEngineOutcome::awaiting_materialization &&
              engine.selected_plan() != nullptr,
          "a parked engine must resume the retained transaction to its normal plan-only handoff");
  require(engine.state().working_plan &&
              engine.state().working_plan->coarse_global.configured_preparation_mode ==
                  GlobalPreparationMode::progressive_seed &&
              engine.state().working_plan->coarse_global
                      .configured_static_seed_preparation_timeout_ms ==
                  static_cast<std::size_t>(config.initial_global_preparation_timeout.count()),
          "a demand-qualified initial allocation must request bounded progressive preparation");
}

void cancellation_and_deadline_dispose_parked_engine_work() {
  const auto board = two_pad_board();
  const auto config = experiment_config();
  RoutingEngine cancelled(board, board, config);
  require(cancelled.advance({.work_units = 1U}).outcome == RoutingEngineOutcome::yielded,
          "cancellation fixture must first park a live transaction");
  std::stop_source stop;
  stop.request_stop();
  require(cancelled.advance({}, stop.get_token()).outcome == RoutingEngineOutcome::cancelled &&
              cancelled.state().outcome == RoutingEngineOutcome::cancelled &&
              cancelled.state().attempts.empty() && !cancelled.selected_plan() &&
              !cancelled.selected_detailed(),
          "cancelling a parked transaction must clear uncommitted global and detailed checkpoints");
  const auto session = run_persistent_routing_session(board, board, {}, stop.get_token());
  require(session.engine_outcome == RoutingEngineOutcome::cancelled &&
              session.routing.coarse_global.stop_reason == GlobalRouterStopReason::cancelled &&
              session.detailed.stop_reason == DetailedRouterStopReason::cancelled &&
              !session.detailed.candidate,
          "a stopped session without checkpoints must not report default invalid input or copper");

  auto timed_config = config;
  timed_config.shared_deadline = std::chrono::milliseconds{100};
  RoutingEngine timed(board, board, timed_config);
  require(timed.advance({.work_units = 1U}).outcome == RoutingEngineOutcome::yielded,
          "deadline fixture must first park a live transaction");
  std::this_thread::sleep_for(std::chrono::milliseconds{150});
  require(timed.advance().outcome == RoutingEngineOutcome::timed_out && timed.state().attempts.empty() &&
              timed.state().outcome == RoutingEngineOutcome::timed_out && !timed.selected_plan() &&
              !timed.selected_detailed(),
          "deadline expiry must remain distinct from cancellation without publishing checkpoints");
}

void retained_checkpoint_preview_never_claims_final_assessment() {
  const auto source = two_pad_board();
  CopperScaffoldResult retained;
  retained.copper = source;
  retained.complete = true;
  retained.copper.tracks.push_back({ObjectId{"preview-added-track"}, source.nets.front().id,
      LayerId{"F.Cu"}, mm(10, 10), mm(20, 10), Nanometres::from_mm(0.25), false});
  const auto before = retained.copper;
  for (const auto outcome : {RoutingEngineOutcome::exhausted, RoutingEngineOutcome::timed_out,
                             RoutingEngineOutcome::cancelled}) {
    const auto preview = retained_scaffold_diagnostic_preview(source, retained, outcome);
    require(preview.candidate && *preview.candidate == before && preview.added_tracks == 1U &&
                !preview.complete() && !preview.provisional_zone_refill_candidate() &&
                preview.diagnostic.find("unassessed") != std::string::npos,
            "retained export must copy committed copper without inventing final DRC or curve success");
  }
  require(retained.copper == before,
          "diagnostic export must leave the retained transaction checkpoint immutable");
}

void continuation_dependencies_and_knowledge_merge_are_bounded() {
  require(routing_engine_action_uses_physical_continuation(
              RoutingEngineAction::regional_realization) &&
              routing_engine_action_uses_physical_continuation(RoutingEngineAction::xpbd_repair) &&
              routing_engine_action_uses_physical_continuation(
                  RoutingEngineAction::confined_erosion_repair),
          "regional continuations must invalidate on physical geometry changes");
  require(!routing_engine_action_uses_physical_continuation(
              RoutingEngineAction::feedback_global_replan) &&
              !routing_engine_action_uses_physical_continuation(
                  RoutingEngineAction::usped_global_replan),
          "global continuations must remain dependent on their full history input");

  const NetId net{"continuation-net"};
  const LayerId front{"F.Cu"};
  RoutingPlanResult plan;
  GlobalRoutePath path;
  path.net = net;
  path.points = {{mm(1.0, 1.0), front}, {mm(2.0, 1.0), front}};
  path.segment_widths = {Nanometres::from_mm(0.20)};
  plan.paths = {path};
  plan.coarse_global.paths = {path};
  RegionalSplineRealization spline;
  spline.fragment.net = net;
  spline.layer = front;
  spline.width = Nanometres::from_mm(0.20);
  spline.native_pieces = {{.start = mm(1.0, 1.0), .end = mm(2.0, 1.0),
                           .control1 = mm(1.25, 1.0), .control2 = mm(1.75, 1.0)}};
  plan.regional.bundle_assignments = {{.spline_realizations = {spline}}};
  LiveCopperTerrainSnapshot terrain;
  terrain.features = {{.kind = LiveCopperTerrainKind::segment, .net = net, .layer = front,
                       .bounds = {mm(0.5, 0.5), mm(2.5, 1.5)}, .axis_start = mm(1.0, 1.0),
                       .axis_end = mm(2.0, 1.0), .radius = Nanometres::from_mm(0.10)}};
  const auto regional_key = routing_engine_continuation_dependency_key(
      RoutingEngineAction::xpbd_repair, &plan, terrain, "history-A");
  require(regional_key == routing_engine_continuation_dependency_key(
                              RoutingEngineAction::xpbd_repair, &plan, terrain, "history-B"),
          "history-only scheduler changes must preserve a physical regional continuation");
  auto history_changed = plan;
  history_changed.routability_knowledge.history.push_back(
      {RoutabilityFailureKind::packing_capacity_exceeded,
       RoutabilityResourceReference{.stable_id = "history-only"}, 2U, 7U,
       RoutabilityFailureStage::final_validation, RoutabilityRelaxation::change_lane_order});
  history_changed.routability_knowledge.diagnoses.push_back(
      {.kind = RoutabilityFailureKind::packing_capacity_exceeded,
       .resource = RoutabilityResourceReference{.stable_id = "diagnosis-only"}});
  require(regional_key == routing_engine_continuation_dependency_key(
                              RoutingEngineAction::xpbd_repair, &history_changed, terrain, "history-B"),
          "history pressure and diagnoses must not invalidate unchanged regional geometry");
  require(routing_engine_continuation_dependency_key(
              RoutingEngineAction::feedback_global_replan, &plan, terrain, "history-A") !=
              routing_engine_continuation_dependency_key(
                  RoutingEngineAction::feedback_global_replan, &plan, terrain, "history-B"),
          "global continuation keys must retain history dependency");
  const auto original_knowledge = history_changed.routability_knowledge;
  const auto original_history_key = routing_engine_negotiation_dependency_key(original_knowledge);
  const auto changed_history = [&](auto mutate) {
    auto knowledge = original_knowledge;
    mutate(knowledge);
    const auto key = routing_engine_negotiation_dependency_key(knowledge);
    require(key != original_history_key, "global dependency must cover every negotiated cost input");
    auto changed_plan = plan;
    changed_plan.routability_knowledge = std::move(knowledge);
    require(regional_key == routing_engine_continuation_dependency_key(
        RoutingEngineAction::xpbd_repair, &changed_plan, terrain, key),
        "the same history mutation must preserve physical regional validity");
  };
  changed_history([](auto& k) { ++k.history.front().observations; });
  changed_history([](auto& k) { ++k.history.front().pressure; });
  changed_history([](auto& k) { k.history.front().kind = RoutabilityFailureKind::curvature_infeasible; });
  changed_history([](auto& k) { k.history.front().stage = RoutabilityFailureStage::spline_realization; });
  changed_history([](auto& k) { k.history.front().resource.bounds.maximum = mm(4.0, 5.0); });
  changed_history([](auto& k) { k.history.front().resource.nets.push_back(NetId{"new-net"}); });
  changed_history([](auto& k) { k.history.front().resource.layer = LayerId{"B.Cu"}; });
  changed_history([](auto& k) { k.success_history.push_back({.resource = {.stable_id = "clean"},
                                                          .start = mm(1.0, 1.0),
                                                          .end = mm(2.0, 1.0), .strength = 2U}); });
  auto changed = plan;
  changed.coarse_global.paths.front().points.front().position = mm(1.1, 1.0);
  require(regional_key != routing_engine_continuation_dependency_key(
                              RoutingEngineAction::xpbd_repair, &changed, terrain, "history-A"),
          "coarse path geometry must invalidate a regional continuation");
  changed = plan;
  changed.paths.front().points.front().layer = LayerId{"B.Cu"};
  require(regional_key != routing_engine_continuation_dependency_key(
                              RoutingEngineAction::xpbd_repair, &changed, terrain, "history-A"),
          "candidate layer changes must invalidate a regional continuation");
  changed = plan;
  changed.paths.front().segment_widths.front() = Nanometres::from_mm(0.25);
  require(regional_key != routing_engine_continuation_dependency_key(
                              RoutingEngineAction::xpbd_repair, &changed, terrain, "history-A"),
          "candidate width changes must invalidate a regional continuation");
  changed = plan;
  changed.regional.bundle_assignments.front().spline_realizations.front().native_pieces.front().control1 =
      mm(1.30, 1.0);
  require(regional_key != routing_engine_continuation_dependency_key(
                              RoutingEngineAction::xpbd_repair, &changed, terrain, "history-A"),
          "native control geometry must invalidate a regional continuation");
  changed = plan;
  changed.coarse_global.paths.front().segment_width_kinds = {GlobalRouteSegmentWidthKind::terminal_neck_down};
  require(regional_key != routing_engine_continuation_dependency_key(
      RoutingEngineAction::xpbd_repair, &changed, terrain, "history-A"),
      "width authorization changes must invalidate regional preparation even at the same width");
  changed = plan;
  changed.coarse_global.paths.front().geometric_corridors.push_back({.layer = front,
      .topology_cells = {Polygon{{mm(0.0, 0.0), mm(3.0, 0.0), mm(3.0, 3.0)}}},
      .minimum_physical_cross_section_width = Nanometres::from_mm(0.3), .lane_capacity = 1U});
  const auto corridor_key = routing_engine_continuation_dependency_key(
      RoutingEngineAction::xpbd_repair, &changed, terrain, "history-A");
  require(regional_key != corridor_key, "corridor topology must enter regional dependency identity");
  changed.coarse_global.paths.front().geometric_corridors.front().minimum_physical_cross_section_width =
      Nanometres::from_mm(0.4);
  require(corridor_key != routing_engine_continuation_dependency_key(
      RoutingEngineAction::xpbd_repair, &changed, terrain, "history-A"),
      "corridor packing constraints must invalidate regional preparation");
  auto changed_terrain = terrain;
  changed_terrain.features.front().axis_end = mm(2.1, 1.0);
  require(regional_key != routing_engine_continuation_dependency_key(
                              RoutingEngineAction::xpbd_repair, &plan, changed_terrain, "history-A"),
          "live copper geometry must invalidate a regional continuation");

  RoutabilityKnowledge checkpoint;
  checkpoint.history.push_back({RoutabilityFailureKind::topological_trap,
                                RoutabilityResourceReference{.stable_id = "stale-region"},
                                1U, 1U, RoutabilityFailureStage::regional_discrete,
                                RoutabilityRelaxation::change_corridor});
  RoutabilityKnowledge current;
  current.history.push_back({RoutabilityFailureKind::packing_capacity_exceeded,
                             RoutabilityResourceReference{.stable_id = "newer-region"},
                             4U, 9U, RoutabilityFailureStage::final_validation,
                             RoutabilityRelaxation::change_lane_order});
  checkpoint.diagnoses.push_back({.kind = RoutabilityFailureKind::topological_trap,
                                  .resource = RoutabilityResourceReference{.stable_id = "stale-diagnosis"}});
  current.diagnoses.push_back({.kind = RoutabilityFailureKind::packing_capacity_exceeded,
                               .resource = RoutabilityResourceReference{.stable_id = "resolved-now"}});
  retain_current_routability_knowledge(checkpoint, current, 1U);
  require(checkpoint.history.size() == 1U &&
              checkpoint.history.front().resource.stable_id == "newer-region" &&
              checkpoint.history.front().pressure == 9U,
          "newer feedback must replace stale unique history when a resumed snapshot is bounded");
  require(checkpoint.diagnoses == current.diagnoses,
          "current diagnoses must replace stale snapshot diagnoses at resumed commit");
}

void external_rejection_switches_a_real_experiment_pool_before_safe_cancellation() {
  const auto board = two_pad_board();
  auto config = experiment_config();
  config.materialize = true;
  config.planner.curved_tracks = true;
  config.experiment_quantum = std::chrono::milliseconds{1};
  config.max_actions = 32U;
  config.max_decision_points = 4'096U;
  config.max_parked_experiments = 1U;
  config.max_parked_experiment_bytes = 0U;
  config.planner.regional.xpbd.iterations = 1U;
  RoutingEngine engine(board, board, config);

  bool externally_rejected = false;
  std::size_t suspended_at_rejection = 0U;
  std::size_t resumed_at_rejection = 0U;
  for (std::size_t step = 0U; step < 100'000U &&
                              (!externally_rejected ||
                               engine.state().experiment_resumed <= resumed_at_rejection) &&
                              !engine.terminal();
       ++step) {
    const auto result = engine.advance(
        {.work_units = 1U, .time_slice = std::chrono::milliseconds{1}});
    if (!externally_rejected && result.outcome == RoutingEngineOutcome::awaiting_external_validation) {
      RoutabilityDiagnosis rejection;
      rejection.resource.stable_id = "engine-experiment-external-corridor";
      rejection.resource.nets = {NetId{"engine-experiment-net"}};
      rejection.suggested_relaxation = RoutabilityRelaxation::change_corridor;
      engine.acknowledge_external_validation(engine.state().generation, false, {rejection});
      suspended_at_rejection = engine.state().experiment_suspended;
      resumed_at_rejection = engine.state().experiment_resumed;
      externally_rejected = true;
    }
  }
  require(externally_rejected && engine.state().experiment_suspended > suspended_at_rejection,
          "an externally rejected curved handoff must park an alternate repair experiment");
  require(engine.state().experiment_resumed > resumed_at_rejection &&
              engine.state().experiment_memory_evictions == 0U,
          "a full one-slot pool must resume live repair before admitting work that evicts its frontier");
  require(engine.state().working_plan.has_value(),
          "a parked experiment must retain the latest public routing checkpoint");
  const auto retained_paths = engine.state().working_plan->paths;
  std::stop_source stop;
  stop.request_stop();
  const auto cancelled = engine.advance({}, stop.get_token());
  require(cancelled.outcome == RoutingEngineOutcome::cancelled &&
              engine.state().working_plan.has_value() &&
              engine.state().working_plan->paths == retained_paths,
          "cancelling a parked experiment pool must release workers without discarding its best checkpoint");
}
void legal_scaffold_bypasses_regional_preparation() {
  const auto board = two_pad_board();
  auto config = experiment_config();
  config.copper_scaffold.enabled = true;
  RoutingEngine engine(board, board, config);
  for (std::size_t step = 0; step < 20'000U && !engine.terminal(); ++step)
    static_cast<void>(engine.advance({.work_units = 256U}));
  require(engine.state().outcome == RoutingEngineOutcome::awaiting_materialization,
          "legal scaffold must reach plan-only materialization handoff");
  require(engine.state().legal_scaffold && engine.state().legal_scaffold->complete &&
              engine.selected_plan() && engine.selected_plan()->legal_scaffold_complete &&
              engine.selected_plan()->quantized_copper_oracle.feasible(),
          "handoff requires retained scaffold and whole-board exact connectivity/DRC evidence");
  require(engine.state().scaffold_oracle && engine.state().scaffold_oracle->feasible(),
          "scaffold certification evidence must survive independently of plan promotion");
  require(std::none_of(engine.state().attempts.begin(), engine.state().attempts.end(),
              [](const auto& a) { return a.action == RoutingEngineAction::regional_realization; }),
          "legal scaffold should not pay regional portfolio preparation before fitting");
}
void scaffold_certifies_materialization_source() {
  const auto planning = two_pad_board();
  auto source = planning;
  // This physical obstacle is deliberately absent from the coarse planning representation.
  source.nets.push_back({NetId{"other-net"}, "OTHER", 0U});
  source.tracks.push_back({ObjectId{"source-only-barrier"}, NetId{"other-net"},
      LayerId{"F.Cu"}, mm(50, 38), mm(50, 42), Nanometres::from_mm(0.5), false});
  source.tracks.push_back({ObjectId{"source-only-barrier-back"}, NetId{"other-net"},
      LayerId{"B.Cu"}, mm(50, 38), mm(50, 42), Nanometres::from_mm(0.5), false});
  GlobalRouteResult direct;
  direct.paths = {GlobalRoutePath{NetId{"engine-experiment-net"}, 0U,
                                  {{mm(20, 40), LayerId{"F.Cu"}},
                                   {mm(80, 40), LayerId{"F.Cu"}}}}};
  require(!probe_quantized_copper_feasibility(source, direct).feasible(),
          "source-only barrier fixture must reject the direct front-layer route");
  auto config = experiment_config();
  config.copper_scaffold.enabled = true;
  config.copper_scaffold.grid_pitch = Nanometres::from_mm(0.5);
  config.planner.quantized_copper_oracle.enabled = false;
  RoutingEngine engine(planning, source, config);
  for (std::size_t step = 0; step < 20'000U && !engine.terminal(); ++step)
    static_cast<void>(engine.advance({.work_units = 256U}));
  require(engine.selected_plan() && engine.selected_plan()->legal_scaffold_complete,
          "scaffold must find a physical detour around a source-only obstacle");
  const auto exact = probe_quantized_copper_feasibility(source, engine.selected_plan()->coarse_global);
  require(exact.feasible(), "scaffold certificate must use final physical source and mandatory oracle");
}
void rejected_scaffold_oracle_does_not_veto_an_independent_candidate() {
  const auto planning = two_pad_board();
  auto source = planning;
  auto extra = source.footprints.front();
  extra.id = ObjectId{"source-only-terminal"};
  extra.reference = "U3";
  extra.position = mm(50, 65);
  extra.pads.front().id = ObjectId{"source-only-pad"};
  // Keep an independent rejection case outside this routing domain. Represented-net
  // residual terminals are now explicitly repaired rather than intentionally left open.
  const NetId omitted{"source-only-net"};
  source.nets.push_back({omitted, "SOURCE_ONLY", 0U});
  extra.pads.front().net = omitted;
  auto second_extra = extra;
  second_extra.id = ObjectId{"source-only-terminal-2"};
  second_extra.reference = "U4";
  second_extra.position = mm(60, 65);
  second_extra.pads.front().id = ObjectId{"source-only-pad-2"};
  source.footprints.push_back(std::move(second_extra));
  source.footprints.push_back(std::move(extra));
  auto config = experiment_config();
  config.copper_scaffold.enabled = true;
  config.planner.quantized_copper_oracle.enabled = false;
  // The exact source has an additional open outside the routed net. Retain that scaffold rejection,
  // but let the independently planned candidate reach detailed source validation. Handoff is
  // not a claim that the extra source terminal is connected.
  RoutingEngine engine(planning, source, config);
  for (std::size_t step = 0; step < 20'000U && !engine.terminal(); ++step) {
    static_cast<void>(engine.advance({.work_units = 256U}));
    if (engine.state().scaffold_oracle && !engine.state().scaffold_oracle->feasible())
      require(std::none_of(engine.state().pending_diagnoses.begin(), engine.state().pending_diagnoses.end(),
                  [&](const auto& diagnosis) {
                    return std::find(engine.state().scaffold_oracle->diagnoses.begin(),
                        engine.state().scaffold_oracle->diagnoses.end(), diagnosis) !=
                        engine.state().scaffold_oracle->diagnoses.end();
                  }),
              "scaffold-only oracle witnesses must not enter independent candidate history before handoff");
  }
  require(engine.state().scaffold_oracle && !engine.state().scaffold_oracle->feasible() &&
              !engine.state().scaffold_oracle->diagnoses.empty(),
          "fixture must retain failed scaffold-oracle evidence");
  require(engine.state().outcome == RoutingEngineOutcome::awaiting_materialization &&
              engine.selected_plan() && !engine.selected_plan()->legal_scaffold_complete,
          "a rejected scaffold must not veto an independent regional materialization candidate");
  require(!engine.state().accepted_detailed,
          "plan-only handoff must not certify detailed copper without materialization");
  require(std::none_of(engine.state().attempts.begin(), engine.state().attempts.end(),
              [](const auto& attempt) {
                return attempt.action == RoutingEngineAction::construct_copper_scaffold &&
                    attempt.evidence_recorded;
              }),
          "retaining a failed scaffold certificate must not mark another working plan evaluated or roll it back");
}
void empty_scaffold_dispatches_missing_demand_to_regional_repair() {
  const auto planning = two_pad_board();
  auto source = planning;
  source.nets.push_back({NetId{"obstacle"}, "OBSTACLE", 1U});
  for (const auto& layer : {LayerId{"F.Cu"}, LayerId{"B.Cu"}})
    source.tracks.push_back({ObjectId{"locked-wall-" + layer.value()}, NetId{"obstacle"}, layer,
                             mm(50.0, 0.0), mm(50.0, 80.0), Nanometres::from_mm(1.0), true});

  auto config = experiment_config();
  config.copper_scaffold.enabled = true;
  config.copper_scaffold.maximum_expansions_per_connection = 0U;
  config.copper_scaffold.maximum_victim_exchanges = 0U;
  config.copper_scaffold.maximum_coupled_exchange_attempts = 0U;
  config.max_physical_region_requests = 1U;
  config.max_actions = 4U;
  RoutingEngine engine(planning, source, config);
  for (std::size_t step = 0U; step < 20'000U && !engine.terminal() &&
                               (engine.state().physical_repair_evidence.empty() ||
                                engine.state().physical_repair_evidence.back().regional_admitted_physical_requests == 0U);
       ++step)
    static_cast<void>(engine.advance({.work_units = 64U}));

  require(engine.state().legal_scaffold && !engine.state().legal_scaffold->complete &&
              engine.state().legal_scaffold->paths.empty() &&
              engine.state().legal_scaffold->missing_indices.size() == 1U,
          "a fully blocked scaffold must retain its missing demand even with no accepted paths");
  if (engine.state().physical_repair_evidence.empty())
    std::cerr << "zero-path regional actions=" << engine.state().scaffold_regional_actions << "; no evidence\n";
  else if (engine.state().physical_repair_evidence.back().regional_admitted_physical_requests != 1U)
    std::cerr << "zero-path regional actions=" << engine.state().scaffold_regional_actions
              << "; requests=" << engine.state().physical_repair_evidence.back().regional_physical_requests
              << "; admitted=" << engine.state().physical_repair_evidence.back().regional_admitted_physical_requests
              << "; windows=" << engine.state().physical_repair_evidence.back().regional_windows << '\n';
  require(engine.state().scaffold_regional_actions == 1U &&
              !engine.state().physical_repair_evidence.empty() &&
              engine.state().physical_repair_evidence.back().regional_physical_requests == 1U &&
              engine.state().physical_repair_evidence.back().regional_admitted_physical_requests == 1U,
          "a zero-path scaffold must still dispatch its failed demand into bounded regional repair");
}

void scaffold_replaces_all_routed_nets_when_scope_is_null() {
  const auto planning = two_pad_board();
  auto source = planning;
  source.tracks.push_back({ObjectId{"source-track"}, NetId{"engine-experiment-net"},
                           LayerId{"F.Cu"}, mm(20, 40), mm(80, 40),
                           Nanometres::from_mm(0.25), false});
  auto config = experiment_config();
  config.copper_scaffold.enabled = true;
  config.detailed.existing_routing = ExistingRoutingPolicy::replace_unlocked_for_routed_nets;
  // A null scope is the direct-routing policy: replace every net in the coarse global plan.
  config.detailed.existing_routing_replacement_scope.reset();
  RoutingEngine engine(planning, source, config);
  for (std::size_t step = 0; step < 20'000U && !engine.terminal(); ++step)
    static_cast<void>(engine.advance({.work_units = 256U}));
  require(engine.state().legal_scaffold && engine.state().legal_scaffold->complete &&
              !engine.state().legal_scaffold->paths.empty(),
          "null replacement scope must not let source copper complete the scaffold as a no-op");
}
void bounded_scaffold_hands_failed_requests_to_regional_before_sweep_completion() {
  auto planning = two_pad_board();
  for (int n = 0; n < 2; ++n) {
    const NetId net{"deferred-" + std::to_string(n)};
    planning.nets.push_back({net, "DEFERRED" + std::to_string(n), n + 1});
    for (int i = 0; i < 2; ++i) {
      Footprint footprint;
      footprint.id = ObjectId{"D" + std::to_string(n) + "-" + std::to_string(i)};
      footprint.reference = footprint.id.value();
      footprint.position = mm(10 + i * 5, 10 + n * 10);
      footprint.courtyard = Polygon{{mm(-1, -1), mm(1, -1), mm(1, 1), mm(-1, 1)}};
      footprint.pads = {{ObjectId{footprint.id.value() + "-pad"}, net, Point{}, {LayerId{"F.Cu"}}}};
      planning.footprints.push_back(footprint);
    }
  }
  auto source = planning;
  source.nets.push_back({NetId{"obstacle"}, "OBSTACLE", 3});
  // This wall exists only in the physical source. Global/regional planning therefore retains
  // the coarse straight crossing, while exact realization has no legal repair to commit.
  Keepout source_only_wall;
  source_only_wall.id = ObjectId{"source-only-regional-wall"};
  source_only_wall.area = Polygon{{mm(49, 0), mm(51, 0), mm(51, 80), mm(49, 80)}};
  source_only_wall.layers = {LayerId{"F.Cu"}, LayerId{"B.Cu"}};
  source_only_wall.prohibit = {true, true, false, false, false};
  source.keepouts.push_back(std::move(source_only_wall));
  for (const auto& layer : {LayerId{"F.Cu"}, LayerId{"B.Cu"}})
    source.tracks.push_back({ObjectId{"near-wall-" + layer.value()}, NetId{"obstacle"}, layer,
        mm(21.5, 38), mm(21.5, 42), Nanometres::from_mm(0.5), true});
  auto config = experiment_config();
  config.copper_scaffold.enabled = true;
  config.planner.global.priority_nets = {NetId{"engine-experiment-net"}};
  config.copper_scaffold.maximum_expansions_per_connection = 128U;
  config.copper_scaffold.maximum_victim_exchanges = 0U;
  config.copper_scaffold.maximum_coupled_exchange_attempts = 0U;
  config.bounded_scaffold_request_threshold = 0U;
  config.scaffold_requests_per_decision = 1U;
  config.scaffold_failed_requests_per_regional_handoff = 1U;
  config.max_physical_region_requests = 1U;
  config.initial_regional_work_units = 1U;
  config.max_regional_work_units = 1U;
  config.max_regional_decisions_without_improvement = 1U;
  CopperSnapshotSink sink;
  RoutingEngine engine(planning, source, config, &sink);
  for (std::size_t step = 0; step < 20'000U && !engine.terminal() &&
       engine.state().scaffold_repair_handoffs == 0U; ++step)
    static_cast<void>(engine.advance({.work_units = 128U}));
  require(engine.state().scaffold_repair_handoffs == 1U && engine.state().legal_scaffold &&
              engine.state().legal_scaffold->sweep_in_progress,
          "a failed request must park the sweep and open regional repair before deferred work completes");
  require(std::none_of(engine.state().attempts.begin(), engine.state().attempts.end(), [](const auto& attempt) {
            return attempt.action == RoutingEngineAction::construct_copper_scaffold;
          }), "early regional handoff must not fabricate a completed scaffold action");
  require(engine.state().legal_scaffold->failed_request_indices.size() == 1U &&
              engine.state().legal_scaffold->missing_indices.size() > 1U &&
              check_copper_rules(engine.state().legal_scaffold->copper).ok(),
          "only the attempted failure is admitted; deferred demands and source legality are preserved");
  for (std::size_t step = 0; step < 20'000U && !engine.terminal() &&
       engine.state().scaffold_regional_actions == 0U; ++step)
    static_cast<void>(engine.advance({.work_units = 128U}));
  require(engine.state().scaffold_regional_actions == 1U &&
              engine.state().scaffold_regional_missing_requests == 1U,
          "the next bounded action must receive exactly the failed request and its actual copper context");
  for (std::size_t step = 0; step < 20'000U && !engine.terminal() &&
       engine.state().legal_scaffold && engine.state().legal_scaffold->sweep_in_progress; ++step)
    static_cast<void>(engine.advance({.work_units = 128U}));
  require(engine.state().experiment_resumed > 0U && engine.state().legal_scaffold &&
              !engine.state().legal_scaffold->sweep_in_progress &&
              std::all_of(engine.state().legal_scaffold->requests.begin(),
                          engine.state().legal_scaffold->requests.end(),
                          [&](const auto& request) {
                            if (!request.net.value().starts_with("deferred-")) return true;
                            return std::find_if(engine.state().legal_scaffold->paths.begin(),
                                engine.state().legal_scaffold->paths.end(), [&](const auto& path) {
                                  return path.net == request.net &&
                                      path.connection_index == request.connection_index;
                                }) != engine.state().legal_scaffold->paths.end();
                          }) && check_copper_rules(engine.state().legal_scaffold->copper).ok(),
          "a no-op regional repair must resume the parked scaffold and satisfy deferred easy requests");
}

void selected_physical_repair_survives_cancellation_before_exact_admission() {
  const auto planning = two_pad_board();
  auto source = planning;
  source.nets.push_back({NetId{"obstacle"}, "OBSTACLE", 1U});
  for (const auto& layer : {LayerId{"F.Cu"}, LayerId{"B.Cu"}})
    source.tracks.push_back({ObjectId{"cancel-wall-" + layer.value()}, NetId{"obstacle"}, layer,
                             mm(50.0, 0.0), mm(50.0, 80.0), Nanometres::from_mm(1.0), true});
  auto config = experiment_config();
  config.copper_scaffold.enabled = true;
  config.copper_scaffold.maximum_expansions_per_connection = 0U;
  config.copper_scaffold.maximum_victim_exchanges = 0U;
  config.copper_scaffold.maximum_coupled_exchange_attempts = 0U;
  config.bounded_scaffold_request_threshold = 0U;
  config.scaffold_requests_per_decision = 1U;
  config.scaffold_failed_requests_per_regional_handoff = 1U;
  config.max_physical_region_requests = 1U;
  RoutingEngine engine(planning, source, config);
  for (std::size_t step = 0U; step < 20'000U && !engine.terminal() &&
       engine.state().scaffold_repair_handoffs == 0U; ++step)
    static_cast<void>(engine.advance({.work_units = 64U}));
  require(engine.state().scaffold_repair_handoffs == 1U &&
              engine.state().scaffold_newly_failed_requests.size() == 1U &&
              engine.state().physical_repair_admitted_attempts == 0U,
          "selecting a physical repair must retain its queue entry until exact admission");
  const auto queued = engine.state().scaffold_newly_failed_requests;
  std::stop_source stop;
  stop.request_stop();
  static_cast<void>(engine.advance({}, stop.get_token()));
  require(engine.state().scaffold_newly_failed_requests == queued &&
              engine.state().physical_repair_admitted_attempts == 0U,
          "cancelling selected regional preparation must preserve fair retry eligibility");
}

void large_bounded_scaffold_batches_failures_without_losing_success_queue_context() {
  auto planning = two_pad_board();
  const auto append = [&](std::string name, Point first, Point second, std::uint32_t ordinal) {
    const NetId net{std::move(name)};
    planning.nets.push_back({net, net.value(), ordinal});
    for (const auto [index, point] : {std::pair{0U, first}, std::pair{1U, second}}) {
      Footprint footprint;
      footprint.id = ObjectId{"B" + std::to_string(ordinal) + "-" + std::to_string(index)};
      footprint.reference = footprint.id.value();
      footprint.position = point;
      footprint.courtyard = Polygon{{mm(-1, -1), mm(1, -1), mm(1, 1), mm(-1, 1)}};
      footprint.pads = {{ObjectId{footprint.id.value() + "-pad"}, net, Point{}, {LayerId{"F.Cu"}}}};
      planning.footprints.push_back(std::move(footprint));
    }
    return net;
  };
  const auto hard0 = append("batched-hard-0", mm(20, 10), mm(80, 10), 1U);
  const auto hard1 = append("batched-hard-1", mm(20, 20), mm(80, 20), 2U);
  const auto hard2 = append("batched-hard-2", mm(20, 30), mm(80, 30), 3U);
  const auto easy = append("batched-easy", mm(10, 70), mm(30, 70), 4U);
  const auto hard3 = append("batched-hard-3", mm(20, 50), mm(80, 50), 5U);
  auto source = planning;
  source.nets.push_back({NetId{"batched-wall"}, "WALL", 6U});
  for (const auto& layer : {LayerId{"F.Cu"}, LayerId{"B.Cu"}})
    source.tracks.push_back({ObjectId{"batched-wall-" + layer.value()}, NetId{"batched-wall"}, layer,
                             mm(50, 0), mm(50, 80), Nanometres::from_mm(1.0), true});
  auto config = experiment_config();
  config.copper_scaffold.enabled = true;
  config.planner.global.priority_nets = {NetId{"engine-experiment-net"}, hard0, hard1, hard2, easy, hard3};
  config.copper_scaffold.maximum_expansions_per_connection = 0U;
  config.copper_scaffold.maximum_victim_exchanges = 0U;
  config.copper_scaffold.maximum_coupled_exchange_attempts = 0U;
  config.bounded_scaffold_request_threshold = 5U;
  config.scaffold_requests_per_decision = 1U;
  config.scaffold_failed_requests_per_regional_handoff = 4U;
  config.max_physical_region_requests = 6U;
  config.max_actions_without_physical_progress = 2U;
  RoutingEngine engine(planning, source, config);
  bool success_preserved_queue = false;
  for (std::size_t step = 0U; step < 100'000U && !engine.terminal() &&
       engine.state().scaffold_repair_handoffs == 0U; ++step) {
    static_cast<void>(engine.advance({.work_units = 64U}));
    success_preserved_queue = success_preserved_queue ||
        (!engine.state().scaffold_newly_failed_requests.empty() && engine.state().legal_scaffold &&
         !engine.state().legal_scaffold->satisfied_indices.empty() &&
         engine.state().scaffold_repair_handoffs == 0U);
  }
  const auto& queued = engine.state().scaffold_newly_failed_requests;
  require(success_preserved_queue && engine.state().scaffold_repair_handoffs == 1U &&
              queued.size() == 4U && std::set<std::size_t>(queued.begin(), queued.end()).size() == 4U &&
              engine.state().scaffold_regional_missing_requests == 0U,
          "a large bounded sweep must preserve failures across an ordinary success and hand off four distinct requests");
  require(queued.size() <= config.max_physical_region_requests,
          "the queued causal handoff must remain below the configured physical-region cap");
}

void forced_scaffold_eviction_does_not_preempt_unfinished_physical_regional_work() {
  auto planning = two_pad_board();
  for (int n = 0; n < 2; ++n) {
    const NetId net{"deferred-" + std::to_string(n)};
    planning.nets.push_back({net, "DEFERRED" + std::to_string(n), static_cast<std::uint32_t>(n + 1)});
    for (int i = 0; i < 2; ++i) {
      Footprint footprint;
      footprint.id = ObjectId{"D" + std::to_string(n) + "-" + std::to_string(i)};
      footprint.reference = footprint.id.value();
      footprint.position = mm(10 + i * 5, 10 + n * 10);
      footprint.courtyard = Polygon{{mm(-1, -1), mm(1, -1), mm(1, 1), mm(-1, 1)}};
      footprint.pads = {{ObjectId{footprint.id.value() + "-pad"}, net, Point{}, {LayerId{"F.Cu"}}}};
      planning.footprints.push_back(std::move(footprint));
    }
  }
  auto source = planning;
  source.nets.push_back({NetId{"forced-eviction-obstacle"}, "OBSTACLE", 2U});
  Keepout wall;
  wall.id = ObjectId{"forced-eviction-source-wall"};
  wall.area = Polygon{{mm(49, 0), mm(51, 0), mm(51, 80), mm(49, 80)}};
  wall.layers = {LayerId{"F.Cu"}, LayerId{"B.Cu"}};
  wall.prohibit = {true, true, false, false, false};
  source.keepouts.push_back(std::move(wall));
  for (const auto& layer : {LayerId{"F.Cu"}, LayerId{"B.Cu"}})
    source.tracks.push_back({ObjectId{"forced-eviction-near-wall-" + layer.value()},
        NetId{"forced-eviction-obstacle"}, layer, mm(21.5, 38), mm(21.5, 42),
        Nanometres::from_mm(0.5), true});

  auto config = experiment_config();
  config.copper_scaffold.enabled = true;
  config.planner.global.priority_nets = {NetId{"engine-experiment-net"}};
  config.copper_scaffold.maximum_expansions_per_connection = 128U;
  config.copper_scaffold.maximum_victim_exchanges = 0U;
  config.copper_scaffold.maximum_coupled_exchange_attempts = 0U;
  config.bounded_scaffold_request_threshold = 0U;
  config.scaffold_requests_per_decision = 1U;
  config.scaffold_failed_requests_per_regional_handoff = 1U;
  config.max_physical_region_requests = 1U;
  config.max_actions_without_physical_progress = 2U;
  config.experiment_quantum = std::chrono::milliseconds{1U};
  config.max_parked_experiments = 0U;
  config.max_parked_experiment_bytes = 0U;
  config.initial_regional_work_units = 1'000'000U;
  config.max_regional_work_units = 1'000'000U;
  config.max_regional_decisions_without_improvement = 32U;
  CopperSnapshotSink sink;
  RoutingEngine engine(planning, source, config, &sink);
  const RoutingEngineStepBudget quantum{.work_units = 1U, .time_slice = std::chrono::milliseconds{1U}};
  for (std::size_t step = 0U; step < 100'000U && !engine.terminal() &&
       engine.state().scaffold_repair_handoffs == 0U; ++step)
    static_cast<void>(engine.advance(quantum));
  require(engine.state().scaffold_repair_handoffs == 1U && sink.scaffold_begins == 1U &&
              engine.state().experiment_memory_evictions > 0U,
          "the forced parked-cap fixture must evict the scaffold at its physical-repair handoff");

  bool regional_yielded_before_finish = false;
  std::optional<std::size_t> scaffold_begins_at_regional_finish;
  for (std::size_t step = 0U; step < 100'000U && !engine.terminal() &&
       !scaffold_begins_at_regional_finish; ++step) {
    const auto result = engine.advance(quantum);
    if (sink.regional_begins > 0U && sink.regional_ends == 0U &&
        result.action == RoutingEngineAction::regional_realization &&
        result.outcome == RoutingEngineOutcome::yielded) {
      regional_yielded_before_finish = true;
      require(sink.scaffold_begins == 1U,
              "an unfinished physical regional continuation must run before a fresh scaffold begins");
    }
    if (sink.regional_ends > 0U) scaffold_begins_at_regional_finish = sink.scaffold_begins;
  }
  require(regional_yielded_before_finish && scaffold_begins_at_regional_finish &&
              *scaffold_begins_at_regional_finish == 1U,
          "forced eviction must preserve unfinished physical regional work through bounded yield and finish");
  for (std::size_t step = 0U; step < 100'000U && !engine.terminal() &&
       engine.state().legal_scaffold && engine.state().legal_scaffold->sweep_in_progress; ++step)
    static_cast<void>(engine.advance(quantum));
  require(engine.state().legal_scaffold && !engine.state().legal_scaffold->sweep_in_progress &&
              std::all_of(engine.state().legal_scaffold->requests.begin(),
                          engine.state().legal_scaffold->requests.end(), [&](const auto& request) {
                            if (!request.net.value().starts_with("deferred-")) return true;
                            return std::find_if(engine.state().legal_scaffold->paths.begin(),
                                engine.state().legal_scaffold->paths.end(), [&](const auto& path) {
                                  return path.net == request.net &&
                                      path.connection_index == request.connection_index;
                                }) != engine.state().legal_scaffold->paths.end();
                          }),
          "after the evaluated no-op repair, a reconstructed scaffold must reach deferred requests");
  require(engine.state().checkpoint_progress_events > 0U,
          "the later strict checked coverage gain must reset scaffold liveness after a failed prefix");
}

void changed_copper_invalidates_a_scaffold_continuation() {
  CopperScaffoldResult retained;
  retained.copper = two_pad_board();
  auto unchanged = retained;
  unchanged.diagnostic = "new negative evidence";
  unchanged.request_evidence.push_back(CopperScaffoldRequestEvidence{});
  require(routing_scaffold_continuation_compatible(&retained, &unchanged),
          "evidence-only updates must preserve an unchanged private sweep and its preparation");
  auto repaired = retained;
  repaired.copper.tracks.push_back({ObjectId{"new-repair"}, NetId{"engine-experiment-net"},
      LayerId{"F.Cu"}, mm(20, 40), mm(30, 40), Nanometres::from_mm(0.25)});
  require(check_copper_rules(repaired.copper).ok() &&
              !routing_scaffold_continuation_compatible(&retained, &repaired),
          "a legal copper repair must invalidate the old continuation even when path metadata is unchanged");
  require(routing_scaffold_copper_scene_records(retained.copper) !=
              routing_scaffold_copper_scene_records(repaired.copper),
          "changed copper with unchanged request/path metadata must reopen scaffold retry identity");
  auto scene = repaired.copper;
  scene.track_arcs.push_back({ObjectId{"arc"}, NetId{"engine-experiment-net"}, LayerId{"F.Cu"},
      mm(30, 40), mm(35, 41), mm(40, 40), Nanometres::from_mm(0.25)});
  scene.track_beziers.push_back({ObjectId{"curve"}, NetId{"engine-experiment-net"}, LayerId{"F.Cu"},
      mm(40, 40), mm(45, 41), mm(50, 41), mm(55, 40), Nanometres::from_mm(0.25)});
  scene.vias.push_back({ObjectId{"via"}, NetId{"engine-experiment-net"}, mm(55, 40),
      Nanometres::from_mm(0.6), Nanometres::from_mm(0.3), {LayerId{"F.Cu"}, LayerId{"B.Cu"}}});
  const auto records = routing_scaffold_copper_scene_records(scene);
  auto reordered_scene = scene;
  std::reverse(reordered_scene.tracks.begin(), reordered_scene.tracks.end());
  std::reverse(reordered_scene.vias.front().layers.begin(), reordered_scene.vias.front().layers.end());
  require(records == routing_scaffold_copper_scene_records(reordered_scene),
          "container and via layer traversal order must preserve canonical copper identity");
  const auto changed = [&](auto edit) {
    auto copy = scene;
    edit(copy);
    require(records != routing_scaffold_copper_scene_records(copy),
            "every routing copper primitive must participate in scaffold retry identity");
  };
  changed([](auto& b) { b.tracks.front().end = mm(31, 40); });
  changed([](auto& b) { b.track_arcs.front().middle = mm(35, 42); });
  changed([](auto& b) { b.track_beziers.front().control1 = mm(45, 42); });
  changed([](auto& b) { b.vias.front().drill = Nanometres::from_mm(0.35); });
  changed([](auto& b) { b.tracks.erase(b.tracks.begin()); });
  auto covered = retained;
  covered.satisfied_indices.push_back(0U);
  require(!routing_scaffold_continuation_compatible(&retained, &covered),
          "changed request coverage must not resume obsolete private bookkeeping");
  auto ordered = retained;
  ordered.requests.resize(2U);
  ordered.requests[0].net = NetId{"a"};
  ordered.requests[1].net = NetId{"b"};
  ordered.request_identities = {"first", "second"};
  auto reordered = ordered;
  std::swap(reordered.requests[0], reordered.requests[1]);
  std::swap(reordered.request_identities[0], reordered.request_identities[1]);
  require(!routing_scaffold_continuation_compatible(&ordered, &reordered),
          "reordered request identities must invalidate index-based private sweep state");

  const GlobalRoutePath request{NetId{"engine-experiment-net"}, 0U,
      {{mm(20, 40), LayerId{"F.Cu"}}, {mm(80, 40), LayerId{"F.Cu"}}}};
  const auto base_scene = two_pad_board();
  auto parked_negative = construct_copper_scaffold(base_scene, {request}, CopperScaffoldConfig{});
  parked_negative.copper = base_scene;
  parked_negative.paths.clear();
  parked_negative.satisfied_indices.clear();
  parked_negative.missing_indices = {0U};
  parked_negative.complete = false;
  parked_negative.sweep_in_progress = true;
  parked_negative.attempted_request_indices = {0U};
  parked_negative.failed_request_indices = {0U};
  parked_negative.request_evidence.front().termination = CopperScaffoldSearchTermination::search_exhausted;
  auto changed_scene = base_scene;
  changed_scene.tracks.push_back({ObjectId{"changed-resume-scene"}, NetId{"engine-experiment-net"},
      LayerId{"F.Cu"}, mm(10, 10), mm(15, 10), Nanometres::from_mm(0.25), true});
  CopperScaffoldConfig resume_config;
  resume_config.resume_unattempted_requests = true;
  const auto fresh_probe = construct_copper_scaffold(
      changed_scene, {request}, resume_config, {}, &parked_negative);
  require(fresh_probe.request_evidence.front().termination != CopperScaffoldSearchTermination::search_exhausted &&
              fresh_probe.request_evidence.front().termination != CopperScaffoldSearchTermination::not_attempted,
          "changed copper must disable retained negative-ledger replay and probe the request again");
}

void partial_scaffold_routes_missing_requests_against_retained_copper() {
  auto planning = two_pad_board();
  const NetId easy{"easy-net"};
  planning.nets.push_back({easy, "EASY", 1U});
  for (int i = 0; i < 2; ++i) {
    Footprint footprint;
    footprint.id = ObjectId{"E" + std::to_string(i)};
    footprint.reference = footprint.id.value();
    footprint.position = mm(10 + i * 10, 10);
    footprint.courtyard = Polygon{{mm(-1, -1), mm(1, -1), mm(1, 1), mm(-1, 1)}};
    footprint.pads = {{ObjectId{footprint.id.value() + "-pad"}, easy, Point{}, {LayerId{"F.Cu"}}}};
    planning.footprints.push_back(footprint);
  }
  auto source = planning;
  source.nets.push_back({NetId{"obstacle"}, "OBSTACLE", 2U});
  source.tracks.push_back({ObjectId{"physical-only"}, NetId{"obstacle"}, LayerId{"F.Cu"},
                          mm(21.5, 38), mm(21.5, 42), Nanometres::from_mm(0.5), true});
  source.tracks.push_back({ObjectId{"physical-only-back"}, NetId{"obstacle"}, LayerId{"B.Cu"},
                          mm(21.5, 38), mm(21.5, 42), Nanometres::from_mm(0.5), true});
  // The global planner may orient the demand either way. Put a legal obstacle near
  // each endpoint so a bounded frontier encounters one before exhausting its allowance.
  source.tracks.push_back({ObjectId{"physical-only-target"}, NetId{"obstacle"}, LayerId{"F.Cu"},
                          mm(78.5, 38), mm(78.5, 42), Nanometres::from_mm(0.5), true});
  source.tracks.push_back({ObjectId{"physical-only-target-back"}, NetId{"obstacle"}, LayerId{"B.Cu"},
                          mm(78.5, 38), mm(78.5, 42), Nanometres::from_mm(0.5), true});
  auto config = experiment_config();
  config.copper_scaffold.enabled = true;
  // Reach the nearby physical blocker, but stop well before a full-board detour.
  config.copper_scaffold.maximum_expansions_per_connection = 128U;
  config.copper_scaffold.maximum_victim_exchanges = 0U;
  config.copper_scaffold.maximum_coupled_exchange_attempts = 0U;
  config.max_actions = 4U;
  CopperSnapshotSink sink;
  RoutingEngine engine(planning, source, config, &sink);
  for (std::size_t step = 0; step < 20'000U && !engine.terminal() &&
                               (engine.state().physical_repair_evidence.empty() ||
                                engine.state().physical_repair_evidence.back().regional_bundle_searches == 0U);
       ++step)
    static_cast<void>(engine.advance({.work_units = 64U}));
  require(engine.state().legal_scaffold && !engine.state().legal_scaffold->complete &&
              engine.state().legal_scaffold->request_count == 2U &&
              engine.state().legal_scaffold->satisfied_indices.size() == 1U,
          "bounded initial search must retain the easy request and leave the blocked one unresolved");
  require(engine.state().scaffold_regional_actions == 1U &&
              engine.state().scaffold_regional_missing_requests == 1U,
          "regional repair must consume only the missing request against checked scaffold copper");
  const auto missing = engine.state().legal_scaffold->missing_indices.front();
  const auto& blocker_evidence = engine.state().legal_scaffold->request_evidence.at(missing);
  if (blocker_evidence.blocker_objects.empty()) {
    std::cerr << "fixture missing=" << missing << " termination="
              << to_string(blocker_evidence.termination) << " expansions="
              << engine.state().legal_scaffold->counters.expansions << " endpoints=";
    for (const auto& point : engine.state().legal_scaffold->requests.at(missing).points)
      std::cerr << point.position.x.value() << ',' << point.position.y.value() << ':' << point.layer.value() << ';';
    std::cerr << '\n';
  }

  require(std::any_of(blocker_evidence.blocker_objects.begin(), blocker_evidence.blocker_objects.end(),
                      [](const auto& id) { return id.value().starts_with("physical-only"); }) &&
              std::find(blocker_evidence.blocker_nets.begin(), blocker_evidence.blocker_nets.end(),
                        NetId{"obstacle"}) != blocker_evidence.blocker_nets.end(),
          "failed scaffold evidence must retain the exact blocking object and owner net");
  require(!engine.state().physical_repair_evidence.empty() &&
              engine.state().physical_repair_evidence.back().regional_fragments >= 1U &&
              engine.state().physical_repair_evidence.back().regional_windows != 0U &&
              engine.state().physical_repair_evidence.back().regional_bundle_searches != 0U &&
              engine.state().physical_repair_evidence.back().regional_physical_requests == 1U &&
              engine.state().physical_repair_evidence.back().regional_admitted_physical_requests == 1U &&
              engine.state().physical_repair_evidence.back().regional_blocker_objects != 0U,
          "a failed scaffold demand must form a bounded regional window from its own fragments "
          "and immutable attributed blocker geometry without a selected-global conflict edge");
  require(!engine.state().live_copper_terrain.provisional &&
              engine.state().working_plan && !engine.state().working_plan->legal_scaffold_complete,
          "partial physical terrain must not become a whole-board completion certificate");
  require(std::any_of(engine.state().legal_scaffold->copper.tracks.begin(),
                     engine.state().legal_scaffold->copper.tracks.end(),
                     [](const auto& track) { return track.id == ObjectId{"physical-only"}; }),
          "partial repair scene must retain physical-source obstacles");
  require(sink.publications > 0U && sink.latest &&
              sink.latest->tracks == engine.state().legal_scaffold->copper.tracks &&
              sink.latest->vias == engine.state().legal_scaffold->copper.vias &&
              !engine.state().accepted_detailed,
          "viewer must receive retained partial copper before whole-board completion");
}

void residual_physical_demands_preserve_checked_components() {
  auto board = two_pad_board();
  const NetId net{"engine-experiment-net"};
  const LayerId front{"F.Cu"};
  for (std::size_t i = 0; i < 2U; ++i) {
    auto pad = board.footprints[i];
    pad.id = ObjectId{"extra-" + std::to_string(i)};
    pad.pads.front().id = ObjectId{"extra-pad-" + std::to_string(i)};
    pad.position = mm(i == 0U ? 20 : 80, 60);
    board.footprints.push_back(std::move(pad));
  }
  board.tracks = {
      {ObjectId{"left-tree"}, net, front, mm(20,40), mm(20,60), Nanometres::from_mm(0.2), false},
      {ObjectId{"right-tree"}, net, front, mm(80,40), mm(80,60), Nanometres::from_mm(0.2), false}};
  CopperScaffoldResult retained;
  retained.copper = board;
  retained.requests = {{net, 3U, {{mm(20,40),front},{mm(20,60),front}}},
                       {net, 8U, {{mm(80,40),front},{mm(80,60),front}}}};
  retained.request_count = 2U;
  retained.satisfied_indices = {0U, 1U};
  retained.complete = true;
  require(append_residual_physical_demands(retained, 0U) == retained.requests,
          "exhausted pair budget must preserve the domain without partially adding a net");
  const auto requests = append_residual_physical_demands(retained);
  require(requests.size() == 3U && requests[0] == retained.requests[0] &&
              requests[1] == retained.requests[1] && requests.back().connection_index > 8U,
          "residual component demand must append a fresh identity and preserve the checked prefix");
  require(!scaffold_request_connected(board, requests.back()),
          "all pads being covered by endpoints must not hide disconnected physical trees");
  const auto result = construct_copper_scaffold(board, requests, CopperScaffoldConfig{});
  require(result.complete && analyze_connectivity(result.copper).open_connections == 0U &&
              scaffold_preserves_satisfied_requests(result, retained),
          "routing the appended demand must close real connectivity while retaining checked peers");
  require(append_residual_physical_demands(result) == result.requests,
          "a physically connected checkpoint must not grow another suffix");
  retained.requests = requests;
  retained.complete = false;
  retained.missing_indices = {2U};
  require(append_residual_physical_demands(retained) == requests,
          "pending residual obligations must not be duplicated");

  auto layered = two_pad_board();
  layered.footprints[1].position = layered.footprints[0].position;
  layered.footprints[1].pads.front().layers = {LayerId{"B.Cu"}};
  retained = CopperScaffoldResult{};
  retained.copper = layered;
  retained.complete = true;
  retained.requests = {{net, 0U, {{mm(20,40),front},{mm(20,40),front}}}};
  const auto vertical = append_residual_physical_demands(retained);
  require(vertical.size() == 2U && vertical.back().points.front().layer != vertical.back().points.back().layer,
          "coincident pads on different layers without a via must remain separate components");
  retained.copper.footprints[0].pads.front().layers = {front, LayerId{"B.Cu"}};
  retained.copper.footprints[1].pads.front().layers = {front};
  retained.copper.footprints[1].position = mm(80,40);
  const auto common = append_residual_physical_demands(retained);
  require(common.size() == 2U && common.back().points.front().layer == front &&
              common.back().points.back().layer == front,
          "a common pad layer must avoid inventing an unnecessary via obligation");
}

void residual_demand_pair_budget_is_atomic() {
  auto board = two_pad_board();
  const auto original = board.footprints.front();
  board.footprints.clear();
  constexpr std::size_t count = 32U;
  for (std::size_t i = 0U; i < count; ++i) {
    auto item = original;
    item.id = ObjectId{"budget-" + std::to_string(i)};
    item.pads.front().id = ObjectId{"budget-pad-" + std::to_string(i)};
    item.position = mm(5 + 2 * static_cast<double>(i), 20);
    board.footprints.push_back(std::move(item));
  }
  CopperScaffoldResult retained;
  retained.copper = board;
  retained.complete = true;
  retained.requests = {{NetId{"engine-experiment-net"}, 0U,
      {{mm(5,20),LayerId{"F.Cu"}},{mm(5,20),LayerId{"F.Cu"}}}}};
  constexpr auto pairs = count * (count - 1U) / 2U;
  require(append_residual_physical_demands(retained, pairs - 1U) == retained.requests,
          "a late pair-budget exhaustion must roll back the entire proposed net suffix");
  require(append_residual_physical_demands(retained, pairs).size() == count,
          "frontier Prim must inspect each component pair only once within its exact allowance");
}

void partial_residual_physical_demands_are_net_scoped() {
  auto board = two_pad_board();
  const NetId completed{"engine-experiment-net"};
  const NetId pending{"unrelated-pending-net"};
  const LayerId front{"F.Cu"};
  board.nets.push_back({pending, "PENDING", 1U});
  for (std::size_t i = 0; i < 2U; ++i) {
    auto pad = board.footprints[i];
    pad.id = ObjectId{"partial-extra-" + std::to_string(i)};
    pad.pads.front().id = ObjectId{"partial-extra-pad-" + std::to_string(i)};
    pad.position = mm(i == 0U ? 20 : 80, 60);
    board.footprints.push_back(std::move(pad));
  }
  for (std::size_t i = 0; i < 2U; ++i) {
    auto pad = board.footprints[i];
    pad.id = ObjectId{"partial-pending-" + std::to_string(i)};
    pad.pads.front().id = ObjectId{"partial-pending-pad-" + std::to_string(i)};
    pad.pads.front().net = pending;
    pad.position = mm(i == 0U ? 20 : 80, 20);
    board.footprints.push_back(std::move(pad));
  }
  board.tracks = {
      {ObjectId{"partial-left-tree"}, completed, front, mm(20,40), mm(20,60),
       Nanometres::from_mm(0.2), false},
      {ObjectId{"partial-right-tree"}, completed, front, mm(80,40), mm(80,60),
       Nanometres::from_mm(0.2), false}};

  CopperScaffoldResult retained;
  retained.copper = board;
  retained.requests = {
      {completed, 3U, {{mm(20,40),front},{mm(20,60),front}}},
      {completed, 8U, {{mm(80,40),front},{mm(80,60),front}}},
      {pending, 0U, {{mm(20,20),front},{mm(80,20),front}}}};
  retained.request_count = retained.requests.size();
  retained.satisfied_indices = {0U, 1U};
  retained.missing_indices = {2U};

  auto incomplete_status = retained;
  incomplete_status.complete = true;
  incomplete_status.satisfied_indices = {0U};
  incomplete_status.missing_indices.clear();
  require(append_residual_physical_demands(incomplete_status) == incomplete_status.requests,
          "a complete flag must not bypass incomplete explicit request classification");

  const auto appended = append_residual_physical_demands(retained);
  require(appended.size() == 4U && appended[0] == retained.requests[0] &&
              appended[1] == retained.requests[1] && appended[2] == retained.requests[2] &&
              appended.back().net == completed && appended.back().connection_index > 8U,
          "a partial checkpoint must append only the completed net's disconnected component gap");

  retained.requests = appended;
  retained.request_count = retained.requests.size();
  retained.missing_indices = {2U, 3U};
  require(append_residual_physical_demands(retained) == appended,
          "a pending residual demand for the completed net must prevent a duplicate suffix");
}

void pad_preservation_demands_restore_only_regressed_components() {
  const NetId net{"engine-experiment-net"};
  const LayerId front{"F.Cu"};
  auto incumbent = two_pad_board();
  for (std::size_t i = 0U; i < 2U; ++i) {
    auto pad = incumbent.footprints.front();
    pad.id = ObjectId{"unrelated-" + std::to_string(i)};
    pad.pads.front().id = ObjectId{"unrelated-pad-" + std::to_string(i)};
    pad.position = mm(35.0 + 30.0 * static_cast<double>(i), 60.0);
    incumbent.footprints.push_back(std::move(pad));
  }
  incumbent.tracks = {{ObjectId{"incumbent-link"}, net, front, mm(20,40), mm(80,40),
                       Nanometres::from_mm(0.2), false}};
  auto candidate = incumbent;
  candidate.tracks.clear();
  const std::vector<GlobalRoutePath> existing = {
      {net, 3U, {{mm(20,40),front},{mm(80,40),front}}},
      {net, 8U, {{mm(20,40),front},{mm(80,40),front}}}};

  const auto demands = derive_pad_preservation_demands(incumbent, candidate, existing);
  require(demands && demands->size() == 1U && demands->front().net == net &&
              demands->front().connection_index == 9U &&
              demands->front().points == std::vector<GlobalRoutePoint>{{mm(20,40),front}, {mm(80,40),front}},
          "pad preservation must restore exactly the incumbent component that candidate split");
  require(derive_pad_preservation_demands(incumbent, candidate, existing) == demands,
          "pad preservation demands must be deterministic including fresh connection identities");

  auto restored = candidate;
  restored.tracks = {{ObjectId{"restored-link"}, net, front, demands->front().points.front().position,
                      demands->front().points.back().position, Nanometres::from_mm(0.2), false}};
  const auto restored_report = analyze_connectivity(restored);
  require(restored_report.open_connections == 2U,
          "restoring a split group must leave unrelated incumbent-open pads untouched");
}

void pad_preservation_demand_bounds_are_all_or_none() {
  const NetId net{"engine-experiment-net"};
  const LayerId front{"F.Cu"};
  auto incumbent = two_pad_board();
  auto third = incumbent.footprints.front();
  third.id = ObjectId{"third"};
  third.pads.front().id = ObjectId{"third-pad"};
  third.position = mm(50, 40);
  incumbent.footprints.push_back(std::move(third));
  incumbent.tracks = {{ObjectId{"left-link"}, net, front, mm(20,40), mm(50,40), Nanometres::from_mm(0.2), false},
                      {ObjectId{"right-link"}, net, front, mm(50,40), mm(80,40), Nanometres::from_mm(0.2), false}};
  auto candidate = incumbent;
  candidate.tracks.clear();
  const std::vector<GlobalRoutePath> existing = {{net, 4U, {{mm(20,40),front},{mm(80,40),front}}}};
  require(!derive_pad_preservation_demands(incumbent, candidate, existing, 1U),
          "a demand limit that cannot restore every split partition must reject the whole suffix");
  require(!derive_pad_preservation_demands(incumbent, candidate, existing, 2U, 2U),
          "a shared pair-check limit must reject rather than return a partial preservation suffix");
  const auto complete = derive_pad_preservation_demands(incumbent, candidate, existing, 2U, 3U);
  require(complete && complete->size() == 2U && complete->front().connection_index == 5U &&
              complete->back().connection_index == 6U,
          "the exact complete bound must admit the deterministic two-edge preservation MST");
}

void source_only_terminal_becomes_a_physical_obligation() {
  const auto planning = two_pad_board();
  auto source = planning;
  auto extra = source.footprints.front();
  extra.id = ObjectId{"physical-extra"};
  extra.reference = "U3";
  extra.position = mm(50,65);
  extra.pads.front().id = ObjectId{"physical-extra-pad"};
  source.footprints.push_back(std::move(extra));
  auto config = experiment_config();
  config.copper_scaffold.enabled = true;
  config.planner.quantized_copper_oracle.enabled = false;
  config.max_actions = 12U;
  RoutingEngine engine(planning, source, config);
  for (std::size_t step = 0; step < 20'000U && !engine.terminal(); ++step)
    static_cast<void>(engine.advance({.work_units = 256U}));
  const auto& state = engine.state();
  require(state.legal_scaffold && state.legal_scaffold->requests.size() == 2U &&
              state.legal_scaffold->complete &&
              analyze_connectivity(state.legal_scaffold->copper).open_connections == 0U,
          "the engine must route a physical source pad omitted from its original planning domain");
  require(state.scaffold_oracle && state.scaffold_oracle->feasible() &&
              engine.selected_plan() && engine.selected_plan()->legal_scaffold_complete,
          "physical suffix completion must earn exact whole-board scaffold certification");
}
void direct_scaffold_terminal_rebind_is_bounded_and_preserves_peers() {
  auto board = two_pad_board();
  const auto front = LayerId{"F.Cu"};
  const auto net = NetId{"engine-experiment-net"};
  Footprint third;
  third.id = ObjectId{"U3"}; third.reference = "U3"; third.position = mm(50, 60);
  third.pads = {{ObjectId{"U3-pad"}, net, Point{}, {front}}};
  board.footprints.push_back(third);
  GlobalRoutePath producer{net, 0U, {{mm(20,40),front},{mm(50,40),front},{mm(80,40),front}}};
  GlobalRoutePath failed{net, 1U, {{mm(50,60),front},{mm(50,40),front}}};
  failed.terminal_neck_down_segments = 1U; failed.terminal_neck_down_length_nm = 1U;
  CopperScaffoldResult retained; retained.copper = board; retained.requests = {producer, failed};
  retained.missing_indices = {1U}; retained.request_evidence.resize(2U);
  retained.request_evidence[1].access_evaluated = true;
  retained.request_evidence[1].termination = CopperScaffoldSearchTermination::attachment_exhausted;
  retained.request_evidence[1].target_legal_attachments = 0U;
  const auto rebound = derive_scaffold_terminal_rebinds(board, retained);
  require(rebound.second.size() == 1U && rebound.first[0] == producer &&
              rebound.first[1].points.back() != failed.points.back() &&
              rebound.first[1].terminal_neck_down_segments == 0U,
          "direct producer rebind must change only the failed demand and clear stale certificates");
  require(derive_scaffold_terminal_rebinds(board, retained, {}, {}, true).second.empty(),
          "transient repair must reject an unconnected producer even when direct discovery may inspect it");
  const auto consumed = derive_scaffold_terminal_rebinds(board, retained, {rebound.second.front().proof});
  require(consumed.second.empty() && consumed.first == retained.requests,
          "a consumed failed proof must not replace requests or request another retry");
  require(derive_scaffold_terminal_rebinds(board, retained, {rebound.second.front().proof},
              "changed-copper-context").second.size() == 1U,
          "a changed physical scene must re-enable the same producer binding");
  retained.satisfied_indices = {0U};
  retained.copper.tracks = {
      {ObjectId{"bypass1"}, net, front, mm(20,40), mm(20,30), Nanometres::from_mm(0.2), false},
      {ObjectId{"bypass2"}, net, front, mm(20,30), mm(80,30), Nanometres::from_mm(0.2), false},
      {ObjectId{"bypass3"}, net, front, mm(80,30), mm(80,40), Nanometres::from_mm(0.2), false}};
  const auto bypass = derive_scaffold_terminal_rebinds(board, retained);
  require(bypass.second.size() == 1U && bypass.first[0] == producer,
          "a satisfied producer bypassing its abstract branch point still needs a terminal binding");
  retained.request_evidence[1].termination = CopperScaffoldSearchTermination::expansion_budget_exhausted;
  retained.request_evidence[1].target_legal_attachments = 1U;
  require(derive_scaffold_terminal_rebinds(board, retained).second.size() == 1U,
          "a legal grid attachment must not pin a failed branch to obsolete producer geometry");
  retained.satisfied_indices.clear();
  require(derive_scaffold_terminal_rebinds(board, retained).second.empty(),
          "a budget failure must not rebind to an unchecked producer");
  retained.satisfied_indices = {0U};
  const auto checked_copper = retained.copper;
  retained.copper = board;
  require(derive_scaffold_terminal_rebinds(board, retained).second.empty(),
          "a satisfaction index alone must not substitute for physical producer connectivity");
  retained.copper = checked_copper;
  require(derive_scaffold_terminal_rebinds(board, retained, {}, {}, true).second.size() == 1U,
          "transient repair may use the same proof only after its producer is physically connected");
  retained.request_evidence[1].target_endpoint_contained_by_copper = true;
  require(derive_scaffold_terminal_rebinds(board, retained).second.empty(),
          "a branch still present in copper must keep its physical target");
  retained.request_evidence[1].target_endpoint_contained_by_copper = false;
  auto checkpoint = retained;
  checkpoint.requests = bypass.first;
  require(scaffold_preserves_satisfied_requests(checkpoint, retained),
          "a rebound checkpoint may retain all checked peers without adding copper");
  checkpoint.satisfied_indices.clear();
  require(!scaffold_preserves_satisfied_requests(checkpoint, retained),
          "a partial rebound checkpoint may not discard a checked peer");
  checkpoint = retained;
  checkpoint.requests[0].points.back() = {mm(51,41), front};
  require(!scaffold_preserves_satisfied_requests(checkpoint, retained),
          "an arbitrary changed terminal must not satisfy the outer preservation gate");
  auto proof_incumbent = retained;
  proof_incumbent.satisfied_indices = {1U};
  auto proof_checkpoint = proof_incumbent;
  proof_checkpoint.requests = bypass.first;
  proof_checkpoint.copper.tracks.push_back(
      {ObjectId{"proved-rebind"}, net, front, failed.points.front().position,
       bypass.first[1].points.back().position, Nanometres::from_mm(0.2), false});
  require(scaffold_preserves_satisfied_requests(proof_checkpoint, proof_incumbent),
          "only a pad-backed producer proof with preserved existing components may rebind a checked demand");
  Footprint unrelated;
  unrelated.id = ObjectId{"U4"}; unrelated.reference = "U4"; unrelated.position = mm(65,60);
  unrelated.pads = {{ObjectId{"U4-pad"}, net, Point{}, {front}}};
  proof_incumbent.copper.footprints.push_back(unrelated);
  auto arbitrary_pad = proof_checkpoint;
  arbitrary_pad.copper.footprints.push_back(unrelated);
  arbitrary_pad.requests[1].points.back() = {mm(65,60), front};
  arbitrary_pad.copper.tracks.push_back({ObjectId{"arbitrary-pad"}, net, front, mm(50,60), mm(65,60),
                                         Nanometres::from_mm(0.2), false});
  require(!scaffold_preserves_satisfied_requests(arbitrary_pad, proof_incumbent),
          "a same-net pad that is not the proved producer terminal must be rejected");
  auto real_pad_original = retained;
  real_pad_original.satisfied_indices = {1U};
  real_pad_original.requests[1].points.back() = producer.points.front();
  auto real_pad_relocation = proof_checkpoint;
  real_pad_relocation.requests[1].points.back() = producer.points.back();
  require(!scaffold_preserves_satisfied_requests(real_pad_relocation, real_pad_original),
          "a relocation from an already real pad is never a virtual-branch rebind");
  auto component_incumbent = proof_incumbent;
  component_incumbent.copper.tracks.push_back({ObjectId{"protected-pad-component"}, net, front,
      mm(50,60), mm(65,60), Nanometres::from_mm(0.2), false});
  auto component_regression = component_incumbent;
  component_regression.requests = bypass.first;
  component_regression.copper.tracks.push_back({ObjectId{"component-rebind"}, net, front,
      failed.points.front().position, bypass.first[1].points.back().position, Nanometres::from_mm(0.2), false});
  std::erase_if(component_regression.copper.tracks, [](const auto& track) {
    return track.id == ObjectId{"protected-pad-component"};
  });
  require(!scaffold_preserves_satisfied_requests(component_regression, component_incumbent),
          "a proof rebind must reject any split of a pre-existing pad component");
  retained.copper.tracks.push_back({ObjectId{"already-connected"}, net, front, mm(50,60), mm(20,40),
                                   Nanometres::from_mm(0.2), false});
  require(derive_scaffold_terminal_rebinds(board, retained).second.empty(),
          "rebinding must not certify a loop to an already connected source component");
  retained.copper = board;
  retained.requests[1].points.back() = {mm(51,41), front};
  require(derive_scaffold_terminal_rebinds(board, retained).second.empty(),
          "no exact producer proof must retain the original failure without nearest-pad fallback");
}
} // namespace

int main() {
  physical_regions_are_causal_bounded_and_fair();
  physical_repair_evidence_token_is_canonical_and_causal();
  physical_repair_evidence_reopens_only_changed_missing_demand();
  strict_scaffold_coverage_progress_excludes_numeric_and_metadata_changes();
  empty_scaffold_dispatches_missing_demand_to_regional_repair();
  bounded_scaffold_hands_failed_requests_to_regional_before_sweep_completion();
  selected_physical_repair_survives_cancellation_before_exact_admission();
  large_bounded_scaffold_batches_failures_without_losing_success_queue_context();
  forced_scaffold_eviction_does_not_preempt_unfinished_physical_regional_work();
  changed_copper_invalidates_a_scaffold_continuation();
  partial_scaffold_routes_missing_requests_against_retained_copper();
  residual_physical_demands_preserve_checked_components();
  residual_demand_pair_budget_is_atomic();
  partial_residual_physical_demands_are_net_scoped();
  pad_preservation_demands_restore_only_regressed_components();
  pad_preservation_demand_bounds_are_all_or_none();
  source_only_terminal_becomes_a_physical_obligation();
  direct_scaffold_terminal_rebind_is_bounded_and_preserves_peers();
  scaffold_certifies_materialization_source();
  rejected_scaffold_oracle_does_not_veto_an_independent_candidate();
  scaffold_replaces_all_routed_nets_when_scope_is_null();
  legal_scaffold_bypasses_regional_preparation();
  parked_engine_resumes_without_committing_partial_state();
  cancellation_and_deadline_dispose_parked_engine_work();
  retained_checkpoint_preview_never_claims_final_assessment();
  continuation_dependencies_and_knowledge_merge_are_bounded();
  external_rejection_switches_a_real_experiment_pool_before_safe_cancellation();
  std::cout << "routing engine experiment continuation tests passed\n";
  return 0;
}
