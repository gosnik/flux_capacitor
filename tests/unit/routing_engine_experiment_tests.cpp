#include "flux/core/routing_engine.hpp"
#include "flux/core/router_telemetry.hpp"
#include "../../src/scaffold_terminal_rebind.hpp"

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <stop_token>
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
  bool try_emit(RouterTelemetryEvent) noexcept override { return true; }
  void publish_generated_copper(const Board& board, std::string) override {
    latest = board;
    ++publications;
  }
  std::optional<Board> latest;
  std::size_t publications{0U};
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

void parked_engine_resumes_without_committing_partial_state() {
  const auto board = two_pad_board();
  RoutingEngine engine(board, board, experiment_config());
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
              cancelled.state().attempts.empty() && !cancelled.selected_plan(),
          "cancelling a parked transaction must clear uncommitted engine work");

  auto timed_config = config;
  timed_config.shared_deadline = std::chrono::milliseconds{100};
  RoutingEngine timed(board, board, timed_config);
  require(timed.advance({.work_units = 1U}).outcome == RoutingEngineOutcome::yielded,
          "deadline fixture must first park a live transaction");
  std::this_thread::sleep_for(std::chrono::milliseconds{150});
  require(timed.advance().outcome == RoutingEngineOutcome::timed_out && timed.state().attempts.empty() &&
              !timed.selected_plan(),
          "deadline expiry must dispose a parked transaction without publishing partial routing");
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
  source.footprints.push_back(std::move(extra));
  auto config = experiment_config();
  config.copper_scaffold.enabled = true;
  config.planner.quantized_copper_oracle.enabled = false;
  // The exact source has an additional open on the same net. Retain that scaffold rejection,
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
  empty_scaffold_dispatches_missing_demand_to_regional_repair();
  partial_scaffold_routes_missing_requests_against_retained_copper();
  direct_scaffold_terminal_rebind_is_bounded_and_preserves_peers();
  scaffold_certifies_materialization_source();
  rejected_scaffold_oracle_does_not_veto_an_independent_candidate();
  scaffold_replaces_all_routed_nets_when_scope_is_null();
  legal_scaffold_bypasses_regional_preparation();
  parked_engine_resumes_without_committing_partial_state();
  cancellation_and_deadline_dispose_parked_engine_work();
  continuation_dependencies_and_knowledge_merge_are_bounded();
  external_rejection_switches_a_real_experiment_pool_before_safe_cancellation();
  std::cout << "routing engine experiment continuation tests passed\n";
  return 0;
}
