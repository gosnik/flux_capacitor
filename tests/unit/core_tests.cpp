#include "flux/core/copper_geometry.hpp"
#include "flux/core/board_free_space.hpp"
#include "flux/core/detailed_router.hpp"
#include "flux/core/drc.hpp"
#include "flux/core/geometry.hpp"
#include "flux/core/global_router.hpp"
#include "flux/core/hydraulic_path.hpp"
#include "flux/core/integrated_optimizer.hpp"
#include "flux/core/navmesh_route.hpp"
#include "flux/core/pns_router.hpp"
#include "flux/core/quantized_copper_oracle.hpp"
#include "flux/core/regional_candidate_ranker.hpp"
#include "flux/core/regional_router.hpp"
#include "flux/core/regional_repair_scheduler.hpp"
#include "flux/core/router_telemetry.hpp"
#include "flux/core/routing_compute.hpp"
#include "flux/core/routing_feedback.hpp"
#include "flux/core/routing_engine.hpp"
#include "flux/core/routing_rules.hpp"
#include "flux/core/spline_route.hpp"
#include "flux/io/kicad_export.hpp"
#include "flux/io/kicad_import.hpp"
#include "flux/io/kicad_netclass_snapshot.hpp"
#include "flux/io/kicad_refill.hpp"
#include "flux/io/placement_move_priorities.hpp"
#include "flux/io/work_plan.hpp"
#include "flux/io/work_plan_executor.hpp"
#include "flux/io/work_plan_router.hpp"
#include "flux/telemetry/event.hpp"

#include <algorithm>
#include <barrier>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iostream>
#include <mutex>
#include <iterator>
#include <limits>
#include <numeric>
#include <set>
#include <stdexcept>
#include <stop_token>
#include <string>
#include <thread>
#include <vector>

namespace {

using flux::Board;
using flux::BoardSide;
using flux::Footprint;
using flux::GlobalRouteCheckpointObjective;
using flux::GlobalRouterConfig;
using flux::GlobalRouterStopReason;
using flux::Keepout;
using flux::LayerId;
using flux::LayerKind;
using flux::Microdegrees;
using flux::Nanometres;
using flux::Net;
using flux::NetId;
using flux::ObjectId;
using flux::Pad;
using flux::Point;
using flux::Polygon;

struct TestCase {
  std::string name;
  std::function<void()> run;
};

class CollectingTelemetrySink final : public flux::RouterTelemetrySink {
public:
  [[nodiscard]] bool accepts(flux::RouterTelemetryLevel) const noexcept override { return true; }

  [[nodiscard]] bool try_emit(flux::RouterTelemetryEvent event) noexcept override {
    std::lock_guard lock(events_mutex_);
    events.push_back(std::move(event));
    return true;
  }

  std::vector<flux::RouterTelemetryEvent> events;
private:
  std::mutex events_mutex_;
};

[[noreturn]] void fail(const std::string& message) { throw std::runtime_error(message); }

void require(bool condition, const std::string& message) {
  if (!condition) {
    fail(message);
  }
}

void regional_candidate_ranker_is_schema_bound_and_deterministic() {
  const std::string model = R"JSON({
    "schema_version": 1,
    "feature_names": [
      "heuristic_rank", "template_count", "via_count", "cross_section_count",
      "via_alternative_switches", "template_conflicts", "residual_conflicts",
      "alternative_template_count", "selected_global_template_count", "distinct_layer_count",
      "total_channel_steps", "total_layer_transitions", "total_topology_cost",
      "total_lower_bound_length_mm", "minimum_lane_slack_mm", "contested_via_count",
      "maximum_via_competitors", "bundle_search_states"
    ],
    "base_score": 0.25,
    "trees": [{"nodes": [
      {"id": 0, "feature": 0, "threshold": 1.5, "yes": 1, "no": 2, "missing": 2},
      {"id": 1, "leaf": 2.0},
      {"id": 2, "leaf": -1.0}
    ]}]
  })JSON";
  std::string diagnostic;
  const auto ranker = flux::RegionalCandidateRanker::load_json(model, diagnostic);
  flux::RegionalCandidateFeatureVector preferred{};
  preferred[0] = 1.0;
  auto deferred = preferred;
  deferred[0] = 2.0;
  require(ranker && diagnostic.empty() && ranker->tree_count() == 1U &&
              std::abs(ranker->score(preferred) - 2.25) < 1.0e-12 &&
              std::abs(ranker->score(deferred) + 0.75) < 1.0e-12,
          "the inference-only ranker must preserve exported split semantics");

  auto incompatible = model;
  const auto feature = incompatible.find("heuristic_rank");
  incompatible.replace(feature, std::string("heuristic_rank").size(), "renamed_feature");
  diagnostic.clear();
  require(!flux::RegionalCandidateRanker::load_json(incompatible, diagnostic) &&
              diagnostic.find("feature schema") != std::string::npos,
          "a model trained against another feature contract must fail closed");

  const auto observation_path =
      std::filesystem::temp_directory_path() /
      ("flux-regional-candidate-observation-" +
       std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()) + ".jsonl");
  flux::RegionalCandidateTrainingObservation observation;
  observation.board_revision = "fixture";
  observation.query_id = "fixture:0:1";
  observation.window_id = 1U;
  observation.features = preferred;
  observation.complete = true;
  observation.realization_us = 42U;
  diagnostic.clear();
  const auto appended = flux::append_regional_candidate_training_observation(
      observation_path, observation, diagnostic);
  std::ifstream observation_stream(observation_path);
  const std::string serialized((std::istreambuf_iterator<char>(observation_stream)),
                               std::istreambuf_iterator<char>());
  std::error_code remove_error;
  std::filesystem::remove(observation_path, remove_error);
  require(appended && diagnostic.empty() && !remove_error &&
              serialized.find("\"query_id\":\"fixture:0:1\"") != std::string::npos &&
              serialized.find("\"complete\":true") != std::string::npos &&
              serialized.ends_with('\n'),
          "candidate outcomes must be emitted as independently parseable append-only JSONL");
}

[[nodiscard]] Point mm(double x, double y) {
  return Point{Nanometres::from_mm(x), Nanometres::from_mm(y)};
}

[[nodiscard]] Polygon rectangle(double min_x, double min_y, double max_x, double max_y) {
  return Polygon{{mm(min_x, min_y), mm(max_x, min_y), mm(max_x, max_y), mm(min_x, max_y)}};
}

[[nodiscard]] Polygon local_rectangle(double half_width, double half_height) {
  return rectangle(-half_width, -half_height, half_width, half_height);
}

[[nodiscard]] Board two_pad_board() {
  const LayerId front{"F.Cu"};
  const LayerId back{"B.Cu"};
  const NetId net{"net-1"};
  Board board;
  board.source_revision = "risk-fixture";
  board.outline = rectangle(0.0, 0.0, 100.0, 80.0);
  board.layers = {{front, "F.Cu", LayerKind::copper, 0}, {back, "B.Cu", LayerKind::copper, 1}};
  board.nets = {{net, "SIGNAL", 0}};

  const auto footprint = [&front, &net](std::string id, Point position) {
    Footprint result;
    result.id = ObjectId{id};
    result.reference = id;
    result.position = position;
    result.courtyard = local_rectangle(4.0, 3.0);
    result.pads = {{ObjectId{id + "-pad"}, net, Point{}, {front}}};
    return result;
  };
  board.footprints = {footprint("U1", mm(20.0, 40.0)), footprint("U2", mm(80.0, 40.0))};
  return board;
}

void routing_engine_resumes_and_requires_current_external_validation() {
  const auto board = two_pad_board();
  flux::RoutingEngineConfig config;
  config.copper_scaffold.enabled = false; // Exercise regional decision-point semantics.
  config.planner.global.grid_pitch = Nanometres::from_mm(5.0);
  config.planner.global.max_iterations = 1U;
  config.planner.global.max_pin_escape_grid_radius = 2U;
  config.planner.global.max_pin_access_nodes_per_terminal = 32U;
  config.planner.max_routability_feedback_retries = 0U;
  config.max_actions = 8U;
  std::size_t yielded_steps = 0U;
  auto run = [&](flux::RoutingEngine& engine, std::size_t work) {
    for (std::size_t step = 0U; step < 100'000U && !engine.terminal(); ++step) {
      const auto result = engine.advance({.work_units = work,
                                          .time_slice = std::chrono::milliseconds{10}});
      if (result.outcome == flux::RoutingEngineOutcome::yielded) {
        ++yielded_steps;
        require(!result.accepted_checkpoint_changed, "yield must not commit partial copper");
      }
      if (result.outcome == flux::RoutingEngineOutcome::awaiting_external_validation ||
          result.outcome == flux::RoutingEngineOutcome::awaiting_materialization) return;
    }
  };
  flux::RoutingEngine small(board, board, config);
  config.speculative_preparation = false;
  CollectingTelemetrySink engine_telemetry;
  flux::RoutingEngine large(board, board, config, &engine_telemetry);
  run(small, 1U);
  run(large, 100'000U);
  require(small.state().outcome == flux::RoutingEngineOutcome::awaiting_external_validation &&
              large.state().outcome == flux::RoutingEngineOutcome::awaiting_external_validation,
          "internally legal copper must wait for external DRC, not report completion");
  require(small.selected_detailed() && large.selected_detailed() &&
              small.selected_detailed()->complete() && large.selected_detailed()->complete() &&
              small.selected_plan()->paths == large.selected_plan()->paths &&
              small.selected_detailed()->candidate == large.selected_detailed()->candidate,
          "changing cooperative slice size must preserve routing and copper");
  require(small.state().live_copper_terrain_revision > 0U &&
              small.state().live_copper_terrain_features > 0U &&
              !small.state().live_copper_terrain.provisional &&
              small.state().live_copper_terrain_exact_replacements > 0U &&
              small.state().live_copper_terrain == large.state().live_copper_terrain,
          "completed materialization must replace provisional terrain with slice-independent copper");
  require(yielded_steps > 0U, "small slices must exercise actual suspended kernel progress");
  require(small.state().decision_points > 0U &&
              small.state().decision_points == large.state().decision_points &&
              small.state().speculative_preparations_started == 1U &&
              large.state().speculative_preparations_started == 0U,
          "semantic decisions must be telemetry/slice independent with one optional preparation worker");
  if (small.selected_plan()->regional.spline_candidate_available) {
    require(small.state().tightening_actions == 1U && large.state().tightening_actions == 1U,
            "a completed curved candidate must receive exactly one bounded tightening action");
    const auto tightening = std::find_if(small.state().attempts.begin(), small.state().attempts.end(),
        [](const auto& attempt) { return attempt.action == flux::RoutingEngineAction::tighten_routes; });
    const auto materialize = std::find_if(small.state().attempts.begin(), small.state().attempts.end(),
        [](const auto& attempt) { return attempt.action == flux::RoutingEngineAction::materialize_copper; });
    require(tightening < materialize, "tightening must precede final copper materialization");
  }
  auto limited_config = config;
  limited_config.max_actions = small.state().attempts.size() - small.state().tightening_actions;
  flux::RoutingEngine limited(board, board, limited_config);
  run(limited, 100'000U);
  require(limited.state().outcome == flux::RoutingEngineOutcome::awaiting_external_validation &&
              limited.state().tightening_actions == 0U,
          "optional tightening must reserve the final action for copper materialization");
  const auto generation = small.state().generation;
  small.acknowledge_external_validation(generation + 1U, true);
  require(small.state().outcome == flux::RoutingEngineOutcome::awaiting_external_validation,
          "stale external validation must not authorize a different copper revision");
  small.acknowledge_external_validation(generation, true);
  require(small.state().outcome == flux::RoutingEngineOutcome::complete,
          "current passing external DRC should complete the engine");
  flux::RoutabilityDiagnosis rejection;
  rejection.resource.stable_id = "external-test-clearance";
  rejection.resource.nets = {NetId{"net-1"}};
  rejection.suggested_relaxation = flux::RoutabilityRelaxation::change_corridor;
  large.acknowledge_external_validation(large.state().generation, false, {rejection});
  require(large.state().outcome == flux::RoutingEngineOutcome::progressed &&
              large.state().next_action != flux::RoutingEngineAction::none &&
              (!large.selected_detailed() || !large.selected_detailed()->complete()),
          "external rejection must revoke acceptance and schedule evidence-directed repair");
}

void routing_engine_distinguishes_deadline_cancellation_and_budget() {
  const auto board = two_pad_board();
  flux::RoutingEngineConfig config;
  config.copper_scaffold.enabled = false; // Exercise regional decision-point semantics.
  config.max_actions = 0U;
  flux::RoutingEngine exhausted(board, board, config);
  require(exhausted.advance().outcome == flux::RoutingEngineOutcome::exhausted &&
              exhausted.selected_plan() == nullptr,
          "zero action budget must exhaust without claiming infeasibility or routing");
  config.max_actions = 4U;
  std::stop_source cancellation;
  cancellation.request_stop();
  flux::RoutingEngine cancelled(board, board, config);
  require(cancelled.advance({}, cancellation.get_token()).outcome ==
              flux::RoutingEngineOutcome::cancelled,
          "caller cancellation must remain distinct from budget exhaustion");
  config.planner.global.grid_pitch = Nanometres::from_mm(5.0);
  config.planner.global.max_iterations = 1U;
  config.planner.max_routability_feedback_retries = 0U;
  config.max_decision_points = 1U;
  flux::RoutingEngine bounded(board, board, config);
  for (std::size_t step = 0; step < 10000U && !bounded.terminal(); ++step)
    static_cast<void>(bounded.advance({.work_units = 100000U}));
  require(bounded.state().outcome == flux::RoutingEngineOutcome::exhausted &&
              bounded.state().decision_points == 1U &&
              !bounded.state().attempts.empty() &&
              bounded.state().attempts.back().interrupted_at_decision &&
              !bounded.state().accepted_detailed,
          "semantic decision limit must stop unevaluated search without publishing copper");
  // Legacy credit settings must not prevent a first physical realization. Runtime and
  // explicit action/decision bounds remain limits, not progress-earned permission.
  auto experiment_config = config;
  experiment_config.max_actions = 32U;
  experiment_config.max_decision_points = 4096U;
  experiment_config.planner.global.max_iterations = 12U;
  experiment_config.preparation_work_units = 0U;
  experiment_config.initial_regional_work_units = 0U;
  experiment_config.regional_work_units_per_improvement = 0U;
  experiment_config.max_regional_work_units = 0U;
  experiment_config.initial_global_retry_attempts = 0U;
  experiment_config.max_changed_context_retries = 0U;
  experiment_config.global_retry_attempts_per_improvement = 0U;
  flux::RoutingEngine experiments(board, board, experiment_config);
  for (std::size_t step = 0U; step < 10000U && !experiments.terminal(); ++step) {
    const auto result = experiments.advance({.work_units = 100000U});
    if (result.outcome == flux::RoutingEngineOutcome::awaiting_external_validation) break;
  }
  require(experiments.state().outcome == flux::RoutingEngineOutcome::awaiting_external_validation &&
              experiments.selected_detailed() && experiments.selected_detailed()->complete(),
          "zero legacy credits must not starve physical realization of a routable board");
  config.shared_deadline = std::chrono::milliseconds{1};
  flux::RoutingEngine timed(board, board, config);
  std::this_thread::sleep_for(std::chrono::milliseconds{20});
  require(timed.advance().outcome == flux::RoutingEngineOutcome::timed_out &&
              timed.selected_detailed() == nullptr,
          "shared deadline must stop before work and retain no invented copper");
}

void routing_engine_explores_supported_physical_families() {
  const auto board = two_pad_board();
  flux::RoutingEngineConfig config;
  // This fixture exercises fallback-family dispatch, not the constructive shortcut.
  config.copper_scaffold.enabled = false;
  config.planner.curved_tracks = true;
  config.planner.global.grid_pitch = Nanometres::from_mm(5.0);
  config.planner.global.max_iterations = 12U;
  config.planner.global.max_pin_escape_grid_radius = 2U;
  config.planner.global.max_pin_access_nodes_per_terminal = 32U;
  config.planner.max_routability_feedback_retries = 0U;
  config.planner.regional.xpbd.iterations = 1U;
  config.planner.regional.max_confined_anisotropic_erosion_checks = 1U;
  config.max_actions = 32U;
  flux::RoutingEngine engine(board, board, config);
  bool probe = false, xpbd = false, erosion = false, usped = false;
  for (std::size_t step = 0U; step < 100'000U && !engine.terminal(); ++step) {
    const auto result = engine.advance({.work_units = 10'000U});
    if (result.outcome == flux::RoutingEngineOutcome::yielded) continue;
    const auto& state = engine.state();
    if (state.working_plan) {
      if (result.action == flux::RoutingEngineAction::regional_realization &&
          state.working_plan->regional_evaluated)
        require(state.working_plan->regional.configured.enable_xpbd_relaxation &&
                    state.working_plan->regional.configured.enable_avbd_relaxation,
                "the first regional action must retain joint XPBD and AVBD eligibility");
      if (result.action == flux::RoutingEngineAction::probe_quantized_copper)
        probe = state.working_plan->quantized_copper_oracle.evaluated;
      if (result.action == flux::RoutingEngineAction::xpbd_repair)
        xpbd = state.working_plan->regional.configured.enable_xpbd_relaxation;
      if (result.action == flux::RoutingEngineAction::confined_erosion_repair)
        erosion = state.working_plan->regional.configured.max_confined_anisotropic_erosion_checks == 1U;
      if (result.action == flux::RoutingEngineAction::usped_global_replan) {
        usped = state.working_plan->coarse_global.configured_global_navmesh_usped.enabled;
        require(state.working_plan->coarse_global.configured_live_copper_terrain_revision > 0U &&
                    !state.working_plan->coarse_global.configured_live_copper_terrain.features.empty(),
                "global replans must consume copper feedback produced by earlier engine actions");
      }
    }
    if (probe && xpbd && erosion && usped) break;
    if (result.outcome == flux::RoutingEngineOutcome::awaiting_external_validation) {
      // The oracle deliberately rejects this otherwise legal fixture. Repeated identical
      // evidence must exercise different eligible families rather than replay one local action.
      flux::RoutabilityDiagnosis rejection;
      rejection.resource.stable_id = "oracle-repeated-physical-conflict";
      rejection.resource.nets = {NetId{"net-1"}};
      rejection.suggested_relaxation = flux::RoutabilityRelaxation::change_corridor;
      engine.acknowledge_external_validation(state.generation, false, {rejection});
    }
  }
  require(probe && xpbd && erosion && usped,
          "bounded exploration must dispatch quantized, XPBD, erosion and USPED kernels with actual configurations");
}

void routing_engine_scaffold_progress_preserves_external_validation_gate() {
  const auto board = two_pad_board();
  flux::RoutingEngineConfig config;
  config.planner.global.grid_pitch = Nanometres::from_mm(5.0);
  config.planner.global.max_iterations = 1U;
  config.max_actions = 12U;
  flux::RoutingEngine engine(board, board, config);
  for (std::size_t step = 0U; step < 10000U && !engine.terminal(); ++step) {
    const auto result = engine.advance({.work_units = 1000U});
    if (result.outcome == flux::RoutingEngineOutcome::awaiting_external_validation) break;
  }
  const auto& state = engine.state();
  require(state.legal_scaffold && state.legal_scaffold->complete &&
              state.legal_scaffold->request_count == 1U &&
              state.legal_scaffold->satisfied_indices.size() == 1U &&
              std::any_of(state.attempts.begin(), state.attempts.end(), [](const auto& attempt) {
                return attempt.action == flux::RoutingEngineAction::construct_copper_scaffold && attempt.accepted;
              }),
          "checked scaffold connections must be retained and counted as accepted physical progress");
  require(state.outcome == flux::RoutingEngineOutcome::awaiting_external_validation &&
              engine.selected_detailed() && engine.selected_detailed()->complete(),
          "constructive scaffold completion must still await external validation");
  const auto generation = state.generation;
  engine.acknowledge_external_validation(generation + 1U, true);
  require(engine.state().outcome == flux::RoutingEngineOutcome::awaiting_external_validation,
          "stale validation cannot accept scaffold materialization");
  engine.acknowledge_external_validation(generation, true);
  require(engine.state().outcome == flux::RoutingEngineOutcome::complete,
          "current passing external validation must complete scaffold routing");
}

void routing_engine_cancels_a_live_continuation() {
  const auto board = two_pad_board();
  flux::RoutingEngineConfig config;
  config.planner.global.grid_pitch = Nanometres::from_mm(5.0);
  config.planner.global.max_iterations = 1U;
  flux::RoutingEngine engine(board, board, config);
  const auto first = engine.advance({.work_units = 1U});
  require(first.outcome == flux::RoutingEngineOutcome::yielded,
          "fixture must hold a suspended global search");
  std::stop_source stop;
  stop.request_stop();
  require(engine.advance({}, stop.get_token()).outcome == flux::RoutingEngineOutcome::cancelled &&
              engine.state().attempts.empty() && !engine.selected_plan(),
          "cancellation must discard in-flight transaction without committing its partial outputs");
  config.shared_deadline = std::chrono::milliseconds{1};
  flux::RoutingEngine timed(board, board, config);
  for (std::size_t step = 0U; step < 100'000U && !timed.terminal(); ++step)
    static_cast<void>(timed.advance({.work_units = 1U}));
  require(timed.state().outcome == flux::RoutingEngineOutcome::timed_out,
          "deadline during an active continuation must not be labelled caller cancellation");
}

void quantized_copper_oracle_exposes_physical_conflicts_and_via_chains() {
  // Keep exercising nontrivial snapping even when normal feedback preserves native coordinates.
  flux::QuantizedCopperOracleConfig coarse_config;
  coarse_config.grid_pitch = Nanometres{100000};
  auto crossing_board = two_pad_board();
  const NetId crossing_net{"net-2"};
  crossing_board.nets.push_back({crossing_net, "CROSSING", 0});
  const auto crossing_terminal = [&](std::string id, Point position) {
    Footprint footprint;
    footprint.id = ObjectId{id};
    footprint.reference = id;
    footprint.position = position;
    footprint.courtyard = local_rectangle(2.0, 2.0);
    footprint.pads = {
        {ObjectId{id + "-pad"}, crossing_net, Point{}, {LayerId{"F.Cu"}}}};
    return footprint;
  };
  crossing_board.footprints.push_back(crossing_terminal("U3", mm(50.0, 20.0)));
  crossing_board.footprints.push_back(crossing_terminal("U4", mm(50.0, 60.0)));

  flux::GlobalRouteResult crossing;
  crossing.paths = {
      flux::GlobalRoutePath{NetId{"net-1"},
                            0U,
                            {{mm(20.0, 40.0), LayerId{"F.Cu"}},
                             {mm(80.0, 40.0), LayerId{"F.Cu"}}}},
      flux::GlobalRoutePath{crossing_net,
                            0U,
                            {{mm(50.0, 20.0), LayerId{"F.Cu"}},
                             {mm(50.0, 60.0), LayerId{"F.Cu"}}}}};
  const auto rejected = flux::probe_quantized_copper_feasibility(crossing_board, crossing, coarse_config);
  require(rejected.evaluated && rejected.all_paths_materialized &&
              !rejected.exact_copper_legal && rejected.drc_violations > 0U &&
              std::any_of(rejected.diagnoses.begin(), rejected.diagnoses.end(),
                          [&](const auto& diagnosis) {
                            return diagnosis.stage ==
                                       flux::RoutabilityFailureStage::quantization &&
                                   diagnosis.resource.nets.size() == 2U;
                          }),
          "fine-grid oracle must expose a mutually illegal quantized allocation before PH");

  auto via_board = two_pad_board();
  flux::GlobalRouteResult via_chain;
  flux::GlobalRoutePath via_path;
  via_path.net = NetId{"net-1"};
  via_path.points = {{mm(20.0, 40.0), LayerId{"F.Cu"}},
                     {mm(40.04, 40.0), LayerId{"F.Cu"}},
                     {mm(40.04, 40.0), LayerId{"B.Cu"}},
                     {mm(60.04, 40.0), LayerId{"B.Cu"}},
                     {mm(60.04, 40.0), LayerId{"F.Cu"}},
                     {mm(80.0, 40.0), LayerId{"F.Cu"}}};
  via_chain.paths = {std::move(via_path)};
  const auto accepted = flux::probe_quantized_copper_feasibility(via_board, via_chain, coarse_config);
  require(accepted.feasible() && accepted.generated_tracks == 3U &&
              accepted.generated_vias == 2U && accepted.drc_violations == 0U,
          "fine-grid oracle must represent a complete F.Cu -> B.Cu -> F.Cu via chain as legal "
          "quantized copper: " +
              accepted.diagnostic);
  auto endpoint_board = two_pad_board();
  endpoint_board.footprints.front().position = mm(20.04, 40.03);
  endpoint_board.footprints.back().position = mm(80.04, 40.03);
  endpoint_board.layers.back().stack_order = 3U;
  endpoint_board.layers.push_back({LayerId{"In1.Cu"}, "In1.Cu", LayerKind::copper, 1});
  endpoint_board.layers.push_back({LayerId{"In2.Cu"}, "In2.Cu", LayerKind::copper, 2});
  auto endpoint_chain = via_chain;
  endpoint_chain.paths.front().points = {
      {mm(20.04, 40.03), LayerId{"F.Cu"}},
      {mm(20.04, 40.03), LayerId{"In1.Cu"}},
      {mm(20.04, 40.03), LayerId{"B.Cu"}},
      {mm(80.04, 40.03), LayerId{"B.Cu"}},
      {mm(80.04, 40.03), LayerId{"In1.Cu"}},
      {mm(80.04, 40.03), LayerId{"F.Cu"}}};
  for (const bool preserve : {true, false}) {
    auto endpoint_config = coarse_config;
    endpoint_config.preserve_path_endpoints = preserve;
    const auto endpoints = flux::probe_quantized_copper_feasibility(
        endpoint_board, endpoint_chain, endpoint_config);
    require(endpoints.feasible() && endpoints.malformed_paths == 0U &&
                endpoints.generated_vias == 2U && endpoints.generated_tracks == 1U,
            "coincident multilayer endpoint chains must quantize as one anchored site: " +
                std::to_string(endpoints.malformed_paths) + "; " + endpoints.diagnostic);
  }
  auto disconnected_source = via_chain;
  disconnected_source.paths.front().points[2].position.x += Nanometres{1};
  const auto disconnected = flux::probe_quantized_copper_feasibility(via_board, disconnected_source, coarse_config);
  require(!disconnected.all_paths_materialized && disconnected.malformed_paths == 1U,
          "rounding two distinct source sites together must not legalize a lateral layer jump");

  // Interior source holes are immutable electrical anchors, not new via candidates.
  for (const bool use_pad : {false, true}) {
    auto anchored_board = via_board;
    anchored_board.rules.hole_to_hole_clearance = Nanometres{250000};
    for (const auto x : {40.04, 60.04}) {
      const auto id = "source-anchor-" + std::to_string(x);
      if (use_pad) {
        auto footprint = anchored_board.footprints.front();
        footprint.id = ObjectId{id};
        footprint.reference = id;
        footprint.position = mm(x, 40.0);
        auto& pad = footprint.pads.front();
        pad.id = ObjectId{id + "-pad"};
        pad.layers = {LayerId{"F.Cu"}, LayerId{"B.Cu"}};
        pad.plated = true;
        pad.drill_width = Nanometres{600000};
        pad.drill_height = pad.drill_width;
        anchored_board.footprints.push_back(std::move(footprint));
      } else {
        anchored_board.vias.push_back(flux::Via{ObjectId{id}, NetId{"net-1"}, mm(x, 40.0),
            Nanometres{600000}, Nanometres{300000}, {LayerId{"F.Cu"}, LayerId{"B.Cu"}}, true});
      }
    }
    for (const bool preserve_endpoints : {false, true}) {
      auto anchor_config = coarse_config;
      anchor_config.preserve_path_endpoints = preserve_endpoints;
      const auto anchored = flux::probe_quantized_copper_feasibility(
          anchored_board, via_chain, anchor_config);
      require(anchored.feasible() && anchored.reused_same_net_transitions == 2U &&
                  anchored.generated_vias == 0U && anchored.generated_tracks == 3U,
              "quantization must preserve exact source-owned interior holes without duplicate vias");
    }
    if (use_pad) {
      for (const int invalid_kind : {0, 1, 2}) {
        auto invalid_anchor = anchored_board;
        auto& pad = invalid_anchor.footprints.back().pads.front();
        if (invalid_kind == 0) {
          invalid_anchor.nets.push_back({NetId{"other-net"}, "OTHER", 0});
          pad.net = NetId{"other-net"};
        } else if (invalid_kind == 1) {
          pad.plated = false;
        } else {
          pad.layers = {LayerId{"F.Cu"}};
        }
        const auto refused = flux::probe_quantized_copper_feasibility(invalid_anchor, via_chain, coarse_config);
        require(!refused.feasible() && refused.reused_same_net_transitions == 1U &&
                    refused.generated_vias == 1U,
                "wrong-net, unplated or layer-incomplete holes must not certify a reused transition");
      }
    }
  }

  // A foreign source via is an immutable obstacle.  This deliberately uses one copper layer:
  // before the planar reclassification, via-site feedback can only charge layer-transition
  // edges, so no history edge is prepared at all.
  auto fixed_via_board = two_pad_board();
  fixed_via_board.layers.resize(1U);
  fixed_via_board.nets.push_back({NetId{"fixed-via-net"}, "FIXED_VIA", 0});
  fixed_via_board.vias.push_back(
      {ObjectId{"fixed-source-via"}, NetId{"fixed-via-net"}, mm(50.0, 40.5),
       Nanometres::from_mm(0.6), Nanometres::from_mm(0.3), {LayerId{"F.Cu"}}, true});
  flux::GlobalRouteResult fixed_via_trace;
  fixed_via_trace.paths = {flux::GlobalRoutePath{NetId{"net-1"}, 0U,
      {{mm(20.0, 40.0), LayerId{"F.Cu"}}, {mm(80.0, 40.0), LayerId{"F.Cu"}}}}};
  const auto fixed_via_rejected =
      flux::probe_quantized_copper_feasibility(fixed_via_board, fixed_via_trace);
  const auto fixed_via_diagnosis = std::find_if(
      fixed_via_rejected.diagnoses.begin(), fixed_via_rejected.diagnoses.end(), [](const auto& diagnosis) {
        return diagnosis.kind == flux::RoutabilityFailureKind::quantization_clearance &&
               diagnosis.suggested_relaxation == flux::RoutabilityRelaxation::change_corridor &&
               diagnosis.resource.nets == std::vector<NetId>{NetId{"net-1"}};
      });
  require(!fixed_via_rejected.feasible() && fixed_via_rejected.generated_tracks == 1U &&
              fixed_via_rejected.generated_vias == 0U &&
              fixed_via_rejected.drc_violation_counts.contains("track_via_clearance") &&
              fixed_via_diagnosis != fixed_via_rejected.diagnoses.end() &&
              fixed_via_diagnosis->resource.bounds.minimum.x > Nanometres::from_mm(49.0) &&
              fixed_via_diagnosis->resource.bounds.maximum.x < Nanometres::from_mm(51.0) &&
              fixed_via_diagnosis->resource.bounds.minimum.y > Nanometres::from_mm(39.0) &&
              fixed_via_diagnosis->resource.bounds.maximum.y < Nanometres::from_mm(41.0) &&
              !fixed_via_diagnosis->resource.bounds.contains(mm(20.0, 40.0)),
          "a generated trace blocked by an immutable source via must retain its DRC rejection "
          "while localizing a planar corridor change to the fixed obstacle");
  auto nearly_clear_via_board = fixed_via_board;
  nearly_clear_via_board.vias.front().position = mm(50.0, 40.624);
  const auto nearly_clear_via =
      flux::probe_quantized_copper_feasibility(nearly_clear_via_board, fixed_via_trace);
  require(nearly_clear_via.drc_violation_counts.contains("track_via_clearance") &&
              std::any_of(nearly_clear_via.diagnoses.begin(), nearly_clear_via.diagnoses.end(),
                          [](const auto& diagnosis) {
                            return diagnosis.kind == flux::RoutabilityFailureKind::quantization_clearance &&
                                diagnosis.suggested_relaxation == flux::RoutabilityRelaxation::change_corridor &&
                                diagnosis.resource.bounds.maximum.x - diagnosis.resource.bounds.minimum.x <
                                    Nanometres::from_mm(2.0);
                          }),
          "a small fixed-via clearance shortfall must not fall back to the whole trace bounds");
  flux::RoutabilityKnowledge fixed_via_knowledge;
  flux::accumulate_routability_knowledge(fixed_via_knowledge, fixed_via_rejected.diagnoses);
  GlobalRouterConfig fixed_via_feedback;
  // The default 0.25 mm trace plus 0.20 mm clearance requires at least a 0.45 mm
  // planar grid.  This 0.50 mm lattice also has edges crossing the localized witness.
  fixed_via_feedback.grid_pitch = Nanometres::from_mm(0.5);
  fixed_via_feedback.max_iterations = 1U;
  fixed_via_feedback.routability_history = fixed_via_knowledge.history;
  const auto fixed_via_feedback_route = flux::route_global(fixed_via_board, fixed_via_feedback);
  require(std::any_of(fixed_via_knowledge.history.begin(), fixed_via_knowledge.history.end(),
                      [](const auto& entry) {
                        return entry.kind == flux::RoutabilityFailureKind::quantization_clearance &&
                               entry.suggested_relaxation ==
                                   flux::RoutabilityRelaxation::change_corridor;
                      }) &&
              fixed_via_feedback_route.stop_reason != flux::GlobalRouterStopReason::invalid_input &&
              fixed_via_feedback_route.configured_grid_pitch == Nanometres::from_mm(0.5) &&
              fixed_via_feedback_route.routability_history_edges > 0U,
          "immutable source-via trace conflicts must accumulate onto planar routing edges");

  // DrillShape applies pad-local drill offsets and the placed footprint transform.  The
  // non-plated source hole must therefore use that world-space geometry when it localizes the
  // generated trace's hole-clearance feedback.
  auto fixed_hole_board = two_pad_board();
  fixed_hole_board.layers.resize(1U);
  fixed_hole_board.rules.hole_clearance = Nanometres::from_mm(0.2);
  fixed_hole_board.nets.push_back({NetId{"fixed-hole-net"}, "FIXED_HOLE", 0});
  Footprint fixed_hole;
  fixed_hole.id = ObjectId{"fixed-drilled-pad"};
  fixed_hole.reference = "FIXED_HOLE";
  fixed_hole.position = mm(50.0, 40.0);
  fixed_hole.rotation = Microdegrees::from_degrees(90.0);
  fixed_hole.pads = {{ObjectId{"fixed-drilled-pad-hole"}, NetId{"fixed-hole-net"},
                      mm(0.2, -0.1), {LayerId{"F.Cu"}}, flux::PadShape::circle,
                      Nanometres::from_mm(1.0), Nanometres::from_mm(1.0), Nanometres{0},
                      Microdegrees::from_degrees(90.0), Nanometres::from_mm(0.6),
                      Nanometres::from_mm(0.6), mm(0.1, 0.0), false}};
  const auto fixed_hole_position =
      flux::pad_drill_world_position(fixed_hole, fixed_hole.pads.front());
  fixed_hole_board.footprints.push_back(fixed_hole);
  flux::GlobalRouteResult fixed_hole_trace;
  fixed_hole_trace.paths = {flux::GlobalRoutePath{NetId{"net-1"}, 0U,
      {{Point{Nanometres::from_mm(20.0), fixed_hole_position.y}, LayerId{"F.Cu"}},
       {Point{Nanometres::from_mm(80.0), fixed_hole_position.y}, LayerId{"F.Cu"}}}}};
  const auto fixed_hole_rejected =
      flux::probe_quantized_copper_feasibility(fixed_hole_board, fixed_hole_trace);
  const auto fixed_hole_diagnosis = std::find_if(
      fixed_hole_rejected.diagnoses.begin(), fixed_hole_rejected.diagnoses.end(), [](const auto& diagnosis) {
        return diagnosis.kind == flux::RoutabilityFailureKind::quantization_clearance &&
               diagnosis.suggested_relaxation == flux::RoutabilityRelaxation::change_corridor &&
               diagnosis.detail.find("hole_clearance") != std::string::npos;
      });
  require(!fixed_hole_rejected.feasible() && fixed_hole_rejected.generated_tracks == 1U &&
              fixed_hole_rejected.generated_vias == 0U &&
              fixed_hole_rejected.drc_violation_counts.contains("hole_clearance") &&
              fixed_hole_diagnosis != fixed_hole_rejected.diagnoses.end() &&
              fixed_hole_diagnosis->resource.bounds.contains(fixed_hole_position) &&
              !fixed_hole_diagnosis->resource.bounds.contains(
                  Point{Nanometres::from_mm(20.0), fixed_hole_position.y}),
          "a generated trace blocked by a transformed fixed drill must request a local planar "
          "corridor change");
}

void straight_regional_lanes_require_exact_copper_proof() {
  auto board = two_pad_board();
  board.footprints.clear();
  board.nets = {{NetId{"a"}, "a", 0}, {NetId{"b"}, "b", 0}};
  flux::GlobalRouteResult global;
  flux::RegionalRouteResult regional;
  flux::RegionalRoutingWindow window;
  window.id = 0U;
  flux::RegionalBundleAssignment assignment;
  assignment.window_id = 0U;
  assignment.stop_reason = flux::RegionalBundleStopReason::complete;
  for (std::size_t i = 0; i < 2; ++i) {
    flux::RegionalRouteFragment fragment;
    fragment.id = {board.nets[i].id, i, 0U, 0U};
    fragment.layer = LayerId{"F.Cu"};
    fragment.start = mm(20, i == 0 ? 39.5 : 40.5);
    fragment.end = mm(80, i == 0 ? 39.5 : 40.5);
    fragment.width = Nanometres{250000};
    fragment.clearance = Nanometres{200000};
    regional.fragments.push_back(fragment);
    window.fragments.push_back(fragment.id);
    window.owned_fragments.push_back(fragment.id);
    flux::RegionalRouteTemplate route_template;
    route_template.id = i;
    route_template.fragment = fragment.id;
    route_template.routing_layer = fragment.layer;
    assignment.templates.push_back(route_template);
    assignment.selected_templates.push_back(i);
    // Both abstract paths share the central copper. The chosen packed lanes must replace it.
    flux::GlobalRoutePath path;
    path.net = fragment.id.net;
    path.connection_index = i;
    path.points = {{fragment.start, fragment.layer}, {mm(35, 40), fragment.layer},
                   {mm(65, 40), fragment.layer}, {fragment.end, fragment.layer}};
    global.paths.push_back(path);
  }
  assignment.packed_cross_sections.push_back(flux::pack_regional_cross_section(
      regional.fragments, window.fragments, LayerId{"F.Cu"}, mm(50, 38), mm(50, 42),
      Nanometres{4000000}));
  regional.windows = {window};
  regional.bundle_assignments = {assignment};
  const auto unresolved = regional;
  flux::realize_regional_straight_candidate(board, global, regional);
  require(regional.spline_candidate_available && regional.native_scene_conflicts == 0U &&
              regional.native_scene_evaluated && regional.native_scene_complete &&
              regional.straight_scene_evaluated && regional.linear_scaffold_checks == 1U &&
              regional.linear_scaffold_rejections == 0U,
          "packed straight lanes must replace colliding abstract copper: " + regional.spline_candidate_diagnostic);
  require(regional.native_realization_manifest.size() == 2U,
          "each owned connection needs an explicit segment transaction");
  auto rebuilt = regional;
  rebuilt.bundle_assignments.front().selected_templates = {99999U};
  flux::realize_regional_straight_candidate(board, global, rebuilt);
  require(!rebuilt.native_scene_evaluated && !rebuilt.native_scene_complete &&
              !rebuilt.straight_scene_evaluated && !rebuilt.spline_candidate_available &&
              rebuilt.native_scene_conflicts == 0U && rebuilt.straight_scene_drc_violations == 0U &&
              flux::regional_checkpoint_better(regional, rebuilt),
          "an aborted scene rebuild must invalidate previous physical assessment evidence");
  for (const auto& route : regional.native_realization_manifest) {
    require(route.straight_realization && route.control_points.empty() &&
                !route.continuous_envelope_evaluated && route.native_copper_legal,
            "straight copper must not claim a PH certificate");
    for (const auto& piece : route.native_pieces) {
      const auto dx = std::abs(piece.end.x.value() - piece.start.x.value());
      const auto dy = std::abs(piece.end.y.value() - piece.start.y.value());
      require(piece.kind == flux::SplineFittedPieceKind::segment &&
                  (dx == 0 || dy == 0 || dx == dy), "all generated pieces must have ordinary angles");
    }
  }
  auto alternative = unresolved;
  auto detour = alternative.bundle_assignments.front().templates.front();
  detour.id = 2U;
  detour.selected_global_topology = false;
  detour.topology_centerline = {regional.fragments.front().start, mm(50, 81),
                                regional.fragments.front().end};
  alternative.bundle_assignments.front().templates.push_back(detour);
  alternative.bundle_assignments.front().selected_templates = {2U, 1U};
  flux::RegionalBundleSelection source_selection;
  source_selection.templates = {0U, 1U};
  source_selection.packed_cross_sections = unresolved.bundle_assignments.front().packed_cross_sections;
  alternative.bundle_assignments.front().spline_candidate_shortlist = {source_selection};
  flux::realize_regional_straight_candidate(board, global, alternative);
  require(alternative.spline_candidate_available && alternative.straight_scene_trials == 2U &&
              alternative.straight_source_fallback_selected,
          "exact copper must recover the retained source-topology lane assignment when a detour fails");
  auto blocked = unresolved;
  board.tracks.push_back({ObjectId{"wall"}, NetId{"b"}, LayerId{"F.Cu"},
                           mm(50, 38), mm(50, 42), Nanometres{250000}, true});
  flux::realize_regional_straight_candidate(board, global, blocked);
  require(!blocked.spline_candidate_available && blocked.native_scene_conflicts > 0U &&
              blocked.linear_scaffold_rejections == 1U,
          "a discrete fitting lane order must not accept a static copper collision");
  const auto diagnoses = flux::diagnose_regional_routability(blocked);
  require(!diagnoses.empty() && std::none_of(diagnoses.begin(), diagnoses.end(), [](const auto& diagnosis) {
            return diagnosis.stage == flux::RoutabilityFailureStage::spline_realization ||
                   diagnosis.stage == flux::RoutabilityFailureStage::local_pns;
          }), "straight failures must retain exact copper evidence without inventing absent PH/PNS failures");
  auto cancelled = unresolved;
  std::stop_source stop;
  stop.request_stop();
  flux::realize_regional_straight_candidate(board, global, cancelled, stop.get_token());
  require(!cancelled.spline_candidate_available, "cancelled straight realization must fail closed");
}

void straight_elbow_alternatives_preserve_exact_copper_contract() {
  for (const auto transform : {0, 1, 2, 3}) {
    for (const bool coupled : {false, true}) {
      const auto point = [transform](double x, double y) {
        if (transform & 1) x = 60.0 - x;
        return (transform & 2) ? mm(60.0 - y, x) : mm(x, y);
      };
      Board board;
      board.source_revision = "staggered-octilinear-elbows";
      board.outline = rectangle(0, 0, 60, 60);
      const LayerId layer{"F.Cu"};
      board.layers = {{layer, "F.Cu", LayerKind::copper, 0}};
      board.nets = {{NetId{"a"}, "A", 0}, {NetId{"b"}, "B", 0},
                    {NetId{"0-unrelated"}, "Unrelated", 0}};
      flux::GlobalRouteResult global;
      flux::RegionalRouteResult scene;
      for (std::size_t i = 0; i < 4U; ++i) {
        flux::RegionalRouteFragment fragment;
        const auto net_index = i < 2U ? i : i == 2U ? 0U : 2U;
        fragment.id = {board.nets[net_index].id, i == 0U ? 1U : 0U, 0U, 0U};
        fragment.layer = layer;
        fragment.start = i < 2U ? point(20.0 + i, 20.0) : point(40.0, i == 2U ? 40.0 : 50.0);
        fragment.end = i < 2U ? point(30.0 + i, 26.0) : point(50.0, i == 2U ? 40.0 : 56.0);
        if (coupled && i == 3U) {
          fragment.start = point(45.0, 35.0);
          fragment.end = point(45.0, 45.0);
        }
        // The earlier-sorting extra fragments test both filters: same-net axis-aligned copper
        // has no elbow alternative, and unrelated non-octilinear copper has no physical conflict.
        fragment.width = Nanometres::from_mm((transform & 1) ? 0.15 : 0.10);
        fragment.clearance = Nanometres::from_mm(0.10);
        fragment.explicit_width = true;
        scene.fragments.push_back(fragment);
        flux::RegionalRoutingWindow window;
        window.id = i;
        window.fragments = {fragment.id};
        window.owned_fragments = {fragment.id};
        scene.windows.push_back(window);
        flux::RegionalRouteTemplate route_template;
        route_template.id = 0U;
        route_template.fragment = fragment.id;
        route_template.routing_layer = layer;
        route_template.selected_global_topology = true;
        flux::RegionalBundleAssignment assignment;
        assignment.window_id = i;
        assignment.templates = {route_template};
        assignment.selected_templates = {0U};
        assignment.stop_reason = flux::RegionalBundleStopReason::complete;
        scene.bundle_assignments.push_back(assignment);
        global.paths.push_back({fragment.id.net, fragment.id.connection_index, {{fragment.start, layer}, {fragment.end, layer}}});
      }
      if (coupled) {
        // A second, axis-aligned conflict needs a retained template after the elbow repair.
        // Accepting that template must not silently regenerate the old colliding A/B elbows.
        auto detour = scene.bundle_assignments.back().templates.front();
        detour.id = 1U;
        detour.selected_global_topology = false;
        detour.topology_centerline = {point(45, 35), point(55, 35), point(55, 45), point(45, 45)};
        scene.bundle_assignments.back().templates.push_back(detour);
        flux::RegionalBundleSelection selection;
        selection.templates = {1U};
        scene.bundle_assignments.back().spline_candidate_shortlist.push_back(selection);
      }
      auto baseline = scene;
      flux::realize_regional_straight_candidate(board, global, baseline);
      require(baseline.straight_scene_evaluated && baseline.straight_scene_drc_violations > 0U &&
                  !baseline.spline_candidate_available,
              "fixed diagonal-first elbows must reproduce the staggered overlap");
      // Reserve alternatives are now eligible, but geometry retains its two-probe warmup:
      // the first elbow may fail while the complementary fragment probe succeeds.
      auto reserve_template = scene.bundle_assignments.front().templates.front();
      reserve_template.id = 1U;
      reserve_template.selected_global_topology = false;
      reserve_template.topology_centerline = {scene.fragments.front().start, scene.fragments.front().end};
      scene.bundle_assignments.front().templates.push_back(reserve_template);
      flux::RegionalBundleSelection unused_reserve;
      unused_reserve.templates = {1U};
      scene.bundle_assignments.front().spline_candidate_reserve.push_back(unused_reserve);
      const std::size_t budget = coupled ? 6U : 2U;
      scene.configured.max_straight_portfolio_checks = budget;
      flux::realize_regional_straight_candidate(board, global, scene);
      require(scene.spline_candidate_available && scene.native_scene_complete &&
                  scene.native_scene_conflicts == 0U && scene.straight_scene_evaluated &&
                  scene.straight_scene_drc_violations == 0U && scene.straight_scene_open_connections == 0U &&
                  scene.straight_scene_geometry_checks == 2U && scene.straight_scene_geometry_improvements == 1U &&
                  scene.straight_scene_portfolio_checks > 0U && scene.straight_scene_portfolio_checks <= budget &&
                  scene.native_realization_manifest.size() == 4U,
              "bounded complementary elbows must clear the full scene under reflection/rotation: " +
                  std::to_string(transform) + "/coupled=" + std::to_string(coupled));
      if (coupled) {
        require(scene.bundle_assignments.back().selected_templates == std::vector<std::size_t>{1U},
                "a retained template and a geometry alternative must coexist in the accepted scene");
      }
      for (const auto& route : scene.native_realization_manifest) {
        const auto fragment = std::find_if(scene.fragments.begin(), scene.fragments.end(),
            [&](const auto& item) { return item.id == route.fragment; });
        require(fragment != scene.fragments.end() && route.width == fragment->width &&
                    route.layer == fragment->layer && route.centerline.front() == fragment->start &&
                    route.centerline.back() == fragment->end,
                "elbow alternatives must preserve source width, layer and boundary endpoints");
        for (std::size_t i = 1; i < route.centerline.size(); ++i) {
          const auto dx = std::abs(route.centerline[i].x.value() - route.centerline[i - 1U].x.value());
          const auto dy = std::abs(route.centerline[i].y.value() - route.centerline[i - 1U].y.value());
          require(dx == 0 || dy == 0 || dx == dy, "elbow alternatives must remain integer octilinear copper");
        }
      }
    }
  }
}

void straight_scene_portfolio_replays_retained_window_alternative() {
  Board board;
  board.source_revision = "straight-portfolio-fixture";
  board.outline = rectangle(0.0, 0.0, 100.0, 80.0);
  const LayerId front{"F.Cu"};
  board.layers = {{front, "F.Cu", LayerKind::copper, 0U}};
  const NetId a{"portfolio-a"}, b{"portfolio-b"};
  board.nets = {{a, "A", 0}, {b, "B", 0}};
  const auto terminal = [&](std::string id, NetId net, Point position) {
    Footprint footprint;
    footprint.id = ObjectId{id};
    footprint.reference = id;
    footprint.position = position;
    footprint.courtyard = local_rectangle(1.0, 1.0);
    footprint.pads = {{ObjectId{id + "-pad"}, std::move(net), Point{}, {front}}};
    return footprint;
  };
  board.footprints = {terminal("a1", a, mm(20, 40)), terminal("a2", a, mm(80, 40)),
                      terminal("b1", b, mm(50, 20)), terminal("b2", b, mm(50, 60))};
  const auto fragment = [&](NetId net, Point start, Point end) {
    flux::RegionalRouteFragment value;
    value.id = {std::move(net), 0U, 0U, 0U};
    value.layer = front;
    value.start = start;
    value.end = end;
    value.width = Nanometres::from_mm(0.25);
    value.clearance = Nanometres::from_mm(0.2);
    return value;
  };
  const auto a_fragment = fragment(a, mm(20, 40), mm(80, 40));
  const auto b_fragment = fragment(b, mm(50, 20), mm(50, 60));
  flux::GlobalRouteResult global;
  global.paths = {{a, 0U, {{a_fragment.start, front}, {a_fragment.end, front}}},
                  {b, 0U, {{b_fragment.start, front}, {b_fragment.end, front}}}};
  flux::RegionalRouteResult regional;
  regional.fragments = {a_fragment, b_fragment};
  flux::RegionalRoutingWindow a_window;
  a_window.id = 0U;
  a_window.fragments = {a_fragment.id};
  a_window.owned_fragments = {a_fragment.id};
  flux::RegionalRoutingWindow b_window;
  b_window.id = 1U;
  b_window.fragments = {b_fragment.id};
  b_window.owned_fragments = {b_fragment.id};
  regional.windows = {a_window, b_window};
  flux::RegionalRouteTemplate a_template;
  a_template.id = 0U;
  a_template.fragment = a_fragment.id;
  a_template.routing_layer = front;
  a_template.selected_global_topology = true;
  flux::RegionalRouteTemplate b_crossing;
  b_crossing.id = 0U;
  b_crossing.fragment = b_fragment.id;
  b_crossing.routing_layer = front;
  b_crossing.selected_global_topology = true;
  auto b_dogleg = b_crossing;
  b_dogleg.id = 1U;
  b_dogleg.selected_global_topology = false;
  b_dogleg.topology_centerline = {b_fragment.start, mm(60, 30), mm(85, 30),
                                 mm(85, 50), mm(75, 60), b_fragment.end};
  flux::RegionalBundleAssignment a_assignment;
  a_assignment.window_id = 0U;
  a_assignment.templates = {a_template};
  a_assignment.selected_templates = {0U};
  a_assignment.stop_reason = flux::RegionalBundleStopReason::complete;
  flux::RegionalBundleAssignment b_assignment;
  b_assignment.window_id = 1U;
  b_assignment.templates = {b_crossing, b_dogleg};
  b_assignment.selected_templates = {0U};
  b_assignment.selected_template_conflicts = 1U;
  b_assignment.stop_reason = flux::RegionalBundleStopReason::complete;
  flux::RegionalBundleSelection dogleg_selection;
  dogleg_selection.templates = {1U};
  b_assignment.spline_candidate_shortlist = {dogleg_selection};
  regional.bundle_assignments = {a_assignment, b_assignment};
  const auto retained = regional;
  flux::realize_regional_straight_candidate(board, global, regional);
  require(!regional.spline_candidate_available && regional.straight_scene_evaluated &&
              regional.straight_scene_portfolio_checks == 0U,
          "the default straight transaction must retain selected crossing copper without replaying "
          "a portfolio alternative");
  auto enabled = retained;
  enabled.configured.max_straight_portfolio_checks = 1U;
  flux::realize_regional_straight_candidate(board, global, enabled);
  require(enabled.spline_candidate_available && enabled.straight_scene_portfolio_checks == 1U &&
              enabled.straight_scene_portfolio_improvements == 1U &&
              enabled.straight_scene_drc_violations == 0U &&
              enabled.straight_scene_open_connections == 0U &&
              enabled.bundle_assignments[1].selected_templates == dogleg_selection.templates &&
              std::all_of(enabled.native_realization_manifest.begin(),
                          enabled.native_realization_manifest.end(), [](const auto& route) {
                            return route.straight_realization && route.control_points.empty();
                          }),
          "one retained B dogleg must repair the complete A/B straight scene without changing "
          "A into a curved realization");
  require(std::none_of(enabled.local_routability_diagnoses.begin(),
                      enabled.local_routability_diagnoses.end(), [](const auto& diagnosis) {
                        return diagnosis.resource.stable_id.starts_with("straight-scene:") ||
                               diagnosis.resource.stable_id.starts_with("native-scene:");
                      }),
          "a repaired scene must not retain discarded copper conflicts for upstream feedback");
  require(enabled.validated_copper_scene.data != nullptr,
          "a certified regional scene must retain exact immutable copper");
  flux::RoutingPlanResult scene_plan;
  scene_plan.coarse_global = global;
  scene_plan.coarse_global.stop_reason = flux::GlobalRouterStopReason::complete;
  scene_plan.regional = enabled;
  scene_plan.paths = enabled.spline_candidate_paths;
  scene_plan.stop_reason = flux::RoutingPlanStopReason::complete;
  flux::DetailedRouterConfig scene_config;
  scene_config.restrict_straight_track_angles = true;
  scene_config.continuous_refinement = false;
  scene_config.pull_tight_45_degrees = false;
  const auto handed_off = flux::materialize_detailed_route(board, scene_plan, scene_config);
  require(handed_off.complete() && handed_off.reused_validated_regional_scene &&
              handed_off.candidate->tracks == enabled.validated_copper_scene.data->scene.tracks &&
              handed_off.candidate->vias == enabled.validated_copper_scene.data->scene.vias,
          "detailed handoff must reuse exact regional primitives and pass final DRC/connectivity");
  auto transformed_config = scene_config;
  transformed_config.pull_tight_45_degrees = true;
  const auto transformed = flux::materialize_detailed_route(board, scene_plan, transformed_config);
  require(!transformed.reused_validated_regional_scene,
          "requested geometry transforms must decline cached scene reuse");
  auto edited_source = board;
  edited_source.rules.default_clearance = Nanometres::from_mm(0.3);
  const auto stale_source = flux::materialize_detailed_route(edited_source, scene_plan, scene_config);
  require(!stale_source.reused_validated_regional_scene,
          "source edits under an unchanged revision must invalidate regional scene reuse");
  auto edited_routes = scene_plan;
  edited_routes.regional.spline_candidate_paths.front().points.front().position = mm(21, 40);
  const auto stale_routes = flux::materialize_detailed_route(board, edited_routes, scene_config);
  require(!stale_routes.reused_validated_regional_scene,
          "selected-route edits must invalidate regional scene reuse");
  auto tampered = scene_plan;
  auto tampered_scene = std::make_shared<flux::RegionalCopperSceneData>(
      *enabled.validated_copper_scene.data);
  auto illegal = tampered_scene->scene.tracks.front();
  illegal.id = ObjectId{"injected-conflicting-track"};
  illegal.net = b;
  tampered_scene->scene.tracks.push_back(illegal);
  tampered.regional.validated_copper_scene.data = tampered_scene;
  const auto rechecked = flux::materialize_detailed_route(board, tampered, scene_config);
  require(rechecked.reused_validated_regional_scene && !rechecked.complete() &&
              !rechecked.copper_drc.ok(),
          "stored scene evidence must never bypass final represented-copper DRC");
  auto rejected_then_legal = retained;
  auto& alternatives = rejected_then_legal.bundle_assignments[1];
  auto first_crossing = b_dogleg;
  first_crossing.id = 2U;
  first_crossing.topology_centerline = {b_fragment.start, mm(60, 30),
                                       mm(60, 50), b_fragment.end};
  auto second_crossing = b_dogleg;
  second_crossing.id = 3U;
  second_crossing.topology_centerline = {b_fragment.start, mm(40, 30),
                                        mm(40, 50), b_fragment.end};
  alternatives.templates.push_back(first_crossing);
  alternatives.templates.push_back(second_crossing);
  auto first_rejected = dogleg_selection;
  first_rejected.templates = {2U};
  auto second_rejected = dogleg_selection;
  second_rejected.templates = {3U};
  alternatives.spline_candidate_shortlist = {first_rejected, second_rejected, dogleg_selection};
  rejected_then_legal.configured.max_straight_portfolio_checks = 3U;
  flux::realize_regional_straight_candidate(board, global, rejected_then_legal);
  require(rejected_then_legal.spline_candidate_available &&
              rejected_then_legal.straight_scene_preview_checks == 3U &&
              rejected_then_legal.straight_scene_portfolio_checks == 1U &&
              rejected_then_legal.straight_scene_portfolio_improvements == 1U &&
              rejected_then_legal.straight_scene_trials == 2U &&
              rejected_then_legal.linear_scaffold_checks == 2U &&
              rejected_then_legal.linear_scaffold_rejections == 1U,
          "physical previews must prioritize the compatible bend before full-scene proof "
          "while retaining exact accounting and the original rejection");
  auto reserve_only = retained;
  reserve_only.bundle_assignments[1].spline_candidate_shortlist.clear();
  reserve_only.bundle_assignments[1].spline_candidate_reserve = {dogleg_selection};
  reserve_only.configured.max_straight_portfolio_checks = 1U;
  flux::realize_regional_straight_candidate(board, global, reserve_only);
  require(reserve_only.spline_candidate_available && reserve_only.straight_scene_reserve_candidates == 1U,
          "an empty shortlist must not hide a physically useful reserve candidate");
  auto reserve_escape = retained;
  reserve_escape.bundle_assignments[1].templates = {b_crossing, b_dogleg, first_crossing, second_crossing};
  reserve_escape.bundle_assignments[1].spline_candidate_shortlist = {first_rejected, second_rejected};
  reserve_escape.bundle_assignments[1].spline_candidate_reserve = {dogleg_selection};
  reserve_escape.configured.max_straight_portfolio_checks = 2U;
  flux::realize_regional_straight_candidate(board, global, reserve_escape);
  require(reserve_escape.spline_candidate_available && reserve_escape.straight_scene_reserve_candidates == 1U &&
              reserve_escape.straight_scene_portfolio_checks <= 2U,
          "a full shortlist must not starve the retained physical escape reserve");
  auto cancelled = retained;
  cancelled.configured.max_straight_portfolio_checks = 1U;
  std::stop_source stop;
  stop.request_stop();
  flux::realize_regional_straight_candidate(board, global, cancelled, stop.get_token());
  require(!cancelled.spline_candidate_available && cancelled.straight_scene_portfolio_checks == 0U,
          "a cancelled portfolio replay must not publish a partially rebuilt straight scene");

  // Both retained moves are necessary. A's lowered lane still crosses the original B;
  // B's detour crosses the original A twice. Together they are disjoint. A greedy
  // one-window improvement cannot leave this physical local minimum.
  auto coupled = retained;
  auto a_lowered = a_template;
  a_lowered.id = 1U;
  a_lowered.selected_global_topology = false;
  a_lowered.topology_centerline = {a_fragment.start, mm(20, 30), mm(80, 30), a_fragment.end};
  auto b_around = b_dogleg;
  b_around.topology_centerline = {b_fragment.start, mm(10, 20), mm(10, 50), mm(40, 50),
                                 mm(40, 35), mm(60, 35), mm(60, 60), b_fragment.end};
  coupled.bundle_assignments[0].templates.push_back(a_lowered);
  coupled.bundle_assignments[0].spline_candidate_shortlist = {dogleg_selection};
  coupled.bundle_assignments[1].templates[1] = b_around;
  auto only_a = coupled;
  only_a.bundle_assignments[1].spline_candidate_shortlist.clear();
  only_a.configured.max_straight_portfolio_checks = 3U;
  flux::realize_regional_straight_candidate(board, global, only_a);
  require(!only_a.spline_candidate_available && only_a.straight_scene_portfolio_improvements == 0U,
          "the lowered A lane alone must not improve the crossing scene");
  auto only_b = coupled;
  only_b.bundle_assignments[0].spline_candidate_shortlist.clear();
  only_b.configured.max_straight_portfolio_checks = 3U;
  flux::realize_regional_straight_candidate(board, global, only_b);
  require(!only_b.spline_candidate_available && only_b.straight_scene_portfolio_improvements == 0U &&
              only_b.straight_scene_portfolio_checks == 1U && only_b.linear_scaffold_checks >= 2U,
          "the B detour alone must not improve the crossing scene");
  auto too_small = coupled;
  too_small.configured.max_straight_portfolio_checks = 1U;
  flux::realize_regional_straight_candidate(board, global, too_small);
  require(!too_small.spline_candidate_available && too_small.straight_scene_portfolio_checks == 1U,
          "coordinated search must obey the shared exact-check bound");
  coupled.configured.max_straight_portfolio_checks = 3U;
  flux::realize_regional_straight_candidate(board, global, coupled);
  require(coupled.spline_candidate_available,
          "paired retained moves must find legal copper when neither single move improves it");
  require(coupled.native_scene_complete &&
              coupled.native_scene_conflicts == 0U && coupled.straight_scene_drc_violations == 0U &&
              coupled.straight_scene_open_connections == 0U &&
              coupled.straight_scene_joint_portfolio_checks == 1U &&
              coupled.straight_scene_preview_checks == 3U &&
              coupled.straight_scene_portfolio_checks == 1U &&
              coupled.straight_scene_portfolio_improvements == 1U &&
              coupled.linear_scaffold_checks == 2U && coupled.linear_scaffold_rejections == 1U &&
              coupled.bundle_assignments[0].selected_templates == dogleg_selection.templates &&
              coupled.bundle_assignments[1].selected_templates == dogleg_selection.templates,
          "a paired transaction must escape the one-window local minimum with exact clean copper");
}

void regional_repair_scheduler_preserves_exploration() {
  std::array<flux::RegionalRepairFamilyObservation, 4U> observations{};
  for (auto& entry : observations) entry.available = true;
  flux::RegionalRepairSchedulerState state;
  for (std::uint8_t i = 0; i < 4U; ++i) {
    const auto decision = flux::schedule_regional_repair_family(observations, state);
    require(decision.family == static_cast<flux::RegionalRepairFamily>(i) &&
                decision.reason == flux::RegionalRepairScheduleReason::warmup,
            "each eligible repair family must receive a warmup attempt");
    ++observations[i].attempts;
  }
  observations[0] = {true, 20U, 20U, 5U};
  observations[1] = {true, 1U, 1U, 10U};
  observations[2] = {true, 20U, 0U, 1U};
  observations[3] = {true, 20U, 0U, 1U};
  state.decisions = 4U;
  require(flux::schedule_regional_repair_family(observations, state).family ==
              flux::RegionalRepairFamily::geometry,
          "productive families must receive work according to improvement per deterministic work unit");
  state.decisions = 7U;
  const auto exploration = flux::schedule_regional_repair_family(observations, state);
  require(exploration.family == flux::RegionalRepairFamily::single &&
              exploration.reason == flux::RegionalRepairScheduleReason::exploration,
          "low-yield families must remain eligible through bounded exploration");
  for (auto& entry : observations) entry.accepted_physical_error_reduction = 0U;
  std::array<bool, 4U> seen{};
  for (std::size_t i = 0; i < 16U; ++i) {
    auto decision = flux::schedule_regional_repair_family(observations, state);
    require(decision.family.has_value(), "available repair families must remain schedulable");
    const auto index = flux::regional_repair_family_index(*decision.family);
    seen[index] = true;
    ++observations[index].attempts;
  }
  require(std::all_of(seen.begin(), seen.end(), [](bool value) { return value; }),
          "zero-yield exploration must not starve a repair family");
}

void regional_access_cache_is_bounded_without_changing_solutions() {
  Board board;
  const LayerId layer{"F.Cu"};
  board.layers = {{layer, "F.Cu", LayerKind::copper, 0U}};
  board.outline = rectangle(0, 0, 40, 40);
  flux::RegionalRouteResult regional;
  flux::RegionalRoutingWindow window;
  std::vector<flux::RegionalRouteTemplate> templates;
  std::vector<std::vector<std::size_t>> domains;
  for (std::size_t i = 0; i < 13U; ++i) {
    const NetId net{"cache-" + std::to_string(i)};
    board.nets.push_back({net, net.value(), 0});
    flux::RegionalRouteFragment fragment;
    fragment.id = {net, 0U, 0U, 0U};
    fragment.layer = layer;
    fragment.start = mm(5, static_cast<double>(i + 1U));
    fragment.end = mm(15, static_cast<double>(i + 1U));
    fragment.width = Nanometres::from_mm(0.25);
    fragment.clearance = Nanometres::from_mm(0.2);
    regional.fragments.push_back(fragment);
    window.fragments.push_back(fragment.id);
    window.owned_fragments.push_back(fragment.id);
    std::vector<std::size_t> domain;
    for (std::size_t option = 0; option < 2U; ++option) {
      flux::RegionalRouteTemplate route;
      route.id = templates.size();
      route.fragment = fragment.id;
      route.routing_layer = layer;
      domain.push_back(route.id);
      templates.push_back(route);
    }
    domains.push_back(std::move(domain));
  }
  std::sort(window.owned_fragments.begin(), window.owned_fragments.end());
  const auto choices = flux::enumerate_regional_bundle_choices(
      board, regional, window, templates, domains, 20'000U);
  require(!choices.exhausted && choices.choices.size() == 8'192U &&
              choices.access_selection_cache_entries_peak == 4'096U &&
              choices.access_selection_cache_capacity_skipped_insertions > 0U,
          "bounded access memoization must retain every feasible choice after its storage cap");
}

void regional_performance_samples_preserve_submicrosecond_work() {
  flux::RegionalPerformanceMicroseconds measured;
  for (std::size_t i = 0; i < 4'000U; ++i) measured.add_nanoseconds(250U);
  require(static_cast<std::uint64_t>(measured) == 1'000U,
          "short measurements must accumulate before converting to microseconds");
  flux::RegionalPerformanceMicroseconds first, second;
  first.add_nanoseconds(750U);
  second.add_nanoseconds(750U);
  first += second;
  require(static_cast<std::uint64_t>(first) == 1U,
          "aggregation must preserve fractional microseconds across attempts");
  first += std::uint64_t{2U};
  require(static_cast<std::uint64_t>(first) == 3U,
          "legacy microsecond increments must retain their units");
  require(first == measured,
          "wall-clock observations must remain excluded from semantic route equality");
}

void fixed_template_copper_pruning_is_conservative() {
  Board board;
  board.source_revision = "fixed-template-compatibility";
  board.outline = rectangle(0.0, 0.0, 40.0, 40.0);
  const LayerId front{"F.Cu"};
  const NetId a{"fixed-a"}, b{"fixed-b"};
  board.layers = {{front, "F.Cu", LayerKind::copper, 0U}};
  board.nets = {{a, "A", 0}, {b, "B", 0}};
  flux::RegionalRouteResult regional;
  regional.configured.enable_straight_realization = true;
  regional.configured.enable_spline_realization = false;
  flux::RegionalRoutingWindow window;
  std::vector<flux::RegionalRouteTemplate> templates;
  for (std::size_t i = 0; i < 2; ++i) {
    flux::RegionalRouteFragment fragment;
    fragment.id = {i == 0 ? a : b, 0U, 0U, 0U};
    fragment.layer = front;
    fragment.start = i == 0 ? mm(10,20) : mm(20,10);
    fragment.end = i == 0 ? mm(30,20) : mm(20,30);
    fragment.width = Nanometres::from_mm(0.25);
    fragment.clearance = Nanometres::from_mm(0.2);
    regional.fragments.push_back(fragment);
    window.fragments.push_back(fragment.id);
    window.owned_fragments.push_back(fragment.id);
    flux::RegionalRouteTemplate route;
    route.id = i;
    route.selected_global_topology = false;
    route.fragment = fragment.id;
    route.routing_layer = front;
    route.topology_centerline = {fragment.start, fragment.end};
    templates.push_back(route);
  }
  auto legal = templates[1];
  legal.id = 2U;
  legal.topology_centerline = {mm(20,10), mm(35,10), mm(35,30), mm(20,30)};
  templates.push_back(legal);
  const std::vector<std::vector<std::size_t>> domains{{0U}, {1U,2U}};
  const auto pruned = flux::enumerate_regional_bundle_choices(board, regional, window,
                                                             templates, domains, 128U);
  require(pruned.fixed_geometry_compatibility_checks > 0U &&
              pruned.fixed_geometry_compatibility_rejections > 0U &&
              !pruned.choices.empty() &&
              std::all_of(pruned.choices.begin(), pruned.choices.end(), [](const auto& choice) {
                return choice == std::vector<std::size_t>{0U,1U};
              }),
          "fixed crossing copper must be rejected before access search while retaining the legal detour");
  auto deformable = regional;
  deformable.configured.enable_straight_realization = false;
  deformable.configured.enable_spline_realization = true;
  const auto unknown = flux::enumerate_regional_bundle_choices(board, deformable, window,
                                                              templates, domains, 128U);
  require(unknown.fixed_geometry_compatibility_checks == 0U,
          "potentially deformable curved templates cannot be rejected as fixed copper");
  auto non_octilinear = templates;
  non_octilinear[0].topology_centerline = {mm(10,20), mm(30,25)};
  const auto elbow_unknown = flux::enumerate_regional_bundle_choices(
      board, regional, window, non_octilinear, domains, 128U);
  require(elbow_unknown.fixed_geometry_compatibility_checks == 0U,
          "a chord that will gain an elbow is not a fixed physical realization");
  auto boundary = templates;
  boundary.resize(2U);
  boundary[0].topology_centerline = {mm(10,10), mm(30,10)};
  boundary[1].topology_centerline = {mm(10,10.45), mm(30,10.45)};
  const auto touching = flux::enumerate_regional_bundle_choices(
      board, regional, window, boundary, {{0U},{1U}}, 128U);
  require(!touching.choices.empty() && touching.fixed_geometry_compatibility_rejections == 0U,
          "exact legal clearance equality must remain feasible");
}

void straight_joint_three_window_repair_escapes_pairwise_minimum() {
  Board board;
  board.source_revision = "three-window-cycle";
  board.outline = rectangle(0.0, 0.0, 100.0, 80.0);
  const std::array<LayerId, 3> layers{LayerId{"F.Cu"}, LayerId{"In1.Cu"}, LayerId{"B.Cu"}};
  const std::array<NetId, 3> nets{NetId{"cycle-a"}, NetId{"cycle-b"}, NetId{"cycle-c"}};
  for (std::size_t i = 0; i < 3; ++i) {
    board.layers.push_back({layers[i], layers[i].value(), LayerKind::copper, static_cast<std::uint32_t>(i)});
    board.nets.push_back({nets[i], nets[i].value(), 0});
  }
  flux::GlobalRouteResult global;
  flux::RegionalRouteResult seed;
  for (std::size_t net = 0; net < 3; ++net) {
    flux::RegionalRoutingWindow window;
    window.id = net;
    flux::RegionalBundleAssignment assignment;
    assignment.window_id = net;
    assignment.stop_reason = flux::RegionalBundleStopReason::complete;
    for (std::size_t part = 0; part < 2; ++part) {
      // Three copies of the two-window trap form a cycle. In each layer the horizontal
      // alternate alone ties the original, the vertical alternate worsens it, and both clear it.
      const auto layer = layers[part == 0 ? net : (net + 2) % 3];
      flux::RegionalRouteFragment fragment;
      fragment.id = {nets[net], part, 0U, 0U};
      fragment.layer = layer;
      fragment.start = part == 0 ? mm(20, 40) : mm(50, 20);
      fragment.end = part == 0 ? mm(80, 40) : mm(50, 60);
      fragment.width = Nanometres::from_mm(0.25);
      fragment.clearance = Nanometres::from_mm(0.2);
      seed.fragments.push_back(fragment);
      window.fragments.push_back(fragment.id);
      window.owned_fragments.push_back(fragment.id);
      global.paths.push_back({nets[net], part, {{fragment.start, layer}, {fragment.end, layer}}});
      flux::RegionalRouteTemplate original;
      original.id = part;
      original.fragment = fragment.id;
      original.routing_layer = layer;
      original.selected_global_topology = true;
      assignment.templates.push_back(original);
    }
    for (std::size_t part = 0; part < 2; ++part) {
      auto alternative = assignment.templates[part];
      alternative.id = part + 2;
      alternative.selected_global_topology = false;
      alternative.topology_centerline = part == 0
          ? std::vector<Point>{mm(20,40), mm(20,30), mm(80,30), mm(80,40)}
          : std::vector<Point>{mm(50,20), mm(10,20), mm(10,50), mm(40,50),
                               mm(40,35), mm(60,35), mm(60,60), mm(50,60)};
      assignment.templates.push_back(alternative);
    }
    assignment.selected_templates = {0U, 1U};
    flux::RegionalBundleSelection alternative;
    alternative.templates = {2U, 3U};
    assignment.spline_candidate_shortlist = {alternative};
    seed.windows.push_back(std::move(window));
    seed.bundle_assignments.push_back(std::move(assignment));
  }
  auto pairwise = seed;
  pairwise.configured.max_straight_portfolio_checks = 12U;
  pairwise.configured.max_straight_joint_windows = 2U;
  flux::realize_regional_straight_candidate(board, global, pairwise);
  require(!pairwise.spline_candidate_available && pairwise.straight_scene_portfolio_improvements == 0U,
          "no single or paired move may improve the cyclic three-window fixture");
  auto coordinated = seed;
  coordinated.configured.max_straight_portfolio_checks = 12U;
  flux::realize_regional_straight_candidate(board, global, coordinated);
  require(coordinated.spline_candidate_available && coordinated.native_scene_complete &&
              coordinated.native_scene_conflicts == 0U &&
              coordinated.straight_scene_drc_violations == 0U &&
              coordinated.straight_scene_open_connections == 0U &&
              coordinated.straight_scene_multi_window_checks > 0U &&
              coordinated.straight_scene_multi_window_improvements == 1U &&
              coordinated.straight_scene_portfolio_checks <= 12U,
          "one connected three-window transaction must escape the pairwise minimum with legal copper");
  require(coordinated.straight_scene_search_outcome ==
              flux::RegionalStraightSearchOutcome::found_legal_candidate,
          "a fully validated joint candidate must report physical success");
  for (const auto& assignment : coordinated.bundle_assignments)
    require(assignment.selected_templates == std::vector<std::size_t>{2U, 3U},
            "the joint transaction must atomically retain all three topology changes");
  auto impossible = seed;
  for (auto& assignment : impossible.bundle_assignments) {
    assignment.templates[2].topology_centerline = {mm(20,40), mm(80,40)};
    assignment.templates[3].topology_centerline = {mm(50,20), mm(50,60)};
  }
  impossible.configured.max_straight_portfolio_checks = 12U;
  flux::realize_regional_straight_candidate(board, global, impossible);
  require(!impossible.spline_candidate_available && impossible.straight_scene_pool_complete &&
              impossible.straight_scene_pool_states == 8U &&
              impossible.straight_scene_search_outcome ==
                  flux::RegionalStraightSearchOutcome::retained_fixed_pool_exhausted,
          "all eight fixed retained combinations must be checked before declaring pool exhaustion");
  auto bounded = seed;
  bounded.configured.max_straight_portfolio_checks = 1U;
  flux::realize_regional_straight_candidate(board, global, bounded);
  require(bounded.straight_scene_portfolio_checks <= 1U,
          "multi-window repair must never exceed the existing exact-check budget");
  require(bounded.straight_scene_search_outcome ==
              flux::RegionalStraightSearchOutcome::budget_or_scope_unknown,
          "an exhausted budget must not be described as an infeasible candidate pool");
}

void straight_regional_layer_adapters_preserve_connectivity() {
  auto board = two_pad_board();
  board.footprints.back().pads.front().layers = {LayerId{"B.Cu"}};
  const auto net = board.nets.front().id;
  const LayerId front{"F.Cu"}, back{"B.Cu"};
  flux::GlobalRouteResult global;
  flux::GlobalRoutePath path;
  path.net = net;
  path.points = {{mm(20, 40), front}, {mm(40, 40), front},
                 {mm(40, 40), back}, {mm(80, 40), back}};
  global.paths = {path};
  flux::RegionalRouteResult regional;
  flux::RegionalRouteFragment fragment;
  fragment.id = {net, 0U, 0U, 0U};
  fragment.layer = front;
  fragment.start = mm(20, 40);
  fragment.end = mm(40, 40);
  fragment.width = Nanometres{250000};
  fragment.clearance = Nanometres{200000};
  fragment.begins_at_terminal = true;
  regional.fragments = {fragment};
  flux::RegionalRoutingWindow window;
  window.id = 0U;
  window.fragments = {fragment.id};
  window.owned_fragments = {fragment.id};
  regional.windows = {window};
  flux::RegionalViaChoice entry;
  entry.id = 0U;
  entry.region_id = 0U;
  entry.net = net;
  entry.site = mm(35, 40);
  entry.span = {front, back};
  entry.diameter = Nanometres{600000};
  entry.drill = Nanometres{300000};
  auto exit = entry;
  exit.id = 1U;
  exit.region_id = 1U;
  exit.site = mm(40, 40);
  exit.selected_global_site = true;
  regional.via_choices = {entry, exit};
  flux::RegionalRouteTemplate route_template;
  route_template.fragment = fragment.id;
  route_template.routing_layer = back;
  route_template.selected_global_topology = false;
  route_template.entry_via_choice = entry.id;
  route_template.exit_via_choice = exit.id;
  route_template.topology_centerline = {entry.site, exit.site};
  flux::RegionalBundleAssignment assignment;
  assignment.window_id = 0U;
  assignment.templates = {route_template};
  assignment.selected_templates = {0U};
  assignment.selected_via_choices = {entry.id, exit.id};
  assignment.stop_reason = flux::RegionalBundleStopReason::complete;
  regional.bundle_assignments = {assignment};
  const auto unresolved = regional;
  flux::realize_regional_straight_candidate(board, global, regional);
  require(regional.spline_candidate_available && regional.straight_collapsed_via_adapters == 1U &&
              regional.planned_via_transitions == 1U && regional.straight_scene_open_connections == 0U,
          "an alternate layer joining the source continuation must form one legal via: " +
              regional.spline_candidate_diagnostic);
  // A front-only branch at the removed return adapter still needs real copper. Eliminating a
  // redundant geometric transition cannot authorize dropping that electrical obligation.
  auto branch = board.footprints.front();
  branch.id = ObjectId{"branch"};
  branch.reference = "branch";
  branch.position = mm(40, 40);
  branch.pads.front().id = ObjectId{"branch-pad"};
  board.footprints.push_back(branch);
  auto disconnected = unresolved;
  flux::realize_regional_straight_candidate(board, global, disconnected);
  require(!disconnected.spline_candidate_available && disconnected.straight_scene_open_connections > 0U,
          "removing an adapter must never silently disconnect an intermediate-layer branch");
}

void regional_fragment_compression_preserves_tree_junctions() {
  auto board = two_pad_board();
  auto third = board.footprints.front();
  third.id = ObjectId{"branch"};
  third.reference = "branch";
  third.position = mm(50, 60);
  third.pads.front().id = ObjectId{"branch-pad"};
  board.footprints.push_back(third);
  const auto net = board.nets.front().id;
  const LayerId front{"F.Cu"};
  flux::GlobalRouteResult global;
  flux::GlobalRoutePath trunk;
  trunk.net = net;
  trunk.points = {{mm(20, 40), front}, {mm(50, 40), front}, {mm(80, 40), front}};
  auto branch = trunk;
  branch.connection_index = 1U;
  branch.points = {{mm(50, 60), front}, {mm(50, 40), front}};
  global.paths = {trunk, branch};
  flux::RegionalRouterConfig config;
  config.enable_spline_realization = false;
  auto regional = flux::analyze_regional_conflicts(board, global, config);
  std::vector<flux::RegionalRouteFragment> trunk_fragments;
  for (const auto& fragment : regional.fragments)
    if (fragment.id.connection_index == 0U) trunk_fragments.push_back(fragment);
  require(trunk_fragments.size() == 2U && trunk_fragments[0].end == mm(50, 40) &&
              trunk_fragments[1].start == mm(50, 40) &&
              trunk_fragments[0].last_source_segment == 0U &&
              trunk_fragments[1].first_source_segment == 1U,
          "collinear compression must retain a same-net connection endpoint and source-edge identity");
  flux::RegionalRoutingWindow window;
  window.id = 0U;
  flux::RegionalBundleAssignment assignment;
  assignment.window_id = 0U;
  assignment.stop_reason = flux::RegionalBundleStopReason::complete;
  for (std::size_t i = 0; i < trunk_fragments.size(); ++i) {
    const auto& fragment = trunk_fragments[i];
    window.fragments.push_back(fragment.id);
    window.owned_fragments.push_back(fragment.id);
    flux::RegionalRouteTemplate route_template;
    route_template.id = i;
    route_template.fragment = fragment.id;
    route_template.routing_layer = front;
    assignment.templates.push_back(route_template);
    assignment.selected_templates.push_back(i);
    const auto x = i == 0U ? 35.0 : 65.0;
    assignment.packed_cross_sections.push_back(flux::pack_regional_cross_section(
        regional.fragments, {fragment.id}, front, mm(x, 39.6), mm(x, 41.6), Nanometres{2000000}));
  }
  regional.windows = {window};
  regional.bundle_assignments = {assignment};
  flux::realize_regional_straight_candidate(board, global, regional);
  require(regional.spline_candidate_available && regional.straight_scene_open_connections == 0U,
          "moving both sides of a tree trunk to new lanes must preserve the fixed branch contact: " +
              regional.spline_candidate_diagnostic);
}

void regional_capacity_tracks_longitudinal_demand() {
  // Duplicate tree branches are two connection owners but one copper lane.
  {
    auto board = two_pad_board();
    board.footprints.clear();
    board.nets.clear();
    flux::RegionalRouteResult regional;
    flux::RegionalRoutingWindow window;
    flux::RegionalChannel channel;
    channel.id = 7;
    channel.layer = LayerId{"F.Cu"};
    flux::RegionalCrossSection section;
    section.first_wall = mm(11, 39.4);
    section.second_wall = mm(11, 40.6);
    section.usable_width = Nanometres{1200000};
    channel.cross_sections = {section};
    std::vector<flux::RegionalRouteTemplate> templates;
    for (std::size_t i = 0; i < 3; ++i) {
      const NetId net{i < 2 ? "tree" : "other"};
      if (i != 1) board.nets.push_back({net, net.value(), 0});
      flux::RegionalRouteFragment f;
      f.id = {net, i, 0, 0};
      f.layer = channel.layer;
      f.start = mm(10, 40);
      f.end = mm(12, 40);
      f.width = Nanometres{250000};
      f.clearance = Nanometres{200000};
      regional.fragments.push_back(f);
      window.fragments.push_back(f.id);
      window.owned_fragments.push_back(f.id);
      channel.occupants.push_back(f.id);
      flux::RegionalRouteTemplate t;
      t.id = i;
      t.fragment = f.id;
      t.routing_layer = channel.layer;
      t.channel_sequence = {channel.id};
      t.layer_sequence = {channel.layer};
      templates.push_back(t);
    }
    window.nets = {NetId{"tree"}, NetId{"other"}};
    regional.channels = {channel};
    for (const bool forward : {false, true}) {
      regional.configured.enable_bundle_resource_forward_checking = forward;
      regional.configured.shared_channel_copper = false;
      require(flux::enumerate_regional_bundle_choices(board, regional, window, templates,
                  {{0}, {1}, {2}}, 256, forward).choices.empty(),
              "legacy demand must reproduce double charging of an identical tree branch");
      regional.configured.shared_channel_copper = true;
      require(flux::enumerate_regional_bundle_choices(board, regional, window, templates,
                  {{0}, {1}, {2}}, 256, forward).choices.size() == 1,
              "capacity and resource forward checking must share exact same-net copper");
      auto distinct = regional;
      distinct.fragments[1].start.y = Nanometres{40000001};
      require(flux::enumerate_regional_bundle_choices(board, distinct, window, templates,
                  {{0}, {1}, {2}}, 256, forward).choices.empty(),
              "independent same-net branches must retain their own physical demand");
      auto diverse = templates;
      auto alternate = templates.front();
      alternate.id = diverse.size();
      alternate.selected_global_topology = false;
      alternate.topology_centerline = {mm(10, 40.1), mm(12, 40.1)};
      diverse.push_back(std::move(alternate));
      for (const auto& domain : {std::vector<std::size_t>{0, 3},
                                 std::vector<std::size_t>{3, 0}}) {
        const auto recovered = flux::enumerate_regional_bundle_choices(board, regional, window,
            diverse, {domain, {1}, {2}}, 256, forward);
        require(recovered.choices.size() == 1 && !recovered.exhausted,
                "backtracking must restore shared occupancy for both alternative orders");
      }
    }
    flux::GlobalRouteResult global;
    for (const auto& f : regional.fragments) {
      flux::GlobalRoutePath path;
      path.net = f.id.net;
      path.connection_index = f.id.connection_index;
      auto start = f.start;
      auto end = f.end;
      if (path.net == NetId{"other"}) {
        start.y = start.y + Nanometres{100000};
        end.y = end.y + Nanometres{100000};
      }
      path.points = {{start, f.layer}, {end, f.layer}};
      global.paths.push_back(std::move(path));
    }
    flux::RegionalRouterConfig config;
    config.shared_channel_copper = true;
    config.enable_spline_realization = false;
    config.enable_xpbd_relaxation = false;
    config.enable_xpbd_feedback = false;
    config.max_alternative_template_checks = 1;
    config.max_joint_repair_windows = 1;
    config.max_same_layer_detour_checks = 0;
    config.max_confined_anisotropic_erosion_checks = 0;
    config.physical_conflict_witnesses =
        flux::probe_quantized_copper_feasibility(board, global).diagnoses;
    require(!config.physical_conflict_witnesses.empty(),
            "shared-lane fixture must provide exact conflicts to regional scope construction");
    const auto packed = flux::analyze_regional_conflicts(board, global, config);
    require(packed.candidate_pipeline_stats.access_selection_cache_hits > 0U &&
                packed.candidate_pipeline_stats.access_selection_cache_hits ==
                    packed.candidate_pipeline_stats.access_selection_cache_avoided_calls &&
                packed.candidate_pipeline_stats.access_selection_cache_entries_peak <= 4096U,
            "forward-checked access leaves must be reused within the unchanged window context");
    bool saw_shared_lane = false;
    for (const auto& assignment : packed.bundle_assignments) {
      for (const auto& section : assignment.packed_cross_sections) {
        std::vector<std::int64_t> offsets;
        for (const auto& p : section.placements)
          if (p.fragment.net == NetId{"tree"}) offsets.push_back(p.center_offset_nm);
        if (offsets.size() < 2) continue;
        saw_shared_lane = true;
        require(std::all_of(offsets.begin(), offsets.end(), [&](const auto offset) {
                  return offset == offsets.front();
                }), "identical branch owners must receive the same physical lane target");
      }
    }
    require(saw_shared_lane && packed.fragments.size() == 3,
            "shared lane packing must retain both original connection owners: fragments=" +
                std::to_string(packed.fragments.size()) + ", windows=" +
                std::to_string(packed.windows.size()) + ", assignments=" +
                std::to_string(packed.bundle_assignments.size()) + ", channels=" +
                std::to_string(packed.channels.size()) + "; " + packed.diagnostic);
  }
  for (const bool mirror : {false, true}) {
    auto board = two_pad_board();
    board.footprints.clear();
    board.nets.clear();
    flux::RegionalRouteResult regional;
    flux::RegionalRoutingWindow window;
    flux::RegionalChannel channel;
    channel.id = 7U;
    channel.layer = LayerId{"F.Cu"};
    std::vector<flux::RegionalRouteTemplate> templates;
    std::vector<std::vector<std::size_t>> domains;
    for (std::size_t i = 0; i < 4U; ++i) {
      const NetId net{"longitudinal-" + std::to_string(i)};
      board.nets.push_back({net, net.value(), 0});
      flux::RegionalRouteFragment fragment;
      fragment.id = {net, 0U, 0U, 0U};
      fragment.layer = channel.layer;
      fragment.start = mm(10.0 + 5.0 * i, 40.0);
      fragment.end = mm(12.0 + 5.0 * i, 40.0);
      fragment.width = Nanometres{250000};
      fragment.clearance = Nanometres{200000};
      regional.fragments.push_back(fragment);
      window.fragments.push_back(fragment.id);
      window.owned_fragments.push_back(fragment.id);
      window.nets.push_back(net);
      channel.occupants.push_back(fragment.id);
      flux::RegionalCrossSection section;
      section.first_wall = mm(11.0 + 5.0 * i, 39.6);
      section.second_wall = mm(11.0 + 5.0 * i, 40.4);
      if (mirror) std::swap(section.first_wall, section.second_wall);
      section.usable_width = Nanometres{800000};
      channel.cross_sections.push_back(section);
      flux::RegionalRouteTemplate route;
      route.id = i;
      route.fragment = fragment.id;
      route.routing_layer = channel.layer;
      route.layer_sequence = {channel.layer};
      route.channel_sequence = {channel.id};
      templates.push_back(route);
      domains.push_back({i});
    }
    regional.channels = {channel};
    regional.configured.local_channel_occupancy = false;
    require(flux::enumerate_regional_bundle_choices(board, regional, window, templates,
                domains, 256U, false).choices.empty(),
            "whole-channel capacity must reproduce false rejection of remote sequential lanes");
    const auto scopes = flux::regional_channel_demand_scopes(regional.fragments,
                                                            regional.channels, templates);
    require(scopes.size() == 4U, "remote runs must have four separate demand scopes");
    for (const auto& scope : scopes) {
      require(scope.localized && scope.template_indices.size() == 1U &&
                  scope.cross_section_indices.size() == 1U,
              "each longitudinal run must have localized, inspectable support");
      const auto& section = channel.cross_sections[scope.cross_section_indices.front()];
      require(flux::pack_regional_cross_section(regional.fragments,
                  {templates[scope.template_indices.front()].fragment}, channel.layer,
                  section.first_wall, section.second_wall, section.usable_width).physically_fits,
              "every localized cohort must fit the unchanged physical lane-width model");
    }
    regional.configured.local_channel_occupancy = true;
    for (const bool forward : {false, true}) {
      regional.configured.enable_bundle_resource_forward_checking = forward;
      const auto accepted = flux::enumerate_regional_bundle_choices(board, regional, window,
          templates, domains, 256U, forward);
      require(accepted.choices.size() == 1U && !accepted.exhausted,
              "local resource demand must retain the complete longitudinal allocation");
    }
    // Move the second run onto the first section: 1.10 mm demand cannot fit 0.80 mm.
    auto competing = regional;
    competing.fragments[1].start = competing.fragments[0].start;
    competing.fragments[1].end = competing.fragments[0].end;
    for (const bool forward : {false, true}) {
      const auto rejected = flux::enumerate_regional_bundle_choices(board, competing, window,
          templates, domains, 256U, forward);
      require(rejected.choices.empty(), "true simultaneous lane overflow must still be rejected");
    }
    auto unknown = regional;
    for (auto& section : unknown.channels.front().cross_sections)
      section.second_wall = section.first_wall;
    require(flux::regional_channel_demand_scopes(unknown.fragments, unknown.channels,
                templates).size() == 1U &&
                flux::enumerate_regional_bundle_choices(board, unknown, window, templates,
                    domains, 256U, false).choices.empty(),
            "missing wall geometry must retain the conservative legacy capacity guard");
    // The centerline misses the wall, but its width/clearance envelope reaches it.
    auto near = regional;
    near.fragments[1].start = mm(11.1, 40.5);
    near.fragments[1].end = mm(11.2, 40.6);
    const auto near_scopes = flux::regional_channel_demand_scopes(near.fragments,
                                                                near.channels, templates);
    require(std::any_of(near_scopes.begin(), near_scopes.end(), [](const auto& scope) {
              return std::find(scope.template_indices.begin(), scope.template_indices.end(), 0U) != scope.template_indices.end() &&
                     std::find(scope.template_indices.begin(), scope.template_indices.end(), 1U) != scope.template_indices.end();
            }), "envelope-only contact outside a finite wall must not hide transverse demand");
    // Two distant finite channels may have the same normal coordinate. Each
    // branch has positive local support, but neither consumes the other's wall.
    auto lateral = regional;
    lateral.fragments[1].start = mm(10.0, 50.0);
    lateral.fragments[1].end = mm(12.0, 50.0);
    auto& remote_wall = lateral.channels.front().cross_sections[1];
    remote_wall.first_wall = mm(11.0, 49.6);
    remote_wall.second_wall = mm(11.0, 50.4);
    if (mirror) std::swap(remote_wall.first_wall, remote_wall.second_wall);
    const auto lateral_scopes = flux::regional_channel_demand_scopes(lateral.fragments,
                                                                   lateral.channels, templates);
    require(std::all_of(lateral_scopes.begin(), lateral_scopes.end(), [](const auto& scope) {
              return scope.template_indices.size() == 1U;
            }), "remote finite walls must not couple laterally separated branches");
    require(flux::enumerate_regional_bundle_choices(board, lateral, window, templates,
                domains, 256U, true).choices.size() == 1U,
            "local physical support must preserve laterally separated feasible demand");
  }
}

void physical_probe_preserves_native_coordinate_clearance() {
  for (const double reflection : {1.0, -1.0}) {
    for (const bool source_legal : {false, true}) {
      auto board = two_pad_board();
      board.rules.default_track_width = Nanometres{250000};
      board.rules.minimum_clearance = Nanometres{200000};
      board.rules.copper_to_edge_clearance = Nanometres{200000};
      const auto boundary = source_legal ? 39.71 : 39.75;
      board.outline = reflection > 0.0 ? rectangle(boundary, 0.0, 100.0, 80.0)
                                       : rectangle(-100.0, 0.0, -boundary, 80.0);
      board.footprints.front().position = mm(reflection * 41.0, 20.0);
      board.footprints.back().position = mm(reflection * 41.0, 60.0);
      const auto run_x = reflection * (source_legal ? 40.04 : 40.06);
      flux::GlobalRouteResult global;
      global.paths = {flux::GlobalRoutePath{NetId{"net-1"}, 0U,
          {{board.footprints.front().position, LayerId{"F.Cu"}},
           {mm(run_x, 30.0), LayerId{"F.Cu"}},
           {mm(run_x, 50.0), LayerId{"F.Cu"}},
           {board.footprints.back().position, LayerId{"F.Cu"}}}}};
      const auto native = flux::probe_quantized_copper_feasibility(board, global);
      flux::QuantizedCopperOracleConfig coarse_config;
      coarse_config.grid_pitch = Nanometres{100000};
      const auto coarse = flux::probe_quantized_copper_feasibility(board, global, coarse_config);
      require(native.configured_grid_pitch == Nanometres{1} &&
                  coarse.configured_grid_pitch == coarse_config.grid_pitch &&
                  native.all_paths_materialized && coarse.all_paths_materialized &&
                  native.open_connections == 0U && coarse.open_connections == 0U,
              "both explicit coarse and default native probes must retain connected geometry");
      require(native.feasible() == source_legal && coarse.feasible() != source_legal,
              "physical feedback must neither invent nor conceal clearance errors by coarse "
              "rounding, including mirrored geometry");
      const auto& rejected = source_legal ? coarse : native;
      require(rejected.drc_violation_counts.contains("track_edge_clearance") &&
                  !rejected.diagnoses.empty(),
              "the illegal representation must retain exact edge-conflict feedback");
    }
  }
}

void source_bound_board_rules_control_geometry_and_hole_clearance() {
  auto source = two_pad_board();
  const std::string snapshot = R"((flux_kicad_netclass_snapshot
  (version 3)
  (source_revision "risk-fixture")
  (project_revision "sha256:project-fixture")
  (api_version "10.0.4-test")
  (board_rules
    (minimum_clearance_nm 250000)
    (minimum_track_width_nm 300000)
    (minimum_via_diameter_nm 400000)
    (minimum_via_annular_width_nm 150000)
    (minimum_through_drill_nm 300000)
    (copper_to_edge_clearance_nm 350000)
    (hole_clearance_nm 200000)
    (hole_to_hole_clearance_nm 250000)
  )
  (class
    (name "Default")
    (source_priority 2147483647)
    (clearance_nm 100000)
    (track_width_nm 200000)
    (via_diameter_nm 400000)
    (via_drill_nm 100000)
    (diff_pair_track_width_nm 200000)
    (diff_pair_gap_nm 150000)
    (diff_pair_via_gap_nm 200000)
  )
  (net (name "SIGNAL") (classes))
))";
  const auto applied = flux::io::apply_kicad_netclass_snapshot(source, snapshot);
  require(applied.ok() && applied.applied_board_rules && applied.board &&
              applied.project_revision == "sha256:project-fixture",
          "a version-3 KiCad snapshot must atomically bind project fabrication rules");
  const auto resolved = flux::resolve_net_routing_rules(*applied.board, NetId{"net-1"});
  require(resolved.clearance == Nanometres{250000} && resolved.track_width == Nanometres{300000} &&
              resolved.via_drill == Nanometres{300000} &&
              resolved.via_diameter == Nanometres{600000} &&
              applied.board->rules.copper_to_edge_clearance == Nanometres{350000},
          "project floors must raise generated copper dimensions without rewriting class values");

  auto board = *applied.board;
  board.tracks = {{ObjectId{"same-net-hole-neighbour"}, NetId{"net-1"}, LayerId{"F.Cu"},
                   mm(39.5, 40.3), mm(40.5, 40.3), Nanometres{100'000}, false},
                  {ObjectId{"foreign-hole-neighbour"}, NetId{"net-2"}, LayerId{"F.Cu"},
                   mm(39.5, 39.7), mm(40.5, 39.7), Nanometres{100'000}, false}};
  board.vias = {{ObjectId{"hole-a"},
                 NetId{"net-1"},
                 mm(40.0, 40.0),
                 Nanometres{600000},
                 Nanometres{300000},
                 {LayerId{"F.Cu"}, LayerId{"B.Cu"}},
                 false},
                {ObjectId{"hole-b"},
                 NetId{"net-1"},
                 mm(40.5, 40.0),
                 Nanometres{600000},
                 Nanometres{300000},
                 {LayerId{"F.Cu"}, LayerId{"B.Cu"}},
                 false}};
  const auto whole = flux::check_copper_rules(board);
  const auto has_hole_violation = [](const auto& report) {
    return std::any_of(report.violations.begin(), report.violations.end(), [](const auto& item) {
      return item.kind == flux::CopperViolationKind::hole_to_hole_clearance;
    });
  };
  require(has_hole_violation(whole) &&
              has_hole_violation(flux::check_dynamic_copper_rules(board, {ObjectId{"hole-b"}})),
          "hole-to-hole clearance must remain mechanical across same-net vias in whole and "
          "dynamic exact DRC");
  const auto has_hole_clearance_for = [](const auto& report, std::string_view id) {
    return std::any_of(report.violations.begin(), report.violations.end(), [&](const auto& item) {
      return item.kind == flux::CopperViolationKind::hole_clearance &&
             std::find(item.object_ids.begin(), item.object_ids.end(), id) != item.object_ids.end();
    });
  };
  require(has_hole_clearance_for(whole, "foreign-hole-neighbour") &&
              !has_hole_clearance_for(whole, "same-net-hole-neighbour") &&
              has_hole_clearance_for(
                  flux::check_dynamic_copper_rules(board, {ObjectId{"foreign-hole-neighbour"}}),
                  "foreign-hole-neighbour") &&
              !has_hole_clearance_for(
                  flux::check_dynamic_copper_rules(board, {ObjectId{"same-net-hole-neighbour"}}),
                  "same-net-hole-neighbour"),
          "hole clearance must use exact hole-to-copper boundaries on traversed layers and exempt "
          "same-net copper in whole and dynamic DRC");

  Footprint slotted;
  slotted.id = ObjectId{"slotted"};
  slotted.reference = "H1";
  slotted.position = mm(60.0, 40.0);
  Pad slot;
  slot.id = ObjectId{"oval-hole"};
  slot.net = NetId{"net-1"};
  slot.layers = {LayerId{"F.Cu"}, LayerId{"B.Cu"}};
  slot.drill_width = Nanometres{1'000'000};
  slot.drill_height = Nanometres{400'000};
  slot.plated = false;
  slotted.pads = {slot};
  board.footprints.push_back(slotted);
  board.vias.push_back({ObjectId{"near-slot"},
                        NetId{"net-1"},
                        mm(60.7, 40.0),
                        Nanometres{600'000},
                        Nanometres{300'000},
                        {LayerId{"F.Cu"}, LayerId{"B.Cu"}},
                        false});
  const auto slotted_report = flux::check_copper_rules(board);
  require(std::none_of(slotted_report.violations.begin(), slotted_report.violations.end(),
                       [](const auto& item) {
                         return item.kind == flux::CopperViolationKind::hole_to_hole_clearance &&
                                std::find(item.object_ids.begin(), item.object_ids.end(),
                                          "oval-hole") != item.object_ids.end();
                       }),
          "milled oval holes must not be treated as mechanically drilled hole-to-hole pairs");

  const flux::DrillShape oval{ObjectId{"oval"},
                              NetId{},
                              mm(0.0, 0.0),
                              Nanometres{1'000'000},
                              Nanometres{400'000},
                              Microdegrees{0},
                              flux::DrillOwnerKind::pad,
                              {LayerId{"F.Cu"}},
                              false};
  const flux::DrillShape circle{ObjectId{"circle"},
                                NetId{},
                                mm(0.7, 0.0),
                                Nanometres{300'000},
                                Nanometres{300'000},
                                Microdegrees{0},
                                flux::DrillOwnerKind::via,
                                {LayerId{"F.Cu"}},
                                true};
  require(std::abs(flux::drill_to_drill_boundary_distance_nm(oval, circle) - 50'000.0L) < 1.0L,
          "oval drill spacing must use the exact capsule boundary rather than a bounding circle");
}

[[nodiscard]] std::string load_minimal_kicad_fixture() {
  const auto path = std::filesystem::path{FLUX_TEST_SOURCE_DIR} / "fixtures/minimal.kicad_pcb";
  std::ifstream input(path, std::ios::binary);
  require(static_cast<bool>(input), "minimal KiCad fixture must be readable");
  return std::string{std::istreambuf_iterator<char>{input}, std::istreambuf_iterator<char>{}};
}

[[nodiscard]] std::string load_tentative_placement_fixture() {
  const auto path =
      std::filesystem::path{FLUX_TEST_SOURCE_DIR} / "fixtures/tentative-placement.kicad_pcb";
  std::ifstream input(path, std::ios::binary);
  require(static_cast<bool>(input), "tentative-placement KiCad fixture must be readable");
  return std::string{std::istreambuf_iterator<char>{input}, std::istreambuf_iterator<char>{}};
}

void file_import_preserves_project_minimum_rules() {
  const auto root = std::filesystem::temp_directory_path() /
      ("flux-project-rule-import-" +
       std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
  require(std::filesystem::create_directory(root), "project import test directory must be new");
  struct Cleanup {
    std::filesystem::path path;
    ~Cleanup() { std::error_code error; std::filesystem::remove_all(path, error); }
  } cleanup{root};
  const auto board_path = root / "board.kicad_pcb";
  const auto project_path = root / "board.kicad_pro";
  const auto source = load_minimal_kicad_fixture();
  const auto write = [](const auto& path, const std::string& contents) {
    std::ofstream stream(path, std::ios::binary);
    stream << contents;
    require(static_cast<bool>(stream), "project import fixture must be written completely");
  };
  write(board_path, source);
  const auto without_project = flux::io::import_kicad_pcb_file(board_path);
  require(without_project.strict() && !without_project.project_minimum_rules_applied,
          "standalone board import must explicitly report absent project minima");
  const std::string project = R"JSON({"board":{"design_settings":{"rules":{
    "min_clearance": 0.2,
    "min_track_width": 0.7,
    "min_via_diameter": 0.7,
    "min_via_annular_width": 0.15,
    "min_through_hole_diameter": 0.3,
    "min_copper_edge_clearance": 0.5,
    "min_hole_clearance": 0.25,
    "min_hole_to_hole": 0.254
  }}}})JSON";
  write(project_path, project);
  const auto imported = flux::io::import_kicad_pcb_file(board_path);
  require(imported.strict() && imported.project_minimum_rules_applied &&
              imported.project_rules_fingerprint.starts_with("fnv1a64:") &&
              imported.board->source_revision == without_project.board->source_revision,
          "sibling project minima must be loaded with separate rule provenance");
  const auto& rules = imported.board->rules;
  const auto resolved = flux::resolve_net_routing_rules(*imported.board, NetId{"any-net"});
  require(rules.default_track_width == Nanometres{250000} &&
              rules.default_via_diameter == Nanometres{600000} &&
              resolved.track_width == Nanometres{700000} &&
              resolved.via_diameter == Nanometres{700000} &&
              rules.hole_clearance == Nanometres{250000} &&
              rules.hole_to_hole_clearance == Nanometres{254000},
          "project floors must constrain routing without changing nominal netclass policy");
  const auto exported = flux::io::export_kicad_candidate(source, *imported.board, *imported.board);
  require(exported.ok(), "project minima must preserve strict copper export round-trip identity");
  for (const std::string replacement : {"null", "true", "\"0.7\"", "-0.7", "1e100"}) {
    auto invalid = project;
    const auto offset = invalid.find("\"min_track_width\": 0.7");
    invalid.replace(offset, std::string("\"min_track_width\": 0.7").size(),
                    "\"min_track_width\": " + replacement);
    write(project_path, invalid);
    const auto rejected = flux::io::import_kicad_pcb_file(board_path);
    require(!rejected.ok() && !rejected.project_minimum_rules_applied &&
                rejected.board->rules == without_project.board->rules,
            "invalid project dimension must fail atomically, not leave partially loaded floors");
  }
  for (const std::string invalid : {"{", "{}", "{\"board\":{\"design_settings\":{\"rules\":{}}}}"}) {
    write(project_path, invalid);
    const auto rejected = flux::io::import_kicad_pcb_file(board_path);
    require(!rejected.ok() && !rejected.project_minimum_rules_applied,
            "malformed or incomplete project rules must not fall back to permissive defaults");
  }
}

void units_remain_exact_and_saturating() {
  require(Nanometres::from_mm(1.2345674).value() == 1'234'567,
          "millimetres must quantize to integer nanometres");
  require(Nanometres::from_mm(-0.0000006).value() == -1,
          "negative conversion must round symmetrically");
  require(Microdegrees::from_degrees(-90.0).value() == 270'000'000,
          "angles must normalize into one positive turn");
  require(flux::geometry::transform_point(mm(4.45, -0.25), mm(48.0, 47.0),
                                          Microdegrees::from_degrees(90.0),
                                          BoardSide::front) == mm(47.75, 42.55),
          "positive KiCad footprint rotation must follow its downward-Y board coordinates");
}

void multisource_terminal_membership_preserves_access_preference_order() {
  auto board = two_pad_board();
  board.outline = rectangle(0.0, 0.0, 20.0, 20.0);
  board.footprints[0].position = mm(5.0, 5.0);
  board.footprints[1].position = mm(5.3, 8.0);
  for (auto& footprint : board.footprints) {
    footprint.courtyard = local_rectangle(1.0, 1.0);
  }
  board.rules.minimum_track_width = Nanometres{700000};
  GlobalRouterConfig config;
  config.grid_pitch = Nanometres{900000};
  config.max_iterations = 1U;
  config.max_pin_escape_grid_radius = 3U;
  config.max_pin_access_nodes_per_terminal = 16U;
  config.diversify_pin_access_rings = true;
  config.use_multisource_steiner_growth = true;
  config.hard_capacity_start_iteration = 1U;
  const auto routed = flux::route_global(board, config);
  require(routed.diversified_pin_access_nodes > 0U &&
              !routed.hard_capacity_pass_summaries.empty(),
          "terminal ownership regression must exercise preference-ordered adaptive access: " +
              routed.diagnostic);
  for (const auto& pass : routed.hard_capacity_pass_summaries) {
    require(pass.connection_failures.empty() && pass.unrouted_nets.empty(),
            "a found multi-source path must retain its terminal owner across adaptive rings");
  }
  const auto copper = flux::materialize_detailed_route(board, routed);
  require(routed.complete() && copper.complete() && copper.copper_drc.violation_count() == 0U,
          "adaptive terminal membership must preserve exact connected copper legality");
}

void failed_spline_chains_retry_against_committed_copper() {
  auto board = two_pad_board();
  board.nets.push_back({NetId{"net-2"}, "OUTER", 0});
  board.footprints[0].position = mm(20.0, 20.0);
  board.footprints[1].position = mm(40.0, 40.0);
  for (auto& footprint : board.footprints) {
    footprint.courtyard = local_rectangle(0.15, 0.15);
    footprint.pads.front().width = Nanometres{250000};
    footprint.pads.front().height = Nanometres{250000};
  }
  for (std::size_t index = 0U; index < 2U; ++index) {
    auto footprint = board.footprints[index];
    footprint.id = ObjectId{"outer-" + std::to_string(index)};
    footprint.reference = footprint.id.value();
    footprint.pads.front().id = ObjectId{footprint.id.value() + "-pad"};
    footprint.pads.front().net = NetId{"net-2"};
    footprint.position = index == 0U ? mm(20.0, 19.5) : mm(40.5, 40.0);
    board.footprints.push_back(std::move(footprint));
  }
  flux::GlobalRouteResult global;
  global.stop_reason = GlobalRouterStopReason::complete;
  global.paths = {
      flux::GlobalRoutePath{NetId{"net-1"}, 0U,
          {{mm(20,20), LayerId{"F.Cu"}}, {mm(40,20), LayerId{"F.Cu"}},
           {mm(40,40), LayerId{"F.Cu"}}}},
      flux::GlobalRoutePath{NetId{"net-2"}, 0U,
          {{mm(20,19.5), LayerId{"F.Cu"}}, {mm(40.5,19.5), LayerId{"F.Cu"}},
           {mm(40.5,40), LayerId{"F.Cu"}}}}};
  flux::DetailedRouterConfig config;
  config.pull_tight_45_degrees = false;
  config.continuous_refinement = false;
  config.circular_arc_refinement = true;
  // Isolate PH committed-context retries from the later residual fillet postpass.
  config.residual_bend_repair = false;
  config.spline_minimum_bend_radius = Nanometres::from_mm(1.0);
  config.spline_control_corridor = Nanometres::from_mm(1.0);
  config.retry_failed_spline_routes_after_commits = false;
  const auto single_pass = flux::materialize_detailed_route(board, global, config);
  require(!single_pass.complete() && single_pass.unrounded_non_junction_bends == 1U &&
              single_pass.spline_parallel_route_commits == 1U && single_pass.copper_drc.ok(),
          "nested-corner control must expose one stale-context failure after one legal commit: " +
              single_pass.diagnostic + "/commits=" + std::to_string(single_pass.spline_parallel_route_commits) +
              "/unrounded=" + std::to_string(single_pass.unrounded_non_junction_bends) +
              "/drc=" + std::to_string(single_pass.copper_drc.violation_count()));
  config.retry_failed_spline_routes_after_commits = true;
  const auto result = flux::materialize_detailed_route(board, global, config);
  require(result.complete() && result.copper_drc.ok() &&
              result.connectivity.open_connections == 0U &&
              result.spline_committed_context_retry_tasks == 1U &&
              result.spline_committed_context_retry_commits == 1U,
          "a failed outer bend must be reconsidered after its inner neighbour is rounded: " +
              result.diagnostic);
}

void dense_candidate_grid_requires_enforced_physical_clearance() {
  auto board = two_pad_board();
  board.rules.minimum_track_width = Nanometres{700000};
  GlobalRouterConfig config;
  config.grid_pitch = Nanometres{500000};
  config.max_iterations = 1U;
  config.max_expansions_per_connection = 50000U;
  config.allow_dense_candidate_grid = false;
  const auto conservative = flux::route_global(board, config);
  require(conservative.stop_reason == GlobalRouterStopReason::invalid_input &&
              conservative.diagnostic.find("planar grid") != std::string::npos,
          "dense candidate samples must remain an explicit opt-in");
  config.allow_dense_candidate_grid = true;
  for (const bool advisory : {false, true}) {
    auto unsafe = config;
    unsafe.reserve_sampled_route_clearance = advisory;
    unsafe.clearance_shadows_advisory_only = advisory;
    const auto rejected = flux::route_global(board, unsafe);
    require(rejected.stop_reason == GlobalRouterStopReason::invalid_input,
            "dense samples cannot rely on missing or advisory physical conflict enforcement");
  }
  const auto routed = flux::route_global(board, config);
  require(routed.complete() && routed.configured_dense_candidate_grid &&
              routed.configured_grid_pitch == Nanometres{500000} &&
              routed.clearance_shadow_overflow == 0U,
          "a wide trace must remain routable on denser, physically guarded candidate samples: " +
              routed.diagnostic);
  const auto detailed = flux::materialize_detailed_route(board, routed);
  require(detailed.complete() && detailed.copper_drc.violation_count() == 0U,
          "dense candidate sampling must still produce exact legal copper: " + detailed.diagnostic);
  require(detailed.candidate && !detailed.candidate->tracks.empty() &&
              std::all_of(detailed.candidate->tracks.begin(), detailed.candidate->tracks.end(),
                          [](const auto& track) { return track.width >= Nanometres{700000}; }),
          "the sampling experiment must not narrow generated copper");
  board.nets.push_back({NetId{"net-2"}, "NEIGHBOUR", 0});
  for (std::size_t i = 0; i < 2U; ++i) {
    auto terminal = board.footprints[i];
    terminal.id = ObjectId{"neighbour-" + std::to_string(i)};
    terminal.reference = terminal.id.value();
    terminal.position.y += Nanometres{2000000};
    terminal.pads.front().id = ObjectId{terminal.id.value() + "-pad"};
    terminal.pads.front().net = NetId{"net-2"};
    board.footprints.push_back(std::move(terminal));
  }
  flux::GlobalRouteResult conflicting;
  flux::GlobalRoutePath first;
  first.net = NetId{"net-1"};
  first.points = {{mm(20, 40), LayerId{"F.Cu"}}, {mm(80, 40), LayerId{"F.Cu"}}};
  flux::GlobalRoutePath second;
  second.net = NetId{"net-2"};
  second.points = {{mm(20, 42), LayerId{"F.Cu"}}, {mm(30, 42), LayerId{"F.Cu"}},
                   {mm(30, 40.5), LayerId{"F.Cu"}}, {mm(70, 40.5), LayerId{"F.Cu"}},
                   {mm(70, 42), LayerId{"F.Cu"}}, {mm(80, 42), LayerId{"F.Cu"}}};
  conflicting.paths = {std::move(first), std::move(second)};
  const auto rejected_copper = flux::probe_quantized_copper_feasibility(board, conflicting);
  require(!rejected_copper.feasible() && rejected_copper.open_connections == 0U &&
              rejected_copper.drc_violations > 0U,
          "different occupied grid lines cannot certify overlapping wide copper as legal");
  flux::QuantizedCopperOracleConfig bounded_diagnoses;
  bounded_diagnoses.max_diagnoses = 1U;
  const auto bounded = flux::probe_quantized_copper_feasibility(board, conflicting, bounded_diagnoses);
  const auto raw_count = std::accumulate(
      bounded.drc_violation_counts.begin(), bounded.drc_violation_counts.end(), std::size_t{0},
      [](std::size_t count, const auto& entry) { return count + entry.second; });
  require(raw_count == bounded.drc_violations &&
              bounded.drc_violation_counts == rejected_copper.drc_violation_counts &&
              bounded.drc_violation_counts.contains("track_clearance"),
          "raw DRC categories must survive diagnosis coalescing and truncation");
}

void default_planar_pitch_uses_scoped_source_rules() {
  auto board = two_pad_board();
  GlobalRouterConfig config;
  require(flux::minimum_legal_planar_grid_pitch(board, config) == Nanometres{450000},
          "the default floor must combine resolved trace width and clearance");
  board.rules.minimum_track_width = Nanometres{700000};
  require(flux::minimum_legal_planar_grid_pitch(board, config) == Nanometres{900000},
          "project fabrication minima must determine the automatic floor");
  for (const auto crossing_safe : {false, true}) {
    config.allow_diagonal_grid_edges = !crossing_safe;
    config.allow_crossing_safe_diagonal_grid_edges = crossing_safe;
    require(flux::minimum_legal_planar_grid_pitch(board, config) == Nanometres{1272793},
            "either diagonal mode must round sqrt(2) copper spacing upward to a legal integer");
  }
  config.allow_diagonal_grid_edges = false;
  config.allow_crossing_safe_diagonal_grid_edges = false;
  config.max_iterations = 1U;
  require(flux::route_global(board, config).stop_reason == GlobalRouterStopReason::invalid_input,
          "an explicit undersized library pitch must still fail validation");
  board.tracks.push_back({ObjectId{"already-connected"}, NetId{"net-1"}, LayerId{"F.Cu"},
                          mm(20,40), mm(80,40), Nanometres{700000}, false});
  require(flux::minimum_legal_planar_grid_pitch(board, config) == Nanometres{0},
          "completed nets must not inflate an unscoped routing lattice");
  config.selected_nets = {NetId{"net-1"}};
  require(flux::minimum_legal_planar_grid_pitch(board, config) == Nanometres{900000},
          "explicit selected scope must retain its physical spacing even for connected nets");
  board.rules.minimum_track_width = Nanometres{std::numeric_limits<std::int64_t>::max()};
  require(!flux::minimum_legal_planar_grid_pitch(board, config),
          "unrepresentable required spacing must fail closed rather than overflow");
}

void dimensioned_pathfinder_pressure_preserves_units_and_group_fairness() {
  const NetId first{"a-first"};
  const NetId second{"b-second"};
  flux::GlobalRouteResult route;

  flux::RoutingResourceUsage sampled;
  sampled.demand = 4U;
  sampled.capacity = 2U;
  sampled.nets = {first, second};
  route.resource_usage.push_back(sampled);

  flux::RoutingCorridorUsage corridor;
  corridor.id = 1U;
  corridor.demand = 5U;
  corridor.capacity = 2U;
  corridor.nets = {first};
  route.corridor_usage.push_back(corridor);
  auto neck_down = corridor;
  neck_down.id = 2U;
  neck_down.demand = 2U;
  neck_down.capacity = 1U;
  neck_down.nets = {second};
  route.corridor_neck_down_usage.push_back(neck_down);

  flux::RoutingVoronoiResourceUsage shared_physical;
  shared_physical.id = 3U;
  shared_physical.demand_nm = 1'000U;
  shared_physical.capacity_nm = 700U;
  shared_physical.nets = {first, second};
  route.voronoi_resource_usage.push_back(shared_physical);
  auto severe_physical = shared_physical;
  severe_physical.id = 4U;
  severe_physical.demand_nm = 900U;
  severe_physical.capacity_nm = 500U;
  severe_physical.nets = {first};
  route.voronoi_resource_usage.push_back(severe_physical);

  const auto pressure = flux::collect_global_route_net_pressure(route);
  require(pressure.size() == 2U && pressure[0].net == first &&
              pressure[0].legacy_congested_resources == 4U && pressure[0].sampled_overflow == 2U &&
              pressure[0].corridor_overflow == 3U &&
              pressure[0].total_physical_overflow_nm == 700U &&
              pressure[0].peak_physical_overflow_nm == 400U &&
              pressure[0].congested_physical_resources == 2U,
          "per-net pressure must remain stable and unit-separated");

  const auto group = flux::collect_global_route_group_pressure(route, {second, first, second});
  require(group.net == first && group.legacy_congested_resources == 5U &&
              group.sampled_overflow == 2U && group.corridor_overflow == 4U &&
              group.total_physical_overflow_nm == 700U && group.peak_physical_overflow_nm == 400U &&
              group.congested_physical_resources == 2U,
          "a coupled group must count a shared resource once regardless of duplicate members");

  flux::GlobalRouteResult physical_history_route;
  flux::RoutingVoronoiResourceUsage physical_history;
  physical_history.id = 5U;
  physical_history.layer = LayerId{"F.Cu"};
  physical_history.start = mm(1.0, 1.0);
  physical_history.end = mm(2.0, 1.0);
  physical_history.demand_nm = 1'330'000U;
  physical_history.capacity_nm = 700'000U;
  physical_history.nets = {first};
  physical_history_route.voronoi_resource_usage.push_back(physical_history);
  const auto physical_history_diagnoses = flux::probe_global_routability(physical_history_route);
  require(physical_history_diagnoses.size() == 1U &&
              physical_history_diagnoses.front().severity == 13U &&
              physical_history_diagnoses.front().clearance_deficit == Nanometres{} &&
              physical_history_diagnoses.front().planned_capacity == 1'330'000U &&
              physical_history_diagnoses.front().observed_capacity == 700'000U,
          "physical-medial overflow must enter routability history in clearance quanta, not raw "
          "nanometres that overwhelm negotiated PathFinder cost");

  flux::GlobalRouteResult lane_transition_route;
  flux::RoutingVoronoiResourceUsage first_lane_resource;
  first_lane_resource.id = 7U;
  first_lane_resource.layer = LayerId{"F.Cu"};
  first_lane_resource.start = mm(1.0, 1.0);
  first_lane_resource.end = mm(2.0, 1.0);
  first_lane_resource.nets = {first};
  flux::RoutingVoronoiResourceUsage second_lane_resource = first_lane_resource;
  second_lane_resource.id = 8U;
  second_lane_resource.start = mm(2.0, 1.0);
  second_lane_resource.end = mm(2.0, 2.0);
  second_lane_resource.nets = {second};
  lane_transition_route.voronoi_resource_usage = {first_lane_resource, second_lane_resource};
  flux::RoutingVoronoiLaneTransition rejected_transition;
  rejected_transition.first_resource_id = first_lane_resource.id;
  rejected_transition.second_resource_id = second_lane_resource.id;
  rejected_transition.net = first;
  rejected_transition.layer = LayerId{"F.Cu"};
  rejected_transition.medial_junction = mm(2.0, 1.0);
  rejected_transition.first_lane_endpoint = mm(1.8, 1.0);
  rejected_transition.second_lane_endpoint = mm(2.0, 1.2);
  lane_transition_route.voronoi_lane_transitions.push_back(rejected_transition);
  const auto lane_transition_diagnoses = flux::probe_global_routability(lane_transition_route);
  require(lane_transition_diagnoses.size() == 1U &&
              lane_transition_diagnoses.front().suggested_relaxation ==
                  flux::RoutabilityRelaxation::change_lane_order &&
              !lane_transition_diagnoses.front().contributes_to_history &&
              lane_transition_diagnoses.front().resource.nets == std::vector{first, second},
          "an unaccepted packed-lane bridge must expand incremental feedback to every physical "
          "occupant of its two medial resources, including a blocker without a shadow overlap");

  flux::RoutingNetPressure broad;
  broad.legacy_congested_resources = 2U;
  flux::RoutingNetPressure narrow;
  narrow.legacy_congested_resources = 1U;
  narrow.total_physical_overflow_nm = std::numeric_limits<std::uint64_t>::max();
  require(flux::routing_net_pressure_more_severe(
              broad, narrow, GlobalRouteCheckpointObjective::dimensioned_physical_overflow),
          "congestion breadth must prevent magnitude-driven scheduling starvation");

  broad.legacy_congested_resources = narrow.legacy_congested_resources;
  require(flux::routing_net_pressure_more_severe(
              narrow, broad, GlobalRouteCheckpointObjective::dimensioned_physical_overflow),
          "physical excess must resolve equal-breadth pressure without mixing units");

  Board pressure_board;
  Footprint duplicate;
  duplicate.id = ObjectId{"z-duplicate-net-pads"};
  duplicate.pads = {{ObjectId{"duplicate-1"}, first, Point{}, {LayerId{"F.Cu"}}},
                    {ObjectId{"duplicate-2"}, first, Point{}, {LayerId{"F.Cu"}}},
                    {ObjectId{"duplicate-3"}, first, Point{}, {LayerId{"F.Cu"}}}};
  Footprint single;
  single.id = ObjectId{"a-single-net-pad"};
  single.pads = {{ObjectId{"single-1"}, first, Point{}, {LayerId{"F.Cu"}}}};
  pressure_board.footprints = {std::move(duplicate), std::move(single)};
  const auto footprint_pressure = flux::collect_integrated_footprint_pressure(
      pressure_board, route, GlobalRouteCheckpointObjective::dimensioned_physical_overflow);
  require(footprint_pressure.size() == 2U &&
              footprint_pressure[0].footprint == ObjectId{"a-single-net-pad"} &&
              footprint_pressure[0].distinct_connected_nets == 1U &&
              footprint_pressure[1].distinct_connected_nets == 1U &&
              footprint_pressure[0].legacy_pressure == footprint_pressure[1].legacy_pressure &&
              footprint_pressure[0].physical_pressure_nm ==
                  footprint_pressure[1].physical_pressure_nm &&
              footprint_pressure[0].sampled_pressure == footprint_pressure[1].sampled_pressure &&
              footprint_pressure[0].corridor_pressure == footprint_pressure[1].corridor_pressure,
          "a congested net must contribute once per footprint regardless of repeated pads");

  route.physical_obstacle_impacts.push_back(
      flux::RoutingPhysicalObstacleImpact{3U,
                                          ObjectId{"duplicate-1"},
                                          LayerId{"F.Cu"},
                                          mm(1.0, 2.0),
                                          mm(3.0, 2.0),
                                          300U,
                                          {first, second}});
  const auto feedback = flux::collect_integrated_routing_feedback(route);
  const auto blocker_pressure = flux::collect_integrated_footprint_pressure(
      pressure_board, route, GlobalRouteCheckpointObjective::dimensioned_physical_overflow);
  require(feedback.physical_channel_obstacles == route.physical_obstacle_impacts &&
              std::find(feedback.routing_obstacles.begin(), feedback.routing_obstacles.end(),
                        ObjectId{"duplicate-1"}) != feedback.routing_obstacles.end() &&
              blocker_pressure.front().footprint == ObjectId{"z-duplicate-net-pads"} &&
              blocker_pressure.front().bounds_congested_channel &&
              blocker_pressure.front().blocks_route,
          "a source-bound channel limiter must propagate into placement feedback without losing "
          "its physical units");
}

void physical_checkpoint_ordering_is_feasibility_first_and_fail_closed() {
  using Objective = GlobalRouteCheckpointObjective;
  flux::GlobalRouteResult lhs;
  flux::GlobalRouteResult rhs;
  rhs.unrouted_nets = {NetId{"unrouted"}};
  lhs.total_voronoi_overflow_nm = std::numeric_limits<std::uint64_t>::max();
  require(flux::global_route_checkpoint_better(lhs, rhs, Objective::dimensioned_physical_overflow),
          "connectivity must dominate physical overflow");

  lhs = {};
  rhs = {};
  lhs.total_voronoi_overflow_nm = 100U;
  rhs.total_voronoi_overflow_nm = 200U;
  lhs.sampled_overflow = std::numeric_limits<std::size_t>::max();
  require(flux::global_route_checkpoint_better(lhs, rhs, Objective::dimensioned_physical_overflow),
          "exact physical excess must precede sampled overflow without adding their units");

  flux::IntegratedScore closer_to_strict_feasibility;
  flux::IntegratedScore softer_but_farther_from_strict_feasibility;
  closer_to_strict_feasibility.configured_dimensioned_physical_overflow = true;
  softer_but_farther_from_strict_feasibility.configured_dimensioned_physical_overflow = true;
  closer_to_strict_feasibility.best_hard_capacity_unrouted_nets = 5U;
  closer_to_strict_feasibility.total_physical_overflow_nm =
      std::numeric_limits<std::uint64_t>::max();
  softer_but_farther_from_strict_feasibility.best_hard_capacity_unrouted_nets = 6U;
  require(closer_to_strict_feasibility < softer_but_farther_from_strict_feasibility,
          "integrated placement must reduce the best constructive hard-capacity deficit before "
          "optimizing a selected soft checkpoint");

  auto board = two_pad_board();
  GlobalRouterConfig invalid;
  invalid.grid_pitch = Nanometres::from_mm(5.0);
  invalid.checkpoint_objective = Objective::dimensioned_physical_overflow;
  const auto rejected = flux::route_global(board, invalid);
  require(rejected.stop_reason == GlobalRouterStopReason::invalid_input &&
              rejected.diagnostic.find("requires shared Voronoi allocation") != std::string::npos,
          "physical checkpoint selection must fail closed without physical resources");

  invalid = {};
  invalid.grid_pitch = Nanometres::from_mm(5.0);
  invalid.max_voronoi_resource_segment_length = Nanometres::from_mm(5.0);
  const auto unbacked_segmentation = flux::route_global(board, invalid);
  require(unbacked_segmentation.stop_reason == GlobalRouterStopReason::invalid_input &&
              unbacked_segmentation.diagnostic.find("requires shared Voronoi allocation") !=
                  std::string::npos,
          "longitudinal capacity segmentation must fail closed without physical resources");

  invalid.use_voronoi_allocation_resources = true;
  invalid.max_voronoi_allocation_resources = 1U;
  const auto over_budget_segmentation = flux::route_global(board, invalid);
  require(over_budget_segmentation.stop_reason == GlobalRouterStopReason::invalid_input &&
              over_budget_segmentation.diagnostic.find("resource budget") != std::string::npos,
          "longitudinal capacity segmentation must fail closed at the physical resource budget");

  invalid = {};
  invalid.grid_pitch = Nanometres::from_mm(5.0);
  invalid.via_site_pitch = Nanometres::from_mm(7.5);
  const auto invalid_via_lattice = flux::route_global(board, invalid);
  require(invalid_via_lattice.stop_reason == GlobalRouterStopReason::invalid_input &&
              invalid_via_lattice.diagnostic.find("exact positive multiple") != std::string::npos,
          "a decoupled via lattice must remain an exact bounded sublattice of the planar grid");

  GlobalRouterConfig fine_planar_lattice;
  fine_planar_lattice.grid_pitch = Nanometres::from_mm(0.5);
  fine_planar_lattice.via_site_pitch = Nanometres{0};
  fine_planar_lattice.max_iterations = 1U;
  const auto derived_via_lattice = flux::route_global(board, fine_planar_lattice);
  require(derived_via_lattice.stop_reason != GlobalRouterStopReason::invalid_input &&
              derived_via_lattice.configured_via_site_pitch == Nanometres{0},
          "an omitted via pitch must retain dense candidate sites and defer selected-site spacing "
          "to exact via occupancy instead of invalidating a finer planar route grid");

  invalid = {};
  invalid.grid_pitch = Nanometres::from_mm(5.0);
  invalid.capacity_reserve_per_mille = 1'001U;
  const auto invalid_capacity_reserve = flux::route_global(board, invalid);
  require(invalid_capacity_reserve.stop_reason == GlobalRouterStopReason::invalid_input &&
              invalid_capacity_reserve.diagnostic.find("capacity reserve") != std::string::npos,
          "capacity-reserve pressure must remain bounded and fail closed above one full resource");

  invalid = {};
  invalid.grid_pitch = Nanometres::from_mm(5.0);
  invalid.local_pin_escape_pitch = Nanometres::from_mm(3.0);
  const auto invalid_local_mesh = flux::route_global(board, invalid);
  require(invalid_local_mesh.stop_reason == GlobalRouterStopReason::invalid_input &&
              invalid_local_mesh.diagnostic.find("exact divisor") != std::string::npos,
          "a local escape mesh must remain exactly registered to its global portals");

  invalid = {};
  invalid.grid_pitch = Nanometres::from_mm(5.0);
  invalid.atomic_complete_net_routing = true;
  invalid.selective_full_reroute_interval = 2U;
  const auto unbacked_full_refresh = flux::route_global(board, invalid);
  require(unbacked_full_refresh.stop_reason == GlobalRouterStopReason::invalid_input &&
              unbacked_full_refresh.diagnostic.find("requires selective congestion rerouting") !=
                  std::string::npos,
          "periodic full rerouting must fail closed without selective whole-net reuse");

  invalid = {};
  invalid.grid_pitch = Nanometres::from_mm(5.0);
  invalid.enable_search_diversification = true;
  const auto unbacked_search_diversification = flux::route_global(board, invalid);
  require(unbacked_search_diversification.stop_reason == GlobalRouterStopReason::invalid_input &&
              unbacked_search_diversification.diagnostic.find("atomic complete-net routing") !=
                  std::string::npos,
          "large-neighborhood search must fail closed unless every destroyed coupled group is "
          "rerouted atomically");

  flux::RoutabilityDiagnosis physical_failure;
  physical_failure.kind = flux::RoutabilityFailureKind::packing_capacity_exceeded;
  physical_failure.stage = flux::RoutabilityFailureStage::global_probe;
  physical_failure.resource = flux::RoutabilityResourceReference{
      "fixture-channel", LayerId{"F.Cu"}, {mm(40.0, 35.0), mm(60.0, 45.0)}, {NetId{"net-1"}}};
  physical_failure.severity = 3U;
  flux::RoutabilityKnowledge knowledge;
  flux::accumulate_routability_knowledge(knowledge, {physical_failure});
  auto repeated_failure = physical_failure;
  repeated_failure.severity = 2U;
  flux::accumulate_routability_knowledge(knowledge, {repeated_failure});
  require(knowledge.history.size() == 1U && knowledge.history.front().observations == 2U &&
              knowledge.history.front().pressure == 5U,
          "repeated physical failures must accumulate deterministically as bounded cost evidence");

  GlobalRouterConfig feedback_config;
  feedback_config.grid_pitch = Nanometres::from_mm(5.0);
  feedback_config.max_iterations = 1U;
  feedback_config.routability_history = knowledge.history;
  const auto feedback_route = flux::route_global(board, feedback_config);
  require(feedback_route.routability_history_entries == 1U &&
              feedback_route.routability_history_edges > 0U &&
              feedback_route.routability_history_pressure == 5U,
          "global routing must prepare physical history into net-scoped non-authoritative edge "
          "costs before static and negotiated search");

  flux::RegionalRouteResult regional_feedback;
  auto locally_resolved = physical_failure;
  locally_resolved.resource.stable_id = "resolved-window";
  locally_resolved.locally_resolved = true;
  auto unresolved = physical_failure;
  unresolved.resource.stable_id = "unresolved-window";
  regional_feedback.local_routability_diagnoses = {locally_resolved, unresolved};
  const auto escalated = flux::diagnose_regional_routability(regional_feedback);
  require(escalated.size() == 1U && !escalated.front().locally_resolved,
          "a locally discharged spline/PNS diagnosis must not become upstream PathFinder history");

  flux::DetailedRouteResult rejected_detail;
  rejected_detail.candidate = board;
  rejected_detail.stop_reason = flux::DetailedRouterStopReason::connectivity_incomplete;
  rejected_detail.connectivity.nets = {flux::NetConnectivity{NetId{"net-1"}, 2U, 2U, 1U}};
  rejected_detail.connectivity.open_connections = 1U;
  const auto detailed_diagnoses = flux::diagnose_detailed_routability(rejected_detail);
  require(detailed_diagnoses.size() == 1U &&
              detailed_diagnoses.front().kind == flux::RoutabilityFailureKind::topological_trap &&
              detailed_diagnoses.front().stage == flux::RoutabilityFailureStage::final_validation &&
              detailed_diagnoses.front().resource.nets == std::vector<NetId>{NetId{"net-1"}},
          "final connectivity rejection must return an attributed topological diagnosis");
  rejected_detail.candidate->tracks.push_back(
      flux::TrackSegment{ObjectId{"clean-feedback-track"}, NetId{"net-1"}, LayerId{"F.Cu"},
                         mm(10.0, 10.0), mm(20.0, 20.0), Nanometres::from_mm(0.2), false});
  flux::accumulate_clean_copper_knowledge(knowledge, rejected_detail);
  flux::accumulate_clean_copper_knowledge(knowledge, rejected_detail);
  require(knowledge.success_history.size() == 1U &&
              knowledge.success_history.front().observations == 2U &&
              knowledge.success_history.front().strength == 2U &&
              knowledge.success_history.front().start == mm(10.0, 10.0) &&
              knowledge.success_history.front().end == mm(20.0, 20.0),
          "clean quantized copper must accumulate a bounded directional success prior separately "
          "from failure pressure");
}

void regional_scope_checkpoint_ordering_retains_checked_physical_evidence() {
  auto checked = flux::RegionalRouteResult{};
  checked.native_scene_evaluated = true;
  checked.native_scene_complete = true;
  checked.native_scene_conflicts = 3U;
  checked.native_scene_total_conflicts = 3U;
  auto unassembled = checked;
  unassembled.native_scene_evaluated = false;
  unassembled.native_scene_complete = false;
  unassembled.native_scene_conflicts = 0U;
  unassembled.spline_candidate_available = true;
  require(!unassembled.complete(), "an unassessed candidate must not claim physical completion");
  require(flux::regional_checkpoint_better(checked, unassembled),
          "an assembled native scene must outrank an unassembled retry with zero counters");

  auto better_scene = checked;
  better_scene.straight_scene_trials = 1U;
  better_scene.straight_scene_evaluated = true;
  better_scene.straight_scene_drc_violations = 3U;
  auto worse_scope = better_scene;
  worse_scope.straight_scene_drc_violations = 9U;
  require(flux::regional_checkpoint_better(better_scene, worse_scope),
          "a worse expanded physical scope must not replace the retained checkpoint");

  auto fewer_opens = better_scene;
  fewer_opens.straight_scene_drc_violations = 12U;
  auto more_opens = better_scene;
  more_opens.straight_scene_open_connections = 1U;
  require(flux::regional_checkpoint_better(fewer_opens, more_opens),
          "open connections must be prioritized before DRC/native error counts");

  auto legal = checked;
  legal.native_scene_conflicts = 0U;
  legal.native_scene_total_conflicts = 0U;
  legal.spline_candidate_available = true;
  legal.straight_scene_trials = 1U;
  legal.straight_scene_evaluated = true;
  auto rejected = legal;
  rejected.spline_candidate_available = false;
  rejected.native_scene_conflicts = 1U;
  rejected.native_scene_total_conflicts = 1U;
  rejected.straight_scene_drc_violations = 1U;
  require(flux::regional_checkpoint_better(legal, rejected),
          "a complete legal candidate must beat every rejected scene");
}

void straight_track_angles_preserve_off_grid_endpoints_and_clearance() {
  auto board = two_pad_board();
  board.footprints.back().position = mm(80.3, 42.7);
  Keepout first_elbow_obstacle;
  first_elbow_obstacle.id = ObjectId{"first-elbow-obstacle"};
  first_elbow_obstacle.area = rectangle(20.8, 40.8, 21.2, 41.2);
  first_elbow_obstacle.layers = {LayerId{"F.Cu"}};
  first_elbow_obstacle.prohibit.tracks = true;
  board.keepouts.push_back(first_elbow_obstacle);
  flux::GlobalRouteResult global;
  global.stop_reason = flux::GlobalRouterStopReason::complete;
  global.paths = {flux::GlobalRoutePath{NetId{"net-1"}, 0U,
                  {{mm(20.0, 40.0), LayerId{"F.Cu"}},
                   {mm(80.3, 42.7), LayerId{"F.Cu"}}}}};
  flux::DetailedRouterConfig config;
  config.continuous_refinement = false;
  config.pull_tight_45_degrees = false;
  config.restrict_straight_track_angles = true;
  const auto detailed = flux::materialize_detailed_route(board, global, config);
  const auto ordinary = [](const auto& track) {
    const auto dx = std::abs(track.end.x.value() - track.start.x.value());
    const auto dy = std::abs(track.end.y.value() - track.start.y.value());
    return dx == 0 || dy == 0 || dx == dy;
  };
  require(detailed.complete() && detailed.candidate && detailed.copper_drc.ok() &&
              detailed.connectivity.open_connections == 0U &&
              detailed.straight_angle_segments_converted == 1U &&
              detailed.straight_angle_candidate_checks >= 2U &&
              detailed.candidate->tracks.size() == 2U && detailed.candidate->track_arcs.empty() &&
              std::all_of(detailed.candidate->tracks.begin(), detailed.candidate->tracks.end(), ordinary),
          "an obstructed first octilinear elbow must select a legal alternate while retaining "
          "exact off-grid pad contacts: " + detailed.diagnostic);
  auto anchored = board;
  const auto via_position = mm(50.15, 41.35);
  anchored.vias.push_back(flux::Via{ObjectId{"mid-span-via"}, NetId{"net-1"}, via_position,
                                   Nanometres::from_mm(0.6), Nanometres::from_mm(0.3),
                                   {LayerId{"F.Cu"}, LayerId{"B.Cu"}}, false});
  const auto with_via = flux::materialize_detailed_route(anchored, global, config);
  require(with_via.complete() && with_via.candidate && with_via.copper_drc.ok() &&
              with_via.candidate->vias == anchored.vias &&
              std::count_if(with_via.candidate->tracks.begin(), with_via.candidate->tracks.end(),
                            [&](const auto& track) {
                              return track.start == via_position || track.end == via_position;
                            }) >= 2 &&
              std::all_of(with_via.candidate->tracks.begin(), with_via.candidate->tracks.end(), ordinary),
          "angle conversion must keep an exact interior same-net via as an electrical junction");
  auto blocked = board;
  blocked.keepouts.front().area = rectangle(49.0, 39.0, 51.0, 44.0);
  const auto rejected = flux::materialize_detailed_route(blocked, global, config);
  require(!rejected.complete() && rejected.straight_angle_segments_unresolved == 1U,
          "failure of all bounded ordinary-angle adapters must never accept arbitrary copper");
}

void soft_capacity_cuts_exclude_legally_separated_shadows() {
  Board hard_capacity_source;
  hard_capacity_source.source_revision = "hard-capacity-risk-fixture";
  hard_capacity_source.outline = rectangle(0.0, 0.0, 100.0, 80.0);
  hard_capacity_source.layers = {{LayerId{"F.Cu"}, "F.Cu", LayerKind::copper, 0}};
  hard_capacity_source.nets = {{NetId{"hard-a"}, "HARD_A", 0}, {NetId{"hard-b"}, "HARD_B", 0}};
  hard_capacity_source.rules.default_track_width = Nanometres::from_mm(0.35);
  const auto hard_terminal = [](std::string id, const NetId& net, Point position) {
    Footprint footprint;
    footprint.id = ObjectId{id};
    footprint.reference = id;
    footprint.position = position;
    footprint.courtyard = local_rectangle(0.1, 0.1);
    flux::Pad pad;
    pad.id = ObjectId{id + "-pad"};
    pad.net = net;
    pad.layers = {LayerId{"F.Cu"}};
    pad.shape = flux::PadShape::rectangle;
    pad.width = Nanometres::from_mm(0.1);
    pad.height = Nanometres::from_mm(0.1);
    footprint.pads.push_back(std::move(pad));
    return footprint;
  };
  hard_capacity_source.footprints = {hard_terminal("HA1", NetId{"hard-a"}, mm(20.0, 35.0)),
                                     hard_terminal("HA2", NetId{"hard-a"}, mm(80.0, 35.0)),
                                     hard_terminal("HB1", NetId{"hard-b"}, mm(20.0, 45.0)),
                                     hard_terminal("HB2", NetId{"hard-b"}, mm(80.0, 45.0))};
  Keepout hard_upper;
  hard_upper.id = ObjectId{"hard-upper"};
  hard_upper.area = rectangle(30.0, 0.0, 70.0, 39.3);
  hard_upper.layers = {LayerId{"F.Cu"}};
  hard_upper.prohibit.tracks = true;
  hard_upper.prohibit.vias = true;
  auto hard_lower = hard_upper;
  hard_lower.id = ObjectId{"hard-lower"};
  hard_lower.area = rectangle(30.0, 40.7, 70.0, 80.0);
  hard_capacity_source.keepouts = {std::move(hard_upper), std::move(hard_lower)};
  GlobalRouterConfig hard_capacity_config;
  hard_capacity_config.grid_pitch = Nanometres::from_mm(1.0);
  hard_capacity_config.max_iterations = 2U;
  hard_capacity_config.max_expansions_per_connection = 50'000U;
  // Two ordinary seeds plus the mandatory hydraulic family; RRT is disabled below.
  hard_capacity_config.max_static_seed_candidates_per_connection = 3U;
  hard_capacity_config.use_static_seed_rrt_connect_alternatives = false;
  hard_capacity_config.use_voronoi_allocation_resources = true;
  hard_capacity_config.atomic_complete_net_routing = true;
  hard_capacity_config.hard_capacity_start_iteration = 2U;
  hard_capacity_config.max_hard_capacity_ripup_attempts_per_pass = 2U;
  hard_capacity_config.max_hard_capacity_ripup_nets_per_attempt = 1U;
  hard_capacity_config.max_hard_capacity_conflict_set_states_per_attempt = 16U;
  hard_capacity_config.rollback_failed_hard_capacity_ripup = true;
  hard_capacity_config.defer_hard_capacity_ripup_victims_to_pass_tail = true;
  hard_capacity_source.rules.default_via_diameter = Nanometres::from_mm(3.0);
  const auto hard_capacity_route = flux::route_global(hard_capacity_source, hard_capacity_config);
  require(hard_capacity_route.hard_capacity_soft_cut_routes > 0U &&
              hard_capacity_route.hard_capacity_soft_cut_blockers > 0U &&
              hard_capacity_route.hard_capacity_soft_cut_legal_shadow_observations > 0U,
          "a broad via-sized shadow must retain true bottleneck blockers while excluding legal "
          "track-clearance observations: probes=" +
              std::to_string(hard_capacity_route.hard_capacity_soft_cut_routes) + ", blockers=" +
              std::to_string(hard_capacity_route.hard_capacity_soft_cut_blockers) + ", excluded=" +
              std::to_string(hard_capacity_route.hard_capacity_soft_cut_legal_shadow_observations));
}

void global_to_detailed_route_is_transactional_and_capacity_gated() {
  const auto source = two_pad_board();
  GlobalRouterConfig diversified_config;
  diversified_config.grid_pitch = Nanometres::from_mm(5.0);
  diversified_config.max_iterations = 1U;
  diversified_config.max_pin_escape_grid_radius = 2U;
  diversified_config.max_pin_access_nodes_per_terminal = 32U;
  diversified_config.diversify_pin_access_rings = true;
  diversified_config.hard_capacity_start_iteration = 1U;
  diversified_config.single_lane_direct_voronoi_resources = true;
  diversified_config.priority_nets = {NetId{"net-1"}};
  const auto diversified_route = flux::route_global(source, diversified_config);
  const auto inherited_corridor_capacity = std::any_of(
      diversified_route.resource_usage.begin(), diversified_route.resource_usage.end(),
      [](const auto& usage) {
        return usage.kind == flux::RoutingResourceKind::planar_edge && usage.capacity > 1U;
      });
  require(diversified_route.configured_diversify_pin_access_rings &&
              diversified_route.configured_single_lane_direct_voronoi_resources &&
              diversified_route.configured_priority_nets == diversified_config.priority_nets &&
              diversified_route.configured_residual_corridor_demand_cost ==
                  diversified_config.residual_corridor_demand_cost &&
              diversified_route.configured_critical_cut_cost ==
                  diversified_config.critical_cut_cost &&
              diversified_route.configured_critical_cut_minimum_future_demand ==
                  diversified_config.critical_cut_minimum_future_demand &&
              diversified_route.residual_corridor_demand_nets == 1U &&
              diversified_route.residual_corridor_demand_resources > 0U &&
              diversified_route.diversified_pin_access_nodes > 0U &&
              diversified_route.terminals_with_diversified_pin_access > 0U &&
              diversified_route.additional_successful_pin_access_rings > 0U &&
              diversified_route.failed_pin_escapes == 0U && inherited_corridor_capacity,
          "bounded later-ring portals must remain exact-legal and observable, and straight "
          "free-space samples must inherit physical parallel-lane corridor capacity");
  flux::RoutingPlannerConfig promoted_config;
  require(promoted_config.full_strength_first_pass,
          "the public planner must default to the full physics/repair policy");
  promoted_config.curved_tracks = true; // Explicit coverage of the retained curved planner.
  promoted_config.global = diversified_config;
  promoted_config.global.max_iterations = 2U;
  promoted_config.max_routability_feedback_retries = 1U;
  promoted_config.oracle_first_feedback = true;
  promoted_config.clearance_shadows_advisory = false;
  const auto promoted_plan = flux::plan_routes(source, promoted_config);
  const auto promoted_detail = flux::materialize_detailed_route(source, promoted_plan);
  const auto promoted_incremental =
      flux::materialize_detailed_route_incremental(source, promoted_plan, promoted_detail);
  flux::PersistentRoutingSessionConfig session_config;
  session_config.planner = promoted_config;
  const auto persistent_session =
      flux::run_persistent_routing_session(source, source, session_config);
  require(!promoted_plan.coarse_global.configured_clearance_shadows_advisory_only &&
              promoted_plan.complete() && promoted_plan.provisional_exact_promotions_attempted == 1U &&
              promoted_plan.provisional_exact_promotions_succeeded == 1U &&
              promoted_plan.feedback_probes.size() == 1U &&
              promoted_plan.feedback_probes.front().candidates.size() == 1U &&
              promoted_plan.feedback_probes.front().candidates.front().provisional_plan_complete &&
              promoted_plan.feedback_probes.front().candidates.front().exact_promotion_attempted &&
              promoted_plan.feedback_probes.front().candidates.front().exact_promotion_succeeded &&
              promoted_detail.complete() && promoted_incremental.complete() &&
              persistent_session.routing.complete() && persistent_session.detailed.complete(),
          "an iteration-zero discrete regional solution must be promoted through the full "
          "regional transaction before terminating, and its exact copper must support "
          "incremental rematerialization");

  // Exact interleaved epoch scheduling is retained as a legacy-control regression.
  auto full_strength_config = promoted_config;
  full_strength_config.oracle_first_feedback = false;
  full_strength_config.full_strength_first_pass = true;
  full_strength_config.global.max_iterations = 8U;
  full_strength_config.max_routability_feedback_retries = 2U;
  const auto full_strength_plan = flux::plan_routes_legacy(source, full_strength_config);
  require(full_strength_plan.complete() && !full_strength_plan.feedback_probes.empty() &&
              full_strength_plan.feedback_probes.front().global_epoch_iterations == 1U &&
              !full_strength_plan.feedback_probes.front().intermediate_regional_probe &&
              full_strength_plan.coarse_global.configured_bidirectional_route_growth_start_iteration == 1U &&
              full_strength_plan.coarse_global.configured_max_hard_capacity_ripup_attempts_per_pass >= 8U &&
              full_strength_plan.coarse_global.configured_global_navmesh_usped.enabled &&
              full_strength_plan.regional.configured.enable_spline_realization &&
              full_strength_plan.regional.configured.max_physical_scope_retries ==
                  full_strength_config.regional.max_physical_scope_retries &&
              !full_strength_plan.coarse_global.configured_clearance_shadows_advisory_only &&
              flux::materialize_detailed_route(source, full_strength_plan).complete(),
          "full-strength mode must certify the seed with its full scope budget before later negotiation");
  auto straight_config = full_strength_config;
  straight_config.curved_tracks = false;
  straight_config.global.single_lane_direct_voronoi_resources = false;
  const auto straight_plan = flux::plan_routes_legacy(source, straight_config);
  const auto straight_detail = flux::materialize_detailed_route(source, straight_plan);
  require(straight_plan.complete() && straight_detail.complete() &&
              !straight_plan.regional.configured.enable_spline_realization &&
              !straight_plan.coarse_global.configured_navmesh_funnel_spline_seeds &&
              straight_plan.coarse_global.configured_single_lane_direct_voronoi_resources &&
              straight_plan.provisional_exact_promotions_attempted == 0U &&
              straight_detail.added_track_arcs == 0U &&
              straight_detail.attempted_quintic_ph_runs == 0U &&
              flux::RoutingPlannerConfig{}.curved_tracks,
          "explicit straight routing must survive curved defaults and full-strength scheduling without spline promotion");
  auto one_epoch_config = full_strength_config;
  one_epoch_config.global.max_iterations = 1U;
  const auto one_epoch_plan = flux::plan_routes_legacy(source, one_epoch_config);
  require(one_epoch_plan.complete() && full_strength_plan.feedback_probes.size() == 1U &&
              full_strength_plan.paths == one_epoch_plan.paths,
          "a larger retry budget must preserve an already certifiable initial routing seed");
  full_strength_config.global.max_iterations = 2U;
  const auto short_full_strength = flux::plan_routes_legacy(source, full_strength_config);
  require(short_full_strength.complete() && short_full_strength.cumulative_global_iterations <= 2U &&
              short_full_strength.feedback_probes.front().global_epoch_iterations == 1U,
          "default full repair must clamp retry upper bounds without exceeding a short caller budget");

  // A regional PH candidate has already crossed the continuous-to-manufacturing boundary. Its
  // exact native primitive must survive detailed handoff rather than being reconstructed from the
  // diagnostic tessellation and optimized a second time.
  flux::RoutingPlanResult native_plan;
  native_plan.coarse_global = promoted_plan.coarse_global;
  native_plan.coarse_global.unrouted_nets.clear();
  native_plan.coarse_global.unaccepted_selected_voronoi_lane_transitions = 0U;
  native_plan.coarse_global.stop_reason = flux::GlobalRouterStopReason::complete;
  flux::GlobalRoutePath native_path{NetId{"net-1"},
                                    0U,
                                    {{mm(20.0, 40.0), LayerId{"F.Cu"}},
                                     {mm(50.0, 30.0), LayerId{"F.Cu"}},
                                     {mm(80.0, 40.0), LayerId{"F.Cu"}}}};
  native_plan.regional.spline_candidate_paths = {native_path};
  native_plan.regional.spline_candidate_available = true;
  // This handoff fixture represents an assessed regional transaction. Detailed materialization
  // independently checks the supplied native pieces, including the invalid variants below.
  native_plan.regional.native_scene_evaluated = true;
  native_plan.regional.native_scene_complete = true;
  flux::RegionalSplineRealization native_realization;
  native_realization.fragment = {NetId{"net-1"}, 0U, 0U, 0U};
  native_realization.replacement_end = mm(80.0, 40.0);
  native_realization.layer = LayerId{"F.Cu"};
  native_realization.centerline = {mm(20.0, 40.0), mm(50.0, 30.0), mm(80.0, 40.0)};
  native_realization.native_pieces = {
      flux::SplineFittedPiece{flux::SplineFittedPieceKind::circular_arc, mm(20.0, 40.0),
                              mm(50.0, 30.0), mm(80.0, 40.0), 0.0L, 1.0L}};
  native_realization.width = Nanometres::from_mm(0.25);
  native_realization.native_copper_legal = true;
  native_realization.mutually_legal = true;
  native_realization.quantized_mutually_legal = true;
  native_realization.topology_legal = true;
  native_realization.shape_legal = true;
  flux::RegionalBundleAssignment native_assignment;
  native_assignment.spline_realizations = {native_realization};
  native_assignment.stop_reason = flux::RegionalBundleStopReason::complete;
  native_plan.regional.bundle_assignments = {native_assignment};
  flux::RegionalRoutingWindow initial_native_owner;
  initial_native_owner.id = native_assignment.window_id;
  initial_native_owner.owned_fragments = {native_realization.fragment};
  native_plan.regional.windows = {initial_native_owner};
  native_plan.paths = {native_path};
  native_plan.stop_reason = flux::RoutingPlanStopReason::complete;
  flux::DetailedRouterConfig native_config;
  native_config.track_width = Nanometres::from_mm(0.25);
  native_config.circular_arc_refinement = true;
  native_config.pull_tight_45_degrees = false;
  native_config.continuous_refinement = false;
  const auto native_detail = flux::materialize_detailed_route(source, native_plan, native_config);
  require(native_detail.complete() && native_detail.candidate &&
              native_detail.regional_native_spline_runs_materialized == 1U &&
              native_detail.regional_native_track_arcs_materialized == 1U &&
              native_detail.candidate->track_arcs.size() == 1U &&
              native_detail.candidate->track_arcs.front().start == mm(20.0, 40.0) &&
              native_detail.candidate->track_arcs.front().middle == mm(50.0, 30.0) &&
              native_detail.candidate->track_arcs.front().end == mm(80.0, 40.0),
          "the exact regional native arc must cross detailed materialization byte-for-byte as "
          "connected DRC-clean copper");
  auto native_adapter_plan = native_plan;
  auto first_adapter = native_realization;
  first_adapter.centerline = {mm(20.0, 40.0), mm(50.0, 30.0)};
  first_adapter.replacement_end = mm(50.0, 30.0);
  first_adapter.native_pieces = {
      flux::SplineFittedPiece{flux::SplineFittedPieceKind::segment, mm(20.0, 40.0),
                              mm(35.0, 35.0), mm(50.0, 30.0), 0.0L, 1.0L}};
  auto second_adapter = native_realization;
  second_adapter.fragment.fragment_index = 1U;
  second_adapter.centerline = {mm(50.0, 30.0), mm(80.0, 40.0)};
  second_adapter.native_pieces = {
      flux::SplineFittedPiece{flux::SplineFittedPieceKind::segment, mm(50.0, 30.0),
                              mm(65.0, 35.0), mm(80.0, 40.0), 0.0L, 1.0L}};
  native_adapter_plan.regional.bundle_assignments.front().spline_realizations = {first_adapter, second_adapter};
  native_adapter_plan.regional.windows.front().owned_fragments = {first_adapter.fragment, second_adapter.fragment};
  const auto native_adapter_detail =
      flux::materialize_detailed_route(source, native_adapter_plan, native_config);
  require(native_adapter_detail.complete() && native_adapter_detail.candidate &&
              native_adapter_detail.regional_native_track_segments_materialized == 2U &&
              native_adapter_detail.fitted_cubic_beziers > 0U &&
              native_adapter_detail.unrounded_non_junction_bends == 0U &&
              !native_adapter_detail.candidate->track_beziers.empty() &&
              native_adapter_detail.connectivity.open_connections == 0U &&
              native_adapter_detail.copper_drc.ok(),
          "native straight adapters must enter board-level PH realization as connected cubic copper instead of freezing a degree-two bend: " +
              native_adapter_detail.diagnostic + "; native_segments=" +
              std::to_string(native_adapter_detail.regional_native_track_segments_materialized) +
              "; tasks=" + std::to_string(native_adapter_detail.spline_parallel_route_tasks));
  auto fallback_adapter_config = native_config;
  fallback_adapter_config.max_spline_navmesh_search_expansions_per_run = 1U;
  fallback_adapter_config.spline_minimum_bend_radius = Nanometres::from_mm(1.0);
  const auto fallback_adapter_detail =
      flux::materialize_detailed_route(source, native_adapter_plan, fallback_adapter_config);
  require(fallback_adapter_detail.complete() &&
              fallback_adapter_detail.spline_navmesh_certificate_fallbacks > 0U &&
              fallback_adapter_detail.unrounded_non_junction_bends == 0U,
          "fallback corridor must certify enough legal width for the required bend radius: " +
              fallback_adapter_detail.diagnostic);
  auto limited_native = native_config;
  limited_native.max_circular_arcs = 0U;
  require(!flux::materialize_detailed_route(source, native_plan, limited_native).complete(),
          "regional native primitives must obey the aggregate detailed arc budget");
  limited_native = native_config;
  limited_native.spline_arc_fit_tolerance = Nanometres{1'000};
  require(!flux::materialize_detailed_route(source, native_plan, limited_native).complete(),
          "a coarser regional fit certificate must not satisfy a tighter detailed policy");
  auto shared_native_plan = native_plan;
  flux::RegionalRoutingWindow owner_window;
  owner_window.id = 10U;
  owner_window.owned_fragments = {native_realization.fragment};
  flux::RegionalRoutingWindow context_window;
  context_window.id = 11U;
  shared_native_plan.regional.windows = {owner_window, context_window};
  shared_native_plan.regional.bundle_assignments.front().window_id = owner_window.id;
  auto context_assignment = native_assignment;
  context_assignment.window_id = context_window.id;
  // Frozen context may differ from the owner's eventual curve. It must never be emitted.
  context_assignment.spline_realizations.front().centerline[1] = mm(50.0, 32.0);
  shared_native_plan.regional.bundle_assignments.push_back(context_assignment);
  const auto shared_native =
      flux::materialize_detailed_route(source, shared_native_plan, native_config);
  require(shared_native.complete() && shared_native.regional_native_spline_runs_materialized == 1U,
          "a shared regional boundary must have exactly one owning copper writer");
  auto reversed_native_plan = native_plan;
  std::reverse(reversed_native_plan.regional.spline_candidate_paths.front().points.begin(),
               reversed_native_plan.regional.spline_candidate_paths.front().points.end());
  reversed_native_plan.paths = reversed_native_plan.regional.spline_candidate_paths;
  const auto reversed_native =
      flux::materialize_detailed_route(source, reversed_native_plan, native_config);
  require(reversed_native.complete() &&
              reversed_native.candidate->track_arcs.front().start ==
                  native_realization.centerline.back() &&
              reversed_native.candidate->track_arcs.front().end ==
                  native_realization.centerline.front(),
          "native ownership must preserve exact copper under reversed path orientation");
  auto wrong_width_plan = native_plan;
  wrong_width_plan.regional.bundle_assignments.front().spline_realizations.front().width =
      Nanometres::from_mm(0.3);
  require(!flux::materialize_detailed_route(source, wrong_width_plan, native_config).complete(),
          "native geometry may not bypass selected route width provenance");
  auto scene_source = source;
  scene_source.nets.push_back({NetId{"outside-net"}, "OUTSIDE", 0U});
  // A local stub is owned, but its same-net continuation is still an obstacle to other nets.
  // Omitting the entire net makes the vertical replacement falsely appear locally legal.
  flux::RegionalRouteResult frozen_regional;
  flux::RegionalRouteFragment stub;
  stub.id = {NetId{"outside-net"}, 0U, 0U, 0U};
  stub.layer = LayerId{"F.Cu"};
  stub.start = mm(40.0, 35.0);
  stub.end = mm(45.0, 35.0);
  stub.width = Nanometres::from_mm(0.25);
  stub.clearance = Nanometres::from_mm(0.2);
  auto continuation = stub;
  continuation.id.fragment_index = 1U;
  continuation.start = stub.end;
  continuation.end = mm(60.0, 35.0);
  frozen_regional.fragments = {stub, continuation};
  flux::RegionalRoutingWindow frozen_window;
  frozen_window.id = 7U;
  frozen_window.owned_fragments = {stub.id};
  frozen_regional.windows = {frozen_window};
  const auto source_track_count = scene_source.tracks.size();
  auto frozen_scene = flux::build_regional_frozen_copper_context(scene_source, frozen_regional, 7U);
  require(frozen_scene && frozen_scene->tracks.size() == source_track_count + 1U &&
              frozen_scene->tracks.back().start == continuation.start &&
              scene_source.tracks.size() == source_track_count,
          "frozen context excludes only owned fragments and never mutates source copper");
  const flux::TrackSegment crossing{ObjectId{"frozen-test-candidate"}, source.nets.front().id,
      LayerId{"F.Cu"}, mm(50.0, 30.0), mm(50.0, 40.0), stub.width, false};
  frozen_scene->tracks.push_back(crossing);
  require(!flux::check_copper_rules(*frozen_scene).violations.empty(),
          "untouched same-net continuation must expose an otherwise invisible local crossing");
  frozen_regional.windows.front().owned_fragments.push_back(continuation.id);
  auto jointly_owned = flux::build_regional_frozen_copper_context(scene_source, frozen_regional, 7U);
  require(jointly_owned && jointly_owned->tracks.size() == source_track_count,
          "joint ownership removes both replaced fragments from frozen context");
  jointly_owned->tracks.push_back(crossing);
  require(flux::check_copper_rules(*jointly_owned).violations.empty(),
          "the synthetic candidate must be legal without the untouched continuation");
  require(!flux::build_regional_frozen_copper_context(scene_source, frozen_regional, 99U),
          "unknown frozen-context ownership must fail closed");
  frozen_regional.windows.front().owned_fragments = {stub.id};
  frozen_regional.fragments.back().layer = LayerId{"B.Cu"};
  auto other_layer = flux::build_regional_frozen_copper_context(scene_source, frozen_regional, 7U);
  other_layer->tracks.push_back(crossing);
  require(flux::check_copper_rules(*other_layer).violations.empty(),
          "frozen planar copper on another layer must not obstruct a legal crossing");
  std::map<std::string, flux::RegionalRouteFragmentId> frozen_provenance;
  const auto attributed_scene = flux::build_regional_frozen_copper_context(
      scene_source, frozen_regional, 7U, &frozen_provenance);
  require(attributed_scene && frozen_provenance.size() == 1U &&
              frozen_provenance.at(attributed_scene->tracks.back().id.value()) == continuation.id,
          "generated frozen-copper IDs must preserve exact fragment provenance");
  auto candidate_fragment = stub;
  candidate_fragment.id.net = source.nets.front().id;
  frozen_regional.fragments.push_back(candidate_fragment);
  frozen_regional.windows.front().owned_fragments.push_back(candidate_fragment.id);
  flux::RegionalBundleAssignment failed_frozen;
  failed_frozen.window_id = 7U;
  failed_frozen.stop_reason = flux::RegionalBundleStopReason::spline_realization_failed;
  failed_frozen.frozen_copper_conflicts.push_back({candidate_fragment.id, continuation.id,
      LayerId{"B.Cu"}, Nanometres{200000}, "synthetic-clearance"});
  frozen_regional.bundle_assignments = {failed_frozen};
  const auto outside_diagnoses = flux::diagnose_regional_routability(frozen_regional);
  require(std::any_of(outside_diagnoses.begin(), outside_diagnoses.end(), [&](const auto& d) {
            return d.kind == flux::RoutabilityFailureKind::shove_deadlock &&
                d.resource.layer == LayerId{"B.Cu"} &&
                d.clearance_deficit == Nanometres{200000} &&
                d.suggested_relaxation == flux::RoutabilityRelaxation::change_corridor &&
                std::find(d.resource.nets.begin(), d.resource.nets.end(), candidate_fragment.id.net) !=
                    d.resource.nets.end() &&
                std::find(d.resource.nets.begin(), d.resource.nets.end(), continuation.id.net) !=
                    d.resource.nets.end();
          }),
          "failed physical repair must feed both candidate and outside-blocker nets upstream");
  flux::RegionalRouterConfig scope_config;
  scope_config.frozen_copper_context = true;
  scope_config.expand_frozen_copper_scope = true;
  require(!flux::regional_frozen_scope_retry_pending(frozen_regional, scope_config, 2U),
          "early ownership expansion is opt-in until benchmarked");
  scope_config.prioritize_frozen_scope_expansion = true;
  require(flux::regional_frozen_scope_retry_pending(frozen_regional, scope_config, 2U) &&
              scope_config.frozen_copper_scope_witnesses.empty(),
          "a new outside witness schedules expansion without mutating the current scope");
  require(!flux::regional_frozen_scope_retry_pending(frozen_regional, scope_config, 0U),
          "the final allowed scope must retain all same-scope repair algorithms");
  frozen_regional.spline_candidate_available = true;
  require(!flux::regional_frozen_scope_retry_pending(frozen_regional, scope_config, 2U),
          "a certified candidate must not be replaced by a scope retry");
  frozen_regional.spline_candidate_available = false;
  require(flux::append_regional_frozen_scope_witnesses(frozen_regional, scope_config) &&
              scope_config.frozen_copper_scope_witnesses.size() == 1U &&
              scope_config.frozen_copper_scope_witnesses.front().blocker == continuation.id,
          "an unresolved candidate must carry its exact outside fragment across layer changes");
  require(!flux::append_regional_frozen_scope_witnesses(frozen_regional, scope_config),
          "repeated identical frozen conflicts must not spend another scope retry");
  require(!flux::regional_frozen_scope_retry_pending(frozen_regional, scope_config, 1U),
          "duplicate witnesses must not bypass the remaining local repair search");
  scope_config.frozen_copper_scope_witnesses.clear();
  frozen_regional.bundle_assignments.front().stop_reason = flux::RegionalBundleStopReason::complete;
  require(flux::diagnose_regional_routability(frozen_regional).empty(),
          "a now-legal candidate must not rip up its former outside blocker");
  require(!flux::append_regional_frozen_scope_witnesses(frozen_regional, scope_config),
          "rejections from a now-legal candidate must not enlarge scope");
  frozen_regional.bundle_assignments.front().stop_reason = flux::RegionalBundleStopReason::spline_realization_failed;
  frozen_regional.windows.front().owned_fragments.push_back(continuation.id);
  require(!flux::append_regional_frozen_scope_witnesses(frozen_regional, scope_config),
          "already jointly owned fragments must not be treated as outside blockers");
  auto scene_regional = native_plan.regional;
  scene_regional.native_realization_manifest_available = true;
  scene_regional.native_realization_manifest = {native_realization};
  flux::GlobalRoutePath outside_path{
      NetId{"outside-net"},
      0U,
      {{mm(50.0, 20.0), LayerId{"F.Cu"}}, {mm(50.0, 50.0), LayerId{"F.Cu"}}}};
  scene_regional.spline_candidate_paths.push_back(outside_path);
  flux::assess_regional_native_scene(scene_source, scene_regional);
  require(
      scene_regional.native_scene_conflicts > 0U &&
          scene_regional.native_scene_fixed_conflicts == 0U &&
          !scene_regional.spline_candidate_available && !scene_regional.complete() &&
          !scene_regional.local_routability_diagnoses.empty(),
      "outside-window provisional copper must become a repair obligation, not an accepted handoff");
  require(std::any_of(scene_regional.local_routability_diagnoses.begin(),
                      scene_regional.local_routability_diagnoses.end(), [](const auto& diagnosis) {
        return diagnosis.resource.stable_id.starts_with("native-scene:") &&
               diagnosis.detail.find("object=") != std::string::npos &&
               diagnosis.detail.find("provisional=yes") != std::string::npos;
      }), "last native-scene diagnoses must preserve materialized object provenance");
  auto outside_native = native_realization;
  outside_native.fragment.net = outside_path.net;
  outside_native.centerline = {mm(50.0, 20.0), mm(50.0, 50.0)};
  outside_native.native_pieces = {
      {flux::SplineFittedPieceKind::segment, mm(50.0, 20.0), {}, mm(50.0, 50.0), 0.0L, 1.0L}};
  scene_regional.native_realization_manifest.push_back(outside_native);
  flux::assess_regional_native_scene(scene_source, scene_regional);
  require(scene_regional.native_scene_fixed_conflicts > 0U &&
              !scene_regional.spline_candidate_available,
          "two independently legal native windows must not publish a mutually conflicting scene");
  auto via_scene = native_plan.regional;
  via_scene.native_realization_manifest_available = true;
  via_scene.native_realization_manifest = {native_realization};
  outside_path.points = {{mm(50.0, 20.0), LayerId{"F.Cu"}},
                         {mm(50.0, 30.0), LayerId{"F.Cu"}},
                         {mm(50.0, 30.0), LayerId{"B.Cu"}},
                         {mm(50.0, 50.0), LayerId{"B.Cu"}}};
  via_scene.spline_candidate_paths.push_back(outside_path);
  flux::assess_regional_native_scene(scene_source, via_scene);
  require(via_scene.native_scene_fixed_conflicts > 0U && !via_scene.spline_candidate_available,
          "proposed via copper must participate in the assembled native acceptance scene");
  // Planar centres fit, but a proposed via's larger envelope does not. This is the local
  // candidate-scene gap: both the new via and a neighbouring curve need checking together.
  auto via_source = scene_source;
  via_source.tracks.push_back({ObjectId{"__flux-candidate-via-0"}, source.nets.front().id,
      LayerId{"B.Cu"}, mm(40.0, 35.0), mm(60.0, 35.0), stub.width, false});
  flux::RegionalViaChoice proposed;
  proposed.net = NetId{"outside-net"};
  proposed.site = mm(50.0, 34.5);
  proposed.span = {LayerId{"F.Cu"}, LayerId{"B.Cu"}};
  const auto via_rules = flux::resolve_net_routing_rules(via_source, proposed.net);
  auto planar_scene = via_source;
  planar_scene.tracks.push_back({ObjectId{"planar-via-run"}, proposed.net, LayerId{"B.Cu"},
      mm(49.0, 34.5), mm(51.0, 34.5), stub.width, false});
  require(flux::check_copper_rules(planar_scene).violations.empty(),
          "parallel 0.25 mm traces 0.5 mm apart must fit the synthetic channel");
  std::map<std::string, std::vector<flux::RegionalRouteFragmentId>> via_owners;
  std::vector<std::pair<flux::RegionalRouteFragmentId, flux::RegionalViaChoice>> selected_vias{
      {stub.id, proposed}, {continuation.id, proposed}};
  auto local_vias = flux::build_regional_candidate_via_context(via_source, selected_vias, via_owners);
  require(local_vias && local_vias->vias.size() == via_source.vias.size() + 1U &&
              via_owners.size() == 1U && via_owners.begin()->second.size() == 2U &&
              local_vias->vias.back().diameter == via_rules.via_diameter &&
              local_vias->vias.back().drill == via_rules.via_drill &&
              local_vias->vias.back().id != via_source.tracks.back().id &&
              via_source.vias.size() == scene_source.vias.size(),
          "local candidate vias use assembly rules, preserve source, and deduplicate all owners");
  const auto local_via_drc = flux::check_copper_rules(*local_vias);
  require(std::any_of(local_via_drc.violations.begin(), local_via_drc.violations.end(),
              [](const auto& violation) { return violation.kind == flux::CopperViolationKind::track_via_clearance; }),
          "candidate via copper must expose a collision invisible to planar local validation");
  auto reused_vias = flux::build_regional_candidate_via_context(*local_vias, selected_vias, via_owners);
  require(reused_vias && reused_vias->vias.size() == local_vias->vias.size() && via_owners.empty(),
          "existing same-net span-covering vias must be reused, not duplicated");
  auto pad_via_source = via_source;
  Footprint via_pad;
  via_pad.id = ObjectId{"candidate-via-pad-owner"};
  via_pad.position = proposed.site;
  flux::Pad plated_via_pad{ObjectId{"candidate-via-pad"}, proposed.net, Point{}, proposed.span};
  plated_via_pad.plated = true;
  plated_via_pad.drill_width = Nanometres::from_mm(0.6);
  plated_via_pad.drill_height = plated_via_pad.drill_width;
  via_pad.pads.push_back(plated_via_pad);
  pad_via_source.footprints.push_back(via_pad);
  auto reused_pad = flux::build_regional_candidate_via_context(pad_via_source, selected_vias, via_owners);
  require(reused_pad && reused_pad->vias.size() == pad_via_source.vias.size() && via_owners.empty(),
          "a same-net plated hole covering the full span must replace a candidate via");
  pad_via_source.footprints.back().pads.back().plated = false;
  auto unplated_pad = flux::build_regional_candidate_via_context(pad_via_source, selected_vias, via_owners);
  require(unplated_pad && unplated_pad->vias.size() == pad_via_source.vias.size() + 1U,
          "an unplated hole must not falsely provide a copper layer transition");
  selected_vias.front().second.span = {LayerId{"F.Cu"}};
  require(!flux::build_regional_candidate_via_context(via_source, selected_vias, via_owners) &&
              via_owners.empty(), "invalid candidate via spans must fail closed");
  flux::RoutabilityDiagnosis local_diagnosis;
  local_diagnosis.suggested_relaxation =
      flux::RoutabilityRelaxation::increase_local_clearance_margin;
  flux::RoutabilityDiagnosis regional_diagnosis;
  regional_diagnosis.suggested_relaxation = flux::RoutabilityRelaxation::change_via_site;
  flux::RoutabilityDiagnosis global_diagnosis;
  global_diagnosis.suggested_relaxation = flux::RoutabilityRelaxation::replan_global_topology;
  require(flux::materialization_feedback_scope(local_diagnosis) ==
                  flux::MaterializationFeedbackResult::Scope::local_geometry &&
              flux::materialization_feedback_scope(regional_diagnosis) ==
                  flux::MaterializationFeedbackResult::Scope::regional &&
              flux::materialization_feedback_scope(global_diagnosis) ==
                  flux::MaterializationFeedbackResult::Scope::global,
          "exact failures must dispatch to their nearest capable feedback loop");
  auto branched_source = source;
  auto third_terminal = source.footprints.front();
  third_terminal.id = ObjectId{"U3-branch"};
  third_terminal.reference = "U3-branch";
  third_terminal.position = mm(20.0, 70.0);
  third_terminal.pads.front().id = ObjectId{"U3-branch-pad"};
  branched_source.footprints.push_back(std::move(third_terminal));
  const auto branched_route = flux::route_global(branched_source, diversified_config);
  flux::DetailedRouterConfig branched_detail_config;
  branched_detail_config.pull_tight_45_degrees = false;
  branched_detail_config.continuous_refinement = false;
  const auto branched_detail =
      flux::materialize_detailed_route(branched_source, branched_route, branched_detail_config);
  require(branched_route.complete() && branched_route.static_seed_complete_trees >= 1U &&
              branched_route.static_seed_tree_joins >= 1U &&
              branched_route.static_seed_tree_attachment_nodes >= 1U &&
              branched_route.iteration_zero_paths == 2U &&
              branched_detail.connectivity.open_connections == 0U,
          "later branches must attach only to the first terminal portal that acquired physical "
          "access copper: " +
              std::to_string(branched_route.paths.size()) + "/" +
              std::to_string(branched_detail.connectivity.open_connections) + "/" +
              branched_detail.diagnostic);
  auto paired_source = source;
  paired_source.nets.push_back(Net{NetId{"net-2"}, "SIGNAL_N", 0});
  paired_source.footprints[0].position = mm(20.0, 38.0);
  paired_source.footprints[1].position = mm(80.0, 38.0);
  paired_source.footprints[0].courtyard = local_rectangle(0.1, 0.1);
  paired_source.footprints[1].courtyard = local_rectangle(0.1, 0.1);
  auto negative_start = paired_source.footprints[0];
  negative_start.id = ObjectId{"U1N"};
  negative_start.reference = "U1N";
  negative_start.position = mm(20.0, 42.0);
  negative_start.pads[0].id = ObjectId{"U1N-pad"};
  negative_start.pads[0].net = NetId{"net-2"};
  auto negative_end = paired_source.footprints[1];
  negative_end.id = ObjectId{"U2N"};
  negative_end.reference = "U2N";
  negative_end.position = mm(80.0, 42.0);
  negative_end.pads[0].id = ObjectId{"U2N-pad"};
  negative_end.pads[0].net = NetId{"net-2"};
  paired_source.footprints.push_back(std::move(negative_start));
  paired_source.footprints.push_back(std::move(negative_end));
  paired_source.differential_pairs.push_back({flux::DifferentialPairId{"pair-1"}, "PAIR",
                                              NetId{"net-1"}, NetId{"net-2"},
                                              Nanometres::from_mm(0.2), std::nullopt});
  auto paired_config = diversified_config;
  paired_config.use_voronoi_allocation_resources = true;
  auto serial_paired_config = paired_config;
  serial_paired_config.max_static_seed_workers = 1U;
  const auto serial_paired_route = flux::route_global(paired_source, serial_paired_config);
  paired_config.max_static_seed_workers = 2U;
  const auto paired_route = flux::route_global(paired_source, paired_config);
  require(paired_route.complete() && paired_route.static_seed_joint_groups == 1U &&
              paired_route.static_seed_joint_group_candidates > 0U &&
              paired_route.static_seed_joint_groups_withheld == 0U &&
              serial_paired_route.paths == paired_route.paths &&
              serial_paired_route.static_seed_searches == paired_route.static_seed_searches &&
              serial_paired_route.static_seed_search_expansions ==
                  paired_route.static_seed_search_expansions &&
              paired_route.static_seed_tasks_dispatched == 2U &&
              serial_paired_route.static_seed_workers_used == 1U &&
              paired_route.static_seed_workers_used == 2U &&
              serial_paired_route.iteration_zero_seed_priority_evidence ==
                  paired_route.iteration_zero_seed_priority_evidence &&
              paired_route.iteration_zero_seed_priority_evidence.size() == 1U &&
              paired_route.iteration_zero_seed_priority_evidence.front().nets ==
                  std::vector<NetId>{NetId{"net-1"}, NetId{"net-2"}} &&
              paired_route.initial_pathfinder_order ==
                  std::vector<NetId>{NetId{"net-1"}, NetId{"net-2"}} &&
              paired_route.iteration_zero_staged_priority_qualified &&
              paired_route.iteration_zero_baseline_unrouted_nets ==
                  paired_route.iteration_zero_staged_unrouted_nets &&
              paired_route.iteration_zero_staged_clearance_shadow_conflicts <=
                  paired_route.iteration_zero_baseline_clearance_shadow_conflicts &&
              paired_route.configured_terminal_escape_reservation_cost > 0U &&
              paired_route.terminal_escape_reservation_terminals > 0U &&
              paired_route.terminal_escape_reservation_candidates > 0U &&
              paired_route.terminal_escape_reservation_resources > 0U &&
              paired_route.iteration_zero_trivial_groups +
                      paired_route.iteration_zero_constrained_groups +
                      paired_route.iteration_zero_ordinary_groups +
                      paired_route.iteration_zero_flexible_groups ==
                  1U,
          "declared differential members must enter iteration-zero planning only through a "
          "complete atomic priority-wave/layer-sequence candidate and remain contiguous in the "
          "first negotiated PathFinder pass, independent of worker "
          "scheduling: " +
              std::to_string(paired_route.complete()) + "/" +
              std::to_string(paired_route.static_seed_joint_groups) + "/" +
              std::to_string(paired_route.static_seed_joint_group_candidates) + "/" +
              std::to_string(paired_route.static_seed_joint_groups_withheld) + "/" +
              paired_route.diagnostic);
  auto bus_source = paired_source;
  bus_source.differential_pairs.clear();
  bus_source.routing_buses.push_back({flux::RoutingBusId{"bus-1"},
                                      "BUS",
                                      {NetId{"net-1"}, NetId{"net-2"}},
                                      Nanometres::from_mm(0.2)});
  const auto bus_route = flux::route_global(bus_source, paired_config);
  const auto bus_detail =
      flux::materialize_detailed_route(bus_source, bus_route, branched_detail_config);
  require(bus_route.complete() && bus_route.static_seed_joint_groups == 1U &&
              bus_route.static_seed_joint_group_candidates > 0U &&
              bus_route.static_seed_joint_groups_withheld == 0U && bus_detail.complete() &&
              bus_detail.candidate && bus_detail.connectivity.open_connections == 0U &&
              bus_detail.copper_drc.ok(),
          "declared bus members must retain one atomic ordered seed candidate through exact "
          "detailed acceptance: " +
              std::to_string(bus_route.static_seed_joint_group_candidates) + "/" +
              std::to_string(bus_detail.connectivity.open_connections) + "/" +
              std::to_string(bus_detail.copper_drc.violations.size()) + "/" +
              bus_detail.diagnostic);
  auto unassigned_pad_source = source;
  unassigned_pad_source.nets.front().name.clear();
  const auto unassigned_connectivity = flux::analyze_connectivity(unassigned_pad_source);
  require(unassigned_connectivity.open_connections == 0U &&
              unassigned_connectivity.nets.front().pad_count == 2U,
          "empty-name KiCad net-zero pads must remain inventoried but intentionally unconnected");
  auto physically_connected_source = source;
  physically_connected_source.tracks = {
      {ObjectId{"physical-track-a"}, NetId{"net-1"}, LayerId{"F.Cu"}, mm(20.5, 40.0),
       mm(49.8, 40.0), Nanometres::from_mm(0.4), false},
      {ObjectId{"physical-track-b"}, NetId{"net-1"}, LayerId{"F.Cu"}, mm(50.2, 40.0),
       mm(79.5, 40.0), Nanometres::from_mm(0.4), false}};
  const auto physical_connectivity = flux::analyze_connectivity(physically_connected_source);
  const auto preserved_route = flux::route_global(physically_connected_source, diversified_config);
  require(physical_connectivity.open_connections == 0U && preserved_route.complete() &&
              preserved_route.paths.empty(),
          "touching physical copper must connect pads and suppress redundant source-net rerouting");
  auto scoped_config = diversified_config;
  scoped_config.selected_nets = {NetId{"net-1"}};
  scoped_config.allow_diagonal_grid_edges = true;
  const auto explicitly_rerouted = flux::route_global(physically_connected_source, scoped_config);
  require(explicitly_rerouted.complete() && !explicitly_rerouted.paths.empty() &&
              explicitly_rerouted.configured_selected_nets == scoped_config.selected_nets &&
              explicitly_rerouted.configured_diagonal_grid_edges,
          "an exact selected-net scope must be observable and may deliberately replace a "
          "source-complete net while preserving all unselected copper");
  scoped_config.selected_nets = {NetId{"missing"}};
  const auto invalid_scope = flux::route_global(physically_connected_source, scoped_config);
  require(invalid_scope.stop_reason == GlobalRouterStopReason::invalid_input &&
              invalid_scope.diagnostic.find("unique IDs") != std::string::npos,
          "a selected-net scope must fail closed on a non-source net ID");
  GlobalRouterConfig first_ring_config = diversified_config;
  first_ring_config.diversify_pin_access_rings = false;
  const auto policy_changed_route =
      flux::route_global_incremental(source, diversified_route, {}, first_ring_config);
  require(policy_changed_route.reused_nets.empty() && policy_changed_route.reused_paths == 0U &&
              !policy_changed_route.rerouted_nets.empty(),
          "incremental routing must invalidate later-ring paths when access policy changes");

  auto terrain_changed_config = diversified_config;
  auto terrain_copper = source;
  terrain_copper.tracks.push_back({ObjectId{"live-terrain-test"}, NetId{"foreign-copper"},
                                   LayerId{"F.Cu"}, mm(48.0, 50.0), mm(48.0, 52.0),
                                   Nanometres::from_mm(0.25), false});
  terrain_changed_config.live_copper_terrain =
      flux::make_live_copper_terrain_snapshot(terrain_copper, true, 4096U);
  terrain_changed_config.live_copper_terrain_revision = 1U;
  const auto terrain_changed_route = flux::route_global_incremental(
      source, diversified_route, {}, terrain_changed_config, {}, nullptr, true);
  require(terrain_changed_route.reused_nets.empty() &&
              terrain_changed_route.reused_paths == 0U &&
              !terrain_changed_route.rerouted_nets.empty() &&
              terrain_changed_route.configured_live_copper_terrain ==
                  terrain_changed_config.live_copper_terrain,
          "changed copper terrain must regenerate even otherwise preserved incremental routes");
  const auto terrain_ripped_route = flux::route_global_incremental(
      source, terrain_changed_route, {}, diversified_config, {}, nullptr, true);
  require(terrain_ripped_route.reused_paths == 0U &&
              !terrain_ripped_route.rerouted_nets.empty() &&
              terrain_ripped_route.configured_live_copper_terrain.features.empty(),
          "ripped copper must invalidate old proposals and remove their occupancy snapshot");

  auto trapped_source = source;
  trapped_source.nets.push_back(Net{NetId{"blocker-net"}, "BLOCKER", 0});
  Footprint blocker;
  blocker.id = ObjectId{"U3"};
  blocker.reference = "U3";
  blocker.position = mm(50.0, 70.0);
  blocker.courtyard = local_rectangle(4.0, 3.0);
  flux::Pad blocker_pad;
  blocker_pad.id = ObjectId{"U3-pad"};
  blocker_pad.net = NetId{"blocker-net"};
  blocker_pad.local_position = mm(-30.0, -30.0);
  blocker_pad.layers = {LayerId{"F.Cu"}};
  blocker_pad.shape = flux::PadShape::rectangle;
  blocker_pad.width = Nanometres::from_mm(36.0);
  blocker_pad.height = Nanometres::from_mm(36.0);
  blocker.pads.push_back(std::move(blocker_pad));
  trapped_source.footprints.push_back(std::move(blocker));
  GlobalRouterConfig trapped_config;
  trapped_config.grid_pitch = Nanometres::from_mm(5.0);
  trapped_config.max_iterations = 1U;
  CollectingTelemetrySink trapped_telemetry;
  const auto trapped_route =
      flux::route_global(trapped_source, trapped_config, {}, &trapped_telemetry);
  require(std::any_of(trapped_route.pin_escape_failures.begin(),
                      trapped_route.pin_escape_failures.end(),
                      [](const auto& failure) {
                        return failure.footprint == ObjectId{"U1"} &&
                               failure.pad == ObjectId{"U1-pad"} && failure.net == NetId{"net-1"} &&
                               failure.position == mm(20.0, 40.0);
                      }),
          "failed pin escape evidence must retain stable component, pad, net, and position IDs");
  require(std::any_of(trapped_telemetry.events.begin(), trapped_telemetry.events.end(),
                      [](const auto& event) {
                        return event.kind == flux::RouterTelemetryEventKind::net_failed &&
                               event.net == NetId{"net-1"} &&
                               event.rejection_reason == "no_access_nodes" &&
                               event.x_nm == mm(20.0, 40.0).x.value() &&
                               event.y_nm == mm(20.0, 40.0).y.value() &&
                               event.detail.find("footprint=U1") != std::string::npos &&
                               event.detail.find("pad=U1-pad") != std::string::npos;
                      }),
          "failed pin escapes must publish a terminal-attributed NetFailed event for the live "
          "viewer");
  const auto trapped_feedback = flux::collect_integrated_routing_feedback(trapped_route);
  require(std::find(trapped_feedback.routing_obstacles.begin(),
                    trapped_feedback.routing_obstacles.end(),
                    ObjectId{"U1"}) != trapped_feedback.routing_obstacles.end() &&
              std::find(trapped_feedback.affected_nets.begin(),
                        trapped_feedback.affected_nets.end(),
                        NetId{"net-1"}) != trapped_feedback.affected_nets.end(),
          "failed escapes and unrouted nets must become integrated placement feedback");

  auto neck_down_source = source;
  neck_down_source.nets.push_back(Net{NetId{"neck-down-blocker-net"}, "BLOCKER", 0});
  neck_down_source.footprints.front().pads.front().local_position = mm(0.0, 0.3);
  neck_down_source.footprints.front().pads.front().width = Nanometres::from_mm(0.1);
  neck_down_source.footprints.front().pads.front().height = Nanometres::from_mm(0.1);
  Footprint neck_down_blocker;
  neck_down_blocker.id = ObjectId{"neck-down-blocker"};
  neck_down_blocker.reference = "U3";
  neck_down_blocker.position = mm(50.0, 70.0);
  neck_down_blocker.courtyard = local_rectangle(1.0, 1.0);
  flux::Pad neck_down_blocker_pad;
  neck_down_blocker_pad.id = ObjectId{"neck-down-blocker-pad"};
  neck_down_blocker_pad.net = NetId{"neck-down-blocker-net"};
  neck_down_blocker_pad.local_position = mm(-29.65, -29.85);
  neck_down_blocker_pad.layers = {LayerId{"F.Cu"}};
  neck_down_blocker_pad.width = Nanometres::from_mm(0.1);
  neck_down_blocker_pad.height = Nanometres::from_mm(0.1);
  neck_down_blocker.pads.push_back(std::move(neck_down_blocker_pad));
  neck_down_source.footprints.push_back(std::move(neck_down_blocker));
  GlobalRouterConfig neck_down_config;
  neck_down_config.grid_pitch = Nanometres::from_mm(5.0);
  neck_down_config.max_iterations = 1U;
  neck_down_config.terminal_neck_down_width = Nanometres::from_mm(0.1);
  neck_down_config.max_terminal_neck_down_length = Nanometres::from_mm(1.0);
  const auto neck_down_route = flux::route_global(neck_down_source, neck_down_config);
  const auto selected_blocker = std::find_if(
      neck_down_route.neck_down_obstacle_impacts.begin(),
      neck_down_route.neck_down_obstacle_impacts.end(), [](const auto& impact) {
        return impact.routed_net == NetId{"net-1"} &&
               impact.obstacle == ObjectId{"neck-down-blocker-pad"} &&
               impact.width_kind == flux::GlobalRouteSegmentWidthKind::terminal_neck_down &&
               impact.selected_width == Nanometres::from_mm(0.1);
      });
  const auto neck_down_feedback = flux::collect_integrated_routing_feedback(neck_down_route);
  require(
      neck_down_route.used_terminal_neck_down_segments > 0U &&
          selected_blocker != neck_down_route.neck_down_obstacle_impacts.end() &&
          neck_down_feedback.neck_down_obstacles == neck_down_route.neck_down_obstacle_impacts &&
          std::find(neck_down_feedback.routing_obstacles.begin(),
                    neck_down_feedback.routing_obstacles.end(),
                    ObjectId{"neck-down-blocker-pad"}) !=
              neck_down_feedback.routing_obstacles.end(),
      "a selected narrow segment must attribute its exact nominal-width blocker to placement: " +
          std::to_string(neck_down_route.used_terminal_neck_down_segments) + "/" +
          std::to_string(neck_down_route.terminal_neck_down_access_nodes) + "/" +
          std::to_string(neck_down_route.failed_pin_escapes) + "/" +
          std::to_string(neck_down_route.neck_down_obstacle_impacts.size()) + "/" +
          std::to_string(neck_down_feedback.neck_down_obstacles.size()) + "/" +
          neck_down_route.diagnostic);

  auto disconnected_source = source;
  Keepout wall;
  wall.id = ObjectId{"routing-wall"};
  wall.area = rectangle(45.0, 0.0, 55.0, 80.0);
  wall.layers = {LayerId{"F.Cu"}, LayerId{"B.Cu"}};
  wall.prohibit.tracks = true;
  wall.prohibit.vias = true;
  disconnected_source.keepouts.push_back(std::move(wall));
  GlobalRouterConfig disconnected_config;
  disconnected_config.grid_pitch = Nanometres::from_mm(5.0);
  disconnected_config.max_iterations = 1U;
  disconnected_config.max_expansions_per_connection = 10'000U;
  const auto disconnected_route = flux::route_global(disconnected_source, disconnected_config);
  const auto disconnected_failure = std::find_if(
      disconnected_route.connection_failures.begin(), disconnected_route.connection_failures.end(),
      [](const auto& failure) {
        return failure.net == NetId{"net-1"} &&
               failure.reason == flux::RoutingConnectionFailureReason::search_frontier_exhausted;
      });
  require(disconnected_failure != disconnected_route.connection_failures.end() &&
              disconnected_failure->attempted_terminals.size() == 1U &&
              disconnected_failure->attempted_terminals.front().footprint == ObjectId{"U2"} &&
              disconnected_failure->connected_terminals.size() == 1U &&
              disconnected_failure->connected_terminals.front().footprint == ObjectId{"U1"} &&
              disconnected_failure->expansions > 0U,
          "a disconnected search must retain its attempted terminal, tree root, and exact reason");
  const auto disconnected_feedback = flux::collect_integrated_routing_feedback(disconnected_route);
  require(std::find(disconnected_feedback.routing_obstacles.begin(),
                    disconnected_feedback.routing_obstacles.end(),
                    ObjectId{"U2"}) != disconnected_feedback.routing_obstacles.end(),
          "the failed connection endpoint must become a first-tier placement target");

  auto via_scarce_source = source;
  Keepout front_wall;
  front_wall.id = ObjectId{"front-layer-routing-wall"};
  front_wall.area = rectangle(45.0, 0.0, 55.0, 80.0);
  front_wall.layers = {LayerId{"F.Cu"}};
  front_wall.prohibit.tracks = true;
  via_scarce_source.keepouts.push_back(std::move(front_wall));
  GlobalRouterConfig via_scarce_config;
  via_scarce_config.grid_pitch = Nanometres::from_mm(5.0);
  via_scarce_config.via_site_pitch = Nanometres::from_mm(50.0);
  via_scarce_config.via_site_phase_x = 8U;
  via_scarce_config.via_site_phase_y = 8U;
  via_scarce_config.max_iterations = 1U;
  const auto via_scarce_route = flux::route_global(via_scarce_source, via_scarce_config);
  require(
      via_scarce_route.complete() &&
          via_scarce_route.iteration_zero_seed_priority_evidence.size() == 1U &&
          via_scarce_route.iteration_zero_seed_priority_evidence.front().primary_via_transitions >
              0U &&
          via_scarce_route.iteration_zero_seed_priority_evidence.front().viable_via_sites > 0U &&
          via_scarce_route.iteration_zero_seed_priority_evidence.front().wave ==
              flux::IterationZeroRoutingWave::constrained,
      "a route with a mandatory scarce layer transition must expose its via-site portfolio "
      "and enter the constrained iteration-zero wave: " +
          std::to_string(via_scarce_route.complete()) + "/" +
          std::to_string(via_scarce_route.iteration_zero_seed_priority_evidence.empty()
                             ? 0U
                             : via_scarce_route.iteration_zero_seed_priority_evidence.front()
                                   .primary_via_transitions) +
          "/" + via_scarce_route.diagnostic);

  auto crossing_source = two_pad_board();
  const NetId crossing_net{"crossing-net"};
  crossing_source.nets.push_back({crossing_net, "CROSSING", 0});
  const auto crossing_footprint = [&crossing_net](std::string id, Point position) {
    Footprint result;
    result.id = ObjectId{id};
    result.reference = id;
    result.position = position;
    result.courtyard = local_rectangle(4.0, 3.0);
    result.pads = {{ObjectId{id + "-pad"}, crossing_net, Point{}, {LayerId{"F.Cu"}}}};
    return result;
  };
  crossing_source.footprints.push_back(crossing_footprint("U3", mm(50.0, 10.0)));
  crossing_source.footprints.push_back(crossing_footprint("U4", mm(50.0, 70.0)));
  GlobalRouterConfig crossing_config;
  crossing_config.grid_pitch = Nanometres::from_mm(5.0);
  crossing_config.max_iterations = 1U;
  crossing_config.max_static_seed_workers = 1U;
  const auto crossing_route = flux::route_global(crossing_source, crossing_config);
  require(
      crossing_route.iteration_zero_seed_priority_evidence.size() == 2U &&
          std::all_of(crossing_route.iteration_zero_seed_priority_evidence.begin(),
                      crossing_route.iteration_zero_seed_priority_evidence.end(),
                      [](const auto& evidence) {
                        return evidence.terminal_crossing_pressure == 1U &&
                               evidence.wave != flux::IterationZeroRoutingWave::trivial;
                      }),
      "crossing terminal MST airwires must be observable and suppress a false trivial seed wave");

  auto refresh_source = source;
  refresh_source.layers = {{LayerId{"F.Cu"}, "F.Cu", LayerKind::copper, 0}};
  const NetId competing_net{"competing-net"};
  refresh_source.nets.push_back(Net{competing_net, "COMPETING", 0});
  for (std::size_t index = 0U; index < refresh_source.footprints.size(); ++index) {
    refresh_source.footprints[index].courtyard = local_rectangle(4.0, 8.0);
    refresh_source.footprints[index].pads.push_back(
        flux::Pad{ObjectId{"competing-pad-" + std::to_string(index)},
                  competing_net,
                  mm(0.0, 1.0),
                  {LayerId{"F.Cu"}}});
  }
  Keepout upper_channel_wall;
  upper_channel_wall.id = ObjectId{"upper-channel-wall"};
  upper_channel_wall.area = rectangle(0.0, 0.0, 100.0, 35.0);
  upper_channel_wall.layers = {LayerId{"F.Cu"}};
  upper_channel_wall.prohibit.tracks = true;
  upper_channel_wall.prohibit.vias = true;
  auto lower_channel_wall = upper_channel_wall;
  lower_channel_wall.id = ObjectId{"lower-channel-wall"};
  lower_channel_wall.area = rectangle(0.0, 45.0, 100.0, 80.0);
  refresh_source.keepouts = {std::move(upper_channel_wall), std::move(lower_channel_wall)};
  GlobalRouterConfig refresh_config;
  refresh_config.grid_pitch = Nanometres::from_mm(5.0);
  refresh_config.max_iterations = 3U;
  refresh_config.atomic_complete_net_routing = true;
  refresh_config.selective_congestion_reroute = true;
  refresh_config.selective_full_reroute_interval = 1U;
  const auto refreshed_route = flux::route_global(refresh_source, refresh_config);
  require(refreshed_route.configured_selective_full_reroute_interval == 1U &&
              refreshed_route.selective_reroute_passes == 2U &&
              refreshed_route.selective_full_reroute_refreshes == 2U &&
              !refreshed_route.selected_net_pressure.empty(),
          "periodic refresh must discard active reuse on schedule and publish selected pressure: " +
              std::to_string(refreshed_route.configured_selective_full_reroute_interval) + "/" +
              std::to_string(refreshed_route.selective_reroute_passes) + "/" +
              std::to_string(refreshed_route.selective_full_reroute_refreshes) + "/" +
              std::to_string(refreshed_route.selected_net_pressure.size()) + "/" +
              refreshed_route.diagnostic);

  Board hard_capacity_source;
  hard_capacity_source.source_revision = "hard-capacity-risk-fixture";
  hard_capacity_source.outline = rectangle(0.0, 0.0, 100.0, 80.0);
  hard_capacity_source.layers = {{LayerId{"F.Cu"}, "F.Cu", LayerKind::copper, 0}};
  hard_capacity_source.nets = {{NetId{"hard-a"}, "HARD_A", 0}, {NetId{"hard-b"}, "HARD_B", 0}};
  hard_capacity_source.rules.default_track_width = Nanometres::from_mm(0.35);
  const auto hard_terminal = [](std::string id, const NetId& net, Point position) {
    Footprint footprint;
    footprint.id = ObjectId{id};
    footprint.reference = id;
    footprint.position = position;
    footprint.courtyard = local_rectangle(0.1, 0.1);
    flux::Pad pad;
    pad.id = ObjectId{id + "-pad"};
    pad.net = net;
    pad.layers = {LayerId{"F.Cu"}};
    pad.shape = flux::PadShape::rectangle;
    pad.width = Nanometres::from_mm(0.1);
    pad.height = Nanometres::from_mm(0.1);
    footprint.pads.push_back(std::move(pad));
    return footprint;
  };
  hard_capacity_source.footprints = {hard_terminal("HA1", NetId{"hard-a"}, mm(20.0, 35.0)),
                                     hard_terminal("HA2", NetId{"hard-a"}, mm(80.0, 35.0)),
                                     hard_terminal("HB1", NetId{"hard-b"}, mm(20.0, 45.0)),
                                     hard_terminal("HB2", NetId{"hard-b"}, mm(80.0, 45.0))};
  Keepout hard_upper;
  hard_upper.id = ObjectId{"hard-upper"};
  hard_upper.area = rectangle(30.0, 0.0, 70.0, 39.3);
  hard_upper.layers = {LayerId{"F.Cu"}};
  hard_upper.prohibit.tracks = true;
  hard_upper.prohibit.vias = true;
  auto hard_lower = hard_upper;
  hard_lower.id = ObjectId{"hard-lower"};
  hard_lower.area = rectangle(30.0, 40.7, 70.0, 80.0);
  hard_capacity_source.keepouts = {std::move(hard_upper), std::move(hard_lower)};
  GlobalRouterConfig hard_capacity_config;
  hard_capacity_config.grid_pitch = Nanometres::from_mm(1.0);
  hard_capacity_config.max_iterations = 2U;
  hard_capacity_config.max_expansions_per_connection = 50'000U;
  // Two ordinary seeds plus the mandatory hydraulic family; RRT is disabled below.
  hard_capacity_config.max_static_seed_candidates_per_connection = 3U;
  hard_capacity_config.use_static_seed_rrt_connect_alternatives = false;
  hard_capacity_config.use_voronoi_allocation_resources = true;
  hard_capacity_config.atomic_complete_net_routing = true;
  hard_capacity_config.hard_capacity_start_iteration = 2U;
  hard_capacity_config.max_hard_capacity_ripup_attempts_per_pass = 2U;
  hard_capacity_config.max_hard_capacity_ripup_nets_per_attempt = 1U;
  hard_capacity_config.max_hard_capacity_conflict_set_states_per_attempt = 16U;
  hard_capacity_config.rollback_failed_hard_capacity_ripup = true;
  hard_capacity_config.defer_hard_capacity_ripup_victims_to_pass_tail = true;
  const auto hard_capacity_route = flux::route_global(hard_capacity_source, hard_capacity_config);
  require(
      hard_capacity_route.configured_hard_capacity_start_iteration == 2U &&
          hard_capacity_route.hard_capacity_passes == 1U &&
          hard_capacity_route.hard_sampled_capacity_rejected_transitions > 0U &&
          hard_capacity_route.hard_voronoi_capacity_rejected_transitions > 0U &&
          hard_capacity_route.hard_capacity_diversified_passes == 1U &&
          hard_capacity_route.hard_capacity_pass_summaries.size() == 1U &&
          hard_capacity_route.hard_capacity_pass_summaries.front().attempted_order.size() == 2U &&
          hard_capacity_route.hard_capacity_pass_summaries.front().unrouted_nets.size() == 1U &&
          hard_capacity_route.hard_capacity_net_failures.size() == 1U &&
          hard_capacity_route.hard_capacity_net_failures.front().failed_passes == 1U &&
          !hard_capacity_route.hard_capacity_blockers.empty() &&
          hard_capacity_route.hard_capacity_blockers.front().rejected_transitions > 0U &&
          hard_capacity_route.configured_max_hard_capacity_ripup_attempts_per_pass == 2U &&
          hard_capacity_route.configured_max_hard_capacity_ripup_nets_per_attempt == 1U &&
          hard_capacity_route.configured_max_hard_capacity_conflict_set_states_per_attempt == 16U &&
          hard_capacity_route.hard_capacity_conflict_set_states > 0U &&
          hard_capacity_route.hard_capacity_soft_cut_probes > 0U &&
          hard_capacity_route.hard_capacity_soft_cut_routes > 0U &&
          hard_capacity_route.hard_capacity_soft_cut_blockers > 0U &&
          hard_capacity_route.hard_capacity_joint_ripup_attempts == 0U &&
          hard_capacity_route.configured_rollback_failed_hard_capacity_ripup &&
          hard_capacity_route.hard_capacity_failed_retry_rollbacks == 0U &&
          hard_capacity_route.configured_defer_hard_capacity_ripup_victims_to_pass_tail &&
          hard_capacity_route.hard_capacity_tail_deferred_victims == 2U &&
          hard_capacity_route.hard_capacity_ripup_attempts == 2U &&
          hard_capacity_route.hard_capacity_ripped_nets == 2U &&
          hard_capacity_route.hard_capacity_ripup_recoveries == 2U &&
          hard_capacity_route.hard_capacity_ripup_retained_recoveries == 1U &&
          hard_capacity_route.configured_capacity_reserve_per_mille ==
              hard_capacity_config.capacity_reserve_per_mille &&
          hard_capacity_route.configured_capacity_reserve_cost ==
              hard_capacity_config.capacity_reserve_cost &&
          hard_capacity_route.capacity_reserve_penalty_events > 0U &&
          hard_capacity_route.capacity_reserve_penalty > 0U &&
          hard_capacity_route.configured_former_route_memory_cost > 0U &&
          hard_capacity_route.former_route_memory_ripped_paths > 0U &&
          hard_capacity_route.former_route_memory_edges > 0U &&
          hard_capacity_route.former_route_memory_nodes > 0U &&
          hard_capacity_route.hard_capacity_ripup_records.size() == 2U &&
          hard_capacity_route.unrouted_nets.empty() && hard_capacity_route.paths.size() == 2U &&
          hard_capacity_route.total_voronoi_overflow_nm > 0U,
      "a late hard-cap pass must reject aggregate shared-resource overflow while retaining "
      "the earlier checkpoint: " +
          std::to_string(hard_capacity_route.hard_capacity_passes) + "/" +
          std::to_string(hard_capacity_route.hard_sampled_capacity_rejected_transitions) + "/" +
          std::to_string(hard_capacity_route.hard_voronoi_capacity_rejected_transitions) + "/" +
          std::to_string(hard_capacity_route.hard_capacity_diversified_passes) + "/" +
          std::to_string(hard_capacity_route.hard_capacity_pass_summaries.size()) + "/" +
          std::to_string(hard_capacity_route.hard_capacity_net_failures.size()) + "/" +
          std::to_string(hard_capacity_route.hard_capacity_blockers.size()) + "/" +
          std::to_string(hard_capacity_route.hard_capacity_ripup_attempts) + "/" +
          std::to_string(hard_capacity_route.hard_capacity_ripped_nets) + "/" +
          std::to_string(hard_capacity_route.hard_capacity_ripup_recoveries) + "/" +
          std::to_string(hard_capacity_route.hard_capacity_ripup_retained_recoveries) + "/" +
          std::to_string(hard_capacity_route.hard_capacity_ripup_records.size()) + "/" +
          std::to_string(hard_capacity_route.configured_former_route_memory_cost) + "/" +
          std::to_string(hard_capacity_route.former_route_memory_ripped_paths) + "/" +
          std::to_string(hard_capacity_route.former_route_memory_edges) + "/" +
          std::to_string(hard_capacity_route.former_route_memory_penalty_events) + "/" +
          std::to_string(hard_capacity_route.former_route_memory_penalty) + "/" +
          std::to_string(hard_capacity_route.voronoi_resource_usage.size()) + "/" +
          std::to_string(hard_capacity_route.congested_voronoi_resources) + "/" +
          std::to_string(hard_capacity_route.total_voronoi_overflow_nm) + "/" +
          std::to_string(hard_capacity_route.total_overflow) + "/" +
          std::to_string(hard_capacity_route.unrouted_nets.size()) + "/" +
          std::to_string(hard_capacity_route.paths.size()) + "/" +
          std::to_string(hard_capacity_route.capacity_reserve_penalty_events) + "/" +
          std::to_string(hard_capacity_route.corridor_capacity_reserve_penalty_events) + "/" +
          std::to_string(hard_capacity_route.physical_capacity_reserve_penalty_events) + "/" +
          std::to_string(hard_capacity_route.clearance_shadow_reserve_penalty_events) + "/" +
          std::to_string(hard_capacity_route.capacity_reserve_penalty) + "/" +
          hard_capacity_route.diagnostic);

  const auto& hard_summary = hard_capacity_route.hard_capacity_pass_summaries.front();
  require(!hard_summary.connection_failures.empty() &&
              !hard_summary.connection_failures.front().attempted_terminals.empty(),
          "the strict-capacity fixture must retain an exact attempted terminal for placement "
          "feedback");
  const auto& hard_terminal_evidence =
      hard_summary.connection_failures.front().attempted_terminals.front();
  const auto hard_feedback = flux::collect_integrated_routing_feedback(hard_capacity_route);
  const auto feedback_has = [](const auto& values, const auto& value) {
    return std::find(values.begin(), values.end(), value) != values.end();
  };
  const auto& hard_blocker = hard_capacity_route.hard_capacity_blockers.front();
  const auto hard_pressure = flux::collect_integrated_footprint_pressure(
      hard_capacity_source, hard_capacity_route,
      GlobalRouteCheckpointObjective::dimensioned_physical_overflow);
  const auto attempted_pressure =
      std::find_if(hard_pressure.begin(), hard_pressure.end(), [&](const auto& pressure) {
        return pressure.footprint == hard_terminal_evidence.footprint;
      });
  require(feedback_has(hard_feedback.routing_obstacles, hard_terminal_evidence.footprint) &&
              feedback_has(hard_feedback.routing_obstacles, hard_terminal_evidence.pad) &&
              feedback_has(hard_feedback.affected_nets, hard_blocker.failed_net) &&
              feedback_has(hard_feedback.affected_nets, hard_blocker.blocking_net) &&
              attempted_pressure != hard_pressure.end() && attempted_pressure->blocks_route &&
              attempted_pressure->sampled_pressure > 0U,
          "integrated placement must consume strict failed-net, blocker, and terminal evidence "
          "even when the selected soft checkpoint itself routed every net");

  auto delayed_hard_capacity_config = hard_capacity_config;
  delayed_hard_capacity_config.max_iterations = 7U;
  delayed_hard_capacity_config.hard_capacity_start_iteration = 7U;
  const auto delayed_hard_capacity_route =
      flux::route_global(hard_capacity_source, delayed_hard_capacity_config);
  require(delayed_hard_capacity_route.hard_capacity_passes == 1U &&
              delayed_hard_capacity_route.hard_capacity_pass_summaries.size() == 1U &&
              delayed_hard_capacity_route.hard_capacity_pass_summaries.front().iteration == 7U,
          "soft stagnation must not pre-empt an explicitly scheduled hard-capacity phase: " +
              std::to_string(delayed_hard_capacity_route.iterations) + "/" +
              std::to_string(delayed_hard_capacity_route.hard_capacity_passes) + "/" +
              std::to_string(delayed_hard_capacity_route.hard_capacity_pass_summaries.size()) +
              "/" + delayed_hard_capacity_route.diagnostic);

  auto protected_recovery_config = hard_capacity_config;
  protected_recovery_config.protect_recovered_hard_capacity_ripup_nets = true;
  const auto protected_recovery_route =
      flux::route_global(hard_capacity_source, protected_recovery_config);
  require(
      protected_recovery_route.configured_protect_recovered_hard_capacity_ripup_nets &&
          protected_recovery_route.hard_capacity_protected_recoveries == 1U &&
          protected_recovery_route.hard_capacity_protected_victim_rejections > 0U &&
          protected_recovery_route.hard_capacity_ripup_attempts == 1U &&
          protected_recovery_route.hard_capacity_ripup_recoveries == 1U &&
          protected_recovery_route.hard_capacity_ripup_retained_recoveries == 1U &&
          protected_recovery_route.hard_capacity_pass_summaries.size() == 1U &&
          protected_recovery_route.hard_capacity_pass_summaries.front().unrouted_nets.size() == 1U,
      "same-pass recovery protection must retain the first successful repair instead of "
      "displacing it for an impossible reciprocal retry: " +
          std::to_string(protected_recovery_route.hard_capacity_protected_recoveries) + "/" +
          std::to_string(protected_recovery_route.hard_capacity_protected_victim_rejections) + "/" +
          std::to_string(protected_recovery_route.hard_capacity_ripup_attempts) + "/" +
          std::to_string(protected_recovery_route.hard_capacity_ripup_recoveries) + "/" +
          std::to_string(protected_recovery_route.hard_capacity_ripup_retained_recoveries) + "/" +
          (protected_recovery_route.hard_capacity_pass_summaries.empty()
               ? "0"
               : std::to_string(protected_recovery_route.hard_capacity_pass_summaries.front()
                                    .unrouted_nets.size())) +
          "/" + protected_recovery_route.diagnostic);

  Board rollback_source;
  rollback_source.source_revision = "failed-ripup-rollback-risk-fixture";
  rollback_source.outline = rectangle(0.0, 0.0, 120.0, 40.0);
  rollback_source.layers = {{LayerId{"F.Cu"}, "F.Cu", LayerKind::copper, 0}};
  rollback_source.nets = {{NetId{"gate-a"}, "GATE_A", 30},
                          {NetId{"gate-b"}, "GATE_B", 20},
                          {NetId{"gate-c"}, "GATE_C", 10}};
  rollback_source.rules.default_track_width = Nanometres::from_mm(0.35);
  rollback_source.footprints = {hard_terminal("GA1", NetId{"gate-a"}, mm(10.0, 10.0)),
                                hard_terminal("GA2", NetId{"gate-a"}, mm(50.0, 10.0)),
                                hard_terminal("GB1", NetId{"gate-b"}, mm(50.0, 30.0)),
                                hard_terminal("GB2", NetId{"gate-b"}, mm(110.0, 30.0)),
                                hard_terminal("GC1", NetId{"gate-c"}, mm(10.0, 20.0)),
                                hard_terminal("GC2", NetId{"gate-c"}, mm(110.0, 20.0))};
  const auto gate_wall = [](std::string id, Polygon area) {
    Keepout wall;
    wall.id = ObjectId{std::move(id)};
    wall.area = std::move(area);
    wall.layers = {LayerId{"F.Cu"}};
    wall.prohibit.tracks = true;
    wall.prohibit.vias = true;
    return wall;
  };
  rollback_source.keepouts = {gate_wall("gate-one-low", rectangle(30.0, 0.0, 31.0, 9.3)),
                              gate_wall("gate-one-high", rectangle(30.0, 10.7, 31.0, 40.0)),
                              gate_wall("gate-two-low", rectangle(70.0, 0.0, 71.0, 29.3)),
                              gate_wall("gate-two-high", rectangle(70.0, 30.7, 71.0, 40.0))};
  auto rollback_config = hard_capacity_config;
  rollback_config.max_hard_capacity_ripup_attempts_per_pass = 1U;
  rollback_config.reorder_nets_by_pathfinder_pressure = false;
  const auto rollback_route = flux::route_global(rollback_source, rollback_config);
  require(
      rollback_route.hard_capacity_ripup_attempts == 1U &&
          rollback_route.hard_capacity_ripup_recoveries == 0U &&
          rollback_route.hard_capacity_failed_retry_rollbacks == 1U &&
          rollback_route.hard_capacity_tail_deferred_victims == 1U &&
          rollback_route.hard_capacity_ripup_records.size() == 1U &&
          rollback_route.hard_capacity_ripup_records.front().rolled_back_after_failed_retry &&
          !rollback_route.hard_capacity_ripup_records.front().retry_completed &&
          rollback_route.hard_capacity_pass_summaries.size() == 1U &&
          rollback_route.hard_capacity_pass_summaries.front().unrouted_nets.size() == 1U &&
          rollback_route.unrouted_nets.empty() && rollback_route.paths.size() == 3U,
      "an unsuccessful one-victim retry must restore both successful gate allocations and retain "
      "only the failed net in strict evidence: " +
          std::to_string(rollback_route.hard_capacity_ripup_attempts) + "/" +
          std::to_string(rollback_route.hard_capacity_ripup_recoveries) + "/" +
          std::to_string(rollback_route.hard_capacity_failed_retry_rollbacks) + "/" +
          std::to_string(rollback_route.hard_capacity_pass_summaries.size()) + "/" +
          (rollback_route.hard_capacity_pass_summaries.empty()
               ? "0"
               : std::to_string(
                     rollback_route.hard_capacity_pass_summaries.front().unrouted_nets.size())) +
          "/" + std::to_string(rollback_route.unrouted_nets.size()) + "/" +
          std::to_string(rollback_route.paths.size()) + "/" + rollback_route.diagnostic);

  auto focused_reuse_config = rollback_config;
  focused_reuse_config.max_iterations = 3U;
  focused_reuse_config.selective_congestion_reroute = true;
  focused_reuse_config.max_hard_capacity_blocker_reuse_invalidations_per_failed_net = 1U;
  const auto focused_reuse_route = flux::route_global(rollback_source, focused_reuse_config);
  require(
      focused_reuse_route.configured_max_hard_capacity_blocker_reuse_invalidations_per_failed_net ==
              1U &&
          focused_reuse_route.hard_capacity_blocker_reuse_invalidations == 1U &&
          focused_reuse_route.selective_retained_net_observations > 0U &&
          focused_reuse_route.hard_capacity_pass_summaries.size() == 2U &&
          focused_reuse_route.hard_capacity_pass_summaries.back().attempted_order.size() == 2U,
      "selective strict reuse must retain the unrelated legal gate route and reroute only the "
      "failed net plus its highest-evidence blocker: " +
          std::to_string(
              focused_reuse_route
                  .configured_max_hard_capacity_blocker_reuse_invalidations_per_failed_net) +
          "/" + std::to_string(focused_reuse_route.hard_capacity_blocker_reuse_invalidations) +
          "/" + std::to_string(focused_reuse_route.selective_retained_net_observations) + "/" +
          std::to_string(focused_reuse_route.hard_capacity_pass_summaries.size()) + "/" +
          (focused_reuse_route.hard_capacity_pass_summaries.empty()
               ? "0"
               : std::to_string(focused_reuse_route.hard_capacity_pass_summaries.back()
                                    .attempted_order.size())) +
          "/" + focused_reuse_route.diagnostic);

  auto holed_zone_source = source;
  holed_zone_source.nets.push_back(Net{NetId{"holed-zone-net"}, "HOLED_ZONE", 0});
  flux::CopperZone holed_zone;
  holed_zone.id = ObjectId{"holed-zone"};
  holed_zone.net = NetId{"holed-zone-net"};
  holed_zone.fills = {
      flux::CopperZoneFill{LayerId{"F.Cu"},
                           rectangle(5.0, 5.0, 95.0, 75.0),
                           {rectangle(10.0, 10.0, 15.0, 15.0), rectangle(45.0, 35.0, 55.0, 45.0),
                            rectangle(85.0, 65.0, 90.0, 70.0)}}};
  holed_zone_source.zones.push_back(std::move(holed_zone));
  GlobalRouterConfig holed_zone_config;
  holed_zone_config.grid_pitch = Nanometres::from_mm(5.0);
  holed_zone_config.max_iterations = 1U;
  const auto holed_zone_route = flux::route_global(holed_zone_source, holed_zone_config);
  require(holed_zone_route.stop_reason != GlobalRouterStopReason::invalid_input &&
              holed_zone_route.zone_hole_broad_phase_rejections > 0U &&
              holed_zone_route.zone_hole_exact_checks > 0U &&
              holed_zone_route.zone_hole_broad_phase_rejections >
                  holed_zone_route.zone_hole_exact_checks,
          "holed-zone routing must retain exact checks while rejecting distant contours broadly");

  GlobalRouterConfig config;
  config.grid_pitch = Nanometres::from_mm(5.0);
  config.use_voronoi_allocation_resources = true;
  config.require_voronoi_topological_transitions = true;
  config.max_voronoi_resource_segment_length = Nanometres::from_mm(5.0);
  const auto global = flux::route_global(source, config);
  require(
      global.complete() && global.unrouted_nets.empty() && global.total_overflow == 0U &&
          global.configured_max_voronoi_resource_segment_length == Nanometres::from_mm(5.0) &&
          global.segmented_voronoi_source_resources > 0U &&
          global.additional_voronoi_segment_resources > 0U &&
          std::all_of(global.voronoi_resource_usage.begin(), global.voronoi_resource_usage.end(),
                      [](const auto& resource) {
                        return resource.start != resource.end &&
                               flux::geometry::point_distance_nm(resource.start, resource.end) <=
                                   static_cast<long double>(Nanometres::from_mm(5.0).value());
                      }),
      "segmented physical routing must produce bounded local resources and a feasible plan");

  const auto detailed = flux::materialize_detailed_route(source, global);
  require(detailed.complete() && detailed.candidate && detailed.added_tracks > 0U &&
              detailed.connectivity.open_connections == 0U && detailed.copper_drc.ok() &&
              source.tracks.empty(),
          "detailed routing must produce checked copper on a copy without mutating the source: " +
              std::to_string(static_cast<int>(detailed.stop_reason)) + "/" +
              std::to_string(detailed.candidate.has_value()) + "/" +
              std::to_string(detailed.added_tracks) + "/" + std::to_string(detailed.added_vias) +
              "/" + std::to_string(detailed.connectivity.open_connections) + "/" +
              std::to_string(detailed.copper_drc.violations.size()));

  auto overfull = global;
  overfull.total_overflow = 1U;
  const auto withheld = flux::materialize_detailed_route(source, overfull);
  require(withheld.stop_reason == flux::DetailedRouterStopReason::incomplete_global_route &&
              !withheld.candidate && withheld.added_tracks == 0U && withheld.added_vias == 0U,
          "abstract overflow must transactionally block copper materialization");

  auto refill_source = two_pad_board();
  refill_source.layers.resize(1U);
  refill_source.nets.push_back(Net{NetId{"plane-net"}, "PLANE", 0});
  flux::CopperZone blocking_fill;
  blocking_fill.id = ObjectId{"saved-plane-fill"};
  blocking_fill.net = NetId{"plane-net"};
  blocking_fill.fills = {
      flux::CopperZoneFill{LayerId{"F.Cu"}, rectangle(45.0, 0.0, 55.0, 80.0), {}}};
  refill_source.zones.push_back(std::move(blocking_fill));
  GlobalRouterConfig immutable_fill_config;
  immutable_fill_config.grid_pitch = Nanometres::from_mm(5.0);
  immutable_fill_config.max_iterations = 2U;
  immutable_fill_config.bidirectional_route_growth_start_iteration = 1U;
  const auto immutable_fill_route = flux::route_global(refill_source, immutable_fill_config);
  auto refillable_config = immutable_fill_config;
  refillable_config.treat_saved_zone_fills_as_refillable = true;
  const auto refillable_route = flux::route_global(refill_source, refillable_config);
  flux::DetailedRouterConfig refillable_detail_config;
  refillable_detail_config.pull_tight_45_degrees = false;
  refillable_detail_config.continuous_refinement = false;
  refillable_detail_config.defer_saved_zone_fill_clearance_to_refill = true;
  const auto refillable_detail =
      flux::materialize_detailed_route(refill_source, refillable_route, refillable_detail_config);

  auto unfilled_source = refill_source;
  unfilled_source.zones.front().fills.clear();
  unfilled_source.zones.front().requires_refill = true;
  const auto unfilled_validation = flux::validate_board(unfilled_source);
  const auto unfilled_route = flux::route_global(unfilled_source, refillable_config);
  const auto unfilled_default_detail =
      flux::materialize_detailed_route(unfilled_source, unfilled_route);
  const auto unfilled_provisional_detail =
      flux::materialize_detailed_route(unfilled_source, unfilled_route, refillable_detail_config);
  require(!immutable_fill_route.complete() && refillable_route.complete() &&
              immutable_fill_route.bidirectional_route_growth_enabled &&
              immutable_fill_route.configured_bidirectional_route_growth_start_iteration == 1U &&
              immutable_fill_route.bidirectional_route_growth_attempts > 0U &&
              immutable_fill_route.bidirectional_route_growth_accepted +
                      immutable_fill_route.bidirectional_route_growth_fallbacks ==
                  immutable_fill_route.bidirectional_route_growth_attempts &&
              refillable_detail.provisional_zone_refill_candidate() &&
              !refillable_detail.complete() && refillable_detail.candidate &&
              refillable_detail.connectivity.open_connections == 0U &&
              refillable_detail.copper_drc.ok(),
          "saved fills must remain immutable by default while explicit refill planning can only "
          "publish a provisional externally gated candidate");
  require(unfilled_validation.ok() && unfilled_validation.warning_count() == 1U &&
              unfilled_route.complete() &&
              unfilled_default_detail.stop_reason ==
                  flux::DetailedRouterStopReason::invalid_input &&
              !unfilled_default_detail.candidate &&
              unfilled_provisional_detail.provisional_zone_refill_candidate() &&
              unfilled_provisional_detail.candidate &&
              unfilled_provisional_detail.connectivity.open_connections == 0U &&
              unfilled_provisional_detail.copper_drc.ok(),
          "an unfilled source zone must remain a valid immutable marker while only the explicit "
          "refill workflow can publish provisional copper");

  auto contradictory_zone_source = unfilled_source;
  contradictory_zone_source.zones.front().fills = {
      flux::CopperZoneFill{LayerId{"F.Cu"}, rectangle(45.0, 0.0, 55.0, 80.0), {}}};
  const auto contradictory_validation = flux::validate_board(contradictory_zone_source);
  require(!contradictory_validation.ok() && std::any_of(contradictory_validation.issues.begin(),
                                                        contradictory_validation.issues.end(),
                                                        [](const auto& issue) {
                                                          return issue.code ==
                                                                 "contradictory_zone_refill_state";
                                                        }),
          "saved physical copper and a requires-refill marker must be mutually exclusive");
}

void arc_free_space_preserves_the_unswept_region() {
  auto board = two_pad_board();
  board.footprints.clear();
  const flux::TrackArc arc{
      ObjectId{"arc-obstacle"}, NetId{"net-1"}, LayerId{"F.Cu"},           mm(20.0, 30.0),
      mm(50.0, 31.0),           mm(80.0, 30.0), Nanometres::from_mm(0.25), false};
  const auto parameters = flux::geometry::circular_arc_parameters(arc.start, arc.middle, arc.end);
  require(parameters.has_value(), "shallow arc fixture must be valid");
  for (const bool reversed : {false, true}) {
    auto oriented = arc;
    if (reversed)
      std::swap(oriented.start, oriented.end);
    board.track_arcs = {oriented};
    const auto free = flux::extract_polygonal_board_free_space(
        board, {LayerId{"F.Cu"}, NetId{"net-1"}, true, false, false});
    require(free.complete() && !free.domain.obstacles.empty() &&
                free.domain.obstacles.size() == free.obstacle_ids.size() &&
                free.domain.obstacles.size() == free.obstacle_nets.size(),
            "arc envelope pieces must retain obstacle attribution");
    const auto covered = [&](Point point) {
      return std::any_of(free.domain.obstacles.begin(), free.domain.obstacles.end(),
                         [&](const auto& polygon) {
                           return flux::geometry::locate_point(polygon, point) !=
                                  flux::geometry::PointLocation::outside;
                         });
    };
    require(!covered(mm(50.0, 10.0)),
            "the unoccupied interior of a shallow arc's supporting circle must remain free");
    for (std::size_t sample = 0U; sample <= 100U; ++sample) {
      const auto angle = parameters->start_angle_radians +
                         parameters->sweep_radians * static_cast<long double>(sample) / 100.0L;
      for (const auto offset : {-125'000.0L, 0.0L, 125'000.0L}) {
        const auto radius = parameters->radius_nm + offset;
        const Point point{Nanometres{static_cast<std::int64_t>(
                              std::llround(parameters->center_x_nm + radius * std::cos(angle)))},
                          Nanometres{static_cast<std::int64_t>(
                              std::llround(parameters->center_y_nm + radius * std::sin(angle)))}};
        require(covered(point), "conservative arc envelope must contain both copper boundaries");
      }
    }
  }
}

void quintic_ph_geometry_preserves_invariants_and_exact_offsets() {
  using flux::SplineVector;
  const auto mm_value = [](long double value) {
    return value * static_cast<long double>(Nanometres::from_mm(1.0).value());
  };
  const std::vector<SplineVector> parameter_points{
      {0.0L, 0.0L}, {mm_value(1.0L), 0.0L}, {mm_value(5.0L), 0.0L}};
  const auto parameters = flux::select_quintic_ph_parameters(parameter_points, 0.5L);
  require(parameters.size() == parameter_points.size() && parameters.front() == 0.0L &&
              parameters.back() == 1.0L && parameters[1] > 0.0L && parameters[1] < 0.5L,
          "centripetal parameter selection must retain order and de-emphasize a long chord");

  auto local_rebuild = flux::make_quintic_ph_route(
      {mm(0.0, 0.0), mm(4.0, 1.0), mm(8.0, 0.0), mm(12.0, 1.0), mm(16.0, 0.0), mm(20.0, 0.5)});
  require(local_rebuild.has_value(), "local PH rebuild fixture must interpolate");
  auto complete_rebuild = *local_rebuild;
  local_rebuild->control_points[2U].y += mm_value(0.35L);
  complete_rebuild.control_points[2U].y += mm_value(0.35L);
  require(flux::rebuild_quintic_ph_route_around_control(*local_rebuild, 2U) &&
              flux::rebuild_quintic_ph_route(complete_rebuild),
          "one-node and complete PH rebuilds must both retain a valid prepared chain");
  for (std::size_t sample = 0U; sample <= 32U; ++sample) {
    const auto parameter = static_cast<long double>(sample) / 32.0L;
    const auto local = flux::evaluate_quintic_ph(*local_rebuild, parameter);
    const auto complete = flux::evaluate_quintic_ph(complete_rebuild, parameter);
    require(std::hypotl(local.x - complete.x, local.y - complete.y) < 1.0e-3L,
            "bounded one-node PH reconstruction must match a complete exact rebuild");
  }

  const auto segment = flux::interpolate_quintic_ph_segment(
      {0.0L, 0.0L}, {mm_value(10.0L), mm_value(4.0L)}, {mm_value(9.0L), 0.0L},
      {mm_value(7.0L), mm_value(4.0L)});
  require(segment.has_value() && segment->bezier.front() == SplineVector{0.0L, 0.0L} &&
              std::hypotl(segment->bezier.back().x - mm_value(10.0L),
                          segment->bezier.back().y - mm_value(4.0L)) < 0.5L &&
              segment->cusp_count == 0U &&
              flux::quintic_ph_arc_length(*segment) >= std::hypotl(mm_value(10.0L), mm_value(4.0L)),
          "PH Hermite interpolation must prefer a cusp-free endpoint-exact branch and expose "
          "exact polynomial length");
  for (std::size_t coefficient = 0U; coefficient < segment->hodograph_power.size(); ++coefficient) {
    const auto scale = static_cast<long double>(coefficient + 1U);
    require(
        std::hypotl(segment->position_power[coefficient + 1U].x * scale -
                        segment->hodograph_power[coefficient].x,
                    segment->position_power[coefficient + 1U].y * scale -
                        segment->hodograph_power[coefficient].y) < 1.0e-3L,
        "cached position power coefficients must differentiate to cached w-squared coefficients");
  }
  for (std::size_t coefficient = 0U; coefficient < segment->speed_power.size(); ++coefficient) {
    require(std::abs(segment->arc_length_power[coefficient + 1U] *
                         static_cast<long double>(coefficient + 1U) -
                     segment->speed_power[coefficient]) < 1.0e-3L,
            "cached exact arc-length power coefficients must differentiate to cached speed");
  }
  for (const auto parameter : {0.0L, 0.17L, 0.5L, 0.83L, 1.0L}) {
    const auto offset = flux::make_quintic_ph_offset(*segment, mm_value(0.25L));
    const auto base_route = flux::make_quintic_ph_route({mm(0.0, 0.0), mm(10.0, 4.0)});
    require(offset.regular && base_route.has_value(),
            "a regular PH quintic must have a rational offset");
    // Evaluate the same authoritative segment directly through a one-segment route.
    auto route = *base_route;
    route.segments = {*segment};
    route.segments.front().route_parameter_start = 0.0L;
    route.segments.front().route_parameter_end = 1.0L;
    const auto point = flux::evaluate_quintic_ph(route, parameter);
    const auto differential = flux::evaluate_quintic_ph_differential(route, parameter);
    const auto evaluate_vector_power = [parameter](const auto& coefficients) {
      SplineVector value{};
      for (auto iterator = coefficients.rbegin(); iterator != coefficients.rend(); ++iterator) {
        value.x = value.x * parameter + iterator->x;
        value.y = value.y * parameter + iterator->y;
      }
      return value;
    };
    const auto preimage = evaluate_vector_power(segment->preimage_power);
    const auto hodograph = evaluate_vector_power(segment->hodograph_power);
    require(
        std::hypotl(hodograph.x - (preimage.x * preimage.x - preimage.y * preimage.y),
                    hodograph.y - 2.0L * preimage.x * preimage.y) < 1.0e-3L &&
            std::hypotl(differential.tangent.x - hodograph.x,
                        differential.tangent.y - hodograph.y) < 1.0e-3L,
        "hot differential evaluation must read the prepared quadratic-w/quartic-w-squared cache");
    const auto offset_point = flux::evaluate_quintic_ph_offset(offset, parameter);
    require(std::abs(std::hypotl(offset_point.x - point.x, offset_point.y - point.y) -
                     mm_value(0.25L)) < 1.0e-4L,
            "degree-nine rational PH offset evaluation must preserve exact signed distance");
    require(point.x >= segment->bounds_minimum.x - 1.0e-3L &&
                point.x <= segment->bounds_maximum.x + 1.0e-3L &&
                point.y >= segment->bounds_minimum.y - 1.0e-3L &&
                point.y <= segment->bounds_maximum.y + 1.0e-3L,
            "cached algebraic PH extrema must bound every evaluated curve position");
  }
  const auto extrema = flux::quintic_ph_curvature_extrema(*segment);
  require(extrema.size() == segment->curvature_extremum_count && extrema.size() >= 2U &&
              extrema.front().parameter == 0.0L && extrema.back().parameter == 1.0L,
          "curvature analysis must include endpoints and every isolated interior extremum");

  const std::vector<flux::SplineSample> shallow_samples{
      {0.0L, {mm_value(53.0L), mm_value(66.31L)}},
      {0.5L, {mm_value(45.009929L), mm_value(65.546615L)}},
      {1.0L, {mm_value(39.51749L), mm_value(64.989416L)}}};
  const auto shallow_pieces = flux::fit_spline_samples_to_lines_and_arcs(
      shallow_samples, Nanometres::from_mm(0.02), 16U, 16U);
  require(shallow_pieces.size() == 1U &&
              shallow_pieces.front().kind == flux::SplineFittedPieceKind::segment,
          "near-collinear PH samples must collapse to one tolerance-bounded line instead of a "
          "huge-radius KiCad arc");

  auto chain =
      flux::make_quintic_ph_route({mm(0.0, 0.0), mm(8.0, 0.0), mm(14.0, 5.0), mm(22.0, 5.0)});
  require(chain.has_value() && chain->tangent_continuous && chain->segments.size() == 3U,
          "multi-segment PH interpolation must construct a C1 chain");
  flux::NativeSplineFitConfig one_pass_native_fit;
  one_pass_native_fit.maximum_refinement_attempts = 1U;
  flux::NativeSplineFitStats native_fit_stats;
  const auto native_fit =
      flux::fit_quintic_ph_to_native_copper(*chain, one_pass_native_fit, {}, &native_fit_stats);
  require(!native_fit.empty() && flux::native_spline_max_join_angle(native_fit) <= 0.035L,
          "native fitting must bound quantized arc/line join angles independently of PH G1/G2");
  require(native_fit_stats.attempts == 1U && native_fit_stats.peak_samples >= 2U,
          "native fitting diagnostics must expose the bounded refinement and sample workload");
  for (std::size_t sample = 0U; sample <= 128U; ++sample) {
    const auto evaluated =
        flux::evaluate_quintic_ph(*chain, static_cast<long double>(sample) / 128.0L);
    const Point point{Nanometres{static_cast<std::int64_t>(std::llround(evaluated.x))},
                      Nanometres{static_cast<std::int64_t>(std::llround(evaluated.y))}};
    auto distance = std::numeric_limits<long double>::infinity();
    for (const auto& piece : native_fit)
      distance = std::min(
          distance, piece.kind == flux::SplineFittedPieceKind::circular_arc
                        ? flux::geometry::point_circular_arc_distance_nm(point, piece.start,
                                                                         piece.middle, piece.end)
                        : flux::geometry::point_segment_distance_nm(point, piece.start, piece.end));
    require(distance <= 40'002.0L,
            "tangent-matched native fitting must also retain the PH positional tolerance");
  }
  one_pass_native_fit.maximum_pieces = 1U;
  require(flux::fit_quintic_ph_to_native_copper(*chain, one_pass_native_fit).empty(),
          "biarc fitting must fail closed when its native piece budget cannot fit the route");
  const std::vector<flux::SplineFittedPiece> kink{
      {flux::SplineFittedPieceKind::segment, mm(0.0, 0.0), {}, mm(1.0, 0.0), 0.0L, 0.5L},
      {flux::SplineFittedPieceKind::segment, mm(1.0, 0.0), {}, mm(1.0, 1.0), 0.5L, 1.0L}};
  require(flux::native_spline_max_join_angle(kink) > 1.5L,
          "native smoothness validation must detect positional-only right-angle joins");
  auto cancelled_chain = *chain;
  const auto original_segments = cancelled_chain.segments;
  std::stop_source cancelled_promotion;
  cancelled_promotion.request_stop();
  require(!flux::enforce_quintic_ph_continuity(cancelled_chain, flux::PHJoinContinuity::curvature,
                                               1.0e-10L, 24U, cancelled_promotion.get_token()) &&
              std::equal(cancelled_chain.segments.begin(), cancelled_chain.segments.end(),
                         original_segments.begin(), original_segments.end(),
                         [](const auto& a, const auto& b) {
                           return a.bezier == b.bezier && a.preimage == b.preimage;
                         }),
          "cancelled G2 promotion must leave the G1 candidate unchanged");
  for (std::size_t join = 1U; join < chain->segments.size(); ++join) {
    const auto& left = chain->segments[join - 1U].bezier;
    const auto& right = chain->segments[join].bezier;
    const auto left_span = chain->segments[join - 1U].route_parameter_end -
                           chain->segments[join - 1U].route_parameter_start;
    const auto right_span =
        chain->segments[join].route_parameter_end - chain->segments[join].route_parameter_start;
    const auto left_tangent = SplineVector{5.0L * (left[5].x - left[4].x) / left_span,
                                           5.0L * (left[5].y - left[4].y) / left_span};
    const auto right_tangent = SplineVector{5.0L * (right[1].x - right[0].x) / right_span,
                                            5.0L * (right[1].y - right[0].y) / right_span};
    require(std::hypotl(left_tangent.x - right_tangent.x, left_tangent.y - right_tangent.y) < 0.1L,
            "adjacent PH Bézier segments must share their endpoint derivative exactly");
  }
  const auto inflections =
      std::accumulate(chain->segments.begin(), chain->segments.end(), std::size_t{0U},
                      [](auto count, const auto& candidate) {
                        return count + flux::quintic_ph_inflections(candidate).size();
                      });
  require(inflections <= chain->segments.size(),
          "isolated PH inflections must be explicitly bounded rather than inferred from samples");

  const auto horizontal = flux::interpolate_quintic_ph_segment(
      {0.0L, 0.0L}, {mm_value(10.0L), 0.0L}, {mm_value(10.0L), 0.0L}, {mm_value(10.0L), 0.0L});
  const auto vertical = flux::interpolate_quintic_ph_segment(
      {mm_value(5.0L), -mm_value(5.0L)}, {mm_value(5.0L), mm_value(5.0L)}, {0.0L, mm_value(10.0L)},
      {0.0L, mm_value(10.0L)});
  require(horizontal && vertical, "orthogonal PH intersection fixtures must interpolate");
  const auto far_parallel = flux::interpolate_quintic_ph_segment(
      {0.0L, mm_value(2.0L)}, {mm_value(10.0L), mm_value(2.0L)},
      {mm_value(10.0L), 0.0L}, {mm_value(10.0L), 0.0L});
  require(far_parallel.has_value(), "parallel PH clearance fixture must interpolate");
  const auto horizontal_certificate = flux::prepare_quintic_ph_segment_certificate(*horizontal);
  const auto repeated_horizontal_certificate =
      flux::prepare_quintic_ph_segment_certificate(*horizontal);
  const auto vertical_certificate = flux::prepare_quintic_ph_segment_certificate(*vertical);
  const auto far_parallel_certificate =
      flux::prepare_quintic_ph_segment_certificate(*far_parallel);
  flux::PHCenterlineClearanceStats separated_stats;
  const auto separated_verdict = flux::certify_quintic_ph_segment_clearance(
      horizontal_certificate, far_parallel_certificate, mm_value(0.5L), Nanometres{10},
      16'384U, &separated_stats);
  flux::PHCenterlineClearanceStats collision_stats;
  const auto collision_verdict = flux::certify_quintic_ph_segment_clearance(
      horizontal_certificate, vertical_certificate, mm_value(0.5L), Nanometres{10}, 16'384U,
      &collision_stats);
  const auto tangent_parallel = flux::interpolate_quintic_ph_segment(
      {0.0L, mm_value(0.5L)}, {mm_value(10.0L), mm_value(0.5L)},
      {mm_value(10.0L), 0.0L}, {mm_value(10.0L), 0.0L});
  require(tangent_parallel.has_value(), "near-tangent PH precision fixture must interpolate");
  flux::PHCenterlineClearanceStats tangent_stats;
  const auto tangent_verdict = flux::certify_quintic_ph_segment_clearance(
      flux::prepare_quintic_ph_segment_certificate(*horizontal, 1U),
      flux::prepare_quintic_ph_segment_certificate(*tangent_parallel, 1U), mm_value(0.5L),
      Nanometres{0}, 16U, &tangent_stats);
  require(horizontal_certificate.geometry_fingerprint ==
                  repeated_horizontal_certificate.geometry_fingerprint &&
              separated_verdict == flux::PHCenterlineClearanceVerdict::separated &&
              separated_stats.double_separation_proofs > 0U &&
              collision_verdict == flux::PHCenterlineClearanceVerdict::colliding &&
              collision_stats.long_double_collision_proofs > 0U &&
              tangent_verdict == flux::PHCenterlineClearanceVerdict::ambiguous &&
              tangent_stats.multiprecision_comparisons > 0U,
          "prepared PH certificate trees must deterministically prove common separated and "
          "colliding curve pairs and reserve multiprecision for degenerate contacts before "
          "rational-offset isolation");
  flux::PHOffsetIntersectionStats offset_stats;
  const auto intersections =
      flux::intersect_quintic_ph_offsets(flux::make_quintic_ph_offset(*horizontal, 0.0L),
                                         flux::make_quintic_ph_offset(*vertical, 0.0L),
                                         Nanometres{10}, 18U, 64U, 1'000'000U, &offset_stats);
  require(std::any_of(intersections.begin(), intersections.end(),
                      [&](const auto& intersection) {
                        return std::hypotl(intersection.position.x - mm_value(5.0L),
                                           intersection.position.y) <= 10.0L;
                      }),
          "bounded rational-offset intersection must recover an exact transverse crossing");
  require(offset_stats.subdivision_nodes > 0U && offset_stats.isolated_leaf_pairs > 0U &&
              !offset_stats.node_budget_exhausted,
          "rational-offset certificates must expose subdivision work and budget exhaustion");
  flux::PHOffsetIntersectionStats exhausted_offset_stats;
  static_cast<void>(
      flux::intersect_quintic_ph_offsets(flux::make_quintic_ph_offset(*horizontal, 0.0L),
                                         flux::make_quintic_ph_offset(*vertical, 0.0L),
                                         Nanometres{10}, 18U, 64U, 1U, &exhausted_offset_stats));
  require(exhausted_offset_stats.node_budget_exhausted,
          "bounded rational-offset isolation must explicitly report an exhausted node budget");
  const auto displaced_intersections = flux::intersect_quintic_ph_offsets(
      flux::make_quintic_ph_offset(*horizontal, mm_value(0.2L)),
      flux::make_quintic_ph_offset(*vertical, -mm_value(0.3L)), Nanometres{10});
  require(std::any_of(displaced_intersections.begin(), displaced_intersections.end(),
                      [&](const auto& intersection) {
                        return std::hypotl(intersection.position.x - mm_value(5.3L),
                                           intersection.position.y - mm_value(0.2L)) <= 10.0L;
                      }),
          "non-zero degree-nine PH offsets must intersect at their displaced physical envelopes");
  const auto horizontal_hulls = flux::subdivide_quintic_ph_offset_bounds(
      flux::make_quintic_ph_offset(*horizontal, 0.0L), 4U, Nanometres{10});
  const auto far_vertical_hulls = flux::subdivide_quintic_ph_offset_bounds(
      flux::make_quintic_ph_offset(*vertical, mm_value(20.0L)), 4U, Nanometres{10});
  const auto hulls_overlap = [](const auto& a, const auto& b) {
    return a.maximum_x_nm >= b.minimum_x_nm && b.maximum_x_nm >= a.minimum_x_nm &&
           a.maximum_y_nm >= b.minimum_y_nm && b.maximum_y_nm >= a.minimum_y_nm;
  };
  require(horizontal_hulls.size() == 16U && far_vertical_hulls.size() == 16U &&
              std::none_of(horizontal_hulls.begin(), horizontal_hulls.end(),
                           [&](const auto& a) {
                             return std::any_of(far_vertical_hulls.begin(),
                                                far_vertical_hulls.end(),
                                                [&](const auto& b) { return hulls_overlap(a, b); });
                           }),
          "fixed-depth rational-offset hulls must conservatively reject separated curves");

  flux::PHInterpolationConfig g2;
  g2.continuity = flux::PHJoinContinuity::curvature;
  const auto g2_line = flux::make_quintic_ph_route({mm(0.0, 0.0), mm(5.0, 0.0), mm(10.0, 0.0)}, g2);
  require(g2_line && g2_line->curvature_continuous &&
              flux::quintic_ph_max_curvature_jump(*g2_line) <= g2.curvature_tolerance_per_nm,
          "explicit geometric-G2 PH joining must certify scalar curvature continuity");

  auto cell_crossing = flux::make_quintic_ph_route({mm(0.0, 0.0), mm(10.0, 0.0), mm(20.0, 0.0)});
  Polygon left_cell;
  left_cell.vertices = {mm(0.0, -1.0), mm(10.0, -1.0), mm(10.0, 1.0), mm(0.0, 1.0)};
  Polygon right_cell;
  right_cell.vertices = {mm(10.0, -1.0), mm(20.0, -1.0), mm(20.0, 1.0), mm(10.0, 1.0)};
  require(cell_crossing && flux::set_quintic_ph_corridor(*cell_crossing, {left_cell, right_cell}),
          "a PH route must retain a multi-cell certified corridor");
  const auto cell_certificate = flux::certify_quintic_ph_continuous_envelope(
      *cell_crossing, Nanometres{0}, [](flux::SplineVector) {
        return flux::SplineClearanceProbe{std::numeric_limits<long double>::infinity(), {}, true};
      });
  require(cell_certificate.legal && cell_certificate.corridor_violations == 0U,
          "a PH hull crossing a shared NavMesh cell boundary must be certified by the cell union");
  const auto certified_cell_boundary = [](flux::SplineVector) {
    return flux::SplineClearanceProbe{0.0L, {}, true};
  };
  const auto redundantly_inflated_certificate = flux::certify_quintic_ph_continuous_envelope(
      *cell_crossing, Nanometres{0}, certified_cell_boundary);
  flux::SplineContinuousEnvelopeConfig certified_corridor_config;
  certified_corridor_config.corridor_certifies_clearance = true;
  const auto certified_corridor_certificate = flux::certify_quintic_ph_continuous_envelope(
      *cell_crossing, Nanometres{0}, certified_cell_boundary, certified_corridor_config);
  require(!redundantly_inflated_certificate.legal && certified_corridor_certificate.legal,
          "a full-envelope-certified NavMesh cell must prove its contained PH hull without "
          "subtracting the copper clearance a second time");

  auto nearly_straight =
      flux::make_quintic_ph_route({Point{Nanometres{43'000'000}, Nanometres{44'000'000}},
                                   Point{Nanometres{42'853'109}, Nanometres{42'531'095}},
                                   Point{Nanometres{42'846'891}, Nanometres{42'468'905}},
                                   Point{Nanometres{42'700'000}, Nanometres{41'000'000}}});
  flux::SplineShapeCertificationConfig nearly_straight_shape;
  nearly_straight_shape.minimum_bend_radius = Nanometres::from_mm(0.25);
  const auto nearly_straight_certificate =
      nearly_straight ? flux::certify_quintic_ph_shape(*nearly_straight, nearly_straight_shape)
                      : flux::SplineShapeCertificate{};
  require(
      nearly_straight && nearly_straight_certificate.bend_radius_violations == 0U,
      "quantized nearly-collinear corridor points must not create artificial tight PH bends: " +
          std::to_string(static_cast<double>(nearly_straight_certificate.minimum_bend_radius_nm)));

  auto rubber_band = flux::make_quintic_ph_route(
      {mm(0.0, 0.0), mm(5.0, 1.0), mm(10.0, -1.0), mm(15.0, 1.0), mm(20.0, 0.0)});
  require(rubber_band.has_value(), "rubber-band fixture must form a PH chain");
  const auto control_roughness = [](const flux::QuinticPHRoute& route) {
    long double value = 0.0L;
    for (std::size_t point = 1U; point + 1U < route.control_points.size(); ++point) {
      const auto second_x = route.control_points[point - 1U].x -
                            2.0L * route.control_points[point].x +
                            route.control_points[point + 1U].x;
      const auto second_y = route.control_points[point - 1U].y -
                            2.0L * route.control_points[point].y +
                            route.control_points[point + 1U].y;
      value += std::hypotl(second_x, second_y);
    }
    return value;
  };
  const auto initial_roughness = control_roughness(*rubber_band);
  flux::SplineTrajectoryOptimizationConfig rubber_config;
  rubber_config.minimum_bend_radius = Nanometres{0};
  rubber_config.control_step = Nanometres::from_mm(0.25);
  rubber_config.maximum_corridor_displacement = Nanometres::from_mm(2.0);
  rubber_config.maximum_iterations = 32U;
  rubber_config.maximum_candidate_checks = 4'096U;
  rubber_config.weights.elastic_anchor = 0.02L;
  rubber_config.weights.control_complexity = 1.0L;
  const auto rubber_optimized =
      flux::optimize_quintic_ph_trajectory(*rubber_band, rubber_config, {});
  const auto final_roughness = control_roughness(*rubber_band);
  require(rubber_optimized.accepted && rubber_optimized.accepted_control_moves > 0U &&
              final_roughness < initial_roughness * 0.65L,
          "whole-chain PH optimization must pull a sampled zigzag taut instead of only rounding "
          "its corners: roughness=" +
              std::to_string(static_cast<double>(initial_roughness)) + "->" +
              std::to_string(static_cast<double>(final_roughness)) +
              ", moves=" + std::to_string(rubber_optimized.accepted_control_moves));
  const auto rubber_samples =
      flux::adaptive_tessellate_quintic_ph(*rubber_band, Nanometres{20'000}, 16U, 4'096U);
  const auto rubber_pieces =
      flux::fit_spline_samples_to_lines_and_arcs(rubber_samples, Nanometres{20'000}, 64U, 4'096U);
  for (std::size_t piece = 1U; piece < rubber_pieces.size(); ++piece) {
    if (rubber_pieces[piece - 1U].kind != flux::SplineFittedPieceKind::segment ||
        rubber_pieces[piece].kind != flux::SplineFittedPieceKind::segment)
      continue;
    const auto& before = rubber_pieces[piece - 1U];
    const auto& after = rubber_pieces[piece];
    const auto first_x = before.end.x.value() - before.start.x.value();
    const auto first_y = before.end.y.value() - before.start.y.value();
    const auto second_x = after.end.x.value() - after.start.x.value();
    const auto second_y = after.end.y.value() - after.start.y.value();
    require(static_cast<long double>(first_x) * second_y ==
                static_cast<long double>(first_y) * second_x,
            "PH line/arc fitting must not recreate a polygonal bend as consecutive non-collinear "
            "straight chords");
  }
  std::vector<flux::SplineSample> long_flat_samples;
  for (std::size_t sample = 0U; sample < 130U; ++sample) {
    const auto parameter = static_cast<long double>(sample) / 129.0L;
    long_flat_samples.push_back(
        {parameter,
         flux::SplineVector{parameter * static_cast<long double>(Nanometres::from_mm(20.0).value()),
                            10'000.0L * std::sin(std::numbers::pi_v<long double> * parameter)}});
  }
  const auto long_flat_pieces =
      flux::fit_spline_samples_to_lines_and_arcs(long_flat_samples, Nanometres{20'000}, 8U, 256U);
  require(long_flat_pieces.size() == 1U &&
              long_flat_pieces.front().kind == flux::SplineFittedPieceKind::segment,
          "the arc sample bound must not fragment one complete tolerance-flat PH span into "
          "differently directed straight chords");

  auto gpu_rubber_band = flux::make_quintic_ph_route(
      {mm(0.0, 0.0), mm(5.0, 1.0), mm(10.0, -1.0), mm(15.0, 1.0), mm(20.0, 0.0)});
  auto gpu_rubber_config = rubber_config;
  gpu_rubber_config.prefer_gpu_preconditioner = true;
  gpu_rubber_config.gpu_precondition_iterations = 32U;
  gpu_rubber_config.prefer_gpu_polishing = true;
  gpu_rubber_config.gpu_polish_shortlist = 4U;
  gpu_rubber_config.exact_candidate_workers = 4U;
  const auto gpu_rubber_optimized =
      flux::optimize_quintic_ph_trajectory(*gpu_rubber_band, gpu_rubber_config, {});
  require(gpu_rubber_optimized.gpu_precondition_attempted &&
              (!flux::routing_compute_gpu_available() ||
               gpu_rubber_optimized.gpu_precondition_used_gpu) &&
              gpu_rubber_optimized.gpu_precondition_accepted &&
              gpu_rubber_optimized.gpu_precondition_legal &&
              (!flux::routing_compute_gpu_available() ||
               (gpu_rubber_optimized.gpu_polish_dispatches > 0U &&
                gpu_rubber_optimized.gpu_polish_exact_shortlist_checks > 0U &&
                gpu_rubber_optimized.gpu_polish_exact_shortlist_checks <
                    gpu_rubber_optimized.gpu_polish_candidates)) &&
              gpu_rubber_optimized.parallel_exact_waves > 0U &&
              gpu_rubber_optimized.parallel_exact_checks >=
                  gpu_rubber_optimized.parallel_exact_waves * 2U &&
              gpu_rubber_optimized.accepted &&
              gpu_rubber_band->control_points.front() == SplineVector{0.0L, 0.0L} &&
              gpu_rubber_band->control_points.back() == SplineVector{mm_value(20.0L), 0.0L},
          "whole-chain PH preconditioning must use the resident Vulkan XPBD path when available, "
          "preserve fixed endpoints, and remain subject to CPU trajectory acceptance");
}

void constrained_ph_rebound_avoids_inter_sample_obstacle() {
  auto spline = flux::make_quintic_ph_route(
      std::vector<Point>{mm(10.0, 10.0), mm(20.0, 10.0), mm(30.0, 10.0)});
  require(spline.has_value(), "continuous-envelope fixture must form a quintic PH curve");
  Polygon corridor;
  corridor.vertices = {mm(9.0, 8.0), mm(31.0, 8.0), mm(31.0, 12.0), mm(9.0, 12.0)};
  require(flux::set_quintic_ph_corridor(*spline, {corridor}),
          "the spline must retain its selected convex free-space corridor");
  const auto clearance = [](flux::SplineVector point) {
    const auto dx = point.x - static_cast<long double>(Nanometres::from_mm(20.0).value());
    const auto dy = point.y - static_cast<long double>(Nanometres::from_mm(10.25).value());
    const auto distance = std::hypotl(dx, dy);
    const auto radius = static_cast<long double>(Nanometres::from_mm(0.45).value());
    return flux::SplineClearanceProbe{distance - radius,
                                      distance > 0.0L
                                          ? flux::SplineVector{dx / distance, dy / distance}
                                          : flux::SplineVector{0.0L, -1.0L},
                                      distance >= radius};
  };
  const auto initial =
      flux::certify_quintic_ph_continuous_envelope(*spline, Nanometres{0}, clearance);
  flux::SplineTrajectoryOptimizationConfig config;
  config.minimum_bend_radius = Nanometres{0};
  config.control_step = Nanometres::from_mm(0.25);
  config.maximum_corridor_displacement = Nanometres::from_mm(1.5);
  config.clearance_influence_distance = Nanometres::from_mm(0.4);
  config.samples = 8U;
  config.maximum_iterations = 16U;
  config.maximum_candidate_checks = 192U;
  const auto optimized = flux::optimize_quintic_ph_trajectory(*spline, config, clearance);
  const auto final =
      flux::certify_quintic_ph_continuous_envelope(*spline, Nanometres{0}, clearance);
  flux::PreparedQuinticPHEvaluator rebound_samples(*spline, config.samples);
  require(rebound_samples.update(*spline), "optimized rebound must remain evaluable");
  std::size_t sampled_clearance_failures = 0U;
  long double minimum_sample_speed = std::numeric_limits<long double>::infinity();
  for (const auto& sample : rebound_samples.samples()) {
    sampled_clearance_failures += clearance(sample.position).represented_legal ? 0U : 1U;
    minimum_sample_speed =
        std::min(minimum_sample_speed, std::hypotl(sample.tangent.x, sample.tangent.y));
  }
  require(!initial.legal && optimized.accepted && final.legal &&
              flux::certify_quintic_ph_shape(*spline).legal,
          "continuous collision attribution must produce a smooth corridor-preserving rebound: " +
              std::to_string(initial.violations()) + "/" + std::to_string(optimized.accepted) +
              "/" + std::to_string(optimized.initial_violations) + "->" +
              std::to_string(optimized.final_violations) +
              "(shape=" + std::to_string(optimized.final_shape_violations) +
              ",continuous=" + std::to_string(optimized.final_continuous_violations) + ")/" +
              std::to_string(final.violations()) +
              "/moves=" + std::to_string(optimized.accepted_control_moves) +
              "/sample-clearance=" + std::to_string(sampled_clearance_failures) +
              "/min-speed=" + std::to_string(static_cast<double>(minimum_sample_speed)));

  flux::RegionalViaChoice via;
  via.net = NetId{"via-net"};
  via.site = mm(20.0, 10.5);
  via.span = {LayerId{"F.Cu"}, LayerId{"B.Cu"}};
  via.diameter = Nanometres::from_mm(0.6);
  const auto trace_width = Nanometres::from_mm(0.25);
  const auto trace_clearance = Nanometres::from_mm(0.2);
  const auto via_clearance = [&](flux::SplineVector point) {
    return flux::regional_candidate_via_clearance(NetId{"trace-net"}, LayerId{"B.Cu"},
        trace_width, trace_clearance, point, {via}, trace_clearance);
  };
  const auto probe = via_clearance({20'000'000.0L, 10'000'000.0L});
  require(std::abs(probe.clearance_nm + 125'000.0L) < 0.5L && probe.obstacle_normal.y < 0,
          "via guidance must use copper radius plus half trace width and maximum clearance");
  const std::vector<flux::RoutingComputePoint> via_queries(64U,
      {20'000'000, 10'000'000, 325'000, 1U, 0U});
  const std::vector<flux::RoutingComputeSegment> via_disks(64U,
      {20'000'000, 10'500'000, 20'000'000, 10'500'000, 500'000, 1U, 1U});
  for (const bool prefer_gpu : {false, true}) {
    const auto broad = flux::probe_routing_proximity(via_queries, via_disks, 1'024, prefer_gpu);
    require(broad.ok() && broad.queries.size() == via_queries.size() &&
                std::all_of(broad.queries.begin(), broad.queries.end(),
                    [](const auto& item) { return item.possible_conflict; }),
            "CPU/GPU proximity must preserve candidate-via disks as collision candidates");
  }
  require(std::isinf(flux::regional_candidate_via_clearance(via.net, LayerId{"B.Cu"},
              trace_width, trace_clearance, {20'000'000.0L, 10'000'000.0L}, {via}, trace_clearance).clearance_nm) &&
          std::isinf(flux::regional_candidate_via_clearance(NetId{"trace-net"}, LayerId{"In1.Cu"},
              trace_width, trace_clearance, {20'000'000.0L, 10'000'000.0L}, {via}, trace_clearance).clearance_nm),
          "same-net copper and layers outside the via span must not create guidance pressure");
  auto via_spline = flux::make_quintic_ph_route(std::vector<Point>{mm(10.0, 10.0), mm(30.0, 10.0)});
  require(via_spline && flux::set_quintic_ph_corridor(*via_spline, {corridor}),
          "via-guided rebound must preserve the original selected corridor");
  require(!flux::certify_quintic_ph_continuous_envelope(*via_spline, Nanometres{0}, via_clearance).legal,
          "the unsmoothed trace must expose the inter-sample via envelope collision");
  auto bounded_via_spline = *via_spline;
  flux::SplineTrajectoryOptimizationConfig regional_budget;
  regional_budget.control_step = Nanometres::from_mm(0.2);
  regional_budget.gradient_probe_step = Nanometres::from_mm(0.04);
  regional_budget.maximum_corridor_displacement = Nanometres::from_mm(2.0);
  regional_budget.samples = 32U;
  regional_budget.maximum_iterations = 2U;
  regional_budget.maximum_candidate_checks = 64U;
  const auto bounded_via = flux::optimize_quintic_ph_trajectory(
      bounded_via_spline, regional_budget, via_clearance);
  require(bounded_via.accepted && bounded_via.legal && !bounded_via.budget_exhausted &&
              flux::certify_quintic_ph_continuous_envelope(
                  bounded_via_spline, Nanometres{0}, via_clearance).legal,
          "the ordinary regional two-iteration/64-check budget must solve the isolated via rebound");
  const auto via_optimized = flux::optimize_quintic_ph_trajectory(*via_spline, config, via_clearance);
  require(via_optimized.accepted && flux::certify_quintic_ph_shape(*via_spline).legal &&
              flux::certify_quintic_ph_continuous_envelope(*via_spline, Nanometres{0}, via_clearance).legal,
          "existing trajectory optimisation must bend around a full via envelope without changing topology");
  Board native_via_board;
  native_via_board.outline = rectangle(0.0, 0.0, 40.0, 20.0);
  native_via_board.layers = {{LayerId{"F.Cu"}, "F.Cu", LayerKind::copper, 0},
                             {LayerId{"B.Cu"}, "B.Cu", LayerKind::copper, 1}};
  native_via_board.nets = {{NetId{"trace-net"}, "TRACE", 0}, {via.net, "VIA", 0}};
  native_via_board.vias.push_back({ObjectId{"obstacle-via"}, via.net, via.site,
      via.diameter, Nanometres::from_mm(0.3), via.span, false});
  const auto via_native = flux::fit_quintic_ph_to_native_copper(*via_spline, {});
  require(!via_native.empty(), "via rebound must have a native copper realization");
  for (std::size_t index = 0; index < via_native.size(); ++index) {
    const auto& piece = via_native[index];
    const ObjectId id{"via-rebound-" + std::to_string(index)};
    if (piece.kind == flux::SplineFittedPieceKind::circular_arc)
      native_via_board.track_arcs.push_back({id, NetId{"trace-net"}, LayerId{"B.Cu"},
          piece.start, piece.middle, piece.end, trace_width, false});
    else
      native_via_board.tracks.push_back({id, NetId{"trace-net"}, LayerId{"B.Cu"},
          piece.start, piece.end, trace_width, false});
  }
  require(flux::check_copper_rules(native_via_board).violations.empty(),
          "via-guided rebound must remain legal after fitting actual copper arcs and segments");
}

void curved_ph_materialization_is_exact_and_drc_clean() {
  auto curved_source = two_pad_board();
  curved_source.nets.push_back(Net{NetId{"arc-blocker-net"}, "ARC_BLOCKER", 0});
  curved_source.tracks.push_back(
      flux::TrackSegment{ObjectId{"arc-radius-blocker"}, NetId{"arc-blocker-net"}, LayerId{"F.Cu"},
                         mm(49.3, 40.39), mm(49.3, 40.7), Nanometres::from_mm(0.1), true});
  flux::GlobalRouteResult curved_global;
  curved_global.stop_reason = GlobalRouterStopReason::complete;
  flux::GlobalRoutePath curved_path;
  curved_path.net = NetId{"net-1"};
  curved_path.points = {{mm(20.0, 40.0), LayerId{"F.Cu"}},
                        {mm(50.0, 40.0), LayerId{"F.Cu"}},
                        {mm(50.0, 60.0), LayerId{"F.Cu"}},
                        {mm(80.0, 60.0), LayerId{"F.Cu"}},
                        {mm(80.0, 40.0), LayerId{"F.Cu"}}};
  curved_global.paths.push_back(std::move(curved_path));
  const auto curved_seed = flux::make_quintic_ph_route(std::vector<Point>{
      mm(20.0, 40.0), mm(50.0, 40.0), mm(50.0, 60.0), mm(80.0, 60.0), mm(80.0, 40.0)});
  require(curved_seed.has_value(), "curved materialization fixture must form a spline seed");
  const auto curved_seed_shape = flux::certify_quintic_ph_shape(*curved_seed);
  flux::DetailedRouterConfig curved_config;
  curved_config.pull_tight_45_degrees = false;
  curved_config.continuous_refinement = false;
  curved_config.circular_arc_refinement = true;
  curved_config.spline_minimum_bend_radius = Nanometres{0};
  curved_config.maximum_circular_arc_radius = Nanometres::from_mm(1.0);
  const auto curved = flux::materialize_detailed_route(curved_source, curved_global, curved_config);
  require(curved.complete() && curved.candidate && curved.fitted_cubic_beziers > 0U &&
              !curved.candidate->track_beziers.empty() &&
              curved.attempted_quintic_ph_runs > 0U &&
              curved.spline_g2_promotion_attempts > 0U &&
              curved.spline_g2_promotions > 0U &&
              curved.spline_g2_promotions + curved.spline_g2_fallbacks ==
                  curved.spline_g2_promotion_attempts &&
              curved.adaptive_spline_tessellation_samples > curved.fitted_circular_arcs &&
              curved.unrounded_non_junction_bends == 0U && curved.copper_drc.ok(),
          "a bent route must be adaptively fitted from a quintic PH curve to connected DRC-clean "
          "native cubic copper: " +
              std::to_string(curved.adaptive_spline_tessellation_samples) + "/" +
              std::to_string(curved.fitted_cubic_beziers) +
              "/runs=" + std::to_string(curved.attempted_quintic_ph_runs) + "/" +
              std::to_string(curved.fitted_quintic_ph_runs) +
              "/g2=" + std::to_string(curved.spline_g2_promotions) + "/" +
              std::to_string(curved.spline_g2_promotion_attempts) +
              "/complete=" + std::to_string(curved.complete()) +
              "/nav=" + std::to_string(curved.spline_navmesh_routes) + "/" +
              std::to_string(curved.spline_navmesh_failures) +
              "/traj=" + std::to_string(curved.spline_trajectory_accepted) + "/" +
              std::to_string(curved.spline_trajectory_initial_violations) + "->" +
              std::to_string(curved.spline_trajectory_final_violations) + "/" +
              std::to_string(curved.spline_trajectory_initial_shape_violations) + "->" +
              std::to_string(curved.spline_trajectory_final_shape_violations) + "/continuous=" +
              std::to_string(curved.spline_trajectory_initial_continuous_violations) + "->" +
              std::to_string(curved.spline_trajectory_final_continuous_violations) +
              "/moves=" + std::to_string(curved.spline_trajectory_control_moves) +
              "/checks=" + std::to_string(curved.spline_trajectory_candidate_checks) + "/" +
              std::to_string(curved_seed_shape.self_intersections) + "/" +
              std::to_string(curved_seed_shape.cusps) + "/" +
              std::to_string(curved_seed_shape.station_reversals) + "/" +
              std::to_string(curved_seed_shape.excess_turn_violations) + "/" +
              curved.spline_navmesh_last_diagnostic + "/topology=" +
              curved.spline_navmesh_last_topology_diagnostic + "/" + curved.diagnostic);

  curved_config.defer_saved_zone_fill_clearance_to_refill = true;
  const auto refillable_curve =
      flux::materialize_detailed_route(curved_source, curved_global, curved_config);
  require(refillable_curve.provisional_zone_refill_candidate() && refillable_curve.candidate &&
              refillable_curve.fitted_cubic_beziers > 0U &&
              !refillable_curve.candidate->track_beziers.empty() &&
              !refillable_curve.fallback_polyline_materialization,
          "a legal curved refill candidate must remain provisional without being mistaken for a "
          "failed curved transaction");
}

void exact_copper_witnesses_preserve_branch_identity() {
  Board board;
  board.outline = rectangle(0, 0, 30, 30);
  board.layers = {{LayerId{"F.Cu"}, "F.Cu", LayerKind::copper, 0}};
  board.nets = {{NetId{"a"}, "A", 1}, {NetId{"b"}, "B", 2}};
  flux::GlobalRoutePath carrier;
  carrier.net = NetId{"a"};
  carrier.points = {{mm(10, 15), LayerId{"F.Cu"}}, {mm(20, 15), LayerId{"F.Cu"}}};
  auto tap = carrier;
  tap.connection_index = 1U;
  tap.points = {{mm(15, 20), LayerId{"F.Cu"}}, {mm(15, 15), LayerId{"F.Cu"}}};
  auto crossing = carrier;
  crossing.net = NetId{"b"};
  crossing.points = {{mm(10, 10), LayerId{"F.Cu"}}, {mm(20, 20), LayerId{"F.Cu"}}};
  flux::RegionalRouterConfig config;
  config.enable_spline_realization = false;
  config.enable_terminal_portal_templates = false;
  config.max_alternative_template_checks = 1U;
  config.max_joint_alternative_template_checks = 1U;
  config.max_local_layer_bypass_checks = 1U;
  config.max_same_layer_detour_checks = 0U;
  config.max_confined_anisotropic_erosion_checks = 0U;
  for (const bool split : {false, true}) {
    flux::GlobalRouteResult global;
    global.stop_reason = GlobalRouterStopReason::complete;
    global.paths = {carrier, tap, crossing};
    if (split) {
      global.paths.front().points.insert(global.paths.front().points.begin() + 1,
                                         {mm(15, 15), LayerId{"F.Cu"}});
      std::reverse(global.paths.begin(), global.paths.end());
    }
    flux::QuantizedCopperOracleConfig oracle_config;
    oracle_config.diagnosis_coalescing_pitch = Nanometres::from_mm(100);
    const auto oracle = flux::probe_quantized_copper_feasibility(board, global, oracle_config);
    require(!oracle.exact_copper_legal && oracle.diagnoses.size() == 2U,
            "junction must expose both carrier/crossing and tap/crossing diagnoses");
    const auto pair_count = std::accumulate(oracle.diagnoses.begin(), oracle.diagnoses.end(),
        std::size_t{0}, [](auto count, const auto& d) { return count + d.source_segment_pairs.size(); });
    require(pair_count == (split ? 3U : 2U),
            "spatial coalescing must retain distinct exact segment pairs");
    config.physical_conflict_witnesses = oracle.diagnoses;
    const auto observed = flux::analyze_regional_conflicts(board, global, config);
    require(observed.mapped_exact_copper_pairs == pair_count &&
                observed.unmapped_exact_copper_pairs == 0U && observed.conflict_edges.size() == pair_count &&
                observed.windows.size() == 1U &&
                observed.windows.front().owned_fragments.size() == (split ? 4U : 3U),
            "exact provenance must own both junction branches and retain an explicit shared vertex");
    for (auto& d : config.physical_conflict_witnesses)
      for (auto& pair : d.source_segment_pairs) pair.first.segment_index = 100U;
    const auto stale_index = flux::analyze_regional_conflicts(board, global, config);
    require(stale_index.conflict_edges.empty() && stale_index.unmapped_exact_copper_pairs == pair_count,
            "stale edge identity must not fall back to a nearby unrelated branch");
    config.physical_conflict_witnesses = oracle.diagnoses;
    for (auto& d : config.physical_conflict_witnesses)
      for (auto& pair : d.source_segment_pairs) pair.first.start = mm(1, 1);
    const auto stale_geometry = flux::analyze_regional_conflicts(board, global, config);
    require(stale_geometry.conflict_edges.empty() &&
                stale_geometry.unmapped_exact_copper_pairs == pair_count,
            "matching edge numbers cannot authorize provenance from changed source geometry");
    config.physical_conflict_witnesses = oracle.diagnoses;
    for (auto& d : config.physical_conflict_witnesses) d.resource.layer = LayerId{"B.Cu"};
    const auto wrong_layer = flux::analyze_regional_conflicts(board, global, config);
    require(wrong_layer.conflict_edges.empty() && wrong_layer.unmapped_exact_copper_pairs == pair_count,
            "typed planar provenance must not cross the reported conflict layer");
    config.physical_conflict_witnesses = oracle.diagnoses;
    auto changed = global;
    for (auto& path : changed.paths)
      if (path.net == carrier.net && path.connection_index == carrier.connection_index)
        path.points.front().position = mm(9, 15);
    const auto stale_subsegment = flux::analyze_regional_conflicts(board, changed, config);
    require(stale_subsegment.unmapped_exact_copper_pairs == 1U &&
                stale_subsegment.mapped_exact_copper_pairs == pair_count - 1U,
            "old endpoints inside an enlarged collinear fragment must still match the current source edge");
  }
}

void exact_copper_witnesses_create_missing_regional_scope() {
  Board board;
  board.source_revision = "exact-witness-scope";
  board.outline = rectangle(0.0, 0.0, 30.0, 30.0);
  board.layers = {{LayerId{"F.Cu"}, "F.Cu", LayerKind::copper, 0}};
  board.nets = {{NetId{"a"}, "A", 1}, {NetId{"b"}, "B", 2}};
  flux::GlobalRouteResult global;
  global.stop_reason = GlobalRouterStopReason::complete;
  flux::GlobalRoutePath a;
  a.net = NetId{"a"};
  a.points = {{mm(10, 10), LayerId{"F.Cu"}}, {mm(20, 20), LayerId{"F.Cu"}}};
  auto b = a;
  b.net = NetId{"b"};
  b.points = {{mm(10, 20), LayerId{"F.Cu"}}, {mm(20, 10), LayerId{"F.Cu"}}};
  global.paths = {a, b};
  flux::RegionalRouterConfig config;
  config.enable_spline_realization = false;
  config.enable_terminal_portal_templates = false;
  config.max_alternative_template_checks = 1U;
  config.max_joint_alternative_template_checks = 1U;
  config.max_local_layer_bypass_checks = 1U;
  config.max_same_layer_detour_checks = 0U;
  config.max_confined_anisotropic_erosion_checks = 0U;
  const auto unobserved = flux::analyze_regional_conflicts(board, global, config);
  require(unobserved.conflict_edges.empty() && unobserved.windows.empty(),
          "the synthetic global checkpoint deliberately has no sampled conflict witnesses");
  flux::RoutabilityDiagnosis witness;
  witness.kind = flux::RoutabilityFailureKind::packing_capacity_exceeded;
  witness.stage = flux::RoutabilityFailureStage::final_validation;
  witness.resource = {"exact-crossing", LayerId{"F.Cu"}, {mm(14, 14), mm(16, 16)},
                      {NetId{"b"}, NetId{"a"}, NetId{"a"}}};
  witness.clearance_deficit = Nanometres::from_mm(0.4);
  config.physical_conflict_witnesses = {witness};
  const auto observed = flux::analyze_regional_conflicts(board, global, config);
  require(observed.source_exact_copper_witnesses == 1U &&
              observed.mapped_exact_copper_pairs == 1U &&
              observed.unmapped_exact_copper_pairs == 0U &&
              observed.conflict_edges.size() == 1U && !observed.windows.empty() &&
              observed.conflict_edges.front().exact_copper_sample_count == 1U &&
              observed.conflict_edges.front().capacity_sample_count == 0U &&
              observed.conflict_edges.front().peak_clearance_deficit == witness.clearance_deficit,
          "exact DRC net pairs and bounds must create a regional repair window without inventing "
          "sampled capacity evidence: " + std::to_string(observed.source_exact_copper_witnesses) +
              "/" + std::to_string(observed.mapped_exact_copper_pairs) + "/" +
              std::to_string(observed.unmapped_exact_copper_pairs) + "/edges=" +
              std::to_string(observed.conflict_edges.size()) + "/windows=" +
              std::to_string(observed.windows.size()) + "; " + observed.diagnostic);
  config.physical_conflict_witnesses.front().resource.layer = LayerId{"B.Cu"};
  {
    auto coupled_board = board;
    coupled_board.nets.push_back({NetId{"c"}, "c"});
    auto coupled_global = global;
    auto c = a;
    c.net = NetId{"c"};
    c.points = {{mm(10, 18), LayerId{"F.Cu"}}, {mm(20, 18), LayerId{"F.Cu"}}};
    coupled_global.paths.push_back(c);
    auto coupled_config = config;
    auto second = witness;
    second.resource = {"second-exact-crossing", LayerId{"F.Cu"}, {mm(11, 17), mm(13, 19)},
                       {NetId{"b"}, NetId{"c"}}};
    coupled_config.physical_conflict_witnesses = {witness, second};
    const auto coupled = flux::analyze_regional_conflicts(coupled_board, coupled_global, coupled_config);
    require(coupled.mapped_exact_copper_pairs == 2 && coupled.windows.size() == 1 &&
                coupled.windows.front().owned_fragments.size() == 3,
            "a physical three-net component must not split into separately owned articulation windows");
    // One oversized interaction plus an independent eligible crossing. The required global
    // replan must not wait for native search in the unrelated window, nor erase its fragments.
    coupled_board.nets.push_back({NetId{"d"}, "d"});
    coupled_board.nets.push_back({NetId{"e"}, "e"});
    auto d = a;
    d.net = NetId{"d"};
    d.points = {{mm(22, 22), LayerId{"F.Cu"}}, {mm(28, 28), LayerId{"F.Cu"}}};
    auto e = d;
    e.net = NetId{"e"};
    e.points = {{mm(22, 28), LayerId{"F.Cu"}}, {mm(28, 22), LayerId{"F.Cu"}}};
    coupled_global.paths.push_back(d);
    coupled_global.paths.push_back(e);
    auto independent = witness;
    independent.resource = {"independent-crossing", LayerId{"F.Cu"}, {mm(24, 24), mm(26, 26)},
                            {NetId{"d"}, NetId{"e"}}};
    coupled_config.physical_conflict_witnesses.push_back(independent);
    coupled_config.max_nets_per_window = 2U;
    coupled_config.prioritize_frozen_scope_expansion = true;
    const auto inspection = flux::analyze_regional_conflicts(
        coupled_board, coupled_global, coupled_config);
    require(inspection.windows.size() == 2U && !inspection.channels.empty(),
            "decomposition-only inspection must still build the independent eligible corridor");
    coupled_config.enable_spline_realization = true;
    const auto escalated = flux::analyze_regional_conflicts(
        coupled_board, coupled_global, coupled_config);
    require(!escalated.complete() && !escalated.spline_candidate_available &&
                escalated.windows.size() == 2U && escalated.fragments.size() == 5U &&
                escalated.channels.empty() && escalated.alternative_template_checks == 0U &&
                escalated.joint_spline_candidates_evaluated == 0U &&
                escalated.bundle_assignments.size() == 1U &&
                escalated.bundle_assignments.front().stop_reason ==
                    flux::RegionalBundleStopReason::window_ineligible,
            "an oversized required scope must fail closed before unrelated portfolio/native work");
    const auto oversized_diagnoses = flux::diagnose_regional_routability(escalated);
    require(std::any_of(oversized_diagnoses.begin(), oversized_diagnoses.end(), [](const auto& item) {
              return item.suggested_relaxation == flux::RoutabilityRelaxation::replan_global_topology &&
                     item.resource.nets.size() == 3U;
            }), "scope-size escalation must retain the exact affected net set for global feedback");
  }
  const auto wrong_layer = flux::analyze_regional_conflicts(board, global, config);
  require(!wrong_layer.complete() && wrong_layer.conflict_edges.empty() && wrong_layer.unmapped_exact_copper_pairs == 1U,
          "an exact planar witness cannot be attributed to a different layer");
  auto identified_board = board;
  identified_board.layers.push_back({LayerId{"B.Cu"}, "B.Cu", LayerKind::copper, 1});
  auto identified_global = global;
  for (auto& point : identified_global.paths.back().points) point.layer = LayerId{"B.Cu"};
  auto identified_config = config;
  identified_config.physical_conflict_witnesses.clear();
  identified_config.frozen_copper_context = true;
  identified_config.expand_frozen_copper_scope = true;
  identified_config.frozen_copper_scope_witnesses = {
      {{NetId{"a"}, 0U, 0U, 0U}, {NetId{"b"}, 0U, 0U, 0U}, LayerId{"B.Cu"},
       Nanometres::from_mm(0.4), "candidate-layer-crossing"}};
  const auto identified = flux::analyze_regional_conflicts(
      identified_board, identified_global, identified_config);
  require(identified.mapped_exact_copper_pairs == 1U &&
              identified.unmapped_exact_copper_pairs == 0U &&
              identified.windows.size() == 1U && identified.windows.front().owned_fragments.size() == 2U,
          "candidate-layer conflicts with exact original IDs must create joint ownership across layers");
  config.physical_conflict_witnesses.front() = witness;
  config.physical_conflict_witnesses.front().resource.nets = {NetId{"a"}, NetId{"missing"}};
  const auto missing = flux::analyze_regional_conflicts(board, global, config);
  require(missing.conflict_edges.empty() && missing.unmapped_exact_copper_pairs == 1U,
          "unknown exact witness owners must be explicitly unmapped");
}

void parallel_spline_workers_shutdown_after_short_optimizations() {
  flux::SplineTrajectoryOptimizationConfig config;
  config.exact_candidate_workers = 4;
  config.maximum_iterations = 1;
  config.maximum_candidate_checks = 8;
  config.maximum_multistart_attempts = 0;
  config.maximum_adaptive_control_insertions = 0;
  config.samples = 8;
  for (std::size_t repetition = 0; repetition < 512; ++repetition) {
    const std::vector<Point> points = repetition % 2 == 0
        ? std::vector<Point>{mm(0, 0), mm(10, 0)}
        : std::vector<Point>{mm(0, 0), mm(5, 1), mm(10, 0)};
    auto route = flux::make_quintic_ph_route(points);
    require(route.has_value(), "worker lifecycle fixture must construct a valid route");
    const auto result = flux::optimize_quintic_ph_trajectory(*route, config, {});
    require(result.legal, "short parallel optimization must retain legal geometry");
  }
}

void exact_via_witnesses_create_regional_repair_scope() {
  Board board;
  board.outline = rectangle(0, 0, 40, 40);
  board.layers = {{LayerId{"F.Cu"}, "F.Cu", LayerKind::copper, 0},
                  {LayerId{"B.Cu"}, "B.Cu", LayerKind::copper, 1}};
  board.nets = {{NetId{"approach-a"}, "a"}, {NetId{"approach-b"}, "b"}};
  flux::GlobalRouteResult global;
  global.paths = {
      flux::GlobalRoutePath{NetId{"approach-a"}, 0U,
          {{mm(20, 19.35), LayerId{"B.Cu"}}, {mm(20, 20), LayerId{"B.Cu"}},
           {mm(20, 20), LayerId{"F.Cu"}}, {mm(20, 21.335), LayerId{"F.Cu"}}}},
      flux::GlobalRoutePath{NetId{"approach-b"}, 0U,
          {{mm(20.65, 16.385), LayerId{"F.Cu"}}, {mm(20.65, 20), LayerId{"F.Cu"}},
           {mm(20.65, 20), LayerId{"B.Cu"}}, {mm(20, 20.65), LayerId{"B.Cu"}}}}};
  flux::RegionalRouterConfig config;
  config.enable_spline_realization = false;
  config.max_bundle_search_states = 8;
  config.max_alternative_template_checks = 1;
  config.max_joint_alternative_template_checks = 1;
  config.max_same_layer_detour_checks = 0;
  config.max_physical_scope_retries = 0;
  config.physical_conflict_witnesses = flux::probe_quantized_copper_feasibility(board, global).diagnoses;
  const auto regional = flux::analyze_regional_conflicts(board, global, config);
  require(config.physical_conflict_witnesses.size() == 2U &&
              std::all_of(config.physical_conflict_witnesses.begin(),
                          config.physical_conflict_witnesses.end(), [](const auto& witness) {
                return witness.kind == flux::RoutabilityFailureKind::via_site_exhausted &&
                       witness.suggested_relaxation ==
                           flux::RoutabilityRelaxation::change_via_site &&
                       witness.source_segment_pairs.size() == 1U;
              }), "the microcase must isolate exact via failures without track-track conflicts");
  require(regional.source_exact_copper_witnesses == 2U &&
              regional.mapped_exact_copper_pairs == 2U &&
              regional.unmapped_exact_copper_pairs == 0U && !regional.windows.empty() &&
              !regional.complete(),
          "exact via failures must create scope or remain explicitly unresolved, never disappear");
  auto reversed_global = global;
  std::reverse(reversed_global.paths.begin(), reversed_global.paths.end());
  require(flux::probe_quantized_copper_feasibility(board, reversed_global).diagnoses ==
              config.physical_conflict_witnesses,
          "via repair ownership and common layer must not depend on path enumeration order");
  auto unavailable_layer = config;
  for (auto& witness : unavailable_layer.physical_conflict_witnesses)
    witness.resource.layer = LayerId{"In1.Cu"};
  const auto unmapped = flux::analyze_regional_conflicts(board, global, unavailable_layer);
  require(unmapped.source_exact_copper_witnesses == 2U &&
              unmapped.unmapped_exact_copper_pairs == 2U && !unmapped.complete(),
          "a via witness without a matching routing layer must remain explicitly unresolved");
}

void regional_bundle_forward_checking_preserves_access_solutions() {
  Board board;
  board.outline = rectangle(0, 0, 100, 100);
  board.layers = {{LayerId{"F.Cu"}, "F.Cu", LayerKind::copper, 0},
                  {LayerId{"B.Cu"}, "B.Cu", LayerKind::copper, 1}};
  flux::RegionalRouteResult regional;
  // This fixture compares the historical DFS propagation controls. HC composition is tested
  // separately and must not change what the explicit forward-checking ablation measures.
  regional.configured.enable_bounded_hc_composition = false;
  flux::RegionalRoutingWindow window;
  std::vector<flux::RegionalRouteTemplate> templates;
  std::vector<std::vector<std::size_t>> domains;
  flux::RegionalChannel channel;
  flux::RegionalCrossSection section;
  section.usable_width = Nanometres::from_mm(100);
  channel.cross_sections.push_back(section);
  regional.channels.push_back(channel);
  // Eleven unconstrained binary choices separate an early bad escape from its last blocker.
  // The original bounded DFS spends its whole budget below the bad escape. Forward checking
  // must reach the good escape while retaining every feasible combination of unrelated routes.
  for (std::size_t slot = 0; slot < 12; ++slot) {
    const NetId net{"escape-" + std::to_string(slot)};
    board.nets.push_back({net, net.value()});
    flux::RegionalRouteFragment fragment;
    fragment.id = {net, 0, 0, 0};
    fragment.layer = LayerId{"F.Cu"};
    fragment.width = Nanometres::from_mm(0.25);
    fragment.clearance = Nanometres::from_mm(0.2);
    regional.fragments.push_back(fragment);
    window.fragments.push_back(fragment.id);
    window.owned_fragments.push_back(fragment.id);
    window.nets.push_back(net);
    std::vector<std::size_t> domain;
    for (std::size_t option = 0; option < 2; ++option) {
      flux::RegionalRouteTemplate route;
      route.id = templates.size();
      route.fragment = fragment.id;
      route.routing_layer = fragment.layer;
      route.channel_sequence = {0};
      if (slot == 0 || slot == 11) {
        flux::RegionalTerminalChoice terminal;
        terminal.id = regional.terminal_choices.size();
        terminal.net = net;
        terminal.layer = fragment.layer;
        terminal.width = fragment.width;
        terminal.access_path = slot == 11
                                   ? std::vector<Point>{mm(15, 5), mm(15, 15)}
                                   : std::vector<Point>{mm(10, option ? 30 : 10),
                                                        mm(20, option ? 30 : 10)};
        route.terminal_entry_choice = terminal.id;
        regional.terminal_choices.push_back(terminal);
      }
      domain.push_back(route.id);
      templates.push_back(route);
    }
    domains.push_back(domain);
  }
  std::sort(window.owned_fragments.begin(), window.owned_fragments.end());
  const auto control = flux::enumerate_regional_bundle_choices(
      board, regional, window, templates, domains, 128, false);
  require(control.exhausted && !control.choices.empty() &&
              std::all_of(control.choices.begin(), control.choices.end(),
                          [](const auto& choice) { return choice.front() == 0; }),
          "the bounded control must remain trapped under the conflicting terminal escape");
  const auto checked = flux::enumerate_regional_bundle_choices(
      board, regional, window, templates, domains, 128);
  require(!checked.choices.empty() && checked.forward_domain_wipeouts > 0 &&
              checked.terminal_pair_pruned_states > 0 &&
              std::all_of(checked.choices.begin(), checked.choices.end(),
                          [](const auto& choice) { return choice.front() == 1; }),
          "forward checking must reach the feasible escape within the identical state bound");
  const auto exhaustive = flux::enumerate_regional_bundle_choices(
      board, regional, window, templates, domains, 10000);
  require(!exhaustive.exhausted && exhaustive.choices.size() == 2048,
          "early access propagation must retain every combination below the legal escape");

  // Replace the terminal contradiction with a late channel-capacity contradiction. The
  // physical rule is unchanged; propagating it must skip a doomed prefix before enumeration.
  auto narrow = regional.channels.front();
  narrow.id = 1;
  narrow.cross_sections.front().usable_width = Nanometres::from_mm(0.65);
  regional.channels.push_back(narrow);
  for (auto& route : templates) route.terminal_entry_choice.reset();
  templates.front().channel_sequence = {1};
  templates[22].channel_sequence = {1};
  templates[23].channel_sequence = {1};
  regional.configured.enable_bundle_resource_forward_checking = false;
  const auto capacity_trapped = flux::enumerate_regional_bundle_choices(
      board, regional, window, templates, domains, 128);
  require(capacity_trapped.exhausted && capacity_trapped.choices.empty(),
          "access-only propagation must reproduce the late capacity ordering trap");
  regional.configured.enable_bundle_resource_forward_checking = true;
  const auto capacity_checked = flux::enumerate_regional_bundle_choices(
      board, regional, window, templates, domains, 128);
  require(!capacity_checked.choices.empty() && capacity_checked.forward_capacity_rejections > 0 &&
              capacity_checked.forward_domain_wipeouts > 0 &&
              std::all_of(capacity_checked.choices.begin(), capacity_checked.choices.end(),
                          [](const auto& choice) { return choice.front() == 1; }),
          "resource propagation must reach the capacity-feasible escape under the same bound");
  const auto capacity_exhaustive = flux::enumerate_regional_bundle_choices(
      board, regional, window, templates, domains, 10000);
  require(!capacity_exhaustive.exhausted && capacity_exhaustive.choices.size() == 2048,
          "resource propagation must preserve every capacity-feasible combination");
  regional.configured.enable_bundle_resource_forward_checking = false;
  for (auto& route : templates) route.channel_sequence = {0};

  // The existing DFS requires one nominated site per physical via region even when access
  // geometry can relocate it. Propagate that identity into distant future fragments too.
  auto& last_fragment = regional.fragments.back();
  last_fragment.id = regional.fragments.front().id;
  last_fragment.id.fragment_index = 1;
  window.fragments.back() = last_fragment.id;
  window.owned_fragments = window.fragments;
  std::sort(window.owned_fragments.begin(), window.owned_fragments.end());
  templates[22].fragment = last_fragment.id;
  templates[23].fragment = last_fragment.id;
  for (std::size_t i = 0; i < 2; ++i) {
    flux::RegionalViaChoice via;
    via.id = i;
    via.net = regional.fragments.front().id.net;
    via.site = mm(i ? 20 : 10, 10);
    via.span = {LayerId{"F.Cu"}, LayerId{"B.Cu"}};
    via.diameter = Nanometres::from_mm(0.6);
    via.drill = Nanometres::from_mm(0.3);
    via.selected_global_site = true;
    regional.via_choices.push_back(via);
    templates[i].via_choices = {i};
    templates[22 + i].via_choices = {1};
  }
  const auto identity_trapped = flux::enumerate_regional_bundle_choices(
      board, regional, window, templates, domains, 128);
  require(identity_trapped.exhausted && identity_trapped.choices.empty(),
          "access-only propagation must reproduce the nominated via-identity ordering trap");
  regional.configured.enable_bundle_resource_forward_checking = true;
  const auto identity_checked = flux::enumerate_regional_bundle_choices(
      board, regional, window, templates, domains, 128);
  require(!identity_checked.choices.empty() && identity_checked.forward_via_identity_rejections > 0 &&
              std::all_of(identity_checked.choices.begin(), identity_checked.choices.end(),
                          [](const auto& choice) { return choice.front() == 1; }),
          "resource propagation must reach the common nominated via site within the same bound");
  const auto identity_exhaustive = flux::enumerate_regional_bundle_choices(
      board, regional, window, templates, domains, 10000);
  require(!identity_exhaustive.exhausted && identity_exhaustive.choices.size() == 2048,
          "resource propagation must retain every identity-consistent combination");
  regional.configured.enable_bundle_resource_forward_checking = false;
  regional.via_choices.clear();

  // A pair of nominated vias may collide while the selected global topology can legally
  // relocate one. The partial query must use its full domain, not just the nominated sites.
  regional.fragments.resize(2);
  window.fragments.resize(2);
  window.owned_fragments = window.fragments;
  std::sort(window.owned_fragments.begin(), window.owned_fragments.end());
  templates.resize(2);
  for (std::size_t i = 0; i < 2; ++i) {
    templates[i].id = i;
    templates[i].fragment = regional.fragments[i].id;
    templates[i].terminal_entry_choice.reset();
    templates[i].via_choices = {i};
    flux::RegionalViaChoice via;
    via.id = i;
    via.region_id = i;
    via.net = regional.fragments[i].id.net;
    via.site = mm(10, 10);
    via.span = {LayerId{"F.Cu"}, LayerId{"B.Cu"}};
    via.diameter = Nanometres::from_mm(0.6);
    via.drill = Nanometres::from_mm(0.3);
    via.selected_global_site = true;
    regional.via_choices.push_back(via);
  }
  auto relocated = regional.via_choices.back();
  relocated.id = 2;
  relocated.site = mm(20, 20);
  relocated.selected_global_site = false;
  regional.via_choices.push_back(relocated);
  domains = {{0}, {1}};
  const auto movable = flux::enumerate_regional_bundle_choices(
      board, regional, window, templates, domains, 100);
  require(movable.choices.size() == 1 && movable.access_pruned_states == 0,
          "a colliding nominated via must keep its legal relocation domain");
  regional.configured.max_access_assignment_states = 1;
  const auto unknown = flux::enumerate_regional_bundle_choices(
      board, regional, window, templates, domains, 100);
  require(unknown.choices.size() == 1 && unknown.access_assignment_exhaustions > 0 &&
              unknown.access_pruned_states == 0 && unknown.access_selection_cache_entries_peak == 0U,
          "an exhausted access subproblem must remain unknown and cannot prune the topology");
  regional.configured.max_access_assignment_states = 2048;
  regional.via_choices.pop_back();
  const auto impossible = flux::enumerate_regional_bundle_choices(
      board, regional, window, templates, domains, 100);
  require(impossible.choices.empty() && impossible.via_pair_pruned_states > 0 &&
              !impossible.exhausted,
          "a proved collision between fixed via domains must be rejected before a complete leaf");

  // Two fragments nominate the same physical via but different runup representatives. The
  // earlier fragment supplies the legal representative in the complete selection. A pair
  // containing only the later fragment and a foreign via must not reject that completion.
  regional.via_choices[1].site = mm(20, 10);
  auto bad_runup = regional.via_choices.front();
  bad_runup.id = 2;
  bad_runup.attached_runups.push_back(
      {LayerId{"F.Cu"}, mm(25, 10), {mm(25, 10), mm(10, 10)}});
  regional.via_choices.push_back(bad_runup);
  auto third_fragment = regional.fragments.front();
  third_fragment.id.fragment_index = 1;
  regional.fragments.push_back(third_fragment);
  for (auto& f : regional.fragments) f.explicit_width = true;
  window.fragments.push_back(third_fragment.id);
  window.owned_fragments = window.fragments;
  std::sort(window.owned_fragments.begin(), window.owned_fragments.end());
  auto third_template = templates.front();
  third_template.id = 2;
  third_template.fragment = third_fragment.id;
  third_template.via_choices = {2};
  templates.push_back(third_template);
  domains = {{0}, {1}, {2}};
  const auto equivalent_runups = flux::enumerate_regional_bundle_choices(
      board, regional, window, templates, domains, 100);
  require(equivalent_runups.choices.size() == 1,
          "pair pruning must allow an earlier fragment's legal equivalent via representative");
  domains = {{1}, {2}};
  const auto bad_complete_runup = flux::enumerate_regional_bundle_choices(
      board, regional, window, templates, domains, 100);
  require(bad_complete_runup.choices.empty() && bad_complete_runup.via_pair_pruned_states > 0,
          "complete-leaf access checks must still certify the actual chosen runup representative");
}

void frozen_foreign_copper_rejects_selected_vias_before_spline_selection() {
  Board board;
  board.outline = rectangle(0, 0, 40, 40);
  board.layers = {{LayerId{"F.Cu"}, "F.Cu", LayerKind::copper, 0},
                  {LayerId{"B.Cu"}, "B.Cu", LayerKind::copper, 1},
                  {LayerId{"In1.Cu"}, "In1.Cu", LayerKind::copper, 2}};
  const NetId candidate_net{"via-owner"};
  const NetId frozen_net{"frozen-lane"};
  board.nets = {{candidate_net, "VIA_OWNER"}, {frozen_net, "FROZEN_LANE"}};

  flux::RegionalRouteResult regional;
  regional.configured.frozen_copper_context = true;
  flux::RegionalRouteFragment candidate;
  candidate.id = {candidate_net, 0U, 0U, 0U};
  candidate.layer = LayerId{"F.Cu"};
  candidate.start = mm(5.0, 10.0);
  candidate.end = mm(20.0, 10.0);
  candidate.width = Nanometres::from_mm(0.25);
  candidate.clearance = Nanometres::from_mm(0.2);
  flux::RegionalRouteFragment frozen;
  frozen.id = {frozen_net, 0U, 0U, 0U};
  frozen.layer = LayerId{"F.Cu"};
  frozen.start = mm(10.0, 5.0);
  frozen.end = mm(10.0, 15.0);
  frozen.width = Nanometres::from_mm(0.25);
  frozen.clearance = Nanometres::from_mm(0.2);
  regional.fragments = {candidate, frozen};

  flux::RegionalRoutingWindow window;
  window.id = 9U;
  window.fragments = {candidate.id};
  window.owned_fragments = window.fragments;

  flux::RegionalViaChoice selected;
  selected.id = 0U;
  selected.region_id = 0U;
  selected.net = candidate_net;
  selected.site = mm(10.15, 10.0);
  selected.span = {LayerId{"F.Cu"}, LayerId{"B.Cu"}};
  selected.diameter = Nanometres::from_mm(0.7);
  selected.drill = Nanometres::from_mm(0.3);
  selected.selected_global_site = true;
  auto alternative = selected;
  alternative.id = 1U;
  alternative.site = mm(20.0, 10.0);
  alternative.selected_global_site = false;
  regional.via_choices = {selected, alternative};

  flux::RegionalRouteTemplate route;
  route.id = 0U;
  route.fragment = candidate.id;
  route.routing_layer = LayerId{"F.Cu"};
  route.selected_global_topology = true;
  route.via_choices = {selected.id};
  const std::vector<flux::RegionalRouteTemplate> templates{route};
  const std::vector<std::vector<std::size_t>> domains{{0U}};
  const auto select = [&] {
    return flux::enumerate_regional_bundle_choices(board, regional, window, templates, domains,
                                                    16U);
  };

  const auto alternative_selected = select();
  require(alternative_selected.choices.size() == 1U &&
              alternative_selected.access_pruned_states == 0U,
          "a frozen foreign lane must reject only the colliding selected via and preserve a legal "
          "same-region alternative before spline realization");

  regional.via_choices.pop_back();
  const auto fixed_site_rejected = select();
  require(fixed_site_rejected.choices.empty() &&
              fixed_site_rejected.frozen_copper_via_pruned_states > 0U &&
              !fixed_site_rejected.exhausted,
          "a fixed selected via must not survive access selection through immutable foreign "
          "copper");
  require(fixed_site_rejected.frozen_copper_conflicts.size() == 1U &&
              fixed_site_rejected.frozen_copper_conflicts.front().candidate == candidate.id &&
              fixed_site_rejected.frozen_copper_conflicts.front().blocker == frozen.id &&
              fixed_site_rejected.frozen_copper_conflicts.front().layer == LayerId{"F.Cu"} &&
              fixed_site_rejected.frozen_copper_conflicts.front().clearance_deficit ==
                  Nanometres::from_mm(0.525) &&
              fixed_site_rejected.frozen_copper_conflicts.front().code ==
                  "frozen-via-copper-clearance",
          "a frozen-via rejection must preserve its owned candidate, exact foreign blocker, "
          "layer, deficit and clearance authority");
  regional.configured.enable_spline_realization = false;
  regional.configured.enable_straight_realization = true;
  require(select().choices.size() == 1U,
          "straight whole-scene realization must not reject a via against a foreign coarse "
          "fragment that another window may replace");
  regional.configured.enable_spline_realization = true;
  regional.configured.enable_straight_realization = false;

  // A foreign coarse centreline can cross A's selected via, yet be replaced by B's legal
  // dogleg in the assembled straight transaction. The source has no copper to retain.
  Board transaction_board;
  transaction_board.outline = rectangle(0, 0, 40, 40);
  transaction_board.layers = {{LayerId{"F.Cu"}, "F.Cu", LayerKind::copper, 0},
                              {LayerId{"B.Cu"}, "B.Cu", LayerKind::copper, 1}};
  const NetId transaction_a{"transaction-a"};
  const NetId transaction_b{"transaction-b"};
  transaction_board.nets = {{transaction_a, "A", 0}, {transaction_b, "B", 0}};
  transaction_board.rules.default_via_diameter = Nanometres::from_mm(0.7);
  transaction_board.rules.default_via_drill = Nanometres::from_mm(0.3);
  const auto transaction_rules = flux::resolve_net_routing_rules(transaction_board, transaction_a);
  require(transaction_rules.via_diameter == Nanometres::from_mm(0.7) &&
              transaction_rules.via_drill == Nanometres::from_mm(0.3),
          "the transaction fixture must resolve the selected via's actual source rule");
  const auto terminal = [](std::string id, const NetId& net, Point position,
                           std::vector<LayerId> layers) {
    Footprint footprint;
    footprint.id = ObjectId{id};
    footprint.reference = id;
    footprint.position = position;
    footprint.courtyard = local_rectangle(1.0, 1.0);
    footprint.pads = {{ObjectId{id + "-pad"}, net, Point{}, std::move(layers)}};
    return footprint;
  };
  transaction_board.footprints = {
      terminal("a-front", transaction_a, mm(5, 10), {LayerId{"F.Cu"}}),
      terminal("a-back", transaction_a, mm(5, 9), {LayerId{"B.Cu"}}),
      terminal("b-start", transaction_b, mm(10, 5), {LayerId{"F.Cu"}}),
      terminal("b-end", transaction_b, mm(10, 15), {LayerId{"F.Cu"}})};
  flux::GlobalRouteResult transaction_global;
  transaction_global.paths = {
      {transaction_a, 0U, {{mm(5, 10), LayerId{"F.Cu"}}, {mm(10.15, 10), LayerId{"F.Cu"}},
                           {mm(10.15, 10), LayerId{"B.Cu"}}, {mm(9.15, 9), LayerId{"B.Cu"}},
                           {mm(5, 9), LayerId{"B.Cu"}}}},
      {transaction_b, 0U, {{mm(10, 5), LayerId{"F.Cu"}}, {mm(10, 15), LayerId{"F.Cu"}}}}};
  const auto coarse_transaction_oracle =
      flux::probe_quantized_copper_feasibility(transaction_board, transaction_global);
  require(transaction_board.tracks.empty() && transaction_board.vias.empty() &&
              coarse_transaction_oracle.drc_violations > 0U &&
              coarse_transaction_oracle.open_connections == 0U,
          "the original foreign coarse centreline and selected via must be illegal without "
          "being retained as source copper");
  flux::RegionalRouteResult transaction_regional;
  transaction_regional.configured.frozen_copper_context = true;
  transaction_regional.configured.enable_spline_realization = false;
  transaction_regional.configured.enable_straight_realization = true;
  const auto make_fragment = [](flux::RegionalRouteFragmentId id, LayerId layer, Point start, Point end,
                                std::size_t first_segment) {
    flux::RegionalRouteFragment fragment;
    fragment.id = std::move(id);
    fragment.layer = std::move(layer);
    fragment.start = start;
    fragment.end = end;
    fragment.width = Nanometres::from_mm(0.25);
    fragment.clearance = Nanometres::from_mm(0.2);
    fragment.first_source_segment = first_segment;
    fragment.last_source_segment = first_segment;
    return fragment;
  };
  const auto a_front = make_fragment({transaction_a, 0U, 0U, 0U}, LayerId{"F.Cu"},
                                     mm(5, 10), mm(10.15, 10), 0U);
  const auto a_back = make_fragment({transaction_a, 0U, 0U, 1U}, LayerId{"B.Cu"},
                                    mm(10.15, 10), mm(5, 9), 2U);
  const auto b_original = make_fragment({transaction_b, 0U, 0U, 0U}, LayerId{"F.Cu"},
                                        mm(10, 5), mm(10, 15), 0U);
  transaction_regional.fragments = {a_front, a_back, b_original};
  flux::RegionalViaChoice transaction_via;
  transaction_via.id = 0U;
  transaction_via.region_id = 0U;
  transaction_via.net = transaction_a;
  transaction_via.site = mm(10.15, 10);
  transaction_via.span = {LayerId{"F.Cu"}, LayerId{"B.Cu"}};
  transaction_via.diameter = Nanometres::from_mm(0.7);
  transaction_via.drill = Nanometres::from_mm(0.3);
  transaction_via.selected_global_site = true;
  transaction_regional.via_choices = {transaction_via};
  flux::RegionalRoutingWindow transaction_window_a;
  transaction_window_a.id = 0U;
  transaction_window_a.fragments = {a_front.id, a_back.id};
  transaction_window_a.owned_fragments = transaction_window_a.fragments;
  flux::RegionalRoutingWindow transaction_window_b;
  transaction_window_b.id = 1U;
  transaction_window_b.fragments = {b_original.id};
  transaction_window_b.owned_fragments = transaction_window_b.fragments;
  transaction_regional.windows = {transaction_window_a, transaction_window_b};
  flux::RegionalRouteTemplate a_front_template;
  a_front_template.id = 0U;
  a_front_template.fragment = a_front.id;
  a_front_template.routing_layer = LayerId{"F.Cu"};
  a_front_template.selected_global_topology = true;
  a_front_template.exit_via_choice = 0U;
  flux::RegionalRouteTemplate a_back_template;
  a_back_template.id = 1U;
  a_back_template.fragment = a_back.id;
  a_back_template.routing_layer = LayerId{"B.Cu"};
  a_back_template.selected_global_topology = false;
  a_back_template.entry_via_choice = 0U;
  a_back_template.topology_centerline = {mm(10.15, 10), mm(9.15, 9), mm(5, 9)};
  flux::RegionalBundleAssignment transaction_assignment_a;
  transaction_assignment_a.window_id = transaction_window_a.id;
  transaction_assignment_a.templates = {a_front_template, a_back_template};
  transaction_assignment_a.selected_templates = {0U, 1U};
  transaction_assignment_a.selected_via_choices = {0U};
  transaction_assignment_a.stop_reason = flux::RegionalBundleStopReason::complete;
  flux::RegionalRouteTemplate b_template;
  b_template.id = 0U;
  b_template.fragment = b_original.id;
  b_template.routing_layer = LayerId{"F.Cu"};
  b_template.selected_global_topology = false;
  b_template.topology_centerline = {mm(10, 5), mm(10, 8), mm(11, 9),
                                    mm(11, 11), mm(10, 12), mm(10, 15)};
  flux::RegionalBundleAssignment transaction_assignment_b;
  transaction_assignment_b.window_id = transaction_window_b.id;
  transaction_assignment_b.templates = {b_template};
  transaction_assignment_b.selected_templates = {0U};
  transaction_assignment_b.stop_reason = flux::RegionalBundleStopReason::complete;
  transaction_regional.bundle_assignments = {transaction_assignment_a, transaction_assignment_b};
  flux::realize_regional_straight_candidate(transaction_board, transaction_global,
                                             transaction_regional);
  require(transaction_regional.spline_candidate_available &&
              transaction_regional.native_scene_evaluated && transaction_regional.native_scene_complete &&
              transaction_regional.native_scene_conflicts == 0U &&
              transaction_regional.straight_scene_open_connections == 0U &&
              transaction_regional.straight_scene_drc_violations == 0U &&
              transaction_regional.native_realization_manifest.size() == 3U &&
              std::any_of(transaction_regional.native_realization_manifest.begin(),
                          transaction_regional.native_realization_manifest.end(), [&](const auto& route) {
                            return route.fragment == b_original.id && route.centerline ==
                                b_template.topology_centerline;
                          }),
          "the assembled straight transaction must replace B's colliding coarse line with its "
          "dogleg and certify the resulting copper scene");
  auto finalized_transaction = transaction_global;
  finalized_transaction.paths = transaction_regional.spline_candidate_paths;
  const auto finalized_transaction_oracle =
      flux::probe_quantized_copper_feasibility(transaction_board, finalized_transaction);
  require(finalized_transaction_oracle.drc_violations == 0U &&
              finalized_transaction_oracle.open_connections == 0U,
          "the whole-scene quantized oracle must certify the assembled dogleg transaction");
  regional.windows = {window};
  flux::RegionalBundleAssignment failed_assignment;
  failed_assignment.window_id = window.id;
  failed_assignment.stop_reason = flux::RegionalBundleStopReason::spline_realization_failed;
  failed_assignment.frozen_copper_conflicts = fixed_site_rejected.frozen_copper_conflicts;
  regional.bundle_assignments = {failed_assignment};
  flux::RegionalRouterConfig scope_config;
  scope_config.frozen_copper_context = true;
  scope_config.expand_frozen_copper_scope = true;
  require(flux::append_regional_frozen_scope_witnesses(regional, scope_config) &&
              scope_config.frozen_copper_scope_witnesses ==
                  fixed_site_rejected.frozen_copper_conflicts,
          "an early frozen-via witness must create an exact ownership-expansion obligation");
  scope_config.frozen_copper_scope_witnesses.clear();
  regional.windows.front().owned_fragments.push_back(frozen.id);
  std::sort(regional.windows.front().owned_fragments.begin(),
            regional.windows.front().owned_fragments.end());
  require(!flux::append_regional_frozen_scope_witnesses(regional, scope_config),
          "an owned frozen fragment must never become a foreign scope-expansion witness");
  regional.windows.front().owned_fragments = {candidate.id};

  regional.configured.frozen_copper_context = false;
  require(select().choices.size() == 1U,
          "the frozen-copper access predicate must remain disabled outside the explicit context");
  regional.configured.frozen_copper_context = true;

  window.owned_fragments.push_back(frozen.id);
  std::sort(window.owned_fragments.begin(), window.owned_fragments.end());
  require(select().choices.size() == 1U,
          "a movable owned lane must remain available to later packing instead of rejecting its "
          "coarse centerline");
  window.owned_fragments = {candidate.id};

  regional.fragments.back().layer = LayerId{"In1.Cu"};
  require(select().choices.size() == 1U,
          "a frozen lane outside the selected via span must not constrain the access site");
  regional.fragments.back().layer = LayerId{"F.Cu"};

  regional.via_choices.front().site = mm(10.675, 10.0);
  require(select().choices.size() == 1U,
          "an exact copper-clearance boundary must remain legal under the strict DRC predicate");
  regional.via_choices.front().site = mm(10.8, 10.0);
  board.rules.hole_clearance = Nanometres::from_mm(0.8);
  require(select().choices.empty(),
          "via drill clearance against frozen foreign copper must use the source DRC rule when "
          "it exceeds the via-copper envelope");

  board.rules.hole_clearance = Nanometres{};
  regional.via_choices.front().site = mm(10.15, 10.0);
  auto second_candidate = candidate;
  second_candidate.id.fragment_index = 1U;
  second_candidate.start = mm(5.0, 11.0);
  second_candidate.end = mm(20.0, 11.0);
  regional.fragments.push_back(second_candidate);
  auto second_route = route;
  second_route.id = 1U;
  second_route.fragment = second_candidate.id;
  auto third_candidate = candidate;
  third_candidate.id.fragment_index = 2U;
  third_candidate.start = mm(5.0, 12.0);
  third_candidate.end = mm(20.0, 12.0);
  regional.fragments.push_back(third_candidate);
  auto third_route = route;
  third_route.id = 2U;
  third_route.fragment = third_candidate.id;
  flux::RegionalRoutingWindow shared_window = window;
  shared_window.fragments = {candidate.id, second_candidate.id, third_candidate.id};
  shared_window.owned_fragments = shared_window.fragments;
  std::sort(shared_window.owned_fragments.begin(), shared_window.owned_fragments.end());
  const auto shared_via_rejected = flux::enumerate_regional_bundle_choices(
      board, regional, shared_window, {route, second_route, third_route}, {{0U}, {1U}, {2U}}, 16U);
  require(shared_via_rejected.choices.empty() &&
              shared_via_rejected.frozen_copper_conflicts.size() == 3U &&
              std::set<flux::RegionalRouteFragmentId>{
                  shared_via_rejected.frozen_copper_conflicts[0].candidate,
                  shared_via_rejected.frozen_copper_conflicts[1].candidate,
                  shared_via_rejected.frozen_copper_conflicts[2].candidate} ==
                  std::set<flux::RegionalRouteFragmentId>{candidate.id, second_candidate.id,
                                                           third_candidate.id},
          "a pairwise forward rejection must retain the third owned fragment incident to the "
          "same rejected via region");
  regional.windows = {shared_window};
  regional.bundle_assignments.front().frozen_copper_conflicts =
      shared_via_rejected.frozen_copper_conflicts;
  scope_config.frozen_copper_scope_witnesses.clear();
  require(flux::append_regional_frozen_scope_witnesses(regional, scope_config) &&
              scope_config.frozen_copper_scope_witnesses.size() == 3U,
          "scope expansion must preserve every owned incident fragment under the witness budget");
  regional.bundle_assignments.front().native_frozen_copper_conflicts = {
      {candidate.id, frozen.id, LayerId{"B.Cu"}, Nanometres{1}, "native-candidate"}};
  scope_config.frozen_copper_scope_witnesses.clear();
  require(flux::append_regional_frozen_scope_witnesses(regional, scope_config) &&
              scope_config.frozen_copper_scope_witnesses.size() == 4U,
          "scope expansion must retain both access-search and current native-candidate evidence");

  // Access forward checking can reject this via group before its final fragment is selected.
  // Witness collection must still retain the whole owned domain, and must not inherit the
  // unrelated access-blocker cap of sixteen individual fragments.
  std::vector<flux::RegionalRouteFragment> wide_group_fragments;
  std::vector<flux::RegionalRouteTemplate> wide_group_templates;
  flux::RegionalRoutingWindow wide_group_window = window;
  for (std::size_t index = 0U; index < 17U; ++index) {
    auto group_fragment = candidate;
    group_fragment.id.fragment_index = index + 10U;
    group_fragment.start.y += Nanometres{static_cast<std::int64_t>(index * 10'000U)};
    group_fragment.end.y += Nanometres{static_cast<std::int64_t>(index * 10'000U)};
    wide_group_fragments.push_back(group_fragment);
    auto group_template = route;
    group_template.id = index;
    group_template.fragment = group_fragment.id;
    wide_group_templates.push_back(group_template);
    wide_group_window.fragments.push_back(group_fragment.id);
    wide_group_window.owned_fragments.push_back(group_fragment.id);
  }
  regional.fragments.insert(regional.fragments.end(), wide_group_fragments.begin(),
                            wide_group_fragments.end());
  std::sort(wide_group_window.owned_fragments.begin(), wide_group_window.owned_fragments.end());
  wide_group_window.owned_fragments.erase(
      std::unique(wide_group_window.owned_fragments.begin(), wide_group_window.owned_fragments.end()),
      wide_group_window.owned_fragments.end());
  std::vector<std::vector<std::size_t>> wide_group_domains;
  for (std::size_t index = 0U; index < wide_group_templates.size(); ++index)
    wide_group_domains.push_back({index});
  const auto wide_group_rejected = flux::enumerate_regional_bundle_choices(
      board, regional, wide_group_window, wide_group_templates, wide_group_domains, 256U);
  std::set<flux::RegionalRouteFragmentId> wide_group_candidates;
  for (const auto& witness : wide_group_rejected.frozen_copper_conflicts)
    wide_group_candidates.insert(witness.candidate);
  require(wide_group_rejected.choices.empty() && wide_group_candidates.size() == 17U,
          "a partial forward rejection must retain all seventeen owned fragments incident to "
          "the rejected via group, without the sixteen-blocker truncation");
}

void regional_spline_tiers_are_selected_breadth_first() {
  std::vector<flux::RegionalBundleAssignment> assignments(5U);
  assignments[0].window_id = 3U;
  assignments[0].stop_reason = flux::RegionalBundleStopReason::complete;
  assignments[0].spline_candidate_reserve.push_back({});
  assignments[1].window_id = 1U;
  assignments[1].stop_reason = flux::RegionalBundleStopReason::spline_realization_failed;
  assignments[1].spline_candidate_reserve.push_back({});
  assignments[2].window_id = 2U;
  assignments[2].stop_reason = flux::RegionalBundleStopReason::spline_realization_failed;
  assignments[3].window_id = 4U;
  assignments[3].stop_reason = flux::RegionalBundleStopReason::spline_realization_failed;
  assignments[3].spline_candidate_reserve.push_back({});
  assignments[3].spline_fallback_waves = 1U;
  assignments[4] = assignments[1];

  require(flux::select_regional_spline_tier_windows(assignments, 0U) ==
              std::vector<std::size_t>({3U, 1U, 2U, 4U}),
          "the active tier must retain every window once in stable assignment order");
  require(flux::select_regional_spline_tier_windows(assignments, 1U) ==
              std::vector<std::size_t>({1U}),
          "fallback wave one must contain only failed windows with an unconsumed reserve");
  assignments[1].spline_fallback_waves = 1U;
  assignments[4].spline_fallback_waves = 1U;
  require(flux::select_regional_spline_tier_windows(assignments, 2U) ==
              std::vector<std::size_t>({1U, 4U}),
          "the next wave must admit each still-failed reserve window exactly once");
}

void adaptive_portal_direction_is_independent_of_path_sampling() {
  // Unequal neighboring cells share a vertical portal but their centers need not align.
  // Inserting collinear points must not change which side of that portal the path occupies.
  flux::NavMeshTopologyGate gate;
  gate.left_boundary = mm(3.0, 0.0);
  gate.right_boundary = mm(3.0, 1.0);
  gate.from_center = mm(2.0, 0.5);
  gate.to_center = mm(4.0, 0.75);
  const auto signature = flux::make_navmesh_topology_signature({gate});
  const std::vector<Point> coarse{mm(2.0, 0.95), mm(4.0, 0.95)};
  const std::vector<Point> sampled{coarse.front(), mm(2.99, 0.95), mm(3.01, 0.95), coarse.back()};
  require(flux::validate_navmesh_topology_signature(signature, coarse).preserved &&
              flux::validate_navmesh_topology_signature(signature, sampled).preserved,
          "a skew-center adaptive portal must accept the same forward crossing before and after "
          "collinear subdivision");
  auto reversed = sampled;
  std::reverse(reversed.begin(), reversed.end());
  require(flux::validate_navmesh_topology_signature(signature, reversed).reason ==
              flux::NavMeshTopologyCheckReason::reversed_gate,
          "portal-normal orientation must still reject reverse traversal");
  const std::vector<Point> repeated{mm(2.0, 0.95), mm(4.0, 0.95), mm(2.0, 0.95), mm(4.0, 0.95)};
  require(flux::validate_navmesh_topology_signature(signature, repeated).reason ==
              flux::NavMeshTopologyCheckReason::repeated_gate,
          "portal-normal orientation must still reject multiple crossings");
  const std::vector<Point> tangent{mm(2.0, 0.95), mm(3.0, 0.95), mm(2.0, 0.5)};
  require(!flux::validate_navmesh_topology_signature(signature, tangent).preserved,
          "touching a portal without changing sides is not a crossing");
  std::swap(gate.left_boundary, gate.right_boundary);
  require(flux::validate_navmesh_topology_signature(
              flux::make_navmesh_topology_signature({gate}), sampled).preserved,
          "boundary vertex order must not override the directed cell transition");
}

void short_terminal_dogleg_has_a_regular_ph_realization() {
  const std::vector<Point> points{mm(26.0, 24.0), mm(26.0, 36.5), mm(26.5, 36.5),
                                  mm(26.5, 37.0), mm(31.5, 37.0), mm(31.5, 37.5),
                                  mm(32.5, 37.5), mm(32.5, 41.0), mm(32.54, 41.0)};
  std::string diagnostic;
  std::size_t legal_starts = 0U;
  for (const auto guard_nm :
       {1LL, 5'000LL, 10'000LL, 20'000LL, 50'000LL, 100'000LL, 250'000LL, 500'000LL}) {
    const auto candidate = flux::make_corner_rounded_quintic_ph_route(points, Nanometres{guard_nm});
    if (!candidate) {
      diagnostic += std::to_string(guard_nm) + ":missing;";
      continue;
    }
    const auto shape = flux::certify_quintic_ph_shape(*candidate);
    legal_starts += shape.legal ? 1U : 0U;
    diagnostic += std::to_string(guard_nm) + ":" + std::to_string(shape.violations()) + "/" +
                  std::to_string(shape.unintended_inflections) + "/" +
                  std::to_string(candidate->control_points.size()) + ";";
  }
  require(legal_starts > 0U,
          "an extreme short terminal run-up must retain at least one regular, non-wiggling PH "
          "interpolation start: " +
              diagnostic);

  auto source = two_pad_board();
  source.footprints.front().position = points.front();
  source.footprints.back().position = points.back();
  flux::GlobalRouteResult global;
  global.stop_reason = GlobalRouterStopReason::complete;
  flux::GlobalRoutePath path;
  path.net = NetId{"net-1"};
  for (const auto point : points)
    path.points.push_back({point, LayerId{"F.Cu"}});
  global.paths.push_back(std::move(path));
  flux::DetailedRouterConfig config;
  config.pull_tight_45_degrees = false;
  config.continuous_refinement = false;
  config.circular_arc_refinement = true;
  config.spline_minimum_bend_radius = Nanometres{0};
  const auto detailed = flux::materialize_detailed_route(source, global, config);
  const auto retained_planning_stub =
      detailed.candidate &&
      std::any_of(
          detailed.candidate->tracks.begin(), detailed.candidate->tracks.end(),
          [&](const auto& track) {
            return track.net == NetId{"net-1"} &&
                   ((track.start == points[points.size() - 2U] && track.end == points.back()) ||
                    (track.end == points[points.size() - 2U] && track.start == points.back()));
          });
  require(detailed.complete() && detailed.candidate && detailed.fitted_quintic_ph_runs > 0U &&
              detailed.unrounded_non_junction_bends == 0U && !retained_planning_stub,
          "a sub-cell planning run-up may not impose a mandatory pad tangent when its exact "
          "same-homotopy replacement chord is legal: " +
              detailed.diagnostic + "/" + detailed.spline_navmesh_last_topology_diagnostic);
}

void fallback_corridor_preserves_curvature_room() {
  auto source = two_pad_board();
  Keepout center;
  center.id = ObjectId{"fallback-corner-obstacle"};
  center.area = rectangle(23.0, 31.0, 77.0, 45.0);
  center.layers = {LayerId{"F.Cu"}};
  center.prohibit.tracks = true;
  center.prohibit.vias = true;
  source.keepouts.push_back(std::move(center));
  flux::GlobalRouteResult global;
  global.stop_reason = GlobalRouterStopReason::complete;
  flux::GlobalRoutePath path;
  path.net = NetId{"net-1"};
  for (const auto point : {mm(20.0, 40.0), mm(20.0, 30.0),
                           mm(80.0, 30.0), mm(80.0, 40.0)})
    path.points.push_back({point, LayerId{"F.Cu"}});
  global.paths.push_back(std::move(path));
  flux::DetailedRouterConfig config;
  config.pull_tight_45_degrees = false;
  config.continuous_refinement = false;
  config.circular_arc_refinement = true;
  config.max_spline_navmesh_search_expansions_per_run = 1U;
  config.spline_minimum_bend_radius = Nanometres::from_mm(1.0);
  const auto detailed = flux::materialize_detailed_route(source, global, config);
  require(detailed.complete() && detailed.candidate &&
              detailed.spline_navmesh_certificate_fallbacks > 0U &&
              detailed.unrounded_non_junction_bends == 0U && detailed.copper_drc.ok() &&
              detailed.connectivity.open_connections == 0U,
          "right-angle source-corridor fallback must retain space for a legal minimum-radius curve: " +
              detailed.diagnostic + "/" + detailed.spline_navmesh_last_topology_diagnostic);
}

void fallback_corridor_retains_one_sided_bend_room() {
  auto source = two_pad_board();
  // The vertical approach is only 50 micrometres from the offset board edge,
  // but there is ample room on its right for a one-millimetre-radius bend.
  source.outline = rectangle(19.325, 0.0, 100.0, 80.0);
  for (auto& footprint : source.footprints) {
    footprint.courtyard = local_rectangle(0.15, 0.15);
    footprint.pads.front().width = Nanometres{250000};
    footprint.pads.front().height = Nanometres{250000};
  }
  flux::GlobalRouteResult global;
  global.stop_reason = GlobalRouterStopReason::complete;
  flux::GlobalRoutePath path;
  path.net = NetId{"net-1"};
  for (const auto point : {mm(20.0, 40.0), mm(20.0, 30.0),
                           mm(80.0, 30.0), mm(80.0, 40.0)})
    path.points.push_back({point, LayerId{"F.Cu"}});
  global.paths.push_back(std::move(path));
  flux::DetailedRouterConfig config;
  config.pull_tight_45_degrees = false;
  config.continuous_refinement = false;
  config.circular_arc_refinement = true;
  config.max_spline_navmesh_search_expansions_per_run = 1U;
  config.spline_minimum_bend_radius = Nanometres::from_mm(1.0);
  for (unsigned reflection = 0U; reflection < 4U; ++reflection) {
    auto reflected_source = source;
    auto reflected_global = global;
    const auto reflect = [reflection](Point point) {
      if (reflection & 1U)
        point.x = Nanometres{100000000 - point.x.value()};
      if (reflection & 2U)
        point.y = Nanometres{80000000 - point.y.value()};
      return point;
    };
    if (reflection & 1U)
      reflected_source.outline = rectangle(0.0, 0.0, 80.675, 80.0);
    for (auto& footprint : reflected_source.footprints)
      footprint.position = reflect(footprint.position);
    for (auto& point : reflected_global.paths.front().points)
      point.position = reflect(point.position);
    const auto detailed = flux::materialize_detailed_route(reflected_source, reflected_global, config);
    require(detailed.complete() && detailed.candidate &&
                detailed.spline_navmesh_certificate_fallbacks > 0U &&
                detailed.unrounded_non_junction_bends == 0U && detailed.copper_drc.ok() &&
                detailed.connectivity.open_connections == 0U,
            "an edge-constrained source corridor must preserve independently legal inward room: " +
                std::to_string(reflection) + "/" + detailed.diagnostic + "/" +
                detailed.spline_last_materialization_diagnostic);
  }
}

void corner_blends_preserve_straight_run_tangents() {
  for (const auto sign : {-1.0L, 1.0L}) {
    const std::vector<Point> points{mm(10.0 * sign, 10.0), mm(20.0 * sign, 10.0),
                                    mm(20.0 * sign, 30.0), mm(10.0 * sign, 30.0)};
    const auto ordinary = flux::make_corner_rounded_quintic_ph_route(
        points, Nanometres::from_mm(1.0));
    auto aligned = flux::make_run_aligned_quintic_ph_route(points, Nanometres::from_mm(1.0));
    require(ordinary && aligned, "both corner seed families must interpolate the micro-route");
    const auto maximum_outward = [sign](const auto& route) {
      long double result = -std::numeric_limits<long double>::infinity();
      for (std::size_t i = 0U; i <= 512U; ++i)
        result = std::max(result, sign * flux::evaluate_quintic_ph(route, i / 512.0L).x);
      return result;
    };
    require(maximum_outward(*ordinary) > 20050000.0L,
            "bisector shoulder tangents must reproduce an outward bow beyond a 50um margin");
    const auto check = [&] {
      require(aligned->tangent_continuous && maximum_outward(*aligned) <= 20000000.01L &&
                  flux::certify_quintic_ph_shape(*aligned).legal,
              "run-aligned shoulders must keep the connecting PH run straight and regular");
      require(aligned->node_tangent_directions.size() == aligned->control_points.size(),
              "Hermite direction constraints must stay aligned with interpolation nodes");
    };
    check();
    require(flux::rebuild_quintic_ph_route(*aligned), "aligned seed must rebuild");
    check();
    require(flux::insert_quintic_ph_interpolation_node(*aligned, 0.5L),
            "the straight PH span must support exact adaptive insertion");
    check();
    require(flux::rebuild_quintic_ph_route(*aligned), "inserted direction constraints must rebuild");
    check();
    auto malformed = *aligned;
    malformed.node_tangent_directions.pop_back();
    require(!flux::rebuild_quintic_ph_route(malformed),
            "misaligned Hermite direction metadata must fail closed");
  }
}

void obstacle_aware_diagonals_and_curves_remain_capacity_and_drc_gated() {
  Board crossing;
  crossing.source_revision = "crossing-safe-diagonal-risk-fixture";
  crossing.outline = rectangle(0.0, 0.0, 30.0, 30.0);
  crossing.layers = {{LayerId{"F.Cu"}, "F.Cu", LayerKind::copper, 0}};
  crossing.nets = {{NetId{"diagonal-a"}, "DIAGONAL_A", 2}, {NetId{"diagonal-b"}, "DIAGONAL_B", 1}};
  const auto terminal = [](std::string id, NetId net, Point position) {
    Footprint footprint;
    footprint.id = ObjectId{id};
    footprint.reference = id;
    footprint.position = position;
    footprint.courtyard = local_rectangle(0.4, 0.4);
    footprint.pads = {{ObjectId{id + "-pad"}, std::move(net), Point{}, {LayerId{"F.Cu"}}}};
    return footprint;
  };
  crossing.footprints = {terminal("A1", NetId{"diagonal-a"}, mm(10.0, 10.0)),
                         terminal("A2", NetId{"diagonal-a"}, mm(20.0, 20.0)),
                         terminal("B1", NetId{"diagonal-b"}, mm(20.0, 10.0)),
                         terminal("B2", NetId{"diagonal-b"}, mm(10.0, 20.0))};
  GlobalRouterConfig crossing_config;
  crossing_config.grid_pitch = Nanometres::from_mm(10.0);
  crossing_config.max_iterations = 1U;
  crossing_config.max_pin_escape_grid_radius = 0U;
  crossing_config.max_pin_access_nodes_per_terminal = 1U;
  crossing_config.use_free_space_corridors = false;
  crossing_config.allow_crossing_safe_diagonal_grid_edges = true;
  crossing_config.clearance_shadow_cost_quantum = Nanometres::from_mm(0.05);
  crossing_config.present_congestion_cost = 0U;
  crossing_config.history_congestion_cost = 0U;
  const auto crossed = flux::route_global(crossing, crossing_config);
  const auto shared_cell = std::find_if(
      crossed.resource_usage.begin(), crossed.resource_usage.end(), [](const auto& usage) {
        return usage.kind == flux::RoutingResourceKind::planar_edge && usage.demand == 2U &&
               usage.capacity == 1U;
      });
  require(crossed.configured_crossing_safe_diagonal_grid_edges && crossed.paths.size() == 2U &&
              crossed.sampled_overflow == 1U && crossed.clearance_shadow_overflow > 0U &&
              !crossed.clearance_shadow_conflicts.empty() &&
              std::all_of(crossed.clearance_shadow_conflicts.begin(),
                          crossed.clearance_shadow_conflicts.end(),
                          [](const auto& conflict) {
                            return conflict.shadow_owner_first.has_value() &&
                                   conflict.shadow_owner_second.has_value();
                          }) &&
              crossed.peak_clearance_shadow_history_increment > 1U &&
              crossed.clearance_shadow_history_increment >=
                  crossed.peak_clearance_shadow_history_increment &&
              crossed.residual_conflict_groups.size() == 1U &&
              shared_cell != crossed.resource_usage.end(),
          "opposite exact-legal cell diagonals must be discoverable but share one hard physical "
          "crossing resource whose two negotiated participants retain their own attribution");

  auto oracle_owned_config = crossing_config;
  oracle_owned_config.clearance_shadows_advisory_only = true;
  const auto oracle_owned = flux::route_global(crossing, oracle_owned_config);
  flux::RoutabilityProbeConfig advisory_probe_config;
  advisory_probe_config.clearance_shadows_contribute_to_history = false;
  const auto advisory_shadow_diagnoses =
      flux::probe_global_routability(oracle_owned, advisory_probe_config);
  const auto exact_oracle = flux::probe_quantized_copper_feasibility(crossing, oracle_owned);
  flux::RoutabilityKnowledge oracle_owned_knowledge;
  flux::accumulate_routability_knowledge(oracle_owned_knowledge, advisory_shadow_diagnoses,
                                        advisory_probe_config);
  flux::accumulate_routability_knowledge(oracle_owned_knowledge, exact_oracle.diagnoses,
                                        advisory_probe_config);
  require(oracle_owned.configured_clearance_shadows_advisory_only &&
              oracle_owned.clearance_shadow_overflow > 0U &&
              oracle_owned.clearance_shadow_conflicts.size() <=
                  crossed.clearance_shadow_conflicts.size() &&
              oracle_owned.clearance_shadow_raster_samples ==
                  oracle_owned.clearance_shadow_conflicts.size() +
                      oracle_owned.clearance_shadow_coalesced_samples &&
              oracle_owned.total_overflow == oracle_owned.sampled_overflow &&
              oracle_owned.clearance_shadow_history_increment == 0U &&
              oracle_owned.peak_clearance_shadow_history_increment == 0U &&
              !oracle_owned.residual_conflict_groups.empty() &&
              std::any_of(advisory_shadow_diagnoses.begin(), advisory_shadow_diagnoses.end(),
                          [](const auto& diagnosis) {
                            return !diagnosis.contributes_to_history &&
                                   diagnosis.detail.find("selected copper envelopes overlap") !=
                                       std::string::npos;
                          }) &&
              !exact_oracle.feasible() && !exact_oracle.diagnoses.empty() &&
              !oracle_owned_knowledge.history.empty() &&
              std::any_of(oracle_owned_knowledge.history.begin(),
                          oracle_owned_knowledge.history.end(), [](const auto& entry) {
                            return entry.stage == flux::RoutabilityFailureStage::quantization;
                          }),
          "oracle-owned physical feasibility must retain coalesced shadow scope evidence while "
          "charging only typed exact-copper no-goods: configured=" +
              std::to_string(oracle_owned.configured_clearance_shadows_advisory_only) +
              ", shadows=" + std::to_string(oracle_owned.clearance_shadow_overflow) + "/" +
              std::to_string(crossed.clearance_shadow_overflow) +
              ", overflow=" + std::to_string(oracle_owned.total_overflow) + "/" +
              std::to_string(oracle_owned.sampled_overflow) +
              ", shadow-history=" +
              std::to_string(oracle_owned.clearance_shadow_history_increment) +
              ", groups=" + std::to_string(oracle_owned.residual_conflict_groups.size()) +
              ", shadow-diag=" + std::to_string(advisory_shadow_diagnoses.size()) +
              ", exact=" + std::to_string(exact_oracle.feasible()) + "/" +
              std::to_string(exact_oracle.diagnoses.size()) +
              ", history=" + std::to_string(oracle_owned_knowledge.history.size()));

  auto diversified_crossing_config = crossing_config;
  diversified_crossing_config.max_iterations = 6U;
  diversified_crossing_config.atomic_complete_net_routing = true;
  diversified_crossing_config.enable_search_diversification = true;
  diversified_crossing_config.global_route_beam_width = 4U;
  diversified_crossing_config.max_large_neighborhood_attempts = 5U;
  diversified_crossing_config.large_neighborhood_stagnation_iterations = 1U;
  diversified_crossing_config.destroy_route_percent = 25U;
  diversified_crossing_config.max_routing_order_variants = 4U;
  diversified_crossing_config.routing_order_diversification_seed = 37U;
  const auto diversified_crossing = flux::route_global(crossing, diversified_crossing_config);
  const auto repeated_diversified_crossing =
      flux::route_global(crossing, diversified_crossing_config);
  require(diversified_crossing.configured_search_diversification &&
              diversified_crossing.configured_global_route_beam_width == 4U &&
              diversified_crossing.global_route_beam_peak > 0U &&
              diversified_crossing.global_route_beam_peak <= 4U &&
              diversified_crossing.temporary_illegal_checkpoints > 0U &&
              diversified_crossing.routing_order_variants_attempted > 0U &&
              diversified_crossing.large_neighborhood_attempts == 5U &&
              diversified_crossing.large_neighborhood_destroyed_nets >= 2U &&
              diversified_crossing.large_neighborhood_operators.size() == 5U &&
              diversified_crossing.large_neighborhood_operators.back() ==
                  "destroy_terminal_via_cluster_and_random_routes" &&
              diversified_crossing.paths == repeated_diversified_crossing.paths &&
              diversified_crossing.large_neighborhood_operators ==
                  repeated_diversified_crossing.large_neighborhood_operators &&
              diversified_crossing.routing_order_variants_attempted ==
                  repeated_diversified_crossing.routing_order_variants_attempted &&
              diversified_crossing.large_neighborhood_destroyed_nets ==
                  repeated_diversified_crossing.large_neighborhood_destroyed_nets,
          "a fixed diversification seed must preserve deterministic beam/order/destruction search "
          "while allowing bounded temporary physical illegality: " +
              std::to_string(diversified_crossing.global_route_beam_peak) + "/" +
              std::to_string(diversified_crossing.temporary_illegal_checkpoints) + "/" +
              std::to_string(diversified_crossing.routing_order_variants_attempted) + "/" +
              std::to_string(diversified_crossing.large_neighborhood_attempts) + "/" +
              std::to_string(diversified_crossing.large_neighborhood_destroyed_nets) + "/" +
              diversified_crossing.diagnostic);

  flux::RegionalRouterConfig terminal_portal_config;
  // The broad fixture repeats regional analysis for determinism and telemetry. Keep the optional
  // terminal topology portfolio out of this deliberately impossible one-layer crossing; its
  // production-scale fallback is exercised by the routing corpus instead. XPBD has its own
  // focused CPU/GPU test above; disable it here because this assertion compares complete results,
  // including deliberately non-deterministic host timing telemetry, across three repeated runs.
  terminal_portal_config.enable_terminal_portal_templates = false;
  terminal_portal_config.enable_xpbd_relaxation = false;
  terminal_portal_config.enable_xpbd_feedback = false;
  terminal_portal_config.max_same_layer_detour_checks = 0U;
  terminal_portal_config.max_confined_anisotropic_erosion_checks = 4U;
  terminal_portal_config.confined_anisotropic_erosion_flow.relaxation_iterations = 128U;
  terminal_portal_config.confined_anisotropic_erosion_flow.droplet_batches = 1U;
  terminal_portal_config.confined_anisotropic_erosion_flow.droplets_per_batch = 64U;
  terminal_portal_config.confined_anisotropic_erosion_flow.maximum_steps_per_droplet = 128U;
  const auto preflight_rejected = flux::analyze_regional_conflicts(crossing, crossed, terminal_portal_config);
  require(preflight_rejected.linear_scaffold_checks > 0 &&
              preflight_rejected.linear_scaffold_rejections > 0 &&
              preflight_rejected.spline_interpolation_portfolio_candidates == 0 &&
              !preflight_rejected.complete(),
          "an impossible fixed one-layer scaffold with joint relaxation disabled must fail before PH portfolios");
  // The remainder deliberately exercises legacy PH rejection telemetry on this impossible
  // crossing. Disable only its new early gate; the production gate is asserted immediately above.
  terminal_portal_config.require_legal_linear_scaffold = false;
  CollectingTelemetrySink regional_candidate_telemetry;
  const auto regional = flux::analyze_regional_conflicts(crossing, crossed, terminal_portal_config,
                                                         {}, &regional_candidate_telemetry);
  const auto has_context_modes = [](const auto& events, bool spline, bool straight) {
    return std::any_of(events.begin(), events.end(), [&](const auto& event) {
      if (event.kind != flux::RouterTelemetryEventKind::search_summary ||
          !event.stable_id.starts_with("regional-bundle-context:"))
        return false;
      return event.detail.find("\"schema\":4") != std::string::npos &&
             event.detail.find(std::string{"\"enable_spline_realization\":"} +
                               (spline ? "true" : "false")) != std::string::npos &&
             event.detail.find(std::string{"\"enable_straight_realization\":"} +
                               (straight ? "true" : "false")) != std::string::npos;
    });
  };
  auto straight_context_config = terminal_portal_config;
  straight_context_config.enable_spline_realization = false;
  straight_context_config.enable_straight_realization = true;
  CollectingTelemetrySink straight_context_telemetry;
  static_cast<void>(flux::analyze_regional_conflicts(crossing, crossed, straight_context_config,
                                                     {}, &straight_context_telemetry));
  const auto repeated_regional =
      flux::analyze_regional_conflicts(crossing, crossed, terminal_portal_config);
  flux::RouterTelemetryConfig regional_telemetry_config;
  regional_telemetry_config.queue_capacity = 128U;
  regional_telemetry_config.publish_rate_hz = 1'000U;
  regional_telemetry_config.level = flux::RouterTelemetryLevel::diagnostic;
  flux::RouterTelemetryService regional_telemetry(regional_telemetry_config);
  const auto observed_regional = flux::analyze_regional_conflicts(
      crossing, crossed, terminal_portal_config, {}, &regional_telemetry);
  regional_telemetry.stop();
  const auto regional_telemetry_state = regional_telemetry.state();
  const auto decoded_regional_snapshot =
      flux::telemetry::decode_events(flux::serialize_router_telemetry_snapshot(
          regional_telemetry_state, regional_telemetry.statistics()));
  const auto first_candidate_spline = std::find_if(
      regional_candidate_telemetry.events.begin(), regional_candidate_telemetry.events.end(),
      [](const auto& event) {
        return event.kind == flux::RouterTelemetryEventKind::route_candidate_selected &&
               event.route_geometry == flux::RouterTelemetryRouteGeometry::quintic_ph &&
               event.spline_control_points.size() >= 6U;
      });
  const auto spline_phase_completed = std::find_if(
      regional_candidate_telemetry.events.begin(), regional_candidate_telemetry.events.end(),
      [](const auto& event) {
        return event.kind == flux::RouterTelemetryEventKind::search_summary &&
               event.detail.find("regional spline realization completed") != std::string::npos;
      });
  require(
      regional == repeated_regional && regional == observed_regional && decoded_regional_snapshot &&
          decoded_regional_snapshot.events.size() == 1U &&
          decoded_regional_snapshot.events.front().dynamic_state_included &&
          decoded_regional_snapshot.events.front().conflicts.size() ==
              regional.conflict_edges.size() &&
          regional_telemetry_state.total_groups == regional.components.size() &&
          regional_telemetry_state.regional_conflicts == regional.conflict_edges.size() &&
          !regional_telemetry_state.observed_routes.empty() &&
          has_context_modes(regional_candidate_telemetry.events, true, false) &&
          has_context_modes(straight_context_telemetry.events, false, true) &&
          first_candidate_spline != regional_candidate_telemetry.events.end() &&
          spline_phase_completed != regional_candidate_telemetry.events.end() &&
          first_candidate_spline < spline_phase_completed &&
          std::any_of(
              regional_telemetry_state.observed_routes.begin(),
              regional_telemetry_state.observed_routes.end(),
              [](const auto& route) {
                return route.route_geometry == flux::RouterTelemetryRouteGeometry::quintic_ph &&
                       route.spline_degree == 5U && route.spline_control_points.size() >= 6U;
              }) &&
          regional.source_clearance_shadow_conflicts == crossed.clearance_shadow_conflicts.size() &&
          regional.spline_topology_seed_gpu_batches > 0U &&
          regional.spline_topology_seed_gpu_fallbacks < regional.spline_topology_seed_gpu_batches &&
          regional.spline_topology_seed_gpu_candidates >=
              regional.spline_topology_seed_exact_candidates &&
          regional.spline_interpolation_gpu_batches > 0U &&
          regional.spline_interpolation_gpu_fallbacks < regional.spline_interpolation_gpu_batches &&
          regional.spline_interpolation_gpu_candidates >=
              regional.spline_interpolation_exact_candidates &&
          regional.spline_geometry_cache_hits > 0U &&
          ((regional.native_arc_windows_checked > 0U && regional.native_arc_pieces_checked > 0U) ||
           (regional.spline_centerline_certificate_pairs > 0U &&
            regional.spline_centerline_separation_proofs +
                    regional.spline_centerline_collision_proofs >
                0U)) &&
          (regional.spline_offset_broadphase_batches > 0U ||
           regional.spline_centerline_ambiguous_pairs == 0U) &&
          static_cast<std::uint64_t>(regional.spline_realization_us) > 0U &&
          regional.coalesced_clearance_shadow_samples +
                  regional.unmapped_clearance_shadow_conflicts ==
              regional.source_clearance_shadow_conflicts &&
          regional.lane_solver_queries > 0U && regional.lane_solver_states > 0U &&
          regional.lane_solver_cache_hits <= regional.lane_solver_queries &&
          regional.joint_spline_candidates_generated >=
              regional.joint_spline_active_candidates_retained &&
          regional.joint_spline_fallback_candidates_promoted <=
              regional.joint_spline_fallback_candidates_retained &&
          regional.joint_spline_fallback_candidates_remaining <=
              regional.joint_spline_fallback_candidates_retained &&
          regional.joint_spline_fallback_candidates_promoted +
                  regional.joint_spline_fallback_candidates_remaining <=
              regional.joint_spline_fallback_candidates_retained &&
          regional.joint_spline_candidates_evaluated <=
              regional.joint_spline_active_candidates_retained +
                  regional.joint_spline_fallback_candidates_promoted &&
          regional.joint_spline_fallback_windows_recovered <=
              regional.joint_spline_fallback_candidates_promoted &&
          regional.conflict_directed_topology_cores <=
              terminal_portal_config.max_conflict_directed_topology_windows &&
          regional.conflict_directed_windows_recovered +
                  regional.conflict_directed_windows_exhausted <=
              regional.conflict_directed_topology_cores &&
          (regional.conflict_directed_topology_cores == 0U ||
           regional.conflict_directed_master_reruns >= regional.conflict_directed_topology_cores) &&
          (regional.conflict_directed_topology_cores == 0U ||
           regional.confined_anisotropic_erosion_checks > 0U) &&
          regional.confined_anisotropic_erosion_accepted +
                  regional.confined_anisotropic_erosion_rejected ==
              regional.confined_anisotropic_erosion_checks &&
          regional.confined_anisotropic_erosion_accepted <=
              regional.confined_anisotropic_erosion_gpu_routes +
                  regional.confined_anisotropic_erosion_cpu_routes &&
          regional.unmapped_clearance_shadow_conflicts == 0U && !regional.fragments.empty() &&
          !regional.conflict_edges.empty() && regional.components.size() == 1U &&
          regional.components.front().nets.size() == 2U &&
          regional.complete() == regional.spline_candidate_available &&
          (!regional.complete() ||
           std::all_of(regional.bundle_assignments.begin(), regional.bundle_assignments.end(),
                       [](const auto& assignment) { return assignment.complete(); })),
      "regional analysis must deterministically attribute every exact clearance-shadow "
      "observation to stable route fragments, "
      "retain a bounded compact active/fallback spline portfolio, and require an all-window "
      "spline candidate before bounded source conflicts count as regionally complete: " +
          std::to_string(regional.joint_spline_candidates_generated) + "/" +
          std::to_string(regional.joint_spline_active_candidates_retained) + "/" +
          std::to_string(regional.joint_spline_fallback_candidates_retained) + "/" +
          std::to_string(regional.joint_spline_fallback_candidates_promoted) + "/" +
          std::to_string(regional.joint_spline_fallback_candidates_remaining) + "/" +
          std::to_string(regional.joint_spline_candidates_evaluated) +
          "/cert=" + std::to_string(regional.spline_centerline_certificate_pairs) +
          "/native=" + std::to_string(regional.native_arc_windows_checked) + "/" +
          std::to_string(regional.native_arc_pieces_checked) + "/" +
          std::to_string(regional.native_arc_copper_violations));

  // A bounded joint shortlist must have identical authoritative selection whether alternatives
  // are realized serially or against private immutable snapshots in a parallel batch. Deliberately
  // exclude timing/cache counters: they describe scheduling rather than selected copper.
  auto serial_candidate_config = terminal_portal_config;
  serial_candidate_config.enable_terminal_portal_templates = true;
  serial_candidate_config.max_terminal_choices_per_terminal = 2U;
  serial_candidate_config.max_terminal_navmesh_template_checks = 16U;
  // The base crossing diagnostic intentionally disables detours so it can isolate the rejected
  // scaffold. Re-enable a bounded topology portfolio here: these distinct detours create the
  // independent joint candidates that the batch executor must actually evaluate.
  serial_candidate_config.max_same_layer_detour_checks = 32U;
  serial_candidate_config.max_same_layer_detour_offsets = 3U;
  serial_candidate_config.max_joint_spline_candidates = 4U;
  serial_candidate_config.max_joint_spline_active_pool_candidates = 8U;
  serial_candidate_config.max_legal_spline_candidates_per_window = 2U;
  serial_candidate_config.max_joint_spline_fallback_waves = 1U;
  serial_candidate_config.max_spline_candidate_workers = 1U;
  serial_candidate_config.max_spline_window_workers = 1U;
  const auto serial_candidates =
      flux::analyze_regional_conflicts(crossing, crossed, serial_candidate_config);
  auto parallel_candidate_config = serial_candidate_config;
  parallel_candidate_config.max_spline_candidate_workers = 4U;
  parallel_candidate_config.max_spline_window_workers = 1U;
  parallel_candidate_config.max_spline_cpu_workers = 4U;
  parallel_candidate_config.max_spline_window_scratch_bytes = 256U * 1024U * 1024U;
  parallel_candidate_config.estimated_spline_window_scratch_bytes = 64U * 1024U * 1024U;
  const auto parallel_candidates =
      flux::analyze_regional_conflicts(crossing, crossed, parallel_candidate_config);
  require(parallel_candidates.spline_candidate_parallel_tasks > 0U &&
              parallel_candidates.spline_candidate_parallel_workers > 1U &&
              serial_candidates.spline_candidate_parallel_tasks == 0U &&
              serial_candidates.stop_reason == parallel_candidates.stop_reason &&
              serial_candidates.spline_candidate_available ==
                  parallel_candidates.spline_candidate_available &&
              serial_candidates.native_scene_complete == parallel_candidates.native_scene_complete &&
              serial_candidates.native_scene_conflicts == parallel_candidates.native_scene_conflicts &&
              serial_candidates.spline_candidate_paths == parallel_candidates.spline_candidate_paths &&
              serial_candidates.bundle_assignments.size() ==
                  parallel_candidates.bundle_assignments.size(),
          "parallel regional candidate batches must preserve the serial physical outcome: tasks=" +
              std::to_string(parallel_candidates.spline_candidate_parallel_tasks) +
              ", workers=" + std::to_string(parallel_candidates.spline_candidate_parallel_workers) +
              ", stop_equal=" + std::to_string(serial_candidates.stop_reason == parallel_candidates.stop_reason) +
              ", available=" + std::to_string(serial_candidates.spline_candidate_available) + "/" +
              std::to_string(parallel_candidates.spline_candidate_available) +
              ", native=" + std::to_string(serial_candidates.native_scene_complete) + "/" +
              std::to_string(parallel_candidates.native_scene_complete) +
              ", conflicts=" + std::to_string(serial_candidates.native_scene_conflicts) + "/" +
              std::to_string(parallel_candidates.native_scene_conflicts) +
              ", paths_equal=" + std::to_string(serial_candidates.spline_candidate_paths == parallel_candidates.spline_candidate_paths) +
              ", assignments=" + std::to_string(serial_candidates.bundle_assignments.size()) + "/" +
              std::to_string(parallel_candidates.bundle_assignments.size()));
  for (std::size_t index = 0U; index < serial_candidates.bundle_assignments.size(); ++index) {
    const auto& serial_assignment = serial_candidates.bundle_assignments[index];
    const auto& parallel_assignment = parallel_candidates.bundle_assignments[index];
    require(serial_assignment.selected_templates == parallel_assignment.selected_templates &&
                serial_assignment.selected_via_choices == parallel_assignment.selected_via_choices &&
                serial_assignment.spline_candidate_rank == parallel_assignment.spline_candidate_rank &&
                serial_assignment.stop_reason == parallel_assignment.stop_reason &&
                serial_assignment.spline_realizations == parallel_assignment.spline_realizations,
            "parallel candidate reduction must retain the same ranked realized geometry");
  }
  std::stop_source candidate_stop;
  candidate_stop.request_stop();
  const auto cancelled_candidates = flux::analyze_regional_conflicts(
      crossing, crossed, parallel_candidate_config, candidate_stop.get_token());
  require(cancelled_candidates.stop_reason == flux::RegionalRouterStopReason::cancelled &&
              !cancelled_candidates.spline_candidate_available &&
              cancelled_candidates.spline_candidate_parallel_tasks == 0U &&
              std::none_of(cancelled_candidates.bundle_assignments.begin(),
                           cancelled_candidates.bundle_assignments.end(), [](const auto& assignment) {
                             return assignment.spline_candidate_realized ||
                                    !assignment.spline_realizations.empty();
                           }),
          "pre-cancelled regional candidate batches must not commit partial realization");

  flux::RoutingPlannerConfig deferred_regional_config;
  deferred_regional_config.curved_tracks = true; // Retained curve/physics scheduling regression.
  deferred_regional_config.full_strength_first_pass = false; // Explicit legacy scheduling coverage.
  deferred_regional_config.global = crossing_config;
  deferred_regional_config.max_routability_feedback_retries = 0U;
  deferred_regional_config.regional.max_nets_per_window = 1U;
  const auto deferred_regional_plan = flux::plan_routes_legacy(crossing, deferred_regional_config);
  require(
      !deferred_regional_plan.complete() &&
          deferred_regional_plan.regional.diagnostic !=
              "regional realization deferred until every coarse connection is routed" &&
          deferred_regional_plan.feedback_probes.size() == 1U &&
          deferred_regional_plan.feedback_probes.front().pass == 0U &&
          deferred_regional_plan.feedback_probes.front().candidates.size() == 1U &&
          deferred_regional_plan.feedback_probes.front().candidates.front().regional_consulted &&
          deferred_regional_plan.feedback_probes.front().history_entries_after_global >=
              deferred_regional_plan.feedback_probes.front().history_entries_before,
      "a complete coarse route must hand a spatial conflict component to regional window "
      "partitioning rather than rejecting its entire global connected component, and every "
      "outer physical-feedback probe must publish its history and regional hand-off state");

  auto broad_feedback_config = deferred_regional_config;
  broad_feedback_config.max_routability_feedback_retries = 1U;
  const auto broad_feedback_plan = flux::plan_routes_legacy(crossing, broad_feedback_config);
  auto repeated_physics_config = broad_feedback_config;
  repeated_physics_config.full_strength_first_pass = true;
  repeated_physics_config.global.max_iterations = 3U;
  repeated_physics_config.max_routability_feedback_retries = 2U;
  CollectingTelemetrySink repeated_physics_telemetry;
  const auto repeated_physics =
      flux::plan_routes_legacy(crossing, repeated_physics_config, {}, &repeated_physics_telemetry);
  require(!repeated_physics.complete() && repeated_physics.feedback_probes.size() == 3U &&
              repeated_physics.cumulative_global_iterations == 3U &&
              repeated_physics.feedback_probes.front().next_feedback_affected_nets >= 2U &&
              repeated_physics.regional.configured.enable_xpbd_relaxation &&
              repeated_physics.regional.configured.frozen_copper_context &&
              std::all_of(repeated_physics.feedback_probes.begin(), repeated_physics.feedback_probes.end(),
                  [](const auto& epoch) {
                    return !epoch.intermediate_regional_probe &&
                        std::any_of(epoch.candidates.begin(), epoch.candidates.end(), [](const auto& candidate) {
                          return candidate.regional_consulted && candidate.regional_spline_realization_enabled;
                        });
                  }),
          "failed physical windows must drive bounded rip-up/replan epochs with physics enabled every time");
  std::map<std::string, std::vector<flux::RouterTelemetryPathPoint>> replayed_global_paths;
  std::map<std::string, std::vector<flux::RouterTelemetryPathPoint>> last_global_attempt_paths;
  const auto is_global_route = [](const flux::RouterTelemetryEvent& event) {
    return event.stable_id.starts_with("global-route:");
  };
  for (const auto& event : repeated_physics_telemetry.events) {
    if (event.kind == flux::RouterTelemetryEventKind::route_removed && is_global_route(event)) {
      replayed_global_paths.erase(event.stable_id);
    } else if (event.kind == flux::RouterTelemetryEventKind::route_candidate_selected &&
               is_global_route(event)) {
      replayed_global_paths.insert_or_assign(event.stable_id, event.route_points);
    }
    if (event.kind == flux::RouterTelemetryEventKind::global_routing_finished) {
      last_global_attempt_paths = replayed_global_paths;
    }
  }
  std::map<std::string, std::vector<flux::RouterTelemetryPathPoint>> retained_global_paths;
  for (const auto& path : repeated_physics.coarse_global.paths) {
    std::vector<flux::RouterTelemetryPathPoint> points;
    points.reserve(path.points.size());
    for (const auto& point : path.points)
      points.push_back({point.position.x.value(), point.position.y.value(), point.layer});
    retained_global_paths.emplace(
        "global-route:" + path.net.value() + ':' + std::to_string(path.connection_index),
        std::move(points));
  }
  require(replayed_global_paths == retained_global_paths,
          "replayed active global-route telemetry must exactly restore the retained coarse "
          "checkpoint and exclude paths from a later discarded attempt");
  for (const auto& [id, ignored] : last_global_attempt_paths) {
    static_cast<void>(ignored);
    if (retained_global_paths.contains(id)) {
      continue;
    }
    require(std::any_of(repeated_physics_telemetry.events.begin(), repeated_physics_telemetry.events.end(),
                        [&](const auto& event) {
                          return event.kind == flux::RouterTelemetryEventKind::route_removed &&
                                 event.stable_id == id;
                        }),
            "a global-route ID absent from the retained checkpoint must be removed from the "
            "discarded final attempt");
  }
  auto straight_feedback_config = repeated_physics_config;
  straight_feedback_config.curved_tracks = false;
  const auto straight_feedback = flux::plan_routes_legacy(crossing, straight_feedback_config);
  require(!straight_feedback.feedback_probes.empty() &&
              straight_feedback.feedback_probes.front().intermediate_regional_probe &&
              straight_feedback.cumulative_global_iterations <= 3U &&
              std::all_of(straight_feedback.feedback_probes.begin(),
                          straight_feedback.feedback_probes.end(), [](const auto& epoch) {
                return std::all_of(epoch.candidates.begin(), epoch.candidates.end(),
                    [&](const auto& candidate) {
                      return !candidate.regional_spline_realization_enabled &&
                          (!epoch.intermediate_regional_probe ||
                           candidate.global_sampled_overflow == 0U || !candidate.regional_consulted);
                    });
              }),
          "straight conflicts must return to negotiation before expensive regional portfolios");
  // Distinct lattice rows fit abstract capacity, but wide copper overlaps at their terminal
  // approaches. Unlike the crossing fixture above, this reaches the negotiated topology gate.
  auto parallel = crossing;
  parallel.source_revision = "parallel-terminal-physical-feedback";
  parallel.rules.default_track_width = Nanometres::from_mm(0.6);
  parallel.rules.default_clearance = Nanometres::from_mm(0.2);
  parallel.nets = {{NetId{"a"}, "A", 0}, {NetId{"b"}, "B", 0}};
  parallel.footprints = {terminal("a1", NetId{"a"}, mm(5.0, 10.0)),
                         terminal("a2", NetId{"a"}, mm(25.0, 10.0)),
                         terminal("b1", NetId{"b"}, mm(5.0, 10.6)),
                         terminal("b2", NetId{"b"}, mm(25.0, 10.6))};
  for (auto& footprint : parallel.footprints) {
    footprint.courtyard = local_rectangle(0.2, 0.2);
    footprint.pads.front().width = Nanometres::from_mm(0.2);
    footprint.pads.front().height = Nanometres::from_mm(0.2);
  }
  flux::RoutingPlannerConfig assembled_feedback_config;
  require(!assembled_feedback_config.enable_assembled_straight_feedback,
          "reserved assembled straight feedback must remain opt-in by default");
  assembled_feedback_config.curved_tracks = false;
  assembled_feedback_config.full_strength_first_pass = false;
  assembled_feedback_config.max_routability_feedback_retries = 2U;
  assembled_feedback_config.global = crossing_config;
  assembled_feedback_config.global.grid_pitch = Nanometres::from_mm(1.0);
  assembled_feedback_config.global.max_iterations = 3U;
  assembled_feedback_config.global.allow_crossing_safe_diagonal_grid_edges = false;
  assembled_feedback_config.regional.max_nets_per_window = 2U;
  const auto default_assembled_feedback = flux::plan_routes_legacy(parallel, assembled_feedback_config);
  require(default_assembled_feedback.feedback_probes.size() == 3U &&
              default_assembled_feedback.feedback_probes[1].intermediate_regional_probe &&
              std::none_of(default_assembled_feedback.feedback_probes[1].candidates.begin(),
                           default_assembled_feedback.feedback_probes[1].candidates.end(),
                           [](const auto& candidate) {
                             return candidate.regional_straight_scene_evaluated;
                           }),
          "the default scheduler must retain the inexpensive intermediate straight probe");
  assembled_feedback_config.enable_assembled_straight_feedback = true;
  const auto assembled_feedback = flux::plan_routes_legacy(parallel, assembled_feedback_config);
  require(assembled_feedback.feedback_probes.size() == 3U &&
              assembled_feedback.cumulative_global_iterations == 3U &&
              !assembled_feedback.complete(),
          "assembled feedback must preserve the three-iteration budget and reject colliding copper: " +
              std::to_string(assembled_feedback.feedback_probes.size()) + "/" +
              std::to_string(assembled_feedback.cumulative_global_iterations) + "/" +
              assembled_feedback.diagnostic);
  // The assembled straight scene is the physical authority for the next epoch: its exact
  // failures must be published after regional realization, then consumed by the following
  // global retry.  This deliberately keeps curved/spline realization disabled.
  const auto interleaved_straight_retry = [&] {
    if (assembled_feedback.feedback_probes.size() >= 2U) {
      const auto i = assembled_feedback.feedback_probes.size() - 2U;
      const auto& assembled = assembled_feedback.feedback_probes[i];
      const auto& retry = assembled_feedback.feedback_probes[i + 1U];
      const bool exact_failure = std::any_of(
          assembled.candidates.begin(), assembled.candidates.end(), [](const auto& candidate) {
            return candidate.global_sampled_overflow == 0U && candidate.global_corridor_overflow == 0U &&
                candidate.regional_consulted && candidate.regional_straight_scene_evaluated &&
                candidate.regional_straight_scene_violations > 0U &&
                !candidate.regional_spline_realization_enabled;
          });
      if (!exact_failure ||
          assembled.history_pressure_after_regional <= assembled.history_pressure_after_global ||
          retry.history_entries_before != assembled.history_entries_after_regional ||
          retry.history_pressure_before != assembled.history_pressure_after_regional ||
          assembled.next_feedback_affected_nets == 0U) {
        return false;
      }
      if (std::any_of(retry.candidates.begin(), retry.candidates.end(),
                      [](const auto& candidate) {
                        return candidate.history_informed_global && candidate.feedback_affected_nets > 0U &&
                            candidate.regional_consulted && candidate.regional_straight_scene_evaluated;
                      })) {
        return true;
      }
    }
    return false;
  }();
  require(interleaved_straight_retry,
          "the penultimate assembled straight scene must publish exact regional failures and "
          "the subsequent global retry must consume that history and affected-net scope: " + [&] {
            std::string detail;
            for (const auto& epoch : assembled_feedback.feedback_probes) {
              detail += " pass=" + std::to_string(epoch.pass) +
                  " pressure=" + std::to_string(epoch.history_pressure_after_global) + "/" +
                  std::to_string(epoch.history_pressure_after_regional);
              for (const auto& candidate : epoch.candidates) {
                detail += " [overflow=" + std::to_string(candidate.global_sampled_overflow) +
                    "/" + std::to_string(candidate.global_corridor_overflow) +
                    " consulted=" + std::to_string(candidate.regional_consulted) +
                    " scene=" + std::to_string(candidate.regional_straight_scene_evaluated) +
                    " violations=" + std::to_string(candidate.regional_straight_scene_violations) + "]";
              }
            }
            return detail;
          }());
  auto single_portfolio_config = assembled_feedback_config;
  single_portfolio_config.retained_global_portfolios = 1U;
  single_portfolio_config.intermediate_regional_portfolios = 1U;
  const auto single_portfolio = flux::plan_routes_legacy(parallel, single_portfolio_config);
  require(single_portfolio.feedback_probes.size() == 3U &&
              std::all_of(single_portfolio.feedback_probes.begin(),
                          single_portfolio.feedback_probes.end(), [](const auto& epoch) {
                            return epoch.candidates.size() == 1U;
                          }) &&
              single_portfolio.feedback_probes.back().candidates.front().regional_straight_scene_evaluated,
          "forcing the physically informed replan into final assessment must respect a one-portfolio bound");
  auto straight_no_retry_config = assembled_feedback_config;
  straight_no_retry_config.max_routability_feedback_retries = 0U;
  const auto straight_no_retry = flux::plan_routes_legacy(parallel, straight_no_retry_config);
  require(straight_no_retry.feedback_probes.size() == 1U &&
              std::any_of(straight_no_retry.feedback_probes.front().candidates.begin(),
                          straight_no_retry.feedback_probes.front().candidates.end(),
                          [](const auto& candidate) {
                            return candidate.regional_straight_scene_evaluated;
                          }),
          "the zero-retry straight boundary must evaluate only the initial assembled scene");
  repeated_physics_config.global.max_iterations = 6U;
  const auto distributed_physics = flux::plan_routes_legacy(crossing, repeated_physics_config);
  require(!distributed_physics.complete() && distributed_physics.feedback_probes.size() == 3U &&
              distributed_physics.cumulative_global_iterations == 6U &&
              distributed_physics.feedback_probes[0].global_epoch_iterations == 1U &&
              distributed_physics.feedback_probes[1].global_epoch_iterations == 3U &&
              distributed_physics.feedback_probes[2].global_epoch_iterations == 2U,
          "failed seed certification must distribute, not discard or exceed, the remaining global budget");
  auto fresh_feedback_config = repeated_physics_config;
  fresh_feedback_config.regional.prioritize_frozen_scope_expansion = true;
  const auto fresh_feedback = flux::plan_routes_legacy(crossing, fresh_feedback_config);
  require(!fresh_feedback.complete() && fresh_feedback.feedback_probes.size() == 3U &&
              fresh_feedback.cumulative_global_iterations == 6U &&
              fresh_feedback.feedback_probes[1].candidates.size() == 1U &&
              fresh_feedback.feedback_probes[1].candidates.front().regional_consulted &&
              !fresh_feedback.feedback_probes[1].intermediate_regional_probe &&
              fresh_feedback.feedback_probes.back().retained_global_portfolios > 0U &&
              !fresh_feedback.feedback_probes.back().candidates.empty(),
          "fresh intermediate feedback must keep physical authority, global budget and final fallbacks");
  auto cone_config = broad_feedback_config.global;
  cone_config.atomic_complete_net_routing = true;
  cone_config.selective_congestion_reroute = true;
  cone_config.hard_capacity_start_iteration = 1U;
  cone_config.max_hard_capacity_blocker_reuse_invalidations_per_failed_net = 4U;
  const auto repaired_cone = flux::route_global_with_sampled_conflict_repair(crossing, cone_config, 2U);
  require(repaired_cone.stop_reason != flux::GlobalRouterStopReason::invalid_input &&
              !repaired_cone.paths.empty() && repaired_cone.post_negotiation_cone_repair_attempts > 0U,
          "sampled-cone repair must clear selective-only options and never replace routed copper with invalid input");
  auto oracle_first_config = broad_feedback_config;
  oracle_first_config.oracle_first_feedback = true;
  oracle_first_config.clearance_shadows_advisory = true;
  const auto oracle_first_plan = flux::plan_routes_legacy(crossing, oracle_first_config);
  require(oracle_first_plan.feedback_probes.size() >= 2U &&
              oracle_first_plan.feedback_probes.front().quantized_copper_oracle_violations > 0U &&
              std::all_of(oracle_first_plan.feedback_probes.front().candidates.begin(),
                          oracle_first_plan.feedback_probes.front().candidates.end(),
                          [](const auto& candidate) {
                            return !candidate.regional_consulted && !candidate.exact_promotion_attempted;
                          }) &&
              !oracle_first_plan.feedback_probes.back().intermediate_regional_probe &&
              std::any_of(oracle_first_plan.feedback_probes.back().candidates.begin(),
                          oracle_first_plan.feedback_probes.back().candidates.end(),
                          [](const auto& candidate) { return candidate.regional_consulted; }),
          "exact oracle failures must bypass intermediate regional work but not final realization");
  require(oracle_first_plan.regional_evaluated &&
              oracle_first_plan.regional.diagnostic.find("deferred") == std::string::npos,
          "an unevaluated regional checkpoint must not displace the final physical assessment");
  const auto broad_feedback_candidate = [&]() -> const flux::RoutingPlanFeedbackCandidate* {
    for (const auto& probe : broad_feedback_plan.feedback_probes) {
      const auto found =
          std::find_if(probe.candidates.begin(), probe.candidates.end(),
                       [](const auto& candidate) { return candidate.history_informed_global; });
      if (found != probe.candidates.end()) {
        return &*found;
      }
    }
    return nullptr;
  }();
  require(broad_feedback_candidate != nullptr &&
              broad_feedback_candidate->feedback_fell_back_to_full_global &&
              !broad_feedback_candidate->incremental_global &&
              broad_feedback_candidate->feedback_affected_nets >= 2U &&
              broad_feedback_plan.global_feedback_epochs ==
                  broad_feedback_plan.feedback_probes.size() &&
              broad_feedback_plan.cumulative_global_iterations > 0U &&
              broad_feedback_plan.retained_global_portfolio_peak > 0U &&
              broad_feedback_plan.retained_global_portfolios_evaluated > 0U &&
              broad_feedback_plan.intermediate_regional_probes > 0U &&
              broad_feedback_plan.provisional_exact_promotions_attempted <=
                  broad_feedback_plan.intermediate_regional_probes &&
              std::all_of(
                  broad_feedback_plan.feedback_probes.begin(),
                  broad_feedback_plan.feedback_probes.end(), [](const auto& probe) {
                    return !probe.intermediate_regional_probe ||
                           std::all_of(probe.candidates.begin(), probe.candidates.end(),
                                       [](const auto& candidate) {
                                         return (candidate.global_sampled_overflow == 0U &&
                                                 candidate.global_corridor_overflow == 0U &&
                                                 candidate.global_unaccepted_lane_transitions ==
                                                     0U) ||
                                                (!candidate.regional_consulted &&
                                                 !candidate.exact_promotion_attempted &&
                                                 !candidate.regional_spline_realization_enabled &&
                                                 candidate.regional_spline_candidates_evaluated ==
                                                     0U);
                                       });
                  }) &&
              std::all_of(
                  broad_feedback_plan.feedback_probes.begin(),
                  broad_feedback_plan.feedback_probes.end(),
                  [](const auto& probe) {
                    const auto cheap_probe =
                        !probe.intermediate_regional_probe ||
                        std::all_of(
                            probe.candidates.begin(), probe.candidates.end(),
                            [](const auto& candidate) {
                              return candidate.exact_promotion_attempted
                                         ? candidate.regional_spline_realization_enabled
                                         : !candidate.regional_spline_realization_enabled &&
                                               candidate.regional_spline_candidates_evaluated == 0U;
                            });
                    return probe.global_epoch_iterations > 0U &&
                           probe.cumulative_global_iterations > 0U &&
                           probe.retained_global_portfolios > 0U && cheap_probe;
                  }),
          "a feedback cone which covers essentially every net must re-run global allocation "
          "rather than preserving a token subset of incompatible routes, and short global "
          "epochs must retain observable topology breadth while negotiated overflow returns "
          "directly to PathFinder/rip-up instead of entering regional PH");

  flux::RoutabilityKnowledge typed_history;
  const flux::RoutabilityResourceReference shared_resource{
      "same-cell", LayerId{"F.Cu"}, {mm(40.0, 40.0), mm(41.0, 41.0)}, {NetId{"net-1"}}};
  flux::accumulate_routability_knowledge(typed_history,
                                         {{flux::RoutabilityFailureKind::packing_capacity_exceeded,
                                           flux::RoutabilityFailureStage::regional_discrete,
                                           shared_resource,
                                           2U,
                                           {},
                                           std::nullopt,
                                           std::nullopt,
                                           flux::RoutabilityRelaxation::change_lane_order,
                                           "lane",
                                           false},
                                          {flux::RoutabilityFailureKind::packing_capacity_exceeded,
                                           flux::RoutabilityFailureStage::regional_discrete,
                                           shared_resource,
                                           3U,
                                           {},
                                           std::nullopt,
                                           std::nullopt,
                                           flux::RoutabilityRelaxation::change_corridor,
                                           "corridor",
                                           false}});
  require(typed_history.history.size() == 2U &&
              typed_history.history.front().suggested_relaxation !=
                  typed_history.history.back().suggested_relaxation,
          "routability history must not collapse lane-order and corridor-change failures into "
          "one actionless scalar pressure entry");

  flux::RegionalRouteResult search_only_regional;
  search_only_regional.windows.push_back(
      {0U, {mm(40.0, 40.0), mm(41.0, 41.0)}, {LayerId{"F.Cu"}}, {}, {}, {}, {}, {},
       {NetId{"net-1"}}});
  flux::RegionalBundleAssignment search_only_assignment;
  search_only_assignment.window_id = 0U;
  search_only_assignment.stop_reason = flux::RegionalBundleStopReason::state_budget_exhausted;
  search_only_assignment.packing_rejected_states = 73U;
  search_only_assignment.access_assignment_exhaustions = 9U;
  search_only_assignment.partial_access_blockers = {
      {flux::RegionalAccessBlockerKind::via_pair, {}, {}, {}, 17U},
      {flux::RegionalAccessBlockerKind::terminal_via, {}, {}, {}, 19U}};
  search_only_regional.bundle_assignments.push_back(search_only_assignment);
  const auto search_only_diagnoses = flux::diagnose_regional_routability(search_only_regional);
  flux::RoutabilityKnowledge search_only_history;
  flux::accumulate_routability_knowledge(search_only_history, search_only_diagnoses);
  require(search_only_diagnoses.size() == 3U &&
              std::all_of(search_only_diagnoses.begin(), search_only_diagnoses.end(),
                          [](const auto& diagnosis) { return !diagnosis.contributes_to_history; }) &&
              search_only_history.history.empty(),
          "bounded packing and broad access-search observations must remain diagnostic evidence "
          "without inventing physical routability history");
  auto zero_rejected_regional = search_only_regional;
  zero_rejected_regional.bundle_assignments.front().partial_access_blockers.clear();
  zero_rejected_regional.bundle_assignments.front().packing_rejected_states = 0U;
  const auto zero_rejected_diagnoses =
      flux::diagnose_regional_routability(zero_rejected_regional);
  auto no_fitting_regional = zero_rejected_regional;
  no_fitting_regional.bundle_assignments.front().stop_reason =
      flux::RegionalBundleStopReason::no_fitting_lane_order;
  no_fitting_regional.bundle_assignments.front().packing_rejected_states = 997U;
  const auto no_fitting_diagnoses = flux::diagnose_regional_routability(no_fitting_regional);
  flux::RoutabilityKnowledge counter_history;
  flux::accumulate_routability_knowledge(counter_history, zero_rejected_diagnoses);
  flux::accumulate_routability_knowledge(counter_history, no_fitting_diagnoses);
  require(zero_rejected_diagnoses.size() == 1U &&
              zero_rejected_diagnoses.front().severity == 1U &&
              !zero_rejected_diagnoses.front().contributes_to_history &&
              no_fitting_diagnoses.size() == 1U &&
              no_fitting_diagnoses.front().severity == 997U &&
              !no_fitting_diagnoses.front().contributes_to_history &&
              counter_history.history.empty(),
          "zero or large rejected-state counters must remain telemetry severity, never physical history");
  auto physical_regional = search_only_regional;
  physical_regional.local_routability_diagnoses.push_back(
      {flux::RoutabilityFailureKind::packing_capacity_exceeded,
       flux::RoutabilityFailureStage::regional_discrete,
       {"", LayerId{"F.Cu"}, {mm(40.0, 40.0), mm(41.0, 41.0)}, {NetId{"net-1"}}},
       3U, Nanometres::from_mm(0.1), {}, {}, flux::RoutabilityRelaxation::change_corridor,
       "exact regional copper overlap"});
  const auto physical_diagnoses = flux::diagnose_regional_routability(physical_regional);
  flux::RoutabilityKnowledge physical_history;
  flux::accumulate_routability_knowledge(physical_history, physical_diagnoses);
  require(std::any_of(physical_diagnoses.begin(), physical_diagnoses.end(),
                      [](const auto& diagnosis) { return diagnosis.contributes_to_history; }) &&
              physical_history.history.size() == 1U && physical_history.pressure() == 3U,
          "localized physical regional evidence must continue to feed routability history");

  Board conflict_chain;
  conflict_chain.source_revision = "regional-conflict-chain";
  conflict_chain.outline = rectangle(0.0, 0.0, 100.0, 40.0);
  conflict_chain.layers = {{LayerId{"F.Cu"}, "F.Cu", LayerKind::copper, 0}};
  for (std::size_t index = 1U; index <= 4U; ++index) {
    conflict_chain.nets.push_back(Net{NetId{"chain-" + std::to_string(index)}, "CHAIN",
                                      static_cast<std::uint32_t>(5U - index)});
  }
  flux::GlobalRouteResult chain_global;
  for (std::size_t index = 1U; index <= 4U; ++index) {
    const auto y = 10.0 + static_cast<double>(index) * 0.1;
    chain_global.paths.push_back(
        flux::GlobalRoutePath{NetId{"chain-" + std::to_string(index)},
                              0U,
                              {{mm(5.0, y), LayerId{"F.Cu"}}, {mm(95.0, y), LayerId{"F.Cu"}}}});
  }
  const auto add_shadow = [&chain_global](std::size_t first_net, std::size_t second_net,
                                          double first_x, double second_x) {
    const auto first_y = 10.0 + static_cast<double>(first_net) * 0.1;
    const auto second_y = 10.0 + static_cast<double>(second_net) * 0.1;
    chain_global.clearance_shadow_conflicts.push_back(flux::RoutingClearanceShadowConflict{
        flux::RoutingClearanceShadowResourceKind::planar_edge,
        {mm(first_x, first_y), LayerId{"F.Cu"}},
        {mm(second_x, first_y), LayerId{"F.Cu"}},
        NetId{"chain-" + std::to_string(first_net)},
        NetId{"chain-" + std::to_string(second_net)},
        Nanometres::from_mm(0.1),
        flux::GlobalRoutePoint{mm(first_x, second_y), LayerId{"F.Cu"}},
        flux::GlobalRoutePoint{mm(second_x, second_y), LayerId{"F.Cu"}}});
  };
  add_shadow(1U, 2U, 10.0, 15.0);
  add_shadow(1U, 2U, 70.0, 75.0);
  add_shadow(2U, 3U, 30.0, 35.0);
  add_shadow(3U, 4U, 50.0, 55.0);
  flux::RegionalRouterConfig chain_config;
  // This fixture isolates spatial conflict-graph decomposition; terminal topology expansion is
  // orthogonal and would spend production continuous-routing budgets on every synthetic block.
  chain_config.enable_terminal_portal_templates = false;
  chain_config.max_same_layer_detour_checks = 0U;
  chain_config.enable_spline_realization = false;
  const auto chain_regional =
      flux::analyze_regional_conflicts(conflict_chain, chain_global, chain_config);
  require(chain_regional.source_clearance_shadow_conflicts == 4U &&
              chain_regional.conflict_edges.size() == 4U &&
              chain_regional.components.size() == 1U &&
              chain_regional.biconnected_blocks.size() == 3U &&
              chain_regional.articulation_fragments.size() == 2U &&
              !chain_regional.windows.empty() && chain_regional.windows.size() <= 3U &&
              std::all_of(chain_regional.windows.begin(), chain_regional.windows.end(),
                          [&](const auto& window) {
                            return !window.conflict_edges.empty() && !window.portals.empty() &&
                                   window.fragments.size() <
                                       chain_regional.components.front().fragments.size();
                          }),
          "a weak net chain must retain distant same-owner conflicts while decomposing into "
          "articulation-bounded physical windows instead of one whole-net repair group: " +
              std::to_string(chain_regional.source_clearance_shadow_conflicts) + "/" +
              std::to_string(chain_regional.conflict_edges.size()) + "/" +
              std::to_string(chain_regional.components.size()) + "/" +
              std::to_string(chain_regional.biconnected_blocks.size()) + "/" +
              std::to_string(chain_regional.articulation_fragments.size()) + "/" +
              std::to_string(chain_regional.windows.size()));

  flux::RegionalRouteResult split_regional;
  split_regional.configured.max_conflict_fragments_per_window = 3U;
  split_regional.configured.max_nets_per_window = 3U;
  split_regional.configured.window_margin = Nanometres::from_mm(2.0);
  for (std::size_t index = 1U; index <= 4U; ++index) {
    const auto id =
        flux::RegionalRouteFragmentId{NetId{"chain-" + std::to_string(index)}, 0U, 0U, 0U};
    split_regional.fragments.push_back(flux::RegionalRouteFragment{
        id,
        LayerId{"F.Cu"},
        mm(1.0, 5.0 + static_cast<double>(index)),
        mm(99.0, 5.0 + static_cast<double>(index)),
        Nanometres::from_mm(0.2),
        Nanometres::from_mm(0.1),
        {mm(1.0, 5.0 + static_cast<double>(index)), mm(99.0, 5.0 + static_cast<double>(index))}});
  }
  const auto add_partition_edge = [&split_regional](std::size_t first, std::size_t second,
                                                    double x) {
    flux::RegionalConflictEdge edge;
    edge.id = split_regional.conflict_edges.size();
    edge.first = split_regional.fragments[first].id;
    edge.second = split_regional.fragments[second].id;
    edge.layer = LayerId{"F.Cu"};
    edge.conflict_bounds = {mm(x, 8.0), mm(x + 1.0, 12.0)};
    edge.peak_clearance_deficit = Nanometres::from_mm(0.1);
    edge.observed_overlap_length_nm = static_cast<std::uint64_t>(Nanometres::from_mm(1.0).value());
    edge.sample_count = 1U;
    edge.clearance_shadow_sample_count = 1U;
    split_regional.conflict_edges.push_back(std::move(edge));
  };
  add_partition_edge(0U, 1U, 10.0);
  add_partition_edge(1U, 2U, 12.0);
  add_partition_edge(2U, 3U, 70.0);
  add_partition_edge(3U, 0U, 72.0);
  flux::partition_regional_conflicts(conflict_chain, split_regional);
  std::size_t partitioned_edge_count = 0U;
  for (const auto& window : split_regional.windows) {
    partitioned_edge_count += window.conflict_edges.size();
  }
  require(split_regional.biconnected_blocks.size() == 1U &&
              split_regional.geometrically_split_windows == 1U &&
              split_regional.uncuttable_oversized_blocks == 0U &&
              split_regional.windows.size() == 2U && partitioned_edge_count == 4U &&
              std::all_of(split_regional.windows.begin(), split_regional.windows.end(),
                          [](const auto& window) {
                            return window.nets.size() == 3U && window.eligible() &&
                                   !window.portals.empty();
                          }),
          "a spatially separated biconnected cycle must split at a conflict-free geometric cut "
          "without dropping edges or exceeding the bounded three-net window");

  flux::RegionalRouteResult anchored_regional;
  for (std::size_t index = 0U; index < 2U; ++index) {
    const auto id =
        flux::RegionalRouteFragmentId{NetId{"anchor-" + std::to_string(index)}, 0U, 0U, 0U};
    anchored_regional.fragments.push_back(flux::RegionalRouteFragment{
        id,
        LayerId{"F.Cu"},
        mm(5.0, 10.0 + static_cast<double>(index)),
        mm(95.0, 10.0 + static_cast<double>(index)),
        Nanometres::from_mm(0.2),
        Nanometres::from_mm(0.1),
        {mm(5.0, 10.0 + static_cast<double>(index)), mm(95.0, 10.0 + static_cast<double>(index))}});
  }
  flux::RegionalConflictEdge displaced_edge;
  displaced_edge.id = 0U;
  displaced_edge.first = anchored_regional.fragments[0].id;
  displaced_edge.second = anchored_regional.fragments[1].id;
  displaced_edge.layer = LayerId{"F.Cu"};
  displaced_edge.conflict_bounds = {mm(45.0, 30.0), mm(55.0, 31.0)};
  displaced_edge.peak_clearance_deficit = Nanometres::from_mm(0.1);
  displaced_edge.observed_overlap_length_nm =
      static_cast<std::uint64_t>(Nanometres::from_mm(1.0).value());
  displaced_edge.sample_count = 1U;
  anchored_regional.conflict_edges.push_back(std::move(displaced_edge));
  flux::partition_regional_conflicts(conflict_chain, anchored_regional);
  require(anchored_regional.windows.size() == 1U && anchored_regional.windows.front().eligible() &&
              anchored_regional.window_fragment_anchor_expansions == 2U &&
              anchored_regional.peak_window_fragment_anchor_expansion > Nanometres::from_mm(10.0) &&
              anchored_regional.windows.front().portals.size() == 4U,
          "a conflict-derived window must add deterministic nearest-fragment anchors before "
          "portal clipping instead of rejecting attributed physical routes that lie outside the "
          "sample bounds");

  std::vector<flux::RegionalRouteFragment> lane_fragments;
  for (std::size_t index = 0U; index < 3U; ++index) {
    const auto id =
        flux::RegionalRouteFragmentId{NetId{"lane-" + std::to_string(index)}, 0U, 0U, 0U};
    lane_fragments.push_back(flux::RegionalRouteFragment{
        id, LayerId{"F.Cu"}, mm(0.0, 0.0), mm(10.0, 0.0),
        Nanometres::from_mm(index == 1U ? 0.3 : 0.2), Nanometres::from_mm(0.1),
        flux::geometry::AxisAlignedBounds{mm(0.0, 0.0), mm(10.0, 0.0)}});
  }
  std::vector<flux::RegionalRouteFragmentId> lane_order;
  std::transform(lane_fragments.begin(), lane_fragments.end(), std::back_inserter(lane_order),
                 [](const auto& fragment) { return fragment.id; });
  const auto fitting_lanes =
      flux::pack_regional_cross_section(lane_fragments, lane_order, LayerId{"F.Cu"}, mm(0.0, 0.0),
                                        mm(0.0, 1.1), Nanometres::from_mm(1.1));
  const auto blocked_lanes =
      flux::pack_regional_cross_section(lane_fragments, lane_order, LayerId{"F.Cu"}, mm(0.0, 0.0),
                                        mm(0.0, 1.09), Nanometres::from_mm(1.09));
  const auto reversed_walls =
      flux::pack_regional_cross_section(lane_fragments, lane_order, LayerId{"F.Cu"}, mm(0.0, 1.1),
                                        mm(0.0, 0.0), Nanometres::from_mm(1.1));
  require(fitting_lanes.physically_fits && fitting_lanes.placements.size() == 3U &&
              fitting_lanes.packed_width == Nanometres::from_mm(1.1) &&
              !blocked_lanes.physically_fits && reversed_walls == fitting_lanes,
          "regional corridor packing must use heterogeneous copper widths and adjacent physical "
          "clearances rather than a scalar lane count, and reversed route direction must not "
          "reverse the physical lane axis");

  // Stable route IDs deliberately run opposite to their physical bottom-to-top positions. The
  // joint allocator must preserve the incoming geometry rather than treating the ID ordering as
  // a lane topology decision.
  auto geometric_lane_fragments = lane_fragments;
  for (std::size_t index = 0U; index < geometric_lane_fragments.size(); ++index) {
    const auto y = 2.0 - static_cast<double>(index);
    geometric_lane_fragments[index].start = mm(0.0, y);
    geometric_lane_fragments[index].end = mm(10.0, y);
  }
  const auto geometric_lane_order = flux::geometric_regional_cross_section_order(
      geometric_lane_fragments, lane_order, mm(5.0, 0.0), mm(5.0, 2.0));
  const auto reversed_geometric_lane_order = flux::geometric_regional_cross_section_order(
      geometric_lane_fragments, lane_order, mm(5.0, 2.0), mm(5.0, 0.0));
  auto reversed_lane_enumeration = lane_order;
  std::reverse(reversed_lane_enumeration.begin(), reversed_lane_enumeration.end());
  const auto reordered_input_lane_order = flux::geometric_regional_cross_section_order(
      geometric_lane_fragments, reversed_lane_enumeration, mm(5.0, 0.0), mm(5.0, 2.0));
  require(geometric_lane_order == std::vector<flux::RegionalRouteFragmentId>{lane_order[2],
                                                                             lane_order[1],
                                                                             lane_order[0]} &&
              reversed_geometric_lane_order == geometric_lane_order &&
              reordered_input_lane_order == geometric_lane_order,
          "regional lane references must follow physical cross-section intersections, remain "
          "invariant to wall/input enumeration, and use stable IDs only as coincident ties");

  flux::GlobalRouteResult corridor_membership_global;
  flux::GlobalRoutePath corridor_membership_path;
  corridor_membership_path.net = NetId{"diagonal-a"};
  corridor_membership_path.points = {{mm(2.0, 2.0), LayerId{"F.Cu"}},
                                     {mm(8.0, 2.0), LayerId{"F.Cu"}},
                                     {mm(8.0, 20.0), LayerId{"F.Cu"}}};
  flux::RoutingGeometricCorridorRun local_corridor;
  local_corridor.layer = LayerId{"F.Cu"};
  local_corridor.topology_cells = {rectangle(1.0, 1.0, 7.0, 3.0)};
  local_corridor.lane_centerline = {mm(2.0, 2.0), mm(7.0, 2.0)};
  local_corridor.required_envelope_width = Nanometres::from_mm(0.4);
  local_corridor.minimum_physical_cross_section_width = Nanometres::from_mm(2.0);
  local_corridor.lane_capacity = 3U;
  local_corridor.selected_path_preserves_topology = true;
  corridor_membership_path.geometric_corridors.push_back(std::move(local_corridor));
  corridor_membership_global.paths.push_back(std::move(corridor_membership_path));
  const auto local_corridor_regional = flux::analyze_regional_conflicts(
      crossing, corridor_membership_global, terminal_portal_config);
  require(local_corridor_regional.fragments.size() == 2U &&
              local_corridor_regional.channels.size() == 1U &&
              local_corridor_regional.channels.front().occupants ==
                  std::vector<flux::RegionalRouteFragmentId>{
                      local_corridor_regional.fragments.front().id},
          "a retained geometric corridor must own only route fragments that physically traverse "
          "its cells, not every same-layer fragment of the complete connection: fragments=" +
              std::to_string(local_corridor_regional.fragments.size()) + ", channels=" +
              std::to_string(local_corridor_regional.channels.size()) + ", occupants=" +
              std::to_string(local_corridor_regional.channels.empty()
                                 ? 0U
                                 : local_corridor_regional.channels.front().occupants.size()));

  // A broad topology cell can overlap a conflicting branch even though its
  // only capacity sample lies at the other end of the board. Cell membership
  // must not suppress generation of a local physical cross section.
  auto remote_sample_global = crossed;
  for (auto& path : remote_sample_global.paths) {
    flux::RoutingGeometricCorridorRun corridor;
    corridor.layer = LayerId{"F.Cu"};
    corridor.topology_cells = {rectangle(1.0, 1.0, 29.0, 29.0)};
    corridor.lane_centerline = {mm(2.0, 2.0), mm(8.0, 2.0)};
    corridor.minimum_physical_cross_section_width = Nanometres::from_mm(1.0);
    corridor.selected_path_preserves_topology = true;
    path.geometric_corridors = {corridor};
  }
  auto local_sample_config = terminal_portal_config;
  local_sample_config.local_channel_occupancy = true;
  const auto local_sample_result = flux::analyze_regional_conflicts(
      crossing, remote_sample_global, local_sample_config);
  require(!local_sample_result.windows.empty(), "local-section fixture must retain its conflict");
  for (const auto& window : local_sample_result.windows) {
    for (const auto& id : window.fragments) {
      const auto f = std::find_if(local_sample_result.fragments.begin(), local_sample_result.fragments.end(),
                                  [&](const auto& fragment) { return fragment.id == id; });
      require(f != local_sample_result.fragments.end(), "every governed route must be retained");
      require(std::any_of(local_sample_result.channels.begin(), local_sample_result.channels.end(),
                           [&](const auto& channel) {
        return std::find(channel.occupants.begin(), channel.occupants.end(), id) != channel.occupants.end() &&
               std::any_of(channel.cross_sections.begin(), channel.cross_sections.end(), [&](const auto& section) {
          return flux::geometry::segment_distance_nm(f->start, f->end, section.first_wall,
                                                     section.second_wall) <=
                 static_cast<long double>(f->width.value()) / 2.0L + f->clearance.value();
        });
      }), "every governed fragment must recover a local sampled resource, not merely cell membership");
    }
  }

  auto bounded_crossed = crossed;
  for (auto& path : bounded_crossed.paths) {
    flux::RoutingGeometricCorridorRun corridor;
    corridor.layer = LayerId{"F.Cu"};
    corridor.topology_cells = {rectangle(5.0, 5.0, 25.0, 25.0)};
    corridor.lane_centerline.reserve(path.points.size());
    for (const auto& point : path.points) {
      corridor.lane_centerline.push_back(point.position);
    }
    corridor.required_envelope_width = Nanometres::from_mm(0.65);
    corridor.minimum_physical_cross_section_width = Nanometres::from_mm(2.0);
    corridor.lane_capacity = 3U;
    corridor.selected_path_preserves_topology = true;
    path.geometric_corridors.push_back(std::move(corridor));
  }
  for (auto& group : bounded_crossed.residual_conflict_groups) {
    group.all_nets_have_preserved_geometric_corridors = true;
    group.peak_geometric_lane_capacity = 3U;
  }
  flux::DetailedRouterConfig bounded_crossed_config;
  bounded_crossed_config.pull_tight_45_degrees = false;
  bounded_crossed_config.continuous_refinement = false;
  const auto straight_bounded_detail =
      flux::materialize_detailed_route(crossing, bounded_crossed, bounded_crossed_config);
  require(!straight_bounded_detail.complete() && straight_bounded_detail.added_track_arcs == 0U &&
              straight_bounded_detail.residual_joint_candidate_states == 0U &&
              straight_bounded_detail.attempted_quintic_ph_runs == 0U &&
              straight_bounded_detail.copper_drc.violation_count() > 0U,
          "straight residual repair must report crossing copper without enabling PH implicitly");
  bounded_crossed_config.circular_arc_refinement = true;
  const auto direct_bounded_detail =
      flux::materialize_detailed_route(crossing, bounded_crossed, bounded_crossed_config);
  require(
      direct_bounded_detail.residual_conflict_handoff_attempted &&
          direct_bounded_detail.residual_conflict_groups_attempted == 1U &&
          direct_bounded_detail.residual_conflict_nets_attempted == 2U &&
          direct_bounded_detail.residual_joint_candidate_states ==
              bounded_crossed_config.max_residual_joint_candidate_states &&
          !direct_bounded_detail.complete(),
      "a topology-complete bounded residual checkpoint must be handed to the joint "
      "quintic PH curve/PNS transaction after ordinary detailed copper fails, while retaining an "
      "observable exact accept/reject outcome: handoff=" +
          std::to_string(direct_bounded_detail.residual_conflict_handoff_attempted) +
          ", groups=" + std::to_string(direct_bounded_detail.residual_conflict_groups_attempted) +
          ", nets=" + std::to_string(direct_bounded_detail.residual_conflict_nets_attempted) +
          ", states=" + std::to_string(direct_bounded_detail.residual_joint_candidate_states) +
          ", rejected=" +
          std::to_string(direct_bounded_detail.residual_joint_exact_transactions_rejected) +
          ", accepted=" +
          std::to_string(direct_bounded_detail.residual_joint_transaction_accepted) +
          ", diagnostic=" + direct_bounded_detail.diagnostic);
  flux::RoutingPlanResult crossed_plan;
  crossed_plan.coarse_global = bounded_crossed;
  crossed_plan.regional =
      flux::analyze_regional_conflicts(crossing, bounded_crossed, terminal_portal_config);
  crossed_plan.paths = crossed_plan.regional.spline_candidate_available
                           ? crossed_plan.regional.spline_candidate_paths
                           : bounded_crossed.paths;
  crossed_plan.stop_reason = flux::RoutingPlanStopReason::regional_conflicts;
  const auto crossed_plan_detail =
      flux::materialize_detailed_route(crossing, crossed_plan, bounded_crossed_config);
  require(!crossed_plan_detail.complete() ||
              (crossed_plan_detail.candidate && crossed_plan_detail.validation.ok() &&
               crossed_plan_detail.connectivity.open_connections == 0U &&
               crossed_plan_detail.copper_drc.ok()),
          "a regional quintic PH curve proposal must remain one atomic exact transaction; a "
          "newly solvable crossing may be accepted only as connected DRC-clean copper");

  auto stacked_via_board = crossing;
  stacked_via_board.layers = {{LayerId{"F.Cu"}, "F.Cu", LayerKind::copper, 0},
                              {LayerId{"In1.Cu"}, "In1.Cu", LayerKind::copper, 1},
                              {LayerId{"B.Cu"}, "B.Cu", LayerKind::copper, 2}};
  flux::GlobalRouteResult stacked_via_global;
  stacked_via_global.paths = {flux::GlobalRoutePath{NetId{"diagonal-a"},
                                                    0U,
                                                    {{mm(10.0, 10.0), LayerId{"F.Cu"}},
                                                     {mm(10.0, 10.0), LayerId{"In1.Cu"}},
                                                     {mm(10.0, 10.0), LayerId{"B.Cu"}},
                                                     {mm(20.0, 20.0), LayerId{"B.Cu"}}}},
                              flux::GlobalRoutePath{NetId{"diagonal-b"},
                                                    0U,
                                                    {{mm(20.0, 10.0), LayerId{"F.Cu"}},
                                                     {mm(20.0, 10.0), LayerId{"In1.Cu"}},
                                                     {mm(20.0, 10.0), LayerId{"B.Cu"}},
                                                     {mm(10.0, 20.0), LayerId{"B.Cu"}}}}};
  for (auto& path : stacked_via_global.paths) {
    flux::RoutingGeometricCorridorRun corridor;
    corridor.layer = LayerId{"B.Cu"};
    corridor.topology_cells = {rectangle(5.0, 5.0, 25.0, 25.0)};
    corridor.lane_centerline = {path.points[path.points.size() - 2U].position,
                                path.points.back().position};
    corridor.required_envelope_width = Nanometres::from_mm(0.65);
    corridor.minimum_physical_cross_section_width = Nanometres::from_mm(2.0);
    corridor.lane_capacity = 3U;
    corridor.selected_path_preserves_topology = true;
    path.geometric_corridors.push_back(std::move(corridor));
  }
  stacked_via_global.clearance_shadow_conflicts.push_back(flux::RoutingClearanceShadowConflict{
      flux::RoutingClearanceShadowResourceKind::planar_edge,
      {mm(10.0, 10.0), LayerId{"B.Cu"}},
      {mm(20.0, 20.0), LayerId{"B.Cu"}},
      NetId{"diagonal-a"},
      NetId{"diagonal-b"},
      Nanometres::from_mm(0.1),
      flux::GlobalRoutePoint{mm(20.0, 10.0), LayerId{"B.Cu"}},
      flux::GlobalRoutePoint{mm(10.0, 20.0), LayerId{"B.Cu"}}});
  flux::RegionalRouterConfig stacked_via_config;
  stacked_via_config.max_alternative_template_checks = 1U;
  stacked_via_config.max_same_layer_detour_checks = 0U;
  stacked_via_config.enable_terminal_portal_templates = false;
  stacked_via_config.enable_spline_realization = false;
  const auto stacked_via_regional =
      flux::analyze_regional_conflicts(stacked_via_board, stacked_via_global, stacked_via_config);
  const auto selected_stacked_via_regions = static_cast<std::size_t>(
      std::count_if(stacked_via_regional.via_region_evidence.begin(),
                    stacked_via_regional.via_region_evidence.end(),
                    [](const auto& evidence) { return evidence.selected_global_candidates > 0U; }));
  require(selected_stacked_via_regions == 2U &&
              std::count_if(stacked_via_regional.via_choices.begin(),
                            stacked_via_regional.via_choices.end(),
                            [](const auto& choice) {
                              return choice.selected_global_site && choice.span.size() == 3U;
                            }) == 2,
          "each maximal same-coordinate layer stack must be one physical regional via variable "
          "with its complete span, matching detailed materialization");
  require(std::all_of(stacked_via_regional.fragments.begin(), stacked_via_regional.fragments.end(),
                      [](const auto& fragment) { return fragment.begins_at_terminal; }),
          "a planar run after a stacked via at the connection endpoint must retain terminal "
          "provenance so selected-topology relocation requires explicit pad fanout geometry");

  auto multilayer_crossing = crossing;
  multilayer_crossing.layers.push_back({LayerId{"B.Cu"}, "B.Cu", LayerKind::copper, 1});
  multilayer_crossing.nets.push_back(Net{NetId{"via-site-blocker"}, "VIA_SITE_BLOCKER", 0});
  for (std::size_t index = 0U; index < crossing.footprints.size(); ++index) {
    const auto center = crossing.footprints[index].position;
    multilayer_crossing.tracks.push_back(flux::TrackSegment{
        ObjectId{"via-site-blocker-" + std::to_string(index)}, NetId{"via-site-blocker"},
        LayerId{"B.Cu"},
        Point{Nanometres{center.x.value() - Nanometres::from_mm(0.05).value()}, center.y},
        Point{Nanometres{center.x.value() + Nanometres::from_mm(0.05).value()}, center.y},
        Nanometres::from_mm(0.2), false});
  }
  auto multilayer_global = bounded_crossed;
  flux::RegionalRouterConfig multilayer_regional_config;
  multilayer_regional_config.max_same_layer_detour_checks = 0U;
  multilayer_regional_config.navmesh_cell_pitch = Nanometres::from_mm(1.0);
  multilayer_regional_config.max_navmesh_cells_per_template = 20'000U;
  multilayer_regional_config.max_navmesh_expansions_per_template = 20'000U;
  auto multilayer_plan = flux::RoutingPlanResult{};
  multilayer_plan.coarse_global = multilayer_global;
  multilayer_plan.regional = flux::analyze_regional_conflicts(
      multilayer_crossing, multilayer_global, multilayer_regional_config);
  std::size_t isolated_alternative_channels = 0U;
  for (const auto& assignment : multilayer_plan.regional.bundle_assignments) {
    for (const auto& route_template : assignment.templates) {
      for (const auto channel_id : route_template.channel_sequence) {
        const auto channel = std::find_if(multilayer_plan.regional.channels.begin(), multilayer_plan.regional.channels.end(),
            [&](const auto& candidate) { return candidate.id == channel_id; });
        require(channel != multilayer_plan.regional.channels.end(), "every template channel must exist");
        if (route_template.selected_global_topology) {
          require(!channel->alternative_template_only,
                  "generating unselected detours must not add their gates or free space to the "
                  "retained global template");
        } else {
          isolated_alternative_channels += channel->alternative_template_only ? 1U : 0U;
        }
      }
    }
  }
  require(isolated_alternative_channels > 0U,
          "the crossing fixture must exercise separately selectable alternative channels");
  multilayer_plan.paths = multilayer_plan.regional.spline_candidate_available
                              ? multilayer_plan.regional.spline_candidate_paths
                              : multilayer_global.paths;
  multilayer_plan.stop_reason = flux::RoutingPlanStopReason::regional_conflicts;
  auto shared_access_budget_config = multilayer_regional_config;
  shared_access_budget_config.max_access_assignment_states = 1U;
  shared_access_budget_config.enable_spline_realization = false;
  const auto shared_access_budget_regional = flux::analyze_regional_conflicts(
      multilayer_crossing, multilayer_global, shared_access_budget_config);
  const auto shared_access_budget_is_bounded =
      !shared_access_budget_regional.bundle_assignments.empty() &&
      std::all_of(shared_access_budget_regional.bundle_assignments.begin(),
                  shared_access_budget_regional.bundle_assignments.end(),
                  [&](const auto& assignment) {
                    // Each cached pair and complete leaf has an independent bounded query.
                    // Exhausting one query cannot consume the allowance of another topology.
                    return assignment.partial_access_assignment_states <=
                           shared_access_budget_config.max_access_assignment_states *
                               (assignment.partial_pairwise_access_checks +
                                assignment.bundle_search_states);
                  });
  const auto selected_alternative = std::any_of(
      multilayer_plan.regional.bundle_assignments.begin(),
      multilayer_plan.regional.bundle_assignments.end(), [&](const auto& assignment) {
        return std::any_of(assignment.selected_templates.begin(),
                           assignment.selected_templates.end(), [&](const auto template_id) {
                             return template_id < assignment.templates.size() &&
                                    !assignment.templates[template_id].selected_global_topology;
                           });
      });
  const auto selected_relocated_via = std::any_of(
      multilayer_plan.regional.bundle_assignments.begin(),
      multilayer_plan.regional.bundle_assignments.end(), [&](const auto& assignment) {
        return std::any_of(
            assignment.selected_templates.begin(), assignment.selected_templates.end(),
            [&](const auto template_id) {
              if (template_id >= assignment.templates.size()) {
                return false;
              }
              const auto& route_template = assignment.templates[template_id];
              if (route_template.selected_global_topology) {
                return false;
              }
              const auto fragment = std::find_if(
                  multilayer_plan.regional.fragments.begin(),
                  multilayer_plan.regional.fragments.end(),
                  [&](const auto& candidate) { return candidate.id == route_template.fragment; });
              if (fragment == multilayer_plan.regional.fragments.end()) {
                return false;
              }
              return std::any_of(route_template.via_choices.begin(),
                                 route_template.via_choices.end(), [&](const auto choice_id) {
                                   const auto choice =
                                       std::find_if(multilayer_plan.regional.via_choices.begin(),
                                                    multilayer_plan.regional.via_choices.end(),
                                                    [&](const auto& candidate) {
                                                      return candidate.id == choice_id;
                                                    });
                                   return choice != multilayer_plan.regional.via_choices.end() &&
                                          choice->site != fragment->start &&
                                          choice->site != fragment->end;
                                 });
            });
      });
  const auto multilayer_detail = flux::materialize_detailed_route(
      multilayer_crossing, multilayer_plan, bounded_crossed_config);
  require(multilayer_plan.regional.candidate_pipeline_stats.layered_navmesh_graph_nodes > 0U &&
              multilayer_plan.regional.candidate_pipeline_stats.layered_navmesh_search_expansions > 0U &&
              multilayer_plan.regional.candidate_pipeline_stats.layered_navmesh_selected_proposals > 0U,
          "regional layer alternatives must consume a certified multilayer cell/via corridor: nodes=" +
              std::to_string(multilayer_plan.regional.candidate_pipeline_stats.layered_navmesh_graph_nodes) +
              " transitions=" + std::to_string(multilayer_plan.regional.candidate_pipeline_stats.layered_navmesh_certified_transitions) +
              " expansions=" + std::to_string(multilayer_plan.regional.candidate_pipeline_stats.layered_navmesh_search_expansions) +
              " selected=" + std::to_string(multilayer_plan.regional.candidate_pipeline_stats.layered_navmesh_selected_proposals));
  const auto retained_via_choices =
      std::accumulate(multilayer_plan.regional.via_region_evidence.begin(),
                      multilayer_plan.regional.via_region_evidence.end(), std::size_t{0},
                      [](const auto total, const auto& evidence) {
                        return total + evidence.candidate_choices.size();
                      });
  const auto bounded_via_portfolio =
      !multilayer_plan.regional.via_region_evidence.empty() &&
      retained_via_choices == multilayer_plan.regional.via_choices.size() &&
      std::all_of(multilayer_plan.regional.via_region_evidence.begin(),
                  multilayer_plan.regional.via_region_evidence.end(),
                  [&](const auto& evidence) {
                    return !evidence.candidate_choices.empty() &&
                           evidence.candidate_choices.size() <=
                               multilayer_regional_config.max_via_sites_per_region;
                  }) &&
      std::any_of(multilayer_plan.regional.via_region_evidence.begin(),
                  multilayer_plan.regional.via_region_evidence.end(),
                  [](const auto& evidence) { return evidence.candidate_choices.size() > 1U; });
  const auto stable_via_assignment_order = std::all_of(
      multilayer_plan.regional.bundle_assignments.begin(),
      multilayer_plan.regional.bundle_assignments.end(), [](const auto& assignment) {
        const std::set<std::size_t> unique(assignment.via_assignment_region_order.begin(),
                                           assignment.via_assignment_region_order.end());
        return unique.size() == assignment.via_assignment_region_order.size();
      });
  const auto complete_two_via_plan =
      std::any_of(multilayer_plan.regional.multilayer_route_plans.begin(),
                  multilayer_plan.regional.multilayer_route_plans.end(), [](const auto& plan) {
                    return plan.via_transitions.size() >= 2U &&
                           plan.layer_runs.size() == plan.via_transitions.size() + 1U;
                  });
  const auto selected_multilayer_solution = selected_alternative && selected_relocated_via &&
                                            complete_two_via_plan &&
                                            multilayer_plan.regional.planned_via_transitions >= 2U;
  const auto selected_direct_ph_solution = !selected_alternative && !selected_relocated_via &&
                                           multilayer_plan.regional.spline_candidate_available &&
                                           multilayer_detail.complete();
  require(multilayer_regional_config.enable_terminal_portal_templates &&
              shared_access_budget_is_bounded &&
              multilayer_plan.regional.accepted_alternative_templates > 0U &&
              (selected_multilayer_solution || selected_direct_ph_solution) &&
              bounded_via_portfolio && stable_via_assignment_order &&
              multilayer_plan.regional.complete_via_dependent_routes ==
                  multilayer_plan.regional.multilayer_route_plans.size() &&
              multilayer_plan.regional.spline_candidate_available &&
              multilayer_detail.residual_conflict_handoff_attempted &&
              multilayer_detail.residual_conflict_groups_attempted > 0U &&
              multilayer_detail.residual_conflict_nets_attempted == 2U &&
              multilayer_detail.residual_joint_groups_optimized ==
                  multilayer_plan.regional.bundle_assignments.size() &&
              multilayer_detail.residual_joint_transaction_accepted &&
              multilayer_detail.residual_joint_exact_transactions_rejected == 0U &&
              std::all_of(multilayer_plan.regional.bundle_assignments.begin(),
                          multilayer_plan.regional.bundle_assignments.end(),
                          [](const auto& assignment) {
                            return std::all_of(
                                assignment.spline_realizations.begin(),
                                assignment.spline_realizations.end(), [](const auto& realization) {
                                  return realization.quantized_mutually_legal;
                                });
                          }) &&
              (selected_direct_ph_solution ||
               std::any_of(multilayer_plan.paths.begin(), multilayer_plan.paths.end(),
                           [](const auto& path) {
                             return std::adjacent_find(
                                        path.points.begin(), path.points.end(),
                                        [](const auto& first, const auto& second) {
                                          return first.position == second.position &&
                                                 first.layer != second.layer;
                                        }) != path.points.end();
                           })) &&
              multilayer_detail.complete() &&
              (selected_direct_ph_solution || multilayer_detail.added_vias >= 2U),
          "a bounded crossing window must accept attributed sampled overflow, retain exact-checked "
          "alternate layer/via states, and select either clean direct PH geometry or a relocated "
          "multilayer realization, "
          "realize it as quintic PH curve-derived copper with endpoint vias, and commit only after "
          "whole-board connectivity and DRC; alternatives=" +
              std::to_string(multilayer_plan.regional.accepted_alternative_templates) +
              " selected=" + std::to_string(selected_alternative) + " spline=" +
              std::to_string(multilayer_plan.regional.spline_candidate_available) +
              " relocated=" + std::to_string(selected_relocated_via) + " via_regions=" +
              std::to_string(multilayer_plan.regional.via_region_evidence.size()) +
              " retained_vias=" + std::to_string(retained_via_choices) + " detail=" +
              std::to_string(multilayer_detail.complete()) + " diagnostic=" +
              multilayer_detail.diagnostic + " bundles=" +
              std::to_string(multilayer_plan.regional.bundle_assignments.size()) +
              " bundle_stop=" +
              (multilayer_plan.regional.bundle_assignments.empty()
                   ? std::string{"none"}
                   : std::to_string(static_cast<int>(
                         multilayer_plan.regional.bundle_assignments.front().stop_reason))) +
              " realizations=" +
              (multilayer_plan.regional.bundle_assignments.empty()
                   ? std::string{"none"}
                   : std::to_string(multilayer_plan.regional.bundle_assignments.front()
                                        .spline_realizations.size())) +
              " realization_flags=" +
              (multilayer_plan.regional.bundle_assignments.empty()
                   ? std::string{"none"}
                   : [&]() {
                       std::string flags;
                       for (const auto& realization :
                            multilayer_plan.regional.bundle_assignments.front()
                                .spline_realizations) {
                         flags += std::to_string(realization.topology_legal) + "/" +
                                  std::to_string(realization.shape_legal) + "/" +
                                  std::to_string(realization.mutually_legal) + "/" +
                                  std::to_string(realization.quantized_mutually_legal) + "/" +
                                  std::to_string(realization.centerline.size()) + "/" +
                                  std::to_string(realization.shape_self_intersections) + "/" +
                                  std::to_string(realization.shape_cusps) + "/" +
                                  std::to_string(realization.shape_control_station_reversals) +
                                  "/" + std::to_string(realization.shape_station_reversals) + "/" +
                                  std::to_string(realization.shape_excess_turn_violations) + "/" +
                                  std::to_string(realization.shape_unintended_inflections) + "/" +
                                  std::to_string(realization.shape_sample_budget_exhausted) + "/" +
                                  std::to_string(static_cast<double>(
                                      realization.shape_total_turn_radians)) + "/" +
                                  std::to_string(static_cast<double>(
                                      realization.shape_anchor_turn_radians)) + "/" +
                                  std::to_string(static_cast<double>(
                                      realization.interpolation_parameter_exponent)) + "/" +
                                  std::to_string(static_cast<double>(
                                      realization.interpolation_tangent_scale)) + ";";
                       }
                       return flags;
                     }()) +
              " selected_residual=" +
              (multilayer_plan.regional.bundle_assignments.empty()
                   ? std::string{"none"}
                   : std::to_string(multilayer_plan.regional.bundle_assignments.front()
                                        .selected_residual_conflicts)));

  auto multi_transition_board = multilayer_crossing;
  for (auto& layer : multi_transition_board.layers) {
    if (layer.id == LayerId{"B.Cu"}) {
      layer.stack_order = 2U;
    }
  }
  multi_transition_board.layers.push_back({LayerId{"In1.Cu"}, "In1.Cu", LayerKind::copper, 1U});
  flux::GlobalRoutePath multi_transition_path;
  multi_transition_path.net = NetId{"diagonal-a"};
  multi_transition_path.connection_index = 7U;
  multi_transition_path.points = {
      {mm(2.0, 2.0), LayerId{"F.Cu"}},   {mm(4.0, 2.0), LayerId{"F.Cu"}},
      {mm(4.0, 2.0), LayerId{"In1.Cu"}}, {mm(8.0, 2.0), LayerId{"In1.Cu"}},
      {mm(8.0, 2.0), LayerId{"B.Cu"}},   {mm(12.0, 2.0), LayerId{"B.Cu"}},
      {mm(12.0, 2.0), LayerId{"F.Cu"}},  {mm(16.0, 2.0), LayerId{"F.Cu"}},
  };
  const auto multi_transition_plan =
      flux::plan_multilayer_route(multi_transition_board, multi_transition_path);
  require(multi_transition_plan && multi_transition_plan->via_transitions.size() == 3U &&
              multi_transition_plan->layer_runs.size() == 4U &&
              multi_transition_plan->via_transitions.back().span.size() == 3U,
          "multilayer planning must preserve an arbitrary sequence of distinct physical vias and "
          "planar layer runs rather than assuming exactly one via pair");

  if (selected_multilayer_solution) {
    auto stale_multilayer_plan = multilayer_plan;
    stale_multilayer_plan.stop_reason = flux::RoutingPlanStopReason::complete;
    stale_multilayer_plan.paths = stale_multilayer_plan.regional.spline_candidate_paths;
    stale_multilayer_plan.regional.multilayer_route_plans.clear();
    const auto stale_multilayer_detail = flux::materialize_detailed_route(
        multilayer_crossing, stale_multilayer_plan, bounded_crossed_config);
    require(
        !stale_multilayer_detail.complete() && !stale_multilayer_detail.candidate,
        "detailed routing must fail closed when selected multilayer planning evidence no longer "
        "matches the spline candidate path");
  }

  auto explicit_width_global = multilayer_global;
  explicit_width_global.configured_terminal_neck_down_width = Nanometres::from_mm(0.15);
  explicit_width_global.configured_max_terminal_neck_down_length = Nanometres::from_mm(30.0);
  auto& explicit_path = explicit_width_global.paths.front();
  explicit_path.segment_widths.assign(explicit_path.points.size() - 1U, Nanometres::from_mm(0.15));
  explicit_path.segment_width_kinds.assign(explicit_path.segment_widths.size(),
                                           flux::GlobalRouteSegmentWidthKind::terminal_neck_down);
  explicit_path.terminal_neck_down_segments = explicit_path.segment_widths.size();
  explicit_path.terminal_neck_down_length_nm = 0U;
  for (std::size_t edge = 1U; edge < explicit_path.points.size(); ++edge) {
    explicit_path.terminal_neck_down_length_nm +=
        static_cast<std::uint64_t>(std::ceil(flux::geometry::point_distance_nm(
            explicit_path.points[edge - 1U].position, explicit_path.points[edge].position)));
  }
  flux::RoutingPlanResult explicit_width_plan;
  explicit_width_plan.coarse_global = explicit_width_global;
  explicit_width_plan.regional = flux::analyze_regional_conflicts(
      multilayer_crossing, explicit_width_global, multilayer_regional_config);
  explicit_width_plan.paths = explicit_width_plan.regional.spline_candidate_available
                                  ? explicit_width_plan.regional.spline_candidate_paths
                                  : explicit_width_global.paths;
  explicit_width_plan.stop_reason = flux::RoutingPlanStopReason::regional_conflicts;
  const auto routed_explicit = std::find_if(
      explicit_width_plan.paths.begin(), explicit_width_plan.paths.end(), [&](const auto& path) {
        return path.net == explicit_path.net &&
               path.connection_index == explicit_path.connection_index;
      });
  const auto explicit_width_detail = flux::materialize_detailed_route(
      multilayer_crossing, explicit_width_plan, bounded_crossed_config);
  require(
      explicit_width_plan.regional.refittable_terminal_width_fragments > 0U &&
          explicit_width_plan.regional.fixed_resource_width_fragments == 0U &&
          std::all_of(explicit_width_plan.regional.windows.begin(),
                      explicit_width_plan.regional.windows.end(),
                      [](const auto& window) { return window.eligible(); }) &&
          explicit_width_plan.regional.spline_candidate_available &&
          routed_explicit != explicit_width_plan.paths.end() &&
          routed_explicit->segment_widths.size() + 1U == routed_explicit->points.size() &&
          routed_explicit->segment_width_kinds.size() == routed_explicit->segment_widths.size() &&
          routed_explicit->terminal_neck_down_segments ==
              static_cast<std::size_t>(
                  std::count(routed_explicit->segment_width_kinds.begin(),
                             routed_explicit->segment_width_kinds.end(),
                             flux::GlobalRouteSegmentWidthKind::terminal_neck_down)) &&
          routed_explicit->terminal_neck_down_length_nm <=
              static_cast<std::uint64_t>(
                  explicit_width_global.configured_max_terminal_neck_down_length.value()) &&
          explicit_width_detail.complete(),
      "regional spline replacement must preserve an explicit terminal width and its provenance, "
      "recompute the tessellated segment evidence, and pass the exact transaction: fragments=" +
          std::to_string(explicit_width_plan.regional.refittable_terminal_width_fragments) +
          " spline=" + std::to_string(explicit_width_plan.regional.spline_candidate_available) +
          " detail=" + std::to_string(explicit_width_detail.complete()) +
          " diagnostic=" + explicit_width_detail.diagnostic);

  GlobalRouterConfig early_spline_config;
  early_spline_config.grid_pitch = Nanometres::from_mm(5.0);
  early_spline_config.max_iterations = 1U;
  // The mandatory portfolio retains shortest, diversified, RRT, and hydraulic discovery. RRT is
  // disabled here but its prepared-NavMesh behavior has a separate assertion.
  early_spline_config.max_static_seed_candidates_per_connection = 4U;
  early_spline_config.use_static_seed_rrt_connect_alternatives = false;
  auto early_spline_board = two_pad_board();
  early_spline_board.footprints[1].position = mm(80.0, 60.0);
  early_spline_board.nets.push_back(Net{NetId{"early-pns-obstacle-net"}, "EARLY_PNS", 0});
  early_spline_board.tracks.push_back(flux::TrackSegment{
      ObjectId{"early-pns-obstacle"}, NetId{"early-pns-obstacle-net"}, LayerId{"F.Cu"},
      mm(30.0, 40.1), mm(70.0, 40.1), Nanometres::from_mm(0.25), false});
  flux::OffsetFreeSpaceRegion direct_navmesh_region;
  direct_navmesh_region.area = rectangle(0.5, 0.5, 99.5, 79.5);
  direct_navmesh_region.holes.push_back(rectangle(29.5, 39.5, 70.5, 40.7));
  const flux::NavMeshConfig direct_navmesh_config{Nanometres::from_mm(5.0),
                                                  100'000U,
                                                  100'000U,
                                                  {},
                                                  Nanometres{0},
                                                  1U,
                                                  Nanometres{0},
                                                  std::nullopt,
                                                  false};
  const auto direct_navmesh = flux::build_navmesh_funnel_route(
      direct_navmesh_region, mm(20.0, 40.0), mm(80.0, 60.0), direct_navmesh_config);
  const auto prepared_navmesh =
      flux::prepare_navmesh_domain(direct_navmesh_region, direct_navmesh_config);
  const auto reused_navmesh = flux::route_prepared_navmesh(prepared_navmesh, mm(20.0, 40.0),
                                                           mm(80.0, 60.0), direct_navmesh_config);
  const auto flooded_navmesh = flux::route_prepared_navmesh_flood_fill(
      prepared_navmesh, mm(20.0, 40.0), {mm(80.0, 60.0), mm(20.0, 60.0)}, direct_navmesh_config);
  const auto closed_cell = reused_navmesh.corridor[reused_navmesh.corridor.size() / 2U];
  const auto reflooded_navmesh = flux::route_prepared_navmesh_flood_fill(
      prepared_navmesh, mm(20.0, 40.0), {mm(80.0, 60.0)}, direct_navmesh_config, {closed_cell});
  // A portal endpoint belongs to both adjacent cells. Closing the first cell must not
  // hide the still-open incident cell or let an interior endpoint escape a dynamic wall.
  const auto& binding_cell = prepared_navmesh.cells[prepared_navmesh.cells.size() - 1U];
  std::vector<std::size_t> binding_walls;
  for (const auto& cell : prepared_navmesh.cells)
    if (cell.id != binding_cell.id) binding_walls.push_back(cell.id);
  const auto boundary_bound = flux::route_prepared_navmesh_flood_fill(
      prepared_navmesh, binding_cell.corners.front(), {binding_cell.center},
      direct_navmesh_config, binding_walls);
  require(boundary_bound.ok(),
          "a shared-face endpoint must bind to the open incident cell of a constrained corridor");
  const auto closed_endpoint = flux::route_prepared_navmesh_flood_fill(
      prepared_navmesh, prepared_navmesh.cells[0U].center, {binding_cell.center},
      direct_navmesh_config, binding_walls);
  require(!closed_endpoint.ok(), "an endpoint inside a dynamic wall must not escape it");
  const auto rrt_navmesh = flux::route_prepared_navmesh_rrt_connect(
      prepared_navmesh, mm(20.0, 40.0), mm(80.0, 60.0), direct_navmesh_config,
      flux::NavMeshRrtConnectConfig{20'000U, 17U, 0U, 8U});
  flux::HydraulicPathConfig hydraulic_flow;
  hydraulic_flow.relaxation_iterations = 128U;
  hydraulic_flow.droplet_batches = 2U;
  hydraulic_flow.droplets_per_batch = 128U;
  hydraulic_flow.maximum_steps_per_droplet = 128U;
  hydraulic_flow.prefer_gpu = true;
  flux::NavMeshHydraulicConfig hydraulic_config;
  hydraulic_config.flow = hydraulic_flow;
  hydraulic_config.opened_guard_rings = 1U;
  const auto hydraulic_navmesh = flux::route_prepared_navmesh_hydraulic(
      prepared_navmesh, mm(20.0, 40.0), mm(80.0, 60.0), direct_navmesh_config, hydraulic_config);
  hydraulic_flow.prefer_gpu = false;
  hydraulic_config.flow = hydraulic_flow;
  const auto hydraulic_cpu_navmesh = flux::route_prepared_navmesh_hydraulic(
      prepared_navmesh, mm(20.0, 40.0), mm(80.0, 60.0), direct_navmesh_config, hydraulic_config);
  const std::vector<flux::RoutingComputeSegment> compute_segments{
      {0, 0, 1'000'000, 0, 50'000, 0U, 0U},
      {500'000, -500'000, 500'000, 500'000, 50'000, 0U, 1U},
      {2'000'000, 0, 2'000'000, 0, 50'000, 0U, 2U},
      {2'075'000, 0, 2'075'000, 0, 50'000, 0U, 3U}};
  std::vector<std::pair<std::uint32_t, std::uint32_t>> compute_pairs{{0U, 1U}, {0U, 2U}, {2U, 3U}};
  while (compute_pairs.size() < 256U) {
    compute_pairs.push_back(compute_pairs[compute_pairs.size() % 3U]);
  }
  const auto compute_conflicts_gpu =
      flux::discover_routing_segment_conflicts(compute_segments, compute_pairs, 1'024, true);
  const auto compute_conflicts_cpu =
      flux::discover_routing_segment_conflicts(compute_segments, compute_pairs, 1'024, false);
  std::vector<flux::RoutingComputePoint> compute_proximity_points;
  std::vector<flux::RoutingComputeSegment> compute_proximity_segments;
  for (std::size_t index = 0U; index < 64U; ++index) {
    compute_proximity_points.push_back(flux::RoutingComputePoint{
        static_cast<std::int64_t>(index) * 100'000, index % 2U == 0U ? 0 : 1'000'000, 10'000, 0U,
        std::numeric_limits<std::uint32_t>::max()});
    compute_proximity_segments.push_back(
        flux::RoutingComputeSegment{static_cast<std::int64_t>(index) * 100'000, -100'000,
                                    static_cast<std::int64_t>(index) * 100'000, 100'000, 10'000, 0U,
                                    static_cast<std::uint32_t>(index)});
  }
  const auto compute_proximity_gpu = flux::probe_routing_proximity(
      compute_proximity_points, compute_proximity_segments, 1'024, true);
  const auto compute_proximity_cpu = flux::probe_routing_proximity(
      compute_proximity_points, compute_proximity_segments, 1'024, false);
  flux::RoutingComputeWavefrontInput compute_wavefront;
  compute_wavefront.width = 64U;
  compute_wavefront.height = 64U;
  compute_wavefront.traversable.assign(64U * 64U, 1U);
  for (std::size_t y = 0U; y < 63U; ++y) {
    if (y != 31U)
      compute_wavefront.traversable[y * 64U + 32U] = 0U;
  }
  compute_wavefront.sources = {0U};
  compute_wavefront.maximum_passes = 256U;
  const auto compute_wavefront_gpu = flux::compute_routing_wavefront(compute_wavefront, true);
  const auto compute_wavefront_cpu = flux::compute_routing_wavefront(compute_wavefront, false);
  const auto compute_clearance_gpu = flux::compute_routing_clearance_field(
      compute_wavefront.width, compute_wavefront.height, compute_wavefront.traversable, 256U, true);
  const auto compute_clearance_cpu =
      flux::compute_routing_clearance_field(compute_wavefront.width, compute_wavefront.height,
                                            compute_wavefront.traversable, 256U, false);
  flux::RoutingComputeSplineBatchInput compute_spline;
  compute_spline.control_points_nm = {
      {0, 0},         {1'000'000, 0},         {1'000'000, 1'000'000}, {2'000'000, 1'000'000},
      {0, 2'000'000}, {1'000'000, 2'000'000}, {1'000'000, 3'000'000}, {2'000'000, 3'000'000}};
  compute_spline.sample_count = 128U;
  compute_spline.candidate_count = 2U;
  for (std::size_t candidate = 0U; candidate < compute_spline.candidate_count; ++candidate) {
    for (std::size_t sample = 0U; sample < compute_spline.sample_count; ++sample) {
      const auto parameter = static_cast<long double>(sample) /
                             static_cast<long double>(compute_spline.sample_count - 1U);
      const auto inverse = 1.0L - parameter;
      compute_spline.basis_coefficients.insert(
          compute_spline.basis_coefficients.end(),
          {inverse * inverse * inverse, 3.0L * inverse * inverse * parameter,
           3.0L * inverse * parameter * parameter, parameter * parameter * parameter});
    }
  }
  const auto compute_spline_gpu = flux::evaluate_routing_spline_batch(compute_spline, true);
  const auto compute_spline_cpu = flux::evaluate_routing_spline_batch(compute_spline, false);
  std::vector<flux::RoutingComputeLinearCandidate> compute_candidates(64U);
  for (std::size_t candidate = 0U; candidate < compute_candidates.size(); ++candidate) {
    compute_candidates[candidate].terms = {
        static_cast<std::uint32_t>(candidate), 2U, 3U, 4U, 5U, 6U, 7U, 8U};
  }
  const auto compute_scores_gpu = flux::score_routing_compute_candidates(
      compute_candidates, {1U, 10U, 100U, 1'000U, 10'000U, 100'000U, 1'000'000U, 10'000'000U},
      true);
  const auto compute_scores_cpu = flux::score_routing_compute_candidates(
      compute_candidates, {1U, 10U, 100U, 1'000U, 10'000U, 100'000U, 1'000'000U, 10'000'000U},
      false);
  const auto spline_batch_matches =
      compute_spline_gpu.ok() && compute_spline_cpu.ok() &&
      compute_spline_gpu.positions_nm.size() == compute_spline_cpu.positions_nm.size() &&
      std::equal(compute_spline_gpu.positions_nm.begin(), compute_spline_gpu.positions_nm.end(),
                 compute_spline_cpu.positions_nm.begin(),
                 [](const auto& gpu, const auto& cpu) {
                   return std::abs(gpu.first - cpu.first) <= 512.0L &&
                          std::abs(gpu.second - cpu.second) <= 512.0L;
                 }) &&
      compute_spline_cpu.positions_nm.size() == 256U &&
      std::abs(compute_spline_cpu.positions_nm[128U].second - 2'000'000.0L) <= 0.5L;
  require(
      direct_navmesh.ok() && prepared_navmesh.ok() && reused_navmesh.ok() &&
          direct_navmesh.corridor == reused_navmesh.corridor &&
          direct_navmesh.centerline == reused_navmesh.centerline &&
          reused_navmesh.used_flood_fill && reused_navmesh.flood_fill_cells_visited > 0U &&
          flooded_navmesh.ok() && flooded_navmesh.used_flood_fill &&
          flooded_navmesh.goal == mm(20.0, 60.0) && reflooded_navmesh.ok() &&
          std::find(reflooded_navmesh.corridor.begin(), reflooded_navmesh.corridor.end(),
                    closed_cell) == reflooded_navmesh.corridor.end() &&
          rrt_navmesh.ok() && rrt_navmesh.used_rrt_connect &&
          rrt_navmesh.rrt_connect_iterations > 0U &&
          flux::validate_navmesh_topology_signature(rrt_navmesh.topology_signature,
                                                    rrt_navmesh.centerline)
              .preserved &&
          hydraulic_navmesh.ok() && hydraulic_navmesh.used_hydraulic_erosion &&
          hydraulic_navmesh.hydraulic_direct_polygon_discovery &&
          hydraulic_navmesh.hydraulic_field_cell_pitch.value() ==
              direct_navmesh_config.cell_pitch.value() / 2 &&
          hydraulic_navmesh.hydraulic_discovery_cells > prepared_navmesh.cells.size() &&
          hydraulic_navmesh.hydraulic_exact_certified_chords > 0U &&
          hydraulic_navmesh.hydraulic_relaxation_passes > 0U &&
          hydraulic_navmesh.hydraulic_droplet_steps > 0U &&
          flux::validate_navmesh_topology_signature(hydraulic_navmesh.topology_signature,
                                                    hydraulic_navmesh.centerline)
              .preserved &&
          hydraulic_cpu_navmesh.ok() &&
          hydraulic_cpu_navmesh.centerline == hydraulic_navmesh.centerline &&
          hydraulic_cpu_navmesh.hydraulic_flow_field == hydraulic_navmesh.hydraulic_flow_field &&
          hydraulic_cpu_navmesh.hydraulic_erosion_field ==
              hydraulic_navmesh.hydraulic_erosion_field &&
          hydraulic_cpu_navmesh.hydraulic_heightmap == hydraulic_navmesh.hydraulic_heightmap &&
          hydraulic_cpu_navmesh.hydraulic_free_space_field ==
              hydraulic_navmesh.hydraulic_free_space_field &&
          hydraulic_navmesh.hydraulic_heightmap.size() ==
              hydraulic_navmesh.hydraulic_field_width * hydraulic_navmesh.hydraulic_field_height &&
          std::count_if(hydraulic_navmesh.hydraulic_free_space_field.begin(),
                        hydraulic_navmesh.hydraulic_free_space_field.end(),
                        [](std::uint16_t coverage) { return coverage > 0U; }) >
              hydraulic_navmesh.hydraulic_erosion_field.size() / 2U &&
          std::all_of(reused_navmesh.centerline.begin(), reused_navmesh.centerline.end(),
                      [&](const auto point) {
                        return flux::navmesh_corridor_contains(reused_navmesh, point);
                      }) &&
          compute_conflicts_gpu.ok() && compute_conflicts_cpu.ok() &&
          compute_conflicts_gpu.possible_conflicts == compute_conflicts_cpu.possible_conflicts &&
          compute_conflicts_cpu.possible_conflicts[0] == 1U &&
          compute_conflicts_cpu.possible_conflicts[1] == 0U &&
          compute_conflicts_cpu.possible_conflicts[2] == 1U && compute_proximity_gpu.ok() &&
          compute_proximity_cpu.ok() &&
          compute_proximity_gpu.queries.size() == compute_proximity_cpu.queries.size() &&
          std::equal(compute_proximity_gpu.queries.begin(), compute_proximity_gpu.queries.end(),
                     compute_proximity_cpu.queries.begin(),
                     [](const auto& gpu, const auto& cpu) {
                       return gpu.possible_conflict == cpu.possible_conflict &&
                              std::abs(gpu.conservative_margin_nm - cpu.conservative_margin_nm) <=
                                  512;
                     }) &&
          compute_wavefront_gpu.ok() && compute_wavefront_cpu.ok() &&
          compute_wavefront_gpu.distance == compute_wavefront_cpu.distance &&
          compute_clearance_gpu.ok() && compute_clearance_cpu.ok() &&
          compute_clearance_gpu.signed_distance == compute_clearance_cpu.signed_distance &&
          spline_batch_matches && compute_scores_gpu.ok() && compute_scores_cpu.ok() &&
          compute_scores_gpu.scores == compute_scores_cpu.scores,
      "prepared, one-shot, and multi-source flood-fill NavMesh routing must select legal "
      "deterministic topology and centerlines: " +
          direct_navmesh.diagnostic + "/" + reused_navmesh.diagnostic + "/" +
          flooded_navmesh.diagnostic + "/" + reflooded_navmesh.diagnostic + "/" +
          hydraulic_navmesh.diagnostic);
  const auto preserved_navmesh_topology = flux::validate_navmesh_topology_signature(
      direct_navmesh.topology_signature, direct_navmesh.centerline);
  auto reversed_navmesh_centerline = direct_navmesh.centerline;
  std::reverse(reversed_navmesh_centerline.begin(), reversed_navmesh_centerline.end());
  const auto reversed_navmesh_topology = flux::validate_navmesh_topology_signature(
      direct_navmesh.topology_signature, reversed_navmesh_centerline);
  require(direct_navmesh.topology_signature.valid &&
              !direct_navmesh.topology_signature.gates.empty() &&
              preserved_navmesh_topology.preserved && !reversed_navmesh_topology.preserved,
          "a selected NavMesh corridor must publish a directed gate certificate that accepts its "
          "funnel trace and rejects the reverse obstacle-side traversal");

  flux::NavMeshTopologyGate first_gate;
  first_gate.ordinal = 0U;
  first_gate.left_boundary = mm(3.0, -1.0);
  first_gate.right_boundary = mm(3.0, 1.0);
  first_gate.from_center = mm(2.0, 0.0);
  first_gate.to_center = mm(4.0, 0.0);
  auto second_gate = first_gate;
  second_gate.ordinal = 1U;
  second_gate.left_boundary = mm(7.0, -1.0);
  second_gate.right_boundary = mm(7.0, 1.0);
  second_gate.from_center = mm(6.0, 0.0);
  second_gate.to_center = mm(8.0, 0.0);
  const auto raster_gate_signature =
      flux::make_navmesh_topology_signature({first_gate, second_gate});
  const std::vector<Point> contained_gate_subsequence{mm(0.0, 0.0), mm(5.0, 0.0)};
  const auto strict_gate_sequence = flux::validate_navmesh_topology_signature(
      raster_gate_signature, contained_gate_subsequence);
  const auto relaxed_gate_subsequence = flux::validate_navmesh_topology_signature(
      raster_gate_signature, contained_gate_subsequence, true);
  require(!strict_gate_sequence.preserved &&
              strict_gate_sequence.reason == flux::NavMeshTopologyCheckReason::missing_gate &&
              relaxed_gate_subsequence.preserved && relaxed_gate_subsequence.gates_crossed == 1U,
          "a guard-contained smooth route may skip raster-only gates while retaining direction "
          "and order for every gate it actually crosses");

  const auto topology_spline = flux::make_topology_preserving_quintic_ph_route(
      direct_navmesh.centerline, Nanometres::from_mm(0.1));
  require(topology_spline.has_value() && topology_spline->tangent_continuous &&
              topology_spline->segments.size() + 1U == topology_spline->control_points.size() &&
              topology_spline->node_parameters.size() == topology_spline->control_points.size(),
          "a topology-preserving seed must be an explicit tangent-continuous quintic PH chain");
  auto knot_refined_spline = *topology_spline;
  const auto refined_start = flux::evaluate_quintic_ph(knot_refined_spline, 0.0L);
  const auto refined_end = flux::evaluate_quintic_ph(knot_refined_spline, 1.0L);
  const std::array refined_shape_parameters{0.19L, 0.37L, 0.61L, 0.83L};
  std::array<flux::SplineVector, refined_shape_parameters.size()> refined_shape_before{};
  for (std::size_t sample = 0U; sample < refined_shape_parameters.size(); ++sample) {
    refined_shape_before[sample] =
        flux::evaluate_quintic_ph(knot_refined_spline, refined_shape_parameters[sample]);
  }
  const auto knot_control_count = knot_refined_spline.control_points.size();
  const auto refined_shape_preserved = [&] {
    for (std::size_t sample = 0U; sample < refined_shape_parameters.size(); ++sample) {
      const auto after =
          flux::evaluate_quintic_ph(knot_refined_spline, refined_shape_parameters[sample]);
      if (std::hypotl(after.x - refined_shape_before[sample].x,
                      after.y - refined_shape_before[sample].y) > 1.0L)
        return false;
    }
    return true;
  };
  require(flux::insert_quintic_ph_interpolation_node(knot_refined_spline, 0.37L) &&
              knot_refined_spline.control_points.size() == knot_control_count + 1U &&
              knot_refined_spline.tangent_continuous &&
              flux::evaluate_quintic_ph(knot_refined_spline, 0.0L) == refined_start &&
              flux::evaluate_quintic_ph(knot_refined_spline, 1.0L) == refined_end &&
              refined_shape_preserved(),
          "adaptive PH refinement must exactly subdivide one span while retaining its shape, "
          "endpoints, and C1 continuity");
  const auto topology_samples = flux::adaptive_tessellate_quintic_ph(
      *topology_spline, Nanometres::from_mm(0.005), 12U, 2'048U);
  std::vector<Point> topology_polyline;
  topology_polyline.reserve(topology_samples.size());
  for (const auto& sample : topology_samples) {
    const auto point =
        Point{Nanometres{static_cast<std::int64_t>(std::llround(sample.position.x))},
              Nanometres{static_cast<std::int64_t>(std::llround(sample.position.y))}};
    if (topology_polyline.empty() || topology_polyline.back() != point) {
      topology_polyline.push_back(point);
    }
  }
  const auto topology_spline_check = flux::validate_navmesh_topology_signature(
      direct_navmesh.topology_signature, topology_polyline);
  require(topology_spline_check.preserved && flux::certify_quintic_ph_shape(*topology_spline).legal,
          "the rounded piecewise-cubic seed must preserve the selected directed NavMesh gates "
          "without loops or stationary reversals: " +
              topology_spline_check.diagnostic);

  const auto prepared_polygon = flux::geometry::prepare_polygon(direct_navmesh_region.area);
  for (const auto point : {mm(0.5, 0.5), mm(50.0, 50.0), mm(100.0, 50.0)}) {
    require(
        flux::geometry::locate_point(prepared_polygon, point) ==
                flux::geometry::locate_point(direct_navmesh_region.area, point) &&
            std::abs(flux::geometry::point_polygon_boundary_distance_nm(point, prepared_polygon) -
                     flux::geometry::point_polygon_boundary_distance_nm(
                         point, direct_navmesh_region.area)) < 1.0e-6L,
        "prepared polygon point location and nearest-boundary distance must remain exact");
  }

  const std::vector<Point> evaluator_points{mm(10.0, 10.0), mm(20.0, 15.0), mm(30.0, 8.0),
                                            mm(40.0, 20.0), mm(50.0, 10.0)};

  // Spline controls describe direction changes, not source sampling density. A straight route
  // with duplicated/subdivided points must remain exactly straight even when a soft clearance
  // gradient would otherwise reward bowing it into open space.
  const std::vector<Point> oversampled_straight{mm(10.0, 10.0), mm(10.0, 10.0), mm(15.0, 10.0),
                                                mm(20.0, 10.0), mm(25.0, 10.0), mm(30.0, 10.0),
                                                mm(30.0, 10.0)};
  auto straight_spline = flux::make_quintic_ph_route(oversampled_straight);
  require(straight_spline.has_value() && straight_spline->control_points.size() == 2U,
          "straight spline seeds must collapse repeated and forward-collinear samples");
  flux::SplineTrajectoryOptimizationConfig straight_config;
  straight_config.minimum_bend_radius = Nanometres{0};
  straight_config.maximum_corridor_displacement = Nanometres::from_mm(2.0);
  straight_config.samples = 24U;
  straight_config.maximum_iterations = 4U;
  straight_config.maximum_candidate_checks = 128U;
  const auto straight_optimized = flux::optimize_quintic_ph_trajectory(
      *straight_spline, straight_config, [](flux::SplineVector point) {
        return flux::SplineClearanceProbe{
            std::max(1.0L, 100'000.0L + point.y - 10'000'000.0L), {0.0L, 1.0L}, true};
      });
  const auto straight_samples =
      flux::adaptive_tessellate_quintic_ph(*straight_spline, Nanometres{1}, 8U, 128U);
  require(straight_optimized.accepted &&
              std::all_of(straight_samples.begin(), straight_samples.end(),
                          [](const auto& sample) {
                            return std::abs(sample.position.y - 10'000'000.0L) < 0.5L;
                          }),
          "trajectory optimization must not introduce a curve into an already legal straight "
          "route solely to improve a soft clearance objective");

  const auto evaluator_spline = flux::make_quintic_ph_route(evaluator_points);
  require(evaluator_spline.has_value(), "prepared spline evaluator fixture must form a spline");
  flux::PreparedQuinticPHEvaluator evaluator(*evaluator_spline, 17U);
  require(evaluator.update(*evaluator_spline),
          "prepared spline evaluator must accept its topology");
  for (std::size_t sample = 0U; sample < evaluator.samples().size(); ++sample) {
    const auto parameter = static_cast<long double>(sample) /
                           static_cast<long double>(evaluator.samples().size() - 1U);
    const auto expected = flux::evaluate_quintic_ph_differential(*evaluator_spline, parameter);
    const auto& actual = evaluator.samples()[sample];
    require(std::abs(expected.position.x - actual.position.x) < 1.0e-6L &&
                std::abs(expected.position.y - actual.position.y) < 1.0e-6L &&
                std::abs(expected.tangent.x - actual.tangent.x) < 1.0e-6L &&
                std::abs(expected.tangent.y - actual.tangent.y) < 1.0e-6L &&
                std::abs(expected.second_derivative.x - actual.second_derivative.x) < 1.0e-6L &&
                std::abs(expected.second_derivative.y - actual.second_derivative.y) < 1.0e-6L,
            "prepared spline samples must match analytical on-demand differentials");
  }
  const auto early_spline = flux::route_global(early_spline_board, early_spline_config);
  const auto retained_geometric_corridors = static_cast<std::size_t>(
      std::count_if(early_spline.paths.begin(), early_spline.paths.end(), [](const auto& path) {
        return std::any_of(path.geometric_corridors.begin(), path.geometric_corridors.end(),
                           [](const auto& corridor) {
                             return !corridor.topology_cells.empty() &&
                                    corridor.topology_signature.valid &&
                                    corridor.lane_centerline.size() >= 2U &&
                                    corridor.minimum_physical_cross_section_width >=
                                        corridor.required_envelope_width &&
                                    corridor.lane_capacity > 0U;
                           });
      }));
  require(early_spline.complete() && early_spline.static_seed_connections == 1U &&
              early_spline.static_seed_searches == 4U &&
              early_spline.static_seed_candidates >= 3U &&
              early_spline.iteration_zero_portfolios_evaluated > 0U &&
              early_spline.static_seed_hydraulic_attempts > 0U &&
              early_spline.static_seed_hydraulic_accepted +
                      early_spline.static_seed_hydraulic_failures ==
                  early_spline.static_seed_hydraulic_attempts &&
              early_spline.static_seed_corridor_nodes > 0U &&
              !early_spline.static_seed_budget_exhausted && retained_geometric_corridors > 0U,
          "the early hybrid-graph seed search must evaluate its complete mandatory portfolio, "
          "including hydraulic erosion, around a blocker before considering a copper shove: " +
              std::to_string(early_spline.complete()) + "/" +
              std::to_string(early_spline.static_seed_searches) + "/" +
              std::to_string(early_spline.static_seed_candidates) + "/" +
              std::to_string(early_spline.static_seed_corridor_nodes) + "/" +
              std::to_string(early_spline.static_seed_budget_exhausted) +
              "/geometry=" + std::to_string(retained_geometric_corridors) +
              "/nav=" + std::to_string(early_spline.static_seed_navmesh_routes) + "/" +
              std::to_string(early_spline.static_seed_navmesh_failures) +
              "/hydraulic=" + std::to_string(early_spline.static_seed_hydraulic_attempts) + "/" +
              std::to_string(early_spline.static_seed_hydraulic_accepted) + "/" +
              std::to_string(early_spline.static_seed_hydraulic_failures) +
              "/traj=" + std::to_string(early_spline.static_seed_trajectory_accepted) + "/" +
              std::to_string(early_spline.static_seed_trajectory_rejected));

  const std::vector<Point> spline_polyline{mm(10.0, 10.0), mm(30.0, 10.0), mm(30.0, 30.0),
                                           mm(50.0, 30.0)};
  auto local_spline = flux::make_quintic_ph_route(spline_polyline);
  require(local_spline.has_value(), "a bent polyline must initialize a quintic PH curve");
  const auto original_controls = local_spline->control_points;
  const auto original_midpoint = flux::evaluate_quintic_ph(*local_spline, 0.5L);
  const auto moved_controls = flux::shove_quintic_ph_interpolation_corridor(
      *local_spline, 0.5L, flux::SplineVector{0.0L, 1.0L}, Nanometres::from_mm(0.5),
      Nanometres::from_mm(1.0));
  const auto moved_midpoint = flux::evaluate_quintic_ph(*local_spline, 0.5L);
  require(moved_controls > 0U && moved_midpoint.y > original_midpoint.y &&
              local_spline->control_points[0] == original_controls[0] &&
              local_spline->control_points.back() == original_controls.back() &&
              local_spline->tangent_continuous,
          "PH interpolation-node shove must remain local, preserve endpoints, and rebuild C1");
  const auto differential = flux::evaluate_quintic_ph_differential(*local_spline, 0.5L);
  require(std::isfinite(differential.tangent.x) && std::isfinite(differential.tangent.y) &&
              differential.bend_radius_nm > 0.0L,
          "analytical quintic PH curve derivatives must expose finite tangent and bend-radius "
          "evidence");
  flux::SplineTrajectoryOptimizationConfig elastic_config;
  elastic_config.minimum_bend_radius = Nanometres{0};
  elastic_config.maximum_corridor_displacement = Nanometres::from_mm(5.0);
  elastic_config.samples = 12U;
  elastic_config.maximum_iterations = 1U;
  elastic_config.maximum_candidate_checks = 4U;
  const auto prior_elastic_midpoint = flux::evaluate_quintic_ph(*local_spline, 0.5L);
  const auto new_end =
      flux::SplineVector{local_spline->control_points.back().x,
                         local_spline->control_points.back().y + Nanometres::from_mm(1.0).value()};
  const auto elastic = flux::deform_quintic_ph_elastic(
      *local_spline, local_spline->control_points.front(), new_end, elastic_config,
      [](flux::SplineVector) { return flux::SplineClearanceProbe{1'000'000.0L, {}, true}; });
  const auto elastic_midpoint = flux::evaluate_quintic_ph(*local_spline, 0.5L);
  require(elastic.accepted && elastic.elastic_predeformed_controls > 0U &&
              local_spline->control_points.back() == new_end &&
              elastic_midpoint.y > prior_elastic_midpoint.y &&
              elastic_midpoint.y - prior_elastic_midpoint.y < Nanometres::from_mm(1.0).value(),
          "elastic endpoint deformation must preserve the endpoint constraint and attenuate its "
          "effect through nearby spline controls");

  // Shape validity is a hard route property, not a soft objective. An obstacle-free curve that
  // loops or reverses along its source corridor must therefore be rejected before materialization.
  flux::QuinticPHRoute looping_spline;
  for (const auto point : {mm(0.0, 0.0), mm(1.0, 0.0), mm(3.0, 3.0), mm(1.0, 3.0), mm(3.0, -3.0),
                           mm(5.0, 0.0), mm(6.0, 0.0)}) {
    looping_spline.control_points.push_back(flux::SplineVector{
        static_cast<long double>(point.x.value()), static_cast<long double>(point.y.value())});
  }
  for (std::int64_t x = 0; x <= 6; ++x) {
    looping_spline.corridor_anchors.push_back(
        flux::SplineVector{static_cast<long double>(Nanometres::from_mm(x).value()), 0.0L});
  }
  require(flux::rebuild_quintic_ph_route(looping_spline),
          "looping PH fixture must still be algebraically constructible");
  const auto looping_shape = flux::certify_quintic_ph_shape(looping_spline);
  require(!looping_shape.legal && looping_shape.violations() > 0U &&
              looping_shape.control_station_reversals > 0U,
          "a looping or corridor-reversing quintic PH curve must fail hard shape certification");
  for (const auto& smooth_seed :
       std::vector<std::vector<Point>>{{mm(0.0, 0.0), mm(10.0, 0.0), mm(20.0, 10.0)},
                                       {mm(0.0, 0.0), mm(10.0, 0.0), mm(10.0, 10.0)},
                                       {mm(0.0, 0.0), mm(10.0, 0.0), mm(3.0, 7.0)}}) {
    const auto smooth_spline = flux::make_quintic_ph_route(smooth_seed);
    require(smooth_spline.has_value() && flux::certify_quintic_ph_shape(*smooth_spline).legal,
            "ordinary acute, right, and obtuse route turns must remain simple forward curves");
  }

  // Equal endpoint motion is a rigid translation component. It must not bow the middle of the
  // route as the previous independent cubic endpoint falloffs did.
  auto translated_spline = flux::make_quintic_ph_route(
      std::vector<Point>{mm(10.0, 40.0), mm(20.0, 40.0), mm(30.0, 40.0)});
  require(translated_spline.has_value(), "translation fixture must initialize a quintic PH curve");
  const auto translated_before = translated_spline->control_points;
  const auto translation =
      flux::SplineVector{static_cast<long double>(Nanometres::from_mm(1.0).value()),
                         static_cast<long double>(Nanometres::from_mm(2.0).value())};
  auto translation_config = elastic_config;
  translation_config.maximum_iterations = 1U;
  translation_config.maximum_candidate_checks = 4U;
  const auto translated = flux::deform_quintic_ph_elastic(
      *translated_spline,
      flux::SplineVector{translated_before.front().x + translation.x,
                         translated_before.front().y + translation.y},
      flux::SplineVector{translated_before.back().x + translation.x,
                         translated_before.back().y + translation.y},
      translation_config,
      [](flux::SplineVector) { return flux::SplineClearanceProbe{1.0e12L, {}, true}; });
  require(translated.accepted &&
              std::equal(translated_before.begin(), translated_before.end(),
                         translated_spline->control_points.begin(),
                         [&](const auto& before, const auto& after) {
                           return std::abs(after.x - before.x - translation.x) < 0.5L &&
                                  std::abs(after.y - before.y - translation.y) < 0.5L;
                         }) &&
              flux::certify_quintic_ph_shape(*translated_spline).legal,
          "equal endpoint displacement must rigidly translate every control and retain a simple "
          "forward curve");

  // A large local collision displacement may be rejected, but can never be committed as a loop,
  // cusp, station reversal, or sub-radius bend.
  auto guarded_shove = flux::make_quintic_ph_route(
      std::vector<Point>{mm(0.0, 50.0), mm(1.0, 50.0), mm(2.0, 50.0), mm(3.0, 50.0)});
  require(guarded_shove.has_value(), "guarded shove fixture must initialize a quintic PH curve");
  const auto guarded_before = *guarded_shove;
  auto guarded_config = elastic_config;
  guarded_config.minimum_bend_radius = Nanometres::from_mm(0.1);
  guarded_config.maximum_corridor_displacement = Nanometres::from_mm(2.0);
  guarded_config.maximum_iterations = 4U;
  guarded_config.maximum_candidate_checks = 128U;
  const auto guarded_result = flux::deform_quintic_ph_elastic_at(
      *guarded_shove, 0.5L, flux::SplineVector{0.0L, 1.0L}, Nanometres::from_mm(2.0),
      guarded_config,
      [](flux::SplineVector) { return flux::SplineClearanceProbe{1.0e12L, {}, true}; });
  flux::SplineShapeCertificationConfig guarded_shape_config;
  guarded_shape_config.minimum_bend_radius = guarded_config.minimum_bend_radius;
  const auto guarded_shape = flux::certify_quintic_ph_shape(*guarded_shove, guarded_shape_config);
  require(guarded_shape.legal && (guarded_result.accepted ||
                                  guarded_shove->control_points == guarded_before.control_points),
          "a rejected shove must roll back, while an accepted shove must retain exact shape "
          "legality and bend radius");

  Board pns_board;
  pns_board.source_revision = "pns-actionable-risk-fixture";
  pns_board.outline = rectangle(0.0, 0.0, 100.0, 80.0);
  pns_board.layers = {{LayerId{"F.Cu"}, "F.Cu", LayerKind::copper, 0}};
  pns_board.nets = {{NetId{"net-1"}, "INTRUDER", 0},
                    {NetId{"pns-obstacle-net"}, "PNS_OBSTACLE", 0}};
  pns_board.tracks.push_back(flux::TrackSegment{ObjectId{"pns-intruder"}, NetId{"net-1"},
                                                LayerId{"F.Cu"}, mm(30.0, 50.0), mm(70.0, 50.0),
                                                Nanometres::from_mm(0.25), false});
  pns_board.tracks.push_back(flux::TrackSegment{ObjectId{"pns-obstacle"}, NetId{"pns-obstacle-net"},
                                                LayerId{"F.Cu"}, mm(20.0, 50.1), mm(80.0, 50.1),
                                                Nanometres::from_mm(0.25), false});
  const auto dynamic_pns_drc =
      flux::check_dynamic_copper_rules(pns_board, {ObjectId{"pns-intruder"}});
  require(!dynamic_pns_drc.ok() &&
              std::all_of(dynamic_pns_drc.violations.begin(), dynamic_pns_drc.violations.end(),
                          [](const auto& violation) {
                            return std::find(violation.object_ids.begin(),
                                             violation.object_ids.end(),
                                             "pns-intruder") != violation.object_ids.end();
                          }),
          "a dynamic local DRC probe must report exact violations involving its selected copper "
          "without rechecking unrelated source-only pairs");
  auto bounded_drc_board = pns_board;
  bounded_drc_board.nets.push_back({NetId{"static-a-net"}, "STATIC_A", 0});
  bounded_drc_board.nets.push_back({NetId{"static-b-net"}, "STATIC_B", 0});
  bounded_drc_board.tracks.push_back(flux::TrackSegment{
      ObjectId{"unrelated-static-a"}, NetId{"static-a-net"}, LayerId{"F.Cu"}, mm(30.0, 60.0),
      mm(70.0, 60.0), Nanometres::from_mm(0.25), false});
  bounded_drc_board.tracks.push_back(flux::TrackSegment{
      ObjectId{"unrelated-static-b"}, NetId{"static-b-net"}, LayerId{"F.Cu"}, mm(30.0, 60.1),
      mm(70.0, 60.1), Nanometres::from_mm(0.25), false});
  const auto whole_bounded_fixture = flux::check_copper_rules(bounded_drc_board);
  const auto dynamic_bounded_fixture =
      flux::check_dynamic_copper_rules(bounded_drc_board, {ObjectId{"pns-intruder"}});
  const auto canonical_violations = [](const auto& report, std::string_view selected) {
    std::set<std::string> keys;
    for (const auto& violation : report.violations) {
      if (!selected.empty() && std::find(violation.object_ids.begin(), violation.object_ids.end(),
                                         selected) == violation.object_ids.end())
        continue;
      auto ids = violation.object_ids;
      std::sort(ids.begin(), ids.end());
      auto key = std::to_string(static_cast<int>(violation.kind)) + ":" + violation.code;
      for (const auto& id : ids)
        key += ":" + id;
      keys.insert(std::move(key));
    }
    return keys;
  };
  require(canonical_violations(whole_bounded_fixture, "pns-intruder") ==
              canonical_violations(dynamic_bounded_fixture, "") &&
              whole_bounded_fixture.violation_count() >
                  dynamic_bounded_fixture.violation_count(),
          "bounded proposal DRC must exactly match the selected subset of whole-board DRC while "
          "excluding an unrelated immutable violation");
  auto selected_via_board = bounded_drc_board;
  selected_via_board.nets.push_back({NetId{"dynamic-via-net"}, "DYNAMIC_VIA", 0});
  selected_via_board.vias.push_back(
      {ObjectId{"dynamic-via"}, NetId{"dynamic-via-net"}, mm(50.0, 50.0),
       Nanometres::from_mm(0.6), Nanometres::from_mm(0.3), {LayerId{"F.Cu"}}, false});
  const auto whole_selected_via = flux::check_copper_rules(selected_via_board);
  const auto dynamic_selected_via =
      flux::check_dynamic_copper_rules(selected_via_board, {ObjectId{"dynamic-via"}});
  require(canonical_violations(whole_selected_via, "dynamic-via") ==
              canonical_violations(dynamic_selected_via, "") &&
              std::any_of(dynamic_selected_via.violations.begin(),
                          dynamic_selected_via.violations.end(), [](const auto& violation) {
                            return violation.kind ==
                                   flux::CopperViolationKind::track_via_clearance;
                          }),
          "bounded DRC for a selected via must retain track/via checks whose track side is "
          "immutable");
  const flux::PnsShoveRequest pns_request{{"pns-intruder"},
                                          {"pns-obstacle"},
                                          {},
                                          mm(50.0, 50.0),
                                          0.0L,
                                          1.0L,
                                          Nanometres::from_mm(0.5),
                                          {}};
  const auto pns = flux::pns_push_and_shove(pns_board, pns_request);
  require(pns.accepted() && pns.moved_tracks == 1U && pns.local_splits == 1U &&
              pns.added_dogleg_tracks >= 3U && flux::check_copper_rules(*pns.candidate).ok(),
          "bounded PNS must move an unlocked parallel blocker with connectivity-preserving "
          "doglegs and exact DRC acceptance: " +
              pns.diagnostic);

  Board via_pns_board;
  via_pns_board.source_revision = "pns-via-transaction-risk-fixture";
  via_pns_board.outline = rectangle(0.0, 0.0, 100.0, 80.0);
  via_pns_board.layers = {{LayerId{"F.Cu"}, "F.Cu", LayerKind::copper, 0},
                          {LayerId{"B.Cu"}, "B.Cu", LayerKind::copper, 1}};
  via_pns_board.nets = {{NetId{"via-intruder"}, "VIA_INTRUDER", 0},
                        {NetId{"via-shoved"}, "VIA_SHOVED", 0}};
  via_pns_board.tracks = {
      flux::TrackSegment{ObjectId{"via-intruder-track"}, NetId{"via-intruder"}, LayerId{"F.Cu"},
                         mm(30.0, 50.0), mm(70.0, 50.0), Nanometres::from_mm(0.25), false},
      flux::TrackSegment{ObjectId{"via-attached-front"}, NetId{"via-shoved"}, LayerId{"F.Cu"},
                         mm(20.0, 50.1), mm(50.0, 50.1), Nanometres::from_mm(0.25), false},
      flux::TrackSegment{ObjectId{"via-attached-back"}, NetId{"via-shoved"}, LayerId{"B.Cu"},
                         mm(50.0, 50.1), mm(80.0, 60.0), Nanometres::from_mm(0.25), false}};
  via_pns_board.vias = {flux::Via{ObjectId{"movable-via"},
                                  NetId{"via-shoved"},
                                  mm(50.0, 50.1),
                                  Nanometres::from_mm(0.8),
                                  Nanometres::from_mm(0.4),
                                  {LayerId{"F.Cu"}, LayerId{"B.Cu"}},
                                  false}};
  const flux::PnsShoveRequest via_pns_request{{"via-intruder-track"},
                                              {"movable-via"},
                                              {},
                                              mm(50.0, 50.0),
                                              0.0L,
                                              1.0L,
                                              Nanometres::from_mm(0.5),
                                              {}};
  const auto via_pns = flux::pns_push_and_shove(via_pns_board, via_pns_request);
  require(via_pns.accepted() && via_pns.moved_vias == 1U && via_pns.moved_tracks == 2U &&
              via_pns.candidate && via_pns.candidate->vias.front().position != mm(50.0, 50.1) &&
              flux::check_copper_rules(*via_pns.candidate).ok(),
          "bounded PNS must move an unlocked via and every attached straight-track endpoint as "
          "one exact transaction: " +
              via_pns.diagnostic);
  flux::GlobalRouteResult shove_global;
  shove_global.stop_reason = GlobalRouterStopReason::complete;
  flux::GlobalRoutingShoveProposal shove_proposal;
  shove_proposal.id = 0U;
  shove_proposal.intruding_net = NetId{"net-1"};
  shove_proposal.affected_nets = {NetId{"pns-obstacle-net"}};
  shove_proposal.replaced_tracks = {ObjectId{"pns-obstacle"}};
  shove_proposal.propagation_depth = pns.propagation_depth;
  const std::set<ObjectId> pns_source_ids{ObjectId{"pns-intruder"}, ObjectId{"pns-obstacle"}};
  for (const auto& track : pns.candidate->tracks) {
    if (!pns_source_ids.contains(track.id)) {
      shove_proposal.replacement_tracks.push_back(track);
    }
  }
  shove_global.selected_shove_proposals.push_back(std::move(shove_proposal));
  const auto shove_detail = flux::materialize_detailed_route(pns_board, shove_global);
  require(shove_detail.complete() && shove_detail.candidate && shove_detail.copper_drc.ok() &&
              std::none_of(shove_detail.candidate->tracks.begin(),
                           shove_detail.candidate->tracks.end(),
                           [](const auto& track) { return track.id == ObjectId{"pns-obstacle"}; }),
          "an iteration-zero PNS delta must remain actionable through detailed transactional "
          "materialization");
}

void integrated_placement_rejects_proxy_improvement_that_violates_copper_rules() {
  auto board = two_pad_board();
  board.layers = {{LayerId{"F.Cu"}, "F.Cu", LayerKind::copper, 0}};
  board.footprints.front().locked = true;
  Keepout pad_keepout;
  pad_keepout.id = ObjectId{"pad-only-keepout"};
  pad_keepout.area = rectangle(25.0, 35.0, 40.0, 45.0);
  pad_keepout.layers = {LayerId{"F.Cu"}};
  pad_keepout.prohibit.pads = true;
  board.keepouts.push_back(std::move(pad_keepout));

  flux::IntegratedOptimizerConfig config;
  config.enable_global_placement = true;
  config.global_placer.iterations = 30U;
  config.global_placer.attraction_strength = 0.5L;
  config.global_placer.source_anchor_weight = 0.05L;
  config.global_placer.placement_grid = Nanometres::from_mm(5.0);
  config.global_router.grid_pitch = Nanometres::from_mm(5.0);
  config.max_passes = 1U;
  config.max_candidate_evaluations = 2U;

  const auto result = flux::optimize_integrated_placement_routing(board, config);
  require(result.complete() && result.global_placement && result.global_placement->complete() &&
              result.global_placement->after.hpwl_nm < result.global_placement->before.hpwl_nm,
          "fixture must generate a proxy-improving placement");
  require(!result.global_seed_accepted && result.accepted_moves.empty() && result.candidate &&
              result.candidate->footprints[1].position == board.footprints[1].position &&
              result.score.copper_violations == 0U,
          "exact copper acceptance must reject the proxy improvement and retain the source");

  auto continuation_config = config;
  continuation_config.enable_global_placement = false;
  continuation_config.optimize_routed_quality = true;
  continuation_config.max_candidate_evaluations = 1U;
  continuation_config.candidate_evaluation_offset = 2U;
  const auto continuation =
      flux::optimize_integrated_placement_routing(two_pad_board(), continuation_config);
  require(continuation.configured_candidate_evaluation_offset == 2U &&
              continuation.candidate_evaluations_skipped == 2U &&
              continuation.candidate_evaluations == 1U && continuation.accepted_moves.empty() &&
              continuation.stop_reason == flux::IntegratedOptimizerStopReason::evaluation_limit,
          "a deterministic candidate offset must skip only the certified prefix and must not "
          "claim a standalone local optimum");

  auto serial_wave_config = continuation_config;
  serial_wave_config.candidate_evaluation_offset = 0U;
  serial_wave_config.max_candidate_evaluations = 6U;
  serial_wave_config.max_candidate_evaluation_workers = 1U;
  serial_wave_config.max_full_routing_candidates_per_wave = 2U;
  const auto serial_wave =
      flux::optimize_integrated_placement_routing(two_pad_board(), serial_wave_config);
  auto parallel_wave_config = serial_wave_config;
  parallel_wave_config.max_candidate_evaluation_workers = 4U;
  const auto parallel_wave =
      flux::optimize_integrated_placement_routing(two_pad_board(), parallel_wave_config);
  const bool same_candidate_transforms =
      serial_wave.candidate && parallel_wave.candidate &&
      serial_wave.candidate->footprints.size() == parallel_wave.candidate->footprints.size() &&
      std::equal(serial_wave.candidate->footprints.begin(), serial_wave.candidate->footprints.end(),
                 parallel_wave.candidate->footprints.begin(), [](const auto& lhs, const auto& rhs) {
                   return lhs.id == rhs.id && lhs.position == rhs.position &&
                          lhs.rotation == rhs.rotation && lhs.side == rhs.side;
                 });
  require(serial_wave.score == parallel_wave.score &&
              serial_wave.stop_reason == parallel_wave.stop_reason && same_candidate_transforms &&
              serial_wave.accepted_moves.size() == parallel_wave.accepted_moves.size() &&
              serial_wave.candidate_evaluations == parallel_wave.candidate_evaluations &&
              serial_wave.candidate_evaluation_tasks_dispatched ==
                  parallel_wave.candidate_evaluation_tasks_dispatched &&
              serial_wave.candidate_evaluation_waves > 0U &&
              serial_wave.peak_candidate_evaluation_workers == 1U &&
              parallel_wave.peak_candidate_evaluation_workers > 1U,
          "candidate-wave reduction must retain the same exact winner and score independently of "
          "worker completion order");
}

void kicad_drill_offsets_preserve_copper_and_hole_anchors() {
  const auto imported = flux::io::import_kicad_pcb_file(
      std::filesystem::path{FLUX_TEST_SOURCE_DIR} / "fixtures/pad-offset-frames.kicad_pcb");
  require(imported.strict() && imported.board->footprints.size() == 8U,
          "offset-pad frame fixture must import strictly");
  const auto shapes = flux::copper_pad_shapes(*imported.board);
  for (const auto& footprint : imported.board->footprints) {
    for (std::size_t index = 0; index < footprint.pads.size(); ++index) {
      const auto& pad = footprint.pads[index];
      const auto hole = flux::geometry::transform_point(
          mm(2.0 + 2.0 * index, 3.0), footprint.position, footprint.rotation, BoardSide::front);
      require(flux::pad_drill_world_position(footprint, pad) == hole,
              "serialized offset must not displace the physical hole anchor");
      const auto shape = std::find_if(shapes.begin(), shapes.end(),
                                     [&](const auto& item) { return item.id == pad.id; });
      const auto copper = pad.plated ? flux::geometry::transform_point(
          mm(0.2, 0.1), hole, Microdegrees{}, BoardSide::front) : hole;
      require(shape != shapes.end() && shape->position == copper,
              "copper offsets must move the annulus, while non-plated obstacles stay at the hole");
    }
  }
}

void kicad_pad_frames_match_placed_board_coordinates() {
  const auto imported = flux::io::import_kicad_pcb_file(
      std::filesystem::path{FLUX_TEST_SOURCE_DIR} / "fixtures/pad-coordinate-frames.kicad_pcb");
  require(imported.strict() && imported.board->footprints.size() == 8U,
          "front/back pad-frame fixture must import strictly");
  const auto copper = flux::copper_pad_shapes(*imported.board);
  for (const auto& footprint : imported.board->footprints) {
    for (std::size_t index = 0; index < footprint.pads.size(); ++index) {
      const auto& pad = footprint.pads[index];
      const auto shape = std::find_if(copper.begin(), copper.end(),
                                     [&](const auto& item) { return item.id == pad.id; });
      const auto file_local = mm(2.0 + 2.0 * index, 3.0);
      // KiCad board syntax already contains reflected back-side local positions. Its placed
      // coordinates therefore apply rotation and translation once, with no second mirror.
      const auto expected = flux::geometry::transform_point(
          file_local, footprint.position, footprint.rotation, BoardSide::front);
      require(shape != copper.end() && shape->position == expected,
              "back-side file coordinates must not be reflected twice");
      const auto expected_angle = Microdegrees{index == 0U ? 0 : -45'000'000}.value();
      require(shape->rotation.value() == expected_angle,
              "omitted and explicit pad angles must remain absolute in the placed board frame");
    }
    const auto world_courtyard = flux::geometry::transform_polygon(
        footprint.courtyard, footprint.position, footprint.rotation, footprint.side);
    const auto expected_corner = flux::geometry::transform_point(
        mm(6.0, 8.0), footprint.position, footprint.rotation, BoardSide::front);
    require(std::find(world_courtyard.vertices.begin(), world_courtyard.vertices.end(),
                      expected_corner) != world_courtyard.vertices.end(),
            "asymmetric back-side courtyards must share the pad coordinate frame");
  }
  const auto edge_board = flux::io::import_kicad_pcb(R"((kicad_pcb
    (version 20241229) (generator "pcbnew")
    (layers (0 "F.Cu" signal) (31 "B.Cu" signal) (44 "Edge.Cuts" user))
    (net 0 "")
    (footprint "EdgeFixture" (layer "B.Cu") (at 20 30 90)
      (fp_rect (start 0 0) (end 6 8) (stroke (width 0.05) (type default))
        (fill none) (layer "Edge.Cuts")))))");
  require(edge_board.strict() &&
              flux::geometry::prepare_polygon(edge_board.board->outline).bounds.minimum == mm(20.0, 24.0) &&
              flux::geometry::prepare_polygon(edge_board.board->outline).bounds.maximum == mm(28.0, 30.0),
          "back-side footprint Edge.Cuts must not acquire a second reflection");
  const auto reference = flux::io::import_kicad_pcb_file(
      std::filesystem::path{FLUX_TEST_SOURCE_DIR} / "fixtures/routing-corpus/reference/bm_svf.kicad_pcb");
  require(reference.strict() && flux::analyze_connectivity(*reference.board).open_connections == 0U,
          "known-connected KRT copper must remain connected after canonical import");
}

void cancellation_and_kicad_export_boundaries_fail_closed() {
  const auto board = two_pad_board();
  std::stop_source stop;
  stop.request_stop();
  require(flux::route_global(board, {}, stop.get_token()).stop_reason ==
              GlobalRouterStopReason::cancelled,
          "routing must honor cancellation before mutation");
  require(flux::optimize_integrated_placement_routing(board, {}, stop.get_token()).stop_reason ==
              flux::IntegratedOptimizerStopReason::cancelled,
          "integrated optimization must honor cancellation before mutation");

  GlobalRouterConfig telemetry_route_config;
  telemetry_route_config.grid_pitch = Nanometres::from_mm(5.0);
  telemetry_route_config.max_iterations = 2U;
  const auto unobserved_route = flux::route_global(board, telemetry_route_config);
  flux::RouterTelemetryConfig telemetry_config;
  telemetry_config.queue_capacity = 64U;
  telemetry_config.publish_rate_hz = 1'000U;
  flux::RouterTelemetryService telemetry(telemetry_config);
  const auto observed_route = flux::route_global(board, telemetry_route_config, {}, &telemetry);
  telemetry.stop();
  const auto observed_statistics = telemetry.statistics();
  const auto observed_state = telemetry.state();
  const auto serialized_snapshot =
      flux::serialize_router_telemetry_snapshot(observed_state, observed_statistics);
  const auto decoded_snapshot = flux::telemetry::decode_events(serialized_snapshot);
  const auto retained_coarse_routes = static_cast<std::size_t>(std::count_if(
      observed_state.observed_routes.begin(), observed_state.observed_routes.end(),
      [](const auto& route) { return route.route_lifecycle == "retained_fallback"; }));
  require(
      unobserved_route.stop_reason == observed_route.stop_reason &&
          unobserved_route.paths == observed_route.paths &&
          unobserved_route.unrouted_nets == observed_route.unrouted_nets &&
          unobserved_route.total_overflow == observed_route.total_overflow &&
          unobserved_route.total_cost == observed_route.total_cost &&
          observed_statistics.produced == observed_statistics.consumed &&
          observed_statistics.produced >= 4U && observed_statistics.dropped == 0U &&
          !observed_state.routing_finished &&
          observed_state.observed_routes.size() >= observed_route.paths.size() &&
          retained_coarse_routes > 0U && decoded_snapshot && decoded_snapshot.events.size() == 1U &&
          decoded_snapshot.events.front().type == flux::telemetry::EventType::snapshot &&
          decoded_snapshot.events.front().message == "Routing snapshot" &&
          decoded_snapshot.events.front().metrics.has_value() &&
          decoded_snapshot.events.front().metrics->route_cost == observed_state.route_cost &&
          decoded_snapshot.events.front().routes.size() == observed_state.observed_routes.size() &&
          !decoded_snapshot.events.front().routes.front().points.empty(),
      "a disabled/enabled telemetry observer must preserve deterministic routing bytes and "
      "drain a viewer-decodable versioned final snapshot without fabricating a successful "
      "routing outcome or becoming routing state");

  flux::RouterTelemetryEvent queued_event;
  queued_event.kind = flux::RouterTelemetryEventKind::net_queued;
  queued_event.net = NetId{"net-1"};
  queued_event.stable_id = "pair:gate";
  queued_event.priority_stage = "constrained";
  queued_event.explicit_priority = 7U;
  queued_event.terminal_portals = 1U;
  queued_event.corridor_alternatives = 2U;
  queued_event.via_transitions = 1U;
  queued_event.via_alternatives = 1U;
  queued_event.via_contention = 3U;
  queued_event.estimated_interference = 4U;
  queued_event.physical_envelope_nm = 500'000U;
  queued_event.topology_alternatives = 2U;
  queued_event.layer_alternatives = 1U;
  queued_event.priority_ordinal = 9U;
  const auto decoded_priority =
      flux::telemetry::decode_events(flux::serialize_router_telemetry_event(queued_event));
  require(decoded_priority && decoded_priority.events.size() == 1U &&
              decoded_priority.events.front().type == flux::telemetry::EventType::net_queued &&
              decoded_priority.events.front().active_net == std::optional<std::string>{"net-1"} &&
              decoded_priority.events.front().priorities.size() == 1U &&
              decoded_priority.events.front().priorities.front().group == "pair:gate" &&
              decoded_priority.events.front().priorities.front().staged_class == "constrained",
          "routing-priority telemetry must use the viewer protocol and retain stable Flux IDs");

  flux::RouterTelemetryEvent global_stage_finished;
  global_stage_finished.kind = flux::RouterTelemetryEventKind::global_routing_finished;
  global_stage_finished.stable_id = "global-routing-stage";
  global_stage_finished.detail = "global routing stage completed: checkpoint retained";
  const auto decoded_global_stage =
      flux::telemetry::decode_events(flux::serialize_router_telemetry_event(global_stage_finished));
  require(
      decoded_global_stage && decoded_global_stage.events.size() == 1U &&
          decoded_global_stage.events.front().type == flux::telemetry::EventType::search_summary &&
          decoded_global_stage.events.front().message.find("stage completed") != std::string::npos,
      "a nested global-router completion must remain a stage summary rather than marking the "
      "complete routing session finished");

  flux::RouterTelemetryConfig overflow_config;
  overflow_config.queue_capacity = 1U;
  overflow_config.publish_rate_hz = 1U;
  flux::RouterTelemetryService overflow_telemetry(overflow_config);
  std::vector<std::thread> producers;
  for (std::size_t producer = 0U; producer < 4U; ++producer) {
    producers.emplace_back([&overflow_telemetry, producer]() {
      for (std::size_t event = 0U; event < 2'000U; ++event) {
        flux::RouterTelemetryEvent observation;
        observation.kind = flux::RouterTelemetryEventKind::route_candidate_selected;
        observation.stable_id = std::to_string(producer) + ":" + std::to_string(event);
        static_cast<void>(overflow_telemetry.try_emit(std::move(observation)));
      }
    });
  }
  for (auto& producer : producers) {
    producer.join();
  }
  overflow_telemetry.stop();
  const auto overflow_statistics = overflow_telemetry.statistics();
  require(overflow_statistics.dropped > 0U &&
              overflow_statistics.produced == overflow_statistics.consumed &&
              overflow_statistics.produced + overflow_statistics.dropped == 8'000U,
          "the bounded MPSC telemetry queue must drop under contention without losing accepted "
          "events or waiting for its consumer");

  auto validation_board = board;
  flux::CopperZone zone;
  zone.id = ObjectId{"static-zone"};
  zone.net = NetId{"net-1"};
  zone.name = "Static zone";
  zone.fills = {flux::CopperZoneFill{
      LayerId{"F.Cu"}, rectangle(5.0, 5.0, 15.0, 15.0), {rectangle(7.0, 7.0, 9.0, 9.0)}}};
  validation_board.zones.push_back(std::move(zone));
  const auto baseline = flux::make_board_validation_baseline(validation_board);
  auto placement_candidate = validation_board;
  placement_candidate.footprints[1].position = mm(75.0, 40.0);
  const auto ordinary_validation = flux::validate_board(placement_candidate);
  const auto reused_validation = flux::validate_board_candidate(placement_candidate, baseline);
  const auto issue_identity = [](const flux::ValidationReport& lhs,
                                 const flux::ValidationReport& rhs) {
    if (lhs.issues.size() != rhs.issues.size()) {
      return false;
    }
    for (std::size_t index = 0U; index < lhs.issues.size(); ++index) {
      if (lhs.issues[index].severity != rhs.issues[index].severity ||
          lhs.issues[index].code != rhs.issues[index].code ||
          lhs.issues[index].object_ids != rhs.issues[index].object_ids) {
        return false;
      }
    }
    return true;
  };
  require(reused_validation.reused_static_zone_geometry &&
              issue_identity(ordinary_validation, reused_validation.validation),
          "an exact unchanged-zone baseline must preserve canonical candidate validation");
  placement_candidate.zones.front().fills.front().holes.front().vertices.front() = mm(4.0, 4.0);
  const auto changed_zone_validation =
      flux::validate_board_candidate(placement_candidate, baseline);
  require(!changed_zone_validation.reused_static_zone_geometry &&
              std::any_of(changed_zone_validation.validation.issues.begin(),
                          changed_zone_validation.validation.issues.end(),
                          [](const auto& issue) { return issue.code == "invalid_zone_fill_hole"; }),
          "changed static geometry must fall back to full validation and retain exact rejection");

  const auto contents = load_minimal_kicad_fixture();
  const auto imported = flux::io::import_kicad_pcb(contents);
  require(imported.strict(), "the canonical KiCad fixture must import without loss");
  const auto stackup_path =
      std::filesystem::path{FLUX_TEST_SOURCE_DIR} / "fixtures/routing-corpus/bm_svf.kicad_pcb";
  std::ifstream stackup_input(stackup_path, std::ios::binary);
  const std::string stackup_contents{std::istreambuf_iterator<char>{stackup_input},
                                     std::istreambuf_iterator<char>{}};
  const auto stackup_import = flux::io::import_kicad_pcb(stackup_contents);
  const auto dielectric =
      stackup_import.board && stackup_import.board->stackup
          ? std::find_if(
                stackup_import.board->stackup->layers.begin(),
                stackup_import.board->stackup->layers.end(),
                [](const auto& layer) { return layer.kind == flux::StackupLayerKind::dielectric; })
          : std::vector<flux::StackupLayer>::const_iterator{};
  require(!stackup_contents.empty() && stackup_import.strict() && stackup_import.board &&
              stackup_import.board->physical_thickness == Nanometres{1'600'000} &&
              stackup_import.board->stackup && stackup_import.board->stackup->layers.size() == 9U &&
              dielectric != stackup_import.board->stackup->layers.end() &&
              dielectric->sublayers.size() == 1U &&
              dielectric->sublayers.front().thickness == Nanometres{1'510'000} &&
              dielectric->sublayers.front().relative_permittivity_ppb == 4'500'000'000LL &&
              dielectric->sublayers.front().loss_tangent_ppb == 20'000'000LL,
          "KiCad stackup thickness and dielectric properties must enter the canonical board as "
          "deterministic fixed-point physical data");
  const auto unchanged_stackup_export = flux::io::export_kicad_candidate(
      stackup_contents, *stackup_import.board, *stackup_import.board);
  require(unchanged_stackup_export.ok(),
          "source-preserving export must round-trip an unchanged represented stackup exactly");
  auto sublayer_stackup = stackup_contents;
  const std::string loss_tangent_property{"(loss_tangent 0.02)"};
  const auto sublayer_insertion = sublayer_stackup.find(loss_tangent_property);
  require(sublayer_insertion != std::string::npos,
          "stackup fixture must retain an identifiable dielectric sublayer boundary");
  sublayer_stackup.insert(
      sublayer_insertion + loss_tangent_property.size(),
      "\n\t\t\taddsublayer\n\t\t\t(thickness 0.1 locked)\n\t\t\t(material \"Resin\")\n"
      "\t\t\t(epsilon_r 3.25)\n\t\t\t(loss_tangent 0.015)");
  const auto sublayer_import = flux::io::import_kicad_pcb(sublayer_stackup);
  const std::vector<flux::StackupSublayer>* imported_sublayers = nullptr;
  if (sublayer_import.board && sublayer_import.board->stackup) {
    const auto imported_dielectric =
        std::find_if(sublayer_import.board->stackup->layers.begin(),
                     sublayer_import.board->stackup->layers.end(), [](const auto& layer) {
                       return layer.kind == flux::StackupLayerKind::dielectric;
                     });
    if (imported_dielectric != sublayer_import.board->stackup->layers.end()) {
      imported_sublayers = &imported_dielectric->sublayers;
    }
  }
  require(sublayer_import.strict() && imported_sublayers && imported_sublayers->size() == 2U &&
              (*imported_sublayers)[1].thickness == Nanometres{100'000} &&
              (*imported_sublayers)[1].thickness_locked &&
              (*imported_sublayers)[1].relative_permittivity_ppb == 3'250'000'000LL &&
              (*imported_sublayers)[1].loss_tangent_ppb == 15'000'000LL,
          "KiCad addsublayer construction must preserve each dielectric material independently");
  auto unsupported_stackup = stackup_contents;
  const auto loss_tangent = unsupported_stackup.find("(loss_tangent 0.02)");
  require(loss_tangent != std::string::npos,
          "stackup fixture must retain an identifiable dielectric property");
  unsupported_stackup.insert(loss_tangent, "(spec_frequency 1000000000)\n\t\t\t\t");
  const auto unsupported_stackup_import = flux::io::import_kicad_pcb(unsupported_stackup);
  require(!unsupported_stackup_import.strict() &&
              std::any_of(unsupported_stackup_import.diagnostics.begin(),
                          unsupported_stackup_import.diagnostics.end(),
                          [](const auto& diagnostic) {
                            return diagnostic.code == "unsupported_stackup_layer_property";
                          }),
          "unrepresented frequency-dependent stackup semantics must stop strict import with an "
          "attributed diagnostic");
  auto microvia_contents = contents;
  const auto microvia_board_close = microvia_contents.rfind(')');
  require(microvia_board_close != std::string::npos,
          "microvia fixture must retain the root closing delimiter");
  microvia_contents.insert(
      microvia_board_close,
      "  (via micro (at 50 40) (size 0.4) (drill 0.1) (layers \"F.Cu\" \"B.Cu\") "
      "(net 1) (uuid \"unsupported-microvia\"))\n");
  const auto microvia_import = flux::io::import_kicad_pcb(microvia_contents);
  require(!microvia_import.strict() &&
              std::any_of(
                  microvia_import.diagnostics.begin(), microvia_import.diagnostics.end(),
                  [](const auto& diagnostic) { return diagnostic.code == "unsupported_microvia"; }),
          "laser-drilled microvias must fail strict import until their fabrication semantics are "
          "represented explicitly");
  auto unfilled_zone_contents = contents;
  const auto board_close = unfilled_zone_contents.rfind(')');
  require(board_close != std::string::npos,
          "unfilled-zone fixture must retain the root closing delimiter");
  unfilled_zone_contents.insert(
      board_close, "  (zone (layer \"B.Cu\") (uuid \"unfilled-zone-test\") (hatch edge 0.5)\n"
                   "    (polygon (pts (xy 20 20) (xy 30 20) (xy 30 30) (xy 20 30))))\n");
  const auto unfilled_zone_import = flux::io::import_kicad_pcb(unfilled_zone_contents);
  const auto imported_unfilled_zone =
      unfilled_zone_import.board
          ? std::find_if(unfilled_zone_import.board->zones.begin(),
                         unfilled_zone_import.board->zones.end(),
                         [](const auto& zone) { return zone.id == ObjectId{"unfilled-zone-test"}; })
          : std::vector<flux::CopperZone>::const_iterator{};
  require(unfilled_zone_import.strict() && unfilled_zone_import.board &&
              imported_unfilled_zone != unfilled_zone_import.board->zones.end() &&
              imported_unfilled_zone->requires_refill && imported_unfilled_zone->fills.empty() &&
              imported_unfilled_zone->net == NetId{"kicad-net-0"},
          "a netless KiCad zone without saved fill must retain strict source-bound refill state");
  const auto unfilled_zone_export = flux::io::export_kicad_candidate(
      unfilled_zone_contents, *unfilled_zone_import.board, *unfilled_zone_import.board);
  const auto unfilled_zone_reimport =
      unfilled_zone_export.contents ? flux::io::import_kicad_pcb(*unfilled_zone_export.contents)
                                    : flux::io::KicadImportResult{};
  require(unfilled_zone_export.ok() && unfilled_zone_export.contents &&
              unfilled_zone_export.contents->find("unfilled-zone-test") != std::string::npos &&
              unfilled_zone_reimport.strict() && unfilled_zone_reimport.board &&
              std::any_of(unfilled_zone_reimport.board->zones.begin(),
                          unfilled_zone_reimport.board->zones.end(),
                          [](const auto& zone) {
                            return zone.id == ObjectId{"unfilled-zone-test"} &&
                                   zone.requires_refill && zone.fills.empty();
                          }),
          "an unchanged unfilled zone must retain exact source syntax and canonical refill state "
          "through source-preserving export");
  auto named_net_contents = contents;
  const auto erase_once = [&named_net_contents](std::string_view value) {
    const auto position = named_net_contents.find(value);
    require(position != std::string::npos, "named-net fixture source syntax must be identifiable");
    named_net_contents.erase(position, value.size());
  };
  const auto replace_all = [&named_net_contents](std::string_view from, std::string_view to) {
    auto position = named_net_contents.find(from);
    while (position != std::string::npos) {
      named_net_contents.replace(position, from.size(), to);
      position = named_net_contents.find(from, position + to.size());
    }
  };
  erase_once("  (net 0 \"\")\n");
  erase_once("  (net 1 \"SIGNAL\")\n");
  replace_all("(net 1 \"SIGNAL\")", "(net \"SIGNAL\")");
  replace_all("(net 1)", "(net \"SIGNAL\")");
  replace_all("(net 0)", "(net \"\")");
  const auto named_net_import = flux::io::import_kicad_pcb(named_net_contents);
  require(named_net_import.strict() && named_net_import.board &&
              std::any_of(named_net_import.board->nets.begin(), named_net_import.board->nets.end(),
                          [](const auto& net) {
                            return net.id == NetId{"kicad-net-name:SIGNAL"} && net.name == "SIGNAL";
                          }),
          "a KiCad 10 board without an ordinal net table must derive stable named-net identity");
  auto named_net_candidate = *named_net_import.board;
  named_net_candidate.tracks.push_back(
      {ObjectId{"named-net-added-track"}, NetId{"kicad-net-name:SIGNAL"}, LayerId{"B.Cu"},
       mm(25.0, 35.0), mm(30.0, 35.0), Nanometres::from_mm(0.25), false});
  const auto named_net_export = flux::io::export_kicad_candidate(
      named_net_contents, *named_net_import.board, named_net_candidate);
  require(named_net_export.ok() && named_net_export.contents &&
              named_net_export.contents->find("(net \"SIGNAL\")") != std::string::npos,
          "new KiCad 10 copper must retain quoted net-name references through strict round trip");
  auto differential_suffix_contents = contents;
  const auto signal_net = differential_suffix_contents.find("(net 1 \"SIGNAL\")");
  require(signal_net != std::string::npos,
          "differential-suffix fixture must retain the canonical net declaration");
  differential_suffix_contents.replace(
      signal_net, std::string{"(net 1 \"SIGNAL\")"}.size(),
      "(net 1 \"DATA_P\")\n  (net 2 \"DATA_N\")\n  (net 3 \"Net-(U5A-+)\")\n"
      "  (net 4 \"Net-(U5A--)\")");
  const auto differential_suffix_import = flux::io::import_kicad_pcb(differential_suffix_contents);
  require(differential_suffix_import.strict() &&
              differential_suffix_import.board->differential_pairs.size() == 1U &&
              differential_suffix_import.board->differential_pairs.front().positive_net ==
                  NetId{"kicad-net-1"} &&
              differential_suffix_import.board->differential_pairs.front().negative_net ==
                  NetId{"kicad-net-2"},
          "KiCad differential-pair suffix parsing must stop at punctuation and must not couple "
          "autogenerated Net-(...) pin names");
  auto rotated_pad_contents = contents;
  const auto footprint_at = rotated_pad_contents.find("(at 80 40 180)");
  require(footprint_at != std::string::npos, "rotation fixture footprint must remain identifiable");
  rotated_pad_contents.replace(footprint_at, std::string{"(at 80 40 180)"}.size(), "(at 80 40 90)");
  const auto last_pad = rotated_pad_contents.rfind("(pad \"1\" smd circle");
  require(last_pad != std::string::npos, "rotation fixture pad must remain identifiable");
  rotated_pad_contents.replace(last_pad + std::string{"(pad \"1\" smd "}.size(),
                               std::string{"circle"}.size(), "rect");
  const auto last_pad_at = rotated_pad_contents.find("(at 0 0)", last_pad);
  require(last_pad_at != std::string::npos,
          "rotation fixture pad position must remain identifiable");
  rotated_pad_contents.replace(last_pad_at, std::string{"(at 0 0)"}.size(), "(at 0 0 90)");
  const auto last_pad_size = rotated_pad_contents.find("(size 2 2)", last_pad);
  require(last_pad_size != std::string::npos, "rotation fixture pad size must remain identifiable");
  rotated_pad_contents.replace(last_pad_size, std::string{"(size 2 2)"}.size(), "(size 1 3)");
  const auto rotated_pad_import = flux::io::import_kicad_pcb(rotated_pad_contents);
  require(rotated_pad_import.strict() &&
              rotated_pad_import.board->footprints.back().pads.front().rotation == Microdegrees{} &&
              flux::point_to_pad_copper_distance_nm(
                  mm(81.4, 40.0), flux::copper_pad_shapes(*rotated_pad_import.board).back()) ==
                  0.0L,
          "a KiCad board-file pad angle must be converted to local orientation before the "
          "footprint transform is applied exactly once");
  auto oval_drill_contents = contents;
  const auto oval_pad = oval_drill_contents.rfind("(pad \"1\" smd circle");
  require(oval_pad != std::string::npos,
          "oval-drill fixture pad declaration must remain identifiable");
  oval_drill_contents.replace(oval_pad + std::string{"(pad \"1\" "}.size(),
                              std::string{"smd circle"}.size(), "np_thru_hole oval");
  const auto oval_layers =
      oval_drill_contents.find("(layers \"F.Cu\" \"F.Paste\" \"F.Mask\")", oval_pad);
  require(oval_layers != std::string::npos,
          "oval-drill fixture pad layers must remain identifiable");
  oval_drill_contents.replace(
      oval_layers, std::string{"(layers \"F.Cu\" \"F.Paste\" \"F.Mask\")"}.size(),
      "(drill oval 0.6 1.4 (offset 0.2 0))\n      (layers \"*.Cu\" \"*.Mask\")");
  const auto oval_drill_import = flux::io::import_kicad_pcb(oval_drill_contents);
  const auto oval_drill_shapes = oval_drill_import.board
                                     ? flux::copper_pad_shapes(*oval_drill_import.board)
                                     : std::vector<flux::CopperPadShape>{};
  require(
      oval_drill_import.strict() && oval_drill_import.board &&
          !oval_drill_import.board->footprints.back().pads.front().plated &&
          oval_drill_import.board->footprints.back().pads.front().drill_width ==
              Nanometres::from_mm(0.6) &&
          oval_drill_import.board->footprints.back().pads.front().drill_height ==
              Nanometres::from_mm(1.4) &&
          oval_drill_import.board->footprints.back().pads.front().drill_offset == mm(-0.2, 0.0) &&
          !oval_drill_shapes.empty() && oval_drill_shapes.back().shape == flux::PadShape::oval &&
          oval_drill_shapes.back().width == Nanometres::from_mm(0.6) &&
          oval_drill_shapes.back().height == Nanometres::from_mm(1.4) &&
          oval_drill_shapes.back().position == mm(80.0, 40.0),
      "oval and offset non-plated drill geometry must retain its exact local dimensions and "
      "world-space routing obstacle");
  const auto priority_template = flux::io::make_placement_move_priorities_template(*imported.board);
  const auto priorities =
      flux::io::load_placement_move_priorities(*imported.board, priority_template);
  require(priorities.ok() && priorities.policy && priorities.policy->footprints.size() == 2U &&
              std::all_of(
                  priorities.policy->footprints.begin(), priorities.policy->footprints.end(),
                  [](const auto& assignment) {
                    return assignment.priority_class == flux::PlacementMovePriorityClass::connector;
                  }),
          "the source-bound priority template must completely classify J prefixes connector-last");
  const auto suggested_priorities =
      flux::io::make_suggested_placement_move_priorities(*imported.board);
  require(suggested_priorities.source_revision == imported.board->source_revision &&
              suggested_priorities.order == flux::default_placement_move_priority_order() &&
              suggested_priorities.footprints == priorities.policy->footprints,
          "in-memory prefix suggestions must match the deterministic review template exactly");
  auto stale_priority_board = *imported.board;
  stale_priority_board.source_revision += ":stale";
  require(flux::io::load_placement_move_priorities(stale_priority_board, priority_template)
              .has_errors(),
          "a placement-move priority sidecar must reject a stale source revision");
  flux::IntegratedOptimizerConfig stale_priority_config;
  stale_priority_config.placement_move_priorities = *priorities.policy;
  stale_priority_config.placement_move_priorities->source_revision += ":stale";
  const auto stale_priority_result =
      flux::optimize_integrated_placement_routing(*imported.board, stale_priority_config);
  require(
      stale_priority_result.stop_reason == flux::IntegratedOptimizerStopReason::invalid_config &&
          stale_priority_result.diagnostic.find("different source revision") != std::string::npos,
      "the core optimizer must retain and enforce the policy's source binding");
  auto moved = *imported.board;
  moved.footprints.front().position = mm(25.0, 40.0);
  require(flux::io::export_kicad_candidate(contents, *imported.board, moved).ok(),
          "a supported source-bound move must export");

  auto lossy_contents = contents;
  const auto keepout_start = lossy_contents.find("(keepout");
  const auto keepout_end = lossy_contents.find("\n    )", keepout_start);
  require(keepout_start != std::string::npos && keepout_end != std::string::npos,
          "KiCad fixture keepout must remain identifiable");
  auto keepout_rule = lossy_contents.find("not_allowed", keepout_start);
  while (keepout_rule != std::string::npos && keepout_rule < keepout_end) {
    lossy_contents.replace(keepout_rule, std::string{"not_allowed"}.size(), "allowed");
    keepout_rule = lossy_contents.find("not_allowed", keepout_rule + 1U);
  }
  const auto lossy_import = flux::io::import_kicad_pcb(lossy_contents);
  require(lossy_import.board && !lossy_import.strict(),
          "empty keepout syntax must retain a non-strict canonical projection: board=" +
              std::to_string(lossy_import.board.has_value()) +
              ", strict=" + std::to_string(lossy_import.strict()) +
              ", lossy=" + std::to_string(lossy_import.lossy) +
              ", diagnostics=" + std::to_string(lossy_import.diagnostics.size()));
  const auto qualified_lossy_export =
      flux::io::export_kicad_candidate(lossy_contents, *lossy_import.board, *lossy_import.board);
  require(!qualified_lossy_export.ok() && std::any_of(qualified_lossy_export.diagnostics.begin(),
                                                      qualified_lossy_export.diagnostics.end(),
                                                      [](const auto& diagnostic) {
                                                        return diagnostic.code ==
                                                               "source_not_strict";
                                                      }),
          "ordinary export must continue to reject a lossy source projection");
  flux::io::KicadExportOptions preview_options;
  preview_options.verify_round_trip = false;
  preview_options.allow_lossy_diagnostic_preview = true;
  const auto diagnostic_lossy_export = flux::io::export_kicad_candidate(
      lossy_contents, *lossy_import.board, *lossy_import.board, preview_options);
  require(diagnostic_lossy_export.ok() &&
              std::any_of(diagnostic_lossy_export.diagnostics.begin(),
                          diagnostic_lossy_export.diagnostics.end(),
                          [](const auto& diagnostic) {
                            return diagnostic.code == "lossy_diagnostic_preview" &&
                                   diagnostic.severity == flux::io::ExportSeverity::warning;
                          }),
          "explicit diagnostic export must write a visibly non-qualified lossy artifact");

  auto invalid_contents = contents;
  const auto first_footprint = invalid_contents.find("(footprint");
  const auto first_pad_size = invalid_contents.find("(size 2 2)", first_footprint);
  require(first_pad_size != std::string::npos, "KiCad fixture pad must remain identifiable");
  invalid_contents.replace(first_pad_size, std::string{"(size 2 2)"}.size(), "(size 0 2)");
  const auto invalid_import = flux::io::import_kicad_pcb(invalid_contents);
  require(invalid_import.strict() && invalid_import.board &&
              !flux::validate_board(*invalid_import.board).ok(),
          "zero-width imported pad must isolate canonical structural validation");
  const auto qualified_invalid_export = flux::io::export_kicad_candidate(
      invalid_contents, *invalid_import.board, *invalid_import.board);
  require(!qualified_invalid_export.ok() &&
              std::any_of(qualified_invalid_export.diagnostics.begin(),
                          qualified_invalid_export.diagnostics.end(),
                          [](const auto& diagnostic) {
                            return diagnostic.code == "invalid_source_snapshot";
                          }),
          "ordinary export must continue to reject a structurally invalid projection");
  const auto diagnostic_invalid_export = flux::io::export_kicad_candidate(
      invalid_contents, *invalid_import.board, *invalid_import.board, preview_options);
  require(diagnostic_invalid_export.ok() &&
              std::any_of(diagnostic_invalid_export.diagnostics.begin(),
                          diagnostic_invalid_export.diagnostics.end(),
                          [](const auto& diagnostic) {
                            return diagnostic.code == "invalid_diagnostic_preview" &&
                                   diagnostic.severity == flux::io::ExportSeverity::warning;
                          }),
          "diagnostic export must preserve structural-invalid evidence without suppressing bytes");

  const auto stale = flux::io::export_kicad_candidate(contents + "\n", *imported.board, moved);
  require(!stale.ok() && std::any_of(stale.diagnostics.begin(), stale.diagnostics.end(),
                                     [](const auto& diagnostic) {
                                       return diagnostic.code == "stale_source_snapshot";
                                     }),
          "changed source bytes must reject a stale candidate");

  auto unsupported = *imported.board;
  unsupported.outline.vertices.front() = mm(1.0, 1.0);
  const auto rejected = flux::io::export_kicad_candidate(contents, *imported.board, unsupported);
  require(!rejected.ok() && std::any_of(rejected.diagnostics.begin(), rejected.diagnostics.end(),
                                        [](const auto& diagnostic) {
                                          return diagnostic.code == "unsupported_design_change";
                                        }),
          "unsupported board changes must be rejected rather than silently dropped");

  const auto tentative_contents = load_tentative_placement_fixture();
  flux::io::KicadImportOptions tentative_options;
  tentative_options.derive_tentative_placement_envelopes = true;
  const auto tentative = flux::io::import_kicad_pcb(tentative_contents, tentative_options);
  require(
      tentative.strict() && tentative.board && tentative.board->footprints.size() == 2U &&
          std::all_of(tentative.board->footprints.begin(), tentative.board->footprints.end(),
                      [](const Footprint& footprint) {
                        return footprint.has_tentative_placement_envelope() &&
                               !footprint.has_exact_placement_envelope();
                      }) &&
          tentative.board->footprints.front().courtyard.vertices.front() == mm(-5.25, -4.25),
      "opt-in tentative geometry must include Fab/Silk bounds and retain provisional provenance");

  flux::IntegratedOptimizerConfig tentative_config;
  const auto tentative_rejected =
      flux::optimize_integrated_placement_routing(*tentative.board, tentative_config);
  require(!tentative_rejected.complete() &&
              tentative_rejected.diagnostic.find("explicit tentative envelope opt-in") !=
                  std::string::npos,
          "integrated placement must reject tentative geometry by default");

  tentative_config.allow_tentative_placement_envelopes = true;
  tentative_config.placement_move_priorities =
      flux::io::make_suggested_placement_move_priorities(*tentative.board);
  tentative_config.suggested_placement_move_priorities = true;
  tentative_config.enable_global_placement = true;
  tentative_config.max_passes = 1U;
  tentative_config.max_candidate_evaluations = 2U;
  const auto tentative_accepted =
      flux::optimize_integrated_placement_routing(*tentative.board, tentative_config);
  require(tentative_accepted.complete() && tentative_accepted.tentative_placement_envelopes == 2U &&
              tentative_accepted.human_placement_review_required &&
              tentative_accepted.configured_placement_move_priorities &&
              tentative_accepted.suggested_placement_move_priorities &&
              tentative_accepted.placement_move_priority_review_required &&
              tentative_accepted.configured_max_passes == 1U &&
              tentative_accepted.configured_max_candidate_evaluations == 2U,
          "explicit tentative placement must remain feasible and require post-process review");

  auto overlapping_tentative = *tentative.board;
  overlapping_tentative.footprints.back().position =
      overlapping_tentative.footprints.front().position;
  require(!flux::validate_board(overlapping_tentative).ok(),
          "the tentative legalization fixture must begin with an illegal overlap");
  flux::GlobalPlacerConfig tentative_placer_config;
  tentative_placer_config.iterations = 1U;
  const auto overlapping_rejected =
      flux::place_global(overlapping_tentative, tentative_placer_config);
  require(!overlapping_rejected.complete(),
          "global placement must reject an invalid tentative source without explicit opt-in");
  tentative_placer_config.allow_tentative_placement_envelopes = true;
  auto structurally_invalid_tentative = overlapping_tentative;
  structurally_invalid_tentative.footprints.front().pads.front().net = flux::NetId{"missing-net"};
  const auto structurally_invalid_result =
      flux::place_global(structurally_invalid_tentative, tentative_placer_config);
  require(!structurally_invalid_result.complete() &&
              !structurally_invalid_result.repairable_tentative_source_accepted,
          "tentative source legalization must not whitelist an unrelated structural error");
  const auto overlapping_legalized =
      flux::place_global(overlapping_tentative, tentative_placer_config);
  require(overlapping_legalized.complete() && overlapping_legalized.candidate &&
              overlapping_legalized.repairable_tentative_source_accepted &&
              overlapping_legalized.repairable_tentative_source_errors > 0U &&
              flux::validate_board(*overlapping_legalized.candidate).ok(),
          "explicit tentative placement must repair source overlap before publishing a proposal");

  auto tentatively_moved = *tentative.board;
  tentatively_moved.footprints.front().position = mm(25.0, 40.0);
  require(flux::io::export_kicad_candidate(tentative_contents, *tentative.board, tentatively_moved)
              .ok(),
          "non-serialized tentative geometry must round-trip against the unchanged KiCad source");
}

void routing_xpbd_joint_relaxation_preserves_endpoints_and_reports_stress() {
  const flux::RegionalRouterConfig production_regional_defaults;
  require(production_regional_defaults.enable_xpbd_relaxation &&
              production_regional_defaults.enable_xpbd_feedback,
          "normal regional routing must run XPBD repair and publish its upstream feedback by "
          "default");
  require(production_regional_defaults.max_spline_exact_candidate_workers == 4U,
          "normal regional routing must evaluate exact local-repair moves in parallel");
  const std::vector<flux::RoutingXpbdParticle> particles{
      {-2'000'000, -1'000'000, 200'000, 0.0L, 0U, 0U, flux::routing_xpbd_endpoint, -2'000'000,
       -1'000'000, 100'000},
      {0, 0, 200'000, 1.0L, 0U, 0U, 0U, 0, 0, 1'200'000},
      {2'000'000, -1'000'000, 200'000, 0.0L, 0U, 0U, flux::routing_xpbd_endpoint, 2'000'000,
       -1'000'000, 100'000},
      {-2'000'000, 1'000'000, 200'000, 0.0L, 1U, 0U, flux::routing_xpbd_endpoint, -2'000'000,
       1'000'000, 100'000},
      {0, 0, 200'000, 1.0L, 1U, 0U, 0U, 0, 0, 1'200'000},
      {2'000'000, 1'000'000, 200'000, 0.0L, 1U, 0U, flux::routing_xpbd_endpoint, 2'000'000,
       1'000'000, 100'000}};
  const std::vector<flux::RoutingXpbdSegment> segments{{0U, 1U, 0U, 0U, 200'000, 2'236'068},
                                                       {1U, 2U, 0U, 0U, 200'000, 2'236'068},
                                                       {3U, 4U, 1U, 0U, 200'000, 2'236'068},
                                                       {4U, 5U, 1U, 0U, 200'000, 2'236'068}};
  flux::RoutingXpbdConfig config;
  config.iterations = 64U;
  config.maximum_step_nm = 100'000;
  config.bend_compliance = 0.01L;
  config.pressure_width = 32U;
  config.pressure_height = 32U;
  config.pressure_origin_x_nm = -4'000'000;
  config.pressure_origin_y_nm = -4'000'000;
  config.pressure_pitch_nm = 250'000;
  config.record_iteration_metrics = true;
  config.prefer_gpu = false;
  const auto cpu = flux::relax_routing_xpbd(particles, segments, {}, {}, config);
  config.prefer_gpu = true;
  const auto gpu = flux::relax_routing_xpbd(particles, segments, {}, {}, config);
  require(cpu.ok() && gpu.ok() &&
              cpu.positions_nm.front() ==
                  std::pair<std::int64_t, std::int64_t>{-2'000'000, -1'000'000} &&
              cpu.positions_nm[2] == std::pair<std::int64_t, std::int64_t>{2'000'000, -1'000'000} &&
              cpu.final.total_residual_nm < cpu.initial.total_residual_nm &&
              cpu.constraints.stretch == segments.size() && cpu.constraints.bend == 2U &&
              cpu.constraints.trace_clearance > 0U && cpu.converged &&
              cpu.pressure_field.size() == 32U * 32U &&
              gpu.positions_nm.front() == cpu.positions_nm.front() &&
              gpu.positions_nm[2] == cpu.positions_nm[2] &&
              gpu.positions_nm[1] != gpu.positions_nm[4] &&
              std::abs(gpu.positions_nm[1].first) < 4'000'000 &&
              std::abs(gpu.positions_nm[1].second) < 4'000'000 &&
              std::abs(gpu.positions_nm[4].first) < 4'000'000 &&
              std::abs(gpu.positions_nm[4].second) < 4'000'000 &&
              gpu.initial.total_residual_nm > 0.0L &&
              gpu.final.total_residual_nm <= gpu.initial.total_residual_nm && gpu.converged &&
              gpu.constraints.trace_clearance == gpu.candidate_pairs && gpu.timings.solve_us > 0U &&
              (!gpu.used_gpu ||
               (gpu.used_gpu_broadphase && !gpu.broadphase_fallback && gpu.broadphase_cells > 0U &&
                gpu.broadphase_pair_emissions > 0U && gpu.used_gpu_pressure_field &&
                gpu.pressure_field.size() == 32U * 32U &&
                gpu.iteration_metrics.size() == config.iterations &&
                gpu.particle_stress_by_type.size() == particles.size() &&
                gpu.stress.trace_clearance_nm > 0.0L)),
          "joint XPBD relaxation must keep terminal particles fixed, reduce inter-route stress, "
          "publish bounded pressure, and preserve endpoint semantics on CPU and Vulkan paths: " +
              cpu.diagnostic + "/" + gpu.diagnostic + "; cpu=" +
              std::to_string(cpu.initial.violations) + "->" + std::to_string(cpu.final.violations) +
              ", " + std::to_string(static_cast<double>(cpu.final.maximum_residual_nm)) + "; gpu=" +
              std::to_string(gpu.initial.violations) + "->" + std::to_string(gpu.final.violations) +
              ", " + std::to_string(static_cast<double>(gpu.final.maximum_residual_nm)));

  const auto require_trace_pack = [&](std::string_view name,
                                      const std::vector<std::int64_t>& endpoint_y,
                                      const std::vector<std::int64_t>& interior_y) {
    std::vector<flux::RoutingXpbdParticle> pack_particles;
    std::vector<flux::RoutingXpbdSegment> pack_segments;
    for (std::size_t route = 0U; route < endpoint_y.size(); ++route) {
      const auto first = static_cast<std::uint32_t>(pack_particles.size());
      for (std::size_t control = 0U; control < 5U; ++control) {
        const auto x = -2'000'000 + static_cast<std::int64_t>(control) * 1'000'000;
        const auto endpoint = control == 0U || control == 4U;
        const auto y = endpoint ? endpoint_y[route] : interior_y[route];
        pack_particles.push_back({x, y, 100'000, endpoint ? 0.0L : 1.0L,
                                  static_cast<std::uint32_t>(route), 0U,
                                  endpoint ? flux::routing_xpbd_endpoint : 0U, x, y, 1'500'000});
        if (control > 0U) {
          pack_segments.push_back({first + static_cast<std::uint32_t>(control - 1U),
                                   first + static_cast<std::uint32_t>(control),
                                   static_cast<std::uint32_t>(route), 0U, 100'000, 1'000'000,
                                   100'000});
        }
      }
    }
    auto pack_config = config;
    pack_config.iterations = 512U;
    pack_config.maximum_step_nm = 100'000;
    pack_config.prefer_gpu = false;
    const auto cpu_pack =
        flux::relax_routing_xpbd(pack_particles, pack_segments, {}, {}, pack_config);
    pack_config.prefer_gpu = true;
    const auto gpu_pack =
        flux::relax_routing_xpbd(pack_particles, pack_segments, {}, {}, pack_config);
    require(cpu_pack.ok() && gpu_pack.ok() && cpu_pack.initial.violations > 0U &&
                gpu_pack.initial.violations > 0U && cpu_pack.converged && gpu_pack.converged,
            "joint XPBD trace packing must resolve " + std::string(name) +
                ": cpu=" + std::to_string(cpu_pack.initial.violations) + "->" +
                std::to_string(cpu_pack.final.violations) +
                ", gpu=" + std::to_string(gpu_pack.initial.violations) + "->" +
                std::to_string(gpu_pack.final.violations));
  };
  require_trace_pack("parallel traces that are too close", {-500'000, 500'000},
                     {-100'000, 100'000});
  require_trace_pack("three traces competing in one neck", {-700'000, 0, 700'000},
                     {-150'000, 0, 150'000});

  const std::vector<flux::RoutingXpbdParticle> obstacle_particles{
      {-2'000'000, 0, 100'000, 0.0L, 0U, 0U, flux::routing_xpbd_endpoint, -2'000'000, 0, 1'500'000},
      {-1'000'000, 0, 100'000, 1.0L, 0U, 0U, 0U, -1'000'000, 0, 1'500'000},
      {0, 0, 100'000, 1.0L, 0U, 0U, 0U, 0, 0, 1'500'000},
      {1'000'000, 0, 100'000, 1.0L, 0U, 0U, 0U, 1'000'000, 0, 1'500'000},
      {2'000'000, 0, 100'000, 0.0L, 0U, 0U, flux::routing_xpbd_endpoint, 2'000'000, 0, 1'500'000}};
  const std::vector<flux::RoutingXpbdSegment> obstacle_route{
      {0U, 1U, 0U, 0U, 100'000, 1'000'000, 100'000},
      {1U, 2U, 0U, 0U, 100'000, 1'000'000, 100'000},
      {2U, 3U, 0U, 0U, 100'000, 1'000'000, 100'000},
      {3U, 4U, 0U, 0U, 100'000, 1'000'000, 100'000}};
  const std::vector<flux::RoutingXpbdObstacleSegment> obstacle{
      {0, -500'000, 0, 500'000, 200'000, 0U}};
  config.prefer_gpu = false;
  config.iterations = 96U;
  config.bend_compliance = 0.05L;
  const auto obstacle_cpu =
      flux::relax_routing_xpbd(obstacle_particles, obstacle_route, obstacle, {}, config);
  config.prefer_gpu = true;
  const auto obstacle_gpu =
      flux::relax_routing_xpbd(obstacle_particles, obstacle_route, obstacle, {}, config);
  require(obstacle_cpu.ok() && obstacle_gpu.ok() && obstacle_cpu.converged &&
              obstacle_gpu.converged &&
              (!flux::routing_compute_gpu_available() ||
               (obstacle_gpu.used_gpu && obstacle_gpu.stress.obstacle_nm > 0.0L)) &&
              obstacle_cpu.final.total_residual_nm < obstacle_cpu.initial.total_residual_nm,
          "CPU and Vulkan XPBD segment/obstacle constraints must remove a finite static-geometry "
          "penetration and attribute obstacle stress: " +
              obstacle_cpu.diagnostic + "/" + obstacle_gpu.diagnostic +
              "; violations=" + std::to_string(obstacle_cpu.final.violations) + "/" +
              std::to_string(obstacle_gpu.final.violations));

  // One bounded synthetic matrix covers the risky static-geometry shapes from Stage B without
  // proliferating independent unit fixtures: rectangular penetration, narrow passage, concave
  // corner, and a fixed-pad escape whose first movable controls must bend around a blocker.
  const auto require_static_case =
      [&](std::string_view name, const std::vector<flux::RoutingXpbdParticle>& case_particles,
          const std::vector<flux::RoutingXpbdSegment>& case_segments,
          const std::vector<flux::RoutingXpbdObstacleSegment>& case_obstacles) {
        auto case_config = config;
        case_config.iterations = 1'024U;
        case_config.maximum_step_nm = 100'000;
        case_config.bend_compliance = 0.05L;
        case_config.prefer_gpu = false;
        const auto cpu_case = flux::relax_routing_xpbd(case_particles, case_segments,
                                                       case_obstacles, {}, case_config);
        case_config.prefer_gpu = true;
        const auto gpu_case = flux::relax_routing_xpbd(case_particles, case_segments,
                                                       case_obstacles, {}, case_config);
        require(cpu_case.ok() && gpu_case.ok() && cpu_case.converged && gpu_case.converged &&
                    cpu_case.final.total_residual_nm < cpu_case.initial.total_residual_nm &&
                    gpu_case.final.total_residual_nm < gpu_case.initial.total_residual_nm,
                "XPBD static-geometry case must converge on CPU and Vulkan: " + std::string(name) +
                    "; cpu=" + std::to_string(cpu_case.initial.violations) + "->" +
                    std::to_string(cpu_case.final.violations) +
                    ", gpu=" + std::to_string(gpu_case.initial.violations) + "->" +
                    std::to_string(gpu_case.final.violations) + ", residual=" +
                    std::to_string(static_cast<double>(cpu_case.final.maximum_residual_nm)) + "/" +
                    std::to_string(static_cast<double>(gpu_case.final.maximum_residual_nm)) +
                    ", controls=" + std::to_string(cpu_case.positions_nm[1].first) + ":" +
                    std::to_string(cpu_case.positions_nm[1].second) + "," +
                    std::to_string(cpu_case.positions_nm[2].first) + ":" +
                    std::to_string(cpu_case.positions_nm[2].second) + "," +
                    std::to_string(cpu_case.positions_nm[3].first) + ":" +
                    std::to_string(cpu_case.positions_nm[3].second) +
                    "; gpu_controls=" + std::to_string(gpu_case.positions_nm[1].first) + ":" +
                    std::to_string(gpu_case.positions_nm[1].second) + "," +
                    std::to_string(gpu_case.positions_nm[2].first) + ":" +
                    std::to_string(gpu_case.positions_nm[2].second) + "," +
                    std::to_string(gpu_case.positions_nm[3].first) + ":" +
                    std::to_string(gpu_case.positions_nm[3].second) +
                    "; diagnostics=" + cpu_case.diagnostic + "/" + gpu_case.diagnostic);
      };
  const std::vector<flux::RoutingXpbdObstacleSegment> rectangle_obstacle{
      {-300'000, -500'000, 300'000, -500'000, 0, 0U, std::numeric_limits<std::uint32_t>::max(), 0,
       -600'000},
      {300'000, -500'000, 300'000, 500'000, 0, 0U, std::numeric_limits<std::uint32_t>::max(),
       1'000'000, 0},
      {300'000, 500'000, -300'000, 500'000, 0, 0U, std::numeric_limits<std::uint32_t>::max(), 0,
       600'000},
      {-300'000, 500'000, -300'000, -500'000, 0, 0U, std::numeric_limits<std::uint32_t>::max(),
       -1'000'000, 0}};
  require_static_case("rectangular penetration", obstacle_particles, obstacle_route,
                      rectangle_obstacle);
  auto passage_particles = obstacle_particles;
  passage_particles[1].y_nm = 250'000;
  passage_particles[2].y_nm = 250'000;
  passage_particles[3].y_nm = 250'000;
  const std::vector<flux::RoutingXpbdObstacleSegment> passage_obstacles{
      {-1'500'000, 450'000, 1'500'000, 450'000, 0, 0U},
      {-1'500'000, -450'000, 1'500'000, -450'000, 0, 0U}};
  require_static_case("narrow passage", passage_particles, obstacle_route, passage_obstacles);
  const std::vector<flux::RoutingXpbdObstacleSegment> concave_obstacle{
      {-250'000, -700'000, -250'000, 500'000, 0, 0U, std::numeric_limits<std::uint32_t>::max(),
       -1'200'000, 0},
      {-250'000, 500'000, 600'000, 500'000, 0, 0U, std::numeric_limits<std::uint32_t>::max(), 0,
       850'000},
      {600'000, 500'000, 600'000, 100'000, 0, 0U, std::numeric_limits<std::uint32_t>::max(),
       400'000, 0}};
  require_static_case("concave corner", obstacle_particles, obstacle_route, concave_obstacle);
  const std::vector<flux::RoutingXpbdObstacleSegment> pad_escape_obstacle{
      {-1'150'000, -400'000, -1'150'000, 400'000, 0, 0U, std::numeric_limits<std::uint32_t>::max(),
       0, 800'000}};
  require_static_case("fixed-pad escape", obstacle_particles, obstacle_route, pad_escape_obstacle);

  const std::vector<flux::RoutingXpbdParticle> terrain_particles{
      {-2'000'000, 0, 100'000, 0.0L, 0U, 0U, flux::routing_xpbd_endpoint, -2'000'000, 0, 100'000},
      {0, 0, 100'000, 1.0L, 0U, 0U, 0U, 0, 0, 2'000'000},
      {2'000'000, 0, 100'000, 0.0L, 0U, 0U, flux::routing_xpbd_endpoint, 2'000'000, 0, 100'000}};
  const std::vector<flux::RoutingXpbdSegment> terrain_segments{
      {0U, 1U, 0U, 0U, 100'000, 2'000'000, 100'000}, {1U, 2U, 0U, 0U, 100'000, 2'000'000, 100'000}};
  flux::RoutingXpbdTerrainField terrain;
  terrain.width = 9U;
  terrain.height = 9U;
  terrain.origin_x_nm = -4'000'000;
  terrain.origin_y_nm = -4'000'000;
  terrain.pitch_nm = 1'000'000;
  terrain.pressure.resize(terrain.width * terrain.height);
  for (std::size_t y = 0U; y < terrain.height; ++y) {
    for (std::size_t x = 0U; x < terrain.width; ++x)
      terrain.pressure[y * terrain.width + x] = static_cast<float>(y * 10U);
  }
  config.iterations = 32U;
  config.maximum_step_nm = 50'000;
  config.terrain_gain = 0.05L;
  config.prefer_gpu = false;
  const auto terrain_cpu =
      flux::relax_routing_xpbd(terrain_particles, terrain_segments, {}, terrain, config);
  config.prefer_gpu = true;
  const auto terrain_gpu =
      flux::relax_routing_xpbd(terrain_particles, terrain_segments, {}, terrain, config);
  require(terrain_cpu.ok() && terrain_gpu.ok() && terrain_cpu.positions_nm[1].second < 0 &&
              terrain_gpu.positions_nm[1].second < 0 &&
              terrain_cpu.positions_nm.front() == terrain_gpu.positions_nm.front(),
          "CPU and Vulkan XPBD must sample the same routing-terrain gradient without moving fixed "
          "terminals");

  auto layered_particles = terrain_particles;
  for (const auto& particle : terrain_particles) {
    auto copy = particle;
    copy.route = 1U;
    copy.layer = 1U;
    layered_particles.push_back(copy);
  }
  auto layered_segments = terrain_segments;
  for (const auto& segment : terrain_segments) {
    auto copy = segment;
    copy.first_particle += 3U;
    copy.second_particle += 3U;
    copy.route = 1U;
    copy.layer = 1U;
    layered_segments.push_back(copy);
  }
  auto layered_terrain = terrain;
  layered_terrain.layer_count = 2U;
  const auto terrain_plane = layered_terrain.width * layered_terrain.height;
  layered_terrain.pressure.assign(terrain_plane * layered_terrain.layer_count, 0.0F);
  for (std::size_t y = 0U; y < layered_terrain.height; ++y) {
    for (std::size_t x = 0U; x < layered_terrain.width; ++x)
      layered_terrain.pressure[y * layered_terrain.width + x] = static_cast<float>(y * 10U);
  }
  config.prefer_gpu = false;
  const auto layered_cpu =
      flux::relax_routing_xpbd(layered_particles, layered_segments, {}, layered_terrain, config);
  config.prefer_gpu = true;
  const auto layered_gpu =
      flux::relax_routing_xpbd(layered_particles, layered_segments, {}, layered_terrain, config);
  const auto layered_gpu_repeated =
      flux::relax_routing_xpbd(layered_particles, layered_segments, {}, layered_terrain, config);
  require(layered_cpu.ok() && layered_gpu.ok() && layered_gpu_repeated.ok() &&
              layered_cpu.positions_nm[1].second < 0 && layered_cpu.positions_nm[4].second == 0 &&
              layered_gpu.positions_nm[1].second < 0 && layered_gpu.positions_nm[4].second == 0 &&
              layered_cpu.pressure_layer_count == 2U && layered_gpu.pressure_layer_count == 2U &&
              layered_cpu.pressure_field.size() == 32U * 32U * 2U &&
              layered_gpu.pressure_field.size() == 32U * 32U * 2U &&
              (!flux::routing_compute_gpu_available() ||
               (layered_gpu.used_gpu && layered_gpu.used_gpu_pressure_field &&
                layered_gpu_repeated.resident_input_buffer_reuses >= 4U)),
          "XPBD terrain, stress diffusion, and resident static inputs must remain isolated by "
          "copper layer on CPU and Vulkan paths: cpu_y=" +
              std::to_string(layered_cpu.positions_nm[1].second) + "/" +
              std::to_string(layered_cpu.positions_nm[4].second) +
              ", gpu_y=" + std::to_string(layered_gpu.positions_nm[1].second) + "/" +
              std::to_string(layered_gpu.positions_nm[4].second) +
              ", layers=" + std::to_string(layered_cpu.pressure_layer_count) + "/" +
              std::to_string(layered_gpu.pressure_layer_count) +
              ", cells=" + std::to_string(layered_cpu.pressure_field.size()) + "/" +
              std::to_string(layered_gpu.pressure_field.size()) +
              ", reuse=" + std::to_string(layered_gpu_repeated.resident_input_buffer_reuses) +
              ", diag=" + layered_cpu.diagnostic + "/" + layered_gpu.diagnostic + "/" +
              layered_gpu_repeated.diagnostic);

  flux::RegionalRouteResult feedback;
  feedback.configured.enable_xpbd_feedback = true;
  flux::RegionalRoutingWindow feedback_window;
  feedback_window.id = 7U;
  feedback_window.bounds = {mm(40.0, 35.0), mm(60.0, 45.0)};
  feedback_window.layers = {LayerId{"F.Cu"}};
  feedback_window.nets = {NetId{"net-1"}, NetId{"n-b"}};
  feedback.windows.push_back(feedback_window);
  flux::RegionalXpbdInvocation feedback_invocation;
  feedback_invocation.window_id = feedback_window.id;
  feedback_invocation.final_violations = 2U;
  feedback_invocation.final_peak_residual = Nanometres{75'000};
  feedback_invocation.topology_escape_required = true;
  feedback.xpbd_invocations.push_back(feedback_invocation);
  feedback.local_routability_diagnoses.push_back(flux::RoutabilityDiagnosis{
      flux::RoutabilityFailureKind::shove_deadlock, flux::RoutabilityFailureStage::local_pns,
      flux::RoutabilityResourceReference{"regional-window:7:xpbd-cell:2:3",
                                         LayerId{"F.Cu"},
                                         {mm(40.0, 35.0), mm(60.0, 45.0)},
                                         feedback_window.nets},
      100U, Nanometres{75'000}, std::nullopt, std::nullopt,
      flux::RoutabilityRelaxation::change_corridor, "diffused XPBD constraint-pressure cell", false,
      true});
  const auto feedback_diagnoses = flux::diagnose_regional_routability(feedback);
  require(std::any_of(feedback_diagnoses.begin(), feedback_diagnoses.end(),
                      [](const auto& diagnosis) {
                        return diagnosis.stage == flux::RoutabilityFailureStage::local_pns &&
                               diagnosis.suggested_relaxation ==
                                   flux::RoutabilityRelaxation::change_corridor;
                      }),
          "an exhausted XPBD solve must request a discrete corridor/topology change instead of "
          "another equivalent local shove");
  flux::RoutabilityKnowledge knowledge;
  flux::accumulate_routability_knowledge(knowledge, feedback_diagnoses);
  require(std::any_of(knowledge.history.begin(), knowledge.history.end(),
                      [](const auto& entry) {
                        return entry.kind == flux::RoutabilityFailureKind::shove_deadlock &&
                               entry.resource.bounds.contains(mm(50.0, 40.0)) &&
                               entry.pressure > 0U;
                      }),
          "diffused XPBD stress must enter the existing bounded spatial routability history");
  GlobalRouterConfig baseline_config;
  baseline_config.grid_pitch = Nanometres::from_mm(5.0);
  baseline_config.max_iterations = 1U;
  const auto baseline_route = flux::route_global(two_pad_board(), baseline_config);
  auto pressured_config = baseline_config;
  pressured_config.routability_history = knowledge.history;
  const auto pressured_route = flux::route_global(two_pad_board(), pressured_config);
  const auto pressure_region = rectangle(40.0, 35.0, 60.0, 45.0);
  const auto pressure_region_crossings = [&](const flux::GlobalRouteResult& route) {
    std::size_t crossings = 0U;
    for (const auto& path : route.paths) {
      for (std::size_t point = 1U; point < path.points.size(); ++point) {
        if (path.points[point - 1U].layer == path.points[point].layer &&
            flux::geometry::segment_polygon_copper_distance_nm(path.points[point - 1U].position,
                                                               path.points[point].position,
                                                               pressure_region) <= 0.0L) {
          ++crossings;
        }
      }
    }
    return crossings;
  };
  const auto baseline_pressure_crossings = pressure_region_crossings(baseline_route);
  const auto pressured_pressure_crossings = pressure_region_crossings(pressured_route);
  require(baseline_route.unrouted_nets.empty() && pressured_route.unrouted_nets.empty() &&
              baseline_route.paths != pressured_route.paths &&
              baseline_pressure_crossings > pressured_pressure_crossings &&
              pressured_route.routability_history_edges > 0U &&
              pressured_route.routability_history_pressure >= 100U,
          "diffused XPBD pressure must alter a subsequent connected PathFinder route while "
          "moving it away from the stressed region and remaining a non-authoritative cost: " +
              std::to_string(baseline_pressure_crossings) + "->" +
              std::to_string(pressured_pressure_crossings));
}

void anisotropic_hydraulic_erosion_is_directional_and_gpu_equivalent() {
  constexpr std::size_t width = 21U;
  constexpr std::size_t height = 11U;
  flux::HydraulicPathInput input;
  input.width = width;
  input.height = height;
  input.walkable.assign(width * height, 1U);
  input.obstacle_clearance.assign(width * height, 8U);
  input.source = 5U * width + 2U;
  input.target = 5U * width + 18U;

  flux::HydraulicPathConfig isotropic_config;
  isotropic_config.prefer_gpu = false;
  isotropic_config.relaxation_iterations = 64U;
  isotropic_config.droplet_batches = 2U;
  isotropic_config.droplets_per_batch = 128U;
  isotropic_config.maximum_steps_per_droplet = 64U;
  const auto isotropic = flux::solve_hydraulic_erosion_path(input, isotropic_config);

  auto anisotropic_config = isotropic_config;
  anisotropic_config.anisotropy_axis_x = 1'024;
  anisotropic_config.transverse_anisotropy_per_mille = 4'000U;
  const auto anisotropic = flux::solve_hydraulic_erosion_path(input, anisotropic_config);
  const auto transverse_bank = input.source - width;
  require(isotropic.ok() && anisotropic.ok() &&
              anisotropic.static_potential[transverse_bank] >
                  isotropic.static_potential[transverse_bank] &&
              anisotropic.erosion[transverse_bank] < isotropic.erosion[transverse_bank] &&
              anisotropic.walkable == input.walkable,
          "anisotropic hydraulic routing must raise transverse travel cost and confine bank "
          "erosion without modifying authoritative free space");

  auto invalid_config = anisotropic_config;
  invalid_config.anisotropy_axis_x = 0;
  const auto invalid = flux::solve_hydraulic_erosion_path(input, invalid_config);
  require(!invalid.ok() && invalid.diagnostic == "invalid hydraulic erosion path request",
          "an enabled anisotropic metric must reject a missing direction axis");

  anisotropic_config.prefer_gpu = true;
  const auto gpu = flux::solve_hydraulic_erosion_path(input, anisotropic_config);
  require(gpu.ok() && gpu.used_gpu && gpu.riverbed == anisotropic.riverbed &&
              gpu.static_potential == anisotropic.static_potential &&
              gpu.erosion == anisotropic.erosion &&
              gpu.accumulated_flow == anisotropic.accumulated_flow,
          "the Vulkan anisotropic hydraulic field must remain an integer-exact CPU equivalent: " +
              gpu.diagnostic);
}

void hydraulic_polygon_discovery_projects_into_coarse_navmesh() {
  flux::OffsetFreeSpaceRegion region;
  region.area = rectangle(0.5, 0.5, 99.5, 79.5);
  region.holes.push_back(rectangle(29.5, 39.5, 70.5, 40.7));
  const flux::NavMeshConfig navmesh_config{Nanometres::from_mm(5.0),
                                           100'000U,
                                           100'000U,
                                           {},
                                           Nanometres{0},
                                           1U,
                                           Nanometres{0},
                                           std::nullopt};
  const auto prepared = flux::prepare_navmesh_domain(region, navmesh_config);
  flux::NavMeshHydraulicConfig hydraulic_config;
  hydraulic_config.flow.relaxation_iterations = 128U;
  hydraulic_config.flow.droplet_batches = 2U;
  hydraulic_config.flow.droplets_per_batch = 128U;
  hydraulic_config.flow.maximum_steps_per_droplet = 128U;
  hydraulic_config.flow.prefer_gpu = true;
  hydraulic_config.downstream_feedback.push_back(flux::NavMeshHydraulicTerrainDeposit{
      flux::geometry::AxisAlignedBounds{mm(45.0, 45.0), mm(55.0, 55.0)}, mm(45.0, 45.0),
      mm(55.0, 55.0), true, 7U, 3U});
  const auto gpu = flux::route_prepared_navmesh_hydraulic(prepared, mm(20.0, 40.0), mm(80.0, 60.0),
                                                          navmesh_config, hydraulic_config);
  hydraulic_config.flow.prefer_gpu = false;
  const auto cpu = flux::route_prepared_navmesh_hydraulic(prepared, mm(20.0, 40.0), mm(80.0, 60.0),
                                                          navmesh_config, hydraulic_config);
  require(gpu.ok() && gpu.used_hydraulic_gpu && gpu.hydraulic_direct_polygon_discovery &&
              gpu.hydraulic_field_cell_pitch.value() == navmesh_config.cell_pitch.value() / 2 &&
              gpu.hydraulic_discovery_cells > prepared.cells.size() &&
              gpu.hydraulic_exact_certified_chords > 0U &&
              flux::validate_navmesh_topology_signature(gpu.topology_signature, gpu.centerline)
                  .preserved &&
              cpu.ok() && cpu.centerline == gpu.centerline &&
              cpu.hydraulic_heightmap == gpu.hydraulic_heightmap &&
              cpu.hydraulic_flow_field == gpu.hydraulic_flow_field &&
              cpu.hydraulic_erosion_field == gpu.hydraulic_erosion_field &&
              cpu.hydraulic_free_space_field == gpu.hydraulic_free_space_field &&
              cpu.hydraulic_downstream_pressure_field == gpu.hydraulic_downstream_pressure_field &&
              cpu.hydraulic_success_prior_field == gpu.hydraulic_success_prior_field &&
              gpu.hydraulic_maximum_downstream_pressure == 7U &&
              gpu.hydraulic_maximum_success_prior == 3U,
          "hydraulic routing must discover a fine polygonal free-space river before exact "
          "certification and coarse NavMesh resource projection: " +
              gpu.diagnostic + "/" + cpu.diagnostic);
}

void hydraulic_gpu_fuses_clearance_and_batches_independent_requests() {
  constexpr std::size_t width = 48U;
  constexpr std::size_t height = 32U;
  std::vector<flux::HydraulicPathInput> inputs(4U);
  for (std::size_t index = 0U; index < inputs.size(); ++index) {
    auto& input = inputs[index];
    input.width = width;
    input.height = height;
    input.walkable.assign(width * height, 1U);
    for (std::size_t y = 6U; y < 26U; ++y) {
      if (y != 13U + index)
        input.walkable[y * width + 24U] = 0U;
    }
    input.source = (4U + index) * width + 3U;
    input.target = (26U - index) * width + 44U;
  }
  flux::HydraulicPathConfig cpu_config;
  cpu_config.prefer_gpu = false;
  cpu_config.relaxation_iterations = 96U;
  cpu_config.droplet_batches = 2U;
  cpu_config.droplets_per_batch = 64U;
  cpu_config.maximum_steps_per_droplet = 96U;
  cpu_config.anisotropy_axis_x = 1'024;
  cpu_config.transverse_anisotropy_per_mille = 2'500U;
  std::vector<flux::HydraulicPathResult> expected;
  for (const auto& input : inputs)
    expected.push_back(flux::solve_hydraulic_erosion_path(input, cpu_config));

  auto gpu_config = cpu_config;
  gpu_config.prefer_gpu = true;
  std::vector<flux::HydraulicPathResult> actual(inputs.size());
  std::vector<std::jthread> workers;
  std::barrier ready(static_cast<std::ptrdiff_t>(inputs.size()));
  for (std::size_t index = 0U; index < inputs.size(); ++index) {
    workers.emplace_back([&, index] {
      ready.arrive_and_wait();
      actual[index] = flux::solve_hydraulic_erosion_path(inputs[index], gpu_config);
    });
  }
  workers.clear();
  for (std::size_t index = 0U; index < inputs.size(); ++index) {
    require(actual[index].ok() && actual[index].used_gpu && actual[index].gpu_batch_size == 4U &&
                actual[index].used_gpu_clearance_field && expected[index].ok() &&
                actual[index].riverbed == expected[index].riverbed &&
                actual[index].obstacle_clearance == expected[index].obstacle_clearance &&
                actual[index].static_potential == expected[index].static_potential &&
                actual[index].erosion == expected[index].erosion &&
                actual[index].accumulated_flow == expected[index].accumulated_flow,
            "concurrent hydraulic requests must retain deterministic CPU-equivalent clearance, "
            "potential, erosion, flow, and riverbed results: " +
                actual[index].diagnostic);
  }
  const auto reused = flux::solve_hydraulic_erosion_path(inputs.front(), gpu_config);
  require(reused.ok() && reused.gpu_clearance_cache_hit &&
              reused.gpu_persistent_buffer_reuses == 14U && reused.gpu_buffer_growths == 0U &&
              reused.gpu_mask_upload_reuses == 1U && reused.gpu_mask_uploads == 0U,
          "a repeated hydraulic domain must reuse its private buffers and resident shared mask");

  flux::RoutingComputeViaCandidateInput via_filter;
  via_filter.clearance_nm = 100'000;
  via_filter.top_k = 8U;
  for (std::size_t index = 0U; index < 128U; ++index) {
    flux::RoutingComputeViaCandidate candidate;
    candidate.x_nm = static_cast<std::int64_t>(index) * 1'000'000;
    candidate.y_nm = 5'000'000;
    candidate.radius_nm = 250'000;
    candidate.runup_radius_nm = 100'000;
    candidate.owner = 1U;
    candidate.layer_mask = std::uint64_t{1U} << 1U;
    candidate.runup_count = 1U;
    candidate.runup_origins_nm[0] = {candidate.x_nm, 0};
    candidate.runup_layers[0] = 1U;
    candidate.corridor_occupancy_pressure = index >= 64U ? 1U : 0U;
    via_filter.candidates.push_back(candidate);
  }
  for (std::size_t index = 0U; index < 16U; ++index) {
    const auto x = static_cast<std::int64_t>(index) * 1'000'000;
    via_filter.obstacles.push_back(
        flux::RoutingComputeSegment{x, 4'900'000, x, 5'100'000, 250'000, 1U, 2U});
  }
  const auto via_cpu = flux::filter_routing_via_candidates(via_filter, false);
  const auto via_gpu = flux::filter_routing_via_candidates(via_filter, true);
  require(via_cpu.ok() && via_gpu.ok() && via_gpu.used_gpu &&
              via_cpu.shortlist.size() == via_gpu.shortlist.size() &&
              std::equal(via_cpu.shortlist.begin(), via_cpu.shortlist.end(),
                         via_gpu.shortlist.begin(),
                         [](const auto& lhs, const auto& rhs) {
                           return lhs.candidate == rhs.candidate &&
                                  (lhs.rejection_bits == 0U) == (rhs.rejection_bits == 0U);
                         }) &&
              std::count_if(via_gpu.rejection_bitmap.begin(), via_gpu.rejection_bitmap.end(),
                            [](auto bits) { return bits != 0U; }) >= 16,
          "regional GPU via filtering must retain the deterministic legal top-K while rejecting "
          "obstacle/run-up conflicts before exact regional checks");
}

void regional_ownership_and_immutable_cache_are_dependency_bound() {
  flux::OffsetFreeSpaceRegion region;
  region.area = rectangle(0, 0, 10, 10);
  flux::NavMeshConfig config;
  flux::NavMeshPreparationCache cache(10'000, 2);
  const auto first = cache.prepare(region, config);
  const auto repeated = cache.prepare(region, config);
  require(first->ok() && first == repeated && cache.stats().hits == 1 && cache.stats().misses == 1,
          "unchanged geometry/config must reuse the same immutable prepared domain");
  auto changed_region = region;
  changed_region.holes.push_back(rectangle(4, 4, 6, 6));
  const auto changed = cache.prepare(changed_region, config);
  require(changed != first && !flux::query_prepared_navmesh_point(*changed, mm(5, 5)).inside &&
              flux::query_prepared_navmesh_point(*first, mm(5, 5)).inside,
          "geometry changes must invalidate the key without mutating an outstanding snapshot");
  auto changed_config = config;
  changed_config.minimum_boundary_clearance = Nanometres::from_mm(0.1);
  const auto tighter = cache.prepare(region, changed_config);
  require(tighter != first && cache.stats().evictions == 1 && first->ok(),
          "clearance changes miss the cache and bounded eviction preserves live readers");
  const auto disabled = flux::NavMeshPreparationCache(0, 0).prepare(region, config);
  require(disabled->ok(), "zero cache capacity disables reuse, not preparation");
  flux::RegionalRouteResult transaction;
  transaction.native_realization_manifest_available = true;
  flux::RegionalSplineRealization route;
  route.fragment.net = NetId{"owner"};
  route.owner_window_id = 7;
  flux::RegionalRoutingWindow window;
  window.id = 7;
  window.owned_fragments = {route.fragment};
  transaction.windows = {window};
  transaction.native_realization_manifest = {route};
  require(flux::validate_regional_native_ownership(transaction), "single explicit writer is valid");
  transaction.native_realization_manifest.push_back(route);
  require(!flux::validate_regional_native_ownership(transaction),
          "even identical duplicate copper writers must be rejected");
  transaction.native_realization_manifest.resize(1);
  transaction.native_realization_manifest.front().owner_window_id.reset();
  require(!flux::validate_regional_native_ownership(transaction), "missing ownership fails closed");
  transaction.native_realization_manifest.front().owner_window_id = 7;
  window.id = 8;
  transaction.windows.push_back(window);
  require(!flux::validate_regional_native_ownership(transaction), "two owning windows fail closed");
}

void usped_mountains_inform_navmesh_without_changing_free_space() {
  const std::vector<std::uint8_t> mask{1, 1, 0, 1, 1};
  const std::vector<double> clearance{0.1, 3.0, 0.0, 3.0, 0.1};
  const auto field = flux::build_usped_field(5, 1, mask, clearance);
  require(field.ok() && field.height[2] == 1.0 && field.accumulation[2] == 0 &&
              field.accumulation[1] == 2 && field.accumulation[3] == 2 &&
              field.divergence[0] > 0 && field.divergence[1] < 0 && field.sinks == 2,
          "USPED must map obstacles to masked mountains and accumulate independently in valleys");
  double sum = 0;
  for (auto d : field.divergence) sum += d;
  require(std::abs(sum) < 1e-12 &&
              flux::build_usped_field(5, 1, mask, clearance).divergence == field.divergence,
          "USPED masked edge flux must be conservative and deterministic");
  require(!flux::build_usped_field(5, 1, mask, clearance, 4).ok() &&
              !flux::build_usped_field(0, 1, {}, {}).ok() &&
              !flux::build_usped_field(1, 1, {1}, {-1}).ok(),
          "USPED rejects malformed or over-budget fields");
  const auto diagonal = flux::build_usped_field(2, 2, {1, 0, 0, 1}, {0.1, 0, 0, 3});
  require(diagonal.ok() && diagonal.accumulation[3] == 1 && diagonal.sinks == 2,
          "USPED must not cut diagonal obstacle corners");
  const auto flat = flux::build_usped_field(2, 2, {1, 1, 1, 1}, {2, 2, 2, 2});
  require(flat.ok() && flat.sinks == 4 && flat.transport == std::vector<double>(4, 0) &&
              flat.refinement_priority == std::vector<double>(4, 0),
          "flat terrain must remain finite and must not invent drainage through mountains");

  flux::OffsetFreeSpaceRegion region;
  region.area = rectangle(0, 0, 12, 10);
  region.holes.push_back(rectangle(5, 3, 7, 7));
  flux::NavMeshConfig config;
  const auto baseline = flux::prepare_navmesh_domain(region, config);
  config.usped.enabled = true;
  const auto informed = flux::prepare_navmesh_domain(region, config);
  std::cout << "USPED_MICRO baseline_cells=" << baseline.cells.size()
            << " informed_cells=" << informed.cells.size()
            << " refined_parents=" << informed.usped_refined_parents
            << " raster_cells=" << informed.usped_field.height.size() << '\n';
  require(baseline.ok() && informed.ok() && informed.usped_field.ok() &&
              informed.usped_refined_parents > 0 &&
              informed.usped_refined_parents <= config.usped.maximum_refined_parents &&
              informed.cells.size() == baseline.cells.size() + 3 * informed.usped_refined_parents,
          "USPED prepass must subdivide bounded certified coarse cells without discarding baseline cells");
  require(!flux::query_prepared_navmesh_point(informed, mm(6, 5)).inside &&
              flux::route_prepared_navmesh(informed, mm(2, 5), mm(10, 5), config).ok(),
          "USPED mesh must retain obstacle exclusion and a certified route around the mountain");
  config.usped.enabled = false;
  require(!flux::route_prepared_navmesh(informed, mm(2, 5), mm(10, 5), config).ok(),
          "prepared mesh must reject mismatched USPED configuration");
  config.usped.enabled = true;
  config.maximum_cells = baseline.cells.size();
  const auto limited = flux::prepare_navmesh_domain(region, config);
  require(limited.ok() && limited.cells.size() == baseline.cells.size() &&
              limited.usped_refined_parents == 0 && !limited.usped_diagnostic.empty(),
          "optional USPED subdivision must fall back to baseline when its mesh budget is exhausted");
  config.usped.maximum_cells = 1;
  const auto skipped = flux::prepare_navmesh_domain(region, config);
  require(skipped.ok() && !skipped.usped_field.ok() && !skipped.usped_diagnostic.empty(),
          "over-budget prepass must preserve ordinary boundary refinement");
}

void adaptive_navmesh_recovers_clearance_clipped_passage() {
  require(flux::NavMeshConfig{}.adaptive_clearance_refinement &&
              flux::NavMeshConfig{}.maximum_refinement_depth == 2U,
          "adaptive NavMesh must default to two bounded refinement levels");
  flux::FreeSpaceDomain heterogeneous_domain;
  heterogeneous_domain.outline = rectangle(0.0, 0.0, 20.0, 10.0);
  heterogeneous_domain.obstacles = {rectangle(4.0, 4.0, 5.0, 5.0), rectangle(12.0, 4.0, 13.0, 5.0)};
  const std::vector<std::pair<Point, Point>> validation_chords{
      {mm(1.0, 1.0), mm(3.0, 1.0)}, {mm(1.0, 4.5), mm(6.0, 4.5)}};
  require(flux::validate_free_space_chord_domain(heterogeneous_domain) &&
              flux::assess_validated_free_space_chords(heterogeneous_domain,
                                                       validation_chords) ==
                  flux::assess_free_space_chords(heterogeneous_domain, validation_chords),
          "prevalidated batched chord certificates must retain the fail-closed exact geometry "
          "result while omitting only repeated invariant domain validation");
  flux::FreeSpaceOffsetConfig heterogeneous_config{Nanometres::from_mm(0.1)};
  heterogeneous_config.obstacle_clearances = {Nanometres::from_mm(0.2), Nanometres::from_mm(2.0)};
  const auto heterogeneous =
      flux::offset_polygonal_free_space(heterogeneous_domain, heterogeneous_config);
  const auto represented_free = [&](Point point) {
    return std::any_of(heterogeneous.regions.begin(), heterogeneous.regions.end(),
                       [&](const auto& candidate) {
                         if (flux::geometry::locate_point(candidate.area, point) ==
                             flux::geometry::PointLocation::outside)
                           return false;
                         return std::none_of(candidate.holes.begin(), candidate.holes.end(),
                                             [&](const auto& hole) {
                                               return flux::geometry::locate_point(hole, point) !=
                                                      flux::geometry::PointLocation::outside;
                                             });
                       });
  };
  require(heterogeneous.ok() && represented_free(mm(6.0, 4.5)) && !represented_free(mm(14.0, 4.5)),
          "heterogeneous free-space preparation must preserve room beside a low-clearance "
          "obstacle while reserving the larger envelope of a high-clearance obstacle: " +
              heterogeneous.diagnostic);

  flux::OffsetFreeSpaceRegion region;
  region.area = rectangle(0.0, 0.0, 10.0, 10.0);
  region.holes.push_back(rectangle(4.2, 0.1, 5.8, 8.8));
  flux::NavMeshConfig coarse;
  coarse.cell_pitch = Nanometres::from_mm(2.0);
  coarse.maximum_cells = 20'000U;
  coarse.maximum_search_expansions = 20'000U;
  coarse.corridor_guard_rings = 1U;
  coarse.adaptive_clearance_refinement = false;
  const auto coarse_domain = flux::prepare_navmesh_domain(region, coarse);
  const auto coarse_route =
      flux::route_prepared_navmesh(coarse_domain, mm(2.0, 5.0), mm(8.0, 5.0), coarse);

  auto adaptive = coarse;
  adaptive.adaptive_clearance_refinement = true;
  adaptive.maximum_refinement_depth = 4U;
  adaptive.minimum_refined_cell_pitch = Nanometres::from_mm(0.125);
  const auto adaptive_domain = flux::prepare_navmesh_domain(region, adaptive);
  const auto adaptive_route =
      flux::route_prepared_navmesh(adaptive_domain, mm(2.0, 5.0), mm(8.0, 5.0), adaptive);
  flux::RoutingComputeGraphWavefrontInput graph;
  graph.node_count = 64U * 64U;
  graph.maximum_passes = 8'192U;
  graph.sources = {0U};
  graph.offsets.reserve(graph.node_count + 1U);
  for (std::size_t node = 0U; node < graph.node_count; ++node) {
    graph.offsets.push_back(static_cast<std::uint32_t>(graph.edges.size()));
    const auto x = node % 64U;
    const auto y = node / 64U;
    if (x + 1U < 64U)
      graph.edges.push_back({static_cast<std::uint32_t>(node + 1U), 1U});
    if (x > 0U)
      graph.edges.push_back({static_cast<std::uint32_t>(node - 1U), 1U});
    if (y + 1U < 64U)
      graph.edges.push_back({static_cast<std::uint32_t>(node + 64U), 1U});
    if (y > 0U)
      graph.edges.push_back({static_cast<std::uint32_t>(node - 64U), 1U});
  }
  graph.offsets.push_back(static_cast<std::uint32_t>(graph.edges.size()));
  const auto graph_gpu = flux::compute_routing_graph_wavefront(graph, true);
  const auto graph_cpu = flux::compute_routing_graph_wavefront(graph, false);
  require(!coarse_route.ok() && adaptive_route.ok() && adaptive_domain.refined_parent_cells > 0U &&
              adaptive_domain.refined_leaf_cells > 0U &&
              adaptive_domain.maximum_realized_refinement_depth > 0U &&
              adaptive_route.portals.size() + 1U == adaptive_route.corridor.size() &&
              std::all_of(adaptive_route.centerline.begin(), adaptive_route.centerline.end(),
                          [&](Point point) {
                            return flux::navmesh_corridor_contains(adaptive_route, point);
                          }) &&
              flux::validate_navmesh_topology_signature(adaptive_route.topology_signature,
                                                        adaptive_route.centerline)
                  .preserved &&
              graph_gpu.ok() && graph_gpu.used_gpu && graph_cpu.ok() &&
              graph_gpu.distance == graph_cpu.distance,
          "adaptive NavMesh must retain exact mixed-size portals through a passage discarded by "
          "the coarse lattice: coarse=" +
              coarse_route.diagnostic + " adaptive=" + adaptive_route.diagnostic +
              " leaves=" + std::to_string(adaptive_domain.refined_leaf_cells));
}

void advanced_routing_compute_shortlists_preserve_cpu_authority() {
  flux::RoutingComputeQuantizedCandidateInput quantized;
  quantized.candidate_count = 16U;
  quantized.points_per_candidate = 4U;
  quantized.routes = {{0U, 2U, 0U, 0U, 100'000}, {2U, 2U, 0U, 1U, 100'000}};
  quantized.clearance_nm = 100'000;
  quantized.top_k = 16U;
  for (std::size_t candidate = 0U; candidate < quantized.candidate_count; ++candidate) {
    const auto y = static_cast<long double>(candidate) * 50'000.0L;
    quantized.candidate_points_nm.insert(quantized.candidate_points_nm.end(),
                                         {{0.0L, 0.0L},
                                          {1'000'000.0L, 0.0L},
                                          {500'000.0L, -500'000.0L + y},
                                          {500'000.0L, 500'000.0L + y}});
  }
  const auto quantized_cpu = flux::score_quantized_routing_candidates(quantized, false);
  const auto quantized_gpu = flux::score_quantized_routing_candidates(quantized, true);

  flux::RoutingComputeAssignmentInput assignments;
  assignments.maximum_states = 4'096U;
  assignments.top_k = 32U;
  assignments.option_costs.resize(24U);
  std::iota(assignments.option_costs.begin(), assignments.option_costs.end(), 0U);
  for (std::uint32_t variable = 0U; variable < 6U; ++variable)
    assignments.variable_domains.push_back(
        {variable * 4U, variable * 4U + 1U, variable * 4U + 2U, variable * 4U + 3U});
  assignments.incompatible_options = {{0U, 4U}, {1U, 9U}, {7U, 22U}};
  const auto assignment_cpu = flux::evaluate_routing_assignments(assignments, false);
  const auto assignment_gpu = flux::evaluate_routing_assignments(assignments, true);

  flux::RoutingComputeLaneOrderInput lane;
  lane.widths_nm.assign(7U, 100'000U);
  lane.clearances_nm = {30'000U, 40'000U, 20'000U, 50'000U, 25'000U, 35'000U, 30'000U};
  lane.reference_position = {6U, 5U, 4U, 3U, 2U, 1U, 0U};
  lane.coupled_rank.assign(7U, std::numeric_limits<std::uint32_t>::max());
  lane.usable_width_nm = 1'500'000U;
  const auto lane_cpu = flux::solve_routing_lane_order(lane, false);
  const auto lane_gpu = flux::solve_routing_lane_order(lane, true);

  flux::RoutingComputeOrientationInput orientation;
  orientation.bit_count = 8U;
  orientation.top_k = 16U;
  orientation.bit_flip_costs.assign(8U, 1U);
  orientation.forbidden_clauses = {{1U, 1U}, {6U, 6U}};
  const auto orientation_cpu = flux::evaluate_routing_orientations(orientation, false);
  const auto orientation_gpu = flux::evaluate_routing_orientations(orientation, true);

  std::vector<flux::RoutingComputeCopperPrimitive> copper{
      {flux::RoutingComputeCopperKind::segment, 0, 0, 1'000'000, 0, 100'000, 0U, 0U},
      {flux::RoutingComputeCopperKind::circle, 500'000, 50'000, 500'000, 50'000, 100'000, 0U, 1U}};
  std::vector<std::pair<std::uint32_t, std::uint32_t>> copper_pairs(256U, {0U, 1U});
  const auto copper_cpu =
      flux::assess_simple_copper_pairs(copper, copper_pairs, 100'000, 2'048, false);
  const auto copper_gpu =
      flux::assess_simple_copper_pairs(copper, copper_pairs, 100'000, 2'048, true);

  flux::RoutingComputeBatchedGraphWavefrontInput graph;
  graph.node_count = 2'048U;
  graph.maximum_passes = 2'048U;
  graph.offsets.reserve(graph.node_count + 1U);
  for (std::size_t node = 0U; node < graph.node_count; ++node) {
    graph.offsets.push_back(static_cast<std::uint32_t>(graph.edges.size()));
    if (node > 0U)
      graph.edges.push_back({static_cast<std::uint32_t>(node - 1U), 1U});
    if (node + 1U < graph.node_count)
      graph.edges.push_back({static_cast<std::uint32_t>(node + 1U), 1U});
  }
  graph.offsets.push_back(static_cast<std::uint32_t>(graph.edges.size()));
  graph.source_groups = {{0U}, {graph.node_count - 1U}};
  const auto graph_cpu = flux::compute_batched_routing_graph_wavefront(graph, false);
  const auto graph_gpu = flux::compute_batched_routing_graph_wavefront(graph, true);

  std::vector<flux::RoutingComputeArcCandidate> arcs(4U);
  for (std::size_t index = 0U; index < arcs.size(); ++index) {
    arcs[index].points_nm = {
        {{0, 0}, {100'000, 0}, {200'000, 100'000}, {300'000, 200'000}, {400'000, 200'000}}};
    arcs[index].radius_nm = static_cast<std::int64_t>(index + 1U) * 100'000;
    arcs[index].copper_radius_nm = 50'000;
  }
  const auto arcs_cpu = flux::score_routing_arc_candidates(arcs, {}, 100'000, 4U, false);
  const auto arcs_gpu = flux::score_routing_arc_candidates(arcs, {}, 100'000, 4U, true);

  std::vector<flux::RoutingComputeNegotiatedResource> resources(256U, {3U, 1U, 7U, 2U});
  const auto resources_cpu = flux::reduce_negotiated_routing_resources(resources, false);
  const auto resources_gpu = flux::reduce_negotiated_routing_resources(resources, true);

  std::vector<flux::RoutingComputePrimitiveBounds> offset_hulls;
  offset_hulls.reserve(64U);
  for (std::size_t owner = 0U; owner < 2U; ++owner) {
    for (std::size_t index = 0U; index < 32U; ++index) {
      const auto x = static_cast<std::int64_t>(index) * 100'000;
      const auto shift = owner == 0U ? 0 : 25'000;
      offset_hulls.push_back(
          {x + shift, 0, x + shift + 50'000, 50'000, 0U, static_cast<std::uint32_t>(owner)});
    }
  }
  const auto hulls_cpu =
      flux::discover_routing_broadphase_pairs(offset_hulls, 100'000, 1'024U, false);
  const auto hulls_gpu =
      flux::discover_routing_broadphase_pairs(offset_hulls, 100'000, 1'024U, true);

  require(
      quantized_cpu.ok() && quantized_gpu.ok() && quantized_gpu.used_gpu &&
          quantized_cpu.shortlist.front().candidate == quantized_gpu.shortlist.front().candidate &&
          assignment_cpu.ok() && assignment_gpu.ok() && assignment_gpu.used_gpu &&
          assignment_cpu.shortlist.front().options == assignment_gpu.shortlist.front().options &&
          lane_cpu.ok() && lane_gpu.ok() && lane_gpu.used_gpu && lane_cpu.physically_fits &&
          lane_gpu.physically_fits && orientation_cpu.states == orientation_gpu.states &&
          orientation_gpu.used_gpu && copper_cpu.ok() && copper_gpu.ok() && copper_gpu.used_gpu &&
          copper_cpu.pairs.front().possible_conflict ==
              copper_gpu.pairs.front().possible_conflict &&
          graph_cpu.ok() && graph_gpu.ok() && graph_gpu.used_gpu &&
          graph_cpu.distance == graph_gpu.distance && arcs_cpu.ok() && arcs_gpu.ok() &&
          arcs_gpu.used_gpu &&
          arcs_cpu.shortlist.front().candidate == arcs_gpu.shortlist.front().candidate &&
          hulls_cpu.ok() && hulls_gpu.ok() && hulls_gpu.used_gpu &&
          hulls_cpu.candidate_pairs == hulls_gpu.candidate_pairs &&
          resources_cpu.overflow == resources_gpu.overflow && resources_gpu.used_gpu,
      "advanced GPU shortlists/reductions must agree with their deterministic CPU authority");
}

void routing_compute_chord_batches_are_bounded_and_conservative() {
  constexpr auto origin = std::int64_t{8'000'000'000'000'000LL};
  std::vector<flux::RoutingComputeSegment> chords;
  std::vector<flux::RoutingComputeSegment> boundaries;
  chords.reserve(300U);
  boundaries.reserve(1'100U);
  for (std::size_t index = 0U; index < 300U; ++index) {
    const auto x = origin + static_cast<std::int64_t>(index) * 10'000;
    chords.push_back({x, origin, x + 5'000, origin, 0, 0U, 0U});
  }
  for (std::size_t index = 0U; index < 1'100U; ++index) {
    const auto x = origin + static_cast<std::int64_t>(index) * 5'000;
    boundaries.push_back({x, origin + 10'000'000, x + 2'500, origin + 10'000'000, 0, 0U, 1U});
  }
  // This chord is deliberately a survivor. It crosses a boundary that lies in the second
  // resident boundary chunk, which proves clear classification is aggregated across chunks.
  chords[299U] = {origin + 5'350'000, origin + 10'000'000, origin + 5'355'000,
                  origin + 10'000'000, 0, 0U, 0U};
  const auto cpu = flux::classify_routing_chords(chords, boundaries, 1'000, 2'048, false);
  const auto gpu = flux::classify_routing_chords(chords, boundaries, 1'000, 2'048, true);
  const auto invalid = flux::classify_routing_chords(
      chords, boundaries, std::numeric_limits<std::int64_t>::max(), 1, true);
  const auto same_classification =
      cpu.ok() && gpu.ok() && cpu.chords.size() == gpu.chords.size() &&
      std::equal(cpu.chords.begin(), cpu.chords.end(), gpu.chords.begin(),
                 [](const auto& exact, const auto& screened) {
                   // GPU intersection is intentionally ambiguous rather than a final rejection.
                   return (exact.classification == flux::RoutingComputeChordClass::definitely_clear) ==
                          (screened.classification == flux::RoutingComputeChordClass::definitely_clear);
                 });
  require(same_classification &&
              gpu.chords.back().classification != flux::RoutingComputeChordClass::definitely_clear &&
              !invalid.ok() &&
              (!gpu.used_gpu ||
               (gpu.gpu_batch_count == 3U && gpu.gpu_work_items == chords.size() * boundaries.size())) &&
              (!gpu.used_gpu || gpu.chords.front().closest_boundary == 0U),
          "bounded translated chord screening must retain exact survivors and aggregate every "
          "resident boundary chunk");
}

void physical_copper_t_junction_is_preserved_during_ph_realization() {
  auto source = two_pad_board();
  auto branch_footprint = source.footprints.front();
  branch_footprint.id = ObjectId{"U3"};
  branch_footprint.reference = "U3";
  branch_footprint.position = mm(50.0, 60.0);
  branch_footprint.pads.front().id = ObjectId{"U3-pad"};
  source.footprints.push_back(std::move(branch_footprint));

  flux::GlobalRouteResult global;
  global.stop_reason = GlobalRouterStopReason::complete;
  flux::GlobalRoutePath main;
  main.net = NetId{"net-1"};
  main.connection_index = 0U;
  main.points = {{mm(20.0, 40.0), LayerId{"F.Cu"}},
                 {mm(50.0, 40.0), LayerId{"F.Cu"}},
                 {mm(50.0, 60.0), LayerId{"F.Cu"}}};
  flux::GlobalRoutePath branch;
  branch.net = NetId{"net-1"};
  branch.connection_index = 1U;
  branch.points = {{mm(80.0, 40.0), LayerId{"F.Cu"}}, {mm(50.0, 41.0), LayerId{"F.Cu"}}};
  global.paths = {std::move(main), std::move(branch)};

  flux::DetailedRouterConfig config;
  config.pull_tight_45_degrees = false;
  config.continuous_refinement = false;
  config.circular_arc_refinement = true;
  config.spline_minimum_bend_radius = Nanometres{0};
  config.maximum_circular_arc_radius = Nanometres::from_mm(1.0);
  const auto detailed = flux::materialize_detailed_route(source, global, config);
  require(detailed.complete() && detailed.candidate &&
              detailed.connectivity.open_connections == 0U &&
              detailed.unrounded_non_junction_bends == 0U,
          "an exact same-net branch ending inside an unsplit carrier must become a fixed PH "
          "attachment boundary before the adjacent non-junction corner is rounded: " +
              detailed.diagnostic);

  // A terminal is itself the third electrical incidence at a two-track centreline meeting. The
  // PH run must remain anchored there, while the final quality audit must not confuse that
  // intentional terminal junction with a free-space polygonal bend.
  auto terminal_global = global;
  terminal_global.paths[1].points.back().position = mm(50.0, 60.0);
  const auto terminal = flux::materialize_detailed_route(source, terminal_global, config);
  require(terminal.complete() && terminal.candidate &&
              terminal.connectivity.open_connections == 0U &&
              terminal.unrounded_non_junction_bends == 0U,
          "two same-net tracks meeting at a pad centre must be classified as an anchored terminal "
          "junction rather than an unrealized free-space PH bend: " +
              terminal.diagnostic);

  // A connection may pass through an already connected through-hole terminal instead of ending
  // there.  That interior pad contact is part of the routed tree topology and must remain fixed
  // while the bends on either side are converted to PH geometry.
  auto pass_through_source = source;
  auto fourth_footprint = source.footprints.front();
  fourth_footprint.id = ObjectId{"U4"};
  fourth_footprint.reference = "U4";
  fourth_footprint.position = mm(60.0, 70.0);
  fourth_footprint.pads.front().id = ObjectId{"U4-pad"};
  pass_through_source.footprints.push_back(std::move(fourth_footprint));
  auto pass_through_global = global;
  pass_through_global.paths[1].points = {{mm(80.0, 40.0), LayerId{"F.Cu"}},
                                         {mm(50.0, 40.0), LayerId{"F.Cu"}},
                                         {mm(50.0, 60.0), LayerId{"F.Cu"}},
                                         {mm(60.0, 70.0), LayerId{"F.Cu"}}};
  const auto pass_through =
      flux::materialize_detailed_route(pass_through_source, pass_through_global, config);
  require(pass_through.complete() && pass_through.candidate &&
              pass_through.connectivity.open_connections == 0U &&
              pass_through.unrounded_non_junction_bends == 0U,
          "an interior same-net pad contact must split the PH chain and remain electrically "
          "anchored through curved realization: " +
              pass_through.diagnostic);
}

void same_net_vertical_features_are_reused_as_transition_portals() {
  auto via_board = two_pad_board();
  via_board.footprints.back().pads.front().layers = {LayerId{"B.Cu"}};
  const auto via_position = mm(50.3, 40.2);
  via_board.vias.push_back(flux::Via{ObjectId{"existing-net-via"},
                                     NetId{"net-1"},
                                     via_position,
                                     Nanometres::from_mm(0.8),
                                     Nanometres::from_mm(0.4),
                                     {LayerId{"F.Cu"}, LayerId{"B.Cu"}},
                                     true});
  GlobalRouterConfig config;
  config.grid_pitch = Nanometres::from_mm(5.0);
  config.via_site_pitch = Nanometres::from_mm(100.0);
  config.via_site_phase_x = 19U;
  config.via_site_phase_y = 19U;
  config.max_iterations = 2U;
  const auto global = flux::route_global(via_board, config);
  const auto exact_transition =
      std::any_of(global.paths.begin(), global.paths.end(), [&](const auto& path) {
        for (std::size_t index = 1U; index < path.points.size(); ++index) {
          if (path.points[index - 1U].position == via_position &&
              path.points[index].position == via_position &&
              path.points[index - 1U].layer != path.points[index].layer) {
            return true;
          }
        }
        return false;
      });
  require(global.complete() && global.reused_existing_via_transitions > 0U && exact_transition,
          "an off-grid same-net via must become a preferred exact transition portal when the "
          "ordinary via lattice has no usable site: " +
              global.diagnostic);

  flux::DetailedRouterConfig detailed_config;
  detailed_config.pull_tight_45_degrees = false;
  detailed_config.continuous_refinement = false;
  detailed_config.circular_arc_refinement = false;
  const auto detailed = flux::materialize_detailed_route(via_board, global, detailed_config);
  require(detailed.complete() && detailed.candidate && detailed.added_vias == 0U &&
              detailed.candidate->vias.size() == 1U,
          "materialization must retain a covering locked same-net via instead of adding "
          "concentric copper: " +
              detailed.diagnostic);

  auto pad_board = two_pad_board();
  pad_board.footprints.back().pads.front().layers = {LayerId{"B.Cu"}};
  Footprint through_hole;
  through_hole.id = ObjectId{"J1"};
  through_hole.reference = "J1";
  through_hole.position = via_position;
  through_hole.courtyard = local_rectangle(3.0, 2.0);
  flux::Pad plated_pad{
      ObjectId{"J1-pad"}, NetId{"net-1"}, Point{}, {LayerId{"F.Cu"}, LayerId{"B.Cu"}}};
  plated_pad.width = Nanometres::from_mm(1.6);
  plated_pad.height = Nanometres::from_mm(1.2);
  plated_pad.shape = flux::PadShape::oval;
  plated_pad.drill_width = Nanometres::from_mm(0.6);
  plated_pad.drill_height = Nanometres::from_mm(0.6);
  plated_pad.drill_offset = mm(0.2, 0.0);
  plated_pad.plated = true;
  through_hole.pads.push_back(std::move(plated_pad));
  pad_board.footprints.push_back(std::move(through_hole));
  const auto pad_hole_position = mm(50.5, 40.2);
  const auto global_pad_route = flux::route_global(pad_board, config);
  const auto exact_pad_transition =
      std::any_of(global_pad_route.paths.begin(), global_pad_route.paths.end(), [&](const auto& p) {
        for (std::size_t index = 1U; index < p.points.size(); ++index) {
          if (p.points[index - 1U].position == pad_hole_position &&
              p.points[index].position == pad_hole_position &&
              p.points[index - 1U].layer != p.points[index].layer) {
            return true;
          }
        }
        return false;
      });
  require(global_pad_route.complete() && global_pad_route.reused_component_pad_transitions > 0U &&
              exact_pad_transition,
          "a plated pad must expose its offset physical hole, rather than its copper anchor, as "
          "the reusable global layer-transition portal: " +
              global_pad_route.diagnostic);
  flux::GlobalRouteResult pad_route;
  pad_route.stop_reason = GlobalRouterStopReason::complete;
  flux::GlobalRoutePath path;
  path.net = NetId{"net-1"};
  path.points = {{mm(20.0, 40.0), LayerId{"F.Cu"}},
                 {pad_hole_position, LayerId{"F.Cu"}},
                 {pad_hole_position, LayerId{"B.Cu"}},
                 {mm(80.0, 40.0), LayerId{"B.Cu"}}};
  pad_route.paths.push_back(std::move(path));
  const auto through_hole_detailed =
      flux::materialize_detailed_route(pad_board, pad_route, detailed_config);
  require(through_hole_detailed.complete() && through_hole_detailed.candidate &&
              through_hole_detailed.added_vias == 0U &&
              through_hole_detailed.candidate->vias.empty(),
          "a plated through-hole component pad must provide the vertical connection without a "
          "synthetic via: " +
              through_hole_detailed.diagnostic);
}

void kicad_zone_refill_adapter_is_bounded_and_fail_closed() {
#ifdef FLUX_TEST_PYTHON_EXECUTABLE
  const auto nonce =
      static_cast<std::uint64_t>(std::chrono::steady_clock::now().time_since_epoch().count());
  const auto root =
      std::filesystem::temp_directory_path() / ("flux-kicad-refill-test-" + std::to_string(nonce));
  std::error_code error;
  require(std::filesystem::create_directory(root, error) && !error,
          "zone-refill test must create an isolated temporary directory");
  struct Cleanup {
    std::filesystem::path path;
    ~Cleanup() {
      std::error_code ignored;
      std::filesystem::remove_all(path, ignored);
    }
  } cleanup{root};
  const auto provisional = root / "provisional.kicad_pcb";
  {
    std::ofstream stream(provisional, std::ios::binary);
    stream << "provisional-board\n";
  }
  const auto passing_helper = root / "passing refill helper.py";
  {
    std::ofstream stream(passing_helper, std::ios::binary);
    stream << R"PY(import json
import pathlib
import shutil
import sys
arguments = dict(zip(sys.argv[1::2], sys.argv[2::2]))
shutil.copyfile(pathlib.Path(arguments["--board"]), pathlib.Path(arguments["--output"]))
pathlib.Path(arguments["--report"]).write_text(json.dumps({"ok": True}), encoding="utf-8")
print("non-JSON diagnostic before the dedicated report")
)PY";
  }
  const auto passing_directory = root / "passing";
  require(std::filesystem::create_directory(passing_directory, error) && !error,
          "zone-refill test must create a passing diagnostic directory");
  const auto passing_output = passing_directory / "refilled.kicad_pcb";
  const auto passing = flux::io::run_kicad_zone_refill(
      {std::filesystem::path{FLUX_TEST_PYTHON_EXECUTABLE}, passing_helper, provisional,
       passing_output, passing_directory, std::chrono::seconds{5}, 4U * 1024U, 1U * 1024U * 1024U});
  require(passing.run && passing.passed && passing.exit_code == 0 &&
              std::filesystem::is_regular_file(passing_output),
          "a shell-free bounded refill adapter must accept an explicit successful JSON/artifact "
          "transaction: " +
              passing.diagnostic);

  const auto failing_helper = root / "failing refill helper.py";
  {
    std::ofstream stream(failing_helper, std::ios::binary);
    stream << "import json, sys\nprint(json.dumps({'ok': False, 'error': 'intentional'}))\n"
              "sys.exit(5)\n";
  }
  const auto failing_directory = root / "failing";
  error.clear();
  require(std::filesystem::create_directory(failing_directory, error) && !error,
          "zone-refill test must create a failing diagnostic directory");
  const auto failing = flux::io::run_kicad_zone_refill(
      {std::filesystem::path{FLUX_TEST_PYTHON_EXECUTABLE}, failing_helper, provisional,
       failing_directory / "must-not-exist.kicad_pcb", failing_directory, std::chrono::seconds{5},
       4U * 1024U, 1U * 1024U * 1024U});
  require(failing.run && !failing.passed && failing.exit_code == 5,
          "a nonzero refill helper result must remain a failed transaction");

  const auto unavailable_directory = root / "unavailable";
  error.clear();
  require(std::filesystem::create_directory(unavailable_directory, error) && !error,
          "zone-refill test must create an unavailable diagnostic directory");
  const auto unavailable = flux::io::run_kicad_zone_refill(
      {std::filesystem::path{FLUX_TEST_PYTHON_EXECUTABLE}, root / "missing-helper.py", provisional,
       unavailable_directory / "must-not-exist.kicad_pcb", unavailable_directory,
       std::chrono::seconds{5}, 4U * 1024U, 1U * 1024U * 1024U});
  require(!unavailable.run && !unavailable.passed && unavailable.exit_code == -1,
          "an unavailable refill helper must fail before a child process is launched");
#endif
}

void generated_copper_telemetry_replaces_latest_scene_and_retains_beziers() {
  flux::RouterTelemetryConfig config;
  config.listen_address.reset();
  config.listen_port.reset();
  flux::RouterTelemetryService telemetry(config);
  Board source;
  source.source_revision = "telemetry-source";
  source.outline = rectangle(0.0, 0.0, 20.0, 20.0);
  source.layers = {{LayerId{"F.Cu"}, "F.Cu", LayerKind::copper, 0U},
                   {LayerId{"B.Cu"}, "B.Cu", LayerKind::copper, 1U}};
  source.nets = {{NetId{"signal"}, "SIGNAL", 0U}};
  const auto curve = [](std::string id, LayerId layer, double y) {
    return flux::TrackBezier{ObjectId{std::move(id)}, NetId{"signal"}, std::move(layer),
                              mm(2.0, y), mm(6.0, y + 2.0), mm(14.0, y - 2.0), mm(18.0, y),
                              Nanometres::from_mm(0.2), false};
  };
  source.track_beziers.push_back(curve("source-front", LayerId{"F.Cu"}, 5.0));
  telemetry.publish_source_board(source, "telemetry-source.kicad_pcb");

  auto generated = source;
  generated.source_revision = "generated-front";
  generated.track_beziers = {curve("generated-front", LayerId{"F.Cu"}, 10.0)};
  telemetry.publish_generated_copper(generated, "front curve");
  const auto latest_scene = [](const flux::RouterTelemetryState& state) {
    std::vector<std::shared_ptr<const flux::telemetry::SourceBoard>> scenes;
    for (const auto& scene : state.source_boards)
      if (scene && scene->id == "generated-copper:latest") scenes.push_back(scene);
    return scenes;
  };
  auto state = telemetry.state();
  auto generated_scenes = latest_scene(state);
  require(generated_scenes.size() == 1U &&
              std::count_if(generated_scenes.front()->primitives.begin(), generated_scenes.front()->primitives.end(),
                            [](const auto& primitive) {
                              return primitive.kind == "track_bezier" &&
                                     primitive.layers == std::vector<std::string>{"F.Cu"};
                            }) == 1U,
          "generated copper telemetry must retain front-layer Bezier tracks");

  generated.source_revision = "generated-back";
  generated.track_beziers = {curve("generated-back", LayerId{"B.Cu"}, 10.0)};
  telemetry.publish_generated_copper(generated, "back curve");
  state = telemetry.state();
  generated_scenes = latest_scene(state);
  require(generated_scenes.size() == 1U &&
              std::count_if(generated_scenes.front()->primitives.begin(), generated_scenes.front()->primitives.end(),
                            [](const auto& primitive) {
                              return primitive.kind == "track_bezier" &&
                                     primitive.layers == std::vector<std::string>{"B.Cu"};
                            }) == 1U &&
              std::none_of(generated_scenes.front()->primitives.begin(), generated_scenes.front()->primitives.end(),
                           [](const auto& primitive) {
                             return primitive.kind == "track_bezier" &&
                                    primitive.layers == std::vector<std::string>{"F.Cu"};
                           }),
          "a later generated snapshot must replace the front scene with the back scene");

  generated.source_revision = "generated-empty";
  generated.track_beziers.clear();
  telemetry.publish_generated_copper(generated, "removed curve");
  state = telemetry.state();
  generated_scenes = latest_scene(state);
  const auto source_scene = std::find_if(state.source_boards.begin(), state.source_boards.end(),
      [](const auto& scene) { return scene && scene->id.starts_with("source-board:"); });
  require(generated_scenes.size() == 1U &&
              std::none_of(generated_scenes.front()->primitives.begin(), generated_scenes.front()->primitives.end(),
                           [](const auto& primitive) { return primitive.kind == "track_bezier"; }) &&
              source_scene != state.source_boards.end() &&
              std::any_of((*source_scene)->primitives.begin(), (*source_scene)->primitives.end(),
                          [](const auto& primitive) { return primitive.kind == "track_bezier"; }),
          "generated snapshot removal must preserve the immutable source scene");
  telemetry.stop();
}

void work_plan_round_trip_is_strict_source_bound_and_non_authoritative() {
  const auto board = two_pad_board();
  auto plan = flux::io::make_flux_work_plan_template(board, "board.kicad_pcb");
  plan.mode = flux::io::WorkPlanMode::force_reroute_selected;
  flux::io::WorkPlanStep route;
  route.id = "route-signals";
  route.kind = flux::io::WorkPlanStepKind::route;
  route.selectors.exact_nets = {"SIGNAL"};
  route.options = {{"arcs", true},
                   {"max_router_iterations", std::uint64_t{30U}},
                   {"priority_nets", std::vector<std::string>{"SIGNAL"}},
                   {"signed_test", std::int64_t{-1}},
                   {"strategy", std::string{"scarcity"}}};
  flux::io::WorkPlanStep validation;
  validation.id = "validate";
  validation.kind = flux::io::WorkPlanStepKind::validation;
  validation.depends_on = {route.id};
  plan.steps = {route, validation};

  const auto serialized = flux::io::serialize_flux_work_plan(plan);
  const auto parsed = flux::io::parse_flux_work_plan(serialized, &board);
  require(parsed.ok() && parsed.plan && *parsed.plan == plan &&
              flux::io::serialize_flux_work_plan(*parsed.plan) == serialized,
          "FluxWorkPlan must round-trip byte-stably without changing typed options");
  require(flux::io::required_flux_work_plan_capabilities(plan) ==
              std::vector<std::string>{"routing.single-ended", "routing.scope",
                                       "verification.diagnostics"},
          "ordered step kinds must produce stable capability preflight IDs");

  auto stale = board;
  stale.source_revision += "-changed";
  const auto stale_result = flux::io::parse_flux_work_plan(serialized, &stale);
  require(
      !stale_result.ok() &&
          std::any_of(stale_result.diagnostics.begin(), stale_result.diagnostics.end(),
                      [](const auto& diagnostic) { return diagnostic.code == "stale_work_plan"; }),
      "a saved plan must fail closed against another source revision");

  auto type_confused = serialized;
  const auto enabled = type_confused.find("\"enabled\":true");
  require(enabled != std::string::npos, "serialized plan must contain typed telemetry state");
  type_confused.replace(enabled, std::string{"\"enabled\":true"}.size(), "\"enabled\":\"true\"");
  const auto confused_result = flux::io::parse_flux_work_plan(type_confused, &board);
  require(!confused_result.ok() &&
              std::any_of(confused_result.diagnostics.begin(), confused_result.diagnostics.end(),
                          [](const auto& diagnostic) {
                            return diagnostic.code == "invalid_work_plan_boolean";
                          }),
          "the plan parser must reject JSON type confusion rather than coerce strings");

  auto scoped_board = two_pad_board();
  const NetId signal_a{"a"};
  const NetId signal_b{"b"};
  const NetId power{"p"};
  const NetId auxiliary{"x"};
  const flux::NetClassId power_class{"power-class"};
  scoped_board.nets = {{signal_a, "SIG_A", 0},
                       {signal_b, "SIG_B", 0},
                       {power, "PWR", 0, {power_class}},
                       {auxiliary, "AUX", 0}};
  flux::NetClass net_class;
  net_class.id = power_class;
  net_class.name = "Power";
  scoped_board.net_classes = {net_class};
  scoped_board.footprints[0].pads = {{ObjectId{"u1-a"}, signal_a, Point{}, {LayerId{"F.Cu"}}},
                                     {ObjectId{"u1-b"}, signal_b, Point{}, {LayerId{"F.Cu"}}}};
  scoped_board.footprints[1].pads = {{ObjectId{"u2-a"}, signal_a, Point{}, {LayerId{"F.Cu"}}},
                                     {ObjectId{"u2-p"}, power, Point{}, {LayerId{"F.Cu"}}}};
  Footprint third;
  third.id = ObjectId{"U3"};
  third.reference = "U3";
  third.position = mm(50.0, 15.0);
  third.courtyard = local_rectangle(4.0, 3.0);
  third.pads = {{ObjectId{"u3-b"}, signal_b, Point{}, {LayerId{"F.Cu"}}},
                {ObjectId{"u3-p"}, power, Point{}, {LayerId{"F.Cu"}}},
                {ObjectId{"u3-x"}, auxiliary, Point{}, {LayerId{"F.Cu"}}}};
  scoped_board.footprints.push_back(third);
  scoped_board.placement_groups = {
      {flux::PlacementGroupId{"core-id"}, "core", {ObjectId{"U1"}, ObjectId{"U2"}}}};
  scoped_board.tracks = {{ObjectId{"track-a"}, signal_a, LayerId{"F.Cu"}, mm(1.0, 1.0),
                          mm(2.0, 1.0), Nanometres::from_mm(0.25), false},
                         {ObjectId{"track-p-locked"}, power, LayerId{"F.Cu"}, mm(3.0, 1.0),
                          mm(4.0, 1.0), Nanometres::from_mm(0.25), true},
                         {ObjectId{"track-x"}, auxiliary, LayerId{"F.Cu"}, mm(5.0, 1.0),
                          mm(6.0, 1.0), Nanometres::from_mm(0.25), false}};
  scoped_board.track_arcs = {{ObjectId{"arc-p"}, power, LayerId{"F.Cu"}, mm(7.0, 1.0), mm(7.5, 1.5),
                              mm(8.0, 1.0), Nanometres::from_mm(0.25), false}};
  scoped_board.vias = {{ObjectId{"via-p"},
                        power,
                        mm(9.0, 1.0),
                        Nanometres::from_mm(0.6),
                        Nanometres::from_mm(0.3),
                        {LayerId{"F.Cu"}, LayerId{"B.Cu"}},
                        false},
                       {ObjectId{"via-x-locked"},
                        auxiliary,
                        mm(10.0, 1.0),
                        Nanometres::from_mm(0.6),
                        Nanometres::from_mm(0.3),
                        {LayerId{"F.Cu"}, LayerId{"B.Cu"}},
                        true}};
  flux::CopperZone immutable_zone;
  immutable_zone.id = ObjectId{"zone-p"};
  immutable_zone.net = power;
  scoped_board.zones = {immutable_zone};

  auto scoped_plan = flux::io::make_flux_work_plan_template(scoped_board, "scoped.kicad_pcb");
  scoped_plan.mode = flux::io::WorkPlanMode::force_reroute_selected;
  flux::io::WorkPlanStep scoped_route;
  scoped_route.id = "scoped-route";
  scoped_route.kind = flux::io::WorkPlanStepKind::route;
  scoped_route.selectors.include_globs = {"SIG_*", "A*"};
  scoped_route.selectors.exclude_globs = {"SIG_B"};
  scoped_route.selectors.net_classes = {"Power"};
  scoped_route.selectors.placement_groups = {"core"};
  scoped_route.selectors.group_scope = flux::io::WorkPlanSelectorScope::internal;
  scoped_route.options = {
      {"atomic_complete_net_routing", true},
      {"capacity_reserve_cost", std::uint64_t{777U}},
      {"capacity_reserve_per_mille", std::uint64_t{300U}},
      {"circular_arc_refinement", true},
      {"clearance_shadow_cost_quantum_nm", std::uint64_t{10'000U}},
      {"corridor_neck_down_width_nm", std::uint64_t{130'000U}},
      {"crossing_safe_diagonal_grid_edges", false},
      {"defer_hard_capacity_ripup_victims_to_pass_tail", true},
      {"diagonal_grid_edges", true},
      {"differential_pair_skew_tuning_batch_assignment", true},
      {"expansion_retry_heuristic_weight_per_mille", std::uint64_t{2'500U}},
      {"former_route_memory_cost", std::uint64_t{1'777U}},
      {"grid_pitch_nm", std::uint64_t{250'000U}},
      {"hard_capacity_start_iteration", std::uint64_t{3U}},
      {"local_pin_escape_pitch_nm", std::uint64_t{50'000U}},
      {"max_corridor_neck_down_length_nm", std::uint64_t{700'000U}},
      {"max_corridor_neck_down_states", std::uint64_t{12'345U}},
      {"max_hard_capacity_blocker_reuse_invalidations_per_failed_net", std::uint64_t{2U}},
      {"max_hard_capacity_conflict_set_states_per_attempt", std::uint64_t{64U}},
      {"max_hard_capacity_ripup_attempts_per_pass", std::uint64_t{2U}},
      {"max_hard_capacity_ripup_nets_per_attempt", std::uint64_t{3U}},
      {"max_final_routability_feedback_passes", std::uint64_t{7U}},
      {"max_local_pin_escape_expansions_per_terminal", std::uint64_t{23'456U}},
      {"max_medial_neck_down_length_nm", std::uint64_t{600'000U}},
      {"max_pin_access_nodes_per_terminal", std::uint64_t{24U}},
      {"max_post_negotiation_sampled_conflict_repair_cones", std::uint64_t{6U}},
      {"max_present_congestion_scale_per_mille", std::uint64_t{4'800U}},
      {"max_router_iterations", std::uint64_t{42U}},
      {"max_terminal_escape_reservation_resources", std::uint64_t{1'234U}},
      {"max_terminal_neck_down_length_nm", std::uint64_t{500'000U}},
      {"max_voronoi_resource_segment_length_nm", std::uint64_t{2'000'000U}},
      {"medial_neck_down_width_nm", std::uint64_t{140'000U}},
      {"physical_envelope_ordering", true},
      {"physical_overflow_history", true},
      {"pin_escape_grid_radius", std::uint64_t{5U}},
      {"post_negotiation_sampled_conflict_repair", true},
      {"present_congestion_escalation_start_iteration", std::uint64_t{3U}},
      {"present_congestion_multiplier_per_mille", std::uint64_t{1'200U}},
      {"priority_nets", std::vector<std::string>{"PWR", "SIG_A"}},
      {"protect_recovered_hard_capacity_ripup_nets", true},
      {"reuse_weighted_search_frontier", true},
      {"river_following_cost", std::uint64_t{333U}},
      {"rollback_failed_hard_capacity_ripup", true},
      {"sampled_route_clearance_reservations", true},
      {"search_diversification", true},
      {"selective_congestion_reroute", true},
      {"selective_full_reroute_interval", std::uint64_t{5U}},
      {"terminal_escape_proximity_cost", std::uint64_t{66U}},
      {"terminal_escape_reservation_cost", std::uint64_t{555U}},
      {"terminal_neck_down_width_nm", std::uint64_t{150'000U}},
      {"via_site_phase_x", std::uint64_t{1U}},
      {"via_site_phase_y", std::uint64_t{3U}},
      {"via_site_pitch_nm", std::uint64_t{1'000'000U}},
      {"width_reduction_cost_per_grid_step", std::uint64_t{444U}},
      {"xpbd_feedback", true},
      {"xpbd_iterations", std::uint64_t{17U}},
      {"xpbd_maximum_step_nm", std::uint64_t{80'000U}},
      {"xpbd_prefer_gpu", true}};
  scoped_plan.steps = {scoped_route};
  const auto expansion = flux::io::expand_flux_work_plan(scoped_board, scoped_plan);
  require(
      expansion.ok() && expansion.steps.size() == 1U &&
          expansion.steps.front().nets == std::vector<NetId>{signal_a, power, auxiliary} &&
          expansion.steps.front().footprints ==
              std::vector<ObjectId>{ObjectId{"U1"}, ObjectId{"U2"}} &&
          expansion.steps.front().wildcard_only_nets == std::vector<NetId>{auxiliary} &&
          expansion.copper_authority.reroutable_nets == std::vector<NetId>{signal_a, power} &&
          expansion.copper_authority.removable_tracks ==
              std::vector<ObjectId>{ObjectId{"track-a"}} &&
          expansion.copper_authority.removable_track_arcs ==
              std::vector<ObjectId>{ObjectId{"arc-p"}} &&
          expansion.copper_authority.removable_vias == std::vector<ObjectId>{ObjectId{"via-p"}} &&
          expansion.copper_authority.immutable_tracks ==
              std::vector<ObjectId>{ObjectId{"track-p-locked"}, ObjectId{"track-x"}} &&
          expansion.copper_authority.immutable_vias ==
              std::vector<ObjectId>{ObjectId{"via-x-locked"}} &&
          expansion.copper_authority.immutable_zones == std::vector<ObjectId>{ObjectId{"zone-p"}} &&
          expansion.selected_net_mask == std::vector<std::uint8_t>({1U, 0U, 1U, 1U}) &&
          expansion.reroutable_net_mask == std::vector<std::uint8_t>({1U, 0U, 1U, 0U}),
      "selector expansion must union exact physical scopes, apply exclusions last, expose "
      "device-ready masks, and keep wildcard-only or locked source copper immutable");
  const auto compiled = flux::io::compile_flux_work_plan_routes(scoped_board, scoped_plan);
  require(
      compiled.ok() && compiled.route_steps.size() == 1U &&
          compiled.route_steps.front().selected_nets ==
              std::vector<NetId>{signal_a, power, auxiliary} &&
          compiled.route_steps.front().priority_nets == std::vector<NetId>{power, signal_a} &&
          compiled.route_steps.front().configuration.planner.global.grid_pitch ==
              Nanometres{250'000} &&
          compiled.route_steps.front().configuration.planner.global.via_site_pitch ==
              Nanometres{1'000'000} &&
          compiled.route_steps.front().configuration.planner.global.via_site_phase_x == 1U &&
          compiled.route_steps.front().configuration.planner.global.via_site_phase_y == 3U &&
          compiled.route_steps.front().configuration.planner.global.max_pin_escape_grid_radius ==
              5U &&
          compiled.route_steps.front()
                  .configuration.planner.global.max_pin_access_nodes_per_terminal == 24U &&
          compiled.route_steps.front().configuration.planner.global.local_pin_escape_pitch ==
              Nanometres{50'000} &&
          compiled.route_steps.front()
                  .configuration.planner.global.max_local_pin_escape_expansions_per_terminal ==
              23'456U &&
          compiled.route_steps.front().configuration.planner.global.terminal_neck_down_width ==
              Nanometres{150'000} &&
          compiled.route_steps.front().configuration.planner.global.max_terminal_neck_down_length ==
              Nanometres{500'000} &&
          compiled.route_steps.front().configuration.planner.global.medial_neck_down_width ==
              Nanometres{140'000} &&
          compiled.route_steps.front().configuration.planner.global.max_medial_neck_down_length ==
              Nanometres{600'000} &&
          compiled.route_steps.front().configuration.planner.global.corridor_neck_down_width ==
              Nanometres{130'000} &&
          compiled.route_steps.front().configuration.planner.global.max_corridor_neck_down_length ==
              Nanometres{700'000} &&
          compiled.route_steps.front().configuration.planner.global.max_corridor_neck_down_states ==
              12'345U &&
          compiled.route_steps.front()
                  .configuration.planner.global.max_voronoi_resource_segment_length ==
              Nanometres{2'000'000} &&
          compiled.route_steps.front().configuration.planner.global.max_iterations == 42U &&
          compiled.route_steps.front().max_final_routability_feedback_passes == 7U &&
          compiled.route_steps.front().post_negotiation_sampled_conflict_repair &&
          compiled.route_steps.front().max_post_negotiation_sampled_conflict_repair_cones == 6U &&
          compiled.route_steps.front().configuration.planner.global.allow_diagonal_grid_edges &&
          !compiled.route_steps.front()
               .configuration.planner.global.allow_crossing_safe_diagonal_grid_edges &&
          compiled.route_steps.front()
              .configuration.planner.global.reserve_sampled_route_clearance &&
          compiled.route_steps.front()
                  .configuration.planner.global.expansion_retry_heuristic_weight_per_mille ==
              2'500U &&
          compiled.route_steps.front()
              .configuration.planner.global.reuse_weighted_search_frontier &&
          compiled.route_steps.front()
                  .configuration.planner.global.present_congestion_multiplier_per_mille == 1'200U &&
          compiled.route_steps.front()
                  .configuration.planner.global.max_present_congestion_scale_per_mille == 4'800U &&
          compiled.route_steps.front()
                  .configuration.planner.global.present_congestion_escalation_start_iteration ==
              3U &&
          compiled.route_steps.front()
              .configuration.planner.global.scale_voronoi_history_by_overflow &&
          compiled.route_steps.front()
              .configuration.planner.global.order_routing_groups_by_physical_envelope &&
          compiled.route_steps.front().configuration.planner.global.clearance_shadow_cost_quantum ==
              Nanometres{10'000} &&
          compiled.route_steps.front().configuration.planner.global.capacity_reserve_per_mille ==
              300U &&
          compiled.route_steps.front().configuration.planner.global.capacity_reserve_cost == 777U &&
          compiled.route_steps.front().configuration.planner.global.river_following_cost == 333U &&
          compiled.route_steps.front()
                  .configuration.planner.global.terminal_escape_reservation_cost == 555U &&
          compiled.route_steps.front()
                  .configuration.planner.global.terminal_escape_proximity_cost == 66U &&
          compiled.route_steps.front()
                  .configuration.planner.global.max_terminal_escape_reservation_resources ==
              1'234U &&
          compiled.route_steps.front()
                  .configuration.planner.global.width_reduction_cost_per_grid_step == 444U &&
          compiled.route_steps.front().configuration.planner.global.hard_capacity_start_iteration ==
              3U &&
          compiled.route_steps.front()
                  .configuration.planner.global.max_hard_capacity_ripup_attempts_per_pass == 2U &&
          compiled.route_steps.front()
                  .configuration.planner.global.max_hard_capacity_ripup_nets_per_attempt == 3U &&
          compiled.route_steps.front()
                  .configuration.planner.global.max_hard_capacity_conflict_set_states_per_attempt ==
              64U &&
          compiled.route_steps.front()
                  .configuration.planner.global
                  .max_hard_capacity_blocker_reuse_invalidations_per_failed_net == 2U &&
          compiled.route_steps.front()
              .configuration.planner.global.rollback_failed_hard_capacity_ripup &&
          compiled.route_steps.front()
              .configuration.planner.global.defer_hard_capacity_ripup_victims_to_pass_tail &&
          compiled.route_steps.front()
              .configuration.planner.global.protect_recovered_hard_capacity_ripup_nets &&
          compiled.route_steps.front()
                  .configuration.planner.global.selective_full_reroute_interval == 5U &&
          compiled.route_steps.front().configuration.planner.global.former_route_memory_cost ==
              1'777U &&
          compiled.route_steps.front().configuration.planner.global.enable_search_diversification &&
          compiled.route_steps.front().configuration.planner.global.atomic_complete_net_routing &&
          compiled.route_steps.front().configuration.planner.global.selective_congestion_reroute &&
          compiled.route_steps.front().configuration.planner.regional.enable_xpbd_relaxation &&
          compiled.route_steps.front().configuration.planner.regional.enable_xpbd_feedback &&
          compiled.route_steps.front().configuration.planner.regional.xpbd.iterations == 17U &&
          compiled.route_steps.front().configuration.planner.regional.xpbd.maximum_step_nm ==
              80'000 &&
          compiled.route_steps.front().configuration.planner.regional.xpbd.prefer_gpu &&
          compiled.route_steps.front().configuration.detailed.circular_arc_refinement &&
          compiled.route_steps.front().configuration.detailed.differential_pair_skew_tuning &&
          compiled.route_steps.front()
              .configuration.detailed.differential_pair_skew_tuning_batch_assignment &&
          compiled.route_steps.front().configuration.detailed.existing_routing ==
              flux::ExistingRoutingPolicy::replace_unlocked_for_routed_nets &&
          compiled.route_steps.front().configuration.detailed.existing_routing_replacement_scope ==
              std::optional<std::vector<NetId>>{std::vector<NetId>{signal_a, power}},
      "route options must compile into the shared autoroute_v1 configuration without giving "
      "wildcard-only nets source-copper replacement authority");

  auto invalid_recovery_topology_plan = scoped_plan;
  invalid_recovery_topology_plan.steps.front().options["strict_medial_topology"] = false;
  invalid_recovery_topology_plan.steps.front().options["direct_medial_edges"] = false;
  const auto invalid_recovery_topology =
      flux::io::compile_flux_work_plan_routes(scoped_board, invalid_recovery_topology_plan);
  require(!invalid_recovery_topology.ok() &&
              std::any_of(invalid_recovery_topology.diagnostics.begin(),
                          invalid_recovery_topology.diagnostics.end(),
                          [](const auto& diagnostic) {
                            return diagnostic.code == "invalid_work_plan_route_option_dependency";
                          }),
          "hard-capacity WorkPlan recovery must fail closed without a represented medial "
          "topology");

  auto incomplete_recovery_bounds_plan = scoped_plan;
  incomplete_recovery_bounds_plan.steps.front().options.erase(
      "max_hard_capacity_ripup_nets_per_attempt");
  const auto incomplete_recovery_bounds =
      flux::io::compile_flux_work_plan_routes(scoped_board, incomplete_recovery_bounds_plan);
  require(!incomplete_recovery_bounds.ok() &&
              std::any_of(incomplete_recovery_bounds.diagnostics.begin(),
                          incomplete_recovery_bounds.diagnostics.end(),
                          [](const auto& diagnostic) {
                            return diagnostic.code == "invalid_work_plan_route_option_dependency";
                          }),
          "hard-capacity WorkPlan recovery must require both bounded attempt and victim counts");

  auto invalid_search_pressure_plan = scoped_plan;
  invalid_search_pressure_plan.steps.front().options["expansion_retry_heuristic_weight_per_mille"] =
      std::uint64_t{0U};
  const auto invalid_search_pressure =
      flux::io::compile_flux_work_plan_routes(scoped_board, invalid_search_pressure_plan);
  require(!invalid_search_pressure.ok() &&
              std::any_of(invalid_search_pressure.diagnostics.begin(),
                          invalid_search_pressure.diagnostics.end(),
                          [](const auto& diagnostic) {
                            return diagnostic.code == "invalid_work_plan_route_option_dependency";
                          }),
          "weighted-frontier WorkPlan search must fail closed without a retry weight");

  auto incomplete_neck_down_plan = scoped_plan;
  incomplete_neck_down_plan.steps.front().options.erase("max_terminal_neck_down_length_nm");
  const auto incomplete_neck_down =
      flux::io::compile_flux_work_plan_routes(scoped_board, incomplete_neck_down_plan);
  require(!incomplete_neck_down.ok() &&
              std::any_of(incomplete_neck_down.diagnostics.begin(),
                          incomplete_neck_down.diagnostics.end(),
                          [](const auto& diagnostic) {
                            return diagnostic.code == "invalid_work_plan_route_option_dependency";
                          }),
          "terminal neck-down WorkPlan geometry must require a paired physical length bound");

  auto orphan_sampled_repair_budget_plan = scoped_plan;
  orphan_sampled_repair_budget_plan.steps.front()
      .options["post_negotiation_sampled_conflict_repair"] = false;
  const auto orphan_sampled_repair_budget =
      flux::io::compile_flux_work_plan_routes(scoped_board, orphan_sampled_repair_budget_plan);
  require(!orphan_sampled_repair_budget.ok() &&
              std::any_of(orphan_sampled_repair_budget.diagnostics.begin(),
                          orphan_sampled_repair_budget.diagnostics.end(),
                          [](const auto& diagnostic) {
                            return diagnostic.code == "invalid_work_plan_route_option_dependency";
                          }),
          "a WorkPlan sampled-conflict repair budget must not be accepted when the repair "
          "transaction is disabled");

  auto refill_plan = scoped_plan;
  refill_plan.steps.front().options["refillable_zone_fills"] = true;
  refill_plan.steps.front().options["differential_pair_skew_tuning"] = false;
  refill_plan.steps.front().options["differential_pair_skew_tuning_batch_assignment"] = false;
  const auto refill_compile = flux::io::compile_flux_work_plan_routes(scoped_board, refill_plan);
  require(refill_compile.ok() && refill_compile.route_steps.size() == 1U &&
              refill_compile.route_steps.front().refillable_zone_fills &&
              refill_compile.route_steps.front()
                  .configuration.planner.global.treat_saved_zone_fills_as_refillable &&
              refill_compile.route_steps.front()
                  .configuration.planner.regional.treat_saved_zone_fills_as_refillable &&
              refill_compile.route_steps.front()
                  .configuration.detailed.defer_saved_zone_fill_clearance_to_refill &&
              !refill_compile.route_steps.front().configuration.detailed.pull_tight_45_degrees &&
              !refill_compile.route_steps.front().configuration.detailed.continuous_refinement &&
              !refill_compile.route_steps.front()
                   .configuration.detailed.topology_locked_medial_materialization,
          "refill-required WorkPlan routing must compile the same conservative provisional "
          "configuration as the direct KiCad workflow");
  auto nonterminal_refill_plan = refill_plan;
  flux::io::WorkPlanStep post_refill_validation;
  post_refill_validation.id = "post-refill-validation";
  post_refill_validation.kind = flux::io::WorkPlanStepKind::validation;
  post_refill_validation.depends_on = {refill_plan.steps.front().id};
  nonterminal_refill_plan.steps.push_back(post_refill_validation);
  const auto nonterminal_refill_compile =
      flux::io::compile_flux_work_plan_routes(scoped_board, nonterminal_refill_plan);
  require(!nonterminal_refill_compile.ok() &&
              std::any_of(nonterminal_refill_compile.diagnostics.begin(),
                          nonterminal_refill_compile.diagnostics.end(),
                          [](const auto& diagnostic) {
                            return diagnostic.code == "invalid_work_plan_route_option_dependency";
                          }),
          "a refill-required route must terminate the private plan because later steps would "
          "observe stale saved-fill geometry");

  auto diagnostic_preview_plan = scoped_plan;
  diagnostic_preview_plan.mode = flux::io::WorkPlanMode::preview;
  diagnostic_preview_plan.steps.front().options["retain_incomplete_diagnostic_preview"] = true;
  const auto diagnostic_preview_compile =
      flux::io::compile_flux_work_plan_routes(scoped_board, diagnostic_preview_plan);
  require(diagnostic_preview_compile.ok() && diagnostic_preview_compile.route_steps.size() == 1U &&
              diagnostic_preview_compile.route_steps.front().retain_incomplete_diagnostic_preview,
          "the diagnostic incomplete-copper transaction must compile only as explicit preview "
          "policy");
  auto mutating_diagnostic_preview_plan = diagnostic_preview_plan;
  mutating_diagnostic_preview_plan.mode = flux::io::WorkPlanMode::new_routing;
  const auto mutating_diagnostic_preview_compile =
      flux::io::compile_flux_work_plan_routes(scoped_board, mutating_diagnostic_preview_plan);
  require(!mutating_diagnostic_preview_compile.ok() &&
              std::any_of(mutating_diagnostic_preview_compile.diagnostics.begin(),
                          mutating_diagnostic_preview_compile.diagnostics.end(),
                          [](const auto& diagnostic) {
                            return diagnostic.code == "invalid_work_plan_route_option_dependency";
                          }),
          "an incomplete diagnostic candidate must never compile in a mutating WorkPlan mode");
  auto refill_diagnostic_preview_plan = diagnostic_preview_plan;
  refill_diagnostic_preview_plan.steps.front().options["refillable_zone_fills"] = true;
  refill_diagnostic_preview_plan.steps.front().options["differential_pair_skew_tuning"] = false;
  refill_diagnostic_preview_plan.steps.front()
      .options["differential_pair_skew_tuning_batch_assignment"] = false;
  const auto refill_diagnostic_preview_compile =
      flux::io::compile_flux_work_plan_routes(scoped_board, refill_diagnostic_preview_plan);
  require(!refill_diagnostic_preview_compile.ok(),
          "incomplete diagnostic copper must not enter a zone-refill transaction");

  auto wildcard_plan = scoped_plan;
  wildcard_plan.steps.front().selectors.net_classes.clear();
  wildcard_plan.steps.front().selectors.placement_groups.clear();
  const auto wildcard_expansion = flux::io::expand_flux_work_plan(scoped_board, wildcard_plan);
  require(!wildcard_expansion.ok() && std::any_of(wildcard_expansion.diagnostics.begin(),
                                                  wildcard_expansion.diagnostics.end(),
                                                  [](const auto& diagnostic) {
                                                    return diagnostic.code ==
                                                           "wildcard_only_force_reroute_scope";
                                                  }),
          "glob selection must never infer authority to remove pre-existing copper");

  auto allowlisted_plan = scoped_plan;
  allowlisted_plan.mode = flux::io::WorkPlanMode::allowlisted_ripup;
  allowlisted_plan.ripup_allowlist = {"AUX"};
  const auto allowlisted = flux::io::expand_flux_work_plan(scoped_board, allowlisted_plan);
  require(allowlisted.ok() &&
              allowlisted.copper_authority.reroutable_nets == std::vector<NetId>{auxiliary} &&
              allowlisted.copper_authority.removable_tracks ==
                  std::vector<ObjectId>{ObjectId{"track-x"}} &&
              allowlisted.copper_authority.immutable_vias ==
                  std::vector<ObjectId>{ObjectId{"via-p"}, ObjectId{"via-x-locked"}},
          "an exact allowlist may expose only unlocked copper on named nets while locked copper "
          "remains immutable");
  const auto compiled_allowlisted =
      flux::io::compile_flux_work_plan_routes(scoped_board, allowlisted_plan);
  require(compiled_allowlisted.ok() && compiled_allowlisted.route_steps.size() == 1U &&
              compiled_allowlisted.route_steps.front().selected_nets ==
                  std::vector<NetId>{signal_a, power, auxiliary} &&
              compiled_allowlisted.route_steps.front()
                      .configuration.detailed.existing_routing_replacement_scope ==
                  std::optional<std::vector<NetId>>{std::vector<NetId>{auxiliary}},
          "allowlisted blockers must join the planned route set while only their exact unlocked "
          "copper enters replacement authority");

  auto unsupported_option_plan = scoped_plan;
  unsupported_option_plan.steps.front().options["not_a_router_option"] = true;
  const auto unsupported =
      flux::io::compile_flux_work_plan_routes(scoped_board, unsupported_option_plan);
  require(!unsupported.ok() &&
              std::any_of(unsupported.diagnostics.begin(), unsupported.diagnostics.end(),
                          [](const auto& diagnostic) {
                            return diagnostic.code == "unsupported_work_plan_route_option";
                          }),
          "unknown route options must fail compilation instead of being ignored");

  auto coupled_board = two_pad_board();
  const NetId pair_positive{"net-1"};
  const NetId pair_negative{"net-2"};
  coupled_board.nets = {{pair_positive, "PAIR_P", 0}, {pair_negative, "PAIR_N", 0}};
  coupled_board.footprints[0].position = mm(20.0, 38.0);
  coupled_board.footprints[1].position = mm(80.0, 38.0);
  coupled_board.footprints[0].courtyard = local_rectangle(0.1, 0.1);
  coupled_board.footprints[1].courtyard = local_rectangle(0.1, 0.1);
  auto negative_start = coupled_board.footprints[0];
  negative_start.id = ObjectId{"U1N-work-plan"};
  negative_start.reference = "U1N-work-plan";
  negative_start.position = mm(20.0, 42.0);
  negative_start.pads.front().id = ObjectId{"U1N-work-plan-pad"};
  negative_start.pads.front().net = pair_negative;
  auto negative_end = coupled_board.footprints[1];
  negative_end.id = ObjectId{"U2N-work-plan"};
  negative_end.reference = "U2N-work-plan";
  negative_end.position = mm(80.0, 42.0);
  negative_end.pads.front().id = ObjectId{"U2N-work-plan-pad"};
  negative_end.pads.front().net = pair_negative;
  coupled_board.footprints.push_back(std::move(negative_start));
  coupled_board.footprints.push_back(std::move(negative_end));
  coupled_board.tracks = {{ObjectId{"pair-p-track"}, pair_positive, LayerId{"F.Cu"}, mm(20.0, 38.0),
                           mm(80.0, 38.0), Nanometres::from_mm(0.25), false},
                          {ObjectId{"pair-n-track"}, pair_negative, LayerId{"F.Cu"}, mm(20.0, 42.0),
                           mm(80.0, 42.0), Nanometres::from_mm(0.25), false}};
  coupled_board.differential_pairs = {{flux::DifferentialPairId{"pair-1"}, "PAIR", pair_positive,
                                       pair_negative, Nanometres::from_mm(0.2), std::nullopt}};

  auto pair_plan = flux::io::make_flux_work_plan_template(coupled_board, "pair.kicad_pcb");
  pair_plan.mode = flux::io::WorkPlanMode::force_reroute_selected;
  flux::io::WorkPlanStep pair_step;
  pair_step.id = "route-pair";
  pair_step.kind = flux::io::WorkPlanStepKind::differential_pair;
  pair_step.selectors.exact_nets = {"PAIR_P", "PAIR_N"};
  pair_step.options = {{"checkpoint_objective", std::string{"aggregate-overflow"}},
                       {"crossing_safe_diagonal_grid_edges", false},
                       {"direct_medial_edges", false},
                       {"diversify_pin_access_rings", true},
                       {"grid_pitch_nm", std::uint64_t{5'000'000U}},
                       {"interleave_routability_feedback", false},
                       {"max_pin_access_nodes_per_terminal", std::uint64_t{32U}},
                       {"max_router_iterations", std::uint64_t{2U}},
                       {"negotiate_voronoi_lane_conflicts", false},
                       {"physical_envelope_ordering", false},
                       {"physical_overflow_history", false},
                       {"pin_escape_grid_radius", std::uint64_t{2U}},
                       {"routability_feedback_retries", std::uint64_t{0U}},
                       {"strict_medial_topology", false},
                       {"topology_locked_medial_materialization", false}};
  pair_plan.steps = {pair_step};
  const auto pair_compile = flux::io::compile_flux_work_plan_routes(coupled_board, pair_plan);
  require(pair_compile.ok() && pair_compile.route_steps.size() == 1U &&
              pair_compile.route_steps.front().kind ==
                  flux::io::WorkPlanStepKind::differential_pair &&
              pair_compile.route_steps.front().selected_nets ==
                  std::vector<NetId>{pair_positive, pair_negative} &&
              pair_compile.expansion.copper_authority.reroutable_nets ==
                  std::vector<NetId>{pair_positive, pair_negative} &&
              pair_compile.expansion.copper_authority.removable_tracks ==
                  std::vector<ObjectId>{ObjectId{"pair-n-track"}, ObjectId{"pair-p-track"}},
          "a differential-pair step must compile as one atomic route transaction with exact "
          "selected-copper authority");

  auto connected_pair_plan = pair_plan;
  connected_pair_plan.mode = flux::io::WorkPlanMode::new_routing;
  const auto connected_pair_execution =
      flux::io::execute_flux_work_plan(coupled_board, connected_pair_plan);
  require(connected_pair_execution.ok() && connected_pair_execution.whole_board_accepted &&
              connected_pair_execution.steps.size() == 1U &&
              connected_pair_execution.steps.front().kind ==
                  flux::io::WorkPlanStepKind::differential_pair &&
              connected_pair_execution.steps.front().status ==
                  flux::io::WorkPlanStepExecutionStatus::completed,
          "an already-connected declared pair must execute through the coupled adapter without "
          "mutating authoritative copper");
  auto open_pair_board = coupled_board;
  open_pair_board.tracks.clear();
  const auto open_pair_execution =
      flux::io::execute_flux_work_plan(open_pair_board, connected_pair_plan);
  require(open_pair_execution.ok() && open_pair_execution.whole_board_accepted &&
              open_pair_execution.candidate && open_pair_execution.steps.size() == 1U &&
              open_pair_execution.steps.front().status ==
                  flux::io::WorkPlanStepExecutionStatus::completed &&
              open_pair_execution.steps.front().global_unrouted_nets == 0U &&
              !open_pair_execution.candidate->tracks.empty(),
          "an open declared pair must complete global, regional, detailed, connectivity, and DRC "
          "acceptance as one coupled WorkPlan transaction: " +
              flux::io::serialize_flux_work_plan_execution_summary(open_pair_execution));

  auto invalid_pair_board = coupled_board;
  invalid_pair_board.nets.push_back({NetId{"ordinary"}, "ORDINARY", 0});
  auto invalid_pair_plan =
      flux::io::make_flux_work_plan_template(invalid_pair_board, "pair.kicad_pcb");
  flux::io::WorkPlanStep invalid_pair_step;
  invalid_pair_step.id = "invalid-pair";
  invalid_pair_step.kind = flux::io::WorkPlanStepKind::differential_pair;
  invalid_pair_step.selectors.exact_nets = {"ORDINARY"};
  invalid_pair_plan.steps = {invalid_pair_step};
  const auto invalid_pair_compile =
      flux::io::compile_flux_work_plan_routes(invalid_pair_board, invalid_pair_plan);
  require(!invalid_pair_compile.ok() &&
              std::any_of(invalid_pair_compile.diagnostics.begin(),
                          invalid_pair_compile.diagnostics.end(),
                          [](const auto& diagnostic) {
                            return diagnostic.code == "invalid_work_plan_differential_pair_scope";
                          }),
          "a differential-pair step must reject ordinary nets instead of degrading to a generic "
          "route");

  auto bus_board = coupled_board;
  bus_board.differential_pairs.clear();
  bus_board.routing_buses = {{flux::RoutingBusId{"bus-1"},
                              "PAIR_BUS",
                              {pair_positive, pair_negative},
                              Nanometres::from_mm(0.5)}};
  auto bus_plan = flux::io::make_flux_work_plan_template(bus_board, "bus.kicad_pcb");
  bus_plan.mode = flux::io::WorkPlanMode::force_reroute_selected;
  flux::io::WorkPlanStep bus_step;
  bus_step.id = "route-bus";
  bus_step.kind = flux::io::WorkPlanStepKind::bus;
  bus_step.selectors.exact_nets = {"PAIR_P", "PAIR_N"};
  bus_step.options = pair_step.options;
  bus_plan.steps = {bus_step};
  const auto bus_compile = flux::io::compile_flux_work_plan_routes(bus_board, bus_plan);
  require(bus_compile.ok() && bus_compile.route_steps.size() == 1U &&
              bus_compile.route_steps.front().kind == flux::io::WorkPlanStepKind::bus &&
              bus_compile.expansion.copper_authority.reroutable_nets ==
                  std::vector<NetId>{pair_positive, pair_negative},
          "an ordered-bus step must compile as one atomic route transaction with exact "
          "selected-copper authority");
  auto connected_bus_plan = bus_plan;
  connected_bus_plan.mode = flux::io::WorkPlanMode::new_routing;
  const auto connected_bus_execution =
      flux::io::execute_flux_work_plan(bus_board, connected_bus_plan);
  require(connected_bus_execution.ok() && connected_bus_execution.whole_board_accepted &&
              connected_bus_execution.steps.size() == 1U &&
              connected_bus_execution.steps.front().kind == flux::io::WorkPlanStepKind::bus &&
              connected_bus_execution.steps.front().status ==
                  flux::io::WorkPlanStepExecutionStatus::completed,
          "an already-connected declared bus must execute through the coupled adapter without "
          "mutating authoritative copper");
  auto open_bus_board = bus_board;
  open_bus_board.tracks.clear();
  const auto open_bus_execution =
      flux::io::execute_flux_work_plan(open_bus_board, connected_bus_plan);
  require(open_bus_execution.ok() && open_bus_execution.whole_board_accepted &&
              open_bus_execution.candidate && open_bus_execution.steps.size() == 1U &&
              open_bus_execution.steps.front().status ==
                  flux::io::WorkPlanStepExecutionStatus::completed &&
              open_bus_execution.steps.front().global_unrouted_nets == 0U &&
              !open_bus_execution.candidate->tracks.empty(),
          "an open declared bus must complete global, regional, detailed, connectivity, and DRC "
          "acceptance as one coupled WorkPlan transaction: " +
              flux::io::serialize_flux_work_plan_execution_summary(open_bus_execution));

  auto connected = two_pad_board();
  connected.tracks = {{ObjectId{"source-track"}, NetId{"net-1"}, LayerId{"F.Cu"}, mm(20.0, 40.0),
                       mm(80.0, 40.0), Nanometres::from_mm(0.25), false}};
  auto executable_plan = flux::io::make_flux_work_plan_template(connected, "board.kicad_pcb");
  executable_plan.mode = flux::io::WorkPlanMode::force_reroute_selected;
  flux::io::WorkPlanStep executable_route;
  executable_route.id = "reroute";
  executable_route.kind = flux::io::WorkPlanStepKind::route;
  executable_route.selectors.exact_nets = {"SIGNAL"};
  executable_route.options = {{"grid_pitch_nm", std::uint64_t{1'000'000U}},
                              {"max_expansions_per_connection", std::uint64_t{100'000U}},
                              {"max_router_iterations", std::uint64_t{4U}}};
  flux::io::WorkPlanStep executable_validation;
  executable_validation.id = "validate";
  executable_validation.kind = flux::io::WorkPlanStepKind::validation;
  executable_validation.depends_on = {"reroute"};
  executable_plan.steps = {executable_route, executable_validation};
  CollectingTelemetrySink work_plan_telemetry;
  const auto execution =
      flux::io::execute_flux_work_plan(connected, executable_plan, {}, &work_plan_telemetry);
  require(
      execution.ok() && execution.whole_board_accepted && execution.candidate_ready_for_export &&
          execution.candidate && execution.steps.size() == 2U &&
          execution.steps.front().status == flux::io::WorkPlanStepExecutionStatus::completed &&
          execution.steps.front().replacement_nets == std::vector<NetId>{NetId{"net-1"}} &&
          std::none_of(execution.candidate->tracks.begin(), execution.candidate->tracks.end(),
                       [](const auto& track) { return track.id == ObjectId{"source-track"}; }) &&
          connected.tracks.size() == 1U &&
          connected.tracks.front().id == ObjectId{"source-track"} &&
          flux::io::serialize_flux_work_plan_execution_summary(execution) ==
              flux::io::serialize_flux_work_plan_execution_summary(execution) &&
          std::any_of(work_plan_telemetry.events.begin(), work_plan_telemetry.events.end(),
                      [](const auto& event) {
                        return event.kind == flux::RouterTelemetryEventKind::search_summary &&
                               event.stable_id == "work-plan-step:reroute" &&
                               event.detail == "Work plan step completed: reroute";
                      }),
      "work-plan execution must reroute on a copy, replace only exact unlocked source copper, "
      "and commit a deterministic exact-clean checkpoint through validation");

  auto unrelated_open = connected;
  const NetId unrelated_net{"unrelated-open"};
  unrelated_open.nets.push_back({unrelated_net, "UNRELATED_OPEN", 0});
  auto unrelated_start = unrelated_open.footprints.front();
  unrelated_start.id = ObjectId{"U3-unrelated"};
  unrelated_start.reference = "U3-unrelated";
  unrelated_start.position = mm(20.0, 20.0);
  unrelated_start.pads.front().id = ObjectId{"U3-unrelated-pad"};
  unrelated_start.pads.front().net = unrelated_net;
  auto unrelated_end = unrelated_start;
  unrelated_end.id = ObjectId{"U4-unrelated"};
  unrelated_end.reference = "U4-unrelated";
  unrelated_end.position = mm(80.0, 20.0);
  unrelated_end.pads.front().id = ObjectId{"U4-unrelated-pad"};
  unrelated_open.footprints.push_back(std::move(unrelated_start));
  unrelated_open.footprints.push_back(std::move(unrelated_end));
  auto route_only_plan = executable_plan;
  route_only_plan.steps.pop_back();
  const auto unrelated_open_execution =
      flux::io::execute_flux_work_plan(unrelated_open, route_only_plan);
  require(unrelated_open_execution.ok() && !unrelated_open_execution.whole_board_accepted &&
              !unrelated_open_execution.candidate_ready_for_export &&
              unrelated_open_execution.final_acceptance.open_connections > 0U,
          "selected-net step success must not authorize publication while an unrelated whole-board "
          "connection remains open: " +
              flux::io::serialize_flux_work_plan_execution_summary(unrelated_open_execution));

  auto preview_plan = executable_plan;
  preview_plan.mode = flux::io::WorkPlanMode::preview;
  const auto preview = flux::io::execute_flux_work_plan(connected, preview_plan);
  require(preview.ok() && preview.preview && !preview.candidate_ready_for_export &&
              preview.steps.front().selected_nets.empty() && preview.candidate &&
              preview.candidate->tracks == connected.tracks &&
              preview.candidate->footprints == connected.footprints,
          "preview mode must observe an already complete selected net without replacing or "
          "export-authorizing its source copper");

  auto placement_source = two_pad_board();
  Footprint placement_bystander;
  placement_bystander.id = ObjectId{"BYSTANDER"};
  placement_bystander.reference = "BYSTANDER";
  placement_bystander.position = mm(50.0, 15.0);
  placement_bystander.courtyard = local_rectangle(2.0, 2.0);
  placement_source.footprints.push_back(placement_bystander);
  auto placement_plan =
      flux::io::make_flux_work_plan_template(placement_source, "placement.kicad_pcb");
  placement_plan.mode = flux::io::WorkPlanMode::new_routing;
  flux::io::WorkPlanStep placement_step;
  placement_step.id = "place-and-route";
  placement_step.kind = flux::io::WorkPlanStepKind::placement;
  placement_step.selectors.footprints = {"U1", "U2"};
  placement_step.selectors.group_scope = flux::io::WorkPlanSelectorScope::touching;
  placement_step.options = {{"checkpoint_objective", std::string{"aggregate-overflow"}},
                            {"crossing_safe_diagonal_grid_edges", false},
                            {"direct_medial_edges", false},
                            {"enable_incremental_routing", false},
                            {"enable_routing_guided_moves", false},
                            {"grid_pitch_nm", std::uint64_t{5'000'000U}},
                            {"max_candidate_evaluations", std::uint64_t{2U}},
                            {"max_candidate_evaluation_workers", std::uint64_t{1U}},
                            {"max_expansions_per_connection", std::uint64_t{100'000U}},
                            {"max_full_routing_candidates_per_wave", std::uint64_t{1U}},
                            {"max_placement_passes", std::uint64_t{1U}},
                            {"max_router_iterations", std::uint64_t{2U}},
                            {"negotiate_voronoi_lane_conflicts", false},
                            {"physical_envelope_ordering", false},
                            {"physical_overflow_history", false},
                            {"strict_medial_topology", false},
                            {"topology_locked_medial_materialization", false}};
  placement_plan.steps = {placement_step};
  const auto placement_compile =
      flux::io::compile_flux_work_plan_routes(placement_source, placement_plan);
  require(placement_compile.ok() && placement_compile.route_steps.size() == 1U &&
              placement_compile.route_steps.front().placement &&
              placement_compile.route_steps.front().selected_footprints ==
                  std::vector<ObjectId>{ObjectId{"U1"}, ObjectId{"U2"}} &&
              placement_compile.route_steps.front().selected_nets ==
                  std::vector<NetId>{NetId{"net-1"}} &&
              placement_compile.route_steps.front().placement->max_passes == 1U &&
              placement_compile.route_steps.front().placement->max_candidate_evaluations == 2U &&
              placement_compile.route_steps.front()
                      .configuration.detailed.existing_routing_replacement_scope ==
                  std::optional<std::vector<NetId>>{std::vector<NetId>{}},
          "a placement step must compile selected footprints and all touching nets through the "
          "shared router plus bounded integrated-optimizer configuration");
  CollectingTelemetrySink placement_telemetry;
  const auto placement_execution =
      flux::io::execute_flux_work_plan(placement_source, placement_plan, {}, &placement_telemetry);
  const bool bystander_preserved =
      placement_execution.candidate &&
      std::any_of(placement_execution.candidate->footprints.begin(),
                  placement_execution.candidate->footprints.end(),
                  [&placement_bystander](const auto& footprint) {
                    return footprint.id == ObjectId{"BYSTANDER"} &&
                           footprint.position == placement_bystander.position && !footprint.locked;
                  });
  require(placement_execution.ok() && placement_execution.whole_board_accepted &&
              placement_execution.candidate_ready_for_export && placement_execution.candidate &&
              placement_execution.steps.size() == 1U &&
              placement_execution.steps.front().status ==
                  flux::io::WorkPlanStepExecutionStatus::completed &&
              placement_execution.steps.front().placement_attempted &&
              placement_execution.steps.front().placement_candidate_evaluations > 0U &&
              placement_execution.steps.front().placement_full_route_evaluations > 0U &&
              bystander_preserved && placement_source.tracks.empty() &&
              !placement_execution.candidate->tracks.empty() &&
              std::any_of(placement_telemetry.events.begin(), placement_telemetry.events.end(),
                          [](const auto& event) {
                            return event.stable_id == "work-plan-step:place-and-route" &&
                                   event.total_groups == 2U && event.search_expansions > 0U;
                          }),
          "placement execution must route on a private board, preserve unselected transforms and "
          "locks, and commit only an exact accepted integrated checkpoint: " +
              flux::io::serialize_flux_work_plan_execution_summary(placement_execution));

  auto reviewed_placement_plan = placement_plan;
  reviewed_placement_plan.policies = {{"placement-move-priorities", "reviewed-priorities.policy",
                                       std::string(64U, '0'), placement_source.source_revision}};
  flux::PlacementMovePriorityPolicy reviewed_priorities;
  reviewed_priorities.source_revision = placement_source.source_revision;
  reviewed_priorities.order = flux::default_placement_move_priority_order();
  for (const auto& footprint : placement_source.footprints) {
    reviewed_priorities.footprints.push_back(
        {footprint.id, flux::PlacementMovePriorityClass::other});
  }
  flux::io::WorkPlanLoadResult loaded_placement;
  loaded_placement.plan = reviewed_placement_plan;
  loaded_placement.policy_board = placement_source;
  loaded_placement.placement_move_priorities = reviewed_priorities;
  loaded_placement.applied_policy_kinds = {"placement-move-priorities"};
  const auto reviewed_placement_execution =
      flux::io::execute_loaded_flux_work_plan(loaded_placement);
  require(reviewed_placement_execution.ok() && reviewed_placement_execution.steps.size() == 1U &&
              reviewed_placement_execution.steps.front().placement_move_priorities_configured,
          "the one-read reviewed move-priority policy must reach the integrated placement "
          "optimizer rather than being discarded after identity validation");

  auto unsafe_placement_plan = placement_plan;
  unsafe_placement_plan.steps.front().options["allow_tentative_placement_envelopes"] = true;
  require(!flux::io::compile_flux_work_plan_routes(placement_source, unsafe_placement_plan).ok(),
          "tentative placement envelopes must remain preview-only in a saved WorkPlan");
  auto ignored_placement_option_plan = placement_plan;
  ignored_placement_option_plan.steps.front().options["refillable_zone_fills"] = true;
  require(!flux::io::compile_flux_work_plan_routes(placement_source, ignored_placement_option_plan)
               .ok(),
          "a placement step must reject route-transaction wrappers that its optimizer would not "
          "execute");
  auto unprojected_placement_plan = placement_plan;
  unprojected_placement_plan.steps.front().selectors.group_scope =
      flux::io::WorkPlanSelectorScope::selected;
  require(
      !flux::io::compile_flux_work_plan_routes(placement_source, unprojected_placement_plan).ok(),
      "placement must project every selected footprint to touching nets before execution");
  auto route_with_placement_option = executable_plan;
  route_with_placement_option.steps.front().options["max_placement_passes"] = std::uint64_t{2U};
  require(!flux::io::compile_flux_work_plan_routes(connected, route_with_placement_option).ok(),
          "placement-only configuration must not be silently accepted by an ordinary route step");

  auto refill_source = two_pad_board();
  refill_source.layers.resize(1U);
  refill_source.nets.push_back(Net{NetId{"work-plan-plane"}, "WORK_PLAN_PLANE", 0});
  flux::CopperZone refill_zone;
  refill_zone.id = ObjectId{"work-plan-saved-fill"};
  refill_zone.net = NetId{"work-plan-plane"};
  refill_zone.fills = {flux::CopperZoneFill{LayerId{"F.Cu"}, rectangle(45.0, 0.0, 55.0, 80.0), {}}};
  refill_source.zones.push_back(std::move(refill_zone));
  auto refill_execution_plan =
      flux::io::make_flux_work_plan_template(refill_source, "refill.kicad_pcb");
  refill_execution_plan.mode = flux::io::WorkPlanMode::new_routing;
  flux::io::WorkPlanStep refill_route;
  refill_route.id = "route-before-refill";
  refill_route.kind = flux::io::WorkPlanStepKind::route;
  refill_route.selectors.exact_nets = {"SIGNAL"};
  refill_route.options = {{"checkpoint_objective", std::string{"aggregate-overflow"}},
                          {"crossing_safe_diagonal_grid_edges", false},
                          {"direct_medial_edges", false},
                          {"grid_pitch_nm", std::uint64_t{5'000'000U}},
                          {"interleave_routability_feedback", false},
                          {"max_router_iterations", std::uint64_t{2U}},
                          {"negotiate_voronoi_lane_conflicts", false},
                          {"physical_envelope_ordering", false},
                          {"physical_overflow_history", false},
                          {"refillable_zone_fills", true},
                          {"routability_feedback_retries", std::uint64_t{0U}},
                          {"strict_medial_topology", false}};
  refill_execution_plan.steps = {refill_route};
  const auto provisional_refill =
      flux::io::execute_flux_work_plan(refill_source, refill_execution_plan);
  require(provisional_refill.ok() && !provisional_refill.whole_board_accepted &&
              provisional_refill.zone_refill_required &&
              provisional_refill.candidate_ready_for_zone_refill &&
              !provisional_refill.candidate_ready_for_export && provisional_refill.candidate &&
              provisional_refill.steps.size() == 1U &&
              provisional_refill.steps.front().status ==
                  flux::io::WorkPlanStepExecutionStatus::completed &&
              provisional_refill.steps.front().provisional_zone_refill_required,
          "a refill-required WorkPlan must retain the exact provisional candidate privately and "
          "authorize only the external refill transaction: " +
              flux::io::serialize_flux_work_plan_execution_summary(provisional_refill));

  Board partial_preview_source;
  partial_preview_source.source_revision = "partial-preview-fixture";
  partial_preview_source.outline = rectangle(0.0, 0.0, 100.0, 80.0);
  partial_preview_source.layers = {{LayerId{"F.Cu"}, "F.Cu", LayerKind::copper, 0U}};
  const NetId preview_easy{"preview-easy"};
  const NetId preview_blocked{"preview-blocked"};
  partial_preview_source.nets = {{preview_easy, "PREVIEW_EASY", 0},
                                 {preview_blocked, "PREVIEW_BLOCKED", 0}};
  const auto preview_terminal = [](std::string id, NetId net, Point position) {
    Footprint footprint;
    footprint.id = ObjectId{id};
    footprint.reference = id;
    footprint.position = position;
    footprint.courtyard = local_rectangle(2.0, 2.0);
    footprint.pads = {{ObjectId{id + "-pad"}, net, Point{}, {LayerId{"F.Cu"}}}};
    return footprint;
  };
  partial_preview_source.footprints = {preview_terminal("E1", preview_easy, mm(10.0, 20.0)),
                                       preview_terminal("E2", preview_easy, mm(30.0, 20.0)),
                                       preview_terminal("B1", preview_blocked, mm(10.0, 60.0)),
                                       preview_terminal("B2", preview_blocked, mm(90.0, 60.0))};
  partial_preview_source.keepouts = {{ObjectId{"preview-wall"},
                                      rectangle(45.0, 0.0, 55.0, 80.0),
                                      {LayerId{"F.Cu"}},
                                      flux::KeepoutRules{true, true, false, false, false},
                                      "preview wall"}};
  auto incomplete_preview_plan =
      flux::io::make_flux_work_plan_template(partial_preview_source, "partial-preview.kicad_pcb");
  incomplete_preview_plan.mode = flux::io::WorkPlanMode::preview;
  flux::io::WorkPlanStep incomplete_preview_route;
  incomplete_preview_route.id = "incomplete-preview";
  incomplete_preview_route.kind = flux::io::WorkPlanStepKind::route;
  incomplete_preview_route.selectors.exact_nets = {"PREVIEW_EASY", "PREVIEW_BLOCKED"};
  incomplete_preview_route.options = {{"checkpoint_objective", std::string{"aggregate-overflow"}},
                                      {"crossing_safe_diagonal_grid_edges", false},
                                      {"direct_medial_edges", false},
                                      {"grid_pitch_nm", std::uint64_t{5'000'000U}},
                                      {"interleave_routability_feedback", false},
                                      {"max_expansions_per_connection", std::uint64_t{100'000U}},
                                      {"max_router_iterations", std::uint64_t{2U}},
                                      {"negotiate_voronoi_lane_conflicts", false},
                                      {"physical_envelope_ordering", false},
                                      {"physical_overflow_history", false},
                                      {"retain_incomplete_diagnostic_preview", true},
                                      {"routability_feedback_retries", std::uint64_t{0U}},
                                      {"strict_medial_topology", false},
                                      {"topology_locked_medial_materialization", false}};
  incomplete_preview_plan.steps = {incomplete_preview_route};
  CollectingTelemetrySink incomplete_preview_telemetry;
  const auto incomplete_preview_execution = flux::io::execute_flux_work_plan(
      partial_preview_source, incomplete_preview_plan, {}, &incomplete_preview_telemetry);
  require(!incomplete_preview_execution.ok() && incomplete_preview_execution.preview &&
              incomplete_preview_execution.candidate_is_incomplete_diagnostic_preview &&
              !incomplete_preview_execution.candidate_ready_for_export &&
              !incomplete_preview_execution.candidate_ready_for_zone_refill &&
              incomplete_preview_execution.candidate &&
              !incomplete_preview_execution.candidate->tracks.empty() &&
              partial_preview_source.tracks.empty() &&
              incomplete_preview_execution.steps.size() == 1U &&
              incomplete_preview_execution.steps.front().status ==
                  flux::io::WorkPlanStepExecutionStatus::failed &&
              incomplete_preview_execution.steps.front().incomplete_diagnostic_preview_attempted &&
              incomplete_preview_execution.steps.front().incomplete_diagnostic_preview_available &&
              incomplete_preview_execution.steps.front().incomplete_diagnostic_preview_paths > 0U &&
              std::any_of(incomplete_preview_telemetry.events.begin(),
                          incomplete_preview_telemetry.events.end(),
                          [](const auto& event) {
                            return event.stable_id ==
                                   "work-plan-diagnostic-preview:incomplete-preview";
                          }),
          "an incomplete preview must retain partial copper and telemetry without changing failure "
          "or publication authority: " +
              flux::io::serialize_flux_work_plan_execution_summary(incomplete_preview_execution));

  std::stop_source cancelled_source;
  cancelled_source.request_stop();
  const auto cancelled =
      flux::io::execute_flux_work_plan(connected, executable_plan, cancelled_source.get_token());
  require(
      !cancelled.ok() && !cancelled.steps.empty() &&
          cancelled.steps.front().status == flux::io::WorkPlanStepExecutionStatus::cancelled &&
          cancelled.candidate && cancelled.candidate->tracks == connected.tracks &&
          cancelled.candidate->footprints == connected.footprints,
      "a pre-requested stop token must cancel before routing and preserve the source checkpoint");
}

} // namespace

int main(int argc, char** argv) {
  const std::vector<TestCase> tests{
      {"routing_engine_explores_supported_physical_families", routing_engine_explores_supported_physical_families},
      {"routing_engine_scaffold_progress_preserves_external_validation_gate",
       routing_engine_scaffold_progress_preserves_external_validation_gate},
      {"routing_engine_cancels_a_live_continuation", routing_engine_cancels_a_live_continuation},
      {"routing_engine_resumes_and_requires_current_external_validation", routing_engine_resumes_and_requires_current_external_validation},
      {"routing_engine_distinguishes_deadline_cancellation_and_budget", routing_engine_distinguishes_deadline_cancellation_and_budget},
      {"kicad_drill_offsets_preserve_copper_and_hole_anchors", kicad_drill_offsets_preserve_copper_and_hole_anchors},
      {"kicad_pad_frames_match_placed_board_coordinates", kicad_pad_frames_match_placed_board_coordinates},
      {"units_remain_exact_and_saturating", units_remain_exact_and_saturating},
      {"multisource_terminal_membership_preserves_access_preference_order",
       multisource_terminal_membership_preserves_access_preference_order},
      {"failed_spline_chains_retry_against_committed_copper",
       failed_spline_chains_retry_against_committed_copper},
      {"dense_candidate_grid_requires_enforced_physical_clearance",
       dense_candidate_grid_requires_enforced_physical_clearance},
      {"default_planar_pitch_uses_scoped_source_rules", default_planar_pitch_uses_scoped_source_rules},
      {"regional_candidate_ranker_is_schema_bound_and_deterministic",
       regional_candidate_ranker_is_schema_bound_and_deterministic},
      {"source_bound_board_rules_control_geometry_and_hole_clearance",
       source_bound_board_rules_control_geometry_and_hole_clearance},
      {"file_import_preserves_project_minimum_rules", file_import_preserves_project_minimum_rules},
      {"dimensioned_pathfinder_pressure_preserves_units_and_group_fairness",
       dimensioned_pathfinder_pressure_preserves_units_and_group_fairness},
      {"quantized_copper_oracle_exposes_physical_conflicts_and_via_chains",
       quantized_copper_oracle_exposes_physical_conflicts_and_via_chains},
      {"physical_probe_preserves_native_coordinate_clearance",
       physical_probe_preserves_native_coordinate_clearance},
      {"regional_fragment_compression_preserves_tree_junctions", regional_fragment_compression_preserves_tree_junctions},
      {"straight_regional_layer_adapters_preserve_connectivity", straight_regional_layer_adapters_preserve_connectivity},
      {"straight_regional_lanes_require_exact_copper_proof", straight_regional_lanes_require_exact_copper_proof},
      {"straight_elbow_alternatives_preserve_exact_copper_contract",
       straight_elbow_alternatives_preserve_exact_copper_contract},
      {"regional_repair_scheduler_preserves_exploration",
       regional_repair_scheduler_preserves_exploration},
      {"regional_access_cache_is_bounded_without_changing_solutions",
       regional_access_cache_is_bounded_without_changing_solutions},
      {"regional_performance_samples_preserve_submicrosecond_work",
       regional_performance_samples_preserve_submicrosecond_work},
      {"fixed_template_copper_pruning_is_conservative",
       fixed_template_copper_pruning_is_conservative},
      {"straight_joint_three_window_repair_escapes_pairwise_minimum",
       straight_joint_three_window_repair_escapes_pairwise_minimum},
      {"straight_scene_portfolio_replays_retained_window_alternative",
       straight_scene_portfolio_replays_retained_window_alternative},
      {"regional_capacity_tracks_longitudinal_demand",
       regional_capacity_tracks_longitudinal_demand},
      {"physical_checkpoint_ordering_is_feasibility_first_and_fail_closed",
       physical_checkpoint_ordering_is_feasibility_first_and_fail_closed},
      {"regional_scope_checkpoint_ordering_retains_checked_physical_evidence",
       regional_scope_checkpoint_ordering_retains_checked_physical_evidence},
      {"routing_xpbd_joint_relaxation_preserves_endpoints_and_reports_stress",
       routing_xpbd_joint_relaxation_preserves_endpoints_and_reports_stress},
      {"straight_track_angles_preserve_off_grid_endpoints_and_clearance",
       straight_track_angles_preserve_off_grid_endpoints_and_clearance},
      {"soft_capacity_cuts_exclude_legally_separated_shadows",
       soft_capacity_cuts_exclude_legally_separated_shadows},
      {"global_to_detailed_route_is_transactional_and_capacity_gated",
       global_to_detailed_route_is_transactional_and_capacity_gated},
      {"constrained_ph_rebound_avoids_inter_sample_obstacle",
       constrained_ph_rebound_avoids_inter_sample_obstacle},
      {"quintic_ph_geometry_preserves_invariants_and_exact_offsets",
       quintic_ph_geometry_preserves_invariants_and_exact_offsets},
      {"arc_free_space_preserves_the_unswept_region", arc_free_space_preserves_the_unswept_region},
      {"curved_ph_materialization_is_exact_and_drc_clean",
       curved_ph_materialization_is_exact_and_drc_clean},
      {"exact_copper_witnesses_create_missing_regional_scope",
       exact_copper_witnesses_create_missing_regional_scope},
      {"regional_spline_tiers_are_selected_breadth_first",
       regional_spline_tiers_are_selected_breadth_first},
      {"exact_via_witnesses_create_regional_repair_scope",
       exact_via_witnesses_create_regional_repair_scope},
      {"regional_bundle_forward_checking_preserves_access_solutions",
       regional_bundle_forward_checking_preserves_access_solutions},
      {"frozen_foreign_copper_rejects_selected_vias_before_spline_selection",
       frozen_foreign_copper_rejects_selected_vias_before_spline_selection},
      {"parallel_spline_workers_shutdown_after_short_optimizations",
       parallel_spline_workers_shutdown_after_short_optimizations},
      {"exact_copper_witnesses_preserve_branch_identity",
       exact_copper_witnesses_preserve_branch_identity},
      {"adaptive_portal_direction_is_independent_of_path_sampling",
       adaptive_portal_direction_is_independent_of_path_sampling},
      {"short_terminal_dogleg_has_a_regular_ph_realization",
       short_terminal_dogleg_has_a_regular_ph_realization},
      {"fallback_corridor_preserves_curvature_room", fallback_corridor_preserves_curvature_room},
      {"fallback_corridor_retains_one_sided_bend_room", fallback_corridor_retains_one_sided_bend_room},
      {"corner_blends_preserve_straight_run_tangents", corner_blends_preserve_straight_run_tangents},
      {"obstacle_aware_diagonals_and_curves_remain_capacity_and_drc_gated",
       obstacle_aware_diagonals_and_curves_remain_capacity_and_drc_gated},
      {"hydraulic_polygon_discovery_projects_into_coarse_navmesh",
       hydraulic_polygon_discovery_projects_into_coarse_navmesh},
      {"anisotropic_hydraulic_erosion_is_directional_and_gpu_equivalent",
       anisotropic_hydraulic_erosion_is_directional_and_gpu_equivalent},
      {"hydraulic_gpu_fuses_clearance_and_batches_independent_requests",
       hydraulic_gpu_fuses_clearance_and_batches_independent_requests},
      {"adaptive_navmesh_recovers_clearance_clipped_passage",
       adaptive_navmesh_recovers_clearance_clipped_passage},
      {"usped_mountains_inform_navmesh_without_changing_free_space",
       usped_mountains_inform_navmesh_without_changing_free_space},
      {"regional_ownership_and_immutable_cache_are_dependency_bound",
       regional_ownership_and_immutable_cache_are_dependency_bound},
      {"advanced_routing_compute_shortlists_preserve_cpu_authority",
       advanced_routing_compute_shortlists_preserve_cpu_authority},
      {"routing_compute_chord_batches_are_bounded_and_conservative",
       routing_compute_chord_batches_are_bounded_and_conservative},
      {"physical_copper_t_junction_is_preserved_during_ph_realization",
       physical_copper_t_junction_is_preserved_during_ph_realization},
      {"same_net_vertical_features_are_reused_as_transition_portals",
       same_net_vertical_features_are_reused_as_transition_portals},
      {"kicad_zone_refill_adapter_is_bounded_and_fail_closed",
       kicad_zone_refill_adapter_is_bounded_and_fail_closed},
      {"generated_copper_telemetry_replaces_latest_scene_and_retains_beziers",
       generated_copper_telemetry_replaces_latest_scene_and_retains_beziers},
      {"work_plan_round_trip_is_strict_source_bound_and_non_authoritative",
       work_plan_round_trip_is_strict_source_bound_and_non_authoritative},
      {"integrated_placement_rejects_proxy_improvement_that_violates_copper_rules",
       integrated_placement_rejects_proxy_improvement_that_violates_copper_rules},
      {"cancellation_and_kicad_export_boundaries_fail_closed",
       cancellation_and_kicad_export_boundaries_fail_closed},
  };

  std::size_t passed = 0U;
  std::size_t selected = 0U;
  for (const auto& test : tests) {
    if (argc > 1 && test.name != argv[1]) {
      continue;
    }
    ++selected;
    try {
      test.run();
      ++passed;
      std::cout << "PASS " << test.name << '\n';
    } catch (const std::exception& error) {
      std::cerr << "FAIL " << test.name << ": " << error.what() << '\n';
    }
  }
  std::cout << passed << '/' << selected << " tests passed\n";
  return selected > 0U && passed == selected ? 0 : 1;
}
