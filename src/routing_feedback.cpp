#include "flux/core/routing_feedback.hpp"

#include "flux/core/routability_knowledge.hpp"

#include <algorithm>
#include <array>
#include <limits>
#include <map>
#include <numeric>
#include <set>
#include <tuple>
#include <utility>
#include <vector>

namespace flux {
namespace {

void publish_materialization_summary(const DetailedRouteResult& result, std::size_t pass,
                                     RouterTelemetrySink* telemetry) {
  if (telemetry == nullptr)
    return;
  RouterTelemetryEvent event;
  event.kind = RouterTelemetryEventKind::search_summary;
  event.stable_id = "persistent-materialization:" + std::to_string(pass);
  event.detail = "materialization pass " + std::to_string(pass) +
      ": complete=" + (result.complete() ? "yes" : "no") +
      ", candidate=" + (result.candidate ? "yes" : "no") +
      ", opens=" + std::to_string(result.connectivity.open_connections) +
      ", copper_violations=" + std::to_string(result.copper_drc.violation_count()) +
      ", tracks=" + std::to_string(result.added_tracks) +
      ", arcs=" + std::to_string(result.added_track_arcs) +
      ", beziers=" + std::to_string(result.added_track_beziers) +
      ", vias=" + std::to_string(result.added_vias) +
      ", unrounded_bends=" + std::to_string(result.unrounded_non_junction_bends);
  std::map<std::string, std::size_t> categories;
  for (const auto& violation : result.copper_drc.violations)
    ++categories[violation.code];
  for (const auto& [category, count] : categories)
    event.detail += ", " + category + "=" + std::to_string(count);
  event.detail += "; " + result.diagnostic;
  static_cast<void>(telemetry->try_emit(std::move(event)));
}

} // namespace

GlobalRouteResult route_global_with_sampled_conflict_repair(const Board& board,
                                                            const GlobalRouterConfig& config,
                                                            std::size_t max_repair_cones,
                                                            std::stop_token stop_token,
                                                            RouterTelemetrySink* telemetry) {
  auto result = route_global(board, config, stop_token, telemetry);
  if (max_repair_cones == 0U || stop_token.stop_requested() || result.complete() ||
      !result.unrouted_nets.empty() ||
      (result.sampled_overflow == 0U && result.total_voronoi_overflow_nm == 0U &&
       result.clearance_shadow_overflow == 0U) ||
      result.corridor_overflow != 0U) {
    return result;
  }

  struct RepairCone {
    std::vector<NetId> nets;
    std::size_t pressure{0U};
    std::tuple<std::string, std::int64_t, std::int64_t> tile;
  };
  struct TileMembers {
    std::set<NetId> nets;
    std::size_t pressure{0U};
  };
  const auto pitch = config.grid_pitch.value();
  if (pitch <= 0) {
    return result;
  }
  const auto tile_span = pitch > std::numeric_limits<std::int64_t>::max() / 4 ? pitch : pitch * 4;
  const auto tile_coordinate = [tile_span](std::int64_t coordinate) {
    auto quotient = coordinate / tile_span;
    if (coordinate % tile_span < 0) {
      --quotient;
    }
    return quotient;
  };
  std::map<std::tuple<std::string, std::int64_t, std::int64_t>, TileMembers> tiles;
  const auto add_tile = [&](Point first, Point second, const LayerId& layer, const auto& nets,
                            std::size_t pressure) {
    const auto midpoint_x = std::midpoint(first.x.value(), second.x.value());
    const auto midpoint_y = std::midpoint(first.y.value(), second.y.value());
    auto& tile = tiles[{layer.value(), tile_coordinate(midpoint_x), tile_coordinate(midpoint_y)}];
    tile.nets.insert(nets.begin(), nets.end());
    tile.pressure = pressure > std::numeric_limits<std::size_t>::max() - tile.pressure
                        ? std::numeric_limits<std::size_t>::max()
                        : tile.pressure + pressure;
  };
  for (const auto& conflict : result.clearance_shadow_conflicts) {
    const std::array nets{conflict.first_net, conflict.second_net};
    add_tile(conflict.first.position, conflict.second.position, conflict.first.layer, nets, 1U);
  }
  for (const auto& resource : result.resource_usage) {
    if (resource.congested()) {
      add_tile(resource.first.position, resource.second.position, resource.first.layer,
               resource.nets, resource.demand - resource.capacity);
    }
  }
  for (const auto& resource : result.voronoi_resource_usage) {
    if (resource.congested()) {
      add_tile(resource.start, resource.end, resource.layer, resource.nets, 1U);
    }
  }
  std::vector<RepairCone> repair_cones;
  repair_cones.reserve(tiles.size());
  for (auto& [key, members] : tiles) {
    if (members.nets.size() >= 2U) {
      repair_cones.push_back(RepairCone{
          std::vector<NetId>(members.nets.begin(), members.nets.end()), members.pressure, key});
    }
  }
  std::stable_sort(repair_cones.begin(), repair_cones.end(), [](const auto& lhs, const auto& rhs) {
    return lhs.pressure != rhs.pressure
               ? lhs.pressure > rhs.pressure
               : std::tie(lhs.nets, lhs.tile) < std::tie(rhs.nets, rhs.tile);
  });

  auto repair_config = config;
  repair_config.atomic_complete_net_routing = true;
  repair_config.diversify_pin_access_rings = true;
  repair_config.hard_capacity_start_iteration = 1U;
  repair_config.max_iterations = std::min(config.max_iterations, std::size_t{12U});
  repair_config.selective_congestion_reroute = false;
  repair_config.selective_full_reroute_interval = 0U;
  repair_config.max_hard_capacity_blocker_reuse_invalidations_per_failed_net = 0U;
  std::set<std::vector<NetId>> attempted;
  std::size_t attempts = 0U;
  std::size_t accepted = 0U;
  std::size_t nets_attempted = 0U;
  for (const auto& cone : repair_cones) {
    if (attempts >= max_repair_cones || stop_token.stop_requested()) {
      break;
    }
    if (cone.nets.empty() || !attempted.insert(cone.nets).second) {
      continue;
    }
    ++attempts;
    nets_attempted += cone.nets.size();
    auto repair_checkpoint = result;
    repair_checkpoint.configured_diversify_pin_access_rings = true;
    repair_checkpoint.configured_hard_capacity_start_iteration = 1U;
    auto repaired = route_global_incremental(board, repair_checkpoint, cone.nets, repair_config,
                                             stop_token, telemetry);
    // Invalid/cancelled attempts often have empty paths and zero overflow. They are not
    // better physical checkpoints and must never erase the valid incumbent.
    if (repaired.stop_reason != GlobalRouterStopReason::invalid_input &&
        repaired.stop_reason != GlobalRouterStopReason::cancelled &&
        global_route_checkpoint_better(repaired, result, config.checkpoint_objective)) {
      result = std::move(repaired);
      ++accepted;
    }
  }
  result.post_negotiation_cone_repair_attempts = attempts;
  result.post_negotiation_cone_repairs_accepted = accepted;
  result.post_negotiation_cone_repair_nets_attempted = nets_attempted;
  return result;
}

RoutingPlanResult plan_routes_with_sampled_conflict_repair(const Board& board,
                                                           const RoutingPlannerConfig& config,
                                                           std::size_t max_repair_cones,
                                                           std::stop_token stop_token,
                                                           RouterTelemetrySink* telemetry) {
  RoutingPlanResult result;
  auto global_config = config.global;
  auto regional_config = config.regional;
  if (!config.curved_tracks) {
    global_config.use_navmesh_funnel_spline_seeds = false;
    global_config.single_lane_direct_voronoi_resources = true;
    regional_config.enable_spline_realization = false;
  }
  result.coarse_global = route_global_with_sampled_conflict_repair(
      board, global_config, max_repair_cones, stop_token, telemetry);
  if (stop_token.stop_requested() ||
      result.coarse_global.stop_reason == GlobalRouterStopReason::cancelled) {
    result.stop_reason = RoutingPlanStopReason::cancelled;
    return result;
  }
  result.regional = analyze_regional_conflicts(board, result.coarse_global, regional_config,
                                               stop_token, telemetry);
  if (stop_token.stop_requested() ||
      result.regional.stop_reason == RegionalRouterStopReason::cancelled) {
    result.stop_reason = RoutingPlanStopReason::cancelled;
    return result;
  }
  result.paths = result.regional.spline_candidate_available ? result.regional.spline_candidate_paths
                                                            : result.coarse_global.paths;
  result.stop_reason = !result.coarse_global.complete()
                           ? RoutingPlanStopReason::coarse_global_incomplete
                       : result.regional.complete() ? RoutingPlanStopReason::complete
                                                    : RoutingPlanStopReason::regional_conflicts;
  return result;
}

RoutingPlanResult plan_routes(const Board& board, const RoutingPlannerConfig& config,
                              std::stop_token stop_token, RouterTelemetrySink* telemetry) {
  RoutingEngineConfig engine_config;
  engine_config.planner = config;
  engine_config.materialize = false;
  RoutingEngine engine(board, board, engine_config, telemetry);
  while (!engine.terminal()) {
    const auto step = engine.advance({}, stop_token);
    if (step.outcome == RoutingEngineOutcome::awaiting_materialization ||
        step.outcome == RoutingEngineOutcome::awaiting_external_validation) break;
  }
  if (const auto* selected = engine.selected_plan()) return *selected;
  RoutingPlanResult result;
  result.stop_reason = RoutingPlanStopReason::cancelled;
  result.diagnostic = std::string("routing engine stopped before a planning checkpoint: ") +
                      to_string(engine.state().outcome);
  return result;
}

MaterializationFeedbackResult
run_persistent_routing_session(const Board& planning_board, const Board& materialization_source,
                               const PersistentRoutingSessionConfig& config,
                               std::stop_token stop_token, RouterTelemetrySink* telemetry) {
  auto engine_config = config.engine;
  engine_config.planner = config.planner;
  engine_config.detailed = config.detailed;
  engine_config.materialize = true;
  engine_config.post_negotiation_sampled_conflict_repair = config.post_negotiation_sampled_conflict_repair;
  engine_config.max_post_negotiation_sampled_conflict_repair_cones = config.max_post_negotiation_sampled_conflict_repair_cones;
  // The persistent engine owns its deadline and explicit action bound. The legacy final-pass
  // count must not silently truncate exploration of its independent repair strategies.
  RoutingEngine engine(planning_board, materialization_source, engine_config, telemetry);
  while (!engine.terminal()) {
    const auto step = engine.advance(config.step_budget, stop_token);
    if (step.outcome == RoutingEngineOutcome::awaiting_external_validation ||
        step.outcome == RoutingEngineOutcome::awaiting_materialization) break;
  }
  MaterializationFeedbackResult result;
  if (const auto* selected = engine.selected_plan()) result.routing = *selected;
  if (const auto* selected = engine.selected_detailed()) result.detailed = *selected;
  result.engine_outcome = engine.state().outcome;
  result.engine_attempts = engine.state().attempts;
  if (engine.state().legal_scaffold)
    result.retained_scaffold_counters = engine.state().legal_scaffold->counters;
  result.engine_tightening_actions = engine.state().tightening_actions;
  result.engine_tightening_improvements = engine.state().tightening_improvements;
  result.engine_decision_points = engine.state().decision_points;
  result.engine_regional_action_switches = engine.state().regional_action_switches;
  result.engine_regional_continuations_resumed = engine.state().regional_continuations_resumed;
  result.engine_regional_work_budget = engine.state().regional_work_budget;
  result.engine_preparation_work_budget = engine.state().preparation_work_budget;
  result.engine_global_retry_budget = engine.state().global_retry_budget;
  result.engine_changed_context_retry_budget = engine.state().changed_context_retry_budget;
  result.engine_physical_context_changes = engine.state().physical_context_changes;
  result.engine_preparation_budget_exhaustions = engine.state().preparation_budget_exhaustions;
  result.engine_global_retry_credit_events = engine.state().global_retry_credit_events;
  result.engine_regional_work_credit_events = engine.state().regional_work_credit_events;
  result.engine_regional_work_budget_exhaustions = engine.state().regional_work_budget_exhaustions;
  result.engine_experiment_initial_explorations = engine.state().experiment_initial_explorations;
  result.engine_experiment_state_suppressions = engine.state().experiment_state_suppressions;
  result.engine_experiment_started = engine.state().experiment_started;
  result.engine_experiment_suspended = engine.state().experiment_suspended;
  result.engine_experiment_resumed = engine.state().experiment_resumed;
  result.engine_experiment_invalidated = engine.state().experiment_invalidated;
  result.engine_experiment_retired = engine.state().experiment_retired;
  result.engine_experiment_memory_evictions = engine.state().experiment_memory_evictions;
  result.engine_experiment_active_us = engine.state().experiment_active_us;
  result.engine_preparation_reuses = engine.state().preparation_reuses;
  result.engine_speculative_preparations_started = engine.state().speculative_preparations_started;
  result.engine_speculative_preparations_used = engine.state().speculative_preparations_used;
  result.engine_speculative_preparations_discarded = engine.state().speculative_preparations_discarded;

  if (!result.detailed.complete())
    result.detailed.diagnostic += std::string("; routing engine: ") + to_string(result.engine_outcome);
  return result;
}

MaterializationFeedbackResult::Scope
materialization_feedback_scope(const RoutabilityDiagnosis& diagnosis) {
  switch (diagnosis.suggested_relaxation) {
  case RoutabilityRelaxation::increase_local_clearance_margin:
    return MaterializationFeedbackResult::Scope::local_geometry;
  case RoutabilityRelaxation::reduce_resource_demand:
  case RoutabilityRelaxation::change_terminal_order:
  case RoutabilityRelaxation::change_via_site:
  case RoutabilityRelaxation::change_lane_order:
  case RoutabilityRelaxation::change_layer_sequence:
  case RoutabilityRelaxation::change_corridor:
    return MaterializationFeedbackResult::Scope::regional;
  case RoutabilityRelaxation::replan_global_topology:
    return MaterializationFeedbackResult::Scope::global;
  }
  return MaterializationFeedbackResult::Scope::global;
}

MaterializationFeedbackResult refine_materialization_with_routability_feedback(
    const Board& planning_board, const Board& materialization_source,
    RoutingPlanResult initial_routing, DetailedRouteResult initial_detailed,
    const RoutingPlannerConfig& planner_config, const DetailedRouterConfig& detailed_config,
    std::size_t max_feedback_passes, std::stop_token stop_token, RouterTelemetrySink* telemetry) {
  MaterializationFeedbackResult result{
      std::move(initial_routing), std::move(initial_detailed), false, false, {}, RoutingEngineOutcome::progressed, {}, std::nullopt};
  if (max_feedback_passes == 0U || result.detailed.complete() ||
      result.detailed.provisional_zone_refill_candidate() ||
      result.detailed.routability_diagnoses.empty() || stop_token.stop_requested()) {
    return result;
  }

  const auto exact_quality = [](const DetailedRouteResult& candidate) {
    return std::tuple{
        candidate.complete() ? 0U : 1U,         candidate.candidate ? 0U : 1U,
        candidate.validation.error_count(),     candidate.connectivity.open_connections,
        candidate.copper_drc.violation_count(), candidate.routability_diagnoses.size()};
  };
  const auto classify_plan = [](RoutingPlanResult& plan) {
    const auto connected = plan.coarse_global.unrouted_nets.empty() &&
                           !plan.coarse_global.paths.empty() &&
                           plan.coarse_global.unaccepted_selected_voronoi_lane_transitions == 0U;
    const auto regional_complete =
        plan.regional.complete() &&
        (plan.coarse_global.complete() || plan.regional.spline_candidate_available);
    plan.stop_reason = connected && regional_complete ? RoutingPlanStopReason::complete
                       : !connected ? RoutingPlanStopReason::coarse_global_incomplete
                                    : RoutingPlanStopReason::regional_conflicts;
    plan.diagnostic = plan.stop_reason == RoutingPlanStopReason::complete
                          ? "persistent routing session produced a realizable regional plan"
                      : plan.stop_reason == RoutingPlanStopReason::coarse_global_incomplete
                          ? "persistent routing session requires a different global topology"
                          : plan.regional.diagnostic;
  };
  const auto diagnosis_key = [](const RoutabilityDiagnosis& diagnosis) {
    std::string key = std::to_string(static_cast<unsigned>(diagnosis.kind)) + ':' +
                      std::to_string(static_cast<unsigned>(diagnosis.suggested_relaxation)) + ':' +
                      diagnosis.resource.stable_id;
    for (const auto& net : diagnosis.resource.nets) {
      key += ':' + net.value();
    }
    return key;
  };
  const auto scope_name = [](MaterializationFeedbackResult::Scope scope) {
    switch (scope) {
    case MaterializationFeedbackResult::Scope::local_geometry:
      return "local-geometry";
    case MaterializationFeedbackResult::Scope::regional:
      return "regional";
    case MaterializationFeedbackResult::Scope::global:
      return "global";
    }
    return "global";
  };

  auto feedback_knowledge = result.routing.routability_knowledge;
  auto working_routing = result.routing;
  auto diagnoses = result.detailed.routability_diagnoses;
  std::map<std::string, std::size_t> repeated_failures;
  std::size_t diagnoses_consumed = 0U;
  std::size_t improvements = 0U;
  const auto final_feedback_epoch_iterations = std::max<std::size_t>(
      1U, (planner_config.global.max_iterations + max_feedback_passes - 1U) / max_feedback_passes);

  for (std::size_t pass_index = 0U;
       pass_index < max_feedback_passes && !diagnoses.empty() && !stop_token.stop_requested();
       ++pass_index) {
    MaterializationFeedbackResult::Pass pass;
    pass.index = pass_index;
    pass.diagnoses = diagnoses.size();
    diagnoses_consumed += diagnoses.size();

    std::set<NetId> affected_feedback_nets;
    auto requested_scope = MaterializationFeedbackResult::Scope::local_geometry;
    bool repeated = false;
    for (const auto& diagnosis : diagnoses) {
      affected_feedback_nets.insert(diagnosis.resource.nets.begin(), diagnosis.resource.nets.end());
      requested_scope = std::max(requested_scope, materialization_feedback_scope(diagnosis));
      repeated = ++repeated_failures[diagnosis_key(diagnosis)] > 1U || repeated;
      if (telemetry != nullptr) {
        RouterTelemetryEvent event;
        event.kind = RouterTelemetryEventKind::search_summary;
        event.stable_id = diagnosis.resource.stable_id;
        event.layer = diagnosis.resource.layer;
        if (!diagnosis.resource.nets.empty())
          event.net = diagnosis.resource.nets.front();
        if (diagnosis.resource.nets.size() > 1U)
          event.other_net = diagnosis.resource.nets[1U];
        event.x_nm = std::midpoint(diagnosis.resource.bounds.minimum.x.value(),
                                   diagnosis.resource.bounds.maximum.x.value());
        event.y_nm = std::midpoint(diagnosis.resource.bounds.minimum.y.value(),
                                   diagnosis.resource.bounds.maximum.y.value());
        event.clearance_deficit_nm = diagnosis.clearance_deficit.value();
        event.detail = "persistent routability diagnosis pass " + std::to_string(pass_index + 1U) +
                       ": " + std::string(to_string(diagnosis.kind)) +
                       ", relaxation=" + std::string(to_string(diagnosis.suggested_relaxation)) +
                       "; " + diagnosis.detail;
        static_cast<void>(telemetry->try_emit(std::move(event)));
      }
    }
    // A first local/PH/quantization failure stays downstream.  If the same exact failure survives
    // a regional alternative, escalate one level so the session cannot cycle forever around an
    // unchanged lane/via/corridor state.
    if (repeated && requested_scope != MaterializationFeedbackResult::Scope::global) {
      requested_scope = requested_scope == MaterializationFeedbackResult::Scope::local_geometry
                            ? MaterializationFeedbackResult::Scope::regional
                            : MaterializationFeedbackResult::Scope::global;
      pass.repeated_failure_escalation = true;
    }
    pass.scope = requested_scope;
    pass.affected_nets = affected_feedback_nets.size();

    accumulate_clean_copper_knowledge(feedback_knowledge, result.detailed);
    accumulate_routability_knowledge(feedback_knowledge, diagnoses);

    RoutingPlanResult feedback_plan;
    if (requested_scope == MaterializationFeedbackResult::Scope::global) {
      auto feedback_global_config = planner_config.global;
      if (!planner_config.curved_tracks) {
        feedback_global_config.use_navmesh_funnel_spline_seeds = false;
        feedback_global_config.single_lane_direct_voronoi_resources = true;
      }
      feedback_global_config.max_iterations = final_feedback_epoch_iterations;
      if (feedback_global_config.hard_capacity_start_iteration > 0U) {
        feedback_global_config.hard_capacity_start_iteration = 1U;
      }
      if (feedback_global_config.bidirectional_route_growth_start_iteration > 0U) {
        feedback_global_config.bidirectional_route_growth_start_iteration = 1U;
      }
      if (feedback_global_config.present_congestion_escalation_start_iteration > 0U) {
        feedback_global_config.present_congestion_escalation_start_iteration = 1U;
      }
      feedback_global_config.routability_history = feedback_knowledge.history;
      feedback_global_config.hydraulic_success_history = feedback_knowledge.success_history;
      const std::vector<NetId> affected_nets(affected_feedback_nets.begin(),
                                             affected_feedback_nets.end());
      pass.incremental_global =
          !affected_nets.empty() && !working_routing.coarse_global.paths.empty();
      feedback_plan.coarse_global =
          pass.incremental_global
              ? route_global_incremental(planning_board, working_routing.coarse_global,
                                         affected_nets, feedback_global_config, stop_token,
                                         telemetry, true)
              : route_global(planning_board, feedback_global_config, stop_token, telemetry);
    } else {
      // Preserve global topology and give the regional exact master/PH/PNS loop the failure it is
      // designed to act on.  This is the common path for lane, terminal, via, layer, curvature,
      // shove, and quantization diagnoses.
      feedback_plan.coarse_global = working_routing.coarse_global;
    }

    auto regional_config = planner_config.regional;
    regional_config.enable_spline_realization = planner_config.curved_tracks;
    regional_config.xpbd_history = feedback_knowledge.history;
    regional_config.physical_conflict_witnesses.clear();
    if (requested_scope != MaterializationFeedbackResult::Scope::global)
      regional_config.physical_conflict_witnesses = diagnoses;
    else {
      feedback_plan.quantized_copper_oracle = probe_quantized_copper_feasibility(
          planning_board, feedback_plan.coarse_global, planner_config.quantized_copper_oracle, stop_token);
      regional_config.physical_conflict_witnesses = feedback_plan.quantized_copper_oracle.diagnoses;
    }
    feedback_plan.regional = analyze_regional_conflicts(planning_board, feedback_plan.coarse_global,
                                                        regional_config, stop_token, telemetry);
    feedback_plan.paths = feedback_plan.regional.spline_candidate_available
                              ? feedback_plan.regional.spline_candidate_paths
                              : feedback_plan.coarse_global.paths;
    classify_plan(feedback_plan);
    feedback_plan.routability_knowledge = feedback_knowledge;
    pass.plan_complete = feedback_plan.complete();

    if (telemetry != nullptr) {
      RouterTelemetryEvent event;
      event.kind = RouterTelemetryEventKind::search_summary;
      event.stable_id = "persistent-feedback:" + std::to_string(pass_index + 1U);
      event.iteration = pass_index + 1U;
      event.total_nets = pass.affected_nets;
      event.detail = "feedback dispatched to " + std::string(scope_name(requested_scope)) +
                     " loop; plan_complete=" + (pass.plan_complete ? "true" : "false");
      static_cast<void>(telemetry->try_emit(std::move(event)));
    }

    if (!feedback_plan.complete()) {
      auto feedback_diagnoses = probe_global_routability(feedback_plan.coarse_global);
      const auto regional_diagnoses = diagnose_regional_routability(feedback_plan.regional);
      feedback_diagnoses.insert(feedback_diagnoses.end(), regional_diagnoses.begin(),
                                regional_diagnoses.end());
      accumulate_routability_knowledge(feedback_knowledge, feedback_diagnoses);
      feedback_plan.routability_knowledge = feedback_knowledge;
      working_routing = std::move(feedback_plan);
      diagnoses = std::move(feedback_diagnoses);
      result.passes.push_back(pass);
      continue;
    }

    pass.incremental_detailed = result.detailed.candidate.has_value();
    auto feedback_detailed =
        pass.incremental_detailed
            ? materialize_detailed_route_incremental(materialization_source, feedback_plan,
                                                     result.detailed, detailed_config, stop_token)
            : materialize_detailed_route(materialization_source, feedback_plan, detailed_config,
                                         stop_token);
    publish_materialization_summary(feedback_detailed, pass_index + 1U, telemetry);
    pass.exact_complete =
        feedback_detailed.complete() || feedback_detailed.provisional_zone_refill_candidate();
    accumulate_clean_copper_knowledge(feedback_knowledge, feedback_detailed);
    diagnoses = feedback_detailed.routability_diagnoses;
    // Retain the newly explored routing state even when its exact score is not yet the best.  The
    // next diagnosis therefore repairs the state which produced it instead of replaying the same
    // old checkpoint.  Only the best exact transaction remains externally selectable.
    feedback_plan.routability_knowledge = feedback_knowledge;
    working_routing = feedback_plan;
    if (exact_quality(feedback_detailed) < exact_quality(result.detailed)) {
      ++improvements;
      pass.selected = true;
      result.routing = std::move(feedback_plan);
      result.detailed = std::move(feedback_detailed);
    }
    result.passes.push_back(pass);
    if (pass.exact_complete) {
      if (!pass.selected) {
        result.routing = std::move(working_routing);
        result.detailed = std::move(feedback_detailed);
        ++improvements;
        result.passes.back().selected = true;
      }
      break;
    }
  }
  result.routing.routability_knowledge = feedback_knowledge;
  result.detailed.final_routability_feedback_attempted = !result.passes.empty();
  result.detailed.final_routability_feedback_selected = improvements > 0U;
  result.detailed.final_routability_feedback_passes = result.passes.size();
  result.detailed.final_routability_feedback_improvements = improvements;
  result.detailed.final_routability_diagnoses_consumed = diagnoses_consumed;
  for (const auto& pass : result.passes) {
    switch (pass.scope) {
    case MaterializationFeedbackResult::Scope::local_geometry:
      ++result.detailed.final_routability_local_geometry_passes;
      break;
    case MaterializationFeedbackResult::Scope::regional:
      ++result.detailed.final_routability_regional_passes;
      break;
    case MaterializationFeedbackResult::Scope::global:
      ++result.detailed.final_routability_global_passes;
      break;
    }
    result.detailed.final_routability_repeated_failure_escalations +=
        pass.repeated_failure_escalation ? 1U : 0U;
    result.detailed.final_routability_incremental_detailed_passes +=
        pass.incremental_detailed ? 1U : 0U;
  }
  return result;
}

} // namespace flux
