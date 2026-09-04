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

GlobalRouteResult route_global_with_sampled_conflict_repair(
    const Board& board, const GlobalRouterConfig& config, std::size_t max_repair_cones,
    std::stop_token stop_token, RouterTelemetrySink* telemetry) {
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
  const auto tile_span =
      pitch > std::numeric_limits<std::int64_t>::max() / 4 ? pitch : pitch * 4;
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
      repair_cones.push_back(
          RepairCone{std::vector<NetId>(members.nets.begin(), members.nets.end()),
                     members.pressure, key});
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
    if (global_route_checkpoint_better(repaired, result, config.checkpoint_objective)) {
      result = std::move(repaired);
      ++accepted;
    }
  }
  result.post_negotiation_cone_repair_attempts = attempts;
  result.post_negotiation_cone_repairs_accepted = accepted;
  result.post_negotiation_cone_repair_nets_attempted = nets_attempted;
  return result;
}

RoutingPlanResult plan_routes_with_sampled_conflict_repair(
    const Board& board, const RoutingPlannerConfig& config, std::size_t max_repair_cones,
    std::stop_token stop_token, RouterTelemetrySink* telemetry) {
  RoutingPlanResult result;
  result.coarse_global = route_global_with_sampled_conflict_repair(
      board, config.global, max_repair_cones, stop_token, telemetry);
  if (stop_token.stop_requested() || result.coarse_global.stop_reason ==
                                         GlobalRouterStopReason::cancelled) {
    result.stop_reason = RoutingPlanStopReason::cancelled;
    return result;
  }
  result.regional =
      analyze_regional_conflicts(board, result.coarse_global, config.regional, stop_token,
                                 telemetry);
  if (stop_token.stop_requested() ||
      result.regional.stop_reason == RegionalRouterStopReason::cancelled) {
    result.stop_reason = RoutingPlanStopReason::cancelled;
    return result;
  }
  result.paths = result.regional.spline_candidate_available
                     ? result.regional.spline_candidate_paths
                     : result.coarse_global.paths;
  result.stop_reason =
      !result.coarse_global.complete() ? RoutingPlanStopReason::coarse_global_incomplete
      : result.regional.complete()     ? RoutingPlanStopReason::complete
                                       : RoutingPlanStopReason::regional_conflicts;
  return result;
}

MaterializationFeedbackResult refine_materialization_with_routability_feedback(
    const Board& planning_board, const Board& materialization_source,
    RoutingPlanResult initial_routing, DetailedRouteResult initial_detailed,
    const RoutingPlannerConfig& planner_config, const DetailedRouterConfig& detailed_config,
    std::size_t max_feedback_passes, std::stop_token stop_token,
    RouterTelemetrySink* telemetry) {
  MaterializationFeedbackResult result{std::move(initial_routing), std::move(initial_detailed)};
  if (max_feedback_passes == 0U || result.detailed.complete() ||
      result.detailed.provisional_zone_refill_candidate() ||
      result.detailed.routability_diagnoses.empty() || stop_token.stop_requested()) {
    return result;
  }

  const auto exact_quality = [](const DetailedRouteResult& candidate) {
    return std::tuple{candidate.complete() ? 0U : 1U,
                      candidate.candidate ? 0U : 1U,
                      candidate.validation.error_count(),
                      candidate.connectivity.open_connections,
                      candidate.copper_drc.violation_count(),
                      candidate.routability_diagnoses.size()};
  };
  auto feedback_knowledge = result.routing.routability_knowledge;
  auto diagnoses = result.detailed.routability_diagnoses;
  std::set<NetId> affected_feedback_nets;
  std::size_t diagnoses_consumed = 0U;
  std::size_t feedback_passes = 0U;
  std::size_t improvements = 0U;
  const auto final_feedback_epoch_iterations = std::max<std::size_t>(
      1U, (planner_config.global.max_iterations + max_feedback_passes - 1U) /
              max_feedback_passes);

  for (; feedback_passes < max_feedback_passes && !diagnoses.empty() &&
         !stop_token.stop_requested();
       ++feedback_passes) {
    diagnoses_consumed += diagnoses.size();
    for (const auto& diagnosis : diagnoses) {
      affected_feedback_nets.insert(diagnosis.resource.nets.begin(),
                                    diagnosis.resource.nets.end());
      if (telemetry != nullptr) {
        RouterTelemetryEvent event;
        event.kind = RouterTelemetryEventKind::search_summary;
        event.stable_id = diagnosis.resource.stable_id;
        event.detail =
            "final routability diagnosis pass " + std::to_string(feedback_passes + 1U) + ": " +
            std::string(to_string(diagnosis.kind)) +
            ", relaxation=" + std::string(to_string(diagnosis.suggested_relaxation));
        static_cast<void>(telemetry->try_emit(std::move(event)));
      }
    }
    accumulate_clean_copper_knowledge(feedback_knowledge, result.detailed);
    accumulate_routability_knowledge(feedback_knowledge, diagnoses);

    auto feedback_global_config = planner_config.global;
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

    RoutingPlanResult feedback_plan;
    const std::vector<NetId> affected_nets(affected_feedback_nets.begin(),
                                           affected_feedback_nets.end());
    feedback_plan.coarse_global = route_global_incremental(
        planning_board, result.routing.coarse_global, affected_nets, feedback_global_config,
        stop_token, telemetry, true);
    feedback_plan.regional = analyze_regional_conflicts(
        planning_board, feedback_plan.coarse_global, planner_config.regional, stop_token,
        telemetry);
    feedback_plan.paths = feedback_plan.regional.spline_candidate_available
                              ? feedback_plan.regional.spline_candidate_paths
                              : feedback_plan.coarse_global.paths;
    const auto connected_topology = feedback_plan.coarse_global.unrouted_nets.empty() &&
                                    !feedback_plan.coarse_global.paths.empty();
    const auto physically_realizable =
        feedback_plan.regional.complete() &&
        (feedback_plan.coarse_global.complete() ||
         feedback_plan.regional.spline_candidate_available);
    feedback_plan.stop_reason =
        connected_topology && physically_realizable ? RoutingPlanStopReason::complete
        : !connected_topology ? RoutingPlanStopReason::coarse_global_incomplete
                              : RoutingPlanStopReason::regional_conflicts;
    auto feedback_diagnoses = probe_global_routability(feedback_plan.coarse_global);
    const auto regional_diagnoses = diagnose_regional_routability(feedback_plan.regional);
    feedback_diagnoses.insert(feedback_diagnoses.end(), regional_diagnoses.begin(),
                              regional_diagnoses.end());
    accumulate_routability_knowledge(feedback_knowledge, feedback_diagnoses);
    feedback_plan.routability_knowledge = feedback_knowledge;
    if (!feedback_plan.complete()) {
      diagnoses = std::move(feedback_diagnoses);
      continue;
    }

    auto feedback_detailed = materialize_detailed_route(
        materialization_source, feedback_plan, detailed_config, stop_token);
    feedback_knowledge = feedback_plan.routability_knowledge;
    accumulate_clean_copper_knowledge(feedback_knowledge, feedback_detailed);
    diagnoses = feedback_detailed.routability_diagnoses;
    if (exact_quality(feedback_detailed) < exact_quality(result.detailed)) {
      ++improvements;
      result.routing = std::move(feedback_plan);
      result.detailed = std::move(feedback_detailed);
      if (result.detailed.complete() || result.detailed.provisional_zone_refill_candidate()) {
        ++feedback_passes;
        break;
      }
    }
  }
  result.detailed.final_routability_feedback_attempted = feedback_passes > 0U;
  result.detailed.final_routability_feedback_selected = improvements > 0U;
  result.detailed.final_routability_feedback_passes = feedback_passes;
  result.detailed.final_routability_feedback_improvements = improvements;
  result.detailed.final_routability_diagnoses_consumed = diagnoses_consumed;
  return result;
}

} // namespace flux
