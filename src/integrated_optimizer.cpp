#include "flux/core/bezier_copper.hpp"
#include "flux/core/integrated_optimizer.hpp"
#include "flux/core/regional_router.hpp"

#include "flux/core/geometry.hpp"
#include "flux/core/copper_geometry.hpp"
#include "flux/core/routing_compute.hpp"
#include "flux/core/routing_feedback.hpp"
#include "flux/core/routing_rules.hpp"
#include "flux/core/validation.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <compare>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <limits>
#include <map>
#include <numeric>
#include <set>
#include <stdexcept>
#include <stop_token>
#include <thread>
#include <tuple>
#include <utility>
#include <vector>

namespace flux {
namespace {

struct CandidateEvaluation {
  Board placement;
  IntegratedScore score;
  GlobalRouteResult global;
  std::optional<RoutingPlanResult> routing_plan;
  DetailedRouteResult detailed;
  bool reused_static_zone_validation{false};
};

struct Bounds {
  long double min_x{0.0L};
  long double max_x{0.0L};
  long double min_y{0.0L};
  long double max_y{0.0L};
};

[[nodiscard]] bool diagnosis_touches_footprint(const Footprint& footprint,
                                               const RoutabilityDiagnosis& diagnosis,
                                               Nanometres margin);

struct MoveProposal {
  Point position;
  Microdegrees rotation;
  BoardSide side{BoardSide::front};
  PlacementMoveKind kind{PlacementMoveKind::local_repair};
  std::optional<std::size_t> swap_partner_index;
  std::vector<std::size_t> group_member_indices;
  std::optional<PlacementGroupId> placement_group;
  std::vector<RelativePlacementConstraintId> relative_constraints;
  std::vector<PlacementGroupId> constraint_groups;
};

struct PlacementPolicyClosure {
  std::set<ObjectId> footprints;
  std::vector<RelativePlacementConstraintId> relative_constraints;
  std::vector<PlacementGroupId> placement_groups;
};

struct PlacementPriorityContext {
  bool enabled{false};
  std::vector<PlacementMovePriorityClass> order;
  std::map<PlacementMovePriorityClass, std::size_t> tier_by_class;
  std::map<ObjectId, PlacementMovePriorityClass> class_by_footprint;

  [[nodiscard]] std::size_t tier_count() const { return enabled ? order.size() : 1U; }

  [[nodiscard]] bool allows(const ObjectId& footprint, std::size_t active_tier) const {
    if (!enabled) {
      return true;
    }
    const auto assigned = class_by_footprint.find(footprint);
    return assigned != class_by_footprint.end() &&
           tier_by_class.at(assigned->second) <= active_tier;
  }
};

[[nodiscard]] std::optional<PlacementPriorityContext>
make_placement_priority_context(const Board& source,
                                const std::optional<PlacementMovePriorityPolicy>& policy,
                                std::string& diagnostic) {
  PlacementPriorityContext result;
  if (!policy) {
    return result;
  }
  result.enabled = true;
  if (policy->source_revision != source.source_revision) {
    diagnostic = "placement-move priority policy targets a different source revision";
    return std::nullopt;
  }
  result.order = policy->order;
  if (result.order.size() != default_placement_move_priority_order().size()) {
    diagnostic = "placement-move priority policy must order every supported class exactly once";
    return std::nullopt;
  }
  for (std::size_t index = 0U; index < result.order.size(); ++index) {
    if (!result.tier_by_class.emplace(result.order[index], index).second) {
      diagnostic = "placement-move priority policy contains a duplicate class";
      return std::nullopt;
    }
  }
  if (policy->footprints.size() != source.footprints.size()) {
    diagnostic = "placement-move priority policy must classify every footprint exactly once";
    return std::nullopt;
  }
  std::set<ObjectId> known;
  for (const auto& footprint : source.footprints) {
    known.insert(footprint.id);
  }
  for (const auto& assignment : policy->footprints) {
    if (!known.contains(assignment.footprint) ||
        !result.tier_by_class.contains(assignment.priority_class) ||
        !result.class_by_footprint.emplace(assignment.footprint, assignment.priority_class)
             .second) {
      diagnostic =
          "placement-move priority policy contains an unknown, duplicate, or unordered assignment";
      return std::nullopt;
    }
  }
  return result;
}

[[nodiscard]] PlacementPolicyClosure placement_policy_closure(const Board& board,
                                                              const ObjectId& seed) {
  PlacementPolicyClosure result;
  result.footprints.insert(seed);
  std::set<RelativePlacementConstraintId> relative_ids;
  std::set<PlacementGroupId> group_ids;
  bool changed = true;
  while (changed) {
    changed = false;
    for (const auto& constraint : board.relative_placement_constraints) {
      if (!result.footprints.contains(constraint.first_footprint) &&
          !result.footprints.contains(constraint.second_footprint)) {
        continue;
      }
      relative_ids.insert(constraint.id);
      changed = result.footprints.insert(constraint.first_footprint).second || changed;
      changed = result.footprints.insert(constraint.second_footprint).second || changed;
    }
    for (const auto& group : board.placement_groups) {
      const bool touches =
          std::any_of(group.footprints.begin(), group.footprints.end(),
                      [&](const ObjectId& id) { return result.footprints.contains(id); });
      if (!touches) {
        continue;
      }
      group_ids.insert(group.id);
      for (const auto& id : group.footprints) {
        changed = result.footprints.insert(id).second || changed;
      }
    }
  }
  result.relative_constraints.assign(relative_ids.begin(), relative_ids.end());
  result.placement_groups.assign(group_ids.begin(), group_ids.end());
  return result;
}

[[nodiscard]] const Footprint* find_footprint(const Board& board, const ObjectId& id) {
  const auto found = std::find_if(board.footprints.begin(), board.footprints.end(),
                                  [&id](const Footprint& footprint) { return footprint.id == id; });
  return found == board.footprints.end() ? nullptr : &*found;
}

[[nodiscard]] std::size_t placement_group_violations(const Board& source, const Board& candidate) {
  std::size_t violations = 0U;
  for (const auto& group : source.placement_groups) {
    std::optional<Point> displacement;
    bool invalid = false;
    for (const auto& id : group.footprints) {
      const auto* original = find_footprint(source, id);
      const auto* placed = find_footprint(candidate, id);
      if (original == nullptr || placed == nullptr || original->rotation != placed->rotation ||
          original->side != placed->side) {
        invalid = true;
        break;
      }
      const auto member_displacement = placed->position - original->position;
      if (!displacement) {
        displacement = member_displacement;
      } else if (*displacement != member_displacement) {
        invalid = true;
        break;
      }
    }
    violations += invalid ? 1U : 0U;
  }
  return violations;
}

[[nodiscard]] bool footprint_is_grouped(const Board& board, const ObjectId& id) {
  return std::any_of(board.placement_groups.begin(), board.placement_groups.end(),
                     [&id](const PlacementGroup& group) {
                       return std::find(group.footprints.begin(), group.footprints.end(), id) !=
                              group.footprints.end();
                     });
}

[[nodiscard]] std::uint64_t saturating_add(std::uint64_t lhs, std::uint64_t rhs) {
  const auto remaining = std::numeric_limits<std::uint64_t>::max() - lhs;
  return rhs > remaining ? std::numeric_limits<std::uint64_t>::max() : lhs + rhs;
}

[[nodiscard]] std::size_t saturating_add_size(std::size_t lhs, std::size_t rhs) {
  const auto remaining = std::numeric_limits<std::size_t>::max() - lhs;
  return rhs > remaining ? std::numeric_limits<std::size_t>::max() : lhs + rhs;
}

[[nodiscard]] std::uint64_t displacement(const Board& source, const Board& candidate) {
  std::uint64_t result = 0;
  for (const auto& original : source.footprints) {
    for (const auto& placed : candidate.footprints) {
      if (original.id == placed.id) {
        result =
            saturating_add(result, geometry::segment_length_nm(original.position, placed.position));
        break;
      }
    }
  }
  return result;
}

[[nodiscard]] std::uint64_t routed_length(const Board& board) {
  std::uint64_t result = 0;
  for (const auto& track : board.tracks) {
    result = saturating_add(result, geometry::segment_length_nm(track.start, track.end));
  }
  for (const auto& arc : board.track_arcs) {
    result =
        saturating_add(result, geometry::circular_arc_length_nm(arc.start, arc.middle, arc.end));
  }
  for (const auto& curve : board.track_beziers) {
    const auto flattened = tessellate_track_bezier(curve);
    if (!flattened.certified) return std::numeric_limits<std::uint64_t>::max();
    for (const auto& piece : flattened.segments)
      result = saturating_add(result, geometry::segment_length_nm(piece.start, piece.end));
  }
  return result;
}

[[nodiscard]] std::uint64_t rotation_distance(Microdegrees lhs, Microdegrees rhs) {
  constexpr std::int64_t full_turn = 360'000'000;
  constexpr std::int64_t half_turn = full_turn / 2;
  auto difference = std::abs(lhs.value() - rhs.value());
  if (difference > half_turn) {
    difference = full_turn - difference;
  }
  return static_cast<std::uint64_t>(difference);
}

void source_orientation_cost(const Board& source, const Board& candidate, IntegratedScore& score) {
  for (const auto& original : source.footprints) {
    const auto found =
        std::find_if(candidate.footprints.begin(), candidate.footprints.end(),
                     [&original](const Footprint& placed) { return placed.id == original.id; });
    if (found == candidate.footprints.end()) {
      continue;
    }
    if (found->side != original.side) {
      ++score.side_changes;
    }
    score.rotation_change_microdegrees = saturating_add(
        score.rotation_change_microdegrees, rotation_distance(original.rotation, found->rotation));
  }
}

[[nodiscard]] CandidateEvaluation
evaluate_candidate(const Board& source, Board placement, const IntegratedOptimizerConfig& config,
                   const BoardValidationBaseline& validation_baseline, std::stop_token stop_token,
                   RouterTelemetrySink* telemetry = nullptr,
                   const GlobalRouteResult* checkpoint = nullptr,
                   const std::vector<NetId>& affected_nets = {},
                   const DetailedRouteResult* detailed_checkpoint = nullptr) {
  CandidateEvaluation result;
  result.placement = std::move(placement);
  result.score.configured_dimensioned_physical_overflow =
      config.global_router.checkpoint_objective ==
      GlobalRouteCheckpointObjective::dimensioned_physical_overflow;
  auto candidate_validation = validate_board_candidate(result.placement, validation_baseline);
  result.reused_static_zone_validation = candidate_validation.reused_static_zone_geometry;
  const auto& validation = candidate_validation.validation;
  result.score.placement_violations = validation.error_count();
  const auto group_violations = placement_group_violations(source, result.placement);
  result.score.placement_violations =
      group_violations > std::numeric_limits<std::size_t>::max() - result.score.placement_violations
          ? std::numeric_limits<std::size_t>::max()
          : result.score.placement_violations + group_violations;
  result.score.displacement_nm = displacement(source, result.placement);
  source_orientation_cost(source, result.placement, result.score);
  if (!validation.ok() || group_violations != 0U || stop_token.stop_requested()) {
    result.score.global_route_failures = 1;
    result.score.unrouted_nets = std::numeric_limits<std::size_t>::max();
    result.score.failed_pin_escapes = std::numeric_limits<std::size_t>::max();
    result.score.best_hard_capacity_unrouted_nets = std::numeric_limits<std::size_t>::max();
    if (result.score.configured_dimensioned_physical_overflow) {
      result.score.total_physical_overflow_nm = std::numeric_limits<std::uint64_t>::max();
      result.score.peak_physical_overflow_nm = std::numeric_limits<std::uint64_t>::max();
      result.score.sampled_overflow = std::numeric_limits<std::size_t>::max();
      result.score.corridor_overflow = std::numeric_limits<std::size_t>::max();
      result.score.congested_physical_resources = std::numeric_limits<std::size_t>::max();
    }
    result.score.global_overflow = std::numeric_limits<std::size_t>::max();
    result.score.copper_violations = std::numeric_limits<std::size_t>::max();
    result.score.open_connections = std::numeric_limits<std::size_t>::max();
    result.score.width_reduction_cost = std::numeric_limits<std::uint64_t>::max();
    result.score.routed_length_nm = std::numeric_limits<std::uint64_t>::max();
    result.score.route_cost = std::numeric_limits<std::uint64_t>::max();
    return result;
  }

  if (checkpoint == nullptr) {
    auto planner_config = RoutingPlannerConfig{};
    planner_config.global = config.global_router;
    planner_config.regional = config.regional_router;
    planner_config.curved_tracks = config.detailed_router.circular_arc_refinement;
    result.routing_plan = plan_routes(result.placement, planner_config, stop_token, telemetry);
    result.global = result.routing_plan->coarse_global;
  } else {
    auto incremental_global_config = config.global_router;
    if (!config.detailed_router.circular_arc_refinement) {
      incremental_global_config.use_navmesh_funnel_spline_seeds = false;
      incremental_global_config.single_lane_direct_voronoi_resources = true;
    }
    result.global = route_global_incremental(result.placement, *checkpoint, affected_nets,
                                             incremental_global_config, stop_token, telemetry);
    RoutingPlanResult incremental_plan;
    incremental_plan.coarse_global = result.global;
    auto regional_config = config.regional_router;
    regional_config.enable_spline_realization = config.detailed_router.circular_arc_refinement;
    regional_config.treat_saved_zone_fills_as_refillable =
        config.global_router.treat_saved_zone_fills_as_refillable;
    incremental_plan.regional = analyze_regional_conflicts(result.placement, result.global,
                                                           regional_config, stop_token, telemetry);
    incremental_plan.paths = incremental_plan.regional.spline_candidate_available
                                 ? incremental_plan.regional.spline_candidate_paths
                                 : result.global.paths;
    incremental_plan.stop_reason = result.global.complete() && incremental_plan.regional.complete()
                                       ? RoutingPlanStopReason::complete
                                       : RoutingPlanStopReason::regional_conflicts;
    result.routing_plan = std::move(incremental_plan);
  }
  const auto regional_handoff_ready =
      result.routing_plan && result.global.unrouted_nets.empty() &&
      result.global.unaccepted_selected_voronoi_lane_transitions == 0U &&
      result.routing_plan->regional.spline_candidate_available &&
      result.routing_plan->regional.unmapped_clearance_shadow_conflicts == 0U &&
      result.routing_plan->regional.unmapped_capacity_conflicts == 0U &&
      !result.routing_plan->regional.bundle_assignments.empty() &&
      std::all_of(result.routing_plan->regional.bundle_assignments.begin(),
                  result.routing_plan->regional.bundle_assignments.end(),
                  [](const auto& assignment) { return assignment.complete(); });
  result.score.global_route_failures = result.global.complete() ? 0U : 1U;
  result.score.unrouted_nets = result.global.unrouted_nets.size();
  result.score.failed_pin_escapes = result.global.failed_pin_escapes;
  if (!result.global.hard_capacity_pass_summaries.empty()) {
    result.score.best_hard_capacity_unrouted_nets =
        std::min_element(result.global.hard_capacity_pass_summaries.begin(),
                         result.global.hard_capacity_pass_summaries.end(),
                         [](const auto& lhs, const auto& rhs) {
                           return lhs.unrouted_nets.size() < rhs.unrouted_nets.size();
                         })
            ->unrouted_nets.size();
  }
  result.score.total_physical_overflow_nm = result.global.total_voronoi_overflow_nm;
  result.score.peak_physical_overflow_nm = result.global.peak_voronoi_overflow_nm;
  result.score.sampled_overflow = result.global.sampled_overflow;
  result.score.corridor_overflow = result.global.corridor_overflow;
  result.score.congested_physical_resources = result.global.congested_voronoi_resources;
  result.score.global_overflow = result.global.total_overflow;
  result.score.width_reduction_cost = result.global.total_width_reduction_cost;
  result.score.route_cost = result.global.total_cost;
  if (!result.global.complete() && !regional_handoff_ready) {
    result.score.copper_violations = std::numeric_limits<std::size_t>::max();
    result.score.open_connections = std::numeric_limits<std::size_t>::max();
    result.score.routed_length_nm = std::numeric_limits<std::uint64_t>::max();
    return result;
  }

  auto detailed_config = config.detailed_router;
  detailed_config.existing_routing = ExistingRoutingPolicy::replace_unlocked_for_routed_nets;
  result.detailed =
      detailed_checkpoint == nullptr
          ? materialize_detailed_route(result.placement, *result.routing_plan, detailed_config,
                                       stop_token)
      : regional_handoff_ready
          ? materialize_detailed_route_incremental(result.placement, *result.routing_plan,
                                                   *detailed_checkpoint, detailed_config,
                                                   stop_token)
          : materialize_detailed_route_incremental(
                result.placement, result.global, *detailed_checkpoint, detailed_config, stop_token);
  if (checkpoint == nullptr && result.routing_plan &&
      config.max_final_routability_feedback_passes > 0U && !result.detailed.complete() &&
      !result.detailed.provisional_zone_refill_candidate() &&
      !result.detailed.routability_diagnoses.empty() && !stop_token.stop_requested()) {
    RoutingPlannerConfig planner_config;
    planner_config.global = config.global_router;
    planner_config.regional = config.regional_router;
    planner_config.curved_tracks = config.detailed_router.circular_arc_refinement;
    auto feedback = refine_materialization_with_routability_feedback(
        result.placement, result.placement, std::move(*result.routing_plan),
        std::move(result.detailed), planner_config, detailed_config,
        config.max_final_routability_feedback_passes, stop_token, telemetry);
    result.routing_plan = std::move(feedback.routing);
    result.global = result.routing_plan->coarse_global;
    result.detailed = std::move(feedback.detailed);
    result.score.global_route_failures = result.global.complete() ? 0U : 1U;
    result.score.unrouted_nets = result.global.unrouted_nets.size();
    result.score.failed_pin_escapes = result.global.failed_pin_escapes;
    if (!result.global.hard_capacity_pass_summaries.empty()) {
      result.score.best_hard_capacity_unrouted_nets =
          std::min_element(result.global.hard_capacity_pass_summaries.begin(),
                           result.global.hard_capacity_pass_summaries.end(),
                           [](const auto& lhs, const auto& rhs) {
                             return lhs.unrouted_nets.size() < rhs.unrouted_nets.size();
                           })
              ->unrouted_nets.size();
    }
    result.score.total_physical_overflow_nm = result.global.total_voronoi_overflow_nm;
    result.score.peak_physical_overflow_nm = result.global.peak_voronoi_overflow_nm;
    result.score.sampled_overflow = result.global.sampled_overflow;
    result.score.corridor_overflow = result.global.corridor_overflow;
    result.score.congested_physical_resources = result.global.congested_voronoi_resources;
    result.score.global_overflow = result.global.total_overflow;
    result.score.width_reduction_cost = result.global.total_width_reduction_cost;
    result.score.route_cost = result.global.total_cost;
  }
  if (result.detailed.complete()) {
    result.score.global_route_failures = 0U;
    result.score.total_physical_overflow_nm = 0U;
    result.score.peak_physical_overflow_nm = 0U;
    result.score.sampled_overflow = 0U;
    result.score.corridor_overflow = 0U;
    result.score.congested_physical_resources = 0U;
    result.score.global_overflow = 0U;
  }
  result.score.copper_violations = result.detailed.copper_drc.violation_count();
  result.score.open_connections = result.detailed.connectivity.open_connections;
  if (result.detailed.candidate) {
    result.score.routed_length_nm = routed_length(*result.detailed.candidate);
  } else {
    result.score.copper_violations = std::numeric_limits<std::size_t>::max();
    result.score.open_connections = std::numeric_limits<std::size_t>::max();
    result.score.routed_length_nm = std::numeric_limits<std::uint64_t>::max();
  }
  return result;
}

[[nodiscard]] const RoutingHardCapacityPass*
representative_hard_capacity_pass(const GlobalRouteResult& route);

[[nodiscard]] std::vector<NetId> affected_nets_for_move(const CandidateEvaluation& evaluation,
                                                        std::size_t footprint_index) {
  const auto& footprint = evaluation.placement.footprints[footprint_index];
  std::set<NetId> affected;
  std::set<ObjectId> moved_objects{footprint.id};
  for (const auto& pad : footprint.pads) {
    affected.insert(pad.net);
    moved_objects.insert(pad.id);
  }
  for (const auto& keepout : footprint.keepouts) {
    moved_objects.insert(keepout.id);
  }
  for (const auto& impact : evaluation.global.obstacle_impacts) {
    if (moved_objects.contains(impact.obstacle)) {
      affected.insert(impact.routed_net);
    }
  }
  for (const auto& impact : evaluation.global.neck_down_obstacle_impacts) {
    if (moved_objects.contains(impact.obstacle)) {
      affected.insert(impact.routed_net);
    }
  }
  for (const auto& impact : evaluation.global.physical_obstacle_impacts) {
    if (moved_objects.contains(impact.obstacle)) {
      affected.insert(impact.routed_nets.begin(), impact.routed_nets.end());
    }
  }
  for (const auto& failure : evaluation.global.pin_escape_failures) {
    if (moved_objects.contains(failure.footprint) || moved_objects.contains(failure.pad)) {
      affected.insert(failure.net);
    }
  }
  for (const auto& failure : evaluation.global.connection_failures) {
    const auto terminal_moved = [&moved_objects](const RoutingTerminalEvidence& terminal) {
      return moved_objects.contains(terminal.footprint) || moved_objects.contains(terminal.pad);
    };
    if (std::any_of(failure.attempted_terminals.begin(), failure.attempted_terminals.end(),
                    terminal_moved) ||
        std::any_of(failure.connected_terminals.begin(), failure.connected_terminals.end(),
                    terminal_moved)) {
      affected.insert(failure.net);
    }
  }
  if (const auto* hard_pass = representative_hard_capacity_pass(evaluation.global)) {
    for (const auto& failure : hard_pass->connection_failures) {
      const auto terminal_moved = [&moved_objects](const RoutingTerminalEvidence& terminal) {
        return moved_objects.contains(terminal.footprint) || moved_objects.contains(terminal.pad);
      };
      if (std::any_of(failure.attempted_terminals.begin(), failure.attempted_terminals.end(),
                      terminal_moved) ||
          std::any_of(failure.connected_terminals.begin(), failure.connected_terminals.end(),
                      terminal_moved)) {
        affected.insert(failure.net);
      }
    }
  }
  for (const auto& diagnosis : evaluation.detailed.routability_diagnoses) {
    if (!diagnosis.locally_resolved &&
        diagnosis_touches_footprint(footprint, diagnosis, Nanometres{0})) {
      affected.insert(diagnosis.resource.nets.begin(), diagnosis.resource.nets.end());
    }
  }
  return {affected.begin(), affected.end()};
}

[[nodiscard]] std::vector<NetId> affected_nets_for_proposal(const CandidateEvaluation& evaluation,
                                                            std::size_t footprint_index,
                                                            const MoveProposal& proposal) {
  auto result = affected_nets_for_move(evaluation, footprint_index);
  std::vector<std::size_t> additional;
  if (proposal.swap_partner_index) {
    additional.push_back(*proposal.swap_partner_index);
  }
  additional.insert(additional.end(), proposal.group_member_indices.begin(),
                    proposal.group_member_indices.end());
  std::sort(additional.begin(), additional.end());
  additional.erase(std::unique(additional.begin(), additional.end()), additional.end());
  for (const auto index : additional) {
    if (index == footprint_index) {
      continue;
    }
    const auto member = affected_nets_for_move(evaluation, index);
    result.insert(result.end(), member.begin(), member.end());
  }
  std::sort(result.begin(), result.end());
  result.erase(std::unique(result.begin(), result.end()), result.end());
  return result;
}

[[nodiscard]] std::vector<Point> move_offsets(const IntegratedOptimizerConfig& config) {
  const auto step = config.move_step;
  std::vector<Point> result{
      {step, Nanometres{0}}, {-step, Nanometres{0}}, {Nanometres{0}, step}, {Nanometres{0}, -step}};
  if (config.allow_diagonal_moves) {
    result.insert(result.end(), {{step, step}, {step, -step}, {-step, step}, {-step, -step}});
  }
  return result;
}

[[nodiscard]] Bounds polygon_bounds(const Polygon& polygon) {
  Bounds result{static_cast<long double>(polygon.vertices.front().x.value()),
                static_cast<long double>(polygon.vertices.front().x.value()),
                static_cast<long double>(polygon.vertices.front().y.value()),
                static_cast<long double>(polygon.vertices.front().y.value())};
  for (const auto point : polygon.vertices) {
    result.min_x = std::min(result.min_x, static_cast<long double>(point.x.value()));
    result.max_x = std::max(result.max_x, static_cast<long double>(point.x.value()));
    result.min_y = std::min(result.min_y, static_cast<long double>(point.y.value()));
    result.max_y = std::max(result.max_y, static_cast<long double>(point.y.value()));
  }
  return result;
}

[[nodiscard]] bool diagnosis_touches_footprint(const Footprint& footprint,
                                               const RoutabilityDiagnosis& diagnosis,
                                               Nanometres margin) {
  const auto connected =
      std::any_of(footprint.pads.begin(), footprint.pads.end(), [&](const Pad& pad) {
        return std::find(diagnosis.resource.nets.begin(), diagnosis.resource.nets.end(), pad.net) !=
               diagnosis.resource.nets.end();
      });
  if (connected) {
    return true;
  }
  const auto courtyard = polygon_bounds(geometry::transform_polygon(
      footprint.courtyard, footprint.position, footprint.rotation, footprint.side));
  const auto expansion = static_cast<long double>(margin.value());
  const auto& diagnosed = diagnosis.resource.bounds;
  return courtyard.max_x + expansion >= static_cast<long double>(diagnosed.minimum.x.value()) &&
         courtyard.min_x - expansion <= static_cast<long double>(diagnosed.maximum.x.value()) &&
         courtyard.max_y + expansion >= static_cast<long double>(diagnosed.minimum.y.value()) &&
         courtyard.min_y - expansion <= static_cast<long double>(diagnosed.maximum.y.value());
}

[[nodiscard]] Bounds point_bounds(Point point, long double radius = 0.0L) {
  return Bounds{static_cast<long double>(point.x.value()) - radius,
                static_cast<long double>(point.x.value()) + radius,
                static_cast<long double>(point.y.value()) - radius,
                static_cast<long double>(point.y.value()) + radius};
}

[[nodiscard]] std::optional<Bounds> object_bounds(const Board& board, const ObjectId& id) {
  for (const auto& footprint : board.footprints) {
    if (footprint.id == id) {
      return polygon_bounds(geometry::transform_polygon(footprint.courtyard, footprint.position,
                                                        footprint.rotation, footprint.side));
    }
    for (const auto& pad : footprint.pads) {
      if (pad.id == id) {
        const auto position = geometry::transform_point(pad.local_position, footprint.position,
                                                        footprint.rotation, footprint.side);
        const auto radius = conservative_pad_radius_nm(pad);
        return point_bounds(position, radius);
      }
    }
    for (const auto& keepout : footprint.keepouts) {
      if (keepout.id == id) {
        return polygon_bounds(geometry::transform_polygon(keepout.area, footprint.position,
                                                          footprint.rotation, footprint.side));
      }
    }
  }
  for (const auto& track : board.tracks) {
    if (track.id == id) {
      const auto radius = static_cast<long double>(track.width.value()) / 2.0L;
      return Bounds{static_cast<long double>(std::min(track.start.x, track.end.x).value()) - radius,
                    static_cast<long double>(std::max(track.start.x, track.end.x).value()) + radius,
                    static_cast<long double>(std::min(track.start.y, track.end.y).value()) - radius,
                    static_cast<long double>(std::max(track.start.y, track.end.y).value()) +
                        radius};
    }
  }
  for (const auto& curve : board.track_beziers) {
    if (curve.id != id) continue;
    auto bounds = point_bounds(curve.start, static_cast<long double>(curve.width.value()) / 2.0L);
    for (const auto p : {curve.control1, curve.control2, curve.end}) {
      const auto b = point_bounds(p, static_cast<long double>(curve.width.value()) / 2.0L);
      bounds.min_x = std::min(bounds.min_x, b.min_x); bounds.max_x = std::max(bounds.max_x, b.max_x);
      bounds.min_y = std::min(bounds.min_y, b.min_y); bounds.max_y = std::max(bounds.max_y, b.max_y);
    }
    return bounds;
  }
  for (const auto& arc : board.track_arcs) {
    if (arc.id == id) {
      const auto points = geometry::circular_arc_extrema(arc.start, arc.middle, arc.end);
      auto bounds =
          point_bounds(points.front(), static_cast<long double>(arc.width.value()) / 2.0L);
      for (std::size_t index = 1; index < points.size(); ++index) {
        const auto point =
            point_bounds(points[index], static_cast<long double>(arc.width.value()) / 2.0L);
        bounds.min_x = std::min(bounds.min_x, point.min_x);
        bounds.max_x = std::max(bounds.max_x, point.max_x);
        bounds.min_y = std::min(bounds.min_y, point.min_y);
        bounds.max_y = std::max(bounds.max_y, point.max_y);
      }
      return bounds;
    }
  }
  for (const auto& via : board.vias) {
    if (via.id == id) {
      return point_bounds(via.position, static_cast<long double>(via.diameter.value()) / 2.0L);
    }
  }
  for (const auto& zone : board.zones) {
    if (zone.id == id && !zone.fills.empty()) {
      auto bounds = polygon_bounds(zone.fills.front().area);
      for (std::size_t index = 1; index < zone.fills.size(); ++index) {
        const auto fill = polygon_bounds(zone.fills[index].area);
        bounds.min_x = std::min(bounds.min_x, fill.min_x);
        bounds.max_x = std::max(bounds.max_x, fill.max_x);
        bounds.min_y = std::min(bounds.min_y, fill.min_y);
        bounds.max_y = std::max(bounds.max_y, fill.max_y);
      }
      return bounds;
    }
  }
  for (const auto& keepout : board.keepouts) {
    if (keepout.id == id) {
      return polygon_bounds(keepout.area);
    }
  }
  return std::nullopt;
}

[[nodiscard]] std::optional<Point> bounded_point(long double x, long double y) {
  const auto limit = static_cast<long double>(max_abs_coordinate_nm);
  if (!std::isfinite(x) || !std::isfinite(y) || x < -limit || x > limit || y < -limit ||
      y > limit) {
    return std::nullopt;
  }
  return Point{Nanometres{static_cast<std::int64_t>(std::llround(x))},
               Nanometres{static_cast<std::int64_t>(std::llround(y))}};
}

[[nodiscard]] std::optional<Point>
other_pin_centroid(const Board& board, std::size_t footprint_index, const NetId& net) {
  long double x = 0.0L;
  long double y = 0.0L;
  std::size_t count = 0;
  for (std::size_t index = 0; index < board.footprints.size(); ++index) {
    if (index == footprint_index) {
      continue;
    }
    const auto& footprint = board.footprints[index];
    for (const auto& pad : footprint.pads) {
      if (pad.net == net) {
        const auto position = geometry::transform_point(pad.local_position, footprint.position,
                                                        footprint.rotation, footprint.side);
        x += static_cast<long double>(position.x.value());
        y += static_cast<long double>(position.y.value());
        ++count;
      }
    }
  }
  return count == 0U ? std::nullopt
                     : bounded_point(x / static_cast<long double>(count),
                                     y / static_cast<long double>(count));
}

[[nodiscard]] bool contains_net(const std::set<NetId>& nets, const std::vector<NetId>& candidates) {
  return std::any_of(candidates.begin(), candidates.end(),
                     [&nets](const NetId& net) { return nets.contains(net); });
}

[[nodiscard]] bool footprint_owns_object(const Footprint& footprint, const ObjectId& object) {
  return footprint.id == object ||
         std::any_of(footprint.pads.begin(), footprint.pads.end(),
                     [&object](const Pad& pad) { return pad.id == object; }) ||
         std::any_of(footprint.keepouts.begin(), footprint.keepouts.end(),
                     [&object](const Keepout& keepout) { return keepout.id == object; });
}

[[nodiscard]] std::optional<Point> perpendicular_pressure_target(const Footprint& footprint,
                                                                 Point first, Point second,
                                                                 Nanometres move_step) {
  const auto midpoint_x =
      (static_cast<long double>(first.x.value()) + static_cast<long double>(second.x.value())) /
      2.0L;
  const auto midpoint_y =
      (static_cast<long double>(first.y.value()) + static_cast<long double>(second.y.value())) /
      2.0L;
  const auto dx = static_cast<long double>(footprint.position.x.value()) - midpoint_x;
  const auto dy = static_cast<long double>(footprint.position.y.value()) - midpoint_y;
  const auto step = static_cast<long double>(move_step.value());
  const auto edge_dx = second.x.value() - first.x.value();
  const auto edge_dy = second.y.value() - first.y.value();
  if (std::abs(edge_dx) >= std::abs(edge_dy)) {
    return bounded_point(static_cast<long double>(footprint.position.x.value()),
                         static_cast<long double>(footprint.position.y.value()) +
                             (dy < 0.0L ? -step : step));
  }
  return bounded_point(static_cast<long double>(footprint.position.x.value()) +
                           (dx < 0.0L ? -step : step),
                       static_cast<long double>(footprint.position.y.value()));
}

[[nodiscard]] std::vector<Point>
neck_down_obstacle_positions(const CandidateEvaluation& evaluation, std::size_t footprint_index,
                             const IntegratedOptimizerConfig& config) {
  std::vector<Point> result;
  std::set<Point> unique;
  const auto& footprint = evaluation.placement.footprints[footprint_index];
  for (const auto& impact : evaluation.global.neck_down_obstacle_impacts) {
    if (!footprint_owns_object(footprint, impact.obstacle)) {
      continue;
    }
    const auto target =
        perpendicular_pressure_target(footprint, impact.start, impact.end, config.move_step);
    if (target && *target != footprint.position && unique.insert(*target).second &&
        result.size() < config.max_routing_guided_targets_per_footprint) {
      result.push_back(*target);
    }
  }
  return result;
}

[[nodiscard]] std::vector<Point>
physical_channel_obstacle_positions(const CandidateEvaluation& evaluation,
                                    std::size_t footprint_index,
                                    const IntegratedOptimizerConfig& config) {
  std::vector<Point> result;
  std::set<Point> unique;
  const auto& footprint = evaluation.placement.footprints[footprint_index];
  for (const auto& impact : evaluation.global.physical_obstacle_impacts) {
    if (!footprint_owns_object(footprint, impact.obstacle)) {
      continue;
    }
    const auto target =
        perpendicular_pressure_target(footprint, impact.start, impact.end, config.move_step);
    if (target && *target != footprint.position && unique.insert(*target).second &&
        result.size() < config.max_routing_guided_targets_per_footprint) {
      result.push_back(*target);
    }
  }
  return result;
}

[[nodiscard]] std::vector<Point> routing_guided_positions(const CandidateEvaluation& evaluation,
                                                          std::size_t footprint_index,
                                                          const IntegratedOptimizerConfig& config) {
  std::vector<Point> result;
  std::set<Point> unique;
  const auto& board = evaluation.placement;
  const auto& footprint = board.footprints[footprint_index];
  std::set<NetId> footprint_nets;
  for (const auto& pad : footprint.pads) {
    footprint_nets.insert(pad.net);
  }
  const auto append = [&](Point point) {
    if (point != footprint.position &&
        result.size() < config.max_routing_guided_targets_per_footprint &&
        unique.insert(point).second) {
      result.push_back(point);
    }
  };

  const auto* representative_hard_pass = representative_hard_capacity_pass(evaluation.global);
  const bool failed_endpoint =
      std::any_of(evaluation.global.pin_escape_failures.begin(),
                  evaluation.global.pin_escape_failures.end(),
                  [&](const auto& failure) { return failure.footprint == footprint.id; }) ||
      std::any_of(evaluation.global.connection_failures.begin(),
                  evaluation.global.connection_failures.end(),
                  [&](const auto& failure) {
                    return failure.reason == RoutingConnectionFailureReason::no_access_nodes &&
                           std::any_of(failure.attempted_terminals.begin(),
                                       failure.attempted_terminals.end(),
                                       [&](const RoutingTerminalEvidence& terminal) {
                                         return terminal.footprint == footprint.id;
                                       });
                  }) ||
      (representative_hard_pass != nullptr &&
       std::any_of(representative_hard_pass->connection_failures.begin(),
                   representative_hard_pass->connection_failures.end(), [&](const auto& failure) {
                     return std::any_of(failure.attempted_terminals.begin(),
                                        failure.attempted_terminals.end(),
                                        [&](const RoutingTerminalEvidence& terminal) {
                                          return terminal.footprint == footprint.id;
                                        });
                   }));
  if (failed_endpoint) {
    const auto step = std::min(config.move_step, config.global_router.grid_pitch);
    if (step.value() > 0) {
      const std::array<Point, 8U> fine_offsets{
          Point{step, Nanometres{0}},  Point{-step, Nanometres{0}}, Point{Nanometres{0}, step},
          Point{Nanometres{0}, -step}, Point{step, step},           Point{step, -step},
          Point{-step, step},          Point{-step, -step}};
      for (const auto offset : fine_offsets) {
        const auto target = bounded_point(static_cast<long double>(footprint.position.x.value()) +
                                              static_cast<long double>(offset.x.value()),
                                          static_cast<long double>(footprint.position.y.value()) +
                                              static_cast<long double>(offset.y.value()));
        if (target) {
          append(*target);
        }
      }
    }
  }

  const auto append_pressure_target = [&](Point first, Point second, bool segment_like) {
    const auto midpoint_x =
        (static_cast<long double>(first.x.value()) + static_cast<long double>(second.x.value())) /
        2.0L;
    const auto midpoint_y =
        (static_cast<long double>(first.y.value()) + static_cast<long double>(second.y.value())) /
        2.0L;
    const auto dx = static_cast<long double>(footprint.position.x.value()) - midpoint_x;
    const auto dy = static_cast<long double>(footprint.position.y.value()) - midpoint_y;
    const auto step = static_cast<long double>(config.move_step.value());
    std::optional<Point> target;
    if (segment_like) {
      target = perpendicular_pressure_target(footprint, first, second, config.move_step);
    } else if (std::abs(dx) >= std::abs(dy)) {
      target = bounded_point(static_cast<long double>(footprint.position.x.value()) +
                                 (dx < 0.0L ? -step : step),
                             static_cast<long double>(footprint.position.y.value()));
    } else {
      target = bounded_point(static_cast<long double>(footprint.position.x.value()),
                             static_cast<long double>(footprint.position.y.value()) +
                                 (dy < 0.0L ? -step : step));
    }
    if (target) {
      append(*target);
    }
  };

  for (const auto& usage : evaluation.global.resource_usage) {
    if (!usage.congested() || !contains_net(footprint_nets, usage.nets)) {
      continue;
    }
    append_pressure_target(usage.first.position, usage.second.position,
                           usage.kind == RoutingResourceKind::planar_edge ||
                               usage.kind == RoutingResourceKind::medial_edge);
  }
  for (const auto& usage : evaluation.global.voronoi_resource_usage) {
    if (!usage.congested() || !contains_net(footprint_nets, usage.nets)) {
      continue;
    }
    append_pressure_target(usage.start, usage.end, true);
  }
  if (config.global_router.checkpoint_objective ==
      GlobalRouteCheckpointObjective::dimensioned_physical_overflow) {
    const auto append_corridor_targets = [&](const std::vector<RoutingCorridorUsage>& usages) {
      for (const auto& usage : usages) {
        if (!usage.congested() || !contains_net(footprint_nets, usage.nets)) {
          continue;
        }
        append_pressure_target(usage.sampled_medial_start, usage.sampled_medial_end, true);
      }
    };
    append_corridor_targets(evaluation.global.corridor_usage);
    append_corridor_targets(evaluation.global.corridor_neck_down_usage);
  }
  if (evaluation.routing_plan) {
    const auto& regional = evaluation.routing_plan->regional;
    for (const auto& window : regional.windows) {
      if (!contains_net(footprint_nets, window.nets)) {
        continue;
      }
      const auto assignment =
          std::find_if(regional.bundle_assignments.begin(), regional.bundle_assignments.end(),
                       [&](const auto& candidate) { return candidate.window_id == window.id; });
      const auto unresolved =
          !window.eligible() || assignment == regional.bundle_assignments.end() ||
          !assignment->complete() || assignment->selected_residual_conflicts != 0U;
      if (unresolved || !evaluation.detailed.complete()) {
        append_pressure_target(window.bounds.minimum, window.bounds.maximum, false);
      }
    }
    for (const auto& edge : regional.conflict_edges) {
      if (std::none_of(
              edge.attributed_obstacles.begin(), edge.attributed_obstacles.end(),
              [&](const auto& obstacle) { return footprint_owns_object(footprint, obstacle); })) {
        continue;
      }
      append_pressure_target(edge.conflict_bounds.minimum, edge.conflict_bounds.maximum, false);
    }
  }
  for (const auto& diagnosis : evaluation.detailed.routability_diagnoses) {
    if (diagnosis.locally_resolved ||
        !diagnosis_touches_footprint(footprint, diagnosis, config.routing_feedback_margin)) {
      continue;
    }
    append_pressure_target(diagnosis.resource.bounds.minimum, diagnosis.resource.bounds.maximum,
                           false);
  }
  for (const auto& path : evaluation.global.paths) {
    if (!footprint_nets.contains(path.net) ||
        path.segment_widths.size() + 1U != path.points.size() ||
        path.segment_width_kinds.size() != path.segment_widths.size()) {
      continue;
    }
    for (std::size_t index = 0U; index < path.segment_widths.size(); ++index) {
      if (path.segment_widths[index].value() <= 0 ||
          path.segment_width_kinds[index] == GlobalRouteSegmentWidthKind::nominal) {
        continue;
      }
      append_pressure_target(path.points[index].position, path.points[index + 1U].position, true);
    }
  }

  const auto relative_courtyard = polygon_bounds(geometry::transform_polygon(
      footprint.courtyard, Point{}, footprint.rotation, footprint.side));
  for (const auto& impact : evaluation.global.obstacle_impacts) {
    // Placement repair must attribute an obstacle to its physical owner, not only to nets
    // electrically connected to that footprint.  A component keepout commonly blocks another
    // net's escape while the component's own connection remains perfectly routable.
    if (!footprint_owns_object(footprint, impact.obstacle)) {
      continue;
    }
    const auto obstacle = object_bounds(board, impact.obstacle);
    const auto centroid = other_pin_centroid(board, footprint_index, impact.routed_net);
    if (!obstacle || !centroid) {
      continue;
    }
    const auto displacement_x = footprint.position.x.value() - centroid->x.value();
    const auto displacement_y = footprint.position.y.value() - centroid->y.value();
    const auto step = static_cast<long double>(config.move_step.value());
    const auto escape = std::abs(displacement_x) >= std::abs(displacement_y)
                            ? bounded_point(static_cast<long double>(footprint.position.x.value()) +
                                                (displacement_x < 0 ? -step : step),
                                            static_cast<long double>(footprint.position.y.value()))
                            : bounded_point(static_cast<long double>(footprint.position.x.value()),
                                            static_cast<long double>(footprint.position.y.value()) +
                                                (displacement_y < 0 ? -step : step));
    if (escape) {
      // One perpendicular escape target avoids averaging incompatible left/right/up/down
      // suggestions for the same blocked route.  Additional impacts may still contribute their
      // own bounded targets below.
      append(*escape);
      continue;
    }
    const auto margin = static_cast<long double>(config.routing_feedback_margin.value()) +
                        static_cast<long double>(
                            resolve_net_routing_rules(board, impact.routed_net).clearance.value());
    std::vector<Point> targets;
    const auto append_target = [&targets](std::optional<Point> target) {
      if (target) {
        targets.push_back(*target);
      }
    };
    append_target(bounded_point(obstacle->min_x - margin - relative_courtyard.max_x,
                                static_cast<long double>(footprint.position.y.value())));
    append_target(bounded_point(obstacle->max_x + margin - relative_courtyard.min_x,
                                static_cast<long double>(footprint.position.y.value())));
    append_target(bounded_point(static_cast<long double>(footprint.position.x.value()),
                                obstacle->min_y - margin - relative_courtyard.max_y));
    append_target(bounded_point(static_cast<long double>(footprint.position.x.value()),
                                obstacle->max_y + margin - relative_courtyard.min_y));
    std::sort(targets.begin(), targets.end(), [&centroid](Point lhs, Point rhs) {
      const auto distance = [&centroid](Point point) {
        return std::abs(point.x.value() - centroid->x.value()) +
               std::abs(point.y.value() - centroid->y.value());
      };
      const auto lhs_distance = distance(lhs);
      const auto rhs_distance = distance(rhs);
      return lhs_distance != rhs_distance ? lhs_distance < rhs_distance : lhs < rhs;
    });
    for (const auto target : targets) {
      append(target);
    }
  }
  return result;
}

[[nodiscard]] std::vector<MoveProposal> move_proposals(const CandidateEvaluation& evaluation,
                                                       std::size_t footprint_index,
                                                       const IntegratedOptimizerConfig& config) {
  std::vector<MoveProposal> result;
  std::set<std::tuple<Point, Microdegrees, BoardSide>> unique;
  const auto& footprint = evaluation.placement.footprints[footprint_index];
  const bool orientation_in_scope =
      config.orientation_scope.empty() ||
      std::find(config.orientation_scope.begin(), config.orientation_scope.end(), footprint.id) !=
          config.orientation_scope.end();
  const auto rotation_allowed = [&footprint](Microdegrees rotation) {
    const auto& allowed = footprint.orientation_constraints.allowed_rotations;
    return allowed.empty() || std::find(allowed.begin(), allowed.end(), rotation) != allowed.end();
  };
  const auto side_allowed = [&footprint](BoardSide side) {
    return side == BoardSide::front ? footprint.orientation_constraints.allow_front
                                    : footprint.orientation_constraints.allow_back;
  };
  const auto append = [&](Point position, Microdegrees rotation, BoardSide side,
                          PlacementMoveKind kind) {
    const auto state = std::tuple{position, rotation, side};
    if (rotation_allowed(rotation) && side_allowed(side) &&
        state != std::tuple{footprint.position, footprint.rotation, footprint.side} &&
        unique.insert(state).second) {
      result.push_back(
          MoveProposal{position, rotation, side, kind, std::nullopt, {}, std::nullopt, {}, {}});
    }
  };

  std::vector<Microdegrees> alternative_rotations;
  alternative_rotations.reserve(config.rotation_offsets.size());
  if (orientation_in_scope) {
    for (const auto offset : config.rotation_offsets) {
      const auto rotation = Microdegrees{footprint.rotation.value() + offset.value()};
      if (rotation != footprint.rotation &&
          std::find(alternative_rotations.begin(), alternative_rotations.end(), rotation) ==
              alternative_rotations.end()) {
        alternative_rotations.push_back(rotation);
      }
    }
  }
  std::size_t orientation_count = 0;
  const auto append_orientation = [&](Microdegrees rotation, BoardSide side,
                                      PlacementMoveKind kind) {
    if (orientation_count >= config.max_orientation_proposals_per_footprint) {
      return;
    }
    const auto old_size = result.size();
    append(footprint.position, rotation, side, kind);
    if (result.size() != old_size) {
      ++orientation_count;
    }
  };
  for (const auto rotation : alternative_rotations) {
    append_orientation(rotation, footprint.side, PlacementMoveKind::rotation_repair);
  }
  if (config.enable_side_flips && orientation_in_scope) {
    const auto opposite = footprint.side == BoardSide::front ? BoardSide::back : BoardSide::front;
    append_orientation(footprint.rotation, opposite, PlacementMoveKind::side_flip_repair);
    for (const auto rotation : alternative_rotations) {
      append_orientation(rotation, opposite, PlacementMoveKind::orientation_repair);
    }
  }
  if (config.enable_routing_guided_moves) {
    for (const auto position : neck_down_obstacle_positions(evaluation, footprint_index, config)) {
      append(position, footprint.rotation, footprint.side,
             PlacementMoveKind::neck_down_obstacle_repair);
    }
    for (const auto position :
         physical_channel_obstacle_positions(evaluation, footprint_index, config)) {
      append(position, footprint.rotation, footprint.side,
             PlacementMoveKind::physical_channel_obstacle_repair);
    }
    for (const auto position : routing_guided_positions(evaluation, footprint_index, config)) {
      append(position, footprint.rotation, footprint.side,
             PlacementMoveKind::routing_guided_repair);
    }
  }
  const auto origin = footprint.position;
  for (const auto offset : move_offsets(config)) {
    const auto position = bounded_point(
        static_cast<long double>(origin.x.value()) + static_cast<long double>(offset.x.value()),
        static_cast<long double>(origin.y.value()) + static_cast<long double>(offset.y.value()));
    if (position) {
      append(*position, footprint.rotation, footprint.side, PlacementMoveKind::local_repair);
    }
  }
  return result;
}

[[nodiscard]] std::optional<std::pair<LayerId, LayerId>> outer_copper_layers(const Board& board) {
  std::vector<const Layer*> copper;
  for (const auto& layer : board.layers) {
    if (layer.kind == LayerKind::copper) {
      copper.push_back(&layer);
    }
  }
  if (copper.size() < 2U) {
    return std::nullopt;
  }
  std::sort(copper.begin(), copper.end(), [](const Layer* lhs, const Layer* rhs) {
    return lhs->stack_order != rhs->stack_order ? lhs->stack_order < rhs->stack_order
                                                : lhs->id < rhs->id;
  });
  return std::pair{copper.front()->id, copper.back()->id};
}

void remap_outer_layers(std::vector<LayerId>& layers, const Board& board,
                        const std::pair<LayerId, LayerId>& outer) {
  for (auto& layer : layers) {
    if (layer == outer.first) {
      layer = outer.second;
    } else if (layer == outer.second) {
      layer = outer.first;
    }
  }
  const auto stack_order = [&board](const LayerId& id) {
    const auto found = std::find_if(board.layers.begin(), board.layers.end(),
                                    [&id](const Layer& layer) { return layer.id == id; });
    return found == board.layers.end() ? std::numeric_limits<std::uint32_t>::max()
                                       : found->stack_order;
  };
  std::sort(layers.begin(), layers.end(), [&stack_order](const LayerId& lhs, const LayerId& rhs) {
    const auto lhs_order = stack_order(lhs);
    const auto rhs_order = stack_order(rhs);
    return lhs_order != rhs_order ? lhs_order < rhs_order : lhs < rhs;
  });
  layers.erase(std::unique(layers.begin(), layers.end()), layers.end());
}

[[nodiscard]] bool apply_proposal(Board& candidate, std::size_t footprint_index,
                                  const MoveProposal& proposal) {
  if (!proposal.group_member_indices.empty()) {
    if (std::find(proposal.group_member_indices.begin(), proposal.group_member_indices.end(),
                  footprint_index) == proposal.group_member_indices.end()) {
      return false;
    }
    const auto& primary = candidate.footprints[footprint_index];
    const auto delta_x = static_cast<long double>(proposal.position.x.value()) -
                         static_cast<long double>(primary.position.x.value());
    const auto delta_y = static_cast<long double>(proposal.position.y.value()) -
                         static_cast<long double>(primary.position.y.value());
    std::vector<std::pair<std::size_t, Point>> translated;
    translated.reserve(proposal.group_member_indices.size());
    std::set<std::size_t> unique;
    for (const auto member_index : proposal.group_member_indices) {
      if (member_index >= candidate.footprints.size() || !unique.insert(member_index).second ||
          candidate.footprints[member_index].locked) {
        return false;
      }
      const auto& member = candidate.footprints[member_index];
      const auto position =
          bounded_point(static_cast<long double>(member.position.x.value()) + delta_x,
                        static_cast<long double>(member.position.y.value()) + delta_y);
      if (!position) {
        return false;
      }
      translated.emplace_back(member_index, *position);
    }
    for (const auto& [member_index, position] : translated) {
      candidate.footprints[member_index].position = position;
    }
    return true;
  }
  if (proposal.swap_partner_index) {
    const auto partner_index = *proposal.swap_partner_index;
    if (partner_index >= candidate.footprints.size() || partner_index == footprint_index ||
        candidate.footprints[footprint_index].locked ||
        candidate.footprints[partner_index].locked) {
      return false;
    }
    std::swap(candidate.footprints[footprint_index].position,
              candidate.footprints[partner_index].position);
    return true;
  }
  auto& footprint = candidate.footprints[footprint_index];
  if (proposal.side != footprint.side) {
    const auto outer = outer_copper_layers(candidate);
    if (!outer) {
      return false;
    }
    for (auto& pad : footprint.pads) {
      remap_outer_layers(pad.layers, candidate, *outer);
    }
    for (auto& keepout : footprint.keepouts) {
      remap_outer_layers(keepout.layers, candidate, *outer);
    }
  }
  footprint.position = proposal.position;
  footprint.rotation = proposal.rotation;
  footprint.side = proposal.side;
  return true;
}

[[nodiscard]] std::optional<std::size_t>
proposal_priority_tier(const CandidateEvaluation& evaluation, std::size_t footprint_index,
                       const MoveProposal& proposal, const PlacementPriorityContext& priorities) {
  if (!priorities.enabled) {
    return 0U;
  }
  std::vector<std::size_t> moved{footprint_index};
  if (proposal.swap_partner_index) {
    moved.push_back(*proposal.swap_partner_index);
  }
  moved.insert(moved.end(), proposal.group_member_indices.begin(),
               proposal.group_member_indices.end());
  std::sort(moved.begin(), moved.end());
  moved.erase(std::unique(moved.begin(), moved.end()), moved.end());
  std::size_t required = 0U;
  for (const auto index : moved) {
    if (index >= evaluation.placement.footprints.size()) {
      return std::nullopt;
    }
    const auto assigned =
        priorities.class_by_footprint.find(evaluation.placement.footprints[index].id);
    if (assigned == priorities.class_by_footprint.end()) {
      return std::nullopt;
    }
    required = std::max(required, priorities.tier_by_class.at(assigned->second));
  }
  return required;
}

[[nodiscard]] Board priority_scoped_board(const Board& source,
                                          const PlacementPriorityContext& priorities,
                                          std::size_t active_tier) {
  auto result = source;
  if (!priorities.enabled) {
    return result;
  }
  for (auto& footprint : result.footprints) {
    if (!priorities.allows(footprint.id, active_tier)) {
      footprint.locked = true;
    }
  }
  return result;
}

void restore_source_locks(Board& candidate, const Board& source) {
  for (auto& footprint : candidate.footprints) {
    const auto original = find_footprint(source, footprint.id);
    if (original != nullptr) {
      footprint.locked = original->locked;
    }
  }
}

[[nodiscard]] bool footprint_transforms_changed(const Board& lhs, const Board& rhs) {
  return std::any_of(lhs.footprints.begin(), lhs.footprints.end(), [&](const Footprint& footprint) {
    const auto* other = find_footprint(rhs, footprint.id);
    return other == nullptr || footprint.position != other->position ||
           footprint.rotation != other->rotation || footprint.side != other->side;
  });
}

[[nodiscard]] std::vector<std::size_t>
feedback_ordered_footprints(const CandidateEvaluation& evaluation,
                            const IntegratedOptimizerConfig& config) {
  const auto ranked = collect_integrated_footprint_pressure(
      evaluation.placement, evaluation.global, config.global_router.checkpoint_objective);
  std::map<ObjectId, std::size_t> index_by_id;
  for (std::size_t index = 0U; index < evaluation.placement.footprints.size(); ++index) {
    index_by_id.emplace(evaluation.placement.footprints[index].id, index);
  }
  std::vector<std::size_t> result;
  result.reserve(ranked.size());
  for (const auto& item : ranked) {
    result.push_back(index_by_id.at(item.footprint));
  }
  std::vector<std::uint64_t> diagnosis_pressure(evaluation.placement.footprints.size(), 0U);
  for (std::size_t index = 0U; index < evaluation.placement.footprints.size(); ++index) {
    for (const auto& diagnosis : evaluation.detailed.routability_diagnoses) {
      if (!diagnosis.locally_resolved &&
          diagnosis_touches_footprint(evaluation.placement.footprints[index], diagnosis,
                                      config.routing_feedback_margin)) {
        diagnosis_pressure[index] = saturating_add(diagnosis_pressure[index],
                                                   std::max<std::uint64_t>(1U, diagnosis.severity));
      }
    }
  }
  std::stable_sort(result.begin(), result.end(), [&](const auto lhs, const auto rhs) {
    return diagnosis_pressure[lhs] > diagnosis_pressure[rhs];
  });
  return result;
}

[[nodiscard]] std::vector<MoveProposal>
swap_proposals(const CandidateEvaluation& evaluation, std::size_t footprint_index,
               const std::vector<std::size_t>& ordered_footprints,
               const IntegratedOptimizerConfig& config,
               std::set<std::pair<std::size_t, std::size_t>>& proposed_pairs) {
  std::vector<MoveProposal> result;
  if (!config.enable_swap_moves) {
    return result;
  }
  const auto& footprint = evaluation.placement.footprints[footprint_index];
  for (const auto partner_index : ordered_footprints) {
    if (result.size() >= config.max_swap_partners_per_footprint) {
      break;
    }
    if (partner_index == footprint_index) {
      continue;
    }
    const auto& partner = evaluation.placement.footprints[partner_index];
    if (partner.locked || footprint_is_grouped(evaluation.placement, partner.id) ||
        partner.position == footprint.position) {
      continue;
    }
    const auto pair = std::pair{std::min(footprint_index, partner_index),
                                std::max(footprint_index, partner_index)};
    if (!proposed_pairs.insert(pair).second) {
      continue;
    }
    result.push_back(MoveProposal{partner.position,
                                  footprint.rotation,
                                  footprint.side,
                                  PlacementMoveKind::swap_repair,
                                  partner_index,
                                  {},
                                  std::nullopt,
                                  {},
                                  {}});
  }
  return result;
}

[[nodiscard]] std::vector<MoveProposal>
group_proposals(const CandidateEvaluation& evaluation, std::size_t footprint_index,
                const std::vector<std::size_t>& ordered_footprints,
                const IntegratedOptimizerConfig& config) {
  std::vector<MoveProposal> result;
  if (!config.enable_group_moves) {
    return result;
  }
  const auto& board = evaluation.placement;
  const auto& primary = board.footprints[footprint_index];
  std::map<ObjectId, std::size_t> index_by_id;
  for (std::size_t index = 0U; index < board.footprints.size(); ++index) {
    index_by_id.emplace(board.footprints[index].id, index);
  }
  for (const auto& group : board.placement_groups) {
    std::vector<std::size_t> members;
    members.reserve(group.footprints.size());
    for (const auto& id : group.footprints) {
      const auto found = index_by_id.find(id);
      if (found == index_by_id.end()) {
        return {};
      }
      members.push_back(found->second);
    }
    const auto leader = std::find_if(
        ordered_footprints.begin(), ordered_footprints.end(), [&members](std::size_t candidate) {
          return std::find(members.begin(), members.end(), candidate) != members.end();
        });
    if (leader == ordered_footprints.end() || *leader != footprint_index) {
      continue;
    }
    if (std::any_of(members.begin(), members.end(),
                    [&board](std::size_t index) { return board.footprints[index].locked; })) {
      return result;
    }

    std::set<Point> displacements;
    const auto append = [&](Point displacement) {
      if (result.size() >= config.max_group_translation_proposals_per_group ||
          displacement == Point{} || !displacements.insert(displacement).second) {
        return;
      }
      const auto target = bounded_point(static_cast<long double>(primary.position.x.value()) +
                                            static_cast<long double>(displacement.x.value()),
                                        static_cast<long double>(primary.position.y.value()) +
                                            static_cast<long double>(displacement.y.value()));
      if (target) {
        MoveProposal proposal{*target,
                              primary.rotation,
                              primary.side,
                              PlacementMoveKind::group_repair,
                              std::nullopt,
                              members,
                              group.id,
                              {},
                              {}};
        auto candidate = board;
        if (apply_proposal(candidate, footprint_index, proposal) &&
            validate_board(candidate).ok()) {
          result.push_back(std::move(proposal));
        }
      }
    };
    if (config.enable_routing_guided_moves) {
      for (const auto member_index : ordered_footprints) {
        if (std::find(members.begin(), members.end(), member_index) == members.end()) {
          continue;
        }
        const auto& member = board.footprints[member_index];
        for (const auto target : routing_guided_positions(evaluation, member_index, config)) {
          append(target - member.position);
        }
      }
    }
    for (const auto displacement : move_offsets(config)) {
      append(displacement);
    }
    return result;
  }
  return result;
}

[[nodiscard]] std::vector<MoveProposal>
relative_component_proposals(const CandidateEvaluation& evaluation, std::size_t footprint_index,
                             const std::vector<std::size_t>& ordered_footprints,
                             const IntegratedOptimizerConfig& config) {
  std::vector<MoveProposal> result;
  if (!config.enable_relative_component_moves) {
    return result;
  }
  const auto& board = evaluation.placement;
  const auto& primary = board.footprints[footprint_index];
  const auto closure = placement_policy_closure(board, primary.id);
  if (closure.relative_constraints.empty() || closure.footprints.size() < 2U) {
    return result;
  }
  std::map<ObjectId, std::size_t> index_by_id;
  for (std::size_t index = 0U; index < board.footprints.size(); ++index) {
    index_by_id.emplace(board.footprints[index].id, index);
  }
  std::vector<std::size_t> members;
  members.reserve(closure.footprints.size());
  for (const auto& id : closure.footprints) {
    const auto found = index_by_id.find(id);
    if (found == index_by_id.end()) {
      return {};
    }
    members.push_back(found->second);
  }
  const auto leader = std::find_if(
      ordered_footprints.begin(), ordered_footprints.end(), [&members](std::size_t candidate) {
        return std::find(members.begin(), members.end(), candidate) != members.end();
      });
  if (leader == ordered_footprints.end() || *leader != footprint_index ||
      std::any_of(members.begin(), members.end(),
                  [&board](std::size_t index) { return board.footprints[index].locked; })) {
    return result;
  }

  std::set<Point> displacements;
  const auto append = [&](Point displacement) {
    if (result.size() >= config.max_relative_component_translation_proposals ||
        displacement == Point{} || !displacements.insert(displacement).second) {
      return;
    }
    const auto target = bounded_point(static_cast<long double>(primary.position.x.value()) +
                                          static_cast<long double>(displacement.x.value()),
                                      static_cast<long double>(primary.position.y.value()) +
                                          static_cast<long double>(displacement.y.value()));
    if (!target) {
      return;
    }
    MoveProposal proposal{*target,
                          primary.rotation,
                          primary.side,
                          PlacementMoveKind::relative_component_repair,
                          std::nullopt,
                          members,
                          std::nullopt,
                          closure.relative_constraints,
                          closure.placement_groups};
    auto candidate = board;
    if (apply_proposal(candidate, footprint_index, proposal) && validate_board(candidate).ok()) {
      result.push_back(std::move(proposal));
    }
  };
  if (config.enable_routing_guided_moves) {
    for (const auto member_index : ordered_footprints) {
      if (std::find(members.begin(), members.end(), member_index) == members.end()) {
        continue;
      }
      const auto& member = board.footprints[member_index];
      for (const auto target : routing_guided_positions(evaluation, member_index, config)) {
        append(target - member.position);
      }
    }
  }
  for (const auto displacement : move_offsets(config)) {
    append(displacement);
  }
  return result;
}

[[nodiscard]] Board result_board(const CandidateEvaluation& evaluation) {
  return evaluation.detailed.candidate ? *evaluation.detailed.candidate : evaluation.placement;
}

[[nodiscard]] auto pareto_dimensions(const IntegratedScore& score) {
  return std::tie(score.width_reduction_cost, score.routed_length_nm, score.route_cost,
                  score.side_changes, score.rotation_change_microdegrees, score.displacement_nm);
}

[[nodiscard]] bool pareto_dominates(const IntegratedScore& lhs, const IntegratedScore& rhs) {
  const auto lhs_values = pareto_dimensions(lhs);
  const auto rhs_values = pareto_dimensions(rhs);
  const bool no_worse = lhs.width_reduction_cost <= rhs.width_reduction_cost &&
                        lhs.routed_length_nm <= rhs.routed_length_nm &&
                        lhs.route_cost <= rhs.route_cost && lhs.side_changes <= rhs.side_changes &&
                        lhs.rotation_change_microdegrees <= rhs.rotation_change_microdegrees &&
                        lhs.displacement_nm <= rhs.displacement_nm;
  return no_worse && lhs_values != rhs_values;
}

[[nodiscard]] bool fully_feasible(const CandidateEvaluation& evaluation) {
  const auto& score = evaluation.score;
  return evaluation.detailed.complete() && evaluation.detailed.candidate &&
         score.placement_violations == 0U && score.global_route_failures == 0U &&
         score.unrouted_nets == 0U && score.failed_pin_escapes == 0U &&
         score.best_hard_capacity_unrouted_nets == 0U && score.global_overflow == 0U &&
         score.copper_violations == 0U && score.open_connections == 0U;
}

void archive_full_checkpoint(IntegratedOptimizationResult& result,
                             const CandidateEvaluation& evaluation,
                             std::size_t full_route_evaluation, std::size_t maximum_size) {
  if (!fully_feasible(evaluation)) {
    return;
  }
  const auto rejected = std::any_of(result.pareto_archive.begin(), result.pareto_archive.end(),
                                    [&](const ParetoCheckpoint& checkpoint) {
                                      return pareto_dimensions(checkpoint.score) ==
                                                 pareto_dimensions(evaluation.score) ||
                                             pareto_dominates(checkpoint.score, evaluation.score);
                                    });
  if (rejected) {
    ++result.pareto_candidates_discarded;
    return;
  }
  const auto old_size = result.pareto_archive.size();
  std::erase_if(result.pareto_archive, [&](const ParetoCheckpoint& checkpoint) {
    return pareto_dominates(evaluation.score, checkpoint.score);
  });
  result.pareto_candidates_discarded += old_size - result.pareto_archive.size();
  result.pareto_archive.push_back(
      ParetoCheckpoint{result_board(evaluation), evaluation.score, full_route_evaluation});
  std::sort(result.pareto_archive.begin(), result.pareto_archive.end(),
            [](const ParetoCheckpoint& lhs, const ParetoCheckpoint& rhs) {
              if (lhs.score != rhs.score) {
                return lhs.score < rhs.score;
              }
              return lhs.full_route_evaluation < rhs.full_route_evaluation;
            });
  if (result.pareto_archive.size() > maximum_size) {
    result.pareto_candidates_discarded += result.pareto_archive.size() - maximum_size;
    result.pareto_archive.resize(maximum_size);
  }
}

void append_routing_context(PlacementMove& move, const CandidateEvaluation& evaluation) {
  const auto feedback = collect_integrated_routing_feedback(evaluation.global);
  move.routing_obstacles = feedback.routing_obstacles;
  move.affected_nets.insert(move.affected_nets.end(), feedback.affected_nets.begin(),
                            feedback.affected_nets.end());
  std::sort(move.affected_nets.begin(), move.affected_nets.end());
  move.affected_nets.erase(std::unique(move.affected_nets.begin(), move.affected_nets.end()),
                           move.affected_nets.end());
  move.routing_pressure = feedback.routing_pressure;
  move.routing_corridor_pressure = feedback.routing_corridor_pressure;
  move.routing_physical_pressure = feedback.routing_physical_pressure;
  move.neck_down_obstacles = feedback.neck_down_obstacles;
  move.physical_channel_obstacles = feedback.physical_channel_obstacles;
  for (const auto& diagnosis : evaluation.detailed.routability_diagnoses) {
    if (!diagnosis.locally_resolved) {
      move.affected_nets.insert(move.affected_nets.end(), diagnosis.resource.nets.begin(),
                                diagnosis.resource.nets.end());
    }
  }
  std::sort(move.affected_nets.begin(), move.affected_nets.end());
  move.affected_nets.erase(std::unique(move.affected_nets.begin(), move.affected_nets.end()),
                           move.affected_nets.end());
}

[[nodiscard]] const RoutingHardCapacityPass*
representative_hard_capacity_pass(const GlobalRouteResult& route) {
  if (route.hard_capacity_pass_summaries.empty()) {
    return nullptr;
  }
  return &*std::min_element(
      route.hard_capacity_pass_summaries.begin(), route.hard_capacity_pass_summaries.end(),
      [](const RoutingHardCapacityPass& lhs, const RoutingHardCapacityPass& rhs) {
        if (lhs.unrouted_nets.size() != rhs.unrouted_nets.size()) {
          return lhs.unrouted_nets.size() < rhs.unrouted_nets.size();
        }
        // Equal-deficit later passes include the most recently diversified order and blocker
        // history, so use their terminal evidence for the next placement neighborhood.
        return lhs.iteration > rhs.iteration;
      });
}

} // namespace

IntegratedRoutingFeedback collect_integrated_routing_feedback(const GlobalRouteResult& route) {
  IntegratedRoutingFeedback result;
  for (const auto& impact : route.obstacle_impacts) {
    result.routing_obstacles.push_back(impact.obstacle);
    result.affected_nets.push_back(impact.routed_net);
  }
  for (const auto& impact : route.neck_down_obstacle_impacts) {
    result.routing_obstacles.push_back(impact.obstacle);
    result.affected_nets.push_back(impact.routed_net);
    result.neck_down_obstacles.push_back(impact);
  }
  for (const auto& impact : route.physical_obstacle_impacts) {
    result.routing_obstacles.push_back(impact.obstacle);
    result.affected_nets.insert(result.affected_nets.end(), impact.routed_nets.begin(),
                                impact.routed_nets.end());
    result.physical_channel_obstacles.push_back(impact);
  }
  for (const auto& failure : route.pin_escape_failures) {
    result.routing_obstacles.push_back(failure.footprint);
    result.routing_obstacles.push_back(failure.pad);
    result.affected_nets.push_back(failure.net);
  }
  for (const auto& failure : route.connection_failures) {
    result.affected_nets.push_back(failure.net);
    for (const auto& terminal : failure.attempted_terminals) {
      result.routing_obstacles.push_back(terminal.footprint);
      result.routing_obstacles.push_back(terminal.pad);
    }
  }
  // A feasibility-first result can legitimately select an overflowed soft checkpoint while
  // retaining later constructive hard-capacity attempts as separate evidence. Placement repair
  // must still see the nets and exact terminals that prevented a strict allocation.
  for (const auto& failure : route.hard_capacity_net_failures) {
    result.affected_nets.push_back(failure.net);
  }
  for (const auto& blocker : route.hard_capacity_blockers) {
    result.affected_nets.push_back(blocker.failed_net);
    result.affected_nets.push_back(blocker.blocking_net);
  }
  if (const auto* hard_pass = representative_hard_capacity_pass(route)) {
    result.affected_nets.insert(result.affected_nets.end(), hard_pass->unrouted_nets.begin(),
                                hard_pass->unrouted_nets.end());
    for (const auto& failure : hard_pass->connection_failures) {
      result.affected_nets.push_back(failure.net);
      for (const auto& terminal : failure.attempted_terminals) {
        result.routing_obstacles.push_back(terminal.footprint);
        result.routing_obstacles.push_back(terminal.pad);
      }
    }
  }
  result.affected_nets.insert(result.affected_nets.end(), route.unrouted_nets.begin(),
                              route.unrouted_nets.end());
  for (const auto& usage : route.resource_usage) {
    if (!usage.congested()) {
      continue;
    }
    result.routing_pressure.push_back(usage);
    result.affected_nets.insert(result.affected_nets.end(), usage.nets.begin(), usage.nets.end());
  }
  const auto append_corridors = [&](const std::vector<RoutingCorridorUsage>& usages) {
    for (const auto& usage : usages) {
      if (!usage.congested()) {
        continue;
      }
      result.routing_corridor_pressure.push_back(usage);
      result.affected_nets.insert(result.affected_nets.end(), usage.nets.begin(), usage.nets.end());
    }
  };
  append_corridors(route.corridor_usage);
  append_corridors(route.corridor_neck_down_usage);
  for (const auto& usage : route.voronoi_resource_usage) {
    if (!usage.congested()) {
      continue;
    }
    result.routing_physical_pressure.push_back(usage);
    result.affected_nets.insert(result.affected_nets.end(), usage.nets.begin(), usage.nets.end());
  }
  for (const auto& path : route.paths) {
    if (std::any_of(path.segment_widths.begin(), path.segment_widths.end(),
                    [](Nanometres width) { return width.value() > 0; })) {
      result.affected_nets.push_back(path.net);
    }
  }
  std::sort(result.routing_obstacles.begin(), result.routing_obstacles.end());
  result.routing_obstacles.erase(
      std::unique(result.routing_obstacles.begin(), result.routing_obstacles.end()),
      result.routing_obstacles.end());
  std::sort(result.affected_nets.begin(), result.affected_nets.end());
  result.affected_nets.erase(std::unique(result.affected_nets.begin(), result.affected_nets.end()),
                             result.affected_nets.end());
  return result;
}

std::vector<IntegratedFootprintPressure>
collect_integrated_footprint_pressure(const Board& board, const GlobalRouteResult& route,
                                      GlobalRouteCheckpointObjective objective) {
  std::set<ObjectId> blocking_objects;
  std::set<ObjectId> channel_blocking_objects;
  std::set<NetId> affected_nets;
  std::map<NetId, std::size_t> legacy_pressure_by_net;
  std::map<NetId, std::size_t> sampled_pressure_by_net;
  std::map<NetId, std::size_t> corridor_pressure_by_net;
  std::map<NetId, std::uint64_t> physical_pressure_by_net;
  const bool dimensioned =
      objective == GlobalRouteCheckpointObjective::dimensioned_physical_overflow;

  for (const auto& impact : route.obstacle_impacts) {
    blocking_objects.insert(impact.obstacle);
    affected_nets.insert(impact.routed_net);
  }
  for (const auto& impact : route.neck_down_obstacle_impacts) {
    blocking_objects.insert(impact.obstacle);
    affected_nets.insert(impact.routed_net);
  }
  for (const auto& impact : route.physical_obstacle_impacts) {
    blocking_objects.insert(impact.obstacle);
    channel_blocking_objects.insert(impact.obstacle);
    affected_nets.insert(impact.routed_nets.begin(), impact.routed_nets.end());
  }
  for (const auto& failure : route.pin_escape_failures) {
    blocking_objects.insert(failure.footprint);
    blocking_objects.insert(failure.pad);
    affected_nets.insert(failure.net);
  }
  for (const auto& failure : route.connection_failures) {
    affected_nets.insert(failure.net);
    for (const auto& terminal : failure.attempted_terminals) {
      blocking_objects.insert(terminal.footprint);
      blocking_objects.insert(terminal.pad);
    }
  }
  for (const auto& failure : route.hard_capacity_net_failures) {
    affected_nets.insert(failure.net);
    legacy_pressure_by_net[failure.net] =
        saturating_add_size(legacy_pressure_by_net[failure.net], failure.failed_passes);
    if (dimensioned) {
      sampled_pressure_by_net[failure.net] =
          saturating_add_size(sampled_pressure_by_net[failure.net], failure.failed_passes);
    }
  }
  for (const auto& blocker : route.hard_capacity_blockers) {
    affected_nets.insert(blocker.failed_net);
    affected_nets.insert(blocker.blocking_net);
    for (const auto& net : {blocker.failed_net, blocker.blocking_net}) {
      legacy_pressure_by_net[net] =
          saturating_add_size(legacy_pressure_by_net[net], blocker.rejected_transitions);
      if (dimensioned) {
        sampled_pressure_by_net[net] =
            saturating_add_size(sampled_pressure_by_net[net], blocker.rejected_transitions);
      }
    }
  }
  if (const auto* hard_pass = representative_hard_capacity_pass(route)) {
    affected_nets.insert(hard_pass->unrouted_nets.begin(), hard_pass->unrouted_nets.end());
    for (const auto& failure : hard_pass->connection_failures) {
      affected_nets.insert(failure.net);
      for (const auto& terminal : failure.attempted_terminals) {
        blocking_objects.insert(terminal.footprint);
        blocking_objects.insert(terminal.pad);
      }
    }
  }
  affected_nets.insert(route.unrouted_nets.begin(), route.unrouted_nets.end());
  for (const auto& usage : route.resource_usage) {
    if (!usage.congested()) {
      continue;
    }
    affected_nets.insert(usage.nets.begin(), usage.nets.end());
    const auto overflow = usage.demand - usage.capacity;
    for (const auto& net : usage.nets) {
      auto& pressure = legacy_pressure_by_net[net];
      pressure = saturating_add_size(pressure, overflow);
      if (dimensioned) {
        auto& sampled = sampled_pressure_by_net[net];
        sampled = saturating_add_size(sampled, overflow);
      }
    }
  }
  if (dimensioned) {
    const auto collect_corridors = [&](const std::vector<RoutingCorridorUsage>& usages) {
      for (const auto& usage : usages) {
        if (!usage.congested()) {
          continue;
        }
        affected_nets.insert(usage.nets.begin(), usage.nets.end());
        const auto overflow = usage.demand - usage.capacity;
        for (const auto& net : usage.nets) {
          auto& pressure = corridor_pressure_by_net[net];
          pressure = saturating_add_size(pressure, overflow);
        }
      }
    };
    collect_corridors(route.corridor_usage);
    collect_corridors(route.corridor_neck_down_usage);
    for (const auto& usage : route.voronoi_resource_usage) {
      if (!usage.congested()) {
        continue;
      }
      affected_nets.insert(usage.nets.begin(), usage.nets.end());
      const auto overflow = usage.demand_nm - usage.capacity_nm;
      for (const auto& net : usage.nets) {
        auto& pressure = physical_pressure_by_net[net];
        pressure = saturating_add(pressure, overflow);
      }
    }
  }
  for (const auto& path : route.paths) {
    if (!std::any_of(path.segment_widths.begin(), path.segment_widths.end(),
                     [](Nanometres width) { return width.value() > 0; })) {
      continue;
    }
    affected_nets.insert(path.net);
    auto& pressure = legacy_pressure_by_net[path.net];
    pressure = saturating_add_size(pressure, 1U);
  }

  std::vector<IntegratedFootprintPressure> result;
  result.reserve(board.footprints.size());
  for (const auto& footprint : board.footprints) {
    IntegratedFootprintPressure item;
    item.footprint = footprint.id;
    item.bounds_congested_channel = channel_blocking_objects.contains(footprint.id);
    item.blocks_route = blocking_objects.contains(footprint.id);
    std::set<NetId> distinct_nets;
    for (const auto& pad : footprint.pads) {
      item.blocks_route = item.blocks_route || blocking_objects.contains(pad.id);
      item.bounds_congested_channel =
          item.bounds_congested_channel || channel_blocking_objects.contains(pad.id);
      if (!pad.net.value().empty()) {
        distinct_nets.insert(pad.net);
      }
    }
    item.distinct_connected_nets = distinct_nets.size();
    for (const auto& net : distinct_nets) {
      item.terminates_affected_net = item.terminates_affected_net || affected_nets.contains(net);
      if (const auto found = legacy_pressure_by_net.find(net);
          found != legacy_pressure_by_net.end()) {
        item.legacy_pressure = saturating_add_size(item.legacy_pressure, found->second);
      }
      if (const auto found = physical_pressure_by_net.find(net);
          found != physical_pressure_by_net.end()) {
        item.physical_pressure_nm = saturating_add(item.physical_pressure_nm, found->second);
      }
      if (const auto found = sampled_pressure_by_net.find(net);
          found != sampled_pressure_by_net.end()) {
        item.sampled_pressure = saturating_add_size(item.sampled_pressure, found->second);
      }
      if (const auto found = corridor_pressure_by_net.find(net);
          found != corridor_pressure_by_net.end()) {
        item.corridor_pressure = saturating_add_size(item.corridor_pressure, found->second);
      }
    }
    for (const auto& keepout : footprint.keepouts) {
      item.blocks_route = item.blocks_route || blocking_objects.contains(keepout.id);
      item.bounds_congested_channel =
          item.bounds_congested_channel || channel_blocking_objects.contains(keepout.id);
    }
    result.push_back(std::move(item));
  }
  const auto category = [](const IntegratedFootprintPressure& item) {
    return item.bounds_congested_channel
               ? 0U
               : (item.blocks_route ? 1U : (item.terminates_affected_net ? 2U : 3U));
  };
  std::sort(result.begin(), result.end(), [&](const auto& lhs, const auto& rhs) {
    if (category(lhs) != category(rhs)) {
      return category(lhs) < category(rhs);
    }
    if (dimensioned) {
      const auto lhs_pressure = std::tie(lhs.physical_pressure_nm, lhs.sampled_pressure,
                                         lhs.corridor_pressure, lhs.legacy_pressure);
      const auto rhs_pressure = std::tie(rhs.physical_pressure_nm, rhs.sampled_pressure,
                                         rhs.corridor_pressure, rhs.legacy_pressure);
      if (lhs_pressure != rhs_pressure) {
        return lhs_pressure > rhs_pressure;
      }
    } else if (lhs.legacy_pressure != rhs.legacy_pressure) {
      return lhs.legacy_pressure > rhs.legacy_pressure;
    }
    return lhs.footprint < rhs.footprint;
  });
  return result;
}

std::strong_ordering IntegratedScore::operator<=>(const IntegratedScore& rhs) const {
  const auto prefix =
      std::tie(placement_violations, global_route_failures, unrouted_nets, failed_pin_escapes,
               best_hard_capacity_unrouted_nets, configured_dimensioned_physical_overflow);
  const auto rhs_prefix =
      std::tie(rhs.placement_violations, rhs.global_route_failures, rhs.unrouted_nets,
               rhs.failed_pin_escapes, rhs.best_hard_capacity_unrouted_nets,
               rhs.configured_dimensioned_physical_overflow);
  if (const auto comparison = prefix <=> rhs_prefix; comparison != 0) {
    return comparison;
  }
  if (configured_dimensioned_physical_overflow) {
    const auto physical =
        std::tie(total_physical_overflow_nm, peak_physical_overflow_nm, sampled_overflow,
                 corridor_overflow, congested_physical_resources);
    const auto rhs_physical =
        std::tie(rhs.total_physical_overflow_nm, rhs.peak_physical_overflow_nm,
                 rhs.sampled_overflow, rhs.corridor_overflow, rhs.congested_physical_resources);
    if (const auto comparison = physical <=> rhs_physical; comparison != 0) {
      return comparison;
    }
  } else if (const auto comparison = global_overflow <=> rhs.global_overflow; comparison != 0) {
    return comparison;
  }
  return std::tie(copper_violations, open_connections, width_reduction_cost, routed_length_nm,
                  route_cost, side_changes, rotation_change_microdegrees, displacement_nm) <=>
         std::tie(rhs.copper_violations, rhs.open_connections, rhs.width_reduction_cost,
                  rhs.routed_length_nm, rhs.route_cost, rhs.side_changes,
                  rhs.rotation_change_microdegrees, rhs.displacement_nm);
}

bool IntegratedScore::operator==(const IntegratedScore& rhs) const { return (*this <=> rhs) == 0; }

IntegratedOptimizationResult
optimize_integrated_placement_routing(const Board& source, const IntegratedOptimizerConfig& config,
                                      std::stop_token stop_token, RouterTelemetrySink* telemetry) {
  IntegratedOptimizationResult result;
  result.score.configured_dimensioned_physical_overflow =
      config.global_router.checkpoint_objective ==
      GlobalRouteCheckpointObjective::dimensioned_physical_overflow;
  result.global_route.configured_checkpoint_objective = config.global_router.checkpoint_objective;
  result.detailed_route.materialized_config = config.detailed_router;
  result.configured_max_passes = config.max_passes;
  result.configured_max_candidate_evaluations = config.max_candidate_evaluations;
  result.configured_max_candidate_evaluation_workers = config.max_candidate_evaluation_workers;
  result.configured_max_full_routing_candidates_per_wave =
      config.max_full_routing_candidates_per_wave;
  result.configured_candidate_evaluation_offset = config.candidate_evaluation_offset;
  result.configured_move_step = config.move_step;
  result.tentative_placement_envelopes = static_cast<std::size_t>(std::count_if(
      source.footprints.begin(), source.footprints.end(),
      [](const Footprint& footprint) { return footprint.has_tentative_placement_envelope(); }));
  result.suggested_placement_move_priorities = config.suggested_placement_move_priorities;
  result.placement_move_priority_review_required = config.suggested_placement_move_priorities;
  result.human_placement_review_required =
      result.tentative_placement_envelopes > 0U || result.placement_move_priority_review_required;
  if (config.move_step.value() <= 0 || config.move_step.value() > max_abs_coordinate_nm ||
      config.routing_feedback_margin.value() < 0 ||
      config.routing_feedback_margin.value() > max_abs_coordinate_nm || config.max_passes == 0U ||
      config.max_candidate_evaluations == 0U || config.max_candidate_evaluation_workers == 0U ||
      config.max_full_routing_candidates_per_wave == 0U || config.max_pareto_checkpoints == 0U) {
    result.diagnostic = "integrated optimizer configuration is invalid";
    return result;
  }
  if (config.enable_routing_guided_moves && config.max_routing_guided_targets_per_footprint == 0U) {
    result.diagnostic = "routing-guided movement requires a positive target bound";
    return result;
  }
  if (config.enable_swap_moves && config.max_swap_partners_per_footprint == 0U) {
    result.diagnostic = "footprint swap search requires a positive partner bound";
    return result;
  }
  if (config.enable_group_moves && config.max_group_translation_proposals_per_group == 0U) {
    result.diagnostic = "placement-group search requires a positive proposal bound";
    return result;
  }
  if (config.enable_group_moves && source.placement_groups.empty()) {
    result.diagnostic = "placement-group search requires at least one declared group";
    return result;
  }
  if (config.enable_relative_component_moves &&
      config.max_relative_component_translation_proposals == 0U) {
    result.diagnostic = "relative-component search requires a positive proposal bound";
    return result;
  }
  if (config.enable_relative_component_moves && source.relative_placement_constraints.empty()) {
    result.diagnostic = "relative-component search requires explicit relative-placement policy";
    return result;
  }
  if (config.enable_global_routing_guidance && !config.enable_global_placement) {
    result.diagnostic = "global routing guidance requires global placement";
    return result;
  }
  if (config.global_placer.enable_assignment_legalization && !config.enable_global_placement) {
    result.diagnostic = "assignment legalization requires global placement";
    return result;
  }
  if (config.global_placer.enable_assignment_legalization &&
      (config.global_placer.max_assignment_components == 0U ||
       config.global_placer.max_assignment_candidates_per_component == 0U ||
       config.global_placer.max_assignment_search_states == 0U)) {
    result.diagnostic =
        "assignment legalization requires positive component, candidate, and search-state bounds";
    return result;
  }
  if (config.enable_global_routing_guidance &&
      (config.max_global_routing_guides == 0U ||
       config.max_routing_guided_targets_per_footprint == 0U ||
       !std::isfinite(config.global_placer.routing_guidance_weight) ||
       config.global_placer.routing_guidance_weight <= 0.0L)) {
    result.diagnostic = "global routing guidance requires positive finite bounds and weight";
    return result;
  }
  if ((!config.rotation_offsets.empty() || config.enable_side_flips) &&
      config.max_orientation_proposals_per_footprint == 0U) {
    result.diagnostic = "orientation search requires a positive proposal bound";
    return result;
  }
  if (config.enable_incremental_routing && config.full_routing_verification_interval == 0U) {
    result.diagnostic = "incremental routing requires a positive full-verification interval";
    return result;
  }
  if (config.detailed_router.differential_pair_skew_tuning_batch_assignment &&
      !config.detailed_router.differential_pair_skew_tuning) {
    result.diagnostic = "skew-tuning batch assignment requires skew tuning to be enabled";
    return result;
  }
  if (config.detailed_router.differential_pair_skew_tuning &&
      (config.detailed_router.differential_pair_skew_tuning_step.value() <= 0 ||
       config.detailed_router.differential_pair_skew_tuning_step.value() > max_abs_coordinate_nm ||
       config.detailed_router.maximum_differential_pair_skew_tuning_amplitude.value() <= 0 ||
       config.detailed_router.minimum_differential_pair_skew_tuning_lead.value() <= 0 ||
       config.detailed_router.minimum_differential_pair_skew_tuning_spacing.value() <= 0 ||
       config.detailed_router.maximum_differential_pair_skew_tuning_amplitude.value() >
           max_abs_coordinate_nm ||
       config.detailed_router.minimum_differential_pair_skew_tuning_lead.value() >
           max_abs_coordinate_nm ||
       config.detailed_router.minimum_differential_pair_skew_tuning_spacing.value() >
           max_abs_coordinate_nm ||
       config.detailed_router.max_differential_pair_skew_tuning_candidates == 0U ||
       config.detailed_router.max_differential_pair_skew_tuning_periods == 0U ||
       (config.detailed_router.differential_pair_skew_tuning_batch_assignment &&
        (config.detailed_router.max_differential_pair_skew_tuning_assignment_candidates_per_pair ==
             0U ||
         config.detailed_router.max_differential_pair_skew_tuning_assignment_states == 0U)))) {
    result.diagnostic =
        "differential-pair skew tuning requires positive bounded geometry and candidates";
    return result;
  }
  if (std::any_of(source.footprints.begin(), source.footprints.end(),
                  [&config](const Footprint& footprint) {
                    return !footprint.has_exact_placement_envelope() &&
                           !(config.allow_tentative_placement_envelopes &&
                             footprint.has_tentative_placement_envelope());
                  })) {
    result.diagnostic =
        "integrated placement/routing requires an authoritative envelope or explicit tentative "
        "envelope opt-in for every footprint";
    return result;
  }
  std::set<ObjectId> orientation_scope;
  for (const auto& id : config.orientation_scope) {
    const auto known =
        std::any_of(source.footprints.begin(), source.footprints.end(),
                    [&id](const Footprint& footprint) { return footprint.id == id; });
    if (id.value().empty() || !orientation_scope.insert(id).second || !known) {
      result.diagnostic = "orientation scope contains an empty, duplicate, or unknown footprint";
      return result;
    }
  }
  const auto priorities =
      make_placement_priority_context(source, config.placement_move_priorities, result.diagnostic);
  if (!priorities) {
    return result;
  }
  result.configured_placement_move_priorities = priorities->enabled;
  if (config.suggested_placement_move_priorities && !priorities->enabled) {
    result.diagnostic = "suggested placement-move priorities require a complete generated policy";
    return result;
  }
  if (priorities->enabled) {
    result.placement_priority_metrics.reserve(priorities->order.size());
    for (const auto priority_class : priorities->order) {
      const auto count = static_cast<std::size_t>(std::count_if(
          priorities->class_by_footprint.begin(), priorities->class_by_footprint.end(),
          [priority_class](const auto& assignment) {
            return assignment.second == priority_class;
          }));
      result.placement_priority_metrics.push_back(
          PlacementMovePriorityMetrics{priority_class, count, 0U, 0U, 0U});
    }
  }
  if (stop_token.stop_requested()) {
    result.stop_reason = IntegratedOptimizerStopReason::cancelled;
    result.diagnostic = "integrated optimization cancelled before evaluation";
    return result;
  }

  const auto validation_baseline = make_board_validation_baseline(source);
  auto current =
      evaluate_candidate(source, source, config, validation_baseline, stop_token, telemetry);
  result.candidate_evaluations = 1;
  result.board_candidate_validations = 1;
  result.reused_static_zone_validations = current.reused_static_zone_validation ? 1U : 0U;
  result.full_route_evaluations = 1;
  archive_full_checkpoint(result, current, result.full_route_evaluations,
                          config.max_pareto_checkpoints);
  if (stop_token.stop_requested()) {
    result.stop_reason = IntegratedOptimizerStopReason::cancelled;
  }
  if (config.enable_global_placement && !stop_token.stop_requested()) {
    for (std::size_t active_tier = 0U;
         active_tier < priorities->tier_count() && !stop_token.stop_requested(); ++active_tier) {
      if (priorities->enabled) {
        result.placement_priority_tiers_unlocked =
            std::max(result.placement_priority_tiers_unlocked, active_tier + 1U);
      }
      auto scoped_source = priority_scoped_board(source, *priorities, active_tier);
      std::vector<GlobalPlacementGuide> global_guidance;
      if (config.enable_global_routing_guidance) {
        for (const auto footprint_index : feedback_ordered_footprints(current, config)) {
          if (global_guidance.size() >= config.max_global_routing_guides) {
            break;
          }
          const auto& footprint = current.placement.footprints[footprint_index];
          if (!priorities->allows(footprint.id, active_tier)) {
            continue;
          }
          const auto closure = placement_policy_closure(scoped_source, footprint.id);
          const bool immobilized = std::any_of(
              closure.footprints.begin(), closure.footprints.end(), [&](const ObjectId& id) {
                const auto* member = find_footprint(scoped_source, id);
                return member == nullptr || member->locked;
              });
          if (immobilized) {
            continue;
          }
          for (const auto target : routing_guided_positions(current, footprint_index, config)) {
            if (global_guidance.size() >= config.max_global_routing_guides) {
              break;
            }
            global_guidance.push_back(GlobalPlacementGuide{footprint.id, target, 1U});
          }
        }
      }
      auto global_placer_config = config.global_placer;
      global_placer_config.allow_tentative_placement_envelopes =
          config.allow_tentative_placement_envelopes;
      auto attempted =
          place_global(scoped_source, global_placer_config, stop_token, global_guidance);
      if (attempted.candidate) {
        restore_source_locks(*attempted.candidate, source);
      }
      result.global_placement = std::move(attempted);
      if (priorities->enabled) {
        ++result.placement_priority_metrics[active_tier].proposals;
      }
      if (stop_token.stop_requested()) {
        result.stop_reason = IntegratedOptimizerStopReason::cancelled;
        break;
      }
      if (!result.global_placement->complete() || !result.global_placement->candidate ||
          !footprint_transforms_changed(source, *result.global_placement->candidate)) {
        if (priorities->enabled && active_tier + 1U < priorities->tier_count()) {
          ++result.placement_priority_escalations;
        }
        continue;
      }
      if (result.candidate_evaluations >= config.max_candidate_evaluations) {
        break;
      }
      auto seeded = evaluate_candidate(source, *result.global_placement->candidate, config,
                                       validation_baseline, stop_token, telemetry);
      ++result.candidate_evaluations;
      ++result.board_candidate_validations;
      result.reused_static_zone_validations += seeded.reused_static_zone_validation ? 1U : 0U;
      ++result.full_route_evaluations;
      if (priorities->enabled) {
        ++result.placement_priority_metrics[active_tier].candidate_evaluations;
      }
      archive_full_checkpoint(result, seeded, result.full_route_evaluations,
                              config.max_pareto_checkpoints);
      if (!stop_token.stop_requested() && seeded.score < current.score) {
        std::map<ObjectId, std::size_t> source_index_by_id;
        for (std::size_t index = 0U; index < source.footprints.size(); ++index) {
          source_index_by_id.emplace(source.footprints[index].id, index);
        }
        std::set<ObjectId> recorded_footprints;
        for (std::size_t index = 0; index < source.footprints.size(); ++index) {
          if (source.footprints[index].position == seeded.placement.footprints[index].position ||
              recorded_footprints.contains(source.footprints[index].id)) {
            continue;
          }
          const auto closure = placement_policy_closure(source, source.footprints[index].id);
          PlacementMove move;
          move.footprint = source.footprints[index].id;
          move.from = source.footprints[index].position;
          move.to = seeded.placement.footprints[index].position;
          move.from_rotation = source.footprints[index].rotation;
          move.to_rotation = seeded.placement.footprints[index].rotation;
          move.from_side = source.footprints[index].side;
          move.to_side = seeded.placement.footprints[index].side;
          move.before = current.score;
          move.after = seeded.score;
          move.kind = PlacementMoveKind::global_seed;
          move.relative_constraints = closure.relative_constraints;
          move.constraint_groups = closure.placement_groups;
          std::size_t required_tier = 0U;
          if (priorities->enabled) {
            for (const auto& member_id : closure.footprints) {
              const auto assigned = priorities->class_by_footprint.at(member_id);
              required_tier = std::max(required_tier, priorities->tier_by_class.at(assigned));
            }
            move.priority_class = priorities->order[required_tier];
          }
          if (closure.placement_groups.size() == 1U) {
            move.placement_group = closure.placement_groups.front();
          }
          for (const auto& member_id : closure.footprints) {
            recorded_footprints.insert(member_id);
            const auto member_index = source_index_by_id.at(member_id);
            const auto& member = source.footprints[member_index];
            const auto& placed_member = seeded.placement.footprints[member_index];
            for (const auto& pad : member.pads) {
              move.affected_nets.push_back(pad.net);
            }
            if (member_index != index && member.position != placed_member.position) {
              move.companion_moves.push_back(
                  PlacementMoveMember{member.id, member.position, placed_member.position});
            }
          }
          append_routing_context(move, current);
          result.accepted_moves.push_back(std::move(move));
          if (priorities->enabled) {
            ++result.placement_priority_metrics[required_tier].accepted_moves;
          }
        }
        current = std::move(seeded);
        result.global_seed_accepted = true;
        break;
      }
      if (priorities->enabled && active_tier + 1U < priorities->tier_count()) {
        ++result.placement_priority_escalations;
      }
    }
  }
  struct CandidateWaveItem {
    std::size_t evaluation_budget_index{0};
    std::size_t footprint_index{0};
    std::size_t required_tier{0};
    MoveProposal proposal;
    std::vector<NetId> affected_nets;
    bool periodic_full_verification{false};
  };
  struct CandidateWaveResult {
    std::optional<CandidateEvaluation> evaluation;
    std::exception_ptr failure;
  };
  const auto add_count = [](std::size_t& target, std::size_t value) {
    target = value > std::numeric_limits<std::size_t>::max() - target
                 ? std::numeric_limits<std::size_t>::max()
                 : target + value;
  };
  const auto account_incremental_evaluation = [&](const CandidateEvaluation& evaluated) {
    ++result.board_candidate_validations;
    add_count(result.reused_static_zone_validations,
              evaluated.reused_static_zone_validation ? 1U : 0U);
    ++result.incremental_route_evaluations;
    add_count(result.checkpoint_paths_reused, evaluated.global.reused_paths);
    if (evaluated.global.complete()) {
      ++result.incremental_detailed_evaluations;
      add_count(result.checkpoint_tracks_reused, evaluated.detailed.reused_tracks);
      add_count(result.checkpoint_vias_reused, evaluated.detailed.reused_vias);
      add_count(result.detailed_reuse_fallbacks,
                evaluated.detailed.fallback_full_materialization ? 1U : 0U);
    }
  };

  auto remaining_candidate_offset = config.candidate_evaluation_offset;
  for (std::size_t pass = 0; pass < config.max_passes && !stop_token.stop_requested(); ++pass) {
    if (current.detailed.complete() && !config.optimize_routed_quality) {
      result.stop_reason = IntegratedOptimizerStopReason::complete;
      break;
    }
    auto best = current;
    std::optional<PlacementMove> best_move;
    bool budget_exhausted = false;
    const auto ordered_footprints = feedback_ordered_footprints(current, config);
    for (std::size_t active_tier = 0U;
         active_tier < priorities->tier_count() && !stop_token.stop_requested(); ++active_tier) {
      if (priorities->enabled) {
        result.placement_priority_tiers_unlocked =
            std::max(result.placement_priority_tiers_unlocked, active_tier + 1U);
      }
      std::set<std::pair<std::size_t, std::size_t>> proposed_swap_pairs;
      // Each tier/obstacle phase is one immutable-checkpoint wave. Proposal order and evaluation
      // budget indices are fixed before any worker starts.
      for (const bool physical_obstacle_phase : {true, false}) {
        std::vector<CandidateWaveItem> wave;
        for (const auto footprint_index : ordered_footprints) {
          const auto& footprint = current.placement.footprints[footprint_index];
          if (footprint.locked || !priorities->allows(footprint.id, active_tier)) {
            continue;
          }
          std::vector<MoveProposal> proposals;
          if (physical_obstacle_phase) {
            if (!footprint_is_grouped(current.placement, footprint.id)) {
              proposals = move_proposals(current, footprint_index, config);
            }
          } else {
            proposals =
                relative_component_proposals(current, footprint_index, ordered_footprints, config);
            auto groups = group_proposals(current, footprint_index, ordered_footprints, config);
            proposals.insert(proposals.end(), groups.begin(), groups.end());
            if (!footprint_is_grouped(current.placement, footprint.id)) {
              auto swaps = swap_proposals(current, footprint_index, ordered_footprints, config,
                                          proposed_swap_pairs);
              proposals.insert(proposals.end(), swaps.begin(), swaps.end());
              auto single_moves = move_proposals(current, footprint_index, config);
              proposals.insert(proposals.end(), single_moves.begin(), single_moves.end());
            }
          }
          for (const auto& proposal : proposals) {
            const bool physical_obstacle =
                proposal.kind == PlacementMoveKind::physical_channel_obstacle_repair;
            if (physical_obstacle != physical_obstacle_phase) {
              continue;
            }
            const auto required_tier =
                proposal_priority_tier(current, footprint_index, proposal, *priorities);
            if (!required_tier || *required_tier != active_tier) {
              continue;
            }
            if (proposal.kind == PlacementMoveKind::neck_down_obstacle_repair) {
              ++result.neck_down_obstacle_move_proposals;
            }
            if (proposal.kind == PlacementMoveKind::physical_channel_obstacle_repair) {
              ++result.physical_channel_obstacle_move_proposals;
            }
            if (priorities->enabled) {
              ++result.placement_priority_metrics[*required_tier].proposals;
            }
            auto structural_probe = current.placement;
            if (!apply_proposal(structural_probe, footprint_index, proposal)) {
              continue;
            }
            if (remaining_candidate_offset > 0U) {
              --remaining_candidate_offset;
              ++result.candidate_evaluations_skipped;
              continue;
            }
            if (result.candidate_evaluations + wave.size() >= config.max_candidate_evaluations) {
              budget_exhausted = true;
              break;
            }
            if (proposal.kind == PlacementMoveKind::neck_down_obstacle_repair) {
              ++result.neck_down_obstacle_candidate_evaluations;
            }
            if (proposal.kind == PlacementMoveKind::physical_channel_obstacle_repair) {
              ++result.physical_channel_obstacle_candidate_evaluations;
            }
            if (priorities->enabled) {
              ++result.placement_priority_metrics[*required_tier].candidate_evaluations;
            }
            const auto incremental_ordinal =
                result.incremental_route_evaluations + wave.size() + 1U;
            wave.push_back(CandidateWaveItem{
                result.candidate_evaluations + wave.size(), footprint_index, *required_tier,
                proposal, affected_nets_for_proposal(current, footprint_index, proposal),
                config.enable_incremental_routing &&
                    incremental_ordinal % config.full_routing_verification_interval == 0U});
          }
          if (budget_exhausted) {
            break;
          }
        }

        if (!wave.empty()) {
          ++result.candidate_evaluation_waves;
          add_count(result.candidate_evaluation_tasks_dispatched, wave.size());
          const auto worker_count = std::min(config.max_candidate_evaluation_workers, wave.size());
          result.peak_candidate_evaluation_workers =
              std::max(result.peak_candidate_evaluation_workers, worker_count);
          std::vector<std::optional<IntegratedScore>> screened_scores(wave.size());
          std::vector<std::size_t> shortlist;
          // Evaluate the immutable wave in cheap physical-feasibility order while retaining its
          // original budget indices and reduction slots. This changes neither the comparator nor
          // the accepted result; it only presents the most local, least disruptive candidates to
          // the bounded worker pool first and is useful when a stop request truncates a long wave.
          std::vector<RoutingComputeLinearCandidate> placement_probe_candidates;
          placement_probe_candidates.reserve(wave.size());
          for (const auto& item : wave) {
            const auto& footprint = current.placement.footprints[item.footprint_index];
            const auto absolute_x = std::abs(item.proposal.position.x.value() -
                                             footprint.position.x.value());
            const auto absolute_y = std::abs(item.proposal.position.y.value() -
                                             footprint.position.y.value());
            const auto displacement_units =
                static_cast<std::uint64_t>(absolute_x / 100'000) +
                static_cast<std::uint64_t>(absolute_y / 100'000);
            const auto bounded = [](std::uint64_t value) {
              return value > std::numeric_limits<std::uint32_t>::max()
                         ? std::numeric_limits<std::uint32_t>::max()
                         : static_cast<std::uint32_t>(value);
            };
            RoutingComputeLinearCandidate candidate;
            candidate.terms = {
                bounded(displacement_units), bounded(item.affected_nets.size()),
                bounded(item.proposal.group_member_indices.size() +
                        (item.proposal.swap_partner_index ? 1U : 0U)),
                item.proposal.rotation != footprint.rotation ? 1U : 0U,
                item.proposal.side != footprint.side ? 1U : 0U,
                item.periodic_full_verification ? 1U : 0U,
                bounded(static_cast<std::uint64_t>(item.proposal.kind)), bounded(item.required_tier)};
            placement_probe_candidates.push_back(candidate);
          }
          const auto placement_probe_scores = score_routing_compute_candidates(
              placement_probe_candidates, {4U, 40U, 100U, 500U, 2'000U, 50U, 1U, 250U});
          std::vector<std::size_t> execution_order(wave.size());
          std::iota(execution_order.begin(), execution_order.end(), 0U);
          if (placement_probe_scores.ok() &&
              placement_probe_scores.scores.size() == execution_order.size()) {
            std::stable_sort(execution_order.begin(), execution_order.end(),
                             [&](std::size_t lhs, std::size_t rhs) {
                               return placement_probe_scores.scores[lhs] !=
                                              placement_probe_scores.scores[rhs]
                                          ? placement_probe_scores.scores[lhs] <
                                                placement_probe_scores.scores[rhs]
                                          : wave[lhs].evaluation_budget_index <
                                                wave[rhs].evaluation_budget_index;
                             });
          }
          const auto consider_full_candidate = [&](std::size_t index,
                                                   CandidateEvaluation candidate) {
            if (!(candidate.score < best.score)) {
              return;
            }
            const auto footprint_index = wave[index].footprint_index;
            const auto& proposal = wave[index].proposal;
            const auto& footprint = current.placement.footprints[footprint_index];
            const auto& placed = candidate.placement.footprints[footprint_index];
            PlacementMove move;
            move.footprint = footprint.id;
            move.from = footprint.position;
            move.to = placed.position;
            move.from_rotation = footprint.rotation;
            move.to_rotation = placed.rotation;
            move.from_side = footprint.side;
            move.to_side = placed.side;
            if (proposal.swap_partner_index) {
              const auto partner_index = *proposal.swap_partner_index;
              const auto& partner = current.placement.footprints[partner_index];
              const auto& placed_partner = candidate.placement.footprints[partner_index];
              move.partner_footprint = partner.id;
              move.partner_from = partner.position;
              move.partner_to = placed_partner.position;
            }
            move.placement_group = proposal.placement_group;
            move.relative_constraints = proposal.relative_constraints;
            move.constraint_groups = proposal.constraint_groups;
            for (const auto member_index : proposal.group_member_indices) {
              if (member_index == footprint_index) {
                continue;
              }
              const auto& member = current.placement.footprints[member_index];
              const auto& placed_member = candidate.placement.footprints[member_index];
              move.companion_moves.push_back(
                  PlacementMoveMember{member.id, member.position, placed_member.position});
            }
            move.before = current.score;
            move.after = candidate.score;
            move.kind = proposal.kind;
            if (priorities->enabled) {
              move.priority_class = priorities->order[wave[index].required_tier];
            }
            move.affected_nets = wave[index].affected_nets;
            append_routing_context(move, current);
            best_move = std::move(move);
            best = std::move(candidate);
          };

          // A whole deterministic wave is known up front, but only one bounded batch retains full
          // board/routing results at a time. Reduced score slots are small and preserve wave order.
          std::size_t completed_evaluations = 0U;
          for (std::size_t batch_start = 0U;
               batch_start < wave.size() && !stop_token.stop_requested();
               batch_start += worker_count) {
            const auto batch_size = std::min(worker_count, wave.size() - batch_start);
            std::vector<CandidateWaveResult> batch(batch_size);
            std::atomic_size_t next_candidate{0U};
            const auto run_candidate = [&]() {
              while (!stop_token.stop_requested()) {
                const auto local_index = next_candidate.fetch_add(1U, std::memory_order_relaxed);
                if (local_index >= batch_size) {
                  return;
                }
                const auto index = execution_order[batch_start + local_index];
                try {
                  auto candidate = current.placement;
                  if (!apply_proposal(candidate, wave[index].footprint_index,
                                      wave[index].proposal)) {
                    throw std::logic_error("generated placement proposal cannot be replayed");
                  }
                  auto worker_config = config;
                  // Avoid nested hardware-wide static-seed pools while several candidate routes
                  // are already resident. The cap is runtime-only and cannot change route geometry.
                  worker_config.global_router.max_static_seed_workers = 1U;
                  batch[local_index].evaluation =
                      config.enable_incremental_routing
                          ? evaluate_candidate(source, std::move(candidate), worker_config,
                                               validation_baseline, stop_token, telemetry,
                                               &current.global, wave[index].affected_nets,
                                               &current.detailed)
                          : evaluate_candidate(source, std::move(candidate), worker_config,
                                               validation_baseline, stop_token, telemetry);
                } catch (...) {
                  batch[local_index].failure = std::current_exception();
                }
              }
            };
            std::vector<std::jthread> workers;
            workers.reserve(batch_size);
            for (std::size_t index = 0U; index < batch_size; ++index) {
              workers.emplace_back(run_candidate);
            }
            workers.clear();

            for (std::size_t local_index = 0U; local_index < batch_size; ++local_index) {
              const auto index = execution_order[batch_start + local_index];
              if (batch[local_index].failure) {
                std::rethrow_exception(batch[local_index].failure);
              }
              if (!batch[local_index].evaluation) {
                continue;
              }
              auto evaluated = std::move(*batch[local_index].evaluation);
              ++completed_evaluations;
              screened_scores[index] = evaluated.score;
              if (config.enable_incremental_routing) {
                account_incremental_evaluation(evaluated);
                if (evaluated.score < current.score || wave[index].periodic_full_verification) {
                  shortlist.push_back(index);
                }
              } else {
                ++result.board_candidate_validations;
                add_count(result.reused_static_zone_validations,
                          evaluated.reused_static_zone_validation ? 1U : 0U);
                ++result.full_route_evaluations;
                archive_full_checkpoint(result, evaluated, result.full_route_evaluations,
                                        config.max_pareto_checkpoints);
                consider_full_candidate(index, std::move(evaluated));
              }
            }
          }
          add_count(result.candidate_evaluations, completed_evaluations);
          budget_exhausted =
              budget_exhausted || result.candidate_evaluations >= config.max_candidate_evaluations;

          if (config.enable_incremental_routing && !stop_token.stop_requested()) {
            std::sort(shortlist.begin(), shortlist.end(), [&](const auto lhs, const auto rhs) {
              return *screened_scores[lhs] != *screened_scores[rhs]
                         ? *screened_scores[lhs] < *screened_scores[rhs]
                         : wave[lhs].evaluation_budget_index < wave[rhs].evaluation_budget_index;
            });
            if (shortlist.size() > config.max_full_routing_candidates_per_wave) {
              shortlist.resize(config.max_full_routing_candidates_per_wave);
            }
            std::sort(shortlist.begin(), shortlist.end());
            for (const auto index : shortlist) {
              auto candidate = current.placement;
              if (!apply_proposal(candidate, wave[index].footprint_index, wave[index].proposal)) {
                throw std::logic_error("shortlisted placement proposal cannot be replayed");
              }
              auto fully_routed = evaluate_candidate(source, std::move(candidate), config,
                                                     validation_baseline, stop_token, telemetry);
              ++result.board_candidate_validations;
              add_count(result.reused_static_zone_validations,
                        fully_routed.reused_static_zone_validation ? 1U : 0U);
              ++result.full_route_evaluations;
              ++result.shortlisted_full_route_evaluations;
              archive_full_checkpoint(result, fully_routed, result.full_route_evaluations,
                                      config.max_pareto_checkpoints);
              consider_full_candidate(index, std::move(fully_routed));
            }
          }
        }
        if (best_move || budget_exhausted || stop_token.stop_requested()) {
          break;
        }
      }
      if (best_move || budget_exhausted || stop_token.stop_requested()) {
        break;
      }
      if (priorities->enabled && active_tier + 1U < priorities->tier_count()) {
        ++result.placement_priority_escalations;
      }
    }

    if (best_move) {
      if (best_move->kind == PlacementMoveKind::neck_down_obstacle_repair) {
        ++result.neck_down_obstacle_moves_accepted;
      }
      if (best_move->kind == PlacementMoveKind::physical_channel_obstacle_repair) {
        ++result.physical_channel_obstacle_moves_accepted;
      }
      if (best_move->priority_class) {
        const auto tier = priorities->tier_by_class.at(*best_move->priority_class);
        ++result.placement_priority_metrics[tier].accepted_moves;
      }
      current = std::move(best);
      result.accepted_moves.push_back(std::move(*best_move));
      if (current.detailed.complete() && !config.optimize_routed_quality) {
        result.stop_reason = IntegratedOptimizerStopReason::complete;
        break;
      }
    } else if (!budget_exhausted) {
      result.stop_reason = current.detailed.complete() ? IntegratedOptimizerStopReason::complete
                           : config.candidate_evaluation_offset > 0U
                               ? IntegratedOptimizerStopReason::evaluation_limit
                               : IntegratedOptimizerStopReason::no_improving_move;
      break;
    }
    if (budget_exhausted) {
      result.stop_reason = IntegratedOptimizerStopReason::evaluation_limit;
      break;
    }
    if (pass + 1U == config.max_passes) {
      result.stop_reason = IntegratedOptimizerStopReason::pass_limit;
    }
  }

  if (stop_token.stop_requested()) {
    result.stop_reason = IntegratedOptimizerStopReason::cancelled;
  }
  result.score = current.score;
  result.global_route = current.global;
  result.detailed_route = current.detailed;
  result.candidate = result_board(current);
  result.footprint_pressure = collect_integrated_footprint_pressure(
      current.placement, current.global, config.global_router.checkpoint_objective);
  switch (result.stop_reason) {
  case IntegratedOptimizerStopReason::complete:
    result.diagnostic = config.optimize_routed_quality
                            ? "feasible routed checkpoint is a bounded local optimum"
                            : "placement and materialized routing satisfy represented rules";
    break;
  case IntegratedOptimizerStopReason::no_improving_move:
    result.diagnostic = "bounded neighborhood contains no improving placement or orientation move";
    break;
  case IntegratedOptimizerStopReason::pass_limit:
    result.diagnostic = "integrated optimization pass limit reached";
    break;
  case IntegratedOptimizerStopReason::evaluation_limit:
    result.diagnostic = "integrated candidate evaluation limit reached";
    break;
  case IntegratedOptimizerStopReason::cancelled:
    result.diagnostic = "integrated optimization cancelled";
    break;
  case IntegratedOptimizerStopReason::invalid_config:
    break;
  }
  return result;
}

} // namespace flux
