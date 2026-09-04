#include "flux/io/work_plan_router.hpp"

#include <algorithm>
#include <iterator>
#include <limits>
#include <set>
#include <string_view>

namespace flux::io {
namespace {

void error(WorkPlanRouteCompileResult& result, std::string code, std::string message,
           std::string pointer) {
  result.diagnostics.push_back(
      {WorkPlanDiagnosticSeverity::error, std::move(code), std::move(message), std::move(pointer)});
}

[[nodiscard]] const WorkPlanOptionValue* option(const WorkPlanStep& step, std::string_view name) {
  const auto found = step.options.find(std::string(name));
  return found == step.options.end() ? nullptr : &found->second;
}

[[nodiscard]] std::optional<bool> boolean_option(const WorkPlanStep& step, std::string_view name,
                                                 WorkPlanRouteCompileResult& result,
                                                 std::string_view pointer) {
  const auto* value = option(step, name);
  if (value == nullptr) {
    return std::nullopt;
  }
  if (const auto* typed = std::get_if<bool>(value)) {
    return *typed;
  }
  error(result, "invalid_work_plan_option_type", std::string(name) + " must be a boolean",
        std::string(pointer));
  return std::nullopt;
}

[[nodiscard]] std::optional<std::uint64_t>
unsigned_option(const WorkPlanStep& step, std::string_view name, std::uint64_t minimum,
                std::uint64_t maximum, WorkPlanRouteCompileResult& result,
                std::string_view pointer) {
  const auto* value = option(step, name);
  if (value == nullptr) {
    return std::nullopt;
  }
  const auto* typed = std::get_if<std::uint64_t>(value);
  if (typed == nullptr || *typed < minimum || *typed > maximum) {
    error(result, "invalid_work_plan_option_value",
          std::string(name) + " must be an unsigned integer in its supported range",
          std::string(pointer));
    return std::nullopt;
  }
  return *typed;
}

[[nodiscard]] std::optional<std::string> string_option(const WorkPlanStep& step,
                                                       std::string_view name,
                                                       WorkPlanRouteCompileResult& result,
                                                       std::string_view pointer) {
  const auto* value = option(step, name);
  if (value == nullptr) {
    return std::nullopt;
  }
  if (const auto* typed = std::get_if<std::string>(value)) {
    return *typed;
  }
  error(result, "invalid_work_plan_option_type", std::string(name) + " must be a string",
        std::string(pointer));
  return std::nullopt;
}

[[nodiscard]] std::optional<std::vector<std::string>>
string_list_option(const WorkPlanStep& step, std::string_view name,
                   WorkPlanRouteCompileResult& result, std::string_view pointer) {
  const auto* value = option(step, name);
  if (value == nullptr) {
    return std::nullopt;
  }
  if (const auto* typed = std::get_if<std::vector<std::string>>(value)) {
    return *typed;
  }
  error(result, "invalid_work_plan_option_type", std::string(name) + " must be a string list",
        std::string(pointer));
  return std::nullopt;
}

[[nodiscard]] std::vector<NetId> resolve_priority_nets(const Board& source,
                                                       const std::vector<std::string>& names,
                                                       const std::vector<NetId>& selected,
                                                       WorkPlanRouteCompileResult& result,
                                                       std::string_view pointer) {
  const std::set<NetId> selected_set(selected.begin(), selected.end());
  std::set<NetId> seen;
  std::vector<NetId> resolved;
  for (std::size_t index = 0U; index < names.size(); ++index) {
    const auto found = std::find_if(source.nets.begin(), source.nets.end(), [&](const Net& net) {
      return !net.name.empty() && net.name == names[index];
    });
    if (found == source.nets.end() || !selected_set.contains(found->id) ||
        !seen.insert(found->id).second) {
      error(result, "invalid_work_plan_priority_net",
            "priority_nets must contain unique exact names inside the expanded route scope",
            std::string(pointer) + "/" + std::to_string(index));
    } else {
      resolved.push_back(found->id);
    }
  }
  return resolved;
}

[[nodiscard]] bool compiled_route_kind(WorkPlanStepKind kind) {
  return kind == WorkPlanStepKind::route || kind == WorkPlanStepKind::differential_pair ||
         kind == WorkPlanStepKind::bus;
}

[[nodiscard]] bool belongs_to_differential_pair(const Board& source, NetId net) {
  return std::any_of(source.differential_pairs.begin(), source.differential_pairs.end(),
                     [net](const auto& pair) {
                       return pair.positive_net == net || pair.negative_net == net;
                     });
}

[[nodiscard]] bool belongs_to_routing_bus(const Board& source, NetId net) {
  return std::any_of(source.routing_buses.begin(), source.routing_buses.end(),
                     [net](const auto& bus) {
                       return std::find(bus.ordered_nets.begin(), bus.ordered_nets.end(), net) !=
                              bus.ordered_nets.end();
                     });
}

} // namespace

bool WorkPlanRouteCompileResult::has_errors() const {
  return std::any_of(diagnostics.begin(), diagnostics.end(), [](const auto& diagnostic) {
    return diagnostic.severity == WorkPlanDiagnosticSeverity::error;
  });
}

WorkPlanRouteCompileResult compile_flux_work_plan_routes(const Board& source,
                                                         const FluxWorkPlan& plan) {
  WorkPlanRouteCompileResult result;
  result.expansion = expand_flux_work_plan(source, plan);
  result.diagnostics = result.expansion.diagnostics;
  static const std::set<std::string_view> supported_options{
      "atomic_complete_net_routing",
      "bounded_residual_conflict_refinement",
      "checkpoint_objective",
      "circular_arc_refinement",
      "clearance_shadow_cost_quantum_nm",
      "capacity_reserve_cost",
      "capacity_reserve_per_mille",
      "corridor_neck_down_width_nm",
      "crossing_safe_diagonal_grid_edges",
      "diagonal_grid_edges",
      "differential_pair_skew_tuning",
      "differential_pair_skew_tuning_batch_assignment",
      "direct_medial_edges",
      "diversify_pin_access_rings",
      "expansion_retry_heuristic_weight_per_mille",
      "grid_pitch_nm",
      "hard_capacity_start_iteration",
      "interleave_routability_feedback",
      "local_pin_escape_pitch_nm",
      "max_expansions_per_connection",
      "max_final_routability_feedback_passes",
      "max_corridor_neck_down_length_nm",
      "max_corridor_neck_down_states",
      "max_hard_capacity_blocker_reuse_invalidations_per_failed_net",
      "max_hard_capacity_conflict_set_states_per_attempt",
      "max_hard_capacity_ripup_attempts_per_pass",
      "max_hard_capacity_ripup_nets_per_attempt",
      "max_present_congestion_scale_per_mille",
      "max_local_pin_escape_expansions_per_terminal",
      "max_medial_neck_down_length_nm",
      "max_pin_access_nodes_per_terminal",
      "max_post_negotiation_sampled_conflict_repair_cones",
      "max_router_iterations",
      "max_static_seed_workers",
      "max_terminal_escape_reservation_resources",
      "max_terminal_neck_down_length_nm",
      "max_voronoi_resource_segment_length_nm",
      "medial_neck_down_width_nm",
      "multisource_steiner_growth",
      "negotiate_voronoi_lane_conflicts",
      "former_route_memory_cost",
      "physical_envelope_ordering",
      "physical_overflow_history",
      "pin_escape_grid_radius",
      "post_negotiation_sampled_conflict_repair",
      "present_congestion_escalation_start_iteration",
      "present_congestion_multiplier_per_mille",
      "priority_nets",
      "retained_global_portfolios",
      "refillable_zone_fills",
      "reuse_weighted_search_frontier",
      "river_following_cost",
      "routability_feedback_retries",
      "rollback_failed_hard_capacity_ripup",
      "sampled_route_clearance_reservations",
      "defer_hard_capacity_ripup_victims_to_pass_tail",
      "protect_recovered_hard_capacity_ripup_nets",
      "search_diversification",
      "selective_congestion_reroute",
      "selective_full_reroute_interval",
      "spline_route_workers",
      "strict_medial_topology",
      "terminal_escape_proximity_cost",
      "terminal_escape_reservation_cost",
      "terminal_neck_down_width_nm",
      "topology_locked_medial_materialization",
      "via_site_phase_x",
      "via_site_phase_y",
      "via_site_pitch_nm",
      "width_reduction_cost_per_grid_step",
      "xpbd_feedback",
      "xpbd_iterations",
      "xpbd_maximum_step_nm",
      "xpbd_prefer_gpu",
      "xpbd_relaxation",
  };

  for (std::size_t step_index = 0U; step_index < plan.steps.size(); ++step_index) {
    const auto& step = plan.steps[step_index];
    if (!compiled_route_kind(step.kind)) {
      continue;
    }
    const auto expanded = std::find_if(
        result.expansion.steps.begin(), result.expansion.steps.end(),
        [&step](const WorkPlanStepExpansion& candidate) { return candidate.id == step.id; });
    if (expanded == result.expansion.steps.end()) {
      error(result, "missing_work_plan_expansion", "route-like step has no selector expansion",
            "/steps/" + std::to_string(step_index));
      continue;
    }
    CompiledWorkPlanRouteStep compiled;
    compiled.id = step.id;
    compiled.kind = step.kind;
    compiled.configuration = make_autoroute_v1_configuration();
    compiled.selected_nets = expanded->nets;
    compiled.configuration.planner.global.selected_nets = compiled.selected_nets;
    if (step.kind == WorkPlanStepKind::differential_pair &&
        std::any_of(compiled.selected_nets.begin(), compiled.selected_nets.end(),
                    [&source](NetId net) { return !belongs_to_differential_pair(source, net); })) {
      error(result, "invalid_work_plan_differential_pair_scope",
            "a differential-pair step may select only members of declared differential pairs",
            "/steps/" + std::to_string(step_index) + "/selectors");
    }
    if (step.kind == WorkPlanStepKind::bus &&
        std::any_of(compiled.selected_nets.begin(), compiled.selected_nets.end(),
                    [&source](NetId net) { return !belongs_to_routing_bus(source, net); })) {
      error(result, "invalid_work_plan_routing_bus_scope",
            "a bus step may select only members of declared ordered routing buses",
            "/steps/" + std::to_string(step_index) + "/selectors");
    }

    if (plan.mode == WorkPlanMode::force_reroute_selected ||
        plan.mode == WorkPlanMode::allowlisted_ripup) {
      compiled.configuration.detailed.existing_routing =
          ExistingRoutingPolicy::replace_unlocked_for_routed_nets;
      if (plan.mode == WorkPlanMode::force_reroute_selected) {
        std::set<NetId> wildcard(expanded->wildcard_only_nets.begin(),
                                 expanded->wildcard_only_nets.end());
        std::vector<NetId> replacement;
        std::copy_if(compiled.selected_nets.begin(), compiled.selected_nets.end(),
                     std::back_inserter(replacement),
                     [&wildcard](NetId net) { return !wildcard.contains(net); });
        compiled.configuration.detailed.existing_routing_replacement_scope = std::move(replacement);
      } else {
        compiled.configuration.detailed.existing_routing_replacement_scope =
            result.expansion.copper_authority.reroutable_nets;
      }
      if (plan.mode == WorkPlanMode::allowlisted_ripup &&
          step.kind == WorkPlanStepKind::route) {
        for (const auto& net : result.expansion.copper_authority.reroutable_nets) {
          if (std::find(compiled.selected_nets.begin(), compiled.selected_nets.end(), net) ==
              compiled.selected_nets.end()) {
            compiled.selected_nets.push_back(net);
          }
        }
        std::sort(compiled.selected_nets.begin(), compiled.selected_nets.end());
        compiled.configuration.planner.global.selected_nets = compiled.selected_nets;
      }
    }

    for (const auto& [name, unused] : step.options) {
      static_cast<void>(unused);
      if (!supported_options.contains(name)) {
        error(result, "unsupported_work_plan_route_option",
              "route option is not supported by the canonical configuration adapter",
              "/steps/" + std::to_string(step_index) + "/options/" + name);
      } else {
        compiled.applied_options.push_back(name);
      }
    }
    const auto option_pointer = [&](std::string_view name) {
      return "/steps/" + std::to_string(step_index) + "/options/" + std::string(name);
    };
    if (const auto value = unsigned_option(step, "grid_pitch_nm", 1U, max_abs_coordinate_nm, result,
                                           option_pointer("grid_pitch_nm")))
      compiled.configuration.planner.global.grid_pitch =
          Nanometres{static_cast<std::int64_t>(*value)};
    if (const auto value = unsigned_option(step, "via_site_pitch_nm", 0U, max_abs_coordinate_nm,
                                           result, option_pointer("via_site_pitch_nm")))
      compiled.configuration.planner.global.via_site_pitch =
          Nanometres{static_cast<std::int64_t>(*value)};
    if (const auto value = unsigned_option(step, "max_router_iterations", 1U, 100'000U, result,
                                           option_pointer("max_router_iterations")))
      compiled.configuration.planner.global.max_iterations = static_cast<std::size_t>(*value);
    if (const auto value = unsigned_option(step, "max_expansions_per_connection", 1U, 100'000'000U,
                                           result, option_pointer("max_expansions_per_connection")))
      compiled.configuration.planner.global.max_expansions_per_connection =
          static_cast<std::size_t>(*value);
    if (const auto value = unsigned_option(step, "max_static_seed_workers", 0U, 256U, result,
                                           option_pointer("max_static_seed_workers")))
      compiled.configuration.planner.global.max_static_seed_workers =
          static_cast<std::size_t>(*value);
    if (const auto value = unsigned_option(
            step, "max_final_routability_feedback_passes", 1U, 64U, result,
            option_pointer("max_final_routability_feedback_passes")))
      compiled.max_final_routability_feedback_passes = static_cast<std::size_t>(*value);
    if (const auto value = unsigned_option(
            step, "max_post_negotiation_sampled_conflict_repair_cones", 1U, 64U, result,
            option_pointer("max_post_negotiation_sampled_conflict_repair_cones")))
      compiled.max_post_negotiation_sampled_conflict_repair_cones =
          static_cast<std::size_t>(*value);
    if (const auto value = unsigned_option(step, "routability_feedback_retries", 0U, 64U, result,
                                           option_pointer("routability_feedback_retries")))
      compiled.configuration.planner.max_routability_feedback_retries =
          static_cast<std::size_t>(*value);
    if (const auto value = unsigned_option(step, "retained_global_portfolios", 1U, 64U, result,
                                           option_pointer("retained_global_portfolios")))
      compiled.configuration.planner.retained_global_portfolios = static_cast<std::size_t>(*value);
    if (const auto value = unsigned_option(step, "xpbd_iterations", 1U, 4'096U, result,
                                           option_pointer("xpbd_iterations")))
      compiled.configuration.planner.regional.xpbd.iterations = static_cast<std::size_t>(*value);
    if (const auto value = unsigned_option(step, "spline_route_workers", 1U, 256U, result,
                                           option_pointer("spline_route_workers")))
      compiled.configuration.detailed.spline_route_workers = static_cast<std::size_t>(*value);
    constexpr auto maximum_size = static_cast<std::uint64_t>(
        std::numeric_limits<std::size_t>::max());
    const auto set_global_size = [&](std::string_view name, std::size_t& target,
                                     std::uint64_t minimum = 1U) {
      if (const auto value =
              unsigned_option(step, name, minimum, maximum_size, result, option_pointer(name))) {
        target = static_cast<std::size_t>(*value);
      }
    };
    set_global_size("hard_capacity_start_iteration",
                    compiled.configuration.planner.global.hard_capacity_start_iteration);
    set_global_size("max_hard_capacity_ripup_attempts_per_pass",
                    compiled.configuration.planner.global
                        .max_hard_capacity_ripup_attempts_per_pass);
    set_global_size("max_hard_capacity_ripup_nets_per_attempt",
                    compiled.configuration.planner.global.max_hard_capacity_ripup_nets_per_attempt);
    set_global_size("max_hard_capacity_conflict_set_states_per_attempt",
                    compiled.configuration.planner.global
                        .max_hard_capacity_conflict_set_states_per_attempt);
    set_global_size("max_hard_capacity_blocker_reuse_invalidations_per_failed_net",
                    compiled.configuration.planner.global
                        .max_hard_capacity_blocker_reuse_invalidations_per_failed_net);
    set_global_size("selective_full_reroute_interval",
                    compiled.configuration.planner.global.selective_full_reroute_interval);
    set_global_size("via_site_phase_x", compiled.configuration.planner.global.via_site_phase_x,
                    0U);
    set_global_size("via_site_phase_y", compiled.configuration.planner.global.via_site_phase_y,
                    0U);
    set_global_size("pin_escape_grid_radius",
                    compiled.configuration.planner.global.max_pin_escape_grid_radius, 0U);
    set_global_size("max_pin_access_nodes_per_terminal",
                    compiled.configuration.planner.global.max_pin_access_nodes_per_terminal);
    set_global_size("max_local_pin_escape_expansions_per_terminal",
                    compiled.configuration.planner.global
                        .max_local_pin_escape_expansions_per_terminal);
    set_global_size("max_corridor_neck_down_states",
                    compiled.configuration.planner.global.max_corridor_neck_down_states);
    set_global_size("max_terminal_escape_reservation_resources",
                    compiled.configuration.planner.global
                        .max_terminal_escape_reservation_resources,
                    0U);
    const auto set_global_u64 = [&](std::string_view name, std::uint64_t& target) {
      if (const auto value = unsigned_option(step, name, 0U,
                                             std::numeric_limits<std::uint64_t>::max(), result,
                                             option_pointer(name))) {
        target = *value;
      }
    };
    set_global_u64("capacity_reserve_cost",
                   compiled.configuration.planner.global.capacity_reserve_cost);
    set_global_u64("former_route_memory_cost",
                   compiled.configuration.planner.global.former_route_memory_cost);
    set_global_u64("river_following_cost",
                   compiled.configuration.planner.global.river_following_cost);
    set_global_u64("terminal_escape_proximity_cost",
                   compiled.configuration.planner.global.terminal_escape_proximity_cost);
    set_global_u64("terminal_escape_reservation_cost",
                   compiled.configuration.planner.global.terminal_escape_reservation_cost);
    set_global_u64("width_reduction_cost_per_grid_step",
                   compiled.configuration.planner.global.width_reduction_cost_per_grid_step);
    if (const auto value = unsigned_option(
            step, "clearance_shadow_cost_quantum_nm", 1U, max_abs_coordinate_nm, result,
            option_pointer("clearance_shadow_cost_quantum_nm"))) {
      compiled.configuration.planner.global.clearance_shadow_cost_quantum =
          Nanometres{static_cast<std::int64_t>(*value)};
    }
    const auto set_global_positive_nm = [&](std::string_view name, Nanometres& target) {
      if (const auto value = unsigned_option(step, name, 1U, max_abs_coordinate_nm, result,
                                             option_pointer(name))) {
        target = Nanometres{static_cast<std::int64_t>(*value)};
      }
    };
    set_global_positive_nm("local_pin_escape_pitch_nm",
                           compiled.configuration.planner.global.local_pin_escape_pitch);
    set_global_positive_nm("terminal_neck_down_width_nm",
                           compiled.configuration.planner.global.terminal_neck_down_width);
    set_global_positive_nm("max_terminal_neck_down_length_nm",
                           compiled.configuration.planner.global.max_terminal_neck_down_length);
    set_global_positive_nm("medial_neck_down_width_nm",
                           compiled.configuration.planner.global.medial_neck_down_width);
    set_global_positive_nm("max_medial_neck_down_length_nm",
                           compiled.configuration.planner.global.max_medial_neck_down_length);
    set_global_positive_nm("corridor_neck_down_width_nm",
                           compiled.configuration.planner.global.corridor_neck_down_width);
    set_global_positive_nm("max_corridor_neck_down_length_nm",
                           compiled.configuration.planner.global.max_corridor_neck_down_length);
    set_global_positive_nm("max_voronoi_resource_segment_length_nm",
                           compiled.configuration.planner.global
                               .max_voronoi_resource_segment_length);
    if (const auto value = unsigned_option(step, "xpbd_maximum_step_nm", 1U,
                                           max_abs_coordinate_nm, result,
                                           option_pointer("xpbd_maximum_step_nm"))) {
      compiled.configuration.planner.regional.xpbd.maximum_step_nm =
          static_cast<std::int64_t>(*value);
    }
    if (const auto value = unsigned_option(step, "capacity_reserve_per_mille", 0U, 1'000U,
                                           result, option_pointer("capacity_reserve_per_mille"))) {
      compiled.configuration.planner.global.capacity_reserve_per_mille =
          static_cast<std::uint32_t>(*value);
    }
    if (const auto value = unsigned_option(
            step, "expansion_retry_heuristic_weight_per_mille", 0U, 10'000U, result,
            option_pointer("expansion_retry_heuristic_weight_per_mille"))) {
      if (*value != 0U && *value <= 1'000U) {
        error(result, "invalid_work_plan_option_value",
              "expansion_retry_heuristic_weight_per_mille must be zero or in (1000, 10000]",
              option_pointer("expansion_retry_heuristic_weight_per_mille"));
      } else {
        compiled.configuration.planner.global.expansion_retry_heuristic_weight_per_mille =
            static_cast<std::uint32_t>(*value);
      }
    }
    if (const auto value = unsigned_option(step, "present_congestion_multiplier_per_mille",
                                           1'000U,
                                           std::numeric_limits<std::uint32_t>::max(), result,
                                           option_pointer("present_congestion_multiplier_per_mille"))) {
      compiled.configuration.planner.global.present_congestion_multiplier_per_mille =
          static_cast<std::uint32_t>(*value);
    }
    if (const auto value = unsigned_option(
            step, "max_present_congestion_scale_per_mille", 1'000U,
            std::numeric_limits<std::uint64_t>::max(), result,
            option_pointer("max_present_congestion_scale_per_mille"))) {
      compiled.configuration.planner.global.max_present_congestion_scale_per_mille = *value;
    }
    set_global_size("present_congestion_escalation_start_iteration",
                    compiled.configuration.planner.global
                        .present_congestion_escalation_start_iteration,
                    2U);

    const auto set_bool = [&](std::string_view name, bool& target) {
      if (const auto value = boolean_option(step, name, result, option_pointer(name))) {
        target = *value;
      }
    };
    set_bool("crossing_safe_diagonal_grid_edges",
             compiled.configuration.planner.global.allow_crossing_safe_diagonal_grid_edges);
    set_bool("diagonal_grid_edges",
             compiled.configuration.planner.global.allow_diagonal_grid_edges);
    set_bool("sampled_route_clearance_reservations",
             compiled.configuration.planner.global.reserve_sampled_route_clearance);
    set_bool("strict_medial_topology",
             compiled.configuration.planner.global.require_voronoi_topological_transitions);
    set_bool("direct_medial_edges",
             compiled.configuration.planner.global.use_direct_voronoi_resource_edges);
    set_bool("negotiate_voronoi_lane_conflicts",
             compiled.configuration.planner.global.negotiate_voronoi_lane_conflicts);
    set_bool("diversify_pin_access_rings",
             compiled.configuration.planner.global.diversify_pin_access_rings);
    set_bool("multisource_steiner_growth",
             compiled.configuration.planner.global.use_multisource_steiner_growth);
    set_bool("atomic_complete_net_routing",
             compiled.configuration.planner.global.atomic_complete_net_routing);
    set_bool("selective_congestion_reroute",
             compiled.configuration.planner.global.selective_congestion_reroute);
    set_bool("reuse_weighted_search_frontier",
             compiled.configuration.planner.global.reuse_weighted_search_frontier);
    set_bool("physical_overflow_history",
             compiled.configuration.planner.global.scale_voronoi_history_by_overflow);
    set_bool("physical_envelope_ordering",
             compiled.configuration.planner.global.order_routing_groups_by_physical_envelope);
    set_bool("rollback_failed_hard_capacity_ripup",
             compiled.configuration.planner.global.rollback_failed_hard_capacity_ripup);
    set_bool("defer_hard_capacity_ripup_victims_to_pass_tail",
             compiled.configuration.planner.global
                 .defer_hard_capacity_ripup_victims_to_pass_tail);
    set_bool("protect_recovered_hard_capacity_ripup_nets",
             compiled.configuration.planner.global.protect_recovered_hard_capacity_ripup_nets);
    set_bool("search_diversification",
             compiled.configuration.planner.global.enable_search_diversification);
    set_bool("interleave_routability_feedback",
             compiled.configuration.planner.interleave_routability_feedback);
    set_bool("xpbd_relaxation", compiled.configuration.planner.regional.enable_xpbd_relaxation);
    set_bool("xpbd_feedback", compiled.configuration.planner.regional.enable_xpbd_feedback);
    set_bool("xpbd_prefer_gpu", compiled.configuration.planner.regional.xpbd.prefer_gpu);
    set_bool("topology_locked_medial_materialization",
             compiled.configuration.detailed.topology_locked_medial_materialization);
    set_bool("bounded_residual_conflict_refinement",
             compiled.configuration.detailed.bounded_residual_conflict_refinement);
    set_bool("circular_arc_refinement", compiled.configuration.detailed.circular_arc_refinement);
    set_bool("differential_pair_skew_tuning",
             compiled.configuration.detailed.differential_pair_skew_tuning);
    set_bool("differential_pair_skew_tuning_batch_assignment",
             compiled.configuration.detailed.differential_pair_skew_tuning_batch_assignment);
    if (const auto value = boolean_option(
            step, "post_negotiation_sampled_conflict_repair", result,
            option_pointer("post_negotiation_sampled_conflict_repair"))) {
      compiled.post_negotiation_sampled_conflict_repair = *value;
    }
    if (const auto value = boolean_option(step, "refillable_zone_fills", result,
                                          option_pointer("refillable_zone_fills"))) {
      compiled.refillable_zone_fills = *value;
    }
    if (compiled.configuration.planner.regional.enable_xpbd_feedback) {
      compiled.configuration.planner.regional.enable_xpbd_relaxation = true;
    }
    if (compiled.configuration.planner.global.enable_search_diversification) {
      // Match the direct CLI's high-level switch: large-neighborhood destruction operates on
      // complete net trees, so enabling it also enables atomic routing.
      compiled.configuration.planner.global.atomic_complete_net_routing = true;
    }
    if (compiled.configuration.detailed.differential_pair_skew_tuning_batch_assignment) {
      compiled.configuration.detailed.differential_pair_skew_tuning = true;
    }
    if (const auto value = string_option(step, "checkpoint_objective", result,
                                         option_pointer("checkpoint_objective"))) {
      if (*value == "aggregate-overflow") {
        compiled.configuration.planner.global.checkpoint_objective =
            GlobalRouteCheckpointObjective::aggregate_overflow;
      } else if (*value == "dimensioned-physical-overflow") {
        compiled.configuration.planner.global.checkpoint_objective =
            GlobalRouteCheckpointObjective::dimensioned_physical_overflow;
      } else {
        error(result, "invalid_work_plan_option_value",
              "checkpoint_objective must name a supported feasibility comparator",
              option_pointer("checkpoint_objective"));
      }
    }
    if (const auto names =
            string_list_option(step, "priority_nets", result, option_pointer("priority_nets"))) {
      compiled.priority_nets = resolve_priority_nets(source, *names, compiled.selected_nets, result,
                                                     option_pointer("priority_nets"));
      compiled.configuration.planner.global.priority_nets = compiled.priority_nets;
    }
    const auto& global = compiled.configuration.planner.global;
    const auto dependency_error = [&](std::string_view name, std::string message) {
      error(result, "invalid_work_plan_route_option_dependency", std::move(message),
            option_pointer(name));
    };
    if (option(step, "max_post_negotiation_sampled_conflict_repair_cones") != nullptr &&
        !compiled.post_negotiation_sampled_conflict_repair) {
      dependency_error("max_post_negotiation_sampled_conflict_repair_cones",
                       "a sampled-conflict repair cone budget requires post-negotiation repair");
    }
    if (compiled.refillable_zone_fills) {
      if (step_index + 1U != plan.steps.size()) {
        dependency_error("refillable_zone_fills",
                         "a refill-required route must be the final WorkPlan step");
      }
      if (compiled.configuration.detailed.differential_pair_skew_tuning ||
          compiled.configuration.detailed.differential_pair_skew_tuning_batch_assignment) {
        dependency_error("refillable_zone_fills",
                         "refill-required routing cannot run differential-pair skew tuning");
      }
      compiled.configuration.planner.global.treat_saved_zone_fills_as_refillable = true;
      compiled.configuration.planner.regional.treat_saved_zone_fills_as_refillable = true;
      compiled.configuration.planner.global.single_lane_direct_voronoi_resources =
          compiled.configuration.planner.global.use_direct_voronoi_resource_edges;
      compiled.configuration.detailed.defer_saved_zone_fill_clearance_to_refill = true;
      compiled.configuration.detailed.pull_tight_45_degrees = false;
      compiled.configuration.detailed.continuous_refinement = false;
      compiled.configuration.detailed.topology_locked_medial_materialization = false;
    }
    if (global.hard_capacity_start_iteration > 0U &&
        !global.require_voronoi_topological_transitions &&
        !global.use_direct_voronoi_resource_edges) {
      dependency_error("hard_capacity_start_iteration",
                       "hard-capacity routing requires strict-medial topology or direct-medial "
                       "edges");
    }
    if (global.allow_diagonal_grid_edges && global.allow_crossing_safe_diagonal_grid_edges) {
      dependency_error("diagonal_grid_edges",
                       "checkerboard and crossing-safe diagonal grid policies are mutually "
                       "exclusive");
    }
    const bool explicit_via_pitch = global.via_site_pitch.value() > 0;
    if ((!explicit_via_pitch &&
         (global.via_site_phase_x != 0U || global.via_site_phase_y != 0U)) ||
        (explicit_via_pitch &&
         (global.via_site_pitch < global.grid_pitch ||
          global.via_site_pitch.value() % global.grid_pitch.value() != 0 ||
          global.via_site_phase_x >= static_cast<std::size_t>(global.via_site_pitch.value() /
                                                               global.grid_pitch.value()) ||
          global.via_site_phase_y >= static_cast<std::size_t>(global.via_site_pitch.value() /
                                                               global.grid_pitch.value())))) {
      dependency_error("via_site_pitch_nm",
                       "via-site pitch must be a grid multiple and X/Y phases must lie within its "
                       "stride");
    }
    if (global.local_pin_escape_pitch.value() > 0 &&
        (global.local_pin_escape_pitch > global.grid_pitch ||
         global.grid_pitch.value() % global.local_pin_escape_pitch.value() != 0)) {
      dependency_error("local_pin_escape_pitch_nm",
                       "local pin-escape pitch must be a positive exact divisor of grid pitch");
    }
    const auto require_paired_dimensions = [&](std::string_view first_name, Nanometres first,
                                               std::string_view second_name,
                                               Nanometres second) {
      if ((first.value() > 0) != (second.value() > 0)) {
        dependency_error(first_name, std::string(first_name) + " and " +
                                         std::string(second_name) +
                                         " must be positive together");
      }
    };
    require_paired_dimensions("terminal_neck_down_width_nm", global.terminal_neck_down_width,
                              "max_terminal_neck_down_length_nm",
                              global.max_terminal_neck_down_length);
    require_paired_dimensions("medial_neck_down_width_nm", global.medial_neck_down_width,
                              "max_medial_neck_down_length_nm",
                              global.max_medial_neck_down_length);
    require_paired_dimensions("corridor_neck_down_width_nm", global.corridor_neck_down_width,
                              "max_corridor_neck_down_length_nm",
                              global.max_corridor_neck_down_length);
    if (global.medial_neck_down_width.value() > 0 &&
        !global.use_direct_voronoi_resource_edges) {
      dependency_error("medial_neck_down_width_nm",
                       "medial neck-down requires direct-medial routing edges");
    }
    if (global.reuse_weighted_search_frontier &&
        global.expansion_retry_heuristic_weight_per_mille == 0U) {
      dependency_error("reuse_weighted_search_frontier",
                       "weighted frontier reuse requires a nonzero retry heuristic weight");
    }
    if (global.max_present_congestion_scale_per_mille <
        global.present_congestion_multiplier_per_mille) {
      dependency_error("max_present_congestion_scale_per_mille",
                       "present-congestion scale cap must not be smaller than its multiplier");
    }
    if (global.hard_capacity_start_iteration > global.max_iterations) {
      dependency_error("hard_capacity_start_iteration",
                       "hard-capacity routing must start within the configured iteration bound");
    }
    if (global.selective_congestion_reroute && !global.atomic_complete_net_routing) {
      dependency_error("selective_congestion_reroute",
                       "selective congestion rerouting requires atomic complete-net routing");
    }
    const bool has_ripup_attempts = global.max_hard_capacity_ripup_attempts_per_pass > 0U;
    const bool has_ripup_nets = global.max_hard_capacity_ripup_nets_per_attempt > 0U;
    if (has_ripup_attempts != has_ripup_nets) {
      dependency_error("max_hard_capacity_ripup_attempts_per_pass",
                       "hard-capacity rip-up attempt and net bounds must be positive together");
    } else if (has_ripup_attempts &&
               (global.hard_capacity_start_iteration == 0U ||
                !global.atomic_complete_net_routing)) {
      dependency_error("max_hard_capacity_ripup_attempts_per_pass",
                       "hard-capacity rip-up requires a hard-capacity start and atomic complete-net "
                       "routing");
    }
    if (global.max_hard_capacity_conflict_set_states_per_attempt > 0U &&
        !has_ripup_attempts) {
      dependency_error("max_hard_capacity_conflict_set_states_per_attempt",
                       "joint blocker-set search requires hard-capacity rip-up");
    }
    if ((global.rollback_failed_hard_capacity_ripup ||
         global.defer_hard_capacity_ripup_victims_to_pass_tail ||
         global.protect_recovered_hard_capacity_ripup_nets) &&
        !has_ripup_attempts) {
      dependency_error("max_hard_capacity_ripup_attempts_per_pass",
                       "hard-capacity transaction policies require hard-capacity rip-up");
    }
    if (global.selective_full_reroute_interval > 0U &&
        !global.selective_congestion_reroute) {
      dependency_error("selective_full_reroute_interval",
                       "periodic full rerouting requires selective congestion rerouting");
    }
    if (global.max_hard_capacity_blocker_reuse_invalidations_per_failed_net > 0U &&
        (!global.selective_congestion_reroute || global.hard_capacity_start_iteration == 0U)) {
      dependency_error("max_hard_capacity_blocker_reuse_invalidations_per_failed_net",
                       "blocker-driven reuse invalidation requires selective rerouting and a "
                       "hard-capacity start");
    }
    result.route_steps.push_back(std::move(compiled));
  }
  return result;
}

} // namespace flux::io
