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
      "crossing_safe_diagonal_grid_edges",
      "direct_medial_edges",
      "diversify_pin_access_rings",
      "grid_pitch_nm",
      "hard_capacity_start_iteration",
      "interleave_routability_feedback",
      "max_expansions_per_connection",
      "max_hard_capacity_blocker_reuse_invalidations_per_failed_net",
      "max_hard_capacity_conflict_set_states_per_attempt",
      "max_hard_capacity_ripup_attempts_per_pass",
      "max_hard_capacity_ripup_nets_per_attempt",
      "max_router_iterations",
      "max_static_seed_workers",
      "multisource_steiner_growth",
      "negotiate_voronoi_lane_conflicts",
      "former_route_memory_cost",
      "priority_nets",
      "retained_global_portfolios",
      "routability_feedback_retries",
      "rollback_failed_hard_capacity_ripup",
      "defer_hard_capacity_ripup_victims_to_pass_tail",
      "protect_recovered_hard_capacity_ripup_nets",
      "search_diversification",
      "selective_congestion_reroute",
      "selective_full_reroute_interval",
      "spline_route_workers",
      "strict_medial_topology",
      "topology_locked_medial_materialization",
      "via_site_pitch_nm",
      "xpbd_feedback",
      "xpbd_iterations",
      "xpbd_relaxation",
  };

  for (std::size_t step_index = 0U; step_index < plan.steps.size(); ++step_index) {
    const auto& step = plan.steps[step_index];
    if (step.kind != WorkPlanStepKind::route) {
      continue;
    }
    const auto expanded = std::find_if(
        result.expansion.steps.begin(), result.expansion.steps.end(),
        [&step](const WorkPlanStepExpansion& candidate) { return candidate.id == step.id; });
    if (expanded == result.expansion.steps.end()) {
      error(result, "missing_work_plan_expansion", "route step has no selector expansion",
            "/steps/" + std::to_string(step_index));
      continue;
    }
    CompiledWorkPlanRouteStep compiled;
    compiled.id = step.id;
    compiled.configuration = make_autoroute_v1_configuration();
    compiled.selected_nets = expanded->nets;
    compiled.configuration.planner.global.selected_nets = compiled.selected_nets;

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
      if (plan.mode == WorkPlanMode::allowlisted_ripup) {
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
    if (const auto value = unsigned_option(step, "former_route_memory_cost", 0U, UINT64_MAX,
                                           result, option_pointer("former_route_memory_cost"))) {
      compiled.configuration.planner.global.former_route_memory_cost = *value;
    }

    const auto set_bool = [&](std::string_view name, bool& target) {
      if (const auto value = boolean_option(step, name, result, option_pointer(name))) {
        target = *value;
      }
    };
    set_bool("crossing_safe_diagonal_grid_edges",
             compiled.configuration.planner.global.allow_crossing_safe_diagonal_grid_edges);
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
    set_bool("topology_locked_medial_materialization",
             compiled.configuration.detailed.topology_locked_medial_materialization);
    set_bool("bounded_residual_conflict_refinement",
             compiled.configuration.detailed.bounded_residual_conflict_refinement);
    set_bool("circular_arc_refinement", compiled.configuration.detailed.circular_arc_refinement);
    if (compiled.configuration.planner.regional.enable_xpbd_feedback) {
      compiled.configuration.planner.regional.enable_xpbd_relaxation = true;
    }
    if (compiled.configuration.planner.global.enable_search_diversification) {
      // Match the direct CLI's high-level switch: large-neighborhood destruction operates on
      // complete net trees, so enabling it also enables atomic routing.
      compiled.configuration.planner.global.atomic_complete_net_routing = true;
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
    if (global.hard_capacity_start_iteration > 0U &&
        !global.require_voronoi_topological_transitions &&
        !global.use_direct_voronoi_resource_edges) {
      dependency_error("hard_capacity_start_iteration",
                       "hard-capacity routing requires strict-medial topology or direct-medial "
                       "edges");
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
