#include "flux/io/work_plan_router.hpp"

#include <algorithm>
#include <iterator>
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
      "interleave_routability_feedback",
      "max_expansions_per_connection",
      "max_router_iterations",
      "max_static_seed_workers",
      "multisource_steiner_growth",
      "negotiate_voronoi_lane_conflicts",
      "priority_nets",
      "retained_global_portfolios",
      "routability_feedback_retries",
      "selective_congestion_reroute",
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
    result.route_steps.push_back(std::move(compiled));
  }
  return result;
}

} // namespace flux::io
