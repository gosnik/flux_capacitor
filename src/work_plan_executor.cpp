#include "flux/io/work_plan_executor.hpp"

#include "flux/core/connectivity.hpp"
#include "flux/core/drc.hpp"
#include "flux/core/integrated_optimizer.hpp"
#include "flux/core/routing_feedback.hpp"
#include "flux/core/validation.hpp"
#include "flux/io/placement_move_priorities.hpp"

#include <boost/json.hpp>
#include <openssl/evp.h>

#include <algorithm>
#include <array>
#include <map>
#include <memory>
#include <set>

namespace flux::io {
namespace {

namespace json = boost::json;

[[nodiscard]] std::string sha256(std::string_view contents) {
  const auto deleter = [](EVP_MD_CTX* context) { EVP_MD_CTX_free(context); };
  std::unique_ptr<EVP_MD_CTX, decltype(deleter)> context(EVP_MD_CTX_new(), deleter);
  std::array<unsigned char, EVP_MAX_MD_SIZE> digest{};
  unsigned int size = 0U;
  if (!context || EVP_DigestInit_ex(context.get(), EVP_sha256(), nullptr) != 1 ||
      EVP_DigestUpdate(context.get(), contents.data(), contents.size()) != 1 ||
      EVP_DigestFinal_ex(context.get(), digest.data(), &size) != 1 || size != 32U) {
    return {};
  }
  constexpr char hexadecimal[] = "0123456789abcdef";
  std::string result(size * 2U, '0');
  for (std::size_t index = 0U; index < size; ++index) {
    result[index * 2U] = hexadecimal[digest[index] >> 4U];
    result[index * 2U + 1U] = hexadecimal[digest[index] & 0x0fU];
  }
  return result;
}

[[nodiscard]] WorkPlanAcceptanceState acceptance(const Board& board,
                                                 const std::set<NetId>* selected = nullptr) {
  WorkPlanAcceptanceState result;
  const auto validation = validate_board(board);
  auto connectivity = analyze_connectivity(board);
  if (selected != nullptr) {
    std::erase_if(connectivity.nets,
                  [selected](const auto& net) { return !selected->contains(net.net); });
    connectivity.open_connections = 0U;
    connectivity.unrouted_nets = 0U;
    for (const auto& net : connectivity.nets) {
      connectivity.open_connections += net.open_connections;
      connectivity.unrouted_nets += net.open_connections == 0U ? 0U : 1U;
    }
  }
  const auto copper = check_copper_rules(board);
  result.structural_validation_ok = validation.ok();
  result.structural_issue_count = validation.issues.size();
  result.open_connections = connectivity.open_connections;
  result.unrouted_nets = connectivity.unrouted_nets;
  result.copper_violations = copper.violation_count();
  result.exact_internal_acceptance =
      validation.ok() && connectivity.open_connections == 0U && copper.violation_count() == 0U;
  return result;
}

void remove_authorized_routing(Board& board, const std::set<NetId>& nets) {
  std::erase_if(board.tracks,
                [&nets](const auto& track) { return !track.locked && nets.contains(track.net); });
  std::erase_if(board.track_beziers,
                [&nets](const auto& curve) { return !curve.locked && nets.contains(curve.net); });
  std::erase_if(board.track_arcs,
                [&nets](const auto& arc) { return !arc.locked && nets.contains(arc.net); });
  std::erase_if(board.vias,
                [&nets](const auto& via) { return !via.locked && nets.contains(via.net); });
}

[[nodiscard]] std::set<NetId> nets_requiring_new_routing(const Board& board,
                                                         const std::vector<NetId>& selected) {
  std::set<NetId> result;
  const std::set<NetId> selected_set(selected.begin(), selected.end());
  for (const auto& net : analyze_connectivity(board).nets) {
    if (selected_set.contains(net.net) && net.open_connections > 0U) {
      result.insert(net.net);
    }
  }
  bool changed = true;
  while (changed) {
    changed = false;
    const auto close_group = [&](const std::vector<NetId>& members) {
      if (std::any_of(members.begin(), members.end(),
                      [&result](NetId net) { return result.contains(net); })) {
        for (const auto& net : members) {
          if (selected_set.contains(net)) {
            changed = result.insert(net).second || changed;
          }
        }
      }
    };
    for (const auto& pair : board.differential_pairs) {
      close_group({pair.positive_net, pair.negative_net});
    }
    for (const auto& bus : board.routing_buses) {
      close_group(bus.ordered_nets);
    }
  }
  return result;
}

[[nodiscard]] std::optional<CompiledWorkPlanRouteStep>
compiled_step(const WorkPlanRouteCompileResult& compiled, std::string_view id) {
  const auto found = std::find_if(compiled.route_steps.begin(), compiled.route_steps.end(),
                                  [id](const auto& step) { return step.id == id; });
  return found == compiled.route_steps.end() ? std::nullopt
                                             : std::optional<CompiledWorkPlanRouteStep>{*found};
}

[[nodiscard]] std::string_view
integrated_stop_reason_name(IntegratedOptimizerStopReason reason) {
  switch (reason) {
  case IntegratedOptimizerStopReason::complete:
    return "complete";
  case IntegratedOptimizerStopReason::no_improving_move:
    return "no-improving-move";
  case IntegratedOptimizerStopReason::pass_limit:
    return "pass-limit";
  case IntegratedOptimizerStopReason::evaluation_limit:
    return "evaluation-limit";
  case IntegratedOptimizerStopReason::invalid_config:
    return "invalid-config";
  case IntegratedOptimizerStopReason::cancelled:
    return "cancelled";
  }
  return "unknown";
}

void restore_footprint_locks(Board& candidate, const Board& source) {
  for (auto& footprint : candidate.footprints) {
    const auto original = std::find_if(
        source.footprints.begin(), source.footprints.end(),
        [&footprint](const Footprint& item) { return item.id == footprint.id; });
    if (original != source.footprints.end()) {
      footprint.locked = original->locked;
    }
  }
}

[[nodiscard]] bool only_selected_footprints_moved(const Board& source, const Board& candidate,
                                                  const std::set<ObjectId>& selected) {
  return std::all_of(source.footprints.begin(), source.footprints.end(), [&](const auto& original) {
    const auto placed = std::find_if(candidate.footprints.begin(), candidate.footprints.end(),
                                     [&](const auto& item) { return item.id == original.id; });
    return placed != candidate.footprints.end() &&
           (selected.contains(original.id) ||
            (placed->position == original.position && placed->rotation == original.rotation &&
             placed->side == original.side));
  });
}

void append_diagnostic(WorkPlanExecutionResult& result, std::string code, std::string message,
                       std::string pointer) {
  result.diagnostics.push_back(
      {WorkPlanDiagnosticSeverity::error, std::move(code), std::move(message), std::move(pointer)});
}

} // namespace

std::string_view work_plan_step_execution_status_name(WorkPlanStepExecutionStatus status) {
  switch (status) {
  case WorkPlanStepExecutionStatus::completed:
    return "completed";
  case WorkPlanStepExecutionStatus::failed:
    return "failed";
  case WorkPlanStepExecutionStatus::cancelled:
    return "cancelled";
  case WorkPlanStepExecutionStatus::skipped:
    return "skipped";
  }
  return "unknown";
}

namespace {

WorkPlanExecutionResult execute_impl(const Board& source, const FluxWorkPlan& plan,
                                     bool policies_preapplied, std::stop_token stop_token,
                                     RouterTelemetrySink* telemetry,
                                     const PlacementMovePriorityPolicy* placement_priorities) {
  WorkPlanExecutionResult result;
  const auto emit = [telemetry](RouterTelemetryEvent event) {
    if (telemetry != nullptr) {
      static_cast<void>(telemetry->try_emit(std::move(event)));
    }
  };
  result.mode = plan.mode;
  result.preview = plan.mode == WorkPlanMode::preview;
  result.plan_sha256 = sha256(serialize_flux_work_plan(plan));
  const auto compiled = compile_flux_work_plan_routes(source, plan);
  result.diagnostics = compiled.diagnostics;
  if (!plan.policies.empty() && !policies_preapplied) {
    append_diagnostic(
        result, "unsupported_work_plan_execution_policy",
        "bound policy identities must have an authoritative board adapter before execution",
        "/policies");
  }
  for (std::size_t index = 0U; index < plan.steps.size(); ++index) {
    const auto& step = plan.steps[index];
    if (step.kind != WorkPlanStepKind::route &&
        step.kind != WorkPlanStepKind::differential_pair &&
        step.kind != WorkPlanStepKind::bus && step.kind != WorkPlanStepKind::placement &&
        step.kind != WorkPlanStepKind::validation) {
      append_diagnostic(result, "unsupported_work_plan_execution_step",
                        "this step kind has no authoritative execution adapter",
                        "/steps/" + std::to_string(index) + "/kind");
    }
    if (step.kind == WorkPlanStepKind::validation && !step.options.empty()) {
      append_diagnostic(result, "unsupported_work_plan_validation_option",
                        "validation options are not yet supported",
                        "/steps/" + std::to_string(index) + "/options");
    }
  }
  if (!compiled.ok() || !result.diagnostics.empty() || result.plan_sha256.empty()) {
    if (result.plan_sha256.empty()) {
      append_diagnostic(result, "work_plan_identity_failed",
                        "unable to calculate deterministic plan identity", "");
    }
    result.candidate = source;
    result.final_acceptance = acceptance(source);
    return result;
  }

  for (const auto& expansion : compiled.expansion.steps) {
    RouterTelemetryEvent selector;
    selector.kind = RouterTelemetryEventKind::search_summary;
    selector.minimum_level = RouterTelemetryLevel::diagnostic;
    selector.stable_id = "work-plan-selector:" + expansion.id;
    selector.total_nets = expansion.nets.size();
    selector.total_groups = expansion.footprints.size();
    selector.remaining_alternatives = expansion.wildcard_only_nets.size();
    selector.detail = "Work plan selector expanded: " + expansion.id;
    emit(std::move(selector));
  }

  RouterTelemetryEvent plan_started;
  plan_started.kind = RouterTelemetryEventKind::search_summary;
  plan_started.stable_id = "work-plan:" + result.plan_sha256;
  plan_started.detail = "Work plan execution started";
  emit(std::move(plan_started));

  Board current = source;
  std::optional<Board> incomplete_diagnostic_candidate;
  std::map<std::string, WorkPlanStepExecutionStatus> status_by_id;
  bool halted = false;
  for (const auto& step : plan.steps) {
    WorkPlanStepExecutionResult step_result;
    step_result.id = step.id;
    step_result.kind = step.kind;
    RouterTelemetryEvent step_started;
    step_started.kind = RouterTelemetryEventKind::search_summary;
    step_started.stable_id = "work-plan-step:" + step.id;
    step_started.detail = "Work plan step started: " + step.id + " (" +
                          std::string(work_plan_step_kind_name(step.kind)) + ")";
    emit(std::move(step_started));
    const bool dependency_failed = std::any_of(
        step.depends_on.begin(), step.depends_on.end(), [&status_by_id](const std::string& id) {
          const auto found = status_by_id.find(id);
          return found == status_by_id.end() ||
                 found->second != WorkPlanStepExecutionStatus::completed;
        });
    if (halted || dependency_failed) {
      step_result.status = WorkPlanStepExecutionStatus::skipped;
      step_result.diagnostic = halted ? "work plan halted after an earlier failed step"
                                      : "a required preceding step was not completed";
      status_by_id.emplace(step.id, step_result.status);
      result.steps.push_back(std::move(step_result));
      RouterTelemetryEvent skipped;
      skipped.kind = RouterTelemetryEventKind::search_summary;
      skipped.stable_id = "work-plan-step:" + step.id;
      skipped.detail = "Work plan step skipped: " + step.id;
      emit(std::move(skipped));
      continue;
    }
    if (stop_token.stop_requested()) {
      step_result.status = WorkPlanStepExecutionStatus::cancelled;
      step_result.diagnostic = "work plan cancelled before step execution";
      status_by_id.emplace(step.id, step_result.status);
      result.steps.push_back(std::move(step_result));
      RouterTelemetryEvent cancelled;
      cancelled.kind = RouterTelemetryEventKind::search_summary;
      cancelled.stable_id = "work-plan-step:" + step.id;
      cancelled.detail = "Work plan step cancelled: " + step.id;
      emit(std::move(cancelled));
      halted = true;
      continue;
    }

    if (step.kind == WorkPlanStepKind::validation) {
      step_result.acceptance = acceptance(current);
      step_result.status = step_result.acceptance.exact_internal_acceptance
                               ? WorkPlanStepExecutionStatus::completed
                               : WorkPlanStepExecutionStatus::failed;
      step_result.diagnostic = step_result.acceptance.exact_internal_acceptance
                                   ? "whole-board internal validation passed"
                                   : "whole-board internal validation failed";
    } else if (step.kind == WorkPlanStepKind::placement) {
      auto compiled_placement = compiled_step(compiled, step.id);
      if (!compiled_placement || !compiled_placement->placement) {
        step_result.status = WorkPlanStepExecutionStatus::failed;
        step_result.diagnostic = "compiled placement step is unavailable";
      } else {
        step_result.selected_footprints = compiled_placement->selected_footprints;
        std::set<NetId> active;
        if (plan.mode == WorkPlanMode::preview || plan.mode == WorkPlanMode::new_routing) {
          active = nets_requiring_new_routing(current, compiled_placement->selected_nets);
        } else {
          active.insert(compiled_placement->selected_nets.begin(),
                        compiled_placement->selected_nets.end());
        }
        step_result.selected_nets.assign(active.begin(), active.end());
        if (active.empty()) {
          const std::set<NetId> selected(compiled_placement->selected_nets.begin(),
                                         compiled_placement->selected_nets.end());
          step_result.acceptance = acceptance(current, &selected);
          step_result.status = step_result.acceptance.exact_internal_acceptance
                                   ? WorkPlanStepExecutionStatus::completed
                                   : WorkPlanStepExecutionStatus::failed;
          step_result.diagnostic =
              "selected footprints have no newly routable touching nets; placement preserved";
        } else {
          const auto& placement = *compiled_placement->placement;
          IntegratedOptimizerConfig configuration;
          configuration.global_router = compiled_placement->configuration.planner.global;
          configuration.regional_router = compiled_placement->configuration.planner.regional;
          configuration.detailed_router = compiled_placement->configuration.detailed;
          configuration.global_router.selected_nets.assign(active.begin(), active.end());
          std::erase_if(configuration.global_router.priority_nets,
                        [&active](NetId net) { return !active.contains(net); });
          std::set<NetId> replacement;
          if (configuration.detailed_router.existing_routing_replacement_scope) {
            for (const auto& net :
                 *configuration.detailed_router.existing_routing_replacement_scope) {
              if (active.contains(net)) {
                replacement.insert(net);
              }
            }
          }
          configuration.detailed_router.existing_routing_replacement_scope =
              std::vector<NetId>{replacement.begin(), replacement.end()};
          step_result.replacement_nets.assign(replacement.begin(), replacement.end());
          configuration.move_step = placement.move_step;
          configuration.routing_feedback_margin = placement.routing_feedback_margin;
          configuration.max_passes = placement.max_passes;
          configuration.max_candidate_evaluations = placement.max_candidate_evaluations;
          configuration.max_candidate_evaluation_workers =
              placement.max_candidate_evaluation_workers;
          configuration.max_full_routing_candidates_per_wave =
              placement.max_full_routing_candidates_per_wave;
          configuration.candidate_evaluation_offset = placement.candidate_evaluation_offset;
          configuration.max_routing_guided_targets_per_footprint =
              placement.max_routing_guided_targets_per_footprint;
          configuration.max_swap_partners_per_footprint =
              placement.max_swap_partners_per_footprint;
          configuration.max_group_translation_proposals_per_group =
              placement.max_group_translation_proposals_per_group;
          configuration.max_relative_component_translation_proposals =
              placement.max_relative_component_translation_proposals;
          configuration.max_global_routing_guides = placement.max_global_routing_guides;
          configuration.max_orientation_proposals_per_footprint =
              placement.max_orientation_proposals_per_footprint;
          configuration.full_routing_verification_interval =
              placement.full_routing_verification_interval;
          configuration.max_pareto_checkpoints = placement.max_pareto_checkpoints;
          configuration.allow_diagonal_moves = placement.allow_diagonal_moves;
          configuration.enable_global_placement = placement.enable_global_placement;
          configuration.enable_global_routing_guidance =
              placement.enable_global_routing_guidance;
          configuration.enable_routing_guided_moves = placement.enable_routing_guided_moves;
          configuration.enable_swap_moves = placement.enable_swap_moves;
          configuration.enable_group_moves = placement.enable_group_moves;
          configuration.enable_relative_component_moves =
              placement.enable_relative_component_moves;
          configuration.enable_incremental_routing = placement.enable_incremental_routing;
          configuration.optimize_routed_quality = placement.optimize_routed_quality;
          configuration.allow_tentative_placement_envelopes =
              placement.allow_tentative_placement_envelopes;
          configuration.global_placer.enable_assignment_legalization =
              placement.assignment_global_placement;
          configuration.orientation_scope = compiled_placement->selected_footprints;

          if (placement_priorities != nullptr) {
            configuration.placement_move_priorities = *placement_priorities;
          } else if (placement.suggested_placement_move_priorities) {
            configuration.placement_move_priorities =
                make_suggested_placement_move_priorities(current);
            configuration.suggested_placement_move_priorities = true;
          }

          std::set<Microdegrees> rotation_offsets;
          for (const auto& id : compiled_placement->selected_footprints) {
            const auto footprint = std::find_if(
                current.footprints.begin(), current.footprints.end(),
                [&id](const Footprint& item) { return item.id == id; });
            if (footprint == current.footprints.end()) {
              continue;
            }
            if (placement.orientation_repair) {
              for (const auto target : footprint->orientation_constraints.allowed_rotations) {
                const auto offset = Microdegrees{target.value() - footprint->rotation.value()};
                if (offset != Microdegrees{} && rotation_offsets.insert(offset).second) {
                  configuration.rotation_offsets.push_back(offset);
                }
              }
            }
            if (placement.side_flip_repair) {
              configuration.enable_side_flips =
                  configuration.enable_side_flips ||
                  (footprint->side == BoardSide::front
                       ? footprint->orientation_constraints.allow_back
                       : footprint->orientation_constraints.allow_front);
            }
          }

          const std::set<ObjectId> selected(compiled_placement->selected_footprints.begin(),
                                            compiled_placement->selected_footprints.end());
          auto scoped_source = current;
          for (auto& footprint : scoped_source.footprints) {
            if (!selected.contains(footprint.id)) {
              footprint.locked = true;
            }
          }
          step_result.placement_attempted = true;
          auto optimized = optimize_integrated_placement_routing(
              scoped_source, configuration, stop_token, telemetry);
          step_result.placement_candidate_evaluations = optimized.candidate_evaluations;
          step_result.placement_full_route_evaluations = optimized.full_route_evaluations;
          step_result.placement_moves_accepted = optimized.accepted_moves.size();
          step_result.placement_priority_tiers_unlocked =
              optimized.placement_priority_tiers_unlocked;
          step_result.placement_move_priorities_configured =
              optimized.configured_placement_move_priorities;
          step_result.placement_human_review_required =
              optimized.human_placement_review_required;
          step_result.placement_stop_reason =
              std::string(integrated_stop_reason_name(optimized.stop_reason));
          if (optimized.candidate) {
            restore_footprint_locks(*optimized.candidate, current);
            step_result.acceptance = acceptance(*optimized.candidate, &active);
          } else {
            step_result.acceptance = acceptance(current, &active);
          }
          const bool scope_preserved =
              optimized.candidate && only_selected_footprints_moved(current, *optimized.candidate,
                                                                    selected);
          if (optimized.feasible() && optimized.candidate && scope_preserved &&
              step_result.acceptance.exact_internal_acceptance) {
            current = std::move(*optimized.candidate);
            step_result.status = WorkPlanStepExecutionStatus::completed;
            step_result.diagnostic = optimized.diagnostic;
          } else if (stop_token.stop_requested() ||
                     optimized.stop_reason == IntegratedOptimizerStopReason::cancelled) {
            step_result.status = WorkPlanStepExecutionStatus::cancelled;
            step_result.diagnostic = optimized.diagnostic.empty()
                                         ? "placement step cancelled"
                                         : optimized.diagnostic;
          } else {
            step_result.status = WorkPlanStepExecutionStatus::failed;
            step_result.diagnostic = !scope_preserved && optimized.candidate
                                         ? "placement candidate changed an unselected footprint"
                                         : optimized.diagnostic;
          }
        }
      }
    } else {
      auto route = compiled_step(compiled, step.id);
      if (!route) {
        step_result.status = WorkPlanStepExecutionStatus::failed;
        step_result.diagnostic = "compiled route step is unavailable";
      } else {
        std::set<NetId> active;
        if (plan.mode == WorkPlanMode::preview || plan.mode == WorkPlanMode::new_routing) {
          active = nets_requiring_new_routing(current, route->selected_nets);
        } else {
          active.insert(route->selected_nets.begin(), route->selected_nets.end());
        }
        step_result.selected_nets.assign(active.begin(), active.end());
        if (active.empty()) {
          const std::set<NetId> selected(route->selected_nets.begin(), route->selected_nets.end());
          step_result.acceptance = acceptance(current, &selected);
          step_result.status = step_result.acceptance.exact_internal_acceptance
                                   ? WorkPlanStepExecutionStatus::completed
                                   : WorkPlanStepExecutionStatus::failed;
          step_result.diagnostic = "selected nets require no new routing";
        } else {
          auto configuration = route->configuration;
          configuration.planner.global.selected_nets.assign(active.begin(), active.end());
          std::erase_if(configuration.planner.global.priority_nets,
                        [&active](NetId net) { return !active.contains(net); });
          std::set<NetId> replacement;
          if (configuration.detailed.existing_routing_replacement_scope) {
            for (const auto& net : *configuration.detailed.existing_routing_replacement_scope) {
              if (active.contains(net)) {
                replacement.insert(net);
              }
            }
            configuration.detailed.existing_routing_replacement_scope =
                std::vector<NetId>{replacement.begin(), replacement.end()};
          }
          step_result.replacement_nets.assign(replacement.begin(), replacement.end());
          auto planning_board = current;
          remove_authorized_routing(planning_board, replacement);
          PersistentRoutingSessionConfig session_config;
          session_config.planner = configuration.planner;
          session_config.detailed = configuration.detailed;
          session_config.max_final_feedback_passes = route->max_final_routability_feedback_passes;
          session_config.post_negotiation_sampled_conflict_repair =
              route->post_negotiation_sampled_conflict_repair;
          session_config.max_post_negotiation_sampled_conflict_repair_cones =
              route->max_post_negotiation_sampled_conflict_repair_cones;
          auto session = run_persistent_routing_session(planning_board, current, session_config,
                                                        stop_token, telemetry);
          auto routing = std::move(session.routing);
          auto detailed = std::move(session.detailed);
          step_result.global_unrouted_nets = routing.coarse_global.unrouted_nets.size();
          step_result.global_overflow = routing.coarse_global.total_overflow;
          step_result.global_cost = routing.coarse_global.total_cost;
          step_result.post_negotiation_sampled_conflict_repair_enabled =
              route->post_negotiation_sampled_conflict_repair;
          step_result.post_negotiation_cone_repair_attempts =
              routing.coarse_global.post_negotiation_cone_repair_attempts;
          step_result.post_negotiation_cone_repairs_accepted =
              routing.coarse_global.post_negotiation_cone_repairs_accepted;
          step_result.post_negotiation_cone_repair_nets_attempted =
              routing.coarse_global.post_negotiation_cone_repair_nets_attempted;
          step_result.coarse_polyline_fallback_attempted =
              session.coarse_polyline_fallback_attempted;
          step_result.coarse_polyline_fallback_accepted = session.coarse_polyline_fallback_accepted;
          step_result.global_unrouted_nets = routing.coarse_global.unrouted_nets.size();
          step_result.global_overflow = routing.coarse_global.total_overflow;
          step_result.global_cost = routing.coarse_global.total_cost;
          step_result.final_routability_feedback_attempted =
              detailed.final_routability_feedback_attempted;
          step_result.final_routability_feedback_selected =
              detailed.final_routability_feedback_selected;
          step_result.final_routability_feedback_passes =
              detailed.final_routability_feedback_passes;
          step_result.final_routability_feedback_improvements =
              detailed.final_routability_feedback_improvements;
          step_result.final_routability_diagnoses_consumed =
              detailed.final_routability_diagnoses_consumed;
          const bool final_coarse_topology_complete =
              routing.coarse_global.unrouted_nets.empty() &&
              routing.coarse_global.unaccepted_selected_voronoi_lane_transitions == 0U;
          if (!route->post_negotiation_sampled_conflict_repair && final_coarse_topology_complete &&
              routing.coarse_global.total_overflow == 0U &&
              detailed.stop_reason == DetailedRouterStopReason::copper_violations &&
              !stop_token.stop_requested()) {
            step_result.sampled_fallback_attempted = true;
            auto sampled = configuration;
            sampled.planner.global.use_direct_voronoi_resource_edges = false;
            sampled.planner.global.use_voronoi_allocation_resources = false;
            sampled.planner.global.require_voronoi_topological_transitions = false;
            sampled.planner.global.checkpoint_objective =
                GlobalRouteCheckpointObjective::aggregate_overflow;
            sampled.planner.global.scale_voronoi_history_by_overflow = false;
            sampled.planner.global.order_routing_groups_by_physical_envelope = false;
            sampled.planner.global.negotiate_voronoi_lane_conflicts = false;
            auto sampled_routing =
                plan_routes(planning_board, sampled.planner, stop_token, telemetry);
            auto sampled_detailed =
                materialize_detailed_route(current, sampled_routing, sampled.detailed, stop_token);
            if (sampled_detailed.complete()) {
              routing = std::move(sampled_routing);
              detailed = std::move(sampled_detailed);
              step_result.sampled_fallback_accepted = true;
              step_result.global_unrouted_nets = routing.coarse_global.unrouted_nets.size();
              step_result.global_overflow = routing.coarse_global.total_overflow;
              step_result.global_cost = routing.coarse_global.total_cost;
            }
          }
          if (result.preview && route->retain_incomplete_diagnostic_preview &&
              !routing.coarse_global.complete() && !routing.coarse_global.paths.empty() &&
              !stop_token.stop_requested()) {
            step_result.incomplete_diagnostic_preview_attempted = true;
            step_result.incomplete_diagnostic_preview_paths = routing.coarse_global.paths.size();
            auto preview_global = routing.coarse_global;
            preview_global.stop_reason = GlobalRouterStopReason::complete;
            preview_global.unrouted_nets.clear();
            preview_global.total_overflow = 0U;
            preview_global.diagnostic =
                "incomplete WorkPlan checkpoint awaiting non-authoritative preview materialization";
            auto preview_config = configuration.detailed;
            preview_config.topology_locked_medial_materialization = false;
            preview_config.differential_pair_skew_tuning = false;
            preview_config.differential_pair_skew_tuning_batch_assignment = false;
            auto preview_detailed =
                materialize_detailed_route(current, preview_global, preview_config, stop_token);
            if (preview_detailed.candidate) {
              incomplete_diagnostic_candidate = std::move(*preview_detailed.candidate);
              step_result.incomplete_diagnostic_preview_available = true;
              RouterTelemetryEvent preview_available;
              preview_available.kind = RouterTelemetryEventKind::search_summary;
              preview_available.stable_id = "work-plan-diagnostic-preview:" + step.id;
              preview_available.total_nets = step_result.incomplete_diagnostic_preview_paths;
              preview_available.remaining_alternatives = step_result.global_unrouted_nets;
              preview_available.detail =
                  "Incomplete WorkPlan route retained for diagnostic viewing only";
              emit(std::move(preview_available));
            }
          }
          if (detailed.candidate) {
            step_result.acceptance = acceptance(*detailed.candidate, &active);
          } else {
            step_result.acceptance = acceptance(current, &active);
          }
          if (detailed.complete() && detailed.candidate &&
              step_result.acceptance.exact_internal_acceptance) {
            current = std::move(*detailed.candidate);
            step_result.status = WorkPlanStepExecutionStatus::completed;
            step_result.diagnostic = detailed.diagnostic;
          } else if (route->refillable_zone_fills && detailed.provisional_zone_refill_candidate() &&
                     detailed.candidate && detailed.validation.ok() &&
                     detailed.connectivity.open_connections == 0U && detailed.copper_drc.ok()) {
            current = std::move(*detailed.candidate);
            step_result.provisional_zone_refill_required = true;
            step_result.status = WorkPlanStepExecutionStatus::completed;
            step_result.diagnostic = detailed.diagnostic;
          } else if (stop_token.stop_requested() ||
                     detailed.stop_reason == DetailedRouterStopReason::cancelled) {
            step_result.status = WorkPlanStepExecutionStatus::cancelled;
            step_result.diagnostic =
                detailed.diagnostic.empty() ? "route step cancelled" : detailed.diagnostic;
          } else {
            step_result.status = WorkPlanStepExecutionStatus::failed;
            step_result.diagnostic =
                detailed.diagnostic.empty() ? routing.diagnostic : detailed.diagnostic;
          }
        }
      }
    }
    status_by_id.emplace(step.id, step_result.status);
    halted = step_result.status != WorkPlanStepExecutionStatus::completed;
    RouterTelemetryEvent step_finished;
    step_finished.kind = RouterTelemetryEventKind::search_summary;
    step_finished.stable_id = "work-plan-step:" + step.id;
    step_finished.detail = "Work plan step " +
                           std::string(work_plan_step_execution_status_name(step_result.status)) +
                           ": " + step.id;
    if (step_result.placement_attempted) {
      step_finished.total_groups = step_result.selected_footprints.size();
      step_finished.search_expansions = step_result.placement_candidate_evaluations;
      step_finished.remaining_alternatives = step_result.placement_moves_accepted;
    }
    emit(std::move(step_finished));
    result.steps.push_back(std::move(step_result));
  }

  result.internally_complete =
      result.diagnostics.empty() &&
      std::all_of(result.steps.begin(), result.steps.end(), [](const auto& step) {
        return step.status == WorkPlanStepExecutionStatus::completed;
      });
  if (result.preview && !result.internally_complete && incomplete_diagnostic_candidate) {
    result.candidate = std::move(incomplete_diagnostic_candidate);
    result.candidate_is_incomplete_diagnostic_preview = true;
  } else {
    result.candidate = std::move(current);
  }
  result.final_acceptance = acceptance(*result.candidate);
  result.whole_board_accepted =
      result.internally_complete && result.final_acceptance.exact_internal_acceptance;
  result.zone_refill_required =
      std::any_of(result.steps.begin(), result.steps.end(), [](const auto& step) {
        return step.provisional_zone_refill_required;
      });
  result.candidate_ready_for_zone_refill = result.internally_complete && !result.preview &&
                                           result.zone_refill_required && result.candidate.has_value();
  result.candidate_ready_for_export = result.internally_complete && !result.preview &&
                                      !result.zone_refill_required &&
                                      result.final_acceptance.exact_internal_acceptance;
  RouterTelemetryEvent plan_finished;
  plan_finished.kind = RouterTelemetryEventKind::search_summary;
  plan_finished.stable_id = "work-plan:" + result.plan_sha256;
  plan_finished.detail =
      result.internally_complete ? "Work plan execution completed" : "Work plan execution failed";
  emit(std::move(plan_finished));
  return result;
}

} // namespace

WorkPlanExecutionResult execute_flux_work_plan(const Board& source, const FluxWorkPlan& plan,
                                               std::stop_token stop_token,
                                               RouterTelemetrySink* telemetry) {
  return execute_impl(source, plan, false, stop_token, telemetry, nullptr);
}

WorkPlanExecutionResult execute_loaded_flux_work_plan(const WorkPlanLoadResult& loaded,
                                                      std::stop_token stop_token,
                                                      RouterTelemetrySink* telemetry) {
  if (loaded.ok() && loaded.plan && loaded.policy_board) {
    if (telemetry != nullptr) {
      for (const auto& policy : loaded.plan->policies) {
        RouterTelemetryEvent event;
        event.kind = RouterTelemetryEventKind::search_summary;
        event.minimum_level = RouterTelemetryLevel::diagnostic;
        event.stable_id = "work-plan-policy:" + policy.kind + ":" + policy.sha256;
        event.detail = "Work plan policy applied: " + policy.kind;
        static_cast<void>(telemetry->try_emit(std::move(event)));
      }
    }
    return execute_impl(*loaded.policy_board, *loaded.plan, true, stop_token, telemetry,
                        loaded.placement_move_priorities
                            ? &*loaded.placement_move_priorities
                            : nullptr);
  }
  WorkPlanExecutionResult result;
  result.diagnostics = loaded.diagnostics;
  result.candidate = loaded.policy_board;
  if (result.diagnostics.empty()) {
    append_diagnostic(result, "work_plan_not_prepared",
                      "file-loaded work plan has no exact policy-applied source board", "");
  }
  return result;
}

std::string serialize_flux_work_plan_execution_summary(const WorkPlanExecutionResult& result) {
  const auto acceptance_json = [](const WorkPlanAcceptanceState& acceptance_state) {
    json::object value;
    value["structural_validation_ok"] = acceptance_state.structural_validation_ok;
    value["structural_issue_count"] = acceptance_state.structural_issue_count;
    value["open_connections"] = acceptance_state.open_connections;
    value["unrouted_nets"] = acceptance_state.unrouted_nets;
    value["copper_violations"] = acceptance_state.copper_violations;
    value["exact_internal_acceptance"] = acceptance_state.exact_internal_acceptance;
    return value;
  };
  const auto ids_json = [](const std::vector<NetId>& ids) {
    json::array values;
    values.reserve(ids.size());
    for (const auto& id : ids) {
      values.emplace_back(id.value());
    }
    return values;
  };
  const auto object_ids_json = [](const std::vector<ObjectId>& ids) {
    json::array values;
    values.reserve(ids.size());
    for (const auto& id : ids) {
      values.emplace_back(id.value());
    }
    return values;
  };
  json::object root;
  root["schema"] = "flux_work_plan_execution";
  root["schema_version"] = 1U;
  root["plan_sha256"] = result.plan_sha256;
  root["mode"] = work_plan_mode_name(result.mode);
  root["preview"] = result.preview;
  root["internally_complete"] = result.internally_complete;
  root["whole_board_accepted"] = result.whole_board_accepted;
  root["candidate_ready_for_export"] = result.candidate_ready_for_export;
  root["candidate_ready_for_zone_refill"] = result.candidate_ready_for_zone_refill;
  root["zone_refill_required"] = result.zone_refill_required;
  root["candidate_is_incomplete_diagnostic_preview"] =
      result.candidate_is_incomplete_diagnostic_preview;
  root["zone_refill_run"] = result.zone_refill_run;
  root["zone_refill_passed"] = result.zone_refill_passed;
  root["zone_refill_exit_code"] = result.zone_refill_exit_code;
  root["zone_refill_executable"] = result.zone_refill_executable;
  root["zone_refill_diagnostic"] = result.zone_refill_diagnostic;
  root["external_kicad_drc_run"] = result.external_kicad_drc_run;
  root["external_kicad_drc_passed"] = result.external_kicad_drc_passed;
  root["external_kicad_drc_exit_code"] = result.external_kicad_drc_exit_code;
  root["external_kicad_drc_executable"] = result.external_kicad_drc_executable;
  root["external_kicad_drc_diagnostic"] = result.external_kicad_drc_diagnostic;
  root["final_acceptance"] = acceptance_json(result.final_acceptance);
  json::array steps;
  steps.reserve(result.steps.size());
  for (const auto& step : result.steps) {
    json::object value;
    value["id"] = step.id;
    value["kind"] = work_plan_step_kind_name(step.kind);
    value["status"] = work_plan_step_execution_status_name(step.status);
    value["selected_nets"] = ids_json(step.selected_nets);
    value["selected_footprints"] = object_ids_json(step.selected_footprints);
    value["replacement_nets"] = ids_json(step.replacement_nets);
    value["global_unrouted_nets"] = step.global_unrouted_nets;
    value["global_overflow"] = step.global_overflow;
    value["global_cost"] = step.global_cost;
    value["post_negotiation_sampled_conflict_repair_enabled"] =
        step.post_negotiation_sampled_conflict_repair_enabled;
    value["post_negotiation_cone_repair_attempts"] =
        step.post_negotiation_cone_repair_attempts;
    value["post_negotiation_cone_repairs_accepted"] =
        step.post_negotiation_cone_repairs_accepted;
    value["post_negotiation_cone_repair_nets_attempted"] =
        step.post_negotiation_cone_repair_nets_attempted;
    value["coarse_polyline_fallback_attempted"] = step.coarse_polyline_fallback_attempted;
    value["coarse_polyline_fallback_accepted"] = step.coarse_polyline_fallback_accepted;
    value["sampled_fallback_attempted"] = step.sampled_fallback_attempted;
    value["sampled_fallback_accepted"] = step.sampled_fallback_accepted;
    value["final_routability_feedback_attempted"] =
        step.final_routability_feedback_attempted;
    value["final_routability_feedback_selected"] = step.final_routability_feedback_selected;
    value["final_routability_feedback_passes"] = step.final_routability_feedback_passes;
    value["final_routability_feedback_improvements"] =
        step.final_routability_feedback_improvements;
    value["final_routability_diagnoses_consumed"] =
        step.final_routability_diagnoses_consumed;
    value["provisional_zone_refill_required"] = step.provisional_zone_refill_required;
    value["incomplete_diagnostic_preview_attempted"] =
        step.incomplete_diagnostic_preview_attempted;
    value["incomplete_diagnostic_preview_available"] =
        step.incomplete_diagnostic_preview_available;
    value["incomplete_diagnostic_preview_paths"] =
        step.incomplete_diagnostic_preview_paths;
    value["placement_attempted"] = step.placement_attempted;
    value["placement_candidate_evaluations"] = step.placement_candidate_evaluations;
    value["placement_full_route_evaluations"] = step.placement_full_route_evaluations;
    value["placement_moves_accepted"] = step.placement_moves_accepted;
    value["placement_priority_tiers_unlocked"] = step.placement_priority_tiers_unlocked;
    value["placement_move_priorities_configured"] =
        step.placement_move_priorities_configured;
    value["placement_human_review_required"] = step.placement_human_review_required;
    value["placement_stop_reason"] = step.placement_stop_reason;
    value["acceptance"] = acceptance_json(step.acceptance);
    value["diagnostic"] = step.diagnostic;
    steps.push_back(std::move(value));
  }
  root["steps"] = std::move(steps);
  json::array diagnostics;
  diagnostics.reserve(result.diagnostics.size());
  for (const auto& diagnostic : result.diagnostics) {
    json::object value;
    value["severity"] =
        diagnostic.severity == WorkPlanDiagnosticSeverity::error ? "error" : "warning";
    value["code"] = diagnostic.code;
    value["message"] = diagnostic.message;
    value["json_pointer"] = diagnostic.json_pointer;
    diagnostics.push_back(std::move(value));
  }
  root["diagnostics"] = std::move(diagnostics);
  return json::serialize(root) + '\n';
}

} // namespace flux::io
