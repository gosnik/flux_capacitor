#include "flux/core/routing_engine_policy.hpp"
#include "flux/core/routing_engine.hpp"

#include <cstdlib>
#include <iostream>

namespace {
void require(bool value, const char* message) {
  if (!value) { std::cerr << message << '\n'; std::exit(1); }
}
}
int main() {
  flux::RoutingPlanResult plan;
  plan.stop_reason = flux::RoutingPlanStopReason::complete;
  plan.regional.native_scene_evaluated = true;
  plan.regional.native_scene_complete = true;
  plan.regional.spline_candidate_available = true;
  require(flux::ready_for_materialization(plan, {}), "complete evaluated regional candidate is ready");
  plan.regional.native_scene_complete = false;
  require(!flux::ready_for_materialization(plan, {}), "unevaluated regional evidence must block materialization");
  flux::RoutingPlanResult unknown_plan;
  flux::RoutingPlanResult probed_plan;
  probed_plan.quantized_copper_oracle.evaluated = true;
  probed_plan.quantized_copper_oracle.drc_violations = 4U;
  probed_plan.quantized_copper_oracle.open_connections = 1U;
  require(flux::routing_quality(&probed_plan, nullptr) < flux::routing_quality(&unknown_plan, nullptr),
          "missing physical evidence must not outrank a measured conflict");
  auto improved_plan = probed_plan;
  improved_plan.quantized_copper_oracle.drc_violations = 1U;
  require(flux::routing_quality(&improved_plan, nullptr) < flux::routing_quality(&probed_plan, nullptr),
          "fewer physical violations must improve the common quality");
  auto partial_native = probed_plan;
  partial_native.regional_evaluated = true;
  partial_native.regional.native_scene_evaluated = true;
  partial_native.regional.native_scene_complete = true;
  partial_native.regional.native_scene_conflicts = 0U;
  partial_native.regional.native_scene_total_conflicts = 8U;
  partial_native.regional.native_scene_open_connections = 1U;
  require(!flux::RoutingEngineProgressBudget::strict_physical_improvement(
              flux::routing_quality(&probed_plan, nullptr),
              flux::routing_quality(&partial_native, nullptr), true),
          "zero local contacts cannot earn credit while whole-board conflicts worsen");
  partial_native.regional.native_scene_total_conflicts = 1U;
  require(flux::RoutingEngineProgressBudget::strict_physical_improvement(
              flux::routing_quality(&probed_plan, nullptr),
              flux::routing_quality(&partial_native, nullptr), true),
          "whole-scene conflict reduction with preserved connectivity earns progress");
  partial_native.regional.native_scene_complete = false;
  require(!flux::RoutingEngineProgressBudget::strict_physical_improvement(
              flux::routing_quality(&probed_plan, nullptr),
              flux::routing_quality(&partial_native, nullptr), true),
          "an incomplete native assembly cannot earn physical progress");
  flux::DetailedRouteResult detailed;
  detailed.candidate.emplace();
  detailed.unrounded_non_junction_bends = 16U;
  const auto bent_quality = flux::routing_quality(nullptr, &detailed);
  detailed.unrounded_non_junction_bends = 0U;
  require(flux::routing_quality(nullptr, &detailed) < bent_quality,
          "curve failure must affect quality even when raw DRC is unchanged");
  flux::RoutabilityDiagnosis diagnosis;
  diagnosis.kind = flux::RoutabilityFailureKind::shove_deadlock;
  require(flux::diagnosis_action(diagnosis, true, false) == flux::RoutingEngineAction::xpbd_repair,
          "shove deadlocks select XPBD when enabled");
  diagnosis.kind = flux::RoutabilityFailureKind::packing_capacity_exceeded;
  require(flux::diagnosis_action(diagnosis, false, false) == flux::RoutingEngineAction::usped_global_replan,
          "capacity failures select upstream replanning");
  diagnosis.kind = flux::RoutabilityFailureKind::packing_unresolved;
  require(flux::diagnosis_action(diagnosis, true, true) == flux::RoutingEngineAction::regional_realization,
          "missing packing proof requests bounded regional work, not congestion punishment");
  flux::RoutingPlanResult pending_packing;
  pending_packing.stop_reason = flux::RoutingPlanStopReason::complete;
  pending_packing.regional.stop_reason = flux::RegionalRouterStopReason::no_conflicts;
  pending_packing.regional.source_packing_obligations = 1;
  pending_packing.coarse_global.unresolved_corridor_packing.emplace_back();
  require(!pending_packing.regional.complete() &&
              !flux::ready_for_materialization(pending_packing, {}),
          "integer capacity and no-conflicts labels cannot discharge packing obligations");
  pending_packing.regional.spline_candidate_available = true;
  pending_packing.regional.native_scene_evaluated = true;
  pending_packing.regional.native_scene_complete = false;
  require(!pending_packing.regional.complete(), "partial native evidence cannot prove packing");
  pending_packing.regional.native_scene_complete = true;
  pending_packing.regional.native_scene_total_conflicts = 1;
  require(!pending_packing.regional.complete(), "a scene with a physical collision cannot prove packing");
  pending_packing.regional.native_scene_total_conflicts = 0;
  require(pending_packing.regional.complete() && pending_packing.regional.unresolved_packing_obligations() == 0 &&
              flux::ready_for_materialization(pending_packing, {}),
          "complete legal physical scene discharges coarse packing obligations");
  pending_packing.regional.native_scene_evaluated = false;
  pending_packing.regional.source_packing_obligations = 0;
  pending_packing.regional.packing_obligation_budget_exhausted = true;
  require(!pending_packing.regional.complete(), "truncated obligation storage cannot imply proof");
  flux::RoutingPlanResult scaffold;
  scaffold.stop_reason = flux::RoutingPlanStopReason::complete;
  scaffold.legal_scaffold_complete = true;
  scaffold.paths.emplace_back();
  scaffold.paths.back().legal_scaffold = true;
  require(!flux::ready_for_materialization(scaffold, {}),
          "a scaffold label cannot replace whole-board exact evidence");
  scaffold.quantized_copper_oracle.evaluated = true;
  scaffold.quantized_copper_oracle.all_paths_materialized = true;
  scaffold.quantized_copper_oracle.exact_copper_legal = true;
  require(flux::ready_for_materialization(scaffold, {}),
          "certified scaffold bypasses unrelated regional portfolio preparation");
  scaffold.quantized_copper_oracle.open_connections = 1U;
  require(!flux::ready_for_materialization(scaffold, {}),
          "missing scaffold connections must prevent curved handoff");
  // The standalone experiment scheduler explores every eligible strategy/state before using
  // throughput, then gives an aged strategy a bounded fairness turn.
  const std::vector<flux::RoutingExperimentCandidate> experiments{{1U, "A", true}, {2U, "A", true}};
  flux::RoutingExperimentScheduleState schedule;
  const auto first = flux::select_routing_experiment(experiments, schedule);
  require(first && first->strategy == 1U && first->initial_exploration,
          "untried strategies must be explored in deterministic order");
  schedule.trials.push_back({1U, "A", 10U, 100.0L, 1U, false, false});
  schedule.last_dispatched_sequence = 1U;
  const auto second = flux::select_routing_experiment(experiments, schedule);
  require(second && second->strategy == 2U && second->initial_exploration,
          "fast prior strategy cannot starve initial exploration");
  schedule.trials.push_back({2U, "A", 1U, 1.0L, 2U, false, false});
  schedule.last_dispatched_sequence = 2U;
  const auto timed = flux::select_routing_experiment(experiments, schedule);
  require(timed && timed->strategy == 1U,
          "rolling physical gain per active microsecond must prefer the productive strategy");
  schedule.trials.push_back({1U, "A", 10U, 100.0L, 3U, false, false});
  schedule.last_dispatched_sequence = 4U;
  const auto fair = flux::select_routing_experiment(experiments, schedule);
  require(fair && fair->strategy == 2U,
          "aging must eventually schedule an otherwise lower-gain strategy");
  schedule.trials.push_back({2U, "A", 1U, 0.0L, 5U, false, true});
  schedule.last_dispatched_sequence = 5U;
  const auto suspended = flux::select_routing_experiment(experiments, schedule);
  require(suspended && suspended->strategy == 1U,
          "stagnant state-specific strategy must be suspended");
  const std::vector<flux::RoutingExperimentCandidate> changed_context{{2U, "B", true}};
  const auto reopened = flux::select_routing_experiment(changed_context, schedule);
  require(reopened && reopened->initial_exploration,
          "a changed state key must invalidate only the old suspension");
  flux::RoutingExperimentScheduleState churn;
  churn.trials = {{2U, "old", 10U, 0.0L, 1U, false, false},
                  {1U, "new-1", 10U, 10.0L, 2U, false, false},
                  {1U, "new-2", 10U, 10.0L, 3U, false, false}};
  churn.last_dispatched_sequence = 3U;
  const auto churn_choice = flux::select_routing_experiment(
      {{1U, "new-3", true, 0U}, {2U, "old", true, 1U}}, churn);
  require(churn_choice && churn_choice->strategy == 2U,
          "fresh contexts must not starve an older eligible strategy");
  const auto shared_churn_choice = flux::select_routing_experiment(
      {{1U, "new-3", true, 0U}, {2U, "new-3", true, 1U}}, churn);
  require(shared_churn_choice && shared_churn_choice->strategy == 2U,
          "changing every strategy's context must still preserve strategy fairness");
  require(flux::routing_physical_gain({true, 2U, 4U, 0U}, {true, 1U, 4U, 0U}) > 0.0L &&
              flux::routing_physical_gain({true, 2U, 4U, 0U}, {true, 1U, 5U, 0U}) == 0.0L &&
              flux::routing_physical_gain({false, 2U, 4U, 0U}, {true, 0U, 0U, 0U}) == 0.0L,
          "only complete non-regressing physical evidence may earn scheduling gain");
  return 0;
}
