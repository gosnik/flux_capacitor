#include "flux/core/routing_engine_policy.hpp"
#include "flux/core/routing_engine.hpp"

#include <limits>
#include <algorithm>
#include <cmath>
#include <map>
#include <set>
#include <tuple>

namespace flux {
namespace {
constexpr std::size_t unknown = RoutingQuality::unknown_rank;
constexpr std::size_t evidence_unknown = 2U;
}

RoutingQuality routing_quality(const RoutingPlanResult* plan, const DetailedRouteResult* detailed) {
  // Completion of a plan is not completion of copper. Unknown physical evidence
  // cannot improve on measured evidence with the same connectivity.
  RoutingQuality quality{{4U, unknown, unknown, unknown, unknown, unknown, evidence_unknown}};
  if (plan) {
    quality.ranks[0] = plan->quantized_copper_oracle.evaluated ||
                               (plan->regional_evaluated &&
                                (plan->regional.native_scene_evaluated ||
                                 plan->regional.straight_scene_evaluated)) ? 2U : 3U;
    quality.ranks[1] = plan->coarse_global.unrouted_nets.size();
    quality.ranks[5] = plan->coarse_global.total_overflow;
    quality.ranks[6] = plan->regional_evaluated ? 0U : 1U;
    if (plan->quantized_copper_oracle.evaluated) {
      quality.ranks[1] = plan->quantized_copper_oracle.open_connections;
      quality.ranks[2] = plan->quantized_copper_oracle.drc_violations;
      quality.ranks[3] = plan->quantized_copper_oracle.malformed_paths;
    }
    if (plan->regional_evaluated) {
      const auto& regional = plan->regional;
      if (regional.straight_scene_evaluated) {
        quality.ranks[1] = regional.straight_scene_open_connections;
        quality.ranks[2] = regional.straight_scene_drc_violations;
      }
      if (regional.native_scene_evaluated) {
        // Local native-contact counts omit provisional/provisional collisions. Only a complete
        // assembled scene supplies comparable physical evidence against the whole-board oracle.
        quality.ranks[1] = regional.native_scene_complete ? regional.native_scene_open_connections : unknown;
        quality.ranks[2] = regional.native_scene_complete ? regional.native_scene_total_conflicts : unknown;
        quality.ranks[3] = regional.native_scene_complete ? 0U : unknown;
      }
      if (ready_for_materialization(*plan, {})) quality.ranks[0] = 1U;
    }
  }
  if (detailed && detailed->candidate) {
    quality.ranks[0] = detailed->complete() ? 0U : 2U;
    quality.ranks[1] = detailed->connectivity.open_connections;
    quality.ranks[2] = detailed->copper_drc.violation_count();
    quality.ranks[3] = detailed->validation.error_count();
    quality.ranks[4] = detailed->unrounded_non_junction_bends;
    quality.ranks[6] = 0U;
  }
  return quality;
}

RoutingEngineAction diagnosis_action(const RoutabilityDiagnosis& diagnosis, bool xpbd_enabled,
                                     bool erosion_enabled) {
  switch (diagnosis.kind) {
  case RoutabilityFailureKind::packing_unresolved:
    return RoutingEngineAction::regional_realization;
  case RoutabilityFailureKind::shove_deadlock:
    return xpbd_enabled ? RoutingEngineAction::xpbd_repair : RoutingEngineAction::regional_realization;
  case RoutabilityFailureKind::packing_capacity_exceeded:
  case RoutabilityFailureKind::topological_trap:
  case RoutabilityFailureKind::layer_transition_conflict:
    return RoutingEngineAction::usped_global_replan;
  case RoutabilityFailureKind::curvature_infeasible:
  case RoutabilityFailureKind::quantization_clearance:
    return erosion_enabled ? RoutingEngineAction::confined_erosion_repair
                           : RoutingEngineAction::regional_realization;
  default:
    return RoutingEngineAction::feedback_global_replan;
  }
}

bool ready_for_materialization(const RoutingPlanResult& plan,
                               const std::vector<RoutabilityDiagnosis>& diagnoses) {
  if (!plan.complete()) return false;
  if (plan.legal_scaffold_complete && plan.quantized_copper_oracle.feasible() &&
      !plan.paths.empty() && std::all_of(plan.paths.begin(), plan.paths.end(),
          [](const auto& path) { return path.legal_scaffold; })) return true;
  const auto& regional = plan.regional;
  if (regional.native_scene_evaluated && (!regional.native_scene_complete ||
      regional.native_scene_total_conflicts != 0U || regional.native_scene_open_connections != 0U))
    return false;
  if (regional.native_scene_evaluated && regional.native_scene_complete && regional.native_scene_conflicts == 0U &&
      regional.spline_candidate_available) return true;
  // Existing complete straight-source fallback remains legal evidence on simple boards.
  if (regional.straight_scene_evaluated && regional.straight_scene_open_connections == 0U &&
      regional.straight_scene_drc_violations == 0U) return true;
  if (plan.quantized_copper_oracle.feasible() && regional.complete()) return true;
  if (!plan.coarse_global.unresolved_corridor_packing.empty() ||
      plan.coarse_global.corridor_packing_obligation_budget_exhausted)
    return false;
  // A conflict-free regional pass needs no alternate spline scene on a simple board.
  // Materialization still performs the authoritative validation/connectivity/DRC checks.
  return regional.stop_reason == RegionalRouterStopReason::no_conflicts && regional.complete() &&
      std::none_of(diagnoses.begin(), diagnoses.end(),
                   [](const auto& diagnosis) { return !diagnosis.locally_resolved; });
}

long double routing_physical_gain(const RoutingPhysicalEvidence& before,
                                  const RoutingPhysicalEvidence& after) {
  if (!before.complete || !after.complete) return 0.0L;
  // A regression in any physical gate cannot improve scheduling priority.  Coarse global
  // overflow intentionally has no representation here.
  if (after.open_connections > before.open_connections ||
      after.drc_violations > before.drc_violations ||
      after.malformed_paths > before.malformed_paths) return 0.0L;
  return static_cast<long double>(before.open_connections - after.open_connections) * 1'000'000.0L +
         static_cast<long double>(before.drc_violations - after.drc_violations) * 1'000.0L +
         static_cast<long double>(before.malformed_paths - after.malformed_paths);
}

namespace {
std::uint64_t saturating_add(std::uint64_t left, std::uint64_t right) {
  return right > std::numeric_limits<std::uint64_t>::max() - left
      ? std::numeric_limits<std::uint64_t>::max() : left + right;
}

long double finite_gain(long double gain) {
  return std::isfinite(gain) ? gain : 0.0L;
}

long double saturating_add(long double left, long double right) {
  const auto sum = left + right;
  return std::isfinite(sum) ? sum : std::numeric_limits<long double>::max();
}

void append_throughput_sample(RoutingExperimentTrial& trial,
                              RoutingExperimentThroughputSample sample) {
  sample.physical_gain = finite_gain(sample.physical_gain);
  const auto size = std::min(trial.throughput_history_size,
                             routing_experiment_throughput_history_limit);
  if (size < routing_experiment_throughput_history_limit) {
    trial.throughput_history[size] = sample;
    trial.throughput_history_size = size + 1U;
  } else {
    std::move(trial.throughput_history.begin() + 1U, trial.throughput_history.end(),
              trial.throughput_history.begin());
    trial.throughput_history.back() = sample;
  }
}

void append_trial_history(RoutingExperimentTrial& destination,
                          const RoutingExperimentTrial& source) {
  const auto size = std::min(source.throughput_history_size,
                             routing_experiment_throughput_history_limit);
  if (size == 0U) {
    append_throughput_sample(destination, {source.dispatched_sequence, source.active_elapsed_us,
                                           source.physical_gain});
    return;
  }
  for (std::size_t index = 0U; index < size; ++index)
    append_throughput_sample(destination, source.throughput_history[index]);
}
} // namespace

void update_routing_experiment_trial(RoutingExperimentScheduleState& state,
                                     RoutingExperimentTrial trial) {
  trial.physical_gain = finite_gain(trial.physical_gain);
  const auto found = std::find_if(state.trials.begin(), state.trials.end(), [&](const auto& existing) {
    return existing.strategy == trial.strategy && existing.state_key == trial.state_key;
  });
  if (found == state.trials.end()) {
    RoutingExperimentTrial aggregate;
    aggregate.strategy = trial.strategy;
    aggregate.state_key = std::move(trial.state_key);
    aggregate.active_elapsed_us = trial.active_elapsed_us;
    aggregate.physical_gain = trial.physical_gain;
    aggregate.dispatched_sequence = trial.dispatched_sequence;
    aggregate.exhausted = trial.exhausted;
    aggregate.stagnant = trial.stagnant;
    append_trial_history(aggregate, trial);
    state.trials.push_back(std::move(aggregate));
  } else {
    found->active_elapsed_us = saturating_add(found->active_elapsed_us, trial.active_elapsed_us);
    found->physical_gain = saturating_add(finite_gain(found->physical_gain), trial.physical_gain);
    found->dispatched_sequence = std::max(found->dispatched_sequence, trial.dispatched_sequence);
    found->exhausted = found->exhausted || trial.exhausted;
    found->stagnant = found->stagnant || trial.stagnant;
    append_trial_history(*found, trial);
  }
  state.last_dispatched_sequence = std::max(state.last_dispatched_sequence, trial.dispatched_sequence);
}

std::optional<RoutingExperimentSelection>
select_routing_experiment(const std::vector<RoutingExperimentCandidate>& candidates,
                          const RoutingExperimentScheduleState& state) {
  struct Aggregate {
    bool seen{false};
    bool blocked{false};
    std::vector<std::pair<std::uint64_t, std::pair<std::uint64_t, long double>>> samples;
    std::uint64_t last_sequence{0U};
  };
  using Key = std::pair<RoutingExperimentStrategyId, std::string>;
  std::map<Key, Aggregate> aggregates;
  std::map<RoutingExperimentStrategyId, std::uint64_t> strategy_last_dispatch;
  for (const auto& trial : state.trials) {
    auto& aggregate = aggregates[{trial.strategy, trial.state_key}];
    aggregate.seen = true;
    aggregate.blocked = aggregate.blocked || trial.exhausted || trial.stagnant;
    const auto history_size = std::min(trial.throughput_history_size,
                                       routing_experiment_throughput_history_limit);
    if (history_size == 0U) {
      aggregate.samples.push_back({trial.dispatched_sequence,
          {trial.active_elapsed_us, finite_gain(trial.physical_gain)}});
    } else {
      for (std::size_t index = 0U; index < history_size; ++index) {
        const auto& sample = trial.throughput_history[index];
        aggregate.samples.push_back({sample.dispatched_sequence,
                                     {sample.active_elapsed_us, finite_gain(sample.physical_gain)}});
      }
    }
    aggregate.last_sequence = std::max(aggregate.last_sequence, trial.dispatched_sequence);
    strategy_last_dispatch[trial.strategy] =
        std::max(strategy_last_dispatch[trial.strategy], trial.dispatched_sequence);
  }
  std::vector<RoutingExperimentSelection> eligible;
  for (const auto& candidate : candidates) {
    if (!candidate.eligible || candidate.state_key.empty()) continue;
    const auto it = aggregates.find({candidate.strategy, candidate.state_key});
    if (it != aggregates.end() && it->second.blocked) continue;
    RoutingExperimentSelection selection{candidate.strategy, candidate.state_key,
                                         it == aggregates.end() || !it->second.seen,
                                         candidate.diagnostic_priority, 0.0L};
    if (it != aggregates.end()) {
      auto samples = it->second.samples;
      std::sort(samples.begin(), samples.end(), [](const auto& a, const auto& b) { return a.first < b.first; });
      const auto begin = samples.size() > 8U ? samples.size() - 8U : 0U;
      std::uint64_t elapsed = 0U; long double gain = 0.0L;
      for (std::size_t sample = begin; sample < samples.size(); ++sample) {
        elapsed = saturating_add(elapsed, samples[sample].second.first);
        gain = saturating_add(gain, samples[sample].second.second);
      }
      selection.rolling_gain_per_us = gain / static_cast<long double>(std::max<std::uint64_t>(1U, elapsed));
    }
    eligible.push_back(std::move(selection));
  }
  if (eligible.empty()) return std::nullopt;
  // Deduplicate by identity before priority sorting; differing priorities must not leave the
  // same strategy counted twice in the aging window.
  std::sort(eligible.begin(), eligible.end(), [](const auto& a, const auto& b) {
    return std::tie(a.strategy, a.state_key, a.diagnostic_priority) <
           std::tie(b.strategy, b.state_key, b.diagnostic_priority);
  });
  eligible.erase(std::unique(eligible.begin(), eligible.end(), [](const auto& a, const auto& b) {
    return a.strategy == b.strategy && a.state_key == b.state_key;
  }), eligible.end());
  std::sort(eligible.begin(), eligible.end(), [](const auto& a, const auto& b) {
    return std::tie(a.diagnostic_priority, a.strategy, a.state_key) <
           std::tie(b.diagnostic_priority, b.strategy, b.state_key);
  });
  // A stream of changed keys must not perpetually restart initial exploration and starve a
  // still-eligible strategy.  Age is tracked at strategy scope across keys, while individual
  // keys retain their own one-shot initial trial once the finite fairness window allows it.
  std::set<RoutingExperimentStrategyId> strategy_ids;
  for (const auto& item : eligible) strategy_ids.insert(item.strategy);
  const auto strategy_window = std::max<std::uint64_t>(2U, strategy_ids.size());
  const auto strategy_age = [&](const RoutingExperimentSelection& item) {
    const auto found = strategy_last_dispatch.find(item.strategy);
    const auto last = found == strategy_last_dispatch.end() ? 0U : found->second;
    return state.last_dispatched_sequence >= last ? state.last_dispatched_sequence - last : 0U;
  };
  const auto overdue_existing = std::max_element(eligible.begin(), eligible.end(), [&](const auto& a, const auto& b) {
    const auto a_overdue = strategy_last_dispatch.contains(a.strategy) && strategy_age(a) >= strategy_window;
    const auto b_overdue = strategy_last_dispatch.contains(b.strategy) && strategy_age(b) >= strategy_window;
    if (a_overdue != b_overdue) return !a_overdue;
    if (!a_overdue) return false;
    if (strategy_age(a) != strategy_age(b)) return strategy_age(a) < strategy_age(b);
    if (a.rolling_gain_per_us != b.rolling_gain_per_us) return a.rolling_gain_per_us < b.rolling_gain_per_us;
    return std::tie(a.strategy, a.state_key) > std::tie(b.strategy, b.state_key);
  });
  if (overdue_existing != eligible.end() && strategy_last_dispatch.contains(overdue_existing->strategy) &&
      strategy_age(*overdue_existing) >= strategy_window)
    return *overdue_existing;
  const auto initial = std::find_if(eligible.begin(), eligible.end(),
                                    [](const auto& item) { return item.initial_exploration; });
  if (initial != eligible.end()) return *initial;
  const auto age = [&](const RoutingExperimentSelection& item) {
    const auto found = aggregates.find({item.strategy, item.state_key});
    const auto last = found == aggregates.end() ? 0U : found->second.last_sequence;
    return state.last_dispatched_sequence >= last ? state.last_dispatched_sequence - last : 0U;
  };
  const auto fairness_window = static_cast<std::uint64_t>(eligible.size());
  const auto overdue = std::max_element(eligible.begin(), eligible.end(), [&](const auto& a, const auto& b) {
    const auto age_a = age(a), age_b = age(b);
    const auto a_overdue = age_a >= fairness_window, b_overdue = age_b >= fairness_window;
    if (a_overdue != b_overdue) return !a_overdue;
    if (a_overdue && age_a != age_b) return age_a < age_b;
    if (a.rolling_gain_per_us != b.rolling_gain_per_us) return a.rolling_gain_per_us < b.rolling_gain_per_us;
    if (age_a != age_b) return age_a < age_b;
    return std::tie(a.strategy, a.state_key) > std::tie(b.strategy, b.state_key);
  });
  return *overdue;
}
} // namespace flux
