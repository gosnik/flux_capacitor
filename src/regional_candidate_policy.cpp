#include "flux/core/regional_candidate_policy.hpp"

#include <boost/multiprecision/cpp_int.hpp>

#include <algorithm>
#include <cstdint>
#include <limits>
#include <map>
#include <set>

namespace flux {
namespace {

using FragmentMap = std::map<RegionalRouteFragmentId, const RegionalRouteFragment*>;

[[nodiscard]] FragmentMap fragment_map(const RegionalRouteResult& regional) {
  FragmentMap result;
  for (const auto& fragment : regional.fragments) result.emplace(fragment.id, &fragment);
  return result;
}

[[nodiscard]] bool is_copper_layer(const Board& board, const LayerId& layer) {
  return std::any_of(board.layers.begin(), board.layers.end(), [&](const Layer& candidate) {
    return candidate.id == layer && candidate.kind == LayerKind::copper;
  });
}

[[nodiscard]] bool is_fixed_terminal_endpoint(const RegionalRouteFragment& fragment,
                                               const bool start) {
  return start ? fragment.begins_at_terminal : fragment.ends_at_terminal;
}

[[nodiscard]] Point endpoint(const RegionalRouteFragment& fragment, const bool start) {
  return start ? fragment.start : fragment.end;
}

// The closed disks may touch at their boundaries. That tangency is representable by the
// clearance-expanded model, so only a strict overlap is a certificate of impossibility.
[[nodiscard]] bool endpoint_envelopes_strictly_overlap(
    const RegionalRouteFragment& first, const Point first_endpoint,
    const RegionalRouteFragment& second, const Point second_endpoint) {
  if (first.width.value() < 0 || second.width.value() < 0 || first.clearance.value() < 0 ||
      second.clearance.value() < 0) return false;
  // Pair clearance is governed by the stricter trace rule. Summing the two per-fragment
  // clearances would manufacture a rejection where each envelope was independently expanded.
  using boost::multiprecision::int256_t;
  const auto radius = int256_t{first.width.value() / 2} + int256_t{second.width.value() / 2} +
      int256_t{std::max(first.clearance.value(), second.clearance.value())};
  const auto dx = int256_t{first_endpoint.x.value()} - int256_t{second_endpoint.x.value()};
  const auto dy = int256_t{first_endpoint.y.value()} - int256_t{second_endpoint.y.value()};
  return dx * dx + dy * dy < radius * radius;
}

[[nodiscard]] std::set<RegionalRouteFragmentId> selected_fragments(
    const RegionalBundleAssignment& assignment) {
  std::set<RegionalRouteFragmentId> result;
  for (const auto index : assignment.selected_templates) {
    if (index < assignment.templates.size()) result.insert(assignment.templates[index].fragment);
  }
  // Partial or pre-realization assignments do not necessarily retain a template selection.
  // Their realized fragment identities are still immutable candidate evidence.
  for (const auto& realization : assignment.spline_realizations) result.insert(realization.fragment);
  return result;
}

[[nodiscard]] std::set<RegionalRouteFragmentId> immovable_terminal_fragments(
    const RegionalBundleAssignment& assignment, const FragmentMap& fragments) {
  std::set<RegionalRouteFragmentId> result;
  for (const auto index : assignment.selected_templates) {
    if (index >= assignment.templates.size()) continue;
    const auto& candidate = assignment.templates[index];
    const auto found = fragments.find(candidate.fragment);
    if (found == fragments.end() || candidate.routing_layer != found->second->layer ||
        candidate.entry_via_choice || candidate.exit_via_choice ||
        candidate.terminal_entry_choice || candidate.terminal_exit_choice)
      continue;
    result.insert(candidate.fragment);
  }
  return result;
}

[[nodiscard]] std::optional<std::int64_t> minimum_packed_width(
    const std::vector<const RegionalRouteFragment*>& fragments) {
  if (fragments.empty()) return std::int64_t{0};
  // This is a deliberately weak, order-independent lower bound. It proves an overfull section
  // without treating one failed lane order as a candidate failure, and remains cheap for large
  // channels. A stronger packing search belongs in the existing bounded lane allocator.
  std::int64_t widths = 0;
  std::int64_t smallest_clearance = std::numeric_limits<std::int64_t>::max();
  for (const auto* fragment : fragments) {
    if (fragment->width.value() <= 0 || fragment->clearance.value() < 0) return std::nullopt;
    if (widths > std::numeric_limits<std::int64_t>::max() - fragment->width.value()) return std::nullopt;
    widths += fragment->width.value();
    smallest_clearance = std::min(smallest_clearance, fragment->clearance.value());
  }
  if (smallest_clearance > (std::numeric_limits<std::int64_t>::max() - widths) / 2) return std::nullopt;
  return widths + 2 * smallest_clearance;
}

[[nodiscard]] std::optional<std::size_t> complete_realization_progress(
    const RegionalBundleAssignment& assignment, const std::set<RegionalRouteFragmentId>& selected) {
  if (selected.empty()) return std::nullopt;
  std::map<RegionalRouteFragmentId, const RegionalSplineRealization*> realization_by_fragment;
  for (const auto& realization : assignment.spline_realizations)
    realization_by_fragment.emplace(realization.fragment, &realization);
  if (realization_by_fragment.size() < selected.size()) return std::nullopt;
  std::size_t legal = 0U;
  for (const auto& fragment : selected) {
    const auto found = realization_by_fragment.find(fragment);
    if (found == realization_by_fragment.end()) return std::nullopt;
    const auto& realization = *found->second;
    legal += realization.mutually_legal && realization.quantized_mutually_legal &&
                     realization.native_copper_legal
                 ? 1U
                 : 0U;
  }
  return legal;
}

[[nodiscard]] std::size_t common_count(const auto& first, const auto& second) {
  std::size_t result = 0U;
  for (const auto& value : first)
    result += std::find(second.begin(), second.end(), value) != second.end() ? 1U : 0U;
  return result;
}

[[nodiscard]] const RegionalCandidateEvaluation* related_evaluation(
    const RegionalCandidateSchedulingState* state,
    const RegionalCandidateScheduleSignature& signature) {
  if (state == nullptr) return nullptr;
  const RegionalCandidateEvaluation* result = nullptr;
  std::size_t best_similarity = 0U;
  // A matching template, via, or immutable lane order identifies shared conflict
  // responsibility. This deliberately remains advisory: no historic result can reject a new
  // topology, and ties retain reducer order rather than wall-clock completion order.
  for (const auto& observation : state->observations) {
    auto similarity = common_count(signature.template_ids, observation.signature.template_ids) +
        common_count(signature.via_choices, observation.signature.via_choices);
    for (const auto& lane_order : signature.lane_orders)
      similarity += std::find(observation.signature.lane_orders.begin(),
                              observation.signature.lane_orders.end(), lane_order) !=
                            observation.signature.lane_orders.end()
                        ? 1U
                        : 0U;
    if (similarity > best_similarity) {
      best_similarity = similarity;
      result = &observation.evaluation;
    }
  }
  return result;
}

} // namespace

bool regional_assignment_is_no_good_certificate(
    const RegionalBundleStopReason stop_reason, const bool shape_sample_budget_exhausted) {
  return !shape_sample_budget_exhausted &&
         stop_reason == RegionalBundleStopReason::candidate_preflight_infeasible;
}

RegionalCandidatePreflight preflight_regional_candidate(const Board& board,
                                                         const RegionalRouteResult& regional,
                                                         const RegionalBundleAssignment& assignment) {
  RegionalCandidatePreflight result;
  const auto fragments = fragment_map(regional);
  const auto selected = selected_fragments(assignment);
  const auto immovable_terminals = immovable_terminal_fragments(assignment, fragments);
  std::vector<const RegionalRouteFragment*> selected_fragment_values;
  for (const auto& id : selected) {
    if (const auto found = fragments.find(id); found != fragments.end())
      selected_fragment_values.push_back(found->second);
  }

  for (std::size_t first_index = 0U; first_index < selected_fragment_values.size(); ++first_index) {
    const auto& first = *selected_fragment_values[first_index];
    if (!immovable_terminals.contains(first.id)) continue;
    if (!is_copper_layer(board, first.layer)) continue;
    for (std::size_t second_index = first_index + 1U; second_index < selected_fragment_values.size(); ++second_index) {
      const auto& second = *selected_fragment_values[second_index];
      if (!immovable_terminals.contains(second.id) || first.id.net == second.id.net ||
          first.layer != second.layer) continue;
      for (const auto first_start : {true, false}) {
        if (!is_fixed_terminal_endpoint(first, first_start)) continue;
        for (const auto second_start : {true, false}) {
          if (!is_fixed_terminal_endpoint(second, second_start) ||
              !endpoint_envelopes_strictly_overlap(first, endpoint(first, first_start), second,
                                                   endpoint(second, second_start))) continue;
          result.infeasibilities.push_back({RegionalCandidateInfeasibilityKind::fixed_endpoint_envelope_overlap,
                                            first.id, second.id, first.layer, std::nullopt, std::nullopt});
        }
      }
    }
  }

  for (const auto& section : assignment.packed_cross_sections) {
      if (!is_copper_layer(board, section.layer)) continue;
      const auto& listed = section.ordered_occupants;
      std::vector<const RegionalRouteFragment*> occupants;
      std::set<NetId> counted_nets;
      for (const auto& id : listed) {
        if (!selected.contains(id)) continue;
        if (const auto found = fragments.find(id); found != fragments.end() &&
            found->second->layer == section.layer && counted_nets.insert(id.net).second)
          occupants.push_back(found->second);
      }
      if (occupants.empty() || section.usable_width.value() < 0) continue;
      const auto packed = minimum_packed_width(occupants);
      if (!packed || *packed <= section.usable_width.value()) continue;
      result.infeasibilities.push_back({RegionalCandidateInfeasibilityKind::cross_section_packing_impossible,
                                        occupants.front()->id, occupants.back()->id, section.layer,
                                        section.channel_id, section.id});
  }
  return result;
}

RegionalCandidateScheduleSignature regional_candidate_schedule_signature(
    const RegionalBundleAssignment& assignment) {
  RegionalCandidateScheduleSignature result;
  for (const auto index : assignment.selected_templates) {
    if (index < assignment.templates.size()) result.template_ids.push_back(assignment.templates[index].id);
  }
  result.via_choices = assignment.selected_via_choices;
  for (const auto& section : assignment.packed_cross_sections)
    result.lane_orders.push_back(section.ordered_occupants);
  return result;
}

void record_regional_candidate_evaluation(RegionalCandidateSchedulingState& state,
                                          RegionalCandidateScheduleSignature signature,
                                          RegionalCandidateEvaluation evaluation) {
  state.observations.push_back({std::move(signature), evaluation});
}

std::vector<RegionalCandidateScheduleEntry> schedule_regional_candidates(
    const Board& board, const RegionalRouteResult& regional,
    const std::span<const RegionalBundleAssignment> assignments,
    const RegionalCandidateSchedulingState* state) {
  std::vector<RegionalCandidateScheduleEntry> result;
  result.reserve(assignments.size());
  for (std::size_t index = 0U; index < assignments.size(); ++index) {
    const auto selected = selected_fragments(assignments[index]);
    result.push_back({index, preflight_regional_candidate(board, regional, assignments[index]),
                      assignments[index].selected_template_conflicts,
                      assignments[index].selected_residual_conflicts,
                      complete_realization_progress(assignments[index], selected)});
  }
  std::stable_sort(result.begin(), result.end(), [&](const auto& first, const auto& second) {
    const auto first_impossible = first.preflight.definitively_infeasible();
    const auto second_impossible = second.preflight.definitively_infeasible();
    if (first_impossible != second_impossible) return !first_impossible;
    const auto first_packing = std::count_if(first.preflight.infeasibilities.begin(), first.preflight.infeasibilities.end(),
        [](const auto& value) { return value.kind == RegionalCandidateInfeasibilityKind::cross_section_packing_impossible; });
    const auto second_packing = std::count_if(second.preflight.infeasibilities.begin(), second.preflight.infeasibilities.end(),
        [](const auto& value) { return value.kind == RegionalCandidateInfeasibilityKind::cross_section_packing_impossible; });
    if (first_packing != second_packing) return first_packing < second_packing;
    if (first.known_residual_conflicts != second.known_residual_conflicts)
      return first.known_residual_conflicts < second.known_residual_conflicts;
    if (first.known_template_conflicts != second.known_template_conflicts)
      return first.known_template_conflicts < second.known_template_conflicts;
    const auto* first_evaluation = related_evaluation(
        state, regional_candidate_schedule_signature(assignments[first.assignment_index]));
    const auto* second_evaluation = related_evaluation(
        state, regional_candidate_schedule_signature(assignments[second.assignment_index]));
    // A previously complete signature cannot improve the remaining regional state. Defer it
    // behind untried candidates, and otherwise prefer the observed lower-conflict continuation.
    if (first_evaluation != nullptr && second_evaluation != nullptr) {
      if (first_evaluation->complete != second_evaluation->complete)
        return !first_evaluation->complete;
      if (first_evaluation->physical_violations != second_evaluation->physical_violations)
        return first_evaluation->physical_violations < second_evaluation->physical_violations;
      if (first_evaluation->conflict_free_realizations != second_evaluation->conflict_free_realizations)
        return first_evaluation->conflict_free_realizations > second_evaluation->conflict_free_realizations;
    } else if (first_evaluation != nullptr || second_evaluation != nullptr) {
      return first_evaluation == nullptr;
    }
    if (first.conflict_free_realization_progress.has_value() !=
        second.conflict_free_realization_progress.has_value())
      return first.conflict_free_realization_progress.has_value();
    if (first.conflict_free_realization_progress &&
        *first.conflict_free_realization_progress != *second.conflict_free_realization_progress)
      return *first.conflict_free_realization_progress > *second.conflict_free_realization_progress;
    return first.assignment_index < second.assignment_index;
  });
  return result;
}

} // namespace flux
