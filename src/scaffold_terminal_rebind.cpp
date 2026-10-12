#include "scaffold_terminal_rebind.hpp"

#include "flux/core/copper_geometry.hpp"
#include "scaffold_access.hpp"
#include "scaffold_connectivity_guard.hpp"

#include <boost/multiprecision/cpp_int.hpp>

#include <algorithm>
#include <iterator>
#include <tuple>

namespace flux {
namespace {

[[nodiscard]] bool point_on_same_layer_segment(const GlobalRoutePoint& point,
                                                const GlobalRoutePoint& first,
                                                const GlobalRoutePoint& second) {
  if (point.layer != first.layer || point.layer != second.layer) return false;
  const auto ax = first.position.x.value(), ay = first.position.y.value();
  const auto bx = second.position.x.value(), by = second.position.y.value();
  const auto px = point.position.x.value(), py = point.position.y.value();
  using boost::multiprecision::cpp_int;
  const cpp_int cross = (cpp_int{bx} - ax) * (cpp_int{py} - ay) -
                        (cpp_int{by} - ay) * (cpp_int{px} - ax);
  return cross == 0 && px >= std::min(ax, bx) && px <= std::max(ax, bx) &&
         py >= std::min(ay, by) && py <= std::max(ay, by);
}

[[nodiscard]] bool endpoint_is_real_pad(const std::vector<CopperPadShape>& pads, const NetId& net,
                                        const GlobalRoutePoint& endpoint) {
  return std::any_of(pads.begin(), pads.end(), [&](const auto& pad) {
    return pad.net == net && std::find(pad.layers.begin(), pad.layers.end(), endpoint.layer) != pad.layers.end() &&
        point_to_pad_copper_distance_nm(endpoint.position, pad) <= 0.0L;
  });
}

[[nodiscard]] bool producer_contains(const GlobalRoutePath& producer, const GlobalRoutePoint& point) {
  for (std::size_t segment = 1U; segment < producer.points.size(); ++segment)
    if (point_on_same_layer_segment(point, producer.points[segment - 1U], producer.points[segment])) return true;
  return false;
}

// The direct discovery path preserves its existing source-loop guard.  The outer checkpoint
// gate below deliberately does not use that guard: the rebuilt demand is already physically
// connected when it is accepted, so the guard would reject the exact candidate it must validate.
[[nodiscard]] std::pair<std::vector<GlobalRoutePath>, std::vector<ScaffoldTerminalRebind>>
rebind_exhausted_scaffold_terminals(const Board& board, const CopperScaffoldResult& retained,
                                    bool require_connected_producer) {
  auto rebound = retained.requests;
  std::vector<ScaffoldTerminalRebind> bindings;
  const auto pads = copper_pad_shapes(board);
  for (const auto index : retained.missing_indices) {
    if (index >= retained.requests.size() || index >= retained.request_evidence.size()) continue;
    const auto& request = retained.requests[index]; const auto& evidence = retained.request_evidence[index];
    if (request.points.size() < 2U || !evidence.access_evaluated || evidence.target_endpoint_contained_by_copper) continue;
    const bool inaccessible = evidence.termination == CopperScaffoldSearchTermination::attachment_exhausted &&
        evidence.target_legal_attachments == 0U;
    const bool exhausted_branch_search = evidence.termination == CopperScaffoldSearchTermination::expansion_budget_exhausted;
    if (!inaccessible && !exhausted_branch_search) continue;
    const auto orphan = request.points.back();
    if (endpoint_is_real_pad(pads, request.net, orphan)) continue;
    std::vector<ScaffoldTerminalRebind> candidates;
    for (std::size_t producer = 0U; producer < retained.requests.size(); ++producer) {
      if (producer == index) continue;
      const auto& path = retained.requests[producer];
      if (path.net != request.net || path.points.size() < 2U) continue;
      if (require_connected_producer || !inaccessible) {
        if (std::find(retained.satisfied_indices.begin(), retained.satisfied_indices.end(), producer) == retained.satisfied_indices.end() ||
            !scaffold_request_connected(retained.copper, path)) continue;
      }
      if (!producer_contains(path, orphan)) continue;
      for (const auto& terminal : {path.points.front(), path.points.back()}) {
        if (!endpoint_is_real_pad(pads, request.net, terminal)) continue;
        GlobalRoutePath component_probe; component_probe.net = request.net;
        component_probe.points = {request.points.front(), terminal};
        if (scaffold_request_connected(retained.copper, component_probe)) continue;
        ScaffoldTerminalRebind candidate;
        candidate.request_index = index; candidate.original_endpoint = orphan; candidate.terminal = terminal;
        candidate.proof = "request=" + std::to_string(index) + "; producer=" + std::to_string(producer) +
            "; net=" + request.net.value() + "; layer=" + orphan.layer.value() + "; orphan=" +
            std::to_string(orphan.position.x.value()) + "," + std::to_string(orphan.position.y.value()) +
            "; terminal=" + std::to_string(terminal.position.x.value()) + "," + std::to_string(terminal.position.y.value());
        candidates.push_back(std::move(candidate));
      }
    }
    std::sort(candidates.begin(), candidates.end(), [](const auto& a, const auto& b) {
      return std::tuple{a.terminal.position.x.value(), a.terminal.position.y.value(), a.terminal.layer, a.proof} <
             std::tuple{b.terminal.position.x.value(), b.terminal.position.y.value(), b.terminal.layer, b.proof};
    });
    if (candidates.empty()) continue;
    rebound[index].points.back() = candidates.front().terminal;
    rebound[index].medial_resource_sequence.clear(); rebound[index].medial_transition_portals.clear();
    rebound[index].medial_transition_grid_edges.clear(); rebound[index].geometric_corridors.clear();
    rebound[index].segment_widths.clear(); rebound[index].segment_width_kinds.clear();
    rebound[index].medial_topology_breaks = 0U;
    rebound[index].medial_direct_portal_transitions = 0U;
    rebound[index].medial_unmapped_nodes = 0U;
    rebound[index].medial_interior_unmapped_nodes = 0U;
    rebound[index].direct_medial_edges = 0U;
    rebound[index].direct_medial_grid_steps_bypassed = 0U;
    rebound[index].terminal_neck_down_segments = 0U;
    rebound[index].terminal_neck_down_length_nm = 0U;
    rebound[index].medial_neck_down_segments = 0U;
    rebound[index].medial_neck_down_length_nm = 0U;
    rebound[index].corridor_neck_down_segments = 0U;
    rebound[index].corridor_neck_down_length_nm = 0U;
    rebound[index].legal_scaffold = false;
    bindings.push_back(std::move(candidates.front()));
  }
  return {std::move(rebound), std::move(bindings)};
}

[[nodiscard]] bool same_rebind_prefix(const GlobalRoutePath& original, const GlobalRoutePath& changed) {
  if (original.net != changed.net || original.connection_index != changed.connection_index ||
      original.points.size() != changed.points.size() || original.points.size() < 2U) return false;
  return std::equal(original.points.begin(), std::prev(original.points.end()), changed.points.begin());
}

[[nodiscard]] bool cleared_rebind_metadata(const GlobalRoutePath& path) {
  return path.medial_resource_sequence.empty() && path.medial_transition_portals.empty() &&
      path.medial_transition_grid_edges.empty() && path.geometric_corridors.empty() &&
      path.segment_widths.empty() && path.segment_width_kinds.empty() && path.medial_topology_breaks == 0U &&
      path.medial_direct_portal_transitions == 0U && path.medial_unmapped_nodes == 0U &&
      path.medial_interior_unmapped_nodes == 0U && path.direct_medial_edges == 0U &&
      path.direct_medial_grid_steps_bypassed == 0U && path.terminal_neck_down_segments == 0U &&
      path.terminal_neck_down_length_nm == 0U && path.medial_neck_down_segments == 0U &&
      path.medial_neck_down_length_nm == 0U && path.corridor_neck_down_segments == 0U &&
      path.corridor_neck_down_length_nm == 0U && !path.legal_scaffold;
}

[[nodiscard]] bool proven_rebind_mapping(const CopperScaffoldResult& candidate,
                                         const CopperScaffoldResult& incumbent,
                                         const GlobalRoutePath& original,
                                         const GlobalRoutePath& changed) {
  const auto pads = copper_pad_shapes(candidate.copper);
  if (!same_rebind_prefix(original, changed) || original.points.back() == changed.points.back() ||
      !cleared_rebind_metadata(changed) || endpoint_is_real_pad(pads, original.net, original.points.back()) ||
      !scaffold_request_connected(candidate.copper, changed)) return false;
  if (!endpoint_is_real_pad(pads, changed.net, changed.points.back())) return false;
  for (const auto& producer : incumbent.requests) {
    if (producer.net != original.net || producer.connection_index == original.connection_index ||
        producer.points.size() < 2U || !producer_contains(producer, original.points.back())) continue;
    const auto candidate_producer = std::find_if(candidate.requests.begin(), candidate.requests.end(), [&](const auto& path) {
      return path.net == producer.net && path.connection_index == producer.connection_index && path.points == producer.points;
    });
    if (candidate_producer == candidate.requests.end() || !scaffold_request_connected(candidate.copper, *candidate_producer)) continue;
    for (const auto& terminal : {producer.points.front(), producer.points.back()})
      if (terminal == changed.points.back() && endpoint_is_real_pad(pads, producer.net, terminal)) return true;
  }
  return false;
}

}  // namespace

std::pair<std::vector<GlobalRoutePath>, std::vector<ScaffoldTerminalRebind>>
derive_scaffold_terminal_rebinds(const Board& board, const CopperScaffoldResult& retained,
                                const std::vector<std::string>& attempted_proofs,
                                const std::string& physical_context, bool require_connected_producer) {
  auto result = rebind_exhausted_scaffold_terminals(board, retained, require_connected_producer);
  for (auto& binding : result.second) binding.proof += "; physical-context=" + physical_context;
  std::erase_if(result.second, [&](const auto& binding) {
    if (std::find(attempted_proofs.begin(), attempted_proofs.end(), binding.proof) == attempted_proofs.end()) return false;
    result.first[binding.request_index] = retained.requests[binding.request_index]; return true;
  });
  return result;
}

bool scaffold_preserves_satisfied_requests(const CopperScaffoldResult& candidate,
                                          const CopperScaffoldResult& incumbent) {
  bool changed_binding = false;
  for (const auto old_index : incumbent.satisfied_indices) {
    if (old_index >= incumbent.requests.size()) return false;
    const auto& original = incumbent.requests[old_index];
    const auto exact = std::find(candidate.requests.begin(), candidate.requests.end(), original);
    if (exact != candidate.requests.end()) {
      const auto exact_index = static_cast<std::size_t>(exact - candidate.requests.begin());
      if (std::find(candidate.satisfied_indices.begin(), candidate.satisfied_indices.end(), exact_index) != candidate.satisfied_indices.end())
        continue;
    }
    const auto found = std::find_if(candidate.requests.begin(), candidate.requests.end(), [&](const auto& request) {
      return proven_rebind_mapping(candidate, incumbent, original, request);
    });
    if (found == candidate.requests.end()) return false;
    const auto new_index = static_cast<std::size_t>(found - candidate.requests.begin());
    if (std::find(candidate.satisfied_indices.begin(), candidate.satisfied_indices.end(), new_index) == candidate.satisfied_indices.end()) return false;
    changed_binding = true;
  }
  if (!changed_binding) return true;
  return preserves_existing_pad_components(analyze_connectivity(incumbent.copper), analyze_connectivity(candidate.copper));
}

}  // namespace flux
