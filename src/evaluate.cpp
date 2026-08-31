#include "flux/core/evaluate.hpp"

#include "flux/core/geometry.hpp"

#include <compare>
#include <limits>
#include <string_view>
#include <tuple>

namespace flux {
namespace {

[[nodiscard]] bool is_routing_issue(std::string_view code) {
  return code.starts_with("track_") || code.starts_with("via_") || code.starts_with("zone_") ||
         code.starts_with("unknown_track_") || code.starts_with("unknown_via_") ||
         code.starts_with("unknown_zone_") ||
         code.find("differential_pair") != std::string_view::npos;
}

} // namespace

std::strong_ordering ObjectiveVector::operator<=>(const ObjectiveVector& rhs) const {
  return std::tie(hard_placement_violations, routing_rule_violations, open_connections,
                  unrouted_nets, via_count, routed_length_nm) <=>
         std::tie(rhs.hard_placement_violations, rhs.routing_rule_violations, rhs.open_connections,
                  rhs.unrouted_nets, rhs.via_count, rhs.routed_length_nm);
}

EvaluationResult evaluate_board(const Board& board) {
  EvaluationResult result;
  result.validation = validate_board(board);
  if (result.validation.ok()) {
    result.connectivity = analyze_connectivity(board);
    result.copper_drc = check_copper_rules(board);
  }

  for (const auto& issue : result.validation.issues) {
    if (issue.severity != IssueSeverity::error) {
      continue;
    }
    if (is_routing_issue(issue.code)) {
      ++result.objective.routing_rule_violations;
    } else {
      ++result.objective.hard_placement_violations;
    }
  }
  result.objective.open_connections = result.connectivity.open_connections;
  result.objective.unrouted_nets = result.connectivity.unrouted_nets;
  result.objective.routing_rule_violations += result.copper_drc.violation_count();
  result.objective.via_count = board.vias.size();
  for (const auto& track : board.tracks) {
    const auto length = geometry::segment_length_nm(track.start, track.end);
    const auto remaining =
        std::numeric_limits<std::uint64_t>::max() - result.objective.routed_length_nm;
    result.objective.routed_length_nm += length > remaining ? remaining : length;
  }
  for (const auto& arc : board.track_arcs) {
    const auto length = geometry::circular_arc_length_nm(arc.start, arc.middle, arc.end);
    const auto remaining =
        std::numeric_limits<std::uint64_t>::max() - result.objective.routed_length_nm;
    result.objective.routed_length_nm += length > remaining ? remaining : length;
  }
  return result;
}

} // namespace flux
