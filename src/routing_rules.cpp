#include "flux/core/routing_rules.hpp"

#include <algorithm>
#include <cstdint>
#include <limits>
#include <map>

namespace flux {

ResolvedNetRoutingRules resolve_net_routing_rules(const Board& board, const NetId& net) {
  ResolvedNetRoutingRules result{board.rules.default_clearance,
                                 board.rules.default_track_width,
                                 board.rules.default_via_diameter,
                                 board.rules.default_via_drill,
                                 board.rules.default_differential_pair_track_width,
                                 board.rules.default_differential_pair_gap,
                                 board.rules.default_differential_pair_via_gap};
  const auto found_net = std::find_if(board.nets.begin(), board.nets.end(),
                                      [&net](const Net& candidate) { return candidate.id == net; });
  std::map<std::uint32_t, const NetClass*> assigned;
  if (found_net != board.nets.end()) {
    for (const auto& class_id : found_net->classes) {
      const auto found_class =
          std::find_if(board.net_classes.begin(), board.net_classes.end(),
                       [&class_id](const NetClass& candidate) { return candidate.id == class_id; });
      if (found_class != board.net_classes.end()) {
        assigned.emplace(found_class->priority, &*found_class);
      }
    }
  }
  for (const auto& [priority, net_class] : assigned) {
    static_cast<void>(priority);
    if (net_class->clearance) {
      result.clearance = *net_class->clearance;
    }
    if (net_class->track_width) {
      result.track_width = *net_class->track_width;
    }
    if (net_class->via_diameter) {
      result.via_diameter = *net_class->via_diameter;
    }
    if (net_class->via_drill) {
      result.via_drill = *net_class->via_drill;
    }
    if (net_class->differential_pair_track_width) {
      result.differential_pair_track_width = *net_class->differential_pair_track_width;
    }
    if (net_class->differential_pair_gap) {
      result.differential_pair_gap = *net_class->differential_pair_gap;
    }
    if (net_class->differential_pair_via_gap) {
      result.differential_pair_via_gap = *net_class->differential_pair_via_gap;
    }
  }
  const bool in_differential_pair =
      std::any_of(board.differential_pairs.begin(), board.differential_pairs.end(),
                  [&net](const DifferentialPair& pair) {
                    return pair.positive_net == net || pair.negative_net == net;
                  });
  if (in_differential_pair) {
    result.track_width = result.differential_pair_track_width;
  }
  result.clearance = std::max(result.clearance, board.rules.minimum_clearance);
  result.track_width = std::max(result.track_width, board.rules.minimum_track_width);
  result.differential_pair_track_width =
      std::max(result.differential_pair_track_width, board.rules.minimum_track_width);
  result.via_drill = std::max(result.via_drill, board.rules.minimum_through_drill);
  result.via_diameter = std::max(result.via_diameter, board.rules.minimum_via_diameter);
  const auto maximum = std::numeric_limits<std::int64_t>::max();
  const auto drill = result.via_drill.value();
  const auto annular = board.rules.minimum_via_annular_width.value();
  const auto annular_diameter =
      annular > (maximum - drill) / 2 ? maximum : drill + annular * 2;
  result.via_diameter = std::max(result.via_diameter, Nanometres{annular_diameter});
  return result;
}

Nanometres resolve_pairwise_clearance(const Board& board, const NetId& lhs, const NetId& rhs) {
  return std::max(resolve_net_routing_rules(board, lhs).clearance,
                  resolve_net_routing_rules(board, rhs).clearance);
}

} // namespace flux
