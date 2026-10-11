#include "flux/core/live_copper_terrain.hpp"
#include "flux/core/geometry.hpp"
#include "flux/core/routing_rules.hpp"

#include <algorithm>
#include <limits>
#include <tuple>

namespace flux {
namespace {

[[nodiscard]] geometry::AxisAlignedBounds expanded_bounds(const std::vector<Point>& points,
                                                           Nanometres radius) {
  auto min_x = points.front().x.value();
  auto max_x = min_x;
  auto min_y = points.front().y.value();
  auto max_y = min_y;
  for (const auto point : points) {
    min_x = std::min(min_x, point.x.value());
    max_x = std::max(max_x, point.x.value());
    min_y = std::min(min_y, point.y.value());
    max_y = std::max(max_y, point.y.value());
  }
  const auto grow = std::max<std::int64_t>(0, radius.value());
  const auto subtract = [](std::int64_t value, std::int64_t amount) {
    return value < std::numeric_limits<std::int64_t>::min() + amount
               ? std::numeric_limits<std::int64_t>::min()
               : value - amount;
  };
  const auto add = [](std::int64_t value, std::int64_t amount) {
    return value > std::numeric_limits<std::int64_t>::max() - amount
               ? std::numeric_limits<std::int64_t>::max()
               : value + amount;
  };
  return {{Nanometres{subtract(min_x, grow)}, Nanometres{subtract(min_y, grow)}},
          {Nanometres{add(max_x, grow)}, Nanometres{add(max_y, grow)}}};
}

[[nodiscard]] std::uint32_t nonzero_pressure(std::uint32_t pressure) {
  return std::max<std::uint32_t>(1U, pressure);
}
[[nodiscard]] Nanometres copper_radius(Nanometres width, Nanometres clearance) {
  const auto value = std::max<std::int64_t>(0, width.value());
  const auto half = value / 2 + value % 2;
  const auto clear = std::max<std::int64_t>(0, clearance.value());
  return Nanometres{half > std::numeric_limits<std::int64_t>::max() - clear
                         ? std::numeric_limits<std::int64_t>::max()
                         : half + clear};
}

} // namespace

LiveCopperTerrainSnapshot make_live_copper_terrain_snapshot(const Board& board, bool provisional,
                                                             std::uint32_t pressure) {
  LiveCopperTerrainSnapshot result;
  result.provisional = provisional;
  const auto feature_pressure = nonzero_pressure(pressure);
  // Use the largest resolved clearance so the snapshot is valid for every later querying net.
  // This is conservative advisory pressure; exact pairwise rules remain in the copper oracle.
  auto clearance = std::max(board.rules.default_clearance, board.rules.minimum_clearance);
  for (const auto& net : board.nets)
    clearance = std::max(clearance, resolve_net_routing_rules(board, net.id).clearance);
  for (const auto& track : board.tracks) {
    result.features.push_back({LiveCopperTerrainKind::segment, track.net, track.layer,
                               expanded_bounds({track.start, track.end}, copper_radius(track.width, clearance)),
                               track.start, track.end,
                               copper_radius(track.width, clearance), true,
                               feature_pressure});
  }
  for (const auto& arc : board.track_arcs) {
    auto extrema = geometry::circular_arc_extrema(arc.start, arc.middle, arc.end);
    extrema.push_back(arc.start);
    extrema.push_back(arc.middle);
    extrema.push_back(arc.end);
    result.features.push_back({LiveCopperTerrainKind::arc, arc.net, arc.layer,
                               expanded_bounds(extrema, copper_radius(arc.width, clearance)),
                               arc.start, arc.end,
                               copper_radius(arc.width, clearance), false,
                               feature_pressure});
  }
  for (const auto& bezier : board.track_beziers) {
    result.features.push_back({LiveCopperTerrainKind::bezier, bezier.net, bezier.layer,
                               expanded_bounds({bezier.start, bezier.control1, bezier.control2,
                                                bezier.end},
                                               copper_radius(bezier.width, clearance)),
                               bezier.start, bezier.end,
                               copper_radius(bezier.width, clearance), false,
                               feature_pressure});
  }
  for (const auto& via : board.vias) {
    const auto bounds = expanded_bounds(
        {via.position}, copper_radius(via.diameter, clearance));
    std::vector<LayerId> layers = via.layers;
    // KiCad through/blind via endpoints describe an inclusive copper-stack span.  Imported
    // explicit layer lists remain valid too, and duplicates are harmlessly removed.
    if (via.layers.size() >= 2U) {
      std::vector<std::pair<std::uint32_t, LayerId>> copper_layers;
      for (const auto& candidate : board.layers)
        if (candidate.kind == LayerKind::copper)
          copper_layers.emplace_back(candidate.stack_order, candidate.id);
      const auto first = std::find_if(copper_layers.begin(), copper_layers.end(),
                                      [&](const auto& value) { return value.second == via.layers.front(); });
      const auto last = std::find_if(copper_layers.begin(), copper_layers.end(),
                                     [&](const auto& value) { return value.second == via.layers.back(); });
      if (first != copper_layers.end() && last != copper_layers.end()) {
        const auto low = std::min(first->first, last->first);
        const auto high = std::max(first->first, last->first);
        layers.clear();
        for (const auto& candidate : copper_layers)
          if (candidate.first >= low && candidate.first <= high)
            layers.push_back(candidate.second);
      }
    }
    std::sort(layers.begin(), layers.end());
    layers.erase(std::unique(layers.begin(), layers.end()), layers.end());
    for (const auto& layer : layers) {
      result.features.push_back({LiveCopperTerrainKind::via, via.net, layer, bounds, via.position,
                                 via.position,
                                 copper_radius(via.diameter, clearance), false,
                                 feature_pressure});
    }
  }
  std::sort(result.features.begin(), result.features.end(), [](const auto& left, const auto& right) {
    return std::tie(left.net, left.layer, left.kind, left.bounds.minimum.x, left.bounds.minimum.y,
                    left.bounds.maximum.x, left.bounds.maximum.y, left.axis_start, left.axis_end,
                    left.radius) <
           std::tie(right.net, right.layer, right.kind, right.bounds.minimum.x, right.bounds.minimum.y,
                    right.bounds.maximum.x, right.bounds.maximum.y, right.axis_start, right.axis_end,
                    right.radius);
  });
  return result;
}

std::vector<NavMeshHydraulicTerrainDeposit>
live_copper_terrain_deposits(const LiveCopperTerrainSnapshot& snapshot, NetId net, LayerId layer) {
  std::vector<NavMeshHydraulicTerrainDeposit> result;
  for (const auto& feature : snapshot.features) {
    if (feature.net == net || feature.layer != layer || feature.pressure == 0U)
      continue;
    result.push_back({feature.bounds, feature.axis_start, feature.axis_end, feature.directional,
                      feature.pressure, 0U, feature.radius});
  }
  return result;
}

} // namespace flux
