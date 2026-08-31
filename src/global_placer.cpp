#include "flux/core/global_placer.hpp"

#include "flux/core/geometry.hpp"
#include "flux/core/keepout_geometry.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <map>
#include <set>
#include <stop_token>
#include <tuple>
#include <utility>
#include <vector>

namespace flux {
namespace {

struct FloatPoint {
  long double x{0.0L};
  long double y{0.0L};
};

struct NetStats {
  long double x{0.0L};
  long double y{0.0L};
  std::size_t count{0};
};

struct PlacementComponent {
  std::vector<std::size_t> members;
  bool constrained{false};
  bool locked{false};
};

class DisjointSets {
public:
  explicit DisjointSets(std::size_t size) : parents_(size), ranks_(size, 0U) {
    for (std::size_t index = 0U; index < size; ++index) {
      parents_[index] = index;
    }
  }

  [[nodiscard]] std::size_t find(std::size_t index) {
    if (parents_[index] != index) {
      parents_[index] = find(parents_[index]);
    }
    return parents_[index];
  }

  void unite(std::size_t first, std::size_t second) {
    first = find(first);
    second = find(second);
    if (first == second) {
      return;
    }
    if (ranks_[first] < ranks_[second]) {
      std::swap(first, second);
    }
    parents_[second] = first;
    if (ranks_[first] == ranks_[second]) {
      ++ranks_[first];
    }
  }

private:
  std::vector<std::size_t> parents_;
  std::vector<std::size_t> ranks_;
};

[[nodiscard]] std::vector<PlacementComponent> placement_components(const Board& board) {
  DisjointSets sets(board.footprints.size());
  std::map<ObjectId, std::size_t> index_by_id;
  for (std::size_t index = 0U; index < board.footprints.size(); ++index) {
    index_by_id.emplace(board.footprints[index].id, index);
  }
  std::vector<bool> constrained(board.footprints.size(), false);
  for (const auto& group : board.placement_groups) {
    const auto first = index_by_id.at(group.footprints.front());
    constrained[first] = true;
    for (std::size_t member = 1U; member < group.footprints.size(); ++member) {
      const auto index = index_by_id.at(group.footprints[member]);
      constrained[index] = true;
      sets.unite(first, index);
    }
  }
  for (const auto& constraint : board.relative_placement_constraints) {
    const auto first = index_by_id.at(constraint.first_footprint);
    const auto second = index_by_id.at(constraint.second_footprint);
    constrained[first] = true;
    constrained[second] = true;
    sets.unite(first, second);
  }

  std::map<std::size_t, std::vector<std::size_t>> members_by_root;
  for (std::size_t index = 0U; index < board.footprints.size(); ++index) {
    members_by_root[sets.find(index)].push_back(index);
  }
  std::vector<PlacementComponent> result;
  result.reserve(members_by_root.size());
  for (auto& [root, members] : members_by_root) {
    static_cast<void>(root);
    std::sort(members.begin(), members.end(), [&board](std::size_t lhs, std::size_t rhs) {
      return board.footprints[lhs].id < board.footprints[rhs].id;
    });
    const bool has_policy =
        std::any_of(members.begin(), members.end(),
                    [&constrained](std::size_t index) { return constrained[index]; });
    const bool has_lock = std::any_of(members.begin(), members.end(), [&board](std::size_t index) {
      return board.footprints[index].locked;
    });
    result.push_back(PlacementComponent{std::move(members), has_policy, has_lock});
  }
  std::sort(result.begin(), result.end(),
            [&board](const PlacementComponent& lhs, const PlacementComponent& rhs) {
              return board.footprints[lhs.members.front()].id <
                     board.footprints[rhs.members.front()].id;
            });
  return result;
}

[[nodiscard]] std::uint64_t saturating_add(std::uint64_t lhs, std::uint64_t rhs) {
  return rhs > std::numeric_limits<std::uint64_t>::max() - lhs
             ? std::numeric_limits<std::uint64_t>::max()
             : lhs + rhs;
}

[[nodiscard]] bool is_repairable_source_placement_error(const ValidationIssue& issue) {
  if (issue.severity != IssueSeverity::error) {
    return true;
  }
  static const std::set<std::string> repairable_codes{
      "pad_outside_board", "placement_keepout", "placement_outside_board", "placement_overlap"};
  return repairable_codes.contains(issue.code);
}

[[nodiscard]] std::uint64_t bounded_distance(Point first, Point second) {
  return geometry::segment_length_nm(first, second);
}

[[nodiscard]] Point rounded_point(FloatPoint point) {
  return Point{Nanometres{static_cast<std::int64_t>(std::llround(point.x))},
               Nanometres{static_cast<std::int64_t>(std::llround(point.y))}};
}

[[nodiscard]] FloatPoint as_float(Point point) {
  return {static_cast<long double>(point.x.value()), static_cast<long double>(point.y.value())};
}

[[nodiscard]] Point pad_offset(const Footprint& footprint, const Pad& pad) {
  return geometry::transform_point(pad.local_position, Point{}, footprint.rotation, footprint.side);
}

[[nodiscard]] long double courtyard_radius(const Footprint& footprint) {
  long double result = 0.0L;
  for (const auto vertex : footprint.courtyard.vertices) {
    result = std::max(result, std::hypot(static_cast<long double>(vertex.x.value()),
                                         static_cast<long double>(vertex.y.value())));
  }
  return result;
}

[[nodiscard]] LayerId side_layer(BoardSide side) {
  return LayerId{side == BoardSide::front ? "F.Cu" : "B.Cu"};
}

[[nodiscard]] bool keepout_applies_to_side(const Keepout& keepout, BoardSide side) {
  return keepout.prohibit.footprints && std::find(keepout.layers.begin(), keepout.layers.end(),
                                                  side_layer(side)) != keepout.layers.end();
}

[[nodiscard]] Polygon placed_polygon(const Polygon& local, const Footprint& footprint,
                                     Point position) {
  return geometry::transform_polygon(local, position, footprint.rotation, footprint.side);
}

[[nodiscard]] bool placement_geometry_legal(const Board& source, const Board& placement,
                                            std::size_t candidate_index, Point position,
                                            const std::vector<bool>& placed,
                                            bool reserve_unplaced_source = true) {
  const auto& footprint = placement.footprints[candidate_index];
  const auto courtyard = placed_polygon(footprint.courtyard, footprint, position);
  if (!geometry::polygon_inside_board_area(courtyard, source)) {
    return false;
  }
  const auto region =
      std::find_if(source.placement_regions.begin(), source.placement_regions.end(),
                   [&footprint](const PlacementRegion& candidate) {
                     return std::find(candidate.footprints.begin(), candidate.footprints.end(),
                                      footprint.id) != candidate.footprints.end();
                   });
  if (region != source.placement_regions.end() &&
      !geometry::polygon_inside(courtyard, region->area)) {
    return false;
  }
  for (const auto& constraint : source.relative_placement_constraints) {
    const bool candidate_is_first = constraint.first_footprint == footprint.id;
    const bool candidate_is_second = constraint.second_footprint == footprint.id;
    if (!candidate_is_first && !candidate_is_second) {
      continue;
    }
    const auto& other_id =
        candidate_is_first ? constraint.second_footprint : constraint.first_footprint;
    const auto other =
        std::find_if(source.footprints.begin(), source.footprints.end(),
                     [&other_id](const Footprint& candidate) { return candidate.id == other_id; });
    if (other == source.footprints.end()) {
      return false;
    }
    const auto other_index = static_cast<std::size_t>(other - source.footprints.begin());
    const auto other_position =
        placed[other_index] ? placement.footprints[other_index].position : other->position;
    const auto first_position = candidate_is_first ? position : other_position;
    const auto second_position = candidate_is_second ? position : other_position;
    const auto actual = second_position - first_position;
    const auto x_deviation = actual.x.value() - constraint.nominal_offset.x.value();
    const auto y_deviation = actual.y.value() - constraint.nominal_offset.y.value();
    if (x_deviation < -constraint.maximum_x_deviation.value() ||
        x_deviation > constraint.maximum_x_deviation.value() ||
        y_deviation < -constraint.maximum_y_deviation.value() ||
        y_deviation > constraint.maximum_y_deviation.value()) {
      return false;
    }
  }
  for (const auto& pad : footprint.pads) {
    const auto world =
        geometry::transform_point(pad.local_position, position, footprint.rotation, footprint.side);
    if (!geometry::coordinate_in_range(world) ||
        geometry::locate_point_in_board_area(source, world) == geometry::PointLocation::outside) {
      return false;
    }
  }
  for (const auto& keepout : source.keepouts) {
    if (keepout_applies_to_side(keepout, footprint.side) &&
        geometry::polygons_overlap(courtyard, keepout.area)) {
      return false;
    }
  }

  std::vector<Polygon> candidate_keepouts;
  for (const auto& keepout : footprint.keepouts) {
    auto area = placed_polygon(keepout.area, footprint, position);
    if (!geometry::polygon_inside_board_area(area, source)) {
      return false;
    }
    candidate_keepouts.push_back(std::move(area));
  }

  for (std::size_t other_index = 0; other_index < placement.footprints.size(); ++other_index) {
    if (other_index == candidate_index) {
      continue;
    }
    if (!placed[other_index] && !reserve_unplaced_source) {
      continue;
    }
    const auto& other = placement.footprints[other_index];
    const auto other_position =
        placed[other_index] ? other.position : source.footprints[other_index].position;
    const auto other_courtyard = placed_polygon(other.courtyard, other, other_position);
    if (geometry::polygons_overlap(courtyard, other_courtyard)) {
      return false;
    }
    for (const auto& keepout : other.keepouts) {
      if (keepout_applies_to_side(keepout, footprint.side) &&
          geometry::polygons_overlap(courtyard,
                                     placed_polygon(keepout.area, other, other_position))) {
        return false;
      }
    }
    for (std::size_t keepout_index = 0; keepout_index < footprint.keepouts.size();
         ++keepout_index) {
      const auto& keepout = footprint.keepouts[keepout_index];
      if (keepout_applies_to_side(keepout, other.side) &&
          geometry::polygons_overlap(candidate_keepouts[keepout_index], other_courtyard)) {
        return false;
      }
    }
  }
  return true;
}

[[nodiscard]] bool apply_source_component_translation(const Board& source, Board& candidate,
                                                      const PlacementComponent& component,
                                                      Point displacement) {
  for (const auto index : component.members) {
    if (source.footprints[index].locked && displacement != Point{}) {
      return false;
    }
    const auto x = static_cast<long double>(source.footprints[index].position.x.value()) +
                   static_cast<long double>(displacement.x.value());
    const auto y = static_cast<long double>(source.footprints[index].position.y.value()) +
                   static_cast<long double>(displacement.y.value());
    if (x < -static_cast<long double>(max_abs_coordinate_nm) ||
        x > static_cast<long double>(max_abs_coordinate_nm) ||
        y < -static_cast<long double>(max_abs_coordinate_nm) ||
        y > static_cast<long double>(max_abs_coordinate_nm)) {
      return false;
    }
    candidate.footprints[index].position =
        Point{Nanometres{static_cast<std::int64_t>(x)}, Nanometres{static_cast<std::int64_t>(y)}};
  }
  return true;
}

[[nodiscard]] std::optional<Board>
translated_component_candidate(const Board& source, const Board& placement,
                               const PlacementComponent& component, Point displacement) {
  Board candidate = placement;
  if (!apply_source_component_translation(source, candidate, component, displacement)) {
    return std::nullopt;
  }
  if (!validate_board(candidate).ok()) {
    return std::nullopt;
  }
  return candidate;
}

[[nodiscard]] std::vector<Point> legalization_candidates(Point target, Point original,
                                                         const Polygon& outline,
                                                         const GlobalPlacerConfig& config,
                                                         std::size_t maximum_candidates) {
  const auto [min_x, max_x] =
      std::minmax_element(outline.vertices.begin(), outline.vertices.end(),
                          [](Point lhs, Point rhs) { return lhs.x < rhs.x; });
  const auto [min_y, max_y] =
      std::minmax_element(outline.vertices.begin(), outline.vertices.end(),
                          [](Point lhs, Point rhs) { return lhs.y < rhs.y; });
  const auto pitch = config.placement_grid.value();
  const auto snap = [pitch](std::int64_t value, std::int64_t origin) {
    const auto coordinate =
        static_cast<long double>(value - origin) / static_cast<long double>(pitch);
    return origin + static_cast<std::int64_t>(std::llround(coordinate)) * pitch;
  };
  const auto snapped = Point{Nanometres{snap(target.x.value(), min_x->x.value())},
                             Nanometres{snap(target.y.value(), min_y->y.value())}};

  std::vector<Point> result;
  std::set<Point> unique;
  const auto append = [&](Point point) {
    if (point.x >= min_x->x && point.x <= max_x->x && point.y >= min_y->y && point.y <= max_y->y &&
        unique.insert(point).second) {
      result.push_back(point);
    }
  };
  append(target);
  append(snapped);
  for (std::int64_t ring = 1; result.size() < maximum_candidates; ++ring) {
    const auto delta = ring * pitch;
    for (std::int64_t offset = -ring; offset <= ring; ++offset) {
      const auto step = offset * pitch;
      append(Point{Nanometres{snapped.x.value() + step}, Nanometres{snapped.y.value() - delta}});
      append(Point{Nanometres{snapped.x.value() + step}, Nanometres{snapped.y.value() + delta}});
    }
    for (std::int64_t offset = -ring + 1; offset < ring; ++offset) {
      const auto step = offset * pitch;
      append(Point{Nanometres{snapped.x.value() - delta}, Nanometres{snapped.y.value() + step}});
      append(Point{Nanometres{snapped.x.value() + delta}, Nanometres{snapped.y.value() + step}});
    }
    const auto spans_outline =
        delta > max_x->x.value() - min_x->x.value() && delta > max_y->y.value() - min_y->y.value();
    if (spans_outline) {
      break;
    }
  }
  if (unique.insert(original).second) {
    result.push_back(original);
  }
  return result;
}

[[nodiscard]] std::vector<FloatPoint>
analytical_positions(const Board& board, const std::vector<PlacementComponent>& components,
                     const std::vector<GlobalPlacementGuide>& guidance,
                     const GlobalPlacerConfig& config, std::stop_token stop_token,
                     std::size_t& completed_iterations) {
  std::vector<FloatPoint> positions;
  positions.reserve(board.footprints.size());
  for (const auto& footprint : board.footprints) {
    positions.push_back(as_float(footprint.position));
  }
  std::map<NetId, std::uint32_t> priorities;
  for (const auto& net : board.nets) {
    priorities.emplace(net.id, net.priority);
  }
  std::vector<long double> radii;
  radii.reserve(board.footprints.size());
  for (const auto& footprint : board.footprints) {
    radii.push_back(courtyard_radius(footprint));
  }
  std::map<ObjectId, std::size_t> index_by_id;
  for (std::size_t index = 0U; index < board.footprints.size(); ++index) {
    index_by_id.emplace(board.footprints[index].id, index);
  }
  std::vector<std::vector<const GlobalPlacementGuide*>> guidance_by_footprint(
      board.footprints.size());
  for (const auto& guide : guidance) {
    guidance_by_footprint[index_by_id.at(guide.footprint)].push_back(&guide);
  }
  std::vector<bool> immobilized(board.footprints.size(), false);
  for (const auto& component : components) {
    if (!component.locked) {
      continue;
    }
    for (const auto index : component.members) {
      immobilized[index] = true;
    }
  }

  const auto [min_x, max_x] =
      std::minmax_element(board.outline.vertices.begin(), board.outline.vertices.end(),
                          [](Point lhs, Point rhs) { return lhs.x < rhs.x; });
  const auto [min_y, max_y] =
      std::minmax_element(board.outline.vertices.begin(), board.outline.vertices.end(),
                          [](Point lhs, Point rhs) { return lhs.y < rhs.y; });

  for (std::size_t iteration = 0; iteration < config.iterations; ++iteration) {
    if (stop_token.stop_requested()) {
      break;
    }
    std::map<NetId, NetStats> net_stats;
    std::vector<std::map<NetId, NetStats>> own_stats(board.footprints.size());
    for (std::size_t index = 0; index < board.footprints.size(); ++index) {
      const auto& footprint = board.footprints[index];
      for (const auto& pad : footprint.pads) {
        const auto offset = pad_offset(footprint, pad);
        const auto x = positions[index].x + static_cast<long double>(offset.x.value());
        const auto y = positions[index].y + static_cast<long double>(offset.y.value());
        auto& total = net_stats[pad.net];
        total.x += x;
        total.y += y;
        ++total.count;
        auto& own = own_stats[index][pad.net];
        own.x += x;
        own.y += y;
        ++own.count;
      }
    }

    auto proposed = positions;
    for (std::size_t index = 0; index < board.footprints.size(); ++index) {
      const auto& footprint = board.footprints[index];
      if (immobilized[index]) {
        continue;
      }
      long double weighted_x =
          static_cast<long double>(board.footprints[index].position.x.value()) *
          config.source_anchor_weight;
      long double weighted_y =
          static_cast<long double>(board.footprints[index].position.y.value()) *
          config.source_anchor_weight;
      long double total_weight = config.source_anchor_weight;
      for (const auto& pad : footprint.pads) {
        const auto& total = net_stats.at(pad.net);
        const auto& own = own_stats[index].at(pad.net);
        if (total.count <= own.count) {
          continue;
        }
        const auto other_count = static_cast<long double>(total.count - own.count);
        const auto offset = pad_offset(footprint, pad);
        const auto desired_x =
            (total.x - own.x) / other_count - static_cast<long double>(offset.x.value());
        const auto desired_y =
            (total.y - own.y) / other_count - static_cast<long double>(offset.y.value());
        const auto weight = static_cast<long double>(priorities.at(pad.net)) + 1.0L;
        weighted_x += desired_x * weight;
        weighted_y += desired_y * weight;
        total_weight += weight;
      }
      for (const auto* guide : guidance_by_footprint[index]) {
        const auto weight =
            config.routing_guidance_weight * static_cast<long double>(guide->weight);
        weighted_x += static_cast<long double>(guide->target.x.value()) * weight;
        weighted_y += static_cast<long double>(guide->target.y.value()) * weight;
        total_weight += weight;
      }
      if (total_weight > 0.0L) {
        const auto target_x = weighted_x / total_weight;
        const auto target_y = weighted_y / total_weight;
        proposed[index].x += config.attraction_strength * (target_x - positions[index].x);
        proposed[index].y += config.attraction_strength * (target_y - positions[index].y);
      }
    }

    std::vector<FloatPoint> repulsion(board.footprints.size());
    for (std::size_t first = 0; first < board.footprints.size(); ++first) {
      for (std::size_t second = first + 1U; second < board.footprints.size(); ++second) {
        auto dx = proposed[first].x - proposed[second].x;
        auto dy = proposed[first].y - proposed[second].y;
        auto distance = std::hypot(dx, dy);
        const auto required = radii[first] + radii[second];
        if (distance >= required) {
          continue;
        }
        if (distance < 1.0L) {
          dx = ((first + second) % 2U == 0U) ? 1.0L : 0.0L;
          dy = dx == 0.0L ? 1.0L : 0.0L;
          distance = 1.0L;
        }
        const auto push = (required - distance) / distance;
        const bool first_movable = !immobilized[first];
        const bool second_movable = !immobilized[second];
        const auto share = first_movable && second_movable ? 0.5L : 1.0L;
        if (first_movable) {
          repulsion[first].x += dx * push * share;
          repulsion[first].y += dy * push * share;
        }
        if (second_movable) {
          repulsion[second].x -= dx * push * share;
          repulsion[second].y -= dy * push * share;
        }
      }
    }
    for (std::size_t index = 0; index < board.footprints.size(); ++index) {
      if (immobilized[index]) {
        continue;
      }
      proposed[index].x += config.density_strength * repulsion[index].x;
      proposed[index].y += config.density_strength * repulsion[index].y;
      proposed[index].x = std::clamp(proposed[index].x, static_cast<long double>(min_x->x.value()),
                                     static_cast<long double>(max_x->x.value()));
      proposed[index].y = std::clamp(proposed[index].y, static_cast<long double>(min_y->y.value()),
                                     static_cast<long double>(max_y->y.value()));
    }
    for (const auto& component : components) {
      if (!component.constrained) {
        continue;
      }
      if (component.locked) {
        for (const auto index : component.members) {
          proposed[index] = positions[index];
        }
        continue;
      }
      FloatPoint mean_delta;
      for (const auto index : component.members) {
        mean_delta.x += proposed[index].x - positions[index].x;
        mean_delta.y += proposed[index].y - positions[index].y;
      }
      const auto count = static_cast<long double>(component.members.size());
      mean_delta.x /= count;
      mean_delta.y /= count;
      long double minimum_x_delta = -std::numeric_limits<long double>::infinity();
      long double maximum_x_delta = std::numeric_limits<long double>::infinity();
      long double minimum_y_delta = -std::numeric_limits<long double>::infinity();
      long double maximum_y_delta = std::numeric_limits<long double>::infinity();
      for (const auto index : component.members) {
        minimum_x_delta = std::max(minimum_x_delta,
                                   static_cast<long double>(min_x->x.value()) - positions[index].x);
        maximum_x_delta = std::min(maximum_x_delta,
                                   static_cast<long double>(max_x->x.value()) - positions[index].x);
        minimum_y_delta = std::max(minimum_y_delta,
                                   static_cast<long double>(min_y->y.value()) - positions[index].y);
        maximum_y_delta = std::min(maximum_y_delta,
                                   static_cast<long double>(max_y->y.value()) - positions[index].y);
      }
      mean_delta.x = std::clamp(mean_delta.x, minimum_x_delta, maximum_x_delta);
      mean_delta.y = std::clamp(mean_delta.y, minimum_y_delta, maximum_y_delta);
      for (const auto index : component.members) {
        proposed[index].x = positions[index].x + mean_delta.x;
        proposed[index].y = positions[index].y + mean_delta.y;
      }
    }
    positions = std::move(proposed);
    ++completed_iterations;
  }
  return positions;
}

[[nodiscard]] std::uint64_t placement_displacement(const Board& source, const Board& candidate) {
  std::uint64_t result = 0;
  for (std::size_t index = 0; index < source.footprints.size(); ++index) {
    result = saturating_add(result, bounded_distance(source.footprints[index].position,
                                                     candidate.footprints[index].position));
  }
  return result;
}

struct AssignmentScore {
  std::uint64_t target_distance_nm{0};
  std::uint64_t source_displacement_nm{0};
};

[[nodiscard]] bool assignment_score_less(const AssignmentScore& lhs, const AssignmentScore& rhs) {
  return std::tie(lhs.target_distance_nm, lhs.source_displacement_nm) <
         std::tie(rhs.target_distance_nm, rhs.source_displacement_nm);
}

[[nodiscard]] AssignmentScore add_assignment_score(AssignmentScore lhs, AssignmentScore rhs) {
  lhs.target_distance_nm = saturating_add(lhs.target_distance_nm, rhs.target_distance_nm);
  lhs.source_displacement_nm =
      saturating_add(lhs.source_displacement_nm, rhs.source_displacement_nm);
  return lhs;
}

[[nodiscard]] AssignmentScore
component_assignment_score(const Board& source, const PlacementComponent& component,
                           Point leader_position, const std::vector<Point>& rounded_targets) {
  AssignmentScore result;
  const auto leader = component.members.front();
  const auto displacement = leader_position - source.footprints[leader].position;
  for (const auto index : component.members) {
    const auto position = source.footprints[index].position + displacement;
    result.target_distance_nm = saturating_add(result.target_distance_nm,
                                               bounded_distance(position, rounded_targets[index]));
    result.source_displacement_nm =
        saturating_add(result.source_displacement_nm,
                       bounded_distance(source.footprints[index].position, position));
  }
  return result;
}

[[nodiscard]] AssignmentScore
placement_assignment_score(const Board& source, const Board& candidate,
                           const std::vector<Point>& rounded_targets) {
  AssignmentScore result;
  for (std::size_t index = 0U; index < source.footprints.size(); ++index) {
    result.target_distance_nm = saturating_add(
        result.target_distance_nm,
        bounded_distance(candidate.footprints[index].position, rounded_targets[index]));
    result.source_displacement_nm = saturating_add(
        result.source_displacement_nm,
        bounded_distance(source.footprints[index].position, candidate.footprints[index].position));
  }
  return result;
}

[[nodiscard]] bool placement_positions_less(const Board& lhs, const Board& rhs) {
  std::vector<std::size_t> order(lhs.footprints.size());
  for (std::size_t index = 0U; index < order.size(); ++index) {
    order[index] = index;
  }
  std::sort(order.begin(), order.end(), [&lhs](std::size_t first, std::size_t second) {
    return lhs.footprints[first].id < lhs.footprints[second].id;
  });
  for (const auto index : order) {
    if (lhs.footprints[index].position != rhs.footprints[index].position) {
      return lhs.footprints[index].position < rhs.footprints[index].position;
    }
  }
  return false;
}

[[nodiscard]] Point component_target_position(const Board& source,
                                              const PlacementComponent& component,
                                              const std::vector<FloatPoint>& targets) {
  const auto leader = component.members.front();
  FloatPoint mean_displacement;
  for (const auto index : component.members) {
    mean_displacement.x +=
        targets[index].x - static_cast<long double>(source.footprints[index].position.x.value());
    mean_displacement.y +=
        targets[index].y - static_cast<long double>(source.footprints[index].position.y.value());
  }
  const auto member_count = static_cast<long double>(component.members.size());
  mean_displacement.x /= member_count;
  mean_displacement.y /= member_count;
  return rounded_point(FloatPoint{
      static_cast<long double>(source.footprints[leader].position.x.value()) + mean_displacement.x,
      static_cast<long double>(source.footprints[leader].position.y.value()) +
          mean_displacement.y});
}

[[nodiscard]] bool apply_partial_component_assignment(const Board& source,
                                                      const PlacementComponent& component,
                                                      Point leader_position, Board& partial,
                                                      std::vector<bool>& assigned) {
  const auto leader = component.members.front();
  if (!apply_source_component_translation(source, partial, component,
                                          leader_position - source.footprints[leader].position)) {
    return false;
  }
  for (const auto index : component.members) {
    assigned[index] = true;
  }
  for (const auto index : component.members) {
    if (!placement_geometry_legal(source, partial, index, partial.footprints[index].position,
                                  assigned, false)) {
      return false;
    }
  }
  return true;
}

struct AssignmentCandidate {
  Point leader_position;
  AssignmentScore score;
};

struct AssignmentSearchResult {
  Board candidate;
  AssignmentScore score;
  std::size_t candidate_edges{0};
  std::size_t search_states{0};
  bool budget_exhausted{false};
  bool cancelled{false};
};

[[nodiscard]] AssignmentSearchResult
search_component_assignment(const Board& source, const Board& greedy_candidate,
                            const std::vector<PlacementComponent>& components,
                            const std::vector<std::size_t>& order,
                            const std::vector<FloatPoint>& targets,
                            const GlobalPlacerConfig& config, std::stop_token stop_token) {
  std::vector<Point> rounded_targets;
  rounded_targets.reserve(targets.size());
  for (const auto target : targets) {
    rounded_targets.push_back(rounded_point(target));
  }

  AssignmentSearchResult result;
  result.candidate = greedy_candidate;
  result.score = placement_assignment_score(source, greedy_candidate, rounded_targets);

  std::vector<bool> fixed_assigned(source.footprints.size(), false);
  for (const auto& component : components) {
    if (component.locked) {
      for (const auto index : component.members) {
        fixed_assigned[index] = true;
      }
    }
  }

  std::vector<std::vector<AssignmentCandidate>> candidates_by_depth;
  candidates_by_depth.reserve(order.size());
  for (const auto component_index : order) {
    const auto& component = components[component_index];
    const auto leader = component.members.front();
    const auto original = source.footprints[leader].position;
    const auto target = component_target_position(source, component, targets);
    std::vector<Point> positions;
    std::set<Point> unique;
    const auto append = [&](Point position) {
      if (positions.size() < config.max_assignment_candidates_per_component &&
          unique.insert(position).second) {
        positions.push_back(position);
      }
    };
    append(target);
    for (const auto site_component_index : order) {
      append(source.footprints[components[site_component_index].members.front()].position);
    }
    for (const auto position :
         legalization_candidates(target, original, source.outline, config,
                                 config.max_assignment_candidates_per_component)) {
      append(position);
    }
    if (!unique.contains(original)) {
      if (positions.size() == config.max_assignment_candidates_per_component) {
        unique.erase(positions.back());
        positions.pop_back();
      }
      unique.insert(original);
      positions.push_back(original);
    }

    std::vector<AssignmentCandidate> candidates;
    for (const auto position : positions) {
      Board partial = source;
      auto assigned = fixed_assigned;
      if (apply_partial_component_assignment(source, component, position, partial, assigned)) {
        candidates.push_back(AssignmentCandidate{
            position, component_assignment_score(source, component, position, rounded_targets)});
      }
    }
    std::sort(candidates.begin(), candidates.end(),
              [](const AssignmentCandidate& lhs, const AssignmentCandidate& rhs) {
                if (assignment_score_less(lhs.score, rhs.score)) {
                  return true;
                }
                if (assignment_score_less(rhs.score, lhs.score)) {
                  return false;
                }
                return lhs.leader_position < rhs.leader_position;
              });
    result.candidate_edges += candidates.size();
    candidates_by_depth.push_back(std::move(candidates));
  }

  std::vector<AssignmentScore> suffix_lower_bound(order.size() + 1U);
  for (std::size_t depth = order.size(); depth > 0U; --depth) {
    if (candidates_by_depth[depth - 1U].empty()) {
      suffix_lower_bound[depth - 1U] = suffix_lower_bound[depth];
    } else {
      suffix_lower_bound[depth - 1U] = add_assignment_score(
          candidates_by_depth[depth - 1U].front().score, suffix_lower_bound[depth]);
    }
  }

  const auto search = [&](const auto& recurse, std::size_t depth, const Board& partial,
                          const std::vector<bool>& assigned, AssignmentScore score) -> void {
    if (result.cancelled || result.budget_exhausted) {
      return;
    }
    if (stop_token.stop_requested()) {
      result.cancelled = true;
      return;
    }
    if (depth == order.size()) {
      if (validate_board(partial).ok() && (assignment_score_less(score, result.score) ||
                                           (!assignment_score_less(result.score, score) &&
                                            placement_positions_less(partial, result.candidate)))) {
        result.candidate = partial;
        result.score = score;
      }
      return;
    }
    const auto optimistic = add_assignment_score(score, suffix_lower_bound[depth]);
    if (assignment_score_less(result.score, optimistic)) {
      return;
    }
    const auto& component = components[order[depth]];
    for (const auto& candidate : candidates_by_depth[depth]) {
      if (result.search_states >= config.max_assignment_search_states) {
        result.budget_exhausted = true;
        return;
      }
      ++result.search_states;
      Board next = partial;
      auto next_assigned = assigned;
      if (!apply_partial_component_assignment(source, component, candidate.leader_position, next,
                                              next_assigned)) {
        continue;
      }
      const auto next_score = add_assignment_score(score, candidate.score);
      const auto lower_bound = add_assignment_score(next_score, suffix_lower_bound[depth + 1U]);
      if (assignment_score_less(result.score, lower_bound)) {
        continue;
      }
      recurse(recurse, depth + 1U, next, next_assigned, next_score);
      if (result.cancelled || result.budget_exhausted) {
        return;
      }
    }
  };

  search(search, 0U, source, fixed_assigned, {});
  return result;
}

} // namespace

std::uint64_t placement_hpwl_nm(const Board& board) {
  struct Bounds {
    std::int64_t min_x{0};
    std::int64_t max_x{0};
    std::int64_t min_y{0};
    std::int64_t max_y{0};
    bool initialized{false};
  };
  std::map<NetId, Bounds> bounds;
  for (const auto& footprint : board.footprints) {
    for (const auto& pad : footprint.pads) {
      const auto position = geometry::transform_point(pad.local_position, footprint.position,
                                                      footprint.rotation, footprint.side);
      auto& net = bounds[pad.net];
      if (!net.initialized) {
        net.min_x = net.max_x = position.x.value();
        net.min_y = net.max_y = position.y.value();
        net.initialized = true;
      } else {
        net.min_x = std::min(net.min_x, position.x.value());
        net.max_x = std::max(net.max_x, position.x.value());
        net.min_y = std::min(net.min_y, position.y.value());
        net.max_y = std::max(net.max_y, position.y.value());
      }
    }
  }
  std::uint64_t result = 0;
  for (const auto& [net, value] : bounds) {
    static_cast<void>(net);
    const auto span = static_cast<std::uint64_t>(value.max_x - value.min_x) +
                      static_cast<std::uint64_t>(value.max_y - value.min_y);
    result = saturating_add(result, span);
  }
  return result;
}

GlobalPlacementResult place_global(const Board& source, const GlobalPlacerConfig& config,
                                   std::stop_token stop_token,
                                   const std::vector<GlobalPlacementGuide>& guidance) {
  GlobalPlacementResult result;
  result.before.hpwl_nm = placement_hpwl_nm(source);
  auto pair_factor_a = source.footprints.size();
  auto pair_factor_b = pair_factor_a == 0U ? 0U : pair_factor_a - 1U;
  if (pair_factor_a % 2U == 0U) {
    pair_factor_a /= 2U;
  } else {
    pair_factor_b /= 2U;
  }
  const bool pairwise_problem_too_large =
      pair_factor_b != 0U && pair_factor_a > config.max_pairwise_checks / pair_factor_b;
  if (config.iterations == 0U || !std::isfinite(config.attraction_strength) ||
      !std::isfinite(config.density_strength) || !std::isfinite(config.source_anchor_weight) ||
      !std::isfinite(config.routing_guidance_weight) || config.attraction_strength <= 0.0L ||
      config.attraction_strength > 1.0L || config.density_strength < 0.0L ||
      config.source_anchor_weight < 0.0L || config.routing_guidance_weight < 0.0L ||
      (!guidance.empty() && config.routing_guidance_weight <= 0.0L) ||
      config.placement_grid.value() <= 0 ||
      config.max_legalization_candidates_per_footprint == 0U || pairwise_problem_too_large) {
    result.diagnostic = "global placer configuration or pairwise problem size is invalid";
    return result;
  }
  if (config.enable_assignment_legalization &&
      (config.max_assignment_components == 0U ||
       config.max_assignment_candidates_per_component == 0U ||
       config.max_assignment_search_states == 0U)) {
    result.diagnostic = "assignment legalization requires positive component, candidate, and "
                        "search-state bounds";
    return result;
  }
  result.validation = validate_board(source);
  if (!result.validation.ok()) {
    const auto repairable_errors = static_cast<std::size_t>(
        std::count_if(result.validation.issues.begin(), result.validation.issues.end(),
                      [](const auto& issue) { return issue.severity == IssueSeverity::error; }));
    const bool all_errors_repairable =
        std::all_of(result.validation.issues.begin(), result.validation.issues.end(),
                    is_repairable_source_placement_error);
    const bool has_tentative_envelope = std::any_of(
        source.footprints.begin(), source.footprints.end(),
        [](const Footprint& footprint) { return footprint.has_tentative_placement_envelope(); });
    if (!config.allow_tentative_placement_envelopes || !has_tentative_envelope ||
        !all_errors_repairable) {
      result.diagnostic = "global placement requires a structurally legal source placement";
      return result;
    }
    result.repairable_tentative_source_accepted = true;
    result.repairable_tentative_source_errors = repairable_errors;
  }
  if (std::any_of(source.footprints.begin(), source.footprints.end(),
                  [&config](const Footprint& footprint) {
                    return !footprint.has_exact_placement_envelope() &&
                           !(config.allow_tentative_placement_envelopes &&
                             footprint.has_tentative_placement_envelope());
                  })) {
    result.diagnostic =
        "global placement requires an authoritative envelope or explicit tentative envelope "
        "opt-in for every footprint";
    return result;
  }
  if (stop_token.stop_requested()) {
    result.stop_reason = GlobalPlacerStopReason::cancelled;
    result.diagnostic = "global placement cancelled before optimization";
    return result;
  }

  const auto components = placement_components(source);
  std::map<ObjectId, std::size_t> footprint_index_by_id;
  for (std::size_t index = 0U; index < source.footprints.size(); ++index) {
    footprint_index_by_id.emplace(source.footprints[index].id, index);
  }
  std::vector<bool> immobilized(source.footprints.size(), false);
  for (const auto& component : components) {
    if (component.locked) {
      for (const auto index : component.members) {
        immobilized[index] = true;
      }
    }
    if (!component.constrained) {
      continue;
    }
    ++result.constraint_components;
    if (component.locked) {
      ++result.immobilized_constraint_components;
    }
  }
  std::set<std::pair<ObjectId, Point>> unique_guides;
  std::set<ObjectId> guided_footprints;
  for (const auto& guide : guidance) {
    const auto found = footprint_index_by_id.find(guide.footprint);
    if (guide.weight == 0U || found == footprint_index_by_id.end() ||
        !geometry::coordinate_in_range(guide.target) || immobilized[found->second] ||
        !unique_guides.emplace(guide.footprint, guide.target).second) {
      result.diagnostic =
          "global placement guidance is empty-weighted, unknown, out of range, immobilized, or "
          "duplicated";
      return result;
    }
    guided_footprints.insert(guide.footprint);
  }
  result.routing_guides = guidance.size();
  result.routing_guided_footprints = guided_footprints.size();
  const auto targets =
      analytical_positions(source, components, guidance, config, stop_token, result.iterations);
  if (stop_token.stop_requested()) {
    result.stop_reason = GlobalPlacerStopReason::cancelled;
    result.diagnostic = "global placement cancelled during analytical optimization";
    return result;
  }
  std::vector<Point> rounded_targets;
  rounded_targets.reserve(targets.size());
  for (const auto target : targets) {
    rounded_targets.push_back(rounded_point(target));
  }
  result.before.analytical_target_distance_nm =
      placement_assignment_score(source, source, rounded_targets).target_distance_nm;

  Board candidate = source;
  std::vector<bool> placed(source.footprints.size(), false);
  std::vector<std::size_t> order;
  order.reserve(components.size());
  for (std::size_t component_index = 0U; component_index < components.size(); ++component_index) {
    const auto& component = components[component_index];
    if (component.locked) {
      for (const auto index : component.members) {
        placed[index] = true;
      }
    } else {
      order.push_back(component_index);
    }
  }
  const auto component_area = [&source, &components](std::size_t component_index) {
    long double area = 0.0L;
    for (const auto index : components[component_index].members) {
      area += std::abs(static_cast<long double>(
          geometry::signed_double_area(source.footprints[index].courtyard)));
    }
    return area;
  };
  std::sort(order.begin(), order.end(),
            [&source, &components, &component_area](std::size_t lhs, std::size_t rhs) {
              const auto lhs_area = component_area(lhs);
              const auto rhs_area = component_area(rhs);
              if (lhs_area != rhs_area) {
                return lhs_area > rhs_area;
              }
              return source.footprints[components[lhs].members.front()].id <
                     source.footprints[components[rhs].members.front()].id;
            });

  for (const auto component_index : order) {
    if (stop_token.stop_requested()) {
      result.stop_reason = GlobalPlacerStopReason::cancelled;
      result.diagnostic = "global placement cancelled during legalization";
      return result;
    }
    const auto& component = components[component_index];
    const auto leader = component.members.front();
    const auto target = component_target_position(source, component, targets);
    bool found = false;
    for (const auto position :
         legalization_candidates(target, source.footprints[leader].position, source.outline, config,
                                 config.max_legalization_candidates_per_footprint)) {
      ++result.legalization_attempts;
      if (component.constrained) {
        const auto translated = translated_component_candidate(
            source, candidate, component, position - source.footprints[leader].position);
        if (!translated) {
          continue;
        }
        candidate = std::move(*translated);
      } else {
        if (!placement_geometry_legal(source, candidate, leader, position, placed)) {
          continue;
        }
        candidate.footprints[leader].position = position;
      }
      for (const auto index : component.members) {
        placed[index] = true;
      }
      found = true;
      break;
    }
    if (!found) {
      result.stop_reason = GlobalPlacerStopReason::infeasible;
      result.diagnostic = "deterministic legalizer could not place every unlocked component";
      return result;
    }
    if (component.constrained) {
      ++result.atomically_legalized_components;
    }
  }

  if (config.enable_assignment_legalization) {
    result.assignment_components = order.size();
    if (order.size() > config.max_assignment_components) {
      result.assignment_legalization_withheld = true;
    } else {
      result.assignment_legalization_attempted = true;
      auto assignment = search_component_assignment(source, candidate, components, order, targets,
                                                    config, stop_token);
      result.assignment_candidate_edges = assignment.candidate_edges;
      result.assignment_search_states = assignment.search_states;
      result.assignment_search_budget_exhausted = assignment.budget_exhausted;
      if (assignment.cancelled || stop_token.stop_requested()) {
        result.stop_reason = GlobalPlacerStopReason::cancelled;
        result.diagnostic = "global placement cancelled during simultaneous legalization";
        return result;
      }
      if (assignment.candidate.footprints != candidate.footprints) {
        candidate = std::move(assignment.candidate);
        result.assignment_legalization_accepted = true;
      }
    }
  }

  result.validation = validate_board(candidate);
  if (!result.validation.ok()) {
    result.stop_reason = GlobalPlacerStopReason::infeasible;
    result.diagnostic = "global placement proposal failed exact canonical legalization";
    return result;
  }
  for (std::size_t index = 0; index < source.footprints.size(); ++index) {
    if (source.footprints[index].position != candidate.footprints[index].position) {
      ++result.moved_footprints;
    }
  }
  for (const auto& component : components) {
    if (std::any_of(component.members.begin(), component.members.end(),
                    [&source, &candidate](std::size_t index) {
                      return source.footprints[index].position !=
                             candidate.footprints[index].position;
                    })) {
      ++result.moved_components;
    }
  }
  result.after.hpwl_nm = placement_hpwl_nm(candidate);
  result.after.displacement_nm = placement_displacement(source, candidate);
  result.after.analytical_target_distance_nm =
      placement_assignment_score(source, candidate, rounded_targets).target_distance_nm;
  result.candidate = std::move(candidate);
  result.stop_reason = GlobalPlacerStopReason::complete;
  result.diagnostic = result.assignment_legalization_accepted
                          ? "analytical placement proposal passed bounded simultaneous exact "
                            "component legalization"
                          : "analytical placement proposal passed deterministic exact component "
                            "legalization";
  return result;
}

} // namespace flux
