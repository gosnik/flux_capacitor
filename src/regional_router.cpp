#include "flux/core/regional_router.hpp"

#include "flux/core/board_free_space.hpp"
#include "flux/core/copper_geometry.hpp"
#include "flux/core/drc.hpp"
#include "flux/core/free_space_offset.hpp"
#include "flux/core/keepout_geometry.hpp"
#include "flux/core/router_telemetry.hpp"
#include "flux/core/routing_rules.hpp"
#include "flux/core/validation.hpp"

#include <algorithm>
#include <bit>
#include <cmath>
#include <functional>
#include <iterator>
#include <limits>
#include <map>
#include <memory>
#include <numeric>
#include <optional>
#include <set>
#include <string>
#include <string_view>
#include <tuple>
#include <unordered_map>
#include <utility>

namespace flux {
namespace {

[[nodiscard]] geometry::AxisAlignedBounds segment_bounds(Point start, Point end) {
  return geometry::AxisAlignedBounds{Point{std::min(start.x, end.x), std::min(start.y, end.y)},
                                     Point{std::max(start.x, end.x), std::max(start.y, end.y)}};
}

[[nodiscard]] bool proper_regional_segment_intersection(Point first_start, Point first_end,
                                                        Point second_start, Point second_end) {
  const auto orientation = [](Point start, Point end, Point point) {
    return static_cast<long double>(end.x.value() - start.x.value()) *
               static_cast<long double>(point.y.value() - start.y.value()) -
           static_cast<long double>(end.y.value() - start.y.value()) *
               static_cast<long double>(point.x.value() - start.x.value());
  };
  const auto first_a = orientation(first_start, first_end, second_start);
  const auto first_b = orientation(first_start, first_end, second_end);
  const auto second_a = orientation(second_start, second_end, first_start);
  const auto second_b = orientation(second_start, second_end, first_end);
  return ((first_a < 0.0L && first_b > 0.0L) || (first_a > 0.0L && first_b < 0.0L)) &&
         ((second_a < 0.0L && second_b > 0.0L) || (second_a > 0.0L && second_b < 0.0L));
}

[[nodiscard]] geometry::AxisAlignedBounds union_bounds(const geometry::AxisAlignedBounds& lhs,
                                                       const geometry::AxisAlignedBounds& rhs) {
  return geometry::AxisAlignedBounds{
      Point{std::min(lhs.minimum.x, rhs.minimum.x), std::min(lhs.minimum.y, rhs.minimum.y)},
      Point{std::max(lhs.maximum.x, rhs.maximum.x), std::max(lhs.maximum.y, rhs.maximum.y)}};
}

[[nodiscard]] bool bounds_touch(const geometry::AxisAlignedBounds& lhs,
                                const geometry::AxisAlignedBounds& rhs) {
  constexpr std::int64_t guard = 1;
  const auto separated = [](Nanometres first_maximum, Nanometres second_minimum) {
    return first_maximum.value() < second_minimum.value() - guard;
  };
  return !separated(lhs.maximum.x, rhs.minimum.x) && !separated(rhs.maximum.x, lhs.minimum.x) &&
         !separated(lhs.maximum.y, rhs.minimum.y) && !separated(rhs.maximum.y, lhs.minimum.y);
}

[[nodiscard]] bool coupled_net(const Board& board, const NetId& net) {
  if (std::any_of(board.differential_pairs.begin(), board.differential_pairs.end(),
                  [&net](const auto& pair) {
                    return pair.positive_net == net || pair.negative_net == net;
                  })) {
    return true;
  }
  return std::any_of(board.routing_buses.begin(), board.routing_buses.end(),
                     [&net](const auto& bus) {
                       return std::find(bus.ordered_nets.begin(), bus.ordered_nets.end(), net) !=
                              bus.ordered_nets.end();
                     });
}

[[nodiscard]] std::vector<RegionalRouteFragment> make_fragments(const Board& board,
                                                                const GlobalRouteResult& global) {
  std::vector<RegionalRouteFragment> result;
  const auto collinear_forward = [](Point first, Point middle, Point last) {
    const auto first_x = static_cast<long double>(middle.x.value() - first.x.value());
    const auto first_y = static_cast<long double>(middle.y.value() - first.y.value());
    const auto second_x = static_cast<long double>(last.x.value() - middle.x.value());
    const auto second_y = static_cast<long double>(last.y.value() - middle.y.value());
    return first_x * second_y == first_y * second_x &&
           first_x * second_x + first_y * second_y > 0.0L;
  };
  for (const auto& path : global.paths) {
    if (path.points.size() < 2U) {
      continue;
    }
    const auto rules = resolve_net_routing_rules(board, path.net);
    std::optional<std::size_t> first_planar_edge;
    std::optional<std::size_t> last_planar_edge;
    for (std::size_t point_index = 1U; point_index < path.points.size(); ++point_index) {
      const auto& first = path.points[point_index - 1U];
      const auto& second = path.points[point_index];
      if (first.layer == second.layer && first.position != second.position) {
        first_planar_edge = first_planar_edge.value_or(point_index);
        last_planar_edge = point_index;
      }
    }
    if (!first_planar_edge || !last_planar_edge) {
      continue;
    }
    std::size_t run_index = 0U;
    std::size_t fragment_index = 0U;
    for (std::size_t point_index = 1U; point_index < path.points.size(); ++point_index) {
      const auto& first = path.points[point_index - 1U];
      const auto& second = path.points[point_index];
      if (first.layer != second.layer) {
        ++run_index;
        fragment_index = 0U;
        continue;
      }
      if (first.position == second.position) {
        continue;
      }
      auto width = rules.track_width;
      bool explicit_width = false;
      auto width_kind = GlobalRouteSegmentWidthKind::nominal;
      if (path.segment_widths.size() + 1U == path.points.size() &&
          path.segment_widths[point_index - 1U].value() > 0) {
        width = path.segment_widths[point_index - 1U];
        explicit_width = true;
        if (path.segment_width_kinds.size() == path.segment_widths.size()) {
          width_kind = path.segment_width_kinds[point_index - 1U];
        }
      }
      if (!result.empty()) {
        auto& prior = result.back();
        if (prior.id.net == path.net && prior.id.connection_index == path.connection_index &&
            prior.id.run_index == run_index && prior.layer == first.layer &&
            prior.end == first.position && prior.width == width &&
            prior.explicit_width == explicit_width && prior.width_kind == width_kind &&
            collinear_forward(prior.start, prior.end, second.position)) {
          prior.end = second.position;
          prior.bounds =
              union_bounds(prior.bounds, segment_bounds(first.position, second.position));
          prior.ends_at_terminal = point_index == *last_planar_edge;
          continue;
        }
      }
      result.push_back(RegionalRouteFragment{
          RegionalRouteFragmentId{path.net, path.connection_index, run_index, fragment_index++},
          first.layer, first.position, second.position, width, rules.clearance,
          segment_bounds(first.position, second.position), point_index == *first_planar_edge,
          point_index == *last_planar_edge, coupled_net(board, path.net), explicit_width,
          width_kind});
    }
  }
  std::sort(result.begin(), result.end(),
            [](const auto& lhs, const auto& rhs) { return lhs.id < rhs.id; });
  return result;
}

struct ConflictReference {
  Point start;
  Point end;
  LayerId layer;
};

[[nodiscard]] std::optional<std::size_t>
nearest_fragment(const std::vector<RegionalRouteFragment>& fragments, const NetId& net,
                 const ConflictReference& reference) {
  const auto nearest = [&](bool require_layer) {
    std::optional<std::size_t> best;
    auto best_distance = std::numeric_limits<long double>::infinity();
    for (std::size_t index = 0U; index < fragments.size(); ++index) {
      const auto& fragment = fragments[index];
      if (fragment.id.net != net || (require_layer && fragment.layer != reference.layer)) {
        continue;
      }
      const auto distance = geometry::segment_distance_nm(fragment.start, fragment.end,
                                                          reference.start, reference.end);
      if (!best || distance < best_distance ||
          (distance == best_distance && fragment.id < fragments[*best].id)) {
        best = index;
        best_distance = distance;
      }
    }
    return best;
  };
  if (const auto same_layer = nearest(true)) {
    return same_layer;
  }
  // A through-via shadow can be reported on a spanned layer on which its path has no planar
  // fragment. Attribute it to the nearest attached run so the regional via variable owns the
  // conflict instead of losing otherwise exact physical evidence at the graph boundary.
  return nearest(false);
}

struct ConflictSample {
  RegionalRouteFragmentId first;
  RegionalRouteFragmentId second;
  LayerId layer;
  geometry::AxisAlignedBounds bounds;
  Nanometres deficit;
  std::uint64_t observed_length_nm{0};
  std::uint8_t evidence_mask{0};
  std::size_t discrete_excess{0};
  std::uint64_t physical_excess_nm{0};
  std::vector<ObjectId> attributed_obstacles;
};

constexpr std::uint8_t clearance_shadow_evidence = 1U;
constexpr std::uint8_t sampled_capacity_evidence = 2U;
constexpr std::uint8_t corridor_capacity_evidence = 4U;
constexpr std::uint8_t physical_capacity_evidence = 8U;

[[nodiscard]] std::uint64_t observed_length(const ConflictReference& first,
                                            const ConflictReference& second) {
  const auto first_length = geometry::segment_length_nm(first.start, first.end);
  const auto second_length = geometry::segment_length_nm(second.start, second.end);
  const auto selected = std::min(first_length, second_length);
  return std::max(std::uint64_t{1}, selected);
}

[[nodiscard]] bool valid_config(const RegionalRouterConfig& config) {
  return config.window_margin.value() >= 0 && config.max_conflict_fragments_per_window > 0U &&
         config.max_nets_per_window > 0U && config.max_templates_per_fragment > 0U &&
         config.max_terminal_choices_per_terminal > 0U &&
         config.max_via_regions_per_fragment > 0U && config.max_via_sites_per_region > 0U &&
         config.max_selected_via_site_candidate_checks > 0U &&
         config.max_selected_via_site_checks_per_region > 0U &&
         config.max_access_assignment_states > 0U && config.max_access_blockers_per_window > 0U &&
         config.max_layer_sequences_per_fragment > 0U && config.max_bundle_search_states > 0U &&
         config.max_lane_order_states > 0U && config.max_spline_sweeps > 0U &&
         config.max_exact_transactions > 0U && config.navmesh_cell_pitch.value() > 0 &&
         config.max_navmesh_cells_per_template > 0U &&
         config.max_navmesh_expansions_per_template > 0U &&
         config.max_alternative_template_checks > 0U && config.max_joint_repair_windows > 0U &&
         config.max_joint_repair_fragments_per_window > 0U &&
         config.max_joint_alternative_template_checks > 0U &&
         config.max_joint_templates_per_fragment > config.max_templates_per_fragment &&
         config.max_joint_via_site_ring > 1U && config.max_local_layer_bypass_checks > 0U &&
         config.max_local_layer_bypasses_per_fragment > 0U &&
         config.max_local_layer_bypasses_per_fragment < config.max_joint_templates_per_fragment &&
         config.max_joint_spline_candidates > 0U && config.max_local_routability_diagnoses > 0U &&
         config.max_spline_portfolio_windows >= config.max_joint_repair_windows &&
         config.max_same_layer_detour_checks > 0U &&
         config.max_adaptive_pair_shove_checks_per_window > 0U &&
         config.max_quantization_feedback_iterations > 0U &&
         config.max_quantized_segment_pair_checks_per_window > 0U &&
         config.max_residual_conflict_cone_windows > 1U &&
         config.max_residual_retry_bundle_assignments > 0U;
}

void populate_components(RegionalRouteResult& result) {
  std::map<RegionalRouteFragmentId, RegionalRouteFragmentId> parent;
  const auto find_root = [&parent](RegionalRouteFragmentId id) {
    auto root = id;
    while (parent.at(root) != root) {
      root = parent.at(root);
    }
    return root;
  };
  for (const auto& edge : result.conflict_edges) {
    parent.try_emplace(edge.first, edge.first);
    parent.try_emplace(edge.second, edge.second);
  }
  for (const auto& edge : result.conflict_edges) {
    const auto first = find_root(edge.first);
    const auto second = find_root(edge.second);
    if (first != second) {
      parent[std::max(first, second)] = std::min(first, second);
    }
  }
  std::map<RegionalRouteFragmentId, RegionalConflictComponent> components;
  for (const auto& [fragment, unused] : parent) {
    static_cast<void>(unused);
    components[find_root(fragment)].fragments.push_back(fragment);
  }
  for (const auto& edge : result.conflict_edges) {
    components.at(find_root(edge.first)).conflict_edges.push_back(edge.id);
  }
  result.components.clear();
  result.components.reserve(components.size());
  for (auto& [root, component] : components) {
    static_cast<void>(root);
    std::set<NetId> nets;
    for (const auto& fragment : component.fragments) {
      nets.insert(fragment.net);
    }
    component.nets.assign(nets.begin(), nets.end());
    component.id = result.components.size();
    result.components.push_back(std::move(component));
  }
}

[[nodiscard]] Nanometres bounded_add(Nanometres lhs, Nanometres rhs) {
  if (rhs.value() > 0 && lhs.value() > std::numeric_limits<std::int64_t>::max() - rhs.value()) {
    return Nanometres{std::numeric_limits<std::int64_t>::max()};
  }
  return Nanometres{lhs.value() + rhs.value()};
}

[[nodiscard]] geometry::AxisAlignedBounds
expanded_bounds(const geometry::AxisAlignedBounds& bounds, Nanometres margin,
                const geometry::AxisAlignedBounds& board_bounds) {
  const auto subtract = [](Nanometres value, Nanometres amount) {
    return value.value() < std::numeric_limits<std::int64_t>::min() + amount.value()
               ? Nanometres{std::numeric_limits<std::int64_t>::min()}
               : Nanometres{value.value() - amount.value()};
  };
  const auto add = [](Nanometres value, Nanometres amount) {
    return value.value() > std::numeric_limits<std::int64_t>::max() - amount.value()
               ? Nanometres{std::numeric_limits<std::int64_t>::max()}
               : Nanometres{value.value() + amount.value()};
  };
  return geometry::AxisAlignedBounds{
      Point{std::max(board_bounds.minimum.x, subtract(bounds.minimum.x, margin)),
            std::max(board_bounds.minimum.y, subtract(bounds.minimum.y, margin))},
      Point{std::min(board_bounds.maximum.x, add(bounds.maximum.x, margin)),
            std::min(board_bounds.maximum.y, add(bounds.maximum.y, margin))}};
}

struct ClippedSegment {
  long double first_parameter{0.0L};
  long double second_parameter{1.0L};
};

[[nodiscard]] std::optional<ClippedSegment>
clip_segment_to_bounds(Point start, Point end, const geometry::AxisAlignedBounds& bounds) {
  const auto start_x = static_cast<long double>(start.x.value());
  const auto start_y = static_cast<long double>(start.y.value());
  const auto dx = static_cast<long double>(end.x.value() - start.x.value());
  const auto dy = static_cast<long double>(end.y.value() - start.y.value());
  long double first = 0.0L;
  long double second = 1.0L;
  const auto clip = [&first, &second](long double p, long double q) {
    if (p == 0.0L) {
      return q >= 0.0L;
    }
    const auto parameter = q / p;
    if (p < 0.0L) {
      if (parameter > second) {
        return false;
      }
      first = std::max(first, parameter);
    } else {
      if (parameter < first) {
        return false;
      }
      second = std::min(second, parameter);
    }
    return true;
  };
  if (!clip(-dx, start_x - static_cast<long double>(bounds.minimum.x.value())) ||
      !clip(dx, static_cast<long double>(bounds.maximum.x.value()) - start_x) ||
      !clip(-dy, start_y - static_cast<long double>(bounds.minimum.y.value())) ||
      !clip(dy, static_cast<long double>(bounds.maximum.y.value()) - start_y) || first > second) {
    return std::nullopt;
  }
  return ClippedSegment{first, second};
}

[[nodiscard]] Point interpolate_quantized(Point start, Point end, long double parameter) {
  const auto x = static_cast<long double>(start.x.value()) +
                 parameter * static_cast<long double>(end.x.value() - start.x.value());
  const auto y = static_cast<long double>(start.y.value()) +
                 parameter * static_cast<long double>(end.y.value() - start.y.value());
  return Point{Nanometres{static_cast<std::int64_t>(std::llround(x))},
               Nanometres{static_cast<std::int64_t>(std::llround(y))}};
}

[[nodiscard]] long double point_bounds_distance_nm(Point point,
                                                   const geometry::AxisAlignedBounds& bounds) {
  const auto dx = point.x < bounds.minimum.x
                      ? static_cast<long double>(bounds.minimum.x.value() - point.x.value())
                  : point.x > bounds.maximum.x
                      ? static_cast<long double>(point.x.value() - bounds.maximum.x.value())
                      : 0.0L;
  const auto dy = point.y < bounds.minimum.y
                      ? static_cast<long double>(bounds.minimum.y.value() - point.y.value())
                  : point.y > bounds.maximum.y
                      ? static_cast<long double>(point.y.value() - bounds.maximum.y.value())
                      : 0.0L;
  return std::hypotl(dx, dy);
}

[[nodiscard]] Point nearest_segment_point_to_bounds(Point start, Point end,
                                                    const geometry::AxisAlignedBounds& bounds) {
  std::vector<Point> candidates{start, end};
  const std::array<Point, 4U> corners{bounds.minimum, Point{bounds.maximum.x, bounds.minimum.y},
                                      bounds.maximum, Point{bounds.minimum.x, bounds.maximum.y}};
  const auto dx = static_cast<long double>(end.x.value() - start.x.value());
  const auto dy = static_cast<long double>(end.y.value() - start.y.value());
  const auto length_squared = dx * dx + dy * dy;
  if (length_squared > 0.0L) {
    for (const auto corner : corners) {
      const auto parameter =
          std::clamp((static_cast<long double>(corner.x.value() - start.x.value()) * dx +
                      static_cast<long double>(corner.y.value() - start.y.value()) * dy) /
                         length_squared,
                     0.0L, 1.0L);
      candidates.push_back(interpolate_quantized(start, end, parameter));
    }
  }
  return *std::min_element(candidates.begin(), candidates.end(), [&](Point lhs, Point rhs) {
    const auto lhs_distance = point_bounds_distance_nm(lhs, bounds);
    const auto rhs_distance = point_bounds_distance_nm(rhs, bounds);
    return lhs_distance != rhs_distance ? lhs_distance < rhs_distance : lhs < rhs;
  });
}

void populate_blocks(RegionalRouteResult& result) {
  result.biconnected_blocks.clear();
  result.articulation_fragments.clear();
  if (result.conflict_edges.empty()) {
    return;
  }
  std::map<RegionalRouteFragmentId, std::size_t> vertex_index;
  for (const auto& edge : result.conflict_edges) {
    vertex_index.try_emplace(edge.first, vertex_index.size());
    vertex_index.try_emplace(edge.second, vertex_index.size());
  }
  std::vector<RegionalRouteFragmentId> vertices(vertex_index.size());
  for (const auto& [id, index] : vertex_index) {
    vertices[index] = id;
  }
  std::vector<std::vector<std::size_t>> adjacency(vertices.size());
  for (const auto& edge : result.conflict_edges) {
    adjacency[vertex_index.at(edge.first)].push_back(edge.id);
    adjacency[vertex_index.at(edge.second)].push_back(edge.id);
  }
  for (auto& edges : adjacency) {
    std::sort(edges.begin(), edges.end());
  }
  std::vector<std::size_t> discovery(vertices.size(), 0U);
  std::vector<std::size_t> low(vertices.size(), 0U);
  std::vector<bool> articulation(vertices.size(), false);
  std::vector<std::size_t> edge_stack;
  std::vector<std::vector<std::size_t>> blocks;
  std::size_t next_discovery = 0U;
  constexpr std::size_t no_edge = std::numeric_limits<std::size_t>::max();
  std::function<void(std::size_t, std::size_t)> visit = [&](std::size_t vertex,
                                                            std::size_t parent_edge) {
    discovery[vertex] = low[vertex] = ++next_discovery;
    std::size_t child_count = 0U;
    for (const auto edge_id : adjacency[vertex]) {
      const auto& edge = result.conflict_edges[edge_id];
      const auto other_id = edge.first == vertices[vertex] ? edge.second : edge.first;
      const auto other = vertex_index.at(other_id);
      if (discovery[other] == 0U) {
        ++child_count;
        edge_stack.push_back(edge_id);
        visit(other, edge_id);
        low[vertex] = std::min(low[vertex], low[other]);
        if (low[other] >= discovery[vertex]) {
          if (parent_edge != no_edge || child_count > 1U) {
            articulation[vertex] = true;
          }
          std::vector<std::size_t> block;
          while (!edge_stack.empty()) {
            const auto selected = edge_stack.back();
            edge_stack.pop_back();
            block.push_back(selected);
            if (selected == edge_id) {
              break;
            }
          }
          std::sort(block.begin(), block.end());
          blocks.push_back(std::move(block));
        }
      } else if (edge_id != parent_edge && discovery[other] < discovery[vertex]) {
        low[vertex] = std::min(low[vertex], discovery[other]);
        edge_stack.push_back(edge_id);
      }
    }
  };
  for (std::size_t vertex = 0U; vertex < vertices.size(); ++vertex) {
    if (discovery[vertex] == 0U) {
      visit(vertex, no_edge);
    }
  }
  for (std::size_t index = 0U; index < vertices.size(); ++index) {
    if (articulation[index]) {
      result.articulation_fragments.push_back(vertices[index]);
    }
  }
  std::sort(blocks.begin(), blocks.end(), [&result](const auto& lhs, const auto& rhs) {
    return lhs.front() < rhs.front() || (lhs.front() == rhs.front() && lhs < rhs);
  });
  for (const auto& edge_ids : blocks) {
    RegionalConflictBlock block;
    block.id = result.biconnected_blocks.size();
    block.conflict_edges = edge_ids;
    std::set<RegionalRouteFragmentId> fragments;
    std::set<NetId> nets;
    bool first_bounds = true;
    for (const auto edge_id : edge_ids) {
      const auto& edge = result.conflict_edges[edge_id];
      fragments.insert(edge.first);
      fragments.insert(edge.second);
      nets.insert(edge.first.net);
      nets.insert(edge.second.net);
      block.conflict_bounds = first_bounds
                                  ? edge.conflict_bounds
                                  : union_bounds(block.conflict_bounds, edge.conflict_bounds);
      first_bounds = false;
      block.observed_overlap_length_nm =
          block.observed_overlap_length_nm >
                  std::numeric_limits<std::uint64_t>::max() - edge.observed_overlap_length_nm
              ? std::numeric_limits<std::uint64_t>::max()
              : block.observed_overlap_length_nm + edge.observed_overlap_length_nm;
      block.peak_clearance_deficit =
          std::max(block.peak_clearance_deficit, edge.peak_clearance_deficit);
    }
    block.fragments.assign(fragments.begin(), fragments.end());
    block.nets.assign(nets.begin(), nets.end());
    std::set_intersection(
        block.fragments.begin(), block.fragments.end(), result.articulation_fragments.begin(),
        result.articulation_fragments.end(), std::back_inserter(block.articulation_fragments));
    result.biconnected_blocks.push_back(std::move(block));
  }
}

[[nodiscard]] std::pair<std::size_t, std::size_t>
regional_edge_set_cardinality(const RegionalRouteResult& result,
                              const std::vector<std::size_t>& edges) {
  std::set<RegionalRouteFragmentId> fragments;
  std::set<NetId> nets;
  for (const auto edge_id : edges) {
    const auto& edge = result.conflict_edges[edge_id];
    fragments.insert(edge.first);
    fragments.insert(edge.second);
    nets.insert(edge.first.net);
    nets.insert(edge.second.net);
  }
  return {fragments.size(), nets.size()};
}

struct RegionalSweepSplit {
  std::vector<std::size_t> first;
  std::vector<std::size_t> second;
  std::int64_t gap_nm{0};
  std::size_t axis{0};
};

[[nodiscard]] std::optional<RegionalSweepSplit>
best_conflict_free_sweep_split(const RegionalRouteResult& result,
                               const std::vector<std::size_t>& edges) {
  std::optional<RegionalSweepSplit> best;
  for (std::size_t axis = 0U; axis < 2U; ++axis) {
    auto ordered = edges;
    const auto minimum = [&](const RegionalConflictEdge& edge) {
      return axis == 0U ? edge.conflict_bounds.minimum.x.value()
                        : edge.conflict_bounds.minimum.y.value();
    };
    const auto maximum = [&](const RegionalConflictEdge& edge) {
      return axis == 0U ? edge.conflict_bounds.maximum.x.value()
                        : edge.conflict_bounds.maximum.y.value();
    };
    std::sort(ordered.begin(), ordered.end(), [&](const auto lhs, const auto rhs) {
      const auto& first = result.conflict_edges[lhs];
      const auto& second = result.conflict_edges[rhs];
      return std::tuple{minimum(first), maximum(first), lhs} <
             std::tuple{minimum(second), maximum(second), rhs};
    });
    if (ordered.size() < 2U) {
      continue;
    }
    auto prefix_maximum = maximum(result.conflict_edges[ordered.front()]);
    for (std::size_t split = 1U; split < ordered.size(); ++split) {
      const auto next_minimum = minimum(result.conflict_edges[ordered[split]]);
      if (next_minimum > prefix_maximum) {
        const auto gap = next_minimum - prefix_maximum;
        RegionalSweepSplit candidate;
        candidate.first.assign(ordered.begin(),
                               ordered.begin() + static_cast<std::ptrdiff_t>(split));
        candidate.second.assign(ordered.begin() + static_cast<std::ptrdiff_t>(split),
                                ordered.end());
        std::sort(candidate.first.begin(), candidate.first.end());
        std::sort(candidate.second.begin(), candidate.second.end());
        candidate.gap_nm = gap;
        candidate.axis = axis;
        if (!best || candidate.gap_nm > best->gap_nm ||
            (candidate.gap_nm == best->gap_nm &&
             std::tie(candidate.axis, candidate.first, candidate.second) <
                 std::tie(best->axis, best->first, best->second))) {
          best = std::move(candidate);
        }
      }
      prefix_maximum = std::max(prefix_maximum, maximum(result.conflict_edges[ordered[split]]));
    }
  }
  return best;
}

[[nodiscard]] std::optional<RegionalSweepSplit>
balanced_geometric_edge_split(const RegionalRouteResult& result,
                              const std::vector<std::size_t>& edges) {
  if (edges.size() < 2U) {
    return std::nullopt;
  }
  const auto center = [](const RegionalConflictEdge& edge, std::size_t axis) {
    const auto minimum = axis == 0U ? edge.conflict_bounds.minimum.x.value()
                                    : edge.conflict_bounds.minimum.y.value();
    const auto maximum = axis == 0U ? edge.conflict_bounds.maximum.x.value()
                                    : edge.conflict_bounds.maximum.y.value();
    return std::midpoint(minimum, maximum);
  };
  std::array<std::int64_t, 2U> spans{};
  for (std::size_t axis = 0U; axis < 2U; ++axis) {
    auto minimum = center(result.conflict_edges[edges.front()], axis);
    auto maximum = minimum;
    for (const auto edge_id : edges) {
      const auto value = center(result.conflict_edges[edge_id], axis);
      minimum = std::min(minimum, value);
      maximum = std::max(maximum, value);
    }
    spans[axis] = maximum - minimum;
  }
  const auto axis = spans[1] > spans[0] ? 1U : 0U;
  auto ordered = edges;
  std::stable_sort(ordered.begin(), ordered.end(), [&](const auto lhs, const auto rhs) {
    return std::tuple{center(result.conflict_edges[lhs], axis), lhs} <
           std::tuple{center(result.conflict_edges[rhs], axis), rhs};
  });
  const auto middle = ordered.size() / 2U;
  if (middle == 0U || middle == ordered.size()) {
    return std::nullopt;
  }
  RegionalSweepSplit split;
  split.axis = axis;
  split.first.assign(ordered.begin(), ordered.begin() + static_cast<std::ptrdiff_t>(middle));
  split.second.assign(ordered.begin() + static_cast<std::ptrdiff_t>(middle), ordered.end());
  std::sort(split.first.begin(), split.first.end());
  std::sort(split.second.begin(), split.second.end());
  return split;
}

[[nodiscard]] std::vector<std::vector<std::size_t>>
partition_block_edges(const RegionalRouteResult& result, const RegionalConflictBlock& block,
                      std::size_t& split_count, std::size_t& uncuttable_count) {
  std::vector<std::vector<std::size_t>> pending{block.conflict_edges};
  std::vector<std::vector<std::size_t>> completed;
  while (!pending.empty()) {
    auto edges = std::move(pending.back());
    pending.pop_back();
    const auto [fragments, nets] = regional_edge_set_cardinality(result, edges);
    if (fragments <= result.configured.max_conflict_fragments_per_window &&
        nets <= result.configured.max_nets_per_window) {
      completed.push_back(std::move(edges));
      continue;
    }
    const auto split = best_conflict_free_sweep_split(result, edges);
    const auto minimum_gap =
        result.configured.window_margin.value() > std::numeric_limits<std::int64_t>::max() / 2
            ? std::numeric_limits<std::int64_t>::max()
            : result.configured.window_margin.value() * 2;
    auto selected_split =
        split && split->gap_nm > minimum_gap ? split : balanced_geometric_edge_split(result, edges);
    if (!selected_split) {
      ++uncuttable_count;
      completed.push_back(std::move(edges));
      continue;
    }
    ++split_count;
    // Push the lexicographically later half first so the stable earlier half is processed first.
    pending.push_back(std::move(selected_split->second));
    pending.push_back(std::move(selected_split->first));
  }
  std::sort(completed.begin(), completed.end());
  return completed;
}

[[nodiscard]] RegionalConflictBlock regional_block_from_edges(const RegionalRouteResult& result,
                                                              const RegionalConflictBlock& source,
                                                              std::vector<std::size_t> edges) {
  RegionalConflictBlock block;
  block.id = source.id;
  block.conflict_edges = std::move(edges);
  std::set<RegionalRouteFragmentId> fragments;
  std::set<NetId> nets;
  bool first_bounds = true;
  for (const auto edge_id : block.conflict_edges) {
    const auto& edge = result.conflict_edges[edge_id];
    fragments.insert(edge.first);
    fragments.insert(edge.second);
    nets.insert(edge.first.net);
    nets.insert(edge.second.net);
    block.conflict_bounds = first_bounds
                                ? edge.conflict_bounds
                                : union_bounds(block.conflict_bounds, edge.conflict_bounds);
    first_bounds = false;
    block.observed_overlap_length_nm =
        block.observed_overlap_length_nm >
                std::numeric_limits<std::uint64_t>::max() - edge.observed_overlap_length_nm
            ? std::numeric_limits<std::uint64_t>::max()
            : block.observed_overlap_length_nm + edge.observed_overlap_length_nm;
    block.peak_clearance_deficit =
        std::max(block.peak_clearance_deficit, edge.peak_clearance_deficit);
  }
  block.fragments.assign(fragments.begin(), fragments.end());
  block.nets.assign(nets.begin(), nets.end());
  std::set_intersection(block.fragments.begin(), block.fragments.end(),
                        source.articulation_fragments.begin(), source.articulation_fragments.end(),
                        std::back_inserter(block.articulation_fragments));
  return block;
}

void populate_windows(const Board& board, RegionalRouteResult& result) {
  result.windows.clear();
  result.oversized_windows = 0U;
  result.uncuttable_windows = 0U;
  result.window_fragment_anchor_expansions = 0U;
  result.peak_window_fragment_anchor_expansion = Nanometres{0};
  result.geometrically_split_windows = 0U;
  result.coupled_window_merges = 0U;
  result.uncuttable_oversized_blocks = 0U;
  const auto prepared_outline = geometry::prepare_polygon(board.outline);
  if (!prepared_outline.valid()) {
    return;
  }
  std::map<RegionalRouteFragmentId, const RegionalRouteFragment*> fragments;
  for (const auto& fragment : result.fragments) {
    fragments.emplace(fragment.id, &fragment);
  }
  std::vector<RegionalConflictBlock> window_blocks;
  for (const auto& block : result.biconnected_blocks) {
    const auto partitions = partition_block_edges(result, block, result.geometrically_split_windows,
                                                  result.uncuttable_oversized_blocks);
    for (auto edges : partitions) {
      window_blocks.push_back(regional_block_from_edges(result, block, std::move(edges)));
    }
  }
  std::map<RegionalRouteFragmentId, std::size_t> block_memberships;
  for (const auto& block : window_blocks) {
    for (const auto& fragment : block.fragments) {
      ++block_memberships[fragment];
    }
  }
  for (auto& block : window_blocks) {
    for (const auto& fragment : block.fragments) {
      if (block_memberships[fragment] > 1U) {
        block.articulation_fragments.push_back(fragment);
      }
    }
    std::sort(block.articulation_fragments.begin(), block.articulation_fragments.end());
    block.articulation_fragments.erase(
        std::unique(block.articulation_fragments.begin(), block.articulation_fragments.end()),
        block.articulation_fragments.end());
  }
  for (const auto& block : window_blocks) {
    RegionalRoutingWindow window;
    window.id = result.windows.size();
    window.fragments = block.fragments;
    window.articulation_fragments = block.articulation_fragments;
    window.conflict_edges = block.conflict_edges;
    window.nets = block.nets;
    window.observed_overlap_length_nm = block.observed_overlap_length_nm;
    window.peak_clearance_deficit = block.peak_clearance_deficit;
    Nanometres margin = result.configured.window_margin;
    std::set<LayerId> layers;
    for (const auto& id : block.fragments) {
      const auto found = fragments.find(id);
      if (found == fragments.end()) {
        window.failure_reason = RegionalWindowFailureReason::uncuttable_fragment;
        continue;
      }
      const auto* fragment = found->second;
      layers.insert(fragment->layer);
      margin = std::max(margin,
                        bounded_add(fragment->clearance, Nanometres{fragment->width.value() / 2}));
    }
    window.layers.assign(layers.begin(), layers.end());
    const auto sampled_bounds =
        expanded_bounds(block.conflict_bounds, margin, prepared_outline.bounds);
    auto anchored_bounds = block.conflict_bounds;
    for (const auto& id : block.fragments) {
      const auto found = fragments.find(id);
      if (found == fragments.end() ||
          clip_segment_to_bounds(found->second->start, found->second->end, sampled_bounds)) {
        continue;
      }
      const auto anchor =
          nearest_segment_point_to_bounds(found->second->start, found->second->end, sampled_bounds);
      const auto distance = point_bounds_distance_nm(anchor, sampled_bounds);
      if (distance > 0.0L) {
        ++result.window_fragment_anchor_expansions;
        result.peak_window_fragment_anchor_expansion =
            std::max(result.peak_window_fragment_anchor_expansion,
                     Nanometres{static_cast<std::int64_t>(std::ceil(distance))});
      }
      anchored_bounds = union_bounds(anchored_bounds, geometry::AxisAlignedBounds{anchor, anchor});
    }
    window.bounds = expanded_bounds(anchored_bounds, margin, prepared_outline.bounds);
    if (window.fragments.size() > result.configured.max_conflict_fragments_per_window) {
      window.failure_reason = RegionalWindowFailureReason::oversized_fragment_set;
      ++result.oversized_windows;
    } else if (window.nets.size() > result.configured.max_nets_per_window) {
      window.failure_reason = RegionalWindowFailureReason::oversized_net_set;
      ++result.oversized_windows;
    }
    for (const auto& id : window.fragments) {
      const auto found = fragments.find(id);
      if (found == fragments.end()) {
        continue;
      }
      const auto& fragment = *found->second;
      const auto clipped = clip_segment_to_bounds(fragment.start, fragment.end, window.bounds);
      if (!clipped) {
        window.failure_reason = RegionalWindowFailureReason::uncuttable_fragment;
        continue;
      }
      const auto dx = static_cast<long double>(fragment.end.x.value() - fragment.start.x.value());
      const auto dy = static_cast<long double>(fragment.end.y.value() - fragment.start.y.value());
      const auto magnitude = std::hypotl(dx, dy);
      const auto tangent =
          magnitude > 0.0L ? SplineVector{dx / magnitude, dy / magnitude} : SplineVector{};
      const auto add_portal = [&](long double parameter, SplineVector inward) {
        window.portals.push_back(
            RegionalBoundaryPortal{window.portals.size(), id,
                                   interpolate_quantized(fragment.start, fragment.end, parameter),
                                   fragment.layer, inward, fragment.width});
      };
      if (clipped->first_parameter > 0.0L || !fragment.begins_at_terminal) {
        add_portal(clipped->first_parameter, tangent);
      }
      if (clipped->second_parameter < 1.0L || !fragment.ends_at_terminal) {
        add_portal(clipped->second_parameter, SplineVector{-tangent.x, -tangent.y});
      }
      if (window.portals.empty() && !fragment.begins_at_terminal && !fragment.ends_at_terminal) {
        window.failure_reason = RegionalWindowFailureReason::uncuttable_fragment;
      }
    }
    if (window.failure_reason == RegionalWindowFailureReason::uncuttable_fragment) {
      ++result.uncuttable_windows;
    }
    result.windows.push_back(std::move(window));
  }
  std::stable_sort(result.windows.begin(), result.windows.end(),
                   [](const auto& lhs, const auto& rhs) {
                     if (lhs.observed_overlap_length_nm != rhs.observed_overlap_length_nm) {
                       return lhs.observed_overlap_length_nm > rhs.observed_overlap_length_nm;
                     }
                     if (lhs.peak_clearance_deficit != rhs.peak_clearance_deficit) {
                       return lhs.peak_clearance_deficit > rhs.peak_clearance_deficit;
                     }
                     return std::tie(lhs.bounds.minimum, lhs.bounds.maximum, lhs.fragments) <
                            std::tie(rhs.bounds.minimum, rhs.bounds.maximum, rhs.fragments);
                   });
  for (std::size_t index = 0U; index < result.windows.size(); ++index) {
    result.windows[index].id = index;
    for (std::size_t portal = 0U; portal < result.windows[index].portals.size(); ++portal) {
      result.windows[index].portals[portal].id = portal;
    }
  }
  std::map<RegionalRouteFragmentId, std::pair<std::uint64_t, std::size_t>> owner_by_fragment;
  for (const auto& window : result.windows) {
    for (const auto& fragment : window.fragments) {
      std::uint64_t pressure = 0U;
      for (const auto edge_id : window.conflict_edges) {
        const auto& edge = result.conflict_edges[edge_id];
        if (edge.first != fragment && edge.second != fragment) {
          continue;
        }
        pressure =
            pressure > std::numeric_limits<std::uint64_t>::max() - edge.observed_overlap_length_nm
                ? std::numeric_limits<std::uint64_t>::max()
                : pressure + edge.observed_overlap_length_nm;
      }
      const auto found = owner_by_fragment.find(fragment);
      if (found == owner_by_fragment.end() || pressure > found->second.first ||
          (pressure == found->second.first && window.id < found->second.second)) {
        owner_by_fragment[fragment] = {pressure, window.id};
      }
    }
  }
  for (auto& window : result.windows) {
    for (const auto& fragment : window.fragments) {
      if (owner_by_fragment.at(fragment).second == window.id) {
        window.owned_fragments.push_back(fragment);
      }
    }
  }
}

[[nodiscard]] bool
build_residual_conflict_cone_windows(RegionalRouteResult& retry,
                                     const RegionalRouteResult& ordinary,
                                     std::set<std::size_t>& replaced_window_ids) {
  std::vector<const RegionalRoutingWindow*> failed;
  for (const auto& assignment : ordinary.bundle_assignments) {
    if (assignment.complete()) {
      continue;
    }
    const auto window =
        std::find_if(ordinary.windows.begin(), ordinary.windows.end(),
                     [&](const auto& candidate) { return candidate.id == assignment.window_id; });
    if (window != ordinary.windows.end() && window->eligible()) {
      failed.push_back(&*window);
    }
  }
  std::stable_sort(failed.begin(), failed.end(), [](const auto* lhs, const auto* rhs) {
    if (lhs->observed_overlap_length_nm != rhs->observed_overlap_length_nm) {
      return lhs->observed_overlap_length_nm > rhs->observed_overlap_length_nm;
    }
    if (lhs->peak_clearance_deficit != rhs->peak_clearance_deficit) {
      return lhs->peak_clearance_deficit > rhs->peak_clearance_deficit;
    }
    return lhs->id < rhs->id;
  });

  std::map<RegionalRouteFragmentId, std::size_t> owner_window;
  for (const auto& window : ordinary.windows) {
    for (const auto& fragment : window.owned_fragments) {
      owner_window.emplace(fragment, window.id);
    }
  }
  std::set<std::size_t> consumed;
  std::vector<std::vector<std::size_t>> cones;
  for (const auto* failed_window : failed) {
    if (consumed.contains(failed_window->id)) {
      continue;
    }
    std::set<std::size_t> members{failed_window->id};
    // Take the transitive owner closure. A one-hop merge can import an articulation fragment
    // owned by a third window, causing the retry and retained checkpoint to publish two different
    // realizations for the same fragment. The bounded closure makes the retry an atomic regional
    // transaction instead of relying on candidate reconstruction to discover that split later.
    bool expanded = true;
    while (expanded) {
      expanded = false;
      const auto current = members;
      for (const auto member_id : current) {
        const auto member =
            std::find_if(ordinary.windows.begin(), ordinary.windows.end(),
                         [&](const auto& candidate) { return candidate.id == member_id; });
        if (member == ordinary.windows.end()) {
          continue;
        }
        for (const auto& fragment : member->fragments) {
          const auto owner = owner_window.find(fragment);
          if (owner != owner_window.end()) {
            expanded = members.insert(owner->second).second || expanded;
          }
        }
      }
    }
    if (members.size() < 2U ||
        members.size() > retry.configured.max_residual_conflict_cone_windows ||
        std::any_of(members.begin(), members.end(),
                    [&](const auto id) { return consumed.contains(id); })) {
      continue;
    }
    std::set<RegionalRouteFragmentId> fragments;
    std::set<NetId> nets;
    bool eligible = true;
    for (const auto id : members) {
      const auto window = std::find_if(retry.windows.begin(), retry.windows.end(),
                                       [&](const auto& candidate) { return candidate.id == id; });
      if (window == retry.windows.end() || !window->eligible()) {
        eligible = false;
        break;
      }
      fragments.insert(window->fragments.begin(), window->fragments.end());
      nets.insert(window->nets.begin(), window->nets.end());
    }
    if (!eligible || fragments.size() > retry.configured.max_conflict_fragments_per_window ||
        nets.size() > retry.configured.max_nets_per_window) {
      continue;
    }
    cones.emplace_back(members.begin(), members.end());
    consumed.insert(members.begin(), members.end());
  }
  if (cones.empty()) {
    return false;
  }
  std::map<std::size_t, std::size_t> cone_by_window;
  for (std::size_t cone = 0U; cone < cones.size(); ++cone) {
    for (const auto window : cones[cone]) {
      cone_by_window.emplace(window, cone);
    }
  }
  std::vector<RegionalRoutingWindow> merged_windows;
  merged_windows.reserve(cones.size());
  std::set<std::size_t> emitted_cones;
  for (const auto& source : retry.windows) {
    const auto cone = cone_by_window.find(source.id);
    if (cone == cone_by_window.end()) {
      continue;
    }
    if (!emitted_cones.insert(cone->second).second) {
      continue;
    }
    RegionalRoutingWindow merged;
    merged.id = *std::min_element(cones[cone->second].begin(), cones[cone->second].end());
    bool first_bounds = true;
    std::set<LayerId> layers;
    std::set<RegionalRouteFragmentId> fragments;
    std::set<RegionalRouteFragmentId> articulations;
    std::set<std::size_t> conflict_edges;
    std::set<NetId> nets;
    for (const auto member_id : cones[cone->second]) {
      const auto member =
          std::find_if(retry.windows.begin(), retry.windows.end(),
                       [&](const auto& candidate) { return candidate.id == member_id; });
      if (member == retry.windows.end()) {
        continue;
      }
      merged.bounds = first_bounds ? member->bounds : union_bounds(merged.bounds, member->bounds);
      first_bounds = false;
      layers.insert(member->layers.begin(), member->layers.end());
      fragments.insert(member->fragments.begin(), member->fragments.end());
      articulations.insert(member->articulation_fragments.begin(),
                           member->articulation_fragments.end());
      conflict_edges.insert(member->conflict_edges.begin(), member->conflict_edges.end());
      nets.insert(member->nets.begin(), member->nets.end());
      merged.portals.insert(merged.portals.end(), member->portals.begin(), member->portals.end());
    }
    merged.layers.assign(layers.begin(), layers.end());
    merged.fragments.assign(fragments.begin(), fragments.end());
    merged.articulation_fragments.assign(articulations.begin(), articulations.end());
    merged.conflict_edges.assign(conflict_edges.begin(), conflict_edges.end());
    merged.nets.assign(nets.begin(), nets.end());
    for (const auto edge_id : merged.conflict_edges) {
      if (edge_id >= retry.conflict_edges.size()) {
        continue;
      }
      const auto& edge = retry.conflict_edges[edge_id];
      merged.observed_overlap_length_nm =
          merged.observed_overlap_length_nm >
                  std::numeric_limits<std::uint64_t>::max() - edge.observed_overlap_length_nm
              ? std::numeric_limits<std::uint64_t>::max()
              : merged.observed_overlap_length_nm + edge.observed_overlap_length_nm;
      merged.peak_clearance_deficit =
          std::max(merged.peak_clearance_deficit, edge.peak_clearance_deficit);
    }
    std::stable_sort(merged.portals.begin(), merged.portals.end(),
                     [](const auto& lhs, const auto& rhs) {
                       return std::tie(lhs.fragment, lhs.position, lhs.layer, lhs.width) <
                              std::tie(rhs.fragment, rhs.position, rhs.layer, rhs.width);
                     });
    merged.portals.erase(
        std::unique(merged.portals.begin(), merged.portals.end(),
                    [](const auto& lhs, const auto& rhs) {
                      return std::tie(lhs.fragment, lhs.position, lhs.layer, lhs.width) ==
                             std::tie(rhs.fragment, rhs.position, rhs.layer, rhs.width);
                    }),
        merged.portals.end());
    merged_windows.push_back(std::move(merged));
  }
  retry.windows = std::move(merged_windows);
  for (auto& window : retry.windows) {
    window.owned_fragments.clear();
    for (std::size_t portal = 0U; portal < window.portals.size(); ++portal) {
      window.portals[portal].id = portal;
    }
  }

  std::map<RegionalRouteFragmentId, std::pair<std::uint64_t, std::size_t>> selected_owner;
  for (const auto& window : retry.windows) {
    for (const auto& fragment : window.fragments) {
      std::uint64_t pressure = 0U;
      for (const auto edge_id : window.conflict_edges) {
        if (edge_id >= retry.conflict_edges.size()) {
          continue;
        }
        const auto& edge = retry.conflict_edges[edge_id];
        if (edge.first == fragment || edge.second == fragment) {
          pressure =
              pressure > std::numeric_limits<std::uint64_t>::max() - edge.observed_overlap_length_nm
                  ? std::numeric_limits<std::uint64_t>::max()
                  : pressure + edge.observed_overlap_length_nm;
        }
      }
      const auto found = selected_owner.find(fragment);
      if (found == selected_owner.end() || pressure > found->second.first ||
          (pressure == found->second.first && window.id < found->second.second)) {
        selected_owner[fragment] = {pressure, window.id};
      }
    }
  }
  for (auto& window : retry.windows) {
    for (const auto& fragment : window.fragments) {
      if (selected_owner.at(fragment).second == window.id) {
        window.owned_fragments.push_back(fragment);
      }
    }
  }
  retry.residual_conflict_cones_built = cones.size();
  retry.residual_conflict_cone_windows_merged = consumed.size();
  replaced_window_ids = std::move(consumed);
  return true;
}

[[nodiscard]] std::size_t next_channel_id(const RegionalRouteResult& result) {
  return result.channels.empty() ? 0U
                                 : std::max_element(
                                       result.channels.begin(), result.channels.end(),
                                       [](const auto& lhs, const auto& rhs) {
                                         return lhs.id < rhs.id;
                                       })->id +
                                       1U;
}

[[nodiscard]] std::size_t next_terminal_choice_id(const RegionalRouteResult& result) {
  return result.terminal_choices.empty()
             ? 0U
             : std::max_element(
                   result.terminal_choices.begin(), result.terminal_choices.end(),
                   [](const auto& lhs, const auto& rhs) {
                     return lhs.id < rhs.id;
                   })->id +
                   1U;
}

[[nodiscard]] std::size_t next_via_choice_id(const RegionalRouteResult& result) {
  return result.via_choices.empty() ? 0U
                                    : std::max_element(
                                          result.via_choices.begin(), result.via_choices.end(),
                                          [](const auto& lhs, const auto& rhs) {
                                            return lhs.id < rhs.id;
                                          })->id +
                                          1U;
}

[[nodiscard]] std::size_t next_via_region_id(const RegionalRouteResult& result) {
  const auto choices = result.via_choices.empty()
                           ? 0U
                           : std::max_element(
                                 result.via_choices.begin(), result.via_choices.end(),
                                 [](const auto& lhs, const auto& rhs) {
                                   return lhs.region_id < rhs.region_id;
                                 })->region_id +
                                 1U;
  const auto evidence =
      result.via_region_evidence.empty()
          ? 0U
          : std::max_element(
                result.via_region_evidence.begin(), result.via_region_evidence.end(),
                [](const auto& lhs, const auto& rhs) {
                  return lhs.region_id < rhs.region_id;
                })->region_id +
                1U;
  return std::max(choices, evidence);
}

void offset_residual_retry_ids_for_append(RegionalRouteResult& retry,
                                          const RegionalRouteResult& retained) {
  const auto channel_offset = next_channel_id(retained);
  const auto terminal_offset = next_terminal_choice_id(retained);
  const auto via_offset = next_via_choice_id(retained);
  const auto via_region_offset = next_via_region_id(retained);
  for (auto& channel : retry.channels) {
    channel.id += channel_offset;
    for (auto& adjacent : channel.adjacent_channels) {
      adjacent += channel_offset;
    }
  }
  for (auto& choice : retry.via_choices) {
    choice.id += via_offset;
    choice.region_id += via_region_offset;
  }
  for (auto& evidence : retry.via_region_evidence) {
    evidence.region_id += via_region_offset;
    for (auto& choice : evidence.candidate_choices) {
      choice += via_offset;
    }
  }
  for (auto& choice : retry.terminal_choices) {
    choice.id += terminal_offset;
    if (choice.via_choice) {
      *choice.via_choice += via_offset;
    }
  }
  const auto remap_optional = [](std::optional<std::size_t>& id, std::size_t offset) {
    if (id) {
      *id += offset;
    }
  };
  const auto remap_selection = [&](RegionalBundleSelection& selection) {
    for (auto& choice : selection.via_choices) {
      choice += via_offset;
    }
    for (auto& region : selection.via_assignment_region_order) {
      region += via_region_offset;
    }
  };
  for (auto& assignment : retry.bundle_assignments) {
    for (auto& route_template : assignment.templates) {
      for (auto& channel : route_template.channel_sequence) {
        channel += channel_offset;
      }
      for (auto& choice : route_template.via_choices) {
        choice += via_offset;
      }
      remap_optional(route_template.entry_via_choice, via_offset);
      remap_optional(route_template.exit_via_choice, via_offset);
      remap_optional(route_template.terminal_entry_choice, terminal_offset);
      remap_optional(route_template.terminal_exit_choice, terminal_offset);
    }
    for (auto& choice : assignment.selected_via_choices) {
      choice += via_offset;
    }
    for (auto& region : assignment.via_assignment_region_order) {
      region += via_region_offset;
    }
    for (auto& selection : assignment.spline_candidate_shortlist) {
      remap_selection(selection);
    }
    for (auto& blocker : assignment.partial_access_blockers) {
      remap_optional(blocker.first_via_choice, via_offset);
      remap_optional(blocker.second_via_choice, via_offset);
      remap_optional(blocker.terminal_choice, terminal_offset);
    }
  }
}

[[nodiscard]] RegionalRouteResult
merge_residual_cone_checkpoint(const RegionalRouteResult& ordinary, RegionalRouteResult retry,
                               const std::set<std::size_t>& replaced_window_ids) {
  RegionalRouteResult combined = ordinary;
  std::erase_if(combined.windows,
                [&](const auto& window) { return replaced_window_ids.contains(window.id); });
  std::erase_if(combined.bundle_assignments, [&](const auto& assignment) {
    return replaced_window_ids.contains(assignment.window_id);
  });
  const auto reused_assignments = combined.bundle_assignments.size();
  const auto recomputed_assignments = retry.bundle_assignments.size();
  offset_residual_retry_ids_for_append(retry, combined);
  combined.windows.insert(combined.windows.end(), std::make_move_iterator(retry.windows.begin()),
                          std::make_move_iterator(retry.windows.end()));
  combined.channels.insert(combined.channels.end(), std::make_move_iterator(retry.channels.begin()),
                           std::make_move_iterator(retry.channels.end()));
  combined.terminal_choices.insert(combined.terminal_choices.end(),
                                   std::make_move_iterator(retry.terminal_choices.begin()),
                                   std::make_move_iterator(retry.terminal_choices.end()));
  combined.via_choices.insert(combined.via_choices.end(),
                              std::make_move_iterator(retry.via_choices.begin()),
                              std::make_move_iterator(retry.via_choices.end()));
  combined.via_region_evidence.insert(combined.via_region_evidence.end(),
                                      std::make_move_iterator(retry.via_region_evidence.begin()),
                                      std::make_move_iterator(retry.via_region_evidence.end()));
  combined.bundle_assignments.insert(combined.bundle_assignments.end(),
                                     std::make_move_iterator(retry.bundle_assignments.begin()),
                                     std::make_move_iterator(retry.bundle_assignments.end()));
  std::stable_sort(combined.windows.begin(), combined.windows.end(),
                   [](const auto& lhs, const auto& rhs) { return lhs.id < rhs.id; });
  std::stable_sort(combined.bundle_assignments.begin(), combined.bundle_assignments.end(),
                   [](const auto& lhs, const auto& rhs) { return lhs.window_id < rhs.window_id; });
  combined.residual_conflict_cone_retries = 1U;
  combined.residual_conflict_cones_built = retry.residual_conflict_cones_built;
  combined.residual_conflict_cone_windows_merged = retry.residual_conflict_cone_windows_merged;
  combined.residual_conflict_cone_reused_assignments = reused_assignments;
  combined.residual_conflict_cone_recomputed_assignments = recomputed_assignments;
  combined.residual_conflict_cone_retry_withheld_assignments = 0U;
  combined.lane_solver_queries += retry.lane_solver_queries;
  combined.lane_solver_cache_hits += retry.lane_solver_cache_hits;
  combined.lane_solver_states += retry.lane_solver_states;
  combined.lane_solver_pruned_states += retry.lane_solver_pruned_states;
  combined.lane_solver_analytical_rejections += retry.lane_solver_analytical_rejections;
  combined.lane_solver_budget_exhaustions += retry.lane_solver_budget_exhaustions;
  combined.partial_lane_capacity_pruned_states += retry.partial_lane_capacity_pruned_states;
  combined.spline_candidate_paths.clear();
  combined.spline_candidate_available = false;
  combined.spline_candidate_conflicting_articulation = false;
  return combined;
}

[[nodiscard]] bool
coupled_template_selection_legal(const Board& board, const RegionalRoutingWindow& window,
                                 const std::vector<RegionalRouteTemplate>& templates,
                                 const std::vector<std::size_t>& selected) {
  const auto atom_legal = [&](const std::vector<NetId>& atom) {
    const auto participating =
        static_cast<std::size_t>(std::count_if(atom.begin(), atom.end(), [&](const auto& net) {
          return std::find(window.nets.begin(), window.nets.end(), net) != window.nets.end();
        }));
    if (participating == 0U) {
      return true;
    }
    if (participating != atom.size()) {
      return std::all_of(selected.begin(), selected.end(), [&](const auto template_id) {
        if (template_id >= templates.size()) {
          return false;
        }
        const auto& route_template = templates[template_id];
        return std::find(atom.begin(), atom.end(), route_template.fragment.net) == atom.end() ||
               route_template.selected_global_topology;
      });
    }
    std::map<NetId, std::vector<std::vector<LayerId>>> changed_sequences_by_member;
    for (const auto template_id : selected) {
      if (template_id >= templates.size()) {
        return false;
      }
      const auto& route_template = templates[template_id];
      if (std::find(atom.begin(), atom.end(), route_template.fragment.net) == atom.end()) {
        continue;
      }
      if (!route_template.selected_global_topology) {
        changed_sequences_by_member[route_template.fragment.net].push_back(
            route_template.layer_sequence);
      }
    }
    if (changed_sequences_by_member.empty()) {
      return true;
    }
    for (auto& [member, sequences] : changed_sequences_by_member) {
      static_cast<void>(member);
      std::sort(sequences.begin(), sequences.end());
    }
    const auto reference = changed_sequences_by_member.find(atom.front());
    return reference != changed_sequences_by_member.end() &&
           std::all_of(atom.begin() + 1, atom.end(), [&](const auto& member) {
             const auto found = changed_sequences_by_member.find(member);
             return found != changed_sequences_by_member.end() &&
                    found->second == reference->second;
           });
  };
  for (const auto& pair : board.differential_pairs) {
    if (!atom_legal({pair.positive_net, pair.negative_net})) {
      return false;
    }
  }
  for (const auto& bus : board.routing_buses) {
    if (!atom_legal(bus.ordered_nets)) {
      return false;
    }
  }
  return true;
}

enum class RegionalAccessFailureReason {
  none,
  invalid_choice,
  terminal_pair,
  via_pair,
  terminal_via
};

[[nodiscard]] bool regional_via_spans_overlap(const RegionalViaChoice& first,
                                              const RegionalViaChoice& second) {
  return std::any_of(first.span.begin(), first.span.end(), [&](const auto& layer) {
    return std::find(second.span.begin(), second.span.end(), layer) != second.span.end();
  });
}

[[nodiscard]] bool regional_via_choices_conflict(const RegionalViaChoice& first,
                                                 const RegionalViaChoice& second,
                                                 Nanometres clearance) {
  if (first.net == second.net || !regional_via_spans_overlap(first, second)) {
    return false;
  }
  const auto required =
      static_cast<long double>(first.diameter.value() + second.diameter.value()) / 2.0L +
      static_cast<long double>(clearance.value());
  return geometry::point_distance_nm(first.site, second.site) < required;
}

[[nodiscard]] bool regional_terminal_via_conflict(const RegionalTerminalChoice& terminal,
                                                  const RegionalViaChoice& via,
                                                  Nanometres clearance) {
  if (terminal.net == via.net ||
      std::find(via.span.begin(), via.span.end(), terminal.layer) == via.span.end()) {
    return false;
  }
  const auto required =
      static_cast<long double>(terminal.width.value() + via.diameter.value()) / 2.0L +
      static_cast<long double>(clearance.value());
  for (std::size_t edge = 1U; edge < terminal.access_path.size(); ++edge) {
    if (geometry::point_segment_distance_nm(via.site, terminal.access_path[edge - 1U],
                                            terminal.access_path[edge]) < required) {
      return true;
    }
  }
  return false;
}

struct RegionalAccessSelection {
  RegionalAccessFailureReason failure{RegionalAccessFailureReason::invalid_choice};
  std::vector<std::size_t> via_choices;
  std::vector<std::size_t> region_order;
  std::size_t states{0};
  std::size_t alternative_switches{0};
  bool exhausted{false};
  std::optional<std::pair<std::size_t, std::size_t>> via_pair_blocker;
  std::optional<std::pair<std::size_t, std::size_t>> terminal_via_blocker;

  [[nodiscard]] bool legal() const { return failure == RegionalAccessFailureReason::none; }
};

[[nodiscard]] RegionalAccessSelection select_access_geometry(
    const Board& board, const RegionalRouteResult& regional, const RegionalRoutingWindow& window,
    const std::vector<RegionalRouteTemplate>& templates, const std::vector<std::size_t>& selected) {
  RegionalAccessSelection result;
  std::vector<const RegionalTerminalChoice*> terminal_choices;
  struct ViaVariable {
    std::pair<NetId, std::size_t> key;
    std::vector<const RegionalViaChoice*> candidates;
    const RegionalViaRegionEvidence* evidence{nullptr};
  };
  std::map<std::pair<NetId, std::size_t>, ViaVariable> variables;
  for (const auto template_id : selected) {
    if (template_id >= templates.size()) {
      return result;
    }
    const auto& route_template = templates[template_id];
    if (!std::binary_search(window.owned_fragments.begin(), window.owned_fragments.end(),
                            route_template.fragment)) {
      continue;
    }
    const auto route_fragment = std::find_if(
        regional.fragments.begin(), regional.fragments.end(),
        [&](const auto& candidate) { return candidate.id == route_template.fragment; });
    if (route_fragment == regional.fragments.end()) {
      return result;
    }
    for (const auto choice_id :
         {route_template.terminal_entry_choice, route_template.terminal_exit_choice}) {
      if (!choice_id) {
        continue;
      }
      const auto found =
          std::find_if(regional.terminal_choices.begin(), regional.terminal_choices.end(),
                       [&](const auto& choice) { return choice.id == *choice_id; });
      if (found == regional.terminal_choices.end()) {
        return result;
      }
      terminal_choices.push_back(&*found);
    }
    for (const auto choice_id : route_template.via_choices) {
      const auto found = std::find_if(regional.via_choices.begin(), regional.via_choices.end(),
                                      [&](const auto& choice) { return choice.id == choice_id; });
      if (found == regional.via_choices.end()) {
        return result;
      }
      const auto key = std::pair{found->net, found->region_id};
      auto [variable_iterator, inserted] = variables.try_emplace(key);
      auto& variable = variable_iterator->second;
      if (inserted) {
        variable.key = key;
      }
      std::vector<const RegionalViaChoice*> allowed;
      const auto terminal_endpoint_via =
          (route_template.entry_via_choice == choice_id && route_fragment->begins_at_terminal) ||
          (route_template.exit_via_choice == choice_id && route_fragment->ends_at_terminal);
      const auto topology_can_move_via = route_template.selected_global_topology &&
                                         !route_fragment->explicit_width &&
                                         !terminal_endpoint_via && found->selected_global_site;
      if (topology_can_move_via) {
        for (const auto& candidate : regional.via_choices) {
          if (candidate.net == found->net && candidate.region_id == found->region_id &&
              candidate.span == found->span) {
            allowed.push_back(&candidate);
          }
        }
      } else {
        allowed.push_back(&*found);
      }
      if (variable.candidates.empty()) {
        variable.candidates = std::move(allowed);
      } else {
        std::erase_if(variable.candidates, [&](const auto* candidate) {
          return std::none_of(allowed.begin(), allowed.end(), [&](const auto* option) {
            return option->site == candidate->site && option->span == candidate->span;
          });
        });
      }
      if (variable.candidates.empty()) {
        return result;
      }
    }
  }
  std::map<std::pair<NetId, NetId>, Nanometres> pairwise_clearances;
  const auto pairwise_clearance = [&](NetId first, NetId second) {
    if (second < first) {
      std::swap(first, second);
    }
    const auto key = std::pair{first, second};
    const auto found = pairwise_clearances.find(key);
    if (found != pairwise_clearances.end()) {
      return found->second;
    }
    return pairwise_clearances.emplace(key, resolve_pairwise_clearance(board, first, second))
        .first->second;
  };
  for (std::size_t first = 0U; first < terminal_choices.size(); ++first) {
    const auto* lhs = terminal_choices[first];
    for (std::size_t second = first + 1U; second < terminal_choices.size(); ++second) {
      const auto* rhs = terminal_choices[second];
      if (lhs->net == rhs->net || lhs->layer != rhs->layer) {
        continue;
      }
      const auto required =
          static_cast<long double>(lhs->width.value() + rhs->width.value()) / 2.0L +
          static_cast<long double>(pairwise_clearance(lhs->net, rhs->net).value());
      for (std::size_t lhs_edge = 1U; lhs_edge < lhs->access_path.size(); ++lhs_edge) {
        for (std::size_t rhs_edge = 1U; rhs_edge < rhs->access_path.size(); ++rhs_edge) {
          if (geometry::segment_distance_nm(
                  lhs->access_path[lhs_edge - 1U], lhs->access_path[lhs_edge],
                  rhs->access_path[rhs_edge - 1U], rhs->access_path[rhs_edge]) < required) {
            result.failure = RegionalAccessFailureReason::terminal_pair;
            return result;
          }
        }
      }
    }
  }
  std::vector<ViaVariable> ordered_variables;
  for (auto& [key, variable] : variables) {
    variable.evidence = [&]() -> const RegionalViaRegionEvidence* {
      const auto found =
          std::find_if(regional.via_region_evidence.begin(), regional.via_region_evidence.end(),
                       [&](const auto& evidence) {
                         return evidence.net == key.first && evidence.region_id == key.second;
                       });
      return found == regional.via_region_evidence.end() ? nullptr : &*found;
    }();
    ordered_variables.push_back(std::move(variable));
  }
  const auto evidence_value = [](const ViaVariable& variable, const auto& selector) {
    return variable.evidence == nullptr ? std::size_t{0} : selector(*variable.evidence);
  };
  std::stable_sort(
      ordered_variables.begin(), ordered_variables.end(), [&](const auto& lhs, const auto& rhs) {
        if (lhs.candidates.size() != rhs.candidates.size()) {
          return lhs.candidates.size() < rhs.candidates.size();
        }
        const auto lhs_competing =
            evidence_value(lhs, [](const auto& evidence) { return evidence.competing_regions; });
        const auto rhs_competing =
            evidence_value(rhs, [](const auto& evidence) { return evidence.competing_regions; });
        if (lhs_competing != rhs_competing) {
          return lhs_competing > rhs_competing;
        }
        const auto lhs_conflicts = evidence_value(
            lhs, [](const auto& evidence) { return evidence.conflicting_candidate_pairs; });
        const auto rhs_conflicts = evidence_value(
            rhs, [](const auto& evidence) { return evidence.conflicting_candidate_pairs; });
        if (lhs_conflicts != rhs_conflicts) {
          return lhs_conflicts > rhs_conflicts;
        }
        const auto lhs_terminal = evidence_value(
            lhs, [](const auto& evidence) { return evidence.terminal_escape_conflicts; });
        const auto rhs_terminal = evidence_value(
            rhs, [](const auto& evidence) { return evidence.terminal_escape_conflicts; });
        if (lhs_terminal != rhs_terminal) {
          return lhs_terminal > rhs_terminal;
        }
        const auto lhs_layers =
            evidence_value(lhs, [](const auto& evidence) { return evidence.span.size(); });
        const auto rhs_layers =
            evidence_value(rhs, [](const auto& evidence) { return evidence.span.size(); });
        return lhs_layers != rhs_layers ? lhs_layers > rhs_layers : lhs.key < rhs.key;
      });
  result.region_order.reserve(ordered_variables.size());
  for (const auto& variable : ordered_variables) {
    result.region_order.push_back(variable.key.second);
  }
  const auto terminal_conflict =
      [&](const RegionalViaChoice* via) -> const RegionalTerminalChoice* {
    const auto found =
        std::find_if(terminal_choices.begin(), terminal_choices.end(), [&](const auto* terminal) {
          return regional_terminal_via_conflict(*terminal, *via,
                                                pairwise_clearance(terminal->net, via->net));
        });
    return found == terminal_choices.end() ? nullptr : *found;
  };
  const auto terminal_legal = [&](const RegionalViaChoice* via) {
    return terminal_conflict(via) == nullptr;
  };
  const auto assigned_conflict =
      [&](const RegionalViaChoice* via,
          const std::vector<const RegionalViaChoice*>& assigned) -> const RegionalViaChoice* {
    const auto found = std::find_if(assigned.begin(), assigned.end(), [&](const auto* prior) {
      return regional_via_choices_conflict(*prior, *via, pairwise_clearance(prior->net, via->net));
    });
    return found == assigned.end() ? nullptr : *found;
  };
  const auto compatible_with_assigned = [&](const RegionalViaChoice* via,
                                            const std::vector<const RegionalViaChoice*>& assigned) {
    return assigned_conflict(via, assigned) == nullptr;
  };
  std::vector<const RegionalViaChoice*> assigned;
  bool budget_exhausted = false;
  std::function<bool(std::size_t)> search = [&](std::size_t variable) {
    if (variable == ordered_variables.size()) {
      result.failure = RegionalAccessFailureReason::none;
      result.via_choices.reserve(assigned.size());
      std::transform(assigned.begin(), assigned.end(), std::back_inserter(result.via_choices),
                     [](const auto* via) { return via->id; });
      result.alternative_switches = static_cast<std::size_t>(
          std::count_if(assigned.begin(), assigned.end(),
                        [](const auto* via) { return !via->selected_global_site; }));
      return true;
    }
    for (std::size_t candidate_index = 0U;
         candidate_index < ordered_variables[variable].candidates.size(); ++candidate_index) {
      if (result.states >= regional.configured.max_access_assignment_states) {
        budget_exhausted = true;
        return false;
      }
      ++result.states;
      const auto* candidate = ordered_variables[variable].candidates[candidate_index];
      if (const auto* terminal = terminal_conflict(candidate); terminal != nullptr) {
        if (result.failure == RegionalAccessFailureReason::invalid_choice) {
          result.failure = RegionalAccessFailureReason::terminal_via;
        }
        result.terminal_via_blocker = std::pair{terminal->id, candidate->id};
        continue;
      }
      if (const auto* prior = assigned_conflict(candidate, assigned); prior != nullptr) {
        result.failure = RegionalAccessFailureReason::via_pair;
        result.via_pair_blocker = std::minmax(prior->id, candidate->id);
        continue;
      }
      assigned.push_back(candidate);
      bool forward_legal = true;
      for (std::size_t future = variable + 1U; future < ordered_variables.size(); ++future) {
        const auto has_terminal_legal_option =
            std::any_of(ordered_variables[future].candidates.begin(),
                        ordered_variables[future].candidates.end(), terminal_legal);
        const auto has_compatible_option = std::any_of(
            ordered_variables[future].candidates.begin(),
            ordered_variables[future].candidates.end(), [&](const auto* option) {
              return terminal_legal(option) && compatible_with_assigned(option, assigned);
            });
        if (!has_compatible_option) {
          result.failure = has_terminal_legal_option ? RegionalAccessFailureReason::via_pair
                                                     : RegionalAccessFailureReason::terminal_via;
          if (has_terminal_legal_option) {
            for (const auto* option : ordered_variables[future].candidates) {
              if (!terminal_legal(option)) {
                continue;
              }
              if (const auto* prior = assigned_conflict(option, assigned); prior != nullptr) {
                result.via_pair_blocker = std::minmax(prior->id, option->id);
                break;
              }
            }
          } else {
            for (const auto* option : ordered_variables[future].candidates) {
              if (const auto* terminal = terminal_conflict(option); terminal != nullptr) {
                result.terminal_via_blocker = std::pair{terminal->id, option->id};
                break;
              }
            }
          }
          forward_legal = false;
          break;
        }
      }
      if (forward_legal && search(variable + 1U)) {
        return true;
      }
      assigned.pop_back();
      if (budget_exhausted) {
        return false;
      }
    }
    return false;
  };
  if (ordered_variables.empty()) {
    result.failure = RegionalAccessFailureReason::none;
    result.states = 1U;
    return result;
  }
  static_cast<void>(search(0U));
  result.exhausted = budget_exhausted;
  return result;
}

[[nodiscard]] bool polygon_bounds_overlap(const Polygon& first, const Polygon& second) {
  const auto first_prepared = geometry::prepare_polygon(first);
  const auto second_prepared = geometry::prepare_polygon(second);
  return first_prepared.valid() && second_prepared.valid() &&
         first_prepared.bounds.overlaps(second_prepared.bounds) &&
         geometry::polygons_overlap(first_prepared, second_prepared);
}

[[nodiscard]] bool corridor_cells_overlap(const RegionalChannel& channel,
                                          const RoutingGeometricCorridorRun& corridor) {
  return std::any_of(
      channel.topology_cells.begin(), channel.topology_cells.end(), [&corridor](const auto& first) {
        return std::any_of(
            corridor.topology_cells.begin(), corridor.topology_cells.end(),
            [&first](const auto& second) { return polygon_bounds_overlap(first, second); });
      });
}

void populate_channels(const Board& board, const GlobalRouteResult& global,
                       RegionalRouteResult& result) {
  result.channels.clear();
  std::map<std::pair<NetId, std::size_t>, std::vector<RegionalRouteFragmentId>> path_fragments;
  for (const auto& fragment : result.fragments) {
    path_fragments[{fragment.id.net, fragment.id.connection_index}].push_back(fragment.id);
  }
  for (const auto& path : global.paths) {
    for (const auto& corridor : path.geometric_corridors) {
      if (!corridor.selected_path_preserves_topology || corridor.topology_cells.empty() ||
          corridor.minimum_physical_cross_section_width.value() <= 0) {
        continue;
      }
      std::vector<geometry::PreparedPolygon> prepared_corridor_cells;
      prepared_corridor_cells.reserve(corridor.topology_cells.size());
      std::transform(corridor.topology_cells.begin(), corridor.topology_cells.end(),
                     std::back_inserter(prepared_corridor_cells),
                     [](const auto& cell) { return geometry::prepare_polygon(cell); });
      const auto fragment_traverses_corridor = [&](const RegionalRouteFragment& fragment) {
        const geometry::AxisAlignedBounds segment_bounds{
            Point{Nanometres{std::min(fragment.start.x.value(), fragment.end.x.value())},
                  Nanometres{std::min(fragment.start.y.value(), fragment.end.y.value())}},
            Point{Nanometres{std::max(fragment.start.x.value(), fragment.end.x.value())},
                  Nanometres{std::max(fragment.start.y.value(), fragment.end.y.value())}}};
        return std::any_of(prepared_corridor_cells.begin(), prepared_corridor_cells.end(),
                           [&](const auto& cell) {
                             if (!cell.valid()) {
                               return false;
                             }
                             if (!cell.bounds.overlaps(segment_bounds)) {
                               return false;
                             }
                             return geometry::locate_point(cell, fragment.start) !=
                                        geometry::PointLocation::outside ||
                                    geometry::locate_point(cell, fragment.end) !=
                                        geometry::PointLocation::outside ||
                                    geometry::segment_polygon_boundary_distance_nm(
                                        fragment.start, fragment.end, cell) == 0.0L;
                           });
      };
      auto channel = std::find_if(
          result.channels.begin(), result.channels.end(), [&corridor](const auto& existing) {
            return existing.layer == corridor.layer && corridor_cells_overlap(existing, corridor);
          });
      if (channel == result.channels.end()) {
        result.channels.push_back(RegionalChannel{});
        channel = std::prev(result.channels.end());
        channel->id = result.channels.size() - 1U;
        channel->layer = corridor.layer;
        channel->topology_cells = corridor.topology_cells;
        channel->portals = corridor.portals;
      } else {
        channel->topology_cells.insert(channel->topology_cells.end(),
                                       corridor.topology_cells.begin(),
                                       corridor.topology_cells.end());
        channel->portals.insert(channel->portals.end(), corridor.portals.begin(),
                                corridor.portals.end());
      }
      const auto found = path_fragments.find({path.net, path.connection_index});
      if (found != path_fragments.end()) {
        for (const auto& fragment_id : found->second) {
          const auto fragment = std::find_if(
              result.fragments.begin(), result.fragments.end(),
              [&fragment_id](const auto& candidate) { return candidate.id == fragment_id; });
          if (fragment != result.fragments.end() && fragment->layer == corridor.layer &&
              fragment_traverses_corridor(*fragment) &&
              std::find(channel->occupants.begin(), channel->occupants.end(), fragment_id) ==
                  channel->occupants.end()) {
            channel->occupants.push_back(fragment_id);
          }
        }
      }
      std::sort(channel->occupants.begin(), channel->occupants.end());
      if (corridor.lane_centerline.size() >= 2U) {
        const auto first = corridor.lane_centerline.front();
        const auto last = corridor.lane_centerline.back();
        const auto dx = static_cast<long double>(last.x.value() - first.x.value());
        const auto dy = static_cast<long double>(last.y.value() - first.y.value());
        const auto magnitude = std::hypotl(dx, dy);
        if (magnitude > 0.0L) {
          const auto half =
              static_cast<long double>(corridor.minimum_physical_cross_section_width.value()) /
              2.0L;
          const auto middle_x = (static_cast<long double>(first.x.value()) + last.x.value()) / 2.0L;
          const auto middle_y = (static_cast<long double>(first.y.value()) + last.y.value()) / 2.0L;
          const auto normal_x = -dy / magnitude;
          const auto normal_y = dx / magnitude;
          const Point first_wall{
              Nanometres{static_cast<std::int64_t>(std::llround(middle_x - half * normal_x))},
              Nanometres{static_cast<std::int64_t>(std::llround(middle_y - half * normal_y))}};
          const Point second_wall{
              Nanometres{static_cast<std::int64_t>(std::llround(middle_x + half * normal_x))},
              Nanometres{static_cast<std::int64_t>(std::llround(middle_y + half * normal_y))}};
          auto cross_section = pack_regional_cross_section(
              result.fragments, channel->occupants, corridor.layer, first_wall, second_wall,
              corridor.minimum_physical_cross_section_width);
          cross_section.id = channel->cross_sections.size();
          channel->cross_sections.push_back(std::move(cross_section));
        }
      }
    }
  }
  std::set<RegionalRouteFragmentId> governed_fragments;
  for (const auto& window : result.windows) {
    if (window.eligible()) {
      governed_fragments.insert(window.fragments.begin(), window.fragments.end());
    }
  }
  const auto has_channel = [&](const RegionalRouteFragmentId& fragment) {
    return std::any_of(result.channels.begin(), result.channels.end(), [&](const auto& channel) {
      return std::find(channel.occupants.begin(), channel.occupants.end(), fragment) !=
             channel.occupants.end();
    });
  };
  using DomainKey = std::pair<NetId, LayerId>;
  std::map<DomainKey, std::vector<const RegionalRouteFragment*>> missing_by_domain;
  for (const auto& fragment : result.fragments) {
    if (governed_fragments.contains(fragment.id) && !has_channel(fragment.id)) {
      missing_by_domain[{fragment.id.net, fragment.layer}].push_back(&fragment);
    }
  }
  std::set<std::pair<RegionalRouteFragmentId, RegionalRouteFragmentId>> conflict_pairs;
  const auto ordered_pair = [](RegionalRouteFragmentId first, RegionalRouteFragmentId second) {
    if (second < first) {
      std::swap(first, second);
    }
    return std::pair{std::move(first), std::move(second)};
  };
  for (const auto& edge : result.conflict_edges) {
    conflict_pairs.insert(ordered_pair(edge.first, edge.second));
  }
  const auto conflicts_with_channel = [&](const RegionalRouteFragmentId& fragment,
                                          const RegionalChannel& channel) {
    return std::any_of(channel.occupants.begin(), channel.occupants.end(), [&](const auto& other) {
      return conflict_pairs.contains(ordered_pair(fragment, other));
    });
  };
  const auto append_selected_channel = [&](const RegionalRouteFragment& fragment,
                                           Nanometres topology_half_width,
                                           Nanometres usable_half_width) {
    const auto dx = static_cast<long double>(fragment.end.x.value() - fragment.start.x.value());
    const auto dy = static_cast<long double>(fragment.end.y.value() - fragment.start.y.value());
    const auto magnitude = std::hypotl(dx, dy);
    if (!(magnitude > 0.0L) || topology_half_width.value() <= 0 || usable_half_width.value() <= 0) {
      return false;
    }
    const auto normal_x = -dy / magnitude;
    const auto normal_y = dx / magnitude;
    const auto offset_point = [&](Point point, long double direction, Nanometres distance) {
      return Point{Nanometres{static_cast<std::int64_t>(std::llround(
                       static_cast<long double>(point.x.value()) +
                       direction * static_cast<long double>(distance.value()) * normal_x))},
                   Nanometres{static_cast<std::int64_t>(std::llround(
                       static_cast<long double>(point.y.value()) +
                       direction * static_cast<long double>(distance.value()) * normal_y))}};
    };
    const Polygon cell{{offset_point(fragment.start, 1.0L, topology_half_width),
                        offset_point(fragment.end, 1.0L, topology_half_width),
                        offset_point(fragment.end, -1.0L, topology_half_width),
                        offset_point(fragment.start, -1.0L, topology_half_width)}};
    if (!geometry::is_simple_polygon(cell)) {
      return false;
    }
    auto channel =
        std::find_if(result.channels.begin(), result.channels.end(), [&](const auto& existing) {
          return existing.layer == fragment.layer &&
                 conflicts_with_channel(fragment.id, existing) &&
                 std::any_of(existing.topology_cells.begin(), existing.topology_cells.end(),
                             [&](const auto& existing_cell) {
                               return polygon_bounds_overlap(existing_cell, cell);
                             });
        });
    if (channel == result.channels.end()) {
      result.channels.push_back(RegionalChannel{});
      channel = std::prev(result.channels.end());
      channel->id = result.channels.size() - 1U;
      channel->layer = fragment.layer;
    }
    channel->topology_cells.push_back(cell);
    if (std::find(channel->occupants.begin(), channel->occupants.end(), fragment.id) ==
        channel->occupants.end()) {
      channel->occupants.push_back(fragment.id);
      std::sort(channel->occupants.begin(), channel->occupants.end());
    }
    const auto bounded_width =
        usable_half_width.value() > std::numeric_limits<std::int64_t>::max() / 2
            ? std::numeric_limits<std::int64_t>::max()
            : usable_half_width.value() * 2;
    const auto middle = Point{Nanometres{fragment.start.x.value() +
                                         (fragment.end.x.value() - fragment.start.x.value()) / 2},
                              Nanometres{fragment.start.y.value() +
                                         (fragment.end.y.value() - fragment.start.y.value()) / 2}};
    const auto first_wall = offset_point(middle, -1.0L, usable_half_width);
    const auto second_wall = offset_point(middle, 1.0L, usable_half_width);
    auto cross_section =
        pack_regional_cross_section(result.fragments, channel->occupants, fragment.layer,
                                    first_wall, second_wall, Nanometres{bounded_width});
    cross_section.id = channel->cross_sections.size();
    channel->cross_sections.push_back(std::move(cross_section));
    return true;
  };
  for (const auto& [key, fragments] : missing_by_domain) {
    const auto extracted = extract_polygonal_board_free_space(
        board,
        BoardFreeSpaceExtractionConfig{key.second, key.first, true, true,
                                       !result.configured.treat_saved_zone_fills_as_refillable});
    if (!extracted.complete()) {
      continue;
    }
    std::vector<std::pair<Point, Point>> chords;
    chords.reserve(fragments.size());
    for (const auto* fragment : fragments) {
      chords.emplace_back(fragment->start, fragment->end);
    }
    const auto certificates = assess_free_space_chords(extracted.domain, chords);
    for (std::size_t index = 0U; index < fragments.size(); ++index) {
      const auto* fragment = fragments[index];
      const auto& certificate = certificates[index];
      const auto required = bounded_add(
          fragment->clearance, Nanometres{std::max<std::int64_t>(1, fragment->width.value() / 2)});
      if (!certificate || certificate->value() < required.value()) {
        continue;
      }
      static_cast<void>(append_selected_channel(*fragment, *certificate, *certificate));
    }
  }
  for (const auto& fragment : result.fragments) {
    if (!governed_fragments.contains(fragment.id) || has_channel(fragment.id)) {
      continue;
    }
    const auto required = bounded_add(
        fragment.clearance, Nanometres{std::max<std::int64_t>(1, fragment.width.value() / 2)});
    if (append_selected_channel(fragment, Nanometres{1}, required)) {
      ++result.selected_global_narrow_corridor_fallbacks;
    }
  }
  result.fragments_without_channel_topology = 0U;
  for (const auto& fragment : result.fragments) {
    if (std::none_of(result.channels.begin(), result.channels.end(),
                     [&fragment](const auto& channel) {
                       return std::find(channel.occupants.begin(), channel.occupants.end(),
                                        fragment.id) != channel.occupants.end();
                     })) {
      ++result.fragments_without_channel_topology;
    }
  }
}

[[nodiscard]] std::size_t inversion_count(const std::vector<RegionalRouteFragmentId>& current,
                                          const std::vector<RegionalRouteFragmentId>& candidate) {
  std::map<RegionalRouteFragmentId, std::size_t> position;
  for (std::size_t index = 0U; index < current.size(); ++index) {
    position.emplace(current[index], index);
  }
  std::size_t inversions = 0U;
  for (std::size_t first = 0U; first < candidate.size(); ++first) {
    for (std::size_t second = first + 1U; second < candidate.size(); ++second) {
      if (position.at(candidate[first]) > position.at(candidate[second])) {
        ++inversions;
      }
    }
  }
  return inversions;
}

struct RegionalBundleChoicePortfolio {
  std::vector<std::vector<std::size_t>> choices;
  std::vector<RegionalAccessBlocker> access_blockers;
  std::size_t states{0};
  std::size_t access_pruned_states{0};
  std::size_t access_assignment_states{0};
  std::size_t access_assignment_exhaustions{0};
  std::size_t terminal_pair_pruned_states{0};
  std::size_t via_pair_pruned_states{0};
  std::size_t terminal_via_pruned_states{0};
  std::size_t lane_capacity_pruned_states{0};
  bool exhausted{false};
};

[[nodiscard]] RegionalBundleChoicePortfolio generate_consistent_bundle_choices(
    const Board& board, const RegionalRouteResult& regional, const RegionalRoutingWindow& window,
    const std::vector<RegionalRouteTemplate>& templates,
    const std::vector<std::vector<std::size_t>>& templates_by_fragment,
    std::size_t maximum_states) {
  RegionalBundleChoicePortfolio result;
  if (templates_by_fragment.empty() || maximum_states == 0U) {
    return result;
  }
  std::vector<std::size_t> variable_order(templates_by_fragment.size());
  std::iota(variable_order.begin(), variable_order.end(), 0U);
  std::stable_sort(
      variable_order.begin(), variable_order.end(), [&](const auto lhs, const auto rhs) {
        return templates_by_fragment[lhs].size() != templates_by_fragment[rhs].size()
                   ? templates_by_fragment[lhs].size() < templates_by_fragment[rhs].size()
                   : lhs < rhs;
      });
  using ViaRegionKey = std::pair<NetId, std::size_t>;
  struct ViaSiteIdentity {
    Point site;
    std::vector<LayerId> span;
  };
  std::vector<std::size_t> choice(templates_by_fragment.size(), 0U);
  struct FragmentEnvelope {
    long double width_and_clearance_nm{0.0L};
    long double clearance_nm{0.0L};
    bool valid{false};
  };
  std::map<RegionalRouteFragmentId, FragmentEnvelope> fragment_envelopes;
  for (const auto& fragment : regional.fragments) {
    fragment_envelopes.emplace(
        fragment.id,
        FragmentEnvelope{static_cast<long double>(fragment.width.value()) +
                             static_cast<long double>(fragment.clearance.value()),
                         static_cast<long double>(fragment.clearance.value()),
                         fragment.width.value() > 0 && fragment.clearance.value() >= 0});
  }
  std::map<std::size_t, long double> maximum_channel_width_nm;
  for (const auto& channel : regional.channels) {
    long double maximum_width = 0.0L;
    for (const auto& cross_section : channel.cross_sections) {
      maximum_width =
          std::max(maximum_width, static_cast<long double>(cross_section.usable_width.value()));
    }
    maximum_channel_width_nm.emplace(channel.id, maximum_width);
  }
  struct PartialChannelEnvelope {
    long double width_and_clearance_nm{0.0L};
    long double maximum_clearance_nm{0.0L};
  };
  std::map<std::size_t, PartialChannelEnvelope> channel_envelopes;
  using AccessBlockerKey =
      std::tuple<RegionalAccessBlockerKind, std::size_t, std::size_t, std::size_t>;
  std::map<AccessBlockerKey, std::size_t> access_blocker_counts;
  std::function<void(std::size_t, const std::map<ViaRegionKey, ViaSiteIdentity>&,
                     const std::vector<std::size_t>&)>
      search = [&](const std::size_t depth,
                   const std::map<ViaRegionKey, ViaSiteIdentity>& assigned_vias,
                   const std::vector<std::size_t>& selected_templates) {
        if (result.exhausted) {
          return;
        }
        if (depth == variable_order.size()) {
          result.choices.push_back(choice);
          return;
        }
        const auto position = variable_order[depth];
        for (std::size_t candidate_index = 0U;
             candidate_index < templates_by_fragment[position].size(); ++candidate_index) {
          if (result.states >= maximum_states) {
            result.exhausted = true;
            return;
          }
          ++result.states;
          const auto template_id = templates_by_fragment[position][candidate_index];
          if (template_id >= templates.size()) {
            continue;
          }
          auto next_vias = assigned_vias;
          bool compatible = true;
          for (const auto choice_id : templates[template_id].via_choices) {
            const auto via =
                std::find_if(regional.via_choices.begin(), regional.via_choices.end(),
                             [&](const auto& candidate) { return candidate.id == choice_id; });
            if (via == regional.via_choices.end()) {
              compatible = false;
              break;
            }
            const auto key = ViaRegionKey{via->net, via->region_id};
            const auto found = next_vias.find(key);
            const auto identity = ViaSiteIdentity{via->site, via->span};
            if (found != next_vias.end() &&
                (found->second.site != identity.site || found->second.span != identity.span)) {
              compatible = false;
              break;
            }
            next_vias.insert_or_assign(key, identity);
          }
          if (!compatible) {
            continue;
          }
          const auto fragment_envelope = fragment_envelopes.find(templates[template_id].fragment);
          bool channel_capacity_legal =
              fragment_envelope != fragment_envelopes.end() && fragment_envelope->second.valid;
          std::vector<std::pair<std::size_t, std::optional<PartialChannelEnvelope>>>
              changed_channel_envelopes;
          auto candidate_channels = templates[template_id].channel_sequence;
          std::sort(candidate_channels.begin(), candidate_channels.end());
          candidate_channels.erase(
              std::unique(candidate_channels.begin(), candidate_channels.end()),
              candidate_channels.end());
          for (const auto channel_id : candidate_channels) {
            if (!channel_capacity_legal) {
              break;
            }
            const auto maximum_width = maximum_channel_width_nm.find(channel_id);
            if (maximum_width == maximum_channel_width_nm.end() || maximum_width->second <= 0.0L) {
              channel_capacity_legal = false;
              break;
            }
            const auto current = channel_envelopes.find(channel_id);
            const auto previous = current == channel_envelopes.end()
                                      ? std::optional<PartialChannelEnvelope>{}
                                      : std::optional<PartialChannelEnvelope>{current->second};
            auto next = previous.value_or(PartialChannelEnvelope{});
            next.width_and_clearance_nm += fragment_envelope->second.width_and_clearance_nm;
            next.maximum_clearance_nm =
                std::max(next.maximum_clearance_nm, fragment_envelope->second.clearance_nm);
            changed_channel_envelopes.emplace_back(channel_id, previous);
            channel_envelopes.insert_or_assign(channel_id, next);
            if (next.width_and_clearance_nm + next.maximum_clearance_nm > maximum_width->second) {
              channel_capacity_legal = false;
              break;
            }
          }
          if (!channel_capacity_legal) {
            ++result.lane_capacity_pruned_states;
            for (auto changed = changed_channel_envelopes.rbegin();
                 changed != changed_channel_envelopes.rend(); ++changed) {
              if (changed->second) {
                channel_envelopes.insert_or_assign(changed->first, *changed->second);
              } else {
                channel_envelopes.erase(changed->first);
              }
            }
            continue;
          }
          auto next_templates = selected_templates;
          next_templates.push_back(template_id);
          const auto access =
              select_access_geometry(board, regional, window, templates, next_templates);
          result.access_assignment_states += access.states;
          result.access_assignment_exhaustions += access.exhausted ? 1U : 0U;
          if (!access.legal()) {
            ++result.access_pruned_states;
            switch (access.failure) {
            case RegionalAccessFailureReason::terminal_pair:
              ++result.terminal_pair_pruned_states;
              break;
            case RegionalAccessFailureReason::via_pair:
              ++result.via_pair_pruned_states;
              if (access.via_pair_blocker) {
                ++access_blocker_counts[{
                    RegionalAccessBlockerKind::via_pair, access.via_pair_blocker->first,
                    access.via_pair_blocker->second, std::numeric_limits<std::size_t>::max()}];
              }
              break;
            case RegionalAccessFailureReason::terminal_via:
              ++result.terminal_via_pruned_states;
              if (access.terminal_via_blocker) {
                ++access_blocker_counts[{
                    RegionalAccessBlockerKind::terminal_via, access.terminal_via_blocker->second,
                    std::numeric_limits<std::size_t>::max(), access.terminal_via_blocker->first}];
              }
              break;
            case RegionalAccessFailureReason::none:
            case RegionalAccessFailureReason::invalid_choice:
              break;
            }
            for (auto changed = changed_channel_envelopes.rbegin();
                 changed != changed_channel_envelopes.rend(); ++changed) {
              if (changed->second) {
                channel_envelopes.insert_or_assign(changed->first, *changed->second);
              } else {
                channel_envelopes.erase(changed->first);
              }
            }
            continue;
          }
          choice[position] = candidate_index;
          search(depth + 1U, next_vias, next_templates);
          for (auto changed = changed_channel_envelopes.rbegin();
               changed != changed_channel_envelopes.rend(); ++changed) {
            if (changed->second) {
              channel_envelopes.insert_or_assign(changed->first, *changed->second);
            } else {
              channel_envelopes.erase(changed->first);
            }
          }
          if (result.exhausted) {
            return;
          }
        }
      };
  search(0U, {}, {});
  for (const auto& [key, observations] : access_blocker_counts) {
    const auto [kind, first_via, second_via, terminal] = key;
    result.access_blockers.push_back(RegionalAccessBlocker{
        kind,
        first_via == std::numeric_limits<std::size_t>::max()
            ? std::optional<std::size_t>{}
            : std::optional<std::size_t>{first_via},
        second_via == std::numeric_limits<std::size_t>::max()
            ? std::optional<std::size_t>{}
            : std::optional<std::size_t>{second_via},
        terminal == std::numeric_limits<std::size_t>::max() ? std::optional<std::size_t>{}
                                                            : std::optional<std::size_t>{terminal},
        observations});
  }
  std::stable_sort(result.access_blockers.begin(), result.access_blockers.end(),
                   [](const auto& lhs, const auto& rhs) {
                     return lhs.observations != rhs.observations
                                ? lhs.observations > rhs.observations
                                : std::tuple{lhs.kind, lhs.first_via_choice, lhs.second_via_choice,
                                             lhs.terminal_choice} <
                                      std::tuple{rhs.kind, rhs.first_via_choice,
                                                 rhs.second_via_choice, rhs.terminal_choice};
                   });
  if (result.access_blockers.size() > regional.configured.max_access_blockers_per_window) {
    result.access_blockers.resize(regional.configured.max_access_blockers_per_window);
  }
  return result;
}

[[nodiscard]] std::vector<LayerId> regional_via_span(const Board& board, const LayerId& first,
                                                     const LayerId& last);

void populate_selected_endpoint_choices(const Board& board, const GlobalRouteResult& global,
                                        RegionalRouteResult& result) {
  result.terminal_choices.clear();
  result.via_choices.clear();
  std::map<std::pair<NetId, Point>, std::pair<ObjectId, ObjectId>> pad_by_position;
  for (const auto& footprint : board.footprints) {
    for (const auto& pad : footprint.pads) {
      const auto position = geometry::transform_point(pad.local_position, footprint.position,
                                                      footprint.rotation, footprint.side);
      pad_by_position.try_emplace(std::pair{pad.net, position}, std::pair{footprint.id, pad.id});
    }
  }
  const auto static_pads = copper_pad_shapes(board);
  const auto static_keepouts = world_keepouts(board);
  std::map<std::pair<NetId, NetId>, Nanometres> pairwise_clearances;
  const auto pairwise_clearance = [&](NetId first, NetId second) {
    if (second < first) {
      std::swap(first, second);
    }
    const auto key = std::pair{first, second};
    const auto found = pairwise_clearances.find(key);
    if (found != pairwise_clearances.end()) {
      return found->second;
    }
    return pairwise_clearances.emplace(key, resolve_pairwise_clearance(board, first, second))
        .first->second;
  };
  std::size_t selected_via_site_candidate_checks = 0U;
  struct PlannedViaSite {
    NetId net;
    Point site;
    std::vector<LayerId> span;
    Nanometres diameter;
    Nanometres clearance;
  };
  struct SelectedViaTransition {
    const GlobalRoutePath* path{nullptr};
    std::size_t begin_point_index{0};
    std::size_t end_point_index{0};
    Point site;
    std::vector<LayerId> span;
  };
  std::vector<SelectedViaTransition> selected_transitions;
  std::vector<PlannedViaSite> planned_via_sites;
  for (const auto& path : global.paths) {
    const auto rules = resolve_net_routing_rules(board, path.net);
    for (std::size_t begin = 0U; begin + 1U < path.points.size();) {
      if (path.points[begin].layer == path.points[begin + 1U].layer ||
          path.points[begin].position != path.points[begin + 1U].position) {
        ++begin;
        continue;
      }
      auto end = begin + 1U;
      while (end + 1U < path.points.size() &&
             path.points[end].layer != path.points[end + 1U].layer &&
             path.points[end + 1U].position == path.points[begin].position) {
        ++end;
      }
      const auto site = path.points[begin].position;
      if (std::none_of(result.windows.begin(), result.windows.end(), [&](const auto& window) {
            return window.bounds.contains(site) &&
                   std::find(window.nets.begin(), window.nets.end(), path.net) != window.nets.end();
          })) {
        begin = end;
        continue;
      }
      const auto span = regional_via_span(board, path.points[begin].layer, path.points[end].layer);
      if (span.size() >= 2U) {
        selected_transitions.push_back(SelectedViaTransition{&path, begin, end, site, span});
        planned_via_sites.push_back(
            PlannedViaSite{path.net, site, span, rules.via_diameter, rules.clearance});
      }
      begin = end;
    }
  }
  const auto via_site_legal = [&](const NetId& net, Point site, const std::vector<LayerId>& span,
                                  Nanometres diameter) {
    const auto radius = static_cast<long double>(diameter.value()) / 2.0L;
    const auto touches_layer = [&](const LayerId& layer) {
      return std::find(span.begin(), span.end(), layer) != span.end();
    };
    if (geometry::locate_point_in_board_area(board, site) == geometry::PointLocation::outside ||
        geometry::point_board_boundary_distance_nm(site, board) <
            radius + static_cast<long double>(board.rules.copper_to_edge_clearance.value())) {
      return false;
    }
    for (const auto& track : board.tracks) {
      if (track.net == net || !touches_layer(track.layer)) {
        continue;
      }
      const auto required = radius + static_cast<long double>(track.width.value()) / 2.0L +
                            static_cast<long double>(pairwise_clearance(net, track.net).value());
      if (geometry::point_segment_distance_nm(site, track.start, track.end) < required) {
        return false;
      }
    }
    for (const auto& arc : board.track_arcs) {
      if (arc.net == net || !touches_layer(arc.layer)) {
        continue;
      }
      const auto required = radius + static_cast<long double>(arc.width.value()) / 2.0L +
                            static_cast<long double>(pairwise_clearance(net, arc.net).value());
      if (geometry::point_circular_arc_distance_nm(site, arc.start, arc.middle, arc.end) <
          required) {
        return false;
      }
    }
    for (const auto& via : board.vias) {
      if (via.net == net || std::none_of(via.layers.begin(), via.layers.end(), touches_layer)) {
        continue;
      }
      const auto required = radius + static_cast<long double>(via.diameter.value()) / 2.0L +
                            static_cast<long double>(pairwise_clearance(net, via.net).value());
      if (geometry::point_distance_nm(site, via.position) < required) {
        return false;
      }
    }
    for (const auto& pad : static_pads) {
      if (pad.net == net || std::none_of(pad.layers.begin(), pad.layers.end(), touches_layer)) {
        continue;
      }
      const auto required =
          radius + static_cast<long double>(pairwise_clearance(net, pad.net).value());
      if (point_to_pad_copper_distance_nm(site, pad) < required) {
        return false;
      }
    }
    if (!result.configured.treat_saved_zone_fills_as_refillable) {
      for (const auto& zone : board.zones) {
        if (zone.net == net) {
          continue;
        }
        const auto required =
            radius + static_cast<long double>(pairwise_clearance(net, zone.net).value());
        for (const auto& fill : zone.fills) {
          if (touches_layer(fill.layer) &&
              geometry::point_filled_region_copper_distance_nm(site, fill) < required) {
            return false;
          }
        }
      }
    }
    for (const auto& keepout : static_keepouts) {
      if (!keepout.prohibit.vias ||
          std::none_of(keepout.layers.begin(), keepout.layers.end(), touches_layer)) {
        continue;
      }
      if (point_copper_overlaps_keepout(site, radius, keepout.area)) {
        return false;
      }
    }
    return true;
  };
  const auto runup_legal = [&](const NetId& net, const LayerId& layer, Point start, Point end,
                               Nanometres width) {
    const auto radius = static_cast<long double>(width.value()) / 2.0L;
    if (!geometry::segment_inside_board_area(start, end, board) ||
        geometry::segment_board_boundary_distance_nm(start, end, board) <
            radius + static_cast<long double>(board.rules.copper_to_edge_clearance.value())) {
      return false;
    }
    for (const auto& track : board.tracks) {
      if (track.net == net || track.layer != layer) {
        continue;
      }
      const auto required = radius + static_cast<long double>(track.width.value()) / 2.0L +
                            static_cast<long double>(pairwise_clearance(net, track.net).value());
      if (geometry::segment_distance_nm(start, end, track.start, track.end) < required) {
        return false;
      }
    }
    for (const auto& arc : board.track_arcs) {
      if (arc.net == net || arc.layer != layer) {
        continue;
      }
      const auto required = radius + static_cast<long double>(arc.width.value()) / 2.0L +
                            static_cast<long double>(pairwise_clearance(net, arc.net).value());
      if (geometry::segment_circular_arc_distance_nm(start, end, arc.start, arc.middle, arc.end) <
          required) {
        return false;
      }
    }
    for (const auto& via : board.vias) {
      if (via.net == net ||
          std::find(via.layers.begin(), via.layers.end(), layer) == via.layers.end()) {
        continue;
      }
      const auto required = radius + static_cast<long double>(via.diameter.value()) / 2.0L +
                            static_cast<long double>(pairwise_clearance(net, via.net).value());
      if (geometry::point_segment_distance_nm(via.position, start, end) < required) {
        return false;
      }
    }
    for (const auto& pad : static_pads) {
      if (pad.net == net ||
          std::find(pad.layers.begin(), pad.layers.end(), layer) == pad.layers.end()) {
        continue;
      }
      const auto required =
          radius + static_cast<long double>(pairwise_clearance(net, pad.net).value());
      if (segment_to_pad_copper_distance_nm(start, end, pad) < required) {
        return false;
      }
    }
    if (!result.configured.treat_saved_zone_fills_as_refillable) {
      for (const auto& zone : board.zones) {
        if (zone.net == net) {
          continue;
        }
        const auto required =
            radius + static_cast<long double>(pairwise_clearance(net, zone.net).value());
        for (const auto& fill : zone.fills) {
          if (fill.layer == layer &&
              geometry::segment_filled_region_copper_distance_nm(start, end, fill) < required) {
            return false;
          }
        }
      }
    }
    return std::none_of(static_keepouts.begin(), static_keepouts.end(), [&](const auto& keepout) {
      return keepout.prohibit.tracks &&
             std::find(keepout.layers.begin(), keepout.layers.end(), layer) !=
                 keepout.layers.end() &&
             segment_copper_overlaps_keepout(start, end, radius, keepout.area);
    });
  };
  for (const auto& window : result.windows) {
    for (const auto& fragment_id : window.fragments) {
      const auto fragment = std::find_if(
          result.fragments.begin(), result.fragments.end(),
          [&fragment_id](const auto& candidate) { return candidate.id == fragment_id; });
      if (fragment == result.fragments.end()) {
        continue;
      }
      const auto append_terminal = [&](Point endpoint, bool terminal) {
        if (!terminal) {
          return;
        }
        const auto found = pad_by_position.find({fragment->id.net, endpoint});
        if (found == pad_by_position.end()) {
          return;
        }
        const auto existing = std::find_if(result.terminal_choices.begin(),
                                           result.terminal_choices.end(), [&](const auto& choice) {
                                             return choice.footprint == found->second.first &&
                                                    choice.pad == found->second.second &&
                                                    choice.net == fragment->id.net &&
                                                    choice.layer == fragment->layer &&
                                                    choice.portal == endpoint &&
                                                    choice.selected_global_access;
                                           });
        if (existing == result.terminal_choices.end()) {
          result.terminal_choices.push_back(RegionalTerminalChoice{result.terminal_choices.size(),
                                                                   found->second.first,
                                                                   found->second.second,
                                                                   fragment->id.net,
                                                                   endpoint,
                                                                   endpoint,
                                                                   fragment->layer,
                                                                   {endpoint},
                                                                   fragment->width,
                                                                   std::nullopt,
                                                                   true});
        }
        const auto pressured = std::any_of(
            window.conflict_edges.begin(), window.conflict_edges.end(), [&](const auto edge_id) {
              if (edge_id >= result.conflict_edges.size()) {
                return false;
              }
              const auto& conflict = result.conflict_edges[edge_id];
              return conflict.first == fragment->id || conflict.second == fragment->id;
            });
        if (!pressured || result.configured.max_terminal_choices_per_terminal <= 1U) {
          return;
        }
        const auto step = std::max<std::int64_t>(
            1, std::max(result.configured.navmesh_cell_pitch.value(), fragment->width.value()));
        std::vector<Point> portals;
        for (const auto [x, y] : std::array<std::pair<int, int>, 8U>{
                 std::pair{-1, -1}, std::pair{-1, 0}, std::pair{-1, 1}, std::pair{0, -1},
                 std::pair{0, 1}, std::pair{1, -1}, std::pair{1, 0}, std::pair{1, 1}}) {
          const Point portal{Nanometres{endpoint.x.value() + x * step},
                             Nanometres{endpoint.y.value() + y * step}};
          if (runup_legal(fragment->id.net, fragment->layer, endpoint, portal, fragment->width)) {
            portals.push_back(portal);
          }
        }
        const auto pressure_distance = [&](Point portal) {
          long double distance = 0.0L;
          for (const auto edge_id : window.conflict_edges) {
            if (edge_id >= result.conflict_edges.size()) {
              continue;
            }
            const auto& conflict = result.conflict_edges[edge_id];
            if (conflict.first != fragment->id && conflict.second != fragment->id) {
              continue;
            }
            const Point center{
                Nanometres{std::midpoint(conflict.conflict_bounds.minimum.x.value(),
                                         conflict.conflict_bounds.maximum.x.value())},
                Nanometres{std::midpoint(conflict.conflict_bounds.minimum.y.value(),
                                         conflict.conflict_bounds.maximum.y.value())}};
            distance += geometry::point_distance_nm(portal, center);
          }
          return distance;
        };
        std::stable_sort(portals.begin(), portals.end(), [&](Point lhs, Point rhs) {
          const auto lhs_pressure = pressure_distance(lhs);
          const auto rhs_pressure = pressure_distance(rhs);
          if (lhs_pressure != rhs_pressure) {
            return lhs_pressure > rhs_pressure;
          }
          return lhs < rhs;
        });
        portals.erase(std::unique(portals.begin(), portals.end()), portals.end());
        const auto limit = result.configured.max_terminal_choices_per_terminal - 1U;
        if (portals.size() > limit) {
          portals.resize(limit);
        }
        for (const auto portal : portals) {
          const auto duplicate =
              std::find_if(result.terminal_choices.begin(), result.terminal_choices.end(),
                           [&](const auto& choice) {
                             return choice.footprint == found->second.first &&
                                    choice.pad == found->second.second &&
                                    choice.net == fragment->id.net &&
                                    choice.layer == fragment->layer && choice.portal == portal;
                           });
          if (duplicate == result.terminal_choices.end()) {
            result.terminal_choices.push_back(RegionalTerminalChoice{result.terminal_choices.size(),
                                                                     found->second.first,
                                                                     found->second.second,
                                                                     fragment->id.net,
                                                                     endpoint,
                                                                     portal,
                                                                     fragment->layer,
                                                                     {endpoint, portal},
                                                                     fragment->width,
                                                                     std::nullopt,
                                                                     false});
          }
        }
      };
      append_terminal(fragment->start, fragment->begins_at_terminal);
      append_terminal(fragment->end, fragment->ends_at_terminal);
    }
  }
  for (const auto& transition : selected_transitions) {
    const auto& path = *transition.path;
    const auto rules = resolve_net_routing_rules(board, path.net);
    const auto& first = path.points[transition.begin_point_index];
    const auto& last = path.points[transition.end_point_index];
    const auto region_id = result.via_choices.size();
    const auto& span = transition.span;
    const auto step = std::max<std::int64_t>(1, result.configured.navmesh_cell_pitch.value());
    std::vector<Point> candidate_sites;
    candidate_sites.reserve(4'225U);
    candidate_sites.push_back(first.position);
    for (std::int64_t y = -32; y <= 32; ++y) {
      for (std::int64_t x = -32; x <= 32; ++x) {
        if (x == 0 && y == 0) {
          continue;
        }
        candidate_sites.push_back(Point{Nanometres{first.position.x.value() + x * step},
                                        Nanometres{first.position.y.value() + y * step}});
      }
    }
    const auto planned_margin = [&](Point site) {
      auto margin = std::numeric_limits<long double>::infinity();
      for (const auto& planned : planned_via_sites) {
        if (planned.net == path.net ||
            std::none_of(span.begin(), span.end(), [&](const auto& layer) {
              return std::find(planned.span.begin(), planned.span.end(), layer) !=
                     planned.span.end();
            })) {
          continue;
        }
        const auto required =
            static_cast<long double>(rules.via_diameter.value() + planned.diameter.value()) / 2.0L +
            static_cast<long double>(std::max(rules.clearance, planned.clearance).value());
        margin = std::min(margin, geometry::point_distance_nm(site, planned.site) - required);
      }
      return margin;
    };
    struct RankedViaSite {
      Point site;
      long double planned_margin{0.0L};
      long double source_distance{0.0L};
    };
    std::vector<RankedViaSite> ranked_sites;
    ranked_sites.reserve(candidate_sites.size() - 1U);
    for (auto site = std::next(candidate_sites.begin()); site != candidate_sites.end(); ++site) {
      ranked_sites.push_back(RankedViaSite{*site, planned_margin(*site),
                                           geometry::point_distance_nm(*site, first.position)});
    }
    std::stable_sort(
        ranked_sites.begin(), ranked_sites.end(), [](const auto& lhs, const auto& rhs) {
          const auto lhs_clears = lhs.planned_margin >= 0.0L;
          const auto rhs_clears = rhs.planned_margin >= 0.0L;
          if (lhs_clears != rhs_clears) {
            return lhs_clears;
          }
          if (lhs_clears && lhs.source_distance != rhs.source_distance) {
            return lhs.source_distance < rhs.source_distance;
          }
          return lhs.planned_margin != rhs.planned_margin ? lhs.planned_margin > rhs.planned_margin
                                                          : lhs.site < rhs.site;
        });
    candidate_sites.resize(1U);
    for (const auto& ranked : ranked_sites) {
      candidate_sites.push_back(ranked.site);
    }
    const auto original_needs_relocation = planned_margin(first.position) < 0.0L;
    std::vector<std::pair<LayerId, Point>> attached_runups;
    if (transition.begin_point_index > 0U &&
        path.points[transition.begin_point_index - 1U].layer == first.layer &&
        path.points[transition.begin_point_index - 1U].position != first.position) {
      attached_runups.emplace_back(first.layer,
                                   path.points[transition.begin_point_index - 1U].position);
    }
    if (transition.end_point_index + 1U < path.points.size() &&
        path.points[transition.end_point_index + 1U].layer == last.layer &&
        path.points[transition.end_point_index + 1U].position != last.position) {
      attached_runups.emplace_back(last.layer,
                                   path.points[transition.end_point_index + 1U].position);
    }
    const auto attached_runup_geometry =
        [&](Point site) -> std::optional<std::vector<RegionalViaRunup>> {
      std::vector<RegionalViaRunup> selected;
      selected.reserve(attached_runups.size());
      for (const auto& runup : attached_runups) {
        std::vector<std::vector<Point>> candidates{
            {runup.second, site},
            {runup.second, Point{site.x, runup.second.y}, site},
            {runup.second, Point{runup.second.x, site.y}, site}};
        for (auto& candidate : candidates) {
          candidate.erase(std::unique(candidate.begin(), candidate.end()), candidate.end());
        }
        std::erase_if(candidates, [&](const auto& candidate) {
          if (candidate.size() < 2U) {
            return true;
          }
          for (std::size_t edge = 1U; edge < candidate.size(); ++edge) {
            if (!runup_legal(path.net, runup.first, candidate[edge - 1U], candidate[edge],
                             rules.track_width)) {
              return true;
            }
          }
          return false;
        });
        if (candidates.empty()) {
          return std::nullopt;
        }
        const auto length = [](const std::vector<Point>& candidate) {
          std::uint64_t total = 0U;
          for (std::size_t edge = 1U; edge < candidate.size(); ++edge) {
            const auto amount = geometry::segment_length_nm(candidate[edge - 1U], candidate[edge]);
            total = total > std::numeric_limits<std::uint64_t>::max() - amount
                        ? std::numeric_limits<std::uint64_t>::max()
                        : total + amount;
          }
          return total;
        };
        std::stable_sort(candidates.begin(), candidates.end(),
                         [&](const auto& lhs, const auto& rhs) {
                           const auto lhs_length = length(lhs);
                           const auto rhs_length = length(rhs);
                           return lhs_length != rhs_length ? lhs_length < rhs_length : lhs < rhs;
                         });
        selected.push_back(RegionalViaRunup{runup.first, runup.second, candidates.front()});
      }
      return selected;
    };
    std::vector<RegionalViaRunup> source_runups;
    source_runups.reserve(attached_runups.size());
    for (const auto& runup : attached_runups) {
      source_runups.push_back(
          RegionalViaRunup{runup.first, runup.second, {runup.second, first.position}});
    }
    const auto minimum_site_diversity = bounded_add(rules.via_diameter, rules.clearance);
    std::size_t region_candidate_checks = 0U;
    for (const auto site : candidate_sites) {
      if (site != first.position && !original_needs_relocation) {
        break;
      }
      if (result.via_choices.size() - region_id >= result.configured.max_via_sites_per_region) {
        break;
      }
      if (site == first.position) {
        result.via_choices.push_back(RegionalViaChoice{
            result.via_choices.size(), region_id, path.net, site, span, rules.via_diameter,
            rules.via_drill, source_runups, true, !original_needs_relocation});
        continue;
      }
      if (std::any_of(result.via_choices.begin(), result.via_choices.end(),
                      [&](const auto& retained) {
                        return retained.region_id == region_id &&
                               geometry::point_distance_nm(site, retained.site) <
                                   static_cast<long double>(minimum_site_diversity.value());
                      })) {
        ++result.via_site_diversity_rejections;
        continue;
      }
      if (selected_via_site_candidate_checks >=
          result.configured.max_selected_via_site_candidate_checks) {
        result.selected_via_site_candidate_budget_exhausted = true;
        break;
      }
      if (region_candidate_checks >= result.configured.max_selected_via_site_checks_per_region) {
        break;
      }
      ++selected_via_site_candidate_checks;
      ++region_candidate_checks;
      ++result.via_site_candidate_checks;
      const auto runups = attached_runup_geometry(site);
      if (!via_site_legal(path.net, site, span, rules.via_diameter) || !runups) {
        continue;
      }
      result.via_choices.push_back(RegionalViaChoice{result.via_choices.size(), region_id, path.net,
                                                     site, span, rules.via_diameter,
                                                     rules.via_drill, *runups, false});
    }
  }
}

[[nodiscard]] std::vector<LayerId> regional_via_span(const Board& board, const LayerId& first,
                                                     const LayerId& last) {
  std::vector<std::pair<std::uint32_t, LayerId>> layers;
  const auto first_layer =
      std::find_if(board.layers.begin(), board.layers.end(),
                   [&](const auto& l) { return l.id == first && l.kind == LayerKind::copper; });
  const auto last_layer =
      std::find_if(board.layers.begin(), board.layers.end(),
                   [&](const auto& l) { return l.id == last && l.kind == LayerKind::copper; });
  if (first_layer == board.layers.end() || last_layer == board.layers.end()) {
    return {};
  }
  const auto low = std::min(first_layer->stack_order, last_layer->stack_order);
  const auto high = std::max(first_layer->stack_order, last_layer->stack_order);
  for (const auto& layer : board.layers) {
    if (layer.kind == LayerKind::copper && layer.stack_order >= low && layer.stack_order <= high) {
      layers.emplace_back(layer.stack_order, layer.id);
    }
  }
  std::sort(layers.begin(), layers.end());
  std::vector<LayerId> result;
  result.reserve(layers.size());
  for (auto& [order, layer] : layers) {
    static_cast<void>(order);
    result.push_back(std::move(layer));
  }
  return result;
}

struct RegionalAlternativeDomain {
  PreparedNavMeshDomain navmesh;
  Nanometres minimum_boundary_clearance;
};

struct RegionalAlternativeGeometry {
  std::vector<OffsetFreeSpaceRegion> regions;
  Nanometres minimum_boundary_clearance;
};

using RegionalAlternativeGeometryKey = std::tuple<NetId, LayerId, Nanometres, bool, Point, Point>;
using RegionalAlternativeDomainKey = std::tuple<NetId, LayerId, Nanometres, bool, Point, Point>;
struct RegionalAlternativeDomainCache {
  std::map<RegionalAlternativeGeometryKey, RegionalAlternativeGeometry> geometry;
  std::map<RegionalAlternativeDomainKey, std::vector<RegionalAlternativeDomain>> navigation;

  void release_unretained_if_oversized() {
    // Bound whole-board Boolean products and search-specific lattices without throwing away every
    // adjacent fragment's same-net/layer reuse. Accepted templates own shared immutable cell sets,
    // so clearing the scratch portfolio cannot invalidate retained candidates.
    constexpr std::size_t maximum_geometry_entries = 8U;
    constexpr std::size_t maximum_navigation_entries = 32U;
    if (geometry.size() > maximum_geometry_entries ||
        navigation.size() > maximum_navigation_entries) {
      navigation.clear();
      geometry.clear();
    }
  }
};

[[nodiscard]] std::optional<geometry::AxisAlignedBounds>
regional_polygon_bounds(const Polygon& polygon) {
  if (polygon.vertices.empty()) {
    return std::nullopt;
  }
  geometry::AxisAlignedBounds result{polygon.vertices.front(), polygon.vertices.front()};
  for (const auto point : polygon.vertices) {
    result.minimum.x = std::min(result.minimum.x, point.x);
    result.minimum.y = std::min(result.minimum.y, point.y);
    result.maximum.x = std::max(result.maximum.x, point.x);
    result.maximum.y = std::max(result.maximum.y, point.y);
  }
  return result;
}

[[nodiscard]] std::optional<FreeSpaceDomain>
local_regional_free_space(const FreeSpaceDomain& source,
                          const geometry::AxisAlignedBounds& search_bounds, Nanometres envelope,
                          Nanometres cell_pitch) {
  const auto outline_bounds = regional_polygon_bounds(source.outline);
  if (!outline_bounds) {
    return std::nullopt;
  }
  const auto positive_pitch = Nanometres{std::max<std::int64_t>(0, cell_pitch.value())};
  const auto margin = bounded_add(envelope, bounded_add(positive_pitch, positive_pitch));
  const auto local_bounds = expanded_bounds(search_bounds, margin, *outline_bounds);
  if (local_bounds.minimum.x >= local_bounds.maximum.x ||
      local_bounds.minimum.y >= local_bounds.maximum.y) {
    return std::nullopt;
  }
  Polygon local_outline{
      {local_bounds.minimum, Point{local_bounds.maximum.x, local_bounds.minimum.y},
       local_bounds.maximum, Point{local_bounds.minimum.x, local_bounds.maximum.y}}};
  // A rectangular local domain is used only when its complete boundary is proven inside the source
  // outline. Irregular/concave edge windows retain the full-board conservative fallback.
  if (!geometry::polygon_inside(local_outline, source.outline)) {
    return std::nullopt;
  }
  const auto overlaps_local_bounds = [&](const Polygon& polygon) {
    const auto bounds = regional_polygon_bounds(polygon);
    return bounds && bounds->overlaps(local_bounds);
  };
  FreeSpaceDomain result;
  result.outline = std::move(local_outline);
  result.obstacles.reserve(source.cutouts.size() + source.obstacles.size());
  // A board cutout intersecting the local rectangle is equivalent to an ordinary blocked polygon
  // for this zero-offset local Boolean. Keeping it as a cutout would require clipping it to the
  // artificial outline; obstacles may legally cross that outline and remain conservative.
  std::copy_if(source.cutouts.begin(), source.cutouts.end(), std::back_inserter(result.obstacles),
               overlaps_local_bounds);
  std::copy_if(source.obstacles.begin(), source.obstacles.end(),
               std::back_inserter(result.obstacles), overlaps_local_bounds);
  std::copy_if(source.filled_obstacles.begin(), source.filled_obstacles.end(),
               std::back_inserter(result.filled_obstacles),
               [&](const auto& region) { return overlaps_local_bounds(region.area); });
  return result;
}

[[nodiscard]] const std::vector<RegionalAlternativeDomain>&
prepared_alternative_domains(const Board& board, const NetId& net, const LayerId& layer,
                             Nanometres envelope, const geometry::AxisAlignedBounds& search_bounds,
                             RegionalRouteResult& result, RegionalAlternativeDomainCache& cache) {
  const RegionalAlternativeDomainKey key{net,
                                         layer,
                                         envelope,
                                         result.configured.treat_saved_zone_fills_as_refillable,
                                         search_bounds.minimum,
                                         search_bounds.maximum};
  const auto found = cache.navigation.find(key);
  if (found != cache.navigation.end()) {
    return found->second;
  }
  const RegionalAlternativeGeometryKey geometry_key{
      net,
      layer,
      envelope,
      result.configured.treat_saved_zone_fills_as_refillable,
      search_bounds.minimum,
      search_bounds.maximum};
  auto geometry = cache.geometry.find(geometry_key);
  if (geometry == cache.geometry.end()) {
    RegionalAlternativeGeometry prepared_geometry;
    const auto extracted = extract_polygonal_board_free_space(
        board,
        BoardFreeSpaceExtractionConfig{layer, net, true, true,
                                       !result.configured.treat_saved_zone_fills_as_refillable});
    if (extracted.complete()) {
      auto domain = local_regional_free_space(extracted.domain, search_bounds, envelope,
                                              result.configured.navmesh_cell_pitch);
      auto offset = offset_polygonal_free_space(domain ? *domain : extracted.domain,
                                                FreeSpaceOffsetConfig{Nanometres{0}, 200'000U});
      prepared_geometry.minimum_boundary_clearance = envelope;
      if (offset.ok()) {
        prepared_geometry.regions = std::move(offset.regions);
      }
    }
    geometry = cache.geometry.emplace(geometry_key, std::move(prepared_geometry)).first;
  }
  std::vector<RegionalAlternativeDomain> domains;
  for (const auto& region : geometry->second.regions) {
    const NavMeshConfig navmesh_config{result.configured.navmesh_cell_pitch,
                                       result.configured.max_navmesh_cells_per_template,
                                       result.configured.max_navmesh_expansions_per_template,
                                       {},
                                       Nanometres{0},
                                       1U,
                                       geometry->second.minimum_boundary_clearance,
                                       search_bounds};
    // These regions came directly from offset_polygonal_free_space(), which already performed the
    // exact topology validation. Avoid repeating quadratic simplicity/disjointness checks for each
    // bounded regional endpoint domain.
    auto prepared = prepare_validated_navmesh_domain(region, navmesh_config);
    if (prepared.ok()) {
      domains.push_back(RegionalAlternativeDomain{std::move(prepared),
                                                  geometry->second.minimum_boundary_clearance});
    }
  }
  return cache.navigation.emplace(key, std::move(domains)).first->second;
}

[[nodiscard]] bool regional_domain_contains_point(const RegionalAlternativeDomain& domain,
                                                  Point point) {
  if (geometry::locate_point(domain.navmesh.prepared_outline, point) ==
          geometry::PointLocation::outside ||
      std::any_of(domain.navmesh.prepared_cutouts.begin(), domain.navmesh.prepared_cutouts.end(),
                  [&](const auto& hole) {
                    return geometry::locate_point(hole, point) != geometry::PointLocation::outside;
                  })) {
    return false;
  }
  auto clearance =
      geometry::point_polygon_boundary_distance_nm(point, domain.navmesh.prepared_outline);
  for (const auto& hole : domain.navmesh.prepared_cutouts) {
    clearance = std::min(clearance, geometry::point_polygon_boundary_distance_nm(point, hole));
  }
  return clearance >= static_cast<long double>(domain.minimum_boundary_clearance.value());
}

[[nodiscard]] bool
regional_domains_contain_point(const std::vector<RegionalAlternativeDomain>& domains, Point point) {
  return std::any_of(domains.begin(), domains.end(), [&](const auto& domain) {
    return regional_domain_contains_point(domain, point);
  });
}

[[nodiscard]] bool
regional_domains_contain_chord(const std::vector<RegionalAlternativeDomain>& domains, Point first,
                               Point second) {
  if (first == second) {
    return regional_domains_contain_point(domains, first);
  }
  return std::any_of(domains.begin(), domains.end(), [&](const auto& domain) {
    // The domain was validated and its immutable boundary BVHs were built once by
    // prepare_navmesh_domain(). Reconstructing FreeSpaceDomain and invoking
    // assess_free_space_chord() here re-ran O(n^2) polygon simplicity validation for every
    // endpoint probe. Endpoint inclusion plus a positive prepared-boundary certificate proves
    // that the complete chord stays inside the outline and outside every hole.
    if (!regional_domain_contains_point(domain, first) ||
        !regional_domain_contains_point(domain, second)) {
      return false;
    }
    const auto required = static_cast<long double>(domain.minimum_boundary_clearance.value());
    if (geometry::segment_polygon_boundary_distance_nm(
            first, second, domain.navmesh.prepared_outline) < required) {
      return false;
    }
    return std::all_of(
        domain.navmesh.prepared_cutouts.begin(), domain.navmesh.prepared_cutouts.end(),
        [&](const auto& hole) {
          return geometry::locate_point(hole, first) == geometry::PointLocation::outside &&
                 geometry::locate_point(hole, second) == geometry::PointLocation::outside &&
                 geometry::segment_polygon_boundary_distance_nm(first, second, hole) >= required;
        });
  });
}

[[nodiscard]] std::vector<RegionalRouteTemplate> build_alternative_layer_templates(
    const Board& board, const GlobalRouteResult& global, const RegionalRouteFragment& fragment,
    RegionalRouteResult& result, RegionalAlternativeDomainCache& domain_cache,
    std::size_t check_limit, std::size_t site_ring = 1U, std::size_t template_limit = 0U,
    bool joint_repair = false, bool local_bypass_only = false) {
  std::vector<RegionalRouteTemplate> alternatives;
  // A selected narrow width is authorized by its original terminal or graph-resource evidence.
  // Moving it to another layer would discard that certificate, so regional search keeps its
  // topology and lets neighboring nominal-width routes take the alternative layer instead.
  if (fragment.explicit_width) {
    return alternatives;
  }
  auto& template_checks = local_bypass_only ? result.local_layer_bypass_checks
                          : joint_repair    ? result.joint_repair_template_checks
                                            : result.alternative_template_checks;
  auto& template_budget_exhausted = local_bypass_only ? result.local_layer_bypass_budget_exhausted
                                    : joint_repair ? result.joint_repair_template_budget_exhausted
                                                   : result.alternative_template_budget_exhausted;
  const auto configured_check_limit =
      local_bypass_only ? result.configured.max_local_layer_bypass_checks
      : joint_repair    ? result.configured.max_joint_alternative_template_checks
                        : result.configured.max_alternative_template_checks;
  check_limit = std::min(check_limit, configured_check_limit);
  template_limit =
      template_limit == 0U ? result.configured.max_templates_per_fragment : template_limit;
  site_ring = std::max(std::size_t{1}, site_ring);
  if (template_checks >= check_limit) {
    template_budget_exhausted = template_checks >= configured_check_limit;
    return alternatives;
  }
  const auto rules = resolve_net_routing_rules(board, fragment.id.net);
  const auto envelope = bounded_add(rules.clearance, Nanometres{rules.track_width.value() / 2});
  std::optional<geometry::AxisAlignedBounds> search_bounds;
  std::optional<std::pair<long double, long double>> conflict_parameter_range;
  const auto fragment_dx =
      static_cast<long double>(fragment.end.x.value() - fragment.start.x.value());
  const auto fragment_dy =
      static_cast<long double>(fragment.end.y.value() - fragment.start.y.value());
  const auto fragment_length_squared = fragment_dx * fragment_dx + fragment_dy * fragment_dy;
  const auto projected_parameter = [&](Point point) {
    if (!(fragment_length_squared > 0.0L)) {
      return 0.0L;
    }
    return std::clamp(
        (static_cast<long double>(point.x.value() - fragment.start.x.value()) * fragment_dx +
         static_cast<long double>(point.y.value() - fragment.start.y.value()) * fragment_dy) /
            fragment_length_squared,
        0.0L, 1.0L);
  };
  for (const auto& window : result.windows) {
    if (std::find(window.fragments.begin(), window.fragments.end(), fragment.id) ==
        window.fragments.end()) {
      continue;
    }
    search_bounds = search_bounds ? union_bounds(*search_bounds, window.bounds) : window.bounds;
    for (const auto edge_id : window.conflict_edges) {
      if (edge_id >= result.conflict_edges.size()) {
        continue;
      }
      const auto& conflict = result.conflict_edges[edge_id];
      if (conflict.first != fragment.id && conflict.second != fragment.id) {
        continue;
      }
      const std::array<Point, 4U> corners{
          conflict.conflict_bounds.minimum,
          Point{conflict.conflict_bounds.maximum.x, conflict.conflict_bounds.minimum.y},
          conflict.conflict_bounds.maximum,
          Point{conflict.conflict_bounds.minimum.x, conflict.conflict_bounds.maximum.y}};
      auto first = 1.0L;
      auto last = 0.0L;
      for (const auto corner : corners) {
        const auto parameter = projected_parameter(corner);
        first = std::min(first, parameter);
        last = std::max(last, parameter);
      }
      if (!conflict_parameter_range) {
        conflict_parameter_range = std::pair{first, last};
      } else {
        conflict_parameter_range->first = std::min(conflict_parameter_range->first, first);
        conflict_parameter_range->second = std::max(conflict_parameter_range->second, last);
      }
    }
  }
  if (!search_bounds) {
    return alternatives;
  }
  std::vector<std::pair<std::uint32_t, LayerId>> layers;
  for (const auto& layer : board.layers) {
    if (layer.kind == LayerKind::copper && layer.id != fragment.layer) {
      layers.emplace_back(layer.stack_order, layer.id);
    }
  }
  std::sort(layers.begin(), layers.end());
  const auto candidate_sites = [&](Point center, Point toward, const std::vector<LayerId>& span) {
    const auto step = std::max<std::int64_t>(rules.via_diameter.value(),
                                             result.configured.navmesh_cell_pitch.value());
    std::vector<Point> sites{center};
    for (std::size_t multiplier = 1U; multiplier <= site_ring; ++multiplier) {
      const auto offset = static_cast<std::int64_t>(multiplier) * step;
      sites.insert(
          sites.end(),
          {Point{Nanometres{center.x.value() + offset}, center.y},
           Point{Nanometres{center.x.value() - offset}, center.y},
           Point{center.x, Nanometres{center.y.value() + offset}},
           Point{center.x, Nanometres{center.y.value() - offset}},
           Point{Nanometres{center.x.value() + offset}, Nanometres{center.y.value() + offset}},
           Point{Nanometres{center.x.value() + offset}, Nanometres{center.y.value() - offset}},
           Point{Nanometres{center.x.value() - offset}, Nanometres{center.y.value() + offset}},
           Point{Nanometres{center.x.value() - offset}, Nanometres{center.y.value() - offset}}});
    }
    const auto length = geometry::point_distance_nm(center, toward);
    if (length > 0.0L) {
      for (const auto multiplier : {1.0L, 2.0L}) {
        sites.push_back(interpolate_quantized(
            center, toward, std::min(1.0L, multiplier * static_cast<long double>(step) / length)));
      }
    }
    std::sort(sites.begin(), sites.end());
    sites.erase(std::unique(sites.begin(), sites.end()), sites.end());
    std::vector<Point> bounded;
    for (const auto site : sites) {
      if (geometry::locate_point(board.outline, site) != geometry::PointLocation::outside) {
        bounded.push_back(site);
      }
    }
    const auto selected_via_margin = [&](Point site) {
      auto margin = std::numeric_limits<long double>::infinity();
      for (const auto& existing : result.via_choices) {
        if (!existing.selected_global_site || existing.net == fragment.id.net ||
            std::none_of(span.begin(), span.end(), [&](const auto& layer) {
              return std::find(existing.span.begin(), existing.span.end(), layer) !=
                     existing.span.end();
            })) {
          continue;
        }
        const auto required =
            static_cast<long double>(rules.via_diameter.value() + existing.diameter.value()) /
                2.0L +
            static_cast<long double>(
                std::max(rules.clearance, resolve_net_routing_rules(board, existing.net).clearance)
                    .value());
        margin = std::min(margin, geometry::point_distance_nm(site, existing.site) - required);
      }
      return margin;
    };
    std::stable_sort(bounded.begin(), bounded.end(), [&](Point lhs, Point rhs) {
      const auto lhs_margin = selected_via_margin(lhs);
      const auto rhs_margin = selected_via_margin(rhs);
      if (lhs_margin != rhs_margin) {
        return lhs_margin > rhs_margin;
      }
      const auto lhs_distance = geometry::point_distance_nm(lhs, center);
      const auto rhs_distance = geometry::point_distance_nm(rhs, center);
      return lhs_distance != rhs_distance ? lhs_distance < rhs_distance : lhs < rhs;
    });
    return bounded;
  };
  const auto first_region_id = [&]() {
    std::size_t next = 0U;
    for (const auto& choice : result.via_choices) {
      next = std::max(next, choice.region_id + 1U);
    }
    return next;
  };
  std::size_t layer_sequences = 0U;
  for (const auto& [stack_order, layer] : layers) {
    static_cast<void>(stack_order);
    if (alternatives.size() + 1U >= template_limit ||
        layer_sequences + 1U >= result.configured.max_layer_sequences_per_fragment ||
        layer_sequences >= result.configured.max_via_regions_per_fragment) {
      break;
    }
    const auto span = regional_via_span(board, fragment.layer, layer);
    if (span.size() < 2U) {
      continue;
    }
    const auto& domains = prepared_alternative_domains(board, fragment.id.net, layer, envelope,
                                                       *search_bounds, result, domain_cache);
    if (domains.empty()) {
      ++result.alternative_domain_rejections;
      continue;
    }
    const auto& original_track_domains = prepared_alternative_domains(
        board, fragment.id.net, fragment.layer, envelope, *search_bounds, result, domain_cache);
    const auto via_envelope =
        bounded_add(rules.clearance, Nanometres{rules.via_diameter.value() / 2});
    const auto vias_fit_span = [&](Point site) {
      return std::all_of(span.begin(), span.end(), [&](const auto& span_layer) {
        const auto& via_domains = prepared_alternative_domains(
            board, fragment.id.net, span_layer, via_envelope, *search_bounds, result, domain_cache);
        return regional_domains_contain_point(via_domains, site);
      });
    };
    const auto legal_endpoint_sites = [&](Point endpoint, Point toward) {
      std::vector<Point> sites;
      // A whole-fragment layer change is often needlessly expensive and may move a via into a
      // terminal escape. Prefer a bounded local overpass around the conflict interval: preserve
      // the selected global route on the original layer, transition shortly before the physical
      // conflict, and return shortly after it. The existing template representation already
      // carries those original-layer run-ups transactionally.
      const auto endpoint_sites =
          local_bypass_only ? std::vector<Point>{} : candidate_sites(endpoint, toward, span);
      sites.insert(sites.end(), endpoint_sites.begin(), endpoint_sites.end());
      if (local_bypass_only && conflict_parameter_range && fragment_length_squared > 0.0L) {
        const auto fragment_length = std::sqrt(fragment_length_squared);
        const auto local_step = static_cast<long double>(
            std::max<std::int64_t>(result.configured.navmesh_cell_pitch.value(),
                                   rules.via_diameter.value() + 2 * rules.clearance.value()));
        const auto entry = endpoint == fragment.start;
        for (std::size_t multiplier = 1U; multiplier <= site_ring; ++multiplier) {
          const auto delta = static_cast<long double>(multiplier) * local_step / fragment_length;
          const auto parameter = entry ? conflict_parameter_range->first - delta
                                       : conflict_parameter_range->second + delta;
          if (parameter > 0.0L && parameter < 1.0L) {
            sites.push_back(interpolate_quantized(fragment.start, fragment.end, parameter));
          }
        }
      }
      // Preserve insertion order. The ordinary portfolio keeps its longstanding endpoint choices
      // first; the independent local-bypass wave contains conflict-bracketing sites only.
      std::vector<Point> unique_sites;
      unique_sites.reserve(sites.size());
      for (const auto site : sites) {
        if (std::find(unique_sites.begin(), unique_sites.end(), site) == unique_sites.end()) {
          unique_sites.push_back(site);
        }
      }
      sites = std::move(unique_sites);
      std::erase_if(sites, [&](Point site) {
        if (!regional_domains_contain_chord(original_track_domains, endpoint, site)) {
          ++result.alternative_runup_rejections;
          return true;
        }
        if (!vias_fit_span(site)) {
          ++result.alternative_via_span_rejections;
          return true;
        }
        return false;
      });
      return sites;
    };
    const auto start_sites = legal_endpoint_sites(fragment.start, fragment.end);
    const auto end_sites = legal_endpoint_sites(fragment.end, fragment.start);
    if (start_sites.empty() || end_sites.empty()) {
      continue;
    }
    const auto start_region = first_region_id();
    const auto end_region = start_region + 1U;
    bool accepted_on_layer = false;
    for (const auto start_site : start_sites) {
      for (const auto end_site : end_sites) {
        if (alternatives.size() + 1U >= template_limit) {
          break;
        }
        if (template_checks >= check_limit) {
          template_budget_exhausted = template_checks >= configured_check_limit;
          break;
        }
        ++template_checks;
        ++result.via_site_candidate_checks;
        // The two transitions must bracket a non-empty forward run. This is also required for
        // replacement materialization to retain monotonic original-layer run-ups.
        if (local_bypass_only && projected_parameter(start_site) >= projected_parameter(end_site)) {
          ++result.alternative_runup_rejections;
          continue;
        }
        std::optional<NavMeshRouteResult> navmesh;
        for (const auto& domain : domains) {
          if (!regional_domain_contains_point(domain, start_site) ||
              !regional_domain_contains_point(domain, end_site)) {
            continue;
          }
          const NavMeshConfig navmesh_config{result.configured.navmesh_cell_pitch,
                                             result.configured.max_navmesh_cells_per_template,
                                             result.configured.max_navmesh_expansions_per_template,
                                             {},
                                             Nanometres{0},
                                             1U,
                                             domain.minimum_boundary_clearance,
                                             *search_bounds};
          auto candidate =
              route_prepared_navmesh(domain.navmesh, start_site, end_site, navmesh_config);
          if (candidate.ok()) {
            navmesh = std::move(candidate);
            break;
          }
        }
        if (!navmesh || navmesh->centerline.size() < 2U) {
          ++result.alternative_navmesh_rejections;
          continue;
        }
        const auto append_via_choice = [&](std::size_t region, Point site) {
          const auto found = std::find_if(
              result.via_choices.begin(), result.via_choices.end(), [&](const auto& choice) {
                return choice.region_id == region && choice.net == fragment.id.net &&
                       choice.site == site && choice.span == span;
              });
          if (found != result.via_choices.end()) {
            return found->id;
          }
          const auto id = result.via_choices.size();
          result.via_choices.push_back(RegionalViaChoice{id,
                                                         region,
                                                         fragment.id.net,
                                                         site,
                                                         span,
                                                         rules.via_diameter,
                                                         rules.via_drill,
                                                         {},
                                                         false});
          return id;
        };
        const auto start_via_choice = append_via_choice(start_region, start_site);
        const auto end_via_choice = append_via_choice(end_region, end_site);
        const auto append_terminal_choice =
            [&](Point endpoint, Point portal, bool is_terminal,
                std::size_t via_choice) -> std::optional<std::size_t> {
          if (!is_terminal) {
            return std::nullopt;
          }
          for (const auto& footprint : board.footprints) {
            for (const auto& pad : footprint.pads) {
              if (pad.net != fragment.id.net ||
                  geometry::transform_point(pad.local_position, footprint.position,
                                            footprint.rotation, footprint.side) != endpoint) {
                continue;
              }
              const auto found =
                  std::find_if(result.terminal_choices.begin(), result.terminal_choices.end(),
                               [&](const auto& choice) {
                                 return choice.footprint == footprint.id && choice.pad == pad.id &&
                                        choice.portal == portal && choice.via_choice == via_choice;
                               });
              if (found != result.terminal_choices.end()) {
                return found->id;
              }
              const auto id = result.terminal_choices.size();
              std::vector<Point> path{endpoint};
              if (portal != endpoint) {
                path.push_back(portal);
              }
              result.terminal_choices.push_back(RegionalTerminalChoice{
                  id, footprint.id, pad.id, fragment.id.net, endpoint, portal, fragment.layer,
                  std::move(path), rules.track_width, via_choice, false});
              return id;
            }
          }
          return std::nullopt;
        };
        const auto terminal_entry_choice = append_terminal_choice(
            fragment.start, start_site, fragment.begins_at_terminal, start_via_choice);
        const auto terminal_exit_choice = append_terminal_choice(
            fragment.end, end_site, fragment.ends_at_terminal, end_via_choice);
        RegionalChannel channel;
        channel.id = result.channels.size();
        channel.layer = layer;
        channel.portals = navmesh->portals;
        channel.topology_signature = navmesh->topology_signature;
        channel.occupants = {fragment.id};
        for (const auto cell_id : navmesh->topology_guard_cells) {
          if (cell_id < navmesh->cells.size()) {
            channel.topology_cells.push_back(navmesh->cells[cell_id].polygon());
          }
        }
        if (channel.topology_cells.empty()) {
          ++result.alternative_topology_cell_rejections;
          continue;
        }
        const auto append_endpoint_link = [&](Point endpoint, std::size_t cell_id) {
          if (cell_id >= navmesh->cells.size()) {
            return;
          }
          const auto& cell = navmesh->cells[cell_id];
          const auto radius = Nanometres{static_cast<std::int64_t>(
              std::ceil(geometry::point_distance_nm(cell.center, cell.corners.front())))};
          channel.endpoint_links.push_back(RegionalTopologyLink{endpoint, cell.center, radius});
        };
        append_endpoint_link(start_site, navmesh->corridor.front());
        append_endpoint_link(end_site, navmesh->corridor.back());
        const auto first = navmesh->centerline.front();
        const auto last = navmesh->centerline.back();
        const auto dx = static_cast<long double>(last.x.value() - first.x.value());
        const auto dy = static_cast<long double>(last.y.value() - first.y.value());
        const auto length = std::hypotl(dx, dy);
        if (!(length > 0.0L)) {
          continue;
        }
        const auto usable = bounded_add(rules.track_width, Nanometres{2 * rules.clearance.value()});
        const auto middle_x = (static_cast<long double>(first.x.value()) + last.x.value()) / 2.0L;
        const auto middle_y = (static_cast<long double>(first.y.value()) + last.y.value()) / 2.0L;
        const auto normal_x = -dy / length;
        const auto normal_y = dx / length;
        const auto half = static_cast<long double>(usable.value()) / 2.0L;
        const Point first_wall{
            Nanometres{static_cast<std::int64_t>(std::llround(middle_x - half * normal_x))},
            Nanometres{static_cast<std::int64_t>(std::llround(middle_y - half * normal_y))}};
        const Point second_wall{
            Nanometres{static_cast<std::int64_t>(std::llround(middle_x + half * normal_x))},
            Nanometres{static_cast<std::int64_t>(std::llround(middle_y + half * normal_y))}};
        auto cross_section = pack_regional_cross_section(result.fragments, channel.occupants, layer,
                                                         first_wall, second_wall, usable);
        cross_section.id = 0U;
        channel.cross_sections.push_back(std::move(cross_section));
        result.channels.push_back(std::move(channel));

        RegionalRouteTemplate route_template;
        route_template.fragment = fragment.id;
        route_template.routing_layer = layer;
        route_template.channel_sequence = {result.channels.back().id};
        route_template.layer_sequence = {fragment.layer, layer, fragment.layer};
        route_template.via_choices = {start_via_choice, end_via_choice};
        route_template.entry_via_choice = start_via_choice;
        route_template.exit_via_choice = end_via_choice;
        route_template.terminal_entry_choice = terminal_entry_choice;
        route_template.terminal_exit_choice = terminal_exit_choice;
        route_template.topology_centerline = navmesh->centerline;
        route_template.topology_signature = navmesh->topology_signature;
        route_template.lower_bound_length_nm =
            geometry::segment_length_nm(fragment.start, start_site) +
            geometry::segment_length_nm(end_site, fragment.end);
        for (std::size_t edge = 1U; edge < navmesh->centerline.size(); ++edge) {
          const auto edge_length = geometry::segment_length_nm(navmesh->centerline[edge - 1U],
                                                               navmesh->centerline[edge]);
          route_template.lower_bound_length_nm =
              route_template.lower_bound_length_nm >
                      std::numeric_limits<std::uint64_t>::max() - edge_length
                  ? std::numeric_limits<std::uint64_t>::max()
                  : route_template.lower_bound_length_nm + edge_length;
        }
        route_template.topology_cost = route_template.lower_bound_length_nm;
        route_template.selected_global_topology = false;
        route_template.exact_static_checked = true;
        alternatives.push_back(std::move(route_template));
        ++result.accepted_alternative_templates;
        result.joint_repair_accepted_templates += joint_repair && !local_bypass_only ? 1U : 0U;
        result.accepted_local_layer_bypasses += local_bypass_only ? 1U : 0U;
        accepted_on_layer = true;
        if (alternatives.size() + 1U >= template_limit || template_budget_exhausted ||
            result.via_site_candidate_budget_exhausted) {
          break;
        }
      }
    }
    if (accepted_on_layer) {
      ++layer_sequences;
    }
    if (template_budget_exhausted || result.via_site_candidate_budget_exhausted) {
      break;
    }
  }
  static_cast<void>(global);
  return alternatives;
}

[[nodiscard]] std::vector<RegionalRouteTemplate> build_same_layer_detour_templates(
    const Board& board, const RegionalRoutingWindow& window, const RegionalRouteFragment& fragment,
    RegionalRouteResult& result, RegionalAlternativeDomainCache& domain_cache) {
  std::vector<RegionalRouteTemplate> alternatives;
  if (fragment.explicit_width || window.conflict_edges.empty() ||
      result.same_layer_detour_checks >= result.configured.max_same_layer_detour_checks) {
    return alternatives;
  }
  struct AttributedBlocker {
    const RegionalRouteFragment* fragment{nullptr};
    std::size_t conflict_edge{0};
    long double required_nm{0.0L};
  };
  std::vector<AttributedBlocker> blockers;
  std::optional<geometry::AxisAlignedBounds> conflict_bounds;
  for (const auto edge_id : window.conflict_edges) {
    if (edge_id >= result.conflict_edges.size()) {
      continue;
    }
    const auto& conflict = result.conflict_edges[edge_id];
    if (conflict.first != fragment.id && conflict.second != fragment.id) {
      continue;
    }
    const auto other_id = conflict.first == fragment.id ? conflict.second : conflict.first;
    const auto other =
        std::find_if(result.fragments.begin(), result.fragments.end(),
                     [&](const auto& candidate) { return candidate.id == other_id; });
    if (other == result.fragments.end() || other->layer != fragment.layer) {
      continue;
    }
    const auto required =
        static_cast<long double>(fragment.width.value() + other->width.value()) / 2.0L +
        static_cast<long double>(std::max(fragment.clearance, other->clearance).value()) +
        static_cast<long double>(result.configured.navmesh_cell_pitch.value());
    blockers.push_back(AttributedBlocker{&*other, edge_id, required});
    conflict_bounds = conflict_bounds ? union_bounds(*conflict_bounds, conflict.conflict_bounds)
                                      : conflict.conflict_bounds;
  }
  if (blockers.empty() || !conflict_bounds) {
    return alternatives;
  }
  const auto dx = static_cast<long double>(fragment.end.x.value() - fragment.start.x.value());
  const auto dy = static_cast<long double>(fragment.end.y.value() - fragment.start.y.value());
  const auto length = std::hypotl(dx, dy);
  if (!(length > 0.0L)) {
    return alternatives;
  }
  const auto rules = resolve_net_routing_rules(board, fragment.id.net);
  const auto envelope = bounded_add(rules.clearance, Nanometres{rules.track_width.value() / 2});
  auto search_bounds = union_bounds(window.bounds, segment_bounds(fragment.start, fragment.end));
  const auto& domains = prepared_alternative_domains(board, fragment.id.net, fragment.layer,
                                                     envelope, search_bounds, result, domain_cache);
  if (domains.empty()) {
    return alternatives;
  }
  const auto conflict_center = Point{Nanometres{std::midpoint(conflict_bounds->minimum.x.value(),
                                                              conflict_bounds->maximum.x.value())},
                                     Nanometres{std::midpoint(conflict_bounds->minimum.y.value(),
                                                              conflict_bounds->maximum.y.value())}};
  const auto required =
      std::max_element(blockers.begin(), blockers.end(), [](const auto& lhs, const auto& rhs) {
        return lhs.required_nm < rhs.required_nm;
      })->required_nm;
  const auto normal = SplineVector{-dy / length, dx / length};
  for (const auto sign : {-1.0L, 1.0L}) {
    if (result.same_layer_detour_checks >= result.configured.max_same_layer_detour_checks ||
        alternatives.size() + 1U >= result.configured.max_templates_per_fragment) {
      break;
    }
    ++result.same_layer_detour_checks;
    const auto waypoint = Point{
        Nanometres{static_cast<std::int64_t>(std::llround(
            static_cast<long double>(conflict_center.x.value()) + sign * normal.x * required))},
        Nanometres{static_cast<std::int64_t>(std::llround(
            static_cast<long double>(conflict_center.y.value()) + sign * normal.y * required))}};
    for (const auto& domain : domains) {
      if (!regional_domain_contains_point(domain, fragment.start) ||
          !regional_domain_contains_point(domain, waypoint) ||
          !regional_domain_contains_point(domain, fragment.end)) {
        continue;
      }
      const NavMeshConfig navmesh_config{result.configured.navmesh_cell_pitch,
                                         result.configured.max_navmesh_cells_per_template,
                                         result.configured.max_navmesh_expansions_per_template,
                                         {},
                                         Nanometres{0},
                                         1U,
                                         domain.minimum_boundary_clearance,
                                         search_bounds};
      std::vector<std::size_t> dynamic_walls;
      const auto half_pitch =
          static_cast<long double>(result.configured.navmesh_cell_pitch.value()) / 2.0L;
      for (const auto& cell : domain.navmesh.cells) {
        if (cell.contains(fragment.start) || cell.contains(waypoint) ||
            cell.contains(fragment.end)) {
          continue;
        }
        if (std::any_of(blockers.begin(), blockers.end(), [&](const auto& blocker) {
              const auto wall_distance = blocker.required_nm + std::hypot(half_pitch, half_pitch);
              return geometry::point_segment_distance_nm(cell.center, blocker.fragment->start,
                                                         blocker.fragment->end) < wall_distance;
            })) {
          dynamic_walls.push_back(cell.id);
        }
      }
      auto entry = route_prepared_navmesh_flood_fill(domain.navmesh, fragment.start, {waypoint},
                                                     navmesh_config, dynamic_walls);
      auto exit = route_prepared_navmesh_flood_fill(domain.navmesh, waypoint, {fragment.end},
                                                    navmesh_config, dynamic_walls);
      if (!entry.ok() || !exit.ok()) {
        continue;
      }
      RegionalChannel channel;
      channel.id = result.channels.size();
      channel.layer = fragment.layer;
      channel.occupants = {fragment.id};
      const auto append_route_evidence = [&](const NavMeshRouteResult& route) {
        channel.portals.insert(channel.portals.end(), route.portals.begin(), route.portals.end());
        channel.topology_signature.gates.insert(channel.topology_signature.gates.end(),
                                                 route.topology_signature.gates.begin(),
                                                 route.topology_signature.gates.end());
        for (const auto cell_id : route.topology_guard_cells) {
          if (cell_id < route.cells.size()) {
            channel.topology_cells.push_back(route.cells[cell_id].polygon());
          }
        }
      };
      append_route_evidence(entry);
      append_route_evidence(exit);
      if (channel.topology_cells.empty()) {
        continue;
      }
      auto centerline = entry.centerline;
      centerline.insert(centerline.end(), exit.centerline.begin() + 1, exit.centerline.end());
      bool clears_attributed_fragments = true;
      for (std::size_t edge = 1U; edge < centerline.size(); ++edge) {
        if (std::any_of(blockers.begin(), blockers.end(), [&](const auto& blocker) {
              return geometry::segment_distance_nm(centerline[edge - 1U], centerline[edge],
                                                   blocker.fragment->start,
                                                   blocker.fragment->end) < blocker.required_nm;
            })) {
          clears_attributed_fragments = false;
          break;
        }
      }
      if (!clears_attributed_fragments) {
        continue;
      }
      const auto append_endpoint_link = [&](Point endpoint, const NavMeshRouteResult& route,
                                            std::size_t cell_id) {
        if (cell_id >= route.cells.size()) {
          return;
        }
        const auto& cell = route.cells[cell_id];
        const auto radius = Nanometres{static_cast<std::int64_t>(
            std::ceil(geometry::point_distance_nm(cell.center, cell.corners.front())))};
        channel.endpoint_links.push_back(RegionalTopologyLink{endpoint, cell.center, radius});
      };
      append_endpoint_link(fragment.start, entry, entry.corridor.front());
      append_endpoint_link(fragment.end, exit, exit.corridor.back());
      channel.topology_signature =
          make_navmesh_topology_signature(std::move(channel.topology_signature.gates));
      const auto usable = bounded_add(fragment.width, Nanometres{2 * fragment.clearance.value()});
      const auto middle_x = (static_cast<long double>(fragment.start.x.value()) +
                             static_cast<long double>(fragment.end.x.value())) /
                            2.0L;
      const auto middle_y = (static_cast<long double>(fragment.start.y.value()) +
                             static_cast<long double>(fragment.end.y.value())) /
                            2.0L;
      const auto half = static_cast<long double>(usable.value()) / 2.0L;
      const Point first_wall{
          Nanometres{static_cast<std::int64_t>(std::llround(middle_x + normal.x * half))},
          Nanometres{static_cast<std::int64_t>(std::llround(middle_y + normal.y * half))}};
      const Point second_wall{
          Nanometres{static_cast<std::int64_t>(std::llround(middle_x - normal.x * half))},
          Nanometres{static_cast<std::int64_t>(std::llround(middle_y - normal.y * half))}};
      auto cross_section = pack_regional_cross_section(
          result.fragments, channel.occupants, fragment.layer, first_wall, second_wall, usable);
      cross_section.id = 0U;
      channel.cross_sections.push_back(std::move(cross_section));
      result.channels.push_back(std::move(channel));

      RegionalRouteTemplate route_template;
      route_template.fragment = fragment.id;
      route_template.routing_layer = fragment.layer;
      route_template.channel_sequence = {result.channels.back().id};
      route_template.layer_sequence = {fragment.layer};
      for (const auto& blocker : blockers) {
        route_template.avoided_conflict_edges.push_back(blocker.conflict_edge);
      }
      std::sort(route_template.avoided_conflict_edges.begin(),
                route_template.avoided_conflict_edges.end());
      route_template.topology_centerline = std::move(centerline);
      route_template.topology_signature = result.channels.back().topology_signature;
      for (std::size_t edge = 1U; edge < route_template.topology_centerline.size(); ++edge) {
        const auto edge_length =
            geometry::segment_length_nm(route_template.topology_centerline[edge - 1U],
                                        route_template.topology_centerline[edge]);
        route_template.lower_bound_length_nm =
            route_template.lower_bound_length_nm >
                    std::numeric_limits<std::uint64_t>::max() - edge_length
                ? std::numeric_limits<std::uint64_t>::max()
                : route_template.lower_bound_length_nm + edge_length;
      }
      route_template.topology_cost = route_template.lower_bound_length_nm;
      route_template.selected_global_topology = false;
      route_template.exact_static_checked = true;
      alternatives.push_back(std::move(route_template));
      ++result.accepted_same_layer_detours;
      break;
    }
  }
  return alternatives;
}

void populate_via_region_evidence(const Board& board, RegionalRouteResult& result) {
  result.via_region_evidence.clear();
  result.contested_via_regions = 0U;
  result.peak_via_region_contention = 0U;
  std::map<std::pair<NetId, NetId>, Nanometres> pairwise_clearances;
  const auto pairwise_clearance = [&](NetId first, NetId second) {
    if (second < first) {
      std::swap(first, second);
    }
    const auto key = std::pair{first, second};
    const auto found = pairwise_clearances.find(key);
    if (found != pairwise_clearances.end()) {
      return found->second;
    }
    return pairwise_clearances.emplace(key, resolve_pairwise_clearance(board, first, second))
        .first->second;
  };
  std::map<std::pair<NetId, std::size_t>, std::vector<const RegionalViaChoice*>> grouped;
  for (const auto& choice : result.via_choices) {
    grouped[{choice.net, choice.region_id}].push_back(&choice);
  }
  result.via_region_evidence.reserve(grouped.size());
  for (auto& [key, choices] : grouped) {
    std::sort(choices.begin(), choices.end(),
              [](const auto* lhs, const auto* rhs) { return lhs->id < rhs->id; });
    RegionalViaRegionEvidence evidence;
    evidence.region_id = key.second;
    evidence.net = key.first;
    evidence.span = choices.front()->span;
    for (const auto* choice : choices) {
      evidence.candidate_choices.push_back(choice->id);
      evidence.selected_global_candidates += choice->selected_global_site ? 1U : 0U;
      evidence.alternative_search_deferred =
          evidence.alternative_search_deferred || choice->alternative_search_deferred;
    }
    result.via_region_evidence.push_back(std::move(evidence));
  }
  for (std::size_t first = 0U; first < result.via_region_evidence.size(); ++first) {
    auto& lhs = result.via_region_evidence[first];
    for (std::size_t second = first + 1U; second < result.via_region_evidence.size(); ++second) {
      auto& rhs = result.via_region_evidence[second];
      if (lhs.net == rhs.net) {
        continue;
      }
      const auto span_overlap =
          std::any_of(lhs.span.begin(), lhs.span.end(), [&](const auto& layer) {
            return std::find(rhs.span.begin(), rhs.span.end(), layer) != rhs.span.end();
          });
      if (!span_overlap) {
        continue;
      }
      ++lhs.overlapping_span_regions;
      ++rhs.overlapping_span_regions;
      std::size_t conflicts = 0U;
      for (const auto lhs_id : lhs.candidate_choices) {
        const auto& lhs_choice = result.via_choices[lhs_id];
        for (const auto rhs_id : rhs.candidate_choices) {
          const auto& rhs_choice = result.via_choices[rhs_id];
          conflicts +=
              regional_via_choices_conflict(lhs_choice, rhs_choice,
                                            pairwise_clearance(lhs_choice.net, rhs_choice.net))
                  ? 1U
                  : 0U;
        }
      }
      if (conflicts == 0U) {
        continue;
      }
      ++lhs.competing_regions;
      ++rhs.competing_regions;
      lhs.conflicting_candidate_pairs += conflicts;
      rhs.conflicting_candidate_pairs += conflicts;
      lhs.contested = true;
      rhs.contested = true;
    }
  }
  for (auto& evidence : result.via_region_evidence) {
    for (const auto choice_id : evidence.candidate_choices) {
      const auto& choice = result.via_choices[choice_id];
      for (const auto& terminal : result.terminal_choices) {
        if (regional_terminal_via_conflict(terminal, choice,
                                           pairwise_clearance(terminal.net, choice.net))) {
          ++evidence.terminal_escape_conflicts;
          evidence.contested = true;
        }
      }
    }
    result.contested_via_regions += evidence.contested ? 1U : 0U;
    result.peak_via_region_contention =
        std::max(result.peak_via_region_contention, evidence.competing_regions);
  }
}

using RegionalFragmentIndex = std::map<RegionalRouteFragmentId, const RegionalRouteFragment*>;

[[nodiscard]] std::vector<RegionalRouteFragmentId>
geometric_cross_section_order_indexed(const RegionalFragmentIndex& fragments,
                                      const std::vector<RegionalRouteFragmentId>& occupants,
                                      Point first_wall, Point second_wall) {
  if (second_wall < first_wall) {
    std::swap(first_wall, second_wall);
  }
  const auto wall_dx = static_cast<long double>(second_wall.x.value() - first_wall.x.value());
  const auto wall_dy = static_cast<long double>(second_wall.y.value() - first_wall.y.value());
  const auto wall_length_squared = wall_dx * wall_dx + wall_dy * wall_dy;
  if (!(wall_length_squared > 0.0L)) {
    auto ordered = occupants;
    std::sort(ordered.begin(), ordered.end());
    return ordered;
  }
  struct LocatedOccupant {
    RegionalRouteFragmentId id;
    long double coordinate{0.0L};
  };
  std::vector<LocatedOccupant> located;
  located.reserve(occupants.size());
  for (const auto& id : occupants) {
    const auto found = fragments.find(id);
    if (found == fragments.end()) {
      located.push_back(LocatedOccupant{id, std::numeric_limits<long double>::infinity()});
      continue;
    }
    const auto& fragment = *found->second;
    const auto fragment_dx =
        static_cast<long double>(fragment.end.x.value() - fragment.start.x.value());
    const auto fragment_dy =
        static_cast<long double>(fragment.end.y.value() - fragment.start.y.value());
    const auto offset_x = static_cast<long double>(fragment.start.x.value() - first_wall.x.value());
    const auto offset_y = static_cast<long double>(fragment.start.y.value() - first_wall.y.value());
    const auto denominator = wall_dx * fragment_dy - wall_dy * fragment_dx;
    long double coordinate = 0.0L;
    if (denominator != 0.0L &&
        geometry::segments_intersect(first_wall, second_wall, fragment.start, fragment.end)) {
      coordinate = (offset_x * fragment_dy - offset_y * fragment_dx) / denominator;
    } else {
      const auto midpoint_x =
          (static_cast<long double>(fragment.start.x.value()) + fragment.end.x.value()) / 2.0L;
      const auto midpoint_y =
          (static_cast<long double>(fragment.start.y.value()) + fragment.end.y.value()) / 2.0L;
      coordinate = ((midpoint_x - static_cast<long double>(first_wall.x.value())) * wall_dx +
                    (midpoint_y - static_cast<long double>(first_wall.y.value())) * wall_dy) /
                   wall_length_squared;
    }
    located.push_back(LocatedOccupant{id, coordinate});
  }
  std::sort(located.begin(), located.end(), [](const auto& first, const auto& second) {
    return first.coordinate != second.coordinate ? first.coordinate < second.coordinate
                                                 : first.id < second.id;
  });
  std::vector<RegionalRouteFragmentId> ordered;
  ordered.reserve(located.size());
  std::transform(located.begin(), located.end(), std::back_inserter(ordered),
                 [](const auto& occupant) { return occupant.id; });
  return ordered;
}

[[nodiscard]] RegionalCrossSection
pack_cross_section_indexed(const RegionalFragmentIndex& fragments,
                           const std::vector<RegionalRouteFragmentId>& order, LayerId layer,
                           Point first_wall, Point second_wall, Nanometres usable_width) {
  if (second_wall < first_wall) {
    std::swap(first_wall, second_wall);
  }
  RegionalCrossSection result;
  result.layer = std::move(layer);
  result.first_wall = first_wall;
  result.second_wall = second_wall;
  result.usable_width = usable_width;
  result.ordered_occupants = order;
  std::vector<const RegionalRouteFragment*> selected;
  selected.reserve(order.size());
  for (const auto& id : order) {
    const auto found = fragments.find(id);
    if (found == fragments.end() || found->second->width.value() <= 0 ||
        found->second->clearance.value() < 0) {
      return result;
    }
    selected.push_back(found->second);
  }
  if (selected.empty() || usable_width.value() <= 0) {
    return result;
  }
  std::int64_t packed = selected.front()->clearance.value();
  for (std::size_t index = 0U; index < selected.size(); ++index) {
    const auto width = selected[index]->width.value();
    if (packed > std::numeric_limits<std::int64_t>::max() - width) {
      return result;
    }
    packed += width;
    if (index + 1U < selected.size()) {
      const auto gap =
          std::max(selected[index]->clearance, selected[index + 1U]->clearance).value();
      if (packed > std::numeric_limits<std::int64_t>::max() - gap) {
        return result;
      }
      packed += gap;
    }
  }
  if (packed > std::numeric_limits<std::int64_t>::max() - selected.back()->clearance.value()) {
    return result;
  }
  packed += selected.back()->clearance.value();
  result.packed_width = Nanometres{packed};
  if (packed > usable_width.value()) {
    return result;
  }
  result.physically_fits = true;
  auto cursor = -(packed / 2) + selected.front()->clearance.value();
  for (std::size_t index = 0U; index < selected.size(); ++index) {
    cursor += selected[index]->width.value() / 2;
    result.placements.push_back(RegionalLanePlacement{order[index], cursor});
    cursor += selected[index]->width.value() - selected[index]->width.value() / 2;
    if (index + 1U < selected.size()) {
      cursor += std::max(selected[index]->clearance, selected[index + 1U]->clearance).value();
    }
  }
  return result;
}

struct PreparedLaneCrossSection {
  const RegionalCrossSection* source{nullptr};
  std::vector<RegionalRouteFragmentId> geometric_order;
  std::size_t stable_order_inversions{0};
};

struct LanePackingSolveResult {
  std::optional<RegionalCrossSection> packing;
  std::size_t states{0};
  std::size_t pruned_states{0};
  std::size_t analytically_rejected_cross_sections{0};
  bool budget_exhausted{false};
  bool complete_search{false};
};

struct LaneOrderSearchResult {
  std::vector<RegionalRouteFragmentId> order;
  std::size_t inversions{std::numeric_limits<std::size_t>::max()};
  std::size_t states{0};
  std::size_t pruned_states{0};
  bool budget_exhausted{false};
};

// Finds the minimum-inversion order that fits one physical cross section. For the same placed set
// and last occupant, future width and inversion increments depend only on that state, so a Pareto
// frontier over (partial width, inversions) replaces factorial complete-order enumeration.
[[maybe_unused, nodiscard]] LaneOrderSearchResult solve_lane_order_dynamic_program(
    const RegionalFragmentIndex& fragments, const std::vector<RegionalRouteFragmentId>& occupants,
    const std::vector<RegionalRouteFragmentId>& geometric_order, Nanometres usable_width,
    std::size_t maximum_states, const std::function<void(std::size_t)>& progress) {
  LaneOrderSearchResult result;
  const auto count = occupants.size();
  if (count == 0U || count > 63U || maximum_states == 0U || usable_width.value() <= 0) {
    result.budget_exhausted = count > 63U || maximum_states == 0U;
    return result;
  }

  std::vector<std::uint64_t> widths(count, 0U);
  std::vector<std::uint64_t> clearances(count, 0U);
  std::vector<std::size_t> reference_position(count, 0U);
  std::vector<std::optional<std::size_t>> coupled_rank(count);
  std::vector<std::uint64_t> inversion_predecessors(count, 0U);
  std::map<RegionalRouteFragmentId, std::size_t> local_index;
  long double total_occupant_width = 0.0L;
  for (std::size_t index = 0U; index < count; ++index) {
    local_index.emplace(occupants[index], index);
    const auto fragment = fragments.find(occupants[index]);
    if (fragment == fragments.end() || fragment->second->width.value() <= 0 ||
        fragment->second->clearance.value() < 0) {
      return result;
    }
    widths[index] = static_cast<std::uint64_t>(fragment->second->width.value());
    clearances[index] = static_cast<std::uint64_t>(fragment->second->clearance.value());
    total_occupant_width += static_cast<long double>(widths[index]);
  }
  if (geometric_order.size() != count) {
    return result;
  }
  for (std::size_t position = 0U; position < geometric_order.size(); ++position) {
    const auto found = local_index.find(geometric_order[position]);
    if (found == local_index.end()) {
      return result;
    }
    reference_position[found->second] = position;
  }
  std::size_t coupled_count = 0U;
  std::uint64_t coupled_mask = 0U;
  for (std::size_t index = 0U; index < count; ++index) {
    const auto fragment = fragments.find(occupants[index]);
    if (fragment != fragments.end() && fragment->second->coupled_member) {
      coupled_rank[index] = coupled_count++;
      coupled_mask |= std::uint64_t{1} << index;
    }
    for (std::size_t predecessor = 0U; predecessor < count; ++predecessor) {
      if (reference_position[predecessor] > reference_position[index]) {
        inversion_predecessors[index] |= std::uint64_t{1} << predecessor;
      }
    }
  }
  const auto minimum_clearance = *std::min_element(clearances.begin(), clearances.end());

  struct Label {
    std::uint64_t width_nm{0U};
    long double remaining_width_nm{0.0L};
    std::size_t inversions{0U};
    std::size_t parent{std::numeric_limits<std::size_t>::max()};
    std::size_t last{0U};
  };
  struct StateKey {
    std::uint64_t mask{0U};
    std::size_t last{0U};

    bool operator==(const StateKey&) const = default;
  };
  struct StateKeyHash {
    std::size_t operator()(const StateKey& key) const noexcept {
      const auto mixed = key.mask ^ (key.mask >> 33U) ^
                         (static_cast<std::uint64_t>(key.last) * 0x9e3779b97f4a7c15ULL);
      return static_cast<std::size_t>(mixed ^ (mixed >> 32U));
    }
  };
  // For one (placed set, last occupant), nondominated labels are ordered by inversions and have
  // strictly decreasing packed width. This makes dominance insertion logarithmic instead of
  // repeatedly scanning a growing vector on heterogeneous-clearance channels.
  using ParetoFrontier = std::map<std::size_t, std::size_t>;
  using FrontierMap = std::unordered_map<StateKey, ParetoFrontier, StateKeyHash>;
  std::vector<FrontierMap> frontiers(count + 1U);
  std::vector<std::vector<StateKey>> frontier_order(count + 1U);
  std::vector<Label> labels;
  labels.reserve(std::min<std::size_t>(maximum_states, 100'000U));
  const auto capacity = static_cast<std::uint64_t>(usable_width.value());
  const auto full_mask =
      count == 64U ? std::numeric_limits<std::uint64_t>::max() : ((std::uint64_t{1} << count) - 1U);
  const auto reconstruct = [&](std::size_t label_index) {
    std::vector<std::size_t> order;
    while (label_index != std::numeric_limits<std::size_t>::max()) {
      order.push_back(labels[label_index].last);
      label_index = labels[label_index].parent;
    }
    std::reverse(order.begin(), order.end());
    return order;
  };
  const auto coupled_candidate_legal = [&](std::uint64_t mask, std::size_t candidate) {
    if (!coupled_rank[candidate]) {
      return true;
    }
    const auto placed_coupled = static_cast<std::size_t>(std::popcount(mask & coupled_mask));
    return *coupled_rank[candidate] == placed_coupled;
  };
  const auto optimistic_fits = [&](std::uint64_t mask, std::uint64_t partial_width,
                                   std::size_t last, long double remaining_width) {
    auto lower = static_cast<long double>(partial_width) + remaining_width;
    const auto remaining = count - static_cast<std::size_t>(std::popcount(mask));
    if (remaining == 0U) {
      lower += static_cast<long double>(clearances[last]);
    } else {
      lower +=
          static_cast<long double>(remaining + 1U) * static_cast<long double>(minimum_clearance);
    }
    return lower <= static_cast<long double>(capacity);
  };
  const auto consume_state = [&]() {
    if (result.states >= maximum_states) {
      result.budget_exhausted = true;
      return false;
    }
    ++result.states;
    if (progress) {
      progress(result.states);
    }
    return true;
  };
  const auto add_label = [&](std::size_t depth, StateKey key, Label label) {
    auto [state, inserted] = frontiers[depth].try_emplace(key);
    if (inserted) {
      frontier_order[depth].push_back(key);
    }
    auto& frontier = state->second;
    auto position = frontier.lower_bound(label.inversions);
    if (position != frontier.end() && position->first == label.inversions) {
      if (labels[position->second].width_nm <= label.width_nm) {
        ++result.pruned_states;
        return;
      }
      position = frontier.erase(position);
    }
    if (position != frontier.begin()) {
      const auto predecessor = std::prev(position);
      if (labels[predecessor->second].width_nm <= label.width_nm) {
        ++result.pruned_states;
        return;
      }
    }
    while (position != frontier.end() && labels[position->second].width_nm >= label.width_nm) {
      position = frontier.erase(position);
    }
    labels.push_back(label);
    frontier.emplace_hint(position, label.inversions, labels.size() - 1U);
  };
  const auto consider_complete = [&](std::size_t label_index) {
    const auto& label = labels[label_index];
    if (clearances[label.last] > capacity || label.width_nm > capacity - clearances[label.last]) {
      ++result.pruned_states;
      return;
    }
    const auto local_order = reconstruct(label_index);
    std::vector<RegionalRouteFragmentId> order;
    order.reserve(local_order.size());
    for (const auto local : local_order) {
      order.push_back(occupants[local]);
    }
    if (result.order.empty() || label.inversions < result.inversions ||
        (label.inversions == result.inversions && order < result.order)) {
      result.order = std::move(order);
      result.inversions = label.inversions;
    }
  };

  for (std::size_t candidate = 0U; candidate < count; ++candidate) {
    if (!consume_state()) {
      return result;
    }
    if (!coupled_candidate_legal(0U, candidate)) {
      ++result.pruned_states;
      continue;
    }
    const auto partial = widths[candidate] + clearances[candidate];
    const auto mask = std::uint64_t{1} << candidate;
    const auto remaining_width = total_occupant_width - static_cast<long double>(widths[candidate]);
    if (!optimistic_fits(mask, partial, candidate, remaining_width)) {
      ++result.pruned_states;
      continue;
    }
    add_label(
        1U, StateKey{mask, candidate},
        Label{partial, remaining_width, 0U, std::numeric_limits<std::size_t>::max(), candidate});
  }
  if (count == 1U) {
    for (const auto& key : frontier_order[1U]) {
      for (const auto& [inversions, label] : frontiers[1U].at(key)) {
        static_cast<void>(inversions);
        consider_complete(label);
      }
    }
    return result;
  }

  for (std::size_t depth = 1U; depth < count && !result.budget_exhausted; ++depth) {
    for (const auto& key : frontier_order[depth]) {
      const auto state = frontiers[depth].find(key);
      if (state == frontiers[depth].end()) {
        continue;
      }
      const auto& source_labels = state->second;
      for (const auto& [inversions, source_index] : source_labels) {
        static_cast<void>(inversions);
        const auto source = labels[source_index];
        for (std::size_t candidate = 0U; candidate < count; ++candidate) {
          const auto bit = std::uint64_t{1} << candidate;
          if ((key.mask & bit) != 0U) {
            continue;
          }
          if (!consume_state()) {
            break;
          }
          if (!coupled_candidate_legal(key.mask, candidate)) {
            ++result.pruned_states;
            continue;
          }
          const auto gap = std::max(clearances[source.last], clearances[candidate]);
          if (source.width_nm >
              std::numeric_limits<std::uint64_t>::max() - gap - widths[candidate]) {
            ++result.pruned_states;
            continue;
          }
          const auto width = source.width_nm + gap + widths[candidate];
          const auto mask = key.mask | bit;
          const auto remaining_width =
              source.remaining_width_nm - static_cast<long double>(widths[candidate]);
          if (!optimistic_fits(mask, width, candidate, remaining_width)) {
            ++result.pruned_states;
            continue;
          }
          const auto added_inversions =
              static_cast<std::size_t>(std::popcount(key.mask & inversion_predecessors[candidate]));
          add_label(depth + 1U, StateKey{mask, candidate},
                    Label{width, remaining_width, source.inversions + added_inversions,
                          source_index, candidate});
        }
        if (result.budget_exhausted) {
          break;
        }
      }
      if (result.budget_exhausted) {
        break;
      }
    }
  }
  // A bounded search can exhaust its budget after already producing complete labels. Retain the
  // best such legal result even though it is not safe to memoize as an optimal answer.
  for (const auto& key : frontier_order[count]) {
    if (key.mask != full_mask) {
      continue;
    }
    for (const auto& [inversions, label] : frontiers[count].at(key)) {
      static_cast<void>(inversions);
      consider_complete(label);
    }
  }
  return result;
}

[[nodiscard]] LanePackingSolveResult solve_prepared_lane_packing(
    const RegionalFragmentIndex& fragments, const std::vector<RegionalRouteFragmentId>& occupants,
    LayerId layer, const std::vector<PreparedLaneCrossSection>& cross_sections,
    std::size_t maximum_states, const std::function<void(std::size_t)>& progress) {
  LanePackingSolveResult result;
  if (occupants.empty() || cross_sections.empty() || maximum_states == 0U) {
    result.budget_exhausted = maximum_states == 0U;
    return result;
  }
  long double minimum_packed_width = 0.0L;
  long double maximum_clearance = 0.0L;
  for (const auto& occupant : occupants) {
    const auto found = fragments.find(occupant);
    if (found == fragments.end() || found->second->width.value() <= 0 ||
        found->second->clearance.value() < 0) {
      return result;
    }
    minimum_packed_width += static_cast<long double>(found->second->width.value()) +
                            static_cast<long double>(found->second->clearance.value());
    maximum_clearance =
        std::max(maximum_clearance, static_cast<long double>(found->second->clearance.value()));
  }
  minimum_packed_width += maximum_clearance;
  const auto analytically_viable = [&](const PreparedLaneCrossSection& prepared) {
    return static_cast<long double>(prepared.source->usable_width.value()) >= minimum_packed_width;
  };
  result.analytically_rejected_cross_sections = static_cast<std::size_t>(
      std::count_if(cross_sections.begin(), cross_sections.end(),
                    [&](const auto& prepared) { return !analytically_viable(prepared); }));
  if (result.analytically_rejected_cross_sections == cross_sections.size()) {
    result.complete_search = true;
    return result;
  }
  const auto preserves_coupled_order = [&](const std::vector<RegionalRouteFragmentId>& order) {
    const auto coupled = [&](const auto& id) {
      const auto found = fragments.find(id);
      return found != fragments.end() && found->second->coupled_member;
    };
    std::vector<RegionalRouteFragmentId> expected;
    std::vector<RegionalRouteFragmentId> actual;
    std::copy_if(occupants.begin(), occupants.end(), std::back_inserter(expected), coupled);
    std::copy_if(order.begin(), order.end(), std::back_inserter(actual), coupled);
    return expected == actual;
  };
  std::optional<std::size_t> best_inversions;
  const auto consider = [&](const std::vector<RegionalRouteFragmentId>& order,
                            const PreparedLaneCrossSection& prepared) {
    if (!analytically_viable(prepared) || !preserves_coupled_order(order)) {
      return;
    }
    auto packed =
        pack_cross_section_indexed(fragments, order, layer, prepared.source->first_wall,
                                   prepared.source->second_wall, prepared.source->usable_width);
    if (!packed.physically_fits) {
      return;
    }
    const auto inversions = inversion_count(prepared.geometric_order, order);
    packed.geometric_reference_order = prepared.geometric_order;
    packed.stable_order_inversions = prepared.stable_order_inversions;
    packed.selected_order_inversions = inversions;
    if (!result.packing || inversions < *best_inversions ||
        (inversions == *best_inversions &&
         packed.ordered_occupants < result.packing->ordered_occupants)) {
      result.packing = std::move(packed);
      best_inversions = inversions;
    }
  };
  const auto consume_seed = [&]() {
    if (result.states >= maximum_states) {
      result.budget_exhausted = true;
      return false;
    }
    ++result.states;
    if (progress) {
      progress(result.states);
    }
    return true;
  };
  std::vector<std::vector<RegionalRouteFragmentId>> seeded_orders;
  for (const auto& prepared : cross_sections) {
    if (!analytically_viable(prepared)) {
      continue;
    }
    if (std::find(seeded_orders.begin(), seeded_orders.end(), prepared.geometric_order) ==
        seeded_orders.end()) {
      seeded_orders.push_back(prepared.geometric_order);
    }
  }
  if (std::find(seeded_orders.begin(), seeded_orders.end(), occupants) == seeded_orders.end()) {
    seeded_orders.push_back(occupants);
  }
  for (const auto& order : seeded_orders) {
    if (!consume_seed()) {
      return result;
    }
    for (const auto& prepared : cross_sections) {
      consider(order, prepared);
    }
  }
  if (best_inversions && *best_inversions == 0U) {
    result.complete_search = true;
    return result;
  }

  const auto homogeneous_clearance = [&]() {
    std::optional<Nanometres> clearance;
    for (const auto& occupant : occupants) {
      const auto found = fragments.find(occupant);
      if (found == fragments.end()) {
        return false;
      }
      if (!clearance) {
        clearance = found->second->clearance;
      } else if (*clearance != found->second->clearance) {
        return false;
      }
    }
    return true;
  }();
  // With homogeneous clearance every permutation has the same packed width. A geometric seed is
  // therefore either the zero-inversion optimum or proof that no order can fit.
  if (homogeneous_clearance) {
    result.complete_search = true;
    return result;
  }

  // Preserve the established lexicographic order choice for every physically plausible cross
  // section. Width dominance alone cannot discard a wider partial order: it may still fit and be
  // the lexicographically preferred complete order used to seed downstream spline geometry. The
  // analytical bound above removes the expensive physically impossible cases; this bounded tail
  // retains exact historical selection semantics for the remaining cases.
  auto order = occupants;
  do {
    if (std::find(seeded_orders.begin(), seeded_orders.end(), order) != seeded_orders.end()) {
      continue;
    }
    if (!consume_seed()) {
      break;
    }
    for (const auto& prepared : cross_sections) {
      consider(order, prepared);
    }
    if (best_inversions && *best_inversions == 0U) {
      result.complete_search = true;
      return result;
    }
  } while (std::next_permutation(order.begin(), order.end()));
  result.complete_search = !result.budget_exhausted;
  return result;
}

void populate_bundle_assignments(const Board& board, const GlobalRouteResult& global,
                                 RegionalRouteResult& result, RouterTelemetrySink* telemetry,
                                 std::string_view phase) {
  populate_selected_endpoint_choices(board, global, result);
  std::map<RegionalRouteFragmentId, std::vector<RegionalRouteTemplate>> alternatives;
  RegionalAlternativeDomainCache domain_cache;
  RegionalFragmentIndex fragment_by_id;
  for (const auto& fragment : result.fragments) {
    fragment_by_id.emplace(fragment.id, &fragment);
  }
  const auto publish_portfolio_progress = [&]() {
    if (telemetry == nullptr) {
      return;
    }
    RouterTelemetryEvent progress;
    progress.kind = RouterTelemetryEventKind::search_summary;
    progress.stable_id = "regional-alternative-progress:" + std::string(phase);
    progress.detail = std::string(phase) + " regional alternative portfolio: checks=" +
                      std::to_string(result.alternative_template_checks) + "/" +
                      std::to_string(result.configured.max_alternative_template_checks) +
                      ", accepted=" + std::to_string(result.accepted_alternative_templates) +
                      ", geometry_cache=" + std::to_string(domain_cache.geometry.size()) +
                      ", navigation_cache=" + std::to_string(domain_cache.navigation.size());
    static_cast<void>(telemetry->try_emit(std::move(progress)));
  };
  const auto publish_lane_progress = [&](std::size_t window_id, std::size_t states,
                                         std::string_view scope) {
    constexpr std::size_t progress_interval = 4096U;
    if (telemetry == nullptr || ((states & (progress_interval - 1U)) != 0U &&
                                 states != result.configured.max_lane_order_states)) {
      return;
    }
    RouterTelemetryEvent progress;
    progress.kind = RouterTelemetryEventKind::search_summary;
    progress.stable_id = "regional-lane-progress:" + std::string(phase) + ':' +
                         std::to_string(window_id) + ':' + std::string(scope);
    progress.detail = std::string(phase) +
                      " regional lane search: window=" + std::to_string(window_id) +
                      ", scope=" + std::string(scope) + ", states=" + std::to_string(states) + '/' +
                      std::to_string(result.configured.max_lane_order_states);
    static_cast<void>(telemetry->try_emit(std::move(progress)));
  };
  struct LanePackingCacheValue {
    std::optional<RegionalCrossSection> packing;
  };
  std::map<std::size_t, std::map<std::vector<RegionalRouteFragmentId>, LanePackingCacheValue>>
      lane_packing_cache;
  const auto solve_channel_lanes =
      [&](const RegionalChannel& channel, const std::vector<RegionalRouteFragmentId>& occupants,
          std::size_t maximum_states, const std::function<void(std::size_t)>& progress) {
        ++result.lane_solver_queries;
        auto& channel_cache = lane_packing_cache[channel.id];
        if (const auto cached = channel_cache.find(occupants); cached != channel_cache.end()) {
          ++result.lane_solver_cache_hits;
          LanePackingSolveResult solved;
          solved.packing = cached->second.packing;
          solved.complete_search = true;
          return solved;
        }
        std::vector<PreparedLaneCrossSection> prepared;
        prepared.reserve(channel.cross_sections.size());
        for (const auto& source : channel.cross_sections) {
          auto geometric_order = geometric_cross_section_order_indexed(
              fragment_by_id, occupants, source.first_wall, source.second_wall);
          prepared.push_back(PreparedLaneCrossSection{&source, geometric_order,
                                                      inversion_count(geometric_order, occupants)});
        }
        auto solved = solve_prepared_lane_packing(fragment_by_id, occupants, channel.layer,
                                                  prepared, maximum_states, progress);
        result.lane_solver_states += solved.states;
        result.lane_solver_pruned_states += solved.pruned_states;
        result.lane_solver_analytical_rejections += solved.analytically_rejected_cross_sections;
        result.lane_solver_budget_exhaustions += solved.budget_exhausted ? 1U : 0U;
        if (solved.complete_search) {
          channel_cache.emplace(occupants, LanePackingCacheValue{solved.packing});
        }
        return solved;
      };
  std::set<RegionalRouteFragmentId> seen_fragments;
  const auto selected_window_has_proper_crossing = [&](const RegionalRoutingWindow& window) {
    if (!window.eligible()) {
      return false;
    }
    for (const auto edge_id : window.conflict_edges) {
      if (edge_id >= result.conflict_edges.size()) {
        continue;
      }
      const auto& edge = result.conflict_edges[edge_id];
      const auto first =
          std::find_if(result.fragments.begin(), result.fragments.end(),
                       [&](const auto& fragment) { return fragment.id == edge.first; });
      const auto second =
          std::find_if(result.fragments.begin(), result.fragments.end(),
                       [&](const auto& fragment) { return fragment.id == edge.second; });
      if (first != result.fragments.end() && second != result.fragments.end() &&
          first->layer == second->layer &&
          proper_regional_segment_intersection(first->start, first->end, second->start,
                                               second->end)) {
        return true;
      }
    }
    return false;
  };
  const auto selected_window_fails_packing = [&](const RegionalRoutingWindow& window) {
    if (!window.eligible()) {
      return false;
    }
    std::map<std::size_t, std::vector<RegionalRouteFragmentId>> channel_occupants;
    for (const auto& fragment_id : window.fragments) {
      bool has_channel = false;
      const auto fragment = std::find_if(
          result.fragments.begin(), result.fragments.end(),
          [&fragment_id](const auto& candidate) { return candidate.id == fragment_id; });
      if (fragment == result.fragments.end()) {
        return true;
      }
      for (const auto& channel : result.channels) {
        if (channel.layer != fragment->layer ||
            std::find(channel.occupants.begin(), channel.occupants.end(), fragment_id) ==
                channel.occupants.end()) {
          continue;
        }
        has_channel = true;
        channel_occupants[channel.id].push_back(fragment_id);
      }
      if (!has_channel) {
        return true;
      }
    }
    for (auto& [channel_id, occupants] : channel_occupants) {
      const auto channel =
          std::find_if(result.channels.begin(), result.channels.end(),
                       [channel_id](const auto& candidate) { return candidate.id == channel_id; });
      if (channel == result.channels.end() || channel->cross_sections.empty()) {
        return true;
      }
      std::sort(occupants.begin(), occupants.end());
      occupants.erase(std::unique(occupants.begin(), occupants.end()), occupants.end());
      const auto fits = std::any_of(
          channel->cross_sections.begin(), channel->cross_sections.end(), [&](const auto& source) {
            return pack_regional_cross_section(result.fragments, occupants, channel->layer,
                                               source.first_wall, source.second_wall,
                                               source.usable_width)
                .physically_fits;
          });
      if (!fits) {
        return true;
      }
    }
    return false;
  };
  std::vector<const RegionalRoutingWindow*> alternative_windows;
  alternative_windows.reserve(result.windows.size());
  for (const auto& window : result.windows) {
    if (window.eligible()) {
      alternative_windows.push_back(&window);
    }
  }
  const auto packing_windows = static_cast<std::size_t>(
      std::count_if(alternative_windows.begin(), alternative_windows.end(),
                    [&](const auto* window) { return selected_window_fails_packing(*window); }));
  std::erase_if(alternative_windows, [&](const auto* window) {
    return packing_windows > 0U ? !selected_window_fails_packing(*window)
                                : !selected_window_has_proper_crossing(*window);
  });
  result.alternative_priority_windows = alternative_windows.size();
  const auto fragment_pressure = [&](const RegionalRoutingWindow& window,
                                     const RegionalRouteFragmentId& fragment) {
    std::uint64_t overlap = 0U;
    std::uint64_t deficit = 0U;
    for (const auto edge_id : window.conflict_edges) {
      const auto& edge = result.conflict_edges[edge_id];
      if (edge.first != fragment && edge.second != fragment) {
        continue;
      }
      overlap =
          overlap > std::numeric_limits<std::uint64_t>::max() - edge.observed_overlap_length_nm
              ? std::numeric_limits<std::uint64_t>::max()
              : overlap + edge.observed_overlap_length_nm;
      deficit = deficit > std::numeric_limits<std::uint64_t>::max() -
                              static_cast<std::uint64_t>(
                                  std::max<std::int64_t>(0, edge.peak_clearance_deficit.value()))
                    ? std::numeric_limits<std::uint64_t>::max()
                    : deficit + static_cast<std::uint64_t>(
                                    std::max<std::int64_t>(0, edge.peak_clearance_deficit.value()));
    }
    return std::pair{overlap, deficit};
  };
  std::stable_sort(alternative_windows.begin(), alternative_windows.end(),
                   [&](const auto* lhs, const auto* rhs) {
                     if (lhs->observed_overlap_length_nm != rhs->observed_overlap_length_nm) {
                       return lhs->observed_overlap_length_nm > rhs->observed_overlap_length_nm;
                     }
                     if (lhs->peak_clearance_deficit != rhs->peak_clearance_deficit) {
                       return lhs->peak_clearance_deficit > rhs->peak_clearance_deficit;
                     }
                     return lhs->id < rhs->id;
                   });
  for (std::size_t window_index = 0U; window_index < alternative_windows.size(); ++window_index) {
    const auto* window_pointer = alternative_windows[window_index];
    const auto& window = *window_pointer;
    const auto priority_window = window_index < result.alternative_priority_windows;
    if (!priority_window) {
      break;
    }
    const auto remaining_priority_windows =
        priority_window ? result.alternative_priority_windows - window_index : 0U;
    const auto remaining_checks =
        result.configured.max_alternative_template_checks - result.alternative_template_checks;
    const auto window_share =
        priority_window && remaining_priority_windows > 0U
            ? (remaining_checks + remaining_priority_windows - 1U) / remaining_priority_windows
            : remaining_checks;
    const auto window_check_limit = result.alternative_template_checks + window_share;
    auto ranked = window.fragments;
    std::stable_sort(ranked.begin(), ranked.end(), [&](const auto& lhs, const auto& rhs) {
      const auto lhs_pressure = fragment_pressure(window, lhs);
      const auto rhs_pressure = fragment_pressure(window, rhs);
      return lhs_pressure != rhs_pressure ? lhs_pressure > rhs_pressure : lhs < rhs;
    });
    std::erase_if(ranked, [&](const auto& fragment) {
      return !std::binary_search(window.owned_fragments.begin(), window.owned_fragments.end(),
                                 fragment) ||
             seen_fragments.contains(fragment);
    });
    for (const auto& fragment : ranked) {
      if (result.alternative_template_checks >= window_check_limit ||
          result.alternative_template_budget_exhausted ||
          result.via_site_candidate_budget_exhausted) {
        break;
      }
      // A shared articulation fragment may first appear in a higher-pressure non-owning window.
      // Filter ownership before recording it as seen, otherwise its actual owner can never build
      // the one global alternative portfolio reused by every window containing that fragment.
      seen_fragments.insert(fragment);
      const auto found =
          std::find_if(result.fragments.begin(), result.fragments.end(),
                       [&fragment](const auto& candidate) { return candidate.id == fragment; });
      if (found != result.fragments.end()) {
        auto generated = build_alternative_layer_templates(board, global, *found, result,
                                                           domain_cache, window_check_limit);
        alternatives.emplace(fragment, std::move(generated));
        domain_cache.release_unretained_if_oversized();
        publish_portfolio_progress();
      }
    }
    if (result.alternative_template_budget_exhausted ||
        result.via_site_candidate_budget_exhausted) {
      break;
    }
  }
  // Packing feasibility gets first refusal on the bounded alternative budget. Once those windows
  // have a viable discrete state, spend otherwise-unused pair checks on the highest-pressure
  // owner windows so the bundle objective can remove same-layer physical conflicts before spline
  // realization. Each residual window receives at most one complete endpoint-pair check in this
  // wave; retained templates remain exact-prevalidated and ordinary bundle negotiation decides
  // whether switching layers is worthwhile.
  if (packing_windows > 0U &&
      result.alternative_template_checks < result.configured.max_alternative_template_checks &&
      !result.alternative_template_budget_exhausted &&
      !result.via_site_candidate_budget_exhausted) {
    std::vector<const RegionalRoutingWindow*> residual_windows;
    for (const auto& window : result.windows) {
      if (window.eligible() && !selected_window_fails_packing(window)) {
        residual_windows.push_back(&window);
      }
    }
    std::stable_sort(residual_windows.begin(), residual_windows.end(),
                     [](const auto* lhs, const auto* rhs) {
                       if (lhs->observed_overlap_length_nm != rhs->observed_overlap_length_nm) {
                         return lhs->observed_overlap_length_nm > rhs->observed_overlap_length_nm;
                       }
                       if (lhs->peak_clearance_deficit != rhs->peak_clearance_deficit) {
                         return lhs->peak_clearance_deficit > rhs->peak_clearance_deficit;
                       }
                       return lhs->id < rhs->id;
                     });
    const auto remaining_checks =
        result.configured.max_alternative_template_checks - result.alternative_template_checks;
    if (residual_windows.size() > remaining_checks) {
      residual_windows.resize(remaining_checks);
    }
    result.alternative_residual_priority_windows = residual_windows.size();
    for (const auto* window_pointer : residual_windows) {
      const auto& window = *window_pointer;
      auto ranked = window.fragments;
      std::stable_sort(ranked.begin(), ranked.end(), [&](const auto& lhs, const auto& rhs) {
        const auto lhs_pressure = fragment_pressure(window, lhs);
        const auto rhs_pressure = fragment_pressure(window, rhs);
        return lhs_pressure != rhs_pressure ? lhs_pressure > rhs_pressure : lhs < rhs;
      });
      std::erase_if(ranked, [&](const auto& fragment) {
        return !std::binary_search(window.owned_fragments.begin(), window.owned_fragments.end(),
                                   fragment) ||
               seen_fragments.contains(fragment);
      });
      const auto window_check_limit = result.alternative_template_checks + 1U;
      for (const auto& fragment : ranked) {
        if (result.alternative_template_checks >= window_check_limit ||
            result.alternative_template_budget_exhausted ||
            result.via_site_candidate_budget_exhausted) {
          break;
        }
        seen_fragments.insert(fragment);
        const auto found =
            std::find_if(result.fragments.begin(), result.fragments.end(),
                         [&](const auto& candidate) { return candidate.id == fragment; });
        if (found != result.fragments.end()) {
          auto generated = build_alternative_layer_templates(board, global, *found, result,
                                                             domain_cache, window_check_limit);
          alternatives.emplace(fragment, std::move(generated));
          domain_cache.release_unretained_if_oversized();
          publish_portfolio_progress();
        }
      }
      if (result.alternative_template_budget_exhausted ||
          result.via_site_candidate_budget_exhausted) {
        break;
      }
    }
    auto followup_windows = residual_windows;
    const auto window_has_retained_alternative = [&](const RegionalRoutingWindow& window) {
      return std::any_of(window.owned_fragments.begin(), window.owned_fragments.end(),
                         [&](const auto& fragment) {
                           const auto found = alternatives.find(fragment);
                           return found != alternatives.end() && !found->second.empty();
                         });
    };
    for (const auto* window_pointer : residual_windows) {
      const auto& window = *window_pointer;
      if (window.conflict_edges.size() != 1U || window_has_retained_alternative(window) ||
          result.same_layer_detour_checks >= result.configured.max_same_layer_detour_checks) {
        continue;
      }
      auto ranked = window.owned_fragments;
      std::stable_sort(ranked.begin(), ranked.end(), [&](const auto& lhs, const auto& rhs) {
        const auto lhs_pressure = fragment_pressure(window, lhs);
        const auto rhs_pressure = fragment_pressure(window, rhs);
        return lhs_pressure != rhs_pressure ? lhs_pressure > rhs_pressure : lhs < rhs;
      });
      for (const auto& fragment_id : ranked) {
        const auto fragment =
            std::find_if(result.fragments.begin(), result.fragments.end(),
                         [&](const auto& candidate) { return candidate.id == fragment_id; });
        if (fragment == result.fragments.end()) {
          continue;
        }
        auto generated =
            build_same_layer_detour_templates(board, window, *fragment, result, domain_cache);
        domain_cache.release_unretained_if_oversized();
        publish_portfolio_progress();
        if (generated.empty()) {
          continue;
        }
        auto& retained = alternatives[fragment_id];
        retained.insert(retained.end(), std::make_move_iterator(generated.begin()),
                        std::make_move_iterator(generated.end()));
        break;
      }
    }
    std::stable_sort(
        followup_windows.begin(), followup_windows.end(), [&](const auto* lhs, const auto* rhs) {
          const auto lhs_key = std::tuple{
              window_has_retained_alternative(*lhs), lhs->conflict_edges.size() != 1U,
              std::numeric_limits<std::uint64_t>::max() - lhs->observed_overlap_length_nm, lhs->id};
          const auto rhs_key = std::tuple{
              window_has_retained_alternative(*rhs), rhs->conflict_edges.size() != 1U,
              std::numeric_limits<std::uint64_t>::max() - rhs->observed_overlap_length_nm, rhs->id};
          return lhs_key < rhs_key;
        });
    const auto followup_checks_begin = result.alternative_template_checks;
    // Some owner windows cannot spend their one-check share because their highest-ranked fragment
    // has no complete endpoint pair. Reuse the remaining global budget on another fragment,
    // prioritizing no-alternative single-conflict windows because clearing one improves the
    // feasibility-first window count. Each round still gives a window at most one check and
    // seen-fragment ownership prevents rediscovery of the same alternative.
    while (result.alternative_template_checks < result.configured.max_alternative_template_checks &&
           !result.alternative_template_budget_exhausted &&
           !result.via_site_candidate_budget_exhausted) {
      bool consumed_check = false;
      bool visited_fragment = false;
      for (const auto* window_pointer : followup_windows) {
        const auto& window = *window_pointer;
        auto ranked = window.fragments;
        std::stable_sort(ranked.begin(), ranked.end(), [&](const auto& lhs, const auto& rhs) {
          const auto lhs_pressure = fragment_pressure(window, lhs);
          const auto rhs_pressure = fragment_pressure(window, rhs);
          return lhs_pressure != rhs_pressure ? lhs_pressure > rhs_pressure : lhs < rhs;
        });
        std::erase_if(ranked, [&](const auto& fragment) {
          return !std::binary_search(window.owned_fragments.begin(), window.owned_fragments.end(),
                                     fragment) ||
                 seen_fragments.contains(fragment);
        });
        if (ranked.empty()) {
          continue;
        }
        visited_fragment = true;
        const auto checks_before = result.alternative_template_checks;
        const auto window_check_limit = checks_before + 1U;
        for (const auto& fragment : ranked) {
          if (result.alternative_template_checks >= window_check_limit ||
              result.alternative_template_budget_exhausted ||
              result.via_site_candidate_budget_exhausted) {
            break;
          }
          seen_fragments.insert(fragment);
          const auto found =
              std::find_if(result.fragments.begin(), result.fragments.end(),
                           [&](const auto& candidate) { return candidate.id == fragment; });
          if (found != result.fragments.end()) {
            auto generated = build_alternative_layer_templates(board, global, *found, result,
                                                               domain_cache, window_check_limit);
            alternatives.emplace(fragment, std::move(generated));
            domain_cache.release_unretained_if_oversized();
            publish_portfolio_progress();
          }
        }
        consumed_check = consumed_check || result.alternative_template_checks > checks_before;
        if (result.alternative_template_budget_exhausted ||
            result.via_site_candidate_budget_exhausted) {
          break;
        }
      }
      if (!visited_fragment || !consumed_check) {
        break;
      }
    }
    result.alternative_residual_followup_checks =
        result.alternative_template_checks - followup_checks_begin;
  }
  populate_via_region_evidence(board, result);
  const auto assemble_window_templates = [&](const RegionalRoutingWindow& window) {
    RegionalBundleAssignment assignment;
    assignment.window_id = window.id;
    assignment.stop_reason = window.eligible() ? RegionalBundleStopReason::no_fitting_lane_order
                                               : RegionalBundleStopReason::window_ineligible;
    if (!window.eligible()) {
      return assignment;
    }
    for (const auto& fragment_id : window.fragments) {
      const auto fragment = std::find_if(
          result.fragments.begin(), result.fragments.end(),
          [&fragment_id](const auto& candidate) { return candidate.id == fragment_id; });
      if (fragment == result.fragments.end()) {
        continue;
      }
      RegionalRouteTemplate route_template;
      route_template.id = assignment.templates.size();
      route_template.fragment = fragment_id;
      route_template.routing_layer = fragment->layer;
      route_template.layer_sequence = {fragment->layer};
      route_template.topology_centerline = {fragment->start, fragment->end};
      route_template.lower_bound_length_nm =
          geometry::segment_length_nm(fragment->start, fragment->end);
      const auto entry_choice = std::find_if(
          result.terminal_choices.begin(), result.terminal_choices.end(), [&](const auto& choice) {
            return choice.net == fragment_id.net && choice.pad_position == fragment->start;
          });
      if (entry_choice != result.terminal_choices.end()) {
        route_template.terminal_entry_choice = entry_choice->id;
      }
      const auto exit_choice = std::find_if(
          result.terminal_choices.begin(), result.terminal_choices.end(), [&](const auto& choice) {
            return choice.net == fragment_id.net && choice.pad_position == fragment->end;
          });
      if (exit_choice != result.terminal_choices.end()) {
        route_template.terminal_exit_choice = exit_choice->id;
      }
      std::set<std::size_t> governed_via_regions;
      for (const auto& via : result.via_choices) {
        if (via.net == fragment_id.net && via.selected_global_site &&
            (via.site == fragment->start || via.site == fragment->end)) {
          governed_via_regions.insert(via.region_id);
        }
      }
      for (const auto& via : result.via_choices) {
        if (via.net == fragment_id.net && via.selected_global_site &&
            governed_via_regions.contains(via.region_id)) {
          route_template.via_choices.push_back(via.id);
          if (via.site == fragment->start && !route_template.entry_via_choice) {
            route_template.entry_via_choice = via.id;
          }
          if (via.site == fragment->end && !route_template.exit_via_choice) {
            route_template.exit_via_choice = via.id;
          }
          route_template.layer_sequence.insert(route_template.layer_sequence.end(),
                                               via.span.begin(), via.span.end());
        }
      }
      std::sort(route_template.layer_sequence.begin(), route_template.layer_sequence.end());
      route_template.layer_sequence.erase(
          std::unique(route_template.layer_sequence.begin(), route_template.layer_sequence.end()),
          route_template.layer_sequence.end());
      for (const auto& portal : window.portals) {
        if (portal.fragment != fragment_id) {
          continue;
        }
        if (portal.position == fragment->start) {
          route_template.entry_portals.push_back(portal.id);
        } else if (portal.position == fragment->end) {
          route_template.exit_portals.push_back(portal.id);
        } else {
          route_template.entry_portals.push_back(portal.id);
          route_template.exit_portals.push_back(portal.id);
        }
      }
      for (const auto& channel : result.channels) {
        if (channel.layer == fragment->layer &&
            std::find(channel.occupants.begin(), channel.occupants.end(), fragment_id) !=
                channel.occupants.end()) {
          route_template.channel_sequence.push_back(channel.id);
        }
      }
      const auto selected_template_index = assignment.templates.size();
      assignment.templates.push_back(std::move(route_template));
      assignment.selected_templates.push_back(assignment.templates.back().id);
      if (result.configured.enable_terminal_portal_templates) {
        // Terminal access is a bounded joint routing decision. Clone the selected global topology
        // with each exact-static-checked same-layer portal so the bundle CSP can choose escapes
        // atomically with lanes and vias. The spline starts/ends at the selected portal; candidate
        // reconstruction retains the pad-to-portal access segment in the same exact transaction.
        const auto terminal_options = [&](Point endpoint) {
          std::vector<std::size_t> options;
          for (const auto& choice : result.terminal_choices) {
            if (choice.net == fragment_id.net && choice.pad_position == endpoint &&
                choice.layer == fragment->layer && !choice.via_choice) {
              options.push_back(choice.id);
            }
          }
          std::sort(options.begin(), options.end());
          if (options.size() > 2U) {
            options.resize(2U);
          }
          return options;
        };
        auto entry_options = fragment->begins_at_terminal ? terminal_options(fragment->start)
                                                          : std::vector<std::size_t>{};
        auto exit_options = fragment->ends_at_terminal ? terminal_options(fragment->end)
                                                       : std::vector<std::size_t>{};
        if (entry_options.empty()) {
          entry_options.push_back(std::numeric_limits<std::size_t>::max());
        }
        if (exit_options.empty()) {
          exit_options.push_back(std::numeric_limits<std::size_t>::max());
        }
        const auto terminal_choice = [&](std::size_t id) -> const RegionalTerminalChoice* {
          if (id == std::numeric_limits<std::size_t>::max()) {
            return nullptr;
          }
          const auto found =
              std::find_if(result.terminal_choices.begin(), result.terminal_choices.end(),
                           [&](const auto& choice) { return choice.id == id; });
          return found == result.terminal_choices.end() ? nullptr : &*found;
        };
        for (const auto entry_id : entry_options) {
          for (const auto exit_id : exit_options) {
            if (assignment.templates.size() - selected_template_index >=
                result.configured.max_templates_per_fragment) {
              break;
            }
            const auto* entry = terminal_choice(entry_id);
            const auto* exit = terminal_choice(exit_id);
            const auto& selected = assignment.templates[selected_template_index];
            const auto selected_entry =
                selected.terminal_entry_choice.value_or(std::numeric_limits<std::size_t>::max());
            const auto selected_exit =
                selected.terminal_exit_choice.value_or(std::numeric_limits<std::size_t>::max());
            if (entry_id == selected_entry && exit_id == selected_exit) {
              continue;
            }
            const auto start = entry == nullptr ? fragment->start : entry->portal;
            const auto end = exit == nullptr ? fragment->end : exit->portal;
            if (start == end) {
              continue;
            }
            auto alternative = selected;
            alternative.id = assignment.templates.size();
            alternative.terminal_entry_choice = entry == nullptr
                                                    ? std::optional<std::size_t>{}
                                                    : std::optional<std::size_t>{entry->id};
            alternative.terminal_exit_choice = exit == nullptr
                                                   ? std::optional<std::size_t>{}
                                                   : std::optional<std::size_t>{exit->id};
            alternative.topology_centerline = {start, end};
            alternative.lower_bound_length_nm = geometry::segment_length_nm(start, end);
            if (entry != nullptr) {
              alternative.lower_bound_length_nm +=
                  geometry::segment_length_nm(entry->pad_position, entry->portal);
            }
            if (exit != nullptr) {
              alternative.lower_bound_length_nm +=
                  geometry::segment_length_nm(exit->pad_position, exit->portal);
            }
            assignment.templates.push_back(std::move(alternative));
          }
        }
      }
      const auto found_alternatives = alternatives.find(fragment_id);
      if (found_alternatives == alternatives.end() ||
          !std::binary_search(window.owned_fragments.begin(), window.owned_fragments.end(),
                              fragment_id)) {
        continue;
      }
      for (auto alternative : found_alternatives->second) {
        alternative.id = assignment.templates.size();
        alternative.entry_portals = assignment.templates[selected_template_index].entry_portals;
        alternative.exit_portals = assignment.templates[selected_template_index].exit_portals;
        if (!alternative.terminal_entry_choice) {
          alternative.terminal_entry_choice =
              assignment.templates[selected_template_index].terminal_entry_choice;
        }
        if (!alternative.terminal_exit_choice) {
          alternative.terminal_exit_choice =
              assignment.templates[selected_template_index].terminal_exit_choice;
        }
        assignment.templates.push_back(std::move(alternative));
      }
    }
    return assignment;
  };
  const auto template_candidates_by_fragment = [](const RegionalRoutingWindow& window,
                                                  const RegionalBundleAssignment& assignment) {
    std::vector<std::vector<std::size_t>> candidates_by_fragment;
    candidates_by_fragment.reserve(window.fragments.size());
    for (const auto& fragment_id : window.fragments) {
      std::vector<std::size_t> candidates;
      for (const auto& route_template : assignment.templates) {
        if (route_template.fragment == fragment_id && !route_template.channel_sequence.empty()) {
          candidates.push_back(route_template.id);
        }
      }
      candidates_by_fragment.push_back(std::move(candidates));
    }
    return candidates_by_fragment;
  };
  struct JointRepairCandidate {
    const RegionalRoutingWindow* window{nullptr};
    RegionalBundleAssignment assignment;
    RegionalBundleChoicePortfolio portfolio;
    bool has_fitting_bundle{false};
  };
  std::vector<JointRepairCandidate> joint_repair_candidates;
  for (const auto& window : result.windows) {
    if (!window.eligible()) {
      continue;
    }
    auto assignment = assemble_window_templates(window);
    const auto candidates_by_fragment = template_candidates_by_fragment(window, assignment);
    if (std::any_of(candidates_by_fragment.begin(), candidates_by_fragment.end(),
                    [](const auto& candidates) { return candidates.empty(); })) {
      continue;
    }
    auto portfolio = generate_consistent_bundle_choices(board, result, window, assignment.templates,
                                                        candidates_by_fragment,
                                                        result.configured.max_bundle_search_states);
    result.partial_lane_capacity_pruned_states += portfolio.lane_capacity_pruned_states;
    bool has_fitting_bundle = false;
    std::size_t lane_states = 0U;
    for (const auto& choice : portfolio.choices) {
      if (lane_states >= result.configured.max_lane_order_states) {
        break;
      }
      std::vector<std::size_t> selected;
      std::map<std::size_t, std::vector<RegionalRouteFragmentId>> channel_occupants;
      for (std::size_t fragment = 0U; fragment < choice.size(); ++fragment) {
        const auto template_id = candidates_by_fragment[fragment][choice[fragment]];
        selected.push_back(template_id);
        for (const auto channel_id : assignment.templates[template_id].channel_sequence) {
          channel_occupants[channel_id].push_back(assignment.templates[template_id].fragment);
        }
      }
      if (!coupled_template_selection_legal(board, window, assignment.templates, selected) ||
          !select_access_geometry(board, result, window, assignment.templates, selected).legal()) {
        continue;
      }
      bool choice_fits = true;
      for (auto& [channel_id, occupants] : channel_occupants) {
        const auto channel = std::find_if(
            result.channels.begin(), result.channels.end(),
            [channel_id](const auto& candidate) { return candidate.id == channel_id; });
        if (channel == result.channels.end() || channel->cross_sections.empty()) {
          choice_fits = false;
          break;
        }
        std::sort(occupants.begin(), occupants.end());
        occupants.erase(std::unique(occupants.begin(), occupants.end()), occupants.end());
        const auto base_states = lane_states;
        const auto solved = solve_channel_lanes(
            *channel, occupants, result.configured.max_lane_order_states - lane_states,
            [&](const auto local_states) {
              publish_lane_progress(window.id, base_states + local_states, "joint-repair-screen");
            });
        lane_states += solved.states;
        if (!solved.packing) {
          choice_fits = false;
          break;
        }
      }
      if (choice_fits) {
        has_fitting_bundle = true;
        break;
      }
    }
    const auto attributed_discrete_failure = !has_fitting_bundle &&
                                             portfolio.via_pair_pruned_states > 0U &&
                                             !portfolio.access_blockers.empty();
    if (attributed_discrete_failure ||
        (has_fitting_bundle && window.observed_overlap_length_nm > 0U)) {
      joint_repair_candidates.push_back(JointRepairCandidate{
          &window, std::move(assignment), std::move(portfolio), has_fitting_bundle});
    }
  }
  std::stable_sort(
      joint_repair_candidates.begin(), joint_repair_candidates.end(),
      [](const auto& lhs, const auto& rhs) {
        if (lhs.has_fitting_bundle != rhs.has_fitting_bundle) {
          return !lhs.has_fitting_bundle;
        }
        const auto lhs_single_conflict = lhs.window->conflict_edges.size() == 1U;
        const auto rhs_single_conflict = rhs.window->conflict_edges.size() == 1U;
        if (lhs_single_conflict != rhs_single_conflict) {
          return lhs_single_conflict;
        }
        if (lhs.portfolio.via_pair_pruned_states != rhs.portfolio.via_pair_pruned_states) {
          return lhs.portfolio.via_pair_pruned_states > rhs.portfolio.via_pair_pruned_states;
        }
        if (lhs.window->observed_overlap_length_nm != rhs.window->observed_overlap_length_nm) {
          return lhs.window->observed_overlap_length_nm > rhs.window->observed_overlap_length_nm;
        }
        return lhs.window->id < rhs.window->id;
      });
  if (joint_repair_candidates.size() > result.configured.max_joint_repair_windows) {
    joint_repair_candidates.resize(result.configured.max_joint_repair_windows);
  }
  std::set<std::size_t> expanded_joint_repair_windows;
  for (std::size_t candidate_index = 0U; candidate_index < joint_repair_candidates.size();
       ++candidate_index) {
    const auto& candidate = joint_repair_candidates[candidate_index];
    ++result.joint_repair_windows_attempted;
    result.joint_repair_window_ids.push_back(candidate.window->id);
    std::map<RegionalRouteFragmentId, std::size_t> blocker_pressure;
    for (const auto& blocker : candidate.portfolio.access_blockers) {
      for (const auto choice_id : {blocker.first_via_choice, blocker.second_via_choice}) {
        if (!choice_id) {
          continue;
        }
        for (const auto& route_template : candidate.assignment.templates) {
          if (route_template.selected_global_topology ||
              std::find(route_template.via_choices.begin(), route_template.via_choices.end(),
                        *choice_id) == route_template.via_choices.end()) {
            continue;
          }
          blocker_pressure[route_template.fragment] += blocker.observations;
        }
      }
    }
    std::vector<std::pair<std::uint64_t, RegionalRouteFragmentId>> ranked_fragments;
    for (const auto& [fragment, pressure] : blocker_pressure) {
      if (std::binary_search(candidate.window->owned_fragments.begin(),
                             candidate.window->owned_fragments.end(), fragment)) {
        ranked_fragments.emplace_back(pressure, fragment);
      }
    }
    if (ranked_fragments.empty()) {
      for (const auto& fragment : candidate.window->owned_fragments) {
        const auto [overlap, deficit] = fragment_pressure(*candidate.window, fragment);
        const auto pressure = overlap > std::numeric_limits<std::uint64_t>::max() - deficit
                                  ? std::numeric_limits<std::uint64_t>::max()
                                  : overlap + deficit;
        if (pressure > 0U) {
          ranked_fragments.emplace_back(pressure, fragment);
        }
      }
    }
    std::stable_sort(
        ranked_fragments.begin(), ranked_fragments.end(), [](const auto& lhs, const auto& rhs) {
          return lhs.first != rhs.first ? lhs.first > rhs.first : lhs.second < rhs.second;
        });
    if (ranked_fragments.size() > result.configured.max_joint_repair_fragments_per_window) {
      ranked_fragments.resize(result.configured.max_joint_repair_fragments_per_window);
    }
    const auto remaining_candidates = joint_repair_candidates.size() - candidate_index;
    const auto remaining_checks = result.configured.max_joint_alternative_template_checks -
                                  std::min(result.configured.max_joint_alternative_template_checks,
                                           result.joint_repair_template_checks);
    const auto fair_share =
        remaining_candidates == 0U
            ? remaining_checks
            : (remaining_checks + remaining_candidates - 1U) / remaining_candidates;
    const auto discrete_failure_reserve =
        (result.configured.max_joint_alternative_template_checks + 2U) / 3U;
    const auto window_share =
        candidate.has_fitting_bundle
            ? fair_share
            : std::min(remaining_checks, std::max(fair_share, discrete_failure_reserve));
    const auto window_check_limit = result.joint_repair_template_checks + window_share;
    bool expanded = false;
    for (const auto& [pressure, fragment_id] : ranked_fragments) {
      static_cast<void>(pressure);
      if (result.joint_repair_template_budget_exhausted) {
        break;
      }
      const auto fragment =
          std::find_if(result.fragments.begin(), result.fragments.end(),
                       [&](const auto& value) { return value.id == fragment_id; });
      if (fragment == result.fragments.end()) {
        continue;
      }
      auto& retained = alternatives[fragment_id];
      if (candidate.window->conflict_edges.size() == 1U &&
          result.same_layer_detour_checks < result.configured.max_same_layer_detour_checks) {
        auto generated = build_same_layer_detour_templates(board, *candidate.window, *fragment,
                                                           result, domain_cache);
        domain_cache.release_unretained_if_oversized();
        publish_portfolio_progress();
        for (auto& route_template : generated) {
          const auto duplicate =
              std::any_of(retained.begin(), retained.end(), [&](const auto& prior) {
                return prior.routing_layer == route_template.routing_layer &&
                       prior.topology_centerline == route_template.topology_centerline;
              });
          if (!duplicate) {
            retained.push_back(std::move(route_template));
            expanded = true;
          }
        }
      }
      auto existing_template_count = retained.size() + 1U;
      // A dense residual window is normally a connected set of several conflict edges.  Reserve
      // a small independent slot for a conflict-bracketing stub/layer/via swap in every joint
      // candidate, not just the degenerate one-edge case.  The local generator unions the
      // fragment's attributed conflict interval and must still pass the same bounded NavMesh,
      // via-span, lane-order, spline, PNS, and complete exact transaction gates.
      const auto local_reserve = result.configured.max_local_layer_bypasses_per_fragment;
      const auto ordinary_template_limit =
          result.configured.max_joint_templates_per_fragment - local_reserve;
      if (existing_template_count < ordinary_template_limit) {
        const auto remaining_templates = ordinary_template_limit - existing_template_count;
        auto generated = build_alternative_layer_templates(
            board, global, *fragment, result, domain_cache, window_check_limit,
            result.configured.max_joint_via_site_ring, remaining_templates + 1U, true);
        domain_cache.release_unretained_if_oversized();
        publish_portfolio_progress();
        for (auto& route_template : generated) {
          const auto duplicate =
              std::any_of(retained.begin(), retained.end(), [&](const auto& prior) {
                return prior.routing_layer == route_template.routing_layer &&
                       prior.topology_centerline == route_template.topology_centerline;
              });
          if (!duplicate) {
            retained.push_back(std::move(route_template));
            expanded = true;
          }
        }
      }
      existing_template_count = retained.size() + 1U;
      if (existing_template_count >= result.configured.max_joint_templates_per_fragment ||
          result.local_layer_bypass_budget_exhausted) {
        continue;
      }
      const auto remaining_bypass_checks = result.configured.max_local_layer_bypass_checks -
                                           std::min(result.configured.max_local_layer_bypass_checks,
                                                    result.local_layer_bypass_checks);
      const auto local_share =
          remaining_candidates == 0U
              ? remaining_bypass_checks
              : (remaining_bypass_checks + remaining_candidates - 1U) / remaining_candidates;
      const auto local_check_limit = result.local_layer_bypass_checks + local_share;
      const auto remaining_local_templates =
          std::min(result.configured.max_local_layer_bypasses_per_fragment,
                   result.configured.max_joint_templates_per_fragment - existing_template_count);
      auto local_generated = build_alternative_layer_templates(
          board, global, *fragment, result, domain_cache, local_check_limit,
          result.configured.max_joint_via_site_ring, remaining_local_templates + 1U, true, true);
      domain_cache.release_unretained_if_oversized();
      publish_portfolio_progress();
      for (auto& route_template : local_generated) {
        const auto duplicate =
            std::any_of(retained.begin(), retained.end(), [&](const auto& prior) {
              return prior.routing_layer == route_template.routing_layer &&
                     prior.topology_centerline == route_template.topology_centerline;
            });
        if (!duplicate) {
          retained.push_back(std::move(route_template));
          expanded = true;
        }
      }
    }
    result.joint_repair_windows_with_expanded_portfolio += expanded ? 1U : 0U;
    if (expanded) {
      expanded_joint_repair_windows.insert(candidate.window->id);
    }
  }
  if (!joint_repair_candidates.empty()) {
    populate_via_region_evidence(board, result);
  }
  result.bundle_assignments.clear();
  for (const auto& window : result.windows) {
    auto assignment = assemble_window_templates(window);
    if (!window.eligible()) {
      assignment.stop_reason = RegionalBundleStopReason::window_ineligible;
      result.bundle_assignments.push_back(std::move(assignment));
      continue;
    }
    auto templates_by_fragment = template_candidates_by_fragment(window, assignment);
    const auto missing_topology =
        std::any_of(templates_by_fragment.begin(), templates_by_fragment.end(),
                    [](const auto& candidates) { return candidates.empty(); });
    assignment.selected_templates.clear();
    assignment.packed_cross_sections.clear();
    bool bundle_budget_exhausted = false;
    bool lane_budget_exhausted = false;
    bool any_fitting_state = false;
    using BundleObjective = std::tuple<std::size_t, std::size_t, std::size_t, std::size_t,
                                       std::uint64_t, std::vector<std::size_t>>;
    std::optional<BundleObjective> best_objective;
    std::vector<std::pair<BundleObjective, RegionalBundleSelection>> joint_spline_pool;
    std::optional<std::pair<BundleObjective, RegionalBundleSelection>> selected_global_fallback;
    if (!missing_topology) {
      const auto portfolio = generate_consistent_bundle_choices(
          board, result, window, assignment.templates, templates_by_fragment,
          result.configured.max_bundle_search_states);
      assignment.bundle_search_states = portfolio.states;
      assignment.partial_access_pruned_states = portfolio.access_pruned_states;
      assignment.partial_access_assignment_states = portfolio.access_assignment_states;
      assignment.partial_access_assignment_exhaustions = portfolio.access_assignment_exhaustions;
      assignment.partial_terminal_pair_pruned_states = portfolio.terminal_pair_pruned_states;
      assignment.partial_via_pair_pruned_states = portfolio.via_pair_pruned_states;
      assignment.partial_terminal_via_pruned_states = portfolio.terminal_via_pruned_states;
      assignment.partial_lane_capacity_pruned_states = portfolio.lane_capacity_pruned_states;
      result.partial_lane_capacity_pruned_states += portfolio.lane_capacity_pruned_states;
      assignment.partial_access_blockers = portfolio.access_blockers;
      bundle_budget_exhausted = portfolio.exhausted;
      for (const auto& choice : portfolio.choices) {
        std::vector<std::size_t> selected;
        std::map<RegionalRouteFragmentId, LayerId> selected_layers;
        std::map<RegionalRouteFragmentId, const RegionalRouteTemplate*>
            selected_templates_by_fragment;
        std::map<std::size_t, std::vector<RegionalRouteFragmentId>> channel_occupants;
        std::size_t via_count = 0U;
        std::uint64_t topology_cost = 0U;
        for (std::size_t fragment = 0U; fragment < choice.size(); ++fragment) {
          const auto template_id = templates_by_fragment[fragment][choice[fragment]];
          selected.push_back(template_id);
          const auto& route_template = assignment.templates[template_id];
          selected_layers.emplace(route_template.fragment, route_template.routing_layer);
          selected_templates_by_fragment.emplace(route_template.fragment, &route_template);
          via_count += route_template.via_choices.size();
          topology_cost = topology_cost > std::numeric_limits<std::uint64_t>::max() -
                                              route_template.topology_cost
                              ? std::numeric_limits<std::uint64_t>::max()
                              : topology_cost + route_template.topology_cost;
          for (const auto channel_id : route_template.channel_sequence) {
            channel_occupants[channel_id].push_back(route_template.fragment);
          }
        }
        const auto coupled_legal =
            coupled_template_selection_legal(board, window, assignment.templates, selected);
        const auto access =
            select_access_geometry(board, result, window, assignment.templates, selected);
        assignment.access_assignment_states += access.states;
        assignment.access_assignment_exhaustions += access.exhausted ? 1U : 0U;
        const auto access_legal = access.legal();
        if (!coupled_legal) {
          ++assignment.coupled_rejected_states;
        }
        if (!access_legal) {
          ++assignment.access_rejected_states;
          switch (access.failure) {
          case RegionalAccessFailureReason::invalid_choice:
            ++assignment.invalid_access_rejected_states;
            break;
          case RegionalAccessFailureReason::terminal_pair:
            ++assignment.terminal_pair_rejected_states;
            break;
          case RegionalAccessFailureReason::via_pair:
            ++assignment.via_pair_rejected_states;
            break;
          case RegionalAccessFailureReason::terminal_via:
            ++assignment.terminal_via_rejected_states;
            break;
          case RegionalAccessFailureReason::none:
            break;
          }
        }
        std::size_t residual_conflicts = 0U;
        for (const auto edge_id : window.conflict_edges) {
          const auto& edge = result.conflict_edges[edge_id];
          const auto first_template = selected_templates_by_fragment.at(edge.first);
          const auto second_template = selected_templates_by_fragment.at(edge.second);
          const auto first_avoids =
              std::binary_search(first_template->avoided_conflict_edges.begin(),
                                 first_template->avoided_conflict_edges.end(), edge_id);
          const auto second_avoids =
              std::binary_search(second_template->avoided_conflict_edges.begin(),
                                 second_template->avoided_conflict_edges.end(), edge_id);
          if (selected_layers.at(edge.first) == selected_layers.at(edge.second) && !first_avoids &&
              !second_avoids) {
            ++residual_conflicts;
          }
        }
        struct TemplateSegment {
          RegionalRouteFragmentId fragment;
          LayerId layer;
          Point first;
          Point second;
          Nanometres width;
          Nanometres clearance;
          bool alternative{false};
        };
        std::vector<TemplateSegment> template_segments;
        const auto selected_via_for =
            [&](const std::optional<std::size_t>& choice_id) -> const RegionalViaChoice* {
          if (!choice_id) {
            return static_cast<const RegionalViaChoice*>(nullptr);
          }
          const auto source = std::find_if(result.via_choices.begin(), result.via_choices.end(),
                                           [&](const auto& via) { return via.id == *choice_id; });
          if (source == result.via_choices.end()) {
            return static_cast<const RegionalViaChoice*>(nullptr);
          }
          for (const auto selected_choice_id : access.via_choices) {
            const auto selected_choice =
                std::find_if(result.via_choices.begin(), result.via_choices.end(),
                             [&](const auto& via) { return via.id == selected_choice_id; });
            if (selected_choice != result.via_choices.end() &&
                selected_choice->net == source->net &&
                selected_choice->region_id == source->region_id &&
                selected_choice->span == source->span) {
              return &*selected_choice;
            }
          }
          return &*source;
        };
        const auto append_segment = [&](const RegionalRouteFragment& fragment, LayerId layer,
                                        Point first, Point second, bool alternative) {
          if (first != second) {
            template_segments.push_back(TemplateSegment{fragment.id, std::move(layer), first,
                                                        second, fragment.width, fragment.clearance,
                                                        alternative});
          }
        };
        std::set<std::pair<RegionalRouteFragmentId, RegionalRouteFragmentId>> template_conflicts;
        if (expanded_joint_repair_windows.contains(window.id)) {
          for (const auto template_id : selected) {
            const auto& route_template = assignment.templates[template_id];
            const auto fragment = std::find_if(
                result.fragments.begin(), result.fragments.end(),
                [&](const auto& value) { return value.id == route_template.fragment; });
            if (fragment == result.fragments.end()) {
              continue;
            }
            for (std::size_t edge = 1U; edge < route_template.topology_centerline.size(); ++edge) {
              append_segment(*fragment, route_template.routing_layer,
                             route_template.topology_centerline[edge - 1U],
                             route_template.topology_centerline[edge],
                             !route_template.selected_global_topology);
            }
            const auto append_runup = [&](const RegionalViaChoice* via, Point endpoint) {
              if (via == nullptr || via->site == endpoint) {
                return;
              }
              const auto runup = std::find_if(
                  via->attached_runups.begin(), via->attached_runups.end(), [&](const auto& value) {
                    return value.layer == fragment->layer && value.centerline.size() >= 2U &&
                           value.anchor == endpoint;
                  });
              if (runup == via->attached_runups.end()) {
                append_segment(*fragment, fragment->layer, endpoint, via->site, true);
                return;
              }
              for (std::size_t edge = 1U; edge < runup->centerline.size(); ++edge) {
                append_segment(*fragment, runup->layer, runup->centerline[edge - 1U],
                               runup->centerline[edge], true);
              }
            };
            append_runup(selected_via_for(route_template.entry_via_choice), fragment->start);
            append_runup(selected_via_for(route_template.exit_via_choice), fragment->end);
          }
          for (std::size_t first = 0U; first < template_segments.size(); ++first) {
            for (std::size_t second = first + 1U; second < template_segments.size(); ++second) {
              const auto& lhs = template_segments[first];
              const auto& rhs = template_segments[second];
              if (lhs.fragment.net == rhs.fragment.net || lhs.layer != rhs.layer ||
                  (!lhs.alternative && !rhs.alternative)) {
                continue;
              }
              const auto required =
                  static_cast<long double>(lhs.width.value() + rhs.width.value()) / 2.0L +
                  static_cast<long double>(std::max(lhs.clearance, rhs.clearance).value());
              if (geometry::segment_distance_nm(lhs.first, lhs.second, rhs.first, rhs.second) <
                  required) {
                template_conflicts.insert(std::minmax(lhs.fragment, rhs.fragment));
              }
            }
          }
        }
        std::vector<RegionalCrossSection> packed_sections;
        bool packing_failed = !coupled_legal || !access_legal;
        for (auto channel_iterator = channel_occupants.begin();
             !packing_failed && channel_iterator != channel_occupants.end(); ++channel_iterator) {
          const auto channel_id = channel_iterator->first;
          auto& occupants = channel_iterator->second;
          const auto channel = std::find_if(
              result.channels.begin(), result.channels.end(),
              [channel_id](const auto& candidate) { return candidate.id == channel_id; });
          if (channel == result.channels.end() || channel->cross_sections.empty()) {
            packing_failed = true;
            break;
          }
          std::sort(occupants.begin(), occupants.end());
          occupants.erase(std::unique(occupants.begin(), occupants.end()), occupants.end());
          const auto base_states = assignment.lane_order_states;
          const auto remaining_states =
              result.configured.max_lane_order_states -
              std::min(result.configured.max_lane_order_states, assignment.lane_order_states);
          auto solved = solve_channel_lanes(
              *channel, occupants, remaining_states, [&](const auto local_states) {
                publish_lane_progress(window.id, base_states + local_states, "bundle-assignment");
              });
          assignment.lane_order_states += solved.states;
          lane_budget_exhausted = lane_budget_exhausted || solved.budget_exhausted;
          if (!solved.packing) {
            packing_failed = true;
            break;
          }
          solved.packing->id = packed_sections.size();
          packed_sections.push_back(std::move(*solved.packing));
        }
        if (!packing_failed) {
          any_fitting_state = true;
          const auto relocated_vias = static_cast<std::size_t>(std::count_if(
              access.via_choices.begin(), access.via_choices.end(), [&](const auto choice_id) {
                const auto found =
                    std::find_if(result.via_choices.begin(), result.via_choices.end(),
                                 [&](const auto& candidate) { return candidate.id == choice_id; });
                return found != result.via_choices.end() && !found->selected_global_site;
              }));
          const auto template_conflict_objective =
              expanded_joint_repair_windows.contains(window.id) ? template_conflicts.size() : 0U;
          const BundleObjective objective{template_conflict_objective,
                                          residual_conflicts,
                                          relocated_vias,
                                          via_count,
                                          topology_cost,
                                          selected};
          RegionalBundleSelection selection{
              selected,          access.via_choices,          access.region_order,
              packed_sections,   access.alternative_switches, template_conflicts.size(),
              residual_conflicts};
          const auto selected_global_state =
              std::all_of(selected.begin(), selected.end(), [&](const auto template_id) {
                return template_id < assignment.templates.size() &&
                       assignment.templates[template_id].selected_global_topology;
              });
          if (selected_global_state &&
              (!selected_global_fallback || objective < selected_global_fallback->first)) {
            selected_global_fallback = std::pair{objective, selection};
          }
          const auto insertion = std::lower_bound(
              joint_spline_pool.begin(), joint_spline_pool.end(), objective,
              [](const auto& candidate, const auto& value) { return candidate.first < value; });
          if (insertion == joint_spline_pool.end() || insertion->first != objective) {
            joint_spline_pool.insert(insertion, {objective, selection});
            const auto pool_limit = result.configured.max_joint_spline_candidates >
                                            std::numeric_limits<std::size_t>::max() / 8U
                                        ? std::numeric_limits<std::size_t>::max()
                                        : result.configured.max_joint_spline_candidates * 8U;
            if (joint_spline_pool.size() > pool_limit) {
              joint_spline_pool.pop_back();
            }
          }
          if (!best_objective || objective < *best_objective) {
            best_objective = objective;
            assignment.selected_templates = selection.templates;
            assignment.selected_via_choices = selection.via_choices;
            assignment.via_assignment_region_order = selection.via_assignment_region_order;
            assignment.selected_via_alternative_switches = selection.via_alternative_switches;
            assignment.packed_cross_sections = selection.packed_cross_sections;
            assignment.selected_template_conflicts = selection.template_conflicts;
            assignment.selected_residual_conflicts = selection.residual_conflicts;
          }
        } else if (coupled_legal && access_legal) {
          ++assignment.packing_rejected_states;
        }
        if (lane_budget_exhausted) {
          break;
        }
      }
    }
    assignment.stop_reason =
        missing_topology ? RegionalBundleStopReason::missing_corridor_topology
        : !any_fitting_state && (bundle_budget_exhausted || lane_budget_exhausted)
            ? RegionalBundleStopReason::state_budget_exhausted
        : !any_fitting_state ? RegionalBundleStopReason::no_fitting_lane_order
                             : RegionalBundleStopReason::complete;
    std::optional<std::size_t> selected_global_fallback_index;
    if (selected_global_fallback) {
      const auto found = std::find_if(
          joint_spline_pool.begin(), joint_spline_pool.end(), [&](const auto& candidate) {
            return candidate.second.templates == selected_global_fallback->second.templates &&
                   candidate.second.via_choices == selected_global_fallback->second.via_choices;
          });
      if (found == joint_spline_pool.end()) {
        joint_spline_pool.push_back(std::move(*selected_global_fallback));
        selected_global_fallback_index = joint_spline_pool.size() - 1U;
      } else {
        selected_global_fallback_index =
            static_cast<std::size_t>(found - joint_spline_pool.begin());
      }
    }
    if (assignment.complete() && joint_spline_pool.size() > 1U) {
      // Scalar-nearest states commonly differ only in one via site and reach the same continuous
      // local minimum. Keep the best state, then maximize the minimum template Hamming distance
      // from the retained set. Both the 8x pool and the output shortlist are fixed bounds.
      std::vector<std::size_t> retained;
      if (!joint_spline_pool.empty()) {
        retained.push_back(0U);
      }
      if (selected_global_fallback_index && *selected_global_fallback_index != 0U &&
          retained.size() < result.configured.max_joint_spline_candidates) {
        retained.push_back(*selected_global_fallback_index);
      }
      while (retained.size() < result.configured.max_joint_spline_candidates &&
             retained.size() < joint_spline_pool.size()) {
        std::optional<std::size_t> best_index;
        std::size_t best_distance = 0U;
        for (std::size_t candidate = 0U; candidate < joint_spline_pool.size(); ++candidate) {
          if (std::find(retained.begin(), retained.end(), candidate) != retained.end()) {
            continue;
          }
          const auto& candidate_templates = joint_spline_pool[candidate].second.templates;
          auto minimum_distance = std::numeric_limits<std::size_t>::max();
          for (const auto selected : retained) {
            const auto& selected_templates = joint_spline_pool[selected].second.templates;
            std::size_t distance =
                candidate_templates.size() != selected_templates.size()
                    ? std::max(candidate_templates.size(), selected_templates.size())
                    : 0U;
            if (candidate_templates.size() == selected_templates.size()) {
              for (std::size_t fragment = 0U; fragment < candidate_templates.size(); ++fragment) {
                distance += candidate_templates[fragment] != selected_templates[fragment] ? 1U : 0U;
              }
            }
            minimum_distance = std::min(minimum_distance, distance);
          }
          if (!best_index || minimum_distance > best_distance ||
              (minimum_distance == best_distance &&
               joint_spline_pool[candidate].first < joint_spline_pool[*best_index].first)) {
            best_index = candidate;
            best_distance = minimum_distance;
          }
        }
        if (!best_index) {
          break;
        }
        retained.push_back(*best_index);
      }
      assignment.spline_candidate_shortlist.reserve(retained.size());
      for (const auto selected : retained) {
        assignment.spline_candidate_shortlist.push_back(
            std::move(joint_spline_pool[selected].second));
      }
    }
    result.bundle_assignments.push_back(std::move(assignment));
  }
  std::vector<std::size_t> spline_portfolio_indices;
  for (std::size_t index = 0U; index < result.bundle_assignments.size(); ++index) {
    const auto& assignment = result.bundle_assignments[index];
    if (assignment.complete() && assignment.spline_candidate_shortlist.size() > 1U &&
        (assignment.selected_residual_conflicts > 0U ||
         expanded_joint_repair_windows.contains(assignment.window_id))) {
      spline_portfolio_indices.push_back(index);
    }
  }
  std::stable_sort(
      spline_portfolio_indices.begin(), spline_portfolio_indices.end(),
      [&](const auto lhs, const auto rhs) {
        const auto& first = result.bundle_assignments[lhs];
        const auto& second = result.bundle_assignments[rhs];
        const auto first_expanded = expanded_joint_repair_windows.contains(first.window_id);
        const auto second_expanded = expanded_joint_repair_windows.contains(second.window_id);
        if (first_expanded != second_expanded) {
          return first_expanded;
        }
        if (first.selected_residual_conflicts != second.selected_residual_conflicts) {
          return first.selected_residual_conflicts > second.selected_residual_conflicts;
        }
        const auto overlap_for = [&](const std::size_t window_id) {
          const auto found =
              std::find_if(result.windows.begin(), result.windows.end(),
                           [&](const auto& candidate) { return candidate.id == window_id; });
          return found == result.windows.end() ? std::uint64_t{0}
                                               : found->observed_overlap_length_nm;
        };
        const auto first_overlap = overlap_for(first.window_id);
        const auto second_overlap = overlap_for(second.window_id);
        return first_overlap != second_overlap ? first_overlap > second_overlap
                                               : first.window_id < second.window_id;
      });
  if (spline_portfolio_indices.size() > result.configured.max_spline_portfolio_windows) {
    spline_portfolio_indices.resize(result.configured.max_spline_portfolio_windows);
  }
  std::set<std::size_t> retained_spline_portfolios;
  for (const auto index : spline_portfolio_indices) {
    retained_spline_portfolios.insert(result.bundle_assignments[index].window_id);
    result.spline_portfolio_window_ids.push_back(result.bundle_assignments[index].window_id);
  }
  result.spline_portfolio_windows = result.spline_portfolio_window_ids.size();
  for (auto& assignment : result.bundle_assignments) {
    if (!retained_spline_portfolios.contains(assignment.window_id)) {
      assignment.spline_candidate_shortlist.clear();
    }
  }
  for (const auto& assignment : result.bundle_assignments) {
    if (assignment.complete() && expanded_joint_repair_windows.contains(assignment.window_id)) {
      result.joint_repair_completed_window_ids.push_back(assignment.window_id);
    }
  }
  result.joint_repair_windows_completed = result.joint_repair_completed_window_ids.size();
  for (const auto& assignment : result.bundle_assignments) {
    if (!assignment.complete()) {
      continue;
    }
    for (const auto& cross_section : assignment.packed_cross_sections) {
      ++result.selected_lane_cross_sections;
      auto stable_order = cross_section.ordered_occupants;
      std::sort(stable_order.begin(), stable_order.end());
      if (stable_order != cross_section.ordered_occupants) {
        ++result.selected_geometric_lane_reorders;
      }
      if (cross_section.stable_order_inversions > cross_section.selected_order_inversions) {
        result.avoided_lane_order_inversions +=
            cross_section.stable_order_inversions - cross_section.selected_order_inversions;
      }
    }
  }
}

struct WorkingRegionalSpline {
  RegionalRouteFragmentId fragment;
  Point source_start;
  Point replacement_end;
  LayerId layer;
  CubicBSplineRoute spline;
  NavMeshTopologySignature topology_signature;
  std::vector<Polygon> topology_cells;
  struct TopologyCellIndex;
  std::shared_ptr<const TopologyCellIndex> topology_cell_index;
  std::vector<RegionalTopologyLink> topology_links;
  std::optional<Point> entry_via_site;
  std::optional<Point> exit_via_site;
  Nanometres width;
  Nanometres clearance;
  GlobalRouteSegmentWidthKind width_kind{GlobalRouteSegmentWidthKind::nominal};
  bool owned{false};
  bool emit_replacement{true};
  bool frozen_boundary{false};
  bool selected_global_topology{false};
  std::size_t merged_fragments{1U};
  SplineVector lane_displacement;
  std::size_t sweeps{0};
  std::size_t control_moves{0};
};

struct WorkingRegionalSpline::TopologyCellIndex {
  std::int64_t pitch_nm{1};
  std::map<std::pair<std::int64_t, std::int64_t>, std::vector<std::size_t>> bins;
};

[[nodiscard]] std::int64_t regional_topology_bin(std::int64_t coordinate, std::int64_t pitch) {
  const auto quotient = coordinate / pitch;
  const auto remainder = coordinate % pitch;
  return remainder < 0 ? quotient - 1 : quotient;
}

[[nodiscard]] std::shared_ptr<const WorkingRegionalSpline::TopologyCellIndex>
prepare_regional_topology_cell_index(const std::vector<Polygon>& cells) {
  auto index = std::make_shared<WorkingRegionalSpline::TopologyCellIndex>();
  index->pitch_nm = Nanometres::from_mm(1.0).value();
  constexpr std::size_t maximum_index_entries = 1'000'000U;
  std::size_t entries = 0U;
  for (std::size_t cell = 0U; cell < cells.size(); ++cell) {
    const auto bounds = regional_polygon_bounds(cells[cell]);
    if (!bounds) {
      continue;
    }
    const auto minimum_x = regional_topology_bin(bounds->minimum.x.value(), index->pitch_nm);
    const auto maximum_x = regional_topology_bin(bounds->maximum.x.value(), index->pitch_nm);
    const auto minimum_y = regional_topology_bin(bounds->minimum.y.value(), index->pitch_nm);
    const auto maximum_y = regional_topology_bin(bounds->maximum.y.value(), index->pitch_nm);
    const auto columns = static_cast<std::uint64_t>(maximum_x - minimum_x) + 1U;
    const auto rows = static_cast<std::uint64_t>(maximum_y - minimum_y) + 1U;
    if (columns > maximum_index_entries || rows > maximum_index_entries ||
        columns * rows > maximum_index_entries - entries) {
      return {};
    }
    entries += static_cast<std::size_t>(columns * rows);
    for (auto x = minimum_x; x <= maximum_x; ++x) {
      for (auto y = minimum_y; y <= maximum_y; ++y) {
        index->bins[{x, y}].push_back(cell);
      }
    }
  }
  return index;
}

[[nodiscard]] Point quantize_regional_spline(SplineVector point) {
  return Point{Nanometres{static_cast<std::int64_t>(std::llround(point.x))},
               Nanometres{static_cast<std::int64_t>(std::llround(point.y))}};
}

[[nodiscard]] bool regional_spline_topology_contains(const WorkingRegionalSpline& route,
                                                     SplineVector position) {
  const auto point = quantize_regional_spline(position);
  const auto in_cell = [&]() {
    if (route.topology_cell_index) {
      const auto key =
          std::pair{regional_topology_bin(point.x.value(), route.topology_cell_index->pitch_nm),
                    regional_topology_bin(point.y.value(), route.topology_cell_index->pitch_nm)};
      const auto found = route.topology_cell_index->bins.find(key);
      return found != route.topology_cell_index->bins.end() &&
             std::any_of(found->second.begin(), found->second.end(), [&](const auto cell) {
               return cell < route.topology_cells.size() &&
                      geometry::locate_point(route.topology_cells[cell], point) !=
                          geometry::PointLocation::outside;
             });
    }
    return std::any_of(
        route.topology_cells.begin(), route.topology_cells.end(), [&](const auto& cell) {
          return geometry::locate_point(cell, point) != geometry::PointLocation::outside;
        });
  };
  return in_cell() ||
         std::any_of(route.topology_links.begin(), route.topology_links.end(),
                     [&](const auto& link) {
                       return geometry::point_segment_distance_nm(point, link.first, link.second) <=
                              static_cast<long double>(link.radius.value());
                     });
}

[[nodiscard]] NavMeshTopologyCheck regional_spline_gate_sequence(
    const WorkingRegionalSpline& route, const std::vector<SplineVector>& samples) {
  // Existing global corridors that predate NavMesh signatures retain their conservative cell
  // containment check.  They are intentionally not upgraded to a guessed obstacle-side claim.
  if (!route.topology_signature.valid) {
    return NavMeshTopologyCheck{true, NavMeshTopologyCheckReason::preserved, 0U, 0U,
                                "no directed NavMesh gate certificate"};
  }
  std::vector<Point> polyline;
  polyline.reserve(samples.size());
  for (const auto sample : samples) {
    const auto point = quantize_regional_spline(sample);
    if (polyline.empty() || polyline.back() != point) {
      polyline.push_back(point);
    }
  }
  return validate_navmesh_topology_signature(route.topology_signature, polyline);
}

[[nodiscard]] bool regional_spline_chord_stays_in_topology(const WorkingRegionalSpline& route,
                                                           Point start, Point end,
                                                           Nanometres sample_spacing) {
  const auto length = geometry::segment_length_nm(start, end);
  if (!(length > 0.0L) || sample_spacing.value() <= 0) {
    return start == end;
  }
  const auto steps = static_cast<std::size_t>(std::max<long double>(
      1.0L, std::ceil(length / static_cast<long double>(sample_spacing.value()))));
  // This is a bounded seed simplifier, not a legality oracle. Long chords that exceed the probe
  // budget remain unsimplified; adaptive spline checks and exact materialization remain the hard
  // gates for every accepted route.
  constexpr std::size_t maximum_samples_per_chord = 4096U;
  if (steps > maximum_samples_per_chord) {
    return false;
  }
  const auto dx = static_cast<long double>(end.x.value() - start.x.value());
  const auto dy = static_cast<long double>(end.y.value() - start.y.value());
  for (std::size_t sample = 0U; sample <= steps; ++sample) {
    const auto parameter = static_cast<long double>(sample) / static_cast<long double>(steps);
    if (!regional_spline_topology_contains(
            route, SplineVector{static_cast<long double>(start.x.value()) + parameter * dx,
                                static_cast<long double>(start.y.value()) + parameter * dy})) {
      return false;
    }
  }
  return true;
}

[[nodiscard]] std::vector<Point>
string_pull_regional_spline_seed(const WorkingRegionalSpline& route, Nanometres sample_spacing) {
  std::vector<Point> source;
  source.reserve(route.spline.corridor_anchors.size());
  for (const auto anchor : route.spline.corridor_anchors) {
    const auto point = quantize_regional_spline(anchor);
    if (source.empty() || source.back() != point) {
      source.push_back(point);
    }
  }
  if (source.size() <= 2U) {
    return source;
  }

  // Advance the visible end of each chord until the first corridor boundary. This is the same
  // string-pulling principle as a funnel centerline, applied to the already selected union of
  // regional topology cells. It preserves the corridor/obstacle-side decision while removing
  // sampled-grid staircases before they become cubic degrees of freedom.
  std::vector<Point> result{source.front()};
  std::size_t anchor = 0U;
  while (anchor + 1U < source.size()) {
    auto farthest = anchor + 1U;
    auto candidate = farthest + 1U;
    while (candidate < source.size() &&
           regional_spline_chord_stays_in_topology(route, source[anchor], source[candidate],
                                                   sample_spacing)) {
      farthest = candidate++;
    }
    if (result.back() != source[farthest]) {
      result.push_back(source[farthest]);
    }
    anchor = farthest;
  }
  return result;
}

[[nodiscard]] std::optional<CubicBSplineRoute>
make_topology_tight_regional_spline(const std::vector<Point>& seed,
                                    std::size_t corner_multiplicity) {
  if (corner_multiplicity <= 1U) {
    return make_cubic_bspline_route(seed);
  }
  std::vector<Point> canonical;
  canonical.reserve(seed.size());
  for (const auto point : seed) {
    if (canonical.empty() || canonical.back() != point) {
      canonical.push_back(point);
    }
  }
  if (canonical.size() < 3U || canonical.front() == canonical.back()) {
    return make_cubic_bspline_route(canonical);
  }
  CubicBSplineRoute spline;
  spline.control_points.reserve(2U + (canonical.size() - 2U) * corner_multiplicity + 2U);
  const auto as_vector = [](Point point) {
    return SplineVector{static_cast<long double>(point.x.value()),
                        static_cast<long double>(point.y.value())};
  };
  const auto first = as_vector(canonical.front());
  const auto second = as_vector(canonical[1U]);
  spline.control_points.push_back(first);
  spline.control_points.push_back(
      SplineVector{std::midpoint(first.x, second.x), std::midpoint(first.y, second.y)});
  for (std::size_t point = 1U; point + 1U < canonical.size(); ++point) {
    const auto control = as_vector(canonical[point]);
    spline.control_points.insert(spline.control_points.end(), corner_multiplicity, control);
  }
  const auto previous = as_vector(canonical[canonical.size() - 2U]);
  const auto last = as_vector(canonical.back());
  spline.control_points.push_back(
      SplineVector{std::midpoint(last.x, previous.x), std::midpoint(last.y, previous.y)});
  spline.control_points.push_back(last);
  spline.corridor_anchors = spline.control_points;
  return spline;
}

[[nodiscard]] bool regional_spline_stays_in_topology(const WorkingRegionalSpline& route,
                                                     const CubicBSplineRoute& spline) {
  const auto samples =
      adaptive_tessellate_cubic_bspline(spline, Nanometres::from_mm(0.02), 10U, 512U);
  if (samples.size() < 2U) {
    return false;
  }
  std::vector<SplineVector> positions;
  positions.reserve(samples.size());
  for (const auto& sample : samples) {
    positions.push_back(sample.position);
    if (!regional_spline_topology_contains(route, sample.position)) {
      return false;
    }
  }
  return regional_spline_gate_sequence(route, positions).preserved;
}

[[nodiscard]] SplineClearanceProbe
regional_spline_clearance(const WorkingRegionalSpline& route, SplineVector position,
                          const std::vector<WorkingRegionalSpline>& routes, std::size_t route_index,
                          const std::vector<std::vector<SplineVector>>& route_samples) {
  const auto topology_legal = regional_spline_topology_contains(route, position);
  long double best_margin = std::numeric_limits<long double>::infinity();
  SplineVector best_normal;
  for (std::size_t other = 0U; other < routes.size(); ++other) {
    if (other == route_index || routes[other].layer != route.layer ||
        routes[other].fragment.net == route.fragment.net) {
      continue;
    }
    const auto& samples = route_samples[other];
    const auto required =
        static_cast<long double>(route.width.value() + routes[other].width.value()) / 2.0L +
        static_cast<long double>(std::max(route.clearance, routes[other].clearance).value());
    for (std::size_t sample = 1U; sample < samples.size(); ++sample) {
      const auto first = samples[sample - 1U];
      const auto second = samples[sample];
      const auto dx = second.x - first.x;
      const auto dy = second.y - first.y;
      const auto length_squared = dx * dx + dy * dy;
      const auto parameter =
          length_squared > 0.0L
              ? std::clamp(((position.x - first.x) * dx + (position.y - first.y) * dy) /
                               length_squared,
                           0.0L, 1.0L)
              : 0.0L;
      const auto closest = SplineVector{first.x + parameter * dx, first.y + parameter * dy};
      const auto away = SplineVector{position.x - closest.x, position.y - closest.y};
      const auto distance = std::hypotl(away.x, away.y);
      const auto margin = distance - required;
      if (margin < best_margin) {
        best_margin = margin;
        best_normal =
            distance > 0.0L ? SplineVector{away.x / distance, away.y / distance} : SplineVector{};
      }
    }
  }
  if (!std::isfinite(best_margin)) {
    best_margin = 1.0e12L;
  }
  return SplineClearanceProbe{best_margin, best_normal, topology_legal && best_margin >= 0.0L};
}

struct RegionalSplineLegalityAssessment {
  std::size_t topology_violations{0};
  std::size_t topology_gate_violations{0};
  std::size_t mutual_violations{0};
  long double total_clearance_deficit_nm{0.0L};
  long double worst_clearance_nm{std::numeric_limits<long double>::infinity()};
  long double worst_parameter{0.5L};
  SplineVector worst_normal;
};

[[nodiscard]] RegionalSplineLegalityAssessment assess_regional_spline_legality(
    const std::vector<WorkingRegionalSpline>& routes, std::size_t route_index,
    const std::vector<std::vector<SplineVector>>& route_samples, std::size_t sample_count) {
  RegionalSplineLegalityAssessment result;
  if (route_index >= routes.size() || sample_count < 3U) {
    result.topology_violations = 1U;
    return result;
  }
  const auto gate_sequence =
      route_index < route_samples.size()
          ? regional_spline_gate_sequence(routes[route_index], route_samples[route_index])
          : NavMeshTopologyCheck{};
  if (!gate_sequence.preserved) {
    ++result.topology_violations;
    ++result.topology_gate_violations;
  }
  for (std::size_t sample = 0U; sample < sample_count; ++sample) {
    const auto parameter =
        static_cast<long double>(sample) / static_cast<long double>(sample_count - 1U);
    const auto position = evaluate_cubic_bspline(routes[route_index].spline, parameter);
    if (!regional_spline_topology_contains(routes[route_index], position)) {
      ++result.topology_violations;
    }
    const auto probe = regional_spline_clearance(routes[route_index], position, routes, route_index,
                                                 route_samples);
    if (probe.clearance_nm < 0.0L) {
      ++result.mutual_violations;
      result.total_clearance_deficit_nm -= probe.clearance_nm;
      // Endpoints and their tangent controls are fixed. Attribute them, but select a movable
      // interior collision for the local basis-weighted shove.
      if (sample > 0U && sample + 1U < sample_count &&
          probe.clearance_nm < result.worst_clearance_nm) {
        result.worst_clearance_nm = probe.clearance_nm;
        result.worst_parameter = parameter;
        result.worst_normal = probe.obstacle_normal;
      }
    }
  }
  return result;
}

[[nodiscard]] bool
better_regional_spline_legality(const RegionalSplineLegalityAssessment& candidate,
                                const RegionalSplineLegalityAssessment& current) {
  if (candidate.topology_violations != current.topology_violations) {
    return candidate.topology_violations < current.topology_violations;
  }
  if (candidate.mutual_violations != current.mutual_violations) {
    return candidate.mutual_violations < current.mutual_violations;
  }
  return candidate.total_clearance_deficit_nm + 0.5L < current.total_clearance_deficit_nm;
}

struct AdaptiveRegionalWindowAssessment {
  std::size_t topology_illegal_routes{0};
  std::size_t mutual_illegal_routes{0};
  std::size_t owned_topology_illegal_routes{0};
  std::size_t owned_mutual_illegal_routes{0};
  std::size_t topology_violations{0};
  std::size_t topology_gate_violations{0};
  std::size_t mutual_violations{0};
  long double total_clearance_deficit_nm{0.0L};
  long double worst_clearance_nm{std::numeric_limits<long double>::infinity()};
  std::size_t worst_route{0};
  long double worst_parameter{0.5L};
  SplineVector worst_normal;

  [[nodiscard]] auto objective() const {
    return std::tuple{topology_illegal_routes, mutual_illegal_routes, topology_violations,
                      topology_gate_violations, mutual_violations, total_clearance_deficit_nm};
  }
};

[[nodiscard]] AdaptiveRegionalWindowAssessment
assess_adaptive_regional_window(const std::vector<WorkingRegionalSpline>& routes) {
  AdaptiveRegionalWindowAssessment result;
  std::vector<std::vector<SplineSample>> sampled(routes.size());
  std::vector<std::vector<SplineVector>> positions(routes.size());
  for (std::size_t route = 0U; route < routes.size(); ++route) {
    sampled[route] = adaptive_tessellate_cubic_bspline(routes[route].spline,
                                                       Nanometres::from_mm(0.02), 10U, 512U);
    positions[route].reserve(sampled[route].size());
    for (const auto& sample : sampled[route]) {
      positions[route].push_back(sample.position);
    }
  }
  for (std::size_t route = 0U; route < routes.size(); ++route) {
    bool topology_illegal = false;
    bool mutual_illegal = false;
    if (const auto gate_sequence = regional_spline_gate_sequence(routes[route], positions[route]);
        !gate_sequence.preserved) {
      topology_illegal = true;
      ++result.topology_violations;
      ++result.topology_gate_violations;
    }
    for (std::size_t sample = 0U; sample < sampled[route].size(); ++sample) {
      const auto& point = sampled[route][sample];
      if (!regional_spline_topology_contains(routes[route], point.position)) {
        topology_illegal = true;
        ++result.topology_violations;
      }
      const auto probe =
          regional_spline_clearance(routes[route], point.position, routes, route, positions);
      if (probe.clearance_nm >= 0.0L) {
        continue;
      }
      mutual_illegal = true;
      ++result.mutual_violations;
      result.total_clearance_deficit_nm -= probe.clearance_nm;
      if (!routes[route].frozen_boundary && sample > 0U && sample + 1U < sampled[route].size() &&
          probe.clearance_nm < result.worst_clearance_nm) {
        result.worst_clearance_nm = probe.clearance_nm;
        result.worst_route = route;
        result.worst_parameter = point.parameter;
        result.worst_normal = probe.obstacle_normal;
      }
    }
    result.topology_illegal_routes += topology_illegal ? 1U : 0U;
    result.mutual_illegal_routes += mutual_illegal ? 1U : 0U;
    if (routes[route].owned) {
      result.owned_topology_illegal_routes += topology_illegal ? 1U : 0U;
      result.owned_mutual_illegal_routes += mutual_illegal ? 1U : 0U;
    }
  }
  return result;
}

struct QuantizedRegionalCollision {
  std::size_t first_route{0U};
  std::size_t second_route{0U};
  std::size_t first_segment{0U};
  std::size_t second_segment{0U};
  long double deficit_nm{0.0L};
  SplineVector first_away_normal;
};

struct QuantizedRegionalAssessment {
  std::vector<bool> route_legal;
  std::size_t exact_checks{0U};
  std::size_t conflicts{0U};
  bool budget_exhausted{false};
  std::optional<QuantizedRegionalCollision> worst_collision;
};

[[nodiscard]] QuantizedRegionalAssessment
assess_quantized_regional_window(const std::vector<WorkingRegionalSpline>& routes,
                                 const std::vector<std::vector<Point>>& centerlines,
                                 std::size_t maximum_exact_checks) {
  QuantizedRegionalAssessment result;
  result.route_legal.assign(routes.size(), true);
  for (std::size_t first = 0U; first < routes.size(); ++first) {
    if (first >= centerlines.size() || centerlines[first].size() < 2U) {
      result.route_legal[first] = false;
      continue;
    }
    for (std::size_t second = first + 1U; second < routes.size(); ++second) {
      if (second >= centerlines.size() || centerlines[second].size() < 2U) {
        result.route_legal[second] = false;
        continue;
      }
      if (routes[first].layer != routes[second].layer ||
          routes[first].fragment.net == routes[second].fragment.net) {
        continue;
      }
      const auto required =
          static_cast<long double>(routes[first].width.value() + routes[second].width.value()) /
              2.0L +
          static_cast<long double>(
              std::max(routes[first].clearance, routes[second].clearance).value());
      for (std::size_t first_segment = 1U; first_segment < centerlines[first].size();
           ++first_segment) {
        const auto first_start = centerlines[first][first_segment - 1U];
        const auto first_end = centerlines[first][first_segment];
        const auto first_minimum_x =
            static_cast<long double>(std::min(first_start.x, first_end.x).value()) - required;
        const auto first_maximum_x =
            static_cast<long double>(std::max(first_start.x, first_end.x).value()) + required;
        const auto first_minimum_y =
            static_cast<long double>(std::min(first_start.y, first_end.y).value()) - required;
        const auto first_maximum_y =
            static_cast<long double>(std::max(first_start.y, first_end.y).value()) + required;
        for (std::size_t second_segment = 1U; second_segment < centerlines[second].size();
             ++second_segment) {
          const auto second_start = centerlines[second][second_segment - 1U];
          const auto second_end = centerlines[second][second_segment];
          const auto second_minimum_x =
              static_cast<long double>(std::min(second_start.x, second_end.x).value());
          const auto second_maximum_x =
              static_cast<long double>(std::max(second_start.x, second_end.x).value());
          const auto second_minimum_y =
              static_cast<long double>(std::min(second_start.y, second_end.y).value());
          const auto second_maximum_y =
              static_cast<long double>(std::max(second_start.y, second_end.y).value());
          if (first_maximum_x < second_minimum_x || second_maximum_x < first_minimum_x ||
              first_maximum_y < second_minimum_y || second_maximum_y < first_minimum_y) {
            continue;
          }
          if (result.exact_checks >= maximum_exact_checks) {
            result.budget_exhausted = true;
            return result;
          }
          ++result.exact_checks;
          const auto distance =
              geometry::segment_distance_nm(first_start, first_end, second_start, second_end);
          if (distance >= required) {
            continue;
          }
          ++result.conflicts;
          result.route_legal[first] = false;
          result.route_legal[second] = false;
          const auto deficit = required - distance;
          if (!result.worst_collision || deficit > result.worst_collision->deficit_nm) {
            const auto first_middle =
                SplineVector{(static_cast<long double>(first_start.x.value()) +
                              static_cast<long double>(first_end.x.value())) /
                                 2.0L,
                             (static_cast<long double>(first_start.y.value()) +
                              static_cast<long double>(first_end.y.value())) /
                                 2.0L};
            const auto second_middle =
                SplineVector{(static_cast<long double>(second_start.x.value()) +
                              static_cast<long double>(second_end.x.value())) /
                                 2.0L,
                             (static_cast<long double>(second_start.y.value()) +
                              static_cast<long double>(second_end.y.value())) /
                                 2.0L};
            auto normal =
                SplineVector{first_middle.x - second_middle.x, first_middle.y - second_middle.y};
            const auto length = std::hypotl(normal.x, normal.y);
            if (length > 0.0L) {
              normal.x /= length;
              normal.y /= length;
            }
            result.worst_collision = QuantizedRegionalCollision{
                first, second, first_segment - 1U, second_segment - 1U, deficit, normal};
          }
        }
      }
    }
  }
  return result;
}

void realize_bundle_splines(RegionalRouteResult& result, std::stop_token stop_token,
                            RouterTelemetrySink* telemetry) {
  // Complete windows may publish bounded diagnostic realizations even while another window is
  // unresolved. build_regional_spline_candidate() still requires every assignment to complete,
  // so partial geometry can improve observability without becoming committable copper.
  std::vector<RegionalBundleAssignment> candidate_assignments;
  candidate_assignments.reserve(result.bundle_assignments.size() +
                                result.configured.max_joint_spline_candidates);
  for (auto& assignment : result.bundle_assignments) {
    if (!assignment.complete() || assignment.spline_candidate_shortlist.empty()) {
      candidate_assignments.push_back(std::move(assignment));
      continue;
    }
    auto shortlist = std::move(assignment.spline_candidate_shortlist);
    assignment.spline_candidate_shortlist.clear();
    for (std::size_t rank = 0U; rank < shortlist.size(); ++rank) {
      auto candidate = assignment;
      candidate.selected_templates = shortlist[rank].templates;
      candidate.selected_via_choices = shortlist[rank].via_choices;
      candidate.via_assignment_region_order = shortlist[rank].via_assignment_region_order;
      candidate.packed_cross_sections = shortlist[rank].packed_cross_sections;
      candidate.selected_via_alternative_switches = shortlist[rank].via_alternative_switches;
      candidate.selected_template_conflicts = shortlist[rank].template_conflicts;
      candidate.selected_residual_conflicts = shortlist[rank].residual_conflicts;
      candidate.spline_realizations.clear();
      candidate.spline_candidate_rank = rank;
      candidate.joint_spline_candidate = true;
      candidate_assignments.push_back(std::move(candidate));
    }
  }
  result.bundle_assignments = std::move(candidate_assignments);
  std::size_t assignment_index = 0U;
  for (auto& assignment : result.bundle_assignments) {
    ++assignment_index;
    if (!assignment.complete() || stop_token.stop_requested()) {
      continue;
    }
    if (telemetry != nullptr) {
      RouterTelemetryEvent progress;
      progress.kind = RouterTelemetryEventKind::search_summary;
      progress.iteration = assignment_index;
      progress.total_groups = result.bundle_assignments.size();
      progress.group_id = "regional-window:" + std::to_string(assignment.window_id);
      progress.stable_id = "regional-spline-progress:" + std::to_string(assignment.window_id);
      progress.detail =
          "regional spline realization started: window=" + std::to_string(assignment.window_id) +
          ", assignment=" + std::to_string(assignment_index) + "/" +
          std::to_string(result.bundle_assignments.size());
      static_cast<void>(telemetry->try_emit(std::move(progress)));
    }
    struct LaneTarget {
      long double parameter{0.0L};
      Point position;
      SplineVector displacement;
    };
    // A fragment may cross several physical channels. Preserve every selected cross-section as
    // an ordered world-space lane target; retaining only one scalar offset discards both the
    // corridor direction and its longitudinal variation before spline realization.
    std::map<RegionalRouteFragmentId, std::vector<LaneTarget>> lane_targets;
    for (const auto& cross_section : assignment.packed_cross_sections) {
      const auto cross_dx = static_cast<long double>(cross_section.second_wall.x.value() -
                                                     cross_section.first_wall.x.value());
      const auto cross_dy = static_cast<long double>(cross_section.second_wall.y.value() -
                                                     cross_section.first_wall.y.value());
      const auto cross_length = std::hypotl(cross_dx, cross_dy);
      if (!(cross_length > 0.0L)) {
        assignment.stop_reason = RegionalBundleStopReason::spline_realization_failed;
        break;
      }
      for (const auto& placement : cross_section.placements) {
        const auto offset = static_cast<long double>(placement.center_offset_nm);
        const auto lane_position =
            SplineVector{(static_cast<long double>(cross_section.first_wall.x.value()) +
                          static_cast<long double>(cross_section.second_wall.x.value())) /
                                 2.0L +
                             cross_dx * offset / cross_length,
                         (static_cast<long double>(cross_section.first_wall.y.value()) +
                          static_cast<long double>(cross_section.second_wall.y.value())) /
                                 2.0L +
                             cross_dy * offset / cross_length};
        const auto fragment =
            std::find_if(result.fragments.begin(), result.fragments.end(),
                         [&](const auto& candidate) { return candidate.id == placement.fragment; });
        if (fragment == result.fragments.end()) {
          assignment.stop_reason = RegionalBundleStopReason::spline_realization_failed;
          break;
        }
        if (!geometry::segments_intersect(fragment->start, fragment->end, cross_section.first_wall,
                                          cross_section.second_wall)) {
          continue;
        }
        const auto fragment_dx =
            static_cast<long double>(fragment->end.x.value() - fragment->start.x.value());
        const auto fragment_dy =
            static_cast<long double>(fragment->end.y.value() - fragment->start.y.value());
        const auto fragment_length_squared = fragment_dx * fragment_dx + fragment_dy * fragment_dy;
        if (!(fragment_length_squared > 0.0L)) {
          assignment.stop_reason = RegionalBundleStopReason::spline_realization_failed;
          break;
        }
        const auto parameter = std::clamp(
            ((lane_position.x - static_cast<long double>(fragment->start.x.value())) * fragment_dx +
             (lane_position.y - static_cast<long double>(fragment->start.y.value())) *
                 fragment_dy) /
                fragment_length_squared,
            0.0L, 1.0L);
        const auto source = SplineVector{
            static_cast<long double>(fragment->start.x.value()) + parameter * fragment_dx,
            static_cast<long double>(fragment->start.y.value()) + parameter * fragment_dy};
        lane_targets[placement.fragment].push_back(
            LaneTarget{parameter, quantize_regional_spline(lane_position),
                       SplineVector{lane_position.x - source.x, lane_position.y - source.y}});
      }
      if (assignment.stop_reason == RegionalBundleStopReason::spline_realization_failed) {
        break;
      }
    }
    if (assignment.stop_reason == RegionalBundleStopReason::spline_realization_failed) {
      continue;
    }
    for (const auto& [fragment, targets] : lane_targets) {
      static_cast<void>(fragment);
      result.spline_lane_targets += targets.size();
      result.spline_multi_target_fragments += targets.size() > 1U ? 1U : 0U;
    }
    const auto window =
        std::find_if(result.windows.begin(), result.windows.end(),
                     [&](const auto& candidate) { return candidate.id == assignment.window_id; });
    if (window == result.windows.end()) {
      assignment.stop_reason = RegionalBundleStopReason::spline_realization_failed;
      continue;
    }
    for (const auto& fragment : window->fragments) {
      if (!std::binary_search(window->owned_fragments.begin(), window->owned_fragments.end(),
                              fragment)) {
        lane_targets.erase(fragment);
      }
    }
    std::vector<WorkingRegionalSpline> routes;
    for (const auto template_id : assignment.selected_templates) {
      if (template_id >= assignment.templates.size()) {
        assignment.stop_reason = RegionalBundleStopReason::spline_realization_failed;
        break;
      }
      const auto& route_template = assignment.templates[template_id];
      const auto fragment = std::find_if(
          result.fragments.begin(), result.fragments.end(),
          [&](const auto& candidate) { return candidate.id == route_template.fragment; });
      if (fragment == result.fragments.end() || route_template.channel_sequence.empty()) {
        assignment.stop_reason = RegionalBundleStopReason::spline_realization_failed;
        break;
      }
      std::vector<Polygon> cells;
      std::vector<RegionalTopologyLink> topology_links;
      auto topology_signature = route_template.topology_signature;
      std::vector<NavMeshTopologyGate> channel_gates;
      for (const auto channel_id : route_template.channel_sequence) {
        const auto channel = std::find_if(
            result.channels.begin(), result.channels.end(),
            [channel_id](const auto& candidate) { return candidate.id == channel_id; });
        if (channel != result.channels.end()) {
          cells.insert(cells.end(), channel->topology_cells.begin(), channel->topology_cells.end());
          topology_links.insert(topology_links.end(), channel->endpoint_links.begin(),
                                channel->endpoint_links.end());
          if (!topology_signature.valid && channel->topology_signature.valid) {
            channel_gates.insert(channel_gates.end(), channel->topology_signature.gates.begin(),
                                 channel->topology_signature.gates.end());
          }
        }
      }
      if (!topology_signature.valid && !channel_gates.empty()) {
        topology_signature = make_navmesh_topology_signature(std::move(channel_gates));
      }
      if (cells.empty()) {
        assignment.stop_reason = RegionalBundleStopReason::spline_realization_failed;
        break;
      }
      const auto selected_via =
          [&](const std::optional<std::size_t>& source_choice) -> const RegionalViaChoice* {
        if (!source_choice) {
          return nullptr;
        }
        const auto source =
            std::find_if(result.via_choices.begin(), result.via_choices.end(),
                         [&](const auto& choice) { return choice.id == *source_choice; });
        if (source == result.via_choices.end()) {
          return nullptr;
        }
        for (const auto selected_choice : assignment.selected_via_choices) {
          const auto candidate =
              std::find_if(result.via_choices.begin(), result.via_choices.end(),
                           [&](const auto& choice) { return choice.id == selected_choice; });
          if (candidate != result.via_choices.end() && candidate->net == source->net &&
              candidate->region_id == source->region_id && candidate->span == source->span) {
            return &*candidate;
          }
        }
        return &*source;
      };
      const auto* entry_via = selected_via(route_template.entry_via_choice);
      const auto* exit_via = selected_via(route_template.exit_via_choice);
      const auto selected_terminal =
          [&](const std::optional<std::size_t>& choice_id) -> const RegionalTerminalChoice* {
        if (!choice_id) {
          return nullptr;
        }
        const auto found =
            std::find_if(result.terminal_choices.begin(), result.terminal_choices.end(),
                         [&](const auto& choice) { return choice.id == *choice_id; });
        return found == result.terminal_choices.end() ? nullptr : &*found;
      };
      const auto* entry_terminal = selected_terminal(route_template.terminal_entry_choice);
      const auto* exit_terminal = selected_terminal(route_template.terminal_exit_choice);
      const auto entry_via_site =
          entry_via == nullptr ? std::optional<Point>{} : std::optional<Point>{entry_via->site};
      const auto exit_via_site =
          exit_via == nullptr ? std::optional<Point>{} : std::optional<Point>{exit_via->site};
      const auto route_start = entry_via_site && (!route_template.selected_global_topology ||
                                                  !fragment->begins_at_terminal)
                                   ? *entry_via_site
                               : entry_terminal != nullptr ? entry_terminal->portal
                                                           : fragment->start;
      const auto route_end =
          exit_via_site && (!route_template.selected_global_topology || !fragment->ends_at_terminal)
              ? *exit_via_site
          : exit_terminal != nullptr ? exit_terminal->portal
                                     : fragment->end;
      const auto selected_runup = [&](const RegionalViaChoice* via, bool entry) {
        std::vector<Point> runup;
        if (via == nullptr || via->selected_global_site) {
          return runup;
        }
        const auto found = std::find_if(
            via->attached_runups.begin(), via->attached_runups.end(), [&](const auto& candidate) {
              return candidate.layer == fragment->layer && candidate.centerline.size() >= 2U &&
                     geometry::point_segment_distance_nm(candidate.anchor, fragment->start,
                                                         fragment->end) == 0.0L;
            });
        if (found == via->attached_runups.end()) {
          return runup;
        }
        runup = found->centerline;
        if (entry) {
          std::reverse(runup.begin(), runup.end());
        }
        return runup;
      };
      const auto entry_runup = selected_runup(entry_via, true);
      const auto exit_runup = selected_runup(exit_via, false);
      const auto dx = static_cast<long double>(route_end.x.value() - route_start.x.value());
      const auto dy = static_cast<long double>(route_end.y.value() - route_start.y.value());
      const auto length = std::hypotl(dx, dy);
      if (!(length > 0.0L)) {
        assignment.stop_reason = RegionalBundleStopReason::spline_realization_failed;
        break;
      }
      const auto resource_width_fixed =
          fragment->explicit_width &&
          fragment->width_kind != GlobalRouteSegmentWidthKind::terminal_neck_down;
      const auto owned = std::binary_search(window->owned_fragments.begin(),
                                            window->owned_fragments.end(), fragment->id);
      auto targets = resource_width_fixed ? std::vector<LaneTarget>{} : lane_targets[fragment->id];
      std::sort(targets.begin(), targets.end(), [](const auto& lhs, const auto& rhs) {
        return lhs.parameter != rhs.parameter ? lhs.parameter < rhs.parameter
                                              : lhs.position < rhs.position;
      });
      targets.erase(std::unique(targets.begin(), targets.end(),
                                [](const auto& lhs, const auto& rhs) {
                                  return lhs.position == rhs.position;
                                }),
                    targets.end());
      SplineVector target_displacement;
      for (const auto& target : targets) {
        target_displacement.x += target.displacement.x;
        target_displacement.y += target.displacement.y;
      }
      if (!targets.empty()) {
        target_displacement.x /= static_cast<long double>(targets.size());
        target_displacement.y /= static_cast<long double>(targets.size());
      }
      const auto interpolate = [&](long double parameter, bool shifted) {
        return Point{Nanometres{static_cast<std::int64_t>(
                         std::llround(static_cast<long double>(route_start.x.value()) +
                                      parameter * dx + (shifted ? target_displacement.x : 0.0L)))},
                     Nanometres{static_cast<std::int64_t>(
                         std::llround(static_cast<long double>(route_start.y.value()) +
                                      parameter * dy + (shifted ? target_displacement.y : 0.0L)))}};
      };
      const auto topology_locked_seed = [&](const std::vector<Point>& centerline) {
        std::vector<Point> prepared;
        if (centerline.size() < 2U) {
          return prepared;
        }
        prepared.reserve(centerline.size());
        for (const auto point : centerline) {
          if (prepared.empty() || prepared.back() != point) {
            prepared.push_back(point);
          }
        }
        return prepared;
      };
      std::vector<Point> seed;
      if (!route_template.selected_global_topology &&
          route_template.topology_centerline.size() >= 2U) {
        // Keep the corridor centerline's direction changes, but do not turn repeated samples into
        // independent controls. The later corridor string pull removes any remaining staircase.
        seed = topology_locked_seed(route_template.topology_centerline);
        if (seed.size() == 2U) {
          seed.insert(
              seed.begin() + 1,
              Point{Nanometres{std::midpoint(seed.front().x.value(), seed.back().x.value())},
                    Nanometres{std::midpoint(seed.front().y.value(), seed.back().y.value())}});
        }
      } else {
        std::vector<Point> lane_centerline{route_start};
        const auto append_centerline = [&](Point point) {
          if (lane_centerline.back() != point) {
            lane_centerline.push_back(point);
          }
        };
        for (const auto point : entry_runup) {
          append_centerline(point);
        }
        const auto source_parameter = [&](Point point) {
          const auto source_dx =
              static_cast<long double>(fragment->end.x.value() - fragment->start.x.value());
          const auto source_dy =
              static_cast<long double>(fragment->end.y.value() - fragment->start.y.value());
          const auto denominator = source_dx * source_dx + source_dy * source_dy;
          return denominator > 0.0L
                     ? std::clamp(
                           (static_cast<long double>(point.x.value() - fragment->start.x.value()) *
                                source_dx +
                            static_cast<long double>(point.y.value() - fragment->start.y.value()) *
                                source_dy) /
                               denominator,
                           0.0L, 1.0L)
                     : 0.0L;
        };
        const auto minimum_parameter =
            entry_runup.empty() ? 0.0L : source_parameter(entry_runup.back());
        const auto maximum_parameter =
            exit_runup.empty() ? 1.0L : source_parameter(exit_runup.front());
        for (const auto& target : targets) {
          if (target.parameter > minimum_parameter && target.parameter < maximum_parameter &&
              target.position != lane_centerline.back() && target.position != route_end) {
            lane_centerline.push_back(target.position);
          }
        }
        for (const auto point : exit_runup) {
          append_centerline(point);
        }
        append_centerline(route_end);
        if (lane_centerline.size() > 2U) {
          seed = topology_locked_seed(lane_centerline);
        } else {
          seed = {route_start, interpolate(1.0L / 3.0L, true), interpolate(2.0L / 3.0L, true),
                  route_end};
        }
      }
      auto spline = make_cubic_bspline_route(seed);
      if (!spline) {
        assignment.stop_reason = RegionalBundleStopReason::spline_realization_failed;
        break;
      }
      if (route_template.selected_global_topology) {
        // The selected global fragment is exact-static-checked topology, but merged corridor cells
        // need not contain its clipped endpoints. Preserve a narrow route-specific run-up guard
        // instead of allowing fixed endpoints to fail a cell-only containment test.
        const auto radius =
            Nanometres{std::max<std::int64_t>(1, result.configured.navmesh_cell_pitch.value() / 2)};
        for (std::size_t point = 1U; point < seed.size(); ++point) {
          if (seed[point - 1U] != seed[point]) {
            topology_links.push_back(RegionalTopologyLink{seed[point - 1U], seed[point], radius});
          }
        }
      }
      routes.push_back(WorkingRegionalSpline{fragment->id,
                                             fragment->start,
                                             fragment->end,
                                             route_template.routing_layer,
                                             std::move(*spline),
                                             std::move(topology_signature),
                                             std::move(cells),
                                             {},
                                             std::move(topology_links),
                                             entry_via_site,
                                             exit_via_site,
                                             fragment->width,
                                             fragment->clearance,
                                             fragment->width_kind,
                                             owned,
                                             !resource_width_fixed,
                                             !owned || resource_width_fixed,
                                             route_template.selected_global_topology,
                                             1U,
                                             target_displacement});
    }
    if (assignment.stop_reason == RegionalBundleStopReason::spline_realization_failed) {
      continue;
    }
    // Fragment-local splines fix every sampled corner. Merge contiguous owned fragments from one
    // unchanged global run so only the outer endpoints remain fixed and internal coarse vertices
    // become local cubic controls inside the union topology corridor.
    std::vector<WorkingRegionalSpline> merged_routes;
    std::vector<bool> consumed(routes.size(), false);
    for (std::size_t first = 0U; first < routes.size(); ++first) {
      if (consumed[first]) {
        continue;
      }
      std::vector<std::size_t> chain{first};
      if (routes[first].owned && routes[first].emit_replacement &&
          routes[first].selected_global_topology && !routes[first].entry_via_site &&
          !routes[first].exit_via_site) {
        auto tail = first;
        while (true) {
          std::optional<std::size_t> next;
          for (std::size_t candidate = 0U; candidate < routes.size(); ++candidate) {
            if (consumed[candidate] ||
                std::find(chain.begin(), chain.end(), candidate) != chain.end()) {
              continue;
            }
            const auto& lhs = routes[tail];
            const auto& rhs = routes[candidate];
            if (!rhs.owned || !rhs.emit_replacement || !rhs.selected_global_topology ||
                rhs.entry_via_site || rhs.exit_via_site || lhs.fragment.net != rhs.fragment.net ||
                lhs.fragment.connection_index != rhs.fragment.connection_index ||
                lhs.fragment.run_index != rhs.fragment.run_index || lhs.layer != rhs.layer ||
                lhs.width != rhs.width || lhs.clearance != rhs.clearance ||
                lhs.width_kind != rhs.width_kind || lhs.replacement_end != rhs.source_start ||
                lhs.fragment.fragment_index + 1U != rhs.fragment.fragment_index) {
              continue;
            }
            if (!next || rhs.fragment < routes[*next].fragment) {
              next = candidate;
            }
          }
          if (!next) {
            break;
          }
          chain.push_back(*next);
          tail = *next;
        }
      }
      auto merged = std::move(routes[first]);
      consumed[first] = true;
      if (chain.size() > 1U) {
        const auto unmerged = merged;
        std::vector<Point> seed{merged.source_start};
        for (std::size_t member = 0U; member < chain.size(); ++member) {
          const auto index = chain[member];
          if (index != first) {
            merged.topology_cells.insert(merged.topology_cells.end(),
                                         routes[index].topology_cells.begin(),
                                         routes[index].topology_cells.end());
            merged.topology_links.insert(merged.topology_links.end(),
                                         routes[index].topology_links.begin(),
                                         routes[index].topology_links.end());
            merged.lane_displacement.x += routes[index].lane_displacement.x;
            merged.lane_displacement.y += routes[index].lane_displacement.y;
          }
          const auto endpoint = routes[index].replacement_end;
          if (seed.back() != endpoint) {
            seed.push_back(endpoint);
          }
        }
        if (auto spline = make_cubic_bspline_route(seed)) {
          merged.spline = std::move(*spline);
          merged.replacement_end = routes[chain.back()].replacement_end;
          merged.merged_fragments = chain.size();
          merged.lane_displacement.x /= static_cast<long double>(chain.size());
          merged.lane_displacement.y /= static_cast<long double>(chain.size());
          for (std::size_t member = 1U; member < chain.size(); ++member) {
            consumed[chain[member]] = true;
          }
          ++result.spline_merged_runs;
          result.spline_merged_fragments += chain.size();
        } else {
          merged = unmerged;
        }
      }
      merged_routes.push_back(std::move(merged));
    }
    routes = std::move(merged_routes);
    for (auto& route : routes) {
      if (route.topology_signature.valid) {
        ++result.spline_topology_gate_signatures;
        result.spline_topology_gates += route.topology_signature.gates.size();
      }
      route.topology_cell_index = prepare_regional_topology_cell_index(route.topology_cells);
      const auto configured_spacing = result.configured.navmesh_cell_pitch.value() > 0
                                          ? result.configured.navmesh_cell_pitch.value() / 4
                                          : Nanometres::from_mm(0.25).value();
      const auto sample_spacing = Nanometres{
          std::clamp<std::int64_t>(configured_spacing, 1, Nanometres::from_mm(0.25).value())};
      const auto simplified_seed = string_pull_regional_spline_seed(route, sample_spacing);
      // Prefer one control per meaningful turn. Only a corner whose smooth cubic would leave the
      // selected corridor receives additional coincident controls, and use the smallest bounded
      // concentration that proves topology containment.
      for (const auto corner_multiplicity : {1U, 2U, 4U}) {
        const auto simplified =
            make_topology_tight_regional_spline(simplified_seed, corner_multiplicity);
        if (simplified && regional_spline_stays_in_topology(route, *simplified)) {
          route.spline = *simplified;
          break;
        }
      }
    }
    std::vector<std::vector<SplineVector>> route_samples(routes.size());
    const auto refresh_route_samples = [&](std::size_t index) {
      const auto sampled = adaptive_tessellate_cubic_bspline(
          routes[index].spline,
          Nanometres{std::max<std::int64_t>(1, result.configured.window_margin.value() / 20)}, 8U,
          128U);
      auto& prepared = route_samples[index];
      prepared.clear();
      prepared.reserve(sampled.size());
      for (const auto& sample : sampled) {
        prepared.push_back(sample.position);
      }
    };
    for (std::size_t index = 0U; index < routes.size(); ++index) {
      refresh_route_samples(index);
    }
    if (std::any_of(route_samples.begin(), route_samples.end(),
                    [](const auto& samples) { return samples.size() < 2U; })) {
      assignment.stop_reason = RegionalBundleStopReason::spline_realization_failed;
      continue;
    }
    // Legalize the selected lane bundle collectively before asking the ordinary smoothness
    // optimizer to refine each route. This is a bounded elastic-band/PNS step: the most deeply
    // colliding interior curve sample supplies the obstacle normal and cubic basis influence,
    // while a whole-route legality score prevents a shove from trading clearance for a corridor
    // crossing. Every window is still committed only as one exact-clean transaction.
    constexpr std::size_t joint_samples = 33U;
    for (std::size_t sweep = 0U; sweep < result.configured.max_spline_sweeps; ++sweep) {
      bool accepted_in_sweep = false;
      for (std::size_t index = 0U; index < routes.size(); ++index) {
        if (routes[index].frozen_boundary) {
          continue;
        }
        const auto current =
            assess_regional_spline_legality(routes, index, route_samples, joint_samples);
        if (current.mutual_violations == 0U || !std::isfinite(current.worst_clearance_nm) ||
            !(current.worst_clearance_nm < 0.0L)) {
          continue;
        }
        auto normal = current.worst_normal;
        if (!(std::hypotl(normal.x, normal.y) > 0.0L)) {
          normal = routes[index].lane_displacement;
        }
        if (!(std::hypotl(normal.x, normal.y) > 0.0L)) {
          const auto tangent =
              evaluate_cubic_bspline_differential(routes[index].spline, current.worst_parameter)
                  .tangent;
          normal = SplineVector{-tangent.y, tangent.x};
          if ((index & 1U) != 0U) {
            normal.x = -normal.x;
            normal.y = -normal.y;
          }
        }
        if (!(std::hypotl(normal.x, normal.y) > 0.0L)) {
          continue;
        }
        const auto original = routes[index].spline;
        auto distance = std::max<std::int64_t>(
            1, static_cast<std::int64_t>(std::ceil(-current.worst_clearance_nm + 1.0L)));
        bool accepted = false;
        for (std::size_t backtrack = 0U; backtrack < 8U && distance > 0; ++backtrack) {
          ++result.spline_joint_shove_attempts;
          routes[index].spline = original;
          const auto moved = shove_cubic_bspline_control_corridor(
              routes[index].spline, current.worst_parameter, normal, Nanometres{distance},
              result.configured.window_margin);
          if (moved == 0U) {
            break;
          }
          refresh_route_samples(index);
          const auto candidate =
              assess_regional_spline_legality(routes, index, route_samples, joint_samples);
          if (better_regional_spline_legality(candidate, current)) {
            routes[index].control_moves += moved;
            ++result.spline_joint_shove_accepted;
            accepted = true;
            accepted_in_sweep = true;
            break;
          }
          distance /= 2;
        }
        if (!accepted) {
          routes[index].spline = original;
          refresh_route_samples(index);
        }
      }
      if (!accepted_in_sweep) {
        break;
      }
    }
    for (std::size_t sweep = 0U; sweep < result.configured.max_spline_sweeps; ++sweep) {
      bool moved = false;
      for (std::size_t index = 0U; index < routes.size(); ++index) {
        if (routes[index].frozen_boundary) {
          continue;
        }
        SplineTrajectoryOptimizationConfig config;
        config.minimum_clearance = Nanometres{0};
        config.control_step =
            Nanometres{std::max<std::int64_t>(1, result.configured.window_margin.value() / 10)};
        config.gradient_probe_step =
            Nanometres{std::max<std::int64_t>(1, config.control_step.value() / 5)};
        config.maximum_corridor_displacement = result.configured.window_margin;
        config.samples = 32U;
        config.maximum_iterations = 8U;
        config.maximum_candidate_checks = 512U;
        const auto evaluator = [&](SplineVector position) {
          return regional_spline_clearance(routes[index], position, routes, index, route_samples);
        };
        const auto original = routes[index].spline;
        const auto optimized =
            optimize_cubic_bspline_trajectory(routes[index].spline, config, evaluator);
        refresh_route_samples(index);
        if (!regional_spline_gate_sequence(routes[index], route_samples[index]).preserved) {
          routes[index].spline = original;
          refresh_route_samples(index);
          ++result.spline_topology_gate_rejections;
          continue;
        }
        routes[index].control_moves += optimized.accepted_control_moves;
        routes[index].sweeps = sweep + 1U;
        moved = moved || optimized.accepted_control_moves > 0U;
      }
      if (!moved) {
        break;
      }
    }

    // Finish with bounded two-route transactions judged by the same adaptive whole-window
    // representation used by final realization. Earlier sampled pair shoves could improve one
    // curve while regressing the complete window; this phase accepts only a lexicographic decrease
    // in adaptive topology/mutual violations and keeps the unchanged routes as its fallback.
    const auto has_mutual_pressure = [&]() {
      for (std::size_t route = 0U; route < routes.size(); ++route) {
        for (const auto& position : route_samples[route]) {
          if (regional_spline_clearance(routes[route], position, routes, route, route_samples)
                  .clearance_nm < 0.0L) {
            return true;
          }
        }
      }
      return false;
    }();
    if (has_mutual_pressure) {
      const auto adaptive_baseline = assess_adaptive_regional_window(routes);
      auto adaptive_current = adaptive_baseline;
      bool adaptive_window_improved = false;
      std::size_t adaptive_candidate_checks = 0U;
      for (std::size_t sweep = 0U; sweep < result.configured.max_spline_sweeps; ++sweep) {
        if (adaptive_candidate_checks >=
            result.configured.max_adaptive_pair_shove_checks_per_window) {
          break;
        }
        if (!std::isfinite(adaptive_current.worst_clearance_nm) ||
            !(adaptive_current.worst_clearance_nm < 0.0L) ||
            adaptive_current.worst_route >= routes.size()) {
          break;
        }
        const auto route = adaptive_current.worst_route;
        auto normal = adaptive_current.worst_normal;
        if (!(std::hypotl(normal.x, normal.y) > 0.0L)) {
          const auto tangent = evaluate_cubic_bspline_differential(routes[route].spline,
                                                                   adaptive_current.worst_parameter)
                                   .tangent;
          normal = SplineVector{-tangent.y, tangent.x};
        }
        const auto normal_length = std::hypotl(normal.x, normal.y);
        if (!(normal_length > 0.0L)) {
          break;
        }
        normal.x /= normal_length;
        normal.y /= normal_length;
        const auto collision_position =
            evaluate_cubic_bspline(routes[route].spline, adaptive_current.worst_parameter);
        const auto base_distance = std::max<std::int64_t>(
            1, static_cast<std::int64_t>(std::ceil(-adaptive_current.worst_clearance_nm + 1.0L)));
        std::optional<std::vector<WorkingRegionalSpline>> best_routes;
        std::optional<AdaptiveRegionalWindowAssessment> best_assessment;
        struct PairShoveOther {
          std::size_t route{0};
          long double parameter{0.5L};
          long double distance_nm{0.0L};
        };
        std::vector<PairShoveOther> other_routes;
        for (std::size_t other = 0U; other < routes.size(); ++other) {
          if (other == route || routes[other].layer != routes[route].layer ||
              routes[other].fragment.net == routes[route].fragment.net) {
            continue;
          }
          const auto samples = adaptive_tessellate_cubic_bspline(
              routes[other].spline, Nanometres::from_mm(0.02), 10U, 512U);
          if (samples.size() < 3U) {
            continue;
          }
          const auto closest = std::min_element(
              samples.begin() + 1, samples.end() - 1, [&](const auto& lhs, const auto& rhs) {
                return std::hypotl(lhs.position.x - collision_position.x,
                                   lhs.position.y - collision_position.y) <
                       std::hypotl(rhs.position.x - collision_position.x,
                                   rhs.position.y - collision_position.y);
              });
          other_routes.push_back(
              PairShoveOther{other, closest->parameter,
                             std::hypotl(closest->position.x - collision_position.x,
                                         closest->position.y - collision_position.y)});
        }
        std::stable_sort(
            other_routes.begin(), other_routes.end(), [](const auto& lhs, const auto& rhs) {
              return lhs.distance_nm != rhs.distance_nm ? lhs.distance_nm < rhs.distance_nm
                                                        : lhs.route < rhs.route;
            });
        for (const auto& other_candidate : other_routes) {
          if (adaptive_candidate_checks >=
              result.configured.max_adaptive_pair_shove_checks_per_window) {
            break;
          }
          const auto other = other_candidate.route;
          auto distance = base_distance;
          for (std::size_t backtrack = 0U; backtrack < 4U && distance > 0; ++backtrack) {
            if (adaptive_candidate_checks >=
                result.configured.max_adaptive_pair_shove_checks_per_window) {
              break;
            }
            ++adaptive_candidate_checks;
            ++result.spline_adaptive_pair_shove_attempts;
            auto trial = routes;
            const auto movable_other = !trial[other].frozen_boundary;
            const auto route_distance =
                movable_other ? std::max<std::int64_t>(1, (distance + 1) / 2) : distance;
            const auto route_moves = shove_cubic_bspline_control_corridor(
                trial[route].spline, adaptive_current.worst_parameter, normal,
                Nanometres{route_distance}, result.configured.window_margin);
            std::size_t other_moves = 0U;
            if (movable_other) {
              other_moves = shove_cubic_bspline_control_corridor(
                  trial[other].spline, other_candidate.parameter,
                  SplineVector{-normal.x, -normal.y},
                  Nanometres{std::max<std::int64_t>(1, distance / 2)},
                  result.configured.window_margin);
            }
            if (route_moves == 0U || (movable_other && other_moves == 0U)) {
              distance /= 2;
              continue;
            }
            const auto assessment = assess_adaptive_regional_window(trial);
            if (assessment.objective() < adaptive_current.objective() &&
                (!best_assessment || assessment.objective() < best_assessment->objective())) {
              trial[route].control_moves += route_moves;
              trial[other].control_moves += other_moves;
              best_routes = std::move(trial);
              best_assessment = assessment;
            }
            distance /= 2;
          }
        }
        if (!best_routes || !best_assessment) {
          break;
        }
        routes = std::move(*best_routes);
        adaptive_current = *best_assessment;
        ++result.spline_adaptive_pair_shove_accepted;
        adaptive_window_improved = true;
      }
      if (adaptive_window_improved) {
        ++result.spline_adaptive_pair_windows_improved;
        if ((adaptive_baseline.owned_topology_illegal_routes > 0U ||
             adaptive_baseline.owned_mutual_illegal_routes > 0U) &&
            adaptive_current.owned_topology_illegal_routes == 0U &&
            adaptive_current.owned_mutual_illegal_routes == 0U) {
          ++result.spline_adaptive_pair_windows_legalized;
        }
        for (std::size_t index = 0U; index < routes.size(); ++index) {
          refresh_route_samples(index);
        }
      }
    }
    std::vector<std::vector<SplineSample>> final_route_samples(routes.size());
    std::vector<std::vector<Point>> quantized_route_samples(routes.size());
    const auto refresh_final_samples = [&]() {
      for (std::size_t index = 0U; index < routes.size(); ++index) {
        final_route_samples[index] = adaptive_tessellate_cubic_bspline(
            routes[index].spline, Nanometres::from_mm(0.02), 10U, 512U);
        route_samples[index].clear();
        route_samples[index].reserve(final_route_samples[index].size());
        quantized_route_samples[index].clear();
        quantized_route_samples[index].reserve(final_route_samples[index].size());
        for (const auto& sample : final_route_samples[index]) {
          route_samples[index].push_back(sample.position);
          quantized_route_samples[index].push_back(quantize_regional_spline(sample.position));
        }
      }
    };
    refresh_final_samples();
    const auto continuous_before_quantization = assess_adaptive_regional_window(routes);
    auto quantized_assessment = assess_quantized_regional_window(
        routes, quantized_route_samples,
        result.configured.max_quantized_segment_pair_checks_per_window);
    result.quantized_segment_pair_checks += quantized_assessment.exact_checks;
    const auto initial_quantized_conflicts = quantized_assessment.conflicts;
    bool quantization_budget_seen = quantized_assessment.budget_exhausted;
    const auto continuous_window_legal =
        continuous_before_quantization.owned_topology_illegal_routes == 0U &&
        continuous_before_quantization.owned_mutual_illegal_routes == 0U;
    if (continuous_window_legal && quantized_assessment.conflicts > 0U &&
        !quantized_assessment.budget_exhausted) {
      for (std::size_t iteration = 0U;
           iteration < result.configured.max_quantization_feedback_iterations; ++iteration) {
        if (!quantized_assessment.worst_collision) {
          break;
        }
        const auto collision = *quantized_assessment.worst_collision;
        auto route = collision.first_route;
        auto segment = collision.first_segment;
        auto normal = collision.first_away_normal;
        if (route >= routes.size() || routes[route].frozen_boundary) {
          route = collision.second_route;
          segment = collision.second_segment;
          normal.x = -normal.x;
          normal.y = -normal.y;
        }
        if (route >= routes.size() || routes[route].frozen_boundary ||
            route >= final_route_samples.size() ||
            segment + 1U >= final_route_samples[route].size()) {
          break;
        }
        auto parameter = (final_route_samples[route][segment].parameter +
                          final_route_samples[route][segment + 1U].parameter) /
                         2.0L;
        if (!(std::hypotl(normal.x, normal.y) > 0.0L)) {
          const auto tangent =
              evaluate_cubic_bspline_differential(routes[route].spline, parameter).tangent;
          normal = SplineVector{-tangent.y, tangent.x};
          const auto length = std::hypotl(normal.x, normal.y);
          if (length > 0.0L) {
            normal.x /= length;
            normal.y /= length;
          }
        }
        if (!(std::hypotl(normal.x, normal.y) > 0.0L)) {
          break;
        }
        ++result.quantization_feedback_attempts;
        const auto original = routes[route].spline;
        const auto distance = Nanometres{std::max<std::int64_t>(
            1, static_cast<std::int64_t>(std::ceil(collision.deficit_nm + 2.0L)))};
        const auto moved = shove_cubic_bspline_control_corridor(
            routes[route].spline, parameter, normal, distance, result.configured.window_margin);
        if (moved == 0U) {
          routes[route].spline = original;
          break;
        }
        refresh_final_samples();
        const auto continuous_candidate = assess_adaptive_regional_window(routes);
        auto quantized_candidate = assess_quantized_regional_window(
            routes, quantized_route_samples,
            result.configured.max_quantized_segment_pair_checks_per_window);
        result.quantized_segment_pair_checks += quantized_candidate.exact_checks;
        quantization_budget_seen = quantization_budget_seen || quantized_candidate.budget_exhausted;
        const auto continuous_candidate_legal =
            continuous_candidate.owned_topology_illegal_routes == 0U &&
            continuous_candidate.owned_mutual_illegal_routes == 0U;
        if (!quantized_candidate.budget_exhausted && continuous_candidate_legal &&
            quantized_candidate.conflicts < quantized_assessment.conflicts) {
          routes[route].control_moves += moved;
          quantized_assessment = std::move(quantized_candidate);
          ++result.quantization_feedback_accepted;
          if (quantized_assessment.conflicts == 0U) {
            break;
          }
          continue;
        }
        routes[route].spline = original;
        refresh_final_samples();
        break;
      }
    }
    result.quantized_clearance_conflicts += quantized_assessment.conflicts;
    if (quantized_assessment.worst_collision) {
      result.peak_quantized_clearance_deficit =
          std::max(result.peak_quantized_clearance_deficit,
                   Nanometres{static_cast<std::int64_t>(
                       std::ceil(quantized_assessment.worst_collision->deficit_nm))});
    }
    if (quantization_budget_seen) {
      ++result.quantization_check_budget_exhausted_windows;
    }
    if (initial_quantized_conflicts > 0U && quantized_assessment.conflicts == 0U &&
        !quantized_assessment.budget_exhausted) {
      ++result.quantization_feedback_windows_legalized;
    }
    bool all_legal = true;
    for (std::size_t index = 0U; index < routes.size(); ++index) {
      const auto& samples = final_route_samples[index];
      RegionalSplineRealization realized;
      realized.fragment = routes[index].fragment;
      realized.replacement_end = routes[index].replacement_end;
      realized.layer = routes[index].layer;
      realized.width = routes[index].width;
      realized.width_kind = routes[index].width_kind;
      realized.entry_via_site = routes[index].entry_via_site;
      realized.exit_via_site = routes[index].exit_via_site;
      realized.optimization_sweeps = routes[index].sweeps;
      realized.accepted_control_moves = routes[index].control_moves;
      realized.topology_signature = routes[index].topology_signature.fingerprint;
      realized.topology_gate_count = routes[index].topology_signature.gates.size();
      realized.control_points.reserve(routes[index].spline.control_points.size());
      for (const auto& control : routes[index].spline.control_points) {
        realized.control_points.push_back(quantize_regional_spline(control));
      }
      for (const auto& sample : samples) {
        const auto point = quantize_regional_spline(sample.position);
        if (realized.centerline.empty() || realized.centerline.back() != point) {
          realized.centerline.push_back(point);
        }
      }
      const auto realized_point_is_in_topology = [&](Point point) {
        return regional_spline_topology_contains(
            routes[index], SplineVector{static_cast<long double>(point.x.value()),
                                        static_cast<long double>(point.y.value())});
      };
      const auto endpoints_topology_legal =
          !realized.centerline.empty() &&
          realized_point_is_in_topology(realized.centerline.front()) &&
          realized_point_is_in_topology(realized.centerline.back());
      const auto interior_topology_legal =
          realized.centerline.size() <= 2U ||
          std::all_of(realized.centerline.begin() + 1, realized.centerline.end() - 1,
                      realized_point_is_in_topology);
      const auto gate_sequence = regional_spline_gate_sequence(routes[index], route_samples[index]);
      realized.topology_gate_sequence_legal = gate_sequence.preserved;
      realized.topology_legal = endpoints_topology_legal && interior_topology_legal &&
                                realized.topology_gate_sequence_legal;
      realized.endpoint_topology_legal = endpoints_topology_legal;
      realized.interior_topology_legal = interior_topology_legal;
      realized.mutually_legal =
          std::all_of(samples.begin(), samples.end(), [&](const auto& sample) {
            return regional_spline_clearance(routes[index], sample.position, routes, index,
                                             route_samples)
                .represented_legal;
          });
      realized.quantized_mutually_legal = !quantized_assessment.budget_exhausted &&
                                          index < quantized_assessment.route_legal.size() &&
                                          quantized_assessment.route_legal[index];
      if (!routes[index].owned) {
        continue;
      }
      result.spline_short_realization_rejections += realized.centerline.size() < 2U ? 1U : 0U;
      result.spline_topology_rejections += !realized.topology_legal ? 1U : 0U;
      result.spline_topology_endpoint_rejections += !endpoints_topology_legal ? 1U : 0U;
      result.spline_topology_interior_rejections += !interior_topology_legal ? 1U : 0U;
      result.spline_topology_gate_rejections += !realized.topology_gate_sequence_legal ? 1U : 0U;
      result.spline_mutual_clearance_rejections += !realized.mutually_legal ? 1U : 0U;
      all_legal = all_legal && realized.centerline.size() >= 2U && realized.topology_legal &&
                  realized.mutually_legal && realized.quantized_mutually_legal;
      if (routes[index].emit_replacement) {
        assignment.spline_realizations.push_back(std::move(realized));
      }
    }
    if (!all_legal) {
      assignment.stop_reason = RegionalBundleStopReason::spline_realization_failed;
    }
  }

  // A shortlist is an atomic continuous feasibility test for one joint window, not multiple
  // regional assignments. Reduce in stable objective/rank order and retain the first complete
  // realization. If every candidate fails, retain the lexicographically least-illegal complete
  // diagnostic realization so the next repair wave receives the strongest partial checkpoint;
  // no failed or partial candidate is ever promoted to copper.
  std::vector<RegionalBundleAssignment> selected_assignments;
  selected_assignments.reserve(result.windows.size());
  for (std::size_t index = 0U; index < result.bundle_assignments.size();) {
    if (!result.bundle_assignments[index].joint_spline_candidate) {
      selected_assignments.push_back(std::move(result.bundle_assignments[index]));
      ++index;
      continue;
    }
    const auto window_id = result.bundle_assignments[index].window_id;
    auto end = index;
    while (end < result.bundle_assignments.size() &&
           result.bundle_assignments[end].joint_spline_candidate &&
           result.bundle_assignments[end].window_id == window_id) {
      ++end;
    }
    result.joint_spline_candidates_evaluated += end - index;
    result.joint_spline_candidates_legal += static_cast<std::size_t>(
        std::count_if(result.bundle_assignments.begin() + static_cast<std::ptrdiff_t>(index),
                      result.bundle_assignments.begin() + static_cast<std::ptrdiff_t>(end),
                      [](const auto& candidate) { return candidate.complete(); }));
    struct SplineFailureProfile {
      std::size_t short_routes{0U};
      std::size_t topology{0U};
      std::size_t mutual{0U};
      std::size_t quantization{0U};
    };
    const auto failure_profile = [](const RegionalBundleAssignment& candidate) {
      return SplineFailureProfile{
          static_cast<std::size_t>(std::count_if(
              candidate.spline_realizations.begin(), candidate.spline_realizations.end(),
              [](const auto& realization) { return realization.centerline.size() < 2U; })),
          static_cast<std::size_t>(std::count_if(
              candidate.spline_realizations.begin(), candidate.spline_realizations.end(),
              [](const auto& realization) { return !realization.topology_legal; })),
          static_cast<std::size_t>(std::count_if(
              candidate.spline_realizations.begin(), candidate.spline_realizations.end(),
              [](const auto& realization) { return !realization.mutually_legal; })),
          static_cast<std::size_t>(std::count_if(
              candidate.spline_realizations.begin(), candidate.spline_realizations.end(),
              [](const auto& realization) { return !realization.quantized_mutually_legal; }))};
    };
    const auto initial_failure = failure_profile(result.bundle_assignments[index]);
    const auto diagnosed_kind = [&]() {
      if (result.bundle_assignments[index].spline_realizations.empty() ||
          initial_failure.short_routes > 0U) {
        return RoutabilityFailureKind::curvature_infeasible;
      }
      if (initial_failure.quantization > 0U && initial_failure.topology == 0U &&
          initial_failure.mutual == 0U) {
        return RoutabilityFailureKind::quantization_clearance;
      }
      return initial_failure.mutual >= initial_failure.topology
                 ? RoutabilityFailureKind::shove_deadlock
                 : RoutabilityFailureKind::topological_trap;
    }();
    const auto diagnosed_count = [&](const RegionalBundleAssignment& candidate) {
      const auto profile = failure_profile(candidate);
      switch (diagnosed_kind) {
      case RoutabilityFailureKind::curvature_infeasible:
        return profile.short_routes;
      case RoutabilityFailureKind::shove_deadlock:
        return profile.mutual;
      case RoutabilityFailureKind::topological_trap:
        return profile.topology;
      case RoutabilityFailureKind::quantization_clearance:
        return profile.quantization;
      default:
        return profile.topology + profile.mutual + profile.quantization + profile.short_routes;
      }
    };
    auto selected = index;
    bool found_legal = false;
    for (auto candidate = index; candidate < end; ++candidate) {
      if (result.bundle_assignments[candidate].complete()) {
        selected = candidate;
        found_legal = true;
        break;
      }
    }
    if (!found_legal) {
      const auto maximum_realizations =
          std::max_element(result.bundle_assignments.begin() + static_cast<std::ptrdiff_t>(index),
                           result.bundle_assignments.begin() + static_cast<std::ptrdiff_t>(end),
                           [](const auto& lhs, const auto& rhs) {
                             return lhs.spline_realizations.size() < rhs.spline_realizations.size();
                           })
              ->spline_realizations.size();
      const auto failed_objective = [&](const RegionalBundleAssignment& candidate) {
        const auto profile = failure_profile(candidate);
        return std::tuple{maximum_realizations - candidate.spline_realizations.size(),
                          profile.short_routes,
                          profile.topology + profile.mutual + profile.quantization,
                          diagnosed_count(candidate),
                          profile.topology,
                          profile.mutual,
                          profile.quantization,
                          candidate.selected_template_conflicts,
                          candidate.selected_residual_conflicts,
                          candidate.spline_candidate_rank};
      };
      const auto rank_zero_objective = failed_objective(result.bundle_assignments[index]);
      for (auto candidate = index + 1U; candidate < end; ++candidate) {
        if (failed_objective(result.bundle_assignments[candidate]) <
            failed_objective(result.bundle_assignments[selected])) {
          selected = candidate;
        }
      }
      if (failed_objective(result.bundle_assignments[selected]) < rank_zero_objective) {
        ++result.joint_spline_failed_fallback_improvements;
        result.joint_spline_failed_improved_window_ids.push_back(window_id);
      }
    }
    if (!result.bundle_assignments[index].complete()) {
      const auto window =
          std::find_if(result.windows.begin(), result.windows.end(),
                       [&](const auto& candidate) { return candidate.id == window_id; });
      const auto selected_improves_diagnosis =
          selected != index && (result.bundle_assignments[selected].complete() ||
                                diagnosed_count(result.bundle_assignments[selected]) <
                                    diagnosed_count(result.bundle_assignments[index]));
      if (selected_improves_diagnosis) {
        ++result.diagnosis_directed_spline_selections;
        const auto& initial = result.bundle_assignments[index];
        const auto& replacement = result.bundle_assignments[selected];
        result.diagnosis_directed_template_changes +=
            initial.selected_templates != replacement.selected_templates ? 1U : 0U;
        result.diagnosis_directed_via_changes +=
            initial.selected_via_choices != replacement.selected_via_choices ? 1U : 0U;
        const auto lane_orders = [](const RegionalBundleAssignment& assignment) {
          std::vector<std::vector<RegionalRouteFragmentId>> orders;
          orders.reserve(assignment.packed_cross_sections.size());
          for (const auto& cross_section : assignment.packed_cross_sections) {
            orders.push_back(cross_section.ordered_occupants);
          }
          return orders;
        };
        result.diagnosis_directed_lane_order_changes +=
            lane_orders(initial) != lane_orders(replacement) ? 1U : 0U;
      }
      if (window != result.windows.end()) {
        const auto relaxation = diagnosed_kind == RoutabilityFailureKind::quantization_clearance
                                    ? RoutabilityRelaxation::increase_local_clearance_margin
                                : diagnosed_kind == RoutabilityFailureKind::shove_deadlock
                                    ? RoutabilityRelaxation::change_lane_order
                                : diagnosed_kind == RoutabilityFailureKind::topological_trap
                                    ? RoutabilityRelaxation::change_corridor
                                    : RoutabilityRelaxation::change_corridor;
        const auto stage = diagnosed_kind == RoutabilityFailureKind::quantization_clearance
                               ? RoutabilityFailureStage::quantization
                           : diagnosed_kind == RoutabilityFailureKind::shove_deadlock
                               ? RoutabilityFailureStage::local_pns
                               : RoutabilityFailureStage::spline_realization;
        const auto selected_profile = failure_profile(result.bundle_assignments[selected]);
        const auto severity = diagnosed_kind == RoutabilityFailureKind::quantization_clearance
                                  ? std::max<std::size_t>(1U, initial_failure.quantization)
                              : diagnosed_kind == RoutabilityFailureKind::curvature_infeasible
                                  ? std::max<std::size_t>(1U, initial_failure.short_routes)
                              : diagnosed_kind == RoutabilityFailureKind::shove_deadlock
                                  ? std::max<std::size_t>(1U, initial_failure.mutual)
                                  : std::max<std::size_t>(1U, initial_failure.topology);
        RoutabilityDiagnosis diagnosis{
            diagnosed_kind,
            stage,
            RoutabilityResourceReference{"regional-window:" + std::to_string(window_id) + ":" +
                                             std::string(to_string(diagnosed_kind)),
                                         window->layers.empty() ? LayerId{}
                                                                : window->layers.front(),
                                         window->bounds, window->nets},
            static_cast<std::uint64_t>(severity),
            window->peak_clearance_deficit,
            std::nullopt,
            std::nullopt,
            relaxation,
            "initial spline/PNS realization diagnosed " + std::string(to_string(diagnosed_kind)) +
                "; selected state has topology=" + std::to_string(selected_profile.topology) +
                ", mutual=" + std::to_string(selected_profile.mutual) +
                ", quantization=" + std::to_string(selected_profile.quantization) +
                ", short=" + std::to_string(selected_profile.short_routes),
            result.bundle_assignments[selected].complete()};
        result.locally_resolved_routability_diagnoses += diagnosis.locally_resolved ? 1U : 0U;
        if (result.local_routability_diagnoses.size() <
            result.configured.max_local_routability_diagnoses) {
          result.local_routability_diagnoses.push_back(diagnosis);
        } else {
          ++result.dropped_local_routability_diagnoses;
        }
        if (telemetry != nullptr) {
          RouterTelemetryEvent event;
          event.kind = RouterTelemetryEventKind::search_summary;
          event.group_id = "regional-window:" + std::to_string(window_id);
          event.stable_id = diagnosis.resource.stable_id;
          event.detail = diagnosis.detail + ", locally_resolved=" +
                         std::string(diagnosis.locally_resolved ? "yes" : "no");
          static_cast<void>(telemetry->try_emit(std::move(event)));
        }
      }
    }
    if (result.bundle_assignments[selected].complete() &&
        result.bundle_assignments[selected].spline_candidate_rank > 0U) {
      ++result.joint_spline_windows_recovered;
      result.joint_spline_recovered_window_ids.push_back(window_id);
    }
    selected_assignments.push_back(std::move(result.bundle_assignments[selected]));
    index = end;
  }
  result.bundle_assignments = std::move(selected_assignments);

  result.selected_lane_cross_sections = 0U;
  result.selected_geometric_lane_reorders = 0U;
  result.avoided_lane_order_inversions = 0U;
  for (const auto& assignment : result.bundle_assignments) {
    if (!assignment.complete()) {
      continue;
    }
    for (const auto& cross_section : assignment.packed_cross_sections) {
      ++result.selected_lane_cross_sections;
      auto stable_order = cross_section.ordered_occupants;
      std::sort(stable_order.begin(), stable_order.end());
      result.selected_geometric_lane_reorders +=
          stable_order != cross_section.ordered_occupants ? 1U : 0U;
      if (cross_section.stable_order_inversions > cross_section.selected_order_inversions) {
        result.avoided_lane_order_inversions +=
            cross_section.stable_order_inversions - cross_section.selected_order_inversions;
      }
    }
  }

  // Trial failures are search evidence, but the longstanding rejection counters describe the
  // retained regional state. Recompute them after shortlist reduction so downstream comparison
  // and telemetry are not biased by discarded alternatives.
  result.spline_short_realization_rejections = 0U;
  result.spline_topology_rejections = 0U;
  result.spline_topology_endpoint_rejections = 0U;
  result.spline_topology_interior_rejections = 0U;
  result.spline_topology_gate_signatures = 0U;
  result.spline_topology_gates = 0U;
  result.spline_topology_gate_rejections = 0U;
  result.spline_mutual_clearance_rejections = 0U;
  result.selected_terminal_portal_choices = 0U;
  std::set<std::size_t> selected_terminal_portals;
  for (const auto& assignment : result.bundle_assignments) {
    for (const auto template_id : assignment.selected_templates) {
      if (template_id >= assignment.templates.size()) {
        continue;
      }
      const auto& route_template = assignment.templates[template_id];
      for (const auto choice_id :
           {route_template.terminal_entry_choice, route_template.terminal_exit_choice}) {
        if (!choice_id) {
          continue;
        }
        const auto choice =
            std::find_if(result.terminal_choices.begin(), result.terminal_choices.end(),
                         [&](const auto& candidate) { return candidate.id == *choice_id; });
        if (choice != result.terminal_choices.end() && !choice->selected_global_access) {
          selected_terminal_portals.insert(choice->id);
        }
      }
    }
    for (const auto& realization : assignment.spline_realizations) {
      result.spline_short_realization_rejections += realization.centerline.size() < 2U ? 1U : 0U;
      result.spline_topology_rejections += !realization.topology_legal ? 1U : 0U;
      result.spline_topology_endpoint_rejections += !realization.endpoint_topology_legal ? 1U : 0U;
      result.spline_topology_interior_rejections += !realization.interior_topology_legal ? 1U : 0U;
      result.spline_topology_gate_signatures += realization.topology_signature != 0U ? 1U : 0U;
      result.spline_topology_gates += realization.topology_gate_count;
      result.spline_topology_gate_rejections +=
          !realization.topology_gate_sequence_legal ? 1U : 0U;
      result.spline_mutual_clearance_rejections += !realization.mutually_legal ? 1U : 0U;
    }
  }
  result.selected_terminal_portal_choices = selected_terminal_portals.size();
}

void build_regional_spline_candidate(const Board& board, const GlobalRouteResult& global,
                                     RegionalRouteResult& result) {
  result.spline_candidate_paths.clear();
  result.multilayer_route_plans.clear();
  result.complete_via_dependent_routes = 0U;
  result.planned_layer_runs = 0U;
  result.planned_via_transitions = 0U;
  result.spline_candidate_available = false;
  result.spline_candidate_conflicting_articulation = false;
  if (result.bundle_assignments.empty() ||
      std::any_of(result.bundle_assignments.begin(), result.bundle_assignments.end(),
                  [](const auto& assignment) { return !assignment.complete(); })) {
    return;
  }
  struct Replacement {
    LayerId layer;
    std::vector<Point> centerline;
    Point replacement_end;
    Nanometres width;
    GlobalRouteSegmentWidthKind width_kind{GlobalRouteSegmentWidthKind::nominal};
    std::optional<Point> entry_via_site;
    std::optional<Point> exit_via_site;

    bool operator==(const Replacement&) const = default;
  };
  std::map<RegionalRouteFragmentId, Replacement> replacements;
  std::map<std::pair<NetId, Point>, Point> relocated_via_sites;
  for (const auto& assignment : result.bundle_assignments) {
    for (const auto& realization : assignment.spline_realizations) {
      const auto [found, inserted] = replacements.emplace(
          realization.fragment,
          Replacement{realization.layer, realization.centerline, realization.replacement_end,
                      realization.width, realization.width_kind, realization.entry_via_site,
                      realization.exit_via_site});
      if (!inserted &&
          found->second != Replacement{realization.layer, realization.centerline,
                                       realization.replacement_end, realization.width,
                                       realization.width_kind, realization.entry_via_site,
                                       realization.exit_via_site}) {
        result.spline_candidate_conflicting_articulation = true;
        return;
      }
    }
    for (const auto choice_id : assignment.selected_via_choices) {
      const auto choice =
          std::find_if(result.via_choices.begin(), result.via_choices.end(),
                       [&](const auto& candidate) { return candidate.id == choice_id; });
      if (choice == result.via_choices.end() || choice->selected_global_site) {
        continue;
      }
      const auto source = std::find_if(
          result.via_choices.begin(), result.via_choices.end(), [&](const auto& candidate) {
            return candidate.selected_global_site && candidate.net == choice->net &&
                   candidate.region_id == choice->region_id && candidate.span == choice->span;
          });
      if (source == result.via_choices.end()) {
        continue;
      }
      const auto [found, inserted] =
          relocated_via_sites.emplace(std::pair{choice->net, source->site}, choice->site);
      if (!inserted && found->second != choice->site) {
        result.spline_candidate_conflicting_articulation = true;
        return;
      }
    }
  }
  using FragmentStartKey = std::tuple<NetId, std::size_t, std::size_t, LayerId, Point>;
  std::map<FragmentStartKey, const RegionalRouteFragment*> fragment_by_start;
  for (const auto& fragment : result.fragments) {
    fragment_by_start.emplace(FragmentStartKey{fragment.id.net, fragment.id.connection_index,
                                               fragment.id.run_index, fragment.layer,
                                               fragment.start},
                              &fragment);
  }
  result.spline_candidate_paths = global.paths;
  for (auto& path : result.spline_candidate_paths) {
    if (path.points.size() < 2U) {
      continue;
    }
    const auto original = path.points;
    const auto original_widths = path.segment_widths;
    const auto original_kinds = path.segment_width_kinds;
    std::vector<GlobalRoutePoint> points;
    std::vector<Nanometres> widths;
    std::vector<GlobalRouteSegmentWidthKind> kinds;
    points.push_back(original.front());
    std::size_t run_index = 0U;
    for (std::size_t edge = 0U; edge + 1U < original.size(); ++edge) {
      const auto first = original[edge];
      auto second = original[edge + 1U];
      if (first.layer != second.layer) {
        if (points.back() != second) {
          points.push_back(second);
          widths.push_back(Nanometres{0});
          kinds.push_back(GlobalRouteSegmentWidthKind::nominal);
        }
        ++run_index;
        continue;
      }
      if (first.position == second.position) {
        continue;
      }
      const auto source_fragment = fragment_by_start.find(FragmentStartKey{
          path.net, path.connection_index, run_index, first.layer, first.position});
      const auto replacement = source_fragment == fragment_by_start.end()
                                   ? replacements.end()
                                   : replacements.find(source_fragment->second->id);
      const auto width =
          original_widths.size() + 1U == original.size() ? original_widths[edge] : Nanometres{0};
      const auto kind = original_kinds.size() + 1U == original.size()
                            ? original_kinds[edge]
                            : GlobalRouteSegmentWidthKind::nominal;
      if (replacement == replacements.end()) {
        if (points.back() != second) {
          points.push_back(second);
          widths.push_back(width);
          kinds.push_back(kind);
        }
        continue;
      }
      while (second.position != replacement->second.replacement_end) {
        if (edge + 2U >= original.size() ||
            original[edge + 1U].layer != original[edge + 2U].layer) {
          result.spline_candidate_conflicting_articulation = true;
          result.spline_candidate_paths.clear();
          return;
        }
        ++edge;
        second = original[edge + 1U];
      }
      const auto append = [&](GlobalRoutePoint point, Nanometres edge_width,
                              GlobalRouteSegmentWidthKind edge_kind) {
        if (points.back() != point) {
          points.push_back(std::move(point));
          widths.push_back(edge_width);
          kinds.push_back(edge_kind);
        }
      };
      const auto& selected_replacement = replacement->second;
      if (selected_replacement.width != source_fragment->second->width ||
          selected_replacement.width_kind != source_fragment->second->width_kind) {
        result.spline_candidate_paths.clear();
        return;
      }
      const auto& replacement_layer = selected_replacement.layer;
      const auto encoded_replacement_width =
          selected_replacement.width_kind == GlobalRouteSegmentWidthKind::nominal
              ? Nanometres{0}
              : selected_replacement.width;
      const auto entry_site = selected_replacement.entry_via_site.value_or(first.position);
      const auto exit_site = selected_replacement.exit_via_site.value_or(second.position);
      append(GlobalRoutePoint{entry_site, first.layer}, encoded_replacement_width,
             selected_replacement.width_kind);
      if (replacement_layer != first.layer) {
        append(GlobalRoutePoint{entry_site, replacement_layer}, Nanometres{0},
               GlobalRouteSegmentWidthKind::nominal);
      }
      for (const auto point : selected_replacement.centerline) {
        const GlobalRoutePoint selected{point, replacement_layer};
        append(selected, encoded_replacement_width, selected_replacement.width_kind);
      }
      append(GlobalRoutePoint{exit_site, replacement_layer}, encoded_replacement_width,
             selected_replacement.width_kind);
      if (replacement_layer != second.layer) {
        append(GlobalRoutePoint{exit_site, second.layer}, Nanometres{0},
               GlobalRouteSegmentWidthKind::nominal);
      }
      append(second, encoded_replacement_width, selected_replacement.width_kind);
    }
    path.points = std::move(points);
    path.segment_widths = std::move(widths);
    path.segment_width_kinds = std::move(kinds);
  }
  for (auto& path : result.spline_candidate_paths) {
    for (std::size_t index = 1U; index < path.points.size(); ++index) {
      auto& first = path.points[index - 1U];
      auto& second = path.points[index];
      if (first.layer == second.layer || first.position != second.position) {
        continue;
      }
      const auto relocation = relocated_via_sites.find({path.net, first.position});
      if (relocation != relocated_via_sites.end()) {
        first.position = relocation->second;
        second.position = relocation->second;
      }
    }
  }
  const auto saturating_length_add = [](std::uint64_t total, std::uint64_t amount) {
    return total > std::numeric_limits<std::uint64_t>::max() - amount
               ? std::numeric_limits<std::uint64_t>::max()
               : total + amount;
  };
  const auto resource_width_edges = [](const GlobalRoutePath& path) {
    using Edge = std::tuple<GlobalRouteSegmentWidthKind, LayerId, Point, Point, Nanometres>;
    std::vector<Edge> edges;
    if (path.segment_widths.size() + 1U != path.points.size() ||
        path.segment_width_kinds.size() != path.segment_widths.size()) {
      return edges;
    }
    for (std::size_t edge = 0U; edge < path.segment_widths.size(); ++edge) {
      const auto kind = path.segment_width_kinds[edge];
      if (kind == GlobalRouteSegmentWidthKind::medial_neck_down ||
          kind == GlobalRouteSegmentWidthKind::corridor_neck_down) {
        edges.emplace_back(kind, path.points[edge].layer, path.points[edge].position,
                           path.points[edge + 1U].position, path.segment_widths[edge]);
      }
    }
    return edges;
  };
  const auto update_width_evidence = [&](GlobalRoutePath& path, const GlobalRoutePath& source) {
    if (path.segment_widths.empty() && path.segment_width_kinds.empty()) {
      return true;
    }
    if (path.segment_widths.size() + 1U != path.points.size() ||
        path.segment_width_kinds.size() != path.segment_widths.size() ||
        resource_width_edges(path) != resource_width_edges(source)) {
      return false;
    }
    path.terminal_neck_down_segments = 0U;
    path.terminal_neck_down_length_nm = 0U;
    path.medial_neck_down_segments = 0U;
    path.medial_neck_down_length_nm = 0U;
    path.corridor_neck_down_segments = 0U;
    path.corridor_neck_down_length_nm = 0U;
    long double terminal_run = 0.0L;
    long double corridor_run = 0.0L;
    const auto finish_terminal = [&]() {
      const auto legal =
          terminal_run <=
          static_cast<long double>(global.configured_max_terminal_neck_down_length.value());
      terminal_run = 0.0L;
      return legal;
    };
    const auto finish_corridor = [&]() {
      const auto legal =
          corridor_run <=
          static_cast<long double>(global.configured_max_corridor_neck_down_length.value());
      corridor_run = 0.0L;
      return legal;
    };
    for (std::size_t edge = 0U; edge < path.segment_widths.size(); ++edge) {
      const auto width = path.segment_widths[edge];
      const auto kind = path.segment_width_kinds[edge];
      const auto& first = path.points[edge];
      const auto& second = path.points[edge + 1U];
      if ((width.value() == 0) != (kind == GlobalRouteSegmentWidthKind::nominal) ||
          width.value() < 0 ||
          (kind != GlobalRouteSegmentWidthKind::nominal && first.layer != second.layer)) {
        return false;
      }
      const auto length = geometry::point_distance_nm(first.position, second.position);
      const auto rounded = static_cast<std::uint64_t>(std::ceil(length));
      switch (kind) {
      case GlobalRouteSegmentWidthKind::nominal:
        if (!finish_terminal() || !finish_corridor()) {
          return false;
        }
        break;
      case GlobalRouteSegmentWidthKind::terminal_neck_down:
        if (!finish_corridor() || width != global.configured_terminal_neck_down_width ||
            global.configured_max_terminal_neck_down_length.value() <= 0) {
          return false;
        }
        terminal_run += length;
        ++path.terminal_neck_down_segments;
        path.terminal_neck_down_length_nm =
            saturating_length_add(path.terminal_neck_down_length_nm, rounded);
        break;
      case GlobalRouteSegmentWidthKind::medial_neck_down:
        if (!finish_terminal() || !finish_corridor() ||
            width != global.configured_medial_neck_down_width ||
            length >
                static_cast<long double>(global.configured_max_medial_neck_down_length.value())) {
          return false;
        }
        ++path.medial_neck_down_segments;
        path.medial_neck_down_length_nm =
            saturating_length_add(path.medial_neck_down_length_nm, rounded);
        break;
      case GlobalRouteSegmentWidthKind::corridor_neck_down:
        if (!finish_terminal() || width != global.configured_corridor_neck_down_width ||
            global.configured_max_corridor_neck_down_length.value() <= 0) {
          return false;
        }
        corridor_run += length;
        ++path.corridor_neck_down_segments;
        path.corridor_neck_down_length_nm =
            saturating_length_add(path.corridor_neck_down_length_nm, rounded);
        break;
      }
    }
    return finish_terminal() && finish_corridor();
  };
  for (std::size_t path_index = 0U; path_index < result.spline_candidate_paths.size();
       ++path_index) {
    if (!update_width_evidence(result.spline_candidate_paths[path_index],
                               global.paths[path_index])) {
      result.spline_candidate_paths.clear();
      return;
    }
  }
  for (const auto& path : result.spline_candidate_paths) {
    const auto crosses_layers = std::adjacent_find(
        path.points.begin(), path.points.end(),
        [](const auto& first, const auto& second) { return first.layer != second.layer; });
    if (crosses_layers == path.points.end()) {
      continue;
    }
    auto multilayer = plan_multilayer_route(board, path);
    if (!multilayer) {
      result.spline_candidate_paths.clear();
      result.multilayer_route_plans.clear();
      result.complete_via_dependent_routes = 0U;
      result.planned_layer_runs = 0U;
      result.planned_via_transitions = 0U;
      return;
    }
    ++result.complete_via_dependent_routes;
    result.planned_layer_runs += multilayer->layer_runs.size();
    result.planned_via_transitions += multilayer->via_transitions.size();
    result.multilayer_route_plans.push_back(std::move(*multilayer));
  }
  result.spline_candidate_available = !result.spline_candidate_paths.empty();
}

[[nodiscard]] std::string regional_group_id(std::size_t component_id) {
  return "regional-group:" + std::to_string(component_id);
}

[[nodiscard]] std::string regional_via_id(std::size_t choice_id) {
  return "regional-via:" + std::to_string(choice_id);
}

void publish_regional_telemetry(const Board& board, const RegionalRouteResult& result,
                                RouterTelemetrySink* telemetry) {
  if (telemetry == nullptr) {
    return;
  }
  const auto diagnostic = telemetry->accepts(RouterTelemetryLevel::diagnostic);
  const auto emit = [telemetry](RouterTelemetryEvent event) {
    static_cast<void>(telemetry->try_emit(std::move(event)));
  };

  std::map<std::size_t, std::size_t> component_by_edge;
  for (const auto& component : result.components) {
    for (const auto edge_id : component.conflict_edges) {
      component_by_edge.try_emplace(edge_id, component.id);
    }
    RouterTelemetryEvent group;
    group.kind = RouterTelemetryEventKind::residual_group_updated;
    group.stable_id = regional_group_id(component.id);
    group.group_id = group.stable_id;
    group.total_groups = result.components.size();
    group.regional_conflicts = result.conflict_edges.size();
    group.detail = "regional conflict group: nets=" + std::to_string(component.nets.size()) +
                   ", edges=" + std::to_string(component.conflict_edges.size());
    emit(std::move(group));
  }
  if (result.components.empty()) {
    RouterTelemetryEvent group;
    group.kind = RouterTelemetryEventKind::residual_group_updated;
    group.stable_id = "regional-summary";
    group.total_groups = 0U;
    group.regional_conflicts = result.conflict_edges.size();
    group.detail = result.diagnostic;
    emit(std::move(group));
  }

  if (diagnostic) {
    for (const auto window_id : result.joint_repair_window_ids) {
      const auto window =
          std::find_if(result.windows.begin(), result.windows.end(),
                       [&](const auto& candidate) { return candidate.id == window_id; });
      RouterTelemetryEvent repair;
      repair.kind = RouterTelemetryEventKind::residual_group_updated;
      repair.minimum_level = RouterTelemetryLevel::diagnostic;
      repair.stable_id = "regional-joint-repair:" + std::to_string(window_id);
      repair.group_id = "regional-window:" + std::to_string(window_id);
      repair.regional_conflicts =
          window == result.windows.end() ? 0U : window->conflict_edges.size();
      const auto complete =
          std::binary_search(result.joint_repair_completed_window_ids.begin(),
                             result.joint_repair_completed_window_ids.end(), window_id);
      const auto selected =
          std::find_if(result.bundle_assignments.begin(), result.bundle_assignments.end(),
                       [&](const auto& assignment) { return assignment.window_id == window_id; });
      repair.detail =
          "bounded joint topology/via/lane portfolio: expanded=yes, discrete_complete=" +
          std::string(complete ? "yes" : "no") +
          ", total_joint_checks=" + std::to_string(result.joint_repair_template_checks) +
          ", accepted_templates=" + std::to_string(result.joint_repair_accepted_templates) +
          ", spline_candidates_evaluated=" +
          std::to_string(result.joint_spline_candidates_evaluated) + ", selected_spline_rank=" +
          std::to_string(
              selected == result.bundle_assignments.end() ? 0U : selected->spline_candidate_rank);
      emit(std::move(repair));
    }
  }

  if (diagnostic) {
    for (const auto& edge : result.conflict_edges) {
      RouterTelemetryEvent conflict;
      conflict.kind = RouterTelemetryEventKind::conflict_added;
      conflict.minimum_level = RouterTelemetryLevel::diagnostic;
      conflict.stable_id = "regional-conflict:" + std::to_string(edge.id);
      conflict.resource_id = "regional-edge:" + std::to_string(edge.id);
      if (const auto component = component_by_edge.find(edge.id);
          component != component_by_edge.end()) {
        conflict.group_id = regional_group_id(component->second);
      }
      conflict.net = edge.first.net;
      conflict.other_net = edge.second.net;
      conflict.layer = edge.layer;
      conflict.x_nm = std::midpoint(edge.conflict_bounds.minimum.x.value(),
                                    edge.conflict_bounds.maximum.x.value());
      conflict.y_nm = std::midpoint(edge.conflict_bounds.minimum.y.value(),
                                    edge.conflict_bounds.maximum.y.value());
      conflict.clearance_deficit_nm = edge.peak_clearance_deficit.value();
      conflict.pressure = static_cast<double>(edge.sample_count);
      const auto single_kind = [](std::uint8_t value) {
        return value != 0U && (value & (value - 1U)) == 0U;
      };
      if (!single_kind(edge.evidence_mask)) {
        conflict.conflict_kind = "regional";
      } else if ((edge.evidence_mask & clearance_shadow_evidence) != 0U) {
        conflict.conflict_kind = "clearance_shadow";
      } else if ((edge.evidence_mask & physical_capacity_evidence) != 0U) {
        conflict.conflict_kind = "physical";
      } else {
        conflict.conflict_kind = "congestion";
      }
      conflict.detail = "overlap_nm=" + std::to_string(edge.observed_overlap_length_nm) +
                        ", discrete_excess=" + std::to_string(edge.discrete_excess) +
                        ", physical_excess_nm=" + std::to_string(edge.physical_excess_nm);
      emit(std::move(conflict));
    }
  }

  std::set<std::size_t> assigned_choices;
  for (const auto& assignment : result.bundle_assignments) {
    assigned_choices.insert(assignment.selected_via_choices.begin(),
                            assignment.selected_via_choices.end());
  }
  std::map<std::pair<NetId, NetId>, Nanometres> pairwise_clearances;
  const auto pairwise_clearance = [&](NetId first, NetId second) {
    if (second < first) {
      std::swap(first, second);
    }
    const auto key = std::pair{first, second};
    const auto found = pairwise_clearances.find(key);
    if (found != pairwise_clearances.end()) {
      return found->second;
    }
    return pairwise_clearances.emplace(key, resolve_pairwise_clearance(board, first, second))
        .first->second;
  };
  for (const auto& choice : result.via_choices) {
    const auto evidence = std::find_if(
        result.via_region_evidence.begin(), result.via_region_evidence.end(),
        [&](const auto& candidate) {
          return candidate.net == choice.net && candidate.region_id == choice.region_id;
        });
    const auto selected_global = std::find_if(
        result.via_choices.begin(), result.via_choices.end(), [&](const auto& candidate) {
          return candidate.net == choice.net && candidate.region_id == choice.region_id &&
                 candidate.span == choice.span && candidate.selected_global_site;
        });
    std::set<NetId> competing_nets;
    if (diagnostic) {
      for (const auto& other : result.via_choices) {
        if (other.net != choice.net &&
            regional_via_choices_conflict(choice, other,
                                          pairwise_clearance(choice.net, other.net))) {
          competing_nets.insert(other.net);
        }
      }
    }
    RouterTelemetryEvent candidate;
    candidate.kind = RouterTelemetryEventKind::via_candidate_generated;
    candidate.minimum_level = RouterTelemetryLevel::diagnostic;
    candidate.stable_id = regional_via_id(choice.id);
    candidate.group_id = "regional-via-region:" + std::to_string(choice.region_id);
    candidate.net = choice.net;
    candidate.x_nm = choice.site.x.value();
    candidate.y_nm = choice.site.y.value();
    if (!choice.span.empty()) {
      candidate.layer = choice.span.front();
      candidate.last_layer = choice.span.back();
    }
    candidate.diameter_nm =
        static_cast<std::uint64_t>(std::max<std::int64_t>(0, choice.diameter.value()));
    candidate.score = selected_global == result.via_choices.end()
                          ? 0.0
                          : -static_cast<double>(
                                geometry::point_distance_nm(choice.site, selected_global->site)) /
                                1'000'000.0;
    candidate.clearance_margin_nm = 0;
    candidate.competing_nets = competing_nets.size();
    candidate.local_congestion = evidence == result.via_region_evidence.end()
                                     ? 0.0
                                     : static_cast<double>(evidence->conflicting_candidate_pairs +
                                                           evidence->terminal_escape_conflicts);
    candidate.remaining_alternatives =
        evidence == result.via_region_evidence.end() || evidence->candidate_choices.empty()
            ? 0U
            : evidence->candidate_choices.size() - 1U;
    candidate.detail =
        "regional derived via candidate; clearance margin is a conservative zero lower bound";
    if (diagnostic) {
      emit(candidate);
    }
    if (assigned_choices.contains(choice.id)) {
      candidate.kind = RouterTelemetryEventKind::via_assigned;
      candidate.minimum_level = RouterTelemetryLevel::normal;
      candidate.detail = "selected by bounded regional access assignment; not committed copper";
      emit(std::move(candidate));

      const auto rules = resolve_net_routing_rules(board, choice.net);
      for (std::size_t runup_index = 0U; runup_index < choice.attached_runups.size();
           ++runup_index) {
        const auto& runup = choice.attached_runups[runup_index];
        if (runup.centerline.size() < 2U) {
          continue;
        }
        RouterTelemetryEvent route;
        route.kind = RouterTelemetryEventKind::route_candidate_selected;
        route.stable_id =
            "regional-via-runup:" + std::to_string(choice.id) + ":" + std::to_string(runup_index);
        route.group_id = "regional-via-region:" + std::to_string(choice.region_id);
        route.net = choice.net;
        route.layer = runup.layer;
        route.route_revision = 1U;
        route.route_width_nm =
            static_cast<std::uint64_t>(std::max<std::int64_t>(0, rules.track_width.value()));
        route.route_points.reserve(runup.centerline.size());
        for (const auto point : runup.centerline) {
          route.route_points.push_back(
              RouterTelemetryPathPoint{point.x.value(), point.y.value(), runup.layer});
        }
        route.detail =
            "selected regional via run-up; provisional exact-static-checked planning geometry";
        emit(std::move(route));
      }
    }
  }

  for (const auto& assignment : result.bundle_assignments) {
    for (const auto& realization : assignment.spline_realizations) {
      RouterTelemetryEvent route;
      route.kind = RouterTelemetryEventKind::route_candidate_selected;
      route.stable_id = "regional-spline:" + std::to_string(assignment.window_id) + ":" +
                        realization.fragment.net.value() + ":" +
                        std::to_string(realization.fragment.connection_index) + ":" +
                        std::to_string(realization.fragment.run_index) + ":" +
                        std::to_string(realization.fragment.fragment_index);
      route.group_id = "regional-window:" + std::to_string(assignment.window_id);
      route.net = realization.fragment.net;
      route.layer = realization.layer;
      route.route_revision = 1U;
      route.route_width_nm =
          static_cast<std::uint64_t>(std::max<std::int64_t>(0, realization.width.value()));
      route.route_geometry = RouterTelemetryRouteGeometry::cubic_bspline;
      route.spline_degree = 3U;
      route.topology_signature = realization.topology_signature;
      route.topology_gate_count = realization.topology_gate_count;
      route.topology_gate_sequence_legal = realization.topology_gate_sequence_legal;
      route.spline_control_points.reserve(realization.control_points.size());
      for (const auto point : realization.control_points) {
        route.spline_control_points.push_back(
            RouterTelemetryPathPoint{point.x.value(), point.y.value(), realization.layer});
      }
      route.route_points.reserve(realization.centerline.size());
      for (const auto point : realization.centerline) {
        route.route_points.push_back(
            RouterTelemetryPathPoint{point.x.value(), point.y.value(), realization.layer});
      }
      route.detail =
          "regional B-spline tessellation; diagnostic planning geometry; topology_legal=" +
          std::string(realization.topology_legal ? "yes" : "no") +
          ", gate_sequence=" +
          std::string(realization.topology_gate_sequence_legal ? "yes" : "no") +
          ", gate_count=" + std::to_string(realization.topology_gate_count) +
          ", mutually_legal=" + std::string(realization.mutually_legal ? "yes" : "no") +
          ", complete-board commit required";
      emit(std::move(route));
    }
  }

  for (const auto& plan : result.multilayer_route_plans) {
    const auto path = std::find_if(result.spline_candidate_paths.begin(),
                                   result.spline_candidate_paths.end(), [&](const auto& candidate) {
                                     return candidate.net == plan.net &&
                                            candidate.connection_index == plan.connection_index;
                                   });
    if (path == result.spline_candidate_paths.end()) {
      continue;
    }
    RouterTelemetryEvent route;
    route.kind = RouterTelemetryEventKind::route_candidate_selected;
    route.stable_id = "regional-multilayer-plan:" + plan.net.value() + ':' +
                      std::to_string(plan.connection_index);
    route.net = plan.net;
    route.layer = plan.layer_runs.front().layer;
    route.route_revision = 1U;
    route.via_transitions = plan.via_transitions.size();
    const auto rules = resolve_net_routing_rules(board, plan.net);
    route.route_width_nm =
        static_cast<std::uint64_t>(std::max<std::int64_t>(0, rules.track_width.value()));
    route.route_points.reserve(path->points.size());
    for (const auto& point : path->points) {
      route.route_points.push_back(RouterTelemetryPathPoint{point.position.x.value(),
                                                            point.position.y.value(), point.layer});
    }
    route.detail = "complete end-to-end multilayer candidate: layer_runs=" +
                   std::to_string(plan.layer_runs.size()) +
                   ", physical_via_transitions=" + std::to_string(plan.via_transitions.size()) +
                   "; awaiting transactional materialization/connectivity/DRC";
    emit(std::move(route));
  }

  RouterTelemetryEvent summary;
  summary.kind = RouterTelemetryEventKind::residual_group_updated;
  summary.stable_id = "regional-summary";
  summary.total_groups = result.components.size();
  summary.regional_conflicts = result.conflict_edges.size();
  const auto partial_access_pruned =
      std::accumulate(result.bundle_assignments.begin(), result.bundle_assignments.end(),
                      std::size_t{0}, [](const auto total, const auto& assignment) {
                        return total + assignment.partial_access_pruned_states;
                      });
  const auto attributed_access_blockers =
      std::accumulate(result.bundle_assignments.begin(), result.bundle_assignments.end(),
                      std::size_t{0}, [](const auto total, const auto& assignment) {
                        return total + assignment.partial_access_blockers.size();
                      });
  summary.detail =
      "regional analysis complete: windows=" + std::to_string(result.windows.size()) +
      ", complete_bundles=" +
      std::to_string(std::count_if(result.bundle_assignments.begin(),
                                   result.bundle_assignments.end(),
                                   [](const auto& assignment) { return assignment.complete(); })) +
      ", lane_cross_sections=" + std::to_string(result.selected_lane_cross_sections) +
      ", geometric_lane_reorders=" + std::to_string(result.selected_geometric_lane_reorders) +
      ", avoided_lane_inversions=" + std::to_string(result.avoided_lane_order_inversions) +
      ", lane_solver_states=" + std::to_string(result.lane_solver_states) +
      ", lane_solver_pruned=" + std::to_string(result.lane_solver_pruned_states) +
      ", lane_solver_analytical_rejections=" +
      std::to_string(result.lane_solver_analytical_rejections) +
      ", lane_solver_cache_hits=" + std::to_string(result.lane_solver_cache_hits) + "/" +
      std::to_string(result.lane_solver_queries) +
      ", lane_solver_exhaustions=" + std::to_string(result.lane_solver_budget_exhaustions) +
      ", partial_lane_capacity_pruned=" +
      std::to_string(result.partial_lane_capacity_pruned_states) +
      ", partial_access_pruned=" + std::to_string(partial_access_pruned) +
      ", attributed_access_blockers=" + std::to_string(attributed_access_blockers) +
      ", joint_repair_windows=" + std::to_string(result.joint_repair_windows_attempted) +
      ", joint_repair_completed=" + std::to_string(result.joint_repair_windows_completed) +
      ", joint_repair_checks=" + std::to_string(result.joint_repair_template_checks) +
      ", spline_portfolio_windows=" + std::to_string(result.spline_portfolio_windows) +
      ", joint_spline_candidates=" + std::to_string(result.joint_spline_candidates_evaluated) +
      ", joint_spline_recovered=" + std::to_string(result.joint_spline_windows_recovered) +
      ", joint_spline_failed_improved=" +
      std::to_string(result.joint_spline_failed_fallback_improvements) +
      ", adaptive_pair_shoves=" + std::to_string(result.spline_adaptive_pair_shove_accepted) +
      ", adaptive_pair_legalized=" + std::to_string(result.spline_adaptive_pair_windows_legalized) +
      ", residual_retry_reused_assignments=" +
      std::to_string(result.residual_conflict_cone_reused_assignments) +
      ", residual_retry_recomputed_assignments=" +
      std::to_string(result.residual_conflict_cone_recomputed_assignments) +
      ", residual_retry_withheld_assignments=" +
      std::to_string(result.residual_conflict_cone_retry_withheld_assignments) +
      ", complete_via_dependent_routes=" + std::to_string(result.complete_via_dependent_routes) +
      ", planned_layer_runs=" + std::to_string(result.planned_layer_runs) +
      ", planned_via_transitions=" + std::to_string(result.planned_via_transitions) +
      ", spline_candidate=" + std::string(result.spline_candidate_available ? "yes" : "no");
  emit(std::move(summary));
}

[[nodiscard]] bool coarse_topology_complete(const GlobalRouteResult& global) {
  return global.unrouted_nets.empty() && global.unaccepted_selected_voronoi_lane_transitions == 0U;
}

} // namespace

std::optional<RegionalMultilayerRoutePlan> plan_multilayer_route(const Board& board,
                                                                 const GlobalRoutePath& path) {
  if (path.points.size() < 2U) {
    return std::nullopt;
  }

  RegionalMultilayerRoutePlan result;
  result.net = path.net;
  result.connection_index = path.connection_index;
  result.layer_runs.push_back(
      RegionalPlannedLayerRun{path.points.front().layer, {path.points.front().position}});

  bool found_transition = false;
  std::size_t index = 1U;
  while (index < path.points.size()) {
    const auto& previous = path.points[index - 1U];
    const auto& current = path.points[index];
    if (previous.layer == current.layer) {
      if (result.layer_runs.empty() || result.layer_runs.back().layer != current.layer) {
        return std::nullopt;
      }
      if (result.layer_runs.back().centerline.back() != current.position) {
        result.layer_runs.back().centerline.push_back(current.position);
      }
      ++index;
      continue;
    }
    if (previous.position != current.position || result.layer_runs.empty() ||
        result.layer_runs.back().layer != previous.layer) {
      return std::nullopt;
    }

    found_transition = true;
    const auto site = previous.position;
    const auto first_layer = previous.layer;
    auto last_layer = current.layer;
    // A KiCad via may span several intervening copper layers. Collapse an uninterrupted stack of
    // same-site layer points into one physical transition, but never collapse transitions that
    // have a planar run between them.
    while (index + 1U < path.points.size() && path.points[index + 1U].position == site) {
      ++index;
      last_layer = path.points[index].layer;
    }
    if (first_layer == last_layer) {
      return std::nullopt;
    }
    auto span = regional_via_span(board, first_layer, last_layer);
    if (span.size() < 2U || std::find(span.begin(), span.end(), first_layer) == span.end() ||
        std::find(span.begin(), span.end(), last_layer) == span.end()) {
      return std::nullopt;
    }
    result.via_transitions.push_back(RegionalPlannedViaTransition{site, std::move(span)});
    result.layer_runs.push_back(RegionalPlannedLayerRun{last_layer, {site}});
    ++index;
  }

  if (!found_transition || result.layer_runs.size() != result.via_transitions.size() + 1U ||
      std::any_of(result.layer_runs.begin(), result.layer_runs.end(),
                  [](const auto& run) { return run.layer.empty() || run.centerline.empty(); })) {
    return std::nullopt;
  }
  return result;
}

RegionalRouteResult analyze_regional_conflicts(const Board& board, const GlobalRouteResult& global,
                                               const RegionalRouterConfig& config,
                                               std::stop_token stop_token,
                                               RouterTelemetrySink* telemetry) {
  RegionalRouteResult result;
  result.configured = config;
  result.source_clearance_shadow_conflicts = global.clearance_shadow_conflicts.size();
  const auto emit_progress = [telemetry](std::string stable_id, std::string detail) {
    if (telemetry == nullptr) {
      return;
    }
    RouterTelemetryEvent progress;
    progress.kind = RouterTelemetryEventKind::search_summary;
    progress.stable_id = std::move(stable_id);
    progress.detail = std::move(detail);
    static_cast<void>(telemetry->try_emit(std::move(progress)));
  };
  const auto spline_count = [](const RegionalRouteResult& candidate) {
    return std::accumulate(candidate.bundle_assignments.begin(), candidate.bundle_assignments.end(),
                           std::size_t{0U}, [](const auto count, const auto& assignment) {
                             return count + assignment.spline_realizations.size();
                           });
  };
  if (!valid_config(config)) {
    result.diagnostic = "regional routing requires positive bounded search configuration";
    return result;
  }
  if (stop_token.stop_requested()) {
    result.stop_reason = RegionalRouterStopReason::cancelled;
    result.diagnostic = "regional conflict analysis cancelled before fragment construction";
    return result;
  }
  emit_progress("regional-analysis:start",
                "regional conflict extraction and joint physical packing started");
  result.fragments = make_fragments(board, global);
  result.refittable_terminal_width_fragments = static_cast<std::size_t>(
      std::count_if(result.fragments.begin(), result.fragments.end(), [](const auto& fragment) {
        return fragment.explicit_width &&
               fragment.width_kind == GlobalRouteSegmentWidthKind::terminal_neck_down;
      }));
  result.fixed_resource_width_fragments = static_cast<std::size_t>(
      std::count_if(result.fragments.begin(), result.fragments.end(), [](const auto& fragment) {
        return fragment.explicit_width &&
               fragment.width_kind != GlobalRouteSegmentWidthKind::terminal_neck_down;
      }));
  emit_progress("regional-analysis:fragments", "regional fragments constructed: fragments=" +
                                                   std::to_string(result.fragments.size()));

  std::vector<ConflictSample> samples;
  samples.reserve(global.clearance_shadow_conflicts.size());
  for (const auto& conflict : global.clearance_shadow_conflicts) {
    if (stop_token.stop_requested()) {
      result.stop_reason = RegionalRouterStopReason::cancelled;
      result.diagnostic = "regional conflict analysis cancelled during shadow attribution";
      return result;
    }
    const ConflictReference candidate{conflict.first.position, conflict.second.position,
                                      conflict.first.layer};
    const auto owner_start = conflict.shadow_owner_first ? conflict.shadow_owner_first->position
                                                         : conflict.first.position;
    const auto owner_end =
        conflict.shadow_owner_second ? conflict.shadow_owner_second->position : owner_start;
    const ConflictReference owner{owner_start, owner_end, conflict.first.layer};
    const auto first = nearest_fragment(result.fragments, conflict.first_net, candidate);
    const auto second = nearest_fragment(result.fragments, conflict.second_net, owner);
    if (!first || !second || result.fragments[*first].id == result.fragments[*second].id) {
      ++result.unmapped_clearance_shadow_conflicts;
      continue;
    }
    auto first_id = result.fragments[*first].id;
    auto second_id = result.fragments[*second].id;
    if (second_id < first_id) {
      std::swap(first_id, second_id);
    }
    samples.push_back(ConflictSample{first_id,
                                     second_id,
                                     conflict.first.layer,
                                     union_bounds(segment_bounds(candidate.start, candidate.end),
                                                  segment_bounds(owner.start, owner.end)),
                                     conflict.clearance_deficit,
                                     observed_length(candidate, owner),
                                     clearance_shadow_evidence,
                                     0U,
                                     0U,
                                     {}});
  }
  const auto append_capacity_samples = [&](const std::vector<NetId>& nets,
                                           const ConflictReference& reference,
                                           std::uint8_t evidence_mask, std::size_t discrete_excess,
                                           std::uint64_t physical_excess_nm,
                                           const std::vector<ObjectId>& obstacles) {
    for (std::size_t first_net = 0U; first_net < nets.size(); ++first_net) {
      for (std::size_t second_net = first_net + 1U; second_net < nets.size(); ++second_net) {
        const auto first = nearest_fragment(result.fragments, nets[first_net], reference);
        const auto second = nearest_fragment(result.fragments, nets[second_net], reference);
        if (!first || !second || result.fragments[*first].id == result.fragments[*second].id) {
          ++result.unmapped_capacity_conflicts;
          continue;
        }
        auto first_id = result.fragments[*first].id;
        auto second_id = result.fragments[*second].id;
        if (second_id < first_id) {
          std::swap(first_id, second_id);
        }
        samples.push_back(ConflictSample{
            first_id, second_id, reference.layer, segment_bounds(reference.start, reference.end),
            Nanometres{0},
            std::max(std::uint64_t{1}, geometry::segment_length_nm(reference.start, reference.end)),
            evidence_mask, discrete_excess, physical_excess_nm, obstacles});
      }
    }
  };
  for (const auto& resource : global.resource_usage) {
    if (!resource.congested()) {
      continue;
    }
    ++result.source_sampled_capacity_conflicts;
    append_capacity_samples(
        resource.nets,
        ConflictReference{resource.first.position, resource.second.position, resource.first.layer},
        sampled_capacity_evidence, resource.demand - resource.capacity, 0U, {});
  }
  for (const auto& corridor : global.corridor_usage) {
    if (!corridor.congested()) {
      continue;
    }
    ++result.source_corridor_capacity_conflicts;
    append_capacity_samples(corridor.nets,
                            ConflictReference{corridor.sampled_medial_start,
                                              corridor.sampled_medial_end, corridor.layer},
                            corridor_capacity_evidence, corridor.demand - corridor.capacity, 0U,
                            {});
  }
  for (const auto& resource : global.voronoi_resource_usage) {
    if (!resource.congested()) {
      continue;
    }
    ++result.source_physical_capacity_conflicts;
    std::vector<ObjectId> obstacles;
    for (const auto& impact : global.physical_obstacle_impacts) {
      if (impact.resource_id == resource.id) {
        obstacles.push_back(impact.obstacle);
      }
    }
    std::sort(obstacles.begin(), obstacles.end());
    obstacles.erase(std::unique(obstacles.begin(), obstacles.end()), obstacles.end());
    append_capacity_samples(
        resource.nets, ConflictReference{resource.start, resource.end, resource.layer},
        physical_capacity_evidence, 0U, resource.demand_nm - resource.capacity_nm, obstacles);
  }
  std::sort(samples.begin(), samples.end(), [](const auto& lhs, const auto& rhs) {
    return std::tie(lhs.first, lhs.second, lhs.layer, lhs.bounds.minimum, lhs.bounds.maximum,
                    lhs.deficit, lhs.observed_length_nm) <
           std::tie(rhs.first, rhs.second, rhs.layer, rhs.bounds.minimum, rhs.bounds.maximum,
                    rhs.deficit, rhs.observed_length_nm);
  });

  for (const auto& sample : samples) {
    auto merge = std::find_if(
        result.conflict_edges.rbegin(), result.conflict_edges.rend(), [&sample](const auto& edge) {
          return edge.first == sample.first && edge.second == sample.second &&
                 edge.layer == sample.layer && bounds_touch(edge.conflict_bounds, sample.bounds);
        });
    if (merge == result.conflict_edges.rend()) {
      result.conflict_edges.push_back(RegionalConflictEdge{
          0U, sample.first, sample.second, sample.layer, sample.bounds, sample.deficit,
          sample.observed_length_nm, 1U,
          sample.evidence_mask == clearance_shadow_evidence ? 1U : 0U,
          sample.evidence_mask == clearance_shadow_evidence ? 0U : 1U, sample.evidence_mask,
          sample.discrete_excess, sample.physical_excess_nm, sample.attributed_obstacles});
      continue;
    }
    merge->conflict_bounds = union_bounds(merge->conflict_bounds, sample.bounds);
    merge->peak_clearance_deficit = std::max(merge->peak_clearance_deficit, sample.deficit);
    merge->observed_overlap_length_nm =
        merge->observed_overlap_length_nm >
                std::numeric_limits<std::uint64_t>::max() - sample.observed_length_nm
            ? std::numeric_limits<std::uint64_t>::max()
            : merge->observed_overlap_length_nm + sample.observed_length_nm;
    ++merge->sample_count;
    if (sample.evidence_mask == clearance_shadow_evidence) {
      ++merge->clearance_shadow_sample_count;
    } else {
      ++merge->capacity_sample_count;
    }
    merge->evidence_mask = static_cast<std::uint8_t>(merge->evidence_mask | sample.evidence_mask);
    merge->discrete_excess =
        merge->discrete_excess > std::numeric_limits<std::size_t>::max() - sample.discrete_excess
            ? std::numeric_limits<std::size_t>::max()
            : merge->discrete_excess + sample.discrete_excess;
    merge->physical_excess_nm =
        merge->physical_excess_nm >
                std::numeric_limits<std::uint64_t>::max() - sample.physical_excess_nm
            ? std::numeric_limits<std::uint64_t>::max()
            : merge->physical_excess_nm + sample.physical_excess_nm;
    merge->attributed_obstacles.insert(merge->attributed_obstacles.end(),
                                       sample.attributed_obstacles.begin(),
                                       sample.attributed_obstacles.end());
    std::sort(merge->attributed_obstacles.begin(), merge->attributed_obstacles.end());
    merge->attributed_obstacles.erase(
        std::unique(merge->attributed_obstacles.begin(), merge->attributed_obstacles.end()),
        merge->attributed_obstacles.end());
  }
  std::sort(result.conflict_edges.begin(), result.conflict_edges.end(),
            [](const auto& lhs, const auto& rhs) {
              return std::tie(lhs.first, lhs.second, lhs.layer, lhs.conflict_bounds.minimum,
                              lhs.conflict_bounds.maximum) <
                     std::tie(rhs.first, rhs.second, rhs.layer, rhs.conflict_bounds.minimum,
                              rhs.conflict_bounds.maximum);
            });
  for (std::size_t index = 0U; index < result.conflict_edges.size(); ++index) {
    auto& edge = result.conflict_edges[index];
    edge.id = index;
    result.coalesced_clearance_shadow_samples += edge.clearance_shadow_sample_count;
    result.total_observed_overlap_length_nm =
        result.total_observed_overlap_length_nm >
                std::numeric_limits<std::uint64_t>::max() - edge.observed_overlap_length_nm
            ? std::numeric_limits<std::uint64_t>::max()
            : result.total_observed_overlap_length_nm + edge.observed_overlap_length_nm;
    result.peak_clearance_deficit =
        std::max(result.peak_clearance_deficit, edge.peak_clearance_deficit);
  }
  populate_components(result);
  emit_progress(
      "regional-analysis:conflicts",
      "regional conflict graph constructed: edges=" + std::to_string(result.conflict_edges.size()) +
          ", components=" + std::to_string(result.components.size()));
  const auto solve_windows = [&](RegionalRouteResult& candidate, std::string_view phase) {
    populate_blocks(candidate);
    populate_windows(board, candidate);
    emit_progress("regional-analysis:" + std::string(phase) + ":windows",
                  std::string(phase) + " regional windows constructed: windows=" +
                      std::to_string(candidate.windows.size()));
    populate_channels(board, global, candidate);
    emit_progress("regional-analysis:" + std::string(phase) + ":channels",
                  std::string(phase) + " regional corridors constructed: channels=" +
                      std::to_string(candidate.channels.size()));
    populate_bundle_assignments(board, global, candidate, telemetry, phase);
    emit_progress(
        "regional-analysis:" + std::string(phase) + ":assignments",
        std::string(phase) + " regional bundle portfolio prepared: assignments=" +
            std::to_string(candidate.bundle_assignments.size()) +
            ", alternative_templates=" + std::to_string(candidate.accepted_alternative_templates));
    realize_bundle_splines(candidate, stop_token, telemetry);
    emit_progress("regional-analysis:" + std::string(phase) + ":splines",
                  std::string(phase) + " regional spline realization completed: splines=" +
                      std::to_string(spline_count(candidate)));
    build_regional_spline_candidate(board, global, candidate);
  };
  const auto unresolved_objective = [](const RegionalRouteResult& candidate) {
    std::size_t unresolved = 0U;
    std::size_t discrete = 0U;
    std::size_t topology = 0U;
    std::size_t mutual = 0U;
    for (const auto& assignment : candidate.bundle_assignments) {
      if (!assignment.complete()) {
        ++unresolved;
      }
      discrete +=
          assignment.stop_reason == RegionalBundleStopReason::no_fitting_lane_order ||
                  assignment.stop_reason == RegionalBundleStopReason::missing_corridor_topology ||
                  assignment.stop_reason == RegionalBundleStopReason::state_budget_exhausted
              ? 1U
              : 0U;
      for (const auto& realization : assignment.spline_realizations) {
        topology += !realization.topology_legal ? 1U : 0U;
        mutual += !realization.mutually_legal ? 1U : 0U;
      }
    }
    return std::tuple{candidate.spline_candidate_available ? 0U : 1U,
                      unresolved,
                      discrete,
                      topology + mutual,
                      topology,
                      mutual};
  };

  auto ordinary = result;
  solve_windows(ordinary, "ordinary");
  result = std::move(ordinary);
  if (!result.spline_candidate_available && !stop_token.stop_requested()) {
    auto retry = RegionalRouteResult{};
    retry.configured = config;
    retry.source_clearance_shadow_conflicts = result.source_clearance_shadow_conflicts;
    retry.coalesced_clearance_shadow_samples = result.coalesced_clearance_shadow_samples;
    retry.unmapped_clearance_shadow_conflicts = result.unmapped_clearance_shadow_conflicts;
    retry.source_sampled_capacity_conflicts = result.source_sampled_capacity_conflicts;
    retry.source_corridor_capacity_conflicts = result.source_corridor_capacity_conflicts;
    retry.source_physical_capacity_conflicts = result.source_physical_capacity_conflicts;
    retry.unmapped_capacity_conflicts = result.unmapped_capacity_conflicts;
    retry.fragments = result.fragments;
    retry.conflict_edges = result.conflict_edges;
    retry.components = result.components;
    retry.refittable_terminal_width_fragments = result.refittable_terminal_width_fragments;
    retry.fixed_resource_width_fragments = result.fixed_resource_width_fragments;
    retry.total_observed_overlap_length_nm = result.total_observed_overlap_length_nm;
    retry.peak_clearance_deficit = result.peak_clearance_deficit;
    populate_blocks(retry);
    populate_windows(board, retry);
    std::set<std::size_t> replaced_window_ids;
    const auto residual_cones_built =
        build_residual_conflict_cone_windows(retry, result, replaced_window_ids);
    if (residual_cones_built &&
        retry.windows.size() <= config.max_residual_retry_bundle_assignments) {
      ++retry.residual_conflict_cone_retries;
      emit_progress("regional-analysis:residual-cones",
                    "residual conflict cones constructed: cones=" +
                        std::to_string(retry.residual_conflict_cones_built) +
                        ", windows=" + std::to_string(retry.windows.size()));
      populate_channels(board, global, retry);
      emit_progress("regional-analysis:residual-channels",
                    "residual cone corridors constructed: channels=" +
                        std::to_string(retry.channels.size()));
      populate_bundle_assignments(board, global, retry, telemetry, "residual");
      emit_progress("regional-analysis:residual-assignments",
                    "residual cone portfolio prepared: assignments=" +
                        std::to_string(retry.bundle_assignments.size()));
      realize_bundle_splines(retry, stop_token, telemetry);
      emit_progress("regional-analysis:residual-splines",
                    "residual cone spline realization completed: splines=" +
                        std::to_string(spline_count(retry)));
      const auto cones_built = retry.residual_conflict_cones_built;
      const auto windows_merged = retry.residual_conflict_cone_windows_merged;
      const auto recomputed = retry.bundle_assignments.size();
      const auto retained_assignments = static_cast<std::size_t>(
          std::count_if(result.bundle_assignments.begin(), result.bundle_assignments.end(),
                        [&](const auto& assignment) {
                          return !replaced_window_ids.contains(assignment.window_id);
                        }));
      const auto retry_lane_queries = retry.lane_solver_queries;
      const auto retry_lane_cache_hits = retry.lane_solver_cache_hits;
      const auto retry_lane_states = retry.lane_solver_states;
      const auto retry_lane_pruned_states = retry.lane_solver_pruned_states;
      const auto retry_lane_analytical_rejections = retry.lane_solver_analytical_rejections;
      const auto retry_lane_budget_exhaustions = retry.lane_solver_budget_exhaustions;
      const auto retry_partial_lane_capacity_pruned = retry.partial_lane_capacity_pruned_states;
      const auto retain_retry_lane_metrics = [&]() {
        result.lane_solver_queries += retry_lane_queries;
        result.lane_solver_cache_hits += retry_lane_cache_hits;
        result.lane_solver_states += retry_lane_states;
        result.lane_solver_pruned_states += retry_lane_pruned_states;
        result.lane_solver_analytical_rejections += retry_lane_analytical_rejections;
        result.lane_solver_budget_exhaustions += retry_lane_budget_exhaustions;
        result.partial_lane_capacity_pruned_states += retry_partial_lane_capacity_pruned;
      };
      const auto complete_cone_transaction =
          !retry.bundle_assignments.empty() &&
          std::all_of(retry.bundle_assignments.begin(), retry.bundle_assignments.end(),
                      [](const auto& assignment) { return assignment.complete(); });
      if (complete_cone_transaction) {
        auto combined =
            merge_residual_cone_checkpoint(result, std::move(retry), replaced_window_ids);
        build_regional_spline_candidate(board, global, combined);
        // Retained windows are immutable. Only a complete recomputed cone can replace its member
        // assignments, and the complete-board fragment/via reconstruction must still improve the
        // feasibility-first checkpoint. Exact detailed copper and DRC remain downstream gates.
        if (unresolved_objective(combined) < unresolved_objective(result)) {
          combined.residual_conflict_cone_selected = true;
          result = std::move(combined);
        } else {
          retain_retry_lane_metrics();
          result.residual_conflict_cone_retries = 1U;
          result.residual_conflict_cones_built = cones_built;
          result.residual_conflict_cone_windows_merged = windows_merged;
          result.residual_conflict_cone_recomputed_assignments = recomputed;
          result.residual_conflict_cone_reused_assignments = retained_assignments;
        }
      } else {
        retain_retry_lane_metrics();
        result.residual_conflict_cone_retries = 1U;
        result.residual_conflict_cones_built = cones_built;
        result.residual_conflict_cone_windows_merged = windows_merged;
        result.residual_conflict_cone_recomputed_assignments = recomputed;
        result.residual_conflict_cone_reused_assignments = retained_assignments;
      }
    } else if (residual_cones_built && !retry.windows.empty() &&
               retry.windows.size() > config.max_residual_retry_bundle_assignments) {
      result.residual_conflict_cone_retry_withheld_assignments = retry.windows.size();
    }
  }

  if (result.conflict_edges.empty() && result.unmapped_clearance_shadow_conflicts == 0U &&
      result.unmapped_capacity_conflicts == 0U) {
    result.stop_reason = RegionalRouterStopReason::no_conflicts;
    result.diagnostic = "selected global paths contain no regional physical conflicts";
  } else {
    result.stop_reason = RegionalRouterStopReason::conflicts_detected;
    result.diagnostic = result.unmapped_clearance_shadow_conflicts == 0U
                            ? "selected global paths require regional physical packing"
                            : "selected global paths contain unmapped regional conflict evidence";
  }
  publish_regional_telemetry(board, result, telemetry);
  return result;
}

std::vector<RegionalRouteFragmentId>
geometric_regional_cross_section_order(const std::vector<RegionalRouteFragment>& fragments,
                                       const std::vector<RegionalRouteFragmentId>& occupants,
                                       Point first_wall, Point second_wall) {
  if (second_wall < first_wall) {
    std::swap(first_wall, second_wall);
  }
  const auto wall_dx = static_cast<long double>(second_wall.x.value() - first_wall.x.value());
  const auto wall_dy = static_cast<long double>(second_wall.y.value() - first_wall.y.value());
  const auto wall_length_squared = wall_dx * wall_dx + wall_dy * wall_dy;
  if (!(wall_length_squared > 0.0L)) {
    auto ordered = occupants;
    std::sort(ordered.begin(), ordered.end());
    return ordered;
  }
  const auto coordinate = [&](const RegionalRouteFragmentId& id) {
    const auto fragment = std::find_if(fragments.begin(), fragments.end(),
                                       [&](const auto& candidate) { return candidate.id == id; });
    if (fragment == fragments.end()) {
      return std::numeric_limits<long double>::infinity();
    }
    const auto fragment_dx =
        static_cast<long double>(fragment->end.x.value() - fragment->start.x.value());
    const auto fragment_dy =
        static_cast<long double>(fragment->end.y.value() - fragment->start.y.value());
    const auto offset_x =
        static_cast<long double>(fragment->start.x.value() - first_wall.x.value());
    const auto offset_y =
        static_cast<long double>(fragment->start.y.value() - first_wall.y.value());
    const auto denominator = wall_dx * fragment_dy - wall_dy * fragment_dx;
    if (denominator != 0.0L &&
        geometry::segments_intersect(first_wall, second_wall, fragment->start, fragment->end)) {
      return (offset_x * fragment_dy - offset_y * fragment_dx) / denominator;
    }
    const auto midpoint_x =
        (static_cast<long double>(fragment->start.x.value()) + fragment->end.x.value()) / 2.0L;
    const auto midpoint_y =
        (static_cast<long double>(fragment->start.y.value()) + fragment->end.y.value()) / 2.0L;
    return ((midpoint_x - static_cast<long double>(first_wall.x.value())) * wall_dx +
            (midpoint_y - static_cast<long double>(first_wall.y.value())) * wall_dy) /
           wall_length_squared;
  };
  struct LocatedOccupant {
    RegionalRouteFragmentId id;
    long double coordinate{0.0L};
  };
  std::vector<LocatedOccupant> located;
  located.reserve(occupants.size());
  for (const auto& id : occupants) {
    located.push_back(LocatedOccupant{id, coordinate(id)});
  }
  std::sort(located.begin(), located.end(), [](const auto& first, const auto& second) {
    if (first.coordinate != second.coordinate) {
      return first.coordinate < second.coordinate;
    }
    return first.id < second.id;
  });
  std::vector<RegionalRouteFragmentId> ordered;
  ordered.reserve(located.size());
  std::transform(located.begin(), located.end(), std::back_inserter(ordered),
                 [](const auto& occupant) { return occupant.id; });
  return ordered;
}

RegionalCrossSection
pack_regional_cross_section(const std::vector<RegionalRouteFragment>& fragments,
                            const std::vector<RegionalRouteFragmentId>& order, LayerId layer,
                            Point first_wall, Point second_wall, Nanometres usable_width) {
  // Corridors inherited from oppositely directed source routes describe the same physical cross
  // section with reversed wall endpoints. Canonicalize that axis before assigning an ordered lane
  // vector so the same occupant cannot silently swap sides between adjacent channel evidence.
  if (second_wall < first_wall) {
    std::swap(first_wall, second_wall);
  }
  RegionalCrossSection result;
  result.layer = std::move(layer);
  result.first_wall = first_wall;
  result.second_wall = second_wall;
  result.usable_width = usable_width;
  result.ordered_occupants = order;
  std::vector<const RegionalRouteFragment*> selected;
  selected.reserve(order.size());
  for (const auto& id : order) {
    const auto found = std::find_if(fragments.begin(), fragments.end(),
                                    [&id](const auto& fragment) { return fragment.id == id; });
    if (found == fragments.end() || found->width.value() <= 0 || found->clearance.value() < 0) {
      return result;
    }
    selected.push_back(&*found);
  }
  if (selected.empty() || usable_width.value() <= 0) {
    return result;
  }
  std::int64_t packed = selected.front()->clearance.value();
  for (std::size_t index = 0U; index < selected.size(); ++index) {
    const auto width = selected[index]->width.value();
    if (packed > std::numeric_limits<std::int64_t>::max() - width) {
      return result;
    }
    packed += width;
    if (index + 1U < selected.size()) {
      const auto gap =
          std::max(selected[index]->clearance, selected[index + 1U]->clearance).value();
      if (packed > std::numeric_limits<std::int64_t>::max() - gap) {
        return result;
      }
      packed += gap;
    }
  }
  if (packed > std::numeric_limits<std::int64_t>::max() - selected.back()->clearance.value()) {
    return result;
  }
  packed += selected.back()->clearance.value();
  result.packed_width = Nanometres{packed};
  if (packed > usable_width.value()) {
    return result;
  }
  result.physically_fits = true;
  auto cursor = -(packed / 2) + selected.front()->clearance.value();
  for (std::size_t index = 0U; index < selected.size(); ++index) {
    cursor += selected[index]->width.value() / 2;
    result.placements.push_back(RegionalLanePlacement{order[index], cursor});
    cursor += selected[index]->width.value() - selected[index]->width.value() / 2;
    if (index + 1U < selected.size()) {
      cursor += std::max(selected[index]->clearance, selected[index + 1U]->clearance).value();
    }
  }
  return result;
}

void partition_regional_conflicts(const Board& board, RegionalRouteResult& regional,
                                  std::stop_token stop_token) {
  if (stop_token.stop_requested()) {
    regional.stop_reason = RegionalRouterStopReason::cancelled;
    regional.diagnostic = "regional conflict partitioning cancelled";
    return;
  }
  populate_blocks(regional);
  populate_windows(board, regional);
}

RoutingPlanResult plan_routes(const Board& board, const RoutingPlannerConfig& config,
                              std::stop_token stop_token, RouterTelemetrySink* telemetry) {
  RoutabilityKnowledge knowledge;
  knowledge.history = config.global.routability_history;
  std::optional<RoutingPlanResult> best;
  std::optional<GlobalRouteResult> best_global;
  std::size_t global_feedback_passes = 0U;
  std::size_t regional_feedback_passes = 0U;

  auto regional_config = config.regional;
  regional_config.treat_saved_zone_fills_as_refillable =
      config.global.treat_saved_zone_fills_as_refillable;
  const auto classify = [](RoutingPlanResult& candidate) {
    if (candidate.regional.stop_reason == RegionalRouterStopReason::invalid_input) {
      candidate.stop_reason = RoutingPlanStopReason::invalid_input;
      candidate.diagnostic = candidate.regional.diagnostic;
    } else if (!coarse_topology_complete(candidate.coarse_global)) {
      candidate.stop_reason = RoutingPlanStopReason::coarse_global_incomplete;
      candidate.diagnostic = "coarse global topology remains incomplete";
    } else if (!candidate.regional.complete()) {
      candidate.stop_reason = RoutingPlanStopReason::regional_conflicts;
      candidate.diagnostic = candidate.regional.diagnostic;
    } else if (!candidate.coarse_global.complete() &&
               !candidate.regional.spline_candidate_available) {
      candidate.stop_reason = RoutingPlanStopReason::coarse_global_incomplete;
      candidate.diagnostic =
          "coarse global overflow was not discharged by a complete regional candidate";
    } else {
      candidate.stop_reason = RoutingPlanStopReason::complete;
      candidate.diagnostic =
          candidate.coarse_global.complete()
              ? "coarse global routing and regional physical-conflict analysis completed"
              : "connected coarse topology was legalized by a complete regional spline candidate";
    }
  };
  const auto better = [](const RoutingPlanResult& lhs, const RoutingPlanResult& rhs) {
    const auto incomplete_assignments = [](const RoutingPlanResult& plan) {
      return static_cast<std::size_t>(std::count_if(
          plan.regional.bundle_assignments.begin(), plan.regional.bundle_assignments.end(),
          [](const auto& assignment) { return !assignment.complete(); }));
    };
    return std::tuple{lhs.complete() ? 0U : 1U, lhs.coarse_global.unrouted_nets.size(),
                      lhs.coarse_global.total_overflow, incomplete_assignments(lhs),
                      lhs.coarse_global.total_cost} <
           std::tuple{rhs.complete() ? 0U : 1U, rhs.coarse_global.unrouted_nets.size(),
                      rhs.coarse_global.total_overflow, incomplete_assignments(rhs),
                      rhs.coarse_global.total_cost};
  };
  const auto better_global = [](const GlobalRouteResult& lhs, const GlobalRouteResult& rhs) {
    return std::tuple{lhs.unrouted_nets.size(), lhs.total_overflow, lhs.total_cost} <
           std::tuple{rhs.unrouted_nets.size(), rhs.total_overflow, rhs.total_cost};
  };
  const auto publish_selected_global = [&](const GlobalRouteResult& selected,
                                           const GlobalRouteResult& discarded_probe) {
    if (telemetry == nullptr) {
      return;
    }
    std::set<std::string> selected_route_ids;
    for (const auto& path : selected.paths) {
      selected_route_ids.insert("global-route:" + path.net.value() + ':' +
                                std::to_string(path.connection_index));
    }
    for (const auto& path : discarded_probe.paths) {
      const auto id =
          "global-route:" + path.net.value() + ':' + std::to_string(path.connection_index);
      if (!selected_route_ids.contains(id)) {
        RouterTelemetryEvent removed;
        removed.kind = RouterTelemetryEventKind::route_removed;
        removed.stable_id = id;
        removed.net = path.net;
        removed.detail = "discarded global routability probe route removed";
        static_cast<void>(telemetry->try_emit(std::move(removed)));
      }
    }
    std::set<NetId> routed_nets;
    for (const auto& path : selected.paths) {
      routed_nets.insert(path.net);
      RouterTelemetryEvent route;
      route.kind = RouterTelemetryEventKind::route_candidate_selected;
      route.stable_id =
          "global-route:" + path.net.value() + ':' + std::to_string(path.connection_index);
      route.net = path.net;
      route.route_revision = selected.selected_iteration;
      const auto rules = resolve_net_routing_rules(board, path.net);
      route.route_width_nm =
          static_cast<std::uint64_t>(std::max<std::int64_t>(0, rules.track_width.value()));
      route.route_points.reserve(path.points.size());
      for (const auto& point : path.points) {
        route.route_points.push_back(RouterTelemetryPathPoint{
            point.position.x.value(), point.position.y.value(), point.layer});
      }
      route.detail = "feasibility-first global checkpoint restored for regional routing";
      static_cast<void>(telemetry->try_emit(std::move(route)));
    }
    RouterTelemetryEvent checkpoint;
    checkpoint.kind = RouterTelemetryEventKind::checkpoint_selected;
    checkpoint.iteration = selected.selected_iteration;
    auto all_nets = routed_nets;
    all_nets.insert(selected.unrouted_nets.begin(), selected.unrouted_nets.end());
    checkpoint.total_nets = all_nets.size();
    checkpoint.routed_nets = all_nets.size() >= selected.unrouted_nets.size()
                                 ? all_nets.size() - selected.unrouted_nets.size()
                                 : 0U;
    checkpoint.unrouted_nets = selected.unrouted_nets.size();
    checkpoint.sampled_overflow = selected.sampled_overflow;
    checkpoint.corridor_overflow = selected.corridor_overflow;
    checkpoint.physical_overflow_nm = selected.total_voronoi_overflow_nm;
    checkpoint.clearance_shadow_conflicts = selected.clearance_shadow_conflicts.size();
    checkpoint.route_cost = selected.total_cost;
    checkpoint.detail =
        "feasibility-first global checkpoint selected after routability feedback probe";
    static_cast<void>(telemetry->try_emit(std::move(checkpoint)));
  };

  for (std::size_t pass = 0U; pass <= config.max_routability_feedback_retries; ++pass) {
    auto global_config = config.global;
    global_config.routability_history = knowledge.history;
    RoutingPlanResult candidate;
    auto probed_global = route_global(board, global_config, stop_token, telemetry);
    candidate.cheap_physical_probes = pass + 1U;
    candidate.global_feedback_passes = global_feedback_passes;
    candidate.regional_feedback_passes = regional_feedback_passes;
    if (stop_token.stop_requested() ||
        probed_global.stop_reason == GlobalRouterStopReason::cancelled) {
      candidate.stop_reason = RoutingPlanStopReason::cancelled;
      candidate.diagnostic = "routing planner cancelled during coarse global routing";
      candidate.routability_knowledge = std::move(knowledge);
      return candidate;
    }

    const auto global_diagnoses = probe_global_routability(probed_global, config.routability_probe);
    accumulate_routability_knowledge(knowledge, global_diagnoses, config.routability_probe);
    const bool retained_prior_global =
        best_global.has_value() && !better_global(probed_global, *best_global);
    if (!retained_prior_global) {
      best_global = std::move(probed_global);
    }
    const bool retry_available = pass < config.max_routability_feedback_retries;
    // Spend at most one retry on the cheap global probe before consulting the regional solver;
    // later physical failures retain an upstream retry instead of starving the nested loop. The
    // physical-pressure retry may improve congestion while temporarily losing nets, so hand the
    // feasibility-first retained global checkpoint to regional routing rather than the last probe.
    if (!global_diagnoses.empty() && retry_available && global_feedback_passes == 0U) {
      ++global_feedback_passes;
      continue;
    }

    candidate.coarse_global = std::move(*best_global);
    best_global.reset();
    candidate.paths = candidate.coarse_global.paths;
    if (retained_prior_global) {
      publish_selected_global(candidate.coarse_global, probed_global);
    }
    if (retained_prior_global && best) {
      // A regional retry changes only upstream PathFinder history. If that probe loses the coarse
      // feasibility comparison, the restored GlobalRouteResult is the exact checkpoint whose
      // regional result is already archived in `best`; repeating deterministic packing and
      // spline/PNS realization cannot add information.
      best->routability_knowledge = knowledge;
      best->global_feedback_passes = global_feedback_passes;
      best->regional_feedback_passes = regional_feedback_passes;
      best->cheap_physical_probes = pass + 1U;
      best->routability_feedback_budget_exhausted = true;
      return *best;
    }

    candidate.regional = analyze_regional_conflicts(board, candidate.coarse_global, regional_config,
                                                    stop_token, telemetry);
    if (candidate.regional.spline_candidate_available) {
      candidate.paths = candidate.regional.spline_candidate_paths;
    }
    const auto regional_diagnoses =
        diagnose_regional_routability(candidate.regional, config.routability_probe);
    accumulate_routability_knowledge(knowledge, regional_diagnoses, config.routability_probe);
    candidate.routability_knowledge = knowledge;
    candidate.global_feedback_passes = global_feedback_passes;
    candidate.regional_feedback_passes = regional_feedback_passes;
    classify(candidate);
    if (!best || better(candidate, *best)) {
      best = candidate;
    }
    if (candidate.complete()) {
      return candidate;
    }
    if ((!regional_diagnoses.empty() || !global_diagnoses.empty()) && retry_available) {
      ++regional_feedback_passes;
      // Preserve the best topology while the next history-informed global probe is evaluated.
      // This is routing state, not telemetry state, and keeps the same feasibility-first order as
      // the final plan comparator.
      best_global = best->coarse_global;
      continue;
    }
    candidate.routability_feedback_budget_exhausted =
        !global_diagnoses.empty() || !regional_diagnoses.empty();
    if (best && better(*best, candidate)) {
      best->routability_feedback_budget_exhausted = candidate.routability_feedback_budget_exhausted;
      best->routability_knowledge = knowledge;
      best->global_feedback_passes = global_feedback_passes;
      best->regional_feedback_passes = regional_feedback_passes;
      best->cheap_physical_probes = pass + 1U;
      return *best;
    }
    return candidate;
  }
  RoutingPlanResult exhausted = best.value_or(RoutingPlanResult{});
  exhausted.routability_knowledge = std::move(knowledge);
  exhausted.global_feedback_passes = global_feedback_passes;
  exhausted.regional_feedback_passes = regional_feedback_passes;
  exhausted.routability_feedback_budget_exhausted = true;
  return exhausted;
}

} // namespace flux
