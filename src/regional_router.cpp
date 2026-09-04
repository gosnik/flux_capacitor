#include "flux/core/regional_router.hpp"

#include "flux/core/board_free_space.hpp"
#include "flux/core/copper_geometry.hpp"
#include "flux/core/drc.hpp"
#include "flux/core/free_space_offset.hpp"
#include "flux/core/keepout_geometry.hpp"
#include "flux/core/router_telemetry.hpp"
#include "flux/core/routing_compute.hpp"
#include "flux/core/routing_rules.hpp"
#include "flux/core/validation.hpp"

#include <algorithm>
#include <array>
#include <bit>
#include <chrono>
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
         config.max_joint_spline_candidates > 0U &&
         config.max_joint_spline_active_pool_candidates >= config.max_joint_spline_candidates &&
         config.max_joint_spline_fallback_candidates > 0U &&
         config.max_joint_spline_fallback_waves > 0U &&
         config.max_local_routability_diagnoses > 0U &&
         config.max_residual_cone_template_coverage_states > 0U &&
         config.max_spline_portfolio_windows >= config.max_joint_repair_windows &&
         config.max_same_layer_detour_offsets > 0U &&
         config.max_adaptive_pair_shove_checks_per_window > 0U &&
         config.max_quantization_feedback_iterations > 0U &&
         config.max_quantization_feedback_candidates_per_iteration > 0U &&
         config.max_quantized_segment_pair_checks_per_window > 0U &&
         config.max_residual_conflict_cone_windows > 1U &&
         config.max_residual_retry_bundle_assignments > 0U &&
         (!config.enable_xpbd_relaxation ||
          (config.xpbd.iterations > 0U && config.xpbd.maximum_step_nm > 0 &&
           config.xpbd.maximum_candidate_pairs > 0U &&
           config.max_xpbd_obstacle_segments_per_window > 0U &&
           config.max_xpbd_interpolation_nodes_per_route > 2U &&
           config.xpbd.stretch_compliance >= 0.0L && config.xpbd.bend_compliance >= 0.0L &&
           config.xpbd.clearance_compliance >= 0.0L && config.xpbd.corridor_compliance >= 0.0L));
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
                                     std::set<std::size_t>& replaced_window_ids,
                                     std::map<std::size_t, std::set<std::size_t>>& cone_members) {
  std::map<RegionalRouteFragmentId, const RegionalRouteFragment*> fragment_by_id;
  for (const auto& fragment : retry.fragments) {
    fragment_by_id.emplace(fragment.id, &fragment);
  }
  const auto expand_endpoint_stubs = [&](std::set<RegionalRouteFragmentId> fragments,
                                         const std::set<std::size_t>& conflict_edges) {
    const auto add_adjacent = [&](const RegionalRouteFragment& source, bool before,
                                  std::set<RegionalRouteFragmentId>& expanded) {
      for (const auto& candidate : retry.fragments) {
        if (candidate.id.net != source.id.net ||
            candidate.id.connection_index != source.id.connection_index ||
            candidate.id.run_index != source.id.run_index || candidate.layer != source.layer ||
            candidate.coupled_member || candidate.explicit_width) {
          continue;
        }
        const auto adjacent = before
                                  ? candidate.id.fragment_index + 1U == source.id.fragment_index &&
                                        candidate.end == source.start
                                  : source.id.fragment_index + 1U == candidate.id.fragment_index &&
                                        source.end == candidate.start;
        if (adjacent) {
          expanded.insert(candidate.id);
        }
      }
    };
    for (const auto edge_id : conflict_edges) {
      if (edge_id >= retry.conflict_edges.size()) {
        continue;
      }
      const auto& edge = retry.conflict_edges[edge_id];
      for (const auto& id : {edge.first, edge.second}) {
        const auto found = fragment_by_id.find(id);
        if (found == fragment_by_id.end() || found->second->coupled_member ||
            found->second->explicit_width) {
          continue;
        }
        const auto& source = *found->second;
        const auto endpoint_guard = static_cast<long double>(
            source.clearance.value() + std::max<std::int64_t>(1, source.width.value() / 2));
        if (point_bounds_distance_nm(source.start, edge.conflict_bounds) <= endpoint_guard) {
          add_adjacent(source, true, fragments);
        }
        if (point_bounds_distance_nm(source.end, edge.conflict_bounds) <= endpoint_guard) {
          add_adjacent(source, false, fragments);
        }
      }
    }
    // Only one deterministic neighbouring segment is admitted on each affected side.  This
    // moves the regional fixed portal away from an endpoint conflict without turning a local
    // retry into a whole-net reroute.  Large-neighbourhood escalation remains the next layer.
    return fragments;
  };
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
    // Import the owner of an immediately adjacent endpoint continuation once, from the original
    // failed window.  Imported owner windows must not contribute their unrelated conflicts to a
    // recursive endpoint expansion: that turns a local boundary repair into an accidental
    // board-wide large-neighbourhood wave and exhausted the Tier-1 time budget.  The ordinary
    // fragment-owner closure below remains transitive so the resulting transaction still has one
    // authoritative realization for every fragment it actually contains.
    {
      std::set<RegionalRouteFragmentId> endpoint_fragments{failed_window->fragments.begin(),
                                                           failed_window->fragments.end()};
      const std::set<std::size_t> endpoint_conflicts{failed_window->conflict_edges.begin(),
                                                     failed_window->conflict_edges.end()};
      endpoint_fragments = expand_endpoint_stubs(std::move(endpoint_fragments), endpoint_conflicts);
      for (const auto& fragment : endpoint_fragments) {
        const auto owner = owner_window.find(fragment);
        if (owner != owner_window.end()) {
          members.insert(owner->second);
        }
      }
    }
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
    std::set<std::size_t> conflict_edges;
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
      conflict_edges.insert(window->conflict_edges.begin(), window->conflict_edges.end());
    }
    fragments = expand_endpoint_stubs(std::move(fragments), conflict_edges);
    for (const auto& fragment : fragments) {
      nets.insert(fragment.net);
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
    merged.residual_conflict_cone = true;
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
    const auto conflict_only_fragments = fragments;
    fragments = expand_endpoint_stubs(std::move(fragments), conflict_edges);
    merged.residual_stub_fragments = fragments.size() - conflict_only_fragments.size();
    retry.residual_conflict_cone_stub_fragments += merged.residual_stub_fragments;
    for (const auto& fragment_id : fragments) {
      if (conflict_only_fragments.contains(fragment_id)) {
        continue;
      }
      const auto found = fragment_by_id.find(fragment_id);
      if (found == fragment_by_id.end()) {
        continue;
      }
      const auto& fragment = *found->second;
      layers.insert(fragment.layer);
      nets.insert(fragment.id.net);
      merged.bounds = union_bounds(merged.bounds, fragment.bounds);
      const auto dx = static_cast<long double>(fragment.end.x.value() - fragment.start.x.value());
      const auto dy = static_cast<long double>(fragment.end.y.value() - fragment.start.y.value());
      const auto length = std::hypotl(dx, dy);
      const auto tangent = length > 0.0L ? SplineVector{dx / length, dy / length} : SplineVector{};
      if (!fragment.begins_at_terminal) {
        merged.portals.push_back(RegionalBoundaryPortal{0U, fragment.id, fragment.start,
                                                        fragment.layer, tangent, fragment.width});
      }
      if (!fragment.ends_at_terminal) {
        merged.portals.push_back(
            RegionalBoundaryPortal{0U, fragment.id, fragment.end, fragment.layer,
                                   SplineVector{-tangent.x, -tangent.y}, fragment.width});
      }
    }
    merged.layers.assign(layers.begin(), layers.end());
    merged.fragments.assign(fragments.begin(), fragments.end());
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
  for (const auto& cone : cones) {
    const auto root = *std::min_element(cone.begin(), cone.end());
    cone_members.emplace(root, std::set<std::size_t>{cone.begin(), cone.end()});
  }
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
    for (auto& selection : assignment.spline_candidate_reserve) {
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
  combined.xpbd_invocations.insert(combined.xpbd_invocations.end(),
                                   std::make_move_iterator(retry.xpbd_invocations.begin()),
                                   std::make_move_iterator(retry.xpbd_invocations.end()));
  std::stable_sort(combined.windows.begin(), combined.windows.end(),
                   [](const auto& lhs, const auto& rhs) { return lhs.id < rhs.id; });
  std::stable_sort(combined.bundle_assignments.begin(), combined.bundle_assignments.end(),
                   [](const auto& lhs, const auto& rhs) { return lhs.window_id < rhs.window_id; });
  combined.residual_conflict_cone_retries = 1U;
  combined.residual_conflict_cones_built = retry.residual_conflict_cones_built;
  combined.residual_conflict_cone_windows_merged = retry.residual_conflict_cone_windows_merged;
  combined.residual_conflict_cone_stub_fragments = retry.residual_conflict_cone_stub_fragments;
  combined.residual_conflict_cone_reused_assignments = reused_assignments;
  combined.residual_conflict_cone_recomputed_assignments = recomputed_assignments;
  combined.residual_conflict_cone_retry_withheld_assignments = 0U;
  combined.residual_cone_template_coverage_states += retry.residual_cone_template_coverage_states;
  combined.residual_cone_template_coverage_retained +=
      retry.residual_cone_template_coverage_retained;
  combined.lane_solver_queries += retry.lane_solver_queries;
  combined.lane_solver_cache_hits += retry.lane_solver_cache_hits;
  combined.lane_solver_states += retry.lane_solver_states;
  combined.lane_solver_pruned_states += retry.lane_solver_pruned_states;
  combined.lane_solver_analytical_rejections += retry.lane_solver_analytical_rejections;
  combined.lane_solver_budget_exhaustions += retry.lane_solver_budget_exhaustions;
  combined.partial_lane_capacity_pruned_states += retry.partial_lane_capacity_pruned_states;
  combined.spline_interpolation_gpu_batches += retry.spline_interpolation_gpu_batches;
  combined.spline_interpolation_gpu_fallbacks += retry.spline_interpolation_gpu_fallbacks;
  combined.spline_interpolation_gpu_candidates += retry.spline_interpolation_gpu_candidates;
  combined.spline_interpolation_exact_candidates += retry.spline_interpolation_exact_candidates;
  combined.spline_geometry_cache_hits += retry.spline_geometry_cache_hits;
  combined.spline_geometry_cache_misses += retry.spline_geometry_cache_misses;
  combined.spline_offset_broadphase_batches += retry.spline_offset_broadphase_batches;
  combined.spline_offset_broadphase_gpu_batches += retry.spline_offset_broadphase_gpu_batches;
  combined.spline_offset_broadphase_rejections += retry.spline_offset_broadphase_rejections;
  combined.spline_offset_exact_pairs += retry.spline_offset_exact_pairs;
  combined.spline_offset_subdivision_nodes += retry.spline_offset_subdivision_nodes;
  combined.spline_offset_isolated_leaf_pairs += retry.spline_offset_isolated_leaf_pairs;
  combined.spline_offset_node_budget_exhaustions +=
      retry.spline_offset_node_budget_exhaustions;
  combined.spline_interpolation_rank_us += retry.spline_interpolation_rank_us;
  combined.spline_interpolation_exact_us += retry.spline_interpolation_exact_us;
  combined.spline_adaptive_assessment_us += retry.spline_adaptive_assessment_us;
  combined.spline_offset_broadphase_us += retry.spline_offset_broadphase_us;
  combined.spline_offset_exact_us += retry.spline_offset_exact_us;
  combined.spline_realization_us += retry.spline_realization_us;
  combined.xpbd_windows_attempted += retry.xpbd_windows_attempted;
  combined.xpbd_unconflicted_candidates_skipped += retry.xpbd_unconflicted_candidates_skipped;
  combined.xpbd_windows_improved += retry.xpbd_windows_improved;
  combined.xpbd_windows_legalized += retry.xpbd_windows_legalized;
  combined.xpbd_gpu_invocations += retry.xpbd_gpu_invocations;
  combined.xpbd_cpu_fallbacks += retry.xpbd_cpu_fallbacks;
  combined.xpbd_topology_rejections += retry.xpbd_topology_rejections;
  combined.xpbd_pressure_cells += retry.xpbd_pressure_cells;
  combined.xpbd_terrain_cells_sampled += retry.xpbd_terrain_cells_sampled;
  combined.xpbd_obstacle_segments += retry.xpbd_obstacle_segments;
  combined.xpbd_obstacle_segment_budget_exhaustions +=
      retry.xpbd_obstacle_segment_budget_exhaustions;
  combined.xpbd_constraints += retry.xpbd_constraints;
  combined.xpbd_broadphase_cells += retry.xpbd_broadphase_cells;
  combined.xpbd_broadphase_pair_emissions += retry.xpbd_broadphase_pair_emissions;
  combined.xpbd_resident_input_buffer_reuses += retry.xpbd_resident_input_buffer_reuses;
  combined.xpbd_gpu_broadphase_invocations += retry.xpbd_gpu_broadphase_invocations;
  combined.xpbd_broadphase_fallbacks += retry.xpbd_broadphase_fallbacks;
  combined.xpbd_gpu_pressure_field_invocations += retry.xpbd_gpu_pressure_field_invocations;
  combined.xpbd_unresolved_trace_pairs += retry.xpbd_unresolved_trace_pairs;
  combined.xpbd_stress += retry.xpbd_stress;
  combined.xpbd_initial_violations += retry.xpbd_initial_violations;
  combined.xpbd_final_violations += retry.xpbd_final_violations;
  combined.xpbd_peak_initial_residual =
      std::max(combined.xpbd_peak_initial_residual, retry.xpbd_peak_initial_residual);
  combined.xpbd_peak_final_residual =
      std::max(combined.xpbd_peak_final_residual, retry.xpbd_peak_final_residual);
  combined.xpbd_broadphase_us += retry.xpbd_broadphase_us;
  combined.xpbd_constraint_generation_us += retry.xpbd_constraint_generation_us;
  combined.xpbd_solve_us += retry.xpbd_solve_us;
  combined.xpbd_stress_reduction_us += retry.xpbd_stress_reduction_us;
  combined.xpbd_readback_us += retry.xpbd_readback_us;
  combined.xpbd_peak_pressure = std::max(combined.xpbd_peak_pressure, retry.xpbd_peak_pressure);
  combined.terminal_navmesh_template_checks += retry.terminal_navmesh_template_checks;
  combined.accepted_terminal_corridor_rebinds += retry.accepted_terminal_corridor_rebinds;
  combined.accepted_terminal_navmesh_templates += retry.accepted_terminal_navmesh_templates;
  combined.rejected_terminal_navmesh_templates += retry.rejected_terminal_navmesh_templates;
  combined.terminal_navmesh_template_budget_exhausted =
      combined.terminal_navmesh_template_budget_exhausted ||
      retry.terminal_navmesh_template_budget_exhausted;
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

[[nodiscard]] bool run_layer_selection_legal(const std::vector<RegionalRouteTemplate>& templates,
                                             const std::vector<std::size_t>& selected) {
  using RunKey = std::tuple<NetId, std::size_t, std::size_t>;
  std::map<RunKey, std::vector<const RegionalRouteTemplate*>> by_run;
  for (const auto template_id : selected) {
    if (template_id >= templates.size()) {
      return false;
    }
    const auto& route_template = templates[template_id];
    by_run[RunKey{route_template.fragment.net, route_template.fragment.connection_index,
                  route_template.fragment.run_index}]
        .push_back(&route_template);
  }
  for (auto& [run, members] : by_run) {
    static_cast<void>(run);
    std::sort(members.begin(), members.end(), [](const auto* lhs, const auto* rhs) {
      return lhs->fragment.fragment_index < rhs->fragment.fragment_index;
    });
    for (std::size_t index = 1U; index < members.size(); ++index) {
      const auto* previous = members[index - 1U];
      const auto* current = members[index];
      if (previous->fragment.fragment_index + 1U != current->fragment.fragment_index ||
          previous->selected_global_topology || current->selected_global_topology) {
        continue;
      }
      // Adjacent alternatives are one continuous route run.  Different routing layers would
      // require an unmodelled internal via state; reject that combination and let the bounded
      // solver choose a coherent layer or two explicitly separated excursions.
      if (previous->routing_layer != current->routing_layer) {
        return false;
      }
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

[[nodiscard]] RegionalAccessSelection
select_access_geometry(const Board& board, const RegionalRouteResult& regional,
                       const RegionalRoutingWindow& window,
                       const std::vector<RegionalRouteTemplate>& templates,
                       const std::vector<std::size_t>& selected, std::size_t maximum_states) {
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
      if (result.states >= maximum_states) {
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
      const auto fragment_matches_corridor_direction = [&](const RegionalRouteFragment& fragment) {
        return corridor.topology_signature.valid && corridor.lane_centerline.size() >= 2U &&
               fragment.start == corridor.lane_centerline.front() &&
               fragment.end == corridor.lane_centerline.back();
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
          if (fragment == result.fragments.end() || fragment->layer != corridor.layer ||
              !fragment_traverses_corridor(*fragment)) {
            continue;
          }
          if (std::find(channel->occupants.begin(), channel->occupants.end(), fragment_id) ==
              channel->occupants.end()) {
            channel->occupants.push_back(fragment_id);
          }
          if (fragment_matches_corridor_direction(*fragment) &&
              std::none_of(channel->fragment_topology_bindings.begin(),
                           channel->fragment_topology_bindings.end(),
                           [&](const auto& binding) { return binding.fragment == fragment_id; })) {
            channel->fragment_topology_bindings.push_back(RegionalFragmentTopologyBinding{
                fragment_id, corridor.topology_signature, corridor.lane_centerline});
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
    const RegionalRouteResult& regional, const std::vector<RegionalRouteTemplate>& templates,
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
  // Dense template products are first filtered as compatibility bitsets on the GPU. For a
  // completely enumerated product this replaces branch-and-bound enumeration; every surviving
  // assignment is still rechecked here for via identity and aggregate physical channel width.
  RoutingComputeAssignmentInput compute_assignment;
  compute_assignment.maximum_states = maximum_states;
  compute_assignment.top_k = maximum_states;
  compute_assignment.option_costs.resize(templates.size(), 0U);
  for (std::size_t position = 0U; position < templates_by_fragment.size(); ++position) {
    auto& domain = compute_assignment.variable_domains.emplace_back();
    for (const auto template_id : templates_by_fragment[position]) {
      if (template_id >= templates.size())
        continue;
      domain.push_back(static_cast<std::uint32_t>(template_id));
      compute_assignment.option_costs[template_id] = static_cast<std::uint32_t>(
          std::min<std::size_t>(std::numeric_limits<std::uint32_t>::max(),
                                templates[template_id].via_choices.size() * 4U +
                                    templates[template_id].channel_sequence.size()));
    }
  }
  for (std::size_t first = 0U; first < templates.size(); ++first) {
    for (std::size_t second = first + 1U; second < templates.size(); ++second) {
      if (templates[first].fragment == templates[second].fragment)
        continue;
      bool incompatible = false;
      for (const auto first_via_id : templates[first].via_choices) {
        const auto first_via =
            std::find_if(regional.via_choices.begin(), regional.via_choices.end(),
                         [&](const auto& via) { return via.id == first_via_id; });
        if (first_via == regional.via_choices.end())
          continue;
        for (const auto second_via_id : templates[second].via_choices) {
          const auto second_via =
              std::find_if(regional.via_choices.begin(), regional.via_choices.end(),
                           [&](const auto& via) { return via.id == second_via_id; });
          if (second_via != regional.via_choices.end() && first_via->net == second_via->net &&
              first_via->region_id == second_via->region_id &&
              (first_via->site != second_via->site || first_via->span != second_via->span)) {
            incompatible = true;
            break;
          }
        }
        if (incompatible)
          break;
      }
      if (incompatible)
        compute_assignment.incompatible_options.emplace_back(static_cast<std::uint32_t>(first),
                                                             static_cast<std::uint32_t>(second));
    }
  }
  const auto accelerated_assignments = evaluate_routing_assignments(compute_assignment, true);
  const auto exact_accelerated_assignment = [&](const RoutingComputeAssignmentCandidate& candidate)
      -> std::optional<std::vector<std::size_t>> {
    if (candidate.options.size() != templates_by_fragment.size())
      return std::nullopt;
    std::map<ViaRegionKey, ViaSiteIdentity> assigned_vias;
    std::map<std::size_t, PartialChannelEnvelope> assigned_channels;
    std::vector<std::size_t> local_choices(candidate.options.size(), 0U);
    for (std::size_t position = 0U; position < candidate.options.size(); ++position) {
      const auto template_id = static_cast<std::size_t>(candidate.options[position]);
      const auto local = std::find(templates_by_fragment[position].begin(),
                                   templates_by_fragment[position].end(), template_id);
      if (template_id >= templates.size() || local == templates_by_fragment[position].end())
        return std::nullopt;
      local_choices[position] =
          static_cast<std::size_t>(local - templates_by_fragment[position].begin());
      for (const auto via_id : templates[template_id].via_choices) {
        const auto via = std::find_if(regional.via_choices.begin(), regional.via_choices.end(),
                                      [&](const auto& value) { return value.id == via_id; });
        if (via == regional.via_choices.end())
          return std::nullopt;
        const auto key = ViaRegionKey{via->net, via->region_id};
        const ViaSiteIdentity identity{via->site, via->span};
        if (const auto found = assigned_vias.find(key);
            found != assigned_vias.end() &&
            (found->second.site != identity.site || found->second.span != identity.span))
          return std::nullopt;
        assigned_vias.insert_or_assign(key, identity);
      }
      const auto envelope = fragment_envelopes.find(templates[template_id].fragment);
      if (envelope == fragment_envelopes.end() || !envelope->second.valid)
        return std::nullopt;
      auto channels = templates[template_id].channel_sequence;
      std::sort(channels.begin(), channels.end());
      channels.erase(std::unique(channels.begin(), channels.end()), channels.end());
      for (const auto channel_id : channels) {
        const auto capacity = maximum_channel_width_nm.find(channel_id);
        if (capacity == maximum_channel_width_nm.end())
          return std::nullopt;
        auto& used = assigned_channels[channel_id];
        used.width_and_clearance_nm += envelope->second.width_and_clearance_nm;
        used.maximum_clearance_nm =
            std::max(used.maximum_clearance_nm, envelope->second.clearance_nm);
        if (used.width_and_clearance_nm + used.maximum_clearance_nm > capacity->second)
          return std::nullopt;
      }
    }
    return local_choices;
  };
  std::vector<std::vector<std::size_t>> accelerated_exact_choices;
  const bool accelerated_portfolio_complete = accelerated_assignments.ok() &&
                                              accelerated_assignments.used_gpu &&
                                              !accelerated_assignments.state_space_truncated;
  if (accelerated_portfolio_complete) {
    for (const auto& candidate : accelerated_assignments.shortlist)
      if (const auto exact = exact_accelerated_assignment(candidate))
        accelerated_exact_choices.push_back(*exact);
  }
  // Preserve the established deterministic DFS as the primary portfolio.  A dense GPU result is
  // a fallback reservoir for branches the bounded DFS could not reach, not permission to remove
  // or reorder a previously reachable regional topology.  This matters because downstream exact
  // spline/via realization can prefer a nominally more expensive template.
  // This search establishes compatible topology/layer/via-template bundles. Terminal escape and
  // via run-up selection is a separate bounded CSP evaluated below for each retained complete
  // choice. Do not spend one shared access budget here: an early difficult leaf could exhaust it
  // and discard every later topology as though it had been proved illegal. In particular that
  // erased longer fallback corridors in residual cones. Keep the complete leaves and let the
  // authoritative per-choice access/lane stage reject or accept them with its own full budget.
  std::function<void(std::size_t, const std::map<ViaRegionKey, ViaSiteIdentity>&)> search =
      [&](const std::size_t depth, const std::map<ViaRegionKey, ViaSiteIdentity>& assigned_vias) {
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
          choice[position] = candidate_index;
          search(depth + 1U, next_vias);
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
  search(0U, {});
  if ((result.exhausted || result.choices.empty()) && accelerated_portfolio_complete) {
    for (const auto& accelerated : accelerated_exact_choices) {
      if (result.choices.size() >= maximum_states)
        break;
      if (std::find(result.choices.begin(), result.choices.end(), accelerated) ==
          result.choices.end())
        result.choices.push_back(accelerated);
    }
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
  const auto static_drills = drill_shapes(board);
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
  std::map<LayerId, std::uint32_t> via_filter_layer_codes;
  for (const auto& layer : board.layers)
    if (layer.kind == LayerKind::copper)
      via_filter_layer_codes.emplace(layer.id,
                                     static_cast<std::uint32_t>(via_filter_layer_codes.size()));
  std::map<NetId, std::uint32_t> via_filter_net_codes;
  for (const auto& net : board.nets)
    via_filter_net_codes.emplace(net.id, static_cast<std::uint32_t>(via_filter_net_codes.size()));
  std::vector<RoutingComputeSegment> via_filter_obstacles;
  for (const auto& track : board.tracks) {
    const auto layer = via_filter_layer_codes.find(track.layer);
    if (layer == via_filter_layer_codes.end())
      continue;
    via_filter_obstacles.push_back(RoutingComputeSegment{
        track.start.x.value(), track.start.y.value(), track.end.x.value(), track.end.y.value(),
        track.width.value() / 2, layer->second,
        via_filter_net_codes.contains(track.net) ? via_filter_net_codes.at(track.net)
                                                 : 0xffffffffU});
  }
  for (const auto& arc : board.track_arcs) {
    const auto layer = via_filter_layer_codes.find(arc.layer);
    if (layer == via_filter_layer_codes.end())
      continue;
    const auto owner =
        via_filter_net_codes.contains(arc.net) ? via_filter_net_codes.at(arc.net) : 0xffffffffU;
    for (const auto [start, end] :
         {std::pair{arc.start, arc.middle}, std::pair{arc.middle, arc.end}}) {
      via_filter_obstacles.push_back(
          RoutingComputeSegment{start.x.value(), start.y.value(), end.x.value(), end.y.value(),
                                arc.width.value() / 2, layer->second, owner});
    }
  }
  for (const auto& via : board.vias) {
    for (const auto& layer_id : via.layers) {
      const auto layer = via_filter_layer_codes.find(layer_id);
      if (layer == via_filter_layer_codes.end())
        continue;
      via_filter_obstacles.push_back(RoutingComputeSegment{
          via.position.x.value(), via.position.y.value(), via.position.x.value(),
          via.position.y.value(), via.diameter.value() / 2, layer->second,
          via_filter_net_codes.contains(via.net) ? via_filter_net_codes.at(via.net) : 0xffffffffU});
    }
  }
  for (const auto& pad : static_pads) {
    for (const auto& layer_id : pad.layers) {
      const auto layer = via_filter_layer_codes.find(layer_id);
      if (layer == via_filter_layer_codes.end())
        continue;
      via_filter_obstacles.push_back(RoutingComputeSegment{
          pad.position.x.value(), pad.position.y.value(), pad.position.x.value(),
          pad.position.y.value(), static_cast<std::int64_t>(std::ceil(pad.conservative_radius_nm)),
          layer->second,
          via_filter_net_codes.contains(pad.net) ? via_filter_net_codes.at(pad.net) : 0xffffffffU});
    }
  }
  struct PlannedViaSite {
    NetId net;
    Point site;
    std::vector<LayerId> span;
    Nanometres diameter;
    Nanometres drill;
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
            PlannedViaSite{path.net, site, span, rules.via_diameter, rules.via_drill,
                           rules.clearance});
      }
      begin = end;
    }
  }
  const auto via_site_legal = [&](const NetId& net, Point site, const std::vector<LayerId>& span,
                                  Nanometres diameter) {
    const auto rules = resolve_net_routing_rules(board, net);
    const auto radius = static_cast<long double>(diameter.value()) / 2.0L;
    const auto drill_radius = static_cast<long double>(rules.via_drill.value()) / 2.0L;
    const auto hole_clearance = static_cast<long double>(board.rules.hole_clearance.value());
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
      const auto required =
          std::max(radius + static_cast<long double>(pairwise_clearance(net, track.net).value()),
                   drill_radius + hole_clearance) +
          static_cast<long double>(track.width.value()) / 2.0L;
      if (geometry::point_segment_distance_nm(site, track.start, track.end) < required) {
        return false;
      }
    }
    for (const auto& arc : board.track_arcs) {
      if (arc.net == net || !touches_layer(arc.layer)) {
        continue;
      }
      const auto required =
          std::max(radius + static_cast<long double>(pairwise_clearance(net, arc.net).value()),
                   drill_radius + hole_clearance) +
          static_cast<long double>(arc.width.value()) / 2.0L;
      if (geometry::point_circular_arc_distance_nm(site, arc.start, arc.middle, arc.end) <
          required) {
        return false;
      }
    }
    for (const auto& via : board.vias) {
      if (via.net == net || std::none_of(via.layers.begin(), via.layers.end(), touches_layer)) {
        continue;
      }
      const auto required =
          std::max(radius + static_cast<long double>(pairwise_clearance(net, via.net).value()),
                   drill_radius + hole_clearance) +
          static_cast<long double>(via.diameter.value()) / 2.0L;
      if (geometry::point_distance_nm(site, via.position) < required) {
        return false;
      }
    }
    for (const auto& pad : static_pads) {
      if (pad.net == net || std::none_of(pad.layers.begin(), pad.layers.end(), touches_layer)) {
        continue;
      }
      const auto required =
          std::max(radius + static_cast<long double>(pairwise_clearance(net, pad.net).value()),
                   drill_radius + hole_clearance);
      if (point_to_pad_copper_distance_nm(site, pad) < required) {
        return false;
      }
    }
    for (const auto& drill : static_drills) {
      if (drill.net == net && drill.position == site) {
        continue;
      }
      const auto overlaps_span =
          std::any_of(drill.layers.begin(), drill.layers.end(), touches_layer);
      if (drill.net != net && overlaps_span && hole_clearance > 0.0L &&
          point_to_drill_boundary_distance_nm(site, drill) < radius + hole_clearance) {
        return false;
      }
      if (drill.mechanically_drilled && board.rules.hole_to_hole_clearance.value() > 0 &&
          point_to_drill_boundary_distance_nm(site, drill) <
              drill_radius +
                  static_cast<long double>(board.rules.hole_to_hole_clearance.value())) {
        return false;
      }
    }
    if (!result.configured.treat_saved_zone_fills_as_refillable) {
      for (const auto& zone : board.zones) {
        if (zone.net == net) {
          continue;
        }
        const auto required =
            std::max(radius + static_cast<long double>(pairwise_clearance(net, zone.net).value()),
                     drill_radius + hole_clearance);
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
    if (board.rules.hole_clearance.value() > 0) {
      const auto required =
          radius + static_cast<long double>(board.rules.hole_clearance.value());
      for (const auto& drill : static_drills) {
        if (drill.net == net ||
            std::find(drill.layers.begin(), drill.layers.end(), layer) == drill.layers.end()) {
          continue;
        }
        if (segment_to_pad_copper_distance_nm(start, end, drill_clearance_shape(drill)) <
            required) {
          return false;
        }
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
        const bool spans_overlap =
            std::any_of(span.begin(), span.end(), [&](const auto& layer) {
              return std::find(planned.span.begin(), planned.span.end(), layer) !=
                     planned.span.end();
            });
        if (planned.net != path.net && spans_overlap) {
          const auto required = std::max(
              {static_cast<long double>(rules.via_diameter.value() + planned.diameter.value()) /
                       2.0L +
                   static_cast<long double>(std::max(rules.clearance, planned.clearance).value()),
               static_cast<long double>(rules.via_drill.value() + planned.diameter.value()) /
                       2.0L +
                   static_cast<long double>(board.rules.hole_clearance.value()),
               static_cast<long double>(rules.via_diameter.value() + planned.drill.value()) /
                       2.0L +
                   static_cast<long double>(board.rules.hole_clearance.value())});
          margin = std::min(margin, geometry::point_distance_nm(site, planned.site) - required);
        }
        if (!(planned.net == path.net && planned.site == site) &&
            board.rules.hole_to_hole_clearance.value() > 0) {
          const auto required =
              static_cast<long double>(rules.via_drill.value() + planned.drill.value()) / 2.0L +
              static_cast<long double>(board.rules.hole_to_hole_clearance.value());
          margin = std::min(margin, geometry::point_distance_nm(site, planned.site) - required);
        }
      }
      return margin;
    };
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
    struct RankedViaSite {
      Point site;
      long double planned_margin{0.0L};
      long double source_distance{0.0L};
      std::size_t gpu_possible_conflicts{0U};
      std::int64_t gpu_margin_nm{std::numeric_limits<std::int64_t>::max()};
      std::uint32_t gpu_rejection_bits{0U};
      std::uint64_t gpu_runup_length_nm{0U};
      long double gpu_score{0.0L};
    };
    std::vector<RankedViaSite> ranked_sites;
    ranked_sites.reserve(candidate_sites.size() - 1U);
    for (auto site = std::next(candidate_sites.begin()); site != candidate_sites.end(); ++site) {
      ranked_sites.push_back(RankedViaSite{*site, planned_margin(*site),
                                           geometry::point_distance_nm(*site, first.position)});
    }
    auto regional_filter_obstacles = via_filter_obstacles;
    for (const auto& planned : planned_via_sites) {
      for (const auto& layer_id : planned.span) {
        const auto layer = via_filter_layer_codes.find(layer_id);
        if (layer == via_filter_layer_codes.end())
          continue;
        regional_filter_obstacles.push_back(RoutingComputeSegment{
            planned.site.x.value(), planned.site.y.value(), planned.site.x.value(),
            planned.site.y.value(), planned.diameter.value() / 2, layer->second,
            via_filter_net_codes.contains(planned.net) ? via_filter_net_codes.at(planned.net)
                                                       : 0xffffffffU});
      }
    }
    std::uint64_t span_mask = 0U;
    for (const auto& layer_id : span) {
      const auto layer = via_filter_layer_codes.find(layer_id);
      if (layer != via_filter_layer_codes.end() && layer->second < 64U)
        span_mask |= std::uint64_t{1U} << layer->second;
    }
    RoutingComputeViaCandidateInput via_filter_input;
    via_filter_input.obstacles = std::move(regional_filter_obstacles);
    via_filter_input.clearance_nm = rules.clearance.value();
    via_filter_input.top_k = std::min<std::size_t>(
        ranked_sites.size(), result.configured.max_selected_via_site_checks_per_region);
    via_filter_input.candidates.reserve(ranked_sites.size());
    for (std::size_t candidate = 0U; candidate < ranked_sites.size(); ++candidate) {
      RoutingComputeViaCandidate compute_candidate;
      compute_candidate.x_nm = ranked_sites[candidate].site.x.value();
      compute_candidate.y_nm = ranked_sites[candidate].site.y.value();
      compute_candidate.radius_nm = rules.via_diameter.value() / 2;
      compute_candidate.runup_radius_nm = rules.track_width.value() / 2;
      compute_candidate.owner =
          via_filter_net_codes.contains(path.net) ? via_filter_net_codes.at(path.net) : 0xffffffffU;
      compute_candidate.layer_mask = span_mask;
      compute_candidate.runup_count =
          std::min(attached_runups.size(), compute_candidate.runup_origins_nm.size());
      for (std::size_t runup = 0U; runup < compute_candidate.runup_count; ++runup) {
        compute_candidate.runup_origins_nm[runup] = {attached_runups[runup].second.x.value(),
                                                     attached_runups[runup].second.y.value()};
        const auto layer = via_filter_layer_codes.find(attached_runups[runup].first);
        compute_candidate.runup_layers[runup] =
            layer != via_filter_layer_codes.end() ? layer->second : 0xffffffffU;
      }
      compute_candidate.corridor_occupancy_pressure = static_cast<std::uint32_t>(std::count_if(
          planned_via_sites.begin(), planned_via_sites.end(), [&](const auto& planned) {
            return geometry::point_distance_nm(ranked_sites[candidate].site, planned.site) <
                   static_cast<long double>(step * 4);
          }));
      via_filter_input.candidates.push_back(compute_candidate);
    }
    const auto via_filter = filter_routing_via_candidates(via_filter_input, true);
    if (via_filter.ok()) {
      ++result.via_site_gpu_filter_batches;
      result.via_site_gpu_candidates += via_filter.assessed_candidates;
      result.via_site_gpu_shortlisted += via_filter.shortlist.size();
      result.via_site_gpu_rejected += static_cast<std::size_t>(
          std::count_if(via_filter.rejection_bitmap.begin(), via_filter.rejection_bitmap.end(),
                        [](auto rejection) { return rejection != 0U; }));
      result.via_site_gpu_cpu_fallbacks += via_filter.cpu_fallback ? 1U : 0U;
      std::vector<RankedViaSite> shortlisted;
      shortlisted.reserve(via_filter.shortlist.size());
      for (const auto& assessment : via_filter.shortlist) {
        auto candidate = ranked_sites[assessment.candidate];
        candidate.gpu_possible_conflicts = assessment.approximate_conflicts;
        candidate.gpu_margin_nm = assessment.conservative_margin_nm;
        candidate.gpu_rejection_bits = assessment.rejection_bits;
        candidate.gpu_runup_length_nm = assessment.runup_length_nm;
        candidate.gpu_score = assessment.score;
        shortlisted.push_back(candidate);
      }
      ranked_sites = std::move(shortlisted);
    }
    std::stable_sort(
        ranked_sites.begin(), ranked_sites.end(), [](const auto& lhs, const auto& rhs) {
          const auto lhs_rejected = lhs.gpu_rejection_bits != 0U;
          const auto rhs_rejected = rhs.gpu_rejection_bits != 0U;
          if (lhs_rejected != rhs_rejected)
            return !lhs_rejected;
          if (lhs.gpu_possible_conflicts != rhs.gpu_possible_conflicts)
            return lhs.gpu_possible_conflicts < rhs.gpu_possible_conflicts;
          if (lhs.gpu_margin_nm != rhs.gpu_margin_nm)
            return lhs.gpu_margin_nm > rhs.gpu_margin_nm;
          const auto lhs_clears = lhs.planned_margin >= 0.0L;
          const auto rhs_clears = rhs.planned_margin >= 0.0L;
          if (lhs_clears != rhs_clears) {
            return lhs_clears;
          }
          if (lhs_clears && lhs.source_distance != rhs.source_distance) {
            return lhs.source_distance < rhs.source_distance;
          }
          if (lhs.gpu_score != rhs.gpu_score)
            return lhs.gpu_score < rhs.gpu_score;
          if (lhs.gpu_runup_length_nm != rhs.gpu_runup_length_nm)
            return lhs.gpu_runup_length_nm < rhs.gpu_runup_length_nm;
          return lhs.planned_margin != rhs.planned_margin ? lhs.planned_margin > rhs.planned_margin
                                                          : lhs.site < rhs.site;
        });
    candidate_sites.resize(1U);
    for (const auto& ranked : ranked_sites) {
      candidate_sites.push_back(ranked.site);
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
      if (result.via_choices.size() - region_id >= result.configured.max_via_sites_per_region) {
        break;
      }
      if (site == first.position) {
        result.via_choices.push_back(
            RegionalViaChoice{result.via_choices.size(), region_id, path.net, site, span,
                              rules.via_diameter, rules.via_drill, source_runups, true, false});
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
using RegionalAlternativeSourceKey = std::tuple<NetId, LayerId, bool>;
struct RegionalAlternativeDomainCache {
  std::map<RegionalAlternativeSourceKey, BoardFreeSpaceExtractionResult> sources;
  std::map<RegionalAlternativeGeometryKey, RegionalAlternativeGeometry> geometry;
  std::map<RegionalAlternativeDomainKey, std::vector<RegionalAlternativeDomain>> navigation;

  void release_unretained_if_oversized() {
    // Bound whole-board Boolean products and search-specific lattices without throwing away every
    // adjacent fragment's same-net/layer reuse. Accepted templates own shared immutable cell sets,
    // so clearing the scratch portfolio cannot invalidate retained candidates.
    constexpr std::size_t maximum_geometry_entries = 8U;
    constexpr std::size_t maximum_navigation_entries = 32U;
    if (sources.size() > maximum_geometry_entries || geometry.size() > maximum_geometry_entries ||
        navigation.size() > maximum_navigation_entries) {
      navigation.clear();
      geometry.clear();
      sources.clear();
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
    const auto source_key = RegionalAlternativeSourceKey{
        net, layer, result.configured.treat_saved_zone_fills_as_refillable};
    auto extracted = cache.sources.find(source_key);
    if (extracted == cache.sources.end()) {
      extracted = cache.sources
                      .emplace(source_key,
                               extract_polygonal_board_free_space(
                                   board,
                                   BoardFreeSpaceExtractionConfig{
                                       layer, net, true, true,
                                       !result.configured.treat_saved_zone_fills_as_refillable}))
                      .first;
    }
    if (extracted->second.complete()) {
      auto domain = local_regional_free_space(extracted->second.domain, search_bounds, envelope,
                                              result.configured.navmesh_cell_pitch);
      auto offset = offset_polygonal_free_space(domain ? *domain : extracted->second.domain,
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
                                       search_bounds,
                                       true,
                                       2U,
                                       Nanometres{0}};
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
                                             *search_bounds,
                                             true,
                                             2U,
                                             Nanometres{0}};
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

[[nodiscard]] std::optional<RegionalRouteTemplate>
build_terminal_navmesh_template(const Board& board, const RegionalRouteFragment& fragment,
                                Point start, Point end, RegionalRouteResult& result,
                                RegionalAlternativeDomainCache& domain_cache) {
  if (start == end || result.terminal_navmesh_template_checks >=
                          result.configured.max_terminal_navmesh_template_checks) {
    result.terminal_navmesh_template_budget_exhausted =
        result.terminal_navmesh_template_checks >=
        result.configured.max_terminal_navmesh_template_checks;
    return std::nullopt;
  }
  ++result.terminal_navmesh_template_checks;
  const auto reject = [&]() -> std::optional<RegionalRouteTemplate> {
    ++result.rejected_terminal_navmesh_templates;
    return std::nullopt;
  };
  const auto rules = resolve_net_routing_rules(board, fragment.id.net);
  const auto envelope = bounded_add(rules.clearance, Nanometres{rules.track_width.value() / 2});
  auto search_bounds = segment_bounds(start, end);
  bool represented_in_window = false;
  for (const auto& window : result.windows) {
    if (std::find(window.fragments.begin(), window.fragments.end(), fragment.id) ==
        window.fragments.end()) {
      continue;
    }
    search_bounds = union_bounds(search_bounds, window.bounds);
    represented_in_window = true;
  }
  if (!represented_in_window) {
    return reject();
  }
  const auto& domains = prepared_alternative_domains(board, fragment.id.net, fragment.layer,
                                                     envelope, search_bounds, result, domain_cache);
  std::optional<NavMeshRouteResult> navmesh;
  for (const auto& domain : domains) {
    if (!regional_domain_contains_point(domain, start) ||
        !regional_domain_contains_point(domain, end)) {
      continue;
    }
    const NavMeshConfig navmesh_config{result.configured.navmesh_cell_pitch,
                                       result.configured.max_navmesh_cells_per_template,
                                       result.configured.max_navmesh_expansions_per_template,
                                       {},
                                       Nanometres{0},
                                       1U,
                                       domain.minimum_boundary_clearance,
                                       search_bounds,
                                       true,
                                       2U,
                                       Nanometres{0}};
    auto candidate = route_prepared_navmesh(domain.navmesh, start, end, navmesh_config);
    if (candidate.ok()) {
      navmesh = std::move(candidate);
      break;
    }
  }
  if (!navmesh || navmesh->centerline.size() < 2U || navmesh->corridor.empty()) {
    return reject();
  }

  RegionalChannel channel;
  channel.id = result.channels.size();
  channel.layer = fragment.layer;
  channel.portals = navmesh->portals;
  channel.topology_signature = navmesh->topology_signature;
  channel.occupants = {fragment.id};
  for (const auto cell_id : navmesh->topology_guard_cells) {
    if (cell_id < navmesh->cells.size()) {
      channel.topology_cells.push_back(navmesh->cells[cell_id].polygon());
    }
  }
  if (channel.topology_cells.empty()) {
    return reject();
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
  append_endpoint_link(start, navmesh->corridor.front());
  append_endpoint_link(end, navmesh->corridor.back());

  const auto first = navmesh->centerline.front();
  const auto last = navmesh->centerline.back();
  const auto dx = static_cast<long double>(last.x.value() - first.x.value());
  const auto dy = static_cast<long double>(last.y.value() - first.y.value());
  const auto length = std::hypotl(dx, dy);
  if (!(length > 0.0L)) {
    return reject();
  }
  const auto usable = bounded_add(fragment.width, Nanometres{2 * fragment.clearance.value()});
  const auto middle_x =
      (static_cast<long double>(first.x.value()) + static_cast<long double>(last.x.value())) / 2.0L;
  const auto middle_y =
      (static_cast<long double>(first.y.value()) + static_cast<long double>(last.y.value())) / 2.0L;
  const auto normal_x = -dy / length;
  const auto normal_y = dx / length;
  const auto half = static_cast<long double>(usable.value()) / 2.0L;
  const Point first_wall{
      Nanometres{static_cast<std::int64_t>(std::llround(middle_x - half * normal_x))},
      Nanometres{static_cast<std::int64_t>(std::llround(middle_y - half * normal_y))}};
  const Point second_wall{
      Nanometres{static_cast<std::int64_t>(std::llround(middle_x + half * normal_x))},
      Nanometres{static_cast<std::int64_t>(std::llround(middle_y + half * normal_y))}};
  auto cross_section = pack_regional_cross_section(result.fragments, channel.occupants,
                                                   fragment.layer, first_wall, second_wall, usable);
  cross_section.id = 0U;
  channel.cross_sections.push_back(std::move(cross_section));
  result.channels.push_back(std::move(channel));

  RegionalRouteTemplate route_template;
  route_template.fragment = fragment.id;
  route_template.routing_layer = fragment.layer;
  route_template.channel_sequence = {result.channels.back().id};
  route_template.layer_sequence = {fragment.layer};
  route_template.topology_centerline = navmesh->centerline;
  route_template.topology_signature = navmesh->topology_signature;
  for (std::size_t edge = 1U; edge < route_template.topology_centerline.size(); ++edge) {
    const auto edge_length = geometry::segment_length_nm(
        route_template.topology_centerline[edge - 1U], route_template.topology_centerline[edge]);
    route_template.lower_bound_length_nm =
        route_template.lower_bound_length_nm >
                std::numeric_limits<std::uint64_t>::max() - edge_length
            ? std::numeric_limits<std::uint64_t>::max()
            : route_template.lower_bound_length_nm + edge_length;
  }
  route_template.topology_cost = route_template.lower_bound_length_nm;
  route_template.selected_global_topology = false;
  route_template.exact_static_checked = true;
  ++result.accepted_terminal_navmesh_templates;
  return route_template;
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
  std::vector<long double> offset_scales;
  offset_scales.reserve(result.configured.max_same_layer_detour_offsets * 2U);
  for (std::size_t multiplier = 1U; multiplier <= result.configured.max_same_layer_detour_offsets;
       ++multiplier) {
    offset_scales.push_back(-static_cast<long double>(multiplier));
    offset_scales.push_back(static_cast<long double>(multiplier));
  }
  for (const auto offset_scale : offset_scales) {
    if (result.same_layer_detour_checks >= result.configured.max_same_layer_detour_checks ||
        alternatives.size() + 1U >= result.configured.max_templates_per_fragment) {
      break;
    }
    ++result.same_layer_detour_checks;
    const auto waypoint =
        Point{Nanometres{static_cast<std::int64_t>(
                  std::llround(static_cast<long double>(conflict_center.x.value()) +
                               offset_scale * normal.x * required))},
              Nanometres{static_cast<std::int64_t>(
                  std::llround(static_cast<long double>(conflict_center.y.value()) +
                               offset_scale * normal.y * required))}};
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
                                         search_bounds,
                                         true,
                                         2U,
                                         Nanometres{0}};
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
      const auto signature = make_navmesh_topology_signature([&]() {
        auto gates = entry.topology_signature.gates;
        gates.insert(gates.end(), exit.topology_signature.gates.begin(),
                     exit.topology_signature.gates.end());
        return gates;
      }());
      if (std::any_of(alternatives.begin(), alternatives.end(), [&](const auto& prior) {
            return prior.topology_signature.fingerprint == signature.fingerprint ||
                   prior.topology_centerline == centerline;
          })) {
        continue;
      }
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
      channel.topology_signature = signature;
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

  // Via and terminal candidate generation can produce thousands of repeated exact distance
  // checks. Build one conservative physical-envelope broad phase for the complete regional
  // portfolio. A missing GPU merely executes the same bounded kernel on the CPU; an omitted pair
  // is safe to skip because the broad phase uses the maximum pairwise clearance plus a numeric
  // guard, while every reported pair still reaches the existing exact test below.
  constexpr std::size_t maximum_compute_pairs = 2'000'000U;
  constexpr std::int64_t compute_guard_nm = 1'024;
  std::map<NetId, std::uint32_t> owner_by_net;
  const auto owner_for = [&](const NetId& net) {
    const auto found = owner_by_net.find(net);
    if (found != owner_by_net.end()) {
      return found->second;
    }
    const auto owner = static_cast<std::uint32_t>(owner_by_net.size());
    owner_by_net.emplace(net, owner);
    return owner;
  };
  Nanometres maximum_clearance{};
  std::vector<NetId> candidate_nets;
  candidate_nets.reserve(result.via_choices.size() + result.terminal_choices.size());
  for (const auto& choice : result.via_choices) {
    candidate_nets.push_back(choice.net);
  }
  for (const auto& terminal : result.terminal_choices) {
    candidate_nets.push_back(terminal.net);
  }
  std::sort(candidate_nets.begin(), candidate_nets.end());
  candidate_nets.erase(std::unique(candidate_nets.begin(), candidate_nets.end()),
                       candidate_nets.end());
  for (std::size_t first = 0U; first < candidate_nets.size(); ++first) {
    static_cast<void>(owner_for(candidate_nets[first]));
    for (std::size_t second = first + 1U; second < candidate_nets.size(); ++second) {
      maximum_clearance = std::max(
          maximum_clearance, pairwise_clearance(candidate_nets[first], candidate_nets[second]));
    }
  }

  std::set<std::pair<std::size_t, std::size_t>> possible_via_pairs;
  bool use_via_broad_phase = false;
  if (result.via_choices.size() >= 2U) {
    std::vector<RoutingComputeSegment> compute_vias;
    compute_vias.reserve(result.via_choices.size());
    for (const auto& choice : result.via_choices) {
      compute_vias.push_back(RoutingComputeSegment{
          choice.site.x.value(), choice.site.y.value(), choice.site.x.value(),
          choice.site.y.value(), choice.diameter.value() / 2, 0U, owner_for(choice.net)});
    }
    std::vector<std::pair<std::uint32_t, std::uint32_t>> compute_pairs;
    std::vector<std::pair<std::size_t, std::size_t>> choice_pairs;
    bool pair_budget_exhausted = false;
    for (std::size_t first = 0U; first < result.via_choices.size(); ++first) {
      for (std::size_t second = first + 1U; second < result.via_choices.size(); ++second) {
        if (result.via_choices[first].net == result.via_choices[second].net ||
            !regional_via_spans_overlap(result.via_choices[first], result.via_choices[second])) {
          continue;
        }
        if (compute_pairs.size() == maximum_compute_pairs) {
          compute_pairs.clear();
          choice_pairs.clear();
          pair_budget_exhausted = true;
          break;
        }
        compute_pairs.emplace_back(static_cast<std::uint32_t>(first),
                                   static_cast<std::uint32_t>(second));
        choice_pairs.emplace_back(result.via_choices[first].id, result.via_choices[second].id);
      }
      if (pair_budget_exhausted) {
        break;
      }
    }
    if (!compute_pairs.empty()) {
      const auto broad = discover_routing_segment_conflicts(
          compute_vias, compute_pairs, maximum_clearance.value() + compute_guard_nm);
      if (broad.ok() && broad.possible_conflicts.size() == choice_pairs.size()) {
        use_via_broad_phase = true;
        for (std::size_t pair = 0U; pair < choice_pairs.size(); ++pair) {
          if (broad.possible_conflicts[pair] != 0U) {
            possible_via_pairs.insert(choice_pairs[pair]);
          }
        }
      }
    }
  }

  std::set<std::pair<std::size_t, std::size_t>> possible_terminal_via_pairs;
  bool use_terminal_via_broad_phase = false;
  {
    struct ComputeSegmentIdentity {
      bool via{false};
      std::size_t id{0U};
    };
    std::map<LayerId, std::uint32_t> layer_codes;
    const auto layer_for = [&](const LayerId& layer) {
      const auto found = layer_codes.find(layer);
      if (found != layer_codes.end()) {
        return found->second;
      }
      const auto code = static_cast<std::uint32_t>(layer_codes.size());
      layer_codes.emplace(layer, code);
      return code;
    };
    std::vector<RoutingComputeSegment> segments;
    std::vector<ComputeSegmentIdentity> identities;
    std::vector<std::uint32_t> terminal_segments;
    std::vector<std::uint32_t> via_segments;
    for (const auto& terminal : result.terminal_choices) {
      for (std::size_t edge = 1U; edge < terminal.access_path.size(); ++edge) {
        terminal_segments.push_back(static_cast<std::uint32_t>(segments.size()));
        segments.push_back(RoutingComputeSegment{
            terminal.access_path[edge - 1U].x.value(), terminal.access_path[edge - 1U].y.value(),
            terminal.access_path[edge].x.value(), terminal.access_path[edge].y.value(),
            terminal.width.value() / 2, layer_for(terminal.layer), owner_for(terminal.net)});
        identities.push_back(ComputeSegmentIdentity{false, terminal.id});
      }
    }
    for (const auto& via : result.via_choices) {
      for (const auto& layer : via.span) {
        via_segments.push_back(static_cast<std::uint32_t>(segments.size()));
        segments.push_back(RoutingComputeSegment{
            via.site.x.value(), via.site.y.value(), via.site.x.value(), via.site.y.value(),
            via.diameter.value() / 2, layer_for(layer), owner_for(via.net)});
        identities.push_back(ComputeSegmentIdentity{true, via.id});
      }
    }
    std::vector<std::pair<std::uint32_t, std::uint32_t>> compute_pairs;
    bool pair_budget_exhausted = false;
    for (const auto terminal_segment : terminal_segments) {
      for (const auto via_segment : via_segments) {
        if (segments[terminal_segment].layer != segments[via_segment].layer ||
            segments[terminal_segment].owner == segments[via_segment].owner) {
          continue;
        }
        if (compute_pairs.size() == maximum_compute_pairs) {
          compute_pairs.clear();
          pair_budget_exhausted = true;
          break;
        }
        compute_pairs.emplace_back(terminal_segment, via_segment);
      }
      if (pair_budget_exhausted) {
        break;
      }
    }
    if (!compute_pairs.empty()) {
      const auto broad = discover_routing_segment_conflicts(
          segments, compute_pairs, maximum_clearance.value() + compute_guard_nm);
      if (broad.ok() && broad.possible_conflicts.size() == compute_pairs.size()) {
        use_terminal_via_broad_phase = true;
        for (std::size_t pair = 0U; pair < compute_pairs.size(); ++pair) {
          if (broad.possible_conflicts[pair] == 0U) {
            continue;
          }
          const auto& terminal = identities[compute_pairs[pair].first];
          const auto& via = identities[compute_pairs[pair].second];
          possible_terminal_via_pairs.emplace(terminal.id, via.id);
        }
      }
    }
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
          const auto broad_key = std::pair{std::min(lhs_choice.id, rhs_choice.id),
                                           std::max(lhs_choice.id, rhs_choice.id)};
          if (use_via_broad_phase && !possible_via_pairs.contains(broad_key)) {
            continue;
          }
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
        if (use_terminal_via_broad_phase &&
            !possible_terminal_via_pairs.contains({terminal.id, choice.id})) {
          continue;
        }
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

  // Dense subset DP becomes worthwhile only once the permutation tail is genuinely large. Keep
  // small windows on the established exact solver: for two- and three-lane crossings its tie
  // ordering carries useful topology diversity and is cheaper than a GPU dispatch.
  if (occupants.size() >= 9U && occupants.size() <= 12U) {
    for (const auto& prepared : cross_sections) {
      if (!analytically_viable(prepared))
        continue;
      RoutingComputeLaneOrderInput compute;
      compute.usable_width_nm = static_cast<std::uint32_t>(std::min<std::int64_t>(
          prepared.source->usable_width.value(), std::numeric_limits<std::uint32_t>::max()));
      std::map<RegionalRouteFragmentId, std::uint32_t> geometric_position;
      for (std::size_t position = 0U; position < prepared.geometric_order.size(); ++position)
        geometric_position.emplace(prepared.geometric_order[position],
                                   static_cast<std::uint32_t>(position));
      std::uint32_t coupled_rank = 0U;
      for (const auto& occupant : occupants) {
        const auto found = fragments.find(occupant);
        compute.widths_nm.push_back(static_cast<std::uint32_t>(found->second->width.value()));
        compute.clearances_nm.push_back(
            static_cast<std::uint32_t>(found->second->clearance.value()));
        compute.reference_position.push_back(geometric_position.at(occupant));
        compute.coupled_rank.push_back(found->second->coupled_member
                                           ? coupled_rank++
                                           : std::numeric_limits<std::uint32_t>::max());
      }
      const auto accelerated = solve_routing_lane_order(compute, true);
      if (!accelerated.ok() || !accelerated.physically_fits ||
          accelerated.order.size() != occupants.size())
        continue;
      std::vector<RegionalRouteFragmentId> order;
      for (const auto index : accelerated.order)
        order.push_back(occupants[index]);
      consider(order, prepared);
    }
    if (best_inversions && *best_inversions == 0U) {
      result.complete_search = true;
      return result;
    }
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
  if (telemetry != nullptr && result.via_site_gpu_filter_batches > 0U) {
    RouterTelemetryEvent progress;
    progress.kind = RouterTelemetryEventKind::search_summary;
    progress.stable_id = "regional-via-gpu-filter:" + std::string(phase);
    progress.detail = std::string(phase) + " regional GPU via filtering: batches=" +
                      std::to_string(result.via_site_gpu_filter_batches) +
                      ", candidates=" + std::to_string(result.via_site_gpu_candidates) +
                      ", shortlisted=" + std::to_string(result.via_site_gpu_shortlisted) +
                      ", approximate_rejections=" + std::to_string(result.via_site_gpu_rejected) +
                      ", cpu_fallbacks=" + std::to_string(result.via_site_gpu_cpu_fallbacks);
    static_cast<void>(telemetry->try_emit(std::move(progress)));
  }
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
  using TerminalNavmeshTemplateKey = std::tuple<RegionalRouteFragmentId, Point, Point>;
  std::map<TerminalNavmeshTemplateKey, std::optional<RegionalRouteTemplate>>
      terminal_navmesh_template_cache;
  const auto terminal_navmesh_template = [&](const RegionalRouteFragment& fragment, Point start,
                                             Point end) -> std::optional<RegionalRouteTemplate> {
    const auto key = TerminalNavmeshTemplateKey{fragment.id, start, end};
    if (const auto found = terminal_navmesh_template_cache.find(key);
        found != terminal_navmesh_template_cache.end()) {
      return found->second;
    }
    auto generated =
        build_terminal_navmesh_template(board, fragment, start, end, result, domain_cache);
    domain_cache.release_unretained_if_oversized();
    terminal_navmesh_template_cache.emplace(key, generated);
    return generated;
  };
  const auto rebind_terminal_to_selected_corridor =
      [&](const RegionalRouteTemplate& selected, Point start,
          Point end) -> std::optional<RegionalRouteTemplate> {
    if (selected.topology_centerline.size() < 2U || start == end) {
      return std::nullopt;
    }
    struct PreparedTopologyCell {
      const Polygon* polygon{nullptr};
      geometry::AxisAlignedBounds bounds;
    };
    std::vector<PreparedTopologyCell> topology_cells;
    std::vector<RegionalTopologyLink> endpoint_links;
    for (const auto channel_id : selected.channel_sequence) {
      const auto channel =
          std::find_if(result.channels.begin(), result.channels.end(),
                       [&](const auto& candidate) { return candidate.id == channel_id; });
      if (channel == result.channels.end()) {
        continue;
      }
      for (const auto& cell : channel->topology_cells) {
        if (const auto bounds = regional_polygon_bounds(cell)) {
          topology_cells.push_back(PreparedTopologyCell{&cell, *bounds});
        }
      }
      endpoint_links.insert(endpoint_links.end(), channel->endpoint_links.begin(),
                            channel->endpoint_links.end());
    }
    const auto chord_in_one_cell = [&](Point first, Point second) {
      return first == second ||
             std::any_of(topology_cells.begin(), topology_cells.end(), [&](const auto& cell) {
               return cell.bounds.contains(first) && cell.bounds.contains(second) &&
                      geometry::locate_point(*cell.polygon, first) !=
                          geometry::PointLocation::outside &&
                      geometry::locate_point(*cell.polygon, second) !=
                          geometry::PointLocation::outside &&
                      geometry::segment_inside_polygon(first, second, *cell.polygon);
             });
    };
    auto centerline = selected.topology_centerline;
    const auto prepend_endpoint = [&](Point endpoint) {
      if (endpoint == centerline.front()) {
        return true;
      }
      if (chord_in_one_cell(endpoint, centerline.front())) {
        centerline.insert(centerline.begin(), endpoint);
        return true;
      }
      // The selected NavMesh channel retains a certified endpoint-to-cell-centre chord.  A new
      // terminal portal in that same first cell can splice through the centre without rebuilding
      // the complete free-space decomposition.  Preserve the original endpoint chord as part of
      // the certificate; later simplification may remove it only after the ordinary exact gates.
      const auto link =
          std::find_if(endpoint_links.begin(), endpoint_links.end(), [&](const auto& candidate) {
            return candidate.first == centerline.front() &&
                   chord_in_one_cell(endpoint, candidate.second);
          });
      if (link == endpoint_links.end()) {
        return false;
      }
      centerline.insert(centerline.begin(), link->second);
      centerline.insert(centerline.begin(), endpoint);
      return true;
    };
    const auto append_endpoint = [&](Point endpoint) {
      if (endpoint == centerline.back()) {
        return true;
      }
      if (chord_in_one_cell(centerline.back(), endpoint)) {
        centerline.push_back(endpoint);
        return true;
      }
      const auto link =
          std::find_if(endpoint_links.begin(), endpoint_links.end(), [&](const auto& candidate) {
            return candidate.first == centerline.back() &&
                   chord_in_one_cell(candidate.second, endpoint);
          });
      if (link == endpoint_links.end()) {
        return false;
      }
      centerline.push_back(link->second);
      centerline.push_back(endpoint);
      return true;
    };
    if (!prepend_endpoint(start) || !append_endpoint(end)) {
      return std::nullopt;
    }
    centerline.erase(std::unique(centerline.begin(), centerline.end()), centerline.end());
    if (selected.topology_signature.valid) {
      const auto topology =
          validate_navmesh_topology_signature(selected.topology_signature, centerline);
      if (!topology.preserved) {
        return std::nullopt;
      }
    }
    auto rebound = selected;
    rebound.topology_centerline = std::move(centerline);
    rebound.lower_bound_length_nm = 0U;
    for (std::size_t edge = 1U; edge < rebound.topology_centerline.size(); ++edge) {
      const auto edge_length = geometry::segment_length_nm(rebound.topology_centerline[edge - 1U],
                                                           rebound.topology_centerline[edge]);
      rebound.lower_bound_length_nm =
          rebound.lower_bound_length_nm > std::numeric_limits<std::uint64_t>::max() - edge_length
              ? std::numeric_limits<std::uint64_t>::max()
              : rebound.lower_bound_length_nm + edge_length;
    }
    rebound.topology_cost = rebound.lower_bound_length_nm;
    rebound.selected_global_topology = false;
    rebound.exact_static_checked = true;
    ++result.accepted_terminal_corridor_rebinds;
    return rebound;
  };
  const auto build_terminal_portal_navmesh_templates = [&](const RegionalRouteFragment& fragment,
                                                           const RegionalRouteTemplate& selected,
                                                           std::size_t maximum_alternatives,
                                                           bool allow_full_navmesh_search) {
    std::vector<RegionalRouteTemplate> generated;
    if (!result.configured.enable_terminal_portal_templates || maximum_alternatives == 0U ||
        (!fragment.begins_at_terminal && !fragment.ends_at_terminal)) {
      return generated;
    }
    constexpr auto no_terminal_choice = std::numeric_limits<std::size_t>::max();
    const auto terminal_options = [&](Point endpoint) {
      std::vector<std::size_t> options;
      for (const auto& choice : result.terminal_choices) {
        if (choice.net == fragment.id.net && choice.pad_position == endpoint &&
            choice.layer == fragment.layer && !choice.via_choice) {
          options.push_back(choice.id);
        }
      }
      std::stable_sort(options.begin(), options.end(), [&](const auto lhs, const auto rhs) {
        const auto lhs_choice =
            std::find_if(result.terminal_choices.begin(), result.terminal_choices.end(),
                         [&](const auto& choice) { return choice.id == lhs; });
        const auto rhs_choice =
            std::find_if(result.terminal_choices.begin(), result.terminal_choices.end(),
                         [&](const auto& choice) { return choice.id == rhs; });
        const auto lhs_selected =
            lhs_choice != result.terminal_choices.end() && lhs_choice->selected_global_access;
        const auto rhs_selected =
            rhs_choice != result.terminal_choices.end() && rhs_choice->selected_global_access;
        return lhs_selected != rhs_selected ? lhs_selected : lhs < rhs;
      });
      if (options.size() > result.configured.max_terminal_choices_per_terminal) {
        options.resize(result.configured.max_terminal_choices_per_terminal);
      }
      return options;
    };
    auto entry_options =
        fragment.begins_at_terminal ? terminal_options(fragment.start) : std::vector<std::size_t>{};
    auto exit_options =
        fragment.ends_at_terminal ? terminal_options(fragment.end) : std::vector<std::size_t>{};
    if (entry_options.empty()) {
      entry_options.push_back(no_terminal_choice);
    }
    if (exit_options.empty()) {
      exit_options.push_back(no_terminal_choice);
    }
    const auto selected_entry = selected.terminal_entry_choice.value_or(no_terminal_choice);
    const auto selected_exit = selected.terminal_exit_choice.value_or(no_terminal_choice);
    if (selected.entry_via_choice && fragment.begins_at_terminal) {
      entry_options = {selected_entry};
    }
    if (selected.exit_via_choice && fragment.ends_at_terminal) {
      exit_options = {selected_exit};
    }
    std::vector<std::pair<std::size_t, std::size_t>> endpoint_pairs;
    const auto append_pair = [&](std::size_t entry, std::size_t exit) {
      const auto pair = std::pair{entry, exit};
      if (pair != std::pair{selected_entry, selected_exit} &&
          std::find(endpoint_pairs.begin(), endpoint_pairs.end(), pair) == endpoint_pairs.end()) {
        endpoint_pairs.push_back(pair);
      }
    };
    for (const auto entry : entry_options) {
      append_pair(entry, selected_exit);
    }
    for (const auto exit : exit_options) {
      append_pair(selected_entry, exit);
    }
    for (const auto entry : entry_options) {
      for (const auto exit : exit_options) {
        append_pair(entry, exit);
      }
    }
    const auto terminal_choice = [&](std::size_t id) -> const RegionalTerminalChoice* {
      if (id == no_terminal_choice) {
        return nullptr;
      }
      const auto found =
          std::find_if(result.terminal_choices.begin(), result.terminal_choices.end(),
                       [&](const auto& choice) { return choice.id == id; });
      return found == result.terminal_choices.end() ? nullptr : &*found;
    };
    const auto append_template = [&](std::size_t entry_id, std::size_t exit_id,
                                     RegionalRouteTemplate navmesh) {
      const auto* entry = terminal_choice(entry_id);
      const auto* exit = terminal_choice(exit_id);
      auto alternative = selected;
      alternative.routing_layer = navmesh.routing_layer;
      alternative.channel_sequence = std::move(navmesh.channel_sequence);
      alternative.topology_centerline = std::move(navmesh.topology_centerline);
      alternative.topology_signature = std::move(navmesh.topology_signature);
      alternative.lower_bound_length_nm = navmesh.lower_bound_length_nm;
      alternative.topology_cost = navmesh.topology_cost;
      alternative.selected_global_topology = false;
      alternative.exact_static_checked = true;
      alternative.terminal_entry_choice =
          entry == nullptr ? std::optional<std::size_t>{} : std::optional<std::size_t>{entry->id};
      alternative.terminal_exit_choice =
          exit == nullptr ? std::optional<std::size_t>{} : std::optional<std::size_t>{exit->id};
      if (entry != nullptr) {
        alternative.lower_bound_length_nm +=
            geometry::segment_length_nm(entry->pad_position, entry->portal);
      }
      if (exit != nullptr) {
        alternative.lower_bound_length_nm +=
            geometry::segment_length_nm(exit->pad_position, exit->portal);
      }
      alternative.topology_cost = alternative.lower_bound_length_nm;
      generated.push_back(std::move(alternative));
    };
    // Reuse the selected corridor first.  A fresh NavMesh route is intentionally a fallback:
    // rebuilding it for every rejected portal pair was both expensive and wasteful when a
    // later pair could be certified by the retained corridor's endpoint links.
    for (const auto [entry_id, exit_id] : endpoint_pairs) {
      if (generated.size() >= maximum_alternatives) {
        break;
      }
      const auto* entry = terminal_choice(entry_id);
      const auto* exit = terminal_choice(exit_id);
      const auto start = entry == nullptr ? fragment.start : entry->portal;
      const auto end = exit == nullptr ? fragment.end : exit->portal;
      auto rebound = rebind_terminal_to_selected_corridor(selected, start, end);
      if (rebound) {
        append_template(entry_id, exit_id, std::move(*rebound));
      }
    }
    if (generated.empty() && allow_full_navmesh_search) {
      for (const auto [entry_id, exit_id] : endpoint_pairs) {
        if (generated.size() >= maximum_alternatives) {
          break;
        }
        const auto* entry = terminal_choice(entry_id);
        const auto* exit = terminal_choice(exit_id);
        const auto start = entry == nullptr ? fragment.start : entry->portal;
        const auto end = exit == nullptr ? fragment.end : exit->portal;
        auto navmesh = terminal_navmesh_template(fragment, start, end);
        if (navmesh) {
          append_template(entry_id, exit_id, std::move(*navmesh));
        }
      }
    }
    return generated;
  };
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
          if (!route_template.topology_signature.valid) {
            const auto binding =
                std::find_if(channel.fragment_topology_bindings.begin(),
                             channel.fragment_topology_bindings.end(), [&](const auto& candidate) {
                               return candidate.fragment == fragment_id &&
                                      candidate.signature.valid &&
                                      candidate.centerline.size() >= 2U;
                             });
            if (binding != channel.fragment_topology_bindings.end()) {
              route_template.topology_signature = binding->signature;
              route_template.topology_centerline = binding->centerline;
            }
          }
        }
      }
      const auto selected_template_index = assignment.templates.size();
      assignment.templates.push_back(std::move(route_template));
      assignment.selected_templates.push_back(assignment.templates.back().id);
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
    auto portfolio =
        generate_consistent_bundle_choices(result, assignment.templates, candidates_by_fragment,
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
          !run_layer_selection_legal(assignment.templates, selected) ||
          !select_access_geometry(board, result, window, assignment.templates, selected,
                                  result.configured.max_access_assignment_states)
               .legal()) {
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
    const auto attributed_discrete_failure =
        !has_fitting_bundle && (portfolio.lane_capacity_pruned_states > 0U ||
                                portfolio.access_pruned_states > 0U || portfolio.exhausted);
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
        if (lhs.portfolio.lane_capacity_pruned_states !=
            rhs.portfolio.lane_capacity_pruned_states) {
          return lhs.portfolio.lane_capacity_pruned_states >
                 rhs.portfolio.lane_capacity_pruned_states;
        }
        if (lhs.portfolio.access_pruned_states != rhs.portfolio.access_pruned_states) {
          return lhs.portfolio.access_pruned_states > rhs.portfolio.access_pruned_states;
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
    // Access pruning is terminal evidence, but the earlier ranking was built mostly from
    // conflict-edge overlap and via blockers.  On dense boards that could fill the bounded
    // fragment quota with interior segments and never invoke terminal-corridor alternatives at
    // all.  Explicitly retain every owned terminal fragment as a high-priority fallback when the
    // discrete access solver reports pressure; stable truncation below still bounds the work.
    if (candidate.portfolio.access_pruned_states > 0U) {
      for (const auto& fragment_id : candidate.window->owned_fragments) {
        const auto fragment = fragment_by_id.find(fragment_id);
        if (fragment == fragment_by_id.end() ||
            (!fragment->second->begins_at_terminal && !fragment->second->ends_at_terminal)) {
          continue;
        }
        const auto [overlap, deficit] = fragment_pressure(*candidate.window, fragment_id);
        const auto local = overlap > std::numeric_limits<std::uint64_t>::max() - deficit
                               ? std::numeric_limits<std::uint64_t>::max()
                               : overlap + deficit;
        constexpr auto terminal_priority_base = std::numeric_limits<std::uint64_t>::max() / 2U;
        const auto priority = terminal_priority_base + std::min(local, terminal_priority_base - 1U);
        const auto ranked =
            std::find_if(ranked_fragments.begin(), ranked_fragments.end(),
                         [&](const auto& value) { return value.second == fragment_id; });
        if (ranked == ranked_fragments.end()) {
          ranked_fragments.emplace_back(priority, fragment_id);
        } else {
          ranked->first = std::max(ranked->first, priority);
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
      const auto source_template =
          std::find_if(candidate.assignment.templates.begin(), candidate.assignment.templates.end(),
                       [&](const auto& route_template) {
                         return route_template.fragment == fragment_id &&
                                route_template.selected_global_topology;
                       });
      const auto terminal_escape_pressure =
          (fragment->begins_at_terminal || fragment->ends_at_terminal) &&
          ((!candidate.has_fitting_bundle && candidate.portfolio.access_pruned_states > 0U) ||
           (candidate.has_fitting_bundle && candidate.window->conflict_edges.size() == 1U &&
            candidate.window->observed_overlap_length_nm > 0U));
      if (terminal_escape_pressure && source_template != candidate.assignment.templates.end() &&
          retained.size() + 1U < result.configured.max_joint_templates_per_fragment) {
        const auto terminal_limit = std::min(
            candidate.has_fitting_bundle ? std::size_t{1U}
                                         : result.configured.max_terminal_choices_per_terminal,
            result.configured.max_joint_templates_per_fragment - retained.size() - 1U);
        auto generated = build_terminal_portal_navmesh_templates(
            *fragment, *source_template, terminal_limit, !candidate.has_fitting_bundle);
        publish_portfolio_progress();
        for (auto& route_template : generated) {
          if (retained.size() + 1U >= result.configured.max_joint_templates_per_fragment) {
            break;
          }
          const auto duplicate =
              std::any_of(retained.begin(), retained.end(), [&](const auto& prior) {
                return prior.routing_layer == route_template.routing_layer &&
                       prior.terminal_entry_choice == route_template.terminal_entry_choice &&
                       prior.terminal_exit_choice == route_template.terminal_exit_choice &&
                       prior.topology_signature.fingerprint ==
                           route_template.topology_signature.fingerprint;
              });
          if (!duplicate) {
            retained.push_back(std::move(route_template));
            expanded = true;
          }
        }
      }
      // A same-layer detour is not limited to a pairwise conflict.  The generator constructs its
      // dynamic walls from every attributed same-layer blocker in this window and proves the
      // complete NavMesh detour against all of them.  Keeping the old one-edge guard made a
      // three-way crossing with no usable layer swap retain exactly one template per fragment:
      // continuous spline/PNS then had no legal topological degree of freedom to resolve it.
      // The existing per-fragment output cap and global detour-check budget keep this expansion
      // bounded; it remains a regional repair candidate until the complete transaction passes
      // exact materialization, connectivity, and DRC.
      if (result.same_layer_detour_checks < result.configured.max_same_layer_detour_checks) {
        auto generated = build_same_layer_detour_templates(board, *candidate.window, *fragment,
                                                           result, domain_cache);
        domain_cache.release_unretained_if_oversized();
        publish_portfolio_progress();
        for (auto& route_template : generated) {
          if (retained.size() + 1U >= result.configured.max_joint_templates_per_fragment) {
            break;
          }
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
    std::vector<std::pair<BundleObjective, RegionalBundleSelection>> residual_coverage_pool;
    std::optional<std::pair<BundleObjective, RegionalBundleSelection>> selected_global_fallback;
    if (!missing_topology) {
      // Low-cost global states naturally keep every fragment of a long same-net run on its
      // current layer. A residual cone may instead need the *complete run* to change layer and
      // return through a matched via pair. Generate only that atomic run state here; it remains
      // subject to the ordinary access/lane CSP and is protected only through the small regional
      // continuous shortlist below.
      std::set<std::vector<std::size_t>> residual_coverage_templates;
      if (window.residual_conflict_cone) {
        std::vector<std::size_t> global_templates;
        global_templates.reserve(templates_by_fragment.size());
        bool complete_global_choice = true;
        for (const auto& candidates : templates_by_fragment) {
          const auto selected_global =
              std::find_if(candidates.begin(), candidates.end(), [&](const auto template_id) {
                return template_id < assignment.templates.size() &&
                       assignment.templates[template_id].selected_global_topology;
              });
          if (selected_global == candidates.end()) {
            complete_global_choice = false;
            break;
          }
          global_templates.push_back(*selected_global);
        }
        if (complete_global_choice) {
          using RouteRunKey = std::tuple<NetId, std::size_t, std::size_t>;
          std::map<RouteRunKey, std::vector<std::size_t>> run_fragments;
          for (std::size_t fragment = 0U; fragment < window.fragments.size(); ++fragment) {
            const auto& id = window.fragments[fragment];
            run_fragments[RouteRunKey{id.net, id.connection_index, id.run_index}].push_back(
                fragment);
          }
          for (const auto& [run, fragments] : run_fragments) {
            static_cast<void>(run);
            if (fragments.size() < 2U ||
                residual_coverage_templates.size() >=
                    result.configured.max_residual_cone_template_coverage_states) {
              continue;
            }
            auto coverage = global_templates;
            bool complete_alternate_run = true;
            for (const auto fragment : fragments) {
              const auto alternate = std::min_element(
                  templates_by_fragment[fragment].begin(), templates_by_fragment[fragment].end(),
                  [&](const auto lhs, const auto rhs) {
                    const auto& first = assignment.templates[lhs];
                    const auto& second = assignment.templates[rhs];
                    const auto first_key =
                        std::tuple{first.selected_global_topology ? 1U : 0U,
                                   first.via_choices.size(), first.topology_cost, first.id};
                    const auto second_key =
                        std::tuple{second.selected_global_topology ? 1U : 0U,
                                   second.via_choices.size(), second.topology_cost, second.id};
                    return first_key < second_key;
                  });
              if (alternate == templates_by_fragment[fragment].end() ||
                  assignment.templates[*alternate].selected_global_topology) {
                complete_alternate_run = false;
                break;
              }
              coverage[fragment] = *alternate;
            }
            if (complete_alternate_run && coverage != global_templates) {
              residual_coverage_templates.insert(std::move(coverage));
            }
          }
        }
      }
      auto portfolio =
          generate_consistent_bundle_choices(result, assignment.templates, templates_by_fragment,
                                             result.configured.max_bundle_search_states);
      for (const auto& templates : residual_coverage_templates) {
        std::vector<std::size_t> choice;
        choice.reserve(templates.size());
        bool valid_choice = templates.size() == templates_by_fragment.size();
        for (std::size_t fragment = 0U; valid_choice && fragment < templates.size(); ++fragment) {
          const auto found = std::find(templates_by_fragment[fragment].begin(),
                                       templates_by_fragment[fragment].end(), templates[fragment]);
          if (found == templates_by_fragment[fragment].end()) {
            valid_choice = false;
            break;
          }
          choice.push_back(
              static_cast<std::size_t>(found - templates_by_fragment[fragment].begin()));
        }
        if (valid_choice && std::find(portfolio.choices.begin(), portfolio.choices.end(), choice) ==
                                portfolio.choices.end()) {
          portfolio.choices.push_back(std::move(choice));
          ++result.residual_cone_template_coverage_states;
        }
      }
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
        const auto residual_coverage_state =
            window.residual_conflict_cone &&
            std::any_of(residual_coverage_templates.begin(), residual_coverage_templates.end(),
                        [&](const auto& candidate) { return candidate == selected; });
        assignment.residual_cone_template_coverage_states += residual_coverage_state ? 1U : 0U;
        const auto coupled_legal =
            coupled_template_selection_legal(board, window, assignment.templates, selected) &&
            run_layer_selection_legal(assignment.templates, selected);
        const auto access =
            select_access_geometry(board, result, window, assignment.templates, selected,
                                   result.configured.max_access_assignment_states);
        assignment.access_assignment_states += access.states;
        assignment.access_assignment_exhaustions += access.exhausted ? 1U : 0U;
        const auto access_legal = access.legal();
        if (!coupled_legal) {
          ++assignment.coupled_rejected_states;
          assignment.residual_cone_template_coverage_coupled_rejections +=
              residual_coverage_state ? 1U : 0U;
        }
        if (!access_legal) {
          ++assignment.access_rejected_states;
          assignment.residual_cone_template_coverage_access_rejections +=
              residual_coverage_state ? 1U : 0U;
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
          const auto residual_coverage_selection = residual_coverage_state;
          const auto selected_global_state =
              std::all_of(selected.begin(), selected.end(), [&](const auto template_id) {
                return template_id < assignment.templates.size() &&
                       assignment.templates[template_id].selected_global_topology;
              });
          if (selected_global_state &&
              (!selected_global_fallback || objective < selected_global_fallback->first)) {
            selected_global_fallback = std::pair{objective, selection};
          }
          // Equal scalar objectives do not identify the same route state. Two choices can have
          // identical aggregate cost while selecting different portals, topology templates,
          // lane orders, layers, or via sites. Retain full discrete identities and use the
          // objective only for stable active/reserve ordering.
          const auto duplicate =
              std::find_if(joint_spline_pool.begin(), joint_spline_pool.end(),
                           [&](const auto& candidate) { return candidate.second == selection; });
          if (duplicate == joint_spline_pool.end()) {
            ++result.joint_spline_candidates_generated;
            const auto insertion = std::upper_bound(
                joint_spline_pool.begin(), joint_spline_pool.end(), objective,
                [](const auto& value, const auto& candidate) { return value < candidate.first; });
            joint_spline_pool.insert(insertion, {objective, selection});
            const auto pool_limit =
                result.configured.max_joint_spline_active_pool_candidates >
                        std::numeric_limits<std::size_t>::max() -
                            result.configured.max_joint_spline_fallback_candidates
                    ? std::numeric_limits<std::size_t>::max()
                    : result.configured.max_joint_spline_active_pool_candidates +
                          result.configured.max_joint_spline_fallback_candidates;
            if (joint_spline_pool.size() > pool_limit) {
              joint_spline_pool.pop_back();
              ++result.joint_spline_fallback_candidates_dropped;
            }
          }
          if (residual_coverage_selection) {
            const auto coverage_insertion = std::lower_bound(
                residual_coverage_pool.begin(), residual_coverage_pool.end(), objective,
                [](const auto& candidate, const auto& value) { return candidate.first < value; });
            if (coverage_insertion == residual_coverage_pool.end() ||
                coverage_insertion->second.templates != selection.templates) {
              residual_coverage_pool.insert(coverage_insertion, {objective, selection});
              if (residual_coverage_pool.size() >
                  result.configured.max_residual_cone_template_coverage_states) {
                residual_coverage_pool.pop_back();
              }
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
          assignment.residual_cone_template_coverage_packing_rejections +=
              residual_coverage_state ? 1U : 0U;
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
    std::optional<std::size_t> residual_coverage_index;
    if (!residual_coverage_pool.empty()) {
      const auto& protected_choice = residual_coverage_pool.front();
      const auto found = std::find_if(
          joint_spline_pool.begin(), joint_spline_pool.end(), [&](const auto& candidate) {
            return candidate.second.templates == protected_choice.second.templates &&
                   candidate.second.via_choices == protected_choice.second.via_choices;
          });
      if (found == joint_spline_pool.end()) {
        // This state was intentionally retained outside the ordinary low-cost pool. Appending it
        // here leaves index zero as the normal feasibility-first seed while guaranteeing one
        // atomic alternate-run realization reaches the continuous solver.
        joint_spline_pool.push_back(protected_choice);
        residual_coverage_index = joint_spline_pool.size() - 1U;
      } else {
        residual_coverage_index = static_cast<std::size_t>(found - joint_spline_pool.begin());
      }
    }
    if (assignment.complete() && !joint_spline_pool.empty()) {
      // Scalar-nearest states commonly differ only in one via site and reach the same continuous
      // local minimum. Keep the best active state, then maximize the minimum template Hamming
      // distance. Every other retained state remains a compact fallback rather than disappearing
      // before continuous failure can identify which decision needs to change.
      std::vector<RoutingComputeLinearCandidate> portfolio_probe_candidates;
      portfolio_probe_candidates.reserve(joint_spline_pool.size());
      const auto bounded_term = [](const auto value) {
        using Value = decltype(value);
        return value > static_cast<Value>(std::numeric_limits<std::uint32_t>::max())
                   ? std::numeric_limits<std::uint32_t>::max()
                   : static_cast<std::uint32_t>(value);
      };
      for (const auto& [objective, selection] : joint_spline_pool) {
        RoutingComputeLinearCandidate candidate;
        candidate.terms = {bounded_term(std::get<0>(objective)),
                           bounded_term(std::get<1>(objective)),
                           bounded_term(std::get<2>(objective)),
                           bounded_term(std::get<3>(objective)),
                           bounded_term(std::get<4>(objective)),
                           bounded_term(selection.via_alternative_switches),
                           bounded_term(selection.template_conflicts),
                           bounded_term(selection.residual_conflicts)};
        portfolio_probe_candidates.push_back(candidate);
      }
      // This is a cheap advisory ordering inside the already bounded diverse beam. The complete
      // state still has to survive spline realization, quantization, local PNS, and exact DRC.
      const auto portfolio_probe_scores = score_routing_compute_candidates(
          portfolio_probe_candidates,
          {1'000'000U, 250'000U, 50'000U, 20'000U, 1U, 5'000U, 250'000U, 250'000U});
      std::vector<std::size_t> retained;
      const auto ordinary_shortlist_limit = result.configured.max_joint_spline_candidates;
      const auto expanded_shortlist_limit =
          ordinary_shortlist_limit > std::numeric_limits<std::size_t>::max() / 2U
              ? std::numeric_limits<std::size_t>::max()
              : ordinary_shortlist_limit * 2U;
      // If the complete discrete state space is already tiny, realize it completely. Dropping
      // five of thirteen states merely to enforce the ordinary beam can discard the only
      // compatible lane/layer combination, while retaining those states is still a strict bound
      // and does not enlarge a genuinely combinatorial window.
      const auto shortlist_limit =
          assignment.bundle_search_states <= expanded_shortlist_limit
              ? std::min(expanded_shortlist_limit, joint_spline_pool.size())
              : std::min(ordinary_shortlist_limit, joint_spline_pool.size());
      if (!joint_spline_pool.empty()) {
        retained.push_back(0U);
      }
      if (selected_global_fallback_index && *selected_global_fallback_index != 0U &&
          retained.size() < shortlist_limit) {
        retained.push_back(*selected_global_fallback_index);
      }
      if (residual_coverage_index && *residual_coverage_index != 0U &&
          std::find(retained.begin(), retained.end(), *residual_coverage_index) == retained.end() &&
          retained.size() < shortlist_limit) {
        retained.push_back(*residual_coverage_index);
        ++result.residual_cone_template_coverage_retained;
      }
      while (retained.size() < shortlist_limit && retained.size() < joint_spline_pool.size()) {
        std::optional<std::size_t> best_index;
        std::size_t best_distance = 0U;
        const auto active_pool_limit = std::min(
            result.configured.max_joint_spline_active_pool_candidates, joint_spline_pool.size());
        for (std::size_t candidate = 0U; candidate < active_pool_limit; ++candidate) {
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
          const auto candidate_score =
              portfolio_probe_scores.ok() && candidate < portfolio_probe_scores.scores.size()
                  ? portfolio_probe_scores.scores[candidate]
                  : std::numeric_limits<std::uint64_t>::max();
          const auto best_score = best_index && portfolio_probe_scores.ok() &&
                                          *best_index < portfolio_probe_scores.scores.size()
                                      ? portfolio_probe_scores.scores[*best_index]
                                      : std::numeric_limits<std::uint64_t>::max();
          if (!best_index || minimum_distance > best_distance ||
              (minimum_distance == best_distance && candidate_score < best_score) ||
              (minimum_distance == best_distance && candidate_score == best_score &&
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
      result.joint_spline_active_candidates_retained +=
          assignment.spline_candidate_shortlist.size();
      std::vector<bool> active(joint_spline_pool.size(), false);
      for (const auto selected : retained) {
        if (selected < active.size()) {
          active[selected] = true;
        }
      }
      assignment.spline_candidate_reserve.reserve(
          std::min(result.configured.max_joint_spline_fallback_candidates,
                   joint_spline_pool.size() - assignment.spline_candidate_shortlist.size()));
      for (std::size_t candidate = 0U; candidate < joint_spline_pool.size() &&
                                       assignment.spline_candidate_reserve.size() <
                                           result.configured.max_joint_spline_fallback_candidates;
           ++candidate) {
        if (!active[candidate]) {
          assignment.spline_candidate_reserve.push_back(
              std::move(joint_spline_pool[candidate].second));
        }
      }
      result.joint_spline_fallback_candidates_retained +=
          assignment.spline_candidate_reserve.size();
    }
    if (telemetry != nullptr) {
      std::set<std::uint64_t> topology_classes;
      std::size_t alternative_templates = 0U;
      std::size_t via_templates = 0U;
      std::size_t terminal_templates = 0U;
      for (const auto& route_template : assignment.templates) {
        alternative_templates += route_template.selected_global_topology ? 0U : 1U;
        via_templates += route_template.via_choices.empty() ? 0U : 1U;
        terminal_templates +=
            route_template.terminal_entry_choice || route_template.terminal_exit_choice ? 1U : 0U;
        if (route_template.topology_signature.valid) {
          topology_classes.insert(route_template.topology_signature.fingerprint);
        }
      }
      RouterTelemetryEvent portfolio_event;
      portfolio_event.kind = RouterTelemetryEventKind::search_summary;
      portfolio_event.minimum_level = RouterTelemetryLevel::diagnostic;
      portfolio_event.stable_id =
          "regional-window-portfolio:" + std::to_string(assignment.window_id);
      portfolio_event.group_id = "regional-window:" + std::to_string(assignment.window_id);
      portfolio_event.detail =
          "regional window portfolio: templates=" + std::to_string(assignment.templates.size()) +
          ", alternatives=" + std::to_string(alternative_templates) +
          ", topology_classes=" + std::to_string(topology_classes.size()) +
          ", terminal_templates=" + std::to_string(terminal_templates) +
          ", via_templates=" + std::to_string(via_templates) +
          ", complete_states=" + std::to_string(joint_spline_pool.size()) +
          ", active=" + std::to_string(assignment.spline_candidate_shortlist.size()) +
          ", reserve=" + std::to_string(assignment.spline_candidate_reserve.size()) +
          ", bundle_states=" + std::to_string(assignment.bundle_search_states) +
          ", stop=" + std::to_string(static_cast<unsigned>(assignment.stop_reason));
      static_cast<void>(telemetry->try_emit(std::move(portfolio_event)));
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
  QuinticPHRoute spline;
  // Immutable polyline that established the selected NavMesh homotopy. This is deliberately
  // separate from QuinticPHRoute::corridor_anchors: those anchors live in control space and
  // are not guaranteed to cross the same directed portals as the source funnel centerline.
  std::vector<Point> topology_seed;
  NavMeshTopologySignature topology_signature;
  std::vector<Polygon> topology_cells;
  struct TopologyCellIndex;
  std::shared_ptr<const TopologyCellIndex> topology_cell_index;
  std::vector<RegionalTopologyLink> topology_links;
  std::vector<std::size_t> channel_sequence;
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

struct CachedRegionalOffsetGeometry {
  using SignedOffsets = std::array<PHRationalOffset, 2U>;
  std::vector<SignedOffsets> offsets;
  std::vector<std::array<std::vector<PHOffsetSubdivisionBounds>, 2U>> subdivision_bounds;
};

struct CachedRegionalSplineGeometry {
  std::vector<SplineSample> adaptive_samples;
  std::vector<SplineVector> adaptive_positions;
  std::map<std::int64_t, CachedRegionalOffsetGeometry> offsets_by_required_twice_nm;
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

[[nodiscard]] std::vector<Polygon>
regional_spline_convex_corridor(const WorkingRegionalSpline& route) {
  auto result = route.topology_cells;
  result.reserve(result.size() + route.topology_links.size());
  for (const auto& link : route.topology_links) {
    const auto dx = static_cast<long double>(link.second.x.value() - link.first.x.value());
    const auto dy = static_cast<long double>(link.second.y.value() - link.first.y.value());
    const auto length = std::hypotl(dx, dy);
    if (!(length > 0.0L) || link.radius.value() <= 0) {
      continue;
    }
    const auto nx = -dy / length * static_cast<long double>(link.radius.value());
    const auto ny = dx / length * static_cast<long double>(link.radius.value());
    const auto make = [](long double x, long double y) {
      return Point{Nanometres{static_cast<std::int64_t>(std::llround(x))},
                   Nanometres{static_cast<std::int64_t>(std::llround(y))}};
    };
    Polygon strip;
    strip.vertices = {make(static_cast<long double>(link.first.x.value()) + nx,
                           static_cast<long double>(link.first.y.value()) + ny),
                      make(static_cast<long double>(link.second.x.value()) + nx,
                           static_cast<long double>(link.second.y.value()) + ny),
                      make(static_cast<long double>(link.second.x.value()) - nx,
                           static_cast<long double>(link.second.y.value()) - ny),
                      make(static_cast<long double>(link.first.x.value()) - nx,
                           static_cast<long double>(link.first.y.value()) - ny)};
    if (geometry::is_simple_polygon(strip)) {
      result.push_back(std::move(strip));
    }
  }
  return result;
}

[[nodiscard]] NavMeshTopologyCheck
regional_spline_gate_sequence(const WorkingRegionalSpline& route,
                              const std::vector<SplineVector>& samples) {
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

// The cell-union check above only answers whether a chord lies somewhere in free space.  That is
// insufficient for a selected NavMesh route: a chord can stay in the same union while silently
// bypassing a directed portal and therefore changing its homotopy class.  Keep the full selected
// gate sequence attached to every candidate string-pull replacement.  This is intentionally a
// bounded seed test, not an additional geometry authority; exact continuous and copper checks
// still decide whether a regional transaction may be committed.
[[nodiscard]] bool regional_polyline_stays_in_topology(const WorkingRegionalSpline& route,
                                                       const std::vector<Point>& source,
                                                       Nanometres sample_spacing) {
  std::vector<Point> polyline;
  polyline.reserve(source.size());
  for (const auto point : source) {
    if (polyline.empty() || polyline.back() != point) {
      polyline.push_back(point);
    }
  }
  if (polyline.size() < 2U) {
    return false;
  }
  for (std::size_t point = 1U; point < polyline.size(); ++point) {
    if (!regional_spline_chord_stays_in_topology(route, polyline[point - 1U], polyline[point],
                                                 sample_spacing)) {
      return false;
    }
  }
  return !route.topology_signature.valid ||
         validate_navmesh_topology_signature(route.topology_signature, polyline).preserved;
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
  // regional topology cells.  A trial always includes the unchanged suffix, so a shortcut must
  // also preserve the complete directed-gate certificate rather than merely remaining inside the
  // union of cells.  This keeps the later quintic PH curve in the chosen obstacle-side class.
  std::vector<Point> result{source.front()};
  std::size_t anchor = 0U;
  while (anchor + 1U < source.size()) {
    auto farthest = anchor + 1U;
    auto candidate = farthest + 1U;
    while (candidate < source.size()) {
      std::vector<Point> trial = result;
      if (result.back() != source[candidate]) {
        trial.push_back(source[candidate]);
      }
      trial.insert(trial.end(), source.begin() + static_cast<std::ptrdiff_t>(candidate + 1U),
                   source.end());
      if (!regional_polyline_stays_in_topology(route, trial, sample_spacing)) {
        break;
      }
      farthest = candidate++;
    }
    if (result.back() != source[farthest]) {
      result.push_back(source[farthest]);
    }
    anchor = farthest;
  }
  return result;
}

[[nodiscard]] std::optional<QuinticPHRoute>
make_topology_tight_regional_spline(const std::vector<Point>& seed,
                                    std::optional<Nanometres> corner_guard,
                                    const PHInterpolationConfig& interpolation) {
  return corner_guard
             ? make_topology_preserving_quintic_ph_route(seed, *corner_guard, interpolation)
             : make_quintic_ph_route(seed, interpolation);
}

[[nodiscard]] bool regional_spline_stays_in_topology(const WorkingRegionalSpline& route,
                                                     const QuinticPHRoute& spline) {
  const auto samples = adaptive_tessellate_quintic_ph(spline, Nanometres::from_mm(0.02), 10U, 512U);
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
        static_cast<long double>(std::max(route.clearance, routes[other].clearance).value()) +
        // The foreign route is represented by an adaptively tessellated polyline. Reserve its
        // maximum chord deviation so continuous curve-to-curve clearance is not overstated.
        static_cast<long double>(Nanometres::from_mm(0.02).value());
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

[[nodiscard]] RoutingComputeProximityResult
probe_regional_spline_samples(const std::vector<WorkingRegionalSpline>& routes,
                              std::size_t route_index,
                              const std::vector<SplineVector>& query_positions,
                              const std::vector<std::vector<SplineVector>>& route_samples) {
  std::map<LayerId, std::uint32_t> layer_codes;
  for (const auto& route : routes) {
    if (!layer_codes.contains(route.layer)) {
      layer_codes.emplace(route.layer, static_cast<std::uint32_t>(layer_codes.size()));
    }
  }
  std::vector<RoutingComputePoint> queries;
  queries.reserve(query_positions.size());
  const auto query_radius =
      routes[route_index].width.value() / 2 + routes[route_index].clearance.value();
  for (const auto position : query_positions) {
    queries.push_back(RoutingComputePoint{static_cast<std::int64_t>(std::llround(position.x)),
                                          static_cast<std::int64_t>(std::llround(position.y)),
                                          query_radius, layer_codes.at(routes[route_index].layer),
                                          static_cast<std::uint32_t>(route_index)});
  }
  std::vector<RoutingComputeSegment> segments;
  for (std::size_t route = 0U; route < routes.size(); ++route) {
    if (route >= route_samples.size() || route_samples[route].size() < 2U)
      continue;
    const auto radius = routes[route].width.value() / 2 + routes[route].clearance.value() +
                        Nanometres::from_mm(0.02).value();
    for (std::size_t sample = 1U; sample < route_samples[route].size(); ++sample) {
      segments.push_back(RoutingComputeSegment{
          static_cast<std::int64_t>(std::llround(route_samples[route][sample - 1U].x)),
          static_cast<std::int64_t>(std::llround(route_samples[route][sample - 1U].y)),
          static_cast<std::int64_t>(std::llround(route_samples[route][sample].x)),
          static_cast<std::int64_t>(std::llround(route_samples[route][sample].y)), radius,
          layer_codes.at(routes[route].layer), static_cast<std::uint32_t>(route)});
    }
  }
  // Own clearances are summed by the broad phase, which is stricter than the exact max-clearance
  // rule. Therefore a non-negative result safely eliminates the expensive exact pair loop; every
  // possible conflict is still recomputed by the authoritative long-double geometry below.
  return probe_routing_proximity(queries, segments, 1'024);
}

[[nodiscard]] std::vector<SplineVector>
evaluate_regional_spline_advisory_batch(const QuinticPHRoute& spline, std::size_t sample_count) {
  RoutingComputeSplineBatchInput input;
  input.sample_count = sample_count;
  const auto bezier_controls = quintic_ph_bezier_controls(spline);
  input.control_points_nm.reserve(bezier_controls.size());
  for (const auto control : bezier_controls) {
    input.control_points_nm.emplace_back(static_cast<std::int64_t>(std::llround(control.x)),
                                         static_cast<std::int64_t>(std::llround(control.y)));
  }
  input.basis_coefficients.reserve(sample_count * bezier_controls.size());
  for (std::size_t sample = 0U; sample < sample_count; ++sample) {
    const auto parameter =
        static_cast<long double>(sample) / static_cast<long double>(sample_count - 1U);
    auto basis = quintic_ph_bezier_basis(spline, parameter);
    if (basis.size() != bezier_controls.size()) {
      return {};
    }
    input.basis_coefficients.insert(input.basis_coefficients.end(), basis.begin(), basis.end());
  }
  const auto evaluated = evaluate_routing_spline_batch(input);
  if (!evaluated.ok() || evaluated.positions_nm.size() != sample_count) {
    return {};
  }
  std::vector<SplineVector> positions;
  positions.reserve(sample_count);
  for (const auto [x, y] : evaluated.positions_nm) {
    positions.push_back(SplineVector{x, y});
  }
  return positions;
}

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
  std::vector<SplineVector> uniform_positions;
  uniform_positions.reserve(sample_count);
  for (std::size_t sample = 0U; sample < sample_count; ++sample) {
    const auto parameter =
        static_cast<long double>(sample) / static_cast<long double>(sample_count - 1U);
    uniform_positions.push_back(evaluate_quintic_ph(routes[route_index].spline, parameter));
  }
  const auto advisory_positions =
      evaluate_regional_spline_advisory_batch(routes[route_index].spline, sample_count);
  const auto broad = probe_regional_spline_samples(
      routes, route_index, advisory_positions.empty() ? uniform_positions : advisory_positions,
      route_samples);
  for (std::size_t sample = 0U; sample < sample_count; ++sample) {
    const auto parameter =
        static_cast<long double>(sample) / static_cast<long double>(sample_count - 1U);
    const auto position = uniform_positions[sample];
    if (!regional_spline_topology_contains(routes[route_index], position)) {
      ++result.topology_violations;
    }
    if (broad.ok() && sample < broad.queries.size() && !broad.queries[sample].possible_conflict) {
      continue;
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

// Exact PH offset-boundary crossing probe used at the final regional gate.  The
// ordinary closest-distance check remains necessary for containment/parallel
// overlap, while this rational test closes the gap where adaptive centerline
// samples can step across a transverse copper-envelope intersection.
[[nodiscard]] bool regional_ph_envelope_boundaries_intersect(
    const std::vector<WorkingRegionalSpline>& routes, std::size_t first_route,
    std::size_t second_route, std::vector<CachedRegionalSplineGeometry>* cache = nullptr,
    RegionalRouteResult* metrics = nullptr) {
  if (first_route >= routes.size() || second_route >= routes.size())
    return true;
  const auto& first = routes[first_route];
  const auto& second = routes[second_route];
  if (first.layer != second.layer || first.fragment.net == second.fragment.net)
    return false;
  // Twice the envelope separation is integral even when the required signed offset is at a
  // half-nanometre. It is therefore also a stable cache key across all neighbouring trials.
  const auto required_twice = first.width.value() + second.width.value() +
                              2 * std::max(first.clearance, second.clearance).value();
  const auto required = static_cast<long double>(required_twice) * 0.5L;
  const auto half = required * 0.5L;
  const auto prepare_offsets = [&](std::size_t route_index) -> const CachedRegionalOffsetGeometry* {
    if (cache == nullptr || route_index >= cache->size())
      return nullptr;
    auto& route_cache = (*cache)[route_index];
    if (const auto found = route_cache.offsets_by_required_twice_nm.find(required_twice);
        found != route_cache.offsets_by_required_twice_nm.end()) {
      if (metrics != nullptr)
        ++metrics->spline_geometry_cache_hits;
      return &found->second;
    }
    CachedRegionalOffsetGeometry prepared;
    prepared.offsets.reserve(routes[route_index].spline.segments.size());
    prepared.subdivision_bounds.reserve(routes[route_index].spline.segments.size());
    for (const auto& segment : routes[route_index].spline.segments) {
      prepared.offsets.push_back(
          {make_quintic_ph_offset(segment, -half), make_quintic_ph_offset(segment, half)});
      const auto& offsets = prepared.offsets.back();
      prepared.subdivision_bounds.push_back(
          {subdivide_quintic_ph_offset_bounds(offsets[0], 4U, Nanometres{10}),
           subdivide_quintic_ph_offset_bounds(offsets[1], 4U, Nanometres{10})});
    }
    if (metrics != nullptr)
      ++metrics->spline_geometry_cache_misses;
    return &route_cache.offsets_by_required_twice_nm
                .emplace(required_twice, std::move(prepared))
                .first->second;
  };
  CachedRegionalOffsetGeometry uncached_first;
  CachedRegionalOffsetGeometry uncached_second;
  const auto prepare_uncached = [&](const WorkingRegionalSpline& route,
                                    CachedRegionalOffsetGeometry& prepared) {
    prepared.offsets.reserve(route.spline.segments.size());
    prepared.subdivision_bounds.reserve(route.spline.segments.size());
    for (const auto& segment : route.spline.segments) {
      prepared.offsets.push_back(
          {make_quintic_ph_offset(segment, -half), make_quintic_ph_offset(segment, half)});
      const auto& offsets = prepared.offsets.back();
      prepared.subdivision_bounds.push_back(
          {subdivide_quintic_ph_offset_bounds(offsets[0], 4U, Nanometres{10}),
           subdivide_quintic_ph_offset_bounds(offsets[1], 4U, Nanometres{10})});
    }
  };
  auto* first_geometry = prepare_offsets(first_route);
  auto* second_geometry = prepare_offsets(second_route);
  if (first_geometry == nullptr) {
    prepare_uncached(first, uncached_first);
    first_geometry = &uncached_first;
  }
  if (second_geometry == nullptr) {
    prepare_uncached(second, uncached_second);
    second_geometry = &uncached_second;
  }
  const auto& first_offsets = first_geometry->offsets;
  const auto& second_offsets = second_geometry->offsets;
  for (const auto& offsets : first_offsets)
    if (!offsets[0].regular || !offsets[1].regular)
      return true;
  for (const auto& offsets : second_offsets)
    if (!offsets[0].regular || !offsets[1].regular)
      return true;

  const auto broadphase_started = std::chrono::steady_clock::now();
  std::vector<RoutingComputePrimitiveBounds> hulls;
  struct HullSource {
    std::size_t segment{0U};
    std::size_t sign{0U};
    bool first{false};
  };
  std::vector<HullSource> hull_sources;
  bool complete_subdivision = true;
  for (std::size_t segment_index = 0U;
       segment_index < first_geometry->subdivision_bounds.size(); ++segment_index)
    for (std::size_t sign_index = 0U; sign_index < 2U; ++sign_index) {
      const auto& sign = first_geometry->subdivision_bounds[segment_index][sign_index];
      complete_subdivision = complete_subdivision && !sign.empty();
      for (const auto& bounds : sign) {
        hulls.push_back({bounds.minimum_x_nm, bounds.minimum_y_nm, bounds.maximum_x_nm,
                         bounds.maximum_y_nm, 0U, 0U});
        hull_sources.push_back({segment_index, sign_index, true});
      }
    }
  for (std::size_t segment_index = 0U;
       segment_index < second_geometry->subdivision_bounds.size(); ++segment_index)
    for (std::size_t sign_index = 0U; sign_index < 2U; ++sign_index) {
      const auto& sign = second_geometry->subdivision_bounds[segment_index][sign_index];
      complete_subdivision = complete_subdivision && !sign.empty();
      for (const auto& bounds : sign) {
        hulls.push_back({bounds.minimum_x_nm, bounds.minimum_y_nm, bounds.maximum_x_nm,
                         bounds.maximum_y_nm, 0U, 1U});
        hull_sources.push_back({segment_index, sign_index, false});
      }
    }
  const auto broad = complete_subdivision
                         ? discover_routing_broadphase_pairs(
                               hulls, std::max<std::int64_t>(
                                          1, Nanometres::from_mm(0.25).value()),
                               1'048'576U, true)
                         : RoutingComputeBroadphaseResult{};
  if (metrics != nullptr) {
    ++metrics->spline_offset_broadphase_batches;
    metrics->spline_offset_broadphase_gpu_batches += broad.used_gpu ? 1U : 0U;
    metrics->spline_offset_broadphase_us += static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() -
                                                              broadphase_started)
            .count());
  }
  if (complete_subdivision && broad.ok() && broad.candidate_pairs.empty()) {
    if (metrics != nullptr)
      ++metrics->spline_offset_broadphase_rejections;
    return false;
  }
  std::set<std::tuple<std::size_t, std::size_t, std::size_t, std::size_t>> ambiguous_offsets;
  if (complete_subdivision && broad.ok()) {
    for (const auto [first_hull, second_hull] : broad.candidate_pairs) {
      if (first_hull >= hull_sources.size() || second_hull >= hull_sources.size())
        continue;
      auto a = hull_sources[first_hull];
      auto b = hull_sources[second_hull];
      if (a.first == b.first)
        continue;
      if (!a.first)
        std::swap(a, b);
      ambiguous_offsets.emplace(a.segment, a.sign, b.segment, b.sign);
    }
  }

  for (std::size_t first_index = 0U; first_index < first.spline.segments.size(); ++first_index) {
    const auto& first_segment = first.spline.segments[first_index];
    for (std::size_t second_index = 0U; second_index < second.spline.segments.size();
         ++second_index) {
      const auto& second_segment = second.spline.segments[second_index];
      if (first_segment.bounds_maximum.x + required < second_segment.bounds_minimum.x ||
          second_segment.bounds_maximum.x + required < first_segment.bounds_minimum.x ||
          first_segment.bounds_maximum.y + required < second_segment.bounds_minimum.y ||
          second_segment.bounds_maximum.y + required < first_segment.bounds_minimum.y) {
        continue;
      }
      for (std::size_t first_sign = 0U; first_sign < 2U; ++first_sign) {
        for (std::size_t second_sign = 0U; second_sign < 2U; ++second_sign) {
          if (complete_subdivision && broad.ok() &&
              !ambiguous_offsets.contains(
                  {first_index, first_sign, second_index, second_sign}))
            continue;
          const auto& first_offset = first_offsets[first_index][first_sign];
          const auto& second_offset = second_offsets[second_index][second_sign];
          const auto exact_started = std::chrono::steady_clock::now();
          PHOffsetIntersectionStats stats;
          const auto intersections = intersect_quintic_ph_offsets(
              first_offset, second_offset, Nanometres{10}, 16U, 1U, 50'000U, &stats);
          if (metrics != nullptr) {
            ++metrics->spline_offset_exact_pairs;
            metrics->spline_offset_subdivision_nodes += stats.subdivision_nodes;
            metrics->spline_offset_isolated_leaf_pairs += stats.isolated_leaf_pairs;
            metrics->spline_offset_node_budget_exhaustions +=
                stats.node_budget_exhausted ? 1U : 0U;
            metrics->spline_offset_exact_us += static_cast<std::uint64_t>(
                std::chrono::duration_cast<std::chrono::microseconds>(
                    std::chrono::steady_clock::now() - exact_started)
                    .count());
          }
          // An exhausted isolator is geometrically ambiguous and therefore cannot prove legal
          // copper. Preserve CPU authority by failing closed rather than treating an empty partial
          // search as separation.
          if (stats.node_budget_exhausted || !intersections.empty())
            return true;
        }
      }
    }
  }
  return false;
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
    return std::tuple{topology_illegal_routes,  mutual_illegal_routes, topology_violations,
                      topology_gate_violations, mutual_violations,     total_clearance_deficit_nm};
  }
};

[[nodiscard]] AdaptiveRegionalWindowAssessment
assess_adaptive_regional_window(const std::vector<WorkingRegionalSpline>& routes,
                                const std::vector<CachedRegionalSplineGeometry>* cache = nullptr,
                                RegionalRouteResult* metrics = nullptr) {
  const auto started = std::chrono::steady_clock::now();
  AdaptiveRegionalWindowAssessment result;
  std::vector<std::vector<SplineSample>> sampled(routes.size());
  std::vector<std::vector<SplineVector>> positions(routes.size());
  for (std::size_t route = 0U; route < routes.size(); ++route) {
    if (cache != nullptr && route < cache->size() &&
        !(*cache)[route].adaptive_samples.empty()) {
      sampled[route] = (*cache)[route].adaptive_samples;
      positions[route] = (*cache)[route].adaptive_positions;
      if (metrics != nullptr)
        ++metrics->spline_geometry_cache_hits;
    } else {
      sampled[route] = adaptive_tessellate_quintic_ph(
          routes[route].spline, Nanometres::from_mm(0.02), 10U, 512U);
      positions[route].reserve(sampled[route].size());
      for (const auto& sample : sampled[route]) {
        positions[route].push_back(sample.position);
      }
      if (metrics != nullptr)
        ++metrics->spline_geometry_cache_misses;
    }
  }
  for (std::size_t route = 0U; route < routes.size(); ++route) {
    bool topology_illegal = false;
    bool mutual_illegal = false;
    const auto broad = probe_regional_spline_samples(routes, route, positions[route], positions);
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
      if (broad.ok() && sample < broad.queries.size() && !broad.queries[sample].possible_conflict) {
        continue;
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
  if (metrics != nullptr) {
    metrics->spline_adaptive_assessment_us += static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() -
                                                              started)
            .count());
  }
  return result;
}

[[nodiscard]] RoutingComputeQuantizedCandidateResult rank_regional_interpolation_candidates(
    const std::vector<WorkingRegionalSpline>& routes, std::size_t changing_route,
    const std::vector<QuinticPHRoute>& candidates, std::size_t top_k) {
  RoutingComputeQuantizedCandidateInput input;
  if (changing_route >= routes.size() || candidates.empty() || top_k == 0U)
    return {{}, 0U, false, false, "invalid regional interpolation batch"};
  constexpr std::size_t sample_count = 48U;
  input.candidate_count = candidates.size();
  input.points_per_candidate = routes.size() * sample_count;
  input.quantization_nm = 1;
  input.clearance_nm = 0;
  input.numerical_guard_nm = 1'024;
  input.top_k = std::min(top_k, candidates.size());

  std::map<LayerId, std::uint32_t> layer_codes;
  std::map<NetId, std::uint32_t> net_codes;
  for (const auto& route : routes) {
    if (!layer_codes.contains(route.layer))
      layer_codes.emplace(route.layer, static_cast<std::uint32_t>(layer_codes.size()));
    if (!net_codes.contains(route.fragment.net))
      net_codes.emplace(route.fragment.net, static_cast<std::uint32_t>(net_codes.size()));
  }
  input.routes.reserve(routes.size());
  for (std::size_t route = 0U; route < routes.size(); ++route) {
    input.routes.push_back(RoutingComputeQuantizedRoute{
        static_cast<std::uint32_t>(route * sample_count),
        static_cast<std::uint32_t>(sample_count), layer_codes.at(routes[route].layer),
        net_codes.at(routes[route].fragment.net),
        routes[route].width.value() / 2 + routes[route].clearance.value()});
  }

  // One candidate-major buffer is consumed by one compute dispatch. Unchanged neighbouring
  // curves are evaluated once here and copied into each candidate slice; candidate ranking then
  // stays resident until the deterministic top-K readback.
  std::vector<std::vector<std::pair<long double, long double>>> fixed_positions(routes.size());
  for (std::size_t route = 0U; route < routes.size(); ++route) {
    if (route == changing_route)
      continue;
    auto& positions = fixed_positions[route];
    positions.reserve(sample_count);
    for (std::size_t sample = 0U; sample < sample_count; ++sample) {
      const auto parameter = static_cast<long double>(sample) /
                             static_cast<long double>(sample_count - 1U);
      const auto point = evaluate_quintic_ph(routes[route].spline, parameter);
      positions.emplace_back(point.x, point.y);
    }
  }
  input.candidate_points_nm.reserve(input.candidate_count * input.points_per_candidate);
  for (const auto& candidate : candidates) {
    for (std::size_t route = 0U; route < routes.size(); ++route) {
      if (route != changing_route) {
        input.candidate_points_nm.insert(input.candidate_points_nm.end(),
                                         fixed_positions[route].begin(),
                                         fixed_positions[route].end());
        continue;
      }
      for (std::size_t sample = 0U; sample < sample_count; ++sample) {
        const auto parameter = static_cast<long double>(sample) /
                               static_cast<long double>(sample_count - 1U);
        const auto point = evaluate_quintic_ph(candidate, parameter);
        input.candidate_points_nm.emplace_back(point.x, point.y);
      }
    }
  }
  return score_quantized_routing_candidates(input, true);
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
  struct FlatSegment {
    std::size_t route{0U};
    std::size_t segment{0U};
  };
  std::map<LayerId, std::uint32_t> layer_codes;
  std::vector<RoutingComputeSegment> compute_segments;
  std::vector<FlatSegment> flat_segments;
  for (std::size_t route = 0U; route < routes.size(); ++route) {
    if (!layer_codes.contains(routes[route].layer))
      layer_codes.emplace(routes[route].layer, static_cast<std::uint32_t>(layer_codes.size()));
    if (route >= centerlines.size())
      continue;
    for (std::size_t segment = 1U; segment < centerlines[route].size(); ++segment) {
      const auto radius = routes[route].width.value() / 2 + routes[route].clearance.value();
      compute_segments.push_back(RoutingComputeSegment{
          centerlines[route][segment - 1U].x.value(), centerlines[route][segment - 1U].y.value(),
          centerlines[route][segment].x.value(), centerlines[route][segment].y.value(), radius,
          layer_codes.at(routes[route].layer), static_cast<std::uint32_t>(route)});
      flat_segments.push_back(FlatSegment{route, segment - 1U});
    }
  }
  std::vector<std::pair<std::uint32_t, std::uint32_t>> compute_pairs;
  const auto maximum_broad_pairs = std::min<std::size_t>(
      2'000'000U, maximum_exact_checks > std::numeric_limits<std::size_t>::max() / 8U
                      ? std::numeric_limits<std::size_t>::max()
                      : std::max<std::size_t>(4'096U, maximum_exact_checks * 8U));
  bool broad_pair_budget_exhausted = false;
  for (std::size_t first = 0U; first < compute_segments.size(); ++first) {
    for (std::size_t second = first + 1U; second < compute_segments.size(); ++second) {
      if (flat_segments[first].route != flat_segments[second].route &&
          compute_segments[first].layer == compute_segments[second].layer &&
          routes[flat_segments[first].route].fragment.net !=
              routes[flat_segments[second].route].fragment.net) {
        if (compute_pairs.size() == maximum_broad_pairs) {
          compute_pairs.clear();
          broad_pair_budget_exhausted = true;
          break;
        }
        compute_pairs.emplace_back(static_cast<std::uint32_t>(first),
                                   static_cast<std::uint32_t>(second));
      }
    }
    if (broad_pair_budget_exhausted)
      break;
  }
  const auto broad_conflicts =
      broad_pair_budget_exhausted
          ? RoutingComputeSegmentPairResult{}
          : discover_routing_segment_conflicts(compute_segments, compute_pairs, 1'024);
  std::set<std::tuple<std::size_t, std::size_t, std::size_t, std::size_t>> possible_pairs;
  const auto use_broad_conflicts =
      !broad_pair_budget_exhausted && broad_conflicts.ok() &&
      broad_conflicts.possible_conflicts.size() == compute_pairs.size();
  if (use_broad_conflicts) {
    for (std::size_t pair = 0U; pair < compute_pairs.size(); ++pair) {
      if (broad_conflicts.possible_conflicts[pair] == 0U)
        continue;
      const auto& first = flat_segments[compute_pairs[pair].first];
      const auto& second = flat_segments[compute_pairs[pair].second];
      possible_pairs.emplace(first.route, second.route, first.segment, second.segment);
    }
  }
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
          if (use_broad_conflicts &&
              !possible_pairs.contains({first, second, first_segment - 1U, second_segment - 1U})) {
            continue;
          }
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

void publish_regional_spline_assignment_telemetry(const RegionalBundleAssignment& assignment,
                                                  RouterTelemetrySink* telemetry,
                                                  RouterTelemetryLevel minimum_level,
                                                  std::string_view lifecycle) {
  if (telemetry == nullptr || !telemetry->accepts(minimum_level)) {
    return;
  }
  for (const auto& realization : assignment.spline_realizations) {
    RouterTelemetryEvent route;
    route.kind = RouterTelemetryEventKind::route_candidate_selected;
    route.minimum_level = minimum_level;
    route.stable_id = "regional-spline:" + std::to_string(assignment.window_id) + ":" +
                      realization.fragment.net.value() + ":" +
                      std::to_string(realization.fragment.connection_index) + ":" +
                      std::to_string(realization.fragment.run_index) + ":" +
                      std::to_string(realization.fragment.fragment_index);
    route.group_id = "regional-window:" + std::to_string(assignment.window_id);
    route.net = realization.fragment.net;
    route.layer = realization.layer;
    route.route_revision = assignment.spline_candidate_rank + 1U;
    route.route_family = "regional_spline";
    route.route_lifecycle = std::string(lifecycle);
    route.route_width_nm =
        static_cast<std::uint64_t>(std::max<std::int64_t>(0, realization.width.value()));
    route.route_geometry = RouterTelemetryRouteGeometry::quintic_ph;
    route.spline_degree = 5U;
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
        std::string(lifecycle) +
        " regional quintic PH curve tessellation; diagnostic planning geometry; "
        "candidate_rank=" +
        std::to_string(assignment.spline_candidate_rank) +
        ", topology_legal=" + std::string(realization.topology_legal ? "yes" : "no") +
        ", endpoint_topology=" + std::string(realization.endpoint_topology_legal ? "yes" : "no") +
        ", interior_topology=" + std::string(realization.interior_topology_legal ? "yes" : "no") +
        ", gate_sequence=" + std::string(realization.topology_gate_sequence_legal ? "yes" : "no") +
        ", gate_count=" + std::to_string(realization.topology_gate_count) +
        ", topology_diagnostic=" + realization.topology_diagnostic +
        ", mutually_legal=" + std::string(realization.mutually_legal ? "yes" : "no") +
        ", complete-board commit required";
    static_cast<void>(telemetry->try_emit(std::move(route)));
  }
}

void realize_bundle_splines(const Board& board, RegionalRouteResult& result,
                            std::stop_token stop_token, RouterTelemetrySink* telemetry,
                            std::size_t maximum_fallback_waves = 1U) {
  const auto realization_started = std::chrono::steady_clock::now();
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
      if (rank != 0U) {
        candidate.spline_candidate_reserve.clear();
      }
      candidate_assignments.push_back(std::move(candidate));
    }
  }
  result.bundle_assignments = std::move(candidate_assignments);
  std::size_t assignment_index = 0U;
  for (auto& assignment : result.bundle_assignments) {
    ++assignment_index;
    if (!assignment.complete() || stop_token.stop_requested() ||
        (!assignment.joint_spline_candidate && !assignment.spline_realizations.empty())) {
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
      if (route_template.topology_signature.valid &&
          route_template.topology_centerline.size() >= 2U) {
        // An exact global NavMesh certificate remains a hard route-class constraint in regional
        // realization.  Reuse its funnel centreline as the quintic PH curve seed before lane/PNS
        // forces are allowed to act.
        seed = topology_locked_seed(route_template.topology_centerline);
        if (seed.size() == 2U) {
          seed.insert(
              seed.begin() + 1,
              Point{Nanometres{std::midpoint(seed.front().x.value(), seed.back().x.value())},
                    Nanometres{std::midpoint(seed.front().y.value(), seed.back().y.value())}});
        }
      } else if (!route_template.selected_global_topology &&
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
      auto spline = make_quintic_ph_route(seed);
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
                                             seed,
                                             std::move(topology_signature),
                                             std::move(cells),
                                             {},
                                             std::move(topology_links),
                                             route_template.channel_sequence,
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
    const auto prepare_route_topology_seed = [&](WorkingRegionalSpline& route) {
      route.topology_cell_index = prepare_regional_topology_cell_index(route.topology_cells);
      const auto configured_spacing = result.configured.navmesh_cell_pitch.value() > 0
                                          ? result.configured.navmesh_cell_pitch.value() / 4
                                          : Nanometres::from_mm(0.25).value();
      const auto sample_spacing = Nanometres{
          std::clamp<std::int64_t>(configured_spacing, 1, Nanometres::from_mm(0.25).value())};
      std::vector<Point> raw_topology_seed;
      raw_topology_seed.reserve(route.topology_seed.size());
      for (const auto point : route.topology_seed) {
        if (raw_topology_seed.empty() || raw_topology_seed.back() != point) {
          raw_topology_seed.push_back(point);
        }
      }
      const auto simplified_seed = string_pull_regional_spline_seed(route, sample_spacing);
      const auto select_topology_seed = [&](const std::vector<Point>& seed,
                                            std::optional<Nanometres> corner_guard = std::nullopt) {
        // PH Hermite interpolation has several regular algebraic branches, and one fixed nodal
        // derivative magnitude can still overshoot a perfectly valid funnel at a tight
        // entry/via turn. Search a small interpolation portfolio before changing topology. Short
        // tangents reduce that overshoot; chord-length parameters redistribute freedom on highly
        // unequal spans. Every trial keeps identical interpolation nodes and is accepted only by
        // the existing corridor, directed-gate, and shape certificates.
        constexpr std::array<std::pair<long double, long double>, 16U> interpolation_starts{
            {{0.5L, 1.0L},
             {1.0L, 1.0L},
             {0.5L, 0.75L},
             {1.0L, 0.75L},
             {0.5L, 0.5L},
             {1.0L, 0.5L},
             {0.5L, 0.45L},
             {1.0L, 0.45L},
             {0.5L, 0.4L},
             {1.0L, 0.4L},
             {0.5L, 0.35L},
             {1.0L, 0.35L},
             {0.5L, 0.3L},
             {1.0L, 0.3L},
             {0.5L, 0.25L},
             {1.0L, 0.25L}}};
        const auto corridor = regional_spline_convex_corridor(route);
        for (const auto [parameter_exponent, tangent_scale] : interpolation_starts) {
          PHInterpolationConfig interpolation;
          interpolation.parameter_exponent = parameter_exponent;
          interpolation.tangent_scale = tangent_scale;
          auto candidate = make_topology_tight_regional_spline(seed, corner_guard, interpolation);
          if (candidate && set_quintic_ph_corridor(*candidate, corridor) &&
              regional_spline_stays_in_topology(route, *candidate) &&
              certify_quintic_ph_shape(*candidate).legal) {
            route.spline = std::move(*candidate);
            return true;
          }
        }
        return false;
      };
      bool topology_seed_selected = select_topology_seed(simplified_seed);
      if (!topology_seed_selected) {
        // A single fixed corner guard is too coarse for a narrow portal and too conservative for
        // a broad one.  Tighten the rounded corner deterministically around the original funnel
        // vertex until the cubic remains in the selected directed gate sequence.  This is still
        // genuine cubic geometry: the fallback changes its local support, not the final route
        // into a polyline or an unchecked visual approximation.
        auto guard = sample_spacing.value();
        const auto maximum_guard_attempts =
            route.topology_signature.gates.size() <= 32U && raw_topology_seed.size() <= 64U ? 4U
                                                                                            : 1U;
        for (std::size_t attempt = 0U;
             attempt < maximum_guard_attempts && guard > 0 && !topology_seed_selected; ++attempt) {
          topology_seed_selected = select_topology_seed(raw_topology_seed, Nanometres{guard});
          guard /= 2;
        }
      }
      if (!topology_seed_selected) {
        topology_seed_selected = select_topology_seed(raw_topology_seed);
      }
      return topology_seed_selected;
    };
    // The discrete CSP orders variables by scarcity, not by physical run position.  Canonicalize
    // the selected realizations before building contiguous same-net chains; otherwise a later
    // fragment can be consumed first and permanently prevent its preceding absorbed stub from
    // moving the fixed boundary away from an endpoint collision.
    std::stable_sort(routes.begin(), routes.end(),
                     [](const auto& lhs, const auto& rhs) { return lhs.fragment < rhs.fragment; });
    // Fragment-local splines fix every sampled corner. Merge contiguous owned fragments from one
    // unchanged global run so only the outer endpoints remain fixed and internal coarse vertices
    // become local PH interpolation nodes inside the union topology corridor.
    std::vector<WorkingRegionalSpline> merged_routes;
    std::vector<bool> consumed(routes.size(), false);
    for (std::size_t first = 0U; first < routes.size(); ++first) {
      if (consumed[first]) {
        continue;
      }
      std::vector<std::size_t> chain{first};
      if (routes[first].owned && routes[first].emit_replacement && !routes[first].entry_via_site &&
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
            if (!rhs.owned || !rhs.emit_replacement || rhs.entry_via_site || rhs.exit_via_site ||
                lhs.fragment.net != rhs.fragment.net ||
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
        std::vector<Point> seed;
        std::vector<NavMeshTopologyGate> merged_gates;
        for (std::size_t member = 0U; member < chain.size(); ++member) {
          const auto index = chain[member];
          const auto& member_route = index == first ? merged : routes[index];
          merged.selected_global_topology =
              merged.selected_global_topology && member_route.selected_global_topology;
          if (member_route.topology_signature.valid) {
            merged_gates.insert(merged_gates.end(), member_route.topology_signature.gates.begin(),
                                member_route.topology_signature.gates.end());
          }
          if (index != first) {
            merged.topology_cells.insert(merged.topology_cells.end(),
                                         member_route.topology_cells.begin(),
                                         member_route.topology_cells.end());
            merged.topology_links.insert(merged.topology_links.end(),
                                         member_route.topology_links.begin(),
                                         member_route.topology_links.end());
            merged.channel_sequence.insert(merged.channel_sequence.end(),
                                           member_route.channel_sequence.begin(),
                                           member_route.channel_sequence.end());
            merged.lane_displacement.x += member_route.lane_displacement.x;
            merged.lane_displacement.y += member_route.lane_displacement.y;
          }
          for (const auto point : member_route.topology_seed) {
            if (seed.empty() || seed.back() != point) {
              seed.push_back(point);
            }
          }
          const auto endpoint = member_route.replacement_end;
          if (seed.empty() || seed.back() != endpoint) {
            seed.push_back(endpoint);
          }
        }
        const auto merged_signature =
            merged_gates.empty() ? NavMeshTopologySignature{}
                                 : make_navmesh_topology_signature(std::move(merged_gates));
        // Fragment merging is an optional degree-of-freedom reduction, never permission to drop
        // an obstacle-side decision.  A concatenated certificate must accept the complete merged
        // seed; otherwise retain the original fragments and their independently proved gates.
        // Legacy cell-only fragments carry no directed certificate, so they may still merge under
        // the existing union-containment guard without inventing a topology claim.
        const auto merged_topology_legal =
            !merged_signature.valid ||
            validate_navmesh_topology_signature(merged_signature, seed).preserved;
        bool merged_run = false;
        if (merged_topology_legal) {
          if (auto spline = make_quintic_ph_route(seed)) {
            merged.spline = std::move(*spline);
            merged.topology_seed = seed;
            merged.topology_signature = merged_signature;
            merged.replacement_end = routes[chain.back()].replacement_end;
            merged.merged_fragments = chain.size();
            merged.lane_displacement.x /= static_cast<long double>(chain.size());
            merged.lane_displacement.y /= static_cast<long double>(chain.size());
            for (std::size_t member = 1U; member < chain.size(); ++member) {
              consumed[chain[member]] = true;
            }
            ++result.spline_merged_runs;
            result.spline_merged_fragments += chain.size();
            merged_run = true;
          }
        }
        if (!merged_run) {
          merged = unmerged;
        } else {
          std::sort(merged.channel_sequence.begin(), merged.channel_sequence.end());
          merged.channel_sequence.erase(
              std::unique(merged.channel_sequence.begin(), merged.channel_sequence.end()),
              merged.channel_sequence.end());
        }
      }
      merged_routes.push_back(std::move(merged));
    }
    routes = std::move(merged_routes);
    bool topology_seed_failure = false;
    for (auto& route : routes) {
      if (route.topology_signature.valid) {
        ++result.spline_topology_gate_signatures;
        result.spline_topology_gates += route.topology_signature.gates.size();
      }
      if (!prepare_route_topology_seed(route)) {
        topology_seed_failure = true;
        break;
      }
    }
    if (topology_seed_failure) {
      assignment.stop_reason = RegionalBundleStopReason::spline_realization_failed;
      continue;
    }
    std::vector<std::vector<SplineVector>> route_samples(routes.size());
    std::vector<CachedRegionalSplineGeometry> geometry_cache(routes.size());
    std::map<std::pair<std::size_t, std::size_t>, bool> offset_pair_cache;
    const auto refresh_route_samples = [&](std::size_t index) {
      auto& cached = geometry_cache[index];
      cached.adaptive_samples = adaptive_tessellate_quintic_ph(
          routes[index].spline, Nanometres::from_mm(0.02), 10U, 512U);
      cached.adaptive_positions.clear();
      cached.adaptive_positions.reserve(cached.adaptive_samples.size());
      for (const auto& sample : cached.adaptive_samples)
        cached.adaptive_positions.push_back(sample.position);
      cached.offsets_by_required_twice_nm.clear();
      auto& prepared = route_samples[index];
      prepared = cached.adaptive_positions;
      std::erase_if(offset_pair_cache, [&](const auto& entry) {
        return entry.first.first == index || entry.first.second == index;
      });
      ++result.spline_geometry_cache_misses;
    };
    for (std::size_t index = 0U; index < routes.size(); ++index) {
      refresh_route_samples(index);
    }
    const auto offset_pair_intersects = [&](std::size_t first, std::size_t second) {
      const auto key = std::minmax(first, second);
      if (const auto found = offset_pair_cache.find(key); found != offset_pair_cache.end()) {
        ++result.spline_geometry_cache_hits;
        return found->second;
      }
      const auto intersects = regional_ph_envelope_boundaries_intersect(
          routes, key.first, key.second, &geometry_cache, &result);
      offset_pair_cache.emplace(key, intersects);
      return intersects;
    };
    // Select PH interpolation parameters jointly with the already assembled neighbouring routes.
    // The topology-only seed pass can find several shape-legal realizations of the same anchors;
    // choosing its first candidate independently may consume the clearance needed by the next
    // lane. Two deterministic coordinate sweeps keep topology/anchors fixed and minimize the
    // complete adaptive window objective before any elastic shove is attempted.
    const auto interpolation_baseline =
        assess_adaptive_regional_window(routes, &geometry_cache, &result);
    auto interpolation_current = interpolation_baseline;
    bool interpolation_window_improved = false;
    for (std::size_t sweep = 0U; sweep < 2U; ++sweep) {
      bool improved = false;
      for (std::size_t route_index = 0U; route_index < routes.size(); ++route_index) {
        if (routes[route_index].frozen_boundary) {
          continue;
        }
        std::vector<std::pair<long double, long double>> starts{
            {routes[route_index].spline.parameter_exponent,
             routes[route_index].spline.tangent_scale},
            {0.5L, 1.0L},
            {1.0L, 1.0L},
            {0.5L, 0.75L},
            {1.0L, 0.75L},
            {0.5L, 0.5L},
            {1.0L, 0.5L},
            {0.5L, 0.45L},
            {1.0L, 0.45L},
            {0.5L, 0.4L},
            {1.0L, 0.4L},
            {0.5L, 0.35L},
            {1.0L, 0.35L},
            {0.5L, 0.3L},
            {1.0L, 0.3L},
            {0.5L, 0.25L},
            {1.0L, 0.25L},
            {0.5L, 1.25L},
            {1.0L, 1.25L}};
        std::sort(starts.begin(), starts.end());
        starts.erase(std::unique(starts.begin(), starts.end()), starts.end());
        const auto anchors = [&]() {
          std::vector<Point> points;
          points.reserve(routes[route_index].spline.corridor_anchors.size());
          for (const auto anchor : routes[route_index].spline.corridor_anchors) {
            const auto point = quantize_regional_spline(anchor);
            if (points.empty() || points.back() != point) {
              points.push_back(point);
            }
          }
          return points;
        }();
        const auto corridor = regional_spline_convex_corridor(routes[route_index]);
        const auto original = routes[route_index].spline;
        auto best = original;
        auto best_assessment = interpolation_current;
        const auto offset_intersections = [&](std::size_t candidate_route) {
          std::size_t intersections = 0U;
          for (std::size_t other_index = 0U; other_index < routes.size(); ++other_index) {
            if (other_index != candidate_route &&
                offset_pair_intersects(candidate_route, other_index))
              ++intersections;
          }
          return intersections;
        };
        const auto original_offset_intersections = offset_intersections(route_index);
        auto best_offset_intersections = original_offset_intersections;
        std::vector<QuinticPHRoute> interpolation_candidates;
        interpolation_candidates.reserve(starts.size());
        for (const auto [parameter_exponent, tangent_scale] : starts) {
          ++result.spline_interpolation_portfolio_candidates;
          PHInterpolationConfig interpolation;
          interpolation.parameter_exponent = parameter_exponent;
          interpolation.tangent_scale = tangent_scale;
          auto candidate = make_quintic_ph_route(anchors, interpolation);
          if (!candidate || !set_quintic_ph_corridor(*candidate, corridor)) {
            continue;
          }
          interpolation_candidates.push_back(std::move(*candidate));
        }
        const auto rank_started = std::chrono::steady_clock::now();
        const auto ranking = rank_regional_interpolation_candidates(
            routes, route_index, interpolation_candidates, 2U);
        result.spline_interpolation_rank_us += static_cast<std::uint64_t>(
            std::chrono::duration_cast<std::chrono::microseconds>(
                std::chrono::steady_clock::now() - rank_started)
                .count());
        ++result.spline_interpolation_gpu_batches;
        result.spline_interpolation_gpu_fallbacks += ranking.used_gpu ? 0U : 1U;
        result.spline_interpolation_gpu_candidates += ranking.assessed_candidates;
        for (const auto& ranked : ranking.shortlist) {
          if (ranked.candidate >= interpolation_candidates.size())
            continue;
          const auto exact_started = std::chrono::steady_clock::now();
          ++result.spline_interpolation_exact_candidates;
          const auto& candidate = interpolation_candidates[ranked.candidate];
          if (!regional_spline_stays_in_topology(routes[route_index], candidate) ||
              !certify_quintic_ph_shape(candidate).legal) {
            result.spline_interpolation_exact_us += static_cast<std::uint64_t>(
                std::chrono::duration_cast<std::chrono::microseconds>(
                    std::chrono::steady_clock::now() - exact_started)
                    .count());
            continue;
          }
          routes[route_index].spline = candidate;
          refresh_route_samples(route_index);
          const auto assessment =
              assess_adaptive_regional_window(routes, &geometry_cache, &result);
          const auto candidate_offset_intersections = offset_intersections(route_index);
          result.spline_interpolation_portfolio_offset_rejections +=
              candidate_offset_intersections > 0U ? 1U : 0U;
          if (candidate_offset_intersections < best_offset_intersections ||
              (candidate_offset_intersections == best_offset_intersections &&
               assessment.objective() < best_assessment.objective())) {
            best = routes[route_index].spline;
            best_assessment = assessment;
            best_offset_intersections = candidate_offset_intersections;
          }
          result.spline_interpolation_exact_us += static_cast<std::uint64_t>(
              std::chrono::duration_cast<std::chrono::microseconds>(
                  std::chrono::steady_clock::now() - exact_started)
                  .count());
        }
        routes[route_index].spline = std::move(best);
        refresh_route_samples(route_index);
        if (best_offset_intersections < original_offset_intersections ||
            best_assessment.objective() < interpolation_current.objective()) {
          interpolation_current = best_assessment;
          ++result.spline_interpolation_portfolio_accepted;
          interpolation_window_improved = true;
          improved = true;
        }
      }
      if (!improved) {
        break;
      }
    }
    if (interpolation_window_improved) {
      ++result.spline_interpolation_portfolio_windows_improved;
    }
    const auto refit_after_shove = [&](std::vector<WorkingRegionalSpline>& candidate_routes,
                                       std::size_t route_index) {
      std::vector<std::vector<SplineVector>> candidate_samples(candidate_routes.size());
      for (std::size_t index = 0U; index < candidate_routes.size(); ++index) {
        const auto sampled = adaptive_tessellate_quintic_ph(candidate_routes[index].spline,
                                                            Nanometres::from_mm(0.02), 10U, 512U);
        candidate_samples[index].reserve(sampled.size());
        for (const auto& sample : sampled) {
          candidate_samples[index].push_back(sample.position);
        }
      }
      SplineTrajectoryOptimizationConfig config;
      config.minimum_clearance = Nanometres{0};
      config.control_step =
          Nanometres{std::max<std::int64_t>(1, result.configured.window_margin.value() / 10)};
      config.gradient_probe_step =
          Nanometres{std::max<std::int64_t>(1, config.control_step.value() / 5)};
      config.maximum_corridor_displacement = result.configured.window_margin;
      config.samples = 32U;
      config.maximum_iterations = 2U;
      config.maximum_candidate_checks = 64U;
      const auto evaluator = [&](SplineVector position) {
        return regional_spline_clearance(candidate_routes[route_index], position, candidate_routes,
                                         route_index, candidate_samples);
      };
      return optimize_quintic_ph_trajectory(candidate_routes[route_index].spline, config,
                                            evaluator);
    };
    if (std::any_of(route_samples.begin(), route_samples.end(),
                    [](const auto& samples) { return samples.size() < 2U; })) {
      assignment.stop_reason = RegionalBundleStopReason::spline_realization_failed;
      continue;
    }
    if (result.configured.enable_xpbd_relaxation && routes.size() > 1U) {
      const auto baseline = assess_adaptive_regional_window(routes, &geometry_cache, &result);
      // XPBD is a physical-conflict repair stage, not a second unconditional spline optimizer.
      // A topology-legal candidate with no mutual copper-envelope conflict has nothing for the
      // joint clearance solver to repair and will still pass through the ordinary continuous,
      // quantization and exact-DRC gates below. This guard is especially important for retained
      // fallback portfolios, where hundreds of individually legal alternatives may be inspected.
      if (baseline.mutual_illegal_routes == 0U) {
        ++result.xpbd_unconflicted_candidates_skipped;
      } else {
        // XPBD operates on straight particle links while the authoritative regional route is a
        // PH curve. Subdivide the quadratic preimage exactly at the same adaptive 0.02 mm samples
        // used by the CPU clearance gate before constructing particles. The links then form a
        // conservative-resolution proxy for the actual curved centreline instead of cutting
        // across long PH spans. A single straight PH span also receives a midpoint because its
        // two connection endpoints are fixed and otherwise offer no deformation freedom. Every
        // inserted point reproduces the identical curve until XPBD moves it.
        for (std::size_t route_index = 0U; route_index < routes.size(); ++route_index) {
          auto& spline = routes[route_index].spline;
          if (routes[route_index].frozen_boundary ||
              spline.control_points.size() >=
                  result.configured.max_xpbd_interpolation_nodes_per_route) {
            continue;
          }
          bool inserted = false;
          if (spline.control_points.size() == 2U) {
            inserted = insert_quintic_ph_interpolation_node(spline, 0.5L);
          }
          const auto samples = adaptive_tessellate_quintic_ph(
              spline, Nanometres::from_mm(0.02), 10U,
              result.configured.max_xpbd_interpolation_nodes_per_route);
          for (const auto& sample : samples) {
            if (spline.control_points.size() >=
                    result.configured.max_xpbd_interpolation_nodes_per_route ||
                !(sample.parameter > 0.0L && sample.parameter < 1.0L)) {
              continue;
            }
            inserted = insert_quintic_ph_interpolation_node(spline, sample.parameter) || inserted;
          }
          if (inserted) {
            refresh_route_samples(route_index);
          }
        }
        ++result.xpbd_windows_attempted;
        std::map<LayerId, std::uint32_t> layer_codes;
        std::map<NetId, std::uint32_t> route_collision_groups;
        std::vector<RoutingXpbdParticle> particles;
        std::vector<RoutingXpbdSegment> segments;
        std::vector<std::size_t> particle_route_indices;
        std::vector<std::pair<std::size_t, std::size_t>> route_particle_ranges;
        route_particle_ranges.reserve(routes.size());
        for (std::size_t route_index = 0U; route_index < routes.size(); ++route_index) {
          const auto& route = routes[route_index];
          const auto [layer, inserted] =
              layer_codes.try_emplace(route.layer, static_cast<std::uint32_t>(layer_codes.size()));
          static_cast<void>(inserted);
          const auto [collision_group, collision_group_inserted] =
              route_collision_groups.try_emplace(
                  route.fragment.net, static_cast<std::uint32_t>(route_collision_groups.size()));
          static_cast<void>(collision_group_inserted);
          const auto start = particles.size();
          const auto copper_radius = std::max<std::int64_t>(1, (route.width.value() + 1) / 2);
          for (std::size_t control = 0U; control < route.spline.control_points.size(); ++control) {
            const auto& point = route.spline.control_points[control];
            const auto endpoint =
                control == 0U || control + 1U == route.spline.control_points.size();
            const auto fixed = endpoint || route.frozen_boundary;
            std::uint32_t flags = endpoint ? routing_xpbd_endpoint : 0U;
            if ((control == 0U && route.entry_via_site) ||
                (control + 1U == route.spline.control_points.size() && route.exit_via_site)) {
              flags |= routing_xpbd_via;
            }
            particles.push_back(
                RoutingXpbdParticle{static_cast<std::int64_t>(std::llround(point.x)),
                                    static_cast<std::int64_t>(std::llround(point.y)), copper_radius,
                                    fixed ? 0.0L : 1.0L, collision_group->second, layer->second,
                                    flags, static_cast<std::int64_t>(std::llround(point.x)),
                                    static_cast<std::int64_t>(std::llround(point.y)),
                                    result.configured.window_margin.value()});
            particle_route_indices.push_back(route_index);
          }
          const auto finish = particles.size();
          route_particle_ranges.emplace_back(start, finish);
          for (std::size_t particle = start + 1U; particle < finish; ++particle) {
            const auto& first = particles[particle - 1U];
            const auto& second = particles[particle];
            const auto rest = static_cast<std::int64_t>(
                std::llround(std::hypotl(static_cast<long double>(second.x_nm - first.x_nm),
                                         static_cast<long double>(second.y_nm - first.y_nm))));
            segments.push_back(RoutingXpbdSegment{
                static_cast<std::uint32_t>(particle - 1U), static_cast<std::uint32_t>(particle),
                collision_group->second, layer->second, copper_radius,
                std::max<std::int64_t>(1, rest), route.clearance.value()});
          }
        }
        std::vector<LayerId> xpbd_layers(layer_codes.size());
        for (const auto& [layer, code] : layer_codes)
          xpbd_layers[code] = layer;
        std::vector<RoutingXpbdObstacleSegment> obstacles;
        bool obstacle_budget_exhausted = false;
        const auto add_obstacle_polygon = [&](const Polygon& polygon, std::uint32_t layer,
                                              std::uint32_t route, bool forbidden_inside = true) {
          if (polygon.vertices.size() < 2U || obstacle_budget_exhausted)
            return;
          long double twice_area = 0.0L;
          for (std::size_t edge = 0U; edge < polygon.vertices.size(); ++edge) {
            const auto first = polygon.vertices[edge];
            const auto second = polygon.vertices[(edge + 1U) % polygon.vertices.size()];
            twice_area += static_cast<long double>(first.x.value()) * second.y.value() -
                          static_cast<long double>(second.x.value()) * first.y.value();
          }
          const auto counter_clockwise = twice_area > 0.0L;
          for (std::size_t edge = 0U; edge < polygon.vertices.size(); ++edge) {
            const auto first = polygon.vertices[edge];
            const auto second = polygon.vertices[(edge + 1U) % polygon.vertices.size()];
            const auto margin = result.configured.window_margin.value();
            const auto minimum_x = std::min(first.x.value(), second.x.value());
            const auto maximum_x = std::max(first.x.value(), second.x.value());
            const auto minimum_y = std::min(first.y.value(), second.y.value());
            const auto maximum_y = std::max(first.y.value(), second.y.value());
            if (maximum_x < window->bounds.minimum.x.value() - margin ||
                minimum_x > window->bounds.maximum.x.value() + margin ||
                maximum_y < window->bounds.minimum.y.value() - margin ||
                minimum_y > window->bounds.maximum.y.value() + margin) {
              continue;
            }
            if (obstacles.size() == result.configured.max_xpbd_obstacle_segments_per_window) {
              obstacle_budget_exhausted = true;
              return;
            }
            const auto dx = second.x.value() - first.x.value();
            const auto dy = second.y.value() - first.y.value();
            auto outward_x = counter_clockwise ? dy : -dy;
            auto outward_y = counter_clockwise ? -dx : dx;
            if (!forbidden_inside) {
              outward_x = -outward_x;
              outward_y = -outward_y;
            }
            obstacles.push_back(RoutingXpbdObstacleSegment{first.x.value(), first.y.value(),
                                                           second.x.value(), second.y.value(), 0,
                                                           layer, route, outward_x, outward_y});
          }
        };
        for (std::size_t route_index = 0U;
             route_index < routes.size() && !obstacle_budget_exhausted; ++route_index) {
          const auto extracted = extract_polygonal_board_free_space(
              board, BoardFreeSpaceExtractionConfig{
                         routes[route_index].layer, routes[route_index].fragment.net, true, true,
                         !result.configured.treat_saved_zone_fills_as_refillable});
          if (!extracted.diagnostic.empty())
            continue;
          const auto layer = layer_codes.at(routes[route_index].layer);
          const auto collision_group = route_collision_groups.at(routes[route_index].fragment.net);
          for (const auto& cutout : extracted.domain.cutouts) {
            add_obstacle_polygon(cutout, layer, collision_group);
          }
          for (const auto& obstacle : extracted.domain.obstacles) {
            add_obstacle_polygon(obstacle, layer, collision_group);
          }
          for (const auto& fill : extracted.domain.filled_obstacles) {
            add_obstacle_polygon(fill.area, layer, collision_group);
            for (const auto& hole : fill.holes) {
              add_obstacle_polygon(hole, layer, collision_group, false);
            }
          }
        }
        result.xpbd_obstacle_segments += obstacles.size();
        result.xpbd_obstacle_segment_budget_exhaustions += obstacle_budget_exhausted ? 1U : 0U;
        auto xpbd_config = result.configured.xpbd;
        // Particle links are chords of the adaptively tessellated PH route.  The CPU mutual-
        // clearance probe reserves the tessellator's maximum chord deviation, so carry the same
        // bound into XPBD.  Without it a pair separated by less than the 0.02 mm tessellation
        // tolerance can appear collision-free to the GPU and then fail the continuous envelope
        // gate without receiving any deformation pressure.
        constexpr auto xpbd_chord_deviation_guard_nm = std::int64_t{20'000};
        xpbd_config.numerical_guard_nm =
            xpbd_config.numerical_guard_nm >
                    std::numeric_limits<std::int64_t>::max() - xpbd_chord_deviation_guard_nm
                ? std::numeric_limits<std::int64_t>::max()
                : xpbd_config.numerical_guard_nm + xpbd_chord_deviation_guard_nm;
        xpbd_config.record_iteration_metrics =
            telemetry != nullptr && telemetry->accepts(RouterTelemetryLevel::trace);
        if (result.configured.enable_xpbd_feedback && xpbd_config.pressure_width == 0U) {
          constexpr std::size_t pressure_dimension = 64U;
          const auto width = std::max<std::int64_t>(1, window->bounds.maximum.x.value() -
                                                           window->bounds.minimum.x.value());
          const auto height = std::max<std::int64_t>(1, window->bounds.maximum.y.value() -
                                                            window->bounds.minimum.y.value());
          xpbd_config.pressure_width = pressure_dimension;
          xpbd_config.pressure_height = pressure_dimension;
          xpbd_config.pressure_origin_x_nm = window->bounds.minimum.x.value();
          xpbd_config.pressure_origin_y_nm = window->bounds.minimum.y.value();
          xpbd_config.pressure_pitch_nm = std::max<std::int64_t>(
              1, std::max(width, height) / static_cast<std::int64_t>(pressure_dimension - 1U));
        }
        RoutingXpbdTerrainField xpbd_terrain;
        if (result.configured.enable_xpbd_feedback && !result.configured.xpbd_history.empty() &&
            xpbd_config.pressure_width > 1U && xpbd_config.pressure_height > 1U &&
            xpbd_config.pressure_pitch_nm > 0) {
          xpbd_terrain.width = xpbd_config.pressure_width;
          xpbd_terrain.height = xpbd_config.pressure_height;
          xpbd_terrain.layer_count = layer_codes.size();
          xpbd_terrain.origin_x_nm = xpbd_config.pressure_origin_x_nm;
          xpbd_terrain.origin_y_nm = xpbd_config.pressure_origin_y_nm;
          xpbd_terrain.pitch_nm = xpbd_config.pressure_pitch_nm;
          const auto terrain_plane = xpbd_terrain.width * xpbd_terrain.height;
          xpbd_terrain.pressure.assign(terrain_plane * xpbd_terrain.layer_count, 0.0F);
          for (const auto& history : result.configured.xpbd_history) {
            const auto& bounds = history.resource.bounds;
            if (bounds.maximum.x < window->bounds.minimum.x ||
                window->bounds.maximum.x < bounds.minimum.x ||
                bounds.maximum.y < window->bounds.minimum.y ||
                window->bounds.maximum.y < bounds.minimum.y ||
                (!history.resource.layer.value().empty() &&
                 !layer_codes.contains(history.resource.layer))) {
              continue;
            }
            const auto centre_x = bounds.minimum.x.value() +
                                  (bounds.maximum.x.value() - bounds.minimum.x.value()) / 2;
            const auto centre_y = bounds.minimum.y.value() +
                                  (bounds.maximum.y.value() - bounds.minimum.y.value()) / 2;
            const auto cell_x = (centre_x - xpbd_terrain.origin_x_nm) / xpbd_terrain.pitch_nm;
            const auto cell_y = (centre_y - xpbd_terrain.origin_y_nm) / xpbd_terrain.pitch_nm;
            const auto amplitude = static_cast<float>(std::min<long double>(
                1'000.0L, std::log1pl(static_cast<long double>(
                              std::max<std::uint64_t>(1U, history.pressure)))));
            const auto first_layer =
                history.resource.layer.value().empty()
                    ? 0U
                    : static_cast<std::size_t>(layer_codes.at(history.resource.layer));
            const auto last_layer = history.resource.layer.value().empty()
                                        ? xpbd_terrain.layer_count
                                        : first_layer + 1U;
            for (auto layer = first_layer; layer < last_layer; ++layer) {
              const auto layer_offset = layer * terrain_plane;
              for (std::int64_t dy = -2; dy <= 2; ++dy) {
                for (std::int64_t dx = -2; dx <= 2; ++dx) {
                  const auto x = cell_x + dx;
                  const auto y = cell_y + dy;
                  if (x < 0 || y < 0 || x >= static_cast<std::int64_t>(xpbd_terrain.width) ||
                      y >= static_cast<std::int64_t>(xpbd_terrain.height))
                    continue;
                  const auto distance_squared = static_cast<float>(dx * dx + dy * dy);
                  xpbd_terrain
                      .pressure[layer_offset + static_cast<std::size_t>(y) * xpbd_terrain.width +
                                static_cast<std::size_t>(x)] +=
                      amplitude * std::exp(-0.5F * distance_squared);
                }
              }
            }
          }
        }
        const auto relaxed =
            relax_routing_xpbd(particles, segments, obstacles, xpbd_terrain, xpbd_config);
        RegionalXpbdInvocation invocation;
        invocation.window_id = assignment.window_id;
        invocation.candidate_rank = assignment.spline_candidate_rank;
        invocation.route_count = routes.size();
        invocation.particle_count = particles.size();
        invocation.segment_count = segments.size();
        invocation.obstacle_segment_count = obstacles.size();
        invocation.terrain_cells = static_cast<std::size_t>(
            std::count_if(xpbd_terrain.pressure.begin(), xpbd_terrain.pressure.end(),
                          [](const auto value) { return value > 0.0F; }));
        invocation.terrain_layers = xpbd_terrain.pressure.empty() ? 0U : xpbd_terrain.layer_count;
        invocation.pressure_layers = relaxed.pressure_layer_count;
        invocation.candidate_pairs = relaxed.candidate_pairs;
        invocation.broadphase_cells = relaxed.broadphase_cells;
        invocation.broadphase_pair_emissions = relaxed.broadphase_pair_emissions;
        invocation.resident_input_buffer_reuses = relaxed.resident_input_buffer_reuses;
        invocation.unresolved_trace_pairs = relaxed.unresolved_trace_pairs;
        for (const auto& route : routes)
          invocation.nets.push_back(route.fragment.net);
        std::sort(invocation.nets.begin(), invocation.nets.end());
        invocation.nets.erase(std::unique(invocation.nets.begin(), invocation.nets.end()),
                              invocation.nets.end());
        invocation.constraints = relaxed.constraints;
        invocation.stress = relaxed.stress;
        for (std::size_t particle = 0U;
             particle < particles.size() && particle < relaxed.particle_stress_by_type.size();
             ++particle) {
          if (particle >= particle_route_indices.size() ||
              particle_route_indices[particle] >= routes.size())
            continue;
          const auto route_index = particle_route_indices[particle];
          const auto& net = routes[route_index].fragment.net;
          auto found = std::find_if(invocation.net_stress.begin(), invocation.net_stress.end(),
                                    [&](const auto& value) { return value.net == net; });
          if (found == invocation.net_stress.end()) {
            invocation.net_stress.push_back({net, {}, {}, 0.0L});
            found = std::prev(invocation.net_stress.end());
          }
          const auto& stress = relaxed.particle_stress_by_type[particle];
          found->stress += stress;
          if (stress.total_nm() > found->worst_stress_nm &&
              particle < relaxed.positions_nm.size()) {
            found->worst_stress_nm = stress.total_nm();
            found->worst_location = Point{Nanometres{relaxed.positions_nm[particle].first},
                                          Nanometres{relaxed.positions_nm[particle].second}};
          }
          for (const auto channel_id : routes[route_index].channel_sequence) {
            auto corridor =
                std::find_if(invocation.corridor_stress.begin(), invocation.corridor_stress.end(),
                             [&](const auto& value) { return value.channel_id == channel_id; });
            if (corridor == invocation.corridor_stress.end()) {
              invocation.corridor_stress.push_back({channel_id, 0.0L, 0.0L});
              corridor = std::prev(invocation.corridor_stress.end());
            }
            corridor->accumulated_stress_nm += stress.total_nm();
            corridor->peak_stress_nm = std::max(corridor->peak_stress_nm, stress.total_nm());
          }
        }
        std::sort(invocation.net_stress.begin(), invocation.net_stress.end(),
                  [](const auto& lhs, const auto& rhs) { return lhs.net < rhs.net; });
        std::sort(invocation.corridor_stress.begin(), invocation.corridor_stress.end(),
                  [](const auto& lhs, const auto& rhs) { return lhs.channel_id < rhs.channel_id; });
        invocation.timings = relaxed.timings;
        invocation.iterations = relaxed.iterations_executed;
        invocation.initial_violations = relaxed.initial.violations;
        invocation.final_violations = relaxed.final.violations;
        const auto metric_nm = [](long double value) {
          return Nanometres{static_cast<std::int64_t>(std::clamp<long double>(
              std::ceil(std::max(0.0L, value)), 0.0L,
              static_cast<long double>(std::numeric_limits<std::int64_t>::max())))};
        };
        const auto metric_sum = [](long double value) {
          return static_cast<std::uint64_t>(std::clamp<long double>(
              std::ceil(std::max(0.0L, value)), 0.0L,
              static_cast<long double>(std::numeric_limits<std::uint64_t>::max())));
        };
        invocation.initial_peak_residual = metric_nm(relaxed.initial.maximum_residual_nm);
        invocation.final_peak_residual = metric_nm(relaxed.final.maximum_residual_nm);
        invocation.initial_total_residual_nm = metric_sum(relaxed.initial.total_residual_nm);
        invocation.final_total_residual_nm = metric_sum(relaxed.final.total_residual_nm);
        invocation.used_gpu = relaxed.used_gpu;
        invocation.used_gpu_broadphase = relaxed.used_gpu_broadphase;
        invocation.broadphase_fallback = relaxed.broadphase_fallback;
        invocation.used_gpu_pressure_field = relaxed.used_gpu_pressure_field;
        invocation.cpu_fallback = relaxed.cpu_fallback;
        invocation.converged = relaxed.converged;
        invocation.stalled = relaxed.stalled;
        invocation.topology_escape_required = relaxed.topology_escape_required;
        invocation.pair_budget_exhausted = relaxed.pair_budget_exhausted;
        invocation.diagnostic = relaxed.diagnostic;
        result.xpbd_gpu_invocations += relaxed.used_gpu ? 1U : 0U;
        result.xpbd_cpu_fallbacks += relaxed.cpu_fallback ? 1U : 0U;
        result.xpbd_pressure_cells += relaxed.pressure_field.size();
        result.xpbd_terrain_cells_sampled += invocation.terrain_cells;
        result.xpbd_constraints += relaxed.constraints.total();
        result.xpbd_broadphase_cells += relaxed.broadphase_cells;
        result.xpbd_broadphase_pair_emissions += relaxed.broadphase_pair_emissions;
        result.xpbd_resident_input_buffer_reuses += relaxed.resident_input_buffer_reuses;
        result.xpbd_gpu_broadphase_invocations += relaxed.used_gpu_broadphase ? 1U : 0U;
        result.xpbd_broadphase_fallbacks += relaxed.broadphase_fallback ? 1U : 0U;
        result.xpbd_gpu_pressure_field_invocations += relaxed.used_gpu_pressure_field ? 1U : 0U;
        result.xpbd_unresolved_trace_pairs += relaxed.unresolved_trace_pairs;
        result.xpbd_stress += relaxed.stress;
        result.xpbd_initial_violations += relaxed.initial.violations;
        result.xpbd_final_violations += relaxed.final.violations;
        result.xpbd_peak_initial_residual =
            std::max(result.xpbd_peak_initial_residual, invocation.initial_peak_residual);
        result.xpbd_peak_final_residual =
            std::max(result.xpbd_peak_final_residual, invocation.final_peak_residual);
        result.xpbd_broadphase_us += relaxed.timings.broadphase_us;
        result.xpbd_constraint_generation_us += relaxed.timings.constraint_generation_us;
        result.xpbd_solve_us += relaxed.timings.solve_us;
        result.xpbd_stress_reduction_us += relaxed.timings.stress_reduction_us;
        result.xpbd_readback_us += relaxed.timings.readback_us;
        if (!relaxed.pressure_field.empty()) {
          result.xpbd_peak_pressure = std::max<long double>(
              result.xpbd_peak_pressure,
              *std::max_element(relaxed.pressure_field.begin(), relaxed.pressure_field.end()));
        }
        if (relaxed.ok() && relaxed.positions_nm.size() == particles.size()) {
          auto trial = routes;
          for (std::size_t route_index = 0U; route_index < trial.size(); ++route_index) {
            const auto [start, finish] = route_particle_ranges[route_index];
            for (std::size_t particle = start; particle < finish; ++particle) {
              const auto control = particle - start;
              trial[route_index].spline.control_points[control] =
                  SplineVector{static_cast<long double>(relaxed.positions_nm[particle].first),
                               static_cast<long double>(relaxed.positions_nm[particle].second)};
            }
            if (!rebuild_quintic_ph_route(trial[route_index].spline)) {
              trial.clear();
              break;
            }
          }
          if (trial.empty()) {
            invocation.diagnostic = "XPBD deformation could not be reconstructed as a PH chain";
            result.xpbd_invocations.push_back(std::move(invocation));
            continue;
          }
          const auto candidate = assess_adaptive_regional_window(trial);
          if (candidate.objective() < baseline.objective()) {
            routes = std::move(trial);
            invocation.accepted = true;
            ++result.xpbd_windows_improved;
            if ((baseline.owned_topology_illegal_routes > 0U ||
                 baseline.owned_mutual_illegal_routes > 0U) &&
                candidate.owned_topology_illegal_routes == 0U &&
                candidate.owned_mutual_illegal_routes == 0U) {
              ++result.xpbd_windows_legalized;
            }
            for (std::size_t index = 0U; index < routes.size(); ++index) {
              refresh_route_samples(index);
            }
          } else if (candidate.topology_illegal_routes > baseline.topology_illegal_routes ||
                     candidate.topology_gate_violations > baseline.topology_gate_violations) {
            ++result.xpbd_topology_rejections;
          }
          if (!invocation.accepted) {
            invocation.diagnostic +=
                (invocation.diagnostic.empty() ? "" : "; ") +
                std::string("PH reconstruction rejected: baseline topology=") +
                std::to_string(baseline.topology_illegal_routes) +
                ", mutual=" + std::to_string(baseline.mutual_illegal_routes) +
                "; candidate topology=" + std::to_string(candidate.topology_illegal_routes) +
                ", mutual=" + std::to_string(candidate.mutual_illegal_routes) +
                ", gate_violations=" + std::to_string(candidate.topology_gate_violations);
          }
        }
        if (result.configured.enable_xpbd_feedback &&
            (!invocation.accepted || !invocation.converged) && !relaxed.pressure_field.empty() &&
            relaxed.pressure_width > 0U && relaxed.pressure_height > 0U &&
            xpbd_config.pressure_pitch_nm > 0) {
          std::vector<std::size_t> pressure_cells(relaxed.pressure_field.size());
          std::iota(pressure_cells.begin(), pressure_cells.end(), 0U);
          std::stable_sort(pressure_cells.begin(), pressure_cells.end(),
                           [&](const auto lhs, const auto rhs) {
                             return relaxed.pressure_field[lhs] != relaxed.pressure_field[rhs]
                                        ? relaxed.pressure_field[lhs] > relaxed.pressure_field[rhs]
                                        : lhs < rhs;
                           });
          std::vector<NetId> pressure_nets;
          for (const auto& route : routes)
            pressure_nets.push_back(route.fragment.net);
          std::sort(pressure_nets.begin(), pressure_nets.end());
          pressure_nets.erase(std::unique(pressure_nets.begin(), pressure_nets.end()),
                              pressure_nets.end());
          const auto remaining_diagnoses = result.local_routability_diagnoses.size() <
                                                   result.configured.max_local_routability_diagnoses
                                               ? result.configured.max_local_routability_diagnoses -
                                                     result.local_routability_diagnoses.size()
                                               : 0U;
          const auto maximum_cells = std::min<std::size_t>(8U, remaining_diagnoses);
          for (std::size_t rank = 0U; rank < maximum_cells && rank < pressure_cells.size();
               ++rank) {
            const auto cell = pressure_cells[rank];
            const auto pressure = relaxed.pressure_field[cell];
            if (!(pressure > 0.0F))
              break;
            const auto pressure_plane = relaxed.pressure_width * relaxed.pressure_height;
            const auto layer_code = pressure_plane == 0U ? 0U : cell / pressure_plane;
            const auto local_cell = pressure_plane == 0U ? cell : cell % pressure_plane;
            const auto x = local_cell % relaxed.pressure_width;
            const auto y = local_cell / relaxed.pressure_width;
            const auto pressure_layer =
                layer_code < xpbd_layers.size() ? xpbd_layers[layer_code] : LayerId{};
            const auto minimum =
                Point{Nanometres{xpbd_config.pressure_origin_x_nm +
                                 static_cast<std::int64_t>(x) * xpbd_config.pressure_pitch_nm},
                      Nanometres{xpbd_config.pressure_origin_y_nm +
                                 static_cast<std::int64_t>(y) * xpbd_config.pressure_pitch_nm}};
            const auto maximum =
                Point{Nanometres{minimum.x.value() + xpbd_config.pressure_pitch_nm},
                      Nanometres{minimum.y.value() + xpbd_config.pressure_pitch_nm}};
            result.local_routability_diagnoses.push_back(RoutabilityDiagnosis{
                RoutabilityFailureKind::shove_deadlock, RoutabilityFailureStage::local_pns,
                RoutabilityResourceReference{
                    "regional-window:" + std::to_string(assignment.window_id) + ":xpbd-cell:" +
                        pressure_layer.value() + ":" + std::to_string(x) + ":" + std::to_string(y),
                    pressure_layer, geometry::AxisAlignedBounds{minimum, maximum}, pressure_nets},
                std::max<std::uint64_t>(1U, static_cast<std::uint64_t>(std::ceil(
                                                static_cast<long double>(pressure) * 1'000.0L))),
                invocation.final_peak_residual, std::nullopt, std::nullopt,
                RoutabilityRelaxation::change_corridor, "diffused XPBD constraint-pressure cell",
                false, true});
          }
        }
        if (telemetry != nullptr) {
          RouterTelemetryEvent event;
          event.kind = RouterTelemetryEventKind::search_summary;
          event.minimum_level = RouterTelemetryLevel::normal;
          event.stable_id = "regional-xpbd:" + std::to_string(assignment.window_id) + ":" +
                            std::to_string(assignment.spline_candidate_rank);
          event.group_id = "regional-window:" + std::to_string(assignment.window_id);
          event.detail =
              "joint XPBD relaxation: particles=" + std::to_string(invocation.particle_count) +
              ", segments=" + std::to_string(invocation.segment_count) +
              ", obstacles=" + std::to_string(invocation.obstacle_segment_count) +
              ", terrain_cells=" + std::to_string(invocation.terrain_cells) +
              ", terrain_layers=" + std::to_string(invocation.terrain_layers) +
              ", pressure_layers=" + std::to_string(invocation.pressure_layers) +
              ", candidate_pairs=" + std::to_string(invocation.candidate_pairs) +
              ", broadphase_cells=" + std::to_string(invocation.broadphase_cells) +
              ", gpu_broadphase=" + std::string(invocation.used_gpu_broadphase ? "yes" : "no") +
              ", broadphase_fallback=" +
              std::string(invocation.broadphase_fallback ? "yes" : "no") +
              ", resident_reuses=" + std::to_string(invocation.resident_input_buffer_reuses) +
              ", constraints=" + std::to_string(invocation.constraints.total()) +
              ", iterations=" + std::to_string(invocation.iterations) +
              ", initial_violations=" + std::to_string(invocation.initial_violations) +
              ", final_violations=" + std::to_string(invocation.final_violations) +
              ", initial_peak_nm=" + std::to_string(invocation.initial_peak_residual.value()) +
              ", final_peak_nm=" + std::to_string(invocation.final_peak_residual.value()) +
              ", solve_us=" + std::to_string(invocation.timings.solve_us) + ", trace_stress_nm=" +
              std::to_string(static_cast<double>(invocation.stress.trace_clearance_nm)) +
              ", obstacle_stress_nm=" +
              std::to_string(static_cast<double>(invocation.stress.obstacle_nm)) +
              ", unresolved_pairs=" + std::to_string(invocation.unresolved_trace_pairs) +
              ", gpu_pressure=" + std::string(invocation.used_gpu_pressure_field ? "yes" : "no") +
              ", stalled=" + std::string(invocation.stalled ? "yes" : "no") +
              ", accepted=" + std::string(invocation.accepted ? "yes" : "no") +
              ", backend=" + std::string(invocation.used_gpu ? "vulkan" : "cpu") +
              (invocation.diagnostic.empty() ? "" : ", diagnostic=" + invocation.diagnostic);
          static_cast<void>(telemetry->try_emit(std::move(event)));
          if (telemetry->accepts(RouterTelemetryLevel::trace)) {
            for (std::size_t iteration = 0U; iteration < relaxed.iteration_metrics.size();
                 ++iteration) {
              const auto& metrics = relaxed.iteration_metrics[iteration];
              RouterTelemetryEvent iteration_event;
              iteration_event.kind = RouterTelemetryEventKind::search_summary;
              iteration_event.minimum_level = RouterTelemetryLevel::trace;
              iteration_event.stable_id =
                  "regional-xpbd-iteration:" + std::to_string(assignment.window_id) + ":" +
                  std::to_string(assignment.spline_candidate_rank) + ":" +
                  std::to_string(iteration);
              iteration_event.group_id = "regional-window:" + std::to_string(assignment.window_id);
              iteration_event.iteration = iteration + 1U;
              iteration_event.detail =
                  "XPBD residual: violations=" + std::to_string(metrics.violations) +
                  ", peak_nm=" + std::to_string(static_cast<double>(metrics.maximum_residual_nm)) +
                  ", total_nm=" + std::to_string(static_cast<double>(metrics.total_residual_nm));
              static_cast<void>(telemetry->try_emit(std::move(iteration_event)));
            }
          }
        }
        result.xpbd_invocations.push_back(std::move(invocation));
      }
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
              evaluate_quintic_ph_differential(routes[index].spline, current.worst_parameter)
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
          const auto moved = shove_quintic_ph_interpolation_corridor(
              routes[index].spline, current.worst_parameter, normal, Nanometres{distance},
              result.configured.window_margin);
          if (moved == 0U) {
            break;
          }
          const auto refitted = refit_after_shove(routes, index);
          if (!refitted.accepted) {
            distance /= 2;
            continue;
          }
          refresh_route_samples(index);
          const auto candidate =
              assess_regional_spline_legality(routes, index, route_samples, joint_samples);
          if (better_regional_spline_legality(candidate, current)) {
            routes[index].control_moves += moved + refitted.accepted_control_moves;
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
        config.maximum_iterations = 2U;
        config.maximum_candidate_checks = 128U;
        const auto evaluator = [&](SplineVector position) {
          return regional_spline_clearance(routes[index], position, routes, index, route_samples);
        };
        const auto original = routes[index].spline;
        const auto optimized =
            optimize_quintic_ph_trajectory(routes[index].spline, config, evaluator);
        refresh_route_samples(index);
        if (!optimized.accepted ||
            !regional_spline_gate_sequence(routes[index], route_samples[index]).preserved) {
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
      const auto adaptive_baseline =
          assess_adaptive_regional_window(routes, &geometry_cache, &result);
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
          const auto tangent = evaluate_quintic_ph_differential(routes[route].spline,
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
            evaluate_quintic_ph(routes[route].spline, adaptive_current.worst_parameter);
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
          const auto samples = adaptive_tessellate_quintic_ph(routes[other].spline,
                                                              Nanometres::from_mm(0.02), 10U, 512U);
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
            // A symmetric half-and-half shove is not always feasible inside asymmetric
            // corridor cross sections.  Explore a tiny deterministic family of lane-preserving
            // force splits: one route may need to absorb the whole displacement while its
            // neighbour is pinned by a pad/via run-up, or the feasible solution may be biased
            // toward either side.  All trials retain the selected topology and are judged by the
            // full adaptive window before acceptance.
            struct ForceSplit {
              std::int64_t route_weight;
              std::int64_t other_weight;
            };
            constexpr std::array<ForceSplit, 5U> force_splits{
                {{1, 1}, {1, 0}, {0, 1}, {3, 1}, {1, 3}}};
            for (const auto split : force_splits) {
              if (adaptive_candidate_checks >=
                  result.configured.max_adaptive_pair_shove_checks_per_window) {
                break;
              }
              const auto movable_route = !routes[route].frozen_boundary;
              const auto movable_other = !routes[other].frozen_boundary;
              if ((!movable_route && split.route_weight > 0) ||
                  (!movable_other && split.other_weight > 0) ||
                  (split.route_weight == 0 && split.other_weight == 0)) {
                continue;
              }
              const auto total_weight = split.route_weight + split.other_weight;
              const auto weighted_distance = [&](std::int64_t weight) {
                return weight == 0 ? 0
                                   : std::max<std::int64_t>(
                                         1, (distance * weight + total_weight - 1) / total_weight);
              };
              ++adaptive_candidate_checks;
              ++result.spline_adaptive_pair_shove_attempts;
              auto trial = routes;
              std::size_t route_moves = 0U;
              std::size_t other_moves = 0U;
              if (split.route_weight > 0) {
                route_moves = shove_quintic_ph_interpolation_corridor(
                    trial[route].spline, adaptive_current.worst_parameter, normal,
                    Nanometres{weighted_distance(split.route_weight)},
                    result.configured.window_margin);
              }
              if (split.other_weight > 0) {
                other_moves = shove_quintic_ph_interpolation_corridor(
                    trial[other].spline, other_candidate.parameter,
                    SplineVector{-normal.x, -normal.y},
                    Nanometres{weighted_distance(split.other_weight)},
                    result.configured.window_margin);
              }
              if ((split.route_weight > 0 && route_moves == 0U) ||
                  (split.other_weight > 0 && other_moves == 0U)) {
                continue;
              }
              const auto route_refit =
                  split.route_weight > 0
                      ? refit_after_shove(trial, route)
                      : SplineTrajectoryOptimizationResult{.accepted = true, .legal = true};
              const auto other_refit =
                  split.other_weight > 0
                      ? refit_after_shove(trial, other)
                      : SplineTrajectoryOptimizationResult{.accepted = true, .legal = true};
              if (!route_refit.accepted || !other_refit.accepted) {
                continue;
              }
              const auto assessment = assess_adaptive_regional_window(trial);
              if (assessment.objective() < adaptive_current.objective() &&
                  (!best_assessment || assessment.objective() < best_assessment->objective())) {
                trial[route].control_moves += route_moves + route_refit.accepted_control_moves;
                trial[other].control_moves += other_moves + other_refit.accepted_control_moves;
                best_routes = std::move(trial);
                best_assessment = assessment;
              }
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
        final_route_samples[index] = adaptive_tessellate_quintic_ph(
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
    const auto continuous_before_quantization =
        assess_adaptive_regional_window(routes, &geometry_cache, &result);
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
        const auto first_route = collision.first_route;
        const auto second_route = collision.second_route;
        const auto first_segment = collision.first_segment;
        const auto second_segment = collision.second_segment;
        if (first_route >= routes.size() || second_route >= routes.size() ||
            first_route >= final_route_samples.size() ||
            second_route >= final_route_samples.size() ||
            first_segment + 1U >= final_route_samples[first_route].size() ||
            second_segment + 1U >= final_route_samples[second_route].size()) {
          break;
        }
        auto normal = collision.first_away_normal;
        const auto first_parameter =
            (final_route_samples[first_route][first_segment].parameter +
             final_route_samples[first_route][first_segment + 1U].parameter) /
            2.0L;
        const auto second_parameter =
            (final_route_samples[second_route][second_segment].parameter +
             final_route_samples[second_route][second_segment + 1U].parameter) /
            2.0L;
        if (!(std::hypotl(normal.x, normal.y) > 0.0L)) {
          const auto tangent =
              evaluate_quintic_ph_differential(routes[first_route].spline, first_parameter).tangent;
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
        const auto distance = std::max<std::int64_t>(
            1, static_cast<std::int64_t>(std::ceil(collision.deficit_nm + 2.0L)));
        // Materialization is itself a bounded local solver. A single curve may have no legal
        // quantized displacement, although the two colliding splines have a legal joint move.
        // Keep the same topology/corridor guards as ordinary PNS and accept only a strict exact
        // copper-envelope improvement.
        const auto original_routes = routes;
        std::optional<std::vector<WorkingRegionalSpline>> best_routes;
        std::optional<QuantizedRegionalAssessment> best_quantized;
        std::size_t best_first_moves = 0U;
        std::size_t best_second_moves = 0U;
        struct QuantizedTrial {
          std::vector<WorkingRegionalSpline> routes;
          std::size_t first_moves{0U};
          std::size_t second_moves{0U};
        };
        std::vector<QuantizedTrial> trials;
        const auto trial_limit =
            std::clamp(result.configured.max_quantization_feedback_candidates_per_iteration,
                       std::size_t{1U}, std::size_t{128U});
        // Coordinated elastic perturbations span lateral split and displacement magnitude. The
        // complete batch is scored after tessellation and KiCad-coordinate quantization; only a
        // small deterministic shortlist pays the exact regional copper assessment.
        for (std::size_t trial_index = 0U; trial_index < trial_limit; ++trial_index) {
          const auto split_step = trial_index % 9U;
          const auto scale_step = trial_index / 9U;
          const auto first_weight = static_cast<std::int64_t>(split_step);
          const auto second_weight = static_cast<std::int64_t>(8U - split_step);
          const auto scaled_distance = std::max<std::int64_t>(
              1, (distance * static_cast<std::int64_t>(2U + scale_step) + 1) / 2);
          const auto first_movable = !original_routes[first_route].frozen_boundary;
          const auto second_movable = !original_routes[second_route].frozen_boundary;
          if ((!first_movable && first_weight > 0) || (!second_movable && second_weight > 0)) {
            continue;
          }
          constexpr std::int64_t total_weight = 8;
          const auto weighted_distance = [&](std::int64_t weight) {
            return weight == 0
                       ? 0
                       : std::max<std::int64_t>(1, (scaled_distance * weight + total_weight - 1) /
                                                       total_weight);
          };
          routes = original_routes;
          std::size_t first_moves = 0U;
          std::size_t second_moves = 0U;
          if (first_weight > 0) {
            first_moves = shove_quintic_ph_interpolation_corridor(
                routes[first_route].spline, first_parameter, normal,
                Nanometres{weighted_distance(first_weight)}, result.configured.window_margin);
          }
          if (second_weight > 0) {
            second_moves = shove_quintic_ph_interpolation_corridor(
                routes[second_route].spline, second_parameter, SplineVector{-normal.x, -normal.y},
                Nanometres{weighted_distance(second_weight)}, result.configured.window_margin);
          }
          if ((first_weight > 0 && first_moves == 0U) ||
              (second_weight > 0 && second_moves == 0U)) {
            continue;
          }
          const auto first_refit =
              first_weight > 0
                  ? refit_after_shove(routes, first_route)
                  : SplineTrajectoryOptimizationResult{.accepted = true, .legal = true};
          const auto second_refit =
              second_weight > 0
                  ? refit_after_shove(routes, second_route)
                  : SplineTrajectoryOptimizationResult{.accepted = true, .legal = true};
          if (!first_refit.accepted || !second_refit.accepted) {
            continue;
          }
          ++result.quantization_feedback_attempts;
          trials.push_back(QuantizedTrial{routes, first_moves + first_refit.accepted_control_moves,
                                          second_moves + second_refit.accepted_control_moves});
        }
        std::vector<std::size_t> exact_trial_order;
        std::size_t accelerated_shortlist_size = 0U;
        if (!trials.empty()) {
          constexpr std::size_t samples_per_route = 64U;
          RoutingComputeQuantizedCandidateInput compute;
          compute.candidate_count = trials.size();
          compute.points_per_candidate = routes.size() * samples_per_route;
          compute.quantization_nm = 1;
          compute.top_k = std::min<std::size_t>(8U, trials.size());
          compute.numerical_guard_nm = 2;
          std::map<LayerId, std::uint32_t> compute_layers;
          std::map<NetId, std::uint32_t> compute_nets;
          for (std::size_t route_index = 0U; route_index < routes.size(); ++route_index) {
            const auto [layer, inserted_layer] = compute_layers.try_emplace(
                routes[route_index].layer, static_cast<std::uint32_t>(compute_layers.size()));
            static_cast<void>(inserted_layer);
            const auto [net, inserted_net] = compute_nets.try_emplace(
                routes[route_index].fragment.net, static_cast<std::uint32_t>(compute_nets.size()));
            static_cast<void>(inserted_net);
            compute.clearance_nm =
                std::max(compute.clearance_nm, routes[route_index].clearance.value());
            compute.routes.push_back(RoutingComputeQuantizedRoute{
                static_cast<std::uint32_t>(route_index * samples_per_route),
                static_cast<std::uint32_t>(samples_per_route), layer->second, net->second,
                std::max<std::int64_t>(1, routes[route_index].width.value() / 2)});
          }
          for (const auto& trial : trials) {
            for (const auto& trial_route : trial.routes) {
              for (std::size_t sample = 0U; sample < samples_per_route; ++sample) {
                const auto parameter = static_cast<long double>(sample) /
                                       static_cast<long double>(samples_per_route - 1U);
                const auto point = evaluate_quintic_ph(trial_route.spline, parameter);
                compute.candidate_points_nm.emplace_back(point.x, point.y);
              }
            }
          }
          const auto ranked = score_quantized_routing_candidates(compute, true);
          if (ranked.ok())
            for (const auto& candidate : ranked.shortlist)
              exact_trial_order.push_back(candidate.candidate);
          accelerated_shortlist_size = exact_trial_order.size();
          for (std::size_t candidate = 0U; candidate < trials.size(); ++candidate)
            if (std::find(exact_trial_order.begin(), exact_trial_order.end(), candidate) ==
                exact_trial_order.end())
              exact_trial_order.push_back(candidate);
        }
        if (exact_trial_order.empty()) {
          exact_trial_order.resize(trials.size());
          std::iota(exact_trial_order.begin(), exact_trial_order.end(), 0U);
        }
        for (std::size_t ranked_index = 0U; ranked_index < exact_trial_order.size();
             ++ranked_index) {
          if (ranked_index == accelerated_shortlist_size && best_quantized)
            break;
          const auto trial_index = exact_trial_order[ranked_index];
          if (trial_index >= trials.size())
            continue;
          routes = trials[trial_index].routes;
          refresh_final_samples();
          const auto continuous_candidate =
              assess_adaptive_regional_window(routes, &geometry_cache, &result);
          auto quantized_candidate = assess_quantized_regional_window(
              routes, quantized_route_samples,
              result.configured.max_quantized_segment_pair_checks_per_window);
          result.quantized_segment_pair_checks += quantized_candidate.exact_checks;
          quantization_budget_seen =
              quantization_budget_seen || quantized_candidate.budget_exhausted;
          const auto continuous_candidate_legal =
              continuous_candidate.owned_topology_illegal_routes == 0U &&
              continuous_candidate.owned_mutual_illegal_routes == 0U;
          if (!quantized_candidate.budget_exhausted && continuous_candidate_legal &&
              quantized_candidate.conflicts < quantized_assessment.conflicts &&
              (!best_quantized || quantized_candidate.conflicts < best_quantized->conflicts)) {
            best_routes = routes;
            best_quantized = std::move(quantized_candidate);
            best_first_moves = trials[trial_index].first_moves;
            best_second_moves = trials[trial_index].second_moves;
          }
        }
        if (!best_routes || !best_quantized) {
          routes = original_routes;
          refresh_final_samples();
          break;
        }
        routes = std::move(*best_routes);
        routes[first_route].control_moves += best_first_moves;
        routes[second_route].control_moves += best_second_moves;
        quantized_assessment = std::move(*best_quantized);
        ++result.quantization_feedback_accepted;
        if (quantized_assessment.conflicts == 0U) {
          break;
        }
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
      const auto ph_controls = quintic_ph_bezier_controls(routes[index].spline);
      realized.control_points.reserve(ph_controls.size());
      realized.merged_fragments = routes[index].merged_fragments;
      for (const auto& control : ph_controls) {
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
      realized.topology_diagnostic = gate_sequence.diagnostic;
      realized.topology_legal = endpoints_topology_legal && interior_topology_legal &&
                                realized.topology_gate_sequence_legal;
      realized.endpoint_topology_legal = endpoints_topology_legal;
      realized.interior_topology_legal = interior_topology_legal;
      const auto final_clearance_evaluator = [&](SplineVector position) {
        return regional_spline_clearance(routes[index], position, routes, index, route_samples);
      };
      const auto continuous_envelope = certify_quintic_ph_continuous_envelope(
          routes[index].spline, Nanometres{0}, final_clearance_evaluator);
      result.spline_continuous_hulls_checked += continuous_envelope.hulls_checked;
      result.spline_continuous_envelope_rejections += continuous_envelope.legal ? 0U : 1U;
      result.spline_continuous_budget_exhaustions += continuous_envelope.budget_exhausted ? 1U : 0U;
      // Rational PH offset intersection is the exact transverse-crossing closure for an
      // otherwise legal continuous envelope.  It cannot rescue an envelope that has already
      // failed, so do not pay its recursive root/isolation cost for that rejected candidate.
      bool rational_offset_intersection = false;
      if (continuous_envelope.legal) {
        for (std::size_t other_index = 0U; other_index < routes.size(); ++other_index) {
          if (other_index != index && offset_pair_intersects(index, other_index)) {
            rational_offset_intersection = true;
            break;
          }
        }
      }
      realized.mutually_legal = continuous_envelope.legal && !rational_offset_intersection;
      realized.quantized_mutually_legal = !quantized_assessment.budget_exhausted &&
                                          index < quantized_assessment.route_legal.size() &&
                                          quantized_assessment.route_legal[index];
      const auto shape = certify_quintic_ph_shape(routes[index].spline);
      realized.shape_legal = shape.legal;
      realized.shape_sample_budget_exhausted = shape.sample_budget_exhausted;
      realized.shape_self_intersections = shape.self_intersections;
      realized.shape_cusps = shape.cusps;
      realized.shape_control_station_reversals = shape.control_station_reversals;
      realized.shape_station_reversals = shape.station_reversals;
      realized.shape_excess_turn_violations = shape.excess_turn_violations;
      realized.shape_unintended_inflections = shape.unintended_inflections;
      realized.shape_total_turn_radians = shape.total_turn_radians;
      realized.shape_anchor_turn_radians = shape.anchor_turn_radians;
      realized.interpolation_parameter_exponent = routes[index].spline.parameter_exponent;
      realized.interpolation_tangent_scale = routes[index].spline.tangent_scale;
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
                  realized.shape_legal && realized.mutually_legal &&
                  realized.quantized_mutually_legal;
      if (routes[index].emit_replacement) {
        assignment.spline_realizations.push_back(std::move(realized));
      }
    }
    if (!all_legal) {
      assignment.stop_reason = RegionalBundleStopReason::spline_realization_failed;
    }
    // A dense window may spend minutes evaluating its bounded continuous portfolio. Publish the
    // completed candidate now so trace/live viewers can inspect the real PH Bezier controls even if
    // a later candidate is cancelled or the caller's time budget expires. This remains diagnostic,
    // provisional state; final selected geometry is republished below at normal level.
    publish_regional_spline_assignment_telemetry(assignment, telemetry,
                                                 RouterTelemetryLevel::diagnostic, "candidate");
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
    const auto legal_candidates = static_cast<std::size_t>(
        std::count_if(result.bundle_assignments.begin() + static_cast<std::ptrdiff_t>(index),
                      result.bundle_assignments.begin() + static_cast<std::ptrdiff_t>(end),
                      [](const auto& candidate) { return candidate.complete(); }));
    result.joint_spline_candidates_legal += legal_candidates;
    struct SplineFailureProfile {
      std::size_t short_routes{0U};
      std::size_t topology{0U};
      std::size_t shape{0U};
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
              [](const auto& realization) { return !realization.shape_legal; })),
          static_cast<std::size_t>(std::count_if(
              candidate.spline_realizations.begin(), candidate.spline_realizations.end(),
              [](const auto& realization) { return !realization.mutually_legal; })),
          static_cast<std::size_t>(std::count_if(
              candidate.spline_realizations.begin(), candidate.spline_realizations.end(),
              [](const auto& realization) { return !realization.quantized_mutually_legal; }))};
    };
    const auto initial_failure = failure_profile(result.bundle_assignments[index]);
    auto minimum_failure = initial_failure;
    std::vector<RegionalSplineCandidateSummary> candidate_summaries;
    candidate_summaries.reserve(end - index);
    for (auto candidate = index; candidate < end; ++candidate) {
      const auto profile = failure_profile(result.bundle_assignments[candidate]);
      candidate_summaries.push_back(RegionalSplineCandidateSummary{
          result.bundle_assignments[candidate].spline_candidate_rank,
          result.bundle_assignments[candidate].selected_templates, profile.short_routes,
          profile.topology, profile.shape, profile.mutual, profile.quantization,
          result.bundle_assignments[candidate].complete()});
    }
    for (auto candidate = index + 1U; candidate < end; ++candidate) {
      const auto profile = failure_profile(result.bundle_assignments[candidate]);
      minimum_failure.short_routes = std::min(minimum_failure.short_routes, profile.short_routes);
      minimum_failure.topology = std::min(minimum_failure.topology, profile.topology);
      minimum_failure.shape = std::min(minimum_failure.shape, profile.shape);
      minimum_failure.mutual = std::min(minimum_failure.mutual, profile.mutual);
      minimum_failure.quantization = std::min(minimum_failure.quantization, profile.quantization);
    }
    const auto failure_kind = [&](const RegionalBundleAssignment& candidate) {
      const auto profile = failure_profile(candidate);
      if (candidate.spline_realizations.empty() || profile.short_routes > 0U ||
          profile.shape > 0U) {
        return RoutabilityFailureKind::curvature_infeasible;
      }
      if (profile.quantization > 0U && profile.topology == 0U && profile.mutual == 0U) {
        return RoutabilityFailureKind::quantization_clearance;
      }
      return profile.mutual >= profile.topology ? RoutabilityFailureKind::shove_deadlock
                                                : RoutabilityFailureKind::topological_trap;
    };
    const auto failure_count = [&](const RegionalBundleAssignment& candidate,
                                   RoutabilityFailureKind kind) {
      const auto profile = failure_profile(candidate);
      switch (kind) {
      case RoutabilityFailureKind::curvature_infeasible:
        return profile.short_routes + profile.shape;
      case RoutabilityFailureKind::shove_deadlock:
        return profile.mutual;
      case RoutabilityFailureKind::topological_trap:
        return profile.topology;
      case RoutabilityFailureKind::quantization_clearance:
        return profile.quantization;
      default:
        return profile.topology + profile.shape + profile.mutual + profile.quantization +
               profile.short_routes;
      }
    };
    auto selected = index;
    bool found_legal = false;
    const auto legal_realization_objective = [](const RegionalBundleAssignment& candidate) {
      std::size_t control_moves = 0U;
      std::size_t optimization_sweeps = 0U;
      long double centerline_length_nm = 0.0L;
      for (const auto& realization : candidate.spline_realizations) {
        control_moves += realization.accepted_control_moves;
        optimization_sweeps += realization.optimization_sweeps;
        for (std::size_t point = 1U; point < realization.centerline.size(); ++point) {
          centerline_length_nm += geometry::point_distance_nm(realization.centerline[point - 1U],
                                                              realization.centerline[point]);
        }
      }
      // Every candidate reaching this comparator already passed the same exact topology,
      // continuous-envelope and quantized-envelope gates. Prefer the solution that required the
      // least elastic deformation before considering physical length and original portfolio rank.
      // This prevents an aggressively polished early topology from hiding a cleaner lane/layer/via
      // alternative that the regional solver deliberately retained.
      return std::tuple{control_moves,
                        optimization_sweeps,
                        centerline_length_nm,
                        candidate.selected_template_conflicts,
                        candidate.selected_residual_conflicts,
                        candidate.spline_candidate_rank};
    };
    for (auto candidate = index; candidate < end; ++candidate) {
      if (result.bundle_assignments[candidate].complete() &&
          (!found_legal || legal_realization_objective(result.bundle_assignments[candidate]) <
                               legal_realization_objective(result.bundle_assignments[selected]))) {
        selected = candidate;
        found_legal = true;
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
                          profile.topology + profile.shape + profile.mutual + profile.quantization,
                          profile.topology,
                          profile.shape,
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
    const auto diagnosed_kind = failure_kind(result.bundle_assignments[selected]);
    const auto selected_profile = failure_profile(result.bundle_assignments[selected]);
    if (!result.bundle_assignments[index].complete()) {
      const auto window =
          std::find_if(result.windows.begin(), result.windows.end(),
                       [&](const auto& candidate) { return candidate.id == window_id; });
      const auto selected_improves_diagnosis =
          selected != index &&
          (result.bundle_assignments[selected].complete() ||
           failure_count(result.bundle_assignments[selected], diagnosed_kind) <
               failure_count(result.bundle_assignments[index], diagnosed_kind));
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
        const auto severity =
            diagnosed_kind == RoutabilityFailureKind::quantization_clearance
                ? std::max<std::size_t>(1U, selected_profile.quantization)
            : diagnosed_kind == RoutabilityFailureKind::curvature_infeasible
                ? std::max<std::size_t>(1U, selected_profile.short_routes + selected_profile.shape)
            : diagnosed_kind == RoutabilityFailureKind::shove_deadlock
                ? std::max<std::size_t>(1U, selected_profile.mutual)
                : std::max<std::size_t>(1U, selected_profile.topology);
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
            "selected spline/PNS realization diagnosed " + std::string(to_string(diagnosed_kind)) +
                "; selected state has realizations=" +
                std::to_string(result.bundle_assignments[selected].spline_realizations.size()) +
                ", topology=" + std::to_string(selected_profile.topology) +
                ", shape=" + std::to_string(selected_profile.shape) +
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
    result.bundle_assignments[selected].spline_candidates_evaluated = end - index;
    result.bundle_assignments[selected].spline_candidates_legal = legal_candidates;
    result.bundle_assignments[selected].spline_candidate_min_topology_rejections =
        minimum_failure.topology;
    result.bundle_assignments[selected].spline_candidate_min_shape_rejections =
        minimum_failure.shape;
    result.bundle_assignments[selected].spline_candidate_min_mutual_clearance_rejections =
        minimum_failure.mutual;
    result.bundle_assignments[selected].spline_candidate_min_quantized_clearance_rejections =
        minimum_failure.quantization;
    result.bundle_assignments[selected].spline_candidate_summaries = std::move(candidate_summaries);
    if (selected != index && result.bundle_assignments[selected].spline_candidate_reserve.empty()) {
      result.bundle_assignments[selected].spline_candidate_reserve =
          std::move(result.bundle_assignments[index].spline_candidate_reserve);
    }
    result.bundle_assignments[selected].joint_spline_candidate = false;
    selected_assignments.push_back(std::move(result.bundle_assignments[selected]));
    index = end;
  }
  result.bundle_assignments = std::move(selected_assignments);

  if (maximum_fallback_waves > 0U && !stop_token.stop_requested()) {
    std::vector<std::size_t> promoted_windows;
    std::map<std::size_t, std::size_t> promoted_wave;
    for (auto& assignment : result.bundle_assignments) {
      if (assignment.complete() || assignment.spline_candidate_reserve.empty() ||
          assignment.spline_fallback_waves >= maximum_fallback_waves) {
        continue;
      }
      std::set<RegionalRouteFragmentId> implicated_fragments;
      bool topology_failure = false;
      for (const auto& realization : assignment.spline_realizations) {
        if (!realization.topology_legal || !realization.mutually_legal ||
            !realization.quantized_mutually_legal || realization.centerline.size() < 2U) {
          implicated_fragments.insert(realization.fragment);
        }
        topology_failure = topology_failure || !realization.topology_legal;
      }
      // Failure can occur before even one continuous realization is published. In that case the
      // selected templates are the only exact attribution available. Mark their fragments as
      // implicated so corridor alternatives remain eligible; the previous via/lane-only filter
      // left hundreds of useful retained states permanently unreachable.
      if (assignment.spline_realizations.empty()) {
        for (const auto selected_template : assignment.selected_templates) {
          if (selected_template < assignment.templates.size()) {
            implicated_fragments.insert(assignment.templates[selected_template].fragment);
          }
        }
      }
      struct Promotion {
        std::size_t reserve_index{0U};
        std::size_t implicated_template_changes{0U};
        std::size_t other_template_changes{0U};
        bool via_change{false};
        bool lane_change{false};
      };
      std::vector<Promotion> ranked;
      ranked.reserve(assignment.spline_candidate_reserve.size());
      for (std::size_t reserve_index = 0U;
           reserve_index < assignment.spline_candidate_reserve.size(); ++reserve_index) {
        const auto& fallback = assignment.spline_candidate_reserve[reserve_index];
        Promotion promotion;
        promotion.reserve_index = reserve_index;
        const auto compared =
            std::min(assignment.selected_templates.size(), fallback.templates.size());
        for (std::size_t position = 0U; position < compared; ++position) {
          if (assignment.selected_templates[position] == fallback.templates[position]) {
            continue;
          }
          const auto selected_template = assignment.selected_templates[position];
          const auto implicated =
              selected_template < assignment.templates.size() &&
              implicated_fragments.contains(assignment.templates[selected_template].fragment);
          if (implicated) {
            ++promotion.implicated_template_changes;
          } else {
            ++promotion.other_template_changes;
          }
        }
        promotion.other_template_changes += assignment.selected_templates.size() > compared
                                                ? assignment.selected_templates.size() - compared
                                                : fallback.templates.size() - compared;
        promotion.via_change = fallback.via_choices != assignment.selected_via_choices;
        promotion.lane_change = fallback.packed_cross_sections != assignment.packed_cross_sections;
        const auto physically_relevant = promotion.implicated_template_changes > 0U ||
                                         promotion.via_change || promotion.lane_change;
        if (physically_relevant) {
          ranked.push_back(promotion);
        }
      }
      std::stable_sort(ranked.begin(), ranked.end(), [&](const auto& lhs, const auto& rhs) {
        const auto objective = [&](const Promotion& candidate) {
          // A topology diagnosis must first change the implicated corridor/portal template.
          // Clearance diagnoses may instead change an implicated template, a via site, or the
          // packed lane state. Minimize unrelated destruction after satisfying that condition.
          const auto required_change_missing =
              topology_failure ? candidate.implicated_template_changes == 0U
                               : candidate.implicated_template_changes == 0U &&
                                     !candidate.via_change && !candidate.lane_change;
          return std::tuple{required_change_missing ? 1U : 0U,
                            candidate.other_template_changes,
                            candidate.implicated_template_changes == 0U ? 1U : 0U,
                            candidate.via_change ? 0U : 1U,
                            candidate.lane_change ? 0U : 1U,
                            candidate.reserve_index};
        };
        return objective(lhs) < objective(rhs);
      });
      const auto promotion_limit =
          std::min(result.configured.max_joint_spline_candidates, ranked.size());
      if (promotion_limit == 0U ||
          (topology_failure && ranked.front().implicated_template_changes == 0U)) {
        continue;
      }
      std::vector<bool> promoted(assignment.spline_candidate_reserve.size(), false);
      assignment.spline_candidate_shortlist.reserve(promotion_limit);
      for (std::size_t index = 0U; index < promotion_limit; ++index) {
        promoted[ranked[index].reserve_index] = true;
        assignment.spline_candidate_shortlist.push_back(
            std::move(assignment.spline_candidate_reserve[ranked[index].reserve_index]));
      }
      std::vector<RegionalBundleSelection> remaining_reserve;
      remaining_reserve.reserve(assignment.spline_candidate_reserve.size() - promotion_limit);
      for (std::size_t index = 0U; index < assignment.spline_candidate_reserve.size(); ++index) {
        if (!promoted[index]) {
          remaining_reserve.push_back(std::move(assignment.spline_candidate_reserve[index]));
        }
      }
      result.joint_spline_fallback_candidates_promoted += promotion_limit;
      ++result.joint_spline_fallback_promotion_waves;
      ++assignment.spline_fallback_waves;
      promoted_windows.push_back(assignment.window_id);
      promoted_wave.emplace(assignment.window_id, assignment.spline_fallback_waves);
      assignment.spline_candidate_reserve = std::move(remaining_reserve);
      assignment.spline_realizations.clear();
      assignment.stop_reason = RegionalBundleStopReason::complete;
      assignment.joint_spline_candidate = false;
      if (telemetry != nullptr) {
        RouterTelemetryEvent event;
        event.kind = RouterTelemetryEventKind::search_summary;
        event.group_id = "regional-window:" + std::to_string(assignment.window_id);
        event.stable_id = "regional-fallback-promotion:" + std::to_string(assignment.window_id);
        event.detail = "promoted " + std::to_string(promotion_limit) +
                       " compact regional fallback states after " +
                       std::string(topology_failure ? "topology" : "clearance") +
                       " failure; wave=" + std::to_string(assignment.spline_fallback_waves) + "/" +
                       std::to_string(maximum_fallback_waves) +
                       ", remaining=" + std::to_string(assignment.spline_candidate_reserve.size());
        static_cast<void>(telemetry->try_emit(std::move(event)));
      }
    }
    if (!promoted_windows.empty()) {
      // Re-rank after each wave because its least-illegal result changes which
      // corridor/lane/via alternatives are most relevant for the next bounded wave.
      realize_bundle_splines(board, result, stop_token, telemetry, maximum_fallback_waves);
      for (const auto window_id : promoted_windows) {
        const auto assignment =
            std::find_if(result.bundle_assignments.begin(), result.bundle_assignments.end(),
                         [&](const auto& candidate) { return candidate.window_id == window_id; });
        const auto wave = promoted_wave.find(window_id);
        // Only the recursion level whose directly promoted wave found the legal state owns the
        // recovery. Outer levels see a larger wave ordinal and must not double-count it.
        result.joint_spline_fallback_windows_recovered +=
            assignment != result.bundle_assignments.end() && assignment->complete() &&
                    wave != promoted_wave.end() && assignment->spline_fallback_waves == wave->second
                ? 1U
                : 0U;
      }
      return;
    }
  }

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
      result.spline_topology_gate_rejections += !realization.topology_gate_sequence_legal ? 1U : 0U;
      result.spline_mutual_clearance_rejections += !realization.mutually_legal ? 1U : 0U;
    }
  }
  result.selected_terminal_portal_choices = selected_terminal_portals.size();
  result.spline_realization_us += static_cast<std::uint64_t>(
      std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() -
                                                            realization_started)
          .count());
}

void build_regional_spline_candidate(const Board& board, const GlobalRouteResult& global,
                                     RegionalRouteResult& result) {
  result.spline_candidate_paths.clear();
  result.multilayer_route_plans.clear();
  result.complete_via_dependent_routes = 0U;
  result.planned_layer_runs = 0U;
  result.planned_via_transitions = 0U;
  result.stitched_multilayer_runs = 0U;
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
  // Adjacent fragments of one source run may independently select the same alternate layer.
  // Treat that as one physical excursion: retaining each fragment's local return transition
  // creates an unnecessary via pair at every internal coarse vertex and can even form an
  // invalid same-site L2->L1->L2 stack.  Stitch only across an independently checked alternate-
  // layer chord; the first entry and final exit remain the transaction's physical vias.
  const auto bridge_is_static_legal = [&](const RegionalRouteFragment& source, LayerId layer,
                                          Point first, Point second) {
    if (first == second) {
      return true;
    }
    const auto extracted = extract_polygonal_board_free_space(
        board,
        BoardFreeSpaceExtractionConfig{layer, source.id.net, true, true,
                                       !result.configured.treat_saved_zone_fills_as_refillable});
    if (!extracted.complete()) {
      return false;
    }
    const auto certificate = assess_free_space_chord(extracted.domain, first, second);
    const auto required = Nanometres{source.clearance.value() +
                                     std::max<std::int64_t>(1, (source.width.value() + 1) / 2)};
    return certificate && certificate->value() >= required.value();
  };
  for (auto current = replacements.begin(); current != replacements.end(); ++current) {
    bool stitched = false;
    while (!current->second.centerline.empty()) {
      const auto source =
          std::find_if(result.fragments.begin(), result.fragments.end(),
                       [&](const auto& fragment) { return fragment.id == current->first; });
      if (source == result.fragments.end()) {
        break;
      }
      const auto next_source =
          std::find_if(result.fragments.begin(), result.fragments.end(), [&](const auto& fragment) {
            return fragment.id.net == source->id.net &&
                   fragment.id.connection_index == source->id.connection_index &&
                   fragment.id.run_index == source->id.run_index &&
                   fragment.layer == source->layer &&
                   fragment.start == current->second.replacement_end && fragment.id != source->id;
          });
      if (next_source == result.fragments.end()) {
        break;
      }
      const auto next = replacements.find(next_source->id);
      if (next == replacements.end() || next->second.centerline.empty() ||
          next->second.layer != current->second.layer ||
          next->second.width != current->second.width ||
          next->second.width_kind != current->second.width_kind) {
        break;
      }
      const auto bridge_start = current->second.centerline.back();
      const auto bridge_end = next->second.centerline.front();
      if (!bridge_is_static_legal(*source, current->second.layer, bridge_start, bridge_end)) {
        break;
      }
      if (bridge_start != bridge_end) {
        current->second.centerline.push_back(bridge_end);
      }
      for (const auto point : next->second.centerline) {
        if (current->second.centerline.back() != point) {
          current->second.centerline.push_back(point);
        }
      }
      current->second.replacement_end = next->second.replacement_end;
      current->second.exit_via_site = next->second.exit_via_site;
      replacements.erase(next);
      stitched = true;
    }
    result.stitched_multilayer_runs += stitched ? 1U : 0U;
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
    const auto stop_name = [](RegionalBundleStopReason reason) -> std::string_view {
      switch (reason) {
      case RegionalBundleStopReason::complete:
        return "complete";
      case RegionalBundleStopReason::missing_corridor_topology:
        return "missing_corridor_topology";
      case RegionalBundleStopReason::no_fitting_lane_order:
        return "no_fitting_lane_order";
      case RegionalBundleStopReason::window_ineligible:
        return "window_ineligible";
      case RegionalBundleStopReason::state_budget_exhausted:
        return "state_budget_exhausted";
      case RegionalBundleStopReason::spline_realization_failed:
        return "spline_realization_failed";
      }
      return "unknown";
    };
    for (const auto& assignment : result.bundle_assignments) {
      if (assignment.complete()) {
        continue;
      }
      RouterTelemetryEvent incomplete;
      incomplete.kind = RouterTelemetryEventKind::residual_group_updated;
      incomplete.minimum_level = RouterTelemetryLevel::diagnostic;
      incomplete.stable_id = "regional-incomplete:" + std::to_string(assignment.window_id);
      incomplete.group_id = "regional-window:" + std::to_string(assignment.window_id);
      incomplete.detail =
          "regional bundle incomplete: stop=" + std::string(stop_name(assignment.stop_reason)) +
          ", templates=" + std::to_string(assignment.templates.size()) +
          ", bundle_states=" + std::to_string(assignment.bundle_search_states) +
          ", access_pruned=" + std::to_string(assignment.partial_access_pruned_states) +
          ", terminal_pair=" + std::to_string(assignment.partial_terminal_pair_pruned_states) +
          ", via_pair=" + std::to_string(assignment.partial_via_pair_pruned_states) +
          ", terminal_via=" + std::to_string(assignment.partial_terminal_via_pruned_states) +
          ", lane_capacity=" + std::to_string(assignment.partial_lane_capacity_pruned_states) +
          ", packing=" + std::to_string(assignment.packing_rejected_states) +
          ", spline_candidates=" + std::to_string(assignment.spline_candidates_evaluated);
      emit(std::move(incomplete));
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
        route.route_family = "regional_via_runup";
        route.route_lifecycle = "selected";
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
    publish_regional_spline_assignment_telemetry(assignment, telemetry,
                                                 RouterTelemetryLevel::normal, "selected");
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
    route.route_family = "regional_multilayer";
    route.route_lifecycle = "selected";
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
      ", terminal_navmesh=" + std::to_string(result.accepted_terminal_navmesh_templates) + "/" +
      std::to_string(result.terminal_navmesh_template_checks) +
      ", terminal_navmesh_rejected=" + std::to_string(result.rejected_terminal_navmesh_templates) +
      ", terminal_navmesh_exhausted=" +
      std::string(result.terminal_navmesh_template_budget_exhausted ? "yes" : "no") +
      ", joint_repair_windows=" + std::to_string(result.joint_repair_windows_attempted) +
      ", joint_repair_completed=" + std::to_string(result.joint_repair_windows_completed) +
      ", joint_repair_checks=" + std::to_string(result.joint_repair_template_checks) +
      ", spline_portfolio_windows=" + std::to_string(result.spline_portfolio_windows) +
      ", joint_spline_generated=" + std::to_string(result.joint_spline_candidates_generated) +
      ", joint_spline_active=" + std::to_string(result.joint_spline_active_candidates_retained) +
      ", joint_spline_reserve=" + std::to_string(result.joint_spline_fallback_candidates_retained) +
      ", joint_spline_dropped=" + std::to_string(result.joint_spline_fallback_candidates_dropped) +
      ", joint_spline_promoted=" +
      std::to_string(result.joint_spline_fallback_candidates_promoted) +
      ", joint_spline_promotion_waves=" +
      std::to_string(result.joint_spline_fallback_promotion_waves) + ", joint_spline_remaining=" +
      std::to_string(result.joint_spline_fallback_candidates_remaining) +
      ", joint_spline_candidates=" + std::to_string(result.joint_spline_candidates_evaluated) +
      ", joint_spline_recovered=" + std::to_string(result.joint_spline_windows_recovered) +
      ", joint_spline_fallback_recovered=" +
      std::to_string(result.joint_spline_fallback_windows_recovered) +
      ", joint_spline_failed_improved=" +
      std::to_string(result.joint_spline_failed_fallback_improvements) +
      ", adaptive_pair_shoves=" + std::to_string(result.spline_adaptive_pair_shove_accepted) +
      ", adaptive_pair_legalized=" + std::to_string(result.spline_adaptive_pair_windows_legalized) +
      ", residual_retry_reused_assignments=" +
      std::to_string(result.residual_conflict_cone_reused_assignments) +
      ", residual_retry_recomputed_assignments=" +
      std::to_string(result.residual_conflict_cone_recomputed_assignments) +
      ", residual_retry_stub_fragments=" +
      std::to_string(result.residual_conflict_cone_stub_fragments) +
      ", residual_retry_withheld_assignments=" +
      std::to_string(result.residual_conflict_cone_retry_withheld_assignments) +
      ", complete_via_dependent_routes=" + std::to_string(result.complete_via_dependent_routes) +
      ", planned_layer_runs=" + std::to_string(result.planned_layer_runs) +
      ", planned_via_transitions=" + std::to_string(result.planned_via_transitions) +
      ", stitched_multilayer_runs=" + std::to_string(result.stitched_multilayer_runs) +
      ", spline_candidate=" + std::string(result.spline_candidate_available ? "yes" : "no");
  emit(std::move(summary));
}

[[nodiscard]] bool coarse_topology_complete(const GlobalRouteResult& global) {
  return global.unrouted_nets.empty() && global.unaccepted_selected_voronoi_lane_transitions == 0U;
}

// Regional packing is intentionally a bounded local solver, not a fallback global router.  It
// must wait for every coarse connection, but a global connected component is not itself a local
// window: populate_windows() partitions biconnected conflict edges spatially and explicitly
// withholds any uncuttable oversized window.  Rejecting an entire 23-net component here would
// discard that decomposition and starve viable 2-8-net lane/via/spline repairs.  The caller still
// preserves every global conflict as routability history; no deferred or regional result is
// accepted without the ordinary exact gates.
[[nodiscard]] bool regional_realization_is_bounded(const GlobalRouteResult& global,
                                                   const RegionalRouterConfig& config) {
  static_cast<void>(config);
  // Regional conflict extraction needs one connected coarse path per required connection; it does
  // not need every optional packed-medial bridge to have been accepted already. Those unaccepted
  // bridges remain a hard final-materialization gate, but deferring all lane/via/spline work here
  // makes a scalar global resource defect hide the bounded physical conflicts which could supply
  // the next PathFinder history update.
  return global.unrouted_nets.empty();
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
    if (config.enable_spline_realization) {
      realize_bundle_splines(board, candidate, stop_token, telemetry);
      emit_progress("regional-analysis:" + std::string(phase) + ":splines",
                    std::string(phase) + " regional spline realization completed: splines=" +
                        std::to_string(spline_count(candidate)));
      build_regional_spline_candidate(board, global, candidate);
    }
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
  if (config.enable_spline_realization && !result.spline_candidate_available &&
      !stop_token.stop_requested()) {
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
    std::map<std::size_t, std::set<std::size_t>> cone_members;
    const auto residual_cones_built =
        build_residual_conflict_cone_windows(retry, result, replaced_window_ids, cone_members);
    if (residual_cones_built &&
        retry.windows.size() <= config.max_residual_retry_bundle_assignments) {
      const auto cones_built = retry.residual_conflict_cones_built;
      const auto windows_merged = retry.residual_conflict_cone_windows_merged;
      const auto stub_fragments = retry.residual_conflict_cone_stub_fragments;
      const auto retry_windows = retry.windows;
      std::vector<RegionalResidualConflictConeAttempt> cone_attempts;
      cone_attempts.reserve(retry_windows.size());
      std::size_t recomputed = 0U;
      std::size_t retry_lane_queries = 0U;
      std::size_t retry_lane_cache_hits = 0U;
      std::size_t retry_lane_states = 0U;
      std::size_t retry_lane_pruned_states = 0U;
      std::size_t retry_lane_analytical_rejections = 0U;
      std::size_t retry_lane_budget_exhaustions = 0U;
      std::size_t retry_partial_lane_capacity_pruned = 0U;
      emit_progress("regional-analysis:residual-cones",
                    "residual conflict cones constructed: cones=" + std::to_string(cones_built) +
                        ", windows=" + std::to_string(retry_windows.size()));
      // Each merged window is one independent conflict cone.  Requiring every cone to finish
      // before retaining any result makes a small deadlocked cone discard an unrelated legal
      // transaction.  Recompute and admit one cone at a time; its member assignments still move
      // atomically, and the complete canonical-board DRC transaction remains the only commit.
      for (const auto& cone_window : retry_windows) {
        if (stop_token.stop_requested()) {
          break;
        }
        const auto members = cone_members.find(cone_window.id);
        if (members == cone_members.end() || members->second.empty()) {
          continue;
        }
        auto cone = retry;
        cone.windows = {cone_window};
        cone.channels.clear();
        cone.terminal_choices.clear();
        cone.via_choices.clear();
        cone.via_region_evidence.clear();
        cone.bundle_assignments.clear();
        cone.spline_candidate_paths.clear();
        cone.multilayer_route_plans.clear();
        cone.spline_candidate_available = false;
        ++cone.residual_conflict_cone_retries;
        emit_progress("regional-analysis:residual-channels",
                      "residual cone corridors constructed: window=" +
                          std::to_string(cone_window.id));
        populate_channels(board, global, cone);
        populate_bundle_assignments(board, global, cone, telemetry, "residual");
        ++recomputed;
        emit_progress("regional-analysis:residual-assignments",
                      "residual cone portfolio prepared: window=" + std::to_string(cone_window.id) +
                          ", assignments=" + std::to_string(cone.bundle_assignments.size()));
        realize_bundle_splines(board, cone, stop_token, telemetry,
                               config.max_joint_spline_fallback_waves);
        emit_progress(
            "regional-analysis:residual-splines",
            "residual cone spline realization completed: window=" + std::to_string(cone_window.id) +
                ", splines=" + std::to_string(spline_count(cone)));
        retry_lane_queries += cone.lane_solver_queries;
        retry_lane_cache_hits += cone.lane_solver_cache_hits;
        retry_lane_states += cone.lane_solver_states;
        retry_lane_pruned_states += cone.lane_solver_pruned_states;
        retry_lane_analytical_rejections += cone.lane_solver_analytical_rejections;
        retry_lane_budget_exhaustions += cone.lane_solver_budget_exhaustions;
        retry_partial_lane_capacity_pruned += cone.partial_lane_capacity_pruned_states;
        const auto complete_cone_transaction =
            !cone.bundle_assignments.empty() &&
            std::all_of(cone.bundle_assignments.begin(), cone.bundle_assignments.end(),
                        [](const auto& assignment) { return assignment.complete(); });
        RegionalResidualConflictConeAttempt attempt;
        attempt.root_window_id = cone_window.id;
        attempt.source_window_ids.assign(members->second.begin(), members->second.end());
        attempt.fragment_count = cone_window.fragments.size();
        attempt.net_count = cone_window.nets.size();
        attempt.assignment_count = cone.bundle_assignments.size();
        attempt.complete = complete_cone_transaction;
        for (const auto& assignment : cone.bundle_assignments) {
          attempt.incomplete_assignments += assignment.complete() ? 0U : 1U;
          for (const auto& realization : assignment.spline_realizations) {
            attempt.topology_rejections += realization.topology_legal ? 0U : 1U;
            attempt.mutual_clearance_rejections += realization.mutually_legal ? 0U : 1U;
            attempt.quantized_clearance_rejections +=
                realization.quantized_mutually_legal ? 0U : 1U;
            if (!realization.topology_legal) {
              attempt.topology_rejected_fragments.push_back(
                  {realization.fragment, realization.merged_fragments});
            }
            if (!realization.mutually_legal) {
              attempt.mutual_clearance_rejected_fragments.push_back(
                  {realization.fragment, realization.merged_fragments});
            }
            if (!realization.quantized_mutually_legal) {
              attempt.quantized_clearance_rejected_fragments.push_back(
                  {realization.fragment, realization.merged_fragments});
            }
          }
        }
        const auto canonicalize_failures = [](auto& fragments) {
          std::sort(fragments.begin(), fragments.end());
          fragments.erase(std::unique(fragments.begin(), fragments.end()), fragments.end());
        };
        canonicalize_failures(attempt.topology_rejected_fragments);
        canonicalize_failures(attempt.mutual_clearance_rejected_fragments);
        canonicalize_failures(attempt.quantized_clearance_rejected_fragments);
        for (const auto& assignment : cone.bundle_assignments) {
          attempt.spline_candidates_evaluated += assignment.spline_candidates_evaluated;
          attempt.spline_candidates_legal += assignment.spline_candidates_legal;
        }
        attempt.spline_merged_runs = cone.spline_merged_runs;
        attempt.spline_merged_fragments = cone.spline_merged_fragments;
        attempt.template_coverage_states = cone.residual_cone_template_coverage_states;
        attempt.template_coverage_retained = cone.residual_cone_template_coverage_retained;
        attempt.stub_fragments = cone_window.residual_stub_fragments;
        for (const auto& assignment : cone.bundle_assignments) {
          attempt.template_coverage_coupled_rejections +=
              assignment.residual_cone_template_coverage_coupled_rejections;
          attempt.template_coverage_access_rejections +=
              assignment.residual_cone_template_coverage_access_rejections;
          attempt.template_coverage_packing_rejections +=
              assignment.residual_cone_template_coverage_packing_rejections;
        }
        cone_attempts.push_back(std::move(attempt));
        if (!complete_cone_transaction) {
          continue;
        }
        auto combined = merge_residual_cone_checkpoint(result, std::move(cone), members->second);
        build_regional_spline_candidate(board, global, combined);
        // Retained windows are immutable.  A complete recomputed cone may replace only its own
        // member assignments and only on a strict feasibility-first improvement.  Exact detailed
        // copper, connectivity, and DRC remain downstream transactional gates.
        if (unresolved_objective(combined) < unresolved_objective(result)) {
          combined.residual_conflict_cone_selected = true;
          result = std::move(combined);
          cone_attempts.back().selected = true;
        }
      }
      result.lane_solver_queries += retry_lane_queries;
      result.lane_solver_cache_hits += retry_lane_cache_hits;
      result.lane_solver_states += retry_lane_states;
      result.lane_solver_pruned_states += retry_lane_pruned_states;
      result.lane_solver_analytical_rejections += retry_lane_analytical_rejections;
      result.lane_solver_budget_exhaustions += retry_lane_budget_exhaustions;
      result.partial_lane_capacity_pruned_states += retry_partial_lane_capacity_pruned;
      result.residual_conflict_cone_retries = recomputed == 0U ? 0U : 1U;
      result.residual_conflict_cones_built = cones_built;
      result.residual_conflict_cone_windows_merged = windows_merged;
      result.residual_conflict_cone_stub_fragments = stub_fragments;
      result.residual_conflict_cone_recomputed_assignments = recomputed;
      result.residual_conflict_cone_attempts = std::move(cone_attempts);
      result.residual_conflict_cone_reused_assignments = static_cast<std::size_t>(
          std::count_if(result.bundle_assignments.begin(), result.bundle_assignments.end(),
                        [&](const auto& assignment) {
                          return !replaced_window_ids.contains(assignment.window_id);
                        }));
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
  result.joint_spline_fallback_candidates_remaining =
      std::accumulate(result.bundle_assignments.begin(), result.bundle_assignments.end(),
                      std::size_t{0}, [](const auto total, const auto& assignment) {
                        return total + assignment.spline_candidate_reserve.size();
                      });
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
  knowledge.success_history = config.global.hydraulic_success_history;
  std::vector<RoutingPlanFeedbackProbe> feedback_probes;
  std::optional<RoutingPlanResult> best;
  // The global router retains a pass-local beam.  Keep a second, small beam across physical
  // feedback epochs so a topology which is temporarily worse under scalar congestion remains
  // available after regional lane/via/escape evidence changes the cost field.
  std::vector<GlobalRouteResult> retained_global_portfolios;
  // Feedback is deliberately a bounded affected-cone replan.  A selected checkpoint remains
  // immutable while the global router reconstructs occupancy from its unchanged paths and only
  // negotiates the nets named by current physical diagnoses.  This is the global counterpart of
  // a regional repair window: it retains valid topology outside the conflict cone instead of
  // relearning the complete board on every physical-feedback pass.
  std::optional<GlobalRouteResult> feedback_checkpoint;
  std::vector<NetId> feedback_affected_nets;
  std::size_t global_feedback_passes = 0U;
  std::size_t regional_feedback_passes = 0U;
  std::size_t stagnant_physical_feedback_passes = 0U;
  std::size_t cumulative_global_iterations = 0U;
  std::size_t retained_global_portfolio_peak = 0U;
  std::size_t retained_global_portfolios_evaluated = 0U;
  std::size_t intermediate_regional_probes = 0U;

  if (config.retained_global_portfolios == 0U || config.intermediate_regional_portfolios == 0U ||
      config.intermediate_regional_portfolios > config.retained_global_portfolios ||
      config.intermediate_regional_bundle_states == 0U ||
      config.intermediate_regional_exact_transactions == 0U ||
      config.max_stagnant_feedback_epochs == 0U) {
    RoutingPlanResult invalid;
    invalid.stop_reason = RoutingPlanStopReason::invalid_input;
    invalid.diagnostic = "interleaved routing requires positive, consistently bounded portfolio, "
                         "regional-probe, and stagnation limits";
    return invalid;
  }
  const auto feedback_epoch_count = config.max_routability_feedback_retries + 1U;
  const auto global_epoch_iterations =
      !config.interleave_routability_feedback || feedback_epoch_count == 1U
          ? config.global.max_iterations
          : std::max<std::size_t>(1U, (config.global.max_iterations + feedback_epoch_count - 1U) /
                                          feedback_epoch_count);

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
    // Once two candidates have complete coarse topology, the regional solver is the more
    // authoritative physical representation: a history-informed topology with a larger
    // provisional shadow count can nevertheless leave fewer unsolved lane/via/spline windows.
    // Preserve connectivity and topology as hard lexicographic gates, then retain that partial
    // physical improvement for the next feedback pass before comparing coarse overflow.
    return std::tuple{lhs.complete() ? 0U : 1U,
                      lhs.coarse_global.unrouted_nets.size(),
                      coarse_topology_complete(lhs.coarse_global) ? 0U : 1U,
                      incomplete_assignments(lhs),
                      lhs.coarse_global.total_overflow,
                      lhs.coarse_global.total_cost} <
           std::tuple{rhs.complete() ? 0U : 1U,
                      rhs.coarse_global.unrouted_nets.size(),
                      coarse_topology_complete(rhs.coarse_global) ? 0U : 1U,
                      incomplete_assignments(rhs),
                      rhs.coarse_global.total_overflow,
                      rhs.coarse_global.total_cost};
  };
  const auto attach_feedback_probes = [&](RoutingPlanResult& candidate) {
    candidate.feedback_probes = feedback_probes;
    candidate.global_feedback_epochs = feedback_probes.size();
    candidate.cumulative_global_iterations = cumulative_global_iterations;
    candidate.retained_global_portfolio_peak = retained_global_portfolio_peak;
    candidate.retained_global_portfolios_evaluated = retained_global_portfolios_evaluated;
    candidate.intermediate_regional_probes = intermediate_regional_probes;
  };
  const auto same_global_topology = [](const GlobalRouteResult& lhs, const GlobalRouteResult& rhs) {
    return lhs.paths == rhs.paths && lhs.unrouted_nets == rhs.unrouted_nets;
  };
  const auto retain_global_portfolio = [&](const GlobalRouteResult& global) {
    if (std::none_of(
            retained_global_portfolios.begin(), retained_global_portfolios.end(),
            [&](const auto& retained) { return same_global_topology(retained, global); })) {
      retained_global_portfolios.push_back(global);
    }
    std::stable_sort(retained_global_portfolios.begin(), retained_global_portfolios.end(),
                     [&](const auto& lhs, const auto& rhs) {
                       return global_route_checkpoint_better(lhs, rhs,
                                                             config.global.checkpoint_objective);
                     });
    if (retained_global_portfolios.size() > config.retained_global_portfolios) {
      retained_global_portfolios.resize(config.retained_global_portfolios);
    }
    retained_global_portfolio_peak =
        std::max(retained_global_portfolio_peak, retained_global_portfolios.size());
  };
  const auto diagnosis_nets = [](const std::vector<RoutabilityDiagnosis>& diagnoses) {
    std::set<NetId> nets;
    for (const auto& diagnosis : diagnoses) {
      nets.insert(diagnosis.resource.nets.begin(), diagnosis.resource.nets.end());
    }
    return std::vector<NetId>{nets.begin(), nets.end()};
  };
  const auto expand_feedback_nets = [](const GlobalRouteResult& global,
                                       std::vector<NetId> selected) {
    std::set<NetId> nets(selected.begin(), selected.end());
    nets.insert(global.unrouted_nets.begin(), global.unrouted_nets.end());
    for (const auto& failure : global.hard_capacity_net_failures) {
      nets.insert(failure.net);
    }
    // Preserve blocker provenance across epoch boundaries.  A failed net without its repeatedly
    // observed blockers merely recreates the same local minimum in the next short PathFinder
    // wave.
    for (const auto& blocker : global.hard_capacity_blockers) {
      if (nets.contains(blocker.failed_net)) {
        nets.insert(blocker.blocking_net);
      }
    }
    // Conflict components can be weakly chained. Expand only groups already touched by the
    // diagnosis/rip-up set; the global router's own LNS then progressively enlarges that cone.
    const auto before_groups = nets;
    for (const auto& group : global.residual_conflict_groups) {
      if (std::any_of(group.nets.begin(), group.nets.end(),
                      [&](const auto& net) { return before_groups.contains(net); })) {
        nets.insert(group.nets.begin(), group.nets.end());
      }
    }
    return std::vector<NetId>{nets.begin(), nets.end()};
  };
  const auto feedback_candidate_summary = [](const GlobalRouteResult& global, bool history_informed,
                                             bool incremental, bool fell_back_to_full_global,
                                             std::size_t affected_nets) {
    RoutingPlanFeedbackCandidate summary;
    summary.history_informed_global = history_informed;
    summary.incremental_global = incremental;
    summary.feedback_fell_back_to_full_global = fell_back_to_full_global;
    summary.feedback_affected_nets = affected_nets;
    summary.global_reused_nets = global.reused_nets.size();
    summary.global_rerouted_nets = global.rerouted_nets.size();
    summary.global_unrouted_nets = global.unrouted_nets.size();
    summary.global_total_overflow = global.total_overflow;
    summary.global_clearance_shadow_conflicts = global.clearance_shadow_conflicts.size();
    summary.global_unaccepted_lane_transitions =
        global.unaccepted_selected_voronoi_lane_transitions;
    summary.coarse_topology_complete = coarse_topology_complete(global);
    return summary;
  };
  const auto feedback_cone_is_local = [](const GlobalRouteResult& checkpoint,
                                         std::size_t affected_nets) {
    std::set<NetId> total_nets;
    for (const auto& path : checkpoint.paths) {
      total_nets.insert(path.net);
    }
    total_nets.insert(checkpoint.unrouted_nets.begin(), checkpoint.unrouted_nets.end());
    if (total_nets.empty()) {
      return false;
    }
    // An affected cone must leave at least one quarter of the routed-net groups intact.  Below
    // that threshold incremental reuse preserves real global context; above it a full PathFinder
    // wave is both a better escape move and avoids pretending that two frozen nets form a useful
    // local checkpoint on a 25-net board.
    const auto required_preserved = (total_nets.size() + 3U) / 4U;
    return affected_nets <= total_nets.size() - required_preserved;
  };
  const auto publish_feedback_selection = [&](const GlobalRouteResult& selected) {
    if (telemetry == nullptr) {
      return;
    }
    RouterTelemetryEvent checkpoint;
    checkpoint.kind = RouterTelemetryEventKind::checkpoint_selected;
    checkpoint.iteration = selected.selected_iteration;
    std::set<NetId> routed_nets;
    for (const auto& path : selected.paths) {
      routed_nets.insert(path.net);
    }
    std::set<NetId> all_nets = routed_nets;
    all_nets.insert(selected.unrouted_nets.begin(), selected.unrouted_nets.end());
    checkpoint.total_nets = all_nets.size();
    checkpoint.routed_nets = routed_nets.size();
    checkpoint.unrouted_nets = selected.unrouted_nets.size();
    checkpoint.sampled_overflow = selected.sampled_overflow;
    checkpoint.corridor_overflow = selected.corridor_overflow;
    checkpoint.physical_overflow_nm = selected.total_voronoi_overflow_nm;
    checkpoint.clearance_shadow_conflicts = selected.clearance_shadow_conflicts.size();
    checkpoint.route_cost = selected.total_cost;
    checkpoint.detail =
        "feasibility-first physical-feedback global candidate selected for regional realization";
    static_cast<void>(telemetry->try_emit(std::move(checkpoint)));
  };

  for (std::size_t pass = 0U; pass <= config.max_routability_feedback_retries; ++pass) {
    RoutingPlanFeedbackProbe feedback_probe;
    feedback_probe.pass = pass;
    const bool final_epoch = pass == config.max_routability_feedback_retries;
    const auto epoch_iterations =
        config.interleave_routability_feedback
            ? std::min(global_epoch_iterations, config.global.max_iterations)
            : config.global.max_iterations;
    feedback_probe.global_epoch_iterations = epoch_iterations;
    feedback_probe.history_entries_before = knowledge.history.size();
    feedback_probe.history_pressure_before = knowledge.pressure();
    auto global_config = config.global;
    global_config.max_iterations = epoch_iterations;
    global_config.feedback_epoch_index = pass;
    // Iteration ordinals restart inside route_global_incremental.  Later epochs have already
    // consumed the soft exploratory phase, so enable the mature bidirectional, congestion, and
    // hard-capacity policies at their first local iteration instead of repeatedly delaying them.
    if (config.interleave_routability_feedback && pass > 0U) {
      if (global_config.hard_capacity_start_iteration > 0U) {
        global_config.hard_capacity_start_iteration = 1U;
      }
      if (global_config.bidirectional_route_growth_start_iteration > 0U) {
        global_config.bidirectional_route_growth_start_iteration = 1U;
      }
      if (global_config.present_congestion_escalation_start_iteration > 0U) {
        global_config.present_congestion_escalation_start_iteration = 2U;
      }
    } else {
      global_config.hard_capacity_start_iteration =
          std::min(global_config.hard_capacity_start_iteration, epoch_iterations);
    }
    global_config.routability_history = knowledge.history;
    global_config.hydraulic_success_history = knowledge.success_history;
    if (telemetry != nullptr) {
      RouterTelemetryEvent epoch;
      epoch.kind = RouterTelemetryEventKind::search_summary;
      epoch.stable_id = "routing-feedback-epoch:" + std::to_string(pass + 1U);
      epoch.iteration = pass + 1U;
      epoch.route_revision = pass + 1U;
      epoch.detail = "interleaved routing epoch started: pathfinder_budget=" +
                     std::to_string(epoch_iterations) +
                     ", retained_portfolios=" + std::to_string(retained_global_portfolios.size()) +
                     ", physical_history_entries=" + std::to_string(knowledge.history.size());
      static_cast<void>(telemetry->try_emit(std::move(epoch)));
    }
    RoutingPlanResult candidate;
    const bool feedback_checkpoint_available =
        feedback_checkpoint.has_value() && !feedback_affected_nets.empty();
    const bool incremental_feedback =
        feedback_checkpoint_available &&
        feedback_cone_is_local(*feedback_checkpoint, feedback_affected_nets.size());
    const bool feedback_fell_back_to_full_global =
        feedback_checkpoint_available && !incremental_feedback;
    const auto affected_count = feedback_affected_nets.size();
    auto probed_global =
        incremental_feedback
            ? route_global_incremental(board, *feedback_checkpoint, feedback_affected_nets,
                                       global_config, stop_token, telemetry, true)
            : route_global(board, global_config, stop_token, telemetry);
    cumulative_global_iterations += probed_global.iterations;
    feedback_probe.cumulative_global_iterations = cumulative_global_iterations;
    candidate.cheap_physical_probes = pass + 1U;
    candidate.global_feedback_passes = global_feedback_passes;
    candidate.regional_feedback_passes = regional_feedback_passes;
    if (stop_token.stop_requested() ||
        probed_global.stop_reason == GlobalRouterStopReason::cancelled) {
      candidate.stop_reason = RoutingPlanStopReason::cancelled;
      candidate.diagnostic = "routing planner cancelled during coarse global routing";
      candidate.routability_knowledge = std::move(knowledge);
      feedback_probe.candidates.push_back(
          feedback_candidate_summary(probed_global, pass > 0U, incremental_feedback,
                                     feedback_fell_back_to_full_global, affected_count));
      feedback_probes.push_back(std::move(feedback_probe));
      attach_feedback_probes(candidate);
      return candidate;
    }
    if (probed_global.stop_reason == GlobalRouterStopReason::invalid_input) {
      candidate.coarse_global = std::move(probed_global);
      candidate.stop_reason = RoutingPlanStopReason::invalid_input;
      candidate.diagnostic = candidate.coarse_global.diagnostic;
      candidate.routability_knowledge = std::move(knowledge);
      feedback_probe.candidates.push_back(
          feedback_candidate_summary(candidate.coarse_global, pass > 0U, incremental_feedback,
                                     feedback_fell_back_to_full_global, affected_count));
      feedback_probes.push_back(std::move(feedback_probe));
      attach_feedback_probes(candidate);
      return candidate;
    }

    const auto global_diagnoses = probe_global_routability(probed_global, config.routability_probe);
    feedback_probe.global_diagnoses = global_diagnoses.size();
    accumulate_routability_knowledge(knowledge, global_diagnoses, config.routability_probe);
    feedback_probe.history_entries_after_global = knowledge.history.size();
    feedback_probe.history_pressure_after_global = knowledge.pressure();
    const bool retry_available = pass < config.max_routability_feedback_retries;
    if (pass > 0U) {
      ++global_feedback_passes;
    }

    retain_global_portfolio(probed_global);
    feedback_probe.retained_global_portfolios = retained_global_portfolios.size();

    std::vector<std::pair<GlobalRouteResult, bool>> regional_inputs;
    const auto regional_portfolio_limit = config.intermediate_regional_portfolios;
    if (final_epoch && best) {
      regional_inputs.emplace_back(best->coarse_global, true);
    }
    for (const auto& retained : retained_global_portfolios) {
      if (regional_inputs.size() >= regional_portfolio_limit) {
        break;
      }
      if (std::any_of(regional_inputs.begin(), regional_inputs.end(), [&](const auto& input) {
            return same_global_topology(input.first, retained);
          })) {
        continue;
      }
      regional_inputs.emplace_back(retained, pass > 0U);
    }
    if (regional_inputs.empty()) {
      regional_inputs.emplace_back(std::move(probed_global), pass > 0U);
    }
    retained_global_portfolios_evaluated += regional_inputs.size();
    const bool intermediate_regional_probe = config.interleave_routability_feedback && !final_epoch;
    feedback_probe.intermediate_regional_probe = intermediate_regional_probe;
    if (intermediate_regional_probe) {
      ++intermediate_regional_probes;
    }
    std::optional<RoutingPlanResult> pass_best;
    std::optional<GlobalRouteResult> lane_transition_repair_branch;
    bool has_regional_diagnoses = false;
    for (auto& [global, history_informed] : regional_inputs) {
      RoutingPlanFeedbackCandidate feedback_candidate = feedback_candidate_summary(
          global, history_informed, history_informed && incremental_feedback,
          history_informed && feedback_fell_back_to_full_global,
          history_informed && feedback_checkpoint_available ? affected_count : 0U);

      RoutingPlanResult regional_candidate;
      regional_candidate.coarse_global = std::move(global);
      regional_candidate.paths = regional_candidate.coarse_global.paths;
      regional_candidate.cheap_physical_probes = pass + 1U;
      regional_candidate.global_feedback_passes = global_feedback_passes;
      regional_candidate.regional_feedback_passes = regional_feedback_passes;
      auto regional_attempt_config = regional_config;
      regional_attempt_config.xpbd_history = knowledge.history;
      if (intermediate_regional_probe) {
        regional_attempt_config.max_bundle_search_states =
            std::min(regional_attempt_config.max_bundle_search_states,
                     config.intermediate_regional_bundle_states);
        regional_attempt_config.max_lane_order_states =
            std::min(regional_attempt_config.max_lane_order_states,
                     config.intermediate_regional_bundle_states);
        regional_attempt_config.max_access_assignment_states =
            std::min(regional_attempt_config.max_access_assignment_states,
                     config.intermediate_regional_bundle_states);
        regional_attempt_config.max_exact_transactions =
            std::min(regional_attempt_config.max_exact_transactions,
                     config.intermediate_regional_exact_transactions);
        regional_attempt_config.max_joint_spline_candidates =
            std::min(regional_attempt_config.max_joint_spline_candidates, std::size_t{2U});
        regional_attempt_config.max_joint_spline_fallback_waves =
            std::min(regional_attempt_config.max_joint_spline_fallback_waves, std::size_t{1U});
        regional_attempt_config.max_spline_sweeps =
            std::min(regional_attempt_config.max_spline_sweeps, std::size_t{2U});
        regional_attempt_config.max_quantization_feedback_iterations =
            std::min(regional_attempt_config.max_quantization_feedback_iterations, std::size_t{1U});
      }
      if (regional_realization_is_bounded(regional_candidate.coarse_global,
                                          regional_attempt_config)) {
        feedback_candidate.regional_consulted = true;
        regional_candidate.regional =
            analyze_regional_conflicts(board, regional_candidate.coarse_global,
                                       regional_attempt_config, stop_token, telemetry);
      } else {
        regional_candidate.regional.configured = regional_attempt_config;
        regional_candidate.regional.stop_reason = RegionalRouterStopReason::conflicts_detected;
        regional_candidate.regional.diagnostic =
            "regional realization deferred until every coarse connection is routed";
      }
      if (regional_candidate.regional.spline_candidate_available) {
        regional_candidate.paths = regional_candidate.regional.spline_candidate_paths;
      }
      const auto regional_diagnoses =
          diagnose_regional_routability(regional_candidate.regional, config.routability_probe);
      feedback_candidate.regional_incomplete_bundle_assignments = static_cast<std::size_t>(
          std::count_if(regional_candidate.regional.bundle_assignments.begin(),
                        regional_candidate.regional.bundle_assignments.end(),
                        [](const auto& assignment) { return !assignment.complete(); }));
      feedback_candidate.regional_diagnoses = regional_diagnoses.size();
      has_regional_diagnoses = has_regional_diagnoses || !regional_diagnoses.empty();
      accumulate_routability_knowledge(knowledge, regional_diagnoses, config.routability_probe);
      regional_candidate.routability_knowledge = knowledge;
      classify(regional_candidate);
      feedback_candidate.plan_complete = regional_candidate.complete();
      feedback_probe.candidates.push_back(std::move(feedback_candidate));
      // Do not accept this topology over a physically better checkpoint, but keep one bounded
      // repair branch when a history-informed allocation is blocked solely by an unaccepted
      // packed lane bridge.  Its next cone is enlarged from the bridge's two resource occupants;
      // this is an escape move, not a relaxation of the topology or exact lane checks.
      if (history_informed && !coarse_topology_complete(regional_candidate.coarse_global) &&
          regional_candidate.coarse_global.unaccepted_selected_voronoi_lane_transitions > 0U &&
          !lane_transition_repair_branch) {
        lane_transition_repair_branch = regional_candidate.coarse_global;
      }
      if (!pass_best || better(regional_candidate, *pass_best)) {
        pass_best = std::move(regional_candidate);
      }
    }
    feedback_probe.history_entries_after_regional = knowledge.history.size();
    feedback_probe.history_pressure_after_regional = knowledge.pressure();
    if (telemetry != nullptr) {
      RouterTelemetryEvent epoch;
      epoch.kind = RouterTelemetryEventKind::search_summary;
      epoch.stable_id = "routing-feedback-epoch:" + std::to_string(pass + 1U);
      epoch.iteration = pass + 1U;
      epoch.route_revision = pass + 1U;
      epoch.route_cost = pass_best ? pass_best->coarse_global.total_cost : 0U;
      epoch.sampled_overflow = pass_best ? pass_best->coarse_global.sampled_overflow : 0U;
      epoch.corridor_overflow = pass_best ? pass_best->coarse_global.corridor_overflow : 0U;
      epoch.physical_overflow_nm =
          pass_best ? pass_best->coarse_global.total_voronoi_overflow_nm : 0U;
      epoch.clearance_shadow_conflicts =
          pass_best ? pass_best->coarse_global.clearance_shadow_conflicts.size() : 0U;
      epoch.detail = "interleaved routing epoch completed: regional_candidates=" +
                     std::to_string(regional_inputs.size()) +
                     ", history_entries=" + std::to_string(knowledge.history.size()) +
                     ", history_pressure=" + std::to_string(knowledge.pressure());
      static_cast<void>(telemetry->try_emit(std::move(epoch)));
    }
    const auto final_equivalent_refresh = final_epoch && pass_best && best &&
                                          !better(*pass_best, *best) && !better(*best, *pass_best);
    const auto improved_best =
        pass_best && (!best || better(*pass_best, *best) || final_equivalent_refresh);
    if (improved_best) {
      best = std::move(*pass_best);
      stagnant_physical_feedback_passes = 0U;
    } else if (pass_best && best) {
      ++stagnant_physical_feedback_passes;
    }
    if (best) {
      best->routability_knowledge = knowledge;
      best->global_feedback_passes = global_feedback_passes;
      best->regional_feedback_passes = regional_feedback_passes;
      best->cheap_physical_probes = pass + 1U;
      // Build the next cone from the retained physical checkpoint itself, not from every
      // diagnostic seen in discarded alternatives.  A rejected history-informed lane bridge is
      // the one bounded exception: it may seed one expanded repair cone so fixed neighbours can
      // move together instead of repeatedly recreating the same impossible local ordering.
      if (!best->complete()) {
        if (retry_available && lane_transition_repair_branch) {
          const auto repair_diagnoses =
              probe_global_routability(*lane_transition_repair_branch, config.routability_probe);
          const auto repair_nets = expand_feedback_nets(*lane_transition_repair_branch,
                                                        diagnosis_nets(repair_diagnoses));
          if (!repair_nets.empty()) {
            feedback_checkpoint = std::move(*lane_transition_repair_branch);
            feedback_affected_nets = repair_nets;
            feedback_probe.next_feedback_uses_repair_branch = true;
          }
        }
        if (!feedback_probe.next_feedback_uses_repair_branch) {
          auto selected_diagnoses =
              probe_global_routability(best->coarse_global, config.routability_probe);
          const auto selected_regional =
              diagnose_regional_routability(best->regional, config.routability_probe);
          selected_diagnoses.insert(selected_diagnoses.end(), selected_regional.begin(),
                                    selected_regional.end());
          const GlobalRouteResult* selected_checkpoint = &best->coarse_global;
          // When the physically best plan has not improved, deliberately revisit another
          // retained topology.  The choice is deterministic and bounded; exact regional gates
          // still decide whether that alternative can replace the best checkpoint.
          if (stagnant_physical_feedback_passes > 0U && retained_global_portfolios.size() > 1U) {
            const auto alternative_index =
                stagnant_physical_feedback_passes % retained_global_portfolios.size();
            selected_checkpoint = &retained_global_portfolios[alternative_index];
            selected_diagnoses =
                probe_global_routability(*selected_checkpoint, config.routability_probe);
          }
          feedback_checkpoint = *selected_checkpoint;
          feedback_affected_nets =
              expand_feedback_nets(*selected_checkpoint, diagnosis_nets(selected_diagnoses));
        }
        feedback_probe.next_feedback_affected_nets = feedback_affected_nets.size();
      }
    }
    feedback_probes.push_back(std::move(feedback_probe));
    if (best) {
      attach_feedback_probes(*best);
      publish_feedback_selection(best->coarse_global);
    }
    if (best && best->complete()) {
      return *best;
    }
    // Stagnation no longer ends the search while an unevaluated epoch or retained topology can
    // still produce new information.  It only changes the next checkpoint above.  The bound is
    // reported at final exhaustion so callers can distinguish exhausted alternatives from the
    // old two-pass early exit.
    if (best && final_epoch &&
        stagnant_physical_feedback_passes >= config.max_stagnant_feedback_epochs) {
      best->diagnostic += "; interleaved physical-feedback alternatives exhausted after " +
                          std::to_string(stagnant_physical_feedback_passes) +
                          " non-improving epochs";
    }
    if ((has_regional_diagnoses || !global_diagnoses.empty()) && retry_available) {
      ++regional_feedback_passes;
      continue;
    }
    if (best) {
      best->routability_feedback_budget_exhausted =
          !global_diagnoses.empty() || has_regional_diagnoses;
      best->routability_knowledge = knowledge;
      best->global_feedback_passes = global_feedback_passes;
      best->regional_feedback_passes = regional_feedback_passes;
      best->cheap_physical_probes = pass + 1U;
      attach_feedback_probes(*best);
      return *best;
    }
  }
  RoutingPlanResult exhausted = best.value_or(RoutingPlanResult{});
  exhausted.routability_knowledge = std::move(knowledge);
  attach_feedback_probes(exhausted);
  exhausted.global_feedback_passes = global_feedback_passes;
  exhausted.regional_feedback_passes = regional_feedback_passes;
  exhausted.routability_feedback_budget_exhausted = true;
  return exhausted;
}

} // namespace flux
