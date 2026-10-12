#include "flux/core/routing_kernel.hpp"
#include "flux/core/detailed_router.hpp"

#include "flux/core/board_free_space.hpp"
#include "flux/core/cpu_worker_budget.hpp"
#include "flux/core/copper_geometry.hpp"
#include "flux/core/copper_transaction.hpp"
#include "flux/core/free_space_offset.hpp"
#include "flux/core/geometry.hpp"
#include "flux/core/navmesh_route.hpp"
#include "flux/core/pns_router.hpp"
#include "flux/core/routing_compute.hpp"
#include "flux/core/routing_rules.hpp"
#include "flux/core/spline_route.hpp"
#include "scaffold_connectivity_guard.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <numbers>
#include <numeric>
#include <optional>
#include <set>
#include <stop_token>
#include <string>
#include <string_view>
#include <thread>
#include <tuple>
#include <utility>
#include <vector>

namespace flux {
namespace {
void record_transaction_metrics(DetailedRouteResult&, const CopperTransactionReport&);


class MaterializationTimer {
public:
  explicit MaterializationTimer(std::uint64_t& total) : total_(total) {}
  ~MaterializationTimer() { finish(); }
  void finish() {
    if (finished_) return;
    total_ += static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::steady_clock::now() - start_).count());
    finished_ = true;
  }
private:
  std::uint64_t& total_;
  std::chrono::steady_clock::time_point start_{std::chrono::steady_clock::now()};
  bool finished_{false};
};

[[nodiscard]] bool collinear(Point first, Point middle, Point last) {
  const auto first_x = middle.x.value() - first.x.value();
  const auto first_y = middle.y.value() - first.y.value();
  const auto second_x = last.x.value() - middle.x.value();
  const auto second_y = last.y.value() - middle.y.value();
  const bool forward_x = first_x == 0 || second_x == 0 || (first_x < 0) == (second_x < 0);
  const bool forward_y = first_y == 0 || second_y == 0 || (first_y < 0) == (second_y < 0);
  return first_x * second_y == first_y * second_x && forward_x && forward_y;
}

[[nodiscard]] std::vector<GlobalRoutePoint>
simplify_path(const std::vector<GlobalRoutePoint>& input) {
  std::vector<GlobalRoutePoint> result;
  result.reserve(input.size());
  for (const auto& point : input) {
    if (!result.empty() && result.back() == point) {
      continue;
    }
    if (result.size() >= 2U && result[result.size() - 2U].layer == result.back().layer &&
        result.back().layer == point.layer &&
        collinear(result[result.size() - 2U].position, result.back().position, point.position)) {
      result.back() = point;
    } else {
      result.push_back(point);
    }
  }
  return result;
}

struct WidthAwarePath {
  std::vector<GlobalRoutePoint> points;
  std::vector<Nanometres> widths;
};

// Width changes are physical vertices. Collapse only forward-collinear edges with the same
// resolved width; layer transitions and every nominal/neck-down boundary are retained.
[[nodiscard]] std::optional<WidthAwarePath>
simplify_width_aware_points(const WidthAwarePath& input, std::size_t& removed_points) {
  if (input.points.empty() || input.widths.size() + 1U != input.points.size()) {
    return std::nullopt;
  }
  WidthAwarePath result;
  result.points.reserve(input.points.size());
  result.widths.reserve(input.widths.size());
  result.points.push_back(input.points.front());
  for (std::size_t index = 0U; index < input.widths.size(); ++index) {
    const auto width = input.widths[index];
    if (width.value() <= 0) {
      return std::nullopt;
    }
    const auto& point = input.points[index + 1U];
    if (result.points.back() == point) {
      ++removed_points;
      continue;
    }
    if (result.points.size() >= 2U && !result.widths.empty() && result.widths.back() == width &&
        result.points[result.points.size() - 2U].layer == result.points.back().layer &&
        result.points.back().layer == point.layer &&
        collinear(result.points[result.points.size() - 2U].position, result.points.back().position,
                  point.position)) {
      result.points.back() = point;
      ++removed_points;
      continue;
    }
    result.widths.push_back(width);
    result.points.push_back(point);
  }
  if (result.widths.size() + 1U != result.points.size()) {
    return std::nullopt;
  }
  return result;
}

[[nodiscard]] std::optional<WidthAwarePath> simplify_width_aware_path(const GlobalRoutePath& path,
                                                                      Nanometres nominal_width,
                                                                      std::size_t& removed_points) {
  if (path.points.empty() || path.segment_widths.size() + 1U != path.points.size()) {
    return std::nullopt;
  }
  WidthAwarePath resolved;
  resolved.points = path.points;
  resolved.widths.reserve(path.segment_widths.size());
  for (const auto selected : path.segment_widths) {
    const auto width = selected.value() > 0 ? selected : nominal_width;
    if (width.value() <= 0) {
      return std::nullopt;
    }
    resolved.widths.push_back(width);
  }
  return simplify_width_aware_points(resolved, removed_points);
}

[[nodiscard]] std::int64_t magnitude(std::int64_t value) { return value < 0 ? -value : value; }

[[nodiscard]] std::int64_t direction(std::int64_t value) {
  return value < 0 ? -1 : value > 0 ? 1 : 0;
}

[[nodiscard]] bool ordinary_track_angle(Point first, Point last) {
  const auto dx = magnitude(last.x.value() - first.x.value());
  const auto dy = magnitude(last.y.value() - first.y.value());
  return dx == 0 || dy == 0 || dx == dy;
}

[[nodiscard]] std::vector<GlobalRoutePoint>
pull_tight_45_degrees(const std::vector<GlobalRoutePoint>& input, std::size_t& corner_count) {
  const auto points = simplify_path(input);
  if (points.size() < 3U) {
    return points;
  }
  std::vector<GlobalRoutePoint> result;
  result.reserve(points.size() * 2U);
  result.push_back(points.front());
  for (std::size_t index = 1; index + 1U < points.size(); ++index) {
    const auto& previous = points[index - 1U];
    const auto& corner = points[index];
    const auto& next = points[index + 1U];
    if (previous.layer != corner.layer || corner.layer != next.layer) {
      result.push_back(corner);
      continue;
    }
    const auto incoming_x = corner.position.x.value() - previous.position.x.value();
    const auto incoming_y = corner.position.y.value() - previous.position.y.value();
    const auto outgoing_x = next.position.x.value() - corner.position.x.value();
    const auto outgoing_y = next.position.y.value() - corner.position.y.value();
    const bool incoming_orthogonal = (incoming_x == 0) != (incoming_y == 0);
    const bool outgoing_orthogonal = (outgoing_x == 0) != (outgoing_y == 0);
    const bool perpendicular =
        static_cast<long double>(incoming_x) * static_cast<long double>(outgoing_x) +
            static_cast<long double>(incoming_y) * static_cast<long double>(outgoing_y) ==
        0.0L;
    const auto incoming_length = std::max(magnitude(incoming_x), magnitude(incoming_y));
    const auto outgoing_length = std::max(magnitude(outgoing_x), magnitude(outgoing_y));
    const auto inset = std::min(incoming_length, outgoing_length) / 2;
    if (!incoming_orthogonal || !outgoing_orthogonal || !perpendicular || inset == 0) {
      result.push_back(corner);
      continue;
    }
    const Point entry{Nanometres{corner.position.x.value() - direction(incoming_x) * inset},
                      Nanometres{corner.position.y.value() - direction(incoming_y) * inset}};
    const Point exit{Nanometres{corner.position.x.value() + direction(outgoing_x) * inset},
                     Nanometres{corner.position.y.value() + direction(outgoing_y) * inset}};
    result.push_back(GlobalRoutePoint{entry, corner.layer});
    result.push_back(GlobalRoutePoint{exit, corner.layer});
    ++corner_count;
  }
  result.push_back(points.back());
  return simplify_path(result);
}

// Apply the same bounded 45-degree shortcut only within equal-width runs. The edge-aligned width
// vector is transformed alongside the points so a width transition can never move implicitly.
[[nodiscard]] std::optional<WidthAwarePath>
pull_tight_width_aware_45_degrees(const WidthAwarePath& input, std::size_t& corner_count,
                                  std::size_t& removed_points) {
  const auto simplified = simplify_width_aware_points(input, removed_points);
  if (!simplified) {
    return std::nullopt;
  }
  const auto& points = simplified->points;
  const auto& widths = simplified->widths;
  if (points.size() < 3U) {
    return simplified;
  }
  WidthAwarePath pulled;
  pulled.points.reserve(points.size() * 2U);
  pulled.widths.reserve(widths.size() * 2U);
  pulled.points.push_back(points.front());
  const auto append = [&pulled](GlobalRoutePoint point, Nanometres width) {
    pulled.widths.push_back(width);
    pulled.points.push_back(point);
  };
  for (std::size_t index = 1U; index + 1U < points.size(); ++index) {
    const auto& previous = points[index - 1U];
    const auto& corner = points[index];
    const auto& next = points[index + 1U];
    const auto incoming_width = widths[index - 1U];
    const auto outgoing_width = widths[index];
    if (incoming_width != outgoing_width || previous.layer != corner.layer ||
        corner.layer != next.layer) {
      append(corner, incoming_width);
      continue;
    }
    const auto incoming_x = corner.position.x.value() - previous.position.x.value();
    const auto incoming_y = corner.position.y.value() - previous.position.y.value();
    const auto outgoing_x = next.position.x.value() - corner.position.x.value();
    const auto outgoing_y = next.position.y.value() - corner.position.y.value();
    const bool incoming_orthogonal = (incoming_x == 0) != (incoming_y == 0);
    const bool outgoing_orthogonal = (outgoing_x == 0) != (outgoing_y == 0);
    const bool perpendicular =
        static_cast<long double>(incoming_x) * static_cast<long double>(outgoing_x) +
            static_cast<long double>(incoming_y) * static_cast<long double>(outgoing_y) ==
        0.0L;
    const auto incoming_length = std::max(magnitude(incoming_x), magnitude(incoming_y));
    const auto outgoing_length = std::max(magnitude(outgoing_x), magnitude(outgoing_y));
    const auto inset = std::min(incoming_length, outgoing_length) / 2;
    if (!incoming_orthogonal || !outgoing_orthogonal || !perpendicular || inset == 0) {
      append(corner, incoming_width);
      continue;
    }
    const Point entry{Nanometres{corner.position.x.value() - direction(incoming_x) * inset},
                      Nanometres{corner.position.y.value() - direction(incoming_y) * inset}};
    const Point exit{Nanometres{corner.position.x.value() + direction(outgoing_x) * inset},
                     Nanometres{corner.position.y.value() + direction(outgoing_y) * inset}};
    append(GlobalRoutePoint{entry, corner.layer}, incoming_width);
    append(GlobalRoutePoint{exit, corner.layer}, incoming_width);
    ++corner_count;
  }
  append(points.back(), widths.back());
  return simplify_width_aware_points(pulled, removed_points);
}

[[nodiscard]] long double local_refinement_objective(Point previous, Point point, Point next,
                                                     Point anchor, long double trust_nm) {
  const auto displacement = geometry::point_distance_nm(point, anchor);
  return geometry::point_distance_nm(previous, point) + geometry::point_distance_nm(point, next) +
         displacement * displacement / (2.0L * trust_nm);
}

[[nodiscard]] Point project_into_trust_region(Point point, Point start, Point end, Point anchor,
                                              long double trust_nm) {
  const auto segment_x = static_cast<long double>(end.x.value() - start.x.value());
  const auto segment_y = static_cast<long double>(end.y.value() - start.y.value());
  const auto length_squared = segment_x * segment_x + segment_y * segment_y;
  if (length_squared == 0.0L) {
    return point;
  }
  const auto point_x = static_cast<long double>(point.x.value() - start.x.value());
  const auto point_y = static_cast<long double>(point.y.value() - start.y.value());
  const auto projection =
      std::clamp((point_x * segment_x + point_y * segment_y) / length_squared, 0.0L, 1.0L);
  auto candidate_x = static_cast<long double>(start.x.value()) + projection * segment_x;
  auto candidate_y = static_cast<long double>(start.y.value()) + projection * segment_y;
  const auto anchor_dx = candidate_x - static_cast<long double>(anchor.x.value());
  const auto anchor_dy = candidate_y - static_cast<long double>(anchor.y.value());
  const auto displacement = std::hypot(anchor_dx, anchor_dy);
  if (displacement > trust_nm) {
    candidate_x = static_cast<long double>(anchor.x.value()) + anchor_dx * trust_nm / displacement;
    candidate_y = static_cast<long double>(anchor.y.value()) + anchor_dy * trust_nm / displacement;
  }
  return Point{Nanometres{static_cast<std::int64_t>(std::llround(candidate_x))},
               Nanometres{static_cast<std::int64_t>(std::llround(candidate_y))}};
}

[[nodiscard]] bool continuous_candidate_clear(const Board& board, const NetId& net,
                                              const LayerId& layer, Point previous, Point candidate,
                                              Point next, Nanometres width, std::size_t serial) {
  Board probe = board;
  const auto first_id = "flux-continuous-probe-a-" + std::to_string(serial);
  const auto second_id = "flux-continuous-probe-b-" + std::to_string(serial);
  probe.tracks.push_back(
      TrackSegment{ObjectId{first_id}, net, layer, previous, candidate, width, false});
  probe.tracks.push_back(
      TrackSegment{ObjectId{second_id}, net, layer, candidate, next, width, false});
  const auto report = check_copper_rules(probe);
  return std::none_of(
      report.violations.begin(), report.violations.end(), [&](const auto& violation) {
        if (violation.kind == CopperViolationKind::differential_pair_skew) {
          return false;
        }
        return std::find(violation.object_ids.begin(), violation.object_ids.end(), first_id) !=
                   violation.object_ids.end() ||
               std::find(violation.object_ids.begin(), violation.object_ids.end(), second_id) !=
                   violation.object_ids.end();
      });
}

[[nodiscard]] Point interpolate_backtracking_candidate(Point current, Point projected,
                                                       long double scale) {
  return Point{
      Nanometres{current.x.value() +
                 static_cast<std::int64_t>(std::llround(
                     static_cast<long double>(projected.x.value() - current.x.value()) * scale))},
      Nanometres{current.y.value() +
                 static_cast<std::int64_t>(std::llround(
                     static_cast<long double>(projected.y.value() - current.y.value()) * scale))}};
}

// Slicer-style centerline relaxation: alternate projections of each free vertex onto its
// neighbouring chord, constrained by a source-lane trust region. Floating point is proposal-only;
// quantized candidates undergo bounded exact-clearance line search before whole-board acceptance.
[[nodiscard]] std::vector<GlobalRoutePoint>
refine_path_continuously(const Board& board, const NetId& net, Nanometres track_width,
                         const std::vector<GlobalRoutePoint>& input,
                         const DetailedRouterConfig& config, DetailedRouteResult& route_result,
                         std::size_t& completed_iterations,
                         const std::vector<Nanometres>* segment_widths = nullptr) {
  auto result = segment_widths == nullptr ? simplify_path(input) : input;
  if (segment_widths != nullptr && segment_widths->size() + 1U != result.size()) {
    return {};
  }
  if (!config.continuous_refinement || config.continuous_refinement_iterations == 0U ||
      result.size() < 3U) {
    return result;
  }
  const auto anchors = result;
  std::vector<bool> moved(result.size(), false);
  bool budget_exhausted = false;
  for (std::size_t iteration = 0; iteration < config.continuous_refinement_iterations;
       ++iteration) {
    bool improved = false;
    for (std::size_t index = 1; index + 1U < result.size(); ++index) {
      if (result[index - 1U].layer != result[index].layer ||
          result[index].layer != result[index + 1U].layer ||
          (segment_widths != nullptr &&
           (*segment_widths)[index - 1U] != (*segment_widths)[index])) {
        continue;
      }
      auto trust_nm = static_cast<long double>(config.continuous_trust_region.value());
      if (trust_nm == 0.0L) {
        trust_nm =
            static_cast<long double>(std::min(
                geometry::segment_length_nm(anchors[index - 1U].position, anchors[index].position),
                geometry::segment_length_nm(anchors[index].position,
                                            anchors[index + 1U].position))) /
            2.0L;
      }
      if (trust_nm < 1.0L) {
        continue;
      }
      const auto projected =
          project_into_trust_region(result[index].position, result[index - 1U].position,
                                    result[index + 1U].position, anchors[index].position, trust_nm);
      if (projected == result[index].position) {
        continue;
      }
      const auto current = result[index].position;
      const auto before = local_refinement_objective(result[index - 1U].position, current,
                                                     result[index + 1U].position,
                                                     anchors[index].position, trust_nm);
      Point previous_trial = current;
      auto scale = 1.0L;
      for (std::size_t step = 0; step <= config.continuous_backtracking_steps; ++step) {
        const auto candidate = interpolate_backtracking_candidate(current, projected, scale);
        scale *= 0.5L;
        if (candidate == current) {
          break;
        }
        if (candidate == previous_trial) {
          continue;
        }
        previous_trial = candidate;
        const auto after = local_refinement_objective(result[index - 1U].position, candidate,
                                                      result[index + 1U].position,
                                                      anchors[index].position, trust_nm);
        if (after + 0.5L >= before) {
          continue;
        }
        if (route_result.attempted_continuous_refinement_candidates >=
            config.max_continuous_exact_candidate_checks) {
          ++route_result.withheld_continuous_refinement_candidates;
          route_result.continuous_refinement_budget_exhausted = true;
          budget_exhausted = true;
          break;
        }
        ++route_result.attempted_continuous_refinement_candidates;
        const auto local_width = segment_widths == nullptr ? track_width : (*segment_widths)[index];
        if (!continuous_candidate_clear(board, net, result[index].layer,
                                        result[index - 1U].position, candidate,
                                        result[index + 1U].position, local_width,
                                        route_result.attempted_continuous_refinement_candidates)) {
          ++route_result.rejected_continuous_refinement_candidates;
          continue;
        }
        ++route_result.accepted_continuous_refinement_candidates;
        result[index].position = candidate;
        moved[index] = true;
        improved = true;
        break;
      }
      if (budget_exhausted) {
        break;
      }
    }
    ++completed_iterations;
    if (!improved || budget_exhausted) {
      break;
    }
  }
  route_result.continuously_refined_points +=
      static_cast<std::size_t>(std::count(moved.begin(), moved.end(), true));
  return segment_widths == nullptr ? simplify_path(result) : result;
}

[[nodiscard]] ObjectId unique_id(std::string base, std::set<std::string>& used_ids) {
  if (used_ids.insert(base).second) {
    return ObjectId{std::move(base)};
  }
  for (std::size_t suffix = 1;; ++suffix) {
    auto candidate = base + "-" + std::to_string(suffix);
    if (used_ids.insert(candidate).second) {
      return ObjectId{std::move(candidate)};
    }
  }
}

[[nodiscard]] std::set<std::string> object_ids(const Board& board) {
  std::set<std::string> result;
  for (const auto& footprint : board.footprints) {
    result.insert(footprint.id.value());
    for (const auto& pad : footprint.pads) {
      result.insert(pad.id.value());
    }
    for (const auto& keepout : footprint.keepouts) {
      result.insert(keepout.id.value());
    }
  }
  for (const auto& track : board.tracks) {
    result.insert(track.id.value());
  }
  for (const auto& arc : board.track_arcs) {
    result.insert(arc.id.value());
  }
  for (const auto& bezier : board.track_beziers) {
    result.insert(bezier.id.value());
  }
  for (const auto& via : board.vias) {
    result.insert(via.id.value());
  }
  for (const auto& zone : board.zones) {
    result.insert(zone.id.value());
  }
  for (const auto& keepout : board.keepouts) {
    result.insert(keepout.id.value());
  }
  return result;
}

[[nodiscard]] std::vector<LayerId> via_span(const Board& board, const LayerId& first,
                                            const LayerId& last) {
  std::map<LayerId, std::uint32_t> order;
  for (const auto& layer : board.layers) {
    if (layer.kind == LayerKind::copper) {
      order.emplace(layer.id, layer.stack_order);
    }
  }
  if (!order.contains(first) || !order.contains(last)) {
    return {};
  }
  const auto low = std::min(order.at(first), order.at(last));
  const auto high = std::max(order.at(first), order.at(last));
  std::vector<std::pair<std::uint32_t, LayerId>> selected;
  for (const auto& [layer, stack_order] : order) {
    if (stack_order >= low && stack_order <= high) {
      selected.emplace_back(stack_order, layer);
    }
  }
  std::sort(selected.begin(), selected.end());
  std::vector<LayerId> result;
  result.reserve(selected.size());
  for (auto& [stack_order, layer] : selected) {
    static_cast<void>(stack_order);
    result.push_back(std::move(layer));
  }
  return result;
}

[[nodiscard]] ResolvedNetRoutingRules materialization_rules(const Board& board, const NetId& net,
                                                            const DetailedRouterConfig& config) {
  auto result = resolve_net_routing_rules(board, net);
  if (config.track_width.value() > 0) {
    result.track_width = config.track_width;
  }
  if (config.via_diameter.value() > 0) {
    result.via_diameter = config.via_diameter;
  }
  if (config.via_drill.value() > 0) {
    result.via_drill = config.via_drill;
  }
  return result;
}

void remove_unlocked_routing(Board& board, const std::set<NetId>& routed_nets,
                             DetailedRouteResult& result) {
  const auto old_track_count = board.tracks.size();
  std::erase_if(board.tracks, [&routed_nets](const auto& track) {
    return !track.locked && routed_nets.contains(track.net);
  });
  result.removed_tracks = old_track_count - board.tracks.size();
  const auto old_arc_count = board.track_arcs.size();
  std::erase_if(board.track_arcs, [&routed_nets](const auto& arc) {
    return !arc.locked && routed_nets.contains(arc.net);
  });
  result.removed_track_arcs = old_arc_count - board.track_arcs.size();
  const auto old_bezier_count = board.track_beziers.size();
  std::erase_if(board.track_beziers, [&routed_nets](const auto& bezier) {
    return !bezier.locked && routed_nets.contains(bezier.net);
  });
  result.removed_track_beziers = old_bezier_count - board.track_beziers.size();
  const auto old_via_count = board.vias.size();
  std::erase_if(board.vias, [&routed_nets](const auto& via) {
    return !via.locked && routed_nets.contains(via.net);
  });
  result.removed_vias = old_via_count - board.vias.size();
}

[[nodiscard]] const RoutingVoronoiLaneAssignment*
find_lane_assignment(const GlobalRouteResult& global, std::size_t resource_id, const NetId& net) {
  const auto found =
      std::find_if(global.voronoi_lane_assignments.begin(), global.voronoi_lane_assignments.end(),
                   [resource_id, &net](const auto& lane) {
                     return lane.resource_id == resource_id && lane.net == net;
                   });
  return found == global.voronoi_lane_assignments.end() ? nullptr : &*found;
}

[[nodiscard]] const RoutingVoronoiResourceUsage*
find_resource_usage(const GlobalRouteResult& global, std::size_t resource_id) {
  const auto found =
      std::find_if(global.voronoi_resource_usage.begin(), global.voronoi_resource_usage.end(),
                   [resource_id](const auto& usage) { return usage.id == resource_id; });
  return found == global.voronoi_resource_usage.end() ? nullptr : &*found;
}

[[nodiscard]] const RoutingVoronoiLaneTransition*
find_lane_transition(const GlobalRouteResult& global, std::size_t first_resource,
                     std::size_t second_resource, const NetId& net,
                     const std::optional<Point>& direct_portal) {
  const auto found = std::find_if(
      global.voronoi_lane_transitions.begin(), global.voronoi_lane_transitions.end(),
      [first_resource, second_resource, &net, &direct_portal](const auto& transition) {
        const auto resources_match = (transition.first_resource_id == first_resource &&
                                      transition.second_resource_id == second_resource) ||
                                     (transition.first_resource_id == second_resource &&
                                      transition.second_resource_id == first_resource);
        const auto portal_matches = direct_portal ? transition.direct_portal_bridge &&
                                                        transition.medial_junction == *direct_portal
                                                  : !transition.direct_portal_bridge;
        return transition.net == net && resources_match && portal_matches;
      });
  return found == global.voronoi_lane_transitions.end() ? nullptr : &*found;
}

[[nodiscard]] std::optional<Point>
transition_endpoint(const RoutingVoronoiLaneTransition& transition, std::size_t resource_id) {
  if (transition.first_resource_id == resource_id) {
    return transition.first_lane_endpoint;
  }
  if (transition.second_resource_id == resource_id) {
    return transition.second_lane_endpoint;
  }
  return std::nullopt;
}

[[nodiscard]] std::optional<Point> other_lane_endpoint(const RoutingVoronoiLaneAssignment& lane,
                                                       Point endpoint) {
  if (lane.shifted_start == endpoint) {
    return lane.shifted_end;
  }
  if (lane.shifted_end == endpoint) {
    return lane.shifted_start;
  }
  return std::nullopt;
}

struct TopologyLockedLanePath {
  struct PackedRun {
    std::size_t first_index{0};
    std::size_t last_index{0};
    LayerId layer;
    std::vector<std::size_t> resource_ids;
  };
  std::vector<GlobalRoutePoint> points;
  std::vector<PackedRun> packed_runs;
  bool multilayer{false};
  bool coupled_member{false};
  bool preserved_terminal_access{false};
  std::size_t first_packed_index{0};
  std::size_t last_packed_index{0};
};

[[nodiscard]] std::optional<Point>
find_layer_transition_anchor(const std::vector<GlobalRoutePoint>& points, const LayerId& from,
                             const LayerId& to, std::size_t& cursor) {
  for (std::size_t index = cursor; index + 1U < points.size(); ++index) {
    if (points[index].layer != from) {
      continue;
    }
    const auto position = points[index].position;
    auto transition = index;
    while (transition + 1U < points.size() && points[transition + 1U].position == position) {
      ++transition;
      if (points[transition].layer == to) {
        cursor = transition;
        return position;
      }
    }
  }
  return std::nullopt;
}

[[nodiscard]] std::optional<TopologyLockedLanePath>
topology_locked_lane_points(const GlobalRouteResult& global, const GlobalRoutePath& path,
                            Nanometres track_width, std::size_t max_terminal_access_points) {
  if (!path.medial_topology_complete() || path.points.size() < 2U ||
      path.medial_resource_sequence.empty()) {
    return std::nullopt;
  }
  std::vector<const RoutingVoronoiLaneAssignment*> lanes;
  std::vector<const RoutingVoronoiResourceUsage*> resources;
  lanes.reserve(path.medial_resource_sequence.size());
  resources.reserve(path.medial_resource_sequence.size());
  for (const auto resource_id : path.medial_resource_sequence) {
    const auto* lane = find_lane_assignment(global, resource_id, path.net);
    const auto* resource = find_resource_usage(global, resource_id);
    if (lane == nullptr || resource == nullptr || !lane->static_clearance_checked ||
        lane->track_width != track_width) {
      return std::nullopt;
    }
    lanes.push_back(lane);
    resources.push_back(resource);
  }

  struct LaneRun {
    std::size_t begin{0};
    std::size_t end{0};
    LayerId layer;
  };
  std::vector<LaneRun> runs;
  for (std::size_t begin = 0U; begin < resources.size();) {
    auto end = begin + 1U;
    while (end < resources.size() && resources[end]->layer == resources[begin]->layer) {
      ++end;
    }
    runs.push_back(LaneRun{begin, end, resources[begin]->layer});
    begin = end;
  }
  const bool coupled_member = std::any_of(lanes.begin(), lanes.end(), [](const auto* lane) {
    return lane->differential_pair_member || lane->routing_bus.has_value();
  });

  std::vector<Point> anchors(runs.size() + 1U);
  std::vector<std::size_t> path_run_boundaries(runs.size() + 1U, 0U);
  path_run_boundaries.back() = path.points.size() - 1U;
  std::size_t transition_cursor = 0U;
  for (std::size_t index = 0U; index + 1U < runs.size(); ++index) {
    const auto anchor = find_layer_transition_anchor(path.points, runs[index].layer,
                                                     runs[index + 1U].layer, transition_cursor);
    if (!anchor) {
      return std::nullopt;
    }
    anchors[index + 1U] = *anchor;
    path_run_boundaries[index + 1U] = transition_cursor;
  }

  const auto closest_lane_path_point =
      [&path](const RoutingVoronoiLaneAssignment& lane, const LayerId& layer, std::size_t begin,
              std::size_t end, bool prefer_last) -> std::optional<std::size_t> {
    std::optional<std::size_t> selected;
    auto selected_distance = std::numeric_limits<long double>::infinity();
    for (auto index = begin; index <= end && index < path.points.size(); ++index) {
      if (path.points[index].layer != layer) {
        continue;
      }
      const auto distance =
          std::min(geometry::point_distance_nm(path.points[index].position, lane.shifted_start),
                   geometry::point_distance_nm(path.points[index].position, lane.shifted_end));
      if (!selected || distance < selected_distance ||
          (distance == selected_distance &&
           ((prefer_last && index > *selected) || (!prefer_last && index < *selected)))) {
        selected = index;
        selected_distance = distance;
      }
    }
    return selected;
  };
  const auto first_splice = closest_lane_path_point(
      *lanes.front(), runs.front().layer, 0U,
      runs.size() > 1U ? path_run_boundaries[1U] : path.points.size() - 1U, false);
  const auto last_splice = closest_lane_path_point(
      *lanes.back(), runs.back().layer,
      runs.size() > 1U ? path_run_boundaries[runs.size() - 1U] : 0U, path.points.size() - 1U, true);
  if (!first_splice || !last_splice || *first_splice > *last_splice) {
    return std::nullopt;
  }
  const auto retained_points = *first_splice + 1U + path.points.size() - *last_splice;
  if (retained_points > max_terminal_access_points) {
    return std::nullopt;
  }
  anchors.front() = path.points[*first_splice].position;
  anchors.back() = path.points[*last_splice].position;

  TopologyLockedLanePath result;
  const auto project_to_segment = [](Point point, Point start, Point end) {
    const auto segment_x = static_cast<long double>(end.x.value() - start.x.value());
    const auto segment_y = static_cast<long double>(end.y.value() - start.y.value());
    const auto length_squared = segment_x * segment_x + segment_y * segment_y;
    if (length_squared == 0.0L) {
      return start;
    }
    const auto point_x = static_cast<long double>(point.x.value() - start.x.value());
    const auto point_y = static_cast<long double>(point.y.value() - start.y.value());
    const auto projection =
        std::clamp((point_x * segment_x + point_y * segment_y) / length_squared, 0.0L, 1.0L);
    return Point{Nanometres{static_cast<std::int64_t>(std::llround(
                     static_cast<long double>(start.x.value()) + projection * segment_x))},
                 Nanometres{static_cast<std::int64_t>(std::llround(
                     static_cast<long double>(start.y.value()) + projection * segment_y))}};
  };
  const auto append = [&result](Point point, const LayerId& point_layer) {
    if (result.points.empty() || result.points.back().position != point ||
        result.points.back().layer != point_layer) {
      result.points.push_back(GlobalRoutePoint{point, point_layer});
    }
    return result.points.size() - 1U;
  };
  for (std::size_t index = 0U; index <= *first_splice; ++index) {
    append(path.points[index].position, path.points[index].layer);
  }
  for (std::size_t run_index = 0U; run_index < runs.size(); ++run_index) {
    const auto& run = runs[run_index];
    const auto count = run.end - run.begin;
    std::vector<const RoutingVoronoiLaneTransition*> transitions;
    transitions.reserve(count - 1U);
    for (std::size_t index = run.begin + 1U; index < run.end; ++index) {
      const auto* transition = find_lane_transition(
          global, path.medial_resource_sequence[index - 1U], path.medial_resource_sequence[index],
          path.net,
          path.medial_transition_portals.size() + 1U == path.medial_resource_sequence.size()
              ? path.medial_transition_portals[index - 1U]
              : std::nullopt);
      if (transition == nullptr || transition->layer != run.layer ||
          !transition->static_clearance_checked || !transition->mutual_clearance_checked) {
        return std::nullopt;
      }
      transitions.push_back(transition);
    }

    std::vector<Point> entries(count);
    std::vector<Point> exits(count);
    if (count == 1U) {
      const auto* lane = lanes[run.begin];
      entries.front() =
          project_to_segment(anchors[run_index], lane->shifted_start, lane->shifted_end);
      exits.front() =
          project_to_segment(anchors[run_index + 1U], lane->shifted_start, lane->shifted_end);
    } else {
      for (std::size_t index = 0U; index < transitions.size(); ++index) {
        const auto sequence_index = run.begin + index;
        const auto current =
            transition_endpoint(*transitions[index], path.medial_resource_sequence[sequence_index]);
        const auto next = transition_endpoint(*transitions[index],
                                              path.medial_resource_sequence[sequence_index + 1U]);
        if (!current || !next) {
          return std::nullopt;
        }
        exits[index] = *current;
        entries[index + 1U] = *next;
      }
      const auto first_entry = other_lane_endpoint(*lanes[run.begin], exits.front());
      const auto last_exit = other_lane_endpoint(*lanes[run.end - 1U], entries.back());
      if (!first_entry || !last_exit) {
        return std::nullopt;
      }
      entries.front() = *first_entry;
      exits.back() = *last_exit;
      for (std::size_t index = 1U; index + 1U < count; ++index) {
        const auto opposite = other_lane_endpoint(*lanes[run.begin + index], entries[index]);
        if (entries[index] == exits[index] || !opposite || *opposite != exits[index]) {
          return std::nullopt;
        }
      }
      entries.front() = project_to_segment(anchors[run_index], entries.front(), exits.front());
      exits.back() = project_to_segment(anchors[run_index + 1U], entries.back(), exits.back());
    }

    append(anchors[run_index], run.layer);
    std::optional<std::size_t> packed_run_first;
    std::size_t packed_run_last = 0U;
    const auto record_packed = [&](std::size_t packed_index) {
      if (!packed_run_first) {
        packed_run_first = packed_index;
      }
      packed_run_last = packed_index;
      if (result.first_packed_index == 0U) {
        result.first_packed_index = packed_index;
      }
      result.last_packed_index = packed_index;
    };
    for (std::size_t index = 0U; index < count; ++index) {
      const bool entered_through_miter = index > 0U && transitions[index - 1U]->miter.has_value();
      if (!entered_through_miter) {
        record_packed(append(entries[index], run.layer));
      }
      if (index < transitions.size() && transitions[index]->miter) {
        record_packed(append(*transitions[index]->miter, run.layer));
      } else {
        record_packed(append(exits[index], run.layer));
      }
    }
    if (!packed_run_first) {
      return std::nullopt;
    }
    result.packed_runs.push_back(TopologyLockedLanePath::PackedRun{
        *packed_run_first, packed_run_last, run.layer,
        std::vector<std::size_t>(
            path.medial_resource_sequence.begin() + static_cast<std::ptrdiff_t>(run.begin),
            path.medial_resource_sequence.begin() + static_cast<std::ptrdiff_t>(run.end))});
    append(anchors[run_index + 1U], run.layer);
  }
  for (auto index = *last_splice; index < path.points.size(); ++index) {
    append(path.points[index].position, path.points[index].layer);
  }
  result.multilayer = std::adjacent_find(result.points.begin(), result.points.end(),
                                         [](const auto& lhs, const auto& rhs) {
                                           return lhs.layer != rhs.layer;
                                         }) != result.points.end();
  result.coupled_member = coupled_member;
  result.preserved_terminal_access = true;
  return result.points.size() >= 2U && !result.packed_runs.empty() &&
                 result.first_packed_index > 0U &&
                 result.last_packed_index >= result.first_packed_index
             ? std::optional<TopologyLockedLanePath>{std::move(result)}
             : std::nullopt;
}

struct CoupledRoutingGroup {
  std::vector<NetId> members;
  Nanometres minimum_adjacent_track_gap;
  Nanometres minimum_adjacent_via_gap;
  bool ordered{false};
};

[[nodiscard]] std::vector<CoupledRoutingGroup>
coupled_routing_groups(const Board& board, const std::set<NetId>& routed_nets) {
  std::vector<CoupledRoutingGroup> result;
  for (const auto& pair : board.differential_pairs) {
    if (!routed_nets.contains(pair.positive_net) || !routed_nets.contains(pair.negative_net)) {
      continue;
    }
    const auto positive = resolve_net_routing_rules(board, pair.positive_net);
    const auto negative = resolve_net_routing_rules(board, pair.negative_net);
    result.push_back(CoupledRoutingGroup{
        {pair.positive_net, pair.negative_net},
        std::max({positive.differential_pair_gap, negative.differential_pair_gap,
                  pair.minimum_gap.value_or(Nanometres{})}),
        std::max(positive.differential_pair_via_gap, negative.differential_pair_via_gap),
        false});
  }
  for (const auto& bus : board.routing_buses) {
    if (!std::all_of(bus.ordered_nets.begin(), bus.ordered_nets.end(),
                     [&routed_nets](const auto& net) { return routed_nets.contains(net); })) {
      continue;
    }
    result.push_back(CoupledRoutingGroup{bus.ordered_nets, bus.minimum_gap.value_or(Nanometres{}),
                                         bus.minimum_gap.value_or(Nanometres{}), true});
  }
  return result;
}

struct CoupledTerminalFanoutSide {
  std::vector<std::vector<GlobalRoutePoint>> polylines;
  std::vector<std::size_t> packed_indices;
  std::vector<std::size_t> anchor_indices;
};

[[nodiscard]] std::optional<std::size_t>
packed_run_anchor_index(const TopologyLockedLanePath& path, std::size_t run_index, bool at_start) {
  if (run_index >= path.packed_runs.size()) {
    return std::nullopt;
  }
  const auto& run = path.packed_runs[run_index];
  auto index = at_start ? run.first_index : run.last_index;
  if (index >= path.points.size() || path.points[index].layer != run.layer) {
    return std::nullopt;
  }
  if (at_start) {
    while (index > 0U && path.points[index - 1U].layer == run.layer) {
      --index;
    }
  } else {
    while (index + 1U < path.points.size() && path.points[index + 1U].layer == run.layer) {
      ++index;
    }
  }
  return index;
}

[[nodiscard]] std::optional<CoupledTerminalFanoutSide>
coupled_terminal_fanout_side_for_runs(const std::vector<TopologyLockedLanePath*>& paths,
                                      bool at_start, bool trim_dead_tail, bool reverse_staircase,
                                      const std::vector<std::size_t>& run_indices) {
  if (paths.size() < 2U || run_indices.size() != paths.size()) {
    return std::nullopt;
  }
  struct Endpoint {
    Point terminal;
    Point lane;
    Point inside;
    LayerId layer;
    bool horizontal{false};
    std::int64_t outward_direction{0};
    std::size_t packed_index{0};
  };
  std::vector<Endpoint> endpoints;
  endpoints.reserve(paths.size());
  for (std::size_t member = 0U; member < paths.size(); ++member) {
    const auto* path = paths[member];
    const auto run_index = run_indices[member];
    if (path == nullptr || path->points.size() < 3U || run_index >= path->packed_runs.size()) {
      return std::nullopt;
    }
    const auto& packed_run = path->packed_runs[run_index];
    auto packed_index = at_start ? packed_run.first_index : packed_run.last_index;
    const auto terminal_index = packed_run_anchor_index(*path, run_index, at_start);
    if (!terminal_index) {
      return std::nullopt;
    }
    const auto step = at_start ? 1 : -1;
    const auto& terminal = path->points[*terminal_index];
    bool found_terminal_facing_tangent = false;
    while (!found_terminal_facing_tangent) {
      const auto append_terminal_axis_endpoint = [&](const GlobalRoutePoint& lane) {
        if (terminal.layer != lane.layer) {
          return false;
        }
        const auto terminal_x = terminal.position.x.value() - lane.position.x.value();
        const auto terminal_y = terminal.position.y.value() - lane.position.y.value();
        const bool horizontal = magnitude(terminal_x) >= magnitude(terminal_y);
        const auto outward = direction(horizontal ? terminal_x : terminal_y);
        if (outward == 0) {
          return false;
        }
        endpoints.push_back(Endpoint{terminal.position, lane.position, lane.position, lane.layer,
                                     horizontal, outward, packed_index});
        return true;
      };
      if (!trim_dead_tail) {
        if (!append_terminal_axis_endpoint(path->points[packed_index])) {
          return std::nullopt;
        }
        found_terminal_facing_tangent = true;
        continue;
      }
      auto inside_index = static_cast<std::int64_t>(packed_index) + step;
      while (inside_index >= 0 && inside_index < static_cast<std::int64_t>(path->points.size()) &&
             path->points[static_cast<std::size_t>(inside_index)].position ==
                 path->points[packed_index].position) {
        inside_index += step;
      }
      if (inside_index < static_cast<std::int64_t>(packed_run.first_index) ||
          inside_index > static_cast<std::int64_t>(packed_run.last_index)) {
        if (!append_terminal_axis_endpoint(path->points[packed_index])) {
          return std::nullopt;
        }
        found_terminal_facing_tangent = true;
        continue;
      }
      const auto& lane = path->points[packed_index];
      const auto& inside = path->points[static_cast<std::size_t>(inside_index)];
      if (terminal.layer != lane.layer) {
        return std::nullopt;
      }
      if (inside.layer != lane.layer) {
        if (!append_terminal_axis_endpoint(lane)) {
          return std::nullopt;
        }
        found_terminal_facing_tangent = true;
        continue;
      }
      const auto tangent_x = lane.position.x.value() - inside.position.x.value();
      const auto tangent_y = lane.position.y.value() - inside.position.y.value();
      const bool horizontal = magnitude(tangent_x) >= magnitude(tangent_y);
      const auto outward = direction(horizontal ? tangent_x : tangent_y);
      const auto terminal_direction =
          direction(horizontal ? terminal.position.x.value() - lane.position.x.value()
                               : terminal.position.y.value() - lane.position.y.value());
      if (outward != 0 && terminal_direction == outward) {
        endpoints.push_back(Endpoint{terminal.position, lane.position, inside.position, lane.layer,
                                     horizontal, outward, packed_index});
        found_terminal_facing_tangent = true;
      } else {
        packed_index = static_cast<std::size_t>(inside_index);
      }
    }
  }
  auto horizontal = endpoints.front().horizontal;
  auto outward = endpoints.front().outward_direction;
  bool extended_staircase = false;
  if (!std::all_of(endpoints.begin(), endpoints.end(), [horizontal, outward](const auto& endpoint) {
        return endpoint.horizontal == horizontal && endpoint.outward_direction == outward;
      })) {
    const auto common_direction = [&endpoints](bool use_x) -> std::optional<std::int64_t> {
      std::optional<std::int64_t> selected;
      for (const auto& endpoint : endpoints) {
        const auto delta = use_x ? endpoint.terminal.x.value() - endpoint.lane.x.value()
                                 : endpoint.terminal.y.value() - endpoint.lane.y.value();
        const auto candidate = direction(delta);
        if (candidate == 0 || (selected && *selected != candidate)) {
          return std::nullopt;
        }
        selected = candidate;
      }
      return selected;
    };
    const auto horizontal_direction = common_direction(true);
    const auto vertical_direction = common_direction(false);
    if (horizontal_direction) {
      horizontal = true;
      outward = *horizontal_direction;
    } else if (vertical_direction) {
      horizontal = false;
      outward = *vertical_direction;
    } else {
      return std::nullopt;
    }
    extended_staircase = true;
  }

  const auto major = [horizontal](Point point) {
    return horizontal ? point.x.value() : point.y.value();
  };
  const auto cross = [horizontal](Point point) {
    return horizontal ? point.y.value() : point.x.value();
  };
  std::vector<std::pair<std::int64_t, std::size_t>> lane_order;
  std::vector<std::pair<std::int64_t, std::size_t>> terminal_order;
  auto available = std::numeric_limits<std::int64_t>::max();
  std::int64_t cross_excursion = 0;
  for (std::size_t member = 0U; member < endpoints.size(); ++member) {
    lane_order.emplace_back(cross(endpoints[member].lane), member);
    terminal_order.emplace_back(cross(endpoints[member].terminal), member);
    const auto distance =
        (major(endpoints[member].terminal) - major(endpoints[member].lane)) * outward;
    if (distance <= 0) {
      return std::nullopt;
    }
    available = std::min(available, distance);
    cross_excursion = std::max(cross_excursion, magnitude(cross(endpoints[member].terminal) -
                                                          cross(endpoints[member].lane)));
  }
  std::sort(lane_order.begin(), lane_order.end());
  std::sort(terminal_order.begin(), terminal_order.end());
  for (std::size_t rank = 0U; rank < lane_order.size(); ++rank) {
    if ((rank > 0U && lane_order[rank].first == lane_order[rank - 1U].first) ||
        (rank > 0U && terminal_order[rank].first == terminal_order[rank - 1U].first) ||
        lane_order[rank].second != terminal_order[rank].second) {
      return std::nullopt;
    }
  }
  if (available <= static_cast<std::int64_t>(endpoints.size())) {
    return std::nullopt;
  }
  if (extended_staircase) {
    if (cross_excursion > std::numeric_limits<std::int64_t>::max() - available) {
      return std::nullopt;
    }
    available += cross_excursion;
  }
  std::vector<std::size_t> rank_by_member(endpoints.size(), 0U);
  for (std::size_t rank = 0U; rank < lane_order.size(); ++rank) {
    rank_by_member[lane_order[rank].second] = rank;
  }
  const auto terminal_span = terminal_order.back().first - terminal_order.front().first;
  const auto lane_span = lane_order.back().first - lane_order.front().first;
  std::optional<std::int64_t> common_cross_direction;
  bool common_cross_motion = true;
  for (const auto& endpoint : endpoints) {
    const auto candidate = direction(cross(endpoint.lane) - cross(endpoint.terminal));
    if (candidate == 0 || (common_cross_direction && *common_cross_direction != candidate)) {
      common_cross_motion = false;
      break;
    }
    common_cross_direction = candidate;
  }
  std::optional<std::size_t> cross_first_member;
  if (common_cross_motion && common_cross_direction && terminal_span > lane_span) {
    const auto rank = *common_cross_direction < 0 ? 0U : lane_order.size() - 1U;
    cross_first_member = lane_order[rank].second;
  }

  CoupledTerminalFanoutSide result;
  result.polylines.resize(paths.size());
  result.packed_indices.resize(paths.size());
  result.anchor_indices.resize(paths.size());
  for (std::size_t member = 0U; member < endpoints.size(); ++member) {
    const auto rank =
        reverse_staircase ? endpoints.size() - rank_by_member[member] : rank_by_member[member] + 1U;
    const auto distance = static_cast<long double>(available) * static_cast<long double>(rank) /
                          static_cast<long double>(endpoints.size() + 1U);
    const auto station = static_cast<long double>(major(endpoints[member].lane)) +
                         static_cast<long double>(outward) * distance;
    constexpr auto low = static_cast<long double>(std::numeric_limits<std::int64_t>::min());
    constexpr auto high = static_cast<long double>(std::numeric_limits<std::int64_t>::max());
    if (!std::isfinite(station) || station < low || station > high) {
      return std::nullopt;
    }
    const auto station_nm = static_cast<std::int64_t>(std::llround(station));
    const auto make_point = [horizontal](std::int64_t major_value, std::int64_t cross_value) {
      return horizontal ? Point{Nanometres{major_value}, Nanometres{cross_value}}
                        : Point{Nanometres{cross_value}, Nanometres{major_value}};
    };
    auto& polyline = result.polylines[member];
    result.packed_indices[member] = endpoints[member].packed_index;
    result.anchor_indices[member] =
        *packed_run_anchor_index(*paths[member], run_indices[member], at_start);
    const auto append = [&polyline, &endpoints, member](Point point) {
      if (polyline.empty() || polyline.back().position != point) {
        polyline.push_back(GlobalRoutePoint{point, endpoints[member].layer});
      }
    };
    append(endpoints[member].terminal);
    if (cross_first_member == member) {
      append(make_point(major(endpoints[member].terminal), cross(endpoints[member].lane)));
    } else {
      append(make_point(station_nm, cross(endpoints[member].terminal)));
    }
    append(make_point(station_nm, cross(endpoints[member].lane)));
    append(endpoints[member].lane);
  }
  return result;
}

[[nodiscard]] std::optional<CoupledTerminalFanoutSide>
coupled_terminal_fanout_side(const std::vector<TopologyLockedLanePath*>& paths, bool at_start,
                             bool trim_dead_tail, bool reverse_staircase, std::size_t run_index) {
  return coupled_terminal_fanout_side_for_runs(paths, at_start, trim_dead_tail, reverse_staircase,
                                               std::vector<std::size_t>(paths.size(), run_index));
}

[[nodiscard]] std::optional<CoupledTerminalFanoutSide>
existing_terminal_fanout_side_for_runs(const std::vector<TopologyLockedLanePath*>& paths,
                                       bool at_start, const std::vector<std::size_t>& run_indices) {
  if (run_indices.size() != paths.size()) {
    return std::nullopt;
  }
  CoupledTerminalFanoutSide result;
  result.polylines.reserve(paths.size());
  result.packed_indices.reserve(paths.size());
  result.anchor_indices.reserve(paths.size());
  for (std::size_t member = 0U; member < paths.size(); ++member) {
    const auto* path = paths[member];
    const auto run_index = run_indices[member];
    if (path == nullptr || run_index >= path->packed_runs.size()) {
      return std::nullopt;
    }
    const auto& run = path->packed_runs[run_index];
    const auto anchor = packed_run_anchor_index(*path, run_index, at_start);
    if (!anchor) {
      return std::nullopt;
    }
    std::vector<GlobalRoutePoint> polyline;
    if (at_start) {
      polyline.assign(path->points.begin() + static_cast<std::ptrdiff_t>(*anchor),
                      path->points.begin() + static_cast<std::ptrdiff_t>(run.first_index + 1U));
    } else {
      for (auto index = *anchor + 1U; index > run.last_index; --index) {
        polyline.push_back(path->points[index - 1U]);
      }
    }
    if (polyline.empty()) {
      return std::nullopt;
    }
    result.polylines.push_back(std::move(polyline));
    result.packed_indices.push_back(at_start ? run.first_index : run.last_index);
    result.anchor_indices.push_back(*anchor);
  }
  return result;
}

[[nodiscard]] std::optional<CoupledTerminalFanoutSide>
existing_terminal_fanout_side(const std::vector<TopologyLockedLanePath*>& paths, bool at_start,
                              std::size_t run_index) {
  return existing_terminal_fanout_side_for_runs(paths, at_start,
                                                std::vector<std::size_t>(paths.size(), run_index));
}

[[nodiscard]] bool
coupled_path_geometry_clear(const Board& board, const CoupledRoutingGroup& group,
                            const std::vector<TopologyLockedLanePath>& paths,
                            const std::map<NetId, ResolvedNetRoutingRules>& dimensions,
                            bool enforce_pair_skew = true) {
  if (paths.size() != group.members.size()) {
    return false;
  }
  struct Segment {
    Point start;
    Point end;
    LayerId layer;
    Nanometres width;
  };
  std::vector<std::vector<Segment>> segments(paths.size());
  Board probe = board;
  for (std::size_t member = 0U; member < paths.size(); ++member) {
    const auto width = dimensions.at(group.members[member]).track_width;
    for (std::size_t index = 1U; index < paths[member].points.size(); ++index) {
      const auto& first = paths[member].points[index - 1U];
      const auto& second = paths[member].points[index];
      if (first.layer != second.layer || first.position == second.position) {
        continue;
      }
      segments[member].push_back(Segment{first.position, second.position, first.layer, width});
      probe.tracks.push_back(TrackSegment{
          ObjectId{"flux-coupled-fanout-probe-" + std::to_string(member) + "-" +
                   std::to_string(index)},
          group.members[member], first.layer, first.position, second.position, width, false});
    }
  }
  for (std::size_t first = 0U; first < segments.size(); ++first) {
    for (std::size_t second = first + 1U; second < segments.size(); ++second) {
      auto gap = resolve_pairwise_clearance(board, group.members[first], group.members[second]);
      if (second == first + 1U) {
        gap = std::max(gap, group.minimum_adjacent_track_gap);
      }
      const auto required =
          static_cast<long double>(
              segments[first].empty() ? 0 : segments[first].front().width.value()) /
              2.0L +
          static_cast<long double>(gap.value()) +
          static_cast<long double>(
              segments[second].empty() ? 0 : segments[second].front().width.value()) /
              2.0L;
      for (const auto& lhs : segments[first]) {
        for (const auto& rhs : segments[second]) {
          if (lhs.layer == rhs.layer &&
              geometry::segment_distance_nm(lhs.start, lhs.end, rhs.start, rhs.end) < required) {
            return false;
          }
        }
      }
    }
  }
  const auto report = check_copper_rules(probe);
  return std::all_of(report.violations.begin(), report.violations.end(),
                     [enforce_pair_skew](const auto& violation) {
                       return !enforce_pair_skew &&
                              violation.kind == CopperViolationKind::differential_pair_skew;
                     });
}

[[nodiscard]] bool
synchronize_coupled_terminal_fanouts(const Board& board, const CoupledRoutingGroup& group,
                                     std::vector<TopologyLockedLanePath*>& paths,
                                     const std::map<NetId, ResolvedNetRoutingRules>& dimensions,
                                     std::size_t max_candidates, std::size_t& attempted_candidates,
                                     std::size_t& synchronized_fanouts) {
  if (paths.size() != group.members.size() || paths.empty()) {
    return false;
  }
  if (attempted_candidates >= max_candidates) {
    return false;
  }
  ++attempted_candidates;
  std::vector<TopologyLockedLanePath> direct;
  direct.reserve(paths.size());
  for (const auto* path : paths) {
    direct.push_back(*path);
  }
  if (coupled_path_geometry_clear(board, group, direct, dimensions)) {
    return true;
  }
  const auto run_count = paths.front()->packed_runs.size();
  if (run_count == 0U || !std::all_of(paths.begin(), paths.end(), [run_count](const auto* path) {
        return path != nullptr && path->packed_runs.size() == run_count;
      })) {
    return false;
  }
  const auto boundary_needs_fanout = [&](bool at_start, std::size_t run) {
    const auto side = existing_terminal_fanout_side(paths, at_start, run);
    if (!side) {
      return true;
    }
    std::vector<TopologyLockedLanePath> local;
    local.reserve(paths.size());
    for (std::size_t member = 0U; member < paths.size(); ++member) {
      TopologyLockedLanePath candidate;
      candidate.points = side->polylines[member];
      const auto& source = *paths[member];
      const auto& packed_run = source.packed_runs[run];
      const auto packed_index = side->packed_indices[member];
      const auto inside_index = at_start ? packed_index + 1U : packed_index - 1U;
      if (inside_index >= packed_run.first_index && inside_index <= packed_run.last_index) {
        const auto& inside = source.points[inside_index];
        if (candidate.points.empty() || candidate.points.back() != inside) {
          candidate.points.push_back(inside);
        }
      }
      local.push_back(std::move(candidate));
    }
    return !coupled_path_geometry_clear(board, group, local, dimensions, false);
  };
  std::vector<bool> needs_start_fanout;
  std::vector<bool> needs_end_fanout;
  needs_start_fanout.reserve(run_count);
  needs_end_fanout.reserve(run_count);
  for (std::size_t run = 0U; run < run_count; ++run) {
    needs_start_fanout.push_back(boundary_needs_fanout(true, run));
    needs_end_fanout.push_back(boundary_needs_fanout(false, run));
  }
  for (std::size_t start_variant = 0U; start_variant < 3U; ++start_variant) {
    std::vector<CoupledTerminalFanoutSide> starts;
    std::vector<bool> synthesized_starts;
    starts.reserve(run_count);
    synthesized_starts.reserve(run_count);
    for (std::size_t run = 0U; run < run_count; ++run) {
      auto start = start_variant == 0U || !needs_start_fanout[run]
                       ? existing_terminal_fanout_side(paths, true, run)
                       : coupled_terminal_fanout_side(paths, true, start_variant == 2U,
                                                      start_variant == 1U, run);
      bool synthesized = start_variant != 0U && needs_start_fanout[run] && start.has_value();
      if (!start && start_variant != 0U && needs_start_fanout[run]) {
        start = existing_terminal_fanout_side(paths, true, run);
        synthesized = false;
      }
      if (!start) {
        starts.clear();
        break;
      }
      starts.push_back(std::move(*start));
      synthesized_starts.push_back(synthesized);
    }
    if (starts.size() != run_count) {
      continue;
    }
    for (std::size_t end_variant = 0U; end_variant < 3U; ++end_variant) {
      if (start_variant == 0U && end_variant == 0U) {
        continue;
      }
      if (attempted_candidates >= max_candidates) {
        return false;
      }
      ++attempted_candidates;
      std::vector<CoupledTerminalFanoutSide> ends;
      std::vector<bool> synthesized_ends;
      ends.reserve(run_count);
      synthesized_ends.reserve(run_count);
      for (std::size_t run = 0U; run < run_count; ++run) {
        auto end = end_variant == 0U || !needs_end_fanout[run]
                       ? existing_terminal_fanout_side(paths, false, run)
                       : coupled_terminal_fanout_side(paths, false, end_variant == 2U,
                                                      end_variant == 1U, run);
        bool synthesized = end_variant != 0U && needs_end_fanout[run] && end.has_value();
        if (!end && end_variant != 0U && needs_end_fanout[run]) {
          end = existing_terminal_fanout_side(paths, false, run);
          synthesized = false;
        }
        if (!end) {
          ends.clear();
          break;
        }
        ends.push_back(std::move(*end));
        synthesized_ends.push_back(synthesized);
      }
      if (ends.size() != run_count) {
        continue;
      }
      std::vector<TopologyLockedLanePath> working;
      working.reserve(paths.size());
      for (std::size_t member = 0U; member < paths.size(); ++member) {
        const auto& source = *paths[member];
        auto candidate = source;
        candidate.points.clear();
        candidate.packed_runs.clear();
        const auto append = [&candidate](const GlobalRoutePoint& point) {
          if (candidate.points.empty() || candidate.points.back() != point) {
            candidate.points.push_back(point);
          }
          return candidate.points.size() - 1U;
        };
        std::optional<std::size_t> previous_end_anchor;
        bool valid = true;
        for (std::size_t run = 0U; run < run_count; ++run) {
          const auto& start = starts[run];
          const auto& end = ends[run];
          if (start.packed_indices[member] > end.packed_indices[member] ||
              start.anchor_indices[member] > start.packed_indices[member] ||
              end.packed_indices[member] > end.anchor_indices[member] ||
              (previous_end_anchor && *previous_end_anchor >= start.anchor_indices[member])) {
            valid = false;
            break;
          }
          if (previous_end_anchor) {
            for (auto index = *previous_end_anchor + 1U; index <= start.anchor_indices[member];
                 ++index) {
              append(source.points[index]);
            }
          }
          for (const auto& point : start.polylines[member]) {
            append(point);
          }
          const auto run_first = candidate.points.size() - 1U;
          for (auto index = start.packed_indices[member] + 1U; index < end.packed_indices[member];
               ++index) {
            append(source.points[index]);
          }
          auto run_last = candidate.points.size() - 1U;
          for (auto point = end.polylines[member].rbegin(); point != end.polylines[member].rend();
               ++point) {
            const auto appended = append(*point);
            if (point == end.polylines[member].rbegin()) {
              run_last = appended;
            }
          }
          candidate.packed_runs.push_back(
              TopologyLockedLanePath::PackedRun{run_first, run_last, source.packed_runs[run].layer,
                                                source.packed_runs[run].resource_ids});
          previous_end_anchor = end.anchor_indices[member];
        }
        if (!valid || candidate.packed_runs.size() != run_count) {
          working.clear();
          break;
        }
        candidate.first_packed_index = candidate.packed_runs.front().first_index;
        candidate.last_packed_index = candidate.packed_runs.back().last_index;
        candidate.multilayer = std::adjacent_find(candidate.points.begin(), candidate.points.end(),
                                                  [](const auto& lhs, const auto& rhs) {
                                                    return lhs.layer != rhs.layer;
                                                  }) != candidate.points.end();
        working.push_back(std::move(candidate));
      }
      if (working.size() != paths.size()) {
        continue;
      }
      if (!coupled_path_geometry_clear(board, group, working, dimensions)) {
        continue;
      }
      for (std::size_t member = 0U; member < paths.size(); ++member) {
        *paths[member] = std::move(working[member]);
      }
      synchronized_fanouts += static_cast<std::size_t>(
          std::count(synthesized_starts.begin(), synthesized_starts.end(), true));
      synchronized_fanouts += static_cast<std::size_t>(
          std::count(synthesized_ends.begin(), synthesized_ends.end(), true));
      return true;
    }
  }
  return false;
}

struct ViaSpanProposal {
  std::vector<LayerId> layers;
  std::size_t begin_index{0};
  std::size_t end_index{0};
  Point current;
  std::optional<Point> before;
  std::optional<Point> after;
};

[[nodiscard]] std::optional<std::vector<ViaSpanProposal>>
via_span_proposals(const TopologyLockedLanePath& path) {
  std::vector<ViaSpanProposal> result;
  for (std::size_t index = 0U; index + 1U < path.points.size();) {
    if (path.points[index].layer == path.points[index + 1U].layer) {
      ++index;
      continue;
    }
    const auto begin = index;
    const auto position = path.points[index].position;
    ViaSpanProposal proposal;
    proposal.begin_index = begin;
    proposal.current = position;
    proposal.layers.push_back(path.points[index].layer);
    while (index + 1U < path.points.size() &&
           path.points[index].layer != path.points[index + 1U].layer) {
      if (path.points[index + 1U].position != position) {
        return std::nullopt;
      }
      ++index;
      proposal.layers.push_back(path.points[index].layer);
    }
    proposal.end_index = index;
    for (auto cursor = begin; cursor > 0U; --cursor) {
      if (path.points[cursor - 1U].position != position) {
        proposal.before = path.points[cursor - 1U].position;
        break;
      }
    }
    for (auto cursor = index + 1U; cursor < path.points.size(); ++cursor) {
      if (path.points[cursor].position != position) {
        proposal.after = path.points[cursor].position;
        break;
      }
    }
    result.push_back(std::move(proposal));
  }
  return result;
}

[[nodiscard]] bool synchronize_shared_resource_fanouts(
    const Board& board, const CoupledRoutingGroup& group,
    std::vector<TopologyLockedLanePath*>& paths, const std::vector<std::size_t>& run_indices,
    const std::map<NetId, ResolvedNetRoutingRules>& dimensions, std::size_t max_candidates,
    std::size_t& attempted_candidates, std::size_t& synchronized_fanouts) {
  if (paths.size() != group.members.size() || paths.size() < 2U ||
      run_indices.size() != paths.size()) {
    return false;
  }
  std::vector<TopologyLockedLanePath> direct;
  direct.reserve(paths.size());
  for (const auto* path : paths) {
    if (path == nullptr) {
      return false;
    }
    direct.push_back(*path);
  }
  if (coupled_path_geometry_clear(board, group, direct, dimensions, false)) {
    return true;
  }

  for (std::size_t start_variant = 0U; start_variant < 3U; ++start_variant) {
    auto start = start_variant == 0U
                     ? existing_terminal_fanout_side_for_runs(paths, true, run_indices)
                     : coupled_terminal_fanout_side_for_runs(paths, true, start_variant == 2U,
                                                             start_variant == 1U, run_indices);
    if (!start) {
      continue;
    }
    for (std::size_t end_variant = 0U; end_variant < 3U; ++end_variant) {
      if (start_variant == 0U && end_variant == 0U) {
        continue;
      }
      if (attempted_candidates >= max_candidates) {
        return false;
      }
      ++attempted_candidates;
      auto end = end_variant == 0U
                     ? existing_terminal_fanout_side_for_runs(paths, false, run_indices)
                     : coupled_terminal_fanout_side_for_runs(paths, false, end_variant == 2U,
                                                             end_variant == 1U, run_indices);
      if (!end) {
        continue;
      }

      std::vector<TopologyLockedLanePath> working;
      working.reserve(paths.size());
      for (std::size_t member = 0U; member < paths.size(); ++member) {
        const auto& source = *paths[member];
        const auto selected_run = run_indices[member];
        if (selected_run >= source.packed_runs.size() ||
            start->anchor_indices[member] > start->packed_indices[member] ||
            start->packed_indices[member] > end->packed_indices[member] ||
            end->packed_indices[member] > end->anchor_indices[member]) {
          working.clear();
          break;
        }

        auto candidate = source;
        candidate.points.clear();
        candidate.packed_runs.clear();
        std::vector<std::optional<std::size_t>> mapped_indices(source.points.size());
        const auto append = [&candidate](const GlobalRoutePoint& point) {
          if (candidate.points.empty() || candidate.points.back() != point) {
            candidate.points.push_back(point);
          }
          return candidate.points.size() - 1U;
        };
        const auto append_source = [&](std::size_t index) {
          const auto mapped = append(source.points[index]);
          mapped_indices[index] = mapped;
          return mapped;
        };

        for (std::size_t index = 0U; index < start->anchor_indices[member]; ++index) {
          append_source(index);
        }
        std::optional<std::size_t> start_anchor;
        std::optional<std::size_t> run_first;
        for (const auto& point : start->polylines[member]) {
          const auto appended = append(point);
          if (!start_anchor) {
            start_anchor = appended;
          }
          run_first = appended;
        }
        if (!start_anchor || !run_first) {
          working.clear();
          break;
        }
        mapped_indices[start->anchor_indices[member]] = *start_anchor;
        mapped_indices[start->packed_indices[member]] = *run_first;
        for (auto index = start->packed_indices[member] + 1U; index < end->packed_indices[member];
             ++index) {
          append_source(index);
        }
        std::optional<std::size_t> run_last;
        for (auto point = end->polylines[member].rbegin(); point != end->polylines[member].rend();
             ++point) {
          const auto appended = append(*point);
          if (!run_last) {
            run_last = appended;
          }
        }
        if (!run_last) {
          working.clear();
          break;
        }
        mapped_indices[end->packed_indices[member]] = *run_last;
        mapped_indices[end->anchor_indices[member]] = candidate.points.size() - 1U;
        for (auto index = end->anchor_indices[member] + 1U; index < source.points.size(); ++index) {
          append_source(index);
        }

        bool valid = true;
        for (std::size_t run = 0U; run < source.packed_runs.size(); ++run) {
          if (run == selected_run) {
            candidate.packed_runs.push_back(TopologyLockedLanePath::PackedRun{
                *run_first, *run_last, source.packed_runs[run].layer,
                source.packed_runs[run].resource_ids});
            continue;
          }
          const auto& source_run = source.packed_runs[run];
          if (source_run.first_index >= mapped_indices.size() ||
              source_run.last_index >= mapped_indices.size() ||
              !mapped_indices[source_run.first_index] || !mapped_indices[source_run.last_index]) {
            valid = false;
            break;
          }
          candidate.packed_runs.push_back(TopologyLockedLanePath::PackedRun{
              *mapped_indices[source_run.first_index], *mapped_indices[source_run.last_index],
              source_run.layer, source_run.resource_ids});
        }
        if (!valid || candidate.packed_runs.size() != source.packed_runs.size()) {
          working.clear();
          break;
        }
        candidate.first_packed_index = candidate.packed_runs.front().first_index;
        candidate.last_packed_index = candidate.packed_runs.back().last_index;
        candidate.multilayer = std::adjacent_find(candidate.points.begin(), candidate.points.end(),
                                                  [](const auto& lhs, const auto& rhs) {
                                                    return lhs.layer != rhs.layer;
                                                  }) != candidate.points.end();
        working.push_back(std::move(candidate));
      }
      if (working.size() != paths.size() ||
          !coupled_path_geometry_clear(board, group, working, dimensions, false)) {
        continue;
      }
      for (std::size_t member = 0U; member < paths.size(); ++member) {
        *paths[member] = std::move(working[member]);
      }
      synchronized_fanouts += (start_variant == 0U ? 0U : 1U) + (end_variant == 0U ? 0U : 1U);
      return true;
    }
  }
  return false;
}

[[nodiscard]] bool
via_array_has_coupled_geometry(const Board& board, const CoupledRoutingGroup& group,
                               const std::vector<Point>& positions,
                               const std::map<NetId, ResolvedNetRoutingRules>& dimensions) {
  if (positions.size() != group.members.size() || positions.size() < 2U) {
    return false;
  }
  for (std::size_t first = 0U; first < positions.size(); ++first) {
    if (!geometry::coordinate_in_range(positions[first])) {
      return false;
    }
    for (std::size_t second = first + 1U; second < positions.size(); ++second) {
      const auto distance = geometry::point_distance_nm(positions[first], positions[second]);
      const auto first_radius =
          static_cast<long double>(dimensions.at(group.members[first]).via_diameter.value()) / 2.0L;
      const auto second_radius =
          static_cast<long double>(dimensions.at(group.members[second]).via_diameter.value()) /
          2.0L;
      auto required =
          resolve_pairwise_clearance(board, group.members[first], group.members[second]);
      if (second == first + 1U) {
        required = std::max(required, group.minimum_adjacent_via_gap);
      }
      if (distance + 0.5L <
          first_radius + second_radius + static_cast<long double>(required.value())) {
        return false;
      }
    }
  }
  if (!group.ordered || positions.size() < 3U) {
    return true;
  }
  const auto axis_x =
      static_cast<long double>(positions.back().x.value() - positions.front().x.value());
  const auto axis_y =
      static_cast<long double>(positions.back().y.value() - positions.front().y.value());
  const auto axis_length = std::hypotl(axis_x, axis_y);
  if (!(axis_length > 0.0L)) {
    return false;
  }
  auto prior_projection = -std::numeric_limits<long double>::infinity();
  for (const auto position : positions) {
    const auto relative_x =
        static_cast<long double>(position.x.value() - positions.front().x.value());
    const auto relative_y =
        static_cast<long double>(position.y.value() - positions.front().y.value());
    const auto projection = (relative_x * axis_x + relative_y * axis_y) / axis_length;
    const auto perpendicular = std::abs(relative_x * axis_y - relative_y * axis_x) / axis_length;
    if (projection <= prior_projection || perpendicular > 2.0L) {
      return false;
    }
    prior_projection = projection;
  }
  return true;
}

[[nodiscard]] bool via_array_has_coupled_escape_geometry(
    const Board& board, const CoupledRoutingGroup& group, const std::vector<Point>& positions,
    const std::vector<TopologyLockedLanePath>& paths,
    const std::vector<std::vector<ViaSpanProposal>>& proposals, std::size_t transition,
    const std::map<NetId, ResolvedNetRoutingRules>& dimensions) {
  if (positions.size() != paths.size() || positions.size() != group.members.size()) {
    return false;
  }
  for (std::size_t via_member = 0U; via_member < positions.size(); ++via_member) {
    const auto via_radius =
        static_cast<long double>(dimensions.at(group.members[via_member]).via_diameter.value()) /
        2.0L;
    const std::set<LayerId> via_layers(proposals[via_member][transition].layers.begin(),
                                       proposals[via_member][transition].layers.end());
    for (std::size_t track_member = 0U; track_member < paths.size(); ++track_member) {
      if (track_member == via_member) {
        continue;
      }
      const auto required =
          via_radius +
          static_cast<long double>(dimensions.at(group.members[track_member]).track_width.value()) /
              2.0L +
          static_cast<long double>(resolve_pairwise_clearance(board, group.members[via_member],
                                                              group.members[track_member])
                                       .value());
      for (std::size_t index = 1U; index < paths[track_member].points.size(); ++index) {
        const auto& first = paths[track_member].points[index - 1U];
        const auto& second = paths[track_member].points[index];
        if (first.layer != second.layer || first.position == second.position ||
            !via_layers.contains(first.layer)) {
          continue;
        }
        if (geometry::point_segment_distance_nm(positions[via_member], first.position,
                                                second.position) < required) {
          return false;
        }
      }
    }
  }
  return true;
}

[[nodiscard]] std::optional<std::vector<Point>>
staggered_via_array(const CoupledRoutingGroup& group,
                    const std::vector<std::vector<ViaSpanProposal>>& proposals,
                    std::size_t transition,
                    const std::map<NetId, ResolvedNetRoutingRules>& dimensions, bool reverse) {
  if (proposals.size() < 2U) {
    return std::nullopt;
  }
  const auto first = proposals.front()[transition].current;
  const auto last = proposals.back()[transition].current;
  const auto axis_x = static_cast<long double>(last.x.value() - first.x.value());
  const auto axis_y = static_cast<long double>(last.y.value() - first.y.value());
  const auto length = std::hypotl(axis_x, axis_y);
  if (!(length > 0.0L)) {
    return std::nullopt;
  }
  auto pitch = group.minimum_adjacent_via_gap.value();
  for (const auto& member : group.members) {
    pitch = std::max(pitch, dimensions.at(member).via_diameter.value() +
                                group.minimum_adjacent_via_gap.value());
  }
  if (pitch > std::numeric_limits<std::int64_t>::max() / 3) {
    return std::nullopt;
  }
  pitch *= 3;
  std::vector<Point> result;
  result.reserve(proposals.size());
  for (std::size_t member = 0U; member < proposals.size(); ++member) {
    const auto offset = static_cast<long double>(pitch) * (reverse ? -1.0L : 1.0L);
    const auto x = static_cast<long double>(proposals[member][transition].current.x.value()) -
                   axis_y / length * offset;
    const auto y = static_cast<long double>(proposals[member][transition].current.y.value()) +
                   axis_x / length * offset;
    constexpr auto low = static_cast<long double>(std::numeric_limits<std::int64_t>::min());
    constexpr auto high = static_cast<long double>(std::numeric_limits<std::int64_t>::max());
    if (!std::isfinite(x) || !std::isfinite(y) || x < low || x > high || y < low || y > high) {
      return std::nullopt;
    }
    result.push_back(Point{Nanometres{static_cast<std::int64_t>(std::llround(x))},
                           Nanometres{static_cast<std::int64_t>(std::llround(y))}});
  }
  return result;
}

[[nodiscard]] bool
synchronize_coupled_via_arrays(const Board& board, const CoupledRoutingGroup& group,
                               std::vector<TopologyLockedLanePath*>& paths,
                               const std::map<NetId, ResolvedNetRoutingRules>& dimensions,
                               std::size_t& array_count) {
  if (paths.size() != group.members.size() || paths.empty()) {
    return false;
  }
  std::vector<TopologyLockedLanePath> working;
  working.reserve(paths.size());
  for (const auto* path : paths) {
    working.push_back(*path);
  }
  std::vector<std::vector<ViaSpanProposal>> proposals;
  proposals.reserve(working.size());
  for (const auto& path : working) {
    const auto extracted = via_span_proposals(path);
    if (!extracted || extracted->empty()) {
      return false;
    }
    proposals.push_back(*extracted);
  }
  if (!std::all_of(proposals.begin(), proposals.end(), [&proposals](const auto& candidate) {
        return candidate.size() == proposals.front().size();
      })) {
    return false;
  }
  for (std::size_t transition = 0U; transition < proposals.front().size(); ++transition) {
    const auto& expected_layers = proposals.front()[transition].layers;
    if (!std::all_of(proposals.begin(), proposals.end(), [&](const auto& candidate) {
          return candidate[transition].layers == expected_layers;
        })) {
      return false;
    }
    std::vector<Point> selected;
    selected.reserve(paths.size());
    bool found = false;
    for (std::size_t choice = 0U; choice < 5U && !found; ++choice) {
      selected.clear();
      bool available = true;
      if (choice >= 3U) {
        const auto staggered =
            staggered_via_array(group, proposals, transition, dimensions, choice == 4U);
        if (staggered) {
          selected = *staggered;
        } else {
          available = false;
        }
      } else {
        for (const auto& candidate : proposals) {
          const auto& proposal = candidate[transition];
          const std::optional<Point> position = choice == 0U
                                                    ? std::optional<Point>{proposal.current}
                                                : choice == 1U ? proposal.before
                                                               : proposal.after;
          if (!position) {
            available = false;
            break;
          }
          selected.push_back(*position);
        }
      }
      if (available && via_array_has_coupled_geometry(board, group, selected, dimensions)) {
        auto escape_paths = working;
        for (std::size_t member = 0U; member < escape_paths.size(); ++member) {
          const auto& proposal = proposals[member][transition];
          for (auto index = proposal.begin_index; index <= proposal.end_index; ++index) {
            escape_paths[member].points[index].position = selected[member];
          }
        }
        found = via_array_has_coupled_escape_geometry(board, group, selected, escape_paths,
                                                      proposals, transition, dimensions);
      }
    }
    if (!found) {
      return false;
    }
    for (std::size_t member = 0U; member < working.size(); ++member) {
      const auto& proposal = proposals[member][transition];
      for (auto index = proposal.begin_index; index <= proposal.end_index; ++index) {
        working[member].points[index].position = selected[member];
      }
    }
  }
  for (std::size_t member = 0U; member < paths.size(); ++member) {
    *paths[member] = std::move(working[member]);
  }
  array_count += proposals.front().size();
  return true;
}

[[nodiscard]] bool path_crosses_layers(const GlobalRoutePath& path) {
  return std::adjacent_find(path.points.begin(), path.points.end(),
                            [](const auto& lhs, const auto& rhs) {
                              return lhs.layer != rhs.layer;
                            }) != path.points.end();
}

struct SplineRun {
  LayerId layer;
  Nanometres width;
  std::vector<Point> points;
  std::vector<std::string> original_track_ids;
  bool topology_locked{false};
};

struct PendingSplineRun {
  NetId net;
  std::string base;
  SplineRun run;
};

struct SplineNavigationPreparation {
  bool complete{false};
  bool regions_are_offset{false};
  std::vector<OffsetFreeSpaceRegion> regions;
  std::string diagnostic;
};

struct SplineNavigationPreparationEntry {
  std::size_t remaining_consumers{0U};
  std::size_t consumers{0U};
  std::mutex mutex;
  std::shared_ptr<const SplineNavigationPreparation> preparation;
};

struct SplineNavigationPreparationCache {
  BoardFreeSpaceExtractionCache obstacle_geometry{16U};
  FreeSpaceOffsetCache obstacle_offsets{4096U};
  using Key = std::tuple<NetId, LayerId, Nanometres>;
  std::mutex mutex;
  std::map<Key, std::shared_ptr<SplineNavigationPreparationEntry>> entries;
};

[[nodiscard]] std::size_t split_tracks_at_exact_same_net_branches(Board& board,
                                                                  std::set<std::string>& used_ids,
                                                                  const std::set<std::string>& protected_ids,
                                                                  std::stop_token stop_token = {},
                                                                  bool split_crossings = false,
                                                                  std::size_t max_output_tracks = std::numeric_limits<std::size_t>::max(),
                                                                  bool* exhausted = nullptr) {
  if (exhausted != nullptr) *exhausted = false;
  const auto source_tracks = board.tracks;
  if (source_tracks.size() > max_output_tracks) {
    if (exhausted != nullptr) *exhausted = true;
    return 0U;
  }
  std::vector<TrackSegment> split_tracks;
  split_tracks.reserve(source_tracks.size());
  std::size_t inserted_pieces = 0U;
  for (const auto& carrier : source_tracks) {
    if (split_tracks.size() >= max_output_tracks) {
      if (exhausted != nullptr) *exhausted = true;
      return 0U;
    }
    routing_kernel_checkpoint();
    if (stop_token.stop_requested()) return 0U;
    if (carrier.locked || protected_ids.contains(carrier.id.value()) || carrier.start == carrier.end) {
      split_tracks.push_back(carrier);
      continue;
    }
    const auto dx = static_cast<long double>(carrier.end.x.value() - carrier.start.x.value());
    const auto dy = static_cast<long double>(carrier.end.y.value() - carrier.start.y.value());
    const auto length_squared = dx * dx + dy * dy;
    std::vector<std::pair<long double, Point>> contacts;
    const auto append_exact_contact = [&](Point contact) {
      const auto parameter =
          (static_cast<long double>(contact.x.value() - carrier.start.x.value()) * dx +
           static_cast<long double>(contact.y.value() - carrier.start.y.value()) * dy) /
          length_squared;
      if (!(parameter > 0.0L && parameter < 1.0L))
        return;
      const auto projected = Point{
          Nanometres{static_cast<std::int64_t>(
              std::llround(static_cast<long double>(carrier.start.x.value()) + parameter * dx))},
          Nanometres{static_cast<std::int64_t>(
              std::llround(static_cast<long double>(carrier.start.y.value()) + parameter * dy))}};
      if (geometry::point_distance_nm(contact, projected) <= 1.0L &&
          projected != carrier.start && projected != carrier.end) {
        contacts.emplace_back(parameter, projected);
      }
    };
    std::size_t inspected_branches = 0U;
    for (const auto& branch : source_tracks) {
      if ((inspected_branches++ % 64U) == 0U) {
        routing_kernel_checkpoint();
        if (stop_token.stop_requested()) return 0U;
      }
      if (branch.id == carrier.id || branch.net != carrier.net || branch.layer != carrier.layer ||
          branch.start == branch.end) {
        continue;
      }
      for (const auto endpoint : {branch.start, branch.end}) {
        // This structural split is for an imported unsplit centreline T, not for general
        // same-net copper proximity. Wider overlap remains governed by connectivity and exact
        // transaction checks; promoting it to a junction here would fragment nearby routes.
        append_exact_contact(endpoint);
      }
      // Imported and generated centreline trees occasionally meet in the interior of two
      // straight tracks.  Split only an exactly representable intersection: rounded or
      // finite-width-only contacts deliberately remain untouched and are guarded by the exact
      // copper transaction below.
      if (!split_crossings ||
          !geometry::segments_intersect(carrier.start, carrier.end, branch.start, branch.end))
        continue;
      const auto bx = static_cast<long double>(branch.end.x.value() - branch.start.x.value());
      const auto by = static_cast<long double>(branch.end.y.value() - branch.start.y.value());
      const auto denominator = dx * by - dy * bx;
      if (std::abs(denominator) <= 0.0L)
        continue;
      const auto offset_x = static_cast<long double>(branch.start.x.value() - carrier.start.x.value());
      const auto offset_y = static_cast<long double>(branch.start.y.value() - carrier.start.y.value());
      const auto parameter = (offset_x * by - offset_y * bx) / denominator;
      const auto intersection = Point{
          Nanometres{static_cast<std::int64_t>(std::llround(carrier.start.x.value() + parameter * dx))},
          Nanometres{static_cast<std::int64_t>(std::llround(carrier.start.y.value() + parameter * dy))}};
      if (geometry::point_on_segment(intersection, carrier.start, carrier.end) &&
          geometry::point_on_segment(intersection, branch.start, branch.end)) {
        append_exact_contact(intersection);
      }
    }
    // A pad/via centre lying in the interior of a route is also a topological attachment, even
    // when no second track endpoint lands at exactly that coordinate.  In particular, a
    // multi-terminal tree may deliberately pass through a through-hole pad and connect another
    // branch anywhere inside the pad's copper.  Preserve the exact centre as a chain boundary so
    // PH rounding cannot bow the carrier away and silently open the net.
    for (const auto& via : board.vias) {
      if (via.net == carrier.net &&
          std::find(via.layers.begin(), via.layers.end(), carrier.layer) != via.layers.end()) {
        append_exact_contact(via.position);
      }
    }
    for (const auto& footprint : board.footprints) {
      for (const auto& pad : footprint.pads) {
        if (pad.net != carrier.net ||
            std::find(pad.layers.begin(), pad.layers.end(), carrier.layer) == pad.layers.end()) {
          continue;
        }
        append_exact_contact(geometry::transform_point(
            pad.local_position, footprint.position, footprint.rotation, footprint.side));
      }
    }
    std::sort(contacts.begin(), contacts.end(), [](const auto& lhs, const auto& rhs) {
      return lhs.first != rhs.first ? lhs.first < rhs.first : lhs.second < rhs.second;
    });
    contacts.erase(
        std::unique(contacts.begin(), contacts.end(),
                    [](const auto& lhs, const auto& rhs) { return lhs.second == rhs.second; }),
        contacts.end());
    if (contacts.empty()) {
      split_tracks.push_back(carrier);
      continue;
    }
    std::vector<Point> points{carrier.start};
    for (const auto& [parameter, point] : contacts) {
      (void)parameter;
      if (points.back() != point)
        points.push_back(point);
    }
    points.push_back(carrier.end);
    if (points.size() - 1U > max_output_tracks - split_tracks.size()) {
      if (exhausted != nullptr) *exhausted = true;
      return 0U;
    }
    for (std::size_t piece = 1U; piece < points.size(); ++piece) {
      auto split = carrier;
      split.id = unique_id(carrier.id.value() + "-branch-" + std::to_string(piece - 1U), used_ids);
      split.start = points[piece - 1U];
      split.end = points[piece];
      split_tracks.push_back(std::move(split));
    }
    inserted_pieces += points.size() - 2U;
  }
  board.tracks = std::move(split_tracks);
  return inserted_pieces;
}

// Keep curve fitting from spending its bounded budget on generated, electrically useless straight
// stubs.  This intentionally reasons only about exact centreline vertices after the branch split:
// finite-width near contacts are ambiguous, so they fail closed.  Native/source/coupled routing is
// excluded by net, and each whole leaf chain is accepted only by the ordinary exact board gates.
void cleanup_generated_straight_dead_tails(
    Board& board, const std::set<std::string>& protected_ids,
    const std::set<NetId>& protected_routing_nets, const std::set<NetId>& coupled_nets,
    const DetailedRouterConfig& config, DetailedRouteResult& result, std::stop_token stop_token) {
  if (!config.enable_generated_tail_cleanup || config.max_generated_tail_cleanup_candidates == 0U)
    return;
  std::set<NetId> excluded_nets = protected_routing_nets;
  excluded_nets.insert(coupled_nets.begin(), coupled_nets.end());
  // Filled zones and native curved copper have no safe straight-centreline contact model here.
  for (const auto& zone : board.zones) excluded_nets.insert(zone.net);
  for (const auto& arc : board.track_arcs) excluded_nets.insert(arc.net);
  for (const auto& curve : board.track_beziers) excluded_nets.insert(curve.net);
  for (const auto& track : board.tracks)
    if (track.locked) excluded_nets.insert(track.net);
  for (const auto& track : board.tracks) {
    if (track.locked || protected_ids.contains(track.id.value()) || excluded_nets.contains(track.net))
      ++result.generated_tail_cleanup_skipped_protected;
  }
  const auto pads = copper_pad_shapes(board);
  const auto has_pad = [&](const TrackSegment& track, Point point) {
    return std::any_of(pads.begin(), pads.end(), [&](const auto& pad) {
      return pad.net == track.net &&
             std::find(pad.layers.begin(), pad.layers.end(), track.layer) != pad.layers.end() &&
             point_to_pad_copper_distance_nm(point, pad) <= 0.0L;
    });
  };
  const auto has_via = [&](const TrackSegment& track, Point point) {
    return std::any_of(board.vias.begin(), board.vias.end(), [&](const auto& via) {
      return via.net == track.net &&
             std::find(via.layers.begin(), via.layers.end(), track.layer) != via.layers.end() &&
             geometry::point_distance_nm(point, via.position) <=
                 static_cast<long double>(via.diameter.value()) / 2.0L;
    });
  };
  const auto before_connectivity = analyze_connectivity(board);
  struct TerminalMembership {
    std::vector<CopperConnectivityProbe> probes;
    std::vector<CopperProbeComponentMembership> memberships;
  };
  std::map<NetId, TerminalMembership> terminal_memberships;
  std::set<LayerId> copper_layers;
  for (const auto& layer : board.layers)
    if (layer.kind == LayerKind::copper) copper_layers.insert(layer.id);
  const auto shares_component = [](const CopperProbeComponentMembership& lhs,
                                   const CopperProbeComponentMembership& rhs) {
    return std::any_of(lhs.components.begin(), lhs.components.end(), [&](const auto component) {
      return std::find(rhs.components.begin(), rhs.components.end(), component) != rhs.components.end();
    });
  };
  const auto terminals_preserved = [&](const Board& after, const NetId& net) {
    auto found = terminal_memberships.find(net);
    if (found == terminal_memberships.end()) {
      TerminalMembership membership;
      for (const auto& pad : pads) {
        if (pad.net != net) continue;
        for (const auto& layer : pad.layers) {
          if (copper_layers.contains(layer)) membership.probes.push_back({pad.position, {layer}});
        }
      }
      for (const auto& via : board.vias) {
        if (via.net != net) continue;
        for (const auto& layer : via.layers) {
          if (copper_layers.contains(layer)) membership.probes.push_back({via.position, {layer}});
        }
      }
      // A bounded cleanup must not turn an unusually large terminal set into an unbounded
      // quadratic equivalence check.  Such a net simply remains untouched in this pass.
      if (membership.probes.size() > 4096U) return false;
      membership.memberships = copper_probe_component_memberships(board, net, membership.probes);
      found = terminal_memberships.emplace(net, std::move(membership)).first;
    }
    const auto after_memberships =
        copper_probe_component_memberships(after, net, found->second.probes);
    if (after_memberships.size() != found->second.memberships.size()) return false;
    for (std::size_t first = 0U; first < found->second.memberships.size(); ++first) {
      if (found->second.memberships[first].contained_by_copper &&
          !after_memberships[first].contained_by_copper) return false;
      for (std::size_t second = first + 1U; second < found->second.memberships.size(); ++second) {
        if (shares_component(found->second.memberships[first], found->second.memberships[second]) &&
            !shares_component(after_memberships[first], after_memberships[second])) return false;
      }
    }
    return true;
  };
  for (;;) {
    routing_kernel_checkpoint();
    if (stop_token.stop_requested()) return;
    struct EndpointEdge { std::size_t track; Point other; };
    using CopperNode = std::tuple<NetId, LayerId, Point>;
    std::map<CopperNode, std::vector<EndpointEdge>> incident;
    std::vector<bool> eligible(board.tracks.size(), false);
    for (std::size_t index = 0U; index < board.tracks.size(); ++index) {
      const auto& track = board.tracks[index];
      if (track.start == track.end) continue;
      // Every straight track participates in incidence, including protected copper.  It then
      // becomes a boundary rather than disappearing and turning its adjacent generated segment
      // into a false leaf.
      incident[{track.net, track.layer, track.start}].push_back({index, track.end});
      incident[{track.net, track.layer, track.end}].push_back({index, track.start});
      if (track.locked || protected_ids.contains(track.id.value()) || excluded_nets.contains(track.net) ||
          std::any_of(board.vias.begin(), board.vias.end(), [&](const auto& via) {
            return via.net == track.net &&
                   std::find(via.layers.begin(), via.layers.end(), track.layer) != via.layers.end() &&
                   geometry::point_segment_distance_nm(via.position, track.start, track.end) <=
                       static_cast<long double>(via.diameter.value() + track.width.value()) / 2.0L;
          })) continue;
      eligible[index] = true;
    }
    std::optional<std::vector<std::size_t>> removal;
    for (std::size_t seed = 0U; seed < board.tracks.size() && !removal; ++seed) {
      if (!eligible[seed]) continue;
      for (const auto leaf : {board.tracks[seed].start, board.tracks[seed].end}) {
        const auto found = incident.find({board.tracks[seed].net, board.tracks[seed].layer, leaf});
        if (found == incident.end() || found->second.size() != 1U || has_pad(board.tracks[seed], leaf) ||
            has_via(board.tracks[seed], leaf)) continue;
        if (result.generated_tail_cleanup_candidates >= config.max_generated_tail_cleanup_candidates) {
          result.generated_tail_cleanup_budget_exhausted = true;
          return;
        }
        ++result.generated_tail_cleanup_candidates;
        // A padless floating component can be deliberate construction copper.  Find its complete
        // exact-centreline component first and require a physical pad attachment somewhere in it.
        std::set<Point> visited_points{leaf};
        std::vector<Point> pending{leaf};
        bool component_has_pad = false;
        while (!pending.empty() && visited_points.size() <= 4096U) {
          routing_kernel_checkpoint();
          if (stop_token.stop_requested()) return;
          const auto point = pending.back(); pending.pop_back();
          for (const auto& edge : incident.at({board.tracks[seed].net, board.tracks[seed].layer, point})) {
            const auto& track = board.tracks[edge.track];
            component_has_pad = component_has_pad || has_pad(track, point) || has_pad(track, edge.other);
            if (visited_points.insert(edge.other).second) pending.push_back(edge.other);
          }
        }
        if (visited_points.size() > 4096U || !component_has_pad) continue;
        std::vector<std::size_t> chain;
        Point current = leaf;
        std::optional<std::size_t> prior;
        for (;;) {
          routing_kernel_checkpoint();
          if (stop_token.stop_requested()) return;
          if (chain.size() >= 4096U) { chain.clear(); break; }
          const auto& edges = incident.at({board.tracks[seed].net, board.tracks[seed].layer, current});
          const auto next = std::find_if(edges.begin(), edges.end(), [&](const auto& edge) {
            return (!prior || edge.track != *prior) && eligible[edge.track];
          });
          if (next == edges.end()) break;
          chain.push_back(next->track);
          prior = next->track;
          current = next->other;
          const auto& current_edges = incident.at({board.tracks[seed].net, board.tracks[seed].layer, current});
          if (has_pad(board.tracks[*prior], current) || has_via(board.tracks[*prior], current) ||
              current_edges.size() != 2U) break;
        }
        if (!chain.empty()) removal = std::move(chain);
        break;
      }
    }
    if (!removal) return;
    std::set<ObjectId> removed;
    for (const auto index : *removal) removed.insert(board.tracks[index].id);
    Board transaction = board;
    std::erase_if(transaction.tracks, [&removed](const auto& track) { return removed.contains(track.id); });
    // Keep all decisions at chain granularity: no partial leaf surgery survives a failed exact
    // check.
    const auto validation = validate_board(transaction);
    const auto copper = check_copper_rules(transaction);
    const auto after_connectivity = analyze_connectivity(transaction);
    if (!validation.ok() || !copper.ok() ||
        !preserves_existing_pad_components(before_connectivity, after_connectivity) ||
        !terminals_preserved(transaction, board.tracks[removal->front()].net) ||
        after_connectivity.open_connections > before_connectivity.open_connections) {
      // It is unsafe to try a shorter suffix after a rejected chain: an unsplit finite-width
      // contact may be the real attachment.  Mark all of its edges ineligible by protecting them.
      for (const auto index : *removal) excluded_nets.insert(board.tracks[index].net);
      continue;
    }
    result.generated_tail_cleanup_removed_tracks += removal->size();
    ++result.generated_tail_cleanup_removed_chains;
    result.added_tracks -= std::min(result.added_tracks, removal->size());
    board = std::move(transaction);
  }
}

// A board-complete source polyline has already passed exact copper DRC. If the coarse cell graph
// cannot reproduce that known route, retain its homotopy as a narrow union of independently
// certified rectangles. This is a bounded geometric certificate, not permission to bypass the
// offset domain: every side and diagonal of every rectangle is checked against that domain.
[[nodiscard]] std::optional<NavMeshRouteResult>
make_source_polyline_corridor(const OffsetFreeSpaceRegion& region, const SplineRun& run,
                              Nanometres cell_pitch, Nanometres maximum_half_width,
                              Nanometres preferred_half_width = Nanometres{0}) {
  if (run.points.size() < 2U || cell_pitch.value() <= 0)
    return std::nullopt;
  FreeSpaceDomain domain;
  domain.outline = region.area;
  domain.cutouts = region.holes;
  // The offset region is immutable for this complete source run. Validate it once before the
  // potentially thousands of rectangle-width probes below; each probe still uses the same exact
  // chord-in-free-space and clearance predicates.
  if (!validate_free_space_chord_domain(domain))
    return std::nullopt;
  const auto sample_spacing = std::max<std::int64_t>(1, cell_pitch.value() / 4);
  const auto initial_half_width = std::max<std::int64_t>(
      1, std::min(maximum_half_width.value(),
                  std::max({std::int64_t{1}, cell_pitch.value() / 8,
                            preferred_half_width.value()})));

  NavMeshRouteResult result;
  result.start = run.points.front();
  result.goal = run.points.back();
  result.centerline.push_back(run.points.front());
  result.cell_pitch = cell_pitch;
  auto append_certified_cell = [&](Point first, Point second) {
    const std::array<std::int64_t, 4U> source_bounds{
        std::min(first.x.value(), second.x.value()),
        std::max(first.x.value(), second.x.value()),
        std::min(first.y.value(), second.y.value()),
        std::max(first.y.value(), second.y.value())};
    const auto corners_for = [](const auto& bounds) {
      return std::array<Point, 4U>{
          Point{Nanometres{bounds[0]}, Nanometres{bounds[2]}},
          Point{Nanometres{bounds[1]}, Nanometres{bounds[2]}},
          Point{Nanometres{bounds[1]}, Nanometres{bounds[3]}},
          Point{Nanometres{bounds[0]}, Nanometres{bounds[3]}}};
    };
    const auto certified = [&](const auto& bounds) {
      const auto [minimum_x, maximum_x, minimum_y, maximum_y] = bounds;
      const bool encloses_hole_vertex = std::any_of(
          region.holes.begin(), region.holes.end(), [&](const auto& hole) {
            return std::any_of(hole.vertices.begin(), hole.vertices.end(), [&](Point point) {
              return point.x.value() >= minimum_x && point.x.value() <= maximum_x &&
                     point.y.value() >= minimum_y && point.y.value() <= maximum_y;
            });
          });
      if (encloses_hole_vertex)
        return false;
      const auto corners = corners_for(bounds);
      const std::vector<std::pair<Point, Point>> certificates{
          {corners[0], corners[1]}, {corners[1], corners[2]},
          {corners[2], corners[3]}, {corners[3], corners[0]},
          {corners[0], corners[2]}, {corners[1], corners[3]}};
      const auto assessed = assess_validated_free_space_chords(domain, certificates);
      return std::all_of(assessed.begin(), assessed.end(),
                         [](const auto& clearance) { return clearance.has_value(); });
    };
    auto half_width = initial_half_width;
    while (half_width >= 1) {
      std::array<std::int64_t, 4U> bounds{
          source_bounds[0] - half_width, source_bounds[1] + half_width,
          source_bounds[2] - half_width, source_bounds[3] + half_width};
      // Boundary chords alone cannot detect a hole wholly enclosed by this rectangle.
      // Reject such cells as well; together with the exact boundary tests below this
      // certifies the whole cell, including when curvature requests a wider guard.
      if (certified(bounds)) {
        // A wall on one side must not erase free room on the other three sides.
        // Grow a known-safe cell monotonically; every accepted rectangle receives
        // the same whole-cell certificate, with a bounded number of probes per side.
        for (std::size_t side = 0U; side < bounds.size() && half_width < initial_half_width;
             ++side) {
          const auto direction = side % 2U == 0U ? -1 : 1;
          auto extra = initial_half_width - half_width;
          for (std::size_t probe = 0U; probe < 8U && extra > 0; ++probe, extra /= 2) {
            auto expanded = bounds;
            expanded[side] += direction * extra;
            if (certified(expanded)) {
              bounds = expanded;
              break;
            }
          }
        }
        const auto corners = corners_for(bounds);
        const auto id = result.cells.size();
        result.cells.push_back(
            NavMeshCell{id, corners,
                        Point{Nanometres{bounds[0] + (bounds[1] - bounds[0]) / 2},
                              Nanometres{bounds[2] + (bounds[3] - bounds[2]) / 2}},
                        0U});
        result.topology_guard_cells.push_back(id);
        return true;
      }
      if (half_width == 1)
        break;
      half_width = std::max<std::int64_t>(1, half_width / 2);
    }
    return false;
  };

  for (std::size_t segment = 1U; segment < run.points.size(); ++segment) {
    const auto start = run.points[segment - 1U];
    const auto end = run.points[segment];
    const auto length = geometry::segment_length_nm(start, end);
    const auto steps = static_cast<std::size_t>(
        std::max<long double>(1.0L, std::ceil(length / static_cast<long double>(sample_spacing))));
    if (steps > 16'384U)
      return std::nullopt;
    auto prior = start;
    for (std::size_t step = 1U; step <= steps; ++step) {
      const auto t = static_cast<long double>(step) / static_cast<long double>(steps);
      const auto point =
          Point{Nanometres{static_cast<std::int64_t>(
                    std::llround(static_cast<long double>(start.x.value()) +
                                 t * static_cast<long double>(end.x.value() - start.x.value())))},
                Nanometres{static_cast<std::int64_t>(
                    std::llround(static_cast<long double>(start.y.value()) +
                                 t * static_cast<long double>(end.y.value() - start.y.value())))}};
      if (point != prior && !append_certified_cell(prior, point))
        return std::nullopt;
      if (result.centerline.back() != point)
        result.centerline.push_back(point);
      prior = point;
    }
  }
  if (result.cells.empty())
    return std::nullopt;
  // The synthetic corridor is one already-certified source route class. Its cells are used only
  // as a union guard, so internal subdivision faces are deliberately not promoted to topology
  // gates.
  result.corridor = {0U};
  result.topology_signature = make_navmesh_topology_signature({});
  result.diagnostic.clear();
  return result;
}

void augment_terminal_planning_chord_corridor(const OffsetFreeSpaceRegion& region,
                                              const SplineRun& original,
                                              const std::vector<Point>& planning_points,
                                              Nanometres cell_pitch,
                                              Nanometres maximum_half_width,
                                              NavMeshRouteResult& navigation) {
  if (planning_points == original.points || original.points.size() < 3U ||
      planning_points.size() < 2U) {
    return;
  }
  const auto append_chord = [&](Point first, Point second) {
    const SplineRun chord{original.layer, original.width, {first, second}, {}};
    const auto certified =
        make_source_polyline_corridor(region, chord, cell_pitch, maximum_half_width);
    if (!certified)
      return;
    const auto offset = navigation.cells.size();
    for (auto cell : certified->cells) {
      cell.id += offset;
      navigation.cells.push_back(std::move(cell));
    }
    navigation.topology_guard_cells.reserve(navigation.topology_guard_cells.size() +
                                            certified->topology_guard_cells.size());
    for (const auto cell : certified->topology_guard_cells)
      navigation.topology_guard_cells.push_back(offset + cell);
  };
  if (planning_points.front() == original.points.front() && original.points.size() >= 3U &&
      planning_points[1U] == original.points[2U]) {
    append_chord(planning_points.front(), planning_points[1U]);
  }
  if (planning_points.back() == original.points.back() && original.points.size() >= 3U &&
      planning_points[planning_points.size() - 2U] == original.points[original.points.size() - 3U]) {
    append_chord(planning_points[planning_points.size() - 2U], planning_points.back());
  }
}

void augment_with_certified_planning_route(const OffsetFreeSpaceRegion& region,
                                           const SplineRun& planning_run,
                                           Nanometres certificate_pitch,
                                           Nanometres maximum_half_width,
                                           NavMeshRouteResult& navigation) {
  const auto certified = make_source_polyline_corridor(region, planning_run, certificate_pitch,
                                                        maximum_half_width);
  if (!certified)
    return;
  const auto offset = navigation.cells.size();
  for (auto cell : certified->cells) {
    cell.id += offset;
    navigation.cells.push_back(std::move(cell));
  }
  navigation.topology_guard_cells.reserve(navigation.topology_guard_cells.size() +
                                          certified->topology_guard_cells.size());
  for (const auto cell : certified->topology_guard_cells)
    navigation.topology_guard_cells.push_back(offset + cell);
}

struct CopperEndpointIncidence {
  bool straight_track{false};
  Point direction;
  std::size_t track_index{std::numeric_limits<std::size_t>::max()};
};

using CopperEndpointKey = std::tuple<NetId, LayerId, Point>;
using CopperEndpointMap = std::map<CopperEndpointKey, std::vector<CopperEndpointIncidence>>;

[[nodiscard]] bool has_additional_same_net_track_contact(
    const Board& board, const NetId& net, const LayerId& layer, Point point,
    const std::vector<CopperEndpointIncidence>& endpoint_incidences) {
  for (std::size_t candidate_index = 0U; candidate_index < board.tracks.size(); ++candidate_index) {
    const auto& candidate = board.tracks[candidate_index];
    if (candidate.net != net || candidate.layer != layer || candidate.start == candidate.end ||
        std::any_of(endpoint_incidences.begin(), endpoint_incidences.end(),
                    [&](const auto& incidence) {
                      return incidence.straight_track && incidence.track_index == candidate_index;
                    })) {
      continue;
    }
    for (const auto& incidence : endpoint_incidences) {
      if (!incidence.straight_track || incidence.track_index >= board.tracks.size())
        continue;
      const auto& incident = board.tracks[incidence.track_index];
      const auto contact_radius =
          (static_cast<long double>(candidate.width.value()) + incident.width.value()) / 2.0L;
      if (geometry::point_segment_distance_nm(point, candidate.start, candidate.end) <=
          contact_radius) {
        return true;
      }
    }
  }
  return false;
}

[[nodiscard]] CopperEndpointMap copper_endpoint_incidences(const Board& board) {
  CopperEndpointMap incidences;
  for (std::size_t index = 0U; index < board.tracks.size(); ++index) {
    const auto& track = board.tracks[index];
    if (track.start == track.end) {
      continue;
    }
    incidences[{track.net, track.layer, track.start}].push_back(
        CopperEndpointIncidence{true,
                                Point{Nanometres{track.end.x.value() - track.start.x.value()},
                                      Nanometres{track.end.y.value() - track.start.y.value()}},
                                index});
    incidences[{track.net, track.layer, track.end}].push_back(
        CopperEndpointIncidence{true,
                                Point{Nanometres{track.start.x.value() - track.end.x.value()},
                                      Nanometres{track.start.y.value() - track.end.y.value()}},
                                index});
  }
  for (const auto& arc : board.track_arcs) {
    if (arc.start == arc.end) {
      continue;
    }
    incidences[{arc.net, arc.layer, arc.start}].push_back(CopperEndpointIncidence{});
    incidences[{arc.net, arc.layer, arc.end}].push_back(CopperEndpointIncidence{});
  }
  for (const auto& bezier : board.track_beziers) {
    if (bezier.start == bezier.end)
      continue;
    incidences[{bezier.net, bezier.layer, bezier.start}].push_back(
        CopperEndpointIncidence{false, bezier.control1 - bezier.start});
    incidences[{bezier.net, bezier.layer, bezier.end}].push_back(
        CopperEndpointIncidence{false, bezier.control2 - bezier.end});
  }
  // The PH chain collector treats pad and via centres as electrical attachment boundaries so a
  // continuous replacement cannot bypass a terminal or layer transition. Mirror that physical
  // topology in the final bend audit. Two straight pieces meeting at such a point are not an
  // unfinished free-space corner: together with the pad/via copper they form a junction. Omitting
  // this incidence made a fully realized route fail only because the auditor and collector used
  // different definitions of a junction.
  for (const auto& via : board.vias) {
    for (const auto& layer : via.layers) {
      incidences[{via.net, layer, via.position}].push_back(CopperEndpointIncidence{});
    }
  }
  for (const auto& footprint : board.footprints) {
    for (const auto& pad : footprint.pads) {
      const auto position = geometry::transform_point(pad.local_position, footprint.position,
                                                      footprint.rotation, footprint.side);
      for (const auto& layer : pad.layers) {
        incidences[{pad.net, layer, position}].push_back(CopperEndpointIncidence{});
      }
    }
  }
  return incidences;
}

struct UnroundedBendCounts {
  std::size_t straight_track{0U};
  std::size_t curve_tangent{0U};

  [[nodiscard]] std::size_t total() const noexcept { return straight_track + curve_tangent; }
};

[[nodiscard]] UnroundedBendCounts count_unrounded_non_junction_bends(const Board& board) {
  const auto incidences = copper_endpoint_incidences(board);

  UnroundedBendCounts bends;
  for (const auto& [key, at_endpoint] : incidences) {
    const auto& [net, layer, point] = key;
    // A true branch is permitted to retain an angular junction. Every degree-two change of
    // direction between straight primitives is unfinished spline geometry, irrespective of its
    // angle. Collinear subdivision is harmless and does not count as a bend.
    if (at_endpoint.size() != 2U ||
        (at_endpoint[0].direction.x.value() == 0 && at_endpoint[0].direction.y.value() == 0) ||
        (at_endpoint[1].direction.x.value() == 0 && at_endpoint[1].direction.y.value() == 0)) {
      continue;
    }
    // KiCad connectivity is copper-area based: a branch endpoint may touch the middle of either
    // incident segment without sharing the exact centreline coordinate. Such a physical T remains
    // a junction and is the one permitted angular case; do not mislabel it as unfinished PH
    // geometry merely because the imported primitives were not split at the contact coordinate.
    if (at_endpoint[0].straight_track && at_endpoint[1].straight_track &&
        has_additional_same_net_track_contact(board, net, layer, point, at_endpoint))
      continue;
    const auto ax = static_cast<long double>(at_endpoint[0].direction.x.value());
    const auto ay = static_cast<long double>(at_endpoint[0].direction.y.value());
    const auto bx = static_cast<long double>(at_endpoint[1].direction.x.value());
    const auto by = static_cast<long double>(at_endpoint[1].direction.y.value());
    const auto cross = ax * by - ay * bx;
    const auto product = std::hypotl(ax, ay) * std::hypotl(bx, by);
    const auto angular_error = product > 0.0L ? std::abs(cross) / product : 1.0L;
    const auto dot = ax * bx + ay * by;
    const auto has_curve_tangent = !at_endpoint[0].straight_track || !at_endpoint[1].straight_track;
    const auto smooth = has_curve_tangent ? (dot < 0.0L && angular_error <= 0.035L)
                                          : (cross == 0.0L);
    if (!smooth) {
      if (at_endpoint[0].straight_track && at_endpoint[1].straight_track)
        ++bends.straight_track;
      else
        ++bends.curve_tangent;
    }
  }
  return bends;
}

[[nodiscard]] std::vector<PendingSplineRun>
collect_unrounded_spline_chains(const Board& board,
                                const std::set<std::string>& protected_track_ids = {},
                                const std::set<std::string>& topology_locked_track_ids = {}) {
  using EndpointKey = std::tuple<NetId, LayerId, Point>;
  struct EndpointNode {
    std::size_t total_incidences{0U};
    std::vector<std::size_t> eligible_tracks;
  };
  std::map<EndpointKey, EndpointNode> nodes;
  for (std::size_t index = 0U; index < board.tracks.size(); ++index) {
    const auto& track = board.tracks[index];
    if (track.start == track.end) {
      continue;
    }
    for (const auto point : {track.start, track.end}) {
      auto& node = nodes[{track.net, track.layer, point}];
      ++node.total_incidences;
      if (!track.locked && !protected_track_ids.contains(track.id.value())) {
        node.eligible_tracks.push_back(index);
      }
    }
  }
  for (const auto& arc : board.track_arcs) {
    if (arc.start == arc.end) {
      continue;
    }
    ++nodes[{arc.net, arc.layer, arc.start}].total_incidences;
    ++nodes[{arc.net, arc.layer, arc.end}].total_incidences;
  }
  for (const auto& curve : board.track_beziers) {
    if (curve.start == curve.end) continue;
    ++nodes[{curve.net, curve.layer, curve.start}].total_incidences;
    ++nodes[{curve.net, curve.layer, curve.end}].total_incidences;
  }
  // Pads and vias are electrical attachment boundaries even when exactly two straight tracks meet
  // at their centre. Treating that point as an ordinary degree-two continuation lets a smoothed
  // chord bypass the attachment and is detected only by the later connectivity transaction.
  for (const auto& via : board.vias) {
    for (const auto& layer : via.layers)
      ++nodes[{via.net, layer, via.position}].total_incidences;
  }
  for (const auto& footprint : board.footprints) {
    for (const auto& pad : footprint.pads) {
      const auto position = geometry::transform_point(pad.local_position, footprint.position,
                                                      footprint.rotation, footprint.side);
      for (const auto& layer : pad.layers)
        ++nodes[{pad.net, layer, position}].total_incidences;
    }
  }
  const auto continuation = [&](const TrackSegment& track, Point endpoint,
                                std::size_t current) -> std::optional<std::size_t> {
    const auto found = nodes.find({track.net, track.layer, endpoint});
    if (found == nodes.end() || found->second.total_incidences != 2U ||
        found->second.eligible_tracks.size() != 2U) {
      return std::nullopt;
    }
    for (const auto candidate : found->second.eligible_tracks) {
      if (candidate != current && candidate < board.tracks.size() &&
          board.tracks[candidate].width == track.width) {
        return candidate;
      }
    }
    return std::nullopt;
  };
  const auto boundary = [&](std::size_t track, Point endpoint) {
    return !continuation(board.tracks[track], endpoint, track).has_value();
  };

  std::vector<PendingSplineRun> result;
  std::vector<std::uint8_t> visited(board.tracks.size(), 0U);
  std::size_t serial = 0U;
  const auto append_chain = [&](std::size_t first, Point start,
                                std::vector<PendingSplineRun>& output) {
    if (first >= board.tracks.size() || visited[first] != 0U || board.tracks[first].locked ||
        protected_track_ids.contains(board.tracks[first].id.value())) {
      return;
    }
    const auto& first_track = board.tracks[first];
    SplineRun run{first_track.layer, first_track.width, {start}, {}};
    auto current = first;
    auto endpoint = start;
    while (current < board.tracks.size() && visited[current] == 0U) {
      const auto& track = board.tracks[current];
      if (track.locked || track.net != first_track.net || track.layer != first_track.layer ||
          track.width != first_track.width || (track.start != endpoint && track.end != endpoint)) {
        break;
      }
      visited[current] = 1U;
      run.original_track_ids.push_back(track.id.value());
      endpoint = track.start == endpoint ? track.end : track.start;
      run.points.push_back(endpoint);
      const auto next = continuation(track, endpoint, current);
      if (!next || visited[*next] != 0U) {
        break;
      }
      current = *next;
    }
    const bool has_bend =
        run.points.size() >= 3U &&
        std::any_of(run.points.begin() + 1, run.points.end() - 1, [&](const auto& point) {
          const auto index = static_cast<std::size_t>(&point - run.points.data());
          const auto& before = run.points[index - 1U];
          const auto& after = run.points[index + 1U];
          const auto first_x = point.x.value() - before.x.value();
          const auto first_y = point.y.value() - before.y.value();
          const auto second_x = after.x.value() - point.x.value();
          const auto second_y = after.y.value() - point.y.value();
          return static_cast<long double>(first_x) * second_y !=
                 static_cast<long double>(first_y) * second_x;
        });
    if (has_bend) {
      run.topology_locked = !run.original_track_ids.empty() &&
          std::all_of(run.original_track_ids.begin(), run.original_track_ids.end(),
                      [&topology_locked_track_ids](const std::string& id) {
                        return topology_locked_track_ids.contains(id);
                      });
      output.push_back(PendingSplineRun{
          first_track.net, "flux-board-spline-" + std::to_string(serial++), std::move(run)});
    }
  };

  // Begin at physical or policy boundaries first so each open chain is ordered end-to-end.
  for (std::size_t track = 0U; track < board.tracks.size(); ++track) {
    if (visited[track] != 0U || board.tracks[track].locked) {
      continue;
    }
    if (boundary(track, board.tracks[track].start)) {
      append_chain(track, board.tracks[track].start, result);
    } else if (boundary(track, board.tracks[track].end)) {
      append_chain(track, board.tracks[track].end, result);
    }
  }
  // Closed degree-two components have no boundary. They are uncommon in routed copper but remain
  // bounded and deterministic rather than being silently skipped.
  for (std::size_t track = 0U; track < board.tracks.size(); ++track) {
    if (visited[track] == 0U && !board.tracks[track].locked) {
      append_chain(track, board.tracks[track].start, result);
    }
  }
  return result;
}

[[nodiscard]] std::optional<TrackArc>
make_circular_fillet(Point previous, Point corner, Point next, Nanometres maximum_radius) {
  const auto first_x = static_cast<long double>(previous.x.value() - corner.x.value());
  const auto first_y = static_cast<long double>(previous.y.value() - corner.y.value());
  const auto second_x = static_cast<long double>(next.x.value() - corner.x.value());
  const auto second_y = static_cast<long double>(next.y.value() - corner.y.value());
  const auto first_length = std::hypot(first_x, first_y);
  const auto second_length = std::hypot(second_x, second_y);
  const auto determinant = first_x * second_y - first_y * second_x;
  if (first_length <= 1.0L || second_length <= 1.0L || determinant == 0.0L)
    return std::nullopt;
  const auto first_unit_x = first_x / first_length;
  const auto first_unit_y = first_y / first_length;
  const auto second_unit_x = second_x / second_length;
  const auto second_unit_y = second_y / second_length;
  const auto cosine =
      std::clamp(first_unit_x * second_unit_x + first_unit_y * second_unit_y, -1.0L, 1.0L);
  const auto half_angle = std::acos(cosine) / 2.0L;
  const auto tangent = std::tan(half_angle);
  const auto sine = std::sin(half_angle);
  if (tangent <= 1.0e-12L || sine <= 1.0e-12L)
    return std::nullopt;
  const auto requested_radius = static_cast<long double>(maximum_radius.value());
  auto trim = requested_radius / tangent;
  trim = std::min({trim, first_length * 0.45L, second_length * 0.45L});
  const auto radius = trim * tangent;
  if (trim <= 1.0L || radius <= 1.0L)
    return std::nullopt;
  const auto bisector_x = first_unit_x + second_unit_x;
  const auto bisector_y = first_unit_y + second_unit_y;
  const auto bisector_length = std::hypot(bisector_x, bisector_y);
  if (bisector_length <= 1.0e-12L)
    return std::nullopt;
  const auto corner_x = static_cast<long double>(corner.x.value());
  const auto corner_y = static_cast<long double>(corner.y.value());
  const auto center_distance = radius / sine;
  const auto center_x = corner_x + bisector_x / bisector_length * center_distance;
  const auto center_y = corner_y + bisector_y / bisector_length * center_distance;
  const auto quantize = [](long double x, long double y) {
    return Point{Nanometres{static_cast<std::int64_t>(std::llround(x))},
                 Nanometres{static_cast<std::int64_t>(std::llround(y))}};
  };
  const auto arc_start = quantize(corner_x + first_unit_x * trim, corner_y + first_unit_y * trim);
  const auto arc_end = quantize(corner_x + second_unit_x * trim, corner_y + second_unit_y * trim);
  const auto toward_corner_x = corner_x - center_x;
  const auto toward_corner_y = corner_y - center_y;
  const auto toward_corner_length = std::hypot(toward_corner_x, toward_corner_y);
  const auto arc_middle = quantize(center_x + toward_corner_x / toward_corner_length * radius,
                                   center_y + toward_corner_y / toward_corner_length * radius);
  const auto arc = geometry::circular_arc_parameters(arc_start, arc_middle, arc_end);
  if (!arc)
    return std::nullopt;
  const auto join_is_smooth = [](long double ax, long double ay, long double bx, long double by) {
    const auto product = std::hypot(ax, ay) * std::hypot(bx, by);
    return product > 0.0L && ax * bx + ay * by > 0.0L &&
           std::abs(ax * by - ay * bx) / product <= 0.035L;
  };
  const auto sweep_sign = arc->sweep_radians > 0.0L ? 1.0L : -1.0L;
  const auto arc_tangent = [&](Point point) {
    return std::pair{-sweep_sign * (static_cast<long double>(point.y.value()) - arc->center_y_nm),
                     sweep_sign * (static_cast<long double>(point.x.value()) - arc->center_x_nm)};
  };
  const auto [start_tx, start_ty] = arc_tangent(arc_start);
  const auto [end_tx, end_ty] = arc_tangent(arc_end);
  const auto incoming_x = static_cast<long double>(arc_start.x.value() - previous.x.value());
  const auto incoming_y = static_cast<long double>(arc_start.y.value() - previous.y.value());
  const auto outgoing_x = static_cast<long double>(next.x.value() - arc_end.x.value());
  const auto outgoing_y = static_cast<long double>(next.y.value() - arc_end.y.value());
  // Quantization is authoritative copper geometry. Reject an arc whose quantized endpoint
  // tangents no longer meet the final audit's 0.035 angular-error tolerance.
  if (!join_is_smooth(incoming_x, incoming_y, start_tx, start_ty) ||
      !join_is_smooth(end_tx, end_ty, outgoing_x, outgoing_y))
    return std::nullopt;
  return TrackArc{ObjectId{}, NetId{}, LayerId{}, arc_start, arc_middle, arc_end, Nanometres{}, false};
}

// Repairs only the degree-two straight chains that survived PH realization. It deliberately uses
// the original exact board transaction: each candidate is independently validated and rejected
// unless it preserves connectivity and introduces no copper violation involving changed copper.
[[nodiscard]] bool repair_residual_straight_bends(
    Board& board, std::set<std::string>& used_ids, DetailedRouteResult& result,
    const DetailedRouterConfig& config, const std::set<std::string>& protected_track_ids,
    std::stop_token stop_token) {
  if (!config.residual_bend_repair || config.max_residual_bend_repair_batches == 0U ||
      config.max_residual_bend_repair_runs == 0U || config.max_residual_bend_repair_corners == 0U)
    return true;
  // `scaffold_track_ids` suppresses the primary PH refit so certified scaffold centerlines stay
  // intact. This exact local trim retains both incident route endpoints and is separately
  // certified, so it may consider those mutable generated tracks. Source/native protection is
  // still supplied through protected_track_ids and TrackSegment::locked.
  const auto runs = collect_unrounded_spline_chains(board, protected_track_ids);
  const auto per_net_connectivity_regressed = [](const ConnectivityReport& before,
                                                  const ConnectivityReport& after) {
    for (const auto& prior : before.nets) {
      const auto current = std::find_if(after.nets.begin(), after.nets.end(), [&](const auto& net) {
        return net.net == prior.net;
      });
      if (current == after.nets.end() || current->open_connections > prior.open_connections)
        return true;
    }
    return false;
  };
  const auto audited_straight_bend = [](const Board& current, const NetId& net,
                                   const LayerId& layer, Point point) {
    const auto incidences = copper_endpoint_incidences(current);
    const auto found = incidences.find({net, layer, point});
    if (found == incidences.end() || found->second.size() != 2U ||
        !found->second[0].straight_track || !found->second[1].straight_track ||
        (found->second[0].direction.x.value() == 0 &&
         found->second[0].direction.y.value() == 0) ||
        (found->second[1].direction.x.value() == 0 &&
         found->second[1].direction.y.value() == 0) ||
        has_additional_same_net_track_contact(current, net, layer, point, found->second))
      return false;
    const auto ax = static_cast<long double>(found->second[0].direction.x.value());
    const auto ay = static_cast<long double>(found->second[0].direction.y.value());
    const auto bx = static_cast<long double>(found->second[1].direction.x.value());
    const auto by = static_cast<long double>(found->second[1].direction.y.value());
    return ax * by != ay * bx;
  };
  std::size_t run_index = 0U;
  std::size_t corner = 1U;
  for (std::size_t batch = 0U;
       batch < config.max_residual_bend_repair_batches && run_index < runs.size(); ++batch) {
    routing_kernel_checkpoint();
    if (stop_token.stop_requested())
      return false;
    ++result.residual_bend_repair_batches;
    std::size_t runs_in_batch = 0U;
    std::size_t eligible_corners_in_batch = 0U;
    bool yielded_for_run_budget = false;
    bool yielded_for_corner_budget = false;
    while (run_index < runs.size()) {
      if (runs_in_batch >= config.max_residual_bend_repair_runs) {
        yielded_for_run_budget = true;
        break;
      }
      ++runs_in_batch;
      ++result.residual_bend_repair_scanned_runs;
      const auto& pending = runs[run_index];
      const auto& run = pending.run;
      // A floating closed component has no deterministic seam predecessor, so keep its audited
      // seam for the final audit. A closed run rooted at an audit-exempt pad/via junction can
      // still repair every interior corner without moving that physical attachment.
      const bool closed = run.points.size() >= 2U && run.points.front() == run.points.back();
      if (run.topology_locked || run.points.size() < 3U ||
          (closed && audited_straight_bend(board, pending.net, run.layer, run.points.front())) ||
          run.original_track_ids.size() + 1U != run.points.size()) {
        ++run_index;
        corner = 1U;
        continue;
      }
      while (corner + 1U < run.points.size()) {
        routing_kernel_checkpoint();
        if (stop_token.stop_requested())
          return false;
        ++result.residual_bend_repair_scanned_points;
        const auto corner_index = corner++;
        if (result.fitted_circular_arcs >= config.max_circular_arcs ||
            result.attempted_circular_arc_candidates >= config.max_circular_arc_candidate_checks) {
          result.circular_arc_candidate_budget_exhausted =
              result.attempted_circular_arc_candidates >= config.max_circular_arc_candidate_checks;
          return true;
        }
        const auto incoming = std::find_if(board.tracks.begin(), board.tracks.end(), [&](const auto& track) {
          return track.id.value() == run.original_track_ids[corner_index - 1U];
        });
        const auto outgoing = std::find_if(board.tracks.begin(), board.tracks.end(), [&](const auto& track) {
          return track.id.value() == run.original_track_ids[corner_index];
        });
        if (incoming == board.tracks.end() || outgoing == board.tracks.end() || incoming->locked ||
            outgoing->locked || incoming->net != pending.net || outgoing->net != pending.net ||
            incoming->layer != run.layer || outgoing->layer != run.layer || incoming->width != run.width ||
            outgoing->width != run.width)
          continue;
        const auto shared_endpoint = [&]() -> std::optional<Point> {
          for (const auto point : {incoming->start, incoming->end})
            if (point == outgoing->start || point == outgoing->end)
              return point;
          return std::nullopt;
        }();
        if (!shared_endpoint || !audited_straight_bend(board, pending.net, run.layer, *shared_endpoint))
          continue;
        if (eligible_corners_in_batch >= config.max_residual_bend_repair_corners) {
          --corner;
          yielded_for_corner_budget = true;
          break;
        }
        ++eligible_corners_in_batch;
        ++result.residual_bend_repair_attempted;
        ++result.attempted_circular_arc_corners;
        const auto incoming_index = static_cast<std::size_t>(incoming - board.tracks.begin());
        const auto outgoing_index = static_cast<std::size_t>(outgoing - board.tracks.begin());
        const auto original_incoming = board.tracks[incoming_index];
        const auto original_outgoing = board.tracks[outgoing_index];
        const auto previous = original_incoming.start == *shared_endpoint ? original_incoming.end : original_incoming.start;
        const auto next = original_outgoing.start == *shared_endpoint ? original_outgoing.end : original_outgoing.start;
        Board connectivity_before = board;
        if (config.defer_saved_zone_fill_clearance_to_refill)
          for (auto& zone : connectivity_before.zones) zone.fills.clear();
        CopperTransactionValidator transaction_validator(connectivity_before);
        bool accepted = false;
        std::optional<std::tuple<Point, Point, Point>> previous_geometry;
        auto radius = config.maximum_circular_arc_radius;
        for (std::size_t backtrack = 0U; backtrack <= config.circular_arc_backtracking_steps;
             ++backtrack, radius = Nanometres{radius.value() / 2}) {
          routing_kernel_checkpoint();
          if (stop_token.stop_requested()) return false;
          if (radius.value() <= 1 || result.attempted_circular_arc_candidates >= config.max_circular_arc_candidate_checks)
            break;
          auto arc = make_circular_fillet(previous, *shared_endpoint, next, radius);
          if (!arc) break;
          const auto geometry = std::tuple{arc->start, arc->middle, arc->end};
          if (previous_geometry && *previous_geometry == geometry) continue;
          previous_geometry = geometry;
          ++result.attempted_circular_arc_candidates;
          arc->id = unique_id(pending.base + "-residual-arc-" + std::to_string(corner_index), used_ids);
          arc->net = pending.net; arc->layer = run.layer; arc->width = run.width;
          auto& mutable_incoming = board.tracks[incoming_index];
          auto& mutable_outgoing = board.tracks[outgoing_index];
          if (mutable_incoming.end == *shared_endpoint) mutable_incoming.end = arc->start;
          else mutable_incoming.start = arc->start;
          if (mutable_outgoing.start == *shared_endpoint) mutable_outgoing.start = arc->end;
          else mutable_outgoing.end = arc->end;
          board.track_arcs.push_back(*arc);
          Board probe = board;
          if (config.defer_saved_zone_fill_clearance_to_refill)
            for (auto& zone : probe.zones) zone.fills.clear();
          const auto transaction_report = transaction_validator.validate(probe);
          record_transaction_metrics(result, transaction_report);
          const auto& validation = transaction_report.structural;
          const auto& copper = transaction_report.copper;
          const std::array changed_ids{arc->id.value(), mutable_incoming.id.value(), mutable_outgoing.id.value()};
          const auto changed_copper_violation = std::any_of(copper.violations.begin(), copper.violations.end(),
              [&changed_ids](const auto& violation) {
                if (violation.kind == CopperViolationKind::differential_pair_skew) return false;
                return std::any_of(changed_ids.begin(), changed_ids.end(), [&](const auto& id) {
                  return std::find(violation.object_ids.begin(), violation.object_ids.end(), id) != violation.object_ids.end();
                });
              });
          if (validation.ok() && !changed_copper_violation && transaction_report.preserves_pad_components &&
              !per_net_connectivity_regressed(transaction_report.before_connectivity,
                                               transaction_report.after_connectivity)) {
            accepted = true;
            ++result.residual_bend_repair_accepted;
            ++result.added_track_arcs;
            ++result.fitted_circular_arcs;
            if (backtrack > 0U) ++result.radius_reduced_circular_arcs;
            used_ids.insert(arc->id.value());
            break;
          }
          board.track_arcs.pop_back();
          board.tracks[incoming_index] = original_incoming;
          board.tracks[outgoing_index] = original_outgoing;
          ++result.rejected_circular_arc_candidates;
        }
        if (!accepted) {
          ++result.residual_bend_repair_rejected;
          ++result.withheld_circular_arc_corners;
        }
      }
      if (yielded_for_corner_budget) break;
      ++run_index;
      corner = 1U;
    }
    if (yielded_for_run_budget) ++result.residual_bend_repair_run_budget_exhaustions;
    if (yielded_for_corner_budget) ++result.residual_bend_repair_corner_budget_exhaustions;
  }
  if (run_index < runs.size()) result.residual_bend_repair_budget_exhausted = true;
  return true;
}

[[nodiscard]] bool navmesh_chord_stays_in_selected_corridor(const NavMeshRouteResult& navigation,
                                                            Point start, Point end,
                                                            Nanometres sample_spacing) {
  const auto length = geometry::segment_length_nm(start, end);
  if (!(length > 0.0L) || sample_spacing.value() <= 0) {
    return start == end;
  }
  const auto steps = static_cast<std::size_t>(std::max<long double>(
      1.0L, std::ceil(length / static_cast<long double>(sample_spacing.value()))));
  constexpr std::size_t maximum_samples_per_chord = 4096U;
  if (steps > maximum_samples_per_chord) {
    return false;
  }
  const auto dx = static_cast<long double>(end.x.value() - start.x.value());
  const auto dy = static_cast<long double>(end.y.value() - start.y.value());
  for (std::size_t sample = 0U; sample <= steps; ++sample) {
    const auto parameter = static_cast<long double>(sample) / static_cast<long double>(steps);
    const auto point =
        Point{Nanometres{static_cast<std::int64_t>(
                  std::llround(static_cast<long double>(start.x.value()) + parameter * dx))},
              Nanometres{static_cast<std::int64_t>(
                  std::llround(static_cast<long double>(start.y.value()) + parameter * dy))}};
    if (!navmesh_corridor_contains(navigation, point)) {
      return false;
    }
  }
  return true;
}

[[nodiscard]] std::vector<Point>
string_pull_detailed_spline_seed(const NavMeshRouteResult& navigation, Nanometres sample_spacing) {
  const auto& source = navigation.centerline;
  if (source.size() <= 2U) {
    return source;
  }
  std::vector<Point> result{source.front()};
  std::size_t anchor = 0U;
  while (anchor + 1U < source.size()) {
    auto farthest = anchor + 1U;
    for (auto candidate = farthest + 1U;
         candidate < source.size() &&
         navmesh_chord_stays_in_selected_corridor(navigation, source[anchor], source[candidate],
                                                  sample_spacing);
         ++candidate) {
      farthest = candidate;
    }
    if (result.back() != source[farthest]) {
      result.push_back(source[farthest]);
    }
    anchor = farthest;
  }
  return result;
}

[[nodiscard]] std::vector<Point> relax_detailed_spline_seed(const NavMeshRouteResult& navigation,
                                                            std::vector<Point> points,
                                                            Nanometres sample_spacing,
                                                            std::size_t passes) {
  if (points.size() < 3U)
    return points;
  for (std::size_t pass = 0U; pass < passes; ++pass) {
    auto candidate = points;
    bool moved = false;
    for (std::size_t point = 1U; point + 1U < points.size(); ++point) {
      // One Laplacian rubber-band step removes lattice-frequency direction changes without
      // changing endpoints. A point moves only when both incident replacement chords remain in
      // the already selected topology guard, so this is a seed improvement rather than a new
      // corridor search.
      const auto proposed =
          Point{Nanometres{(points[point - 1U].x.value() + 2 * points[point].x.value() +
                            points[point + 1U].x.value()) /
                           4},
                Nanometres{(points[point - 1U].y.value() + 2 * points[point].y.value() +
                            points[point + 1U].y.value()) /
                           4}};
      if (proposed == points[point])
        continue;
      if (navmesh_chord_stays_in_selected_corridor(navigation, candidate[point - 1U], proposed,
                                                   sample_spacing) &&
          navmesh_chord_stays_in_selected_corridor(navigation, proposed, points[point + 1U],
                                                   sample_spacing)) {
        candidate[point] = proposed;
        moved = true;
      }
    }
    if (!moved)
      break;
    points = std::move(candidate);
  }
  return points;
}

// The sampled cell sequence is an implementation detail of the NavMesh search, not a homotopy
// invariant. A smooth curve will normally cross different lattice portals from the funnel chord
// even while staying on the same side of every physical obstacle. Preserve the actual route class
// by requiring the candidate to remain inside the selected guard tube and by comparing the winding
// of candidate-versus-funnel loops around every free-space hole.
[[nodiscard]] std::optional<Point> polygon_interior_representative(const Polygon& polygon) {
  if (polygon.vertices.size() < 3U)
    return std::nullopt;
  std::int64_t minimum_x = polygon.vertices.front().x.value();
  std::int64_t maximum_x = minimum_x;
  std::int64_t minimum_y = polygon.vertices.front().y.value();
  std::int64_t maximum_y = minimum_y;
  long double sum_x = 0.0L;
  long double sum_y = 0.0L;
  for (const auto point : polygon.vertices) {
    minimum_x = std::min(minimum_x, point.x.value());
    maximum_x = std::max(maximum_x, point.x.value());
    minimum_y = std::min(minimum_y, point.y.value());
    maximum_y = std::max(maximum_y, point.y.value());
    sum_x += static_cast<long double>(point.x.value());
    sum_y += static_cast<long double>(point.y.value());
  }
  const auto inside = [&](long double x, long double y) -> std::optional<Point> {
    const auto candidate = Point{Nanometres{static_cast<std::int64_t>(std::llround(x))},
                                 Nanometres{static_cast<std::int64_t>(std::llround(y))}};
    return geometry::locate_point(polygon, candidate) == geometry::PointLocation::inside
               ? std::optional<Point>{candidate}
               : std::nullopt;
  };
  if (const auto centroid = inside(sum_x / static_cast<long double>(polygon.vertices.size()),
                                   sum_y / static_cast<long double>(polygon.vertices.size()))) {
    return centroid;
  }
  if (const auto centre = inside((static_cast<long double>(minimum_x) + maximum_x) * 0.5L,
                                 (static_cast<long double>(minimum_y) + maximum_y) * 0.5L)) {
    return centre;
  }
  // Concave offset contours can place both simple centroids outside. A small deterministic grid
  // is sufficient to find an interior witness without adding a second polygon representation.
  constexpr std::size_t probes = 16U;
  for (std::size_t y = 0U; y < probes; ++y) {
    for (std::size_t x = 0U; x < probes; ++x) {
      const auto candidate_x =
          static_cast<long double>(minimum_x) + static_cast<long double>(maximum_x - minimum_x) *
                                                    (static_cast<long double>(x) + 0.5L) /
                                                    static_cast<long double>(probes);
      const auto candidate_y =
          static_cast<long double>(minimum_y) + static_cast<long double>(maximum_y - minimum_y) *
                                                    (static_cast<long double>(y) + 0.5L) /
                                                    static_cast<long double>(probes);
      if (const auto candidate = inside(candidate_x, candidate_y))
        return candidate;
    }
  }
  return std::nullopt;
}

[[nodiscard]] long long closed_route_winding_difference(const std::vector<Point>& reference,
                                                        const std::vector<Point>& candidate,
                                                        Point obstacle) {
  if (reference.size() < 2U || candidate.size() < 2U)
    return 1;
  long double angle = 0.0L;
  const auto add_edge = [&](Point first, Point second) {
    const auto first_x = static_cast<long double>(first.x.value() - obstacle.x.value());
    const auto first_y = static_cast<long double>(first.y.value() - obstacle.y.value());
    const auto second_x = static_cast<long double>(second.x.value() - obstacle.x.value());
    const auto second_y = static_cast<long double>(second.y.value() - obstacle.y.value());
    angle += std::atan2(first_x * second_y - first_y * second_x,
                        first_x * second_x + first_y * second_y);
  };
  for (std::size_t point = 1U; point < reference.size(); ++point)
    add_edge(reference[point - 1U], reference[point]);
  for (std::size_t point = candidate.size() - 1U; point > 0U; --point)
    add_edge(candidate[point], candidate[point - 1U]);
  constexpr long double two_pi = 2.0L * std::numbers::pi_v<long double>;
  return std::llround(angle / two_pi);
}

[[nodiscard]] bool detailed_spline_preserves_route_class(const NavMeshRouteResult& navigation,
                                                         const OffsetFreeSpaceRegion& region,
                                                         const std::vector<Point>& candidate,
                                                         Nanometres sample_spacing,
                                                         bool corridor_containment_certified,
                                                         std::string& diagnostic) {
  if (candidate.size() < 2U || navigation.centerline.size() < 2U ||
      candidate.front() != navigation.centerline.front() ||
      candidate.back() != navigation.centerline.back()) {
    diagnostic = "continuous route endpoints differ from its selected NavMesh corridor";
    return false;
  }
  if (!corridor_containment_certified) {
    for (std::size_t point = 1U; point < candidate.size(); ++point) {
      if (!navmesh_chord_stays_in_selected_corridor(navigation, candidate[point - 1U],
                                                    candidate[point], sample_spacing)) {
        diagnostic = "continuous route left the selected NavMesh guard tube";
        return false;
      }
    }
  }
  for (const auto& hole : region.holes) {
    const auto representative = polygon_interior_representative(hole);
    if (!representative) {
      diagnostic = "could not construct an interior witness for a NavMesh obstacle";
      return false;
    }
    if (closed_route_winding_difference(navigation.centerline, candidate, *representative) != 0) {
      diagnostic = "continuous route changed obstacle-side homotopy inside its NavMesh guard";
      return false;
    }
  }
  diagnostic = "continuous route retained guard containment and obstacle winding class";
  return true;
}

// A topology-tight seed adds distinct approach/departure controls around a funnel corner. Repeated
// coincident controls create zero-speed spans and are deliberately forbidden by shape validity.
[[nodiscard]] std::optional<QuinticPHRoute>
make_topology_tight_detailed_spline(const std::vector<Point>& points, Nanometres guard) {
  return make_corner_rounded_quintic_ph_route(points, guard);
}

[[nodiscard]] std::optional<Point> obstacle_reference_point(const Board& board, std::string_view id,
                                                            Point collision) {
  const auto track = std::find_if(board.tracks.begin(), board.tracks.end(),
                                  [&](const auto& item) { return item.id.value() == id; });
  if (track != board.tracks.end()) {
    const auto dx = static_cast<long double>(track->end.x.value() - track->start.x.value());
    const auto dy = static_cast<long double>(track->end.y.value() - track->start.y.value());
    const auto length_squared = dx * dx + dy * dy;
    const auto parameter =
        length_squared > 0.0L
            ? std::clamp(
                  (static_cast<long double>(collision.x.value() - track->start.x.value()) * dx +
                   static_cast<long double>(collision.y.value() - track->start.y.value()) * dy) /
                      length_squared,
                  0.0L, 1.0L)
            : 0.0L;
    return Point{Nanometres{static_cast<std::int64_t>(std::llround(
                     static_cast<long double>(track->start.x.value()) + parameter * dx))},
                 Nanometres{static_cast<std::int64_t>(std::llround(
                     static_cast<long double>(track->start.y.value()) + parameter * dy))}};
  }
  const auto arc = std::find_if(board.track_arcs.begin(), board.track_arcs.end(),
                                [&](const auto& item) { return item.id.value() == id; });
  if (arc != board.track_arcs.end()) {
    return arc->middle;
  }
  const auto curve = std::find_if(board.track_beziers.begin(), board.track_beziers.end(),
                                  [&](const auto& item) { return item.id.value() == id; });
  if (curve != board.track_beziers.end()) {
    const auto midpoint = [](Nanometres a, Nanometres b, Nanometres c, Nanometres d) {
      return Nanometres{static_cast<std::int64_t>(std::llround(
          (static_cast<long double>(a.value()) + 3.0L * b.value() + 3.0L * c.value() + d.value()) / 8.0L))};
    };
    return Point{midpoint(curve->start.x, curve->control1.x, curve->control2.x, curve->end.x),
                 midpoint(curve->start.y, curve->control1.y, curve->control2.y, curve->end.y)};
  }
  const auto via = std::find_if(board.vias.begin(), board.vias.end(),
                                [&](const auto& item) { return item.id.value() == id; });
  if (via != board.vias.end()) {
    return via->position;
  }
  for (const auto& footprint : board.footprints) {
    for (const auto& pad : footprint.pads) {
      if (pad.id.value() == id) {
        return geometry::transform_point(pad.local_position, footprint.position, footprint.rotation,
                                         footprint.side);
      }
    }
  }
  return std::nullopt;
}

struct SplineProbe {
  Board board;
  std::vector<std::string> generated_ids;
  std::map<std::string, long double> parameter_by_id;
  std::size_t tracks{0};
  std::size_t arcs{0};
  std::size_t beziers{0};
};

[[nodiscard]] std::optional<SplineProbe>
materialize_spline_probe(const Board& source, const SplineRun& run,
                         const std::vector<SplineFittedPiece>& pieces, const std::string& base,
                         std::size_t serial, std::set<std::string>& used_ids) {
  if (pieces.empty()) {
    return std::nullopt;
  }
  SplineProbe result;
  result.board = source;
  for (std::size_t index = 0U; index < pieces.size(); ++index) {
    const auto& piece = pieces[index];
    if (piece.start == piece.end) {
      return std::nullopt;
    }
    const auto id = unique_id(
        base + "-spline-" + std::to_string(serial) + "-" + std::to_string(index), used_ids);
    result.generated_ids.push_back(id.value());
    result.parameter_by_id.emplace(id.value(),
                                   (piece.first_parameter + piece.last_parameter) / 2.0L);
    if (piece.kind == SplineFittedPieceKind::circular_arc) {
      result.board.track_arcs.push_back(
          TrackArc{id, NetId{}, run.layer, piece.start, piece.middle, piece.end, run.width, false});
      ++result.arcs;
    } else if (piece.kind == SplineFittedPieceKind::cubic_bezier) {
      result.board.track_beziers.push_back(TrackBezier{
          id, NetId{}, run.layer, piece.start, piece.control1, piece.control2, piece.end,
          run.width, false});
      ++result.beziers;
    } else {
      result.board.tracks.push_back(
          TrackSegment{id, NetId{}, run.layer, piece.start, piece.end, run.width, false});
      ++result.tracks;
    }
  }
  return result;
}

void assign_spline_net(SplineProbe& probe, const NetId& net) {
  const std::set<std::string> ids(probe.generated_ids.begin(), probe.generated_ids.end());
  for (auto& track : probe.board.tracks) {
    if (ids.contains(track.id.value())) {
      track.net = net;
    }
  }
  for (auto& arc : probe.board.track_arcs) {
    if (ids.contains(arc.id.value())) {
      arc.net = net;
    }
  }
  for (auto& bezier : probe.board.track_beziers) {
    if (ids.contains(bezier.id.value()))
      bezier.net = net;
  }
}

void record_transaction_metrics(DetailedRouteResult& result, const CopperTransactionReport& report) {
  ++result.incremental_transaction_checks;
  result.incremental_transaction_changed_objects += report.changed_objects;
  result.incremental_transaction_neighbours += report.neighbour_objects;
  result.incremental_transaction_affected_nets += report.affected_nets;
  result.incremental_transaction_connectivity_hits += report.connectivity_cache_hits;
  result.incremental_transaction_full_fallbacks += report.used_full_fallback ? 1U : 0U;
}

[[nodiscard]] const CopperViolation*
first_spline_collision(const CopperDrcReport& report,
                       const std::vector<std::string>& generated_ids) {
  return [&]() -> const CopperViolation* {
    for (const auto& violation : report.violations) {
      if (violation.kind == CopperViolationKind::differential_pair_skew) {
        continue;
      }
      if (std::any_of(generated_ids.begin(), generated_ids.end(), [&](const auto& id) {
            return std::find(violation.object_ids.begin(), violation.object_ids.end(), id) !=
                   violation.object_ids.end();
          })) {
        return &violation;
      }
    }
    return nullptr;
  }();
}

[[nodiscard]] bool is_pad_attachment(const Board& board, const NetId& net, const LayerId& layer,
                                     Point point) {
  return std::any_of(board.footprints.begin(), board.footprints.end(), [&](const auto& footprint) {
    return std::any_of(footprint.pads.begin(), footprint.pads.end(), [&](const auto& pad) {
      if (pad.net != net ||
          std::find(pad.layers.begin(), pad.layers.end(), layer) == pad.layers.end()) {
        return false;
      }
      return geometry::transform_point(pad.local_position, footprint.position, footprint.rotation,
                                       footprint.side) == point;
    });
  });
}

// A sub-cell pad run-up is terminal-access planning evidence, not a mandatory copper tangent.
// Retain the physical pad endpoint but remove the adjacent interpolation point when the direct
// chord is independently legal in the clearance-offset domain and closes a zero-winding loop
// around every obstacle. The complete quantized curve remains subject to connectivity and DRC.
[[nodiscard]] std::vector<Point> remove_legal_terminal_planning_stubs(
    const Board& board, const NetId& net, const LayerId& layer,
    const OffsetFreeSpaceRegion& region, const std::vector<Point>& source,
    Nanometres maximum_stub_length) {
  if (source.size() < 3U || maximum_stub_length.value() <= 0)
    return source;

  FreeSpaceDomain domain;
  domain.outline = region.area;
  domain.cutouts = region.holes;
  auto result = source;
  const auto same_homotopy = [&](const std::vector<Point>& candidate) {
    for (const auto& hole : region.holes) {
      const auto witness = polygon_interior_representative(hole);
      if (!witness || closed_route_winding_difference(result, candidate, *witness) != 0)
        return false;
    }
    return true;
  };
  const auto try_remove = [&](bool at_start) {
    if (result.size() < 3U)
      return;
    const auto endpoint = at_start ? result.front() : result.back();
    if (!is_pad_attachment(board, net, layer, endpoint))
      return;
    const auto adjacent = at_start ? result[1U] : result[result.size() - 2U];
    if (geometry::segment_length_nm(endpoint, adjacent) >
        static_cast<long double>(maximum_stub_length.value())) {
      return;
    }
    const auto chord_other = at_start ? result[2U] : result[result.size() - 3U];
    if (!assess_free_space_chord(domain, endpoint, chord_other))
      return;
    auto candidate = result;
    candidate.erase(candidate.begin() +
                    static_cast<std::ptrdiff_t>(at_start ? 1U : candidate.size() - 2U));
    if (same_homotopy(candidate))
      result = std::move(candidate);
  };
  try_remove(true);
  try_remove(false);
  return result;
}

[[nodiscard]] SplineNavigationPreparation prepare_spline_navigation(
    const Board& navigation_board, const NetId& net, const SplineRun& run,
    const DetailedRouterConfig& config, SplineNavigationPreparationCache* cache) {
  SplineNavigationPreparation preparation;
  const auto extracted = extract_polygonal_board_free_space(
      navigation_board, BoardFreeSpaceExtractionConfig{run.layer, net, true, true,
                                                        !config.defer_saved_zone_fill_clearance_to_refill},
      cache ? &cache->obstacle_geometry : nullptr);
  if (!extracted.complete()) {
    preparation.diagnostic = extracted.diagnostic.empty() ? "unrepresented detailed routing obstacle"
                                                          : extracted.diagnostic;
    return preparation;
  }
  const auto route_radius = Nanometres{(run.width.value() + 1) / 2};
  const auto center_clearance =
      Nanometres{route_radius.value() + navigation_board.rules.copper_to_edge_clearance.value()};
  FreeSpaceOffsetConfig offset_config{center_clearance, 1'000'000U};
  offset_config.obstacle_clearances.reserve(extracted.obstacle_nets.size());
  for (const auto& owner : extracted.obstacle_nets) {
    offset_config.obstacle_clearances.push_back(
        owner ? Nanometres{route_radius.value() +
                           resolve_pairwise_clearance(navigation_board, net, *owner).value()}
              : route_radius);
  }
  offset_config.filled_obstacle_clearances.reserve(extracted.filled_obstacle_nets.size());
  for (const auto& owner : extracted.filled_obstacle_nets) {
    offset_config.filled_obstacle_clearances.push_back(
        owner ? Nanometres{route_radius.value() +
                           resolve_pairwise_clearance(navigation_board, net, *owner).value()}
              : route_radius);
  }
  const auto offset = offset_polygonal_free_space(extracted.domain, offset_config,
      cache ? &cache->obstacle_offsets : nullptr);
  preparation.regions_are_offset = offset.ok();
  if (offset.ok()) {
    preparation.regions = offset.regions;
    preparation.complete = true;
    return preparation;
  }
  const auto unioned = offset_polygonal_free_space(
      extracted.domain, FreeSpaceOffsetConfig{Nanometres{0}, 1'000'000U},
      cache ? &cache->obstacle_offsets : nullptr);
  if (!unioned.ok()) {
    preparation.diagnostic = offset.diagnostic + "; " + unioned.diagnostic;
    return preparation;
  }
  preparation.regions = unioned.regions;
  preparation.diagnostic = offset.diagnostic;
  preparation.complete = true;
  return preparation;
}

[[nodiscard]] bool refine_spline_run(Board& board, const Board& navigation_board, const NetId& net,
                                     const SplineRun& run, const std::string& base,
                                     std::set<std::string>& used_ids, DetailedRouteResult& result,
                                     const DetailedRouterConfig& config,
                                     std::stop_token stop_token,
                                     SplineNavigationPreparationCache* preparation_cache = nullptr,
                                     NavMeshScreeningContext* screening_context = nullptr) {
  MaterializationTimer refinement_timer(result.spline_route_refinement_microseconds);
  std::shared_ptr<SplineNavigationPreparationEntry> entry;
  if (preparation_cache != nullptr) {
    const SplineNavigationPreparationCache::Key key{net, run.layer, run.width};
    std::scoped_lock lock(preparation_cache->mutex);
    const auto found = preparation_cache->entries.find(key);
    if (found != preparation_cache->entries.end()) {
      entry = found->second;
      if (entry->remaining_consumers < entry->consumers)
        ++result.spline_navigation_preparation_hits;
      // Active consumers retain their shared pointer. The map need not retain geometry after
      // the final scheduled consumer acquires it, even while another consumer is still building.
      if (--entry->remaining_consumers == 0U)
        preparation_cache->entries.erase(found);
    }
  }
  const auto build_preparation = [&] {
    const auto started = std::chrono::steady_clock::now();
    auto prepared = std::make_shared<const SplineNavigationPreparation>(
        prepare_spline_navigation(navigation_board, net, run, config, preparation_cache));
    ++result.spline_navigation_preparation_builds;
    result.spline_navigation_preparation_microseconds +=
        static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now() - started).count());
    return prepared;
  };
  std::shared_ptr<const SplineNavigationPreparation> shared_preparation;
  if (entry) {
    std::scoped_lock entry_lock(entry->mutex);
    if (!entry->preparation) entry->preparation = build_preparation();
    shared_preparation = entry->preparation;
  } else {
    shared_preparation = build_preparation();
  }
  const auto& preparation = *shared_preparation;
  if (!preparation.complete) {
    ++result.spline_navmesh_failures;
    result.spline_navmesh_last_diagnostic = preparation.diagnostic;
    return false;
  }
  const auto route_radius = Nanometres{(run.width.value() + 1) / 2};
  const auto center_clearance =
      Nanometres{route_radius.value() + navigation_board.rules.copper_to_edge_clearance.value()};
  MaterializationTimer navigation_timer(result.spline_navigation_mesh_microseconds);
  const auto& navigation_regions = preparation.regions;
  const bool navigation_regions_are_offset = preparation.regions_are_offset;
  if (!navigation_regions_are_offset) {
    ++result.spline_navmesh_certificate_fallbacks;
    result.spline_navmesh_last_diagnostic = preparation.diagnostic;
  }
  std::optional<NavMeshRouteResult> navigation;
  std::shared_ptr<const PreparedNavMeshDomain> prepared_navigation;
  const OffsetFreeSpaceRegion* selected_region = nullptr;
  auto selected_planning_points = run.points;
  for (const auto& region : navigation_regions) {
    auto planning_run = run;
    // Only the clearance-offset domain proves the replacement copper envelope. The zero-offset
    // certificate fallback retains the original terminal run-up and relies on final DRC as before.
    if (navigation_regions_are_offset && !run.topology_locked) {
      planning_run.points = remove_legal_terminal_planning_stubs(
          navigation_board, net, run.layer, region, run.points,
          Nanometres{std::max<std::int64_t>(1, config.spline_navmesh_cell_pitch.value() / 2)});
    }
    const NavMeshConfig navmesh_config{
        config.spline_navmesh_cell_pitch,
        config.max_spline_navmesh_cells_per_run,
        config.max_spline_navmesh_search_expansions_per_run,
        planning_run.points,
        Nanometres{config.spline_control_corridor.value() +
                   config.spline_navmesh_cell_pitch.value()},
        static_cast<std::size_t>(
            std::max<std::int64_t>(1, (config.spline_control_corridor.value() +
                                       config.spline_navmesh_cell_pitch.value() - 1) /
                                          config.spline_navmesh_cell_pitch.value())),
        navigation_regions_are_offset ? Nanometres{0} : center_clearance,
        std::nullopt,
        true,
        2U,
        Nanometres{0}};
    auto prepared = screening_context != nullptr
        ? screening_context->prepare(region, navmesh_config)
        : std::make_shared<const PreparedNavMeshDomain>(
              prepare_navmesh_domain(region, navmesh_config));
    // Certified scaffolds use the NavMesh only as a local clearance guard.  Re-searching between
    // their endpoints could replace the selected route class, so retain the source corridor.
    const auto source_corridor = run.topology_locked
        ? make_source_polyline_corridor(region, planning_run, config.spline_navmesh_cell_pitch,
                                        config.spline_control_corridor,
                                        config.spline_minimum_bend_radius)
        : std::optional<NavMeshRouteResult>{};
    auto routed = source_corridor ? *source_corridor
        : route_prepared_navmesh(*prepared, run.points.front(), run.points.back(), navmesh_config);
    result.spline_navmesh_refined_parents += routed.adaptive_refined_parent_cells;
    result.spline_navmesh_refined_leaves += routed.adaptive_refined_leaf_cells;
    result.spline_navmesh_rejected_leaves += routed.adaptive_rejected_leaf_cells;
    result.spline_navmesh_gpu_wavefront_routes += routed.used_gpu_wavefront ? 1U : 0U;
    result.spline_navmesh_gpu_wavefront_cpu_fallbacks +=
        routed.gpu_wavefront_cpu_fallback ? 1U : 0U;
    result.spline_navmesh_last_diagnostic = routed.diagnostic;
    if (routed.ok()) {
      if (!run.topology_locked && planning_run.points != run.points) {
        // Merge, rather than replace, the ordinary broad NavMesh corridor. A certificate pitch
        // eight times the search pitch yields an initial half-width equal to the search pitch,
        // which gives a default-radius PH bend room to leave the source centreline; every cell is
        // nevertheless independently reduced until all sides and diagonals lie in offset space.
        augment_with_certified_planning_route(
            region, planning_run,
            Nanometres{config.spline_navmesh_cell_pitch.value() * static_cast<std::int64_t>(8)},
            config.spline_control_corridor, routed);
      }
      if (!run.topology_locked) {
        augment_terminal_planning_chord_corridor(
            region, run, planning_run.points, config.spline_navmesh_cell_pitch,
            config.spline_control_corridor, routed);
      }
      navigation = std::move(routed);
      prepared_navigation = std::move(prepared);
      selected_region = &region;
      selected_planning_points = std::move(planning_run.points);
      break;
    }
    if (const auto source_corridor = make_source_polyline_corridor(
            region, planning_run, config.spline_navmesh_cell_pitch,
            config.spline_control_corridor, config.spline_minimum_bend_radius)) {
      navigation = std::move(*source_corridor);
      prepared_navigation = std::move(prepared);
      selected_region = &region;
      selected_planning_points = std::move(planning_run.points);
      ++result.spline_navmesh_certificate_fallbacks;
      result.spline_navmesh_last_diagnostic =
          "coarse NavMesh failed; retained exact source-polyline corridor certificate";
      break;
    }
  }
  if (!navigation || !prepared_navigation || selected_region == nullptr) {
    ++result.spline_navmesh_failures;
    return false;
  }
  navigation_timer.finish();
  ++result.spline_navmesh_routes;
  result.spline_funnel_points += navigation->centerline.size();
  if (!navigation->topology_signature.valid) {
    ++result.spline_navmesh_failures;
    result.spline_navmesh_last_topology_diagnostic =
        "selected NavMesh route has no valid topology signature";
    return false;
  }
  ++result.spline_navmesh_topology_signatures;
  result.spline_navmesh_topology_gates += navigation->topology_signature.gates.size();
  auto spline_points = run.topology_locked
      ? run.points
      : string_pull_detailed_spline_seed(
            *navigation,
            Nanometres{std::max<std::int64_t>(1, config.spline_navmesh_cell_pitch.value() / 4)});
  // String pulling can synthesize a different sub-cell portal point at the pad, so matching only
  // the source stub identity is insufficient. Reapply the exact rule to the actual interpolation
  // sequence and extend the selected guard only around any newly accepted terminal chord.
  const SplineRun pulled_run{run.layer, run.width, spline_points, {}};
  auto corrected_spline_points = run.topology_locked
      ? spline_points
      : remove_legal_terminal_planning_stubs(
            navigation_board, net, run.layer, *selected_region, spline_points,
            Nanometres{std::max<std::int64_t>(1, config.spline_navmesh_cell_pitch.value() / 2)});
  if (corrected_spline_points != spline_points) {
    augment_terminal_planning_chord_corridor(
        *selected_region, pulled_run, corrected_spline_points, config.spline_navmesh_cell_pitch,
        config.spline_control_corridor, *navigation);
    spline_points = std::move(corrected_spline_points);
  }
  if (spline_points.size() == 2U) {
    spline_points.insert(
        spline_points.begin() + 1,
        Point{Nanometres{spline_points.front().x.value() +
                         (spline_points.back().x.value() - spline_points.front().x.value()) / 2},
              Nanometres{spline_points.front().y.value() +
                         (spline_points.back().y.value() - spline_points.front().y.value()) / 2}});
  }
  auto spline = make_quintic_ph_route(spline_points);
  if (!spline) {
    return false;
  }
  std::vector<Polygon> topology_corridor;
  topology_corridor.reserve(navigation->topology_guard_cells.size());
  for (const auto cell_id : navigation->topology_guard_cells) {
    if (cell_id >= navigation->cells.size())
      return false;
    const auto& corners = navigation->cells[cell_id].corners;
    topology_corridor.push_back(Polygon{{corners[0], corners[1], corners[2], corners[3]}});
  }
  if (topology_corridor.empty() || !set_quintic_ph_corridor(*spline, topology_corridor)) {
    return false;
  }
  const auto quantize = [](SplineVector point) {
    return Point{Nanometres{static_cast<std::int64_t>(std::llround(point.x))},
                 Nanometres{static_cast<std::int64_t>(std::llround(point.y))}};
  };
  const auto clearance_evaluator = [&](SplineVector position) {
    const auto point = quantize(position);
    const auto query = query_prepared_navmesh_point(*prepared_navigation, point);
    const auto distance = query.boundary_distance_nm;
    const auto probe = std::max<std::int64_t>(1, config.spline_navmesh_cell_pitch.value() / 100);
    const auto dx = query_prepared_navmesh_point(
                        *prepared_navigation, Point{Nanometres{point.x.value() + probe}, point.y})
                        .boundary_distance_nm -
                    query_prepared_navmesh_point(
                        *prepared_navigation, Point{Nanometres{point.x.value() - probe}, point.y})
                        .boundary_distance_nm;
    const auto dy = query_prepared_navmesh_point(
                        *prepared_navigation, Point{point.x, Nanometres{point.y.value() + probe}})
                        .boundary_distance_nm -
                    query_prepared_navmesh_point(
                        *prepared_navigation, Point{point.x, Nanometres{point.y.value() - probe}})
                        .boundary_distance_nm;
    const auto length = std::hypot(dx, dy);
    return SplineClearanceProbe{
        query.inside ? distance : -distance,
        length > 0.0L ? SplineVector{dx / length, dy / length} : SplineVector{}, query.inside};
  };
  // Advisory field samples are independent and share this immutable offset domain.  Keep
  // scalar CPU queries for shape, topology and continuous-copper certificates.
  std::vector<RoutingComputeSegment> field_boundaries;
  const auto add_contour = [&](const Polygon& polygon) {
    for (std::size_t edge = 0U; edge < polygon.vertices.size(); ++edge) {
      const auto a = polygon.vertices[edge];
      const auto b = polygon.vertices[(edge + 1U) % polygon.vertices.size()];
      field_boundaries.push_back({a.x.value(), a.y.value(), b.x.value(), b.y.value()});
    }
  };
  if (config.gpu_materialization_batches && config.prefer_gpu_spline_trajectory) {
    add_contour(selected_region->area);
    for (const auto& hole : selected_region->holes) add_contour(hole);
  }
  const SplineClearanceBatchEvaluator clearance_batch_evaluator = field_boundaries.empty()
      ? SplineClearanceBatchEvaluator{}
      : SplineClearanceBatchEvaluator{[&](const std::vector<SplineVector>& points) {
          std::vector<std::pair<std::int64_t, std::int64_t>> queries;
          queries.reserve(points.size());
          for (const auto point : points) {
            const auto exact = quantize(point);
            queries.emplace_back(exact.x.value(), exact.y.value());
          }
          const auto assessed = probe_routing_domain_points(queries, field_boundaries, true);
          result.spline_clearance_batch_queries += queries.size();
          result.spline_clearance_gpu_dispatches += assessed.dispatches;
          result.spline_clearance_resident_uploads += assessed.resident_geometry_uploads;
          result.spline_clearance_resident_reuses += assessed.resident_geometry_reuses;
          result.spline_clearance_cpu_fallbacks += assessed.cpu_fallback ? 1U : 0U;
          result.spline_clearance_batch_microseconds += assessed.elapsed_us;
          std::vector<SplineClearanceProbe> probes;
          if (!assessed.ok() || assessed.signed_distances_nm.size() != queries.size()) return probes;
          probes.reserve(queries.size());
          for (const auto distance : assessed.signed_distances_nm)
            probes.push_back({distance, {}, distance >= 0.0L});
          return probes;
        }};
  // Exact-candidate workers may invoke certificates concurrently. Accumulate locally and fold
  // only after every synchronous optimizer call has joined its workers.
  struct EnvelopeCounters {
    DetailedRouteResult& result;
    std::atomic<std::uint64_t> spline_envelope_queries{0U};
    std::atomic<std::uint64_t> spline_envelope_proven_clear{0U};
    std::atomic<std::uint64_t> spline_envelope_gpu_dispatches{0U};
    std::atomic<std::uint64_t> spline_envelope_cpu_fallbacks{0U};
    std::atomic<std::uint64_t> spline_envelope_resident_uploads{0U};
    std::atomic<std::uint64_t> spline_envelope_resident_reuses{0U};
    std::atomic<std::uint64_t> spline_envelope_microseconds{0U};
    ~EnvelopeCounters() {
      result.spline_envelope_queries += spline_envelope_queries.load(std::memory_order_relaxed);
      result.spline_envelope_proven_clear += spline_envelope_proven_clear.load(std::memory_order_relaxed);
      result.spline_envelope_gpu_dispatches += spline_envelope_gpu_dispatches.load(std::memory_order_relaxed);
      result.spline_envelope_cpu_fallbacks += spline_envelope_cpu_fallbacks.load(std::memory_order_relaxed);
      result.spline_envelope_resident_uploads += spline_envelope_resident_uploads.load(std::memory_order_relaxed);
      result.spline_envelope_resident_reuses += spline_envelope_resident_reuses.load(std::memory_order_relaxed);
      result.spline_envelope_microseconds += spline_envelope_microseconds.load(std::memory_order_relaxed);
    }
  } envelope_counters{result};
  const SplineEnvelopeScreenEvaluator envelope_screen_evaluator = field_boundaries.empty()
      ? SplineEnvelopeScreenEvaluator{}
      : SplineEnvelopeScreenEvaluator{[&](const std::vector<SplineEnvelopeChord>& capsules) {
          SplineEnvelopeScreenResult screened;
          const auto started = std::chrono::steady_clock::now();
          screened.definitely_clear.resize(capsules.size(), 0U);
          std::vector<RoutingComputeSegment> chords;
          chords.reserve(capsules.size());
          for (const auto& capsule : capsules) {
            const auto first = quantize(capsule.first), second = quantize(capsule.second);
            chords.push_back({first.x.value(), first.y.value(), second.x.value(), second.y.value(),
                              0, 0U, 0U});
          }
          const auto assessed = classify_routing_chords(chords, field_boundaries, 0, 2048, true);
          screened.used_gpu = assessed.used_gpu;
          screened.cpu_fallback = assessed.cpu_fallback;
          screened.dispatches = assessed.gpu_batch_count;
          envelope_counters.spline_envelope_queries += capsules.size();
          envelope_counters.spline_envelope_gpu_dispatches += assessed.gpu_batch_count;
          envelope_counters.spline_envelope_cpu_fallbacks += assessed.cpu_fallback ? 1U : 0U;
          envelope_counters.spline_envelope_resident_uploads += assessed.resident_boundary_uploads;
          envelope_counters.spline_envelope_resident_reuses += assessed.resident_boundary_reuses;
          if (assessed.ok() && assessed.chords.size() == capsules.size()) {
            for (std::size_t index = 0U; index < capsules.size(); ++index) {
              const auto& proof = assessed.chords[index];
              // These boundaries already include the copper envelope. Two additional nm cover
              // endpoint rounding; the classifier has already subtracted its FP32 guard.
              if (proof.classification != RoutingComputeChordClass::definitely_clear ||
                  capsules[index].radius.value() < 0 ||
                  static_cast<long double>(proof.conservative_clearance_nm) <
                      static_cast<long double>(capsules[index].radius.value()) + 2.0L)
                continue;
              if (query_prepared_navmesh_point(*prepared_navigation,
                                               quantize(capsules[index].first)).inside &&
                  query_prepared_navmesh_point(*prepared_navigation,
                                               quantize(capsules[index].second)).inside) {
                screened.definitely_clear[index] = 1U;
                ++envelope_counters.spline_envelope_proven_clear;
              }
            }
          }
          screened.elapsed_us = static_cast<std::uint64_t>(
              std::chrono::duration_cast<std::chrono::microseconds>(
                  std::chrono::steady_clock::now() - started).count());
          envelope_counters.spline_envelope_microseconds += screened.elapsed_us;
          return screened;
        }};
  // The exact offset free-space query remains the physical authority. The selected guard-cell
  // union is additionally retained as a continuous topology corridor so an initially overshooting
  // PH branch can be pulled back into its route class instead of being rejected before the
  // rubber-band optimizer gets a chance to move its interpolation nodes.
  const auto topology_preserved = [&](const std::vector<SplineSample>& samples) {
    SplineContinuousEnvelopeConfig corridor_certificate_config;
    corridor_certificate_config.corridor_certifies_clearance = true;
    const auto corridor_certificate = certify_quintic_ph_continuous_envelope(
        *spline, Nanometres{0}, clearance_evaluator, corridor_certificate_config);
    result.spline_continuous_hulls_checked += corridor_certificate.hulls_checked;
    if (!corridor_certificate.legal) {
      ++result.spline_navmesh_topology_rejections;
      result.spline_navmesh_last_topology_diagnostic =
          "continuous PH hull left the selected NavMesh guard union";
      return false;
    }
    std::vector<Point> polyline;
    polyline.reserve(samples.size());
    for (const auto& sample : samples) {
      const auto point =
          Point{Nanometres{static_cast<std::int64_t>(std::llround(sample.position.x))},
                Nanometres{static_cast<std::int64_t>(std::llround(sample.position.y))}};
      if (polyline.empty() || polyline.back() != point) {
        polyline.push_back(point);
      }
    }
    const auto preserved = detailed_spline_preserves_route_class(
        *navigation, *selected_region, polyline,
        Nanometres{std::max<std::int64_t>(1, config.spline_navmesh_cell_pitch.value() /
                                                 static_cast<std::int64_t>(4))},
        true, result.spline_navmesh_last_topology_diagnostic);
    if (!preserved) {
      ++result.spline_navmesh_topology_rejections;
    }
    return preserved;
  };
  // A funnel polyline is a topological seed, not the finished curve. PH interpolation through a
  // polygonal corner can overshoot or acquire a bad branch before coordinate descent begins. Build
  // a small deterministic portfolio containing the raw funnel, several corridor-checked
  // rubber-band relaxations, and guarded corner variants. Rank the starts by the same exact shape
  // and continuous convex-hull certificates used after optimization, not by sampled aesthetics.
  auto shape_config = SplineShapeCertificationConfig{};
  shape_config.minimum_bend_radius = config.spline_minimum_bend_radius;
  SplineContinuousEnvelopeConfig seed_envelope_config;
  seed_envelope_config.corridor_certifies_clearance = true;
  const auto route_class_preserved = [&](const QuinticPHRoute& candidate) {
    const auto samples = adaptive_tessellate_quintic_ph(
        candidate, config.spline_tessellation_tolerance, config.spline_max_tessellation_depth,
        config.max_spline_samples_per_run);
    if (samples.size() < 2U || samples.size() >= config.max_spline_samples_per_run)
      return false;
    std::vector<Point> polyline;
    polyline.reserve(samples.size());
    for (const auto& sample : samples) {
      const auto point = quantize(sample.position);
      if (polyline.empty() || polyline.back() != point)
        polyline.push_back(point);
    }
    std::string diagnostic;
    return detailed_spline_preserves_route_class(
        *navigation, *selected_region, polyline,
        Nanometres{std::max<std::int64_t>(1, config.spline_navmesh_cell_pitch.value() /
                                                 static_cast<std::int64_t>(4))},
        true, diagnostic);
  };
  // Topology is not repairable by the local trajectory optimizer and therefore ranks first.
  // Among topology-valid starts, minimize the complete hard-defect count and prefer a
  // shape-regular branch over one that already contains a loop/inflection. Clearance-envelope
  // defects remain eligible because elastic optimization can move them within the selected
  // corridor.
  using SeedScore = std::tuple<std::size_t, std::size_t, std::size_t, std::size_t, long double>;
  auto best_seed = *spline;
  SeedScore best_seed_score{
      std::numeric_limits<std::size_t>::max(), std::numeric_limits<std::size_t>::max(),
      std::numeric_limits<std::size_t>::max(), std::numeric_limits<std::size_t>::max(),
      std::numeric_limits<long double>::infinity()};
  std::optional<SeedScore> raw_seed_score;
  auto consider_seed = [&](std::optional<QuinticPHRoute> candidate, bool raw) {
    if (!candidate || !set_quintic_ph_corridor(*candidate, topology_corridor))
      return;
    ++result.spline_seed_candidates;
    const auto shape = certify_quintic_ph_shape(*candidate, shape_config);
    const auto envelope = certify_quintic_ph_continuous_envelope(
        *candidate, Nanometres{0}, clearance_evaluator, seed_envelope_config);
    const auto topology_legal = route_class_preserved(*candidate);
    long double length = 0.0L;
    for (const auto& segment : candidate->segments)
      length += quintic_ph_arc_length(segment);
    const SeedScore score{topology_legal ? 0U : 1U, shape.violations() + envelope.violations(),
                          shape.violations(), envelope.violations(), length};
    if (topology_legal && shape.legal && envelope.legal)
      ++result.spline_seed_exact_legal;
    if (raw)
      raw_seed_score = score;
    if (score < best_seed_score) {
      best_seed_score = score;
      best_seed = std::move(*candidate);
    }
  };
  const auto initial_legal_seeds = result.spline_seed_exact_legal;
  consider_seed(spline, true);
  const auto seed_probe_spacing = Nanometres{std::max<std::int64_t>(
      1, config.spline_navmesh_cell_pitch.value() / static_cast<std::int64_t>(4))};
  for (const auto passes : {1U, 2U, 4U, 8U}) {
    auto relaxed =
        relax_detailed_spline_seed(*navigation, spline_points, seed_probe_spacing, passes);
    if (relaxed != spline_points)
      consider_seed(make_quintic_ph_route(relaxed), false);
  }
  // The NavMesh funnel is preferred for topology discovery, but it is not the only legal
  // interpolation parameterization. Retain source-copper-derived starts as bounded fallback
  // columns. This is especially important at pad entries where a very short final run-up beside a
  // much longer corridor span can condition the funnel's Hermite derivatives badly even though a
  // regular corner-rounded PH chain exists in the same route class.
  consider_seed(make_quintic_ph_route(selected_planning_points), false);
  for (const auto factor : {1, 2, 4}) {
    const auto guard = Nanometres{
        std::min(config.spline_control_corridor.value(),
                 config.spline_navmesh_cell_pitch.value() * static_cast<std::int64_t>(factor))};
    consider_seed(make_topology_tight_detailed_spline(spline_points, guard), false);
    consider_seed(make_corner_rounded_quintic_ph_route(selected_planning_points, guard), false);
  }
  for (const auto divisor : {2, 3, 4, 6, 8, 12, 16, 32}) {
    consider_seed(make_topology_tight_detailed_spline(
                      spline_points, Nanometres{std::max<std::int64_t>(
                                         1, config.spline_navmesh_cell_pitch.value() / divisor)}),
                  false);
    consider_seed(make_corner_rounded_quintic_ph_route(
                      selected_planning_points,
                      Nanometres{std::max<std::int64_t>(
                          1, config.spline_navmesh_cell_pitch.value() / divisor)}),
                  false);
  }
  if (result.spline_seed_exact_legal == initial_legal_seeds) {
    // A corner's shoulder bisectors can bow an otherwise straight intervening run
    // across a nearby boundary. Try exact Hermite run directions before asking
    // position-only optimization to undo that seed-construction defect.
    const auto candidates_before = result.spline_seed_candidates;
    for (const auto factor : {1, 2, 4, 8}) {
      const auto guard = Nanometres{std::min(
          config.spline_control_corridor.value(),
          config.spline_navmesh_cell_pitch.value() * static_cast<std::int64_t>(factor))};
      consider_seed(make_run_aligned_quintic_ph_route(selected_planning_points, guard), false);
      consider_seed(make_run_aligned_quintic_ph_route(spline_points, guard), false);
    }
    result.spline_run_aligned_seed_candidates += result.spline_seed_candidates - candidates_before;
    result.spline_run_aligned_seed_exact_legal += result.spline_seed_exact_legal - initial_legal_seeds;
  }
  if (raw_seed_score && best_seed_score < *raw_seed_score)
    ++result.spline_seed_portfolio_improvements;
  *spline = std::move(best_seed);
  ++result.attempted_quintic_ph_runs;
  SplineTrajectoryOptimizationConfig trajectory_config;
  // Every retained NavMesh cell has already exact-checked all four sides and both diagonals at
  // center_clearance. The PH envelope checker additionally proves each subdivided Bezier hull is
  // contained by one of those certified convex cells. Asking the point-distance oracle for the
  // same copper radius/clearance again subtracts that envelope twice and rejects legal curves at
  // cell or terminal boundaries. Here it proves only that the continuous hull stays in represented
  // free space; the cell certificate supplies the physical envelope, and quantized whole-board
  // copper DRC remains authoritative.
  trajectory_config.minimum_clearance = Nanometres{0};
  trajectory_config.minimum_bend_radius = config.spline_minimum_bend_radius;
  trajectory_config.control_step = config.spline_shove_step;
  trajectory_config.gradient_probe_step = Nanometres{
      std::max<std::int64_t>(1, config.spline_shove_step.value() / static_cast<std::int64_t>(5))};
  trajectory_config.maximum_corridor_displacement = config.spline_control_corridor;
  trajectory_config.samples = config.spline_trajectory_samples;
  trajectory_config.maximum_iterations = config.max_spline_trajectory_iterations;
  trajectory_config.maximum_candidate_checks = config.max_spline_trajectory_candidate_checks;
  trajectory_config.continuous_envelope.corridor_certifies_clearance = true;
  trajectory_config.prefer_gpu_preconditioner = config.prefer_gpu_spline_trajectory;
  trajectory_config.gpu_precondition_iterations = config.spline_gpu_precondition_iterations;
  trajectory_config.prefer_gpu_polishing = config.prefer_gpu_spline_trajectory;
  trajectory_config.gpu_polish_shortlist = config.spline_gpu_polish_shortlist;
  trajectory_config.exact_candidate_workers = config.spline_exact_candidate_workers;
  trajectory_config.preferred_start_tangent = SplineVector{
      static_cast<long double>(spline_points[1U].x.value() - spline_points[0U].x.value()),
      static_cast<long double>(spline_points[1U].y.value() - spline_points[0U].y.value())};
  trajectory_config.preferred_end_tangent =
      SplineVector{static_cast<long double>(spline_points.back().x.value() -
                                            spline_points[spline_points.size() - 2U].x.value()),
                   static_cast<long double>(spline_points.back().y.value() -
                                            spline_points[spline_points.size() - 2U].y.value())};
  const auto route_constraint = route_class_preserved;
  // Post-polish may remove or replace interpolation stations.  It is otherwise an intentionally
  // cheap G1 operation, but a station layout that cannot be promoted to the requested native G2
  // realization is not useful here.  Test that one bounded post-polish result before accepting
  // it; keep the expensive curvature solve out of the optimizer's ordinary candidate loop.
  if (config.prefer_g2_ph_splines) {
    trajectory_config.postpolish_route_constraint = [&](const QuinticPHRoute& candidate) {
      if (candidate.segments.size() <= 1U || candidate.curvature_continuous)
        return true;
      auto fair = candidate;
      if (!enforce_quintic_ph_continuity(
              fair, PHJoinContinuity::curvature, 1.0e-10L,
              std::min<std::size_t>(24U, config.max_spline_trajectory_iterations), stop_token))
        return false;
      auto fair_shape = trajectory_config.shape;
      fair_shape.minimum_bend_radius = trajectory_config.minimum_bend_radius;
      SplineContinuousEnvelopeConfig fair_envelope_config;
      fair_envelope_config.corridor_certifies_clearance = true;
      return certify_quintic_ph_continuous_envelope(
                 fair, Nanometres{0}, clearance_evaluator, fair_envelope_config)
                 .legal &&
             certify_quintic_ph_shape(fair, fair_shape).legal && route_constraint(fair);
    };
  }
  ++result.spline_trajectory_attempts;
  const auto spline_before_optimization = *spline;
  const auto trajectory_started = std::chrono::steady_clock::now();
  const auto optimized = optimize_quintic_ph_trajectory(*spline, trajectory_config,
                                                        clearance_evaluator, route_constraint, clearance_batch_evaluator,
                                                        envelope_screen_evaluator);
  result.spline_trajectory_microseconds +=
      static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::microseconds>(
          std::chrono::steady_clock::now() - trajectory_started).count());
  result.spline_trajectory_control_moves += optimized.accepted_control_moves;
  result.spline_trajectory_candidate_checks += optimized.candidate_checks;
  result.spline_span_certificate_cache_hits += optimized.span_certificate_cache_hits;
  result.spline_span_certificate_cache_misses += optimized.span_certificate_cache_misses;
  result.spline_equivalent_candidates_skipped += optimized.equivalent_candidates_skipped;
  result.spline_non_improving_candidates_skipped += optimized.non_improving_candidates_skipped;
  result.spline_paired_obstacle_support_recoveries += optimized.paired_obstacle_support_recoveries;
  result.spline_paired_obstacle_supports_inserted += optimized.paired_obstacle_supports_inserted;
  result.spline_paired_obstacle_support_attempts += optimized.paired_obstacle_support_attempts;
  result.spline_multistart_attempts += optimized.multistart_attempts;
  result.spline_multistart_recoveries += optimized.multistart_recoveries;
  result.spline_gpu_precondition_attempts += optimized.gpu_precondition_attempted ? 1U : 0U;
  result.spline_gpu_precondition_dispatches += optimized.gpu_precondition_used_gpu ? 1U : 0U;
  result.spline_gpu_precondition_cpu_fallbacks += optimized.gpu_precondition_cpu_fallback ? 1U : 0U;
  result.spline_gpu_precondition_accepted += optimized.gpu_precondition_accepted ? 1U : 0U;
  result.spline_gpu_precondition_legal += optimized.gpu_precondition_legal ? 1U : 0U;
  result.spline_gpu_precondition_solve_us += optimized.gpu_precondition_solve_us;
  result.spline_gpu_polish_microseconds += optimized.gpu_polish_solve_us;
  result.spline_gpu_polish_batches += optimized.gpu_polish_batches;
  result.spline_gpu_polish_dispatches += optimized.gpu_polish_dispatches;
  result.spline_gpu_polish_cpu_fallbacks += optimized.gpu_polish_cpu_fallbacks;
  result.spline_gpu_polish_candidates += optimized.gpu_polish_candidates;
  result.spline_gpu_polish_exact_shortlist_checks += optimized.gpu_polish_exact_shortlist_checks;
  result.spline_parallel_exact_waves += optimized.parallel_exact_waves;
  result.spline_parallel_exact_checks += optimized.parallel_exact_checks;
  result.spline_trajectory_initial_violations += optimized.initial_violations;
  result.spline_trajectory_final_violations += optimized.final_violations;
  result.spline_trajectory_initial_shape_violations += optimized.initial_shape_violations;
  result.spline_trajectory_final_shape_violations += optimized.final_shape_violations;
  result.spline_trajectory_final_self_intersections += optimized.final_self_intersections;
  result.spline_trajectory_final_cusps += optimized.final_cusps;
  result.spline_trajectory_final_control_station_reversals +=
      optimized.final_control_station_reversals;
  result.spline_trajectory_final_station_reversals += optimized.final_station_reversals;
  result.spline_trajectory_final_excess_turn_violations += optimized.final_excess_turn_violations;
  result.spline_trajectory_final_unintended_inflections += optimized.final_unintended_inflections;
  result.spline_trajectory_final_bend_radius_violations += optimized.final_bend_radius_violations;
  result.spline_trajectory_initial_continuous_violations += optimized.initial_continuous_violations;
  result.spline_trajectory_final_continuous_violations += optimized.final_continuous_violations;
  result.spline_continuous_hulls_checked += optimized.continuous_hulls_checked;
  result.spline_continuous_envelope_rejections +=
      optimized.final_continuous_violations > 0U ? 1U : 0U;
  result.spline_continuous_budget_exhaustions += optimized.continuous_budget_exhausted ? 1U : 0U;
  if (!optimized.accepted) {
    result.spline_last_materialization_diagnostic =
        "trajectory rejected: sampled_clearance=" +
        std::to_string(optimized.final_sampled_clearance_violations) +
        ";continuous_corridor=" +
        std::to_string(optimized.final_continuous_corridor_violations) +
        ";continuous_clearance=" +
        std::to_string(optimized.final_continuous_clearance_violations);
    return false;
  }
  // The optimizer works in the cheaper tangent-continuous PH space. Promote only its selected
  // result to a fair curvature-continuous chain, once, immediately before native arc fitting.
  // This keeps G2 construction out of the candidate inner loop. A promotion must retain the same
  // exact guard containment, shape and homotopy; otherwise the legal G1 result remains intact.
  std::optional<QuinticPHRoute> unpromoted_spline;
  if (config.prefer_g2_ph_splines && spline->segments.size() > 1U &&
      !spline->curvature_continuous &&
      config.max_spline_candidate_checks - std::min(config.max_spline_candidate_checks,
          result.attempted_circular_arc_candidates) > 1U) {
    ++result.spline_g2_promotion_attempts;
    const auto promotion_started = std::chrono::steady_clock::now();
    auto fair = *spline;
    auto promoted = enforce_quintic_ph_continuity(
        fair, PHJoinContinuity::curvature, 1.0e-10L,
        std::min<std::size_t>(24U, config.max_spline_trajectory_iterations), stop_token);
    // The G1 optimizer can improve length by perturbing a derivative layout beyond the bounded
    // scalar-G2 promoter's basin.  Keep its checkpoint and make one upstream retry rather than
    // relaxing curvature tolerance or discarding the route's smooth realization altogether.
    // Both alternatives preserve interpolation stations; the same hard guards below decide
    // which one may reach native copper fitting.
    if (!promoted && !stop_token.stop_requested()) {
      fair = spline_before_optimization;
      promoted = enforce_quintic_ph_continuity(
          fair, PHJoinContinuity::curvature, 1.0e-10L,
          std::min<std::size_t>(24U, config.max_spline_trajectory_iterations), stop_token);
    }
    auto fair_shape = trajectory_config.shape;
    fair_shape.minimum_bend_radius = trajectory_config.minimum_bend_radius;
    SplineContinuousEnvelopeConfig fair_envelope_config;
    fair_envelope_config.corridor_certifies_clearance = true;
    const auto fair_envelope =
        promoted ? certify_quintic_ph_continuous_envelope(
                       fair, Nanometres{0}, clearance_evaluator, fair_envelope_config)
                 : SplineContinuousEnvelopeCertificate{};
    if (promoted && fair_envelope.legal && certify_quintic_ph_shape(fair, fair_shape).legal &&
        route_class_preserved(fair)) {
      unpromoted_spline = *spline;
      *spline = std::move(fair);
    } else {
      ++result.spline_g2_fallbacks;
    }
    result.spline_g2_promotion_microseconds += static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now() - promotion_started).count());
  }
  const auto optimized_topology_samples = adaptive_tessellate_quintic_ph(
      *spline, config.spline_tessellation_tolerance, config.spline_max_tessellation_depth,
      config.max_spline_samples_per_run);
  if (!topology_preserved(optimized_topology_samples)) {
    // Optimization may not change the selected homotopy. Retain the pre-optimization seed only if
    // it independently passes the same continuous route-class check; a shape-valid PH branch is
    // not automatically topology-valid merely because it came from funnel interpolation nodes.
    *spline = unpromoted_spline ? *unpromoted_spline : spline_before_optimization;
    if (unpromoted_spline) {
      unpromoted_spline.reset();
      ++result.spline_g2_fallbacks;
    }
    auto fallback_shape = trajectory_config.shape;
    fallback_shape.minimum_bend_radius = trajectory_config.minimum_bend_radius;
    const auto fallback_samples = adaptive_tessellate_quintic_ph(
        *spline, config.spline_tessellation_tolerance, config.spline_max_tessellation_depth,
        config.max_spline_samples_per_run);
    if (!certify_quintic_ph_shape(*spline, fallback_shape).legal ||
        !topology_preserved(fallback_samples)) {
      return false;
    }
  } else {
    ++result.spline_trajectory_accepted;
  }
  bool promotion_pending = unpromoted_spline.has_value();
  const auto materialize_selected_spline = [&]() -> bool {
    Board corridor_board = board;
    CopperTransactionValidator transaction_validator(board, config.defer_saved_zone_fill_clearance_to_refill);
    std::size_t pending_pns_shoves = 0U;
    std::size_t pending_pns_tracks = 0U;
    std::size_t pending_pns_vias = 0U;
    std::size_t pending_pns_doglegs = 0U;
    for (std::size_t iteration = 0U; iteration <= config.max_spline_shove_iterations; ++iteration) {
      routing_kernel_checkpoint();
      if (stop_token.stop_requested()) {
        return false;
      }
      if (result.attempted_circular_arc_candidates >= config.max_spline_candidate_checks) {
        result.spline_candidate_budget_exhausted = true;
        result.circular_arc_candidate_budget_exhausted = true;
        return false;
      }
      const auto samples = adaptive_tessellate_quintic_ph(
          *spline, config.spline_tessellation_tolerance, config.spline_max_tessellation_depth,
          config.max_spline_samples_per_run);
      result.adaptive_spline_tessellation_samples += samples.size();
      if (samples.size() < 2U || samples.size() >= config.max_spline_samples_per_run) {
        return false;
      }
      if (!topology_preserved(samples)) {
        return false;
      }
      const NativeSplineFitConfig fit_policy{
          config.spline_tessellation_tolerance, config.spline_arc_fit_tolerance,
          config.spline_max_tessellation_depth, config.max_spline_samples_per_run,
          config.max_spline_arc_samples,        config.max_spline_fitted_pieces_per_run,
          config.spline_max_join_angle_radians};
      const auto pieces = fit_quintic_ph_to_cubic_copper(*spline, fit_policy, stop_token);
      // A rubber-band solution may legitimately pull a staircase completely straight. That is a
      // successful whole-run PH realization even though its exact KiCad decomposition needs no arc.
      if (pieces.empty()) {
        return false;
      }
      ++result.attempted_circular_arc_candidates;
      auto probe = materialize_spline_probe(corridor_board, run, pieces, base, iteration, used_ids);
      if (!probe) {
        return false;
      }
      assign_spline_net(*probe, net);
      const auto transaction_report = transaction_validator.validate(probe->board);
      record_transaction_metrics(result, transaction_report);
      const auto& validation = transaction_report.structural;
      const auto& copper = transaction_report.copper;
      const auto* collision = first_spline_collision(copper, probe->generated_ids);
      if (validation.ok() && collision == nullptr) {
        const std::set<std::string> original_ids(run.original_track_ids.begin(),
                                                 run.original_track_ids.end());
        std::erase_if(probe->board.tracks,
                      [&](const auto& track) { return original_ids.contains(track.id.value()); });
        Board final_checked = probe->board;
        if (config.defer_saved_zone_fill_clearance_to_refill) {
          for (auto& zone : final_checked.zones) {
            zone.fills.clear();
          }
        }
        const auto final_report = transaction_validator.validate(final_checked);
        record_transaction_metrics(result, final_report);
        const auto& final_validation = final_report.structural;
        const auto& final_copper = final_report.copper;
        const auto& final_connectivity = final_report.after_connectivity;
        const auto& baseline_connectivity = final_report.before_connectivity;
        const auto* final_collision = first_spline_collision(final_copper, probe->generated_ids);
        const auto connectivity_regressed =
            final_connectivity.open_connections > baseline_connectivity.open_connections ||
            final_connectivity.unrouted_nets > baseline_connectivity.unrouted_nets;
        if (!final_validation.ok() || final_collision != nullptr || connectivity_regressed ||
            !final_report.preserves_pad_components) {
          if (!final_validation.ok()) {
            ++result.spline_final_validation_rejections;
            result.spline_last_materialization_diagnostic =
                "quantized PH replacement failed canonical board validation";
          } else if (final_collision != nullptr) {
            ++result.spline_final_copper_rejections;
            result.spline_last_materialization_diagnostic =
                final_collision->code + ": quantized PH replacement failed copper DRC";
          } else {
            ++result.spline_final_connectivity_rejections;
            result.spline_last_materialization_diagnostic =
                "quantized PH replacement regressed exact connectivity";
          }
          ++result.rejected_circular_arc_candidates;
          return false;
        }
        // Commit the exact-checked replacement transaction, not the probe board.  The probe still
        // contains the superseded source segments so it can be validated as an additive candidate;
        // retaining it here would leave duplicate same-net bends for later local curve passes.
        board = std::move(final_checked);
        result.added_tracks -= std::min(result.added_tracks, run.original_track_ids.size());
        result.added_tracks += probe->tracks;
        result.added_track_arcs += probe->arcs;
        result.fitted_circular_arcs += probe->arcs;
        result.added_track_beziers += probe->beziers;
        result.fitted_cubic_beziers += probe->beziers;
        result.spline_fitted_track_segments += probe->tracks;
        ++result.fitted_quintic_ph_runs;
        result.spline_last_materialization_diagnostic = "quantized PH replacement committed";
        result.accepted_pns_shoves += pending_pns_shoves;
        result.pns_moved_tracks += pending_pns_tracks;
        result.pns_moved_vias += pending_pns_vias;
        result.pns_added_dogleg_tracks += pending_pns_doglegs;
        return true;
      }
      ++result.rejected_circular_arc_candidates;
      ++result.spline_collision_envelopes;
      if (promotion_pending || collision == nullptr) {
        return false;
      }
      const auto generated =
          std::find_if(collision->object_ids.begin(), collision->object_ids.end(),
                       [&](const auto& id) { return probe->parameter_by_id.contains(id); });
      if (generated == collision->object_ids.end()) {
        return false;
      }
      const auto parameter = probe->parameter_by_id.at(*generated);
      const auto position = evaluate_quintic_ph(*spline, parameter);
      const auto before = evaluate_quintic_ph(*spline, std::max(0.0L, parameter - 1.0e-4L));
      const auto after = evaluate_quintic_ph(*spline, std::min(1.0L, parameter + 1.0e-4L));
      SplineVector normal{-(after.y - before.y), after.x - before.x};
      std::vector<std::string> obstacles;
      for (const auto& id : collision->object_ids) {
        if (!probe->parameter_by_id.contains(id)) {
          obstacles.push_back(id);
        }
      }
      const auto collision_point =
          Point{Nanometres{static_cast<std::int64_t>(std::llround(position.x))},
                Nanometres{static_cast<std::int64_t>(std::llround(position.y))}};
      for (const auto& id : obstacles) {
        if (const auto reference = obstacle_reference_point(corridor_board, id, collision_point)) {
          const auto away_x = position.x - static_cast<long double>(reference->x.value());
          const auto away_y = position.y - static_cast<long double>(reference->y.value());
          if (std::hypot(away_x, away_y) > 1.0L) {
            normal = {away_x, away_y};
            break;
          }
        }
      }
      const auto deficit =
          std::max(config.spline_shove_step.value(),
                   static_cast<std::int64_t>(std::ceil(std::max(
                       0.0L, static_cast<long double>(collision->required_clearance.value()) -
                                 collision->actual_distance_nm + 2.0L))));
      ++result.pns_shove_attempts;
      const PnsShoveConfig pns_config{config.max_pns_propagation_depth,
                                      config.max_pns_candidate_checks, config.spline_shove_step,
                                      config.maximum_pns_displacement};
      const PnsShoveRequest request{probe->generated_ids, obstacles, run.original_track_ids,
                                    collision_point,      -normal.x, -normal.y,
                                    Nanometres{deficit},  {}};
      auto pns = pns_push_and_shove(probe->board, request, pns_config);
      result.pns_shove_candidates += pns.candidate_checks;
      result.pns_candidate_budget_exhausted =
          result.pns_candidate_budget_exhausted || pns.budget_exhausted;
      if (pns.accepted()) {
        const std::set<std::string> generated_ids(probe->generated_ids.begin(),
                                                  probe->generated_ids.end());
        std::erase_if(pns.candidate->tracks,
                      [&](const auto& track) { return generated_ids.contains(track.id.value()); });
        std::erase_if(pns.candidate->track_arcs,
                      [&](const auto& arc) { return generated_ids.contains(arc.id.value()); });
        std::erase_if(pns.candidate->track_beziers,
                      [&](const auto& curve) { return generated_ids.contains(curve.id.value()); });
        corridor_board = std::move(*pns.candidate);
        for (const auto& track : corridor_board.tracks) {
          used_ids.insert(track.id.value());
        }
        for (const auto& arc : corridor_board.track_arcs) {
          used_ids.insert(arc.id.value());
        }
        for (const auto& curve : corridor_board.track_beziers) used_ids.insert(curve.id.value());
        for (const auto& via : corridor_board.vias) {
          used_ids.insert(via.id.value());
        }
        ++pending_pns_shoves;
        pending_pns_tracks += pns.moved_tracks;
        pending_pns_vias += pns.moved_vias;
        pending_pns_doglegs += pns.added_dogleg_tracks;
        const auto refit_started = std::chrono::steady_clock::now();
        const auto refitted =
            optimize_quintic_ph_trajectory(*spline, trajectory_config, clearance_evaluator, {},
                                            clearance_batch_evaluator, envelope_screen_evaluator);
        result.spline_trajectory_microseconds +=
            static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::microseconds>(
                std::chrono::steady_clock::now() - refit_started).count());
        result.spline_trajectory_control_moves += refitted.accepted_control_moves;
        result.spline_trajectory_candidate_checks += refitted.candidate_checks;
        result.spline_span_certificate_cache_hits += refitted.span_certificate_cache_hits;
        result.spline_span_certificate_cache_misses += refitted.span_certificate_cache_misses;
        result.spline_equivalent_candidates_skipped += refitted.equivalent_candidates_skipped;
        result.spline_non_improving_candidates_skipped += refitted.non_improving_candidates_skipped;
        result.spline_paired_obstacle_support_recoveries += refitted.paired_obstacle_support_recoveries;
        result.spline_paired_obstacle_supports_inserted += refitted.paired_obstacle_supports_inserted;
        result.spline_paired_obstacle_support_attempts += refitted.paired_obstacle_support_attempts;
        if (!refitted.accepted) {
          return false;
        }
        ++result.spline_refits;
        continue;
      }
      const auto refitted = deform_quintic_ph_elastic_at(
          *spline, parameter, normal, Nanometres{deficit}, trajectory_config, clearance_evaluator);
      if (!refitted.accepted || refitted.elastic_predeformed_controls == 0U) {
        return false;
      }
      const auto shoved_topology_samples = adaptive_tessellate_quintic_ph(
          *spline, config.spline_tessellation_tolerance, config.spline_max_tessellation_depth,
          config.max_spline_samples_per_run);
      if (!topology_preserved(shoved_topology_samples)) {
        return false;
      }
      result.spline_control_point_shoves += refitted.elastic_predeformed_controls;
      result.spline_trajectory_control_moves += refitted.accepted_control_moves;
      result.spline_trajectory_candidate_checks += refitted.candidate_checks;
      result.spline_span_certificate_cache_hits += refitted.span_certificate_cache_hits;
      result.spline_span_certificate_cache_misses += refitted.span_certificate_cache_misses;
      result.spline_equivalent_candidates_skipped += refitted.equivalent_candidates_skipped;
      result.spline_non_improving_candidates_skipped += refitted.non_improving_candidates_skipped;
      result.spline_paired_obstacle_support_recoveries += refitted.paired_obstacle_support_recoveries;
      result.spline_paired_obstacle_supports_inserted += refitted.paired_obstacle_supports_inserted;
      result.spline_paired_obstacle_support_attempts += refitted.paired_obstacle_support_attempts;
      ++result.spline_refits;
    }
    return false;
  };
  if (materialize_selected_spline()) {
    if (unpromoted_spline)
      ++result.spline_g2_promotions;
    return true;
  }
  if (!unpromoted_spline)
    return false;
  ++result.spline_g2_fallbacks;
  if (stop_token.stop_requested())
    return false;
  // A promotion earns acceptance only after its quantized copper commits. Retry the original
  // optimized G1 geometry against a fresh local transaction on fitting/DRC/shove failure.
  *spline = std::move(*unpromoted_spline);
  promotion_pending = false;
  return materialize_selected_spline();
}

template <typename CopperObject> struct CopperObjectDelta {
  // Objects which must still match the immutable task snapshot before this transaction can merge.
  std::vector<CopperObject> expected;
  std::vector<std::string> removed_ids;
  std::vector<CopperObject> upserts;
};

struct SplineRouteCopperDelta {
  CopperObjectDelta<TrackSegment> tracks;
  CopperObjectDelta<TrackArc> arcs;
  CopperObjectDelta<TrackBezier> beziers;
  CopperObjectDelta<Via> vias;
};

template <typename CopperObject>
[[nodiscard]] CopperObjectDelta<CopperObject>
make_copper_object_delta(const std::vector<CopperObject>& before,
                         const std::vector<CopperObject>& after) {
  CopperObjectDelta<CopperObject> result;
  std::map<std::string, const CopperObject*> before_by_id;
  std::map<std::string, const CopperObject*> after_by_id;
  for (const auto& object : before)
    before_by_id.emplace(object.id.value(), &object);
  for (const auto& object : after)
    after_by_id.emplace(object.id.value(), &object);
  for (const auto& [id, original] : before_by_id) {
    const auto replacement = after_by_id.find(id);
    if (replacement == after_by_id.end() || *replacement->second != *original) {
      result.expected.push_back(*original);
      result.removed_ids.push_back(id);
      if (replacement != after_by_id.end())
        result.upserts.push_back(*replacement->second);
    }
  }
  for (const auto& [id, object] : after_by_id) {
    if (!before_by_id.contains(id))
      result.upserts.push_back(*object);
  }
  return result;
}

[[nodiscard]] SplineRouteCopperDelta make_spline_route_delta(const Board& before,
                                                             const Board& after) {
  return {make_copper_object_delta(before.tracks, after.tracks),
          make_copper_object_delta(before.track_arcs, after.track_arcs),
          make_copper_object_delta(before.track_beziers, after.track_beziers),
          make_copper_object_delta(before.vias, after.vias)};
}

template <typename CopperObject>
[[nodiscard]] bool apply_copper_object_delta(std::vector<CopperObject>& objects,
                                             const CopperObjectDelta<CopperObject>& delta) {
  const auto find = [&](std::string_view id) {
    return std::find_if(objects.begin(), objects.end(),
                        [&](const auto& object) { return object.id.value() == id; });
  };
  std::set<std::string> expected_ids;
  for (const auto& expected : delta.expected) {
    expected_ids.insert(expected.id.value());
    const auto current = find(expected.id.value());
    if (current == objects.end() || *current != expected)
      return false;
  }
  for (const auto& upsert : delta.upserts) {
    if (!expected_ids.contains(upsert.id.value()) && find(upsert.id.value()) != objects.end()) {
      return false;
    }
  }
  const std::set<std::string> removed(delta.removed_ids.begin(), delta.removed_ids.end());
  std::erase_if(objects, [&](const auto& object) { return removed.contains(object.id.value()); });
  objects.insert(objects.end(), delta.upserts.begin(), delta.upserts.end());
  return true;
}

[[nodiscard]] bool apply_spline_route_delta(Board& board, const SplineRouteCopperDelta& delta) {
  // Apply to a disposable board transaction. A failure cannot partially mutate authoritative
  // state because callers publish only the complete returned candidate.
  return apply_copper_object_delta(board.tracks, delta.tracks) &&
         apply_copper_object_delta(board.track_arcs, delta.arcs) &&
         apply_copper_object_delta(board.track_beziers, delta.beziers) &&
         apply_copper_object_delta(board.vias, delta.vias);
}

[[nodiscard]] std::vector<std::string>
spline_route_delta_touched_ids(const SplineRouteCopperDelta& delta) {
  std::vector<std::string> result;
  result.reserve(delta.tracks.upserts.size() + delta.arcs.upserts.size() + delta.beziers.upserts.size() +
                 delta.vias.upserts.size());
  for (const auto& track : delta.tracks.upserts)
    result.push_back(track.id.value());
  for (const auto& arc : delta.arcs.upserts)
    result.push_back(arc.id.value());
  for (const auto& bezier : delta.beziers.upserts)
    result.push_back(bezier.id.value());
  for (const auto& via : delta.vias.upserts)
    result.push_back(via.id.value());
  return result;
}

void accumulate_spline_route_attempt_metrics(DetailedRouteResult& target,
                                             const DetailedRouteResult& source) {
#define FLUX_ACCUMULATE_SPLINE_METRIC(field) target.field += source.field
  FLUX_ACCUMULATE_SPLINE_METRIC(attempted_circular_arc_candidates);
  FLUX_ACCUMULATE_SPLINE_METRIC(rejected_circular_arc_candidates);
  FLUX_ACCUMULATE_SPLINE_METRIC(attempted_quintic_ph_runs);
  FLUX_ACCUMULATE_SPLINE_METRIC(spline_g2_promotion_attempts);
  FLUX_ACCUMULATE_SPLINE_METRIC(spline_g2_promotions);
  FLUX_ACCUMULATE_SPLINE_METRIC(spline_g2_fallbacks);
  FLUX_ACCUMULATE_SPLINE_METRIC(spline_g2_promotion_microseconds);
  FLUX_ACCUMULATE_SPLINE_METRIC(regional_native_spline_runs_materialized);
  FLUX_ACCUMULATE_SPLINE_METRIC(regional_native_track_segments_materialized);
  FLUX_ACCUMULATE_SPLINE_METRIC(regional_native_track_arcs_materialized);
  FLUX_ACCUMULATE_SPLINE_METRIC(regional_native_track_beziers_materialized);
  FLUX_ACCUMULATE_SPLINE_METRIC(fitted_cubic_beziers);
  FLUX_ACCUMULATE_SPLINE_METRIC(adaptive_spline_tessellation_samples);
  FLUX_ACCUMULATE_SPLINE_METRIC(spline_collision_envelopes);
  FLUX_ACCUMULATE_SPLINE_METRIC(spline_control_point_shoves);
  FLUX_ACCUMULATE_SPLINE_METRIC(spline_refits);
  FLUX_ACCUMULATE_SPLINE_METRIC(spline_navmesh_routes);
  FLUX_ACCUMULATE_SPLINE_METRIC(spline_navmesh_failures);
  FLUX_ACCUMULATE_SPLINE_METRIC(incremental_transaction_checks);
  FLUX_ACCUMULATE_SPLINE_METRIC(incremental_transaction_changed_objects);
  FLUX_ACCUMULATE_SPLINE_METRIC(incremental_transaction_neighbours);
  FLUX_ACCUMULATE_SPLINE_METRIC(incremental_transaction_affected_nets);
  FLUX_ACCUMULATE_SPLINE_METRIC(incremental_transaction_connectivity_hits);
  FLUX_ACCUMULATE_SPLINE_METRIC(incremental_transaction_full_fallbacks);
  FLUX_ACCUMULATE_SPLINE_METRIC(spline_navigation_preparation_builds);
  FLUX_ACCUMULATE_SPLINE_METRIC(spline_navigation_preparation_hits);
  FLUX_ACCUMULATE_SPLINE_METRIC(spline_obstacle_geometry_hits);
  FLUX_ACCUMULATE_SPLINE_METRIC(spline_obstacle_geometry_misses);
  FLUX_ACCUMULATE_SPLINE_METRIC(spline_obstacle_offset_hits);
  FLUX_ACCUMULATE_SPLINE_METRIC(spline_obstacle_offset_misses);
  FLUX_ACCUMULATE_SPLINE_METRIC(spline_free_space_input_validation_hits);
  FLUX_ACCUMULATE_SPLINE_METRIC(spline_free_space_input_validation_misses);
  FLUX_ACCUMULATE_SPLINE_METRIC(spline_free_space_prepared_domain_hits);
  FLUX_ACCUMULATE_SPLINE_METRIC(spline_free_space_prepared_source_hits);
  FLUX_ACCUMULATE_SPLINE_METRIC(spline_free_space_prepared_domain_misses);
  FLUX_ACCUMULATE_SPLINE_METRIC(spline_free_space_prepared_source_misses);
  FLUX_ACCUMULATE_SPLINE_METRIC(spline_free_space_outline_cache_hits);
  FLUX_ACCUMULATE_SPLINE_METRIC(spline_free_space_union_cache_hits);
  FLUX_ACCUMULATE_SPLINE_METRIC(spline_free_space_outline_cache_misses);
  FLUX_ACCUMULATE_SPLINE_METRIC(spline_free_space_union_cache_misses);
  FLUX_ACCUMULATE_SPLINE_METRIC(spline_free_space_boolean_conversion_microseconds);
  FLUX_ACCUMULATE_SPLINE_METRIC(spline_free_space_boolean_offset_preparation_microseconds);
  FLUX_ACCUMULATE_SPLINE_METRIC(spline_free_space_boolean_union_microseconds);
  FLUX_ACCUMULATE_SPLINE_METRIC(spline_free_space_boolean_subtraction_microseconds);
  FLUX_ACCUMULATE_SPLINE_METRIC(spline_free_space_boolean_contour_extraction_microseconds);
  FLUX_ACCUMULATE_SPLINE_METRIC(spline_free_space_clearance_certificate_hits);
  FLUX_ACCUMULATE_SPLINE_METRIC(spline_free_space_clearance_certificate_misses);
  FLUX_ACCUMULATE_SPLINE_METRIC(spline_free_space_input_validation_microseconds);
  FLUX_ACCUMULATE_SPLINE_METRIC(spline_free_space_preparation_microseconds);
  FLUX_ACCUMULATE_SPLINE_METRIC(spline_free_space_boolean_operations_microseconds);
  FLUX_ACCUMULATE_SPLINE_METRIC(spline_free_space_output_validation_microseconds);

  FLUX_ACCUMULATE_SPLINE_METRIC(spline_navigation_preparation_microseconds);
  FLUX_ACCUMULATE_SPLINE_METRIC(spline_trajectory_microseconds);
  FLUX_ACCUMULATE_SPLINE_METRIC(spline_envelope_queries);
  FLUX_ACCUMULATE_SPLINE_METRIC(spline_envelope_proven_clear);
  FLUX_ACCUMULATE_SPLINE_METRIC(spline_envelope_gpu_dispatches);
  FLUX_ACCUMULATE_SPLINE_METRIC(spline_envelope_cpu_fallbacks);
  FLUX_ACCUMULATE_SPLINE_METRIC(spline_envelope_resident_uploads);
  FLUX_ACCUMULATE_SPLINE_METRIC(spline_envelope_resident_reuses);
  FLUX_ACCUMULATE_SPLINE_METRIC(spline_envelope_microseconds);
  FLUX_ACCUMULATE_SPLINE_METRIC(spline_clearance_batch_queries);
  FLUX_ACCUMULATE_SPLINE_METRIC(spline_clearance_gpu_dispatches);
  FLUX_ACCUMULATE_SPLINE_METRIC(spline_clearance_cpu_fallbacks);
  FLUX_ACCUMULATE_SPLINE_METRIC(spline_clearance_resident_uploads);
  FLUX_ACCUMULATE_SPLINE_METRIC(spline_clearance_resident_reuses);
  FLUX_ACCUMULATE_SPLINE_METRIC(spline_clearance_batch_microseconds);
  FLUX_ACCUMULATE_SPLINE_METRIC(spline_gpu_polish_microseconds);
  FLUX_ACCUMULATE_SPLINE_METRIC(spline_navigation_mesh_microseconds);
  FLUX_ACCUMULATE_SPLINE_METRIC(spline_route_refinement_microseconds);

  FLUX_ACCUMULATE_SPLINE_METRIC(spline_navmesh_refined_parents);
  FLUX_ACCUMULATE_SPLINE_METRIC(spline_navmesh_refined_leaves);
  FLUX_ACCUMULATE_SPLINE_METRIC(spline_navmesh_rejected_leaves);
  FLUX_ACCUMULATE_SPLINE_METRIC(spline_navmesh_gpu_wavefront_routes);
  FLUX_ACCUMULATE_SPLINE_METRIC(spline_navmesh_gpu_wavefront_cpu_fallbacks);
  FLUX_ACCUMULATE_SPLINE_METRIC(spline_navmesh_certificate_fallbacks);
  FLUX_ACCUMULATE_SPLINE_METRIC(spline_navmesh_topology_signatures);
  FLUX_ACCUMULATE_SPLINE_METRIC(spline_navmesh_topology_gates);
  FLUX_ACCUMULATE_SPLINE_METRIC(spline_navmesh_topology_rejections);
  FLUX_ACCUMULATE_SPLINE_METRIC(spline_funnel_points);
  FLUX_ACCUMULATE_SPLINE_METRIC(spline_seed_candidates);
  FLUX_ACCUMULATE_SPLINE_METRIC(spline_seed_exact_legal);
  FLUX_ACCUMULATE_SPLINE_METRIC(spline_run_aligned_seed_candidates);
  FLUX_ACCUMULATE_SPLINE_METRIC(spline_run_aligned_seed_exact_legal);
  FLUX_ACCUMULATE_SPLINE_METRIC(spline_seed_portfolio_improvements);
  FLUX_ACCUMULATE_SPLINE_METRIC(spline_trajectory_attempts);
  FLUX_ACCUMULATE_SPLINE_METRIC(spline_trajectory_accepted);
  FLUX_ACCUMULATE_SPLINE_METRIC(spline_trajectory_control_moves);
  FLUX_ACCUMULATE_SPLINE_METRIC(spline_paired_obstacle_support_attempts);
  FLUX_ACCUMULATE_SPLINE_METRIC(spline_paired_obstacle_supports_inserted);
  FLUX_ACCUMULATE_SPLINE_METRIC(spline_paired_obstacle_support_recoveries);
  FLUX_ACCUMULATE_SPLINE_METRIC(spline_trajectory_candidate_checks);
  FLUX_ACCUMULATE_SPLINE_METRIC(spline_span_certificate_cache_hits);
  FLUX_ACCUMULATE_SPLINE_METRIC(spline_span_certificate_cache_misses);
  FLUX_ACCUMULATE_SPLINE_METRIC(spline_equivalent_candidates_skipped);
  FLUX_ACCUMULATE_SPLINE_METRIC(spline_non_improving_candidates_skipped);
  FLUX_ACCUMULATE_SPLINE_METRIC(spline_multistart_attempts);
  FLUX_ACCUMULATE_SPLINE_METRIC(spline_multistart_recoveries);
  FLUX_ACCUMULATE_SPLINE_METRIC(spline_gpu_precondition_attempts);
  FLUX_ACCUMULATE_SPLINE_METRIC(spline_gpu_precondition_dispatches);
  FLUX_ACCUMULATE_SPLINE_METRIC(spline_gpu_precondition_cpu_fallbacks);
  FLUX_ACCUMULATE_SPLINE_METRIC(spline_gpu_precondition_accepted);
  FLUX_ACCUMULATE_SPLINE_METRIC(spline_gpu_precondition_legal);
  FLUX_ACCUMULATE_SPLINE_METRIC(spline_gpu_precondition_solve_us);
  FLUX_ACCUMULATE_SPLINE_METRIC(spline_gpu_polish_batches);
  FLUX_ACCUMULATE_SPLINE_METRIC(spline_gpu_polish_dispatches);
  FLUX_ACCUMULATE_SPLINE_METRIC(spline_gpu_polish_cpu_fallbacks);
  FLUX_ACCUMULATE_SPLINE_METRIC(spline_gpu_polish_candidates);
  FLUX_ACCUMULATE_SPLINE_METRIC(spline_gpu_polish_exact_shortlist_checks);
  FLUX_ACCUMULATE_SPLINE_METRIC(spline_parallel_exact_waves);
  FLUX_ACCUMULATE_SPLINE_METRIC(spline_parallel_exact_checks);
  FLUX_ACCUMULATE_SPLINE_METRIC(spline_trajectory_initial_violations);
  FLUX_ACCUMULATE_SPLINE_METRIC(spline_trajectory_final_violations);
  FLUX_ACCUMULATE_SPLINE_METRIC(spline_trajectory_initial_shape_violations);
  FLUX_ACCUMULATE_SPLINE_METRIC(spline_trajectory_final_shape_violations);
  FLUX_ACCUMULATE_SPLINE_METRIC(spline_trajectory_final_self_intersections);
  FLUX_ACCUMULATE_SPLINE_METRIC(spline_trajectory_final_cusps);
  FLUX_ACCUMULATE_SPLINE_METRIC(spline_trajectory_final_control_station_reversals);
  FLUX_ACCUMULATE_SPLINE_METRIC(spline_trajectory_final_station_reversals);
  FLUX_ACCUMULATE_SPLINE_METRIC(spline_trajectory_final_excess_turn_violations);
  FLUX_ACCUMULATE_SPLINE_METRIC(spline_trajectory_final_unintended_inflections);
  FLUX_ACCUMULATE_SPLINE_METRIC(spline_trajectory_final_bend_radius_violations);
  FLUX_ACCUMULATE_SPLINE_METRIC(spline_trajectory_initial_continuous_violations);
  FLUX_ACCUMULATE_SPLINE_METRIC(spline_trajectory_final_continuous_violations);
  FLUX_ACCUMULATE_SPLINE_METRIC(spline_continuous_hulls_checked);
  FLUX_ACCUMULATE_SPLINE_METRIC(spline_continuous_envelope_rejections);
  FLUX_ACCUMULATE_SPLINE_METRIC(spline_continuous_budget_exhaustions);
  FLUX_ACCUMULATE_SPLINE_METRIC(spline_final_validation_rejections);
  FLUX_ACCUMULATE_SPLINE_METRIC(spline_final_copper_rejections);
  FLUX_ACCUMULATE_SPLINE_METRIC(spline_final_connectivity_rejections);
  FLUX_ACCUMULATE_SPLINE_METRIC(pns_shove_attempts);
  FLUX_ACCUMULATE_SPLINE_METRIC(pns_shove_candidates);
#undef FLUX_ACCUMULATE_SPLINE_METRIC
  target.circular_arc_candidate_budget_exhausted = target.circular_arc_candidate_budget_exhausted ||
                                                   source.circular_arc_candidate_budget_exhausted;
  target.spline_candidate_budget_exhausted =
      target.spline_candidate_budget_exhausted || source.spline_candidate_budget_exhausted;
  target.pns_candidate_budget_exhausted =
      target.pns_candidate_budget_exhausted || source.pns_candidate_budget_exhausted;
  if (!source.spline_navmesh_last_diagnostic.empty())
    target.spline_navmesh_last_diagnostic = source.spline_navmesh_last_diagnostic;
  if (!source.spline_navmesh_last_topology_diagnostic.empty())
    target.spline_navmesh_last_topology_diagnostic = source.spline_navmesh_last_topology_diagnostic;
  if (!source.spline_last_materialization_diagnostic.empty())
    target.spline_last_materialization_diagnostic = source.spline_last_materialization_diagnostic;
}

void accumulate_committed_spline_route_metrics(DetailedRouteResult& target,
                                               const DetailedRouteResult& source) {
  target.fitted_circular_arcs += source.fitted_circular_arcs;
  target.fitted_quintic_ph_runs += source.fitted_quintic_ph_runs;
  target.spline_fitted_track_segments += source.spline_fitted_track_segments;
  target.accepted_pns_shoves += source.accepted_pns_shoves;
  target.pns_moved_tracks += source.pns_moved_tracks;
  target.pns_moved_vias += source.pns_moved_vias;
  target.pns_added_dogleg_tracks += source.pns_added_dogleg_tracks;
}

[[nodiscard]] bool materialize_path(Board& board, const GlobalRoutePath& path,
                                    Nanometres track_width, Nanometres via_diameter,
                                    Nanometres via_drill, std::set<std::string>& used_ids,
                                    DetailedRouteResult& result, const DetailedRouterConfig& config,
                                    bool use_pull_tight, bool use_continuous_refinement,
                                    bool use_circular_arc_refinement, std::stop_token stop_token,
                                    const std::vector<GlobalRoutePoint>* topology_locked_points,
                                    std::vector<PendingSplineRun>* pending_spline_runs,
                                    const std::vector<RegionalSplineRealization>* regional_splines,
                                    std::set<std::string>* protected_native_track_ids,
                                    std::set<std::string>* scaffold_track_ids) {
  if ((!path.segment_widths.empty() && path.segment_widths.size() + 1U != path.points.size()) ||
      path.segment_width_kinds.size() != path.segment_widths.size() ||
      std::any_of(path.segment_widths.begin(), path.segment_widths.end(),
                  [](Nanometres width) { return width.value() < 0; })) {
    return false;
  }
  // Search paths may spell out the nominal width on every edge, including via edges.
  // That is the same policy as an implicit-width path, not an unaccounted neck-down.
  // Keep non-nominal kinds/counters on the strict width-aware validation path.
  const bool explicit_nominal_policy =
      std::all_of(path.segment_widths.begin(), path.segment_widths.end(),
          [track_width](Nanometres width) { return width.value() == 0 || width == track_width; }) &&
      std::all_of(path.segment_width_kinds.begin(), path.segment_width_kinds.end(),
          [](GlobalRouteSegmentWidthKind kind) { return kind == GlobalRouteSegmentWidthKind::nominal; }) &&
      path.terminal_neck_down_segments == 0U && path.medial_neck_down_segments == 0U &&
      path.corridor_neck_down_segments == 0U;
  const bool width_aware = !explicit_nominal_policy &&
      std::any_of(path.segment_widths.begin(), path.segment_widths.end(),
                  [](Nanometres width) { return width.value() > 0; });
  if (!explicit_nominal_policy && !width_aware) {
    return false;
  }
  const auto explicit_width_segments = static_cast<std::size_t>(std::count_if(
      path.segment_widths.begin(), path.segment_widths.end(),
      [track_width](Nanometres width) { return width.value() > 0 && width < track_width; }));
  if (width_aware && (explicit_width_segments != path.terminal_neck_down_segments +
                                                     path.medial_neck_down_segments +
                                                     path.corridor_neck_down_segments ||
                      explicit_width_segments == 0U)) {
    return false;
  }
  struct NativeRunMatch {
    std::size_t first_point{0U};
    std::size_t last_point{0U};
    const RegionalSplineRealization* realization{nullptr};
    bool reversed{false};
  };
  std::vector<NativeRunMatch> native_matches;
  if (regional_splines != nullptr) {
    for (const auto& realization : *regional_splines) {
      if (realization.fragment.net != path.net ||
          realization.fragment.connection_index != path.connection_index) {
        continue;
      }
      if (!realization.native_copper_legal || realization.centerline.size() < 2U ||
          realization.native_pieces.empty() || realization.width.value() <= 0 ||
          realization.native_fit_policy.fit_tolerance > config.spline_arc_fit_tolerance ||
          realization.native_fit_policy.tessellation_tolerance > config.spline_tessellation_tolerance ||
          realization.native_pieces.size() > config.max_spline_fitted_pieces_per_run ||
          native_spline_max_join_angle(realization.native_pieces) > config.spline_max_join_angle_radians) {
        return false;
      }
      std::optional<NativeRunMatch> match;
      for (std::size_t first = 0U;
           !match && first + realization.centerline.size() <= path.points.size(); ++first) {
        const auto matches = [&](bool reversed) {
          for (std::size_t offset = 0U; offset < realization.centerline.size(); ++offset) {
            const auto spline_index =
                reversed ? realization.centerline.size() - 1U - offset : offset;
            if (path.points[first + offset].layer != realization.layer ||
                path.points[first + offset].position != realization.centerline[spline_index]) {
              return false;
            }
          }
          return true;
        };
        if (matches(false)) {
          match = NativeRunMatch{first, first + realization.centerline.size() - 1U, &realization,
                                 false};
        } else if (matches(true)) {
          match =
              NativeRunMatch{first, first + realization.centerline.size() - 1U, &realization, true};
        }
      }
      if (!match) {
        return false;
      }
      for (std::size_t edge = match->first_point; edge < match->last_point; ++edge) {
        const auto width = edge < path.segment_widths.size() && path.segment_widths[edge].value() > 0
                               ? path.segment_widths[edge] : track_width;
        const auto kind = edge < path.segment_width_kinds.size() ? path.segment_width_kinds[edge]
                                                               : GlobalRouteSegmentWidthKind::nominal;
        if (realization.width != width || realization.width_kind != kind) return false;
      }
      native_matches.push_back(*match);
    }
    std::sort(native_matches.begin(), native_matches.end(), [](const auto& lhs, const auto& rhs) {
      return std::tie(lhs.first_point, lhs.last_point) < std::tie(rhs.first_point, rhs.last_point);
    });
    if (std::adjacent_find(native_matches.begin(), native_matches.end(),
                           [](const auto& lhs, const auto& rhs) {
                             return lhs.last_point > rhs.first_point;
                           }) != native_matches.end()) {
      return false;
    }
  }
  std::vector<GlobalRoutePoint> points;
  std::vector<Nanometres> segment_widths;
  std::size_t width_aware_removed_points = 0U;
  if (!native_matches.empty() && !width_aware && topology_locked_points == nullptr) {
    // Preserve each certified native interval byte-for-byte, but do not freeze the rest of a
    // potentially long connection into its raster staircase. Process the open intervals on both
    // sides with the ordinary pull-tight/continuous path and remap the protected native ranges
    // into that result. This is still a single realization of every regional PH curve.
    const auto source_matches = native_matches;
    native_matches.clear();
    const auto append_open_interval = [&](std::size_t first, std::size_t last) {
      if (first > last || last >= path.points.size()) {
        return false;
      }
      const auto first_offset = static_cast<std::ptrdiff_t>(first);
      const auto last_offset = static_cast<std::ptrdiff_t>(last + 1U);
      std::vector<GlobalRoutePoint> interval(path.points.begin() + first_offset,
                                             path.points.begin() + last_offset);
      interval = use_pull_tight ? pull_tight_45_degrees(interval, result.pulled_tight_corners)
                                : simplify_path(interval);
      if (use_continuous_refinement && interval.size() >= 2U) {
        std::size_t iterations = 0U;
        interval = refine_path_continuously(board, path.net, track_width, interval, config, result,
                                            iterations, nullptr);
        result.continuous_refinement_iterations =
            std::max(result.continuous_refinement_iterations, iterations);
      }
      for (const auto& point : interval) {
        if (points.empty() || points.back() != point) {
          points.push_back(point);
        }
      }
      return !interval.empty();
    };
    std::size_t cursor = 0U;
    for (const auto& match : source_matches) {
      if (!append_open_interval(cursor, match.first_point) || points.empty() ||
          points.back() != path.points[match.first_point]) {
        return false;
      }
      NativeRunMatch remapped{points.size() - 1U, 0U, match.realization, match.reversed};
      for (std::size_t source_point = match.first_point + 1U; source_point <= match.last_point;
           ++source_point) {
        points.push_back(path.points[source_point]);
      }
      remapped.last_point = points.size() - 1U;
      native_matches.push_back(remapped);
      cursor = match.last_point;
    }
    if (!append_open_interval(cursor, path.points.size() - 1U)) {
      return false;
    }
  } else if (!native_matches.empty()) {
    // Width-policy and topology-locked paths keep their complete selected point sequence. Their
    // existing specialized transformations cannot yet move only the open intervals around an
    // immutable native curve.
    points = path.points;
    segment_widths = path.segment_widths;
  } else if (topology_locked_points != nullptr) {
    // A certified scaffold is an exact handoff contract.  Do not simplify a collinear escape
    // stub away: its first/last directions and repeated-position layer transitions are selected
    // topology.  Explicit widths remain aligned with this unmodified point sequence.
    points = *topology_locked_points;
    if (width_aware) {
      segment_widths.reserve(path.segment_widths.size());
      for (const auto selected : path.segment_widths) {
        const auto width = selected.value() > 0 ? selected : track_width;
        if (width.value() <= 0) return false;
        segment_widths.push_back(width);
      }
    }
  } else if (width_aware) {
    const auto simplified =
        simplify_width_aware_path(path, track_width, width_aware_removed_points);
    if (!simplified) {
      return false;
    }
    points = simplified->points;
    segment_widths = simplified->widths;
    if (use_pull_tight) {
      const auto corners_before = result.pulled_tight_corners;
      const auto pulled = pull_tight_width_aware_45_degrees(WidthAwarePath{points, segment_widths},
                                                            result.pulled_tight_corners,
                                                            width_aware_removed_points);
      if (!pulled) {
        return false;
      }
      points = pulled->points;
      segment_widths = pulled->widths;
      const auto pulled_corners = result.pulled_tight_corners - corners_before;
      result.width_aware_pulled_tight_corners += pulled_corners;
      if (pulled_corners > 0U) {
        ++result.width_aware_pulled_tight_paths;
      }
    }
  } else {
    points = use_pull_tight ? pull_tight_45_degrees(path.points, result.pulled_tight_corners)
                             : simplify_path(path.points);
  }
  if (use_continuous_refinement && topology_locked_points == nullptr && native_matches.empty()) {
    std::size_t iterations = 0;
    const auto refined_before = result.continuously_refined_points;
    points = refine_path_continuously(board, path.net, track_width, points, config, result,
                                      iterations, width_aware ? &segment_widths : nullptr);
    if (width_aware) {
      const auto refined = result.continuously_refined_points - refined_before;
      result.width_aware_continuously_refined_points += refined;
      if (refined > 0U) {
        ++result.width_aware_continuously_refined_paths;
      }
    }
    result.continuous_refinement_iterations =
        std::max(result.continuous_refinement_iterations, iterations);
  }
  if (points.empty()) {
    return false;
  }
  const auto base = "flux-route-" + path.net.value() + "-" + std::to_string(path.connection_index);
  std::size_t track_index = 0;
  std::size_t native_arc_index = 0;
  std::size_t via_index = 0;
  std::map<std::size_t, std::size_t> planar_track_by_edge;
  for (std::size_t i = 0; i + 1U < points.size();) {
    routing_kernel_checkpoint();
    if (stop_token.stop_requested()) {
      return false;
    }
    const auto& current = points[i];
    const auto& next = points[i + 1U];
    const auto native = std::find_if(native_matches.begin(), native_matches.end(),
                                     [i](const auto& match) { return match.first_point == i; });
    if (native != native_matches.end()) {
      const auto& realization = *native->realization;
      const auto expected_start = current.position;
      const auto expected_end = points[native->last_point].position;
      const auto& first_piece =
          native->reversed ? realization.native_pieces.back() : realization.native_pieces.front();
      const auto& last_piece =
          native->reversed ? realization.native_pieces.front() : realization.native_pieces.back();
      const auto materialized_start = native->reversed ? first_piece.end : first_piece.start;
      const auto materialized_end = native->reversed ? last_piece.start : last_piece.end;
      if (materialized_start != expected_start || materialized_end != expected_end) {
        return false;
      }
      const auto arcs = static_cast<std::size_t>(std::count_if(
          realization.native_pieces.begin(), realization.native_pieces.end(),
          [](const auto& piece) { return piece.kind == SplineFittedPieceKind::circular_arc; }));
      if (arcs > config.max_circular_arcs - std::min(config.max_circular_arcs, result.fitted_circular_arcs)) {
        result.circular_arc_candidate_budget_exhausted = true;
        return false;
      }
      for (std::size_t piece = 0U; piece < realization.native_pieces.size(); ++piece) {
        const auto& current_piece = realization.native_pieces[piece];
        if (current_piece.start == current_piece.end ||
            (piece > 0U && realization.native_pieces[piece - 1U].end != current_piece.start))
          return false;
      }
      for (std::size_t piece_index = 0U; piece_index < realization.native_pieces.size();
           ++piece_index) {
        const auto source_index =
            native->reversed ? realization.native_pieces.size() - 1U - piece_index : piece_index;
        const auto& piece = realization.native_pieces[source_index];
        const auto start = native->reversed ? piece.end : piece.start;
        const auto end = native->reversed ? piece.start : piece.end;
        if (piece.kind == SplineFittedPieceKind::cubic_bezier) {
          const auto id = unique_id(base + "-regional-native-bezier-" + std::to_string(track_index++), used_ids);
          const auto control1 = native->reversed ? piece.control2 : piece.control1;
          const auto control2 = native->reversed ? piece.control1 : piece.control2;
          board.track_beziers.push_back(TrackBezier{id, path.net, realization.layer, start,
                                                    control1, control2, end, realization.width,
                                                    false});
          if (protected_native_track_ids != nullptr) protected_native_track_ids->insert(id.value());
          ++result.added_track_beziers;
          ++result.fitted_cubic_beziers;
          ++result.regional_native_track_beziers_materialized;
        } else if (piece.kind == SplineFittedPieceKind::circular_arc) {
          const auto id = unique_id(base + "-regional-native-arc-" + std::to_string(native_arc_index++), used_ids);
          board.track_arcs.push_back(TrackArc{
              id,
              path.net, realization.layer, start, piece.middle, end, realization.width, false});
          if (protected_native_track_ids != nullptr) protected_native_track_ids->insert(id.value());
          ++result.added_track_arcs;
          ++result.fitted_circular_arcs;
          ++result.regional_native_track_arcs_materialized;
        } else {
          const auto id =
              unique_id(base + "-regional-native-track-" + std::to_string(track_index++), used_ids);
          board.tracks.push_back(
              TrackSegment{id, path.net, realization.layer, start, end, realization.width, false});
          // Native line adapters are legal copper, but they are not necessarily finished
          // curves: two adapters, or an adapter and an ordinary path, may form a bend.
          // Let the board-complete degree-two PH transaction realize those seams. Native
          // arcs remain immutable above; all replacements still pass exact merge checks.
          ++result.added_tracks;
          ++result.spline_fitted_track_segments;
          ++result.regional_native_track_segments_materialized;
        }
      }
      ++result.fitted_quintic_ph_runs;
      ++result.regional_native_spline_runs_materialized;
      i = native->last_point;
      continue;
    }
    if (current.layer == next.layer) {
      if (current.position != next.position) {
        const auto width = width_aware ? segment_widths[i] : track_width;
        if (width.value() <= 0 || (width_aware && width > track_width)) {
          return false;
        }
        const auto board_track_index = board.tracks.size();
        board.tracks.push_back(
            TrackSegment{unique_id(base + "-track-" + std::to_string(track_index++), used_ids),
                         path.net, current.layer, current.position, next.position, width, false});
        if (scaffold_track_ids != nullptr && path.legal_scaffold) {
          scaffold_track_ids->insert(board.tracks.back().id.value());
        }
        planar_track_by_edge.emplace(i, board_track_index);
        ++result.added_tracks;
      }
      ++i;
      continue;
    }
    if (current.position != next.position) {
      return false;
    }
    if (width_aware && segment_widths[i] != track_width) {
      return false;
    }
    auto last_transition = i + 1U;
    while (last_transition + 1U < points.size() &&
           points[last_transition + 1U].position == current.position &&
           points[last_transition + 1U].layer != points[last_transition].layer) {
      ++last_transition;
    }
    auto layers = via_span(board, current.layer, points[last_transition].layer);
    if (layers.size() < 2U) {
      return false;
    }
    // A source or checkpoint via spanning more layers than this particular route transition is
    // already conductive across the requested span. Reuse it rather than placing concentric
    // same-net copper merely because the requested layer vector is a strict subset.
    const auto covers_layers = [&](const auto& existing_layers) {
      return std::all_of(layers.begin(), layers.end(), [&](const auto& layer) {
        return std::find(existing_layers.begin(), existing_layers.end(), layer) !=
               existing_layers.end();
      });
    };
    const bool duplicate_via =
        std::any_of(board.vias.begin(), board.vias.end(), [&](const auto& via) {
          return via.net == path.net && via.position == current.position &&
                 covers_layers(via.layers);
        });
    const bool conductive_pad =
        std::any_of(board.footprints.begin(), board.footprints.end(), [&](const auto& footprint) {
          return std::any_of(footprint.pads.begin(), footprint.pads.end(), [&](const auto& pad) {
            return pad.net == path.net && pad.plated && pad_has_drill(pad) &&
                   pad_drill_world_position(footprint, pad) == current.position &&
                   covers_layers(pad.layers);
          });
        });
    const bool duplicate = duplicate_via || conductive_pad;
    if (!duplicate) {
      board.vias.push_back(Via{unique_id(base + "-via-" + std::to_string(via_index++), used_ids),
                               path.net, current.position, via_diameter, via_drill,
                               std::move(layers), false});
      ++result.added_vias;
    }
    i = last_transition;
  }
  if (use_circular_arc_refinement) {
    std::vector<SplineRun> spline_runs;
    for (std::size_t edge = 0U; edge + 1U < points.size();) {
      const auto first_track = planar_track_by_edge.find(edge);
      if (first_track == planar_track_by_edge.end()) {
        ++edge;
        continue;
      }
      const auto layer = points[edge].layer;
      const auto width = board.tracks[first_track->second].width;
      const auto first_edge = edge;
      while (edge + 1U < points.size()) {
        const auto found = planar_track_by_edge.find(edge);
        if (found == planar_track_by_edge.end() || points[edge].layer != layer ||
            points[edge + 1U].layer != layer || board.tracks[found->second].width != width) {
          break;
        }
        ++edge;
      }
      if (edge - first_edge < 2U) {
        continue;
      }
      SplineRun run{layer, width, {}, {}};
      for (auto point = first_edge; point <= edge; ++point) {
        run.points.push_back(points[point].position);
      }
      for (auto run_edge = first_edge; run_edge < edge; ++run_edge) {
        run.original_track_ids.push_back(
            board.tracks[planar_track_by_edge.at(run_edge)].id.value());
      }
      bool bends = false;
      for (std::size_t point = 1U; point + 1U < run.points.size(); ++point) {
        bends =
            bends || !collinear(run.points[point - 1U], run.points[point], run.points[point + 1U]);
      }
      if (bends) {
        spline_runs.push_back(std::move(run));
      }
    }
    for (const auto& run : spline_runs) {
      result.attempted_circular_arc_corners += run.points.size() - 2U;
      if (pending_spline_runs != nullptr) {
        pending_spline_runs->push_back(PendingSplineRun{path.net, base, run});
      } else if (!refine_spline_run(board, board, path.net, run, base, used_ids, result, config,
                                    stop_token)) {
        result.withheld_circular_arc_corners += run.points.size() - 2U;
      }
    }
  }
  // The v0.104 independent-corner implementation remains source history only. A positive spline
  // candidate budget is mandatory, so every enabled curved transaction uses the quintic PH
  // curve/PNS pipeline above rather than running a second unrelated fillet pass.
  if (use_circular_arc_refinement && config.max_spline_candidate_checks == 0U) {
    const auto fitted_before = result.fitted_circular_arcs;
    const auto make_fillet = make_circular_fillet;

    std::size_t arc_index = 0;
    for (std::size_t corner_index = 1U; corner_index + 1U < points.size(); ++corner_index) {
      routing_kernel_checkpoint();
      if (stop_token.stop_requested()) {
        return false;
      }
      if (points[corner_index - 1U].layer != points[corner_index].layer ||
          points[corner_index].layer != points[corner_index + 1U].layer) {
        continue;
      }
      const auto incoming = planar_track_by_edge.find(corner_index - 1U);
      const auto outgoing = planar_track_by_edge.find(corner_index);
      if (incoming == planar_track_by_edge.end() || outgoing == planar_track_by_edge.end()) {
        continue;
      }
      if (width_aware && segment_widths[corner_index - 1U] != segment_widths[corner_index]) {
        continue;
      }
      ++result.attempted_circular_arc_corners;
      if (result.fitted_circular_arcs >= config.max_circular_arcs ||
          result.attempted_circular_arc_candidates >= config.max_circular_arc_candidate_checks) {
        result.circular_arc_candidate_budget_exhausted =
            result.attempted_circular_arc_candidates >= config.max_circular_arc_candidate_checks;
        ++result.withheld_circular_arc_corners;
        continue;
      }
      auto& incoming_track = board.tracks[incoming->second];
      auto& outgoing_track = board.tracks[outgoing->second];
      const auto original_incoming_end = incoming_track.end;
      const auto original_outgoing_start = outgoing_track.start;
      const auto arc_id = unique_id(base + "-arc-" + std::to_string(arc_index++), used_ids);
      const auto local_width = width_aware ? segment_widths[corner_index] : track_width;
      std::optional<std::tuple<Point, Point, Point>> previous_geometry;
      std::vector<std::size_t> ranked_backtracks;
      std::vector<RoutingComputeArcCandidate> arc_compute_candidates;
      std::vector<std::size_t> arc_compute_backtracks;
      auto preview_radius = config.maximum_circular_arc_radius;
      std::optional<std::tuple<Point, Point, Point>> preview_previous;
      for (std::size_t backtrack = 0U; backtrack <= config.circular_arc_backtracking_steps;
           ++backtrack) {
        if (preview_radius.value() <= 1)
          break;
        const auto preview =
            make_fillet(points[corner_index - 1U].position, points[corner_index].position,
                        points[corner_index + 1U].position, preview_radius);
        if (!preview)
          break;
        const auto geometry = std::tuple{preview->start, preview->middle, preview->end};
        if (!preview_previous || *preview_previous != geometry) {
          preview_previous = geometry;
          arc_compute_candidates.push_back(
              RoutingComputeArcCandidate{{{{points[corner_index - 1U].position.x.value(),
                                            points[corner_index - 1U].position.y.value()},
                                           {preview->start.x.value(), preview->start.y.value()},
                                           {preview->middle.x.value(), preview->middle.y.value()},
                                           {preview->end.x.value(), preview->end.y.value()},
                                           {points[corner_index + 1U].position.x.value(),
                                            points[corner_index + 1U].position.y.value()}}},
                                         preview_radius.value(),
                                         local_width.value() / 2,
                                         0U,
                                         0U});
          arc_compute_backtracks.push_back(backtrack);
        }
        preview_radius = Nanometres{preview_radius.value() / 2};
      }
      std::vector<RoutingComputeSegment> arc_obstacles;
      for (const auto& track : board.tracks) {
        if (track.layer == points[corner_index].layer && track.net != path.net)
          arc_obstacles.push_back({track.start.x.value(), track.start.y.value(),
                                   track.end.x.value(), track.end.y.value(),
                                   track.width.value() / 2, 0U, 1U});
      }
      for (const auto& via : board.vias) {
        if (via.net != path.net && std::find(via.layers.begin(), via.layers.end(),
                                             points[corner_index].layer) != via.layers.end())
          arc_obstacles.push_back({via.position.x.value(), via.position.y.value(),
                                   via.position.x.value(), via.position.y.value(),
                                   via.diameter.value() / 2, 0U, 1U});
      }
      const auto arc_ranking =
          arc_compute_candidates.empty()
              ? RoutingComputeArcCandidateResult{}
              : score_routing_arc_candidates(
                    arc_compute_candidates, arc_obstacles,
                    resolve_net_routing_rules(board, path.net).clearance.value(),
                    arc_compute_candidates.size(), true);
      if (arc_ranking.ok())
        for (const auto& candidate : arc_ranking.shortlist)
          ranked_backtracks.push_back(arc_compute_backtracks[candidate.candidate]);
      if (ranked_backtracks.empty())
        ranked_backtracks = arc_compute_backtracks;
      bool accepted = false;
      for (const auto backtrack : ranked_backtracks) {
        auto radius = config.maximum_circular_arc_radius;
        for (std::size_t step = 0U; step < backtrack; ++step)
          radius = Nanometres{radius.value() / 2};
        if (radius.value() <= 1 ||
            result.attempted_circular_arc_candidates >= config.max_circular_arc_candidate_checks) {
          result.circular_arc_candidate_budget_exhausted =
              result.attempted_circular_arc_candidates >= config.max_circular_arc_candidate_checks;
          break;
        }
        auto fillet = make_fillet(points[corner_index - 1U].position, points[corner_index].position,
                                  points[corner_index + 1U].position, radius);
        if (!fillet) {
          break;
        }
        const auto geometry = std::tuple{fillet->start, fillet->middle, fillet->end};
        if (previous_geometry && *previous_geometry == geometry) {
          continue;
        }
        previous_geometry = geometry;
        ++result.attempted_circular_arc_candidates;
        fillet->id = arc_id;
        fillet->net = path.net;
        fillet->layer = points[corner_index].layer;
        fillet->width = local_width;
        incoming_track.end = fillet->start;
        outgoing_track.start = fillet->end;
        board.track_arcs.push_back(*fillet);

        Board probe = board;
        if (config.defer_saved_zone_fill_clearance_to_refill) {
          for (auto& zone : probe.zones) {
            zone.fills.clear();
          }
        }
        const auto validation = validate_board(probe);
        const auto copper = check_copper_rules(probe);
        const std::array changed_ids{arc_id.value(), incoming_track.id.value(),
                                     outgoing_track.id.value()};
        const auto changed_copper_violation = std::any_of(
            copper.violations.begin(), copper.violations.end(),
            [&changed_ids](const auto& violation) {
              if (violation.kind == CopperViolationKind::differential_pair_skew) {
                return false;
              }
              return std::any_of(changed_ids.begin(), changed_ids.end(), [&](const auto& id) {
                return std::find(violation.object_ids.begin(), violation.object_ids.end(), id) !=
                       violation.object_ids.end();
              });
            });
        if (validation.ok() && !changed_copper_violation) {
          accepted = true;
          ++result.added_track_arcs;
          ++result.fitted_circular_arcs;
          if (backtrack > 0U) {
            ++result.radius_reduced_circular_arcs;
          }
          break;
        }

        board.track_arcs.pop_back();
        incoming_track.end = original_incoming_end;
        outgoing_track.start = original_outgoing_start;
        ++result.rejected_circular_arc_candidates;
      }
      if (!accepted) {
        incoming_track.end = original_incoming_end;
        outgoing_track.start = original_outgoing_start;
        ++result.withheld_circular_arc_corners;
      }
    }
    if (width_aware) {
      const auto fitted = result.fitted_circular_arcs - fitted_before;
      result.width_aware_fitted_circular_arcs += fitted;
      if (fitted > 0U) {
        ++result.width_aware_circular_arc_paths;
      }
    }
  }
  if (width_aware) {
    result.width_aware_simplified_points += width_aware_removed_points;
    if (path.terminal_neck_down_segments > 0U) {
      ++result.materialized_terminal_neck_down_paths;
    }
    result.materialized_terminal_neck_down_segments += path.terminal_neck_down_segments;
    if (path.medial_neck_down_segments > 0U) {
      ++result.materialized_medial_neck_down_paths;
    }
    result.materialized_medial_neck_down_segments += path.medial_neck_down_segments;
    if (path.corridor_neck_down_segments > 0U) {
      ++result.materialized_corridor_neck_down_paths;
    }
    result.materialized_corridor_neck_down_segments += path.corridor_neck_down_segments;
  }
  return true;
}

struct SkewTuningTrackCandidate {
  std::size_t index{0};
  std::uint64_t length_nm{0};
  std::string id;
};

[[nodiscard]] std::uint64_t absolute_difference(std::uint64_t lhs, std::uint64_t rhs) {
  return lhs > rhs ? lhs - rhs : rhs - lhs;
}

[[nodiscard]] std::vector<std::uint64_t>
skew_tuning_total_amplitudes(std::uint64_t difference_nm, std::uint64_t maximum_skew_nm,
                             const DetailedRouterConfig& config) {
  if (difference_nm <= maximum_skew_nm) {
    return {};
  }
  const auto excess = difference_nm - maximum_skew_nm;
  const auto minimum_total_amplitude = excess / 2U + excess % 2U;
  const auto sum = difference_nm > std::numeric_limits<std::uint64_t>::max() - maximum_skew_nm
                       ? std::numeric_limits<std::uint64_t>::max()
                       : difference_nm + maximum_skew_nm;
  const auto period_amplitude =
      static_cast<std::uint64_t>(config.maximum_differential_pair_skew_tuning_amplitude.value());
  const auto period_count =
      static_cast<std::uint64_t>(config.max_differential_pair_skew_tuning_periods);
  const auto bounded_total =
      period_amplitude > std::numeric_limits<std::uint64_t>::max() / period_count
          ? std::numeric_limits<std::uint64_t>::max()
          : period_amplitude * period_count;
  const auto maximum_total_amplitude = std::min(sum / 2U, bounded_total);
  if (minimum_total_amplitude > maximum_total_amplitude) {
    return {};
  }

  std::vector<std::uint64_t> result;
  const auto append = [&](std::uint64_t total_amplitude) {
    if (total_amplitude < minimum_total_amplitude || total_amplitude > maximum_total_amplitude ||
        total_amplitude == 0U ||
        std::find(result.begin(), result.end(), total_amplitude) != result.end()) {
      return;
    }
    result.push_back(total_amplitude);
  };
  const auto lower_midpoint = difference_nm / 2U;
  const auto upper_midpoint = lower_midpoint + difference_nm % 2U;
  append(lower_midpoint);
  append(upper_midpoint);
  append(minimum_total_amplitude);
  append(maximum_total_amplitude);

  const auto step = static_cast<std::uint64_t>(config.differential_pair_skew_tuning_step.value());
  const auto cap = config.max_differential_pair_skew_tuning_candidates;
  for (std::uint64_t offset = step; result.size() < cap;) {
    bool in_range = false;
    if (lower_midpoint >= offset) {
      const auto below = lower_midpoint - offset;
      if (below >= minimum_total_amplitude) {
        append(below);
        in_range = true;
      }
    }
    if (lower_midpoint <= std::numeric_limits<std::uint64_t>::max() - offset) {
      const auto above = lower_midpoint + offset;
      if (above <= maximum_total_amplitude) {
        append(above);
        in_range = true;
      }
    }
    if (!in_range || offset > std::numeric_limits<std::uint64_t>::max() - step) {
      break;
    }
    offset += step;
  }
  return result;
}

struct SkewTuningFrame {
  enum class Axis { none, horizontal, vertical };

  Point origin;
  long double tangent_x{0.0L};
  long double tangent_y{0.0L};
  long double normal_x{0.0L};
  long double normal_y{0.0L};
  std::uint64_t baseline_length_nm{0};
  Axis axis{Axis::none};
  std::int64_t along_sign{0};
};

[[nodiscard]] std::optional<SkewTuningFrame> make_skew_tuning_frame(const TrackSegment& original) {
  const auto dx = static_cast<long double>(original.end.x.value()) -
                  static_cast<long double>(original.start.x.value());
  const auto dy = static_cast<long double>(original.end.y.value()) -
                  static_cast<long double>(original.start.y.value());
  const auto length = std::hypot(dx, dy);
  if (!std::isfinite(length) || length <= 0.0L) {
    return std::nullopt;
  }
  const auto tangent_x = dx / length;
  const auto tangent_y = dy / length;
  auto normal_x = -tangent_y;
  auto normal_y = tangent_x;
  if (normal_y < 0.0L || (normal_y == 0.0L && normal_x < 0.0L)) {
    normal_x = -normal_x;
    normal_y = -normal_y;
  }
  auto axis = SkewTuningFrame::Axis::none;
  std::int64_t along_sign = 0;
  if (original.start.y == original.end.y) {
    axis = SkewTuningFrame::Axis::horizontal;
    along_sign = original.end.x > original.start.x ? 1 : -1;
  } else if (original.start.x == original.end.x) {
    axis = SkewTuningFrame::Axis::vertical;
    along_sign = original.end.y > original.start.y ? 1 : -1;
  }
  return SkewTuningFrame{original.start, tangent_x,
                         tangent_y,      normal_x,
                         normal_y,       geometry::segment_length_nm(original.start, original.end),
                         axis,           along_sign};
}

[[nodiscard]] std::optional<Point> quantize_tuning_point(const SkewTuningFrame& frame,
                                                         std::uint64_t along_nm,
                                                         std::int64_t offset_nm) {
  if (frame.axis != SkewTuningFrame::Axis::none) {
    if (along_nm > static_cast<std::uint64_t>(std::numeric_limits<std::int64_t>::max())) {
      return std::nullopt;
    }
    const auto checked_add = [](std::int64_t value,
                                std::int64_t delta) -> std::optional<std::int64_t> {
      if ((delta > 0 && value > std::numeric_limits<std::int64_t>::max() - delta) ||
          (delta < 0 && value < std::numeric_limits<std::int64_t>::min() - delta)) {
        return std::nullopt;
      }
      return value + delta;
    };
    const auto along_delta = static_cast<std::int64_t>(along_nm) * frame.along_sign;
    const auto x_delta = frame.axis == SkewTuningFrame::Axis::horizontal ? along_delta : offset_nm;
    const auto y_delta = frame.axis == SkewTuningFrame::Axis::horizontal ? offset_nm : along_delta;
    const auto x = checked_add(frame.origin.x.value(), x_delta);
    const auto y = checked_add(frame.origin.y.value(), y_delta);
    if (!x || !y) {
      return std::nullopt;
    }
    return Point{Nanometres{*x}, Nanometres{*y}};
  }
  const auto along = static_cast<long double>(along_nm);
  const auto offset = static_cast<long double>(offset_nm);
  const auto x = static_cast<long double>(frame.origin.x.value()) + frame.tangent_x * along +
                 frame.normal_x * offset;
  const auto y = static_cast<long double>(frame.origin.y.value()) + frame.tangent_y * along +
                 frame.normal_y * offset;
  constexpr auto minimum = static_cast<long double>(std::numeric_limits<std::int64_t>::min());
  constexpr auto maximum = static_cast<long double>(std::numeric_limits<std::int64_t>::max());
  if (!std::isfinite(x) || !std::isfinite(y) || x < minimum || x > maximum || y < minimum ||
      y > maximum) {
    return std::nullopt;
  }
  return Point{Nanometres{static_cast<std::int64_t>(std::llround(x))},
               Nanometres{static_cast<std::int64_t>(std::llround(y))}};
}

[[nodiscard]] std::vector<std::uint64_t>
distribute_tuning_amplitude(std::uint64_t total_amplitude, std::size_t periods,
                            std::uint64_t maximum_period_amplitude) {
  if (periods == 0U || total_amplitude < periods) {
    return {};
  }
  const auto period_count = static_cast<std::uint64_t>(periods);
  const auto base = total_amplitude / period_count;
  const auto remainder = total_amplitude % period_count;
  if (base == 0U || base > maximum_period_amplitude ||
      (remainder > 0U && base == maximum_period_amplitude)) {
    return {};
  }
  std::vector<std::uint64_t> amplitudes(periods, base);
  for (std::size_t index = 0U; index < static_cast<std::size_t>(remainder); ++index) {
    ++amplitudes[index];
  }
  return amplitudes;
}

[[nodiscard]] std::vector<std::pair<Point, Point>>
make_skew_tuning_pattern(const TrackSegment& original, std::uint64_t lead_nm,
                         std::uint64_t spacing_nm, const std::vector<std::uint64_t>& amplitudes,
                         std::size_t layout) {
  if (amplitudes.empty()) {
    return {};
  }
  const auto frame = make_skew_tuning_frame(original);
  if (!frame) {
    return {};
  }
  const auto periods = static_cast<std::uint64_t>(amplitudes.size());
  if (periods > std::numeric_limits<std::uint64_t>::max() / 2U) {
    return {};
  }
  const auto occupied_periods = periods * 2U - 1U;
  if (spacing_nm > std::numeric_limits<std::uint64_t>::max() / occupied_periods) {
    return {};
  }
  const auto occupied = occupied_periods * spacing_nm;
  if (frame->baseline_length_nm <= lead_nm * 2U ||
      occupied > frame->baseline_length_nm - lead_nm * 2U) {
    return {};
  }
  const auto slack = frame->baseline_length_nm - lead_nm * 2U - occupied;
  auto along = lead_nm + slack / 2U;
  auto current = quantize_tuning_point(*frame, along, 0);
  if (!current) {
    return {};
  }
  std::vector<std::pair<Point, Point>> segments;
  segments.reserve(amplitudes.size() * 4U + 1U);
  const auto append = [&segments](Point start, Point end) {
    if (start == end) {
      return false;
    }
    segments.emplace_back(start, end);
    return true;
  };
  if (!append(original.start, *current)) {
    return {};
  }
  for (std::size_t period = 0U; period < amplitudes.size(); ++period) {
    int side = layout == 0U ? -1 : 1;
    if (layout >= 2U) {
      const auto first_side = layout == 2U ? -1 : 1;
      side = period % 2U == 0U ? first_side : -first_side;
    }
    if (amplitudes[period] > static_cast<std::uint64_t>(std::numeric_limits<std::int64_t>::max())) {
      return {};
    }
    const auto signed_amplitude = static_cast<std::int64_t>(amplitudes[period]) * side;
    const auto offset = quantize_tuning_point(*frame, along, signed_amplitude);
    along += spacing_nm;
    const auto offset_end = quantize_tuning_point(*frame, along, signed_amplitude);
    const auto baseline_end = quantize_tuning_point(*frame, along, 0);
    if (!offset || !offset_end || !baseline_end || !append(*current, *offset) ||
        !append(*offset, *offset_end) || !append(*offset_end, *baseline_end)) {
      return {};
    }
    current = baseline_end;
    if (period + 1U < amplitudes.size()) {
      along += spacing_nm;
      const auto next = quantize_tuning_point(*frame, along, 0);
      if (!next || !append(*current, *next)) {
        return {};
      }
      current = next;
    }
  }
  if (!append(*current, original.end)) {
    return {};
  }
  return segments;
}

[[nodiscard]] bool skew_tuning_candidate_is_legal(const Board& board,
                                                  const DifferentialPair& pair) {
  if (!validate_board(board).ok() || analyze_connectivity(board).open_connections != 0U) {
    return false;
  }
  const auto drc = check_copper_rules(board);
  return std::all_of(drc.violations.begin(), drc.violations.end(), [&](const auto& violation) {
    return violation.kind == CopperViolationKind::differential_pair_skew &&
           !violation.object_ids.empty() && violation.object_ids.front() != pair.id.value();
  });
}

[[nodiscard]] bool skew_tuning_selection_is_legal(const Board& board,
                                                  const std::set<std::string>& selected_pair_ids) {
  if (!validate_board(board).ok() || analyze_connectivity(board).open_connections != 0U) {
    return false;
  }
  const auto drc = check_copper_rules(board);
  return std::all_of(drc.violations.begin(), drc.violations.end(), [&](const auto& violation) {
    return violation.kind == CopperViolationKind::differential_pair_skew &&
           !violation.object_ids.empty() &&
           !selected_pair_ids.contains(violation.object_ids.front());
  });
}

enum class SkewTuningStatus { completed, cancelled };

[[nodiscard]] SkewTuningStatus tune_differential_pair_skew(
    Board& board, const GlobalRouteResult& global, const std::set<ObjectId>& protected_track_ids,
    const std::set<NetId>& protected_routing_nets, std::set<std::string>& used_ids,
    const DetailedRouterConfig& config, DetailedRouteResult& result, std::stop_token stop_token) {
  std::vector<DifferentialPair> pairs;
  for (const auto& pair : board.differential_pairs) {
    if (pair.maximum_skew) {
      pairs.push_back(pair);
    }
  }
  std::sort(pairs.begin(), pairs.end(),
            [](const auto& lhs, const auto& rhs) { return lhs.id < rhs.id; });

  for (const auto& pair_snapshot : pairs) {
    const auto* pair = &pair_snapshot;
    routing_kernel_checkpoint();
    if (stop_token.stop_requested()) {
      return SkewTuningStatus::cancelled;
    }
    const auto positive_length = routed_net_centerline_length_nm(board, pair->positive_net);
    const auto negative_length = routed_net_centerline_length_nm(board, pair->negative_net);
    const auto difference = absolute_difference(positive_length, negative_length);
    const auto maximum_skew = static_cast<std::uint64_t>(pair->maximum_skew->value());
    if (difference <= maximum_skew) {
      continue;
    }
    ++result.attempted_differential_pair_skew_tuning_pairs;
    const auto pad_count = [&board](const NetId& net) {
      std::size_t count = 0U;
      for (const auto& footprint : board.footprints) {
        count += static_cast<std::size_t>(
            std::count_if(footprint.pads.begin(), footprint.pads.end(),
                          [&net](const auto& pad) { return pad.net == net; }));
      }
      return count;
    };
    const auto path_count = [&global](const NetId& net) {
      return static_cast<std::size_t>(
          std::count_if(global.paths.begin(), global.paths.end(),
                        [&net](const auto& path) { return path.net == net; }));
    };
    const bool physical_zone =
        std::any_of(board.zones.begin(), board.zones.end(), [&](const auto& zone) {
          return (zone.net == pair->positive_net || zone.net == pair->negative_net) &&
                 !zone.fills.empty();
        });
    if (pad_count(pair->positive_net) != 2U || pad_count(pair->negative_net) != 2U ||
        path_count(pair->positive_net) != 1U || path_count(pair->negative_net) != 1U ||
        protected_routing_nets.contains(pair->positive_net) ||
        protected_routing_nets.contains(pair->negative_net) || physical_zone) {
      ++result.withheld_differential_pair_skew_tuning_pairs;
      continue;
    }
    const auto& shorter_net =
        positive_length < negative_length ? pair->positive_net : pair->negative_net;
    const auto shorter_length = std::min(positive_length, negative_length);
    const auto total_amplitudes = skew_tuning_total_amplitudes(difference, maximum_skew, config);
    std::vector<SkewTuningTrackCandidate> tracks;
    const auto lead =
        static_cast<std::uint64_t>(config.minimum_differential_pair_skew_tuning_lead.value());
    for (std::size_t index = 0U; index < board.tracks.size(); ++index) {
      const auto& track = board.tracks[index];
      if (track.net != shorter_net || track.locked || protected_track_ids.contains(track.id)) {
        continue;
      }
      const auto length = geometry::segment_length_nm(track.start, track.end);
      if (length <= lead * 2U) {
        continue;
      }
      tracks.push_back({index, length, track.id.value()});
    }
    std::sort(tracks.begin(), tracks.end(), [](const auto& lhs, const auto& rhs) {
      if (lhs.length_nm != rhs.length_nm) {
        return lhs.length_nm > rhs.length_nm;
      }
      return lhs.id < rhs.id;
    });

    bool accepted = false;
    for (const auto& selected : tracks) {
      const auto original = board.tracks[selected.index];
      const auto routing_rules = resolve_net_routing_rules(board, shorter_net);
      const auto configured_spacing =
          static_cast<std::uint64_t>(config.minimum_differential_pair_skew_tuning_spacing.value());
      const auto physical_spacing =
          static_cast<std::uint64_t>(original.width.value() + routing_rules.clearance.value());
      const auto spacing = std::max(configured_spacing, physical_spacing);
      const auto central_length = selected.length_nm - lead * 2U;
      const auto period_capacity =
          static_cast<std::size_t>((central_length + spacing) / (spacing * 2U));
      result.maximum_differential_pair_skew_tuning_period_capacity =
          std::max(result.maximum_differential_pair_skew_tuning_period_capacity, period_capacity);
      const auto maximum_periods =
          std::min(period_capacity, config.max_differential_pair_skew_tuning_periods);
      if (maximum_periods == 0U) {
        continue;
      }

      ++result.assessed_differential_pair_skew_tuning_regions;
      const bool any_direction =
          original.start.x != original.end.x && original.start.y != original.end.y;
      if (any_direction) {
        ++result.assessed_any_direction_differential_pair_skew_tuning_regions;
      }
      const auto free_space = extract_polygonal_board_free_space(
          board, BoardFreeSpaceExtractionConfig{original.layer, shorter_net, true, true});
      if (!free_space.complete()) {
        ++result.withheld_differential_pair_skew_tuning_regions;
        continue;
      }
      auto obstacle_clearance = board.rules.copper_to_edge_clearance;
      for (const auto& net : board.nets) {
        if (net.id != shorter_net) {
          obstacle_clearance =
              std::max(obstacle_clearance, resolve_pairwise_clearance(board, shorter_net, net.id));
        }
      }
      if (pair->minimum_gap) {
        obstacle_clearance = std::max(obstacle_clearance, *pair->minimum_gap);
      }
      const auto required_free_space =
          Nanometres{(original.width.value() + 1) / 2 + obstacle_clearance.value()};
      const auto maximum_period_amplitude = static_cast<std::uint64_t>(
          config.maximum_differential_pair_skew_tuning_amplitude.value());

      for (const auto total_amplitude : total_amplitudes) {
        const auto minimum_periods =
            static_cast<std::size_t>(total_amplitude / maximum_period_amplitude +
                                     (total_amplitude % maximum_period_amplitude == 0U ? 0U : 1U));
        if (minimum_periods == 0U || minimum_periods > maximum_periods) {
          continue;
        }
        for (auto periods = minimum_periods; periods <= maximum_periods; ++periods) {
          const auto amplitudes =
              distribute_tuning_amplitude(total_amplitude, periods, maximum_period_amplitude);
          if (amplitudes.empty()) {
            continue;
          }
          const auto layout_count = periods == 1U ? 2U : 4U;
          for (std::size_t layout = 0U; layout < layout_count; ++layout) {
            routing_kernel_checkpoint();
            if (stop_token.stop_requested()) {
              return SkewTuningStatus::cancelled;
            }
            if (result.attempted_differential_pair_skew_tuning_candidates >=
                config.max_differential_pair_skew_tuning_candidates) {
              result.differential_pair_skew_tuning_budget_exhausted = true;
              break;
            }
            ++result.attempted_differential_pair_skew_tuning_candidates;
            const auto segments =
                make_skew_tuning_pattern(original, lead, spacing, amplitudes, layout);
            if (segments.empty()) {
              if (any_direction) {
                ++result.withheld_differential_pair_skew_tuning_quantization_candidates;
              }
              continue;
            }
            const auto certificates = assess_free_space_chords(free_space.domain, segments);
            if (certificates.size() != segments.size() ||
                std::any_of(certificates.begin(), certificates.end(),
                            [required_free_space](const auto& certificate) {
                              return !certificate || *certificate < required_free_space;
                            })) {
              ++result.withheld_differential_pair_skew_tuning_free_space_candidates;
              continue;
            }

            Board proposal = board;
            auto proposal_ids = used_ids;
            proposal.tracks.erase(proposal.tracks.begin() +
                                  static_cast<std::ptrdiff_t>(selected.index));
            std::vector<TrackSegment> replacements;
            replacements.reserve(segments.size());
            for (std::size_t segment = 0U; segment < segments.size(); ++segment) {
              replacements.push_back(TrackSegment{
                  unique_id(original.id.value() + "-skew-" + std::to_string(segment), proposal_ids),
                  original.net, original.layer, segments[segment].first, segments[segment].second,
                  original.width, false});
            }
            proposal.tracks.insert(proposal.tracks.begin() +
                                       static_cast<std::ptrdiff_t>(selected.index),
                                   replacements.begin(), replacements.end());
            const auto tuned_positive =
                routed_net_centerline_length_nm(proposal, pair->positive_net);
            const auto tuned_negative =
                routed_net_centerline_length_nm(proposal, pair->negative_net);
            const auto tuned_shorter =
                shorter_net == pair->positive_net ? tuned_positive : tuned_negative;
            if (tuned_shorter <= shorter_length ||
                absolute_difference(tuned_positive, tuned_negative) > maximum_skew ||
                !skew_tuning_candidate_is_legal(proposal, *pair)) {
              continue;
            }
            board = std::move(proposal);
            used_ids = std::move(proposal_ids);
            result.added_tracks += replacements.size() - 1U;
            result.added_differential_pair_skew_tuning_length_nm += tuned_shorter - shorter_length;
            result.materialized_differential_pair_skew_tuning_periods += periods;
            if (any_direction) {
              ++result.materialized_any_direction_differential_pair_skew_tuning_patterns;
            }
            ++result.tuned_differential_pairs;
            accepted = true;
            break;
          }
          if (accepted || result.differential_pair_skew_tuning_budget_exhausted) {
            break;
          }
        }
        if (accepted || result.differential_pair_skew_tuning_budget_exhausted) {
          break;
        }
      }
      if (accepted || result.differential_pair_skew_tuning_budget_exhausted) {
        break;
      }
    }
    if (!accepted) {
      ++result.withheld_differential_pair_skew_tuning_pairs;
    }
    if (result.differential_pair_skew_tuning_budget_exhausted) {
      break;
    }
  }
  return SkewTuningStatus::completed;
}

struct SkewTuningAssignmentCandidate {
  DifferentialPairId pair;
  ObjectId original_track;
  std::vector<TrackSegment> replacements;
  std::uint64_t added_length_nm{0};
  std::size_t periods{0};
  bool any_direction{false};
};

struct SkewTuningAssignmentPair {
  const DifferentialPair* pair{nullptr};
  std::vector<SkewTuningAssignmentCandidate> candidates;
};

[[nodiscard]] bool
apply_skew_tuning_assignment_candidate(Board& board, std::set<std::string>& used_ids,
                                       const SkewTuningAssignmentCandidate& candidate) {
  const auto original =
      std::find_if(board.tracks.begin(), board.tracks.end(),
                   [&](const auto& track) { return track.id == candidate.original_track; });
  if (original == board.tracks.end()) {
    return false;
  }
  for (const auto& replacement : candidate.replacements) {
    if (!used_ids.insert(replacement.id.value()).second) {
      return false;
    }
  }
  const auto index = static_cast<std::size_t>(std::distance(board.tracks.begin(), original));
  board.tracks.erase(original);
  board.tracks.insert(board.tracks.begin() + static_cast<std::ptrdiff_t>(index),
                      candidate.replacements.begin(), candidate.replacements.end());
  return true;
}

[[nodiscard]] SkewTuningStatus tune_differential_pair_skew_batch(
    Board& board, const GlobalRouteResult& global, const std::set<ObjectId>& protected_track_ids,
    const std::set<NetId>& protected_routing_nets, std::set<std::string>& used_ids,
    const DetailedRouterConfig& config, DetailedRouteResult& result, std::stop_token stop_token) {
  std::vector<const DifferentialPair*> pairs;
  for (const auto& pair : board.differential_pairs) {
    if (pair.maximum_skew) {
      pairs.push_back(&pair);
    }
  }
  std::sort(pairs.begin(), pairs.end(),
            [](const auto* lhs, const auto* rhs) { return lhs->id < rhs->id; });

  std::vector<SkewTuningAssignmentPair> pair_candidates;
  for (const auto* pair : pairs) {
    routing_kernel_checkpoint();
    if (stop_token.stop_requested()) {
      return SkewTuningStatus::cancelled;
    }
    const auto positive_length = routed_net_centerline_length_nm(board, pair->positive_net);
    const auto negative_length = routed_net_centerline_length_nm(board, pair->negative_net);
    const auto difference = absolute_difference(positive_length, negative_length);
    const auto maximum_skew = static_cast<std::uint64_t>(pair->maximum_skew->value());
    if (difference <= maximum_skew) {
      continue;
    }
    ++result.attempted_differential_pair_skew_tuning_pairs;
    SkewTuningAssignmentPair generated{pair, {}};
    if (result.differential_pair_skew_tuning_budget_exhausted) {
      pair_candidates.push_back(std::move(generated));
      continue;
    }

    const auto pad_count = [&board](const NetId& net) {
      std::size_t count = 0U;
      for (const auto& footprint : board.footprints) {
        count += static_cast<std::size_t>(
            std::count_if(footprint.pads.begin(), footprint.pads.end(),
                          [&net](const auto& pad) { return pad.net == net; }));
      }
      return count;
    };
    const auto path_count = [&global](const NetId& net) {
      return static_cast<std::size_t>(
          std::count_if(global.paths.begin(), global.paths.end(),
                        [&net](const auto& path) { return path.net == net; }));
    };
    const bool physical_zone =
        std::any_of(board.zones.begin(), board.zones.end(), [&](const auto& zone) {
          return (zone.net == pair->positive_net || zone.net == pair->negative_net) &&
                 !zone.fills.empty();
        });
    if (pad_count(pair->positive_net) != 2U || pad_count(pair->negative_net) != 2U ||
        path_count(pair->positive_net) != 1U || path_count(pair->negative_net) != 1U ||
        protected_routing_nets.contains(pair->positive_net) ||
        protected_routing_nets.contains(pair->negative_net) || physical_zone) {
      pair_candidates.push_back(std::move(generated));
      continue;
    }

    const auto& shorter_net =
        positive_length < negative_length ? pair->positive_net : pair->negative_net;
    const auto shorter_length = std::min(positive_length, negative_length);
    const auto total_amplitudes = skew_tuning_total_amplitudes(difference, maximum_skew, config);
    std::vector<SkewTuningTrackCandidate> tracks;
    const auto lead =
        static_cast<std::uint64_t>(config.minimum_differential_pair_skew_tuning_lead.value());
    for (std::size_t index = 0U; index < board.tracks.size(); ++index) {
      const auto& track = board.tracks[index];
      if (track.net != shorter_net || track.locked || protected_track_ids.contains(track.id)) {
        continue;
      }
      const auto length = geometry::segment_length_nm(track.start, track.end);
      if (length > lead * 2U) {
        tracks.push_back({index, length, track.id.value()});
      }
    }
    std::sort(tracks.begin(), tracks.end(), [](const auto& lhs, const auto& rhs) {
      if (lhs.length_nm != rhs.length_nm) {
        return lhs.length_nm > rhs.length_nm;
      }
      return lhs.id < rhs.id;
    });

    for (const auto& selected : tracks) {
      if (generated.candidates.size() >=
              config.max_differential_pair_skew_tuning_assignment_candidates_per_pair ||
          result.differential_pair_skew_tuning_budget_exhausted) {
        break;
      }
      const auto original = board.tracks[selected.index];
      const auto routing_rules = resolve_net_routing_rules(board, shorter_net);
      const auto configured_spacing =
          static_cast<std::uint64_t>(config.minimum_differential_pair_skew_tuning_spacing.value());
      const auto physical_spacing =
          static_cast<std::uint64_t>(original.width.value() + routing_rules.clearance.value());
      const auto spacing = std::max(configured_spacing, physical_spacing);
      const auto central_length = selected.length_nm - lead * 2U;
      const auto period_capacity =
          static_cast<std::size_t>((central_length + spacing) / (spacing * 2U));
      result.maximum_differential_pair_skew_tuning_period_capacity =
          std::max(result.maximum_differential_pair_skew_tuning_period_capacity, period_capacity);
      const auto maximum_periods =
          std::min(period_capacity, config.max_differential_pair_skew_tuning_periods);
      if (maximum_periods == 0U) {
        continue;
      }

      ++result.assessed_differential_pair_skew_tuning_regions;
      const bool any_direction =
          original.start.x != original.end.x && original.start.y != original.end.y;
      if (any_direction) {
        ++result.assessed_any_direction_differential_pair_skew_tuning_regions;
      }
      const auto free_space = extract_polygonal_board_free_space(
          board, BoardFreeSpaceExtractionConfig{original.layer, shorter_net, true, true});
      if (!free_space.complete()) {
        ++result.withheld_differential_pair_skew_tuning_regions;
        continue;
      }
      auto obstacle_clearance = board.rules.copper_to_edge_clearance;
      for (const auto& net : board.nets) {
        if (net.id != shorter_net) {
          obstacle_clearance =
              std::max(obstacle_clearance, resolve_pairwise_clearance(board, shorter_net, net.id));
        }
      }
      if (pair->minimum_gap) {
        obstacle_clearance = std::max(obstacle_clearance, *pair->minimum_gap);
      }
      const auto required_free_space =
          Nanometres{(original.width.value() + 1) / 2 + obstacle_clearance.value()};
      const auto maximum_period_amplitude = static_cast<std::uint64_t>(
          config.maximum_differential_pair_skew_tuning_amplitude.value());

      for (const auto total_amplitude : total_amplitudes) {
        if (generated.candidates.size() >=
                config.max_differential_pair_skew_tuning_assignment_candidates_per_pair ||
            result.differential_pair_skew_tuning_budget_exhausted) {
          break;
        }
        const auto minimum_periods =
            static_cast<std::size_t>(total_amplitude / maximum_period_amplitude +
                                     (total_amplitude % maximum_period_amplitude == 0U ? 0U : 1U));
        if (minimum_periods == 0U || minimum_periods > maximum_periods) {
          continue;
        }
        for (auto periods = minimum_periods; periods <= maximum_periods; ++periods) {
          if (generated.candidates.size() >=
                  config.max_differential_pair_skew_tuning_assignment_candidates_per_pair ||
              result.differential_pair_skew_tuning_budget_exhausted) {
            break;
          }
          const auto amplitudes =
              distribute_tuning_amplitude(total_amplitude, periods, maximum_period_amplitude);
          if (amplitudes.empty()) {
            continue;
          }
          const auto layout_count = periods == 1U ? 2U : 4U;
          for (std::size_t layout = 0U; layout < layout_count; ++layout) {
            routing_kernel_checkpoint();
            if (stop_token.stop_requested()) {
              return SkewTuningStatus::cancelled;
            }
            if (generated.candidates.size() >=
                config.max_differential_pair_skew_tuning_assignment_candidates_per_pair) {
              break;
            }
            if (result.attempted_differential_pair_skew_tuning_candidates >=
                config.max_differential_pair_skew_tuning_candidates) {
              result.differential_pair_skew_tuning_budget_exhausted = true;
              break;
            }
            ++result.attempted_differential_pair_skew_tuning_candidates;
            const auto segments =
                make_skew_tuning_pattern(original, lead, spacing, amplitudes, layout);
            if (segments.empty()) {
              if (any_direction) {
                ++result.withheld_differential_pair_skew_tuning_quantization_candidates;
              }
              continue;
            }
            const auto certificates = assess_free_space_chords(free_space.domain, segments);
            if (certificates.size() != segments.size() ||
                std::any_of(certificates.begin(), certificates.end(),
                            [required_free_space](const auto& certificate) {
                              return !certificate || *certificate < required_free_space;
                            })) {
              ++result.withheld_differential_pair_skew_tuning_free_space_candidates;
              continue;
            }

            Board proposal = board;
            auto proposal_ids = used_ids;
            proposal.tracks.erase(proposal.tracks.begin() +
                                  static_cast<std::ptrdiff_t>(selected.index));
            std::vector<TrackSegment> replacements;
            replacements.reserve(segments.size());
            for (std::size_t segment = 0U; segment < segments.size(); ++segment) {
              replacements.push_back(TrackSegment{
                  unique_id(original.id.value() + "-skew-" + std::to_string(segment), proposal_ids),
                  original.net, original.layer, segments[segment].first, segments[segment].second,
                  original.width, false});
            }
            proposal.tracks.insert(proposal.tracks.begin() +
                                       static_cast<std::ptrdiff_t>(selected.index),
                                   replacements.begin(), replacements.end());
            const auto tuned_positive =
                routed_net_centerline_length_nm(proposal, pair->positive_net);
            const auto tuned_negative =
                routed_net_centerline_length_nm(proposal, pair->negative_net);
            const auto tuned_shorter =
                shorter_net == pair->positive_net ? tuned_positive : tuned_negative;
            if (tuned_shorter <= shorter_length ||
                absolute_difference(tuned_positive, tuned_negative) > maximum_skew ||
                !skew_tuning_candidate_is_legal(proposal, *pair)) {
              continue;
            }
            generated.candidates.push_back(SkewTuningAssignmentCandidate{
                pair->id, original.id, std::move(replacements), tuned_shorter - shorter_length,
                periods, any_direction});
            ++result.legal_differential_pair_skew_tuning_candidates;
          }
        }
      }
    }
    pair_candidates.push_back(std::move(generated));
  }

  std::vector<const SkewTuningAssignmentPair*> assignable;
  for (const auto& generated : pair_candidates) {
    if (!generated.candidates.empty()) {
      assignable.push_back(&generated);
    }
  }
  result.differential_pair_skew_tuning_assignment_attempted =
      result.attempted_differential_pair_skew_tuning_pairs > 0U;

  Board best_board = board;
  auto best_ids = used_ids;
  std::vector<const SkewTuningAssignmentCandidate*> best_selection;
  std::vector<const SkewTuningAssignmentCandidate*> selection;
  std::set<std::string> selected_pair_ids;
  bool cancelled = false;

  RoutingComputeAssignmentInput skew_compute;
  skew_compute.maximum_states = config.max_differential_pair_skew_tuning_assignment_states;
  skew_compute.top_k = config.max_differential_pair_skew_tuning_assignment_states;
  std::vector<const SkewTuningAssignmentCandidate*> skew_option;
  for (const auto* generated : assignable) {
    auto& domain = skew_compute.variable_domains.emplace_back();
    for (const auto& candidate : generated->candidates) {
      domain.push_back(static_cast<std::uint32_t>(skew_option.size()));
      skew_option.push_back(&candidate);
      skew_compute.option_costs.push_back(0U);
    }
    domain.push_back(static_cast<std::uint32_t>(skew_option.size()));
    skew_option.push_back(nullptr);
    // Skipping a tunable pair is legal, but ranks after every complete compatible assignment.
    skew_compute.option_costs.push_back(1U);
  }
  for (std::size_t first = 0U; first < skew_option.size(); ++first) {
    if (skew_option[first] == nullptr)
      continue;
    for (std::size_t second = first + 1U; second < skew_option.size(); ++second) {
      if (skew_option[second] == nullptr || skew_option[first]->pair == skew_option[second]->pair)
        continue;
      Board probe = board;
      auto probe_ids = used_ids;
      if (!apply_skew_tuning_assignment_candidate(probe, probe_ids, *skew_option[first]) ||
          !apply_skew_tuning_assignment_candidate(probe, probe_ids, *skew_option[second]) ||
          !skew_tuning_selection_is_legal(
              probe, {skew_option[first]->pair.value(), skew_option[second]->pair.value()}))
        skew_compute.incompatible_options.emplace_back(static_cast<std::uint32_t>(first),
                                                       static_cast<std::uint32_t>(second));
    }
  }
  const auto accelerated_skew = skew_compute.variable_domains.empty()
                                    ? RoutingComputeAssignmentResult{}
                                    : evaluate_routing_assignments(skew_compute, true);
  if (accelerated_skew.ok()) {
    for (const auto& assignment : accelerated_skew.shortlist) {
      Board branch = board;
      auto branch_ids = used_ids;
      std::vector<const SkewTuningAssignmentCandidate*> selected;
      std::set<std::string> pair_ids;
      bool legal = true;
      for (const auto option : assignment.options) {
        if (option >= skew_option.size() || skew_option[option] == nullptr)
          continue;
        legal = apply_skew_tuning_assignment_candidate(branch, branch_ids, *skew_option[option]);
        if (!legal)
          break;
        selected.push_back(skew_option[option]);
        pair_ids.insert(skew_option[option]->pair.value());
      }
      ++result.differential_pair_skew_tuning_assignment_states;
      if (!legal || !skew_tuning_selection_is_legal(branch, pair_ids)) {
        ++result.rejected_differential_pair_skew_tuning_assignment_conflicts;
        continue;
      }
      if (selected.size() > best_selection.size()) {
        best_board = std::move(branch);
        best_ids = std::move(branch_ids);
        best_selection = std::move(selected);
      }
    }
  }

  const auto search = [&](const auto& self, std::size_t pair_index, const Board& partial,
                          const std::set<std::string>& partial_ids) -> void {
    if (cancelled || result.differential_pair_skew_tuning_assignment_budget_exhausted) {
      return;
    }
    routing_kernel_checkpoint();
    if (stop_token.stop_requested()) {
      cancelled = true;
      return;
    }
    if (selection.size() + assignable.size() - pair_index <= best_selection.size()) {
      return;
    }
    if (pair_index >= assignable.size()) {
      return;
    }

    const auto* generated = assignable[pair_index];
    for (const auto& candidate : generated->candidates) {
      if (result.differential_pair_skew_tuning_assignment_states >=
          config.max_differential_pair_skew_tuning_assignment_states) {
        result.differential_pair_skew_tuning_assignment_budget_exhausted = true;
        return;
      }
      ++result.differential_pair_skew_tuning_assignment_states;
      Board branch = partial;
      auto branch_ids = partial_ids;
      if (!apply_skew_tuning_assignment_candidate(branch, branch_ids, candidate)) {
        ++result.rejected_differential_pair_skew_tuning_assignment_conflicts;
        continue;
      }
      selection.push_back(&candidate);
      selected_pair_ids.insert(candidate.pair.value());
      if (!skew_tuning_selection_is_legal(branch, selected_pair_ids)) {
        ++result.rejected_differential_pair_skew_tuning_assignment_conflicts;
        selected_pair_ids.erase(candidate.pair.value());
        selection.pop_back();
        continue;
      }
      if (selection.size() > best_selection.size()) {
        best_board = branch;
        best_ids = branch_ids;
        best_selection = selection;
      }
      self(self, pair_index + 1U, branch, branch_ids);
      selected_pair_ids.erase(candidate.pair.value());
      selection.pop_back();
      if (cancelled || result.differential_pair_skew_tuning_assignment_budget_exhausted) {
        return;
      }
    }

    if (result.differential_pair_skew_tuning_assignment_states >=
        config.max_differential_pair_skew_tuning_assignment_states) {
      result.differential_pair_skew_tuning_assignment_budget_exhausted = true;
      return;
    }
    ++result.differential_pair_skew_tuning_assignment_states;
    self(self, pair_index + 1U, partial, partial_ids);
  };
  if (!accelerated_skew.ok() || accelerated_skew.state_space_truncated)
    search(search, 0U, board, used_ids);
  if (cancelled) {
    return SkewTuningStatus::cancelled;
  }

  board = std::move(best_board);
  used_ids = std::move(best_ids);
  for (const auto* candidate : best_selection) {
    result.added_tracks += candidate->replacements.size() - 1U;
    result.added_differential_pair_skew_tuning_length_nm += candidate->added_length_nm;
    result.materialized_differential_pair_skew_tuning_periods += candidate->periods;
    if (candidate->any_direction) {
      ++result.materialized_any_direction_differential_pair_skew_tuning_patterns;
    }
  }
  result.tuned_differential_pairs += best_selection.size();
  result.withheld_differential_pair_skew_tuning_pairs +=
      result.attempted_differential_pair_skew_tuning_pairs - best_selection.size();
  return SkewTuningStatus::completed;
}

// Preserve electrical contacts before replacing off-angle terminal/medial adapters. Proposals
// are bounded (two shortest octilinear elbows, then two Manhattan elbows), and exact copper
// checks own acceptance. The ordinary final connectivity/DRC transaction still checks the result.
void constrain_straight_track_angles(Board& board, const std::set<ObjectId>& protected_ids,
                                     std::set<std::string>& used_ids, DetailedRouteResult& result,
                                     std::stop_token stop_token) {
  const auto needs_conversion = [&](const auto& track) {
    return !track.locked && !protected_ids.contains(track.id) &&
           !ordinary_track_angle(track.start, track.end);
  };
  if (std::none_of(board.tracks.begin(), board.tracks.end(), needs_conversion)) return;
  std::set<std::string> protected_strings;
  for (const auto& id : protected_ids) protected_strings.insert(id.value());
  for (const auto& track : board.tracks) {
    if (ordinary_track_angle(track.start, track.end)) protected_strings.insert(track.id.value());
  }
  result.added_tracks += split_tracks_at_exact_same_net_branches(board, used_ids, protected_strings, stop_token);
  const auto count = board.tracks.size();
  for (std::size_t index = 0U; index < count; ++index) {
    if (stop_token.stop_requested()) return;
    const auto track = board.tracks[index];
    if (!needs_conversion(track)) continue;
    const auto dx = track.end.x.value() - track.start.x.value();
    const auto dy = track.end.y.value() - track.start.y.value();
    const auto diagonal = std::min(magnitude(dx), magnitude(dy));
    const std::array<Point, 4> elbows{
        Point{Nanometres{track.start.x.value() + direction(dx) * diagonal},
              Nanometres{track.start.y.value() + direction(dy) * diagonal}},
        Point{Nanometres{track.end.x.value() - direction(dx) * diagonal},
              Nanometres{track.end.y.value() - direction(dy) * diagonal}},
        Point{track.start.x, track.end.y}, Point{track.end.x, track.start.y}};
    bool converted = false;
    for (const auto elbow : elbows) {
      if (stop_token.stop_requested()) return;
      ++result.straight_angle_candidate_checks;
      if (!continuous_candidate_clear(board, track.net, track.layer, track.start, elbow,
                                      track.end, track.width,
                                      result.straight_angle_candidate_checks)) continue;
      auto second = track;
      second.id = unique_id(track.id.value() + "-45-adapter", used_ids);
      second.start = elbow;
      board.tracks[index].end = elbow;
      board.tracks.push_back(std::move(second));
      ++result.added_tracks;
      ++result.straight_angle_segments_converted;
      converted = true;
      break;
    }
    if (!converted) ++result.straight_angle_segments_unresolved;
  }
}

// One final acceptance gate is shared by ordinary construction and exact regional-scene reuse.
// Reusing geometry never reuses authority: validation, connectivity and DRC run again.
void validate_materialized_candidate(const Board& source, const GlobalRouteResult& global,
                                     const DetailedRouterConfig& config, Board candidate,
                                     DetailedRouteResult& result, bool bounded_conflict_handoff,
                                     const DetailedRouteResult* checkpoint) {
  result.validation = validate_board(candidate);
  result.connectivity = analyze_connectivity(candidate);
  if (!global.configured_selected_nets.empty()) {
    const std::set<NetId> accepted_nets(global.configured_selected_nets.begin(),
                                        global.configured_selected_nets.end());
    std::erase_if(result.connectivity.nets,
                  [&accepted_nets](const auto& net) { return !accepted_nets.contains(net.net); });
    result.connectivity.open_connections = 0U;
    result.connectivity.unrouted_nets = 0U;
    for (const auto& net : result.connectivity.nets) {
      result.connectivity.open_connections += net.open_connections;
      result.connectivity.unrouted_nets += net.open_connections == 0U ? 0U : 1U;
    }
  }
  if (config.defer_saved_zone_fill_clearance_to_refill) {
    auto refill_planning_candidate = candidate;
    for (auto& zone : refill_planning_candidate.zones) {
      zone.fills.clear();
    }
    result.copper_drc = check_copper_rules(refill_planning_candidate);
    std::set<std::string> source_copper_ids;
    for (const auto& track : source.tracks) {
      source_copper_ids.insert(track.id.value());
    }
    for (const auto& arc : source.track_arcs) {
      source_copper_ids.insert(arc.id.value());
    }
    for (const auto& bezier : source.track_beziers) {
      source_copper_ids.insert(bezier.id.value());
    }
    for (const auto& via : source.vias) {
      source_copper_ids.insert(via.id.value());
    }
    for (const auto& footprint : source.footprints) {
      for (const auto& pad : footprint.pads) {
        source_copper_ids.insert(pad.id.value());
      }
    }
    std::erase_if(result.copper_drc.violations, [&source_copper_ids](const auto& violation) {
      return std::all_of(
          violation.object_ids.begin(), violation.object_ids.end(),
          [&source_copper_ids](const auto& id) { return source_copper_ids.contains(id); });
    });
  } else {
    result.copper_drc = check_copper_rules(candidate);
  }
  const auto unrounded = count_unrounded_non_junction_bends(candidate);
  result.unrounded_non_junction_bends = unrounded.total();
  result.unrounded_straight_track_bends = unrounded.straight_track;
  result.unrounded_curve_tangent_bends = unrounded.curve_tangent;
  result.candidate = std::move(candidate);
  if (!result.validation.ok()) {
    result.stop_reason = DetailedRouterStopReason::structural_violations;
    result.diagnostic = "materialized route failed structural validation";
  } else if (result.connectivity.open_connections != 0U) {
    result.stop_reason = DetailedRouterStopReason::connectivity_incomplete;
    result.diagnostic = "materialized route did not connect every multi-pad net";
  } else if (!result.copper_drc.ok()) {
    result.stop_reason = DetailedRouterStopReason::copper_violations;
    result.diagnostic = bounded_conflict_handoff
                            ? "bounded residual spline/PNS repair did not clear every represented "
                              "copper conflict"
                            : "materialized route violates represented copper rules";
  } else if (result.straight_angle_segments_unresolved > 0U) {
    result.stop_reason = DetailedRouterStopReason::invalid_path;
    result.diagnostic = "ordinary straight-track angle conversion has no legal bounded adapter";
  } else if (config.circular_arc_refinement && result.unrounded_non_junction_bends > 0U) {
    result.stop_reason = DetailedRouterStopReason::invalid_path;
    result.diagnostic = "materialized curved route retains " +
                        std::to_string(result.unrounded_non_junction_bends) +
                        " non-junction direction changes without quintic PH curve realization";
  } else if (config.defer_saved_zone_fill_clearance_to_refill) {
    result.stop_reason = DetailedRouterStopReason::zone_refill_required;
    result.diagnostic =
        "provisional copper is legal without saved zone fills; KiCad zone refill, strict "
        "re-import, connectivity, and DRC are required before acceptance";
  } else {
    result.stop_reason = DetailedRouterStopReason::complete;
    if (result.tuned_differential_pairs > 0U) {
      result.diagnostic = "differential-pair skew tuned as connected DRC-clean polyline copper";
    } else if (result.fitted_circular_arcs > 0U && result.materialized_topology_locked_paths > 0U) {
      result.diagnostic =
          "topology-locked medial lanes and quintic PH curve fitted arcs materialized as DRC-clean "
          "copper";
    } else if (result.fitted_circular_arcs > 0U) {
      result.diagnostic =
          "quintic PH curve routes materialized as connected DRC-clean native line/arc copper";
    } else if (result.materialized_topology_locked_paths > 0U) {
      result.diagnostic = "topology-locked medial lanes materialized as connected DRC-clean copper";
    } else {
      result.diagnostic = checkpoint == nullptr
                              ? "global paths materialized as legal simplified copper"
                              : "affected paths materialized with validated detailed copper reused";
    }
  }
}

DetailedRouteResult
materialize_impl(const Board& source, const GlobalRouteResult& global,
                 const DetailedRouterConfig& config, const DetailedRouteResult* checkpoint,
                 std::stop_token stop_token, bool use_topology_locked,
                 bool use_circular_arc_refinement, bool use_pull_tight,
                 bool use_continuous_refinement, const std::set<NetId>& topology_excluded_nets = {},
                 const std::vector<RegionalSplineRealization>* regional_splines = nullptr,
                 const Board* validated_regional_scene = nullptr) {
  DetailedRouteResult result;
  if (config.restrict_straight_track_angles && config.circular_arc_refinement) {
    result.stop_reason = DetailedRouterStopReason::invalid_input;
    result.diagnostic = "ordinary straight-track angles and curved refinement are incompatible";
    return result;
  }
  routing_kernel_checkpoint();
  if (stop_token.stop_requested()) {
    result.stop_reason = DetailedRouterStopReason::cancelled;
    result.diagnostic = "detailed routing cancelled before materialization";
    return result;
  }
  const auto source_validation = validate_board(source);
  if (!source_validation.ok() && !config.allow_invalid_source_for_diagnostic_preview) {
    result.validation = source_validation;
    result.diagnostic = "source board failed structural validation";
    return result;
  }
  const bool source_requires_zone_refill =
      std::any_of(source.zones.begin(), source.zones.end(),
                  [](const CopperZone& zone) { return zone.requires_refill; });
  if (source_requires_zone_refill && !config.defer_saved_zone_fill_clearance_to_refill) {
    result.stop_reason = DetailedRouterStopReason::invalid_input;
    result.validation = source_validation;
    result.diagnostic =
        "source contains an unfilled zone; use the explicit provisional zone-refill workflow";
    return result;
  }
  std::size_t residual_nets = 0U;
  const auto bounded_conflict_handoff =
      config.bounded_residual_conflict_refinement && global.total_overflow > 0U &&
      global.unrouted_nets.empty() && !global.paths.empty() &&
      !global.residual_conflict_groups.empty() &&
      global.residual_conflict_groups.size() <= config.max_residual_conflict_groups &&
      global.sampled_overflow <= config.max_residual_sampled_overflow &&
      global.clearance_shadow_overflow <= config.max_residual_clearance_shadow_overflow &&
      std::all_of(global.residual_conflict_groups.begin(), global.residual_conflict_groups.end(),
                  [&](const auto& group) {
                    residual_nets += group.nets.size();
                    return !group.nets.empty() &&
                           group.nets.size() <= config.max_residual_conflict_nets_per_group &&
                           group.all_nets_have_preserved_geometric_corridors;
                  });
  if ((!global.complete() || global.total_overflow != 0U) && !bounded_conflict_handoff) {
    result.stop_reason = DetailedRouterStopReason::incomplete_global_route;
    result.diagnostic =
        "global route must be overflow-free or expose only bounded topology-preserved residual "
        "conflict groups";
    return result;
  }

  if (config.track_width.value() < 0 || config.via_diameter.value() < 0 ||
      config.via_drill.value() < 0 || config.continuous_trust_region.value() < 0 ||
      (config.existing_routing == ExistingRoutingPolicy::preserve &&
       config.existing_routing_replacement_scope.has_value()) ||
      config.maximum_circular_arc_radius.value() < 0 ||
      (config.circular_arc_refinement &&
       (config.maximum_circular_arc_radius.value() == 0 || config.max_circular_arcs == 0U ||
        config.max_circular_arc_candidate_checks == 0U ||
        config.spline_tessellation_tolerance.value() <= 0 ||
        config.spline_arc_fit_tolerance.value() < 0 ||
        config.spline_control_corridor.value() <= 0 || config.spline_shove_step.value() <= 0 ||
        config.spline_navmesh_cell_pitch.value() <= 0 ||
        config.spline_minimum_bend_radius.value() < 0 ||
        config.max_spline_navmesh_cells_per_run == 0U ||
        config.max_spline_navmesh_search_expansions_per_run == 0U ||
        config.spline_trajectory_samples < 3U || config.max_spline_trajectory_iterations == 0U ||
        config.max_spline_trajectory_candidate_checks < 4U || config.spline_route_workers == 0U ||
        config.spline_max_tessellation_depth == 0U || config.max_spline_samples_per_run < 3U ||
        config.max_spline_fitted_pieces_per_run == 0U || config.max_spline_arc_samples < 3U ||
        config.max_spline_shove_iterations == 0U || config.max_spline_candidate_checks == 0U ||
        config.max_pns_propagation_depth == 0U || config.max_pns_candidate_checks == 0U ||
        config.maximum_pns_displacement < config.spline_shove_step)) ||
      (config.differential_pair_skew_tuning_batch_assignment &&
       !config.differential_pair_skew_tuning) ||
      (config.bounded_residual_conflict_refinement &&
       (config.max_residual_conflict_groups == 0U ||
        config.max_residual_conflict_nets_per_group == 0U ||
        config.max_residual_sampled_overflow == 0U ||
        config.max_residual_clearance_shadow_overflow == 0U ||
        config.max_residual_joint_candidate_states == 0U ||
        config.max_residual_joint_optimization_sweeps == 0U)) ||
      (config.differential_pair_skew_tuning &&
       (config.differential_pair_skew_tuning_step.value() <= 0 ||
        config.differential_pair_skew_tuning_step.value() > max_abs_coordinate_nm ||
        config.maximum_differential_pair_skew_tuning_amplitude.value() <= 0 ||
        config.minimum_differential_pair_skew_tuning_lead.value() <= 0 ||
        config.minimum_differential_pair_skew_tuning_spacing.value() <= 0 ||
        config.maximum_differential_pair_skew_tuning_amplitude.value() > max_abs_coordinate_nm ||
        config.minimum_differential_pair_skew_tuning_lead.value() > max_abs_coordinate_nm ||
        config.minimum_differential_pair_skew_tuning_spacing.value() > max_abs_coordinate_nm ||
        config.max_differential_pair_skew_tuning_candidates == 0U ||
        config.max_differential_pair_skew_tuning_periods == 0U ||
        (config.differential_pair_skew_tuning_batch_assignment &&
         (config.max_differential_pair_skew_tuning_assignment_candidates_per_pair == 0U ||
          config.max_differential_pair_skew_tuning_assignment_states == 0U)))) ||
      (config.topology_locked_medial_materialization &&
       (config.max_topology_locked_terminal_access_points == 0U ||
        config.max_coupled_terminal_fanout_candidates == 0U ||
        config.max_shared_resource_fanout_candidates == 0U ||
        config.max_topology_locked_subset_repair_candidates == 0U)) ||
      (config.defer_saved_zone_fill_clearance_to_refill &&
       (config.continuous_refinement || config.topology_locked_medial_materialization ||
        config.differential_pair_skew_tuning))) {
    result.diagnostic = "detailed routing dimensions are invalid";
    return result;
  }
  result.materialized_config = config;
  result.residual_conflict_handoff_attempted = bounded_conflict_handoff;
  if (bounded_conflict_handoff) {
    result.residual_conflict_groups_attempted = global.residual_conflict_groups.size();
    result.residual_conflict_nets_attempted = residual_nets;
    result.residual_sampled_overflow_attempted = global.sampled_overflow;
    result.residual_clearance_shadow_overflow_attempted = global.clearance_shadow_overflow;
  }

  std::set<NetId> routed_nets;
  for (const auto& path : global.paths) {
    routed_nets.insert(path.net);
  }
  std::set<NetId> replacement_nets = routed_nets;
  if (config.existing_routing_replacement_scope) {
    replacement_nets.clear();
    for (const auto& net : *config.existing_routing_replacement_scope) {
      const auto source_net =
          std::find_if(source.nets.begin(), source.nets.end(), [&net](const Net& candidate) {
            return candidate.id == net && !candidate.name.empty();
          });
      if (source_net == source.nets.end() || !routed_nets.contains(net) ||
          !replacement_nets.insert(net).second) {
        result.stop_reason = DetailedRouterStopReason::invalid_input;
        result.diagnostic =
            "existing-routing replacement scope must contain unique named routed nets";
        return result;
      }
    }
  }
  const auto coupled_groups = coupled_routing_groups(source, routed_nets);
  std::map<NetId, ResolvedNetRoutingRules> dimensions;
  for (const auto& net : routed_nets) {
    const auto resolved = materialization_rules(source, net, config);
    if (resolved.track_width.value() <= 0 || resolved.via_diameter.value() <= 0 ||
        resolved.via_drill.value() < 0 || resolved.via_drill > resolved.via_diameter) {
      result.diagnostic = "resolved detailed routing dimensions are invalid";
      return result;
    }
    dimensions.emplace(net, resolved);
  }
  if (validated_regional_scene != nullptr) {
    // The caller has matched complete source/route identity. Recheck dimensions/policy above,
    // and the resulting copper below; no geometry regeneration or optimizer can alter it.
    Board candidate = *validated_regional_scene;
    result.reused_validated_regional_scene = true;
    result.added_tracks = candidate.tracks.size() - source.tracks.size();
    result.added_track_arcs = candidate.track_arcs.size() - source.track_arcs.size();
    result.added_track_beziers = candidate.track_beziers.size() - source.track_beziers.size();
    result.added_vias = candidate.vias.size() - source.vias.size();
    if (config.restrict_straight_track_angles) {
      for (std::size_t index = source.tracks.size(); index < candidate.tracks.size(); ++index) {
        const auto& track = candidate.tracks[index];
        const auto dx = track.end.x.value() - track.start.x.value();
        const auto dy = track.end.y.value() - track.start.y.value();
        if (dx != 0 && dy != 0 && std::abs(dx) != std::abs(dy))
          ++result.straight_angle_segments_unresolved;
      }
      result.straight_angle_segments_unresolved += result.added_track_arcs;
    }
    routing_kernel_checkpoint();
    if (stop_token.stop_requested()) {
      result.stop_reason = DetailedRouterStopReason::cancelled;
      result.diagnostic = "detailed routing cancelled before regional scene validation";
      return result;
    }
    validate_materialized_candidate(source, global, config, std::move(candidate), result,
                                    false, nullptr);
    if (result.complete())
      result.diagnostic = "exact regional copper reused with final connectivity and DRC verified";
    return result;
  }
  std::map<NetId, std::set<Nanometres>> allowed_track_widths;
  for (const auto& net : routed_nets) {
    allowed_track_widths[net].insert(dimensions.at(net).track_width);
  }
  for (const auto& path : global.paths) {
    for (const auto width : path.segment_widths) {
      if (width.value() > 0 && width < dimensions.at(path.net).track_width) {
        allowed_track_widths[path.net].insert(width);
      }
    }
  }
  std::set<NetId> reusable_nets;
  std::set<NetId> invalidated_reuse_nets;
  if (checkpoint != nullptr) {
    reusable_nets.insert(global.reused_nets.begin(), global.reused_nets.end());
    if (!checkpoint->candidate || !checkpoint->validation.ok() ||
        checkpoint->materialized_config != result.materialized_config) {
      invalidated_reuse_nets = reusable_nets;
      reusable_nets.clear();
    } else {
      std::map<std::string, NetId> object_nets;
      for (const auto& footprint : checkpoint->candidate->footprints) {
        for (const auto& pad : footprint.pads) {
          object_nets.emplace(pad.id.value(), pad.net);
        }
      }
      for (const auto& track : checkpoint->candidate->tracks) {
        object_nets.emplace(track.id.value(), track.net);
      }
      for (const auto& arc : checkpoint->candidate->track_arcs) {
        object_nets.emplace(arc.id.value(), arc.net);
      }
      for (const auto& bezier : checkpoint->candidate->track_beziers) {
        object_nets.emplace(bezier.id.value(), bezier.net);
      }
      for (const auto& via : checkpoint->candidate->vias) {
        object_nets.emplace(via.id.value(), via.net);
      }
      for (const auto& zone : checkpoint->candidate->zones) {
        object_nets.emplace(zone.id.value(), zone.net);
      }
      for (const auto& connectivity : checkpoint->connectivity.nets) {
        if (connectivity.open_connections != 0U && reusable_nets.contains(connectivity.net)) {
          invalidated_reuse_nets.insert(connectivity.net);
        }
      }
      for (const auto& violation : checkpoint->copper_drc.violations) {
        for (const auto& object_id : violation.object_ids) {
          if (const auto found = object_nets.find(object_id);
              found != object_nets.end() && reusable_nets.contains(found->second)) {
            invalidated_reuse_nets.insert(found->second);
          }
        }
      }
      std::map<ObjectId, const TrackSegment*> source_tracks;
      std::map<ObjectId, const TrackArc*> source_arcs;
      std::map<ObjectId, const TrackBezier*> source_beziers;
      std::map<ObjectId, const Via*> source_vias;
      for (const auto& track : source.tracks) {
        source_tracks.emplace(track.id, &track);
      }
      for (const auto& via : source.vias) {
        source_vias.emplace(via.id, &via);
      }
      for (const auto& arc : source.track_arcs) {
        source_arcs.emplace(arc.id, &arc);
      }
      for (const auto& curve : source.track_beziers) source_beziers.emplace(curve.id, &curve);
      for (const auto& track : checkpoint->candidate->tracks) {
        if (!reusable_nets.contains(track.net)) {
          continue;
        }
        const auto source_track = source_tracks.find(track.id);
        if ((source_track == source_tracks.end() &&
             !allowed_track_widths.at(track.net).contains(track.width)) ||
            (source_track != source_tracks.end() && *source_track->second != track)) {
          invalidated_reuse_nets.insert(track.net);
        }
      }
      for (const auto& via : checkpoint->candidate->vias) {
        if (!reusable_nets.contains(via.net)) {
          continue;
        }
        const auto source_via = source_vias.find(via.id);
        if ((source_via == source_vias.end() &&
             (via.diameter != dimensions.at(via.net).via_diameter ||
              via.drill != dimensions.at(via.net).via_drill)) ||
            (source_via != source_vias.end() && *source_via->second != via)) {
          invalidated_reuse_nets.insert(via.net);
        }
      }
      for (const auto& arc : checkpoint->candidate->track_arcs) {
        if (!reusable_nets.contains(arc.net)) {
          continue;
        }
        const auto source_arc = source_arcs.find(arc.id);
        if ((source_arc == source_arcs.end() &&
             !allowed_track_widths.at(arc.net).contains(arc.width)) ||
            (source_arc != source_arcs.end() && *source_arc->second != arc)) {
          invalidated_reuse_nets.insert(arc.net);
        }
      }
      for (const auto& curve : checkpoint->candidate->track_beziers) {
        if (!reusable_nets.contains(curve.net)) continue;
        const auto original = source_beziers.find(curve.id);
        if ((original == source_beziers.end() &&
             !allowed_track_widths.at(curve.net).contains(curve.width)) ||
            (original != source_beziers.end() && *original->second != curve))
          invalidated_reuse_nets.insert(curve.net);
      }
      for (const auto& net : invalidated_reuse_nets) {
        reusable_nets.erase(net);
      }
    }
  }
  for (const auto& group : coupled_groups) {
    const auto reusable_members = static_cast<std::size_t>(
        std::count_if(group.members.begin(), group.members.end(),
                      [&reusable_nets](const auto& net) { return reusable_nets.contains(net); }));
    if (reusable_members == 0U || reusable_members == group.members.size()) {
      continue;
    }
    for (const auto& net : group.members) {
      if (reusable_nets.erase(net) > 0U) {
        invalidated_reuse_nets.insert(net);
      }
    }
  }
  result.reused_nets.assign(reusable_nets.begin(), reusable_nets.end());
  result.invalidated_reuse_nets.assign(invalidated_reuse_nets.begin(),
                                       invalidated_reuse_nets.end());
  for (const auto& net : routed_nets) {
    if (!reusable_nets.contains(net)) {
      result.rematerialized_nets.push_back(net);
    }
  }

  Board candidate = source;
  for (const auto& proposal : global.selected_shove_proposals) {
    std::set<ObjectId> replaced(proposal.replaced_tracks.begin(), proposal.replaced_tracks.end());
    const auto found = static_cast<std::size_t>(
        std::count_if(candidate.tracks.begin(), candidate.tracks.end(),
                      [&](const auto& track) { return replaced.contains(track.id); }));
    if (found != replaced.size()) {
      result.diagnostic = "global PNS proposal no longer binds every replaced source track";
      return result;
    }
    std::erase_if(candidate.tracks, [&](const auto& track) { return replaced.contains(track.id); });
    candidate.tracks.insert(candidate.tracks.end(), proposal.replacement_tracks.begin(),
                            proposal.replacement_tracks.end());
    const std::set<ObjectId> replaced_vias(proposal.replaced_vias.begin(),
                                           proposal.replaced_vias.end());
    const auto found_vias = static_cast<std::size_t>(
        std::count_if(candidate.vias.begin(), candidate.vias.end(),
                      [&](const auto& via) { return replaced_vias.contains(via.id); }));
    if (found_vias != replaced_vias.size()) {
      result.diagnostic = "global PNS proposal no longer binds every replaced source via";
      return result;
    }
    std::erase_if(candidate.vias, [&](const auto& via) { return replaced_vias.contains(via.id); });
    candidate.vias.insert(candidate.vias.end(), proposal.replacement_vias.begin(),
                          proposal.replacement_vias.end());
  }
  if (!global.selected_shove_proposals.empty() &&
      (!validate_board(candidate).ok() || !check_copper_rules(candidate).ok())) {
    result.diagnostic = "global PNS proposal failed detailed structural or copper acceptance";
    return result;
  }
  if (config.existing_routing == ExistingRoutingPolicy::replace_unlocked_for_routed_nets) {
    remove_unlocked_routing(candidate, replacement_nets, result);
  }
  auto used_ids = object_ids(candidate);
  if (checkpoint != nullptr && checkpoint->candidate) {
    for (const auto& track : checkpoint->candidate->tracks) {
      if (reusable_nets.contains(track.net) && used_ids.insert(track.id.value()).second) {
        candidate.tracks.push_back(track);
        ++result.reused_tracks;
      }
    }
    for (const auto& arc : checkpoint->candidate->track_arcs) {
      if (reusable_nets.contains(arc.net) && used_ids.insert(arc.id.value()).second) {
        candidate.track_arcs.push_back(arc);
        ++result.reused_track_arcs;
      }
    }
    for (const auto& bezier : checkpoint->candidate->track_beziers) {
      if (reusable_nets.contains(bezier.net) && used_ids.insert(bezier.id.value()).second) {
        candidate.track_beziers.push_back(bezier);
        ++result.reused_track_beziers;
      }
    }
    for (const auto& via : checkpoint->candidate->vias) {
      if (reusable_nets.contains(via.net) && used_ids.insert(via.id.value()).second) {
        candidate.vias.push_back(via);
        ++result.reused_vias;
      }
    }
  }
  std::set<ObjectId> protected_track_ids;
  std::set<NetId> protected_routing_nets;
  for (const auto& track : candidate.tracks) {
    protected_track_ids.insert(track.id);
    protected_routing_nets.insert(track.net);
  }
  for (const auto& arc : candidate.track_arcs) {
    protected_routing_nets.insert(arc.net);
  }
  for (const auto& curve : candidate.track_beziers) protected_routing_nets.insert(curve.net);
  for (const auto& via : candidate.vias) {
    protected_routing_nets.insert(via.net);
  }
  using PathKey = std::pair<NetId, std::size_t>;
  std::map<PathKey, const GlobalRoutePath*> paths_by_key;
  std::map<PathKey, TopologyLockedLanePath> prepared_topology_paths;
  for (const auto& path : global.paths) {
    const PathKey key{path.net, path.connection_index};
    paths_by_key.emplace(key, &path);
    if (!use_topology_locked || reusable_nets.contains(path.net) ||
        topology_excluded_nets.contains(path.net) || path.terminal_neck_down_segments > 0U ||
        path.medial_neck_down_segments > 0U || path.corridor_neck_down_segments > 0U) {
      continue;
    }
    const auto prepared =
        topology_locked_lane_points(global, path, dimensions.at(path.net).track_width,
                                    config.max_topology_locked_terminal_access_points);
    if (prepared) {
      prepared_topology_paths.emplace(key, *prepared);
    }
  }
  std::set<PathKey> governed_coupled_paths;
  std::set<PathKey> accepted_coupled_paths;
  std::set<PathKey> governed_coupled_fanout_paths;
  std::set<PathKey> accepted_coupled_fanout_paths;
  std::set<PathKey> governed_shared_resource_fanout_paths;
  std::set<PathKey> rejected_shared_resource_fanout_paths;
  const bool has_legal_scaffold = std::any_of(
      global.paths.begin(), global.paths.end(),
      [](const GlobalRoutePath& path) { return path.legal_scaffold; });
  std::set<NetId> materialized_topology_nets;
  if (use_topology_locked) {
    struct SharedRunOccurrence {
      PathKey key;
      TopologyLockedLanePath* path{nullptr};
      std::size_t run_index{0};
      std::int64_t signed_offset_nm{0};
    };
    std::map<std::pair<std::size_t, NetId>, std::int64_t> lane_offsets;
    for (const auto& lane : global.voronoi_lane_assignments) {
      lane_offsets.emplace(std::pair{lane.resource_id, lane.net}, lane.signed_offset_nm);
    }
    std::map<std::size_t, std::vector<SharedRunOccurrence>> shared_runs_by_resource;
    for (auto& [key, path] : prepared_topology_paths) {
      if (path.coupled_member) {
        continue;
      }
      for (std::size_t run_index = 0U; run_index < path.packed_runs.size(); ++run_index) {
        std::set<std::size_t> distinct_resources(path.packed_runs[run_index].resource_ids.begin(),
                                                 path.packed_runs[run_index].resource_ids.end());
        for (const auto resource_id : distinct_resources) {
          const auto offset = lane_offsets.find(std::pair{resource_id, key.first});
          if (offset == lane_offsets.end()) {
            continue;
          }
          shared_runs_by_resource[resource_id].push_back(
              SharedRunOccurrence{key, &path, run_index, offset->second});
        }
      }
    }
    std::set<std::vector<std::pair<PathKey, std::size_t>>> attempted_run_groups;
    for (auto& [resource_id, occurrences] : shared_runs_by_resource) {
      std::map<NetId, std::vector<SharedRunOccurrence*>> occurrences_by_net;
      for (auto& occurrence : occurrences) {
        occurrences_by_net[occurrence.key.first].push_back(&occurrence);
      }
      std::vector<SharedRunOccurrence*> selected;
      for (auto& [net, net_occurrences] : occurrences_by_net) {
        static_cast<void>(net);
        if (net_occurrences.size() == 1U) {
          selected.push_back(net_occurrences.front());
        }
      }
      if (selected.size() < 2U) {
        continue;
      }
      std::sort(selected.begin(), selected.end(), [](const auto* lhs, const auto* rhs) {
        return std::tie(lhs->signed_offset_nm, lhs->key, lhs->run_index) <
               std::tie(rhs->signed_offset_nm, rhs->key, rhs->run_index);
      });
      std::vector<std::pair<PathKey, std::size_t>> signature;
      signature.reserve(selected.size());
      for (const auto* occurrence : selected) {
        signature.emplace_back(occurrence->key, occurrence->run_index);
      }
      auto canonical_signature = signature;
      std::sort(canonical_signature.begin(), canonical_signature.end());
      if (!attempted_run_groups.insert(std::move(canonical_signature)).second) {
        continue;
      }

      CoupledRoutingGroup group;
      std::vector<TopologyLockedLanePath*> group_paths;
      std::vector<std::size_t> run_indices;
      group.members.reserve(selected.size());
      group_paths.reserve(selected.size());
      run_indices.reserve(selected.size());
      for (const auto* occurrence : selected) {
        group.members.push_back(occurrence->key.first);
        group_paths.push_back(occurrence->path);
        run_indices.push_back(occurrence->run_index);
        governed_shared_resource_fanout_paths.insert(occurrence->key);
      }
      ++result.attempted_shared_resource_fanout_groups;
      const auto* shared_resource = find_resource_usage(global, resource_id);
      if (shared_resource != nullptr &&
          synchronize_shared_resource_fanouts(source, group, group_paths, run_indices, dimensions,
                                              config.max_shared_resource_fanout_candidates,
                                              result.attempted_shared_resource_fanout_candidates,
                                              result.synchronized_shared_resource_fanouts)) {
        ++result.materialized_shared_resource_fanout_groups;
      } else {
        ++result.shared_resource_fanout_fallbacks;
        std::vector<NetId> selected_fallback_nets;
        if (shared_resource != nullptr) {
          SharedResourceFanoutFailure failure{resource_id,
                                              shared_resource->layer,
                                              shared_resource->start,
                                              shared_resource->end,
                                              {},
                                              {},
                                              {},
                                              {}};
          failure.nets.reserve(selected.size());
          for (const auto* occurrence : selected) {
            failure.nets.push_back(occurrence->key.first);
            const auto& run = occurrence->path->packed_runs[occurrence->run_index];
            const auto start_anchor =
                packed_run_anchor_index(*occurrence->path, occurrence->run_index, true);
            const auto end_anchor =
                packed_run_anchor_index(*occurrence->path, occurrence->run_index, false);
            if (start_anchor && end_anchor) {
              failure.members.push_back(SharedResourceFanoutFailureMember{
                  occurrence->key.first, occurrence->path->points[*start_anchor].position,
                  occurrence->path->points[run.first_index].position,
                  occurrence->path->points[run.last_index].position,
                  occurrence->path->points[*end_anchor].position});
            }
          }
          std::sort(failure.nets.begin(), failure.nets.end());
          failure.nets.erase(std::unique(failure.nets.begin(), failure.nets.end()),
                             failure.nets.end());
          std::sort(failure.members.begin(), failure.members.end(),
                    [](const auto& lhs, const auto& rhs) { return lhs.net < rhs.net; });
          struct EndpointOrderEvidence {
            NetId net;
            Point anchor;
            Point lane;
            long double outward_distance{0.0L};
          };
          std::array<std::vector<EndpointOrderEvidence>, 2U> endpoint_members;
          const auto resource_dx = static_cast<long double>(failure.end.x.value()) -
                                   static_cast<long double>(failure.start.x.value());
          const auto resource_dy = static_cast<long double>(failure.end.y.value()) -
                                   static_cast<long double>(failure.start.y.value());
          const auto resource_length_squared =
              resource_dx * resource_dx + resource_dy * resource_dy;
          const auto endpoint = [&failure, resource_dx, resource_dy,
                                 resource_length_squared](Point lane) {
            const auto projection = (static_cast<long double>(lane.x.value()) -
                                     static_cast<long double>(failure.start.x.value())) *
                                        resource_dx +
                                    (static_cast<long double>(lane.y.value()) -
                                     static_cast<long double>(failure.start.y.value())) *
                                        resource_dy;
            return std::abs(projection) <= std::abs(projection - resource_length_squared) ? 0U : 1U;
          };
          const auto longitudinal_projection = [&failure, resource_dx, resource_dy](Point point) {
            return (static_cast<long double>(point.x.value()) -
                    static_cast<long double>(failure.start.x.value())) *
                       resource_dx +
                   (static_cast<long double>(point.y.value()) -
                    static_cast<long double>(failure.start.y.value())) *
                       resource_dy;
          };
          for (const auto& member : failure.members) {
            const auto start_endpoint = endpoint(member.start_lane);
            const auto end_endpoint = endpoint(member.end_lane);
            const auto start_projection = longitudinal_projection(member.start_anchor);
            const auto end_projection = longitudinal_projection(member.end_anchor);
            endpoint_members[start_endpoint].push_back(EndpointOrderEvidence{
                member.net, member.start_anchor, member.start_lane,
                start_endpoint == 0U ? -start_projection
                                     : start_projection - resource_length_squared});
            endpoint_members[end_endpoint].push_back(EndpointOrderEvidence{
                member.net, member.end_anchor, member.end_lane,
                end_endpoint == 0U ? -end_projection : end_projection - resource_length_squared});
          }
          const auto cross = [resource_dx, resource_dy](Point point) {
            return -resource_dy * static_cast<long double>(point.x.value()) +
                   resource_dx * static_cast<long double>(point.y.value());
          };
          const auto cross_direction = [](long double value) {
            return value < 0.0L ? -1 : value > 0.0L ? 1 : 0;
          };
          std::set<NetId> conflicting_nets;
          for (const auto& side_members : endpoint_members) {
            std::map<NetId, EndpointOrderEvidence> best_by_net;
            for (const auto& member : side_members) {
              const auto found = best_by_net.find(member.net);
              if (found == best_by_net.end() ||
                  member.outward_distance > found->second.outward_distance) {
                best_by_net.insert_or_assign(member.net, member);
              }
            }
            std::vector<EndpointOrderEvidence> side;
            side.reserve(best_by_net.size());
            for (const auto& [net, member] : best_by_net) {
              static_cast<void>(net);
              side.push_back(member);
            }
            for (std::size_t first = 0U; first < side.size(); ++first) {
              for (std::size_t second = first + 1U; second < side.size(); ++second) {
                const auto lane_order =
                    cross_direction(cross(side[first].lane) - cross(side[second].lane));
                const auto anchor_order =
                    cross_direction(cross(side[first].anchor) - cross(side[second].anchor));
                if (lane_order == 0 || anchor_order == 0 || lane_order != anchor_order) {
                  conflicting_nets.insert(side[first].net);
                  conflicting_nets.insert(side[second].net);
                }
              }
            }
          }
          failure.conflicting_nets.assign(conflicting_nets.begin(), conflicting_nets.end());
          std::size_t survivor_mask = 0U;
          std::size_t survivor_count = 0U;
          if (selected.size() < std::numeric_limits<std::size_t>::digits) {
            const auto state_count = std::size_t{1U} << selected.size();
            for (auto mask = state_count - 1U;
                 mask > 0U && result.attempted_shared_resource_fanout_candidates <
                                  config.max_shared_resource_fanout_candidates;
                 --mask) {
              std::size_t member_count = 0U;
              for (std::size_t member = 0U; member < selected.size(); ++member) {
                member_count += (mask & (std::size_t{1U} << member)) != 0U ? 1U : 0U;
              }
              if (member_count <= survivor_count) {
                continue;
              }
              CoupledRoutingGroup survivor_group;
              std::vector<TopologyLockedLanePath> survivor_paths;
              survivor_group.members.reserve(member_count);
              survivor_paths.reserve(member_count);
              for (std::size_t member = 0U; member < selected.size(); ++member) {
                if ((mask & (std::size_t{1U} << member)) == 0U) {
                  continue;
                }
                survivor_group.members.push_back(selected[member]->key.first);
                survivor_paths.push_back(*selected[member]->path);
              }
              ++result.attempted_shared_resource_fanout_candidates;
              if (coupled_path_geometry_clear(source, survivor_group, survivor_paths, dimensions,
                                              false)) {
                survivor_mask = mask;
                survivor_count = member_count;
              }
            }
          }
          if (selected.size() >= std::numeric_limits<std::size_t>::digits) {
            failure.fallback_nets = failure.nets;
          } else {
            for (std::size_t member = 0U; member < selected.size(); ++member) {
              if ((survivor_mask & (std::size_t{1U} << member)) == 0U) {
                failure.fallback_nets.push_back(selected[member]->key.first);
              }
            }
          }
          std::sort(failure.fallback_nets.begin(), failure.fallback_nets.end());
          selected_fallback_nets = failure.fallback_nets;
          result.shared_resource_fanout_failures.push_back(std::move(failure));
        }
        for (const auto* occurrence : selected) {
          if (selected_fallback_nets.empty() ||
              std::binary_search(selected_fallback_nets.begin(), selected_fallback_nets.end(),
                                 occurrence->key.first)) {
            rejected_shared_resource_fanout_paths.insert(occurrence->key);
          }
        }
      }
    }
    for (const auto& group : coupled_groups) {
      std::set<std::size_t> multilayer_connections;
      for (const auto& member : group.members) {
        for (const auto& [key, path] : paths_by_key) {
          if (key.first == member && !reusable_nets.contains(member) &&
              path_crosses_layers(*path)) {
            multilayer_connections.insert(key.second);
          }
        }
      }
      for (const auto connection : multilayer_connections) {
        ++result.attempted_coupled_multilayer_groups;
        std::vector<PathKey> keys;
        std::vector<TopologyLockedLanePath*> group_paths;
        keys.reserve(group.members.size());
        group_paths.reserve(group.members.size());
        bool complete_group = true;
        for (const auto& member : group.members) {
          const PathKey key{member, connection};
          keys.push_back(key);
          governed_coupled_paths.insert(key);
          const auto source_path = paths_by_key.find(key);
          const auto prepared = prepared_topology_paths.find(key);
          if (source_path == paths_by_key.end() || prepared == prepared_topology_paths.end() ||
              !prepared->second.multilayer || !prepared->second.coupled_member) {
            complete_group = false;
            continue;
          }
          group_paths.push_back(&prepared->second);
        }
        std::size_t arrays = 0U;
        if (complete_group &&
            synchronize_coupled_via_arrays(source, group, group_paths, dimensions, arrays)) {
          accepted_coupled_paths.insert(keys.begin(), keys.end());
          ++result.materialized_coupled_multilayer_groups;
          result.synchronized_coupled_via_arrays += arrays;
        } else {
          ++result.coupled_multilayer_group_fallbacks;
        }
      }
    }
    for (const auto& group : coupled_groups) {
      std::set<std::size_t> connections;
      for (const auto& member : group.members) {
        for (const auto& [key, path] : paths_by_key) {
          static_cast<void>(path);
          if (key.first == member && !reusable_nets.contains(member)) {
            connections.insert(key.second);
          }
        }
      }
      for (const auto connection : connections) {
        ++result.attempted_coupled_terminal_fanout_groups;
        std::vector<PathKey> keys;
        std::vector<TopologyLockedLanePath*> group_paths;
        keys.reserve(group.members.size());
        group_paths.reserve(group.members.size());
        bool complete_group = true;
        bool requires_multilayer_group = false;
        for (const auto& member : group.members) {
          const PathKey key{member, connection};
          keys.push_back(key);
          governed_coupled_fanout_paths.insert(key);
          const auto source_path = paths_by_key.find(key);
          const auto prepared = prepared_topology_paths.find(key);
          if (source_path == paths_by_key.end() || prepared == prepared_topology_paths.end() ||
              !prepared->second.coupled_member) {
            complete_group = false;
            continue;
          }
          requires_multilayer_group = requires_multilayer_group || prepared->second.multilayer;
          group_paths.push_back(&prepared->second);
        }
        if (requires_multilayer_group &&
            !std::all_of(keys.begin(), keys.end(), [&accepted_coupled_paths](const auto& key) {
              return accepted_coupled_paths.contains(key);
            })) {
          complete_group = false;
        }
        if (complete_group && synchronize_coupled_terminal_fanouts(
                                  source, group, group_paths, dimensions,
                                  config.max_coupled_terminal_fanout_candidates,
                                  result.attempted_coupled_terminal_fanout_candidates,
                                  result.synchronized_coupled_terminal_fanouts)) {
          accepted_coupled_fanout_paths.insert(keys.begin(), keys.end());
          ++result.materialized_coupled_terminal_fanout_groups;
        } else {
          ++result.coupled_terminal_fanout_fallbacks;
        }
      }
    }
  }
  std::vector<PendingSplineRun> pending_spline_runs;
  std::set<std::string> protected_native_track_ids;
  std::set<std::string> scaffold_track_ids;
  for (const auto& path : global.paths) {
    if (reusable_nets.contains(path.net)) {
      continue;
    }
    const auto& resolved = dimensions.at(path.net);
    const TopologyLockedLanePath* topology_locked_path = nullptr;
    const std::vector<GlobalRoutePoint>* exact_scaffold_points =
        path.legal_scaffold ? &path.points : nullptr;
    if (use_topology_locked) {
      ++result.attempted_topology_locked_paths;
      const PathKey key{path.net, path.connection_index};
      const auto prepared = prepared_topology_paths.find(key);
      const bool requires_coupled_group =
          governed_coupled_paths.contains(key) || governed_coupled_fanout_paths.contains(key) ||
          (prepared != prepared_topology_paths.end() && prepared->second.coupled_member &&
           prepared->second.multilayer);
      const bool coupled_group_accepted =
          (!requires_coupled_group ||
           ((!governed_coupled_paths.contains(key) || accepted_coupled_paths.contains(key)) &&
            (!governed_coupled_fanout_paths.contains(key) ||
             accepted_coupled_fanout_paths.contains(key)))) &&
          (!governed_shared_resource_fanout_paths.contains(key) ||
           !rejected_shared_resource_fanout_paths.contains(key));
      if (exact_scaffold_points != nullptr) {
        // A constructive certificate owns this route's topology, including its via sites.
      } else if (prepared != prepared_topology_paths.end() && coupled_group_accepted) {
        topology_locked_path = &prepared->second;
        materialized_topology_nets.insert(path.net);
        ++result.materialized_topology_locked_paths;
        if (topology_locked_path->preserved_terminal_access) {
          ++result.preserved_topology_locked_terminal_access_paths;
        }
        if (topology_locked_path->multilayer) {
          ++result.materialized_multilayer_topology_locked_paths;
        }
      } else {
        ++result.topology_locked_path_fallbacks;
      }
    }
    if (!materialize_path(candidate, path, resolved.track_width, resolved.via_diameter,
                          resolved.via_drill, used_ids, result, config, use_pull_tight,
                          use_continuous_refinement, use_circular_arc_refinement, stop_token,
                          exact_scaffold_points != nullptr ? exact_scaffold_points
                          : topology_locked_path != nullptr ? &topology_locked_path->points : nullptr,
                          &pending_spline_runs, regional_splines, &protected_native_track_ids,
                          &scaffold_track_ids)) {
      result.candidate = std::move(candidate);
      routing_kernel_checkpoint();
      if (stop_token.stop_requested()) {
        result.stop_reason = DetailedRouterStopReason::cancelled;
        result.diagnostic = "detailed routing cancelled during materialization";
      } else {
        result.stop_reason = DetailedRouterStopReason::invalid_path;
        result.diagnostic = "path materialization rejected geometry or width metadata; net=" +
            path.net.value() + "; connection=" + std::to_string(path.connection_index);
      }
      return result;
    }
  }

  // Curve fitting operates on a stable, exact-cleaned straight scaffold.  Source and reused
  // routing remains protected above; generated certified scaffold copper is eligible only when
  // its physical pad partition survives the transactional cleanup.
  std::set<std::string> cleanup_protected_track_ids;
  for (const auto& id : protected_track_ids) cleanup_protected_track_ids.insert(id.value());
  cleanup_protected_track_ids.insert(protected_native_track_ids.begin(),
                                     protected_native_track_ids.end());
  std::set<NetId> coupled_nets;
  for (const auto& group : coupled_groups)
    coupled_nets.insert(group.members.begin(), group.members.end());
  // Bound the quadratic discovery pass independently of the candidate transaction budget.
  // Larger boards retain their original topology; ordinary curved realization remains available.
  const bool cleanup_requested = config.enable_generated_tail_cleanup &&
                                 config.max_generated_tail_cleanup_candidates > 0U;
  const auto cleanup_started = std::chrono::steady_clock::now();
  const auto cleanup_split_limit = std::min<std::size_t>(4096U,
      config.max_generated_tail_cleanup_split_tracks);
  const bool run_generated_tail_cleanup = cleanup_requested &&
                                           candidate.tracks.size() <= cleanup_split_limit;
  if (cleanup_requested && !run_generated_tail_cleanup)
    result.generated_tail_cleanup_budget_exhausted = true;
  if (run_generated_tail_cleanup) {
    bool split_exhausted = false;
    auto split_used_ids = used_ids;
    const auto inserted = split_tracks_at_exact_same_net_branches(
        candidate, split_used_ids, cleanup_protected_track_ids, stop_token, true, cleanup_split_limit,
        &split_exhausted);
    if (split_exhausted) {
      result.generated_tail_cleanup_budget_exhausted = true;
    } else {
      used_ids = std::move(split_used_ids);
      result.added_tracks += inserted;
      cleanup_generated_straight_dead_tails(candidate, cleanup_protected_track_ids,
                                            protected_routing_nets, coupled_nets, config, result,
                                            stop_token);
    }
    routing_kernel_checkpoint();
    if (stop_token.stop_requested()) {
      result.candidate = std::move(candidate);
      result.stop_reason = DetailedRouterStopReason::cancelled;
      result.diagnostic = "detailed routing cancelled during generated-tail cleanup";
      return result;
    }
  }
  if (cleanup_requested) {
    result.generated_tail_cleanup_microseconds +=
        static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now() - cleanup_started).count());
  }

  // Realize complete degree-two routes against one immutable, board-complete copper snapshot.
  // NavMesh preparation, PH optimization, recursive certificates, arc fitting and bounded PNS are
  // independent at this boundary. Workers publish compact copper deltas into stable task slots;
  // only the original task-order reduction below may modify the candidate board.
  for (const auto& id : protected_native_track_ids) protected_track_ids.insert(ObjectId{id});
  std::set<std::string> curve_protected_track_ids;
  for (const auto& id : protected_track_ids) curve_protected_track_ids.insert(id.value());
  if (use_circular_arc_refinement && !run_generated_tail_cleanup) {
    result.added_tracks += split_tracks_at_exact_same_net_branches(candidate, used_ids, curve_protected_track_ids, stop_token);
  }
  const auto spline_realization_started = std::chrono::steady_clock::now();
  const auto realization_passes = config.retry_failed_spline_routes_after_commits ? 2U : 1U;
  for (std::size_t realization_pass = 0U;
       use_circular_arc_refinement && realization_pass < realization_passes; ++realization_pass) {
    const Board spline_navigation_board = candidate;
    const auto commits_before = result.spline_parallel_route_commits;
    const auto board_level_spline_runs =
        collect_unrounded_spline_chains(candidate, curve_protected_track_ids, scaffold_track_ids);
    if (board_level_spline_runs.empty())
      break;
    if (realization_pass > 0U)
      result.spline_committed_context_retry_tasks += board_level_spline_runs.size();
    struct ParallelSplineRouteResult {
      DetailedRouteResult metrics;
      std::optional<SplineRouteCopperDelta> delta;
      std::exception_ptr failure;
    };
    std::vector<std::optional<ParallelSplineRouteResult>> task_results(
        board_level_spline_runs.size());
    const auto requested_workers =
        std::min(config.spline_route_workers, board_level_spline_runs.size());
    auto route_worker_lease = acquire_cpu_workers(requested_workers);
    const auto worker_count = route_worker_lease.workers();
    const auto candidate_budget_per_route =
        std::max<std::size_t>(1U, config.max_spline_candidate_checks /
                                      std::max<std::size_t>(1U, board_level_spline_runs.size()));
    std::unique_ptr<SplineNavigationPreparationCache> navigation_preparation_cache;
    if (config.reuse_spline_navigation_preparation) {
      navigation_preparation_cache = std::make_unique<SplineNavigationPreparationCache>();
      std::map<SplineNavigationPreparationCache::Key, std::size_t> consumers;
      for (const auto& pending : board_level_spline_runs)
        ++consumers[{pending.net, pending.run.layer, pending.run.width}];
      // Only repeated keys benefit. Retention is bounded by the shared worker budget, not
      // by board net count; select keys in stable order and release after their last consumer.
      const auto max_entries = 2U * std::max<std::size_t>(1U, worker_count);
      for (const auto& [key, count] : consumers) {
        if (count < 2U) continue;
        if (navigation_preparation_cache->entries.size() >= max_entries) break;
        auto entry = std::make_shared<SplineNavigationPreparationEntry>();
        entry->remaining_consumers = entry->consumers = count;
        navigation_preparation_cache->entries.emplace(key, std::move(entry));
      }
    }
    std::unique_ptr<NavMeshScreeningContext> screening_context;
    if (config.gpu_materialization_batches)
      screening_context = std::make_unique<NavMeshScreeningContext>();
    std::atomic<std::size_t> next_task{0U};
    const auto run_task = [&] {
      while (!stop_token.stop_requested()) {
        const auto task = next_task.fetch_add(1U, std::memory_order_relaxed);
        if (task >= board_level_spline_runs.size())
          break;
        ParallelSplineRouteResult completed;
        try {
          const auto& pending = board_level_spline_runs[task];
          auto local_board = spline_navigation_board;
          auto local_used_ids = used_ids;
          auto local_config = config;
          local_config.spline_exact_candidate_workers = 1U;
          local_config.max_spline_candidate_checks = candidate_budget_per_route;
          if (refine_spline_run(local_board, spline_navigation_board, pending.net, pending.run,
                                pending.base, local_used_ids, completed.metrics, local_config,
                                stop_token, navigation_preparation_cache.get(), screening_context.get())) {
            completed.delta = make_spline_route_delta(spline_navigation_board, local_board);
          }
        } catch (...) {
          completed.failure = std::current_exception();
        }
        task_results[task] = std::move(completed);
      }
    };
    if (worker_count == 0U) {
      // A nested/parallel routing phase already owns the shared CPU budget. Preserve the exact
      // task order and cancellation behavior without waiting for a permit that could deadlock.
      run_task();
    } else {
      std::vector<std::jthread> workers;
      workers.reserve(worker_count);
      for (std::size_t worker = 0U; worker < worker_count; ++worker)
        workers.emplace_back(run_task);
      workers.clear();
    }

    if (screening_context) {
      const auto screening = screening_context->stats();
      result.spline_screening_batched_chords += screening.batched_chord_requests;
      result.spline_screening_resident_uploads += screening.resident_boundary_uploads;
      result.spline_screening_resident_reuses += screening.resident_boundary_reuses;
      result.spline_screening_cache_hits += screening.chord_cache_hits;
      result.spline_screening_cache_misses += screening.chord_cache_misses;
      result.spline_screening_coalesced_waits += screening.coalesced_chord_waits;
      result.spline_screening_batches += screening.classifier_batches;
      result.spline_screening_gpu_dispatches += screening.gpu_dispatches;
      result.spline_screening_gpu_work_items += screening.gpu_work_items;
      result.spline_screening_gpu_pruned_work_items += screening.gpu_pruned_work_items;
      result.spline_navmesh_prepared_hits += screening.prepared_domain_hits;
      result.spline_navmesh_prepared_builds += screening.prepared_domain_misses;
      result.spline_navmesh_prepared_waits += screening.coalesced_preparation_waits;
      result.spline_navmesh_prepared_evictions += screening.prepared_domain_evictions;
      result.spline_navmesh_geometry_hits += screening.geometry_hits;
      result.spline_navmesh_geometry_builds += screening.geometry_misses;
      result.spline_screening_microseconds += screening.classifier_elapsed_us;
    }
    if (navigation_preparation_cache) {
      const auto geometry = navigation_preparation_cache->obstacle_geometry.stats();
      const auto offsets = navigation_preparation_cache->obstacle_offsets.stats();
      result.spline_obstacle_geometry_hits += geometry.hits;
      result.spline_obstacle_geometry_misses += geometry.misses;
      result.spline_obstacle_offset_hits += offsets.hits;
      result.spline_obstacle_offset_misses += offsets.misses;
      result.spline_free_space_input_validation_hits += offsets.input_validation_hits;
      result.spline_free_space_input_validation_misses += offsets.input_validation_misses;
      result.spline_free_space_prepared_domain_hits += offsets.prepared_domain_hits;
      result.spline_free_space_prepared_source_hits += offsets.prepared_source_hits;
      result.spline_free_space_prepared_domain_misses += offsets.prepared_domain_misses;
      result.spline_free_space_prepared_source_misses += offsets.prepared_source_misses;
      result.spline_free_space_outline_cache_hits += offsets.outline_hits;
      result.spline_free_space_union_cache_hits += offsets.union_cache_hits;
      result.spline_free_space_outline_cache_misses += offsets.outline_misses;
      result.spline_free_space_union_cache_misses += offsets.union_cache_misses;
      result.spline_free_space_boolean_conversion_microseconds +=
          static_cast<std::uint64_t>(offsets.boolean_conversion_seconds * 1'000'000.0);
      result.spline_free_space_boolean_offset_preparation_microseconds +=
          static_cast<std::uint64_t>(offsets.boolean_offset_preparation_seconds * 1'000'000.0);
      result.spline_free_space_boolean_union_microseconds +=
          static_cast<std::uint64_t>(offsets.boolean_union_seconds * 1'000'000.0);
      result.spline_free_space_boolean_subtraction_microseconds +=
          static_cast<std::uint64_t>(offsets.boolean_subtraction_seconds * 1'000'000.0);
      result.spline_free_space_boolean_contour_extraction_microseconds +=
          static_cast<std::uint64_t>(offsets.boolean_contour_extraction_seconds * 1'000'000.0);
      result.spline_free_space_clearance_certificate_hits += offsets.clearance_certificate_hits;
      result.spline_free_space_clearance_certificate_misses += offsets.clearance_certificate_misses;
      result.spline_free_space_input_validation_microseconds +=
          static_cast<std::uint64_t>(offsets.input_validation_seconds * 1'000'000.0);
      result.spline_free_space_preparation_microseconds +=
          static_cast<std::uint64_t>(offsets.preparation_seconds * 1'000'000.0);
      result.spline_free_space_boolean_operations_microseconds +=
          static_cast<std::uint64_t>(offsets.boolean_operations_seconds * 1'000'000.0);
      result.spline_free_space_output_validation_microseconds +=
          static_cast<std::uint64_t>(offsets.output_validation_seconds * 1'000'000.0);
    }
    result.spline_parallel_route_tasks += board_level_spline_runs.size();
    result.spline_parallel_route_workers += worker_count;
    std::unique_ptr<CopperTransactionValidator> merge_validator;
    for (std::size_t task = 0U; task < task_results.size(); ++task) {
      MaterializationTimer merge_timer(result.spline_exact_merge_microseconds);
      routing_kernel_checkpoint();
      if (stop_token.stop_requested()) {
        result.candidate = std::move(candidate);
        result.stop_reason = DetailedRouterStopReason::cancelled;
        result.diagnostic = "detailed routing cancelled during parallel route realization";
        merge_timer.finish();
        return result;
      }
      const auto& pending = board_level_spline_runs[task];
      if (!task_results[task]) {
        result.withheld_circular_arc_corners += pending.run.points.size() - 2U;
        continue;
      }
      auto& completed = *task_results[task];
      if (completed.failure)
        std::rethrow_exception(completed.failure);
      accumulate_spline_route_attempt_metrics(result, completed.metrics);
      if (!completed.delta) {
        const auto stage = [&] {
          if (completed.metrics.spline_navmesh_routes == 0U)
            return "navmesh";
          if (completed.metrics.spline_trajectory_accepted == 0U)
            return "trajectory";
          if (completed.metrics.spline_final_validation_rejections > 0U)
            return "validation";
          if (completed.metrics.spline_final_copper_rejections > 0U)
            return "copper";
          if (completed.metrics.spline_final_connectivity_rejections > 0U)
            return "connectivity";
          return "materialization";
        }();
        auto detail =
            "net=" + pending.net.value() + ";layer=" + pending.run.layer.value() +
            ";base=" + pending.base + ";context_pass=" + std::to_string(realization_pass) +
            ";points=" + std::to_string(pending.run.points.size()) +
            ";stage=" + stage + ";initial_shape=" +
            std::to_string(completed.metrics.spline_trajectory_initial_shape_violations) +
            ";final_shape=" +
            std::to_string(completed.metrics.spline_trajectory_final_shape_violations) +
            ";final_inflections=" +
            std::to_string(completed.metrics.spline_trajectory_final_unintended_inflections) +
            ";final_turn=" +
            std::to_string(completed.metrics.spline_trajectory_final_excess_turn_violations) +
            ";navmesh=" + completed.metrics.spline_navmesh_last_diagnostic +
            ";topology=" + completed.metrics.spline_navmesh_last_topology_diagnostic +
            ";materialization=" + completed.metrics.spline_last_materialization_diagnostic +
            ";polyline=";
        for (std::size_t point = 0U; point < pending.run.points.size(); ++point) {
          if (point > 0U)
            detail += '|';
          detail += std::to_string(pending.run.points[point].x.value()) + ',' +
                    std::to_string(pending.run.points[point].y.value());
        }
        result.spline_route_failure_details.push_back(std::move(detail));
        result.withheld_circular_arc_corners += pending.run.points.size() - 2U;
        continue;
      }
      ++result.spline_parallel_route_candidates;
      Board transaction = candidate;
      const auto native_touched = [&](const auto& delta) {
        return std::any_of(delta.expected.begin(), delta.expected.end(), [&](const auto& object) {
          return protected_native_track_ids.contains(object.id.value());
        });
      };
      if (native_touched(completed.delta->tracks) || native_touched(completed.delta->arcs) ||
          native_touched(completed.delta->beziers)) {
        ++result.spline_parallel_route_merge_rejections;
        result.withheld_circular_arc_corners += pending.run.points.size() - 2U;
        continue;
      }
      if (!apply_spline_route_delta(transaction, *completed.delta)) {
        ++result.spline_parallel_route_merge_rejections;
        result.withheld_circular_arc_corners += pending.run.points.size() - 2U;
        continue;
      }
      const auto additional_arcs = transaction.track_arcs.size() > candidate.track_arcs.size()
                                       ? transaction.track_arcs.size() - candidate.track_arcs.size()
                                       : 0U;
      if (additional_arcs > config.max_circular_arcs -
                                std::min(config.max_circular_arcs, result.fitted_circular_arcs)) {
        result.circular_arc_candidate_budget_exhausted = true;
        ++result.spline_parallel_route_merge_rejections;
        result.withheld_circular_arc_corners += pending.run.points.size() - 2U;
        continue;
      }
      const auto additional_beziers = transaction.track_beziers.size() > candidate.track_beziers.size()
                                          ? transaction.track_beziers.size() - candidate.track_beziers.size()
                                          : 0U;
      if (!merge_validator)
        merge_validator = std::make_unique<CopperTransactionValidator>(
            candidate, config.defer_saved_zone_fill_clearance_to_refill);
      const auto transaction_report = merge_validator->validate(transaction);
      record_transaction_metrics(result, transaction_report);
      const auto& validation = transaction_report.structural;
      const auto& copper = transaction_report.copper;
      const auto touched_ids = spline_route_delta_touched_ids(*completed.delta);
      const auto* collision = first_spline_collision(copper, touched_ids);
      const auto& before_connectivity = transaction_report.before_connectivity;
      const auto& after_connectivity = transaction_report.after_connectivity;
      const bool connectivity_regressed =
          after_connectivity.open_connections > before_connectivity.open_connections ||
          after_connectivity.unrouted_nets > before_connectivity.unrouted_nets;
      if (!validation.ok() || collision != nullptr || connectivity_regressed ||
          !transaction_report.preserves_pad_components) {
        ++result.spline_parallel_route_merge_rejections;
        result.withheld_circular_arc_corners += pending.run.points.size() - 2U;
        continue;
      }
      const auto old_track_count = candidate.tracks.size();
      const auto old_arc_count = candidate.track_arcs.size();
      const auto old_bezier_count = candidate.track_beziers.size();
      const auto old_via_count = candidate.vias.size();
      candidate = std::move(transaction);
      merge_validator.reset();
      const auto adjust_count = [](std::size_t& counter, std::size_t before, std::size_t after) {
        if (after >= before) {
          counter += after - before;
        } else {
          counter -= std::min(counter, before - after);
        }
      };
      adjust_count(result.added_tracks, old_track_count, candidate.tracks.size());
      adjust_count(result.added_track_arcs, old_arc_count, candidate.track_arcs.size());
      adjust_count(result.added_track_beziers, old_bezier_count, candidate.track_beziers.size());
      result.fitted_cubic_beziers += additional_beziers;
      adjust_count(result.added_vias, old_via_count, candidate.vias.size());
      accumulate_committed_spline_route_metrics(result, completed.metrics);
      ++result.spline_parallel_route_commits;
      if (realization_pass > 0U)
        ++result.spline_committed_context_retry_commits;
      const auto nominal_width = dimensions.at(pending.net).track_width;
      const bool width_aware = std::any_of(
          pending.run.original_track_ids.begin(), pending.run.original_track_ids.end(),
          [&](const auto& id) {
            const auto found = std::find_if(
                spline_navigation_board.tracks.begin(), spline_navigation_board.tracks.end(),
                [&](const auto& track) { return track.id.value() == id; });
            return found != spline_navigation_board.tracks.end() && found->width != nominal_width;
          });
      if (width_aware) {
        result.width_aware_fitted_circular_arcs += completed.metrics.fitted_circular_arcs;
        if (completed.metrics.fitted_circular_arcs > 0U)
          ++result.width_aware_circular_arc_paths;
      }
      for (const auto& id : touched_ids)
        used_ids.insert(id);
    }
    if (result.spline_parallel_route_commits == commits_before)
      break;
  }

  // PH/PNS owns the primary realization. Only after those bounded transactions have committed do
  // we inspect the remaining eligible straight chains and apply the exact fillet transaction.
  // Mixed curve tangencies remain for the final audit; this pass never reinterprets them.
  MaterializationTimer residual_timer(result.residual_bend_repair_microseconds);
  if (use_circular_arc_refinement && !repair_residual_straight_bends(
          candidate, used_ids, result, config, curve_protected_track_ids, stop_token)) {
    result.candidate = std::move(candidate);
    result.stop_reason = DetailedRouterStopReason::cancelled;
    result.diagnostic = "detailed routing cancelled during residual bend repair";
    residual_timer.finish();
    return result;
  }
  residual_timer.finish();
  result.spline_realization_microseconds +=
      static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::microseconds>(
          std::chrono::steady_clock::now() - spline_realization_started).count());

  const auto skew_tuning_status = [&]() {
    if (!config.differential_pair_skew_tuning) {
      return SkewTuningStatus::completed;
    }
    if (config.differential_pair_skew_tuning_batch_assignment) {
      return tune_differential_pair_skew_batch(candidate, global, protected_track_ids,
                                               protected_routing_nets, used_ids, config, result,
                                               stop_token);
    }
    return tune_differential_pair_skew(candidate, global, protected_track_ids,
                                       protected_routing_nets, used_ids, config, result,
                                       stop_token);
  }();
  if (skew_tuning_status == SkewTuningStatus::cancelled) {
    result.candidate = std::move(candidate);
    result.stop_reason = DetailedRouterStopReason::cancelled;
    result.diagnostic = "detailed routing cancelled during differential-pair skew tuning";
    return result;
  }

  if (config.restrict_straight_track_angles) {
    constrain_straight_track_angles(candidate, protected_track_ids, used_ids, result, stop_token);
    routing_kernel_checkpoint();
    if (stop_token.stop_requested()) {
      result.candidate = std::move(candidate);
      result.stop_reason = DetailedRouterStopReason::cancelled;
      result.diagnostic = "detailed routing cancelled during straight-track angle conversion";
      return result;
    }
  }

  validate_materialized_candidate(source, global, config, std::move(candidate), result,
                                  bounded_conflict_handoff, checkpoint);
  const bool higher_order_fallback_available =
      (use_topology_locked && result.materialized_topology_locked_paths > 0U) ||
      (use_continuous_refinement && result.continuously_refined_points > 0U) ||
      (use_pull_tight && result.pulled_tight_corners > 0U) ||
      (checkpoint != nullptr && !reusable_nets.empty());
  if (!result.complete() && !result.provisional_zone_refill_candidate() &&
      use_circular_arc_refinement && result.fitted_circular_arcs > 0U &&
      result.stop_reason != DetailedRouterStopReason::invalid_path &&
      (!higher_order_fallback_available || has_legal_scaffold) && !stop_token.stop_requested()) {
    auto curved_rejection = result.diagnostic;
    if (!result.copper_drc.violations.empty()) {
      curved_rejection += "; first copper violation " + result.copper_drc.violations.front().code;
    }
    auto fallback = materialize_impl(
        source, global, config, checkpoint, stop_token, use_topology_locked, false, use_pull_tight,
        use_continuous_refinement, topology_excluded_nets, regional_splines);
    fallback.fallback_polyline_materialization = true;
    fallback.attempted_circular_arc_corners = result.attempted_circular_arc_corners;
    fallback.attempted_circular_arc_candidates = result.attempted_circular_arc_candidates;
    fallback.rejected_circular_arc_candidates = result.rejected_circular_arc_candidates;
    fallback.fitted_circular_arcs = result.fitted_circular_arcs;
    fallback.fitted_cubic_beziers = result.fitted_cubic_beziers;
    fallback.radius_reduced_circular_arcs = result.radius_reduced_circular_arcs;
    fallback.withheld_circular_arc_corners = result.withheld_circular_arc_corners;
    fallback.circular_arc_candidate_budget_exhausted =
        result.circular_arc_candidate_budget_exhausted;
    fallback.width_aware_circular_arc_paths = result.width_aware_circular_arc_paths;
    fallback.width_aware_fitted_circular_arcs = result.width_aware_fitted_circular_arcs;
    fallback.attempted_quintic_ph_runs = result.attempted_quintic_ph_runs;
    fallback.spline_g2_promotion_attempts = result.spline_g2_promotion_attempts;
    fallback.spline_g2_promotions = result.spline_g2_promotions;
    fallback.spline_g2_fallbacks = result.spline_g2_fallbacks;
    fallback.spline_g2_promotion_microseconds = result.spline_g2_promotion_microseconds;
    fallback.incremental_transaction_checks += result.incremental_transaction_checks;
    fallback.incremental_transaction_changed_objects += result.incremental_transaction_changed_objects;
    fallback.incremental_transaction_neighbours += result.incremental_transaction_neighbours;
    fallback.incremental_transaction_affected_nets += result.incremental_transaction_affected_nets;
    fallback.incremental_transaction_connectivity_hits += result.incremental_transaction_connectivity_hits;
    fallback.incremental_transaction_full_fallbacks += result.incremental_transaction_full_fallbacks;
    fallback.spline_navigation_preparation_builds += result.spline_navigation_preparation_builds;
    fallback.spline_navigation_preparation_hits += result.spline_navigation_preparation_hits;
    fallback.spline_obstacle_geometry_hits += result.spline_obstacle_geometry_hits;
    fallback.spline_obstacle_geometry_misses += result.spline_obstacle_geometry_misses;
    fallback.spline_obstacle_offset_hits += result.spline_obstacle_offset_hits;
    fallback.spline_obstacle_offset_misses += result.spline_obstacle_offset_misses;
    fallback.spline_free_space_input_validation_hits += result.spline_free_space_input_validation_hits;
    fallback.spline_free_space_input_validation_misses += result.spline_free_space_input_validation_misses;
    fallback.spline_free_space_prepared_domain_hits += result.spline_free_space_prepared_domain_hits;
    fallback.spline_free_space_prepared_source_hits += result.spline_free_space_prepared_source_hits;
    fallback.spline_free_space_prepared_domain_misses += result.spline_free_space_prepared_domain_misses;
    fallback.spline_free_space_prepared_source_misses += result.spline_free_space_prepared_source_misses;
    fallback.spline_free_space_outline_cache_hits += result.spline_free_space_outline_cache_hits;
    fallback.spline_free_space_union_cache_hits += result.spline_free_space_union_cache_hits;
    fallback.spline_free_space_outline_cache_misses += result.spline_free_space_outline_cache_misses;
    fallback.spline_free_space_union_cache_misses += result.spline_free_space_union_cache_misses;
    fallback.spline_free_space_boolean_conversion_microseconds += result.spline_free_space_boolean_conversion_microseconds;
    fallback.spline_free_space_boolean_offset_preparation_microseconds += result.spline_free_space_boolean_offset_preparation_microseconds;
    fallback.spline_free_space_boolean_union_microseconds += result.spline_free_space_boolean_union_microseconds;
    fallback.spline_free_space_boolean_subtraction_microseconds += result.spline_free_space_boolean_subtraction_microseconds;
    fallback.spline_free_space_boolean_contour_extraction_microseconds += result.spline_free_space_boolean_contour_extraction_microseconds;
    fallback.spline_free_space_clearance_certificate_hits += result.spline_free_space_clearance_certificate_hits;
    fallback.spline_free_space_clearance_certificate_misses += result.spline_free_space_clearance_certificate_misses;
    fallback.spline_free_space_input_validation_microseconds += result.spline_free_space_input_validation_microseconds;
    fallback.spline_free_space_preparation_microseconds += result.spline_free_space_preparation_microseconds;
    fallback.spline_free_space_boolean_operations_microseconds += result.spline_free_space_boolean_operations_microseconds;
    fallback.spline_free_space_output_validation_microseconds += result.spline_free_space_output_validation_microseconds;

    fallback.spline_navigation_preparation_microseconds += result.spline_navigation_preparation_microseconds;
    fallback.spline_trajectory_microseconds += result.spline_trajectory_microseconds;
    fallback.generated_tail_cleanup_microseconds += result.generated_tail_cleanup_microseconds;
    fallback.spline_realization_microseconds += result.spline_realization_microseconds;
    fallback.spline_screening_batched_chords += result.spline_screening_batched_chords;
    fallback.spline_screening_resident_uploads += result.spline_screening_resident_uploads;
    fallback.spline_screening_resident_reuses += result.spline_screening_resident_reuses;
    fallback.spline_screening_cache_hits += result.spline_screening_cache_hits;
    fallback.spline_screening_cache_misses += result.spline_screening_cache_misses;
    fallback.spline_screening_coalesced_waits += result.spline_screening_coalesced_waits;
    fallback.spline_screening_batches += result.spline_screening_batches;
    fallback.spline_screening_gpu_dispatches += result.spline_screening_gpu_dispatches;
    fallback.spline_screening_gpu_work_items += result.spline_screening_gpu_work_items;
    fallback.spline_screening_gpu_pruned_work_items += result.spline_screening_gpu_pruned_work_items;
    fallback.spline_navmesh_prepared_hits += result.spline_navmesh_prepared_hits;
    fallback.spline_navmesh_prepared_builds += result.spline_navmesh_prepared_builds;
    fallback.spline_navmesh_prepared_waits += result.spline_navmesh_prepared_waits;
    fallback.spline_navmesh_prepared_evictions += result.spline_navmesh_prepared_evictions;
    fallback.spline_navmesh_geometry_hits += result.spline_navmesh_geometry_hits;
    fallback.spline_navmesh_geometry_builds += result.spline_navmesh_geometry_builds;
    fallback.spline_screening_microseconds += result.spline_screening_microseconds;
    fallback.spline_envelope_queries += result.spline_envelope_queries;
    fallback.spline_envelope_proven_clear += result.spline_envelope_proven_clear;
    fallback.spline_envelope_gpu_dispatches += result.spline_envelope_gpu_dispatches;
    fallback.spline_envelope_cpu_fallbacks += result.spline_envelope_cpu_fallbacks;
    fallback.spline_envelope_resident_uploads += result.spline_envelope_resident_uploads;
    fallback.spline_envelope_resident_reuses += result.spline_envelope_resident_reuses;
    fallback.spline_envelope_microseconds += result.spline_envelope_microseconds;
    fallback.spline_clearance_batch_queries += result.spline_clearance_batch_queries;
    fallback.spline_clearance_gpu_dispatches += result.spline_clearance_gpu_dispatches;
    fallback.spline_clearance_cpu_fallbacks += result.spline_clearance_cpu_fallbacks;
    fallback.spline_clearance_resident_uploads += result.spline_clearance_resident_uploads;
    fallback.spline_clearance_resident_reuses += result.spline_clearance_resident_reuses;
    fallback.spline_clearance_batch_microseconds += result.spline_clearance_batch_microseconds;
    fallback.spline_gpu_polish_microseconds += result.spline_gpu_polish_microseconds;
    fallback.spline_navigation_mesh_microseconds += result.spline_navigation_mesh_microseconds;
    fallback.spline_route_refinement_microseconds += result.spline_route_refinement_microseconds;
    fallback.spline_exact_merge_microseconds += result.spline_exact_merge_microseconds;
    fallback.residual_bend_repair_microseconds += result.residual_bend_repair_microseconds;

    fallback.regional_native_spline_runs_materialized =
        result.regional_native_spline_runs_materialized;
    fallback.regional_native_track_segments_materialized =
        result.regional_native_track_segments_materialized;
    fallback.regional_native_track_arcs_materialized =
        result.regional_native_track_arcs_materialized;
    fallback.regional_native_track_beziers_materialized =
        result.regional_native_track_beziers_materialized;
    fallback.fitted_quintic_ph_runs = result.fitted_quintic_ph_runs;
    fallback.adaptive_spline_tessellation_samples = result.adaptive_spline_tessellation_samples;
    fallback.spline_fitted_track_segments = result.spline_fitted_track_segments;
    fallback.spline_collision_envelopes = result.spline_collision_envelopes;
    fallback.spline_control_point_shoves = result.spline_control_point_shoves;
    fallback.spline_refits = result.spline_refits;
    fallback.spline_navmesh_routes = result.spline_navmesh_routes;
    fallback.spline_navmesh_failures = result.spline_navmesh_failures;
    fallback.spline_navmesh_refined_parents = result.spline_navmesh_refined_parents;
    fallback.spline_navmesh_refined_leaves = result.spline_navmesh_refined_leaves;
    fallback.spline_navmesh_rejected_leaves = result.spline_navmesh_rejected_leaves;
    fallback.spline_navmesh_gpu_wavefront_routes = result.spline_navmesh_gpu_wavefront_routes;
    fallback.spline_navmesh_gpu_wavefront_cpu_fallbacks =
        result.spline_navmesh_gpu_wavefront_cpu_fallbacks;
    fallback.spline_navmesh_certificate_fallbacks = result.spline_navmesh_certificate_fallbacks;
    fallback.spline_navmesh_topology_signatures = result.spline_navmesh_topology_signatures;
    fallback.spline_navmesh_topology_gates = result.spline_navmesh_topology_gates;
    fallback.spline_navmesh_topology_rejections = result.spline_navmesh_topology_rejections;
    fallback.spline_funnel_points = result.spline_funnel_points;
    fallback.spline_seed_candidates = result.spline_seed_candidates;
    fallback.spline_seed_exact_legal = result.spline_seed_exact_legal;
    fallback.spline_run_aligned_seed_candidates = result.spline_run_aligned_seed_candidates;
    fallback.spline_run_aligned_seed_exact_legal = result.spline_run_aligned_seed_exact_legal;
    fallback.spline_seed_portfolio_improvements = result.spline_seed_portfolio_improvements;
    fallback.spline_trajectory_attempts = result.spline_trajectory_attempts;
    fallback.spline_trajectory_accepted = result.spline_trajectory_accepted;
    fallback.spline_trajectory_control_moves = result.spline_trajectory_control_moves;
    fallback.spline_paired_obstacle_support_attempts = result.spline_paired_obstacle_support_attempts;
    fallback.spline_paired_obstacle_supports_inserted = result.spline_paired_obstacle_supports_inserted;
    fallback.spline_paired_obstacle_support_recoveries = result.spline_paired_obstacle_support_recoveries;
    fallback.spline_trajectory_candidate_checks = result.spline_trajectory_candidate_checks;
    fallback.spline_span_certificate_cache_hits += result.spline_span_certificate_cache_hits;
    fallback.spline_span_certificate_cache_misses += result.spline_span_certificate_cache_misses;
    fallback.spline_equivalent_candidates_skipped += result.spline_equivalent_candidates_skipped;
    fallback.spline_non_improving_candidates_skipped += result.spline_non_improving_candidates_skipped;
    fallback.spline_multistart_attempts = result.spline_multistart_attempts;
    fallback.spline_multistart_recoveries = result.spline_multistart_recoveries;
    fallback.spline_gpu_precondition_attempts = result.spline_gpu_precondition_attempts;
    fallback.spline_gpu_precondition_dispatches = result.spline_gpu_precondition_dispatches;
    fallback.spline_gpu_precondition_cpu_fallbacks = result.spline_gpu_precondition_cpu_fallbacks;
    fallback.spline_gpu_precondition_accepted = result.spline_gpu_precondition_accepted;
    fallback.spline_gpu_precondition_legal = result.spline_gpu_precondition_legal;
    fallback.spline_gpu_precondition_solve_us = result.spline_gpu_precondition_solve_us;
    fallback.spline_gpu_polish_batches = result.spline_gpu_polish_batches;
    fallback.spline_gpu_polish_dispatches = result.spline_gpu_polish_dispatches;
    fallback.spline_gpu_polish_cpu_fallbacks = result.spline_gpu_polish_cpu_fallbacks;
    fallback.spline_gpu_polish_candidates = result.spline_gpu_polish_candidates;
    fallback.spline_gpu_polish_exact_shortlist_checks =
        result.spline_gpu_polish_exact_shortlist_checks;
    fallback.spline_parallel_exact_waves = result.spline_parallel_exact_waves;
    fallback.spline_parallel_exact_checks = result.spline_parallel_exact_checks;
    fallback.spline_parallel_route_tasks = result.spline_parallel_route_tasks;
    fallback.spline_parallel_route_workers = result.spline_parallel_route_workers;
    fallback.spline_parallel_route_candidates = result.spline_parallel_route_candidates;
    fallback.spline_parallel_route_commits = result.spline_parallel_route_commits;
    fallback.spline_parallel_route_merge_rejections = result.spline_parallel_route_merge_rejections;
    fallback.spline_trajectory_initial_violations = result.spline_trajectory_initial_violations;
    fallback.spline_trajectory_final_violations = result.spline_trajectory_final_violations;
    fallback.spline_trajectory_initial_shape_violations =
        result.spline_trajectory_initial_shape_violations;
    fallback.spline_trajectory_final_shape_violations =
        result.spline_trajectory_final_shape_violations;
    fallback.spline_trajectory_final_self_intersections =
        result.spline_trajectory_final_self_intersections;
    fallback.spline_trajectory_final_cusps = result.spline_trajectory_final_cusps;
    fallback.spline_trajectory_final_control_station_reversals =
        result.spline_trajectory_final_control_station_reversals;
    fallback.spline_trajectory_final_station_reversals =
        result.spline_trajectory_final_station_reversals;
    fallback.spline_trajectory_final_excess_turn_violations =
        result.spline_trajectory_final_excess_turn_violations;
    fallback.spline_trajectory_final_unintended_inflections =
        result.spline_trajectory_final_unintended_inflections;
    fallback.spline_trajectory_final_bend_radius_violations =
        result.spline_trajectory_final_bend_radius_violations;
    fallback.spline_trajectory_initial_continuous_violations =
        result.spline_trajectory_initial_continuous_violations;
    fallback.spline_trajectory_final_continuous_violations =
        result.spline_trajectory_final_continuous_violations;
    fallback.spline_continuous_hulls_checked = result.spline_continuous_hulls_checked;
    fallback.spline_continuous_envelope_rejections = result.spline_continuous_envelope_rejections;
    fallback.spline_continuous_budget_exhaustions = result.spline_continuous_budget_exhaustions;
    fallback.spline_final_validation_rejections = result.spline_final_validation_rejections;
    fallback.spline_final_copper_rejections = result.spline_final_copper_rejections;
    fallback.spline_final_connectivity_rejections = result.spline_final_connectivity_rejections;
    fallback.spline_navmesh_last_diagnostic = result.spline_navmesh_last_diagnostic;
    fallback.spline_navmesh_last_topology_diagnostic =
        result.spline_navmesh_last_topology_diagnostic;
    fallback.spline_last_materialization_diagnostic = result.spline_last_materialization_diagnostic;
    fallback.spline_route_failure_details = result.spline_route_failure_details;
    fallback.pns_shove_attempts = result.pns_shove_attempts;
    fallback.pns_shove_candidates = result.pns_shove_candidates;
    fallback.accepted_pns_shoves = result.accepted_pns_shoves;
    fallback.pns_moved_tracks = result.pns_moved_tracks;
    fallback.pns_moved_vias = result.pns_moved_vias;
    fallback.pns_added_dogleg_tracks = result.pns_added_dogleg_tracks;
    fallback.spline_candidate_budget_exhausted = result.spline_candidate_budget_exhausted;
    fallback.pns_candidate_budget_exhausted = result.pns_candidate_budget_exhausted;
    if (fallback.complete()) {
      fallback.diagnostic = "quintic PH curve/PNS geometry was rejected (" + curved_rejection +
                            "); conventional polyline materialization completed legally";
    }
    return fallback;
  }
  if (!result.complete() && use_topology_locked && result.materialized_topology_locked_paths > 0U &&
      !stop_token.stop_requested()) {
    const auto first_copper_rejection = [&result] {
      if (result.copper_drc.ok()) {
        return std::string{};
      }
      const auto& violation = result.copper_drc.violations.front();
      auto description = violation.code + " required " +
                         std::to_string(violation.required_clearance.value()) + " actual " +
                         std::to_string(static_cast<double>(violation.actual_distance_nm));
      for (const auto& object : violation.object_ids) {
        description += " " + object;
        if (result.candidate) {
          const auto track = std::find_if(
              result.candidate->tracks.begin(), result.candidate->tracks.end(),
              [&object](const auto& candidate) { return candidate.id.value() == object; });
          if (track != result.candidate->tracks.end()) {
            description += "[" + std::to_string(track->start.x.value()) + "," +
                           std::to_string(track->start.y.value()) + "->" +
                           std::to_string(track->end.x.value()) + "," +
                           std::to_string(track->end.y.value()) + "]";
          }
        }
      }
      return description;
    }();
    const auto rejection =
        !result.validation.ok() ? std::string{"structural validation"}
        : result.connectivity.open_connections != 0U
            ? "connectivity (" + std::to_string(result.connectivity.open_connections) + " open)"
        : !result.copper_drc.ok() ? "copper DRC (" + first_copper_rejection + ")"
                                  : "detailed acceptance";
    std::size_t attempted_topology_subset_nets = 0U;
    std::size_t attempted_topology_subset_candidates = 0U;
    if (topology_excluded_nets.empty() && result.candidate && !result.copper_drc.ok()) {
      std::map<std::string, NetId> candidate_object_nets;
      for (const auto& track : result.candidate->tracks) {
        candidate_object_nets.emplace(track.id.value(), track.net);
      }
      for (const auto& arc : result.candidate->track_arcs) {
        candidate_object_nets.emplace(arc.id.value(), arc.net);
      }
      for (const auto& via : result.candidate->vias) {
        candidate_object_nets.emplace(via.id.value(), via.net);
      }
      std::set<NetId> rejected_topology_nets;
      for (const auto& violation : result.copper_drc.violations) {
        for (const auto& object : violation.object_ids) {
          const auto found = candidate_object_nets.find(object);
          if (found != candidate_object_nets.end() &&
              materialized_topology_nets.contains(found->second)) {
            rejected_topology_nets.insert(found->second);
          }
        }
      }
      std::vector<NetId> repair_nets(rejected_topology_nets.begin(), rejected_topology_nets.end());
      attempted_topology_subset_nets = repair_nets.size();
      if (!rejected_topology_nets.empty() &&
          repair_nets.size() < std::numeric_limits<std::size_t>::digits) {
        const auto state_count = std::size_t{1U} << repair_nets.size();
        for (std::size_t excluded_count = 1U;
             excluded_count < repair_nets.size() &&
             attempted_topology_subset_candidates <
                 config.max_topology_locked_subset_repair_candidates;
             ++excluded_count) {
          for (std::size_t mask = 1U;
               mask < state_count && attempted_topology_subset_candidates <
                                         config.max_topology_locked_subset_repair_candidates;
               ++mask) {
            std::set<NetId> excluded;
            for (std::size_t net = 0U; net < repair_nets.size(); ++net) {
              if ((mask & (std::size_t{1U} << net)) != 0U) {
                excluded.insert(repair_nets[net]);
              }
            }
            if (excluded.size() != excluded_count || excluded.size() >= repair_nets.size()) {
              continue;
            }
            ++attempted_topology_subset_candidates;
            auto subset = materialize_impl(source, global, config, checkpoint, stop_token, true,
                                           use_circular_arc_refinement, use_pull_tight,
                                           use_continuous_refinement, excluded, regional_splines);
            if (!subset.complete() || subset.fallback_topology_locked_materialization ||
                subset.materialized_topology_locked_paths == 0U) {
              continue;
            }
            subset.fallback_topology_locked_subset_materialization = true;
            subset.attempted_topology_locked_subset_repair_nets = attempted_topology_subset_nets;
            subset.attempted_topology_locked_subset_repair_candidates =
                attempted_topology_subset_candidates;
            subset.retained_topology_locked_subset_paths =
                subset.materialized_topology_locked_paths;
            subset.topology_locked_rejection_copper_violations =
                result.copper_drc.violation_count();
            subset.topology_locked_rejection_first_violation = first_copper_rejection;
            subset.shared_resource_fanout_failures = result.shared_resource_fanout_failures;
            subset.attempted_shared_resource_fanout_groups =
                result.attempted_shared_resource_fanout_groups;
            subset.attempted_shared_resource_fanout_candidates =
                result.attempted_shared_resource_fanout_candidates;
            subset.shared_resource_fanout_fallbacks = result.shared_resource_fanout_fallbacks;
            subset.diagnostic = "illegal topology-locked paths were removed as a bounded exact "
                                "subset; retained medial copper completed legally";
            return subset;
          }
        }
      }
    }
    auto fallback = materialize_impl(
        source, global, config, checkpoint, stop_token, false, use_circular_arc_refinement,
        use_pull_tight, use_continuous_refinement, topology_excluded_nets, regional_splines);
    fallback.fallback_topology_locked_materialization = true;
    fallback.attempted_topology_locked_subset_repair_nets = attempted_topology_subset_nets;
    fallback.attempted_topology_locked_subset_repair_candidates =
        attempted_topology_subset_candidates;
    fallback.attempted_topology_locked_paths = result.attempted_topology_locked_paths;
    fallback.materialized_topology_locked_paths = result.materialized_topology_locked_paths;
    fallback.materialized_multilayer_topology_locked_paths =
        result.materialized_multilayer_topology_locked_paths;
    fallback.topology_locked_path_fallbacks = result.topology_locked_path_fallbacks;
    fallback.topology_locked_rejection_copper_violations = result.copper_drc.violation_count();
    fallback.topology_locked_rejection_first_violation = first_copper_rejection;
    fallback.preserved_topology_locked_terminal_access_paths =
        result.preserved_topology_locked_terminal_access_paths;
    fallback.attempted_coupled_terminal_fanout_groups =
        result.attempted_coupled_terminal_fanout_groups;
    fallback.attempted_coupled_terminal_fanout_candidates =
        result.attempted_coupled_terminal_fanout_candidates;
    fallback.materialized_coupled_terminal_fanout_groups =
        result.materialized_coupled_terminal_fanout_groups;
    fallback.synchronized_coupled_terminal_fanouts = result.synchronized_coupled_terminal_fanouts;
    fallback.coupled_terminal_fanout_fallbacks = result.coupled_terminal_fanout_fallbacks;
    fallback.attempted_shared_resource_fanout_groups =
        result.attempted_shared_resource_fanout_groups;
    fallback.attempted_shared_resource_fanout_candidates =
        result.attempted_shared_resource_fanout_candidates;
    fallback.materialized_shared_resource_fanout_groups =
        result.materialized_shared_resource_fanout_groups;
    fallback.synchronized_shared_resource_fanouts = result.synchronized_shared_resource_fanouts;
    fallback.shared_resource_fanout_fallbacks = result.shared_resource_fanout_fallbacks;
    fallback.shared_resource_fanout_failures = result.shared_resource_fanout_failures;
    fallback.attempted_coupled_multilayer_groups = result.attempted_coupled_multilayer_groups;
    fallback.materialized_coupled_multilayer_groups = result.materialized_coupled_multilayer_groups;
    fallback.synchronized_coupled_via_arrays = result.synchronized_coupled_via_arrays;
    fallback.coupled_multilayer_group_fallbacks = result.coupled_multilayer_group_fallbacks;
    if (fallback.complete()) {
      fallback.diagnostic = "topology-locked medial copper was rejected by " + rejection +
                            "; conventional materialization completed legally";
    }
    return fallback;
  }
  if (!result.complete() && use_continuous_refinement && result.continuously_refined_points > 0U &&
      !stop_token.stop_requested()) {
    auto fallback =
        materialize_impl(source, global, config, checkpoint, stop_token, use_topology_locked,
                         use_circular_arc_refinement, use_pull_tight, false, topology_excluded_nets,
                         regional_splines);
    fallback.fallback_discrete_materialization = true;
    fallback.continuously_refined_points = result.continuously_refined_points;
    fallback.width_aware_continuously_refined_paths = result.width_aware_continuously_refined_paths;
    fallback.width_aware_continuously_refined_points =
        result.width_aware_continuously_refined_points;
    fallback.continuous_refinement_iterations = result.continuous_refinement_iterations;
    fallback.attempted_continuous_refinement_candidates =
        result.attempted_continuous_refinement_candidates;
    fallback.accepted_continuous_refinement_candidates =
        result.accepted_continuous_refinement_candidates;
    fallback.rejected_continuous_refinement_candidates =
        result.rejected_continuous_refinement_candidates;
    fallback.withheld_continuous_refinement_candidates =
        result.withheld_continuous_refinement_candidates;
    fallback.continuous_refinement_budget_exhausted = result.continuous_refinement_budget_exhausted;
    if (fallback.complete()) {
      fallback.diagnostic =
          "continuous trace refinement was illegal; discrete materialization completed legally";
    }
    return fallback;
  }
  if (!result.complete() && use_pull_tight && result.pulled_tight_corners > 0U &&
      !stop_token.stop_requested()) {
    auto fallback = materialize_impl(source, global, config, checkpoint, stop_token,
                                     use_topology_locked, use_circular_arc_refinement, false, false,
                                     topology_excluded_nets, regional_splines);
    fallback.fallback_orthogonal_materialization = true;
    if (fallback.complete()) {
      fallback.diagnostic =
          "45-degree pull-tight was illegal; orthogonal materialization completed legally";
    }
    return fallback;
  }
  if (checkpoint != nullptr && !result.complete() && !stop_token.stop_requested() &&
      !reusable_nets.empty()) {
    auto invalidated = invalidated_reuse_nets;
    invalidated.insert(reusable_nets.begin(), reusable_nets.end());
    auto fallback =
        materialize_impl(source, global, config, nullptr, stop_token, use_topology_locked,
                         use_circular_arc_refinement, use_pull_tight, use_continuous_refinement,
                         topology_excluded_nets, regional_splines);
    fallback.fallback_full_materialization = true;
    fallback.invalidated_reuse_nets.assign(invalidated.begin(), invalidated.end());
    if (fallback.complete()) {
      fallback.diagnostic =
          "detailed checkpoint reuse was invalidated; full materialization completed legally";
    }
    return fallback;
  }
  return result;
}

struct ResidualJointSplineRoute {
  std::size_t path_index{0};
  NetId net;
  LayerId layer;
  QuinticPHRoute spline;
  std::vector<Polygon> topology_cells;
  Nanometres track_width;
  Nanometres clearance;
};

[[nodiscard]] Point quantize_joint_spline(SplineVector point) {
  return Point{Nanometres{static_cast<std::int64_t>(std::llround(point.x))},
               Nanometres{static_cast<std::int64_t>(std::llround(point.y))}};
}

struct JointNearestPoint {
  long double distance{std::numeric_limits<long double>::infinity()};
  SplineVector away;
};

[[nodiscard]] JointNearestPoint nearest_joint_segment(SplineVector point, SplineVector first,
                                                      SplineVector second) {
  const auto dx = second.x - first.x;
  const auto dy = second.y - first.y;
  const auto length_squared = dx * dx + dy * dy;
  const auto parameter =
      length_squared > 0.0L
          ? std::clamp(((point.x - first.x) * dx + (point.y - first.y) * dy) / length_squared, 0.0L,
                       1.0L)
          : 0.0L;
  const auto closest = SplineVector{first.x + parameter * dx, first.y + parameter * dy};
  const auto away = SplineVector{point.x - closest.x, point.y - closest.y};
  const auto distance = std::hypot(away.x, away.y);
  return JointNearestPoint{distance, distance > 0.0L
                                         ? SplineVector{away.x / distance, away.y / distance}
                                         : SplineVector{}};
}

[[nodiscard]] bool in_joint_topology(const ResidualJointSplineRoute& route, Point point) {
  return std::any_of(
      route.topology_cells.begin(), route.topology_cells.end(), [&](const auto& cell) {
        return geometry::locate_point(cell, point) != geometry::PointLocation::outside;
      });
}

[[nodiscard]] std::optional<GlobalRouteResult>
make_joint_spline_candidate(const Board& source, const GlobalRouteResult& global,
                            const DetailedRouterConfig& config, std::size_t state,
                            std::size_t& optimized_groups, std::size_t& optimized_routes,
                            std::size_t& sweeps, std::size_t& control_moves) {
  auto candidate = global;
  std::vector<ResidualJointSplineRoute> routes;
  std::set<std::size_t> governed_paths;
  for (const auto& group : global.residual_conflict_groups) {
    std::vector<std::size_t> group_paths;
    for (std::size_t path_index = 0U; path_index < global.paths.size(); ++path_index) {
      const auto& path = global.paths[path_index];
      if (std::find(group.nets.begin(), group.nets.end(), path.net) == group.nets.end()) {
        continue;
      }
      group_paths.push_back(path_index);
    }
    if (group_paths.size() < 2U) {
      return std::nullopt;
    }
    std::vector<NetId> ordered_nets = group.nets;
    std::sort(ordered_nets.begin(), ordered_nets.end());
    if ((state & 1U) != 0U) {
      std::reverse(ordered_nets.begin(), ordered_nets.end());
    }
    const auto scale = 1.0L + static_cast<long double>(state / 2U) * 0.125L;
    for (const auto path_index : group_paths) {
      const auto& path = global.paths[path_index];
      if (path.points.size() < 3U ||
          !std::all_of(path.points.begin(), path.points.end(), [&](const auto& point) {
            return point.layer == path.points.front().layer;
          })) {
        return std::nullopt;
      }
      std::vector<Point> points;
      points.reserve(path.points.size());
      for (const auto& point : path.points) {
        if (points.empty() || points.back() != point.position) {
          points.push_back(point.position);
        }
      }
      auto spline = make_quintic_ph_route(points);
      if (!spline) {
        return std::nullopt;
      }
      std::vector<Polygon> cells;
      for (const auto& corridor : path.geometric_corridors) {
        if (corridor.layer == path.points.front().layer &&
            corridor.selected_path_preserves_topology) {
          cells.insert(cells.end(), corridor.topology_cells.begin(), corridor.topology_cells.end());
        }
      }
      if (cells.empty()) {
        return std::nullopt;
      }
      const auto rules = materialization_rules(source, path.net, config);
      const auto rank = static_cast<long double>(
          std::find(ordered_nets.begin(), ordered_nets.end(), path.net) - ordered_nets.begin());
      const auto centered_rank = rank - static_cast<long double>(ordered_nets.size() - 1U) / 2.0L;
      const auto dx = static_cast<long double>(points.back().x.value() - points.front().x.value());
      const auto dy = static_cast<long double>(points.back().y.value() - points.front().y.value());
      const auto chord = std::hypot(dx, dy);
      if (!(chord > 0.0L)) {
        return std::nullopt;
      }
      const auto envelope =
          static_cast<long double>(rules.track_width.value() + 2 * rules.clearance.value());
      const auto offset = centered_rank * envelope * scale;
      const auto normal = SplineVector{-dy / chord, dx / chord};
      for (std::size_t control = 2U; control + 2U < spline->control_points.size(); ++control) {
        spline->control_points[control].x += normal.x * offset;
        spline->control_points[control].y += normal.y * offset;
      }
      if (!set_quintic_ph_corridor(*spline, cells)) {
        return std::nullopt;
      }
      routes.push_back(ResidualJointSplineRoute{path_index, path.net, path.points.front().layer,
                                                std::move(*spline), std::move(cells),
                                                rules.track_width, rules.clearance});
      governed_paths.insert(path_index);
    }
    ++optimized_groups;
  }
  if (routes.size() < 2U || governed_paths.size() != routes.size()) {
    return std::nullopt;
  }
  optimized_routes += routes.size();

  for (std::size_t sweep = 0U; sweep < config.max_residual_joint_optimization_sweeps; ++sweep) {
    bool moved = false;
    for (std::size_t route_index = 0U; route_index < routes.size(); ++route_index) {
      auto& route = routes[route_index];
      std::vector<std::vector<SplineSample>> other_samples(routes.size());
      for (std::size_t other = 0U; other < routes.size(); ++other) {
        if (other == route_index || routes[other].net == route.net ||
            routes[other].layer != route.layer) {
          continue;
        }
        other_samples[other] = adaptive_tessellate_quintic_ph(
            routes[other].spline, config.spline_tessellation_tolerance,
            config.spline_max_tessellation_depth, config.max_spline_samples_per_run);
      }
      const auto evaluator = [&](SplineVector position) {
        const auto point = quantize_joint_spline(position);
        auto legal = in_joint_topology(route, point);
        long double margin = std::numeric_limits<long double>::infinity();
        SplineVector away;
        for (std::size_t other = 0U; other < routes.size(); ++other) {
          const auto& samples = other_samples[other];
          if (samples.size() < 2U) {
            continue;
          }
          const auto required =
              static_cast<long double>(route.track_width.value() +
                                       routes[other].track_width.value()) /
                  2.0L +
              static_cast<long double>(std::max(route.clearance, routes[other].clearance).value());
          for (std::size_t sample = 1U; sample < samples.size(); ++sample) {
            const auto nearest = nearest_joint_segment(position, samples[sample - 1U].position,
                                                       samples[sample].position);
            const auto candidate_margin = nearest.distance - required;
            if (candidate_margin < margin) {
              margin = candidate_margin;
              away = nearest.away;
            }
          }
        }
        if (!std::isfinite(margin)) {
          margin = 1.0e12L;
        }
        legal = legal && margin >= 0.0L;
        return SplineClearanceProbe{margin, away, legal};
      };
      SplineTrajectoryOptimizationConfig trajectory;
      trajectory.minimum_clearance = Nanometres{0};
      trajectory.minimum_bend_radius = config.spline_minimum_bend_radius;
      trajectory.control_step = config.spline_shove_step;
      trajectory.gradient_probe_step =
          Nanometres{std::max<std::int64_t>(1, config.spline_shove_step.value() / 5)};
      trajectory.maximum_corridor_displacement = config.spline_control_corridor;
      trajectory.samples = config.spline_trajectory_samples;
      trajectory.maximum_iterations = config.max_spline_trajectory_iterations;
      trajectory.maximum_candidate_checks = config.max_spline_trajectory_candidate_checks;
      const auto optimized = optimize_quintic_ph_trajectory(route.spline, trajectory, evaluator);
      control_moves += optimized.accepted_control_moves;
      moved = moved || optimized.accepted_control_moves > 0U;
    }
    ++sweeps;
    if (!moved) {
      break;
    }
  }

  std::vector<std::vector<SplineSample>> final_samples(routes.size());
  for (std::size_t route_index = 0U; route_index < routes.size(); ++route_index) {
    auto& route = routes[route_index];
    auto& samples = final_samples[route_index];
    samples = adaptive_tessellate_quintic_ph(route.spline, config.spline_tessellation_tolerance,
                                             config.spline_max_tessellation_depth,
                                             config.max_spline_samples_per_run);
    if (samples.size() < 2U || samples.size() >= config.max_spline_samples_per_run) {
      return std::nullopt;
    }
    if (std::any_of(samples.begin(), samples.end(), [&](const auto& sample) {
          return !in_joint_topology(route, quantize_joint_spline(sample.position));
        })) {
      return std::nullopt;
    }
  }
  for (std::size_t first_route = 0U; first_route < routes.size(); ++first_route) {
    for (std::size_t second_route = first_route + 1U; second_route < routes.size();
         ++second_route) {
      if (routes[first_route].net == routes[second_route].net ||
          routes[first_route].layer != routes[second_route].layer) {
        continue;
      }
      const auto required =
          static_cast<long double>(routes[first_route].track_width.value() +
                                   routes[second_route].track_width.value()) /
              2.0L +
          static_cast<long double>(
              std::max(routes[first_route].clearance, routes[second_route].clearance).value());
      for (std::size_t first = 1U; first < final_samples[first_route].size(); ++first) {
        for (std::size_t second = 1U; second < final_samples[second_route].size(); ++second) {
          if (geometry::segment_distance_nm(
                  quantize_joint_spline(final_samples[first_route][first - 1U].position),
                  quantize_joint_spline(final_samples[first_route][first].position),
                  quantize_joint_spline(final_samples[second_route][second - 1U].position),
                  quantize_joint_spline(final_samples[second_route][second].position)) < required) {
            return std::nullopt;
          }
        }
      }
    }
  }
  for (std::size_t route_index = 0U; route_index < routes.size(); ++route_index) {
    auto& route = routes[route_index];
    const auto& samples = final_samples[route_index];
    auto& path = candidate.paths[route.path_index];
    const auto first = path.points.front();
    const auto last = path.points.back();
    path.points.clear();
    path.points.reserve(samples.size());
    for (const auto& sample : samples) {
      const auto point = quantize_joint_spline(sample.position);
      if (path.points.empty() || path.points.back().position != point) {
        path.points.push_back(GlobalRoutePoint{point, route.layer});
      }
    }
    if (path.points.size() < 3U) {
      return std::nullopt;
    }
    path.points.front() = first;
    path.points.back() = last;
    path.segment_widths.assign(path.points.size() - 1U, Nanometres{0});
    path.segment_width_kinds.assign(path.points.size() - 1U, GlobalRouteSegmentWidthKind::nominal);
  }
  candidate.total_overflow = 0U;
  candidate.sampled_overflow = 0U;
  candidate.corridor_overflow = 0U;
  candidate.clearance_shadow_overflow = 0U;
  candidate.total_voronoi_overflow_nm = 0U;
  candidate.peak_voronoi_overflow_nm = 0U;
  candidate.clearance_shadow_conflicts.clear();
  candidate.residual_conflict_groups.clear();
  candidate.stop_reason = GlobalRouterStopReason::complete;
  candidate.diagnostic = "bounded joint quintic PH curve residual-conflict candidate";
  return candidate;
}

[[nodiscard]] DetailedRouteResult
attempt_residual_joint_transaction(const Board& source, const GlobalRouteResult& global,
                                   const DetailedRouterConfig& config, DetailedRouteResult baseline,
                                   std::stop_token stop_token) {
  if (!config.circular_arc_refinement || !baseline.residual_conflict_handoff_attempted ||
      stop_token.stop_requested()) {
    return baseline;
  }
  for (std::size_t state = 0U; state < config.max_residual_joint_candidate_states; ++state) {
    ++baseline.residual_joint_candidate_states;
    std::size_t groups = 0U;
    std::size_t routes = 0U;
    std::size_t sweeps = 0U;
    std::size_t moves = 0U;
    const auto candidate =
        make_joint_spline_candidate(source, global, config, state, groups, routes, sweeps, moves);
    baseline.residual_joint_groups_optimized += groups;
    baseline.residual_joint_spline_routes += routes;
    baseline.residual_joint_optimization_sweeps += sweeps;
    baseline.residual_joint_control_moves += moves;
    if (!candidate) {
      continue;
    }
    auto curved_config = config;
    curved_config.circular_arc_refinement = true;
    auto exact =
        materialize_impl(source, *candidate, curved_config, nullptr, stop_token,
                         curved_config.topology_locked_medial_materialization, true,
                         curved_config.pull_tight_45_degrees, curved_config.continuous_refinement);
    if (exact.complete()) {
      exact.residual_conflict_handoff_attempted = true;
      exact.residual_conflict_groups_attempted = baseline.residual_conflict_groups_attempted;
      exact.residual_conflict_nets_attempted = baseline.residual_conflict_nets_attempted;
      exact.residual_sampled_overflow_attempted = baseline.residual_sampled_overflow_attempted;
      exact.residual_clearance_shadow_overflow_attempted =
          baseline.residual_clearance_shadow_overflow_attempted;
      exact.residual_joint_candidate_states = baseline.residual_joint_candidate_states;
      exact.residual_joint_groups_optimized = baseline.residual_joint_groups_optimized;
      exact.residual_joint_spline_routes = baseline.residual_joint_spline_routes;
      exact.residual_joint_optimization_sweeps = baseline.residual_joint_optimization_sweeps;
      exact.residual_joint_control_moves = baseline.residual_joint_control_moves;
      exact.residual_joint_exact_transactions_rejected =
          baseline.residual_joint_exact_transactions_rejected;
      exact.residual_joint_transaction_accepted = true;
      exact.diagnostic =
          "bounded joint quintic PH curve/PNS conflict transaction completed legally";
      return exact;
    }
    ++baseline.residual_joint_exact_transactions_rejected;
    routing_kernel_checkpoint();
    if (stop_token.stop_requested()) {
      break;
    }
  }
  return baseline;
}

} // namespace

DetailedRouteResult materialize_detailed_route(const Board& source, const GlobalRouteResult& global,
                                               const DetailedRouterConfig& config,
                                               std::stop_token stop_token) {
  auto result = materialize_impl(
      source, global, config, nullptr, stop_token, config.topology_locked_medial_materialization,
      config.circular_arc_refinement, config.pull_tight_45_degrees, config.continuous_refinement);
  // A bounded residual checkpoint is deliberately allowed through materialize_impl so its
  // ordinary polyline realization can serve as the exact baseline.  If that baseline remains
  // illegal, hand the complete attributed conflict set to the joint quintic PH curve/PNS
  // transaction. The joint candidate clears planning overflow only inside its private copy and is
  // committed solely when whole-board validation, connectivity, and copper DRC all pass.
  if (!result.complete() && result.residual_conflict_handoff_attempted &&
      !stop_token.stop_requested()) {
    result =
        attempt_residual_joint_transaction(source, global, config, std::move(result), stop_token);
  }
  result.routability_diagnoses = diagnose_detailed_routability(result);
  return result;
}

static void copy_regional_candidate_ranker_evidence(DetailedRouteResult& detailed,
                                                    const RegionalRouteResult& regional) {
  detailed.regional_candidate_ranker_requested = regional.candidate_ranker_requested;
  detailed.regional_candidate_ranker_loaded = regional.candidate_ranker_loaded;
  detailed.regional_candidate_ranker_trees = regional.candidate_ranker_trees;
  detailed.regional_candidate_ranker_groups_scored = regional.candidate_ranker_groups_scored;
  detailed.regional_candidate_ranker_candidates_scored =
      regional.candidate_ranker_candidates_scored;
  detailed.regional_candidate_ranker_candidates_reordered =
      regional.candidate_ranker_candidates_reordered;
  detailed.regional_candidate_ranker_training_rows_written =
      regional.candidate_ranker_training_rows_written;
  detailed.regional_candidate_ranker_training_rows_failed =
      regional.candidate_ranker_training_rows_failed;
  detailed.regional_candidate_ranker_diagnostic = regional.candidate_ranker_diagnostic;
}

[[nodiscard]] static std::vector<RegionalSplineRealization>
collect_regional_native_realizations(const RegionalRouteResult& regional) {
  if (regional.native_realization_manifest_available)
    return regional.native_realization_manifest;
  std::vector<RegionalSplineRealization> result;
  for (const auto& assignment : regional.bundle_assignments) {
    const auto window = std::find_if(regional.windows.begin(), regional.windows.end(),
        [&](const auto& candidate) { return candidate.id == assignment.window_id; });
    for (const auto& realization : assignment.spline_realizations) {
      if (window != regional.windows.end() &&
          std::find(window->owned_fragments.begin(), window->owned_fragments.end(),
                    realization.fragment) == window->owned_fragments.end())
        continue;
      // Identical owned publications are idempotent. Conflicting publications remain present
      // and fail the overlap check; never silently choose one writer's geometry.
      if (std::find(result.begin(), result.end(), realization) == result.end())
        result.push_back(realization);
    }
  }
  std::sort(result.begin(), result.end(),
            [](const auto& lhs, const auto& rhs) { return lhs.fragment < rhs.fragment; });
  return result;
}

DetailedRouteResult materialize_detailed_route(const Board& source, const RoutingPlanResult& plan,
                                               const DetailedRouterConfig& config,
                                               std::stop_token stop_token) {
  const bool direct_legal_scaffold = plan.legal_scaffold_complete && !plan.paths.empty() &&
      std::all_of(plan.paths.begin(), plan.paths.end(),
                  [](const GlobalRoutePath& path) { return path.legal_scaffold; });
  if (direct_legal_scaffold ||
      (plan.complete() && !plan.regional.spline_candidate_available &&
      plan.regional.native_scene_conflicts == 0U && plan.regional.unmapped_exact_copper_pairs == 0U &&
      validate_regional_native_ownership(plan.regional))) {
    auto selected = plan.coarse_global;
    selected.paths = plan.paths;
    auto detailed = materialize_detailed_route(source, selected, config, stop_token);
    copy_regional_candidate_ranker_evidence(detailed, plan.regional);
    return detailed;
  }
  const auto coarse_topology_complete =
      plan.coarse_global.unrouted_nets.empty() &&
      plan.coarse_global.unaccepted_selected_voronoi_lane_transitions == 0U;
  const auto bundles_complete =
      !plan.regional.bundle_assignments.empty() &&
      std::all_of(plan.regional.bundle_assignments.begin(), plan.regional.bundle_assignments.end(),
                  [](const auto& assignment) { return assignment.complete(); });
  std::vector<RegionalMultilayerRoutePlan> verified_multilayer_plans;
  std::size_t verified_layer_runs = 0U;
  std::size_t verified_via_transitions = 0U;
  bool multilayer_plans_complete = true;
  for (const auto& path : plan.regional.spline_candidate_paths) {
    const auto crosses_layers = std::adjacent_find(
        path.points.begin(), path.points.end(),
        [](const auto& first, const auto& second) { return first.layer != second.layer; });
    if (crosses_layers == path.points.end()) {
      continue;
    }
    auto verified = plan_multilayer_route(source, path);
    if (!verified) {
      multilayer_plans_complete = false;
      break;
    }
    verified_layer_runs += verified->layer_runs.size();
    verified_via_transitions += verified->via_transitions.size();
    verified_multilayer_plans.push_back(std::move(*verified));
  }
  multilayer_plans_complete =
      multilayer_plans_complete &&
      verified_multilayer_plans == plan.regional.multilayer_route_plans &&
      verified_multilayer_plans.size() == plan.regional.complete_via_dependent_routes &&
      verified_layer_runs == plan.regional.planned_layer_runs &&
      verified_via_transitions == plan.regional.planned_via_transitions;
  if (!coarse_topology_complete || !bundles_complete || !plan.regional.spline_candidate_available ||
      !plan.regional.complete() || !validate_regional_native_ownership(plan.regional) ||
      !multilayer_plans_complete || plan.regional.unmapped_clearance_shadow_conflicts != 0U ||
      plan.regional.unmapped_capacity_conflicts != 0U || stop_token.stop_requested()) {
    DetailedRouteResult rejected;
    rejected.stop_reason = stop_token.stop_requested()
                               ? DetailedRouterStopReason::cancelled
                               : DetailedRouterStopReason::incomplete_global_route;
    rejected.diagnostic =
        "regional materialization requires complete coarse topology, fully attributed bounded "
        "conflicts, a complete spline bundle candidate, and exact ordered multilayer run/via "
        "planning evidence";
    copy_regional_candidate_ranker_evidence(rejected, plan.regional);
    rejected.routability_diagnoses = diagnose_detailed_routability(rejected);
    return rejected;
  }
  auto candidate = plan.coarse_global;
  candidate.paths = plan.regional.spline_candidate_paths;
  candidate.total_overflow = 0U;
  candidate.sampled_overflow = 0U;
  candidate.corridor_overflow = 0U;
  candidate.clearance_shadow_overflow = 0U;
  candidate.total_voronoi_overflow_nm = 0U;
  candidate.peak_voronoi_overflow_nm = 0U;
  candidate.clearance_shadow_conflicts.clear();
  candidate.residual_conflict_groups.clear();
  candidate.stop_reason = GlobalRouterStopReason::complete;
  candidate.diagnostic =
      "bounded regional quintic PH curve candidate awaiting exact materialization";
  const auto native_realizations = config.circular_arc_refinement && plan.regional.configured.native_arc_materialization_authority
                                       ? collect_regional_native_realizations(plan.regional)
                                       : std::vector<RegionalSplineRealization>{};
  const auto& artifact = plan.regional.validated_copper_scene.data;
  const auto replacement_changes_source = [&](const auto& objects) {
    return config.existing_routing != ExistingRoutingPolicy::preserve &&
        std::any_of(objects.begin(), objects.end(), [&](const auto& object) {
          return !object.locked &&
              std::any_of(candidate.paths.begin(), candidate.paths.end(),
                          [&](const auto& path) { return path.net == object.net; });
        });
  };
  const auto scene_preserves_source = [&] {
    if (!artifact || artifact->scene.tracks.size() < source.tracks.size() ||
        artifact->scene.track_arcs.size() < source.track_arcs.size() ||
        artifact->scene.track_beziers.size() < source.track_beziers.size() ||
        artifact->scene.vias.size() < source.vias.size()) return false;
    auto prefix = artifact->scene;
    prefix.tracks.resize(source.tracks.size());
    prefix.track_arcs.resize(source.track_arcs.size());
    prefix.track_beziers.resize(source.track_beziers.size());
    prefix.vias.resize(source.vias.size());
    auto expected = source;
    if (artifact->refillable_zone_fills)
      for (auto& zone : expected.zones) zone.fills.clear();
    return prefix == expected;
  };
  // A DRC certificate does not authorize bypassing requested geometry transforms.
  // Until artifacts carry a full materialization-policy fingerprint, reuse only
  // straight copper under an explicit no-refinement policy.
  const bool reuse_scene = artifact &&
      !config.circular_arc_refinement && !config.continuous_refinement &&
      !config.topology_locked_medial_materialization && !config.pull_tight_45_degrees &&
      !config.differential_pair_skew_tuning &&
      !config.differential_pair_skew_tuning_batch_assignment &&
      artifact->scene.track_arcs.size() == source.track_arcs.size() &&
      artifact->scene.track_beziers.size() == source.track_beziers.size() &&
      artifact->source == source && scene_preserves_source() &&
      artifact->paths == candidate.paths &&
      artifact->manifest == plan.regional.native_realization_manifest &&
      artifact->refillable_zone_fills == config.defer_saved_zone_fill_clearance_to_refill &&
      config.track_width.value() == 0 && config.via_diameter.value() == 0 &&
      config.via_drill.value() == 0 &&
      !replacement_changes_source(source.tracks) &&
      !replacement_changes_source(source.track_arcs) && !replacement_changes_source(source.vias) &&
      !replacement_changes_source(source.track_beziers) &&
      artifact->scene.tracks.size() >= source.tracks.size() &&
      artifact->scene.track_arcs.size() >= source.track_arcs.size() &&
      artifact->scene.vias.size() >= source.vias.size();
  auto exact = materialize_impl(source, candidate, config, nullptr, stop_token,
                                config.topology_locked_medial_materialization,
                                config.circular_arc_refinement, config.pull_tight_45_degrees,
                                config.continuous_refinement, {}, &native_realizations,
                                reuse_scene ? &artifact->scene : nullptr);
  exact.routability_diagnoses = diagnose_detailed_routability(exact);
  // Preserve the regional handoff as observable transaction evidence.  The candidate passed to
  // the ordinary detailed router is intentionally overflow-free, so these facts must be copied
  // from the authoritative coarse/regional plan rather than inferred from that private candidate.
  exact.residual_conflict_handoff_attempted = true;
  exact.residual_conflict_groups_attempted =
      !plan.coarse_global.residual_conflict_groups.empty()
          ? plan.coarse_global.residual_conflict_groups.size()
          : plan.regional.components.size();
  std::set<NetId> handed_off_nets;
  for (const auto& component : plan.regional.components) {
    handed_off_nets.insert(component.nets.begin(), component.nets.end());
  }
  exact.residual_conflict_nets_attempted = handed_off_nets.size();
  exact.residual_sampled_overflow_attempted = plan.coarse_global.sampled_overflow;
  exact.residual_clearance_shadow_overflow_attempted = plan.coarse_global.clearance_shadow_overflow;
  exact.residual_joint_candidate_states = plan.regional.joint_spline_candidates_evaluated;
  exact.residual_joint_groups_optimized = plan.regional.bundle_assignments.size();
  exact.residual_joint_spline_routes = std::accumulate(
      plan.regional.bundle_assignments.begin(), plan.regional.bundle_assignments.end(),
      std::size_t{0U}, [](const auto total, const auto& assignment) {
        return total + assignment.spline_realizations.size();
      });
  exact.residual_joint_optimization_sweeps =
      plan.regional.spline_joint_shove_attempts + plan.regional.spline_adaptive_pair_shove_attempts;
  exact.residual_joint_control_moves =
      plan.regional.spline_joint_shove_accepted + plan.regional.spline_adaptive_pair_shove_accepted;
  exact.residual_joint_exact_transactions_rejected = exact.complete() ? 0U : 1U;
  exact.residual_joint_transaction_accepted = exact.complete();
  copy_regional_candidate_ranker_evidence(exact, plan.regional);
  if (exact.complete()) {
    exact.diagnostic =
        "bounded regional quintic PH curve transaction completed connectivity and copper DRC";
  } else if (!stop_token.stop_requested()) {
    exact.diagnostic =
        "bounded regional quintic PH curve transaction was discarded by exact materialization: " +
        exact.diagnostic;
  }
  return exact;
}

DetailedRouteResult materialize_detailed_route_incremental(const Board& source,
                                                           const GlobalRouteResult& global,
                                                           const DetailedRouteResult& checkpoint,
                                                           const DetailedRouterConfig& config,
                                                           std::stop_token stop_token) {
  auto result = materialize_impl(source, global, config, &checkpoint, stop_token,
                                 config.topology_locked_medial_materialization,
                                 config.circular_arc_refinement, config.pull_tight_45_degrees,
                                 config.continuous_refinement);
  result.routability_diagnoses = diagnose_detailed_routability(result);
  return result;
}

DetailedRouteResult materialize_detailed_route_incremental(const Board& source,
                                                           const RoutingPlanResult& plan,
                                                           const DetailedRouteResult& checkpoint,
                                                           const DetailedRouterConfig& config,
                                                           std::stop_token stop_token) {
  const bool direct_legal_scaffold = plan.legal_scaffold_complete && !plan.paths.empty() &&
      std::all_of(plan.paths.begin(), plan.paths.end(),
                  [](const GlobalRoutePath& path) { return path.legal_scaffold; });
  if (direct_legal_scaffold ||
      (plan.complete() && !plan.regional.spline_candidate_available &&
      plan.regional.native_scene_conflicts == 0U && plan.regional.unmapped_exact_copper_pairs == 0U &&
      validate_regional_native_ownership(plan.regional))) {
    auto selected = plan.coarse_global;
    selected.paths = plan.paths;
    auto detailed =
        materialize_detailed_route_incremental(source, selected, checkpoint, config, stop_token);
    copy_regional_candidate_ranker_evidence(detailed, plan.regional);
    return detailed;
  }

  const auto coarse_topology_complete =
      plan.coarse_global.unrouted_nets.empty() &&
      plan.coarse_global.unaccepted_selected_voronoi_lane_transitions == 0U;
  const auto bundles_complete =
      !plan.regional.bundle_assignments.empty() &&
      std::all_of(plan.regional.bundle_assignments.begin(), plan.regional.bundle_assignments.end(),
                  [](const auto& assignment) { return assignment.complete(); });
  std::vector<RegionalMultilayerRoutePlan> verified_multilayer_plans;
  std::size_t verified_layer_runs = 0U;
  std::size_t verified_via_transitions = 0U;
  bool multilayer_plans_complete = true;
  for (const auto& path : plan.regional.spline_candidate_paths) {
    const auto crosses_layers = std::adjacent_find(
        path.points.begin(), path.points.end(),
        [](const auto& first, const auto& second) { return first.layer != second.layer; });
    if (crosses_layers == path.points.end()) {
      continue;
    }
    auto verified = plan_multilayer_route(source, path);
    if (!verified) {
      multilayer_plans_complete = false;
      break;
    }
    verified_layer_runs += verified->layer_runs.size();
    verified_via_transitions += verified->via_transitions.size();
    verified_multilayer_plans.push_back(std::move(*verified));
  }
  multilayer_plans_complete =
      multilayer_plans_complete &&
      verified_multilayer_plans == plan.regional.multilayer_route_plans &&
      verified_multilayer_plans.size() == plan.regional.complete_via_dependent_routes &&
      verified_layer_runs == plan.regional.planned_layer_runs &&
      verified_via_transitions == plan.regional.planned_via_transitions;
  if (!coarse_topology_complete || !bundles_complete || !plan.regional.spline_candidate_available ||
      !plan.regional.complete() || !validate_regional_native_ownership(plan.regional) ||
      !multilayer_plans_complete || plan.regional.unmapped_clearance_shadow_conflicts != 0U ||
      plan.regional.unmapped_capacity_conflicts != 0U || stop_token.stop_requested()) {
    DetailedRouteResult rejected;
    rejected.stop_reason = stop_token.stop_requested()
                               ? DetailedRouterStopReason::cancelled
                               : DetailedRouterStopReason::incomplete_global_route;
    rejected.diagnostic =
        "incremental regional materialization requires complete coarse topology, fully "
        "attributed bounded conflicts, a complete spline bundle candidate, and exact ordered "
        "multilayer run/via planning evidence";
    copy_regional_candidate_ranker_evidence(rejected, plan.regional);
    rejected.routability_diagnoses = diagnose_detailed_routability(rejected);
    return rejected;
  }

  auto candidate = plan.coarse_global;
  candidate.paths = plan.regional.spline_candidate_paths;
  candidate.total_overflow = 0U;
  candidate.sampled_overflow = 0U;
  candidate.corridor_overflow = 0U;
  candidate.clearance_shadow_overflow = 0U;
  candidate.total_voronoi_overflow_nm = 0U;
  candidate.peak_voronoi_overflow_nm = 0U;
  candidate.clearance_shadow_conflicts.clear();
  candidate.residual_conflict_groups.clear();
  candidate.stop_reason = GlobalRouterStopReason::complete;
  candidate.diagnostic =
      "bounded regional quintic PH curve candidate awaiting incremental exact materialization";
  const auto native_realizations = config.circular_arc_refinement && plan.regional.configured.native_arc_materialization_authority
                                       ? collect_regional_native_realizations(plan.regional)
                                       : std::vector<RegionalSplineRealization>{};
  auto exact = materialize_impl(source, candidate, config, &checkpoint, stop_token,
                                config.topology_locked_medial_materialization,
                                config.circular_arc_refinement, config.pull_tight_45_degrees,
                                config.continuous_refinement, {}, &native_realizations);
  exact.routability_diagnoses = diagnose_detailed_routability(exact);
  exact.residual_conflict_handoff_attempted = true;
  exact.residual_conflict_groups_attempted =
      !plan.coarse_global.residual_conflict_groups.empty()
          ? plan.coarse_global.residual_conflict_groups.size()
          : plan.regional.components.size();
  std::set<NetId> handed_off_nets;
  for (const auto& component : plan.regional.components) {
    handed_off_nets.insert(component.nets.begin(), component.nets.end());
  }
  exact.residual_conflict_nets_attempted = handed_off_nets.size();
  exact.residual_sampled_overflow_attempted = plan.coarse_global.sampled_overflow;
  exact.residual_clearance_shadow_overflow_attempted = plan.coarse_global.clearance_shadow_overflow;
  exact.residual_joint_candidate_states = plan.regional.joint_spline_candidates_evaluated;
  exact.residual_joint_groups_optimized = plan.regional.bundle_assignments.size();
  exact.residual_joint_spline_routes = std::accumulate(
      plan.regional.bundle_assignments.begin(), plan.regional.bundle_assignments.end(),
      std::size_t{0U}, [](const auto total, const auto& assignment) {
        return total + assignment.spline_realizations.size();
      });
  exact.residual_joint_optimization_sweeps =
      plan.regional.spline_joint_shove_attempts + plan.regional.spline_adaptive_pair_shove_attempts;
  exact.residual_joint_control_moves =
      plan.regional.spline_joint_shove_accepted + plan.regional.spline_adaptive_pair_shove_accepted;
  exact.residual_joint_exact_transactions_rejected = exact.complete() ? 0U : 1U;
  exact.residual_joint_transaction_accepted = exact.complete();
  copy_regional_candidate_ranker_evidence(exact, plan.regional);
  if (exact.complete()) {
    exact.diagnostic =
        "incremental bounded regional quintic PH curve transaction completed connectivity and "
        "copper DRC";
  } else if (!stop_token.stop_requested()) {
    exact.diagnostic =
        "incremental bounded regional quintic PH curve transaction was discarded by exact "
        "materialization: " +
        exact.diagnostic;
  }
  return exact;
}

} // namespace flux
