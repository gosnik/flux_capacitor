#include "flux/core/navmesh_route.hpp"

#include "flux/core/free_space_skeleton.hpp"
#include "flux/core/geometry.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <map>
#include <numeric>
#include <optional>
#include <queue>
#include <set>
#include <tuple>

namespace flux {
namespace {

[[nodiscard]] long double twice_area(Point first, Point second, Point third) {
  return static_cast<long double>(second.x.value() - first.x.value()) *
             static_cast<long double>(third.y.value() - first.y.value()) -
         static_cast<long double>(second.y.value() - first.y.value()) *
             static_cast<long double>(third.x.value() - first.x.value());
}

[[nodiscard]] std::vector<Point> funnel(Point start, Point goal,
                                        const std::vector<NavMeshPortal>& portals) {
  std::vector<NavMeshPortal> sequence;
  sequence.reserve(portals.size() + 2U);
  sequence.push_back(NavMeshPortal{start, start});
  sequence.insert(sequence.end(), portals.begin(), portals.end());
  sequence.push_back(NavMeshPortal{goal, goal});
  std::vector<Point> result{start};
  Point apex = start;
  Point left = start;
  Point right = start;
  std::size_t apex_index = 0U;
  std::size_t left_index = 0U;
  std::size_t right_index = 0U;
  for (std::size_t index = 1U; index < sequence.size(); ++index) {
    const auto next_left = sequence[index].left;
    const auto next_right = sequence[index].right;
    if (twice_area(apex, right, next_right) <= 0.0L) {
      if (apex == right || twice_area(apex, left, next_right) > 0.0L) {
        right = next_right;
        right_index = index;
      } else {
        if (result.back() != left) {
          result.push_back(left);
        }
        apex = left;
        apex_index = left_index;
        left = apex;
        right = apex;
        left_index = apex_index;
        right_index = apex_index;
        index = apex_index;
        continue;
      }
    }
    if (twice_area(apex, left, next_left) >= 0.0L) {
      if (apex == left || twice_area(apex, right, next_left) < 0.0L) {
        left = next_left;
        left_index = index;
      } else {
        if (result.back() != right) {
          result.push_back(right);
        }
        apex = right;
        apex_index = right_index;
        left = apex;
        right = apex;
        left_index = apex_index;
        right_index = apex_index;
        index = apex_index;
      }
    }
  }
  if (result.back() != goal) {
    result.push_back(goal);
  }
  return result;
}

[[nodiscard]] geometry::AxisAlignedBounds chord_bounds(Point first, Point second) {
  return geometry::AxisAlignedBounds{
      Point{std::min(first.x, second.x), std::min(first.y, second.y)},
      Point{std::max(first.x, second.x), std::max(first.y, second.y)}};
}

[[nodiscard]] long double bounds_distance_nm(const geometry::AxisAlignedBounds& first,
                                             const geometry::AxisAlignedBounds& second) {
  const auto dx = first.maximum.x < second.minimum.x
                      ? static_cast<long double>(second.minimum.x.value() - first.maximum.x.value())
                  : second.maximum.x < first.minimum.x
                      ? static_cast<long double>(first.minimum.x.value() - second.maximum.x.value())
                      : 0.0L;
  const auto dy = first.maximum.y < second.minimum.y
                      ? static_cast<long double>(second.minimum.y.value() - first.maximum.y.value())
                  : second.maximum.y < first.minimum.y
                      ? static_cast<long double>(first.minimum.y.value() - second.maximum.y.value())
                      : 0.0L;
  return std::hypot(dx, dy);
}

[[nodiscard]] geometry::AxisAlignedBounds
union_bounds(const geometry::AxisAlignedBounds& first,
             const geometry::AxisAlignedBounds& second) {
  return geometry::AxisAlignedBounds{
      Point{std::min(first.minimum.x, second.minimum.x),
            std::min(first.minimum.y, second.minimum.y)},
      Point{std::max(first.maximum.x, second.maximum.x),
            std::max(first.maximum.y, second.maximum.y)}};
}

constexpr std::uint64_t topology_hash_offset = 1469598103934665603ULL;
constexpr std::uint64_t topology_hash_prime = 1099511628211ULL;

void topology_hash_append(std::uint64_t& hash, std::uint64_t value) {
  for (std::size_t byte = 0U; byte < sizeof(value); ++byte) {
    hash ^= (value >> (byte * 8U)) & 0xffU;
    hash *= topology_hash_prime;
  }
}

[[nodiscard]] std::uint64_t topology_boundary_site_id(Point point) {
  auto hash = topology_hash_offset;
  topology_hash_append(hash, static_cast<std::uint64_t>(point.x.value()));
  topology_hash_append(hash, static_cast<std::uint64_t>(point.y.value()));
  return hash;
}

[[nodiscard]] long double topology_side(const NavMeshTopologyGate& gate, Point point) {
  const auto middle_x =
      (static_cast<long double>(gate.left_boundary.x.value()) +
       static_cast<long double>(gate.right_boundary.x.value())) /
      2.0L;
  const auto middle_y =
      (static_cast<long double>(gate.left_boundary.y.value()) +
       static_cast<long double>(gate.right_boundary.y.value())) /
      2.0L;
  const auto normal_x = static_cast<long double>(gate.to_center.x.value() -
                                                  gate.from_center.x.value());
  const auto normal_y = static_cast<long double>(gate.to_center.y.value() -
                                                  gate.from_center.y.value());
  return (static_cast<long double>(point.x.value()) - middle_x) * normal_x +
         (static_cast<long double>(point.y.value()) - middle_y) * normal_y;
}

struct TopologyGateIntersection {
  long double progress_nm{0.0L};
  bool forward{false};
};

[[nodiscard]] std::optional<long double>
segment_gate_intersection_parameter(Point start, Point end, const NavMeshTopologyGate& gate) {
  const auto route_x = static_cast<long double>(end.x.value() - start.x.value());
  const auto route_y = static_cast<long double>(end.y.value() - start.y.value());
  const auto gate_x = static_cast<long double>(gate.right_boundary.x.value() -
                                               gate.left_boundary.x.value());
  const auto gate_y = static_cast<long double>(gate.right_boundary.y.value() -
                                               gate.left_boundary.y.value());
  const auto denominator = route_x * gate_y - route_y * gate_x;
  // A centreline travelling along a portal has no unambiguous side transition and must not be
  // accepted as topology-preserving.
  if (std::abs(denominator) <= 1.0e-18L) {
    return std::nullopt;
  }
  const auto offset_x = static_cast<long double>(gate.left_boundary.x.value() - start.x.value());
  const auto offset_y = static_cast<long double>(gate.left_boundary.y.value() - start.y.value());
  const auto route_parameter = (offset_x * gate_y - offset_y * gate_x) / denominator;
  const auto gate_parameter = (offset_x * route_y - offset_y * route_x) / denominator;
  constexpr long double tolerance = 1.0e-12L;
  if (route_parameter < -tolerance || route_parameter > 1.0L + tolerance ||
      gate_parameter < -tolerance || gate_parameter > 1.0L + tolerance) {
    return std::nullopt;
  }
  return std::clamp(route_parameter, 0.0L, 1.0L);
}

void build_cutout_bvh(PreparedNavMeshDomain& prepared) {
  prepared.cutout_order.resize(prepared.prepared_cutouts.size());
  std::iota(prepared.cutout_order.begin(), prepared.cutout_order.end(), std::size_t{0U});
  prepared.cutout_bvh.clear();
  if (prepared.cutout_order.empty()) {
    return;
  }
  const auto build = [&](auto&& self, std::size_t first, std::size_t count) -> std::size_t {
    auto bounds = prepared.prepared_cutouts[prepared.cutout_order[first]].bounds;
    for (std::size_t index = 1U; index < count; ++index) {
      bounds =
          union_bounds(bounds, prepared.prepared_cutouts[prepared.cutout_order[first + index]].bounds);
    }
    const auto node_index = prepared.cutout_bvh.size();
    prepared.cutout_bvh.push_back(NavMeshCutoutNode{bounds});
    constexpr std::size_t leaf_size = 8U;
    if (count <= leaf_size) {
      prepared.cutout_bvh[node_index].first = first;
      prepared.cutout_bvh[node_index].count = count;
      return node_index;
    }
    const auto width = bounds.maximum.x.value() - bounds.minimum.x.value();
    const auto height = bounds.maximum.y.value() - bounds.minimum.y.value();
    const auto compare = [&](std::size_t lhs, std::size_t rhs) {
      const auto& lhs_bounds = prepared.prepared_cutouts[lhs].bounds;
      const auto& rhs_bounds = prepared.prepared_cutouts[rhs].bounds;
      const auto lhs_coordinate =
          width >= height
              ? lhs_bounds.minimum.x.value() +
                    (lhs_bounds.maximum.x.value() - lhs_bounds.minimum.x.value()) / 2
              : lhs_bounds.minimum.y.value() +
                    (lhs_bounds.maximum.y.value() - lhs_bounds.minimum.y.value()) / 2;
      const auto rhs_coordinate =
          width >= height
              ? rhs_bounds.minimum.x.value() +
                    (rhs_bounds.maximum.x.value() - rhs_bounds.minimum.x.value()) / 2
              : rhs_bounds.minimum.y.value() +
                    (rhs_bounds.maximum.y.value() - rhs_bounds.minimum.y.value()) / 2;
      return lhs_coordinate != rhs_coordinate ? lhs_coordinate < rhs_coordinate : lhs < rhs;
    };
    std::stable_sort(prepared.cutout_order.begin() + static_cast<std::ptrdiff_t>(first),
                     prepared.cutout_order.begin() +
                         static_cast<std::ptrdiff_t>(first + count),
                     compare);
    const auto left_count = count / 2U;
    const auto left = self(self, first, left_count);
    const auto right = self(self, first + left_count, count - left_count);
    prepared.cutout_bvh[node_index].left = left;
    prepared.cutout_bvh[node_index].right = right;
    return node_index;
  };
  static_cast<void>(build(build, 0U, prepared.cutout_order.size()));
}

// The NavMesh free-space domain contains only its outer contour and cutouts. Use the immutable
// prepared polygons already owned by the domain so thousands of cell certificates do not rebuild
// or linearly rescan polygon boundary state. Domain topology is validated once by the caller.
[[nodiscard]] std::vector<std::optional<Nanometres>>
assess_prepared_navmesh_chords(const PreparedNavMeshDomain& prepared,
                               const std::vector<std::pair<Point, Point>>& chords) {
  std::vector<std::optional<Nanometres>> result(chords.size());
  const auto acceptance_clearance = static_cast<long double>(std::max<std::int64_t>(
      prepared.pitch.value() / 4, prepared.minimum_boundary_clearance.value()));
  std::vector<std::size_t> pending;
  pending.reserve(64U);
  for (std::size_t index = 0U; index < chords.size(); ++index) {
    const auto [start, end] = chords[index];
    if (start == end ||
        geometry::locate_point(prepared.prepared_outline, start) ==
            geometry::PointLocation::outside ||
        geometry::locate_point(prepared.prepared_outline, end) ==
            geometry::PointLocation::outside) {
      continue;
    }
    const auto bounds = chord_bounds(start, end);
    auto clearance = geometry::segment_polygon_boundary_distance_nm_capped(
        start, end, prepared.prepared_outline, acceptance_clearance);
    // Positive distance plus interior endpoints proves that the chord does not leave a simple
    // outline. Preserve the exact legacy boundary-touch semantics only for the exceptional zero
    // distance case instead of linearly scanning the outline for every interior cell chord.
    if (clearance <= 0.0L &&
        !geometry::segment_inside_polygon(start, end, prepared.free_space.outline)) {
      continue;
    }
    auto blocked = false;
    pending.clear();
    if (!prepared.cutout_bvh.empty()) {
      pending.push_back(0U);
    }
    while (!pending.empty() && !blocked) {
      const auto node_index = pending.back();
      pending.pop_back();
      const auto& node = prepared.cutout_bvh[node_index];
      if (bounds_distance_nm(bounds, node.bounds) > clearance) {
        continue;
      }
      if (!node.leaf()) {
        pending.push_back(node.right);
        pending.push_back(node.left);
        continue;
      }
      for (std::size_t offset = 0U; offset < node.count; ++offset) {
        const auto& cutout =
            prepared.prepared_cutouts[prepared.cutout_order[node.first + offset]];
        if (bounds_distance_nm(bounds, cutout.bounds) > clearance) {
          continue;
        }
        const auto distance = geometry::segment_polygon_boundary_distance_nm_capped(
            start, end, cutout, clearance);
        if (geometry::locate_point(cutout, start) != geometry::PointLocation::outside ||
            geometry::locate_point(cutout, end) != geometry::PointLocation::outside ||
            distance <= 0.0L) {
          blocked = true;
          break;
        }
        clearance = std::min(clearance, distance);
      }
    }
    if (blocked) {
      continue;
    }
    if (!std::isfinite(clearance) || clearance < 0.0L ||
        clearance > static_cast<long double>(std::numeric_limits<std::int64_t>::max())) {
      continue;
    }
    result[index] = Nanometres{static_cast<std::int64_t>(std::floor(clearance))};
  }
  return result;
}

} // namespace

namespace {

PreparedNavMeshDomain prepare_navmesh_domain_impl(const OffsetFreeSpaceRegion& region,
                                                  const NavMeshConfig& config,
                                                  bool validate_source_topology) {
  PreparedNavMeshDomain result;
  if (config.cell_pitch.value() <= 0 || config.maximum_cells == 0U ||
      config.maximum_guide_distance.value() < 0 || config.minimum_boundary_clearance.value() < 0 ||
      (!config.topology_guide.empty() &&
       (config.topology_guide.size() < 2U || config.maximum_guide_distance.value() <= 0)) ||
      (config.search_bounds &&
       (config.search_bounds->minimum.x >= config.search_bounds->maximum.x ||
        config.search_bounds->minimum.y >= config.search_bounds->maximum.y)) ||
      region.area.vertices.size() < 3U) {
    result.diagnostic = "invalid NavMesh configuration or free-space region";
    return result;
  }
  result.free_space.outline = region.area;
  result.free_space.cutouts = region.holes;
  result.prepared_outline = geometry::prepare_polygon(region.area);
  result.prepared_cutouts.reserve(region.holes.size());
  for (const auto& hole : region.holes) {
    result.prepared_cutouts.push_back(geometry::prepare_polygon(hole));
  }
  build_cutout_bvh(result);
  if (validate_source_topology &&
      !validate_free_space_chord_domain(result.free_space, &result.diagnostic)) {
    return result;
  }
  result.minimum = result.prepared_outline.bounds.minimum;
  auto maximum = result.prepared_outline.bounds.maximum;
  if (config.search_bounds) {
    result.minimum = Point{std::max(result.minimum.x, config.search_bounds->minimum.x),
                           std::max(result.minimum.y, config.search_bounds->minimum.y)};
    maximum = Point{std::min(maximum.x, config.search_bounds->maximum.x),
                    std::min(maximum.y, config.search_bounds->maximum.y)};
  }
  result.pitch = config.cell_pitch;
  result.topology_guide = config.topology_guide;
  result.maximum_guide_distance = config.maximum_guide_distance;
  result.minimum_boundary_clearance = config.minimum_boundary_clearance;
  result.search_bounds = config.search_bounds;
  const auto pitch = result.pitch.value();
  result.width = static_cast<std::size_t>(
      std::max<std::int64_t>(0, (maximum.x.value() - result.minimum.x.value()) / pitch));
  result.height = static_cast<std::size_t>(
      std::max<std::int64_t>(0, (maximum.y.value() - result.minimum.y.value()) / pitch));
  if (result.width == 0U || result.height == 0U ||
      result.width > std::numeric_limits<std::size_t>::max() / result.height) {
    result.diagnostic = "NavMesh cell lattice is empty or exceeds its safety bound";
    return result;
  }
  constexpr auto no_cell = std::numeric_limits<std::size_t>::max();
  result.cell_by_coordinate.assign(result.width * result.height, no_cell);
  struct CellCandidate {
    std::size_t x{0U};
    std::size_t y{0U};
    std::array<Point, 4U> corners;
    Point center;
  };
  constexpr std::size_t candidate_batch_size = 2'048U;
  std::vector<CellCandidate> candidates;
  candidates.reserve(candidate_batch_size);
  const auto commit_candidates = [&]() {
    if (candidates.empty()) {
      return true;
    }
    std::vector<std::pair<Point, Point>> chords;
    chords.reserve(candidates.size() * 6U);
    for (const auto& candidate : candidates) {
      const auto& corners = candidate.corners;
      chords.emplace_back(corners[0], corners[1]);
      chords.emplace_back(corners[1], corners[2]);
      chords.emplace_back(corners[2], corners[3]);
      chords.emplace_back(corners[3], corners[0]);
      chords.emplace_back(corners[0], corners[2]);
      chords.emplace_back(corners[1], corners[3]);
    }
    const auto certificates = assess_prepared_navmesh_chords(result, chords);
    for (std::size_t index = 0U; index < candidates.size(); ++index) {
      auto legal = true;
      for (std::size_t chord = 0U; chord < 6U; ++chord) {
        const auto& certificate = certificates[index * 6U + chord];
        if (!certificate || certificate->value() < config.minimum_boundary_clearance.value()) {
          legal = false;
          break;
        }
      }
      if (!legal) {
        continue;
      }
      if (result.cells.size() >= config.maximum_cells) {
        result.cell_budget_exhausted = true;
        result.diagnostic = "NavMesh cell budget exhausted";
        return false;
      }
      const auto& candidate = candidates[index];
      const auto id = result.cells.size();
      result.cells.push_back(NavMeshCell{id, candidate.corners, candidate.center});
      result.cell_by_coordinate[candidate.y * result.width + candidate.x] = id;
      result.coordinate_by_cell.emplace_back(candidate.x, candidate.y);
    }
    candidates.clear();
    return true;
  };
  for (std::size_t y = 0U; y < result.height; ++y) {
    for (std::size_t x = 0U; x < result.width; ++x) {
      const auto first =
          Point{Nanometres{result.minimum.x.value() + static_cast<std::int64_t>(x) * pitch},
                Nanometres{result.minimum.y.value() + static_cast<std::int64_t>(y) * pitch}};
      const auto opposite =
          Point{Nanometres{first.x.value() + pitch}, Nanometres{first.y.value() + pitch}};
      const std::array corners{first, Point{opposite.x, first.y}, opposite,
                               Point{first.x, opposite.y}};
      const auto center =
          Point{Nanometres{first.x.value() + pitch / 2}, Nanometres{first.y.value() + pitch / 2}};
      if (!config.topology_guide.empty()) {
        auto guide_distance = std::numeric_limits<long double>::infinity();
        for (std::size_t guide = 1U; guide < config.topology_guide.size(); ++guide) {
          guide_distance = std::min(guide_distance, geometry::point_segment_distance_nm(
                                                        center, config.topology_guide[guide - 1U],
                                                        config.topology_guide[guide]));
        }
        if (guide_distance > static_cast<long double>(config.maximum_guide_distance.value())) {
          continue;
        }
      }
      candidates.push_back(CellCandidate{x, y, corners, center});
      if (candidates.size() == candidate_batch_size && !commit_candidates()) {
        return result;
      }
    }
  }
  if (!commit_candidates()) {
    return result;
  }
  if (result.cells.empty()) {
    result.diagnostic = "NavMesh free-space decomposition is empty";
  }
  return result;
}

} // namespace

PreparedNavMeshDomain prepare_navmesh_domain(const OffsetFreeSpaceRegion& region,
                                             const NavMeshConfig& config) {
  return prepare_navmesh_domain_impl(region, config, true);
}

PreparedNavMeshDomain prepare_validated_navmesh_domain(const OffsetFreeSpaceRegion& region,
                                                       const NavMeshConfig& config) {
  return prepare_navmesh_domain_impl(region, config, false);
}

NavMeshTopologySignature make_navmesh_topology_signature(std::vector<NavMeshTopologyGate> gates) {
  NavMeshTopologySignature result;
  for (std::size_t index = 0U; index < gates.size(); ++index) {
    gates[index].ordinal = index;
  }
  result.gates = std::move(gates);
  result.valid = true;
  auto hash = topology_hash_offset;
  topology_hash_append(hash, static_cast<std::uint64_t>(result.gates.size()));
  for (const auto& gate : result.gates) {
    topology_hash_append(hash, static_cast<std::uint64_t>(gate.ordinal));
    topology_hash_append(hash, static_cast<std::uint64_t>(gate.from_cell));
    topology_hash_append(hash, static_cast<std::uint64_t>(gate.to_cell));
    topology_hash_append(hash, gate.left_boundary_id);
    topology_hash_append(hash, gate.right_boundary_id);
  }
  result.fingerprint = hash;
  return result;
}

NavMeshTopologySignature derive_navmesh_topology_signature(const NavMeshRouteResult& route) {
  NavMeshTopologySignature result;
  if (route.corridor.empty() || route.corridor.size() != route.portals.size() + 1U) {
    return result;
  }
  result.gates.reserve(route.portals.size());
  for (std::size_t index = 0U; index < route.portals.size(); ++index) {
    const auto from_cell = route.corridor[index];
    const auto to_cell = route.corridor[index + 1U];
    if (from_cell >= route.cells.size() || to_cell >= route.cells.size() || from_cell == to_cell) {
      result.gates.clear();
      return result;
    }
    const auto& portal = route.portals[index];
    NavMeshTopologyGate gate;
    gate.ordinal = index;
    gate.from_cell = from_cell;
    gate.to_cell = to_cell;
    gate.left_boundary = portal.left;
    gate.right_boundary = portal.right;
    gate.from_center = route.cells[from_cell].center;
    gate.to_center = route.cells[to_cell].center;
    gate.left_boundary_id = topology_boundary_site_id(gate.left_boundary);
    gate.right_boundary_id = topology_boundary_site_id(gate.right_boundary);
    result.gates.push_back(gate);
  }
  // A rectangular lattice portal is not intrinsically a homotopy cut: the funnel can legally
  // skip many internal cell boundaries while retaining the same obstacle-side decision.  Retain
  // only directed portals which the selected funnel trace itself crosses.  This turns sampled
  // cells into a compact, route-class certificate instead of requiring a spline to replay every
  // arbitrary grid transition.
  if (route.centerline.size() >= 2U && !result.gates.empty()) {
    std::vector<NavMeshTopologyGate> selected;
    selected.reserve(result.gates.size());
    for (const auto& gate : result.gates) {
      NavMeshTopologySignature singleton;
      singleton.gates = {gate};
      singleton.valid = true;
      if (validate_navmesh_topology_signature(singleton, route.centerline).preserved) {
        selected.push_back(gate);
      }
    }
    result.gates = std::move(selected);
  }
  return make_navmesh_topology_signature(std::move(result.gates));
}

NavMeshTopologyCheck validate_navmesh_topology_signature(
    const NavMeshTopologySignature& signature, const std::vector<Point>& polyline) {
  NavMeshTopologyCheck result;
  if (!signature.valid) {
    result.reason = NavMeshTopologyCheckReason::invalid_signature;
    result.diagnostic = "NavMesh topology signature is invalid";
    return result;
  }
  if (polyline.size() < 2U) {
    result.reason = NavMeshTopologyCheckReason::invalid_polyline;
    result.diagnostic = "topology candidate has fewer than two points";
    return result;
  }
  if (signature.gates.empty()) {
    result.preserved = true;
    result.reason = NavMeshTopologyCheckReason::preserved;
    result.diagnostic = "single-cell NavMesh topology preserved";
    return result;
  }

  std::vector<long double> prefix_distance(polyline.size(), 0.0L);
  for (std::size_t index = 1U; index < polyline.size(); ++index) {
    prefix_distance[index] = prefix_distance[index - 1U] +
                             geometry::segment_length_nm(polyline[index - 1U], polyline[index]);
  }
  std::optional<long double> prior_progress;
  for (const auto& gate : signature.gates) {
    std::vector<TopologyGateIntersection> intersections;
    for (std::size_t segment = 1U; segment < polyline.size(); ++segment) {
      const auto start = polyline[segment - 1U];
      const auto end = polyline[segment];
      const auto parameter = segment_gate_intersection_parameter(start, end, gate);
      if (!parameter) {
        continue;
      }
      const auto segment_length = geometry::segment_length_nm(start, end);
      if (!(segment_length > 0.0L)) {
        continue;
      }
      Point before = start;
      Point after = end;
      constexpr long double endpoint_tolerance = 1.0e-12L;
      if (*parameter <= endpoint_tolerance) {
        for (std::size_t index = segment - 1U; index > 0U; --index) {
          if (polyline[index - 1U] != start) {
            before = polyline[index - 1U];
            break;
          }
        }
      }
      if (*parameter >= 1.0L - endpoint_tolerance) {
        for (std::size_t index = segment + 1U; index < polyline.size(); ++index) {
          if (polyline[index] != end) {
            after = polyline[index];
            break;
          }
        }
      }
      const auto before_side = topology_side(gate, before);
      const auto after_side = topology_side(gate, after);
      const auto scale = std::max(
          1.0L, std::hypotl(static_cast<long double>(gate.to_center.x.value() -
                                                     gate.from_center.x.value()),
                             static_cast<long double>(gate.to_center.y.value() -
                                                     gate.from_center.y.value())));
      const auto side_tolerance = scale * 1.0e-9L;
      const auto forward = before_side < -side_tolerance && after_side > side_tolerance;
      const auto reversed = before_side > side_tolerance && after_side < -side_tolerance;
      // A funnel chord may touch a neighbouring portal corner while passing through the selected
      // gate sequence.  A tangential touch is neither a class transition nor evidence of a
      // repeated crossing, so retain only an actual signed-side change.
      if (!forward && !reversed) {
        continue;
      }
      const auto progress = prefix_distance[segment - 1U] + *parameter * segment_length;
      if (intersections.empty() ||
          std::abs(intersections.back().progress_nm - progress) > 0.5L) {
        intersections.push_back(TopologyGateIntersection{progress, forward});
      } else {
        intersections.back().forward = intersections.back().forward || forward;
      }
    }
    result.offending_gate = gate.ordinal;
    if (intersections.empty()) {
      result.reason = NavMeshTopologyCheckReason::missing_gate;
      result.diagnostic = "candidate did not cross selected NavMesh gate " +
                          std::to_string(gate.ordinal);
      return result;
    }
    if (intersections.size() != 1U) {
      result.reason = NavMeshTopologyCheckReason::repeated_gate;
      result.diagnostic = "candidate crossed selected NavMesh gate more than once: " +
                          std::to_string(gate.ordinal);
      return result;
    }
    if (!intersections.front().forward) {
      result.reason = NavMeshTopologyCheckReason::reversed_gate;
      result.diagnostic = "candidate crossed selected NavMesh gate in the reverse direction: " +
                          std::to_string(gate.ordinal);
      return result;
    }
    if (prior_progress && intersections.front().progress_nm + 0.5L < *prior_progress) {
      result.reason = NavMeshTopologyCheckReason::reordered_gate;
      result.diagnostic = "candidate crossed selected NavMesh gates out of order at gate " +
                          std::to_string(gate.ordinal);
      return result;
    }
    prior_progress = intersections.front().progress_nm;
    ++result.gates_crossed;
  }
  result.preserved = true;
  result.reason = NavMeshTopologyCheckReason::preserved;
  result.diagnostic = "directed NavMesh gate sequence preserved";
  return result;
}

NavMeshRouteResult
route_prepared_navmesh_flood_fill(const PreparedNavMeshDomain& prepared, Point start,
                                  const std::vector<Point>& goals, const NavMeshConfig& config,
                                  const std::vector<std::size_t>& dynamically_closed_cells) {
  NavMeshRouteResult result;
  result.start = start;
  result.used_flood_fill = true;
  auto ordered_goals = goals;
  std::sort(ordered_goals.begin(), ordered_goals.end());
  ordered_goals.erase(std::unique(ordered_goals.begin(), ordered_goals.end()), ordered_goals.end());
  if (!ordered_goals.empty()) {
    result.goal = ordered_goals.front();
  }
  if (ordered_goals.empty() || !prepared.ok() || config.maximum_search_expansions == 0U ||
      config.cell_pitch != prepared.pitch || config.topology_guide != prepared.topology_guide ||
      config.maximum_guide_distance != prepared.maximum_guide_distance ||
      config.minimum_boundary_clearance != prepared.minimum_boundary_clearance ||
      config.search_bounds != prepared.search_bounds) {
    result.cell_budget_exhausted = prepared.cell_budget_exhausted;
    result.diagnostic = prepared.diagnostic.empty() ? "invalid prepared NavMesh route request"
                                                    : prepared.diagnostic;
    return result;
  }
  result.cells = prepared.cells;
  result.cell_grid_minimum = prepared.minimum;
  result.cell_pitch = prepared.pitch;
  result.cell_grid_width = prepared.width;
  result.cell_grid_height = prepared.height;
  std::vector<std::uint8_t> closed(result.cells.size(), 0U);
  for (const auto cell : dynamically_closed_cells) {
    if (cell >= closed.size()) {
      result.diagnostic = "NavMesh dynamic flood wall references an invalid cell";
      return result;
    }
    closed[cell] = 1U;
  }
  const auto point_in_domain = [&](Point point) {
    return (!prepared.search_bounds || prepared.search_bounds->contains(point)) &&
           geometry::locate_point(prepared.prepared_outline, point) !=
               geometry::PointLocation::outside &&
           std::none_of(prepared.prepared_cutouts.begin(), prepared.prepared_cutouts.end(),
                        [&](const auto& cutout) {
                          return geometry::locate_point(cutout, point) !=
                                 geometry::PointLocation::outside;
                        });
  };
  if (!point_in_domain(start)) {
    result.diagnostic = "NavMesh endpoints are outside represented free space";
    return result;
  }
  const auto pitch = prepared.pitch.value();
  const auto endpoint_cell = [&](Point endpoint) -> std::optional<std::size_t> {
    const auto relative_x = endpoint.x.value() - prepared.minimum.x.value();
    const auto relative_y = endpoint.y.value() - prepared.minimum.y.value();
    const auto base_x = static_cast<std::int64_t>(
        std::floor(static_cast<long double>(relative_x) / static_cast<long double>(pitch)));
    const auto base_y = static_cast<std::int64_t>(
        std::floor(static_cast<long double>(relative_y) / static_cast<long double>(pitch)));
    for (auto dy = -1; dy <= 1; ++dy) {
      for (auto dx = -1; dx <= 1; ++dx) {
        const auto x = base_x + dx;
        const auto y = base_y + dy;
        if (x < 0 || y < 0 || x >= static_cast<std::int64_t>(prepared.width) ||
            y >= static_cast<std::int64_t>(prepared.height)) {
          continue;
        }
        const auto cell = prepared.cell_by_coordinate[static_cast<std::size_t>(y) * prepared.width +
                                                      static_cast<std::size_t>(x)];
        if (cell != std::numeric_limits<std::size_t>::max() &&
            result.cells[cell].contains(endpoint)) {
          return cell;
        }
      }
    }
    std::vector<std::pair<Point, Point>> endpoint_links;
    endpoint_links.reserve(result.cells.size());
    for (const auto& cell : result.cells) {
      if (cell.contains(endpoint)) {
        return cell.id;
      }
      endpoint_links.emplace_back(endpoint, cell.center);
    }
    const auto certificates = assess_prepared_navmesh_chords(prepared, endpoint_links);
    std::optional<std::size_t> selected;
    long double distance = std::numeric_limits<long double>::infinity();
    for (std::size_t index = 0U; index < result.cells.size(); ++index) {
      const auto& cell = result.cells[index];
      const auto candidate = geometry::point_distance_nm(endpoint, cell.center);
      if (candidate < distance && certificates[index]) {
        selected = cell.id;
        distance = candidate;
      }
    }
    return selected;
  };
  const auto start_cell = endpoint_cell(start);
  if (!start_cell || closed[*start_cell] != 0U) {
    result.diagnostic = "NavMesh cannot bind an endpoint to a legal cell";
    return result;
  }
  std::map<std::size_t, Point> goal_by_cell;
  for (const auto goal : ordered_goals) {
    if (!point_in_domain(goal)) {
      continue;
    }
    if (const auto cell = endpoint_cell(goal)) {
      if (closed[*cell] != 0U) {
        continue;
      }
      const auto found = goal_by_cell.find(*cell);
      if (found == goal_by_cell.end() || goal < found->second) {
        goal_by_cell[*cell] = goal;
      }
    }
  }
  if (goal_by_cell.empty()) {
    result.diagnostic = "NavMesh cannot bind a flood-fill goal to a legal cell";
    return result;
  }
  std::vector<std::size_t> distance(result.cells.size(), std::numeric_limits<std::size_t>::max());
  std::queue<std::size_t> queue;
  for (const auto& [cell, unused_goal] : goal_by_cell) {
    static_cast<void>(unused_goal);
    distance[cell] = 0U;
    queue.push(cell);
  }
  constexpr std::array<std::pair<int, int>, 4U> directions{std::pair{1, 0}, std::pair{-1, 0},
                                                           std::pair{0, 1}, std::pair{0, -1}};
  while (!queue.empty() && result.search_expansions < config.maximum_search_expansions) {
    const auto cell = queue.front();
    queue.pop();
    ++result.search_expansions;
    ++result.flood_fill_cells_visited;
    if (cell == *start_cell) {
      break;
    }
    const auto coordinate = prepared.coordinate_by_cell[cell];
    for (const auto [dx, dy] : directions) {
      const auto x = static_cast<std::int64_t>(coordinate.first) + dx;
      const auto y = static_cast<std::int64_t>(coordinate.second) + dy;
      if (x < 0 || y < 0) {
        continue;
      }
      if (x >= static_cast<std::int64_t>(prepared.width) ||
          y >= static_cast<std::int64_t>(prepared.height)) {
        continue;
      }
      const auto next = prepared.cell_by_coordinate[static_cast<std::size_t>(y) * prepared.width +
                                                    static_cast<std::size_t>(x)];
      if (next == std::numeric_limits<std::size_t>::max() || closed[next] != 0U) {
        continue;
      }
      if (distance[next] == std::numeric_limits<std::size_t>::max()) {
        distance[next] = distance[cell] + 1U;
        queue.push(next);
      }
    }
  }
  if (distance[*start_cell] == std::numeric_limits<std::size_t>::max()) {
    result.search_budget_exhausted = result.search_expansions >= config.maximum_search_expansions;
    result.diagnostic = result.search_budget_exhausted
                            ? "NavMesh flood-fill search budget exhausted"
                            : "NavMesh flood-fill corridor is disconnected";
    return result;
  }
  auto cell = *start_cell;
  result.corridor.push_back(cell);
  while (distance[cell] > 0U) {
    const auto coordinate = prepared.coordinate_by_cell[cell];
    std::optional<std::size_t> next_cell;
    for (const auto [dx, dy] : directions) {
      const auto x = static_cast<std::int64_t>(coordinate.first) + dx;
      const auto y = static_cast<std::int64_t>(coordinate.second) + dy;
      if (x < 0 || y < 0 || x >= static_cast<std::int64_t>(prepared.width) ||
          y >= static_cast<std::int64_t>(prepared.height)) {
        continue;
      }
      const auto next = prepared.cell_by_coordinate[static_cast<std::size_t>(y) * prepared.width +
                                                    static_cast<std::size_t>(x)];
      if (next != std::numeric_limits<std::size_t>::max() && closed[next] == 0U &&
          distance[next] + 1U == distance[cell]) {
        next_cell = next;
        break;
      }
    }
    if (!next_cell) {
      result.diagnostic = "NavMesh flood-fill gradient reconstruction failed";
      result.corridor.clear();
      return result;
    }
    cell = *next_cell;
    result.corridor.push_back(cell);
  }
  const auto selected_goal = goal_by_cell.find(result.corridor.back());
  if (selected_goal == goal_by_cell.end()) {
    result.diagnostic = "NavMesh flood-fill reached an unbound goal cell";
    result.corridor.clear();
    return result;
  }
  result.goal = selected_goal->second;
  const auto goal = result.goal;
  std::set<std::size_t> guard_cells(result.corridor.begin(), result.corridor.end());
  for (const auto selected : result.corridor) {
    const auto coordinate = prepared.coordinate_by_cell[selected];
    const auto rings = static_cast<std::int64_t>(config.corridor_guard_rings);
    for (auto dy = -rings; dy <= rings; ++dy) {
      for (auto dx = -rings; dx <= rings; ++dx) {
        const auto x = static_cast<std::int64_t>(coordinate.first) + dx;
        const auto y = static_cast<std::int64_t>(coordinate.second) + dy;
        if (x < 0 || y < 0) {
          continue;
        }
        if (x < static_cast<std::int64_t>(prepared.width) &&
            y < static_cast<std::int64_t>(prepared.height)) {
          const auto cell =
              prepared.cell_by_coordinate[static_cast<std::size_t>(y) * prepared.width +
                                          static_cast<std::size_t>(x)];
          if (cell != std::numeric_limits<std::size_t>::max() && closed[cell] == 0U) {
            guard_cells.insert(cell);
          }
        }
      }
    }
  }
  result.topology_guard_cells.assign(guard_cells.begin(), guard_cells.end());
  result.topology_guard_coordinate_mask.assign(prepared.width * prepared.height, 0U);
  for (const auto cell : result.topology_guard_cells) {
    const auto [x, y] = prepared.coordinate_by_cell[cell];
    result.topology_guard_coordinate_mask[y * prepared.width + x] = 1U;
  }
  const auto point_in_topology_guard = [&](Point point) {
    if (point == start || point == goal) {
      return true;
    }
    if (std::any_of(result.topology_guard_cells.begin(), result.topology_guard_cells.end(),
                    [&](const auto cell) {
                      return result.cells[cell].contains(point);
                    })) {
      return true;
    }
    const auto in_endpoint_link = [&](Point endpoint, std::size_t cell) {
      const auto radius =
          geometry::point_distance_nm(result.cells[cell].center, result.cells[cell].corners.front());
      return geometry::point_segment_distance_nm(point, endpoint, result.cells[cell].center) <=
             radius;
    };
    return in_endpoint_link(start, result.corridor.front()) ||
           in_endpoint_link(goal, result.corridor.back());
  };
  const auto chord_in_topology_guard = [&](Point first, Point last) {
    const auto length = geometry::segment_length_nm(first, last);
    const auto interval = static_cast<std::uint64_t>(std::max<std::int64_t>(1, pitch / 4));
    const auto samples = std::max<std::uint64_t>(1U, (length + interval - 1U) / interval);
    for (std::uint64_t sample = 0U; sample <= samples; ++sample) {
      const auto x = static_cast<long double>(first.x.value()) +
                     static_cast<long double>(last.x.value() - first.x.value()) *
                         static_cast<long double>(sample) / static_cast<long double>(samples);
      const auto y = static_cast<long double>(first.y.value()) +
                     static_cast<long double>(last.y.value() - first.y.value()) *
                         static_cast<long double>(sample) / static_cast<long double>(samples);
      if (!point_in_topology_guard(Point{Nanometres{static_cast<std::int64_t>(std::llround(x))},
                                         Nanometres{static_cast<std::int64_t>(std::llround(y))}})) {
        return false;
      }
    }
    return true;
  };
  for (std::size_t index = 1U; index < result.corridor.size(); ++index) {
    const auto& first = result.cells[result.corridor[index - 1U]];
    const auto& second = result.cells[result.corridor[index]];
    const auto dx = second.center.x.value() - first.center.x.value();
    const auto dy = second.center.y.value() - first.center.y.value();
    const auto half = pitch / 2;
    if (dx > 0) {
      result.portals.push_back(
          {{Nanometres{first.center.x.value() + half}, Nanometres{first.center.y.value() + half}},
           {Nanometres{first.center.x.value() + half}, Nanometres{first.center.y.value() - half}}});
    } else if (dx < 0) {
      result.portals.push_back(
          {{Nanometres{first.center.x.value() - half}, Nanometres{first.center.y.value() - half}},
           {Nanometres{first.center.x.value() - half}, Nanometres{first.center.y.value() + half}}});
    } else if (dy > 0) {
      result.portals.push_back(
          {{Nanometres{first.center.x.value() - half}, Nanometres{first.center.y.value() + half}},
           {Nanometres{first.center.x.value() + half}, Nanometres{first.center.y.value() + half}}});
    } else {
      result.portals.push_back(
          {{Nanometres{first.center.x.value() + half}, Nanometres{first.center.y.value() - half}},
           {Nanometres{first.center.x.value() - half}, Nanometres{first.center.y.value() - half}}});
    }
  }
  result.centerline = funnel(start, goal, result.portals);
  const auto legal_centerline = [&](const std::vector<Point>& centerline) {
    if (centerline.size() < 2U) {
      return false;
    }
    std::vector<std::pair<Point, Point>> chords;
    chords.reserve(centerline.size() - 1U);
    for (std::size_t index = 1U; index < centerline.size(); ++index) {
      chords.emplace_back(centerline[index - 1U], centerline[index]);
    }
    const auto certificates = assess_prepared_navmesh_chords(prepared, chords);
    for (std::size_t index = 1U; index < centerline.size(); ++index) {
      const auto& certificate = certificates[index - 1U];
      if (!certificate ||
          certificate->value() < std::max(pitch / 4, config.minimum_boundary_clearance.value()) ||
          !chord_in_topology_guard(centerline[index - 1U], centerline[index])) {
        return false;
      }
    }
    return true;
  };
  if (!legal_centerline(result.centerline)) {
    // Degenerate alternating portals can make the arithmetic funnel choose a chord outside a
    // conservative non-convex region. Retain the same selected cell sequence and perform bounded
    // visibility string-pulling over its centres; this is longer but preserves the corridor.
    std::vector<Point> chain{start};
    for (const auto cell : result.corridor) {
      if (chain.back() != result.cells[cell].center) {
        chain.push_back(result.cells[cell].center);
      }
    }
    if (chain.back() != goal) {
      chain.push_back(goal);
    }
    std::vector<Point> pulled{chain.front()};
    for (std::size_t first = 0U; first + 1U < chain.size();) {
      std::vector<std::pair<Point, Point>> chords;
      chords.reserve(chain.size() - first - 1U);
      for (auto last = chain.size() - 1U; last > first; --last) {
        chords.emplace_back(chain[first], chain[last]);
      }
      const auto certificates = assess_prepared_navmesh_chords(prepared, chords);
      auto selected = first;
      for (std::size_t index = 0U; index < chords.size(); ++index) {
        const auto last = chain.size() - 1U - index;
        const auto& certificate = certificates[index];
        if (certificate &&
            certificate->value() >=
                std::max(pitch / 4, config.minimum_boundary_clearance.value()) &&
            chord_in_topology_guard(chain[first], chain[last])) {
          selected = last;
          break;
        }
      }
      if (selected == first) {
        result.diagnostic = "funnel and conservative string pulling escaped represented free space";
        result.centerline.clear();
        return result;
      }
      pulled.push_back(chain[selected]);
      first = selected;
    }
    result.centerline = std::move(pulled);
    if (!legal_centerline(result.centerline)) {
      result.diagnostic = "funnel centerline escaped represented free space";
      result.centerline.clear();
      return result;
    }
  }
  result.topology_signature = derive_navmesh_topology_signature(result);
  const auto topology = validate_navmesh_topology_signature(result.topology_signature,
                                                            result.centerline);
  if (!topology.preserved) {
    result.diagnostic = "NavMesh funnel failed to preserve its directed gate sequence: " +
                        topology.diagnostic;
    result.centerline.clear();
  }
  return result;
}

NavMeshRouteResult route_prepared_navmesh(const PreparedNavMeshDomain& prepared, Point start,
                                          Point goal, const NavMeshConfig& config) {
  return route_prepared_navmesh_flood_fill(prepared, start, std::vector<Point>{goal}, config);
}

NavMeshRouteResult build_navmesh_funnel_route(const OffsetFreeSpaceRegion& region, Point start,
                                              Point goal, const NavMeshConfig& config) {
  const auto domain = prepare_navmesh_domain(region, config);
  return route_prepared_navmesh(domain, start, goal, config);
}

bool navmesh_corridor_contains(const NavMeshRouteResult& route, Point point) {
  if (!route.ok()) {
    return false;
  }
  if (point == route.start || point == route.goal) {
    return true;
  }
  if (route.cell_pitch.value() > 0 && route.cell_grid_width > 0U && route.cell_grid_height > 0U &&
      !route.topology_guard_coordinate_mask.empty()) {
    const auto relative_x = point.x.value() - route.cell_grid_minimum.x.value();
    const auto relative_y = point.y.value() - route.cell_grid_minimum.y.value();
    const auto base_x = static_cast<std::int64_t>(std::floor(
        static_cast<long double>(relative_x) / static_cast<long double>(route.cell_pitch.value())));
    const auto base_y = static_cast<std::int64_t>(std::floor(
        static_cast<long double>(relative_y) / static_cast<long double>(route.cell_pitch.value())));
    for (auto dy = -1; dy <= 1; ++dy) {
      for (auto dx = -1; dx <= 1; ++dx) {
        const auto x = base_x + dx;
        const auto y = base_y + dy;
        if (x < 0 || y < 0 || x >= static_cast<std::int64_t>(route.cell_grid_width) ||
            y >= static_cast<std::int64_t>(route.cell_grid_height)) {
          continue;
        }
        const auto index =
            static_cast<std::size_t>(y) * route.cell_grid_width + static_cast<std::size_t>(x);
        if (route.topology_guard_coordinate_mask[index] == 0U) {
          continue;
        }
        const auto minimum =
            Point{Nanometres{route.cell_grid_minimum.x.value() + x * route.cell_pitch.value()},
                  Nanometres{route.cell_grid_minimum.y.value() + y * route.cell_pitch.value()}};
        const auto maximum = Point{Nanometres{minimum.x.value() + route.cell_pitch.value()},
                                   Nanometres{minimum.y.value() + route.cell_pitch.value()}};
        if (minimum.x <= point.x && point.x <= maximum.x && minimum.y <= point.y &&
            point.y <= maximum.y) {
          return true;
        }
      }
    }
  } else {
    const auto& guard =
        route.topology_guard_cells.empty() ? route.corridor : route.topology_guard_cells;
    if (std::any_of(guard.begin(), guard.end(), [&](const auto cell) {
          return cell < route.cells.size() && route.cells[cell].contains(point);
        })) {
      return true;
    }
  }
  const auto in_endpoint_link = [&](Point endpoint, std::size_t cell) {
    if (cell >= route.cells.size()) {
      return false;
    }
    const auto radius =
        geometry::point_distance_nm(route.cells[cell].center, route.cells[cell].corners.front());
    return geometry::point_segment_distance_nm(point, endpoint, route.cells[cell].center) <= radius;
  };
  return in_endpoint_link(route.start, route.corridor.front()) ||
         in_endpoint_link(route.goal, route.corridor.back());
}

} // namespace flux
