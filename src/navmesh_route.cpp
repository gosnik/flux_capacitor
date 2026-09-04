#include "flux/core/navmesh_route.hpp"

#include "flux/core/free_space_skeleton.hpp"
#include "flux/core/geometry.hpp"
#include "flux/core/routing_compute.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <deque>
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

[[nodiscard]] geometry::AxisAlignedBounds union_bounds(const geometry::AxisAlignedBounds& first,
                                                       const geometry::AxisAlignedBounds& second) {
  return geometry::AxisAlignedBounds{Point{std::min(first.minimum.x, second.minimum.x),
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
  const auto middle_x = (static_cast<long double>(gate.left_boundary.x.value()) +
                         static_cast<long double>(gate.right_boundary.x.value())) /
                        2.0L;
  const auto middle_y = (static_cast<long double>(gate.left_boundary.y.value()) +
                         static_cast<long double>(gate.right_boundary.y.value())) /
                        2.0L;
  const auto normal_x =
      static_cast<long double>(gate.to_center.x.value() - gate.from_center.x.value());
  const auto normal_y =
      static_cast<long double>(gate.to_center.y.value() - gate.from_center.y.value());
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
  const auto gate_x =
      static_cast<long double>(gate.right_boundary.x.value() - gate.left_boundary.x.value());
  const auto gate_y =
      static_cast<long double>(gate.right_boundary.y.value() - gate.left_boundary.y.value());
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
      bounds = union_bounds(bounds,
                            prepared.prepared_cutouts[prepared.cutout_order[first + index]].bounds);
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
          width >= height ? lhs_bounds.minimum.x.value() +
                                (lhs_bounds.maximum.x.value() - lhs_bounds.minimum.x.value()) / 2
                          : lhs_bounds.minimum.y.value() +
                                (lhs_bounds.maximum.y.value() - lhs_bounds.minimum.y.value()) / 2;
      const auto rhs_coordinate =
          width >= height ? rhs_bounds.minimum.x.value() +
                                (rhs_bounds.maximum.x.value() - rhs_bounds.minimum.x.value()) / 2
                          : rhs_bounds.minimum.y.value() +
                                (rhs_bounds.maximum.y.value() - rhs_bounds.minimum.y.value()) / 2;
      return lhs_coordinate != rhs_coordinate ? lhs_coordinate < rhs_coordinate : lhs < rhs;
    };
    std::stable_sort(prepared.cutout_order.begin() + static_cast<std::ptrdiff_t>(first),
                     prepared.cutout_order.begin() + static_cast<std::ptrdiff_t>(first + count),
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
  std::vector<RoutingComputeSegment> compute_chords;
  compute_chords.reserve(chords.size());
  for (const auto& [start, end] : chords)
    compute_chords.push_back(RoutingComputeSegment{start.x.value(), start.y.value(), end.x.value(),
                                                   end.y.value(), 0, 0U, 0U});
  std::vector<RoutingComputeSegment> compute_boundaries;
  const auto append_boundaries = [&](const geometry::PreparedPolygon& polygon) {
    for (const auto& segment : polygon.boundary_segments)
      compute_boundaries.push_back(RoutingComputeSegment{
          segment.start.x.value(), segment.start.y.value(), segment.end.x.value(),
          segment.end.y.value(), 0, 0U, std::numeric_limits<std::uint32_t>::max()});
  };
  append_boundaries(prepared.prepared_outline);
  for (const auto& cutout : prepared.prepared_cutouts)
    append_boundaries(cutout);
  const auto gpu_classification = classify_routing_chords(
      compute_chords, compute_boundaries,
      static_cast<std::int64_t>(std::ceil(acceptance_clearance)), 2'048, true);
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
    const auto endpoint_in_cutout = std::any_of(
        prepared.prepared_cutouts.begin(), prepared.prepared_cutouts.end(),
        [&](const auto& cutout) {
          return geometry::locate_point(cutout, start) != geometry::PointLocation::outside ||
                 geometry::locate_point(cutout, end) != geometry::PointLocation::outside;
        });
    if (!endpoint_in_cutout && gpu_classification.ok() &&
        index < gpu_classification.chords.size() &&
        gpu_classification.chords[index].classification ==
            RoutingComputeChordClass::definitely_clear) {
      result[index] = Nanometres{gpu_classification.chords[index].conservative_clearance_nm};
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
        const auto& cutout = prepared.prepared_cutouts[prepared.cutout_order[node.first + offset]];
        if (bounds_distance_nm(bounds, cutout.bounds) > clearance) {
          continue;
        }
        const auto distance =
            geometry::segment_polygon_boundary_distance_nm_capped(start, end, cutout, clearance);
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

struct NavMeshFace {
  std::size_t cell{0U};
  std::int64_t minimum{0};
  std::int64_t maximum{0};
};

void build_navmesh_adjacency(PreparedNavMeshDomain& prepared) {
  std::map<std::int64_t, std::vector<NavMeshFace>> left_faces;
  std::map<std::int64_t, std::vector<NavMeshFace>> right_faces;
  std::map<std::int64_t, std::vector<NavMeshFace>> bottom_faces;
  std::map<std::int64_t, std::vector<NavMeshFace>> top_faces;
  prepared.adjacency_by_cell.assign(prepared.cells.size(), {});
  for (const auto& cell : prepared.cells) {
    const auto minimum_x = cell.corners[0].x.value();
    const auto minimum_y = cell.corners[0].y.value();
    const auto maximum_x = cell.corners[2].x.value();
    const auto maximum_y = cell.corners[2].y.value();
    left_faces[minimum_x].push_back(NavMeshFace{cell.id, minimum_y, maximum_y});
    right_faces[maximum_x].push_back(NavMeshFace{cell.id, minimum_y, maximum_y});
    bottom_faces[minimum_y].push_back(NavMeshFace{cell.id, minimum_x, maximum_x});
    top_faces[maximum_y].push_back(NavMeshFace{cell.id, minimum_x, maximum_x});
  }
  const auto ordered_faces = [](std::vector<NavMeshFace>& faces) {
    std::sort(faces.begin(), faces.end(), [](const auto& lhs, const auto& rhs) {
      return std::tie(lhs.minimum, lhs.maximum, lhs.cell) <
             std::tie(rhs.minimum, rhs.maximum, rhs.cell);
    });
  };
  for (auto& [unused, faces] : left_faces) {
    static_cast<void>(unused);
    ordered_faces(faces);
  }
  for (auto& [unused, faces] : right_faces) {
    static_cast<void>(unused);
    ordered_faces(faces);
  }
  for (auto& [unused, faces] : bottom_faces) {
    static_cast<void>(unused);
    ordered_faces(faces);
  }
  for (auto& [unused, faces] : top_faces) {
    static_cast<void>(unused);
    ordered_faces(faces);
  }
  const auto join_vertical = [&](std::int64_t x, const std::vector<NavMeshFace>& west,
                                 const std::vector<NavMeshFace>& east) {
    std::size_t first = 0U;
    std::size_t second = 0U;
    while (first < west.size() && second < east.size()) {
      const auto lower = std::max(west[first].minimum, east[second].minimum);
      const auto upper = std::min(west[first].maximum, east[second].maximum);
      if (lower < upper) {
        const auto aperture = Nanometres{upper - lower};
        prepared.adjacency_by_cell[west[first].cell].push_back(
            NavMeshAdjacency{east[second].cell,
                             NavMeshPortal{Point{Nanometres{x}, Nanometres{upper}},
                                           Point{Nanometres{x}, Nanometres{lower}}},
                             aperture});
        prepared.adjacency_by_cell[east[second].cell].push_back(
            NavMeshAdjacency{west[first].cell,
                             NavMeshPortal{Point{Nanometres{x}, Nanometres{lower}},
                                           Point{Nanometres{x}, Nanometres{upper}}},
                             aperture});
      }
      const auto first_maximum = west[first].maximum;
      const auto second_maximum = east[second].maximum;
      if (first_maximum <= second_maximum)
        ++first;
      if (second_maximum <= first_maximum)
        ++second;
    }
  };
  const auto join_horizontal = [&](std::int64_t y, const std::vector<NavMeshFace>& south,
                                   const std::vector<NavMeshFace>& north) {
    std::size_t first = 0U;
    std::size_t second = 0U;
    while (first < south.size() && second < north.size()) {
      const auto lower = std::max(south[first].minimum, north[second].minimum);
      const auto upper = std::min(south[first].maximum, north[second].maximum);
      if (lower < upper) {
        const auto aperture = Nanometres{upper - lower};
        prepared.adjacency_by_cell[south[first].cell].push_back(
            NavMeshAdjacency{north[second].cell,
                             NavMeshPortal{Point{Nanometres{lower}, Nanometres{y}},
                                           Point{Nanometres{upper}, Nanometres{y}}},
                             aperture});
        prepared.adjacency_by_cell[north[second].cell].push_back(
            NavMeshAdjacency{south[first].cell,
                             NavMeshPortal{Point{Nanometres{upper}, Nanometres{y}},
                                           Point{Nanometres{lower}, Nanometres{y}}},
                             aperture});
      }
      const auto first_maximum = south[first].maximum;
      const auto second_maximum = north[second].maximum;
      if (first_maximum <= second_maximum)
        ++first;
      if (second_maximum <= first_maximum)
        ++second;
    }
  };
  for (const auto& [coordinate, west] : right_faces) {
    const auto east = left_faces.find(coordinate);
    if (east != left_faces.end())
      join_vertical(coordinate, west, east->second);
  }
  for (const auto& [coordinate, south] : top_faces) {
    const auto north = bottom_faces.find(coordinate);
    if (north != bottom_faces.end())
      join_horizontal(coordinate, south, north->second);
  }
  for (std::size_t cell = 0U; cell < prepared.adjacency_by_cell.size(); ++cell) {
    auto& adjacency = prepared.adjacency_by_cell[cell];
    const auto& source = prepared.cells[cell].center;
    std::sort(adjacency.begin(), adjacency.end(), [&](const auto& lhs, const auto& rhs) {
      const auto rank = [&](const auto& edge) {
        const auto& target = prepared.cells[edge.cell].center;
        if (target.x > source.x)
          return 0;
        if (target.x < source.x)
          return 1;
        if (target.y > source.y)
          return 2;
        return 3;
      };
      return std::tuple{rank(lhs), lhs.portal.left, lhs.portal.right, lhs.cell} <
             std::tuple{rank(rhs), rhs.portal.left, rhs.portal.right, rhs.cell};
    });
  }
}

} // namespace

namespace {

PreparedNavMeshDomain prepare_navmesh_domain_impl(const OffsetFreeSpaceRegion& region,
                                                  const NavMeshConfig& config,
                                                  bool validate_source_topology) {
  PreparedNavMeshDomain result;
  if (config.cell_pitch.value() <= 0 || config.maximum_cells == 0U ||
      config.maximum_guide_distance.value() < 0 || config.minimum_boundary_clearance.value() < 0 ||
      config.minimum_refined_cell_pitch.value() < 0 ||
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
  result.adaptive_clearance_refinement = config.adaptive_clearance_refinement;
  result.maximum_refinement_depth = config.maximum_refinement_depth;
  result.minimum_refined_cell_pitch =
      config.minimum_refined_cell_pitch.value() > 0
          ? config.minimum_refined_cell_pitch
          : Nanometres{std::max<std::int64_t>(
                1, config.cell_pitch.value() /
                       static_cast<std::int64_t>(std::uint64_t{1U} << std::min<std::size_t>(
                                                     config.maximum_refinement_depth, 30U)))};
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
  result.cells_by_coordinate.resize(result.width * result.height);
  struct CellCandidate {
    std::size_t x{0U};
    std::size_t y{0U};
    std::array<Point, 4U> corners;
    Point center;
    std::size_t refinement_level{0U};
  };
  constexpr std::size_t candidate_batch_size = 2'048U;
  std::deque<CellCandidate> pending;
  const auto make_candidate = [](std::size_t x, std::size_t y, Point minimum, Point maximum,
                                 std::size_t level) {
    return CellCandidate{
        x,
        y,
        {minimum, Point{maximum.x, minimum.y}, maximum, Point{minimum.x, maximum.y}},
        Point{Nanometres{minimum.x.value() + (maximum.x.value() - minimum.x.value()) / 2},
              Nanometres{minimum.y.value() + (maximum.y.value() - minimum.y.value()) / 2}},
        level};
  };
  const auto intersects_guide = [&](const CellCandidate& candidate) {
    if (config.topology_guide.empty())
      return true;
    auto guide_distance = std::numeric_limits<long double>::infinity();
    for (std::size_t guide = 1U; guide < config.topology_guide.size(); ++guide) {
      guide_distance = std::min(
          guide_distance,
          geometry::point_segment_distance_nm(candidate.center, config.topology_guide[guide - 1U],
                                              config.topology_guide[guide]));
    }
    const auto half_diagonal =
        config.adaptive_clearance_refinement && candidate.refinement_level > 0U
            ? geometry::point_distance_nm(candidate.center, candidate.corners.front())
            : 0.0L;
    return guide_distance <=
           static_cast<long double>(config.maximum_guide_distance.value()) + half_diagonal;
  };
  for (std::size_t y = 0U; y < result.height; ++y) {
    for (std::size_t x = 0U; x < result.width; ++x) {
      const auto first =
          Point{Nanometres{result.minimum.x.value() + static_cast<std::int64_t>(x) * pitch},
                Nanometres{result.minimum.y.value() + static_cast<std::int64_t>(y) * pitch}};
      const auto opposite =
          Point{Nanometres{first.x.value() + pitch}, Nanometres{first.y.value() + pitch}};
      auto candidate = make_candidate(x, y, first, opposite, 0U);
      if (intersects_guide(candidate))
        pending.push_back(std::move(candidate));
    }
  }
  std::vector<CellCandidate> candidates;
  candidates.reserve(candidate_batch_size);
  while (!pending.empty()) {
    candidates.clear();
    while (!pending.empty() && candidates.size() < candidate_batch_size) {
      candidates.push_back(std::move(pending.front()));
      pending.pop_front();
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
        const auto& candidate = candidates[index];
        const auto cell_width = candidate.corners[2].x.value() - candidate.corners[0].x.value();
        const auto cell_height = candidate.corners[2].y.value() - candidate.corners[0].y.value();
        const auto half_diagonal =
            geometry::point_distance_nm(candidate.center, candidate.corners.front());
        const auto center_query = query_prepared_navmesh_point(result, candidate.center);
        const auto can_intersect_free_space =
            center_query.inside || center_query.boundary_distance_nm <= half_diagonal + 1.0L;
        const auto can_refine = config.adaptive_clearance_refinement &&
                                candidate.refinement_level < config.maximum_refinement_depth &&
                                cell_width > result.minimum_refined_cell_pitch.value() &&
                                cell_height > result.minimum_refined_cell_pitch.value() &&
                                cell_width >= 2 && cell_height >= 2 && can_intersect_free_space;
        if (!can_refine) {
          ++result.rejected_leaf_cells;
          continue;
        }
        const auto middle_x = candidate.corners[0].x.value() + cell_width / 2;
        const auto middle_y = candidate.corners[0].y.value() + cell_height / 2;
        if (middle_x <= candidate.corners[0].x.value() ||
            middle_x >= candidate.corners[2].x.value() ||
            middle_y <= candidate.corners[0].y.value() ||
            middle_y >= candidate.corners[2].y.value()) {
          ++result.rejected_leaf_cells;
          continue;
        }
        ++result.refined_parent_cells;
        const auto level = candidate.refinement_level + 1U;
        const std::array children{
            make_candidate(candidate.x, candidate.y, candidate.corners[0],
                           Point{Nanometres{middle_x}, Nanometres{middle_y}}, level),
            make_candidate(candidate.x, candidate.y,
                           Point{Nanometres{middle_x}, candidate.corners[0].y},
                           Point{candidate.corners[2].x, Nanometres{middle_y}}, level),
            make_candidate(candidate.x, candidate.y,
                           Point{candidate.corners[0].x, Nanometres{middle_y}},
                           Point{Nanometres{middle_x}, candidate.corners[2].y}, level),
            make_candidate(candidate.x, candidate.y,
                           Point{Nanometres{middle_x}, Nanometres{middle_y}}, candidate.corners[2],
                           level)};
        for (auto child : children) {
          if (intersects_guide(child))
            pending.push_back(std::move(child));
        }
        continue;
      }
      if (result.cells.size() >= config.maximum_cells) {
        result.cell_budget_exhausted = true;
        result.diagnostic = "NavMesh cell budget exhausted";
        return result;
      }
      const auto& candidate = candidates[index];
      const auto id = result.cells.size();
      result.cells.push_back(
          NavMeshCell{id, candidate.corners, candidate.center, candidate.refinement_level});
      const auto coordinate = candidate.y * result.width + candidate.x;
      if (result.cell_by_coordinate[coordinate] == no_cell) {
        result.cell_by_coordinate[coordinate] = id;
      }
      result.cells_by_coordinate[coordinate].push_back(id);
      result.coordinate_by_cell.emplace_back(candidate.x, candidate.y);
      if (candidate.refinement_level > 0U)
        ++result.refined_leaf_cells;
      result.maximum_realized_refinement_depth =
          std::max(result.maximum_realized_refinement_depth, candidate.refinement_level);
    }
  }
  if (result.cells.empty()) {
    result.diagnostic = "NavMesh free-space decomposition is empty";
    return result;
  }
  build_navmesh_adjacency(result);
  return result;
}

} // namespace

PreparedNavMeshDomain prepare_navmesh_domain(const OffsetFreeSpaceRegion& region,
                                             const NavMeshConfig& config) {
  return prepare_navmesh_domain_impl(region, config, true);
}

PreparedNavMeshPointQuery query_prepared_navmesh_point(const PreparedNavMeshDomain& domain,
                                                       Point point) {
  PreparedNavMeshPointQuery result;
  if (!domain.prepared_outline.valid())
    return result;
  const auto point_bounds_distance = [&](const geometry::AxisAlignedBounds& bounds) {
    const auto dx = point.x < bounds.minimum.x   ? bounds.minimum.x.value() - point.x.value()
                    : point.x > bounds.maximum.x ? point.x.value() - bounds.maximum.x.value()
                                                 : 0;
    const auto dy = point.y < bounds.minimum.y   ? bounds.minimum.y.value() - point.y.value()
                    : point.y > bounds.maximum.y ? point.y.value() - bounds.maximum.y.value()
                                                 : 0;
    return std::hypotl(static_cast<long double>(dx), static_cast<long double>(dy));
  };
  result.inside =
      geometry::locate_point(domain.prepared_outline, point) != geometry::PointLocation::outside;
  auto distance = geometry::point_polygon_boundary_distance_nm(point, domain.prepared_outline);
  std::vector<std::size_t> pending;
  pending.reserve(64U);
  if (!domain.cutout_bvh.empty())
    pending.push_back(0U);
  while (!pending.empty()) {
    const auto node_index = pending.back();
    pending.pop_back();
    const auto& node = domain.cutout_bvh[node_index];
    if (point_bounds_distance(node.bounds) > distance && !node.bounds.contains(point))
      continue;
    if (!node.leaf()) {
      pending.push_back(node.right);
      pending.push_back(node.left);
      continue;
    }
    for (std::size_t offset = 0U; offset < node.count; ++offset) {
      const auto& cutout = domain.prepared_cutouts[domain.cutout_order[node.first + offset]];
      const auto lower_bound = point_bounds_distance(cutout.bounds);
      if (lower_bound <= distance) {
        distance = std::min(distance, geometry::point_polygon_boundary_distance_nm(point, cutout));
      }
      if (result.inside && cutout.bounds.contains(point) &&
          geometry::locate_point(cutout, point) != geometry::PointLocation::outside) {
        result.inside = false;
      }
    }
  }
  result.boundary_distance_nm = distance;
  return result;
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

NavMeshTopologyCheck validate_navmesh_topology_signature(const NavMeshTopologySignature& signature,
                                                         const std::vector<Point>& polyline) {
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
          1.0L,
          std::hypotl(
              static_cast<long double>(gate.to_center.x.value() - gate.from_center.x.value()),
              static_cast<long double>(gate.to_center.y.value() - gate.from_center.y.value())));
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
      if (intersections.empty() || std::abs(intersections.back().progress_nm - progress) > 0.5L) {
        intersections.push_back(TopologyGateIntersection{progress, forward});
      } else {
        intersections.back().forward = intersections.back().forward || forward;
      }
    }
    result.offending_gate = gate.ordinal;
    if (intersections.empty()) {
      result.reason = NavMeshTopologyCheckReason::missing_gate;
      result.diagnostic =
          "candidate did not cross selected NavMesh gate " + std::to_string(gate.ordinal);
      return result;
    }
    if (intersections.size() != 1U) {
      result.reason = NavMeshTopologyCheckReason::repeated_gate;
      result.diagnostic =
          "candidate crossed selected NavMesh gate more than once: " + std::to_string(gate.ordinal);
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
      config.search_bounds != prepared.search_bounds ||
      config.adaptive_clearance_refinement != prepared.adaptive_clearance_refinement ||
      config.maximum_refinement_depth != prepared.maximum_refinement_depth ||
      (config.minimum_refined_cell_pitch.value() > 0
           ? config.minimum_refined_cell_pitch != prepared.minimum_refined_cell_pitch
           : false)) {
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
  result.adaptive_refined_parent_cells = prepared.refined_parent_cells;
  result.adaptive_refined_leaf_cells = prepared.refined_leaf_cells;
  result.adaptive_rejected_leaf_cells = prepared.rejected_leaf_cells;
  result.adaptive_maximum_refinement_depth = prepared.maximum_realized_refinement_depth;
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
        const auto coordinate =
            static_cast<std::size_t>(y) * prepared.width + static_cast<std::size_t>(x);
        for (const auto cell : prepared.cells_by_coordinate[coordinate]) {
          if (result.cells[cell].contains(endpoint))
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
  const auto cost_pitch = static_cast<long double>(
      std::max<std::int64_t>(1, prepared.minimum_refined_cell_pitch.value()));
  const auto transition_cost = [&](std::size_t first, std::size_t second) {
    if (prepared.maximum_realized_refinement_depth == 0U)
      return std::uint32_t{1U};
    const auto physical =
        geometry::point_distance_nm(result.cells[first].center, result.cells[second].center);
    return static_cast<std::uint32_t>(std::clamp<long double>(
        std::ceil(physical / cost_pitch), 1.0L,
        static_cast<long double>(std::numeric_limits<std::uint32_t>::max() - 1U)));
  };
  const auto ordered_neighbours = [&](std::size_t current) {
    std::vector<std::size_t> neighbours;
    if (prepared.maximum_realized_refinement_depth > 0U) {
      neighbours.reserve(prepared.adjacency_by_cell[current].size());
      for (const auto& adjacency : prepared.adjacency_by_cell[current]) {
        neighbours.push_back(adjacency.cell);
      }
      return neighbours;
    }
    const auto [source_x, source_y] = prepared.coordinate_by_cell[current];
    for (const auto [dx, dy] : std::array<std::pair<int, int>, 4U>{
             std::pair{1, 0}, std::pair{-1, 0}, std::pair{0, 1}, std::pair{0, -1}}) {
      const auto x = static_cast<std::int64_t>(source_x) + dx;
      const auto y = static_cast<std::int64_t>(source_y) + dy;
      if (x < 0 || y < 0 || x >= static_cast<std::int64_t>(prepared.width) ||
          y >= static_cast<std::int64_t>(prepared.height)) {
        continue;
      }
      const auto next = prepared.cell_by_coordinate[static_cast<std::size_t>(y) * prepared.width +
                                                    static_cast<std::size_t>(x)];
      if (next != std::numeric_limits<std::size_t>::max())
        neighbours.push_back(next);
    }
    return neighbours;
  };
  RoutingComputeWavefrontResult computed;
  if (prepared.maximum_realized_refinement_depth == 0U) {
    auto gpu_complete = false;
    if (prepared.width * prepared.height >= 4'096U) {
      RoutingComputeWavefrontInput raster;
      raster.width = prepared.width;
      raster.height = prepared.height;
      raster.traversable.assign(prepared.width * prepared.height, 0U);
      for (std::size_t current = 0U; current < prepared.cells.size(); ++current) {
        if (closed[current] == 0U) {
          const auto [x, y] = prepared.coordinate_by_cell[current];
          raster.traversable[y * prepared.width + x] = 1U;
        }
      }
      for (const auto& [goal_cell, unused_goal] : goal_by_cell) {
        static_cast<void>(unused_goal);
        const auto [x, y] = prepared.coordinate_by_cell[goal_cell];
        raster.sources.push_back(y * prepared.width + x);
      }
      raster.maximum_passes =
          std::min(config.maximum_search_expansions,
                   std::max<std::size_t>(256U, 4U * (prepared.width + prepared.height)));
      const auto raster_result = compute_routing_wavefront(raster, true);
      result.used_gpu_wavefront = raster_result.used_gpu;
      result.gpu_wavefront_cpu_fallback = raster_result.cpu_fallback;
      const auto [start_x, start_y] = prepared.coordinate_by_cell[*start_cell];
      const auto start_coordinate = start_y * prepared.width + start_x;
      if (raster_result.ok() &&
          raster_result.distance[start_coordinate] != std::numeric_limits<std::uint32_t>::max()) {
        computed = raster_result;
        std::vector<std::uint32_t> cell_distance(result.cells.size(),
                                                 std::numeric_limits<std::uint32_t>::max());
        for (std::size_t current = 0U; current < prepared.cells.size(); ++current) {
          const auto [x, y] = prepared.coordinate_by_cell[current];
          cell_distance[current] = computed.distance[y * prepared.width + x];
        }
        computed.distance = std::move(cell_distance);
        gpu_complete = true;
      }
    }
    if (!gpu_complete) {
      computed.distance.assign(result.cells.size(), std::numeric_limits<std::uint32_t>::max());
      std::queue<std::size_t> queue;
      for (const auto& [goal_cell, unused_goal] : goal_by_cell) {
        static_cast<void>(unused_goal);
        computed.distance[goal_cell] = 0U;
        queue.push(goal_cell);
      }
      while (!queue.empty() && computed.passes < config.maximum_search_expansions) {
        const auto current = queue.front();
        queue.pop();
        ++computed.passes;
        if (current == *start_cell)
          break;
        for (const auto neighbour : ordered_neighbours(current)) {
          if (closed[neighbour] != 0U ||
              computed.distance[neighbour] != std::numeric_limits<std::uint32_t>::max()) {
            continue;
          }
          computed.distance[neighbour] = computed.distance[current] + 1U;
          queue.push(neighbour);
        }
      }
      computed.budget_exhausted = !queue.empty();
    }
  } else {
    RoutingComputeGraphWavefrontInput wavefront;
    wavefront.node_count = result.cells.size();
    wavefront.offsets.reserve(result.cells.size() + 1U);
    for (std::size_t current = 0U; current < result.cells.size(); ++current) {
      wavefront.offsets.push_back(static_cast<std::uint32_t>(wavefront.edges.size()));
      if (closed[current] != 0U)
        continue;
      for (const auto& adjacency : prepared.adjacency_by_cell[current]) {
        if (closed[adjacency.cell] != 0U)
          continue;
        wavefront.edges.push_back(RoutingComputeGraphEdge{
            static_cast<std::uint32_t>(adjacency.cell), transition_cost(current, adjacency.cell)});
      }
    }
    wavefront.offsets.push_back(static_cast<std::uint32_t>(wavefront.edges.size()));
    for (const auto& [goal_cell, unused_goal] : goal_by_cell) {
      static_cast<void>(unused_goal);
      wavefront.sources.push_back(goal_cell);
    }
    wavefront.maximum_passes = config.maximum_search_expansions;
    computed = compute_routing_graph_wavefront(wavefront, true);
    result.used_gpu_wavefront = computed.used_gpu;
    result.gpu_wavefront_cpu_fallback = computed.cpu_fallback;
    if (computed.ok() && computed.used_gpu &&
        computed.distance[*start_cell] == std::numeric_limits<std::uint32_t>::max()) {
      auto exact = compute_routing_graph_wavefront(wavefront, false);
      exact.used_gpu = true;
      exact.cpu_fallback = true;
      computed = std::move(exact);
    }
  }
  if (!computed.ok()) {
    result.diagnostic = computed.diagnostic;
    return result;
  }
  result.search_expansions = computed.passes;
  result.flood_fill_cells_visited = static_cast<std::size_t>(
      std::count_if(computed.distance.begin(), computed.distance.end(), [](const auto value) {
        return value != std::numeric_limits<std::uint32_t>::max();
      }));
  const auto& distance = computed.distance;
  if (distance[*start_cell] == std::numeric_limits<std::uint32_t>::max()) {
    result.search_budget_exhausted = result.search_expansions >= config.maximum_search_expansions;
    result.diagnostic = result.search_budget_exhausted
                            ? "NavMesh flood-fill search budget exhausted"
                            : "NavMesh flood-fill corridor is disconnected";
    return result;
  }
  auto cell = *start_cell;
  result.corridor.push_back(cell);
  while (distance[cell] > 0U) {
    std::optional<std::size_t> next_cell;
    for (const auto next : ordered_neighbours(cell)) {
      if (closed[next] != 0U || distance[next] == std::numeric_limits<std::uint32_t>::max()) {
        continue;
      }
      const auto cost = transition_cost(cell, next);
      if (distance[next] <= std::numeric_limits<std::uint32_t>::max() - cost &&
          distance[next] + cost == distance[cell]) {
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
          const auto coordinate =
              static_cast<std::size_t>(y) * prepared.width + static_cast<std::size_t>(x);
          for (const auto cell : prepared.cells_by_coordinate[coordinate]) {
            if (closed[cell] == 0U)
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
                    [&](const auto cell) { return result.cells[cell].contains(point); })) {
      return true;
    }
    const auto in_endpoint_link = [&](Point endpoint, std::size_t cell) {
      const auto radius = geometry::point_distance_nm(result.cells[cell].center,
                                                      result.cells[cell].corners.front());
      return geometry::point_segment_distance_nm(point, endpoint, result.cells[cell].center) <=
             radius;
    };
    return in_endpoint_link(start, result.corridor.front()) ||
           in_endpoint_link(goal, result.corridor.back());
  };
  const auto chord_in_topology_guard = [&](Point first, Point last) {
    const auto length = geometry::segment_length_nm(first, last);
    const auto interval = static_cast<std::uint64_t>(
        std::max<std::int64_t>(1, (prepared.maximum_realized_refinement_depth > 0U
                                       ? prepared.minimum_refined_cell_pitch.value()
                                       : pitch) /
                                      4));
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
    const auto from = result.corridor[index - 1U];
    const auto to = result.corridor[index];
    if (prepared.maximum_realized_refinement_depth == 0U) {
      const auto& first = result.cells[from];
      const auto& second = result.cells[to];
      const auto dx = second.center.x.value() - first.center.x.value();
      const auto dy = second.center.y.value() - first.center.y.value();
      const auto half = pitch / 2;
      if (dx > 0) {
        result.portals.push_back(
            {{Nanometres{first.center.x.value() + half}, Nanometres{first.center.y.value() + half}},
             {Nanometres{first.center.x.value() + half},
              Nanometres{first.center.y.value() - half}}});
      } else if (dx < 0) {
        result.portals.push_back(
            {{Nanometres{first.center.x.value() - half}, Nanometres{first.center.y.value() - half}},
             {Nanometres{first.center.x.value() - half},
              Nanometres{first.center.y.value() + half}}});
      } else if (dy > 0) {
        result.portals.push_back(
            {{Nanometres{first.center.x.value() - half}, Nanometres{first.center.y.value() + half}},
             {Nanometres{first.center.x.value() + half},
              Nanometres{first.center.y.value() + half}}});
      } else {
        result.portals.push_back(
            {{Nanometres{first.center.x.value() + half}, Nanometres{first.center.y.value() - half}},
             {Nanometres{first.center.x.value() - half},
              Nanometres{first.center.y.value() - half}}});
      }
      continue;
    }
    const auto found = std::find_if(prepared.adjacency_by_cell[from].begin(),
                                    prepared.adjacency_by_cell[from].end(),
                                    [&](const auto& adjacency) { return adjacency.cell == to; });
    if (found == prepared.adjacency_by_cell[from].end() || found->aperture.value() <= 0) {
      result.diagnostic = "NavMesh corridor contains a missing or collapsed adaptive portal";
      result.centerline.clear();
      result.corridor.clear();
      return result;
    }
    result.portals.push_back(found->portal);
  }
  result.centerline = funnel(start, goal, result.portals);
  const auto centerline_scale = prepared.maximum_realized_refinement_depth > 0U
                                    ? prepared.minimum_refined_cell_pitch.value()
                                    : pitch;
  const auto centerline_clearance = Nanometres{
      std::max<std::int64_t>(centerline_scale / 4, config.minimum_boundary_clearance.value())};
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
      if (!certificate || certificate->value() < centerline_clearance.value() ||
          !chord_in_topology_guard(centerline[index - 1U], centerline[index])) {
        return false;
      }
    }
    return true;
  };
  if (!legal_centerline(result.centerline)) {
    // The source topology guide predates rasterization and may cross cell boundaries that the
    // sampled guard cannot replay exactly. It remains a valid fallback only when every complete
    // chord independently passes the prepared polygonal free-space certificate. The signature is
    // then derived from this guide, so it does not inherit a different funnel homotopy.
    const auto legal_source_guide = [&] {
      if (prepared.topology_guide.size() < 2U || prepared.topology_guide.front() != start ||
          prepared.topology_guide.back() != goal) {
        return false;
      }
      std::vector<std::pair<Point, Point>> chords;
      chords.reserve(prepared.topology_guide.size() - 1U);
      for (std::size_t point = 1U; point < prepared.topology_guide.size(); ++point) {
        chords.emplace_back(prepared.topology_guide[point - 1U], prepared.topology_guide[point]);
      }
      const auto certificates = assess_prepared_navmesh_chords(prepared, chords);
      return std::all_of(certificates.begin(), certificates.end(), [&](const auto& certificate) {
        return certificate && certificate->value() >= config.minimum_boundary_clearance.value();
      });
    }();
    if (legal_source_guide) {
      result.centerline = prepared.topology_guide;
    } else {
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
          if (certificate && certificate->value() >= centerline_clearance.value() &&
              chord_in_topology_guard(chain[first], chain[last])) {
            selected = last;
            break;
          }
        }
        if (selected == first) {
          result.diagnostic = "funnel, source guide, and conservative string pulling escaped "
                              "represented free space";
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
  }
  result.topology_signature = derive_navmesh_topology_signature(result);
  const auto topology =
      validate_navmesh_topology_signature(result.topology_signature, result.centerline);
  if (!topology.preserved) {
    result.diagnostic =
        "NavMesh funnel failed to preserve its directed gate sequence: " + topology.diagnostic;
    result.centerline.clear();
  }
  return result;
}

NavMeshRouteResult route_prepared_navmesh(const PreparedNavMeshDomain& prepared, Point start,
                                          Point goal, const NavMeshConfig& config) {
  return route_prepared_navmesh_flood_fill(prepared, start, std::vector<Point>{goal}, config);
}

NavMeshRouteResult route_prepared_navmesh_rrt_connect(const PreparedNavMeshDomain& prepared,
                                                      Point start, Point goal,
                                                      const NavMeshConfig& navmesh_config,
                                                      const NavMeshRrtConnectConfig& rrt_config) {
  // Reuse endpoint binding and the prepared-domain validity checks.  The returned route is not
  // used as the alternative: only its bound endpoint cells seed the two RRT trees.
  const auto baseline = route_prepared_navmesh(prepared, start, goal, navmesh_config);
  if (!baseline.ok()) {
    return baseline;
  }
  NavMeshRouteResult failed;
  failed.start = start;
  failed.goal = goal;
  failed.cells = prepared.cells;
  failed.cell_grid_minimum = prepared.minimum;
  failed.cell_pitch = prepared.pitch;
  failed.cell_grid_width = prepared.width;
  failed.cell_grid_height = prepared.height;
  if (rrt_config.maximum_iterations == 0U || rrt_config.goal_bias_period == 0U ||
      baseline.corridor.empty()) {
    failed.diagnostic = "invalid deterministic NavMesh RRT-Connect request";
    return failed;
  }
  const auto start_cell = baseline.corridor.front();
  const auto goal_cell = baseline.corridor.back();
  if (start_cell == goal_cell) {
    auto result = baseline;
    result.used_rrt_connect = true;
    return result;
  }

  struct Tree {
    std::vector<std::size_t> nodes;
    std::vector<std::size_t> parent_by_cell;
    std::vector<std::uint8_t> present;
  };
  const auto make_tree = [&](std::size_t root) {
    Tree tree;
    tree.nodes.push_back(root);
    tree.parent_by_cell.assign(prepared.cells.size(), std::numeric_limits<std::size_t>::max());
    tree.present.assign(prepared.cells.size(), 0U);
    tree.parent_by_cell[root] = root;
    tree.present[root] = 1U;
    return tree;
  };
  auto start_tree = make_tree(start_cell);
  auto goal_tree = make_tree(goal_cell);
  const auto neighbours = [&](std::size_t cell) {
    std::vector<std::size_t> result;
    result.reserve(prepared.adjacency_by_cell[cell].size());
    for (const auto& adjacency : prepared.adjacency_by_cell[cell]) {
      result.push_back(adjacency.cell);
    }
    return result;
  };
  const auto cell_distance = [&](std::size_t first, std::size_t second) {
    return std::pair{static_cast<std::uint64_t>(std::ceil(geometry::point_distance_nm(
                         prepared.cells[first].center, prepared.cells[second].center))),
                     first};
  };
  const auto nearest = [&](const Tree& tree, std::size_t target) {
    if (tree.nodes.size() >= 4'096U && routing_compute_gpu_available()) {
      const auto target_point = prepared.cells[target].center;
      std::vector<RoutingComputeSegment> sites;
      sites.reserve(tree.nodes.size());
      for (std::size_t index = 0U; index < tree.nodes.size(); ++index) {
        const auto center = prepared.cells[tree.nodes[index]].center;
        sites.push_back(RoutingComputeSegment{
            center.x.value(), center.y.value(), center.x.value(), center.y.value(), 0, 0U,
            static_cast<std::uint32_t>(index)});
      }
      const auto result = probe_routing_proximity(
          {RoutingComputePoint{target_point.x.value(), target_point.y.value(), 0, 0U,
                               std::numeric_limits<std::uint32_t>::max()}},
          sites, 0, true);
      if (result.ok() && result.used_gpu && !result.queries.empty() &&
          result.queries.front().closest_segment < tree.nodes.size())
        return tree.nodes[result.queries.front().closest_segment];
    }
    auto selected = tree.nodes.front();
    auto score = cell_distance(selected, target);
    for (const auto cell : tree.nodes) {
      const auto candidate = cell_distance(cell, target);
      if (candidate < score) {
        score = candidate;
        selected = cell;
      }
    }
    return selected;
  };
  const auto grow_once = [&](Tree& tree, std::size_t target) -> std::optional<std::size_t> {
    const auto source = nearest(tree, target);
    std::optional<std::size_t> selected;
    std::pair<std::uint64_t, std::size_t> selected_score{std::numeric_limits<std::uint64_t>::max(),
                                                         std::numeric_limits<std::size_t>::max()};
    for (const auto cell : neighbours(source)) {
      if (cell == std::numeric_limits<std::size_t>::max() || tree.present[cell] != 0U) {
        continue;
      }
      const auto score = cell_distance(cell, target);
      if (score < selected_score) {
        selected = cell;
        selected_score = score;
      }
    }
    if (!selected) {
      return std::nullopt;
    }
    tree.nodes.push_back(*selected);
    tree.parent_by_cell[*selected] = source;
    tree.present[*selected] = 1U;
    return selected;
  };
  const auto touching = [&](std::size_t cell, const Tree& other) -> std::optional<std::size_t> {
    if (other.present[cell] != 0U) {
      return cell;
    }
    std::optional<std::size_t> selected;
    for (const auto neighbour : neighbours(cell)) {
      if (neighbour != std::numeric_limits<std::size_t>::max() && other.present[neighbour] != 0U &&
          (!selected || neighbour < *selected)) {
        selected = neighbour;
      }
    }
    return selected;
  };
  const auto halton = [](std::uint64_t index, std::uint64_t base) {
    long double result = 0.0L;
    long double factor = 1.0L / static_cast<long double>(base);
    while (index > 0U) {
      result += factor * static_cast<long double>(index % base);
      index /= base;
      factor /= static_cast<long double>(base);
    }
    return result;
  };
  const auto sampled_cell = [&](std::size_t iteration, bool grow_start) {
    if (iteration % rrt_config.goal_bias_period == 0U) {
      return grow_start ? goal_cell : start_cell;
    }
    const auto index =
        static_cast<std::uint64_t>(iteration + 1U) + rrt_config.deterministic_seed * 2'053U;
    const auto x = std::min(
        prepared.width - 1U,
        static_cast<std::size_t>(halton(index, 2U) * static_cast<long double>(prepared.width)));
    const auto y = std::min(
        prepared.height - 1U,
        static_cast<std::size_t>(halton(index, 3U) * static_cast<long double>(prepared.height)));
    const auto& cells = prepared.cells_by_coordinate[y * prepared.width + x];
    if (!cells.empty()) {
      return cells[(index + rrt_config.deterministic_seed) % cells.size()];
    }
    return prepared.cells[(index + rrt_config.deterministic_seed) % prepared.cells.size()].id;
  };

  std::optional<std::pair<std::size_t, std::size_t>> meeting;
  std::size_t iterations = 0U;
  for (; iterations < rrt_config.maximum_iterations && !meeting; ++iterations) {
    const bool grow_start = (iterations & 1U) == 0U;
    auto& first = grow_start ? start_tree : goal_tree;
    auto& second = grow_start ? goal_tree : start_tree;
    const auto target = sampled_cell(iterations, grow_start);
    const auto added = grow_once(first, target);
    if (!added) {
      continue;
    }
    if (const auto other = touching(*added, second)) {
      meeting = grow_start ? std::pair{*added, *other} : std::pair{*other, *added};
      break;
    }
    // RRT-Connect greedily advances the opposite tree toward the newly reached cell.  Bound each
    // connect attempt by the Manhattan separation, so one sample cannot consume the whole run.
    const auto connect_limit = cell_distance(nearest(second, *added), *added).first + 1U;
    for (std::size_t connect = 0U;
         connect < connect_limit && iterations + 1U < rrt_config.maximum_iterations; ++connect) {
      ++iterations;
      const auto connected = grow_once(second, *added);
      if (!connected) {
        break;
      }
      if (const auto other = touching(*connected, first)) {
        meeting = grow_start ? std::pair{*other, *connected} : std::pair{*connected, *other};
        break;
      }
    }
  }
  failed.rrt_connect_iterations = iterations;
  if (!meeting) {
    failed.search_budget_exhausted = iterations >= rrt_config.maximum_iterations;
    failed.diagnostic = failed.search_budget_exhausted
                            ? "deterministic NavMesh RRT-Connect budget exhausted"
                            : "deterministic NavMesh RRT-Connect trees did not meet";
    return failed;
  }
  const auto trace_to_root = [](const Tree& tree, std::size_t cell) {
    std::vector<std::size_t> path{cell};
    while (tree.parent_by_cell[cell] != cell) {
      cell = tree.parent_by_cell[cell];
      path.push_back(cell);
    }
    return path;
  };
  auto first_half = trace_to_root(start_tree, meeting->first);
  std::reverse(first_half.begin(), first_half.end());
  auto second_half = trace_to_root(goal_tree, meeting->second);
  std::vector<std::size_t> rrt_path = std::move(first_half);
  if (!second_half.empty() && !rrt_path.empty() && second_half.front() == rrt_path.back()) {
    second_half.erase(second_half.begin());
  }
  rrt_path.insert(rrt_path.end(), second_half.begin(), second_half.end());

  std::vector<std::uint8_t> opened(prepared.cells.size(), 0U);
  for (const auto cell : rrt_path) {
    const auto [source_x, source_y] = prepared.coordinate_by_cell[cell];
    const auto rings = static_cast<std::int64_t>(rrt_config.opened_guard_rings);
    for (auto dy = -rings; dy <= rings; ++dy) {
      for (auto dx = -rings; dx <= rings; ++dx) {
        const auto x = static_cast<std::int64_t>(source_x) + dx;
        const auto y = static_cast<std::int64_t>(source_y) + dy;
        if (x < 0 || y < 0 || x >= static_cast<std::int64_t>(prepared.width) ||
            y >= static_cast<std::int64_t>(prepared.height)) {
          continue;
        }
        const auto coordinate =
            static_cast<std::size_t>(y) * prepared.width + static_cast<std::size_t>(x);
        for (const auto neighbour : prepared.cells_by_coordinate[coordinate]) {
          opened[neighbour] = 1U;
        }
      }
    }
  }
  std::vector<std::size_t> closed;
  closed.reserve(prepared.cells.size() - std::min(prepared.cells.size(), rrt_path.size()));
  for (std::size_t cell = 0U; cell < opened.size(); ++cell) {
    if (opened[cell] == 0U) {
      closed.push_back(cell);
    }
  }
  auto result = route_prepared_navmesh_flood_fill(prepared, start, std::vector<Point>{goal},
                                                  navmesh_config, closed);
  result.used_rrt_connect = result.ok();
  result.rrt_connect_iterations = iterations;
  if (!result.ok() && result.diagnostic.empty()) {
    result.diagnostic = "RRT-Connect corridor failed certified funnel realization";
  }
  return result;
}

NavMeshRouteResult
route_prepared_navmesh_hydraulic(const PreparedNavMeshDomain& prepared, Point start, Point goal,
                                 const NavMeshConfig& navmesh_config,
                                 const NavMeshHydraulicConfig& hydraulic_config) {
  NavMeshRouteResult failure;
  failure.start = start;
  failure.goal = goal;
  failure.cells = prepared.cells;
  failure.cell_grid_minimum = prepared.minimum;
  failure.cell_pitch = prepared.pitch;
  failure.cell_grid_width = prepared.width;
  failure.cell_grid_height = prepared.height;
  failure.hydraulic_direct_polygon_discovery = true;
  if (!prepared.ok() || hydraulic_config.discovery_resolution_multiplier == 0U ||
      hydraulic_config.maximum_discovery_cells == 0U) {
    failure.diagnostic = "invalid direct polygonal hydraulic discovery request";
    return failure;
  }

  // Hydraulic discovery is deliberately independent of the square decomposition. Rasterize the
  // prepared polygon itself at a finer pitch and let the coarse cells account for the result only
  // after a river has been found and exact-certified.
  const auto discovery_pitch = Nanometres{std::max<std::int64_t>(
      1, prepared.pitch.value() /
             static_cast<std::int64_t>(hydraulic_config.discovery_resolution_multiplier))};
  auto discovery_minimum = prepared.prepared_outline.bounds.minimum;
  auto discovery_maximum = prepared.prepared_outline.bounds.maximum;
  if (prepared.search_bounds) {
    discovery_minimum = Point{std::max(discovery_minimum.x, prepared.search_bounds->minimum.x),
                              std::max(discovery_minimum.y, prepared.search_bounds->minimum.y)};
    discovery_maximum = Point{std::min(discovery_maximum.x, prepared.search_bounds->maximum.x),
                              std::min(discovery_maximum.y, prepared.search_bounds->maximum.y)};
  }
  const auto span_x = discovery_maximum.x.value() - discovery_minimum.x.value();
  const auto span_y = discovery_maximum.y.value() - discovery_minimum.y.value();
  if (span_x <= 0 || span_y <= 0) {
    failure.diagnostic = "direct polygonal hydraulic discovery bounds are empty";
    return failure;
  }
  const auto discovery_width =
      static_cast<std::size_t>((span_x + discovery_pitch.value() - 1) / discovery_pitch.value());
  const auto discovery_height =
      static_cast<std::size_t>((span_y + discovery_pitch.value() - 1) / discovery_pitch.value());
  if (discovery_width == 0U || discovery_height == 0U ||
      discovery_width > hydraulic_config.maximum_discovery_cells / discovery_height) {
    failure.diagnostic = "direct polygonal hydraulic discovery cell budget exhausted";
    failure.cell_budget_exhausted = true;
    return failure;
  }

  HydraulicPathInput input;
  input.width = discovery_width;
  input.height = discovery_height;
  input.compact_output_max_dimension = 64U;
  input.polygon_raster = HydraulicPolygonRaster{discovery_minimum.x.value(),
                                                discovery_minimum.y.value(),
                                                discovery_pitch.value(),
                                                prepared.minimum_boundary_clearance.value(),
                                                {}};
  const auto append_raster_segments = [&](const geometry::PreparedPolygon& polygon,
                                          std::uint32_t obstacle_id) {
    for (const auto& boundary : polygon.boundary_segments) {
      const auto relative = [&](Nanometres value,
                                Nanometres origin) -> std::optional<std::int32_t> {
        const auto delta = value.value() - origin.value();
        if (delta < std::numeric_limits<std::int32_t>::min() ||
            delta > std::numeric_limits<std::int32_t>::max())
          return std::nullopt;
        return static_cast<std::int32_t>(delta);
      };
      const auto x1 = relative(boundary.start.x, discovery_minimum.x);
      const auto y1 = relative(boundary.start.y, discovery_minimum.y);
      const auto x2 = relative(boundary.end.x, discovery_minimum.x);
      const auto y2 = relative(boundary.end.y, discovery_minimum.y);
      if (!x1 || !y1 || !x2 || !y2)
        return false;
      input.polygon_raster->boundary_segments.push_back(
          HydraulicRasterSegment{*x1, *y1, *x2, *y2, obstacle_id});
    }
    return true;
  };
  if (!append_raster_segments(prepared.prepared_outline, 0U)) {
    failure.diagnostic = "direct polygonal hydraulic raster exceeds GPU relative range";
    return failure;
  }
  for (std::size_t cutout = 0U; cutout < prepared.prepared_cutouts.size(); ++cutout) {
    if (!append_raster_segments(prepared.prepared_cutouts[cutout],
                                static_cast<std::uint32_t>(cutout + 1U))) {
      failure.diagnostic = "direct polygonal hydraulic raster exceeds GPU relative range";
      return failure;
    }
  }
  const auto discovery_point = [&](std::size_t coordinate) {
    const auto x = coordinate % discovery_width;
    const auto y = coordinate / discovery_width;
    return Point{Nanometres{discovery_minimum.x.value() +
                            static_cast<std::int64_t>(x) * discovery_pitch.value() +
                            discovery_pitch.value() / 2},
                 Nanometres{discovery_minimum.y.value() +
                            static_cast<std::int64_t>(y) * discovery_pitch.value() +
                            discovery_pitch.value() / 2}};
  };
  const auto discovery_cell_count = discovery_width * discovery_height;
  input.downstream_pressure.assign(discovery_cell_count, 0U);
  input.success_prior.assign(discovery_cell_count, 0U);
  const auto discovery_index = [&](std::int64_t coordinate, std::int64_t origin,
                                   std::size_t extent) {
    const auto relative = coordinate - origin;
    auto index = relative / discovery_pitch.value();
    if (relative < 0 && relative % discovery_pitch.value() != 0)
      --index;
    return static_cast<std::size_t>(
        std::clamp<std::int64_t>(index, 0, static_cast<std::int64_t>(extent - 1U)));
  };
  for (const auto& feedback : hydraulic_config.downstream_feedback) {
    if (feedback.pressure == 0U && feedback.success_prior == 0U)
      continue;
    if (feedback.bounds.maximum.x < discovery_minimum.x ||
        feedback.bounds.minimum.x > discovery_maximum.x ||
        feedback.bounds.maximum.y < discovery_minimum.y ||
        feedback.bounds.minimum.y > discovery_maximum.y) {
      continue;
    }
    const auto x_begin =
        discovery_index(feedback.bounds.minimum.x.value() - discovery_pitch.value(),
                        discovery_minimum.x.value(), discovery_width);
    const auto y_begin =
        discovery_index(feedback.bounds.minimum.y.value() - discovery_pitch.value(),
                        discovery_minimum.y.value(), discovery_height);
    const auto x_end = discovery_index(feedback.bounds.maximum.x.value() + discovery_pitch.value(),
                                       discovery_minimum.x.value(), discovery_width);
    const auto y_end = discovery_index(feedback.bounds.maximum.y.value() + discovery_pitch.value(),
                                       discovery_minimum.y.value(), discovery_height);
    for (auto y = y_begin; y <= y_end; ++y) {
      for (auto x = x_begin; x <= x_end; ++x) {
        const auto cell = y * discovery_width + x;
        if (feedback.directional &&
            geometry::point_segment_distance_nm(discovery_point(cell), feedback.axis_start,
                                                feedback.axis_end) >
                static_cast<long double>(discovery_pitch.value())) {
          continue;
        }
        input.downstream_pressure[cell] =
            input.downstream_pressure[cell] >
                    std::numeric_limits<std::uint32_t>::max() - feedback.pressure
                ? std::numeric_limits<std::uint32_t>::max()
                : input.downstream_pressure[cell] + feedback.pressure;
        input.success_prior[cell] =
            input.success_prior[cell] >
                    std::numeric_limits<std::uint32_t>::max() - feedback.success_prior
                ? std::numeric_limits<std::uint32_t>::max()
                : input.success_prior[cell] + feedback.success_prior;
      }
    }
  }
  failure.hydraulic_discovery_cells = discovery_cell_count;
  failure.hydraulic_field_origin = discovery_minimum;
  failure.hydraulic_field_cell_pitch = discovery_pitch;

  const auto bind_endpoint = [&](Point endpoint) -> std::optional<std::size_t> {
    const auto endpoint_query = query_prepared_navmesh_point(prepared, endpoint);
    if (!endpoint_query.inside)
      return std::nullopt;
    const auto relative_x = endpoint.x.value() - discovery_minimum.x.value();
    const auto relative_y = endpoint.y.value() - discovery_minimum.y.value();
    const auto base_x = relative_x / discovery_pitch.value();
    const auto base_y = relative_y / discovery_pitch.value();
    for (std::int64_t ring = 0; ring <= 32; ++ring) {
      std::vector<std::size_t> candidates;
      std::vector<std::pair<Point, Point>> links;
      for (auto dy = -ring; dy <= ring; ++dy) {
        for (auto dx = -ring; dx <= ring; ++dx) {
          if (ring > 0 && std::max(std::abs(dx), std::abs(dy)) != ring)
            continue;
          const auto x = base_x + dx;
          const auto y = base_y + dy;
          if (x < 0 || y < 0 || x >= static_cast<std::int64_t>(discovery_width) ||
              y >= static_cast<std::int64_t>(discovery_height)) {
            continue;
          }
          const auto coordinate =
              static_cast<std::size_t>(y) * discovery_width + static_cast<std::size_t>(x);
          const auto raster_query =
              query_prepared_navmesh_point(prepared, discovery_point(coordinate));
          if (!raster_query.inside ||
              raster_query.boundary_distance_nm <
                  static_cast<long double>(prepared.minimum_boundary_clearance.value()))
            continue;
          candidates.push_back(coordinate);
          links.emplace_back(endpoint, discovery_point(coordinate));
        }
      }
      const auto certificates = assess_prepared_navmesh_chords(prepared, links);
      std::optional<std::pair<long double, std::size_t>> selected;
      for (std::size_t index = 0U; index < candidates.size(); ++index) {
        if (!certificates[index] ||
            certificates[index]->value() < prepared.minimum_boundary_clearance.value()) {
          continue;
        }
        const auto distance =
            geometry::point_distance_nm(endpoint, discovery_point(candidates[index]));
        const auto candidate = std::pair{distance, candidates[index]};
        if (!selected || candidate < *selected)
          selected = candidate;
      }
      if (selected)
        return selected->second;
    }
    return std::nullopt;
  };
  const auto source = bind_endpoint(start);
  const auto target = bind_endpoint(goal);
  if (!source || !target || *source == *target) {
    failure.diagnostic = "direct polygonal hydraulic discovery could not bind its endpoints";
    return failure;
  }
  input.source = *source;
  input.target = *target;

  const auto flow = solve_hydraulic_erosion_path(input, hydraulic_config.flow);
  if (!flow.ok()) {
    failure.hydraulic_relaxation_passes = flow.relaxation_passes;
    failure.hydraulic_droplet_steps = flow.droplet_steps;
    failure.hydraulic_droplets_reaching_target = flow.droplets_reaching_target;
    failure.used_hydraulic_gpu = flow.used_gpu;
    failure.hydraulic_gpu_clearance_cache_hit = flow.gpu_clearance_cache_hit;
    failure.hydraulic_gpu_batch_size = flow.gpu_batch_size;
    failure.hydraulic_gpu_buffer_reuses = flow.gpu_persistent_buffer_reuses;
    failure.hydraulic_gpu_buffer_growths = flow.gpu_buffer_growths;
    failure.hydraulic_cpu_fallback = flow.cpu_fallback;
    failure.search_budget_exhausted = flow.budget_exhausted;
    failure.diagnostic = "hydraulic topology proposal failed: " + flow.diagnostic;
    return failure;
  }

  std::vector<Point> riverbed{start};
  for (const auto coordinate : flow.riverbed) {
    const auto point = discovery_point(coordinate);
    if (riverbed.back() != point)
      riverbed.push_back(point);
  }
  if (riverbed.back() != goal)
    riverbed.push_back(goal);
  std::vector<std::pair<Point, Point>> riverbed_chords;
  riverbed_chords.reserve(riverbed.size() - 1U);
  for (std::size_t index = 1U; index < riverbed.size(); ++index) {
    riverbed_chords.emplace_back(riverbed[index - 1U], riverbed[index]);
  }
  const auto riverbed_certificates = assess_prepared_navmesh_chords(prepared, riverbed_chords);
  if (std::any_of(riverbed_certificates.begin(), riverbed_certificates.end(),
                  [&](const auto& item) {
                    return !item || item->value() < prepared.minimum_boundary_clearance.value();
                  })) {
    failure.diagnostic = "direct polygonal hydraulic riverbed failed exact chord certification";
    return failure;
  }
  failure.hydraulic_exact_certified_chords = riverbed_chords.size();

  // Remove raster-scale wiggles with exact visibility pulling before projecting into the coarse
  // resource lattice. A shortcut can change samples but cannot cross an obstacle or leave the
  // represented free-space region.
  std::vector<Point> pulled{riverbed.front()};
  for (std::size_t first = 0U; first + 1U < riverbed.size();) {
    std::vector<std::pair<Point, Point>> chords;
    chords.reserve(riverbed.size() - first - 1U);
    for (auto last = riverbed.size() - 1U; last > first; --last) {
      chords.emplace_back(riverbed[first], riverbed[last]);
    }
    const auto certificates = assess_prepared_navmesh_chords(prepared, chords);
    auto selected = first + 1U;
    for (std::size_t index = 0U; index < chords.size(); ++index) {
      const auto last = riverbed.size() - 1U - index;
      if (certificates[index] &&
          certificates[index]->value() >= prepared.minimum_boundary_clearance.value()) {
        selected = last;
        break;
      }
    }
    pulled.push_back(riverbed[selected]);
    first = selected;
  }

  const auto field_stride =
      flow.used_gpu_compaction
          ? flow.compact_stride
          : std::max<std::size_t>(1U, (std::max(discovery_width, discovery_height) + 63U) / 64U);
  const auto field_width = flow.used_gpu_compaction
                               ? flow.compact_width
                               : (discovery_width + field_stride - 1U) / field_stride;
  const auto field_height = flow.used_gpu_compaction
                                ? flow.compact_height
                                : (discovery_height + field_stride - 1U) / field_stride;
  std::vector<std::uint32_t> sampled_flow(field_width * field_height, 0U);
  std::vector<std::uint32_t> sampled_erosion(field_width * field_height, 0U);
  std::vector<std::uint32_t> sampled_terrain(field_width * field_height,
                                             hydraulic_unreachable_height);
  std::vector<std::uint32_t> sampled_free_cells(field_width * field_height, 0U);
  std::vector<std::uint32_t> sampled_cells(field_width * field_height, 0U);
  std::vector<std::uint32_t> sampled_pressure(field_width * field_height, 0U);
  std::vector<std::uint32_t> sampled_success(field_width * field_height, 0U);
  if (!flow.used_gpu_compaction) {
    for (std::size_t y = 0U; y < discovery_height; ++y) {
      for (std::size_t x = 0U; x < discovery_width; ++x) {
        const auto source = y * discovery_width + x;
        const auto target = (y / field_stride) * field_width + x / field_stride;
        ++sampled_cells[target];
        if (flow.walkable[source] != 0U)
          ++sampled_free_cells[target];
        sampled_flow[target] = std::max(sampled_flow[target], flow.accumulated_flow[source]);
        sampled_erosion[target] = std::max(sampled_erosion[target], flow.erosion[source]);
        sampled_pressure[target] =
            std::max(sampled_pressure[target], flow.downstream_pressure[source]);
        sampled_success[target] = std::max(sampled_success[target], flow.success_prior[source]);
        if (flow.terrain_height[source] != hydraulic_unreachable_height) {
          sampled_terrain[target] = std::min(sampled_terrain[target], flow.terrain_height[source]);
        }
      }
    }
  }
  const auto maximum_flow = flow.used_gpu_compaction
                                ? flow.maximum_flow
                                : *std::max_element(sampled_flow.begin(), sampled_flow.end());
  const auto maximum_erosion =
      flow.used_gpu_compaction ? flow.maximum_erosion
                               : *std::max_element(sampled_erosion.begin(), sampled_erosion.end());
  const auto maximum_pressure =
      flow.used_gpu_compaction
          ? flow.maximum_downstream_pressure
          : *std::max_element(sampled_pressure.begin(), sampled_pressure.end());
  const auto maximum_success =
      flow.used_gpu_compaction ? flow.maximum_success_prior
                               : *std::max_element(sampled_success.begin(), sampled_success.end());
  std::uint32_t maximum_terrain = flow.used_gpu_compaction ? flow.maximum_terrain : 0U;
  for (const auto height : sampled_terrain) {
    if (height != hydraulic_unreachable_height)
      maximum_terrain = std::max(maximum_terrain, height);
  }
  for (std::size_t sample = 0U; !flow.used_gpu_compaction && sample < sampled_terrain.size();
       ++sample) {
    if (sampled_free_cells[sample] != 0U &&
        sampled_terrain[sample] == hydraulic_unreachable_height) {
      // Disconnected legal free space remains visible as the highest plateau. The separate
      // coverage field—not height—continues to define whether terrain exists at this sample.
      sampled_terrain[sample] = maximum_terrain;
    }
  }
  const auto normalize_field = [](const std::vector<std::uint32_t>& values, std::uint32_t maximum) {
    std::vector<std::uint16_t> normalized(values.size(), 0U);
    if (maximum == 0U) {
      return normalized;
    }
    std::transform(values.begin(), values.end(), normalized.begin(), [&](std::uint32_t value) {
      return static_cast<std::uint16_t>(
          std::min<std::uint64_t>(65'535U, static_cast<std::uint64_t>(value) * 65'535U / maximum));
    });
    return normalized;
  };

  std::vector<std::uint8_t> opened(prepared.cells.size(), 0U);
  const auto rings = static_cast<std::int64_t>(hydraulic_config.opened_guard_rings);
  const auto open_point = [&](Point point) {
    const auto coarse_x = static_cast<std::int64_t>(
        std::floor(static_cast<long double>(point.x.value() - prepared.minimum.x.value()) /
                   static_cast<long double>(prepared.pitch.value())));
    const auto coarse_y = static_cast<std::int64_t>(
        std::floor(static_cast<long double>(point.y.value() - prepared.minimum.y.value()) /
                   static_cast<long double>(prepared.pitch.value())));
    for (auto dy = -rings; dy <= rings; ++dy) {
      for (auto dx = -rings; dx <= rings; ++dx) {
        const auto x = coarse_x + dx;
        const auto y = coarse_y + dy;
        if (x < 0 || y < 0 || x >= static_cast<std::int64_t>(prepared.width) ||
            y >= static_cast<std::int64_t>(prepared.height)) {
          continue;
        }
        const auto coordinate =
            static_cast<std::size_t>(y) * prepared.width + static_cast<std::size_t>(x);
        for (const auto cell : prepared.cells_by_coordinate[coordinate]) {
          opened[cell] = 1U;
        }
      }
    }
  };
  for (std::size_t index = 1U; index < pulled.size(); ++index) {
    const auto first = pulled[index - 1U];
    const auto last = pulled[index];
    const auto length = geometry::segment_length_nm(first, last);
    const auto samples = std::max<std::uint64_t>(
        1U, (length + static_cast<std::uint64_t>(discovery_pitch.value()) - 1U) /
                static_cast<std::uint64_t>(discovery_pitch.value()));
    for (std::uint64_t sample = 0U; sample <= samples; ++sample) {
      open_point(Point{
          Nanometres{static_cast<std::int64_t>(std::llround(
              static_cast<long double>(first.x.value()) +
              static_cast<long double>(last.x.value() - first.x.value()) * sample / samples))},
          Nanometres{static_cast<std::int64_t>(std::llround(
              static_cast<long double>(first.y.value()) +
              static_cast<long double>(last.y.value() - first.y.value()) * sample / samples))}});
    }
  }
  std::vector<std::size_t> closed;
  closed.reserve(prepared.cells.size());
  for (std::size_t cell = 0U; cell < opened.size(); ++cell) {
    if (opened[cell] == 0U) {
      closed.push_back(cell);
    }
  }
  auto result = route_prepared_navmesh_flood_fill(prepared, start, std::vector<Point>{goal},
                                                  navmesh_config, closed);
  result.hydraulic_relaxation_passes = flow.relaxation_passes;
  result.hydraulic_droplet_steps = flow.droplet_steps;
  result.hydraulic_droplets_reaching_target = flow.droplets_reaching_target;
  result.hydraulic_field_origin = discovery_minimum;
  result.hydraulic_field_cell_pitch = discovery_pitch;
  result.hydraulic_discovery_cells = discovery_cell_count;
  result.hydraulic_exact_certified_chords = riverbed_chords.size();
  result.hydraulic_direct_polygon_discovery = true;
  result.hydraulic_field_width = field_width;
  result.hydraulic_field_height = field_height;
  result.hydraulic_field_stride = field_stride;
  result.hydraulic_maximum_flow = maximum_flow;
  result.hydraulic_maximum_erosion = maximum_erosion;
  result.hydraulic_maximum_terrain_height = maximum_terrain;
  result.hydraulic_maximum_downstream_pressure = maximum_pressure;
  result.hydraulic_maximum_success_prior = maximum_success;
  result.hydraulic_flow_field =
      flow.used_gpu_compaction ? flow.compact_flow : normalize_field(sampled_flow, maximum_flow);
  result.hydraulic_erosion_field = flow.used_gpu_compaction
                                       ? flow.compact_erosion
                                       : normalize_field(sampled_erosion, maximum_erosion);
  result.hydraulic_heightmap = flow.used_gpu_compaction
                                   ? flow.compact_terrain
                                   : normalize_field(sampled_terrain, maximum_terrain);
  result.hydraulic_free_space_field = flow.used_gpu_compaction
                                          ? flow.compact_free_space
                                          : std::vector<std::uint16_t>(sampled_cells.size(), 0U);
  result.hydraulic_downstream_pressure_field =
      flow.used_gpu_compaction ? flow.compact_downstream_pressure
                               : normalize_field(sampled_pressure, maximum_pressure);
  result.hydraulic_success_prior_field = flow.used_gpu_compaction
                                             ? flow.compact_success_prior
                                             : normalize_field(sampled_success, maximum_success);
  for (std::size_t sample = 0U; !flow.used_gpu_compaction && sample < sampled_cells.size();
       ++sample) {
    if (sampled_cells[sample] == 0U)
      continue;
    result.hydraulic_free_space_field[sample] = static_cast<std::uint16_t>(
        static_cast<std::uint64_t>(sampled_free_cells[sample]) * 65'535U / sampled_cells[sample]);
  }
  result.used_hydraulic_erosion = result.ok();
  result.used_hydraulic_gpu = flow.used_gpu;
  result.hydraulic_gpu_clearance_cache_hit = flow.gpu_clearance_cache_hit;
  result.hydraulic_gpu_batch_size = flow.gpu_batch_size;
  result.hydraulic_gpu_buffer_reuses = flow.gpu_persistent_buffer_reuses;
  result.hydraulic_gpu_buffer_growths = flow.gpu_buffer_growths;
  result.hydraulic_cpu_fallback = flow.cpu_fallback;
  if (!result.ok() && result.diagnostic.empty()) {
    result.diagnostic = "hydraulic riverbed failed certified NavMesh realization";
  }
  return result;
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
  if (route.adaptive_maximum_refinement_depth == 0U && route.cell_pitch.value() > 0 &&
      route.cell_grid_width > 0U && route.cell_grid_height > 0U &&
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
