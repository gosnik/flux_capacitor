#include "flux/core/free_space_skeleton.hpp"

#include "flux/core/geometry.hpp"

#include <boost/polygon/segment_data.hpp>
#include <boost/polygon/voronoi.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <map>
#include <set>
#include <utility>
#include <vector>

namespace flux {
namespace {

using BoostPoint = boost::polygon::point_data<std::int32_t>;
using BoostSegment = boost::polygon::segment_data<std::int32_t>;
using VoronoiDiagram = boost::polygon::voronoi_diagram<double>;

[[nodiscard]] bool valid_domain(const FreeSpaceDomain& domain,
                                const FreeSpaceSkeletonConfig& config,
                                FreeSpaceSkeletonResult& result,
                                bool require_disjoint_obstacles = true) {
  if (config.minimum_clearance.value() < 0 || config.maximum_boundary_segments == 0U) {
    result.diagnostic = "free-space skeleton limits and clearance must be non-negative";
    return false;
  }
  if (!geometry::is_simple_polygon(domain.outline)) {
    result.diagnostic = "free-space skeleton outline must be a simple polygon";
    return false;
  }
  for (std::size_t index = 0U; index < domain.cutouts.size(); ++index) {
    const auto& cutout = domain.cutouts[index];
    if (!geometry::is_simple_polygon(cutout) || !geometry::polygon_inside(cutout, domain.outline) ||
        geometry::polygon_boundary_distance_nm(cutout, domain.outline) <= 0.0L ||
        std::any_of(domain.cutouts.begin(),
                    domain.cutouts.begin() + static_cast<std::ptrdiff_t>(index),
                    [&cutout](const Polygon& prior) {
                      return geometry::polygons_overlap(cutout, prior);
                    })) {
      result.diagnostic =
          "free-space skeleton cutouts must be simple, disjoint, and strictly inside the outline";
      return false;
    }
  }
  for (std::size_t index = 0; index < domain.obstacles.size(); ++index) {
    const auto& obstacle = domain.obstacles[index];
    if (!geometry::is_simple_polygon(obstacle) ||
        !geometry::polygon_inside(obstacle, domain.outline) ||
        geometry::polygon_boundary_distance_nm(obstacle, domain.outline) <= 0.0L ||
        std::any_of(domain.cutouts.begin(), domain.cutouts.end(),
                    [&obstacle](const Polygon& cutout) {
                      return geometry::polygons_overlap(obstacle, cutout);
                    })) {
      result.diagnostic = "free-space skeleton obstacles must be simple and strictly inside the "
                          "outline";
      return false;
    }
    if (require_disjoint_obstacles &&
        std::any_of(domain.obstacles.begin(),
                    domain.obstacles.begin() + static_cast<std::ptrdiff_t>(index),
                    [&obstacle](const Polygon& prior) {
                      return geometry::polygons_overlap(obstacle, prior);
                    })) {
      result.diagnostic = "free-space skeleton obstacles must be disjoint";
      return false;
    }
  }
  for (const auto& obstacle : domain.filled_obstacles) {
    if (!geometry::is_simple_polygon(obstacle.area) ||
        !geometry::polygon_inside(obstacle.area, domain.outline) ||
        std::any_of(domain.cutouts.begin(), domain.cutouts.end(),
                    [&obstacle](const Polygon& cutout) {
                      return geometry::filled_region_overlaps_polygon(obstacle, cutout);
                    })) {
      result.diagnostic = "filled skeleton obstacles require a simple contained outer contour";
      return false;
    }
    for (std::size_t index = 0; index < obstacle.holes.size(); ++index) {
      const auto& hole = obstacle.holes[index];
      if (!geometry::is_simple_polygon(hole) || !geometry::polygon_inside(hole, obstacle.area) ||
          geometry::polygon_boundary_distance_nm(hole, obstacle.area) <= 0.0L ||
          std::any_of(
              obstacle.holes.begin(), obstacle.holes.begin() + static_cast<std::ptrdiff_t>(index),
              [&hole](const Polygon& prior) { return geometry::polygons_overlap(hole, prior); })) {
        result.diagnostic = "filled skeleton obstacle holes must be simple and disjoint";
        return false;
      }
    }
  }
  return true;
}

void append_segments(const Polygon& polygon, std::vector<BoostSegment>& segments) {
  for (std::size_t index = 0; index < polygon.vertices.size(); ++index) {
    const auto start = polygon.vertices[index];
    const auto end = polygon.vertices[(index + 1U) % polygon.vertices.size()];
    segments.emplace_back(BoostPoint{static_cast<std::int32_t>(start.x.value()),
                                     static_cast<std::int32_t>(start.y.value())},
                          BoostPoint{static_cast<std::int32_t>(end.x.value()),
                                     static_cast<std::int32_t>(end.y.value())});
  }
}

[[nodiscard]] bool chord_in_free_space(Point start, Point end, const FreeSpaceDomain& domain) {
  if (!geometry::segment_inside_polygon(start, end, domain.outline)) {
    return false;
  }
  return std::none_of(domain.cutouts.begin(), domain.cutouts.end(),
                      [start, end](const Polygon& cutout) {
                        return geometry::segment_polygon_copper_distance_nm(start, end, cutout) <=
                               0.0L;
                      }) &&
         std::none_of(domain.obstacles.begin(), domain.obstacles.end(),
                      [start, end](const Polygon& obstacle) {
                        return geometry::segment_polygon_copper_distance_nm(start, end, obstacle) <=
                               0.0L;
                      }) &&
         std::none_of(domain.filled_obstacles.begin(), domain.filled_obstacles.end(),
                      [start, end](const CopperZoneFill& obstacle) {
                        return geometry::segment_filled_region_copper_distance_nm(start, end,
                                                                                  obstacle) <= 0.0L;
                      });
}

[[nodiscard]] long double boundary_clearance(Point start, Point end,
                                             const FreeSpaceDomain& domain) {
  auto clearance = geometry::segment_polygon_boundary_distance_nm(start, end, domain.outline);
  for (const auto& cutout : domain.cutouts) {
    clearance =
        std::min(clearance, geometry::segment_polygon_boundary_distance_nm(start, end, cutout));
  }
  for (const auto& obstacle : domain.obstacles) {
    clearance =
        std::min(clearance, geometry::segment_polygon_boundary_distance_nm(start, end, obstacle));
  }
  for (const auto& obstacle : domain.filled_obstacles) {
    clearance = std::min(clearance,
                         geometry::segment_polygon_boundary_distance_nm(start, end, obstacle.area));
    for (const auto& hole : obstacle.holes) {
      clearance =
          std::min(clearance, geometry::segment_polygon_boundary_distance_nm(start, end, hole));
    }
  }
  return clearance;
}

[[nodiscard]] std::optional<Point> quantize_vertex(const VoronoiDiagram::vertex_type& vertex) {
  if (!std::isfinite(vertex.x()) || !std::isfinite(vertex.y()) ||
      vertex.x() < static_cast<double>(-max_abs_coordinate_nm) ||
      vertex.x() > static_cast<double>(max_abs_coordinate_nm) ||
      vertex.y() < static_cast<double>(-max_abs_coordinate_nm) ||
      vertex.y() > static_cast<double>(max_abs_coordinate_nm)) {
    return std::nullopt;
  }
  return Point{Nanometres{static_cast<std::int64_t>(std::llround(vertex.x()))},
               Nanometres{static_cast<std::int64_t>(std::llround(vertex.y()))}};
}

} // namespace

FreeSpaceSkeletonResult build_free_space_skeleton(const FreeSpaceDomain& domain,
                                                  const FreeSpaceSkeletonConfig& config) {
  FreeSpaceSkeletonResult result;
  if (!valid_domain(domain, config, result)) {
    return result;
  }
  result.boundary_segments = domain.outline.vertices.size();
  for (const auto& cutout : domain.cutouts) {
    if (cutout.vertices.size() > config.maximum_boundary_segments - result.boundary_segments) {
      result.diagnostic = "free-space skeleton exceeds its boundary-segment budget";
      return result;
    }
    result.boundary_segments += cutout.vertices.size();
  }
  for (const auto& obstacle : domain.obstacles) {
    if (obstacle.vertices.size() > config.maximum_boundary_segments - result.boundary_segments) {
      result.diagnostic = "free-space skeleton exceeds its boundary-segment budget";
      return result;
    }
    result.boundary_segments += obstacle.vertices.size();
  }
  for (const auto& obstacle : domain.filled_obstacles) {
    if (obstacle.area.vertices.size() >
        config.maximum_boundary_segments - result.boundary_segments) {
      result.diagnostic = "free-space skeleton exceeds its boundary-segment budget";
      return result;
    }
    result.boundary_segments += obstacle.area.vertices.size();
    for (const auto& hole : obstacle.holes) {
      if (hole.vertices.size() > config.maximum_boundary_segments - result.boundary_segments) {
        result.diagnostic = "free-space skeleton exceeds its boundary-segment budget";
        return result;
      }
      result.boundary_segments += hole.vertices.size();
    }
  }
  if (result.boundary_segments > config.maximum_boundary_segments) {
    result.diagnostic = "free-space skeleton exceeds its boundary-segment budget";
    return result;
  }

  std::vector<BoostSegment> segments;
  segments.reserve(result.boundary_segments);
  append_segments(domain.outline, segments);
  for (const auto& cutout : domain.cutouts) {
    append_segments(cutout, segments);
  }
  for (const auto& obstacle : domain.obstacles) {
    append_segments(obstacle, segments);
  }
  for (const auto& obstacle : domain.filled_obstacles) {
    append_segments(obstacle.area, segments);
    for (const auto& hole : obstacle.holes) {
      append_segments(hole, segments);
    }
  }
  VoronoiDiagram diagram;
  boost::polygon::construct_voronoi(segments.begin(), segments.end(), &diagram);

  std::set<std::pair<Point, Point>> accepted;
  std::set<std::pair<Point, Point>> seen_curved_chords;
  for (const auto& edge : diagram.edges()) {
    if (!edge.is_primary()) {
      continue;
    }
    if (edge.is_infinite()) {
      ++result.skipped_infinite_edges;
      continue;
    }
    const bool curved_bisector_chord = edge.is_curved();
    if (curved_bisector_chord && !config.checked_curved_bisector_chords) {
      ++result.curved_edges_seen;
      ++result.skipped_curved_edges;
      continue;
    }
    const auto start = quantize_vertex(*edge.vertex0());
    const auto end = quantize_vertex(*edge.vertex1());
    if (!start || !end || *start == *end) {
      result.curved_edges_seen += curved_bisector_chord ? 1U : 0U;
      result.skipped_curved_edges += curved_bisector_chord ? 1U : 0U;
      ++result.rejected_outside_edges;
      continue;
    }
    const auto ordered = std::minmax(*start, *end);
    if (curved_bisector_chord && !seen_curved_chords.insert(ordered).second) {
      continue;
    }
    result.curved_edges_seen += curved_bisector_chord ? 1U : 0U;
    if (accepted.contains(ordered)) {
      continue;
    }
    if (!chord_in_free_space(ordered.first, ordered.second, domain)) {
      result.skipped_curved_edges += curved_bisector_chord ? 1U : 0U;
      ++result.rejected_outside_edges;
      continue;
    }
    const auto clearance = boundary_clearance(ordered.first, ordered.second, domain);
    if (clearance < static_cast<long double>(config.minimum_clearance.value())) {
      result.skipped_curved_edges += curved_bisector_chord ? 1U : 0U;
      ++result.rejected_clearance_edges;
      continue;
    }
    accepted.insert(ordered);
    result.edges.push_back(FreeSpaceSkeletonEdge{
        0U, 0U, 0U, ordered.first, ordered.second,
        Nanometres{static_cast<std::int64_t>(std::floor(clearance))}, curved_bisector_chord});
    result.accepted_curved_edge_chords += curved_bisector_chord ? 1U : 0U;
  }
  std::sort(result.edges.begin(), result.edges.end(), [](const auto& lhs, const auto& rhs) {
    return std::tie(lhs.start, lhs.end) < std::tie(rhs.start, rhs.end);
  });

  std::set<Point> positions;
  for (const auto& edge : result.edges) {
    positions.insert(edge.start);
    positions.insert(edge.end);
  }
  std::map<Point, std::size_t> node_by_position;
  result.nodes.reserve(positions.size());
  for (const auto position : positions) {
    const auto id = result.nodes.size();
    const auto clearance = boundary_clearance(position, position, domain);
    result.nodes.push_back(FreeSpaceSkeletonNode{
        id, position, Nanometres{static_cast<std::int64_t>(std::floor(clearance))}, 0U});
    node_by_position.emplace(position, id);
  }
  for (std::size_t index = 0; index < result.edges.size(); ++index) {
    auto& edge = result.edges[index];
    edge.id = index;
    edge.start_node = node_by_position.at(edge.start);
    edge.end_node = node_by_position.at(edge.end);
    ++result.nodes[edge.start_node].degree;
    ++result.nodes[edge.end_node].degree;
  }
  result.junctions = static_cast<std::size_t>(std::count_if(
      result.nodes.begin(), result.nodes.end(), [](const auto& node) { return node.junction(); }));
  return result;
}

std::optional<Nanometres> assess_free_space_chord(const FreeSpaceDomain& domain, Point start,
                                                  Point end) {
  const auto assessed = assess_free_space_chords(domain, {{start, end}});
  return assessed.empty() ? std::nullopt : assessed.front();
}

bool validate_free_space_chord_domain(const FreeSpaceDomain& domain, std::string* diagnostic) {
  FreeSpaceSkeletonResult validation;
  const auto valid = valid_domain(domain, FreeSpaceSkeletonConfig{}, validation, false);
  if (diagnostic != nullptr) {
    *diagnostic = validation.diagnostic;
  }
  return valid;
}

std::vector<std::optional<Nanometres>>
assess_free_space_chords(const FreeSpaceDomain& domain,
                         const std::vector<std::pair<Point, Point>>& chords) {
  std::vector<std::optional<Nanometres>> result(chords.size());
  FreeSpaceSkeletonResult validation;
  if (!valid_domain(domain, FreeSpaceSkeletonConfig{}, validation, false)) {
    return result;
  }
  for (std::size_t index = 0U; index < chords.size(); ++index) {
    const auto [start, end] = chords[index];
    if (start == end || !chord_in_free_space(start, end, domain)) {
      continue;
    }
    const auto clearance = boundary_clearance(start, end, domain);
    if (!std::isfinite(clearance) || clearance < 0.0L ||
        clearance > static_cast<long double>(std::numeric_limits<std::int64_t>::max())) {
      continue;
    }
    result[index] = Nanometres{static_cast<std::int64_t>(std::floor(clearance))};
  }
  return result;
}

std::size_t skeleton_lane_capacity(const FreeSpaceSkeletonEdge& edge, Nanometres track_width,
                                   Nanometres inter_lane_clearance) {
  if (track_width.value() <= 0 || inter_lane_clearance.value() < 0 ||
      edge.minimum_boundary_clearance.value() < track_width.value() / 2) {
    return 0U;
  }
  const auto width = static_cast<std::uint64_t>(edge.minimum_boundary_clearance.value()) * 2U;
  const auto trace = static_cast<std::uint64_t>(track_width.value());
  const auto spacing = static_cast<std::uint64_t>(inter_lane_clearance.value());
  const auto pitch = trace + spacing;
  return static_cast<std::size_t>((width + spacing) / pitch);
}

} // namespace flux
