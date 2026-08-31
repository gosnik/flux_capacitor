#include "flux/core/free_space_offset.hpp"

#include "flux/core/geometry.hpp"

#include <boost/polygon/polygon.hpp>
#include <boost/polygon/polygon_set_concept.hpp>
#include <boost/polygon/polygon_set_data.hpp>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <list>
#include <string>
#include <vector>

namespace flux {
namespace {

using Coordinate = std::int32_t;
using BoostPoint = boost::polygon::point_data<Coordinate>;
using BoostPolygon = boost::polygon::polygon_data<Coordinate>;
using BoostPolygonWithHoles = boost::polygon::polygon_with_holes_data<Coordinate>;
using BoostPolygonSet = boost::polygon::polygon_set_data<Coordinate>;

struct PreparedFilledRegion {
  geometry::PreparedPolygon area;
  std::vector<geometry::PreparedPolygon> holes;
};

struct PreparedValidationDomain {
  geometry::PreparedPolygon outline;
  std::vector<geometry::PreparedPolygon> cutouts;
  std::vector<geometry::PreparedPolygon> obstacles;
  std::vector<PreparedFilledRegion> filled_obstacles;
};

[[nodiscard]] PreparedFilledRegion prepare_filled_region(const Polygon& area,
                                                         const std::vector<Polygon>& holes) {
  PreparedFilledRegion result;
  result.area = geometry::prepare_polygon(area);
  result.holes.reserve(holes.size());
  std::transform(holes.begin(), holes.end(), std::back_inserter(result.holes),
                 [](const auto& hole) { return geometry::prepare_polygon(hole); });
  return result;
}

[[nodiscard]] PreparedValidationDomain prepare_validation_domain(const FreeSpaceDomain& domain) {
  PreparedValidationDomain result;
  result.outline = geometry::prepare_polygon(domain.outline);
  result.cutouts.reserve(domain.cutouts.size());
  std::transform(domain.cutouts.begin(), domain.cutouts.end(),
                 std::back_inserter(result.cutouts),
                 [](const auto& polygon) { return geometry::prepare_polygon(polygon); });
  result.obstacles.reserve(domain.obstacles.size());
  std::transform(domain.obstacles.begin(), domain.obstacles.end(),
                 std::back_inserter(result.obstacles),
                 [](const auto& polygon) { return geometry::prepare_polygon(polygon); });
  result.filled_obstacles.reserve(domain.filled_obstacles.size());
  std::transform(domain.filled_obstacles.begin(), domain.filled_obstacles.end(),
                 std::back_inserter(result.filled_obstacles), [](const auto& region) {
                   return prepare_filled_region(region.area, region.holes);
                 });
  return result;
}

[[nodiscard]] geometry::PointLocation locate_point(const PreparedFilledRegion& region,
                                                   Point point) {
  const auto outer = geometry::locate_point(region.area, point);
  if (outer != geometry::PointLocation::inside) {
    return outer;
  }
  for (const auto& hole : region.holes) {
    const auto location = geometry::locate_point(hole, point);
    if (location == geometry::PointLocation::boundary) {
      return geometry::PointLocation::boundary;
    }
    if (location == geometry::PointLocation::inside) {
      return geometry::PointLocation::outside;
    }
  }
  return geometry::PointLocation::inside;
}

[[nodiscard]] bool boundaries_intersect(const PreparedFilledRegion& region,
                                        const geometry::PreparedPolygon& polygon) {
  if (geometry::polygon_boundaries_intersect(region.area, polygon)) {
    return true;
  }
  return std::any_of(region.holes.begin(), region.holes.end(), [&](const auto& hole) {
    return geometry::polygon_boundaries_intersect(hole, polygon);
  });
}

[[nodiscard]] bool prepared_filled_region_overlaps_polygon(
    const PreparedFilledRegion& region, const geometry::PreparedPolygon& polygon) {
  if (!region.area.valid() || !polygon.valid() || !region.area.bounds.overlaps(polygon.bounds)) {
    return false;
  }
  if (boundaries_intersect(region, polygon)) {
    return true;
  }
  if (std::any_of(polygon.polygon.vertices.begin(), polygon.polygon.vertices.end(),
                  [&](Point point) {
                    return locate_point(region, point) != geometry::PointLocation::outside;
                  })) {
    return true;
  }
  return std::any_of(region.area.polygon.vertices.begin(), region.area.polygon.vertices.end(),
                     [&](Point point) {
                       return geometry::locate_point(polygon, point) !=
                              geometry::PointLocation::outside;
                     });
}

[[nodiscard]] bool prepared_filled_regions_overlap(const PreparedFilledRegion& lhs,
                                                   const PreparedFilledRegion& rhs) {
  if (!lhs.area.valid() || !rhs.area.valid() || !lhs.area.bounds.overlaps(rhs.area.bounds)) {
    return false;
  }
  if (boundaries_intersect(lhs, rhs.area) ||
      std::any_of(rhs.holes.begin(), rhs.holes.end(),
                  [&](const auto& hole) { return boundaries_intersect(lhs, hole); })) {
    return true;
  }
  if (std::any_of(lhs.area.polygon.vertices.begin(), lhs.area.polygon.vertices.end(),
                  [&](Point point) {
                    return locate_point(rhs, point) != geometry::PointLocation::outside;
                  })) {
    return true;
  }
  return std::any_of(rhs.area.polygon.vertices.begin(), rhs.area.polygon.vertices.end(),
                     [&](Point point) {
                       return locate_point(lhs, point) != geometry::PointLocation::outside;
                     });
}

[[nodiscard]] bool validate_input(const FreeSpaceDomain& domain,
                                  const FreeSpaceOffsetConfig& config,
                                  FreeSpaceOffsetResult& result) {
  if (config.clearance.value() < 0 || config.maximum_output_vertices == 0U ||
      config.clearance.value() >= std::numeric_limits<Coordinate>::max()) {
    result.diagnostic = "free-space offset clearance and output budget are invalid";
    return false;
  }
  if (!geometry::is_simple_polygon(domain.outline)) {
    result.diagnostic = "free-space offset outline must be a simple polygon";
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
          "free-space offset cutouts must be simple, disjoint, and strictly inside the outline";
      return false;
    }
  }
  for (std::size_t index = 0; index < domain.obstacles.size(); ++index) {
    const auto& obstacle = domain.obstacles[index];
    if (!geometry::is_simple_polygon(obstacle)) {
      result.diagnostic = "free-space offset obstacles must be simple polygons";
      return false;
    }
  }
  for (const auto& obstacle : domain.filled_obstacles) {
    if (!geometry::is_simple_polygon(obstacle.area)) {
      result.diagnostic = "filled offset obstacles require a simple outer contour";
      return false;
    }
    for (std::size_t index = 0; index < obstacle.holes.size(); ++index) {
      const auto& hole = obstacle.holes[index];
      if (!geometry::is_simple_polygon(hole) || !geometry::polygon_inside(hole, obstacle.area) ||
          geometry::polygon_boundary_distance_nm(hole, obstacle.area) <= 0.0L ||
          std::any_of(
              obstacle.holes.begin(), obstacle.holes.begin() + static_cast<std::ptrdiff_t>(index),
              [&hole](const Polygon& prior) { return geometry::polygons_overlap(hole, prior); })) {
        result.diagnostic = "filled offset obstacle holes must be simple and disjoint";
        return false;
      }
    }
  }
  return true;
}

[[nodiscard]] BoostPolygon to_boost_polygon(const Polygon& polygon) {
  std::vector<BoostPoint> points;
  points.reserve(polygon.vertices.size());
  for (const auto point : polygon.vertices) {
    points.emplace_back(static_cast<Coordinate>(point.x.value()),
                        static_cast<Coordinate>(point.y.value()));
  }
  BoostPolygon result;
  boost::polygon::set_points(result, points.begin(), points.end());
  return result;
}

template <typename BoostPolygonLike>
[[nodiscard]] Polygon from_boost_polygon(const BoostPolygonLike& polygon) {
  Polygon result;
  for (auto point = boost::polygon::begin_points(polygon);
       point != boost::polygon::end_points(polygon); ++point) {
    result.vertices.push_back(
        Point{Nanometres{static_cast<std::int64_t>(boost::polygon::x(*point))},
              Nanometres{static_cast<std::int64_t>(boost::polygon::y(*point))}});
  }
  if (result.vertices.size() > 1U && result.vertices.front() == result.vertices.back()) {
    result.vertices.pop_back();
  }
  return result;
}

[[nodiscard]] long double contour_source_clearance(const Polygon& contour,
                                                   const FreeSpaceDomain& domain) {
  auto clearance = geometry::polygon_boundary_distance_nm(contour, domain.outline);
  for (const auto& cutout : domain.cutouts) {
    clearance = std::min(clearance, geometry::polygon_boundary_distance_nm(contour, cutout));
  }
  for (const auto& obstacle : domain.obstacles) {
    clearance = std::min(clearance, geometry::polygon_boundary_distance_nm(contour, obstacle));
  }
  for (const auto& obstacle : domain.filled_obstacles) {
    clearance = std::min(clearance, geometry::polygon_boundary_distance_nm(contour, obstacle.area));
    for (const auto& hole : obstacle.holes) {
      clearance = std::min(clearance, geometry::polygon_boundary_distance_nm(contour, hole));
    }
  }
  return clearance;
}

[[nodiscard]] bool validate_region(const OffsetFreeSpaceRegion& region,
                                   const FreeSpaceDomain& domain,
                                   const PreparedValidationDomain& prepared_domain,
                                   Nanometres clearance, std::string& diagnostic) {
  if (!geometry::is_simple_polygon(region.area)) {
    diagnostic = "output outer contour is not simple";
    return false;
  }
  if (!geometry::polygon_inside(region.area, domain.outline)) {
    diagnostic = "output outer contour is outside the source outline";
    return false;
  }
  // A zero requested certificate is already implied by the exact containment/overlap checks
  // below. Computing every output-to-source segment pair in that case is pure O(n*m) work; the
  // Boost Boolean still applies its two-nanometre rounding guard before this validation.
  if (clearance.value() > 0 &&
      contour_source_clearance(region.area, domain) <
          static_cast<long double>(clearance.value())) {
    diagnostic = "output outer contour violates source-boundary clearance";
    return false;
  }
  const auto prepared_filled = prepare_filled_region(region.area, region.holes);
  for (const auto& cutout : prepared_domain.cutouts) {
    if (prepared_filled_region_overlaps_polygon(prepared_filled, cutout)) {
      diagnostic = "output region overlaps a board cutout";
      return false;
    }
  }
  for (std::size_t index = 0U; index < prepared_domain.obstacles.size(); ++index) {
    if (prepared_filled_region_overlaps_polygon(prepared_filled,
                                                prepared_domain.obstacles[index])) {
      diagnostic = "output region overlaps polygonal obstacle " + std::to_string(index);
      return false;
    }
  }
  for (std::size_t index = 0U; index < prepared_domain.filled_obstacles.size(); ++index) {
    if (prepared_filled_regions_overlap(prepared_filled,
                                        prepared_domain.filled_obstacles[index])) {
      diagnostic = "output region overlaps filled obstacle " + std::to_string(index);
      return false;
    }
  }
  for (std::size_t index = 0; index < region.holes.size(); ++index) {
    const auto& hole = region.holes[index];
    if (!geometry::is_simple_polygon(hole)) {
      diagnostic = "output hole " + std::to_string(index) + " is not simple";
      return false;
    }
    const auto& prepared_hole = prepared_filled.holes[index];
    // Output holes must be strictly interior. For simple polygons, one interior vertex plus
    // non-intersecting boundaries proves containment; both queries use the prepared outer BVH.
    // This replaces the former O(hole_vertices * outer_vertices) containment and distance pair.
    if (geometry::locate_point(prepared_filled.area, hole.vertices.front()) !=
            geometry::PointLocation::inside ||
        geometry::polygon_boundaries_intersect(prepared_hole, prepared_filled.area)) {
      diagnostic = "output hole " + std::to_string(index) + " is not strictly inside its outer";
      return false;
    }
    if (clearance.value() > 0 &&
        contour_source_clearance(hole, domain) <
            static_cast<long double>(clearance.value())) {
      diagnostic = "output hole " + std::to_string(index) + " violates source clearance";
      return false;
    }
    // The complete region was prepared above for obstacle validation. Reuse those immutable
    // bounds/BVHs here instead of reconstructing bounds and testing every segment pair for every
    // output-hole pair. Boost can return hundreds of obstacle-shaped holes on dense boards.
    if (std::any_of(prepared_filled.holes.begin(),
                    prepared_filled.holes.begin() + static_cast<std::ptrdiff_t>(index),
                    [&](const auto& prior) {
                      return geometry::polygons_overlap(prepared_hole, prior);
                    })) {
      diagnostic = "output hole " + std::to_string(index) + " overlaps a prior hole";
      return false;
    }
  }
  return true;
}

} // namespace

FreeSpaceOffsetResult offset_polygonal_free_space(const FreeSpaceDomain& domain,
                                                  const FreeSpaceOffsetConfig& config) {
  FreeSpaceOffsetResult result;
  if (!validate_input(domain, config, result)) {
    return result;
  }
  const auto prepared_domain = prepare_validation_domain(domain);

  const auto guarded_clearance = static_cast<Coordinate>(config.clearance.value() + 2);
  using namespace boost::polygon::operators;
  BoostPolygonSet free_space;
  free_space.insert(to_boost_polygon(domain.outline));
  BoostPolygonSet cutouts;
  for (const auto& cutout : domain.cutouts) {
    cutouts.insert(to_boost_polygon(cutout));
  }
  free_space -= cutouts;
  free_space.resize(-guarded_clearance, false);

  BoostPolygonSet blocked;
  for (const auto& obstacle : domain.obstacles) {
    blocked.insert(to_boost_polygon(obstacle));
  }
  for (const auto& obstacle : domain.filled_obstacles) {
    BoostPolygonSet region;
    region.insert(to_boost_polygon(obstacle.area));
    BoostPolygonSet holes;
    for (const auto& hole : obstacle.holes) {
      holes.insert(to_boost_polygon(hole));
    }
    region -= holes;
    blocked += region;
  }
  if (!domain.obstacles.empty() || !domain.filled_obstacles.empty()) {
    blocked.resize(guarded_clearance, false);
    free_space -= blocked;
  }

  std::list<BoostPolygonWithHoles> polygons;
  free_space.get(polygons);
  for (const auto& polygon : polygons) {
    OffsetFreeSpaceRegion region;
    region.area = from_boost_polygon(polygon);
    for (auto hole = boost::polygon::begin_holes(polygon);
         hole != boost::polygon::end_holes(polygon); ++hole) {
      region.holes.push_back(from_boost_polygon(*hole));
    }
    auto vertices = region.area.vertices.size();
    for (const auto& hole : region.holes) {
      if (hole.vertices.size() > config.maximum_output_vertices - vertices) {
        result.diagnostic = "free-space offset exceeds its output-vertex budget";
        result.regions.clear();
        return result;
      }
      vertices += hole.vertices.size();
    }
    if (vertices > config.maximum_output_vertices - result.output_vertices) {
      result.diagnostic = "free-space offset exceeds its output-vertex budget";
      result.regions.clear();
      return result;
    }
    std::string validation_diagnostic;
    if (!validate_region(region, domain, prepared_domain, config.clearance,
                         validation_diagnostic)) {
      result.diagnostic =
          "free-space offset output failed exact conservative validation: " + validation_diagnostic;
      result.regions.clear();
      return result;
    }
    result.output_vertices += vertices;
    result.regions.push_back(std::move(region));
  }
  std::sort(result.regions.begin(), result.regions.end(),
            [](const auto& lhs, const auto& rhs) { return lhs.area.vertices < rhs.area.vertices; });
  return result;
}

} // namespace flux
