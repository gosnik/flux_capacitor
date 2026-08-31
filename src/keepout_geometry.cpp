#include "flux/core/keepout_geometry.hpp"

#include "flux/core/geometry.hpp"

#include <algorithm>

namespace flux {

std::vector<WorldKeepout> world_keepouts(const Board& board) {
  std::vector<WorldKeepout> result;
  auto count = board.keepouts.size();
  for (const auto& footprint : board.footprints) {
    count += footprint.keepouts.size();
  }
  result.reserve(count);
  for (const auto& keepout : board.keepouts) {
    result.push_back(
        WorldKeepout{keepout.id, keepout.area, keepout.layers, keepout.prohibit, std::nullopt});
  }
  for (const auto& footprint : board.footprints) {
    for (const auto& keepout : footprint.keepouts) {
      result.push_back(WorldKeepout{keepout.id,
                                    geometry::transform_polygon(keepout.area, footprint.position,
                                                                footprint.rotation, footprint.side),
                                    keepout.layers, keepout.prohibit, footprint.id});
    }
  }
  return result;
}

bool keepout_contains_layer(const WorldKeepout& keepout, const LayerId& layer) {
  return std::find(keepout.layers.begin(), keepout.layers.end(), layer) != keepout.layers.end();
}

bool point_copper_overlaps_keepout(Point center, long double radius_nm, const Polygon& keepout) {
  return geometry::locate_point(keepout, center) != geometry::PointLocation::outside ||
         geometry::point_polygon_boundary_distance_nm(center, keepout) <= radius_nm;
}

bool segment_copper_overlaps_keepout(Point start, Point end, long double radius_nm,
                                     const Polygon& keepout) {
  return geometry::locate_point(keepout, start) != geometry::PointLocation::outside ||
         geometry::locate_point(keepout, end) != geometry::PointLocation::outside ||
         geometry::segment_polygon_boundary_distance_nm(start, end, keepout) <= radius_nm;
}

} // namespace flux
