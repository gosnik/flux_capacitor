#pragma once

#include "flux/core/board.hpp"

#include <cstdint>
#include <cstddef>
#include <optional>
#include <vector>

namespace flux::geometry {

enum class PointLocation { outside, boundary, inside };

struct CircularArcParameters {
  long double center_x_nm{0.0L};
  long double center_y_nm{0.0L};
  long double radius_nm{0.0L};
  long double start_angle_radians{0.0L};
  long double sweep_radians{0.0L};
};

struct AxisAlignedBounds {
  Point minimum;
  Point maximum;

  [[nodiscard]] bool contains(Point point) const;
  [[nodiscard]] bool overlaps(const AxisAlignedBounds& other) const;
  bool operator==(const AxisAlignedBounds&) const = default;
};

struct PreparedBoundarySegment {
  Point start;
  Point end;
  AxisAlignedBounds bounds;
};

struct PreparedBoundaryNode {
  AxisAlignedBounds bounds;
  std::size_t first_segment{0};
  std::size_t segment_count{0};
  std::size_t left{0};
  std::size_t right{0};

  [[nodiscard]] bool leaf() const { return segment_count > 0U; }
};

// Immutable polygon query state. Bounds and boundary segments are computed once, and the
// deterministic median-split BVH is shared by point-location and nearest-boundary callers.
struct PreparedPolygon {
  Polygon polygon;
  AxisAlignedBounds bounds;
  std::vector<PreparedBoundarySegment> boundary_segments;
  std::vector<PreparedBoundaryNode> boundary_bvh;

  [[nodiscard]] bool valid() const { return polygon.vertices.size() >= 3U && !boundary_bvh.empty(); }
};

[[nodiscard]] PreparedPolygon prepare_polygon(const Polygon& polygon);
[[nodiscard]] PointLocation locate_point(const PreparedPolygon& polygon, Point point);
[[nodiscard]] long double point_polygon_boundary_distance_nm(Point point,
                                                             const PreparedPolygon& polygon);
[[nodiscard]] long double segment_polygon_boundary_distance_nm(Point start, Point end,
                                                               const PreparedPolygon& polygon);
// Returns min(actual distance, maximum_distance_nm). This is intended for threshold/certificate
// queries: seeding the BVH search with the only distance the caller cares about avoids traversing
// a distant polygon boundary merely to calculate a larger exact value.
[[nodiscard]] long double segment_polygon_boundary_distance_nm_capped(
    Point start, Point end, const PreparedPolygon& polygon, long double maximum_distance_nm);
[[nodiscard]] bool polygon_boundaries_intersect(const PreparedPolygon& lhs,
                                                const PreparedPolygon& rhs);
[[nodiscard]] bool polygons_overlap(const PreparedPolygon& lhs, const PreparedPolygon& rhs);

[[nodiscard]] bool coordinate_in_range(Point point);
[[nodiscard]] Point transform_point(Point local, Point translation, Microdegrees rotation,
                                    BoardSide side);
[[nodiscard]] Polygon transform_polygon(const Polygon& polygon, Point translation,
                                        Microdegrees rotation, BoardSide side);
[[nodiscard]] bool is_simple_polygon(const Polygon& polygon);
[[nodiscard]] long double signed_double_area(const Polygon& polygon);
[[nodiscard]] PointLocation locate_point(const Polygon& polygon, Point point);
[[nodiscard]] bool segments_intersect(Point a, Point b, Point c, Point d);
[[nodiscard]] bool point_on_segment(Point point, Point start, Point end);
[[nodiscard]] bool polygons_overlap(const Polygon& lhs, const Polygon& rhs);
[[nodiscard]] bool polygon_inside(const Polygon& inner, const Polygon& outer);
[[nodiscard]] std::uint64_t segment_length_nm(Point start, Point end);
[[nodiscard]] long double point_distance_nm(Point lhs, Point rhs);
[[nodiscard]] long double point_segment_distance_nm(Point point, Point start, Point end);
[[nodiscard]] long double segment_distance_nm(Point lhs_start, Point lhs_end, Point rhs_start,
                                              Point rhs_end);
[[nodiscard]] std::optional<CircularArcParameters> circular_arc_parameters(Point start,
                                                                           Point middle, Point end);
[[nodiscard]] bool point_on_circular_arc(Point point, Point start, Point middle, Point end);
[[nodiscard]] std::uint64_t circular_arc_length_nm(Point start, Point middle, Point end);
[[nodiscard]] std::vector<Point> circular_arc_extrema(Point start, Point middle, Point end);
[[nodiscard]] long double point_circular_arc_distance_nm(Point point, Point start, Point middle,
                                                         Point end);
[[nodiscard]] long double segment_circular_arc_distance_nm(Point segment_start, Point segment_end,
                                                           Point arc_start, Point arc_middle,
                                                           Point arc_end);
[[nodiscard]] long double circular_arcs_distance_nm(Point lhs_start, Point lhs_middle,
                                                    Point lhs_end, Point rhs_start,
                                                    Point rhs_middle, Point rhs_end);
[[nodiscard]] long double point_polygon_boundary_distance_nm(Point point, const Polygon& polygon);
[[nodiscard]] long double segment_polygon_boundary_distance_nm(Point start, Point end,
                                                               const Polygon& polygon);
[[nodiscard]] long double circular_arc_polygon_boundary_distance_nm(Point start, Point middle,
                                                                    Point end,
                                                                    const Polygon& polygon);
[[nodiscard]] long double point_polygon_copper_distance_nm(Point point, const Polygon& polygon);
[[nodiscard]] long double segment_polygon_copper_distance_nm(Point start, Point end,
                                                             const Polygon& polygon);
[[nodiscard]] long double circular_arc_polygon_copper_distance_nm(Point start, Point middle,
                                                                  Point end,
                                                                  const Polygon& polygon);
[[nodiscard]] long double polygon_boundary_distance_nm(const Polygon& lhs, const Polygon& rhs);
[[nodiscard]] long double polygon_copper_distance_nm(const Polygon& lhs, const Polygon& rhs);
[[nodiscard]] bool segment_inside_polygon(Point start, Point end, const Polygon& polygon);
[[nodiscard]] bool circular_arc_inside_polygon(Point start, Point middle, Point end,
                                               const Polygon& polygon);
[[nodiscard]] PointLocation locate_point_in_board_area(const Board& board, Point point);
[[nodiscard]] long double point_board_boundary_distance_nm(Point point, const Board& board);
[[nodiscard]] long double segment_board_boundary_distance_nm(Point start, Point end,
                                                             const Board& board);
[[nodiscard]] long double circular_arc_board_boundary_distance_nm(Point start, Point middle,
                                                                  Point end, const Board& board);
[[nodiscard]] long double polygon_board_boundary_distance_nm(const Polygon& polygon,
                                                             const Board& board);
[[nodiscard]] bool segment_inside_board_area(Point start, Point end, const Board& board);
[[nodiscard]] bool circular_arc_inside_board_area(Point start, Point middle, Point end,
                                                  const Board& board);
[[nodiscard]] bool polygon_inside_board_area(const Polygon& polygon, const Board& board);
[[nodiscard]] PointLocation locate_point_in_filled_region(const CopperZoneFill& region,
                                                          Point point);
[[nodiscard]] long double point_filled_region_boundary_distance_nm(Point point,
                                                                   const CopperZoneFill& region);
[[nodiscard]] long double segment_filled_region_boundary_distance_nm(Point start, Point end,
                                                                     const CopperZoneFill& region);
[[nodiscard]] long double point_filled_region_copper_distance_nm(Point point,
                                                                 const CopperZoneFill& region);
[[nodiscard]] long double segment_filled_region_copper_distance_nm(Point start, Point end,
                                                                   const CopperZoneFill& region);
[[nodiscard]] long double
circular_arc_filled_region_copper_distance_nm(Point start, Point middle, Point end,
                                              const CopperZoneFill& region);
[[nodiscard]] bool filled_region_overlaps_polygon(const CopperZoneFill& region,
                                                  const Polygon& polygon);
[[nodiscard]] bool filled_regions_overlap(const CopperZoneFill& lhs, const CopperZoneFill& rhs);
[[nodiscard]] bool filled_region_inside_board_area(const CopperZoneFill& region,
                                                   const Board& board);
[[nodiscard]] long double filled_region_board_boundary_distance_nm(const CopperZoneFill& region,
                                                                   const Board& board);
[[nodiscard]] long double filled_region_polygon_copper_distance_nm(const CopperZoneFill& region,
                                                                   const Polygon& polygon);
[[nodiscard]] long double filled_regions_copper_distance_nm(const CopperZoneFill& lhs,
                                                            const CopperZoneFill& rhs);

} // namespace flux::geometry
