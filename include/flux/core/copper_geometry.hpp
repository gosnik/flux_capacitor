#pragma once

#include "flux/core/board.hpp"

#include <optional>
#include <vector>

namespace flux {

// World-space pad geometry shared by inner-loop DRC and routing. Unsupported KiCad
// shapes are explicitly lossy at import and retain a conservative circular fallback.
struct CopperPadShape {
  ObjectId id;
  NetId net;
  Point position;
  std::vector<LayerId> layers;
  PadShape shape{PadShape::circle};
  Nanometres width;
  Nanometres height;
  Nanometres corner_radius;
  Microdegrees rotation;
  long double conservative_radius_nm{0.0L};
};

[[nodiscard]] long double conservative_pad_radius_nm(const Pad& pad);
[[nodiscard]] bool pad_has_drill(const Pad& pad);
[[nodiscard]] Point pad_drill_world_position(const Footprint& footprint, const Pad& pad);
[[nodiscard]] std::vector<CopperPadShape> copper_pad_shapes(const Board& board);

struct PolygonalPadOutline {
  Polygon area;
  bool conservative{false};
};

[[nodiscard]] std::optional<PolygonalPadOutline>
polygonal_pad_outline(const CopperPadShape& pad,
                      Nanometres maximum_radial_excess = Nanometres::from_mm(0.01));

// Distances are non-negative copper-boundary gaps in nanometres. Supported shapes are
// circle, rectangle, oval, and round rectangle at arbitrary rotations.
[[nodiscard]] long double point_to_pad_copper_distance_nm(Point point, const CopperPadShape& pad);
[[nodiscard]] long double segment_to_pad_copper_distance_nm(Point start, Point end,
                                                            const CopperPadShape& pad);
[[nodiscard]] long double circular_arc_to_pad_copper_distance_nm(Point start, Point middle,
                                                                 Point end,
                                                                 const CopperPadShape& pad);
[[nodiscard]] long double pad_to_pad_copper_distance_nm(const CopperPadShape& lhs,
                                                        const CopperPadShape& rhs);
[[nodiscard]] bool pad_copper_intersects(const CopperPadShape& lhs, const CopperPadShape& rhs);
[[nodiscard]] bool pad_copper_overlaps_polygon(const CopperPadShape& pad, const Polygon& polygon);
[[nodiscard]] long double pad_to_polygon_boundary_distance_nm(const CopperPadShape& pad,
                                                              const Polygon& polygon);
[[nodiscard]] bool pad_inside_polygon(const CopperPadShape& pad, const Polygon& polygon);
[[nodiscard]] bool pad_inside_board_area(const CopperPadShape& pad, const Board& board);
[[nodiscard]] long double pad_to_board_boundary_distance_nm(const CopperPadShape& pad,
                                                            const Board& board);
[[nodiscard]] bool pad_copper_overlaps_filled_region(const CopperPadShape& pad,
                                                     const CopperZoneFill& region);
[[nodiscard]] long double pad_to_filled_region_copper_distance_nm(const CopperPadShape& pad,
                                                                  const CopperZoneFill& region);

} // namespace flux
