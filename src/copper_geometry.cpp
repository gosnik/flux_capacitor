#include "flux/core/copper_geometry.hpp"

#include "flux/core/geometry.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

namespace flux {
namespace {

struct Vector2 {
  long double x{0.0L};
  long double y{0.0L};
};

struct RoundedCore {
  std::vector<Vector2> vertices;
  long double radius_nm{0.0L};
};

[[nodiscard]] Vector2 operator-(Vector2 lhs, Vector2 rhs) { return {lhs.x - rhs.x, lhs.y - rhs.y}; }

[[nodiscard]] long double dot(Vector2 lhs, Vector2 rhs) { return lhs.x * rhs.x + lhs.y * rhs.y; }

[[nodiscard]] long double cross(Vector2 lhs, Vector2 rhs) { return lhs.x * rhs.y - lhs.y * rhs.x; }

[[nodiscard]] long double cross(Vector2 origin, Vector2 lhs, Vector2 rhs) {
  return cross(lhs - origin, rhs - origin);
}

[[nodiscard]] long double vector_distance(Vector2 lhs, Vector2 rhs) {
  return std::hypot(lhs.x - rhs.x, lhs.y - rhs.y);
}

[[nodiscard]] Vector2 as_vector(Point point) {
  return {static_cast<long double>(point.x.value()), static_cast<long double>(point.y.value())};
}

[[nodiscard]] long double orientation_tolerance(Vector2 first, Vector2 second, Vector2 third) {
  const auto scale = std::max({1.0L, std::abs(first.x), std::abs(first.y), std::abs(second.x),
                               std::abs(second.y), std::abs(third.x), std::abs(third.y)});
  return 1.0e-15L * scale * scale;
}

[[nodiscard]] bool point_on_segment(Vector2 point, Vector2 start, Vector2 end) {
  if (std::abs(cross(start, end, point)) > orientation_tolerance(start, end, point)) {
    return false;
  }
  const auto tolerance = 1.0e-9L;
  return point.x >= std::min(start.x, end.x) - tolerance &&
         point.x <= std::max(start.x, end.x) + tolerance &&
         point.y >= std::min(start.y, end.y) - tolerance &&
         point.y <= std::max(start.y, end.y) + tolerance;
}

[[nodiscard]] int orientation(Vector2 first, Vector2 second, Vector2 third) {
  const auto value = cross(first, second, third);
  const auto tolerance = orientation_tolerance(first, second, third);
  return value > tolerance ? 1 : (value < -tolerance ? -1 : 0);
}

[[nodiscard]] bool segments_intersect(Vector2 first_start, Vector2 first_end, Vector2 second_start,
                                      Vector2 second_end) {
  const auto first_a = orientation(first_start, first_end, second_start);
  const auto first_b = orientation(first_start, first_end, second_end);
  const auto second_a = orientation(second_start, second_end, first_start);
  const auto second_b = orientation(second_start, second_end, first_end);
  if (first_a != first_b && second_a != second_b) {
    return true;
  }
  return (first_a == 0 && point_on_segment(second_start, first_start, first_end)) ||
         (first_b == 0 && point_on_segment(second_end, first_start, first_end)) ||
         (second_a == 0 && point_on_segment(first_start, second_start, second_end)) ||
         (second_b == 0 && point_on_segment(first_end, second_start, second_end));
}

[[nodiscard]] long double point_segment_distance(Vector2 point, Vector2 start, Vector2 end) {
  const auto delta = end - start;
  const auto length_squared = dot(delta, delta);
  if (length_squared == 0.0L) {
    return vector_distance(point, start);
  }
  const auto parameter = std::clamp(dot(point - start, delta) / length_squared, 0.0L, 1.0L);
  return vector_distance(point, {start.x + parameter * delta.x, start.y + parameter * delta.y});
}

[[nodiscard]] long double segment_distance(Vector2 first_start, Vector2 first_end,
                                           Vector2 second_start, Vector2 second_end) {
  if (segments_intersect(first_start, first_end, second_start, second_end)) {
    return 0.0L;
  }
  return std::min({point_segment_distance(first_start, second_start, second_end),
                   point_segment_distance(first_end, second_start, second_end),
                   point_segment_distance(second_start, first_start, first_end),
                   point_segment_distance(second_end, first_start, first_end)});
}

[[nodiscard]] bool point_in_convex_polygon(Vector2 point, const std::vector<Vector2>& polygon) {
  if (polygon.size() < 3U) {
    return false;
  }
  int sign = 0;
  for (std::size_t i = 0; i < polygon.size(); ++i) {
    const auto& start = polygon[i];
    const auto& end = polygon[(i + 1U) % polygon.size()];
    const auto current = orientation(start, end, point);
    if (current == 0) {
      continue;
    }
    if (sign != 0 && current != sign) {
      return false;
    }
    sign = current;
  }
  return true;
}

[[nodiscard]] bool point_in_polygon(Vector2 point, const Polygon& polygon) {
  bool inside = false;
  for (std::size_t i = 0, previous = polygon.vertices.size() - 1U; i < polygon.vertices.size();
       previous = i++) {
    const auto current = as_vector(polygon.vertices[i]);
    const auto prior = as_vector(polygon.vertices[previous]);
    if (point_on_segment(point, prior, current)) {
      return true;
    }
    const bool straddles = (current.y > point.y) != (prior.y > point.y);
    if (straddles) {
      const auto crossing_x =
          (prior.x - current.x) * (point.y - current.y) / (prior.y - current.y) + current.x;
      if (point.x < crossing_x) {
        inside = !inside;
      }
    }
  }
  return inside;
}

[[nodiscard]] std::vector<std::pair<Vector2, Vector2>> edges(const RoundedCore& core) {
  std::vector<std::pair<Vector2, Vector2>> result;
  if (core.vertices.size() == 2U) {
    result.emplace_back(core.vertices[0], core.vertices[1]);
  } else if (core.vertices.size() > 2U) {
    result.reserve(core.vertices.size());
    for (std::size_t i = 0; i < core.vertices.size(); ++i) {
      result.emplace_back(core.vertices[i], core.vertices[(i + 1U) % core.vertices.size()]);
    }
  }
  return result;
}

[[nodiscard]] long double point_core_distance(Vector2 point, const RoundedCore& core) {
  if (core.vertices.size() == 1U) {
    return vector_distance(point, core.vertices.front());
  }
  if (core.vertices.size() == 2U) {
    return point_segment_distance(point, core.vertices[0], core.vertices[1]);
  }
  if (point_in_convex_polygon(point, core.vertices)) {
    return 0.0L;
  }
  auto result = std::numeric_limits<long double>::infinity();
  for (const auto& [start, end] : edges(core)) {
    result = std::min(result, point_segment_distance(point, start, end));
  }
  return result;
}

[[nodiscard]] long double segment_core_distance(Vector2 start, Vector2 end,
                                                const RoundedCore& core) {
  if (core.vertices.size() == 1U) {
    return point_segment_distance(core.vertices.front(), start, end);
  }
  if (core.vertices.size() == 2U) {
    return segment_distance(start, end, core.vertices[0], core.vertices[1]);
  }
  if (point_in_convex_polygon(start, core.vertices) ||
      point_in_convex_polygon(end, core.vertices)) {
    return 0.0L;
  }
  auto result = std::numeric_limits<long double>::infinity();
  for (const auto& [edge_start, edge_end] : edges(core)) {
    result = std::min(result, segment_distance(start, end, edge_start, edge_end));
  }
  return result;
}

[[nodiscard]] long double core_distance(const RoundedCore& lhs, const RoundedCore& rhs) {
  if (lhs.vertices.size() == 1U) {
    return point_core_distance(lhs.vertices.front(), rhs);
  }
  if (rhs.vertices.size() == 1U) {
    return point_core_distance(rhs.vertices.front(), lhs);
  }
  if (lhs.vertices.size() == 2U) {
    return segment_core_distance(lhs.vertices[0], lhs.vertices[1], rhs);
  }
  if (rhs.vertices.size() == 2U) {
    return segment_core_distance(rhs.vertices[0], rhs.vertices[1], lhs);
  }
  if (point_in_convex_polygon(lhs.vertices.front(), rhs.vertices) ||
      point_in_convex_polygon(rhs.vertices.front(), lhs.vertices)) {
    return 0.0L;
  }
  auto result = std::numeric_limits<long double>::infinity();
  for (const auto& [lhs_start, lhs_end] : edges(lhs)) {
    for (const auto& [rhs_start, rhs_end] : edges(rhs)) {
      result = std::min(result, segment_distance(lhs_start, lhs_end, rhs_start, rhs_end));
    }
  }
  return result;
}

[[nodiscard]] long double circular_arc_core_distance(Point start, Point middle, Point end,
                                                     const RoundedCore& core) {
  if (core.vertices.size() == 1U) {
    const auto point = core.vertices.front();
    const auto quantized = Point{Nanometres{static_cast<std::int64_t>(std::llround(point.x))},
                                 Nanometres{static_cast<std::int64_t>(std::llround(point.y))}};
    return geometry::point_circular_arc_distance_nm(quantized, start, middle, end);
  }
  if (core.vertices.size() == 2U) {
    const auto first =
        Point{Nanometres{static_cast<std::int64_t>(std::llround(core.vertices[0].x))},
              Nanometres{static_cast<std::int64_t>(std::llround(core.vertices[0].y))}};
    const auto second =
        Point{Nanometres{static_cast<std::int64_t>(std::llround(core.vertices[1].x))},
              Nanometres{static_cast<std::int64_t>(std::llround(core.vertices[1].y))}};
    return geometry::segment_circular_arc_distance_nm(first, second, start, middle, end);
  }
  const auto critical = geometry::circular_arc_extrema(start, middle, end);
  if (std::any_of(critical.begin(), critical.end(), [&core](Point point) {
        return point_in_convex_polygon(as_vector(point), core.vertices);
      })) {
    return 0.0L;
  }
  auto result = std::numeric_limits<long double>::infinity();
  for (const auto& [edge_start, edge_end] : edges(core)) {
    const auto first = Point{Nanometres{static_cast<std::int64_t>(std::llround(edge_start.x))},
                             Nanometres{static_cast<std::int64_t>(std::llround(edge_start.y))}};
    const auto second = Point{Nanometres{static_cast<std::int64_t>(std::llround(edge_end.x))},
                              Nanometres{static_cast<std::int64_t>(std::llround(edge_end.y))}};
    result = std::min(
        result, geometry::segment_circular_arc_distance_nm(first, second, start, middle, end));
  }
  return result;
}

[[nodiscard]] RoundedCore pad_core(const CopperPadShape& pad) {
  const auto half_width = static_cast<long double>(pad.width.value()) / 2.0L;
  const auto half_height = static_cast<long double>(pad.height.value()) / 2.0L;
  long double radius = 0.0L;
  long double core_half_width = 0.0L;
  long double core_half_height = 0.0L;
  switch (pad.shape) {
  case PadShape::circle:
    radius = std::max(half_width, half_height);
    break;
  case PadShape::rectangle:
    core_half_width = half_width;
    core_half_height = half_height;
    break;
  case PadShape::oval:
    radius = std::min(half_width, half_height);
    core_half_width = half_width - radius;
    core_half_height = half_height - radius;
    break;
  case PadShape::round_rectangle:
    radius = static_cast<long double>(pad.corner_radius.value());
    core_half_width = std::max(0.0L, half_width - radius);
    core_half_height = std::max(0.0L, half_height - radius);
    break;
  case PadShape::trapezoid:
  case PadShape::custom:
  case PadShape::unknown:
    radius = pad.conservative_radius_nm;
    break;
  }

  std::vector<Vector2> local;
  if (core_half_width > 0.0L && core_half_height > 0.0L) {
    local = {{-core_half_width, -core_half_height},
             {core_half_width, -core_half_height},
             {core_half_width, core_half_height},
             {-core_half_width, core_half_height}};
  } else if (core_half_width > 0.0L) {
    local = {{-core_half_width, 0.0L}, {core_half_width, 0.0L}};
  } else if (core_half_height > 0.0L) {
    local = {{0.0L, -core_half_height}, {0.0L, core_half_height}};
  } else {
    local = {{0.0L, 0.0L}};
  }

  constexpr long double pi = 3.141592653589793238462643383279502884L;
  const auto angle = static_cast<long double>(pad.rotation.value()) * pi / 180'000'000.0L;
  const auto cosine = std::cos(angle);
  const auto sine = std::sin(angle);
  const auto center = as_vector(pad.position);
  for (auto& point : local) {
    point = {center.x + point.x * cosine - point.y * sine,
             center.y + point.x * sine + point.y * cosine};
  }
  return {std::move(local), radius};
}

[[nodiscard]] long double core_polygon_boundary_distance(const RoundedCore& core,
                                                         const Polygon& polygon) {
  auto result = std::numeric_limits<long double>::infinity();
  if (polygon.vertices.empty()) {
    return result;
  }
  if (core.vertices.size() == 1U) {
    for (std::size_t i = 0; i < polygon.vertices.size(); ++i) {
      result = std::min(
          result,
          point_segment_distance(core.vertices.front(), as_vector(polygon.vertices[i]),
                                 as_vector(polygon.vertices[(i + 1U) % polygon.vertices.size()])));
    }
    return result;
  }
  for (const auto& [core_start, core_end] : edges(core)) {
    for (std::size_t i = 0; i < polygon.vertices.size(); ++i) {
      result = std::min(
          result,
          segment_distance(core_start, core_end, as_vector(polygon.vertices[i]),
                           as_vector(polygon.vertices[(i + 1U) % polygon.vertices.size()])));
    }
  }
  return result;
}

} // namespace

long double conservative_pad_radius_nm(const Pad& pad) {
  if (!pad.plated && pad_has_drill(pad)) {
    return static_cast<long double>(std::max(pad.drill_width, pad.drill_height).value()) / 2.0L;
  }
  const auto half_width = static_cast<long double>(pad.width.value()) / 2.0L;
  const auto half_height = static_cast<long double>(pad.height.value()) / 2.0L;
  if (pad.shape == PadShape::circle || pad.shape == PadShape::oval) {
    return std::max(half_width, half_height);
  }
  return std::hypot(half_width, half_height);
}

bool pad_has_drill(const Pad& pad) {
  return pad.drill_width.value() > 0 && pad.drill_height.value() > 0;
}

Point pad_drill_world_position(const Footprint& footprint, const Pad& pad) {
  const auto local_position = geometry::transform_point(
      pad.drill_offset, pad.local_position, pad.rotation, BoardSide::front);
  return geometry::transform_point(local_position, footprint.position, footprint.rotation,
                                   footprint.side);
}

std::vector<CopperPadShape> copper_pad_shapes(const Board& board) {
  std::vector<CopperPadShape> result;
  for (const auto& footprint : board.footprints) {
    for (const auto& pad : footprint.pads) {
      const bool drill_only = !pad.plated && pad_has_drill(pad);
      const auto rotation_value =
          footprint.rotation.value() +
          (footprint.side == BoardSide::front ? pad.rotation.value() : -pad.rotation.value());
      result.push_back(CopperPadShape{
          pad.id, pad.net,
          drill_only ? pad_drill_world_position(footprint, pad)
                     : geometry::transform_point(pad.local_position, footprint.position,
                                                 footprint.rotation, footprint.side),
          pad.layers,
          drill_only && pad.drill_width != pad.drill_height ? PadShape::oval
                                                           : drill_only ? PadShape::circle
                                                                        : pad.shape,
          drill_only ? pad.drill_width : pad.width,
          drill_only ? pad.drill_height : pad.height,
          drill_only ? Nanometres{0} : pad.corner_radius,
          Microdegrees{-rotation_value}, conservative_pad_radius_nm(pad)});
    }
  }
  return result;
}

std::optional<PolygonalPadOutline> polygonal_pad_outline(const CopperPadShape& pad,
                                                         Nanometres maximum_radial_excess) {
  if (maximum_radial_excess.value() <= 0 || pad.width.value() <= 0 || pad.height.value() <= 0) {
    return std::nullopt;
  }

  if (pad.shape == PadShape::circle) {
    const auto radius =
        static_cast<long double>(std::max(pad.width.value(), pad.height.value())) / 2.0L;
    const auto excess = static_cast<long double>(maximum_radial_excess.value());
    const auto pi = std::acos(-1.0L);
    const auto ratio = std::clamp(radius / (radius + excess), 0.0L, 1.0L);
    const auto half_angle = std::acos(ratio);
    auto sides =
        half_angle > 0.0L ? static_cast<std::size_t>(std::ceil(pi / half_angle)) : std::size_t{256};
    sides = std::clamp(sides, std::size_t{16}, std::size_t{256});
    const auto enclosing_radius = radius / std::cos(pi / static_cast<long double>(sides)) + 1.0L;
    Polygon area;
    area.vertices.reserve(sides);
    for (std::size_t index = 0; index < sides; ++index) {
      const auto angle =
          2.0L * pi * static_cast<long double>(index) / static_cast<long double>(sides);
      area.vertices.push_back(Point{Nanometres{static_cast<std::int64_t>(std::llround(
                                        static_cast<long double>(pad.position.x.value()) +
                                        enclosing_radius * std::cos(angle)))},
                                    Nanometres{static_cast<std::int64_t>(std::llround(
                                        static_cast<long double>(pad.position.y.value()) +
                                        enclosing_radius * std::sin(angle)))}});
    }
    return geometry::is_simple_polygon(area)
               ? std::optional<PolygonalPadOutline>{PolygonalPadOutline{std::move(area), true}}
               : std::nullopt;
  }

  if (pad.shape != PadShape::rectangle && pad.shape != PadShape::oval &&
      pad.shape != PadShape::round_rectangle) {
    return std::nullopt;
  }
  auto enclosing_pad = pad;
  enclosing_pad.shape = PadShape::rectangle;
  enclosing_pad.corner_radius = Nanometres{0};
  const auto core = pad_core(enclosing_pad);
  if (core.radius_nm != 0.0L || core.vertices.size() != 4U) {
    return std::nullopt;
  }
  Polygon area;
  area.vertices.reserve(core.vertices.size());
  for (const auto vertex : core.vertices) {
    area.vertices.push_back(Point{Nanometres{static_cast<std::int64_t>(std::llround(vertex.x))},
                                  Nanometres{static_cast<std::int64_t>(std::llround(vertex.y))}});
  }
  return geometry::is_simple_polygon(area) ? std::optional<PolygonalPadOutline>{PolygonalPadOutline{
                                                 std::move(area), pad.shape != PadShape::rectangle}}
                                           : std::nullopt;
}

long double point_to_pad_copper_distance_nm(Point point, const CopperPadShape& pad) {
  const auto core = pad_core(pad);
  return std::max(0.0L, point_core_distance(as_vector(point), core) - core.radius_nm);
}

long double segment_to_pad_copper_distance_nm(Point start, Point end, const CopperPadShape& pad) {
  const auto core = pad_core(pad);
  return std::max(0.0L,
                  segment_core_distance(as_vector(start), as_vector(end), core) - core.radius_nm);
}

long double circular_arc_to_pad_copper_distance_nm(Point start, Point middle, Point end,
                                                   const CopperPadShape& pad) {
  const auto core = pad_core(pad);
  // Polygonal pad cores are rotated in long double and then quantized for the
  // canonical arc/segment predicates.  Keep rule acceptance conservative at
  // that integer boundary instead of allowing rounding to create clearance.
  constexpr long double quantization_guard_nm = 2.0L;
  return std::max(0.0L, circular_arc_core_distance(start, middle, end, core) - core.radius_nm -
                            quantization_guard_nm);
}

long double pad_to_pad_copper_distance_nm(const CopperPadShape& lhs, const CopperPadShape& rhs) {
  const auto lhs_core = pad_core(lhs);
  const auto rhs_core = pad_core(rhs);
  return std::max(0.0L,
                  core_distance(lhs_core, rhs_core) - lhs_core.radius_nm - rhs_core.radius_nm);
}

bool pad_copper_intersects(const CopperPadShape& lhs, const CopperPadShape& rhs) {
  const auto lhs_core = pad_core(lhs);
  const auto rhs_core = pad_core(rhs);
  return core_distance(lhs_core, rhs_core) <= lhs_core.radius_nm + rhs_core.radius_nm;
}

bool pad_copper_overlaps_polygon(const CopperPadShape& pad, const Polygon& polygon) {
  if (polygon.vertices.size() < 3U) {
    return false;
  }
  const auto core = pad_core(pad);
  if (std::any_of(core.vertices.begin(), core.vertices.end(),
                  [&polygon](const auto point) { return point_in_polygon(point, polygon); })) {
    return true;
  }
  if (std::any_of(polygon.vertices.begin(), polygon.vertices.end(), [&core](const auto point) {
        return point_core_distance(as_vector(point), core) <= core.radius_nm;
      })) {
    return true;
  }
  return core_polygon_boundary_distance(core, polygon) <= core.radius_nm;
}

long double pad_to_polygon_boundary_distance_nm(const CopperPadShape& pad, const Polygon& polygon) {
  const auto core = pad_core(pad);
  return std::max(0.0L, core_polygon_boundary_distance(core, polygon) - core.radius_nm);
}

bool pad_inside_polygon(const CopperPadShape& pad, const Polygon& polygon) {
  if (polygon.vertices.size() < 3U) {
    return false;
  }
  const auto core = pad_core(pad);
  if (!std::all_of(core.vertices.begin(), core.vertices.end(),
                   [&polygon](const auto point) { return point_in_polygon(point, polygon); })) {
    return false;
  }
  const auto boundary_distance = core_polygon_boundary_distance(core, polygon);
  // A zero-radius polygonal core can bridge a concavity while all of its vertices remain
  // inside. Treating boundary contact as outside is a deliberately conservative resolution
  // of that case; positive-radius shapes are rejected by the clearance comparison below.
  if (core.radius_nm == 0.0L && boundary_distance <= 1.0e-6L) {
    return false;
  }
  return boundary_distance + 1.0e-6L >= core.radius_nm;
}

bool pad_inside_board_area(const CopperPadShape& pad, const Board& board) {
  return pad_inside_polygon(pad, board.outline) &&
         std::none_of(board.cutouts.begin(), board.cutouts.end(), [&pad](const auto& cutout) {
           return pad_copper_overlaps_polygon(pad, cutout);
         });
}

long double pad_to_board_boundary_distance_nm(const CopperPadShape& pad, const Board& board) {
  auto distance = pad_to_polygon_boundary_distance_nm(pad, board.outline);
  for (const auto& cutout : board.cutouts) {
    distance = std::min(distance, pad_to_polygon_boundary_distance_nm(pad, cutout));
  }
  return distance;
}

bool pad_copper_overlaps_filled_region(const CopperPadShape& pad, const CopperZoneFill& region) {
  if (!pad_copper_overlaps_polygon(pad, region.area)) {
    return false;
  }
  return std::none_of(region.holes.begin(), region.holes.end(),
                      [&pad](const Polygon& hole) { return pad_inside_polygon(pad, hole); });
}

long double pad_to_filled_region_copper_distance_nm(const CopperPadShape& pad,
                                                    const CopperZoneFill& region) {
  if (pad_copper_overlaps_filled_region(pad, region)) {
    return 0.0L;
  }
  auto distance = pad_to_polygon_boundary_distance_nm(pad, region.area);
  for (const auto& hole : region.holes) {
    distance = std::min(distance, pad_to_polygon_boundary_distance_nm(pad, hole));
  }
  return distance;
}

} // namespace flux
