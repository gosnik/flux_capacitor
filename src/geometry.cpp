#include "flux/core/geometry.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <numeric>

namespace flux::geometry {
namespace {

constexpr long double pi = 3.141592653589793238462643383279502884L;
constexpr long double tau = 2.0L * pi;

[[nodiscard]] long double normalized_angle(long double angle) {
  angle = std::fmod(angle, tau);
  return angle < 0.0L ? angle + tau : angle;
}

[[nodiscard]] long double positive_angle_delta(long double start, long double end) {
  return normalized_angle(end - start);
}

[[nodiscard]] bool angle_on_arc(const CircularArcParameters& arc, long double angle) {
  constexpr long double tolerance = 1.0e-15L;
  if (arc.sweep_radians >= 0.0L) {
    return positive_angle_delta(arc.start_angle_radians, angle) <= arc.sweep_radians + tolerance;
  }
  return positive_angle_delta(angle, arc.start_angle_radians) <= -arc.sweep_radians + tolerance;
}

[[nodiscard]] long double squared(long double value) { return value * value; }

[[nodiscard]] Point quantized_point(long double x, long double y) {
  return Point{Nanometres{static_cast<std::int64_t>(std::llround(x))},
               Nanometres{static_cast<std::int64_t>(std::llround(y))}};
}

[[nodiscard]] std::int64_t cross(Point a, Point b, Point c) {
  const auto ab_x = b.x.value() - a.x.value();
  const auto ab_y = b.y.value() - a.y.value();
  const auto ac_x = c.x.value() - a.x.value();
  const auto ac_y = c.y.value() - a.y.value();
  return ab_x * ac_y - ab_y * ac_x;
}

[[nodiscard]] int orientation(Point a, Point b, Point c) {
  const auto value = cross(a, b, c);
  return (value > 0) - (value < 0);
}

[[nodiscard]] bool boxes_overlap(Point a, Point b, Point c, Point d) {
  const auto lhs_min_x = std::min(a.x.value(), b.x.value());
  const auto lhs_max_x = std::max(a.x.value(), b.x.value());
  const auto lhs_min_y = std::min(a.y.value(), b.y.value());
  const auto lhs_max_y = std::max(a.y.value(), b.y.value());
  const auto rhs_min_x = std::min(c.x.value(), d.x.value());
  const auto rhs_max_x = std::max(c.x.value(), d.x.value());
  const auto rhs_min_y = std::min(c.y.value(), d.y.value());
  const auto rhs_max_y = std::max(c.y.value(), d.y.value());
  return lhs_min_x <= rhs_max_x && rhs_min_x <= lhs_max_x && lhs_min_y <= rhs_max_y &&
         rhs_min_y <= lhs_max_y;
}

[[nodiscard]] bool polygon_boxes_overlap(const Polygon& lhs, const Polygon& rhs) {
  const auto lhs_x =
      std::minmax_element(lhs.vertices.begin(), lhs.vertices.end(),
                          [](Point first, Point second) { return first.x < second.x; });
  const auto lhs_y =
      std::minmax_element(lhs.vertices.begin(), lhs.vertices.end(),
                          [](Point first, Point second) { return first.y < second.y; });
  const auto rhs_x =
      std::minmax_element(rhs.vertices.begin(), rhs.vertices.end(),
                          [](Point first, Point second) { return first.x < second.x; });
  const auto rhs_y =
      std::minmax_element(rhs.vertices.begin(), rhs.vertices.end(),
                          [](Point first, Point second) { return first.y < second.y; });
  return lhs_x.first->x <= rhs_x.second->x && rhs_x.first->x <= lhs_x.second->x &&
         lhs_y.first->y <= rhs_y.second->y && rhs_y.first->y <= lhs_y.second->y;
}

[[nodiscard]] bool proper_segments_intersect(Point a, Point b, Point c, Point d) {
  const auto ab_c = orientation(a, b, c);
  const auto ab_d = orientation(a, b, d);
  const auto cd_a = orientation(c, d, a);
  const auto cd_b = orientation(c, d, b);
  return ab_c != 0 && ab_d != 0 && cd_a != 0 && cd_b != 0 && ab_c != ab_d && cd_a != cd_b;
}

[[nodiscard]] std::size_t next_index(std::size_t index, std::size_t size) {
  return (index + 1U) % size;
}

[[nodiscard]] AxisAlignedBounds segment_bounds(Point start, Point end) {
  return AxisAlignedBounds{Point{std::min(start.x, end.x), std::min(start.y, end.y)},
                           Point{std::max(start.x, end.x), std::max(start.y, end.y)}};
}

[[nodiscard]] AxisAlignedBounds merge_bounds(const AxisAlignedBounds& first,
                                             const AxisAlignedBounds& second) {
  return AxisAlignedBounds{Point{std::min(first.minimum.x, second.minimum.x),
                                 std::min(first.minimum.y, second.minimum.y)},
                           Point{std::max(first.maximum.x, second.maximum.x),
                                 std::max(first.maximum.y, second.maximum.y)}};
}

[[nodiscard]] long double point_bounds_distance_nm(Point point, const AxisAlignedBounds& bounds) {
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
  return std::hypot(dx, dy);
}

[[nodiscard]] long double segment_bounds_distance_lower_bound(Point start, Point end,
                                                              const AxisAlignedBounds& bounds) {
  const auto segment = segment_bounds(start, end);
  const auto dx =
      segment.maximum.x < bounds.minimum.x
          ? static_cast<long double>(bounds.minimum.x.value() - segment.maximum.x.value())
      : bounds.maximum.x < segment.minimum.x
          ? static_cast<long double>(segment.minimum.x.value() - bounds.maximum.x.value())
          : 0.0L;
  const auto dy =
      segment.maximum.y < bounds.minimum.y
          ? static_cast<long double>(bounds.minimum.y.value() - segment.maximum.y.value())
      : bounds.maximum.y < segment.minimum.y
          ? static_cast<long double>(segment.minimum.y.value() - bounds.maximum.y.value())
          : 0.0L;
  return std::hypot(dx, dy);
}

// Prepared polygons use a balanced median-split BVH, so normal traversals fit in this local stack
// without allocating. Retain a vector fallback for externally constructed or unusually deep
// prepared geometry rather than relying on that implementation detail for correctness.
class BoundaryNodeStack {
public:
  [[nodiscard]] bool empty() const { return local_size_ == 0U && overflow_.empty(); }

  void push(std::size_t value) {
    if (!overflow_.empty() || local_size_ == local_.size()) {
      if (overflow_.empty()) {
        overflow_.assign(local_.begin(), local_.begin() + static_cast<std::ptrdiff_t>(local_size_));
        local_size_ = 0U;
      }
      overflow_.push_back(value);
      return;
    }
    local_[local_size_++] = value;
  }

  [[nodiscard]] std::size_t pop() {
    if (!overflow_.empty()) {
      const auto value = overflow_.back();
      overflow_.pop_back();
      return value;
    }
    return local_[--local_size_];
  }

private:
  std::array<std::size_t, 64U> local_{};
  std::size_t local_size_{0U};
  std::vector<std::size_t> overflow_;
};

} // namespace

bool AxisAlignedBounds::contains(Point point) const {
  return minimum.x <= point.x && point.x <= maximum.x && minimum.y <= point.y &&
         point.y <= maximum.y;
}

bool AxisAlignedBounds::overlaps(const AxisAlignedBounds& other) const {
  return minimum.x <= other.maximum.x && other.minimum.x <= maximum.x &&
         minimum.y <= other.maximum.y && other.minimum.y <= maximum.y;
}

PreparedPolygon prepare_polygon(const Polygon& polygon) {
  PreparedPolygon result;
  result.polygon = polygon;
  if (polygon.vertices.size() < 3U) {
    return result;
  }
  result.bounds = AxisAlignedBounds{polygon.vertices.front(), polygon.vertices.front()};
  std::vector<PreparedBoundarySegment> source;
  source.reserve(polygon.vertices.size());
  for (std::size_t index = 0U; index < polygon.vertices.size(); ++index) {
    const auto start = polygon.vertices[index];
    const auto end = polygon.vertices[next_index(index, polygon.vertices.size())];
    const auto bounds = segment_bounds(start, end);
    result.bounds = merge_bounds(result.bounds, bounds);
    source.push_back(PreparedBoundarySegment{start, end, bounds});
  }
  std::vector<std::size_t> order(source.size());
  std::iota(order.begin(), order.end(), 0U);
  constexpr std::size_t leaf_size = 8U;
  const auto build = [&](auto&& self, std::size_t first, std::size_t last) -> std::size_t {
    auto bounds = source[order[first]].bounds;
    for (auto index = first + 1U; index < last; ++index) {
      bounds = merge_bounds(bounds, source[order[index]].bounds);
    }
    const auto node_index = result.boundary_bvh.size();
    result.boundary_bvh.push_back(PreparedBoundaryNode{bounds});
    if (last - first <= leaf_size) {
      const auto output_first = result.boundary_segments.size();
      for (auto index = first; index < last; ++index) {
        result.boundary_segments.push_back(source[order[index]]);
      }
      result.boundary_bvh[node_index].first_segment = output_first;
      result.boundary_bvh[node_index].segment_count = last - first;
      return node_index;
    }
    const auto split_x = bounds.maximum.x.value() - bounds.minimum.x.value() >=
                         bounds.maximum.y.value() - bounds.minimum.y.value();
    const auto middle = first + (last - first) / 2U;
    std::nth_element(
        order.begin() + static_cast<std::ptrdiff_t>(first),
        order.begin() + static_cast<std::ptrdiff_t>(middle),
        order.begin() + static_cast<std::ptrdiff_t>(last), [&](auto lhs, auto rhs) {
          const auto lhs_center =
              split_x ? source[lhs].bounds.minimum.x.value() + source[lhs].bounds.maximum.x.value()
                      : source[lhs].bounds.minimum.y.value() + source[lhs].bounds.maximum.y.value();
          const auto rhs_center =
              split_x ? source[rhs].bounds.minimum.x.value() + source[rhs].bounds.maximum.x.value()
                      : source[rhs].bounds.minimum.y.value() + source[rhs].bounds.maximum.y.value();
          return lhs_center != rhs_center ? lhs_center < rhs_center : lhs < rhs;
        });
    const auto left = self(self, first, middle);
    const auto right = self(self, middle, last);
    result.boundary_bvh[node_index].left = left;
    result.boundary_bvh[node_index].right = right;
    return node_index;
  };
  static_cast<void>(build(build, 0U, order.size()));
  return result;
}

PointLocation locate_point(const PreparedPolygon& polygon, Point point) {
  if (!polygon.valid() || !coordinate_in_range(point) || !polygon.bounds.contains(point)) {
    return PointLocation::outside;
  }
  bool inside = false;
  for (const auto& segment : polygon.boundary_segments) {
    if (point_on_segment(point, segment.start, segment.end)) {
      return PointLocation::boundary;
    }
    const bool straddles =
        (segment.end.y.value() > point.y.value()) != (segment.start.y.value() > point.y.value());
    if (!straddles) {
      continue;
    }
    const auto intersection_x =
        static_cast<long double>(segment.start.x.value()) +
        static_cast<long double>(segment.end.x.value() - segment.start.x.value()) *
            static_cast<long double>(point.y.value() - segment.start.y.value()) /
            static_cast<long double>(segment.end.y.value() - segment.start.y.value());
    inside = inside != (static_cast<long double>(point.x.value()) < intersection_x);
  }
  return inside ? PointLocation::inside : PointLocation::outside;
}

long double point_polygon_boundary_distance_nm(Point point, const PreparedPolygon& polygon) {
  if (!polygon.valid()) {
    return std::numeric_limits<long double>::infinity();
  }
  auto best = std::numeric_limits<long double>::infinity();
  BoundaryNodeStack stack;
  stack.push(0U);
  while (!stack.empty()) {
    const auto node_index = stack.pop();
    const auto& node = polygon.boundary_bvh[node_index];
    if (point_bounds_distance_nm(point, node.bounds) > best) {
      continue;
    }
    if (node.leaf()) {
      for (std::size_t index = 0U; index < node.segment_count; ++index) {
        const auto& segment = polygon.boundary_segments[node.first_segment + index];
        best = std::min(best, point_segment_distance_nm(point, segment.start, segment.end));
      }
      continue;
    }
    const auto left_distance =
        point_bounds_distance_nm(point, polygon.boundary_bvh[node.left].bounds);
    const auto right_distance =
        point_bounds_distance_nm(point, polygon.boundary_bvh[node.right].bounds);
    if (left_distance < right_distance) {
      stack.push(node.right);
      stack.push(node.left);
    } else {
      stack.push(node.left);
      stack.push(node.right);
    }
  }
  return best;
}

long double segment_polygon_boundary_distance_nm_capped(Point start, Point end,
                                                        const PreparedPolygon& polygon,
                                                        long double maximum_distance_nm) {
  if (!polygon.valid()) {
    return std::numeric_limits<long double>::infinity();
  }
  auto best = std::max(0.0L, maximum_distance_nm);
  BoundaryNodeStack stack;
  stack.push(0U);
  while (!stack.empty()) {
    const auto node_index = stack.pop();
    const auto& node = polygon.boundary_bvh[node_index];
    if (segment_bounds_distance_lower_bound(start, end, node.bounds) > best) {
      continue;
    }
    if (node.leaf()) {
      for (std::size_t index = 0U; index < node.segment_count; ++index) {
        const auto& segment = polygon.boundary_segments[node.first_segment + index];
        best = std::min(best, segment_distance_nm(start, end, segment.start, segment.end));
      }
    } else {
      stack.push(node.right);
      stack.push(node.left);
    }
  }
  return best;
}

long double segment_polygon_boundary_distance_nm(Point start, Point end,
                                                 const PreparedPolygon& polygon) {
  return segment_polygon_boundary_distance_nm_capped(
      start, end, polygon, std::numeric_limits<long double>::infinity());
}

bool polygon_boundaries_intersect(const PreparedPolygon& lhs, const PreparedPolygon& rhs) {
  if (!lhs.valid() || !rhs.valid() || !lhs.bounds.overlaps(rhs.bounds)) {
    return false;
  }
  std::vector<std::pair<std::size_t, std::size_t>> pending;
  pending.reserve(64U);
  pending.emplace_back(0U, 0U);
  while (!pending.empty()) {
    const auto [lhs_index, rhs_index] = pending.back();
    pending.pop_back();
    const auto& lhs_node = lhs.boundary_bvh[lhs_index];
    const auto& rhs_node = rhs.boundary_bvh[rhs_index];
    if (!lhs_node.bounds.overlaps(rhs_node.bounds)) {
      continue;
    }
    if (lhs_node.leaf() && rhs_node.leaf()) {
      for (std::size_t first_index = 0U; first_index < lhs_node.segment_count; ++first_index) {
        const auto& first = lhs.boundary_segments[lhs_node.first_segment + first_index];
        for (std::size_t second_index = 0U; second_index < rhs_node.segment_count;
             ++second_index) {
          const auto& second = rhs.boundary_segments[rhs_node.first_segment + second_index];
          if (first.bounds.overlaps(second.bounds) &&
              segments_intersect(first.start, first.end, second.start, second.end)) {
            return true;
          }
        }
      }
      continue;
    }
    if (rhs_node.leaf() ||
        (!lhs_node.leaf() && lhs_node.bounds.maximum.x.value() -
                                   lhs_node.bounds.minimum.x.value() +
                               lhs_node.bounds.maximum.y.value() -
                                   lhs_node.bounds.minimum.y.value() >=
                           rhs_node.bounds.maximum.x.value() - rhs_node.bounds.minimum.x.value() +
                               rhs_node.bounds.maximum.y.value() -
                                   rhs_node.bounds.minimum.y.value())) {
      pending.emplace_back(lhs_node.left, rhs_index);
      pending.emplace_back(lhs_node.right, rhs_index);
    } else {
      pending.emplace_back(lhs_index, rhs_node.left);
      pending.emplace_back(lhs_index, rhs_node.right);
    }
  }
  return false;
}

bool polygons_overlap(const PreparedPolygon& lhs, const PreparedPolygon& rhs) {
  if (!lhs.valid() || !rhs.valid() || !lhs.bounds.overlaps(rhs.bounds)) {
    return false;
  }
  if (polygon_boundaries_intersect(lhs, rhs)) {
    return true;
  }
  return locate_point(lhs, rhs.polygon.vertices.front()) != PointLocation::outside ||
         locate_point(rhs, lhs.polygon.vertices.front()) != PointLocation::outside;
}

bool coordinate_in_range(Point point) {
  return point.x.value() >= -max_abs_coordinate_nm && point.x.value() <= max_abs_coordinate_nm &&
         point.y.value() >= -max_abs_coordinate_nm && point.y.value() <= max_abs_coordinate_nm;
}

Point transform_point(Point local, Point translation, Microdegrees rotation, BoardSide side) {
  auto local_x = local.x.value();
  const auto local_y = local.y.value();
  if (side == BoardSide::back) {
    local_x = -local_x;
  }

  const auto microdegrees = rotation.value();
  constexpr std::int64_t quarter_turn = 90'000'000;
  std::int64_t rotated_x = 0;
  std::int64_t rotated_y = 0;

  if (microdegrees % quarter_turn == 0) {
    switch ((microdegrees / quarter_turn) % 4) {
    case 0:
      rotated_x = local_x;
      rotated_y = local_y;
      break;
    case 1:
      rotated_x = local_y;
      rotated_y = -local_x;
      break;
    case 2:
      rotated_x = -local_x;
      rotated_y = -local_y;
      break;
    default:
      rotated_x = -local_y;
      rotated_y = local_x;
      break;
    }
  } else {
    constexpr long double pi = 3.141592653589793238462643383279502884L;
    const auto radians =
        static_cast<long double>(microdegrees) * pi / static_cast<long double>(180'000'000);
    const auto cosine = std::cos(radians);
    const auto sine = std::sin(radians);
    // KiCad board coordinates increase downward on Y, so a positive file rotation is clockwise
    // in the Cartesian geometry used by the engine.
    rotated_x = static_cast<std::int64_t>(std::llround(static_cast<long double>(local_x) * cosine +
                                                       static_cast<long double>(local_y) * sine));
    rotated_y = static_cast<std::int64_t>(std::llround(-static_cast<long double>(local_x) * sine +
                                                       static_cast<long double>(local_y) * cosine));
  }

  return Point{Nanometres{translation.x.value() + rotated_x},
               Nanometres{translation.y.value() + rotated_y}};
}

Polygon transform_polygon(const Polygon& polygon, Point translation, Microdegrees rotation,
                          BoardSide side) {
  Polygon result;
  result.vertices.reserve(polygon.vertices.size());
  for (const auto vertex : polygon.vertices) {
    result.vertices.push_back(transform_point(vertex, translation, rotation, side));
  }
  return result;
}

long double signed_double_area(const Polygon& polygon) {
  long double result = 0.0L;
  if (polygon.vertices.size() < 3U) {
    return result;
  }
  for (std::size_t i = 0; i < polygon.vertices.size(); ++i) {
    const auto& current = polygon.vertices[i];
    const auto& next = polygon.vertices[next_index(i, polygon.vertices.size())];
    result +=
        static_cast<long double>(current.x.value()) * static_cast<long double>(next.y.value()) -
        static_cast<long double>(current.y.value()) * static_cast<long double>(next.x.value());
  }
  return result;
}

bool point_on_segment(Point point, Point start, Point end) {
  if (!coordinate_in_range(point) || !coordinate_in_range(start) || !coordinate_in_range(end)) {
    return false;
  }
  return cross(start, end, point) == 0 && boxes_overlap(start, end, point, point);
}

bool segments_intersect(Point a, Point b, Point c, Point d) {
  if (!coordinate_in_range(a) || !coordinate_in_range(b) || !coordinate_in_range(c) ||
      !coordinate_in_range(d) || !boxes_overlap(a, b, c, d)) {
    return false;
  }
  const auto ab_c = orientation(a, b, c);
  const auto ab_d = orientation(a, b, d);
  const auto cd_a = orientation(c, d, a);
  const auto cd_b = orientation(c, d, b);
  if (ab_c != ab_d && cd_a != cd_b) {
    return true;
  }
  return (ab_c == 0 && point_on_segment(c, a, b)) || (ab_d == 0 && point_on_segment(d, a, b)) ||
         (cd_a == 0 && point_on_segment(a, c, d)) || (cd_b == 0 && point_on_segment(b, c, d));
}

bool is_simple_polygon(const Polygon& polygon) {
  const auto size = polygon.vertices.size();
  if (size < 3U || signed_double_area(polygon) == 0.0L) {
    return false;
  }
  for (std::size_t i = 0; i < size; ++i) {
    const auto i_next = next_index(i, size);
    if (polygon.vertices[i] == polygon.vertices[i_next] ||
        !coordinate_in_range(polygon.vertices[i])) {
      return false;
    }
    for (std::size_t j = i + 1U; j < size; ++j) {
      const auto j_next = next_index(j, size);
      if (i == j || i_next == j || j_next == i) {
        continue;
      }
      if (segments_intersect(polygon.vertices[i], polygon.vertices[i_next], polygon.vertices[j],
                             polygon.vertices[j_next])) {
        return false;
      }
    }
  }
  return true;
}

PointLocation locate_point(const Polygon& polygon, Point point) {
  if (polygon.vertices.size() < 3U || !coordinate_in_range(point)) {
    return PointLocation::outside;
  }

  bool inside = false;
  for (std::size_t i = 0, j = polygon.vertices.size() - 1U; i < polygon.vertices.size(); j = i++) {
    const auto current = polygon.vertices[i];
    const auto previous = polygon.vertices[j];
    if (point_on_segment(point, previous, current)) {
      return PointLocation::boundary;
    }

    const bool straddles =
        (current.y.value() > point.y.value()) != (previous.y.value() > point.y.value());
    if (!straddles) {
      continue;
    }
    const auto intersection_x =
        static_cast<long double>(previous.x.value()) +
        static_cast<long double>(current.x.value() - previous.x.value()) *
            static_cast<long double>(point.y.value() - previous.y.value()) /
            static_cast<long double>(current.y.value() - previous.y.value());
    if (static_cast<long double>(point.x.value()) < intersection_x) {
      inside = !inside;
    }
  }
  return inside ? PointLocation::inside : PointLocation::outside;
}

bool polygons_overlap(const Polygon& lhs, const Polygon& rhs) {
  if (lhs.vertices.size() < 3U || rhs.vertices.size() < 3U || !polygon_boxes_overlap(lhs, rhs)) {
    return false;
  }
  for (std::size_t i = 0; i < lhs.vertices.size(); ++i) {
    for (std::size_t j = 0; j < rhs.vertices.size(); ++j) {
      if (segments_intersect(lhs.vertices[i], lhs.vertices[next_index(i, lhs.vertices.size())],
                             rhs.vertices[j], rhs.vertices[next_index(j, rhs.vertices.size())])) {
        return true;
      }
    }
  }
  return locate_point(lhs, rhs.vertices.front()) != PointLocation::outside ||
         locate_point(rhs, lhs.vertices.front()) != PointLocation::outside;
}

bool polygon_inside(const Polygon& inner, const Polygon& outer) {
  if (inner.vertices.size() < 3U || outer.vertices.size() < 3U) {
    return false;
  }
  for (const auto point : inner.vertices) {
    if (locate_point(outer, point) == PointLocation::outside) {
      return false;
    }
  }
  for (std::size_t i = 0; i < inner.vertices.size(); ++i) {
    const auto inner_next = next_index(i, inner.vertices.size());
    for (std::size_t j = 0; j < outer.vertices.size(); ++j) {
      const auto outer_next = next_index(j, outer.vertices.size());
      if (proper_segments_intersect(inner.vertices[i], inner.vertices[inner_next],
                                    outer.vertices[j], outer.vertices[outer_next])) {
        return false;
      }
    }
  }
  return true;
}

std::uint64_t segment_length_nm(Point start, Point end) {
  const auto dx = static_cast<long double>(end.x.value() - start.x.value());
  const auto dy = static_cast<long double>(end.y.value() - start.y.value());
  const auto length = std::hypot(dx, dy);
  if (length >= static_cast<long double>(std::numeric_limits<std::uint64_t>::max())) {
    return std::numeric_limits<std::uint64_t>::max();
  }
  return static_cast<std::uint64_t>(std::llround(length));
}

long double point_distance_nm(Point lhs, Point rhs) {
  const auto dx = static_cast<long double>(lhs.x.value()) - static_cast<long double>(rhs.x.value());
  const auto dy = static_cast<long double>(lhs.y.value()) - static_cast<long double>(rhs.y.value());
  return std::hypot(dx, dy);
}

long double point_segment_distance_nm(Point point, Point start, Point end) {
  const auto segment_x =
      static_cast<long double>(end.x.value()) - static_cast<long double>(start.x.value());
  const auto segment_y =
      static_cast<long double>(end.y.value()) - static_cast<long double>(start.y.value());
  const auto length_squared = segment_x * segment_x + segment_y * segment_y;
  if (length_squared == 0.0L) {
    return point_distance_nm(point, start);
  }
  const auto point_x =
      static_cast<long double>(point.x.value()) - static_cast<long double>(start.x.value());
  const auto point_y =
      static_cast<long double>(point.y.value()) - static_cast<long double>(start.y.value());
  const auto projection =
      std::clamp((point_x * segment_x + point_y * segment_y) / length_squared, 0.0L, 1.0L);
  const auto closest_x = static_cast<long double>(start.x.value()) + projection * segment_x;
  const auto closest_y = static_cast<long double>(start.y.value()) + projection * segment_y;
  return std::hypot(static_cast<long double>(point.x.value()) - closest_x,
                    static_cast<long double>(point.y.value()) - closest_y);
}

long double segment_distance_nm(Point lhs_start, Point lhs_end, Point rhs_start, Point rhs_end) {
  if (segments_intersect(lhs_start, lhs_end, rhs_start, rhs_end)) {
    return 0.0L;
  }
  return std::min({point_segment_distance_nm(lhs_start, rhs_start, rhs_end),
                   point_segment_distance_nm(lhs_end, rhs_start, rhs_end),
                   point_segment_distance_nm(rhs_start, lhs_start, lhs_end),
                   point_segment_distance_nm(rhs_end, lhs_start, lhs_end)});
}

std::optional<CircularArcParameters> circular_arc_parameters(Point start, Point middle, Point end) {
  if (!coordinate_in_range(start) || !coordinate_in_range(middle) || !coordinate_in_range(end) ||
      start == middle || middle == end || start == end || cross(start, middle, end) == 0) {
    return std::nullopt;
  }
  const auto ax = static_cast<long double>(start.x.value());
  const auto ay = static_cast<long double>(start.y.value());
  const auto bx = static_cast<long double>(middle.x.value());
  const auto by = static_cast<long double>(middle.y.value());
  const auto cx = static_cast<long double>(end.x.value());
  const auto cy = static_cast<long double>(end.y.value());
  const auto denominator = 2.0L * (ax * (by - cy) + bx * (cy - ay) + cx * (ay - by));
  if (denominator == 0.0L) {
    return std::nullopt;
  }
  const auto a_norm = squared(ax) + squared(ay);
  const auto b_norm = squared(bx) + squared(by);
  const auto c_norm = squared(cx) + squared(cy);
  const auto center_x =
      (a_norm * (by - cy) + b_norm * (cy - ay) + c_norm * (ay - by)) / denominator;
  const auto center_y =
      (a_norm * (cx - bx) + b_norm * (ax - cx) + c_norm * (bx - ax)) / denominator;
  const auto radius = std::hypot(ax - center_x, ay - center_y);
  if (!std::isfinite(center_x) || !std::isfinite(center_y) || !std::isfinite(radius) ||
      radius <= 0.0L) {
    return std::nullopt;
  }
  const auto start_angle = std::atan2(ay - center_y, ax - center_x);
  const auto middle_angle = std::atan2(by - center_y, bx - center_x);
  const auto end_angle = std::atan2(cy - center_y, cx - center_x);
  const auto ccw_sweep = positive_angle_delta(start_angle, end_angle);
  const auto ccw_to_middle = positive_angle_delta(start_angle, middle_angle);
  const auto sweep = ccw_to_middle <= ccw_sweep + 1.0e-15L ? ccw_sweep : ccw_sweep - tau;
  if (std::abs(sweep) <= 1.0e-15L || std::abs(sweep) >= tau - 1.0e-15L) {
    return std::nullopt;
  }
  return CircularArcParameters{center_x, center_y, radius, start_angle, sweep};
}

bool point_on_circular_arc(Point point, Point start, Point middle, Point end) {
  const auto arc = circular_arc_parameters(start, middle, end);
  if (!arc || !coordinate_in_range(point)) {
    return false;
  }
  const auto dx = static_cast<long double>(point.x.value()) - arc->center_x_nm;
  const auto dy = static_cast<long double>(point.y.value()) - arc->center_y_nm;
  return std::abs(std::hypot(dx, dy) - arc->radius_nm) <= 1.0e-6L &&
         angle_on_arc(*arc, std::atan2(dy, dx));
}

std::uint64_t circular_arc_length_nm(Point start, Point middle, Point end) {
  const auto arc = circular_arc_parameters(start, middle, end);
  if (!arc) {
    return 0U;
  }
  const auto length = arc->radius_nm * std::abs(arc->sweep_radians);
  if (length >= static_cast<long double>(std::numeric_limits<std::uint64_t>::max())) {
    return std::numeric_limits<std::uint64_t>::max();
  }
  return static_cast<std::uint64_t>(std::llround(length));
}

std::vector<Point> circular_arc_extrema(Point start, Point middle, Point end) {
  std::vector<Point> result{start, middle, end};
  const auto arc = circular_arc_parameters(start, middle, end);
  if (!arc) {
    return result;
  }
  for (const auto angle : {0.0L, pi / 2.0L, pi, 3.0L * pi / 2.0L}) {
    if (angle_on_arc(*arc, angle)) {
      result.push_back(quantized_point(arc->center_x_nm + arc->radius_nm * std::cos(angle),
                                       arc->center_y_nm + arc->radius_nm * std::sin(angle)));
    }
  }
  std::sort(result.begin(), result.end());
  result.erase(std::unique(result.begin(), result.end()), result.end());
  return result;
}

long double point_circular_arc_distance_nm(Point point, Point start, Point middle, Point end) {
  const auto arc = circular_arc_parameters(start, middle, end);
  if (!arc) {
    return std::min(point_distance_nm(point, start), point_distance_nm(point, end));
  }
  auto result = std::min(point_distance_nm(point, start), point_distance_nm(point, end));
  const auto dx = static_cast<long double>(point.x.value()) - arc->center_x_nm;
  const auto dy = static_cast<long double>(point.y.value()) - arc->center_y_nm;
  if ((dx != 0.0L || dy != 0.0L) && angle_on_arc(*arc, std::atan2(dy, dx))) {
    result = std::min(result, std::abs(std::hypot(dx, dy) - arc->radius_nm));
  }
  return result;
}

long double segment_circular_arc_distance_nm(Point segment_start, Point segment_end,
                                             Point arc_start, Point arc_middle, Point arc_end) {
  const auto arc = circular_arc_parameters(arc_start, arc_middle, arc_end);
  if (!arc) {
    return segment_distance_nm(segment_start, segment_end, arc_start, arc_end);
  }
  auto result =
      std::min({point_circular_arc_distance_nm(segment_start, arc_start, arc_middle, arc_end),
                point_circular_arc_distance_nm(segment_end, arc_start, arc_middle, arc_end),
                point_segment_distance_nm(arc_start, segment_start, segment_end),
                point_segment_distance_nm(arc_end, segment_start, segment_end)});
  const auto sx = static_cast<long double>(segment_start.x.value()) - arc->center_x_nm;
  const auto sy = static_cast<long double>(segment_start.y.value()) - arc->center_y_nm;
  const auto dx = static_cast<long double>(segment_end.x.value() - segment_start.x.value());
  const auto dy = static_cast<long double>(segment_end.y.value() - segment_start.y.value());
  const auto a = dx * dx + dy * dy;
  if (a == 0.0L) {
    return result;
  }
  const auto b = 2.0L * (sx * dx + sy * dy);
  const auto c = sx * sx + sy * sy - arc->radius_nm * arc->radius_nm;
  auto discriminant = b * b - 4.0L * a * c;
  const auto tolerance = 1.0e-18L * std::max({1.0L, std::abs(b * b), std::abs(4.0L * a * c)});
  if (discriminant >= -tolerance) {
    discriminant = std::max(0.0L, discriminant);
    const auto root = std::sqrt(discriminant);
    for (const auto parameter : {(-b - root) / (2.0L * a), (-b + root) / (2.0L * a)}) {
      if (parameter < 0.0L || parameter > 1.0L) {
        continue;
      }
      const auto x = sx + parameter * dx;
      const auto y = sy + parameter * dy;
      if (angle_on_arc(*arc, std::atan2(y, x))) {
        return 0.0L;
      }
    }
  }
  const auto projection = std::clamp(-(sx * dx + sy * dy) / a, 0.0L, 1.0L);
  const auto closest_x = sx + projection * dx;
  const auto closest_y = sy + projection * dy;
  if ((closest_x != 0.0L || closest_y != 0.0L) &&
      angle_on_arc(*arc, std::atan2(closest_y, closest_x))) {
    result = std::min(result, std::abs(std::hypot(closest_x, closest_y) - arc->radius_nm));
  }
  return result;
}

long double circular_arcs_distance_nm(Point lhs_start, Point lhs_middle, Point lhs_end,
                                      Point rhs_start, Point rhs_middle, Point rhs_end) {
  const auto lhs = circular_arc_parameters(lhs_start, lhs_middle, lhs_end);
  const auto rhs = circular_arc_parameters(rhs_start, rhs_middle, rhs_end);
  if (!lhs || !rhs) {
    return segment_distance_nm(lhs_start, lhs_end, rhs_start, rhs_end);
  }
  auto result = std::min({point_circular_arc_distance_nm(lhs_start, rhs_start, rhs_middle, rhs_end),
                          point_circular_arc_distance_nm(lhs_end, rhs_start, rhs_middle, rhs_end),
                          point_circular_arc_distance_nm(rhs_start, lhs_start, lhs_middle, lhs_end),
                          point_circular_arc_distance_nm(rhs_end, lhs_start, lhs_middle, lhs_end)});
  const auto center_dx = rhs->center_x_nm - lhs->center_x_nm;
  const auto center_dy = rhs->center_y_nm - lhs->center_y_nm;
  const auto center_distance = std::hypot(center_dx, center_dy);
  if (center_distance > 0.0L && center_distance <= lhs->radius_nm + rhs->radius_nm + 1.0e-6L &&
      center_distance + std::min(lhs->radius_nm, rhs->radius_nm) + 1.0e-6L >=
          std::max(lhs->radius_nm, rhs->radius_nm)) {
    const auto along =
        (squared(lhs->radius_nm) - squared(rhs->radius_nm) + squared(center_distance)) /
        (2.0L * center_distance);
    const auto height_squared = squared(lhs->radius_nm) - squared(along);
    if (height_squared >= -1.0e-6L) {
      const auto height = std::sqrt(std::max(0.0L, height_squared));
      const auto unit_x = center_dx / center_distance;
      const auto unit_y = center_dy / center_distance;
      const auto base_x = lhs->center_x_nm + along * unit_x;
      const auto base_y = lhs->center_y_nm + along * unit_y;
      for (const auto sign : {-1.0L, 1.0L}) {
        const auto x = base_x + sign * height * -unit_y;
        const auto y = base_y + sign * height * unit_x;
        if (angle_on_arc(*lhs, std::atan2(y - lhs->center_y_nm, x - lhs->center_x_nm)) &&
            angle_on_arc(*rhs, std::atan2(y - rhs->center_y_nm, x - rhs->center_x_nm))) {
          return 0.0L;
        }
      }
    }
  }
  if (center_distance > 0.0L) {
    const auto direction = std::atan2(center_dy, center_dx);
    for (const auto angle : {direction, normalized_angle(direction + pi)}) {
      if (angle_on_arc(*lhs, angle)) {
        const auto point = quantized_point(lhs->center_x_nm + lhs->radius_nm * std::cos(angle),
                                           lhs->center_y_nm + lhs->radius_nm * std::sin(angle));
        result =
            std::min(result, point_circular_arc_distance_nm(point, rhs_start, rhs_middle, rhs_end));
      }
      if (angle_on_arc(*rhs, angle)) {
        const auto point = quantized_point(rhs->center_x_nm + rhs->radius_nm * std::cos(angle),
                                           rhs->center_y_nm + rhs->radius_nm * std::sin(angle));
        result =
            std::min(result, point_circular_arc_distance_nm(point, lhs_start, lhs_middle, lhs_end));
      }
    }
  }
  // The radial closest-point probes above are quantized back into the canonical
  // integer-nanometre coordinate domain. Account for the worst-case error of
  // both coordinates so a clearance check cannot accept a sub-nanometre
  // overlap because that probe rounded away from the other arc.
  constexpr long double quantization_guard_nm = 2.0L;
  return std::max(0.0L, result - quantization_guard_nm);
}

long double point_polygon_boundary_distance_nm(Point point, const Polygon& polygon) {
  if (polygon.vertices.size() < 2U) {
    return std::numeric_limits<long double>::infinity();
  }
  auto distance = std::numeric_limits<long double>::infinity();
  for (std::size_t i = 0; i < polygon.vertices.size(); ++i) {
    distance = std::min(distance, point_segment_distance_nm(
                                      point, polygon.vertices[i],
                                      polygon.vertices[next_index(i, polygon.vertices.size())]));
  }
  return distance;
}

long double segment_polygon_boundary_distance_nm(Point start, Point end, const Polygon& polygon) {
  if (polygon.vertices.size() < 2U) {
    return std::numeric_limits<long double>::infinity();
  }
  auto distance = std::numeric_limits<long double>::infinity();
  for (std::size_t i = 0; i < polygon.vertices.size(); ++i) {
    distance = std::min(
        distance, segment_distance_nm(start, end, polygon.vertices[i],
                                      polygon.vertices[next_index(i, polygon.vertices.size())]));
  }
  return distance;
}

long double circular_arc_polygon_boundary_distance_nm(Point start, Point middle, Point end,
                                                      const Polygon& polygon) {
  if (polygon.vertices.size() < 2U) {
    return std::numeric_limits<long double>::infinity();
  }
  auto distance = std::numeric_limits<long double>::infinity();
  for (std::size_t i = 0; i < polygon.vertices.size(); ++i) {
    distance = std::min(distance, segment_circular_arc_distance_nm(
                                      polygon.vertices[i],
                                      polygon.vertices[next_index(i, polygon.vertices.size())],
                                      start, middle, end));
  }
  return distance;
}

long double point_polygon_copper_distance_nm(Point point, const Polygon& polygon) {
  return locate_point(polygon, point) == PointLocation::outside
             ? point_polygon_boundary_distance_nm(point, polygon)
             : 0.0L;
}

long double segment_polygon_copper_distance_nm(Point start, Point end, const Polygon& polygon) {
  if (locate_point(polygon, start) != PointLocation::outside ||
      locate_point(polygon, end) != PointLocation::outside) {
    return 0.0L;
  }
  return segment_polygon_boundary_distance_nm(start, end, polygon);
}

long double circular_arc_polygon_copper_distance_nm(Point start, Point middle, Point end,
                                                    const Polygon& polygon) {
  if (locate_point(polygon, start) != PointLocation::outside ||
      locate_point(polygon, middle) != PointLocation::outside ||
      locate_point(polygon, end) != PointLocation::outside) {
    return 0.0L;
  }
  return circular_arc_polygon_boundary_distance_nm(start, middle, end, polygon);
}

long double polygon_boundary_distance_nm(const Polygon& lhs, const Polygon& rhs) {
  auto distance = std::numeric_limits<long double>::infinity();
  for (std::size_t index = 0; index < lhs.vertices.size(); ++index) {
    distance = std::min(distance, segment_polygon_boundary_distance_nm(
                                      lhs.vertices[index],
                                      lhs.vertices[next_index(index, lhs.vertices.size())], rhs));
  }
  return distance;
}

long double polygon_copper_distance_nm(const Polygon& lhs, const Polygon& rhs) {
  return polygons_overlap(lhs, rhs) ? 0.0L : polygon_boundary_distance_nm(lhs, rhs);
}

bool segment_inside_polygon(Point start, Point end, const Polygon& polygon) {
  if (locate_point(polygon, start) == PointLocation::outside ||
      locate_point(polygon, end) == PointLocation::outside) {
    return false;
  }
  for (std::size_t i = 0; i < polygon.vertices.size(); ++i) {
    if (proper_segments_intersect(start, end, polygon.vertices[i],
                                  polygon.vertices[next_index(i, polygon.vertices.size())])) {
      return false;
    }
  }
  return true;
}

bool circular_arc_inside_polygon(Point start, Point middle, Point end, const Polygon& polygon) {
  if (!circular_arc_parameters(start, middle, end)) {
    return false;
  }
  const auto critical = circular_arc_extrema(start, middle, end);
  if (std::any_of(critical.begin(), critical.end(), [&polygon](Point point) {
        return locate_point(polygon, point) == PointLocation::outside;
      })) {
    return false;
  }
  // Any unrepresented boundary contact may be a crossing through a concavity. Fail closed.
  return circular_arc_polygon_boundary_distance_nm(start, middle, end, polygon) > 1.0e-6L;
}

PointLocation locate_point_in_board_area(const Board& board, Point point) {
  const auto outer = locate_point(board.outline, point);
  if (outer != PointLocation::inside) {
    return outer;
  }
  for (const auto& cutout : board.cutouts) {
    const auto location = locate_point(cutout, point);
    if (location == PointLocation::boundary) {
      return PointLocation::boundary;
    }
    if (location == PointLocation::inside) {
      return PointLocation::outside;
    }
  }
  return PointLocation::inside;
}

long double point_board_boundary_distance_nm(Point point, const Board& board) {
  auto distance = point_polygon_boundary_distance_nm(point, board.outline);
  for (const auto& cutout : board.cutouts) {
    distance = std::min(distance, point_polygon_boundary_distance_nm(point, cutout));
  }
  return distance;
}

long double segment_board_boundary_distance_nm(Point start, Point end, const Board& board) {
  auto distance = segment_polygon_boundary_distance_nm(start, end, board.outline);
  for (const auto& cutout : board.cutouts) {
    distance = std::min(distance, segment_polygon_boundary_distance_nm(start, end, cutout));
  }
  return distance;
}

long double circular_arc_board_boundary_distance_nm(Point start, Point middle, Point end,
                                                    const Board& board) {
  auto distance = circular_arc_polygon_boundary_distance_nm(start, middle, end, board.outline);
  for (const auto& cutout : board.cutouts) {
    distance =
        std::min(distance, circular_arc_polygon_boundary_distance_nm(start, middle, end, cutout));
  }
  return distance;
}

long double polygon_board_boundary_distance_nm(const Polygon& polygon, const Board& board) {
  auto distance = polygon_boundary_distance_nm(polygon, board.outline);
  for (const auto& cutout : board.cutouts) {
    distance = std::min(distance, polygon_boundary_distance_nm(polygon, cutout));
  }
  return distance;
}

bool segment_inside_board_area(Point start, Point end, const Board& board) {
  if (!segment_inside_polygon(start, end, board.outline)) {
    return false;
  }
  return std::none_of(board.cutouts.begin(), board.cutouts.end(), [start, end](const auto& cutout) {
    return segment_polygon_copper_distance_nm(start, end, cutout) <= 0.0L;
  });
}

bool circular_arc_inside_board_area(Point start, Point middle, Point end, const Board& board) {
  if (!circular_arc_inside_polygon(start, middle, end, board.outline)) {
    return false;
  }
  return std::none_of(
      board.cutouts.begin(), board.cutouts.end(), [start, middle, end](const auto& cutout) {
        return circular_arc_polygon_copper_distance_nm(start, middle, end, cutout) <= 0.0L;
      });
}

bool polygon_inside_board_area(const Polygon& polygon, const Board& board) {
  return polygon_inside(polygon, board.outline) &&
         std::none_of(board.cutouts.begin(), board.cutouts.end(),
                      [&polygon](const auto& cutout) { return polygons_overlap(polygon, cutout); });
}

PointLocation locate_point_in_filled_region(const CopperZoneFill& region, Point point) {
  const auto outer_location = locate_point(region.area, point);
  if (outer_location != PointLocation::inside) {
    return outer_location;
  }
  for (const auto& hole : region.holes) {
    const auto hole_location = locate_point(hole, point);
    if (hole_location == PointLocation::boundary) {
      return PointLocation::boundary;
    }
    if (hole_location == PointLocation::inside) {
      return PointLocation::outside;
    }
  }
  return PointLocation::inside;
}

long double point_filled_region_boundary_distance_nm(Point point, const CopperZoneFill& region) {
  auto distance = point_polygon_boundary_distance_nm(point, region.area);
  for (const auto& hole : region.holes) {
    distance = std::min(distance, point_polygon_boundary_distance_nm(point, hole));
  }
  return distance;
}

long double segment_filled_region_boundary_distance_nm(Point start, Point end,
                                                       const CopperZoneFill& region) {
  auto distance = segment_polygon_boundary_distance_nm(start, end, region.area);
  for (const auto& hole : region.holes) {
    distance = std::min(distance, segment_polygon_boundary_distance_nm(start, end, hole));
  }
  return distance;
}

long double point_filled_region_copper_distance_nm(Point point, const CopperZoneFill& region) {
  return locate_point_in_filled_region(region, point) == PointLocation::outside
             ? point_filled_region_boundary_distance_nm(point, region)
             : 0.0L;
}

long double segment_filled_region_copper_distance_nm(Point start, Point end,
                                                     const CopperZoneFill& region) {
  if (locate_point_in_filled_region(region, start) != PointLocation::outside ||
      locate_point_in_filled_region(region, end) != PointLocation::outside) {
    return 0.0L;
  }
  return segment_filled_region_boundary_distance_nm(start, end, region);
}

long double circular_arc_filled_region_copper_distance_nm(Point start, Point middle, Point end,
                                                          const CopperZoneFill& region) {
  if (locate_point_in_filled_region(region, start) != PointLocation::outside ||
      locate_point_in_filled_region(region, middle) != PointLocation::outside ||
      locate_point_in_filled_region(region, end) != PointLocation::outside) {
    return 0.0L;
  }
  auto distance = circular_arc_polygon_boundary_distance_nm(start, middle, end, region.area);
  for (const auto& hole : region.holes) {
    distance =
        std::min(distance, circular_arc_polygon_boundary_distance_nm(start, middle, end, hole));
  }
  return distance;
}

namespace {

template <typename Visitor>
void visit_region_contours(const CopperZoneFill& region, Visitor&& visitor) {
  visitor(region.area);
  for (const auto& hole : region.holes) {
    visitor(hole);
  }
}

[[nodiscard]] bool contour_boundaries_intersect(const Polygon& lhs, const Polygon& rhs) {
  for (std::size_t i = 0; i < lhs.vertices.size(); ++i) {
    for (std::size_t j = 0; j < rhs.vertices.size(); ++j) {
      if (segments_intersect(lhs.vertices[i], lhs.vertices[next_index(i, lhs.vertices.size())],
                             rhs.vertices[j], rhs.vertices[next_index(j, rhs.vertices.size())])) {
        return true;
      }
    }
  }
  return false;
}

} // namespace

bool filled_region_overlaps_polygon(const CopperZoneFill& region, const Polygon& polygon) {
  if (region.area.vertices.size() < 3U || polygon.vertices.size() < 3U) {
    return false;
  }
  bool boundaries_intersect = false;
  visit_region_contours(region, [&polygon, &boundaries_intersect](const Polygon& contour) {
    boundaries_intersect = boundaries_intersect || contour_boundaries_intersect(contour, polygon);
  });
  if (boundaries_intersect) {
    return true;
  }
  if (std::any_of(polygon.vertices.begin(), polygon.vertices.end(), [&region](Point point) {
        return locate_point_in_filled_region(region, point) != PointLocation::outside;
      })) {
    return true;
  }
  return std::any_of(
      region.area.vertices.begin(), region.area.vertices.end(),
      [&polygon](Point point) { return locate_point(polygon, point) != PointLocation::outside; });
}

bool filled_regions_overlap(const CopperZoneFill& lhs, const CopperZoneFill& rhs) {
  if (lhs.area.vertices.size() < 3U || rhs.area.vertices.size() < 3U) {
    return false;
  }
  bool boundaries_intersect = false;
  visit_region_contours(lhs, [&rhs, &boundaries_intersect](const Polygon& lhs_contour) {
    visit_region_contours(rhs, [&lhs_contour, &boundaries_intersect](const Polygon& rhs_contour) {
      boundaries_intersect =
          boundaries_intersect || contour_boundaries_intersect(lhs_contour, rhs_contour);
    });
  });
  if (boundaries_intersect) {
    return true;
  }
  if (std::any_of(lhs.area.vertices.begin(), lhs.area.vertices.end(), [&rhs](Point point) {
        return locate_point_in_filled_region(rhs, point) != PointLocation::outside;
      })) {
    return true;
  }
  return std::any_of(rhs.area.vertices.begin(), rhs.area.vertices.end(), [&lhs](Point point) {
    return locate_point_in_filled_region(lhs, point) != PointLocation::outside;
  });
}

bool filled_region_inside_board_area(const CopperZoneFill& region, const Board& board) {
  return polygon_inside(region.area, board.outline) &&
         std::none_of(board.cutouts.begin(), board.cutouts.end(), [&region](const auto& cutout) {
           return filled_region_overlaps_polygon(region, cutout);
         });
}

long double filled_region_board_boundary_distance_nm(const CopperZoneFill& region,
                                                     const Board& board) {
  auto distance = polygon_boundary_distance_nm(region.area, board.outline);
  for (const auto& cutout : board.cutouts) {
    distance = std::min(distance, filled_region_polygon_copper_distance_nm(region, cutout));
  }
  return distance;
}

long double filled_region_polygon_copper_distance_nm(const CopperZoneFill& region,
                                                     const Polygon& polygon) {
  if (filled_region_overlaps_polygon(region, polygon)) {
    return 0.0L;
  }
  auto distance = polygon_boundary_distance_nm(region.area, polygon);
  for (const auto& hole : region.holes) {
    distance = std::min(distance, polygon_boundary_distance_nm(hole, polygon));
  }
  return distance;
}

long double filled_regions_copper_distance_nm(const CopperZoneFill& lhs,
                                              const CopperZoneFill& rhs) {
  if (filled_regions_overlap(lhs, rhs)) {
    return 0.0L;
  }
  auto distance = std::numeric_limits<long double>::infinity();
  visit_region_contours(lhs, [&rhs, &distance](const Polygon& lhs_contour) {
    visit_region_contours(rhs, [&lhs_contour, &distance](const Polygon& rhs_contour) {
      distance = std::min(distance, polygon_boundary_distance_nm(lhs_contour, rhs_contour));
    });
  });
  return distance;
}

} // namespace flux::geometry
