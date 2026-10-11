#include "flux/core/bezier_copper.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <set>
#include <string>

namespace flux {
namespace {

struct RealPoint { long double x; long double y; };
struct Cubic { RealPoint p0, p1, p2, p3; };

[[nodiscard]] RealPoint real(Point point) {
  return {static_cast<long double>(point.x.value()), static_cast<long double>(point.y.value())};
}
[[nodiscard]] RealPoint midpoint(RealPoint a, RealPoint b) {
  return {(a.x + b.x) / 2.0L, (a.y + b.y) / 2.0L};
}
[[nodiscard]] long double point_segment_distance(RealPoint p, RealPoint a, RealPoint b) {
  const auto dx = b.x - a.x;
  const auto dy = b.y - a.y;
  const auto length_squared = dx * dx + dy * dy;
  if (length_squared == 0.0L) return std::hypotl(p.x - a.x, p.y - a.y);
  const auto t = std::clamp(((p.x - a.x) * dx + (p.y - a.y) * dy) / length_squared, 0.0L, 1.0L);
  return std::hypotl(p.x - (a.x + t * dx), p.y - (a.y + t * dy));
}
[[nodiscard]] long double deviation_bound(const Cubic& cubic) {
  // Distance to a convex set is convex.  The cubic lies in its controls' convex hull, so the
  // largest distance to the chord is bounded by the largest control-point distance.
  return std::max(point_segment_distance(cubic.p1, cubic.p0, cubic.p3),
                  point_segment_distance(cubic.p2, cubic.p0, cubic.p3));
}
[[nodiscard]] Point quantize(RealPoint point) {
  const auto limit = static_cast<long double>(std::numeric_limits<std::int64_t>::max());
  const auto x = std::clamp(point.x, -limit, limit);
  const auto y = std::clamp(point.y, -limit, limit);
  return {Nanometres{static_cast<std::int64_t>(std::llround(x))},
          Nanometres{static_cast<std::int64_t>(std::llround(y))}};
}
[[nodiscard]] Nanometres ceil_nm(long double value) {
  if (!std::isfinite(value) || value >= static_cast<long double>(std::numeric_limits<std::int64_t>::max()))
    return Nanometres{std::numeric_limits<std::int64_t>::max()};
  return Nanometres{static_cast<std::int64_t>(std::ceil(std::max(0.0L, value)))};
}
void split(const Cubic& c, Cubic& left, Cubic& right) {
  const auto p01 = midpoint(c.p0, c.p1); const auto p12 = midpoint(c.p1, c.p2);
  const auto p23 = midpoint(c.p2, c.p3); const auto p012 = midpoint(p01, p12);
  const auto p123 = midpoint(p12, p23); const auto p = midpoint(p012, p123);
  left = {c.p0, p01, p012, p}; right = {p, p123, p23, c.p3};
}
}

BezierFlattening tessellate_track_bezier(const TrackBezier& track, Nanometres tolerance,
                                         std::size_t maximum_depth) {
  BezierFlattening result;
  if (tolerance.value() < 0 || maximum_depth == 0U) return result;
  const Cubic root{real(track.start), real(track.control1), real(track.control2), real(track.end)};
  const auto emit = [&result](const Cubic& cubic, long double deviation) {
    const auto distance = [](RealPoint a, RealPoint b) { return std::hypotl(a.x-b.x, a.y-b.y); };
    result.length_lower_nm += distance(cubic.p0, cubic.p3);
    result.length_upper_nm += distance(cubic.p0, cubic.p1) + distance(cubic.p1, cubic.p2) +
                              distance(cubic.p2, cubic.p3);
    const auto start = quantize(cubic.p0); const auto end = quantize(cubic.p3);
    const auto quantization = std::max(std::hypotl(cubic.p0.x - start.x.value(), cubic.p0.y - start.y.value()),
                                       std::hypotl(cubic.p3.x - end.x.value(), cubic.p3.y - end.y.value()));
    const auto certified = ceil_nm(deviation);
    result.maximum_deviation_nm = std::max(result.maximum_deviation_nm, certified + ceil_nm(quantization));
    result.segments.push_back({start, end, certified, ceil_nm(quantization)});
  };
  const auto visit = [&](auto&& self, const Cubic& cubic, std::size_t depth) -> bool {
    // Bound work even for hostile coordinates or sub-nanometre requested accuracy.
    if (result.segments.size() >= 65'536U) return false;
    const auto deviation = deviation_bound(cubic);
    if (deviation <= static_cast<long double>(tolerance.value())) { emit(cubic, deviation); return true; }
    if (depth >= maximum_depth) return false;
    Cubic left{}, right{}; split(cubic, left, right);
    return self(self, left, depth + 1U) && self(self, right, depth + 1U);
  };
  result.certified = visit(visit, root, 0U);
  if (!result.certified) result.segments.clear();
  return result;
}

ConservativeBezierBoard flatten_board_beziers_conservatively(const Board& board, Nanometres tolerance,
                                                              std::size_t maximum_depth) {
  ConservativeBezierBoard result; result.board = board; result.board.track_beziers.clear(); result.certified = true;
  std::set<std::string> used;
  for (const auto& item : result.board.tracks) used.insert(item.id.value());
  for (const auto& item : result.board.track_arcs) used.insert(item.id.value());
  for (const auto& item : board.track_beziers) used.insert(item.id.value());
  for (const auto& item : board.vias) used.insert(item.id.value());
  for (const auto& item : board.zones) used.insert(item.id.value());
  for (const auto& item : board.keepouts) used.insert(item.id.value());
  for (const auto& footprint : board.footprints) {
    used.insert(footprint.id.value());
    for (const auto& pad : footprint.pads) used.insert(pad.id.value());
  }
  for (const auto& curve : board.track_beziers) {
    const auto flattening = tessellate_track_bezier(curve, tolerance, maximum_depth);
    result.certified = result.certified && flattening.certified;
    if (!flattening.certified) continue;
    for (std::size_t index = 0; index < flattening.segments.size(); ++index) {
      const auto& segment = flattening.segments[index];
      const auto guard = segment.centerline_deviation_nm.value() + segment.quantization_error_nm.value();
      if (guard > (std::numeric_limits<std::int64_t>::max() - curve.width.value()) / 2) { result.certified = false; continue; }
      auto id = std::string{"__flux_bezier_proxy/"} + curve.id.value() + "/" + std::to_string(index);
      while (!used.insert(id).second) id += "_";
      result.board.tracks.push_back({ObjectId{id}, curve.net, curve.layer, segment.start, segment.end,
                                     Nanometres{curve.width.value() + 2 * guard}, curve.locked});
      result.proxy_sources.emplace(id, curve.id);
    }
  }
  return result;
}

} // namespace flux
