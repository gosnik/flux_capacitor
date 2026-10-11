#include "flux/core/spline_route.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <utility>

namespace flux {
namespace {
SplineVector lerp(SplineVector a, SplineVector b, long double t) {
  return {a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t};
}
long double distance(SplineVector a, SplineVector b) {
  return std::hypotl(a.x - b.x, a.y - b.y);
}
Point quantize(SplineVector p) {
  const auto q = [](long double v) {
    return Nanometres{static_cast<std::int64_t>(std::llround(v))};
  };
  return {q(p.x), q(p.y)};
}
SplineVector unquantize(Point p) {
  return {static_cast<long double>(p.x.value()), static_cast<long double>(p.y.value())};
}
std::pair<std::array<SplineVector, 6U>, std::array<SplineVector, 6U>> split(
    const std::array<SplineVector, 6U>& c) {
  std::array<std::array<SplineVector, 6U>, 6U> tri{};
  tri[0] = c;
  for (std::size_t level = 1; level < 6; ++level)
    for (std::size_t i = 0; i + level < 6; ++i)
      tri[level][i] = lerp(tri[level - 1][i], tri[level - 1][i + 1], 0.5L);
  std::array<SplineVector, 6U> left{}, right{};
  for (std::size_t i = 0; i < 6; ++i) {
    left[i] = tri[i][0];
    right[5 - i] = tri[i][5 - i];
  }
  return {left, right};
}
long double error(const std::array<SplineVector, 6U>& q, const std::array<SplineVector, 6U>& c) {
  long double e = 0.0L;
  for (std::size_t i = 0; i < 6; ++i)
    e = std::max(e, distance(q[i], c[i]));
  return e;
}
std::array<SplineVector, 6U> elevated(const std::array<SplineVector, 4U>& c) {
  const std::array<SplineVector, 5U> q{
      c[0], lerp(c[0], c[1], 0.75L), lerp(c[1], c[2], 0.5L), lerp(c[2], c[3], 0.25L), c[3]};
  return {c[0], lerp(q[0], q[1], 0.8L), lerp(q[1], q[2], 0.6L),
          lerp(q[2], q[3], 0.4L), lerp(q[3], q[4], 0.2L), c[3]};
}
bool fit(const std::array<SplineVector, 6U>& q, long double tolerance, std::size_t depth,
         const NativeSplineFitConfig& cfg, std::stop_token stop, long double first,
         long double last, std::vector<SplineFittedPiece>& out) {
  if (stop.stop_requested() || out.size() >= cfg.maximum_pieces)
    return false;
  constexpr auto safe_limit = static_cast<long double>(std::numeric_limits<std::int64_t>::max()) - 4096.0L;
  const auto safe = [&](SplineVector p) { return std::isfinite(p.x) && std::isfinite(p.y) &&
      std::abs(p.x) < safe_limit && std::abs(p.y) < safe_limit; };
  for (const auto p : q) if (!safe(p)) return false;
  const auto h1 = lerp(q[0], q[1], 5.0L / 3.0L);
  const auto h2 = lerp(q[5], q[4], 5.0L / 3.0L);
  if (safe(h1) && safe(h2)) {
    const auto p0 = quantize(q.front());
    const auto p3 = quantize(q.back());
    if (p0 == p3)
      return false;
    const auto c1 = quantize(h1);
    const auto c2 = quantize(h2);
    const std::array<SplineVector, 4U> c{unquantize(p0), unquantize(c1), unquantize(c2),
                                         unquantize(p3)};
    const auto controls = elevated(c);
    const auto forward = [&](Point a, Point b) {
      const auto dx = static_cast<long double>(p3.x.value()) - p0.x.value();
      const auto dy = static_cast<long double>(p3.y.value()) - p0.y.value();
      return (static_cast<long double>(b.x.value()) - a.x.value()) * dx +
             (static_cast<long double>(b.y.value()) - a.y.value()) * dy > 0.0L;
    };
    // Strict monotonicity along the chord excludes cusps and loops in each emitted span.
    if (error(controls, q) <= tolerance && forward(p0, c1) && forward(c1, c2) && forward(c2, p3)) {
      out.push_back({SplineFittedPieceKind::cubic_bezier, p0, {}, p3, first, last,
                     quantize(h1), quantize(h2)});
      return true;
    }
  }
  if (depth >= cfg.maximum_depth)
    return false;
  const auto [left, right] = split(q);
  const auto middle = (first + last) * 0.5L;
  const auto size = out.size();
  if (!fit(left, tolerance, depth + 1, cfg, stop, first, middle, out) ||
      !fit(right, tolerance, depth + 1, cfg, stop, middle, last, out)) {
    out.resize(size);
    return false;
  }
  return true;
}
} // namespace

std::vector<SplineFittedPiece> fit_quintic_ph_to_cubic_copper(
    const QuinticPHRoute& route, const NativeSplineFitConfig& config, std::stop_token stop_token) {
  std::vector<SplineFittedPiece> result;
  if (config.maximum_depth == 0 || config.maximum_pieces == 0 || config.fit_tolerance.value() < 0)
    return result;
  for (const auto& segment : route.segments) {
    if (stop_token.stop_requested())
      return {};
    if (!fit(segment.bezier, static_cast<long double>(config.fit_tolerance.value()), 0, config,
             stop_token, segment.route_parameter_start, segment.route_parameter_end, result))
      return {};
  }
  if (result.empty() || native_spline_max_join_angle(result) > config.maximum_join_angle_radians)
    return {};
  return result;
}
} // namespace flux
