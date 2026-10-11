#include "flux/core/usped.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>

namespace flux {
UspedField build_usped_field(std::size_t width, std::size_t height,
                             const std::vector<std::uint8_t>& walkable,
                             const std::vector<double>& clearance, std::size_t maximum_cells) {
  UspedField result;
  if (width == 0 || height == 0 || width > maximum_cells / height ||
      walkable.size() != width * height || clearance.size() != walkable.size() ||
      std::any_of(clearance.begin(), clearance.end(),
                  [](double d) { return !std::isfinite(d) || d < 0; })) {
    result.diagnostic = "invalid or over-budget USPED raster";
    return result;
  }
  const auto count = walkable.size();
  result.height.resize(count, 1.0);
  result.accumulation.resize(count, 0.0);
  result.transport.resize(count, 0.0);
  result.divergence.resize(count, 0.0);
  result.refinement_priority.resize(count, 0.0);
  const auto none = std::numeric_limits<std::size_t>::max();
  std::vector<std::size_t> downstream(count, none), order(count);
  std::vector<double> slope(count, 0.0);
  for (std::size_t i = 0; i < count; ++i) {
    if (walkable[i]) {
      result.height[i] = 1.0 / (1.0 + clearance[i]);
      result.accumulation[i] = 1.0;
    }
  }
  for (std::size_t i = 0; i < count; ++i) {
    if (!walkable[i])
      continue;
    const auto x = i % width, y = i / width;
    for (int dy = -1; dy <= 1; ++dy) {
      for (int dx = -1; dx <= 1; ++dx) {
        if ((dx == 0 && dy == 0) || (dx < 0 && x == 0) || (dy < 0 && y == 0) ||
            (dx > 0 && x + 1 == width) || (dy > 0 && y + 1 == height))
          continue;
        const auto nx = dx < 0 ? x - 1 : (dx > 0 ? x + 1 : x);
        const auto ny = dy < 0 ? y - 1 : (dy > 0 ? y + 1 : y);
        const auto j = ny * width + nx;
        if (!walkable[j] ||
            (dx != 0 && dy != 0 && (!walkable[y * width + nx] || !walkable[ny * width + x])))
          continue;
        const auto drop =
            (result.height[i] - result.height[j]) / (dx != 0 && dy != 0 ? std::sqrt(2.0) : 1.0);
        if (drop > slope[i]) {
          slope[i] = drop;
          downstream[i] = j;
        }
      }
    }
    if (downstream[i] == none)
      ++result.sinks;
  }
  std::iota(order.begin(), order.end(), 0U);
  std::sort(order.begin(), order.end(), [&](auto a, auto b) {
    return result.height[a] != result.height[b] ? result.height[a] > result.height[b] : a < b;
  });
  for (auto i : order) {
    if (downstream[i] != none)
      result.accumulation[downstream[i]] += result.accumulation[i];
  }
  // Rill USPED: A^1.6 sin(beta)^1.3, with unit material/rainfall factors.
  // Unlike terrain height evolution, use a conservative edge-flux divergence on the masked D8
  // graph. No central difference samples flux through an obstacle; total divergence is zero.
  for (auto i : order) {
    if (downstream[i] == none)
      continue;
    const auto sin_slope = slope[i] / std::hypot(1.0, slope[i]);
    const auto capacity = std::pow(result.accumulation[i], 1.6) * std::pow(sin_slope, 1.3);
    result.transport[i] = capacity;
    result.divergence[i] += capacity;
    result.divergence[downstream[i]] -= capacity;
  }
  double max_flow = 0, max_divergence = 0;
  for (std::size_t i = 0; i < count; ++i) {
    max_flow = std::max(max_flow, std::log(std::max(1.0, result.accumulation[i])));
    max_divergence = std::max(max_divergence, std::log1p(std::abs(result.divergence[i])));
  }
  for (std::size_t i = 0; i < count; ++i) {
    if (!walkable[i])
      continue;
    result.refinement_priority[i] =
        (max_flow > 0 ? std::log(std::max(1.0, result.accumulation[i])) / max_flow : 0) +
        (max_divergence > 0 ? std::log1p(std::abs(result.divergence[i])) / max_divergence : 0);
  }
  return result;
}
} // namespace flux
