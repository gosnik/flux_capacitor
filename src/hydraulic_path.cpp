#include "flux/core/hydraulic_path.hpp"
#include "flux/core/routing_compute.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <tuple>
#include <utility>

namespace flux {
namespace {

constexpr std::uint32_t infinite_height = hydraulic_unreachable_height;
constexpr std::array<std::pair<int, int>, 8U> directions{
    std::pair{-1, -1}, std::pair{0, -1}, std::pair{1, -1}, std::pair{-1, 0},
    std::pair{1, 0},   std::pair{-1, 1}, std::pair{0, 1},  std::pair{1, 1}};

[[nodiscard]] bool rasterize_polygon_mask(HydraulicPathInput& input) {
  if (!input.polygon_raster)
    return input.walkable.size() == input.width * input.height;
  const auto& raster = *input.polygon_raster;
  if (raster.pitch_nm <= 0 || raster.minimum_boundary_clearance_nm < 0 ||
      raster.boundary_segments.empty())
    return false;
  const auto count = input.width * input.height;
  input.walkable.assign(count, 0U);
  input.obstacle_clearance.assign(count, 0U);
  for (std::size_t cell = 0U; cell < count; ++cell) {
    const auto x = cell % input.width;
    const auto y = cell / input.width;
    const auto px = static_cast<long double>(x) * raster.pitch_nm + raster.pitch_nm / 2.0L;
    const auto py = static_cast<long double>(y) * raster.pitch_nm + raster.pitch_nm / 2.0L;
    bool inside = false;
    auto nearest = std::numeric_limits<long double>::infinity();
    for (const auto& segment : raster.boundary_segments) {
      const auto ax = static_cast<long double>(segment.first_x_nm);
      const auto ay = static_cast<long double>(segment.first_y_nm);
      const auto bx = static_cast<long double>(segment.second_x_nm);
      const auto by = static_cast<long double>(segment.second_y_nm);
      if ((ay > py) != (by > py) && px < ax + (py - ay) * (bx - ax) / (by - ay))
        inside = !inside;
      const auto dx = bx - ax;
      const auto dy = by - ay;
      const auto denominator = dx * dx + dy * dy;
      const auto parameter =
          denominator > 0.0L
              ? std::clamp(((px - ax) * dx + (py - ay) * dy) / denominator, 0.0L, 1.0L)
              : 0.0L;
      nearest =
          std::min(nearest, std::hypotl(px - (ax + parameter * dx), py - (ay + parameter * dy)));
    }
    if (inside && nearest >= static_cast<long double>(raster.minimum_boundary_clearance_nm)) {
      input.walkable[cell] = 1U;
      input.obstacle_clearance[cell] = static_cast<std::uint32_t>(
          std::min<long double>(std::numeric_limits<std::uint32_t>::max(),
                                std::floor(nearest / static_cast<long double>(raster.pitch_nm))));
    }
  }
  return true;
}

[[nodiscard]] std::uint32_t mix(std::uint32_t value) {
  value ^= value >> 16U;
  value *= 0x7feb352dU;
  value ^= value >> 15U;
  value *= 0x846ca68bU;
  value ^= value >> 16U;
  return value;
}

template <typename Visitor>
void visit_neighbours(const HydraulicPathInput& input, std::size_t cell, Visitor visitor) {
  const auto x = cell % input.width;
  const auto y = cell / input.width;
  for (std::size_t direction = 0U; direction < directions.size(); ++direction) {
    const auto [dx, dy] = directions[direction];
    const auto next_x = static_cast<std::int64_t>(x) + dx;
    const auto next_y = static_cast<std::int64_t>(y) + dy;
    if (next_x < 0 || next_y < 0 || next_x >= static_cast<std::int64_t>(input.width) ||
        next_y >= static_cast<std::int64_t>(input.height)) {
      continue;
    }
    const auto next =
        static_cast<std::size_t>(next_y) * input.width + static_cast<std::size_t>(next_x);
    if (dx != 0 && dy != 0) {
      const auto horizontal = y * input.width + static_cast<std::size_t>(next_x);
      const auto vertical = static_cast<std::size_t>(next_y) * input.width + x;
      if (input.walkable[horizontal] == 0U || input.walkable[vertical] == 0U) {
        continue;
      }
    }
    if (input.walkable[next] != 0U) {
      visitor(next, dx != 0 && dy != 0 ? 1'414U : 1'000U, direction);
    }
  }
}

[[nodiscard]] HydraulicPathResult solve_cpu(const HydraulicPathInput& input,
                                            const HydraulicPathConfig& config) {
  HydraulicPathResult result;
  const auto cell_count = input.width * input.height;
  std::vector<std::uint32_t> potential(cell_count, infinite_height);
  std::vector<std::uint32_t> next_potential(cell_count, infinite_height);
  potential[input.target] = 0U;
  for (std::size_t pass = 0U; pass < config.relaxation_iterations; ++pass) {
    bool changed = false;
    for (std::size_t cell = 0U; cell < cell_count; ++cell) {
      if (input.walkable[cell] == 0U) {
        next_potential[cell] = infinite_height;
        continue;
      }
      if (cell == input.target) {
        next_potential[cell] = 0U;
        continue;
      }
      auto value = potential[cell];
      visit_neighbours(input, cell, [&](std::size_t neighbour, std::uint32_t cost, std::size_t) {
        const auto pressure =
            std::min(input.downstream_pressure[neighbour], config.maximum_downstream_pressure);
        const auto feedback_cost =
            static_cast<std::uint64_t>(pressure) * config.downstream_pressure_score_scale;
        const auto total_cost = std::min<std::uint64_t>(
            infinite_height, static_cast<std::uint64_t>(cost) + feedback_cost);
        if (potential[neighbour] < infinite_height - total_cost) {
          value = std::min(value, static_cast<std::uint32_t>(potential[neighbour] + total_cost));
        }
      });
      next_potential[cell] = value;
      changed = changed || value != potential[cell];
    }
    potential.swap(next_potential);
    ++result.relaxation_passes;
    if (!changed) {
      break;
    }
  }
  if (potential[input.source] == infinite_height) {
    result.budget_exhausted = result.relaxation_passes >= config.relaxation_iterations;
    result.diagnostic = result.budget_exhausted ? "hydraulic potential relaxation budget exhausted"
                                                : "hydraulic source is disconnected from target";
    return result;
  }

  result.accumulated_flow.assign(cell_count, 0U);
  result.erosion.resize(cell_count, 0U);
  std::transform(input.success_prior.begin(), input.success_prior.end(), result.erosion.begin(),
                 [&](const auto prior) { return std::min(prior, config.maximum_erosion); });
  result.downstream_pressure = input.downstream_pressure;
  result.success_prior = input.success_prior;
  std::vector<std::uint32_t> batch_flow(cell_count, 0U);
  const auto folded_seed =
      static_cast<std::uint32_t>(config.deterministic_seed ^ (config.deterministic_seed >> 32U));
  for (std::size_t batch = 0U; batch < config.droplet_batches; ++batch) {
    std::fill(batch_flow.begin(), batch_flow.end(), 0U);
    for (std::size_t droplet = 0U; droplet < config.droplets_per_batch; ++droplet) {
      auto cell = input.source;
      ++batch_flow[cell];
      ++result.accumulated_flow[cell];
      for (std::size_t step = 0U; step < config.maximum_steps_per_droplet && cell != input.target;
           ++step) {
        std::optional<std::tuple<std::uint64_t, std::size_t>> selected;
        visit_neighbours(
            input, cell, [&](std::size_t neighbour, std::uint32_t, std::size_t direction) {
              if (potential[neighbour] >= potential[cell]) {
                return;
              }
              const auto erosion_reward = static_cast<std::uint64_t>(
                  std::min(result.erosion[neighbour], config.maximum_erosion));
              const auto clearance_reward = static_cast<std::uint64_t>(std::min(
                  input.obstacle_clearance[neighbour], config.maximum_clearance_reward_cells));
              const auto jitter_sample =
                  mix(folded_seed ^ (static_cast<std::uint32_t>(batch) * 0x9e3779b9U) ^
                      (static_cast<std::uint32_t>(droplet) * 0x85ebca6bU) ^
                      (static_cast<std::uint32_t>(step) * 16U) ^
                      static_cast<std::uint32_t>(direction)) &
                  0x3ffU;
              const auto jitter = static_cast<std::uint64_t>(jitter_sample) *
                                  config.exploration_score_quantum /
                                  static_cast<std::uint64_t>(batch + 1U);
              const auto height = static_cast<std::uint64_t>(potential[neighbour]) * 1'024ULL;
              const auto total_reward = erosion_reward * config.erosion_height_scale +
                                        clearance_reward * config.clearance_score_scale;
              const auto score = height > total_reward ? height - total_reward + jitter
                                                       : static_cast<std::uint64_t>(jitter);
              const auto candidate = std::tuple{score, neighbour};
              if (!selected || candidate < *selected) {
                selected = candidate;
              }
            });
        if (!selected) {
          break;
        }
        cell = std::get<1>(*selected);
        ++batch_flow[cell];
        ++result.accumulated_flow[cell];
        ++result.droplet_steps;
      }
      result.droplets_reaching_target += cell == input.target ? 1U : 0U;
    }
    for (std::size_t cell = 0U; cell < cell_count; ++cell) {
      if (input.walkable[cell] == 0U || cell == input.target) {
        continue;
      }
      std::uint64_t effective_flow = batch_flow[cell];
      visit_neighbours(
          input, cell, [&](std::size_t neighbour, std::uint32_t, std::size_t direction) {
            const auto diagonal =
                directions[direction].first != 0 && directions[direction].second != 0;
            const auto divisor =
                static_cast<std::uint64_t>(config.lateral_erosion_divisor) * (diagonal ? 2U : 1U);
            effective_flow += batch_flow[neighbour] / divisor;
          });
      if (effective_flow == 0U)
        continue;
      const auto increment = std::max<std::uint32_t>(
          1U, static_cast<std::uint32_t>(std::min<std::uint64_t>(
                  std::numeric_limits<std::uint32_t>::max(),
                  effective_flow / std::max(1U, config.erosion_per_flow_quantum))));
      result.erosion[cell] = std::min(config.maximum_erosion, result.erosion[cell] + increment);
    }
  }

  result.static_potential = potential;
  result.terrain_height.resize(cell_count, infinite_height);
  for (std::size_t terrain_cell = 0U; terrain_cell < cell_count; ++terrain_cell) {
    if (potential[terrain_cell] == infinite_height || input.walkable[terrain_cell] == 0U) {
      continue;
    }
    const auto reduction = static_cast<std::uint64_t>(result.erosion[terrain_cell]) *
                           config.erosion_height_scale / 1'024ULL;
    result.terrain_height[terrain_cell] = static_cast<std::uint32_t>(
        reduction < potential[terrain_cell] ? potential[terrain_cell] - reduction : 0U);
  }

  result.riverbed = {input.source};
  auto cell = input.source;
  while (cell != input.target && result.riverbed.size() <= cell_count) {
    std::optional<
        std::tuple<std::uint32_t, std::uint32_t, std::uint32_t, std::uint32_t, std::size_t>>
        selected;
    visit_neighbours(input, cell, [&](std::size_t neighbour, std::uint32_t, std::size_t) {
      if (potential[neighbour] >= potential[cell]) {
        return;
      }
      // tuple is inverted for a minimum comparison: prefer high accumulated flow, then erosion,
      // then low potential and stable coordinate.
      const auto candidate = std::tuple{
          std::numeric_limits<std::uint32_t>::max() - result.accumulated_flow[neighbour],
          std::numeric_limits<std::uint32_t>::max() - result.erosion[neighbour],
          std::numeric_limits<std::uint32_t>::max() - input.obstacle_clearance[neighbour],
          potential[neighbour], neighbour};
      if (!selected || candidate < *selected) {
        selected = candidate;
      }
    });
    if (!selected) {
      result.riverbed.clear();
      result.diagnostic = "hydraulic riverbed extraction reached a local minimum";
      return result;
    }
    cell = std::get<4>(*selected);
    result.riverbed.push_back(cell);
  }
  if (result.riverbed.empty() || result.riverbed.back() != input.target) {
    result.riverbed.clear();
    result.diagnostic = "hydraulic riverbed did not reach its target";
  }
  return result;
}

[[nodiscard]] bool prepare_clearance_field(HydraulicPathInput& input, bool prefer_gpu,
                                           bool& used_gpu) {
  if (!input.obstacle_clearance.empty()) {
    used_gpu = false;
    return input.obstacle_clearance.size() == input.walkable.size();
  }
  auto legal = input.walkable;
  for (std::size_t cell = 0U; cell < input.walkable.size(); ++cell) {
    const auto x = cell % input.width;
    const auto y = cell / input.width;
    if (x == 0U || y == 0U || x + 1U == input.width || y + 1U == input.height) {
      legal[cell] = 0U;
    }
  }
  RoutingComputeWavefrontInput request;
  request.width = input.width;
  request.height = input.height;
  request.traversable.assign(legal.size(), 1U);
  request.maximum_passes = std::max(input.width, input.height);
  for (std::size_t cell = 0U; cell < legal.size(); ++cell) {
    if (legal[cell] == 0U)
      request.sources.push_back(cell);
  }
  const auto clearance = compute_routing_wavefront(request, prefer_gpu);
  if (!clearance.ok())
    return false;
  input.obstacle_clearance.resize(clearance.distance.size());
  for (std::size_t cell = 0U; cell < clearance.distance.size(); ++cell) {
    input.obstacle_clearance[cell] = legal[cell] != 0U ? clearance.distance[cell] : 0U;
  }
  used_gpu = clearance.used_gpu;
  return true;
}

} // namespace

#if FLUX_HAS_VULKAN_HYDRAULIC_PATHS
[[nodiscard]] HydraulicPathResult solve_hydraulic_gpu(const HydraulicPathInput& input,
                                                      const HydraulicPathConfig& config);
[[nodiscard]] bool hydraulic_gpu_runtime_available();
#endif

bool hydraulic_gpu_compute_available() {
#if FLUX_HAS_VULKAN_HYDRAULIC_PATHS
  return hydraulic_gpu_runtime_available();
#else
  return false;
#endif
}

HydraulicPathResult solve_hydraulic_erosion_path(const HydraulicPathInput& input,
                                                 const HydraulicPathConfig& config) {
  const auto count = input.width > 0U && input.height > 0U &&
                             input.width <= std::numeric_limits<std::size_t>::max() / input.height
                         ? input.width * input.height
                         : 0U;
  const auto deferred_raster = input.polygon_raster && input.walkable.empty();
  if (count == 0U || (!deferred_raster && input.walkable.size() != count) ||
      input.source >= count || input.target >= count || input.source == input.target ||
      (!deferred_raster &&
       (input.walkable[input.source] == 0U || input.walkable[input.target] == 0U)) ||
      config.relaxation_iterations == 0U || config.droplet_batches == 0U ||
      config.droplets_per_batch == 0U || config.maximum_steps_per_droplet == 0U ||
      config.erosion_per_flow_quantum == 0U || config.maximum_erosion == 0U ||
      config.exploration_score_quantum == 0U || config.erosion_height_scale == 0U ||
      config.clearance_score_scale == 0U || config.maximum_clearance_reward_cells == 0U ||
      config.lateral_erosion_divisor == 0U || config.downstream_pressure_score_scale == 0U ||
      config.maximum_downstream_pressure == 0U ||
      static_cast<std::uint64_t>(config.downstream_pressure_score_scale) *
              config.maximum_downstream_pressure >
          static_cast<std::uint64_t>(hydraulic_unreachable_height - 1'414U) ||
      (!input.obstacle_clearance.empty() && input.obstacle_clearance.size() != count) ||
      (!input.downstream_pressure.empty() && input.downstream_pressure.size() != count) ||
      (!input.success_prior.empty() && input.success_prior.size() != count)) {
    HydraulicPathResult invalid;
    invalid.diagnostic = "invalid hydraulic erosion path request";
    return invalid;
  }
  auto prepared_input = input;
  if (prepared_input.downstream_pressure.empty()) {
    prepared_input.downstream_pressure.assign(count, 0U);
  }
  if (prepared_input.success_prior.empty()) {
    prepared_input.success_prior.assign(count, 0U);
  }
#if FLUX_HAS_VULKAN_HYDRAULIC_PATHS
  if (config.prefer_gpu) {
    auto gpu = solve_hydraulic_gpu(prepared_input, config);
    if (gpu.ok()) {
      gpu.used_gpu = true;
      return gpu;
    }
    // GPU acceleration is a production requirement. Keep the explicit CPU implementation as a
    // deterministic test/oracle backend, but never turn a requested GPU route into an unnoticed
    // CPU route after initialization, allocation, or dispatch failure.
    return gpu;
  }
#endif
  if (!rasterize_polygon_mask(prepared_input) || prepared_input.walkable[input.source] == 0U ||
      prepared_input.walkable[input.target] == 0U) {
    HydraulicPathResult invalid;
    invalid.diagnostic = "hydraulic polygon rasterization could not bind its endpoints";
    return invalid;
  }
  bool used_gpu_clearance = false;
  if (!prepare_clearance_field(prepared_input, false, used_gpu_clearance)) {
    HydraulicPathResult invalid;
    invalid.diagnostic = "hydraulic obstacle-clearance field generation failed";
    return invalid;
  }
  auto cpu = solve_cpu(prepared_input, config);
  cpu.walkable = prepared_input.walkable;
  cpu.obstacle_clearance = prepared_input.obstacle_clearance;
  cpu.used_gpu_clearance_field = used_gpu_clearance;
  return cpu;
}

} // namespace flux
