#include "flux/core/terrain_evolution.hpp"
#include "terrain_evolution_gpu.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <queue>
#include <tuple>

namespace flux {
namespace {
constexpr std::array<std::pair<int, int>, 8U> directions{
    {{-1, -1}, {0, -1}, {1, -1}, {-1, 0}, {1, 0}, {-1, 1}, {0, 1}, {1, 1}}};
constexpr double epsilon = 1e-12;
constexpr std::size_t absolute_maximum_cells = 1'048'576U;
constexpr std::size_t absolute_maximum_iterations = 256U;
constexpr double maximum_parameter = 1'000.0;

[[nodiscard]] bool valid_config(const TerrainEvolutionConfig& c) {
  return c.maximum_cells != 0U && c.maximum_cells <= absolute_maximum_cells &&
         c.iterations <= absolute_maximum_iterations && std::isfinite(c.erosion_rate) &&
         c.erosion_rate >= 0.0 && c.erosion_rate <= maximum_parameter &&
         std::isfinite(c.deposition_rate) && c.deposition_rate >= 0.0 &&
         c.deposition_rate <= maximum_parameter && std::isfinite(c.capacity_scale) &&
         c.capacity_scale > 0.0 && c.capacity_scale <= maximum_parameter &&
         std::isfinite(c.maximum_height_change) && c.maximum_height_change > 0.0 &&
         c.maximum_height_change <= maximum_parameter && std::isfinite(c.terrain_cost_weight) &&
         c.terrain_cost_weight > 0.0 && c.terrain_cost_weight <= maximum_parameter &&
         std::isfinite(c.pressure_height_scale) && c.pressure_height_scale >= 0.0 &&
         c.pressure_height_scale <= maximum_parameter && std::isfinite(c.success_height_scale) &&
         c.success_height_scale >= 0.0 && c.success_height_scale <= maximum_parameter;
}

[[nodiscard]] std::string terrain_input_diagnostic(
    std::size_t width, std::size_t height, const std::vector<std::uint8_t>& walkable,
    const std::vector<std::uint32_t>& clearance, const TerrainEvolutionConfig& config,
    const std::vector<std::uint32_t>& pressure, const std::vector<std::uint32_t>& success_prior) {
  if (!valid_config(config)) return "invalid terrain evolution configuration";
  if (width == 0U || height == 0U || width > config.maximum_cells / height)
    return "terrain dimensions exceed bounded work budget";
  const auto count = width * height;
  if (count > config.maximum_cells || walkable.size() != count || clearance.size() != count)
    return "invalid terrain input";
  if ((!pressure.empty() && pressure.size() != count) ||
      (!success_prior.empty() && success_prior.size() != count))
    return "terrain feedback field size does not match dimensions";
  return {};
}

[[nodiscard]] bool valid_endpoint_binding(std::size_t width, std::size_t height,
                                          const std::vector<std::uint8_t>& walkable,
                                          std::size_t source, std::size_t target) {
  const auto count = width * height;
  return source < count && target < count && walkable[source] != 0U && walkable[target] != 0U;
}

template <class F>
void neighbours(std::size_t width, std::size_t height, const std::vector<std::uint8_t>& walkable,
                std::size_t cell, F&& visitor) {
  const auto x = cell % width;
  const auto y = cell / width;
  for (const auto [dx, dy] : directions) {
    const auto nx = static_cast<std::ptrdiff_t>(x) + dx;
    const auto ny = static_cast<std::ptrdiff_t>(y) + dy;
    if (nx < 0 || ny < 0 || nx >= static_cast<std::ptrdiff_t>(width) ||
        ny >= static_cast<std::ptrdiff_t>(height))
      continue;
    const auto next = static_cast<std::size_t>(ny) * width + static_cast<std::size_t>(nx);
    if (walkable[next] == 0U)
      continue;
    if (dx != 0 && dy != 0) {
      const auto horizontal = y * width + static_cast<std::size_t>(nx);
      const auto vertical = static_cast<std::size_t>(ny) * width + x;
      if (walkable[horizontal] == 0U || walkable[vertical] == 0U)
        continue;
    }
    visitor(next, dx != 0 && dy != 0 ? std::sqrt(2.0) : 1.0);
  }
}

[[nodiscard]] bool cancelled(std::stop_token token) { return token.stop_requested(); }
} // namespace

static TerrainEvolutionResult
evolve_terrain_fields_impl(std::size_t width, std::size_t height,
                             const std::vector<std::uint8_t>& walkable,
                             const std::vector<std::uint32_t>& clearance,
                             const TerrainEvolutionConfig& config,
                             std::stop_token stop_token, const std::vector<std::uint32_t>& pressure,
                             const std::vector<std::uint32_t>& success_prior, bool gpu) {
  TerrainEvolutionResult result;
  if (const auto diagnostic = terrain_input_diagnostic(width, height, walkable, clearance, config,
                                                        pressure, success_prior);
      !diagnostic.empty()) {
    result.diagnostic = diagnostic;
    return result;
  }
  const auto count = width * height;
  if (cancelled(stop_token)) {
    result.diagnostic = "terrain evolution cancelled";
    return result;
  }

  result.initial_height.resize(count, 0.0);
  result.height.resize(count, 0.0);
  result.flow.assign(count, 0.0);
  result.divergence.assign(count, 0.0);
  for (std::size_t cell = 0U; cell < count; ++cell) {
    if ((cell & 1023U) == 0U && cancelled(stop_token)) {
      result.diagnostic = "terrain evolution cancelled";
      return result;
    }
    // Narrow clearance is high terrain. Obstacles retain a stable sentinel which is never read
    // through a legal neighbour relation.
    if (walkable[cell] != 0U) {
      const auto initial = 1.0 / (1.0 + static_cast<double>(clearance[cell]));
      const auto bounded_pressure =
          pressure.empty() ? 0U : std::min(pressure[cell], config.maximum_pressure);
      const auto bounded_success =
          success_prior.empty() ? 0U : std::min(success_prior[cell], config.maximum_success_prior);
      result.initial_height[cell] =
          std::max(0.0, initial +
                            config.pressure_height_scale * static_cast<double>(bounded_pressure) /
                                std::max(1U, config.maximum_pressure) -
                            config.success_height_scale * static_cast<double>(bounded_success) /
                                std::max(1U, config.maximum_success_prior));
    }
    result.height[cell] = result.initial_height[cell];
  }

  const auto elapsed_us = [](const auto start) {
    return static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::microseconds>(
                                          std::chrono::steady_clock::now() - start)
                                          .count());
  };
  const auto evolution_start = std::chrono::steady_clock::now();
  if (gpu) {
    if (!evolve_terrain_gpu_fields(width, height, walkable, result.initial_height, config,
                                   stop_token, result)) {
      result.evolution_time_us = elapsed_us(evolution_start);
      return result;
    }
  } else {
    std::vector<std::size_t> downstream(count, count);
    std::vector<double> slope(count, 0.0), capacity(count, 0.0);
    for (std::size_t iteration = 0U; iteration < config.iterations; ++iteration) {
      if (cancelled(stop_token)) {
        result.diagnostic = "terrain evolution cancelled";
        result.evolution_time_us = elapsed_us(evolution_start);
        return result;
      }
      std::fill(downstream.begin(), downstream.end(), count);
      std::fill(slope.begin(), slope.end(), 0.0);
      for (std::size_t cell = 0U; cell < count; ++cell) {
        if ((cell & 1023U) == 0U && cancelled(stop_token)) {
          result.diagnostic = "terrain evolution cancelled";
          result.evolution_time_us = elapsed_us(evolution_start);
          return result;
        }
        if (walkable[cell] == 0U)
          continue;
        double best = 0.0;
        std::size_t best_cell = count;
        neighbours(width, height, walkable, cell, [&](std::size_t next, double distance) {
          const auto gradient = (result.height[cell] - result.height[next]) / distance;
          if (gradient > best + epsilon ||
              (std::abs(gradient - best) <= epsilon && gradient > epsilon && next < best_cell)) {
            best = gradient;
            best_cell = next;
          }
        });
        slope[cell] = best;
        downstream[cell] = best_cell;
      }

      // Strict descent makes this a DAG; descending height order is its deterministic topological
      // order. Each legal cell supplies one unit of contributing area as in the shader pipeline.
      result.flow.assign(count, 0.0);
      std::vector<std::size_t> order;
      order.reserve(count);
      for (std::size_t cell = 0U; cell < count; ++cell)
        if (walkable[cell] != 0U) {
          result.flow[cell] = 1.0;
          order.push_back(cell);
        }
      std::sort(order.begin(), order.end(), [&](std::size_t a, std::size_t b) {
        if (result.height[a] != result.height[b])
          return result.height[a] > result.height[b];
        return a < b;
      });
      for (std::size_t order_index = 0U; order_index < order.size(); ++order_index) {
        if ((order_index & 1023U) == 0U && cancelled(stop_token)) {
          result.diagnostic = "terrain evolution cancelled";
          result.evolution_time_us = elapsed_us(evolution_start);
          return result;
        }
        const auto cell = order[order_index];
        if (downstream[cell] != count)
          result.flow[downstream[cell]] += result.flow[cell];
      }
      std::fill(capacity.begin(), capacity.end(), 0.0);
      for (std::size_t order_index = 0U; order_index < order.size(); ++order_index) {
        const auto cell = order[order_index];
        const auto sin_slope = slope[cell] / std::sqrt(1.0 + slope[cell] * slope[cell]);
        capacity[cell] = config.capacity_scale * std::pow(std::max(1.0, result.flow[cell]), 1.6) *
                         std::pow(std::max(0.0, sin_slope), 1.3);
      }
      // Conservative mask-aware finite-volume divergence. Flux is sent only over the selected
      // legal downhill edge; every outgoing amount is mirrored as incoming at its destination.
      result.divergence.assign(count, 0.0);
      for (std::size_t order_index = 0U; order_index < order.size(); ++order_index) {
        if ((order_index & 1023U) == 0U && cancelled(stop_token)) {
          result.diagnostic = "terrain evolution cancelled";
          result.evolution_time_us = elapsed_us(evolution_start);
          return result;
        }
        const auto cell = order[order_index];
        if (downstream[cell] == count)
          continue;
        const auto transport = capacity[cell];
        result.divergence[cell] += transport;
        result.divergence[downstream[cell]] -= transport;
      }
      std::vector<double> next_height = result.height;
      for (std::size_t order_index = 0U; order_index < order.size(); ++order_index) {
        if ((order_index & 1023U) == 0U && cancelled(stop_token)) {
          result.diagnostic = "terrain evolution cancelled";
          result.evolution_time_us = elapsed_us(evolution_start);
          return result;
        }
        const auto cell = order[order_index];
        const auto divergence = result.divergence[cell];
        const auto delta = divergence > 0.0 ? -divergence * config.erosion_rate
                                            : -divergence * config.deposition_rate;
        next_height[cell] =
            std::max(0.0, result.height[cell] + std::clamp(delta, -config.maximum_height_change,
                                                           config.maximum_height_change));
        if (!std::isfinite(next_height[cell])) {
          result.diagnostic = "terrain evolution produced a non-finite height";
          result.evolution_time_us = elapsed_us(evolution_start);
          return result;
        }
      }
      result.height.swap(next_height);
      ++result.iterations_completed;
    }
  }
  result.evolution_time_us = elapsed_us(evolution_start);
  return result;
}

static TerrainEvolutionResult extract_terrain_path(
    TerrainEvolutionResult result, std::size_t width, std::size_t height,
    const std::vector<std::uint8_t>& walkable, std::size_t source, std::size_t target,
    const TerrainEvolutionConfig& config, std::stop_token stop_token,
    const std::vector<std::uint32_t>& pressure, const std::vector<std::uint32_t>& success_prior) {
  const auto count = width * height;
  if (source >= count || target >= count || walkable[source] == 0U || walkable[target] == 0U) {
    result.diagnostic = "invalid terrain input";
    return result;
  }
  const auto elapsed_us = [](const auto start) {
    return static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::microseconds>(
                                          std::chrono::steady_clock::now() - start)
                                          .count());
  };
  const auto extraction_start = std::chrono::steady_clock::now();

  // Path extraction deliberately permits uphill and saddle transitions. The evolved terrain is a
  // soft, positive traversal cost and cannot turn a connected free-space component into a basin.
  using QueueEntry = std::pair<double, std::size_t>;
  std::priority_queue<QueueEntry, std::vector<QueueEntry>, std::greater<>> frontier;
  const auto infinity = std::numeric_limits<double>::infinity();
  std::vector<double> distance(count, infinity);
  std::vector<std::size_t> predecessor(count, count);
  distance[source] = 0.0;
  frontier.emplace(0.0, source);
  while (!frontier.empty()) {
    if (cancelled(stop_token)) {
      result.path.clear();
      result.diagnostic = "terrain evolution cancelled";
      result.extraction_time_us = elapsed_us(extraction_start);
      return result;
    }
    const auto [cost, cell] = frontier.top();
    frontier.pop();
    if (cost != distance[cell])
      continue;
    ++result.expanded_nodes;
    if (cell == target)
      break;
    neighbours(width, height, walkable, cell, [&](std::size_t next, double step) {
      const auto bounded_pressure =
          pressure.empty() ? 0U : std::min(pressure[next], config.maximum_pressure);
      const auto bounded_success =
          success_prior.empty() ? 0U : std::min(success_prior[next], config.maximum_success_prior);
      const auto feedback = config.pressure_height_scale * static_cast<double>(bounded_pressure) /
                                std::max(1U, config.maximum_pressure) -
                            config.success_height_scale * static_cast<double>(bounded_success) /
                                std::max(1U, config.maximum_success_prior);
      const auto candidate =
          cost +
          step * (1.0 + config.terrain_cost_weight * std::max(0.0, result.height[next] + feedback));
      if (candidate + epsilon < distance[next]) {
        distance[next] = candidate;
        predecessor[next] = cell;
        frontier.emplace(candidate, next);
      }
    });
  }
  if (!std::isfinite(distance[target])) {
    result.diagnostic = "terrain source is disconnected from target";
    result.extraction_time_us = elapsed_us(extraction_start);
    return result;
  }
  for (auto cell = target;; cell = predecessor[cell]) {
    result.path.push_back(cell);
    if (cell == source)
      break;
  }
  std::reverse(result.path.begin(), result.path.end());
  result.extraction_time_us = elapsed_us(extraction_start);
  return result;
}

static TerrainEvolutionFields fields_from_evolution(TerrainEvolutionResult result) {
  TerrainEvolutionFields fields;
  fields.initial_height = std::move(result.initial_height);
  fields.height = std::move(result.height);
  fields.flow = std::move(result.flow);
  fields.divergence = std::move(result.divergence);
  fields.iterations_completed = result.iterations_completed;
  fields.used_gpu = result.used_gpu;
  fields.gpu_device = std::move(result.gpu_device);
  fields.gpu_dispatches = result.gpu_dispatches;
  fields.gpu_buffer_growths = result.gpu_buffer_growths;
  fields.gpu_buffer_reuses = result.gpu_buffer_reuses;
  fields.gpu_mask_uploads = result.gpu_mask_uploads;
  fields.gpu_mask_upload_reuses = result.gpu_mask_upload_reuses;
  fields.gpu_mask_cross_pipeline_reuses = result.gpu_mask_cross_pipeline_reuses;
  fields.evolution_time_us = result.evolution_time_us;
  fields.diagnostic = std::move(result.diagnostic);
  return fields;
}

TerrainEvolutionFields evolve_terrain_fields(
    std::size_t width, std::size_t height, const std::vector<std::uint8_t>& walkable,
    const std::vector<std::uint32_t>& clearance, const TerrainEvolutionConfig& config,
    std::stop_token stop_token, const std::vector<std::uint32_t>& pressure,
    const std::vector<std::uint32_t>& success_prior) {
  return fields_from_evolution(evolve_terrain_fields_impl(width, height, walkable, clearance, config,
                                                           stop_token, pressure, success_prior, false));
}

TerrainEvolutionFields evolve_terrain_fields_gpu(
    std::size_t width, std::size_t height, const std::vector<std::uint8_t>& walkable,
    const std::vector<std::uint32_t>& clearance, const TerrainEvolutionConfig& config,
    std::stop_token stop_token, const std::vector<std::uint32_t>& pressure,
    const std::vector<std::uint32_t>& success_prior) {
  return fields_from_evolution(evolve_terrain_fields_impl(width, height, walkable, clearance, config,
                                                           stop_token, pressure, success_prior, true));
}

TerrainEvolutionResult solve_terrain_evolution(
    std::size_t width, std::size_t height, const std::vector<std::uint8_t>& walkable,
    const std::vector<std::uint32_t>& clearance, std::size_t source, std::size_t target,
    const TerrainEvolutionConfig& config, std::stop_token stop_token,
    const std::vector<std::uint32_t>& pressure, const std::vector<std::uint32_t>& success_prior) {
  TerrainEvolutionResult rejected;
  if (const auto diagnostic = terrain_input_diagnostic(width, height, walkable, clearance, config,
                                                        pressure, success_prior);
      !diagnostic.empty()) {
    rejected.diagnostic = diagnostic;
    return rejected;
  }
  if (cancelled(stop_token)) {
    rejected.diagnostic = "terrain evolution cancelled";
    return rejected;
  }
  if (!valid_endpoint_binding(width, height, walkable, source, target)) {
    rejected.diagnostic = "invalid terrain input";
    return rejected;
  }
  auto evolved = evolve_terrain_fields_impl(width, height, walkable, clearance, config, stop_token,
                                             pressure, success_prior, false);
  if (!evolved.diagnostic.empty()) return evolved;
  return extract_terrain_path(std::move(evolved), width, height, walkable, source, target, config,
                              stop_token, pressure, success_prior);
}

TerrainEvolutionResult solve_terrain_evolution_gpu(
    std::size_t width, std::size_t height, const std::vector<std::uint8_t>& walkable,
    const std::vector<std::uint32_t>& clearance, std::size_t source, std::size_t target,
    const TerrainEvolutionConfig& config, std::stop_token stop_token,
    const std::vector<std::uint32_t>& pressure, const std::vector<std::uint32_t>& success_prior) {
  TerrainEvolutionResult rejected;
  if (const auto diagnostic = terrain_input_diagnostic(width, height, walkable, clearance, config,
                                                        pressure, success_prior);
      !diagnostic.empty()) {
    rejected.diagnostic = diagnostic;
    return rejected;
  }
  if (cancelled(stop_token)) {
    rejected.diagnostic = "terrain evolution cancelled";
    return rejected;
  }
  if (!valid_endpoint_binding(width, height, walkable, source, target)) {
    rejected.diagnostic = "invalid terrain input";
    return rejected;
  }
  auto evolved = evolve_terrain_fields_impl(width, height, walkable, clearance, config, stop_token,
                                             pressure, success_prior, true);
  if (!evolved.diagnostic.empty()) return evolved;
  return extract_terrain_path(std::move(evolved), width, height, walkable, source, target, config,
                              stop_token, pressure, success_prior);
}

} // namespace flux
