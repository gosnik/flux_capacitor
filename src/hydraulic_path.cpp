#include "flux/core/hydraulic_path.hpp"
#include "flux/core/routing_compute.hpp"
#include "flux/core/indexed_clearance_map.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <mutex>
#include <atomic>
#include <chrono>
#include <tuple>
#include <utility>

namespace flux {
namespace {

constexpr std::uint32_t infinite_height = hydraulic_unreachable_height;
constexpr std::array<std::pair<int, int>, 8U> directions{
    std::pair{-1, -1}, std::pair{0, -1}, std::pair{1, -1}, std::pair{-1, 0},
    std::pair{1, 0},   std::pair{-1, 1}, std::pair{0, 1},  std::pair{1, 1}};

[[nodiscard]] std::uint32_t directional_scale(const HydraulicPathConfig& config, int dx, int dy) {
  if (config.transverse_anisotropy_per_mille == 1'000U) {
    return 1'000U;
  }
  const auto axis_norm = static_cast<std::uint64_t>(std::abs(config.anisotropy_axis_x)) +
                         static_cast<std::uint64_t>(std::abs(config.anisotropy_axis_y));
  if (axis_norm == 0U) {
    return 1'000U;
  }
  const auto cross = std::abs(static_cast<std::int64_t>(config.anisotropy_axis_x) * dy -
                              static_cast<std::int64_t>(config.anisotropy_axis_y) * dx);
  const auto transverse = static_cast<std::uint64_t>(
      config.transverse_anisotropy_per_mille - 1'000U);
  return static_cast<std::uint32_t>(
      1'000U + transverse * static_cast<std::uint64_t>(cross) / axis_norm);
}

[[nodiscard]] std::uint32_t anisotropic_step_cost(const HydraulicPathConfig& config,
                                                  std::uint32_t ordinary_cost, int dx, int dy) {
  return static_cast<std::uint32_t>(
      (static_cast<std::uint64_t>(ordinary_cost) * directional_scale(config, dx, dy) + 999U) /
      1'000U);
}

struct PreparedIndexedRaster {
  HydraulicPolygonRaster raster;
  std::size_t width{}, height{}, bytes{};
  std::shared_ptr<const IndexedClearanceMap> map;
  std::vector<std::uint8_t> walkable;
  std::vector<std::uint32_t> clearance, nearest_owner;
};

// Exact structural keys exclude endpoints, pressure and history. Each immutable lease includes
// its physical fields; mutable terrain is always created separately for the current solve.
std::shared_ptr<const PreparedIndexedRaster> indexed_polygon_raster(
    const HydraulicPathInput& input, HydraulicPathResult& result, std::stop_token stop) {
  constexpr std::size_t byte_limit = 128U * 1024U * 1024U;
  static std::timed_mutex mutex;
  static std::vector<std::shared_ptr<const PreparedIndexedRaster>> cache;
  static std::atomic<std::uint64_t> revision{0U};
  // One bounded builder avoids duplicate allocations when candidate workers miss together.
  // Waiting remains cancellable; immutable leases are usable after this lock is released.
  std::unique_lock lock(mutex, std::defer_lock);
  while (!lock.try_lock_for(std::chrono::milliseconds(10)))
    if (stop.stop_requested()) return {};
  if (stop.stop_requested()) return {};
  const auto& raster = *input.polygon_raster;
  const auto matches = [&](const auto& entry) {
    return entry->width == input.width && entry->height == input.height && entry->raster == raster;
  };
  const auto describe = [&](const auto& entry, bool reused) {
    result.indexed_clearance_map = entry->map;
    result.indexed_clearance_map_builds = reused ? 0U : 1U;
    result.indexed_clearance_map_reuses = reused ? 1U : 0U;
    result.indexed_clearance_map_cells = entry->map->cell_record_indices().size();
    result.indexed_clearance_map_records = entry->map->records().size();
    result.indexed_clearance_map_references = entry->map->overlap_primitive_indices().size();
  };
  {
    if (const auto found = std::find_if(cache.begin(), cache.end(), matches); found != cache.end()) {
      auto entry = *found;
      cache.erase(found); cache.push_back(entry);
      describe(entry, true);
      return entry;
    }
  }
  const auto count = input.width * input.height;
  const auto physical_width = std::ceil(static_cast<long double>(input.width) * raster.pitch_nm / 50'000.0L);
  const auto physical_height = std::ceil(static_cast<long double>(input.height) * raster.pitch_nm / 50'000.0L);
  const auto fail = [&](const char* why) -> std::shared_ptr<const PreparedIndexedRaster> {
    ++result.indexed_clearance_map_fallbacks;
    result.indexed_clearance_map_diagnostic = why;
    return {};
  };
  if (physical_width < 1 || physical_height < 1 || physical_width * physical_height > 4'000'000.0L ||
      count > byte_limit / (sizeof(std::uint32_t) * 3U + sizeof(std::uint8_t)))
    return fail("indexed physical map exceeds bounded working set; exact polygon raster retained");
  IndexedClearanceMapConfig config;
  config.origin_x_nm = raster.origin_x_nm; config.origin_y_nm = raster.origin_y_nm;
  config.width = static_cast<std::uint32_t>(physical_width);
  config.height = static_cast<std::uint32_t>(physical_height);
  config.geometry_revision = ++revision;
  config.maximum_bytes = byte_limit - count * (sizeof(std::uint32_t) * 3U + sizeof(std::uint8_t));
  config.cancellation_requested = [stop] { return stop.stop_requested(); };
  config.sampled_cells.reserve(count);
  for (std::size_t cell = 0; cell < count; ++cell) {
    if ((cell & 1023U) == 0U && stop.stop_requested()) return {};
    const auto x = static_cast<std::uint32_t>((static_cast<long double>(cell % input.width) + 0.5L) * raster.pitch_nm / config.pitch_nm);
    const auto y = static_cast<std::uint32_t>((static_cast<long double>(cell / input.width) + 0.5L) * raster.pitch_nm / config.pitch_nm);
    config.sampled_cells.push_back(y * config.width + x);
  }
  std::vector<IndexedClearancePrimitive> primitives;
  primitives.reserve(raster.boundary_segments.size());
  const auto coordinate = [](std::int64_t origin, std::int32_t relative) -> std::optional<std::int64_t> {
    if ((relative > 0 && origin > std::numeric_limits<std::int64_t>::max() - relative) ||
        (relative < 0 && origin < std::numeric_limits<std::int64_t>::min() - relative)) return {};
    return origin + relative;
  };
  for (const auto& boundary : raster.boundary_segments) {
    const auto ax = coordinate(raster.origin_x_nm, boundary.first_x_nm);
    const auto ay = coordinate(raster.origin_y_nm, boundary.first_y_nm);
    const auto bx = coordinate(raster.origin_x_nm, boundary.second_x_nm);
    const auto by = coordinate(raster.origin_y_nm, boundary.second_y_nm);
    if (!ax || !ay || !bx || !by) return fail("indexed map coordinate overflow; exact polygon raster retained");
    primitives.push_back({*ax, *ay, *bx, *by, boundary.obstacle_id, {}, {}});
  }
  const auto built = build_indexed_clearance_map(std::move(config), std::move(primitives));
  if (!built.map) {
    if (stop.stop_requested()) return {};
    return fail(built.diagnostic.c_str());
  }
  auto entry = std::make_shared<PreparedIndexedRaster>();
  entry->width = input.width; entry->height = input.height; entry->raster = raster;
  entry->map = built.map;
  entry->walkable.assign(count, 0U); entry->clearance.assign(count, 0U);
  entry->nearest_owner.assign(count, IndexedClearanceMap::invalid_index);
  for (std::size_t cell = 0; cell < count; ++cell) {
    if ((cell & 255U) == 0U && stop.stop_requested()) return {};
    const auto x = static_cast<long double>(raster.origin_x_nm) +
        (static_cast<long double>(cell % input.width) + 0.5L) * raster.pitch_nm;
    const auto y = static_cast<long double>(raster.origin_y_nm) +
        (static_cast<long double>(cell / input.width) + 0.5L) * raster.pitch_nm;
    const auto query = built.map->exact_query(x, y);
    if (query.nearest_primitive_index != IndexedClearanceMap::invalid_index)
      entry->nearest_owner[cell] = built.map->primitives()[query.nearest_primitive_index].owner_id;
    if (query.inside && query.boundary_distance_nm >= raster.minimum_boundary_clearance_nm) {
      entry->walkable[cell] = 1U;
      entry->clearance[cell] = static_cast<std::uint32_t>(std::min<long double>(
          std::numeric_limits<std::uint32_t>::max(), std::floor(query.boundary_distance_nm / raster.pitch_nm)));
    }
  }
  entry->bytes = built.map->allocated_bytes() + sizeof(PreparedIndexedRaster) +
      entry->raster.boundary_segments.capacity() * sizeof(HydraulicRasterSegment) +
      count * (sizeof(std::uint8_t) + 2U * sizeof(std::uint32_t));
  {
    if (const auto found = std::find_if(cache.begin(), cache.end(), matches); found != cache.end()) {
      describe(*found, true); return *found;
    }
    auto bytes = std::size_t{0};
    for (const auto& existing : cache) bytes += existing->bytes;
    while (!cache.empty() && (cache.size() >= 4U || bytes + entry->bytes > byte_limit)) {
      // Erasing a cache reference never invalidates a consumer's immutable map lease.
      bytes -= cache.front()->bytes; cache.erase(cache.begin());
    }
    if (entry->bytes <= byte_limit) cache.push_back(entry);
  }
  describe(entry, false);
  return entry;
}

[[nodiscard]] bool rasterize_polygon_mask(HydraulicPathInput& input, std::stop_token stop_token = {},
    HydraulicPathResult* result = nullptr, bool use_index = false) {
  if (!input.polygon_raster)
    return input.walkable.size() == input.width * input.height;
  const auto& raster = *input.polygon_raster;
  if (raster.pitch_nm <= 0 || raster.minimum_boundary_clearance_nm < 0 ||
      raster.boundary_segments.empty())
    return false;
  const auto count = input.width * input.height;
  if (use_index && result) {
    if (auto prepared = indexed_polygon_raster(input, *result, stop_token)) {
      input.walkable = prepared->walkable;
      input.obstacle_clearance = prepared->clearance;
      result->nearest_obstacle_id = prepared->nearest_owner;
      return true;
    }
    if (stop_token.stop_requested()) return false;
  }
  input.walkable.assign(count, 0U);
  input.obstacle_clearance.assign(count, 0U);
  for (std::size_t cell = 0U; cell < count; ++cell) {
    if ((cell & 255U) == 0U && stop_token.stop_requested()) return false;
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
                                            const HydraulicPathConfig& config, std::stop_token stop) {
  HydraulicPathResult result;
  const auto cancelled = [&] {
    result.riverbed.clear(); result.diagnostic = "hydraulic solve cancelled";
    return result;
  };
  const auto cell_count = input.width * input.height;
  std::vector<std::uint32_t> potential(cell_count, infinite_height);
  std::vector<std::uint32_t> next_potential(cell_count, infinite_height);
  potential[input.target] = 0U;
  for (std::size_t pass = 0U; pass < config.relaxation_iterations; ++pass) {
    if (stop.stop_requested()) return cancelled();
    bool changed = false;
    for (std::size_t cell = 0U; cell < cell_count; ++cell) {
      if ((cell & 1023U) == 0U && stop.stop_requested()) return cancelled();
      if (input.walkable[cell] == 0U) {
        next_potential[cell] = infinite_height;
        continue;
      }
      if (cell == input.target) {
        next_potential[cell] = 0U;
        continue;
      }
      auto value = potential[cell];
      visit_neighbours(input, cell, [&](std::size_t neighbour, std::uint32_t cost,
                                        std::size_t direction) {
        const auto [step_x, step_y] = directions[direction];
        cost = anisotropic_step_cost(config, cost, step_x, step_y);
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
      if (stop.stop_requested()) return cancelled();
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
      if ((cell & 1023U) == 0U && stop.stop_requested()) return cancelled();
      if (input.walkable[cell] == 0U || cell == input.target) {
        continue;
      }
      std::uint64_t effective_flow = batch_flow[cell];
      visit_neighbours(
          input, cell, [&](std::size_t neighbour, std::uint32_t, std::size_t direction) {
            const auto diagonal =
                directions[direction].first != 0 && directions[direction].second != 0;
            const auto [step_x, step_y] = directions[direction];
            const auto divisor =
                (static_cast<std::uint64_t>(config.lateral_erosion_divisor) *
                     (diagonal ? 2U : 1U) * directional_scale(config, step_x, step_y) +
                 999U) /
                1'000U;
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
    if (stop.stop_requested()) return cancelled();
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
                                                 const HydraulicPathConfig& config, std::stop_token stop_token) {
  if (stop_token.stop_requested()) { HydraulicPathResult r; r.diagnostic = "hydraulic discovery cancelled"; return r; }
  const auto count = input.width > 0U && input.height > 0U &&
                             input.width <= std::numeric_limits<std::size_t>::max() / input.height
                         ? input.width * input.height
                         : 0U;
  const auto deferred_raster = input.polygon_raster && input.walkable.empty();
  if (count == 0U || (!deferred_raster && input.walkable.size() != count) ||
      input.source >= count || input.target >= count || input.source == input.target ||
      (!deferred_raster &&
       (input.walkable[input.source] == 0U || input.walkable[input.target] == 0U)) ||
      (!config.evolve_terrain_height && (config.relaxation_iterations == 0U || config.droplet_batches == 0U ||
      config.droplets_per_batch == 0U || config.maximum_steps_per_droplet == 0U ||
      config.erosion_per_flow_quantum == 0U || config.maximum_erosion == 0U ||
      config.exploration_score_quantum == 0U || config.erosion_height_scale == 0U ||
      config.clearance_score_scale == 0U || config.maximum_clearance_reward_cells == 0U ||
      config.lateral_erosion_divisor == 0U || config.downstream_pressure_score_scale == 0U ||
      config.maximum_downstream_pressure == 0U ||
      std::abs(static_cast<std::int64_t>(config.anisotropy_axis_x)) > 1'024 ||
      std::abs(static_cast<std::int64_t>(config.anisotropy_axis_y)) > 1'024 ||
      config.transverse_anisotropy_per_mille < 1'000U ||
      config.transverse_anisotropy_per_mille > 16'000U ||
      (config.transverse_anisotropy_per_mille > 1'000U && config.anisotropy_axis_x == 0 &&
       config.anisotropy_axis_y == 0) ||
      config.lateral_erosion_divisor > 100'000U ||
      static_cast<std::uint64_t>(config.downstream_pressure_score_scale) *
              config.maximum_downstream_pressure >
          static_cast<std::uint64_t>(hydraulic_unreachable_height) -
              (1'414ULL * config.transverse_anisotropy_per_mille + 999ULL) / 1'000ULL)) ||
      (!input.obstacle_clearance.empty() && input.obstacle_clearance.size() != count) ||
      (!input.downstream_pressure.empty() && input.downstream_pressure.size() != count) ||
      (!input.success_prior.empty() && input.success_prior.size() != count)) {
    HydraulicPathResult invalid;
    invalid.diagnostic = "invalid hydraulic erosion path request";
    return invalid;
  }
  if (config.evolve_terrain_height && (count > config.terrain_evolution.maximum_cells || count > 1'048'576U)) {
    HydraulicPathResult invalid;
    invalid.used_height_evolution = true;
    invalid.budget_exhausted = true;
    invalid.diagnostic = "height-evolution raster cell budget exhausted";
    return invalid;
  }
  auto prepared_input = input;
  if (prepared_input.downstream_pressure.empty()) {
    prepared_input.downstream_pressure.assign(count, 0U);
  }
  if (prepared_input.success_prior.empty()) {
    prepared_input.success_prior.assign(count, 0U);
  }
  if (config.evolve_terrain_height) {
    HydraulicPathResult result;
    result.used_height_evolution = true;
    if (count > config.terrain_evolution.maximum_cells) {
      result.budget_exhausted = true;
      result.diagnostic = "height-evolution raster cell budget exhausted";
      return result;
    }
    if (!rasterize_polygon_mask(prepared_input, stop_token, &result, config.use_indexed_clearance_map) ||
        !prepare_clearance_field(prepared_input, false, result.used_gpu_clearance_field)) {
      result.diagnostic = stop_token.stop_requested()
          ? "height-evolution discovery cancelled during rasterization"
          : "height-evolution legal-space rasterization failed";
      return result;
    }
    result.height_evolution_invoked = true;
    const auto evolved = config.prefer_gpu
        ? solve_terrain_evolution_gpu(input.width, input.height, prepared_input.walkable,
                                      prepared_input.obstacle_clearance, input.source, input.target,
                                      config.terrain_evolution, stop_token,
                                      prepared_input.downstream_pressure, prepared_input.success_prior)
        : solve_terrain_evolution(input.width, input.height, prepared_input.walkable,
                                  prepared_input.obstacle_clearance, input.source, input.target,
                                  config.terrain_evolution, stop_token,
                                  prepared_input.downstream_pressure, prepared_input.success_prior);
    // The terrain backend is a real execution result, never an echo of prefer_gpu.  Preserve its
    // buffer counters in the established hydraulic GPU diagnostics so production telemetry sees
    // all device work performed for this route.
    result.used_gpu = evolved.used_gpu;
    result.gpu_persistent_buffer_reuses = evolved.gpu_buffer_reuses;
    result.gpu_buffer_growths = evolved.gpu_buffer_growths;
    result.height_evolution_gpu_invoked = evolved.used_gpu;
    result.height_evolution_gpu_dispatches = evolved.gpu_dispatches;
    result.height_evolution_gpu_mask_uploads = evolved.gpu_mask_uploads;
    result.height_evolution_gpu_mask_upload_reuses = evolved.gpu_mask_upload_reuses;
    result.height_evolution_gpu_mask_cross_pipeline_reuses = evolved.gpu_mask_cross_pipeline_reuses;
    result.gpu_mask_uploads = evolved.gpu_mask_uploads;
    result.gpu_mask_upload_reuses = evolved.gpu_mask_upload_reuses;
    result.gpu_mask_cross_pipeline_reuses = evolved.gpu_mask_cross_pipeline_reuses;
    result.height_evolution_field_us = evolved.evolution_time_us;
    result.height_evolution_extraction_us = evolved.extraction_time_us;
    result.height_evolution_iterations = evolved.iterations_completed;
    result.height_evolution_expanded_nodes = evolved.expanded_nodes;
    for (std::size_t i = 0; i < evolved.height.size(); ++i)
      result.height_evolution_changed_cells += std::abs(evolved.initial_height[i] - evolved.height[i]) > 1e-12;
    if (!evolved.ok()) {
      result.diagnostic = evolved.diagnostic;
      return result;
    }
    result.riverbed = evolved.path;
    result.walkable = prepared_input.walkable;
    result.obstacle_clearance = prepared_input.obstacle_clearance;
    result.downstream_pressure = prepared_input.downstream_pressure;
    result.success_prior = prepared_input.success_prior;
    result.static_potential.assign(count, infinite_height);
    result.terrain_height.assign(count, infinite_height);
    result.accumulated_flow.assign(count, 0U);
    result.erosion.assign(count, 0U);
    const auto encoded = [](double value) {
      return static_cast<std::uint32_t>(std::clamp(value * 1'000'000.0, 0.0,
          static_cast<double>(infinite_height - 1U)));
    };
    for (std::size_t i = 0; i < count; ++i) {
      if (!result.walkable[i]) continue;
      result.static_potential[i] = encoded(evolved.initial_height[i]);
      result.terrain_height[i] = encoded(evolved.height[i]);
      result.accumulated_flow[i] = static_cast<std::uint32_t>(std::clamp(evolved.flow[i], 0.0,
          static_cast<double>(std::numeric_limits<std::uint32_t>::max())));
      result.erosion[i] = encoded(std::max(0.0, evolved.initial_height[i] - evolved.height[i]));
    }
    return result;
  }
#if FLUX_HAS_VULKAN_HYDRAULIC_PATHS
  if (config.prefer_gpu) {
    if (stop_token.stop_requested()) { HydraulicPathResult r; r.diagnostic = "hydraulic GPU solve cancelled"; return r; }
    auto gpu = solve_hydraulic_gpu(prepared_input, config);
    if (stop_token.stop_requested()) {
      gpu.riverbed.clear(); gpu.diagnostic = "hydraulic GPU solve cancelled after fenced dispatch";
      return gpu;
    }
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
  HydraulicPathResult physical;
  if (!rasterize_polygon_mask(prepared_input, stop_token, &physical, config.use_indexed_clearance_map) || prepared_input.walkable[input.source] == 0U ||
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
  auto cpu = solve_cpu(prepared_input, config, stop_token);
  cpu.indexed_clearance_map = std::move(physical.indexed_clearance_map);
  cpu.indexed_clearance_map_builds = physical.indexed_clearance_map_builds;
  cpu.indexed_clearance_map_reuses = physical.indexed_clearance_map_reuses;
  cpu.indexed_clearance_map_cells = physical.indexed_clearance_map_cells;
  cpu.indexed_clearance_map_records = physical.indexed_clearance_map_records;
  cpu.indexed_clearance_map_references = physical.indexed_clearance_map_references;
  cpu.indexed_clearance_map_fallbacks = physical.indexed_clearance_map_fallbacks;
  cpu.indexed_clearance_map_diagnostic = std::move(physical.indexed_clearance_map_diagnostic);
  cpu.nearest_obstacle_id = std::move(physical.nearest_obstacle_id);
  cpu.walkable = prepared_input.walkable;
  cpu.obstacle_clearance = prepared_input.obstacle_clearance;
  cpu.used_gpu_clearance_field = used_gpu_clearance;
  return cpu;
}

} // namespace flux
