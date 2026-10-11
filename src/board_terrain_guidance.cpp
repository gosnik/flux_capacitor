#include "flux/core/board_terrain_guidance.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstddef>
#include <limits>
#include <queue>
#include <set>

namespace flux {
namespace {
constexpr std::uint32_t invalid_component = std::numeric_limits<std::uint32_t>::max();
[[nodiscard]] bool valid_input(const BoardTerrainGuidanceLayerInput& input,
                               const BoardTerrainGuidanceConfig& config) {
  return input.width != 0U && input.height != 0U && input.width <= config.maximum_cells / input.height &&
         input.width * input.height <= config.maximum_cells && input.cell_pitch.value() > 0 &&
         input.walkable.size() == input.width * input.height &&
         input.clearance.size() == input.width * input.height;
}

void add_pressure(BoardTerrainGuidanceLayer& layer, std::size_t cell, std::uint32_t weight,
                  BoardTerrainGuidanceStats& stats, std::uint32_t maximum) {
  if (cell >= layer.pressure.size() || layer.geometry->physical.walkable[cell] == 0U) return;
  const auto old = layer.pressure[cell];
  layer.pressure[cell] = static_cast<std::uint32_t>(std::min<std::uint64_t>(
      maximum, static_cast<std::uint64_t>(old) + weight));
  if (old == 0U && layer.pressure[cell] != 0U) ++stats.pressure_cells;
}

bool label_components(BoardTerrainGuidanceLayerGeometry& geometry, BoardTerrainGuidanceStats& stats,
                      std::stop_token stop_token) {
  const auto started = std::chrono::steady_clock::now();
  const auto width = geometry.physical.width, height = geometry.physical.height;
  geometry.components.assign(width * height, invalid_component);
  for (std::size_t seed = 0U; seed < geometry.components.size(); ++seed) {
    if ((seed & 1023U) == 0U && stop_token.stop_requested()) return false;
    if (geometry.physical.walkable[seed] == 0U || geometry.components[seed] != invalid_component) continue;
    const auto id = static_cast<std::uint32_t>(geometry.component_cells.size());
    std::vector<std::size_t> queue{seed}; geometry.components[seed] = id; std::size_t cells = 0U;
    while (!queue.empty()) {
      const auto cell = queue.back(); queue.pop_back(); ++cells;
      const auto x = cell % width, y = cell / width;
      for (int dy = -1; dy <= 1; ++dy) for (int dx = -1; dx <= 1; ++dx) {
        if (dx == 0 && dy == 0) continue;
        const auto nx = static_cast<std::ptrdiff_t>(x) + dx, ny = static_cast<std::ptrdiff_t>(y) + dy;
        if (nx < 0 || ny < 0 || nx >= static_cast<std::ptrdiff_t>(width) || ny >= static_cast<std::ptrdiff_t>(height)) continue;
        const auto next = static_cast<std::size_t>(ny) * width + static_cast<std::size_t>(nx);
        if (geometry.physical.walkable[next] == 0U || geometry.components[next] != invalid_component) continue;
        if (dx != 0 && dy != 0 &&
            (geometry.physical.walkable[y * width + static_cast<std::size_t>(nx)] == 0U ||
             geometry.physical.walkable[static_cast<std::size_t>(ny) * width + x] == 0U)) continue;
        geometry.components[next] = id; queue.push_back(next);
      }
      if ((cells & 1023U) == 0U && stop_token.stop_requested()) return false;
    }
    geometry.component_cells.push_back(cells);
  }
  stats.component_count += geometry.component_cells.size();
  for (const auto cells : geometry.component_cells) {
    stats.smallest_component_cells = stats.smallest_component_cells == 0U ? cells : std::min(stats.smallest_component_cells, cells);
    stats.largest_component_cells = std::max(stats.largest_component_cells, cells);
  }
  stats.connectivity_time_us += static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - started).count());
  return true;
}

bool deposit_connection_path(BoardTerrainGuidanceLayer& layer, std::size_t first, std::size_t last,
                             std::uint32_t weight, BoardTerrainGuidanceStats& stats,
                             const BoardTerrainGuidanceConfig& config, std::stop_token stop_token) {
  if (first >= layer.pressure.size() || last >= layer.pressure.size() ||
      layer.geometry->physical.walkable[first] == 0U || layer.geometry->physical.walkable[last] == 0U ||
      stats.pressure_paths >= config.maximum_pressure_paths ||
      stats.pressure_search_expansions >= config.maximum_pressure_expansions || stop_token.stop_requested()) return false;
  ++stats.pressure_pairs_inspected;
  if (layer.geometry->components[first] != layer.geometry->components[last]) {
    ++stats.disconnected_pressure_pairs;
    return false;
  }
  ++stats.pressure_paths;
  const auto width = layer.geometry->physical.width;
  const auto count = layer.pressure.size();
  using Item = std::pair<double, std::size_t>;
  std::priority_queue<Item, std::vector<Item>, std::greater<>> queue;
  std::vector<double> distance(count, std::numeric_limits<double>::infinity());
  std::vector<std::size_t> predecessor(count, count);
  distance[first] = 0.0; queue.emplace(0.0, first); std::size_t expanded = 0U;
  bool settled = false;
  while (!queue.empty() && expanded < config.maximum_pressure_expansions_per_path &&
         stats.pressure_search_expansions < config.maximum_pressure_expansions &&
         !stop_token.stop_requested()) {
    const auto [cost, cell] = queue.top(); queue.pop();
    if (cost != distance[cell]) continue;
    ++expanded; ++stats.pressure_search_expansions;
    if (cell == last) { settled = true; break; }
    const auto x = cell % width, y = cell / width;
    for (int dy = -1; dy <= 1; ++dy) for (int dx = -1; dx <= 1; ++dx) {
      if (dx == 0 && dy == 0) continue;
      const auto nx = static_cast<std::ptrdiff_t>(x) + dx, ny = static_cast<std::ptrdiff_t>(y) + dy;
      if (nx < 0 || ny < 0 || nx >= static_cast<std::ptrdiff_t>(width) ||
          ny >= static_cast<std::ptrdiff_t>(layer.geometry->physical.height)) continue;
      const auto next = static_cast<std::size_t>(ny) * width + static_cast<std::size_t>(nx);
      if (layer.geometry->physical.walkable[next] == 0U) continue;
      if (dx != 0 && dy != 0 &&
          (layer.geometry->physical.walkable[y * width + static_cast<std::size_t>(nx)] == 0U ||
           layer.geometry->physical.walkable[static_cast<std::size_t>(ny) * width + x] == 0U)) continue;
      const auto height = next < layer.geometry->evolved_height.size() ? std::max(0.0, layer.geometry->evolved_height[next]) : 0.0;
      const auto candidate = cost + (dx == 0 || dy == 0 ? 1.0 : 1.41421356237) * (1.0 + height);
      if (candidate < distance[next]) { distance[next] = candidate; predecessor[next] = cell; queue.emplace(candidate, next); }
    }
  }
  if (!settled) {
    ++stats.pressure_paths_incomplete;
    if (!queue.empty() && (expanded >= config.maximum_pressure_expansions_per_path ||
        stats.pressure_search_expansions >= config.maximum_pressure_expansions))
      stats.pressure_budget_exhausted = true;
    return false;
  }
  for (auto cell = last;; cell = predecessor[cell]) {
    add_pressure(layer, cell, weight, stats, config.maximum_pressure);
    if (cell == first) break;
  }
  return true;
}
} // namespace

std::uint64_t BoardTerrainGuidance::sample_cost(LayerId layer, Point point) const {
  if (!config_.enabled) return 0U;
  const auto found = std::find_if(layers_.begin(), layers_.end(), [&](const auto& value) {
    return value.geometry->physical.layer == layer;
  });
  if (found == layers_.end()) return 0U;
  const auto& field = *found;
  const auto pitch = field.geometry->physical.cell_pitch.value();
  const auto origin = field.geometry->physical.origin;
  if (point.x < origin.x || point.y < origin.y) return 0U;
  // Unsigned subtraction represents the full nonnegative int64 coordinate difference.
  const auto dx = static_cast<std::uint64_t>(point.x.value()) - static_cast<std::uint64_t>(origin.x.value());
  const auto dy = static_cast<std::uint64_t>(point.y.value()) - static_cast<std::uint64_t>(origin.y.value());
  const auto x = dx / static_cast<std::uint64_t>(pitch), y = dy / static_cast<std::uint64_t>(pitch);
  if (x >= field.geometry->physical.width || y >= field.geometry->physical.height) return 0U;
  const auto index = y * field.geometry->physical.width + x;
  if (field.geometry->physical.walkable[index] == 0U) return 0U;
  const auto height = index < field.geometry->evolved_height.size() && std::isfinite(field.geometry->evolved_height[index])
                          ? std::max(0.0, field.geometry->evolved_height[index]) : 0.0;
  const auto terrain = static_cast<std::uint64_t>(std::min<long double>(
      std::numeric_limits<std::uint64_t>::max() / 4U,
      static_cast<long double>(config_.terrain_cost) * height));
  const auto normalized_pressure =
      std::min<std::uint32_t>(field.pressure[index], config_.maximum_pressure);
  const auto pressure = normalized_pressure == 0U ? 0U :
      config_.pressure_cost > std::numeric_limits<std::uint64_t>::max() /
                                  static_cast<std::uint64_t>(normalized_pressure)
          ? std::numeric_limits<std::uint64_t>::max() / 2U
          : config_.pressure_cost * static_cast<std::uint64_t>(normalized_pressure) /
                (static_cast<std::uint64_t>(normalized_pressure) +
                 std::max(1U, config_.pressure_response_scale));
  return std::min<std::uint64_t>(std::numeric_limits<std::uint64_t>::max() / 2U, terrain + pressure);
}

std::size_t BoardTerrainGuidance::reachable_component_cells(LayerId layer, Point point) const {
  const auto found = std::find_if(layers_.begin(), layers_.end(), [&](const auto& value) {
    return value.geometry->physical.layer == layer;
  });
  if (found == layers_.end()) return 0U;
  const auto& geometry = *found->geometry;
  if (point.x < geometry.physical.origin.x || point.y < geometry.physical.origin.y) return 0U;
  const auto x = (static_cast<std::uint64_t>(point.x.value()) - static_cast<std::uint64_t>(geometry.physical.origin.x.value())) /
                 static_cast<std::uint64_t>(geometry.physical.cell_pitch.value());
  const auto y = (static_cast<std::uint64_t>(point.y.value()) - static_cast<std::uint64_t>(geometry.physical.origin.y.value())) /
                 static_cast<std::uint64_t>(geometry.physical.cell_pitch.value());
  if (x >= geometry.physical.width || y >= geometry.physical.height) return 0U;
  const auto cell = y * geometry.physical.width + x;
  if (geometry.physical.walkable[cell] == 0U || cell >= geometry.components.size()) return 0U;
  const auto component = geometry.components[cell];
  return component == invalid_component || component >= geometry.component_cells.size()
             ? 0U : geometry.component_cells[component];
}

std::shared_ptr<const BoardTerrainGuidance> BoardTerrainGuidanceCache::prepare(
    const std::vector<BoardTerrainGuidanceLayerInput>& inputs, const BoardTerrainGuidanceConfig& config,
    std::size_t physical_revision, std::stop_token stop_token) {
  if (!config.enabled) return std::make_shared<BoardTerrainGuidance>();
  {
    std::scoped_lock lock(mutex_);
    const auto hit = std::find_if(entries_.begin(), entries_.end(), [&](const Entry& entry) {
      return entry.revision == physical_revision && entry.config == config && entry.inputs == inputs;
    });
    if (hit != entries_.end()) {
      // Copy only layer handles and the small stats envelope. Geometry backing is immutable.
      auto reused = std::make_shared<BoardTerrainGuidance>(*hit->guidance);
      reused->stats_.cache_reused = true;
      return reused;
    }
  }
  auto result = std::make_shared<BoardTerrainGuidance>();
  result->config_ = config;
  result->stats_.enabled = true;
  result->stats_.gpu_requested = config.prefer_gpu;
  if (inputs.size() > config.maximum_layers) {
    result->stats_.diagnostic = "board terrain layer limit exceeded";
    return result;
  }
  std::set<LayerId> distinct_layers;
  for (const auto& input : inputs) {
    if (!distinct_layers.insert(input.layer).second) {
      result->stats_.diagnostic = "duplicate board terrain layer";
      return result;
    }
    if (stop_token.stop_requested()) { result->stats_.diagnostic = "board terrain build cancelled"; return result; }
    if (!valid_input(input, config) || result->stats_.cells > config.maximum_cells - input.width * input.height) {
      result->stats_.diagnostic = "board terrain cell limit or input invalid";
      return result;
    }
    auto evolved = config.prefer_gpu
                       ? evolve_terrain_fields_gpu(input.width, input.height, input.walkable, input.clearance,
                                                   config.evolution, stop_token)
                       : evolve_terrain_fields(input.width, input.height, input.walkable, input.clearance,
                                               config.evolution, stop_token);
    if (!evolved.ok()) { result->stats_.diagnostic = evolved.diagnostic; return result; }
    result->stats_.gpu_used = result->stats_.gpu_used || evolved.used_gpu;
    result->stats_.evolution_time_us += evolved.evolution_time_us;
    result->stats_.cells += input.width * input.height;
    BoardTerrainGuidanceLayerGeometry backing{input, std::move(evolved.height), {}, {},
                                                evolved.used_gpu, evolved.evolution_time_us};
    if (!label_components(backing, result->stats_, stop_token)) {
      result->stats_.diagnostic = "board terrain connectivity build cancelled";
      return result;
    }
    result->layers_.push_back(BoardTerrainGuidanceLayer{
        std::make_shared<const BoardTerrainGuidanceLayerGeometry>(std::move(backing)),
        std::vector<std::uint32_t>(input.width * input.height, 0U)});
  }
  result->stats_.layers_built = result->layers_.size();
  std::scoped_lock lock(mutex_);
  entries_.push_back(Entry{physical_revision, config, inputs, result});
  const auto maximum = std::min(maximum_entries_, config.maximum_cache_entries);
  if (maximum == 0U) entries_.clear();
  else if (entries_.size() > maximum) entries_.erase(entries_.begin(), entries_.begin() + static_cast<std::ptrdiff_t>(entries_.size() - maximum));
  return result;
}

std::shared_ptr<const BoardTerrainGuidance> BoardTerrainGuidanceCache::refresh_pressure(
    const std::shared_ptr<const BoardTerrainGuidance>& geometry,
    const std::vector<BoardTerrainGuidanceDemand>& demands, std::stop_token stop_token) const {
  if (!geometry) return {};
  auto refreshed = std::make_shared<BoardTerrainGuidance>(*geometry);
  for (auto& layer : refreshed->layers_) std::fill(layer.pressure.begin(), layer.pressure.end(), 0U);
  refreshed->stats_.pressure_demands = demands.size();
  refreshed->stats_.pressure_cells = 0U;
  refreshed->stats_.pressure_paths = 0U;
  refreshed->stats_.pressure_pairs_inspected = 0U;
  refreshed->stats_.pressure_paths_incomplete = 0U;
  refreshed->stats_.disconnected_pressure_pairs = 0U;
  refreshed->stats_.pressure_search_expansions = 0U;
  refreshed->stats_.pressure_time_us = 0U;
  refreshed->stats_.pressure_budget_exhausted = false;
  const auto pressure_started = std::chrono::steady_clock::now();
  for (const auto& demand : demands) {
    if (refreshed->stats_.pressure_paths >= refreshed->config_.maximum_pressure_paths ||
        refreshed->stats_.pressure_pairs_inspected >= refreshed->config_.maximum_pressure_pair_inspections ||
        refreshed->stats_.pressure_search_expansions >= refreshed->config_.maximum_pressure_expansions) {
      refreshed->stats_.pressure_budget_exhausted = true;
      break;
    }
    if (stop_token.stop_requested()) { refreshed->stats_.diagnostic = "board terrain pressure cancelled"; break; }
    const auto layer = std::find_if(refreshed->layers_.begin(), refreshed->layers_.end(), [&](const auto& value) {
      return value.geometry->physical.layer == demand.layer;
    });
    if (layer == refreshed->layers_.end() || demand.terminal_cells.size() < 2U) continue;
    // Prim: connect every terminal to the already grown tree using deterministic Manhattan cost.
    std::vector<std::size_t> nodes;
    nodes.reserve(std::min<std::size_t>(demand.terminal_cells.size(), 64U));
    for (const auto cell : demand.terminal_cells) {
      if (cell >= layer->pressure.size() || nodes.size() >= 64U) continue;
      if (layer->geometry->physical.walkable[cell] != 0U) { nodes.push_back(cell); continue; }
      // Pad centres are commonly blocked in the all-owner shared terrain. Bind only a nearby
      // advisory free-space representative; candidate-specific terminal escape remains exact.
      const auto x = cell % layer->geometry->physical.width, y = cell / layer->geometry->physical.width;
      std::optional<std::size_t> access;
      for (std::ptrdiff_t ring = 1; ring <= 3 && !access; ++ring) {
        for (std::ptrdiff_t dy = -ring; dy <= ring && !access; ++dy) for (std::ptrdiff_t dx = -ring; dx <= ring; ++dx) {
          if (std::max(std::abs(dx), std::abs(dy)) != ring) continue;
          const auto nx = static_cast<std::ptrdiff_t>(x) + dx, ny = static_cast<std::ptrdiff_t>(y) + dy;
          if (nx >= 0 && ny >= 0 && nx < static_cast<std::ptrdiff_t>(layer->geometry->physical.width) &&
              ny < static_cast<std::ptrdiff_t>(layer->geometry->physical.height)) {
            const auto candidate = static_cast<std::size_t>(ny) * layer->geometry->physical.width + static_cast<std::size_t>(nx);
            if (layer->geometry->physical.walkable[candidate] != 0U) access = candidate;
          }
        }
      }
      if (access) nodes.push_back(*access);
    }
    std::sort(nodes.begin(), nodes.end()); nodes.erase(std::unique(nodes.begin(), nodes.end()), nodes.end());
    if (nodes.size() < 2U) continue;
    std::vector<std::uint8_t> joined(nodes.size(), 0U); joined[0] = 1U;
    for (std::size_t joined_count = 1U; joined_count < nodes.size(); ++joined_count) {
      if (refreshed->stats_.pressure_paths >= refreshed->config_.maximum_pressure_paths ||
        refreshed->stats_.pressure_pairs_inspected >= refreshed->config_.maximum_pressure_pair_inspections ||
          refreshed->stats_.pressure_search_expansions >= refreshed->config_.maximum_pressure_expansions) {
        refreshed->stats_.pressure_budget_exhausted = true;
        break;
      }
      if (stop_token.stop_requested()) break;
      std::optional<std::pair<std::size_t, std::size_t>> best;
      for (std::size_t a = 0U; a < nodes.size(); ++a) if (joined[a]) for (std::size_t b = 0U; b < nodes.size(); ++b) if (!joined[b]) {
        const auto ax = nodes[a] % layer->geometry->physical.width, ay = nodes[a] / layer->geometry->physical.width;
        const auto bx = nodes[b] % layer->geometry->physical.width, by = nodes[b] / layer->geometry->physical.width;
        const auto distance = std::llabs(static_cast<long long>(ax) - static_cast<long long>(bx)) + std::llabs(static_cast<long long>(ay) - static_cast<long long>(by));
        const auto candidate = std::pair{static_cast<std::size_t>(distance), b * nodes.size() + a};
        if (!best || candidate < *best) best = candidate;
      }
      if (!best) break;
      const auto b = best->second / nodes.size(), a = best->second % nodes.size();
      static_cast<void>(deposit_connection_path(*layer, nodes[a], nodes[b], demand.weight,
                                                refreshed->stats_, refreshed->config_, stop_token));
      joined[b] = 1U;
    }
  }
  refreshed->stats_.pressure_time_us = static_cast<std::uint64_t>(
      std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - pressure_started).count());
  return refreshed;
}
} // namespace flux
