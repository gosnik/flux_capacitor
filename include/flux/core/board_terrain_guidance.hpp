#pragma once

#include "flux/core/board.hpp"
#include "flux/core/terrain_evolution.hpp"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <stop_token>
#include <string>
#include <vector>

namespace flux {

// A bounded board-wide, layer-local advisory field.  It deliberately stores no endpoint or
// route: callers compose demand pressure separately and retain their net-specific exact map as
// the authority for walkability and clearance.
struct BoardTerrainGuidanceConfig {
  bool enabled{true};
  bool prefer_gpu{true}; // A failed requested GPU evolution is reported; it never falls back.
  std::size_t maximum_layers{32U};
  std::size_t maximum_cells{262'144U};
  std::size_t maximum_cache_entries{8U};
  std::size_t maximum_pressure_paths{4'096U};
  std::size_t maximum_pressure_pair_inspections{8'192U};
  std::size_t maximum_pressure_expansions_per_path{16'384U};
  std::size_t maximum_pressure_expansions{262'144U};
  std::uint32_t maximum_pressure{4'096U};
  std::uint64_t terrain_cost{120U};
  std::uint64_t pressure_cost{240U};
  std::uint32_t pressure_response_scale{8U};
  TerrainEvolutionConfig evolution{};

  bool operator==(const BoardTerrainGuidanceConfig&) const = default;
};

struct BoardTerrainGuidanceLayerInput {
  LayerId layer;
  Point origin;
  Nanometres cell_pitch;
  std::size_t width{0U};
  std::size_t height{0U};
  // These are derived from the actual layer polygon/free-space extraction. Obstacles remain
  // immutable here and are never reopened by erosion or pressure.
  std::vector<std::uint8_t> walkable;
  std::vector<std::uint32_t> clearance;

  bool operator==(const BoardTerrainGuidanceLayerInput&) const = default;
};

struct BoardTerrainGuidanceDemand {
  LayerId layer;
  // Terminal cells of one actual connection/tree. Pressure is deposited on a deterministic MST
  // whose links are bounded obstacle-aware searches, rather than toward a common sink.
  std::vector<std::size_t> terminal_cells;
  std::uint32_t weight{1U};

  bool operator==(const BoardTerrainGuidanceDemand&) const = default;
};

// Immutable physical/evolved backing shared by every pressure refresh. The GPU implementation
// currently returns one CPU readback which is retained here; it is not claimed to be device-resident.
struct BoardTerrainGuidanceLayerGeometry {
  BoardTerrainGuidanceLayerInput physical;
  std::vector<double> evolved_height;
  // Immutable 8-neighbour free-space components. Blocked cells use invalid_component.
  std::vector<std::uint32_t> components;
  std::vector<std::size_t> component_cells;
  bool evolved_on_gpu{false};
  std::uint64_t evolution_time_us{0U};
};

struct BoardTerrainGuidanceLayer {
  std::shared_ptr<const BoardTerrainGuidanceLayerGeometry> geometry;
  std::vector<std::uint32_t> pressure;
};

struct BoardTerrainGuidanceStats {
  bool enabled{false};
  bool cache_reused{false};
  bool gpu_requested{false};
  bool gpu_used{false};
  std::size_t layers_built{0U};
  std::size_t cells{0U};
  std::size_t pressure_demands{0U};
  std::size_t pressure_cells{0U};
  std::size_t pressure_paths{0U};
  std::size_t pressure_pairs_inspected{0U};
  std::size_t pressure_paths_incomplete{0U};
  std::size_t disconnected_pressure_pairs{0U};
  std::size_t pressure_search_expansions{0U};
  std::size_t component_count{0U};
  std::size_t smallest_component_cells{0U};
  std::size_t largest_component_cells{0U};
  std::uint64_t connectivity_time_us{0U};
  bool pressure_budget_exhausted{false};
  std::uint64_t evolution_time_us{0U};
  std::uint64_t pressure_time_us{0U};
  std::uint64_t raster_time_us{0U};
  std::uint64_t preparation_time_us{0U};
  std::string diagnostic;
};

class BoardTerrainGuidance {
public:
  [[nodiscard]] const BoardTerrainGuidanceStats& stats() const { return stats_; }
  [[nodiscard]] const std::vector<BoardTerrainGuidanceLayer>& layers() const { return layers_; }
  // Returns a bounded non-authoritative additive cost. A false result means this shared field
  // has no sample at the point; it never makes a caller's exact legal cell illegal or legal.
  [[nodiscard]] std::uint64_t sample_cost(LayerId layer, Point point) const;
  // Returns the exact immutable shared-terrain component area at this sample, or zero for an
  // absent layer/out-of-bounds/blocked sample. This is diagnostic evidence only.
  [[nodiscard]] std::size_t reachable_component_cells(LayerId layer, Point point) const;

private:
  friend class BoardTerrainGuidanceCache;
  std::vector<BoardTerrainGuidanceLayer> layers_;
  BoardTerrainGuidanceConfig config_;
  mutable BoardTerrainGuidanceStats stats_;
};

// Full structural equality is the collision guard. The supplied revision narrows candidates but
// cannot by itself reuse a geometry field after changed physical input.
class BoardTerrainGuidanceCache {
public:
  explicit BoardTerrainGuidanceCache(std::size_t maximum_entries = 8U)
      : maximum_entries_(maximum_entries) {}
  [[nodiscard]] std::shared_ptr<const BoardTerrainGuidance> prepare(
      const std::vector<BoardTerrainGuidanceLayerInput>& layers,
      const BoardTerrainGuidanceConfig& config, std::size_t physical_revision,
      std::stop_token stop_token = {});
  [[nodiscard]] std::shared_ptr<const BoardTerrainGuidance> refresh_pressure(
      const std::shared_ptr<const BoardTerrainGuidance>& geometry,
      const std::vector<BoardTerrainGuidanceDemand>& demands, std::stop_token stop_token = {}) const;

private:
  struct Entry {
    std::size_t revision{0U};
    BoardTerrainGuidanceConfig config;
    std::vector<BoardTerrainGuidanceLayerInput> inputs;
    std::shared_ptr<const BoardTerrainGuidance> guidance;
  };
  std::size_t maximum_entries_;
  mutable std::mutex mutex_;
  std::vector<Entry> entries_;
};

} // namespace flux
