#pragma once

#include <cstddef>
#include <cstdint>
#include <stop_token>
#include <string>
#include <vector>

namespace flux {

// Bounded clearance-derived terrain evolution. Both backends are advisory corridor generators;
// CPU path extraction and downstream geometry/DRC gates remain authoritative.
struct TerrainEvolutionConfig {
  std::size_t iterations{10U};
  std::size_t maximum_cells{262'144U};
  double erosion_rate{0.06};
  double deposition_rate{0.18};
  double capacity_scale{1.0};
  double maximum_height_change{0.25};
  double terrain_cost_weight{1.0};
  double pressure_height_scale{0.05};
  double success_height_scale{0.05};
  std::uint32_t maximum_pressure{4'096U};
  std::uint32_t maximum_success_prior{4'096U};

  bool operator==(const TerrainEvolutionConfig&) const = default;
};

struct TerrainEvolutionResult {
  std::vector<double> initial_height;
  std::vector<double> height;
  std::vector<double> flow;
  std::vector<double> divergence;
  std::vector<std::size_t> path;
  std::size_t iterations_completed{0U};
  std::size_t expanded_nodes{0U};
  bool used_gpu{false};
  std::string gpu_device;
  std::size_t gpu_dispatches{0U};
  std::size_t gpu_buffer_growths{0U};
  std::size_t gpu_buffer_reuses{0U};
  std::size_t gpu_mask_uploads{0U};
  std::size_t gpu_mask_upload_reuses{0U};
  std::size_t gpu_mask_cross_pipeline_reuses{0U};
  std::uint64_t evolution_time_us{0U};
  std::uint64_t extraction_time_us{0U};
  std::string diagnostic;

  [[nodiscard]] bool ok() const { return !path.empty() && diagnostic.empty(); }
};

// Target-independent evolved terrain fields. These are advisory inputs for downstream guidance;
// they contain no route, endpoint, or path-extraction state.
struct TerrainEvolutionFields {
  std::vector<double> initial_height;
  std::vector<double> height;
  std::vector<double> flow;
  std::vector<double> divergence;
  std::size_t iterations_completed{0U};
  bool used_gpu{false};
  std::string gpu_device;
  std::size_t gpu_dispatches{0U};
  std::size_t gpu_buffer_growths{0U};
  std::size_t gpu_buffer_reuses{0U};
  std::size_t gpu_mask_uploads{0U};
  std::size_t gpu_mask_upload_reuses{0U};
  std::size_t gpu_mask_cross_pipeline_reuses{0U};
  std::uint64_t evolution_time_us{0U};
  std::string diagnostic;

  [[nodiscard]] bool ok() const { return diagnostic.empty(); }
};

[[nodiscard]] TerrainEvolutionFields evolve_terrain_fields(
    std::size_t width, std::size_t height, const std::vector<std::uint8_t>& walkable,
    const std::vector<std::uint32_t>& clearance, const TerrainEvolutionConfig& config = {},
    std::stop_token stop_token = {}, const std::vector<std::uint32_t>& pressure = {},
    const std::vector<std::uint32_t>& success_prior = {});

// GPU field evolution uses the existing Vulkan field dispatch and never falls back to CPU.
[[nodiscard]] TerrainEvolutionFields evolve_terrain_fields_gpu(
    std::size_t width, std::size_t height, const std::vector<std::uint8_t>& walkable,
    const std::vector<std::uint32_t>& clearance, const TerrainEvolutionConfig& config = {},
    std::stop_token stop_token = {}, const std::vector<std::uint32_t>& pressure = {},
    const std::vector<std::uint32_t>& success_prior = {});

[[nodiscard]] TerrainEvolutionResult solve_terrain_evolution(
    std::size_t width, std::size_t height, const std::vector<std::uint8_t>& walkable,
    const std::vector<std::uint32_t>& clearance, std::size_t source, std::size_t target,
    const TerrainEvolutionConfig& config = {}, std::stop_token stop_token = {},
    const std::vector<std::uint32_t>& pressure = {},
    const std::vector<std::uint32_t>& success_prior = {});

// GPU field evolution with the same validated inputs and CPU path extraction as the reference.
// A device or dispatch failure is reported; this entry point never silently falls back to CPU.
[[nodiscard]] TerrainEvolutionResult solve_terrain_evolution_gpu(
    std::size_t width, std::size_t height, const std::vector<std::uint8_t>& walkable,
    const std::vector<std::uint32_t>& clearance, std::size_t source, std::size_t target,
    const TerrainEvolutionConfig& config = {}, std::stop_token stop_token = {},
    const std::vector<std::uint32_t>& pressure = {},
    const std::vector<std::uint32_t>& success_prior = {});

} // namespace flux
