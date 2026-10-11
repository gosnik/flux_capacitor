#include "flux/core/terrain_evolution.hpp"
#include "flux/core/hydraulic_path.hpp"
#include "../../src/vulkan_runtime.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <future>
#include <iomanip>
#include <iostream>
#include <memory>
#include <numeric>
#include <stop_token>
#include <string_view>
#include <thread>
#include <vector>

namespace {

void require(bool value, const char* message) {
  if (!value) {
    std::cerr << message << '\n';
    std::exit(1);
  }
}

[[nodiscard]] std::size_t at(std::size_t width, std::size_t x, std::size_t y) {
  return y * width + x;
}

struct TerrainInput {
  std::size_t width{0U};
  std::size_t height{0U};
  std::vector<std::uint8_t> walkable;
  std::vector<std::uint32_t> clearance;
  std::vector<std::uint32_t> pressure;
  std::vector<std::uint32_t> success;
  std::size_t source{0U};
  std::size_t target{0U};
};

[[nodiscard]] TerrainInput representative_input(std::size_t width, std::size_t height) {
  TerrainInput input;
  input.width = width;
  input.height = height;
  input.walkable.assign(width * height, 1U);
  input.clearance.resize(width * height);
  input.pressure.resize(width * height);
  input.success.resize(width * height);
  for (std::size_t y = 0U; y < height; ++y) {
    for (std::size_t x = 0U; x < width; ++x) {
      const auto cell = at(width, x, y);
      input.clearance[cell] = static_cast<std::uint32_t>(1U + ((x * 17U + y * 11U) % 31U));
      input.pressure[cell] = static_cast<std::uint32_t>((x * 7U + y * 13U) % 19U);
      input.success[cell] = static_cast<std::uint32_t>((x * 5U + y * 3U) % 17U);
    }
  }
  // Two offset barriers force a corridor choice while retaining a legal, non-diagonal route.
  for (std::size_t y = 3U; y + 3U < height; ++y) {
    if (y % 5U != 2U)
      input.walkable[at(width, width / 3U, y)] = 0U;
    if (y % 7U != 4U)
      input.walkable[at(width, 2U * width / 3U, y)] = 0U;
  }
  input.source = at(width, 1U, 1U);
  input.target = at(width, width - 2U, height - 2U);
  input.walkable[input.source] = 1U;
  input.walkable[input.target] = 1U;
  return input;
}

[[nodiscard]] flux::TerrainEvolutionConfig test_config() {
  flux::TerrainEvolutionConfig config;
  config.iterations = 12U;
  config.maximum_cells = 1'048'576U;
  config.pressure_height_scale = 0.14;
  config.success_height_scale = 0.09;
  return config;
}

[[nodiscard]] flux::TerrainEvolutionResult cpu(const TerrainInput& input,
                                                const flux::TerrainEvolutionConfig& config) {
  return flux::solve_terrain_evolution(input.width, input.height, input.walkable, input.clearance,
                                       input.source, input.target, config, {}, input.pressure,
                                       input.success);
}

[[nodiscard]] flux::TerrainEvolutionResult gpu(const TerrainInput& input,
                                                const flux::TerrainEvolutionConfig& config) {
  return flux::solve_terrain_evolution_gpu(input.width, input.height, input.walkable,
                                           input.clearance, input.source, input.target, config, {},
                                           input.pressure, input.success);
}

void require_gpu(const flux::TerrainEvolutionResult& result, const char* context,
                 bool require_dispatch = true) {
  require(result.ok(), context);
  require(result.used_gpu, "GPU terrain request must not silently fall back to CPU");
  require(!result.gpu_device.empty(), "GPU terrain result must identify its Vulkan device");
  if (require_dispatch)
    require(result.gpu_dispatches > 0U, "GPU terrain result must report a Vulkan dispatch");
}

[[nodiscard]] double max_relative_error(const std::vector<double>& expected,
                                        const std::vector<double>& actual) {
  require(expected.size() == actual.size(), "CPU and GPU field sizes must match");
  double maximum = 0.0;
  for (std::size_t index = 0U; index < expected.size(); ++index) {
    const auto denominator = std::max(1.0, std::abs(expected[index]));
    maximum = std::max(maximum, std::abs(expected[index] - actual[index]) / denominator);
  }
  return maximum;
}

void require_parity(const flux::TerrainEvolutionResult& reference,
                    const flux::TerrainEvolutionResult& accelerated, const char* context) {
  require(reference.ok(), "CPU terrain reference must solve the fixture");
  require_gpu(accelerated, context);
  constexpr double fp32_relative_error = 1e-4;
  const auto height_error = max_relative_error(reference.height, accelerated.height);
  const auto flow_error = max_relative_error(reference.flow, accelerated.flow);
  const auto divergence_error = max_relative_error(reference.divergence, accelerated.divergence);
  if (height_error > fp32_relative_error || flow_error > fp32_relative_error ||
      divergence_error > fp32_relative_error) {
    std::cerr << context << ": height=" << height_error << " flow=" << flow_error
              << " divergence=" << divergence_error << '\n';
    std::exit(1);
  }
  require(reference.path == accelerated.path,
          "CPU and GPU terrain must retain the same deterministic extracted path");
}

void require_gpu_fields(const flux::TerrainEvolutionFields& fields, const char* context) {
  require(fields.ok(), context);
  require(fields.used_gpu, "GPU terrain-field request must not silently fall back to CPU");
  require(!fields.gpu_device.empty(), "GPU terrain fields must identify their Vulkan device");
  require(fields.gpu_dispatches > 0U, "GPU terrain fields must report a Vulkan dispatch");
}

void check_field_only_parity(const TerrainInput& input,
                             const flux::TerrainEvolutionConfig& config) {
  const auto reference = flux::evolve_terrain_fields(
      input.width, input.height, input.walkable, input.clearance, config, {}, input.pressure,
      input.success);
  const auto accelerated = flux::evolve_terrain_fields_gpu(
      input.width, input.height, input.walkable, input.clearance, config, {}, input.pressure,
      input.success);
  require(reference.ok(), "CPU terrain fields must evolve the representative fixture");
  require_gpu_fields(accelerated, "GPU terrain fields must evolve the representative fixture");
  constexpr double fp32_relative_error = 1e-4;
  const auto initial_error = max_relative_error(reference.initial_height, accelerated.initial_height);
  const auto height_error = max_relative_error(reference.height, accelerated.height);
  const auto flow_error = max_relative_error(reference.flow, accelerated.flow);
  const auto divergence_error = max_relative_error(reference.divergence, accelerated.divergence);
  require(initial_error <= fp32_relative_error && height_error <= fp32_relative_error &&
              flow_error <= fp32_relative_error && divergence_error <= fp32_relative_error,
          "target-independent GPU terrain fields must match CPU field evolution");

  const auto solved = gpu(input, config);
  require_gpu(solved, "GPU terrain solve must share the representative field fixture");
  require(max_relative_error(accelerated.initial_height, solved.initial_height) <= fp32_relative_error &&
              max_relative_error(accelerated.height, solved.height) <= fp32_relative_error &&
              max_relative_error(accelerated.flow, solved.flow) <= fp32_relative_error &&
              max_relative_error(accelerated.divergence, solved.divergence) <= fp32_relative_error,
          "field-only GPU evolution must match the GPU solve field for identical pressure and success input");
  require(accelerated.iterations_completed == solved.iterations_completed,
          "field-only GPU evolution must report the same completed iterations as GPU solve");
}

void check_corner_cut_and_disconnection(const flux::TerrainEvolutionConfig& config) {
  const std::vector<std::uint8_t> diagonal_only{1U, 0U, 0U, 1U};
  const std::vector<std::uint32_t> clearance(4U, 1U);
  const auto diagonal = flux::solve_terrain_evolution_gpu(2U, 2U, diagonal_only, clearance, 0U,
                                                           3U, config);
  require(!diagonal.ok() && diagonal.used_gpu && diagonal.gpu_dispatches > 0U,
          "GPU evolution must not invent a diagonal-only corner-cut path");

  TerrainInput neck;
  neck.width = 5U;
  neck.height = 5U;
  neck.walkable.assign(25U, 1U);
  neck.clearance.assign(25U, 3U);
  neck.pressure.assign(25U, 0U);
  neck.success.assign(25U, 0U);
  for (std::size_t x = 0U; x < neck.width; ++x)
    if (x != 2U)
      neck.walkable[at(neck.width, x, 2U)] = 0U;
  neck.source = at(neck.width, 2U, 0U);
  neck.target = at(neck.width, 2U, 4U);
  const auto result = gpu(neck, config);
  require_gpu(result, "masked neck fixture must execute on GPU");
  require(std::find(result.path.begin(), result.path.end(), at(neck.width, 2U, 2U)) !=
              result.path.end(),
          "GPU path must use the only legal masked neck");
}

void check_branched_flow(const flux::TerrainEvolutionConfig& base) {
  constexpr std::size_t width = 7U;
  constexpr std::size_t height = 5U;
  TerrainInput branch;
  branch.width = width;
  branch.height = height;
  branch.walkable.assign(width * height, 0U);
  branch.clearance.assign(width * height, 0U);
  branch.pressure.assign(width * height, 0U);
  branch.success.assign(width * height, 0U);
  const auto open = [&](std::size_t x, std::size_t y, std::uint32_t clearance) {
    const auto cell = at(width, x, y);
    branch.walkable[cell] = 1U;
    branch.clearance[cell] = clearance;
  };
  // Each side branch reaches the merge only through orthogonal steps.  Its diagonal shortcuts
  // are blocked by the corresponding missing side cell, so the expected accumulation is exact.
  open(0U, 0U, 0U); open(3U, 0U, 0U); open(6U, 0U, 0U);
  open(0U, 1U, 1U); open(3U, 1U, 1U); open(6U, 1U, 1U);
  open(0U, 2U, 2U); open(1U, 2U, 3U); open(2U, 2U, 4U);
  open(3U, 2U, 5U); open(4U, 2U, 4U); open(5U, 2U, 3U); open(6U, 2U, 2U);
  open(3U, 3U, 8U); open(3U, 4U, 16U);
  branch.source = at(width, 0U, 0U);
  branch.target = at(width, 3U, 4U);
  auto config = base;
  config.iterations = 1U;
  config.erosion_rate = 0.0;
  config.deposition_rate = 0.0;
  const auto reference = cpu(branch, config);
  const auto accelerated = gpu(branch, config);
  require_parity(reference, accelerated, "branched downhill field must agree");
  const auto merge = at(width, 3U, 2U);
  require(reference.flow[merge] == 13.0 && accelerated.flow[merge] == 13.0,
          "branched downhill flow accumulation must remain exact integer arithmetic");
  const auto conservation = std::accumulate(accelerated.divergence.begin(),
                                            accelerated.divergence.end(), 0.0);
  double transport_scale = 0.0;
  for (const auto value : accelerated.divergence)
    transport_scale += std::abs(value);
  require(std::abs(conservation) <= std::max(1.0, transport_scale) * 1e-6,
          "GPU finite-volume transport must conserve total divergence");
}

void check_zero_and_cancel(const TerrainInput& input, const flux::TerrainEvolutionConfig& base) {
  auto zero = base;
  zero.iterations = 0U;
  const auto result = gpu(input, zero);
  require(result.ok() && !result.used_gpu && result.gpu_dispatches == 0U,
          "zero-iteration terrain must not claim a Vulkan dispatch");
  require(result.height == result.initial_height,
          "zero-iteration GPU terrain must preserve the initial height field");

  std::stop_source stop;
  stop.request_stop();
  const auto cancelled = flux::solve_terrain_evolution_gpu(
      input.width, input.height, input.walkable, input.clearance, input.source, input.target, base,
      stop.get_token(), input.pressure, input.success);
  require(!cancelled.ok() && !cancelled.diagnostic.empty() && !cancelled.used_gpu,
          "pre-cancelled GPU terrain request must fail closed before dispatch");
}

void check_invalid_endpoints_skip_gpu_evolution(const TerrainInput& input,
                                                const flux::TerrainEvolutionConfig& config) {
  const auto invalid_source = flux::solve_terrain_evolution_gpu(
      input.width, input.height, input.walkable, input.clearance, input.walkable.size(), input.target,
      config, {}, input.pressure, input.success);
  auto blocked = input;
  blocked.walkable[blocked.target] = 0U;
  const auto blocked_target = gpu(blocked, config);
  for (const auto* result : {&invalid_source, &blocked_target}) {
    require(!result->ok() && !result->used_gpu && result->gpu_dispatches == 0U &&
                result->iterations_completed == 0U && result->height.empty() &&
                result->evolution_time_us == 0U,
            "invalid source or blocked endpoint must fail before GPU terrain evolution");
  }
}

void check_long_downhill_chain(const flux::TerrainEvolutionConfig& base) {
  constexpr std::size_t length = 1024U;
  TerrainInput chain;
  chain.width = length;
  chain.height = 1U;
  chain.walkable.assign(length, 1U);
  chain.clearance.resize(length);
  chain.pressure.assign(length, 0U);
  chain.success.assign(length, 0U);
  for (std::size_t index = 0U; index < length; ++index)
    chain.clearance[index] = static_cast<std::uint32_t>(index);
  chain.source = 0U;
  chain.target = length - 1U;
  auto config = base;
  config.iterations = 1U;
  config.erosion_rate = 0.0;
  config.deposition_rate = 0.0;
  const auto reference = cpu(chain, config);
  const auto accelerated = gpu(chain, config);
  require_parity(reference, accelerated,
                 "long downhill chain must agree after GPU pointer-doubling flow accumulation");
  require(reference.path.size() == length && accelerated.path.size() == length &&
              reference.flow.back() == static_cast<double>(length) &&
              accelerated.flow.back() == static_cast<double>(length),
          "long downhill chain must accumulate every upstream cell exactly");
}

void check_production_hydraulic_wrapper(const TerrainInput& terrain,
                                        const flux::TerrainEvolutionConfig& evolution) {
  flux::HydraulicPathInput input;
  input.width = terrain.width;
  input.height = terrain.height;
  input.walkable = terrain.walkable;
  input.obstacle_clearance = terrain.clearance;
  input.downstream_pressure = terrain.pressure;
  input.success_prior = terrain.success;
  input.source = terrain.source;
  input.target = terrain.target;
  flux::HydraulicPathConfig config;
  config.evolve_terrain_height = true;
  config.prefer_gpu = true;
  config.terrain_evolution = evolution;
  const auto result = flux::solve_hydraulic_erosion_path(input, config);
  require(result.ok() && result.used_height_evolution && result.height_evolution_invoked &&
              result.used_gpu && result.height_evolution_gpu_invoked &&
              result.height_evolution_gpu_dispatches > 0U,
          "production hydraulic terrain mode must invoke and report the GPU evolution backend");
}

void check_cross_pipeline_mask_sharing() {
  const auto input = representative_input(47U, 35U);
  auto terrain_config = test_config();
  terrain_config.iterations = 2U;
  const auto terrain_first = gpu(input, terrain_config);
  require_gpu(terrain_first, "terrain must publish a shared immutable mask");

  flux::HydraulicPathInput hydraulic_input;
  hydraulic_input.width = input.width;
  hydraulic_input.height = input.height;
  hydraulic_input.walkable = input.walkable;
  hydraulic_input.obstacle_clearance = input.clearance;
  hydraulic_input.downstream_pressure = input.pressure;
  hydraulic_input.success_prior = input.success;
  hydraulic_input.source = input.source;
  hydraulic_input.target = input.target;
  flux::HydraulicPathConfig hydraulic_config;
  hydraulic_config.relaxation_iterations = 128U;
  hydraulic_config.droplet_batches = 1U;
  hydraulic_config.droplets_per_batch = 128U;
  hydraulic_config.maximum_steps_per_droplet = 128U;
  auto hydraulic_cpu_config = hydraulic_config;
  hydraulic_cpu_config.prefer_gpu = false;
  const auto hydraulic_cpu = flux::solve_hydraulic_erosion_path(hydraulic_input, hydraulic_cpu_config);
  const auto hydraulic_first = flux::solve_hydraulic_erosion_path(hydraulic_input, hydraulic_config);
  require(hydraulic_cpu.ok() && hydraulic_first.ok() && hydraulic_first.used_gpu,
          "hydraulic must consume the shared immutable mask on GPU");
  require(hydraulic_first.riverbed == hydraulic_cpu.riverbed &&
              hydraulic_first.static_potential == hydraulic_cpu.static_potential,
          "shared hydraulic mask must preserve CPU-equivalent routing fields");
  require(hydraulic_first.gpu_mask_upload_reuses == 1U &&
              hydraulic_first.gpu_mask_cross_pipeline_reuses == 1U,
          "hydraulic must reuse terrain's exact immutable mask lease");

  const auto terrain_second = gpu(input, terrain_config);
  require_parity(cpu(input, terrain_config), terrain_second,
                 "shared mask reuse must preserve terrain CPU parity");
  require(terrain_second.gpu_mask_upload_reuses == 1U &&
              terrain_second.gpu_mask_cross_pipeline_reuses == 1U,
          "terrain must reuse hydraulic's exact immutable mask lease");

  auto changed = hydraulic_input;
  changed.walkable[at(changed.width, changed.width / 2U, changed.height / 2U)] ^= 1U;
  const auto changed_result = flux::solve_hydraulic_erosion_path(changed, hydraulic_config);
  require(changed_result.ok() && changed_result.used_gpu && changed_result.gpu_mask_uploads == 1U &&
              changed_result.gpu_mask_upload_reuses == 0U,
          "changed mask contents must not alias an immutable GPU cache entry");
  auto resized = hydraulic_input;
  resized.width = 41U;
  resized.height = 40U;
  resized.walkable.assign(resized.width * resized.height, 1U);
  resized.obstacle_clearance.assign(resized.walkable.size(), 1U);
  resized.downstream_pressure.assign(resized.walkable.size(), 0U);
  resized.success_prior.assign(resized.walkable.size(), 0U);
  resized.source = 0U;
  resized.target = resized.walkable.size() - 1U;
  const auto resized_result = flux::solve_hydraulic_erosion_path(resized, hydraulic_config);
  require(resized_result.ok() && resized_result.used_gpu && resized_result.gpu_mask_uploads == 1U,
          "changed mask dimensions must not alias an immutable GPU cache entry");
}

void check_runtime_mask_lease_bounds_and_concurrency() {
  auto& runtime = flux::vulkan::Runtime::instance();
  require(runtime.ok(), "runtime lease test requires Vulkan availability");
  std::array<std::shared_ptr<const flux::vulkan::ImmutableMaskLease>, 16U> leases;
  for (std::size_t index = 0U; index < leases.size(); ++index) {
    const auto height = 1'000U + index;
    std::vector<std::uint8_t> mask(height, static_cast<std::uint8_t>(index % 2U + 1U));
    bool hit = false;
    bool cross = false;
    std::string error;
    require(runtime.acquire_immutable_mask(1U, height, mask, flux::vulkan::MaskConsumer::terrain,
                                           leases[index], hit, cross, error),
            "runtime must publish each bounded immutable mask lease");
  }
  const auto oldest_handle = leases.front()->buffer();
  std::shared_ptr<const flux::vulkan::ImmutableMaskLease> overflow;
  bool hit = false;
  bool cross = false;
  std::string error;
  require(!runtime.acquire_immutable_mask(1U, 1'100U, std::vector<std::uint8_t>(1'100U, 1U),
                                          flux::vulkan::MaskConsumer::terrain, overflow, hit,
                                          cross, error) && !error.empty(),
          "a full cache of pinned immutable leases must fail closed");
  leases[8].reset();
  std::shared_ptr<const flux::vulkan::ImmutableMaskLease> replacement;
  require(runtime.acquire_immutable_mask(1U, 1'101U, std::vector<std::uint8_t>(1'101U, 1U),
                                         flux::vulkan::MaskConsumer::hydraulic, replacement, hit,
                                         cross, error),
          "an unpinned non-oldest entry must be evicted while the oldest lease remains pinned");
  std::shared_ptr<const flux::vulkan::ImmutableMaskLease> oldest_again;
  require(runtime.acquire_immutable_mask(1U, 1'000U, std::vector<std::uint8_t>(1'000U, 1U),
                                         flux::vulkan::MaskConsumer::hydraulic, oldest_again, hit,
                                         cross, error) && hit && oldest_again->buffer() == oldest_handle,
          "a pinned immutable lease must remain valid through unrelated eviction");
  oldest_again.reset();
  replacement.reset();
  for (auto& lease : leases)
    lease.reset();

  struct ConcurrentLease {
    std::shared_ptr<const flux::vulkan::ImmutableMaskLease> lease;
    bool ok{false};
  };
  const std::vector<std::uint8_t> concurrent_mask(53U * 29U, 1U);
  std::array<std::future<ConcurrentLease>, 4U> futures;
  for (auto& future : futures) {
    future = std::async(std::launch::async, [&runtime, &concurrent_mask] {
      ConcurrentLease result;
      bool cache_hit = false;
      bool cross_pipeline = false;
      std::string acquire_error;
      result.ok = runtime.acquire_immutable_mask(
          53U, 29U, concurrent_mask, flux::vulkan::MaskConsumer::terrain, result.lease,
          cache_hit, cross_pipeline, acquire_error);
      return result;
    });
  }
  std::array<ConcurrentLease, 4U> concurrent{};
  for (std::size_t index = 0U; index < futures.size(); ++index)
    concurrent[index] = futures[index].get();
  for (const auto& lease : concurrent)
    require(lease.ok && lease.lease && lease.lease->buffer() == concurrent.front().lease->buffer(),
            "concurrent exact immutable-mask acquisitions must share one resident buffer");
}

void check_polygon_mask_isolation() {
  const auto input = representative_input(47U, 35U);
  auto terrain_config = test_config();
  terrain_config.iterations = 2U;
  require_gpu(gpu(input, terrain_config), "terrain must establish a shared mask before polygon raster");

  flux::HydraulicPathInput polygon;
  polygon.width = input.width;
  polygon.height = input.height;
  polygon.source = input.source;
  polygon.target = input.target;
  polygon.polygon_raster = flux::HydraulicPolygonRaster{
      0, 0, 10, 0,
      {{0, 0, static_cast<std::int32_t>(input.width * 10U), 0, 0U},
       {static_cast<std::int32_t>(input.width * 10U), 0,
        static_cast<std::int32_t>(input.width * 10U), static_cast<std::int32_t>(input.height * 10U), 0U},
       {static_cast<std::int32_t>(input.width * 10U), static_cast<std::int32_t>(input.height * 10U),
        0, static_cast<std::int32_t>(input.height * 10U), 0U},
       {0, static_cast<std::int32_t>(input.height * 10U), 0, 0, 0U}}};
  flux::HydraulicPathConfig config;
  config.relaxation_iterations = 128U;
  config.droplet_batches = 1U;
  config.droplets_per_batch = 128U;
  config.maximum_steps_per_droplet = 128U;
  const auto polygon_result = flux::solve_hydraulic_erosion_path(polygon, config);
  require(polygon_result.ok() && polygon_result.used_gpu_polygon_raster,
          "polygon raster must use its private mutable binding-0 buffer");
  const auto after_polygon = gpu(input, terrain_config);
  require_parity(cpu(input, terrain_config), after_polygon,
                 "polygon raster writes must not mutate a same-sized shared terrain mask");
}

void check_feedback_and_reuse(const TerrainInput& large, const flux::TerrainEvolutionConfig& config) {
  const auto first = gpu(large, config);
  require_gpu(first, "initial GPU terrain request must execute");
  const auto reused = gpu(large, config);
  require_gpu(reused, "resident-mask GPU request must execute");
  require(reused.gpu_mask_upload_reuses == 1U && reused.gpu_mask_uploads == 0U,
          "identical mask must skip its device upload");
  require(first.height == reused.height && first.flow == reused.flow &&
              first.divergence == reused.divergence && first.path == reused.path,
          "resident mask must preserve deterministic terrain output");
  auto feedback_only = large;
  feedback_only.pressure[at(large.width, large.width / 2U, large.height / 2U)] =
      config.maximum_pressure;
  const auto feedback_gpu = gpu(feedback_only, config);
  require_parity(cpu(feedback_only, config), feedback_gpu,
                 "resident mask must still consume updated height feedback");
  require(feedback_gpu.gpu_mask_upload_reuses == 1U && feedback_gpu.gpu_mask_uploads == 0U,
          "height-only feedback should preserve mask residency");

  auto changed = large;
  changed.walkable[at(changed.width, changed.width / 3U, 2U)] = 0U;
  changed.walkable[at(changed.width, changed.width / 3U, 3U)] = 0U;
  changed.pressure[at(changed.width, changed.width / 2U, changed.height / 2U)] =
      config.maximum_pressure;
  changed.success[at(changed.width, changed.width / 2U + 1U, changed.height / 2U)] =
      config.maximum_success_prior;
  const auto changed_cpu = cpu(changed, config);
  const auto changed_gpu = gpu(changed, config);
  require(changed_gpu.gpu_mask_uploads == 1U && changed_gpu.gpu_mask_upload_reuses == 0U,
          "changed copper mask must invalidate device residency");
  require_parity(changed_cpu, changed_gpu, "changed mask and live-copper feedback must not use stale GPU fields");
  require(first.initial_height != changed_gpu.initial_height || first.height != changed_gpu.height,
          "mask/live-copper feedback must change the next GPU terrain request");

  const auto small = representative_input(64U, 48U);
  const auto small_gpu = gpu(small, config);
  require_gpu(small_gpu, "small GPU terrain request must execute after large request");
  require(small_gpu.gpu_mask_uploads == 1U, "changed dimensions must invalidate residency");
  const auto repeated_large = gpu(large, config);
  require_gpu(repeated_large, "large GPU terrain request must execute after small request");
  require(repeated_large.gpu_mask_upload_reuses == 1U && repeated_large.gpu_mask_uploads == 0U,
          "returning to a retained bounded shared mask cache entry must skip upload");
  require(repeated_large.gpu_buffer_reuses > 0U,
          "large-small-large sequence must reuse capacity-grown GPU resources");
  require(first.height == repeated_large.height && first.flow == repeated_large.flow &&
              first.divergence == repeated_large.divergence && first.path == repeated_large.path,
          "reused GPU resources must not retain stale terrain state");
}

void check_concurrency(const TerrainInput& input, const flux::TerrainEvolutionConfig& config) {
  constexpr std::size_t requests = 4U;
  std::vector<std::future<flux::TerrainEvolutionResult>> futures;
  futures.reserve(requests);
  for (std::size_t request = 0U; request < requests; ++request)
    futures.push_back(std::async(std::launch::async, [&] { return gpu(input, config); }));
  std::vector<flux::TerrainEvolutionResult> results;
  results.reserve(requests);
  for (auto& future : futures)
    results.push_back(future.get());
  for (const auto& result : results)
    require_gpu(result, "concurrent GPU terrain request must execute");
  for (std::size_t index = 1U; index < results.size(); ++index)
    require(results[index].height == results.front().height && results[index].flow == results.front().flow &&
                results[index].divergence == results.front().divergence &&
                results[index].path == results.front().path,
            "concurrent repeated GPU terrain requests must be deterministic");
}

void check_queued_gpu_cancellation() {
  const auto large = representative_input(1024U, 1024U);
  const auto small = representative_input(64U, 64U);
  auto warm_config = test_config();
  warm_config.iterations = 1U;
  warm_config.maximum_cells = 1'048'576U;
  require(cpu(large, warm_config).ok(),
          "GPU cancellation regression must establish the large CPU reference path");
  require_gpu(gpu(large, warm_config),
              "GPU cancellation regression must prewarm large Vulkan buffers and pipeline state");
  auto long_config = test_config();
  long_config.iterations = 256U;
  long_config.maximum_cells = 1'048'576U;
  std::stop_source first_stop;
  auto first = std::async(std::launch::async, [&] {
    return flux::solve_terrain_evolution_gpu(
        large.width, large.height, large.walkable, large.clearance, large.source, large.target,
        long_config, first_stop.get_token(), large.pressure, large.success);
  });
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  require(first.wait_for(std::chrono::milliseconds(0)) == std::future_status::timeout,
          "long GPU terrain request must remain active while a queued request is cancelled");

  std::stop_source queued_stop;
  auto queued = std::async(std::launch::async, [&] {
    return flux::solve_terrain_evolution_gpu(
        small.width, small.height, small.walkable, small.clearance, small.source, small.target,
        warm_config, queued_stop.get_token(), small.pressure, small.success);
  });
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  queued_stop.request_stop();
  require(queued.wait_for(std::chrono::milliseconds(200)) == std::future_status::ready,
          "GPU request queued behind active work must observe cancellation while waiting for the mutex");
  const auto cancelled = queued.get();
  require(!cancelled.ok() && !cancelled.used_gpu && cancelled.gpu_dispatches == 0U,
          "cancelled queued GPU terrain request must return without dispatching work");
  require(first.wait_for(std::chrono::milliseconds(0)) == std::future_status::timeout,
          "queued cancellation must return while the long GPU solve still owns the mutex");

  first_stop.request_stop();
  const auto interrupted = first.get();
  require(!interrupted.ok() && interrupted.used_gpu && interrupted.gpu_dispatches > 0U,
          "active GPU terrain work must report an interrupted dispatched solve");
}

void run_benchmark() {
  std::cout << std::fixed << std::setprecision(0);
  for (const auto extent : {64U, 128U, 256U, 512U}) {
    const auto input = representative_input(extent, extent);
    auto config = test_config();
    config.iterations = 16U;
    // Warm persistent GPU buffers and pipeline state before the recorded request.
    require(cpu(input, config).ok(), "CPU benchmark warm-up must solve");
    require_gpu(gpu(input, config), "GPU benchmark warm-up must execute");
    for (const auto backend : {std::string_view{"cpu"}, std::string_view{"gpu"}}) {
      const auto started = std::chrono::steady_clock::now();
      const auto result = backend == "cpu" ? cpu(input, config) : gpu(input, config);
      const auto total = std::chrono::duration_cast<std::chrono::microseconds>(
          std::chrono::steady_clock::now() - started).count();
      if (backend == "cpu")
        require(result.ok(), "CPU benchmark request must solve");
      else
        require_gpu(result, "GPU benchmark request must execute");
      std::cout << "{\"benchmark\":\"terrain_evolution\",\"backend\":\"" << backend
                << "\",\"width\":" << extent << ",\"height\":" << extent
                << ",\"iterations\":" << config.iterations << ",\"warm_cache\":true"
                << ",\"evolution_time_us\":" << result.evolution_time_us
                << ",\"extraction_time_us\":" << result.extraction_time_us
                << ",\"total_time_us\":" << total << ",\"gpu_dispatches\":"
                << result.gpu_dispatches << ",\"used_gpu\":"
                << (result.used_gpu ? "true" : "false") << "}\n";
    }
  }
}

} // namespace

int main(int argc, char** argv) {
  bool benchmark = false;
  if (argc > 1) {
    require(argc == 2 && std::string_view{argv[1]} == "--benchmark",
            "only --benchmark is accepted by the terrain GPU test executable");
    benchmark = true;
  }

  const auto input = representative_input(128U, 96U);
  const auto config = test_config();
  require_parity(cpu(input, config), gpu(input, config),
                 "representative FP32 terrain field must match CPU reference");
  check_field_only_parity(input, config);
  check_corner_cut_and_disconnection(config);
  check_branched_flow(config);
  check_long_downhill_chain(config);
  check_zero_and_cancel(input, config);
  check_invalid_endpoints_skip_gpu_evolution(input, config);
  check_feedback_and_reuse(input, config);
  check_concurrency(input, config);
  check_queued_gpu_cancellation();
  check_production_hydraulic_wrapper(input, config);
  check_cross_pipeline_mask_sharing();
  check_runtime_mask_lease_bounds_and_concurrency();
  check_polygon_mask_isolation();
  if (benchmark)
    run_benchmark();
}
