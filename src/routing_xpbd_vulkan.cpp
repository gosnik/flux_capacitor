#include "flux/core/geometry.hpp"
#include "flux/core/routing_compute.hpp"
#include "vulkan_runtime.hpp"

#include <vulkan/vulkan.h>

#include <algorithm>
#include <array>
#include <bit>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <limits>
#include <mutex>
#include <numeric>
#include <string>
#include <vector>

namespace flux {
namespace {

using SteadyClock = std::chrono::steady_clock;

[[nodiscard]] std::uint64_t elapsed_us(SteadyClock::time_point start,
                                       SteadyClock::time_point finish) {
  return static_cast<std::uint64_t>(
      std::chrono::duration_cast<std::chrono::microseconds>(finish - start).count());
}

struct Buffer {
  VkBuffer handle{VK_NULL_HANDLE};
  VkDeviceMemory memory{VK_NULL_HANDLE};
  void* mapped{nullptr};
  VkDeviceSize size{0U};
};

struct Push {
  std::uint32_t particle_count{0U};
  std::uint32_t segment_count{0U};
  std::uint32_t obstacle_count{0U};
  std::uint32_t pair_count{0U};
  std::uint32_t terrain_width{0U};
  std::uint32_t terrain_height{0U};
  std::uint32_t terrain_layer_count{0U};
  std::uint32_t pressure_layer_count{0U};
  std::uint32_t read_state{0U};
  std::uint32_t iteration{0U};
  std::uint32_t solver_iterations{0U};
  float terrain_origin_x{0.0F};
  float terrain_origin_y{0.0F};
  float terrain_pitch{0.0F};
  float maximum_step{0.0F};
  float guard_nm{0.0F};
  float stretch_weight{0.0F};
  float bend_weight{0.0F};
  float clearance_weight{0.0F};
  float corridor_weight{0.0F};
  float terrain_gain{0.0F};
  std::uint32_t mode{0U};
  std::uint32_t pressure_width{0U};
  std::uint32_t pressure_height{0U};
  std::uint32_t pressure_read_state{0U};
  float pressure_origin_x{0.0F};
  float pressure_origin_y{0.0F};
  float pressure_pitch{0.0F};
  float pressure_diffusion{0.0F};
  float pressure_scale{1'024.0F};
};
static_assert(sizeof(Push) <= 128U);

struct BroadphasePush {
  std::uint32_t mode{0U};
  std::uint32_t particle_count{0U};
  std::uint32_t segment_count{0U};
  std::uint32_t grid_width{0U};
  std::uint32_t grid_height{0U};
  std::uint32_t maximum_cell_occupancy{0U};
  std::uint32_t maximum_pair_emissions{0U};
  std::uint32_t cell_count{0U};
  float origin_x{0.0F};
  float origin_y{0.0F};
  float cell_pitch{0.0F};
  float guard_nm{0.0F};
};
static_assert(sizeof(BroadphasePush) <= 128U);

[[nodiscard]] float compliance_weight(long double compliance) {
  return static_cast<float>(compliance > 0.0L ? 1.0L / (1.0L + compliance * 10'000.0L) : 1.0L);
}

[[nodiscard]] std::uint32_t float_word(float value) { return std::bit_cast<std::uint32_t>(value); }

[[nodiscard]] std::int64_t particle_motion_bound(const RoutingXpbdParticle& particle,
                                                 const RoutingXpbdConfig& config) {
  if (!(particle.inverse_mass > 0.0L))
    return 0;
  const auto iterative =
      static_cast<long double>(std::max<std::int64_t>(0, config.maximum_step_nm)) *
      static_cast<long double>(config.iterations);
  const auto corridor = particle.corridor_radius_nm > 0
                            ? static_cast<long double>(particle.corridor_radius_nm)
                            : iterative;
  return static_cast<std::int64_t>(
      std::min<long double>(std::min(iterative, corridor),
                            static_cast<long double>(std::numeric_limits<std::int64_t>::max())));
}

[[nodiscard]] RoutingXpbdIterationMetrics
measure_positions(const std::vector<std::pair<std::int64_t, std::int64_t>>& positions,
                  const std::vector<RoutingXpbdParticle>& particles,
                  const std::vector<RoutingXpbdSegment>& segments,
                  const std::vector<RoutingXpbdObstacleSegment>& obstacles,
                  const std::vector<std::pair<std::uint32_t, std::uint32_t>>& pairs,
                  const RoutingXpbdConfig& config,
                  std::vector<long double>* particle_stress = nullptr,
                  std::size_t* unresolved_trace_pairs = nullptr) {
  RoutingXpbdIterationMetrics result;
  if (particle_stress != nullptr)
    particle_stress->assign(particles.size(), 0.0L);
  if (unresolved_trace_pairs != nullptr)
    *unresolved_trace_pairs = 0U;
  const auto point = [&](std::size_t particle) {
    return Point{Nanometres{positions[particle].first}, Nanometres{positions[particle].second}};
  };
  const auto record = [&](long double residual, std::initializer_list<std::size_t> owners) {
    if (!(residual > 0.5L))
      return;
    ++result.violations;
    result.maximum_residual_nm = std::max(result.maximum_residual_nm, residual);
    result.total_residual_nm += residual;
    if (particle_stress == nullptr)
      return;
    for (const auto owner : owners) {
      if (owner < particle_stress->size())
        (*particle_stress)[owner] += residual;
    }
  };
  for (const auto pair : pairs) {
    const auto& first = segments[pair.first];
    const auto& second = segments[pair.second];
    const auto distance =
        geometry::segment_distance_nm(point(first.first_particle), point(first.second_particle),
                                      point(second.first_particle), point(second.second_particle));
    const auto required = static_cast<long double>(
        std::max<std::int64_t>(0, first.radius_nm) + std::max<std::int64_t>(0, second.radius_nm) +
        std::max(std::max<std::int64_t>(0, first.clearance_nm),
                 std::max<std::int64_t>(0, second.clearance_nm)) +
        std::max<std::int64_t>(0, config.numerical_guard_nm));
    const auto residual = required - distance;
    record(residual, {first.first_particle, first.second_particle, second.first_particle,
                      second.second_particle});
    if (residual > 0.5L && unresolved_trace_pairs != nullptr)
      ++*unresolved_trace_pairs;
  }
  for (const auto& obstacle : obstacles) {
    const auto obstacle_start =
        Point{Nanometres{obstacle.first_x_nm}, Nanometres{obstacle.first_y_nm}};
    const auto obstacle_end =
        Point{Nanometres{obstacle.second_x_nm}, Nanometres{obstacle.second_y_nm}};
    for (const auto& segment : segments) {
      if (segment.layer != obstacle.layer ||
          (obstacle.route != std::numeric_limits<std::uint32_t>::max() &&
           segment.route != obstacle.route)) {
        continue;
      }
      const auto distance = geometry::segment_distance_nm(point(segment.first_particle),
                                                          point(segment.second_particle),
                                                          obstacle_start, obstacle_end);
      const auto required =
          static_cast<long double>(std::max<std::int64_t>(0, segment.radius_nm) +
                                   std::max<std::int64_t>(0, segment.clearance_nm) +
                                   std::max<std::int64_t>(0, obstacle.radius_nm) +
                                   std::max<std::int64_t>(0, config.numerical_guard_nm));
      record(required - distance, {segment.first_particle, segment.second_particle});
    }
  }
  for (std::size_t particle = 0U; particle < particles.size(); ++particle) {
    if (particles[particle].corridor_radius_nm <= 0)
      continue;
    const auto dx = static_cast<long double>(positions[particle].first -
                                             particles[particle].corridor_anchor_x_nm);
    const auto dy = static_cast<long double>(positions[particle].second -
                                             particles[particle].corridor_anchor_y_nm);
    record(std::hypotl(dx, dy) - static_cast<long double>(particles[particle].corridor_radius_nm),
           {particle});
  }
  return result;
}

class XpbdContext {
public:
  XpbdContext() { initialize(); }
  ~XpbdContext() { shutdown(); }
  XpbdContext(const XpbdContext&) = delete;
  XpbdContext& operator=(const XpbdContext&) = delete;

  [[nodiscard]] bool ok() const { return diagnostic_.empty(); }
  [[nodiscard]] const std::string& diagnostic() const { return diagnostic_; }

  [[nodiscard]] bool broadphase(const std::vector<std::uint32_t>& positions,
                                const std::vector<std::uint32_t>& segments, BroadphasePush push,
                                std::vector<std::pair<std::uint32_t, std::uint32_t>>& pairs,
                                bool& overflow, std::size_t& pair_emissions, std::uint64_t& gpu_us,
                                std::uint64_t& readback_us, std::string& error) {
    std::scoped_lock lock(mutex_);
    if (!ok()) {
      error = diagnostic_;
      return false;
    }
    if (push.particle_count == 0U || push.segment_count == 0U || push.grid_width == 0U ||
        push.grid_height == 0U || push.cell_count == 0U || push.maximum_cell_occupancy == 0U ||
        push.maximum_pair_emissions == 0U || !(push.cell_pitch > 0.0F)) {
      error = "invalid Vulkan XPBD spatial broadphase";
      return false;
    }
    const std::array<std::size_t, 6U> words{
        std::max<std::size_t>(1U, positions.size()),
        std::max<std::size_t>(1U, segments.size()),
        static_cast<std::size_t>(push.cell_count) + 1U,
        static_cast<std::size_t>(push.cell_count) * push.maximum_cell_occupancy,
        2U,
        static_cast<std::size_t>(push.maximum_pair_emissions) * 2U};
    for (std::size_t index = 0U; index < broad_buffers_.size(); ++index) {
      if (!ensure_buffer(words[index] * sizeof(std::uint32_t), broad_buffers_[index])) {
        error = "Vulkan XPBD broadphase buffer allocation failed";
        return false;
      }
    }
    std::memcpy(broad_buffers_[0].mapped, positions.data(),
                positions.size() * sizeof(std::uint32_t));
    std::memcpy(broad_buffers_[1].mapped, segments.data(), segments.size() * sizeof(std::uint32_t));
    if (!update_broadphase_descriptors() || !prepare_broadphase_command()) {
      error = "Vulkan XPBD broadphase command resources failed";
      return false;
    }
    VkCommandBufferBeginInfo begin{VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO};
    if (vkBeginCommandBuffer(broad_command_, &begin) != VK_SUCCESS) {
      error = "Vulkan XPBD broadphase command recording failed";
      return false;
    }
    vkCmdBindPipeline(broad_command_, VK_PIPELINE_BIND_POINT_COMPUTE, broad_pipeline_);
    vkCmdBindDescriptorSets(broad_command_, VK_PIPELINE_BIND_POINT_COMPUTE, broad_layout_, 0U, 1U,
                            &broad_descriptor_set_, 0U, nullptr);
    const auto barrier = [&] {
      VkMemoryBarrier memory{VK_STRUCTURE_TYPE_MEMORY_BARRIER};
      memory.srcAccessMask = VK_ACCESS_SHADER_WRITE_BIT;
      memory.dstAccessMask = VK_ACCESS_SHADER_READ_BIT | VK_ACCESS_SHADER_WRITE_BIT;
      vkCmdPipelineBarrier(broad_command_, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
                           VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, 0U, 1U, &memory, 0U, nullptr, 0U,
                           nullptr);
    };
    push.mode = 0U;
    vkCmdPushConstants(broad_command_, broad_layout_, VK_SHADER_STAGE_COMPUTE_BIT, 0U, sizeof(push),
                       &push);
    vkCmdDispatch(broad_command_, (std::max(push.cell_count + 1U, 2U) + 255U) / 256U, 1U, 1U);
    barrier();
    push.mode = 1U;
    vkCmdPushConstants(broad_command_, broad_layout_, VK_SHADER_STAGE_COMPUTE_BIT, 0U, sizeof(push),
                       &push);
    vkCmdDispatch(broad_command_, (push.segment_count + 255U) / 256U, 1U, 1U);
    barrier();
    push.mode = 2U;
    vkCmdPushConstants(broad_command_, broad_layout_, VK_SHADER_STAGE_COMPUTE_BIT, 0U, sizeof(push),
                       &push);
    vkCmdDispatch(broad_command_, (push.cell_count + 255U) / 256U, 1U, 1U);
    if (vkEndCommandBuffer(broad_command_) != VK_SUCCESS) {
      error = "Vulkan XPBD broadphase command finalization failed";
      return false;
    }
    const auto gpu_start = SteadyClock::now();
    if (!vulkan::Runtime::instance().submit_and_wait(broad_command_, error)) {
      error = "Vulkan XPBD broadphase dispatch failed: " + error;
      return false;
    }
    gpu_us = elapsed_us(gpu_start, SteadyClock::now());
    const auto readback_start = SteadyClock::now();
    const auto* cell_meta = static_cast<const std::uint32_t*>(broad_buffers_[2].mapped);
    const auto* pair_meta = static_cast<const std::uint32_t*>(broad_buffers_[4].mapped);
    const auto emitted = std::min(pair_meta[0], push.maximum_pair_emissions);
    pair_emissions = pair_meta[0];
    overflow = cell_meta[push.cell_count] != 0U || pair_meta[1] != 0U ||
               pair_meta[0] > push.maximum_pair_emissions;
    const auto* pair_words = static_cast<const std::uint32_t*>(broad_buffers_[5].mapped);
    pairs.clear();
    pairs.reserve(emitted);
    for (std::uint32_t index = 0U; index < emitted; ++index)
      pairs.emplace_back(pair_words[index * 2U], pair_words[index * 2U + 1U]);
    std::sort(pairs.begin(), pairs.end());
    pairs.erase(std::unique(pairs.begin(), pairs.end()), pairs.end());
    readback_us = elapsed_us(readback_start, SteadyClock::now());
    return true;
  }

  [[nodiscard]] bool
  dispatch(const std::array<std::vector<std::uint32_t>, 12U>& input, Push push,
           std::size_t iterations, std::size_t pressure_diffusion_passes,
           std::vector<std::uint32_t>& positions, std::vector<std::uint32_t>& pressure,
           std::vector<std::uint32_t>& stress, std::vector<std::uint32_t>& iteration_residuals,
           bool record_iteration_metrics, std::uint64_t& solve_us, std::uint64_t& readback_us,
           std::size_t& resident_reuses, std::string& error) {
    std::scoped_lock lock(mutex_);
    if (!ok()) {
      error = diagnostic_;
      return false;
    }
    if (push.particle_count == 0U || iterations == 0U) {
      error = "invalid Vulkan XPBD dispatch";
      return false;
    }
    resident_reuses = 0U;
    for (std::size_t index = 0U; index < buffers_.size(); ++index) {
      const auto words = std::max<std::size_t>(1U, input[index].size());
      const auto bytes = words * sizeof(std::uint32_t);
      const auto reallocated = buffers_[index].handle == VK_NULL_HANDLE ||
                               buffers_[index].size < static_cast<VkDeviceSize>(bytes);
      if (!ensure_buffer(bytes, buffers_[index])) {
        error = "Vulkan XPBD buffer allocation failed";
        return false;
      }
      if (!input[index].empty()) {
        const auto cacheable = index >= 2U && index <= 7U;
        if (cacheable && !reallocated && resident_inputs_[index] == input[index]) {
          ++resident_reuses;
        } else {
          std::memcpy(buffers_[index].mapped, input[index].data(),
                      input[index].size() * sizeof(std::uint32_t));
          if (cacheable)
            resident_inputs_[index] = input[index];
        }
      }
    }
    // Both ping-pong position buffers start from identical immutable input.
    std::memcpy(buffers_[1].mapped, buffers_[0].mapped, input[0].size() * sizeof(std::uint32_t));
    if (!update_descriptors() || !prepare_command()) {
      error = "Vulkan XPBD command resources failed";
      return false;
    }
    VkCommandBufferBeginInfo begin{VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO};
    if (vkBeginCommandBuffer(command_, &begin) != VK_SUCCESS) {
      error = "Vulkan XPBD command recording failed";
      return false;
    }
    vkCmdBindPipeline(command_, VK_PIPELINE_BIND_POINT_COMPUTE, pipeline_);
    vkCmdBindDescriptorSets(command_, VK_PIPELINE_BIND_POINT_COMPUTE, layout_, 0U, 1U,
                            &descriptor_set_, 0U, nullptr);
    const auto groups = (push.particle_count + 255U) / 256U;
    push.mode = 0U;
    for (std::size_t iteration = 0U; iteration < iterations; ++iteration) {
      push.iteration = static_cast<std::uint32_t>(iteration);
      push.solver_iterations = static_cast<std::uint32_t>(iterations);
      push.read_state = static_cast<std::uint32_t>(iteration & 1U);
      vkCmdPushConstants(command_, layout_, VK_SHADER_STAGE_COMPUTE_BIT, 0U, sizeof(push), &push);
      vkCmdDispatch(command_, groups, 1U, 1U);
      VkMemoryBarrier barrier{VK_STRUCTURE_TYPE_MEMORY_BARRIER};
      barrier.srcAccessMask = VK_ACCESS_SHADER_WRITE_BIT;
      barrier.dstAccessMask = VK_ACCESS_SHADER_READ_BIT | VK_ACCESS_SHADER_WRITE_BIT;
      vkCmdPipelineBarrier(command_, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
                           VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, 0U, 1U, &barrier, 0U, nullptr, 0U,
                           nullptr);
      if (record_iteration_metrics) {
        push.mode = 4U;
        push.read_state = static_cast<std::uint32_t>((iteration + 1U) & 1U);
        vkCmdPushConstants(command_, layout_, VK_SHADER_STAGE_COMPUTE_BIT, 0U, sizeof(push), &push);
        vkCmdDispatch(command_, groups, 1U, 1U);
        vkCmdPipelineBarrier(command_, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
                             VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, 0U, 1U, &barrier, 0U, nullptr,
                             0U, nullptr);
        push.mode = 0U;
      }
    }
    const auto pressure_cells =
        push.pressure_width * push.pressure_height * push.pressure_layer_count;
    if (pressure_cells > 0U && push.pressure_pitch > 0.0F) {
      push.mode = 1U;
      vkCmdPushConstants(command_, layout_, VK_SHADER_STAGE_COMPUTE_BIT, 0U, sizeof(push), &push);
      vkCmdDispatch(command_, (pressure_cells + 255U) / 256U, 1U, 1U);
      VkMemoryBarrier pressure_barrier{VK_STRUCTURE_TYPE_MEMORY_BARRIER};
      pressure_barrier.srcAccessMask = VK_ACCESS_SHADER_WRITE_BIT;
      pressure_barrier.dstAccessMask = VK_ACCESS_SHADER_READ_BIT | VK_ACCESS_SHADER_WRITE_BIT;
      vkCmdPipelineBarrier(command_, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
                           VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, 0U, 1U, &pressure_barrier, 0U,
                           nullptr, 0U, nullptr);
      push.mode = 2U;
      push.read_state = static_cast<std::uint32_t>(iterations & 1U);
      vkCmdPushConstants(command_, layout_, VK_SHADER_STAGE_COMPUTE_BIT, 0U, sizeof(push), &push);
      vkCmdDispatch(command_, groups, 1U, 1U);
      vkCmdPipelineBarrier(command_, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
                           VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, 0U, 1U, &pressure_barrier, 0U,
                           nullptr, 0U, nullptr);
      push.mode = 3U;
      for (std::size_t pass = 0U; pass < pressure_diffusion_passes; ++pass) {
        push.pressure_read_state = static_cast<std::uint32_t>(pass & 1U);
        vkCmdPushConstants(command_, layout_, VK_SHADER_STAGE_COMPUTE_BIT, 0U, sizeof(push), &push);
        vkCmdDispatch(command_, (pressure_cells + 255U) / 256U, 1U, 1U);
        vkCmdPipelineBarrier(command_, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
                             VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, 0U, 1U, &pressure_barrier, 0U,
                             nullptr, 0U, nullptr);
      }
    }
    if (vkEndCommandBuffer(command_) != VK_SUCCESS) {
      error = "Vulkan XPBD command finalization failed";
      return false;
    }
    const auto solve_start = SteadyClock::now();
    if (!vulkan::Runtime::instance().submit_and_wait(command_, error)) {
      error = "Vulkan XPBD dispatch failed: " + error;
      return false;
    }
    solve_us = elapsed_us(solve_start, SteadyClock::now());
    const auto final_buffer = iterations % 2U == 0U ? 0U : 1U;
    const auto readback_start = SteadyClock::now();
    positions.resize(input[0].size());
    std::memcpy(positions.data(), buffers_[final_buffer].mapped,
                positions.size() * sizeof(std::uint32_t));
    stress.resize(input[10].size());
    std::memcpy(stress.data(), buffers_[10].mapped, stress.size() * sizeof(std::uint32_t));
    if (record_iteration_metrics) {
      iteration_residuals.resize(iterations * 4U);
      std::memcpy(iteration_residuals.data(), buffers_[11].mapped,
                  iteration_residuals.size() * sizeof(std::uint32_t));
    }
    if (pressure_cells > 0U && push.pressure_pitch > 0.0F) {
      const auto pressure_buffer = pressure_diffusion_passes % 2U == 0U ? 8U : 9U;
      pressure.resize(pressure_cells);
      std::memcpy(pressure.data(), buffers_[pressure_buffer].mapped,
                  pressure.size() * sizeof(std::uint32_t));
    }
    readback_us = elapsed_us(readback_start, SteadyClock::now());
    return true;
  }

private:
  void initialize() {
    auto& runtime = vulkan::Runtime::instance();
    if (!runtime.ok()) {
      diagnostic_ = runtime.diagnostic();
      return;
    }
    instance_ = runtime.instance_handle();
    physical_ = runtime.physical_device();
    device_ = runtime.device();
    queue_ = runtime.queue();
    family_ = runtime.queue_family();
    VkCommandPoolCreateInfo pool{VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO};
    pool.queueFamilyIndex = family_;
    pool.flags = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT;
    if (vkCreateCommandPool(device_, &pool, nullptr, &command_pool_) != VK_SUCCESS) {
      diagnostic_ = "XPBD command pool failed";
      return;
    }
    std::array<VkDescriptorSetLayoutBinding, 12U> bindings{};
    for (std::uint32_t index = 0U; index < bindings.size(); ++index) {
      bindings[index].binding = index;
      bindings[index].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
      bindings[index].descriptorCount = 1U;
      bindings[index].stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
    }
    VkDescriptorSetLayoutCreateInfo descriptor{VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO};
    descriptor.bindingCount = static_cast<std::uint32_t>(bindings.size());
    descriptor.pBindings = bindings.data();
    if (vkCreateDescriptorSetLayout(device_, &descriptor, nullptr, &descriptor_) != VK_SUCCESS) {
      diagnostic_ = "XPBD descriptor layout failed";
      return;
    }
    VkPushConstantRange range{VK_SHADER_STAGE_COMPUTE_BIT, 0U, sizeof(Push)};
    VkPipelineLayoutCreateInfo layout{VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO};
    layout.setLayoutCount = 1U;
    layout.pSetLayouts = &descriptor_;
    layout.pushConstantRangeCount = 1U;
    layout.pPushConstantRanges = &range;
    if (vkCreatePipelineLayout(device_, &layout, nullptr, &layout_) != VK_SUCCESS) {
      diagnostic_ = "XPBD pipeline layout failed";
      return;
    }
    std::ifstream stream(FLUX_ROUTING_XPBD_SHADER_PATH, std::ios::binary | std::ios::ate);
    if (!stream) {
      diagnostic_ = "XPBD shader is missing";
      return;
    }
    const auto bytes = stream.tellg();
    if (bytes <= 0 || static_cast<std::size_t>(bytes) % sizeof(std::uint32_t) != 0U) {
      diagnostic_ = "XPBD shader is invalid";
      return;
    }
    std::vector<std::uint32_t> code(static_cast<std::size_t>(bytes) / sizeof(std::uint32_t));
    stream.seekg(0);
    stream.read(reinterpret_cast<char*>(code.data()), bytes);
    VkShaderModuleCreateInfo module_info{VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO};
    module_info.codeSize = static_cast<std::size_t>(bytes);
    module_info.pCode = code.data();
    VkShaderModule module = VK_NULL_HANDLE;
    if (vkCreateShaderModule(device_, &module_info, nullptr, &module) != VK_SUCCESS) {
      diagnostic_ = "XPBD shader module failed";
      return;
    }
    VkPipelineShaderStageCreateInfo stage{VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO};
    stage.stage = VK_SHADER_STAGE_COMPUTE_BIT;
    stage.module = module;
    stage.pName = "main";
    VkComputePipelineCreateInfo pipeline{VK_STRUCTURE_TYPE_COMPUTE_PIPELINE_CREATE_INFO};
    pipeline.stage = stage;
    pipeline.layout = layout_;
    const auto status = runtime.create_compute_pipeline(pipeline, pipeline_);
    vkDestroyShaderModule(device_, module, nullptr);
    if (status != VK_SUCCESS) {
      diagnostic_ = "XPBD compute pipeline failed";
      return;
    }

    std::array<VkDescriptorSetLayoutBinding, 6U> broad_bindings{};
    for (std::uint32_t index = 0U; index < broad_bindings.size(); ++index) {
      broad_bindings[index].binding = index;
      broad_bindings[index].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
      broad_bindings[index].descriptorCount = 1U;
      broad_bindings[index].stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
    }
    VkDescriptorSetLayoutCreateInfo broad_descriptor{
        VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO};
    broad_descriptor.bindingCount = static_cast<std::uint32_t>(broad_bindings.size());
    broad_descriptor.pBindings = broad_bindings.data();
    if (vkCreateDescriptorSetLayout(device_, &broad_descriptor, nullptr, &broad_descriptor_) !=
        VK_SUCCESS) {
      diagnostic_ = "XPBD broadphase descriptor layout failed";
      return;
    }
    VkPushConstantRange broad_range{VK_SHADER_STAGE_COMPUTE_BIT, 0U, sizeof(BroadphasePush)};
    VkPipelineLayoutCreateInfo broad_layout{VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO};
    broad_layout.setLayoutCount = 1U;
    broad_layout.pSetLayouts = &broad_descriptor_;
    broad_layout.pushConstantRangeCount = 1U;
    broad_layout.pPushConstantRanges = &broad_range;
    if (vkCreatePipelineLayout(device_, &broad_layout, nullptr, &broad_layout_) != VK_SUCCESS) {
      diagnostic_ = "XPBD broadphase pipeline layout failed";
      return;
    }
    std::ifstream broad_stream(FLUX_ROUTING_XPBD_BROADPHASE_SHADER_PATH,
                               std::ios::binary | std::ios::ate);
    if (!broad_stream) {
      diagnostic_ = "XPBD broadphase shader is missing";
      return;
    }
    const auto broad_bytes = broad_stream.tellg();
    if (broad_bytes <= 0 || static_cast<std::size_t>(broad_bytes) % sizeof(std::uint32_t) != 0U) {
      diagnostic_ = "XPBD broadphase shader is invalid";
      return;
    }
    std::vector<std::uint32_t> broad_code(static_cast<std::size_t>(broad_bytes) /
                                          sizeof(std::uint32_t));
    broad_stream.seekg(0);
    broad_stream.read(reinterpret_cast<char*>(broad_code.data()), broad_bytes);
    VkShaderModuleCreateInfo broad_module_info{VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO};
    broad_module_info.codeSize = static_cast<std::size_t>(broad_bytes);
    broad_module_info.pCode = broad_code.data();
    VkShaderModule broad_module = VK_NULL_HANDLE;
    if (vkCreateShaderModule(device_, &broad_module_info, nullptr, &broad_module) != VK_SUCCESS) {
      diagnostic_ = "XPBD broadphase shader module failed";
      return;
    }
    VkPipelineShaderStageCreateInfo broad_stage{
        VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO};
    broad_stage.stage = VK_SHADER_STAGE_COMPUTE_BIT;
    broad_stage.module = broad_module;
    broad_stage.pName = "main";
    VkComputePipelineCreateInfo broad_pipeline{VK_STRUCTURE_TYPE_COMPUTE_PIPELINE_CREATE_INFO};
    broad_pipeline.stage = broad_stage;
    broad_pipeline.layout = broad_layout_;
    const auto broad_status = runtime.create_compute_pipeline(broad_pipeline, broad_pipeline_);
    vkDestroyShaderModule(device_, broad_module, nullptr);
    if (broad_status != VK_SUCCESS)
      diagnostic_ = "XPBD broadphase compute pipeline failed";
  }

  [[nodiscard]] bool ensure_buffer(std::size_t bytes, Buffer& buffer) {
    if (buffer.handle != VK_NULL_HANDLE && buffer.size >= bytes)
      return true;
    destroy(buffer);
    if (!vulkan::Runtime::instance().allocate_host_buffer(
            static_cast<VkDeviceSize>(bytes), buffer.handle, buffer.memory, buffer.mapped))
      return false;
    buffer.size = bytes;
    return true;
  }

  [[nodiscard]] bool update_descriptors() {
    if (descriptor_pool_ == VK_NULL_HANDLE) {
      VkDescriptorPoolSize size{VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 12U};
      VkDescriptorPoolCreateInfo info{VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO};
      info.maxSets = 1U;
      info.poolSizeCount = 1U;
      info.pPoolSizes = &size;
      if (vkCreateDescriptorPool(device_, &info, nullptr, &descriptor_pool_) != VK_SUCCESS)
        return false;
      VkDescriptorSetAllocateInfo allocate{VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO};
      allocate.descriptorPool = descriptor_pool_;
      allocate.descriptorSetCount = 1U;
      allocate.pSetLayouts = &descriptor_;
      if (vkAllocateDescriptorSets(device_, &allocate, &descriptor_set_) != VK_SUCCESS)
        return false;
    }
    std::array<VkDescriptorBufferInfo, 12U> infos{};
    std::array<VkWriteDescriptorSet, 12U> writes{};
    for (std::uint32_t index = 0U; index < buffers_.size(); ++index) {
      infos[index] = {buffers_[index].handle, 0U, buffers_[index].size};
      writes[index] = {VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
      writes[index].dstSet = descriptor_set_;
      writes[index].dstBinding = index;
      writes[index].descriptorCount = 1U;
      writes[index].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
      writes[index].pBufferInfo = &infos[index];
    }
    vkUpdateDescriptorSets(device_, static_cast<std::uint32_t>(writes.size()), writes.data(), 0U,
                           nullptr);
    return true;
  }

  [[nodiscard]] bool update_broadphase_descriptors() {
    if (broad_descriptor_pool_ == VK_NULL_HANDLE) {
      VkDescriptorPoolSize size{VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 6U};
      VkDescriptorPoolCreateInfo info{VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO};
      info.maxSets = 1U;
      info.poolSizeCount = 1U;
      info.pPoolSizes = &size;
      if (vkCreateDescriptorPool(device_, &info, nullptr, &broad_descriptor_pool_) != VK_SUCCESS)
        return false;
      VkDescriptorSetAllocateInfo allocate{VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO};
      allocate.descriptorPool = broad_descriptor_pool_;
      allocate.descriptorSetCount = 1U;
      allocate.pSetLayouts = &broad_descriptor_;
      if (vkAllocateDescriptorSets(device_, &allocate, &broad_descriptor_set_) != VK_SUCCESS)
        return false;
    }
    std::array<VkDescriptorBufferInfo, 6U> infos{};
    std::array<VkWriteDescriptorSet, 6U> writes{};
    for (std::uint32_t index = 0U; index < broad_buffers_.size(); ++index) {
      infos[index] = {broad_buffers_[index].handle, 0U, broad_buffers_[index].size};
      writes[index] = {VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
      writes[index].dstSet = broad_descriptor_set_;
      writes[index].dstBinding = index;
      writes[index].descriptorCount = 1U;
      writes[index].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
      writes[index].pBufferInfo = &infos[index];
    }
    vkUpdateDescriptorSets(device_, static_cast<std::uint32_t>(writes.size()), writes.data(), 0U,
                           nullptr);
    return true;
  }

  [[nodiscard]] bool prepare_command() {
    if (command_ != VK_NULL_HANDLE) {
      return vkResetCommandBuffer(command_, 0U) == VK_SUCCESS;
    }
    VkCommandBufferAllocateInfo allocate{VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO};
    allocate.commandPool = command_pool_;
    allocate.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
    allocate.commandBufferCount = 1U;
    return vkAllocateCommandBuffers(device_, &allocate, &command_) == VK_SUCCESS;
  }

  [[nodiscard]] bool prepare_broadphase_command() {
    if (broad_command_ != VK_NULL_HANDLE)
      return vkResetCommandBuffer(broad_command_, 0U) == VK_SUCCESS;
    VkCommandBufferAllocateInfo allocate{VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO};
    allocate.commandPool = command_pool_;
    allocate.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
    allocate.commandBufferCount = 1U;
    return vkAllocateCommandBuffers(device_, &allocate, &broad_command_) == VK_SUCCESS;
  }

  void destroy(Buffer& buffer) {
    vulkan::Runtime::instance().release_buffer(buffer.handle, buffer.memory, buffer.mapped);
    buffer = {};
  }

  void shutdown() {
    for (auto& buffer : buffers_)
      destroy(buffer);
    for (auto& buffer : broad_buffers_)
      destroy(buffer);
    if (broad_descriptor_pool_ != VK_NULL_HANDLE)
      vkDestroyDescriptorPool(device_, broad_descriptor_pool_, nullptr);
    if (descriptor_pool_ != VK_NULL_HANDLE)
      vkDestroyDescriptorPool(device_, descriptor_pool_, nullptr);
    if (broad_pipeline_ != VK_NULL_HANDLE)
      vkDestroyPipeline(device_, broad_pipeline_, nullptr);
    if (pipeline_ != VK_NULL_HANDLE)
      vkDestroyPipeline(device_, pipeline_, nullptr);
    if (broad_layout_ != VK_NULL_HANDLE)
      vkDestroyPipelineLayout(device_, broad_layout_, nullptr);
    if (layout_ != VK_NULL_HANDLE)
      vkDestroyPipelineLayout(device_, layout_, nullptr);
    if (broad_descriptor_ != VK_NULL_HANDLE)
      vkDestroyDescriptorSetLayout(device_, broad_descriptor_, nullptr);
    if (descriptor_ != VK_NULL_HANDLE)
      vkDestroyDescriptorSetLayout(device_, descriptor_, nullptr);
    if (command_pool_ != VK_NULL_HANDLE)
      vkDestroyCommandPool(device_, command_pool_, nullptr);
  }

  VkInstance instance_{VK_NULL_HANDLE};
  VkPhysicalDevice physical_{VK_NULL_HANDLE};
  VkDevice device_{VK_NULL_HANDLE};
  std::uint32_t family_{0U};
  VkQueue queue_{VK_NULL_HANDLE};
  VkCommandPool command_pool_{VK_NULL_HANDLE};
  VkCommandBuffer command_{VK_NULL_HANDLE};
  VkCommandBuffer broad_command_{VK_NULL_HANDLE};
  VkDescriptorSetLayout descriptor_{VK_NULL_HANDLE};
  VkDescriptorPool descriptor_pool_{VK_NULL_HANDLE};
  VkDescriptorSet descriptor_set_{VK_NULL_HANDLE};
  VkPipelineLayout layout_{VK_NULL_HANDLE};
  VkPipeline pipeline_{VK_NULL_HANDLE};
  std::array<Buffer, 12U> buffers_{};
  // Immutable particle metadata, anchors, segment topology, static obstacles, candidate pairs,
  // and terrain remain in their mapped Vulkan buffers across equivalent regional attempts.
  std::array<std::vector<std::uint32_t>, 12U> resident_inputs_{};
  VkDescriptorSetLayout broad_descriptor_{VK_NULL_HANDLE};
  VkDescriptorPool broad_descriptor_pool_{VK_NULL_HANDLE};
  VkDescriptorSet broad_descriptor_set_{VK_NULL_HANDLE};
  VkPipelineLayout broad_layout_{VK_NULL_HANDLE};
  VkPipeline broad_pipeline_{VK_NULL_HANDLE};
  std::array<Buffer, 6U> broad_buffers_{};
  std::mutex mutex_;
  std::string diagnostic_;
};

[[nodiscard]] XpbdContext& context() {
  static XpbdContext value;
  return value;
}

} // namespace

RoutingXpbdResult relax_routing_xpbd_gpu(const std::vector<RoutingXpbdParticle>& particles,
                                         const std::vector<RoutingXpbdSegment>& segments,
                                         const std::vector<RoutingXpbdObstacleSegment>& obstacles,
                                         const RoutingXpbdTerrainField& terrain,
                                         const RoutingXpbdConfig& config) {
  RoutingXpbdResult result;
  constexpr std::uint32_t maximum_routing_layers = 64U;
  if (particles.empty() || segments.empty() || config.iterations == 0U ||
      config.maximum_step_nm <= 0 || config.maximum_candidate_pairs == 0U ||
      particles.size() > std::numeric_limits<std::uint32_t>::max() ||
      segments.size() > std::numeric_limits<std::uint32_t>::max() ||
      obstacles.size() > std::numeric_limits<std::uint32_t>::max() ||
      config.iterations > std::numeric_limits<std::uint32_t>::max() ||
      config.iterations > std::numeric_limits<std::size_t>::max() / 4U ||
      config.pressure_width > std::numeric_limits<std::uint32_t>::max() ||
      config.pressure_height > std::numeric_limits<std::uint32_t>::max() ||
      terrain.width > std::numeric_limits<std::uint32_t>::max() ||
      terrain.height > std::numeric_limits<std::uint32_t>::max() ||
      terrain.layer_count > std::numeric_limits<std::uint32_t>::max()) {
    result.diagnostic = "invalid Vulkan XPBD input";
    return result;
  }
  if (std::any_of(particles.begin(), particles.end(),
                  [](const auto& particle) { return particle.layer >= maximum_routing_layers; })) {
    result.diagnostic = "Vulkan XPBD particle layer exceeds the bounded routing-layer count";
    return result;
  }
  const auto pressure_layer_count =
      static_cast<std::size_t>(
          std::max_element(particles.begin(), particles.end(),
                           [](const auto& lhs, const auto& rhs) { return lhs.layer < rhs.layer; })
              ->layer) +
      1U;
  const auto checked_cells = [](std::size_t width, std::size_t height, std::size_t layers,
                                std::size_t& cells) {
    if (width == 0U || height == 0U || layers == 0U ||
        width > std::numeric_limits<std::size_t>::max() / height)
      return false;
    const auto plane = width * height;
    if (plane > std::numeric_limits<std::size_t>::max() / layers)
      return false;
    cells = plane * layers;
    return cells <= std::numeric_limits<std::uint32_t>::max();
  };
  if (!terrain.pressure.empty()) {
    std::size_t terrain_cells = 0U;
    if (!checked_cells(terrain.width, terrain.height, terrain.layer_count, terrain_cells) ||
        terrain.pitch_nm <= 0 || terrain.layer_count > maximum_routing_layers ||
        terrain.pressure.size() != terrain_cells ||
        std::any_of(terrain.pressure.begin(), terrain.pressure.end(),
                    [](const auto value) { return !std::isfinite(value) || value < 0.0F; })) {
      result.diagnostic = "invalid layered Vulkan XPBD terrain field";
      return result;
    }
  }
  std::size_t pressure_cells = 0U;
  const auto pressure_requested = config.pressure_width != 0U || config.pressure_height != 0U;
  if (pressure_requested && (!checked_cells(config.pressure_width, config.pressure_height,
                                            pressure_layer_count, pressure_cells) ||
                             config.pressure_pitch_nm <= 0)) {
    result.diagnostic = "invalid layered Vulkan XPBD pressure field";
    return result;
  }
  const auto origin_x =
      std::min_element(particles.begin(), particles.end(), [](const auto& a, const auto& b) {
        return a.x_nm < b.x_nm;
      })->x_nm;
  const auto origin_y =
      std::min_element(particles.begin(), particles.end(), [](const auto& a, const auto& b) {
        return a.y_nm < b.y_nm;
      })->y_nm;
  const auto broadphase_start = SteadyClock::now();
  std::vector<std::pair<std::uint32_t, std::uint32_t>> pairs;
  std::vector<std::uint32_t> broad_positions;
  broad_positions.reserve(particles.size() * 4U);
  for (const auto& particle : particles) {
    broad_positions.insert(
        broad_positions.end(),
        {float_word(static_cast<float>(particle.x_nm - origin_x)),
         float_word(static_cast<float>(particle.y_nm - origin_y)),
         float_word(static_cast<float>(std::max<long double>(0.0L, particle.inverse_mass))), 0U});
  }
  std::vector<std::uint32_t> broad_segments;
  broad_segments.reserve(segments.size() * 8U);
  auto minimum_x = std::numeric_limits<long double>::max();
  auto minimum_y = std::numeric_limits<long double>::max();
  auto maximum_x = std::numeric_limits<long double>::lowest();
  auto maximum_y = std::numeric_limits<long double>::lowest();
  auto maximum_extent = 1.0L;
  for (const auto& segment : segments) {
    if (segment.first_particle >= particles.size() || segment.second_particle >= particles.size()) {
      result.diagnostic = "Vulkan XPBD segment references invalid particle";
      return result;
    }
    const auto& first = particles[segment.first_particle];
    const auto& second = particles[segment.second_particle];
    const auto motion =
        std::max(particle_motion_bound(first, config), particle_motion_bound(second, config));
    const auto expansion =
        static_cast<long double>(std::max<std::int64_t>(0, segment.radius_nm) +
                                 std::max<std::int64_t>(0, segment.clearance_nm) + motion +
                                 std::max<std::int64_t>(0, config.numerical_guard_nm));
    const auto local_first_x = static_cast<long double>(first.x_nm - origin_x);
    const auto local_first_y = static_cast<long double>(first.y_nm - origin_y);
    const auto local_second_x = static_cast<long double>(second.x_nm - origin_x);
    const auto local_second_y = static_cast<long double>(second.y_nm - origin_y);
    const auto segment_minimum_x = std::min(local_first_x, local_second_x) - expansion;
    const auto segment_minimum_y = std::min(local_first_y, local_second_y) - expansion;
    const auto segment_maximum_x = std::max(local_first_x, local_second_x) + expansion;
    const auto segment_maximum_y = std::max(local_first_y, local_second_y) + expansion;
    minimum_x = std::min(minimum_x, segment_minimum_x);
    minimum_y = std::min(minimum_y, segment_minimum_y);
    maximum_x = std::max(maximum_x, segment_maximum_x);
    maximum_y = std::max(maximum_y, segment_maximum_y);
    maximum_extent = std::max(maximum_extent, std::max(segment_maximum_x - segment_minimum_x,
                                                       segment_maximum_y - segment_minimum_y));
    broad_segments.insert(
        broad_segments.end(),
        {segment.first_particle, segment.second_particle, segment.route, segment.layer,
         float_word(static_cast<float>(std::max<std::int64_t>(0, segment.radius_nm))),
         float_word(static_cast<float>(std::max<std::int64_t>(1, segment.rest_length_nm))),
         float_word(static_cast<float>(std::max<std::int64_t>(0, segment.clearance_nm))),
         float_word(static_cast<float>(motion))});
  }
  constexpr std::uint32_t maximum_grid_dimension = 128U;
  auto cell_pitch = std::max<long double>(1.0L, std::ceil(maximum_extent / 4.0L));
  const auto total_width = std::max<long double>(1.0L, maximum_x - minimum_x);
  const auto total_height = std::max<long double>(1.0L, maximum_y - minimum_y);
  cell_pitch =
      std::max(cell_pitch, std::ceil(std::max(total_width, total_height) /
                                     static_cast<long double>(maximum_grid_dimension - 1U)));
  const auto dimension = [&](long double extent) {
    return static_cast<std::uint32_t>(std::clamp<long double>(std::ceil(extent / cell_pitch) + 1.0L,
                                                              1.0L, maximum_grid_dimension));
  };
  BroadphasePush broad_push;
  broad_push.particle_count = static_cast<std::uint32_t>(particles.size());
  broad_push.segment_count = static_cast<std::uint32_t>(segments.size());
  broad_push.grid_width = dimension(total_width);
  broad_push.grid_height = dimension(total_height);
  broad_push.cell_count = broad_push.grid_width * broad_push.grid_height;
  broad_push.maximum_cell_occupancy =
      static_cast<std::uint32_t>(std::clamp<std::size_t>(segments.size(), 1U, 256U));
  const auto emission_limit = std::min<std::size_t>(
      4'194'304U, config.maximum_candidate_pairs > 4'194'304U / 8U
                      ? 4'194'304U
                      : std::max<std::size_t>(config.maximum_candidate_pairs,
                                              config.maximum_candidate_pairs * 8U));
  broad_push.maximum_pair_emissions = static_cast<std::uint32_t>(emission_limit);
  broad_push.origin_x = static_cast<float>(minimum_x);
  broad_push.origin_y = static_cast<float>(minimum_y);
  broad_push.cell_pitch = static_cast<float>(cell_pitch);
  broad_push.guard_nm = static_cast<float>(std::max<std::int64_t>(0, config.numerical_guard_nm));
  bool broadphase_overflow = false;
  std::size_t broadphase_emissions = 0U;
  std::uint64_t broadphase_gpu_us = 0U;
  std::uint64_t broadphase_readback_us = 0U;
  std::string broadphase_error;
  const auto gpu_broadphase = context().broadphase(
      broad_positions, broad_segments, broad_push, pairs, broadphase_overflow, broadphase_emissions,
      broadphase_gpu_us, broadphase_readback_us, broadphase_error);
  result.used_gpu_broadphase = gpu_broadphase;
  result.broadphase_cells = broad_push.cell_count;
  result.broadphase_pair_emissions = broadphase_emissions;
  result.timings.readback_us += broadphase_readback_us;
  const auto build_all_pairs = [&] {
    pairs.clear();
    for (std::size_t first = 0U; first < segments.size(); ++first) {
      for (std::size_t second = first + 1U; second < segments.size(); ++second) {
        if (segments[first].route == segments[second].route ||
            segments[first].layer != segments[second].layer)
          continue;
        if (pairs.size() == config.maximum_candidate_pairs) {
          result.pair_budget_exhausted = true;
          return;
        }
        pairs.emplace_back(static_cast<std::uint32_t>(first), static_cast<std::uint32_t>(second));
      }
    }
  };
  if (!gpu_broadphase || broadphase_overflow) {
    result.broadphase_fallback = true;
    build_all_pairs();
    if (!gpu_broadphase)
      result.broadphase_pair_emissions = pairs.size();
  } else if (pairs.size() > config.maximum_candidate_pairs) {
    pairs.resize(config.maximum_candidate_pairs);
    result.pair_budget_exhausted = true;
  }
  result.timings.broadphase_us = elapsed_us(broadphase_start, SteadyClock::now());
  result.candidate_pairs = pairs.size();
  const auto generation_start = SteadyClock::now();
  result.constraints.stretch = segments.size();
  std::vector<std::size_t> degrees(particles.size(), 0U);
  for (const auto& segment : segments) {
    ++degrees[segment.first_particle];
    ++degrees[segment.second_particle];
    result.constraints.obstacle += static_cast<std::size_t>(
        std::count_if(obstacles.begin(), obstacles.end(), [&](const auto& obstacle) {
          return segment.layer == obstacle.layer &&
                 (obstacle.route == std::numeric_limits<std::uint32_t>::max() ||
                  obstacle.route == segment.route);
        }));
  }
  result.constraints.bend = static_cast<std::size_t>(
      std::count(degrees.begin(), degrees.end(), static_cast<std::size_t>(2U)));
  result.constraints.trace_clearance = pairs.size();
  result.constraints.corridor = static_cast<std::size_t>(
      std::count_if(particles.begin(), particles.end(),
                    [](const auto& particle) { return particle.corridor_radius_nm > 0; }));
  std::vector<std::pair<std::int64_t, std::int64_t>> initial_positions;
  initial_positions.reserve(particles.size());
  for (const auto& particle : particles) {
    initial_positions.emplace_back(particle.x_nm, particle.y_nm);
  }
  result.initial =
      measure_positions(initial_positions, particles, segments, obstacles, pairs, config);
  std::array<std::vector<std::uint32_t>, 12U> input;
  input[0].reserve(particles.size() * 4U);
  input[1].resize(particles.size() * 4U);
  input[2].reserve(particles.size() * 4U);
  input[3].reserve(particles.size() * 4U);
  for (const auto& particle : particles) {
    input[0].insert(
        input[0].end(),
        {float_word(static_cast<float>(particle.x_nm - origin_x)),
         float_word(static_cast<float>(particle.y_nm - origin_y)),
         float_word(static_cast<float>(std::max<long double>(0.0L, particle.inverse_mass))),
         float_word(static_cast<float>(std::max<std::int64_t>(0, particle.radius_nm)))});
    input[2].insert(input[2].end(), {particle.route, particle.layer, particle.flags, 0U});
    input[3].insert(
        input[3].end(),
        {float_word(static_cast<float>(particle.corridor_anchor_x_nm - origin_x)),
         float_word(static_cast<float>(particle.corridor_anchor_y_nm - origin_y)),
         float_word(static_cast<float>(std::max<std::int64_t>(0, particle.corridor_radius_nm))),
         0U});
  }
  input[4].reserve(segments.size() * 8U);
  for (const auto& segment : segments) {
    auto rest = segment.rest_length_nm;
    if (rest <= 0) {
      const auto& first = particles[segment.first_particle];
      const auto& second = particles[segment.second_particle];
      rest = static_cast<std::int64_t>(
          std::llround(std::hypotl(static_cast<long double>(second.x_nm - first.x_nm),
                                   static_cast<long double>(second.y_nm - first.y_nm))));
    }
    input[4].insert(
        input[4].end(),
        {segment.first_particle, segment.second_particle, segment.route, segment.layer,
         float_word(static_cast<float>(std::max<std::int64_t>(0, segment.radius_nm))),
         float_word(static_cast<float>(std::max<std::int64_t>(1, rest))),
         float_word(static_cast<float>(std::max<std::int64_t>(0, segment.clearance_nm))), 0U});
  }
  input[5].reserve(obstacles.size() * 12U);
  for (const auto& obstacle : obstacles) {
    input[5].insert(input[5].end(),
                    {float_word(static_cast<float>(obstacle.first_x_nm - origin_x)),
                     float_word(static_cast<float>(obstacle.first_y_nm - origin_y)),
                     float_word(static_cast<float>(obstacle.second_x_nm - origin_x)),
                     float_word(static_cast<float>(obstacle.second_y_nm - origin_y)),
                     float_word(static_cast<float>(std::max<std::int64_t>(0, obstacle.radius_nm))),
                     obstacle.layer, obstacle.route,
                     float_word(static_cast<float>(obstacle.outward_normal_x)),
                     float_word(static_cast<float>(obstacle.outward_normal_y)), 0U, 0U, 0U});
  }
  input[6].reserve(pairs.size() * 2U);
  for (const auto pair : pairs)
    input[6].insert(input[6].end(), {pair.first, pair.second});
  input[7].reserve(std::max<std::size_t>(1U, terrain.pressure.size()));
  for (const auto value : terrain.pressure)
    input[7].push_back(float_word(value));
  if (input[7].empty())
    input[7].push_back(0U);
  input[8].assign(std::max<std::size_t>(1U, pressure_cells), 0U);
  input[9].assign(std::max<std::size_t>(1U, pressure_cells), 0U);
  input[10].assign(std::max<std::size_t>(4U, particles.size() * 4U), 0U);
  input[11].assign(std::max<std::size_t>(4U, config.iterations * 4U), 0U);
  Push push;
  push.particle_count = static_cast<std::uint32_t>(particles.size());
  push.segment_count = static_cast<std::uint32_t>(segments.size());
  push.obstacle_count = static_cast<std::uint32_t>(obstacles.size());
  push.pair_count = static_cast<std::uint32_t>(pairs.size());
  push.terrain_width = static_cast<std::uint32_t>(terrain.width);
  push.terrain_height = static_cast<std::uint32_t>(terrain.height);
  push.terrain_layer_count = static_cast<std::uint32_t>(terrain.layer_count);
  push.pressure_layer_count = static_cast<std::uint32_t>(pressure_layer_count);
  push.terrain_origin_x = static_cast<float>(terrain.origin_x_nm - origin_x);
  push.terrain_origin_y = static_cast<float>(terrain.origin_y_nm - origin_y);
  push.terrain_pitch = static_cast<float>(terrain.pitch_nm);
  push.maximum_step = static_cast<float>(config.maximum_step_nm);
  push.guard_nm = static_cast<float>(config.numerical_guard_nm);
  push.stretch_weight = compliance_weight(config.stretch_compliance);
  push.bend_weight = compliance_weight(config.bend_compliance);
  push.clearance_weight = compliance_weight(config.clearance_compliance);
  push.corridor_weight = compliance_weight(config.corridor_compliance);
  push.terrain_gain = static_cast<float>(config.terrain_gain);
  push.pressure_width = static_cast<std::uint32_t>(config.pressure_width);
  push.pressure_height = static_cast<std::uint32_t>(config.pressure_height);
  push.pressure_origin_x = static_cast<float>(config.pressure_origin_x_nm - origin_x);
  push.pressure_origin_y = static_cast<float>(config.pressure_origin_y_nm - origin_y);
  push.pressure_pitch = static_cast<float>(config.pressure_pitch_nm);
  push.pressure_diffusion = static_cast<float>(std::clamp(config.pressure_diffusion, 0.0L, 0.25L));
  result.timings.constraint_generation_us = elapsed_us(generation_start, SteadyClock::now());
  std::vector<std::uint32_t> output;
  std::vector<std::uint32_t> pressure_output;
  std::vector<std::uint32_t> stress_output;
  std::vector<std::uint32_t> iteration_residuals;
  std::uint64_t solver_readback_us = 0U;
  if (!context().dispatch(input, push, config.iterations, config.pressure_diffusion_passes, output,
                          pressure_output, stress_output, iteration_residuals,
                          config.record_iteration_metrics, result.timings.solve_us,
                          solver_readback_us, result.resident_input_buffer_reuses,
                          result.diagnostic))
    return result;
  result.timings.readback_us += solver_readback_us;
  result.positions_nm.reserve(particles.size());
  result.particle_stress_nm.assign(particles.size(), 0.0L);
  for (std::size_t particle = 0U; particle < particles.size(); ++particle) {
    const auto x = std::bit_cast<float>(output[particle * 4U]);
    const auto y = std::bit_cast<float>(output[particle * 4U + 1U]);
    result.positions_nm.emplace_back(origin_x + static_cast<std::int64_t>(std::llround(x)),
                                     origin_y + static_cast<std::int64_t>(std::llround(y)));
  }
  result.particle_stress_by_type.resize(particles.size());
  for (std::size_t particle = 0U; particle < particles.size(); ++particle) {
    auto& attributed = result.particle_stress_by_type[particle];
    attributed.obstacle_nm = std::bit_cast<float>(stress_output[particle * 4U]);
    attributed.trace_clearance_nm = std::bit_cast<float>(stress_output[particle * 4U + 1U]);
    attributed.stretch_bend_nm = std::bit_cast<float>(stress_output[particle * 4U + 2U]);
    attributed.corridor_nm = std::bit_cast<float>(stress_output[particle * 4U + 3U]);
    if ((particles[particle].flags & routing_xpbd_via) != 0U)
      attributed.via_nm = attributed.total_nm();
    result.stress += attributed;
  }
  if (config.record_iteration_metrics && iteration_residuals.size() == config.iterations * 4U) {
    result.iteration_metrics.reserve(config.iterations);
    for (std::size_t iteration = 0U; iteration < config.iterations; ++iteration) {
      result.iteration_metrics.push_back(
          {iteration_residuals[iteration * 4U],
           static_cast<long double>(iteration_residuals[iteration * 4U + 1U]),
           static_cast<long double>(iteration_residuals[iteration * 4U + 2U])});
    }
  }
  // Compact post-solve residual readback. Exact regional spline/copper checks remain authoritative.
  const auto reduction_start = SteadyClock::now();
  result.final =
      measure_positions(result.positions_nm, particles, segments, obstacles, pairs, config,
                        &result.particle_stress_nm, &result.unresolved_trace_pairs);
  result.timings.stress_reduction_us = elapsed_us(reduction_start, SteadyClock::now());
  result.iterations_executed = config.iterations;
  result.converged = result.final.violations == 0U;
  result.stalled = !result.converged &&
                   result.final.total_residual_nm + 0.5L >= result.initial.total_residual_nm;
  result.topology_escape_required = !result.converged;
  result.used_gpu = true;
  if (pressure_output.size() == pressure_cells && pressure_cells > 0U &&
      config.pressure_pitch_nm > 0) {
    result.pressure_width = config.pressure_width;
    result.pressure_height = config.pressure_height;
    result.pressure_layer_count = pressure_layer_count;
    result.pressure_field.reserve(pressure_cells);
    for (const auto encoded : pressure_output)
      result.pressure_field.push_back(static_cast<float>(encoded) / push.pressure_scale);
    result.used_gpu_pressure_field = true;
  }
  return result;
}

} // namespace flux
