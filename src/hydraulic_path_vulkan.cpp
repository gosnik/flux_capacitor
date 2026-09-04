#include "flux/core/hydraulic_path.hpp"
#include "vulkan_runtime.hpp"

#include <vulkan/vulkan.h>

#include <algorithm>
#include <array>
#include <bit>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <cstring>
#include <deque>
#include <fstream>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <tuple>
#include <utility>
#include <vector>

namespace flux {
namespace {

constexpr std::uint32_t infinite_height = hydraulic_unreachable_height;
constexpr std::array<std::pair<int, int>, 8U> directions{
    std::pair{-1, -1}, std::pair{0, -1}, std::pair{1, -1}, std::pair{-1, 0},
    std::pair{1, 0},   std::pair{-1, 1}, std::pair{0, 1},  std::pair{1, 1}};

struct GpuBuffer {
  VkBuffer buffer{VK_NULL_HANDLE};
  VkDeviceMemory memory{VK_NULL_HANDLE};
  void* mapped{nullptr};
  VkDeviceSize size{0U};
};

struct HydraulicPush {
  std::uint32_t width;
  std::uint32_t height;
  std::uint32_t mode;
  std::uint32_t read_a;
  std::uint32_t source;
  std::uint32_t target;
  std::uint32_t droplets;
  std::uint32_t maximum_steps;
  std::uint32_t seed;
  std::uint32_t batch;
  std::uint32_t erosion_quantum;
  std::uint32_t maximum_erosion;
  std::uint32_t exploration_score_quantum;
  std::uint32_t erosion_height_scale;
  std::uint32_t lateral_erosion_divisor;
  std::uint32_t clearance_score_scale;
  std::uint32_t maximum_clearance_reward_cells;
  std::uint32_t downstream_pressure_score_scale;
  std::uint32_t maximum_downstream_pressure;
  std::uint32_t clearance_read_a;
  std::uint32_t polygon_segment_count;
  std::uint32_t raster_pitch_nm;
  std::uint32_t minimum_boundary_clearance_nm;
  std::uint32_t compact_width;
  std::uint32_t compact_height;
  std::uint32_t compact_stride;
};

constexpr std::size_t maximum_hydraulic_batch = 8U;

struct HydraulicBatchItem {
  const HydraulicPathInput* input{nullptr};
  const HydraulicPathConfig* config{nullptr};
};

struct HydraulicGpuSlot {
  std::array<GpuBuffer, 15U> buffers{};
  VkDescriptorPool descriptor_pool{VK_NULL_HANDLE};
  VkDescriptorSet descriptor_set{VK_NULL_HANDLE};
  std::uint32_t final_read_a{1U};
  std::uint32_t clearance_read_a{1U};
  std::size_t count{0U};
  std::size_t buffer_reuses{0U};
  std::size_t buffer_growths{0U};
  bool active{false};
  bool compute_clearance{false};
  bool clearance_cache_hit{false};
  bool polygon_raster{false};
  bool compact_output{false};
  std::size_t compact_width{0U};
  std::size_t compact_height{0U};
  std::size_t compact_stride{0U};
};

struct HydraulicClearanceCacheEntry {
  std::size_t width{0U};
  std::size_t height{0U};
  std::uint64_t hash{0U};
  std::vector<std::uint8_t> walkable;
  std::vector<std::uint32_t> clearance;
};

[[nodiscard]] std::uint64_t clearance_hash(const HydraulicPathInput& input) {
  std::uint64_t hash = 1'469'598'103'934'665'603ULL;
  const auto absorb = [&](std::uint64_t value) {
    hash ^= value;
    hash *= 1'099'511'628'211ULL;
  };
  absorb(input.width);
  absorb(input.height);
  for (const auto cell : input.walkable)
    absorb(cell);
  return hash;
}

class HydraulicGpuContext {
public:
  HydraulicGpuContext() { initialize(); }
  ~HydraulicGpuContext() { shutdown(); }

  HydraulicGpuContext(const HydraulicGpuContext&) = delete;
  HydraulicGpuContext& operator=(const HydraulicGpuContext&) = delete;

  [[nodiscard]] bool ok() const { return diagnostic_.empty(); }
  [[nodiscard]] const std::string& diagnostic() const { return diagnostic_; }

  [[nodiscard]] std::vector<HydraulicPathResult>
  solve_batch(const std::vector<HydraulicBatchItem>& items) {
    std::vector<HydraulicPathResult> results(items.size());
    if (!ok()) {
      for (auto& result : results)
        result.diagnostic = diagnostic_;
      return results;
    }
    if (items.empty() || items.size() > slots_.size())
      return results;

    const auto groups_for = [](std::size_t work_items) {
      return static_cast<std::uint32_t>((work_items + 255U) / 256U);
    };
    bool any_active = false;
    for (std::size_t item_index = 0U; item_index < items.size(); ++item_index) {
      const auto& input = *items[item_index].input;
      const auto& config = *items[item_index].config;
      auto& slot = slots_[item_index];
      slot.active = false;
      const auto polygon_segment_count =
          input.polygon_raster ? input.polygon_raster->boundary_segments.size() : 0U;
      if (input.width > std::numeric_limits<std::uint32_t>::max() ||
          input.height > std::numeric_limits<std::uint32_t>::max() ||
          input.source > std::numeric_limits<std::uint32_t>::max() ||
          input.target > std::numeric_limits<std::uint32_t>::max() ||
          config.droplets_per_batch > std::numeric_limits<std::uint32_t>::max() ||
          config.maximum_steps_per_droplet > std::numeric_limits<std::uint32_t>::max() ||
          polygon_segment_count > std::numeric_limits<std::uint32_t>::max() ||
          (input.polygon_raster &&
           (input.polygon_raster->pitch_nm <= 0 ||
            input.polygon_raster->pitch_nm > std::numeric_limits<std::uint32_t>::max() ||
            input.polygon_raster->minimum_boundary_clearance_nm < 0 ||
            input.polygon_raster->minimum_boundary_clearance_nm >
                std::numeric_limits<std::uint32_t>::max()))) {
        results[item_index].diagnostic = "hydraulic GPU request exceeds 32-bit shader bounds";
        continue;
      }
      slot.count = input.width * input.height;
      const auto byte_count = slot.count * sizeof(std::uint32_t);
      slot.compact_stride =
          input.compact_output_max_dimension > 0U
              ? std::max<std::size_t>(1U, (std::max(input.width, input.height) +
                                           input.compact_output_max_dimension - 1U) /
                                              input.compact_output_max_dimension)
              : 0U;
      slot.compact_width = slot.compact_stride > 0U
                               ? (input.width + slot.compact_stride - 1U) / slot.compact_stride
                               : 0U;
      slot.compact_height = slot.compact_stride > 0U
                                ? (input.height + slot.compact_stride - 1U) / slot.compact_stride
                                : 0U;
      const auto compact_words = slot.compact_width * slot.compact_height * 7U + 6U;
      const std::array<std::size_t, 15U> sizes{
          byte_count,
          byte_count,
          byte_count,
          byte_count,
          byte_count,
          byte_count,
          2U * sizeof(std::uint32_t),
          byte_count,
          byte_count,
          byte_count,
          std::max<std::size_t>(sizeof(std::uint32_t),
                                polygon_segment_count * 5U * sizeof(std::uint32_t)),
          byte_count,
          std::max<std::size_t>(sizeof(std::uint32_t), compact_words * sizeof(std::uint32_t)),
          (slot.count + 1U) * sizeof(std::uint32_t),
          byte_count};
      bool allocated = true;
      slot.buffer_reuses = 0U;
      slot.buffer_growths = 0U;
      for (std::size_t buffer = 0U; buffer < slot.buffers.size(); ++buffer) {
        if (slot.buffers[buffer].buffer != VK_NULL_HANDLE &&
            slot.buffers[buffer].size >= sizes[buffer])
          ++slot.buffer_reuses;
        else
          ++slot.buffer_growths;
        allocated = ensure_buffer(sizes[buffer], slot.buffers[buffer]) && allocated;
      }
      if (!allocated || !update_descriptor_set(slot)) {
        results[item_index].diagnostic = "hydraulic GPU persistent resources could not be prepared";
        continue;
      }
      std::vector<std::uint32_t> walkable(slot.count, 0U);
      if (!input.walkable.empty())
        std::transform(input.walkable.begin(), input.walkable.end(), walkable.begin(),
                       [](std::uint8_t value) { return value == 0U ? 0U : 1U; });
      std::vector<std::uint32_t> potential(slot.count, infinite_height);
      potential[input.target] = 0U;
      std::vector<std::uint32_t> zero(slot.count, 0U);
      std::vector<std::uint32_t> initial_erosion(slot.count, 0U);
      if (!input.success_prior.empty()) {
        std::transform(input.success_prior.begin(), input.success_prior.end(),
                       initial_erosion.begin(),
                       [&](const auto prior) { return std::min(prior, config.maximum_erosion); });
      }
      std::memcpy(slot.buffers[0].mapped, walkable.data(), byte_count);
      std::memcpy(slot.buffers[1].mapped, potential.data(), byte_count);
      std::memcpy(slot.buffers[2].mapped, potential.data(), byte_count);
      std::memcpy(slot.buffers[3].mapped, initial_erosion.data(), byte_count);
      std::memcpy(slot.buffers[4].mapped, zero.data(), byte_count);
      std::memcpy(slot.buffers[5].mapped, zero.data(), byte_count);
      const std::array<std::uint32_t, 2U> zero_stats{0U, 0U};
      std::memcpy(slot.buffers[6].mapped, zero_stats.data(), sizeof(zero_stats));
      std::vector<std::uint32_t> initial_clearance(slot.count, infinite_height);
      slot.compute_clearance = false;
      slot.clearance_cache_hit = false;
      slot.polygon_raster = input.polygon_raster.has_value();
      slot.compact_output = slot.compact_stride > 0U;
      if (slot.polygon_raster) {
        slot.compute_clearance = false;
        std::vector<std::uint32_t> packed_segments;
        packed_segments.reserve(polygon_segment_count * 5U);
        for (const auto& segment : input.polygon_raster->boundary_segments) {
          packed_segments.insert(packed_segments.end(),
                                 {static_cast<std::uint32_t>(segment.first_x_nm),
                                  static_cast<std::uint32_t>(segment.first_y_nm),
                                  static_cast<std::uint32_t>(segment.second_x_nm),
                                  static_cast<std::uint32_t>(segment.second_y_nm),
                                  segment.obstacle_id});
        }
        std::memcpy(slot.buffers[10].mapped, packed_segments.data(),
                    packed_segments.size() * sizeof(std::uint32_t));
      } else if (!input.obstacle_clearance.empty()) {
        initial_clearance = input.obstacle_clearance;
      } else {
        const auto hash = clearance_hash(input);
        const auto cached =
            std::find_if(clearance_cache_.begin(), clearance_cache_.end(), [&](const auto& entry) {
              return entry.hash == hash && entry.width == input.width &&
                     entry.height == input.height && entry.walkable == input.walkable;
            });
        if (cached != clearance_cache_.end()) {
          initial_clearance = cached->clearance;
          slot.clearance_cache_hit = true;
        } else {
          slot.compute_clearance = true;
          for (std::size_t cell = 0U; cell < slot.count; ++cell) {
            const auto x = cell % input.width;
            const auto y = cell / input.width;
            if (input.walkable[cell] == 0U || x == 0U || y == 0U || x + 1U == input.width ||
                y + 1U == input.height) {
              initial_clearance[cell] = 0U;
            }
          }
        }
      }
      std::memcpy(slot.buffers[7].mapped, initial_clearance.data(), byte_count);
      const auto& downstream = input.downstream_pressure.empty() ? zero : input.downstream_pressure;
      std::memcpy(slot.buffers[8].mapped, downstream.data(), byte_count);
      std::memcpy(slot.buffers[9].mapped, initial_clearance.data(), byte_count);
      const auto& success = input.success_prior.empty() ? zero : input.success_prior;
      std::memcpy(slot.buffers[14].mapped, success.data(), byte_count);
      if (slot.compact_output)
        std::memset(slot.buffers[12].mapped, 0, compact_words * sizeof(std::uint32_t));
      std::memset(slot.buffers[13].mapped, 0, (slot.count + 1U) * sizeof(std::uint32_t));
      slot.final_read_a = 1U;
      slot.clearance_read_a = 1U;
      slot.active = true;
      any_active = true;
    }
    if (!any_active)
      return results;
    if (!prepare_command_buffer()) {
      for (std::size_t index = 0U; index < items.size(); ++index)
        if (slots_[index].active)
          results[index].diagnostic = "hydraulic GPU command allocation failed";
      return results;
    }
    VkCommandBufferBeginInfo begin{VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO};
    if (vkBeginCommandBuffer(command_, &begin) != VK_SUCCESS) {
      for (std::size_t index = 0U; index < items.size(); ++index)
        if (slots_[index].active)
          results[index].diagnostic = "hydraulic GPU command recording failed";
      return results;
    }
    vkCmdBindPipeline(command_, VK_PIPELINE_BIND_POINT_COMPUTE, pipeline_);
    const auto barrier = [&] {
      VkMemoryBarrier memory{VK_STRUCTURE_TYPE_MEMORY_BARRIER};
      memory.srcAccessMask = VK_ACCESS_SHADER_WRITE_BIT;
      memory.dstAccessMask = VK_ACCESS_SHADER_READ_BIT | VK_ACCESS_SHADER_WRITE_BIT;
      vkCmdPipelineBarrier(command_, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
                           VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, 0U, 1U, &memory, 0U, nullptr, 0U,
                           nullptr);
    };
    for (std::size_t item_index = 0U; item_index < items.size(); ++item_index) {
      auto& slot = slots_[item_index];
      if (!slot.active)
        continue;
      const auto& input = *items[item_index].input;
      const auto& config = *items[item_index].config;
      vkCmdBindDescriptorSets(command_, VK_PIPELINE_BIND_POINT_COMPUTE, pipeline_layout_, 0U, 1U,
                              &slot.descriptor_set, 0U, nullptr);
      HydraulicPush push{
          static_cast<std::uint32_t>(input.width),
          static_cast<std::uint32_t>(input.height),
          0U,
          1U,
          static_cast<std::uint32_t>(input.source),
          static_cast<std::uint32_t>(input.target),
          static_cast<std::uint32_t>(config.droplets_per_batch),
          static_cast<std::uint32_t>(config.maximum_steps_per_droplet),
          static_cast<std::uint32_t>(config.deterministic_seed ^
                                     (config.deterministic_seed >> 32U)),
          0U,
          config.erosion_per_flow_quantum,
          config.maximum_erosion,
          config.exploration_score_quantum,
          config.erosion_height_scale,
          config.lateral_erosion_divisor,
          config.clearance_score_scale,
          config.maximum_clearance_reward_cells,
          config.downstream_pressure_score_scale,
          config.maximum_downstream_pressure,
          1U,
          static_cast<std::uint32_t>(
              input.polygon_raster ? input.polygon_raster->boundary_segments.size() : 0U),
          static_cast<std::uint32_t>(input.polygon_raster ? input.polygon_raster->pitch_nm : 1),
          static_cast<std::uint32_t>(
              input.polygon_raster ? input.polygon_raster->minimum_boundary_clearance_nm : 0),
          static_cast<std::uint32_t>(slot.compact_width),
          static_cast<std::uint32_t>(slot.compact_height),
          static_cast<std::uint32_t>(slot.compact_stride)};
      if (slot.polygon_raster) {
        push.mode = 5U;
        vkCmdPushConstants(command_, pipeline_layout_, VK_SHADER_STAGE_COMPUTE_BIT, 0U,
                           sizeof(push), &push);
        vkCmdDispatch(command_, groups_for(slot.count), 1U, 1U);
        barrier();
      }
      if (slot.compute_clearance) {
        for (std::size_t pass = 0U; pass < std::max(input.width, input.height); ++pass) {
          push.mode = 4U;
          vkCmdPushConstants(command_, pipeline_layout_, VK_SHADER_STAGE_COMPUTE_BIT, 0U,
                             sizeof(push), &push);
          vkCmdDispatch(command_, groups_for(slot.count), 1U, 1U);
          barrier();
          push.clearance_read_a ^= 1U;
        }
      }
      slot.clearance_read_a = push.clearance_read_a;
      for (std::size_t pass = 0U; pass < config.relaxation_iterations; ++pass) {
        push.mode = 0U;
        vkCmdPushConstants(command_, pipeline_layout_, VK_SHADER_STAGE_COMPUTE_BIT, 0U,
                           sizeof(push), &push);
        vkCmdDispatch(command_, groups_for(slot.count), 1U, 1U);
        barrier();
        push.read_a ^= 1U;
      }
      slot.final_read_a = push.read_a;
      results[item_index].relaxation_passes = config.relaxation_iterations;
      for (std::size_t batch = 0U; batch < config.droplet_batches; ++batch) {
        push.batch = static_cast<std::uint32_t>(batch);
        for (const auto mode : {1U, 2U, 3U}) {
          push.mode = mode;
          vkCmdPushConstants(command_, pipeline_layout_, VK_SHADER_STAGE_COMPUTE_BIT, 0U,
                             sizeof(push), &push);
          vkCmdDispatch(command_, groups_for(mode == 2U ? config.droplets_per_batch : slot.count),
                        1U, 1U);
          barrier();
        }
      }
      if (slot.compact_output) {
        push.mode = 6U;
        vkCmdPushConstants(command_, pipeline_layout_, VK_SHADER_STAGE_COMPUTE_BIT, 0U,
                           sizeof(push), &push);
        vkCmdDispatch(command_, groups_for(slot.compact_width * slot.compact_height), 1U, 1U);
        barrier();
        push.mode = 8U;
        vkCmdPushConstants(command_, pipeline_layout_, VK_SHADER_STAGE_COMPUTE_BIT, 0U,
                           sizeof(push), &push);
        vkCmdDispatch(command_, groups_for(slot.compact_width * slot.compact_height), 1U, 1U);
        barrier();
        push.mode = 7U;
        vkCmdPushConstants(command_, pipeline_layout_, VK_SHADER_STAGE_COMPUTE_BIT, 0U,
                           sizeof(push), &push);
        vkCmdDispatch(command_, 1U, 1U, 1U);
        barrier();
      }
    }
    if (vkEndCommandBuffer(command_) != VK_SUCCESS) {
      for (std::size_t index = 0U; index < items.size(); ++index)
        if (slots_[index].active)
          results[index].diagnostic = "hydraulic GPU command finalization failed";
      return results;
    }
    std::string dispatch_error;
    if (!vulkan::Runtime::instance().submit_and_wait(command_, dispatch_error)) {
      for (std::size_t index = 0U; index < items.size(); ++index)
        if (slots_[index].active)
          results[index].diagnostic = "hydraulic GPU dispatch failed: " + dispatch_error;
      return results;
    }
    for (std::size_t item_index = 0U; item_index < items.size(); ++item_index) {
      auto& slot = slots_[item_index];
      auto& result = results[item_index];
      if (!slot.active || !result.diagnostic.empty())
        continue;
      const auto& input = *items[item_index].input;
      const auto& config = *items[item_index].config;
      const auto bytes = slot.count * sizeof(std::uint32_t);
      result.used_gpu_polygon_raster = slot.polygon_raster;
      result.used_gpu_compaction = slot.compact_output;
      result.used_gpu_clearance_field = input.obstacle_clearance.empty();
      result.gpu_clearance_cache_hit = slot.clearance_cache_hit;
      result.gpu_batch_size = items.size();
      result.gpu_persistent_buffer_reuses = slot.buffer_reuses;
      result.gpu_buffer_growths = slot.buffer_growths;
      if (slot.compute_clearance && !slot.compact_output) {
        result.obstacle_clearance.resize(slot.count);
        std::memcpy(result.obstacle_clearance.data(),
                    slot.clearance_read_a != 0U ? slot.buffers[7].mapped : slot.buffers[9].mapped,
                    bytes);
        const auto hash = clearance_hash(input);
        const auto existing =
            std::find_if(clearance_cache_.begin(), clearance_cache_.end(), [&](const auto& entry) {
              return entry.hash == hash && entry.width == input.width &&
                     entry.height == input.height && entry.walkable == input.walkable;
            });
        if (existing == clearance_cache_.end()) {
          if (clearance_cache_.size() >= maximum_clearance_cache_entries)
            clearance_cache_.pop_front();
          clearance_cache_.push_back(HydraulicClearanceCacheEntry{
              input.width, input.height, hash, input.walkable, result.obstacle_clearance});
        }
      }
      result.downstream_pressure = input.downstream_pressure;
      if (result.downstream_pressure.empty())
        result.downstream_pressure.assign(slot.count, 0U);
      result.success_prior = input.success_prior;
      if (result.success_prior.empty())
        result.success_prior.assign(slot.count, 0U);
      std::array<std::uint32_t, 2U> statistics{};
      std::memcpy(statistics.data(), slot.buffers[6].mapped, sizeof(statistics));
      result.droplets_reaching_target = statistics[0];
      result.droplet_steps = statistics[1];
      const auto* final_potential = static_cast<const std::uint32_t*>(
          slot.final_read_a != 0U ? slot.buffers[1].mapped : slot.buffers[2].mapped);
      if (final_potential[input.source] == infinite_height) {
        result.budget_exhausted = true;
        result.diagnostic = "hydraulic GPU potential relaxation budget exhausted";
      } else if (slot.compact_output) {
        const auto compact_count = slot.compact_width * slot.compact_height;
        const auto* compact = static_cast<const std::uint32_t*>(slot.buffers[12].mapped);
        const auto summary = compact_count * 7U;
        result.maximum_flow = compact[summary];
        result.maximum_erosion = compact[summary + 1U];
        result.maximum_terrain = compact[summary + 2U];
        result.maximum_downstream_pressure = compact[summary + 3U];
        result.maximum_success_prior = compact[summary + 4U];
        result.compact_width = slot.compact_width;
        result.compact_height = slot.compact_height;
        result.compact_stride = slot.compact_stride;
        result.compact_flow.resize(compact_count);
        result.compact_erosion.resize(compact_count);
        result.compact_terrain.resize(compact_count);
        result.compact_free_space.resize(compact_count);
        result.compact_downstream_pressure.resize(compact_count);
        result.compact_success_prior.resize(compact_count);
        for (std::size_t cell = 0U; cell < compact_count; ++cell) {
          const auto base = cell * 7U;
          result.compact_flow[cell] = static_cast<std::uint16_t>(compact[base]);
          result.compact_erosion[cell] = static_cast<std::uint16_t>(compact[base + 1U]);
          result.compact_terrain[cell] = static_cast<std::uint16_t>(compact[base + 2U]);
          result.compact_free_space[cell] = static_cast<std::uint16_t>(compact[base + 3U]);
          result.compact_downstream_pressure[cell] = static_cast<std::uint16_t>(compact[base + 5U]);
          result.compact_success_prior[cell] = static_cast<std::uint16_t>(compact[base + 6U]);
        }
        const auto* riverbed = static_cast<const std::uint32_t*>(slot.buffers[13].mapped);
        const auto length = std::min<std::size_t>(riverbed[0], slot.count);
        result.riverbed.assign(riverbed + 1U, riverbed + 1U + length);
        if (result.riverbed.size() < 2U || result.riverbed.front() != input.source ||
            result.riverbed.back() != input.target) {
          result.riverbed.clear();
          result.diagnostic = "hydraulic GPU predecessor tracing did not reach its target";
        }
      } else {
        result.static_potential.assign(final_potential, final_potential + slot.count);
        result.erosion.resize(slot.count);
        result.accumulated_flow.resize(slot.count);
        result.obstacle_clearance.resize(slot.count);
        result.walkable.resize(slot.count);
        result.nearest_obstacle_id.resize(slot.count);
        std::memcpy(result.erosion.data(), slot.buffers[3].mapped, bytes);
        std::memcpy(result.accumulated_flow.data(), slot.buffers[5].mapped, bytes);
        std::memcpy(result.obstacle_clearance.data(),
                    slot.clearance_read_a != 0U ? slot.buffers[7].mapped : slot.buffers[9].mapped,
                    bytes);
        const auto* walkable = static_cast<const std::uint32_t*>(slot.buffers[0].mapped);
        const auto* obstacles = static_cast<const std::uint32_t*>(slot.buffers[11].mapped);
        std::transform(walkable, walkable + slot.count, result.walkable.begin(),
                       [](std::uint32_t value) { return value != 0U ? 1U : 0U; });
        std::copy(obstacles, obstacles + slot.count, result.nearest_obstacle_id.begin());
        result.terrain_height.assign(slot.count, infinite_height);
        for (std::size_t cell = 0U; cell < slot.count; ++cell) {
          if (result.walkable[cell] == 0U || result.static_potential[cell] == infinite_height)
            continue;
          const auto reduction = static_cast<std::uint64_t>(result.erosion[cell]) *
                                 config.erosion_height_scale / 1'024ULL;
          result.terrain_height[cell] = static_cast<std::uint32_t>(
              reduction < result.static_potential[cell] ? result.static_potential[cell] - reduction
                                                        : 0U);
        }
        auto extraction_input = input;
        extraction_input.walkable = result.walkable;
        extract_riverbed(extraction_input, result.static_potential, result.obstacle_clearance,
                         result);
      }
    }
    return results;
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
    queue_family_ = runtime.queue_family();
    VkCommandPoolCreateInfo pool_info{VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO};
    pool_info.queueFamilyIndex = queue_family_;
    pool_info.flags = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT;
    if (vkCreateCommandPool(device_, &pool_info, nullptr, &command_pool_) != VK_SUCCESS) {
      diagnostic_ = "Vulkan command pool creation failed";
      return;
    }
    std::array<VkDescriptorSetLayoutBinding, 15U> bindings{};
    for (std::uint32_t index = 0U; index < bindings.size(); ++index) {
      bindings[index].binding = index;
      bindings[index].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
      bindings[index].descriptorCount = 1U;
      bindings[index].stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
    }
    VkDescriptorSetLayoutCreateInfo descriptor_info{
        VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO};
    descriptor_info.bindingCount = static_cast<std::uint32_t>(bindings.size());
    descriptor_info.pBindings = bindings.data();
    if (vkCreateDescriptorSetLayout(device_, &descriptor_info, nullptr, &descriptor_layout_) !=
        VK_SUCCESS) {
      diagnostic_ = "Vulkan hydraulic descriptor layout creation failed";
      return;
    }
    VkPushConstantRange push_range{VK_SHADER_STAGE_COMPUTE_BIT, 0U, sizeof(HydraulicPush)};
    VkPipelineLayoutCreateInfo pipeline_layout_info{VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO};
    pipeline_layout_info.setLayoutCount = 1U;
    pipeline_layout_info.pSetLayouts = &descriptor_layout_;
    pipeline_layout_info.pushConstantRangeCount = 1U;
    pipeline_layout_info.pPushConstantRanges = &push_range;
    if (vkCreatePipelineLayout(device_, &pipeline_layout_info, nullptr, &pipeline_layout_) !=
        VK_SUCCESS) {
      diagnostic_ = "Vulkan hydraulic pipeline layout creation failed";
      return;
    }
    std::ifstream stream(FLUX_HYDRAULIC_SHADER_PATH, std::ios::binary | std::ios::ate);
    if (!stream) {
      diagnostic_ = "compiled Vulkan hydraulic shader is unavailable";
      return;
    }
    const auto size = stream.tellg();
    if (size <= 0 || size % static_cast<std::streamoff>(sizeof(std::uint32_t)) != 0) {
      diagnostic_ = "compiled Vulkan hydraulic shader is invalid";
      return;
    }
    std::vector<std::uint32_t> code(static_cast<std::size_t>(size) / sizeof(std::uint32_t));
    stream.seekg(0);
    stream.read(reinterpret_cast<char*>(code.data()), size);
    VkShaderModuleCreateInfo module_info{VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO};
    module_info.codeSize = static_cast<std::size_t>(size);
    module_info.pCode = code.data();
    VkShaderModule module = VK_NULL_HANDLE;
    if (vkCreateShaderModule(device_, &module_info, nullptr, &module) != VK_SUCCESS) {
      diagnostic_ = "Vulkan hydraulic shader module creation failed";
      return;
    }
    VkPipelineShaderStageCreateInfo stage{VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO};
    stage.stage = VK_SHADER_STAGE_COMPUTE_BIT;
    stage.module = module;
    stage.pName = "main";
    VkComputePipelineCreateInfo pipeline_info{VK_STRUCTURE_TYPE_COMPUTE_PIPELINE_CREATE_INFO};
    pipeline_info.stage = stage;
    pipeline_info.layout = pipeline_layout_;
    const auto status = runtime.create_compute_pipeline(pipeline_info, pipeline_);
    vkDestroyShaderModule(device_, module, nullptr);
    if (status != VK_SUCCESS) {
      diagnostic_ = "Vulkan hydraulic compute pipeline creation failed";
    }
  }

  void shutdown() {
    for (auto& slot : slots_) {
      destroy_buffers(slot.buffers);
      if (slot.descriptor_pool != VK_NULL_HANDLE)
        vkDestroyDescriptorPool(device_, slot.descriptor_pool, nullptr);
    }
    if (pipeline_ != VK_NULL_HANDLE)
      vkDestroyPipeline(device_, pipeline_, nullptr);
    if (pipeline_layout_ != VK_NULL_HANDLE)
      vkDestroyPipelineLayout(device_, pipeline_layout_, nullptr);
    if (descriptor_layout_ != VK_NULL_HANDLE)
      vkDestroyDescriptorSetLayout(device_, descriptor_layout_, nullptr);
    if (command_pool_ != VK_NULL_HANDLE)
      vkDestroyCommandPool(device_, command_pool_, nullptr);
  }

  [[nodiscard]] bool create_buffer(VkDeviceSize size, GpuBuffer& output) const {
    output.size = size;
    return vulkan::Runtime::instance().allocate_host_buffer(size, output.buffer, output.memory,
                                                            output.mapped);
  }

  void destroy_buffer(GpuBuffer& buffer) const {
    vulkan::Runtime::instance().release_buffer(buffer.buffer, buffer.memory, buffer.mapped);
    buffer = {};
  }

  [[nodiscard]] bool ensure_buffer(std::size_t required, GpuBuffer& buffer) const {
    if (buffer.buffer != VK_NULL_HANDLE && buffer.size >= required)
      return true;
    const auto old_capacity = static_cast<std::size_t>(buffer.size);
    destroy_buffer(buffer);
    const auto target = std::max(required, old_capacity > 0U ? old_capacity * 2U : 4'096U);
    const auto capacity = std::bit_ceil(target);
    return create_buffer(static_cast<VkDeviceSize>(capacity), buffer);
  }

  template <std::size_t Size> void destroy_buffers(std::array<GpuBuffer, Size>& buffers) const {
    for (auto& buffer : buffers) {
      destroy_buffer(buffer);
    }
  }

  [[nodiscard]] bool update_descriptor_set(HydraulicGpuSlot& slot) const {
    if (slot.descriptor_pool == VK_NULL_HANDLE) {
      VkDescriptorPoolSize pool_size{VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,
                                     static_cast<std::uint32_t>(slot.buffers.size())};
      VkDescriptorPoolCreateInfo pool_info{VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO};
      pool_info.maxSets = 1U;
      pool_info.poolSizeCount = 1U;
      pool_info.pPoolSizes = &pool_size;
      if (vkCreateDescriptorPool(device_, &pool_info, nullptr, &slot.descriptor_pool) != VK_SUCCESS)
        return false;
      VkDescriptorSetAllocateInfo allocate{VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO};
      allocate.descriptorPool = slot.descriptor_pool;
      allocate.descriptorSetCount = 1U;
      allocate.pSetLayouts = &descriptor_layout_;
      if (vkAllocateDescriptorSets(device_, &allocate, &slot.descriptor_set) != VK_SUCCESS)
        return false;
    }
    std::array<VkDescriptorBufferInfo, 15U> buffer_info{};
    std::array<VkWriteDescriptorSet, 15U> writes{};
    for (std::uint32_t index = 0U; index < slot.buffers.size(); ++index) {
      buffer_info[index] = {slot.buffers[index].buffer, 0U, slot.buffers[index].size};
      writes[index].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
      writes[index].dstSet = slot.descriptor_set;
      writes[index].dstBinding = index;
      writes[index].descriptorCount = 1U;
      writes[index].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
      writes[index].pBufferInfo = &buffer_info[index];
    }
    vkUpdateDescriptorSets(device_, static_cast<std::uint32_t>(writes.size()), writes.data(), 0U,
                           nullptr);
    return true;
  }

  [[nodiscard]] bool prepare_command_buffer() {
    if (command_ != VK_NULL_HANDLE)
      return vkResetCommandBuffer(command_, 0U) == VK_SUCCESS;
    VkCommandBufferAllocateInfo allocate{VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO};
    allocate.commandPool = command_pool_;
    allocate.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
    allocate.commandBufferCount = 1U;
    return vkAllocateCommandBuffers(device_, &allocate, &command_) == VK_SUCCESS;
  }

  static void extract_riverbed(const HydraulicPathInput& input,
                               const std::vector<std::uint32_t>& potential,
                               const std::vector<std::uint32_t>& clearance,
                               HydraulicPathResult& result) {
    result.riverbed = {input.source};
    auto cell = input.source;
    while (cell != input.target && result.riverbed.size() <= input.walkable.size()) {
      const auto x = cell % input.width;
      const auto y = cell / input.width;
      std::optional<
          std::tuple<std::uint32_t, std::uint32_t, std::uint32_t, std::uint32_t, std::size_t>>
          selected;
      for (const auto [dx, dy] : directions) {
        const auto next_x = static_cast<std::int64_t>(x) + dx;
        const auto next_y = static_cast<std::int64_t>(y) + dy;
        if (next_x < 0 || next_y < 0 || next_x >= static_cast<std::int64_t>(input.width) ||
            next_y >= static_cast<std::int64_t>(input.height)) {
          continue;
        }
        const auto neighbour =
            static_cast<std::size_t>(next_y) * input.width + static_cast<std::size_t>(next_x);
        if (dx != 0 && dy != 0) {
          const auto horizontal = y * input.width + static_cast<std::size_t>(next_x);
          const auto vertical = static_cast<std::size_t>(next_y) * input.width + x;
          if (input.walkable[horizontal] == 0U || input.walkable[vertical] == 0U)
            continue;
        }
        if (input.walkable[neighbour] == 0U || potential[neighbour] >= potential[cell])
          continue;
        const auto candidate = std::tuple{
            std::numeric_limits<std::uint32_t>::max() - result.accumulated_flow[neighbour],
            std::numeric_limits<std::uint32_t>::max() - result.erosion[neighbour],
            std::numeric_limits<std::uint32_t>::max() - clearance[neighbour], potential[neighbour],
            neighbour};
        if (!selected || candidate < *selected)
          selected = candidate;
      }
      if (!selected) {
        result.riverbed.clear();
        result.diagnostic = "hydraulic GPU riverbed extraction reached a local minimum";
        return;
      }
      cell = std::get<4>(*selected);
      result.riverbed.push_back(cell);
    }
    if (result.riverbed.empty() || result.riverbed.back() != input.target) {
      result.riverbed.clear();
      result.diagnostic = "hydraulic GPU riverbed did not reach its target";
    }
  }

  VkInstance instance_{VK_NULL_HANDLE};
  VkPhysicalDevice physical_{VK_NULL_HANDLE};
  VkDevice device_{VK_NULL_HANDLE};
  VkQueue queue_{VK_NULL_HANDLE};
  std::uint32_t queue_family_{0U};
  VkCommandPool command_pool_{VK_NULL_HANDLE};
  VkCommandBuffer command_{VK_NULL_HANDLE};
  VkDescriptorSetLayout descriptor_layout_{VK_NULL_HANDLE};
  VkPipelineLayout pipeline_layout_{VK_NULL_HANDLE};
  VkPipeline pipeline_{VK_NULL_HANDLE};
  std::array<HydraulicGpuSlot, maximum_hydraulic_batch> slots_{};
  static constexpr std::size_t maximum_clearance_cache_entries = 16U;
  std::deque<HydraulicClearanceCacheEntry> clearance_cache_;
  std::string diagnostic_;
};

struct PendingHydraulicRequest {
  const HydraulicPathInput* input{nullptr};
  const HydraulicPathConfig* config{nullptr};
  HydraulicPathResult result;
  std::mutex mutex;
  std::condition_variable completed;
  bool done{false};
};

class HydraulicBatchExecutor {
public:
  HydraulicBatchExecutor() : worker_([this](std::stop_token token) { run(token); }) {}
  ~HydraulicBatchExecutor() {
    worker_.request_stop();
    queued_.notify_all();
  }

  [[nodiscard]] HydraulicPathResult solve(const HydraulicPathInput& input,
                                          const HydraulicPathConfig& config) {
    PendingHydraulicRequest request{&input, &config};
    {
      const std::lock_guard lock(queue_mutex_);
      queue_.push_back(&request);
    }
    queued_.notify_one();
    std::unique_lock lock(request.mutex);
    request.completed.wait(lock, [&] { return request.done; });
    return std::move(request.result);
  }

private:
  void run(std::stop_token token) {
    while (!token.stop_requested()) {
      std::vector<PendingHydraulicRequest*> pending;
      {
        std::unique_lock lock(queue_mutex_);
        queued_.wait(lock, [&] { return token.stop_requested() || !queue_.empty(); });
        if (token.stop_requested() && queue_.empty())
          return;
        // A short bounded collection window lets concurrent static-seed workers join one Vulkan
        // command buffer without making an isolated request noticeably latent.
        if (queue_.size() < maximum_hydraulic_batch) {
          queued_.wait_for(lock, std::chrono::microseconds{750}, [&] {
            return token.stop_requested() || queue_.size() >= maximum_hydraulic_batch;
          });
        }
        const auto count = std::min(queue_.size(), maximum_hydraulic_batch);
        pending.reserve(count);
        for (std::size_t index = 0U; index < count; ++index) {
          pending.push_back(queue_.front());
          queue_.pop_front();
        }
      }
      std::vector<HydraulicBatchItem> items;
      items.reserve(pending.size());
      for (const auto* request : pending)
        items.push_back({request->input, request->config});
      auto results = context_.solve_batch(items);
      for (std::size_t index = 0U; index < pending.size(); ++index) {
        auto& request = *pending[index];
        {
          const std::lock_guard lock(request.mutex);
          request.result = std::move(results[index]);
          request.done = true;
        }
        request.completed.notify_one();
      }
    }
  }

  HydraulicGpuContext context_;
  std::mutex queue_mutex_;
  std::condition_variable queued_;
  std::deque<PendingHydraulicRequest*> queue_;
  std::jthread worker_;
};

[[nodiscard]] HydraulicBatchExecutor& hydraulic_executor() {
  static HydraulicBatchExecutor executor;
  return executor;
}

} // namespace

HydraulicPathResult solve_hydraulic_gpu(const HydraulicPathInput& input,
                                        const HydraulicPathConfig& config) {
  return hydraulic_executor().solve(input, config);
}

bool hydraulic_gpu_runtime_available() {
  // Deliberately instantiate the same minimal Vulkan context used by routing. A build-time Vulkan
  // SDK is not sufficient evidence that the machine can execute Flux.
  static const bool available = [] {
    HydraulicPathInput probe;
    probe.width = 2U;
    probe.height = 1U;
    probe.walkable = {1U, 1U};
    probe.obstacle_clearance = {1U, 1U};
    probe.source = 0U;
    probe.target = 1U;
    HydraulicPathConfig config;
    config.relaxation_iterations = 2U;
    config.droplet_batches = 1U;
    config.droplets_per_batch = 1U;
    config.maximum_steps_per_droplet = 2U;
    return solve_hydraulic_gpu(probe, config).ok();
  }();
  return available;
}

} // namespace flux
