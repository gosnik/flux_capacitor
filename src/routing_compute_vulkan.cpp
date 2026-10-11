#include "flux/core/routing_compute.hpp"
#include "flux/core/routing_kernel.hpp"
#include "vulkan_runtime.hpp"

#include <vulkan/vulkan.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <limits>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

namespace flux {
namespace {

[[nodiscard]] std::optional<std::int32_t> relative_coordinate(std::int64_t value,
                                                              std::int64_t origin) {
  if (value >= origin) {
    const auto distance = static_cast<std::uint64_t>(value) - static_cast<std::uint64_t>(origin);
    if (distance > static_cast<std::uint64_t>(std::numeric_limits<std::int32_t>::max()))
      return std::nullopt;
    return static_cast<std::int32_t>(distance);
  }
  const auto distance = static_cast<std::uint64_t>(origin) - static_cast<std::uint64_t>(value);
  constexpr auto negative_limit =
      static_cast<std::uint64_t>(std::numeric_limits<std::int32_t>::max()) + 1U;
  if (distance > negative_limit)
    return std::nullopt;
  return distance == negative_limit ? std::numeric_limits<std::int32_t>::min()
                                    : -static_cast<std::int32_t>(distance);
}

struct Buffer {
  VkBuffer handle{VK_NULL_HANDLE};
  VkDeviceMemory memory{VK_NULL_HANDLE};
  void* mapped{nullptr};
  VkDeviceSize size{0U};
};

struct Push {
  std::uint32_t mode{0U};
  std::uint32_t count_a{0U};
  std::uint32_t count_b{0U};
  std::uint32_t width{0U};
  std::uint32_t height{0U};
  std::uint32_t read_state{0U};
  std::uint32_t guard_nm{0U};
  std::uint32_t pass{0U};
  std::array<std::uint32_t, 8U> weights{};
};

class Context {
public:
  Context() { initialize(); }
  ~Context() { shutdown(); }
  Context(const Context&) = delete;
  Context& operator=(const Context&) = delete;

  [[nodiscard]] bool ok() const { return diagnostic_.empty(); }
  [[nodiscard]] const std::string& diagnostic() const { return diagnostic_; }

  [[nodiscard]] bool dispatch(const std::vector<std::uint32_t>& first,
                              const std::vector<std::uint32_t>& second,
                              const std::vector<std::uint32_t>& initial_state,
                              std::size_t output_words, Push push, std::size_t work_items,
                              std::size_t passes, std::vector<std::uint32_t>& output,
                              bool& final_in_state, std::string& error) {
    if (!ok()) {
      error = diagnostic_;
      return false;
    }
    if (work_items == 0U || output_words == 0U ||
        work_items > std::numeric_limits<std::uint32_t>::max()) {
      error = "invalid routing compute dispatch";
      return false;
    }
    const std::array<std::size_t, 4U> words{
        std::max<std::size_t>(1U, first.size()), std::max<std::size_t>(1U, second.size()),
        std::max<std::size_t>(1U, initial_state.size()), std::max<std::size_t>(1U, output_words)};
    for (std::size_t index = 0U; index < buffers_.size(); ++index) {
      if (!ensure_buffer(words[index] * sizeof(std::uint32_t), buffers_[index])) {
        error = "routing compute buffer allocation failed";
        return false;
      }
    }
    if (!first.empty())
      std::memcpy(buffers_[0].mapped, first.data(), first.size() * sizeof(std::uint32_t));
    if (!second.empty())
      std::memcpy(buffers_[1].mapped, second.data(), second.size() * sizeof(std::uint32_t));
    if (!initial_state.empty()) {
      std::memcpy(buffers_[2].mapped, initial_state.data(),
                  initial_state.size() * sizeof(std::uint32_t));
      std::memcpy(buffers_[3].mapped, initial_state.data(),
                  std::min(initial_state.size(), output_words) * sizeof(std::uint32_t));
    }

    if (!update_descriptor_set(buffers_) || !prepare_command_buffer()) {
      error = "routing compute command resources failed";
      return false;
    }
    VkCommandBufferBeginInfo begin{VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO};
    if (vkBeginCommandBuffer(command_, &begin) != VK_SUCCESS) {
      error = "routing compute command recording failed";
      return false;
    }
    vkCmdBindPipeline(command_, VK_PIPELINE_BIND_POINT_COMPUTE, pipeline_);
    vkCmdBindDescriptorSets(command_, VK_PIPELINE_BIND_POINT_COMPUTE, layout_, 0U, 1U,
                            &descriptor_set_, 0U, nullptr);
    const auto groups = static_cast<std::uint32_t>((work_items + 255U) / 256U);
    const auto barrier = [&] {
      VkMemoryBarrier memory{VK_STRUCTURE_TYPE_MEMORY_BARRIER};
      memory.srcAccessMask = VK_ACCESS_SHADER_WRITE_BIT;
      memory.dstAccessMask = VK_ACCESS_SHADER_READ_BIT | VK_ACCESS_SHADER_WRITE_BIT;
      vkCmdPipelineBarrier(command_, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
                           VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, 0U, 1U, &memory, 0U, nullptr, 0U,
                           nullptr);
    };
    const auto dispatches = std::max<std::size_t>(1U, passes);
    for (std::size_t pass = 0U; pass < dispatches; ++pass) {
      push.pass = static_cast<std::uint32_t>(pass);
      vkCmdPushConstants(command_, layout_, VK_SHADER_STAGE_COMPUTE_BIT, 0U, sizeof(push), &push);
      vkCmdDispatch(command_, groups, 1U, 1U);
      barrier();
      if (push.mode == 3U || push.mode == 5U || push.mode == 11U || push.mode == 14U)
        push.read_state = push.read_state == 0U ? 1U : 0U;
    }
    if (vkEndCommandBuffer(command_) != VK_SUCCESS) {
      error = "routing compute command finalization failed";
      return false;
    }
    if (!vulkan::Runtime::instance().submit_and_wait(command_, error)) {
      error = "routing compute dispatch failed: " + error;
      return false;
    }
    final_in_state =
        (push.mode == 3U || push.mode == 5U || push.mode == 11U || push.mode == 14U) &&
        dispatches % 2U == 0U;
    const auto* mapped = final_in_state ? buffers_[2].mapped : buffers_[3].mapped;
    output.resize(output_words);
    std::memcpy(output.data(), mapped, output_words * sizeof(std::uint32_t));
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
    VkCommandPoolCreateInfo command_pool{VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO};
    command_pool.queueFamilyIndex = family_;
    command_pool.flags = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT;
    if (vkCreateCommandPool(device_, &command_pool, nullptr, &command_pool_) != VK_SUCCESS) {
      diagnostic_ = "routing compute command pool failed";
      return;
    }
    std::array<VkDescriptorSetLayoutBinding, 4U> bindings{};
    for (std::uint32_t binding = 0U; binding < bindings.size(); ++binding) {
      bindings[binding].binding = binding;
      bindings[binding].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
      bindings[binding].descriptorCount = 1U;
      bindings[binding].stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
    }
    VkDescriptorSetLayoutCreateInfo descriptor{VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO};
    descriptor.bindingCount = static_cast<std::uint32_t>(bindings.size());
    descriptor.pBindings = bindings.data();
    if (vkCreateDescriptorSetLayout(device_, &descriptor, nullptr, &descriptor_) != VK_SUCCESS) {
      diagnostic_ = "routing compute descriptor layout failed";
      return;
    }
    VkPushConstantRange range{VK_SHADER_STAGE_COMPUTE_BIT, 0U, sizeof(Push)};
    VkPipelineLayoutCreateInfo layout{VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO};
    layout.setLayoutCount = 1U;
    layout.pSetLayouts = &descriptor_;
    layout.pushConstantRangeCount = 1U;
    layout.pPushConstantRanges = &range;
    if (vkCreatePipelineLayout(device_, &layout, nullptr, &layout_) != VK_SUCCESS) {
      diagnostic_ = "routing compute pipeline layout failed";
      return;
    }
    std::ifstream stream(FLUX_ROUTING_COMPUTE_SHADER_PATH, std::ios::binary | std::ios::ate);
    if (!stream) {
      diagnostic_ = "routing compute shader is missing";
      return;
    }
    const auto size = stream.tellg();
    if (size <= 0 || static_cast<std::size_t>(size) % sizeof(std::uint32_t) != 0U) {
      diagnostic_ = "routing compute shader is invalid";
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
      diagnostic_ = "routing compute shader module failed";
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
    if (status != VK_SUCCESS)
      diagnostic_ = "routing compute pipeline failed";
  }

  [[nodiscard]] bool create_buffer(std::size_t size, Buffer& buffer) {
    buffer.size = static_cast<VkDeviceSize>(size);
    return vulkan::Runtime::instance().allocate_host_buffer(buffer.size, buffer.handle,
                                                            buffer.memory, buffer.mapped);
  }

  [[nodiscard]] bool ensure_buffer(std::size_t size, Buffer& buffer) {
    if (buffer.handle != VK_NULL_HANDLE && buffer.size >= size) {
      return true;
    }
    destroy(buffer);
    return create_buffer(size, buffer);
  }

  [[nodiscard]] bool update_descriptor_set(const std::array<Buffer, 4U>& buffers) {
    if (descriptor_pool_ == VK_NULL_HANDLE) {
      VkDescriptorPoolSize size{VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 4U};
      VkDescriptorPoolCreateInfo pool_info{VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO};
      pool_info.maxSets = 1U;
      pool_info.poolSizeCount = 1U;
      pool_info.pPoolSizes = &size;
      if (vkCreateDescriptorPool(device_, &pool_info, nullptr, &descriptor_pool_) != VK_SUCCESS) {
        return false;
      }
      VkDescriptorSetAllocateInfo allocate{VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO};
      allocate.descriptorPool = descriptor_pool_;
      allocate.descriptorSetCount = 1U;
      allocate.pSetLayouts = &descriptor_;
      if (vkAllocateDescriptorSets(device_, &allocate, &descriptor_set_) != VK_SUCCESS) {
        vkDestroyDescriptorPool(device_, descriptor_pool_, nullptr);
        descriptor_pool_ = VK_NULL_HANDLE;
        return false;
      }
    }
    std::array<VkDescriptorBufferInfo, 4U> infos{};
    std::array<VkWriteDescriptorSet, 4U> writes{};
    for (std::uint32_t index = 0U; index < buffers.size(); ++index) {
      infos[index] = VkDescriptorBufferInfo{buffers[index].handle, 0U, buffers[index].size};
      writes[index] = VkWriteDescriptorSet{VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
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

  [[nodiscard]] bool prepare_command_buffer() {
    if (command_ == VK_NULL_HANDLE) {
      VkCommandBufferAllocateInfo allocate{VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO};
      allocate.commandPool = command_pool_;
      allocate.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
      allocate.commandBufferCount = 1U;
      return vkAllocateCommandBuffers(device_, &allocate, &command_) == VK_SUCCESS;
    }
    return vkResetCommandBuffer(command_, 0U) == VK_SUCCESS;
  }

  void destroy(Buffer& buffer) {
    vulkan::Runtime::instance().release_buffer(buffer.handle, buffer.memory, buffer.mapped);
    buffer = Buffer{};
  }
  void destroy(std::array<Buffer, 4U>& buffers) {
    for (auto& buffer : buffers) {
      destroy(buffer);
    }
  }
  void shutdown() {
    if (device_ != VK_NULL_HANDLE)
      destroy(buffers_);
    if (pipeline_ != VK_NULL_HANDLE)
      vkDestroyPipeline(device_, pipeline_, nullptr);
    if (layout_ != VK_NULL_HANDLE)
      vkDestroyPipelineLayout(device_, layout_, nullptr);
    if (descriptor_pool_ != VK_NULL_HANDLE)
      vkDestroyDescriptorPool(device_, descriptor_pool_, nullptr);
    if (descriptor_ != VK_NULL_HANDLE)
      vkDestroyDescriptorSetLayout(device_, descriptor_, nullptr);
    if (command_pool_ != VK_NULL_HANDLE)
      vkDestroyCommandPool(device_, command_pool_, nullptr);
  }

  VkInstance instance_{VK_NULL_HANDLE};
  VkPhysicalDevice physical_{VK_NULL_HANDLE};
  VkDevice device_{VK_NULL_HANDLE};
  VkQueue queue_{VK_NULL_HANDLE};
  std::uint32_t family_{0U};
  VkCommandPool command_pool_{VK_NULL_HANDLE};
  VkCommandBuffer command_{VK_NULL_HANDLE};
  VkDescriptorSetLayout descriptor_{VK_NULL_HANDLE};
  VkDescriptorPool descriptor_pool_{VK_NULL_HANDLE};
  VkDescriptorSet descriptor_set_{VK_NULL_HANDLE};
  VkPipelineLayout layout_{VK_NULL_HANDLE};
  VkPipeline pipeline_{VK_NULL_HANDLE};
  std::array<Buffer, 4U> buffers_;
  std::string diagnostic_;
};

[[nodiscard]] Context& context() {
  static Context value;
  return value;
}
[[nodiscard]] std::mutex& context_mutex() {
  static std::mutex value;
  return value;
}

struct PackedGeometry {
  std::int64_t origin_x{0};
  std::int64_t origin_y{0};
  std::vector<std::uint32_t> points;
  std::vector<std::uint32_t> segments;
  std::string diagnostic;
};

[[nodiscard]] PackedGeometry pack_geometry(const std::vector<RoutingComputePoint>& points,
                                           const std::vector<RoutingComputeSegment>& segments) {
  PackedGeometry packed;
  if (!points.empty()) {
    packed.origin_x = points.front().x_nm;
    packed.origin_y = points.front().y_nm;
  } else if (!segments.empty()) {
    packed.origin_x = segments.front().first_x_nm;
    packed.origin_y = segments.front().first_y_nm;
  }
  const auto coordinate = [&](std::int64_t value,
                              std::int64_t origin) -> std::optional<std::uint32_t> {
    const auto relative = relative_coordinate(value, origin);
    return relative ? std::optional{static_cast<std::uint32_t>(*relative)} : std::nullopt;
  };
  for (const auto& point : points) {
    const auto x = coordinate(point.x_nm, packed.origin_x);
    const auto y = coordinate(point.y_nm, packed.origin_y);
    if (!x || !y || point.radius_nm < 0 ||
        point.radius_nm > std::numeric_limits<std::uint32_t>::max()) {
      packed.diagnostic = "routing compute point exceeds 32-bit relative geometry";
      return packed;
    }
    packed.points.insert(packed.points.end(), {*x, *y, static_cast<std::uint32_t>(point.radius_nm),
                                               point.layer, point.owner});
  }
  for (const auto& segment : segments) {
    const auto x1 = coordinate(segment.first_x_nm, packed.origin_x);
    const auto y1 = coordinate(segment.first_y_nm, packed.origin_y);
    const auto x2 = coordinate(segment.second_x_nm, packed.origin_x);
    const auto y2 = coordinate(segment.second_y_nm, packed.origin_y);
    if (!x1 || !y1 || !x2 || !y2 || segment.radius_nm < 0 ||
        segment.radius_nm > std::numeric_limits<std::uint32_t>::max()) {
      packed.diagnostic = "routing compute segment exceeds 32-bit relative geometry";
      return packed;
    }
    packed.segments.insert(packed.segments.end(),
                           {*x1, *y1, *x2, *y2, static_cast<std::uint32_t>(segment.radius_nm),
                            segment.layer, segment.owner});
  }
  return packed;
}

} // namespace

bool routing_compute_gpu_available_impl() {
  std::lock_guard lock(context_mutex());
  return context().ok();
}

RoutingComputeDeviceInfo routing_compute_device_info_impl() {
  std::lock_guard lock(context_mutex());
  const auto& runtime = vulkan::Runtime::instance();
  const auto& compute = context();
  return {compute.ok(),
          runtime.device_name(),
          runtime.device_type(),
          runtime.vendor_id(),
          runtime.device_id(),
          runtime.api_version(),
          runtime.driver_version(),
          compute.ok() ? std::string{} : compute.diagnostic()};
}

RoutingComputeProximityResult
probe_routing_proximity_gpu(const std::vector<RoutingComputePoint>& queries,
                            const std::vector<RoutingComputeSegment>& segments,
                            std::int64_t guard) {
  RoutingComputeProximityResult result;
  if (guard < 0 || guard > std::numeric_limits<std::uint32_t>::max()) {
    result.diagnostic = "routing compute proximity guard exceeds GPU bounds";
    return result;
  }
  const auto packed = pack_geometry(queries, segments);
  if (!packed.diagnostic.empty()) {
    result.diagnostic = packed.diagnostic;
    return result;
  }
  Push push;
  push.mode = 0U;
  push.count_a = static_cast<std::uint32_t>(queries.size());
  push.count_b = static_cast<std::uint32_t>(segments.size());
  push.guard_nm = static_cast<std::uint32_t>(guard);
  std::vector<std::uint32_t> raw;
  bool final_state = false;
  std::lock_guard lock(context_mutex());
  if (!context().dispatch(packed.points, packed.segments, {}, queries.size() * 4U, push,
                          queries.size(), 1U, raw, final_state, result.diagnostic))
    return result;
  result.queries.resize(queries.size());
  for (std::size_t query = 0U; query < queries.size(); ++query) {
    float margin = 0.0F;
    std::memcpy(&margin, &raw[query * 4U], sizeof(margin));
    result.queries[query].conservative_margin_nm =
        std::isfinite(margin) ? static_cast<std::int64_t>(std::floor(margin))
                              : std::numeric_limits<std::int64_t>::max();
    result.queries[query].closest_segment = raw[query * 4U + 1U];
    result.queries[query].possible_conflict = raw[query * 4U + 2U] != 0U;
    result.queries[query].tested_segments = raw[query * 4U + 3U];
  }
  result.used_gpu = true;
  return result;
}

RoutingComputeSegmentPairResult discover_routing_segment_conflicts_gpu(
    const std::vector<RoutingComputeSegment>& segments,
    const std::vector<std::pair<std::uint32_t, std::uint32_t>>& pairs, std::int64_t guard) {
  RoutingComputeSegmentPairResult result;
  if (guard < 0 || guard > std::numeric_limits<std::uint32_t>::max()) {
    result.diagnostic = "routing compute conflict guard exceeds GPU bounds";
    return result;
  }
  const auto packed = pack_geometry({}, segments);
  if (!packed.diagnostic.empty()) {
    result.diagnostic = packed.diagnostic;
    return result;
  }
  std::vector<std::uint32_t> pair_words;
  pair_words.reserve(pairs.size() * 2U);
  for (const auto [first, second] : pairs) {
    pair_words.push_back(first);
    pair_words.push_back(second);
  }
  Push push;
  push.mode = 1U;
  push.count_a = static_cast<std::uint32_t>(pairs.size());
  push.count_b = static_cast<std::uint32_t>(segments.size());
  push.guard_nm = static_cast<std::uint32_t>(guard);
  std::vector<std::uint32_t> raw;
  bool final_state = false;
  std::lock_guard lock(context_mutex());
  if (!context().dispatch(pair_words, packed.segments, {}, pairs.size(), push, pairs.size(), 1U,
                          raw, final_state, result.diagnostic))
    return result;
  result.possible_conflicts.resize(pairs.size());
  std::transform(raw.begin(), raw.end(), result.possible_conflicts.begin(),
                 [](std::uint32_t value) { return value != 0U ? 1U : 0U; });
  result.used_gpu = true;
  return result;
}

RoutingComputeChordBatchResult
classify_routing_chords_gpu(const std::vector<RoutingComputeSegment>& chords,
                            const std::vector<RoutingComputeSegment>& boundary_segments,
                            std::int64_t required, std::int64_t guard) {
  // Bound both upload buffers and shader loop work. The shader evaluates one chord against all
  // resident boundaries, so the chord count must also be constrained by the work budget.
  constexpr std::size_t maximum_chords_per_batch = 1'024U;
  constexpr std::size_t maximum_boundaries_per_batch = 1'024U;
  constexpr std::size_t maximum_pair_checks_per_batch = 262'144U;
  RoutingComputeChordBatchResult result;
  if (required < 0 || guard < 0 || required > std::numeric_limits<std::uint32_t>::max() ||
      guard > std::numeric_limits<std::uint32_t>::max()) {
    result.diagnostic = "routing chord classification bounds exceed GPU range";
    return result;
  }
  result.chords.resize(chords.size());
  std::vector<float> best_clearance(chords.size(), std::numeric_limits<float>::infinity());
  std::vector<bool> all_chunks_clear(chords.size(), true);
  std::vector<bool> any_intersection(chords.size(), false);
  for (std::size_t boundary_begin = 0U; boundary_begin < boundary_segments.size();) {
    const auto boundary_count = std::min(maximum_boundaries_per_batch,
                                         boundary_segments.size() - boundary_begin);
    const auto chord_count = std::min(
        {maximum_chords_per_batch, maximum_pair_checks_per_batch / boundary_count, chords.size()});
    for (std::size_t chord_begin = 0U; chord_begin < chords.size(); chord_begin += chord_count) {
      const auto this_chord_count = std::min(chord_count, chords.size() - chord_begin);
      std::vector<RoutingComputeSegment> chunk;
      chunk.reserve(this_chord_count + boundary_count);
      chunk.insert(chunk.end(), chords.begin() + static_cast<std::ptrdiff_t>(chord_begin),
                   chords.begin() + static_cast<std::ptrdiff_t>(chord_begin + this_chord_count));
      chunk.insert(chunk.end(),
                   boundary_segments.begin() + static_cast<std::ptrdiff_t>(boundary_begin),
                   boundary_segments.begin() + static_cast<std::ptrdiff_t>(boundary_begin + boundary_count));
      const auto packed = pack_geometry({}, chunk);
      if (!packed.diagnostic.empty()) {
        result.diagnostic = packed.diagnostic;
        return result;
      }
      const auto chord_words = this_chord_count * 7U;
      const auto chord_end = packed.segments.begin() + static_cast<std::ptrdiff_t>(chord_words);
      std::vector<std::uint32_t> packed_chords(packed.segments.begin(), chord_end);
      std::vector<std::uint32_t> packed_boundaries(chord_end, packed.segments.end());
      Push push;
      push.mode = 6U;
      push.count_a = static_cast<std::uint32_t>(this_chord_count);
      push.count_b = static_cast<std::uint32_t>(boundary_count);
      std::vector<std::uint32_t> raw;
      bool final_state = false;
      bool dispatched = false;
      {
        std::lock_guard lock(context_mutex());
        dispatched = context().dispatch(packed_chords, packed_boundaries, {}, this_chord_count * 3U,
                                        push, this_chord_count, 1U, raw, final_state,
                                        result.diagnostic);
      }
      if (!dispatched)
        return result;
      ++result.gpu_batch_count;
      result.gpu_work_items += this_chord_count * boundary_count;
      for (std::size_t local_chord = 0U; local_chord < this_chord_count; ++local_chord) {
        float clearance = 0.0F;
        std::memcpy(&clearance, &raw[local_chord * 3U], sizeof(clearance));
        const auto chord = chord_begin + local_chord;
        if (clearance < best_clearance[chord]) {
          best_clearance[chord] = clearance;
          result.chords[chord].closest_boundary =
              boundary_begin + raw[local_chord * 3U + 1U];
        }
        any_intersection[chord] = any_intersection[chord] || raw[local_chord * 3U + 2U] != 0U;
        all_chunks_clear[chord] = all_chunks_clear[chord] && std::isfinite(clearance) &&
                                  clearance >= static_cast<float>(required + guard);
      }
      const auto more_chord_batches = chord_begin + this_chord_count < chords.size();
      const auto more_boundary_batches =
          boundary_begin + boundary_count < boundary_segments.size();
      if (more_chord_batches || more_boundary_batches)
        routing_kernel_checkpoint();
    }
    boundary_begin += boundary_count;
  }
  for (std::size_t chord = 0U; chord < chords.size(); ++chord) {
    auto& assessment = result.chords[chord];
    assessment.conservative_clearance_nm =
        std::isfinite(best_clearance[chord])
            ? static_cast<std::int64_t>(std::floor(best_clearance[chord] - static_cast<float>(guard)))
            : 0;
    // A chunk can certify only its own resident boundaries. A chord is clear only when every
    // chunk certifies it; any uncertain/intersecting chunk remains an exact-CPU survivor.
    assessment.classification = !any_intersection[chord] && all_chunks_clear[chord]
                                ? RoutingComputeChordClass::definitely_clear
                                : RoutingComputeChordClass::ambiguous;
  }
  result.used_gpu = true;
  return result;
}

RoutingComputeBroadphaseResult
discover_routing_broadphase_pairs_gpu(const std::vector<RoutingComputePrimitiveBounds>& primitives,
                                      std::size_t maximum_pairs) {
  RoutingComputeBroadphaseResult result;
  if (primitives.empty() || primitives.size() > std::numeric_limits<std::uint32_t>::max() ||
      maximum_pairs == 0U || maximum_pairs > std::numeric_limits<std::uint32_t>::max()) {
    result.diagnostic = "routing broadphase exceeds GPU bounds";
    return result;
  }
  const auto origin_x = primitives.front().minimum_x_nm;
  const auto origin_y = primitives.front().minimum_y_nm;
  std::vector<std::uint32_t> packed;
  packed.reserve(primitives.size() * 6U);
  for (const auto& primitive : primitives) {
    const auto x0 = relative_coordinate(primitive.minimum_x_nm, origin_x);
    const auto y0 = relative_coordinate(primitive.minimum_y_nm, origin_y);
    const auto x1 = relative_coordinate(primitive.maximum_x_nm, origin_x);
    const auto y1 = relative_coordinate(primitive.maximum_y_nm, origin_y);
    if (!x0 || !y0 || !x1 || !y1) {
      result.diagnostic = "routing broadphase geometry exceeds relative GPU range";
      return result;
    }
    packed.insert(packed.end(), {static_cast<std::uint32_t>(*x0), static_cast<std::uint32_t>(*y0),
                                 static_cast<std::uint32_t>(*x1), static_cast<std::uint32_t>(*y1),
                                 primitive.layer, primitive.owner});
  }
  Push push;
  push.mode = 8U;
  push.count_a = static_cast<std::uint32_t>(primitives.size());
  push.count_b = static_cast<std::uint32_t>(maximum_pairs);
  std::vector<std::uint32_t> zero(1U + maximum_pairs * 2U, 0U);
  std::vector<std::uint32_t> raw;
  bool final_state = false;
  std::lock_guard lock(context_mutex());
  if (!context().dispatch(packed, {}, zero, zero.size(), push,
                          primitives.size() * primitives.size(), 1U, raw, final_state,
                          result.diagnostic))
    return result;
  const auto reported = raw.empty() ? 0U : raw[0];
  if (reported > maximum_pairs) {
    result.budget_exhausted = true;
    return result;
  }
  result.candidate_pairs.reserve(reported);
  for (std::size_t pair = 0U; pair < reported; ++pair)
    result.candidate_pairs.emplace_back(raw[1U + pair * 2U], raw[2U + pair * 2U]);
  std::sort(result.candidate_pairs.begin(), result.candidate_pairs.end());
  result.used_gpu = true;
  return result;
}

RoutingComputeViaCandidateResult
filter_routing_via_candidates_gpu(const RoutingComputeViaCandidateInput& input) {
  RoutingComputeViaCandidateResult result;
  if (input.candidates.empty() || input.top_k == 0U || input.clearance_nm < 0 ||
      input.numerical_guard_nm < 0 ||
      input.clearance_nm > std::numeric_limits<std::uint32_t>::max() - input.numerical_guard_nm) {
    result.diagnostic = "regional via filter exceeds GPU bounds";
    return result;
  }
  const auto origin_x = input.candidates.front().x_nm;
  const auto origin_y = input.candidates.front().y_nm;
  const auto coordinate = [](std::int64_t value, std::int64_t origin) {
    return relative_coordinate(value, origin);
  };
  std::vector<std::uint32_t> candidates;
  candidates.reserve(input.candidates.size() * 15U);
  for (const auto& candidate : input.candidates) {
    const auto x = coordinate(candidate.x_nm, origin_x);
    const auto y = coordinate(candidate.y_nm, origin_y);
    if (!x || !y || candidate.radius_nm < 0 || candidate.runup_radius_nm < 0 ||
        candidate.radius_nm > std::numeric_limits<std::uint32_t>::max() ||
        candidate.runup_radius_nm > std::numeric_limits<std::uint32_t>::max() ||
        candidate.runup_count > candidate.runup_origins_nm.size()) {
      result.diagnostic = "regional via candidate exceeds GPU geometry bounds";
      return result;
    }
    candidates.insert(candidates.end(),
                      {static_cast<std::uint32_t>(*x), static_cast<std::uint32_t>(*y),
                       static_cast<std::uint32_t>(candidate.radius_nm), candidate.owner,
                       static_cast<std::uint32_t>(candidate.layer_mask),
                       static_cast<std::uint32_t>(candidate.layer_mask >> 32U),
                       static_cast<std::uint32_t>(candidate.runup_radius_nm),
                       static_cast<std::uint32_t>(candidate.runup_count),
                       candidate.corridor_occupancy_pressure});
    for (std::size_t runup = 0U; runup < candidate.runup_origins_nm.size(); ++runup) {
      const auto runup_x = coordinate(candidate.runup_origins_nm[runup].first, origin_x);
      const auto runup_y = coordinate(candidate.runup_origins_nm[runup].second, origin_y);
      if (!runup_x || !runup_y) {
        result.diagnostic = "regional via run-up exceeds GPU geometry bounds";
        return result;
      }
      candidates.insert(candidates.end(),
                        {static_cast<std::uint32_t>(*runup_x), static_cast<std::uint32_t>(*runup_y),
                         candidate.runup_layers[runup]});
    }
  }
  std::vector<std::uint32_t> obstacles;
  obstacles.reserve(input.obstacles.size() * 7U);
  for (const auto& obstacle : input.obstacles) {
    const auto x1 = coordinate(obstacle.first_x_nm, origin_x);
    const auto y1 = coordinate(obstacle.first_y_nm, origin_y);
    const auto x2 = coordinate(obstacle.second_x_nm, origin_x);
    const auto y2 = coordinate(obstacle.second_y_nm, origin_y);
    if (!x1 || !y1 || !x2 || !y2 || obstacle.radius_nm < 0 || obstacle.layer >= 64U ||
        obstacle.radius_nm > std::numeric_limits<std::uint32_t>::max()) {
      result.diagnostic = "regional via obstacle exceeds GPU geometry bounds";
      return result;
    }
    obstacles.insert(obstacles.end(),
                     {static_cast<std::uint32_t>(*x1), static_cast<std::uint32_t>(*y1),
                      static_cast<std::uint32_t>(*x2), static_cast<std::uint32_t>(*y2),
                      static_cast<std::uint32_t>(obstacle.radius_nm), obstacle.layer,
                      obstacle.owner});
  }
  Push push;
  push.mode = 9U;
  push.count_a = static_cast<std::uint32_t>(input.candidates.size());
  push.count_b = static_cast<std::uint32_t>(input.obstacles.size());
  push.guard_nm = static_cast<std::uint32_t>(input.clearance_nm + input.numerical_guard_nm);
  std::vector<std::uint32_t> raw;
  bool final_state = false;
  std::lock_guard lock(context_mutex());
  if (!context().dispatch(candidates, obstacles, {}, input.candidates.size() * 6U, push,
                          input.candidates.size(), 1U, raw, final_state, result.diagnostic))
    return result;
  std::vector<RoutingComputeViaCandidateAssessment> ranking;
  ranking.reserve(input.candidates.size());
  result.rejection_bitmap.resize(input.candidates.size());
  for (std::size_t index = 0U; index < input.candidates.size(); ++index) {
    float margin = 0.0F, runup_length = 0.0F, score = 0.0F;
    std::memcpy(&margin, &raw[index * 6U + 2U], sizeof(margin));
    std::memcpy(&runup_length, &raw[index * 6U + 3U], sizeof(runup_length));
    std::memcpy(&score, &raw[index * 6U + 5U], sizeof(score));
    const auto rejection = raw[index * 6U];
    result.rejection_bitmap[index] = rejection;
    ranking.push_back(RoutingComputeViaCandidateAssessment{
        index, rejection, raw[index * 6U + 1U],
        std::isfinite(margin) ? static_cast<std::int64_t>(std::floor(margin))
                              : std::numeric_limits<std::int64_t>::max(),
        std::isfinite(runup_length)
            ? static_cast<std::uint64_t>(std::max(0.0F, std::round(runup_length)))
            : std::numeric_limits<std::uint64_t>::max(),
        raw[index * 6U + 4U],
        std::isfinite(score) ? static_cast<long double>(score)
                             : std::numeric_limits<long double>::infinity()});
  }
  std::stable_sort(ranking.begin(), ranking.end(), [](const auto& lhs, const auto& rhs) {
    const auto lhs_rejected = lhs.rejection_bits != 0U;
    const auto rhs_rejected = rhs.rejection_bits != 0U;
    if (lhs_rejected != rhs_rejected)
      return !lhs_rejected;
    if (lhs.score != rhs.score)
      return lhs.score < rhs.score;
    if (lhs.conservative_margin_nm != rhs.conservative_margin_nm)
      return lhs.conservative_margin_nm > rhs.conservative_margin_nm;
    return lhs.candidate < rhs.candidate;
  });
  result.assessed_candidates = ranking.size();
  ranking.resize(std::min(input.top_k, ranking.size()));
  result.shortlist = std::move(ranking);
  result.used_gpu = true;
  return result;
}

RoutingComputeSplineBatchResult
evaluate_routing_spline_batch_gpu(const RoutingComputeSplineBatchInput& input) {
  RoutingComputeSplineBatchResult result;
  if (input.candidate_count == 0U || input.control_points_nm.size() % input.candidate_count != 0U ||
      input.control_points_nm.size() > std::numeric_limits<std::uint32_t>::max() ||
      input.sample_count > std::numeric_limits<std::uint32_t>::max() ||
      input.candidate_count > std::numeric_limits<std::uint32_t>::max() ||
      input.sample_count > std::numeric_limits<std::size_t>::max() / input.candidate_count) {
    result.diagnostic = "routing compute spline batch exceeds GPU bounds";
    return result;
  }
  const auto origin_x = input.control_points_nm.front().first;
  const auto origin_y = input.control_points_nm.front().second;
  std::vector<std::uint32_t> controls;
  controls.reserve(input.control_points_nm.size() * 2U);
  for (const auto [x, y] : input.control_points_nm) {
    const auto relative_x = relative_coordinate(x, origin_x);
    const auto relative_y = relative_coordinate(y, origin_y);
    if (!relative_x || !relative_y) {
      result.diagnostic = "routing compute spline controls exceed relative GPU range";
      return result;
    }
    const auto packed_x = static_cast<float>(*relative_x);
    const auto packed_y = static_cast<float>(*relative_y);
    std::uint32_t x_bits = 0U;
    std::uint32_t y_bits = 0U;
    std::memcpy(&x_bits, &packed_x, sizeof(x_bits));
    std::memcpy(&y_bits, &packed_y, sizeof(y_bits));
    controls.push_back(x_bits);
    controls.push_back(y_bits);
  }
  std::vector<std::uint32_t> coefficients;
  coefficients.reserve(input.basis_coefficients.size());
  for (const auto coefficient : input.basis_coefficients) {
    const auto packed = static_cast<float>(coefficient);
    if (!std::isfinite(packed)) {
      result.diagnostic = "routing compute spline coefficient is not finite";
      return result;
    }
    std::uint32_t bits = 0U;
    std::memcpy(&bits, &packed, sizeof(bits));
    coefficients.push_back(bits);
  }
  Push push;
  push.mode = 4U;
  const auto total_samples = input.sample_count * input.candidate_count;
  push.count_a = static_cast<std::uint32_t>(total_samples);
  push.count_b = static_cast<std::uint32_t>(input.control_points_nm.size() / input.candidate_count);
  push.width = static_cast<std::uint32_t>(input.sample_count);
  std::vector<std::uint32_t> raw;
  bool final_state = false;
  std::lock_guard lock(context_mutex());
  if (!context().dispatch(controls, coefficients, {}, total_samples * 2U, push, total_samples, 1U,
                          raw, final_state, result.diagnostic)) {
    return result;
  }
  result.positions_nm.reserve(total_samples);
  for (std::size_t sample = 0U; sample < total_samples; ++sample) {
    float x = 0.0F;
    float y = 0.0F;
    std::memcpy(&x, &raw[sample * 2U], sizeof(x));
    std::memcpy(&y, &raw[sample * 2U + 1U], sizeof(y));
    if (!std::isfinite(x) || !std::isfinite(y)) {
      result.positions_nm.clear();
      result.diagnostic = "routing compute spline output is not finite";
      return result;
    }
    result.positions_nm.emplace_back(static_cast<long double>(origin_x) + x,
                                     static_cast<long double>(origin_y) + y);
  }
  result.used_gpu = true;
  return result;
}

RoutingComputeSplineScoreResult
score_routing_spline_candidates_gpu(const RoutingComputeSplineScoreInput& input) {
  RoutingComputeSplineScoreResult result;
  if (input.control_points_nm.empty() || input.candidate_count == 0U || input.sample_count < 3U ||
      input.control_points_nm.size() % input.candidate_count != 0U ||
      input.field_width > std::numeric_limits<std::uint32_t>::max() ||
      input.field_height > std::numeric_limits<std::uint32_t>::max() || input.field_pitch_nm <= 0 ||
      input.field_pitch_nm > std::numeric_limits<std::int32_t>::max()) {
    result.diagnostic = "routing spline score exceeds GPU bounds";
    return result;
  }
  const auto origin_x = input.control_points_nm.front().first;
  const auto origin_y = input.control_points_nm.front().second;
  std::vector<std::uint32_t> controls;
  controls.reserve(input.control_points_nm.size() * 2U);
  for (const auto [x, y] : input.control_points_nm) {
    const auto relative_x = relative_coordinate(x, origin_x);
    const auto relative_y = relative_coordinate(y, origin_y);
    if (!relative_x || !relative_y) {
      result.diagnostic = "routing spline score controls exceed relative GPU range";
      return result;
    }
    for (const auto value : {static_cast<float>(*relative_x), static_cast<float>(*relative_y)}) {
      std::uint32_t bits = 0U;
      std::memcpy(&bits, &value, sizeof(bits));
      controls.push_back(bits);
    }
  }
  std::vector<std::uint32_t> coefficients;
  coefficients.reserve(input.basis_coefficients.size());
  for (const auto coefficient : input.basis_coefficients) {
    const auto value = static_cast<float>(coefficient);
    std::uint32_t bits = 0U;
    std::memcpy(&bits, &value, sizeof(bits));
    coefficients.push_back(bits);
  }
  const auto field_origin_x = relative_coordinate(input.field_origin_x_nm, origin_x);
  const auto field_origin_y = relative_coordinate(input.field_origin_y_nm, origin_y);
  if (!field_origin_x || !field_origin_y) {
    result.diagnostic = "routing spline score field exceeds relative GPU range";
    return result;
  }
  std::vector<std::uint32_t> field;
  field.reserve(8U + input.signed_clearance_nm.size() + input.corridor_mask.size());
  field.push_back(static_cast<std::uint32_t>(input.field_width));
  field.push_back(static_cast<std::uint32_t>(input.field_height));
  const auto append_float = [&](float value) {
    std::uint32_t bits = 0U;
    std::memcpy(&bits, &value, sizeof(bits));
    field.push_back(bits);
  };
  append_float(static_cast<float>(input.field_pitch_nm));
  append_float(static_cast<float>(*field_origin_x));
  append_float(static_cast<float>(*field_origin_y));
  append_float(static_cast<float>(input.minimum_clearance_nm));
  append_float(static_cast<float>(input.minimum_bend_radius_nm));
  append_float(static_cast<float>(input.clearance_influence_nm));
  for (const auto value : input.signed_clearance_nm)
    append_float(value);
  for (const auto value : input.corridor_mask)
    field.push_back(value != 0U ? 1U : 0U);
  Push push;
  push.mode = 7U;
  push.count_a = static_cast<std::uint32_t>(input.candidate_count);
  push.count_b = static_cast<std::uint32_t>(input.control_points_nm.size() / input.candidate_count);
  push.width = static_cast<std::uint32_t>(input.sample_count);
  for (std::size_t weight = 0U; weight < input.objective_weights.size(); ++weight)
    std::memcpy(&push.weights[weight], &input.objective_weights[weight], sizeof(std::uint32_t));
  std::vector<std::uint32_t> raw;
  bool final_state = false;
  std::lock_guard lock(context_mutex());
  if (!context().dispatch(controls, coefficients, field, input.candidate_count * 4U, push,
                          input.candidate_count, 1U, raw, final_state, result.diagnostic))
    return result;
  std::vector<RoutingComputeSplineCandidateScore> ranking;
  ranking.reserve(input.candidate_count);
  for (std::size_t candidate = 0U; candidate < input.candidate_count; ++candidate) {
    float objective = 0.0F;
    std::memcpy(&objective, &raw[candidate * 4U + 1U], sizeof(objective));
    ranking.push_back(RoutingComputeSplineCandidateScore{candidate, raw[candidate * 4U],
                                                         raw[candidate * 4U + 2U], objective});
  }
  std::stable_sort(ranking.begin(), ranking.end(), [](const auto& lhs, const auto& rhs) {
    if (lhs.violations != rhs.violations)
      return lhs.violations < rhs.violations;
    if (lhs.approximate_self_intersections != rhs.approximate_self_intersections)
      return lhs.approximate_self_intersections < rhs.approximate_self_intersections;
    if (lhs.objective != rhs.objective)
      return lhs.objective < rhs.objective;
    return lhs.candidate < rhs.candidate;
  });
  result.assessed_candidates = ranking.size();
  ranking.resize(std::min(input.top_k, ranking.size()));
  result.shortlist = std::move(ranking);
  result.used_gpu = true;
  return result;
}

RoutingComputeWavefrontResult
compute_routing_wavefront_gpu(const RoutingComputeWavefrontInput& input) {
  RoutingComputeWavefrontResult result;
  if (input.width > std::numeric_limits<std::uint32_t>::max() ||
      input.height > std::numeric_limits<std::uint32_t>::max() ||
      input.maximum_passes > std::numeric_limits<std::uint32_t>::max()) {
    result.diagnostic = "routing compute wavefront exceeds GPU bounds";
    return result;
  }
  const auto count = input.width * input.height;
  std::vector<std::uint32_t> mask(count, 0U),
      initial(count, std::numeric_limits<std::uint32_t>::max());
  std::transform(input.traversable.begin(), input.traversable.end(), mask.begin(),
                 [](std::uint8_t value) { return value != 0U ? 1U : 0U; });
  for (const auto source : input.sources) {
    if (source >= count) {
      result.diagnostic = "routing compute wavefront source is out of range";
      return result;
    }
    mask[source] = 1U;
    initial[source] = 0U;
  }
  Push push;
  push.mode = 3U;
  push.width = static_cast<std::uint32_t>(input.width);
  push.height = static_cast<std::uint32_t>(input.height);
  push.read_state = 1U;
  std::vector<std::uint32_t> dummy(1U, 0U);
  bool final_state = false;
  std::lock_guard lock(context_mutex());
  if (!context().dispatch(mask, dummy, initial, count, push, count, input.maximum_passes,
                          result.distance, final_state, result.diagnostic))
    return result;
  result.passes = input.maximum_passes;
  result.used_gpu = true;
  for (std::size_t cell = 0U; cell < count; ++cell) {
    if (input.traversable[cell] != 0U &&
        result.distance[cell] == std::numeric_limits<std::uint32_t>::max()) {
      result.budget_exhausted = true;
      break;
    }
  }
  return result;
}

RoutingComputeWavefrontResult
compute_routing_graph_wavefront_gpu(const RoutingComputeGraphWavefrontInput& input) {
  RoutingComputeWavefrontResult result;
  if (input.node_count > std::numeric_limits<std::uint32_t>::max() ||
      input.edges.size() > std::numeric_limits<std::uint32_t>::max() ||
      input.maximum_passes > std::numeric_limits<std::uint32_t>::max()) {
    result.diagnostic = "routing compute graph wavefront exceeds GPU bounds";
    return result;
  }
  std::vector<std::uint32_t> graph;
  graph.reserve(input.offsets.size() + input.edges.size() * 2U);
  graph.insert(graph.end(), input.offsets.begin(), input.offsets.end());
  for (const auto& edge : input.edges) {
    graph.push_back(edge.node);
    graph.push_back(edge.cost);
  }
  std::vector<std::uint32_t> initial(input.node_count, std::numeric_limits<std::uint32_t>::max());
  for (const auto source : input.sources) {
    if (source >= input.node_count) {
      result.diagnostic = "routing compute graph wavefront source is out of range";
      return result;
    }
    initial[source] = 0U;
  }
  Push push;
  push.mode = 5U;
  push.count_a = static_cast<std::uint32_t>(input.node_count);
  push.count_b = static_cast<std::uint32_t>(input.edges.size());
  push.read_state = 1U;
  std::vector<std::uint32_t> dummy(1U, 0U);
  bool final_state = false;
  std::lock_guard lock(context_mutex());
  if (!context().dispatch(graph, dummy, initial, input.node_count, push, input.node_count,
                          input.maximum_passes, result.distance, final_state, result.diagnostic)) {
    return result;
  }
  result.passes = input.maximum_passes;
  result.used_gpu = true;
  result.budget_exhausted =
      std::any_of(result.distance.begin(), result.distance.end(), [](std::uint32_t value) {
        return value == std::numeric_limits<std::uint32_t>::max();
      });
  return result;
}

RoutingComputeLinearScores
score_routing_compute_candidates_gpu(const std::vector<RoutingComputeLinearCandidate>& candidates,
                                     const std::array<std::uint32_t, 8U>& weights) {
  RoutingComputeLinearScores result;
  std::vector<std::uint32_t> input;
  input.reserve(candidates.size() * 8U);
  for (const auto& candidate : candidates)
    input.insert(input.end(), candidate.terms.begin(), candidate.terms.end());
  Push push;
  push.mode = 2U;
  push.count_a = static_cast<std::uint32_t>(candidates.size());
  push.weights = weights;
  std::vector<std::uint32_t> raw;
  bool final_state = false;
  std::lock_guard lock(context_mutex());
  if (!context().dispatch(input, {}, {}, candidates.size() * 2U, push, candidates.size(), 1U, raw,
                          final_state, result.diagnostic))
    return result;
  result.scores.resize(candidates.size());
  for (std::size_t index = 0U; index < candidates.size(); ++index)
    result.scores[index] = static_cast<std::uint64_t>(raw[index * 2U]) |
                           (static_cast<std::uint64_t>(raw[index * 2U + 1U]) << 32U);
  result.used_gpu = true;
  return result;
}

RoutingComputeQuantizedCandidateResult score_quantized_routing_candidates_gpu(
    const RoutingComputeQuantizedCandidateInput& input) {
  RoutingComputeQuantizedCandidateResult result;
  if (input.quantization_nm > std::numeric_limits<std::uint32_t>::max() ||
      input.clearance_nm > std::numeric_limits<std::uint32_t>::max() - input.numerical_guard_nm) {
    result.diagnostic = "quantized candidate batch exceeds GPU bounds";
    return result;
  }
  const auto origin = input.candidate_points_nm.front();
  std::vector<std::uint32_t> points;
  points.reserve(input.candidate_points_nm.size() * 2U);
  for (const auto& point : input.candidate_points_nm) {
    const auto x = static_cast<float>(point.first - origin.first);
    const auto y = static_cast<float>(point.second - origin.second);
    if (!std::isfinite(x) || !std::isfinite(y) ||
        std::abs(x) > static_cast<float>(std::numeric_limits<std::int32_t>::max()) ||
        std::abs(y) > static_cast<float>(std::numeric_limits<std::int32_t>::max())) {
      result.diagnostic = "quantized candidate coordinates exceed relative GPU range";
      return result;
    }
    std::uint32_t wx = 0U, wy = 0U;
    std::memcpy(&wx, &x, sizeof(x));
    std::memcpy(&wy, &y, sizeof(y));
    points.push_back(wx);
    points.push_back(wy);
  }
  std::vector<std::uint32_t> routes;
  routes.reserve(input.routes.size() * 5U);
  for (const auto& route : input.routes) {
    if (route.radius_nm < 0 || route.radius_nm > std::numeric_limits<std::uint32_t>::max() ||
        static_cast<std::size_t>(route.point_begin) + route.point_count >
            input.points_per_candidate) {
      result.diagnostic = "quantized route layout exceeds GPU bounds";
      return result;
    }
    routes.insert(routes.end(), {route.point_begin, route.point_count, route.layer, route.owner,
                                 static_cast<std::uint32_t>(route.radius_nm)});
  }
  Push push;
  push.mode = 10U;
  push.count_a = static_cast<std::uint32_t>(input.candidate_count);
  push.count_b = static_cast<std::uint32_t>(input.routes.size());
  push.width = static_cast<std::uint32_t>(input.points_per_candidate);
  push.guard_nm = static_cast<std::uint32_t>(input.clearance_nm + input.numerical_guard_nm);
  push.weights[0] = static_cast<std::uint32_t>(input.quantization_nm);
  std::vector<std::uint32_t> raw;
  bool final_state = false;
  std::lock_guard lock(context_mutex());
  if (!context().dispatch(points, routes, {}, input.candidate_count * 6U, push,
                          input.candidate_count, 1U, raw, final_state, result.diagnostic))
    return result;
  result.shortlist.reserve(input.candidate_count);
  for (std::size_t candidate = 0U; candidate < input.candidate_count; ++candidate) {
    result.shortlist.push_back({candidate, raw[candidate * 6U], raw[candidate * 6U + 1U],
                                static_cast<std::uint64_t>(raw[candidate * 6U + 2U]) |
                                    (static_cast<std::uint64_t>(raw[candidate * 6U + 3U]) << 32U),
                                static_cast<std::uint64_t>(raw[candidate * 6U + 4U]) |
                                    (static_cast<std::uint64_t>(raw[candidate * 6U + 5U]) << 32U)});
  }
  std::stable_sort(result.shortlist.begin(), result.shortlist.end(), [](const auto& a, const auto& b) {
    if (a.clearance_violations != b.clearance_violations)
      return a.clearance_violations < b.clearance_violations;
    if (a.crossings != b.crossings) return a.crossings < b.crossings;
    if (a.total_deficit_nm != b.total_deficit_nm) return a.total_deficit_nm < b.total_deficit_nm;
    return a.length_nm != b.length_nm ? a.length_nm < b.length_nm : a.candidate < b.candidate;
  });
  result.assessed_candidates = input.candidate_count;
  result.shortlist.resize(std::min(input.top_k, result.shortlist.size()));
  result.used_gpu = true;
  return result;
}

RoutingComputeAssignmentResult evaluate_routing_assignments_gpu(
    const RoutingComputeAssignmentInput& input) {
  RoutingComputeAssignmentResult result;
  std::vector<std::uint32_t> domains(input.variable_domains.size() + 1U, 0U);
  std::size_t state_count = 1U;
  for (std::size_t variable = 0U; variable < input.variable_domains.size(); ++variable) {
    if (input.variable_domains[variable].empty())
      return result;
    if (input.variable_domains[variable].size() >
        std::numeric_limits<std::uint32_t>::max() - domains[variable]) {
      result.diagnostic = "regional assignment domains exceed GPU bounds";
      return result;
    }
    domains[variable + 1U] =
        domains[variable] + static_cast<std::uint32_t>(input.variable_domains[variable].size());
    if (state_count > input.maximum_states / input.variable_domains[variable].size()) {
      state_count = input.maximum_states;
      result.state_space_truncated = true;
    } else if (!result.state_space_truncated) {
      state_count *= input.variable_domains[variable].size();
    }
  }
  for (const auto& domain : input.variable_domains)
    domains.insert(domains.end(), domain.begin(), domain.end());
  std::vector<std::uint32_t> compatibility;
  compatibility.reserve(input.incompatible_options.size() * 2U + input.option_costs.size());
  for (const auto [first, second] : input.incompatible_options) {
    compatibility.push_back(first);
    compatibility.push_back(second);
  }
  compatibility.insert(compatibility.end(), input.option_costs.begin(), input.option_costs.end());
  Push push;
  push.mode = 12U;
  push.count_a = static_cast<std::uint32_t>(state_count);
  push.count_b = static_cast<std::uint32_t>(input.incompatible_options.size());
  push.width = static_cast<std::uint32_t>(input.variable_domains.size());
  push.height = static_cast<std::uint32_t>(input.option_costs.size());
  std::vector<std::uint32_t> raw;
  bool final_state = false;
  std::lock_guard lock(context_mutex());
  if (!context().dispatch(domains, compatibility, {}, state_count * 2U, push, state_count, 1U, raw,
                          final_state, result.diagnostic))
    return result;
  for (std::size_t dense = 0U; dense < state_count; ++dense) {
    if (raw[dense * 2U] == 0U)
      continue;
    auto code = dense;
    RoutingComputeAssignmentCandidate candidate;
    candidate.dense_state = dense;
    candidate.score = raw[dense * 2U + 1U];
    for (const auto& domain : input.variable_domains) {
      candidate.options.push_back(domain[code % domain.size()]);
      code /= domain.size();
    }
    result.shortlist.push_back(std::move(candidate));
  }
  result.assessed_states = state_count;
  std::stable_sort(result.shortlist.begin(), result.shortlist.end(), [](const auto& a, const auto& b) {
    return a.score != b.score ? a.score < b.score : a.dense_state < b.dense_state;
  });
  result.shortlist.resize(std::min(input.top_k, result.shortlist.size()));
  result.used_gpu = true;
  return result;
}

RoutingComputeLaneOrderResult solve_routing_lane_order_gpu(const RoutingComputeLaneOrderInput& input) {
  RoutingComputeLaneOrderResult result;
  const auto count = input.widths_nm.size();
  const auto state_count = std::size_t{1U} << count;
  const auto slots = state_count * count;
  std::vector<std::uint32_t> occupants;
  occupants.reserve(count * 4U);
  for (std::size_t index = 0U; index < count; ++index)
    occupants.insert(occupants.end(), {input.widths_nm[index], input.clearances_nm[index],
                                      input.reference_position[index], input.coupled_rank[index]});
  std::vector<std::uint32_t> initial(slots * 2U, std::numeric_limits<std::uint32_t>::max());
  Push push;
  push.mode = 11U;
  push.count_a = static_cast<std::uint32_t>(count);
  push.read_state = 1U;
  std::vector<std::uint32_t> raw;
  bool final_state = false;
  std::lock_guard lock(context_mutex());
  if (!context().dispatch(occupants, {}, initial, initial.size(), push, slots, count, raw,
                          final_state, result.diagnostic))
    return result;
  result.assessed_states = slots;
  const auto full = state_count - 1U;
  std::size_t best = count;
  for (std::size_t last = 0U; last < count; ++last) {
    const auto slot = (full * count + last) * 2U;
    if (raw[slot] == std::numeric_limits<std::uint32_t>::max())
      continue;
    const auto packed = static_cast<std::uint64_t>(raw[slot]) + input.clearances_nm[last];
    if (packed <= input.usable_width_nm &&
        (best == count || raw[slot + 1U] < result.inversions ||
         (raw[slot + 1U] == result.inversions && packed < result.packed_width_nm))) {
      best = last;
      result.inversions = raw[slot + 1U];
      result.packed_width_nm = packed;
    }
  }
  if (best == count) {
    result.used_gpu = true;
    return result;
  }
  auto mask = full;
  auto last = best;
  while (mask != 0U) {
    result.order.push_back(static_cast<std::uint32_t>(last));
    const auto prior_mask = mask & ~(std::size_t{1U} << last);
    if (prior_mask == 0U)
      break;
    const auto slot = (mask * count + last) * 2U;
    std::size_t selected = count;
    for (std::size_t prior = 0U; prior < count; ++prior) {
      if ((prior_mask & (std::size_t{1U} << prior)) == 0U)
        continue;
      const auto prior_slot = (prior_mask * count + prior) * 2U;
      if (raw[prior_slot] == std::numeric_limits<std::uint32_t>::max())
        continue;
      auto added = 0U;
      for (std::size_t placed = 0U; placed < count; ++placed)
        if ((prior_mask & (std::size_t{1U} << placed)) != 0U &&
            input.reference_position[placed] > input.reference_position[last])
          ++added;
      if (raw[prior_slot] + std::max(input.clearances_nm[prior], input.clearances_nm[last]) +
                  input.widths_nm[last] ==
              raw[slot] &&
          raw[prior_slot + 1U] + added == raw[slot + 1U]) {
        selected = prior;
        break;
      }
    }
    if (selected == count) {
      result.diagnostic = "GPU lane-order predecessor reconstruction failed";
      return result;
    }
    mask = prior_mask;
    last = selected;
  }
  std::reverse(result.order.begin(), result.order.end());
  result.physically_fits = true;
  result.used_gpu = true;
  return result;
}

RoutingComputeOrientationResult evaluate_routing_orientations_gpu(
    const RoutingComputeOrientationInput& input) {
  RoutingComputeOrientationResult result;
  const auto state_count = std::size_t{1U} << input.bit_count;
  std::vector<std::uint32_t> clauses;
  for (const auto [mask, value] : input.forbidden_clauses) {
    clauses.push_back(mask);
    clauses.push_back(value);
  }
  Push push;
  push.mode = 13U;
  push.count_a = static_cast<std::uint32_t>(state_count);
  push.count_b = static_cast<std::uint32_t>(input.forbidden_clauses.size());
  push.width = static_cast<std::uint32_t>(input.bit_count);
  push.height = static_cast<std::uint32_t>(input.bit_flip_costs.size());
  std::vector<std::uint32_t> raw;
  bool final_state = false;
  std::lock_guard lock(context_mutex());
  if (!context().dispatch(clauses, input.bit_flip_costs, {}, state_count * 2U, push, state_count, 1U,
                          raw, final_state, result.diagnostic))
    return result;
  std::vector<std::pair<std::uint32_t, std::uint32_t>> ranking;
  for (std::uint32_t state = 0U; state < state_count; ++state)
    if (raw[state * 2U] != 0U)
      ranking.emplace_back(raw[state * 2U + 1U], state);
  std::stable_sort(ranking.begin(), ranking.end());
  ranking.resize(std::min(input.top_k, ranking.size()));
  for (const auto& [cost, state] : ranking) {
    static_cast<void>(cost);
    result.states.push_back(state);
  }
  result.assessed_states = state_count;
  result.used_gpu = true;
  return result;
}

RoutingComputeCopperNarrowphaseResult assess_simple_copper_pairs_gpu(
    const std::vector<RoutingComputeCopperPrimitive>& primitives,
    const std::vector<std::pair<std::uint32_t, std::uint32_t>>& pairs, std::int64_t clearance,
    std::int64_t guard) {
  RoutingComputeCopperNarrowphaseResult result;
  if (primitives.empty())
    return result;
  const auto origin_x = primitives.front().first_x_nm;
  const auto origin_y = primitives.front().first_y_nm;
  std::vector<std::uint32_t> packed;
  packed.reserve(primitives.size() * 8U);
  for (const auto& primitive : primitives) {
    const auto x1 = relative_coordinate(primitive.first_x_nm, origin_x);
    const auto y1 = relative_coordinate(primitive.first_y_nm, origin_y);
    const auto x2 = relative_coordinate(primitive.second_x_nm, origin_x);
    const auto y2 = relative_coordinate(primitive.second_y_nm, origin_y);
    if (!x1 || !y1 || !x2 || !y2 || primitive.radius_nm < 0 ||
        primitive.radius_nm > std::numeric_limits<std::uint32_t>::max() ||
        clearance > std::numeric_limits<std::uint32_t>::max() - guard) {
      result.diagnostic = "simple-copper geometry exceeds relative GPU range";
      return result;
    }
    packed.insert(packed.end(), {static_cast<std::uint32_t>(primitive.kind),
                                 static_cast<std::uint32_t>(*x1), static_cast<std::uint32_t>(*y1),
                                 static_cast<std::uint32_t>(*x2), static_cast<std::uint32_t>(*y2),
                                 static_cast<std::uint32_t>(primitive.radius_nm), primitive.layer,
                                 primitive.owner});
  }
  std::vector<std::uint32_t> pair_words;
  for (const auto [first, second] : pairs) {
    pair_words.push_back(first);
    pair_words.push_back(second);
  }
  Push push;
  push.mode = 15U;
  push.count_a = static_cast<std::uint32_t>(pairs.size());
  push.count_b = static_cast<std::uint32_t>(primitives.size());
  push.guard_nm = static_cast<std::uint32_t>(clearance + guard);
  std::vector<std::uint32_t> raw;
  bool final_state = false;
  std::lock_guard lock(context_mutex());
  if (!context().dispatch(pair_words, packed, {}, pairs.size() * 4U, push, pairs.size(), 1U, raw,
                          final_state, result.diagnostic))
    return result;
  result.pairs.resize(pairs.size());
  for (std::size_t index = 0U; index < pairs.size(); ++index) {
    float deficit = 0.0F, nx = 0.0F, ny = 0.0F;
    std::memcpy(&deficit, &raw[index * 4U + 1U], sizeof(deficit));
    std::memcpy(&nx, &raw[index * 4U + 2U], sizeof(nx));
    std::memcpy(&ny, &raw[index * 4U + 3U], sizeof(ny));
    result.pairs[index] = {static_cast<std::uint8_t>(raw[index * 4U] != 0U),
                           static_cast<std::int64_t>(std::ceil(std::max(0.0F, deficit))), nx, ny};
  }
  result.used_gpu = true;
  return result;
}

RoutingComputeBatchedGraphWavefrontResult compute_batched_routing_graph_wavefront_gpu(
    const RoutingComputeBatchedGraphWavefrontInput& input) {
  RoutingComputeBatchedGraphWavefrontResult result;
  const auto jobs = input.source_groups.size();
  if (input.node_count > std::numeric_limits<std::uint32_t>::max() ||
      jobs > std::numeric_limits<std::uint32_t>::max()) {
    result.diagnostic = "batched graph wavefront exceeds GPU bounds";
    return result;
  }
  std::vector<std::uint32_t> graph(input.offsets.begin(), input.offsets.end());
  for (const auto& edge : input.edges) {
    graph.push_back(edge.node);
    graph.push_back(edge.cost);
  }
  std::vector<std::uint32_t> initial(input.node_count * jobs,
                                     std::numeric_limits<std::uint32_t>::max());
  for (std::size_t job = 0U; job < jobs; ++job)
    for (const auto source : input.source_groups[job]) {
      if (source >= input.node_count) {
        result.diagnostic = "batched graph source is out of range";
        return result;
      }
      initial[job * input.node_count + source] = 0U;
    }
  Push push;
  push.mode = 14U;
  push.count_a = static_cast<std::uint32_t>(input.node_count);
  push.count_b = static_cast<std::uint32_t>(input.edges.size());
  push.width = static_cast<std::uint32_t>(jobs);
  push.read_state = 1U;
  std::vector<std::uint32_t> raw;
  bool final_state = false;
  std::lock_guard lock(context_mutex());
  if (!context().dispatch(graph, {}, initial, initial.size(), push, initial.size(),
                          input.maximum_passes, raw, final_state, result.diagnostic))
    return result;
  result.distance.resize(jobs);
  for (std::size_t job = 0U; job < jobs; ++job) {
    result.distance[job].assign(raw.begin() + static_cast<std::ptrdiff_t>(job * input.node_count),
                                raw.begin() + static_cast<std::ptrdiff_t>((job + 1U) * input.node_count));
    result.budget_exhausted = result.budget_exhausted ||
                              std::find(result.distance[job].begin(), result.distance[job].end(),
                                        std::numeric_limits<std::uint32_t>::max()) !=
                                  result.distance[job].end();
  }
  result.passes = input.maximum_passes;
  result.used_gpu = true;
  return result;
}

RoutingComputeNegotiatedResourceResult reduce_negotiated_routing_resources_gpu(
    const std::vector<RoutingComputeNegotiatedResource>& resources) {
  RoutingComputeNegotiatedResourceResult result;
  std::vector<std::uint32_t> packed;
  packed.reserve(resources.size() * 4U);
  for (const auto& resource : resources)
    packed.insert(packed.end(), {resource.demand, resource.capacity, resource.history,
                                 resource.history_scale});
  Push push;
  push.mode = 17U;
  push.count_a = static_cast<std::uint32_t>(resources.size());
  std::vector<std::uint32_t> raw;
  bool final_state = false;
  std::lock_guard lock(context_mutex());
  if (!context().dispatch(packed, {}, {}, packed.size(), push, resources.size(), 1U, raw,
                          final_state, result.diagnostic))
    return result;
  result.overflow.resize(resources.size());
  result.updated_history.resize(resources.size());
  for (std::size_t index = 0U; index < resources.size(); ++index) {
    result.overflow[index] = raw[index * 4U];
    result.updated_history[index] = raw[index * 4U + 1U];
    result.total_overflow += raw[index * 4U];
    result.peak_overflow = std::max(result.peak_overflow, raw[index * 4U]);
  }
  result.used_gpu = true;
  return result;
}

RoutingComputeArcCandidateResult score_routing_arc_candidates_gpu(
    const std::vector<RoutingComputeArcCandidate>& candidates,
    const std::vector<RoutingComputeSegment>& obstacles, std::int64_t clearance,
    std::size_t top_k) {
  RoutingComputeArcCandidateResult result;
  const auto origin_x = candidates.front().points_nm[0].first;
  const auto origin_y = candidates.front().points_nm[0].second;
  std::vector<std::uint32_t> packed_candidates;
  packed_candidates.reserve(candidates.size() * 14U);
  for (const auto& candidate : candidates) {
    for (const auto& point : candidate.points_nm) {
      const auto x = relative_coordinate(point.first, origin_x);
      const auto y = relative_coordinate(point.second, origin_y);
      if (!x || !y) {
        result.diagnostic = "arc candidate exceeds relative GPU range";
        return result;
      }
      packed_candidates.push_back(static_cast<std::uint32_t>(*x));
      packed_candidates.push_back(static_cast<std::uint32_t>(*y));
    }
    if (candidate.radius_nm < 0 || candidate.copper_radius_nm < 0 ||
        candidate.radius_nm > std::numeric_limits<std::uint32_t>::max() ||
        candidate.copper_radius_nm > std::numeric_limits<std::uint32_t>::max()) {
      result.diagnostic = "arc candidate radius exceeds GPU range";
      return result;
    }
    packed_candidates.insert(packed_candidates.end(),
                             {static_cast<std::uint32_t>(candidate.radius_nm),
                              static_cast<std::uint32_t>(candidate.copper_radius_nm),
                              candidate.layer, candidate.owner});
  }
  std::vector<std::uint32_t> packed_obstacles;
  packed_obstacles.reserve(obstacles.size() * 7U);
  for (const auto& obstacle : obstacles) {
    const auto x1 = relative_coordinate(obstacle.first_x_nm, origin_x);
    const auto y1 = relative_coordinate(obstacle.first_y_nm, origin_y);
    const auto x2 = relative_coordinate(obstacle.second_x_nm, origin_x);
    const auto y2 = relative_coordinate(obstacle.second_y_nm, origin_y);
    if (!x1 || !y1 || !x2 || !y2 || obstacle.radius_nm < 0 ||
        obstacle.radius_nm > std::numeric_limits<std::uint32_t>::max() || clearance < 0 ||
        clearance > std::numeric_limits<std::uint32_t>::max()) {
      result.diagnostic = "arc obstacle exceeds relative GPU range";
      return result;
    }
    packed_obstacles.insert(packed_obstacles.end(),
                            {static_cast<std::uint32_t>(*x1), static_cast<std::uint32_t>(*y1),
                             static_cast<std::uint32_t>(*x2), static_cast<std::uint32_t>(*y2),
                             static_cast<std::uint32_t>(obstacle.radius_nm), obstacle.layer,
                             obstacle.owner});
  }
  Push push;
  push.mode = 16U;
  push.count_a = static_cast<std::uint32_t>(candidates.size());
  push.count_b = static_cast<std::uint32_t>(obstacles.size());
  push.guard_nm = static_cast<std::uint32_t>(clearance);
  std::vector<std::uint32_t> raw;
  bool final_state = false;
  std::lock_guard lock(context_mutex());
  if (!context().dispatch(packed_candidates, packed_obstacles, {}, candidates.size() * 4U, push,
                          candidates.size(), 1U, raw, final_state, result.diagnostic))
    return result;
  for (std::size_t index = 0U; index < candidates.size(); ++index) {
    float tangent = 0.0F;
    std::memcpy(&tangent, &raw[index * 4U + 1U], sizeof(tangent));
    result.shortlist.push_back({index, raw[index * 4U], tangent,
                                static_cast<std::uint64_t>(raw[index * 4U + 2U]) |
                                    (static_cast<std::uint64_t>(raw[index * 4U + 3U]) << 32U)});
  }
  std::stable_sort(result.shortlist.begin(), result.shortlist.end(), [](const auto& a, const auto& b) {
    if (a.approximate_conflicts != b.approximate_conflicts)
      return a.approximate_conflicts < b.approximate_conflicts;
    if (a.tangent_error != b.tangent_error) return a.tangent_error < b.tangent_error;
    return a.approximate_length_nm != b.approximate_length_nm
               ? a.approximate_length_nm < b.approximate_length_nm
               : a.candidate < b.candidate;
  });
  result.shortlist.resize(std::min(top_k, result.shortlist.size()));
  result.used_gpu = true;
  return result;
}

} // namespace flux
