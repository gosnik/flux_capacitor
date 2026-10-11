#include "terrain_evolution_gpu.hpp"
#include "vulkan_runtime.hpp"

#include <vulkan/vulkan.h>

#include <algorithm>
#include <array>
#include <bit>
#include <chrono>
#include <cmath>
#include <cstring>
#include <fstream>
#include <limits>
#include <memory>
#include <mutex>

namespace flux {
namespace {
constexpr std::uint32_t invalid = 0xffffffffU;
struct Buffer {
  VkBuffer handle{};
  VkDeviceMemory memory{};
  void* mapped{};
  VkDeviceSize size{};
};
struct Push {
  std::uint32_t width, height, mode, height_a, flow_a, jump_a;
  float capacity_scale, erosion_rate, deposition_rate, maximum_height_change;
};
class Context {
public:
  Context() { init(); }
  ~Context() {
    for (auto& b : buffers_)
      release(b);
    for (auto& b : upload_)
      release(b);
    for (auto& b : readback_)
      release(b);
    if (pipeline_)
      vkDestroyPipeline(device_, pipeline_, nullptr);
    if (layout_)
      vkDestroyPipelineLayout(device_, layout_, nullptr);
    if (set_layout_)
      vkDestroyDescriptorSetLayout(device_, set_layout_, nullptr);
    if (pool_)
      vkDestroyDescriptorPool(device_, pool_, nullptr);
    if (command_pool_)
      vkDestroyCommandPool(device_, command_pool_, nullptr);
  }
  bool ok() const { return diagnostic_.empty(); }
  const std::string& diagnostic() const { return diagnostic_; }
  bool run(std::size_t w, std::size_t h, const std::vector<std::uint8_t>& walk,
           const std::vector<double>& initial, const TerrainEvolutionConfig& c,
           std::stop_token stop, TerrainEvolutionResult& out) {
    // A long evolution owns the context, but a queued request must remain cancellable rather than
    // waiting for every bounded iteration under this lock.
    std::unique_lock<std::timed_mutex> lock(mutex_, std::defer_lock);
    if (stop.stop_requested()) {
      out.diagnostic = "terrain evolution cancelled";
      return false;
    }
    while (!lock.try_lock_for(std::chrono::milliseconds(1))) {
      if (stop.stop_requested()) {
        out.diagnostic = "terrain evolution cancelled";
        return false;
      }
    }
    if (stop.stop_requested()) {
      out.diagnostic = "terrain evolution cancelled";
      return false;
    }
    const auto n = w * h;
    if (!ok()) {
      out.diagnostic = diagnostic_;
      return false;
    }
    if (w > std::numeric_limits<std::uint32_t>::max() ||
        h > std::numeric_limits<std::uint32_t>::max() ||
        n > std::numeric_limits<std::uint32_t>::max()) {
      out.diagnostic = "terrain GPU request exceeds 32-bit shader bounds";
      return false;
    }
    // Zero iterations are a valid no-op. Do not read an uninitialized GPU field or claim a
    // dispatch.
    if (c.iterations == 0U) {
      out.height = initial;
      out.flow.assign(n, 0.0);
      out.divergence.assign(n, 0.0);
      out.used_gpu = false;
      out.gpu_device.clear();
      out.gpu_dispatches = 0U;
      out.gpu_buffer_reuses = 0U;
      out.gpu_buffer_growths = 0U;
      return true;
    }
    std::shared_ptr<const vulkan::ImmutableMaskLease> mask_lease;
    bool mask_cache_hit = false;
    bool cross_pipeline_reuse = false;
    std::string mask_error;
    if (!vulkan::Runtime::instance().acquire_immutable_mask(
            w, h, walk, vulkan::MaskConsumer::terrain, mask_lease, mask_cache_hit,
            cross_pipeline_reuse, mask_error)) {
      out.diagnostic = "terrain GPU immutable mask preparation failed: " + mask_error;
      return false;
    }
    if (!ensure(n, mask_lease)) {
      out.diagnostic = "terrain GPU persistent resources could not be prepared";
      return false;
    }
    std::vector<float> heights(n);
    for (std::size_t i = 0; i < n; ++i) {
      heights[i] = static_cast<float>(initial[i]);
    }
    std::memcpy(upload_[1].mapped, heights.data(), n * 4);
    bool ha = true, fa = true, ja = true;
    std::uint32_t dispatches = 0;
    const auto stages =
        n > 1U ? static_cast<unsigned>(std::bit_width(static_cast<unsigned>(n - 1U))) : 0U;
    for (std::size_t it = 0; it < c.iterations; ++it) {
      if (stop.stop_requested()) {
        out.diagnostic = "terrain evolution cancelled";
        return false;
      }
      if (!begin()) {
        out.diagnostic = "terrain GPU command preparation failed";
        return false;
      }
      auto pass = [&](std::uint32_t mode) {
        Push p{static_cast<std::uint32_t>(w),
               static_cast<std::uint32_t>(h),
               mode,
               ha ? 1U : 0U,
               fa ? 1U : 0U,
               ja ? 1U : 0U,
               static_cast<float>(c.capacity_scale),
               static_cast<float>(c.erosion_rate),
               static_cast<float>(c.deposition_rate),
               static_cast<float>(c.maximum_height_change)};
        vkCmdPushConstants(command_, layout_, VK_SHADER_STAGE_COMPUTE_BIT, 0, sizeof p, &p);
        vkCmdDispatch(command_, static_cast<std::uint32_t>((n + 127) / 128), 1, 1);
        compute_barrier();
        ++dispatches;
      };
      if (it == 0U) {
        upload_initial_fields(n);
        out.gpu_mask_uploads = mask_cache_hit ? 0U : 1U;
        out.gpu_mask_upload_reuses = mask_cache_hit ? 1U : 0U;
        out.gpu_mask_cross_pipeline_reuses = cross_pipeline_reuse ? 1U : 0U;
      }
      pass(0); // downstream
      // F0 is one unit for each legal cell, and J0 is the selected downstream edge.
      pass(7);
      for (unsigned k = 0; k < stages; ++k) {
        pass(1);
        pass(2); // scatter flow and double the jump pointer from immutable inputs
        fa = !fa;
        ja = !ja;
      }
      pass(4);
      pass(5);
      pass(6);
      if (it + 1U == c.iterations)
        copy_final_fields(ha ? 2U : 1U, fa ? 4U : 5U, n);
      if (vkEndCommandBuffer(command_) != VK_SUCCESS) {
        out.diagnostic = "terrain GPU command finalization failed";
        return false;
      }
      std::string err;
      if (!vulkan::Runtime::instance().submit_and_wait(command_, err)) {
        out.diagnostic = "terrain GPU dispatch failed: " + err;
        return false;
      }
      ha = !ha;
      out.iterations_completed = it + 1;
      out.used_gpu = true;
      out.gpu_device = vulkan::Runtime::instance().device_name();
      out.gpu_dispatches += dispatches;
      out.gpu_buffer_reuses += reuses_;
      out.gpu_buffer_growths += growths_;
      dispatches = 0;
      reuses_ = growths_ = 0;
    }
    out.height.resize(n);
    out.flow.resize(n);
    out.divergence.resize(n);
    auto* hp = static_cast<const float*>(readback_[0].mapped);
    auto* fp = static_cast<const std::uint32_t*>(readback_[1].mapped);
    auto* dp = static_cast<const float*>(readback_[2].mapped);
    for (std::size_t i = 0; i < n; ++i) {
      out.height[i] = hp[i];
      out.flow[i] = fp[i];
      out.divergence[i] = dp[i];
      if (!std::isfinite(out.height[i]) || out.height[i] < 0.0 || out.flow[i] < 0.0 ||
          out.flow[i] > static_cast<double>(n) || !std::isfinite(out.divergence[i])) {
        out.diagnostic = "terrain GPU readback field integrity failure";
        return false;
      }
    }
    return true;
  }

private:
  void init() {
    auto& r = vulkan::Runtime::instance();
    if (!r.ok()) {
      diagnostic_ = r.diagnostic();
      return;
    }
    device_ = r.device();
    VkCommandPoolCreateInfo ci{VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO};
    ci.queueFamilyIndex = r.queue_family();
    ci.flags = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT;
    if (vkCreateCommandPool(device_, &ci, nullptr, &command_pool_) != VK_SUCCESS) {
      diagnostic_ = "terrain GPU command pool creation failed";
      return;
    }
    std::array<VkDescriptorSetLayoutBinding, 11> b{};
    for (std::uint32_t i = 0; i < b.size(); ++i)
      b[i] = {i, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr};
    VkDescriptorSetLayoutCreateInfo si{VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO};
    si.bindingCount = b.size();
    si.pBindings = b.data();
    if (vkCreateDescriptorSetLayout(device_, &si, nullptr, &set_layout_) != VK_SUCCESS) {
      diagnostic_ = "terrain GPU descriptor layout creation failed";
      return;
    }
    VkPushConstantRange pr{VK_SHADER_STAGE_COMPUTE_BIT, 0, sizeof(Push)};
    VkPipelineLayoutCreateInfo li{VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO};
    li.setLayoutCount = 1;
    li.pSetLayouts = &set_layout_;
    li.pushConstantRangeCount = 1;
    li.pPushConstantRanges = &pr;
    if (vkCreatePipelineLayout(device_, &li, nullptr, &layout_) != VK_SUCCESS) {
      diagnostic_ = "terrain GPU pipeline layout creation failed";
      return;
    }
    std::ifstream f(FLUX_TERRAIN_EVOLUTION_SHADER_PATH, std::ios::binary | std::ios::ate);
    if (!f) {
      diagnostic_ = "compiled terrain evolution shader is unavailable";
      return;
    }
    auto z = f.tellg();
    if (z <= 0 || z % 4) {
      diagnostic_ = "compiled terrain evolution shader is invalid";
      return;
    }
    std::vector<std::uint32_t> code(static_cast<size_t>(z) / 4);
    f.seekg(0);
    f.read(reinterpret_cast<char*>(code.data()), z);
    VkShaderModuleCreateInfo mi{VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO};
    mi.codeSize = code.size() * 4;
    mi.pCode = code.data();
    VkShaderModule m{};
    if (vkCreateShaderModule(device_, &mi, nullptr, &m) != VK_SUCCESS) {
      diagnostic_ = "terrain GPU shader module creation failed";
      return;
    }
    VkPipelineShaderStageCreateInfo st{VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO};
    st.stage = VK_SHADER_STAGE_COMPUTE_BIT;
    st.module = m;
    st.pName = "main";
    VkComputePipelineCreateInfo pi{VK_STRUCTURE_TYPE_COMPUTE_PIPELINE_CREATE_INFO};
    pi.stage = st;
    pi.layout = layout_;
    if (r.create_compute_pipeline(pi, pipeline_) != VK_SUCCESS)
      diagnostic_ = "terrain GPU pipeline creation failed";
    vkDestroyShaderModule(device_, m, nullptr);
  }
  bool ensure(size_t n, const std::shared_ptr<const vulkan::ImmutableMaskLease>& mask_lease) {
    growths_ = reuses_ = 0;
    for (std::size_t index = 1U; index < buffers_.size(); ++index) {
      auto& b = buffers_[index];
      if (b.handle && b.size >= n * 4) {
        ++reuses_;
        continue;
      }
      ++growths_;
      release(b);
      auto bytes = std::bit_ceil(std::max<size_t>(4096, n * 4));
      if (!allocate_device_buffer(bytes, b)) {
        return false;
      }
      b.size = bytes;
    }
    for (std::size_t index = 1U; index < upload_.size(); ++index)
      if (!ensure_host_buffer(n * 4U, upload_[index], false))
        return false;
    for (auto& b : readback_)
      if (!ensure_host_buffer(n * 4U, b, true))
        return false;
    if (!pool_) {
      VkDescriptorPoolSize ps{VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 11};
      VkDescriptorPoolCreateInfo pi{VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO};
      pi.maxSets = 1;
      pi.poolSizeCount = 1;
      pi.pPoolSizes = &ps;
      if (vkCreateDescriptorPool(device_, &pi, nullptr, &pool_) != VK_SUCCESS)
        return false;
      VkDescriptorSetAllocateInfo ai{VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO};
      ai.descriptorPool = pool_;
      ai.descriptorSetCount = 1;
      ai.pSetLayouts = &set_layout_;
      if (vkAllocateDescriptorSets(device_, &ai, &set_) != VK_SUCCESS)
        return false;
    }
    std::array<VkDescriptorBufferInfo, 11> bi{};
    std::array<VkWriteDescriptorSet, 11> wr{};
    for (uint32_t i = 0; i < 11; ++i) {
      bi[i] = {i == 0U ? mask_lease->buffer() : buffers_[i].handle, 0,
               i == 0U ? mask_lease->size() : buffers_[i].size};
      wr[i] = {VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET, nullptr, set_,   i,      0, 1,
               VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,      nullptr, &bi[i], nullptr};
    }
    vkUpdateDescriptorSets(device_, 11, wr.data(), 0, nullptr);
    return true;
  }
  bool begin() {
    if (!command_) {
      VkCommandBufferAllocateInfo a{VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO};
      a.commandPool = command_pool_;
      a.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
      a.commandBufferCount = 1;
      if (vkAllocateCommandBuffers(device_, &a, &command_) != VK_SUCCESS)
        return false;
    } else if (vkResetCommandBuffer(command_, 0) != VK_SUCCESS)
      return false;
    VkCommandBufferBeginInfo b{VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO};
    if (vkBeginCommandBuffer(command_, &b) != VK_SUCCESS)
      return false;
    vkCmdBindPipeline(command_, VK_PIPELINE_BIND_POINT_COMPUTE, pipeline_);
    vkCmdBindDescriptorSets(command_, VK_PIPELINE_BIND_POINT_COMPUTE, layout_, 0, 1, &set_, 0,
                            nullptr);
    return true;
  }
  void compute_barrier() {
    VkMemoryBarrier b{VK_STRUCTURE_TYPE_MEMORY_BARRIER};
    b.srcAccessMask = VK_ACCESS_SHADER_WRITE_BIT;
    b.dstAccessMask = VK_ACCESS_SHADER_READ_BIT | VK_ACCESS_SHADER_WRITE_BIT;
    vkCmdPipelineBarrier(command_, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
                         VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, 0, 1, &b, 0, nullptr, 0, nullptr);
  }
  void upload_initial_fields(std::size_t n) {
    VkMemoryBarrier host{VK_STRUCTURE_TYPE_MEMORY_BARRIER};
    host.srcAccessMask = VK_ACCESS_HOST_WRITE_BIT;
    host.dstAccessMask = VK_ACCESS_TRANSFER_READ_BIT;
    vkCmdPipelineBarrier(command_, VK_PIPELINE_STAGE_HOST_BIT, VK_PIPELINE_STAGE_TRANSFER_BIT, 0, 1,
                         &host, 0, nullptr, 0, nullptr);
    VkBufferCopy region{0U, 0U, static_cast<VkDeviceSize>(n * 4U)};
    vkCmdCopyBuffer(command_, upload_[1].handle, buffers_[1].handle, 1, &region);
    VkMemoryBarrier transfer{VK_STRUCTURE_TYPE_MEMORY_BARRIER};
    transfer.srcAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
    transfer.dstAccessMask = VK_ACCESS_SHADER_READ_BIT | VK_ACCESS_SHADER_WRITE_BIT;
    vkCmdPipelineBarrier(command_, VK_PIPELINE_STAGE_TRANSFER_BIT,
                         VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, 0, 1, &transfer, 0, nullptr, 0,
                         nullptr);
  }
  void copy_final_fields(std::uint32_t height, std::uint32_t flow, std::size_t n) {
    VkMemoryBarrier compute{VK_STRUCTURE_TYPE_MEMORY_BARRIER};
    compute.srcAccessMask = VK_ACCESS_SHADER_WRITE_BIT;
    compute.dstAccessMask = VK_ACCESS_TRANSFER_READ_BIT;
    vkCmdPipelineBarrier(command_, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
                         VK_PIPELINE_STAGE_TRANSFER_BIT, 0, 1, &compute, 0, nullptr, 0, nullptr);
    VkBufferCopy region{0U, 0U, static_cast<VkDeviceSize>(n * 4U)};
    vkCmdCopyBuffer(command_, buffers_[height].handle, readback_[0].handle, 1, &region);
    vkCmdCopyBuffer(command_, buffers_[flow].handle, readback_[1].handle, 1, &region);
    vkCmdCopyBuffer(command_, buffers_[10].handle, readback_[2].handle, 1, &region);
    VkMemoryBarrier b{VK_STRUCTURE_TYPE_MEMORY_BARRIER};
    b.srcAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
    b.dstAccessMask = VK_ACCESS_HOST_READ_BIT;
    vkCmdPipelineBarrier(command_, VK_PIPELINE_STAGE_TRANSFER_BIT, VK_PIPELINE_STAGE_HOST_BIT, 0, 1,
                         &b, 0, nullptr, 0, nullptr);
  }
  bool allocate_device_buffer(VkDeviceSize size, Buffer& output) {
    VkBufferCreateInfo info{VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO};
    info.size = size;
    info.usage = VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_SRC_BIT |
                 VK_BUFFER_USAGE_TRANSFER_DST_BIT;
    info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
    if (vkCreateBuffer(device_, &info, nullptr, &output.handle) != VK_SUCCESS)
      return false;
    VkMemoryRequirements requirements{};
    vkGetBufferMemoryRequirements(device_, output.handle, &requirements);
    std::uint32_t type = 0U;
    if (!vulkan::Runtime::instance().find_memory_type(requirements.memoryTypeBits,
                                                      VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT, type)) {
      release(output);
      return false;
    }
    VkMemoryAllocateInfo allocation{VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO};
    allocation.allocationSize = requirements.size;
    allocation.memoryTypeIndex = type;
    if (vkAllocateMemory(device_, &allocation, nullptr, &output.memory) != VK_SUCCESS ||
        vkBindBufferMemory(device_, output.handle, output.memory, 0U) != VK_SUCCESS) {
      release(output);
      return false;
    }
    return true;
  }
  bool ensure_host_buffer(std::size_t required, Buffer& buffer, bool prefer_cached) {
    if (buffer.handle != VK_NULL_HANDLE && buffer.size >= required) {
      ++reuses_;
      return true;
    }
    ++growths_;
    release(buffer);
    const auto capacity = std::bit_ceil(std::max<std::size_t>(4'096U, required));
    const auto allocated = prefer_cached
                               ? allocate_host_cached_buffer(capacity, buffer)
                               : vulkan::Runtime::instance().allocate_host_buffer(
                                     capacity, buffer.handle, buffer.memory, buffer.mapped);
    if (!allocated)
      return false;
    buffer.size = capacity;
    return true;
  }
  bool allocate_host_cached_buffer(VkDeviceSize size, Buffer& output) {
    VkBufferCreateInfo info{VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO};
    info.size = size;
    info.usage = VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_TRANSFER_SRC_BIT;
    info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
    if (vkCreateBuffer(device_, &info, nullptr, &output.handle) != VK_SUCCESS)
      return false;
    VkMemoryRequirements requirements{};
    vkGetBufferMemoryRequirements(device_, output.handle, &requirements);
    std::uint32_t type = 0U;
    constexpr auto cached = VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT |
                            VK_MEMORY_PROPERTY_HOST_COHERENT_BIT |
                            VK_MEMORY_PROPERTY_HOST_CACHED_BIT;
    if (!vulkan::Runtime::instance().find_memory_type(requirements.memoryTypeBits, cached, type)) {
      vkDestroyBuffer(device_, output.handle, nullptr);
      output.handle = VK_NULL_HANDLE;
      return vulkan::Runtime::instance().allocate_host_buffer(size, output.handle, output.memory,
                                                              output.mapped);
    }
    VkMemoryAllocateInfo allocation{VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO};
    allocation.allocationSize = requirements.size;
    allocation.memoryTypeIndex = type;
    if (vkAllocateMemory(device_, &allocation, nullptr, &output.memory) != VK_SUCCESS ||
        vkBindBufferMemory(device_, output.handle, output.memory, 0U) != VK_SUCCESS ||
        vkMapMemory(device_, output.memory, 0U, size, 0U, &output.mapped) != VK_SUCCESS) {
      release(output);
      return false;
    }
    return true;
  }
  void release(Buffer& b) {
    vulkan::Runtime::instance().release_buffer(b.handle, b.memory, b.mapped);
    b = {};
  }
  VkDevice device_{};
  VkCommandPool command_pool_{};
  VkCommandBuffer command_{};
  VkDescriptorSetLayout set_layout_{};
  VkPipelineLayout layout_{};
  VkDescriptorPool pool_{};
  VkDescriptorSet set_{};
  VkPipeline pipeline_{};
  std::array<Buffer, 11> buffers_{};
  std::array<Buffer, 2> upload_{};
  std::array<Buffer, 3> readback_{};
  std::timed_mutex mutex_;
  std::string diagnostic_;
  size_t reuses_{}, growths_{};
};
Context& context() {
  static Context value;
  return value;
}
} // namespace
bool terrain_evolution_gpu_runtime_available(std::string* d) {
  auto& c = context();
  if (d)
    *d = c.diagnostic();
  return c.ok();
}
bool evolve_terrain_gpu_fields(std::size_t w, std::size_t h, const std::vector<std::uint8_t>& walk,
                               const std::vector<double>& initial, const TerrainEvolutionConfig& c,
                               std::stop_token stop, TerrainEvolutionResult& r) {
  return context().run(w, h, walk, initial, c, stop, r);
}
} // namespace flux
