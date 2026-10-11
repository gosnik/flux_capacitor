#pragma once

#include <vulkan/vulkan.h>

#include <array>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace flux::vulkan {

enum class MaskConsumer : std::uint8_t { terrain, hydraulic };

// An immutable, device-local normalized u32 walkability field.  A lease keeps its buffer alive
// while a client has recorded or submitted commands that reference it.
class ImmutableMaskLease {
public:
  ~ImmutableMaskLease();
  [[nodiscard]] VkBuffer buffer() const { return buffer_; }
  [[nodiscard]] VkDeviceSize size() const { return size_; }
  [[nodiscard]] std::size_t width() const { return width_; }
  [[nodiscard]] std::size_t height() const { return height_; }

private:
  friend class Runtime;
  ImmutableMaskLease() = default;

  VkDevice device_{VK_NULL_HANDLE};
  VkBuffer buffer_{VK_NULL_HANDLE};
  VkDeviceMemory memory_{VK_NULL_HANDLE};
  VkDeviceSize size_{0U};
  VkDeviceSize allocation_size_{0U};
  std::size_t width_{0U};
  std::size_t height_{0U};
  std::vector<std::uint32_t> contents_;
  MaskConsumer last_consumer_{MaskConsumer::terrain};
  std::uint64_t last_use_{0U};
};

// Process-wide compute runtime. Vulkan queue operations require external synchronization, but a
// queue need not be idle before the next independent submission is enqueued. submit_and_wait()
// therefore serializes only vkQueueSubmit and waits on one of a bounded ring of fences outside the
// queue lock. Hydraulic, routing-compute, and XPBD contexts retain their own command pools and
// descriptor state while sharing device ownership and the driver pipeline cache.
class Runtime {
public:
  static Runtime& instance();

  Runtime(const Runtime&) = delete;
  Runtime& operator=(const Runtime&) = delete;

  [[nodiscard]] bool ok() const { return diagnostic_.empty(); }
  [[nodiscard]] const std::string& diagnostic() const { return diagnostic_; }
  [[nodiscard]] VkInstance instance_handle() const { return instance_; }
  [[nodiscard]] VkPhysicalDevice physical_device() const { return physical_; }
  [[nodiscard]] VkDevice device() const { return device_; }
  [[nodiscard]] VkQueue queue() const { return queue_; }
  [[nodiscard]] std::uint32_t queue_family() const { return queue_family_; }
  [[nodiscard]] VkPipelineCache pipeline_cache() const { return pipeline_cache_; }
  [[nodiscard]] const std::string& device_name() const { return device_name_; }
  [[nodiscard]] const std::string& device_type() const { return device_type_; }
  [[nodiscard]] std::uint32_t vendor_id() const { return vendor_id_; }
  [[nodiscard]] std::uint32_t device_id() const { return device_id_; }
  [[nodiscard]] std::uint32_t api_version() const { return api_version_; }
  [[nodiscard]] std::uint32_t driver_version() const { return driver_version_; }

  [[nodiscard]] bool submit_and_wait(VkCommandBuffer command, std::string& error);
  [[nodiscard]] bool find_memory_type(std::uint32_t type_bits, VkMemoryPropertyFlags required,
                                      std::uint32_t& selected) const;
  [[nodiscard]] bool allocate_host_buffer(VkDeviceSize size, VkBuffer& buffer,
                                          VkDeviceMemory& memory, void*& mapped) const;
  void release_buffer(VkBuffer& buffer, VkDeviceMemory& memory, void*& mapped) const;
  // Publishes a normalized immutable mask through a fenced transfer submission. Exact dimensions
  // and every normalized u32 are part of its identity. A cache hit may be shared across terrain
  // and hydraulic pipelines; `cross_pipeline_reuse` reports that case.
  [[nodiscard]] bool acquire_immutable_mask(std::size_t width, std::size_t height,
                                             const std::vector<std::uint8_t>& walkable,
                                             MaskConsumer consumer,
                                             std::shared_ptr<const ImmutableMaskLease>& lease,
                                             bool& cache_hit, bool& cross_pipeline_reuse,
                                             std::string& error);
  [[nodiscard]] VkResult create_compute_pipeline(const VkComputePipelineCreateInfo& info,
                                                 VkPipeline& pipeline);

private:
  Runtime();
  ~Runtime();

  static constexpr std::size_t in_flight_count = 8U;

  VkInstance instance_{VK_NULL_HANDLE};
  VkPhysicalDevice physical_{VK_NULL_HANDLE};
  VkDevice device_{VK_NULL_HANDLE};
  VkQueue queue_{VK_NULL_HANDLE};
  std::uint32_t queue_family_{0U};
  VkPipelineCache pipeline_cache_{VK_NULL_HANDLE};
  std::array<VkFence, in_flight_count> fences_{};
  std::array<bool, in_flight_count> fence_busy_{};
  std::mutex queue_mutex_;
  std::mutex pipeline_cache_mutex_;
  std::mutex mask_cache_mutex_;
  std::mutex fence_mutex_;
  std::condition_variable fence_available_;
  std::string device_name_;
  std::string device_type_;
  std::uint32_t vendor_id_{0U};
  std::uint32_t device_id_{0U};
  std::uint32_t api_version_{0U};
  std::uint32_t driver_version_{0U};
  std::string diagnostic_;
  std::deque<std::shared_ptr<ImmutableMaskLease>> immutable_masks_;
  std::uint64_t immutable_mask_use_{0U};
  std::size_t immutable_mask_bytes_{0U};
  static constexpr std::size_t maximum_immutable_mask_entries = 16U;
  static constexpr std::size_t maximum_immutable_mask_bytes = 64U * 1024U * 1024U;
};

} // namespace flux::vulkan
