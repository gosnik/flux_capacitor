#pragma once

#include <vulkan/vulkan.h>

#include <array>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <string>

namespace flux::vulkan {

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

  [[nodiscard]] bool submit_and_wait(VkCommandBuffer command, std::string& error);
  [[nodiscard]] bool find_memory_type(std::uint32_t type_bits, VkMemoryPropertyFlags required,
                                      std::uint32_t& selected) const;
  [[nodiscard]] bool allocate_host_buffer(VkDeviceSize size, VkBuffer& buffer,
                                          VkDeviceMemory& memory, void*& mapped) const;
  void release_buffer(VkBuffer& buffer, VkDeviceMemory& memory, void*& mapped) const;
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
  std::mutex fence_mutex_;
  std::condition_variable fence_available_;
  std::string device_name_;
  std::string diagnostic_;
};

} // namespace flux::vulkan
