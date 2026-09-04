#include "vulkan_runtime.hpp"

#include <algorithm>
#include <charconv>
#include <cstdlib>
#include <limits>
#include <optional>
#include <string_view>
#include <vector>

namespace flux::vulkan {
namespace {

[[nodiscard]] std::optional<std::size_t> requested_device_index(std::string_view value) {
  std::size_t result = 0U;
  const auto parsed = std::from_chars(value.data(), value.data() + value.size(), result);
  if (parsed.ec != std::errc{} || parsed.ptr != value.data() + value.size())
    return std::nullopt;
  return result;
}

[[nodiscard]] bool is_gpu(VkPhysicalDeviceType type) {
  return type == VK_PHYSICAL_DEVICE_TYPE_DISCRETE_GPU ||
         type == VK_PHYSICAL_DEVICE_TYPE_INTEGRATED_GPU ||
         type == VK_PHYSICAL_DEVICE_TYPE_VIRTUAL_GPU;
}

[[nodiscard]] std::string device_type_name(VkPhysicalDeviceType type) {
  switch (type) {
  case VK_PHYSICAL_DEVICE_TYPE_DISCRETE_GPU:
    return "discrete";
  case VK_PHYSICAL_DEVICE_TYPE_INTEGRATED_GPU:
    return "integrated";
  case VK_PHYSICAL_DEVICE_TYPE_VIRTUAL_GPU:
    return "virtual";
  default:
    return "unsupported";
  }
}

} // namespace

Runtime& Runtime::instance() {
  static Runtime runtime;
  return runtime;
}

Runtime::Runtime() {
  VkApplicationInfo application{VK_STRUCTURE_TYPE_APPLICATION_INFO};
  application.pApplicationName = "Flux shared compute runtime";
  application.apiVersion = VK_API_VERSION_1_1;
  VkInstanceCreateInfo instance_info{VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO};
  instance_info.pApplicationInfo = &application;
  if (vkCreateInstance(&instance_info, nullptr, &instance_) != VK_SUCCESS) {
    diagnostic_ = "shared Vulkan instance creation failed";
    return;
  }

  std::uint32_t physical_count = 0U;
  if (vkEnumeratePhysicalDevices(instance_, &physical_count, nullptr) != VK_SUCCESS ||
      physical_count == 0U) {
    diagnostic_ = "shared Vulkan runtime found no compute device";
    return;
  }
  std::vector<VkPhysicalDevice> devices(physical_count);
  vkEnumeratePhysicalDevices(instance_, &physical_count, devices.data());

  const char* override_text = std::getenv("FLUX_VULKAN_DEVICE");
  const std::string_view requested =
      override_text != nullptr ? std::string_view{override_text} : std::string_view{};
  const auto requested_index = requested.empty() ? std::nullopt : requested_device_index(requested);
  struct Candidate {
    VkPhysicalDevice device{VK_NULL_HANDLE};
    std::uint32_t family{0U};
    std::size_t index{0U};
    VkPhysicalDeviceProperties properties{};
  };
  std::vector<Candidate> candidates;
  for (std::size_t device_index = 0U; device_index < devices.size(); ++device_index) {
    std::uint32_t family_count = 0U;
    vkGetPhysicalDeviceQueueFamilyProperties(devices[device_index], &family_count, nullptr);
    std::vector<VkQueueFamilyProperties> families(family_count);
    vkGetPhysicalDeviceQueueFamilyProperties(devices[device_index], &family_count, families.data());
    for (std::uint32_t family = 0U; family < family_count; ++family) {
      if ((families[family].queueFlags & VK_QUEUE_COMPUTE_BIT) == 0U)
        continue;
      Candidate candidate;
      candidate.device = devices[device_index];
      candidate.family = family;
      candidate.index = device_index;
      vkGetPhysicalDeviceProperties(candidate.device, &candidate.properties);
      if (!is_gpu(candidate.properties.deviceType))
        break;
      candidates.push_back(candidate);
      break;
    }
  }
  if (candidates.empty()) {
    diagnostic_ = "shared Vulkan runtime found no compute-capable GPU";
    return;
  }
  const auto matches_override = [&](const Candidate& candidate) {
    if (requested.empty())
      return false;
    if (requested_index)
      return candidate.index == *requested_index;
    return std::string_view{candidate.properties.deviceName}.find(requested) !=
           std::string_view::npos;
  };
  auto selected = requested.empty()
                      ? candidates.end()
                      : std::find_if(candidates.begin(), candidates.end(), matches_override);
  if (!requested.empty() && selected == candidates.end()) {
    diagnostic_ = "FLUX_VULKAN_DEVICE does not identify a compute-capable Vulkan device";
    return;
  }
  if (selected == candidates.end()) {
    selected = std::find_if(candidates.begin(), candidates.end(), [](const Candidate& candidate) {
      return candidate.properties.deviceType == VK_PHYSICAL_DEVICE_TYPE_DISCRETE_GPU;
    });
  }
  if (selected == candidates.end())
    selected = candidates.begin();
  physical_ = selected->device;
  queue_family_ = selected->family;
  device_name_ = selected->properties.deviceName;
  device_type_ = device_type_name(selected->properties.deviceType);
  vendor_id_ = selected->properties.vendorID;
  device_id_ = selected->properties.deviceID;
  api_version_ = selected->properties.apiVersion;
  driver_version_ = selected->properties.driverVersion;

  const float priority = 1.0F;
  VkDeviceQueueCreateInfo queue_info{VK_STRUCTURE_TYPE_DEVICE_QUEUE_CREATE_INFO};
  queue_info.queueFamilyIndex = queue_family_;
  queue_info.queueCount = 1U;
  queue_info.pQueuePriorities = &priority;
  VkDeviceCreateInfo device_info{VK_STRUCTURE_TYPE_DEVICE_CREATE_INFO};
  device_info.queueCreateInfoCount = 1U;
  device_info.pQueueCreateInfos = &queue_info;
  VkPhysicalDeviceFeatures supported_features{};
  vkGetPhysicalDeviceFeatures(physical_, &supported_features);
  // Routing narrow-phase kernels use 64-bit integer cross products so nanometre-coordinate
  // intersection classification is not delegated to float. GPU compute is a mandatory Flux
  // requirement; fail closed on hardware that cannot provide the feature.
  if (supported_features.shaderInt64 != VK_TRUE) {
    diagnostic_ = "selected Vulkan device lacks shaderInt64 required by routing compute";
    return;
  }
  VkPhysicalDeviceFeatures enabled_features{};
  enabled_features.shaderInt64 = VK_TRUE;
  device_info.pEnabledFeatures = &enabled_features;
  if (vkCreateDevice(physical_, &device_info, nullptr, &device_) != VK_SUCCESS) {
    diagnostic_ = "shared Vulkan logical device creation failed";
    return;
  }
  vkGetDeviceQueue(device_, queue_family_, 0U, &queue_);
  VkPipelineCacheCreateInfo cache_info{VK_STRUCTURE_TYPE_PIPELINE_CACHE_CREATE_INFO};
  if (vkCreatePipelineCache(device_, &cache_info, nullptr, &pipeline_cache_) != VK_SUCCESS) {
    diagnostic_ = "shared Vulkan pipeline cache creation failed";
    return;
  }
  VkFenceCreateInfo fence_info{VK_STRUCTURE_TYPE_FENCE_CREATE_INFO};
  for (auto& fence : fences_) {
    if (vkCreateFence(device_, &fence_info, nullptr, &fence) != VK_SUCCESS) {
      diagnostic_ = "shared Vulkan in-flight fence creation failed";
      return;
    }
  }
}

Runtime::~Runtime() {
  if (device_ != VK_NULL_HANDLE)
    vkDeviceWaitIdle(device_);
  for (const auto fence : fences_) {
    if (fence != VK_NULL_HANDLE)
      vkDestroyFence(device_, fence, nullptr);
  }
  if (pipeline_cache_ != VK_NULL_HANDLE)
    vkDestroyPipelineCache(device_, pipeline_cache_, nullptr);
  if (device_ != VK_NULL_HANDLE)
    vkDestroyDevice(device_, nullptr);
  if (instance_ != VK_NULL_HANDLE)
    vkDestroyInstance(instance_, nullptr);
}

bool Runtime::submit_and_wait(VkCommandBuffer command, std::string& error) {
  if (!ok() || command == VK_NULL_HANDLE) {
    error = ok() ? "invalid shared Vulkan command buffer" : diagnostic_;
    return false;
  }
  std::size_t slot = 0U;
  {
    std::unique_lock lock(fence_mutex_);
    fence_available_.wait(lock, [&] {
      return std::any_of(fence_busy_.begin(), fence_busy_.end(), [](bool busy) { return !busy; });
    });
    slot = static_cast<std::size_t>(std::find(fence_busy_.begin(), fence_busy_.end(), false) -
                                    fence_busy_.begin());
    fence_busy_[slot] = true;
  }
  const auto release_slot = [&] {
    {
      const std::lock_guard lock(fence_mutex_);
      fence_busy_[slot] = false;
    }
    fence_available_.notify_one();
  };
  if (vkResetFences(device_, 1U, &fences_[slot]) != VK_SUCCESS) {
    error = "shared Vulkan fence reset failed";
    release_slot();
    return false;
  }
  VkSubmitInfo submit{VK_STRUCTURE_TYPE_SUBMIT_INFO};
  submit.commandBufferCount = 1U;
  submit.pCommandBuffers = &command;
  VkResult submitted = VK_SUCCESS;
  {
    const std::lock_guard lock(queue_mutex_);
    submitted = vkQueueSubmit(queue_, 1U, &submit, fences_[slot]);
  }
  if (submitted != VK_SUCCESS ||
      vkWaitForFences(device_, 1U, &fences_[slot], VK_TRUE,
                      std::numeric_limits<std::uint64_t>::max()) != VK_SUCCESS) {
    error = "shared Vulkan fenced dispatch failed";
    release_slot();
    return false;
  }
  release_slot();
  return true;
}

bool Runtime::find_memory_type(std::uint32_t type_bits, VkMemoryPropertyFlags required,
                               std::uint32_t& selected) const {
  VkPhysicalDeviceMemoryProperties properties{};
  vkGetPhysicalDeviceMemoryProperties(physical_, &properties);
  for (std::uint32_t index = 0U; index < properties.memoryTypeCount; ++index) {
    if ((type_bits & (1U << index)) != 0U &&
        (properties.memoryTypes[index].propertyFlags & required) == required) {
      selected = index;
      return true;
    }
  }
  return false;
}

bool Runtime::allocate_host_buffer(VkDeviceSize size, VkBuffer& buffer, VkDeviceMemory& memory,
                                   void*& mapped) const {
  if (!ok() || size == 0U)
    return false;
  VkBufferCreateInfo info{VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO};
  info.size = size;
  info.usage = VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_SRC_BIT |
               VK_BUFFER_USAGE_TRANSFER_DST_BIT;
  info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
  if (vkCreateBuffer(device_, &info, nullptr, &buffer) != VK_SUCCESS)
    return false;
  VkMemoryRequirements requirements{};
  vkGetBufferMemoryRequirements(device_, buffer, &requirements);
  std::uint32_t type = 0U;
  if (!find_memory_type(requirements.memoryTypeBits,
                        VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT,
                        type)) {
    vkDestroyBuffer(device_, buffer, nullptr);
    buffer = VK_NULL_HANDLE;
    return false;
  }
  VkMemoryAllocateInfo allocation{VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO};
  allocation.allocationSize = requirements.size;
  allocation.memoryTypeIndex = type;
  if (vkAllocateMemory(device_, &allocation, nullptr, &memory) != VK_SUCCESS ||
      vkBindBufferMemory(device_, buffer, memory, 0U) != VK_SUCCESS ||
      vkMapMemory(device_, memory, 0U, size, 0U, &mapped) != VK_SUCCESS) {
    release_buffer(buffer, memory, mapped);
    return false;
  }
  return true;
}

void Runtime::release_buffer(VkBuffer& buffer, VkDeviceMemory& memory, void*& mapped) const {
  if (mapped != nullptr && memory != VK_NULL_HANDLE)
    vkUnmapMemory(device_, memory);
  if (buffer != VK_NULL_HANDLE)
    vkDestroyBuffer(device_, buffer, nullptr);
  if (memory != VK_NULL_HANDLE)
    vkFreeMemory(device_, memory, nullptr);
  buffer = VK_NULL_HANDLE;
  memory = VK_NULL_HANDLE;
  mapped = nullptr;
}

VkResult Runtime::create_compute_pipeline(const VkComputePipelineCreateInfo& info,
                                          VkPipeline& pipeline) {
  const std::lock_guard lock(pipeline_cache_mutex_);
  return vkCreateComputePipelines(device_, pipeline_cache_, 1U, &info, nullptr, &pipeline);
}

} // namespace flux::vulkan
