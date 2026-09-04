#include "flux/core/hydraulic_path.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

namespace {

using Pixel = std::array<std::uint8_t, 3U>;

constexpr std::size_t width = 256U;
constexpr std::size_t height = 160U;

void exclude_circle(std::vector<std::uint8_t>& walkable, int cx, int cy, int radius) {
  for (int y = std::max(0, cy - radius); y <= std::min<int>(height - 1U, cy + radius); ++y) {
    for (int x = std::max(0, cx - radius); x <= std::min<int>(width - 1U, cx + radius); ++x) {
      const auto dx = x - cx;
      const auto dy = y - cy;
      if (dx * dx + dy * dy <= radius * radius) {
        walkable[static_cast<std::size_t>(y) * width + static_cast<std::size_t>(x)] = 0U;
      }
    }
  }
}

void exclude_rectangle(std::vector<std::uint8_t>& walkable, std::size_t x0, std::size_t y0,
                       std::size_t x1, std::size_t y1) {
  for (auto y = y0; y <= y1; ++y) {
    for (auto x = x0; x <= x1; ++x)
      walkable[y * width + x] = 0U;
  }
}

[[nodiscard]] Pixel terrain_colour(double height_value) {
  height_value = std::clamp(height_value, 0.0, 1.0);
  if (height_value < 0.5) {
    const auto t = height_value * 2.0;
    return Pixel{static_cast<std::uint8_t>(16.0 + 42.0 * t),
                 static_cast<std::uint8_t>(72.0 + 126.0 * t),
                 static_cast<std::uint8_t>(166.0 - 74.0 * t)};
  }
  const auto t = (height_value - 0.5) * 2.0;
  return Pixel{static_cast<std::uint8_t>(58.0 + 188.0 * t),
               static_cast<std::uint8_t>(198.0 - 86.0 * t),
               static_cast<std::uint8_t>(92.0 - 58.0 * t)};
}

void blend(Pixel& destination, Pixel source, double alpha) {
  for (std::size_t channel = 0U; channel < destination.size(); ++channel) {
    destination[channel] = static_cast<std::uint8_t>(
        std::clamp((1.0 - alpha) * destination[channel] + alpha * source[channel], 0.0, 255.0));
  }
}

} // namespace

int main(int argc, char** argv) {
  const std::filesystem::path output = argc > 1 ? argv[1] : "hydraulic-heightmap.ppm";
  flux::HydraulicPathInput input;
  input.width = width;
  input.height = height;
  input.walkable.assign(width * height, 1U);

  // Representative expanded PCB obstacles: circular pads, vias, and rectangular keepouts. The
  // two keepout walls leave a strategic middle neck while the islands create alternate valleys.
  exclude_rectangle(input.walkable, 0U, 0U, width - 1U, 2U);
  exclude_rectangle(input.walkable, 0U, height - 3U, width - 1U, height - 1U);
  exclude_circle(input.walkable, 55, 38, 13);
  exclude_circle(input.walkable, 55, 122, 13);
  exclude_circle(input.walkable, 92, 72, 8);
  exclude_circle(input.walkable, 92, 102, 8);
  exclude_rectangle(input.walkable, 112U, 3U, 139U, 59U);
  exclude_rectangle(input.walkable, 112U, 101U, 139U, height - 4U);
  exclude_circle(input.walkable, 169, 72, 15);
  exclude_circle(input.walkable, 169, 111, 10);
  exclude_circle(input.walkable, 211, 47, 8);
  exclude_rectangle(input.walkable, 197U, 91U, 222U, 108U);

  input.source = 12U + 80U * width;
  input.target = 244U + 80U * width;
  input.walkable[input.source] = 1U;
  input.walkable[input.target] = 1U;

  flux::HydraulicPathConfig config;
  config.relaxation_iterations = width + height;
  config.droplet_batches = 8U;
  config.droplets_per_batch = 4'096U;
  config.maximum_steps_per_droplet = width + height;
  config.prefer_gpu = argc <= 2 || std::string{argv[2]} != "--cpu";
  const auto result = flux::solve_hydraulic_erosion_path(input, config);
  if (!result.ok()) {
    std::cerr << "hydraulic demo failed: " << result.diagnostic << '\n';
    return 1;
  }

  std::uint32_t maximum_height = 0U;
  std::uint32_t maximum_flow = 0U;
  std::uint32_t maximum_erosion = 0U;
  std::size_t free_cells = 0U;
  std::size_t finite_cells = 0U;
  std::size_t eroded_cells = 0U;
  for (std::size_t cell = 0U; cell < input.walkable.size(); ++cell) {
    if (input.walkable[cell] == 0U)
      continue;
    ++free_cells;
    if (result.terrain_height[cell] != flux::hydraulic_unreachable_height) {
      ++finite_cells;
      maximum_height = std::max(maximum_height, result.terrain_height[cell]);
    }
    maximum_flow = std::max(maximum_flow, result.accumulated_flow[cell]);
    maximum_erosion = std::max(maximum_erosion, result.erosion[cell]);
    eroded_cells += result.erosion[cell] != 0U ? 1U : 0U;
  }

  constexpr std::size_t panels = 4U;
  std::vector<Pixel> image(width * panels * height, Pixel{0U, 0U, 0U});
  for (std::size_t y = 0U; y < height; ++y) {
    for (std::size_t x = 0U; x < width; ++x) {
      const auto cell = y * width + x;
      if (input.walkable[cell] == 0U)
        continue;
      const auto terrain = maximum_height == 0U
                               ? 0.0
                               : static_cast<double>(result.terrain_height[cell]) / maximum_height;
      const auto flow = maximum_flow == 0U
                            ? 0.0
                            : static_cast<double>(result.accumulated_flow[cell]) / maximum_flow;
      const auto erosion_linear =
          maximum_erosion == 0U ? 0.0 : static_cast<double>(result.erosion[cell]) / maximum_erosion;
      const auto erosion = std::sqrt(erosion_linear);
      image[y * width * panels + x] = terrain_colour(terrain);
      image[y * width * panels + width + x] =
          Pixel{static_cast<std::uint8_t>(18.0 + erosion * 237.0),
                static_cast<std::uint8_t>(22.0 + erosion * 154.0),
                static_cast<std::uint8_t>(34.0 + erosion * 54.0)};
      image[y * width * panels + width * 2U + x] =
          Pixel{static_cast<std::uint8_t>(10.0 + flow * 62.0),
                static_cast<std::uint8_t>(25.0 + flow * 206.0),
                static_cast<std::uint8_t>(42.0 + flow * 213.0)};
      auto composite = terrain_colour(terrain);
      blend(composite, Pixel{18U, 242U, 244U}, std::min(0.88, flow * 0.65 + erosion * 0.35));
      image[y * width * panels + width * 3U + x] = composite;
    }
  }
  for (const auto cell : result.riverbed) {
    const auto x = cell % width;
    const auto y = cell / width;
    image[y * width * panels + width * 3U + x] = Pixel{255U, 255U, 255U};
  }
  image[80U * width * panels + input.source % width] = Pixel{255U, 32U, 48U};
  image[80U * width * panels + width * 3U + input.source % width] = Pixel{255U, 32U, 48U};
  image[80U * width * panels + input.target % width] = Pixel{36U, 255U, 72U};
  image[80U * width * panels + width * 3U + input.target % width] = Pixel{36U, 255U, 72U};

  if (output.has_parent_path())
    std::filesystem::create_directories(output.parent_path());
  std::ofstream stream(output, std::ios::binary);
  stream << "P6\n" << width * panels << ' ' << height << "\n255\n";
  stream.write(reinterpret_cast<const char*>(image.data()),
               static_cast<std::streamsize>(image.size() * sizeof(Pixel)));
  if (!stream) {
    std::cerr << "could not write " << output << '\n';
    return 1;
  }
  std::cout << "output=" << output << " backend=" << (result.used_gpu ? "vulkan" : "cpu")
            << " free_cells=" << free_cells << " finite_height_cells=" << finite_cells
            << " eroded_cells=" << eroded_cells << " riverbed_cells=" << result.riverbed.size()
            << " droplets_reaching_target=" << result.droplets_reaching_target
            << " fused_clearance=" << (result.used_gpu_clearance_field ? "true" : "false")
            << " clearance_cache_hit=" << (result.gpu_clearance_cache_hit ? "true" : "false")
            << " gpu_batch_size=" << result.gpu_batch_size
            << " gpu_buffer_reuses=" << result.gpu_persistent_buffer_reuses
            << " gpu_buffer_growths=" << result.gpu_buffer_growths << '\n';
  return finite_cells == free_cells ? 0 : 2;
}
