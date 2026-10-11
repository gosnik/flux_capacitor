#pragma once

#include "flux/core/terrain_evolution.hpp"

#include <cstddef>
#include <cstdint>
#include <stop_token>
#include <string>
#include <vector>

namespace flux {

// Evolves fields only.  The caller owns validation, initial-height construction, and path search.
[[nodiscard]] bool evolve_terrain_gpu_fields(std::size_t width, std::size_t height,
                                             const std::vector<std::uint8_t>& walkable,
                                             const std::vector<double>& initial_height,
                                             const TerrainEvolutionConfig& config,
                                             std::stop_token stop_token,
                                             TerrainEvolutionResult& result);

// This reports whether the Vulkan runtime can be initialized; it does not claim a dispatch probe.
[[nodiscard]] bool terrain_evolution_gpu_runtime_available(std::string* diagnostic = nullptr);

} // namespace flux
