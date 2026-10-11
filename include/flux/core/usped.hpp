#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace flux {

// Geometry-only, dimensionless USPED refinement prior, not a copper or demand model.
struct UspedConfig {
  bool enabled{false};
  std::size_t maximum_cells{262'144U};
  std::size_t maximum_refined_parents{256U};
  std::uint32_t refinement_per_mille{200U};
  bool operator==(const UspedConfig&) const = default;
};

struct UspedField {
  std::vector<double> height;
  std::vector<double> accumulation;
  std::vector<double> transport;
  // Positive = erosion (outgoing capacity exceeds incoming); negative = deposition.
  std::vector<double> divergence;
  std::vector<double> refinement_priority;
  std::size_t sinks{0U};
  std::string diagnostic;
  [[nodiscard]] bool ok() const { return diagnostic.empty() && !height.empty(); }
};

// Clearance is measured in raster pitches. Masked cells are immutable mountains (height 1),
// receive no rainfall and cannot transport flow. Unit rain on free cells is geometric sampling,
// not net demand. D8 ties use stable cell order; flats remain sinks, never artificial outlets.
[[nodiscard]] UspedField build_usped_field(std::size_t width, std::size_t height,
                                           const std::vector<std::uint8_t>& walkable,
                                           const std::vector<double>& clearance,
                                           std::size_t maximum_cells = 262'144U);
} // namespace flux
