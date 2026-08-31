#pragma once

#include "flux/core/free_space_skeleton.hpp"

#include <cstddef>
#include <string>
#include <vector>

namespace flux {

struct OffsetFreeSpaceRegion {
  Polygon area;
  std::vector<Polygon> holes;

  bool operator==(const OffsetFreeSpaceRegion&) const = default;
};

struct FreeSpaceOffsetConfig {
  Nanometres clearance;
  std::size_t maximum_output_vertices{1'000'000};
};

struct FreeSpaceOffsetResult {
  std::vector<OffsetFreeSpaceRegion> regions;
  std::size_t output_vertices{0};
  std::string diagnostic;

  [[nodiscard]] bool ok() const { return diagnostic.empty(); }
};

// Creates a conservative integer-miter free-space offset. The outer boundary is eroded and
// obstacle solids are dilated by clearance plus a two-nanometre rounding guard, then subtracted.
// Every resulting contour is revalidated against the unoffset domain and requested clearance.
[[nodiscard]] FreeSpaceOffsetResult
offset_polygonal_free_space(const FreeSpaceDomain& domain,
                            const FreeSpaceOffsetConfig& config = {});

} // namespace flux
