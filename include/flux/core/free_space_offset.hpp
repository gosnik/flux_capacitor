#pragma once

#include "flux/core/free_space_skeleton.hpp"

#include <cstddef>
#include <memory>
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
  // Optional heterogeneous obstacle clearances aligned with FreeSpaceDomain::obstacles and
  // filled_obstacles. Empty vectors retain the uniform-clearance API. The outer outline and
  // cutouts always use `clearance`.
  std::vector<Nanometres> obstacle_clearances;
  std::vector<Nanometres> filled_obstacle_clearances;

  FreeSpaceOffsetConfig() = default;
  explicit FreeSpaceOffsetConfig(Nanometres uniform_clearance,
                                 std::size_t vertex_budget = 1'000'000)
      : clearance(uniform_clearance), maximum_output_vertices(vertex_budget) {}
};

struct FreeSpaceOffsetResult {
  std::vector<OffsetFreeSpaceRegion> regions;
  std::size_t output_vertices{0};
  std::string diagnostic;

  [[nodiscard]] bool ok() const { return diagnostic.empty(); }
};

// Reuses a single solid/filled obstacle expansion by its complete contour geometry and exact
// effective clearance. The cache never stores a final free-space result: outer boundaries,
// cutouts, exclusions, and all other obstacles are still combined for each call.
class FreeSpaceOffsetCache {
 public:
  struct Stats { std::size_t hits{0}; std::size_t misses{0}; std::size_t entries{0}; };

  explicit FreeSpaceOffsetCache(std::size_t capacity = 256U);
  ~FreeSpaceOffsetCache();
  FreeSpaceOffsetCache(FreeSpaceOffsetCache&&) noexcept;
  FreeSpaceOffsetCache& operator=(FreeSpaceOffsetCache&&) noexcept;
  FreeSpaceOffsetCache(const FreeSpaceOffsetCache&) = delete;
  FreeSpaceOffsetCache& operator=(const FreeSpaceOffsetCache&) = delete;

  [[nodiscard]] Stats stats() const;
  void clear();

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
  friend FreeSpaceOffsetResult offset_polygonal_free_space(
      const FreeSpaceDomain&, const FreeSpaceOffsetConfig&, FreeSpaceOffsetCache*);
};

// Creates a conservative integer-miter free-space offset. The outer boundary is eroded and
// obstacle solids are dilated by clearance plus a two-nanometre rounding guard, then subtracted.
// Every resulting contour is revalidated against the unoffset domain and requested clearance.
[[nodiscard]] FreeSpaceOffsetResult
offset_polygonal_free_space(const FreeSpaceDomain& domain,
                            const FreeSpaceOffsetConfig& config = {},
                            FreeSpaceOffsetCache* cache = nullptr);

} // namespace flux
