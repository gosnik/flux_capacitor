#include "flux/core/free_space_offset.hpp"

#include "flux/core/geometry.hpp"

#include <boost/polygon/polygon.hpp>
#include <boost/polygon/polygon_set_concept.hpp>
#include <boost/polygon/polygon_set_data.hpp>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <list>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

namespace flux {
struct FreeSpaceOffsetCache::Impl {
  struct Entry {
    OffsetFreeSpaceRegion source;
    Nanometres clearance;
    std::vector<OffsetFreeSpaceRegion> expanded;
    std::size_t age{0U};
  };
  explicit Impl(std::size_t requested_capacity)
      : capacity(std::max<std::size_t>(1U, requested_capacity)) {}
  std::size_t capacity;
  std::size_t clock{0U};
  std::size_t hits{0U};
  std::size_t misses{0U};
  std::unordered_map<std::uint64_t, std::vector<Entry>> entries;
  std::size_t entry_count{0U};
  mutable std::mutex mutex;
};

FreeSpaceOffsetCache::FreeSpaceOffsetCache(std::size_t capacity)
    : impl_(std::make_unique<Impl>(capacity)) {}
FreeSpaceOffsetCache::~FreeSpaceOffsetCache() = default;
FreeSpaceOffsetCache::FreeSpaceOffsetCache(FreeSpaceOffsetCache&&) noexcept = default;
FreeSpaceOffsetCache& FreeSpaceOffsetCache::operator=(FreeSpaceOffsetCache&&) noexcept = default;
FreeSpaceOffsetCache::Stats FreeSpaceOffsetCache::stats() const {
  std::lock_guard lock(impl_->mutex);
  return {impl_->hits, impl_->misses, impl_->entry_count};
}
void FreeSpaceOffsetCache::clear() {
  std::lock_guard lock(impl_->mutex);
  impl_->entries.clear();
  impl_->hits = impl_->misses = impl_->clock = impl_->entry_count = 0U;
}

namespace {

[[nodiscard]] std::uint64_t expansion_key(const OffsetFreeSpaceRegion& region,
                                          Nanometres clearance) {
  std::uint64_t hash = 1469598103934665603ULL;
  const auto add = [&hash](std::uint64_t value) {
    hash ^= value;
    hash *= 1099511628211ULL;
  };
  const auto add_polygon = [&](const Polygon& polygon) {
    add(polygon.vertices.size());
    for (const auto& point : polygon.vertices) {
      add(static_cast<std::uint64_t>(point.x.value()));
      add(static_cast<std::uint64_t>(point.y.value()));
    }
  };
  add(static_cast<std::uint64_t>(clearance.value()));
  add_polygon(region.area);
  add(region.holes.size());
  for (const auto& hole : region.holes) add_polygon(hole);
  return hash;
}

using Coordinate = std::int32_t;
using BoostPoint = boost::polygon::point_data<Coordinate>;
using BoostPolygon = boost::polygon::polygon_data<Coordinate>;
using BoostPolygonWithHoles = boost::polygon::polygon_with_holes_data<Coordinate>;
using BoostPolygonSet = boost::polygon::polygon_set_data<Coordinate>;

struct PreparedFilledRegion {
  geometry::PreparedPolygon area;
  std::vector<geometry::PreparedPolygon> holes;
};

struct PreparedValidationDomain {
  geometry::PreparedPolygon outline;
  std::vector<geometry::PreparedPolygon> cutouts;
  std::vector<geometry::PreparedPolygon> obstacles;
  std::vector<PreparedFilledRegion> filled_obstacles;
};

[[nodiscard]] PreparedFilledRegion prepare_filled_region(const Polygon& area,
                                                         const std::vector<Polygon>& holes) {
  PreparedFilledRegion result;
  result.area = geometry::prepare_polygon(area);
  result.holes.reserve(holes.size());
  std::transform(holes.begin(), holes.end(), std::back_inserter(result.holes),
                 [](const auto& hole) { return geometry::prepare_polygon(hole); });
  return result;
}

[[nodiscard]] PreparedValidationDomain prepare_validation_domain(const FreeSpaceDomain& domain) {
  PreparedValidationDomain result;
  result.outline = geometry::prepare_polygon(domain.outline);
  result.cutouts.reserve(domain.cutouts.size());
  std::transform(domain.cutouts.begin(), domain.cutouts.end(),
                 std::back_inserter(result.cutouts),
                 [](const auto& polygon) { return geometry::prepare_polygon(polygon); });
  result.obstacles.reserve(domain.obstacles.size());
  std::transform(domain.obstacles.begin(), domain.obstacles.end(),
                 std::back_inserter(result.obstacles),
                 [](const auto& polygon) { return geometry::prepare_polygon(polygon); });
  result.filled_obstacles.reserve(domain.filled_obstacles.size());
  std::transform(domain.filled_obstacles.begin(), domain.filled_obstacles.end(),
                 std::back_inserter(result.filled_obstacles), [](const auto& region) {
                   return prepare_filled_region(region.area, region.holes);
                 });
  return result;
}

[[nodiscard]] geometry::PointLocation locate_point(const PreparedFilledRegion& region,
                                                   Point point) {
  const auto outer = geometry::locate_point(region.area, point);
  if (outer != geometry::PointLocation::inside) {
    return outer;
  }
  for (const auto& hole : region.holes) {
    const auto location = geometry::locate_point(hole, point);
    if (location == geometry::PointLocation::boundary) {
      return geometry::PointLocation::boundary;
    }
    if (location == geometry::PointLocation::inside) {
      return geometry::PointLocation::outside;
    }
  }
  return geometry::PointLocation::inside;
}

[[nodiscard]] bool boundaries_intersect(const PreparedFilledRegion& region,
                                        const geometry::PreparedPolygon& polygon) {
  if (geometry::polygon_boundaries_intersect(region.area, polygon)) {
    return true;
  }
  return std::any_of(region.holes.begin(), region.holes.end(), [&](const auto& hole) {
    return geometry::polygon_boundaries_intersect(hole, polygon);
  });
}

[[nodiscard]] bool prepared_filled_region_overlaps_polygon(
    const PreparedFilledRegion& region, const geometry::PreparedPolygon& polygon) {
  if (!region.area.valid() || !polygon.valid() || !region.area.bounds.overlaps(polygon.bounds)) {
    return false;
  }
  if (boundaries_intersect(region, polygon)) {
    return true;
  }
  if (std::any_of(polygon.polygon.vertices.begin(), polygon.polygon.vertices.end(),
                  [&](Point point) {
                    return locate_point(region, point) != geometry::PointLocation::outside;
                  })) {
    return true;
  }
  return std::any_of(region.area.polygon.vertices.begin(), region.area.polygon.vertices.end(),
                     [&](Point point) {
                       return geometry::locate_point(polygon, point) !=
                              geometry::PointLocation::outside;
                     });
}

[[nodiscard]] bool prepared_filled_regions_overlap(const PreparedFilledRegion& lhs,
                                                   const PreparedFilledRegion& rhs) {
  if (!lhs.area.valid() || !rhs.area.valid() || !lhs.area.bounds.overlaps(rhs.area.bounds)) {
    return false;
  }
  if (boundaries_intersect(lhs, rhs.area) ||
      std::any_of(rhs.holes.begin(), rhs.holes.end(),
                  [&](const auto& hole) { return boundaries_intersect(lhs, hole); })) {
    return true;
  }
  if (std::any_of(lhs.area.polygon.vertices.begin(), lhs.area.polygon.vertices.end(),
                  [&](Point point) {
                    return locate_point(rhs, point) != geometry::PointLocation::outside;
                  })) {
    return true;
  }
  return std::any_of(rhs.area.polygon.vertices.begin(), rhs.area.polygon.vertices.end(),
                     [&](Point point) {
                       return locate_point(lhs, point) != geometry::PointLocation::outside;
                     });
}

[[nodiscard]] bool validate_input(const FreeSpaceDomain& domain,
                                  const FreeSpaceOffsetConfig& config,
                                  FreeSpaceOffsetResult& result) {
  if (config.clearance.value() < 0 || config.maximum_output_vertices == 0U ||
      config.clearance.value() >= std::numeric_limits<Coordinate>::max()) {
    result.diagnostic = "free-space offset clearance and output budget are invalid";
    return false;
  }
  if ((!config.obstacle_clearances.empty() &&
       config.obstacle_clearances.size() != domain.obstacles.size()) ||
      (!config.filled_obstacle_clearances.empty() &&
       config.filled_obstacle_clearances.size() != domain.filled_obstacles.size()) ||
      std::any_of(config.obstacle_clearances.begin(), config.obstacle_clearances.end(),
                  [](Nanometres value) { return value.value() < 0; }) ||
      std::any_of(config.filled_obstacle_clearances.begin(),
                  config.filled_obstacle_clearances.end(),
                  [](Nanometres value) { return value.value() < 0; })) {
    result.diagnostic = "free-space offset heterogeneous clearances are invalid or misaligned";
    return false;
  }
  if (!geometry::is_simple_polygon(domain.outline)) {
    result.diagnostic = "free-space offset outline must be a simple polygon";
    return false;
  }
  for (std::size_t index = 0U; index < domain.cutouts.size(); ++index) {
    const auto& cutout = domain.cutouts[index];
    if (!geometry::is_simple_polygon(cutout) || !geometry::polygon_inside(cutout, domain.outline) ||
        geometry::polygon_boundary_distance_nm(cutout, domain.outline) <= 0.0L ||
        std::any_of(domain.cutouts.begin(),
                    domain.cutouts.begin() + static_cast<std::ptrdiff_t>(index),
                    [&cutout](const Polygon& prior) {
                      return geometry::polygons_overlap(cutout, prior);
                    })) {
      result.diagnostic =
          "free-space offset cutouts must be simple, disjoint, and strictly inside the outline";
      return false;
    }
  }
  for (std::size_t index = 0; index < domain.obstacles.size(); ++index) {
    const auto& obstacle = domain.obstacles[index];
    if (!geometry::is_simple_polygon(obstacle)) {
      result.diagnostic = "free-space offset obstacles must be simple polygons";
      return false;
    }
  }
  for (const auto& obstacle : domain.filled_obstacles) {
    if (!geometry::is_simple_polygon(obstacle.area)) {
      result.diagnostic = "filled offset obstacles require a simple outer contour";
      return false;
    }
    for (std::size_t index = 0; index < obstacle.holes.size(); ++index) {
      const auto& hole = obstacle.holes[index];
      if (!geometry::is_simple_polygon(hole) || !geometry::polygon_inside(hole, obstacle.area) ||
          geometry::polygon_boundary_distance_nm(hole, obstacle.area) <= 0.0L ||
          std::any_of(
              obstacle.holes.begin(), obstacle.holes.begin() + static_cast<std::ptrdiff_t>(index),
              [&hole](const Polygon& prior) { return geometry::polygons_overlap(hole, prior); })) {
        result.diagnostic = "filled offset obstacle holes must be simple and disjoint";
        return false;
      }
    }
  }
  return true;
}

[[nodiscard]] BoostPolygon to_boost_polygon(const Polygon& polygon) {
  std::vector<BoostPoint> points;
  points.reserve(polygon.vertices.size());
  for (const auto point : polygon.vertices) {
    points.emplace_back(static_cast<Coordinate>(point.x.value()),
                        static_cast<Coordinate>(point.y.value()));
  }
  BoostPolygon result;
  boost::polygon::set_points(result, points.begin(), points.end());
  return result;
}

template <typename BoostPolygonLike>
[[nodiscard]] Polygon from_boost_polygon(const BoostPolygonLike& polygon) {
  Polygon result;
  for (auto point = boost::polygon::begin_points(polygon);
       point != boost::polygon::end_points(polygon); ++point) {
    result.vertices.push_back(
        Point{Nanometres{static_cast<std::int64_t>(boost::polygon::x(*point))},
              Nanometres{static_cast<std::int64_t>(boost::polygon::y(*point))}});
  }
  if (result.vertices.size() > 1U && result.vertices.front() == result.vertices.back()) {
    result.vertices.pop_back();
  }
  return result;
}

[[nodiscard]] Nanometres obstacle_clearance(const FreeSpaceOffsetConfig& config,
                                            std::size_t index, bool filled) {
  const auto& clearances =
      filled ? config.filled_obstacle_clearances : config.obstacle_clearances;
  return clearances.empty() ? config.clearance : clearances[index];
}

[[nodiscard]] std::vector<OffsetFreeSpaceRegion> expand_solid_region(
    const OffsetFreeSpaceRegion& source, Nanometres clearance) {
  using namespace boost::polygon::operators;
  BoostPolygonSet region;
  region.insert(to_boost_polygon(source.area));
  BoostPolygonSet holes;
  for (const auto& hole : source.holes) holes.insert(to_boost_polygon(hole));
  region -= holes;
  region.resize(static_cast<Coordinate>(clearance.value() + 2), false);
  std::list<BoostPolygonWithHoles> polygons;
  region.get(polygons);
  std::vector<OffsetFreeSpaceRegion> result;
  result.reserve(polygons.size());
  for (const auto& polygon : polygons) {
    OffsetFreeSpaceRegion expanded;
    expanded.area = from_boost_polygon(polygon);
    for (auto hole = boost::polygon::begin_holes(polygon);
         hole != boost::polygon::end_holes(polygon); ++hole) {
      expanded.holes.push_back(from_boost_polygon(*hole));
    }
    result.push_back(std::move(expanded));
  }
  return result;
}

void insert_solid_region(BoostPolygonSet& destination, const OffsetFreeSpaceRegion& source) {
  using namespace boost::polygon::operators;
  destination.insert(to_boost_polygon(source.area));
  BoostPolygonSet holes;
  for (const auto& hole : source.holes) holes.insert(to_boost_polygon(hole));
  destination -= holes;
}

[[nodiscard]] bool contour_has_required_clearance(const Polygon& contour,
                                                  const FreeSpaceDomain& domain,
                                                  const FreeSpaceOffsetConfig& config) {
  const auto required_outer = static_cast<long double>(config.clearance.value());
  if (required_outer > 0.0L &&
      geometry::polygon_boundary_distance_nm(contour, domain.outline) < required_outer) {
    return false;
  }
  for (const auto& cutout : domain.cutouts) {
    if (required_outer > 0.0L &&
        geometry::polygon_boundary_distance_nm(contour, cutout) < required_outer) {
      return false;
    }
  }
  for (std::size_t index = 0U; index < domain.obstacles.size(); ++index) {
    const auto required =
        static_cast<long double>(obstacle_clearance(config, index, false).value());
    if (required > 0.0L &&
        geometry::polygon_boundary_distance_nm(contour, domain.obstacles[index]) < required) {
      return false;
    }
  }
  for (std::size_t index = 0U; index < domain.filled_obstacles.size(); ++index) {
    const auto required =
        static_cast<long double>(obstacle_clearance(config, index, true).value());
    if (required <= 0.0L) continue;
    const auto& obstacle = domain.filled_obstacles[index];
    if (geometry::polygon_boundary_distance_nm(contour, obstacle.area) < required) return false;
    for (const auto& hole : obstacle.holes) {
      if (geometry::polygon_boundary_distance_nm(contour, hole) < required) return false;
    }
  }
  return true;
}

[[nodiscard]] bool validate_region(const OffsetFreeSpaceRegion& region,
                                   const FreeSpaceDomain& domain,
                                   const PreparedValidationDomain& prepared_domain,
                                   const FreeSpaceOffsetConfig& config,
                                   std::string& diagnostic) {
  if (!geometry::is_simple_polygon(region.area)) {
    diagnostic = "output outer contour is not simple";
    return false;
  }
  if (!geometry::polygon_inside(region.area, domain.outline)) {
    diagnostic = "output outer contour is outside the source outline";
    return false;
  }
  // A zero requested certificate is already implied by the exact containment/overlap checks
  // below. Computing every output-to-source segment pair in that case is pure O(n*m) work; the
  // Boost Boolean still applies its two-nanometre rounding guard before this validation.
  if (!contour_has_required_clearance(region.area, domain, config)) {
    diagnostic = "output outer contour violates source-boundary clearance";
    return false;
  }
  const auto prepared_filled = prepare_filled_region(region.area, region.holes);
  for (const auto& cutout : prepared_domain.cutouts) {
    if (prepared_filled_region_overlaps_polygon(prepared_filled, cutout)) {
      diagnostic = "output region overlaps a board cutout";
      return false;
    }
  }
  for (std::size_t index = 0U; index < prepared_domain.obstacles.size(); ++index) {
    if (prepared_filled_region_overlaps_polygon(prepared_filled,
                                                prepared_domain.obstacles[index])) {
      diagnostic = "output region overlaps polygonal obstacle " + std::to_string(index);
      return false;
    }
  }
  for (std::size_t index = 0U; index < prepared_domain.filled_obstacles.size(); ++index) {
    if (prepared_filled_regions_overlap(prepared_filled,
                                        prepared_domain.filled_obstacles[index])) {
      diagnostic = "output region overlaps filled obstacle " + std::to_string(index);
      return false;
    }
  }
  for (std::size_t index = 0; index < region.holes.size(); ++index) {
    const auto& hole = region.holes[index];
    if (!geometry::is_simple_polygon(hole)) {
      diagnostic = "output hole " + std::to_string(index) + " is not simple";
      return false;
    }
    const auto& prepared_hole = prepared_filled.holes[index];
    // Output holes must be strictly interior. For simple polygons, one interior vertex plus
    // non-intersecting boundaries proves containment; both queries use the prepared outer BVH.
    // This replaces the former O(hole_vertices * outer_vertices) containment and distance pair.
    if (geometry::locate_point(prepared_filled.area, hole.vertices.front()) !=
            geometry::PointLocation::inside ||
        geometry::polygon_boundaries_intersect(prepared_hole, prepared_filled.area)) {
      diagnostic = "output hole " + std::to_string(index) + " is not strictly inside its outer";
      return false;
    }
    if (!contour_has_required_clearance(hole, domain, config)) {
      diagnostic = "output hole " + std::to_string(index) + " violates source clearance";
      return false;
    }
    // The complete region was prepared above for obstacle validation. Reuse those immutable
    // bounds/BVHs here instead of reconstructing bounds and testing every segment pair for every
    // output-hole pair. Boost can return hundreds of obstacle-shaped holes on dense boards.
    if (std::any_of(prepared_filled.holes.begin(),
                    prepared_filled.holes.begin() + static_cast<std::ptrdiff_t>(index),
                    [&](const auto& prior) {
                      return geometry::polygons_overlap(prepared_hole, prior);
                    })) {
      diagnostic = "output hole " + std::to_string(index) + " overlaps a prior hole";
      return false;
    }
  }
  return true;
}

} // namespace

FreeSpaceOffsetResult offset_polygonal_free_space(const FreeSpaceDomain& domain,
                                                  const FreeSpaceOffsetConfig& config,
                                                  FreeSpaceOffsetCache* cache) {
  FreeSpaceOffsetResult result;
  if (!validate_input(domain, config, result)) {
    return result;
  }
  const auto prepared_domain = prepare_validation_domain(domain);

  const auto guarded_clearance = static_cast<Coordinate>(config.clearance.value() + 2);
  using namespace boost::polygon::operators;
  BoostPolygonSet free_space;
  free_space.insert(to_boost_polygon(domain.outline));
  BoostPolygonSet cutouts;
  for (const auto& cutout : domain.cutouts) {
    cutouts.insert(to_boost_polygon(cutout));
  }
  free_space -= cutouts;
  free_space.resize(-guarded_clearance, false);

  BoostPolygonSet blocked;
  const auto expanded = [&](const OffsetFreeSpaceRegion& source, Nanometres clearance) {
    if (!cache) return expand_solid_region(source, clearance);
    const auto key = expansion_key(source, clearance);
    {
      std::lock_guard lock(cache->impl_->mutex);
      if (const auto bucket = cache->impl_->entries.find(key);
          bucket != cache->impl_->entries.end()) {
        for (auto& entry : bucket->second) {
          if (entry.clearance == clearance && entry.source == source) {
            entry.age = ++cache->impl_->clock;
            ++cache->impl_->hits;
            return entry.expanded;
          }
        }
      }
    }
    auto computed = expand_solid_region(source, clearance);
    std::lock_guard lock(cache->impl_->mutex);
    if (const auto bucket = cache->impl_->entries.find(key);
        bucket != cache->impl_->entries.end()) {
      for (auto& entry : bucket->second) {
        if (entry.clearance == clearance && entry.source == source) {
          entry.age = ++cache->impl_->clock;
          ++cache->impl_->hits;
          return entry.expanded;
        }
      }
    }
    ++cache->impl_->misses;
    if (cache->impl_->entry_count == cache->impl_->capacity) {
      auto oldest_bucket = cache->impl_->entries.end();
      auto oldest = std::vector<FreeSpaceOffsetCache::Impl::Entry>::iterator{};
      for (auto bucket = cache->impl_->entries.begin(); bucket != cache->impl_->entries.end(); ++bucket) {
        for (auto entry = bucket->second.begin(); entry != bucket->second.end(); ++entry) {
          if (oldest_bucket == cache->impl_->entries.end() || entry->age < oldest->age) {
            oldest_bucket = bucket;
            oldest = entry;
          }
        }
      }
      oldest_bucket->second.erase(oldest);
      if (oldest_bucket->second.empty()) cache->impl_->entries.erase(oldest_bucket);
      --cache->impl_->entry_count;
    }
    auto& bucket = cache->impl_->entries[key];
    bucket.push_back({source, clearance, std::move(computed), ++cache->impl_->clock});
    ++cache->impl_->entry_count;
    return bucket.back().expanded;
  };
  for (std::size_t index = 0U; index < domain.obstacles.size(); ++index) {
    if (!cache) {
      BoostPolygonSet expanded_set;
      expanded_set.insert(to_boost_polygon(domain.obstacles[index]));
      expanded_set.resize(static_cast<Coordinate>(obstacle_clearance(config, index, false).value() + 2), false);
      blocked += expanded_set;
      continue;
    }
    const OffsetFreeSpaceRegion source{domain.obstacles[index], {}};
    BoostPolygonSet expanded_set;
    for (const auto& obstacle : expanded(source, obstacle_clearance(config, index, false))) {
      insert_solid_region(expanded_set, obstacle);
    }
    blocked += expanded_set;
  }
  for (std::size_t index = 0U; index < domain.filled_obstacles.size(); ++index) {
    const auto& obstacle = domain.filled_obstacles[index];
    if (!cache) {
      BoostPolygonSet region;
      region.insert(to_boost_polygon(obstacle.area));
      BoostPolygonSet holes;
      for (const auto& hole : obstacle.holes) holes.insert(to_boost_polygon(hole));
      region -= holes;
      region.resize(static_cast<Coordinate>(obstacle_clearance(config, index, true).value() + 2), false);
      blocked += region;
      continue;
    }
    const OffsetFreeSpaceRegion source{obstacle.area, obstacle.holes};
    BoostPolygonSet expanded_set;
    for (const auto& expanded_obstacle :
         expanded(source, obstacle_clearance(config, index, true))) {
      insert_solid_region(expanded_set, expanded_obstacle);
    }
    blocked += expanded_set;
  }
  if (!domain.obstacles.empty() || !domain.filled_obstacles.empty()) {
    free_space -= blocked;
  }

  std::list<BoostPolygonWithHoles> polygons;
  free_space.get(polygons);
  for (const auto& polygon : polygons) {
    OffsetFreeSpaceRegion region;
    region.area = from_boost_polygon(polygon);
    for (auto hole = boost::polygon::begin_holes(polygon);
         hole != boost::polygon::end_holes(polygon); ++hole) {
      region.holes.push_back(from_boost_polygon(*hole));
    }
    auto vertices = region.area.vertices.size();
    for (const auto& hole : region.holes) {
      if (hole.vertices.size() > config.maximum_output_vertices - vertices) {
        result.diagnostic = "free-space offset exceeds its output-vertex budget";
        result.regions.clear();
        return result;
      }
      vertices += hole.vertices.size();
    }
    if (vertices > config.maximum_output_vertices - result.output_vertices) {
      result.diagnostic = "free-space offset exceeds its output-vertex budget";
      result.regions.clear();
      return result;
    }
    std::string validation_diagnostic;
    if (!validate_region(region, domain, prepared_domain, config,
                         validation_diagnostic)) {
      result.diagnostic =
          "free-space offset output failed exact conservative validation: " + validation_diagnostic;
      result.regions.clear();
      return result;
    }
    result.output_vertices += vertices;
    result.regions.push_back(std::move(region));
  }
  std::sort(result.regions.begin(), result.regions.end(),
            [](const auto& lhs, const auto& rhs) { return lhs.area.vertices < rhs.area.vertices; });
  return result;
}

} // namespace flux
