#include "flux/core/free_space_offset.hpp"

#include "flux/core/geometry.hpp"
#include "free_space_boolean_cache.hpp"

#include <boost/polygon/polygon.hpp>
#include <boost/polygon/polygon_set_concept.hpp>
#include <boost/polygon/polygon_set_data.hpp>

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <deque>
#include <limits>
#include <list>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

namespace flux {
namespace {
struct ValidationEntry;
using PreparedPtr = std::shared_ptr<const geometry::PreparedPolygon>;
using Coordinate = std::int32_t;
using BoostPoint = boost::polygon::point_data<Coordinate>;
using BoostPolygon = boost::polygon::polygon_data<Coordinate>;
using BoostPolygonWithHoles = boost::polygon::polygon_with_holes_data<Coordinate>;
using BoostPolygonSet = boost::polygon::polygon_set_data<Coordinate>;
struct BoundaryWitness {
  PreparedPtr source;
  Nanometres clearance;
};
struct ClearanceCertificate {
  Polygon contour;
  std::vector<BoundaryWitness> witnesses;
  std::size_t vertices{0U};
};
}
struct FreeSpaceOffsetCache::Impl {
  struct Entry {
    OffsetFreeSpaceRegion source;
    Nanometres clearance;
    // Boost's const operations can clean mutable internal state. Cached sets are normalized
    // before publication and callers always make a local copy before using one in a Boolean.
    std::shared_ptr<const BoostPolygonSet> expanded;
    std::size_t age{0U};
  };
  struct OutlineEntry {
    OffsetFreeSpaceRegion source;
    Nanometres clearance;
    std::shared_ptr<const BoostPolygonSet> eroded;
    std::size_t age{0U};
  };
  explicit Impl(std::size_t requested_capacity)
      : capacity(std::max<std::size_t>(1U, requested_capacity)), unions(capacity) {}
  std::size_t capacity;
  detail::FreeSpaceBooleanUnionCache unions;
  std::size_t expanded_units{0U};
  std::size_t outline_units{0U};
  std::size_t clock{0U};
  std::size_t hits{0U};
  std::size_t misses{0U};
  std::unordered_map<std::uint64_t, std::vector<Entry>> entries;
  std::size_t entry_count{0U};
  std::size_t outline_hits{0U};
  std::size_t outline_misses{0U};
  std::unordered_map<std::uint64_t, std::vector<OutlineEntry>> outline_entries;
  std::size_t outline_entry_count{0U};
  std::vector<std::shared_ptr<ValidationEntry>> validation_entries;
  std::unordered_map<std::uint64_t, std::vector<PreparedPtr>> sources;
  std::deque<std::pair<std::uint64_t, PreparedPtr>> source_order;
  std::size_t source_vertices{0U};
  std::unordered_map<std::uint64_t, std::vector<std::shared_ptr<const ClearanceCertificate>>> certificates;
  std::deque<std::pair<std::uint64_t, std::shared_ptr<const ClearanceCertificate>>> certificate_order;
  std::size_t certificate_vertices{0U};
  std::size_t prepared_source_hits{0U};
  std::size_t prepared_source_misses{0U};
  std::size_t input_validation_hits{0U};
  std::size_t input_validation_misses{0U};
  std::size_t prepared_domain_hits{0U};
  std::size_t prepared_domain_misses{0U};
  std::size_t clearance_certificate_hits{0U};
  std::size_t clearance_certificate_misses{0U};
  std::size_t retained_vertices{0U};
  double input_validation_seconds{0.0};
  double preparation_seconds{0.0};
  double boolean_operations_seconds{0.0};
  double boolean_conversion_seconds{0.0};
  double boolean_offset_preparation_seconds{0.0};
  double boolean_union_seconds{0.0};
  double boolean_subtraction_seconds{0.0};
  double boolean_contour_extraction_seconds{0.0};
  double output_validation_seconds{0.0};
  mutable std::mutex mutex;
};

FreeSpaceOffsetCache::FreeSpaceOffsetCache(std::size_t capacity)
    : impl_(std::make_unique<Impl>(capacity)) {}
FreeSpaceOffsetCache::~FreeSpaceOffsetCache() = default;
FreeSpaceOffsetCache::FreeSpaceOffsetCache(FreeSpaceOffsetCache&&) noexcept = default;
FreeSpaceOffsetCache& FreeSpaceOffsetCache::operator=(FreeSpaceOffsetCache&&) noexcept = default;
FreeSpaceOffsetCache::Stats FreeSpaceOffsetCache::stats() const {
  std::lock_guard lock(impl_->mutex);
  const auto unions = impl_->unions.stats();
  return {impl_->hits, impl_->misses, impl_->entry_count,
          impl_->outline_hits, impl_->outline_misses,
          unions.hits, unions.misses,
          impl_->input_validation_hits, impl_->input_validation_misses,
          impl_->prepared_domain_hits, impl_->prepared_domain_misses,
          impl_->clearance_certificate_hits, impl_->clearance_certificate_misses,
          impl_->validation_entries.size(),
          impl_->retained_vertices + impl_->source_vertices + impl_->certificate_vertices,
          impl_->input_validation_seconds, impl_->preparation_seconds,
          impl_->boolean_operations_seconds, impl_->boolean_conversion_seconds,
          impl_->boolean_offset_preparation_seconds, impl_->boolean_union_seconds,
          impl_->boolean_subtraction_seconds, impl_->boolean_contour_extraction_seconds,
          impl_->output_validation_seconds,
          impl_->prepared_source_hits, impl_->prepared_source_misses,
          impl_->expanded_units + impl_->outline_units + unions.retained_units};
}
void FreeSpaceOffsetCache::clear() {
  std::lock_guard lock(impl_->mutex);
  impl_->entries.clear();
  impl_->outline_entries.clear();
  impl_->validation_entries.clear();
  impl_->sources.clear();
  impl_->source_order.clear();
  impl_->certificates.clear();
  impl_->certificate_order.clear();
  impl_->certificate_vertices = 0U;
  impl_->source_vertices = impl_->prepared_source_hits = impl_->prepared_source_misses = 0U;
  impl_->hits = impl_->misses = impl_->clock = impl_->entry_count = 0U;
  impl_->outline_hits = impl_->outline_misses = impl_->outline_entry_count = 0U;
  impl_->unions.clear();
  impl_->expanded_units = impl_->outline_units = 0U;
  impl_->input_validation_hits = impl_->input_validation_misses = 0U;
  impl_->prepared_domain_hits = impl_->prepared_domain_misses = 0U;
  impl_->clearance_certificate_hits = impl_->clearance_certificate_misses = 0U;
  impl_->retained_vertices = 0U;
  impl_->input_validation_seconds = impl_->preparation_seconds = 0.0;
  impl_->boolean_operations_seconds = impl_->output_validation_seconds = 0.0;
  impl_->boolean_conversion_seconds = impl_->boolean_offset_preparation_seconds = 0.0;
  impl_->boolean_union_seconds = impl_->boolean_subtraction_seconds = 0.0;
  impl_->boolean_contour_extraction_seconds = 0.0;
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

struct PreparedFilledRegion {
  PreparedPtr area;
  std::vector<PreparedPtr> holes;
};

struct PreparedValidationDomain {
  PreparedPtr outline;
  std::vector<PreparedPtr> cutouts;
  std::vector<PreparedPtr> obstacles;
  std::vector<PreparedFilledRegion> filled_obstacles;
};

[[nodiscard]] bool domain_equal(const FreeSpaceDomain& lhs, const FreeSpaceDomain& rhs) {
  return lhs.outline == rhs.outline && lhs.cutouts == rhs.cutouts &&
         lhs.obstacles == rhs.obstacles && lhs.filled_obstacles == rhs.filled_obstacles;
}

[[nodiscard]] std::size_t domain_vertices(const FreeSpaceDomain& domain) {
  std::size_t result = domain.outline.vertices.size();
  const auto add = [&result](const Polygon& polygon) { result += polygon.vertices.size(); };
  for (const auto& polygon : domain.cutouts) add(polygon);
  for (const auto& polygon : domain.obstacles) add(polygon);
  for (const auto& region : domain.filled_obstacles) {
    add(region.area);
    for (const auto& hole : region.holes) add(hole);
  }
  return result;
}

struct ValidationEntry {
  std::uint64_t signature{0U};
  FreeSpaceDomain domain;
  PreparedValidationDomain prepared;
  std::size_t vertices{0U};
  std::size_t age{0U};
};

[[nodiscard]] PreparedFilledRegion prepare_filled_region(const Polygon& area,
                                                         const std::vector<Polygon>& holes) {
  PreparedFilledRegion result;
  result.area = std::make_shared<const geometry::PreparedPolygon>(geometry::prepare_polygon(area));
  result.holes.reserve(holes.size());
  std::transform(holes.begin(), holes.end(), std::back_inserter(result.holes),
                 [](const auto& hole) { return std::make_shared<const geometry::PreparedPolygon>(geometry::prepare_polygon(hole)); });
  return result;
}

[[nodiscard]] PreparedValidationDomain prepare_validation_domain(
    const FreeSpaceDomain& domain, const std::function<PreparedPtr(const Polygon&)>& prepare) {
  PreparedValidationDomain result;
  result.outline = prepare(domain.outline);
  for (const auto& polygon : domain.cutouts) result.cutouts.push_back(prepare(polygon));
  for (const auto& polygon : domain.obstacles) result.obstacles.push_back(prepare(polygon));
  for (const auto& region : domain.filled_obstacles) {
    PreparedFilledRegion filled;
    filled.area = prepare(region.area);
    for (const auto& hole : region.holes) filled.holes.push_back(prepare(hole));
    result.filled_obstacles.push_back(std::move(filled));
  }
  return result;
}

[[nodiscard]] geometry::PointLocation locate_point(const PreparedFilledRegion& region,
                                                   Point point) {
  const auto outer = geometry::locate_point(*region.area, point);
  if (outer != geometry::PointLocation::inside) {
    return outer;
  }
  for (const auto& hole : region.holes) {
    const auto location = geometry::locate_point(*hole, point);
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
  if (geometry::polygon_boundaries_intersect(*region.area, polygon)) {
    return true;
  }
  return std::any_of(region.holes.begin(), region.holes.end(), [&](const auto& hole) {
    return geometry::polygon_boundaries_intersect(*hole, polygon);
  });
}

[[nodiscard]] bool prepared_filled_region_overlaps_polygon(
    const PreparedFilledRegion& region, const geometry::PreparedPolygon& polygon) {
  if (!region.area->valid() || !polygon.valid() || !region.area->bounds.overlaps(polygon.bounds)) {
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
  return std::any_of(region.area->polygon.vertices.begin(), region.area->polygon.vertices.end(),
                     [&](Point point) {
                       return geometry::locate_point(polygon, point) !=
                              geometry::PointLocation::outside;
                     });
}

[[nodiscard]] bool prepared_filled_regions_overlap(const PreparedFilledRegion& lhs,
                                                   const PreparedFilledRegion& rhs) {
  if (!lhs.area->valid() || !rhs.area->valid() || !lhs.area->bounds.overlaps(rhs.area->bounds)) {
    return false;
  }
  if (boundaries_intersect(lhs, *rhs.area) ||
      std::any_of(rhs.holes.begin(), rhs.holes.end(),
                  [&](const auto& hole) { return boundaries_intersect(lhs, *hole); })) {
    return true;
  }
  if (std::any_of(lhs.area->polygon.vertices.begin(), lhs.area->polygon.vertices.end(),
                  [&](Point point) {
                    return locate_point(rhs, point) != geometry::PointLocation::outside;
                  })) {
    return true;
  }
  return std::any_of(rhs.area->polygon.vertices.begin(), rhs.area->polygon.vertices.end(),
                     [&](Point point) {
                       return locate_point(lhs, point) != geometry::PointLocation::outside;
                     });
}

[[nodiscard]] bool valid_offset_clearance(Nanometres clearance) {
  // Reserve the two-nanometre conservative guard before conversion and erosion negation.
  return clearance.value() >= 0 &&
         clearance.value() <= static_cast<std::int64_t>(std::numeric_limits<Coordinate>::max()) - 2;
}

[[nodiscard]] bool validate_input(const FreeSpaceDomain& domain,
                                  const FreeSpaceOffsetConfig& config,
                                  FreeSpaceOffsetResult& result,
                                  const std::function<bool(const Polygon&)>& simple) {
  if (!valid_offset_clearance(config.clearance) || config.maximum_output_vertices == 0U) {
    result.diagnostic = "free-space offset clearance and output budget are invalid";
    return false;
  }
  if (
      (!config.obstacle_clearances.empty() &&
       config.obstacle_clearances.size() != domain.obstacles.size()) ||
      (!config.filled_obstacle_clearances.empty() &&
       config.filled_obstacle_clearances.size() != domain.filled_obstacles.size()) ||
      std::any_of(config.obstacle_clearances.begin(), config.obstacle_clearances.end(),
                  [](Nanometres value) { return !valid_offset_clearance(value); }) ||
      std::any_of(config.filled_obstacle_clearances.begin(),
                  config.filled_obstacle_clearances.end(),
                  [](Nanometres value) { return !valid_offset_clearance(value); })) {
    result.diagnostic = "free-space offset heterogeneous clearances are invalid or misaligned";
    return false;
  }
  if (!simple(domain.outline)) {
    result.diagnostic = "free-space offset outline must be a simple polygon";
    return false;
  }
  for (std::size_t index = 0U; index < domain.cutouts.size(); ++index) {
    const auto& cutout = domain.cutouts[index];
    if (!simple(cutout) || !geometry::polygon_inside(cutout, domain.outline) ||
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
    if (!simple(obstacle)) {
      result.diagnostic = "free-space offset obstacles must be simple polygons";
      return false;
    }
  }
  for (const auto& obstacle : domain.filled_obstacles) {
    if (!simple(obstacle.area)) {
      result.diagnostic = "filled offset obstacles require a simple outer contour";
      return false;
    }
    for (std::size_t index = 0; index < obstacle.holes.size(); ++index) {
      const auto& hole = obstacle.holes[index];
      if (!simple(hole) || !geometry::polygon_inside(hole, obstacle.area) ||
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

[[nodiscard]] std::size_t boolean_storage_units(const OffsetFreeSpaceRegion& source,
                                                   const BoostPolygonSet& set) {
  auto units = source.area.vertices.size() + set.value().size();
  for (const auto& hole : source.holes) units += hole.vertices.size();
  return units;
}

[[nodiscard]] std::uint64_t spatial_key(const Polygon& polygon) {
  auto low = polygon.vertices.front();
  auto high = low;
  for (const auto point : polygon.vertices) {
    low.x = std::min(low.x, point.x); low.y = std::min(low.y, point.y);
    high.x = std::max(high.x, point.x); high.y = std::max(high.y, point.y);
  }
  const auto x = static_cast<std::uint32_t>(low.x.value() + (high.x.value() - low.x.value()) / 2) ^ 0x80000000U;
  const auto y = static_cast<std::uint32_t>(low.y.value() + (high.y.value() - low.y.value()) / 2) ^ 0x80000000U;
  std::uint64_t key = 0U;
  for (unsigned bit = 0; bit < 32U; ++bit) {
    key |= static_cast<std::uint64_t>((x >> bit) & 1U) << (2U * bit);
    key |= static_cast<std::uint64_t>((y >> bit) & 1U) << (2U * bit + 1U);
  }
  return key;
}

[[nodiscard]] Nanometres obstacle_clearance(const FreeSpaceOffsetConfig& config,
                                            std::size_t index, bool filled) {
  const auto& clearances =
      filled ? config.filled_obstacle_clearances : config.obstacle_clearances;
  return clearances.empty() ? config.clearance : clearances[index];
}

[[nodiscard]] BoostPolygonSet solid_region_set(const OffsetFreeSpaceRegion& source) {
  using namespace boost::polygon::operators;
  BoostPolygonSet region;
  region.insert(to_boost_polygon(source.area));
  BoostPolygonSet holes;
  for (const auto& hole : source.holes) holes.insert(to_boost_polygon(hole));
  region -= holes;
  return region;
}

[[nodiscard]] BoostPolygonSet normalized_boolean_set(BoostPolygonSet set) {
  // Force Boost to resolve lazy state before publication. Later consumers only operate on a
  // local copy because even nominally const Boost access may update internal representation.
  set.clean();
  return set;
}

// Recompute this neighbourhood on every query. Added/moved boundaries and changed effective
// clearances therefore invalidate certificates even when the output contour itself is unchanged.
[[nodiscard]] std::vector<BoundaryWitness> clearance_neighbours(
    const Polygon& contour, const PreparedValidationDomain& domain,
    const FreeSpaceOffsetConfig& config) {
  geometry::AxisAlignedBounds bounds{contour.vertices.front(), contour.vertices.front()};
  for (const auto point : contour.vertices) {
    bounds.minimum.x = std::min(bounds.minimum.x, point.x);
    bounds.minimum.y = std::min(bounds.minimum.y, point.y);
    bounds.maximum.x = std::max(bounds.maximum.x, point.x);
    bounds.maximum.y = std::max(bounds.maximum.y, point.y);
  }
  std::vector<BoundaryWitness> result;
  const auto add = [&](const PreparedPtr& source, Nanometres clearance) {
    if (clearance.value() <= 0) return;
    const auto gap = [](Nanometres low, Nanometres high) {
      return static_cast<long double>(low.value()) - static_cast<long double>(high.value());
    };
    const auto required = static_cast<long double>(clearance.value());
    // An axis gap is a conservative lower bound on Euclidean boundary distance. Equality is
    // safe: only a strictly smaller clearance is forbidden. Integer coordinates are not expanded.
    if (gap(bounds.minimum.x, source->bounds.maximum.x) >= required ||
        gap(source->bounds.minimum.x, bounds.maximum.x) >= required ||
        gap(bounds.minimum.y, source->bounds.maximum.y) >= required ||
        gap(source->bounds.minimum.y, bounds.maximum.y) >= required) return;
    result.push_back({source, clearance});
  };
  add(domain.outline, config.clearance);
  for (const auto& cutout : domain.cutouts) add(cutout, config.clearance);
  for (std::size_t i = 0; i < domain.obstacles.size(); ++i)
    add(domain.obstacles[i], obstacle_clearance(config, i, false));
  for (std::size_t i = 0; i < domain.filled_obstacles.size(); ++i) {
    const auto clearance = obstacle_clearance(config, i, true);
    add(domain.filled_obstacles[i].area, clearance);
    for (const auto& hole : domain.filled_obstacles[i].holes) add(hole, clearance);
  }
  return result;
}

[[nodiscard]] bool same_witnesses(const std::vector<BoundaryWitness>& lhs,
                                  const std::vector<BoundaryWitness>& rhs) {
  if (lhs.size() != rhs.size()) return false;
  for (std::size_t i = 0; i < lhs.size(); ++i) {
    if (lhs[i].clearance != rhs[i].clearance ||
        (lhs[i].source != rhs[i].source && lhs[i].source->polygon != rhs[i].source->polygon))
      return false;
  }
  return true;
}

[[nodiscard]] bool contour_has_required_clearance(
    const Polygon& contour, const std::vector<BoundaryWitness>& neighbours) {
  for (const auto& neighbour : neighbours) {
    const auto required = static_cast<long double>(neighbour.clearance.value());
    for (std::size_t index = 0U; index < contour.vertices.size(); ++index) {
      if (geometry::segment_polygon_boundary_distance_nm_capped(
              contour.vertices[index], contour.vertices[(index + 1U) % contour.vertices.size()],
              *neighbour.source, required) < required) return false;
    }
  }
  return true;
}

[[nodiscard]] bool validate_region(const OffsetFreeSpaceRegion& region,
                                   const FreeSpaceDomain& domain,
                                   const PreparedValidationDomain& prepared_domain,
                                   const std::function<bool(const Polygon&)>& clearance_check,
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
  if (!clearance_check(region.area)) {
    diagnostic = "output outer contour violates source-boundary clearance";
    return false;
  }
  const auto prepared_filled = prepare_filled_region(region.area, region.holes);
  for (const auto& cutout : prepared_domain.cutouts) {
    if (prepared_filled_region_overlaps_polygon(prepared_filled, *cutout)) {
      diagnostic = "output region overlaps a board cutout";
      return false;
    }
  }
  for (std::size_t index = 0U; index < prepared_domain.obstacles.size(); ++index) {
    if (prepared_filled_region_overlaps_polygon(prepared_filled,
                                                *prepared_domain.obstacles[index])) {
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
    if (geometry::locate_point(*prepared_filled.area, hole.vertices.front()) !=
            geometry::PointLocation::inside ||
        geometry::polygon_boundaries_intersect(*prepared_hole, *prepared_filled.area)) {
      diagnostic = "output hole " + std::to_string(index) + " is not strictly inside its outer";
      return false;
    }
    if (!clearance_check(hole)) {
      diagnostic = "output hole " + std::to_string(index) + " violates source clearance";
      return false;
    }
    // The complete region was prepared above for obstacle validation. Reuse those immutable
    // bounds/BVHs here instead of reconstructing bounds and testing every segment pair for every
    // output-hole pair. Boost can return hundreds of obstacle-shaped holes on dense boards.
    if (std::any_of(prepared_filled.holes.begin(),
                    prepared_filled.holes.begin() + static_cast<std::ptrdiff_t>(index),
                    [&](const auto& prior) {
                      return geometry::polygons_overlap(*prepared_hole, *prior);
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
  const auto input_started = std::chrono::steady_clock::now();
  const auto prepare_source = [&](const Polygon& polygon) -> PreparedPtr {
    if (!cache) {
      return std::make_shared<const geometry::PreparedPolygon>(geometry::prepare_polygon(polygon));
    }
    const auto key = expansion_key({polygon, {}}, Nanometres{0});
    const auto lookup = [&]() -> PreparedPtr {
      const auto bucket = cache->impl_->sources.find(key);
      if (bucket != cache->impl_->sources.end()) {
        for (const auto& entry : bucket->second) {
          if (entry->polygon == polygon) return entry;
        }
      }
      return {};
    };
    {
      std::lock_guard lock(cache->impl_->mutex);
      if (auto found = lookup()) { ++cache->impl_->prepared_source_hits; return found; }
    }
    auto prepared = std::make_shared<const geometry::PreparedPolygon>(geometry::prepare_polygon(polygon));
    std::lock_guard lock(cache->impl_->mutex);
    if (auto found = lookup()) { ++cache->impl_->prepared_source_hits; return found; }
    ++cache->impl_->prepared_source_misses;
    constexpr std::size_t source_vertex_limit = 262'144U;
    if (polygon.vertices.size() > source_vertex_limit) return prepared;
    while (!cache->impl_->source_order.empty() &&
           (cache->impl_->source_order.size() >= cache->impl_->capacity ||
            cache->impl_->source_vertices + polygon.vertices.size() > source_vertex_limit)) {
      const auto& oldest = cache->impl_->source_order.front();
      auto& bucket = cache->impl_->sources.at(oldest.first);
      std::erase(bucket, oldest.second);
      if (bucket.empty()) cache->impl_->sources.erase(oldest.first);
      cache->impl_->source_vertices -= oldest.second->polygon.vertices.size();
      cache->impl_->source_order.pop_front();
    }
    cache->impl_->sources[key].push_back(prepared);
    cache->impl_->source_order.emplace_back(key, prepared);
    cache->impl_->source_vertices += polygon.vertices.size();
    return prepared;
  };
  const auto simple_source = [&](const Polygon& polygon) {
    if (cache) {
      const auto key = expansion_key({polygon, {}}, Nanometres{0});
      std::lock_guard lock(cache->impl_->mutex);
      if (const auto bucket = cache->impl_->sources.find(key); bucket != cache->impl_->sources.end()) {
        for (const auto& source : bucket->second) if (source->polygon == polygon) return true;
      }
    }
    return geometry::is_simple_polygon(polygon);
  };
  std::uint64_t domain_signature = expansion_key({domain.outline, domain.cutouts}, Nanometres{0});
  const auto hash_source = [&](const OffsetFreeSpaceRegion& source) {
    domain_signature ^= expansion_key(source, Nanometres{0});
    domain_signature *= 1099511628211ULL;
  };
  for (const auto& obstacle : domain.obstacles) hash_source({obstacle, {}});
  for (const auto& obstacle : domain.filled_obstacles) hash_source({obstacle.area, obstacle.holes});
  std::shared_ptr<ValidationEntry> validation_entry;
  const auto config_compatible =
      valid_offset_clearance(config.clearance) && config.maximum_output_vertices != 0U &&
      (config.obstacle_clearances.empty() ||
       config.obstacle_clearances.size() == domain.obstacles.size()) &&
      (config.filled_obstacle_clearances.empty() ||
       config.filled_obstacle_clearances.size() == domain.filled_obstacles.size()) &&
      std::none_of(config.obstacle_clearances.begin(), config.obstacle_clearances.end(),
                   [](Nanometres value) { return !valid_offset_clearance(value); }) &&
      std::none_of(config.filled_obstacle_clearances.begin(),
                   config.filled_obstacle_clearances.end(),
                   [](Nanometres value) { return !valid_offset_clearance(value); });
  if (cache && config_compatible) {
    std::lock_guard lock(cache->impl_->mutex);
    for (const auto& entry : cache->impl_->validation_entries) {
      if (entry->signature == domain_signature && domain_equal(entry->domain, domain)) {
        entry->age = ++cache->impl_->clock;
        validation_entry = entry;
        ++cache->impl_->input_validation_hits;
        ++cache->impl_->prepared_domain_hits;
        break;
      }
    }
    if (!validation_entry) {
      ++cache->impl_->input_validation_misses;
      ++cache->impl_->prepared_domain_misses;
    }
  }
  if (!validation_entry && !validate_input(domain, config, result, simple_source)) {
    if (cache) {
      std::lock_guard lock(cache->impl_->mutex);
      cache->impl_->input_validation_seconds +=
          std::chrono::duration<double>(std::chrono::steady_clock::now() - input_started).count();
    }
    return result;
  }
  if (cache) {
    std::lock_guard lock(cache->impl_->mutex);
    cache->impl_->input_validation_seconds +=
        std::chrono::duration<double>(std::chrono::steady_clock::now() - input_started).count();
  }
  const auto preparation_started = std::chrono::steady_clock::now();
  if (!validation_entry) {
    auto candidate = std::make_shared<ValidationEntry>();
    candidate->signature = domain_signature;
    candidate->domain = domain;
    candidate->prepared = prepare_validation_domain(domain, [&](const Polygon& polygon) {
      return cache ? prepare_source(polygon) :
          std::make_shared<const geometry::PreparedPolygon>(geometry::prepare_polygon(polygon));
    });
    candidate->vertices = domain_vertices(domain);
    if (cache && candidate->vertices <= 262'144U) {
      std::lock_guard lock(cache->impl_->mutex);
      // A concurrent caller may have prepared the same exact input while this one was building.
      for (const auto& entry : cache->impl_->validation_entries) {
        if (entry->signature == domain_signature && domain_equal(entry->domain, domain)) {
          validation_entry = entry;
          ++cache->impl_->prepared_domain_hits;
          break;
        }
      }
      if (!validation_entry) {
        constexpr std::size_t maximum_retained_vertices = 262'144U;
        while (!cache->impl_->validation_entries.empty() &&
               (cache->impl_->validation_entries.size() >= cache->impl_->capacity ||
                cache->impl_->retained_vertices + candidate->vertices > maximum_retained_vertices)) {
          cache->impl_->retained_vertices -= cache->impl_->validation_entries.front()->vertices;
          cache->impl_->validation_entries.erase(cache->impl_->validation_entries.begin());
        }
        candidate->age = ++cache->impl_->clock;
        cache->impl_->retained_vertices += candidate->vertices;
        cache->impl_->validation_entries.push_back(candidate);
        validation_entry = std::move(candidate);
      }
    } else {
      validation_entry = std::move(candidate);
    }
  }
  if (cache) {
    std::lock_guard lock(cache->impl_->mutex);
    cache->impl_->preparation_seconds +=
        std::chrono::duration<double>(std::chrono::steady_clock::now() - preparation_started).count();
  }
  const auto& prepared_domain = validation_entry->prepared;

  const auto boolean_started = std::chrono::steady_clock::now();
  using namespace boost::polygon::operators;
  double conversion_seconds = 0.0;
  double offset_preparation_seconds = 0.0;
  double union_seconds = 0.0;
  double subtraction_seconds = 0.0;
  double contour_extraction_seconds = 0.0;
  const auto build_expanded = [&](const OffsetFreeSpaceRegion& source, Nanometres clearance,
                                  bool normalize) {
    auto started = std::chrono::steady_clock::now();
    auto set = solid_region_set(source);
    conversion_seconds += std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count();
    started = std::chrono::steady_clock::now();
    set.resize(static_cast<Coordinate>(clearance.value() + 2), false);
    if (normalize) set = normalized_boolean_set(std::move(set));
    offset_preparation_seconds +=
        std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count();
    return set;
  };
  const auto build_eroded_outline = [&](const OffsetFreeSpaceRegion& source, Nanometres clearance,
                                        bool normalize) {
    auto started = std::chrono::steady_clock::now();
    auto set = solid_region_set(source);
    conversion_seconds += std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count();
    started = std::chrono::steady_clock::now();
    set.resize(-static_cast<Coordinate>(clearance.value() + 2), false);
    if (normalize) set = normalized_boolean_set(std::move(set));
    offset_preparation_seconds +=
        std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count();
    return set;
  };
  const auto cached_eroded_outline = [&](const OffsetFreeSpaceRegion& source,
                                          Nanometres clearance) -> BoostPolygonSet {
    if (!cache) return build_eroded_outline(source, clearance, false);
    const auto key = expansion_key(source, clearance);
    const auto lookup = [&]() -> std::shared_ptr<const BoostPolygonSet> {
      const auto bucket = cache->impl_->outline_entries.find(key);
      if (bucket == cache->impl_->outline_entries.end()) return {};
      for (auto& entry : bucket->second) {
        if (entry.clearance == clearance && entry.source == source) {
          entry.age = ++cache->impl_->clock;
          return entry.eroded;
        }
      }
      return {};
    };
    {
      std::lock_guard lock(cache->impl_->mutex);
      if (auto found = lookup()) { ++cache->impl_->outline_hits; return *found; }
    }
    auto computed = std::make_shared<const BoostPolygonSet>(
        build_eroded_outline(source, clearance, true));
    std::lock_guard lock(cache->impl_->mutex);
    if (auto found = lookup()) { ++cache->impl_->outline_hits; return *found; }
    ++cache->impl_->outline_misses;
    const auto units = boolean_storage_units(source, *computed);
    constexpr std::size_t maximum_boolean_units = 1'048'576U;
    if (units > maximum_boolean_units) return *computed;
    while (cache->impl_->outline_entry_count >= cache->impl_->capacity ||
           cache->impl_->outline_units + units > maximum_boolean_units) {
      auto oldest_bucket = cache->impl_->outline_entries.end();
      auto oldest = std::vector<FreeSpaceOffsetCache::Impl::OutlineEntry>::iterator{};
      for (auto bucket = cache->impl_->outline_entries.begin();
           bucket != cache->impl_->outline_entries.end(); ++bucket) {
        for (auto entry = bucket->second.begin(); entry != bucket->second.end(); ++entry) {
          if (oldest_bucket == cache->impl_->outline_entries.end() || entry->age < oldest->age) {
            oldest_bucket = bucket;
            oldest = entry;
          }
        }
      }
      cache->impl_->outline_units -= boolean_storage_units(oldest->source, *oldest->eroded);
      oldest_bucket->second.erase(oldest);
      if (oldest_bucket->second.empty()) cache->impl_->outline_entries.erase(oldest_bucket);
      --cache->impl_->outline_entry_count;
    }
    cache->impl_->outline_entries[key].push_back(
        {source, clearance, std::move(computed), ++cache->impl_->clock});
    ++cache->impl_->outline_entry_count;
    cache->impl_->outline_units += units;
    return *cache->impl_->outline_entries[key].back().eroded;
  };
  const OffsetFreeSpaceRegion outline_source{domain.outline, domain.cutouts};
  BoostPolygonSet free_space = cached_eroded_outline(outline_source, config.clearance);

  const auto expanded = [&](const OffsetFreeSpaceRegion& source, Nanometres clearance)
      -> std::shared_ptr<const BoostPolygonSet> {
    if (!cache) return std::make_shared<const BoostPolygonSet>(build_expanded(source, clearance, true));
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
    auto computed = std::make_shared<const BoostPolygonSet>(build_expanded(source, clearance, true));
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
    const auto units = boolean_storage_units(source, *computed);
    constexpr std::size_t maximum_boolean_units = 1'048'576U;
    if (units > maximum_boolean_units) return computed;
    while (cache->impl_->entry_count >= cache->impl_->capacity ||
           cache->impl_->expanded_units + units > maximum_boolean_units) {
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
      cache->impl_->expanded_units -= boolean_storage_units(oldest->source, *oldest->expanded);
      oldest_bucket->second.erase(oldest);
      if (oldest_bucket->second.empty()) cache->impl_->entries.erase(oldest_bucket);
      --cache->impl_->entry_count;
    }
    auto& bucket = cache->impl_->entries[key];
    bucket.push_back({source, clearance, std::move(computed), ++cache->impl_->clock});
    ++cache->impl_->entry_count;
    cache->impl_->expanded_units += units;
    return bucket.back().expanded;
  };
  std::vector<detail::FreeSpaceBooleanUnionCache::Input> inputs;
  inputs.reserve(domain.obstacles.size() + domain.filled_obstacles.size());
  const auto add_obstacle = [&](const OffsetFreeSpaceRegion& source, Nanometres clearance) {
    auto shape = expanded(source, clearance);
    inputs.push_back({std::move(shape), spatial_key(source.area), expansion_key(source, clearance)});
  };
  for (std::size_t index = 0U; index < domain.obstacles.size(); ++index)
    add_obstacle({domain.obstacles[index], {}}, obstacle_clearance(config, index, false));
  for (std::size_t index = 0U; index < domain.filled_obstacles.size(); ++index) {
    const auto& source = domain.filled_obstacles[index];
    add_obstacle({source.area, source.holes}, obstacle_clearance(config, index, true));
  }
  // Inputs already apply net exclusions and actual rules. Never subtract excluded copper from
  // a union: doing so would also erase intersecting foreign-net obstacles.
  detail::FreeSpaceBooleanUnionCache uncached_unions{0U};
  auto& unions = cache ? cache->impl_->unions : uncached_unions;
  const auto union_started = std::chrono::steady_clock::now();
  auto united = unions.unite(std::move(inputs));
  union_seconds += std::chrono::duration<double>(std::chrono::steady_clock::now() - union_started).count();
  if (!domain.obstacles.empty() || !domain.filled_obstacles.empty()) {
    const auto started = std::chrono::steady_clock::now();
    const BoostPolygonSet blocked = *united;
    free_space -= blocked;
    subtraction_seconds += std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count();
  }

  std::list<BoostPolygonWithHoles> polygons;
  const auto contour_started = std::chrono::steady_clock::now();
  free_space.get(polygons);
  contour_extraction_seconds +=
      std::chrono::duration<double>(std::chrono::steady_clock::now() - contour_started).count();
  if (cache) {
    std::lock_guard lock(cache->impl_->mutex);
    // Detailed stages are non-overlapping. Preserve the aggregate's historical wall-clock
    // meaning, including cache lookup, cache bookkeeping, and local copies.
    cache->impl_->boolean_conversion_seconds += conversion_seconds;
    cache->impl_->boolean_offset_preparation_seconds += offset_preparation_seconds;
    cache->impl_->boolean_union_seconds += union_seconds;
    cache->impl_->boolean_subtraction_seconds += subtraction_seconds;
    cache->impl_->boolean_contour_extraction_seconds += contour_extraction_seconds;
    cache->impl_->boolean_operations_seconds +=
        std::chrono::duration<double>(std::chrono::steady_clock::now() - boolean_started).count();
  }
  const auto output_validation_started = std::chrono::steady_clock::now();
  const auto record_output_validation = [&] {
    if (!cache) return;
    std::lock_guard lock(cache->impl_->mutex);
    cache->impl_->output_validation_seconds +=
        std::chrono::duration<double>(std::chrono::steady_clock::now() - output_validation_started)
            .count();
  };
  const auto clearance_check = [&](const Polygon& contour) {
    auto neighbours = clearance_neighbours(contour, prepared_domain, config);
    if (!cache) return contour_has_required_clearance(contour, neighbours);
    const auto key = expansion_key({contour, {}}, Nanometres{0});
    const auto lookup = [&] {
      const auto bucket = cache->impl_->certificates.find(key);
      if (bucket == cache->impl_->certificates.end()) return false;
      return std::any_of(bucket->second.begin(), bucket->second.end(), [&](const auto& certificate) {
        return certificate->contour == contour && same_witnesses(certificate->witnesses, neighbours);
      });
    };
    {
      std::lock_guard lock(cache->impl_->mutex);
      if (lookup()) { ++cache->impl_->clearance_certificate_hits; return true; }
      ++cache->impl_->clearance_certificate_misses;
    }
    if (!contour_has_required_clearance(contour, neighbours)) return false;
    // Count referenced source vertices too: certificates must not keep an unbounded set of
    // evicted source BVHs alive. Overcounting shared sources gives a conservative storage bound.
    auto vertices = contour.vertices.size();
    for (const auto& neighbour : neighbours) vertices += neighbour.source->polygon.vertices.size();
    constexpr std::size_t certificate_vertex_limit = 262'144U;
    if (vertices > certificate_vertex_limit) return true;
    std::lock_guard lock(cache->impl_->mutex);
    if (lookup()) return true;
    while (!cache->impl_->certificate_order.empty() &&
           (cache->impl_->certificate_order.size() >= cache->impl_->capacity ||
            cache->impl_->certificate_vertices + vertices > certificate_vertex_limit)) {
      const auto& oldest = cache->impl_->certificate_order.front();
      auto& bucket = cache->impl_->certificates.at(oldest.first);
      std::erase(bucket, oldest.second);
      if (bucket.empty()) cache->impl_->certificates.erase(oldest.first);
      cache->impl_->certificate_vertices -= oldest.second->vertices;
      cache->impl_->certificate_order.pop_front();
    }
    auto certificate = std::make_shared<const ClearanceCertificate>(
        ClearanceCertificate{contour, std::move(neighbours), vertices});
    cache->impl_->certificates[key].push_back(certificate);
    cache->impl_->certificate_order.emplace_back(key, std::move(certificate));
    cache->impl_->certificate_vertices += vertices;
    return true;
  };
  for (const auto& polygon : polygons) {
    OffsetFreeSpaceRegion region;
    region.area = from_boost_polygon(polygon);
    for (auto hole = boost::polygon::begin_holes(polygon);
         hole != boost::polygon::end_holes(polygon); ++hole) {
      region.holes.push_back(from_boost_polygon(*hole));
    }
    auto vertices = region.area.vertices.size();
    if (vertices > config.maximum_output_vertices) {
      result.diagnostic = "free-space offset exceeds its output-vertex budget";
      result.regions.clear();
      record_output_validation();
      return result;
    }
    for (const auto& hole : region.holes) {
      if (hole.vertices.size() > config.maximum_output_vertices - vertices) {
        result.diagnostic = "free-space offset exceeds its output-vertex budget";
        result.regions.clear();
        record_output_validation();
        return result;
      }
      vertices += hole.vertices.size();
    }
    if (vertices > config.maximum_output_vertices - result.output_vertices) {
      result.diagnostic = "free-space offset exceeds its output-vertex budget";
      result.regions.clear();
      record_output_validation();
      return result;
    }
    std::string validation_diagnostic;
    if (!validate_region(region, domain, prepared_domain, clearance_check, validation_diagnostic)) {
      result.diagnostic =
          "free-space offset output failed exact conservative validation: " + validation_diagnostic;
      result.regions.clear();
      record_output_validation();
      return result;
    }
    result.output_vertices += vertices;
    result.regions.push_back(std::move(region));
  }
  std::sort(result.regions.begin(), result.regions.end(),
            [](const auto& lhs, const auto& rhs) { return lhs.area.vertices < rhs.area.vertices; });
  record_output_validation();
  return result;
}

} // namespace flux
