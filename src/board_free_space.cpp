#include "flux/core/board_free_space.hpp"

#include "flux/core/bezier_copper.hpp"
#include "flux/core/copper_geometry.hpp"
#include "flux/core/geometry.hpp"
#include "flux/core/keepout_geometry.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <set>

namespace flux {
struct BoardFreeSpaceExtractionCache::Impl {
  struct Entry {
    Board board;
    LayerId layer;
    bool include_existing_copper;
    bool include_saved_zone_fills;
    BoardFreeSpaceExtractionResult catalogue;
    std::size_t age{0U};
  };
  explicit Impl(std::size_t requested_capacity)
      : capacity(std::max<std::size_t>(1U, requested_capacity)) {}
  std::size_t capacity;
  std::size_t clock{0U};
  std::size_t hits{0U};
  std::size_t misses{0U};
  std::vector<Entry> entries;
  mutable std::mutex mutex;
};

BoardFreeSpaceExtractionCache::BoardFreeSpaceExtractionCache(std::size_t capacity)
    : impl_(std::make_unique<Impl>(capacity)) {}
BoardFreeSpaceExtractionCache::~BoardFreeSpaceExtractionCache() = default;
BoardFreeSpaceExtractionCache::BoardFreeSpaceExtractionCache(
    BoardFreeSpaceExtractionCache&&) noexcept = default;
BoardFreeSpaceExtractionCache& BoardFreeSpaceExtractionCache::operator=(
    BoardFreeSpaceExtractionCache&&) noexcept = default;
BoardFreeSpaceExtractionCache::Stats BoardFreeSpaceExtractionCache::stats() const {
  std::lock_guard lock(impl_->mutex);
  return {impl_->hits, impl_->misses, impl_->entries.size()};
}
void BoardFreeSpaceExtractionCache::clear() {
  std::lock_guard lock(impl_->mutex);
  impl_->entries.clear();
  impl_->hits = impl_->misses = impl_->clock = 0U;
}

namespace {

[[nodiscard]] std::optional<Polygon> conservative_track_outline(const TrackSegment& track) {
  if (track.width.value() <= 0) {
    return std::nullopt;
  }
  const auto dx = static_cast<long double>(track.end.x.value() - track.start.x.value());
  const auto dy = static_cast<long double>(track.end.y.value() - track.start.y.value());
  const auto length = std::hypot(dx, dy);
  if (length == 0.0L) {
    const CopperPadShape point_track{
        track.id,         track.net,
        track.start,      {track.layer},
        PadShape::circle, track.width,
        track.width,      Nanometres{0},
        Microdegrees{0},  static_cast<long double>(track.width.value()) / 2.0L};
    auto outline = polygonal_pad_outline(point_track);
    return outline ? std::optional<Polygon>{std::move(outline->area)} : std::nullopt;
  }

  const auto tangent_x = dx / length;
  const auto tangent_y = dy / length;
  const auto normal_x = -tangent_y;
  const auto normal_y = tangent_x;
  const auto guarded_radius = static_cast<long double>(track.width.value()) / 2.0L + 1.0L;
  const auto point = [](long double x, long double y) {
    return Point{Nanometres{static_cast<std::int64_t>(std::llround(x))},
                 Nanometres{static_cast<std::int64_t>(std::llround(y))}};
  };
  const auto start_x = static_cast<long double>(track.start.x.value());
  const auto start_y = static_cast<long double>(track.start.y.value());
  const auto end_x = static_cast<long double>(track.end.x.value());
  const auto end_y = static_cast<long double>(track.end.y.value());
  Polygon area{{point(start_x - tangent_x * guarded_radius + normal_x * guarded_radius,
                      start_y - tangent_y * guarded_radius + normal_y * guarded_radius),
                point(start_x - tangent_x * guarded_radius - normal_x * guarded_radius,
                      start_y - tangent_y * guarded_radius - normal_y * guarded_radius),
                point(end_x + tangent_x * guarded_radius - normal_x * guarded_radius,
                      end_y + tangent_y * guarded_radius - normal_y * guarded_radius),
                point(end_x + tangent_x * guarded_radius + normal_x * guarded_radius,
                      end_y + tangent_y * guarded_radius + normal_y * guarded_radius)}};
  return geometry::is_simple_polygon(area) ? std::optional<Polygon>{std::move(area)} : std::nullopt;
}

[[nodiscard]] std::optional<Polygon> conservative_via_outline(const Via& via) {
  const CopperPadShape shape{
      via.id,           via.net,
      via.position,     via.layers,
      PadShape::circle, via.diameter,
      via.diameter,     Nanometres{0},
      Microdegrees{0},  static_cast<long double>(via.diameter.value()) / 2.0L};
  auto outline = polygonal_pad_outline(shape);
  return outline ? std::optional<Polygon>{std::move(outline->area)} : std::nullopt;
}

[[nodiscard]] std::optional<std::vector<Polygon>>
conservative_track_arc_outline(const TrackArc& arc) {
  const auto parameters = geometry::circular_arc_parameters(arc.start, arc.middle, arc.end);
  if (!parameters || arc.width.value() <= 0) {
    return std::nullopt;
  }
  const auto pi = std::acos(-1.0L);
  // Cover only the swept arc, not its entire supporting disk. Each chord's copper envelope
  // is enlarged by the exact sagitta and rounding guard, so this remains conservative for
  // either orientation, major arcs, and round end caps. A union of simple rectangles also
  // handles thick arcs without constructing a self-intersecting annular polygon.
  constexpr long double maximum_sagitta_nm = 5'000.0L;
  const auto step = std::min(
      pi / 8.0L,
      2.0L * std::acos(std::clamp(1.0L - maximum_sagitta_nm / parameters->radius_nm, -1.0L, 1.0L)));
  const auto count = std::ceil(std::abs(parameters->sweep_radians) / step);
  if (!std::isfinite(count) || count < 1.0L || count > 8'192.0L)
    return std::nullopt;
  const auto sides = static_cast<std::size_t>(count);
  const auto delta = parameters->sweep_radians / static_cast<long double>(sides);
  const auto sagitta = parameters->radius_nm * (1.0L - std::cos(delta / 2.0L));
  const auto width = static_cast<long double>(arc.width.value()) + 2.0L * sagitta + 4.0L;
  if (width >= static_cast<long double>(std::numeric_limits<std::int64_t>::max()))
    return std::nullopt;
  const auto at = [&](std::size_t index) {
    if (index == 0U)
      return arc.start;
    if (index == sides)
      return arc.end;
    const auto angle = parameters->start_angle_radians + delta * index;
    return Point{Nanometres{static_cast<std::int64_t>(std::llround(
                     parameters->center_x_nm + parameters->radius_nm * std::cos(angle)))},
                 Nanometres{static_cast<std::int64_t>(std::llround(
                     parameters->center_y_nm + parameters->radius_nm * std::sin(angle)))}};
  };
  std::vector<Polygon> areas;
  areas.reserve(sides);
  for (std::size_t index = 0U; index < sides; ++index) {
    auto outline = conservative_track_outline(
        TrackSegment{arc.id, arc.net, arc.layer, at(index), at(index + 1U),
                     Nanometres{static_cast<std::int64_t>(std::ceil(width))}, arc.locked});
    if (!outline)
      return std::nullopt;
    areas.push_back(std::move(*outline));
  }
  return areas;
}

} // namespace

[[nodiscard]] BoardFreeSpaceExtractionResult extract_polygonal_board_free_space_uncached(
    const Board& board, const BoardFreeSpaceExtractionConfig& config) {
  BoardFreeSpaceExtractionResult result;
  result.domain.outline = board.outline;
  result.domain.cutouts = board.cutouts;
  const auto layer =
      std::find_if(board.layers.begin(), board.layers.end(), [&config](const auto& l) {
        return l.id == config.layer && l.kind == LayerKind::copper;
      });
  if (layer == board.layers.end()) {
    result.diagnostic = "polygonal free-space extraction requires a declared copper layer";
    return result;
  }
  if (config.omit_same_net_copper && std::none_of(board.nets.begin(), board.nets.end(),
                   [&config](const auto& net) { return net.id == config.routed_net; })) {
    result.diagnostic = "polygonal free-space extraction requires a declared routed net";
    return result;
  }

  for (const auto& keepout : world_keepouts(board)) {
    if (keepout.prohibit.tracks && keepout_contains_layer(keepout, config.layer)) {
      result.domain.obstacles.push_back(keepout.area);
      result.obstacle_ids.push_back(keepout.id);
      result.obstacle_nets.push_back(std::nullopt);
    }
  }
  for (const auto& pad : copper_pad_shapes(board)) {
    if ((config.omit_same_net_copper && pad.net == config.routed_net) ||
        std::find(pad.layers.begin(), pad.layers.end(), config.layer) == pad.layers.end()) {
      continue;
    }
    if (auto polygon = polygonal_pad_outline(pad)) {
      result.domain.obstacles.push_back(std::move(polygon->area));
      result.obstacle_ids.push_back(pad.id);
      result.obstacle_nets.push_back(pad.net);
      if (polygon->conservative) {
        result.conservative_obstacles.push_back(pad.id);
      }
    } else {
      result.unsupported_obstacles.push_back(pad.id);
    }
  }
  if (config.include_saved_zone_fills) {
    for (const auto& zone : board.zones) {
      if (config.omit_same_net_copper && zone.net == config.routed_net) {
        continue;
      }
      for (const auto& fill : zone.fills) {
        if (fill.layer != config.layer) {
          continue;
        }
        result.domain.filled_obstacles.push_back(fill);
        result.filled_obstacle_ids.push_back(zone.id);
        result.filled_obstacle_nets.push_back(zone.net);
      }
    }
  }
  if (config.include_existing_copper) {
    for (const auto& track : board.tracks) {
      if ((!config.omit_same_net_copper || track.net != config.routed_net) &&
          track.layer == config.layer) {
        if (auto polygon = conservative_track_outline(track)) {
          result.domain.obstacles.push_back(std::move(*polygon));
          result.obstacle_ids.push_back(track.id);
          result.obstacle_nets.push_back(track.net);
          result.conservative_obstacles.push_back(track.id);
        } else {
          result.unsupported_obstacles.push_back(track.id);
        }
      }
    }
    for (const auto& arc : board.track_arcs) {
      if ((!config.omit_same_net_copper || arc.net != config.routed_net) &&
          arc.layer == config.layer) {
        if (auto polygons = conservative_track_arc_outline(arc)) {
          for (auto& polygon : *polygons) {
            result.domain.obstacles.push_back(std::move(polygon));
            result.obstacle_ids.push_back(arc.id);
            result.obstacle_nets.push_back(arc.net);
          }
          result.conservative_obstacles.push_back(arc.id);
        } else {
          result.unsupported_obstacles.push_back(arc.id);
        }
      }
    }
    for (const auto& curve : board.track_beziers) {
      if ((!config.omit_same_net_copper || curve.net != config.routed_net) &&
          curve.layer == config.layer) {
        const auto flattened = tessellate_track_bezier(curve);
        if (!flattened.certified) {
          result.unsupported_obstacles.push_back(curve.id);
          continue;
        }
        bool supported = true;
        for (const auto& segment : flattened.segments) {
          const auto guard = segment.centerline_deviation_nm.value() + segment.quantization_error_nm.value();
          if (guard > (std::numeric_limits<std::int64_t>::max() - curve.width.value()) / 2) {
            supported = false;
            break;
          }
          TrackSegment proxy{curve.id, curve.net, curve.layer, segment.start, segment.end,
                             Nanometres{curve.width.value() + 2 * guard}, curve.locked};
          if (auto polygon = conservative_track_outline(proxy)) {
            result.domain.obstacles.push_back(std::move(*polygon));
            result.obstacle_ids.push_back(curve.id);
            result.obstacle_nets.push_back(curve.net);
          } else { supported = false; break; }
        }
        if (supported) result.conservative_obstacles.push_back(curve.id);
        else result.unsupported_obstacles.push_back(curve.id);
      }
    }
    for (const auto& via : board.vias) {
      if ((!config.omit_same_net_copper || via.net != config.routed_net) &&
          std::find(via.layers.begin(), via.layers.end(), config.layer) != via.layers.end()) {
        if (auto polygon = conservative_via_outline(via)) {
          result.domain.obstacles.push_back(std::move(*polygon));
          result.obstacle_ids.push_back(via.id);
          result.obstacle_nets.push_back(via.net);
          result.conservative_obstacles.push_back(via.id);
        } else {
          result.unsupported_obstacles.push_back(via.id);
        }
      }
    }
  }
  std::sort(result.conservative_obstacles.begin(), result.conservative_obstacles.end());
  result.conservative_obstacles.erase(
      std::unique(result.conservative_obstacles.begin(), result.conservative_obstacles.end()),
      result.conservative_obstacles.end());
  std::sort(result.unsupported_obstacles.begin(), result.unsupported_obstacles.end());
  result.unsupported_obstacles.erase(
      std::unique(result.unsupported_obstacles.begin(), result.unsupported_obstacles.end()),
      result.unsupported_obstacles.end());
  return result;
}

[[nodiscard]] BoardFreeSpaceExtractionResult filter_catalogue(
    const Board& board, const BoardFreeSpaceExtractionResult& catalogue,
    const BoardFreeSpaceExtractionConfig& config) {
  BoardFreeSpaceExtractionResult result;
  result.domain.outline = catalogue.domain.outline;
  result.domain.cutouts = catalogue.domain.cutouts;
  std::set<ObjectId> excluded_ids;
  if (config.omit_same_net_copper) {
    for (const auto& footprint : board.footprints)
      for (const auto& pad : footprint.pads)
        if (pad.net == config.routed_net) excluded_ids.insert(pad.id);
    for (const auto& zone : board.zones)
      if (zone.net == config.routed_net) excluded_ids.insert(zone.id);
    for (const auto& track : board.tracks)
      if (track.net == config.routed_net) excluded_ids.insert(track.id);
    for (const auto& arc : board.track_arcs)
      if (arc.net == config.routed_net) excluded_ids.insert(arc.id);
    for (const auto& curve : board.track_beziers)
      if (curve.net == config.routed_net) excluded_ids.insert(curve.id);
    for (const auto& via : board.vias)
      if (via.net == config.routed_net) excluded_ids.insert(via.id);
  }
  const auto keep_owner = [&](const ObjectId& id) { return !excluded_ids.contains(id); };
  std::copy_if(catalogue.conservative_obstacles.begin(), catalogue.conservative_obstacles.end(),
               std::back_inserter(result.conservative_obstacles), keep_owner);
  std::copy_if(catalogue.unsupported_obstacles.begin(), catalogue.unsupported_obstacles.end(),
               std::back_inserter(result.unsupported_obstacles), keep_owner);
  result.diagnostic = catalogue.diagnostic;
  const auto filter = [&](const auto& source_areas, const auto& source_ids, const auto& source_nets,
                          auto& target_areas, auto& target_ids, auto& target_nets) {
    for (std::size_t index = 0U; index < source_areas.size(); ++index) {
      if (config.omit_same_net_copper && source_nets[index] &&
          *source_nets[index] == config.routed_net) {
        continue;
      }
      target_areas.push_back(source_areas[index]);
      target_ids.push_back(source_ids[index]);
      target_nets.push_back(source_nets[index]);
    }
  };
  filter(catalogue.domain.obstacles, catalogue.obstacle_ids, catalogue.obstacle_nets,
         result.domain.obstacles, result.obstacle_ids, result.obstacle_nets);
  filter(catalogue.domain.filled_obstacles, catalogue.filled_obstacle_ids,
         catalogue.filled_obstacle_nets, result.domain.filled_obstacles,
         result.filled_obstacle_ids, result.filled_obstacle_nets);
  return result;
}

BoardFreeSpaceExtractionResult extract_polygonal_board_free_space(
    const Board& board, const BoardFreeSpaceExtractionConfig& config,
    BoardFreeSpaceExtractionCache* cache) {
  if (!cache) return extract_polygonal_board_free_space_uncached(board, config);
  if (config.omit_same_net_copper && std::none_of(board.nets.begin(), board.nets.end(),
          [&config](const auto& net) { return net.id == config.routed_net; })) {
    BoardFreeSpaceExtractionResult result;
    result.diagnostic = "polygonal free-space extraction requires a declared routed net";
    return result;
  }
  BoardFreeSpaceExtractionResult catalogue;
  {
    std::lock_guard lock(cache->impl_->mutex);
    for (auto& entry : cache->impl_->entries) {
      if (entry.layer == config.layer && entry.include_existing_copper == config.include_existing_copper &&
          entry.include_saved_zone_fills == config.include_saved_zone_fills && entry.board == board) {
        entry.age = ++cache->impl_->clock;
        ++cache->impl_->hits;
        return filter_catalogue(board, entry.catalogue, config);
      }
    }
  }
  const BoardFreeSpaceExtractionConfig catalogue_config{config.layer, NetId{},
                                                         config.include_existing_copper, false,
                                                         config.include_saved_zone_fills};
  catalogue = extract_polygonal_board_free_space_uncached(board, catalogue_config);
  {
    std::lock_guard lock(cache->impl_->mutex);
    for (auto& entry : cache->impl_->entries) {
      if (entry.layer == config.layer && entry.include_existing_copper == config.include_existing_copper &&
          entry.include_saved_zone_fills == config.include_saved_zone_fills && entry.board == board) {
        entry.age = ++cache->impl_->clock;
        ++cache->impl_->hits;
        return filter_catalogue(board, entry.catalogue, config);
      }
    }
    ++cache->impl_->misses;
    if (cache->impl_->entries.size() == cache->impl_->capacity) {
      cache->impl_->entries.erase(std::min_element(
          cache->impl_->entries.begin(), cache->impl_->entries.end(),
          [](const auto& lhs, const auto& rhs) { return lhs.age < rhs.age; }));
    }
    cache->impl_->entries.push_back({board, config.layer, config.include_existing_copper,
                                     config.include_saved_zone_fills, std::move(catalogue),
                                     ++cache->impl_->clock});
    return filter_catalogue(board, cache->impl_->entries.back().catalogue, config);
  }
}

} // namespace flux
