#include "flux/core/indexed_clearance_map.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <numeric>
#include <utility>

namespace flux {
namespace {

using Map = IndexedClearanceMap;
constexpr std::uint32_t leaf_size = 8U;
constexpr long double epsilon = 1e-12L;

struct Bounds {
  std::int64_t min_x, min_y, max_x, max_y;
};

[[nodiscard]] Bounds bounds_of(const IndexedClearancePrimitive& p) {
  return {std::min(p.start_x_nm, p.end_x_nm), std::min(p.start_y_nm, p.end_y_nm),
          std::max(p.start_x_nm, p.end_x_nm), std::max(p.start_y_nm, p.end_y_nm)};
}
[[nodiscard]] Bounds combine(Bounds a, Bounds b) {
  return {std::min(a.min_x, b.min_x), std::min(a.min_y, b.min_y), std::max(a.max_x, b.max_x),
          std::max(a.max_y, b.max_y)};
}
[[nodiscard]] long double point_box_distance_squared(long double x, long double y,
                                                     const Map::BvhNode& b) {
  const auto dx = x < b.min_x   ? static_cast<long double>(b.min_x) - x
                  : x > b.max_x ? x - static_cast<long double>(b.max_x)
                                : 0.0L;
  const auto dy = y < b.min_y   ? static_cast<long double>(b.min_y) - y
                  : y > b.max_y ? y - static_cast<long double>(b.max_y)
                                : 0.0L;
  return dx * dx + dy * dy;
}
[[nodiscard]] long double point_segment_distance_squared(long double x, long double y,
                                                         const IndexedClearancePrimitive& p) {
  const long double ax = p.start_x_nm, ay = p.start_y_nm,
                    dx = static_cast<long double>(p.end_x_nm) - ax,
                    dy = static_cast<long double>(p.end_y_nm) - ay;
  const long double den = dx * dx + dy * dy;
  const long double t =
      den == 0.0L ? 0.0L : std::clamp(((x - ax) * dx + (y - ay) * dy) / den, 0.0L, 1.0L);
  const long double ex = x - (ax + t * dx), ey = y - (ay + t * dy);
  return ex * ex + ey * ey;
}
[[nodiscard]] bool box_overlaps(const Map::BvhNode& n, std::int64_t min_x, std::int64_t min_y,
                                std::int64_t max_x, std::int64_t max_y) {
  return n.min_x <= max_x && n.max_x >= min_x && n.min_y <= max_y && n.max_y >= min_y;
}
[[nodiscard]] bool point_on_segment(long double x, long double y,
                                    const IndexedClearancePrimitive& p) {
  const long double cross = (static_cast<long double>(x) - p.start_x_nm) *
                                (static_cast<long double>(p.end_y_nm) - p.start_y_nm) -
                            (static_cast<long double>(y) - p.start_y_nm) *
                                (static_cast<long double>(p.end_x_nm) - p.start_x_nm);
  return std::abs(cross) <= epsilon && x >= std::min(p.start_x_nm, p.end_x_nm) &&
         x <= std::max(p.start_x_nm, p.end_x_nm) && y >= std::min(p.start_y_nm, p.end_y_nm) &&
         y <= std::max(p.start_y_nm, p.end_y_nm);
}
[[nodiscard]] int orientation(long double ax, long double ay, long double bx, long double by,
                              long double cx, long double cy) {
  const auto v = (bx - ax) * (cy - ay) - (by - ay) * (cx - ax);
  return v > epsilon ? 1 : v < -epsilon ? -1 : 0;
}
[[nodiscard]] bool seg_intersect(long double ax, long double ay, long double bx, long double by,
                                 long double cx, long double cy, long double dx, long double dy) {
  const int a = orientation(ax, ay, bx, by, cx, cy), b = orientation(ax, ay, bx, by, dx, dy),
            c = orientation(cx, cy, dx, dy, ax, ay), d = orientation(cx, cy, dx, dy, bx, by);
  if (a != b && c != d)
    return true;
  const auto on = [](long double ax, long double ay, long double bx, long double by, long double x,
                     long double y) {
    return std::abs((bx - ax) * (y - ay) - (by - ay) * (x - ax)) <= epsilon &&
           x >= std::min(ax, bx) && x <= std::max(ax, bx) && y >= std::min(ay, by) &&
           y <= std::max(ay, by);
  };
  return (a == 0 && on(ax, ay, bx, by, cx, cy)) || (b == 0 && on(ax, ay, bx, by, dx, dy)) ||
         (c == 0 && on(cx, cy, dx, dy, ax, ay)) || (d == 0 && on(cx, cy, dx, dy, bx, by));
}
[[nodiscard]] bool segment_touches_box(const IndexedClearancePrimitive& p, std::int64_t min_x,
                                       std::int64_t min_y, std::int64_t max_x, std::int64_t max_y) {
  const auto b = bounds_of(p);
  if (b.max_x < min_x || b.min_x > max_x || b.max_y < min_y || b.min_y > max_y)
    return false;
  if ((p.start_x_nm >= min_x && p.start_x_nm <= max_x && p.start_y_nm >= min_y &&
       p.start_y_nm <= max_y) ||
      (p.end_x_nm >= min_x && p.end_x_nm <= max_x && p.end_y_nm >= min_y && p.end_y_nm <= max_y))
    return true;
  return seg_intersect(p.start_x_nm, p.start_y_nm, p.end_x_nm, p.end_y_nm, min_x, min_y, max_x,
                       min_y) ||
         seg_intersect(p.start_x_nm, p.start_y_nm, p.end_x_nm, p.end_y_nm, max_x, min_y, max_x,
                       max_y) ||
         seg_intersect(p.start_x_nm, p.start_y_nm, p.end_x_nm, p.end_y_nm, max_x, max_y, min_x,
                       max_y) ||
         seg_intersect(p.start_x_nm, p.start_y_nm, p.end_x_nm, p.end_y_nm, min_x, max_y, min_x,
                       min_y);
}
} // namespace

const IndexedClearanceCellRecord*
IndexedClearanceMap::record_for_cell(std::uint32_t cell_index) const {
  if (cell_index >= cell_record_indices_.size())
    return nullptr;
  const auto index = cell_record_indices_[cell_index];
  return index == invalid_index || index >= records_.size() ? nullptr : &records_[index];
}

IndexedClearancePointQuery IndexedClearanceMap::exact_query(long double x, long double y) const {
  IndexedClearancePointQuery result;
  if (bvh_.empty())
    return result;
  long double best = std::numeric_limits<long double>::infinity();
  std::vector<std::uint32_t> stack{0};
  while (!stack.empty()) {
    const auto ni = stack.back();
    stack.pop_back();
    const auto& n = bvh_[ni];
    if (point_box_distance_squared(x, y, n) > best)
      continue;
    if (n.leaf())
      for (std::uint32_t i = 0; i < n.count; ++i) {
        const auto pi = primitive_order_[n.first + i];
        const auto d = point_segment_distance_squared(x, y, primitives_[pi]);
        if (d < best || (d == best && pi < result.nearest_primitive_index)) {
          best = d;
          result.nearest_primitive_index = pi;
        }
      }
    else {
      if (n.left != invalid_index)
        stack.push_back(n.left);
      if (n.right != invalid_index)
        stack.push_back(n.right);
    }
  }
  result.boundary_distance_nm = std::sqrt(best);
  result.on_boundary = result.boundary_distance_nm <= 1e-9L;
  // Half-open ray test avoids vertex double counting. BVH pruning ensures this never scans all
  // segments except for genuinely overlapping ray bounds.
  bool parity = false;
  stack = {0};
  while (!stack.empty()) {
    const auto ni = stack.back();
    stack.pop_back();
    const auto& n = bvh_[ni];
    if (n.max_y < y || n.min_y > y || n.max_x < x)
      continue;
    if (n.leaf())
      for (std::uint32_t i = 0; i < n.count; ++i) {
        const auto& p = primitives_[primitive_order_[n.first + i]];
        if (point_on_segment(x, y, p))
          result.on_boundary = true;
        const auto ay = p.start_y_nm, by = p.end_y_nm;
        if ((ay > y) != (by > y)) {
          const long double crossing = static_cast<long double>(p.start_x_nm) +
                                       (static_cast<long double>(y) - ay) *
                                           (static_cast<long double>(p.end_x_nm) - p.start_x_nm) /
                                           (static_cast<long double>(by) - ay);
          if (crossing > x)
            parity = !parity;
        }
      }
    else {
      if (n.left != invalid_index)
        stack.push_back(n.left);
      if (n.right != invalid_index)
        stack.push_back(n.right);
    }
  }
  result.inside = parity;
  return result;
}

std::size_t IndexedClearanceMap::allocated_bytes() const {
  return primitives_.capacity() * sizeof(IndexedClearancePrimitive) +
         primitive_order_.capacity() * sizeof(std::uint32_t) + bvh_.capacity() * sizeof(BvhNode) +
         cell_record_indices_.capacity() * sizeof(std::uint32_t) +
         records_.capacity() * sizeof(IndexedClearanceCellRecord) +
         overlap_primitive_indices_.capacity() * sizeof(std::uint32_t);
}

std::vector<std::uint32_t> IndexedClearanceMap::segment_candidates(
    long double ax, long double ay, long double bx, long double by,
    long double candidate_radius_nm, long double clearance_nm,
    std::optional<std::uint32_t> layer_id) const {
  std::vector<std::uint32_t> result;
  if (bvh_.empty()) return result;
  const auto overlaps = [&](long double min_x, long double min_y, long double max_x,
                            long double max_y, long double radius) {
    const auto expansion = radius + candidate_radius_nm + clearance_nm;
    // Invalid/overflowed broad-phase input must retain candidates, never silently clear them.
    if (!std::isfinite(ax) || !std::isfinite(ay) || !std::isfinite(bx) || !std::isfinite(by) ||
        !std::isfinite(expansion) || candidate_radius_nm < 0 || clearance_nm < 0) return true;
    return !(std::max(ax, bx) + expansion < min_x || std::min(ax, bx) - expansion > max_x ||
             std::max(ay, by) + expansion < min_y || std::min(ay, by) - expansion > max_y);
  };
  std::vector<std::uint32_t> stack{0U};
  while (!stack.empty()) {
    const auto index = stack.back(); stack.pop_back();
    const auto& node = bvh_[index];
    if (!overlaps(node.min_x, node.min_y, node.max_x, node.max_y,
                  node.maximum_copper_radius_nm)) continue;
    if (node.leaf()) {
      for (std::uint32_t i = 0; i < node.count; ++i) {
        const auto id = primitive_order_[node.first + i];
        const auto& primitive = primitives_[id];
        if (layer_id && primitive.layer_id && layer_id != primitive.layer_id) continue;
        const auto bounds = bounds_of(primitive);
        if (overlaps(bounds.min_x, bounds.min_y, bounds.max_x, bounds.max_y,
                     primitive.copper_radius_nm)) result.push_back(id);
      }
    } else {
      stack.push_back(node.left); stack.push_back(node.right);
    }
  }
  std::sort(result.begin(), result.end());
  return result;
}

IndexedClearanceMapBuildResult
build_indexed_clearance_map(IndexedClearanceMapConfig config,
                            std::vector<IndexedClearancePrimitive> primitives) {
  IndexedClearanceMapBuildResult out;
  if (std::any_of(primitives.begin(), primitives.end(), [](const auto& p) {
        return !std::isfinite(p.copper_radius_nm) || p.copper_radius_nm < 0.0L;
      })) {
    out.diagnostic = "copper radius must be finite and nonnegative";
    return out;
  }
  if (config.pitch_nm <= 0 || config.width == 0 || config.height == 0) {
    out.diagnostic = "pitch, width, and height must be positive";
    return out;
  }
  const auto valid_extent = [&](std::int64_t origin, std::uint32_t count) {
    const auto limit = std::numeric_limits<std::int64_t>::max();
    if (config.pitch_nm > limit / count) return false;
    const auto extent = config.pitch_nm * static_cast<std::int64_t>(count);
    return origin <= limit - extent;
  };
  if (!valid_extent(config.origin_x_nm, config.width) ||
      !valid_extent(config.origin_y_nm, config.height) ||
      primitives.size() > std::numeric_limits<std::uint32_t>::max() / 2U) {
    out.diagnostic = "grid coordinates or primitive indices exceed representable range";
    return out;
  }
  const auto cells = static_cast<std::uint64_t>(config.width) * config.height;
  if (cells > config.maximum_cells || cells > std::numeric_limits<std::uint32_t>::max()) {
    out.status = IndexedClearanceBuildStatus::limit_exceeded;
    out.diagnostic = "logical cell cap exceeded";
    return out;
  }
  const auto selected = !config.materialize_cells ? 0U :
      config.sampled_cells.empty() ? cells : config.sampled_cells.size();
  for (const auto cell : config.sampled_cells)
    if (cell >= cells) {
      out.diagnostic = "sampled cell is outside grid";
      return out;
    }
  const auto estimate =
      static_cast<unsigned long long>(config.materialize_cells ? cells : 0U) * sizeof(std::uint32_t) +
      static_cast<unsigned long long>(selected) * sizeof(IndexedClearanceCellRecord) +
      static_cast<unsigned long long>(primitives.size()) *
          (sizeof(IndexedClearancePrimitive) + sizeof(std::uint32_t) * 2U +
           sizeof(IndexedClearanceMap::BvhNode) * 2U);
  out.estimated_bytes = estimate > std::numeric_limits<std::size_t>::max()
                            ? std::numeric_limits<std::size_t>::max()
                            : static_cast<std::size_t>(estimate);
  if (estimate > config.maximum_bytes) {
    out.status = IndexedClearanceBuildStatus::limit_exceeded;
    out.diagnostic = "pre-allocation byte cap exceeded";
    return out;
  }
  if (config.cancellation_requested && config.cancellation_requested()) {
    out.status = IndexedClearanceBuildStatus::cancelled;
    out.diagnostic = "cancelled before allocation";
    return out;
  }
  auto map = std::make_shared<IndexedClearanceMap>();
  map->origin_x_nm_ = config.origin_x_nm;
  map->origin_y_nm_ = config.origin_y_nm;
  map->pitch_nm_ = config.pitch_nm;
  map->width_ = config.width;
  map->height_ = config.height;
  map->geometry_revision_ = config.geometry_revision;
  map->primitives_ = std::move(primitives);
  map->primitive_order_.resize(map->primitives_.size());
  std::iota(map->primitive_order_.begin(), map->primitive_order_.end(), 0U);
  // Deterministic median BVH.  Index ranges are reordered, never the immutable primitive pool.
  const auto build = [&](auto&& self, std::uint32_t first, std::uint32_t count) -> std::uint32_t {
    const auto node_index = static_cast<std::uint32_t>(map->bvh_.size());
    map->bvh_.push_back({});
    Bounds b = bounds_of(map->primitives_[map->primitive_order_[first]]);
    for (std::uint32_t i = 1; i < count; ++i)
      b = combine(b, bounds_of(map->primitives_[map->primitive_order_[first + i]]));
    auto& node = map->bvh_[node_index];
    node.min_x = b.min_x;
    node.min_y = b.min_y;
    node.max_x = b.max_x;
    node.max_y = b.max_y;
    for (std::uint32_t i = 0; i < count; ++i)
      node.maximum_copper_radius_nm = std::max(node.maximum_copper_radius_nm,
          map->primitives_[map->primitive_order_[first + i]].copper_radius_nm);
    if (count <= leaf_size) {
      node.first = first;
      node.count = count;
      return node_index;
    }
    const bool split_x = (static_cast<long double>(b.max_x) - b.min_x) >=
                         (static_cast<long double>(b.max_y) - b.min_y);
    const auto mid = first + count / 2U;
    std::nth_element(
        map->primitive_order_.begin() + first, map->primitive_order_.begin() + mid,
        map->primitive_order_.begin() + first + count, [&](std::uint32_t a, std::uint32_t b) {
          const auto& pa = map->primitives_[a];
          const auto& pb = map->primitives_[b];
          const auto ca = (split_x ? static_cast<long double>(pa.start_x_nm) + pa.end_x_nm
                                   : static_cast<long double>(pa.start_y_nm) + pa.end_y_nm);
          const auto cb = (split_x ? static_cast<long double>(pb.start_x_nm) + pb.end_x_nm
                                   : static_cast<long double>(pb.start_y_nm) + pb.end_y_nm);
          return ca == cb ? a < b : ca < cb;
        });
    const auto left = self(self, first, mid - first);
    const auto right = self(self, mid, first + count - mid);
    map->bvh_[node_index].left = left;
    map->bvh_[node_index].right = right;
    return node_index;
  };
  if (!map->primitives_.empty())
    build(build, 0U, static_cast<std::uint32_t>(map->primitives_.size()));
  if (config.cancellation_requested && config.cancellation_requested()) {
    out.status = IndexedClearanceBuildStatus::cancelled;
    out.diagnostic = "cancelled while indexing primitives";
    return out;
  }
  if (!config.materialize_cells) {
    if (map->allocated_bytes() > config.maximum_bytes) {
      out.status = IndexedClearanceBuildStatus::limit_exceeded;
      out.diagnostic = "primitive index exceeds byte cap";
      return out;
    }
    out.status = IndexedClearanceBuildStatus::ok;
    out.estimated_bytes = map->allocated_bytes();
    out.map = std::move(map);
    out.diagnostic = "ok";
    return out;
  }
  const auto collect = [&](std::int64_t min_x, std::int64_t min_y, std::int64_t max_x,
                           std::int64_t max_y, std::vector<std::uint32_t>& candidates) {
    if (map->bvh_.empty())
      return;
    std::vector<std::uint32_t> stack{0};
    while (!stack.empty()) {
      const auto ni = stack.back();
      stack.pop_back();
      const auto& n = map->bvh_[ni];
      if (!box_overlaps(n, min_x, min_y, max_x, max_y))
        continue;
      if (n.leaf())
        for (std::uint32_t i = 0; i < n.count; ++i)
          candidates.push_back(map->primitive_order_[n.first + i]);
      else {
        if (n.left != IndexedClearanceMap::invalid_index)
          stack.push_back(n.left);
        if (n.right != IndexedClearanceMap::invalid_index)
          stack.push_back(n.right);
      }
    }
  };
  const bool sparse = !config.sampled_cells.empty();
  auto requested = std::move(config.sampled_cells);
  if (sparse) {
    std::sort(requested.begin(), requested.end());
    requested.erase(std::unique(requested.begin(), requested.end()), requested.end());
  }
  const auto requested_count = sparse ? static_cast<std::uint64_t>(requested.size()) : cells;
  const auto each_requested = [&](const auto& callback) {
    if (sparse) {
      for (const auto cell : requested)
        callback(cell);
    } else
      for (std::uint32_t cell = 0; cell < cells; ++cell)
        callback(cell);
  };
  const auto cell_bounds = [&](std::uint32_t cell) {
    const auto cx = cell % config.width, cy = cell / config.width;
    const auto min_x = config.origin_x_nm + static_cast<std::int64_t>(cx) * config.pitch_nm,
               min_y = config.origin_y_nm + static_cast<std::int64_t>(cy) * config.pitch_nm;
    return std::array<std::int64_t, 4U>{min_x, min_y, min_x + config.pitch_nm,
                                        min_y + config.pitch_nm};
  };
  std::uint64_t reference_count = 0;
  bool cancelled = false;
  bool cancelled_during_materialization = false;
  each_requested([&](std::uint32_t cell) {
    if (cancelled)
      return;
    if (config.cancellation_requested && config.cancellation_requested()) {
      cancelled = true;
      return;
    }
    const auto b = cell_bounds(cell);
    std::vector<std::uint32_t> candidates;
    collect(b[0], b[1], b[2], b[3], candidates);
    for (const auto pi : candidates)
      if (segment_touches_box(map->primitives_[pi], b[0], b[1], b[2], b[3]))
        ++reference_count;
  });
  if (cancelled) {
    out.status = IndexedClearanceBuildStatus::cancelled;
    out.diagnostic = "cancelled while sizing overlap references";
    return out;
  }
  if (reference_count > config.maximum_references ||
      reference_count > std::numeric_limits<std::uint32_t>::max()) {
    out.status = IndexedClearanceBuildStatus::limit_exceeded;
    out.diagnostic = "overlap reference cap exceeded";
    return out;
  }
  const auto persistent_bytes =
      static_cast<unsigned long long>(map->allocated_bytes()) +
      static_cast<unsigned long long>(cells) * sizeof(std::uint32_t) +
      static_cast<unsigned long long>(requested_count) * sizeof(IndexedClearanceCellRecord) +
      reference_count * sizeof(std::uint32_t);
  out.estimated_bytes = persistent_bytes > std::numeric_limits<std::size_t>::max()
                            ? std::numeric_limits<std::size_t>::max()
                            : static_cast<std::size_t>(persistent_bytes);
  if (persistent_bytes > config.maximum_bytes) {
    out.status = IndexedClearanceBuildStatus::limit_exceeded;
    out.diagnostic = "pre-allocation byte cap exceeded including overlaps";
    return out;
  }
  map->cell_record_indices_.assign(static_cast<std::size_t>(cells),
                                   IndexedClearanceMap::invalid_index);
  map->records_.reserve(static_cast<std::size_t>(requested_count));
  map->overlap_primitive_indices_.reserve(static_cast<std::size_t>(reference_count));
  if (map->allocated_bytes() > config.maximum_bytes) {
    out.status = IndexedClearanceBuildStatus::limit_exceeded;
    out.diagnostic = "allocator capacity exceeded byte cap";
    return out;
  }
  each_requested([&](std::uint32_t cell) {
    if (cancelled_during_materialization)
      return;
    if (config.cancellation_requested && config.cancellation_requested()) {
      cancelled_during_materialization = true;
      return;
    }
    const auto b = cell_bounds(cell);
    const auto q = map->exact_query(
        static_cast<long double>(b[0]) + static_cast<long double>(config.pitch_nm) * 0.5L,
        static_cast<long double>(b[1]) + static_cast<long double>(config.pitch_nm) * 0.5L);
    IndexedClearanceCellRecord r;
    r.center_boundary_distance_nm = static_cast<double>(q.boundary_distance_nm);
    const auto lower =
        std::max(0.0L, q.boundary_distance_nm -
                           std::sqrt(2.0L) * static_cast<long double>(config.pitch_nm) * 0.5L);
    r.whole_cell_distance_lower_bound_nm =
        lower == 0.0L ? 0.0 : std::nextafter(static_cast<double>(lower), 0.0);
    r.nearest_primitive_index = q.nearest_primitive_index;
    r.center_inside = q.inside;
    std::vector<std::uint32_t> candidates;
    collect(b[0], b[1], b[2], b[3], candidates);
    const auto start = map->overlap_primitive_indices_.size();
    for (const auto pi : candidates)
      if (segment_touches_box(map->primitives_[pi], b[0], b[1], b[2], b[3])) {
        r.boundary_touched = true;
        map->overlap_primitive_indices_.push_back(pi);
      }
    r.overlap_offset = static_cast<std::uint32_t>(start);
    r.overlap_count = static_cast<std::uint32_t>(map->overlap_primitive_indices_.size() - start);
    map->cell_record_indices_[cell] = static_cast<std::uint32_t>(map->records_.size());
    map->records_.push_back(r);
  });
  if (cancelled_during_materialization) {
    out.status = IndexedClearanceBuildStatus::cancelled;
    out.diagnostic = "cancelled while materializing cells";
    return out;
  }
  if (map->allocated_bytes() > config.maximum_bytes) {
    out.status = IndexedClearanceBuildStatus::limit_exceeded;
    out.diagnostic = "final allocation exceeded byte cap";
    return out;
  }
  out.status = IndexedClearanceBuildStatus::ok;
  out.map = std::move(map);
  out.diagnostic = "ok";
  return out;
}

} // namespace flux
