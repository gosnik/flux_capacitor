#pragma once

#include "flux/core/free_space_offset.hpp"
#include "flux/core/geometry.hpp"

#include <algorithm>
#include <numeric>
#include <optional>
#include <vector>

namespace flux::detail {

// Immutable, board-bound obstacle bounds. Source order is restored after querying so local
// Boolean geometry and subsequent deterministic navigation see exactly the previous ordering.
class RegionalSourceIndex {
public:
  struct Entry {
    geometry::AxisAlignedBounds bounds;
    unsigned kind; // cutout, obstacle, filled obstacle
    std::size_t index;
  };

  explicit RegionalSourceIndex(const FreeSpaceDomain& source) {
    outline = bounds(source.outline);
    const auto append = [&](const Polygon& polygon, unsigned kind, std::size_t index) {
      if (const auto box = bounds(polygon)) entries_.push_back({*box, kind, index});
    };
    for (std::size_t i = 0; i < source.cutouts.size(); ++i) append(source.cutouts[i], 0U, i);
    for (std::size_t i = 0; i < source.obstacles.size(); ++i) append(source.obstacles[i], 1U, i);
    for (std::size_t i = 0; i < source.filled_obstacles.size(); ++i)
      append(source.filled_obstacles[i].area, 2U, i);
    if (!entries_.empty()) build(0U, entries_.size());
  }

  [[nodiscard]] std::vector<Entry> query(const geometry::AxisAlignedBounds& box) const {
    std::vector<Entry> result;
    if (nodes_.empty()) return result;
    std::vector<std::size_t> pending{0U};
    while (!pending.empty()) {
      const auto& node = nodes_[pending.back()];
      pending.pop_back();
      if (!node.box.overlaps(box)) continue;
      if (node.count != 0U) {
        for (std::size_t i = node.first; i < node.first + node.count; ++i)
          if (entries_[i].bounds.overlaps(box)) result.push_back(entries_[i]);
      } else {
        pending.push_back(node.right);
        pending.push_back(node.left);
      }
    }
    std::sort(result.begin(), result.end(), [](const auto& a, const auto& b) {
      return a.kind != b.kind ? a.kind < b.kind : a.index < b.index;
    });
    return result;
  }

  std::optional<geometry::AxisAlignedBounds> outline;

  [[nodiscard]] std::size_t retained_bytes() const {
    return sizeof(*this) + entries_.capacity() * sizeof(Entry) + nodes_.capacity() * sizeof(Node);
  }

private:
  struct Node {
    geometry::AxisAlignedBounds box;
    std::size_t first{}, count{}, left{}, right{};
  };
  static std::optional<geometry::AxisAlignedBounds> bounds(const Polygon& polygon) {
    if (polygon.vertices.empty()) return std::nullopt;
    geometry::AxisAlignedBounds box{polygon.vertices.front(), polygon.vertices.front()};
    for (const auto point : polygon.vertices) {
      box.minimum.x = std::min(box.minimum.x, point.x);
      box.minimum.y = std::min(box.minimum.y, point.y);
      box.maximum.x = std::max(box.maximum.x, point.x);
      box.maximum.y = std::max(box.maximum.y, point.y);
    }
    return box;
  }
  std::size_t build(std::size_t first, std::size_t count) {
    auto box = entries_[first].bounds;
    for (std::size_t i = first + 1U; i < first + count; ++i) {
      const auto& next = entries_[i].bounds;
      box.minimum.x = std::min(box.minimum.x, next.minimum.x);
      box.minimum.y = std::min(box.minimum.y, next.minimum.y);
      box.maximum.x = std::max(box.maximum.x, next.maximum.x);
      box.maximum.y = std::max(box.maximum.y, next.maximum.y);
    }
    const auto slot = nodes_.size();
    nodes_.push_back({box, first, count, 0U, 0U});
    if (count <= 8U) return slot;
    const bool x_axis = static_cast<long double>(box.maximum.x.value()) - box.minimum.x.value() >=
                        static_cast<long double>(box.maximum.y.value()) - box.minimum.y.value();
    std::stable_sort(entries_.begin() + static_cast<std::ptrdiff_t>(first),
                     entries_.begin() + static_cast<std::ptrdiff_t>(first + count),
                     [x_axis](const auto& a, const auto& b) {
      const auto center = [x_axis](const auto& e) {
        return x_axis ? std::midpoint(e.bounds.minimum.x.value(), e.bounds.maximum.x.value())
                      : std::midpoint(e.bounds.minimum.y.value(), e.bounds.maximum.y.value());
      };
      return center(a) < center(b);
    });
    const auto left = build(first, count / 2U);
    const auto right = build(first + count / 2U, count - count / 2U);
    nodes_[slot].count = 0U;
    nodes_[slot].left = left;
    nodes_[slot].right = right;
    return slot;
  }
  std::vector<Entry> entries_;
  std::vector<Node> nodes_;
};

} // namespace flux::detail
