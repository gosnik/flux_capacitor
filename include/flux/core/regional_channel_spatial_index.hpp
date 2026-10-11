#pragma once

#include "flux/core/geometry.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <map>
#include <optional>
#include <utility>
#include <vector>

namespace flux::detail {

// Append-only conservative index for transient regional-channel topology cells. Oversized cells
// are deliberately returned for every query; callers retain exact prepared-polygon validation.
class RegionalChannelSpatialIndex {
public:
  void append(std::size_t channel_index, const geometry::PreparedPolygon& cell) {
    const auto range = bin_range(cell);
    if (!range) {
      if (cell.valid()) oversized_.push_back(channel_index);
      return;
    }
    for (auto y = (*range)[1U];;) {
      for (auto x = (*range)[0U];;) {
        bins_[{x, y}].push_back(channel_index);
        if (x == (*range)[2U]) break;
        ++x;
      }
      if (y == (*range)[3U]) break;
      ++y;
    }
  }

  [[nodiscard]] std::vector<std::size_t>
  query(const std::vector<geometry::PreparedPolygon>& incoming, std::size_t channel_count) const {
    std::vector<bool> selected(channel_count, false);
    const auto select = [&](std::size_t channel_index) {
      if (channel_index < selected.size()) selected[channel_index] = true;
    };
    for (const auto channel_index : oversized_) select(channel_index);
    for (const auto& cell : incoming) {
      if (!cell.valid()) continue;
      const auto range = bin_range(cell);
      if (!range) {
        std::fill(selected.begin(), selected.end(), true);
        break;
      }
      for (auto y = (*range)[1U];;) {
        for (auto x = (*range)[0U];;) {
          const auto bin = bins_.find({x, y});
          if (bin != bins_.end())
            for (const auto channel_index : bin->second) select(channel_index);
          if (x == (*range)[2U]) break;
          ++x;
        }
        if (y == (*range)[3U]) break;
        ++y;
      }
    }
    std::vector<std::size_t> result;
    for (std::size_t channel_index = 0U; channel_index < selected.size(); ++channel_index)
      if (selected[channel_index]) result.push_back(channel_index);
    return result;
  }

private:
  using Bin = std::pair<std::int64_t, std::int64_t>;
  static constexpr std::int64_t pitch_nm_ = 1'000'000;
  static constexpr std::uint64_t maximum_axis_bins_ = 16U;
  static constexpr std::uint64_t maximum_bins_ = 256U;

  [[nodiscard]] static std::int64_t bin_coordinate(std::int64_t coordinate) {
    auto quotient = coordinate / pitch_nm_;
    if (coordinate < 0 && coordinate % pitch_nm_ != 0) --quotient;
    return quotient;
  }

  [[nodiscard]] static std::optional<std::array<std::int64_t, 4U>>
  bin_range(const geometry::PreparedPolygon& cell) {
    if (!cell.valid()) return std::nullopt;
    const auto x0 = bin_coordinate(cell.bounds.minimum.x.value());
    const auto y0 = bin_coordinate(cell.bounds.minimum.y.value());
    const auto x1 = bin_coordinate(cell.bounds.maximum.x.value());
    const auto y1 = bin_coordinate(cell.bounds.maximum.y.value());
    if (x1 < x0 || y1 < y0) return std::nullopt;
    const auto width = static_cast<long double>(x1) - static_cast<long double>(x0) + 1.0L;
    const auto height = static_cast<long double>(y1) - static_cast<long double>(y0) + 1.0L;
    if (!(width > 0.0L) || !(height > 0.0L) ||
        width > static_cast<long double>(maximum_axis_bins_) ||
        height > static_cast<long double>(maximum_axis_bins_) ||
        width * height > static_cast<long double>(maximum_bins_))
      return std::nullopt;
    return std::array{x0, y0, x1, y1};
  }

  std::map<Bin, std::vector<std::size_t>> bins_;
  std::vector<std::size_t> oversized_;
};

} // namespace flux::detail
