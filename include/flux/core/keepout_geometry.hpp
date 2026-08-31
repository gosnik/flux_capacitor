#pragma once

#include "flux/core/board.hpp"

#include <optional>
#include <vector>

namespace flux {

struct WorldKeepout {
  ObjectId id;
  Polygon area;
  std::vector<LayerId> layers;
  KeepoutRules prohibit;
  std::optional<ObjectId> owner_footprint;
};

[[nodiscard]] std::vector<WorldKeepout> world_keepouts(const Board& board);
[[nodiscard]] bool keepout_contains_layer(const WorldKeepout& keepout, const LayerId& layer);
[[nodiscard]] bool point_copper_overlaps_keepout(Point center, long double radius_nm,
                                                 const Polygon& keepout);
[[nodiscard]] bool segment_copper_overlaps_keepout(Point start, Point end, long double radius_nm,
                                                   const Polygon& keepout);

} // namespace flux
