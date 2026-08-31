#pragma once

#include "flux/core/free_space_skeleton.hpp"

#include <string>
#include <vector>

namespace flux {

struct BoardFreeSpaceExtractionConfig {
  LayerId layer;
  NetId routed_net;
  bool include_existing_copper{true};
  bool omit_same_net_copper{true};
  // Saved zone fills are physical copper by default. An explicit transactional KiCad workflow
  // may omit them while planning, but must refill and revalidate the exported candidate.
  bool include_saved_zone_fills{true};
};

struct BoardFreeSpaceExtractionResult {
  FreeSpaceDomain domain;
  // Stable source owners aligned one-for-one with domain.obstacles and filled_obstacles. These
  // preserve enough provenance to explain which physical objects bound an extracted channel;
  // board outline and cutout boundaries intentionally have no movable owner.
  std::vector<ObjectId> obstacle_ids;
  std::vector<ObjectId> filled_obstacle_ids;
  std::vector<ObjectId> conservative_obstacles;
  std::vector<ObjectId> unsupported_obstacles;
  std::string diagnostic;

  [[nodiscard]] bool complete() const {
    return diagnostic.empty() && unsupported_obstacles.empty();
  }
};

// Extracts polygonal foreign obstacles for one routed net/layer. Same-net copper is omitted.
// Conservative curved-pad envelopes are identified separately; unsupported existing segment/via
// copper remains an explicit blocker. Exact shape predicates remain authoritative for routing.
[[nodiscard]] BoardFreeSpaceExtractionResult
extract_polygonal_board_free_space(const Board& board,
                                   const BoardFreeSpaceExtractionConfig& config);

} // namespace flux
