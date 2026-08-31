#pragma once

#include "flux/core/board.hpp"

#include <cstddef>
#include <optional>
#include <string>
#include <vector>

namespace flux {

struct PnsShoveConfig {
  std::size_t max_propagation_depth{8};
  std::size_t max_candidate_checks{64};
  Nanometres minimum_displacement{Nanometres::from_mm(0.05)};
  Nanometres maximum_displacement{Nanometres::from_mm(2.0)};

  bool operator==(const PnsShoveConfig&) const = default;
};

// A caller-proposed local via site/layer-span alternative. The PNS engine does not infer a
// fabrication policy: it tries only the via's translated current span and these explicit
// alternatives, then subjects the complete via-plus-attached-track transaction to canonical
// validation, connectivity, and copper DRC.
struct PnsViaRelocationCandidate {
  ObjectId via;
  Point position;
  std::vector<LayerId> layers;

  bool operator==(const PnsViaRelocationCandidate&) const = default;
};

struct PnsShoveRequest {
  std::vector<std::string> intruder_object_ids;
  std::vector<std::string> colliding_object_ids;
  std::vector<std::string> protected_object_ids;
  Point collision_point;
  long double normal_x{0.0L};
  long double normal_y{0.0L};
  Nanometres required_displacement;
  std::vector<PnsViaRelocationCandidate> via_relocation_candidates;
};

struct PnsShoveResult {
  std::optional<Board> candidate;
  std::size_t candidate_checks{0};
  std::size_t propagation_depth{0};
  std::size_t moved_tracks{0};
  std::size_t moved_vias{0};
  std::size_t added_dogleg_tracks{0};
  std::size_t local_splits{0};
  bool budget_exhausted{false};
  std::vector<std::string> moved_object_ids;
  std::string diagnostic;

  [[nodiscard]] bool accepted() const { return candidate.has_value(); }
};

// A bounded transactional PNS engine for straight unlocked copper and unlocked vias. A displaced
// segment retains
// unaffected source tails and replaces its local collision interval with entry/parallel/exit
// pieces, preserving electrical endpoints. New collisions may recursively shove other unlocked
// straight segments or vias. A moved via carries every unlocked attached straight-track endpoint;
// explicit candidates may also propose a different legal layer span. Pads, arcs, locked/protected
// attachments, keepouts, board boundaries, budget exhaustion, connectivity regression, or exact
// DRC rejection fail the complete shove transaction closed.
[[nodiscard]] PnsShoveResult pns_push_and_shove(const Board& board, const PnsShoveRequest& request,
                                                const PnsShoveConfig& config = {});

} // namespace flux
