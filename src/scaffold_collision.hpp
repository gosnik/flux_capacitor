#pragma once

#include "flux/core/drc.hpp"
#include "flux/core/indexed_clearance_map.hpp"

namespace flux {

// Immutable broad phase for one retained copper revision. The reduced query board retains all
// rules, board boundaries, zones and keepouts; exact DRC remains the narrow-phase authority.
class ScaffoldCollisionIndex {
public:
  explicit ScaffoldCollisionIndex(const Board& board);
  [[nodiscard]] bool ready() const { return static_cast<bool>(index_); }
  [[nodiscard]] CopperDrcReport check_track(const TrackSegment& track) const;
  [[nodiscard]] CopperDrcReport check_via(const Via& via) const;
private:
  enum class Kind { track, arc, via, footprint };
  struct Owner {
    Kind kind;
    std::size_t index;
    std::size_t pad_index{0U};
  };
  Board source_;
  Board context_;
  std::vector<Owner> owners_;
  std::shared_ptr<const IndexedClearanceMap> index_;
  long double clearance_{0};
  [[nodiscard]] Board nearby(Point a, Point b, long double radius) const;
};

} // namespace flux
