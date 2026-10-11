#pragma once

#include "flux/core/board.hpp"
#include "flux/core/navmesh_route.hpp"

#include <cstdint>
#include <vector>

namespace flux {

// A replacement snapshot of copper which has been laid during this routing session.  It is a
// terrain-only view: exact obstacle extraction and all copper legality continue to use Board.
// Keeping this separate avoids turning provisional routing evidence into canonical board state.
enum class LiveCopperTerrainKind { segment, arc, bezier, via };

struct LiveCopperTerrainFeature {
  LiveCopperTerrainKind kind{LiveCopperTerrainKind::segment};
  NetId net;
  LayerId layer;
  geometry::AxisAlignedBounds bounds;
  Point axis_start;
  Point axis_end;
  Nanometres radius{Nanometres{0}};
  bool directional{false};
  std::uint32_t pressure{0U};

  bool operator==(const LiveCopperTerrainFeature&) const = default;
};

struct LiveCopperTerrainSnapshot {
  // Provisional route scaffolds guide only a subsequent planning pass; detailed candidates are
  // physical laid copper.  Neither kind modifies NavMesh walkability.
  bool provisional{false};
  std::vector<LiveCopperTerrainFeature> features;

  bool operator==(const LiveCopperTerrainSnapshot&) const = default;
};

[[nodiscard]] LiveCopperTerrainSnapshot
make_live_copper_terrain_snapshot(const Board& board, bool provisional = false,
                                  std::uint32_t pressure = 1U);

// Returns only foreign copper on the queried layer.  The returned deposits are intentionally
// advisory NavMesh pressure and cannot alter its polygon-derived legal mask.
[[nodiscard]] std::vector<NavMeshHydraulicTerrainDeposit>
live_copper_terrain_deposits(const LiveCopperTerrainSnapshot& snapshot, NetId net,
                             LayerId layer);

} // namespace flux
