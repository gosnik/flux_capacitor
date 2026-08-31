#pragma once

#include "flux/core/board.hpp"

#include <cstddef>
#include <vector>

namespace flux {

struct NetConnectivity {
  NetId net;
  std::size_t pad_count{0};
  std::size_t connected_pad_groups{0};
  std::size_t open_connections{0};
};

struct ConnectivityReport {
  std::vector<NetConnectivity> nets;
  std::size_t open_connections{0};
  std::size_t unrouted_nets{0};
};

// Deterministic connectivity over routed centerlines, physical widths for zone contact,
// exact supported pad shapes, vias, and immutable filled-zone polygons. Pads owned by the
// canonical empty-name KiCad net-zero entry remain inventoried but require no connection to one
// another. KiCad DRC remains the external acceptance oracle for KiCad artifacts.
[[nodiscard]] ConnectivityReport analyze_connectivity(const Board& board);

} // namespace flux
