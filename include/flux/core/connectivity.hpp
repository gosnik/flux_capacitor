#pragma once

#include "flux/core/board.hpp"

#include <cstddef>
#include <optional>
#include <vector>

namespace flux {

struct NetConnectivity {
  NetId net;
  std::size_t pad_count{0};
  std::size_t connected_pad_groups{0};
  std::size_t open_connections{0};
};

// Root identity emitted by the canonical connectivity DSU for an actual or temporary pad.
// Identities are valid only for the ConnectivityReport that produced them.
struct PadConnectivityComponent {
  ObjectId pad;
  NetId net;
  std::size_t component{0};
};

struct ConnectivityReport {
  std::vector<NetConnectivity> nets;
  std::vector<PadConnectivityComponent> pad_components;
  std::size_t open_connections{0};
  std::size_t unrouted_nets{0};
};

// Deterministic connectivity over routed centerlines, physical widths for zone contact,
// exact supported pad shapes, vias, and immutable filled-zone polygons. Pads owned by the
// canonical empty-name KiCad net-zero entry remain inventoried but require no connection to one
// another. KiCad DRC remains the external acceptance oracle for KiCad artifacts.
[[nodiscard]] ConnectivityReport analyze_connectivity(const Board& board);

// A point/layer probe used for conservative physical-component queries.  `layers` represents
// every copper layer on which the caller may legally attach (for example all layers of a pad),
// rather than an inferred through-hole connection.
struct CopperConnectivityProbe {
  Point position;
  std::vector<LayerId> layers;
};

struct CopperProbeComponentMembership {
  bool contained_by_copper{false};
  std::vector<std::size_t> components;
};

// Resolves a batch of point/layer probes against one canonical connectivity DSU construction.
// Each layer is injected as a separate zero-area pad, so a multi-layer probe cannot fabricate a
// through connection.  Component numbers are only comparable within the returned batch.
[[nodiscard]] std::vector<CopperProbeComponentMembership>
copper_probe_component_memberships(const Board& board, const NetId& net,
                                   const std::vector<CopperConnectivityProbe>& probes);

// Returns true only when both probes are contained by physical same-net copper and that copper
// belongs to one component under the same exact DSU predicates used by analyze_connectivity().
// Empty or unsupported probes fail closed.  This is deliberately a membership query, not a
// proximity test: equal net IDs, coincident coordinates on different layers, and adjacent copper
// are not sufficient.
[[nodiscard]] bool copper_probes_share_component(const Board& board, const NetId& net,
                                                  const CopperConnectivityProbe& first,
                                                  const CopperConnectivityProbe& second);

} // namespace flux
