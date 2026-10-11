#pragma once

#include "flux/core/routing_compute.hpp"

#include <cstddef>
#include <stop_token>
#include <vector>

namespace flux {

// Particles in one group move by a single planar translation.  It is intended for the
// per-layer copies of a via and the endpoints attached to that via; a fixed member pins
// the complete group.
struct RoutingAvbdRigidGroup {
  std::vector<std::size_t> particle_indices;
};

// Bounded augmented-variational block-descent controls.  The XPBD configuration still
// supplies PCB dimensions, corridor envelopes, and the per-step displacement bound.
struct RoutingAvbdConfig {
  std::size_t iterations{16U};
  std::size_t maximum_contacts{65'536U};
  std::size_t maximum_particles{16'384U};
  std::size_t max_workers{0U};
  long double initial_penalty{1.0e-5L};
  long double maximum_penalty{1.0e4L};
  long double penalty_growth{2.0L};
  bool operator==(const RoutingAvbdConfig&) const = default;
};

struct RoutingAvbdResult {
  // This remains diagnostic geometry until the regional caller performs its exact copper
  // legality gate.  `ok()` therefore never claims final DRC legality.
  RoutingXpbdResult relaxation;
  std::size_t contacts{0U};
  std::size_t duplicate_reductions{0U};
  std::size_t colors{0U};
  std::size_t local_block_solves{0U};
  std::size_t duals{0U};
  std::size_t penalty_updates{0U};
  std::size_t maximum_degree{0U};
  std::size_t moved_rigid_owners{0U};
  bool cancelled{false};

  [[nodiscard]] bool ok() const { return !cancelled && relaxation.ok(); }
};

// A bounded CPU augmented variational block descent pass.  Contacts are rebuilt every
// iteration; all cap, index, and numerical failures are fail-closed through diagnostic.
[[nodiscard]] RoutingAvbdResult
relax_routing_avbd(const std::vector<RoutingXpbdParticle>& particles,
                   const std::vector<RoutingXpbdSegment>& segments,
                   const std::vector<RoutingXpbdObstacleSegment>& obstacles,
                   const RoutingXpbdConfig& xpbd_config = {},
                   const std::vector<RoutingAvbdRigidGroup>& rigid_groups = {},
                   const RoutingAvbdConfig& avbd_config = {},
                   std::stop_token stop_token = {});

} // namespace flux
