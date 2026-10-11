#pragma once

#include "flux/core/board.hpp"
#include "flux/core/global_router.hpp"
#include "flux/core/routing_avbd.hpp"

#include <cstddef>
#include <cstdint>
#include <optional>
#include <stop_token>
#include <string>
#include <vector>

namespace flux {

// Bounded controls for a single retained-passage repair.  This is deliberately a
// deformation adapter: it does not search for a new topology, materialize copper,
// or certify DRC legality.  The caller must pass a scene with the selected movable
// tracks removed and atomically validate the complete returned bundle.
struct ScaffoldJointDisplacementConfig {
  std::size_t maximum_particles{256U};
  std::size_t maximum_iterations{32U};
  std::size_t maximum_contacts{32'768U};
  std::int64_t maximum_movement_nm{2'000'000};
  std::int64_t maximum_particle_spacing_nm{500'000};
  bool enable_residual_pns{false};
  RoutingXpbdConfig xpbd{};
  RoutingAvbdConfig avbd{};

  bool operator==(const ScaffoldJointDisplacementConfig&) const = default;
};

struct ScaffoldJointDisplacementResult {
  // Incoming path is first; retained owners follow in their input order.  These
  // paths are approximate relaxation proposals, never a legal_scaffold claim.
  std::optional<std::vector<GlobalRoutePath>> paths;
  std::size_t particles{0U};
  std::size_t segments{0U};
  std::size_t obstacle_segments{0U};
  std::size_t pinned_particles{0U};
  std::size_t xpbd_iterations{0U};
  std::size_t xpbd_candidate_pairs{0U};
  std::size_t xpbd_unresolved_trace_pairs{0U};
  std::size_t avbd_contacts{0U};
  std::size_t avbd_residual_contacts{0U};
  bool used_avbd{false};
  bool cancelled{false};
  bool budget_exhausted{false};
  // PNS is intentionally deferred to the caller's exact transactional scene.
  // It cannot turn this approximate proposal into a legal result by itself.
  bool residual_pns_requested{false};
  std::string diagnostic;
  std::string refinement_diagnostic;

  [[nodiscard]] bool ok() const { return paths.has_value() && !cancelled && diagnostic.empty(); }
};

[[nodiscard]] ScaffoldJointDisplacementResult solve_scaffold_joint_displacement(
    const Board& immutable_scene_without_movable_generated_tracks,
    const std::vector<GlobalRoutePath>& incoming_first_and_owners,
    const ScaffoldJointDisplacementConfig& config = {}, std::stop_token stop_token = {});

} // namespace flux
