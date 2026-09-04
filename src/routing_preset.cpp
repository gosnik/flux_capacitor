#include "flux/core/routing_preset.hpp"

namespace flux {

AutorouteV1Configuration make_autoroute_v1_configuration() {
  AutorouteV1Configuration result;
  auto& global = result.planner.global;
  // The production workflow uses the crossing-capacity-gated 45-degree topology. The core router
  // remains independently configurable so focused diagnostic callers can retain an orthogonal
  // comparison baseline.
  global.allow_crossing_safe_diagonal_grid_edges = true;
  global.max_iterations = 6U;
  global.max_expansions_per_connection = 50'000U;
  global.expansion_retry_heuristic_weight_per_mille = 10'000U;
  global.reuse_weighted_search_frontier = true;
  global.checkpoint_objective = GlobalRouteCheckpointObjective::dimensioned_physical_overflow;
  global.scale_voronoi_history_by_overflow = true;
  global.present_congestion_multiplier_per_mille = 1'050U;
  global.max_present_congestion_scale_per_mille = 10'000U;
  global.present_congestion_escalation_start_iteration = 4U;
  global.use_voronoi_allocation_resources = true;
  global.use_direct_voronoi_resource_edges = true;
  global.require_voronoi_topological_transitions = true;
  global.negotiate_voronoi_lane_conflicts = true;
  global.atomic_complete_net_routing = true;
  global.selective_congestion_reroute = true;
  global.order_routing_groups_by_physical_envelope = true;

  result.detailed.existing_routing = ExistingRoutingPolicy::preserve;
  result.detailed.topology_locked_medial_materialization = true;
  return result;
}

} // namespace flux
