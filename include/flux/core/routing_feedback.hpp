#pragma once

#include "flux/core/detailed_router.hpp"
#include "flux/core/regional_router.hpp"
#include "flux/core/router_telemetry.hpp"

#include <cstddef>
#include <stop_token>

namespace flux {

// Runs ordinary global negotiation, then applies bounded attributed spatial-cone repair only when
// sampled/physical resource overflow remains on an otherwise connected checkpoint.
[[nodiscard]] GlobalRouteResult route_global_with_sampled_conflict_repair(
    const Board& board, const GlobalRouterConfig& config, std::size_t max_repair_cones,
    std::stop_token stop_token = {}, RouterTelemetrySink* telemetry = nullptr);

// Builds the corresponding regional plan without invoking the ordinary outer planner loop. This
// preserves the explicit repair mode's semantics while sharing it between CLI and WorkPlan paths.
[[nodiscard]] RoutingPlanResult plan_routes_with_sampled_conflict_repair(
    const Board& board, const RoutingPlannerConfig& config, std::size_t max_repair_cones,
    std::stop_token stop_token = {}, RouterTelemetrySink* telemetry = nullptr);

struct MaterializationFeedbackResult {
  RoutingPlanResult routing;
  DetailedRouteResult detailed;
};

// Reuses exact materialization failures as cost-only routability knowledge. The global planning
// board may omit unlocked source copper covered by an explicit replacement transaction, while the
// materialization source retains that copper until the complete candidate is accepted. Only a
// strictly better exact detailed result replaces the caller's checkpoint.
[[nodiscard]] MaterializationFeedbackResult refine_materialization_with_routability_feedback(
    const Board& planning_board, const Board& materialization_source,
    RoutingPlanResult initial_routing, DetailedRouteResult initial_detailed,
    const RoutingPlannerConfig& planner_config, const DetailedRouterConfig& detailed_config,
    std::size_t max_feedback_passes, std::stop_token stop_token = {},
    RouterTelemetrySink* telemetry = nullptr);

} // namespace flux
