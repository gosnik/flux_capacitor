#pragma once

#include "flux/core/detailed_router.hpp"
#include "flux/core/regional_router.hpp"

namespace flux {

// One production baseline shared by direct CLI routing and FluxWorkPlan compilation. Callers may
// apply explicit validated overrides, but must not maintain a second implied autoroute profile.
struct AutorouteV1Configuration {
  RoutingPlannerConfig planner;
  DetailedRouterConfig detailed;

  bool operator==(const AutorouteV1Configuration&) const = default;
};

[[nodiscard]] AutorouteV1Configuration make_autoroute_v1_configuration();

} // namespace flux
