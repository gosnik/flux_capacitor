#pragma once

#include "flux/core/board.hpp"
#include "flux/core/global_router.hpp"

#include <cstddef>
#include <vector>

namespace flux {

// Candidate physical attachment sites for a coarse scaffold request.  Starts and goals are
// bounded, deterministic alternatives on the copper components containing the corresponding
// request endpoints.  They are intentionally not a general terminal generator.
struct ScaffoldConnectionAccess {
  std::vector<GlobalRoutePoint> starts;
  std::vector<GlobalRoutePoint> goals;
  bool source_endpoint_contained_by_copper{false};
  bool target_endpoint_contained_by_copper{false};
  bool already_connected{false};
};

[[nodiscard]] ScaffoldConnectionAccess scaffold_connection_access(
    const Board& board, const GlobalRoutePath& request, std::size_t maximum_points = 64U);

// Conservative physical membership check for the endpoints of one request.  This considers all
// layers of a pad containing an endpoint, even if its coarse route point selected another layer.
[[nodiscard]] bool scaffold_request_connected(const Board& board, const GlobalRoutePath& request);

} // namespace flux
