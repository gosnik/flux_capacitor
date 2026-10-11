#pragma once

#include "flux/core/copper_scaffold.hpp"

#include <vector>

namespace flux {

// Returns the retained request domain unchanged unless it is a complete, fully checked
// scaffold.  For such a scene, append deterministic pad-to-pad demands between currently
// disconnected physical components of nets already represented by the domain.  Existing
// requests are retained as an exact prefix. A shared pair-comparison budget bounds nearest
// edge generation; an unfinished net contributes no partial suffix.
[[nodiscard]] std::vector<GlobalRoutePath>
append_residual_physical_demands(const CopperScaffoldResult& retained,
                                 std::size_t maximum_pad_pair_checks = 1'000'000U);

} // namespace flux
