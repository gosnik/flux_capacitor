#pragma once

#include "flux/core/copper_scaffold.hpp"

#include <optional>
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

// Produces only the new demands needed to restore physical pad groups which were connected in
// `incumbent` and became separated in `candidate`. Both boards are queried through the exact
// connectivity oracle. A missing pad/component/layer witness, or either shared bound preventing
// complete restoration, fails closed with nullopt.
[[nodiscard]] std::optional<std::vector<GlobalRoutePath>>
derive_pad_preservation_demands(const Board& incumbent, const Board& candidate,
                                const std::vector<GlobalRoutePath>& existing_requests,
                                std::size_t maximum_demands = 2U,
                                std::size_t maximum_pad_pair_checks = 100'000U);

} // namespace flux
