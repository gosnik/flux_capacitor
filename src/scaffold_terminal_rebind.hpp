#pragma once

#include "flux/core/copper_scaffold.hpp"

#include <string>
#include <utility>
#include <vector>

namespace flux {

struct ScaffoldTerminalRebind {
  std::size_t request_index{0U};
  GlobalRoutePoint original_endpoint;
  GlobalRoutePoint terminal;
  std::string proof;
};

// Deliberately narrow: derives only direct selected-coarse-producer pad rebinds for observed
// zero-attachment abstract endpoints, or budget-exhausted branches of physically connected
// producers. It never substitutes a nearest pad.
[[nodiscard]] std::pair<std::vector<GlobalRoutePath>, std::vector<ScaffoldTerminalRebind>>
derive_scaffold_terminal_rebinds(const Board&, const CopperScaffoldResult&,
                               const std::vector<std::string>& attempted_proofs = {},
                               const std::string& physical_context = {});

// A changed demand domain may never discard an already checked peer obligation.
[[nodiscard]] bool scaffold_preserves_satisfied_requests(const CopperScaffoldResult& candidate,
                                                        const CopperScaffoldResult& incumbent);

} // namespace flux
