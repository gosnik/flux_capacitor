#include "flux/core/routing_engine.hpp"

#include <cstdlib>
#include <iostream>
#include <limits>

namespace {
void require(bool value, const char* message) {
  if (!value) {
    std::cerr << message << '\n';
    std::exit(1);
  }
}
} // namespace

int main() {
  const auto quality = [](std::size_t evidence, std::size_t opens, std::size_t drc,
                          std::size_t malformed, std::size_t bends, std::size_t overflow) {
    return flux::RoutingQuality{{evidence, opens, drc, malformed, bends, overflow, 0U}};
  };
  const auto baseline = quality(2U, 0U, 10U, 0U, 4U, 0U);
  require(!flux::RoutingEngineProgressBudget::strict_physical_improvement(baseline, baseline, true),
          "an identical evaluated physical scene cannot earn repeatedly");
  require(!flux::RoutingEngineProgressBudget::strict_physical_improvement(
              baseline, quality(1U, 0U, 10U, 0U, 4U, 0U), true),
          "topology or evidence-only progress cannot earn regional work");
  require(!flux::RoutingEngineProgressBudget::strict_physical_improvement(
              baseline, quality(2U, 1U, 9U, 0U, 4U, 0U), true),
          "an open-connection regression cannot earn despite fewer DRC violations");
  require(!flux::RoutingEngineProgressBudget::strict_physical_improvement(
              baseline, quality(2U, 0U, 9U, 1U, 4U, 0U), true),
          "a malformed-geometry regression cannot earn despite fewer DRC violations");
  const auto unknown_bends = quality(2U, 0U, 10U, 0U, flux::RoutingQuality::unknown_rank, 0U);
  require(!flux::RoutingEngineProgressBudget::strict_physical_improvement(
              unknown_bends, quality(2U, 0U, 10U, 0U, 0U, 0U), true),
          "measuring unknown bends without changing copper cannot earn more search");
  const auto improved = quality(2U, 0U, 9U, 0U, 4U, 0U);
  require(flux::RoutingEngineProgressBudget::strict_physical_improvement(baseline, improved, true),
          "a strict non-regressing physical improvement earns work");
  require(!flux::RoutingEngineProgressBudget::strict_physical_improvement(
              improved, improved, true),
          "the retained session-best scene prevents restart from re-earning the same credit");
  require(!flux::RoutingEngineProgressBudget::strict_physical_improvement(baseline, improved, false),
          "connectivity or coarse hard-gate loss blocks an otherwise physical improvement");

  // Zero-work decision/phase announcements cannot consume the initial allowance.  The engine
  // checks exhausted() at parked kernel checkpoints, retaining the last evaluated scene
  // and discarding any unfinished private transaction.
  flux::RoutingEngineProgressBudget budget{.allowance = 5U};
  budget.consume(0U);
  require(budget.consumed == 0U && !budget.exhausted(),
          "phase announcements must not consume regional work allowance");

  budget.consume(4U);
  require(budget.earn(3U, 6U) == 1U && budget.allowance == 6U && budget.earned == 1U,
          "a strict physical improvement credits only the remaining bounded allowance");
  require(budget.earn(3U, 6U) == 0U && budget.allowance == 6U,
          "the absolute regional ceiling must reject repeated credits");

  budget.consume(2U);
  require(budget.exhausted(),
          "charged child work must request cancellation at the next regional checkpoint");
  require(budget.earn(0U, 6U) == 0U && budget.allowance == 6U,
          "a non-improving evaluated scene must not restart an exhausted continuation");

  flux::RoutingEngineProgressBudget saturated{.allowance = 1U};
  saturated.consume(std::numeric_limits<std::size_t>::max());
  require(saturated.consumed == std::numeric_limits<std::size_t>::max() && saturated.exhausted(),
          "regional work accounting must saturate rather than wrap");
  return 0;
}
