#pragma once

#include "flux/core/routing_preset.hpp"
#include "flux/io/work_plan.hpp"

#include <cstddef>
#include <string>
#include <vector>

namespace flux::io {

struct CompiledWorkPlanRouteStep {
  std::string id;
  WorkPlanStepKind kind{WorkPlanStepKind::route};
  AutorouteV1Configuration configuration;
  std::vector<NetId> selected_nets;
  std::vector<NetId> priority_nets;
  std::vector<std::string> applied_options;
  // Bounded exact-copper rejection feedback after global/regional checkpoint selection.
  std::size_t max_final_routability_feedback_passes{3U};
  bool post_negotiation_sampled_conflict_repair{false};
  std::size_t max_post_negotiation_sampled_conflict_repair_cones{2U};
  bool refillable_zone_fills{false};
  bool retain_incomplete_diagnostic_preview{false};

  bool operator==(const CompiledWorkPlanRouteStep&) const = default;
};

struct WorkPlanRouteCompileResult {
  WorkPlanExpansionResult expansion;
  std::vector<CompiledWorkPlanRouteStep> route_steps;
  std::vector<WorkPlanDiagnostic> diagnostics;

  [[nodiscard]] bool has_errors() const;
  [[nodiscard]] bool ok() const { return expansion.ok() && !has_errors(); }
};

// Compiles supported route-like steps into the same autoroute_v1 configuration used by the direct
// CLI. Differential-pair and bus steps retain their declared atomic group semantics. Unknown
// options and type/range mismatches fail closed. No search or board mutation occurs here.
[[nodiscard]] WorkPlanRouteCompileResult compile_flux_work_plan_routes(const Board& source,
                                                                       const FluxWorkPlan& plan);

} // namespace flux::io
