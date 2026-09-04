#pragma once

#include "flux/core/routing_preset.hpp"
#include "flux/io/work_plan.hpp"

#include <string>
#include <vector>

namespace flux::io {

struct CompiledWorkPlanRouteStep {
  std::string id;
  AutorouteV1Configuration configuration;
  std::vector<NetId> selected_nets;
  std::vector<NetId> priority_nets;
  std::vector<std::string> applied_options;

  bool operator==(const CompiledWorkPlanRouteStep&) const = default;
};

struct WorkPlanRouteCompileResult {
  WorkPlanExpansionResult expansion;
  std::vector<CompiledWorkPlanRouteStep> route_steps;
  std::vector<WorkPlanDiagnostic> diagnostics;

  [[nodiscard]] bool has_errors() const;
  [[nodiscard]] bool ok() const { return expansion.ok() && !has_errors(); }
};

// Compiles supported route steps into the same autoroute_v1 configuration used by the direct CLI.
// Unknown options and type/range mismatches fail closed. No search or board mutation occurs here.
[[nodiscard]] WorkPlanRouteCompileResult compile_flux_work_plan_routes(const Board& source,
                                                                       const FluxWorkPlan& plan);

} // namespace flux::io
