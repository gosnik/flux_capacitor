#pragma once

#include "flux/core/board.hpp"

#include <cstdint>
#include <filesystem>
#include <map>
#include <optional>
#include <string>
#include <string_view>
#include <variant>
#include <vector>

namespace flux::io {

inline constexpr std::uint32_t flux_work_plan_schema_version = 1U;

enum class WorkPlanMode {
  preview,
  new_routing,
  force_reroute_selected,
  allowlisted_ripup,
};

enum class WorkPlanStepKind {
  route,
  fanout,
  differential_pair,
  bus,
  plane,
  tuning,
  cleanup,
  validation,
  placement,
};

enum class WorkPlanSelectorScope {
  selected,
  touching,
  internal,
};

using WorkPlanOptionValue = std::variant<bool, std::int64_t, std::uint64_t, std::string>;

struct WorkPlanSelectors {
  bool all_nets{false};
  std::vector<std::string> exact_nets;
  std::vector<std::string> include_globs;
  std::vector<std::string> exclude_globs;
  std::vector<std::string> net_classes;
  std::vector<std::string> footprints;
  std::vector<std::string> placement_groups;
  WorkPlanSelectorScope group_scope{WorkPlanSelectorScope::selected};

  bool operator==(const WorkPlanSelectors&) const = default;
};

struct WorkPlanStep {
  std::string id;
  WorkPlanStepKind kind{WorkPlanStepKind::validation};
  std::vector<std::string> depends_on;
  WorkPlanSelectors selectors;
  std::map<std::string, WorkPlanOptionValue> options;

  bool operator==(const WorkPlanStep&) const = default;
};

struct WorkPlanPolicyIdentity {
  std::string kind;
  std::string path;
  std::string sha256;
  std::string source_revision;

  bool operator==(const WorkPlanPolicyIdentity&) const = default;
};

struct WorkPlanTelemetry {
  bool enabled{true};
  std::string listen{"127.0.0.1:8765"};
  std::uint32_t rate_hz{10U};
  std::string level{"normal"};
  std::optional<std::string> record;

  bool operator==(const WorkPlanTelemetry&) const = default;
};

struct FluxWorkPlan {
  std::uint32_t schema_version{flux_work_plan_schema_version};
  std::string source_path;
  std::string source_revision;
  WorkPlanMode mode{WorkPlanMode::preview};
  std::vector<std::string> ripup_allowlist;
  std::vector<WorkPlanPolicyIdentity> policies;
  WorkPlanTelemetry telemetry;
  std::vector<WorkPlanStep> steps;

  bool operator==(const FluxWorkPlan&) const = default;
};

enum class WorkPlanDiagnosticSeverity { warning, error };

struct WorkPlanDiagnostic {
  WorkPlanDiagnosticSeverity severity{WorkPlanDiagnosticSeverity::error};
  std::string code;
  std::string message;
  std::string json_pointer;
};

struct WorkPlanLoadResult {
  std::optional<FluxWorkPlan> plan;
  std::vector<WorkPlanDiagnostic> diagnostics;

  [[nodiscard]] bool has_errors() const;
  [[nodiscard]] bool ok() const { return plan.has_value() && !has_errors(); }
};

// Parses a bounded, strict JSON document and optionally binds it to an immutable board snapshot.
// Unknown schema fields and stale revisions fail closed. Step options are scalar values whose
// semantic support is checked separately by capability preflight before execution.
[[nodiscard]] WorkPlanLoadResult parse_flux_work_plan(std::string_view contents,
                                                      const Board* source = nullptr);

[[nodiscard]] WorkPlanLoadResult load_flux_work_plan_file(const std::filesystem::path& path,
                                                          const Board* source = nullptr);

// Emits a deterministic canonical JSON representation. Parsing and reserializing an unchanged
// plan is byte-stable.
[[nodiscard]] std::string serialize_flux_work_plan(const FluxWorkPlan& plan);

// The empty template infers no work. Callers must add explicit ordered steps before execution.
[[nodiscard]] FluxWorkPlan make_flux_work_plan_template(const Board& source,
                                                        std::string source_path);

[[nodiscard]] std::string_view work_plan_mode_name(WorkPlanMode mode);
[[nodiscard]] std::string_view work_plan_step_kind_name(WorkPlanStepKind kind);
[[nodiscard]] std::string_view work_plan_selector_scope_name(WorkPlanSelectorScope scope);

// Stable capability IDs required by the selected step kinds, in first-use order.
[[nodiscard]] std::vector<std::string>
required_flux_work_plan_capabilities(const FluxWorkPlan& plan);

} // namespace flux::io
