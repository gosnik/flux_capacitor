#pragma once

#include "flux/core/board.hpp"
#include "flux/core/placement_move_priority.hpp"

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

using WorkPlanOptionValue =
    std::variant<bool, std::int64_t, std::uint64_t, std::string, std::vector<std::string>>;

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
  // Present for file-backed loads with a source board after every declared policy has been
  // identity-checked and applied through its strict source-bound adapter.
  std::optional<Board> policy_board;
  // Non-board optimizer policy retained from the same one-read identity boundary.
  std::optional<PlacementMovePriorityPolicy> placement_move_priorities;
  std::vector<std::string> applied_policy_kinds;
  std::vector<WorkPlanDiagnostic> diagnostics;

  [[nodiscard]] bool has_errors() const;
  [[nodiscard]] bool ok() const { return plan.has_value() && !has_errors(); }
};

struct WorkPlanPolicyApplicationResult {
  std::optional<Board> board;
  std::optional<PlacementMovePriorityPolicy> placement_move_priorities;
  std::vector<std::string> applied_policy_kinds;
  std::vector<WorkPlanDiagnostic> diagnostics;

  [[nodiscard]] bool has_errors() const;
  [[nodiscard]] bool ok() const { return board.has_value() && !has_errors(); }
};

// One deterministic selector explanation. The field/value pair preserves why a target entered or
// left the scope; IDs remain canonical and are sorted independently of source serialization order.
struct WorkPlanSelectorEvidence {
  std::string field;
  std::string value;
  bool exclusion{false};
  std::vector<NetId> nets;
  std::vector<ObjectId> footprints;

  bool operator==(const WorkPlanSelectorEvidence&) const = default;
};

struct WorkPlanStepExpansion {
  std::string id;
  WorkPlanStepKind kind{WorkPlanStepKind::validation};
  std::vector<NetId> nets;
  std::vector<ObjectId> footprints;
  // Nets reached only through all_nets/include_globs. They may be planned but cannot authorize
  // removal of pre-existing copper in force-reroute-selected mode.
  std::vector<NetId> wildcard_only_nets;
  std::vector<WorkPlanSelectorEvidence> evidence;

  bool operator==(const WorkPlanStepExpansion&) const = default;
};

struct WorkPlanExistingCopperAuthority {
  std::vector<NetId> reroutable_nets;
  std::vector<ObjectId> removable_tracks;
  std::vector<ObjectId> removable_track_arcs;
  std::vector<ObjectId> removable_track_beziers;
  std::vector<ObjectId> removable_vias;
  std::vector<ObjectId> immutable_tracks;
  std::vector<ObjectId> immutable_track_arcs;
  std::vector<ObjectId> immutable_track_beziers;
  std::vector<ObjectId> immutable_vias;
  std::vector<ObjectId> immutable_zones;

  bool operator==(const WorkPlanExistingCopperAuthority&) const = default;
};

struct WorkPlanExpansionResult {
  std::vector<WorkPlanStepExpansion> steps;
  WorkPlanExistingCopperAuthority copper_authority;
  // Board-net-order masks are ready for one bounded device upload and reuse by later plan steps.
  std::vector<std::uint8_t> selected_net_mask;
  std::vector<std::uint8_t> reroutable_net_mask;
  std::vector<WorkPlanDiagnostic> diagnostics;

  [[nodiscard]] bool has_errors() const;
  [[nodiscard]] bool ok() const { return !has_errors(); }
};

// Parses a bounded, strict JSON document and optionally binds it to an immutable board snapshot.
// Unknown schema fields and stale revisions fail closed. Step options are bounded typed scalars or
// string lists whose semantic support is checked separately before execution.
[[nodiscard]] WorkPlanLoadResult parse_flux_work_plan(std::string_view contents,
                                                      const Board* source = nullptr);

// Relative policy paths resolve from the plan file's directory. Every bound regular file is
// streamed through SHA-256 under a fixed size bound before the loaded plan can be executable.
[[nodiscard]] WorkPlanLoadResult load_flux_work_plan_file(const std::filesystem::path& path,
                                                          const Board* source = nullptr);

// Reads every policy artifact once under a fixed bound, verifies its recorded SHA-256, and applies
// known policy kinds in dependency-safe canonical order. Unknown/duplicate kinds fail closed.
// Relative paths are resolved from policy_base_directory.
[[nodiscard]] WorkPlanPolicyApplicationResult
apply_flux_work_plan_policies(const Board& source, const FluxWorkPlan& plan,
                              const std::filesystem::path& policy_base_directory);

// Expands all selectors against one immutable canonical board. Selection is a union followed by
// glob exclusion. Footprint/group selectors project to nets only for explicit touching/internal
// scope. Locked copper and zones are always immutable, and wildcard-only matches never authorize
// rip-up of pre-existing copper.
[[nodiscard]] WorkPlanExpansionResult expand_flux_work_plan(const Board& source,
                                                            const FluxWorkPlan& plan);

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
