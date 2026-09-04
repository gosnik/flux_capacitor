#include "flux/io/work_plan.hpp"

#include <boost/json.hpp>

#include <algorithm>
#include <cctype>
#include <fstream>
#include <iterator>
#include <limits>
#include <set>
#include <string>
#include <type_traits>
#include <utility>

namespace flux::io {
namespace {

namespace json = boost::json;

constexpr std::size_t maximum_document_bytes = 4U * 1024U * 1024U;
constexpr std::size_t maximum_steps = 1'024U;
constexpr std::size_t maximum_policies = 256U;
constexpr std::size_t maximum_selector_values = 100'000U;
constexpr std::size_t maximum_options_per_step = 4'096U;
constexpr std::size_t maximum_string_bytes = 16U * 1024U;

struct Loader {
  WorkPlanLoadResult result;

  void error(std::string code, std::string message, std::string pointer = {}) {
    result.diagnostics.push_back({WorkPlanDiagnosticSeverity::error, std::move(code),
                                  std::move(message), std::move(pointer)});
  }
};

[[nodiscard]] std::string pointer_child(std::string_view base, std::string_view child) {
  std::string result{base};
  result.push_back('/');
  for (const char character : child) {
    if (character == '~') {
      result += "~0";
    } else if (character == '/') {
      result += "~1";
    } else {
      result.push_back(character);
    }
  }
  return result;
}

void reject_unknown(const json::object& object, const std::set<std::string_view>& allowed,
                    Loader& loader, std::string_view pointer) {
  for (const auto& member : object) {
    if (!allowed.contains(member.key())) {
      loader.error("unknown_work_plan_field", "work plan contains an unknown field",
                   pointer_child(pointer, member.key()));
    }
  }
}

[[nodiscard]] const json::value* required(const json::object& object, std::string_view name,
                                          Loader& loader, std::string_view pointer) {
  const auto* value = object.if_contains(name);
  if (value == nullptr) {
    loader.error("missing_work_plan_field",
                 "work plan is missing required field " + std::string(name),
                 pointer_child(pointer, name));
  }
  return value;
}

[[nodiscard]] std::optional<std::string> parse_string(const json::value* value, Loader& loader,
                                                      std::string_view pointer, bool allow_empty) {
  if (value == nullptr) {
    return std::nullopt;
  }
  if (!value->is_string()) {
    loader.error("invalid_work_plan_string", "work plan field must be a JSON string",
                 std::string(pointer));
    return std::nullopt;
  }
  const auto& source = value->as_string();
  if ((!allow_empty && source.empty()) || source.size() > maximum_string_bytes) {
    loader.error("invalid_work_plan_string", "work plan string is empty or exceeds its bound",
                 std::string(pointer));
    return std::nullopt;
  }
  return std::string(source);
}

[[nodiscard]] std::optional<bool> parse_bool(const json::value* value, Loader& loader,
                                             std::string_view pointer) {
  if (value == nullptr) {
    return std::nullopt;
  }
  if (!value->is_bool()) {
    loader.error("invalid_work_plan_boolean", "work plan field must be a JSON boolean",
                 std::string(pointer));
    return std::nullopt;
  }
  return value->as_bool();
}

[[nodiscard]] std::optional<std::uint64_t>
parse_unsigned(const json::value* value, Loader& loader, std::string_view pointer,
               std::uint64_t maximum = std::numeric_limits<std::uint64_t>::max()) {
  if (value == nullptr) {
    return std::nullopt;
  }
  std::optional<std::uint64_t> result;
  if (value->is_uint64()) {
    result = value->as_uint64();
  } else if (value->is_int64() && value->as_int64() >= 0) {
    result = static_cast<std::uint64_t>(value->as_int64());
  }
  if (!result || *result > maximum) {
    loader.error("invalid_work_plan_integer",
                 "work plan field must be a bounded non-negative JSON integer",
                 std::string(pointer));
    return std::nullopt;
  }
  return result;
}

[[nodiscard]] std::vector<std::string> parse_string_array(const json::value* value, Loader& loader,
                                                          std::string_view pointer) {
  std::vector<std::string> result;
  if (value == nullptr) {
    return result;
  }
  if (!value->is_array()) {
    loader.error("invalid_work_plan_array", "work plan field must be a JSON array",
                 std::string(pointer));
    return result;
  }
  const auto& array = value->as_array();
  if (array.size() > maximum_selector_values) {
    loader.error("work_plan_budget_exceeded", "work plan array exceeds its bounded size",
                 std::string(pointer));
    return result;
  }
  std::set<std::string> seen;
  result.reserve(array.size());
  for (std::size_t index = 0U; index < array.size(); ++index) {
    const auto item_pointer = pointer_child(pointer, std::to_string(index));
    auto item = parse_string(&array[index], loader, item_pointer, false);
    if (item && !seen.insert(*item).second) {
      loader.error("duplicate_work_plan_value", "work plan array values must be unique",
                   item_pointer);
    } else if (item) {
      result.push_back(std::move(*item));
    }
  }
  return result;
}

[[nodiscard]] std::optional<WorkPlanMode> parse_mode(std::string_view value) {
  if (value == "preview")
    return WorkPlanMode::preview;
  if (value == "new-routing")
    return WorkPlanMode::new_routing;
  if (value == "force-reroute-selected")
    return WorkPlanMode::force_reroute_selected;
  if (value == "allowlisted-ripup")
    return WorkPlanMode::allowlisted_ripup;
  return std::nullopt;
}

[[nodiscard]] std::optional<WorkPlanStepKind> parse_step_kind(std::string_view value) {
  constexpr std::array values{
      std::pair{"route", WorkPlanStepKind::route},
      std::pair{"fanout", WorkPlanStepKind::fanout},
      std::pair{"differential-pair", WorkPlanStepKind::differential_pair},
      std::pair{"bus", WorkPlanStepKind::bus},
      std::pair{"plane", WorkPlanStepKind::plane},
      std::pair{"tuning", WorkPlanStepKind::tuning},
      std::pair{"cleanup", WorkPlanStepKind::cleanup},
      std::pair{"validation", WorkPlanStepKind::validation},
      std::pair{"placement", WorkPlanStepKind::placement},
  };
  const auto found = std::find_if(values.begin(), values.end(),
                                  [value](const auto& entry) { return entry.first == value; });
  return found == values.end() ? std::nullopt : std::optional<WorkPlanStepKind>{found->second};
}

[[nodiscard]] std::optional<WorkPlanSelectorScope> parse_scope(std::string_view value) {
  if (value == "selected")
    return WorkPlanSelectorScope::selected;
  if (value == "touching")
    return WorkPlanSelectorScope::touching;
  if (value == "internal")
    return WorkPlanSelectorScope::internal;
  return std::nullopt;
}

[[nodiscard]] WorkPlanSelectors parse_selectors(const json::value* value, Loader& loader,
                                                std::string_view pointer) {
  WorkPlanSelectors result;
  if (value == nullptr || !value->is_object()) {
    loader.error("invalid_work_plan_selectors", "selectors must be a JSON object",
                 std::string(pointer));
    return result;
  }
  const auto& object = value->as_object();
  reject_unknown(object,
                 {"all_nets", "exact_nets", "include_globs", "exclude_globs", "net_classes",
                  "footprints", "placement_groups", "group_scope"},
                 loader, pointer);
  if (const auto parsed = parse_bool(required(object, "all_nets", loader, pointer), loader,
                                     pointer_child(pointer, "all_nets"))) {
    result.all_nets = *parsed;
  }
  result.exact_nets = parse_string_array(required(object, "exact_nets", loader, pointer), loader,
                                         pointer_child(pointer, "exact_nets"));
  result.include_globs = parse_string_array(required(object, "include_globs", loader, pointer),
                                            loader, pointer_child(pointer, "include_globs"));
  result.exclude_globs = parse_string_array(required(object, "exclude_globs", loader, pointer),
                                            loader, pointer_child(pointer, "exclude_globs"));
  result.net_classes = parse_string_array(required(object, "net_classes", loader, pointer), loader,
                                          pointer_child(pointer, "net_classes"));
  result.footprints = parse_string_array(required(object, "footprints", loader, pointer), loader,
                                         pointer_child(pointer, "footprints"));
  result.placement_groups =
      parse_string_array(required(object, "placement_groups", loader, pointer), loader,
                         pointer_child(pointer, "placement_groups"));
  const auto scope_pointer = pointer_child(pointer, "group_scope");
  if (const auto text = parse_string(required(object, "group_scope", loader, pointer), loader,
                                     scope_pointer, false)) {
    if (const auto parsed = parse_scope(*text)) {
      result.group_scope = *parsed;
    } else {
      loader.error("invalid_work_plan_selector_scope",
                   "group_scope must be selected, touching, or internal", scope_pointer);
    }
  }
  return result;
}

[[nodiscard]] std::map<std::string, WorkPlanOptionValue>
parse_options(const json::value* value, Loader& loader, std::string_view pointer) {
  std::map<std::string, WorkPlanOptionValue> result;
  if (value == nullptr || !value->is_object()) {
    loader.error("invalid_work_plan_options", "options must be a JSON object",
                 std::string(pointer));
    return result;
  }
  const auto& object = value->as_object();
  if (object.size() > maximum_options_per_step) {
    loader.error("work_plan_budget_exceeded", "step option count exceeds its bound",
                 std::string(pointer));
    return result;
  }
  for (const auto& member : object) {
    const auto member_pointer = pointer_child(pointer, member.key());
    if (member.key().empty() || member.key().size() > 256U) {
      loader.error("invalid_work_plan_option", "option name is empty or too long", member_pointer);
      continue;
    }
    const auto& option = member.value();
    if (option.is_bool()) {
      result.emplace(member.key(), option.as_bool());
    } else if (option.is_int64() && option.as_int64() >= 0) {
      result.emplace(member.key(), static_cast<std::uint64_t>(option.as_int64()));
    } else if (option.is_int64()) {
      result.emplace(member.key(), option.as_int64());
    } else if (option.is_uint64()) {
      result.emplace(member.key(), option.as_uint64());
    } else if (option.is_string() && option.as_string().size() <= maximum_string_bytes) {
      result.emplace(member.key(), std::string(option.as_string()));
    } else if (option.is_array()) {
      result.emplace(member.key(), parse_string_array(&option, loader, member_pointer));
    } else {
      loader.error("invalid_work_plan_option",
                   "step options must be bounded boolean, integer, string, or string-list values",
                   member_pointer);
    }
  }
  return result;
}

[[nodiscard]] bool valid_sha256(std::string_view value) {
  return value.size() == 64U && std::all_of(value.begin(), value.end(), [](const char character) {
           return (character >= '0' && character <= '9') || (character >= 'a' && character <= 'f');
         });
}

[[nodiscard]] bool selectors_empty(const WorkPlanSelectors& selectors) {
  return !selectors.all_nets && selectors.exact_nets.empty() && selectors.include_globs.empty() &&
         selectors.net_classes.empty() && selectors.footprints.empty() &&
         selectors.placement_groups.empty();
}

void validate_semantics(const FluxWorkPlan& plan, const Board* source, Loader& loader) {
  if (plan.source_path.empty()) {
    loader.error("invalid_work_plan_source", "source path must not be empty", "/source/path");
  }
  if (plan.source_revision.empty()) {
    loader.error("invalid_work_plan_source", "source revision must not be empty",
                 "/source/revision");
  }
  if (source != nullptr && source->source_revision != plan.source_revision) {
    loader.error("stale_work_plan", "work plan targets a different board revision",
                 "/source/revision");
  }
  if (plan.mode == WorkPlanMode::allowlisted_ripup && plan.ripup_allowlist.empty()) {
    loader.error("empty_work_plan_ripup_allowlist",
                 "allowlisted-ripup mode requires at least one exact net name", "/ripup_allowlist");
  }
  if (plan.mode != WorkPlanMode::allowlisted_ripup && !plan.ripup_allowlist.empty()) {
    loader.error("unexpected_work_plan_ripup_allowlist",
                 "ripup_allowlist is legal only in allowlisted-ripup mode", "/ripup_allowlist");
  }
  std::set<std::string> policy_kinds;
  for (std::size_t index = 0U; index < plan.policies.size(); ++index) {
    const auto& policy = plan.policies[index];
    const auto pointer = "/policies/" + std::to_string(index);
    if (policy.kind.empty() || policy.path.empty() || !valid_sha256(policy.sha256)) {
      loader.error("invalid_work_plan_policy_identity",
                   "policy kind/path must be non-empty and sha256 must be lowercase hexadecimal",
                   pointer);
    }
    if (policy.source_revision != plan.source_revision) {
      loader.error("stale_work_plan_policy",
                   "every policy identity must target the plan source revision", pointer);
    }
    if (!policy_kinds.insert(policy.kind).second) {
      loader.error("duplicate_work_plan_policy_kind",
                   "a work plan may bind at most one artifact of each policy kind", pointer);
    }
  }
  if (plan.telemetry.listen.empty() || plan.telemetry.rate_hz == 0U ||
      plan.telemetry.rate_hz > 1'000U ||
      (plan.telemetry.level != "normal" && plan.telemetry.level != "diagnostic" &&
       plan.telemetry.level != "trace")) {
    loader.error("invalid_work_plan_telemetry",
                 "telemetry requires a listen endpoint, 1..1000 Hz, and a known level",
                 "/telemetry");
  }

  std::set<std::string> step_ids;
  bool selected_routing_step = false;
  for (std::size_t index = 0U; index < plan.steps.size(); ++index) {
    const auto& step = plan.steps[index];
    const auto pointer = "/steps/" + std::to_string(index);
    if (step.id.empty() || step.id.size() > 256U || !step_ids.insert(step.id).second) {
      loader.error("invalid_work_plan_step_id", "step IDs must be non-empty, bounded, and unique",
                   pointer_child(pointer, "id"));
    }
    for (const auto& dependency : step.depends_on) {
      if (!step_ids.contains(dependency) || dependency == step.id) {
        loader.error("invalid_work_plan_dependency",
                     "a dependency must name a unique preceding step",
                     pointer_child(pointer, "depends_on"));
      }
    }
    if (step.kind == WorkPlanStepKind::route && !selectors_empty(step.selectors)) {
      selected_routing_step = true;
    }
  }
  if (plan.mode == WorkPlanMode::force_reroute_selected && !selected_routing_step) {
    loader.error("empty_force_reroute_scope",
                 "force-reroute-selected mode requires an explicitly selected route step",
                 "/steps");
  }
}

[[nodiscard]] json::array string_array(const std::vector<std::string>& values) {
  json::array result;
  result.reserve(values.size());
  for (const auto& value : values) {
    result.emplace_back(value);
  }
  return result;
}

[[nodiscard]] json::object selectors_json(const WorkPlanSelectors& selectors) {
  json::object result;
  result["all_nets"] = selectors.all_nets;
  result["exact_nets"] = string_array(selectors.exact_nets);
  result["include_globs"] = string_array(selectors.include_globs);
  result["exclude_globs"] = string_array(selectors.exclude_globs);
  result["net_classes"] = string_array(selectors.net_classes);
  result["footprints"] = string_array(selectors.footprints);
  result["placement_groups"] = string_array(selectors.placement_groups);
  result["group_scope"] = work_plan_selector_scope_name(selectors.group_scope);
  return result;
}

[[nodiscard]] json::object options_json(const std::map<std::string, WorkPlanOptionValue>& options) {
  json::object result;
  for (const auto& [name, value] : options) {
    std::visit(
        [&](const auto& typed) {
          using Value = std::decay_t<decltype(typed)>;
          if constexpr (std::is_same_v<Value, std::vector<std::string>>) {
            result[name] = string_array(typed);
          } else {
            result[name] = typed;
          }
        },
        value);
  }
  return result;
}

[[nodiscard]] bool glob_matches(std::string_view pattern, std::string_view value) {
  std::size_t pattern_index = 0U;
  std::size_t value_index = 0U;
  std::size_t star_index = std::string_view::npos;
  std::size_t star_value_index = 0U;
  while (value_index < value.size()) {
    if (pattern_index < pattern.size() &&
        (pattern[pattern_index] == '?' || pattern[pattern_index] == value[value_index])) {
      ++pattern_index;
      ++value_index;
    } else if (pattern_index < pattern.size() && pattern[pattern_index] == '*') {
      star_index = pattern_index++;
      star_value_index = value_index;
    } else if (star_index != std::string_view::npos) {
      pattern_index = star_index + 1U;
      value_index = ++star_value_index;
    } else {
      return false;
    }
  }
  while (pattern_index < pattern.size() && pattern[pattern_index] == '*') {
    ++pattern_index;
  }
  return pattern_index == pattern.size();
}

template <typename IdType>
[[nodiscard]] std::vector<IdType> sorted_ids(const std::set<IdType>& ids) {
  return {ids.begin(), ids.end()};
}

void expansion_error(WorkPlanExpansionResult& result, std::string code, std::string message,
                     std::string pointer) {
  result.diagnostics.push_back(
      {WorkPlanDiagnosticSeverity::error, std::move(code), std::move(message), std::move(pointer)});
}

[[nodiscard]] bool targets_routing_nets(WorkPlanStepKind kind) {
  return kind == WorkPlanStepKind::route || kind == WorkPlanStepKind::fanout ||
         kind == WorkPlanStepKind::differential_pair || kind == WorkPlanStepKind::bus ||
         kind == WorkPlanStepKind::plane || kind == WorkPlanStepKind::tuning;
}

[[nodiscard]] bool requires_atomic_coupled_scope(WorkPlanStepKind kind) {
  return kind == WorkPlanStepKind::route || kind == WorkPlanStepKind::differential_pair ||
         kind == WorkPlanStepKind::bus || kind == WorkPlanStepKind::tuning;
}

} // namespace

bool WorkPlanLoadResult::has_errors() const {
  return std::any_of(diagnostics.begin(), diagnostics.end(), [](const auto& diagnostic) {
    return diagnostic.severity == WorkPlanDiagnosticSeverity::error;
  });
}

bool WorkPlanExpansionResult::has_errors() const {
  return std::any_of(diagnostics.begin(), diagnostics.end(), [](const auto& diagnostic) {
    return diagnostic.severity == WorkPlanDiagnosticSeverity::error;
  });
}

std::string_view work_plan_mode_name(WorkPlanMode mode) {
  switch (mode) {
  case WorkPlanMode::preview:
    return "preview";
  case WorkPlanMode::new_routing:
    return "new-routing";
  case WorkPlanMode::force_reroute_selected:
    return "force-reroute-selected";
  case WorkPlanMode::allowlisted_ripup:
    return "allowlisted-ripup";
  }
  return "unknown";
}

std::string_view work_plan_step_kind_name(WorkPlanStepKind kind) {
  switch (kind) {
  case WorkPlanStepKind::route:
    return "route";
  case WorkPlanStepKind::fanout:
    return "fanout";
  case WorkPlanStepKind::differential_pair:
    return "differential-pair";
  case WorkPlanStepKind::bus:
    return "bus";
  case WorkPlanStepKind::plane:
    return "plane";
  case WorkPlanStepKind::tuning:
    return "tuning";
  case WorkPlanStepKind::cleanup:
    return "cleanup";
  case WorkPlanStepKind::validation:
    return "validation";
  case WorkPlanStepKind::placement:
    return "placement";
  }
  return "unknown";
}

std::string_view work_plan_selector_scope_name(WorkPlanSelectorScope scope) {
  switch (scope) {
  case WorkPlanSelectorScope::selected:
    return "selected";
  case WorkPlanSelectorScope::touching:
    return "touching";
  case WorkPlanSelectorScope::internal:
    return "internal";
  }
  return "unknown";
}

WorkPlanLoadResult parse_flux_work_plan(std::string_view contents, const Board* source) {
  Loader loader;
  if (contents.size() > maximum_document_bytes) {
    loader.error("work_plan_budget_exceeded", "work plan exceeds the 4 MiB document bound");
    return std::move(loader.result);
  }
  json::parse_options options;
  options.max_depth = 64U;
  boost::system::error_code error;
  auto root = json::parse(contents, error, {}, options);
  if (error) {
    loader.error("invalid_work_plan_json", error.message());
    return std::move(loader.result);
  }
  if (!root.is_object()) {
    loader.error("invalid_work_plan_root", "work plan root must be a JSON object", "");
    return std::move(loader.result);
  }
  const auto& object = root.as_object();
  reject_unknown(object,
                 {"schema", "schema_version", "source", "mode", "ripup_allowlist", "policies",
                  "telemetry", "steps"},
                 loader, "");
  const auto schema =
      parse_string(required(object, "schema", loader, ""), loader, "/schema", false);
  if (schema && *schema != "flux_work_plan") {
    loader.error("invalid_work_plan_schema", "schema must be flux_work_plan", "/schema");
  }
  const auto version = parse_unsigned(required(object, "schema_version", loader, ""), loader,
                                      "/schema_version", UINT32_MAX);
  if (version && *version != flux_work_plan_schema_version) {
    loader.error("unsupported_work_plan_version", "only FluxWorkPlan schema version 1 is supported",
                 "/schema_version");
  }

  FluxWorkPlan plan;
  if (version) {
    plan.schema_version = static_cast<std::uint32_t>(*version);
  }
  const auto* source_value = required(object, "source", loader, "");
  if (source_value == nullptr || !source_value->is_object()) {
    loader.error("invalid_work_plan_source", "source must be a JSON object", "/source");
  } else {
    const auto& source_object = source_value->as_object();
    reject_unknown(source_object, {"path", "revision"}, loader, "/source");
    if (auto value = parse_string(required(source_object, "path", loader, "/source"), loader,
                                  "/source/path", false)) {
      plan.source_path = std::move(*value);
    }
    if (auto value = parse_string(required(source_object, "revision", loader, "/source"), loader,
                                  "/source/revision", false)) {
      plan.source_revision = std::move(*value);
    }
  }
  if (const auto text =
          parse_string(required(object, "mode", loader, ""), loader, "/mode", false)) {
    if (const auto parsed = parse_mode(*text)) {
      plan.mode = *parsed;
    } else {
      loader.error("invalid_work_plan_mode", "work plan mode is not supported", "/mode");
    }
  }
  plan.ripup_allowlist = parse_string_array(required(object, "ripup_allowlist", loader, ""), loader,
                                            "/ripup_allowlist");

  const auto* policies = required(object, "policies", loader, "");
  if (policies == nullptr || !policies->is_array()) {
    loader.error("invalid_work_plan_policies", "policies must be a JSON array", "/policies");
  } else if (policies->as_array().size() > maximum_policies) {
    loader.error("work_plan_budget_exceeded", "policy count exceeds its bound", "/policies");
  } else {
    const auto& array = policies->as_array();
    for (std::size_t index = 0U; index < array.size(); ++index) {
      const auto pointer = "/policies/" + std::to_string(index);
      if (!array[index].is_object()) {
        loader.error("invalid_work_plan_policy_identity", "policy identity must be an object",
                     pointer);
        continue;
      }
      const auto& policy_object = array[index].as_object();
      reject_unknown(policy_object, {"kind", "path", "sha256", "source_revision"}, loader, pointer);
      WorkPlanPolicyIdentity policy;
      if (auto value = parse_string(required(policy_object, "kind", loader, pointer), loader,
                                    pointer_child(pointer, "kind"), false))
        policy.kind = std::move(*value);
      if (auto value = parse_string(required(policy_object, "path", loader, pointer), loader,
                                    pointer_child(pointer, "path"), false))
        policy.path = std::move(*value);
      if (auto value = parse_string(required(policy_object, "sha256", loader, pointer), loader,
                                    pointer_child(pointer, "sha256"), false))
        policy.sha256 = std::move(*value);
      if (auto value = parse_string(required(policy_object, "source_revision", loader, pointer),
                                    loader, pointer_child(pointer, "source_revision"), false))
        policy.source_revision = std::move(*value);
      plan.policies.push_back(std::move(policy));
    }
  }

  const auto* telemetry = required(object, "telemetry", loader, "");
  if (telemetry == nullptr || !telemetry->is_object()) {
    loader.error("invalid_work_plan_telemetry", "telemetry must be a JSON object", "/telemetry");
  } else {
    const auto& telemetry_object = telemetry->as_object();
    reject_unknown(telemetry_object, {"enabled", "listen", "rate_hz", "level", "record"}, loader,
                   "/telemetry");
    if (const auto parsed = parse_bool(required(telemetry_object, "enabled", loader, "/telemetry"),
                                       loader, "/telemetry/enabled"))
      plan.telemetry.enabled = *parsed;
    if (auto value = parse_string(required(telemetry_object, "listen", loader, "/telemetry"),
                                  loader, "/telemetry/listen", false))
      plan.telemetry.listen = std::move(*value);
    if (const auto parsed =
            parse_unsigned(required(telemetry_object, "rate_hz", loader, "/telemetry"), loader,
                           "/telemetry/rate_hz", 1'000U))
      plan.telemetry.rate_hz = static_cast<std::uint32_t>(*parsed);
    if (auto value = parse_string(required(telemetry_object, "level", loader, "/telemetry"), loader,
                                  "/telemetry/level", false))
      plan.telemetry.level = std::move(*value);
    const auto* record = required(telemetry_object, "record", loader, "/telemetry");
    if (record != nullptr && !record->is_null()) {
      if (auto value = parse_string(record, loader, "/telemetry/record", false))
        plan.telemetry.record = std::move(*value);
    }
  }

  const auto* steps = required(object, "steps", loader, "");
  if (steps == nullptr || !steps->is_array()) {
    loader.error("invalid_work_plan_steps", "steps must be a JSON array", "/steps");
  } else if (steps->as_array().size() > maximum_steps) {
    loader.error("work_plan_budget_exceeded", "step count exceeds its bound", "/steps");
  } else {
    const auto& array = steps->as_array();
    for (std::size_t index = 0U; index < array.size(); ++index) {
      const auto pointer = "/steps/" + std::to_string(index);
      if (!array[index].is_object()) {
        loader.error("invalid_work_plan_step", "work plan step must be a JSON object", pointer);
        continue;
      }
      const auto& step_object = array[index].as_object();
      reject_unknown(step_object, {"id", "kind", "depends_on", "selectors", "options"}, loader,
                     pointer);
      WorkPlanStep step;
      if (auto value = parse_string(required(step_object, "id", loader, pointer), loader,
                                    pointer_child(pointer, "id"), false))
        step.id = std::move(*value);
      const auto kind_pointer = pointer_child(pointer, "kind");
      if (const auto text = parse_string(required(step_object, "kind", loader, pointer), loader,
                                         kind_pointer, false)) {
        if (const auto parsed = parse_step_kind(*text)) {
          step.kind = *parsed;
        } else {
          loader.error("invalid_work_plan_step_kind", "work plan step kind is not supported",
                       kind_pointer);
        }
      }
      step.depends_on = parse_string_array(required(step_object, "depends_on", loader, pointer),
                                           loader, pointer_child(pointer, "depends_on"));
      step.selectors = parse_selectors(required(step_object, "selectors", loader, pointer), loader,
                                       pointer_child(pointer, "selectors"));
      step.options = parse_options(required(step_object, "options", loader, pointer), loader,
                                   pointer_child(pointer, "options"));
      plan.steps.push_back(std::move(step));
    }
  }

  validate_semantics(plan, source, loader);
  if (!loader.result.has_errors()) {
    loader.result.plan = std::move(plan);
  }
  return std::move(loader.result);
}

WorkPlanLoadResult load_flux_work_plan_file(const std::filesystem::path& path,
                                            const Board* source) {
  std::ifstream input(path, std::ios::binary);
  if (!input) {
    WorkPlanLoadResult result;
    result.diagnostics.push_back({WorkPlanDiagnosticSeverity::error, "work_plan_read_failed",
                                  "unable to read FluxWorkPlan file", ""});
    return result;
  }
  std::string contents{std::istreambuf_iterator<char>{input}, std::istreambuf_iterator<char>{}};
  auto result = parse_flux_work_plan(contents, source);
  if (!result.plan) {
    return result;
  }
  if (source != nullptr) {
    auto policies = apply_flux_work_plan_policies(*source, *result.plan, path.parent_path());
    result.policy_board = std::move(policies.board);
    result.applied_policy_kinds = std::move(policies.applied_policy_kinds);
    result.diagnostics.insert(result.diagnostics.end(),
                              std::make_move_iterator(policies.diagnostics.begin()),
                              std::make_move_iterator(policies.diagnostics.end()));
  }
  return result;
}

WorkPlanExpansionResult expand_flux_work_plan(const Board& source, const FluxWorkPlan& plan) {
  WorkPlanExpansionResult result;
  Loader semantic_loader;
  validate_semantics(plan, &source, semantic_loader);
  result.diagnostics = std::move(semantic_loader.result.diagnostics);

  const auto net_by_name = [&source](std::string_view name) -> const Net* {
    const auto found = std::find_if(source.nets.begin(), source.nets.end(), [name](const Net& net) {
      return !net.name.empty() && net.name == name;
    });
    return found == source.nets.end() ? nullptr : &*found;
  };
  const auto named_nets = [&source]() {
    std::set<NetId> nets;
    for (const auto& net : source.nets) {
      if (!net.name.empty()) {
        nets.insert(net.id);
      }
    }
    return nets;
  }();

  std::set<NetId> force_reroute_authority;
  for (std::size_t step_index = 0U; step_index < plan.steps.size(); ++step_index) {
    const auto& step = plan.steps[step_index];
    const auto step_pointer = "/steps/" + std::to_string(step_index) + "/selectors";
    WorkPlanStepExpansion expanded;
    expanded.id = step.id;
    expanded.kind = step.kind;
    std::set<NetId> selected_nets;
    std::set<NetId> non_wildcard_nets;
    std::set<ObjectId> selected_footprints;

    const auto record = [&expanded](std::string field, std::string value, bool exclusion,
                                    const std::set<NetId>& nets,
                                    const std::set<ObjectId>& footprints = {}) {
      expanded.evidence.push_back({std::move(field), std::move(value), exclusion, sorted_ids(nets),
                                   sorted_ids(footprints)});
    };
    if (step.selectors.all_nets) {
      selected_nets.insert(named_nets.begin(), named_nets.end());
      record("all_nets", "true", false, named_nets);
    }
    for (std::size_t index = 0U; index < step.selectors.exact_nets.size(); ++index) {
      const auto& name = step.selectors.exact_nets[index];
      const auto* net = net_by_name(name);
      if (net == nullptr) {
        expansion_error(result, "unknown_work_plan_net", "exact_nets names an unknown source net",
                        step_pointer + "/exact_nets/" + std::to_string(index));
        continue;
      }
      const std::set<NetId> match{net->id};
      selected_nets.insert(net->id);
      non_wildcard_nets.insert(net->id);
      record("exact_nets", name, false, match);
    }
    for (std::size_t index = 0U; index < step.selectors.include_globs.size(); ++index) {
      const auto& pattern = step.selectors.include_globs[index];
      std::set<NetId> matches;
      for (const auto& net : source.nets) {
        if (!net.name.empty() && glob_matches(pattern, net.name)) {
          matches.insert(net.id);
        }
      }
      if (matches.empty()) {
        expansion_error(result, "empty_work_plan_glob",
                        "include_globs pattern matches no named net",
                        step_pointer + "/include_globs/" + std::to_string(index));
      }
      selected_nets.insert(matches.begin(), matches.end());
      record("include_globs", pattern, false, matches);
    }
    for (std::size_t index = 0U; index < step.selectors.net_classes.size(); ++index) {
      const auto& selector = step.selectors.net_classes[index];
      std::set<NetClassId> classes;
      for (const auto& net_class : source.net_classes) {
        if (net_class.name == selector || net_class.id.value() == selector) {
          classes.insert(net_class.id);
        }
      }
      if (classes.size() != 1U) {
        expansion_error(result, "unknown_or_ambiguous_work_plan_net_class",
                        "net_classes must identify exactly one source class",
                        step_pointer + "/net_classes/" + std::to_string(index));
        continue;
      }
      std::set<NetId> matches;
      for (const auto& net : source.nets) {
        if (!net.name.empty() &&
            std::any_of(net.classes.begin(), net.classes.end(),
                        [&classes](const NetClassId& id) { return classes.contains(id); })) {
          matches.insert(net.id);
        }
      }
      if (matches.empty()) {
        expansion_error(result, "empty_work_plan_net_class",
                        "selected net class has no named source-net members",
                        step_pointer + "/net_classes/" + std::to_string(index));
      }
      selected_nets.insert(matches.begin(), matches.end());
      non_wildcard_nets.insert(matches.begin(), matches.end());
      record("net_classes", selector, false, matches);
    }

    for (std::size_t index = 0U; index < step.selectors.footprints.size(); ++index) {
      const auto& selector = step.selectors.footprints[index];
      std::set<ObjectId> matches;
      for (const auto& footprint : source.footprints) {
        if (footprint.reference == selector || footprint.id.value() == selector) {
          matches.insert(footprint.id);
        }
      }
      if (matches.size() != 1U) {
        expansion_error(result, "unknown_or_ambiguous_work_plan_footprint",
                        "footprints must identify exactly one source footprint",
                        step_pointer + "/footprints/" + std::to_string(index));
        continue;
      }
      selected_footprints.insert(matches.begin(), matches.end());
      record("footprints", selector, false, {}, matches);
    }
    for (std::size_t index = 0U; index < step.selectors.placement_groups.size(); ++index) {
      const auto& selector = step.selectors.placement_groups[index];
      std::vector<const PlacementGroup*> matches;
      for (const auto& group : source.placement_groups) {
        if (group.name == selector || group.id.value() == selector) {
          matches.push_back(&group);
        }
      }
      if (matches.size() != 1U) {
        expansion_error(result, "unknown_or_ambiguous_work_plan_placement_group",
                        "placement_groups must identify exactly one source placement group",
                        step_pointer + "/placement_groups/" + std::to_string(index));
        continue;
      }
      std::set<ObjectId> group_footprints(matches.front()->footprints.begin(),
                                          matches.front()->footprints.end());
      selected_footprints.insert(group_footprints.begin(), group_footprints.end());
      record("placement_groups", selector, false, {}, group_footprints);
    }

    if (!selected_footprints.empty() &&
        step.selectors.group_scope != WorkPlanSelectorScope::selected) {
      std::map<NetId, std::pair<std::size_t, std::size_t>> terminal_counts;
      for (const auto& footprint : source.footprints) {
        for (const auto& pad : footprint.pads) {
          const auto net =
              std::find_if(source.nets.begin(), source.nets.end(),
                           [&pad](const Net& n) { return n.id == pad.net && !n.name.empty(); });
          if (net == source.nets.end()) {
            continue;
          }
          auto& [total, inside] = terminal_counts[pad.net];
          ++total;
          if (selected_footprints.contains(footprint.id)) {
            ++inside;
          }
        }
      }
      std::set<NetId> matches;
      for (const auto& [net, counts] : terminal_counts) {
        const auto [total, inside] = counts;
        if (inside > 0U &&
            (step.selectors.group_scope == WorkPlanSelectorScope::touching || inside == total)) {
          matches.insert(net);
        }
      }
      selected_nets.insert(matches.begin(), matches.end());
      non_wildcard_nets.insert(matches.begin(), matches.end());
      record("group_scope", std::string(work_plan_selector_scope_name(step.selectors.group_scope)),
             false, matches, selected_footprints);
    }

    for (std::size_t index = 0U; index < step.selectors.exclude_globs.size(); ++index) {
      const auto& pattern = step.selectors.exclude_globs[index];
      std::set<NetId> matches;
      for (const auto& net : source.nets) {
        if (!net.name.empty() && glob_matches(pattern, net.name)) {
          matches.insert(net.id);
        }
      }
      if (matches.empty()) {
        expansion_error(result, "empty_work_plan_glob",
                        "exclude_globs pattern matches no named net",
                        step_pointer + "/exclude_globs/" + std::to_string(index));
      }
      for (const auto& net : matches) {
        selected_nets.erase(net);
        non_wildcard_nets.erase(net);
      }
      record("exclude_globs", pattern, true, matches);
    }

    if (targets_routing_nets(step.kind) && selected_nets.empty()) {
      expansion_error(result, "empty_work_plan_net_scope",
                      "this work-plan step requires at least one explicitly selected named net",
                      step_pointer);
    }
    if (targets_routing_nets(step.kind) && !selected_footprints.empty() &&
        step.selectors.group_scope == WorkPlanSelectorScope::selected && selected_nets.empty()) {
      expansion_error(result, "unprojected_work_plan_footprints",
                      "a routing step must use touching or internal to project footprints to nets",
                      step_pointer + "/group_scope");
    }
    if (step.kind == WorkPlanStepKind::placement && selected_footprints.empty()) {
      expansion_error(result, "empty_work_plan_footprint_scope",
                      "a placement step requires at least one selected footprint", step_pointer);
    }
    if (requires_atomic_coupled_scope(step.kind) && !selected_nets.empty()) {
      const auto partially_selected = [&selected_nets](const std::vector<NetId>& members) {
        const auto selected =
            std::count_if(members.begin(), members.end(),
                          [&selected_nets](NetId id) { return selected_nets.contains(id); });
        return selected != 0 && static_cast<std::size_t>(selected) != members.size();
      };
      for (const auto& pair : source.differential_pairs) {
        if (partially_selected({pair.positive_net, pair.negative_net})) {
          expansion_error(result, "split_work_plan_differential_pair",
                          "selector expansion splits a declared differential pair", step_pointer);
        }
      }
      for (const auto& bus : source.routing_buses) {
        if (partially_selected(bus.ordered_nets)) {
          expansion_error(result, "split_work_plan_routing_bus",
                          "selector expansion splits a declared routing bus", step_pointer);
        }
      }
    }

    expanded.nets = sorted_ids(selected_nets);
    expanded.footprints = sorted_ids(selected_footprints);
    std::set<NetId> wildcard_only;
    std::set_difference(selected_nets.begin(), selected_nets.end(), non_wildcard_nets.begin(),
                        non_wildcard_nets.end(), std::inserter(wildcard_only, wildcard_only.end()));
    expanded.wildcard_only_nets = sorted_ids(wildcard_only);
    if (step.kind == WorkPlanStepKind::route) {
      force_reroute_authority.insert(non_wildcard_nets.begin(), non_wildcard_nets.end());
    }
    result.steps.push_back(std::move(expanded));
  }

  std::set<NetId> reroutable_nets;
  if (plan.mode == WorkPlanMode::force_reroute_selected) {
    reroutable_nets = force_reroute_authority;
    if (reroutable_nets.empty()) {
      expansion_error(
          result, "wildcard_only_force_reroute_scope",
          "wildcards/all_nets may select work but cannot authorize pre-existing copper removal",
          "/steps");
    }
  } else if (plan.mode == WorkPlanMode::allowlisted_ripup) {
    for (std::size_t index = 0U; index < plan.ripup_allowlist.size(); ++index) {
      const auto* net = net_by_name(plan.ripup_allowlist[index]);
      if (net == nullptr) {
        expansion_error(result, "unknown_work_plan_ripup_net",
                        "ripup_allowlist must contain exact source net names",
                        "/ripup_allowlist/" + std::to_string(index));
      } else {
        reroutable_nets.insert(net->id);
      }
    }
  }
  if (!reroutable_nets.empty()) {
    const auto partially_authorized = [&reroutable_nets](const std::vector<NetId>& members) {
      const auto count =
          std::count_if(members.begin(), members.end(),
                        [&reroutable_nets](NetId id) { return reroutable_nets.contains(id); });
      return count != 0 && static_cast<std::size_t>(count) != members.size();
    };
    for (const auto& pair : source.differential_pairs) {
      if (partially_authorized({pair.positive_net, pair.negative_net})) {
        expansion_error(result, "split_work_plan_ripup_differential_pair",
                        "pre-existing copper authority splits a declared differential pair",
                        plan.mode == WorkPlanMode::allowlisted_ripup ? "/ripup_allowlist"
                                                                     : "/steps");
      }
    }
    for (const auto& bus : source.routing_buses) {
      if (partially_authorized(bus.ordered_nets)) {
        expansion_error(result, "split_work_plan_ripup_routing_bus",
                        "pre-existing copper authority splits a declared routing bus",
                        plan.mode == WorkPlanMode::allowlisted_ripup ? "/ripup_allowlist"
                                                                     : "/steps");
      }
    }
  }
  result.copper_authority.reroutable_nets = sorted_ids(reroutable_nets);

  const auto classify = [&reroutable_nets](const auto& copper, auto& removable, auto& immutable) {
    for (const auto& item : copper) {
      if (!item.locked && reroutable_nets.contains(item.net)) {
        removable.push_back(item.id);
      } else {
        immutable.push_back(item.id);
      }
    }
    std::sort(removable.begin(), removable.end());
    std::sort(immutable.begin(), immutable.end());
  };
  classify(source.tracks, result.copper_authority.removable_tracks,
           result.copper_authority.immutable_tracks);
  classify(source.track_arcs, result.copper_authority.removable_track_arcs,
           result.copper_authority.immutable_track_arcs);
  classify(source.vias, result.copper_authority.removable_vias,
           result.copper_authority.immutable_vias);
  for (const auto& zone : source.zones) {
    result.copper_authority.immutable_zones.push_back(zone.id);
  }
  std::sort(result.copper_authority.immutable_zones.begin(),
            result.copper_authority.immutable_zones.end());

  std::set<NetId> selected_union;
  for (const auto& step : result.steps) {
    selected_union.insert(step.nets.begin(), step.nets.end());
  }
  result.selected_net_mask.reserve(source.nets.size());
  result.reroutable_net_mask.reserve(source.nets.size());
  for (const auto& net : source.nets) {
    result.selected_net_mask.push_back(selected_union.contains(net.id) ? 1U : 0U);
    result.reroutable_net_mask.push_back(reroutable_nets.contains(net.id) ? 1U : 0U);
  }
  return result;
}

std::string serialize_flux_work_plan(const FluxWorkPlan& plan) {
  json::object root;
  root["schema"] = "flux_work_plan";
  root["schema_version"] = plan.schema_version;
  json::object source;
  source["path"] = plan.source_path;
  source["revision"] = plan.source_revision;
  root["source"] = std::move(source);
  root["mode"] = work_plan_mode_name(plan.mode);
  root["ripup_allowlist"] = string_array(plan.ripup_allowlist);
  json::array policies;
  policies.reserve(plan.policies.size());
  for (const auto& policy : plan.policies) {
    json::object value;
    value["kind"] = policy.kind;
    value["path"] = policy.path;
    value["sha256"] = policy.sha256;
    value["source_revision"] = policy.source_revision;
    policies.push_back(std::move(value));
  }
  root["policies"] = std::move(policies);
  json::object telemetry;
  telemetry["enabled"] = plan.telemetry.enabled;
  telemetry["listen"] = plan.telemetry.listen;
  telemetry["rate_hz"] = plan.telemetry.rate_hz;
  telemetry["level"] = plan.telemetry.level;
  telemetry["record"] = plan.telemetry.record ? json::value(*plan.telemetry.record) : json::value();
  root["telemetry"] = std::move(telemetry);
  json::array steps;
  steps.reserve(plan.steps.size());
  for (const auto& step : plan.steps) {
    json::object value;
    value["id"] = step.id;
    value["kind"] = work_plan_step_kind_name(step.kind);
    value["depends_on"] = string_array(step.depends_on);
    value["selectors"] = selectors_json(step.selectors);
    value["options"] = options_json(step.options);
    steps.push_back(std::move(value));
  }
  root["steps"] = std::move(steps);
  return json::serialize(root) + '\n';
}

FluxWorkPlan make_flux_work_plan_template(const Board& source, std::string source_path) {
  FluxWorkPlan plan;
  plan.source_path = std::move(source_path);
  plan.source_revision = source.source_revision;
  return plan;
}

std::vector<std::string> required_flux_work_plan_capabilities(const FluxWorkPlan& plan) {
  std::vector<std::string> result;
  const auto append = [&result](std::string value) {
    if (std::find(result.begin(), result.end(), value) == result.end()) {
      result.push_back(std::move(value));
    }
  };
  for (const auto& step : plan.steps) {
    switch (step.kind) {
    case WorkPlanStepKind::route:
      append("routing.single-ended");
      append("routing.scope");
      break;
    case WorkPlanStepKind::fanout:
      append("fanout.package-aware");
      break;
    case WorkPlanStepKind::differential_pair:
      append("routing.differential-pairs");
      break;
    case WorkPlanStepKind::bus:
      append("routing.ordered-buses");
      break;
    case WorkPlanStepKind::plane:
      append("planes.synthesis-repair");
      break;
    case WorkPlanStepKind::tuning:
      append("tuning.length-time");
      break;
    case WorkPlanStepKind::cleanup:
      append("cleanup.copper");
      break;
    case WorkPlanStepKind::validation:
      append("verification.diagnostics");
      break;
    case WorkPlanStepKind::placement:
      append("placement.integrated");
      break;
    }
  }
  if (plan.mode == WorkPlanMode::allowlisted_ripup) {
    append("routing.ripup-recovery");
  }
  return result;
}

} // namespace flux::io
