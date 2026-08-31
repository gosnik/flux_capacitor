#include "flux/io/kicad_netclass_snapshot.hpp"

#include "flux/core/validation.hpp"
#include "flux/io/sexpr.hpp"

#include <algorithm>
#include <charconv>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <map>
#include <optional>
#include <set>
#include <string>
#include <string_view>
#include <system_error>
#include <tuple>
#include <utility>
#include <vector>

namespace flux::io {
namespace {

constexpr std::string_view snapshot_head = "flux_kicad_netclass_snapshot";
constexpr std::string_view default_class_name = "Default";

struct ParsedClass {
  std::string name;
  std::int64_t source_priority{0};
  std::optional<Nanometres> clearance;
  std::optional<Nanometres> track_width;
  std::optional<Nanometres> via_diameter;
  std::optional<Nanometres> via_drill;
  std::optional<Nanometres> differential_pair_track_width;
  std::optional<Nanometres> differential_pair_gap;
  std::optional<Nanometres> differential_pair_via_gap;
};

struct ParsedNet {
  std::string name;
  std::vector<std::string> classes;
};

struct Loader {
  KicadNetClassSnapshotResult result;

  void error(std::string code, std::string message, SourceLocation location = {},
             std::vector<std::string> object_ids = {}) {
    result.diagnostics.push_back(NetClassSnapshotDiagnostic{
        NetClassSnapshotSeverity::error, std::move(code), std::move(message), location.line,
        location.column, std::move(object_ids)});
  }
};

[[nodiscard]] std::optional<std::string_view> atom_at(const SExpression& expression,
                                                      std::size_t index) {
  const auto* child = child_at(expression, index);
  return child != nullptr && child->is_atom() ? std::optional<std::string_view>{child->value}
                                              : std::nullopt;
}

[[nodiscard]] const SExpression* unique_child(const SExpression& parent, std::string_view name,
                                              Loader& loader, bool required = true) {
  const auto children = find_children(parent, name);
  if (children.empty()) {
    if (required) {
      loader.error("missing_netclass_snapshot_field",
                   "missing required " + std::string(name) + " field", parent.location);
    }
    return nullptr;
  }
  if (children.size() != 1U) {
    loader.error("duplicate_netclass_snapshot_field", std::string(name) + " field must be unique",
                 children[1]->location);
    return nullptr;
  }
  return children.front();
}

void reject_unknown_children(const SExpression& parent, const std::set<std::string_view>& allowed,
                             Loader& loader) {
  for (std::size_t index = 1; index < parent.children.size(); ++index) {
    const auto& child = parent.children[index];
    if (!child.is_list() || !allowed.contains(head(child))) {
      loader.error("unknown_netclass_snapshot_field",
                   "net-class snapshot contains an unknown field", child.location);
    }
  }
}

[[nodiscard]] std::optional<std::int64_t> parse_integer(std::string_view value) {
  std::int64_t result = 0;
  const auto parsed = std::from_chars(value.data(), value.data() + value.size(), result);
  return parsed.ec == std::errc{} && parsed.ptr == value.data() + value.size()
             ? std::optional<std::int64_t>{result}
             : std::nullopt;
}

[[nodiscard]] std::optional<Nanometres> parse_dimension(const SExpression* field,
                                                        std::string_view name, Loader& loader,
                                                        const std::string& class_name) {
  if (field == nullptr) {
    return std::nullopt;
  }
  const auto value = atom_at(*field, 1);
  const auto parsed = value ? parse_integer(*value) : std::nullopt;
  if (!parsed || field->children.size() != 2U) {
    loader.error("invalid_netclass_snapshot_dimension",
                 std::string(name) + " requires exactly one integer nanometre value",
                 field->location, {class_name});
    return std::nullopt;
  }
  return Nanometres{*parsed};
}

[[nodiscard]] std::optional<ParsedClass> parse_class(const SExpression& expression,
                                                     Loader& loader) {
  reject_unknown_children(expression,
                          {"name", "source_priority", "clearance_nm", "track_width_nm",
                           "via_diameter_nm", "via_drill_nm", "diff_pair_track_width_nm",
                           "diff_pair_gap_nm", "diff_pair_via_gap_nm"},
                          loader);
  const auto* name_field = unique_child(expression, "name", loader);
  const auto* priority_field = unique_child(expression, "source_priority", loader);
  const auto name = name_field == nullptr ? std::nullopt : atom_at(*name_field, 1);
  if (!name || name->empty() || name_field->children.size() != 2U) {
    loader.error("invalid_netclass_snapshot_name", "class name requires one non-empty value",
                 name_field == nullptr ? expression.location : name_field->location);
    return std::nullopt;
  }
  const auto priority_value =
      priority_field == nullptr ? std::nullopt : atom_at(*priority_field, 1);
  const auto priority = priority_value ? parse_integer(*priority_value) : std::nullopt;
  if (!priority || priority_field->children.size() != 2U) {
    loader.error("invalid_netclass_snapshot_priority",
                 "source_priority requires exactly one integer value",
                 priority_field == nullptr ? expression.location : priority_field->location,
                 {std::string(*name)});
    return std::nullopt;
  }
  ParsedClass result;
  result.name = std::string(*name);
  result.source_priority = *priority;
  result.clearance = parse_dimension(unique_child(expression, "clearance_nm", loader, false),
                                     "clearance_nm", loader, result.name);
  result.track_width = parse_dimension(unique_child(expression, "track_width_nm", loader, false),
                                       "track_width_nm", loader, result.name);
  result.via_diameter = parse_dimension(unique_child(expression, "via_diameter_nm", loader, false),
                                        "via_diameter_nm", loader, result.name);
  result.via_drill = parse_dimension(unique_child(expression, "via_drill_nm", loader, false),
                                     "via_drill_nm", loader, result.name);
  result.differential_pair_track_width =
      parse_dimension(unique_child(expression, "diff_pair_track_width_nm", loader, false),
                      "diff_pair_track_width_nm", loader, result.name);
  result.differential_pair_gap =
      parse_dimension(unique_child(expression, "diff_pair_gap_nm", loader, false),
                      "diff_pair_gap_nm", loader, result.name);
  result.differential_pair_via_gap =
      parse_dimension(unique_child(expression, "diff_pair_via_gap_nm", loader, false),
                      "diff_pair_via_gap_nm", loader, result.name);
  return result;
}

[[nodiscard]] std::optional<ParsedNet> parse_net(const SExpression& expression, Loader& loader) {
  reject_unknown_children(expression, {"name", "classes"}, loader);
  const auto* name_field = unique_child(expression, "name", loader);
  const auto* classes_field = unique_child(expression, "classes", loader);
  const auto name = name_field == nullptr ? std::nullopt : atom_at(*name_field, 1);
  if (!name || name->empty() || name_field->children.size() != 2U) {
    loader.error("invalid_netclass_snapshot_net_name", "net name requires one non-empty value",
                 name_field == nullptr ? expression.location : name_field->location);
    return std::nullopt;
  }
  if (classes_field == nullptr) {
    return std::nullopt;
  }
  ParsedNet result;
  result.name = std::string(*name);
  std::set<std::string> seen;
  for (std::size_t index = 1; index < classes_field->children.size(); ++index) {
    const auto value = atom_at(*classes_field, index);
    if (!value || value->empty()) {
      loader.error("invalid_netclass_snapshot_membership",
                   "class membership requires non-empty class names",
                   classes_field->children[index].location, {result.name});
      continue;
    }
    if (!seen.insert(std::string(*value)).second) {
      loader.error("duplicate_netclass_snapshot_membership",
                   "a net may list each class at most once",
                   classes_field->children[index].location, {result.name, std::string(*value)});
      continue;
    }
    result.classes.emplace_back(*value);
  }
  return result;
}

[[nodiscard]] NetClassId class_id(std::string_view name) {
  return NetClassId{"kicad-netclass:" + std::string(name)};
}

} // namespace

bool KicadNetClassSnapshotResult::has_errors() const {
  return std::any_of(diagnostics.begin(), diagnostics.end(), [](const auto& diagnostic) {
    return diagnostic.severity == NetClassSnapshotSeverity::error;
  });
}

KicadNetClassSnapshotResult apply_kicad_netclass_snapshot(const Board& source,
                                                          std::string_view contents) {
  Loader loader;
  SExpressionParseOptions options;
  options.maximum_depth = 32;
  options.maximum_nodes = 100'000;
  const auto parsed = parse_sexpression(contents, options);
  for (const auto& diagnostic : parsed.diagnostics) {
    loader.error(diagnostic.code, diagnostic.message, diagnostic.location);
  }
  if (!parsed.root || !parsed.diagnostics.empty()) {
    return std::move(loader.result);
  }
  const auto& root = *parsed.root;
  if (head(root) != snapshot_head) {
    loader.error("invalid_netclass_snapshot_root",
                 "snapshot root must be flux_kicad_netclass_snapshot", root.location);
    return std::move(loader.result);
  }
  reject_unknown_children(root, {"version", "source_revision", "api_version", "class", "net"},
                          loader);
  const auto* version = unique_child(root, "version", loader);
  const auto* revision = unique_child(root, "source_revision", loader);
  const auto* api_version = unique_child(root, "api_version", loader);
  if (version == nullptr || atom_at(*version, 1) != std::optional<std::string_view>{"2"} ||
      version->children.size() != 2U) {
    loader.error("unsupported_netclass_snapshot_version", "snapshot version must be 2",
                 version == nullptr ? root.location : version->location);
  }
  const auto revision_value = revision == nullptr ? std::nullopt : atom_at(*revision, 1);
  if (!revision_value || revision->children.size() != 2U) {
    loader.error("invalid_netclass_snapshot_revision", "source_revision requires exactly one value",
                 revision == nullptr ? root.location : revision->location);
  } else if (*revision_value != source.source_revision) {
    loader.error("stale_netclass_snapshot",
                 "net-class snapshot was generated for a different board revision",
                 revision->location);
  }
  const auto api_version_value = api_version == nullptr ? std::nullopt : atom_at(*api_version, 1);
  if (!api_version_value || api_version_value->empty() || api_version->children.size() != 2U) {
    loader.error("invalid_netclass_snapshot_api_version",
                 "api_version requires exactly one non-empty value",
                 api_version == nullptr ? root.location : api_version->location);
  } else {
    loader.result.api_version = std::string(*api_version_value);
  }

  std::map<std::string, ParsedClass> classes;
  for (const auto* entry : find_children(root, "class")) {
    auto parsed_class = parse_class(*entry, loader);
    if (parsed_class && !classes.emplace(parsed_class->name, *parsed_class).second) {
      loader.error("duplicate_netclass_snapshot_class", "class appears more than once",
                   entry->location, {parsed_class->name});
    }
  }
  const auto default_class = classes.find(std::string(default_class_name));
  if (default_class == classes.end()) {
    loader.error("missing_default_netclass", "snapshot must contain KiCad's Default class",
                 root.location);
  } else if (!default_class->second.clearance || !default_class->second.track_width ||
             !default_class->second.via_diameter || !default_class->second.via_drill ||
             !default_class->second.differential_pair_track_width ||
             !default_class->second.differential_pair_gap ||
             !default_class->second.differential_pair_via_gap) {
    loader.error("incomplete_default_netclass",
                 "Default class must define single-ended, via, and differential-pair dimensions",
                 root.location, {std::string(default_class_name)});
  }

  std::map<std::string, ParsedNet> nets;
  for (const auto* entry : find_children(root, "net")) {
    auto parsed_net = parse_net(*entry, loader);
    if (parsed_net && !nets.emplace(parsed_net->name, *parsed_net).second) {
      loader.error("duplicate_netclass_snapshot_net", "net appears more than once", entry->location,
                   {parsed_net->name});
    }
  }
  std::map<std::string, Net*> board_nets;
  Board candidate = source;
  for (auto& net : candidate.nets) {
    if (net.name.empty()) {
      net.classes.clear();
      continue;
    }
    if (!board_nets.emplace(net.name, &net).second) {
      loader.error("ambiguous_board_net_name", "board contains duplicate named nets", root.location,
                   {net.name});
    }
  }
  for (const auto& [name, parsed_net] : nets) {
    static_cast<void>(parsed_net);
    if (!board_nets.contains(name)) {
      loader.error("unknown_netclass_snapshot_net", "snapshot references a net absent from board",
                   root.location, {name});
    }
  }
  for (const auto& [name, net] : board_nets) {
    const auto found = nets.find(name);
    if (found == nets.end()) {
      loader.error("missing_netclass_snapshot_net", "snapshot must cover every named board net",
                   root.location, {name});
      continue;
    }
    for (const auto& class_name : found->second.classes) {
      if (class_name == default_class_name) {
        loader.error("explicit_default_netclass_membership",
                     "Default is an implicit fallback and must not be listed as a membership",
                     root.location, {name, class_name});
      } else if (!classes.contains(class_name)) {
        loader.error("unknown_netclass_snapshot_class", "net references an undefined class",
                     root.location, {name, class_name});
      } else {
        net->classes.push_back(class_id(class_name));
      }
    }
  }
  if (loader.result.has_errors()) {
    return std::move(loader.result);
  }

  candidate.rules.default_clearance = *default_class->second.clearance;
  candidate.rules.default_track_width = *default_class->second.track_width;
  candidate.rules.default_via_diameter = *default_class->second.via_diameter;
  candidate.rules.default_via_drill = *default_class->second.via_drill;
  candidate.rules.default_differential_pair_track_width =
      *default_class->second.differential_pair_track_width;
  candidate.rules.default_differential_pair_gap = *default_class->second.differential_pair_gap;
  candidate.rules.default_differential_pair_via_gap =
      *default_class->second.differential_pair_via_gap;
  std::vector<const ParsedClass*> ordered;
  ordered.reserve(classes.size() - 1U);
  for (const auto& [name, parsed_class] : classes) {
    if (name != default_class_name) {
      ordered.push_back(&parsed_class);
    }
  }
  std::sort(ordered.begin(), ordered.end(), [](const ParsedClass* lhs, const ParsedClass* rhs) {
    return std::tie(lhs->source_priority, lhs->name) < std::tie(rhs->source_priority, rhs->name);
  });
  candidate.net_classes.clear();
  candidate.net_classes.reserve(ordered.size());
  for (std::size_t index = 0; index < ordered.size(); ++index) {
    const auto* parsed_class = ordered[index];
    const auto canonical_priority = static_cast<std::uint32_t>(ordered.size() - index);
    candidate.net_classes.push_back(
        NetClass{class_id(parsed_class->name), parsed_class->name, canonical_priority,
                 parsed_class->clearance, parsed_class->track_width, parsed_class->via_diameter,
                 parsed_class->via_drill, parsed_class->differential_pair_track_width,
                 parsed_class->differential_pair_gap, parsed_class->differential_pair_via_gap});
  }
  const auto validation = validate_board(candidate);
  if (!validation.ok()) {
    for (const auto& issue : validation.issues) {
      if (issue.severity == IssueSeverity::error) {
        loader.error("invalid_applied_netclass_snapshot",
                     "applied snapshot violates canonical rule: " + issue.code, root.location,
                     issue.object_ids);
      }
    }
    return std::move(loader.result);
  }
  loader.result.applied_classes = candidate.net_classes.size();
  loader.result.applied_nets = board_nets.size();
  loader.result.board = std::move(candidate);
  return std::move(loader.result);
}

KicadNetClassSnapshotResult apply_kicad_netclass_snapshot_file(const Board& source,
                                                               const std::filesystem::path& path) {
  std::ifstream input(path, std::ios::binary);
  if (!input) {
    KicadNetClassSnapshotResult result;
    result.diagnostics.push_back(
        NetClassSnapshotDiagnostic{NetClassSnapshotSeverity::error,
                                   "netclass_snapshot_file_read_error",
                                   "unable to read KiCad net-class snapshot",
                                   1,
                                   1,
                                   {path.string()}});
    return result;
  }
  const std::string contents{std::istreambuf_iterator<char>{input},
                             std::istreambuf_iterator<char>{}};
  return apply_kicad_netclass_snapshot(source, contents);
}

} // namespace flux::io
