#include "flux/io/routing_bus_constraints.hpp"

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
#include <utility>

namespace flux::io {
namespace {

constexpr std::string_view sidecar_head = "flux_routing_bus_constraints";

struct Loader {
  RoutingBusConstraintResult result;

  void error(std::string code, std::string message, SourceLocation location = {},
             std::vector<std::string> object_ids = {}) {
    result.diagnostics.push_back(ConstraintDiagnostic{ConstraintSeverity::error, std::move(code),
                                                      std::move(message), location.line,
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
                                              Loader& loader) {
  const auto children = find_children(parent, name);
  if (children.empty()) {
    loader.error("missing_routing_bus_constraint_field",
                 "missing required " + std::string(name) + " field", parent.location);
    return nullptr;
  }
  if (children.size() != 1U) {
    loader.error("duplicate_routing_bus_constraint_field",
                 std::string(name) + " field must be unique", children[1]->location);
    return nullptr;
  }
  return children.front();
}

void reject_unknown_children(const SExpression& parent, const std::set<std::string_view>& allowed,
                             Loader& loader) {
  for (std::size_t index = 1; index < parent.children.size(); ++index) {
    const auto& child = parent.children[index];
    if (!child.is_list() || !allowed.contains(head(child))) {
      loader.error("unknown_routing_bus_constraint_field",
                   "routing-bus sidecar contains an unknown field", child.location);
    }
  }
}

[[nodiscard]] std::optional<Nanometres> parse_optional_gap(const SExpression* field, Loader& loader,
                                                           const RoutingBusId& id) {
  if (field == nullptr) {
    return std::nullopt;
  }
  const auto token = atom_at(*field, 1);
  if (!token || field->children.size() != 2U) {
    loader.error("invalid_routing_bus_gap",
                 "minimum_gap_nm requires one integer nanometre value or none", field->location,
                 {id.value()});
    return std::nullopt;
  }
  if (*token == "none") {
    return std::nullopt;
  }
  std::int64_t value = 0;
  const auto parsed = std::from_chars(token->data(), token->data() + token->size(), value);
  if (parsed.ec != std::errc{} || parsed.ptr != token->data() + token->size() || value < 0 ||
      value > max_abs_coordinate_nm) {
    loader.error("invalid_routing_bus_gap",
                 "minimum_gap_nm must be none or a non-negative bounded integer", field->location,
                 {id.value()});
    return std::nullopt;
  }
  return Nanometres{value};
}

[[nodiscard]] std::string quote(std::string_view value) {
  std::string result{"\""};
  for (const auto character : value) {
    if (character == '\\' || character == '"') {
      result.push_back('\\');
    }
    result.push_back(character);
  }
  result.push_back('"');
  return result;
}

} // namespace

bool RoutingBusConstraintResult::has_errors() const {
  return std::any_of(diagnostics.begin(), diagnostics.end(), [](const auto& diagnostic) {
    return diagnostic.severity == ConstraintSeverity::error;
  });
}

RoutingBusConstraintResult apply_routing_bus_constraints(const Board& source,
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
  if (head(root) != sidecar_head) {
    loader.error("invalid_routing_bus_constraint_root",
                 "sidecar root must be flux_routing_bus_constraints", root.location);
    return std::move(loader.result);
  }
  reject_unknown_children(root, {"version", "source_revision", "bus"}, loader);
  const auto* version = unique_child(root, "version", loader);
  const auto* revision = unique_child(root, "source_revision", loader);
  if (version == nullptr || atom_at(*version, 1) != std::optional<std::string_view>{"1"} ||
      version->children.size() != 2U) {
    loader.error("unsupported_routing_bus_constraint_version",
                 "routing-bus sidecar version must be 1",
                 version == nullptr ? root.location : version->location);
  }
  const auto revision_value = revision == nullptr ? std::nullopt : atom_at(*revision, 1);
  if (!revision_value || revision->children.size() != 2U) {
    loader.error("invalid_routing_bus_constraint_revision",
                 "source_revision requires exactly one value",
                 revision == nullptr ? root.location : revision->location);
  } else if (*revision_value != source.source_revision) {
    loader.error("stale_routing_bus_constraints",
                 "routing-bus sidecar targets a different board revision", revision->location);
  }

  Board candidate = source;
  candidate.routing_buses.clear();
  std::map<std::string, NetId> net_by_name;
  for (const auto& net : candidate.nets) {
    if (!net.name.empty()) {
      net_by_name.emplace(net.name, net.id);
    }
  }
  std::set<RoutingBusId> seen;
  for (const auto* entry : find_children(root, "bus")) {
    reject_unknown_children(*entry, {"id", "name", "ordered_nets", "minimum_gap_nm"}, loader);
    const auto* id_field = unique_child(*entry, "id", loader);
    const auto* name_field = unique_child(*entry, "name", loader);
    const auto* nets_field = unique_child(*entry, "ordered_nets", loader);
    const auto id_value = id_field == nullptr ? std::nullopt : atom_at(*id_field, 1);
    const auto name_value = name_field == nullptr ? std::nullopt : atom_at(*name_field, 1);
    if (!id_value || id_field->children.size() != 2U || id_value->empty()) {
      loader.error("invalid_routing_bus_constraint_id", "bus id requires one non-empty value",
                   id_field == nullptr ? entry->location : id_field->location);
      continue;
    }
    const RoutingBusId id{std::string(*id_value)};
    if (!seen.insert(id).second) {
      loader.error("duplicate_routing_bus_constraint", "bus appears more than once",
                   id_field->location, {id.value()});
      continue;
    }
    if (!name_value || name_field->children.size() != 2U || name_value->empty()) {
      loader.error("invalid_routing_bus_constraint_name", "bus name requires one non-empty value",
                   name_field == nullptr ? entry->location : name_field->location, {id.value()});
      continue;
    }
    std::vector<NetId> ordered_nets;
    if (nets_field == nullptr || nets_field->children.size() < 3U) {
      loader.error("routing_bus_constraint_too_small",
                   "ordered_nets requires at least two net names",
                   nets_field == nullptr ? entry->location : nets_field->location, {id.value()});
    } else {
      for (std::size_t index = 1U; index < nets_field->children.size(); ++index) {
        const auto net_name = atom_at(*nets_field, index);
        if (!net_name || net_name->empty()) {
          loader.error("invalid_routing_bus_constraint_net",
                       "ordered_nets members must be non-empty atom values",
                       nets_field->children[index].location, {id.value()});
          continue;
        }
        const auto found = net_by_name.find(std::string(*net_name));
        if (found == net_by_name.end()) {
          loader.error("unknown_routing_bus_constraint_net",
                       "ordered_nets references an unknown canonical net name",
                       nets_field->children[index].location, {id.value(), std::string(*net_name)});
          continue;
        }
        ordered_nets.push_back(found->second);
      }
    }
    candidate.routing_buses.push_back(
        RoutingBus{id, std::string(*name_value), std::move(ordered_nets),
                   parse_optional_gap(unique_child(*entry, "minimum_gap_nm", loader), loader, id)});
    loader.result.loaded_buses.push_back(id);
  }

  const auto validation = validate_board(candidate);
  if (!validation.ok()) {
    for (const auto& issue : validation.issues) {
      if (issue.severity == IssueSeverity::error) {
        loader.error("invalid_applied_routing_bus_constraints",
                     "applied sidecar violates canonical rule: " + issue.code, root.location,
                     issue.object_ids);
      }
    }
  }
  if (loader.result.has_errors()) {
    loader.result.loaded_buses.clear();
    return std::move(loader.result);
  }
  std::sort(loader.result.loaded_buses.begin(), loader.result.loaded_buses.end());
  loader.result.board = std::move(candidate);
  return std::move(loader.result);
}

RoutingBusConstraintResult apply_routing_bus_constraints_file(const Board& source,
                                                              const std::filesystem::path& path) {
  std::ifstream input(path, std::ios::binary);
  if (!input) {
    RoutingBusConstraintResult result;
    result.diagnostics.push_back(
        ConstraintDiagnostic{ConstraintSeverity::error,
                             "routing_bus_constraint_file_read_error",
                             "unable to read routing-bus constraint sidecar",
                             1,
                             1,
                             {path.string()}});
    return result;
  }
  const std::string contents{std::istreambuf_iterator<char>{input},
                             std::istreambuf_iterator<char>{}};
  return apply_routing_bus_constraints(source, contents);
}

std::string make_routing_bus_constraints_template(const Board& source) {
  return "(flux_routing_bus_constraints\n  (version 1)\n  (source_revision " +
         quote(source.source_revision) + ")\n)\n";
}

} // namespace flux::io
