#include "flux/io/placement_move_priorities.hpp"

#include "flux/io/sexpr.hpp"

#include <algorithm>
#include <cctype>
#include <cstddef>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <map>
#include <optional>
#include <set>
#include <string>
#include <string_view>
#include <utility>

namespace flux::io {
namespace {

constexpr std::string_view sidecar_head = "flux_placement_move_priorities";

struct Loader {
  PlacementMovePriorityResult result;

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
    loader.error("missing_placement_move_priority_field",
                 "missing required " + std::string(name) + " field", parent.location);
    return nullptr;
  }
  if (children.size() != 1U) {
    loader.error("duplicate_placement_move_priority_field", std::string(name) + " must be unique",
                 children[1]->location);
    return nullptr;
  }
  return children.front();
}

void reject_unknown_children(const SExpression& parent, const std::set<std::string_view>& allowed,
                             Loader& loader) {
  for (std::size_t index = 1U; index < parent.children.size(); ++index) {
    const auto& child = parent.children[index];
    if (!child.is_list() || !allowed.contains(head(child))) {
      loader.error("unknown_placement_move_priority_field",
                   "placement-move priority sidecar contains an unknown field", child.location);
    }
  }
}

[[nodiscard]] std::optional<PlacementMovePriorityClass> parse_class(std::string_view value) {
  for (const auto candidate : default_placement_move_priority_order()) {
    if (placement_move_priority_class_name(candidate) == value) {
      return candidate;
    }
  }
  return std::nullopt;
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

[[nodiscard]] PlacementMovePriorityClass suggested_class(std::string_view reference) {
  std::size_t prefix_end = 0U;
  while (prefix_end < reference.size() &&
         std::isalpha(static_cast<unsigned char>(reference[prefix_end])) != 0) {
    ++prefix_end;
  }
  std::string prefix{reference.substr(0U, prefix_end)};
  std::transform(prefix.begin(), prefix.end(), prefix.begin(),
                 [](unsigned char value) { return static_cast<char>(std::toupper(value)); });
  if (prefix == "R") {
    return PlacementMovePriorityClass::resistor;
  }
  if (prefix == "C") {
    return PlacementMovePriorityClass::capacitor;
  }
  if (prefix == "Q") {
    return PlacementMovePriorityClass::transistor;
  }
  if (prefix == "J" || prefix == "P") {
    return PlacementMovePriorityClass::connector;
  }
  if (prefix == "U") {
    return PlacementMovePriorityClass::misc_ic;
  }
  return PlacementMovePriorityClass::other;
}

} // namespace

bool PlacementMovePriorityResult::has_errors() const {
  return std::any_of(diagnostics.begin(), diagnostics.end(), [](const auto& diagnostic) {
    return diagnostic.severity == ConstraintSeverity::error;
  });
}

PlacementMovePriorityResult load_placement_move_priorities(const Board& source,
                                                           std::string_view contents) {
  Loader loader;
  SExpressionParseOptions options;
  options.maximum_depth = 32U;
  options.maximum_nodes = 100'000U;
  const auto parsed = parse_sexpression(contents, options);
  for (const auto& diagnostic : parsed.diagnostics) {
    loader.error(diagnostic.code, diagnostic.message, diagnostic.location);
  }
  if (!parsed.root || !parsed.diagnostics.empty()) {
    return std::move(loader.result);
  }
  const auto& root = *parsed.root;
  if (head(root) != sidecar_head) {
    loader.error("invalid_placement_move_priority_root",
                 "sidecar root must be flux_placement_move_priorities", root.location);
    return std::move(loader.result);
  }
  reject_unknown_children(root, {"version", "source_revision", "tiers", "footprint"}, loader);
  const auto* version = unique_child(root, "version", loader);
  const auto* revision = unique_child(root, "source_revision", loader);
  const auto* tiers = unique_child(root, "tiers", loader);
  if (version == nullptr || atom_at(*version, 1U) != std::optional<std::string_view>{"1"} ||
      version->children.size() != 2U) {
    loader.error("unsupported_placement_move_priority_version",
                 "placement-move priority sidecar version must be 1",
                 version == nullptr ? root.location : version->location);
  }
  const auto revision_value = revision == nullptr ? std::nullopt : atom_at(*revision, 1U);
  if (!revision_value || revision->children.size() != 2U) {
    loader.error("invalid_placement_move_priority_revision",
                 "source_revision requires exactly one value",
                 revision == nullptr ? root.location : revision->location);
  } else if (*revision_value != source.source_revision) {
    loader.error("stale_placement_move_priorities",
                 "placement-move priority sidecar targets a different board revision",
                 revision->location);
  }

  PlacementMovePriorityPolicy policy;
  if (revision_value && *revision_value == source.source_revision) {
    policy.source_revision = std::string(*revision_value);
  }
  std::set<PlacementMovePriorityClass> seen_classes;
  if (tiers != nullptr) {
    if (tiers->children.size() != default_placement_move_priority_order().size() + 1U) {
      loader.error("incomplete_placement_move_priority_tiers",
                   "tiers must contain every supported class exactly once", tiers->location);
    }
    for (std::size_t index = 1U; index < tiers->children.size(); ++index) {
      const auto value = atom_at(*tiers, index);
      const auto priority_class = value ? parse_class(*value) : std::nullopt;
      if (!priority_class) {
        loader.error("unknown_placement_move_priority_class",
                     "tiers contains an unknown priority class", tiers->children[index].location);
        continue;
      }
      if (!seen_classes.insert(*priority_class).second) {
        loader.error("duplicate_placement_move_priority_class",
                     "each priority class may appear once in tiers",
                     tiers->children[index].location,
                     {std::string(placement_move_priority_class_name(*priority_class))});
        continue;
      }
      policy.order.push_back(*priority_class);
    }
  }

  std::map<ObjectId, const Footprint*> footprint_by_id;
  for (const auto& footprint : source.footprints) {
    footprint_by_id.emplace(footprint.id, &footprint);
  }
  std::set<ObjectId> assigned;
  for (const auto* entry : find_children(root, "footprint")) {
    reject_unknown_children(*entry, {"id", "reference", "class"}, loader);
    const auto* id_field = unique_child(*entry, "id", loader);
    const auto* reference_field = unique_child(*entry, "reference", loader);
    const auto* class_field = unique_child(*entry, "class", loader);
    const auto id_value = id_field == nullptr ? std::nullopt : atom_at(*id_field, 1U);
    if (!id_value || id_field->children.size() != 2U || id_value->empty()) {
      loader.error("invalid_placement_move_priority_footprint_id",
                   "footprint id requires one non-empty value",
                   id_field == nullptr ? entry->location : id_field->location);
      continue;
    }
    const ObjectId id{std::string(*id_value)};
    if (!assigned.insert(id).second) {
      loader.error("duplicate_placement_move_priority_footprint",
                   "a footprint may have only one priority class", id_field->location,
                   {id.value()});
      continue;
    }
    const auto found = footprint_by_id.find(id);
    if (found == footprint_by_id.end()) {
      loader.error("unknown_placement_move_priority_footprint",
                   "priority sidecar references an unknown footprint", id_field->location,
                   {id.value()});
      continue;
    }
    const auto reference =
        reference_field == nullptr ? std::nullopt : atom_at(*reference_field, 1U);
    if (!reference || reference_field->children.size() != 2U ||
        *reference != found->second->reference) {
      loader.error("placement_move_priority_reference_mismatch",
                   "reference does not match the bound stable footprint ID",
                   reference_field == nullptr ? entry->location : reference_field->location,
                   {id.value(), found->second->reference});
    }
    const auto class_value = class_field == nullptr ? std::nullopt : atom_at(*class_field, 1U);
    const auto priority_class = class_value ? parse_class(*class_value) : std::nullopt;
    if (!priority_class || class_field->children.size() != 2U) {
      loader.error("invalid_placement_move_priority_assignment",
                   "class requires exactly one supported priority class",
                   class_field == nullptr ? entry->location : class_field->location, {id.value()});
      continue;
    }
    policy.footprints.push_back(FootprintMovePriority{id, *priority_class});
  }
  for (const auto& footprint : source.footprints) {
    if (!assigned.contains(footprint.id)) {
      loader.error("missing_placement_move_priority_footprint",
                   "priority sidecar must classify every footprint", root.location,
                   {footprint.id.value(), footprint.reference});
    }
  }

  if (loader.result.has_errors()) {
    return std::move(loader.result);
  }
  std::sort(policy.footprints.begin(), policy.footprints.end(),
            [](const auto& lhs, const auto& rhs) { return lhs.footprint < rhs.footprint; });
  loader.result.policy = std::move(policy);
  return std::move(loader.result);
}

PlacementMovePriorityResult load_placement_move_priorities_file(const Board& source,
                                                                const std::filesystem::path& path) {
  std::ifstream input(path, std::ios::binary);
  if (!input) {
    PlacementMovePriorityResult result;
    result.diagnostics.push_back(
        ConstraintDiagnostic{ConstraintSeverity::error,
                             "placement_move_priority_file_read_error",
                             "unable to read placement-move priority sidecar",
                             1,
                             1,
                             {path.string()}});
    return result;
  }
  const std::string contents{std::istreambuf_iterator<char>{input},
                             std::istreambuf_iterator<char>{}};
  return load_placement_move_priorities(source, contents);
}

std::string make_placement_move_priorities_template(const Board& source) {
  const auto policy = make_suggested_placement_move_priorities(source);
  std::string result =
      "; Suggested classes from reference prefixes are move priorities, not component-type facts.\n"
      "; Review U* parts and change MCU/SoC devices explicitly before relying on the order.\n"
      "(flux_placement_move_priorities\n  (version 1)\n  (source_revision " +
      quote(source.source_revision) + ")\n  (tiers";
  for (const auto priority_class : policy.order) {
    result += " ";
    result += placement_move_priority_class_name(priority_class);
  }
  result += ")\n";
  std::map<ObjectId, const Footprint*> footprint_by_id;
  for (const auto& footprint : source.footprints) {
    footprint_by_id.emplace(footprint.id, &footprint);
  }
  for (const auto& assignment : policy.footprints) {
    const auto* footprint = footprint_by_id.at(assignment.footprint);
    result += "  (footprint\n    (id " + quote(footprint->id.value()) + ")\n    (reference " +
              quote(footprint->reference) + ")\n    (class " +
              std::string(placement_move_priority_class_name(assignment.priority_class)) + "))\n";
  }
  result += ")\n";
  return result;
}

PlacementMovePriorityPolicy make_suggested_placement_move_priorities(const Board& source) {
  PlacementMovePriorityPolicy result;
  result.source_revision = source.source_revision;
  result.order = default_placement_move_priority_order();
  result.footprints.reserve(source.footprints.size());
  for (const auto& footprint : source.footprints) {
    result.footprints.push_back(
        FootprintMovePriority{footprint.id, suggested_class(footprint.reference)});
  }
  std::sort(result.footprints.begin(), result.footprints.end(),
            [](const auto& lhs, const auto& rhs) { return lhs.footprint < rhs.footprint; });
  return result;
}

} // namespace flux::io
