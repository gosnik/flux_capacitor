#include "flux/io/relative_placement_constraints.hpp"

#include "flux/core/geometry.hpp"
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

constexpr std::string_view sidecar_head = "flux_relative_placement_constraints";

struct Loader {
  RelativePlacementConstraintResult result;

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

[[nodiscard]] std::optional<std::int64_t> parse_integer(std::string_view value) {
  std::int64_t parsed_value = 0;
  const auto parsed = std::from_chars(value.data(), value.data() + value.size(), parsed_value);
  return parsed.ec == std::errc{} && parsed.ptr == value.data() + value.size()
             ? std::optional<std::int64_t>{parsed_value}
             : std::nullopt;
}

[[nodiscard]] const SExpression* unique_child(const SExpression& parent, std::string_view name,
                                              Loader& loader) {
  const auto children = find_children(parent, name);
  if (children.empty()) {
    loader.error("missing_relative_placement_field",
                 "missing required " + std::string(name) + " field", parent.location);
    return nullptr;
  }
  if (children.size() != 1U) {
    loader.error("duplicate_relative_placement_field", std::string(name) + " field must be unique",
                 children[1]->location);
    return nullptr;
  }
  return children.front();
}

[[nodiscard]] const SExpression* optional_unique_child(const SExpression& parent,
                                                       std::string_view name, Loader& loader) {
  const auto children = find_children(parent, name);
  if (children.empty()) {
    return nullptr;
  }
  if (children.size() != 1U) {
    loader.error("duplicate_relative_placement_field", std::string(name) + " field must be unique",
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
      loader.error("unknown_relative_placement_field",
                   "relative-placement sidecar contains an unknown field", child.location);
    }
  }
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

bool RelativePlacementConstraintResult::has_errors() const {
  return std::any_of(diagnostics.begin(), diagnostics.end(), [](const auto& diagnostic) {
    return diagnostic.severity == ConstraintSeverity::error;
  });
}

RelativePlacementConstraintResult apply_relative_placement_constraints(const Board& source,
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
    loader.error("invalid_relative_placement_root",
                 "sidecar root must be flux_relative_placement_constraints", root.location);
    return std::move(loader.result);
  }
  reject_unknown_children(root, {"version", "source_revision", "constraint"}, loader);
  const auto* version = unique_child(root, "version", loader);
  const auto* revision = unique_child(root, "source_revision", loader);
  const auto version_value = version == nullptr ? std::nullopt : atom_at(*version, 1);
  const bool version_1 = version_value == std::optional<std::string_view>{"1"};
  const bool version_2 = version_value == std::optional<std::string_view>{"2"};
  if (version == nullptr || (!version_1 && !version_2) || version->children.size() != 2U) {
    loader.error("unsupported_relative_placement_version",
                 "relative-placement sidecar version must be 1 or 2",
                 version == nullptr ? root.location : version->location);
  }
  const auto revision_value = revision == nullptr ? std::nullopt : atom_at(*revision, 1);
  if (!revision_value || revision->children.size() != 2U) {
    loader.error("invalid_relative_placement_revision",
                 "source_revision requires exactly one value",
                 revision == nullptr ? root.location : revision->location);
  } else if (*revision_value != source.source_revision) {
    loader.error("stale_relative_placement_constraints",
                 "relative-placement sidecar targets a different board revision",
                 revision->location);
  }

  Board candidate = source;
  candidate.relative_placement_constraints.clear();
  std::map<ObjectId, const Footprint*> footprint_by_id;
  for (const auto& footprint : candidate.footprints) {
    footprint_by_id.emplace(footprint.id, &footprint);
  }
  std::set<RelativePlacementConstraintId> seen_ids;
  std::set<std::pair<ObjectId, ObjectId>> seen_pairs;
  for (const auto* entry : find_children(root, "constraint")) {
    auto allowed_fields = std::set<std::string_view>{
        "id", "name", "first", "second", "maximum_x_deviation_nm", "maximum_y_deviation_nm"};
    if (version_2) {
      allowed_fields.insert("maximum_distance_deviation_nm");
      allowed_fields.insert("maximum_rotation_deviation_microdegrees");
    }
    reject_unknown_children(*entry, allowed_fields, loader);
    const auto* id_field = unique_child(*entry, "id", loader);
    const auto* name_field = unique_child(*entry, "name", loader);
    const auto* first_field = unique_child(*entry, "first", loader);
    const auto* second_field = unique_child(*entry, "second", loader);
    const auto* max_x_field = unique_child(*entry, "maximum_x_deviation_nm", loader);
    const auto* max_y_field = unique_child(*entry, "maximum_y_deviation_nm", loader);
    const auto* max_distance_field =
        version_2 ? optional_unique_child(*entry, "maximum_distance_deviation_nm", loader)
                  : nullptr;
    const auto* max_rotation_field =
        version_2 ? optional_unique_child(*entry, "maximum_rotation_deviation_microdegrees", loader)
                  : nullptr;
    const auto id_value = id_field == nullptr ? std::nullopt : atom_at(*id_field, 1);
    const auto name_value = name_field == nullptr ? std::nullopt : atom_at(*name_field, 1);
    if (!id_value || id_field->children.size() != 2U || id_value->empty()) {
      loader.error("invalid_relative_placement_id", "constraint id requires one non-empty value",
                   id_field == nullptr ? entry->location : id_field->location);
      continue;
    }
    const RelativePlacementConstraintId id{std::string(*id_value)};
    std::optional<Nanometres> maximum_distance_deviation;
    if (max_distance_field != nullptr) {
      const auto value = atom_at(*max_distance_field, 1);
      const auto parsed_value = value ? parse_integer(*value) : std::nullopt;
      if (!parsed_value || max_distance_field->children.size() != 2U || *parsed_value < 0) {
        loader.error("invalid_relative_distance_tolerance",
                     "maximum distance deviation requires one non-negative integer nanometre "
                     "value",
                     max_distance_field->location, {id.value()});
        continue;
      }
      maximum_distance_deviation = Nanometres{*parsed_value};
    }
    std::optional<std::uint64_t> maximum_rotation_deviation;
    if (max_rotation_field != nullptr) {
      const auto value = atom_at(*max_rotation_field, 1);
      const auto parsed_value = value ? parse_integer(*value) : std::nullopt;
      constexpr std::int64_t half_turn_microdegrees = 180'000'000;
      if (!parsed_value || max_rotation_field->children.size() != 2U || *parsed_value < 0 ||
          *parsed_value > half_turn_microdegrees) {
        loader.error("invalid_relative_rotation_tolerance",
                     "maximum rotation deviation requires one integer microdegree value from 0 "
                     "through 180000000",
                     max_rotation_field->location, {id.value()});
        continue;
      }
      maximum_rotation_deviation = static_cast<std::uint64_t>(*parsed_value);
    }
    if (!seen_ids.insert(id).second) {
      loader.error("duplicate_relative_placement_constraint", "constraint appears more than once",
                   id_field->location, {id.value()});
      continue;
    }
    if (!name_value || name_field->children.size() != 2U || name_value->empty()) {
      loader.error("invalid_relative_placement_name",
                   "constraint name requires one non-empty value",
                   name_field == nullptr ? entry->location : name_field->location, {id.value()});
      continue;
    }
    const auto x_value = max_x_field == nullptr ? std::nullopt : atom_at(*max_x_field, 1);
    const auto y_value = max_y_field == nullptr ? std::nullopt : atom_at(*max_y_field, 1);
    const auto max_x = x_value ? parse_integer(*x_value) : std::nullopt;
    const auto max_y = y_value ? parse_integer(*y_value) : std::nullopt;
    if (!max_x || max_x_field->children.size() != 2U || *max_x < 0 || !max_y ||
        max_y_field->children.size() != 2U || *max_y < 0) {
      loader.error("invalid_relative_placement_tolerance",
                   "maximum deviations require one non-negative integer nanometre value",
                   entry->location, {id.value()});
      continue;
    }

    const auto parse_endpoint = [&](const SExpression* field,
                                    std::string_view role) -> std::optional<ObjectId> {
      if (field == nullptr) {
        return std::nullopt;
      }
      reject_unknown_children(*field, {"id", "reference"}, loader);
      const auto* endpoint_id_field = unique_child(*field, "id", loader);
      const auto* reference_field = unique_child(*field, "reference", loader);
      const auto endpoint_value =
          endpoint_id_field == nullptr ? std::nullopt : atom_at(*endpoint_id_field, 1);
      if (!endpoint_value || endpoint_id_field->children.size() != 2U || endpoint_value->empty()) {
        loader.error("invalid_relative_placement_endpoint_id",
                     std::string(role) + " footprint id requires one non-empty value",
                     endpoint_id_field == nullptr ? field->location : endpoint_id_field->location,
                     {id.value()});
        return std::nullopt;
      }
      const ObjectId endpoint_id{std::string(*endpoint_value)};
      const auto found = footprint_by_id.find(endpoint_id);
      if (found == footprint_by_id.end()) {
        loader.error("unknown_relative_placement_constraint_footprint",
                     std::string(role) + " references an unknown footprint",
                     endpoint_id_field->location, {id.value(), endpoint_id.value()});
        return std::nullopt;
      }
      const auto reference =
          reference_field == nullptr ? std::nullopt : atom_at(*reference_field, 1);
      if (!reference || reference_field->children.size() != 2U ||
          *reference != found->second->reference) {
        loader.error("relative_placement_reference_mismatch",
                     std::string(role) + " reference does not match the bound stable footprint ID",
                     reference_field == nullptr ? field->location : reference_field->location,
                     {id.value(), endpoint_id.value(), found->second->reference});
      }
      return endpoint_id;
    };
    const auto first = parse_endpoint(first_field, "first");
    const auto second = parse_endpoint(second_field, "second");
    if (!first || !second) {
      continue;
    }
    if (*first == *second) {
      loader.error("relative_placement_constraint_self_pair",
                   "constraint endpoints must be distinct footprints", entry->location,
                   {id.value(), first->value()});
      continue;
    }
    const auto pair = std::minmax(*first, *second);
    if (!seen_pairs.emplace(pair.first, pair.second).second) {
      loader.error("duplicate_relative_placement_constraint_pair",
                   "an unordered footprint pair may appear only once", entry->location,
                   {id.value(), first->value(), second->value()});
      continue;
    }
    const auto first_footprint = footprint_by_id.at(*first);
    const auto second_footprint = footprint_by_id.at(*second);
    const auto nominal_offset = second_footprint->position - first_footprint->position;
    candidate.relative_placement_constraints.push_back(RelativePlacementConstraint{
        id, std::string(*name_value), *first, *second, nominal_offset,
        Nanometres{static_cast<std::int64_t>(geometry::segment_length_nm(Point{}, nominal_offset))},
        Microdegrees{second_footprint->rotation.value() - first_footprint->rotation.value()},
        Nanometres{*max_x}, Nanometres{*max_y}, maximum_distance_deviation,
        maximum_rotation_deviation});
    loader.result.loaded_constraints.push_back(id);
  }

  const auto validation = validate_board(candidate);
  if (!validation.ok()) {
    for (const auto& issue : validation.issues) {
      if (issue.severity == IssueSeverity::error) {
        loader.error("invalid_applied_relative_placement_constraints",
                     "applied sidecar violates canonical rule: " + issue.code, root.location,
                     issue.object_ids);
      }
    }
  }
  if (loader.result.has_errors()) {
    loader.result.loaded_constraints.clear();
    return std::move(loader.result);
  }
  std::sort(loader.result.loaded_constraints.begin(), loader.result.loaded_constraints.end());
  loader.result.board = std::move(candidate);
  return std::move(loader.result);
}

RelativePlacementConstraintResult
apply_relative_placement_constraints_file(const Board& source, const std::filesystem::path& path) {
  std::ifstream input(path, std::ios::binary);
  if (!input) {
    RelativePlacementConstraintResult result;
    result.diagnostics.push_back(
        ConstraintDiagnostic{ConstraintSeverity::error,
                             "relative_placement_constraint_file_read_error",
                             "unable to read relative-placement constraint sidecar",
                             1,
                             1,
                             {path.string()}});
    return result;
  }
  const std::string contents{std::istreambuf_iterator<char>{input},
                             std::istreambuf_iterator<char>{}};
  return apply_relative_placement_constraints(source, contents);
}

std::string make_relative_placement_constraints_template(const Board& source) {
  return "(flux_relative_placement_constraints\n  (version 2)\n  (source_revision " +
         quote(source.source_revision) + ")\n)\n";
}

} // namespace flux::io
