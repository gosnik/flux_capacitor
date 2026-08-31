#include "flux/io/placement_group_constraints.hpp"

#include "flux/core/validation.hpp"
#include "flux/io/sexpr.hpp"

#include <algorithm>
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

constexpr std::string_view sidecar_head = "flux_placement_group_constraints";

struct Loader {
  PlacementGroupConstraintResult result;

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
    loader.error("missing_placement_group_field",
                 "missing required " + std::string(name) + " field", parent.location);
    return nullptr;
  }
  if (children.size() != 1U) {
    loader.error("duplicate_placement_group_field", std::string(name) + " field must be unique",
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
      loader.error("unknown_placement_group_field",
                   "placement-group sidecar contains an unknown field", child.location);
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

bool PlacementGroupConstraintResult::has_errors() const {
  return std::any_of(diagnostics.begin(), diagnostics.end(), [](const auto& diagnostic) {
    return diagnostic.severity == ConstraintSeverity::error;
  });
}

PlacementGroupConstraintResult apply_placement_group_constraints(const Board& source,
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
    loader.error("invalid_placement_group_root",
                 "sidecar root must be flux_placement_group_constraints", root.location);
    return std::move(loader.result);
  }
  reject_unknown_children(root, {"version", "source_revision", "group"}, loader);
  const auto* version = unique_child(root, "version", loader);
  const auto* revision = unique_child(root, "source_revision", loader);
  if (version == nullptr || atom_at(*version, 1) != std::optional<std::string_view>{"1"} ||
      version->children.size() != 2U) {
    loader.error("unsupported_placement_group_version", "placement-group sidecar version must be 1",
                 version == nullptr ? root.location : version->location);
  }
  const auto revision_value = revision == nullptr ? std::nullopt : atom_at(*revision, 1);
  if (!revision_value || revision->children.size() != 2U) {
    loader.error("invalid_placement_group_revision", "source_revision requires exactly one value",
                 revision == nullptr ? root.location : revision->location);
  } else if (*revision_value != source.source_revision) {
    loader.error("stale_placement_group_constraints",
                 "placement-group sidecar targets a different board revision", revision->location);
  }

  Board candidate = source;
  candidate.placement_groups.clear();
  std::map<ObjectId, const Footprint*> footprint_by_id;
  for (const auto& footprint : candidate.footprints) {
    footprint_by_id.emplace(footprint.id, &footprint);
  }
  std::set<PlacementGroupId> seen_groups;
  std::set<ObjectId> grouped_footprints;
  for (const auto* entry : find_children(root, "group")) {
    reject_unknown_children(*entry, {"id", "name", "members"}, loader);
    const auto* id_field = unique_child(*entry, "id", loader);
    const auto* name_field = unique_child(*entry, "name", loader);
    const auto* members_field = unique_child(*entry, "members", loader);
    const auto id_value = id_field == nullptr ? std::nullopt : atom_at(*id_field, 1);
    const auto name_value = name_field == nullptr ? std::nullopt : atom_at(*name_field, 1);
    if (!id_value || id_field->children.size() != 2U || id_value->empty()) {
      loader.error("invalid_placement_group_id", "group id requires one non-empty value",
                   id_field == nullptr ? entry->location : id_field->location);
      continue;
    }
    const PlacementGroupId id{std::string(*id_value)};
    if (!seen_groups.insert(id).second) {
      loader.error("duplicate_placement_group_constraint", "group appears more than once",
                   id_field->location, {id.value()});
      continue;
    }
    if (!name_value || name_field->children.size() != 2U || name_value->empty()) {
      loader.error("invalid_placement_group_name", "group name requires one non-empty value",
                   name_field == nullptr ? entry->location : name_field->location, {id.value()});
      continue;
    }

    std::vector<ObjectId> members;
    std::set<ObjectId> group_members;
    if (members_field == nullptr || members_field->children.size() < 3U) {
      loader.error(
          "placement_group_constraint_too_small", "members requires at least two footprint entries",
          members_field == nullptr ? entry->location : members_field->location, {id.value()});
    } else {
      for (std::size_t index = 1U; index < members_field->children.size(); ++index) {
        const auto& member = members_field->children[index];
        if (!member.is_list() || head(member) != "footprint") {
          loader.error("invalid_placement_group_member", "members must contain footprint records",
                       member.location, {id.value()});
          continue;
        }
        reject_unknown_children(member, {"id", "reference"}, loader);
        const auto* member_id_field = unique_child(member, "id", loader);
        const auto* reference_field = unique_child(member, "reference", loader);
        const auto member_id_value =
            member_id_field == nullptr ? std::nullopt : atom_at(*member_id_field, 1);
        if (!member_id_value || member_id_field->children.size() != 2U ||
            member_id_value->empty()) {
          loader.error("invalid_placement_group_member_id",
                       "member footprint id requires one non-empty value",
                       member_id_field == nullptr ? member.location : member_id_field->location,
                       {id.value()});
          continue;
        }
        const ObjectId member_id{std::string(*member_id_value)};
        if (!group_members.insert(member_id).second) {
          loader.error("duplicate_placement_group_constraint_member",
                       "a footprint may appear once in a group", member_id_field->location,
                       {id.value(), member_id.value()});
          continue;
        }
        const auto found = footprint_by_id.find(member_id);
        if (found == footprint_by_id.end()) {
          loader.error("unknown_placement_group_constraint_footprint",
                       "group references an unknown footprint", member_id_field->location,
                       {id.value(), member_id.value()});
          continue;
        }
        const auto reference =
            reference_field == nullptr ? std::nullopt : atom_at(*reference_field, 1);
        if (!reference || reference_field->children.size() != 2U ||
            *reference != found->second->reference) {
          loader.error("placement_group_reference_mismatch",
                       "member reference does not match the bound stable footprint ID",
                       reference_field == nullptr ? member.location : reference_field->location,
                       {id.value(), member_id.value(), found->second->reference});
        }
        if (!grouped_footprints.insert(member_id).second) {
          loader.error("overlapping_placement_group_constraint_membership",
                       "a footprint may belong to at most one placement group",
                       member_id_field->location, {id.value(), member_id.value()});
        }
        members.push_back(member_id);
      }
    }
    candidate.placement_groups.push_back(
        PlacementGroup{id, std::string(*name_value), std::move(members)});
    loader.result.loaded_groups.push_back(id);
  }

  const auto validation = validate_board(candidate);
  if (!validation.ok()) {
    for (const auto& issue : validation.issues) {
      if (issue.severity == IssueSeverity::error) {
        loader.error("invalid_applied_placement_group_constraints",
                     "applied sidecar violates canonical rule: " + issue.code, root.location,
                     issue.object_ids);
      }
    }
  }
  if (loader.result.has_errors()) {
    loader.result.loaded_groups.clear();
    return std::move(loader.result);
  }
  std::sort(loader.result.loaded_groups.begin(), loader.result.loaded_groups.end());
  loader.result.board = std::move(candidate);
  return std::move(loader.result);
}

PlacementGroupConstraintResult
apply_placement_group_constraints_file(const Board& source, const std::filesystem::path& path) {
  std::ifstream input(path, std::ios::binary);
  if (!input) {
    PlacementGroupConstraintResult result;
    result.diagnostics.push_back(
        ConstraintDiagnostic{ConstraintSeverity::error,
                             "placement_group_constraint_file_read_error",
                             "unable to read placement-group constraint sidecar",
                             1,
                             1,
                             {path.string()}});
    return result;
  }
  const std::string contents{std::istreambuf_iterator<char>{input},
                             std::istreambuf_iterator<char>{}};
  return apply_placement_group_constraints(source, contents);
}

std::string make_placement_group_constraints_template(const Board& source) {
  return "(flux_placement_group_constraints\n  (version 1)\n  (source_revision " +
         quote(source.source_revision) + ")\n)\n";
}

} // namespace flux::io
