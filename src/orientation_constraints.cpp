#include "flux/io/orientation_constraints.hpp"

#include "flux/core/validation.hpp"
#include "flux/io/sexpr.hpp"

#include <algorithm>
#include <charconv>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <optional>
#include <set>
#include <string>
#include <string_view>
#include <system_error>
#include <utility>
#include <vector>

namespace flux::io {
namespace {

constexpr std::string_view sidecar_head = "flux_orientation_constraints";

struct Loader {
  OrientationConstraintResult result;

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

[[nodiscard]] std::optional<double> parse_double(std::string_view value) {
  double result = 0.0;
  const auto parsed = std::from_chars(value.data(), value.data() + value.size(), result,
                                      std::chars_format::general);
  return parsed.ec == std::errc{} && parsed.ptr == value.data() + value.size() &&
                 std::isfinite(result)
             ? std::optional<double>{result}
             : std::nullopt;
}

[[nodiscard]] const SExpression* unique_child(const SExpression& parent, std::string_view name,
                                              Loader& loader, bool required = true) {
  const auto children = find_children(parent, name);
  if (children.empty()) {
    if (required) {
      loader.error("missing_constraint_field", "missing required " + std::string(name) + " field",
                   parent.location);
    }
    return nullptr;
  }
  if (children.size() != 1U) {
    loader.error("duplicate_constraint_field", std::string(name) + " field must be unique",
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
      loader.error("unknown_constraint_field", "constraint sidecar contains an unknown field",
                   child.location);
    }
  }
}

[[nodiscard]] std::string quote(std::string_view value) {
  std::string result{"\""};
  for (const auto character : value) {
    switch (character) {
    case '\\':
      result += "\\\\";
      break;
    case '"':
      result += "\\\"";
      break;
    case '\n':
      result += "\\n";
      break;
    case '\r':
      result += "\\r";
      break;
    case '\t':
      result += "\\t";
      break;
    default:
      result.push_back(character);
      break;
    }
  }
  result.push_back('"');
  return result;
}

[[nodiscard]] std::string angle_text(Microdegrees angle) {
  auto value = std::to_string(angle.value());
  if (value.size() <= 6U) {
    value.insert(0, 7U - value.size(), '0');
  }
  value.insert(value.size() - 6U, 1U, '.');
  while (value.back() == '0') {
    value.pop_back();
  }
  if (value.back() == '.') {
    value.pop_back();
  }
  return value;
}

} // namespace

bool OrientationConstraintResult::has_errors() const {
  return std::any_of(diagnostics.begin(), diagnostics.end(), [](const auto& diagnostic) {
    return diagnostic.severity == ConstraintSeverity::error;
  });
}

OrientationConstraintResult apply_orientation_constraints(const Board& source,
                                                          std::string_view contents) {
  Loader loader;
  SExpressionParseOptions parse_options;
  parse_options.maximum_depth = 32;
  parse_options.maximum_nodes = 100'000;
  const auto parsed = parse_sexpression(contents, parse_options);
  for (const auto& diagnostic : parsed.diagnostics) {
    loader.error(diagnostic.code, diagnostic.message, diagnostic.location);
  }
  if (!parsed.root || !parsed.diagnostics.empty()) {
    return std::move(loader.result);
  }
  const auto& root = *parsed.root;
  if (head(root) != sidecar_head) {
    loader.error("invalid_constraint_root",
                 "constraint sidecar root must be flux_orientation_constraints", root.location);
    return std::move(loader.result);
  }
  reject_unknown_children(root, {"version", "source_revision", "footprint"}, loader);
  const auto* version = unique_child(root, "version", loader);
  const auto* revision = unique_child(root, "source_revision", loader);
  if (version == nullptr || atom_at(*version, 1) != std::optional<std::string_view>{"1"} ||
      version->children.size() != 2U) {
    loader.error("unsupported_constraint_version", "constraint sidecar version must be 1",
                 version == nullptr ? root.location : version->location);
  }
  if (revision == nullptr || !atom_at(*revision, 1) || revision->children.size() != 2U) {
    loader.error("invalid_constraint_revision", "source_revision requires exactly one value",
                 revision == nullptr ? root.location : revision->location);
  } else if (*atom_at(*revision, 1) != source.source_revision) {
    loader.error("stale_constraint_snapshot",
                 "constraint sidecar was generated for a different board snapshot",
                 revision->location);
  }

  auto candidate = source;
  std::set<ObjectId> seen;
  for (const auto* entry : find_children(root, "footprint")) {
    reject_unknown_children(*entry, {"id", "reference", "allowed_rotations", "allowed_sides"},
                            loader);
    const auto* id_field = unique_child(*entry, "id", loader);
    const auto* reference_field = unique_child(*entry, "reference", loader, false);
    const auto* rotations_field = unique_child(*entry, "allowed_rotations", loader);
    const auto* sides_field = unique_child(*entry, "allowed_sides", loader);
    const auto id_value = id_field == nullptr ? std::nullopt : atom_at(*id_field, 1);
    if (!id_value || id_field->children.size() != 2U || id_value->empty()) {
      loader.error("invalid_constraint_footprint_id", "footprint id requires one non-empty value",
                   id_field == nullptr ? entry->location : id_field->location);
      continue;
    }
    const ObjectId id{std::string(*id_value)};
    if (!seen.insert(id).second) {
      loader.error("duplicate_constraint_footprint", "footprint appears more than once",
                   id_field->location, {id.value()});
      continue;
    }
    const auto found =
        std::find_if(candidate.footprints.begin(), candidate.footprints.end(),
                     [&id](const Footprint& footprint) { return footprint.id == id; });
    if (found == candidate.footprints.end()) {
      loader.error("unknown_constraint_footprint", "constraint references an unknown footprint",
                   id_field->location, {id.value()});
      continue;
    }
    if (reference_field != nullptr) {
      const auto reference = atom_at(*reference_field, 1);
      if (!reference || reference_field->children.size() != 2U || *reference != found->reference) {
        loader.error("constraint_reference_mismatch",
                     "footprint reference does not match the bound stable ID",
                     reference_field->location, {id.value(), found->reference});
      }
    }
    FootprintOrientationConstraints constraints;
    constraints.allow_front = false;
    constraints.allow_back = false;
    if (rotations_field != nullptr) {
      if (rotations_field->children.size() < 2U) {
        loader.error("empty_allowed_rotations", "allowed_rotations requires at least one value",
                     rotations_field->location, {id.value()});
      }
      std::set<std::int64_t> rotations;
      for (std::size_t index = 1; index < rotations_field->children.size(); ++index) {
        const auto token = atom_at(*rotations_field, index);
        std::optional<Microdegrees> angle;
        if (token && *token == "current") {
          angle = found->rotation;
        } else if (token) {
          const auto degrees = parse_double(*token);
          if (degrees) {
            try {
              angle = Microdegrees::from_degrees(*degrees);
            } catch (const std::out_of_range&) {
            }
          }
        }
        if (!angle) {
          loader.error("invalid_allowed_rotation",
                       "allowed rotation must be current or a finite degree value",
                       rotations_field->children[index].location, {id.value()});
          continue;
        }
        if (!rotations.insert(angle->value()).second) {
          loader.error("duplicate_allowed_rotation",
                       "allowed rotations must be unique after normalization",
                       rotations_field->children[index].location, {id.value()});
          continue;
        }
        constraints.allowed_rotations.push_back(*angle);
      }
      if (!rotations.contains(found->rotation.value())) {
        loader.error("current_rotation_not_allowed",
                     "sidecar must include the footprint's current rotation",
                     rotations_field->location, {id.value()});
      }
    }
    if (sides_field != nullptr) {
      if (sides_field->children.size() < 2U) {
        loader.error("empty_allowed_sides", "allowed_sides requires at least one value",
                     sides_field->location, {id.value()});
      }
      std::set<BoardSide> sides;
      for (std::size_t index = 1; index < sides_field->children.size(); ++index) {
        const auto token = atom_at(*sides_field, index);
        std::optional<BoardSide> side;
        if (token && *token == "current") {
          side = found->side;
        } else if (token && *token == "front") {
          side = BoardSide::front;
        } else if (token && *token == "back") {
          side = BoardSide::back;
        }
        if (!side) {
          loader.error("invalid_allowed_side", "allowed side must be current, front, or back",
                       sides_field->children[index].location, {id.value()});
          continue;
        }
        if (!sides.insert(*side).second) {
          loader.error("duplicate_allowed_side", "allowed sides must be unique",
                       sides_field->children[index].location, {id.value()});
          continue;
        }
        constraints.allow_front = constraints.allow_front || *side == BoardSide::front;
        constraints.allow_back = constraints.allow_back || *side == BoardSide::back;
      }
      if (!sides.contains(found->side)) {
        loader.error("current_side_not_allowed",
                     "sidecar must include the footprint's current side", sides_field->location,
                     {id.value()});
      }
    }
    found->orientation_constraints = std::move(constraints);
    loader.result.constrained_footprints.push_back(id);
  }
  if (loader.result.constrained_footprints.empty()) {
    loader.error("empty_constraint_scope", "constraint sidecar must name at least one footprint",
                 root.location);
  }
  if (!loader.result.has_errors()) {
    const auto validation = validate_board(candidate);
    if (!validation.ok()) {
      loader.error("constrained_board_invalid",
                   "applying the sidecar makes the canonical board invalid", root.location);
    }
  }
  if (!loader.result.has_errors()) {
    loader.result.board = std::move(candidate);
  }
  return std::move(loader.result);
}

OrientationConstraintResult apply_orientation_constraints_file(const Board& source,
                                                               const std::filesystem::path& path) {
  std::ifstream input(path, std::ios::binary);
  if (!input) {
    OrientationConstraintResult result;
    result.diagnostics.push_back(ConstraintDiagnostic{ConstraintSeverity::error,
                                                      "constraint_file_read_error",
                                                      "constraint sidecar could not be opened",
                                                      1,
                                                      1,
                                                      {path.string()}});
    return result;
  }
  const std::string contents{std::istreambuf_iterator<char>{input},
                             std::istreambuf_iterator<char>{}};
  return apply_orientation_constraints(source, contents);
}

std::string make_orientation_constraints_template(const Board& source) {
  std::vector<const Footprint*> footprints;
  footprints.reserve(source.footprints.size());
  for (const auto& footprint : source.footprints) {
    footprints.push_back(&footprint);
  }
  std::sort(footprints.begin(), footprints.end(),
            [](const Footprint* lhs, const Footprint* rhs) { return lhs->id < rhs->id; });
  std::string result{"(flux_orientation_constraints\n  (version 1)\n  (source_revision "};
  result += quote(source.source_revision);
  result += ")";
  for (const auto* footprint : footprints) {
    result += "\n  (footprint\n    (id " + quote(footprint->id.value()) + ")\n";
    result += "    (reference " + quote(footprint->reference) + ")\n";
    result += "    (allowed_rotations current) ; current = " + angle_text(footprint->rotation) +
              " degrees\n";
    result += std::string{"    (allowed_sides current) ; current = "} +
              (footprint->side == BoardSide::front ? "front" : "back") + "\n  )";
  }
  result += "\n)\n";
  return result;
}

} // namespace flux::io
