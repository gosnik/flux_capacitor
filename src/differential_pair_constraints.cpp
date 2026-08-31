#include "flux/io/differential_pair_constraints.hpp"

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
#include <vector>

namespace flux::io {
namespace {

constexpr std::string_view sidecar_head = "flux_differential_pair_constraints";

struct Loader {
  DifferentialPairConstraintResult result;

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
    loader.error("missing_differential_pair_constraint_field",
                 "missing required " + std::string(name) + " field", parent.location);
    return nullptr;
  }
  if (children.size() != 1U) {
    loader.error("duplicate_differential_pair_constraint_field",
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
      loader.error("unknown_differential_pair_constraint_field",
                   "differential-pair sidecar contains an unknown field", child.location);
    }
  }
}

[[nodiscard]] std::optional<Nanometres>
parse_optional_dimension(const SExpression* field, Loader& loader, const DifferentialPairId& id) {
  if (field == nullptr) {
    return std::nullopt;
  }
  const auto token = atom_at(*field, 1);
  if (!token || field->children.size() != 2U) {
    loader.error("invalid_differential_pair_constraint_dimension",
                 "constraint dimensions require exactly one integer nanometre value or none",
                 field->location, {id.value()});
    return std::nullopt;
  }
  if (*token == "none") {
    return std::nullopt;
  }
  std::int64_t value = 0;
  const auto parsed = std::from_chars(token->data(), token->data() + token->size(), value);
  if (parsed.ec != std::errc{} || parsed.ptr != token->data() + token->size() || value < 0 ||
      value > max_abs_coordinate_nm) {
    loader.error("invalid_differential_pair_constraint_dimension",
                 "constraint dimensions must be none or a non-negative bounded integer",
                 field->location, {id.value()});
    return std::nullopt;
  }
  return Nanometres{value};
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

[[nodiscard]] const Net* find_net(const Board& board, const NetId& id) {
  const auto found = std::find_if(board.nets.begin(), board.nets.end(),
                                  [&id](const Net& net) { return net.id == id; });
  return found == board.nets.end() ? nullptr : &*found;
}

} // namespace

bool DifferentialPairConstraintResult::has_errors() const {
  return std::any_of(diagnostics.begin(), diagnostics.end(), [](const auto& diagnostic) {
    return diagnostic.severity == ConstraintSeverity::error;
  });
}

DifferentialPairConstraintResult apply_differential_pair_constraints(const Board& source,
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
    loader.error("invalid_differential_pair_constraint_root",
                 "sidecar root must be flux_differential_pair_constraints", root.location);
    return std::move(loader.result);
  }
  reject_unknown_children(root, {"version", "source_revision", "pair"}, loader);
  const auto* version = unique_child(root, "version", loader);
  const auto* revision = unique_child(root, "source_revision", loader);
  if (version == nullptr || atom_at(*version, 1) != std::optional<std::string_view>{"1"} ||
      version->children.size() != 2U) {
    loader.error("unsupported_differential_pair_constraint_version",
                 "differential-pair sidecar version must be 1",
                 version == nullptr ? root.location : version->location);
  }
  const auto revision_value = revision == nullptr ? std::nullopt : atom_at(*revision, 1);
  if (!revision_value || revision->children.size() != 2U) {
    loader.error("invalid_differential_pair_constraint_revision",
                 "source_revision requires exactly one value",
                 revision == nullptr ? root.location : revision->location);
  } else if (*revision_value != source.source_revision) {
    loader.error("stale_differential_pair_constraints",
                 "differential-pair sidecar targets a different board revision",
                 revision->location);
  }

  Board candidate = source;
  std::set<DifferentialPairId> seen;
  for (const auto* entry : find_children(root, "pair")) {
    reject_unknown_children(
        *entry, {"id", "name", "positive_net", "negative_net", "minimum_gap_nm", "maximum_skew_nm"},
        loader);
    const auto* id_field = unique_child(*entry, "id", loader);
    const auto id_value = id_field == nullptr ? std::nullopt : atom_at(*id_field, 1);
    if (!id_value || id_field->children.size() != 2U || id_value->empty()) {
      loader.error("invalid_differential_pair_constraint_id",
                   "pair id requires exactly one non-empty value",
                   id_field == nullptr ? entry->location : id_field->location);
      continue;
    }
    const DifferentialPairId id{std::string(*id_value)};
    if (!seen.insert(id).second) {
      loader.error("duplicate_differential_pair_constraint", "pair appears more than once",
                   id_field->location, {id.value()});
      continue;
    }
    const auto found =
        std::find_if(candidate.differential_pairs.begin(), candidate.differential_pairs.end(),
                     [&id](const DifferentialPair& pair) { return pair.id == id; });
    if (found == candidate.differential_pairs.end()) {
      loader.error("unknown_differential_pair_constraint", "sidecar references an unknown pair",
                   id_field->location, {id.value()});
      continue;
    }
    const auto* name_field = unique_child(*entry, "name", loader);
    const auto* positive_field = unique_child(*entry, "positive_net", loader);
    const auto* negative_field = unique_child(*entry, "negative_net", loader);
    const auto name = name_field == nullptr ? std::nullopt : atom_at(*name_field, 1);
    const auto positive = positive_field == nullptr ? std::nullopt : atom_at(*positive_field, 1);
    const auto negative = negative_field == nullptr ? std::nullopt : atom_at(*negative_field, 1);
    const auto* positive_net = find_net(candidate, found->positive_net);
    const auto* negative_net = find_net(candidate, found->negative_net);
    if (!name || name_field->children.size() != 2U || *name != found->name || !positive ||
        positive_field->children.size() != 2U || positive_net == nullptr ||
        *positive != positive_net->name || !negative || negative_field->children.size() != 2U ||
        negative_net == nullptr || *negative != negative_net->name) {
      loader.error("differential_pair_constraint_identity_mismatch",
                   "pair name or polarity nets do not match the bound stable ID", entry->location,
                   {id.value()});
    }
    found->minimum_gap =
        parse_optional_dimension(unique_child(*entry, "minimum_gap_nm", loader), loader, id);
    found->maximum_skew =
        parse_optional_dimension(unique_child(*entry, "maximum_skew_nm", loader), loader, id);
    if (found->minimum_gap || found->maximum_skew) {
      loader.result.constrained_pairs.push_back(id);
    }
  }
  for (const auto& pair : candidate.differential_pairs) {
    if (!seen.contains(pair.id)) {
      loader.error("missing_differential_pair_constraint",
                   "sidecar must cover every current differential pair", root.location,
                   {pair.id.value()});
    }
  }
  const auto validation = validate_board(candidate);
  if (!validation.ok()) {
    for (const auto& issue : validation.issues) {
      if (issue.severity == IssueSeverity::error) {
        loader.error("invalid_applied_differential_pair_constraints",
                     "applied sidecar violates canonical rule: " + issue.code, root.location,
                     issue.object_ids);
      }
    }
  }
  if (loader.result.has_errors()) {
    loader.result.constrained_pairs.clear();
    return std::move(loader.result);
  }
  std::sort(loader.result.constrained_pairs.begin(), loader.result.constrained_pairs.end());
  loader.result.board = std::move(candidate);
  return std::move(loader.result);
}

DifferentialPairConstraintResult
apply_differential_pair_constraints_file(const Board& source, const std::filesystem::path& path) {
  std::ifstream input(path, std::ios::binary);
  if (!input) {
    DifferentialPairConstraintResult result;
    result.diagnostics.push_back(
        ConstraintDiagnostic{ConstraintSeverity::error,
                             "differential_pair_constraint_file_read_error",
                             "unable to read differential-pair constraint sidecar",
                             1,
                             1,
                             {path.string()}});
    return result;
  }
  const std::string contents{std::istreambuf_iterator<char>{input},
                             std::istreambuf_iterator<char>{}};
  return apply_differential_pair_constraints(source, contents);
}

std::string make_differential_pair_constraints_template(const Board& source) {
  std::vector<const DifferentialPair*> pairs;
  pairs.reserve(source.differential_pairs.size());
  for (const auto& pair : source.differential_pairs) {
    pairs.push_back(&pair);
  }
  std::sort(pairs.begin(), pairs.end(),
            [](const auto* lhs, const auto* rhs) { return lhs->id < rhs->id; });
  std::string result = "(flux_differential_pair_constraints\n  (version 1)\n  (source_revision " +
                       quote(source.source_revision) + ")\n";
  for (const auto* pair : pairs) {
    const auto* positive = find_net(source, pair->positive_net);
    const auto* negative = find_net(source, pair->negative_net);
    result += "  (pair\n    (id " + quote(pair->id.value()) + ")\n    (name " + quote(pair->name) +
              ")\n    (positive_net " + quote(positive == nullptr ? "" : positive->name) +
              ")\n    (negative_net " + quote(negative == nullptr ? "" : negative->name) +
              ")\n    (minimum_gap_nm none)\n    (maximum_skew_nm none)\n  )\n";
  }
  result += ")\n";
  return result;
}

} // namespace flux::io
