#include "flux/io/placement_envelope_constraints.hpp"

#include "flux/core/geometry.hpp"
#include "flux/core/copper_geometry.hpp"
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
#include <limits>
#include <optional>
#include <set>
#include <string>
#include <string_view>
#include <system_error>
#include <utility>
#include <vector>

namespace flux::io {
namespace {

constexpr std::string_view sidecar_head = "flux_placement_envelopes";

struct Loader {
  PlacementEnvelopeConstraintResult result;

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
  std::int64_t result = 0;
  const auto parsed = std::from_chars(value.data(), value.data() + value.size(), result);
  return parsed.ec == std::errc{} && parsed.ptr == value.data() + value.size()
             ? std::optional<std::int64_t>{result}
             : std::nullopt;
}

[[nodiscard]] const SExpression* unique_child(const SExpression& parent, std::string_view name,
                                              Loader& loader, bool required = true) {
  const auto children = find_children(parent, name);
  if (children.empty()) {
    if (required) {
      loader.error("missing_envelope_field", "missing required " + std::string(name) + " field",
                   parent.location);
    }
    return nullptr;
  }
  if (children.size() != 1U) {
    loader.error("duplicate_envelope_field", std::string(name) + " field must be unique",
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
      loader.error("unknown_envelope_field", "placement-envelope sidecar contains an unknown field",
                   child.location);
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

[[nodiscard]] Polygon suggested_pad_envelope(const Footprint& footprint) {
  constexpr std::int64_t default_half_extent = 500'000;
  constexpr std::int64_t review_margin = 250'000;
  if (footprint.pads.empty()) {
    return Polygon{{Point{Nanometres{-default_half_extent}, Nanometres{-default_half_extent}},
                    Point{Nanometres{default_half_extent}, Nanometres{-default_half_extent}},
                    Point{Nanometres{default_half_extent}, Nanometres{default_half_extent}},
                    Point{Nanometres{-default_half_extent}, Nanometres{default_half_extent}}}};
  }
  auto min_x = std::numeric_limits<std::int64_t>::max();
  auto max_x = std::numeric_limits<std::int64_t>::min();
  auto min_y = std::numeric_limits<std::int64_t>::max();
  auto max_y = std::numeric_limits<std::int64_t>::min();
  for (const auto& pad : footprint.pads) {
    const auto radius = static_cast<std::int64_t>(
        std::ceil(conservative_pad_radius_nm(pad)));
    min_x = std::min(min_x, pad.local_position.x.value() - radius);
    max_x = std::max(max_x, pad.local_position.x.value() + radius);
    min_y = std::min(min_y, pad.local_position.y.value() - radius);
    max_y = std::max(max_y, pad.local_position.y.value() + radius);
  }
  return Polygon{{Point{Nanometres{min_x - review_margin}, Nanometres{min_y - review_margin}},
                  Point{Nanometres{max_x + review_margin}, Nanometres{min_y - review_margin}},
                  Point{Nanometres{max_x + review_margin}, Nanometres{max_y + review_margin}},
                  Point{Nanometres{min_x - review_margin}, Nanometres{max_y + review_margin}}}};
}

} // namespace

bool PlacementEnvelopeConstraintResult::has_errors() const {
  return std::any_of(diagnostics.begin(), diagnostics.end(), [](const auto& diagnostic) {
    return diagnostic.severity == ConstraintSeverity::error;
  });
}

PlacementEnvelopeConstraintResult apply_placement_envelope_constraints(const Board& source,
                                                                       std::string_view contents) {
  Loader loader;
  SExpressionParseOptions parse_options;
  parse_options.maximum_depth = 32;
  parse_options.maximum_nodes = 1'000'000;
  const auto parsed = parse_sexpression(contents, parse_options);
  for (const auto& diagnostic : parsed.diagnostics) {
    loader.error(diagnostic.code, diagnostic.message, diagnostic.location);
  }
  if (!parsed.root || !parsed.diagnostics.empty()) {
    return std::move(loader.result);
  }
  const auto& root = *parsed.root;
  if (head(root) != sidecar_head) {
    loader.error("invalid_envelope_root", "sidecar root must be flux_placement_envelopes",
                 root.location);
    return std::move(loader.result);
  }
  reject_unknown_children(root, {"version", "source_revision", "footprint"}, loader);
  const auto* version = unique_child(root, "version", loader);
  const auto* revision = unique_child(root, "source_revision", loader);
  if (version == nullptr || atom_at(*version, 1) != std::optional<std::string_view>{"1"} ||
      version->children.size() != 2U) {
    loader.error("unsupported_envelope_version", "placement-envelope sidecar version must be 1",
                 version == nullptr ? root.location : version->location);
  }
  if (revision == nullptr || !atom_at(*revision, 1) || revision->children.size() != 2U) {
    loader.error("invalid_envelope_revision", "source_revision requires exactly one value",
                 revision == nullptr ? root.location : revision->location);
  } else if (*atom_at(*revision, 1) != source.source_revision) {
    loader.error("stale_envelope_snapshot",
                 "placement envelopes were generated for a different board snapshot",
                 revision->location);
  }

  auto candidate = source;
  std::set<ObjectId> seen;
  for (const auto* entry : find_children(root, "footprint")) {
    reject_unknown_children(*entry, {"id", "reference", "approved", "polygon"}, loader);
    const auto* id_field = unique_child(*entry, "id", loader);
    const auto* reference_field = unique_child(*entry, "reference", loader);
    const auto* approved_field = unique_child(*entry, "approved", loader);
    const auto* polygon_field = unique_child(*entry, "polygon", loader);
    const auto id_value = id_field == nullptr ? std::nullopt : atom_at(*id_field, 1);
    if (!id_value || id_field->children.size() != 2U || id_value->empty()) {
      loader.error("invalid_envelope_footprint_id", "footprint id requires one non-empty value",
                   id_field == nullptr ? entry->location : id_field->location);
      continue;
    }
    const ObjectId id{std::string(*id_value)};
    if (!seen.insert(id).second) {
      loader.error("duplicate_envelope_footprint", "footprint appears more than once",
                   id_field->location, {id.value()});
      continue;
    }
    const auto found =
        std::find_if(candidate.footprints.begin(), candidate.footprints.end(),
                     [&id](const Footprint& footprint) { return footprint.id == id; });
    if (found == candidate.footprints.end()) {
      loader.error("unknown_envelope_footprint", "sidecar references an unknown footprint",
                   id_field->location, {id.value()});
      continue;
    }
    const auto reference = reference_field == nullptr ? std::nullopt : atom_at(*reference_field, 1);
    if (!reference || reference_field->children.size() != 2U || *reference != found->reference) {
      loader.error("envelope_reference_mismatch",
                   "footprint reference does not match the bound stable ID",
                   reference_field == nullptr ? entry->location : reference_field->location,
                   {id.value(), found->reference});
    }
    if (found->has_exact_placement_envelope()) {
      loader.error("envelope_already_present",
                   "sidecar may only supply a footprint missing source courtyard geometry",
                   entry->location, {id.value()});
    }
    if (approved_field == nullptr || approved_field->children.size() != 2U ||
        atom_at(*approved_field, 1) != std::optional<std::string_view>{"yes"}) {
      loader.error("envelope_not_approved",
                   "every suggested envelope requires explicit `approved yes` review",
                   approved_field == nullptr ? entry->location : approved_field->location,
                   {id.value()});
    }
    Polygon polygon;
    if (polygon_field != nullptr) {
      for (std::size_t index = 1; index < polygon_field->children.size(); ++index) {
        const auto& point = polygon_field->children[index];
        const auto x = atom_at(point, 1);
        const auto y = atom_at(point, 2);
        const auto parsed_x = x ? parse_integer(*x) : std::nullopt;
        const auto parsed_y = y ? parse_integer(*y) : std::nullopt;
        if (!point.is_list() || head(point) != "xy" || point.children.size() != 3U || !parsed_x ||
            !parsed_y) {
          loader.error("invalid_envelope_vertex",
                       "polygon vertices must be `(xy INTEGER_NM INTEGER_NM)`", point.location,
                       {id.value()});
          continue;
        }
        polygon.vertices.push_back(Point{Nanometres{*parsed_x}, Nanometres{*parsed_y}});
      }
    }
    if (!geometry::is_simple_polygon(polygon)) {
      loader.error("invalid_envelope_polygon",
                   "placement envelope must be a simple non-zero-area local polygon",
                   polygon_field == nullptr ? entry->location : polygon_field->location,
                   {id.value()});
      continue;
    }
    found->courtyard = std::move(polygon);
    found->courtyard_circles.clear();
    found->placement_envelope_source = PlacementEnvelopeSource::external_constraint;
    loader.result.constrained_footprints.push_back(id);
  }

  for (const auto& footprint : candidate.footprints) {
    if (!footprint.has_exact_placement_envelope() && !seen.contains(footprint.id)) {
      loader.error("missing_envelope_footprint",
                   "sidecar must cover every footprint without an exact source courtyard",
                   root.location, {footprint.id.value(), footprint.reference});
    }
  }
  if (!loader.result.has_errors()) {
    const auto validation = validate_board(candidate);
    if (!validation.ok()) {
      loader.error("enveloped_board_invalid",
                   "approved placement envelopes make the canonical board invalid", root.location);
    }
  }
  if (!loader.result.has_errors()) {
    loader.result.board = std::move(candidate);
  }
  return std::move(loader.result);
}

PlacementEnvelopeConstraintResult
apply_placement_envelope_constraints_file(const Board& source, const std::filesystem::path& path) {
  std::ifstream input(path, std::ios::binary);
  if (!input) {
    PlacementEnvelopeConstraintResult result;
    result.diagnostics.push_back(
        ConstraintDiagnostic{ConstraintSeverity::error,
                             "envelope_file_read_error",
                             "placement-envelope sidecar could not be opened",
                             1,
                             1,
                             {path.string()}});
    return result;
  }
  const std::string contents{std::istreambuf_iterator<char>{input},
                             std::istreambuf_iterator<char>{}};
  return apply_placement_envelope_constraints(source, contents);
}

std::string make_placement_envelope_constraints_template(const Board& source) {
  std::vector<const Footprint*> missing;
  for (const auto& footprint : source.footprints) {
    if (!footprint.has_exact_placement_envelope()) {
      missing.push_back(&footprint);
    }
  }
  std::sort(missing.begin(), missing.end(),
            [](const Footprint* lhs, const Footprint* rhs) { return lhs->id < rhs->id; });
  std::string result{"(flux_placement_envelopes\n  (version 1)\n  (source_revision "};
  result += quote(source.source_revision);
  result += ")";
  for (const auto* footprint : missing) {
    const auto suggestion = suggested_pad_envelope(*footprint);
    result += "\n  (footprint\n    (id " + quote(footprint->id.value()) + ")\n";
    result += "    (reference " + quote(footprint->reference) + ")\n";
    result += "    (approved no) ; review against the exact body/assembly envelope\n";
    result += "    (polygon";
    for (const auto point : suggestion.vertices) {
      result += "\n      (xy " + std::to_string(point.x.value()) + " " +
                std::to_string(point.y.value()) + ")";
    }
    result += "\n    )\n  )";
  }
  result += "\n)\n";
  return result;
}

} // namespace flux::io
