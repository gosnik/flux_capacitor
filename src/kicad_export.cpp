#include "flux/io/kicad_export.hpp"

#include "flux/core/validation.hpp"
#include "flux/io/kicad_import.hpp"
#include "flux/io/sexpr.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <limits>
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

struct TextEdit {
  std::size_t begin{0};
  std::size_t end{0};
  std::string replacement;
};

void add_diagnostic(KicadExportResult& result, ExportSeverity severity, std::string code,
                    std::string message, std::vector<std::string> object_ids = {}) {
  result.diagnostics.push_back(
      ExportDiagnostic{severity, std::move(code), std::move(message), std::move(object_ids)});
}

void error(KicadExportResult& result, std::string code, std::string message,
           std::vector<std::string> object_ids = {}) {
  add_diagnostic(result, ExportSeverity::error, std::move(code), std::move(message),
                 std::move(object_ids));
}

void error(KicadChangeSetResult& result, std::string code, std::string message,
           std::vector<std::string> object_ids = {}) {
  result.diagnostics.push_back(ExportDiagnostic{ExportSeverity::error, std::move(code),
                                                std::move(message), std::move(object_ids)});
}

[[nodiscard]] std::optional<std::string_view> atom_at(const SExpression& expression,
                                                      std::size_t index) {
  const auto* value = child_at(expression, index);
  return value != nullptr && value->is_atom() ? std::optional<std::string_view>{value->value}
                                              : std::nullopt;
}

[[nodiscard]] std::string stable_token(const SExpression& expression, std::string fallback) {
  if (const auto* uuid = find_child(expression, "uuid")) {
    if (const auto value = atom_at(*uuid, 1)) {
      return std::string(*value);
    }
  }
  if (const auto* timestamp = find_child(expression, "tstamp")) {
    if (const auto value = atom_at(*timestamp, 1)) {
      return std::string(*value);
    }
  }
  return fallback;
}

[[nodiscard]] bool is_hex(char value) {
  return std::isdigit(static_cast<unsigned char>(value)) != 0 || (value >= 'a' && value <= 'f') ||
         (value >= 'A' && value <= 'F');
}

[[nodiscard]] bool is_uuid(std::string_view value) {
  if (value.size() != 36U) {
    return false;
  }
  for (std::size_t i = 0; i < value.size(); ++i) {
    if (i == 8U || i == 13U || i == 18U || i == 23U) {
      if (value[i] != '-') {
        return false;
      }
    } else if (!is_hex(value[i])) {
      return false;
    }
  }
  return true;
}

[[nodiscard]] bool is_generator_token(std::string_view value) {
  return !value.empty() && std::all_of(value.begin(), value.end(), [](const auto character) {
    return (character >= 'a' && character <= 'z') ||
           std::isdigit(static_cast<unsigned char>(character)) != 0 || character == '_';
  });
}

[[nodiscard]] std::uint64_t fnv1a(std::string_view value, std::uint64_t offset_basis) {
  constexpr std::uint64_t prime = 1099511628211ULL;
  auto hash = offset_basis;
  for (const auto character : value) {
    hash ^= static_cast<unsigned char>(character);
    hash *= prime;
  }
  return hash;
}

[[nodiscard]] std::string deterministic_uuid(std::string_view seed) {
  constexpr std::uint64_t first_basis = 14695981039346656037ULL;
  constexpr std::uint64_t second_basis = 7809847782465536322ULL;
  const auto first = fnv1a(seed, first_basis);
  const auto second = fnv1a(seed, second_basis);
  std::array<unsigned char, 16> bytes{};
  for (std::size_t i = 0; i < 8U; ++i) {
    const auto shift = static_cast<unsigned int>((7U - i) * 8U);
    bytes[i] = static_cast<unsigned char>((first >> shift) & 0xFFU);
    bytes[i + 8U] = static_cast<unsigned char>((second >> shift) & 0xFFU);
  }
  bytes[6] = static_cast<unsigned char>((bytes[6] & 0x0FU) | 0x40U);
  bytes[8] = static_cast<unsigned char>((bytes[8] & 0x3FU) | 0x80U);
  constexpr char digits[] = "0123456789abcdef";
  std::string result;
  result.reserve(36);
  for (std::size_t i = 0; i < bytes.size(); ++i) {
    if (i == 4U || i == 6U || i == 8U || i == 10U) {
      result.push_back('-');
    }
    result.push_back(digits[bytes[i] >> 4U]);
    result.push_back(digits[bytes[i] & 0x0FU]);
  }
  return result;
}

[[nodiscard]] std::string format_fixed(std::int64_t value, std::uint64_t scale) {
  const bool negative = value < 0;
  const auto magnitude =
      negative ? static_cast<std::uint64_t>(-(value + 1)) + 1U : static_cast<std::uint64_t>(value);
  const auto whole = magnitude / scale;
  auto remainder = magnitude % scale;
  auto result = (negative ? "-" : "") + std::to_string(whole);
  if (remainder == 0U) {
    return result;
  }
  std::string fraction;
  for (auto divisor = scale / 10U; divisor > 0U; divisor /= 10U) {
    fraction.push_back(static_cast<char>('0' + remainder / divisor));
    remainder %= divisor;
  }
  while (!fraction.empty() && fraction.back() == '0') {
    fraction.pop_back();
  }
  return result + "." + fraction;
}

[[nodiscard]] std::string format_nm(Nanometres value) {
  return format_fixed(value.value(), 1'000'000U);
}

[[nodiscard]] std::string format_angle(Microdegrees value) {
  return format_fixed(value.value(), 1'000'000U);
}

[[nodiscard]] std::string quote_string(std::string_view value) {
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

[[nodiscard]] std::string serialize_at(const Footprint& footprint) {
  auto result = "(at " + format_nm(footprint.position.x) + " " + format_nm(footprint.position.y);
  if (footprint.rotation.value() != 0) {
    result += " " + format_angle(footprint.rotation);
  }
  return result + ")";
}

[[nodiscard]] std::optional<std::string> net_ordinal(const NetId& net, const Board& source) {
  const auto found = std::find_if(source.nets.begin(), source.nets.end(),
                                  [&net](const auto& candidate) { return candidate.id == net; });
  if (found == source.nets.end()) {
    return std::nullopt;
  }
  constexpr std::string_view prefix{"kicad-net-"};
  if (!found->id.value().starts_with(prefix)) {
    return std::nullopt;
  }
  const auto ordinal = found->id.value().substr(prefix.size());
  return !ordinal.empty() &&
                 std::all_of(ordinal.begin(), ordinal.end(),
                             [](const auto character) {
                               return std::isdigit(static_cast<unsigned char>(character)) != 0;
                             })
             ? std::optional<std::string>{ordinal}
             : std::nullopt;
}

[[nodiscard]] std::string serialize_track(const TrackSegment& track, std::string_view uuid,
                                          std::string_view ordinal) {
  std::string result = "(segment\n    (start " + format_nm(track.start.x) + " " +
                       format_nm(track.start.y) + ")\n    (end " + format_nm(track.end.x) + " " +
                       format_nm(track.end.y) + ")\n    (width " + format_nm(track.width) +
                       ")\n    (layer " + quote_string(track.layer.value()) + ")\n";
  if (track.locked) {
    result += "    (locked)\n";
  }
  result += "    (net " + std::string(ordinal) + ")\n    (uuid " + quote_string(uuid) + ")\n  )";
  return result;
}

[[nodiscard]] std::string serialize_track_arc(const TrackArc& arc, std::string_view uuid,
                                              std::string_view ordinal) {
  std::string result = "(arc\n    (start " + format_nm(arc.start.x) + " " + format_nm(arc.start.y) +
                       ")\n    (mid " + format_nm(arc.middle.x) + " " + format_nm(arc.middle.y) +
                       ")\n    (end " + format_nm(arc.end.x) + " " + format_nm(arc.end.y) +
                       ")\n    (width " + format_nm(arc.width) + ")\n    (layer " +
                       quote_string(arc.layer.value()) + ")\n";
  if (arc.locked) {
    result += "    (locked)\n";
  }
  result += "    (net " + std::string(ordinal) + ")\n    (uuid " + quote_string(uuid) + ")\n  )";
  return result;
}

[[nodiscard]] std::optional<std::pair<LayerId, LayerId>> via_end_layers(const Via& via,
                                                                        const Board& source) {
  std::vector<std::pair<std::uint32_t, LayerId>> layers;
  for (const auto& layer_id : via.layers) {
    const auto found =
        std::find_if(source.layers.begin(), source.layers.end(), [&layer_id](const auto& layer) {
          return layer.id == layer_id && layer.kind == LayerKind::copper;
        });
    if (found == source.layers.end()) {
      return std::nullopt;
    }
    layers.emplace_back(found->stack_order, found->id);
  }
  if (layers.size() < 2U) {
    return std::nullopt;
  }
  std::sort(layers.begin(), layers.end());
  return std::pair{layers.front().second, layers.back().second};
}

[[nodiscard]] std::optional<std::string> serialize_via(const Via& via, std::string_view uuid,
                                                       std::string_view ordinal,
                                                       const Board& source) {
  const auto ends = via_end_layers(via, source);
  if (!ends) {
    return std::nullopt;
  }
  std::string result = "(via\n    (at " + format_nm(via.position.x) + " " +
                       format_nm(via.position.y) + ")\n    (size " + format_nm(via.diameter) +
                       ")\n    (drill " + format_nm(via.drill) + ")\n    (layers " +
                       quote_string(ends->first.value()) + " " +
                       quote_string(ends->second.value()) + ")\n";
  if (via.locked) {
    result += "    (locked)\n";
  }
  result += "    (net " + std::string(ordinal) + ")\n    (uuid " + quote_string(uuid) + ")\n  )";
  return result;
}

template <typename Item>
[[nodiscard]] std::map<ObjectId, const Item*> by_id(const std::vector<Item>& items) {
  std::map<ObjectId, const Item*> result;
  for (const auto& item : items) {
    result.emplace(item.id, &item);
  }
  return result;
}

[[nodiscard]] bool same_kicad_footprint(const Footprint& lhs, const Footprint& rhs) {
  const auto same_envelope = [](const Footprint& first, const Footprint& second) {
    const auto is_nonserialized = [](PlacementEnvelopeSource source) {
      return source == PlacementEnvelopeSource::external_constraint ||
             source == PlacementEnvelopeSource::tentative_from_geometry ||
             source == PlacementEnvelopeSource::approximated_from_pads;
    };
    const bool first_nonserialized = is_nonserialized(first.placement_envelope_source);
    const bool second_nonserialized = is_nonserialized(second.placement_envelope_source);
    if (first_nonserialized && second_nonserialized) {
      return first.courtyard == second.courtyard &&
             first.courtyard_circles == second.courtyard_circles &&
             first.placement_envelope_source == second.placement_envelope_source;
    }
    if (first_nonserialized || second_nonserialized) {
      const auto& native = first_nonserialized ? second : first;
      return native.placement_envelope_source == PlacementEnvelopeSource::absent &&
             native.courtyard.vertices.empty() && native.courtyard_circles.empty();
    }
    return first.courtyard == second.courtyard &&
           first.courtyard_circles == second.courtyard_circles &&
           first.placement_envelope_source == second.placement_envelope_source;
  };
  return lhs.id == rhs.id && lhs.reference == rhs.reference && lhs.position == rhs.position &&
         lhs.rotation == rhs.rotation && lhs.side == rhs.side && lhs.locked == rhs.locked &&
         same_envelope(lhs, rhs) && lhs.pads == rhs.pads && lhs.keepouts == rhs.keepouts;
}

[[nodiscard]] bool same_kicad_net(const Net& lhs, const Net& rhs) {
  return lhs.id == rhs.id && lhs.name == rhs.name && lhs.priority == rhs.priority;
}

[[nodiscard]] bool same_kicad_nets(const std::vector<Net>& lhs, const std::vector<Net>& rhs) {
  return lhs.size() == rhs.size() &&
         std::equal(lhs.begin(), lhs.end(), rhs.begin(), same_kicad_net);
}

[[nodiscard]] bool same_kicad_differential_pair(const DifferentialPair& lhs,
                                                const DifferentialPair& rhs) {
  return lhs.id == rhs.id && lhs.name == rhs.name && lhs.positive_net == rhs.positive_net &&
         lhs.negative_net == rhs.negative_net;
}

[[nodiscard]] bool same_kicad_differential_pairs(const std::vector<DifferentialPair>& lhs,
                                                 const std::vector<DifferentialPair>& rhs) {
  return lhs.size() == rhs.size() &&
         std::equal(lhs.begin(), lhs.end(), rhs.begin(), same_kicad_differential_pair);
}

[[nodiscard]] bool same_kicad_footprints(const std::vector<Footprint>& lhs,
                                         const std::vector<Footprint>& rhs) {
  return lhs.size() == rhs.size() &&
         std::equal(lhs.begin(), lhs.end(), rhs.begin(), same_kicad_footprint);
}

[[nodiscard]] bool same_board_snapshot(const Board& lhs, const Board& rhs) {
  return lhs.schema_version == rhs.schema_version && lhs.source_revision == rhs.source_revision &&
         lhs.outline == rhs.outline && lhs.cutouts == rhs.cutouts && lhs.layers == rhs.layers &&
         same_kicad_nets(lhs.nets, rhs.nets) &&
         same_kicad_differential_pairs(lhs.differential_pairs, rhs.differential_pairs) &&
         same_kicad_footprints(lhs.footprints, rhs.footprints) && lhs.tracks == rhs.tracks &&
         lhs.track_arcs == rhs.track_arcs && lhs.vias == rhs.vias && lhs.zones == rhs.zones &&
         lhs.keepouts == rhs.keepouts;
}

[[nodiscard]] bool compatible_design(const Board& source, const Board& candidate,
                                     KicadExportResult& result) {
  if (candidate.schema_version != source.schema_version || candidate.outline != source.outline ||
      candidate.cutouts != source.cutouts || candidate.rules != source.rules ||
      candidate.layers != source.layers || candidate.nets != source.nets ||
      candidate.net_classes != source.net_classes ||
      candidate.differential_pairs != source.differential_pairs ||
      candidate.routing_buses != source.routing_buses ||
      candidate.placement_groups != source.placement_groups ||
      candidate.placement_regions != source.placement_regions || candidate.zones != source.zones ||
      candidate.relative_placement_constraints != source.relative_placement_constraints ||
      candidate.keepouts != source.keepouts ||
      candidate.footprints.size() != source.footprints.size()) {
    error(result, "unsupported_design_change",
          "candidate changes board structure, rules, layers, nets, or footprint membership");
    return false;
  }
  const auto candidate_footprints = by_id(candidate.footprints);
  for (const auto& original : source.footprints) {
    const auto found = candidate_footprints.find(original.id);
    if (found == candidate_footprints.end()) {
      error(result, "missing_footprint", "candidate removed a source footprint",
            {original.id.value()});
      return false;
    }
    const auto& placed = *found->second;
    if (placed.reference != original.reference || placed.side != original.side ||
        placed.locked != original.locked || placed.courtyard != original.courtyard ||
        placed.courtyard_circles != original.courtyard_circles ||
        placed.placement_envelope_source != original.placement_envelope_source ||
        placed.orientation_constraints != original.orientation_constraints ||
        placed.pads != original.pads || placed.keepouts != original.keepouts) {
      error(result, "unsupported_footprint_change",
            "only footprint position and rotation changes are exportable", {original.id.value()});
      return false;
    }
    if (original.locked &&
        (placed.position != original.position || placed.rotation != original.rotation)) {
      error(result, "locked_footprint_changed", "candidate moved a locked footprint",
            {original.id.value()});
      return false;
    }
  }
  const auto candidate_tracks = by_id(candidate.tracks);
  for (const auto& original : source.tracks) {
    if (!original.locked) {
      continue;
    }
    const auto found = candidate_tracks.find(original.id);
    if (found == candidate_tracks.end() || *found->second != original) {
      error(result, "locked_track_changed", "candidate removed or modified a locked track",
            {original.id.value()});
      return false;
    }
  }
  const auto candidate_arcs = by_id(candidate.track_arcs);
  for (const auto& original : source.track_arcs) {
    if (!original.locked) {
      continue;
    }
    const auto found = candidate_arcs.find(original.id);
    if (found == candidate_arcs.end() || *found->second != original) {
      error(result, "locked_track_arc_changed", "candidate removed or modified a locked track arc",
            {original.id.value()});
      return false;
    }
  }
  const auto candidate_vias = by_id(candidate.vias);
  for (const auto& original : source.vias) {
    if (!original.locked) {
      continue;
    }
    const auto found = candidate_vias.find(original.id);
    if (found == candidate_vias.end() || *found->second != original) {
      error(result, "locked_via_changed", "candidate removed or modified a locked via",
            {original.id.value()});
      return false;
    }
  }
  return true;
}

[[nodiscard]] std::map<ObjectId, const SExpression*>
source_objects(const SExpression& root, std::string_view object_head, std::string_view fallback) {
  std::map<ObjectId, const SExpression*> result;
  std::size_t index = 0;
  for (const auto* expression : find_children(root, object_head)) {
    result.emplace(
        ObjectId{stable_token(*expression, std::string(fallback) + std::to_string(index++))},
        expression);
  }
  return result;
}

void collect_source_uuids(const SExpression& expression, std::set<std::string>& result) {
  if ((head(expression) == "uuid" || head(expression) == "tstamp") && atom_at(expression, 1)) {
    result.insert(std::string(*atom_at(expression, 1)));
  }
  for (const auto& child : expression.children) {
    if (child.is_list()) {
      collect_source_uuids(child, result);
    }
  }
}

[[nodiscard]] std::string assign_uuid(const ObjectId& id, const Board& source,
                                      std::set<std::string>& used, KicadExportResult& result,
                                      bool reuse_existing = false) {
  auto uuid = id.value();
  if (!is_uuid(uuid)) {
    uuid = deterministic_uuid(source.source_revision + "|" + id.value());
    result.generated_id_map.push_back(ExportedObjectId{id, uuid});
  }
  if (reuse_existing && uuid == id.value() && used.contains(uuid)) {
    return uuid;
  }
  if (!used.insert(uuid).second) {
    error(result, "uuid_collision", "two exported objects resolve to the same KiCad UUID",
          {id.value(), uuid});
  }
  return uuid;
}

[[nodiscard]] bool apply_edits(std::string_view source, std::vector<TextEdit> edits,
                               std::string& output, KicadExportResult& result) {
  std::sort(edits.begin(), edits.end(), [](const auto& lhs, const auto& rhs) {
    return lhs.begin != rhs.begin ? lhs.begin > rhs.begin : lhs.end > rhs.end;
  });
  auto previous_begin = source.size() + 1U;
  output = std::string(source);
  for (const auto& edit : edits) {
    if (edit.begin > edit.end || edit.end > source.size() || edit.end > previous_begin) {
      error(result, "overlapping_text_edits", "candidate produced invalid or overlapping edits");
      return false;
    }
    output.replace(edit.begin, edit.end - edit.begin, edit.replacement);
    previous_begin = edit.begin;
  }
  return true;
}

[[nodiscard]] bool equivalent_after_round_trip(const Board& expected, const Board& actual,
                                               const std::map<ObjectId, ObjectId>& remapped_ids) {
  if (expected.schema_version != actual.schema_version || expected.outline != actual.outline ||
      expected.cutouts != actual.cutouts || expected.layers != actual.layers ||
      !same_kicad_nets(expected.nets, actual.nets) ||
      !same_kicad_differential_pairs(expected.differential_pairs, actual.differential_pairs) ||
      !same_kicad_footprints(expected.footprints, actual.footprints) ||
      expected.zones != actual.zones || expected.keepouts != actual.keepouts) {
    return false;
  }
  auto expected_tracks = expected.tracks;
  auto expected_arcs = expected.track_arcs;
  auto expected_vias = expected.vias;
  for (auto& track : expected_tracks) {
    if (const auto found = remapped_ids.find(track.id); found != remapped_ids.end()) {
      track.id = found->second;
    }
  }
  for (auto& via : expected_vias) {
    if (const auto found = remapped_ids.find(via.id); found != remapped_ids.end()) {
      via.id = found->second;
    }
  }
  for (auto& arc : expected_arcs) {
    if (const auto found = remapped_ids.find(arc.id); found != remapped_ids.end()) {
      arc.id = found->second;
    }
  }
  const auto compare_id = [](const auto& lhs, const auto& rhs) { return lhs.id < rhs.id; };
  std::sort(expected_tracks.begin(), expected_tracks.end(), compare_id);
  std::sort(expected_arcs.begin(), expected_arcs.end(), compare_id);
  std::sort(expected_vias.begin(), expected_vias.end(), compare_id);
  auto actual_tracks = actual.tracks;
  auto actual_arcs = actual.track_arcs;
  auto actual_vias = actual.vias;
  std::sort(actual_tracks.begin(), actual_tracks.end(), compare_id);
  std::sort(actual_arcs.begin(), actual_arcs.end(), compare_id);
  std::sort(actual_vias.begin(), actual_vias.end(), compare_id);
  return expected_tracks == actual_tracks && expected_arcs == actual_arcs &&
         expected_vias == actual_vias;
}

} // namespace

bool KicadExportResult::has_errors() const {
  return std::any_of(diagnostics.begin(), diagnostics.end(), [](const auto& diagnostic) {
    return diagnostic.severity == ExportSeverity::error;
  });
}

bool KicadChangeSetResult::has_errors() const {
  return std::any_of(diagnostics.begin(), diagnostics.end(), [](const auto& diagnostic) {
    return diagnostic.severity == ExportSeverity::error;
  });
}

KicadExportResult export_kicad_candidate(std::string_view source_contents, const Board& source,
                                         const Board& candidate,
                                         const KicadExportOptions& options) {
  KicadExportResult result;
  if (!is_generator_token(options.generator) || options.generator_version.empty()) {
    error(result, "invalid_export_metadata",
          "generator must be a lowercase KiCad token and version must be non-empty");
    return result;
  }
  const auto source_import = import_kicad_pcb(source_contents);
  if (!source_import.strict()) {
    error(result, "source_not_strict", "source board must have a strict, lossless import");
    return result;
  }
  if (source_import.board->source_revision != source.source_revision) {
    error(result, "stale_source_snapshot",
          "source contents do not match the board snapshot used for optimization");
    return result;
  }
  if (!same_board_snapshot(*source_import.board, source)) {
    error(result, "source_snapshot_mismatch",
          "canonical source board differs from a fresh import of its source contents");
    return result;
  }
  if (!validate_board(source).ok()) {
    error(result, "invalid_source_snapshot", "canonical source board fails structural validation");
    return result;
  }
  if (!validate_board(candidate).ok()) {
    error(result, "invalid_candidate", "candidate fails canonical structural validation");
    return result;
  }
  if (!compatible_design(source, candidate, result)) {
    return result;
  }

  const auto parsed = parse_sexpression(source_contents);
  if (!parsed.ok() || !parsed.root || head(*parsed.root) != "kicad_pcb") {
    error(result, "source_parse_failed", "source board could not be parsed for patching");
    return result;
  }
  const auto& root = *parsed.root;
  std::vector<TextEdit> edits;
  if (const auto* generator = find_child(root, "generator")) {
    edits.push_back({generator->location.offset, generator->end_offset,
                     "(generator " + options.generator + ")"});
  } else {
    error(result, "missing_generator", "source board has no generator field");
    return result;
  }
  std::string insertion;
  if (const auto* version = find_child(root, "generator_version")) {
    edits.push_back({version->location.offset, version->end_offset,
                     "(generator_version " + quote_string(options.generator_version) + ")"});
  } else {
    insertion += "\n  (generator_version " + quote_string(options.generator_version) + ")";
  }

  const auto source_footprints = source_objects(root, "footprint", "footprint-");
  const auto candidate_footprints = by_id(candidate.footprints);
  for (const auto& original : source.footprints) {
    const auto expression = source_footprints.find(original.id);
    const auto placed = candidate_footprints.find(original.id);
    if (expression == source_footprints.end() || placed == candidate_footprints.end()) {
      error(result, "footprint_mapping_failed", "footprint cannot be mapped to source syntax",
            {original.id.value()});
      return result;
    }
    if (original.position != placed->second->position ||
        original.rotation != placed->second->rotation) {
      const auto* at = find_child(*expression->second, "at");
      if (at == nullptr) {
        error(result, "missing_footprint_position", "source footprint has no at field",
              {original.id.value()});
        return result;
      }
      edits.push_back({at->location.offset, at->end_offset, serialize_at(*placed->second)});
    }
  }

  const auto source_track_syntax = source_objects(root, "segment", "track-");
  const auto source_arc_syntax = source_objects(root, "arc", "track-arc-");
  const auto source_via_syntax = source_objects(root, "via", "via-");
  const auto source_tracks = by_id(source.tracks);
  const auto source_arcs = by_id(source.track_arcs);
  const auto source_vias = by_id(source.vias);
  const auto candidate_tracks = by_id(candidate.tracks);
  const auto candidate_arcs = by_id(candidate.track_arcs);
  const auto candidate_vias = by_id(candidate.vias);
  std::set<std::string> used_uuids;
  collect_source_uuids(root, used_uuids);

  std::map<ObjectId, ObjectId> remapped_ids;
  for (const auto& [id, original] : source_tracks) {
    const auto syntax = source_track_syntax.find(id);
    if (syntax == source_track_syntax.end()) {
      error(result, "track_mapping_failed", "source track cannot be mapped to source syntax",
            {id.value()});
      return result;
    }
    const auto replacement = candidate_tracks.find(id);
    if (replacement == candidate_tracks.end()) {
      edits.push_back({syntax->second->location.offset, syntax->second->end_offset, {}});
    } else if (*replacement->second != *original) {
      const auto ordinal = net_ordinal(replacement->second->net, source);
      if (!ordinal) {
        error(result, "unknown_route_net", "track net has no KiCad ordinal", {id.value()});
        return result;
      }
      const auto uuid = assign_uuid(id, source, used_uuids, result, true);
      remapped_ids.emplace(id, ObjectId{uuid});
      edits.push_back({syntax->second->location.offset, syntax->second->end_offset,
                       serialize_track(*replacement->second, uuid, *ordinal)});
    }
  }
  for (const auto& [id, original] : source_vias) {
    const auto syntax = source_via_syntax.find(id);
    if (syntax == source_via_syntax.end()) {
      error(result, "via_mapping_failed", "source via cannot be mapped to source syntax",
            {id.value()});
      return result;
    }
    const auto replacement = candidate_vias.find(id);
    if (replacement == candidate_vias.end()) {
      edits.push_back({syntax->second->location.offset, syntax->second->end_offset, {}});
    } else if (*replacement->second != *original) {
      const auto ordinal = net_ordinal(replacement->second->net, source);
      const auto uuid = assign_uuid(id, source, used_uuids, result, true);
      const auto serialized =
          ordinal ? serialize_via(*replacement->second, uuid, *ordinal, source) : std::nullopt;
      if (!serialized) {
        error(result, "invalid_export_via", "via net or layer span cannot be exported",
              {id.value()});
        return result;
      }
      remapped_ids.emplace(id, ObjectId{uuid});
      edits.push_back({syntax->second->location.offset, syntax->second->end_offset, *serialized});
    }
  }
  for (const auto& [id, original] : source_arcs) {
    const auto syntax = source_arc_syntax.find(id);
    if (syntax == source_arc_syntax.end()) {
      error(result, "track_arc_mapping_failed",
            "source track arc cannot be mapped to source syntax", {id.value()});
      return result;
    }
    const auto replacement = candidate_arcs.find(id);
    if (replacement == candidate_arcs.end()) {
      edits.push_back({syntax->second->location.offset, syntax->second->end_offset, {}});
    } else if (*replacement->second != *original) {
      const auto ordinal = net_ordinal(replacement->second->net, source);
      if (!ordinal) {
        error(result, "unknown_route_net", "track arc net has no KiCad ordinal", {id.value()});
        return result;
      }
      const auto uuid = assign_uuid(id, source, used_uuids, result, true);
      remapped_ids.emplace(id, ObjectId{uuid});
      edits.push_back({syntax->second->location.offset, syntax->second->end_offset,
                       serialize_track_arc(*replacement->second, uuid, *ordinal)});
    }
  }

  for (const auto& track : candidate.tracks) {
    if (source_tracks.contains(track.id)) {
      continue;
    }
    const auto ordinal = net_ordinal(track.net, source);
    if (!ordinal) {
      error(result, "unknown_route_net", "new track net has no KiCad ordinal", {track.id.value()});
      return result;
    }
    const auto uuid = assign_uuid(track.id, source, used_uuids, result);
    remapped_ids.emplace(track.id, ObjectId{uuid});
    insertion += "\n  " + serialize_track(track, uuid, *ordinal);
  }
  for (const auto& via : candidate.vias) {
    if (source_vias.contains(via.id)) {
      continue;
    }
    const auto ordinal = net_ordinal(via.net, source);
    const auto uuid = assign_uuid(via.id, source, used_uuids, result);
    const auto serialized = ordinal ? serialize_via(via, uuid, *ordinal, source) : std::nullopt;
    if (!serialized) {
      error(result, "invalid_export_via", "new via net or layer span cannot be exported",
            {via.id.value()});
      return result;
    }
    remapped_ids.emplace(via.id, ObjectId{uuid});
    insertion += "\n  " + *serialized;
  }
  for (const auto& arc : candidate.track_arcs) {
    if (source_arcs.contains(arc.id)) {
      continue;
    }
    const auto ordinal = net_ordinal(arc.net, source);
    if (!ordinal) {
      error(result, "unknown_route_net", "new track arc net has no KiCad ordinal",
            {arc.id.value()});
      return result;
    }
    const auto uuid = assign_uuid(arc.id, source, used_uuids, result);
    remapped_ids.emplace(arc.id, ObjectId{uuid});
    insertion += "\n  " + serialize_track_arc(arc, uuid, *ordinal);
  }
  if (result.has_errors()) {
    return result;
  }
  if (!insertion.empty()) {
    insertion += '\n';
    edits.push_back({root.end_offset - 1U, root.end_offset - 1U, std::move(insertion)});
  }

  std::string output;
  if (!apply_edits(source_contents, std::move(edits), output, result)) {
    return result;
  }
  if (options.verify_round_trip) {
    const auto round_trip = import_kicad_pcb(output);
    if (!round_trip.strict() ||
        !equivalent_after_round_trip(candidate, *round_trip.board, remapped_ids)) {
      error(result, "round_trip_mismatch",
            "exported board does not strictly re-import as the requested candidate");
      return result;
    }
  }
  result.contents = std::move(output);
  return result;
}

KicadExportResult export_kicad_candidate_file(const std::filesystem::path& source_path,
                                              const std::filesystem::path& output_path,
                                              const Board& source, const Board& candidate,
                                              const KicadExportOptions& options) {
  KicadExportResult result;
  std::ifstream input(source_path, std::ios::binary);
  if (!input) {
    error(result, "source_file_open_failed", "unable to open source KiCad board file");
    return result;
  }
  const std::string contents{std::istreambuf_iterator<char>{input},
                             std::istreambuf_iterator<char>{}};
  if (!input.good() && !input.eof()) {
    error(result, "source_file_read_failed", "unable to read source KiCad board file");
    return result;
  }
  if (std::filesystem::exists(output_path)) {
    error(result, "output_exists", "refusing to overwrite an existing output board");
    return result;
  }
  result = export_kicad_candidate(contents, source, candidate, options);
  if (!result.ok()) {
    return result;
  }
  std::ofstream output(output_path, std::ios::binary | std::ios::out | std::ios::trunc);
  if (!output) {
    result.contents.reset();
    error(result, "output_file_open_failed", "unable to create output KiCad board file");
    return result;
  }
  output.write(result.contents->data(), static_cast<std::streamsize>(result.contents->size()));
  output.close();
  if (!output) {
    result.contents.reset();
    error(result, "output_file_write_failed", "unable to write complete output KiCad board file");
    std::error_code cleanup_error;
    std::filesystem::remove(output_path, cleanup_error);
    if (cleanup_error) {
      error(result, "partial_output_cleanup_failed",
            "failed to remove an incomplete generated output file");
    }
  }
  return result;
}

KicadChangeSetResult make_kicad_change_set(std::string_view source_contents,
                                           std::string_view candidate_contents,
                                           const KicadExportOptions& options) {
  KicadChangeSetResult result;
  const auto source_import = import_kicad_pcb(source_contents);
  if (!source_import.strict() || !source_import.board) {
    error(result, "change_set_source_not_strict",
          "change-set source must have a strict, lossless KiCad import");
    return result;
  }
  const auto candidate_import = import_kicad_pcb(candidate_contents);
  if (!candidate_import.strict() || !candidate_import.board) {
    error(result, "change_set_candidate_not_strict",
          "change-set candidate must have a strict, lossless KiCad import");
    return result;
  }

  auto provenance_options = options;
  provenance_options.verify_round_trip = true;
  const auto reproduced = export_kicad_candidate(source_contents, *source_import.board,
                                                 *candidate_import.board, provenance_options);
  if (!reproduced.ok()) {
    result.diagnostics = reproduced.diagnostics;
    error(result, "change_set_candidate_not_exportable",
          "candidate does not satisfy the source-preserving export contract");
    return result;
  }
  if (*reproduced.contents != candidate_contents) {
    error(result, "change_set_candidate_provenance_mismatch",
          "candidate bytes are not the deterministic Flux artifact for this exact source");
    return result;
  }

  const auto& source = *source_import.board;
  const auto& candidate = *candidate_import.board;
  KicadChangeSet change_set;
  change_set.board_schema = source.schema_version;
  change_set.source_revision = source.source_revision;

  const auto source_footprints = by_id(source.footprints);
  const auto candidate_footprints = by_id(candidate.footprints);
  for (const auto& [id, original] : source_footprints) {
    if (!is_uuid(id.value())) {
      error(result, "change_set_invalid_live_id",
            "source footprint has no KiCad UUID for live lookup", {id.value()});
      return result;
    }
    const auto placed = candidate_footprints.find(id);
    if (placed == candidate_footprints.end()) {
      error(result, "change_set_missing_footprint", "candidate removed a source footprint",
            {id.value()});
      return result;
    }
    change_set.source_footprints.push_back(KicadFootprintSnapshot{
        id, original->reference, {original->position, original->rotation, original->side}});
    if (original->position != placed->second->position ||
        original->rotation != placed->second->rotation || original->side != placed->second->side) {
      change_set.footprint_transforms.push_back(KicadFootprintTransformChange{
          id,
          original->reference,
          {original->position, original->rotation, original->side},
          {placed->second->position, placed->second->rotation, placed->second->side}});
    }
  }

  const auto find_net_name =
      [&source, &result](const NetId& id, const ObjectId& object_id) -> std::optional<std::string> {
    const auto found = std::find_if(source.nets.begin(), source.nets.end(), [&id](const Net& net) {
      return net.id == id && !net.name.empty();
    });
    if (found == source.nets.end()) {
      error(result, "change_set_unknown_net", "routing object has no named KiCad net",
            {object_id.value(), id.value()});
      return std::nullopt;
    }
    return found->name;
  };
  const auto track_state =
      [&find_net_name, &result](const TrackSegment& track) -> std::optional<KicadTrackChangeState> {
    if (!is_uuid(track.id.value())) {
      error(result, "change_set_invalid_live_id", "track has no KiCad UUID for live lookup",
            {track.id.value()});
      return std::nullopt;
    }
    const auto name = find_net_name(track.net, track.id);
    if (!name) {
      return std::nullopt;
    }
    return KicadTrackChangeState{track.id,  *name,       track.layer, track.start,
                                 track.end, track.width, track.locked};
  };
  const auto arc_state = [&find_net_name,
                          &result](const TrackArc& arc) -> std::optional<KicadTrackArcChangeState> {
    if (!is_uuid(arc.id.value())) {
      error(result, "change_set_invalid_live_id", "track arc has no KiCad UUID for live lookup",
            {arc.id.value()});
      return std::nullopt;
    }
    const auto name = find_net_name(arc.net, arc.id);
    if (!name) {
      return std::nullopt;
    }
    return KicadTrackArcChangeState{arc.id,     *name,   arc.layer, arc.start,
                                    arc.middle, arc.end, arc.width, arc.locked};
  };
  const auto via_state = [&find_net_name,
                          &result](const Via& via) -> std::optional<KicadViaChangeState> {
    if (!is_uuid(via.id.value())) {
      error(result, "change_set_invalid_live_id", "via has no KiCad UUID for live lookup",
            {via.id.value()});
      return std::nullopt;
    }
    const auto name = find_net_name(via.net, via.id);
    if (!name) {
      return std::nullopt;
    }
    return KicadViaChangeState{via.id,    *name,      via.position, via.diameter,
                               via.drill, via.layers, via.locked};
  };

  const auto append_changes = [&result](const auto& source_items, const auto& candidate_items,
                                        auto convert, auto& output) {
    const auto source_by_id = by_id(source_items);
    const auto candidate_by_id = by_id(candidate_items);
    for (const auto& [id, original] : source_by_id) {
      const auto replacement = candidate_by_id.find(id);
      if (replacement == candidate_by_id.end()) {
        const auto before = convert(*original);
        if (!before) {
          return false;
        }
        output.push_back({KicadChangeOperation::remove, *before, std::nullopt});
      } else if (*replacement->second != *original) {
        const auto before = convert(*original);
        const auto after = convert(*replacement->second);
        if (!before || !after) {
          return false;
        }
        output.push_back({KicadChangeOperation::update, *before, *after});
      }
    }
    for (const auto& [id, added] : candidate_by_id) {
      if (source_by_id.contains(id)) {
        continue;
      }
      const auto after = convert(*added);
      if (!after) {
        return false;
      }
      output.push_back({KicadChangeOperation::create, std::nullopt, *after});
    }
    return !result.has_errors();
  };

  for (const auto& item : by_id(source.tracks)) {
    const auto state = track_state(*item.second);
    if (!state) {
      return result;
    }
    change_set.source_tracks.push_back(*state);
  }
  for (const auto& item : by_id(source.track_arcs)) {
    const auto state = arc_state(*item.second);
    if (!state) {
      return result;
    }
    change_set.source_track_arcs.push_back(*state);
  }
  for (const auto& item : by_id(source.vias)) {
    const auto state = via_state(*item.second);
    if (!state) {
      return result;
    }
    change_set.source_vias.push_back(*state);
  }

  if (!append_changes(source.tracks, candidate.tracks, track_state, change_set.tracks) ||
      !append_changes(source.track_arcs, candidate.track_arcs, arc_state, change_set.track_arcs) ||
      !append_changes(source.vias, candidate.vias, via_state, change_set.vias)) {
    return result;
  }
  result.change_set = std::move(change_set);
  return result;
}

KicadChangeSetResult make_kicad_change_set_file(const std::filesystem::path& source_path,
                                                const std::filesystem::path& candidate_path,
                                                const KicadExportOptions& options) {
  KicadChangeSetResult result;
  const auto read = [&result](const std::filesystem::path& path,
                              std::string_view role) -> std::optional<std::string> {
    std::ifstream input(path, std::ios::binary);
    if (!input) {
      error(result, "change_set_file_open_failed",
            "unable to open " + std::string(role) + " KiCad board file", {path.string()});
      return std::nullopt;
    }
    std::string contents{std::istreambuf_iterator<char>{input}, std::istreambuf_iterator<char>{}};
    if (!input.good() && !input.eof()) {
      error(result, "change_set_file_read_failed",
            "unable to read " + std::string(role) + " KiCad board file", {path.string()});
      return std::nullopt;
    }
    return contents;
  };
  const auto source = read(source_path, "source");
  const auto candidate = read(candidate_path, "candidate");
  if (!source || !candidate) {
    return result;
  }
  return make_kicad_change_set(*source, *candidate, options);
}

} // namespace flux::io
