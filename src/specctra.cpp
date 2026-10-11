#include "flux/io/specctra.hpp"

#include "flux/core/copper_geometry.hpp"
#include "flux/core/keepout_geometry.hpp"
#include "flux/core/routing_rules.hpp"
#include "flux/core/validation.hpp"
#include "flux/io/sexpr.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <charconv>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <map>
#include <set>
#include <sstream>
#include <string>
#include <tuple>
#include <utility>

namespace flux::io {
namespace {

constexpr std::int64_t session_resolution = 10;

template <typename Result>
void diagnostic(Result& result, SpecctraSeverity severity, std::string code, std::string message,
                std::vector<std::string> object_ids = {}, std::size_t line = 1,
                std::size_t column = 1) {
  result.diagnostics.push_back(SpecctraDiagnostic{severity, std::move(code), std::move(message),
                                                  line, column, std::move(object_ids)});
}

[[nodiscard]] std::string quote(std::string_view value) {
  std::string output{"\""};
  for (const char character : value) {
    if (character == '\\' || character == '"') {
      output.push_back('\\');
    }
    output.push_back(character);
  }
  output.push_back('"');
  return output;
}

[[nodiscard]] bool is_unquoted_token(std::string_view value) {
  if (value.empty()) {
    return false;
  }
  return std::ranges::all_of(value, [](const char character) {
    const auto byte = static_cast<unsigned char>(character);
    return std::isalnum(byte) != 0 || character == '_' || character == '-' || character == '.' ||
           character == '+' || character == '/';
  });
}

[[nodiscard]] std::string format_scaled(std::int64_t value, std::int64_t divisor) {
  const bool negative = value < 0;
  const auto magnitude =
      static_cast<std::uint64_t>(negative ? -(value + 1) + std::int64_t{1} : value);
  const auto integral = magnitude / static_cast<std::uint64_t>(divisor);
  auto fractional = magnitude % static_cast<std::uint64_t>(divisor);
  std::string output = (negative ? "-" : "") + std::to_string(integral);
  if (fractional == 0U) {
    return output;
  }
  std::string digits(static_cast<std::size_t>(std::log10(static_cast<double>(divisor))), '0');
  for (auto index = digits.size(); index > 0U; --index) {
    digits[index - 1U] = static_cast<char>('0' + fractional % 10U);
    fractional /= 10U;
  }
  while (!digits.empty() && digits.back() == '0') {
    digits.pop_back();
  }
  return output + "." + digits;
}

[[nodiscard]] std::string um(Nanometres value) { return format_scaled(value.value(), 1000); }
[[nodiscard]] std::string x_um(Point point) { return um(point.x); }
[[nodiscard]] std::string y_um(Point point) { return um(-point.y); }
[[nodiscard]] std::string degrees(Microdegrees value) {
  return format_scaled(value.value(), 1'000'000);
}

[[nodiscard]] std::vector<const Layer*> copper_layers(const Board& board) {
  std::vector<const Layer*> layers;
  for (const auto& layer : board.layers) {
    if (layer.kind == LayerKind::copper) {
      layers.push_back(&layer);
    }
  }
  std::ranges::sort(layers, [](const Layer* lhs, const Layer* rhs) {
    return std::tie(lhs->stack_order, lhs->id) < std::tie(rhs->stack_order, rhs->id);
  });
  return layers;
}

[[nodiscard]] std::map<LayerId, std::size_t>
layer_indices(const std::vector<const Layer*>& layers) {
  std::map<LayerId, std::size_t> result;
  for (std::size_t index = 0; index < layers.size(); ++index) {
    result.emplace(layers[index]->id, index);
  }
  return result;
}

[[nodiscard]] const Net* find_net(const Board& board, const NetId& id) {
  const auto found = std::ranges::find(board.nets, id, &Net::id);
  return found == board.nets.end() ? nullptr : &*found;
}

[[nodiscard]] std::string pin_name(std::size_t index) { return "p" + std::to_string(index); }
[[nodiscard]] std::string image_name(const Footprint& footprint) {
  return "flux-image-" + footprint.id.value();
}
[[nodiscard]] std::string padstack_name(const Pad& pad) { return "flux-pad-" + pad.id.value(); }

struct ViaSpecification {
  Nanometres diameter;
  Nanometres drill;
  std::vector<LayerId> layers;

  auto operator<=>(const ViaSpecification&) const = default;
};

[[nodiscard]] std::string via_name(const ViaSpecification& via,
                                   const std::map<LayerId, std::size_t>& indices) {
  const auto first = indices.at(via.layers.front());
  const auto last = indices.at(via.layers.back());
  return "flux-via-" + std::to_string(first) + "-" + std::to_string(last) + "-" +
         std::to_string(via.diameter.value()) + "-" + std::to_string(via.drill.value());
}

void emit_closed_polygon(std::ostringstream& output, const Polygon& polygon) {
  for (const auto point : polygon.vertices) {
    output << ' ' << x_um(point) << ' ' << y_um(point);
  }
  if (!polygon.vertices.empty()) {
    output << ' ' << x_um(polygon.vertices.front()) << ' ' << y_um(polygon.vertices.front());
  }
}

[[nodiscard]] Polygon rectangular_pad_polygon(const Pad& pad) {
  const Nanometres half_width{pad.width.value() / 2};
  const Nanometres half_height{pad.height.value() / 2};
  return Polygon{{Point{-half_width, -half_height}, Point{half_width, -half_height},
                  Point{half_width, half_height}, Point{-half_width, half_height}}};
}

[[nodiscard]] std::vector<LayerId>
specctra_pad_layers(const Pad& pad, BoardSide side, const std::vector<const Layer*>& copper,
                    const std::map<LayerId, std::size_t>& indices) {
  std::vector<LayerId> layers;
  for (const auto& layer : pad.layers) {
    const auto found = indices.find(layer);
    if (found != indices.end()) {
      const auto local_index =
          side == BoardSide::front ? found->second : copper.size() - 1U - found->second;
      layers.push_back(copper[local_index]->id);
    }
  }
  return layers;
}

[[nodiscard]] bool expression_atom(const SExpression& expression, std::size_t index,
                                   std::string_view& value) {
  const auto* child = child_at(expression, index);
  if (child == nullptr || !child->is_atom()) {
    return false;
  }
  value = child->value;
  return true;
}

[[nodiscard]] std::optional<std::int64_t> parse_session_nm(std::string_view value,
                                                           std::int64_t resolution) {
  if (resolution <= 0) {
    return std::nullopt;
  }
  long double parsed = 0.0L;
  const auto begin = value.data();
  const auto end = value.data() + value.size();
  const auto conversion = std::from_chars(begin, end, parsed);
  if (conversion.ec != std::errc{} || conversion.ptr != end || !std::isfinite(parsed)) {
    return std::nullopt;
  }
  const long double scaled = parsed * 1000.0L / static_cast<long double>(resolution);
  if (scaled > static_cast<long double>(std::numeric_limits<std::int64_t>::max()) ||
      scaled < static_cast<long double>(std::numeric_limits<std::int64_t>::min())) {
    return std::nullopt;
  }
  const auto rounded = static_cast<std::int64_t>(std::llround(scaled));
  if (std::fabs(scaled - static_cast<long double>(rounded)) > 1.0e-9L) {
    return std::nullopt;
  }
  return rounded;
}

[[nodiscard]] std::optional<std::int64_t> parse_integer(std::string_view value) {
  std::int64_t parsed = 0;
  const auto conversion = std::from_chars(value.data(), value.data() + value.size(), parsed);
  if (conversion.ec != std::errc{} || conversion.ptr != value.data() + value.size()) {
    return std::nullopt;
  }
  return parsed;
}

[[nodiscard]] std::optional<std::int64_t> parse_microdegrees(std::string_view value) {
  long double parsed = 0.0L;
  const auto conversion = std::from_chars(value.data(), value.data() + value.size(), parsed);
  if (conversion.ec != std::errc{} || conversion.ptr != value.data() + value.size() ||
      !std::isfinite(parsed)) {
    return std::nullopt;
  }
  const long double scaled = parsed * 1'000'000.0L;
  if (scaled > static_cast<long double>(std::numeric_limits<std::int64_t>::max()) ||
      scaled < static_cast<long double>(std::numeric_limits<std::int64_t>::min())) {
    return std::nullopt;
  }
  const auto rounded = static_cast<std::int64_t>(std::llround(scaled));
  if (std::fabs(scaled - static_cast<long double>(rounded)) > 1.0e-9L) {
    return std::nullopt;
  }
  return rounded;
}

[[nodiscard]] std::optional<std::int64_t> parse_resolution(const SExpression& parent) {
  const auto* expression = find_child(parent, "resolution");
  std::string_view unit;
  std::string_view number;
  if (expression == nullptr || !expression_atom(*expression, 1U, unit) || unit != "um" ||
      !expression_atom(*expression, 2U, number)) {
    return std::nullopt;
  }
  return parse_integer(number);
}

[[nodiscard]] SExpressionParseResult parse_specctra_dsn(std::string_view contents) {
  // Specctra's parser directive spells the quote character as `(string_quote ")`. That is a
  // grammar declaration rather than an ordinary balanced quoted atom, so normalize only this
  // declaration for the generic bounded S-expression reader. Provenance still compares original
  // bytes, not the normalized parsing copy.
  std::string normalized{contents};
  constexpr std::string_view directive{"(string_quote \")"};
  constexpr std::string_view parseable{"(string_quote \"\\\"\")"};
  std::size_t position = 0U;
  while ((position = normalized.find(directive, position)) != std::string::npos) {
    normalized.replace(position, directive.size(), parseable);
    position += parseable.size();
  }
  return parse_sexpression(normalized);
}

[[nodiscard]] std::string deterministic_uuid(std::string_view seed) {
  const auto hash = [seed](std::uint64_t basis) {
    constexpr std::uint64_t prime = 1099511628211ULL;
    auto value = basis;
    for (const auto character : seed) {
      value ^= static_cast<unsigned char>(character);
      value *= prime;
    }
    return value;
  };
  const auto high = hash(14695981039346656037ULL);
  auto low = hash(1099511628211ULL);
  std::array<unsigned char, 16> bytes{};
  for (std::size_t index = 0; index < 8U; ++index) {
    bytes[index] = static_cast<unsigned char>((high >> ((7U - index) * 8U)) & 0xffU);
    bytes[8U + index] = static_cast<unsigned char>((low >> ((7U - index) * 8U)) & 0xffU);
  }
  bytes[6] = static_cast<unsigned char>((bytes[6] & 0x0fU) | 0x40U);
  bytes[8] = static_cast<unsigned char>((bytes[8] & 0x3fU) | 0x80U);
  constexpr char hex[] = "0123456789abcdef";
  std::string result;
  result.reserve(36U);
  for (std::size_t index = 0; index < bytes.size(); ++index) {
    if (index == 4U || index == 6U || index == 8U || index == 10U) {
      result.push_back('-');
    }
    result.push_back(hex[bytes[index] >> 4U]);
    result.push_back(hex[bytes[index] & 0x0fU]);
  }
  return result;
}

struct RouteTrack {
  std::string net;
  LayerId layer;
  Point start;
  Point end;
  Nanometres width;
  bool protect{false};

  auto operator<=>(const RouteTrack&) const = default;
};

struct RouteVia {
  std::string net;
  Point position;
  ViaSpecification specification;
  bool protect{false};

  auto operator<=>(const RouteVia&) const = default;
};

[[nodiscard]] RouteTrack normalized(RouteTrack value) {
  if (value.end < value.start) {
    std::swap(value.start, value.end);
  }
  return value;
}

[[nodiscard]] RouteTrack source_track(const Board& board, const TrackSegment& track) {
  const auto* net = find_net(board, track.net);
  return normalized(RouteTrack{net == nullptr ? "" : net->name, track.layer, track.start, track.end,
                               track.width, true});
}

[[nodiscard]] RouteVia source_via(const Board& board, const Via& via) {
  const auto* net = find_net(board, via.net);
  return RouteVia{net == nullptr ? "" : net->name, via.position,
                  ViaSpecification{via.diameter, via.drill, via.layers}, true};
}

template <typename T> [[nodiscard]] std::multiset<T> protected_only(const std::vector<T>& values) {
  std::multiset<T> result;
  for (const auto& value : values) {
    if (value.protect) {
      result.insert(value);
    }
  }
  return result;
}

} // namespace

bool SpecctraDsnExportResult::has_errors() const {
  return std::ranges::any_of(
      diagnostics, [](const auto& item) { return item.severity == SpecctraSeverity::error; });
}

bool SpecctraSesImportResult::has_errors() const {
  return std::ranges::any_of(
      diagnostics, [](const auto& item) { return item.severity == SpecctraSeverity::error; });
}

SpecctraDsnExportResult export_specctra_dsn(const Board& board,
                                            const SpecctraDsnExportOptions& options) {
  SpecctraDsnExportResult result;
  const auto validation = validate_board(board);
  if (!validation.ok()) {
    diagnostic(result, SpecctraSeverity::error, "invalid_board",
               "Specctra export requires a structurally valid canonical board");
  }
  if (options.design_name.empty()) {
    diagnostic(result, SpecctraSeverity::error, "empty_design_name",
               "Specctra design name must not be empty");
  }
  const auto layers = copper_layers(board);
  result.exported_layers = layers.size();
  if (layers.size() < 2U) {
    diagnostic(result, SpecctraSeverity::error, "insufficient_copper_layers",
               "Specctra routing requires at least two ordered copper layers");
  }
  if (!board.cutouts.empty()) {
    diagnostic(result, SpecctraSeverity::error, "unsupported_board_cutouts",
               "Specctra cutout boundary semantics are not enabled in this baseline");
  }
  if (!board.track_beziers.empty()) {
    diagnostic(result, SpecctraSeverity::error, "unsupported_track_beziers",
               "Specctra export cannot represent authoritative cubic copper");
  }
  if (!board.track_arcs.empty()) {
    diagnostic(result, SpecctraSeverity::error, "unsupported_track_arcs",
               "Specctra export does not flatten native track arcs");
  }
  std::set<std::string> references;
  for (const auto& footprint : board.footprints) {
    if (footprint.reference.empty() || !references.insert(footprint.reference).second ||
        !is_unquoted_token(footprint.reference)) {
      diagnostic(result, SpecctraSeverity::error, "invalid_footprint_reference",
                 "footprint references must be unique non-empty Specctra tokens",
                 {footprint.id.value()});
    }
  }
  for (const auto& zone : board.zones) {
    for (const auto& fill : zone.fills) {
      if (!fill.holes.empty()) {
        diagnostic(result, SpecctraSeverity::error, "unsupported_zone_holes",
                   "Specctra export does not flatten filled-zone holes", {zone.id.value()});
      }
    }
  }
  const auto keepouts = world_keepouts(board);
  for (const auto& keepout : keepouts) {
    if (keepout.prohibit.pads || keepout.prohibit.copper_pours || keepout.prohibit.footprints) {
      diagnostic(result, SpecctraSeverity::error, "unsupported_keepout_policy",
                 "only track and via keepout semantics are representable", {keepout.id.value()});
    }
  }
  for (const auto& footprint : board.footprints) {
    for (const auto& pad : footprint.pads) {
      if (pad.shape == PadShape::trapezoid || pad.shape == PadShape::custom ||
          pad.shape == PadShape::unknown) {
        diagnostic(result, SpecctraSeverity::error, "unsupported_pad_shape",
                   "pad shape cannot be represented conservatively in Specctra",
                   {footprint.id.value(), pad.id.value()});
      }
    }
  }
  if (result.has_errors()) {
    return result;
  }

  const auto indices = layer_indices(layers);
  std::set<ViaSpecification> via_specs;
  for (const auto& net : board.nets) {
    if (!net.name.empty()) {
      const auto rules = resolve_net_routing_rules(board, net.id);
      std::vector<LayerId> span;
      for (const auto* layer : layers) {
        span.push_back(layer->id);
      }
      via_specs.insert(ViaSpecification{rules.via_diameter, rules.via_drill, std::move(span)});
    }
  }
  for (const auto& via : board.vias) {
    if (via.layers.size() < 2U || !indices.contains(via.layers.front()) ||
        !indices.contains(via.layers.back()) || via.diameter.value() <= 0 ||
        via.drill.value() <= 0 || via.drill >= via.diameter) {
      diagnostic(result, SpecctraSeverity::error, "unsupported_via",
                 "via span and dimensions must map exactly to ordered copper layers",
                 {via.id.value()});
      continue;
    }
    const auto first = indices.at(via.layers.front());
    const auto last = indices.at(via.layers.back());
    if (first >= last || via.layers.size() != last - first + 1U) {
      diagnostic(result, SpecctraSeverity::error, "unsupported_via_span",
                 "via layers must be one contiguous ordered copper span", {via.id.value()});
      continue;
    }
    for (std::size_t index = first; index <= last; ++index) {
      if (via.layers[index - first] != layers[index]->id) {
        diagnostic(result, SpecctraSeverity::error, "unsupported_via_span",
                   "via layers must be one contiguous ordered copper span", {via.id.value()});
        break;
      }
    }
    via_specs.insert(ViaSpecification{via.diameter, via.drill, via.layers});
  }
  if (result.has_errors()) {
    return result;
  }

  std::ostringstream output;
  output << "(pcb " << quote(options.design_name) << '\n'
         << "  (parser\n"
         << "    (string_quote \")\n"
         << "    (space_in_quoted_tokens on)\n"
         << "    (host_cad \"flux_capacitor\")\n"
         << "    (host_version \"0.101.0\")\n"
         << "  )\n"
         << "  (resolution um " << session_resolution << ")\n"
         << "  (unit um)\n"
         << "  (structure\n";
  for (std::size_t index = 0; index < layers.size(); ++index) {
    output << "    (layer " << quote(layers[index]->name) << " (type signal) (property (index "
           << index << ")))\n";
  }
  output << "    (boundary (path pcb 0";
  emit_closed_polygon(output, board.outline);
  output << "))\n";
  for (const auto& zone : board.zones) {
    const auto* net = find_net(board, zone.net);
    if (net == nullptr || net->name.empty()) {
      continue;
    }
    for (const auto& fill : zone.fills) {
      const auto layer =
          std::ranges::find(layers, fill.layer, [](const Layer* item) { return item->id; });
      if (layer == layers.end()) {
        continue;
      }
      output << "    (plane " << quote(net->name) << " (polygon " << quote((*layer)->name) << " 0";
      emit_closed_polygon(output, fill.area);
      output << "))\n";
    }
  }
  for (const auto& keepout : keepouts) {
    for (const auto& layer_id : keepout.layers) {
      const auto found = indices.find(layer_id);
      if (found == indices.end()) {
        continue;
      }
      const auto layer_name = quote(layers[found->second]->name);
      if (keepout.prohibit.tracks) {
        output << "    (keepout \"\" (polygon " << layer_name << " 0";
        emit_closed_polygon(output, keepout.area);
        output << "))\n";
      }
      if (keepout.prohibit.vias) {
        output << "    (via_keepout \"\" (polygon " << layer_name << " 0";
        emit_closed_polygon(output, keepout.area);
        output << "))\n";
      }
    }
  }
  if (!via_specs.empty()) {
    output << "    (via";
    for (const auto& via : via_specs) {
      output << ' ' << quote(via_name(via, indices));
    }
    output << ")\n";
  }
  Nanometres maximum_clearance = board.rules.default_clearance;
  for (const auto& net : board.nets) {
    if (!net.name.empty()) {
      maximum_clearance =
          std::max(maximum_clearance, resolve_net_routing_rules(board, net.id).clearance);
    }
  }
  output << "    (rule (width " << um(board.rules.default_track_width) << ") (clearance "
         << um(maximum_clearance) << "))\n"
         << "  )\n"
         << "  (placement\n";
  for (const auto& footprint : board.footprints) {
    output << "    (component " << quote(image_name(footprint)) << '\n'
           << "      (place " << footprint.reference << ' ' << x_um(footprint.position) << ' '
           << y_um(footprint.position) << ' '
           << (footprint.side == BoardSide::front ? "front" : "back") << ' '
           << degrees(footprint.rotation) << ")\n"
           << "    )\n";
  }
  output << "  )\n"
         << "  (library\n";
  for (const auto& footprint : board.footprints) {
    output << "    (image " << quote(image_name(footprint)) << '\n';
    std::vector<const Pad*> pads;
    for (const auto& pad : footprint.pads) {
      if (!specctra_pad_layers(pad, footprint.side, layers, indices).empty()) {
        pads.push_back(&pad);
      }
    }
    std::ranges::sort(pads, [](const Pad* lhs, const Pad* rhs) { return lhs->id < rhs->id; });
    for (std::size_t index = 0; index < pads.size(); ++index) {
      output << "      (pin " << quote(padstack_name(*pads[index])) << ' ' << pin_name(index) << ' '
             << x_um(pads[index]->local_position) << ' ' << y_um(pads[index]->local_position);
      if (pads[index]->rotation.value() != 0) {
        output << " (rotate " << degrees(pads[index]->rotation) << ')';
      }
      output << ")\n";
      ++result.exported_pins;
    }
    output << "    )\n";
  }
  for (const auto& footprint : board.footprints) {
    for (const auto& pad : footprint.pads) {
      const auto pad_layers = specctra_pad_layers(pad, footprint.side, layers, indices);
      if (pad_layers.empty()) {
        continue;
      }
      output << "    (padstack " << quote(padstack_name(pad)) << '\n';
      for (const auto& layer_id : pad_layers) {
        const auto layer_name = quote(layers[indices.at(layer_id)]->name);
        if (pad.shape == PadShape::circle) {
          output << "      (shape (path " << layer_name << ' ' << um(pad.width) << " 0 0 0 0))\n";
        } else if (pad.shape == PadShape::rectangle) {
          const Nanometres half_width{pad.width.value() / 2};
          const Nanometres half_height{pad.height.value() / 2};
          output << "      (shape (rect " << layer_name << ' ' << um(-half_width) << ' '
                 << um(-half_height) << ' ' << um(half_width) << ' ' << um(half_height) << "))\n";
        } else {
          const auto polygon = rectangular_pad_polygon(pad);
          output << "      (shape (polygon " << layer_name << " 0";
          emit_closed_polygon(output, polygon);
          output << "))\n";
          if (pad.shape == PadShape::oval || pad.shape == PadShape::round_rectangle) {
            diagnostic(result, SpecctraSeverity::warning, "conservative_pad_envelope",
                       "oval or round-rectangle pad exported as a conservative rectangle",
                       {footprint.id.value(), pad.id.value()});
          }
        }
      }
      output << "      (attach off)\n"
             << "    )\n";
    }
  }
  for (const auto& via : via_specs) {
    output << "    (padstack " << quote(via_name(via, indices)) << '\n';
    for (const auto& layer_id : via.layers) {
      output << "      (shape (circle " << quote(layers[indices.at(layer_id)]->name) << ' '
             << um(via.diameter) << "))\n";
    }
    output << "      (attach off)\n"
           << "    )\n";
  }
  output << "  )\n"
         << "  (network\n";
  for (const auto& net : board.nets) {
    if (net.name.empty()) {
      continue;
    }
    output << "    (net " << quote(net.name) << "\n      (pins";
    for (const auto& footprint : board.footprints) {
      std::vector<const Pad*> pads;
      for (const auto& pad : footprint.pads) {
        if (!specctra_pad_layers(pad, footprint.side, layers, indices).empty()) {
          pads.push_back(&pad);
        }
      }
      std::ranges::sort(pads, [](const Pad* lhs, const Pad* rhs) { return lhs->id < rhs->id; });
      for (std::size_t index = 0; index < pads.size(); ++index) {
        if (pads[index]->net == net.id) {
          output << ' ' << footprint.reference << '-' << pin_name(index);
        }
      }
    }
    output << ")\n    )\n";
  }
  for (const auto& net : board.nets) {
    if (net.name.empty()) {
      continue;
    }
    const auto rules = resolve_net_routing_rules(board, net.id);
    std::vector<LayerId> full_span;
    for (const auto* layer : layers) {
      full_span.push_back(layer->id);
    }
    const ViaSpecification via{rules.via_diameter, rules.via_drill, std::move(full_span)};
    output << "    (class " << quote("flux-class-" + net.id.value()) << ' ' << quote(net.name)
           << "\n      (circuit (use_via " << quote(via_name(via, indices)) << "))\n"
           << "      (rule (width " << um(rules.track_width) << ") (clearance "
           << um(rules.clearance) << "))\n"
           << "    )\n";
  }
  output << "  )\n"
         << "  (wiring\n";
  for (const auto& track : board.tracks) {
    const auto* net = find_net(board, track.net);
    const auto found = indices.find(track.layer);
    if (net == nullptr || net->name.empty() || found == indices.end()) {
      continue;
    }
    output << "    (wire (path " << quote(layers[found->second]->name) << ' ' << um(track.width)
           << ' ' << x_um(track.start) << ' ' << y_um(track.start) << ' ' << x_um(track.end) << ' '
           << y_um(track.end) << ") (net " << quote(net->name) << ") (type protect))\n";
    ++result.protected_tracks;
  }
  for (const auto& via : board.vias) {
    const auto* net = find_net(board, via.net);
    if (net == nullptr || net->name.empty()) {
      continue;
    }
    const ViaSpecification specification{via.diameter, via.drill, via.layers};
    output << "    (via " << quote(via_name(specification, indices)) << ' ' << x_um(via.position)
           << ' ' << y_um(via.position) << " (net " << quote(net->name) << ") (type protect))\n";
    ++result.protected_vias;
  }
  output << "  )\n"
         << ")\n";
  result.contents = output.str();
  return result;
}

SpecctraDsnExportResult export_specctra_dsn_file(const Board& board,
                                                 const std::filesystem::path& output_path) {
  SpecctraDsnExportOptions options;
  options.design_name = output_path.filename().string();
  auto result = export_specctra_dsn(board, options);
  if (!result.ok()) {
    return result;
  }
  std::error_code error_code;
  if (std::filesystem::exists(output_path, error_code) || error_code) {
    diagnostic(result, SpecctraSeverity::error, "output_exists",
               "refusing to overwrite existing Specctra design artifact");
    result.contents.reset();
    return result;
  }
  std::ofstream output(output_path, std::ios::binary | std::ios::out | std::ios::trunc);
  if (!output) {
    diagnostic(result, SpecctraSeverity::error, "output_file_open_failed",
               "unable to open Specctra output artifact");
    result.contents.reset();
    return result;
  }
  output.write(result.contents->data(), static_cast<std::streamsize>(result.contents->size()));
  output.close();
  if (!output) {
    diagnostic(result, SpecctraSeverity::error, "output_file_write_failed",
               "unable to write complete Specctra output artifact");
    result.contents.reset();
    std::error_code cleanup_error;
    std::filesystem::remove(output_path, cleanup_error);
    if (cleanup_error) {
      diagnostic(result, SpecctraSeverity::error, "partial_output_cleanup_failed",
                 "unable to remove incomplete Specctra output artifact");
    }
  }
  return result;
}

SpecctraSesImportResult import_specctra_session(const Board& source, std::string_view dsn_contents,
                                                std::string_view ses_contents) {
  SpecctraSesImportResult result;
  const auto parsed_dsn = parse_specctra_dsn(dsn_contents);
  if (!parsed_dsn.ok() || head(*parsed_dsn.root) != "pcb") {
    diagnostic(result, SpecctraSeverity::error, "invalid_dsn", "input is not one valid DSN pcb");
    return result;
  }
  std::string_view design_name;
  if (!expression_atom(*parsed_dsn.root, 1U, design_name)) {
    diagnostic(result, SpecctraSeverity::error, "missing_design_name",
               "DSN pcb scope has no design name");
    return result;
  }
  const auto regenerated =
      export_specctra_dsn(source, SpecctraDsnExportOptions{std::string{design_name}});
  if (!regenerated.ok()) {
    result.diagnostics = regenerated.diagnostics;
    return result;
  }
  if (*regenerated.contents != dsn_contents) {
    diagnostic(result, SpecctraSeverity::error, "dsn_source_mismatch",
               "DSN bytes are not the deterministic artifact for this source board");
    return result;
  }

  const auto parsed_ses = parse_sexpression(ses_contents);
  if (!parsed_ses.ok() || head(*parsed_ses.root) != "session") {
    diagnostic(result, SpecctraSeverity::error, "invalid_session",
               "input is not one valid Specctra session");
    return result;
  }
  const auto* base_design = find_child(*parsed_ses.root, "base_design");
  std::string_view base_name;
  const auto design_stem = std::filesystem::path{design_name}.stem().string();
  if (base_design == nullptr || !expression_atom(*base_design, 1U, base_name) ||
      (base_name != design_name && base_name != design_stem)) {
    diagnostic(result, SpecctraSeverity::error, "base_design_mismatch",
               "session base_design does not name the bound DSN artifact", {},
               base_design == nullptr ? 1U : base_design->location.line,
               base_design == nullptr ? 1U : base_design->location.column);
    return result;
  }
  if (find_children(*parsed_ses.root, "base_design").size() != 1U ||
      find_children(*parsed_ses.root, "placement").size() != 1U ||
      find_children(*parsed_ses.root, "routes").size() != 1U) {
    diagnostic(result, SpecctraSeverity::error, "duplicate_session_scope",
               "session requires exactly one base_design, placement, and routes scope");
    return result;
  }
  const auto* placement = find_child(*parsed_ses.root, "placement");
  const auto* routes = find_child(*parsed_ses.root, "routes");
  if (placement == nullptr || routes == nullptr) {
    diagnostic(result, SpecctraSeverity::error, "missing_session_scope",
               "session must contain placement and routes scopes");
    return result;
  }
  const auto placement_resolution = parse_resolution(*placement);
  const auto route_resolution = parse_resolution(*routes);
  if (find_children(*placement, "resolution").size() != 1U ||
      find_children(*routes, "resolution").size() != 1U ||
      placement_resolution != session_resolution || route_resolution != session_resolution) {
    diagnostic(result, SpecctraSeverity::error, "unsupported_session_resolution",
               "placement and routes must use resolution (um 10)");
    return result;
  }

  std::map<std::string, const Footprint*> source_footprints;
  for (const auto& footprint : source.footprints) {
    source_footprints.emplace(footprint.reference, &footprint);
  }
  std::set<std::string> seen_footprints;
  for (const auto* component : find_children(*placement, "component")) {
    for (const auto* place : find_children(*component, "place")) {
      std::string_view reference;
      std::string_view x;
      std::string_view y;
      std::string_view side;
      std::string_view angle;
      if (!expression_atom(*place, 1U, reference) || !expression_atom(*place, 2U, x) ||
          !expression_atom(*place, 3U, y) || !expression_atom(*place, 4U, side) ||
          !expression_atom(*place, 5U, angle)) {
        diagnostic(result, SpecctraSeverity::error, "invalid_session_placement",
                   "place statement is incomplete", {}, place->location.line,
                   place->location.column);
        continue;
      }
      const auto found = source_footprints.find(std::string{reference});
      const auto x_nm = parse_session_nm(x, *placement_resolution);
      const auto y_nm = parse_session_nm(y, *placement_resolution);
      const auto angle_microdegrees = parse_microdegrees(angle);
      if (found == source_footprints.end() ||
          !seen_footprints.insert(std::string{reference}).second || !x_nm || !y_nm ||
          !angle_microdegrees ||
          found->second->position != Point{Nanometres{*x_nm}, Nanometres{-*y_nm}} ||
          found->second->rotation != Microdegrees{*angle_microdegrees} ||
          side != (found->second->side == BoardSide::front ? "front" : "back")) {
        diagnostic(result, SpecctraSeverity::error, "placement_drift",
                   "session placement must exactly match every source footprint", {},
                   place->location.line, place->location.column);
      }
    }
  }
  if (seen_footprints.size() != source_footprints.size()) {
    diagnostic(result, SpecctraSeverity::error, "incomplete_session_placement",
               "session placement does not cover every source footprint exactly once");
  }
  if (result.has_errors()) {
    return result;
  }

  const auto layers = copper_layers(source);
  const auto indices = layer_indices(layers);
  std::map<std::string, LayerId> known_layers;
  for (const auto* layer : layers) {
    known_layers.emplace(layer->name, layer->id);
  }
  std::map<std::string, NetId> known_nets;
  for (const auto& net : source.nets) {
    if (!net.name.empty()) {
      known_nets.emplace(net.name, net.id);
    }
  }
  std::map<std::string, ViaSpecification> known_vias;
  for (const auto& net : source.nets) {
    if (net.name.empty()) {
      continue;
    }
    const auto rules = resolve_net_routing_rules(source, net.id);
    std::vector<LayerId> full_span;
    for (const auto* layer : layers) {
      full_span.push_back(layer->id);
    }
    ViaSpecification via{rules.via_diameter, rules.via_drill, std::move(full_span)};
    known_vias.emplace(via_name(via, indices), via);
  }
  for (const auto& via : source.vias) {
    ViaSpecification specification{via.diameter, via.drill, via.layers};
    known_vias.emplace(via_name(specification, indices), std::move(specification));
  }

  std::vector<RouteTrack> parsed_tracks;
  std::vector<RouteVia> parsed_vias;
  const auto* network_out = find_child(*routes, "network_out");
  if (network_out == nullptr || find_children(*routes, "network_out").size() != 1U) {
    diagnostic(result, SpecctraSeverity::error, "missing_network_out",
               "session routes has no network_out scope");
    return result;
  }
  for (const auto* net_expression : find_children(*network_out, "net")) {
    std::string_view net_name;
    if (!expression_atom(*net_expression, 1U, net_name) ||
        !known_nets.contains(std::string{net_name})) {
      diagnostic(result, SpecctraSeverity::error, "unknown_session_net",
                 "session route names a net absent from the source", {},
                 net_expression->location.line, net_expression->location.column);
      continue;
    }
    for (std::size_t child_index = 2U; child_index < net_expression->children.size();
         ++child_index) {
      const auto& route = net_expression->children[child_index];
      if (!route.is_list()) {
        continue;
      }
      if (head(route) == "wire") {
        const auto* path = find_child(route, "path");
        std::string_view layer_name;
        std::string_view width_text;
        if (path == nullptr || find_children(route, "path").size() != 1U ||
            find_children(route, "type").size() > 1U || !expression_atom(*path, 1U, layer_name) ||
            !expression_atom(*path, 2U, width_text) ||
            !known_layers.contains(std::string{layer_name})) {
          diagnostic(result, SpecctraSeverity::error, "invalid_session_wire",
                     "wire path has an unknown layer or incomplete header", {}, route.location.line,
                     route.location.column);
          continue;
        }
        const auto width = parse_session_nm(width_text, *route_resolution);
        if (!width || *width <= 0 || path->children.size() < 7U ||
            (path->children.size() - 3U) % 2U != 0U) {
          diagnostic(result, SpecctraSeverity::error, "invalid_session_path",
                     "wire path requires positive width and at least two exact points", {},
                     path->location.line, path->location.column);
          continue;
        }
        bool protect = false;
        if (const auto* type = find_child(route, "type"); type != nullptr) {
          std::string_view type_name;
          if (!expression_atom(*type, 1U, type_name) ||
              (type_name != "protect" && type_name != "route")) {
            diagnostic(result, SpecctraSeverity::error, "unsupported_wire_type",
                       "wire type must be route or protect", {}, type->location.line,
                       type->location.column);
            continue;
          }
          protect = type_name == "protect";
        }
        std::vector<Point> points;
        for (std::size_t index = 3U; index < path->children.size(); index += 2U) {
          std::string_view x;
          std::string_view y;
          if (!expression_atom(*path, index, x) || !expression_atom(*path, index + 1U, y)) {
            points.clear();
            break;
          }
          const auto x_nm = parse_session_nm(x, *route_resolution);
          const auto y_nm = parse_session_nm(y, *route_resolution);
          if (!x_nm || !y_nm || *x_nm > max_abs_coordinate_nm || *x_nm < -max_abs_coordinate_nm ||
              *y_nm > max_abs_coordinate_nm || *y_nm < -max_abs_coordinate_nm) {
            points.clear();
            break;
          }
          points.push_back(Point{Nanometres{*x_nm}, Nanometres{-*y_nm}});
        }
        if (points.size() < 2U) {
          diagnostic(result, SpecctraSeverity::error, "invalid_session_coordinate",
                     "wire path coordinate is not an exact in-range nanometre value", {},
                     path->location.line, path->location.column);
          continue;
        }
        for (std::size_t index = 1U; index < points.size(); ++index) {
          if (points[index - 1U] == points[index]) {
            diagnostic(result, SpecctraSeverity::error, "zero_length_session_track",
                       "wire path contains a zero-length segment", {}, path->location.line,
                       path->location.column);
            continue;
          }
          parsed_tracks.push_back(normalized(
              RouteTrack{std::string{net_name}, known_layers.at(std::string{layer_name}),
                         points[index - 1U], points[index], Nanometres{*width}, protect}));
        }
      } else if (head(route) == "via") {
        std::string_view padstack;
        std::string_view x;
        std::string_view y;
        if (find_children(route, "type").size() > 1U || !expression_atom(route, 1U, padstack) ||
            !expression_atom(route, 2U, x) || !expression_atom(route, 3U, y) ||
            !known_vias.contains(std::string{padstack})) {
          diagnostic(result, SpecctraSeverity::error, "unknown_session_via",
                     "via uses an unknown padstack or incomplete coordinate", {},
                     route.location.line, route.location.column);
          continue;
        }
        const auto x_nm = parse_session_nm(x, *route_resolution);
        const auto y_nm = parse_session_nm(y, *route_resolution);
        if (!x_nm || !y_nm) {
          diagnostic(result, SpecctraSeverity::error, "invalid_session_coordinate",
                     "via coordinate is not an exact nanometre value", {}, route.location.line,
                     route.location.column);
          continue;
        }
        bool protect = false;
        if (const auto* type = find_child(route, "type"); type != nullptr) {
          std::string_view type_name;
          if (!expression_atom(*type, 1U, type_name) ||
              (type_name != "protect" && type_name != "route")) {
            diagnostic(result, SpecctraSeverity::error, "unsupported_via_type",
                       "via type must be route or protect", {}, type->location.line,
                       type->location.column);
            continue;
          }
          protect = type_name == "protect";
        }
        parsed_vias.push_back(RouteVia{std::string{net_name},
                                       Point{Nanometres{*x_nm}, Nanometres{-*y_nm}},
                                       known_vias.at(std::string{padstack}), protect});
      } else {
        diagnostic(result, SpecctraSeverity::error, "unsupported_session_route",
                   "network_out contains a route primitive other than wire or via", {},
                   route.location.line, route.location.column);
      }
    }
  }
  if (result.has_errors()) {
    return result;
  }

  std::multiset<RouteTrack> expected_tracks;
  for (const auto& track : source.tracks) {
    expected_tracks.insert(source_track(source, track));
  }
  std::multiset<RouteVia> expected_vias;
  for (const auto& via : source.vias) {
    expected_vias.insert(source_via(source, via));
  }
  if (protected_only(parsed_tracks) != expected_tracks) {
    diagnostic(result, SpecctraSeverity::error, "protected_tracks_mismatch",
               "session protected wires are not the exact source-track multiset");
  }
  if (protected_only(parsed_vias) != expected_vias) {
    diagnostic(result, SpecctraSeverity::error, "protected_vias_mismatch",
               "session protected vias are not the exact source-via multiset");
  }
  if (result.has_errors()) {
    return result;
  }
  result.verified_protected_tracks = expected_tracks.size();
  result.verified_protected_vias = expected_vias.size();

  std::vector<RouteTrack> new_tracks;
  for (const auto& track : parsed_tracks) {
    if (!track.protect) {
      new_tracks.push_back(track);
    }
  }
  std::vector<RouteVia> new_vias;
  for (const auto& via : parsed_vias) {
    if (!via.protect) {
      new_vias.push_back(via);
    }
  }
  std::ranges::sort(new_tracks);
  std::ranges::sort(new_vias);

  Board candidate = source;
  std::set<std::string> object_ids;
  for (const auto& footprint : source.footprints) {
    object_ids.insert(footprint.id.value());
    for (const auto& pad : footprint.pads) {
      object_ids.insert(pad.id.value());
    }
  }
  for (const auto& track : source.tracks) {
    object_ids.insert(track.id.value());
  }
  for (const auto& via : source.vias) {
    object_ids.insert(via.id.value());
  }
  for (std::size_t index = 0; index < new_tracks.size(); ++index) {
    const auto& route = new_tracks[index];
    std::ostringstream seed;
    seed << source.source_revision << "|ses-track|" << route.net << '|' << route.layer.value()
         << '|' << route.start.x.value() << '|' << route.start.y.value() << '|'
         << route.end.x.value() << '|' << route.end.y.value() << '|' << route.width.value() << '|'
         << index;
    const auto id = deterministic_uuid(seed.str());
    if (!object_ids.insert(id).second) {
      diagnostic(result, SpecctraSeverity::error, "generated_id_collision",
                 "deterministic session track ID collides with a source object");
      return result;
    }
    candidate.tracks.push_back(TrackSegment{ObjectId{id}, known_nets.at(route.net), route.layer,
                                            route.start, route.end, route.width, false});
  }
  for (std::size_t index = 0; index < new_vias.size(); ++index) {
    const auto& route = new_vias[index];
    std::ostringstream seed;
    seed << source.source_revision << "|ses-via|" << route.net << '|' << route.position.x.value()
         << '|' << route.position.y.value() << '|' << route.specification.diameter.value() << '|'
         << route.specification.drill.value() << '|' << index;
    const auto id = deterministic_uuid(seed.str());
    if (!object_ids.insert(id).second) {
      diagnostic(result, SpecctraSeverity::error, "generated_id_collision",
                 "deterministic session via ID collides with a source object");
      return result;
    }
    candidate.vias.push_back(Via{ObjectId{id}, known_nets.at(route.net), route.position,
                                 route.specification.diameter, route.specification.drill,
                                 route.specification.layers, false});
  }
  result.imported_tracks = new_tracks.size();
  result.imported_vias = new_vias.size();
  result.evaluation = evaluate_board(candidate);
  if (!result.evaluation->validation.ok()) {
    diagnostic(result, SpecctraSeverity::error, "candidate_validation_failed",
               "session candidate failed canonical validation");
  }
  if (result.evaluation->connectivity.open_connections != 0U) {
    diagnostic(result, SpecctraSeverity::error, "candidate_connectivity_failed",
               "session candidate retains open electrical connections");
  }
  if (!result.evaluation->copper_drc.ok()) {
    diagnostic(result, SpecctraSeverity::error, "candidate_copper_drc_failed",
               "session candidate violates represented copper DRC");
  }
  if (result.has_errors()) {
    return result;
  }
  result.candidate = std::move(candidate);
  return result;
}

} // namespace flux::io
