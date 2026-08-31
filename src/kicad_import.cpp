#include "flux/io/kicad_import.hpp"

#include "flux/core/geometry.hpp"
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

struct RawSegment {
  Point start;
  Point end;
  SourceLocation location;
};

struct RawPolygonEdge {
  Point start;
  Point end;
};

enum class CurveProxyPolicy { lossy_tessellation, conservative_courtyard };

struct Importer {
  explicit Importer(KicadImportOptions import_options) : options(std::move(import_options)) {}

  KicadImportOptions options;
  KicadImportResult result;
  std::map<std::string, NetId> net_by_ordinal;
  std::map<std::string, std::size_t> copper_layer_index;

  void diagnostic(ImportSeverity severity, std::string code, std::string message,
                  SourceLocation location = {}) {
    result.diagnostics.push_back(ImportDiagnostic{severity, std::move(code), std::move(message),
                                                  location.line, location.column});
  }

  void warning(std::string code, std::string message, SourceLocation location = {},
               bool makes_lossy = true) {
    diagnostic(ImportSeverity::warning, std::move(code), std::move(message), location);
    result.lossy = result.lossy || makes_lossy;
  }

  void error(std::string code, std::string message, SourceLocation location = {}) {
    diagnostic(ImportSeverity::error, std::move(code), std::move(message), location);
  }
};

[[nodiscard]] const SExpression* value_at(const SExpression& expression, std::size_t index) {
  const auto* value = child_at(expression, index);
  return value != nullptr && value->is_atom() ? value : nullptr;
}

[[nodiscard]] std::optional<std::string_view> atom_at(const SExpression& expression,
                                                      std::size_t index) {
  const auto* value = value_at(expression, index);
  return value == nullptr ? std::nullopt : std::optional<std::string_view>{value->value};
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

[[nodiscard]] std::string content_fingerprint(std::string_view contents) {
  constexpr std::uint64_t offset_basis = 14695981039346656037ULL;
  constexpr std::uint64_t prime = 1099511628211ULL;
  std::uint64_t value = offset_basis;
  for (const auto character : contents) {
    value ^= static_cast<unsigned char>(character);
    value *= prime;
  }
  constexpr char digits[] = "0123456789abcdef";
  std::string result(16, '0');
  for (std::size_t i = 0; i < result.size(); ++i) {
    const auto shift = static_cast<unsigned int>((result.size() - i - 1U) * 4U);
    result[i] = digits[(value >> shift) & 0xFU];
  }
  return result;
}

[[nodiscard]] std::optional<Point> parse_point(const SExpression& expression, Importer& importer,
                                               std::string_view context) {
  const auto x_atom = atom_at(expression, 1);
  const auto y_atom = atom_at(expression, 2);
  const auto x = x_atom ? parse_double(*x_atom) : std::nullopt;
  const auto y = y_atom ? parse_double(*y_atom) : std::nullopt;
  if (!x || !y) {
    importer.error("invalid_coordinate", std::string(context) + " requires two finite numbers",
                   expression.location);
    return std::nullopt;
  }
  try {
    return Point{Nanometres::from_mm(*x), Nanometres::from_mm(*y)};
  } catch (const std::out_of_range&) {
    importer.error("coordinate_out_of_range", std::string(context) + " is out of range",
                   expression.location);
    return std::nullopt;
  }
}

[[nodiscard]] std::optional<Point> named_point(const SExpression& parent, std::string_view name,
                                               Importer& importer) {
  const auto* expression = find_child(parent, name);
  return expression == nullptr ? std::nullopt : parse_point(*expression, importer, name);
}

[[nodiscard]] std::optional<double> named_number(const SExpression& parent, std::string_view name,
                                                 Importer& importer) {
  const auto* expression = find_child(parent, name);
  if (expression == nullptr) {
    return std::nullopt;
  }
  const auto atom = atom_at(*expression, 1);
  const auto value = atom ? parse_double(*atom) : std::nullopt;
  if (!value) {
    importer.error("invalid_number", std::string(name) + " requires a finite number",
                   expression->location);
  }
  return value;
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

[[nodiscard]] bool has_atom(const SExpression& expression, std::string_view value) {
  return expression.is_list() && std::any_of(expression.children.begin(), expression.children.end(),
                                             [value](const auto& child) {
                                               return child.is_atom() && child.value == value;
                                             });
}

[[nodiscard]] std::optional<std::string_view>
named_atom(const SExpression& parent, std::string_view name, std::size_t value_index = 1) {
  const auto* child = find_child(parent, name);
  return child == nullptr ? std::nullopt : atom_at(*child, value_index);
}

[[nodiscard]] std::vector<RawSegment> polygon_segments(const std::vector<Point>& vertices,
                                                       SourceLocation location) {
  std::vector<RawSegment> result;
  if (vertices.size() < 2U) {
    return result;
  }
  result.reserve(vertices.size());
  for (std::size_t i = 0; i < vertices.size(); ++i) {
    result.push_back(RawSegment{vertices[i], vertices[(i + 1U) % vertices.size()], location});
  }
  return result;
}

[[nodiscard]] std::vector<Point> parse_points(const SExpression& expression, Importer& importer) {
  const auto* points = find_child(expression, "pts");
  std::vector<Point> result;
  if (points == nullptr) {
    importer.error("missing_points", "polygon is missing its pts list", expression.location);
    return result;
  }
  for (const auto* coordinate : find_children(*points, "xy")) {
    if (const auto point = parse_point(*coordinate, importer, "xy")) {
      result.push_back(*point);
    }
  }
  if (result.size() < 3U) {
    importer.error("insufficient_points", "polygon requires at least three points",
                   points->location);
  }
  return result;
}

[[nodiscard]] long double normalized_angle(long double value) {
  constexpr long double two_pi = 6.283185307179586476925286766559005768L;
  value = std::fmod(value, two_pi);
  return value < 0.0L ? value + two_pi : value;
}

[[nodiscard]] long double ccw_delta(long double start, long double end) {
  return normalized_angle(end - start);
}

[[nodiscard]] std::vector<Point> tessellate_arc(Point start, Point middle, Point end,
                                                Nanometres tolerance, Importer& importer,
                                                SourceLocation location) {
  const auto x1 = static_cast<long double>(start.x.value());
  const auto y1 = static_cast<long double>(start.y.value());
  const auto x2 = static_cast<long double>(middle.x.value());
  const auto y2 = static_cast<long double>(middle.y.value());
  const auto x3 = static_cast<long double>(end.x.value());
  const auto y3 = static_cast<long double>(end.y.value());
  const auto denominator = 2.0L * (x1 * (y2 - y3) + x2 * (y3 - y1) + x3 * (y1 - y2));
  if (std::abs(denominator) < 1.0L) {
    importer.error("degenerate_arc", "arc start, mid, and end are collinear", location);
    return {};
  }

  const auto first_norm = x1 * x1 + y1 * y1;
  const auto second_norm = x2 * x2 + y2 * y2;
  const auto third_norm = x3 * x3 + y3 * y3;
  const auto center_x =
      (first_norm * (y2 - y3) + second_norm * (y3 - y1) + third_norm * (y1 - y2)) / denominator;
  const auto center_y =
      (first_norm * (x3 - x2) + second_norm * (x1 - x3) + third_norm * (x2 - x1)) / denominator;
  const auto radius = std::hypot(x1 - center_x, y1 - center_y);
  const auto start_angle = std::atan2(y1 - center_y, x1 - center_x);
  const auto middle_angle = std::atan2(y2 - center_y, x2 - center_x);
  const auto end_angle = std::atan2(y3 - center_y, x3 - center_x);
  const auto ccw_sweep = ccw_delta(start_angle, end_angle);
  const bool middle_on_ccw = ccw_delta(start_angle, middle_angle) <= ccw_sweep;
  constexpr long double two_pi = 6.283185307179586476925286766559005768L;
  const auto sweep = middle_on_ccw ? ccw_sweep : ccw_sweep - two_pi;
  const auto arc_length = radius * std::abs(sweep);
  const auto chord = std::max<std::int64_t>(1, tolerance.value());
  auto segment_count =
      static_cast<std::size_t>(std::ceil(arc_length / static_cast<long double>(chord)));
  segment_count = std::clamp<std::size_t>(segment_count, 2U, 4096U);
  if (segment_count == 4096U && arc_length > static_cast<long double>(chord) * 4096.0L) {
    importer.warning("arc_tessellation_capped",
                     "arc tessellation reached the 4096-segment safety limit", location);
  }

  std::vector<Point> points;
  points.reserve(segment_count + 1U);
  points.push_back(start);
  for (std::size_t i = 1; i < segment_count; ++i) {
    const auto fraction = static_cast<long double>(i) / static_cast<long double>(segment_count);
    const auto angle = start_angle + sweep * fraction;
    points.push_back(Point{
        Nanometres{static_cast<std::int64_t>(std::llround(center_x + radius * std::cos(angle)))},
        Nanometres{static_cast<std::int64_t>(std::llround(center_y + radius * std::sin(angle)))}});
  }
  points.push_back(end);
  return points;
}

void append_shape_segments(const SExpression& shape, std::vector<RawSegment>& segments,
                           Importer& importer,
                           CurveProxyPolicy curve_policy = CurveProxyPolicy::lossy_tessellation) {
  const auto shape_head = head(shape);
  if (shape_head == "gr_line" || shape_head == "fp_line") {
    const auto start = named_point(shape, "start", importer);
    const auto end = named_point(shape, "end", importer);
    if (start && end) {
      segments.push_back(RawSegment{*start, *end, shape.location});
    }
    return;
  }
  if (shape_head == "gr_rect" || shape_head == "fp_rect") {
    const auto start = named_point(shape, "start", importer);
    const auto end = named_point(shape, "end", importer);
    if (start && end) {
      const std::vector<Point> vertices{
          {start->x, start->y}, {end->x, start->y}, {end->x, end->y}, {start->x, end->y}};
      const auto rectangle = polygon_segments(vertices, shape.location);
      segments.insert(segments.end(), rectangle.begin(), rectangle.end());
    }
    return;
  }
  if (shape_head == "gr_poly" || shape_head == "fp_poly") {
    const auto vertices = parse_points(shape, importer);
    const auto polygon = polygon_segments(vertices, shape.location);
    segments.insert(segments.end(), polygon.begin(), polygon.end());
    return;
  }
  if (shape_head == "gr_arc" || shape_head == "fp_arc") {
    const auto start = named_point(shape, "start", importer);
    const auto middle = named_point(shape, "mid", importer);
    const auto end = named_point(shape, "end", importer);
    if (start && middle && end) {
      const auto points = tessellate_arc(
          *start, *middle, *end, importer.options.arc_chord_tolerance, importer, shape.location);
      for (std::size_t i = 1; i < points.size(); ++i) {
        segments.push_back(RawSegment{points[i - 1U], points[i], shape.location});
      }
      importer.warning("arc_tessellated", "arc was tessellated into straight segments",
                       shape.location);
    }
    return;
  }
  if (shape_head == "gr_circle" || shape_head == "fp_circle") {
    const auto center = named_point(shape, "center", importer);
    const auto end = named_point(shape, "end", importer);
    if (!center || !end) {
      return;
    }
    const auto dx = static_cast<long double>(end->x.value() - center->x.value());
    const auto dy = static_cast<long double>(end->y.value() - center->y.value());
    const auto radius = std::hypot(dx, dy);
    constexpr long double two_pi = 6.283185307179586476925286766559005768L;
    const auto chord = std::max<std::int64_t>(1, importer.options.arc_chord_tolerance.value());
    auto segment_count =
        static_cast<std::size_t>(std::ceil(two_pi * radius / static_cast<long double>(chord)));
    segment_count = std::clamp<std::size_t>(segment_count, 16U, 4096U);
    const auto proxy_radius = curve_policy == CurveProxyPolicy::conservative_courtyard
                                  ? radius / std::cos(3.141592653589793238462643383279502884L /
                                                      static_cast<long double>(segment_count)) +
                                        2.0L
                                  : radius;
    std::vector<Point> points;
    points.reserve(segment_count);
    for (std::size_t i = 0; i < segment_count; ++i) {
      const auto angle =
          two_pi * static_cast<long double>(i) / static_cast<long double>(segment_count);
      points.push_back(Point{
          Nanometres{static_cast<std::int64_t>(std::llround(
              static_cast<long double>(center->x.value()) + proxy_radius * std::cos(angle)))},
          Nanometres{static_cast<std::int64_t>(std::llround(
              static_cast<long double>(center->y.value()) + proxy_radius * std::sin(angle)))}});
    }
    const auto circle = polygon_segments(points, shape.location);
    segments.insert(segments.end(), circle.begin(), circle.end());
    if (curve_policy == CurveProxyPolicy::conservative_courtyard) {
      importer.warning("circle_conservative_courtyard_proxy",
                       "exact courtyard circle uses a conservative polygonal search proxy",
                       shape.location, false);
    } else {
      importer.warning("circle_tessellated", "circle was tessellated into straight segments",
                       shape.location);
    }
  }
}

[[nodiscard]] std::vector<Polygon> assemble_loops(std::vector<RawSegment> segments,
                                                  Importer& importer, std::string_view context) {
  std::vector<Polygon> loops;
  std::vector<bool> used(segments.size(), false);
  for (std::size_t seed = 0; seed < segments.size(); ++seed) {
    if (used[seed]) {
      continue;
    }
    used[seed] = true;
    std::vector<Point> vertices{segments[seed].start};
    auto cursor = segments[seed].end;
    while (cursor != vertices.front()) {
      const auto found = std::find_if(
          segments.begin(), segments.end(), [&used, &segments, cursor](const auto& candidate) {
            const auto index = static_cast<std::size_t>(&candidate - segments.data());
            return !used[index] && (candidate.start == cursor || candidate.end == cursor);
          });
      if (found == segments.end()) {
        importer.error("open_geometry", std::string(context) + " contains an open contour",
                       segments[seed].location);
        vertices.clear();
        break;
      }
      const auto index = static_cast<std::size_t>(found - segments.begin());
      used[index] = true;
      vertices.push_back(cursor);
      cursor = found->start == cursor ? found->end : found->start;
      if (vertices.size() > segments.size() + 1U) {
        importer.error("geometry_cycle_error", std::string(context) + " cannot be assembled",
                       segments[seed].location);
        vertices.clear();
        break;
      }
    }
    if (vertices.size() >= 3U) {
      Polygon polygon{std::move(vertices)};
      if (!geometry::is_simple_polygon(polygon)) {
        importer.error("invalid_polygon", std::string(context) + " is not a simple polygon",
                       segments[seed].location);
      } else {
        loops.push_back(std::move(polygon));
      }
    }
  }
  return loops;
}

[[nodiscard]] bool shape_on_layer(const SExpression& shape, std::string_view layer_name) {
  const auto layer = named_atom(shape, "layer");
  return layer && *layer == layer_name;
}

void import_layers(const SExpression& root, Board& board, Importer& importer) {
  const auto* layers = find_child(root, "layers");
  if (layers == nullptr) {
    importer.error("missing_layers", "KiCad board has no layers section", root.location);
    return;
  }
  std::uint32_t copper_order = 0;
  for (std::size_t i = 1; i < layers->children.size(); ++i) {
    const auto& definition = layers->children[i];
    const auto name = atom_at(definition, 1);
    const auto type = atom_at(definition, 2);
    if (!name || !type) {
      importer.error("invalid_layer", "layer definition is incomplete", definition.location);
      continue;
    }
    const bool copper = (*type == "signal" || *type == "power") && name->ends_with(".Cu");
    const auto order = copper ? copper_order++ : 0U;
    board.layers.push_back(Layer{LayerId{std::string(*name)}, std::string(*name),
                                 copper ? LayerKind::copper : LayerKind::technical, order});
    if (copper) {
      importer.copper_layer_index.emplace(std::string(*name), order);
    }
  }
  if (importer.copper_layer_index.empty()) {
    importer.error("missing_copper_layers", "KiCad board has no copper layers", layers->location);
  }
}

void import_nets(const SExpression& root, Board& board, Importer& importer) {
  for (const auto* definition : find_children(root, "net")) {
    const auto ordinal = atom_at(*definition, 1);
    const auto name = atom_at(*definition, 2);
    if (!ordinal || !name) {
      importer.error("invalid_net", "net definition is incomplete", definition->location);
      continue;
    }
    const NetId id{"kicad-net-" + std::string(*ordinal)};
    importer.net_by_ordinal.emplace(std::string(*ordinal), id);
    board.nets.push_back(Net{id, std::string(*name), 0});
  }
  if (!importer.net_by_ordinal.contains("0")) {
    const NetId no_net{"kicad-net-0"};
    importer.net_by_ordinal.emplace("0", no_net);
    board.nets.insert(board.nets.begin(), Net{no_net, "", 0});
  }
}

struct DifferentialSuffixMatch {
  int polarity{0};
  std::string complement;
  std::string pair_name;
};

// Mirrors BOARD::MatchDpSuffix: trailing digits/underscores may follow P/N or +/-.
[[nodiscard]] DifferentialSuffixMatch match_differential_suffix(std::string_view name) {
  DifferentialSuffixMatch result;
  std::size_t count = 0;
  char complement = '\0';
  for (auto iterator = name.rbegin(); iterator != name.rend() && result.polarity == 0;
       ++iterator, ++count) {
    const auto character = *iterator;
    if ((character >= '0' && character <= '9') || character == '_') {
      continue;
    }
    if (character == '+') {
      result.polarity = 1;
      complement = '-';
    } else if (character == '-') {
      result.polarity = -1;
      complement = '+';
    } else if (character == 'P') {
      result.polarity = 1;
      complement = 'N';
    } else if (character == 'N') {
      result.polarity = -1;
      complement = 'P';
    }
  }
  if (result.polarity == 0 || count == 0U || count > name.size()) {
    return {};
  }
  const auto prefix = name.substr(0, name.size() - count);
  const auto suffix = name.substr(name.size() - (count - 1U));
  result.complement = std::string(prefix) + complement + std::string(suffix);
  result.pair_name = std::string(prefix) + "*" + std::string(suffix);
  return result;
}

void infer_differential_pairs(Board& board) {
  std::map<std::string, const Net*> by_name;
  for (const auto& net : board.nets) {
    if (!net.name.empty()) {
      by_name.emplace(net.name, &net);
    }
  }
  for (const auto& net : board.nets) {
    const auto match = match_differential_suffix(net.name);
    if (match.polarity != 1) {
      continue;
    }
    const auto complement = by_name.find(match.complement);
    if (complement == by_name.end()) {
      continue;
    }
    board.differential_pairs.push_back(DifferentialPair{
        DifferentialPairId{"kicad-diffpair:" + net.id.value() + ":" +
                           complement->second->id.value()},
        match.pair_name, net.id, complement->second->id, std::nullopt, std::nullopt});
  }
}

void import_outline(const SExpression& root, Board& board, Importer& importer) {
  std::vector<RawSegment> segments;
  for (const auto& child : root.children) {
    if ((head(child) == "gr_line" || head(child) == "gr_rect" || head(child) == "gr_poly" ||
         head(child) == "gr_arc" || head(child) == "gr_circle") &&
        shape_on_layer(child, "Edge.Cuts")) {
      append_shape_segments(child, segments, importer);
    } else if (head(child).starts_with("gr_") && shape_on_layer(child, "Edge.Cuts")) {
      importer.error("unsupported_outline_shape",
                     "unsupported graphic primitive appears on Edge.Cuts", child.location);
    }
  }
  for (const auto* footprint : find_children(root, "footprint")) {
    std::vector<RawSegment> local_segments;
    for (const auto& child : footprint->children) {
      if ((head(child) == "fp_line" || head(child) == "fp_rect" || head(child) == "fp_poly" ||
           head(child) == "fp_arc" || head(child) == "fp_circle") &&
          shape_on_layer(child, "Edge.Cuts")) {
        append_shape_segments(child, local_segments, importer);
      } else if (head(child).starts_with("fp_") && shape_on_layer(child, "Edge.Cuts")) {
        importer.error("unsupported_outline_shape",
                       "unsupported footprint graphic primitive appears on Edge.Cuts",
                       child.location);
      }
    }
    if (local_segments.empty()) {
      continue;
    }
    const auto* at = find_child(*footprint, "at");
    const auto position =
        at ? parse_point(*at, importer, "footprint Edge.Cuts transform") : std::nullopt;
    if (!position) {
      importer.error("missing_footprint_position",
                     "footprint-owned Edge.Cuts geometry requires an at position",
                     footprint->location);
      continue;
    }
    Microdegrees rotation;
    if (const auto angle = atom_at(*at, 3)) {
      if (const auto degrees = parse_double(*angle)) {
        rotation = Microdegrees::from_degrees(*degrees);
      } else {
        importer.error("invalid_rotation", "footprint Edge.Cuts transform rotation is not numeric",
                       at->location);
        continue;
      }
    }
    const auto layer = named_atom(*footprint, "layer");
    const auto side = layer && *layer == "B.Cu" ? BoardSide::back : BoardSide::front;
    for (const auto& segment : local_segments) {
      segments.push_back(RawSegment{
          geometry::transform_point(segment.start, *position, rotation, side),
          geometry::transform_point(segment.end, *position, rotation, side), segment.location});
    }
  }
  if (segments.empty()) {
    importer.error("missing_outline", "no supported geometry was found on Edge.Cuts",
                   root.location);
    return;
  }
  auto loops = assemble_loops(std::move(segments), importer, "board outline");
  if (loops.empty()) {
    return;
  }
  std::sort(loops.begin(), loops.end(), [](const auto& lhs, const auto& rhs) {
    const auto lhs_area = std::abs(geometry::signed_double_area(lhs));
    const auto rhs_area = std::abs(geometry::signed_double_area(rhs));
    return lhs_area != rhs_area ? lhs_area > rhs_area : lhs.vertices < rhs.vertices;
  });
  board.outline = std::move(loops.front());
  for (std::size_t index = 1U; index < loops.size(); ++index) {
    auto& cutout = loops[index];
    if (!geometry::polygon_inside(cutout, board.outline) ||
        geometry::polygon_boundary_distance_nm(cutout, board.outline) <= 0.0L) {
      importer.error("multiple_board_outlines_unsupported",
                     "every secondary Edge.Cuts contour must be a cutout strictly inside the "
                     "single represented outer board",
                     root.location);
      continue;
    }
    if (std::any_of(board.cutouts.begin(), board.cutouts.end(), [&cutout](const Polygon& prior) {
          return geometry::polygons_overlap(cutout, prior);
        })) {
      importer.error("overlapping_board_cutouts",
                     "interior Edge.Cuts contours must be pairwise disjoint", root.location);
      continue;
    }
    board.cutouts.push_back(std::move(cutout));
  }
}

[[nodiscard]] std::vector<LayerId> expand_layers(const SExpression& expression, const Board& board,
                                                 Importer& importer) {
  std::vector<LayerId> result;
  const auto* layers = find_child(expression, "layers");
  if (layers == nullptr) {
    return result;
  }
  for (std::size_t i = 1; i < layers->children.size(); ++i) {
    const auto layer = atom_at(*layers, i);
    if (!layer) {
      continue;
    }
    if (*layer == "*.Cu") {
      for (const auto& board_layer : board.layers) {
        if (board_layer.kind == LayerKind::copper) {
          result.push_back(board_layer.id);
        }
      }
    } else if (*layer == "F&B.Cu") {
      for (const auto& board_layer : board.layers) {
        if (board_layer.id == LayerId{"F.Cu"} || board_layer.id == LayerId{"B.Cu"}) {
          result.push_back(board_layer.id);
        }
      }
    } else if (importer.copper_layer_index.contains(std::string(*layer))) {
      result.emplace_back(std::string(*layer));
    }
  }
  std::sort(result.begin(), result.end());
  result.erase(std::unique(result.begin(), result.end()), result.end());
  return result;
}

[[nodiscard]] std::vector<LayerId> keepout_layers(const SExpression& zone, const Board& board,
                                                  Importer& importer) {
  auto result = expand_layers(zone, board, importer);
  if (const auto* layers = find_child(zone, "layers")) {
    for (std::size_t index = 1; index < layers->children.size(); ++index) {
      const auto layer = atom_at(*layers, index);
      if (layer && *layer != "*.Cu" && *layer != "F&B.Cu" &&
          !importer.copper_layer_index.contains(std::string(*layer))) {
        importer.error("unknown_keepout_layer", "keepout references an unknown copper layer",
                       layers->location);
      }
    }
  }
  if (const auto layer = named_atom(zone, "layer")) {
    if (*layer == "*.Cu") {
      for (const auto& board_layer : board.layers) {
        if (board_layer.kind == LayerKind::copper) {
          result.push_back(board_layer.id);
        }
      }
    } else if (*layer == "F&B.Cu") {
      for (const auto& board_layer : board.layers) {
        if (board_layer.id == LayerId{"F.Cu"} || board_layer.id == LayerId{"B.Cu"}) {
          result.push_back(board_layer.id);
        }
      }
    } else if (importer.copper_layer_index.contains(std::string(*layer))) {
      result.emplace_back(std::string(*layer));
    } else {
      importer.error("unknown_keepout_layer", "keepout references an unknown copper layer",
                     zone.location);
    }
  }
  std::sort(result.begin(), result.end());
  result.erase(std::unique(result.begin(), result.end()), result.end());
  if (result.empty()) {
    importer.error("keepout_without_layers", "keepout has no supported copper layers",
                   zone.location);
  }
  return result;
}

[[nodiscard]] std::optional<Keepout> import_keepout_zone(const SExpression& zone,
                                                         const Board& board, Importer& importer,
                                                         std::string fallback_id) {
  const auto* settings = find_child(zone, "keepout");
  if (settings == nullptr) {
    return std::nullopt;
  }

  KeepoutRules rules;
  bool valid = true;
  const auto parse_rule = [&](std::string_view name, bool& prohibited) {
    const auto value = named_atom(*settings, name);
    if (!value || (*value != "allowed" && *value != "not_allowed")) {
      importer.error("invalid_keepout_settings",
                     "keepout settings must specify allowed or not_allowed for every category",
                     settings->location);
      valid = false;
      return;
    }
    prohibited = *value == "not_allowed";
  };
  parse_rule("tracks", rules.tracks);
  parse_rule("vias", rules.vias);
  parse_rule("pads", rules.pads);
  parse_rule("copperpour", rules.copper_pours);
  parse_rule("footprints", rules.footprints);
  if (!rules.any()) {
    importer.error("empty_keepout_rules", "keepout does not prohibit any object category",
                   settings->location);
    valid = false;
  }

  const auto polygons = find_children(zone, "polygon");
  if (polygons.size() != 1U) {
    importer.error("invalid_keepout_polygon_count",
                   "keepout must contain exactly one polygon outline", zone.location);
    return std::nullopt;
  }
  Polygon area{parse_points(*polygons.front(), importer)};
  if (!geometry::is_simple_polygon(area)) {
    importer.error("invalid_keepout_polygon", "keepout outline must be a simple polygon",
                   polygons.front()->location);
    valid = false;
  }
  auto layers = keepout_layers(zone, board, importer);
  if (layers.empty()) {
    valid = false;
  }
  if (!valid) {
    return std::nullopt;
  }
  return Keepout{ObjectId{stable_token(zone, std::move(fallback_id))}, std::move(area),
                 std::move(layers), rules, std::string(named_atom(zone, "name").value_or(""))};
}

[[nodiscard]] NetId item_net(const SExpression& item, Importer& importer);

[[nodiscard]] std::optional<CopperZoneFill> decode_filled_region(LayerId layer, Polygon fractured) {
  if (fractured.vertices.size() > 1U && fractured.vertices.front() == fractured.vertices.back()) {
    fractured.vertices.pop_back();
  }
  if (geometry::is_simple_polygon(fractured)) {
    return CopperZoneFill{std::move(layer), std::move(fractured), {}};
  }
  if (fractured.vertices.size() < 3U) {
    return std::nullopt;
  }

  std::vector<RawPolygonEdge> edges;
  edges.reserve(fractured.vertices.size());
  std::map<std::pair<Point, Point>, std::vector<std::size_t>> occurrences;
  for (std::size_t index = 0; index < fractured.vertices.size(); ++index) {
    const auto start = fractured.vertices[index];
    const auto end = fractured.vertices[(index + 1U) % fractured.vertices.size()];
    if (start == end || !geometry::coordinate_in_range(start) ||
        !geometry::coordinate_in_range(end)) {
      return std::nullopt;
    }
    edges.push_back(RawPolygonEdge{start, end});
    occurrences[std::minmax(start, end)].push_back(index);
  }

  std::vector<bool> bridge_edge(edges.size(), false);
  bool removed_bridge = false;
  for (const auto& [unused_key, indices] : occurrences) {
    (void)unused_key;
    if (indices.size() == 1U) {
      continue;
    }
    if (indices.size() != 2U) {
      return std::nullopt;
    }
    const auto& first = edges[indices[0]];
    const auto& second = edges[indices[1]];
    if (first.start != second.end || first.end != second.start) {
      return std::nullopt;
    }
    bridge_edge[indices[0]] = true;
    bridge_edge[indices[1]] = true;
    removed_bridge = true;
  }
  if (!removed_bridge) {
    return std::nullopt;
  }

  std::map<Point, std::vector<std::size_t>> incident_edges;
  for (std::size_t index = 0; index < edges.size(); ++index) {
    if (bridge_edge[index]) {
      continue;
    }
    incident_edges[edges[index].start].push_back(index);
    incident_edges[edges[index].end].push_back(index);
  }
  if (incident_edges.empty() ||
      std::any_of(incident_edges.begin(), incident_edges.end(),
                  [](const auto& entry) { return entry.second.size() != 2U; })) {
    return std::nullopt;
  }

  std::vector<bool> visited(edges.size(), false);
  std::vector<Polygon> contours;
  for (std::size_t first_edge = 0; first_edge < edges.size(); ++first_edge) {
    if (bridge_edge[first_edge] || visited[first_edge]) {
      continue;
    }
    Polygon contour;
    const auto first_point = edges[first_edge].start;
    auto current_point = first_point;
    auto current_edge = first_edge;
    for (std::size_t step = 0; step <= edges.size(); ++step) {
      if (visited[current_edge]) {
        return std::nullopt;
      }
      visited[current_edge] = true;
      contour.vertices.push_back(current_point);
      const auto& edge = edges[current_edge];
      if (edge.start == current_point) {
        current_point = edge.end;
      } else if (edge.end == current_point) {
        current_point = edge.start;
      } else {
        return std::nullopt;
      }
      if (current_point == first_point) {
        break;
      }
      const auto incident = incident_edges.find(current_point);
      if (incident == incident_edges.end()) {
        return std::nullopt;
      }
      const auto next = std::find_if(incident->second.begin(), incident->second.end(),
                                     [&visited](std::size_t index) { return !visited[index]; });
      if (next == incident->second.end()) {
        return std::nullopt;
      }
      current_edge = *next;
    }
    if (current_point != first_point || !geometry::is_simple_polygon(contour)) {
      return std::nullopt;
    }
    contours.push_back(std::move(contour));
  }
  if (contours.empty()) {
    return std::nullopt;
  }
  for (std::size_t index = 0; index < edges.size(); ++index) {
    if (!bridge_edge[index] && !visited[index]) {
      return std::nullopt;
    }
  }

  const auto outer = std::max_element(contours.begin(), contours.end(),
                                      [](const Polygon& lhs, const Polygon& rhs) {
                                        return std::abs(geometry::signed_double_area(lhs)) <
                                               std::abs(geometry::signed_double_area(rhs));
                                      });
  CopperZoneFill result{std::move(layer), std::move(*outer), {}};
  contours.erase(outer);
  for (auto& hole : contours) {
    if (!geometry::polygon_inside(hole, result.area) ||
        geometry::polygon_boundary_distance_nm(hole, result.area) <= 0.0L ||
        std::any_of(result.holes.begin(), result.holes.end(), [&hole](const Polygon& prior) {
          return geometry::polygons_overlap(hole, prior);
        })) {
      return std::nullopt;
    }
    result.holes.push_back(std::move(hole));
  }
  std::sort(result.holes.begin(), result.holes.end(), [](const Polygon& lhs, const Polygon& rhs) {
    return std::lexicographical_compare(lhs.vertices.begin(), lhs.vertices.end(),
                                        rhs.vertices.begin(), rhs.vertices.end());
  });
  return result;
}

[[nodiscard]] std::optional<CopperZone>
import_copper_zone(const SExpression& zone, Importer& importer, std::string fallback_id) {
  if (find_child(zone, "keepout") != nullptr) {
    return std::nullopt;
  }
  const auto filled_polygons = find_children(zone, "filled_polygon");
  if (filled_polygons.empty()) {
    importer.warning("unfilled_zone_ignored",
                     "copper zone has no saved filled polygons and cannot provide physical copper",
                     zone.location);
    return std::nullopt;
  }
  CopperZone result;
  result.id = ObjectId{stable_token(zone, std::move(fallback_id))};
  result.net = item_net(zone, importer);
  result.name = std::string(named_atom(zone, "name").value_or(""));
  result.locked = has_atom(zone, "locked") || find_child(zone, "locked") != nullptr;
  bool valid = true;
  for (const auto* filled_polygon : filled_polygons) {
    const auto layer = named_atom(*filled_polygon, "layer");
    if (!layer || !importer.copper_layer_index.contains(std::string(*layer))) {
      importer.error("unknown_zone_fill_layer",
                     "filled zone polygon references an unknown copper layer",
                     filled_polygon->location);
      valid = false;
      continue;
    }
    auto fill = decode_filled_region(LayerId{std::string(*layer)},
                                     Polygon{parse_points(*filled_polygon, importer)});
    if (!fill) {
      importer.warning("unsupported_zone_fill_geometry",
                       "filled zone walk cannot be decomposed exactly into one outer contour "
                       "and disjoint holes",
                       filled_polygon->location);
      valid = false;
      continue;
    }
    result.fills.push_back(std::move(*fill));
  }
  return valid && !result.fills.empty() ? std::optional<CopperZone>{std::move(result)}
                                        : std::nullopt;
}

[[nodiscard]] Polygon tentative_envelope_from_geometry(const SExpression& footprint,
                                                       Importer& importer) {
  bool initialized = false;
  std::int64_t min_x = 0;
  std::int64_t min_y = 0;
  std::int64_t max_x = 0;
  std::int64_t max_y = 0;
  const auto include_point = [&](Point point) {
    if (!initialized) {
      min_x = max_x = point.x.value();
      min_y = max_y = point.y.value();
      initialized = true;
      return;
    }
    min_x = std::min(min_x, point.x.value());
    max_x = std::max(max_x, point.x.value());
    min_y = std::min(min_y, point.y.value());
    max_y = std::max(max_y, point.y.value());
  };
  const auto include_box = [&](std::int64_t low_x, std::int64_t low_y, std::int64_t high_x,
                               std::int64_t high_y) {
    include_point(Point{Nanometres{low_x}, Nanometres{low_y}});
    include_point(Point{Nanometres{high_x}, Nanometres{high_y}});
  };
  for (const auto* pad : find_children(footprint, "pad")) {
    const auto position = named_point(*pad, "at", importer);
    const auto* size_expression = find_child(*pad, "size");
    const auto size = size_expression != nullptr
                          ? parse_point(*size_expression, importer, "pad size")
                          : std::nullopt;
    if (!position || !size) {
      continue;
    }
    const auto half_width = static_cast<long double>(std::abs(size->x.value())) / 2.0L;
    const auto half_height = static_cast<long double>(std::abs(size->y.value())) / 2.0L;
    const auto radius = static_cast<std::int64_t>(std::ceil(std::hypot(half_width, half_height)));
    include_box(position->x.value() - radius, position->y.value() - radius,
                position->x.value() + radius, position->y.value() + radius);
  }

  const auto is_body_layer = [](std::string_view layer) {
    return layer.ends_with(".Fab") || layer.ends_with(".SilkS");
  };
  for (const auto& graphic : footprint.children) {
    const auto layer = named_atom(graphic, "layer");
    if (!layer || !is_body_layer(*layer)) {
      continue;
    }
    const auto graphic_head = head(graphic);
    if (graphic_head == "fp_line" || graphic_head == "fp_rect") {
      const auto start = named_point(graphic, "start", importer);
      const auto end = named_point(graphic, "end", importer);
      if (start && end) {
        include_point(*start);
        include_point(*end);
      }
    } else if (graphic_head == "fp_poly") {
      for (const auto point : parse_points(graphic, importer)) {
        include_point(point);
      }
    } else if (graphic_head == "fp_arc") {
      const auto start = named_point(graphic, "start", importer);
      const auto middle = named_point(graphic, "mid", importer);
      const auto end = named_point(graphic, "end", importer);
      if (start && middle && end) {
        constexpr std::int64_t tentative_arc_tolerance_nm = 50'000;
        const auto tolerance = Nanometres{
            std::min(importer.options.arc_chord_tolerance.value(), tentative_arc_tolerance_nm)};
        for (const auto point :
             tessellate_arc(*start, *middle, *end, tolerance, importer, graphic.location)) {
          include_point(point);
        }
      }
    } else if (graphic_head == "fp_circle") {
      const auto center = named_point(graphic, "center", importer);
      const auto edge = named_point(graphic, "end", importer);
      if (center && edge) {
        const auto dx = static_cast<long double>(edge->x.value() - center->x.value());
        const auto dy = static_cast<long double>(edge->y.value() - center->y.value());
        const auto radius = static_cast<std::int64_t>(std::ceil(std::hypot(dx, dy)));
        include_box(center->x.value() - radius, center->y.value() - radius,
                    center->x.value() + radius, center->y.value() + radius);
      }
    }
  }
  if (!initialized) {
    importer.warning("courtyard_default_box",
                     "footprint has no usable courtyard, pad, Fab, or Silk geometry; used a 1 mm "
                     "tentative box",
                     footprint.location, false);
    constexpr std::int64_t half_extent = 500'000;
    return Polygon{{Point{Nanometres{-half_extent}, Nanometres{-half_extent}},
                    Point{Nanometres{half_extent}, Nanometres{-half_extent}},
                    Point{Nanometres{half_extent}, Nanometres{half_extent}},
                    Point{Nanometres{-half_extent}, Nanometres{half_extent}}}};
  }
  constexpr std::int64_t margin = 250'000;
  return Polygon{{Point{Nanometres{min_x - margin}, Nanometres{min_y - margin}},
                  Point{Nanometres{max_x + margin}, Nanometres{min_y - margin}},
                  Point{Nanometres{max_x + margin}, Nanometres{max_y + margin}},
                  Point{Nanometres{min_x - margin}, Nanometres{max_y + margin}}}};
}

struct ImportedPlacementEnvelope {
  Polygon courtyard;
  std::vector<CourtyardCircle> circles;
  PlacementEnvelopeSource source{PlacementEnvelopeSource::absent};
};

[[nodiscard]] ImportedPlacementEnvelope import_courtyard(const SExpression& footprint,
                                                         BoardSide side, Importer& importer) {
  const auto courtyard_layer = side == BoardSide::front ? "F.CrtYd" : "B.CrtYd";
  std::vector<RawSegment> segments;
  std::vector<CourtyardCircle> circles;
  for (const auto& child : footprint.children) {
    if ((head(child) == "fp_line" || head(child) == "fp_rect" || head(child) == "fp_poly" ||
         head(child) == "fp_arc" || head(child) == "fp_circle") &&
        shape_on_layer(child, courtyard_layer)) {
      if (head(child) == "fp_circle") {
        const auto center = named_point(child, "center", importer);
        const auto edge = named_point(child, "end", importer);
        if (center && edge) {
          circles.push_back(CourtyardCircle{*center, *edge});
        }
      }
      append_shape_segments(child, segments, importer, CurveProxyPolicy::conservative_courtyard);
    } else if (head(child).starts_with("fp_") && shape_on_layer(child, courtyard_layer)) {
      importer.error("unsupported_courtyard_shape",
                     "unsupported graphic primitive appears on the courtyard layer",
                     child.location);
    }
  }
  if (segments.empty()) {
    if (importer.options.derive_tentative_placement_envelopes ||
        importer.options.approximate_missing_courtyards) {
      importer.warning(
          "tentative_placement_envelope",
          "missing courtyard was provisionally bounded from pads and Fab/Silk graphics and "
          "requires post-process human review",
          footprint.location, false);
      return ImportedPlacementEnvelope{tentative_envelope_from_geometry(footprint, importer),
                                       {},
                                       PlacementEnvelopeSource::tentative_from_geometry};
    }
    importer.warning("missing_courtyard",
                     "footprint has no courtyard; placement changes are prohibited until an "
                     "explicit envelope is supplied",
                     footprint.location, false);
    return ImportedPlacementEnvelope{};
  }
  auto loops = assemble_loops(std::move(segments), importer, "footprint courtyard");
  if (loops.empty()) {
    return ImportedPlacementEnvelope{};
  }
  std::sort(loops.begin(), loops.end(), [](const auto& lhs, const auto& rhs) {
    return std::abs(geometry::signed_double_area(lhs)) >
           std::abs(geometry::signed_double_area(rhs));
  });
  if (loops.size() > 1U) {
    importer.warning("courtyard_multiple_contours",
                     "only the largest footprint courtyard contour was retained",
                     footprint.location);
  }
  return ImportedPlacementEnvelope{std::move(loops.front()), std::move(circles),
                                   PlacementEnvelopeSource::explicit_courtyard};
}

[[nodiscard]] NetId pad_net(const SExpression& pad, Importer& importer) {
  const auto ordinal = named_atom(pad, "net");
  if (!ordinal) {
    return importer.net_by_ordinal.at("0");
  }
  const auto found = importer.net_by_ordinal.find(std::string(*ordinal));
  if (found == importer.net_by_ordinal.end()) {
    importer.error("unknown_net_ordinal", "pad references an undeclared net ordinal", pad.location);
    return importer.net_by_ordinal.at("0");
  }
  return found->second;
}

[[nodiscard]] PadShape parse_pad_shape(std::string_view value) {
  if (value == "circle") {
    return PadShape::circle;
  }
  if (value == "rect") {
    return PadShape::rectangle;
  }
  if (value == "oval") {
    return PadShape::oval;
  }
  if (value == "roundrect") {
    return PadShape::round_rectangle;
  }
  if (value == "trapezoid") {
    return PadShape::trapezoid;
  }
  if (value == "custom") {
    return PadShape::custom;
  }
  return PadShape::unknown;
}

[[nodiscard]] Nanometres parse_pad_drill(const SExpression& pad, Importer& importer) {
  const auto* drill = find_child(pad, "drill");
  if (drill == nullptr) {
    return Nanometres{0};
  }
  const auto first = atom_at(*drill, 1);
  const auto first_number = first ? parse_double(*first) : std::nullopt;
  if (first_number) {
    return Nanometres::from_mm(*first_number);
  }
  if (first && *first == "oval") {
    const auto width = atom_at(*drill, 2);
    const auto height = atom_at(*drill, 3);
    const auto width_value = width ? parse_double(*width) : std::nullopt;
    const auto height_value = height ? parse_double(*height) : std::nullopt;
    if (width_value && height_value) {
      return Nanometres::from_mm(std::max(*width_value, *height_value));
    }
  }
  importer.error("invalid_pad_drill", "pad drill dimensions are invalid", drill->location);
  return Nanometres{0};
}

void import_footprints(const SExpression& root, Board& board, Importer& importer) {
  std::size_t footprint_index = 0;
  for (const auto* expression : find_children(root, "footprint")) {
    const auto fallback = "footprint-" + std::to_string(footprint_index++);
    Footprint footprint;
    footprint.id = ObjectId{stable_token(*expression, fallback)};
    footprint.reference = fallback;
    for (const auto* property : find_children(*expression, "property")) {
      if (atom_at(*property, 1) == std::optional<std::string_view>{"Reference"}) {
        if (const auto reference = atom_at(*property, 2)) {
          footprint.reference = std::string(*reference);
        }
      }
    }
    const auto layer = named_atom(*expression, "layer");
    footprint.side = layer && *layer == "B.Cu" ? BoardSide::back : BoardSide::front;
    const auto* at = find_child(*expression, "at");
    if (at == nullptr) {
      importer.error("missing_footprint_position", "footprint has no at position",
                     expression->location);
      continue;
    }
    const auto position = parse_point(*at, importer, "footprint at");
    if (!position) {
      continue;
    }
    footprint.position = *position;
    if (const auto angle = atom_at(*at, 3)) {
      if (const auto degrees = parse_double(*angle)) {
        footprint.rotation = Microdegrees::from_degrees(*degrees);
      } else {
        importer.error("invalid_rotation", "footprint rotation is not numeric", at->location);
      }
    }
    footprint.locked = has_atom(*expression, "locked") || find_child(*expression, "locked");
    const bool owns_board_edge = std::any_of(
        expression->children.begin(), expression->children.end(), [](const auto& child) {
          return head(child).starts_with("fp_") && shape_on_layer(child, "Edge.Cuts");
        });
    if (owns_board_edge && !footprint.locked) {
      footprint.locked = true;
      importer.warning("footprint_edge_cuts_lock_placement",
                       "a footprint contributing Edge.Cuts is conservatively locked because "
                       "moving it would change board geometry",
                       expression->location, false);
    }
    auto envelope = import_courtyard(*expression, footprint.side, importer);
    footprint.courtyard = std::move(envelope.courtyard);
    footprint.courtyard_circles = std::move(envelope.circles);
    footprint.placement_envelope_source = envelope.source;

    std::size_t keepout_index = 0;
    for (const auto* zone : find_children(*expression, "zone")) {
      const auto fallback_id = footprint.id.value() + "-keepout-" + std::to_string(keepout_index++);
      if (const auto keepout = import_keepout_zone(*zone, board, importer, fallback_id)) {
        footprint.keepouts.push_back(*keepout);
      } else if (find_child(*zone, "keepout") == nullptr) {
        importer.warning("footprint_copper_zone_ignored",
                         "non-keepout footprint zone is not represented in the canonical IR",
                         zone->location);
      }
    }

    std::size_t pad_index = 0;
    for (const auto* pad_expression : find_children(*expression, "pad")) {
      const auto pad_name = atom_at(*pad_expression, 1).value_or("unnamed");
      Pad pad;
      pad.id = ObjectId{stable_token(*pad_expression, footprint.id.value() + "-pad-" +
                                                          std::to_string(pad_index++) + "-" +
                                                          std::string(pad_name))};
      pad.net = pad_net(*pad_expression, importer);
      if (const auto* pad_at = find_child(*pad_expression, "at")) {
        if (const auto local = parse_point(*pad_at, importer, "pad at")) {
          pad.local_position = *local;
        }
        if (const auto angle = atom_at(*pad_at, 3)) {
          if (const auto degrees = parse_double(*angle)) {
            const auto file_rotation = Microdegrees::from_degrees(*degrees);
            // A board-file pad position is footprint-relative, but its angle is serialized in
            // placed board coordinates (KiCad's formatter uses PAD::GetOrientation()). Retain a
            // local angle in the canonical IR so later footprint rotation continues to rotate
            // the pad exactly once.
            pad.rotation = footprint.side == BoardSide::front
                               ? Microdegrees{file_rotation.value() - footprint.rotation.value()}
                               : Microdegrees{footprint.rotation.value() - file_rotation.value()};
          } else {
            importer.error("invalid_pad_rotation", "pad rotation is not numeric", pad_at->location);
          }
        }
      }
      pad.layers = expand_layers(*pad_expression, board, importer);
      if (pad.layers.empty()) {
        importer.warning("non_copper_pad_ignored",
                         "pad has no explicit copper layer and was omitted",
                         pad_expression->location, false);
        continue;
      }
      if (const auto shape = atom_at(*pad_expression, 3)) {
        pad.shape = parse_pad_shape(*shape);
      }
      if (pad.shape == PadShape::trapezoid || pad.shape == PadShape::custom ||
          pad.shape == PadShape::unknown) {
        importer.warning("pad_shape_approximated",
                         "trapezoid, custom, or unknown pad shape was reduced to its size envelope",
                         pad_expression->location);
      }
      if (const auto* size = find_child(*pad_expression, "size")) {
        if (const auto dimensions = parse_point(*size, importer, "pad size")) {
          pad.width = Nanometres{std::abs(dimensions->x.value())};
          pad.height = Nanometres{std::abs(dimensions->y.value())};
        }
      } else {
        importer.error("missing_pad_size", "copper pad has no size", pad_expression->location);
      }
      if (pad.shape == PadShape::round_rectangle) {
        const auto ratio = named_number(*pad_expression, "roundrect_rratio", importer);
        if (!ratio) {
          importer.error("missing_roundrect_ratio",
                         "round-rectangle pad has no corner-radius ratio",
                         pad_expression->location);
        } else if (*ratio < 0.0 || *ratio > 0.5) {
          importer.error("invalid_roundrect_ratio",
                         "round-rectangle corner-radius ratio must be between zero and one half",
                         pad_expression->location);
        } else {
          const auto minimum_dimension = std::min(pad.width.value(), pad.height.value());
          pad.corner_radius = Nanometres{static_cast<std::int64_t>(
              std::llround(static_cast<long double>(minimum_dimension) * *ratio))};
        }
      }
      pad.drill = parse_pad_drill(*pad_expression, importer);
      pad.plated = atom_at(*pad_expression, 2) != std::optional<std::string_view>{"np_thru_hole"};
      footprint.pads.push_back(std::move(pad));
    }
    board.footprints.push_back(std::move(footprint));
  }
}

[[nodiscard]] NetId item_net(const SExpression& item, Importer& importer) {
  const auto ordinal = named_atom(item, "net");
  if (!ordinal) {
    importer.error("missing_item_net", "copper item has no net ordinal", item.location);
    return importer.net_by_ordinal.at("0");
  }
  const auto found = importer.net_by_ordinal.find(std::string(*ordinal));
  if (found == importer.net_by_ordinal.end()) {
    importer.error("unknown_net_ordinal", "copper item references an undeclared net ordinal",
                   item.location);
    return importer.net_by_ordinal.at("0");
  }
  return found->second;
}

void import_tracks_and_vias(const SExpression& root, Board& board, Importer& importer) {
  std::size_t track_index = 0;
  for (const auto* expression : find_children(root, "segment")) {
    const auto start = named_point(*expression, "start", importer);
    const auto end = named_point(*expression, "end", importer);
    const auto width = named_number(*expression, "width", importer);
    const auto layer = named_atom(*expression, "layer");
    if (!start || !end || !width || !layer) {
      importer.error("invalid_segment", "track segment is incomplete", expression->location);
      continue;
    }
    board.tracks.push_back(
        TrackSegment{ObjectId{stable_token(*expression, "track-" + std::to_string(track_index++))},
                     item_net(*expression, importer), LayerId{std::string(*layer)}, *start, *end,
                     Nanometres::from_mm(*width), has_atom(*expression, "locked")});
  }

  std::size_t arc_index = 0;
  for (const auto* expression : find_children(root, "arc")) {
    const auto start = named_point(*expression, "start", importer);
    const auto middle = named_point(*expression, "mid", importer);
    const auto end = named_point(*expression, "end", importer);
    const auto width = named_number(*expression, "width", importer);
    const auto layer = named_atom(*expression, "layer");
    if (!start || !middle || !end || !width || !layer) {
      importer.error("invalid_track_arc", "track arc is incomplete", expression->location);
      continue;
    }
    board.track_arcs.push_back(
        TrackArc{ObjectId{stable_token(*expression, "track-arc-" + std::to_string(arc_index++))},
                 item_net(*expression, importer), LayerId{std::string(*layer)}, *start, *middle,
                 *end, Nanometres::from_mm(*width), has_atom(*expression, "locked")});
  }

  std::size_t via_index = 0;
  for (const auto* expression : find_children(root, "via")) {
    const auto position = named_point(*expression, "at", importer);
    const auto diameter = named_number(*expression, "size", importer);
    const auto drill = named_number(*expression, "drill", importer);
    auto layers = expand_layers(*expression, board, importer);
    if (!position || !diameter || !drill || layers.empty()) {
      importer.error("invalid_via", "via is missing position, dimensions, or layers",
                     expression->location);
      continue;
    }
    if (layers.size() == 2U) {
      const auto first = importer.copper_layer_index.find(layers[0].value());
      const auto second = importer.copper_layer_index.find(layers[1].value());
      if (first != importer.copper_layer_index.end() &&
          second != importer.copper_layer_index.end()) {
        const auto low = std::min(first->second, second->second);
        const auto high = std::max(first->second, second->second);
        layers.clear();
        for (const auto& board_layer : board.layers) {
          if (board_layer.kind == LayerKind::copper && board_layer.stack_order >= low &&
              board_layer.stack_order <= high) {
            layers.push_back(board_layer.id);
          }
        }
      }
    }
    board.vias.push_back(
        Via{ObjectId{stable_token(*expression, "via-" + std::to_string(via_index++))},
            item_net(*expression, importer), *position, Nanometres::from_mm(*diameter),
            Nanometres::from_mm(*drill), std::move(layers), has_atom(*expression, "locked")});
  }

  std::size_t keepout_index = 0;
  std::size_t copper_zone_index = 0;
  for (const auto* zone : find_children(root, "zone")) {
    if (const auto keepout = import_keepout_zone(*zone, board, importer,
                                                 "keepout-" + std::to_string(keepout_index))) {
      board.keepouts.push_back(*keepout);
      ++keepout_index;
    } else if (const auto copper = import_copper_zone(
                   *zone, importer, "zone-" + std::to_string(copper_zone_index))) {
      board.zones.push_back(*copper);
      ++copper_zone_index;
    }
  }
}

[[nodiscard]] KicadImportResult translate(const SExpression& root, KicadImportOptions options,
                                          std::string_view fingerprint) {
  Importer importer{std::move(options)};
  if (head(root) != "kicad_pcb") {
    importer.error("wrong_root", "root expression must be kicad_pcb", root.location);
    return std::move(importer.result);
  }
  Board board;
  const auto version = named_atom(root, "version").value_or("unknown");
  const auto generator = named_atom(root, "generator").value_or("unknown");
  board.source_revision = "kicad-" + std::string(version) + ":" + std::string(generator) + ":" +
                          std::string(fingerprint);
  import_layers(root, board, importer);
  import_nets(root, board, importer);
  infer_differential_pairs(board);
  import_outline(root, board, importer);
  import_footprints(root, board, importer);
  import_tracks_and_vias(root, board, importer);
  importer.result.board = std::move(board);
  return std::move(importer.result);
}

} // namespace

bool KicadImportResult::has_errors() const {
  return std::any_of(diagnostics.begin(), diagnostics.end(), [](const auto& diagnostic) {
    return diagnostic.severity == ImportSeverity::error;
  });
}

KicadImportResult import_kicad_pcb(std::string_view contents, const KicadImportOptions& options) {
  auto parsed = parse_sexpression(contents);
  if (!parsed.ok()) {
    KicadImportResult result;
    for (const auto& diagnostic : parsed.diagnostics) {
      result.diagnostics.push_back(ImportDiagnostic{ImportSeverity::error, diagnostic.code,
                                                    diagnostic.message, diagnostic.location.line,
                                                    diagnostic.location.column});
    }
    return result;
  }
  return translate(*parsed.root, options, content_fingerprint(contents));
}

KicadImportResult import_kicad_pcb_file(const std::filesystem::path& path,
                                        const KicadImportOptions& options) {
  std::ifstream input(path, std::ios::binary);
  if (!input) {
    KicadImportResult result;
    result.diagnostics.push_back(ImportDiagnostic{ImportSeverity::error, "file_open_failed",
                                                  "unable to open KiCad board file", 1, 1});
    return result;
  }
  const std::string contents{std::istreambuf_iterator<char>{input},
                             std::istreambuf_iterator<char>{}};
  if (!input.good() && !input.eof()) {
    KicadImportResult result;
    result.diagnostics.push_back(ImportDiagnostic{ImportSeverity::error, "file_read_failed",
                                                  "unable to read KiCad board file", 1, 1});
    return result;
  }
  return import_kicad_pcb(contents, options);
}

} // namespace flux::io
