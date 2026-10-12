#include "flux/core/copper_geometry.hpp"
#include "flux/core/routing_rules.hpp"
#include "flux/io/kicad_import.hpp"
#include "../../../src/scaffold_access.hpp"
#include "../../../src/scaffold_collision.hpp"

#include <boost/json.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <iostream>
#include <iterator>
#include <map>
#include <optional>
#include <set>
#include <string>
#include <vector>

using namespace flux;
using boost::json::value;

namespace {

std::string text(const value& item) { return std::string(item.as_string()); }
Point point(const value& item) {
  const auto& object = item.as_object();
  return {Nanometres{object.at("x_nm").as_int64()}, Nanometres{object.at("y_nm").as_int64()}};
}
void add_fixed(Board& board, const value& item) {
  const auto& object = item.as_object();
  const auto& points = object.at("points").as_array();
  const auto& layers = object.at("layers").as_array();
  if (text(object.at("kind")) == "track") {
    board.tracks.push_back({ObjectId{text(object.at("id"))}, NetId{text(object.at("net"))},
                            LayerId{text(layers.front())}, point(points[0]), point(points[1]),
                            Nanometres{object.at("width_nm").as_int64()}, false});
  } else if (text(object.at("kind")) == "via") {
    std::vector<LayerId> span;
    for (const auto& layer : layers) span.emplace_back(text(layer));
    board.vias.push_back({ObjectId{text(object.at("id"))}, NetId{text(object.at("net"))}, point(points[0]),
                          Nanometres{object.at("diameter_nm").as_int64()},
                          Nanometres{object.at("drill_nm").as_int64()}, std::move(span), false});
  }
}
std::string escape(const std::string& input) {
  std::string result;
  for (const auto character : input) {
    if (character == '"' || character == '\\') result.push_back('\\');
    result.push_back(character);
  }
  return result;
}
std::string kind_for(const Board& board, const std::string& id) {
  if (std::any_of(board.tracks.begin(), board.tracks.end(), [&](const auto& item) { return item.id.value() == id; })) return "track";
  if (std::any_of(board.vias.begin(), board.vias.end(), [&](const auto& item) { return item.id.value() == id; })) return "via";
  for (const auto& footprint : board.footprints)
    if (std::any_of(footprint.pads.begin(), footprint.pads.end(), [&](const auto& item) { return item.id.value() == id; })) return "pad";
  return "other";
}
struct ProbeSummary {
  std::size_t legal{0U};
  std::set<std::pair<std::string, std::string>> blockers;
};
void absorb(ProbeSummary& summary, const CopperDrcReport& report, const ObjectId& probe,
            const Board& board) {
  for (const auto& violation : report.violations) for (const auto& id : violation.object_ids)
    if (id != probe.value()) summary.blockers.emplace(id, kind_for(board, id));
}
Point grid_point(Point origin, std::int64_t x, std::int64_t y) {
  constexpr std::int64_t pitch = 100'000; // The diagnostic uses Flux's 0.1 mm scaffold lattice.
  const auto floor_grid = [pitch](std::int64_t value) {
    return static_cast<std::int64_t>(std::floor(static_cast<long double>(value) / pitch));
  };
  return {Nanometres{(floor_grid(origin.x.value()) + x) * pitch},
          Nanometres{(floor_grid(origin.y.value()) + y) * pitch}};
}
ProbeSummary ring_probe(const Board& board, const ScaffoldCollisionIndex& collision, const NetId& net,
                        const LayerId& layer, Point anchor, Point origin, const std::string& prefix,
                        bool require_first_leg) {
  ProbeSummary result;
  const auto rules = resolve_net_routing_rules(board, net);
  const ObjectId first{prefix + "-stub"};
  if (require_first_leg) {
    const TrackSegment leg{first, net, layer, anchor, origin, rules.track_width, false};
    const auto report = collision.check_track(leg);
    absorb(result, report, first, board);
    if (!report.ok()) return result;
  }
  for (std::int64_t y = -2; y <= 2; ++y) for (std::int64_t x = -2; x <= 2; ++x) {
    if (std::max(std::abs(x), std::abs(y)) > 2) continue;
    const auto candidate = grid_point(origin, x, y);
    const ObjectId id{prefix + "-grid-" + std::to_string(x) + "-" + std::to_string(y)};
    const TrackSegment leg{id, net, layer, origin, candidate, rules.track_width, false};
    const auto report = collision.check_track(leg);
    if (report.ok()) ++result.legal;
    else absorb(result, report, id, board);
  }
  return result;
}
void print_summary(const ProbeSummary& summary) {
  std::cout << "{\"legal_grid_attachments\":" << summary.legal << ",\"violations\":[";
  bool first = true;
  for (const auto& [id, kind] : summary.blockers) {
    if (!first) std::cout << ',';
    first = false;
    std::cout << "{\"id\":\"" << escape(id) << "\",\"kind\":\"" << kind << "\"}";
  }
  std::cout << "]}";
}
std::optional<CopperPadShape> pad_at(const Board& board, const NetId& net, const LayerId& layer,
                                      Point anchor) {
  for (const auto& pad : copper_pad_shapes(board))
    if (pad.net == net && std::find(pad.layers.begin(), pad.layers.end(), layer) != pad.layers.end() &&
        point_to_pad_copper_distance_nm(anchor, pad) <= 0.0L)
      return pad;
  return std::nullopt;
}
std::optional<Point> stub_at(const CopperPadShape& pad, Point anchor, long double extra) {
  const auto dx = static_cast<long double>(anchor.x.value() - pad.position.x.value());
  const auto dy = static_cast<long double>(anchor.y.value() - pad.position.y.value());
  const auto length = std::hypotl(dx, dy);
  if (length <= 0.0L) return std::nullopt;
  const auto radius = pad.conservative_radius_nm + extra;
  return Point{Nanometres{static_cast<std::int64_t>(std::llround(pad.position.x.value() + dx / length * radius))},
               Nanometres{static_cast<std::int64_t>(std::llround(pad.position.y.value() + dy / length * radius))}};
}
void erase_request(Board& board, int request) {
  const auto prefix = std::string{"__flux-copper-scaffold-"} + std::to_string(request) + "-";
  std::erase_if(board.tracks, [&](const auto& item) { return item.id.value().starts_with(prefix); });
  std::erase_if(board.vias, [&](const auto& item) { return item.id.value().starts_with(prefix); });
}
void inspect(const Board& board, const NetId& net, Point endpoint, const char* label, const char* scene,
             bool& first_record) {
  const LayerId layer{"F.Cu"};
  GlobalRoutePath request;
  request.net = net;
  request.points = {{endpoint, layer}, {endpoint, layer}};
  const auto access = scaffold_connection_access(board, request, 128U);
  const ScaffoldCollisionIndex collision(board);
  const auto rules = resolve_net_routing_rules(board, net);
  for (const auto& anchor : access.goals) {
    if (!first_record) std::cout << ',';
    first_record = false;
    std::cout << "{\"scene\":\"" << scene << "\",\"endpoint\":\"" << label
              << "\",\"anchor\":{\"x_nm\":" << anchor.position.x.value() << ",\"y_nm\":"
              << anchor.position.y.value() << ",\"layer\":\"" << anchor.layer.value() << "\"},\"direct\":";
    print_summary(ring_probe(board, collision, net, layer, anchor.position, anchor.position,
                             "direct-" + std::string(label), false));
    const auto pad = pad_at(board, net, layer, anchor.position);
    std::cout << ",\"stubs\":[";
    const std::array<std::pair<const char*, long double>, 3U> lengths{{
        {"radius_plus_pitch", pad ? static_cast<long double>(rules.track_width.value()) / 2.0L + 100'000.0L : 0.0L},
        {"radius_plus_clearance_halfwidth", pad ? static_cast<long double>(rules.clearance.value() + rules.track_width.value() / 2) : 0.0L},
        {"current_long", pad ? static_cast<long double>(rules.clearance.value() + rules.track_width.value() / 2 + 100'000) : 0.0L}}};
    for (std::size_t index = 0U; index < lengths.size(); ++index) {
      if (index) std::cout << ',';
      std::cout << "{\"kind\":\"" << lengths[index].first << "\"";
      if (pad) if (const auto stub = stub_at(*pad, anchor.position, lengths[index].second)) {
        std::cout << ",\"point\":{\"x_nm\":" << stub->x.value() << ",\"y_nm\":" << stub->y.value()
                  << "},\"probe\":";
        print_summary(ring_probe(board, collision, net, layer, anchor.position, *stub,
                                 "stub-" + std::string(label) + "-" + std::to_string(index), true));
      }
      std::cout << '}';
    }
    std::cout << "]}";
  }
}
} // namespace

int main(int argc, char** argv) {
  if (argc != 3) return 2;
  const auto imported = io::import_kicad_pcb_file(argv[1]);
  if (!imported.ok()) return 3;
  std::ifstream input(argv[2]);
  const auto parsed = boost::json::parse(std::string{std::istreambuf_iterator<char>(input), {}});
  Board board = *imported.board;
  for (const auto& primitive : parsed.as_object().at("primitives").as_array()) add_fixed(board, primitive);
  std::cout << '[';
  bool first_record = true;
  inspect(board, NetId{"kicad-net-13"}, Point{Nanometres{123750000}, Nanometres{112675000}}, "44", "full", first_record);
  inspect(board, NetId{"kicad-net-3"}, Point{Nanometres{122750000}, Nanometres{112675000}}, "91", "full", first_record);
  for (const auto& owners : std::vector<std::vector<int>>{{43}, {43,45}, {11}, {19}, {11,19}, {43,45,11,19}}) {
    Board candidate = board;
    std::string scene = "without";
    for (int owner : owners) { erase_request(candidate, owner); scene += "_" + std::to_string(owner); }
    inspect(candidate, NetId{"kicad-net-13"}, Point{Nanometres{123750000}, Nanometres{112675000}}, "44", scene.c_str(), first_record);
    inspect(candidate, NetId{"kicad-net-3"}, Point{Nanometres{122750000}, Nanometres{112675000}}, "91", scene.c_str(), first_record);
  }
  std::cout << "]\n";
}
