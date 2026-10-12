// Offline diagnostic only.  It deliberately consumes saved Round21 physical copper and
// canonical scaffold request identities; it does not invoke the global router.
#include "flux/core/connectivity.hpp"
#include "flux/core/copper_scaffold.hpp"
#include "flux/core/drc.hpp"
#include "flux/io/kicad_import.hpp"

#include <boost/json.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <set>
#include <fstream>
#include <iostream>
#include <iterator>
#include <map>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

using namespace flux;
using boost::json::value;

namespace {

std::string text(const value& item) { return std::string(item.as_string()); }
Point point(const value& item) {
  const auto& object = item.as_object();
  return {Nanometres{object.at("x_nm").as_int64()}, Nanometres{object.at("y_nm").as_int64()}};
}

// Generated-copper telemetry has already been flattened to track/via primitives.  The fixture
// itself is imported separately so pads, rules, outline, keepouts, and stackup stay canonical.
void add_generated(Board& board, const value& item) {
  const auto& object = item.as_object();
  const auto kind = text(object.at("kind"));
  if (kind != "track" && kind != "via") return;
  const auto& points = object.at("points").as_array();
  const auto& layers = object.at("layers").as_array();
  if (kind == "track") {
    board.tracks.push_back({ObjectId{text(object.at("id"))}, NetId{text(object.at("net"))},
                            LayerId{text(layers.front())}, point(points[0]), point(points[1]),
                            Nanometres{object.at("width_nm").as_int64()}, false});
    return;
  }
  std::vector<LayerId> span;
  for (const auto& layer : layers) span.emplace_back(text(layer));
  board.vias.push_back({ObjectId{text(object.at("id"))}, NetId{text(object.at("net"))}, point(points[0]),
                        Nanometres{object.at("diameter_nm").as_int64()},
                        Nanometres{object.at("drill_nm").as_int64()}, std::move(span), false});
}

void erase_owner(Board& board, std::size_t owner) {
  const auto prefix = "__flux-copper-scaffold-" + std::to_string(owner) + "-";
  std::erase_if(board.tracks, [&](const auto& track) { return track.id.value().starts_with(prefix); });
  std::erase_if(board.vias, [&](const auto& via) { return via.id.value().starts_with(prefix); });
}

// Canonical identity format is private to scaffold implementation, but is intentionally emitted
// as stable evidence.  Parse only the path payload, then reject anything malformed rather than
// substituting a route candidate.
class IdentityReader {
 public:
  explicit IdentityReader(std::string_view input) : input_(input) {}
  std::string field() {
    const auto end = input_.find(':', position_);
    if (end == std::string_view::npos) {
      if (position_ >= input_.size()) throw std::runtime_error("truncated canonical identity");
      const auto output = input_.substr(position_); position_ = input_.size(); return std::string(output);
    }
    const auto output = input_.substr(position_, end - position_); position_ = end + 1U;
    return std::string(output);
  }
  std::string sized() {
    return sized(static_cast<std::size_t>(std::stoull(field())));
  }
  std::string sized(std::size_t count) {
    if (position_ + count > input_.size()) throw std::runtime_error("sized canonical field overrun");
    const auto output = input_.substr(position_, count); position_ += count;
    if (position_ >= input_.size() || input_[position_] != ':') throw std::runtime_error("sized canonical field delimiter");
    ++position_; return std::string(output);
  }
 private:
  std::string_view input_; std::size_t position_{0U};
};

GlobalRoutePath parse_identity(const std::string& identity) {
  IdentityReader reader(identity); GlobalRoutePath path;
  path.net = NetId{reader.sized()}; path.connection_index = std::stoull(reader.field());
  const auto point_count = static_cast<std::size_t>(std::stoull(reader.field()));
  if (point_count < 2U) throw std::runtime_error("canonical request has fewer than two points");
  for (std::size_t i = 0; i < point_count; ++i) {
    const auto coordinates = reader.field();
    const auto comma1 = coordinates.find(','); const auto comma2 = coordinates.find(',', comma1 + 1U);
    if (comma1 == std::string::npos || comma2 == std::string::npos) throw std::runtime_error("bad canonical point");
    const auto layer_size = static_cast<std::size_t>(std::stoull(coordinates.substr(comma2 + 1U)));
    const auto x = std::stoll(coordinates.substr(0U, comma1));
    const auto y = std::stoll(coordinates.substr(comma1 + 1U, comma2 - comma1 - 1U));
    const auto layer = reader.sized(layer_size);
    if (layer.size() != layer_size) throw std::runtime_error("canonical layer length mismatch");
    path.points.push_back({Point{Nanometres{x}, Nanometres{y}}, LayerId{layer}});
  }
  // Width/kind suffixes follow all points.  A w0 is intentional: scaffold resolves net rules.
  // No explicit width values occur in the target evidence, so only accept that exact form.
  while (true) {
    const auto suffix = reader.field();
    if (suffix.starts_with("w")) {
      const auto width = std::stoll(suffix.substr(1));
      path.segment_widths.push_back(Nanometres{width});
    } else if (suffix.starts_with("k")) {
      path.segment_width_kinds.push_back(static_cast<GlobalRouteSegmentWidthKind>(std::stoi(suffix.substr(1))));
    } else if (suffix.starts_with("neck=")) break;
    else throw std::runtime_error("unexpected canonical suffix");
  }
  return path;
}

value read_global_result(const char* path) {
  std::ifstream input(path);
  if (!input) throw std::runtime_error("cannot open global result");
  return boost::json::parse(std::string{std::istreambuf_iterator<char>(input), {}});
}

std::vector<value> latest_generated_primitives(const char* path) {
  std::ifstream input(path); if (!input) throw std::runtime_error("cannot open telemetry");
  std::vector<value> latest;
  for (std::string line; std::getline(input, line);) {
    if (line.find("generated-copper:latest") == std::string::npos) continue;
    const auto snapshot = boost::json::parse(line).as_object();
    const auto boards = snapshot.if_contains("source_boards"); if (!boards || !boards->is_array()) continue;
    for (const auto& candidate : boards->as_array()) {
      const auto& board = candidate.as_object();
      const auto id = board.if_contains("id");
      if (!id || !id->is_string() || text(*id) != "generated-copper:latest") continue;
      const auto primitives = board.if_contains("primitives");
      if (!primitives || !primitives->is_array()) throw std::runtime_error("generated copper lacks primitives");
      latest.assign(primitives->as_array().begin(), primitives->as_array().end());
    }
  }
  if (latest.empty()) throw std::runtime_error("no generated-copper:latest snapshot found");
  return latest;
}

std::map<std::size_t, GlobalRoutePath> target_requests(const value& global) {
  const auto& evidence = global.as_object().at("result").as_object().at("copper_scaffold").as_object()
      .at("request_evidence").as_array();
  constexpr std::array<std::size_t, 7U> indices{11U, 19U, 43U, 44U, 45U, 91U, 115U};
  std::map<std::size_t, GlobalRoutePath> requests;
  for (const auto index : indices) {
    if (index >= evidence.size()) throw std::runtime_error("target request index absent from evidence");
    const auto& record = evidence[index].as_object();
    auto request = parse_identity(text(record.at("request_identity")));
    if (request.net.value() != text(record.at("net")) || request.connection_index != record.at("connection_index").as_int64())
      throw std::runtime_error("canonical request identity does not match evidence metadata");
    requests.emplace(index, std::move(request));
  }
  return requests;
}

void json_string(std::ostream& out, const std::string& value) { out << boost::json::serialize(value); }
void json_indices(std::ostream& out, const std::vector<std::size_t>& values) {
  out << '['; for (std::size_t i = 0; i < values.size(); ++i) { if (i) out << ','; out << values[i]; } out << ']';
}
void json_blockers(std::ostream& out, const CopperScaffoldRequestEvidence& evidence) {
  out << '['; for (std::size_t i = 0; i < evidence.blocker_objects.size(); ++i) {
    if (i) out << ','; json_string(out, evidence.blocker_objects[i].value());
  } out << ']';
}

std::string drc_fingerprint(const CopperViolation& violation) {
  auto ids = violation.object_ids; std::sort(ids.begin(), ids.end());
  std::ostringstream stream; stream << violation.code;
  for (const auto& id : ids) stream << '\x1f' << id;
  return stream.str();
}
void json_drc_violations(std::ostream& out, const std::vector<CopperViolation>& violations) {
  out << '[';
  for (std::size_t i = 0; i < violations.size(); ++i) {
    if (i) out << ','; const auto& violation = violations[i];
    out << "{\"code\":"; json_string(out, violation.code);
    out << ",\"kind\":" << static_cast<int>(violation.kind) << ",\"object_ids\":[";
    for (std::size_t id = 0; id < violation.object_ids.size(); ++id) {
      if (id) out << ','; json_string(out, violation.object_ids[id]);
    }
    out << "]}";
  }
  out << ']';
}
std::vector<CopperViolation> new_drc_violations(const CopperDrcReport& baseline,
                                                 const CopperDrcReport& candidate) {
  std::multiset<std::string> prior;
  for (const auto& violation : baseline.violations) prior.insert(drc_fingerprint(violation));
  std::vector<CopperViolation> added;
  for (const auto& violation : candidate.violations) {
    const auto found = prior.find(drc_fingerprint(violation));
    if (found == prior.end()) added.push_back(violation); else prior.erase(found);
  }
  return added;
}
void json_attachments(std::ostream& out, const CopperScaffoldRequestEvidence& evidence) {
  out << "{\"source_component_anchors\":" << evidence.source_component_anchors
      << ",\"target_component_anchors\":" << evidence.target_component_anchors
      << ",\"source_legal_attachments\":" << evidence.source_legal_attachments
      << ",\"target_legal_attachments\":" << evidence.target_legal_attachments << '}';
}
void json_incoming_blocker_geometry(std::ostream& out, const Board& board,
                                    const std::vector<CopperScaffoldRequestEvidence>& evidence) {
  std::set<std::string> blocker_ids;
  for (std::size_t request = 1U; request < evidence.size(); ++request)
    for (const auto& id : evidence[request].blocker_objects)
      if (id.value().starts_with("__flux-copper-scaffold-0-")) blocker_ids.insert(id.value());
  out << '['; bool first = true;
  const auto emit = [&](const std::string& id, const char* kind, const NetId& net, Point start,
                        std::optional<Point> end, const std::vector<LayerId>& layers) {
    if (!blocker_ids.contains(id)) return;
    if (!first) out << ','; first = false;
    out << "{\"id\":"; json_string(out, id); out << ",\"kind\":"; json_string(out, kind);
    out << ",\"net\":"; json_string(out, net.value());
    out << ",\"start\":{\"x_nm\":" << start.x.value() << ",\"y_nm\":" << start.y.value() << '}';
    if (end) out << ",\"end\":{\"x_nm\":" << end->x.value() << ",\"y_nm\":" << end->y.value() << '}';
    out << ",\"layers\":[";
    for (std::size_t layer = 0; layer < layers.size(); ++layer) { if (layer) out << ','; json_string(out, layers[layer].value()); }
    out << "]}";
  };
  for (const auto& track : board.tracks) emit(track.id.value(), "track", track.net, track.start, track.end, {track.layer});
  for (const auto& via : board.vias) emit(via.id.value(), "via", via.net, via.position, std::nullopt, via.layers);
  out << ']';
}

struct Scenario { const char* name; std::size_t incoming; std::vector<std::size_t> victims; };

}  // namespace

int main(int argc, char** argv) {
  if (argc < 4) {
    std::cerr << "usage: replay_joint_repair <ft2232h_jtag.kicad_pcb> <round21-global-result.json> <round21-global.jsonl>\n";
    return 2;
  }
  try {
    const auto imported = io::import_kicad_pcb_file(argv[1]);
    if (!imported.ok()) throw std::runtime_error("cannot import PCB");
    const auto requests = target_requests(read_global_result(argv[2]));
    const auto copper = latest_generated_primitives(argv[3]);
    Board saved = *imported.board; for (const auto& primitive : copper) add_generated(saved, primitive);
    const auto baseline_drc = check_copper_rules(saved);

    const std::vector<Scenario> scenarios{{"incoming-115-no-victims", 115U, {}}};

    std::cout << "{\"source\":{\"generated_copper\":\"generated-copper:latest\",\"target_indices\":[11,19,43,44,45,91,115]},\"baseline_exact_drc\":{\"count\":" << baseline_drc.violations.size() << ",\"violations\":";
    json_drc_violations(std::cout, baseline_drc.violations); std::cout << "},\"outcomes\":[";
    for (std::size_t ordinal = 0; ordinal < scenarios.size(); ++ordinal) {
      const auto& scenario = scenarios[ordinal]; Board base = saved;
      if (argc > 6 && std::string_view{argv[6]} == "static") base = *imported.board;
      for (const auto victim : scenario.victims) erase_owner(base, victim);
      if (argc > 5) base.rules.default_track_width = Nanometres::from_mm(std::stold(argv[5]));
      std::vector<GlobalRoutePath> ordered{requests.at(scenario.incoming)};
      for (const auto victim : scenario.victims) ordered.push_back(requests.at(victim));
      if (argc > 7 && std::string_view{argv[7]} == "reverse") {
        std::reverse(ordered.front().points.begin(), ordered.front().points.end());
        std::reverse(ordered.front().segment_widths.begin(), ordered.front().segment_widths.end());
        std::reverse(ordered.front().segment_width_kinds.begin(), ordered.front().segment_width_kinds.end());
      }
      CopperScaffoldConfig config;
      config.maximum_expansions_per_connection = argc > 4 ? std::stoull(argv[4]) : 50'000U;
      config.compact_search_expansions = config.maximum_expansions_per_connection / 2;
      config.maximum_victim_exchanges = 0U;
      config.maximum_joint_passage_candidates = 0U;
      config.maximum_displacement_attempts = 0U;
      config.maximum_coupled_victims = 1U;
      const auto started = std::chrono::steady_clock::now();
      const auto result = construct_copper_scaffold(base, ordered, config);
      const auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - started).count();
      const auto drc = check_copper_rules(result.copper); const auto connectivity = analyze_connectivity(result.copper);
      const auto new_drc = new_drc_violations(baseline_drc, drc);
      if (ordinal) std::cout << ',';
      std::cout << "{\"scenario\":"; json_string(std::cout, scenario.name);
      std::cout << ",\"incoming\":" << scenario.incoming << ",\"victims\":"; json_indices(std::cout, scenario.victims);
      std::cout << ",\"complete\":" << (result.complete ? "true" : "false")
                << ",\"satisfied_local_indices\":"; json_indices(std::cout, result.satisfied_indices);
      std::cout << ",\"missing_local_indices\":"; json_indices(std::cout, result.missing_indices);
      std::cout << ",\"runtime_ms\":" << elapsed_ms << ",\"expansions\":" << result.counters.expansions
                << ",\"compact_expansions\":" << result.counters.compact_search_expansions
                << ",\"exact_drc_violations\":" << drc.violations.size()
                << ",\"new_exact_drc_violations\":"; json_drc_violations(std::cout, new_drc);
      std::cout
                << ",\"whole_board_open_connections\":" << connectivity.open_connections
                << ",\"whole_board_unrouted_nets\":" << connectivity.unrouted_nets << ",\"requests\":[";
      for (std::size_t i = 0; i < result.request_evidence.size(); ++i) {
        if (i) std::cout << ','; const auto& evidence = result.request_evidence[i];
        std::cout << "{\"index\":" << (i == 0U ? scenario.incoming : scenario.victims[i - 1U])
                  << ",\"termination\":"; json_string(std::cout, std::string(to_string(evidence.termination)));
        std::cout << ",\"blockers\":"; json_blockers(std::cout, evidence);
        std::cout << ",\"attachments\":"; json_attachments(std::cout, evidence); std::cout << '}';
      }
      std::cout << "],\"incoming_blocker_geometry\":";
      json_incoming_blocker_geometry(std::cout, result.copper, result.request_evidence); std::cout << '}';
    }
    std::cout << "]}\n";
  } catch (const std::exception& error) {
    std::cout << "{\"error\":"; json_string(std::cout, error.what()); std::cout << "}\n"; return 1;
  }
}
