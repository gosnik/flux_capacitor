#include "flux/core/copper_scaffold.hpp"
#include "flux/core/drc.hpp"
#include "flux/core/routing_rules.hpp"
#include "flux/io/kicad_import.hpp"
#include <boost/json.hpp>
#include <chrono>
#include <fstream>
#include <iostream>
#include <iterator>

using namespace flux;
using boost::json::value;

static std::string s(const value& v) { return std::string(v.as_string()); }
static Point pt(const value& v) {
  const auto& o = v.as_object();
  return {Nanometres{o.at("x_nm").as_int64()}, Nanometres{o.at("y_nm").as_int64()}};
}

static void add_fixed(Board& b, const value& v) {
  const auto& o = v.as_object();
  const auto& points = o.at("points").as_array();
  const auto& layers = o.at("layers").as_array();
  if (s(o.at("kind")) == "track") {
    b.tracks.push_back({ObjectId{s(o.at("id"))}, NetId{s(o.at("net"))}, LayerId{s(layers.front())},
                        pt(points[0]), pt(points[1]), Nanometres{o.at("width_nm").as_int64()}, false});
  } else if (s(o.at("kind")) == "via") {
    std::vector<LayerId> span;
    for (const auto& l : layers) span.emplace_back(s(l));
    b.vias.push_back({ObjectId{s(o.at("id"))}, NetId{s(o.at("net"))}, pt(points[0]),
                      Nanometres{o.at("diameter_nm").as_int64()}, Nanometres{o.at("drill_nm").as_int64()},
                      std::move(span), false});
  }
}

static GlobalRoutePath path(const value& v) {
  GlobalRoutePath p;
  const auto& o = v.as_object();
  p.net = NetId{s(o.at("net"))};
  p.connection_index = o.at("connection_index").as_int64();
  for (const auto& q : o.at("points").as_array()) {
    const auto& x = q.as_object();
    p.points.push_back({pt(q), LayerId{s(x.at("layer"))}});
  }
  return p;
}

int main(int argc, char** argv) {
  if (argc < 3) return 2;
  auto imported = io::import_kicad_pcb_file(argv[1]);
  if (!imported.ok()) return 3;
  std::ifstream in(argv[2]);
  value parsed = boost::json::parse(std::string{std::istreambuf_iterator<char>(in), {}});
  for (const auto& record : parsed.as_array()) {
    const auto& r = record.as_object();
    Board base = *imported.board;
    for (const auto& f : r.at("fixed").as_array()) add_fixed(base, f);
    std::vector<GlobalRoutePath> requests;
    for (const auto& p : r.at("paths").as_array()) requests.push_back(path(p));
    for (bool compact : {false, true}) {
      CopperScaffoldConfig cfg;
      cfg.maximum_victim_exchanges = 0;
      cfg.maximum_joint_passage_candidates = 0;
      cfg.maximum_displacement_attempts = 0;
      cfg.maximum_coupled_victims = 1;
      cfg.compact_search_expansions = compact ? (argc > 3 ? std::stoull(argv[3]) : 25000U) : 0U;
      const auto start = std::chrono::steady_clock::now();
      auto out = construct_copper_scaffold(base, requests, cfg);
      const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::steady_clock::now() - start).count();
      const auto drc = check_copper_rules(out.copper);
      std::cout << "request=" << r.at("request_index").as_int64()
                << " compact_search=" << compact
                << " requests=" << requests.size()
                << " satisfied=" << out.satisfied_indices.size()
                << " missing=" << out.missing_indices.size()
                << " complete=" << out.complete
                << " expansions=" << out.counters.expansions
                << " compact_expansions=" << out.counters.compact_search_expansions
                << " compact_successes=" << out.counters.compact_search_successes
                << " compact_fallbacks=" << out.counters.compact_search_fallbacks
                << " goal_rejections=" << out.counters.goal_rejections
                << " elapsed_ms=" << ms
                << " exact_drc_violations=" << drc.violations.size() << std::endl;
    }
  }
}
