#include "flux/core/bezier_copper.hpp"
#include "flux/core/drc.hpp"
#include "flux/core/connectivity.hpp"
#include <algorithm>
#include <cmath>

#include <cstdlib>
#include <iostream>

namespace {
using namespace flux;

[[nodiscard]] Point point(std::int64_t x, std::int64_t y) {
  return {Nanometres{x}, Nanometres{y}};
}

void require(bool value, const char* message) {
  if (!value) {
    std::cerr << message << '\n';
    std::exit(1);
  }
}
}

int main() {
  const TrackBezier curve{ObjectId{"curve"}, NetId{"net"}, LayerId{"F.Cu"}, point(0, 0),
                          point(0, 1'000'000), point(1'000'000, 1'000'000), point(1'000'000, 0),
                          Nanometres{100'000}, false};
  const auto tessellation = tessellate_track_bezier(curve, Nanometres{100});
  require(tessellation.certified && !tessellation.segments.empty(),
          "valid cubic must have a certified adaptive tessellation");
  require(tessellation.segments.front().start == curve.start &&
              tessellation.segments.back().end == curve.end,
          "tessellation must retain exact cubic endpoints");
  for (const auto& segment : tessellation.segments) {
    require(segment.centerline_deviation_nm.value() <= 100,
            "every accepted chord must satisfy the requested deviation bound");
  }

  Board board;
  board.tracks.push_back({ObjectId{"__flux_bezier_proxy/curve/0"}, NetId{"net"}, LayerId{"F.Cu"},
                          point(2'000'000, 0), point(3'000'000, 0), Nanometres{100'000}, false});
  board.track_beziers.push_back(curve);
  const auto flattened = flatten_board_beziers_conservatively(board, Nanometres{100});
  require(flattened.certified && flattened.board.track_beziers.empty(),
          "obstacle view must contain only conservative straight copper");
  require(!flattened.proxy_sources.empty(), "every curve proxy must retain source identity");
  for (const auto& [proxy, source] : flattened.proxy_sources) {
    require(proxy != "__flux_bezier_proxy/curve/0" && source == curve.id,
            "generated proxy IDs must be unique and map to the canonical curve");
  }
  Board drc_board;
  drc_board.outline.vertices = {point(-5'000'000, -5'000'000), point(5'000'000, -5'000'000),
                                point(5'000'000, 5'000'000), point(-5'000'000, 5'000'000)};
  drc_board.layers.push_back({LayerId{"F.Cu"}, "F.Cu", LayerKind::copper, 0});
  drc_board.nets.push_back({NetId{"net"}, "net"});
  drc_board.nets.push_back({NetId{"other"}, "other"});
  drc_board.track_beziers.push_back(curve);
  drc_board.rules.minimum_track_width = Nanometres{200'000};
  const auto thin = check_copper_rules(drc_board);
  require(std::any_of(thin.violations.begin(), thin.violations.end(), [](const auto& violation) {
    return violation.code == "track_bezier_width";
  }), "graphic copper must obey Flux minimum trace width even when KiCad does not enforce it");
  drc_board.rules.minimum_track_width = Nanometres{0};
  drc_board.tracks.push_back({ObjectId{"crossing"}, NetId{"other"}, LayerId{"F.Cu"},
      point(500'000, -1'000'000), point(500'000, 2'000'000), Nanometres{100'000}, false});
  const auto collision = check_dynamic_copper_rules(drc_board, {curve.id});
  require(std::any_of(collision.violations.begin(), collision.violations.end(), [&](const auto& violation) {
    return std::find(violation.object_ids.begin(), violation.object_ids.end(), curve.id.value()) !=
           violation.object_ids.end() &&
           std::find(violation.object_ids.begin(), violation.object_ids.end(), "crossing") !=
           violation.object_ids.end();
  }), "dynamic curve collisions must retain the canonical curve identity");
  drc_board.tracks.back().start = point(3'000'000, -1'000'000);
  drc_board.tracks.back().end = point(3'000'000, 2'000'000);
  require(check_dynamic_copper_rules(drc_board, {curve.id}).ok(),
          "separated curve and foreign copper must pass dynamic DRC");
  require(tessellation.length_lower_nm <= 2'000'000.0L &&
              tessellation.length_upper_nm >= 2'000'000.0L,
          "length interval must enclose this cubic's analytic two-millimetre arc length");
  const auto coarse = tessellate_track_bezier(curve);
  long double chords = 0.0L;
  for (const auto& segment : coarse.segments)
    chords += std::hypotl(static_cast<long double>(segment.end.x.value()) - segment.start.x.value(),
                         static_cast<long double>(segment.end.y.value()) - segment.start.y.value());
  drc_board.tracks.back().start = point(2'000'000, -2'000'000);
  drc_board.tracks.back().end = point(2'000'000 + static_cast<std::int64_t>(chords), -2'000'000);
  drc_board.differential_pairs.push_back({DifferentialPairId{"pair"}, "pair", NetId{"net"},
                                         NetId{"other"}, std::nullopt, Nanometres{10}});
  const auto skew = check_copper_rules(drc_board);
  require(std::any_of(skew.violations.begin(), skew.violations.end(), [](const auto& violation) {
    return violation.kind == CopperViolationKind::differential_pair_skew;
  }), "equal proxy chord lengths must not falsely certify true curved length skew");
  Board connected;
  connected.layers = drc_board.layers;
  connected.nets.push_back({NetId{"net"}, "net"});
  connected.track_beziers.push_back(curve);
  Footprint footprint;
  footprint.id = ObjectId{"terminals"};
  for (const auto endpoint : {curve.start, curve.end}) {
    Pad pad;
    pad.id = ObjectId{footprint.pads.empty() ? "start-pad" : "end-pad"};
    pad.net = curve.net; pad.layers = {curve.layer}; pad.local_position = endpoint;
    pad.width = pad.height = Nanometres{100'000};
    footprint.pads.push_back(pad);
  }
  connected.footprints.push_back(footprint);
  require(analyze_connectivity(connected).open_connections == 0,
          "cubic endpoint caps and contracted interior must connect both terminals");
  connected.track_beziers.clear();
  require(analyze_connectivity(connected).open_connections == 1,
          "removing the cubic must expose the disconnected terminal pair");
  require(!tessellate_track_bezier(curve, Nanometres{0}, 1).certified,
          "exhausted subdivision must never claim certified copper");
  return 0;
}
