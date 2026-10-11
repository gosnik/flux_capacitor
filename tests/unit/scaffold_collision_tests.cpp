#include "../../src/scaffold_collision.hpp"

#include <algorithm>
#include <cstdlib>
#include <iostream>
#include <set>
#include <stdexcept>

namespace {
using namespace flux;
Point mm(double x, double y) { return {Nanometres::from_mm(x), Nanometres::from_mm(y)}; }
auto signature(const CopperDrcReport& report) {
  std::set<std::pair<std::string, std::vector<std::string>>> result;
  for (auto violation : report.violations) {
    std::sort(violation.object_ids.begin(), violation.object_ids.end());
    result.emplace(violation.code, std::move(violation.object_ids));
  }
  return result;
}
}

int main() {
  using namespace flux;
  const LayerId front{"F.Cu"}, back{"B.Cu"};
  const NetId a{"a"}, b{"b"};
  Board board;
  board.outline = Polygon{{mm(0,0), mm(20,0), mm(20,20), mm(0,20)}};
  board.cutouts = {Polygon{{mm(14,14), mm(16,14), mm(16,16), mm(14,16)}}};
  board.layers = {{front,"F.Cu",LayerKind::copper,0}, {back,"B.Cu",LayerKind::copper,1}};
  board.nets = {{a,"a",0}, {b,"b",0}};
  board.differential_pairs = {{DifferentialPairId{"pair"}, "pair", a, b,
                               Nanometres::from_mm(.8), std::nullopt}};
  board.rules.hole_clearance = Nanometres::from_mm(.3);
  board.rules.hole_to_hole_clearance = Nanometres::from_mm(.4);
  board.tracks = {{ObjectId{"source-track"}, b, front, mm(3,2), mm(3,18), Nanometres::from_mm(.4), true}};
  board.track_arcs = {{ObjectId{"source-arc"}, b, front, mm(5,5), mm(7,3), mm(9,5), Nanometres::from_mm(.3), true}};
  board.vias = {{ObjectId{"source-via"}, b, mm(10,10), Nanometres::from_mm(.8), Nanometres::from_mm(.4), {front,back}, true}};
  Footprint footprint;
  footprint.id = ObjectId{"footprint"}; footprint.position = mm(12,8);
  Pad pad{ObjectId{"offset-hole"}, b, mm(0,0), {front}};
  pad.width = Nanometres::from_mm(2); pad.height = Nanometres::from_mm(1);
  pad.shape = PadShape::oval; pad.drill_width = Nanometres::from_mm(.5);
  pad.drill_height = Nanometres::from_mm(.7); pad.drill_offset = mm(.6,0);
  footprint.pads.push_back(pad);
  board.footprints.push_back(footprint);
  ScaffoldCollisionIndex index(board);
  if (!index.ready()) throw std::runtime_error("collision index must build");
  std::size_t queries = 0;
  for (double x = .1; x < 20; x += .73) for (double y = .1; y < 20; y += 1.17) {
    TrackSegment track{ObjectId{"query"},a,front,mm(x,y),mm(x+.61,y+.39),Nanometres::from_mm(.25),false};
    auto full = board; full.tracks.push_back(track);
    if (signature(index.check_track(track)) != signature(check_dynamic_copper_rules(full,{track.id})))
      throw std::runtime_error("indexed track narrow phase differs from full-board DRC");
    Via via{ObjectId{"query-via"},a,mm(x,y),Nanometres::from_mm(.6),Nanometres::from_mm(.3),{front,back},false};
    full = board; full.vias.push_back(via);
    if (signature(index.check_via(via)) != signature(check_dynamic_copper_rules(full,{via.id})))
      throw std::runtime_error("indexed via narrow phase differs from full-board DRC");
    queries += 2;
  }
  std::cout << "indexed scaffold collision parity: " << queries << " exact queries passed\n";
}
