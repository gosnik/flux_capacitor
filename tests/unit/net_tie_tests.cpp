#include "flux/core/copper_geometry.hpp"
#include "flux/core/drc.hpp"
#include "flux/core/validation.hpp"
#include "flux/io/kicad_import.hpp"

#include <algorithm>
#include <cstdlib>
#include <iostream>
#include <stdexcept>

namespace {

using namespace flux;

[[nodiscard]] Point point(std::int64_t x, std::int64_t y) {
  return {Nanometres{x}, Nanometres{y}};
}

void require(bool condition, const char* message) {
  if (!condition) throw std::runtime_error(message);
}

Board net_tie_board() {
  Board board;
  board.outline.vertices = {point(-5'000'000, -5'000'000), point(5'000'000, -5'000'000),
                            point(5'000'000, 5'000'000), point(-5'000'000, 5'000'000)};
  board.layers.push_back({LayerId{"F.Cu"}, "F.Cu", LayerKind::copper, 0});
  board.nets.push_back({NetId{"sense"}, "sense"});
  board.nets.push_back({NetId{"power"}, "power"});
  board.nets.push_back({NetId{"other"}, "other"});
  board.rules.default_clearance = Nanometres{150'000};
  board.rules.default_track_width = Nanometres{200'000};
  Footprint footprint;
  footprint.id = ObjectId{"tie"};
  footprint.position = point(0, 0);
  footprint.placement_envelope_source = PlacementEnvelopeSource::absent;
  Pad sense;
  sense.id = ObjectId{"sense-pad"};
  sense.number = "3";
  sense.net = NetId{"sense"};
  sense.layers = {LayerId{"F.Cu"}};
  sense.shape = PadShape::rectangle;
  sense.width = Nanometres{300'000};
  sense.height = Nanometres{400'000};
  Pad power = sense;
  power.id = ObjectId{"power-pad"};
  power.number = "1";
  power.net = NetId{"power"};
  power.width = Nanometres{3'100'000};
  power.height = Nanometres{4'000'000};
  power.local_position = point(-1'400'000, -2'050'000);
  footprint.pads = {sense, power};
  footprint.net_tie_pad_groups = {{sense.id, power.id}};
  board.footprints.push_back(footprint);
  return board;
}

void declared_group_allows_only_local_terminal_contact() {
  auto board = net_tie_board();
  board.tracks.push_back({ObjectId{"escape"}, NetId{"sense"}, LayerId{"F.Cu"},
                          point(0, 0), point(0, 1'500'000), Nanometres{200'000}});
  const auto pads = copper_pad_shapes(board);
  const auto foreign = std::find_if(pads.begin(), pads.end(), [](const CopperPadShape& pad) {
    return pad.id == ObjectId{"power-pad"};
  });
  require(foreign != pads.end() &&
              declared_net_tie_terminal_escape_exempt(
                  pads, NetId{"sense"}, LayerId{"F.Cu"}, point(0, 0), point(0, 1'500'000),
                  Nanometres{200'000}, *foreign, Nanometres{150'000}),
          "the conservative physical-contact enclosure must accept the Kelvin escape");
  require(check_copper_rules(board).ok(),
          "declared net-tie terminal escape inside the own pad must be legal");
  require(check_dynamic_copper_rules(board, {ObjectId{"escape"}}).ok(),
          "the incremental DRC must apply the same localized terminal exception");

  auto crossing = board;
  crossing.tracks.front().start = point(0, 1'500'000);
  crossing.tracks.front().end = point(0, -1'500'000);
  require(!check_copper_rules(crossing).ok(),
          "a track crossing a foreign net-tie pad outside its own pad must remain illegal");

  auto nonmember = board;
  nonmember.footprints.front().net_tie_pad_groups.clear();
  require(!check_copper_rules(nonmember).ok(),
          "the same geometry without a declared group must remain illegal");

  auto wide = board;
  wide.tracks.front().width = Nanometres{1'400'000};
  require(!check_copper_rules(wide).ok(),
          "a wide trace whose physical contact enclosure leaves the own pad must be rejected");

  auto one_nm_outside = board;
  one_nm_outside.tracks.front().width = Nanometres{300'002};
  require(!check_copper_rules(one_nm_outside).ok(),
          "a one-nanometre physical contact enclosure overrun must remain illegal");

  auto rotated_native = board;
  auto& rotated_power = rotated_native.footprints.front().pads[1];
  rotated_power.shape = PadShape::round_rectangle;
  rotated_power.corner_radius = Nanometres{250'000};
  rotated_power.rotation = Microdegrees::from_degrees(45.0);
  rotated_power.local_position = point(0, 0);
  require(!check_copper_rules(rotated_native).ok(),
          "a rotated native pad contact whose enclosure leaves the own pad must be rejected");

  auto rotated_one_nm_outside = rotated_native;
  rotated_one_nm_outside.tracks.front().width = Nanometres{300'002};
  require(!check_copper_rules(rotated_one_nm_outside).ok(),
          "a rotated native contact enclosure one nanometre beyond the own pad must be rejected");
}

void declared_groups_remain_distinct_and_local() {
  auto board = net_tie_board();
  require(validate_board(board).ok(), "resolved net-tie metadata must validate");
  const auto pads = copper_pad_shapes(board);
  require(pads.size() == 2U && declared_net_tie_pad_pair_exempt(pads[0], pads[1]),
          "declared peers on the same footprint must be recognized");
  auto separate = board;
  separate.footprints.front().id = ObjectId{"other-footprint"};
  const auto separate_pads = copper_pad_shapes(separate);
  require(declared_net_tie_pad_pair_exempt(separate_pads[0], separate_pads[1]),
          "footprint metadata is retained when a footprint is relocated");
  require(board.nets[0].id != board.nets[1].id,
          "a net-tie declaration must not merge distinct nets");

  auto overlapping_groups = board;
  overlapping_groups.footprints.front().net_tie_pad_groups.push_back(
      overlapping_groups.footprints.front().net_tie_pad_groups.front());
  require(!validate_board(overlapping_groups).ok(),
          "a pad cannot be declared in two net-tie groups");
}

} // namespace

void imported_groups_resolve_duplicate_pad_numbers() {
  const std::string text = R"pcb((kicad_pcb
    (version 20241229) (generator "flux_capacitor_test")
    (layers (0 "F.Cu" signal) (2 "B.Cu" signal) (25 "Edge.Cuts" user))
    (net 0 "") (net 1 "A") (net 2 "B")
    (footprint "Tie" (layer "F.Cu") (uuid "11111111-1111-4111-8111-111111111111")
      (at 20 20) (net_tie_pad_groups "1,2")
      (pad "1" smd rect (at 0 0) (size 1 1) (layers "F.Cu") (net 1 "A")
        (uuid "22222222-2222-4222-8222-222222222222"))
      (pad "2" smd rect (at 1 0) (size 1 1) (layers "F.Cu") (net 2 "B")
        (uuid "33333333-3333-4333-8333-333333333333"))
      (pad "2" smd rect (at 3 0) (size 1 1) (layers "F.Cu") (net 2 "B")
        (uuid "44444444-4444-4444-8444-444444444444")))
    (gr_rect (start 0 0) (end 40 40) (stroke (width 0.05) (type default))
      (fill none) (layer "Edge.Cuts")
      (uuid "55555555-5555-4555-8555-555555555555"))))pcb";
  const auto imported = flux::io::import_kicad_pcb(text);
  require(imported.strict() && imported.board &&
              imported.board->footprints.front().net_tie_pad_groups.size() == 1U &&
              imported.board->footprints.front().net_tie_pad_groups.front().size() == 3U &&
              validate_board(*imported.board).ok(),
          "a declared number must resolve every pad object with that number");
  auto invalid = text;
  invalid.replace(invalid.find("1,2"), 3U, "1,unknown");
  require(!flux::io::import_kicad_pcb(invalid).strict(),
          "unknown declared pad numbers must fail strict import");
}

int main() {
  declared_group_allows_only_local_terminal_contact();
  declared_groups_remain_distinct_and_local();
  imported_groups_resolve_duplicate_pad_numbers();
  std::cout << "net-tie terminal contact checks passed\n";
  return EXIT_SUCCESS;
}
