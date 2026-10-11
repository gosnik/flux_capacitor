// Read-only canonical pad geometry for differential comparison with KiCad's own loader.
#include "flux/core/copper_geometry.hpp"
#include "flux/io/kicad_import.hpp"
#include <boost/json.hpp>
#include <iostream>
#include <map>

int main(int argc, char** argv) {
  if (argc != 2) {
    std::cerr << "usage: flux-inspect-pad-geometry BOARD.kicad_pcb\n";
    return 1;
  }
  const auto imported = flux::io::import_kicad_pcb_file(argv[1]);
  if (!imported.strict()) {
    std::cerr << "source must import strictly\n";
    return 1;
  }
  std::map<flux::ObjectId, std::string> references;
  for (const auto& footprint : imported.board->footprints)
    for (const auto& pad : footprint.pads) references.emplace(pad.id, footprint.reference);
  boost::json::array pads;
  std::map<flux::ObjectId, flux::Point> drill_positions;
  for (const auto& footprint : imported.board->footprints)
    for (const auto& pad : footprint.pads)
      if (flux::pad_has_drill(pad))
        drill_positions.emplace(pad.id, flux::pad_drill_world_position(footprint, pad));
  for (const auto& pad : flux::copper_pad_shapes(*imported.board)) {
    boost::json::object row;
    row["id"] = pad.id.value();
    row["reference"] = references.at(pad.id);
    row["net"] = pad.net.value();
    row["x_nm"] = pad.position.x.value();
    row["y_nm"] = pad.position.y.value();
    row["clockwise_microdegrees"] = -pad.rotation.value();
    row["width_nm"] = pad.width.value();
    row["height_nm"] = pad.height.value();
    if (const auto drill = drill_positions.find(pad.id); drill != drill_positions.end()) {
      row["drill_x_nm"] = drill->second.x.value();
      row["drill_y_nm"] = drill->second.y.value();
    }
    pads.push_back(std::move(row));
  }
  boost::json::object result;
  result["source_revision"] = imported.board->source_revision;
  result["pads"] = std::move(pads);
  std::cout << result << '\n';
}
