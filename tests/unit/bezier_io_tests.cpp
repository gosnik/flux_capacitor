#include "flux/io/kicad_export.hpp"
#include "flux/io/kicad_import.hpp"

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iterator>
#include <string>
#include <string_view>

namespace {

void require(bool value, const char* message) {
  if (!value) {
    std::cerr << message << '\n';
    std::exit(1);
  }
}

[[nodiscard]] std::string minimal_fixture() {
  const auto path = std::filesystem::path{FLUX_TEST_SOURCE_DIR} / "fixtures/minimal.kicad_pcb";
  std::ifstream input(path, std::ios::binary);
  require(static_cast<bool>(input), "minimal KiCad fixture must be readable");
  return {std::istreambuf_iterator<char>{input}, std::istreambuf_iterator<char>{}};
}

void insert_before_board_close(std::string& contents, std::string_view text) {
  const auto close = contents.rfind(')');
  require(close != std::string::npos, "fixture must have a board close delimiter");
  contents.insert(close, std::string(text));
}

void replace_all(std::string& contents, std::string_view from, std::string_view to) {
  auto position = contents.find(from);
  while (position != std::string::npos) {
    contents.replace(position, from.size(), to);
    position = contents.find(from, position + to.size());
  }
}

void exercise_curve_io(bool named_only_source) {
  auto source = minimal_fixture();
  insert_before_board_close(source, R"(
  (gr_curve
    (pts (xy 10 10) (xy 12 8) (xy 18 8) (xy 20 10))
    (stroke (width 0.15) (type default))
    (layer "F.SilkS")
  )
  (gr_curve
    (pts (xy 20 30) (xy 25 20) (xy 35 40) (xy 40 30))
    (stroke (width 0.25) (type default))
    (layer "F.Cu")
    (locked)
    (net "SIGNAL")
  )
  (gr_curve
    (pts (xy 42 30) (xy 44 20) (xy 50 40) (xy 52 30))
    (stroke (width 0.25) (type default))
    (layer "F.Cu")
    (net "SIGNAL")
    (uuid "c0000000-0000-4000-8000-000000000001")
  )
)");
  if (named_only_source) {
    const auto no_net = source.find("  (net 0 \"\")\n");
    const auto signal_net = source.find("  (net 1 \"SIGNAL\")\n");
    require(no_net != std::string::npos && signal_net != std::string::npos,
            "fixture must retain ordinal net declarations");
    source.erase(no_net, std::string{"  (net 0 \"\")\n"}.size());
    const auto signal_after_no_net = source.find("  (net 1 \"SIGNAL\")\n");
    source.erase(signal_after_no_net, std::string{"  (net 1 \"SIGNAL\")\n"}.size());
    replace_all(source, "(net 1)", "(net \"SIGNAL\")");
    replace_all(source, "(net 0)", "(net \"\")");
  }

  const auto imported = flux::io::import_kicad_pcb(source);
  require(imported.strict() && imported.board && imported.board->track_beziers.size() == 2U,
          "netted KiCad cubic must import without absorbing unrelated graphics");
  const auto& original = imported.board->track_beziers.front();
  require(original.locked && original.width == flux::Nanometres{250'000} &&
              original.layer == flux::LayerId{"F.Cu"} &&
              original.control1 == flux::Point{flux::Nanometres{25'000'000},
                                                flux::Nanometres{20'000'000}},
          "curve import must retain lock, layer, width, and exact controls");

  auto candidate = *imported.board;
  candidate.track_beziers[1].control2 = {flux::Nanometres{51'000'000}, flux::Nanometres{41'000'000}};
  candidate.track_beziers[1].width = flux::Nanometres{300'000};
  candidate.track_beziers.push_back(
      {flux::ObjectId{"curve-added"}, original.net, flux::LayerId{"B.Cu"},
       {flux::Nanometres{45'000'000}, flux::Nanometres{30'000'000}},
       {flux::Nanometres{47'000'000}, flux::Nanometres{20'000'000}},
       {flux::Nanometres{53'000'000}, flux::Nanometres{40'000'000}},
       {flux::Nanometres{55'000'000}, flux::Nanometres{30'000'000}}, flux::Nanometres{250'000},
       false});
  const auto exported = flux::io::export_kicad_candidate(source, *imported.board, candidate);
  require(exported.ok() && exported.contents && exported.contents->find("(net \"SIGNAL\")") != std::string::npos &&
              exported.contents->find("(layer \"F.SilkS\")") != std::string::npos,
          "curve replacement and insertion must retain named electrical net syntax and graphics");
  const auto reimported = flux::io::import_kicad_pcb(*exported.contents);
  require(reimported.strict() && reimported.board && reimported.board->track_beziers.size() == 3U &&
              reimported.board->track_beziers[1].width == flux::Nanometres{300'000},
          "exported electrical curves must strictly re-import with replaced geometry");

  auto locked_removed = candidate;
  locked_removed.track_beziers.erase(locked_removed.track_beziers.begin());
  require(!flux::io::export_kicad_candidate(source, *imported.board, locked_removed).ok(),
          "locked source curve removal must be rejected");
  candidate.track_beziers.erase(candidate.track_beziers.begin() + 1);
  const auto removed = flux::io::export_kicad_candidate(source, *imported.board, candidate);
  const auto removal_reimport = removed.contents ? flux::io::import_kicad_pcb(*removed.contents)
                                                 : flux::io::KicadImportResult{};
  require(removed.ok() && removal_reimport.strict() && removal_reimport.board &&
              removal_reimport.board->track_beziers.size() == 2U,
          "source curve removal must preserve added electrical curve through strict re-import");
}

} // namespace

int main() {
  exercise_curve_io(false);
  exercise_curve_io(true);
  return 0;
}
