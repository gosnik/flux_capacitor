// Diagnostic straight scaffold export; never a final curved routing artifact.
#include "flux/io/kicad_export.hpp"
#define main replay_embedded_main
#include "../exchange-evidence-round22-20261012/replay_joint_repair.cpp"
#undef main
int main(int argc, char** argv) {
  if (argc != 4) return 2;
  const auto imported = io::import_kicad_pcb_file(argv[1]);
  if (!imported.ok()) return 3;
  auto candidate = *imported.board;
  for (const auto& primitive : latest_generated_primitives(argv[2])) add_generated(candidate, primitive);
  const auto result = io::export_kicad_candidate_file(argv[1], argv[3], *imported.board, candidate);
  for (const auto& d : result.diagnostics) std::cerr << d.code << ": " << d.message << '\n';
  if (!result.ok()) return 4;
  const auto connectivity=analyze_connectivity(candidate);
  std::cout << "{\"provisional_straight_scaffold\":true,\"open_connections\":" << connectivity.open_connections
            << ",\"unrouted_nets\":" << connectivity.unrouted_nets << ",\"tracks\":" << candidate.tracks.size()
            << ",\"vias\":" << candidate.vias.size() << "}\n";
}
