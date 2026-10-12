// Offline reservation experiment.  Reuses the Round22 saved-copper decoder, never the router.
#include "flux/core/copper_geometry.hpp"

// Keep the two diagnostics in one translation unit so their canonical identity and telemetry
// loaders cannot drift.  Rename its executable entry point only.
#define main round22_embedded_main
#include "../exchange-evidence-round22-20261012/replay_joint_repair.cpp"
#undef main

namespace {

struct ReservedTrack { TrackSegment track; std::string original_id; };

bool endpoint_touches_matching_pad(const TrackSegment& track, const CopperPadShape& pad) {
  if (track.net != pad.net || track.layer != pad.layers.front()) return false;
  const auto half_width = static_cast<long double>(track.width.value()) / 2.0L;
  return point_to_pad_copper_distance_nm(track.start, pad) <= half_width ||
         point_to_pad_copper_distance_nm(track.end, pad) <= half_width;
}

bool supported_pad_shape(PadShape shape) {
  return shape == PadShape::circle || shape == PadShape::rectangle || shape == PadShape::oval ||
         shape == PadShape::round_rectangle;
}

std::vector<ReservedTrack> select_terminal_tracks(const Board& saved,
                                                  const std::vector<std::size_t>& victims) {
  std::set<std::size_t> wanted(victims.begin(), victims.end());
  std::vector<const TrackSegment*> candidates;
  for (const auto& track : saved.tracks) {
    const auto prefix = std::string{"__flux-copper-scaffold-"};
    if (!track.id.value().starts_with(prefix)) continue;
    const auto remainder = track.id.value().substr(prefix.size());
    const auto dash = remainder.find('-');
    if (dash == std::string::npos) throw std::runtime_error("unsupported generated owner ID");
    const auto owner = static_cast<std::size_t>(std::stoull(remainder.substr(0U, dash)));
    if (wanted.contains(owner)) candidates.push_back(&track);
  }
  std::sort(candidates.begin(), candidates.end(), [](const auto* lhs, const auto* rhs) {
    return lhs->id.value() < rhs->id.value();
  });
  std::vector<ReservedTrack> selected;
  std::set<std::string> selected_ids;
  for (const auto& pad : copper_pad_shapes(saved)) {
    std::size_t selected_for_pad = 0U;
    for (const auto* track : candidates) {
      if (selected.size() == 8U || selected_for_pad == 2U) break;
      if (selected_ids.contains(track->id.value())) continue;
      if (std::find(pad.layers.begin(), pad.layers.end(), track->layer) == pad.layers.end()) continue;
      if (track->net == pad.net && std::find(pad.layers.begin(), pad.layers.end(), track->layer) != pad.layers.end() &&
          !supported_pad_shape(pad.shape))
        throw std::runtime_error("unsupported pad geometry at candidate reservation terminal");
      if (!endpoint_touches_matching_pad(*track, pad)) continue;
      selected_ids.insert(track->id.value());
      selected.push_back({*track, track->id.value()});
      ++selected_for_pad;
    }
    if (selected.size() == 8U) break;
  }
  return selected;
}

void add_reservations(Board& board, const std::vector<ReservedTrack>& reservations) {
  for (std::size_t index = 0; index < reservations.size(); ++index) {
    auto copy = reservations[index].track;
    copy.id = ObjectId{"__round23-reservation-" + std::to_string(index)};
    board.tracks.push_back(std::move(copy));
  }
}

bool erase_reservations(Board& board) {
  const auto before = board.tracks.size();
  std::erase_if(board.tracks, [](const auto& track) {
    return track.id.value().starts_with("__round23-reservation-");
  });
  return board.tracks.size() != before;
}

bool has_reservation(const Board& board) {
  return std::any_of(board.tracks.begin(), board.tracks.end(), [](const auto& track) {
    return track.id.value().starts_with("__round23-reservation-");
  });
}

CopperScaffoldConfig bounded_config() {
  CopperScaffoldConfig config;
  config.maximum_expansions_per_connection = 50'000U;
  config.compact_search_expansions = 25'000U;
  config.maximum_victim_exchanges = 0U;
  config.maximum_joint_passage_candidates = 0U;
  config.maximum_displacement_attempts = 0U;
  config.maximum_coupled_victims = 1U;
  return config;
}

void json_tracks(std::ostream& out, const std::vector<ReservedTrack>& tracks) {
  out << '[';
  for (std::size_t i = 0; i < tracks.size(); ++i) {
    if (i) out << ','; const auto& track = tracks[i].track;
    out << "{\"original_id\":"; json_string(out, tracks[i].original_id);
    out << ",\"net\":"; json_string(out, track.net.value());
    out << ",\"layer\":"; json_string(out, track.layer.value());
    out << ",\"start\":{\"x_nm\":" << track.start.x.value() << ",\"y_nm\":" << track.start.y.value() << '}';
    out << ",\"end\":{\"x_nm\":" << track.end.x.value() << ",\"y_nm\":" << track.end.y.value() << "}}";
  }
  out << ']';
}

void json_result(std::ostream& out, const CopperScaffoldResult& result) {
  out << "{\"complete\":" << (result.complete ? "true" : "false") << ",\"satisfied\":";
  json_indices(out, result.satisfied_indices); out << ",\"missing\":"; json_indices(out, result.missing_indices);
  out << ",\"requests\":[";
  for (std::size_t i = 0; i < result.request_evidence.size(); ++i) {
    if (i) out << ','; const auto& evidence = result.request_evidence[i];
    out << "{\"termination\":"; json_string(out, std::string(to_string(evidence.termination)));
    out << ",\"blockers\":"; json_blockers(out, evidence);
    out << ",\"attachments\":"; json_attachments(out, evidence); out << '}';
  }
  out << "]}";
}

}  // namespace

int main(int argc, char** argv) {
  if (argc != 4) return 2;
  try {
    const auto imported = io::import_kicad_pcb_file(argv[1]);
    if (!imported.ok()) throw std::runtime_error("cannot import PCB");
    const auto requests = target_requests(read_global_result(argv[2]));
    const auto copper = latest_generated_primitives(argv[3]);
    Board saved = *imported.board; for (const auto& primitive : copper) add_generated(saved, primitive);
    const auto baseline_drc = check_copper_rules(saved);
    const std::vector<Scenario> scenarios{
        {"incoming-91-victims-43", 91U, {43U}}, {"incoming-44-victims-43-45", 44U, {43U, 45U}},
        {"incoming-44-victims-45-43", 44U, {45U, 43U}}, {"incoming-44-victims-11-19", 44U, {11U, 19U}},
        {"incoming-44-victims-19-11", 44U, {19U, 11U}}, {"incoming-91-victims-11-19", 91U, {11U, 19U}},
        {"incoming-91-victims-19-11", 91U, {19U, 11U}},
    };
    std::cout << "{\"outcomes\":[";
    for (std::size_t ordinal = 0; ordinal < scenarios.size(); ++ordinal) {
      if (ordinal) std::cout << ','; const auto& scenario = scenarios[ordinal];
      Board ripped = saved; for (const auto victim : scenario.victims) erase_owner(ripped, victim);
      // Same-run no-reservation control reproduces the Round22 ordering with identical bounds.
      const auto control_incoming = construct_copper_scaffold(ripped, {requests.at(scenario.incoming)}, bounded_config());
      std::vector<GlobalRoutePath> victims; for (const auto victim : scenario.victims) victims.push_back(requests.at(victim));
      const auto control_victims = construct_copper_scaffold(control_incoming.copper, victims, bounded_config());
      const auto reservations = select_terminal_tracks(saved, scenario.victims);
      Board incoming_scene = ripped; add_reservations(incoming_scene, reservations);
      const auto incoming_started = std::chrono::steady_clock::now();
      const auto incoming = construct_copper_scaffold(incoming_scene, {requests.at(scenario.incoming)}, bounded_config());
      const auto incoming_ms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - incoming_started).count();
      Board victim_scene = incoming.copper;
      const auto reservations_removed = erase_reservations(victim_scene);
      const auto victim_started = std::chrono::steady_clock::now();
      const auto rebuilt = construct_copper_scaffold(victim_scene, victims, bounded_config());
      const auto victim_ms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - victim_started).count();
      const auto drc = check_copper_rules(rebuilt.copper); const auto connectivity = analyze_connectivity(rebuilt.copper);
      const auto new_drc = new_drc_violations(baseline_drc, drc);
      std::cout << "{\"scenario\":"; json_string(std::cout, scenario.name);
      std::cout << ",\"incoming_request\":" << scenario.incoming << ",\"victim_request_order\":"; json_indices(std::cout, scenario.victims);
      std::cout << ",\"selected_original_tracks\":"; json_tracks(std::cout, reservations);
      std::cout << ",\"no_reservation_control\":{\"incoming_result\":"; json_result(std::cout, control_incoming);
      std::cout << ",\"victim_result\":"; json_result(std::cout, control_victims); std::cout << '}';
      std::cout << ",\"incoming_runtime_ms\":" << incoming_ms << ",\"victim_runtime_ms\":" << victim_ms;
      std::cout << ",\"reservations_removed\":" << (reservations_removed ? "true" : "false")
                << ",\"reservation_leakage\":" << (has_reservation(rebuilt.copper) ? "true" : "false")
                << ",\"incoming_result\":"; json_result(std::cout, incoming);
      std::cout << ",\"victim_result\":"; json_result(std::cout, rebuilt);
      std::cout << ",\"new_exact_drc_violations\":"; json_drc_violations(std::cout, new_drc);
      std::cout << ",\"whole_board_open_connections\":" << connectivity.open_connections
                << ",\"whole_board_unrouted_nets\":" << connectivity.unrouted_nets << '}';
    }
    std::cout << "]}\n";
  } catch (const std::exception& error) {
    std::cout << "{\"error\":"; json_string(std::cout, error.what()); std::cout << "}\n"; return 1;
  }
}
