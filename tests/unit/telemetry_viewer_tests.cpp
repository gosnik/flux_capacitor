#include "flux/core/router_telemetry.hpp"
#include "flux/telemetry/event.hpp"
#include "flux/telemetry/state.hpp"
#include "flux/telemetry/trace.hpp"
#include "viewer_preferences.hpp"

#include <chrono>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

void require(bool condition, const std::string& message) {
  if (!condition) {
    throw std::runtime_error(message);
  }
}

void producer_records_decode_into_viewer_state() {
  flux::RouterTelemetryEvent started;
  started.kind = flux::RouterTelemetryEventKind::routing_started;
  started.sequence = 1;
  started.total_nets = 12;
  started.detail = "board-revision-a";

  flux::RouterTelemetryEvent iteration;
  iteration.kind = flux::RouterTelemetryEventKind::pathfinder_iteration_completed;
  iteration.sequence = 2;
  iteration.iteration = 4;
  iteration.total_nets = 12;
  iteration.routed_nets = 10;
  iteration.unrouted_nets = 2;
  iteration.sampled_overflow = 3;
  iteration.corridor_overflow = 1;
  iteration.physical_overflow_nm = 220'000;
  iteration.clearance_shadow_conflicts = 2;
  iteration.ripup_count = 5;
  iteration.route_cost = 991;
  iteration.search_expansions = 1800;

  flux::RouterTelemetryEvent via;
  via.kind = flux::RouterTelemetryEventKind::via_assigned;
  via.sequence = 3;
  via.stable_id = "regional-via:7";
  via.net = flux::NetId{"SIGNAL"};
  via.layer = flux::LayerId{"F.Cu"};
  via.last_layer = flux::LayerId{"B.Cu"};
  via.x_nm = 4'000'000;
  via.y_nm = 2'000'000;
  via.diameter_nm = 600'000;
  via.score = -1.5;
  via.competing_nets = 2U;
  via.remaining_alternatives = 3U;

  flux::RouterTelemetryEvent conflict;
  conflict.kind = flux::RouterTelemetryEventKind::conflict_added;
  conflict.sequence = 4;
  conflict.stable_id = "regional-conflict:2";
  conflict.resource_id = "regional-edge:2";
  conflict.group_id = "regional-group:0";
  conflict.net = flux::NetId{"SIGNAL"};
  conflict.other_net = flux::NetId{"CLOCK"};
  conflict.layer = flux::LayerId{"F.Cu"};
  conflict.x_nm = 2'500'000;
  conflict.y_nm = 2'000'000;
  conflict.conflict_kind = "clearance_shadow";
  conflict.clearance_deficit_nm = 90'000;

  flux::RouterTelemetryEvent spline;
  spline.kind = flux::RouterTelemetryEventKind::route_candidate_selected;
  spline.sequence = 5;
  spline.stable_id = "regional-spline:7:SIGNAL:0:0:0";
  spline.net = flux::NetId{"SIGNAL"};
  spline.route_width_nm = 180'000;
  spline.route_geometry = flux::RouterTelemetryRouteGeometry::quintic_ph;
  spline.spline_degree = 5U;
  spline.topology_signature = 0x1a2b3c4dU;
  spline.topology_gate_count = 3U;
  spline.topology_gate_sequence_legal = true;
  spline.route_family = "rrt_connect";
  spline.route_lifecycle = "retained_fallback";
  spline.route_points = {{1'000'000, 1'000'000, flux::LayerId{"F.Cu"}},
                         {2'000'000, 1'500'000, flux::LayerId{"F.Cu"}},
                         {3'000'000, 1'000'000, flux::LayerId{"F.Cu"}}};
  spline.spline_control_points = {
      {1'000'000, 1'000'000, flux::LayerId{"F.Cu"}}, {1'500'000, 2'000'000, flux::LayerId{"F.Cu"}},
      {1'900'000, 2'200'000, flux::LayerId{"F.Cu"}}, {2'100'000, 2'200'000, flux::LayerId{"F.Cu"}},
      {2'500'000, 2'000'000, flux::LayerId{"F.Cu"}}, {3'000'000, 1'000'000, flux::LayerId{"F.Cu"}}};

  flux::RouterTelemetryEvent hydraulic;
  hydraulic.kind = flux::RouterTelemetryEventKind::hydraulic_field_updated;
  hydraulic.sequence = 6;
  hydraulic.stable_id = "hydraulic-field:SIGNAL:F.Cu";
  hydraulic.net = flux::NetId{"SIGNAL"};
  hydraulic.layer = flux::LayerId{"F.Cu"};
  hydraulic.x_nm = 500'000;
  hydraulic.y_nm = 750'000;
  hydraulic.hydraulic_field_pitch_nm = 250'000U;
  hydraulic.hydraulic_field_width = 2U;
  hydraulic.hydraulic_field_height = 2U;
  hydraulic.hydraulic_field_stride = 2U;
  hydraulic.hydraulic_maximum_flow = 512U;
  hydraulic.hydraulic_maximum_erosion = 64U;
  hydraulic.hydraulic_maximum_terrain_height = 20'000U;
  hydraulic.hydraulic_maximum_downstream_pressure = 19U;
  hydraulic.hydraulic_maximum_success_prior = 5U;
  hydraulic.hydraulic_flow_field = {0U, 4'096U, 32'768U, 65'535U};
  hydraulic.hydraulic_erosion_field = {0U, 8'192U, 40'000U, 65'535U};
  hydraulic.hydraulic_heightmap = {65'535U, 48'000U, 24'000U, 0U};
  hydraulic.hydraulic_free_space_field = {65'535U, 0U, 65'535U, 65'535U};
  hydraulic.hydraulic_downstream_pressure_field = {0U, 65'535U, 24'000U, 0U};
  hydraulic.hydraulic_success_prior_field = {0U, 0U, 32'768U, 65'535U};

  const std::string jsonl = flux::serialize_router_telemetry_event(started) + "\n" +
                            flux::serialize_router_telemetry_event(iteration) + "\n" +
                            flux::serialize_router_telemetry_event(via) + "\n" +
                            flux::serialize_router_telemetry_event(conflict) + "\n" +
                            flux::serialize_router_telemetry_event(spline) + "\n" +
                            flux::serialize_router_telemetry_event(hydraulic) + "\n";
  const flux::telemetry::DecodeResult decoded = flux::telemetry::decode_events(jsonl);
  require(static_cast<bool>(decoded), "the viewer must decode core producer JSONL records");
  require(decoded.events.size() == 6U, "all producer records must survive decoding");
  require(decoded.events[0].type == flux::telemetry::EventType::routing_started,
          "producer event kinds must map to viewer event kinds");
  require(decoded.events[1].metrics.has_value(),
          "Pathfinder iteration fields must become viewer metrics");
  require(decoded.events[1].metrics->corridor_overflow == 1U &&
              decoded.events[1].metrics->search_expansions == 1800U,
          "authoritative iteration values must not be recomputed by the viewer");
  require(decoded.events[2].vias.size() == 1U &&
              decoded.events[2].vias.front().state == flux::telemetry::ViaState::selected &&
              decoded.events[2].vias.front().to_layer == "B.Cu" &&
              decoded.events[3].conflicts.size() == 1U &&
              decoded.events[3].conflicts.front().nets ==
                  std::vector<std::string>{"SIGNAL", "CLOCK"} &&
              decoded.events[3].conflicts.front().clearance_deficit_nm == 90'000,
          "regional via and conflict producer records must retain physical diagnostic fields");
  require(
      decoded.events[4].routes.size() == 1U &&
          decoded.events[4].routes.front().geometry == flux::telemetry::RouteGeometry::quintic_ph &&
          decoded.events[4].routes.front().spline_degree == 5U &&
          decoded.events[4].routes.front().control_points.size() == 6U &&
          decoded.events[4].routes.front().topology_signature == 0x1a2b3c4dU &&
          decoded.events[4].routes.front().topology_gate_count == 3U &&
          decoded.events[4].routes.front().topology_gate_sequence_legal &&
          decoded.events[4].routes.front().family == "rrt_connect" &&
          decoded.events[4].routes.front().lifecycle == "retained_fallback",
      "regional quintic PH curve controls and their directed topology certificate must cross the "
      "producer/viewer telemetry boundary");
  require(
      decoded.events[5].type == flux::telemetry::EventType::hydraulic_field_updated &&
          decoded.events[5].hydraulic_fields.size() == 1U &&
          decoded.events[5].hydraulic_fields.front().layer == "F.Cu" &&
          decoded.events[5].hydraulic_fields.front().stride == 2U &&
          decoded.events[5].hydraulic_fields.front().erosion.back() == 65'535U &&
          decoded.events[5].hydraulic_fields.front().heightmap.front() == 65'535U &&
          decoded.events[5].hydraulic_fields.front().free_space[1] == 0U &&
          decoded.events[5].hydraulic_fields.front().downstream_pressure[1] == 65'535U &&
          decoded.events[5].hydraulic_fields.front().success_prior.back() == 65'535U,
      "bounded per-layer hydraulic height, obstacle, feedback, and erosion fields must cross the "
      "producer/viewer boundary");

  flux::telemetry::LiveState state;
  for (const auto& event : decoded.events) {
    state.apply(event);
  }
  require(state.iteration() == 4U && state.metrics()->unrouted_nets == 2U,
          "producer events must reduce into live state");
  require(state.source_identity() == "board-revision-a",
          "the routing-start source revision must remain observable");
  require(state.hydraulic_fields().size() == 1U,
          "the live viewer state must retain the latest field for each hydraulic layer proposal");

  flux::RouterTelemetryEvent global_finished;
  global_finished.kind = flux::RouterTelemetryEventKind::global_routing_finished;
  global_finished.detail = "global routing stage completed: checkpoint retained";
  const auto decoded_stage =
      flux::telemetry::decode_events(flux::serialize_router_telemetry_event(global_finished));
  require(decoded_stage &&
              decoded_stage.events.front().type == flux::telemetry::EventType::search_summary,
          "nested global routing completion must decode as a stage summary");
  state.apply(decoded_stage.events.front());
  require(state.routing_active(),
          "a completed global stage must not mark downstream regional/detail work finished");

  flux::telemetry::Event terminal_snapshot;
  terminal_snapshot.type = flux::telemetry::EventType::snapshot;
  terminal_snapshot.message = "Routing finished";
  state.apply(terminal_snapshot);
  require(!state.routing_active() && state.status() == "Routing finished",
          "only the terminal session snapshot may mark routing finished");
}

void rich_overlay_events_round_trip_and_replay() {
  flux::telemetry::Event snapshot;
  snapshot.type = flux::telemetry::EventType::snapshot;
  snapshot.sequence = 10;
  snapshot.monotonic_time_us = 1'000;
  snapshot.source_identity = "fixture-board";
  snapshot.source_state_included = true;
  auto source_board = std::make_shared<flux::telemetry::SourceBoard>();
  source_board->id = "source-board:0";
  source_board->path = "/tmp/fixture.kicad_pcb";
  source_board->revision = "fixture-board";
  source_board->board_schema = 18;
  source_board->layers.push_back({"F.Cu", "F.Cu", "copper", 0});
  source_board->nets.push_back({"SIGNAL", "SIGNAL", 2});
  source_board->primitives.push_back(
      {"board-outline",
       "board_outline",
       {},
       {},
       {},
       {},
       {{0, 0, {}}, {10'000'000, 0, {}}, {10'000'000, 8'000'000, {}}, {0, 8'000'000, {}}}});
  source_board->primitives.push_back({"pad-1",
                                      "pad",
                                      "footprint-1",
                                      "U1",
                                      "SIGNAL",
                                      {"F.Cu"},
                                      {{2'000'000, 3'000'000, "F.Cu"}},
                                      {},
                                      1'200'000,
                                      800'000,
                                      0,
                                      500'000,
                                      200'000,
                                      500'000,
                                      100'000,
                                      -50'000,
                                      90'000'000,
                                      "oval",
                                      "front",
                                      false,
                                      true});
  snapshot.source_boards.push_back(source_board);
  snapshot.routes.push_back(
      flux::telemetry::Route{"route-signal-0",
                             "SIGNAL",
                             3,
                             flux::telemetry::RouteState::committed,
                             180'000,
                             45,
                             {{1'000'000, 2'000'000, "F.Cu"}, {4'000'000, 2'000'000, "F.Cu"}}});
  snapshot.routes.back().geometry = flux::telemetry::RouteGeometry::quintic_ph;
  snapshot.routes.back().spline_degree = 5U;
  snapshot.routes.back().control_points = {
      {1'000'000, 2'000'000, "F.Cu"}, {2'000'000, 3'000'000, "F.Cu"},
      {2'400'000, 3'200'000, "F.Cu"}, {2'600'000, 3'200'000, "F.Cu"},
      {3'000'000, 3'000'000, "F.Cu"}, {4'000'000, 2'000'000, "F.Cu"}};
  snapshot.vias.push_back(flux::telemetry::Via{"via-signal-0",
                                               "SIGNAL",
                                               3,
                                               {4'000'000, 2'000'000, "F.Cu"},
                                               "B.Cu",
                                               600'000,
                                               7.5,
                                               80'000,
                                               2,
                                               0.75,
                                               3,
                                               flux::telemetry::ViaState::contested,
                                               {}});
  snapshot.conflicts.push_back(
      flux::telemetry::Conflict{"shadow-0",
                                flux::telemetry::ConflictKind::clearance_shadow,
                                "edge-4",
                                "group-a",
                                {2'500'000, 2'000'000, "F.Cu"},
                                {"SIGNAL", "CLOCK"},
                                90'000,
                                2.0,
                                4.0});
  snapshot.resources.push_back(
      flux::telemetry::ResourceOccupancy{"corridor-2",
                                         flux::telemetry::ResourceKind::corridor,
                                         "F.Cu",
                                         {1'000'000, 1'500'000, "F.Cu"},
                                         {5'000'000, 1'500'000, "F.Cu"},
                                         3.0,
                                         2.0,
                                         800'000,
                                         {"SIGNAL", "CLOCK"}});
  snapshot.priorities.push_back(flux::telemetry::Priority{"SIGNAL", "group-a", "constrained", 3.0,
                                                          2.0, 4.0, 5.0, 1.0, 0.8, 2, 1, 19.0, 42});

  flux::telemetry::Event ripup;
  ripup.type = flux::telemetry::EventType::route_ripped_up;
  ripup.sequence = 11;
  ripup.monotonic_time_us = 2'000;
  ripup.routes.push_back(flux::telemetry::Route{"route-signal-0", "SIGNAL"});

  const std::string jsonl =
      flux::telemetry::encode_event(snapshot) + "\n" + flux::telemetry::encode_event(ripup) + "\n";
  std::string error;
  flux::telemetry::Trace trace;
  require(trace.load_text(jsonl, error), "the shared JSONL representation must load for replay");
  require(trace.events().front().routes.front() == snapshot.routes.front(),
          "route geometry and layer IDs must round-trip");
  require(trace.events().front().source_state_included &&
              trace.events().front().source_boards.size() == 1U &&
              trace.events().front().source_boards.front()->primitives.size() == 2U,
          "source-board geometry must round-trip in the shared live/replay schema");

  flux::telemetry::Replay replay;
  replay.set_trace(&trace);
  replay.seek(trace.events().size());
  const auto route = replay.state().routes().find("route-signal-0");
  require(route != replay.state().routes().end(), "rip-up evidence must retain route geometry");
  require(
      route->second.state == flux::telemetry::RouteState::ripped_up &&
          route->second.points.size() == 2U &&
          route->second.geometry == flux::telemetry::RouteGeometry::quintic_ph &&
          route->second.control_points.size() == 6U,
      "rip-up replay must preserve the removed path and quintic PH curve controls for its overlay");
  require(replay.state().vias().at("via-signal-0").state == flux::telemetry::ViaState::contested,
          "contested via sites must remain replayable");
  require(replay.state().source_boards().at("source-board:0")->primitives.back().kind == "pad" &&
              replay.state().source_boards().at("source-board:0")->primitives.back().drill_nm ==
                  500'000U &&
              replay.state()
                      .source_boards()
                      .at("source-board:0")
                      ->primitives.back()
                      .drill_width_nm == 200'000U &&
              replay.state()
                      .source_boards()
                      .at("source-board:0")
                      ->primitives.back()
                      .drill_height_nm == 500'000U &&
              replay.state()
                      .source_boards()
                      .at("source-board:0")
                      ->primitives.back()
                      .drill_offset_x_nm == 100'000 &&
              replay.state()
                      .source_boards()
                      .at("source-board:0")
                      ->primitives.back()
                      .drill_offset_y_nm == -50'000,
          "replay must retain the immutable source scene while dynamic routes change");

  flux::telemetry::LiveStateHistory overlay_history;
  overlay_history.append(trace.events()[0]);
  overlay_history.append(trace.events()[1]);
  require(overlay_history.size() == 2U && overlay_history.position() == 2U &&
              overlay_history.state().routes().at("route-signal-0").state ==
                  flux::telemetry::RouteState::ripped_up,
          "overlay history must follow new chronological telemetry while viewing its tail");
  overlay_history.seek(1U);
  require(overlay_history.position() == 1U &&
              overlay_history.state().routes().at("route-signal-0").state ==
                  flux::telemetry::RouteState::committed &&
              overlay_history.state().routes().at("route-signal-0").control_points.size() == 6U,
          "the overlay timeline must reconstruct prior quintic PH curve geometry exactly");
  flux::telemetry::Event recommit;
  recommit.type = flux::telemetry::EventType::route_committed;
  recommit.sequence = 12U;
  recommit.monotonic_time_us = 3'000U;
  recommit.routes.push_back(flux::telemetry::Route{"route-signal-0", "SIGNAL"});
  overlay_history.append(recommit);
  require(overlay_history.position() == 1U,
          "incoming live events must not move a viewer that is inspecting history");
  overlay_history.seek(overlay_history.size());
  require(overlay_history.state().routes().at("route-signal-0").state ==
                  flux::telemetry::RouteState::committed &&
              overlay_history.state().routes().at("route-signal-0").control_points.size() == 6U,
          "returning to the newest overlay must include events received during historical review");

  flux::RouterTelemetryState summary;
  summary.sequence = 12;
  summary.pathfinder_iteration = 5;
  summary.total_nets = 2;
  summary.routed_nets = 1;
  summary.unrouted_nets = 1;
  const auto summary_record = flux::telemetry::decode_events(
      flux::serialize_router_telemetry_snapshot(summary, flux::RouterTelemetryStatistics{}));
  require(static_cast<bool>(summary_record), "legacy/core summary snapshots must decode");
  replay.state().apply(summary_record.events.front());
  require(replay.state().routes().contains("route-signal-0") &&
              replay.state().source_boards().contains("source-board:0"),
          "a metrics-only recovery snapshot must not erase dynamic or source geometry");
}

void trace_directories_load_chronologically_without_throwing() {
  const auto nonce = std::chrono::steady_clock::now().time_since_epoch().count();
  const auto directory =
      std::filesystem::temp_directory_path() / ("flux-telemetry-viewer-" + std::to_string(nonce));
  struct Cleanup {
    std::filesystem::path path;
    ~Cleanup() {
      std::error_code ignored;
      std::filesystem::remove_all(path, ignored);
    }
  } cleanup{directory};
  std::filesystem::create_directories(directory);

  flux::telemetry::Event first;
  first.type = flux::telemetry::EventType::routing_started;
  first.sequence = 7U;
  first.monotonic_time_us = 900U;
  first.source_identity = "first-board";
  flux::telemetry::Event second;
  second.type = flux::telemetry::EventType::routing_started;
  second.sequence = 1U;
  second.monotonic_time_us = 10U;
  second.source_identity = "second-board";
  const auto first_path = directory / "first.jsonl";
  const auto second_path = directory / "second.jsonl";
  {
    std::ofstream output(first_path);
    output << flux::telemetry::encode_event(first) << '\n';
  }
  {
    std::ofstream output(second_path);
    output << flux::telemetry::encode_event(second) << '\n';
  }
  const auto base_time = std::filesystem::file_time_type::clock::now();
  std::filesystem::last_write_time(first_path, base_time - std::chrono::seconds{1});
  std::filesystem::last_write_time(second_path, base_time);

  flux::telemetry::Trace trace;
  std::string error;
  require(trace.load(directory, error), "a telemetry directory must load as one replay");
  require(trace.events().size() == 2U && trace.events()[0].source_identity == "first-board" &&
              trace.events()[1].source_identity == "second-board" &&
              trace.events()[1].sequence > trace.events()[0].sequence &&
              trace.events()[1].monotonic_time_us > trace.events()[0].monotonic_time_us,
          "directory traces must retain chronological file order and rebase per-run clocks");
}

void viewer_preferences_round_trip_and_clamp_unsafe_values() {
  const auto nonce = std::chrono::steady_clock::now().time_since_epoch().count();
  const auto directory =
      std::filesystem::temp_directory_path() / ("flux-viewer-preferences-" + std::to_string(nonce));
  struct Cleanup {
    std::filesystem::path path;
    ~Cleanup() {
      std::error_code ignored;
      std::filesystem::remove_all(path, ignored);
    }
  } cleanup{directory};
  const auto path = directory / "viewer.conf";
  flux::viewer::ViewerPreferences written;
  written.ui_scale = 4.5F;
  written.activity_panel_height = 415.0F;
  written.replay_speed = 2.5F;
  written.endpoint = "ws://router.local:9000/telemetry";
  written.telemetry_level = "diagnostic";
  written.telemetry_rate_hz = 24.0;
  written.trace_directory = directory / "traces with spaces";
  written.record_directory = directory / "records";
  written.overlays.grid = false;
  written.overlays.ph_controls = true;
  written.overlays.clearance_shadows = false;
  written.overlays.layer = "In1.Cu";
  std::string error;
  require(flux::viewer::save_viewer_preferences(path, written, error),
          "viewer preferences must save in the selected platform configuration directory: " +
              error);
  flux::viewer::ViewerPreferences loaded;
  require(flux::viewer::load_viewer_preferences(path, loaded, error),
          "viewer preferences must reload: " + error);
  require(loaded.ui_scale == 3.0F && loaded.activity_panel_height == 415.0F &&
              loaded.replay_speed == 2.5F && loaded.endpoint == written.endpoint &&
              loaded.telemetry_level == "diagnostic" && loaded.telemetry_rate_hz == 24.0 &&
              loaded.trace_directory == written.trace_directory &&
              loaded.record_directory == written.record_directory && !loaded.overlays.grid &&
              loaded.overlays.ph_controls && !loaded.overlays.clearance_shadows &&
              loaded.overlays.layer == "In1.Cu",
          "saved scale, panel size, trace locations, layer, and checked overlays must round-trip; "
          "UI scale must clamp to the supported 3x maximum");
}

} // namespace

int main() {
  producer_records_decode_into_viewer_state();
  rich_overlay_events_round_trip_and_replay();
  trace_directories_load_chronologically_without_throwing();
  viewer_preferences_round_trip_and_clamp_unsafe_values();
  return 0;
}
