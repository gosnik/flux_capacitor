#include "flux/core/router_telemetry.hpp"
#include "flux/telemetry/event.hpp"
#include "flux/telemetry/state.hpp"
#include "flux/telemetry/trace.hpp"
#include "viewer_preferences.hpp"

#include <chrono>
#include <filesystem>
#include <fstream>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>
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
  spline.navmesh_cells = {
      {500'000, 500'000, 2'500'000, 2'500'000, flux::LayerId{"F.Cu"}, 0U},
      {2'500'000, 500'000, 3'500'000, 1'500'000, flux::LayerId{"F.Cu"}, 1U},
      {2'500'000, 1'500'000, 3'000'000, 2'000'000, flux::LayerId{"F.Cu"}, 2U}};

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
          decoded.events[4].routes.front().lifecycle == "retained_fallback" &&
          decoded.events[4].routes.front().navmesh_cells.size() == 3U &&
          decoded.events[4].routes.front().navmesh_cells.back().refinement_level == 2U,
      "regional PH controls, topology certificate, and adaptive NavMesh leaves must cross the "
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
  terminal_snapshot.message = "Routing completed";
  state.apply(terminal_snapshot);
  require(!state.routing_active() && state.status() == "Routing completed",
          "an explicit terminal outcome snapshot must mark routing completed");
}

void live_sessions_append_without_resetting_history() {
  flux::telemetry::LiveSessionTimeline sessions;
  flux::telemetry::LiveStateHistory history;

  const auto first_boundary = sessions.begin_session(1U);
  require(first_boundary && first_boundary->type == flux::telemetry::EventType::session_started,
          "the first live connection must publish a visible session boundary");
  history.append(*first_boundary);

  flux::telemetry::Event first;
  first.type = flux::telemetry::EventType::route_candidate_selected;
  first.sequence = 10U;
  first.monotonic_time_us = 50U;
  first.routes.push_back(flux::telemetry::Route{"session-one-route", "N1"});
  const auto rebased_first = sessions.rebase(first);
  history.append(rebased_first);

  const auto second_boundary = sessions.begin_session(2U);
  require(second_boundary && second_boundary->sequence > rebased_first.sequence &&
              second_boundary->monotonic_time_us > rebased_first.monotonic_time_us,
          "a process reconnect must advance the shared viewer clock");
  history.append(*second_boundary);

  flux::telemetry::Event second;
  second.type = flux::telemetry::EventType::copper_generated;
  second.sequence = 1U;
  second.monotonic_time_us = 2U;
  second.message = "Copper generated";
  const auto rebased_second = sessions.rebase(second);
  history.append(rebased_second);
  require(rebased_second.session_generation == 2U &&
              rebased_second.sequence > second_boundary->sequence &&
              rebased_second.monotonic_time_us > second_boundary->monotonic_time_us &&
              history.size() == 4U && history.state().activity().size() == 4U &&
              history.state().routes().contains("session-one-route"),
          "a reconnect must append rebased events without deleting the preceding session");

  const auto encoded = flux::telemetry::encode_event(rebased_second);
  const auto decoded = flux::telemetry::decode_events(encoded);
  require(decoded && decoded.events.front().type == flux::telemetry::EventType::copper_generated &&
              decoded.events.front().session_generation == 2U,
          "session provenance and copper milestones must survive trace record/replay");

  flux::telemetry::LiveState activity_state;
  for (std::uint64_t index = 1U; index <= 600U; ++index) {
    flux::telemetry::Event event;
    event.type = flux::telemetry::EventType::search_summary;
    event.sequence = index;
    activity_state.apply(event);
  }
  require(activity_state.activity().size() == 600U,
          "the activity view must not silently discard older live-session rows");
}

void new_board_identity_clears_board_specific_live_state() {
  flux::telemetry::LiveState state;

  flux::telemetry::Event first_start;
  first_start.type = flux::telemetry::EventType::routing_started;
  first_start.sequence = 1U;
  first_start.source_identity = "board-a";
  first_start.metrics = flux::telemetry::IterationMetrics{};
  first_start.metrics->total_nets = 4U;
  state.apply(first_start);

  flux::telemetry::Event first_snapshot;
  first_snapshot.type = flux::telemetry::EventType::snapshot;
  first_snapshot.sequence = 2U;
  first_snapshot.source_identity = "board-a";
  first_snapshot.source_state_included = true;
  first_snapshot.dynamic_state_included = true;
  auto board_a = std::make_shared<flux::telemetry::SourceBoard>();
  board_a->id = "source-board-a";
  board_a->revision = "board-a";
  first_snapshot.source_boards.push_back(board_a);
  first_snapshot.routes.push_back(flux::telemetry::Route{"route-a", "net-a"});
  first_snapshot.vias.push_back(flux::telemetry::Via{"via-a", "net-a"});
  first_snapshot.conflicts.push_back(
      flux::telemetry::Conflict{"conflict-a", flux::telemetry::ConflictKind::clearance_shadow});
  first_snapshot.resources.push_back(
      flux::telemetry::ResourceOccupancy{"resource-a", flux::telemetry::ResourceKind::corridor});
  first_snapshot.hydraulic_fields.push_back(
      flux::telemetry::HydraulicField{"field-a", "net-a", "F.Cu"});
  first_snapshot.priorities.push_back(flux::telemetry::Priority{"net-a"});
  first_snapshot.metrics = flux::telemetry::IterationMetrics{};
  first_snapshot.metrics->iteration = 3U;
  first_snapshot.counters = flux::telemetry::Counters{12U, 1U, 2U, 9U, 1U};
  state.apply(first_snapshot);

  flux::telemetry::Event same_board_stage;
  same_board_stage.type = flux::telemetry::EventType::routing_started;
  same_board_stage.sequence = 3U;
  same_board_stage.source_identity = "board-a";
  state.apply(same_board_stage);
  require(state.routes().contains("route-a") && state.activity().size() == 3U,
          "another routing stage for the same board must preserve overlays and activity");

  flux::telemetry::Event second_start;
  second_start.type = flux::telemetry::EventType::routing_started;
  second_start.sequence = 4U;
  second_start.source_identity = "board-b";
  second_start.message = "Routing board B";
  state.apply(second_start);

  require(state.source_identity() == "board-b" && state.routing_active() &&
              state.status() == "Routing board B" && state.sequence() == 4U,
          "the new board's routing boundary must become the complete current status");
  require(state.routes().empty() && state.vias().empty() && state.conflicts().empty() &&
              state.resources().empty() && state.hydraulic_fields().empty() &&
              state.priorities().empty() && state.source_boards().empty() &&
              state.iteration_history().empty() && !state.metrics() &&
              state.counters().produced == 0U && state.activity().size() == 1U &&
              state.activity().front().type == flux::telemetry::EventType::routing_started,
          "a new board must discard every board-specific scene, overlay, metric, and activity row");

  flux::telemetry::LiveStateHistory replay_history;
  replay_history.append(first_start);
  replay_history.append(first_snapshot);
  replay_history.append(same_board_stage);
  replay_history.append(second_start);
  require(replay_history.state().source_identity() == "board-b" &&
              replay_history.state().routes().empty(),
          "the replay tail must not combine geometry from separate boards");
  replay_history.seek(3U);
  require(replay_history.state().source_identity() == "board-a" &&
              replay_history.state().routes().contains("route-a"),
          "seeking before a board boundary must reconstruct the preceding board independently");
  replay_history.seek(4U);
  require(replay_history.state().source_identity() == "board-b" &&
              replay_history.state().routes().empty(),
          "seeking across a board boundary must clean the reconstructed state again");
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
  source_board->physical_thickness_nm = 1'600'000U;
  flux::telemetry::SourceStackup source_stackup;
  source_stackup.copper_finish = "ENIG";
  source_stackup.dielectric_constraints = true;
  source_stackup.edge_connector = "none";
  source_stackup.layers.push_back(
      {"F.Cu", "copper", "copper", "F.Cu", {{35'000U, false, {}, {}, {}, {}}}});
  source_stackup.layers.push_back({"dielectric 1",
                                   "core",
                                   "dielectric",
                                   {},
                                   {{1'530'000U, true, "FR4", 4'500'000'000U, 20'000'000U, {}}}});
  source_board->stackup = std::move(source_stackup);
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
  snapshot.routes.back().navmesh_cells = {
      {{500'000, 1'500'000, "F.Cu"}, {2'500'000, 3'500'000, "F.Cu"}, 0U},
      {{2'500'000, 1'500'000, "F.Cu"}, {3'500'000, 2'500'000, "F.Cu"}, 1U},
      {{3'500'000, 1'500'000, "F.Cu"}, {4'000'000, 2'000'000, "F.Cu"}, 2U}};
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
          route->second.control_points.size() == 6U &&
          route->second.navmesh_cells.size() == 3U &&
          route->second.navmesh_cells.back().refinement_level == 2U,
      "rip-up replay must preserve the removed path, PH controls, and adaptive NavMesh leaves");
  require(replay.state().vias().at("via-signal-0").state == flux::telemetry::ViaState::contested,
          "contested via sites must remain replayable");
  require(
      replay.state().source_boards().at("source-board:0")->primitives.back().kind == "pad" &&
          replay.state().source_boards().at("source-board:0")->primitives.back().drill_nm ==
              500'000U &&
          replay.state().source_boards().at("source-board:0")->primitives.back().drill_width_nm ==
              200'000U &&
          replay.state().source_boards().at("source-board:0")->primitives.back().drill_height_nm ==
              500'000U &&
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
  const auto replayed_source = replay.state().source_boards().at("source-board:0");
  require(replayed_source->physical_thickness_nm == 1'600'000U && replayed_source->stackup &&
              replayed_source->stackup->copper_finish == "ENIG" &&
              replayed_source->stackup->layers.size() == 2U &&
              replayed_source->stackup->layers.back().sublayers.front().relative_permittivity_ppb ==
                  4'500'000'000U,
          "source stackup and fixed-point material properties must survive live/replay JSON");

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

void oversized_trace_records_fail_without_replacing_the_current_replay() {
  const auto nonce = std::chrono::steady_clock::now().time_since_epoch().count();
  const auto path = std::filesystem::temp_directory_path() /
                    ("flux-telemetry-oversized-" + std::to_string(nonce) + ".jsonl");
  struct Cleanup {
    std::filesystem::path path;
    ~Cleanup() {
      std::error_code ignored;
      std::filesystem::remove(path, ignored);
    }
  } cleanup{path};

  flux::telemetry::Event retained;
  retained.type = flux::telemetry::EventType::routing_started;
  retained.source_identity = "retained-before-failed-load";
  flux::telemetry::Trace trace;
  std::string error;
  require(trace.load_text(flux::telemetry::encode_event(retained), error),
          "a valid replay must establish the state protected from a failed load");

  std::string oversized_line(flux::telemetry::Trace::maximum_line_bytes + 1U, 'x');
  {
    std::ofstream output(path, std::ios::binary);
    output << oversized_line << '\n';
  }
  require(!trace.load(path, error) && error.find("8 MiB line limit") != std::string::npos,
          "an oversized JSONL record must be rejected with an actionable limit error");
  require(trace.events().size() == 1U &&
              trace.events().front().source_identity == "retained-before-failed-load",
          "a failed bounded trace load must preserve the previously replayable trace");

  flux::telemetry::Event large;
  large.message.assign(2U * 1024U * 1024U, 'p');
  require(flux::telemetry::estimated_event_bytes(large) > 2U * 1024U * 1024U,
          "the shared budget estimate must account for substantial event payloads");
}

void live_state_rejects_oversized_geometry_atomically() {
  flux::telemetry::LiveState state;
  flux::telemetry::Event retained;
  retained.type = flux::telemetry::EventType::route_committed;
  retained.routes.push_back(flux::telemetry::Route{"retained-route", "NET"});
  retained.routes.front().points = {{0, 0, "F.Cu"}, {1, 1, "F.Cu"}};
  state.apply(retained);

  flux::telemetry::Event oversized;
  oversized.type = flux::telemetry::EventType::snapshot;
  oversized.source_state_included = true;
  auto board = std::make_shared<flux::telemetry::SourceBoard>();
  board->id = "oversized-source";
  flux::telemetry::SourcePrimitive primitive;
  primitive.id = "oversized-primitive";
  primitive.reference.assign(129U * 1024U * 1024U, 'x');
  board->primitives.push_back(std::move(primitive));
  oversized.source_boards.push_back(std::move(board));
  require(flux::telemetry::estimated_event_bytes(oversized) > 128U * 1024U * 1024U,
          "source-scene payloads must count toward shared trace/history/live queue budgets");
  state.apply(oversized);

  require(state.routes().contains("retained-route") && state.source_boards().empty() &&
              state.dropped_oversized_geometry_updates() == 1U,
          "an oversized source snapshot must leave the preceding live geometry intact");

  flux::telemetry::Event region;
  region.active_region = flux::telemetry::ActiveRegion{"region", "active", 0, 0, 1, 1,
                                                        {"F.Cu"}, {"NET"}, true};
  state.apply(region);
  require(state.active_region() && state.active_region()->id == "region",
          "the active-region overlay must be retained while active");
  region.active_region->active = false;
  state.apply(region);
  require(!state.active_region(), "an inactive active-region update must clear the overlay");
}

void live_history_bounds_distinct_source_scene_bytes_and_metadata() {
  flux::telemetry::LiveStateHistory history;
  constexpr std::size_t scene_count = 50U;
  constexpr std::size_t points_per_scene = 50'000U;
  for (std::size_t index = 0U; index < scene_count; ++index) {
    flux::telemetry::Event snapshot;
    snapshot.type = flux::telemetry::EventType::snapshot;
    snapshot.sequence = index + 1U;
    snapshot.source_state_included = true;
    auto board = std::make_shared<flux::telemetry::SourceBoard>();
    board->id = "same-board-id";
    flux::telemetry::SourcePrimitive primitive;
    primitive.id = "dense-scene-" + std::to_string(index);
    primitive.points.assign(points_per_scene, flux::telemetry::Point{0, 0, "F.Cu"});
    board->primitives.push_back(std::move(primitive));
    snapshot.source_boards.push_back(std::move(board));
    history.append(snapshot);
  }
  require(history.dropped_events() > 0U && history.size() == scene_count &&
              history.retained_bytes() <= 64U * 1024U * 1024U,
          "distinct shared source scenes must trigger history byte eviction before its event count cap");
  history.seek(history.retained_begin());
  require(history.position() == history.retained_begin(),
          "seeking the byte-trimmed history base must retain a valid reduced state");
  history.seek(history.size());
  require(history.position() == history.size() &&
              history.state().source_boards().contains("same-board-id"),
          "seeking the byte-trimmed history tail must retain the newest source scene");

  flux::telemetry::LiveState state;
  flux::telemetry::Event verbose;
  verbose.message.assign(2U * 1024U * 1024U, 'm');
  verbose.active_net = std::string(2U * 1024U * 1024U, 'n');
  state.apply(verbose);
  require(!state.activity().empty() && state.activity().back().message.size() <= 8U * 1024U &&
              state.activity().back().net && state.activity().back().net->size() <= 1024U,
          "large telemetry text must be bounded before entering retained LiveState metadata");
}

void malformed_hydraulic_fields_are_rejected_before_rendering() {
  flux::telemetry::HydraulicField edge;
  edge.width = edge.height = edge.stride = 1U;
  edge.cell_pitch_nm = 1U;
  edge.origin_x_nm = std::numeric_limits<std::int64_t>::max();
  edge.flow = edge.erosion = {1U};
  std::string edge_error;
  require(!flux::telemetry::validate_hydraulic_field(edge, edge_error),
          "the far edge of the last heatmap cell must not overflow world coordinates");
  flux::telemetry::HydraulicField field;
  field.id = "bad";
  field.width = std::numeric_limits<std::size_t>::max();
  field.height = 2U;
  field.stride = 1U;
  field.cell_pitch_nm = 1U;
  field.erosion = {1U};
  std::string error;
  require(!flux::telemetry::validate_hydraulic_field(field, error) && !error.empty(),
          "overflowing hydraulic dimensions must be rejected before viewer geometry is formed");
  field.width = 2U;
  field.height = 2U;
  field.flow = {1U, 2U, 3U};
  field.erosion = {1U, 2U, 3U, 4U};
  require(!flux::telemetry::validate_hydraulic_field(field, error),
          "mismatched hydraulic value arrays must be rejected before rendering");
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

void progress_round_trip_and_bounded_history_preserve_retained_seek() {
  flux::telemetry::Event summary;
  summary.type = flux::telemetry::EventType::search_summary;
  summary.monotonic_time_us = 100U;
  summary.progress = flux::telemetry::PhaseProgress{"regional / running", 17U, 42U};
  summary.counters = flux::telemetry::Counters{20U, 1U, 0U, 19U, 2U, 3U};
  const auto decoded = flux::telemetry::decode_events(flux::telemetry::encode_event(summary));
  require(decoded && decoded.events.front().progress == summary.progress,
          "typed phase work and elapsed time must round-trip without a fabricated percentage");
  require(decoded.events.front().counters == summary.counters,
          "transport drops must be visible without conflating producer queue drops");

  flux::telemetry::LiveState state;
  state.apply(decoded.events.front());
  require(state.progress() == summary.progress,
          "a search summary must retain its typed progress in the live state");
  flux::telemetry::Event finished;
  finished.type = flux::telemetry::EventType::routing_finished;
  state.apply(finished);
  require(!state.progress(), "routing completion must clear active phase progress");

  flux::telemetry::Event geometry;
  geometry.type = flux::telemetry::EventType::snapshot;
  geometry.dynamic_state_included = true;
  geometry.monotonic_time_us = 200U;
  flux::telemetry::Route route;
  route.id = "freshness-route";
  route.points = {{0, 0, "F.Cu"}, {100, 0, "F.Cu"}};
  geometry.routes.push_back(route);
  state.apply(geometry);
  geometry.monotonic_time_us = 300U;
  state.apply(geometry);
  require(state.last_geometry_time_us() == 200U,
          "unchanged recovery snapshots must not imply new geometry work");
  geometry.routes.clear();
  geometry.monotonic_time_us = 400U;
  state.apply(geometry);
  require(state.last_geometry_time_us() == 400U,
          "a snapshot removing all geometry must advance geometry freshness");

  flux::telemetry::LiveStateHistory history;
  for (std::size_t index = 0U; index < 4'120U; ++index) {
    flux::telemetry::Event event;
    event.type = flux::telemetry::EventType::search_summary;
    event.sequence = index + 1U;
    event.monotonic_time_us = index + 1U;
    history.append(event);
  }
  require(history.dropped_events() != 0U && history.retained_begin() != 0U &&
              history.size() == 4'120U,
          "live history must report trimmed deltas while retaining monotonic positions");
  history.seek(history.retained_begin());
  require(history.position() == history.retained_begin(),
          "seeking after trimming must reconstruct the retained base checkpoint");
}

void logged_search_diagnostics_are_numeric_bounded_and_scoped() {
  flux::telemetry::LiveState state;
  flux::telemetry::Event summary;
  summary.type = flux::telemetry::EventType::search_summary;
  summary.monotonic_time_us = 55U;
  summary.active_group = "regional:7";
  summary.message = "xpbd_error=0.125 malformed=nan text=hello bad=12foo ratio=4/9 accepted=4";
  state.apply(summary);
  require(state.logged_diagnostics().contains("regional:7.xpbd_error") &&
              state.logged_diagnostics().contains("regional:7.accepted") &&
              !state.logged_diagnostics().contains("regional:7.malformed") &&
              !state.logged_diagnostics().contains("regional:7.text") &&
              !state.logged_diagnostics().contains("regional:7.bad") &&
              !state.logged_diagnostics().contains("regional:7.ratio"),
          "only finite numeric key=value diagnostics from search summaries must be retained");
  flux::telemetry::Event next;
  next.type = flux::telemetry::EventType::search_summary;
  next.monotonic_time_us = 56U;
  next.progress = flux::telemetry::PhaseProgress{"regional / running", 1U, 2U};
  next.message = "lanes=3";
  state.apply(next);
  require(state.logged_diagnostics().at("regional / running.lanes").updated_time_us == 56U,
          "diagnostics without a group must retain their phase scope and source timestamp");
}

void active_region_round_trips_as_explicit_overlay_state() {
  flux::telemetry::Event event;
  event.type = flux::telemetry::EventType::search_summary;
  event.active_region = flux::telemetry::ActiveRegion{"regional:window", "Joint reroute", -10,
                                                       -20, 30, 40, {"F.Cu", "B.Cu"},
                                                       {"N1", "N2"}, true};
  const auto decoded = flux::telemetry::decode_events(flux::telemetry::encode_event(event));
  require(decoded && decoded.events.front().active_region == event.active_region,
          "active regional bounds, layers, nets, and label must round-trip as one optional payload");
  event.active_region->active = false;
  const auto cleared = flux::telemetry::decode_events(flux::telemetry::encode_event(event));
  require(cleared && cleared.events.front().active_region &&
              !cleared.events.front().active_region->active,
          "an explicit inactive regional event must remain distinct from an absent update");
}

} // namespace

int main() {
  {
    flux::telemetry::LiveState state;
    flux::telemetry::Event base;
    base.type = flux::telemetry::EventType::pathfinder_iteration_completed;
    base.metrics.emplace(); base.metrics->iteration = 7; base.metrics->routed_nets = 19;
    state.apply(base);
    flux::RouterTelemetryEvent update;
    update.kind = flux::RouterTelemetryEventKind::search_summary;
    update.packing_obligations_update = true;
    update.unresolved_packing_obligations = 3;
    update.packing_obligation_budget_exhausted = true;
    auto decoded = flux::telemetry::decode_events(flux::serialize_router_telemetry_event(update));
    require(decoded && decoded.events.front().packing_metrics_only, "packing-only wire flag roundtrips");
    state.apply(decoded.events.front());
    require(state.metrics()->unresolved_packing_obligations == 3 &&
                state.metrics()->packing_obligation_budget_exhausted &&
                state.metrics()->routed_nets == 19 && state.iteration() == 7,
            "packing progress preserves unrelated viewer metrics");
    update.unresolved_packing_obligations = 0; update.packing_obligation_budget_exhausted = false;
    decoded = flux::telemetry::decode_events(flux::serialize_router_telemetry_event(update));
    state.apply(decoded.events.front());
    require(state.metrics()->unresolved_packing_obligations == 0 &&
                !state.metrics()->packing_obligation_budget_exhausted &&
                state.metrics()->routed_nets == 19 && state.iteration_history().size() == 1,
            "packing proof clears obligation metrics without resetting routing history");
  }
  producer_records_decode_into_viewer_state();
  rich_overlay_events_round_trip_and_replay();
  live_sessions_append_without_resetting_history();
  new_board_identity_clears_board_specific_live_state();
  trace_directories_load_chronologically_without_throwing();
  oversized_trace_records_fail_without_replacing_the_current_replay();
  live_state_rejects_oversized_geometry_atomically();
  live_history_bounds_distinct_source_scene_bytes_and_metadata();
  malformed_hydraulic_fields_are_rejected_before_rendering();
  progress_round_trip_and_bounded_history_preserve_retained_seek();
  logged_search_diagnostics_are_numeric_bounded_and_scoped();
  active_region_round_trips_as_explicit_overlay_state();
  viewer_preferences_round_trip_and_clamp_unsafe_values();
  return 0;
}
