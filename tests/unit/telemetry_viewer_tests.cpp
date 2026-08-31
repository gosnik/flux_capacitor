#include "flux/core/router_telemetry.hpp"
#include "flux/telemetry/event.hpp"
#include "flux/telemetry/state.hpp"
#include "flux/telemetry/trace.hpp"

#include <stdexcept>
#include <string>
#include <vector>

namespace {

void require(bool condition, const char* message) {
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
  spline.route_geometry = flux::RouterTelemetryRouteGeometry::cubic_bspline;
  spline.spline_degree = 3U;
  spline.topology_signature = 0x1a2b3c4dU;
  spline.topology_gate_count = 3U;
  spline.topology_gate_sequence_legal = true;
  spline.route_points = {{1'000'000, 1'000'000, flux::LayerId{"F.Cu"}},
                         {2'000'000, 1'500'000, flux::LayerId{"F.Cu"}},
                         {3'000'000, 1'000'000, flux::LayerId{"F.Cu"}}};
  spline.spline_control_points = {{1'000'000, 1'000'000, flux::LayerId{"F.Cu"}},
                                  {1'500'000, 2'000'000, flux::LayerId{"F.Cu"}},
                                  {2'500'000, 2'000'000, flux::LayerId{"F.Cu"}},
                                  {3'000'000, 1'000'000, flux::LayerId{"F.Cu"}}};

  const std::string jsonl = flux::serialize_router_telemetry_event(started) + "\n" +
                            flux::serialize_router_telemetry_event(iteration) + "\n" +
                            flux::serialize_router_telemetry_event(via) + "\n" +
                            flux::serialize_router_telemetry_event(conflict) + "\n" +
                            flux::serialize_router_telemetry_event(spline) + "\n";
  const flux::telemetry::DecodeResult decoded = flux::telemetry::decode_events(jsonl);
  require(static_cast<bool>(decoded), "the viewer must decode core producer JSONL records");
  require(decoded.events.size() == 5U, "all producer records must survive decoding");
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
  require(decoded.events[4].routes.size() == 1U &&
              decoded.events[4].routes.front().geometry ==
                  flux::telemetry::RouteGeometry::cubic_bspline &&
              decoded.events[4].routes.front().spline_degree == 3U &&
              decoded.events[4].routes.front().control_points.size() == 4U &&
              decoded.events[4].routes.front().topology_signature == 0x1a2b3c4dU &&
              decoded.events[4].routes.front().topology_gate_count == 3U &&
              decoded.events[4].routes.front().topology_gate_sequence_legal,
          "regional B-spline controls and their directed topology certificate must cross the "
          "producer/viewer telemetry boundary");

  flux::telemetry::LiveState state;
  for (const auto& event : decoded.events) {
    state.apply(event);
  }
  require(state.iteration() == 4U && state.metrics()->unrouted_nets == 2U,
          "producer events must reduce into live state");
  require(state.source_identity() == "board-revision-a",
          "the routing-start source revision must remain observable");
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
                                      300'000,
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
  snapshot.routes.back().geometry = flux::telemetry::RouteGeometry::cubic_bspline;
  snapshot.routes.back().spline_degree = 3U;
  snapshot.routes.back().control_points = {{1'000'000, 2'000'000, "F.Cu"},
                                           {2'000'000, 3'000'000, "F.Cu"},
                                           {3'000'000, 3'000'000, "F.Cu"},
                                           {4'000'000, 2'000'000, "F.Cu"}};
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
  require(route->second.state == flux::telemetry::RouteState::ripped_up &&
              route->second.points.size() == 2U &&
              route->second.geometry == flux::telemetry::RouteGeometry::cubic_bspline &&
              route->second.control_points.size() == 4U,
          "rip-up replay must preserve the removed path and B-spline controls for its overlay");
  require(replay.state().vias().at("via-signal-0").state == flux::telemetry::ViaState::contested,
          "contested via sites must remain replayable");
  require(replay.state().source_boards().at("source-board:0")->primitives.back().kind == "pad" &&
              replay.state().source_boards().at("source-board:0")->primitives.back().drill_nm ==
                  300'000U,
          "replay must retain the immutable source scene while dynamic routes change");

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

} // namespace

int main() {
  producer_records_decode_into_viewer_state();
  rich_overlay_events_round_trip_and_replay();
  return 0;
}
