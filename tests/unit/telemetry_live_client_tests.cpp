#include "flux/core/board.hpp"
#include "flux/core/router_telemetry.hpp"
#include "live_client.hpp"

#include <algorithm>
#include <chrono>
#include <iterator>
#include <memory>
#include <stdexcept>
#include <thread>
#include <vector>

namespace {

void require(bool condition, const char* message) {
  if (!condition) {
    throw std::runtime_error(message);
  }
}

} // namespace

int main() {
  flux::RouterTelemetryConfig config;
  config.listen_address = "127.0.0.1";
  config.listen_port = 28765;
  config.publish_rate_hz = 60;
  flux::viewer::LiveClient client;
  require(client.connect("ws://127.0.0.1:28765/telemetry", flux::telemetry::Level::normal, 20.0),
          "the viewer client must start");

  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  require(client.status().state == flux::viewer::ConnectionState::connecting &&
              client.status().connection_generation == 0U,
          "the viewer must wait when Flux has not started yet");

  auto service = std::make_unique<flux::RouterTelemetryService>(config);
  flux::Board source_board;
  source_board.source_revision = "live-source-revision";
  source_board.outline.vertices = {{flux::Nanometres{0}, flux::Nanometres{0}},
                                   {flux::Nanometres{10'000'000}, flux::Nanometres{0}},
                                   {flux::Nanometres{10'000'000}, flux::Nanometres{8'000'000}},
                                   {flux::Nanometres{0}, flux::Nanometres{8'000'000}}};
  source_board.layers.push_back({flux::LayerId{"F.Cu"}, "F.Cu", flux::LayerKind::copper, 0U});
  source_board.layers.push_back({flux::LayerId{"B.Cu"}, "B.Cu", flux::LayerKind::copper, 1U});
  source_board.nets.push_back({flux::NetId{"fixture-net"}, "FIXTURE", 1U});
  flux::Footprint source_footprint;
  source_footprint.id = flux::ObjectId{"fixture-footprint"};
  source_footprint.reference = "U1";
  source_footprint.position = {flux::Nanometres{2'000'000}, flux::Nanometres{3'000'000}};
  source_footprint.courtyard.vertices = {{flux::Nanometres{-500'000}, flux::Nanometres{-400'000}},
                                         {flux::Nanometres{500'000}, flux::Nanometres{-400'000}},
                                         {flux::Nanometres{500'000}, flux::Nanometres{400'000}},
                                         {flux::Nanometres{-500'000}, flux::Nanometres{400'000}}};
  source_footprint.pads.push_back({flux::ObjectId{"fixture-pad"},
                                   flux::NetId{"fixture-net"},
                                   {},
                                   {flux::LayerId{"F.Cu"}},
                                   flux::PadShape::rectangle,
                                   flux::Nanometres{600'000},
                                   flux::Nanometres{400'000}});
  auto custom_pad = source_footprint.pads.front();
  custom_pad.id = flux::ObjectId{"fixture-custom-pad"};
  custom_pad.shape = flux::PadShape::custom;
  custom_pad.custom_copper_polygon = flux::Polygon{{
      {flux::Nanometres{-300'000}, flux::Nanometres{-200'000}},
      {flux::Nanometres{300'000}, flux::Nanometres{-200'000}},
      {flux::Nanometres{100'000}, flux::Nanometres{0}},
      {flux::Nanometres{300'000}, flux::Nanometres{200'000}},
      {flux::Nanometres{-300'000}, flux::Nanometres{200'000}}}};
  source_footprint.pads.push_back(std::move(custom_pad));
  source_board.footprints.push_back(std::move(source_footprint));
  source_board.tracks.push_back({flux::ObjectId{"fixture-track"},
                                 flux::NetId{"fixture-net"},
                                 flux::LayerId{"F.Cu"},
                                 {flux::Nanometres{2'000'000}, flux::Nanometres{3'000'000}},
                                 {flux::Nanometres{5'000'000}, flux::Nanometres{3'000'000}},
                                 flux::Nanometres{200'000}});
  source_board.track_arcs.push_back({flux::ObjectId{"fixture-arc"},
                                     flux::NetId{"fixture-net"},
                                     flux::LayerId{"F.Cu"},
                                     {flux::Nanometres{5'000'000}, flux::Nanometres{3'000'000}},
                                     {flux::Nanometres{5'500'000}, flux::Nanometres{3'500'000}},
                                     {flux::Nanometres{6'000'000}, flux::Nanometres{3'000'000}},
                                     flux::Nanometres{200'000}});
  source_board.vias.push_back({flux::ObjectId{"fixture-via"},
                               flux::NetId{"fixture-net"},
                               {flux::Nanometres{6'000'000}, flux::Nanometres{3'000'000}},
                               flux::Nanometres{600'000},
                               flux::Nanometres{300'000},
                               {flux::LayerId{"F.Cu"}, flux::LayerId{"B.Cu"}}});
  flux::CopperZone source_zone;
  source_zone.id = flux::ObjectId{"fixture-zone"};
  source_zone.net = flux::NetId{"fixture-net"};
  source_zone.name = "fixture fill";
  source_zone.fills.push_back({flux::LayerId{"B.Cu"},
                               {{{flux::Nanometres{7'000'000}, flux::Nanometres{1'000'000}},
                                 {flux::Nanometres{9'000'000}, flux::Nanometres{1'000'000}},
                                 {flux::Nanometres{9'000'000}, flux::Nanometres{2'000'000}},
                                 {flux::Nanometres{7'000'000}, flux::Nanometres{2'000'000}}}}});
  source_board.zones.push_back(std::move(source_zone));
  service->publish_source_board(source_board, "/tmp/live-source.kicad_pcb");
  auto second_source_board = source_board;
  second_source_board.source_revision = "live-second-source-revision";
  service->publish_source_board(second_source_board, "/tmp/live-second-source.kicad_pcb");
  require(service->state().source_boards.size() == 2U,
          "path-plus-revision source identities must retain multiple KiCad scenes");

  const auto connection_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
  while (client.status().state == flux::viewer::ConnectionState::connecting &&
         std::chrono::steady_clock::now() < connection_deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  require(client.status().state == flux::viewer::ConnectionState::connected,
          "the viewer must complete the flux.telemetry.v1 WebSocket handshake");
  const std::uint64_t first_generation = client.status().connection_generation;
  require(first_generation == 1U, "the initial connection must establish one live generation");

  service->publish_generated_copper(second_source_board,
                                    "Copper generated: live-client fixture");
  require(service->state().source_boards.size() == 3U,
          "resulting copper must be retained as a separate viewer scene");

  flux::RouterTelemetryEvent started;
  started.kind = flux::RouterTelemetryEventKind::routing_started;
  started.total_nets = 3;
  started.detail = "live-client-fixture";
  require(service->try_emit(started), "the producer must accept the routing milestone");

  flux::RouterTelemetryEvent iteration;
  iteration.kind = flux::RouterTelemetryEventKind::pathfinder_iteration_completed;
  iteration.iteration = 2;
  iteration.total_nets = 3;
  iteration.routed_nets = 2;
  iteration.unrouted_nets = 1;
  iteration.corridor_overflow = 4;
  require(service->try_emit(iteration), "the producer must accept the iteration milestone");
  flux::RouterTelemetryEvent routing_completed;
  routing_completed.kind = flux::RouterTelemetryEventKind::routing_completed;
  routing_completed.detail = "Routing completed: live-client fixture";
  require(service->try_emit(routing_completed),
          "the producer must accept an explicit successful routing outcome");

  std::vector<flux::telemetry::Event> events;
  const auto event_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
  const auto received_source = [&]() {
    return std::any_of(events.begin(), events.end(), [](const auto& event) {
      return event.type == flux::telemetry::EventType::snapshot &&
             event.source_boards.size() == 3U;
    });
  };
  const auto received_started = [&]() {
    return std::any_of(events.begin(), events.end(), [](const auto& event) {
      return event.type == flux::telemetry::EventType::routing_started;
    });
  };
  const auto received_iteration = [&]() {
    return std::any_of(events.begin(), events.end(), [](const auto& event) {
      return event.type == flux::telemetry::EventType::pathfinder_iteration_completed;
    });
  };
  const auto received_copper = [&]() {
    return std::any_of(events.begin(), events.end(), [](const auto& event) {
      return event.type == flux::telemetry::EventType::copper_generated;
    });
  };
  const auto received_completed = [&]() {
    return std::any_of(events.begin(), events.end(), [](const auto& event) {
      return event.type == flux::telemetry::EventType::routing_completed;
    });
  };
  while (!(received_source() && received_started() && received_iteration() && received_copper() &&
           received_completed()) &&
         std::chrono::steady_clock::now() < event_deadline) {
    auto received = client.drain();
    events.insert(events.end(), std::make_move_iterator(received.begin()),
                  std::make_move_iterator(received.end()));
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  const auto source_snapshot = std::find_if(events.begin(), events.end(), [](const auto& event) {
    return event.type == flux::telemetry::EventType::snapshot && event.source_boards.size() == 3U;
  });
  require(source_snapshot != events.end() && source_snapshot->source_boards.size() == 3U &&
              source_snapshot->source_boards.front()->path == "/tmp/live-source.kicad_pcb" &&
              source_snapshot->source_boards[1]->path == "/tmp/live-second-source.kicad_pcb",
          "a new live viewer must receive every immutable KiCad source and resulting-copper "
          "scene");
  const auto generated_scene = std::find_if(
      source_snapshot->source_boards.begin(), source_snapshot->source_boards.end(),
      [](const auto& board) { return board && board->id.starts_with("generated-copper:"); });
  require(generated_scene != source_snapshot->source_boards.end() &&
              !(*generated_scene)->primitives.empty() &&
              std::all_of((*generated_scene)->primitives.begin(),
                          (*generated_scene)->primitives.end(), [](const auto& primitive) {
                            return primitive.kind == "track" || primitive.kind == "track_arc" ||
                                   primitive.kind == "via";
                          }),
          "the resulting-copper overlay must contain only exact generated tracks, arcs, and vias");
  const auto& scene = *source_snapshot->source_boards.front();
  const auto has_source_kind = [&](std::string_view kind) {
    return std::any_of(scene.primitives.begin(), scene.primitives.end(),
                       [&](const auto& primitive) { return primitive.kind == kind; });
  };
  require(has_source_kind("board_outline") && has_source_kind("footprint") &&
              has_source_kind("pad") && has_source_kind("track") && has_source_kind("track_arc") &&
              has_source_kind("via") && has_source_kind("zone"),
          "the WebSocket recovery scene must retain the KiCad board, fitting, and copper geometry "
          "required by the viewer");
  const auto custom_primitive = std::find_if(scene.primitives.begin(), scene.primitives.end(),
      [](const auto& primitive) { return primitive.id == "fixture-custom-pad"; });
  require(custom_primitive != scene.primitives.end() && custom_primitive->shape == "custom" &&
              custom_primitive->points.size() == 6U &&
              custom_primitive->points[0].x_nm == 2'000'000 &&
              custom_primitive->points[1].x_nm == 1'700'000 &&
              custom_primitive->points[3].x_nm == 2'100'000,
          "custom pad telemetry must retain its anchor and concave world-space copper outline");
  require(received_started(), "the live event must use the shared canonical representation");
  const auto completed = std::find_if(events.begin(), events.end(), [](const auto& event) {
    return event.type == flux::telemetry::EventType::pathfinder_iteration_completed;
  });
  require(completed != events.end() && completed->metrics &&
              completed->metrics->corridor_overflow == 4U,
          "the live viewer must receive authoritative iteration metrics");
  require(received_copper() && received_completed(),
          "the viewer must receive distinct copper-generated and routing-completed milestones");

  const auto drops_before = service->statistics().websocket_dropped;
  flux::RouterTelemetryEvent oversized;
  oversized.kind = flux::RouterTelemetryEventKind::search_summary;
  oversized.detail.assign(flux::viewer::LiveClient::maximum_message_bytes + 1U, 'x');
  require(service->try_emit(std::move(oversized)), "oversized transport fixture must enqueue");
  const auto drop_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
  while (service->statistics().websocket_dropped == drops_before &&
         std::chrono::steady_clock::now() < drop_deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  require(service->statistics().websocket_dropped > drops_before,
          "producer must count oversized payload drops before the viewer reconnects");
  flux::RouterTelemetryEvent after_limit;
  after_limit.kind = flux::RouterTelemetryEventKind::search_summary;
  after_limit.detail = "after-oversized-payload";
  require(service->try_emit(after_limit), "valid telemetry must enqueue after an oversized record");
  bool recovered = false;
  const auto recovery_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
  while (!recovered && std::chrono::steady_clock::now() < recovery_deadline) {
    const auto received = client.drain();
    recovered = std::any_of(received.begin(), received.end(), [](const auto& event) {
      return event.message == "after-oversized-payload";
    });
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  require(recovered && client.status().connection_generation == first_generation,
          "valid telemetry must continue on the same connection after an oversized record");

  service->stop();
  service.reset();
  const auto disconnect_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
  while (client.status().state == flux::viewer::ConnectionState::connected &&
         std::chrono::steady_clock::now() < disconnect_deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  require(client.status().state == flux::viewer::ConnectionState::connecting,
          "the viewer must return to automatic retry when Flux stops");

  service = std::make_unique<flux::RouterTelemetryService>(config);
  const auto reconnect_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
  while (client.status().connection_generation == first_generation &&
         std::chrono::steady_clock::now() < reconnect_deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  require(client.status().state == flux::viewer::ConnectionState::connected &&
              client.status().connection_generation == first_generation + 1U,
          "the viewer must connect to a replacement Flux instance automatically");

  flux::RouterTelemetryEvent restarted;
  restarted.kind = flux::RouterTelemetryEventKind::routing_started;
  restarted.total_nets = 1;
  restarted.detail = "replacement-live-client-fixture";
  require(service->try_emit(restarted), "the replacement producer must accept telemetry");

  std::vector<flux::telemetry::Event> replacement_events;
  const auto replacement_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
  while (replacement_events.empty() && std::chrono::steady_clock::now() < replacement_deadline) {
    auto received = client.drain();
    replacement_events.insert(replacement_events.end(), std::make_move_iterator(received.begin()),
                              std::make_move_iterator(received.end()));
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  require(std::any_of(replacement_events.begin(), replacement_events.end(),
                      [](const auto& event) {
                        return event.type == flux::telemetry::EventType::routing_started &&
                               event.source_identity == "replacement-live-client-fixture";
                      }),
          "the viewer must receive events from the replacement Flux instance");

  client.disconnect();
  service->stop();
  return 0;
}
