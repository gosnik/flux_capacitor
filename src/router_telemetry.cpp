#include "flux/core/router_telemetry.hpp"
#include "flux/core/board.hpp"
#include "flux/core/geometry.hpp"
#include "flux/telemetry/event.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <fstream>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <stop_token>
#include <thread>
#include <type_traits>
#include <utility>
#include <vector>

#include <boost/asio/bind_executor.hpp>
#include <boost/asio/ip/address.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <boost/asio/post.hpp>
#include <boost/asio/strand.hpp>
#include <boost/beast/core.hpp>
#include <boost/beast/http/field.hpp>
#include <boost/beast/http/read.hpp>
#include <boost/beast/websocket.hpp>

namespace flux {
namespace {

namespace asio = boost::asio;
namespace beast = boost::beast;
namespace websocket = boost::beast::websocket;
using tcp = asio::ip::tcp;

[[nodiscard]] telemetry::Point source_point(Point point, const LayerId& layer = {}) {
  return telemetry::Point{point.x.value(), point.y.value(), layer.value()};
}

[[nodiscard]] std::vector<telemetry::Point> source_polygon(const Polygon& polygon,
                                                           const LayerId& layer = {}) {
  std::vector<telemetry::Point> result;
  result.reserve(polygon.vertices.size());
  for (const auto point : polygon.vertices) {
    result.push_back(source_point(point, layer));
  }
  return result;
}

[[nodiscard]] std::string_view pad_shape_name(PadShape shape) {
  switch (shape) {
  case PadShape::circle:
    return "circle";
  case PadShape::rectangle:
    return "rectangle";
  case PadShape::oval:
    return "oval";
  case PadShape::round_rectangle:
    return "round_rectangle";
  case PadShape::trapezoid:
    return "trapezoid";
  case PadShape::custom:
    return "custom";
  case PadShape::unknown:
    return "unknown";
  }
  return "unknown";
}

[[nodiscard]] std::string keepout_flags(const KeepoutRules& rules) {
  std::string result;
  const auto append = [&](std::string_view value) {
    if (!result.empty()) {
      result += ',';
    }
    result += value;
  };
  if (rules.tracks) {
    append("tracks");
  }
  if (rules.vias) {
    append("vias");
  }
  if (rules.pads) {
    append("pads");
  }
  if (rules.copper_pours) {
    append("copper_pours");
  }
  if (rules.footprints) {
    append("footprints");
  }
  return result;
}

[[nodiscard]] std::uint64_t deterministic_text_hash(std::string_view value);

[[nodiscard]] std::shared_ptr<const telemetry::SourceBoard>
make_source_board_scene(const Board& board, std::string source_path) {
  auto scene = std::make_shared<telemetry::SourceBoard>();
  // A telemetry service may observe more than one immutable source during a higher-level
  // placement/routing workflow. Key by both source identity and exact imported revision so one
  // scene cannot silently overwrite another at the viewer/replay boundary.
  scene->id = "source-board:" +
              std::to_string(deterministic_text_hash(source_path + '\n' + board.source_revision));
  scene->path = std::move(source_path);
  scene->revision = board.source_revision;
  scene->board_schema = board.schema_version;
  scene->layers.reserve(board.layers.size());
  for (const auto& layer : board.layers) {
    scene->layers.push_back(telemetry::SourceLayer{
        layer.id.value(), layer.name, layer.kind == LayerKind::copper ? "copper" : "technical",
        layer.stack_order});
  }
  if (board.physical_thickness) {
    scene->physical_thickness_nm = static_cast<std::uint64_t>(board.physical_thickness->value());
  }
  if (board.stackup) {
    telemetry::SourceStackup source_stackup;
    source_stackup.copper_finish = board.stackup->copper_finish;
    source_stackup.dielectric_constraints = board.stackup->dielectric_constraints;
    source_stackup.edge_plating = board.stackup->edge_plating;
    source_stackup.castellated_pads = board.stackup->castellated_pads;
    switch (board.stackup->edge_connector) {
    case StackupEdgeConnector::none:
      source_stackup.edge_connector = "none";
      break;
    case StackupEdgeConnector::in_use:
      source_stackup.edge_connector = "in_use";
      break;
    case StackupEdgeConnector::bevelled:
      source_stackup.edge_connector = "bevelled";
      break;
    }
    source_stackup.layers.reserve(board.stackup->layers.size());
    for (const auto& layer : board.stackup->layers) {
      telemetry::SourceStackupLayer source_layer;
      source_layer.name = layer.name;
      source_layer.type = layer.type;
      source_layer.board_layer = layer.board_layer ? layer.board_layer->value() : std::string{};
      switch (layer.kind) {
      case StackupLayerKind::copper:
        source_layer.kind = "copper";
        break;
      case StackupLayerKind::dielectric:
        source_layer.kind = "dielectric";
        break;
      case StackupLayerKind::solder_mask:
        source_layer.kind = "solder_mask";
        break;
      case StackupLayerKind::solder_paste:
        source_layer.kind = "solder_paste";
        break;
      case StackupLayerKind::silkscreen:
        source_layer.kind = "silkscreen";
        break;
      }
      source_layer.sublayers.reserve(layer.sublayers.size());
      for (const auto& sublayer : layer.sublayers) {
        telemetry::SourceStackupSublayer source_sublayer;
        if (sublayer.thickness) {
          source_sublayer.thickness_nm = static_cast<std::uint64_t>(sublayer.thickness->value());
        }
        source_sublayer.thickness_locked = sublayer.thickness_locked;
        source_sublayer.material = sublayer.material;
        if (sublayer.relative_permittivity_ppb) {
          source_sublayer.relative_permittivity_ppb =
              static_cast<std::uint64_t>(*sublayer.relative_permittivity_ppb);
        }
        if (sublayer.loss_tangent_ppb) {
          source_sublayer.loss_tangent_ppb = static_cast<std::uint64_t>(*sublayer.loss_tangent_ppb);
        }
        source_sublayer.color = sublayer.color;
        source_layer.sublayers.push_back(std::move(source_sublayer));
      }
      source_stackup.layers.push_back(std::move(source_layer));
    }
    scene->stackup = std::move(source_stackup);
  }
  scene->nets.reserve(board.nets.size());
  for (const auto& net : board.nets) {
    scene->nets.push_back(telemetry::SourceNet{net.id.value(), net.name, net.priority});
  }
  const auto add_polygon = [&](std::string id, std::string kind, const Polygon& polygon,
                               std::string parent = {}, std::vector<LayerId> layers = {},
                               std::string net = {}, bool filled = false,
                               std::vector<Polygon> holes = {}) {
    telemetry::SourcePrimitive primitive;
    primitive.id = std::move(id);
    primitive.kind = std::move(kind);
    primitive.parent = std::move(parent);
    primitive.net = std::move(net);
    for (const auto& layer : layers) {
      primitive.layers.push_back(layer.value());
    }
    const auto point_layer = layers.size() == 1U ? layers.front() : LayerId{};
    primitive.points = source_polygon(polygon, point_layer);
    for (const auto& hole : holes) {
      primitive.holes.push_back(source_polygon(hole, point_layer));
    }
    primitive.filled = filled;
    scene->primitives.push_back(std::move(primitive));
  };

  add_polygon("board-outline", "board_outline", board.outline);
  for (std::size_t index = 0U; index < board.cutouts.size(); ++index) {
    add_polygon("board-cutout:" + std::to_string(index), "board_cutout", board.cutouts[index]);
  }
  for (const auto& footprint : board.footprints) {
    const auto world_courtyard = geometry::transform_polygon(
        footprint.courtyard, footprint.position, footprint.rotation, footprint.side);
    add_polygon(footprint.id.value(), "footprint", world_courtyard);
    auto& source_footprint = scene->primitives.back();
    source_footprint.reference = footprint.reference;
    source_footprint.rotation_microdegrees = footprint.rotation.value();
    source_footprint.side = footprint.side == BoardSide::front ? "front" : "back";
    source_footprint.locked = footprint.locked;
    for (std::size_t circle = 0U; circle < footprint.courtyard_circles.size(); ++circle) {
      const auto& source_circle = footprint.courtyard_circles[circle];
      telemetry::SourcePrimitive primitive;
      primitive.id = footprint.id.value() + ":courtyard-circle:" + std::to_string(circle);
      primitive.kind = "footprint_courtyard_circle";
      primitive.parent = footprint.id.value();
      primitive.reference = footprint.reference;
      primitive.points = {
          source_point(geometry::transform_point(source_circle.center, footprint.position,
                                                 footprint.rotation, footprint.side)),
          source_point(geometry::transform_point(source_circle.edge, footprint.position,
                                                 footprint.rotation, footprint.side))};
      scene->primitives.push_back(std::move(primitive));
    }
    for (const auto& pad : footprint.pads) {
      telemetry::SourcePrimitive primitive;
      primitive.id = pad.id.value();
      primitive.kind = "pad";
      primitive.parent = footprint.id.value();
      primitive.reference = footprint.reference;
      primitive.net = pad.net.value();
      for (const auto& layer : pad.layers) {
        primitive.layers.push_back(layer.value());
      }
      primitive.points.push_back(source_point(geometry::transform_point(
          pad.local_position, footprint.position, footprint.rotation, footprint.side)));
      primitive.width_nm = static_cast<std::uint64_t>(std::max<std::int64_t>(0, pad.width.value()));
      primitive.height_nm =
          static_cast<std::uint64_t>(std::max<std::int64_t>(0, pad.height.value()));
      primitive.drill_width_nm =
          static_cast<std::uint64_t>(std::max<std::int64_t>(0, pad.drill_width.value()));
      primitive.drill_height_nm =
          static_cast<std::uint64_t>(std::max<std::int64_t>(0, pad.drill_height.value()));
      primitive.drill_nm = std::max(primitive.drill_width_nm, primitive.drill_height_nm);
      primitive.drill_offset_x_nm = pad.drill_offset.x.value();
      primitive.drill_offset_y_nm = pad.drill_offset.y.value();
      const auto local_rotation =
          footprint.side == BoardSide::front ? pad.rotation.value() : -pad.rotation.value();
      primitive.rotation_microdegrees =
          Microdegrees{footprint.rotation.value() + local_rotation}.value();
      primitive.shape = pad_shape_name(pad.shape);
      primitive.side = footprint.side == BoardSide::front ? "front" : "back";
      primitive.filled = true;
      scene->primitives.push_back(std::move(primitive));
    }
    for (const auto& keepout : footprint.keepouts) {
      const auto area = geometry::transform_polygon(keepout.area, footprint.position,
                                                    footprint.rotation, footprint.side);
      add_polygon(keepout.id.value(), "keepout", area, footprint.id.value(), keepout.layers);
      scene->primitives.back().shape = keepout_flags(keepout.prohibit);
    }
  }
  for (const auto& track : board.tracks) {
    telemetry::SourcePrimitive primitive;
    primitive.id = track.id.value();
    primitive.kind = "track";
    primitive.net = track.net.value();
    primitive.layers = {track.layer.value()};
    primitive.points = {source_point(track.start, track.layer),
                        source_point(track.end, track.layer)};
    primitive.width_nm = static_cast<std::uint64_t>(std::max<std::int64_t>(0, track.width.value()));
    primitive.locked = track.locked;
    scene->primitives.push_back(std::move(primitive));
  }
  for (const auto& arc : board.track_arcs) {
    telemetry::SourcePrimitive primitive;
    primitive.id = arc.id.value();
    primitive.kind = "track_arc";
    primitive.net = arc.net.value();
    primitive.layers = {arc.layer.value()};
    primitive.points = {source_point(arc.start, arc.layer), source_point(arc.middle, arc.layer),
                        source_point(arc.end, arc.layer)};
    primitive.width_nm = static_cast<std::uint64_t>(std::max<std::int64_t>(0, arc.width.value()));
    primitive.locked = arc.locked;
    scene->primitives.push_back(std::move(primitive));
  }
  for (const auto& via : board.vias) {
    telemetry::SourcePrimitive primitive;
    primitive.id = via.id.value();
    primitive.kind = "via";
    primitive.net = via.net.value();
    for (const auto& layer : via.layers) {
      primitive.layers.push_back(layer.value());
    }
    primitive.points = {
        source_point(via.position, via.layers.empty() ? LayerId{} : via.layers.front())};
    primitive.diameter_nm =
        static_cast<std::uint64_t>(std::max<std::int64_t>(0, via.diameter.value()));
    primitive.drill_nm = static_cast<std::uint64_t>(std::max<std::int64_t>(0, via.drill.value()));
    primitive.locked = via.locked;
    primitive.filled = true;
    scene->primitives.push_back(std::move(primitive));
  }
  for (const auto& zone : board.zones) {
    if (zone.requires_refill) {
      telemetry::SourcePrimitive primitive;
      primitive.id = zone.id.value();
      primitive.kind = "zone_refill_required";
      primitive.reference = zone.name;
      primitive.net = zone.net.value();
      primitive.shape = "source-zone-without-saved-fill";
      primitive.locked = zone.locked;
      scene->primitives.push_back(std::move(primitive));
    }
    for (std::size_t fill = 0U; fill < zone.fills.size(); ++fill) {
      const auto& source_fill = zone.fills[fill];
      add_polygon(zone.id.value() + ":fill:" + std::to_string(fill), "zone", source_fill.area,
                  zone.id.value(), {source_fill.layer}, zone.net.value(), true, source_fill.holes);
      scene->primitives.back().reference = zone.name;
      scene->primitives.back().locked = zone.locked;
    }
  }
  for (const auto& keepout : board.keepouts) {
    add_polygon(keepout.id.value(), "keepout", keepout.area, {}, keepout.layers);
    scene->primitives.back().reference = keepout.name;
    scene->primitives.back().shape = keepout_flags(keepout.prohibit);
  }
  return scene;
}

[[nodiscard]] std::uint64_t deterministic_text_hash(std::string_view value) {
  constexpr std::uint64_t offset_basis = 14695981039346656037ULL;
  constexpr std::uint64_t prime = 1099511628211ULL;
  std::uint64_t result = offset_basis;
  for (const char character : value) {
    result ^= static_cast<unsigned char>(character);
    result *= prime;
  }
  return result;
}

[[nodiscard]] telemetry::EventType telemetry_event_type(RouterTelemetryEventKind kind) {
  using Source = RouterTelemetryEventKind;
  using Target = telemetry::EventType;
  switch (kind) {
  case Source::routing_started:
    return Target::routing_started;
  case Source::routing_finished:
    return Target::routing_finished;
  case Source::global_routing_started:
  case Source::global_routing_finished:
    return Target::search_summary;
  case Source::pathfinder_iteration_started:
    return Target::pathfinder_iteration_started;
  case Source::pathfinder_iteration_completed:
    return Target::pathfinder_iteration_completed;
  case Source::best_checkpoint_updated:
    return Target::best_checkpoint_updated;
  case Source::net_queued:
    return Target::net_queued;
  case Source::net_started:
    return Target::net_started;
  case Source::net_completed:
    return Target::net_completed;
  case Source::net_failed:
    return Target::net_failed;
  case Source::route_candidate_selected:
    return Target::route_candidate_selected;
  case Source::route_committed:
    return Target::route_committed;
  case Source::route_removed:
    return Target::route_removed;
  case Source::route_ripped_up:
    return Target::route_ripped_up;
  case Source::via_candidate_generated:
    return Target::via_candidate_generated;
  case Source::via_candidate_rejected:
    return Target::via_candidate_rejected;
  case Source::via_assigned:
    return Target::via_assigned;
  case Source::via_removed:
    return Target::via_removed;
  case Source::conflict_added:
    return Target::conflict_added;
  case Source::conflict_removed:
    return Target::conflict_removed;
  case Source::hydraulic_field_updated:
    return Target::hydraulic_field_updated;
  case Source::residual_group_updated:
    return Target::residual_group_updated;
  case Source::checkpoint_selected:
    return Target::checkpoint_selected;
  case Source::search_summary:
    return Target::search_summary;
  case Source::placement_proposal_started:
    return Target::placement_proposal_started;
  case Source::placement_proposal_accepted:
    return Target::placement_proposal_accepted;
  case Source::placement_proposal_rejected:
    return Target::placement_proposal_rejected;
  }
  return Target::unknown;
}

[[nodiscard]] telemetry::Level telemetry_level(RouterTelemetryLevel level) {
  switch (level) {
  case RouterTelemetryLevel::normal:
    return telemetry::Level::normal;
  case RouterTelemetryLevel::diagnostic:
    return telemetry::Level::diagnostic;
  case RouterTelemetryLevel::trace:
    return telemetry::Level::trace;
  }
  return telemetry::Level::normal;
}

[[nodiscard]] telemetry::IterationMetrics iteration_metrics(const RouterTelemetryEvent& event) {
  telemetry::IterationMetrics metrics;
  metrics.iteration = event.iteration;
  metrics.total_nets = event.total_nets;
  metrics.routed_nets = event.routed_nets;
  metrics.unrouted_nets = event.unrouted_nets;
  metrics.sampled_overflow = event.sampled_overflow;
  metrics.corridor_overflow = event.corridor_overflow;
  metrics.physical_overflow_nm = event.physical_overflow_nm;
  metrics.clearance_shadow_conflicts = event.clearance_shadow_conflicts;
  metrics.route_cost = event.route_cost;
  metrics.vias = event.via_count;
  metrics.rip_ups = event.ripup_count;
  metrics.search_expansions = event.search_expansions;
  metrics.total_groups = event.total_groups;
  metrics.regional_conflicts = event.regional_conflicts;
  return metrics;
}

[[nodiscard]] telemetry::Priority priority_record(const RouterTelemetryEvent& event) {
  telemetry::Priority priority;
  priority.net = event.net.value();
  priority.group = event.stable_id;
  priority.staged_class = event.priority_stage;
  priority.explicit_priority = static_cast<double>(event.explicit_priority);
  priority.terminal_scarcity = 1.0 / (1.0 + static_cast<double>(event.terminal_portals));
  priority.corridor_scarcity = 1.0 / (1.0 + static_cast<double>(event.corridor_alternatives));
  priority.via_scarcity = event.via_transitions == 0U
                              ? 0.0
                              : (1.0 + static_cast<double>(event.via_contention)) /
                                    (1.0 + static_cast<double>(event.via_alternatives));
  priority.routing_interference = static_cast<double>(event.estimated_interference);
  priority.physical_width_demand = static_cast<double>(event.physical_envelope_nm);
  priority.topology_alternatives = event.topology_alternatives;
  priority.layer_alternatives = event.layer_alternatives;
  priority.final_priority = static_cast<double>(event.priority_ordinal);
  priority.stable_tie_break = deterministic_text_hash(event.stable_id);
  return priority;
}

[[nodiscard]] telemetry::Route route_record(const RouterTelemetryEvent& event) {
  telemetry::Route route;
  route.id = event.stable_id;
  route.net = event.net.value();
  route.revision = event.route_revision;
  route.width_nm = event.route_width_nm;
  route.cost = event.candidate_cost;
  route.family = event.route_family;
  route.lifecycle = event.route_lifecycle;
  route.geometry = event.route_geometry == RouterTelemetryRouteGeometry::quintic_ph
                       ? telemetry::RouteGeometry::quintic_ph
                       : telemetry::RouteGeometry::polyline;
  route.spline_degree = event.spline_degree;
  route.topology_signature = event.topology_signature;
  route.topology_gate_count = event.topology_gate_count;
  route.topology_gate_sequence_legal = event.topology_gate_sequence_legal;
  switch (event.kind) {
  case RouterTelemetryEventKind::route_committed:
    route.state = telemetry::RouteState::committed;
    break;
  case RouterTelemetryEventKind::route_ripped_up:
    route.state = telemetry::RouteState::ripped_up;
    break;
  case RouterTelemetryEventKind::route_removed:
    route.state = telemetry::RouteState::removed;
    break;
  case RouterTelemetryEventKind::route_candidate_selected:
    route.state = telemetry::RouteState::provisional;
    break;
  default:
    route.state = telemetry::RouteState::current;
    break;
  }
  route.points.reserve(event.route_points.size());
  for (const auto& point : event.route_points) {
    route.points.push_back(telemetry::Point{point.x_nm, point.y_nm, point.layer.value()});
  }
  route.control_points.reserve(event.spline_control_points.size());
  for (const auto& point : event.spline_control_points) {
    route.control_points.push_back(telemetry::Point{point.x_nm, point.y_nm, point.layer.value()});
  }
  return route;
}

[[nodiscard]] telemetry::Via via_record(const RouterTelemetryEvent& event) {
  telemetry::Via via;
  via.id = event.stable_id;
  via.net = event.net.value();
  via.revision = event.route_revision;
  via.position = telemetry::Point{event.x_nm, event.y_nm, event.layer.value()};
  via.to_layer = event.last_layer.value();
  via.diameter_nm = event.diameter_nm;
  via.score = event.score;
  via.clearance_margin_nm = event.clearance_margin_nm;
  via.competing_nets = event.competing_nets;
  via.local_congestion = event.local_congestion;
  via.remaining_alternatives = event.remaining_alternatives;
  via.rejection_reason = event.rejection_reason;
  switch (event.kind) {
  case RouterTelemetryEventKind::via_assigned:
    via.state = telemetry::ViaState::selected;
    break;
  case RouterTelemetryEventKind::via_candidate_rejected:
    via.state = telemetry::ViaState::rejected;
    break;
  case RouterTelemetryEventKind::via_removed:
    via.state = telemetry::ViaState::removed;
    break;
  default:
    via.state = event.competing_nets == 0U ? telemetry::ViaState::candidate
                                           : telemetry::ViaState::contested;
    break;
  }
  return via;
}

[[nodiscard]] telemetry::Conflict conflict_record(const RouterTelemetryEvent& event) {
  telemetry::Conflict conflict;
  conflict.id = event.stable_id;
  conflict.kind = telemetry::conflict_kind_from_string(event.conflict_kind);
  conflict.resource = event.resource_id;
  conflict.group = event.group_id;
  conflict.location = telemetry::Point{event.x_nm, event.y_nm, event.layer.value()};
  if (!event.net.empty()) {
    conflict.nets.push_back(event.net.value());
  }
  if (!event.other_net.empty() && event.other_net != event.net) {
    conflict.nets.push_back(event.other_net.value());
  }
  conflict.clearance_deficit_nm = event.clearance_deficit_nm;
  conflict.pressure = event.pressure;
  conflict.history = event.history;
  return conflict;
}

[[nodiscard]] telemetry::HydraulicField hydraulic_field_record(const RouterTelemetryEvent& event) {
  telemetry::HydraulicField field;
  field.id = event.stable_id;
  field.net = event.net.value();
  field.layer = event.layer.value();
  field.revision = event.route_revision;
  field.origin_x_nm = event.x_nm;
  field.origin_y_nm = event.y_nm;
  field.cell_pitch_nm = event.hydraulic_field_pitch_nm;
  field.width = event.hydraulic_field_width;
  field.height = event.hydraulic_field_height;
  field.stride = event.hydraulic_field_stride;
  field.maximum_flow = event.hydraulic_maximum_flow;
  field.maximum_erosion = event.hydraulic_maximum_erosion;
  field.maximum_terrain_height = event.hydraulic_maximum_terrain_height;
  field.maximum_downstream_pressure = event.hydraulic_maximum_downstream_pressure;
  field.maximum_success_prior = event.hydraulic_maximum_success_prior;
  field.flow = event.hydraulic_flow_field;
  field.erosion = event.hydraulic_erosion_field;
  field.heightmap = event.hydraulic_heightmap;
  field.free_space = event.hydraulic_free_space_field;
  field.downstream_pressure = event.hydraulic_downstream_pressure_field;
  field.success_prior = event.hydraulic_success_prior_field;
  return field;
}

[[nodiscard]] telemetry::Event canonical_event(const RouterTelemetryEvent& event,
                                               RouterTelemetryLevel level,
                                               std::uint64_t monotonic_time_us) {
  telemetry::Event result;
  result.sequence = event.sequence;
  result.monotonic_time_us = monotonic_time_us;
  result.level = telemetry_level(level);
  result.type = telemetry_event_type(event.kind);
  if (!event.net.empty()) {
    result.active_net = event.net.value();
  }
  if (!event.stable_id.empty()) {
    result.active_group = event.stable_id;
  }
  if (event.kind == RouterTelemetryEventKind::routing_started) {
    result.source_identity = event.detail;
  } else {
    result.message = event.detail;
  }
  if (event.kind == RouterTelemetryEventKind::routing_started ||
      event.kind == RouterTelemetryEventKind::routing_finished ||
      event.kind == RouterTelemetryEventKind::global_routing_started ||
      event.kind == RouterTelemetryEventKind::global_routing_finished ||
      event.kind == RouterTelemetryEventKind::pathfinder_iteration_started ||
      event.kind == RouterTelemetryEventKind::pathfinder_iteration_completed ||
      event.kind == RouterTelemetryEventKind::best_checkpoint_updated ||
      event.kind == RouterTelemetryEventKind::checkpoint_selected) {
    result.metrics = iteration_metrics(event);
  }
  if (event.kind == RouterTelemetryEventKind::net_queued && !event.net.empty()) {
    result.priorities.push_back(priority_record(event));
  }
  if (event.kind == RouterTelemetryEventKind::route_candidate_selected ||
      event.kind == RouterTelemetryEventKind::route_committed ||
      event.kind == RouterTelemetryEventKind::route_removed ||
      event.kind == RouterTelemetryEventKind::route_ripped_up) {
    result.routes.push_back(route_record(event));
  }
  if (event.kind == RouterTelemetryEventKind::via_candidate_generated ||
      event.kind == RouterTelemetryEventKind::via_candidate_rejected ||
      event.kind == RouterTelemetryEventKind::via_assigned ||
      event.kind == RouterTelemetryEventKind::via_removed) {
    result.vias.push_back(via_record(event));
  }
  if (event.kind == RouterTelemetryEventKind::conflict_added ||
      event.kind == RouterTelemetryEventKind::conflict_removed) {
    result.conflicts.push_back(conflict_record(event));
  }
  if (event.kind == RouterTelemetryEventKind::hydraulic_field_updated) {
    result.hydraulic_fields.push_back(hydraulic_field_record(event));
  }
  if (event.kind == RouterTelemetryEventKind::residual_group_updated) {
    result.metrics = iteration_metrics(event);
  }
  return result;
}

[[nodiscard]] telemetry::Event canonical_snapshot(const RouterTelemetryState& state,
                                                  const RouterTelemetryStatistics& statistics,
                                                  RouterTelemetryLevel level,
                                                  std::uint64_t monotonic_time_us,
                                                  bool include_source) {
  telemetry::Event snapshot;
  snapshot.sequence = state.sequence;
  snapshot.monotonic_time_us = monotonic_time_us;
  snapshot.level = telemetry_level(level);
  snapshot.type = telemetry::EventType::snapshot;
  snapshot.source_identity = state.source_identity;
  if (!state.active_net.empty()) {
    snapshot.active_net = state.active_net.value();
  }
  telemetry::IterationMetrics metrics;
  metrics.iteration = state.pathfinder_iteration;
  metrics.total_nets = state.total_nets;
  metrics.routed_nets = state.routed_nets;
  metrics.unrouted_nets = state.unrouted_nets;
  metrics.sampled_overflow = state.sampled_conflicts;
  metrics.corridor_overflow = state.corridor_conflicts;
  metrics.physical_overflow_nm = state.physical_overflow_nm;
  metrics.clearance_shadow_conflicts = state.clearance_shadow_conflicts;
  metrics.total_groups = state.total_groups;
  metrics.regional_conflicts = state.regional_conflicts;
  metrics.route_cost = state.route_cost;
  metrics.vias = state.via_count;
  metrics.rip_ups = state.ripup_count;
  metrics.search_expansions = state.search_expansions;
  snapshot.metrics = metrics;
  snapshot.counters =
      telemetry::Counters{statistics.produced, statistics.dropped, statistics.coalesced,
                          statistics.consumed, statistics.snapshots};
  snapshot.message = state.routing_finished ? "Routing finished" : "Routing snapshot";
  snapshot.source_state_included = include_source && state.source_state_included;
  if (snapshot.source_state_included) {
    snapshot.source_boards = state.source_boards;
  }
  snapshot.dynamic_state_included = state.dynamic_state_included;
  if (snapshot.dynamic_state_included) {
    for (const auto& event : state.observed_priorities) {
      snapshot.priorities.push_back(priority_record(event));
    }
    for (const auto& event : state.observed_vias) {
      snapshot.vias.push_back(via_record(event));
    }
    for (const auto& event : state.observed_routes) {
      snapshot.routes.push_back(route_record(event));
    }
    for (const auto& event : state.observed_conflicts) {
      snapshot.conflicts.push_back(conflict_record(event));
    }
    for (const auto& event : state.observed_hydraulic_fields) {
      snapshot.hydraulic_fields.push_back(hydraulic_field_record(event));
    }
  }
  return snapshot;
}

class WebSocketTelemetrySession : public std::enable_shared_from_this<WebSocketTelemetrySession> {
public:
  using Closed = std::function<void(WebSocketTelemetrySession*)>;

  WebSocketTelemetrySession(tcp::socket socket, std::size_t queue_capacity, Closed closed)
      : websocket_(std::move(socket)), queue_capacity_(queue_capacity), closed_(std::move(closed)) {
  }

  void run() {
    websocket_.set_option(websocket::stream_base::timeout::suggested(beast::role_type::server));
    websocket_.set_option(websocket::stream_base::decorator([](websocket::response_type& response) {
      response.set(beast::http::field::sec_websocket_protocol,
                   std::string(telemetry::protocol_name));
      response.set(beast::http::field::server, "flux-capacitor-telemetry");
    }));
    beast::http::async_read(
        websocket_.next_layer(), read_buffer_, request_,
        [self = shared_from_this()](beast::error_code error, std::size_t) {
          if (error || self->request_.target() != "/telemetry") {
            self->finish();
            return;
          }
          const auto requested_protocol =
              self->request_[beast::http::field::sec_websocket_protocol];
          if (requested_protocol.find(telemetry::protocol_name) == beast::string_view::npos) {
            self->finish();
            return;
          }
          self->websocket_.async_accept(self->request_, [self](beast::error_code accept_error) {
            self->accepted(accept_error);
          });
        });
  }

  [[nodiscard]] bool send(std::shared_ptr<const std::string> payload) {
    if (closed_flag_) {
      return false;
    }
    if (writes_.size() >= queue_capacity_) {
      return false;
    }
    const bool idle = writes_.empty();
    writes_.push_back(std::move(payload));
    if (idle && accepted_) {
      write_next();
    }
    return true;
  }

  void close() {
    if (closed_flag_) {
      return;
    }
    closed_flag_ = true;
    beast::error_code ignored;
    websocket_.next_layer().socket().shutdown(tcp::socket::shutdown_both, ignored);
    websocket_.next_layer().socket().close(ignored);
  }

private:
  void accepted(beast::error_code error) {
    if (error) {
      finish();
      return;
    }
    accepted_ = true;
    read_next();
    if (!writes_.empty()) {
      write_next();
    }
  }

  void read_next() {
    websocket_.async_read(read_buffer_,
                          [self = shared_from_this()](beast::error_code error, std::size_t) {
                            if (error) {
                              self->finish();
                              return;
                            }
                            // The first protocol is deliberately read-only. Consume requests
                            // without forwarding them to routing state; periodic snapshots provide
                            // recovery.
                            self->read_buffer_.consume(self->read_buffer_.size());
                            self->read_next();
                          });
  }

  void write_next() {
    if (writes_.empty() || closed_flag_) {
      return;
    }
    websocket_.text(true);
    websocket_.async_write(asio::buffer(*writes_.front()),
                           [self = shared_from_this()](beast::error_code error, std::size_t) {
                             if (error) {
                               self->finish();
                               return;
                             }
                             self->writes_.pop_front();
                             if (!self->writes_.empty()) {
                               self->write_next();
                             }
                           });
  }

  void finish() {
    if (closed_flag_) {
      return;
    }
    closed_flag_ = true;
    if (closed_) {
      closed_(this);
    }
  }

  websocket::stream<beast::tcp_stream> websocket_;
  beast::flat_buffer read_buffer_;
  beast::http::request<beast::http::string_body> request_;
  std::deque<std::shared_ptr<const std::string>> writes_;
  std::size_t queue_capacity_{0};
  Closed closed_;
  bool accepted_{false};
  bool closed_flag_{false};
};

class WebSocketTelemetryServer {
public:
  WebSocketTelemetryServer(std::string address, std::uint16_t port,
                           std::size_t session_queue_capacity)
      : acceptor_(io_), strand_(asio::make_strand(io_)),
        session_queue_capacity_(session_queue_capacity) {
    const tcp::endpoint endpoint{asio::ip::make_address(address), port};
    acceptor_.open(endpoint.protocol());
    acceptor_.set_option(asio::socket_base::reuse_address(true));
    acceptor_.bind(endpoint);
    acceptor_.listen(asio::socket_base::max_listen_connections);
    accept_next();
    thread_ = std::thread([this]() { io_.run(); });
  }

  ~WebSocketTelemetryServer() { stop(); }

  WebSocketTelemetryServer(const WebSocketTelemetryServer&) = delete;
  WebSocketTelemetryServer& operator=(const WebSocketTelemetryServer&) = delete;

  [[nodiscard]] bool publish(std::string payload, bool snapshot,
                             bool source_snapshot = false) noexcept {
    try {
      std::unique_lock lock(pending_mutex_, std::try_to_lock);
      if (!lock.owns_lock() || stopped_.load(std::memory_order_relaxed) ||
          pending_.size() >= maximum_pending_messages) {
        dropped_.fetch_add(1U, std::memory_order_relaxed);
        return false;
      }
      pending_.push_back(Pending{std::make_shared<const std::string>(std::move(payload)), snapshot,
                                 source_snapshot});
      asio::post(strand_, [this]() { drain_pending(); });
      return true;
    } catch (...) {
      dropped_.fetch_add(1U, std::memory_order_relaxed);
      return false;
    }
  }

  void stop() noexcept {
    if (stopped_.exchange(true, std::memory_order_relaxed)) {
      return;
    }
    io_.stop();
    if (thread_.joinable()) {
      thread_.join();
    }
    beast::error_code ignored;
    acceptor_.close(ignored);
    for (const auto& session : sessions_) {
      session->close();
    }
    sessions_.clear();
  }

  [[nodiscard]] std::uint64_t dropped() const { return dropped_.load(std::memory_order_relaxed); }
  [[nodiscard]] std::uint64_t published() const {
    return published_.load(std::memory_order_relaxed);
  }

private:
  struct Pending {
    std::shared_ptr<const std::string> payload;
    bool snapshot{false};
    bool source_snapshot{false};
  };

  void accept_next() {
    acceptor_.async_accept(
        asio::bind_executor(strand_, [this](beast::error_code error, tcp::socket socket) {
          if (!error && !stopped_.load(std::memory_order_relaxed) &&
              sessions_.size() < maximum_sessions) {
            auto session = std::make_shared<WebSocketTelemetrySession>(
                std::move(socket), session_queue_capacity_,
                [this](WebSocketTelemetrySession* closed) { remove_session(closed); });
            sessions_.push_back(session);
            if (latest_source_snapshot_) {
              static_cast<void>(session->send(latest_source_snapshot_));
            }
            if (latest_snapshot_) {
              static_cast<void>(session->send(latest_snapshot_));
            }
            session->run();
          }
          if (!stopped_.load(std::memory_order_relaxed)) {
            accept_next();
          }
        }));
  }

  void remove_session(WebSocketTelemetrySession* closed) {
    std::erase_if(sessions_, [closed](const auto& session) { return session.get() == closed; });
  }

  void drain_pending() {
    std::deque<Pending> local;
    {
      std::lock_guard lock(pending_mutex_);
      local.swap(pending_);
    }
    for (auto& pending : local) {
      if (pending.source_snapshot) {
        latest_source_snapshot_ = pending.payload;
      } else if (pending.snapshot) {
        latest_snapshot_ = pending.payload;
      }
      for (const auto& session : sessions_) {
        if (!session->send(pending.payload)) {
          dropped_.fetch_add(1U, std::memory_order_relaxed);
        }
      }
      published_.fetch_add(1U, std::memory_order_relaxed);
    }
  }

  static constexpr std::size_t maximum_sessions = 8U;
  static constexpr std::size_t maximum_pending_messages = 128U;
  asio::io_context io_;
  tcp::acceptor acceptor_;
  asio::strand<asio::io_context::executor_type> strand_;
  std::size_t session_queue_capacity_{0};
  std::thread thread_;
  std::atomic_bool stopped_{false};
  std::mutex pending_mutex_;
  std::deque<Pending> pending_;
  std::vector<std::shared_ptr<WebSocketTelemetrySession>> sessions_;
  std::shared_ptr<const std::string> latest_source_snapshot_;
  std::shared_ptr<const std::string> latest_snapshot_;
  std::atomic_uint64_t dropped_{0};
  std::atomic_uint64_t published_{0};
};

} // namespace

std::string_view router_telemetry_event_kind_name(RouterTelemetryEventKind kind) {
  switch (kind) {
  case RouterTelemetryEventKind::routing_started:
    return "RoutingStarted";
  case RouterTelemetryEventKind::routing_finished:
    return "RoutingFinished";
  case RouterTelemetryEventKind::global_routing_started:
    return "GlobalRoutingStarted";
  case RouterTelemetryEventKind::global_routing_finished:
    return "GlobalRoutingFinished";
  case RouterTelemetryEventKind::pathfinder_iteration_started:
    return "PathFinderIterationStarted";
  case RouterTelemetryEventKind::pathfinder_iteration_completed:
    return "PathFinderIterationCompleted";
  case RouterTelemetryEventKind::best_checkpoint_updated:
    return "BestCheckpointUpdated";
  case RouterTelemetryEventKind::net_queued:
    return "NetQueued";
  case RouterTelemetryEventKind::net_started:
    return "NetStarted";
  case RouterTelemetryEventKind::net_completed:
    return "NetCompleted";
  case RouterTelemetryEventKind::net_failed:
    return "NetFailed";
  case RouterTelemetryEventKind::route_candidate_selected:
    return "RouteCandidateSelected";
  case RouterTelemetryEventKind::route_committed:
    return "RouteCommitted";
  case RouterTelemetryEventKind::route_removed:
    return "RouteRemoved";
  case RouterTelemetryEventKind::route_ripped_up:
    return "RouteRippedUp";
  case RouterTelemetryEventKind::via_candidate_generated:
    return "ViaCandidateGenerated";
  case RouterTelemetryEventKind::via_candidate_rejected:
    return "ViaCandidateRejected";
  case RouterTelemetryEventKind::via_assigned:
    return "ViaAssigned";
  case RouterTelemetryEventKind::via_removed:
    return "ViaRemoved";
  case RouterTelemetryEventKind::conflict_added:
    return "ConflictAdded";
  case RouterTelemetryEventKind::conflict_removed:
    return "ConflictRemoved";
  case RouterTelemetryEventKind::hydraulic_field_updated:
    return "HydraulicFieldUpdated";
  case RouterTelemetryEventKind::residual_group_updated:
    return "ResidualGroupUpdated";
  case RouterTelemetryEventKind::checkpoint_selected:
    return "CheckpointSelected";
  case RouterTelemetryEventKind::search_summary:
    return "SearchSummary";
  case RouterTelemetryEventKind::placement_proposal_started:
    return "PlacementProposalStarted";
  case RouterTelemetryEventKind::placement_proposal_accepted:
    return "PlacementProposalAccepted";
  case RouterTelemetryEventKind::placement_proposal_rejected:
    return "PlacementProposalRejected";
  }
  return "Unknown";
}

std::string serialize_router_telemetry_event(const RouterTelemetryEvent& event) {
  return telemetry::encode_event(canonical_event(event, RouterTelemetryLevel::normal, 0U));
}

std::string serialize_router_telemetry_snapshot(const RouterTelemetryState& state,
                                                const RouterTelemetryStatistics& statistics) {
  return telemetry::encode_event(
      canonical_snapshot(state, statistics, RouterTelemetryLevel::normal, 0U, true));
}

class RouterTelemetryService::Impl {
public:
  explicit Impl(RouterTelemetryConfig config) : config_(std::move(config)) {
    if (config_.queue_capacity == 0U || config_.websocket_session_queue_capacity == 0U ||
        config_.publish_rate_hz == 0U) {
      throw std::invalid_argument("router telemetry bounds and publish rate must be positive");
    }
    static_assert(std::is_nothrow_move_constructible_v<RouterTelemetryEvent>);
    queue_ = std::make_unique<QueueCell[]>(config_.queue_capacity);
    for (std::size_t index = 0U; index < config_.queue_capacity; ++index) {
      queue_[index].sequence.store(index, std::memory_order_relaxed);
    }
    if (config_.record_path) {
      record_.open(*config_.record_path, std::ios::binary | std::ios::trunc);
      if (!record_) {
        recorder_failures_.fetch_add(1U, std::memory_order_relaxed);
      }
    }
    if (config_.websocket_enabled()) {
      try {
        websocket_ = std::make_unique<WebSocketTelemetryServer>(
            *config_.listen_address, *config_.listen_port,
            config_.websocket_session_queue_capacity);
      } catch (...) {
        transport_failures_.fetch_add(1U, std::memory_order_relaxed);
      }
    } else if (config_.listen_address || config_.listen_port) {
      transport_failures_.fetch_add(1U, std::memory_order_relaxed);
    }
    consumer_ = std::jthread([this](std::stop_token token) { consume(token); });
  }

  ~Impl() { stop(); }

  void stop() noexcept {
    if (stopping_.exchange(true, std::memory_order_acq_rel)) {
      return;
    }
    stopped_.store(true, std::memory_order_release);
    consumer_.request_stop();
    queue_condition_.notify_all();
    if (consumer_.joinable()) {
      consumer_.join();
    }
    if (websocket_) {
      websocket_->stop();
    }
  }

  [[nodiscard]] bool try_emit(RouterTelemetryEvent event) noexcept {
    if (static_cast<unsigned int>(event.minimum_level) > static_cast<unsigned int>(config_.level)) {
      return true;
    }
    event.sequence = next_sequence_.fetch_add(1U, std::memory_order_relaxed) + 1U;
    if (stopped_.load(std::memory_order_relaxed)) {
      dropped_.fetch_add(1U, std::memory_order_relaxed);
      return false;
    }
    auto position = enqueue_position_.load(std::memory_order_relaxed);
    QueueCell* cell = nullptr;
    while (true) {
      if (position - dequeue_position_.load(std::memory_order_acquire) >= config_.queue_capacity) {
        dropped_.fetch_add(1U, std::memory_order_relaxed);
        return false;
      }
      cell = &queue_[position % config_.queue_capacity];
      const auto sequence = cell->sequence.load(std::memory_order_acquire);
      const auto difference =
          static_cast<std::intptr_t>(sequence) - static_cast<std::intptr_t>(position);
      if (difference == 0) {
        if (enqueue_position_.compare_exchange_weak(position, position + 1U,
                                                    std::memory_order_relaxed)) {
          break;
        }
      } else if (difference < 0) {
        dropped_.fetch_add(1U, std::memory_order_relaxed);
        return false;
      } else {
        position = enqueue_position_.load(std::memory_order_relaxed);
      }
    }
    cell->event.emplace(std::move(event));
    cell->sequence.store(position + 1U, std::memory_order_release);
    produced_.fetch_add(1U, std::memory_order_relaxed);
    queue_condition_.notify_one();
    return true;
  }

  [[nodiscard]] RouterTelemetryState state() const {
    std::lock_guard lock(state_mutex_);
    return state_;
  }

  [[nodiscard]] RouterTelemetryStatistics statistics() const {
    RouterTelemetryStatistics result;
    result.produced = produced_.load(std::memory_order_relaxed);
    result.consumed = consumed_.load(std::memory_order_relaxed);
    result.dropped = dropped_.load(std::memory_order_relaxed);
    result.coalesced = coalesced_.load(std::memory_order_relaxed);
    result.snapshots = snapshots_.load(std::memory_order_relaxed);
    result.websocket_messages = websocket_ ? websocket_->published() : 0U;
    result.websocket_dropped = websocket_ ? websocket_->dropped() : 0U;
    result.transport_failures = transport_failures_.load(std::memory_order_relaxed);
    result.recorder_failures = recorder_failures_.load(std::memory_order_relaxed);
    return result;
  }

  [[nodiscard]] const RouterTelemetryConfig& config() const { return config_; }

  [[nodiscard]] bool accepts(RouterTelemetryLevel level) const noexcept {
    return static_cast<unsigned int>(level) <= static_cast<unsigned int>(config_.level);
  }

  void publish_source_board(std::shared_ptr<const telemetry::SourceBoard> board) {
    if (!board) {
      return;
    }
    const auto source_identity = board->revision;
    bool first_source = false;
    {
      std::lock_guard lock(state_mutex_);
      first_source = state_.source_boards.empty();
      const auto found =
          std::find_if(state_.source_boards.begin(), state_.source_boards.end(),
                       [&](const auto& existing) { return existing && existing->id == board->id; });
      if (found == state_.source_boards.end()) {
        state_.source_boards.push_back(std::move(board));
      } else {
        *found = std::move(board);
      }
      state_.source_state_included = true;
    }
    if (first_source) {
      RouterTelemetryEvent started;
      started.kind = RouterTelemetryEventKind::routing_started;
      started.detail = source_identity;
      static_cast<void>(try_emit(std::move(started)));
    }
    source_snapshot_requested_.store(true, std::memory_order_release);
    queue_condition_.notify_one();
  }

private:
  struct QueueCell {
    std::atomic_size_t sequence{0U};
    std::optional<RouterTelemetryEvent> event;
  };

  [[nodiscard]] std::optional<RouterTelemetryEvent> try_pop() noexcept {
    const auto position = dequeue_position_.load(std::memory_order_relaxed);
    auto& cell = queue_[position % config_.queue_capacity];
    const auto sequence = cell.sequence.load(std::memory_order_acquire);
    const auto difference =
        static_cast<std::intptr_t>(sequence) - static_cast<std::intptr_t>(position + 1U);
    if (difference != 0) {
      return std::nullopt;
    }
    auto event = std::move(cell.event);
    cell.event.reset();
    cell.sequence.store(position + config_.queue_capacity, std::memory_order_release);
    dequeue_position_.store(position + 1U, std::memory_order_release);
    return event;
  }

  [[nodiscard]] bool queue_empty() const noexcept {
    return dequeue_position_.load(std::memory_order_relaxed) ==
           enqueue_position_.load(std::memory_order_relaxed);
  }

  void update_state(const RouterTelemetryEvent& event) {
    std::lock_guard lock(state_mutex_);
    state_.sequence = event.sequence;
    const auto summary_event =
        event.kind == RouterTelemetryEventKind::routing_started ||
        event.kind == RouterTelemetryEventKind::routing_finished ||
        event.kind == RouterTelemetryEventKind::global_routing_started ||
        event.kind == RouterTelemetryEventKind::global_routing_finished ||
        event.kind == RouterTelemetryEventKind::pathfinder_iteration_started ||
        event.kind == RouterTelemetryEventKind::pathfinder_iteration_completed ||
        event.kind == RouterTelemetryEventKind::best_checkpoint_updated ||
        event.kind == RouterTelemetryEventKind::checkpoint_selected;
    if (summary_event) {
      state_.pathfinder_iteration = event.iteration;
      state_.total_nets = event.total_nets == 0U ? state_.total_nets : event.total_nets;
      state_.routed_nets = event.routed_nets;
      state_.unrouted_nets = event.unrouted_nets;
      state_.sampled_conflicts = event.sampled_overflow;
      state_.corridor_conflicts = event.corridor_overflow;
      state_.clearance_shadow_conflicts = event.clearance_shadow_conflicts;
      state_.ripup_count = event.ripup_count;
      state_.physical_overflow_nm = event.physical_overflow_nm;
      state_.route_cost = event.route_cost;
      state_.search_expansions = event.search_expansions;
    }
    if (event.kind == RouterTelemetryEventKind::residual_group_updated) {
      state_.total_groups = event.total_groups;
      state_.regional_conflicts = event.regional_conflicts;
    }
    if (!event.net.empty()) {
      state_.active_net = event.net;
    }
    constexpr std::size_t maximum_observed_entities = 4'096U;
    const auto upsert = [&](auto& entities, const auto& same_key) {
      const auto found = std::find_if(entities.begin(), entities.end(), same_key);
      if (found != entities.end()) {
        *found = event;
        coalesced_.fetch_add(1U, std::memory_order_relaxed);
        return;
      }
      if (entities.size() == maximum_observed_entities) {
        entities.erase(entities.begin());
        coalesced_.fetch_add(1U, std::memory_order_relaxed);
      }
      entities.push_back(event);
    };
    if (event.kind == RouterTelemetryEventKind::net_queued && !event.net.empty()) {
      state_.dynamic_state_included = true;
      upsert(state_.observed_priorities, [&](const auto& existing) {
        return existing.net == event.net && existing.stable_id == event.stable_id;
      });
    }
    if (event.kind == RouterTelemetryEventKind::route_removed) {
      state_.dynamic_state_included = true;
      std::erase_if(state_.observed_routes,
                    [&](const auto& existing) { return existing.stable_id == event.stable_id; });
    } else if (event.kind == RouterTelemetryEventKind::route_candidate_selected ||
               event.kind == RouterTelemetryEventKind::route_committed ||
               event.kind == RouterTelemetryEventKind::route_ripped_up) {
      state_.dynamic_state_included = true;
      upsert(state_.observed_routes,
             [&](const auto& existing) { return existing.stable_id == event.stable_id; });
    }
    if (event.kind == RouterTelemetryEventKind::via_removed) {
      state_.dynamic_state_included = true;
      std::erase_if(state_.observed_vias,
                    [&](const auto& existing) { return existing.stable_id == event.stable_id; });
    } else if (event.kind == RouterTelemetryEventKind::via_candidate_generated ||
               event.kind == RouterTelemetryEventKind::via_candidate_rejected ||
               event.kind == RouterTelemetryEventKind::via_assigned) {
      state_.dynamic_state_included = true;
      upsert(state_.observed_vias,
             [&](const auto& existing) { return existing.stable_id == event.stable_id; });
    }
    if (event.kind == RouterTelemetryEventKind::conflict_removed) {
      state_.dynamic_state_included = true;
      std::erase_if(state_.observed_conflicts,
                    [&](const auto& existing) { return existing.stable_id == event.stable_id; });
    } else if (event.kind == RouterTelemetryEventKind::conflict_added) {
      state_.dynamic_state_included = true;
      upsert(state_.observed_conflicts,
             [&](const auto& existing) { return existing.stable_id == event.stable_id; });
    }
    if (event.kind == RouterTelemetryEventKind::hydraulic_field_updated) {
      state_.dynamic_state_included = true;
      upsert(state_.observed_hydraulic_fields,
             [&](const auto& existing) { return existing.stable_id == event.stable_id; });
    }
    if (event.kind == RouterTelemetryEventKind::via_candidate_generated ||
        event.kind == RouterTelemetryEventKind::via_candidate_rejected ||
        event.kind == RouterTelemetryEventKind::via_assigned ||
        event.kind == RouterTelemetryEventKind::via_removed) {
      state_.via_count = static_cast<std::size_t>(std::count_if(
          state_.observed_vias.begin(), state_.observed_vias.end(), [](const auto& observed) {
            return observed.kind == RouterTelemetryEventKind::via_assigned;
          }));
    }
    if (event.kind == RouterTelemetryEventKind::conflict_added ||
        event.kind == RouterTelemetryEventKind::conflict_removed) {
      state_.regional_conflicts = state_.observed_conflicts.size();
    }
    state_.active_conflicts = state_.sampled_conflicts + state_.corridor_conflicts +
                              state_.clearance_shadow_conflicts + state_.regional_conflicts;
    if (event.kind == RouterTelemetryEventKind::routing_finished) {
      state_.routing_finished = true;
      state_.active_net = NetId{};
    } else if (event.kind == RouterTelemetryEventKind::routing_started) {
      state_.source_identity = event.detail;
      state_.routing_finished = false;
      state_.dynamic_state_included = true;
      state_.observed_priorities.clear();
      state_.observed_routes.clear();
      state_.observed_vias.clear();
      state_.observed_conflicts.clear();
      state_.observed_hydraulic_fields.clear();
      state_.via_count = 0U;
      state_.regional_conflicts = 0U;
      state_.total_groups = 0U;
      state_.active_conflicts =
          state_.sampled_conflicts + state_.corridor_conflicts + state_.clearance_shadow_conflicts;
    }
  }

  void publish(std::string payload, bool snapshot, bool source_snapshot = false) {
    if (record_) {
      record_ << payload << '\n';
      if (!record_) {
        recorder_failures_.fetch_add(1U, std::memory_order_relaxed);
        record_.close();
      }
    }
    if (websocket_ && !websocket_->publish(std::move(payload), snapshot, source_snapshot)) {
      // The WebSocket server owns its more precise dropped counter. This aggregate only records
      // transport setup/runtime failures that make the publisher unavailable.
    }
  }

  void publish_snapshot(bool include_source = false) {
    snapshots_.fetch_add(1U, std::memory_order_relaxed);
    publish(telemetry::encode_event(canonical_snapshot(state(), statistics(), config_.level,
                                                       elapsed_microseconds(), include_source)),
            true, include_source);
  }

  [[nodiscard]] std::uint64_t elapsed_microseconds() const {
    return static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::microseconds>(
                                          std::chrono::steady_clock::now() - started_at_)
                                          .count());
  }

  void consume(std::stop_token token) {
    const auto period = std::chrono::milliseconds{
        std::max<std::int64_t>(1, 1000 / static_cast<std::int64_t>(config_.publish_rate_hz))};
    auto next_snapshot = std::chrono::steady_clock::now();
    while (true) {
      std::deque<RouterTelemetryEvent> events;
      {
        std::unique_lock lock(wait_mutex_);
        queue_condition_.wait_for(lock, period, [&]() {
          return token.stop_requested() || !queue_empty() ||
                 source_snapshot_requested_.load(std::memory_order_acquire);
        });
      }
      const auto source_snapshot =
          source_snapshot_requested_.exchange(false, std::memory_order_acq_rel);
      while (auto event = try_pop()) {
        events.push_back(std::move(*event));
      }
      for (const auto& event : events) {
        try {
          update_state(event);
          publish(telemetry::encode_event(
                      canonical_event(event, config_.level, elapsed_microseconds())),
                  false);
        } catch (...) {
          transport_failures_.fetch_add(1U, std::memory_order_relaxed);
        }
        consumed_.fetch_add(1U, std::memory_order_relaxed);
      }
      const auto now = std::chrono::steady_clock::now();
      const auto force_snapshot = std::any_of(events.begin(), events.end(), [](const auto& event) {
        return event.kind == RouterTelemetryEventKind::routing_finished ||
               event.kind == RouterTelemetryEventKind::global_routing_finished ||
               event.kind == RouterTelemetryEventKind::best_checkpoint_updated ||
               event.kind == RouterTelemetryEventKind::residual_group_updated;
      });
      const auto snapshot_worthy = std::any_of(events.begin(), events.end(), [](const auto& event) {
        // SearchSummary is an activity-only observation. It changes no reconstructable
        // routing entity, so waking the consumer for spline progress must not amplify into a
        // full recovery snapshot for every regional assignment.
        return event.kind != RouterTelemetryEventKind::search_summary;
      });
      if (source_snapshot || (snapshot_worthy && (now >= next_snapshot || force_snapshot))) {
        try {
          publish_snapshot(source_snapshot);
        } catch (...) {
          transport_failures_.fetch_add(1U, std::memory_order_relaxed);
        }
        next_snapshot = now + period;
      }
      if (token.stop_requested()) {
        if (queue_empty()) {
          break;
        }
      }
    }
    // A rate-limited snapshot may precede the final burst of regional/detailed events. Publish one
    // post-drain recovery point so a trace never requires those events to be lossless in order to
    // reconstruct the final observed state.
    try {
      {
        std::lock_guard lock(state_mutex_);
        state_.routing_finished = true;
        state_.active_net = NetId{};
      }
      publish_snapshot();
    } catch (...) {
      transport_failures_.fetch_add(1U, std::memory_order_relaxed);
    }
  }

  RouterTelemetryConfig config_;
  std::unique_ptr<QueueCell[]> queue_;
  std::atomic_size_t enqueue_position_{0U};
  std::atomic_size_t dequeue_position_{0U};
  std::mutex wait_mutex_;
  std::condition_variable queue_condition_;
  mutable std::mutex state_mutex_;
  RouterTelemetryState state_;
  std::ofstream record_;
  std::unique_ptr<WebSocketTelemetryServer> websocket_;
  std::jthread consumer_;
  std::atomic_uint64_t next_sequence_{0};
  std::atomic_uint64_t produced_{0};
  std::atomic_uint64_t consumed_{0};
  std::atomic_uint64_t dropped_{0};
  std::atomic_uint64_t coalesced_{0};
  std::atomic_uint64_t snapshots_{0};
  std::atomic_uint64_t transport_failures_{0};
  std::atomic_uint64_t recorder_failures_{0};
  std::atomic_bool stopped_{false};
  std::atomic_bool stopping_{false};
  std::atomic_bool source_snapshot_requested_{false};
  const std::chrono::steady_clock::time_point started_at_{std::chrono::steady_clock::now()};
};

RouterTelemetryService::RouterTelemetryService(RouterTelemetryConfig config)
    : impl_(std::make_unique<Impl>(std::move(config))) {}

RouterTelemetryService::~RouterTelemetryService() = default;

bool RouterTelemetryService::accepts(RouterTelemetryLevel level) const noexcept {
  return impl_->accepts(level);
}

bool RouterTelemetryService::try_emit(RouterTelemetryEvent event) noexcept {
  return impl_->try_emit(std::move(event));
}

void RouterTelemetryService::publish_source_board(const Board& board, std::string source_path) {
  impl_->publish_source_board(make_source_board_scene(board, std::move(source_path)));
}

void RouterTelemetryService::stop() noexcept { impl_->stop(); }

RouterTelemetryState RouterTelemetryService::state() const { return impl_->state(); }

RouterTelemetryStatistics RouterTelemetryService::statistics() const { return impl_->statistics(); }

const RouterTelemetryConfig& RouterTelemetryService::config() const { return impl_->config(); }

} // namespace flux
