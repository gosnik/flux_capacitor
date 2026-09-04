#include "flux/telemetry/event.hpp"

#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>
#include <utility>

namespace flux::telemetry {
namespace {

using boost::property_tree::ptree;

template <typename T> T get_or(const ptree& tree, const char* name, T fallback = {}) {
  return tree.get<T>(name, std::move(fallback));
}

std::string escape_json(std::string_view value) {
  std::ostringstream out;
  for (const char character : value) {
    const auto ch = static_cast<unsigned char>(character);
    switch (ch) {
    case '"':
      out << "\\\"";
      break;
    case '\\':
      out << "\\\\";
      break;
    case '\b':
      out << "\\b";
      break;
    case '\f':
      out << "\\f";
      break;
    case '\n':
      out << "\\n";
      break;
    case '\r':
      out << "\\r";
      break;
    case '\t':
      out << "\\t";
      break;
    default:
      if (ch < 0x20U) {
        out << "\\u" << std::hex << std::setw(4) << std::setfill('0')
            << static_cast<unsigned int>(ch) << std::dec;
      } else {
        out << static_cast<char>(ch);
      }
    }
  }
  return out.str();
}

void write_string(std::ostringstream& out, std::string_view name, std::string_view value,
                  bool& first) {
  if (!first) {
    out << ',';
  }
  first = false;
  out << '"' << name << "\":\"" << escape_json(value) << '"';
}

template <typename T>
void write_number(std::ostringstream& out, std::string_view name, T value, bool& first) {
  if (!first) {
    out << ',';
  }
  first = false;
  out << '"' << name << "\":" << value;
}

Point parse_point(const ptree& tree) {
  return Point{get_or<std::int64_t>(tree, "x_nm"), get_or<std::int64_t>(tree, "y_nm"),
               get_or<std::string>(tree, "layer")};
}

Route parse_route(const ptree& tree) {
  Route route;
  route.id = get_or<std::string>(tree, "id");
  route.net = get_or<std::string>(tree, "net");
  route.revision = get_or<std::uint64_t>(tree, "revision");
  route.state = route_state_from_string(get_or<std::string>(tree, "state", "current"));
  route.width_nm = get_or<std::uint64_t>(tree, "width_nm");
  route.cost = get_or<std::uint64_t>(tree, "cost");
  route.family = get_or<std::string>(tree, "family");
  route.lifecycle = get_or<std::string>(tree, "lifecycle");
  route.geometry = route_geometry_from_string(get_or<std::string>(tree, "geometry", "polyline"));
  route.spline_degree = get_or<std::uint32_t>(tree, "spline_degree");
  route.topology_signature = get_or<std::uint64_t>(tree, "topology_signature");
  route.topology_gate_count = get_or<std::size_t>(tree, "topology_gate_count");
  route.topology_gate_sequence_legal =
      get_or<bool>(tree, "topology_gate_sequence_legal", false);
  if (const auto points = tree.get_child_optional("points")) {
    for (const auto& item : *points) {
      route.points.push_back(parse_point(item.second));
    }
  }
  if (const auto controls = tree.get_child_optional("control_points")) {
    for (const auto& item : *controls) {
      route.control_points.push_back(parse_point(item.second));
    }
  }
  return route;
}

Via parse_via(const ptree& tree) {
  Via via;
  via.id = get_or<std::string>(tree, "id");
  via.net = get_or<std::string>(tree, "net");
  via.revision = get_or<std::uint64_t>(tree, "revision");
  if (const auto position = tree.get_child_optional("position")) {
    via.position = parse_point(*position);
  }
  via.to_layer = get_or<std::string>(tree, "to_layer");
  via.diameter_nm = get_or<std::uint64_t>(tree, "diameter_nm");
  via.score = get_or<double>(tree, "score");
  via.clearance_margin_nm = get_or<std::int64_t>(tree, "clearance_margin_nm");
  via.competing_nets = get_or<std::size_t>(tree, "competing_nets");
  via.local_congestion = get_or<double>(tree, "local_congestion");
  via.remaining_alternatives = get_or<std::size_t>(tree, "remaining_alternatives");
  via.state = via_state_from_string(get_or<std::string>(tree, "state", "candidate"));
  via.rejection_reason = get_or<std::string>(tree, "rejection_reason");
  return via;
}

std::vector<std::string> parse_ids(const ptree& tree, const char* name) {
  std::vector<std::string> ids;
  if (const auto values = tree.get_child_optional(name)) {
    for (const auto& item : *values) {
      ids.push_back(item.second.get_value<std::string>());
    }
  }
  return ids;
}

Conflict parse_conflict(const ptree& tree) {
  Conflict conflict;
  conflict.id = get_or<std::string>(tree, "id");
  conflict.kind = conflict_kind_from_string(get_or<std::string>(tree, "kind", "unknown"));
  conflict.resource = get_or<std::string>(tree, "resource");
  conflict.group = get_or<std::string>(tree, "group");
  if (const auto location = tree.get_child_optional("location")) {
    conflict.location = parse_point(*location);
  }
  conflict.nets = parse_ids(tree, "nets");
  conflict.clearance_deficit_nm = get_or<std::int64_t>(tree, "clearance_deficit_nm");
  conflict.pressure = get_or<double>(tree, "pressure");
  conflict.history = get_or<double>(tree, "history");
  return conflict;
}

ResourceOccupancy parse_resource(const ptree& tree) {
  ResourceOccupancy resource;
  resource.id = get_or<std::string>(tree, "id");
  resource.kind = resource_kind_from_string(get_or<std::string>(tree, "kind", "unknown"));
  resource.layer = get_or<std::string>(tree, "layer");
  if (const auto start = tree.get_child_optional("start")) {
    resource.start = parse_point(*start);
  }
  if (const auto end = tree.get_child_optional("end")) {
    resource.end = parse_point(*end);
  }
  resource.demand = get_or<double>(tree, "demand");
  resource.capacity = get_or<double>(tree, "capacity");
  resource.channel_width_nm = get_or<std::uint64_t>(tree, "channel_width_nm");
  resource.nets = parse_ids(tree, "nets");
  return resource;
}

HydraulicField parse_hydraulic_field(const ptree& tree) {
  HydraulicField field;
  field.id = get_or<std::string>(tree, "id");
  field.net = get_or<std::string>(tree, "net");
  field.layer = get_or<std::string>(tree, "layer");
  field.revision = get_or<std::uint64_t>(tree, "revision");
  field.origin_x_nm = get_or<std::int64_t>(tree, "origin_x_nm");
  field.origin_y_nm = get_or<std::int64_t>(tree, "origin_y_nm");
  field.cell_pitch_nm = get_or<std::uint64_t>(tree, "cell_pitch_nm");
  field.width = get_or<std::size_t>(tree, "width");
  field.height = get_or<std::size_t>(tree, "height");
  field.stride = get_or<std::size_t>(tree, "stride");
  field.maximum_flow = get_or<std::uint32_t>(tree, "maximum_flow");
  field.maximum_erosion = get_or<std::uint32_t>(tree, "maximum_erosion");
  field.maximum_terrain_height = get_or<std::uint32_t>(tree, "maximum_terrain_height");
  field.maximum_downstream_pressure =
      get_or<std::uint32_t>(tree, "maximum_downstream_pressure");
  field.maximum_success_prior = get_or<std::uint32_t>(tree, "maximum_success_prior");
  const auto parse_values = [&](const char* name, std::vector<std::uint16_t>& values) {
    if (const auto child = tree.get_child_optional(name)) {
      for (const auto& item : *child) {
        values.push_back(item.second.get_value<std::uint16_t>());
      }
    }
  };
  parse_values("flow", field.flow);
  parse_values("erosion", field.erosion);
  parse_values("heightmap", field.heightmap);
  parse_values("free_space", field.free_space);
  parse_values("downstream_pressure", field.downstream_pressure);
  parse_values("success_prior", field.success_prior);
  return field;
}

Priority parse_priority(const ptree& tree) {
  Priority priority;
  priority.net = get_or<std::string>(tree, "net");
  priority.group = get_or<std::string>(tree, "group");
  priority.staged_class = get_or<std::string>(tree, "staged_class");
  priority.explicit_priority = get_or<double>(tree, "explicit_priority");
  priority.terminal_scarcity = get_or<double>(tree, "terminal_scarcity");
  priority.corridor_scarcity = get_or<double>(tree, "corridor_scarcity");
  priority.via_scarcity = get_or<double>(tree, "via_scarcity");
  priority.routing_interference = get_or<double>(tree, "routing_interference");
  priority.physical_width_demand = get_or<double>(tree, "physical_width_demand");
  priority.topology_alternatives = get_or<std::size_t>(tree, "topology_alternatives");
  priority.layer_alternatives = get_or<std::size_t>(tree, "layer_alternatives");
  priority.final_priority = get_or<double>(tree, "final_priority");
  priority.stable_tie_break = get_or<std::uint64_t>(tree, "stable_tie_break");
  return priority;
}

SourceLayer parse_source_layer(const ptree& tree) {
  return SourceLayer{get_or<std::string>(tree, "id"), get_or<std::string>(tree, "name"),
                     get_or<std::string>(tree, "kind"), get_or<std::uint32_t>(tree, "stack_order")};
}

SourceNet parse_source_net(const ptree& tree) {
  return SourceNet{get_or<std::string>(tree, "id"), get_or<std::string>(tree, "name"),
                   get_or<std::uint32_t>(tree, "priority")};
}

SourcePrimitive parse_source_primitive(const ptree& tree) {
  SourcePrimitive primitive;
  primitive.id = get_or<std::string>(tree, "id");
  primitive.kind = get_or<std::string>(tree, "kind");
  primitive.parent = get_or<std::string>(tree, "parent");
  primitive.reference = get_or<std::string>(tree, "reference");
  primitive.net = get_or<std::string>(tree, "net");
  primitive.layers = parse_ids(tree, "layers");
  if (const auto points = tree.get_child_optional("points")) {
    for (const auto& item : *points) {
      primitive.points.push_back(parse_point(item.second));
    }
  }
  if (const auto holes = tree.get_child_optional("holes")) {
    for (const auto& hole : *holes) {
      std::vector<Point> points;
      for (const auto& item : hole.second) {
        points.push_back(parse_point(item.second));
      }
      primitive.holes.push_back(std::move(points));
    }
  }
  primitive.width_nm = get_or<std::uint64_t>(tree, "width_nm");
  primitive.height_nm = get_or<std::uint64_t>(tree, "height_nm");
  primitive.diameter_nm = get_or<std::uint64_t>(tree, "diameter_nm");
  primitive.drill_nm = get_or<std::uint64_t>(tree, "drill_nm");
  primitive.rotation_microdegrees = get_or<std::int64_t>(tree, "rotation_microdegrees");
  primitive.shape = get_or<std::string>(tree, "shape");
  primitive.side = get_or<std::string>(tree, "side");
  primitive.locked = get_or<bool>(tree, "locked");
  primitive.filled = get_or<bool>(tree, "filled");
  return primitive;
}

std::shared_ptr<const SourceBoard> parse_source_board(const ptree& tree) {
  auto board = std::make_shared<SourceBoard>();
  board->id = get_or<std::string>(tree, "id");
  board->path = get_or<std::string>(tree, "path");
  board->revision = get_or<std::string>(tree, "revision");
  board->board_schema = get_or<std::uint32_t>(tree, "board_schema");
  if (const auto layers = tree.get_child_optional("layers")) {
    for (const auto& item : *layers) {
      board->layers.push_back(parse_source_layer(item.second));
    }
  }
  if (const auto nets = tree.get_child_optional("nets")) {
    for (const auto& item : *nets) {
      board->nets.push_back(parse_source_net(item.second));
    }
  }
  if (const auto primitives = tree.get_child_optional("primitives")) {
    for (const auto& item : *primitives) {
      board->primitives.push_back(parse_source_primitive(item.second));
    }
  }
  return board;
}

IterationMetrics parse_metrics(const ptree& tree) {
  IterationMetrics metrics;
  metrics.iteration = get_or<std::size_t>(tree, "iteration");
  metrics.total_nets = get_or<std::size_t>(tree, "total_nets");
  metrics.routed_nets = get_or<std::size_t>(tree, "routed_nets");
  metrics.unrouted_nets = get_or<std::size_t>(tree, "unrouted_nets");
  metrics.total_groups = get_or<std::size_t>(tree, "total_groups");
  metrics.routed_groups = get_or<std::size_t>(tree, "routed_groups");
  metrics.unrouted_groups = get_or<std::size_t>(tree, "unrouted_groups");
  metrics.sampled_overflow = get_or<std::size_t>(tree, "sampled_overflow");
  metrics.corridor_overflow = get_or<std::size_t>(tree, "corridor_overflow");
  metrics.physical_overflow_nm = get_or<std::uint64_t>(tree, "physical_overflow_nm");
  metrics.clearance_shadow_conflicts = get_or<std::size_t>(tree, "clearance_shadow_conflicts");
  metrics.regional_conflicts = get_or<std::size_t>(tree, "regional_conflicts");
  metrics.route_cost = get_or<std::uint64_t>(tree, "route_cost");
  metrics.vias = get_or<std::size_t>(tree, "vias");
  metrics.rip_ups = get_or<std::size_t>(tree, "rip_ups");
  metrics.search_expansions = get_or<std::size_t>(tree, "search_expansions");
  metrics.selected_checkpoint = get_or<std::uint64_t>(tree, "selected_checkpoint");
  metrics.best_checkpoint = get_or<std::uint64_t>(tree, "best_checkpoint");
  metrics.selected_is_best = get_or<bool>(tree, "selected_is_best");
  return metrics;
}

Counters parse_counters(const ptree& tree) {
  return Counters{get_or<std::uint64_t>(tree, "produced"), get_or<std::uint64_t>(tree, "dropped"),
                  get_or<std::uint64_t>(tree, "coalesced"),
                  get_or<std::uint64_t>(tree, "serialized"),
                  get_or<std::uint64_t>(tree, "snapshots")};
}

Event parse_event(const ptree& tree) {
  const std::string protocol = get_or<std::string>(tree, "protocol");
  if (!protocol.empty() && protocol != protocol_name) {
    throw std::runtime_error("unsupported telemetry protocol '" + protocol + "'");
  }

  Event event;
  event.schema = get_or<std::uint32_t>(tree, "schema", telemetry_schema_version);
  if (event.schema != telemetry_schema_version) {
    throw std::runtime_error("unsupported telemetry schema " + std::to_string(event.schema));
  }
  event.sequence = get_or<std::uint64_t>(tree, "sequence");
  event.monotonic_time_us = get_or<std::uint64_t>(tree, "monotonic_time_us");
  event.level = level_from_string(get_or<std::string>(tree, "level", "normal"));
  const std::string record_type = get_or<std::string>(tree, "type", "unknown");
  event.type = record_type == "snapshot"
                   ? EventType::snapshot
                   : event_type_from_string(get_or<std::string>(tree, "kind", record_type));
  event.source_identity = get_or<std::string>(tree, "source_identity");
  event.configuration_revision = get_or<std::uint64_t>(tree, "configuration_revision");
  if (const auto value = tree.get_optional<std::string>("active_net")) {
    event.active_net = *value;
  }
  if (const auto value = tree.get_optional<std::string>("active_group")) {
    event.active_group = *value;
  }
  if (!event.active_net) {
    if (const auto value = tree.get_optional<std::string>("net"); value && !value->empty()) {
      event.active_net = *value;
    }
  }

  const auto parse_array = [&tree](const char* name, auto& output, const auto& parser) {
    if (const auto child = tree.get_child_optional(name)) {
      for (const auto& item : *child) {
        output.push_back(parser(item.second));
      }
    }
  };
  event.dynamic_state_included = tree.get_child_optional("routes").has_value() ||
                                 tree.get_child_optional("vias").has_value() ||
                                 tree.get_child_optional("conflicts").has_value() ||
                                 tree.get_child_optional("resources").has_value() ||
                                 tree.get_child_optional("hydraulic_fields").has_value() ||
                                 tree.get_child_optional("priorities").has_value();
  event.source_state_included = tree.get_child_optional("source_boards").has_value();
  if (const auto boards = tree.get_child_optional("source_boards")) {
    for (const auto& item : *boards) {
      event.source_boards.push_back(parse_source_board(item.second));
    }
  }
  parse_array("routes", event.routes, parse_route);
  parse_array("vias", event.vias, parse_via);
  parse_array("conflicts", event.conflicts, parse_conflict);
  parse_array("resources", event.resources, parse_resource);
  parse_array("hydraulic_fields", event.hydraulic_fields, parse_hydraulic_field);
  parse_array("priorities", event.priorities, parse_priority);

  if (const auto metrics = tree.get_child_optional("metrics")) {
    event.metrics = parse_metrics(*metrics);
  } else if (event.type == EventType::snapshot ||
             event.type == EventType::pathfinder_iteration_started ||
             event.type == EventType::pathfinder_iteration_completed ||
             event.type == EventType::best_checkpoint_updated ||
             event.type == EventType::routing_finished) {
    event.metrics = parse_metrics(tree);
    event.metrics->vias = get_or<std::size_t>(tree, "via_count", event.metrics->vias);
    event.metrics->rip_ups = get_or<std::size_t>(tree, "ripup_count", event.metrics->rip_ups);
  }
  if (const auto counters = tree.get_child_optional("counters")) {
    event.counters = parse_counters(*counters);
  } else if (event.type == EventType::snapshot) {
    event.counters =
        Counters{get_or<std::uint64_t>(tree, "produced"), get_or<std::uint64_t>(tree, "dropped"),
                 get_or<std::uint64_t>(tree, "coalesced"), get_or<std::uint64_t>(tree, "consumed"),
                 get_or<std::uint64_t>(tree, "snapshots")};
  }
  event.message = get_or<std::string>(tree, "message", get_or<std::string>(tree, "detail"));
  if (event.source_identity.empty() && event.type == EventType::routing_started) {
    event.source_identity = event.message;
  }
  return event;
}

ptree parse_tree(std::string_view text) {
  std::istringstream input{std::string(text)};
  ptree tree;
  boost::property_tree::read_json(input, tree);
  return tree;
}

void decode_tree(const ptree& tree, std::vector<Event>& events) {
  if (const auto batch = tree.get_child_optional("events")) {
    const std::string protocol = get_or<std::string>(tree, "protocol");
    if (!protocol.empty() && protocol != protocol_name) {
      throw std::runtime_error("unsupported telemetry protocol '" + protocol + "'");
    }
    for (const auto& item : *batch) {
      events.push_back(parse_event(item.second));
    }
    return;
  }
  events.push_back(parse_event(tree));
}

void write_point(std::ostringstream& out, const Point& point) {
  out << "{\"x_nm\":" << point.x_nm << ",\"y_nm\":" << point.y_nm << ",\"layer\":\""
      << escape_json(point.layer) << "\"}";
}

void write_ids(std::ostringstream& out, const std::vector<std::string>& ids) {
  out << '[';
  for (std::size_t index = 0; index < ids.size(); ++index) {
    if (index != 0U) {
      out << ',';
    }
    out << '"' << escape_json(ids[index]) << '"';
  }
  out << ']';
}

template <typename Range, typename Writer>
void write_array(std::ostringstream& out, std::string_view name, const Range& values, bool& first,
                 Writer writer) {
  if (values.empty()) {
    return;
  }
  if (!first) {
    out << ',';
  }
  first = false;
  out << '"' << name << "\":[";
  bool first_value = true;
  for (const auto& value : values) {
    if (!first_value) {
      out << ',';
    }
    first_value = false;
    writer(out, value);
  }
  out << ']';
}

void write_route(std::ostringstream& out, const Route& route) {
  out << "{\"id\":\"" << escape_json(route.id) << "\",\"net\":\"" << escape_json(route.net)
      << "\",\"revision\":" << route.revision << ",\"state\":\"" << to_string(route.state)
      << "\",\"width_nm\":" << route.width_nm << ",\"cost\":" << route.cost << ",\"geometry\":\""
      << to_string(route.geometry) << "\",\"family\":\"" << escape_json(route.family)
      << "\",\"lifecycle\":\"" << escape_json(route.lifecycle)
      << "\",\"spline_degree\":" << route.spline_degree
      << ",\"topology_signature\":" << route.topology_signature
      << ",\"topology_gate_count\":" << route.topology_gate_count
      << ",\"topology_gate_sequence_legal\":"
      << (route.topology_gate_sequence_legal ? "true" : "false")
      << ",\"points\":[";
  for (std::size_t index = 0; index < route.points.size(); ++index) {
    if (index != 0U) {
      out << ',';
    }
    write_point(out, route.points[index]);
  }
  out << "],\"control_points\":[";
  for (std::size_t index = 0; index < route.control_points.size(); ++index) {
    if (index != 0U) {
      out << ',';
    }
    write_point(out, route.control_points[index]);
  }
  out << "]}";
}

void write_via(std::ostringstream& out, const Via& via) {
  out << "{\"id\":\"" << escape_json(via.id) << "\",\"net\":\"" << escape_json(via.net)
      << "\",\"revision\":" << via.revision << ",\"position\":";
  write_point(out, via.position);
  out << ",\"to_layer\":\"" << escape_json(via.to_layer) << "\",\"diameter_nm\":" << via.diameter_nm
      << ",\"score\":" << via.score << ",\"clearance_margin_nm\":" << via.clearance_margin_nm
      << ",\"competing_nets\":" << via.competing_nets
      << ",\"local_congestion\":" << via.local_congestion
      << ",\"remaining_alternatives\":" << via.remaining_alternatives << ",\"state\":\""
      << to_string(via.state) << "\",\"rejection_reason\":\"" << escape_json(via.rejection_reason)
      << "\"}";
}

void write_conflict(std::ostringstream& out, const Conflict& conflict) {
  out << "{\"id\":\"" << escape_json(conflict.id) << "\",\"kind\":\"" << to_string(conflict.kind)
      << "\",\"resource\":\"" << escape_json(conflict.resource) << "\",\"group\":\""
      << escape_json(conflict.group) << "\",\"location\":";
  write_point(out, conflict.location);
  out << ",\"nets\":";
  write_ids(out, conflict.nets);
  out << ",\"clearance_deficit_nm\":" << conflict.clearance_deficit_nm
      << ",\"pressure\":" << conflict.pressure << ",\"history\":" << conflict.history << '}';
}

void write_resource(std::ostringstream& out, const ResourceOccupancy& resource) {
  out << "{\"id\":\"" << escape_json(resource.id) << "\",\"kind\":\"" << to_string(resource.kind)
      << "\",\"layer\":\"" << escape_json(resource.layer) << "\",\"start\":";
  write_point(out, resource.start);
  out << ",\"end\":";
  write_point(out, resource.end);
  out << ",\"demand\":" << resource.demand << ",\"capacity\":" << resource.capacity
      << ",\"channel_width_nm\":" << resource.channel_width_nm << ",\"nets\":";
  write_ids(out, resource.nets);
  out << '}';
}

void write_hydraulic_field(std::ostringstream& out, const HydraulicField& field) {
  const auto write_values = [&](const std::vector<std::uint16_t>& values) {
    out << '[';
    for (std::size_t index = 0U; index < values.size(); ++index) {
      if (index != 0U) out << ',';
      out << values[index];
    }
    out << ']';
  };
  out << "{\"id\":\"" << escape_json(field.id) << "\",\"net\":\""
      << escape_json(field.net) << "\",\"layer\":\"" << escape_json(field.layer)
      << "\",\"revision\":" << field.revision << ",\"origin_x_nm\":" << field.origin_x_nm
      << ",\"origin_y_nm\":" << field.origin_y_nm << ",\"cell_pitch_nm\":"
      << field.cell_pitch_nm << ",\"width\":" << field.width << ",\"height\":" << field.height
      << ",\"stride\":" << field.stride << ",\"maximum_flow\":" << field.maximum_flow
      << ",\"maximum_erosion\":" << field.maximum_erosion
      << ",\"maximum_terrain_height\":" << field.maximum_terrain_height
      << ",\"maximum_downstream_pressure\":" << field.maximum_downstream_pressure
      << ",\"maximum_success_prior\":" << field.maximum_success_prior << ",\"flow\":";
  write_values(field.flow);
  out << ",\"erosion\":";
  write_values(field.erosion);
  out << ",\"heightmap\":";
  write_values(field.heightmap);
  out << ",\"free_space\":";
  write_values(field.free_space);
  out << ",\"downstream_pressure\":";
  write_values(field.downstream_pressure);
  out << ",\"success_prior\":";
  write_values(field.success_prior);
  out << '}';
}

void write_priority(std::ostringstream& out, const Priority& priority) {
  out << "{\"net\":\"" << escape_json(priority.net) << "\",\"group\":\""
      << escape_json(priority.group) << "\",\"staged_class\":\""
      << escape_json(priority.staged_class)
      << "\",\"explicit_priority\":" << priority.explicit_priority
      << ",\"terminal_scarcity\":" << priority.terminal_scarcity
      << ",\"corridor_scarcity\":" << priority.corridor_scarcity
      << ",\"via_scarcity\":" << priority.via_scarcity
      << ",\"routing_interference\":" << priority.routing_interference
      << ",\"physical_width_demand\":" << priority.physical_width_demand
      << ",\"topology_alternatives\":" << priority.topology_alternatives
      << ",\"layer_alternatives\":" << priority.layer_alternatives
      << ",\"final_priority\":" << priority.final_priority
      << ",\"stable_tie_break\":" << priority.stable_tie_break << '}';
}

void write_source_layer(std::ostringstream& out, const SourceLayer& layer) {
  out << "{\"id\":\"" << escape_json(layer.id) << "\",\"name\":\"" << escape_json(layer.name)
      << "\",\"kind\":\"" << escape_json(layer.kind) << "\",\"stack_order\":" << layer.stack_order
      << '}';
}

void write_source_net(std::ostringstream& out, const SourceNet& net) {
  out << "{\"id\":\"" << escape_json(net.id) << "\",\"name\":\"" << escape_json(net.name)
      << "\",\"priority\":" << net.priority << '}';
}

void write_points(std::ostringstream& out, const std::vector<Point>& points) {
  out << '[';
  for (std::size_t index = 0U; index < points.size(); ++index) {
    if (index != 0U) {
      out << ',';
    }
    write_point(out, points[index]);
  }
  out << ']';
}

void write_source_primitive(std::ostringstream& out, const SourcePrimitive& primitive) {
  out << "{\"id\":\"" << escape_json(primitive.id) << "\",\"kind\":\""
      << escape_json(primitive.kind) << "\",\"parent\":\"" << escape_json(primitive.parent)
      << "\",\"reference\":\"" << escape_json(primitive.reference) << "\",\"net\":\""
      << escape_json(primitive.net) << "\",\"layers\":";
  write_ids(out, primitive.layers);
  out << ",\"points\":";
  write_points(out, primitive.points);
  out << ",\"holes\":[";
  for (std::size_t index = 0U; index < primitive.holes.size(); ++index) {
    if (index != 0U) {
      out << ',';
    }
    write_points(out, primitive.holes[index]);
  }
  out << "],\"width_nm\":" << primitive.width_nm << ",\"height_nm\":" << primitive.height_nm
      << ",\"diameter_nm\":" << primitive.diameter_nm << ",\"drill_nm\":" << primitive.drill_nm
      << ",\"rotation_microdegrees\":" << primitive.rotation_microdegrees << ",\"shape\":\""
      << escape_json(primitive.shape) << "\",\"side\":\"" << escape_json(primitive.side)
      << "\",\"locked\":" << (primitive.locked ? "true" : "false")
      << ",\"filled\":" << (primitive.filled ? "true" : "false") << '}';
}

void write_source_board(std::ostringstream& out, const std::shared_ptr<const SourceBoard>& board) {
  if (!board) {
    out << "{}";
    return;
  }
  out << "{\"id\":\"" << escape_json(board->id) << "\",\"path\":\"" << escape_json(board->path)
      << "\",\"revision\":\"" << escape_json(board->revision)
      << "\",\"board_schema\":" << board->board_schema << ",\"layers\":[";
  for (std::size_t index = 0U; index < board->layers.size(); ++index) {
    if (index != 0U) {
      out << ',';
    }
    write_source_layer(out, board->layers[index]);
  }
  out << "],\"nets\":[";
  for (std::size_t index = 0U; index < board->nets.size(); ++index) {
    if (index != 0U) {
      out << ',';
    }
    write_source_net(out, board->nets[index]);
  }
  out << "],\"primitives\":[";
  for (std::size_t index = 0U; index < board->primitives.size(); ++index) {
    if (index != 0U) {
      out << ',';
    }
    write_source_primitive(out, board->primitives[index]);
  }
  out << "]}";
}

void write_metrics(std::ostringstream& out, const IterationMetrics& metrics) {
  out << "{\"iteration\":" << metrics.iteration << ",\"total_nets\":" << metrics.total_nets
      << ",\"routed_nets\":" << metrics.routed_nets
      << ",\"unrouted_nets\":" << metrics.unrouted_nets
      << ",\"total_groups\":" << metrics.total_groups
      << ",\"routed_groups\":" << metrics.routed_groups
      << ",\"unrouted_groups\":" << metrics.unrouted_groups
      << ",\"sampled_overflow\":" << metrics.sampled_overflow
      << ",\"corridor_overflow\":" << metrics.corridor_overflow
      << ",\"physical_overflow_nm\":" << metrics.physical_overflow_nm
      << ",\"clearance_shadow_conflicts\":" << metrics.clearance_shadow_conflicts
      << ",\"regional_conflicts\":" << metrics.regional_conflicts
      << ",\"route_cost\":" << metrics.route_cost << ",\"vias\":" << metrics.vias
      << ",\"rip_ups\":" << metrics.rip_ups
      << ",\"search_expansions\":" << metrics.search_expansions
      << ",\"selected_checkpoint\":" << metrics.selected_checkpoint
      << ",\"best_checkpoint\":" << metrics.best_checkpoint
      << ",\"selected_is_best\":" << (metrics.selected_is_best ? "true" : "false") << '}';
}

std::string_view wire_kind_name(EventType type) {
  static constexpr std::array names{std::string_view{"Snapshot"},
                                    std::string_view{"RoutingStarted"},
                                    std::string_view{"RoutingFinished"},
                                    std::string_view{"NetQueued"},
                                    std::string_view{"NetStarted"},
                                    std::string_view{"NetCompleted"},
                                    std::string_view{"NetFailed"},
                                    std::string_view{"RouteCandidateSelected"},
                                    std::string_view{"RouteCommitted"},
                                    std::string_view{"RouteRemoved"},
                                    std::string_view{"RouteRippedUp"},
                                    std::string_view{"ViaCandidateGenerated"},
                                    std::string_view{"ViaCandidateRejected"},
                                    std::string_view{"ViaAssigned"},
                                    std::string_view{"ViaRemoved"},
                                    std::string_view{"PathFinderIterationStarted"},
                                    std::string_view{"PathFinderIterationCompleted"},
                                    std::string_view{"ConflictAdded"},
                                    std::string_view{"ConflictRemoved"},
                                    std::string_view{"ResourceOccupancyUpdated"},
                                    std::string_view{"HydraulicFieldUpdated"},
                                    std::string_view{"ResidualGroupUpdated"},
                                    std::string_view{"CheckpointSelected"},
                                    std::string_view{"BestCheckpointUpdated"},
                                    std::string_view{"PlacementProposalStarted"},
                                    std::string_view{"PlacementProposalAccepted"},
                                    std::string_view{"PlacementProposalRejected"},
                                    std::string_view{"SearchSummary"},
                                    std::string_view{"Unknown"}};
  const std::size_t index = static_cast<std::size_t>(type);
  return index < names.size() ? names[index] : std::string_view{"Unknown"};
}

} // namespace

double ResourceOccupancy::utilization() const {
  if (capacity <= 0.0) {
    return demand > 0.0 ? std::numeric_limits<double>::infinity() : 0.0;
  }
  return demand / capacity;
}

#define FLUX_ENUM_STRINGS(Name, ...)                                                               \
  std::string_view to_string(Name value) {                                                         \
    static constexpr std::array names{__VA_ARGS__};                                                \
    const auto index = static_cast<std::size_t>(value);                                            \
    return index < names.size() ? names[index] : std::string_view{"unknown"};                      \
  }

FLUX_ENUM_STRINGS(Level, "normal", "diagnostic", "trace")
FLUX_ENUM_STRINGS(EventType, "snapshot", "routing_started", "routing_finished", "net_queued",
                  "net_started", "net_completed", "net_failed", "route_candidate_selected",
                  "route_committed", "route_removed", "route_ripped_up", "via_candidate_generated",
                  "via_candidate_rejected", "via_assigned", "via_removed",
                  "pathfinder_iteration_started", "pathfinder_iteration_completed",
                  "conflict_added", "conflict_removed", "resource_occupancy_updated",
                  "hydraulic_field_updated", "residual_group_updated", "checkpoint_selected", "best_checkpoint_updated",
                  "placement_proposal_started", "placement_proposal_accepted",
                  "placement_proposal_rejected", "search_summary", "unknown")
FLUX_ENUM_STRINGS(RouteState, "current", "provisional", "committed", "ripped_up", "removed")
FLUX_ENUM_STRINGS(RouteGeometry, "polyline", "quintic_ph")
FLUX_ENUM_STRINGS(ViaState, "candidate", "selected", "contested", "rejected", "removed")
FLUX_ENUM_STRINGS(ConflictKind, "congestion", "clearance_shadow", "regional", "physical", "unknown")
FLUX_ENUM_STRINGS(ResourceKind, "sampled", "corridor", "physical", "via_site", "unknown")

#undef FLUX_ENUM_STRINGS

template <typename Enum, std::size_t Size>
Enum enum_from_string(std::string_view value, const std::array<std::string_view, Size>& names,
                      Enum fallback) {
  const auto found = std::find(names.begin(), names.end(), value);
  return found == names.end() ? fallback : static_cast<Enum>(std::distance(names.begin(), found));
}

Level level_from_string(std::string_view value) {
  return enum_from_string(value, std::array<std::string_view, 3>{"normal", "diagnostic", "trace"},
                          Level::normal);
}

EventType event_type_from_string(std::string_view value) {
  static constexpr std::array names{std::string_view{"snapshot"},
                                    std::string_view{"routing_started"},
                                    std::string_view{"routing_finished"},
                                    std::string_view{"net_queued"},
                                    std::string_view{"net_started"},
                                    std::string_view{"net_completed"},
                                    std::string_view{"net_failed"},
                                    std::string_view{"route_candidate_selected"},
                                    std::string_view{"route_committed"},
                                    std::string_view{"route_removed"},
                                    std::string_view{"route_ripped_up"},
                                    std::string_view{"via_candidate_generated"},
                                    std::string_view{"via_candidate_rejected"},
                                    std::string_view{"via_assigned"},
                                    std::string_view{"via_removed"},
                                    std::string_view{"pathfinder_iteration_started"},
                                    std::string_view{"pathfinder_iteration_completed"},
                                    std::string_view{"conflict_added"},
                                    std::string_view{"conflict_removed"},
                                    std::string_view{"resource_occupancy_updated"},
                                    std::string_view{"hydraulic_field_updated"},
                                    std::string_view{"residual_group_updated"},
                                    std::string_view{"checkpoint_selected"},
                                    std::string_view{"best_checkpoint_updated"},
                                    std::string_view{"placement_proposal_started"},
                                    std::string_view{"placement_proposal_accepted"},
                                    std::string_view{"placement_proposal_rejected"},
                                    std::string_view{"search_summary"},
                                    std::string_view{"unknown"}};
  EventType result = enum_from_string(value, names, EventType::unknown);
  if (result != EventType::unknown || value == "unknown") {
    return result;
  }
  static constexpr std::array wire_names{std::string_view{"Snapshot"},
                                         std::string_view{"RoutingStarted"},
                                         std::string_view{"RoutingFinished"},
                                         std::string_view{"NetQueued"},
                                         std::string_view{"NetStarted"},
                                         std::string_view{"NetCompleted"},
                                         std::string_view{"NetFailed"},
                                         std::string_view{"RouteCandidateSelected"},
                                         std::string_view{"RouteCommitted"},
                                         std::string_view{"RouteRemoved"},
                                         std::string_view{"RouteRippedUp"},
                                         std::string_view{"ViaCandidateGenerated"},
                                         std::string_view{"ViaCandidateRejected"},
                                         std::string_view{"ViaAssigned"},
                                         std::string_view{"ViaRemoved"},
                                         std::string_view{"PathFinderIterationStarted"},
                                         std::string_view{"PathFinderIterationCompleted"},
                                         std::string_view{"ConflictAdded"},
                                         std::string_view{"ConflictRemoved"},
                                         std::string_view{"ResourceOccupancyUpdated"},
                                         std::string_view{"HydraulicFieldUpdated"},
                                         std::string_view{"ResidualGroupUpdated"},
                                         std::string_view{"CheckpointSelected"},
                                         std::string_view{"BestCheckpointUpdated"},
                                         std::string_view{"PlacementProposalStarted"},
                                         std::string_view{"PlacementProposalAccepted"},
                                         std::string_view{"PlacementProposalRejected"},
                                         std::string_view{"SearchSummary"},
                                         std::string_view{"Unknown"}};
  return enum_from_string(value, wire_names, EventType::unknown);
}

RouteState route_state_from_string(std::string_view value) {
  return enum_from_string(value,
                          std::array<std::string_view, 5>{"current", "provisional", "committed",
                                                          "ripped_up", "removed"},
                          RouteState::current);
}

RouteGeometry route_geometry_from_string(std::string_view value) {
  return enum_from_string(value, std::array<std::string_view, 2>{"polyline", "quintic_ph"},
                          RouteGeometry::polyline);
}

ViaState via_state_from_string(std::string_view value) {
  return enum_from_string(
      value,
      std::array<std::string_view, 5>{"candidate", "selected", "contested", "rejected", "removed"},
      ViaState::candidate);
}

ConflictKind conflict_kind_from_string(std::string_view value) {
  return enum_from_string(value,
                          std::array<std::string_view, 5>{"congestion", "clearance_shadow",
                                                          "regional", "physical", "unknown"},
                          ConflictKind::unknown);
}

ResourceKind resource_kind_from_string(std::string_view value) {
  return enum_from_string(
      value,
      std::array<std::string_view, 5>{"sampled", "corridor", "physical", "via_site", "unknown"},
      ResourceKind::unknown);
}

DecodeResult decode_events(std::string_view json_or_jsonl) {
  DecodeResult result;
  if (json_or_jsonl.empty()) {
    result.error = "telemetry input is empty";
    return result;
  }

  try {
    decode_tree(parse_tree(json_or_jsonl), result.events);
    return result;
  } catch (const std::exception&) {
    result.events.clear();
  }

  std::istringstream lines{std::string(json_or_jsonl)};
  std::string line;
  std::size_t line_number = 0;
  try {
    while (std::getline(lines, line)) {
      ++line_number;
      const bool whitespace_only = std::all_of(
          line.begin(), line.end(), [](unsigned char ch) { return std::isspace(ch) != 0; });
      if (whitespace_only) {
        continue;
      }
      decode_tree(parse_tree(line), result.events);
    }
  } catch (const std::exception& exception) {
    result.events.clear();
    result.error = "telemetry JSON line " + std::to_string(line_number) + ": " + exception.what();
  }
  if (result.events.empty() && result.error.empty()) {
    result.error = "telemetry input contains no events";
  }
  return result;
}

std::string encode_event(const Event& event) {
  std::ostringstream out;
  out << std::setprecision(17) << '{';
  bool first = true;
  write_string(out, "type", event.type == EventType::snapshot ? "snapshot" : "event", first);
  write_string(out, "protocol", protocol_name, first);
  write_number(out, "schema", event.schema, first);
  if (event.type != EventType::snapshot) {
    write_string(out, "kind", wire_kind_name(event.type), first);
  }
  write_number(out, "sequence", event.sequence, first);
  write_number(out, "monotonic_time_us", event.monotonic_time_us, first);
  write_string(out, "level", to_string(event.level), first);
  if (!event.source_identity.empty()) {
    write_string(out, "source_identity", event.source_identity, first);
  }
  if (event.configuration_revision != 0U) {
    write_number(out, "configuration_revision", event.configuration_revision, first);
  }
  if (event.active_net) {
    write_string(out, event.type == EventType::snapshot ? "active_net" : "net", *event.active_net,
                 first);
  }
  if (event.active_group) {
    write_string(out, "active_group", *event.active_group, first);
  }
  write_array(out, "routes", event.routes, first, write_route);
  write_array(out, "vias", event.vias, first, write_via);
  write_array(out, "conflicts", event.conflicts, first, write_conflict);
  write_array(out, "resources", event.resources, first, write_resource);
  write_array(out, "hydraulic_fields", event.hydraulic_fields, first, write_hydraulic_field);
  write_array(out, "priorities", event.priorities, first, write_priority);
  write_array(out, "source_boards", event.source_boards, first, write_source_board);
  if (event.source_state_included && event.source_boards.empty()) {
    if (!first) {
      out << ',';
    }
    first = false;
    out << "\"source_boards\":[]";
  }
  if (event.dynamic_state_included) {
    const auto write_empty_array = [&out, &first](std::string_view name, bool empty) {
      if (!empty) {
        return;
      }
      if (!first) {
        out << ',';
      }
      first = false;
      out << '"' << name << "\":[]";
    };
    write_empty_array("routes", event.routes.empty());
    write_empty_array("vias", event.vias.empty());
    write_empty_array("conflicts", event.conflicts.empty());
    write_empty_array("resources", event.resources.empty());
    write_empty_array("hydraulic_fields", event.hydraulic_fields.empty());
    write_empty_array("priorities", event.priorities.empty());
  }
  if (event.metrics) {
    if (!first) {
      out << ',';
    }
    first = false;
    out << "\"metrics\":";
    write_metrics(out, *event.metrics);
  }
  if (event.counters) {
    if (!first) {
      out << ',';
    }
    first = false;
    const Counters& counters = *event.counters;
    out << "\"counters\":{\"produced\":" << counters.produced << ",\"dropped\":" << counters.dropped
        << ",\"coalesced\":" << counters.coalesced << ",\"serialized\":" << counters.serialized
        << ",\"snapshots\":" << counters.snapshots << '}';
  }
  if (!event.message.empty()) {
    write_string(out, "message", event.message, first);
  }
  out << '}';
  return out.str();
}

} // namespace flux::telemetry
