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

bool validate_hydraulic_field(const HydraulicField& field, std::string& error) {
  if (field.width == 0U || field.height == 0U || field.stride == 0U || field.cell_pitch_nm == 0U) {
    error = "hydraulic field dimensions, stride, and cell pitch must be positive";
    return false;
  }
  if (field.width > std::numeric_limits<std::size_t>::max() / field.height) {
    error = "hydraulic field cell count overflows";
    return false;
  }
  const std::size_t cells = field.width * field.height;
  const auto valid_values = [cells](const std::vector<std::uint16_t>& values) {
    return values.empty() || values.size() == cells;
  };
  if (field.erosion.empty() || !valid_values(field.flow) || !valid_values(field.erosion) ||
      !valid_values(field.heightmap) || !valid_values(field.free_space) ||
      !valid_values(field.downstream_pressure) || !valid_values(field.success_prior)) {
    error = "hydraulic field arrays must be empty or match width times height";
    return false;
  }
  const auto max = std::numeric_limits<std::int64_t>::max();
  const auto valid_extent = [&](std::int64_t origin, std::size_t cells_axis) {
    if (field.stride > std::numeric_limits<std::uint64_t>::max() / field.cell_pitch_nm) return false;
    const std::uint64_t step = field.stride * field.cell_pitch_nm;
    if (step > static_cast<std::uint64_t>(max)) return false;
    if (cells_axis > std::numeric_limits<std::uint64_t>::max() / step) return false;
    // Include the far edge of the last drawn cell, not just its origin.
    const std::uint64_t extent = cells_axis * step;
    return extent <= static_cast<std::uint64_t>(max) &&
           origin <= max - static_cast<std::int64_t>(extent);
  };
  if (!valid_extent(field.origin_x_nm, field.width) || !valid_extent(field.origin_y_nm, field.height)) {
    error = "hydraulic field world extent overflows int64";
    return false;
  }
  error.clear();
  return true;
}
namespace {

using boost::property_tree::ptree;

template <typename T> T get_or(const ptree& tree, const char* name, T fallback = {}) {
  return tree.get<T>(name, std::move(fallback));
}

template <typename T> std::optional<T> get_optional(const ptree& tree, const char* name) {
  const auto value = tree.get_optional<T>(name);
  return value ? std::optional<T>{*value} : std::nullopt;
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

void write_bool(std::ostringstream& out, std::string_view name, bool value, bool& first) {
  if (!first) {
    out << ',';
  }
  first = false;
  out << '"' << name << "\":" << (value ? "true" : "false");
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
  route.topology_gate_sequence_legal = get_or<bool>(tree, "topology_gate_sequence_legal", false);
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
  if (const auto cells = tree.get_child_optional("navmesh_cells")) {
    for (const auto& item : *cells) {
      Route::NavMeshCell cell;
      if (const auto minimum = item.second.get_child_optional("minimum")) {
        cell.minimum = parse_point(*minimum);
      }
      if (const auto maximum = item.second.get_child_optional("maximum")) {
        cell.maximum = parse_point(*maximum);
      }
      cell.refinement_level = get_or<std::uint32_t>(item.second, "refinement_level");
      route.navmesh_cells.push_back(std::move(cell));
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
  field.maximum_downstream_pressure = get_or<std::uint32_t>(tree, "maximum_downstream_pressure");
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
  std::string error;
  if (!validate_hydraulic_field(field, error)) throw std::runtime_error(error);
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

SourceStackupSublayer parse_source_stackup_sublayer(const ptree& tree) {
  SourceStackupSublayer sublayer;
  sublayer.thickness_nm = get_optional<std::uint64_t>(tree, "thickness_nm");
  sublayer.thickness_locked = get_or<bool>(tree, "thickness_locked");
  sublayer.material = get_or<std::string>(tree, "material");
  sublayer.relative_permittivity_ppb =
      get_optional<std::uint64_t>(tree, "relative_permittivity_ppb");
  sublayer.loss_tangent_ppb = get_optional<std::uint64_t>(tree, "loss_tangent_ppb");
  sublayer.color = get_or<std::string>(tree, "color");
  return sublayer;
}

SourceStackupLayer parse_source_stackup_layer(const ptree& tree) {
  SourceStackupLayer layer;
  layer.name = get_or<std::string>(tree, "name");
  layer.type = get_or<std::string>(tree, "type");
  layer.kind = get_or<std::string>(tree, "kind");
  layer.board_layer = get_or<std::string>(tree, "board_layer");
  if (const auto sublayers = tree.get_child_optional("sublayers")) {
    for (const auto& item : *sublayers) {
      layer.sublayers.push_back(parse_source_stackup_sublayer(item.second));
    }
  }
  return layer;
}

SourceStackup parse_source_stackup(const ptree& tree) {
  SourceStackup stackup;
  stackup.copper_finish = get_or<std::string>(tree, "copper_finish");
  stackup.dielectric_constraints = get_or<bool>(tree, "dielectric_constraints");
  stackup.edge_plating = get_or<bool>(tree, "edge_plating");
  stackup.castellated_pads = get_or<bool>(tree, "castellated_pads");
  stackup.edge_connector = get_or<std::string>(tree, "edge_connector");
  if (const auto layers = tree.get_child_optional("layers")) {
    for (const auto& item : *layers) {
      stackup.layers.push_back(parse_source_stackup_layer(item.second));
    }
  }
  return stackup;
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
  primitive.drill_width_nm = get_or<std::uint64_t>(tree, "drill_width_nm");
  primitive.drill_height_nm = get_or<std::uint64_t>(tree, "drill_height_nm");
  primitive.drill_offset_x_nm = get_or<std::int64_t>(tree, "drill_offset_x_nm");
  primitive.drill_offset_y_nm = get_or<std::int64_t>(tree, "drill_offset_y_nm");
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
  board->physical_thickness_nm = get_optional<std::uint64_t>(tree, "physical_thickness_nm");
  if (const auto stackup = tree.get_child_optional("stackup")) {
    board->stackup = parse_source_stackup(*stackup);
  }
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
  metrics.unresolved_packing_obligations =
      get_or<std::size_t>(tree, "unresolved_packing_obligations");
  metrics.packing_obligation_budget_exhausted =
      get_or<bool>(tree, "packing_obligation_budget_exhausted");
  metrics.selected_checkpoint = get_or<std::uint64_t>(tree, "selected_checkpoint");
  metrics.best_checkpoint = get_or<std::uint64_t>(tree, "best_checkpoint");
  metrics.selected_is_best = get_or<bool>(tree, "selected_is_best");
  return metrics;
}

Counters parse_counters(const ptree& tree) {
  return Counters{get_or<std::uint64_t>(tree, "produced"), get_or<std::uint64_t>(tree, "dropped"),
                  get_or<std::uint64_t>(tree, "coalesced"),
                  get_or<std::uint64_t>(tree, "serialized"),
                  get_or<std::uint64_t>(tree, "snapshots"),
                  get_or<std::uint64_t>(tree, "transport_dropped")};
}

PhaseProgress parse_progress(const ptree& tree) {
  return PhaseProgress{get_or<std::string>(tree, "phase"),
                       get_or<std::uint64_t>(tree, "work_units"),
                       get_or<std::uint64_t>(tree, "elapsed_us")};
}

ActiveRegion parse_active_region(const ptree& tree) {
  ActiveRegion region;
  region.id = get_or<std::string>(tree, "id");
  region.label = get_or<std::string>(tree, "label");
  region.minimum_x_nm = get_or<std::int64_t>(tree, "minimum_x_nm");
  region.minimum_y_nm = get_or<std::int64_t>(tree, "minimum_y_nm");
  region.maximum_x_nm = get_or<std::int64_t>(tree, "maximum_x_nm");
  region.maximum_y_nm = get_or<std::int64_t>(tree, "maximum_y_nm");
  region.layers = parse_ids(tree, "layers");
  region.nets = parse_ids(tree, "nets");
  region.active = get_or<bool>(tree, "active");
  return region;
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
  event.session_generation = get_or<std::uint64_t>(tree, "session_generation");
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
    event.packing_metrics_only = get_or<bool>(tree, "packing_metrics_only", false);
  } else if (event.type == EventType::snapshot ||
             event.type == EventType::pathfinder_iteration_started ||
             event.type == EventType::pathfinder_iteration_completed ||
             event.type == EventType::best_checkpoint_updated) {
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
  if (const auto progress = tree.get_child_optional("progress")) {
    const PhaseProgress parsed = parse_progress(*progress);
    if (!parsed.phase.empty()) {
      event.progress = parsed;
    }
  }
  if (const auto region = tree.get_child_optional("active_region")) {
    event.active_region = parse_active_region(*region);
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
      << (route.topology_gate_sequence_legal ? "true" : "false") << ",\"points\":[";
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
  out << "],\"navmesh_cells\":[";
  for (std::size_t index = 0; index < route.navmesh_cells.size(); ++index) {
    if (index != 0U) {
      out << ',';
    }
    const auto& cell = route.navmesh_cells[index];
    out << "{\"minimum\":";
    write_point(out, cell.minimum);
    out << ",\"maximum\":";
    write_point(out, cell.maximum);
    out << ",\"refinement_level\":" << cell.refinement_level << '}';
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
      if (index != 0U)
        out << ',';
      out << values[index];
    }
    out << ']';
  };
  out << "{\"id\":\"" << escape_json(field.id) << "\",\"net\":\"" << escape_json(field.net)
      << "\",\"layer\":\"" << escape_json(field.layer) << "\",\"revision\":" << field.revision
      << ",\"origin_x_nm\":" << field.origin_x_nm << ",\"origin_y_nm\":" << field.origin_y_nm
      << ",\"cell_pitch_nm\":" << field.cell_pitch_nm << ",\"width\":" << field.width
      << ",\"height\":" << field.height << ",\"stride\":" << field.stride
      << ",\"maximum_flow\":" << field.maximum_flow
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

void write_source_stackup_sublayer(std::ostringstream& out, const SourceStackupSublayer& sublayer) {
  out << '{';
  bool first = true;
  if (sublayer.thickness_nm) {
    write_number(out, "thickness_nm", *sublayer.thickness_nm, first);
  }
  write_bool(out, "thickness_locked", sublayer.thickness_locked, first);
  write_string(out, "material", sublayer.material, first);
  if (sublayer.relative_permittivity_ppb) {
    write_number(out, "relative_permittivity_ppb", *sublayer.relative_permittivity_ppb, first);
  }
  if (sublayer.loss_tangent_ppb) {
    write_number(out, "loss_tangent_ppb", *sublayer.loss_tangent_ppb, first);
  }
  write_string(out, "color", sublayer.color, first);
  out << '}';
}

void write_source_stackup_layer(std::ostringstream& out, const SourceStackupLayer& layer) {
  out << "{\"name\":\"" << escape_json(layer.name) << "\",\"type\":\"" << escape_json(layer.type)
      << "\",\"kind\":\"" << escape_json(layer.kind) << "\",\"board_layer\":\""
      << escape_json(layer.board_layer) << "\",\"sublayers\":[";
  for (std::size_t index = 0U; index < layer.sublayers.size(); ++index) {
    if (index != 0U) {
      out << ',';
    }
    write_source_stackup_sublayer(out, layer.sublayers[index]);
  }
  out << "]}";
}

void write_source_stackup(std::ostringstream& out, const SourceStackup& stackup) {
  out << "{\"copper_finish\":\"" << escape_json(stackup.copper_finish)
      << "\",\"dielectric_constraints\":" << (stackup.dielectric_constraints ? "true" : "false")
      << ",\"edge_plating\":" << (stackup.edge_plating ? "true" : "false")
      << ",\"castellated_pads\":" << (stackup.castellated_pads ? "true" : "false")
      << ",\"edge_connector\":\"" << escape_json(stackup.edge_connector) << "\",\"layers\":[";
  for (std::size_t index = 0U; index < stackup.layers.size(); ++index) {
    if (index != 0U) {
      out << ',';
    }
    write_source_stackup_layer(out, stackup.layers[index]);
  }
  out << "]}";
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
      << ",\"drill_width_nm\":" << primitive.drill_width_nm
      << ",\"drill_height_nm\":" << primitive.drill_height_nm
      << ",\"drill_offset_x_nm\":" << primitive.drill_offset_x_nm
      << ",\"drill_offset_y_nm\":" << primitive.drill_offset_y_nm
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
      << "\",\"board_schema\":" << board->board_schema;
  if (board->physical_thickness_nm) {
    out << ",\"physical_thickness_nm\":" << *board->physical_thickness_nm;
  }
  if (board->stackup) {
    out << ",\"stackup\":";
    write_source_stackup(out, *board->stackup);
  }
  out << ",\"layers\":[";
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
      << ",\"unresolved_packing_obligations\":" << metrics.unresolved_packing_obligations
      << ",\"packing_obligation_budget_exhausted\":"
      << (metrics.packing_obligation_budget_exhausted ? "true" : "false")
      << ",\"selected_checkpoint\":" << metrics.selected_checkpoint
      << ",\"best_checkpoint\":" << metrics.best_checkpoint
      << ",\"selected_is_best\":" << (metrics.selected_is_best ? "true" : "false") << '}';
}

std::string_view wire_kind_name(EventType type) {
  static constexpr std::array names{std::string_view{"SessionStarted"},
                                    std::string_view{"Snapshot"},
                                    std::string_view{"RoutingStarted"},
                                    std::string_view{"RoutingFinished"},
                                    std::string_view{"RoutingCompleted"},
                                    std::string_view{"CopperGenerated"},
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
FLUX_ENUM_STRINGS(EventType, "session_started", "snapshot", "routing_started", "routing_finished",
                  "routing_completed", "copper_generated", "net_queued", "net_started",
                  "net_completed", "net_failed", "route_candidate_selected", "route_committed",
                  "route_removed", "route_ripped_up", "via_candidate_generated",
                  "via_candidate_rejected", "via_assigned", "via_removed",
                  "pathfinder_iteration_started", "pathfinder_iteration_completed",
                  "conflict_added", "conflict_removed", "resource_occupancy_updated",
                  "hydraulic_field_updated", "residual_group_updated", "checkpoint_selected",
                  "best_checkpoint_updated", "placement_proposal_started",
                  "placement_proposal_accepted", "placement_proposal_rejected", "search_summary",
                  "unknown")
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
  static constexpr std::array names{std::string_view{"session_started"},
                                    std::string_view{"snapshot"},
                                    std::string_view{"routing_started"},
                                    std::string_view{"routing_finished"},
                                    std::string_view{"routing_completed"},
                                    std::string_view{"copper_generated"},
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
  static constexpr std::array wire_names{std::string_view{"SessionStarted"},
                                         std::string_view{"Snapshot"},
                                         std::string_view{"RoutingStarted"},
                                         std::string_view{"RoutingFinished"},
                                         std::string_view{"RoutingCompleted"},
                                         std::string_view{"CopperGenerated"},
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
  if (event.session_generation != 0U) {
    write_number(out, "session_generation", event.session_generation, first);
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
    if (event.packing_metrics_only) out << ",\"packing_metrics_only\":true";
  }
  if (event.counters) {
    if (!first) {
      out << ',';
    }
    first = false;
    const Counters& counters = *event.counters;
    out << "\"counters\":{\"produced\":" << counters.produced << ",\"dropped\":" << counters.dropped
        << ",\"coalesced\":" << counters.coalesced << ",\"serialized\":" << counters.serialized
        << ",\"snapshots\":" << counters.snapshots
        << ",\"transport_dropped\":" << counters.transport_dropped << '}';
  }
  if (event.progress && !event.progress->phase.empty()) {
    if (!first) {
      out << ',';
    }
    first = false;
    out << "\"progress\":{\"phase\":\"" << escape_json(event.progress->phase)
        << "\",\"work_units\":" << event.progress->work_units << ",\"elapsed_us\":"
        << event.progress->elapsed_us << '}';
  }
  if (event.active_region) {
    const ActiveRegion& region = *event.active_region;
    if (!first) out << ',';
    first = false;
    out << "\"active_region\":{\"id\":\"" << escape_json(region.id)
        << "\",\"label\":\"" << escape_json(region.label) << "\",\"minimum_x_nm\":"
        << region.minimum_x_nm << ",\"minimum_y_nm\":" << region.minimum_y_nm
        << ",\"maximum_x_nm\":" << region.maximum_x_nm << ",\"maximum_y_nm\":"
        << region.maximum_y_nm << ",\"layers\":";
    write_ids(out, region.layers);
    out << ",\"nets\":";
    write_ids(out, region.nets);
    out << ",\"active\":" << (region.active ? "true" : "false") << '}';
  }
  if (!event.message.empty()) {
    write_string(out, "message", event.message, first);
  }
  out << '}';
  return out.str();
}

std::size_t estimated_event_bytes(const Event& event) {
  const auto string_bytes = [](const std::string& value) { return value.capacity(); };
  const auto point_bytes = [](const Point& point) { return sizeof(Point) + point.layer.capacity(); };
  std::size_t bytes = sizeof(Event) + string_bytes(event.source_identity) + string_bytes(event.message);
  if (event.active_net) bytes += string_bytes(*event.active_net);
  if (event.active_group) bytes += string_bytes(*event.active_group);
  if (event.progress) bytes += sizeof(PhaseProgress) + string_bytes(event.progress->phase);
  if (event.active_region) {
    bytes += sizeof(ActiveRegion) + string_bytes(event.active_region->id) +
             string_bytes(event.active_region->label) +
             event.active_region->layers.capacity() * sizeof(std::string) +
             event.active_region->nets.capacity() * sizeof(std::string);
    for (const auto& layer : event.active_region->layers) bytes += string_bytes(layer);
    for (const auto& net : event.active_region->nets) bytes += string_bytes(net);
  }
  bytes += event.routes.capacity() * sizeof(Route) + event.vias.capacity() * sizeof(Via) +
           event.conflicts.capacity() * sizeof(Conflict) +
           event.resources.capacity() * sizeof(ResourceOccupancy) +
           event.hydraulic_fields.capacity() * sizeof(HydraulicField) +
           event.priorities.capacity() * sizeof(Priority);
  for (const Route& route : event.routes) {
    bytes += sizeof(Route) + string_bytes(route.id) + string_bytes(route.net) +
             string_bytes(route.family) + string_bytes(route.lifecycle);
    bytes += route.points.capacity() * sizeof(Point) +
             route.control_points.capacity() * sizeof(Point);
    for (const Point& point : route.points) bytes += point.layer.capacity();
    for (const Point& point : route.control_points) bytes += point.layer.capacity();
    bytes += route.navmesh_cells.capacity() * sizeof(Route::NavMeshCell);
    for (const auto& cell : route.navmesh_cells)
      bytes += cell.minimum.layer.capacity() + cell.maximum.layer.capacity();
  }
  for (const Via& via : event.vias)
    bytes += sizeof(Via) + string_bytes(via.id) + string_bytes(via.net) +
             string_bytes(via.to_layer) + string_bytes(via.rejection_reason) + point_bytes(via.position);
  for (const Conflict& conflict : event.conflicts) {
    bytes += sizeof(Conflict) + string_bytes(conflict.id) + string_bytes(conflict.resource) +
             string_bytes(conflict.group) + point_bytes(conflict.location);
    bytes += conflict.nets.capacity() * sizeof(std::string);
    for (const auto& net : conflict.nets) bytes += string_bytes(net);
  }
  for (const ResourceOccupancy& resource : event.resources) {
    bytes += sizeof(ResourceOccupancy) + string_bytes(resource.id) + string_bytes(resource.layer) +
             point_bytes(resource.start) + point_bytes(resource.end);
    bytes += resource.nets.capacity() * sizeof(std::string);
    for (const auto& net : resource.nets) bytes += string_bytes(net);
  }
  for (const HydraulicField& field : event.hydraulic_fields)
    bytes += sizeof(HydraulicField) + string_bytes(field.id) + string_bytes(field.net) +
             string_bytes(field.layer) + field.flow.capacity() * sizeof(std::uint16_t) +
             field.erosion.capacity() * sizeof(std::uint16_t) + field.heightmap.capacity() * sizeof(std::uint16_t) +
             field.free_space.capacity() * sizeof(std::uint16_t) +
             field.downstream_pressure.capacity() * sizeof(std::uint16_t) +
             field.success_prior.capacity() * sizeof(std::uint16_t);
  for (const Priority& priority : event.priorities)
    bytes += sizeof(Priority) + string_bytes(priority.net) + string_bytes(priority.group) +
             string_bytes(priority.staged_class);
  // A trace/history may be the sole owner of a shared source scene. Charge its deep geometry once
  // per Event so queue and replay budgets cannot retain arbitrary board payloads through pointers.
  bytes += event.source_boards.capacity() * sizeof(std::shared_ptr<const SourceBoard>);
  for (const auto& board : event.source_boards) {
    if (!board) continue;
    bytes += sizeof(SourceBoard) + string_bytes(board->id) + string_bytes(board->revision) +
             string_bytes(board->path) + board->layers.capacity() * sizeof(SourceLayer) +
             board->nets.capacity() * sizeof(SourceNet) +
             board->primitives.capacity() * sizeof(SourcePrimitive);
    for (const auto& layer : board->layers)
      bytes += string_bytes(layer.id) + string_bytes(layer.name) + string_bytes(layer.kind);
    for (const auto& net : board->nets) bytes += string_bytes(net.id) + string_bytes(net.name);
    for (const auto& primitive : board->primitives) {
      bytes += string_bytes(primitive.id) + string_bytes(primitive.kind) +
               string_bytes(primitive.parent) + string_bytes(primitive.reference) +
               string_bytes(primitive.net) + string_bytes(primitive.shape) + string_bytes(primitive.side) +
               primitive.layers.capacity() * sizeof(std::string) +
               primitive.points.capacity() * sizeof(Point) +
               primitive.holes.capacity() * sizeof(std::vector<Point>);
      for (const auto& layer : primitive.layers) bytes += string_bytes(layer);
      for (const auto& point : primitive.points) bytes += string_bytes(point.layer);
      for (const auto& hole : primitive.holes) {
        bytes += hole.capacity() * sizeof(Point);
        for (const auto& point : hole) bytes += string_bytes(point.layer);
      }
    }
    if (board->stackup) {
      bytes += sizeof(SourceStackup) + string_bytes(board->stackup->copper_finish) +
               string_bytes(board->stackup->edge_connector) +
               board->stackup->layers.capacity() * sizeof(SourceStackupLayer);
      for (const auto& layer : board->stackup->layers) {
        bytes += string_bytes(layer.name) + string_bytes(layer.type) + string_bytes(layer.kind) +
                 string_bytes(layer.board_layer) +
                 layer.sublayers.capacity() * sizeof(SourceStackupSublayer);
        for (const auto& sublayer : layer.sublayers)
          bytes += string_bytes(sublayer.material) + string_bytes(sublayer.color);
      }
    }
  }
  return bytes;
}

} // namespace flux::telemetry
