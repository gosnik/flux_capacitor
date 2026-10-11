#include "flux/telemetry/state.hpp"

#include <algorithm>
#include <charconv>
#include <cctype>
#include <cmath>
#include <iterator>
#include <limits>
#include <sstream>
#include <unordered_set>

namespace flux::telemetry {
namespace {

constexpr std::size_t maximum_live_geometry_bytes = 128U * 1024U * 1024U;
constexpr std::size_t maximum_metadata_string_bytes = 8U * 1024U;
constexpr std::size_t maximum_activity_net_bytes = 1024U;

std::string bounded_string(std::string_view value, std::size_t maximum = maximum_metadata_string_bytes) {
  return std::string(value.substr(0U, maximum));
}

std::size_t string_bytes(const std::string& value) { return value.capacity(); }
std::size_t point_bytes(const Point& point) { return sizeof(Point) + string_bytes(point.layer); }

std::size_t route_bytes(const Route& route) {
  std::size_t result = sizeof(Route) + string_bytes(route.id) + string_bytes(route.net) +
                       string_bytes(route.family) + string_bytes(route.lifecycle) +
                       route.points.capacity() * sizeof(Point) +
                       route.control_points.capacity() * sizeof(Point) +
                       route.navmesh_cells.capacity() * sizeof(Route::NavMeshCell);
  for (const auto& point : route.points) result += string_bytes(point.layer);
  for (const auto& point : route.control_points) result += string_bytes(point.layer);
  for (const auto& cell : route.navmesh_cells)
    result += string_bytes(cell.minimum.layer) + string_bytes(cell.maximum.layer);
  return result;
}

std::size_t via_bytes(const Via& via) {
  return sizeof(Via) + string_bytes(via.id) + string_bytes(via.net) +
         string_bytes(via.to_layer) + string_bytes(via.rejection_reason) + point_bytes(via.position);
}

std::size_t conflict_bytes(const Conflict& conflict) {
  std::size_t result = sizeof(Conflict) + string_bytes(conflict.id) + string_bytes(conflict.resource) +
                       string_bytes(conflict.group) + point_bytes(conflict.location) +
                       conflict.nets.capacity() * sizeof(std::string);
  for (const auto& net : conflict.nets) result += string_bytes(net);
  return result;
}

std::size_t resource_bytes(const ResourceOccupancy& resource) {
  std::size_t result = sizeof(ResourceOccupancy) + string_bytes(resource.id) +
                       string_bytes(resource.layer) + point_bytes(resource.start) +
                       point_bytes(resource.end) + resource.nets.capacity() * sizeof(std::string);
  for (const auto& net : resource.nets) result += string_bytes(net);
  return result;
}

std::size_t hydraulic_bytes(const HydraulicField& field) {
  return sizeof(HydraulicField) + string_bytes(field.id) + string_bytes(field.net) +
         string_bytes(field.layer) + field.flow.capacity() * sizeof(std::uint16_t) +
         field.erosion.capacity() * sizeof(std::uint16_t) +
         field.heightmap.capacity() * sizeof(std::uint16_t) +
         field.free_space.capacity() * sizeof(std::uint16_t) +
         field.downstream_pressure.capacity() * sizeof(std::uint16_t) +
         field.success_prior.capacity() * sizeof(std::uint16_t);
}

std::size_t priority_bytes(const Priority& priority) {
  return sizeof(Priority) + string_bytes(priority.net) + string_bytes(priority.group) +
         string_bytes(priority.staged_class);
}

std::size_t source_board_bytes(const SourceBoard& board) {
  std::size_t result = sizeof(SourceBoard) + string_bytes(board.id) + string_bytes(board.path) +
                       string_bytes(board.revision) + board.layers.capacity() * sizeof(SourceLayer) +
                       board.nets.capacity() * sizeof(SourceNet) +
                       board.primitives.capacity() * sizeof(SourcePrimitive);
  for (const auto& layer : board.layers)
    result += string_bytes(layer.id) + string_bytes(layer.name) + string_bytes(layer.kind);
  for (const auto& net : board.nets) result += string_bytes(net.id) + string_bytes(net.name);
  for (const auto& primitive : board.primitives) {
    result += string_bytes(primitive.id) + string_bytes(primitive.kind) + string_bytes(primitive.parent) +
              string_bytes(primitive.reference) + string_bytes(primitive.net) +
              string_bytes(primitive.shape) + string_bytes(primitive.side) +
              primitive.layers.capacity() * sizeof(std::string) +
              primitive.points.capacity() * sizeof(Point) +
              primitive.holes.capacity() * sizeof(std::vector<Point>);
    for (const auto& layer : primitive.layers) result += string_bytes(layer);
    for (const auto& point : primitive.points) result += string_bytes(point.layer);
    for (const auto& hole : primitive.holes) {
      result += hole.capacity() * sizeof(Point);
      for (const auto& point : hole) result += string_bytes(point.layer);
    }
  }
  if (board.stackup) {
    result += sizeof(SourceStackup) + string_bytes(board.stackup->copper_finish) +
              string_bytes(board.stackup->edge_connector) +
              board.stackup->layers.capacity() * sizeof(SourceStackupLayer);
    for (const auto& layer : board.stackup->layers) {
      result += string_bytes(layer.name) + string_bytes(layer.type) + string_bytes(layer.kind) +
                string_bytes(layer.board_layer) + layer.sublayers.capacity() * sizeof(SourceStackupSublayer);
      for (const auto& sublayer : layer.sublayers)
        result += string_bytes(sublayer.material) + string_bytes(sublayer.color);
    }
  }
  return result;
}

bool required_route(const Route& route) {
  return route.state == RouteState::current || route.state == RouteState::committed;
}
bool required_via(const Via& via) { return via.state == ViaState::selected; }

template <typename Map, typename Estimate>
std::size_t map_bytes(const Map& values, Estimate estimate) {
  std::size_t result = 0U;
  for (const auto& [key, value] : values)
    result += sizeof(typename Map::value_type) + 3U * sizeof(void*) + string_bytes(key) + estimate(value);
  return result;
}

std::optional<std::string> event_net(const Event& event) {
  if (event.active_net) {
    return event.active_net;
  }
  if (!event.routes.empty()) {
    return event.routes.front().net;
  }
  if (!event.vias.empty()) {
    return event.vias.front().net;
  }
  if (!event.priorities.empty()) {
    return event.priorities.front().net;
  }
  return std::nullopt;
}

std::string activity_message(const Event& event) {
  if (!event.message.empty()) {
    return event.message;
  }
  std::ostringstream message;
  message << to_string(event.type);
  if (const auto net = event_net(event)) {
    message << " net " << *net;
  }
  if (event.metrics) {
    message << " iteration " << event.metrics->iteration;
  }
  return message.str();
}

void collect_logged_diagnostics(const Event& event,
                                std::map<std::string, LoggedDiagnostic>& diagnostics) {
  if (event.type != EventType::search_summary || event.message.empty()) return;
  const std::string_view message{event.message.data(), std::min<std::size_t>(event.message.size(), 8192U)};
  const std::string scope = event.active_group ? bounded_string(*event.active_group, 256U)
                            : event.active_net ? bounded_string(*event.active_net, 256U)
                            : event.progress ? bounded_string(event.progress->phase, 256U) : "search";
  std::size_t cursor = 0U;
  while (cursor < message.size()) {
    while (cursor < message.size() && !(std::isalnum(static_cast<unsigned char>(message[cursor])) ||
                                        message[cursor] == '_' || message[cursor] == '.')) ++cursor;
    const std::size_t key_begin = cursor;
    while (cursor < message.size() && (std::isalnum(static_cast<unsigned char>(message[cursor])) ||
                                      message[cursor] == '_' || message[cursor] == '.')) ++cursor;
    if (key_begin == cursor || cursor >= message.size() || message[cursor] != '=') continue;
    const std::string key = bounded_string(message.substr(key_begin, cursor - key_begin), 256U);
    ++cursor;
    const char* first = message.data() + cursor;
    const char* last = message.data() + message.size();
    double value = 0.0;
    const auto [end, error] = std::from_chars(first, last, value);
    if (error != std::errc{} || end == first || !std::isfinite(value) ||
        (end != last && (std::isalnum(static_cast<unsigned char>(*end)) || *end == '_' ||
                         *end == '.' || *end == '/'))) {
      continue;
    }
    cursor = static_cast<std::size_t>(end - message.data());
    diagnostics.insert_or_assign(scope + "." + key,
                                 LoggedDiagnostic{value, event.monotonic_time_us, scope});
    constexpr std::size_t maximum_diagnostics = 128U;
    while (diagnostics.size() > maximum_diagnostics) {
      const auto oldest = std::min_element(diagnostics.begin(), diagnostics.end(),
          [](const auto& first_entry, const auto& second_entry) {
            return first_entry.second.updated_time_us < second_entry.second.updated_time_us;
          });
      diagnostics.erase(oldest);
    }
  }
}

} // namespace

void LiveSessionTimeline::clear() { *this = LiveSessionTimeline{}; }

std::optional<Event> LiveSessionTimeline::begin_session(std::uint64_t generation) {
  if (generation == 0U || generation == generation_) {
    return std::nullopt;
  }
  generation_ = generation;
  awaiting_first_event_ = true;

  Event boundary;
  boundary.session_generation = generation_;
  boundary.sequence = last_sequence_ == std::numeric_limits<std::uint64_t>::max()
                          ? last_sequence_
                          : last_sequence_ + 1U;
  boundary.monotonic_time_us = last_time_us_ == std::numeric_limits<std::uint64_t>::max()
                                   ? last_time_us_
                                   : last_time_us_ + 1U;
  boundary.type = EventType::session_started;
  boundary.message = "Telemetry session " + std::to_string(generation_) + " connected";
  last_sequence_ = boundary.sequence;
  last_time_us_ = boundary.monotonic_time_us;
  return boundary;
}

Event LiveSessionTimeline::rebase(Event event) {
  event.session_generation = generation_;
  if (awaiting_first_event_) {
    raw_sequence_origin_ = event.sequence;
    raw_time_origin_ = event.monotonic_time_us;
    rebased_sequence_origin_ = last_sequence_ == std::numeric_limits<std::uint64_t>::max()
                                   ? last_sequence_
                                   : last_sequence_ + 1U;
    rebased_time_origin_ = last_time_us_ == std::numeric_limits<std::uint64_t>::max()
                               ? last_time_us_
                               : last_time_us_ + 1U;
    awaiting_first_event_ = false;
  }
  const auto rebase_value = [](std::uint64_t raw, std::uint64_t raw_origin,
                               std::uint64_t rebased_origin, std::uint64_t previous) {
    const auto delta = raw >= raw_origin ? raw - raw_origin : 0U;
    const auto maximum = std::numeric_limits<std::uint64_t>::max();
    const auto candidate = delta > maximum - rebased_origin ? maximum : rebased_origin + delta;
    return std::max(previous, candidate);
  };
  event.sequence =
      rebase_value(event.sequence, raw_sequence_origin_, rebased_sequence_origin_, last_sequence_);
  event.monotonic_time_us =
      rebase_value(event.monotonic_time_us, raw_time_origin_, rebased_time_origin_, last_time_us_);
  last_sequence_ = event.sequence;
  last_time_us_ = event.monotonic_time_us;
  return event;
}

void LiveState::clear() { *this = LiveState{}; }

bool LiveState::mandatory_geometry_fits(const Event& event, bool reset_boundary) const {
  std::size_t bytes = 0U;
  std::unordered_set<const SourceBoard*> counted_sources;
  const auto add_source = [&](const std::shared_ptr<const SourceBoard>& board) {
    if (board && counted_sources.insert(board.get()).second) bytes += source_board_bytes(*board);
  };
  if (event.source_state_included) {
    for (const auto& board : event.source_boards) {
      if (board) bytes += sizeof(std::pair<const std::string, std::shared_ptr<const SourceBoard>>) +
                          3U * sizeof(void*) + string_bytes(board->id);
      add_source(board);
    }
  } else if (!reset_boundary) {
    for (const auto& [id, board] : source_boards_) {
      bytes += sizeof(decltype(source_boards_)::value_type) + 3U * sizeof(void*) + string_bytes(id);
      add_source(board);
    }
  }

  const auto add_route = [&](const Route& route) {
    if (required_route(route))
      bytes += sizeof(decltype(routes_)::value_type) + 3U * sizeof(void*) + string_bytes(route.id) +
               route_bytes(route);
  };
  const auto add_via = [&](const Via& via) {
    if (required_via(via))
      bytes += sizeof(decltype(vias_)::value_type) + 3U * sizeof(void*) + string_bytes(via.id) +
               via_bytes(via);
  };
  if (event.dynamic_state_included || reset_boundary) {
    for (const auto& route : event.routes) add_route(route);
    for (const auto& via : event.vias) add_via(via);
    return bytes <= maximum_live_geometry_bytes;
  }
  const auto route_replaced = [&](const std::string& id) {
    return std::any_of(event.routes.begin(), event.routes.end(), [&](const Route& route) {
      return route.id == id;
    });
  };
  const auto via_replaced = [&](const std::string& id) {
    return std::any_of(event.vias.begin(), event.vias.end(), [&](const Via& via) {
      return via.id == id;
    });
  };
  if (!reset_boundary) {
    for (const auto& [id, route] : routes_)
      if (!route_replaced(id)) add_route(route);
    for (const auto& [id, via] : vias_)
      if (!via_replaced(id)) add_via(via);
  }
  for (const auto& route : event.routes) {
    Route candidate = route;
    switch (event.type) {
    case EventType::route_candidate_selected: candidate.state = RouteState::provisional; break;
    case EventType::route_committed: candidate.state = RouteState::committed; break;
    case EventType::route_ripped_up: candidate.state = RouteState::ripped_up; break;
    case EventType::route_removed: candidate.state = RouteState::removed; break;
    default: break;
    }
    if (candidate.points.empty()) {
      if (const auto existing = routes_.find(candidate.id); existing != routes_.end()) {
        // The actual apply path retains this geometry; count it without mutating the state.
        candidate.points = existing->second.points;
        candidate.control_points = existing->second.control_points;
        candidate.navmesh_cells = existing->second.navmesh_cells;
      }
    }
    add_route(candidate);
  }
  for (const auto& via : event.vias) {
    Via candidate = via;
    switch (event.type) {
    case EventType::via_candidate_generated:
      candidate.state = candidate.competing_nets > 0U ? ViaState::contested : ViaState::candidate;
      break;
    case EventType::via_candidate_rejected: candidate.state = ViaState::rejected; break;
    case EventType::via_assigned: candidate.state = ViaState::selected; break;
    case EventType::via_removed: candidate.state = ViaState::removed; break;
    default: break;
    }
    if (candidate.position == Point{}) {
      if (const auto existing = vias_.find(candidate.id); existing != vias_.end())
        candidate.position = existing->second.position;
    }
    add_via(candidate);
  }
  return bytes <= maximum_live_geometry_bytes;
}

void LiveState::trim_geometry_to_budget() {
  const auto total_bytes = [&] {
    std::size_t bytes = map_bytes(routes_, route_bytes) + map_bytes(vias_, via_bytes) +
                        map_bytes(conflicts_, conflict_bytes) + map_bytes(resources_, resource_bytes) +
                        map_bytes(hydraulic_fields_, hydraulic_bytes) + map_bytes(priorities_, priority_bytes);
    std::unordered_set<const SourceBoard*> counted_sources;
    for (const auto& [id, board] : source_boards_)
      if (board && counted_sources.insert(board.get()).second)
        bytes += sizeof(decltype(source_boards_)::value_type) + 3U * sizeof(void*) +
                 string_bytes(id) + source_board_bytes(*board);
    return bytes;
  };
  std::size_t bytes = total_bytes();
  const auto entry_bytes = [](const auto& entry, auto estimate) {
    return sizeof(entry) + 3U * sizeof(void*) + string_bytes(entry.first) + estimate(entry.second);
  };
  const auto trim_all = [&](auto& values, auto estimate) {
    while (bytes > maximum_live_geometry_bytes && !values.empty()) {
      bytes -= std::min(bytes, entry_bytes(*values.begin(), estimate));
      values.erase(values.begin());
    }
  };
  trim_all(hydraulic_fields_, hydraulic_bytes);
  trim_all(resources_, resource_bytes);
  trim_all(conflicts_, conflict_bytes);
  trim_all(priorities_, priority_bytes);
  const auto trim_optional = [&](auto& values, auto required, auto estimate) {
    for (auto it = values.begin(); bytes > maximum_live_geometry_bytes && it != values.end();) {
      if (!required(it->second)) {
        bytes -= std::min(bytes, entry_bytes(*it, estimate));
        it = values.erase(it);
      }
      else ++it;
    }
  };
  trim_optional(vias_, required_via, via_bytes);
  trim_optional(routes_, required_route, route_bytes);
}

void LiveState::apply_snapshot(const Event& event) {
  if (event.source_state_included) {
    source_boards_.clear();
    for (const auto& board : event.source_boards) {
      if (board && !board->id.empty()) {
        source_boards_.insert_or_assign(board->id, board);
      }
    }
  }
  if (!event.dynamic_state_included) {
    return;
  }
  routes_.clear();
  vias_.clear();
  conflicts_.clear();
  resources_.clear();
  hydraulic_fields_.clear();
  priorities_.clear();

  for (const Route& route : event.routes) {
    if (!route.id.empty())
      routes_.insert_or_assign(route.id, route);
  }
  for (const Via& via : event.vias) {
    if (!via.id.empty())
      vias_.insert_or_assign(via.id, via);
  }
  for (const Conflict& conflict : event.conflicts) {
    if (!conflict.id.empty())
      conflicts_.insert_or_assign(conflict.id, conflict);
  }
  for (const ResourceOccupancy& resource : event.resources) {
    if (!resource.id.empty())
      resources_.insert_or_assign(resource.id, resource);
  }
  for (const HydraulicField& field : event.hydraulic_fields) {
    if (!field.id.empty())
      hydraulic_fields_.insert_or_assign(field.id, field);
  }
  for (const Priority& priority : event.priorities) {
    if (!priority.net.empty())
      priorities_.insert_or_assign(priority.net, priority);
  }
}

void LiveState::append_activity(const Event& event) {
  std::optional<std::string> net;
  if (const auto value = event_net(event)) net = bounded_string(*value, maximum_activity_net_bytes);
  activity_.push_back(Activity{event.session_generation, event.sequence, event.monotonic_time_us,
                               event.type, event.level, std::move(net),
                               bounded_string(activity_message(event))});
  constexpr std::size_t maximum_activity = 4'096U;
  if (activity_.size() > maximum_activity) {
    activity_.pop_front();
  }
}

void LiveState::apply(const Event& event) {
  // Source identity is the board/run boundary carried by the authoritative producer. A
  // long-lived viewer may receive several boards without reconnecting, and a recorded trace may
  // contain several such runs. Never let geometry, metrics, conflicts, or activity from the
  // preceding board bleed into the new one. Repeated routing-stage starts for the same immutable
  // source deliberately do not reset state.
  const bool reset_boundary = !event.source_identity.empty() && !source_identity_.empty() &&
                              event.source_identity != source_identity_;
  if (!mandatory_geometry_fits(event, reset_boundary)) {
    ++dropped_oversized_geometry_updates_;
    return;
  }
  if (reset_boundary) {
    clear();
  }

  const auto previous_metrics = metrics_;
  // Compare incoming records before applying them; copying all retained geometry on every
  // heartbeat defeats bounded telemetry. Identical recovery snapshots must not reset freshness.
  const bool geometry_updated = [&] {
    const auto changed = [](const auto& retained, const auto& incoming, bool replaces,
                            const auto& key) {
      if (replaces && retained.size() != incoming.size()) return true;
      for (const auto& item : incoming) {
        const auto found = retained.find(key(item));
        if (found == retained.end() || found->second != item) return true;
      }
      return false;
    };
    if (event.source_state_included) {
      if (source_boards_.size() != event.source_boards.size()) return true;
      for (const auto& board : event.source_boards) {
        if (!board) continue;
        const auto found = source_boards_.find(board->id);
        if (found == source_boards_.end() || !found->second ||
            (found->second != board && *found->second != *board)) return true;
      }
    }
    const bool replaces = event.type == EventType::snapshot && event.dynamic_state_included;
    const auto id = [](const auto& value) -> const auto& { return value.id; };
    return changed(routes_, event.routes, replaces, id) ||
           changed(vias_, event.vias, replaces, id) ||
           changed(conflicts_, event.conflicts, replaces, id) ||
           changed(resources_, event.resources, replaces, id) ||
           changed(hydraulic_fields_, event.hydraulic_fields, replaces, id);
  }();

  if (sequence_ != 0U && event.sequence > sequence_ + 1U) {
    observed_sequence_gaps_ += event.sequence - sequence_ - 1U;
  }
  sequence_ = std::max(sequence_, event.sequence);
  if (!event.source_identity.empty()) {
    source_identity_ = bounded_string(event.source_identity);
  }
  if (event.configuration_revision != 0U) {
    configuration_revision_ = event.configuration_revision;
  }
  if (event.active_net) {
    if (event.active_net->empty()) {
      active_net_.reset();
    } else {
      active_net_ = bounded_string(*event.active_net);
    }
  }
  if (event.active_group) {
    if (event.active_group->empty()) {
      active_group_.reset();
    } else {
      active_group_ = bounded_string(*event.active_group);
    }
  }
  if (event.active_region) {
    if (event.active_region->active) {
      active_region_ = event.active_region;
      active_region_->id = bounded_string(active_region_->id);
      active_region_->label = bounded_string(active_region_->label);
      for (auto& layer : active_region_->layers) layer = bounded_string(layer, 256U);
      for (auto& net : active_region_->nets) net = bounded_string(net, 256U);
      constexpr std::size_t maximum_region_items = 128U;
      active_region_->layers.resize(std::min(active_region_->layers.size(), maximum_region_items));
      active_region_->nets.resize(std::min(active_region_->nets.size(), maximum_region_items));
    }
    else active_region_.reset();
  }

  if (event.type == EventType::snapshot) {
    apply_snapshot(event);
  } else {
    for (Route route : event.routes) {
      switch (event.type) {
      case EventType::route_candidate_selected:
        route.state = RouteState::provisional;
        break;
      case EventType::route_committed:
        route.state = RouteState::committed;
        break;
      case EventType::route_ripped_up:
        route.state = RouteState::ripped_up;
        break;
      case EventType::route_removed:
        route.state = RouteState::removed;
        break;
      default:
        break;
      }
      if (!route.id.empty()) {
        if (route.points.empty()) {
          if (const auto existing = routes_.find(route.id); existing != routes_.end()) {
            route.points = existing->second.points;
            route.width_nm = route.width_nm == 0U ? existing->second.width_nm : route.width_nm;
            route.net = route.net.empty() ? existing->second.net : route.net;
            route.geometry = existing->second.geometry;
            route.spline_degree = existing->second.spline_degree;
            route.control_points = existing->second.control_points;
            route.navmesh_cells = existing->second.navmesh_cells;
          }
        }
        routes_.insert_or_assign(route.id, std::move(route));
      }
    }

    for (Via via : event.vias) {
      switch (event.type) {
      case EventType::via_candidate_generated:
        if (via.competing_nets > 0U) {
          via.state = ViaState::contested;
        } else {
          via.state = ViaState::candidate;
        }
        break;
      case EventType::via_candidate_rejected:
        via.state = ViaState::rejected;
        break;
      case EventType::via_assigned:
        via.state = ViaState::selected;
        break;
      case EventType::via_removed:
        via.state = ViaState::removed;
        break;
      default:
        break;
      }
      if (!via.id.empty()) {
        if (const auto existing = vias_.find(via.id); existing != vias_.end()) {
          if (via.position == Point{}) {
            via.position = existing->second.position;
            via.to_layer = existing->second.to_layer;
          }
          via.net = via.net.empty() ? existing->second.net : via.net;
        }
        vias_.insert_or_assign(via.id, std::move(via));
      }
    }

    if (event.type == EventType::conflict_removed) {
      for (const Conflict& conflict : event.conflicts) {
        if (!conflict.id.empty())
          conflicts_.erase(conflict.id);
      }
    } else {
      for (const Conflict& conflict : event.conflicts) {
        if (!conflict.id.empty())
          conflicts_.insert_or_assign(conflict.id, conflict);
      }
    }
    for (const ResourceOccupancy& resource : event.resources) {
      if (!resource.id.empty())
        resources_.insert_or_assign(resource.id, resource);
    }
    for (const HydraulicField& field : event.hydraulic_fields) {
      if (!field.id.empty())
        hydraulic_fields_.insert_or_assign(field.id, field);
    }
    for (const Priority& priority : event.priorities) {
      if (!priority.net.empty())
        priorities_.insert_or_assign(priority.net, priority);
    }
  }

  if (event.metrics) {
    if (event.packing_metrics_only) {
      if (!metrics_) metrics_.emplace();
      metrics_->unresolved_packing_obligations = event.metrics->unresolved_packing_obligations;
      metrics_->packing_obligation_budget_exhausted = event.metrics->packing_obligation_budget_exhausted;
    } else {
      metrics_ = event.metrics;
      iteration_ = event.metrics->iteration;
    }
    if (event.type == EventType::pathfinder_iteration_completed ||
        event.type == EventType::snapshot) {
      if (!iteration_history_.empty() &&
          iteration_history_.back().iteration == event.metrics->iteration) {
        iteration_history_.back() = *event.metrics;
      } else {
        iteration_history_.push_back(*event.metrics);
        constexpr std::size_t maximum_iteration_history = 4'096U;
        if (iteration_history_.size() > maximum_iteration_history) {
          const auto excess = static_cast<std::vector<IterationMetrics>::difference_type>(
              iteration_history_.size() - maximum_iteration_history);
          iteration_history_.erase(iteration_history_.begin(),
                                   iteration_history_.begin() + excess);
        }
      }
    }
  }
  if (event.counters) {
    counters_ = *event.counters;
  }
  collect_logged_diagnostics(event, logged_diagnostics_);
  // Counts prevent iterator and draw-loop pathologies; byte trimming below never removes
  // current/committed routes, selected vias, or source snapshots.
  const auto trim_map = [](auto& values, std::size_t maximum) {
    while (values.size() > maximum) values.erase(values.begin());
  };
  constexpr std::size_t maximum_dynamic_entities = 4'096U;
  trim_map(conflicts_, maximum_dynamic_entities);
  trim_map(resources_, maximum_dynamic_entities);
  trim_map(hydraulic_fields_, maximum_dynamic_entities);
  trim_map(priorities_, maximum_dynamic_entities);
  trim_geometry_to_budget();
  if (event.progress && !event.progress->phase.empty()) {
    progress_ = event.progress;
    progress_->phase = bounded_string(progress_->phase, 1024U);
  }

  if (event.metrics && (!previous_metrics || *previous_metrics != *event.metrics)) {
    last_metrics_time_us_ = event.monotonic_time_us;
  }
  if (geometry_updated) {
    last_geometry_time_us_ = event.monotonic_time_us;
  }

  switch (event.type) {
  case EventType::session_started:
    routing_active_ = false;
    active_net_.reset();
    active_group_.reset();
    progress_.reset();
    active_region_.reset();
    last_metrics_time_us_.reset();
    last_geometry_time_us_.reset();
    logged_diagnostics_.clear();
    status_ = bounded_string(activity_message(event));
    break;
  case EventType::snapshot:
    if (event.message == "Routing completed" || event.message == "Routing stopped") {
      routing_active_ = false;
      active_net_.reset();
      active_group_.reset();
      progress_.reset();
      active_region_.reset();
      status_ = bounded_string(event.message);
    }
    break;
  case EventType::routing_started:
    routing_active_ = true;
    progress_.reset();
    active_region_.reset();
    last_metrics_time_us_.reset();
    last_geometry_time_us_.reset();
    logged_diagnostics_.clear();
    status_ = event.message.empty() ? "Routing" : bounded_string(event.message);
    break;
  case EventType::routing_finished:
    routing_active_ = false;
    active_net_.reset();
    active_group_.reset();
    progress_.reset();
    active_region_.reset();
    logged_diagnostics_.clear();
    status_ = event.message.empty() ? "Routing finished" : bounded_string(event.message);
    break;
  case EventType::routing_completed:
    routing_active_ = false;
    active_net_.reset();
    active_group_.reset();
    progress_.reset();
    active_region_.reset();
    logged_diagnostics_.clear();
    status_ = event.message.empty() ? "Routing completed" : bounded_string(event.message);
    break;
  case EventType::copper_generated:
    status_ = event.message.empty() ? "Copper generated" : bounded_string(event.message);
    break;
  case EventType::net_started:
    status_ = bounded_string(activity_message(event));
    break;
  case EventType::net_completed:
  case EventType::net_failed:
    active_net_.reset();
    status_ = bounded_string(activity_message(event));
    break;
  case EventType::pathfinder_iteration_started:
    if (event.metrics) {
      iteration_ = event.metrics->iteration;
    }
    status_ = bounded_string(activity_message(event));
    break;
  case EventType::pathfinder_iteration_completed:
  case EventType::best_checkpoint_updated:
  case EventType::search_summary:
  case EventType::placement_proposal_accepted:
  case EventType::placement_proposal_rejected:
    status_ = bounded_string(activity_message(event));
    break;
  default:
    break;
  }

  append_activity(event);
}

void LiveStateHistory::clear() { *this = LiveStateHistory{}; }

void LiveStateHistory::append(const Event& event) {
  const bool followed_tail = position_ == size();
  events_.push_back(event);
  retained_bytes_ += estimated_event_bytes(event);
  tail_state_.apply(event);
  // The retained delta window is bounded; replaying at most that window is cheaper and safer
  // than retaining full geometry copies at every recovery snapshot.

  std::size_t remove_count = 0U;
  while (remove_count < events_.size() &&
         (events_.size() - remove_count > maximum_retained_events ||
          retained_bytes_ > maximum_retained_bytes)) {
    retained_bytes_ -= std::min(retained_bytes_, estimated_event_bytes(events_[remove_count]));
    ++remove_count;
  }
  if (remove_count != 0U) {
    for (std::size_t index = 0U; index < remove_count; ++index) {
      base_state_.apply(events_[index]);
    }
    events_.erase(events_.begin(), events_.begin() +
                                   static_cast<std::vector<Event>::difference_type>(remove_count));
    base_position_ += remove_count;
    dropped_events_ += remove_count;
    // All reduced snapshots before this point are subsumed by base_state_.
    if (position_ < base_position_) {
      state_ = base_state_;
      position_ = base_position_;
    }
  }
  if (followed_tail) {
    state_ = tail_state_;
    position_ = size();
  }
}

void LiveStateHistory::seek(std::size_t event_count) {
  event_count = std::clamp(event_count, base_position_, size());
  if (event_count == size()) {
    state_ = tail_state_;
    position_ = event_count;
    return;
  }
  std::size_t replay_position = base_position_;
  state_ = base_state_;
  while (replay_position < event_count) {
    state_.apply(events_[replay_position - base_position_]);
    ++replay_position;
  }
  position_ = event_count;
}

} // namespace flux::telemetry
