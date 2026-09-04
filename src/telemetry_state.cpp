#include "flux/telemetry/state.hpp"

#include <algorithm>
#include <iterator>
#include <sstream>

namespace flux::telemetry {
namespace {

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

} // namespace

void LiveState::clear() { *this = LiveState{}; }

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
  activity_.push_back(Activity{event.sequence, event.monotonic_time_us, event.type, event.level,
                               event_net(event), activity_message(event)});
  constexpr std::size_t maximum_activity = 512;
  while (activity_.size() > maximum_activity) {
    activity_.pop_front();
  }
}

void LiveState::apply(const Event& event) {
  if (sequence_ != 0U && event.sequence > sequence_ + 1U) {
    observed_sequence_gaps_ += event.sequence - sequence_ - 1U;
  }
  sequence_ = std::max(sequence_, event.sequence);
  if (!event.source_identity.empty()) {
    source_identity_ = event.source_identity;
  }
  if (event.configuration_revision != 0U) {
    configuration_revision_ = event.configuration_revision;
  }
  if (event.active_net) {
    if (event.active_net->empty()) {
      active_net_.reset();
    } else {
      active_net_ = event.active_net;
    }
  }
  if (event.active_group) {
    if (event.active_group->empty()) {
      active_group_.reset();
    } else {
      active_group_ = event.active_group;
    }
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
    metrics_ = event.metrics;
    iteration_ = event.metrics->iteration;
    if (event.type == EventType::pathfinder_iteration_completed ||
        event.type == EventType::snapshot) {
      if (!iteration_history_.empty() &&
          iteration_history_.back().iteration == event.metrics->iteration) {
        iteration_history_.back() = *event.metrics;
      } else {
        iteration_history_.push_back(*event.metrics);
      }
    }
  }
  if (event.counters) {
    counters_ = *event.counters;
  }

  switch (event.type) {
  case EventType::snapshot:
    if (event.message == "Routing finished") {
      routing_active_ = false;
      active_net_.reset();
      active_group_.reset();
      status_ = event.message;
    }
    break;
  case EventType::routing_started:
    routing_active_ = true;
    status_ = event.message.empty() ? "Routing" : event.message;
    break;
  case EventType::routing_finished:
    routing_active_ = false;
    active_net_.reset();
    active_group_.reset();
    status_ = event.message.empty() ? "Routing finished" : event.message;
    break;
  case EventType::net_started:
    status_ = activity_message(event);
    break;
  case EventType::net_completed:
  case EventType::net_failed:
    active_net_.reset();
    status_ = activity_message(event);
    break;
  case EventType::pathfinder_iteration_started:
    if (event.metrics) {
      iteration_ = event.metrics->iteration;
    }
    status_ = activity_message(event);
    break;
  case EventType::pathfinder_iteration_completed:
  case EventType::best_checkpoint_updated:
  case EventType::search_summary:
  case EventType::placement_proposal_accepted:
  case EventType::placement_proposal_rejected:
    status_ = activity_message(event);
    break;
  default:
    break;
  }

  append_activity(event);
}

void LiveStateHistory::clear() { *this = LiveStateHistory{}; }

void LiveStateHistory::append(const Event& event) {
  const bool followed_tail = position_ == events_.size();
  events_.push_back(event);
  tail_state_.apply(event);
  if (events_.size() % checkpoint_interval == 0U || event.type == EventType::snapshot) {
    if (!checkpoints_.empty() && checkpoints_.back().position == events_.size()) {
      checkpoints_.back().state = tail_state_;
    } else {
      checkpoints_.push_back(Checkpoint{events_.size(), tail_state_});
    }
  }
  if (followed_tail) {
    state_ = tail_state_;
    position_ = events_.size();
  }
}

void LiveStateHistory::seek(std::size_t event_count) {
  event_count = std::min(event_count, events_.size());
  if (event_count == events_.size()) {
    state_ = tail_state_;
    position_ = event_count;
    return;
  }
  const auto checkpoint = std::upper_bound(checkpoints_.begin(), checkpoints_.end(), event_count,
                                           [](const auto position, const Checkpoint& candidate) {
                                             return position < candidate.position;
                                           });
  std::size_t replay_position = 0U;
  state_.clear();
  if (checkpoint != checkpoints_.begin()) {
    const auto& selected = *std::prev(checkpoint);
    state_ = selected.state;
    replay_position = selected.position;
  }
  while (replay_position < event_count) {
    state_.apply(events_[replay_position]);
    ++replay_position;
  }
  position_ = event_count;
}

} // namespace flux::telemetry
