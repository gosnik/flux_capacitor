#include "flux/core/routability_knowledge.hpp"

#include "flux/core/detailed_router.hpp"
#include "flux/core/global_router.hpp"
#include "flux/core/regional_router.hpp"

#include <algorithm>
#include <limits>
#include <map>
#include <numeric>
#include <set>
#include <tuple>

namespace flux {
namespace {

[[nodiscard]] std::uint64_t saturating_add(std::uint64_t lhs, std::uint64_t rhs) {
  return lhs > std::numeric_limits<std::uint64_t>::max() - rhs
             ? std::numeric_limits<std::uint64_t>::max()
             : lhs + rhs;
}

[[nodiscard]] std::size_t saturating_count_add(std::size_t lhs, std::size_t rhs) {
  return lhs > std::numeric_limits<std::size_t>::max() - rhs
             ? std::numeric_limits<std::size_t>::max()
             : lhs + rhs;
}

[[nodiscard]] std::int64_t floor_divide(std::int64_t value, std::int64_t divisor) {
  const auto quotient = value / divisor;
  const auto remainder = value % divisor;
  return remainder < 0 ? quotient - 1 : quotient;
}

[[nodiscard]] geometry::AxisAlignedBounds point_bounds(Point point) { return {point, point}; }

[[nodiscard]] geometry::AxisAlignedBounds union_bounds(geometry::AxisAlignedBounds lhs,
                                                       geometry::AxisAlignedBounds rhs) {
  return {{std::min(lhs.minimum.x, rhs.minimum.x), std::min(lhs.minimum.y, rhs.minimum.y)},
          {std::max(lhs.maximum.x, rhs.maximum.x), std::max(lhs.maximum.y, rhs.maximum.y)}};
}

[[nodiscard]] geometry::AxisAlignedBounds segment_bounds(Point first, Point second) {
  return {{std::min(first.x, second.x), std::min(first.y, second.y)},
          {std::max(first.x, second.x), std::max(first.y, second.y)}};
}

[[nodiscard]] std::uint64_t clearance_severity(Nanometres deficit,
                                               const RoutabilityProbeConfig& config) {
  const auto quantum = std::max<std::int64_t>(1, config.clearance_severity_quantum.value());
  const auto value = static_cast<std::uint64_t>(std::max<std::int64_t>(0, deficit.value()));
  return saturating_add(1U, value / static_cast<std::uint64_t>(quantum));
}

// Every physical-feedback severity is expressed in the same clearance quantum.  A raw
// nanometre deficit is geometry, not a dimensionless PathFinder multiplier: feeding it directly
// into routability history made a 0.63 mm medial short worth 630,000 cost units instead of the
// roughly 13 physical-clearance quanta it represents.
[[nodiscard]] std::uint64_t physical_overflow_severity(std::uint64_t overflow_nm,
                                                       const RoutabilityProbeConfig& config) {
  const auto bounded = std::min<std::uint64_t>(
      overflow_nm, static_cast<std::uint64_t>(std::numeric_limits<std::int64_t>::max()));
  return clearance_severity(Nanometres{static_cast<std::int64_t>(bounded)}, config);
}

struct CoalescingKey {
  RoutabilityFailureKind kind{RoutabilityFailureKind::topological_trap};
  RoutabilityFailureStage stage{RoutabilityFailureStage::global_probe};
  RoutabilityRelaxation relaxation{RoutabilityRelaxation::replan_global_topology};
  LayerId layer;
  std::int64_t x{0};
  std::int64_t y{0};
  bool contributes_to_history{true};
  std::vector<NetId> nets;

  auto operator<=>(const CoalescingKey&) const = default;
};

class DiagnosisBuilder {
public:
  explicit DiagnosisBuilder(const RoutabilityProbeConfig& config) : config_(config) {}

  void add(RoutabilityDiagnosis diagnosis) {
    if (config_.max_diagnoses == 0U || config_.coalescing_pitch.value() <= 0) {
      ++dropped_;
      return;
    }
    auto& nets = diagnosis.resource.nets;
    std::sort(nets.begin(), nets.end());
    nets.erase(std::unique(nets.begin(), nets.end()), nets.end());
    const auto center_x = std::midpoint(diagnosis.resource.bounds.minimum.x.value(),
                                        diagnosis.resource.bounds.maximum.x.value());
    const auto center_y = std::midpoint(diagnosis.resource.bounds.minimum.y.value(),
                                        diagnosis.resource.bounds.maximum.y.value());
    const CoalescingKey key{diagnosis.kind,
                            diagnosis.stage,
                            diagnosis.suggested_relaxation,
                            diagnosis.resource.layer,
                            floor_divide(center_x, config_.coalescing_pitch.value()),
                            floor_divide(center_y, config_.coalescing_pitch.value()),
                            diagnosis.contributes_to_history,
                            nets};
    if (const auto found = diagnoses_.find(key); found != diagnoses_.end()) {
      auto& retained = found->second;
      retained.resource.bounds = union_bounds(retained.resource.bounds, diagnosis.resource.bounds);
      retained.resource.nets.insert(retained.resource.nets.end(), nets.begin(), nets.end());
      std::sort(retained.resource.nets.begin(), retained.resource.nets.end());
      retained.resource.nets.erase(
          std::unique(retained.resource.nets.begin(), retained.resource.nets.end()),
          retained.resource.nets.end());
      retained.severity = saturating_add(retained.severity, diagnosis.severity);
      retained.clearance_deficit =
          std::max(retained.clearance_deficit, diagnosis.clearance_deficit);
      if (diagnosis.planned_capacity) {
        retained.planned_capacity =
            saturating_add(retained.planned_capacity.value_or(0U), *diagnosis.planned_capacity);
      }
      if (diagnosis.observed_capacity) {
        retained.observed_capacity =
            saturating_add(retained.observed_capacity.value_or(0U), *diagnosis.observed_capacity);
      }
      ++coalesced_;
      return;
    }
    if (diagnoses_.size() >= config_.max_diagnoses) {
      ++dropped_;
      return;
    }
    diagnosis.resource.stable_id = "routability:" + std::string(to_string(diagnosis.kind)) + ':' +
                                   diagnosis.resource.layer.value() + ':' + std::to_string(key.x) +
                                   ':' + std::to_string(key.y);
    diagnoses_.emplace(key, std::move(diagnosis));
  }

  [[nodiscard]] std::vector<RoutabilityDiagnosis> finish() {
    std::vector<RoutabilityDiagnosis> result;
    result.reserve(diagnoses_.size());
    for (auto& [key, diagnosis] : diagnoses_) {
      static_cast<void>(key);
      result.push_back(std::move(diagnosis));
    }
    return result;
  }

  [[nodiscard]] std::size_t coalesced() const { return coalesced_; }
  [[nodiscard]] std::size_t dropped() const { return dropped_; }

private:
  RoutabilityProbeConfig config_;
  std::map<CoalescingKey, RoutabilityDiagnosis> diagnoses_;
  std::size_t coalesced_{0U};
  std::size_t dropped_{0U};
};

[[nodiscard]] const RegionalRoutingWindow* find_window(const RegionalRouteResult& regional,
                                                       std::size_t id) {
  const auto found = std::find_if(regional.windows.begin(), regional.windows.end(),
                                  [id](const auto& window) { return window.id == id; });
  return found == regional.windows.end() ? nullptr : &*found;
}

[[nodiscard]] RoutabilityDiagnosis regional_diagnosis(const RegionalRoutingWindow& window,
                                                      RoutabilityFailureKind kind,
                                                      RoutabilityFailureStage stage,
                                                      RoutabilityRelaxation relaxation,
                                                      std::uint64_t severity, std::string detail) {
  const auto layer = window.layers.empty() ? LayerId{} : window.layers.front();
  return RoutabilityDiagnosis{kind,
                              stage,
                              RoutabilityResourceReference{"", layer, window.bounds, window.nets},
                              std::max<std::uint64_t>(1U, severity),
                              window.peak_clearance_deficit,
                              std::nullopt,
                              std::nullopt,
                              relaxation,
                              std::move(detail),
                              false};
}

} // namespace

std::uint64_t RoutabilityKnowledge::pressure() const {
  std::uint64_t result = 0U;
  for (const auto& entry : history) {
    result = saturating_add(result, entry.pressure);
  }
  return result;
}

std::string_view to_string(RoutabilityFailureKind kind) {
  switch (kind) {
  case RoutabilityFailureKind::packing_capacity_exceeded:
    return "packing_capacity_exceeded";
  case RoutabilityFailureKind::terminal_escape_ordering:
    return "terminal_escape_ordering";
  case RoutabilityFailureKind::via_site_exhausted:
    return "via_site_exhausted";
  case RoutabilityFailureKind::curvature_infeasible:
    return "curvature_infeasible";
  case RoutabilityFailureKind::shove_deadlock:
    return "shove_deadlock";
  case RoutabilityFailureKind::quantization_clearance:
    return "quantization_clearance";
  case RoutabilityFailureKind::topological_trap:
    return "topological_trap";
  case RoutabilityFailureKind::layer_transition_conflict:
    return "layer_transition_conflict";
  }
  return "topological_trap";
}

std::string_view to_string(RoutabilityFailureStage stage) {
  switch (stage) {
  case RoutabilityFailureStage::global_probe:
    return "global_probe";
  case RoutabilityFailureStage::regional_discrete:
    return "regional_discrete";
  case RoutabilityFailureStage::spline_realization:
    return "spline_realization";
  case RoutabilityFailureStage::local_pns:
    return "local_pns";
  case RoutabilityFailureStage::quantization:
    return "quantization";
  case RoutabilityFailureStage::final_validation:
    return "final_validation";
  }
  return "global_probe";
}

std::string_view to_string(RoutabilityRelaxation relaxation) {
  switch (relaxation) {
  case RoutabilityRelaxation::reduce_resource_demand:
    return "reduce_resource_demand";
  case RoutabilityRelaxation::change_terminal_order:
    return "change_terminal_order";
  case RoutabilityRelaxation::change_via_site:
    return "change_via_site";
  case RoutabilityRelaxation::change_lane_order:
    return "change_lane_order";
  case RoutabilityRelaxation::change_layer_sequence:
    return "change_layer_sequence";
  case RoutabilityRelaxation::change_corridor:
    return "change_corridor";
  case RoutabilityRelaxation::increase_local_clearance_margin:
    return "increase_local_clearance_margin";
  case RoutabilityRelaxation::replan_global_topology:
    return "replan_global_topology";
  }
  return "replan_global_topology";
}

std::vector<RoutabilityDiagnosis> probe_global_routability(const GlobalRouteResult& global,
                                                           const RoutabilityProbeConfig& config) {
  DiagnosisBuilder builder{config};
  for (const auto& conflict : global.clearance_shadow_conflicts) {
    auto bounds = segment_bounds(conflict.first.position, conflict.second.position);
    if (conflict.shadow_owner_first) {
      bounds = union_bounds(bounds, point_bounds(conflict.shadow_owner_first->position));
    }
    if (conflict.shadow_owner_second) {
      bounds = union_bounds(bounds, point_bounds(conflict.shadow_owner_second->position));
    }
    builder.add(RoutabilityDiagnosis{
        RoutabilityFailureKind::packing_capacity_exceeded, RoutabilityFailureStage::global_probe,
        RoutabilityResourceReference{
            "", conflict.first.layer, bounds, {conflict.first_net, conflict.second_net}},
        clearance_severity(conflict.clearance_deficit, config), conflict.clearance_deficit, 2U, 1U,
        RoutabilityRelaxation::change_corridor,
        "selected copper envelopes overlap despite sampled-resource compatibility", false});
  }
  for (const auto& resource : global.resource_usage) {
    if (!resource.congested()) {
      continue;
    }
    builder.add(RoutabilityDiagnosis{
        RoutabilityFailureKind::packing_capacity_exceeded,
        RoutabilityFailureStage::global_probe,
        RoutabilityResourceReference{
            "", resource.first.layer,
            segment_bounds(resource.first.position, resource.second.position), resource.nets},
        static_cast<std::uint64_t>(resource.demand - resource.capacity),
        {},
        static_cast<std::uint64_t>(resource.demand),
        static_cast<std::uint64_t>(resource.capacity),
        RoutabilityRelaxation::reduce_resource_demand,
        "sampled resource demand exceeds represented capacity",
        false});
  }
  for (const auto& corridor : global.corridor_usage) {
    if (!corridor.congested()) {
      continue;
    }
    builder.add(RoutabilityDiagnosis{
        RoutabilityFailureKind::packing_capacity_exceeded,
        RoutabilityFailureStage::global_probe,
        RoutabilityResourceReference{
            "", corridor.layer,
            segment_bounds(corridor.sampled_medial_start, corridor.sampled_medial_end),
            corridor.nets},
        static_cast<std::uint64_t>(corridor.demand - corridor.capacity),
        {},
        static_cast<std::uint64_t>(corridor.demand),
        static_cast<std::uint64_t>(corridor.capacity),
        RoutabilityRelaxation::change_corridor,
        "corridor lane demand exceeds its conservative cross-section capacity",
        false});
  }
  for (const auto& resource : global.voronoi_resource_usage) {
    if (!resource.congested()) {
      continue;
    }
    builder.add(RoutabilityDiagnosis{
        RoutabilityFailureKind::packing_capacity_exceeded,
        RoutabilityFailureStage::global_probe,
        RoutabilityResourceReference{"", resource.layer,
                                     segment_bounds(resource.start, resource.end), resource.nets},
        physical_overflow_severity(resource.demand_nm - resource.capacity_nm, config),
        {},
        resource.demand_nm,
        resource.capacity_nm,
        RoutabilityRelaxation::change_corridor,
        "physical corridor envelope demand exceeds certified width",
        false});
  }
  // A packed medial bridge can fail even where its individual resource envelopes have no
  // overflow or clearance shadow.  Attribute that topological failure to every occupant of the
  // two participating resources so an incremental feedback pass expands from the transition net
  // to the real local lane cone.  This is deliberately diagnostic evidence rather than a second
  // generic-history penalty: selected-lane transition history, exact bridge checks, and detailed
  // DRC remain the authorities on legality.
  for (const auto& transition : global.voronoi_lane_transitions) {
    if (transition.static_clearance_checked && transition.mutual_clearance_checked) {
      continue;
    }
    std::vector<NetId> nets{transition.net};
    for (const auto resource_id : {transition.first_resource_id, transition.second_resource_id}) {
      const auto resource =
          std::find_if(global.voronoi_resource_usage.begin(), global.voronoi_resource_usage.end(),
                       [resource_id](const auto& usage) { return usage.id == resource_id; });
      if (resource != global.voronoi_resource_usage.end()) {
        nets.insert(nets.end(), resource->nets.begin(), resource->nets.end());
      }
    }
    std::sort(nets.begin(), nets.end());
    nets.erase(std::unique(nets.begin(), nets.end()), nets.end());
    auto bounds = segment_bounds(transition.first_lane_endpoint, transition.second_lane_endpoint);
    bounds = union_bounds(bounds, point_bounds(transition.medial_junction));
    builder.add(RoutabilityDiagnosis{
        RoutabilityFailureKind::packing_capacity_exceeded,
        RoutabilityFailureStage::global_probe,
        RoutabilityResourceReference{"", transition.layer, bounds, std::move(nets)},
        1U,
        {},
        std::nullopt,
        std::nullopt,
        RoutabilityRelaxation::change_lane_order,
        "selected physical-medial lane bridge has no exact static/mutual realization",
        false,
        false});
  }
  for (const auto& failure : global.pin_escape_failures) {
    builder.add(RoutabilityDiagnosis{
        RoutabilityFailureKind::terminal_escape_ordering,
        RoutabilityFailureStage::global_probe,
        RoutabilityResourceReference{"", {}, point_bounds(failure.position), {failure.net}},
        1U,
        {},
        std::nullopt,
        std::nullopt,
        RoutabilityRelaxation::change_terminal_order,
        "terminal has no exact legal access portal",
        false});
  }
  for (const auto& failure : global.connection_failures) {
    auto bounds = geometry::AxisAlignedBounds{};
    bool has_bounds = false;
    for (const auto& terminal : failure.attempted_terminals) {
      bounds = has_bounds ? union_bounds(bounds, point_bounds(terminal.position))
                          : point_bounds(terminal.position);
      has_bounds = true;
    }
    if (!has_bounds) {
      continue;
    }
    const auto terminal_failure = failure.reason == RoutingConnectionFailureReason::no_access_nodes;
    builder.add(RoutabilityDiagnosis{
        terminal_failure ? RoutabilityFailureKind::terminal_escape_ordering
                         : RoutabilityFailureKind::topological_trap,
        RoutabilityFailureStage::global_probe,
        RoutabilityResourceReference{"", {}, bounds, {failure.net}},
        std::max<std::uint64_t>(1U, failure.topology_rejected_edges),
        {},
        std::nullopt,
        std::nullopt,
        terminal_failure ? RoutabilityRelaxation::change_terminal_order
                         : RoutabilityRelaxation::replan_global_topology,
        terminal_failure ? "terminal access search exhausted"
                         : "global connection search exhausted its legal topology",
        false});
  }
  return builder.finish();
}

std::vector<RoutabilityDiagnosis>
diagnose_regional_routability(const RegionalRouteResult& regional,
                              const RoutabilityProbeConfig& config) {
  DiagnosisBuilder builder{config};
  for (const auto& diagnosis : regional.local_routability_diagnoses) {
    if (!diagnosis.locally_resolved) {
      builder.add(diagnosis);
    }
  }
  if (regional.configured.enable_xpbd_feedback) {
    for (const auto& invocation : regional.xpbd_invocations) {
      if (invocation.accepted && invocation.final_violations == 0U)
        continue;
      const auto* window = find_window(regional, invocation.window_id);
      if (window == nullptr)
        continue;
      auto diagnosis = regional_diagnosis(
          *window, RoutabilityFailureKind::shove_deadlock, RoutabilityFailureStage::local_pns,
          invocation.topology_escape_required ? RoutabilityRelaxation::change_corridor
                                              : RoutabilityRelaxation::change_lane_order,
          std::max<std::uint64_t>(1U, invocation.final_violations),
          "joint XPBD relaxation retained physical constraint stress; peak residual " +
              std::to_string(invocation.final_peak_residual.value()) + " nm");
      diagnosis.clearance_deficit = invocation.final_peak_residual;
      // Per-cell diagnoses already carry the spatial history charge. Retain this invocation-wide
      // summary for attribution without double-counting the same physical stress.
      diagnosis.contributes_to_history = false;
      builder.add(std::move(diagnosis));
    }
  }
  for (const auto& assignment : regional.bundle_assignments) {
    if (assignment.complete()) {
      continue;
    }
    const auto* window = find_window(regional, assignment.window_id);
    if (window == nullptr) {
      continue;
    }
    for (const auto& blocker : assignment.partial_access_blockers) {
      const auto via = blocker.kind == RegionalAccessBlockerKind::via_pair;
      builder.add(
          regional_diagnosis(*window,
                             via ? RoutabilityFailureKind::via_site_exhausted
                                 : RoutabilityFailureKind::layer_transition_conflict,
                             RoutabilityFailureStage::regional_discrete,
                             via ? RoutabilityRelaxation::change_via_site
                                 : RoutabilityRelaxation::change_layer_sequence,
                             blocker.observations,
                             via ? "joint access assignment exhausted compatible via pairs"
                                 : "terminal and via run-up choices are mutually incompatible"));
    }
    switch (assignment.stop_reason) {
    case RegionalBundleStopReason::complete:
      break;
    case RegionalBundleStopReason::missing_corridor_topology:
      builder.add(regional_diagnosis(*window, RoutabilityFailureKind::topological_trap,
                                     RoutabilityFailureStage::regional_discrete,
                                     RoutabilityRelaxation::change_corridor, 1U,
                                     "no complete corridor topology exists for the window"));
      break;
    case RegionalBundleStopReason::no_fitting_lane_order:
      builder.add(regional_diagnosis(
          *window, RoutabilityFailureKind::packing_capacity_exceeded,
          RoutabilityFailureStage::regional_discrete, RoutabilityRelaxation::change_lane_order,
          std::max<std::uint64_t>(1U, assignment.packing_rejected_states),
          "no retained lane ordering fits every selected cross section"));
      break;
    case RegionalBundleStopReason::window_ineligible:
      builder.add(regional_diagnosis(*window, RoutabilityFailureKind::topological_trap,
                                     RoutabilityFailureStage::regional_discrete,
                                     RoutabilityRelaxation::replan_global_topology, 1U,
                                     "regional window violates its bounded decomposition policy"));
      break;
    case RegionalBundleStopReason::state_budget_exhausted:
      builder.add(regional_diagnosis(
          *window, RoutabilityFailureKind::packing_capacity_exceeded,
          RoutabilityFailureStage::regional_discrete, RoutabilityRelaxation::change_corridor,
          std::max<std::uint64_t>(1U, assignment.packing_rejected_states),
          "bounded joint bundle/lane search exhausted before finding a complete state"));
      break;
    case RegionalBundleStopReason::spline_realization_failed: {
      const auto topology = static_cast<std::uint64_t>(std::count_if(
          assignment.spline_realizations.begin(), assignment.spline_realizations.end(),
          [](const auto& realization) { return !realization.topology_legal; }));
      const auto mutual = static_cast<std::uint64_t>(std::count_if(
          assignment.spline_realizations.begin(), assignment.spline_realizations.end(),
          [](const auto& realization) { return !realization.mutually_legal; }));
      const auto quantization = static_cast<std::uint64_t>(std::count_if(
          assignment.spline_realizations.begin(), assignment.spline_realizations.end(),
          [](const auto& realization) { return !realization.quantized_mutually_legal; }));
      if (topology > 0U) {
        builder.add(regional_diagnosis(
            *window, RoutabilityFailureKind::topological_trap,
            RoutabilityFailureStage::spline_realization, RoutabilityRelaxation::change_corridor,
            topology, "quantized spline samples leave the selected corridor topology"));
      }
      if (mutual > 0U) {
        builder.add(regional_diagnosis(
            *window, RoutabilityFailureKind::shove_deadlock, RoutabilityFailureStage::local_pns,
            RoutabilityRelaxation::change_lane_order, mutual,
            "bounded elastic/PNS optimization retained mutual copper-envelope conflicts"));
      }
      if (quantization > 0U && topology == 0U && mutual == 0U) {
        builder.add(regional_diagnosis(
            *window, RoutabilityFailureKind::quantization_clearance,
            RoutabilityFailureStage::quantization,
            RoutabilityRelaxation::increase_local_clearance_margin, quantization,
            "quantized spline polylines violate an otherwise continuous route envelope"));
      }
      const auto has_local_unresolved =
          std::any_of(regional.local_routability_diagnoses.begin(),
                      regional.local_routability_diagnoses.end(), [&](const auto& diagnosis) {
                        return !diagnosis.locally_resolved &&
                               diagnosis.resource.stable_id.starts_with(
                                   "regional-window:" + std::to_string(window->id) + ":");
                      });
      if (topology == 0U && mutual == 0U && quantization == 0U && !has_local_unresolved) {
        builder.add(regional_diagnosis(
            *window, RoutabilityFailureKind::curvature_infeasible,
            RoutabilityFailureStage::spline_realization, RoutabilityRelaxation::change_corridor, 1U,
            "selected regional state could not produce a complete quintic PH realization"));
      }
      break;
    }
    }
  }
  return builder.finish();
}

std::vector<RoutabilityDiagnosis>
diagnose_detailed_routability(const DetailedRouteResult& detailed,
                              const RoutabilityProbeConfig& config) {
  DiagnosisBuilder builder{config};
  if (detailed.complete() || detailed.provisional_zone_refill_candidate()) {
    return {};
  }
  const auto resource_for = [&](const std::vector<std::string>& object_ids,
                                const std::optional<NetId>& selected_net = std::nullopt) {
    RoutabilityResourceReference resource;
    bool has_bounds = false;
    bool has_layer = false;
    const auto add = [&](Point point, const LayerId& layer, const NetId& net) {
      resource.bounds =
          has_bounds ? union_bounds(resource.bounds, point_bounds(point)) : point_bounds(point);
      has_bounds = true;
      if (!has_layer) {
        resource.layer = layer;
        has_layer = true;
      } else if (resource.layer != layer) {
        resource.layer = {};
      }
      resource.nets.push_back(net);
    };
    if (detailed.candidate) {
      const auto selected = [&](const ObjectId& id, const NetId& net) {
        return (selected_net && net == *selected_net) ||
               std::find(object_ids.begin(), object_ids.end(), id.value()) != object_ids.end();
      };
      for (const auto& track : detailed.candidate->tracks) {
        if (selected(track.id, track.net)) {
          add(track.start, track.layer, track.net);
          add(track.end, track.layer, track.net);
        }
      }
      for (const auto& arc : detailed.candidate->track_arcs) {
        if (selected(arc.id, arc.net)) {
          add(arc.start, arc.layer, arc.net);
          add(arc.middle, arc.layer, arc.net);
          add(arc.end, arc.layer, arc.net);
        }
      }
      for (const auto& via : detailed.candidate->vias) {
        if (!selected(via.id, via.net)) {
          continue;
        }
        if (via.layers.empty()) {
          add(via.position, {}, via.net);
        } else {
          for (const auto& layer : via.layers) {
            add(via.position, layer, via.net);
          }
        }
      }
      for (const auto& footprint : detailed.candidate->footprints) {
        for (const auto& pad : footprint.pads) {
          if (!selected(pad.id, pad.net)) {
            continue;
          }
          const auto position = geometry::transform_point(pad.local_position, footprint.position,
                                                          footprint.rotation, footprint.side);
          if (pad.layers.empty()) {
            add(position, {}, pad.net);
          } else {
            for (const auto& layer : pad.layers) {
              add(position, layer, pad.net);
            }
          }
        }
      }
    }
    std::sort(resource.nets.begin(), resource.nets.end());
    resource.nets.erase(std::unique(resource.nets.begin(), resource.nets.end()),
                        resource.nets.end());
    if (!has_bounds) {
      resource.bounds = {};
    }
    return resource;
  };
  for (const auto& net : detailed.connectivity.nets) {
    if (net.open_connections == 0U) {
      continue;
    }
    auto resource = resource_for({}, net.net);
    resource.stable_id = "detailed-connectivity:" + net.net.value();
    builder.add(RoutabilityDiagnosis{
        RoutabilityFailureKind::topological_trap,
        RoutabilityFailureStage::final_validation,
        std::move(resource),
        std::max<std::uint64_t>(1U, static_cast<std::uint64_t>(net.open_connections)),
        {},
        std::nullopt,
        std::nullopt,
        RoutabilityRelaxation::replan_global_topology,
        "quantized copper leaves " + std::to_string(net.open_connections) + " open connection(s)",
        false});
  }
  for (const auto& violation : detailed.copper_drc.violations) {
    auto resource = resource_for(violation.object_ids);
    resource.stable_id = "detailed-drc:" + violation.code;
    for (const auto& object : violation.object_ids) {
      resource.stable_id += ':' + object;
    }
    const auto deficit_value =
        std::max<long double>(0.0L, static_cast<long double>(violation.required_clearance.value()) -
                                        violation.actual_distance_nm);
    const auto bounded_deficit = Nanometres{static_cast<std::int64_t>(std::ceil(std::min(
        deficit_value, static_cast<long double>(std::numeric_limits<std::int64_t>::max()))))};
    const auto edge_or_keepout = violation.kind == CopperViolationKind::copper_edge_clearance ||
                                 violation.kind == CopperViolationKind::copper_outside_board ||
                                 violation.kind == CopperViolationKind::keepout;
    const auto skew = violation.kind == CopperViolationKind::differential_pair_skew;
    const auto via_conflict = violation.kind == CopperViolationKind::track_via_clearance ||
                              violation.kind == CopperViolationKind::via_clearance ||
                              violation.kind == CopperViolationKind::via_pad_clearance ||
                              violation.kind == CopperViolationKind::via_zone_clearance;
    const auto terminal_conflict = violation.kind == CopperViolationKind::track_pad_clearance ||
                                   violation.kind == CopperViolationKind::pad_clearance;
    builder.add(RoutabilityDiagnosis{
        edge_or_keepout     ? RoutabilityFailureKind::topological_trap
        : skew              ? RoutabilityFailureKind::curvature_infeasible
        : via_conflict      ? RoutabilityFailureKind::via_site_exhausted
        : terminal_conflict ? RoutabilityFailureKind::terminal_escape_ordering
                            : RoutabilityFailureKind::packing_capacity_exceeded,
        RoutabilityFailureStage::final_validation, std::move(resource),
        clearance_severity(bounded_deficit, config), bounded_deficit, std::nullopt, std::nullopt,
        edge_or_keepout     ? RoutabilityRelaxation::change_corridor
        : skew              ? RoutabilityRelaxation::change_lane_order
        : via_conflict      ? RoutabilityRelaxation::change_via_site
        : terminal_conflict ? RoutabilityRelaxation::change_terminal_order
                            : RoutabilityRelaxation::change_lane_order,
        "exact detailed copper check rejected " + violation.code, false});
  }
  if (detailed.connectivity.open_connections == 0U && detailed.copper_drc.ok() &&
      detailed.stop_reason != DetailedRouterStopReason::cancelled) {
    RoutabilityResourceReference resource;
    resource.stable_id = "detailed-transaction:" + detailed.diagnostic;
    builder.add(RoutabilityDiagnosis{detailed.stop_reason == DetailedRouterStopReason::invalid_path
                                         ? RoutabilityFailureKind::topological_trap
                                         : RoutabilityFailureKind::packing_capacity_exceeded,
                                     RoutabilityFailureStage::final_validation,
                                     std::move(resource),
                                     1U,
                                     {},
                                     std::nullopt,
                                     std::nullopt,
                                     RoutabilityRelaxation::replan_global_topology,
                                     detailed.diagnostic,
                                     false});
  }
  return builder.finish();
}

void accumulate_routability_knowledge(RoutabilityKnowledge& knowledge,
                                      const std::vector<RoutabilityDiagnosis>& diagnoses,
                                      const RoutabilityProbeConfig& config) {
  for (const auto& diagnosis : diagnoses) {
    if (knowledge.diagnoses.size() < config.max_diagnoses) {
      knowledge.diagnoses.push_back(diagnosis);
    } else {
      knowledge.dropped_diagnoses = saturating_count_add(knowledge.dropped_diagnoses, 1U);
    }
    if (!diagnosis.contributes_to_history) {
      continue;
    }
    const auto found =
        std::find_if(knowledge.history.begin(), knowledge.history.end(), [&](const auto& entry) {
          return entry.kind == diagnosis.kind && entry.stage == diagnosis.stage &&
                 entry.suggested_relaxation == diagnosis.suggested_relaxation &&
                 entry.resource.stable_id == diagnosis.resource.stable_id;
        });
    if (found != knowledge.history.end()) {
      found->observations = saturating_count_add(found->observations, 1U);
      found->pressure = saturating_add(found->pressure, diagnosis.severity);
      found->resource.bounds = union_bounds(found->resource.bounds, diagnosis.resource.bounds);
      found->resource.nets.insert(found->resource.nets.end(), diagnosis.resource.nets.begin(),
                                  diagnosis.resource.nets.end());
      std::sort(found->resource.nets.begin(), found->resource.nets.end());
      found->resource.nets.erase(
          std::unique(found->resource.nets.begin(), found->resource.nets.end()),
          found->resource.nets.end());
      knowledge.coalesced_diagnoses = saturating_count_add(knowledge.coalesced_diagnoses, 1U);
      continue;
    }
    if (knowledge.history.size() >= config.max_history_entries) {
      knowledge.dropped_history_entries =
          saturating_count_add(knowledge.dropped_history_entries, 1U);
      continue;
    }
    knowledge.history.push_back(RoutabilityHistoryEntry{diagnosis.kind, diagnosis.resource, 1U,
                                                        diagnosis.severity, diagnosis.stage,
                                                        diagnosis.suggested_relaxation});
  }
  std::sort(knowledge.diagnoses.begin(), knowledge.diagnoses.end(),
            [](const auto& lhs, const auto& rhs) {
              return std::tie(lhs.stage, lhs.kind, lhs.resource.stable_id, lhs.detail) <
                     std::tie(rhs.stage, rhs.kind, rhs.resource.stable_id, rhs.detail);
            });
  std::sort(
      knowledge.history.begin(), knowledge.history.end(), [](const auto& lhs, const auto& rhs) {
        return std::tie(lhs.stage, lhs.kind, lhs.suggested_relaxation, lhs.resource.stable_id) <
               std::tie(rhs.stage, rhs.kind, rhs.suggested_relaxation, rhs.resource.stable_id);
      });
}

void accumulate_clean_copper_knowledge(RoutabilityKnowledge& knowledge,
                                       const DetailedRouteResult& detailed,
                                       const RoutabilityProbeConfig& config) {
  if (!detailed.candidate || config.max_history_entries == 0U ||
      config.coalescing_pitch.value() <= 0) {
    return;
  }
  std::set<std::string> rejected_objects;
  for (const auto& violation : detailed.copper_drc.violations) {
    rejected_objects.insert(violation.object_ids.begin(), violation.object_ids.end());
  }
  const auto add = [&](LayerId layer, NetId net, Point start, Point end) {
    const auto bounds = segment_bounds(start, end);
    auto first_x = floor_divide(start.x.value(), config.coalescing_pitch.value());
    auto first_y = floor_divide(start.y.value(), config.coalescing_pitch.value());
    auto last_x = floor_divide(end.x.value(), config.coalescing_pitch.value());
    auto last_y = floor_divide(end.y.value(), config.coalescing_pitch.value());
    if (std::tie(last_x, last_y) < std::tie(first_x, first_y)) {
      std::swap(first_x, last_x);
      std::swap(first_y, last_y);
    }
    const auto stable_id = "clean-copper:" + layer.value() + ':' + net.value() + ':' +
                           std::to_string(first_x) + ':' + std::to_string(first_y) + ':' +
                           std::to_string(last_x) + ':' + std::to_string(last_y);
    const auto found =
        std::find_if(knowledge.success_history.begin(), knowledge.success_history.end(),
                     [&](const auto& entry) { return entry.resource.stable_id == stable_id; });
    if (found != knowledge.success_history.end()) {
      found->observations = saturating_count_add(found->observations, 1U);
      // Successful corridors accumulate deliberately slowly and remain capped by the hydraulic
      // solver's maximum erosion when rasterized.
      found->strength = saturating_add(found->strength, 1U);
      found->resource.bounds = union_bounds(found->resource.bounds, bounds);
      found->start = start;
      found->end = end;
      return;
    }
    if (knowledge.success_history.size() >= config.max_history_entries) {
      knowledge.dropped_history_entries =
          saturating_count_add(knowledge.dropped_history_entries, 1U);
      return;
    }
    knowledge.success_history.push_back(RoutabilitySuccessEntry{
        RoutabilityResourceReference{stable_id, layer, bounds, {net}}, start, end, 1U, 1U});
  };
  for (const auto& track : detailed.candidate->tracks) {
    if (!rejected_objects.contains(track.id.value())) {
      add(track.layer, track.net, track.start, track.end);
    }
  }
  for (const auto& arc : detailed.candidate->track_arcs) {
    if (rejected_objects.contains(arc.id.value()))
      continue;
    add(arc.layer, arc.net, arc.start, arc.middle);
    add(arc.layer, arc.net, arc.middle, arc.end);
  }
  std::sort(knowledge.success_history.begin(), knowledge.success_history.end(),
            [](const auto& lhs, const auto& rhs) {
              return lhs.resource.stable_id < rhs.resource.stable_id;
            });
}

} // namespace flux
