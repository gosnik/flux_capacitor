#include "flux/core/quantized_copper_oracle.hpp"

#include "flux/core/copper_geometry.hpp"
#include "flux/core/routing_rules.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <map>
#include <numeric>
#include <set>
#include <string>
#include <tuple>
#include <utility>

namespace flux {
namespace {

struct PrimitiveEvidence {
  NetId net;
  std::size_t connection_index{0U};
  LayerId layer;
  geometry::AxisAlignedBounds bounds;
  bool via{false};
  std::optional<RoutabilitySourceSegment> source_segment{std::nullopt};
  std::vector<RoutabilitySourceSegment> via_access_segments{};
};

// Dynamic copper is the repair owner.  Static drill/via geometry is nevertheless required to
// localize a diagnosis caused by that immutable obstacle; otherwise the generated trace's full
// AABB becomes feedback terrain.
struct StaticObstacleEvidence {
  std::optional<geometry::AxisAlignedBounds> copper_bounds;
  std::optional<geometry::AxisAlignedBounds> drill_bounds;
};

[[nodiscard]] Point quantize(Point point, Nanometres pitch) {
  const auto snap = [step = pitch.value()](std::int64_t value) {
    auto quotient = value / step;
    const auto remainder = value % step;
    const auto magnitude = remainder < 0 ? -remainder : remainder;
    if (magnitude >= (step + 1) / 2) {
      quotient += value < 0 ? -1 : 1;
    }
    return quotient * step;
  };
  return Point{Nanometres{snap(point.x.value())}, Nanometres{snap(point.y.value())}};
}

[[nodiscard]] geometry::AxisAlignedBounds expanded_segment_bounds(Point first, Point second,
                                                                   Nanometres width) {
  const auto radius = width.value() / 2;
  return geometry::AxisAlignedBounds{
      Point{Nanometres{std::min(first.x.value(), second.x.value()) - radius},
            Nanometres{std::min(first.y.value(), second.y.value()) - radius}},
      Point{Nanometres{std::max(first.x.value(), second.x.value()) + radius},
            Nanometres{std::max(first.y.value(), second.y.value()) + radius}}};
}

[[nodiscard]] geometry::AxisAlignedBounds disc_bounds(Point point, Nanometres diameter) {
  const auto radius = diameter.value() / 2;
  return geometry::AxisAlignedBounds{
      Point{Nanometres{point.x.value() - radius}, Nanometres{point.y.value() - radius}},
      Point{Nanometres{point.x.value() + radius}, Nanometres{point.y.value() + radius}}};
}

[[nodiscard]] geometry::AxisAlignedBounds conservative_drill_bounds(const DrillShape& drill) {
  // drill_shapes() owns the pad drill-offset and front/back rotation transform.  Its clearance
  // shape also supplies the conservative radius for arbitrary oval drill orientation.
  const auto shape = drill_clearance_shape(drill);
  const auto radius = static_cast<std::int64_t>(std::ceil(shape.conservative_radius_nm));
  return geometry::AxisAlignedBounds{
      Point{Nanometres{shape.position.x.value() - radius},
            Nanometres{shape.position.y.value() - radius}},
      Point{Nanometres{shape.position.x.value() + radius},
            Nanometres{shape.position.y.value() + radius}}};
}

[[nodiscard]] geometry::AxisAlignedBounds
union_bounds(const geometry::AxisAlignedBounds& first, const geometry::AxisAlignedBounds& second) {
  return geometry::AxisAlignedBounds{
      Point{std::min(first.minimum.x, second.minimum.x),
            std::min(first.minimum.y, second.minimum.y)},
      Point{std::max(first.maximum.x, second.maximum.x),
            std::max(first.maximum.y, second.maximum.y)}};
}

[[nodiscard]] geometry::AxisAlignedBounds
localized_overlap_bounds(const geometry::AxisAlignedBounds& first,
                         const geometry::AxisAlignedBounds& second, Nanometres deficit) {
  const auto margin = std::max<std::int64_t>(1, deficit.value() / 2 + 1);
  const auto expand = [margin](const auto& bounds) {
    return geometry::AxisAlignedBounds{
        Point{Nanometres{bounds.minimum.x.value() - margin},
              Nanometres{bounds.minimum.y.value() - margin}},
        Point{Nanometres{bounds.maximum.x.value() + margin},
              Nanometres{bounds.maximum.y.value() + margin}}};
  };
  const auto lhs = expand(first);
  const auto rhs = expand(second);
  if (!lhs.overlaps(rhs)) {
    return union_bounds(first, second);
  }
  return geometry::AxisAlignedBounds{
      Point{std::max(lhs.minimum.x, rhs.minimum.x), std::max(lhs.minimum.y, rhs.minimum.y)},
      Point{std::min(lhs.maximum.x, rhs.maximum.x), std::min(lhs.maximum.y, rhs.maximum.y)}};
}

[[nodiscard]] std::vector<LayerId> via_span(const Board& board, const LayerId& first,
                                            const LayerId& last) {
  std::map<LayerId, std::uint32_t> orders;
  for (const auto& layer : board.layers) {
    if (layer.kind == LayerKind::copper) {
      orders.emplace(layer.id, layer.stack_order);
    }
  }
  if (!orders.contains(first) || !orders.contains(last)) {
    return {};
  }
  const auto low = std::min(orders.at(first), orders.at(last));
  const auto high = std::max(orders.at(first), orders.at(last));
  std::vector<std::pair<std::uint32_t, LayerId>> ordered;
  for (const auto& [layer, stack_order] : orders) {
    if (stack_order >= low && stack_order <= high) {
      ordered.emplace_back(stack_order, layer);
    }
  }
  std::sort(ordered.begin(), ordered.end());
  std::vector<LayerId> result;
  result.reserve(ordered.size());
  for (const auto& [stack_order, layer] : ordered) {
    static_cast<void>(stack_order);
    result.push_back(layer);
  }
  return result;
}

[[nodiscard]] bool covers_layers(const std::vector<LayerId>& container,
                                 const std::vector<LayerId>& required) {
  return std::all_of(required.begin(), required.end(), [&](const auto& layer) {
    return std::find(container.begin(), container.end(), layer) != container.end();
  });
}

[[nodiscard]] bool reusable_transition(const Board& board, const NetId& net, Point position,
                                       const std::vector<LayerId>& layers) {
  if (std::any_of(board.vias.begin(), board.vias.end(), [&](const auto& via) {
        return via.net == net && via.position == position && covers_layers(via.layers, layers);
      })) {
    return true;
  }
  return std::any_of(board.footprints.begin(), board.footprints.end(), [&](const auto& footprint) {
    return std::any_of(footprint.pads.begin(), footprint.pads.end(), [&](const auto& pad) {
      return pad.net == net && pad.plated && pad_has_drill(pad) &&
             pad_drill_world_position(footprint, pad) == position &&
             covers_layers(pad.layers, layers);
    });
  });
}

[[nodiscard]] ObjectId unique_object_id(std::string base, std::set<std::string>& ids) {
  auto candidate = base;
  std::size_t suffix = 1U;
  while (!ids.insert(candidate).second) {
    candidate = base + "-" + std::to_string(suffix++);
  }
  return ObjectId{std::move(candidate)};
}

[[nodiscard]] std::pair<RoutabilityFailureKind, RoutabilityRelaxation>
classify_violation(CopperViolationKind kind) {
  switch (kind) {
  case CopperViolationKind::track_via_clearance:
  case CopperViolationKind::via_clearance:
  case CopperViolationKind::via_pad_clearance:
  case CopperViolationKind::via_zone_clearance:
  case CopperViolationKind::via_diameter:
  case CopperViolationKind::via_drill:
  case CopperViolationKind::via_annular_width:
  case CopperViolationKind::hole_clearance:
  case CopperViolationKind::hole_to_hole_clearance:
    return {RoutabilityFailureKind::via_site_exhausted,
            RoutabilityRelaxation::change_via_site};
  case CopperViolationKind::copper_edge_clearance:
  case CopperViolationKind::copper_outside_board:
  case CopperViolationKind::keepout:
    return {RoutabilityFailureKind::topological_trap, RoutabilityRelaxation::change_corridor};
  case CopperViolationKind::track_clearance:
  case CopperViolationKind::track_pad_clearance:
  case CopperViolationKind::pad_clearance:
  case CopperViolationKind::track_zone_clearance:
  case CopperViolationKind::pad_zone_clearance:
  case CopperViolationKind::zone_clearance:
  case CopperViolationKind::differential_pair_gap:
  case CopperViolationKind::differential_pair_skew:
  case CopperViolationKind::track_width:
    return {RoutabilityFailureKind::quantization_clearance,
            RoutabilityRelaxation::change_corridor};
  }
  return {RoutabilityFailureKind::quantization_clearance,
          RoutabilityRelaxation::increase_local_clearance_margin};
}

[[nodiscard]] RoutabilityDiagnosis malformed_path_diagnosis(const GlobalRoutePath& path,
                                                             std::string detail) {
  const auto anchor = path.points.empty() ? Point{} : path.points.front().position;
  return RoutabilityDiagnosis{
      RoutabilityFailureKind::layer_transition_conflict,
      RoutabilityFailureStage::quantization,
      RoutabilityResourceReference{
          "quantized-copper:malformed:" + path.net.value() + ":" +
              std::to_string(path.connection_index),
          path.points.empty() ? LayerId{} : path.points.front().layer,
          geometry::AxisAlignedBounds{anchor, anchor},
          {path.net}},
      1U,
      {},
      std::nullopt,
      std::nullopt,
      RoutabilityRelaxation::change_layer_sequence,
      std::move(detail),
      false};
}

[[nodiscard]] std::int64_t floor_divide(std::int64_t value, std::int64_t divisor) {
  const auto quotient = value / divisor;
  return value % divisor < 0 ? quotient - 1 : quotient;
}

void coalesce_diagnoses(std::vector<RoutabilityDiagnosis>& diagnoses,
                        const QuantizedCopperOracleConfig& config) {
  std::map<std::string, RoutabilityDiagnosis> retained;
  for (auto& diagnosis : diagnoses) {
    auto& nets = diagnosis.resource.nets;
    std::sort(nets.begin(), nets.end());
    nets.erase(std::unique(nets.begin(), nets.end()), nets.end());
    const auto center_x = std::midpoint(diagnosis.resource.bounds.minimum.x.value(),
                                        diagnosis.resource.bounds.maximum.x.value());
    const auto center_y = std::midpoint(diagnosis.resource.bounds.minimum.y.value(),
                                        diagnosis.resource.bounds.maximum.y.value());
    auto key = "quantized-copper:" + std::string(to_string(diagnosis.kind)) + ':' +
               std::string(to_string(diagnosis.suggested_relaxation)) + ':' +
               diagnosis.resource.layer.value() + ':' +
               std::to_string(floor_divide(center_x, config.diagnosis_coalescing_pitch.value())) +
               ':' +
               std::to_string(floor_divide(center_y, config.diagnosis_coalescing_pitch.value())) +
               ':' + diagnosis.resource.stable_id;
    for (const auto& net : nets) {
      key += ':' + net.value();
    }
    if (const auto found = retained.find(key); found != retained.end()) {
      found->second.resource.bounds = union_bounds(found->second.resource.bounds,
                                                   diagnosis.resource.bounds);
      found->second.severity = std::max(found->second.severity, diagnosis.severity);
      found->second.clearance_deficit =
          std::max(found->second.clearance_deficit, diagnosis.clearance_deficit);
      for (const auto& pair : diagnosis.source_segment_pairs) {
        auto& pairs = found->second.source_segment_pairs;
        if (std::find(pairs.begin(), pairs.end(), pair) == pairs.end())
          pairs.push_back(pair);
      }
      continue;
    }
    diagnosis.resource.stable_id = key;
    retained.emplace(std::move(key), std::move(diagnosis));
  }
  diagnoses.clear();
  diagnoses.reserve(std::min(retained.size(), config.max_diagnoses));
  for (auto& [key, diagnosis] : retained) {
    static_cast<void>(key);
    if (diagnoses.size() >= config.max_diagnoses) {
      break;
    }
    diagnoses.push_back(std::move(diagnosis));
  }
}

} // namespace

QuantizedCopperOracleResult probe_quantized_copper_feasibility(
    const Board& board, const GlobalRouteResult& global, const QuantizedCopperOracleConfig& config,
    std::stop_token stop_token) {
  QuantizedCopperOracleResult result;
  const auto started = std::chrono::steady_clock::now();
  const auto finish = [&] {
    result.elapsed_us = static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() -
                                                              started)
            .count());
  };
  if (!config.enabled) {
    result.diagnostic = "quantized-copper oracle disabled";
    finish();
    return result;
  }
  if (config.grid_pitch.value() <= 0 || config.clearance_severity_quantum.value() <= 0 ||
      config.diagnosis_coalescing_pitch.value() <= 0 || config.max_diagnoses == 0U) {
    result.diagnostic =
        "quantized-copper oracle requires positive pitch, severity quantum, and diagnosis bound";
    finish();
    return result;
  }
  if (stop_token.stop_requested()) {
    result.diagnostic = "quantized-copper oracle cancelled";
    finish();
    return result;
  }

  result.evaluated = true;
  result.configured_grid_pitch = config.grid_pitch;
  result.paths_examined = global.paths.size();
  Board candidate = board;
  std::set<std::string> ids;
  for (const auto& footprint : board.footprints) {
    ids.insert(footprint.id.value());
    for (const auto& pad : footprint.pads) {
      ids.insert(pad.id.value());
    }
    for (const auto& keepout : footprint.keepouts) {
      ids.insert(keepout.id.value());
    }
  }
  for (const auto& track : board.tracks) {
    ids.insert(track.id.value());
  }
  for (const auto& arc : board.track_arcs) {
    ids.insert(arc.id.value());
  }
  for (const auto& curve : board.track_beziers) ids.insert(curve.id.value());
  for (const auto& via : board.vias) {
    ids.insert(via.id.value());
  }
  for (const auto& zone : board.zones) {
    ids.insert(zone.id.value());
  }
  for (const auto& keepout : board.keepouts) {
    ids.insert(keepout.id.value());
  }

  std::map<std::string, PrimitiveEvidence> evidence;
  std::map<std::string, StaticObstacleEvidence> static_obstacles;
  for (const auto& via : board.vias) {
    static_obstacles[via.id.value()].copper_bounds = disc_bounds(via.position, via.diameter);
  }
  for (const auto& drill : drill_shapes(board)) {
    static_obstacles[drill.id.value()].drill_bounds = conservative_drill_bounds(drill);
  }
  std::map<NetId, geometry::AxisAlignedBounds> route_bounds;
  std::vector<ObjectId> dynamic_ids;
  for (const auto& path : global.paths) {
    if (stop_token.stop_requested()) {
      result.diagnostic = "quantized-copper oracle cancelled while materializing paths";
      finish();
      return result;
    }
    if (path.points.size() < 2U) {
      ++result.malformed_paths;
      if (result.diagnoses.size() < config.max_diagnoses) {
        result.diagnoses.push_back(
            malformed_path_diagnosis(path, "selected global path has fewer than two points"));
      }
      continue;
    }
    const auto rules = resolve_net_routing_rules(board, path.net);
    const auto track_checkpoint = candidate.tracks.size();
    const auto via_checkpoint = candidate.vias.size();
    const auto dynamic_checkpoint = dynamic_ids.size();
    const auto generated_track_checkpoint = result.generated_tracks;
    const auto generated_via_checkpoint = result.generated_vias;
    const auto reused_transition_checkpoint = result.reused_same_net_transitions;
    std::vector<GlobalRoutePoint> points = path.points;
    bool valid = std::adjacent_find(path.points.begin(), path.points.end(),
        [](const auto& first, const auto& second) {
          return first.layer != second.layer && first.position != second.position;
        }) == path.points.end();
    // All layer vertices at one physical site must move together. Protecting just the array
    // endpoint tears off-grid terminal via chains apart; conversely snapping must not conceal
    // a lateral layer jump already present in the source checkpoint.
    for (std::size_t first = 0U; first < points.size();) {
      auto end = first + 1U;
      while (end < points.size() && path.points[end].position == path.points[first].position)
        ++end;
      std::vector<LayerId> site_layers;
      for (auto index = first; index < end; ++index) {
        if (std::find(site_layers.begin(), site_layers.end(), path.points[index].layer) ==
            site_layers.end()) {
          site_layers.push_back(path.points[index].layer);
        }
      }
      // A source-owned via or plated drill is an immutable electrical anchor. Snapping an
      // interior reuse site would miss the exact feature lookup and manufacture a duplicate
      // nearby hole. Require exact source position, net and layer coverage, never proximity.
      const bool source_anchor = site_layers.size() > 1U &&
          reusable_transition(board, path.net, path.points[first].position, site_layers);
      const auto site = source_anchor ||
          (config.preserve_path_endpoints && (first == 0U || end == points.size()))
          ? path.points[first].position : quantize(path.points[first].position, config.grid_pitch);
      for (auto index = first; index < end; ++index) points[index].position = site;
      first = end;
    }

    const auto base = "flux-quantized-oracle-" + path.net.value() + "-" +
                      std::to_string(path.connection_index);
    std::size_t track_index = 0U;
    std::size_t via_index = 0U;
    for (std::size_t index = 0U; valid && index + 1U < points.size();) {
      const auto& current = points[index];
      const auto& next = points[index + 1U];
      if (current.layer == next.layer) {
        if (current.position != next.position) {
          auto width = rules.track_width;
          if (index < path.segment_widths.size() && path.segment_widths[index].value() > 0) {
            width = path.segment_widths[index];
          }
          if (width.value() <= 0 || width > rules.track_width) {
            valid = false;
            break;
          }
          auto id = unique_object_id(base + "-track-" + std::to_string(track_index++), ids);
          const auto bounds = expanded_segment_bounds(current.position, next.position, width);
          evidence.emplace(id.value(),
                           PrimitiveEvidence{path.net, path.connection_index, current.layer, bounds,
                                             false, RoutabilitySourceSegment{path.net,
                                                 path.connection_index, index, current.layer,
                                                 path.points[index].position,
                                                 path.points[index + 1U].position}});
          if (const auto found = route_bounds.find(path.net); found != route_bounds.end()) {
            found->second = union_bounds(found->second, bounds);
          } else {
            route_bounds.emplace(path.net, bounds);
          }
          dynamic_ids.push_back(id);
          candidate.tracks.push_back(TrackSegment{std::move(id), path.net, current.layer,
                                                  current.position, next.position, width, false});
          ++result.generated_tracks;
        }
        ++index;
        continue;
      }
      if (current.position != next.position ||
          (index < path.segment_widths.size() &&
           path.segment_widths[index].value() > 0 && path.segment_widths[index] != rules.track_width)) {
        valid = false;
        break;
      }
      auto last_transition = index + 1U;
      while (last_transition + 1U < points.size() &&
             points[last_transition + 1U].position == current.position &&
             points[last_transition + 1U].layer != points[last_transition].layer) {
        ++last_transition;
      }
      auto layers = via_span(board, current.layer, points[last_transition].layer);
      if (layers.size() < 2U) {
        valid = false;
        break;
      }
      if (reusable_transition(candidate, path.net, current.position, layers)) {
        ++result.reused_same_net_transitions;
      } else {
        auto id = unique_object_id(base + "-via-" + std::to_string(via_index++), ids);
        const auto bounds = disc_bounds(current.position, rules.via_diameter);
        PrimitiveEvidence via_evidence{path.net, path.connection_index,
                                       layers.front(), bounds, true};
        // Preserve the actual branch on each side of this transition. A localized
        // disk-overlap box need not touch its owner's track centerline, so nearest
        // geometry alone cannot reliably recover the via's routing ownership.
        const auto append_access = [&](std::size_t edge) {
          const auto& a = path.points[edge];
          const auto& b = path.points[edge + 1U];
          if (a.layer == b.layer && a.position != b.position)
            via_evidence.via_access_segments.push_back(
                {path.net, path.connection_index, edge, a.layer, a.position, b.position});
        };
        if (index > 0U) append_access(index - 1U);
        if (last_transition + 1U < path.points.size()) append_access(last_transition);
        evidence.emplace(id.value(), std::move(via_evidence));
        if (const auto found = route_bounds.find(path.net); found != route_bounds.end()) {
          found->second = union_bounds(found->second, bounds);
        } else {
          route_bounds.emplace(path.net, bounds);
        }
        dynamic_ids.push_back(id);
        candidate.vias.push_back(Via{std::move(id), path.net, current.position, rules.via_diameter,
                                     rules.via_drill, std::move(layers), false});
        ++result.generated_vias;
      }
      index = last_transition;
    }
    if (!valid) {
      for (std::size_t dynamic = dynamic_checkpoint; dynamic < dynamic_ids.size(); ++dynamic) {
        evidence.erase(dynamic_ids[dynamic].value());
      }
      dynamic_ids.resize(dynamic_checkpoint);
      candidate.tracks.resize(track_checkpoint);
      candidate.vias.resize(via_checkpoint);
      result.generated_tracks = generated_track_checkpoint;
      result.generated_vias = generated_via_checkpoint;
      result.reused_same_net_transitions = reused_transition_checkpoint;
      ++result.malformed_paths;
      if (result.diagnoses.size() < config.max_diagnoses) {
        result.diagnoses.push_back(malformed_path_diagnosis(
            path, "quantization exposed a non-coincident layer transition or invalid width"));
      }
      continue;
    }
    ++result.paths_materialized;
  }

  result.all_paths_materialized = result.paths_materialized == result.paths_examined;
  // Rebuild after transactional path rollback so an invalid path cannot broaden a later open-net
  // diagnosis with geometry that was never part of the provisional copper candidate.
  route_bounds.clear();
  for (const auto& [id, primitive] : evidence) {
    static_cast<void>(id);
    if (const auto found = route_bounds.find(primitive.net); found != route_bounds.end()) {
      found->second = union_bounds(found->second, primitive.bounds);
    } else {
      route_bounds.emplace(primitive.net, primitive.bounds);
    }
  }
  const auto drc = check_dynamic_copper_rules(candidate, dynamic_ids);
  result.drc_violations = drc.violation_count();
  result.exact_copper_legal = drc.ok();
  for (const auto& violation : drc.violations) {
    ++result.drc_violation_counts[violation.code];
  }
  for (const auto& violation : drc.violations) {
    if (result.diagnoses.size() >= config.max_diagnoses) {
      break;
    }
    std::vector<NetId> nets;
    std::set<std::string> route_runs;
    LayerId layer;
    std::vector<geometry::AxisAlignedBounds> object_bounds;
    std::vector<geometry::AxisAlignedBounds> immutable_obstacle_bounds;
    bool involves_generated_via = false;
    bool involves_generated_planar_trace = false;
    bool has_immutable_obstacle_geometry = false;
    std::vector<std::string> immutable_obstacles;
    std::vector<RoutabilitySourceSegment> source_segments;
    std::vector<const PrimitiveEvidence*> via_owners;
    for (const auto& object : violation.object_ids) {
      const auto found = evidence.find(object);
      if (found == evidence.end()) {
        const auto static_found = static_obstacles.find(object);
        if (static_found != static_obstacles.end()) {
          const auto use_drill_geometry =
              violation.kind == CopperViolationKind::hole_clearance ||
              violation.kind == CopperViolationKind::hole_to_hole_clearance;
          const auto& bounds = use_drill_geometry ? static_found->second.drill_bounds
                                                   : static_found->second.copper_bounds;
          if (bounds) {
            immutable_obstacle_bounds.push_back(*bounds);
            immutable_obstacles.push_back(object);
            has_immutable_obstacle_geometry = true;
          }
        }
        continue;
      }
      nets.push_back(found->second.net);
      route_runs.insert(found->second.net.value() + "/" +
                        std::to_string(found->second.connection_index) +
                        (found->second.via ? "/via" : "/run"));
      involves_generated_via = involves_generated_via || found->second.via;
      involves_generated_planar_trace =
          involves_generated_planar_trace ||
          (!found->second.via && found->second.source_segment.has_value());
      if (layer.empty()) {
        layer = found->second.layer;
      }
      object_bounds.push_back(found->second.bounds);
      if (found->second.source_segment)
        source_segments.push_back(*found->second.source_segment);
      if (found->second.via) via_owners.push_back(&found->second);
    }
    // Track/via conflicts belong to the track's layer. For via/via conflicts,
    // choose a common attached routing layer when one exists. These source edges
    // identify repair owners; they do not assert that the edges themselves collide.
    if (!via_owners.empty()) {
      if (!source_segments.empty()) {
        layer = source_segments.front().layer;
      } else if (via_owners.size() == 2U) {
        std::optional<LayerId> common_layer;
        for (const auto& access : via_owners.front()->via_access_segments) {
          if (std::any_of(via_owners.back()->via_access_segments.begin(),
                          via_owners.back()->via_access_segments.end(), [&](const auto& other) {
                            return other.layer == access.layer;
                          }) && (!common_layer || access.layer < *common_layer))
            common_layer = access.layer;
        }
        if (common_layer) layer = *common_layer;
      }
      for (const auto* owner : via_owners) {
        const auto access = std::find_if(owner->via_access_segments.begin(),
                                         owner->via_access_segments.end(), [&](const auto& edge) {
                                           return edge.layer == layer;
                                         });
        if (access != owner->via_access_segments.end()) source_segments.push_back(*access);
      }
    }
    std::sort(nets.begin(), nets.end());
    nets.erase(std::unique(nets.begin(), nets.end()), nets.end());
    if (nets.empty()) {
      continue;
    }
    auto [kind, relaxation] = classify_violation(violation.kind);
    bool reclassified_immutable_planar_obstacle = false;
    if (involves_generated_via) {
      kind = RoutabilityFailureKind::via_site_exhausted;
      relaxation = RoutabilityRelaxation::change_via_site;
    } else if (kind == RoutabilityFailureKind::via_site_exhausted &&
               involves_generated_planar_trace && has_immutable_obstacle_geometry) {
      // The source hole/via is immutable.  A generated planar trace which collides with it
      // must walk around the fixed obstacle; asking the allocator to move a nonexistent
      // generated via makes prepare_routability_history discard every eligible planar edge.
      kind = RoutabilityFailureKind::quantization_clearance;
      relaxation = RoutabilityRelaxation::change_corridor;
      reclassified_immutable_planar_obstacle = true;
    } else if (route_runs.size() > 1U && nets.size() > 1U &&
               (violation.kind == CopperViolationKind::track_clearance ||
                violation.kind == CopperViolationKind::differential_pair_gap ||
                violation.kind == CopperViolationKind::differential_pair_skew)) {
      // Two selected connection runs cannot occupy this exact quantized lane arrangement. Keep
      // their template identity so the next epoch changes lane/template selection rather than
      // spraying history over every shadowed grid edge.
      relaxation = RoutabilityRelaxation::change_lane_order;
    }
    std::string stable_id = violation.code;
    for (const auto& run : route_runs) {
      stable_id += ':' + run;
    }
    if (reclassified_immutable_planar_obstacle) {
      object_bounds.insert(object_bounds.end(), immutable_obstacle_bounds.begin(),
                           immutable_obstacle_bounds.end());
      std::sort(immutable_obstacles.begin(), immutable_obstacles.end());
      immutable_obstacles.erase(std::unique(immutable_obstacles.begin(), immutable_obstacles.end()),
                                immutable_obstacles.end());
      for (const auto& object : immutable_obstacles) {
        stable_id += ":immutable:" + object;
      }
    }
    const auto actual = std::max(0.0L, violation.actual_distance_nm);
    const auto required = static_cast<long double>(violation.required_clearance.value());
    const auto deficit_value = required > actual
                                   ? static_cast<std::int64_t>(std::min(
                                         required - actual,
                                         static_cast<long double>(
                                             std::numeric_limits<std::int64_t>::max())))
                                   : 0;
    auto bounds = object_bounds.front();
    if (object_bounds.size() >= 2U) {
      // For a fixed obstacle, even a small clearance shortfall must produce a local witness.
      // Expanding only by the deficit can leave the two copper bounds disjoint and trigger
      // the whole-track union fallback when their gap is close to the required clearance.
      bounds = localized_overlap_bounds(object_bounds[0], object_bounds[1],
                                        reclassified_immutable_planar_obstacle
                                            ? violation.required_clearance
                                            : Nanometres{deficit_value});
      for (std::size_t index = 2U; index < object_bounds.size(); ++index) {
        bounds = union_bounds(bounds, object_bounds[index]);
      }
    }
    const auto quantum = static_cast<std::uint64_t>(config.clearance_severity_quantum.value());
    const auto severity =
        1U + static_cast<std::uint64_t>(std::max<std::int64_t>(0, deficit_value)) / quantum;
    result.diagnoses.push_back(RoutabilityDiagnosis{
        kind,
        RoutabilityFailureStage::quantization,
        RoutabilityResourceReference{std::move(stable_id), layer, bounds, std::move(nets)},
        severity,
        Nanometres{deficit_value},
        std::nullopt,
        std::nullopt,
        relaxation,
        "fine-grid provisional copper failed exact Flux DRC: " + violation.code,
        false});
    if (source_segments.size() == 2U &&
        source_segments[0].net != source_segments[1].net) {
      if (source_segments[1].net < source_segments[0].net)
        std::swap(source_segments[0], source_segments[1]);
      result.diagnoses.back().source_segment_pairs.push_back(
          {source_segments[0], source_segments[1]});
    }
  }

  const auto connectivity = analyze_connectivity(candidate);
  result.open_connections = connectivity.open_connections;
  result.unrouted_nets = connectivity.unrouted_nets;
  for (const auto& net : connectivity.nets) {
    if (net.open_connections == 0U || result.diagnoses.size() >= config.max_diagnoses) {
      continue;
    }
    const auto found = route_bounds.find(net.net);
    const auto bounds = found == route_bounds.end() ? geometry::AxisAlignedBounds{} : found->second;
    result.diagnoses.push_back(RoutabilityDiagnosis{
        RoutabilityFailureKind::terminal_escape_ordering,
        RoutabilityFailureStage::quantization,
        RoutabilityResourceReference{"quantized-copper:open:" + net.net.value(), LayerId{}, bounds,
                                     {net.net}},
        static_cast<std::uint64_t>(net.open_connections),
        {},
        std::nullopt,
        std::nullopt,
        RoutabilityRelaxation::change_terminal_order,
        "fine-grid provisional copper leaves " + std::to_string(net.open_connections) +
            " exact open connection(s)",
        false});
  }
  coalesce_diagnoses(result.diagnoses, config);
  result.diagnostic = result.feasible()
                          ? "fine-grid provisional copper is exact-DRC-clean and connected"
                          : "fine-grid provisional copper rejected: malformed_paths=" +
                                std::to_string(result.malformed_paths) +
                                ", drc_violations=" + std::to_string(result.drc_violations) +
                                ", opens=" + std::to_string(result.open_connections);
  finish();
  return result;
}

} // namespace flux
