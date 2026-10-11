#include "scaffold_access.hpp"

#include "flux/core/bezier_copper.hpp"
#include "flux/core/connectivity.hpp"
#include "flux/core/copper_geometry.hpp"
#include "flux/core/geometry.hpp"
#include "flux/core/routing_kernel.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace flux {
namespace {

[[nodiscard]] bool contains(const std::vector<LayerId>& layers, const LayerId& layer) {
  return std::find(layers.begin(), layers.end(), layer) != layers.end();
}

void append_layer(std::vector<LayerId>& layers, const LayerId& layer) {
  if (!contains(layers, layer)) layers.push_back(layer);
}

[[nodiscard]] std::vector<LayerId> accessible_layers(const Board& board, const NetId& net,
                                                       const GlobalRoutePoint& endpoint) {
  std::vector<std::vector<LayerId>> contacts;
  for (const auto& pad : copper_pad_shapes(board)) {
    if (pad.net == net && point_to_pad_copper_distance_nm(endpoint.position, pad) <= 0.0L)
      contacts.push_back(pad.layers);
  }
  if (contacts.empty()) return {endpoint.layer};
  std::vector<LayerId> result;
  // Prefer actual pad contacts on the requested layer. Coincident opposite-side SMT pads do
  // not become interchangeable merely because they share a net and XY position.
  for (const auto& layers : contacts) if (contains(layers, endpoint.layer))
    for (const auto& layer : layers) append_layer(result, layer);
  if (!result.empty()) return result;
  // A coarse label may be wrong for a unique pad contact. Correct it only when every matching
  // pad has the same layer set; ambiguous contacts fail closed instead of inventing a bridge.
  auto unique_layers = contacts.front();
  std::sort(unique_layers.begin(), unique_layers.end());
  for (auto layers : contacts) {
    std::sort(layers.begin(), layers.end());
    if (layers != unique_layers) return {};
  }
  result = std::move(unique_layers);
  return result;
}

[[nodiscard]] CopperConnectivityProbe probe_for(const Board& board, const NetId& net,
                                                  const GlobalRoutePoint& endpoint) {
  return {endpoint.position, accessible_layers(board, net, endpoint)};
}

void append_unique(std::vector<GlobalRoutePoint>& points, GlobalRoutePoint point,
                   std::size_t maximum_points) {
  if (points.size() >= maximum_points ||
      std::find(points.begin(), points.end(), point) != points.end()) return;
  points.push_back(point);
}

[[nodiscard]] Point projected_point(Point start, Point end, Point target) {
  const auto dx = static_cast<long double>(end.x.value()) - start.x.value();
  const auto dy = static_cast<long double>(end.y.value()) - start.y.value();
  const auto length2 = dx * dx + dy * dy;
  if (length2 == 0.0L) return start;
  auto t = ((static_cast<long double>(target.x.value()) - start.x.value()) * dx +
            (static_cast<long double>(target.y.value()) - start.y.value()) * dy) / length2;
  t = std::clamp(t, 0.0L, 1.0L);
  return {Nanometres{static_cast<std::int64_t>(std::llround(start.x.value() + t * dx))},
          Nanometres{static_cast<std::int64_t>(std::llround(start.y.value() + t * dy))}};
}

template <typename Visitor>
[[nodiscard]] bool visit_component_anchor_candidates(const Board& board,
                                                      const GlobalRoutePath& request,
                                                      Point opposite, Visitor&& visit) {
  const auto visit_layers = [&](Point point, const std::vector<LayerId>& layers) {
    for (const auto& layer : layers)
      if (!visit(GlobalRoutePoint{point, layer})) return false;
    return true;
  };
  // The visitor applies the output cap only after exact component membership filtering. Board
  // order therefore cannot let disconnected same-net copper consume every usable anchor.
  for (const auto& pad : copper_pad_shapes(board)) {
    routing_kernel_checkpoint();
    if (pad.net == request.net && !visit_layers(pad.position, pad.layers)) return false;
  }
  for (const auto& track : board.tracks) if (track.net == request.net) {
    routing_kernel_checkpoint();
    if (!visit({track.start, track.layer}) || !visit({track.end, track.layer}) ||
        !visit({projected_point(track.start, track.end, opposite), track.layer})) return false;
  }
  for (const auto& via : board.vias) if (via.net == request.net) {
    routing_kernel_checkpoint();
    if (!visit_layers(via.position, via.layers)) return false;
  }
  // Arc/Bezier interiors require a parameter value and an exact containment proof. Their source
  // endpoints are exact round caps in the connectivity oracle, so retain those safe anchors.
  for (const auto& arc : board.track_arcs) if (arc.net == request.net) {
    routing_kernel_checkpoint();
    if (!visit({arc.start, arc.layer}) || !visit({arc.end, arc.layer})) return false;
  }
  for (const auto& curve : board.track_beziers) if (curve.net == request.net) {
    routing_kernel_checkpoint();
    if (!visit({curve.start, curve.layer}) || !visit({curve.end, curve.layer})) return false;
  }
  // Zone fills intentionally have no generated interior sample: a polygon sample would need an
  // exact, deterministic interior-point certificate that also respects holes.
  return true;
}

[[nodiscard]] bool shares_component(const CopperProbeComponentMembership& first,
                                    const CopperProbeComponentMembership& second) {
  if (!first.contained_by_copper || !second.contained_by_copper) return false;
  return std::any_of(first.components.begin(), first.components.end(), [&second](std::size_t component) {
    return std::binary_search(second.components.begin(), second.components.end(), component);
  });
}

[[nodiscard]] bool valid_request(const GlobalRoutePath& request) {
  return request.points.size() >= 2U;
}

} // namespace

bool scaffold_request_connected(const Board& board, const GlobalRoutePath& request) {
  if (!valid_request(request)) return false;
  const auto source = probe_for(board, request.net, request.points.front());
  const auto target = probe_for(board, request.net, request.points.back());
  return copper_probes_share_component(board, request.net, source, target);
}

ScaffoldConnectionAccess scaffold_connection_access(const Board& board,
                                                    const GlobalRoutePath& request,
                                                    std::size_t maximum_points) {
  ScaffoldConnectionAccess result;
  if (!valid_request(request) || maximum_points == 0U) return result;
  const auto& first = request.points.front();
  const auto& last = request.points.back();
  const auto source = probe_for(board, request.net, first);
  const auto target = probe_for(board, request.net, last);
  const auto endpoint_memberships =
      copper_probe_component_memberships(board, request.net, {source, target});
  if (endpoint_memberships.size() != 2U) return result;
  result.source_endpoint_contained_by_copper = endpoint_memberships[0].contained_by_copper;
  result.target_endpoint_contained_by_copper = endpoint_memberships[1].contained_by_copper;
  result.already_connected = shares_component(endpoint_memberships[0], endpoint_memberships[1]);

  const auto collect = [&](const GlobalRoutePoint& endpoint, const CopperConnectivityProbe& probe,
                           bool endpoint_has_copper, Point opposite,
                           std::vector<GlobalRoutePoint>& output) {
    if (!endpoint_has_copper) {
      append_unique(output, endpoint, maximum_points);
      return;
    }
    // Membership IDs are local to each oracle call, so each bounded batch carries the endpoint
    // probe at slot zero. This bounds temporary storage while filtering before the output cap.
    constexpr std::size_t batch_capacity = 128U;
    std::vector<GlobalRoutePoint> batch;
    batch.reserve(batch_capacity);
    const auto flush = [&] {
      if (batch.empty()) return;
      std::vector<CopperConnectivityProbe> probes;
      probes.reserve(batch.size() + 1U);
      probes.push_back(probe);
      for (const auto& candidate : batch) probes.push_back({candidate.position, {candidate.layer}});
      const auto memberships = copper_probe_component_memberships(board, request.net, probes);
      if (memberships.size() == probes.size())
        for (std::size_t index = 0U; index < batch.size() && output.size() < maximum_points; ++index)
          if (shares_component(memberships[0], memberships[index + 1U]))
            append_unique(output, batch[index], maximum_points);
      batch.clear();
    };
    const auto enqueue = [&](GlobalRoutePoint candidate) {
      if (output.size() >= maximum_points) return false;
      if (std::find(batch.begin(), batch.end(), candidate) != batch.end() ||
          std::find(output.begin(), output.end(), candidate) != output.end()) return true;
      batch.push_back(candidate);
      if (batch.size() == batch_capacity) flush();
      return output.size() < maximum_points;
    };
    // Preserve the exact physical branch contact first. It is then independently rechecked by
    // the batch oracle rather than assumed from a coarse net/layer label.
    for (const auto& layer : probe.layers)
      if (!enqueue({endpoint.position, layer})) return;
    static_cast<void>(visit_component_anchor_candidates(board, request, opposite, enqueue));
    flush();
  };
  collect(first, source, endpoint_memberships[0].contained_by_copper, last.position, result.starts);
  collect(last, target, endpoint_memberships[1].contained_by_copper, first.position, result.goals);
  return result;
}
} // namespace flux
