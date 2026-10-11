#include "flux/core/connectivity.hpp"

#include "flux/core/bezier_copper.hpp"
#include "flux/core/copper_geometry.hpp"
#include "flux/core/geometry.hpp"

#include <algorithm>
#include <cstddef>
#include <numeric>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

namespace flux {
namespace {

class DisjointSet {
public:
  explicit DisjointSet(std::size_t size) : parent_(size), rank_(size, 0U) {
    std::iota(parent_.begin(), parent_.end(), std::size_t{0});
  }

  std::size_t find(std::size_t item) {
    if (parent_[item] != item) {
      parent_[item] = find(parent_[item]);
    }
    return parent_[item];
  }

  void unite(std::size_t lhs, std::size_t rhs) {
    lhs = find(lhs);
    rhs = find(rhs);
    if (lhs == rhs) {
      return;
    }
    if (rank_[lhs] < rank_[rhs]) {
      std::swap(lhs, rhs);
    }
    parent_[rhs] = lhs;
    if (rank_[lhs] == rank_[rhs]) {
      ++rank_[lhs];
    }
  }

private:
  std::vector<std::size_t> parent_;
  std::vector<unsigned int> rank_;
};

struct PadContact {
  ObjectId id;
  Point position;
  const std::vector<LayerId>* layers;
  CopperPadShape copper;
};

struct ZoneContact {
  LayerId layer;
  const CopperZoneFill* region;
};

[[nodiscard]] bool contains_layer(const std::vector<LayerId>& layers, const LayerId& layer) {
  return std::find(layers.begin(), layers.end(), layer) != layers.end();
}

[[nodiscard]] bool share_layer(const std::vector<LayerId>& lhs, const std::vector<LayerId>& rhs) {
  return std::any_of(lhs.begin(), lhs.end(),
                     [&rhs](const auto& layer) { return contains_layer(rhs, layer); });
}

[[nodiscard]] bool probe_touches_copper(const Board& board, const NetId& net,
                                         const CopperConnectivityProbe& probe) {
  if (probe.layers.empty() || !geometry::coordinate_in_range(probe.position)) return false;
  const auto on_layer = [&probe](const std::vector<LayerId>& layers) {
    return share_layer(probe.layers, layers);
  };
  for (const auto& pad : copper_pad_shapes(board))
    if (pad.net == net && on_layer(pad.layers) &&
        point_to_pad_copper_distance_nm(probe.position, pad) <= 0.0L) return true;
  for (const auto& track : board.tracks)
    if (track.net == net && contains_layer(probe.layers, track.layer) &&
        geometry::point_segment_distance_nm(probe.position, track.start, track.end) <=
            static_cast<long double>(track.width.value()) / 2.0L) return true;
  for (const auto& arc : board.track_arcs)
    if (arc.net == net && contains_layer(probe.layers, arc.layer) &&
        geometry::point_circular_arc_distance_nm(probe.position, arc.start, arc.middle, arc.end) <=
            static_cast<long double>(arc.width.value()) / 2.0L) return true;
  for (const auto& via : board.vias)
    if (via.net == net && on_layer(via.layers) &&
        geometry::point_distance_nm(probe.position, via.position) <=
            static_cast<long double>(via.diameter.value()) / 2.0L) return true;
  for (const auto& zone : board.zones) if (zone.net == net)
    for (const auto& fill : zone.fills)
      if (contains_layer(probe.layers, fill.layer) &&
          geometry::point_filled_region_copper_distance_nm(probe.position, fill) <= 0.0L) return true;
  // Keep Bezier membership identical to the oracle's contracted-chord representation.  In
  // particular, do not accept a point merely because it lies inside an uncontracted proxy.
  for (const auto& curve : board.track_beziers) {
    if (curve.net != net || !contains_layer(probe.layers, curve.layer)) continue;
    const auto flattened = tessellate_track_bezier(curve);
    if (!flattened.certified) continue;
    const auto endpoint_radius = static_cast<long double>(curve.width.value()) / 2.0L;
    if (geometry::point_distance_nm(probe.position, curve.start) <= endpoint_radius ||
        geometry::point_distance_nm(probe.position, curve.end) <= endpoint_radius) return true;
    for (const auto& segment : flattened.segments) {
      const auto guard = segment.centerline_deviation_nm.value() + segment.quantization_error_nm.value();
      if (guard < 0 || guard > curve.width.value() / 2) continue;
      const auto inner_radius = static_cast<long double>(curve.width.value() - 2 * guard) / 2.0L;
      if (geometry::point_segment_distance_nm(probe.position, segment.start, segment.end) <= inner_radius)
        return true;
    }
  }
  return false;
}

void append_probe_pads(Board& result, const NetId& net, const CopperConnectivityProbe& probe,
                       std::size_t ordinal, std::vector<ObjectId>& ids) {
  const auto id_in_use = [&result](const ObjectId& id) {
    if (std::any_of(result.footprints.begin(), result.footprints.end(),
                    [&id](const Footprint& footprint) { return footprint.id == id; })) return true;
    return std::any_of(result.footprints.begin(), result.footprints.end(), [&id](const Footprint& footprint) {
      return std::any_of(footprint.pads.begin(), footprint.pads.end(),
                         [&id](const Pad& pad) { return pad.id == id; });
    });
  };
  Footprint footprint;
  auto prefix = std::string{"__flux-connectivity-probe-"} + std::to_string(ordinal);
  const auto prefix_in_use = [&id_in_use, &probe](const std::string& candidate) {
    if (id_in_use(ObjectId{candidate + "-footprint"})) return true;
    for (std::size_t index = 0U; index < probe.layers.size(); ++index)
      if (id_in_use(ObjectId{candidate + "-pad-" + std::to_string(index)})) return true;
    return false;
  };
  while (prefix_in_use(prefix)) prefix += "_";
  footprint.id = ObjectId{prefix + "-footprint"};
  footprint.position = {};
  footprint.reference = footprint.id.value();
  for (std::size_t layer_index = 0U; layer_index < probe.layers.size(); ++layer_index) {
    Pad pad;
    pad.id = ObjectId{prefix + "-pad-" + std::to_string(layer_index)};
    pad.net = net;
    pad.local_position = probe.position;
    // One pad per layer is essential: a multi-layer temporary pad would itself be an invented
    // plated connection between otherwise separate copper layers.
    pad.layers = {probe.layers[layer_index]};
    pad.width = Nanometres{0};
    pad.height = Nanometres{0};
    pad.plated = false;
    ids.push_back(pad.id);
    footprint.pads.push_back(std::move(pad));
  }
  result.footprints.push_back(std::move(footprint));
}

} // namespace

ConnectivityReport analyze_connectivity(const Board& board) {
  // Chords are contracted by their certified deviation so every proxy copper point is contained
  // in the actual curve's copper.  This may report an open marginal connection, but it cannot
  // create electrical contact through tessellation inflation.  Degenerate endpoint disks are
  // exact because a round trace cap is present at each cubic endpoint.
  if (!board.track_beziers.empty()) {
    Board proxy = board;
    proxy.track_beziers.clear();
    for (const auto& curve : board.track_beziers) {
      const auto flattened = tessellate_track_bezier(curve);
      if (!flattened.certified) continue;
      for (const auto& segment : flattened.segments) {
        const auto guard = segment.centerline_deviation_nm.value() + segment.quantization_error_nm.value();
        // If the certified error consumes the whole copper radius, there is no non-empty inner
        // chord.  Omit it rather than inventing a one-nanometre bridge outside the real copper.
        if (guard < 0 || guard > curve.width.value() / 2) continue;
        const auto inner_width = curve.width.value() - 2 * guard;
        proxy.tracks.push_back({curve.id, curve.net, curve.layer, segment.start, segment.end,
                                Nanometres{inner_width}, curve.locked});
      }
      proxy.tracks.push_back({curve.id, curve.net, curve.layer, curve.start, curve.start,
                              curve.width, curve.locked});
      if (curve.end != curve.start) {
        proxy.tracks.push_back({curve.id, curve.net, curve.layer, curve.end, curve.end,
                                curve.width, curve.locked});
      }
    }
    return analyze_connectivity(proxy);
  }
  ConnectivityReport report;
  report.nets.reserve(board.nets.size());
  const auto copper_pads = copper_pad_shapes(board);

  for (const auto& net : board.nets) {
    std::vector<PadContact> pads;
    std::vector<const TrackSegment*> tracks;
    std::vector<const TrackArc*> arcs;
    std::vector<const Via*> vias;
    std::vector<ZoneContact> zones;

    for (const auto& footprint : board.footprints) {
      for (const auto& pad : footprint.pads) {
        if (pad.net == net.id) {
          const auto copper = std::find_if(
              copper_pads.begin(), copper_pads.end(),
              [&pad](const CopperPadShape& candidate) { return candidate.id == pad.id; });
          if (copper != copper_pads.end()) {
            pads.push_back(PadContact{pad.id, copper->position, &pad.layers, *copper});
          }
        }
      }
    }
    for (const auto& track : board.tracks) {
      if (track.net == net.id) {
        tracks.push_back(&track);
      }
    }
    for (const auto& arc : board.track_arcs) {
      if (arc.net == net.id) {
        arcs.push_back(&arc);
      }
    }
    for (const auto& via : board.vias) {
      if (via.net == net.id) {
        vias.push_back(&via);
      }
    }
    for (const auto& zone : board.zones) {
      if (zone.net == net.id) {
        for (const auto& fill : zone.fills) {
          zones.push_back(ZoneContact{fill.layer, &fill});
        }
      }
    }

    // KiCad net zero is represented by the one canonical net with an empty name. Its pads are
    // deliberately unassigned, not members of one electrical connection that the router should
    // join. Preserve their inventory while excluding the intentionally separate pads from the
    // ratsnest/open-connection objective.
    if (net.name.empty()) {
      report.nets.push_back(NetConnectivity{net.id, pads.size(), pads.size(), 0U});
      continue;
    }

    const auto track_offset = pads.size();
    const auto arc_offset = track_offset + tracks.size();
    const auto via_offset = arc_offset + arcs.size();
    const auto zone_offset = via_offset + vias.size();
    DisjointSet sets(zone_offset + zones.size());

    for (std::size_t i = 0; i < pads.size(); ++i) {
      if (!geometry::coordinate_in_range(pads[i].position)) {
        continue;
      }
      for (std::size_t j = i + 1U; j < pads.size(); ++j) {
        if (share_layer(*pads[i].layers, *pads[j].layers) &&
            pad_copper_intersects(pads[i].copper, pads[j].copper)) {
          sets.unite(i, j);
        }
      }
      for (std::size_t j = 0; j < tracks.size(); ++j) {
        const auto& track = *tracks[j];
        const auto radius = static_cast<long double>(track.width.value()) / 2.0L;
        if (contains_layer(*pads[i].layers, track.layer) &&
            segment_to_pad_copper_distance_nm(track.start, track.end, pads[i].copper) <= radius) {
          sets.unite(i, track_offset + j);
        }
      }
      for (std::size_t j = 0; j < arcs.size(); ++j) {
        const auto& arc = *arcs[j];
        const auto radius = static_cast<long double>(arc.width.value()) / 2.0L;
        if (contains_layer(*pads[i].layers, arc.layer) &&
            circular_arc_to_pad_copper_distance_nm(arc.start, arc.middle, arc.end,
                                                   pads[i].copper) <= radius) {
          sets.unite(i, arc_offset + j);
        }
      }
      for (std::size_t j = 0; j < vias.size(); ++j) {
        const auto radius = static_cast<long double>(vias[j]->diameter.value()) / 2.0L;
        if (share_layer(*pads[i].layers, vias[j]->layers) &&
            point_to_pad_copper_distance_nm(vias[j]->position, pads[i].copper) <= radius) {
          sets.unite(i, via_offset + j);
        }
      }
      for (std::size_t j = 0; j < zones.size(); ++j) {
        if (contains_layer(*pads[i].layers, zones[j].layer) &&
            pad_copper_overlaps_filled_region(pads[i].copper, *zones[j].region)) {
          sets.unite(i, zone_offset + j);
        }
      }
    }

    for (std::size_t i = 0; i < tracks.size(); ++i) {
      const auto& lhs = *tracks[i];
      if (!geometry::coordinate_in_range(lhs.start) || !geometry::coordinate_in_range(lhs.end)) {
        continue;
      }
      for (std::size_t j = i + 1U; j < tracks.size(); ++j) {
        const auto& rhs = *tracks[j];
        const auto required = (static_cast<long double>(lhs.width.value()) +
                               static_cast<long double>(rhs.width.value())) /
                              2.0L;
        if (lhs.layer == rhs.layer &&
            geometry::segment_distance_nm(lhs.start, lhs.end, rhs.start, rhs.end) <= required) {
          sets.unite(track_offset + i, track_offset + j);
        }
      }
      for (std::size_t j = 0; j < arcs.size(); ++j) {
        const auto& arc = *arcs[j];
        const auto required = (static_cast<long double>(lhs.width.value()) +
                               static_cast<long double>(arc.width.value())) /
                              2.0L;
        if (lhs.layer == arc.layer &&
            geometry::segment_circular_arc_distance_nm(lhs.start, lhs.end, arc.start, arc.middle,
                                                       arc.end) <= required) {
          sets.unite(track_offset + i, arc_offset + j);
        }
      }
      for (std::size_t j = 0; j < vias.size(); ++j) {
        const auto required = (static_cast<long double>(lhs.width.value()) +
                               static_cast<long double>(vias[j]->diameter.value())) /
                              2.0L;
        if (contains_layer(vias[j]->layers, lhs.layer) &&
            geometry::point_segment_distance_nm(vias[j]->position, lhs.start, lhs.end) <=
                required) {
          sets.unite(track_offset + i, via_offset + j);
        }
      }
      for (std::size_t j = 0; j < zones.size(); ++j) {
        const auto radius = static_cast<long double>(lhs.width.value()) / 2.0L;
        if (lhs.layer == zones[j].layer && geometry::segment_filled_region_copper_distance_nm(
                                               lhs.start, lhs.end, *zones[j].region) <= radius) {
          sets.unite(track_offset + i, zone_offset + j);
        }
      }
    }

    for (std::size_t i = 0; i < arcs.size(); ++i) {
      const auto& lhs = *arcs[i];
      const auto lhs_radius = static_cast<long double>(lhs.width.value()) / 2.0L;
      for (std::size_t j = i + 1U; j < arcs.size(); ++j) {
        const auto& rhs = *arcs[j];
        const auto required = lhs_radius + static_cast<long double>(rhs.width.value()) / 2.0L;
        if (lhs.layer == rhs.layer &&
            geometry::circular_arcs_distance_nm(lhs.start, lhs.middle, lhs.end, rhs.start,
                                                rhs.middle, rhs.end) <= required) {
          sets.unite(arc_offset + i, arc_offset + j);
        }
      }
      for (std::size_t j = 0; j < vias.size(); ++j) {
        const auto required =
            lhs_radius + static_cast<long double>(vias[j]->diameter.value()) / 2.0L;
        if (contains_layer(vias[j]->layers, lhs.layer) &&
            geometry::point_circular_arc_distance_nm(vias[j]->position, lhs.start, lhs.middle,
                                                     lhs.end) <= required) {
          sets.unite(arc_offset + i, via_offset + j);
        }
      }
      for (std::size_t j = 0; j < zones.size(); ++j) {
        if (lhs.layer == zones[j].layer &&
            geometry::circular_arc_filled_region_copper_distance_nm(
                lhs.start, lhs.middle, lhs.end, *zones[j].region) <= lhs_radius) {
          sets.unite(arc_offset + i, zone_offset + j);
        }
      }
    }

    for (std::size_t i = 0; i < vias.size(); ++i) {
      for (std::size_t j = i + 1U; j < vias.size(); ++j) {
        const auto required = (static_cast<long double>(vias[i]->diameter.value()) +
                               static_cast<long double>(vias[j]->diameter.value())) /
                              2.0L;
        if (share_layer(vias[i]->layers, vias[j]->layers) &&
            geometry::point_distance_nm(vias[i]->position, vias[j]->position) <= required) {
          sets.unite(via_offset + i, via_offset + j);
        }
      }
      for (std::size_t j = 0; j < zones.size(); ++j) {
        const auto radius = static_cast<long double>(vias[i]->diameter.value()) / 2.0L;
        if (contains_layer(vias[i]->layers, zones[j].layer) &&
            geometry::point_filled_region_copper_distance_nm(vias[i]->position, *zones[j].region) <=
                radius) {
          sets.unite(via_offset + i, zone_offset + j);
        }
      }
    }

    for (std::size_t i = 0; i < zones.size(); ++i) {
      for (std::size_t j = i + 1U; j < zones.size(); ++j) {
        if (zones[i].layer == zones[j].layer &&
            geometry::filled_regions_overlap(*zones[i].region, *zones[j].region)) {
          sets.unite(zone_offset + i, zone_offset + j);
        }
      }
    }

    std::unordered_set<std::size_t> pad_groups;
    for (std::size_t i = 0; i < pads.size(); ++i) {
      const auto component = sets.find(i);
      pad_groups.insert(component);
      report.pad_components.push_back(PadConnectivityComponent{pads[i].id, net.id, component});
    }
    const auto group_count = pad_groups.size();
    const auto open_count = group_count > 0U ? group_count - 1U : 0U;
    report.open_connections += open_count;
    if (open_count > 0U) {
      ++report.unrouted_nets;
    }
    report.nets.push_back(NetConnectivity{net.id, pads.size(), group_count, open_count});
  }

  return report;
}

std::vector<CopperProbeComponentMembership>
copper_probe_component_memberships(const Board& board, const NetId& net,
                                   const std::vector<CopperConnectivityProbe>& probes) {
  std::vector<CopperProbeComponentMembership> result(probes.size());
  const auto net_it = std::find_if(board.nets.begin(), board.nets.end(),
                                   [&net](const Net& candidate) { return candidate.id == net; });
  if (net_it == board.nets.end() || net_it->name.empty()) return result;
  Board probe_board = board;
  // The DSU result for one net is independent of all other nets.  Keeping only the requested
  // net avoids re-running their potentially large connectivity scenes for every access batch.
  probe_board.nets = {*net_it};
  std::vector<std::vector<ObjectId>> ids(probes.size());
  for (std::size_t index = 0U; index < probes.size(); ++index) {
    if (!probe_touches_copper(board, net, probes[index])) continue;
    result[index].contained_by_copper = true;
    append_probe_pads(probe_board, net, probes[index], index, ids[index]);
  }
  const auto report = analyze_connectivity(probe_board);
  for (std::size_t index = 0U; index < ids.size(); ++index) {
    for (const auto& id : ids[index]) {
      const auto found = std::find_if(report.pad_components.begin(), report.pad_components.end(),
                                      [&id, &net](const PadConnectivityComponent& item) {
                                        return item.net == net && item.pad == id;
                                      });
      if (found != report.pad_components.end()) result[index].components.push_back(found->component);
    }
    std::sort(result[index].components.begin(), result[index].components.end());
    result[index].components.erase(std::unique(result[index].components.begin(),
                                                result[index].components.end()),
                                   result[index].components.end());
  }
  return result;
}

bool copper_probes_share_component(const Board& board, const NetId& net,
                                   const CopperConnectivityProbe& first,
                                   const CopperConnectivityProbe& second) {
  const auto memberships = copper_probe_component_memberships(board, net, {first, second});
  if (memberships.size() != 2U || !memberships[0].contained_by_copper ||
      !memberships[1].contained_by_copper) return false;
  return std::any_of(memberships[0].components.begin(), memberships[0].components.end(),
                     [&memberships](const std::size_t component) {
                       return std::binary_search(memberships[1].components.begin(),
                                                 memberships[1].components.end(), component);
                     });
}

} // namespace flux
