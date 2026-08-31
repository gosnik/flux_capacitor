#include "flux/core/connectivity.hpp"

#include "flux/core/copper_geometry.hpp"
#include "flux/core/geometry.hpp"

#include <algorithm>
#include <cstddef>
#include <numeric>
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

} // namespace

ConnectivityReport analyze_connectivity(const Board& board) {
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
            pads.push_back(PadContact{copper->position, &pad.layers, *copper});
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
      pad_groups.insert(sets.find(i));
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

} // namespace flux
