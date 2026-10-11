#include "flux/core/drc.hpp"

#include "flux/core/bezier_copper.hpp"
#include "flux/core/copper_geometry.hpp"
#include "flux/core/geometry.hpp"
#include "flux/core/keepout_geometry.hpp"
#include "flux/core/routing_compute.hpp"
#include "flux/core/routing_rules.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <map>
#include <set>
#include <string>
#include <utility>
#include <vector>

namespace flux {
namespace {

[[nodiscard]] bool contains_layer(const std::vector<LayerId>& layers, const LayerId& layer) {
  return std::find(layers.begin(), layers.end(), layer) != layers.end();
}

[[nodiscard]] bool share_layer(const std::vector<LayerId>& lhs, const std::vector<LayerId>& rhs) {
  return std::any_of(lhs.begin(), lhs.end(),
                     [&rhs](const auto& layer) { return contains_layer(rhs, layer); });
}

struct TrackCenterline {
  ObjectId id;
  NetId net;
  LayerId layer;
  Point start;
  Point middle;
  Point end;
  Nanometres width;
  bool circular{false};
};

[[nodiscard]] std::vector<TrackCenterline> track_centerlines(const Board& board) {
  std::vector<TrackCenterline> result;
  result.reserve(board.tracks.size() + board.track_arcs.size());
  for (const auto& track : board.tracks) {
    result.push_back({track.id, track.net, track.layer, track.start, track.start, track.end,
                      track.width, false});
  }
  for (const auto& arc : board.track_arcs) {
    result.push_back({arc.id, arc.net, arc.layer, arc.start, arc.middle, arc.end, arc.width, true});
  }
  return result;
}

[[nodiscard]] long double centerline_distance(const TrackCenterline& lhs,
                                              const TrackCenterline& rhs) {
  if (lhs.circular && rhs.circular) {
    return geometry::circular_arcs_distance_nm(lhs.start, lhs.middle, lhs.end, rhs.start,
                                               rhs.middle, rhs.end);
  }
  if (lhs.circular) {
    return geometry::segment_circular_arc_distance_nm(rhs.start, rhs.end, lhs.start, lhs.middle,
                                                      lhs.end);
  }
  if (rhs.circular) {
    return geometry::segment_circular_arc_distance_nm(lhs.start, lhs.end, rhs.start, rhs.middle,
                                                      rhs.end);
  }
  return geometry::segment_distance_nm(lhs.start, lhs.end, rhs.start, rhs.end);
}

void add_violation(CopperDrcReport& report, CopperViolationKind kind, std::string code,
                   std::vector<std::string> object_ids, long double required, long double actual) {
  const auto bounded_required = std::clamp(
      required, 0.0L, static_cast<long double>(std::numeric_limits<std::int64_t>::max()));
  report.violations.push_back(
      CopperViolation{kind, std::move(code), std::move(object_ids),
                      Nanometres{static_cast<std::int64_t>(std::ceil(bounded_required))}, actual});
}

[[nodiscard]] bool selected_for_dynamic_check(const std::set<std::string>* selected,
                                              const ObjectId& first,
                                              const ObjectId* second = nullptr);

void remap_bezier_proxy_ids(CopperDrcReport& report,
                            const std::map<std::string, ObjectId>& proxy_sources) {
  for (auto& violation : report.violations) {
    for (auto& id : violation.object_ids) {
      if (const auto found = proxy_sources.find(id); found != proxy_sources.end()) id = found->second.value();
    }
    std::sort(violation.object_ids.begin(), violation.object_ids.end());
    violation.object_ids.erase(std::unique(violation.object_ids.begin(), violation.object_ids.end()),
                               violation.object_ids.end());
  }
}

void check_bezier_fabrication_minimums(const Board& board, CopperDrcReport& report,
                                       const std::set<std::string>* selected = nullptr) {
  const auto minimum = static_cast<long double>(board.rules.minimum_track_width.value());
  if (minimum <= 0.0L) return;
  for (const auto& curve : board.track_beziers) {
    if (selected_for_dynamic_check(selected, curve.id) && curve.width.value() < minimum) {
      add_violation(report, CopperViolationKind::track_width, "track_bezier_width", {curve.id.value()},
                    minimum, static_cast<long double>(curve.width.value()));
    }
  }
}

[[nodiscard]] bool selected_for_dynamic_check(const std::set<std::string>* selected,
                                              const ObjectId& first,
                                              const ObjectId* second) {
  return selected == nullptr || selected->contains(first.value()) ||
         (second != nullptr && selected->contains(second->value()));
}

void check_fabrication_minimums(const Board& board, CopperDrcReport& report,
                                const std::set<std::string>* selected = nullptr) {
  const auto minimum_track = static_cast<long double>(board.rules.minimum_track_width.value());
  if (minimum_track > 0.0L) {
    for (const auto& track : board.tracks) {
      if (selected_for_dynamic_check(selected, track.id) && track.width.value() < minimum_track) {
        add_violation(report, CopperViolationKind::track_width, "track_width",
                      {track.id.value()}, minimum_track,
                      static_cast<long double>(track.width.value()));
      }
    }
    for (const auto& arc : board.track_arcs) {
      if (selected_for_dynamic_check(selected, arc.id) && arc.width.value() < minimum_track) {
        add_violation(report, CopperViolationKind::track_width, "track_arc_width",
                      {arc.id.value()}, minimum_track,
                      static_cast<long double>(arc.width.value()));
      }
    }
  }
  const auto minimum_via = static_cast<long double>(board.rules.minimum_via_diameter.value());
  const auto minimum_drill = static_cast<long double>(board.rules.minimum_through_drill.value());
  const auto minimum_annular =
      static_cast<long double>(board.rules.minimum_via_annular_width.value());
  for (const auto& via : board.vias) {
    if (!selected_for_dynamic_check(selected, via.id)) {
      continue;
    }
    if (minimum_via > 0.0L && via.diameter.value() < minimum_via) {
      add_violation(report, CopperViolationKind::via_diameter, "via_diameter",
                    {via.id.value()}, minimum_via,
                    static_cast<long double>(via.diameter.value()));
    }
    if (minimum_drill > 0.0L && via.drill.value() < minimum_drill) {
      add_violation(report, CopperViolationKind::via_drill, "via_drill", {via.id.value()},
                    minimum_drill, static_cast<long double>(via.drill.value()));
    }
    const auto annular =
        (static_cast<long double>(via.diameter.value()) - via.drill.value()) / 2.0L;
    if (minimum_annular > 0.0L && annular < minimum_annular) {
      add_violation(report, CopperViolationKind::via_annular_width, "via_annular_width",
                    {via.id.value()}, minimum_annular, std::max(0.0L, annular));
    }
  }
}

void check_hole_to_hole(const Board& board, CopperDrcReport& report,
                        const std::set<std::string>* selected = nullptr) {
  const auto required =
      static_cast<long double>(board.rules.hole_to_hole_clearance.value());
  if (required <= 0.0L) {
    return;
  }
  const auto holes = drill_shapes(board);
  for (std::size_t first = 0U; first < holes.size(); ++first) {
    if (!holes[first].mechanically_drilled) {
      continue;
    }
    for (std::size_t second = first + 1U; second < holes.size(); ++second) {
      if (!holes[second].mechanically_drilled) {
        continue;
      }
      if (!selected_for_dynamic_check(selected, holes[first].id, &holes[second].id)) {
        continue;
      }
      const auto actual = drill_to_drill_boundary_distance_nm(holes[first], holes[second]);
      if (actual < required) {
        add_violation(report, CopperViolationKind::hole_to_hole_clearance,
                      "hole_to_hole_clearance",
                      {holes[first].id.value(), holes[second].id.value()}, required, actual);
      }
    }
  }
}

void check_hole_clearance(const Board& board, const std::vector<CopperPadShape>& pads,
                          CopperDrcReport& report,
                          const std::set<std::string>* selected = nullptr) {
  const auto required = static_cast<long double>(board.rules.hole_clearance.value());
  if (required <= 0.0L) {
    return;
  }
  const auto holes = drill_shapes(board);
  const auto eligible = [&](const DrillShape& hole, const ObjectId& copper_id,
                            const NetId& copper_net) {
    return hole.id != copper_id && hole.net != copper_net &&
           selected_for_dynamic_check(selected, hole.id, &copper_id);
  };
  const auto report_if_too_close = [&](const DrillShape& hole, const ObjectId& copper_id,
                                       long double actual) {
    if (actual < required) {
      add_violation(report, CopperViolationKind::hole_clearance, "hole_clearance",
                    {hole.id.value(), copper_id.value()}, required, actual);
    }
  };
  for (const auto& hole : holes) {
    const auto hole_shape = drill_clearance_shape(hole);
    for (const auto& track : board.tracks) {
      if (!eligible(hole, track.id, track.net) || !contains_layer(hole.layers, track.layer)) {
        continue;
      }
      const auto actual = std::max(
          0.0L, segment_to_pad_copper_distance_nm(track.start, track.end, hole_shape) -
                    static_cast<long double>(track.width.value()) / 2.0L);
      report_if_too_close(hole, track.id, actual);
    }
    for (const auto& arc : board.track_arcs) {
      if (!eligible(hole, arc.id, arc.net) || !contains_layer(hole.layers, arc.layer)) {
        continue;
      }
      const auto actual = std::max(
          0.0L, circular_arc_to_pad_copper_distance_nm(arc.start, arc.middle, arc.end, hole_shape) -
                    static_cast<long double>(arc.width.value()) / 2.0L);
      report_if_too_close(hole, arc.id, actual);
    }
    for (const auto& via : board.vias) {
      if (!eligible(hole, via.id, via.net) || !share_layer(hole.layers, via.layers)) {
        continue;
      }
      const auto actual =
          std::max(0.0L, point_to_drill_boundary_distance_nm(via.position, hole) -
                             static_cast<long double>(via.diameter.value()) / 2.0L);
      report_if_too_close(hole, via.id, actual);
    }
    for (const auto& pad : pads) {
      if (!eligible(hole, pad.id, pad.net) || !share_layer(hole.layers, pad.layers)) {
        continue;
      }
      report_if_too_close(hole, pad.id, pad_to_pad_copper_distance_nm(hole_shape, pad));
    }
    for (const auto& zone : board.zones) {
      if (!eligible(hole, zone.id, zone.net)) {
        continue;
      }
      auto actual = std::numeric_limits<long double>::infinity();
      for (const auto& fill : zone.fills) {
        if (contains_layer(hole.layers, fill.layer)) {
          actual =
              std::min(actual, pad_to_filled_region_copper_distance_nm(hole_shape, fill));
        }
      }
      report_if_too_close(hole, zone.id, actual);
    }
  }
}

[[nodiscard]] std::uint64_t routed_length_impl(const Board& board, const NetId& net) {
  std::uint64_t result = 0;
  for (const auto& track : board.tracks) {
    if (track.net != net) {
      continue;
    }
    const auto length = geometry::segment_length_nm(track.start, track.end);
    const auto remaining = std::numeric_limits<std::uint64_t>::max() - result;
    result += length > remaining ? remaining : length;
  }
  for (const auto& arc : board.track_arcs) {
    if (arc.net != net) {
      continue;
    }
    const auto length = geometry::circular_arc_length_nm(arc.start, arc.middle, arc.end);
    const auto remaining = std::numeric_limits<std::uint64_t>::max() - result;
    result += length > remaining ? remaining : length;
  }
  for (const auto& curve : board.track_beziers) {
    if (curve.net != net) continue;
    const auto flattened = tessellate_track_bezier(curve);
    if (!flattened.certified) return std::numeric_limits<std::uint64_t>::max();
    for (const auto& segment : flattened.segments) {
      const auto length = geometry::segment_length_nm(segment.start, segment.end);
      const auto remaining = std::numeric_limits<std::uint64_t>::max() - result;
      result += length > remaining ? remaining : length;
    }
  }
  return result;
}

// Chord lengths are useful estimates, but must not certify differential skew for curved copper.
void check_bezier_skew(const Board& board, CopperDrcReport& report,
                       const std::set<std::string>* selected = nullptr) {
  std::set<NetId> touched;
  const auto collect = [&](const auto& objects) {
    for (const auto& object : objects)
      if (selected == nullptr || selected->contains(object.id.value())) touched.insert(object.net);
  };
  collect(board.tracks); collect(board.track_arcs); collect(board.track_beziers); collect(board.vias);
  Board straight = board;
  straight.track_beziers.clear();
  const auto interval = [&](const NetId& net) {
    long double lower = routed_length_impl(straight, net), upper = lower;
    for (const auto& curve : board.track_beziers) {
      if (curve.net != net) continue;
      const auto flattened = tessellate_track_bezier(curve);
      if (!flattened.certified) return std::pair{0.0L, std::numeric_limits<long double>::infinity()};
      lower += std::max(0.0L, flattened.length_lower_nm - 1.0L);
      upper += flattened.length_upper_nm + 1.0L;
    }
    return std::pair{lower, upper};
  };
  for (const auto& pair : board.differential_pairs) {
    if (!pair.maximum_skew || (!touched.contains(pair.positive_net) && !touched.contains(pair.negative_net))) continue;
    std::erase_if(report.violations, [&](const auto& violation) {
      return violation.kind == CopperViolationKind::differential_pair_skew &&
             std::find(violation.object_ids.begin(), violation.object_ids.end(), pair.id.value()) != violation.object_ids.end();
    });
    const auto positive = interval(pair.positive_net), negative = interval(pair.negative_net);
    const auto worst = std::max(std::abs(positive.second - negative.first),
                                std::abs(negative.second - positive.first));
    if (worst > pair.maximum_skew->value())
      add_violation(report, CopperViolationKind::differential_pair_skew, "differential_pair_skew",
                    {pair.id.value()}, pair.maximum_skew->value(), worst);
  }
}

void check_board_edge(const Board& board, const std::vector<CopperPadShape>& pads,
                      CopperDrcReport& report) {
  const auto edge_clearance =
      static_cast<long double>(board.rules.copper_to_edge_clearance.value());
  for (const auto& track : board.tracks) {
    const auto radius = static_cast<long double>(track.width.value()) / 2.0L;
    if (!geometry::segment_inside_board_area(track.start, track.end, board)) {
      add_violation(report, CopperViolationKind::copper_outside_board, "track_outside_copper_area",
                    {track.id.value()}, radius + edge_clearance, 0.0L);
      continue;
    }
    const auto actual = geometry::segment_board_boundary_distance_nm(track.start, track.end, board);
    const auto required = radius + edge_clearance;
    if (actual < required) {
      add_violation(report, CopperViolationKind::copper_edge_clearance, "track_edge_clearance",
                    {track.id.value()}, required, actual);
    }
  }
  for (const auto& arc : board.track_arcs) {
    const auto radius = static_cast<long double>(arc.width.value()) / 2.0L;
    if (!geometry::circular_arc_inside_board_area(arc.start, arc.middle, arc.end, board)) {
      add_violation(report, CopperViolationKind::copper_outside_board,
                    "track_arc_outside_copper_area", {arc.id.value()}, radius + edge_clearance,
                    0.0L);
      continue;
    }
    const auto actual =
        geometry::circular_arc_board_boundary_distance_nm(arc.start, arc.middle, arc.end, board);
    const auto required = radius + edge_clearance;
    if (actual < required) {
      add_violation(report, CopperViolationKind::copper_edge_clearance, "track_arc_edge_clearance",
                    {arc.id.value()}, required, actual);
    }
  }
  for (const auto& via : board.vias) {
    const auto actual = geometry::point_board_boundary_distance_nm(via.position, board);
    const auto required = static_cast<long double>(via.diameter.value()) / 2.0L + edge_clearance;
    if (geometry::locate_point_in_board_area(board, via.position) ==
        geometry::PointLocation::outside) {
      add_violation(report, CopperViolationKind::copper_outside_board, "via_outside_copper_area",
                    {via.id.value()}, required, 0.0L);
    } else if (actual < required) {
      add_violation(report, CopperViolationKind::copper_edge_clearance, "via_edge_clearance",
                    {via.id.value()}, required, actual);
    }
  }
  for (const auto& pad : pads) {
    const auto actual = pad_to_board_boundary_distance_nm(pad, board);
    const auto required = edge_clearance;
    if (!pad_inside_board_area(pad, board)) {
      add_violation(report, CopperViolationKind::copper_outside_board, "pad_outside_copper_area",
                    {pad.id.value()}, required, 0.0L);
    } else if (actual < required) {
      add_violation(report, CopperViolationKind::copper_edge_clearance, "pad_edge_clearance",
                    {pad.id.value()}, required, actual);
    }
  }
  for (const auto& zone : board.zones) {
    auto actual = std::numeric_limits<long double>::infinity();
    bool outside = false;
    for (const auto& fill : zone.fills) {
      outside = outside || !geometry::filled_region_inside_board_area(fill, board);
      actual = std::min(actual, geometry::filled_region_board_boundary_distance_nm(fill, board));
    }
    if (outside) {
      add_violation(report, CopperViolationKind::copper_outside_board, "zone_outside_copper_area",
                    {zone.id.value()}, edge_clearance, 0.0L);
    } else if (actual < edge_clearance) {
      add_violation(report, CopperViolationKind::copper_edge_clearance, "zone_edge_clearance",
                    {zone.id.value()}, edge_clearance, actual);
    }
  }
}

} // namespace

CopperDrcReport check_copper_rules(const Board& board) {
  if (!board.track_beziers.empty()) {
    const auto flattened = flatten_board_beziers_conservatively(board);
    CopperDrcReport report = check_copper_rules(flattened.board);
    remap_bezier_proxy_ids(report, flattened.proxy_sources);
    check_bezier_fabrication_minimums(board, report);
    check_bezier_skew(board, report);
    if (!flattened.certified) {
      for (const auto& curve : board.track_beziers) {
        const auto represented = std::any_of(flattened.proxy_sources.begin(), flattened.proxy_sources.end(),
                                             [&curve](const auto& entry) { return entry.second == curve.id; });
        if (!tessellate_track_bezier(curve).certified || !represented) {
          add_violation(report, CopperViolationKind::copper_outside_board,
                        "unsupported_track_bezier_geometry", {curve.id.value()}, 0.0L, 0.0L);
        }
      }
    }
    return report;
  }
  CopperDrcReport report;
  const auto clearance = [&board](const NetId& lhs, const NetId& rhs) {
    return static_cast<long double>(resolve_pairwise_clearance(board, lhs, rhs).value());
  };
  const auto pads = copper_pad_shapes(board);
  check_fabrication_minimums(board, report);
  check_hole_to_hole(board, report);
  check_hole_clearance(board, pads, report);
  check_board_edge(board, pads, report);

  for (std::size_t i = 0; i < board.tracks.size(); ++i) {
    const auto& lhs = board.tracks[i];
    for (std::size_t j = i + 1U; j < board.tracks.size(); ++j) {
      const auto& rhs = board.tracks[j];
      if (lhs.net == rhs.net || lhs.layer != rhs.layer) {
        continue;
      }
      const auto actual = geometry::segment_distance_nm(lhs.start, lhs.end, rhs.start, rhs.end);
      const auto required = (static_cast<long double>(lhs.width.value()) +
                             static_cast<long double>(rhs.width.value())) /
                                2.0L +
                            clearance(lhs.net, rhs.net);
      if (actual < required) {
        add_violation(report, CopperViolationKind::track_clearance, "track_clearance",
                      {lhs.id.value(), rhs.id.value()}, required, actual);
      }
    }
    for (const auto& arc : board.track_arcs) {
      if (lhs.net == arc.net || lhs.layer != arc.layer) {
        continue;
      }
      const auto actual = geometry::segment_circular_arc_distance_nm(lhs.start, lhs.end, arc.start,
                                                                     arc.middle, arc.end);
      const auto required = (static_cast<long double>(lhs.width.value()) +
                             static_cast<long double>(arc.width.value())) /
                                2.0L +
                            clearance(lhs.net, arc.net);
      if (actual < required) {
        add_violation(report, CopperViolationKind::track_clearance, "track_track_arc_clearance",
                      {lhs.id.value(), arc.id.value()}, required, actual);
      }
    }
    for (const auto& via : board.vias) {
      if (lhs.net == via.net || !contains_layer(via.layers, lhs.layer)) {
        continue;
      }
      const auto actual = geometry::point_segment_distance_nm(via.position, lhs.start, lhs.end);
      const auto required = (static_cast<long double>(lhs.width.value()) +
                             static_cast<long double>(via.diameter.value())) /
                                2.0L +
                            clearance(lhs.net, via.net);
      if (actual < required) {
        add_violation(report, CopperViolationKind::track_via_clearance, "track_via_clearance",
                      {lhs.id.value(), via.id.value()}, required, actual);
      }
    }
    for (const auto& pad : pads) {
      if (lhs.net == pad.net || !contains_layer(pad.layers, lhs.layer)) {
        continue;
      }
      const auto actual = segment_to_pad_copper_distance_nm(lhs.start, lhs.end, pad);
      const auto required =
          static_cast<long double>(lhs.width.value()) / 2.0L + clearance(lhs.net, pad.net);
      if (actual < required) {
        add_violation(report, CopperViolationKind::track_pad_clearance, "track_pad_clearance",
                      {lhs.id.value(), pad.id.value()}, required, actual);
      }
    }
    for (const auto& zone : board.zones) {
      if (lhs.net == zone.net) {
        continue;
      }
      auto actual = std::numeric_limits<long double>::infinity();
      for (const auto& fill : zone.fills) {
        if (lhs.layer == fill.layer) {
          actual = std::min(
              actual, geometry::segment_filled_region_copper_distance_nm(lhs.start, lhs.end, fill));
        }
      }
      const auto required =
          static_cast<long double>(lhs.width.value()) / 2.0L + clearance(lhs.net, zone.net);
      if (actual < required) {
        add_violation(report, CopperViolationKind::track_zone_clearance, "track_zone_clearance",
                      {lhs.id.value(), zone.id.value()}, required, actual);
      }
    }
  }

  for (std::size_t i = 0; i < board.track_arcs.size(); ++i) {
    const auto& lhs = board.track_arcs[i];
    for (std::size_t j = i + 1U; j < board.track_arcs.size(); ++j) {
      const auto& rhs = board.track_arcs[j];
      if (lhs.net == rhs.net || lhs.layer != rhs.layer) {
        continue;
      }
      const auto actual = geometry::circular_arcs_distance_nm(lhs.start, lhs.middle, lhs.end,
                                                              rhs.start, rhs.middle, rhs.end);
      const auto required = (static_cast<long double>(lhs.width.value()) +
                             static_cast<long double>(rhs.width.value())) /
                                2.0L +
                            clearance(lhs.net, rhs.net);
      if (actual < required) {
        add_violation(report, CopperViolationKind::track_clearance, "track_arc_clearance",
                      {lhs.id.value(), rhs.id.value()}, required, actual);
      }
    }
    for (const auto& via : board.vias) {
      if (lhs.net == via.net || !contains_layer(via.layers, lhs.layer)) {
        continue;
      }
      const auto actual =
          geometry::point_circular_arc_distance_nm(via.position, lhs.start, lhs.middle, lhs.end);
      const auto required = (static_cast<long double>(lhs.width.value()) +
                             static_cast<long double>(via.diameter.value())) /
                                2.0L +
                            clearance(lhs.net, via.net);
      if (actual < required) {
        add_violation(report, CopperViolationKind::track_via_clearance, "track_arc_via_clearance",
                      {lhs.id.value(), via.id.value()}, required, actual);
      }
    }
    for (const auto& pad : pads) {
      if (lhs.net == pad.net || !contains_layer(pad.layers, lhs.layer)) {
        continue;
      }
      const auto actual =
          circular_arc_to_pad_copper_distance_nm(lhs.start, lhs.middle, lhs.end, pad);
      const auto required =
          static_cast<long double>(lhs.width.value()) / 2.0L + clearance(lhs.net, pad.net);
      if (actual < required) {
        add_violation(report, CopperViolationKind::track_pad_clearance, "track_arc_pad_clearance",
                      {lhs.id.value(), pad.id.value()}, required, actual);
      }
    }
    for (const auto& zone : board.zones) {
      if (lhs.net == zone.net) {
        continue;
      }
      auto actual = std::numeric_limits<long double>::infinity();
      for (const auto& fill : zone.fills) {
        if (lhs.layer == fill.layer) {
          actual = std::min(actual, geometry::circular_arc_filled_region_copper_distance_nm(
                                        lhs.start, lhs.middle, lhs.end, fill));
        }
      }
      const auto required =
          static_cast<long double>(lhs.width.value()) / 2.0L + clearance(lhs.net, zone.net);
      if (actual < required) {
        add_violation(report, CopperViolationKind::track_zone_clearance, "track_arc_zone_clearance",
                      {lhs.id.value(), zone.id.value()}, required, actual);
      }
    }
  }

  for (std::size_t i = 0; i < board.vias.size(); ++i) {
    const auto& lhs = board.vias[i];
    for (std::size_t j = i + 1U; j < board.vias.size(); ++j) {
      const auto& rhs = board.vias[j];
      if (lhs.net == rhs.net || !share_layer(lhs.layers, rhs.layers)) {
        continue;
      }
      const auto actual = geometry::point_distance_nm(lhs.position, rhs.position);
      const auto required = (static_cast<long double>(lhs.diameter.value()) +
                             static_cast<long double>(rhs.diameter.value())) /
                                2.0L +
                            clearance(lhs.net, rhs.net);
      if (actual < required) {
        add_violation(report, CopperViolationKind::via_clearance, "via_clearance",
                      {lhs.id.value(), rhs.id.value()}, required, actual);
      }
    }
    for (const auto& pad : pads) {
      if (lhs.net == pad.net || !share_layer(lhs.layers, pad.layers)) {
        continue;
      }
      const auto actual = point_to_pad_copper_distance_nm(lhs.position, pad);
      const auto required =
          static_cast<long double>(lhs.diameter.value()) / 2.0L + clearance(lhs.net, pad.net);
      if (actual < required) {
        add_violation(report, CopperViolationKind::via_pad_clearance, "via_pad_clearance",
                      {lhs.id.value(), pad.id.value()}, required, actual);
      }
    }
    for (const auto& zone : board.zones) {
      if (lhs.net == zone.net) {
        continue;
      }
      auto actual = std::numeric_limits<long double>::infinity();
      for (const auto& fill : zone.fills) {
        if (contains_layer(lhs.layers, fill.layer)) {
          actual = std::min(actual,
                            geometry::point_filled_region_copper_distance_nm(lhs.position, fill));
        }
      }
      const auto required =
          static_cast<long double>(lhs.diameter.value()) / 2.0L + clearance(lhs.net, zone.net);
      if (actual < required) {
        add_violation(report, CopperViolationKind::via_zone_clearance, "via_zone_clearance",
                      {lhs.id.value(), zone.id.value()}, required, actual);
      }
    }
  }

  for (std::size_t i = 0; i < pads.size(); ++i) {
    for (std::size_t j = i + 1U; j < pads.size(); ++j) {
      if (pads[i].net == pads[j].net || !share_layer(pads[i].layers, pads[j].layers)) {
        continue;
      }
      const auto actual = pad_to_pad_copper_distance_nm(pads[i], pads[j]);
      const auto required = clearance(pads[i].net, pads[j].net);
      if (actual < required || (required == 0.0L && pad_copper_intersects(pads[i], pads[j]))) {
        add_violation(report, CopperViolationKind::pad_clearance, "pad_clearance",
                      {pads[i].id.value(), pads[j].id.value()}, required, actual);
      }
    }
    for (const auto& zone : board.zones) {
      if (pads[i].net == zone.net) {
        continue;
      }
      auto actual = std::numeric_limits<long double>::infinity();
      for (const auto& fill : zone.fills) {
        if (!contains_layer(pads[i].layers, fill.layer)) {
          continue;
        }
        actual = std::min(actual, pad_to_filled_region_copper_distance_nm(pads[i], fill));
      }
      const auto required = clearance(pads[i].net, zone.net);
      if (actual < required || (required == 0.0L && actual == 0.0L)) {
        add_violation(report, CopperViolationKind::pad_zone_clearance, "pad_zone_clearance",
                      {pads[i].id.value(), zone.id.value()}, required, actual);
      }
    }
  }

  for (std::size_t i = 0; i < board.zones.size(); ++i) {
    for (std::size_t j = i + 1U; j < board.zones.size(); ++j) {
      const auto& lhs = board.zones[i];
      const auto& rhs = board.zones[j];
      if (lhs.net == rhs.net) {
        continue;
      }
      auto actual = std::numeric_limits<long double>::infinity();
      for (const auto& lhs_fill : lhs.fills) {
        for (const auto& rhs_fill : rhs.fills) {
          if (lhs_fill.layer == rhs_fill.layer) {
            actual =
                std::min(actual, geometry::filled_regions_copper_distance_nm(lhs_fill, rhs_fill));
          }
        }
      }
      const auto required = clearance(lhs.net, rhs.net);
      if (actual < required || (required == 0.0L && actual == 0.0L)) {
        add_violation(report, CopperViolationKind::zone_clearance, "zone_clearance",
                      {lhs.id.value(), rhs.id.value()}, required, actual);
      }
    }
  }

  for (const auto& pair : board.differential_pairs) {
    if (pair.minimum_gap) {
      auto minimum_actual = std::numeric_limits<long double>::infinity();
      std::optional<ObjectId> closest_positive;
      std::optional<ObjectId> closest_negative;
      const auto centerlines = track_centerlines(board);
      for (const auto& positive : centerlines) {
        if (positive.net != pair.positive_net) {
          continue;
        }
        for (const auto& negative : centerlines) {
          if (negative.net != pair.negative_net || positive.layer != negative.layer) {
            continue;
          }
          const auto centerline = centerline_distance(positive, negative);
          const auto actual =
              std::max(0.0L, centerline - (static_cast<long double>(positive.width.value()) +
                                           static_cast<long double>(negative.width.value())) /
                                              2.0L);
          if (actual < minimum_actual) {
            minimum_actual = actual;
            closest_positive = positive.id;
            closest_negative = negative.id;
          }
        }
      }
      const auto required = static_cast<long double>(pair.minimum_gap->value());
      if (minimum_actual < required && closest_positive && closest_negative) {
        add_violation(report, CopperViolationKind::differential_pair_gap, "differential_pair_gap",
                      {pair.id.value(), closest_positive->value(), closest_negative->value()},
                      required, minimum_actual);
      }
    }
    if (pair.maximum_skew) {
      const auto positive_length = routed_net_centerline_length_nm(board, pair.positive_net);
      const auto negative_length = routed_net_centerline_length_nm(board, pair.negative_net);
      const auto difference = positive_length > negative_length ? positive_length - negative_length
                                                                : negative_length - positive_length;
      const auto actual = static_cast<long double>(difference);
      const auto required = static_cast<long double>(pair.maximum_skew->value());
      if (actual > required) {
        add_violation(report, CopperViolationKind::differential_pair_skew, "differential_pair_skew",
                      {pair.id.value()}, required, actual);
      }
    }
  }

  for (const auto& keepout : world_keepouts(board)) {
    if (keepout.prohibit.tracks) {
      for (const auto& track : board.tracks) {
        if (!keepout_contains_layer(keepout, track.layer)) {
          continue;
        }
        const auto radius = static_cast<long double>(track.width.value()) / 2.0L;
        if (segment_copper_overlaps_keepout(track.start, track.end, radius, keepout.area)) {
          const auto actual = std::max(0.0L, geometry::segment_polygon_boundary_distance_nm(
                                                 track.start, track.end, keepout.area) -
                                                 radius);
          add_violation(report, CopperViolationKind::keepout, "track_keepout",
                        {track.id.value(), keepout.id.value()}, 0.0L, actual);
        }
      }
      for (const auto& arc : board.track_arcs) {
        if (!keepout_contains_layer(keepout, arc.layer)) {
          continue;
        }
        const auto radius = static_cast<long double>(arc.width.value()) / 2.0L;
        const auto centerline_distance = geometry::circular_arc_polygon_copper_distance_nm(
            arc.start, arc.middle, arc.end, keepout.area);
        if (centerline_distance <= radius) {
          const auto actual = std::max(0.0L, centerline_distance - radius);
          add_violation(report, CopperViolationKind::keepout, "track_arc_keepout",
                        {arc.id.value(), keepout.id.value()}, 0.0L, actual);
        }
      }
    }
    if (keepout.prohibit.vias) {
      for (const auto& via : board.vias) {
        if (!share_layer(via.layers, keepout.layers)) {
          continue;
        }
        const auto radius = static_cast<long double>(via.diameter.value()) / 2.0L;
        if (point_copper_overlaps_keepout(via.position, radius, keepout.area)) {
          const auto actual = std::max(
              0.0L,
              geometry::point_polygon_boundary_distance_nm(via.position, keepout.area) - radius);
          add_violation(report, CopperViolationKind::keepout, "via_keepout",
                        {via.id.value(), keepout.id.value()}, 0.0L, actual);
        }
      }
    }
    if (keepout.prohibit.pads) {
      for (const auto& pad : pads) {
        if (!share_layer(pad.layers, keepout.layers) ||
            !pad_copper_overlaps_polygon(pad, keepout.area)) {
          continue;
        }
        const auto actual = pad_to_polygon_boundary_distance_nm(pad, keepout.area);
        add_violation(report, CopperViolationKind::keepout, "pad_keepout",
                      {pad.id.value(), keepout.id.value()}, 0.0L, actual);
      }
    }
    if (keepout.prohibit.copper_pours) {
      for (const auto& zone : board.zones) {
        bool overlaps = false;
        for (const auto& fill : zone.fills) {
          if (keepout_contains_layer(keepout, fill.layer) &&
              geometry::filled_region_overlaps_polygon(fill, keepout.area)) {
            overlaps = true;
            break;
          }
        }
        if (overlaps) {
          add_violation(report, CopperViolationKind::keepout, "zone_keepout",
                        {zone.id.value(), keepout.id.value()}, 0.0L, 0.0L);
        }
      }
    }
  }
  return report;
}

CopperDrcReport check_dynamic_copper_rules(const Board& board,
                                           const std::vector<ObjectId>& dynamic_objects) {
  if (!board.track_beziers.empty()) {
    const auto flattened = flatten_board_beziers_conservatively(board);
    std::vector<ObjectId> selected = dynamic_objects;
    std::set<std::string> requested;
    for (const auto& id : dynamic_objects) requested.insert(id.value());
    for (const auto& [proxy, source] : flattened.proxy_sources) {
      if (requested.contains(source.value())) selected.emplace_back(proxy);
    }
    CopperDrcReport report = check_dynamic_copper_rules(flattened.board, selected);
    remap_bezier_proxy_ids(report, flattened.proxy_sources);
    check_bezier_fabrication_minimums(board, report, &requested);
    check_bezier_skew(board, report, &requested);
    if (!flattened.certified) {
      for (const auto& curve : board.track_beziers) {
        const auto represented = std::any_of(flattened.proxy_sources.begin(), flattened.proxy_sources.end(),
                                             [&curve](const auto& entry) { return entry.second == curve.id; });
        if (requested.contains(curve.id.value()) &&
            (!tessellate_track_bezier(curve).certified || !represented)) {
          add_violation(report, CopperViolationKind::copper_outside_board,
                        "unsupported_track_bezier_geometry", {curve.id.value()}, 0.0L, 0.0L);
        }
      }
    }
    return report;
  }
  CopperDrcReport report;
  std::set<std::string> selected;
  for (const auto& id : dynamic_objects) {
    selected.insert(id.value());
  }
  if (selected.empty()) {
    return report;
  }
  const auto is_selected = [&selected](const ObjectId& id) {
    return selected.contains(id.value());
  };
  check_fabrication_minimums(board, report, &selected);
  check_hole_to_hole(board, report, &selected);
  const auto clearance = [&board](const NetId& lhs, const NetId& rhs) {
    return static_cast<long double>(resolve_pairwise_clearance(board, lhs, rhs).value());
  };
  const auto pads = copper_pad_shapes(board);
  check_hole_clearance(board, pads, report, &selected);
  const auto edge_clearance =
      static_cast<long double>(board.rules.copper_to_edge_clearance.value());
  struct BroadphaseObject {
    ObjectId id;
    RoutingComputePrimitiveBounds bounds;
  };
  std::map<LayerId, std::uint32_t> layer_codes;
  for (const auto& layer : board.layers)
    if (layer.kind == LayerKind::copper)
      layer_codes.emplace(layer.id, static_cast<std::uint32_t>(layer_codes.size()));
  std::map<NetId, std::uint32_t> net_codes;
  for (const auto& net : board.nets)
    net_codes.emplace(net.id, static_cast<std::uint32_t>(net_codes.size()));
  auto maximum_clearance = board.rules.default_clearance.value();
  for (const auto& net_class : board.net_classes)
    if (net_class.clearance)
      maximum_clearance = std::max(maximum_clearance, net_class.clearance->value());
  std::vector<BroadphaseObject> broadphase_objects;
  const auto append_bounds = [&](const ObjectId& id, const NetId& net,
                                 const std::vector<LayerId>& layers,
                                 const std::vector<Point>& points, long double radius) {
    if (points.empty())
      return;
    const auto expansion = static_cast<std::int64_t>(std::ceil(radius)) + maximum_clearance;
    const auto [minimum_x, maximum_x] = std::minmax_element(
        points.begin(), points.end(), [](Point a, Point b) { return a.x < b.x; });
    const auto [minimum_y, maximum_y] = std::minmax_element(
        points.begin(), points.end(), [](Point a, Point b) { return a.y < b.y; });
    for (const auto& layer : layers) {
      const auto layer_code = layer_codes.find(layer);
      if (layer_code == layer_codes.end())
        continue;
      broadphase_objects.push_back(BroadphaseObject{
          id, RoutingComputePrimitiveBounds{
                  minimum_x->x.value() - expansion, minimum_y->y.value() - expansion,
                  maximum_x->x.value() + expansion, maximum_y->y.value() + expansion,
                  layer_code->second, net_codes.contains(net) ? net_codes.at(net) : 0xffffffffU}});
    }
  };
  for (const auto& track : board.tracks)
    append_bounds(track.id, track.net, {track.layer}, {track.start, track.end},
                  track.width.value() / 2.0L);
  for (const auto& arc : board.track_arcs) {
    auto points = geometry::circular_arc_extrema(arc.start, arc.middle, arc.end);
    points.insert(points.end(), {arc.start, arc.middle, arc.end});
    append_bounds(arc.id, arc.net, {arc.layer}, points, arc.width.value() / 2.0L);
  }
  for (const auto& via : board.vias)
    append_bounds(via.id, via.net, via.layers, {via.position}, via.diameter.value() / 2.0L);
  for (const auto& pad : pads)
    append_bounds(pad.id, pad.net, pad.layers, {pad.position}, pad.conservative_radius_nm);
  std::vector<RoutingComputePrimitiveBounds> broadphase_bounds;
  broadphase_bounds.reserve(broadphase_objects.size());
  for (const auto& object : broadphase_objects)
    broadphase_bounds.push_back(object.bounds);
  const auto broadphase = discover_routing_broadphase_pairs(
      broadphase_bounds, std::max<std::int64_t>(1'000'000, maximum_clearance * 2), 1'048'576U,
      true);
  std::set<std::pair<std::string, std::string>> possible_pairs;
  if (broadphase.ok() && !broadphase.budget_exhausted) {
    for (const auto [first, second] : broadphase.candidate_pairs) {
      auto ids =
          std::minmax(broadphase_objects[first].id.value(), broadphase_objects[second].id.value());
      possible_pairs.emplace(ids.first, ids.second);
    }
  }
  std::set<std::pair<std::string, std::string>> simple_copper_clear_pairs;
  if (broadphase.ok() && !broadphase.budget_exhausted) {
    std::vector<RoutingComputeCopperPrimitive> simple_primitives;
    std::vector<std::pair<std::uint32_t, std::uint32_t>> simple_pairs;
    std::vector<std::pair<std::string, std::string>> simple_pair_ids;
    const auto append_simple = [&](const ObjectId& id, std::uint32_t layer,
                                   RoutingComputeCopperPrimitive& primitive) {
      if (const auto track = std::find_if(board.tracks.begin(), board.tracks.end(),
                                          [&](const auto& value) { return value.id == id; });
          track != board.tracks.end()) {
        primitive = {RoutingComputeCopperKind::segment,
                     track->start.x.value(), track->start.y.value(), track->end.x.value(),
                     track->end.y.value(), track->width.value() / 2, layer,
                     net_codes.contains(track->net) ? net_codes.at(track->net) : 0xffffffffU};
        return true;
      }
      if (const auto via = std::find_if(board.vias.begin(), board.vias.end(),
                                        [&](const auto& value) { return value.id == id; });
          via != board.vias.end()) {
        primitive = {RoutingComputeCopperKind::circle,
                     via->position.x.value(), via->position.y.value(), via->position.x.value(),
                     via->position.y.value(), via->diameter.value() / 2, layer,
                     net_codes.contains(via->net) ? net_codes.at(via->net) : 0xffffffffU};
        return true;
      }
      return false;
    };
    for (const auto [first, second] : broadphase.candidate_pairs) {
      RoutingComputeCopperPrimitive a, b;
      if (!append_simple(broadphase_objects[first].id, broadphase_objects[first].bounds.layer, a) ||
          !append_simple(broadphase_objects[second].id, broadphase_objects[second].bounds.layer, b))
        continue;
      const auto base = static_cast<std::uint32_t>(simple_primitives.size());
      simple_primitives.push_back(a);
      simple_primitives.push_back(b);
      simple_pairs.emplace_back(base, base + 1U);
      const auto ids =
          std::minmax(broadphase_objects[first].id.value(), broadphase_objects[second].id.value());
      simple_pair_ids.emplace_back(ids.first, ids.second);
    }
    const auto simple =
        assess_simple_copper_pairs(simple_primitives, simple_pairs, maximum_clearance, 2'048, true);
    if (simple.ok() && simple.pairs.size() == simple_pair_ids.size()) {
      std::map<std::pair<std::string, std::string>, bool> all_clear;
      for (std::size_t pair = 0U; pair < simple.pairs.size(); ++pair) {
        const auto clear = simple.pairs[pair].possible_conflict == 0U;
        const auto [found, inserted] = all_clear.try_emplace(simple_pair_ids[pair], clear);
        if (!inserted)
          found->second = found->second && clear;
      }
      for (const auto& [ids, clear] : all_clear)
        if (clear)
          simple_copper_clear_pairs.insert(ids);
    }
  }
  const auto possible_interaction = [&](const ObjectId& first, const ObjectId& second) {
    if (!broadphase.ok() || broadphase.budget_exhausted)
      return true;
    const auto ids = std::minmax(first.value(), second.value());
    const auto key = std::pair{ids.first, ids.second};
    return possible_pairs.contains(key) && !simple_copper_clear_pairs.contains(key);
  };

  const auto check_track_edge = [&](const TrackSegment& track) {
    const auto required = static_cast<long double>(track.width.value()) / 2.0L + edge_clearance;
    if (!geometry::segment_inside_board_area(track.start, track.end, board)) {
      add_violation(report, CopperViolationKind::copper_outside_board, "track_outside_copper_area",
                    {track.id.value()}, required, 0.0L);
    } else {
      const auto actual =
          geometry::segment_board_boundary_distance_nm(track.start, track.end, board);
      if (actual < required) {
        add_violation(report, CopperViolationKind::copper_edge_clearance, "track_edge_clearance",
                      {track.id.value()}, required, actual);
      }
    }
  };
  const auto check_arc_edge = [&](const TrackArc& arc) {
    const auto required = static_cast<long double>(arc.width.value()) / 2.0L + edge_clearance;
    if (!geometry::circular_arc_inside_board_area(arc.start, arc.middle, arc.end, board)) {
      add_violation(report, CopperViolationKind::copper_outside_board,
                    "track_arc_outside_copper_area", {arc.id.value()}, required, 0.0L);
    } else {
      const auto actual =
          geometry::circular_arc_board_boundary_distance_nm(arc.start, arc.middle, arc.end, board);
      if (actual < required) {
        add_violation(report, CopperViolationKind::copper_edge_clearance,
                      "track_arc_edge_clearance", {arc.id.value()}, required, actual);
      }
    }
  };
  const auto check_via_edge = [&](const Via& via) {
    const auto required = static_cast<long double>(via.diameter.value()) / 2.0L + edge_clearance;
    if (geometry::locate_point_in_board_area(board, via.position) ==
        geometry::PointLocation::outside) {
      add_violation(report, CopperViolationKind::copper_outside_board, "via_outside_copper_area",
                    {via.id.value()}, required, 0.0L);
    } else {
      const auto actual = geometry::point_board_boundary_distance_nm(via.position, board);
      if (actual < required) {
        add_violation(report, CopperViolationKind::copper_edge_clearance, "via_edge_clearance",
                      {via.id.value()}, required, actual);
      }
    }
  };

  for (const auto& lhs : board.tracks) {
    if (!is_selected(lhs.id)) {
      continue;
    }
    check_track_edge(lhs);
    for (const auto& rhs : board.tracks) {
      if (lhs.id == rhs.id || lhs.net == rhs.net || lhs.layer != rhs.layer ||
          !possible_interaction(lhs.id, rhs.id)) {
        continue;
      }
      const auto actual = geometry::segment_distance_nm(lhs.start, lhs.end, rhs.start, rhs.end);
      const auto required = (static_cast<long double>(lhs.width.value()) +
                             static_cast<long double>(rhs.width.value())) /
                                2.0L +
                            clearance(lhs.net, rhs.net);
      if (actual < required) {
        add_violation(report, CopperViolationKind::track_clearance, "track_clearance",
                      {lhs.id.value(), rhs.id.value()}, required, actual);
      }
    }
    for (const auto& arc : board.track_arcs) {
      if (lhs.net == arc.net || lhs.layer != arc.layer || !possible_interaction(lhs.id, arc.id)) {
        continue;
      }
      const auto actual = geometry::segment_circular_arc_distance_nm(lhs.start, lhs.end, arc.start,
                                                                     arc.middle, arc.end);
      const auto required = (static_cast<long double>(lhs.width.value()) +
                             static_cast<long double>(arc.width.value())) /
                                2.0L +
                            clearance(lhs.net, arc.net);
      if (actual < required) {
        add_violation(report, CopperViolationKind::track_clearance, "track_track_arc_clearance",
                      {lhs.id.value(), arc.id.value()}, required, actual);
      }
    }
    for (const auto& via : board.vias) {
      if (lhs.net == via.net || !contains_layer(via.layers, lhs.layer) ||
          !possible_interaction(lhs.id, via.id)) {
        continue;
      }
      const auto actual = geometry::point_segment_distance_nm(via.position, lhs.start, lhs.end);
      const auto required = (static_cast<long double>(lhs.width.value()) +
                             static_cast<long double>(via.diameter.value())) /
                                2.0L +
                            clearance(lhs.net, via.net);
      if (actual < required) {
        add_violation(report, CopperViolationKind::track_via_clearance, "track_via_clearance",
                      {lhs.id.value(), via.id.value()}, required, actual);
      }
    }
    for (const auto& pad : pads) {
      if (lhs.net == pad.net || !contains_layer(pad.layers, lhs.layer) ||
          !possible_interaction(lhs.id, pad.id)) {
        continue;
      }
      const auto actual = segment_to_pad_copper_distance_nm(lhs.start, lhs.end, pad);
      const auto required =
          static_cast<long double>(lhs.width.value()) / 2.0L + clearance(lhs.net, pad.net);
      if (actual < required) {
        add_violation(report, CopperViolationKind::track_pad_clearance, "track_pad_clearance",
                      {lhs.id.value(), pad.id.value()}, required, actual);
      }
    }
    for (const auto& zone : board.zones) {
      if (lhs.net == zone.net) {
        continue;
      }
      auto actual = std::numeric_limits<long double>::infinity();
      for (const auto& fill : zone.fills) {
        if (lhs.layer == fill.layer) {
          actual = std::min(
              actual, geometry::segment_filled_region_copper_distance_nm(lhs.start, lhs.end, fill));
        }
      }
      const auto required =
          static_cast<long double>(lhs.width.value()) / 2.0L + clearance(lhs.net, zone.net);
      if (actual < required) {
        add_violation(report, CopperViolationKind::track_zone_clearance, "track_zone_clearance",
                      {lhs.id.value(), zone.id.value()}, required, actual);
      }
    }
  }

  for (const auto& lhs : board.track_arcs) {
    if (!is_selected(lhs.id)) {
      continue;
    }
    check_arc_edge(lhs);
    for (const auto& track : board.tracks) {
      if (lhs.net == track.net || lhs.layer != track.layer ||
          !possible_interaction(lhs.id, track.id)) {
        continue;
      }
      const auto actual = geometry::segment_circular_arc_distance_nm(
          track.start, track.end, lhs.start, lhs.middle, lhs.end);
      const auto required = (static_cast<long double>(lhs.width.value()) +
                             static_cast<long double>(track.width.value())) /
                                2.0L +
                            clearance(lhs.net, track.net);
      if (actual < required) {
        add_violation(report, CopperViolationKind::track_clearance, "track_track_arc_clearance",
                      {track.id.value(), lhs.id.value()}, required, actual);
      }
    }
    for (const auto& rhs : board.track_arcs) {
      if (lhs.id == rhs.id || lhs.net == rhs.net || lhs.layer != rhs.layer ||
          !possible_interaction(lhs.id, rhs.id)) {
        continue;
      }
      const auto actual = geometry::circular_arcs_distance_nm(lhs.start, lhs.middle, lhs.end,
                                                              rhs.start, rhs.middle, rhs.end);
      const auto required = (static_cast<long double>(lhs.width.value()) +
                             static_cast<long double>(rhs.width.value())) /
                                2.0L +
                            clearance(lhs.net, rhs.net);
      if (actual < required) {
        add_violation(report, CopperViolationKind::track_clearance, "track_arc_clearance",
                      {lhs.id.value(), rhs.id.value()}, required, actual);
      }
    }
    for (const auto& via : board.vias) {
      if (lhs.net == via.net || !contains_layer(via.layers, lhs.layer) ||
          !possible_interaction(lhs.id, via.id)) {
        continue;
      }
      const auto actual =
          geometry::point_circular_arc_distance_nm(via.position, lhs.start, lhs.middle, lhs.end);
      const auto required = (static_cast<long double>(lhs.width.value()) +
                             static_cast<long double>(via.diameter.value())) /
                                2.0L +
                            clearance(lhs.net, via.net);
      if (actual < required) {
        add_violation(report, CopperViolationKind::track_via_clearance, "track_arc_via_clearance",
                      {lhs.id.value(), via.id.value()}, required, actual);
      }
    }
    for (const auto& pad : pads) {
      if (lhs.net == pad.net || !contains_layer(pad.layers, lhs.layer) ||
          !possible_interaction(lhs.id, pad.id)) {
        continue;
      }
      const auto actual =
          circular_arc_to_pad_copper_distance_nm(lhs.start, lhs.middle, lhs.end, pad);
      const auto required =
          static_cast<long double>(lhs.width.value()) / 2.0L + clearance(lhs.net, pad.net);
      if (actual < required) {
        add_violation(report, CopperViolationKind::track_pad_clearance, "track_arc_pad_clearance",
                      {lhs.id.value(), pad.id.value()}, required, actual);
      }
    }
    for (const auto& zone : board.zones) {
      if (lhs.net == zone.net) {
        continue;
      }
      auto actual = std::numeric_limits<long double>::infinity();
      for (const auto& fill : zone.fills) {
        if (lhs.layer == fill.layer) {
          actual = std::min(actual, geometry::circular_arc_filled_region_copper_distance_nm(
                                        lhs.start, lhs.middle, lhs.end, fill));
        }
      }
      const auto required =
          static_cast<long double>(lhs.width.value()) / 2.0L + clearance(lhs.net, zone.net);
      if (actual < required) {
        add_violation(report, CopperViolationKind::track_zone_clearance, "track_arc_zone_clearance",
                      {lhs.id.value(), zone.id.value()}, required, actual);
      }
    }
  }

  for (const auto& lhs : board.vias) {
    if (!is_selected(lhs.id)) {
      continue;
    }
    check_via_edge(lhs);
    // Track/via and arc/via rules are normally evaluated from the track/arc side. In a bounded
    // proposal check that side may be immutable and therefore unselected, so visit the symmetric
    // relationship here. Skip selected tracks/arcs because their loops above already reported the
    // same physical pair.
    for (const auto& track : board.tracks) {
      if (is_selected(track.id) || lhs.net == track.net ||
          !contains_layer(lhs.layers, track.layer) ||
          !possible_interaction(lhs.id, track.id)) {
        continue;
      }
      const auto actual = geometry::point_segment_distance_nm(lhs.position, track.start, track.end);
      const auto required = (static_cast<long double>(lhs.diameter.value()) +
                             static_cast<long double>(track.width.value())) /
                                2.0L +
                            clearance(lhs.net, track.net);
      if (actual < required) {
        add_violation(report, CopperViolationKind::track_via_clearance, "track_via_clearance",
                      {track.id.value(), lhs.id.value()}, required, actual);
      }
    }
    for (const auto& arc : board.track_arcs) {
      if (is_selected(arc.id) || lhs.net == arc.net || !contains_layer(lhs.layers, arc.layer) ||
          !possible_interaction(lhs.id, arc.id)) {
        continue;
      }
      const auto actual = geometry::point_circular_arc_distance_nm(
          lhs.position, arc.start, arc.middle, arc.end);
      const auto required = (static_cast<long double>(lhs.diameter.value()) +
                             static_cast<long double>(arc.width.value())) /
                                2.0L +
                            clearance(lhs.net, arc.net);
      if (actual < required) {
        add_violation(report, CopperViolationKind::track_via_clearance,
                      "track_arc_via_clearance", {arc.id.value(), lhs.id.value()}, required,
                      actual);
      }
    }
    for (const auto& rhs : board.vias) {
      if (lhs.id == rhs.id || lhs.net == rhs.net || !share_layer(lhs.layers, rhs.layers) ||
          !possible_interaction(lhs.id, rhs.id)) {
        continue;
      }
      const auto actual = geometry::point_distance_nm(lhs.position, rhs.position);
      const auto required = (static_cast<long double>(lhs.diameter.value()) +
                             static_cast<long double>(rhs.diameter.value())) /
                                2.0L +
                            clearance(lhs.net, rhs.net);
      if (actual < required) {
        add_violation(report, CopperViolationKind::via_clearance, "via_clearance",
                      {lhs.id.value(), rhs.id.value()}, required, actual);
      }
    }
    for (const auto& pad : pads) {
      if (lhs.net == pad.net || !share_layer(lhs.layers, pad.layers) ||
          !possible_interaction(lhs.id, pad.id)) {
        continue;
      }
      const auto actual = point_to_pad_copper_distance_nm(lhs.position, pad);
      const auto required =
          static_cast<long double>(lhs.diameter.value()) / 2.0L + clearance(lhs.net, pad.net);
      if (actual < required) {
        add_violation(report, CopperViolationKind::via_pad_clearance, "via_pad_clearance",
                      {lhs.id.value(), pad.id.value()}, required, actual);
      }
    }
    for (const auto& zone : board.zones) {
      if (lhs.net == zone.net) {
        continue;
      }
      auto actual = std::numeric_limits<long double>::infinity();
      for (const auto& fill : zone.fills) {
        if (contains_layer(lhs.layers, fill.layer)) {
          actual = std::min(actual,
                            geometry::point_filled_region_copper_distance_nm(lhs.position, fill));
        }
      }
      const auto required =
          static_cast<long double>(lhs.diameter.value()) / 2.0L + clearance(lhs.net, zone.net);
      if (actual < required) {
        add_violation(report, CopperViolationKind::via_zone_clearance, "via_zone_clearance",
                      {lhs.id.value(), zone.id.value()}, required, actual);
      }
    }
  }

  std::set<NetId> touched_nets;
  for (const auto& track : board.tracks) {
    if (is_selected(track.id)) {
      touched_nets.insert(track.net);
    }
  }
  for (const auto& arc : board.track_arcs) {
    if (is_selected(arc.id)) {
      touched_nets.insert(arc.net);
    }
  }
  const auto centerlines = track_centerlines(board);
  for (const auto& pair : board.differential_pairs) {
    if (!touched_nets.contains(pair.positive_net) && !touched_nets.contains(pair.negative_net)) {
      continue;
    }
    if (pair.minimum_gap) {
      auto minimum_actual = std::numeric_limits<long double>::infinity();
      std::optional<ObjectId> closest_positive;
      std::optional<ObjectId> closest_negative;
      for (const auto& positive : centerlines) {
        if (positive.net != pair.positive_net) {
          continue;
        }
        for (const auto& negative : centerlines) {
          if (negative.net != pair.negative_net || positive.layer != negative.layer) {
            continue;
          }
          const auto centerline = centerline_distance(positive, negative);
          const auto actual =
              std::max(0.0L, centerline - (static_cast<long double>(positive.width.value()) +
                                           static_cast<long double>(negative.width.value())) /
                                              2.0L);
          if (actual < minimum_actual) {
            minimum_actual = actual;
            closest_positive = positive.id;
            closest_negative = negative.id;
          }
        }
      }
      const auto required = static_cast<long double>(pair.minimum_gap->value());
      if (minimum_actual < required && closest_positive && closest_negative) {
        add_violation(report, CopperViolationKind::differential_pair_gap, "differential_pair_gap",
                      {pair.id.value(), closest_positive->value(), closest_negative->value()},
                      required, minimum_actual);
      }
    }
    if (pair.maximum_skew) {
      const auto positive_length = routed_net_centerline_length_nm(board, pair.positive_net);
      const auto negative_length = routed_net_centerline_length_nm(board, pair.negative_net);
      const auto difference = positive_length > negative_length ? positive_length - negative_length
                                                                : negative_length - positive_length;
      const auto actual = static_cast<long double>(difference);
      const auto required = static_cast<long double>(pair.maximum_skew->value());
      if (actual > required) {
        add_violation(report, CopperViolationKind::differential_pair_skew, "differential_pair_skew",
                      {pair.id.value()}, required, actual);
      }
    }
  }

  for (const auto& keepout : world_keepouts(board)) {
    if (keepout.prohibit.tracks) {
      for (const auto& track : board.tracks) {
        if (!is_selected(track.id) || !keepout_contains_layer(keepout, track.layer)) {
          continue;
        }
        const auto radius = static_cast<long double>(track.width.value()) / 2.0L;
        if (segment_copper_overlaps_keepout(track.start, track.end, radius, keepout.area)) {
          const auto actual = std::max(0.0L, geometry::segment_polygon_boundary_distance_nm(
                                                 track.start, track.end, keepout.area) -
                                                 radius);
          add_violation(report, CopperViolationKind::keepout, "track_keepout",
                        {track.id.value(), keepout.id.value()}, 0.0L, actual);
        }
      }
      for (const auto& arc : board.track_arcs) {
        if (!is_selected(arc.id) || !keepout_contains_layer(keepout, arc.layer)) {
          continue;
        }
        const auto radius = static_cast<long double>(arc.width.value()) / 2.0L;
        const auto distance = geometry::circular_arc_polygon_copper_distance_nm(
            arc.start, arc.middle, arc.end, keepout.area);
        if (distance <= radius) {
          add_violation(report, CopperViolationKind::keepout, "track_arc_keepout",
                        {arc.id.value(), keepout.id.value()}, 0.0L,
                        std::max(0.0L, distance - radius));
        }
      }
    }
    if (keepout.prohibit.vias) {
      for (const auto& via : board.vias) {
        if (!is_selected(via.id) || !share_layer(via.layers, keepout.layers)) {
          continue;
        }
        const auto radius = static_cast<long double>(via.diameter.value()) / 2.0L;
        if (point_copper_overlaps_keepout(via.position, radius, keepout.area)) {
          const auto actual = std::max(
              0.0L,
              geometry::point_polygon_boundary_distance_nm(via.position, keepout.area) - radius);
          add_violation(report, CopperViolationKind::keepout, "via_keepout",
                        {via.id.value(), keepout.id.value()}, 0.0L, actual);
        }
      }
    }
  }
  return report;
}

std::uint64_t routed_net_centerline_length_nm(const Board& board, const NetId& net) {
  return routed_length_impl(board, net);
}

} // namespace flux
