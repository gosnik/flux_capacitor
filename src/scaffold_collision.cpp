#include "scaffold_collision.hpp"

#include "flux/core/bezier_copper.hpp"
#include "flux/core/copper_geometry.hpp"
#include "flux/core/geometry.hpp"

#include <algorithm>
#include <cmath>
#include <map>
#include <set>
#include <tuple>

namespace flux {

ScaffoldCollisionIndex::ScaffoldCollisionIndex(const Board& board) : source_(board) {
  if (!board.track_beziers.empty()) {
    auto flattened = flatten_board_beziers_conservatively(board);
    if (!flattened.certified) return;
    source_ = std::move(flattened.board);
  }
  context_ = source_;
  context_.tracks.clear();
  context_.track_arcs.clear();
  context_.track_beziers.clear();
  context_.vias.clear();
  // Keep footprint-local keepouts and transforms even when no pad is nearby.
  for (auto& footprint : context_.footprints) footprint.pads.clear();
  clearance_ = std::max({board.rules.default_clearance.value(), board.rules.minimum_clearance.value(),
                        board.rules.hole_clearance.value(), board.rules.hole_to_hole_clearance.value()});
  for (const auto& rules : board.net_classes)
    if (rules.clearance) clearance_ = std::max(clearance_, static_cast<long double>(rules.clearance->value()));
  for (const auto& pair : board.differential_pairs)
    if (pair.minimum_gap)
      clearance_ = std::max(clearance_, static_cast<long double>(pair.minimum_gap->value()));
  std::vector<IndexedClearancePrimitive> primitives;
  const auto append = [&](Point a, Point b, long double radius, Owner owner) {
    const auto id = static_cast<std::uint32_t>(owners_.size());
    owners_.push_back(owner);
    primitives.push_back({a.x.value(), a.y.value(), b.x.value(), b.y.value(), id,
                          {}, {}, radius});
  };
  for (std::size_t i = 0; i < source_.tracks.size(); ++i) {
    const auto& track = source_.tracks[i];
    append(track.start, track.end, track.width.value() / 2.0L, {Kind::track, i});
  }
  for (std::size_t i = 0; i < source_.track_arcs.size(); ++i) {
    const auto& arc = source_.track_arcs[i];
    auto points = geometry::circular_arc_extrema(arc.start, arc.middle, arc.end);
    points.insert(points.end(), {arc.start, arc.middle, arc.end});
    // Enclose the extrema's rectangle, not only the extrema themselves: the farthest
    // radial point from an endpoint need not be an x/y extremum of the arc.
    auto min_x = arc.start.x, max_x = arc.start.x, min_y = arc.start.y, max_y = arc.start.y;
    for (const auto p : points) {
      min_x = std::min(min_x, p.x); max_x = std::max(max_x, p.x);
      min_y = std::min(min_y, p.y); max_y = std::max(max_y, p.y);
    }
    points = {{min_x, min_y}, {min_x, max_y}, {max_x, min_y}, {max_x, max_y}};
    long double radius = 0;
    for (const auto p : points)
      radius = std::max(radius, std::hypot(static_cast<long double>(p.x.value()) - arc.start.x.value(),
                                         static_cast<long double>(p.y.value()) - arc.start.y.value()));
    append(arc.start, arc.start, radius + arc.width.value() / 2.0L, {Kind::arc, i});
  }
  for (std::size_t i = 0; i < source_.vias.size(); ++i) {
    const auto& via = source_.vias[i];
    append(via.position, via.position, std::max(via.diameter.value(), via.drill.value()) / 2.0L,
           {Kind::via, i});
  }
  std::map<std::string, Owner> pad_owners;
  for (std::size_t i = 0; i < source_.footprints.size(); ++i)
    for (std::size_t j = 0; j < source_.footprints[i].pads.size(); ++j)
      pad_owners.emplace(source_.footprints[i].pads[j].id.value(), Owner{Kind::footprint, i, j});
  for (const auto& pad : copper_pad_shapes(source_))
    append(pad.position, pad.position, pad.conservative_radius_nm, pad_owners.at(pad.id.value()));
  for (const auto& drill : drill_shapes(source_)) {
    if (drill.owner_kind != DrillOwnerKind::pad) continue;
    append(drill.position, drill.position, std::max(drill.width.value(), drill.height.value()) / 2.0L,
           pad_owners.at(drill.id.value()));
  }
  IndexedClearanceMapConfig config;
  config.width = config.height = 1U;
  config.materialize_cells = false;
  auto built = build_indexed_clearance_map(config, std::move(primitives));
  if (built.status == IndexedClearanceBuildStatus::ok) index_ = std::move(built.map);
}

Board ScaffoldCollisionIndex::nearby(Point a, Point b, long double radius) const {
  if (!index_) return source_; // exact slow fallback; never interpret index failure as free
  Board result = context_;
  std::set<std::tuple<Kind, std::size_t, std::size_t>> copied;
  for (const auto primitive : index_->segment_candidates(a.x.value(), a.y.value(), b.x.value(),
                                                        b.y.value(), radius, clearance_)) {
    const auto& owner = owners_[index_->primitives()[primitive].owner_id];
    copied.emplace(owner.kind, owner.index, owner.pad_index);
  }
  for (const auto& [kind, index, pad_index] : copied) {
    switch (kind) {
    case Kind::track: result.tracks.push_back(source_.tracks[index]); break;
    case Kind::arc: result.track_arcs.push_back(source_.track_arcs[index]); break;
    case Kind::via: result.vias.push_back(source_.vias[index]); break;
    case Kind::footprint:
      result.footprints[index].pads.push_back(source_.footprints[index].pads[pad_index]);
      break;
    }
  }
  return result;
}

CopperDrcReport ScaffoldCollisionIndex::check_track(const TrackSegment& track) const {
  auto local = nearby(track.start, track.end, track.width.value() / 2.0L);
  local.tracks.push_back(track);
  return check_dynamic_copper_rules(local, {track.id});
}

CopperDrcReport ScaffoldCollisionIndex::check_via(const Via& via) const {
  auto local = nearby(via.position, via.position, std::max(via.diameter.value(), via.drill.value()) / 2.0L);
  local.vias.push_back(via);
  return check_dynamic_copper_rules(local, {via.id});
}

} // namespace flux
