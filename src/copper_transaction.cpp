#include "flux/core/copper_transaction.hpp"

#include "flux/core/geometry.hpp"
#include "flux/core/bezier_copper.hpp"
#include "flux/core/indexed_clearance_map.hpp"
#include "scaffold_connectivity_guard.hpp"

#include <algorithm>
#include <cmath>
#include <map>
#include <numeric>
#include <type_traits>
#include <set>
#include <variant>

namespace flux {
namespace {
using Object = std::variant<TrackSegment, TrackArc, TrackBezier, Via>;
using Objects = std::map<ObjectId, Object>;
NetId net_of(const Object& object) { return std::visit([](const auto& item) { return item.net; }, object); }
void clear_copper(Board& board) {
  board.tracks.clear(); board.track_arcs.clear(); board.track_beziers.clear(); board.vias.clear();
}
Board static_view(Board board) { clear_copper(board); return board; }
Board drc_view(Board board, bool defer) {
  if (defer) for (auto& zone : board.zones) zone.fills.clear();
  return board;
}
bool collect(const Board& board, Objects& objects) {
  bool unique = true;
  const auto add = [&](const auto& items) {
    for (const auto& item : items) unique = objects.emplace(item.id, item).second && unique;
  };
  add(board.tracks); add(board.track_arcs); add(board.track_beziers); add(board.vias);
  return unique;
}
void append(Board& board, const Object& object) {
  std::visit([&](const auto& item) {
    using T = std::decay_t<decltype(item)>;
    if constexpr (std::is_same_v<T, TrackSegment>) board.tracks.push_back(item);
    else if constexpr (std::is_same_v<T, TrackArc>) board.track_arcs.push_back(item);
    else if constexpr (std::is_same_v<T, TrackBezier>) board.track_beziers.push_back(item);
    else board.vias.push_back(item);
  }, object);
}
struct Bounds { Point a, b; long double radius; };
Bounds bounds(const Object& object) {
  return std::visit([](const auto& item) -> Bounds {
    using T = std::decay_t<decltype(item)>;
    if constexpr (std::is_same_v<T, TrackSegment>) {
      return {item.start, item.end, item.width.value() / 2.0L};
    } else if constexpr (std::is_same_v<T, Via>) {
      return {item.position, item.position, std::max(item.diameter.value(), item.drill.value()) / 2.0L};
    } else {
      std::vector<Point> points;
      if constexpr (std::is_same_v<T, TrackArc>) {
        points = geometry::circular_arc_extrema(item.start, item.middle, item.end);
        points.insert(points.end(), {item.start, item.middle, item.end});
      } else {
        points = {item.start, item.control1, item.control2, item.end};
      }
      auto low = item.start, high = item.start;
      for (const auto p : points) {
        low.x = std::min(low.x, p.x); low.y = std::min(low.y, p.y);
        high.x = std::max(high.x, p.x); high.y = std::max(high.y, p.y);
      }
      // Match the exact DRC's widened Bezier proxies, including endpoint rounding. Arc
      // extrema also round to integer coordinates. The broad phase must enclose that view.
      const auto guard = std::is_same_v<T, TrackBezier>
          ? static_cast<long double>(default_bezier_flattening_tolerance.value()) + 2.0L : 2.0L;
      // A rectangle-enclosing capsule is substantially tighter than a start-centred disk.
      // Use the longer axis as its spine and cover the complete orthogonal extent.
      const auto middle_x = Nanometres{std::midpoint(low.x.value(), high.x.value())};
      const auto middle_y = Nanometres{std::midpoint(low.y.value(), high.y.value())};
      const auto width = static_cast<long double>(high.x.value()) - low.x.value();
      const auto height = static_cast<long double>(high.y.value()) - low.y.value();
      if (width >= height) {
        const auto radius = std::max(static_cast<long double>(middle_y.value()) - low.y.value(),
                                     static_cast<long double>(high.y.value()) - middle_y.value());
        return {Point{low.x, middle_y}, Point{high.x, middle_y}, radius + item.width.value() / 2.0L + guard};
      }
      const auto radius = std::max(static_cast<long double>(middle_x.value()) - low.x.value(),
                                   static_cast<long double>(high.x.value()) - middle_x.value());
      return {Point{middle_x, low.y}, Point{middle_x, high.y}, radius + item.width.value() / 2.0L + guard};
    }
  }, object);
}
Board net_view(Board board, const NetId& net) {
  std::erase_if(board.nets, [&](const auto& item) { return item.id != net; });
  std::erase_if(board.tracks, [&](const auto& item) { return item.net != net; });
  std::erase_if(board.track_arcs, [&](const auto& item) { return item.net != net; });
  std::erase_if(board.track_beziers, [&](const auto& item) { return item.net != net; });
  std::erase_if(board.vias, [&](const auto& item) { return item.net != net; });
  std::erase_if(board.zones, [&](const auto& item) { return item.net != net; });
  for (auto& footprint : board.footprints)
    std::erase_if(footprint.pads, [&](const auto& item) { return item.net != net; });
  return board;
}
void merge(ConnectivityReport& into, const ConnectivityReport& from) {
  into.nets.insert(into.nets.end(), from.nets.begin(), from.nets.end());
  into.pad_components.insert(into.pad_components.end(), from.pad_components.begin(), from.pad_components.end());
  into.open_connections += from.open_connections; into.unrouted_nets += from.unrouted_nets;
}
} // namespace

struct CopperTransactionValidator::Impl {
  Board base, context;
  BoardValidationBaseline structural;
  Objects objects;
  std::vector<ObjectId> owners;
  std::shared_ptr<const IndexedClearanceMap> index;
  std::map<NetId, ConnectivityReport> connectivity;
  bool defer, unique;
  long double clearance{0};
  Impl(const Board& board, bool deferred)
      : base(board), context(static_view(board)), structural(make_board_validation_baseline(drc_view(board, deferred))),
        defer(deferred), unique(collect(board, objects)) {
    clearance = std::max({board.rules.default_clearance.value(), board.rules.minimum_clearance.value(),
                         board.rules.hole_clearance.value(), board.rules.hole_to_hole_clearance.value()});
    for (const auto& rules : board.net_classes)
      if (rules.clearance) clearance = std::max(clearance, static_cast<long double>(rules.clearance->value()));
    for (const auto& pair : board.differential_pairs)
      if (pair.minimum_gap) clearance = std::max(clearance, static_cast<long double>(pair.minimum_gap->value()));
    if (!unique || !structural.source_validation().ok()) return;
    std::vector<IndexedClearancePrimitive> primitives;
    for (const auto& [id, object] : objects) {
      const auto box = bounds(object);
      primitives.push_back({box.a.x.value(), box.a.y.value(), box.b.x.value(), box.b.y.value(),
                            static_cast<std::uint32_t>(owners.size()), {}, {}, box.radius});
      owners.push_back(id);
    }
    IndexedClearanceMapConfig config; config.width = config.height = 1U; config.materialize_cells = false;
    const auto prepared = build_indexed_clearance_map(config, std::move(primitives));
    if (prepared.status == IndexedClearanceBuildStatus::ok) index = prepared.map;
  }
};
CopperTransactionValidator::CopperTransactionValidator(const Board& base, bool defer)
    : impl_(std::make_unique<Impl>(base, defer)) {}
CopperTransactionValidator::~CopperTransactionValidator() = default;
CopperTransactionValidator::CopperTransactionValidator(CopperTransactionValidator&&) noexcept = default;
CopperTransactionValidator& CopperTransactionValidator::operator=(CopperTransactionValidator&&) noexcept = default;

CopperTransactionReport CopperTransactionValidator::validate(const Board& candidate) {
  auto& state = *impl_;
  CopperTransactionReport result;
  const auto checked = drc_view(candidate, state.defer);
  result.structural = validate_board_candidate(checked, state.structural).validation;
  Objects proposed;
  const bool unique = collect(candidate, proposed);
  const auto full = [&] {
    result.used_full_fallback = true;
    result.copper = check_copper_rules(checked);
    result.before_connectivity = analyze_connectivity(state.base);
    result.after_connectivity = analyze_connectivity(candidate);
    result.preserves_pad_components = preserves_existing_pad_components(result.before_connectivity,
                                                                        result.after_connectivity);
    result.affected_nets = candidate.nets.size();
  };
  if (!state.unique || !unique || !state.index || !result.structural.ok() ||
      state.context != static_view(candidate)) {
    full(); return result;
  }
  std::set<ObjectId> changed;
  std::set<NetId> affected;
  for (const auto& [id, old] : state.objects) {
    const auto found = proposed.find(id);
    if (found == proposed.end() || found->second != old) {
      changed.insert(id); affected.insert(net_of(old));
    }
  }
  for (const auto& [id, object] : proposed) {
    const auto found = state.objects.find(id);
    if (found == state.objects.end() || found->second != object) {
      changed.insert(id); affected.insert(net_of(object));
    }
  }
  result.changed_objects = changed.size(); result.affected_nets = affected.size();
  std::set<NetId> pair_nets;
  for (const auto& pair : candidate.differential_pairs)
    if (affected.contains(pair.positive_net) || affected.contains(pair.negative_net)) {
      pair_nets.insert(pair.positive_net); pair_nets.insert(pair.negative_net);
    }
  std::set<ObjectId> selected;
  for (const auto& [id, object] : proposed)
    if (changed.contains(id) || pair_nets.contains(net_of(object))) selected.insert(id);
  std::set<ObjectId> neighbours;
  for (const auto& id : selected) {
    const auto box = bounds(proposed.at(id));
    for (const auto primitive : state.index->segment_candidates(
             box.a.x.value(), box.a.y.value(), box.b.x.value(), box.b.y.value(), box.radius, state.clearance)) {
      const auto& owner = state.owners[state.index->primitives()[primitive].owner_id];
      if (!changed.contains(owner) && !selected.contains(owner)) neighbours.insert(owner);
    }
  }
  result.neighbour_objects = neighbours.size();
  Board local = drc_view(state.context, state.defer);
  for (const auto& id : neighbours) append(local, state.objects.at(id));
  for (const auto& id : selected) append(local, proposed.at(id));
  result.copper = check_dynamic_copper_rules(local, {selected.begin(), selected.end()});
  for (const auto& net : affected) {
    auto before = state.connectivity.find(net);
    if (before == state.connectivity.end())
      before = state.connectivity.emplace(net, analyze_connectivity(net_view(state.base, net))).first;
    else ++result.connectivity_cache_hits;
    merge(result.before_connectivity, before->second);
    merge(result.after_connectivity, analyze_connectivity(net_view(candidate, net)));
  }
  result.preserves_pad_components = preserves_existing_pad_components(result.before_connectivity,
                                                                      result.after_connectivity);
  return result;
}
} // namespace flux
