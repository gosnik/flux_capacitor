#include "flux/core/copper_scaffold.hpp"

#include "flux/core/connectivity.hpp"
#include "flux/core/drc.hpp"
#include "flux/core/copper_geometry.hpp"
#include "flux/core/geometry.hpp"
#include "flux/core/indexed_clearance_map.hpp"
#include "flux/core/routing_kernel.hpp"
#include "flux/core/routing_rules.hpp"
#include "flux/core/pns_router.hpp"
#include "flux/core/validation.hpp"
#include "scaffold_collision.hpp"
#include "scaffold_access.hpp"
#include "scaffold_via_history.hpp"
#include "scaffold_joint_displacement.hpp"
#include "scaffold_connectivity_guard.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <deque>
#include <limits>
#include <map>
#include <queue>
#include <set>
#include <sstream>
#include <tuple>

namespace flux {
namespace {

struct Node {
  std::int64_t x{0}, y{0};
  std::size_t layer{0U};
  auto operator<=>(const Node&) const = default;
};
// A node alone is not a sufficient search state: different legal source attachments and
// approach directions can have different assembled-copper feasibility at the same goal.
struct SearchState {
  Node node;
  std::size_t source{0U};
  std::size_t arrival{0U};
  std::size_t via_history{0U};
  auto operator<=>(const SearchState&) const = default;
};
struct QueueEntry { std::uint64_t f; std::uint64_t g; SearchState state; };
struct QueueOrder { bool operator()(const QueueEntry& a, const QueueEntry& b) const {
  if (a.f != b.f) return a.f > b.f;
  // Advance toward a goal on equal-cost plateaus instead of expanding every attachment/history
  // at the shallowest depth first. State ordering keeps the result deterministic.
  if (a.g != b.g) return a.g < b.g;
  return a.state > b.state; } };
struct Attachment { Node node; GlobalRoutePoint anchor; std::uint64_t cost{0U}; };


[[nodiscard]] std::string id_for(std::size_t request, std::size_t ordinal, const char* kind) {
  return "__flux-copper-scaffold-" + std::to_string(request) + "-" + kind + "-" +
         std::to_string(ordinal);
}
[[nodiscard]] bool has_id(const Board& board, const std::string& id) {
  const auto equal = [&id](const auto& value) { return value.id.value() == id; };
  return std::any_of(board.tracks.begin(), board.tracks.end(), equal) ||
         std::any_of(board.vias.begin(), board.vias.end(), equal);
}
[[nodiscard]] Point point_for(const Node& node, Point origin, Nanometres pitch) {
  return {Nanometres{origin.x.value() + node.x * pitch.value()},
          Nanometres{origin.y.value() + node.y * pitch.value()}};
}
[[nodiscard]] std::uint64_t heuristic(const Node& node, const Node& target,
                                      const CopperScaffoldConfig& config) {
  const auto dx = static_cast<std::uint64_t>(std::llabs(node.x - target.x));
  const auto dy = static_cast<std::uint64_t>(std::llabs(node.y - target.y));
  const auto diagonal = std::min(dx, dy);
  const auto straight = std::max(dx, dy) - diagonal;
  return diagonal * config.diagonal_step_cost + straight * config.planar_step_cost +
         (node.layer == target.layer ? 0U : config.via_cost);
}
[[nodiscard]] bool valid_widths(const GlobalRoutePath& request) {
  return (request.segment_widths.empty() || request.segment_widths.size() + 1U == request.points.size()) &&
         (request.segment_width_kinds.empty() ||
          request.segment_width_kinds.size() == request.segment_widths.size());
}
void clear_coarse_metadata(GlobalRoutePath& path) {
  path.medial_resource_sequence.clear(); path.medial_transition_portals.clear();
  path.medial_transition_grid_edges.clear(); path.medial_topology_breaks = 0U;
  path.medial_direct_portal_transitions = 0U; path.medial_unmapped_nodes = 0U;
  path.medial_interior_unmapped_nodes = 0U; path.direct_medial_edges = 0U;
  path.direct_medial_grid_steps_bypassed = 0U; path.geometric_corridors.clear();
}
void compress_collinear_grid_runs(GlobalRoutePath& path) {
  if (path.points.size() < 5U) return;
  std::vector<GlobalRoutePoint> compact;
  compact.reserve(path.points.size());
  compact.push_back(path.points.front());
  for (std::size_t i = 1U; i + 1U < path.points.size(); ++i) {
    const auto& previous = compact.back();
    const auto& current = path.points[i];
    const auto& next = path.points[i + 1U];
    // Preserve exact terminal attachment segments and any layer transition verbatim.
    if (i == 1U || i + 2U == path.points.size() || previous.layer != current.layer ||
        current.layer != next.layer) { compact.push_back(current); continue; }
    const long double ax = static_cast<long double>(current.position.x.value() - previous.position.x.value());
    const long double ay = static_cast<long double>(current.position.y.value() - previous.position.y.value());
    const long double bx = static_cast<long double>(next.position.x.value() - current.position.x.value());
    const long double by = static_cast<long double>(next.position.y.value() - current.position.y.value());
    if (ax * by != ay * bx) compact.push_back(current);
  }
  compact.push_back(path.points.back());
  path.points = std::move(compact);
}

[[nodiscard]] bool widths_are_authorized(const Board& board, const GlobalRoutePath& path) {
  if (!valid_widths(path)) return false;
  if (path.segment_widths.empty())
    return path.terminal_neck_down_segments == 0U && path.medial_neck_down_segments == 0U &&
           path.corridor_neck_down_segments == 0U;
  const auto rules = resolve_net_routing_rules(board, path.net);
  std::size_t terminal = 0U, medial = 0U, corridor = 0U;
  for (std::size_t i = 0U; i < path.segment_widths.size(); ++i) {
    const auto width = path.segment_widths[i];
    if (width.value() <= 0) continue;
    if (width < board.rules.minimum_track_width) return false;
    if (width < rules.track_width) {
      if (path.segment_width_kinds.empty()) return false;
      switch (path.segment_width_kinds[i]) {
      case GlobalRouteSegmentWidthKind::terminal_neck_down: ++terminal; break;
      case GlobalRouteSegmentWidthKind::medial_neck_down: ++medial; break;
      case GlobalRouteSegmentWidthKind::corridor_neck_down: ++corridor; break;
      case GlobalRouteSegmentWidthKind::nominal: return false;
      }
    }
  }
  return terminal == path.terminal_neck_down_segments && medial == path.medial_neck_down_segments &&
         corridor == path.corridor_neck_down_segments;
}
[[nodiscard]] bool spatially_legal(const CopperDrcReport& report) {
  return std::all_of(report.violations.begin(), report.violations.end(), [](const auto& violation) {
    return violation.kind == CopperViolationKind::differential_pair_skew;
  });
}

struct Trial { GlobalRoutePath path; Board board; std::vector<ObjectId> dynamic_ids; };

[[nodiscard]] bool all_copper_legal(const Board& board) {
  return spatially_legal(check_copper_rules(board));
}

// PNS replaces a straight generated segment with a connected set of dogleg tracks.  Keep this
// deliberately narrow: a transaction is rejected unless we can rebuild a canonical path from
// every moved generated segment.  Later retained reconstruction therefore never has to infer
// ownership from an opaque PNS flag.
[[nodiscard]] std::optional<GlobalRoutePath> reconstruct_pns_straight_path(
    const Board& board, const GlobalRoutePath& original, const std::vector<ObjectId>& old_ids) {
  if (original.points.size() < 2U ||
      std::any_of(original.segment_width_kinds.begin(), original.segment_width_kinds.end(),
                  [](const auto kind) { return kind != GlobalRouteSegmentWidthKind::nominal; }))
    return std::nullopt;
  const auto layer = original.points.front().layer;
  if (std::any_of(original.points.begin(), original.points.end(), [&](const auto& point) {
        return point.layer != layer;
      })) return std::nullopt; // PNS has no canonical multi-layer path serializer yet.
  std::vector<const TrackSegment*> pieces;
  for (const auto& track : board.tracks) {
    const auto belongs = std::any_of(old_ids.begin(), old_ids.end(), [&](const auto& id) {
      return track.id == id || track.id.value().starts_with(id.value() + "-pns-");
    });
    if (belongs && track.net == original.net && track.layer == layer) pieces.push_back(&track);
  }
  if (pieces.empty()) return std::nullopt;
  GlobalRoutePath rebuilt = original;
  rebuilt.points.clear(); rebuilt.segment_widths.clear(); rebuilt.segment_width_kinds.clear();
  rebuilt.points.push_back(original.points.front());
  Point cursor = original.points.front().position;
  while (!pieces.empty()) {
    const auto found = std::find_if(pieces.begin(), pieces.end(), [&](const auto* track) {
      return track->start == cursor || track->end == cursor;
    });
    if (found == pieces.end()) return std::nullopt;
    const auto* track = *found;
    cursor = track->start == cursor ? track->end : track->start;
    rebuilt.points.push_back({cursor, layer});
    rebuilt.segment_widths.push_back(track->width);
    rebuilt.segment_width_kinds.push_back(GlobalRouteSegmentWidthKind::nominal);
    pieces.erase(found);
  }
  if (cursor != original.points.back().position) return std::nullopt;
  clear_coarse_metadata(rebuilt);
  return rebuilt;
}
[[nodiscard]] bool reusable_transition(const Board& board, const NetId& net, Point position,
                                       LayerId from, LayerId to) {
  const auto spans = [&](const std::vector<LayerId>& layers) {
    return std::find(layers.begin(), layers.end(), from) != layers.end() &&
           std::find(layers.begin(), layers.end(), to) != layers.end();
  };
  for (const auto& via : board.vias)
    if (via.net == net && via.position == position && spans(via.layers)) return true;
  for (const auto& footprint : board.footprints) for (const auto& pad : footprint.pads)
    if (pad.net == net && pad.plated && pad_has_drill(pad) && spans(pad.layers) &&
        pad_drill_world_position(footprint, pad) == position) return true;
  return false;
}
[[nodiscard]] bool append_route(Board& board, const GlobalRoutePath& path, std::size_t request_index,
                                 std::vector<ObjectId>& ids) {
  if (path.points.size() < 2U || !valid_widths(path)) return false;
  const auto rules = resolve_net_routing_rules(board, path.net);
  for (std::size_t i = 1U, ordinal = 0U; i < path.points.size(); ++i) {
    const auto& a = path.points[i - 1U]; const auto& b = path.points[i];
    if (a.position == b.position && a.layer != b.layer) {
      if (reusable_transition(board, path.net, a.position, a.layer, b.layer)) continue;
      std::string text = id_for(request_index, ordinal++, "via"); while (has_id(board, text)) text += "_";
      const ObjectId id{text}; ids.push_back(id);
      std::vector<LayerId> span;
      for (const auto& layer : board.layers)
        if (layer.kind == LayerKind::copper) span.push_back(layer.id);
      board.vias.push_back({id, path.net, a.position, rules.via_diameter, rules.via_drill,
                            std::move(span), false});
    } else if (a.layer == b.layer && a.position != b.position) {
      const auto width = path.segment_widths.empty() || path.segment_widths[i - 1U].value() <= 0
          ? rules.track_width : path.segment_widths[i - 1U];
      std::string text = id_for(request_index, ordinal++, "track"); while (has_id(board, text)) text += "_";
      const ObjectId id{text}; ids.push_back(id);
      board.tracks.push_back({id, path.net, a.layer, a.position, b.position, width, false});
    } else return false;
  }
  return true;
}
[[nodiscard]] bool legal_trial(const Board& base, const GlobalRoutePath& path, std::size_t index,
                               Trial& trial) {
  trial = {path, base, {}};
  if (!widths_are_authorized(base, path) || !append_route(trial.board, path, index, trial.dynamic_ids)) return false;
  const auto report = check_dynamic_copper_rules(trial.board, trial.dynamic_ids);
  // Pair skew is a whole-net measurement. A scaffold may intentionally retain a partial
  // connection subset, so spatial legality stays hard here and the complete-route final gate
  // remains responsible for skew.
  return spatially_legal(report);
}
[[nodiscard]] std::vector<LayerId> copper_layers(const Board& board) {
  std::vector<LayerId> result;
  for (const auto& layer : board.layers) if (layer.kind == LayerKind::copper) result.push_back(layer.id);
  return result;
}
[[nodiscard]] bool retained_matches_request(const Board& board, const GlobalRoutePath& retained,
                                            const GlobalRoutePath& request) {
  if (retained.net != request.net || retained.connection_index != request.connection_index ||
      retained.points.size() < 2U || request.points.size() < 2U || !valid_widths(retained)) return false;
  // Retained geometry may attach through an existing same-net component rather than terminate at
  // the current request's exact anchors. Its dynamic DRC reconstruction and the component proof
  // below are authoritative; endpoint-coordinate equality would reject valid tree attachments.
  static_cast<void>(board);
  return true;
}
[[nodiscard]] std::string canonical_request_identity(const GlobalRoutePath& request) {
  std::ostringstream identity;
  identity << request.net.value().size() << ':' << request.net.value() << ':'
           << request.connection_index << ':' << request.points.size();
  for (const auto& point : request.points)
    identity << ':' << point.position.x.value() << ',' << point.position.y.value() << ','
             << point.layer.value().size() << ':' << point.layer.value();
  for (const auto width : request.segment_widths) identity << ":w" << width.value();
  for (const auto kind : request.segment_width_kinds) identity << ":k" << static_cast<int>(kind);
  identity << ":neck=" << request.terminal_neck_down_segments << ','
           << request.medial_neck_down_segments << ',' << request.corridor_neck_down_segments;
  return identity.str();
}
[[nodiscard]] bool board_limits(const Board& board, Point& minimum, Point& maximum) {
  if (board.outline.vertices.empty()) return false;
  minimum = maximum = board.outline.vertices.front();
  for (const auto point : board.outline.vertices) {
    minimum.x = std::min(minimum.x, point.x); minimum.y = std::min(minimum.y, point.y);
    maximum.x = std::max(maximum.x, point.x); maximum.y = std::max(maximum.y, point.y);
  }
  return true;
}
[[nodiscard]] std::vector<IndexedClearancePrimitive> board_domain_primitives(const Board& board) {
  std::vector<IndexedClearancePrimitive> result;
  const auto append = [&](const Polygon& polygon, std::uint32_t owner) {
    if (polygon.vertices.size() < 3U) return;
    for (std::size_t i = 0U; i < polygon.vertices.size(); ++i) {
      const auto a = polygon.vertices[i]; const auto b = polygon.vertices[(i + 1U) % polygon.vertices.size()];
      result.push_back({a.x.value(), a.y.value(), b.x.value(), b.y.value(), owner, std::nullopt,
                        std::nullopt, 0.0L});
    }
  };
  append(board.outline, 0U);
  for (std::size_t i = 0U; i < board.cutouts.size(); ++i) append(board.cutouts[i], static_cast<std::uint32_t>(i + 1U));
  return result;
}

void record_blocker_evidence(CopperScaffoldRequestEvidence& evidence, const Board& board,
                             const std::set<std::string>& blockers, std::size_t maximum) {
  evidence.blocker_objects.clear();
  evidence.blocker_nets.clear();
  for (const auto& id : blockers) {
    if (evidence.blocker_objects.size() >= maximum) break;
    evidence.blocker_objects.emplace_back(id);
    const auto append_net = [&](const NetId& net) {
      if (std::find(evidence.blocker_nets.begin(), evidence.blocker_nets.end(), net) ==
          evidence.blocker_nets.end())
        evidence.blocker_nets.push_back(net);
    };
    // The collision index reports canonical object IDs for every copper primitive, not just
    // straight tracks and vias. Preserve ownership for curved copper and pads too; an unknown
    // stale ID remains evidence of a blocked search but never invents an owner.
    for (const auto& track : board.tracks)
      if (track.id.value() == id) append_net(track.net);
    for (const auto& arc : board.track_arcs)
      if (arc.id.value() == id) append_net(arc.net);
    for (const auto& curve : board.track_beziers)
      if (curve.id.value() == id) append_net(curve.net);
    for (const auto& via : board.vias)
      if (via.id.value() == id) append_net(via.net);
    for (const auto& footprint : board.footprints)
      for (const auto& pad : footprint.pads)
        if (pad.id.value() == id) append_net(pad.net);
  }
}

void count_exchange_rollback(CopperScaffoldCounters& counters,
                             CopperScaffoldExchangeRollbackReason reason) {
  ++counters.exchange_rollbacks;
  switch (reason) {
  case CopperScaffoldExchangeRollbackReason::incoming_unroutable:
    ++counters.exchange_rollbacks_incoming_unroutable; break;
  case CopperScaffoldExchangeRollbackReason::victim_unroutable:
    ++counters.exchange_rollbacks_victim_unroutable; break;
  case CopperScaffoldExchangeRollbackReason::connectivity_not_preserved:
    ++counters.exchange_rollbacks_connectivity_not_preserved; break;
  case CopperScaffoldExchangeRollbackReason::cancelled:
    ++counters.exchange_rollbacks_cancelled; break;
  }
}

struct PassageSearch {
  const Board* original{nullptr};
  std::size_t maximum_candidates{0U};
  std::vector<GlobalRoutePath> candidates;
};

[[nodiscard]] std::optional<Trial> search_route(const Board& base, const GlobalRoutePath& request,
                                                  std::size_t index, const CopperScaffoldConfig& config,
                                                  CopperScaffoldCounters& counters,
                                                  std::stop_token token,
                                                  std::set<std::string>* frontier_blockers = nullptr,
                                                  CopperScaffoldSearchTermination* termination = nullptr,
                                                  CopperScaffoldRequestEvidence* access_evidence = nullptr,
                                                  PassageSearch* passage_search = nullptr) {
  const auto finish = [&](CopperScaffoldSearchTermination value) {
    if (termination != nullptr) *termination = value;
  };
  if (request.points.size() < 2U || !valid_widths(request) || config.grid_pitch.value() <= 0 ||
      config.planar_step_cost == 0U || config.diagonal_step_cost == 0U || config.via_cost == 0U)
    { finish(CopperScaffoldSearchTermination::invalid_request); return std::nullopt; }
  if (token.stop_requested()) {
    counters.cancelled = true;
    finish(CopperScaffoldSearchTermination::cancelled); return std::nullopt;
  }
  if (config.maximum_expansions_per_connection == 0U) {
    finish(CopperScaffoldSearchTermination::expansion_budget_exhausted); return std::nullopt;
  }
  const auto layers = copper_layers(base); if (layers.empty()) {
    finish(CopperScaffoldSearchTermination::no_copper_layers); return std::nullopt;
  }
  const auto access = scaffold_connection_access(base, request, config.maximum_component_access_points);
  if (access_evidence != nullptr) {
    access_evidence->access_evaluated = true;
    access_evidence->source_endpoint_contained_by_copper = access.source_endpoint_contained_by_copper;
    access_evidence->target_endpoint_contained_by_copper = access.target_endpoint_contained_by_copper;
    access_evidence->source_component_anchors = access.starts.size();
    access_evidence->target_component_anchors = access.goals.size();
  }
  if (access.already_connected) {
    GlobalRoutePath identity; identity.net = request.net; identity.connection_index = request.connection_index;
    finish(CopperScaffoldSearchTermination::already_connected);
    return Trial{std::move(identity), base, {}};
  }
  const auto find_layer = [&layers](LayerId id) -> std::optional<std::size_t> {
    const auto it = std::find(layers.begin(), layers.end(), id);
    if (it == layers.end()) return std::nullopt;
    return static_cast<std::size_t>(it - layers.begin());
  };
  Point minimum, maximum; if (!board_limits(base, minimum, maximum)) {
    finish(CopperScaffoldSearchTermination::missing_board_limits); return std::nullopt;
  }
  const auto pitch = config.grid_pitch.value();
  const auto floor_grid = [pitch](std::int64_t value, std::int64_t origin) {
    return static_cast<std::int64_t>(std::floor((static_cast<long double>(value) - origin) / pitch));
  };
  const auto xmax = floor_grid(maximum.x.value(), minimum.x.value());
  const auto ymax = floor_grid(maximum.y.value(), minimum.y.value());
  if (xmax < 0 || ymax < 0 || static_cast<std::uint64_t>(xmax + 1) *
      static_cast<std::uint64_t>(ymax + 1) > config.maximum_cells) {
    finish(CopperScaffoldSearchTermination::cell_budget_exceeded); return std::nullopt;
  }
  IndexedClearanceMapConfig domain_config;
  domain_config.origin_x_nm = minimum.x.value(); domain_config.origin_y_nm = minimum.y.value();
  domain_config.pitch_nm = pitch; domain_config.width = static_cast<std::uint32_t>(xmax + 1);
  domain_config.height = static_cast<std::uint32_t>(ymax + 1); domain_config.maximum_cells = config.maximum_cells;
  domain_config.materialize_cells = false;
  domain_config.cancellation_requested = [&token] { return token.stop_requested(); };
  const auto domain = build_indexed_clearance_map(domain_config, board_domain_primitives(base));
  if (domain.status != IndexedClearanceBuildStatus::ok || !domain.map) {
    finish(token.stop_requested() ? CopperScaffoldSearchTermination::cancelled :
                                   CopperScaffoldSearchTermination::domain_build_failed);
    return std::nullopt;
  }
  const ScaffoldCollisionIndex collision(base);
  std::optional<ScaffoldCollisionIndex> original_collision;
  if (passage_search) original_collision.emplace(*passage_search->original);
  std::string soft_probe_id = "__flux-joint-soft-probe";
  while (has_id(passage_search ? *passage_search->original : base, soft_probe_id)) soft_probe_id += "_";
  std::map<std::pair<Node, Node>, std::uint64_t> soft_penalties;
  const auto soft_penalty = [&](Node a, Node b) -> std::uint64_t {
    if (!passage_search || a.layer != b.layer) return 0U;
    if (b < a) std::swap(a, b);
    const auto key = std::pair{a, b};
    if (const auto found = soft_penalties.find(key); found != soft_penalties.end()) return found->second;
    const auto rules = resolve_net_routing_rules(base, request.net);
    const TrackSegment probe{ObjectId{soft_probe_id}, request.net, layers[a.layer],
        point_for(a, minimum, config.grid_pitch), point_for(b, minimum, config.grid_pitch), rules.track_width, false};
    // The hard-scene check remains mandatory. This cost only ranks movement through the
    // explicitly selected generated owners; it cannot soften pads, boundaries or source copper.
    const auto report = original_collision->check_track(probe);
    long double cost = 0.0L;
    for (const auto& v : report.violations) if (v.kind != CopperViolationKind::differential_pair_skew)
      cost += 1.0L + std::max(0.0L, static_cast<long double>(v.required_clearance.value()) - v.actual_distance_nm) / pitch;
    const auto penalty = static_cast<std::uint64_t>(std::min(1'000'000.0L, std::ceil(cost * config.planar_step_cost)));
    soft_penalties.emplace(key, penalty);
    return penalty;
  };
  const auto rules = resolve_net_routing_rules(base, request.net);
  const auto edge_is_legal = [&](Point a, LayerId al, Point b, LayerId bl, std::size_t serial) {
    std::string id_text = "__flux-copper-scaffold-probe-" + std::to_string(index) + "-" +
                          std::to_string(serial);
    while (has_id(base, id_text)) id_text += "_";
    const ObjectId id{std::move(id_text)};
    if (al == bl) {
      TrackSegment track{id, request.net, al, a, b, rules.track_width, false};
      if (collision.ready()) {
        const auto report = collision.check_track(track);
        if (!report.ok() && frontier_blockers != nullptr) for (const auto& violation : report.violations)
          for (const auto& object : violation.object_ids)
            if (object != id.value()) frontier_blockers->insert(object);
        return spatially_legal(report);
      }
      GlobalRoutePath edge; edge.net = request.net; edge.points = {{a, al}, {b, bl}};
      Trial fallback; return legal_trial(base, edge, index, fallback);
    }
    if (reusable_transition(base, request.net, a, al, bl)) return true;
    std::vector<LayerId> span;
    for (const auto& layer : base.layers) if (layer.kind == LayerKind::copper) span.push_back(layer.id);
    Via via{id, request.net, a, rules.via_diameter, rules.via_drill, std::move(span), false};
    if (collision.ready()) {
      const auto report = collision.check_via(via);
      if (!report.ok() && frontier_blockers != nullptr) for (const auto& violation : report.violations)
        for (const auto& object : violation.object_ids)
          if (object != id.value()) frontier_blockers->insert(object);
      return spatially_legal(report);
    }
    GlobalRoutePath edge; edge.net = request.net; edge.points = {{a, al}, {b, bl}};
    Trial fallback; return legal_trial(base, edge, index, fallback);
  };
  const auto attachment_cost = [&](Point a, Point b) {
    const auto dx = static_cast<long double>(a.x.value()) - b.x.value();
    const auto dy = static_cast<long double>(a.y.value()) - b.y.value();
    return static_cast<std::uint64_t>(std::ceil(std::hypotl(dx, dy) / pitch * config.planar_step_cost));
  };
  // Iterate rings before anchors, so the bounded set covers different copper owners/layers
  // before spending the allowance on distant attachments of its first owner.
  const auto attachments = [&](const std::vector<GlobalRoutePoint>& anchors) {
    std::vector<Attachment> result;
    const auto radius = static_cast<std::int64_t>(std::min<std::size_t>(config.attachment_radius_cells, 16U));
    for (std::int64_t ring = 0; ring <= radius; ++ring) for (const auto& endpoint : anchors) {
      const auto layer = find_layer(endpoint.layer); if (!layer) continue;
      const auto x = floor_grid(endpoint.position.x.value(), minimum.x.value());
      const auto y = floor_grid(endpoint.position.y.value(), minimum.y.value());
      for (std::int64_t dy = -ring; dy <= ring; ++dy) for (std::int64_t dx = -ring; dx <= ring; ++dx) {
        routing_kernel_checkpoint();
        if (token.stop_requested()) { counters.cancelled = true; return result; }
        if (std::max(std::abs(dx), std::abs(dy)) != ring) continue;
        if (result.size() >= config.maximum_attachment_states) return result;
        const Node node{x + dx, y + dy, *layer};
        if (node.x < 0 || node.y < 0 || node.x > xmax || node.y > ymax) continue;
        const auto point = point_for(node, minimum, config.grid_pitch);
        if (!domain.map->exact_query(point.x.value(), point.y.value()).inside) continue;
        if (point != endpoint.position &&
            (!geometry::segment_inside_board_area(endpoint.position, point, base) ||
             !edge_is_legal(endpoint.position, endpoint.layer, point, endpoint.layer, result.size())))
          continue;
        if (std::none_of(result.begin(), result.end(), [&](const auto& old) {
              return old.node == node && old.anchor == endpoint;
            })) result.push_back({node, endpoint, attachment_cost(point, endpoint.position)});
      }
    }
    return result;
  };
  const auto starts = attachments(access.starts);
  const auto goals = attachments(access.goals);
  counters.source_attachments += starts.size(); counters.target_attachments += goals.size();
  if (access_evidence != nullptr) {
    access_evidence->source_legal_attachments = starts.size();
    access_evidence->target_legal_attachments = goals.size();
  }
  if (counters.cancelled || starts.empty() || goals.empty()) {
    finish(counters.cancelled ? CopperScaffoldSearchTermination::cancelled :
                               CopperScaffoldSearchTermination::attachment_exhausted);
    return std::nullopt;
  }
  std::map<Node, std::vector<std::size_t>> goals_at;
  for (std::size_t i = 0U; i < goals.size(); ++i) goals_at[goals[i].node].push_back(i);
  std::map<Node, std::uint64_t> estimates;
  const auto estimate = [&](const Node& node) {
    if (const auto found = estimates.find(node); found != estimates.end()) return found->second;
    auto best = std::numeric_limits<std::uint64_t>::max();
    for (const auto& goal : goals) best = std::min(best, heuristic(node, goal.node, config) + goal.cost);
    estimates.emplace(node, best);
    return best;
  };
  // Every label searches the same immutable scene: reuse exact edge decisions across source
  // attachments and approach directions without weakening the final assembled-copper check.
  std::map<std::pair<Node, Node>, bool> edge_cache;
  const auto grid_edge_is_legal = [&](Node a, Node b) {
    if (b < a) std::swap(a, b);
    const auto key = std::pair{a, b};
    if (const auto found = edge_cache.find(key); found != edge_cache.end()) return found->second;
    const auto legal = edge_is_legal(point_for(a, minimum, config.grid_pitch), layers[a.layer],
                                     point_for(b, minimum, config.grid_pitch), layers[b.layer],
                                     edge_cache.size());
    edge_cache.emplace(key, legal);
    return legal;
  };
  const std::array<std::pair<int, int>, 8> offsets{{{-1,-1},{0,-1},{1,-1},{-1,0},{1,0},{-1,1},{0,1},{1,1}}};
  // First try a compact optimistic search keyed only by node.  This deliberately discards
  // source/arrival/via-history labels to avoid multiplying the frontier on large boards; every
  // candidate still goes through the exact legality and connectivity gates below.  The richer
  // search remains the fallback for cases where provenance matters.
  const auto compact_limit = std::min(config.compact_search_expansions,
      config.maximum_expansions_per_connection / 2U);
  std::size_t compact_used = 0U, goal_attempts = 0U;
  const auto compact_goal_limit = config.maximum_goal_attempts / 2U;
  if (!passage_search && compact_limit > 0U && compact_goal_limit > 0U) {
    ++counters.compact_search_attempts;
    std::priority_queue<QueueEntry, std::vector<QueueEntry>, QueueOrder> compact_open;
    std::map<Node, std::uint64_t> compact_distance;
    std::map<Node, std::pair<Node, std::size_t>> compact_previous;
    std::map<Node, std::size_t> compact_source;
    for (std::size_t i = 0U; i < starts.size(); ++i) {
      const auto old = compact_distance.find(starts[i].node);
      if (old == compact_distance.end() || starts[i].cost < old->second) {
        compact_distance[starts[i].node] = starts[i].cost;
        compact_source[starts[i].node] = i;
        compact_open.push({starts[i].cost + estimate(starts[i].node), starts[i].cost,
                           {starts[i].node, i, std::numeric_limits<std::size_t>::max(), 0U}});
      }
    }
    while (!compact_open.empty() && compact_used < compact_limit &&
           goal_attempts < compact_goal_limit) {
      routing_kernel_checkpoint();
      if (token.stop_requested()) {
        counters.cancelled = true; finish(CopperScaffoldSearchTermination::cancelled); return std::nullopt;
      }
      const auto current = compact_open.top(); compact_open.pop();
      const auto found = compact_distance.find(current.state.node);
      if (found == compact_distance.end() || found->second != current.g) continue;
      ++compact_used; ++counters.expansions; ++counters.compact_search_expansions;
      const auto goal = goals_at.find(current.state.node);
      if (goal != goals_at.end()) for (const auto goal_id : goal->second) {
        if (goal_attempts >= compact_goal_limit) break;
        routing_kernel_checkpoint();
        if (token.stop_requested()) {
        counters.cancelled = true; finish(CopperScaffoldSearchTermination::cancelled); return std::nullopt;
      }
        ++goal_attempts;
        std::vector<Node> chain{current.state.node};
        for (auto node = current.state.node; compact_previous.contains(node); ) {
          node = compact_previous.at(node).first; chain.push_back(node);
        }
        std::reverse(chain.begin(), chain.end());
        GlobalRoutePath path; path.net = request.net; path.connection_index = request.connection_index;
        path.points.push_back(starts[compact_source.at(chain.front())].anchor);
        for (const auto& node : chain) {
          const GlobalRoutePoint point{point_for(node, minimum, config.grid_pitch), layers[node.layer]};
          if (path.points.back() != point) path.points.push_back(point);
        }
        if (path.points.back() != goals[goal_id].anchor) path.points.push_back(goals[goal_id].anchor);
        clear_coarse_metadata(path); compress_collinear_grid_runs(path);
        path.segment_widths.assign(path.points.size() - 1U, rules.track_width);
        path.segment_width_kinds.assign(path.points.size() - 1U, GlobalRouteSegmentWidthKind::nominal);
        Trial trial;
        if (legal_trial(base, path, index, trial) && scaffold_request_connected(trial.board, request)) {
          ++counters.compact_search_successes;
          finish(CopperScaffoldSearchTermination::search_succeeded); return trial;
        }
        ++counters.goal_rejections; ++counters.candidate_rejections;
      }
      const auto point = point_for(current.state.node, minimum, config.grid_pitch);
      const auto enqueue_compact = [&](Node next, std::uint64_t step) {
        const auto candidate = current.g + step;
        const auto old = compact_distance.find(next);
        if (old == compact_distance.end() || candidate < old->second) {
          compact_distance[next] = candidate; compact_source[next] = compact_source[current.state.node];
          compact_previous[next] = {current.state.node, compact_source[next]};
          compact_open.push({candidate + estimate(next), candidate,
                             {next, compact_source[next], 0U, 0U}});
        }
      };
      for (const auto& [dx, dy] : offsets) {
        Node next{current.state.node.x + dx, current.state.node.y + dy, current.state.node.layer};
        if (next.x < 0 || next.y < 0 || next.x > xmax || next.y > ymax) continue;
        const auto next_point = point_for(next, minimum, config.grid_pitch);
        if (!domain.map->exact_query(next_point.x.value(), next_point.y.value()).inside ||
            !geometry::segment_inside_board_area(point, next_point, base) ||
            !grid_edge_is_legal(current.state.node, next)) continue;
        enqueue_compact(next, (dx && dy ? config.diagonal_step_cost : config.planar_step_cost) +
            soft_penalty(current.state.node, next));
      }
      if (passage_search) continue;
      for (std::size_t layer = 0U; layer < layers.size(); ++layer) if (layer != current.state.node.layer) {
        Node next{current.state.node.x, current.state.node.y, layer};
        if (grid_edge_is_legal(current.state.node, next)) enqueue_compact(next, config.via_cost);
      }
    }
    ++counters.compact_search_fallbacks;
  }
  std::priority_queue<QueueEntry, std::vector<QueueEntry>, QueueOrder> open;
  std::map<SearchState, std::uint64_t> distance;
  std::map<SearchState, SearchState> previous;
  for (std::size_t i = 0U; i < starts.size(); ++i) {
    const SearchState state{starts[i].node, i, std::numeric_limits<std::size_t>::max()};
    distance[state] = starts[i].cost;
    open.push({estimate(state.node) + starts[i].cost, starts[i].cost, state});
  }
  // Only drill-pair rules couple same-net via choices across otherwise legal edges. Keep an
  // exact canonical set of new drilled positions when that rule applies; static clearance and
  // all final copper checks remain authoritative in either mode.
  const bool needs_via_history = base.rules.hole_to_hole_clearance.value() > 0;
  const auto maximum_key = static_cast<std::uint64_t>(xmax + 1) *
                           static_cast<std::uint64_t>(ymax + 1) - 1U;
  ScaffoldViaHistory via_histories(maximum_key, config.maximum_via_history_nodes);
  bool history_budget_reported = false;
  const auto next_via_history = [&](std::size_t history, Node at) -> std::optional<std::size_t> {
    if (!needs_via_history) return 0U;
    const auto key = static_cast<std::uint64_t>(at.y) * static_cast<std::uint64_t>(xmax + 1) +
                     static_cast<std::uint64_t>(at.x);
    const auto result = via_histories.insert(history, key);
    counters.peak_via_history_nodes = std::max(counters.peak_via_history_nodes, via_histories.peak_node_count());
    if (!result && !history_budget_reported) {
      ++counters.via_history_budget_exhaustions;
      history_budget_reported = true;
    }
    return result;
  };
  std::size_t local_expansions = 0U;
  std::deque<std::pair<QueueEntry, std::size_t>> deferred_goals;
  std::set<std::pair<SearchState, std::size_t>> rejected_goals;
  const auto assemble_goal = [&](const QueueEntry& current, std::size_t goal_id) -> std::optional<Trial> {
    routing_kernel_checkpoint();
    if (token.stop_requested()) { counters.cancelled = true; return std::nullopt; }
    if (!rejected_goals.emplace(current.state, goal_id).second) return std::nullopt;
    ++goal_attempts;
    std::vector<Node> chain{current.state.node};
    for (auto state = current.state; previous.contains(state); state = previous.at(state))
      chain.push_back(previous.at(state).node);
    std::reverse(chain.begin(), chain.end());
    GlobalRoutePath path; path.net = request.net; path.connection_index = request.connection_index;
    path.points.reserve(chain.size() + 2U);
    path.points.push_back(starts[current.state.source].anchor);
    for (const auto& node : chain) {
      const GlobalRoutePoint point{point_for(node, minimum, config.grid_pitch), layers[node.layer]};
      if (path.points.back() != point) path.points.push_back(point);
    }
    if (path.points.back() != goals[goal_id].anchor) path.points.push_back(goals[goal_id].anchor);
    clear_coarse_metadata(path); compress_collinear_grid_runs(path);
    path.segment_widths.assign(path.points.size() - 1U, rules.track_width);
    path.segment_width_kinds.assign(path.points.size() - 1U, GlobalRouteSegmentWidthKind::nominal);
    Trial trial;
    if (legal_trial(base, path, index, trial) && scaffold_request_connected(trial.board, request)) {
      if (!passage_search) return trial;
      if (std::none_of(passage_search->candidates.begin(), passage_search->candidates.end(),
                       [&](const auto& prior) { return prior.points == trial.path.points; }))
        passage_search->candidates.push_back(trial.path);
      if (passage_search->candidates.size() >= passage_search->maximum_candidates) return trial;
      return std::nullopt;
    }
    ++counters.goal_rejections; ++counters.candidate_rejections;
    return std::nullopt;
  };
  const auto rich_budget = config.maximum_expansions_per_connection - compact_used;
  while (!open.empty() && local_expansions < rich_budget &&
         goal_attempts < config.maximum_goal_attempts) {
    routing_kernel_checkpoint();
    if (token.stop_requested()) { counters.cancelled = true; return std::nullopt; }
    const auto current = open.top(); open.pop();
    if (distance.at(current.state) != current.g) continue;
    ++counters.expansions; ++local_expansions;
    if (const auto at = goals_at.find(current.state.node); at != goals_at.end()) {
      // Give other source/approach/history labels a turn before spending the attempt allowance
      // on every attachment at this one node. Remaining anchors are retained for a final pass.
      if (auto trial = assemble_goal(current, at->second.front())) {
        finish(CopperScaffoldSearchTermination::search_succeeded); return trial;
      }
      for (std::size_t i = 1U; i < at->second.size() &&
                               deferred_goals.size() < config.maximum_goal_attempts; ++i)
        deferred_goals.emplace_back(current, at->second[i]);
    }
    const auto point = point_for(current.state.node, minimum, config.grid_pitch);
    const auto enqueue = [&](Node next, std::size_t arrival, std::uint64_t step, std::size_t history) {
      const SearchState state{next, current.state.source, arrival, history};
      const auto candidate = current.g + step;
      const auto old = distance.find(state);
      if (old == distance.end() || candidate < old->second) {
        distance[state] = candidate; previous[state] = current.state;
        open.push({candidate + estimate(next), candidate, state});
      }
    };
    for (std::size_t direction = 0U; direction < offsets.size(); ++direction) {
      const auto [dx, dy] = offsets[direction];
      Node next{current.state.node.x + dx, current.state.node.y + dy, current.state.node.layer};
      if (next.x < 0 || next.y < 0 || next.x > xmax || next.y > ymax) continue;
      const auto next_point = point_for(next, minimum, config.grid_pitch);
      if (!domain.map->exact_query(next_point.x.value(), next_point.y.value()).inside ||
          !geometry::segment_inside_board_area(point, next_point, base)) continue;
      if (!grid_edge_is_legal(current.state.node, next)) { ++counters.candidate_rejections; continue; }
      enqueue(next, direction, (dx && dy ? config.diagonal_step_cost : config.planar_step_cost) +
                  soft_penalty(current.state.node, next),
              current.state.via_history);
    }
    for (std::size_t layer = 0U; !passage_search && layer < layers.size(); ++layer) if (layer != current.state.node.layer) {
      Node next{current.state.node.x, current.state.node.y, layer};
      if (!grid_edge_is_legal(current.state.node, next)) { ++counters.candidate_rejections; continue; }
      auto history = std::optional<std::size_t>{current.state.via_history};
      if (!reusable_transition(base, request.net, point, layers[current.state.node.layer], layers[layer]))
        history = next_via_history(current.state.via_history, current.state.node);
      if (history) enqueue(next, offsets.size() + current.state.node.layer, config.via_cost, *history);
    }
  }
  while (!deferred_goals.empty() && goal_attempts < config.maximum_goal_attempts) {
    const auto candidate = deferred_goals.front(); deferred_goals.pop_front();
    if (auto trial = assemble_goal(candidate.first, candidate.second)) {
      finish(CopperScaffoldSearchTermination::search_succeeded); return trial;
    }
    if (counters.cancelled) break;
  }
  if (counters.cancelled) finish(CopperScaffoldSearchTermination::cancelled);
  else if (goal_attempts >= config.maximum_goal_attempts)
    finish(CopperScaffoldSearchTermination::goal_attempt_budget_exhausted);
  else if (local_expansions >= rich_budget)
    finish(CopperScaffoldSearchTermination::expansion_budget_exhausted);
  else finish(CopperScaffoldSearchTermination::search_exhausted);
  return std::nullopt;
}
} // namespace

CopperScaffoldResult construct_copper_scaffold(const Board& board,
                                               const std::vector<GlobalRoutePath>& requests,
                                               const CopperScaffoldConfig& config,
                                               std::stop_token stop_token,
                                               const CopperScaffoldResult* retained,
                                               const std::function<void(const CopperScaffoldResult&)>& publish) {
  CopperScaffoldResult result; result.copper = board; result.requests = requests;
  result.request_count = requests.size();
  result.request_identities.reserve(requests.size());
  result.request_evidence.reserve(requests.size());
  for (const auto& request : requests) {
    result.request_identities.push_back(canonical_request_identity(request));
    result.request_evidence.push_back({result.request_identities.back(), request.net,
                                       request.connection_index,
                                       CopperScaffoldSearchTermination::not_attempted, {}, {}});
  }
  std::vector<bool> complete(requests.size(), false);
  std::vector<std::vector<ObjectId>> request_dynamic_ids(requests.size());
  // Dynamic vias can be reused by a later same-net connection.  Their owner must describe every
  // published path that currently traverses the transition, rather than the transaction that
  // originally created it.  Otherwise a later causal exchange only finds a stale creator and
  // cannot select the actual consumer for rip-up.
  const auto refresh_dynamic_via_consumers = [&] {
    std::set<std::string> dynamic_vias;
    for (const auto& owned : request_dynamic_ids) for (const auto& id : owned)
      if (std::any_of(result.copper.vias.begin(), result.copper.vias.end(),
                      [&](const auto& via) { return via.id == id; }))
        dynamic_vias.insert(id.value());
    if (dynamic_vias.empty()) return;
    for (auto& owned : request_dynamic_ids)
      std::erase_if(owned, [&](const auto& id) { return dynamic_vias.contains(id.value()); });
    for (const auto& path : result.paths) {
      const auto request = std::find_if(requests.begin(), requests.end(), [&](const auto& candidate) {
        return candidate.net == path.net && candidate.connection_index == path.connection_index;
      });
      if (request == requests.end()) continue;
      const auto owner = static_cast<std::size_t>(request - requests.begin());
      for (std::size_t point = 1U; point < path.points.size(); ++point) {
        const auto& first = path.points[point - 1U];
        const auto& second = path.points[point];
        if (first.position != second.position || first.layer == second.layer) continue;
        for (const auto& via : result.copper.vias) {
          const auto spans = std::find(via.layers.begin(), via.layers.end(), first.layer) != via.layers.end() &&
              std::find(via.layers.begin(), via.layers.end(), second.layer) != via.layers.end();
          if (via.net == path.net && via.position == first.position && spans &&
              dynamic_vias.contains(via.id.value()) &&
              std::find(request_dynamic_ids[owner].begin(), request_dynamic_ids[owner].end(), via.id) ==
                  request_dynamic_ids[owner].end())
            request_dynamic_ids[owner].push_back(via.id);
        }
      }
    }
  };
  const auto account_requests = [&](CopperScaffoldResult& target) {
    target.request_count = requests.size();
    target.missing_indices.clear();
    target.satisfied_indices.clear();
    for (std::size_t i = 0U; i < complete.size(); ++i) {
      if (complete[i]) target.satisfied_indices.push_back(i);
      else target.missing_indices.push_back(i);
    }
    target.counters.routed_connections = target.satisfied_indices.size();
    target.counters.published_tracks = target.copper.tracks.size() - board.tracks.size();
    target.counters.published_vias = target.copper.vias.size() - board.vias.size();
  };
  if (!config.enabled) {
    account_requests(result);
    result.diagnostic = "copper scaffold disabled";
    return result;
  }
  const auto publish_legal_subset = [&] {
    if (!publish) return;
    CopperScaffoldResult snapshot = result;
    snapshot.complete = false;
    account_requests(snapshot);
    snapshot.diagnostic = "legal connection checkpoint retained; scaffold search in progress";
    publish(snapshot);
  };
  std::vector<bool> duplicate_request(requests.size(), false);
  std::set<std::string> request_keys;
  for (std::size_t index = 0U; index < requests.size(); ++index) {
    const auto key = requests[index].net.value() + "\x1f" + std::to_string(requests[index].connection_index);
    if (!request_keys.insert(key).second) {
      duplicate_request[index] = true;
      for (std::size_t prior = 0U; prior < index; ++prior)
        if (requests[prior].net == requests[index].net &&
            requests[prior].connection_index == requests[index].connection_index)
          duplicate_request[prior] = true;
    }
  }
  // Retained copper is only accepted where it can be reconstructed and dynamically revalidated.
  if (retained != nullptr) for (const auto& prior : retained->paths) {
    if (stop_token.stop_requested()) {
      result.counters.cancelled = true;
      break;
    }
    const auto found = std::find_if(requests.begin(), requests.end(), [&](const auto& request) {
      return request.net == prior.net && request.connection_index == prior.connection_index; });
    if (found == requests.end() || duplicate_request[static_cast<std::size_t>(found - requests.begin())] ||
        !retained_matches_request(board, prior, *found)) continue;
    const auto index = static_cast<std::size_t>(found - requests.begin()); Trial trial;
    if (!complete[index] && legal_trial(result.copper, prior, index, trial) &&
        scaffold_request_connected(trial.board, *found)) {
      request_dynamic_ids[index] = trial.dynamic_ids;
      result.copper = std::move(trial.board); result.paths.push_back(prior); complete[index] = true;
      refresh_dynamic_via_consumers();
      ++result.counters.completed_transactions;
      publish_legal_subset();
    }
  }
  for (std::size_t index = 0U; index < requests.size(); ++index) {
    if (complete[index] || duplicate_request[index]) {
      if (duplicate_request[index])
        result.request_evidence[index].termination = CopperScaffoldSearchTermination::invalid_request;
      continue;
    }
    ++result.counters.searched_connections;
    if (stop_token.stop_requested()) {
      result.counters.cancelled = true;
      result.request_evidence[index].termination = CopperScaffoldSearchTermination::cancelled;
      break;
    }
    if (requests[index].points.size() < 2U || !valid_widths(requests[index])) {
      result.request_evidence[index].termination = CopperScaffoldSearchTermination::invalid_request;
      continue;
    }
    if (scaffold_request_connected(result.copper, requests[index])) {
      complete[index] = true;
      result.request_evidence[index].termination = CopperScaffoldSearchTermination::already_connected;
      ++result.counters.component_connections;
      ++result.counters.completed_transactions;
      publish_legal_subset();
      continue;
    }
    // A supplied route may already be the best continuous/topological answer. Preserve it when
    // it is physically legal in the currently retained scene; full-domain grid search is only a
    // recovery mechanism, never a reason to replace lawful geometry with a staircase.
    Trial direct;
    if (legal_trial(result.copper, requests[index], index, direct) &&
        scaffold_request_connected(direct.board, requests[index])) {
      clear_coarse_metadata(direct.path);
      request_dynamic_ids[index] = direct.dynamic_ids;
      result.copper = std::move(direct.board); result.paths.push_back(std::move(direct.path));
      refresh_dynamic_via_consumers();
      complete[index] = true;
      result.request_evidence[index].termination = CopperScaffoldSearchTermination::direct_route_accepted;
      ++result.counters.completed_transactions;
      publish_legal_subset();
      continue;
    }
    std::optional<Trial> routed;
    std::set<std::string> frontier_blockers;
    routed = search_route(result.copper, requests[index], index, config, result.counters, stop_token,
                          &frontier_blockers, &result.request_evidence[index].termination,
                          &result.request_evidence[index]);
    if (routed) {
      auto& trial = *routed;
      const auto tracks = static_cast<std::size_t>(std::count_if(
          trial.dynamic_ids.begin(), trial.dynamic_ids.end(), [&](const auto& id) {
            return std::any_of(trial.board.tracks.begin(), trial.board.tracks.end(),
                               [&](const auto& track) { return track.id == id; });
          }));
      if (scaffold_request_connected(trial.board, requests[index])) {
        result.counters.published_tracks += tracks;
        result.counters.published_vias += trial.dynamic_ids.size() - tracks;
        request_dynamic_ids[index] = trial.dynamic_ids;
        result.copper = std::move(trial.board);
        if (!trial.path.points.empty()) result.paths.push_back(std::move(trial.path));
        refresh_dynamic_via_consumers();
        complete[index] = true;
        result.request_evidence[index].termination = CopperScaffoldSearchTermination::search_succeeded;
        ++result.counters.completed_transactions;
        publish_legal_subset();
        continue;
      }
    }
    record_blocker_evidence(result.request_evidence[index], result.copper, frontier_blockers,
                            config.maximum_failure_evidence);
    if (result.counters.cancelled) break;
    // Bounded causal exchange: retain survivor copper, remove one attributed victim, admit the
    // blocked request, then rebuild the victim. Failure leaves the published result untouched.
    std::vector<std::size_t> victim_order;
    constexpr std::size_t maximum_component_victims = 6U;
    const auto extend_causal_victims = [&](const std::set<std::string>& blockers) {
      bool evidence_changed = false;
      for (const auto& object : blockers) {
        for (std::size_t owner = 0U; owner < request_dynamic_ids.size(); ++owner) {
          const auto owns_object = std::any_of(request_dynamic_ids[owner].begin(),
                                               request_dynamic_ids[owner].end(),
                                               [&](const auto& id) { return id.value() == object; });
          if (!owns_object || std::find(victim_order.begin(), victim_order.end(), owner) != victim_order.end())
            continue;
          evidence_changed = frontier_blockers.insert(object).second || evidence_changed;
          if (victim_order.size() < maximum_component_victims) victim_order.push_back(owner);
        }
      }
      if (evidence_changed)
        record_blocker_evidence(result.request_evidence[index], result.copper, frontier_blockers,
                                config.maximum_failure_evidence);
    };
    // Causality comes only from the failed search frontier.  A rejected coarse path is useful
    // evidence, but it must not cause unrelated routes to be displaced after the independent
    // full-domain search found a different obstruction.
    extend_causal_victims(frontier_blockers);

    bool jointly_repaired = false;
    if (config.maximum_joint_passage_candidates > 0U && config.maximum_joint_blocker_owners > 0U &&
        config.maximum_joint_particles > 0U && config.maximum_joint_iterations > 0U &&
        !stop_token.stop_requested()) {
      const auto maximum_owners = std::min<std::size_t>(6U, config.maximum_joint_blocker_owners);
      std::map<std::string, std::size_t> movable_ids;
      std::map<std::size_t, GlobalRoutePath> movable_paths;
      for (std::size_t owner = 0U; owner < requests.size(); ++owner) {
        if (!complete[owner] || requests[owner].net == requests[index].net || request_dynamic_ids[owner].empty()) continue;
        const auto path = std::find_if(result.paths.begin(), result.paths.end(), [&](const auto& p) {
          return p.net == requests[owner].net && p.connection_index == requests[owner].connection_index;
        });
        if (path == result.paths.end() || path->points.size() < 2U || path->points.size() > std::max<std::size_t>(2U, std::min<std::size_t>(config.maximum_joint_particles, 4096U)) ||
            std::any_of(path->points.begin(), path->points.end(), [&](const auto& p) { return p.layer != path->points.front().layer; }) ||
            std::any_of(path->segment_width_kinds.begin(), path->segment_width_kinds.end(),
                        [](const auto k) { return k != GlobalRouteSegmentWidthKind::nominal; })) continue;
        bool eligible = true;
        for (const auto& id : request_dynamic_ids[owner]) {
          const auto track = std::find_if(result.copper.tracks.begin(), result.copper.tracks.end(), [&](const auto& t) { return t.id == id; });
          if (track == result.copper.tracks.end() || track->locked || has_id(board, id.value())) { eligible = false; break; }
          for (std::size_t other = 0U; other < requests.size(); ++other)
            if (other != owner && std::find(request_dynamic_ids[other].begin(), request_dynamic_ids[other].end(), id) != request_dynamic_ids[other].end()) eligible = false;
        }
        if (!eligible) continue;
        movable_paths.emplace(owner, *path);
        for (const auto& id : request_dynamic_ids[owner]) movable_ids.emplace(id.value(), owner);
      }
      // The incoming path is the root of the contact graph. Add its contact owners, then
      // frontier owners, then nearby owners that may need to yield displacement space.
      std::vector<std::size_t> pool;
      const auto add_owner = [&](std::size_t owner) {
        if (movable_paths.contains(owner) && pool.size() < 8U && std::find(pool.begin(), pool.end(), owner) == pool.end()) pool.push_back(owner);
      };
      Trial raw{requests[index], result.copper, {}};
      if (append_route(raw.board, raw.path, index, raw.dynamic_ids))
        for (const auto& v : check_dynamic_copper_rules(raw.board, raw.dynamic_ids).violations)
          for (const auto& id : v.object_ids) if (const auto it = movable_ids.find(id); it != movable_ids.end()) add_owner(it->second);
      for (const auto owner : victim_order) add_owner(owner);
      const auto owners_adjacent = [&](std::size_t a, std::size_t b) {
        const auto& pa = movable_paths.at(a); const auto& pb = movable_paths.at(b);
        if (pa.points.front().layer != pb.points.front().layer) return false;
        const auto ra = resolve_net_routing_rules(result.copper, pa.net);
        const auto rb = resolve_net_routing_rules(result.copper, pb.net);
        const auto reach = 2.0L * config.maximum_displacement.value() +
            static_cast<long double>(ra.track_width.value() + rb.track_width.value()) / 2.0L +
            resolve_pairwise_clearance(result.copper, pa.net, pb.net).value();
        for (std::size_t i = 1U; i < pa.points.size(); ++i) for (std::size_t j = 1U; j < pb.points.size(); ++j)
          if (geometry::segment_distance_nm(pa.points[i-1U].position, pa.points[i].position,
                                            pb.points[j-1U].position, pb.points[j].position) <= reach) return true;
        return false;
      };
      for (std::size_t at = 0U; at < pool.size() && pool.size() < 8U; ++at)
        for (const auto& [owner, path] : movable_paths) {
          (void)path;
          if (owners_adjacent(pool[at], owner)) add_owner(owner);
        }
      std::set<std::string> soft_ids;
      for (const auto owner : pool) for (const auto& id : request_dynamic_ids[owner]) soft_ids.insert(id.value());
      Board hard_scene = result.copper;
      std::erase_if(hard_scene.tracks, [&](const auto& t) { return soft_ids.contains(t.id.value()); });
      PassageSearch search{&result.copper, std::min<std::size_t>(8U, config.maximum_joint_passage_candidates), {}};
      const auto retain_proposal = [&](const GlobalRoutePath& proposal) {
        if (search.candidates.size() >= search.maximum_candidates || proposal.points.size() < 2U ||
            std::any_of(proposal.points.begin(), proposal.points.end(), [&](const auto& p) { return p.layer != proposal.points.front().layer; })) return;
        Trial candidate;
        if (legal_trial(hard_scene, proposal, index, candidate) &&
            std::none_of(search.candidates.begin(), search.candidates.end(), [&](const auto& old) { return old.points == proposal.points; }))
          search.candidates.push_back(std::move(candidate.path));
      };
      if (!pool.empty()) {
        retain_proposal(requests[index]);
        if (retained) for (const auto& previous : retained->blocked_passages)
          if (previous.request_index == index && previous.path.net == requests[index].net &&
              previous.path.connection_index == requests[index].connection_index &&
              retained_matches_request(board, previous.path, requests[index])) retain_proposal(previous.path);
        if (search.candidates.size() < search.maximum_candidates && config.maximum_joint_search_expansions > 0U) {
          auto bounded = config; bounded.maximum_expansions_per_connection = config.maximum_joint_search_expansions;
          const auto before = result.counters.expansions;
          (void)search_route(hard_scene, requests[index], index, bounded, result.counters, stop_token,
                             nullptr, nullptr, nullptr, &search);
          const auto work = result.counters.expansions - before;
          result.counters.joint_search_expansions += work;
          if (work >= config.maximum_joint_search_expansions) ++result.counters.joint_budget_exhausted;
        }
      }
      std::vector<CopperScaffoldBlockedPassage> ranked;
      for (const auto& trial : search.candidates) {
        if (stop_token.stop_requested()) break;
        Trial original{trial, result.copper, {}};
        if (!append_route(original.board, original.path, index, original.dynamic_ids)) continue;
        CopperScaffoldBlockedPassage candidate; candidate.request_index = index; candidate.path = trial;
        bool hard_blocked = false;
        for (const auto& v : check_dynamic_copper_rules(original.board, original.dynamic_ids).violations) {
          if (v.kind == CopperViolationKind::differential_pair_skew) continue;
          bool attributed = false;
          for (const auto& id : v.object_ids) if (const auto it = movable_ids.find(id); it != movable_ids.end() && soft_ids.contains(id)) {
            attributed = true;
            if (std::find(candidate.blocker_owners.begin(), candidate.blocker_owners.end(), it->second) == candidate.blocker_owners.end()) candidate.blocker_owners.push_back(it->second);
          }
          if (!attributed) hard_blocked = true;
          candidate.clearance_deficit_nm = std::max(candidate.clearance_deficit_nm,
              std::max(0.0L, static_cast<long double>(v.required_clearance.value()) - v.actual_distance_nm));
        }
        if (hard_blocked || candidate.blocker_owners.empty() || candidate.blocker_owners.size() > maximum_owners) continue;
        // Adjacent contact owners are included before relaxation, rather than left frozen until
        // the proposed displacement collides with them. The root plus this BFS stays connected.
        for (std::size_t at = 0U; at < candidate.blocker_owners.size() && candidate.blocker_owners.size() < maximum_owners; ++at)
          for (const auto owner : pool)
            if (candidate.blocker_owners.size() < maximum_owners &&
                std::find(candidate.blocker_owners.begin(), candidate.blocker_owners.end(), owner) == candidate.blocker_owners.end() &&
                owners_adjacent(candidate.blocker_owners[at], owner)) candidate.blocker_owners.push_back(owner);
        std::sort(candidate.blocker_owners.begin(), candidate.blocker_owners.end());
        Board fixed = result.copper;
        std::set<std::string> group_ids;
        for (const auto owner : candidate.blocker_owners) for (const auto& id : request_dynamic_ids[owner]) group_ids.insert(id.value());
        std::erase_if(fixed.tracks, [&](const auto& t) { return group_ids.contains(t.id.value()); });
        const ScaffoldCollisionIndex fixed_collision(fixed);
        // Optimistic room estimate against immutable copper; group packing is evaluated by
        // the coupled solve and exact gate, never certified by this ranking heuristic.
        candidate.displacement_space_nm = static_cast<long double>(config.maximum_displacement.value());
        for (const auto owner : candidate.blocker_owners) {
          const auto& path = movable_paths.at(owner);
          long double owner_space = 0.0L, length = 0.0L;
          for (std::size_t segment = 1U; segment < path.points.size(); ++segment)
            length += geometry::point_distance_nm(path.points[segment-1U].position, path.points[segment].position);
          const auto a = path.points.front().position; const auto b = path.points.back().position;
          const auto dx = static_cast<long double>(b.x.value()) - a.x.value();
          const auto dy = static_cast<long double>(b.y.value()) - a.y.value();
          const auto norm = std::hypot(dx, dy);
          if (norm > 0.0L) for (const auto sign : {-1.0L, 1.0L}) for (std::size_t step = 1U; step <= 4U; ++step) {
            const auto delta = static_cast<long double>(config.maximum_displacement.value()) * step / 4.0L;
            bool free = true;
            for (const auto& track : result.copper.tracks) if (std::find(request_dynamic_ids[owner].begin(), request_dynamic_ids[owner].end(), track.id) != request_dynamic_ids[owner].end()) {
              auto probe = track;
              const Point offset{Nanometres{static_cast<std::int64_t>(std::llround(-dy / norm * delta * sign))},
                                 Nanometres{static_cast<std::int64_t>(std::llround(dx / norm * delta * sign))}};
              probe.start = probe.start + offset; probe.end = probe.end + offset;
              if (!spatially_legal(fixed_collision.check_track(probe))) { free = false; break; }
            }
            if (free) owner_space = std::max(owner_space, delta);
          }
          candidate.displacement_space_nm = std::min(candidate.displacement_space_nm, owner_space);
          candidate.movement_cost_nm += candidate.clearance_deficit_nm *
              (1.0L + length / std::max(1.0L, static_cast<long double>(config.maximum_displacement.value())));
        }
        ranked.push_back(std::move(candidate));
      }
      std::stable_sort(ranked.begin(), ranked.end(), [&](const auto& a, const auto& b) {
        return std::tuple{std::max(0.0L, a.clearance_deficit_nm - a.displacement_space_nm), a.movement_cost_nm,
                          a.clearance_deficit_nm, a.blocker_owners.size(), canonical_request_identity(a.path)} <
               std::tuple{std::max(0.0L, b.clearance_deficit_nm - b.displacement_space_nm), b.movement_cost_nm,
                          b.clearance_deficit_nm, b.blocker_owners.size(), canonical_request_identity(b.path)};
      });
      result.counters.joint_passage_candidates += ranked.size();
      for (const auto& candidate : ranked) if (result.blocked_passages.size() < config.maximum_failure_evidence)
        result.blocked_passages.push_back(candidate);
      for (const auto& candidate : ranked) {
        if (stop_token.stop_requested()) { result.counters.cancelled = true; break; }
        // The first adapter deliberately handles planar nominal-width passages only.  Preserve
        // the existing specialized neck-down/via transaction rather than synthesising policy
        // provenance while resampling a joint bundle.
        if (std::any_of(candidate.path.segment_width_kinds.begin(),
                        candidate.path.segment_width_kinds.end(), [](const auto kind) {
                          return kind != GlobalRouteSegmentWidthKind::nominal;
                        })) continue;
        std::vector<GlobalRoutePath> bundle;
        std::vector<std::size_t> owners;
        bundle.push_back(candidate.path);
        for (const auto owner : candidate.blocker_owners) {
          const auto found = movable_paths.find(owner);
          if (found == movable_paths.end()) { bundle.clear(); break; }
          bundle.push_back(found->second);
          owners.push_back(owner);
        }
        if (bundle.size() != owners.size() + 1U || bundle.size() > maximum_owners + 1U) continue;
        Board fixed_scene = result.copper;
        std::set<std::string> selected_ids;
        for (const auto owner : owners)
          for (const auto& id : request_dynamic_ids[owner]) selected_ids.insert(id.value());
        std::erase_if(fixed_scene.tracks, [&](const auto& track) {
          return selected_ids.contains(track.id.value());
        });
        ScaffoldJointDisplacementConfig joint_config;
        joint_config.maximum_particles = std::min<std::size_t>(config.maximum_joint_particles, 4096U);
        joint_config.maximum_iterations = config.maximum_joint_iterations;
        joint_config.maximum_contacts = std::max<std::size_t>(1U, joint_config.maximum_particles * 64U);
        joint_config.maximum_movement_nm = config.maximum_displacement.value();
        joint_config.enable_residual_pns = true;
        ++result.counters.joint_group_attempts;
        if (joint_config.maximum_particles < 2U) {
          ++result.counters.joint_budget_exhausted;
          continue;
        }
        const auto displaced_paths = solve_scaffold_joint_displacement(fixed_scene, bundle, joint_config, stop_token);
        if (displaced_paths.cancelled || stop_token.stop_requested()) {
          result.counters.cancelled = true; break;
        }
        if (displaced_paths.xpbd_iterations > 0U) ++result.counters.joint_xpbd_invocations;
        if (displaced_paths.used_avbd) ++result.counters.joint_avbd_invocations;
        if (displaced_paths.budget_exhausted) ++result.counters.joint_budget_exhausted;
        if (!displaced_paths.paths || displaced_paths.paths->size() != bundle.size()) {
          ++result.counters.joint_exact_rejections;
          continue;
        }

        Board transaction = fixed_scene;
        std::vector<GlobalRoutePath> accepted_paths = *displaced_paths.paths;
        std::vector<std::vector<ObjectId>> transaction_ids(bundle.size());
        bool assembled = true;
        for (std::size_t path = 0U; path < accepted_paths.size(); ++path) {
          const auto request_owner = path == 0U ? index : owners[path - 1U];
          assembled = widths_are_authorized(transaction, accepted_paths[path]) &&
              append_route(transaction, accepted_paths[path], request_owner,
                           transaction_ids[path]);
          if (!assembled) break;
        }
        if (!assembled) { ++result.counters.joint_exact_rejections; continue; }

        // A residual contact is allowed to reach the bounded PNS cleanup, but PNS may only
        // change the just-created group.  It is still followed by the same whole-board gate.
        const auto exact_residual = check_copper_rules(transaction);
        if (!spatially_legal(exact_residual)) {
          ++result.counters.joint_pns_attempts;
          const auto belongs_to_group = [&](const std::string& object) {
            return std::any_of(transaction_ids.begin(), transaction_ids.end(), [&](const auto& ids) {
              return std::any_of(ids.begin(), ids.end(), [&](const auto& id) {
                return id.value() == object;
              });
            });
          };
          const auto violation = std::find_if(exact_residual.violations.begin(), exact_residual.violations.end(),
              [&](const auto& value) {
                return value.object_ids.size() >= 2U &&
                    belongs_to_group(value.object_ids.front()) && belongs_to_group(value.object_ids.back());
              });
          if (violation != exact_residual.violations.end()) {
            PnsShoveRequest shove;
            const auto intruder_owner = std::find_if(transaction_ids.begin(), transaction_ids.end(),
                [&](const auto& ids) { return std::any_of(ids.begin(), ids.end(), [&](const auto& id) {
                  return id.value() == violation->object_ids.front();
                }); });
            if (intruder_owner == transaction_ids.end()) continue;
            for (const auto& id : *intruder_owner) shove.intruder_object_ids.push_back(id.value());
            shove.colliding_object_ids.push_back(violation->object_ids.back());
            for (const auto& track : fixed_scene.tracks) shove.protected_object_ids.push_back(track.id.value());
            for (const auto& via : fixed_scene.vias) shove.protected_object_ids.push_back(via.id.value());
            for (std::size_t owner = 0U; owner < request_dynamic_ids.size(); ++owner)
              if (owner != index && std::find(owners.begin(), owners.end(), owner) == owners.end())
                for (const auto& id : request_dynamic_ids[owner]) shove.protected_object_ids.push_back(id.value());
            const auto intruder = std::find_if(transaction.tracks.begin(), transaction.tracks.end(),
                [&](const auto& track) { return track.id.value() == violation->object_ids.front(); });
            const auto blocker = std::find_if(transaction.tracks.begin(), transaction.tracks.end(),
                [&](const auto& track) { return track.id.value() == shove.colliding_object_ids.front(); });
            if (intruder != transaction.tracks.end() && blocker != transaction.tracks.end()) {
              const auto bx = static_cast<long double>(blocker->end.x.value()) - blocker->start.x.value();
              const auto by = static_cast<long double>(blocker->end.y.value()) - blocker->start.y.value();
              const auto length = std::hypotl(bx, by);
              if (!(length > 0.0L)) continue;
              shove.normal_x = -by / length;
              shove.normal_y = bx / length;
              const auto mid_x = (static_cast<long double>(intruder->start.x.value()) + intruder->end.x.value()) / 2.0L;
              const auto mid_y = (static_cast<long double>(intruder->start.y.value()) + intruder->end.y.value()) / 2.0L;
              const auto t = std::clamp(((mid_x - blocker->start.x.value()) * bx +
                                         (mid_y - blocker->start.y.value()) * by) / (length * length), 0.0L, 1.0L);
              shove.collision_point = {Nanometres{static_cast<std::int64_t>(std::llround(blocker->start.x.value() + t * bx))},
                                       Nanometres{static_cast<std::int64_t>(std::llround(blocker->start.y.value() + t * by))}};
              long double low = 0.0L, high = 0.0L;
              const auto intruder_index = static_cast<std::size_t>(intruder_owner - transaction_ids.begin());
              for (const auto& point : accepted_paths[intruder_index].points) {
                const auto projection = (static_cast<long double>(point.position.x.value()) - shove.collision_point.x.value()) * shove.normal_x +
                                        (static_cast<long double>(point.position.y.value()) - shove.collision_point.y.value()) * shove.normal_y;
                low = std::min(low, projection); high = std::max(high, projection);
              }
              const auto envelope = (static_cast<long double>(intruder->width.value()) + blocker->width.value()) / 2.0L +
                  resolve_pairwise_clearance(transaction, intruder->net, blocker->net).value();
              shove.required_displacement = Nanometres{static_cast<std::int64_t>(
                  std::ceil(std::max(1.0L, std::min(high + envelope, envelope - low)))) + 1};
              PnsShoveConfig pns_config;
              pns_config.max_propagation_depth = config.maximum_displacement_depth;
              pns_config.max_candidate_checks = config.maximum_displacement_candidate_checks;
              pns_config.minimum_displacement = config.grid_pitch;
              pns_config.maximum_displacement = config.maximum_displacement;
              if (const auto shoved = pns_push_and_shove(transaction, shove, pns_config); shoved.accepted()) {
                transaction = std::move(*shoved.candidate);
                // PNS replaces straight segments with named doglegs.  Rebuild every affected
                // logical path and its dynamic ownership before considering the atomic commit.
                for (std::size_t path = 0U; path < accepted_paths.size(); ++path) {
                  const auto rebuilt = reconstruct_pns_straight_path(transaction, accepted_paths[path],
                                                                      transaction_ids[path]);
                  if (!rebuilt) { assembled = false; break; }
                  accepted_paths[path] = *rebuilt;
                  std::vector<ObjectId> rebuilt_ids;
                  for (const auto& old : transaction_ids[path]) for (const auto& track : transaction.tracks)
                    if (track.id == old || track.id.value().starts_with(old.value() + "-pns-"))
                      rebuilt_ids.push_back(track.id);
                  std::sort(rebuilt_ids.begin(), rebuilt_ids.end());
                  rebuilt_ids.erase(std::unique(rebuilt_ids.begin(), rebuilt_ids.end()), rebuilt_ids.end());
                  transaction_ids[path] = std::move(rebuilt_ids);
                }
              }
            }
          }
        }
        bool within_movement_envelope = true;
        for (std::size_t path = 0U; path < accepted_paths.size() && within_movement_envelope; ++path)
          for (const auto& point : accepted_paths[path].points) {
            long double nearest = std::numeric_limits<long double>::infinity();
            for (std::size_t edge = 1U; edge < bundle[path].points.size(); ++edge)
              if (point.layer == bundle[path].points[edge].layer)
                nearest = std::min(nearest, geometry::segment_distance_nm(point.position, point.position,
                    bundle[path].points[edge - 1U].position, bundle[path].points[edge].position));
            if (nearest > static_cast<long double>(config.maximum_displacement.value())) {
              within_movement_envelope = false; break;
            }
          }
        assembled = assembled && within_movement_envelope;
        if (!assembled) { ++result.counters.joint_exact_rejections; continue; }
        if (stop_token.stop_requested()) { result.counters.cancelled = true; break; }
        bool connected = scaffold_request_connected(transaction, requests[index]);
        for (std::size_t request = 0U; connected && request < complete.size(); ++request)
          if (complete[request] && request != index)
            connected = scaffold_request_connected(transaction, requests[request]);
        const auto before_connectivity = analyze_connectivity(result.copper);
        const auto after_connectivity = analyze_connectivity(transaction);
        const auto connectivity_nonregressed = std::all_of(
            before_connectivity.nets.begin(), before_connectivity.nets.end(), [&](const auto& before) {
              const auto after = std::find_if(after_connectivity.nets.begin(), after_connectivity.nets.end(),
                  [&](const auto& value) { return value.net == before.net; });
              return after != after_connectivity.nets.end() &&
                     after->open_connections <= before.open_connections &&
                     after->connected_pad_groups <= before.connected_pad_groups;
            });
        if (!connected || !connectivity_nonregressed ||
            !preserves_existing_pad_components(before_connectivity, after_connectivity) ||
            !validate_board(transaction).ok() ||
            !all_copper_legal(transaction)) {
          ++result.counters.joint_exact_rejections;
          continue;
        }
        if (stop_token.stop_requested()) { result.counters.cancelled = true; break; }
        // Exact acceptance is atomic: publish the complete bundle and replace every affected
        // owner map only after all pre-existing request connections remain physical.
        std::erase_if(result.paths, [&](const auto& path) {
          return std::any_of(owners.begin(), owners.end(), [&](const auto owner) {
            return path.net == requests[owner].net && path.connection_index == requests[owner].connection_index;
          });
        });
        for (const auto& path : accepted_paths) result.paths.push_back(path);
        result.copper = std::move(transaction);
        request_dynamic_ids[index] = std::move(transaction_ids.front());
        for (std::size_t owner = 0U; owner < owners.size(); ++owner)
          request_dynamic_ids[owners[owner]] = std::move(transaction_ids[owner + 1U]);
        complete[index] = true;
        for (const auto owner : owners) complete[owner] = true;
        refresh_dynamic_via_consumers();
        ++result.counters.joint_group_accepted;
        ++result.counters.completed_transactions;
        result.request_evidence[index].termination = CopperScaffoldSearchTermination::search_succeeded;
        jointly_repaired = true;
        publish_legal_subset();
        break;
      }
    }
    if (stop_token.stop_requested()) result.counters.cancelled = true;
    if (result.counters.cancelled || jointly_repaired) continue;

    // Before ripping up an owner, probe the passage with only a small attributed owner set
    // omitted.  The probe is never committed.  Its candidate is reinserted into the original
    // copper and PNS must move the blocking copper in that original scene.  This preserves the
    // distinction between discovering a passage and publishing a legal physical deformation.
    bool displaced = false;
    std::size_t displacement_expansions = 0U;
    for (std::size_t attempt = 0U; !displaced && !result.counters.cancelled &&
                                attempt < victim_order.size() &&
                                attempt < config.maximum_displacement_attempts; ++attempt) {
      const auto victim = victim_order[attempt];
      // PNS paths are reconstructed only for planar, generated owners.  Vias and authored
      // copper stay protected; those cases continue into causal reroute below.
      if (result.paths.end() == std::find_if(result.paths.begin(), result.paths.end(), [&](const auto& path) {
            return path.net == requests[victim].net && path.connection_index == requests[victim].connection_index;
          }) || displacement_expansions >= config.maximum_displacement_expansions)
        continue;
      Board omitted = result.copper;
      bool source_or_via = false;
      for (const auto& id : request_dynamic_ids[victim]) {
        source_or_via = source_or_via || std::any_of(board.tracks.begin(), board.tracks.end(),
            [&](const auto& track) { return track.id == id; }) ||
            std::any_of(board.vias.begin(), board.vias.end(), [&](const auto& via) { return via.id == id; }) ||
            std::any_of(result.copper.vias.begin(), result.copper.vias.end(),
                [&](const auto& via) { return via.id == id; });
      }
      if (source_or_via) continue; // source copper and vias are pinned for this planar backend.
      std::set<std::string> omitted_ids;
      for (const auto& id : request_dynamic_ids[victim]) omitted_ids.insert(id.value());
      std::erase_if(omitted.tracks, [&](const auto& track) { return omitted_ids.contains(track.id.value()); });
      ++result.counters.displacement_attempts;
      std::optional<Trial> passage;
      Trial direct_passage;
      if (legal_trial(omitted, requests[index], index, direct_passage) &&
          scaffold_request_connected(direct_passage.board, requests[index])) {
        passage = std::move(direct_passage);
      } else {
        auto bounded = config;
        // This is a separate bounded diagnostic/proposal search; it shares the displacement
        // transaction cap, not the exhausted primary-search allowance.
        bounded.maximum_expansions_per_connection =
            config.maximum_displacement_expansions - displacement_expansions;
        const auto before = result.counters.expansions;
        std::set<std::string> ignored_probe_blockers;
        passage = search_route(omitted, requests[index], index, bounded, result.counters, stop_token,
                               &ignored_probe_blockers);
        const auto used = result.counters.expansions - before;
        displacement_expansions += used;
        result.counters.displacement_search_expansions += used;
      }
      if (!passage) {
        ++result.counters.displacement_search_failures;
        if (displacement_expansions >= config.maximum_displacement_expansions)
          ++result.counters.displacement_budget_exhausted;
        continue;
      }
      Trial intruder{passage->path, result.copper, {}};
      if (!append_route(intruder.board, intruder.path, index, intruder.dynamic_ids)) {
        ++result.counters.displacement_validation_rejections;
        continue;
      }
      std::set<std::string> collision_ids;
      std::optional<std::pair<std::string, std::string>> collision_pair;
      for (const auto& violation : check_dynamic_copper_rules(intruder.board, intruder.dynamic_ids).violations) {
        std::optional<std::string> intruder_id, blocker_id;
        for (const auto& id : violation.object_ids) {
          if (!intruder_id && std::any_of(intruder.dynamic_ids.begin(), intruder.dynamic_ids.end(),
              [&](const auto& dynamic) { return dynamic.value() == id; })) intruder_id = id;
          if (!blocker_id && omitted_ids.contains(id)) blocker_id = id;
        }
        if (intruder_id && blocker_id) {
          collision_ids.insert(*blocker_id);
          if (!collision_pair) collision_pair = std::pair{*intruder_id, *blocker_id};
        }
      }
      if (collision_ids.empty() || !collision_pair) {
        ++result.counters.displacement_validation_rejections; continue;
      }
      PnsShoveRequest shove;
      for (const auto& id : intruder.dynamic_ids) shove.intruder_object_ids.push_back(id.value());
      shove.colliding_object_ids.assign(collision_ids.begin(), collision_ids.end());
      // Original board copper and all non-attributed generated requests are immutable to PNS.
      for (const auto& track : board.tracks) shove.protected_object_ids.push_back(track.id.value());
      for (const auto& via : board.vias) shove.protected_object_ids.push_back(via.id.value());
      for (std::size_t owner = 0U; owner < request_dynamic_ids.size(); ++owner)
        if (owner != victim) for (const auto& id : request_dynamic_ids[owner]) shove.protected_object_ids.push_back(id.value());
      const auto intruder_track = std::find_if(intruder.board.tracks.begin(), intruder.board.tracks.end(),
          [&](const auto& track) { return track.id.value() == collision_pair->first; });
      const auto blocker_track = std::find_if(intruder.board.tracks.begin(), intruder.board.tracks.end(),
          [&](const auto& track) { return track.id.value() == collision_pair->second; });
      if (intruder_track == intruder.board.tracks.end() || blocker_track == intruder.board.tracks.end()) {
        ++result.counters.displacement_validation_rejections; continue;
      }
      const auto midpoint = [](const TrackSegment& track) {
        return Point{Nanometres{(track.start.x.value() + track.end.x.value()) / 2},
                     Nanometres{(track.start.y.value() + track.end.y.value()) / 2}};
      };
      const auto intruder_midpoint = midpoint(*intruder_track);
      const long double dx = static_cast<long double>(blocker_track->end.x.value() - blocker_track->start.x.value());
      const long double dy = static_cast<long double>(blocker_track->end.y.value() - blocker_track->start.y.value());
      const long double length_squared = dx * dx + dy * dy;
      const long double t = length_squared > 0.0L ? std::clamp(
          ((static_cast<long double>(intruder_midpoint.x.value() - blocker_track->start.x.value()) * dx +
            static_cast<long double>(intruder_midpoint.y.value() - blocker_track->start.y.value()) * dy) /
           length_squared), 0.0L, 1.0L) : 0.0L;
      shove.collision_point = Point{Nanometres{static_cast<std::int64_t>(std::llround(
              static_cast<long double>(blocker_track->start.x.value()) + t * dx))},
          Nanometres{static_cast<std::int64_t>(std::llround(
              static_cast<long double>(blocker_track->start.y.value()) + t * dy))}};
      shove.normal_x = static_cast<long double>(intruder_midpoint.x.value() - shove.collision_point.x.value());
      shove.normal_y = static_cast<long double>(intruder_midpoint.y.value() - shove.collision_point.y.value());
      if (!(std::hypot(shove.normal_x, shove.normal_y) > 0.0L)) {
        shove.normal_x = static_cast<long double>(blocker_track->start.y.value() - blocker_track->end.y.value());
        shove.normal_y = static_cast<long double>(blocker_track->end.x.value() - blocker_track->start.x.value());
      }
      // Seed PNS from the actual clearance envelope, not the navigation grid. A grid-based
      // doubling sequence can jump over the only legal displacement in a narrow channel.
      const auto normal_length = std::hypot(shove.normal_x, shove.normal_y);
      const auto project_normal = [&](Point point) {
        return ((static_cast<long double>(point.x.value()) - shove.collision_point.x.value()) * shove.normal_x +
                (static_cast<long double>(point.y.value()) - shove.collision_point.y.value()) * shove.normal_y) /
               normal_length;
      };
      const auto first_projection = project_normal(intruder_track->start);
      const auto last_projection = project_normal(intruder_track->end);
      const auto clearance_envelope =
          (static_cast<long double>(intruder_track->width.value()) + blocker_track->width.value()) / 2.0L +
          resolve_pairwise_clearance(intruder.board, intruder_track->net, blocker_track->net).value();
      const auto required = std::min(std::max(first_projection, last_projection) + clearance_envelope,
                                    clearance_envelope - std::min(first_projection, last_projection));
      shove.required_displacement = Nanometres{static_cast<std::int64_t>(std::ceil(std::max(1.0L, required))) + 1};
      PnsShoveConfig pns_config;
      pns_config.max_propagation_depth = config.maximum_displacement_depth;
      pns_config.max_candidate_checks = config.maximum_displacement_candidate_checks;
      pns_config.minimum_displacement = config.grid_pitch;
      pns_config.maximum_displacement = config.maximum_displacement;
      auto shoved = pns_push_and_shove(intruder.board, shove, pns_config);
      if (shoved.budget_exhausted) ++result.counters.displacement_budget_exhausted;
      if (!shoved.accepted()) { ++result.counters.displacement_shove_rejections; continue; }
      const auto victim_path = std::find_if(result.paths.begin(), result.paths.end(), [&](const auto& path) {
        return path.net == requests[victim].net && path.connection_index == requests[victim].connection_index;
      });
      if (stop_token.stop_requested()) { result.counters.cancelled = true; break; }
      auto rebuilt_victim = reconstruct_pns_straight_path(*shoved.candidate, *victim_path,
                                                           request_dynamic_ids[victim]);
      if (!rebuilt_victim) { ++result.counters.displacement_reconstruction_rejections; continue; }
      bool connections_preserved = scaffold_request_connected(*shoved.candidate, requests[index]) &&
          scaffold_request_connected(*shoved.candidate, requests[victim]) && all_copper_legal(*shoved.candidate);
      for (std::size_t request = 0U; connections_preserved && request < complete.size(); ++request)
        if (complete[request] && request != victim)
          connections_preserved = scaffold_request_connected(*shoved.candidate, requests[request]);
      if (!connections_preserved) { ++result.counters.displacement_validation_rejections; continue; }
      if (stop_token.stop_requested()) { result.counters.cancelled = true; break; }
      std::erase_if(result.paths, [&](const auto& path) {
        return path.net == requests[victim].net && path.connection_index == requests[victim].connection_index;
      });
      result.paths.push_back(std::move(intruder.path));
      result.paths.push_back(std::move(*rebuilt_victim));
      result.copper = std::move(*shoved.candidate);
      request_dynamic_ids[index] = intruder.dynamic_ids;
      // PNS removes original IDs. Reconstruct the current dynamic ownership from the emitted
      // dogleg prefixes before later causal transactions inspect it.
      request_dynamic_ids[victim].clear();
      for (const auto& old : omitted_ids) for (const auto& track : result.copper.tracks)
        if (track.id.value() == old || track.id.value().starts_with(old + "-pns-"))
          request_dynamic_ids[victim].push_back(track.id);
      complete[index] = true; complete[victim] = true;
      refresh_dynamic_via_consumers();
      result.counters.displacement_moved_tracks += shoved.moved_tracks;
      result.counters.displacement_moved_vias += shoved.moved_vias;
      ++result.counters.displacement_accepted;
      ++result.counters.completed_transactions;
      result.request_evidence[index].termination = CopperScaffoldSearchTermination::search_succeeded;
      displaced = true;
      publish_legal_subset();
    }
    if (result.counters.cancelled || displaced) continue;
    bool exchanged = false;
    const auto record_rollback = [&](CopperScaffoldExchangeRollbackReason reason,
                                     const std::vector<std::size_t>& victims) {
      count_exchange_rollback(result.counters, reason);
      if (result.exchange_rollbacks.size() >= config.maximum_failure_evidence) return;
      CopperScaffoldExchangeRollback record;
      record.request_identity = result.request_identities[index];
      record.reason = reason;
      for (const auto victim : victims) record.victim_request_identities.push_back(result.request_identities[victim]);
      record_blocker_evidence(result.request_evidence[index], result.copper, frontier_blockers,
                              config.maximum_failure_evidence);
      record.blocker_objects = result.request_evidence[index].blocker_objects;
      record.blocker_nets = result.request_evidence[index].blocker_nets;
      result.exchange_rollbacks.push_back(std::move(record));
    };
    for (std::size_t attempt = 0U; attempt < victim_order.size() && !exchanged &&
                                attempt < config.maximum_victim_exchanges; ++attempt) {
      const auto victim = victim_order[attempt];
      ++result.counters.victim_exchanges_attempted;
      Board rebuilt = result.copper;
      std::set<std::string> survivor_ids;
      for (std::size_t owner = 0U; owner < request_dynamic_ids.size(); ++owner) {
        if (owner == victim) continue;
        for (const auto& id : request_dynamic_ids[owner]) survivor_ids.insert(id.value());
      }
      // A survivor may traverse a via that was originally created while satisfying the victim
      // request. Reuse does not add that via to the survivor's dynamic-ID list, so retain it
      // explicitly when the survivor's published path still requires the transition.
      for (const auto& path : result.paths) {
        if (path.net == requests[victim].net &&
            path.connection_index == requests[victim].connection_index) continue;
        for (std::size_t point = 1U; point < path.points.size(); ++point) {
          const auto& first = path.points[point - 1U];
          const auto& second = path.points[point];
          if (first.position != second.position || first.layer == second.layer) continue;
          for (const auto& via : result.copper.vias) {
            const auto spans = std::find(via.layers.begin(), via.layers.end(), first.layer) != via.layers.end() &&
                std::find(via.layers.begin(), via.layers.end(), second.layer) != via.layers.end();
            if (via.net == path.net && via.position == first.position && spans)
              survivor_ids.insert(via.id.value());
          }
        }
      }
      std::set<std::string> removable;
      for (const auto& id : request_dynamic_ids[victim])
        if (!survivor_ids.contains(id.value())) removable.insert(id.value());
      std::vector<ObjectId> retained_victim_ids;
      for (const auto& id : request_dynamic_ids[victim])
        if (!removable.contains(id.value())) retained_victim_ids.push_back(id);
      std::erase_if(rebuilt.tracks, [&](const auto& track) { return removable.contains(track.id.value()); });
      std::erase_if(rebuilt.vias, [&](const auto& via) { return removable.contains(via.id.value()); });
      const auto preserved_tracks = static_cast<std::size_t>(std::count_if(
          rebuilt.tracks.begin(), rebuilt.tracks.end(), [&](const auto& track) {
            return survivor_ids.contains(track.id.value());
          }));
      const auto preserved_vias = static_cast<std::size_t>(std::count_if(
          rebuilt.vias.begin(), rebuilt.vias.end(), [&](const auto& via) {
            return survivor_ids.contains(via.id.value());
          }));
      std::set<std::string> exposed_blockers;
      auto incoming = search_route(rebuilt, requests[index], index, config, result.counters, stop_token,
                                   &exposed_blockers);
      if (!incoming) {
        extend_causal_victims(exposed_blockers);
        record_rollback(result.counters.cancelled ? CopperScaffoldExchangeRollbackReason::cancelled :
                                                   CopperScaffoldExchangeRollbackReason::incoming_unroutable,
                        {victim});
        if (result.counters.cancelled) break;
        continue;
      }
      rebuilt = std::move(incoming->board);
      if (!scaffold_request_connected(rebuilt, requests[index])) {
        record_rollback(CopperScaffoldExchangeRollbackReason::incoming_unroutable, {victim});
        continue;
      }
      exposed_blockers.clear();
      auto replacement = search_route(rebuilt, requests[victim], victim, config,
                                      result.counters, stop_token, &exposed_blockers);
      if (!replacement) {
        extend_causal_victims(exposed_blockers);
        record_rollback(result.counters.cancelled ? CopperScaffoldExchangeRollbackReason::cancelled :
                                                   CopperScaffoldExchangeRollbackReason::victim_unroutable,
                        {victim});
        if (result.counters.cancelled) break;
        continue; // Transaction rollback: result is still the untouched subset.
      }
      rebuilt = std::move(replacement->board);
      if (!scaffold_request_connected(rebuilt, requests[victim])) {
        record_rollback(CopperScaffoldExchangeRollbackReason::victim_unroutable, {victim});
        continue;
      }
      auto candidate_complete = complete;
      candidate_complete[index] = true;
      candidate_complete[victim] = true;
      bool preserved_connectivity = true;
      for (std::size_t request = 0U; request < candidate_complete.size(); ++request)
        if (candidate_complete[request] &&
            !scaffold_request_connected(rebuilt, requests[request])) {
          preserved_connectivity = false;
          break;
        }
      if (!preserved_connectivity) {
        record_rollback(CopperScaffoldExchangeRollbackReason::connectivity_not_preserved, {victim});
        continue;
      }
      std::erase_if(result.paths, [&](const auto& path) {
        return path.net == requests[victim].net &&
               path.connection_index == requests[victim].connection_index;
      });
      request_dynamic_ids[index] = incoming->dynamic_ids;
      request_dynamic_ids[victim] = replacement->dynamic_ids;
      // The survivor path can still depend on a victim-created reused via. Keep that ID owned
      // by the replacement transaction as well as protected by the survivor-path scan, so a
      // later causal exchange can still attribute and preserve/remove it correctly.
      for (const auto& id : retained_victim_ids) {
        const auto prior = std::find_if(request_dynamic_ids[victim].begin(),
                                        request_dynamic_ids[victim].end(),
                                        [&](const auto& owned) { return owned == id; });
        const auto source = std::find_if(result.copper.vias.begin(), result.copper.vias.end(),
                                         [&](const auto& via) { return via.id == id; });
        if (prior == request_dynamic_ids[victim].end() && source != result.copper.vias.end())
          request_dynamic_ids[victim].emplace_back(id);
      }
      if (!incoming->path.points.empty()) result.paths.push_back(std::move(incoming->path));
      if (!replacement->path.points.empty()) result.paths.push_back(std::move(replacement->path));
      result.copper = std::move(rebuilt);
      refresh_dynamic_via_consumers();
      complete = std::move(candidate_complete);
      result.counters.survivor_tracks_preserved += preserved_tracks;
      result.counters.survivor_vias_preserved += preserved_vias;
      ++result.counters.completed_transactions;
      ++result.counters.victim_exchanges_accepted; exchanged = true;
      publish_legal_subset();
    }
    // Components are considered only after every permitted singleton exchange failed. Groups
    // come solely from the failed-search frontier, and retain all non-group IDs verbatim.
    // Cardinality, member selection, and reconstruction order are deterministic: increasing
    // group size, lexicographic victim subsets, then lexicographic request-index permutations.
    std::size_t coupled_attempts = 0U;
    std::size_t coupled_expansions = 0U;
    bool coupled_budget_exhausted = false;
    bool coupled_budget_limited = false;
    std::size_t maximum_component_size = std::min({config.maximum_coupled_victims, victim_order.size(),
                                                   maximum_component_victims});
    const auto attempt_component = [&](const std::vector<std::size_t>& victims) {
        bool attempt_budget_limited = false;
        ++coupled_attempts;
        ++result.counters.victim_exchanges_attempted;
        ++result.counters.coupled_victim_exchanges_attempted;
        result.counters.coupled_component_victims_attempted += victims.size();
        // Reserve a deterministic share of the remaining common cap for larger causal component
        // sizes, then divide this component's share across the incoming demand and every
        // displaced victim. A difficult early pair therefore cannot consume the whole request
        // budget before a larger causal component receives an attempt.
        const auto remaining_shared = config.maximum_coupled_expansions - coupled_expansions;
        // The first ordering of every larger causal component is scheduled before secondary
        // orderings. Reserve only for those actual future component sizes, rather than every
        // configured attempt slot: a frontier containing only one pair may use the full shared
        // cap, while a three-victim candidate still receives a protected first attempt.
        const auto larger_components = maximum_component_size > victims.size()
            ? maximum_component_size - victims.size() : 0U;
        const auto component_cap = (remaining_shared + larger_components) / (larger_components + 1U);
        std::size_t component_expansions = 0U;
        std::size_t remaining_demands = victims.size() + 1U;
        Board rebuilt = result.copper;
        std::set<std::string> exposed_blockers;
        std::set<std::string> survivor_ids;
        for (std::size_t owner = 0U; owner < request_dynamic_ids.size(); ++owner) {
          if (std::find(victims.begin(), victims.end(), owner) != victims.end()) continue;
          for (const auto& id : request_dynamic_ids[owner]) survivor_ids.insert(id.value());
        }
        // Shared vias can be referenced by a survivor even though only a removed request first
        // created them. Preserve them by following every surviving published transition.
        for (const auto& path : result.paths) {
          const auto belongs_to_victim = std::any_of(victims.begin(), victims.end(), [&](const auto victim) {
            return path.net == requests[victim].net && path.connection_index == requests[victim].connection_index;
          });
          if (belongs_to_victim)
            continue;
          for (std::size_t point = 1U; point < path.points.size(); ++point) {
            const auto& a = path.points[point - 1U]; const auto& b = path.points[point];
            if (a.position != b.position || a.layer == b.layer) continue;
            for (const auto& via : result.copper.vias) {
              const auto spans = std::find(via.layers.begin(), via.layers.end(), a.layer) != via.layers.end() &&
                  std::find(via.layers.begin(), via.layers.end(), b.layer) != via.layers.end();
              if (via.net == path.net && via.position == a.position && spans) survivor_ids.insert(via.id.value());
            }
          }
        }
        std::set<std::string> removable;
        for (const auto victim : victims) for (const auto& id : request_dynamic_ids[victim])
          if (!survivor_ids.contains(id.value())) removable.insert(id.value());
        std::erase_if(rebuilt.tracks, [&](const auto& track) { return removable.contains(track.id.value()); });
        std::erase_if(rebuilt.vias, [&](const auto& via) { return removable.contains(via.id.value()); });
        const auto preserved_tracks = static_cast<std::size_t>(std::count_if(
            rebuilt.tracks.begin(), rebuilt.tracks.end(), [&](const auto& track) {
              return survivor_ids.contains(track.id.value());
            }));
        const auto preserved_vias = static_cast<std::size_t>(std::count_if(
            rebuilt.vias.begin(), rebuilt.vias.end(), [&](const auto& via) {
              return survivor_ids.contains(via.id.value());
            }));
        const auto coupled_search = [&](const Board& scene, const GlobalRoutePath& demand,
                                        std::size_t demand_index) {
          if (coupled_expansions >= config.maximum_coupled_expansions) {
            attempt_budget_limited = true; coupled_budget_limited = true;
            coupled_budget_exhausted = true;
            return std::optional<Trial>{};
          }
          if (component_expansions >= component_cap) {
            attempt_budget_limited = true;
            coupled_budget_limited = true;
            return std::optional<Trial>{};
          }
          auto limited = config;
          const auto component_remaining = component_cap - component_expansions;
          const auto fair_demand_cap = std::max<std::size_t>(
              1U, (component_remaining + remaining_demands - 1U) / remaining_demands);
          limited.maximum_expansions_per_connection = std::min(
              {config.maximum_expansions_per_connection, config.maximum_coupled_expansions - coupled_expansions,
               fair_demand_cap});
          const auto before = result.counters.expansions;
          CopperScaffoldSearchTermination termination = CopperScaffoldSearchTermination::not_attempted;
          auto trial = search_route(scene, demand, demand_index, limited, result.counters, stop_token,
                                    &exposed_blockers, &termination);
          const auto consumed = result.counters.expansions - before;
          coupled_expansions += consumed;
          component_expansions += consumed;
          --remaining_demands;
          if (termination == CopperScaffoldSearchTermination::expansion_budget_exhausted) {
            attempt_budget_limited = true;
            coupled_budget_limited = true;
          }
          if (coupled_expansions >= config.maximum_coupled_expansions)
            coupled_budget_exhausted = true;
          return trial;
        };
        auto incoming = coupled_search(rebuilt, requests[index], index);
        if (!incoming || !scaffold_request_connected(incoming ? incoming->board : rebuilt, requests[index])) {
          extend_causal_victims(exposed_blockers);
          maximum_component_size = std::min({config.maximum_coupled_victims, victim_order.size(),
                                             maximum_component_victims});
          if (!attempt_budget_limited)
            record_rollback(result.counters.cancelled ? CopperScaffoldExchangeRollbackReason::cancelled :
                                                       CopperScaffoldExchangeRollbackReason::incoming_unroutable,
                            victims);
          return;
        }
        rebuilt = std::move(incoming->board);
        std::vector<Trial> replacements;
        replacements.reserve(victims.size());
        bool rebuilt_victims = true;
        for (const auto victim : victims) {
          if (coupled_expansions >= config.maximum_coupled_expansions) {
            attempt_budget_limited = true; coupled_budget_limited = true;
            coupled_budget_exhausted = true; rebuilt_victims = false; break;
          }
          auto replacement = coupled_search(rebuilt, requests[victim], victim);
          if (!replacement || !scaffold_request_connected(replacement ? replacement->board : rebuilt, requests[victim])) {
            extend_causal_victims(exposed_blockers);
            maximum_component_size = std::min({config.maximum_coupled_victims, victim_order.size(),
                                               maximum_component_victims});
            rebuilt_victims = false; break;
          }
          rebuilt = std::move(replacement->board); replacements.push_back(std::move(*replacement));
        }
        if (!rebuilt_victims) {
          if (!attempt_budget_limited)
            record_rollback(result.counters.cancelled ? CopperScaffoldExchangeRollbackReason::cancelled :
                                                       CopperScaffoldExchangeRollbackReason::victim_unroutable,
                            victims);
          return;
        }
        auto candidate_complete = complete;
        candidate_complete[index] = true;
        for (const auto victim : victims) candidate_complete[victim] = true;
        bool preserved = true;
        for (std::size_t request = 0U; request < candidate_complete.size(); ++request)
          if (candidate_complete[request] && !scaffold_request_connected(rebuilt, requests[request])) {
            preserved = false; break;
        }
        if (!preserved) {
          record_rollback(CopperScaffoldExchangeRollbackReason::connectivity_not_preserved, victims);
          return;
        }
        std::erase_if(result.paths, [&](const auto& path) {
          return std::any_of(victims.begin(), victims.end(), [&](const auto victim) {
            return path.net == requests[victim].net && path.connection_index == requests[victim].connection_index;
          });
        });
        // Keep the dynamic identity alive until ownership is reconstructed from the replacement
        // scene below.  The previous victim might have created a via now used only by a survivor.
        std::vector<ObjectId> preserved_dynamic_vias;
        for (const auto victim : victims) for (const auto& id : request_dynamic_ids[victim])
          if (!removable.contains(id.value()) &&
              std::any_of(rebuilt.vias.begin(), rebuilt.vias.end(),
                          [&](const auto& via) { return via.id == id; }) &&
              std::find(preserved_dynamic_vias.begin(), preserved_dynamic_vias.end(), id) ==
                  preserved_dynamic_vias.end())
            preserved_dynamic_vias.push_back(id);
        request_dynamic_ids[index] = incoming->dynamic_ids;
        for (std::size_t group = 0U; group < victims.size(); ++group)
          request_dynamic_ids[victims[group]] = replacements[group].dynamic_ids;
        // `refresh_dynamic_via_consumers` removes this temporary owner and associates each ID
        // with the path(s) that actually traverse the retained transition.
        request_dynamic_ids[victims.front()].insert(request_dynamic_ids[victims.front()].end(),
                                                     preserved_dynamic_vias.begin(),
                                                     preserved_dynamic_vias.end());
        if (!incoming->path.points.empty()) result.paths.push_back(std::move(incoming->path));
        for (auto& replacement : replacements)
          if (!replacement.path.points.empty()) result.paths.push_back(std::move(replacement.path));
        result.copper = std::move(rebuilt); complete = std::move(candidate_complete);
        refresh_dynamic_via_consumers();
        result.counters.survivor_tracks_preserved += preserved_tracks;
        result.counters.survivor_vias_preserved += preserved_vias;
        ++result.counters.completed_transactions;
        ++result.counters.victim_exchanges_accepted;
        ++result.counters.coupled_victim_exchanges_accepted;
        result.counters.coupled_component_victims_accepted += victims.size();
        exchanged = true;
        publish_legal_subset();
    };
    std::set<std::vector<std::size_t>> attempted_component_orders;
    const auto attempt_unique_component = [&](const std::vector<std::size_t>& victims) {
      if (attempted_component_orders.emplace(victims).second) attempt_component(victims);
    };
    const auto enumerate_components = [&](auto&& self, std::size_t cardinality, std::size_t next,
                                          std::vector<std::size_t>& selected) -> void {
      if (exchanged || result.counters.cancelled || coupled_budget_exhausted ||
          coupled_attempts >= config.maximum_coupled_exchange_attempts) return;
      if (selected.size() == cardinality) {
        auto ordering = selected;
        std::sort(ordering.begin(), ordering.end());
        do {
          attempt_unique_component(ordering);
          if (exchanged || result.counters.cancelled || coupled_budget_exhausted ||
              coupled_attempts >= config.maximum_coupled_exchange_attempts) break;
        } while (std::next_permutation(ordering.begin(), ordering.end()));
        return;
      }
      for (std::size_t candidate = next; candidate < victim_order.size(); ++candidate) {
        selected.push_back(victim_order[candidate]);
        self(self, cardinality, candidate + 1U, selected);
        selected.pop_back();
        if (exchanged || result.counters.cancelled || coupled_budget_exhausted ||
            coupled_attempts >= config.maximum_coupled_exchange_attempts) return;
      }
    };
    // Give the most causal prefix of each component size its first ordering before spending the
    // attempt allowance on other pair orderings. The complete lazy enumeration below then fills
    // remaining allowance without repeating these signatures.
    for (std::size_t cardinality = 2U; !exchanged && !result.counters.cancelled && !coupled_budget_exhausted &&
         cardinality <= maximum_component_size && coupled_attempts < config.maximum_coupled_exchange_attempts;
         ++cardinality) {
      std::vector<std::size_t> leading(victim_order.begin(),
                                       victim_order.begin() + static_cast<std::ptrdiff_t>(cardinality));
      std::sort(leading.begin(), leading.end());
      attempt_unique_component(leading);
    }
    for (std::size_t cardinality = 2U; !exchanged && !result.counters.cancelled && !coupled_budget_exhausted &&
         cardinality <= maximum_component_size && coupled_attempts < config.maximum_coupled_exchange_attempts;
         ++cardinality) {
      std::vector<std::size_t> selected;
      selected.reserve(cardinality);
      enumerate_components(enumerate_components, cardinality, 0U, selected);
    }
    if (!exchanged && coupled_attempts > 0U && coupled_budget_limited)
      ++result.counters.coupled_expansion_budget_exhaustions;
    if (!exchanged) publish_legal_subset();
  }
  account_requests(result);
  // Regional admission may revalidate a retained subset with search and exchange budgets set to
  // zero.  When that reconstruction produced exactly the same checked scene, an immediate
  // budget termination says nothing new about a still-missing request.  Keep the causal evidence
  // from the identical scene instead of replacing it with an empty blocker set.  Equality is
  // intentionally strict: a changed request order, route, or copper scene must collect fresh
  // evidence rather than inheriting ownership from an obsolete obstacle layout.
  const bool evidence_only_revalidation = retained != nullptr && !result.counters.cancelled &&
      config.maximum_expansions_per_connection == 0U && config.maximum_victim_exchanges == 0U &&
      config.maximum_coupled_exchange_attempts == 0U &&
      retained->request_identities == result.request_identities && retained->paths == result.paths &&
      retained->copper == result.copper && retained->satisfied_indices == result.satisfied_indices &&
      retained->missing_indices == result.missing_indices &&
      retained->request_evidence.size() == result.request_evidence.size();
  if (evidence_only_revalidation) {
    result.request_evidence = retained->request_evidence;
    result.exchange_rollbacks = retained->exchange_rollbacks;
    result.blocked_passages = retained->blocked_passages;
  }
  result.complete = result.missing_indices.empty() && !result.counters.cancelled;
  if (result.complete) result.diagnostic = "all requested connections have a legal copper scaffold";
  else if (result.counters.cancelled) result.diagnostic = "copper scaffold cancelled; legal subset retained";
  else result.diagnostic = "some requested connections have no bounded legal scaffold";
  for (const auto i : result.missing_indices)
    result.diagnostic += "; missing " + requests[i].net.value() + ":" +
                         std::to_string(requests[i].connection_index);
  return result;
}

} // namespace flux
