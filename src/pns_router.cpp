#include "flux/core/pns_router.hpp"

#include "flux/core/connectivity.hpp"
#include "flux/core/drc.hpp"
#include "flux/core/geometry.hpp"
#include "flux/core/validation.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <map>
#include <optional>
#include <set>
#include <string>
#include <utility>
#include <vector>

namespace flux {
namespace {

[[nodiscard]] Point translated(Point point, long double dx, long double dy) {
  constexpr auto low = static_cast<long double>(std::numeric_limits<std::int64_t>::min());
  constexpr auto high = static_cast<long double>(std::numeric_limits<std::int64_t>::max());
  return Point{Nanometres{static_cast<std::int64_t>(std::llround(
                   std::clamp(static_cast<long double>(point.x.value()) + dx, low, high)))},
               Nanometres{static_cast<std::int64_t>(std::llround(
                   std::clamp(static_cast<long double>(point.y.value()) + dy, low, high)))}};
}

[[nodiscard]] std::set<std::string> object_ids(const Board& board) {
  std::set<std::string> result;
  for (const auto& track : board.tracks) {
    result.insert(track.id.value());
  }
  for (const auto& arc : board.track_arcs) {
    result.insert(arc.id.value());
  }
  for (const auto& curve : board.track_beziers) result.insert(curve.id.value());
  for (const auto& via : board.vias) {
    result.insert(via.id.value());
  }
  for (const auto& footprint : board.footprints) {
    result.insert(footprint.id.value());
    for (const auto& pad : footprint.pads) {
      result.insert(pad.id.value());
    }
  }
  for (const auto& zone : board.zones) {
    result.insert(zone.id.value());
  }
  for (const auto& keepout : board.keepouts) {
    result.insert(keepout.id.value());
  }
  return result;
}

[[nodiscard]] ObjectId unique_id(std::string base, std::set<std::string>& used) {
  auto candidate = base;
  std::size_t suffix = 1U;
  while (used.contains(candidate)) {
    candidate = base + "-" + std::to_string(suffix++);
  }
  used.insert(candidate);
  return ObjectId{std::move(candidate)};
}

[[nodiscard]] std::map<NetId, std::size_t> open_connections_by_net(const Board& board) {
  std::map<NetId, std::size_t> result;
  for (const auto& net : analyze_connectivity(board).nets) {
    result.emplace(net.net, net.open_connections);
  }
  return result;
}

[[nodiscard]] bool connectivity_did_not_regress(const std::map<NetId, std::size_t>& baseline,
                                                const Board& candidate) {
  for (const auto& net : analyze_connectivity(candidate).nets) {
    const auto found = baseline.find(net.net);
    const auto allowed = found == baseline.end() ? 0U : found->second;
    if (net.open_connections > allowed) {
      return false;
    }
  }
  return true;
}

[[nodiscard]] bool violation_touches(const CopperViolation& violation,
                                     const std::set<std::string>& active) {
  return std::any_of(violation.object_ids.begin(), violation.object_ids.end(),
                     [&active](const auto& id) { return active.contains(id); });
}

struct AttemptResult {
  std::optional<Board> candidate;
  std::size_t depth{0};
  std::size_t moved_tracks{0};
  std::size_t moved_vias{0};
  std::size_t doglegs{0};
  std::size_t local_splits{0};
  std::vector<std::string> moved_ids;
};

[[nodiscard]] AttemptResult attempt_shove(const Board& source, const PnsShoveRequest& request,
                                          const PnsShoveConfig& config, long double direction,
                                          Nanometres displacement, std::size_t serial,
                                          std::optional<std::size_t> explicit_via_candidate = {}) {
  AttemptResult result;
  Board candidate = source;
  auto used = object_ids(candidate);
  const std::set<std::string> protected_ids(request.protected_object_ids.begin(),
                                            request.protected_object_ids.end());
  std::set<std::string> active(request.intruder_object_ids.begin(),
                               request.intruder_object_ids.end());
  std::set<std::string> moved;
  const auto baseline_connectivity = open_connections_by_net(source);
  const auto normal_length = std::hypot(request.normal_x, request.normal_y);
  if (!(normal_length > 0.0L)) {
    return result;
  }
  const auto dx =
      direction * static_cast<long double>(displacement.value()) * request.normal_x / normal_length;
  const auto dy =
      direction * static_cast<long double>(displacement.value()) * request.normal_y / normal_length;

  std::vector<std::string> pending = request.colliding_object_ids;
  for (std::size_t depth = 0U; depth < config.max_propagation_depth; ++depth) {
    std::optional<std::size_t> selected_track;
    std::optional<std::size_t> selected_via;
    for (const auto& id : pending) {
      const auto found =
          std::find_if(candidate.tracks.begin(), candidate.tracks.end(), [&](const auto& track) {
            return track.id.value() == id && !track.locked && !protected_ids.contains(id) &&
                   !active.contains(id) && !moved.contains(id);
          });
      if (found != candidate.tracks.end()) {
        selected_track = static_cast<std::size_t>(found - candidate.tracks.begin());
        break;
      }
      const auto via =
          std::find_if(candidate.vias.begin(), candidate.vias.end(), [&](const auto& item) {
            return item.id.value() == id && !item.locked && !protected_ids.contains(id) &&
                   !active.contains(id) && !moved.contains(id);
          });
      if (via != candidate.vias.end()) {
        selected_via = static_cast<std::size_t>(via - candidate.vias.begin());
        break;
      }
    }
    if (!selected_track && !selected_via) {
      return result;
    }

    if (selected_via) {
      auto& via = candidate.vias[*selected_via];
      const auto original_position = via.position;
      auto next_position = translated(original_position, dx, dy);
      auto next_layers = via.layers;
      if (explicit_via_candidate) {
        if (*explicit_via_candidate >= request.via_relocation_candidates.size()) {
          return result;
        }
        const auto& proposed = request.via_relocation_candidates[*explicit_via_candidate];
        if (proposed.via != via.id || proposed.layers.empty()) {
          return result;
        }
        next_position = proposed.position;
        next_layers = proposed.layers;
      }
      std::sort(next_layers.begin(), next_layers.end());
      next_layers.erase(std::unique(next_layers.begin(), next_layers.end()), next_layers.end());
      auto original_layers = via.layers;
      std::sort(original_layers.begin(), original_layers.end());
      original_layers.erase(std::unique(original_layers.begin(), original_layers.end()),
                            original_layers.end());
      if (next_layers.empty() ||
          (next_position == original_position && next_layers == original_layers)) {
        return result;
      }

      // A via coincident with a pad or attached arc cannot be moved by this local straight-copper
      // transaction. Those cases require a terminal/group or native-arc solver.
      for (const auto& footprint : candidate.footprints) {
        for (const auto& pad : footprint.pads) {
          if (pad.net == via.net &&
              geometry::transform_point(pad.local_position, footprint.position,
                                        footprint.rotation, footprint.side) == original_position) {
            return result;
          }
        }
      }
      if (std::any_of(candidate.track_arcs.begin(), candidate.track_arcs.end(), [&](const auto& arc) {
            return arc.net == via.net &&
                   (arc.start == original_position || arc.end == original_position);
          })) {
        return result;
      }

      if (std::any_of(candidate.track_beziers.begin(), candidate.track_beziers.end(), [&](const auto& curve) {
            return curve.net == via.net &&
                   (curve.start == original_position || curve.end == original_position);
          })) return result;
      std::vector<std::size_t> attached_tracks;
      for (std::size_t index = 0U; index < candidate.tracks.size(); ++index) {
        const auto& track = candidate.tracks[index];
        if (track.net != via.net ||
            (track.start != original_position && track.end != original_position)) {
          continue;
        }
        if (track.locked || protected_ids.contains(track.id.value()) ||
            std::find(next_layers.begin(), next_layers.end(), track.layer) == next_layers.end()) {
          return result;
        }
        attached_tracks.push_back(index);
      }
      if (attached_tracks.empty()) {
        return result;
      }

      via.position = next_position;
      via.layers = std::move(next_layers);
      active.insert(via.id.value());
      moved.insert(via.id.value());
      result.moved_ids.push_back(via.id.value());
      ++result.moved_vias;
      for (const auto index : attached_tracks) {
        auto& track = candidate.tracks[index];
        if (track.start == original_position) {
          track.start = next_position;
        }
        if (track.end == original_position) {
          track.end = next_position;
        }
        active.insert(track.id.value());
        if (moved.insert(track.id.value()).second) {
          result.moved_ids.push_back(track.id.value());
          ++result.moved_tracks;
        }
      }
      result.depth = depth + 1U;

      if (!validate_board(candidate).ok() ||
          !connectivity_did_not_regress(baseline_connectivity, candidate)) {
        return result;
      }
      const auto drc = check_copper_rules(candidate);
      const auto collision = std::find_if(
          drc.violations.begin(), drc.violations.end(),
          [&active](const auto& violation) { return violation_touches(violation, active); });
      if (collision == drc.violations.end()) {
        result.candidate = std::move(candidate);
        return result;
      }
      pending.clear();
      for (const auto& id : collision->object_ids) {
        if (!active.contains(id) && !moved.contains(id) && !protected_ids.contains(id)) {
          pending.push_back(id);
        }
      }
      continue;
    }

    const auto original = candidate.tracks[*selected_track];
    const auto segment_x =
        static_cast<long double>(original.end.x.value() - original.start.x.value());
    const auto segment_y =
        static_cast<long double>(original.end.y.value() - original.start.y.value());
    const auto segment_length = std::hypot(segment_x, segment_y);
    if (!(segment_length > 0.0L)) {
      return result;
    }
    const auto project = [&](Point point) {
      return std::clamp(
          ((static_cast<long double>(point.x.value() - original.start.x.value()) * segment_x +
            static_cast<long double>(point.y.value() - original.start.y.value()) * segment_y) /
           (segment_length * segment_length)),
          0.0L, 1.0L);
    };
    const auto projected = project(request.collision_point);
    const auto run_up = std::max(
        static_cast<long double>(displacement.value()) * 2.0L,
        static_cast<long double>(original.width.value() + config.minimum_displacement.value()));
    const auto parameter_span = std::min(0.5L, run_up / segment_length);
    auto first_parameter = projected;
    auto last_parameter = projected;
    for (const auto& intruder_id : request.intruder_object_ids) {
      const auto intruder =
          std::find_if(source.tracks.begin(), source.tracks.end(),
                       [&](const auto& track) { return track.id.value() == intruder_id; });
      if (intruder == source.tracks.end() || intruder->layer != original.layer) {
        continue;
      }
      const auto first_projection = project(intruder->start);
      const auto last_projection = project(intruder->end);
      first_parameter = std::min(first_parameter, std::min(first_projection, last_projection));
      last_parameter = std::max(last_parameter, std::max(first_projection, last_projection));
    }
    first_parameter = std::max(0.0L, first_parameter - parameter_span);
    last_parameter = std::min(1.0L, last_parameter + parameter_span);
    const auto interpolate = [&](long double parameter) {
      return Point{
          Nanometres{static_cast<std::int64_t>(std::llround(
              static_cast<long double>(original.start.x.value()) + parameter * segment_x))},
          Nanometres{static_cast<std::int64_t>(std::llround(
              static_cast<long double>(original.start.y.value()) + parameter * segment_y))}};
    };
    const auto local_start = interpolate(first_parameter);
    const auto local_end = interpolate(last_parameter);
    const auto shifted_start = translated(local_start, dx, dy);
    const auto shifted_end = translated(local_end, dx, dy);
    if (local_start == local_end || shifted_start == local_start || shifted_end == local_end ||
        shifted_start == shifted_end) {
      return result;
    }
    candidate.tracks.erase(candidate.tracks.begin() + static_cast<std::ptrdiff_t>(*selected_track));
    const auto prefix =
        original.id.value() + "-pns-" + std::to_string(serial) + "-" + std::to_string(depth);
    const auto append = [&](Point start, Point end, std::string suffix) {
      if (start == end) {
        return;
      }
      auto id = unique_id(prefix + "-" + std::move(suffix), used);
      active.insert(id.value());
      candidate.tracks.push_back(TrackSegment{std::move(id), original.net, original.layer, start,
                                              end, original.width, false});
      ++result.doglegs;
    };
    append(original.start, local_start, "source-tail");
    append(local_start, shifted_start, "entry");
    append(shifted_start, shifted_end, "parallel");
    append(shifted_end, local_end, "exit");
    append(local_end, original.end, "destination-tail");
    moved.insert(original.id.value());
    result.moved_ids.push_back(original.id.value());
    ++result.moved_tracks;
    ++result.local_splits;
    result.depth = depth + 1U;

    if (!validate_board(candidate).ok() ||
        !connectivity_did_not_regress(baseline_connectivity, candidate)) {
      return result;
    }
    const auto drc = check_copper_rules(candidate);
    const auto collision = std::find_if(
        drc.violations.begin(), drc.violations.end(),
        [&active](const auto& violation) { return violation_touches(violation, active); });
    if (collision == drc.violations.end()) {
      result.candidate = std::move(candidate);
      return result;
    }
    pending.clear();
    for (const auto& id : collision->object_ids) {
      if (!active.contains(id) && !moved.contains(id) && !protected_ids.contains(id)) {
        pending.push_back(id);
      }
    }
  }
  return result;
}

} // namespace

PnsShoveResult pns_push_and_shove(const Board& board, const PnsShoveRequest& request,
                                  const PnsShoveConfig& config) {
  PnsShoveResult result;
  if (request.intruder_object_ids.empty() || request.colliding_object_ids.empty() ||
      config.max_propagation_depth == 0U || config.max_candidate_checks == 0U ||
      config.minimum_displacement.value() <= 0 || config.maximum_displacement.value() <= 0 ||
      config.maximum_displacement < config.minimum_displacement ||
      !(std::hypot(request.normal_x, request.normal_y) > 0.0L)) {
    result.diagnostic = "PNS shove request or bounds are invalid";
    return result;
  }
  auto displacement = std::max(request.required_displacement, config.minimum_displacement);
  displacement = std::min(displacement, config.maximum_displacement);
  for (std::size_t scale = 0U; scale < 8U; ++scale) {
    for (const auto direction : {1.0L, -1.0L}) {
      if (result.candidate_checks >= config.max_candidate_checks) {
        result.budget_exhausted = true;
        result.diagnostic = "PNS exact-candidate budget exhausted";
        return result;
      }
      ++result.candidate_checks;
      auto attempted =
          attempt_shove(board, request, config, direction, displacement, result.candidate_checks);
      if (attempted.candidate) {
        result.candidate = std::move(attempted.candidate);
        result.propagation_depth = attempted.depth;
        result.moved_tracks = attempted.moved_tracks;
        result.moved_vias = attempted.moved_vias;
        result.added_dogleg_tracks = attempted.doglegs;
        result.local_splits = attempted.local_splits;
        result.moved_object_ids = std::move(attempted.moved_ids);
        result.diagnostic = "bounded PNS shove completed with exact connectivity and DRC gates";
        return result;
      }
    }
    if (displacement >= config.maximum_displacement ||
        displacement.value() > config.maximum_displacement.value() / 2) {
      break;
    }
    displacement = Nanometres{displacement.value() * 2};
  }
  for (std::size_t candidate_index = 0U;
       candidate_index < request.via_relocation_candidates.size(); ++candidate_index) {
    if (result.candidate_checks >= config.max_candidate_checks) {
      result.budget_exhausted = true;
      result.diagnostic = "PNS exact-candidate budget exhausted";
      return result;
    }
    ++result.candidate_checks;
    auto attempted = attempt_shove(board, request, config, 1.0L, displacement,
                                   result.candidate_checks, candidate_index);
    if (!attempted.candidate) {
      continue;
    }
    result.candidate = std::move(attempted.candidate);
    result.propagation_depth = attempted.depth;
    result.moved_tracks = attempted.moved_tracks;
    result.moved_vias = attempted.moved_vias;
    result.added_dogleg_tracks = attempted.doglegs;
    result.local_splits = attempted.local_splits;
    result.moved_object_ids = std::move(attempted.moved_ids);
    result.diagnostic =
        "bounded PNS via-site transaction completed with exact connectivity and DRC gates";
    return result;
  }
  result.diagnostic = "PNS shove could not move the attributed collision through legal copper";
  return result;
}

} // namespace flux
