#include "flux/core/routing_compute.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <queue>

namespace flux {
namespace {

constexpr auto unreachable = std::numeric_limits<std::uint32_t>::max();

[[nodiscard]] long double point_segment_distance(const RoutingComputePoint& point,
                                                 const RoutingComputeSegment& segment) {
  const auto dx = static_cast<long double>(segment.second_x_nm - segment.first_x_nm);
  const auto dy = static_cast<long double>(segment.second_y_nm - segment.first_y_nm);
  const auto length_squared = dx * dx + dy * dy;
  const auto parameter =
      length_squared > 0.0L
          ? std::clamp(((static_cast<long double>(point.x_nm - segment.first_x_nm) * dx) +
                        (static_cast<long double>(point.y_nm - segment.first_y_nm) * dy)) /
                           length_squared,
                       0.0L, 1.0L)
          : 0.0L;
  return std::hypotl(static_cast<long double>(point.x_nm) -
                         (static_cast<long double>(segment.first_x_nm) + parameter * dx),
                     static_cast<long double>(point.y_nm) -
                         (static_cast<long double>(segment.first_y_nm) + parameter * dy));
}

[[nodiscard]] long double point_segment_distance(std::int64_t x, std::int64_t y,
                                                 const RoutingComputeSegment& segment) {
  return point_segment_distance(
      RoutingComputePoint{x, y, 0, segment.layer, std::numeric_limits<std::uint32_t>::max()},
      segment);
}

[[nodiscard]] bool segments_intersect(const RoutingComputeSegment& first,
                                      const RoutingComputeSegment& second) {
  const auto first_degenerate =
      first.first_x_nm == first.second_x_nm && first.first_y_nm == first.second_y_nm;
  const auto second_degenerate =
      second.first_x_nm == second.second_x_nm && second.first_y_nm == second.second_y_nm;
  if (first_degenerate || second_degenerate) {
    return false;
  }
  const auto orientation = [](std::int64_t ax, std::int64_t ay, std::int64_t bx, std::int64_t by,
                              std::int64_t cx, std::int64_t cy) {
    return static_cast<long double>(bx - ax) * static_cast<long double>(cy - ay) -
           static_cast<long double>(by - ay) * static_cast<long double>(cx - ax);
  };
  const auto a = orientation(first.first_x_nm, first.first_y_nm, first.second_x_nm,
                             first.second_y_nm, second.first_x_nm, second.first_y_nm);
  const auto b = orientation(first.first_x_nm, first.first_y_nm, first.second_x_nm,
                             first.second_y_nm, second.second_x_nm, second.second_y_nm);
  const auto c = orientation(second.first_x_nm, second.first_y_nm, second.second_x_nm,
                             second.second_y_nm, first.first_x_nm, first.first_y_nm);
  const auto d = orientation(second.first_x_nm, second.first_y_nm, second.second_x_nm,
                             second.second_y_nm, first.second_x_nm, first.second_y_nm);
  if (((a < 0.0L && b > 0.0L) || (a > 0.0L && b < 0.0L)) &&
      ((c < 0.0L && d > 0.0L) || (c > 0.0L && d < 0.0L))) {
    return true;
  }
  const auto on_segment = [](std::int64_t ax, std::int64_t ay, std::int64_t bx, std::int64_t by,
                             std::int64_t px, std::int64_t py) {
    return px >= std::min(ax, bx) && px <= std::max(ax, bx) && py >= std::min(ay, by) &&
           py <= std::max(ay, by);
  };
  return (a == 0.0L && on_segment(first.first_x_nm, first.first_y_nm, first.second_x_nm,
                                  first.second_y_nm, second.first_x_nm, second.first_y_nm)) ||
         (b == 0.0L && on_segment(first.first_x_nm, first.first_y_nm, first.second_x_nm,
                                  first.second_y_nm, second.second_x_nm, second.second_y_nm)) ||
         (c == 0.0L && on_segment(second.first_x_nm, second.first_y_nm, second.second_x_nm,
                                  second.second_y_nm, first.first_x_nm, first.first_y_nm)) ||
         (d == 0.0L && on_segment(second.first_x_nm, second.first_y_nm, second.second_x_nm,
                                  second.second_y_nm, first.second_x_nm, first.second_y_nm));
}

[[nodiscard]] long double segment_distance(const RoutingComputeSegment& first,
                                           const RoutingComputeSegment& second) {
  if (segments_intersect(first, second))
    return 0.0L;
  return std::min({point_segment_distance(first.first_x_nm, first.first_y_nm, second),
                   point_segment_distance(first.second_x_nm, first.second_y_nm, second),
                   point_segment_distance(second.first_x_nm, second.first_y_nm, first),
                   point_segment_distance(second.second_x_nm, second.second_y_nm, first)});
}

[[nodiscard]] RoutingComputeProximityResult
probe_cpu(const std::vector<RoutingComputePoint>& queries,
          const std::vector<RoutingComputeSegment>& segments, std::int64_t guard) {
  RoutingComputeProximityResult result;
  result.queries.resize(queries.size());
  for (std::size_t query = 0U; query < queries.size(); ++query) {
    auto& output = result.queries[query];
    long double best = std::numeric_limits<long double>::infinity();
    for (std::size_t segment = 0U; segment < segments.size(); ++segment) {
      if (queries[query].layer != segments[segment].layer ||
          (queries[query].owner != std::numeric_limits<std::uint32_t>::max() &&
           queries[query].owner == segments[segment].owner)) {
        continue;
      }
      ++output.tested_segments;
      const auto margin = point_segment_distance(queries[query], segments[segment]) -
                          static_cast<long double>(queries[query].radius_nm) -
                          static_cast<long double>(segments[segment].radius_nm) - guard;
      if (margin < best) {
        best = margin;
        output.closest_segment = segment;
      }
    }
    if (std::isfinite(best)) {
      output.conservative_margin_nm = static_cast<std::int64_t>(std::floor(best));
      output.possible_conflict = output.conservative_margin_nm < 0;
    }
  }
  return result;
}

[[nodiscard]] RoutingComputeSegmentPairResult
conflicts_cpu(const std::vector<RoutingComputeSegment>& segments,
              const std::vector<std::pair<std::uint32_t, std::uint32_t>>& pairs,
              std::int64_t guard) {
  RoutingComputeSegmentPairResult result;
  result.possible_conflicts.assign(pairs.size(), 0U);
  for (std::size_t pair = 0U; pair < pairs.size(); ++pair) {
    const auto [first, second] = pairs[pair];
    if (first >= segments.size() || second >= segments.size()) {
      result.diagnostic = "routing compute segment pair is out of range";
      return result;
    }
    if (segments[first].layer != segments[second].layer ||
        segments[first].owner == segments[second].owner) {
      continue;
    }
    const auto required = segments[first].radius_nm + segments[second].radius_nm + guard;
    result.possible_conflicts[pair] =
        segment_distance(segments[first], segments[second]) < required ? 1U : 0U;
  }
  return result;
}

[[nodiscard]] RoutingComputeChordBatchResult
classify_chords_cpu(const std::vector<RoutingComputeSegment>& chords,
                    const std::vector<RoutingComputeSegment>& boundaries, std::int64_t required,
                    std::int64_t guard) {
  RoutingComputeChordBatchResult result;
  result.chords.resize(chords.size());
  for (std::size_t chord = 0U; chord < chords.size(); ++chord) {
    auto clearance = std::numeric_limits<long double>::infinity();
    bool intersects = false;
    for (std::size_t boundary = 0U; boundary < boundaries.size(); ++boundary) {
      const auto distance = segment_distance(chords[chord], boundaries[boundary]);
      if (distance < clearance) {
        clearance = distance;
        result.chords[chord].closest_boundary = boundary;
      }
      intersects = intersects || distance <= 0.0L;
    }
    result.chords[chord].conservative_clearance_nm =
        std::isfinite(clearance) ? static_cast<std::int64_t>(std::floor(clearance)) : 0;
    if (intersects)
      result.chords[chord].classification = RoutingComputeChordClass::definitely_blocked;
    else if (clearance >= static_cast<long double>(required + guard))
      result.chords[chord].classification = RoutingComputeChordClass::definitely_clear;
    else
      result.chords[chord].classification = RoutingComputeChordClass::ambiguous;
  }
  return result;
}

[[nodiscard]] RoutingComputeWavefrontResult
wavefront_cpu(const RoutingComputeWavefrontInput& input) {
  RoutingComputeWavefrontResult result;
  const auto count = input.width * input.height;
  result.distance.assign(count, unreachable);
  std::queue<std::size_t> frontier;
  for (const auto source : input.sources) {
    if (source >= count) {
      result.diagnostic = "routing compute wavefront source is out of range";
      result.distance.clear();
      return result;
    }
    result.distance[source] = 0U;
    frontier.push(source);
  }
  while (!frontier.empty()) {
    const auto cell = frontier.front();
    frontier.pop();
    result.passes = std::max(result.passes, static_cast<std::size_t>(result.distance[cell]));
    if (result.distance[cell] >= input.maximum_passes)
      continue;
    const auto x = cell % input.width;
    const auto y = cell / input.width;
    for (const auto [dx, dy] : std::array<std::pair<int, int>, 4U>{
             std::pair{1, 0}, std::pair{-1, 0}, std::pair{0, 1}, std::pair{0, -1}}) {
      const auto nx = static_cast<std::int64_t>(x) + dx;
      const auto ny = static_cast<std::int64_t>(y) + dy;
      if (nx < 0 || ny < 0 || nx >= static_cast<std::int64_t>(input.width) ||
          ny >= static_cast<std::int64_t>(input.height))
        continue;
      const auto next = static_cast<std::size_t>(ny) * input.width + static_cast<std::size_t>(nx);
      if (input.traversable[next] == 0U || result.distance[next] != unreachable)
        continue;
      result.distance[next] = result.distance[cell] + 1U;
      frontier.push(next);
    }
  }
  for (std::size_t cell = 0U; cell < count; ++cell) {
    if (input.traversable[cell] != 0U && result.distance[cell] == unreachable) {
      result.budget_exhausted = true;
      break;
    }
  }
  return result;
}

[[nodiscard]] RoutingComputeWavefrontResult
graph_wavefront_cpu(const RoutingComputeGraphWavefrontInput& input) {
  RoutingComputeWavefrontResult result;
  result.distance.assign(input.node_count, unreachable);
  using QueueEntry = std::pair<std::uint32_t, std::uint32_t>;
  std::priority_queue<QueueEntry, std::vector<QueueEntry>, std::greater<>> frontier;
  for (const auto source : input.sources) {
    if (source >= input.node_count) {
      result.diagnostic = "routing compute graph wavefront source is out of range";
      result.distance.clear();
      return result;
    }
    result.distance[source] = 0U;
    frontier.emplace(0U, static_cast<std::uint32_t>(source));
  }
  while (!frontier.empty() && result.passes < input.maximum_passes) {
    const auto [distance, node] = frontier.top();
    frontier.pop();
    if (distance != result.distance[node])
      continue;
    ++result.passes;
    for (auto edge = input.offsets[node]; edge < input.offsets[node + 1U]; ++edge) {
      const auto& candidate = input.edges[edge];
      const auto next = distance > unreachable - candidate.cost
                            ? unreachable
                            : static_cast<std::uint32_t>(distance + candidate.cost);
      if (next < result.distance[candidate.node]) {
        result.distance[candidate.node] = next;
        frontier.emplace(next, candidate.node);
      }
    }
  }
  result.budget_exhausted = !frontier.empty();
  return result;
}

[[nodiscard]] RoutingComputeLinearScores
scores_cpu(const std::vector<RoutingComputeLinearCandidate>& candidates,
           const std::array<std::uint32_t, 8U>& weights) {
  RoutingComputeLinearScores result;
  result.scores.resize(candidates.size(), 0U);
  for (std::size_t candidate = 0U; candidate < candidates.size(); ++candidate) {
    for (std::size_t term = 0U; term < weights.size(); ++term) {
      const auto value =
          static_cast<std::uint64_t>(candidates[candidate].terms[term]) * weights[term];
      result.scores[candidate] =
          value > std::numeric_limits<std::uint32_t>::max() - result.scores[candidate]
              ? std::numeric_limits<std::uint32_t>::max()
              : result.scores[candidate] + value;
    }
  }
  return result;
}

[[nodiscard]] RoutingComputeSplineBatchResult
spline_batch_cpu(const RoutingComputeSplineBatchInput& input) {
  RoutingComputeSplineBatchResult result;
  const auto controls_per_candidate = input.control_points_nm.size() / input.candidate_count;
  result.positions_nm.resize(input.sample_count * input.candidate_count);
  for (std::size_t candidate = 0U; candidate < input.candidate_count; ++candidate) {
    for (std::size_t sample = 0U; sample < input.sample_count; ++sample) {
      long double x = 0.0L;
      long double y = 0.0L;
      const auto control_base = candidate * controls_per_candidate;
      const auto coefficient_base =
          (candidate * input.sample_count + sample) * controls_per_candidate;
      for (std::size_t control = 0U; control < controls_per_candidate; ++control) {
        const auto coefficient = input.basis_coefficients[coefficient_base + control];
        x += coefficient * input.control_points_nm[control_base + control].first;
        y += coefficient * input.control_points_nm[control_base + control].second;
      }
      result.positions_nm[candidate * input.sample_count + sample] = {x, y};
    }
  }
  return result;
}

[[nodiscard]] RoutingComputeSplineScoreResult
spline_scores_cpu(const RoutingComputeSplineScoreInput& input) {
  RoutingComputeSplineScoreResult result;
  const auto positions = spline_batch_cpu(input);
  if (!positions.ok()) {
    result.diagnostic = positions.diagnostic;
    return result;
  }
  constexpr long double nm_per_mm = 1'000'000.0L;
  std::vector<RoutingComputeSplineCandidateScore> ranking;
  ranking.reserve(input.candidate_count);
  const auto field_cell = [&](long double x, long double y) -> std::optional<std::size_t> {
    const auto fx = static_cast<std::int64_t>(
        std::floor((x - input.field_origin_x_nm) / static_cast<long double>(input.field_pitch_nm)));
    const auto fy = static_cast<std::int64_t>(
        std::floor((y - input.field_origin_y_nm) / static_cast<long double>(input.field_pitch_nm)));
    if (fx < 0 || fy < 0 || fx >= static_cast<std::int64_t>(input.field_width) ||
        fy >= static_cast<std::int64_t>(input.field_height))
      return std::nullopt;
    return static_cast<std::size_t>(fy) * input.field_width + static_cast<std::size_t>(fx);
  };
  for (std::size_t candidate = 0U; candidate < input.candidate_count; ++candidate) {
    RoutingComputeSplineCandidateScore score;
    score.candidate = candidate;
    const auto base = candidate * input.sample_count;
    long double prior_turn = 0.0L;
    for (std::size_t sample = 0U; sample < input.sample_count; ++sample) {
      const auto [x, y] = positions.positions_nm[base + sample];
      const auto cell = field_cell(x, y);
      if (!cell || input.corridor_mask[*cell] == 0U ||
          static_cast<long double>(input.signed_clearance_nm[*cell]) <
              static_cast<long double>(input.minimum_clearance_nm))
        ++score.violations;
      if (cell && static_cast<long double>(input.signed_clearance_nm[*cell]) <
                      static_cast<long double>(input.clearance_influence_nm)) {
        const auto deficit =
            (static_cast<long double>(input.clearance_influence_nm) -
             static_cast<long double>(input.signed_clearance_nm[*cell])) /
            static_cast<long double>(std::max<std::int64_t>(1, input.clearance_influence_nm));
        score.objective += input.objective_weights[4] * deficit * deficit;
      }
      if (sample > 0U) {
        const auto [px, py] = positions.positions_nm[base + sample - 1U];
        score.objective += input.objective_weights[0] * std::hypotl(x - px, y - py) / nm_per_mm;
      }
      if (sample > 0U && sample + 1U < input.sample_count) {
        const auto [px, py] = positions.positions_nm[base + sample - 1U];
        const auto [nx, ny] = positions.positions_nm[base + sample + 1U];
        const auto first_length = std::hypotl(x - px, y - py);
        const auto second_length = std::hypotl(nx - x, ny - y);
        if (first_length > 0.0L && second_length > 0.0L) {
          const auto cross =
              ((x - px) * (ny - y) - (y - py) * (nx - x)) / (first_length * second_length);
          const auto dot =
              ((x - px) * (nx - x) + (y - py) * (ny - y)) / (first_length * second_length);
          const auto turn = std::atan2(cross, dot);
          const auto radius = std::min(first_length, second_length) /
                              std::max(1.0e-9L, 2.0L * std::sin(std::abs(turn) * 0.5L));
          score.objective += input.objective_weights[1] * turn * turn;
          const auto change = turn - prior_turn;
          score.objective += input.objective_weights[2] * change * change;
          if (input.minimum_bend_radius_nm > 0 && radius < input.minimum_bend_radius_nm) {
            ++score.violations;
            const auto deficit = (input.minimum_bend_radius_nm - radius) / nm_per_mm;
            score.objective += input.objective_weights[3] * deficit * deficit;
          }
          prior_turn = turn;
        }
      }
    }
    ranking.push_back(score);
  }
  std::stable_sort(ranking.begin(), ranking.end(), [](const auto& lhs, const auto& rhs) {
    if (lhs.violations != rhs.violations)
      return lhs.violations < rhs.violations;
    if (lhs.approximate_self_intersections != rhs.approximate_self_intersections)
      return lhs.approximate_self_intersections < rhs.approximate_self_intersections;
    if (lhs.objective != rhs.objective)
      return lhs.objective < rhs.objective;
    return lhs.candidate < rhs.candidate;
  });
  result.assessed_candidates = ranking.size();
  ranking.resize(std::min(input.top_k, ranking.size()));
  result.shortlist = std::move(ranking);
  return result;
}

[[nodiscard]] RoutingComputeViaCandidateResult
via_candidates_cpu(const RoutingComputeViaCandidateInput& input) {
  RoutingComputeViaCandidateResult result;
  result.rejection_bitmap.resize(input.candidates.size(), 0U);
  std::vector<RoutingComputeViaCandidateAssessment> ranking;
  ranking.reserve(input.candidates.size());
  for (std::size_t index = 0U; index < input.candidates.size(); ++index) {
    const auto& candidate = input.candidates[index];
    RoutingComputeViaCandidateAssessment assessment;
    assessment.candidate = index;
    assessment.corridor_occupancy_pressure = candidate.corridor_occupancy_pressure;
    if (candidate.layer_mask == 0U || candidate.runup_count > candidate.runup_origins_nm.size() ||
        candidate.radius_nm < 0 || candidate.runup_radius_nm < 0) {
      assessment.rejection_bits |=
          static_cast<std::uint32_t>(RoutingComputeViaRejection::invalid_layer_span);
    }
    auto margin = std::numeric_limits<long double>::infinity();
    const auto required_guard = input.clearance_nm + input.numerical_guard_nm;
    for (const auto& obstacle : input.obstacles) {
      if (obstacle.layer >= 64U ||
          (candidate.layer_mask & (std::uint64_t{1U} << obstacle.layer)) == 0U ||
          (candidate.owner != std::numeric_limits<std::uint32_t>::max() &&
           candidate.owner == obstacle.owner))
        continue;
      const RoutingComputePoint site{candidate.x_nm, candidate.y_nm, candidate.radius_nm,
                                     obstacle.layer, candidate.owner};
      const auto site_margin = point_segment_distance(site, obstacle) - candidate.radius_nm -
                               obstacle.radius_nm - required_guard;
      margin = std::min(margin, site_margin);
      if (site_margin < 0.0L) {
        ++assessment.approximate_conflicts;
        assessment.rejection_bits |=
            static_cast<std::uint32_t>(RoutingComputeViaRejection::obstacle_conflict);
      }
      for (std::size_t runup = 0U; runup < candidate.runup_count; ++runup) {
        if (candidate.runup_layers[runup] != obstacle.layer)
          continue;
        const RoutingComputeSegment runup_segment{candidate.runup_origins_nm[runup].first,
                                                  candidate.runup_origins_nm[runup].second,
                                                  candidate.x_nm,
                                                  candidate.y_nm,
                                                  candidate.runup_radius_nm,
                                                  obstacle.layer,
                                                  candidate.owner};
        const auto runup_margin = segment_distance(runup_segment, obstacle) -
                                  candidate.runup_radius_nm - obstacle.radius_nm - required_guard;
        margin = std::min(margin, runup_margin);
        if (runup_margin < 0.0L) {
          ++assessment.approximate_conflicts;
          assessment.rejection_bits |=
              static_cast<std::uint32_t>(RoutingComputeViaRejection::runup_conflict);
        }
      }
    }
    for (std::size_t runup = 0U; runup < candidate.runup_count; ++runup) {
      assessment.runup_length_nm += static_cast<std::uint64_t>(std::llround(std::hypotl(
          static_cast<long double>(candidate.x_nm - candidate.runup_origins_nm[runup].first),
          static_cast<long double>(candidate.y_nm - candidate.runup_origins_nm[runup].second))));
    }
    assessment.conservative_margin_nm = std::isfinite(margin)
                                            ? static_cast<std::int64_t>(std::floor(margin))
                                            : std::numeric_limits<std::int64_t>::max();
    const auto deficit = std::max<std::int64_t>(0, -assessment.conservative_margin_nm);
    assessment.score = static_cast<long double>(assessment.approximate_conflicts) * 1.0e12L +
                       static_cast<long double>(deficit) * 1.0e3L +
                       static_cast<long double>(candidate.corridor_occupancy_pressure) * 1.0e6L +
                       static_cast<long double>(assessment.runup_length_nm);
    result.rejection_bitmap[index] = assessment.rejection_bits;
    ranking.push_back(assessment);
  }
  std::stable_sort(ranking.begin(), ranking.end(), [](const auto& lhs, const auto& rhs) {
    const auto lhs_rejected = lhs.rejection_bits != 0U;
    const auto rhs_rejected = rhs.rejection_bits != 0U;
    if (lhs_rejected != rhs_rejected)
      return !lhs_rejected;
    if (lhs.score != rhs.score)
      return lhs.score < rhs.score;
    if (lhs.conservative_margin_nm != rhs.conservative_margin_nm)
      return lhs.conservative_margin_nm > rhs.conservative_margin_nm;
    return lhs.candidate < rhs.candidate;
  });
  result.assessed_candidates = ranking.size();
  ranking.resize(std::min(input.top_k, ranking.size()));
  result.shortlist = std::move(ranking);
  return result;
}

} // namespace

#if FLUX_HAS_VULKAN_HYDRAULIC_PATHS
RoutingComputeProximityResult probe_routing_proximity_gpu(const std::vector<RoutingComputePoint>&,
                                                          const std::vector<RoutingComputeSegment>&,
                                                          std::int64_t);
RoutingComputeSegmentPairResult
discover_routing_segment_conflicts_gpu(const std::vector<RoutingComputeSegment>&,
                                       const std::vector<std::pair<std::uint32_t, std::uint32_t>>&,
                                       std::int64_t);
RoutingComputeChordBatchResult
classify_routing_chords_gpu(const std::vector<RoutingComputeSegment>&,
                            const std::vector<RoutingComputeSegment>&, std::int64_t, std::int64_t);
RoutingComputeWavefrontResult compute_routing_wavefront_gpu(const RoutingComputeWavefrontInput&);
RoutingComputeWavefrontResult
compute_routing_graph_wavefront_gpu(const RoutingComputeGraphWavefrontInput&);
RoutingComputeLinearScores
score_routing_compute_candidates_gpu(const std::vector<RoutingComputeLinearCandidate>&,
                                     const std::array<std::uint32_t, 8U>&);
RoutingComputeSplineBatchResult
evaluate_routing_spline_batch_gpu(const RoutingComputeSplineBatchInput&);
RoutingComputeSplineScoreResult
score_routing_spline_candidates_gpu(const RoutingComputeSplineScoreInput&);
RoutingComputeBroadphaseResult
discover_routing_broadphase_pairs_gpu(const std::vector<RoutingComputePrimitiveBounds>&,
                                      std::size_t);
RoutingComputeViaCandidateResult
filter_routing_via_candidates_gpu(const RoutingComputeViaCandidateInput&);
bool routing_compute_gpu_available_impl();
RoutingComputeDeviceInfo routing_compute_device_info_impl();
#endif

bool routing_compute_gpu_available() {
#if FLUX_HAS_VULKAN_HYDRAULIC_PATHS
  return routing_compute_gpu_available_impl();
#else
  return false;
#endif
}

RoutingComputeDeviceInfo routing_compute_device_info() {
#if FLUX_HAS_VULKAN_HYDRAULIC_PATHS
  return routing_compute_device_info_impl();
#else
  return {false, {}, {}, 0U, 0U, 0U, 0U,
          "Flux was built without the mandatory Vulkan routing backend"};
#endif
}

RoutingComputeProximityResult
probe_routing_proximity(const std::vector<RoutingComputePoint>& queries,
                        const std::vector<RoutingComputeSegment>& segments, std::int64_t guard,
                        bool prefer_gpu) {
  if (guard < 0)
    return RoutingComputeProximityResult{{}, false, false, "negative compute guard"};
#if FLUX_HAS_VULKAN_HYDRAULIC_PATHS
  if (prefer_gpu && !queries.empty() && !segments.empty() &&
      queries.size() <= std::numeric_limits<std::size_t>::max() / segments.size() &&
      queries.size() * segments.size() >= 4'096U) {
    auto gpu = probe_routing_proximity_gpu(queries, segments, guard);
    if (gpu.ok())
      return gpu;
    auto cpu = probe_cpu(queries, segments, guard);
    cpu.cpu_fallback = true;
    return cpu;
  }
#else
  static_cast<void>(prefer_gpu);
#endif
  return probe_cpu(queries, segments, guard);
}

RoutingComputeSegmentPairResult discover_routing_segment_conflicts(
    const std::vector<RoutingComputeSegment>& segments,
    const std::vector<std::pair<std::uint32_t, std::uint32_t>>& pairs, std::int64_t guard,
    bool prefer_gpu) {
#if FLUX_HAS_VULKAN_HYDRAULIC_PATHS
  if (prefer_gpu && pairs.size() >= 256U) {
    auto gpu = discover_routing_segment_conflicts_gpu(segments, pairs, guard);
    if (gpu.ok())
      return gpu;
    auto cpu = conflicts_cpu(segments, pairs, guard);
    cpu.cpu_fallback = true;
    return cpu;
  }
#else
  static_cast<void>(prefer_gpu);
#endif
  return conflicts_cpu(segments, pairs, guard);
}

RoutingComputeChordBatchResult
classify_routing_chords(const std::vector<RoutingComputeSegment>& chords,
                        const std::vector<RoutingComputeSegment>& boundary_segments,
                        std::int64_t required_clearance_nm, std::int64_t numerical_guard_nm,
                        bool prefer_gpu) {
  if (chords.empty() || boundary_segments.empty() || required_clearance_nm < 0 ||
      numerical_guard_nm < 0)
    return RoutingComputeChordBatchResult{
        {}, false, false, "invalid routing chord classification request"};
#if FLUX_HAS_VULKAN_HYDRAULIC_PATHS
  if (prefer_gpu &&
      chords.size() <= std::numeric_limits<std::size_t>::max() / boundary_segments.size() &&
      chords.size() * boundary_segments.size() >= 4'096U) {
    auto gpu = classify_routing_chords_gpu(chords, boundary_segments, required_clearance_nm,
                                           numerical_guard_nm);
    if (gpu.ok())
      return gpu;
    auto cpu =
        classify_chords_cpu(chords, boundary_segments, required_clearance_nm, numerical_guard_nm);
    cpu.cpu_fallback = true;
    return cpu;
  }
#else
  static_cast<void>(prefer_gpu);
#endif
  return classify_chords_cpu(chords, boundary_segments, required_clearance_nm, numerical_guard_nm);
}

RoutingComputeBroadphaseResult
discover_routing_broadphase_pairs(const std::vector<RoutingComputePrimitiveBounds>& primitives,
                                  std::int64_t cell_size_nm, std::size_t maximum_pairs,
                                  bool prefer_gpu) {
  RoutingComputeBroadphaseResult result;
  if (primitives.empty())
    return result;
  if (cell_size_nm <= 0 || maximum_pairs == 0U ||
      std::any_of(primitives.begin(), primitives.end(), [](const auto& primitive) {
        return primitive.minimum_x_nm > primitive.maximum_x_nm ||
               primitive.minimum_y_nm > primitive.maximum_y_nm;
      })) {
    result.diagnostic = "invalid routing broadphase request";
    return result;
  }
#if FLUX_HAS_VULKAN_HYDRAULIC_PATHS
  if (prefer_gpu && primitives.size() >= 64U && primitives.size() <= 65'535U &&
      primitives.size() <= std::numeric_limits<std::size_t>::max() / primitives.size()) {
    auto gpu = discover_routing_broadphase_pairs_gpu(primitives, maximum_pairs);
    if (gpu.ok() && !gpu.budget_exhausted)
      return gpu;
    result.cpu_fallback = true;
  }
#else
  static_cast<void>(prefer_gpu);
#endif
  const auto minimum_x =
      std::min_element(primitives.begin(), primitives.end(), [](const auto& a, const auto& b) {
        return a.minimum_x_nm < b.minimum_x_nm;
      })->minimum_x_nm;
  const auto minimum_y =
      std::min_element(primitives.begin(), primitives.end(), [](const auto& a, const auto& b) {
        return a.minimum_y_nm < b.minimum_y_nm;
      })->minimum_y_nm;
  const auto maximum_x =
      std::max_element(primitives.begin(), primitives.end(), [](const auto& a, const auto& b) {
        return a.maximum_x_nm < b.maximum_x_nm;
      })->maximum_x_nm;
  const auto maximum_y =
      std::max_element(primitives.begin(), primitives.end(), [](const auto& a, const auto& b) {
        return a.maximum_y_nm < b.maximum_y_nm;
      })->maximum_y_nm;
  const auto width = static_cast<std::size_t>((maximum_x - minimum_x) / cell_size_nm + 1);
  const auto height = static_cast<std::size_t>((maximum_y - minimum_y) / cell_size_nm + 1);
  constexpr std::size_t maximum_bins = 1'048'576U;
  if (width == 0U || height == 0U || width > maximum_bins / height) {
    result.diagnostic = "routing broadphase bin budget exhausted";
    result.budget_exhausted = true;
    return result;
  }
  std::vector<std::vector<std::uint32_t>> bins(width * height);
  for (std::size_t index = 0U; index < primitives.size(); ++index) {
    const auto& primitive = primitives[index];
    const auto x0 = static_cast<std::size_t>((primitive.minimum_x_nm - minimum_x) / cell_size_nm);
    const auto y0 = static_cast<std::size_t>((primitive.minimum_y_nm - minimum_y) / cell_size_nm);
    const auto x1 = static_cast<std::size_t>((primitive.maximum_x_nm - minimum_x) / cell_size_nm);
    const auto y1 = static_cast<std::size_t>((primitive.maximum_y_nm - minimum_y) / cell_size_nm);
    for (auto y = y0; y <= y1; ++y)
      for (auto x = x0; x <= x1; ++x)
        bins[y * width + x].push_back(static_cast<std::uint32_t>(index));
  }
  for (const auto& bin : bins) {
    if (bin.empty())
      continue;
    ++result.occupied_bins;
    for (std::size_t first = 0U; first < bin.size(); ++first) {
      for (std::size_t second = first + 1U; second < bin.size(); ++second) {
        const auto lhs = std::min(bin[first], bin[second]);
        const auto rhs = std::max(bin[first], bin[second]);
        const auto& a = primitives[lhs];
        const auto& b = primitives[rhs];
        if (a.layer != b.layer || a.owner == b.owner || a.maximum_x_nm < b.minimum_x_nm ||
            b.maximum_x_nm < a.minimum_x_nm || a.maximum_y_nm < b.minimum_y_nm ||
            b.maximum_y_nm < a.minimum_y_nm)
          continue;
        result.candidate_pairs.emplace_back(lhs, rhs);
      }
    }
  }
  std::sort(result.candidate_pairs.begin(), result.candidate_pairs.end());
  result.candidate_pairs.erase(
      std::unique(result.candidate_pairs.begin(), result.candidate_pairs.end()),
      result.candidate_pairs.end());
  if (result.candidate_pairs.size() > maximum_pairs) {
    result.candidate_pairs.resize(maximum_pairs);
    result.budget_exhausted = true;
  }
  return result;
}

RoutingComputeViaCandidateResult
filter_routing_via_candidates(const RoutingComputeViaCandidateInput& input, bool prefer_gpu) {
  if (input.candidates.empty() || input.top_k == 0U || input.clearance_nm < 0 ||
      input.numerical_guard_nm < 0)
    return RoutingComputeViaCandidateResult{{},    {},    0U,
                                            false, false, "invalid regional via filtering request"};
#if FLUX_HAS_VULKAN_HYDRAULIC_PATHS
  if (prefer_gpu && input.candidates.size() >= 64U && input.obstacles.size() >= 16U) {
    auto gpu = filter_routing_via_candidates_gpu(input);
    if (gpu.ok())
      return gpu;
    auto cpu = via_candidates_cpu(input);
    cpu.cpu_fallback = true;
    return cpu;
  }
#else
  static_cast<void>(prefer_gpu);
#endif
  return via_candidates_cpu(input);
}

RoutingComputeSplineBatchResult
evaluate_routing_spline_batch(const RoutingComputeSplineBatchInput& input, bool prefer_gpu) {
  if (input.control_points_nm.empty() || input.sample_count == 0U || input.candidate_count == 0U ||
      input.control_points_nm.size() % input.candidate_count != 0U ||
      input.sample_count > std::numeric_limits<std::size_t>::max() / input.candidate_count ||
      input.control_points_nm.size() >
          std::numeric_limits<std::size_t>::max() / input.sample_count ||
      input.basis_coefficients.size() != input.control_points_nm.size() * input.sample_count) {
    return RoutingComputeSplineBatchResult{
        {}, false, false, "invalid routing compute spline batch"};
  }
#if FLUX_HAS_VULKAN_HYDRAULIC_PATHS
  const auto total_samples = input.sample_count * input.candidate_count;
  if (prefer_gpu && total_samples >= 32U &&
      input.control_points_nm.size() <= std::numeric_limits<std::size_t>::max() / total_samples &&
      total_samples * (input.control_points_nm.size() / input.candidate_count) >= 512U) {
    auto gpu = evaluate_routing_spline_batch_gpu(input);
    if (gpu.ok())
      return gpu;
    auto cpu = spline_batch_cpu(input);
    cpu.cpu_fallback = true;
    return cpu;
  }
#else
  static_cast<void>(prefer_gpu);
#endif
  return spline_batch_cpu(input);
}

RoutingComputeSplineScoreResult
score_routing_spline_candidates(const RoutingComputeSplineScoreInput& input, bool prefer_gpu) {
  const auto field_count =
      input.field_width > 0U && input.field_height > 0U &&
              input.field_width <= std::numeric_limits<std::size_t>::max() / input.field_height
          ? input.field_width * input.field_height
          : 0U;
  if (input.top_k == 0U || input.field_pitch_nm <= 0 ||
      input.signed_clearance_nm.size() != field_count ||
      input.corridor_mask.size() != field_count) {
    return RoutingComputeSplineScoreResult{
        {}, 0U, false, false, "invalid routing spline score request"};
  }
#if FLUX_HAS_VULKAN_HYDRAULIC_PATHS
  if (prefer_gpu && input.candidate_count >= 16U) {
    auto gpu = score_routing_spline_candidates_gpu(input);
    if (gpu.ok())
      return gpu;
    auto cpu = spline_scores_cpu(input);
    cpu.cpu_fallback = true;
    return cpu;
  }
#else
  static_cast<void>(prefer_gpu);
#endif
  return spline_scores_cpu(input);
}

RoutingComputeWavefrontResult compute_routing_wavefront(const RoutingComputeWavefrontInput& input,
                                                        bool prefer_gpu) {
  const auto valid_count =
      input.width > 0U && input.height > 0U &&
              input.width <= std::numeric_limits<std::size_t>::max() / input.height
          ? input.width * input.height
          : 0U;
  if (valid_count == 0U || input.traversable.size() != valid_count || input.sources.empty() ||
      input.maximum_passes == 0U) {
    return RoutingComputeWavefrontResult{{},    0U,    false,
                                         false, false, "invalid routing compute wavefront"};
  }
#if FLUX_HAS_VULKAN_HYDRAULIC_PATHS
  if (prefer_gpu && valid_count >= 4'096U) {
    auto gpu = compute_routing_wavefront_gpu(input);
    if (gpu.ok())
      return gpu;
    auto cpu = wavefront_cpu(input);
    cpu.cpu_fallback = true;
    return cpu;
  }
#else
  static_cast<void>(prefer_gpu);
#endif
  return wavefront_cpu(input);
}

RoutingComputeWavefrontResult
compute_routing_graph_wavefront(const RoutingComputeGraphWavefrontInput& input, bool prefer_gpu) {
  const auto valid =
      input.node_count > 0U && input.node_count <= std::numeric_limits<std::uint32_t>::max() &&
      input.offsets.size() == input.node_count + 1U && !input.sources.empty() &&
      input.maximum_passes > 0U && input.offsets.front() == 0U &&
      input.offsets.back() == input.edges.size() &&
      std::is_sorted(input.offsets.begin(), input.offsets.end()) &&
      std::all_of(input.edges.begin(), input.edges.end(),
                  [&](const auto& edge) { return edge.node < input.node_count && edge.cost > 0U; });
  if (!valid) {
    return RoutingComputeWavefrontResult{{},    0U,    false,
                                         false, false, "invalid routing compute graph wavefront"};
  }
#if FLUX_HAS_VULKAN_HYDRAULIC_PATHS
  if (prefer_gpu && input.node_count >= 2'048U) {
    auto gpu_input = input;
    // One dispatch advances information by one graph edge. Bound a synchronous command buffer;
    // callers may use the exact CPU graph only when the requested target lies beyond this probe.
    const auto geometric_passes = static_cast<std::size_t>(
        std::ceil(8.0L * std::sqrt(static_cast<long double>(input.node_count))));
    gpu_input.maximum_passes =
        std::min<std::size_t>(input.maximum_passes, std::max<std::size_t>(256U, geometric_passes));
    auto gpu = compute_routing_graph_wavefront_gpu(gpu_input);
    if (gpu.ok())
      return gpu;
    auto cpu = graph_wavefront_cpu(input);
    cpu.cpu_fallback = true;
    return cpu;
  }
#else
  static_cast<void>(prefer_gpu);
#endif
  return graph_wavefront_cpu(input);
}

RoutingComputeClearanceFieldResult
compute_routing_clearance_field(std::size_t width, std::size_t height,
                                const std::vector<std::uint8_t>& legal_cells,
                                std::size_t maximum_passes, bool prefer_gpu) {
  RoutingComputeClearanceFieldResult result;
  if (width == 0U || height == 0U || width > std::numeric_limits<std::size_t>::max() / height ||
      legal_cells.size() != width * height || maximum_passes == 0U) {
    result.diagnostic = "invalid routing compute clearance field";
    return result;
  }
  const auto count = width * height;
  RoutingComputeWavefrontInput to_obstacle;
  to_obstacle.width = width;
  to_obstacle.height = height;
  to_obstacle.traversable.assign(count, 1U);
  to_obstacle.maximum_passes = maximum_passes;
  RoutingComputeWavefrontInput to_legal = to_obstacle;
  for (std::size_t cell = 0U; cell < count; ++cell) {
    (legal_cells[cell] != 0U ? to_legal.sources : to_obstacle.sources).push_back(cell);
  }
  if (to_obstacle.sources.empty() || to_legal.sources.empty()) {
    result.diagnostic = "routing compute clearance field needs legal and obstacle cells";
    return result;
  }
  const auto outside = compute_routing_wavefront(to_obstacle, prefer_gpu);
  const auto inside = compute_routing_wavefront(to_legal, prefer_gpu);
  if (!outside.ok() || !inside.ok()) {
    result.diagnostic = !outside.ok() ? outside.diagnostic : inside.diagnostic;
    return result;
  }
  result.signed_distance.resize(count, 0);
  for (std::size_t cell = 0U; cell < count; ++cell) {
    const auto distance = legal_cells[cell] != 0U ? outside.distance[cell] : inside.distance[cell];
    if (distance == unreachable ||
        distance > static_cast<std::uint32_t>(std::numeric_limits<std::int32_t>::max())) {
      result.signed_distance[cell] = legal_cells[cell] != 0U
                                         ? std::numeric_limits<std::int32_t>::max()
                                         : std::numeric_limits<std::int32_t>::min();
    } else {
      const auto signed_value = static_cast<std::int32_t>(distance);
      result.signed_distance[cell] = legal_cells[cell] != 0U ? signed_value : -signed_value;
    }
  }
  result.used_gpu = outside.used_gpu || inside.used_gpu;
  result.cpu_fallback = outside.cpu_fallback || inside.cpu_fallback;
  return result;
}

RoutingComputeLinearScores
score_routing_compute_candidates(const std::vector<RoutingComputeLinearCandidate>& candidates,
                                 const std::array<std::uint32_t, 8U>& weights, bool prefer_gpu) {
#if FLUX_HAS_VULKAN_HYDRAULIC_PATHS
  if (prefer_gpu && candidates.size() >= 32U) {
    auto gpu = score_routing_compute_candidates_gpu(candidates, weights);
    if (gpu.ok())
      return gpu;
    auto cpu = scores_cpu(candidates, weights);
    cpu.cpu_fallback = true;
    return cpu;
  }
#else
  static_cast<void>(prefer_gpu);
#endif
  return scores_cpu(candidates, weights);
}

} // namespace flux
