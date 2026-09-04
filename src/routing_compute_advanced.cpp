#include "flux/core/routing_compute.hpp"

#include <algorithm>
#include <bit>
#include <cmath>
#include <functional>
#include <limits>
#include <numeric>
#include <queue>

namespace flux {
namespace {

constexpr auto unreachable = std::numeric_limits<std::uint32_t>::max();

[[nodiscard]] std::int64_t quantize(const long double value, const std::int64_t quantum) {
  return static_cast<std::int64_t>(std::llround(value / static_cast<long double>(quantum))) *
         quantum;
}

[[nodiscard]] long double point_segment_distance(long double px, long double py, long double ax,
                                                 long double ay, long double bx, long double by) {
  const auto dx = bx - ax, dy = by - ay;
  const auto square = dx * dx + dy * dy;
  const auto t =
      square > 0.0L ? std::clamp(((px - ax) * dx + (py - ay) * dy) / square, 0.0L, 1.0L) : 0.0L;
  return std::hypotl(px - (ax + t * dx), py - (ay + t * dy));
}

[[nodiscard]] long double segment_distance(const RoutingComputeSegment& a,
                                           const RoutingComputeSegment& b) {
  const auto orient = [](long double ax, long double ay, long double bx, long double by,
                         long double cx,
                         long double cy) { return (bx - ax) * (cy - ay) - (by - ay) * (cx - ax); };
  const auto o1 =
      orient(a.first_x_nm, a.first_y_nm, a.second_x_nm, a.second_y_nm, b.first_x_nm, b.first_y_nm);
  const auto o2 = orient(a.first_x_nm, a.first_y_nm, a.second_x_nm, a.second_y_nm, b.second_x_nm,
                         b.second_y_nm);
  const auto o3 =
      orient(b.first_x_nm, b.first_y_nm, b.second_x_nm, b.second_y_nm, a.first_x_nm, a.first_y_nm);
  const auto o4 = orient(b.first_x_nm, b.first_y_nm, b.second_x_nm, b.second_y_nm, a.second_x_nm,
                         a.second_y_nm);
  if (((o1 <= 0.0L && o2 >= 0.0L) || (o1 >= 0.0L && o2 <= 0.0L)) &&
      ((o3 <= 0.0L && o4 >= 0.0L) || (o3 >= 0.0L && o4 <= 0.0L)))
    return 0.0L;
  return std::min({point_segment_distance(a.first_x_nm, a.first_y_nm, b.first_x_nm, b.first_y_nm,
                                          b.second_x_nm, b.second_y_nm),
                   point_segment_distance(a.second_x_nm, a.second_y_nm, b.first_x_nm, b.first_y_nm,
                                          b.second_x_nm, b.second_y_nm),
                   point_segment_distance(b.first_x_nm, b.first_y_nm, a.first_x_nm, a.first_y_nm,
                                          a.second_x_nm, a.second_y_nm),
                   point_segment_distance(b.second_x_nm, b.second_y_nm, a.first_x_nm, a.first_y_nm,
                                          a.second_x_nm, a.second_y_nm)});
}

[[nodiscard]] RoutingComputeQuantizedCandidateResult
quantized_candidates_cpu(const RoutingComputeQuantizedCandidateInput& input) {
  RoutingComputeQuantizedCandidateResult result;
  std::vector<RoutingComputeQuantizedCandidateScore> ranking;
  ranking.reserve(input.candidate_count);
  for (std::size_t candidate = 0; candidate < input.candidate_count; ++candidate) {
    RoutingComputeQuantizedCandidateScore score;
    score.candidate = candidate;
    std::vector<RoutingComputeSegment> segments;
    for (const auto& route : input.routes) {
      const auto base = candidate * input.points_per_candidate + route.point_begin;
      for (std::size_t point = 1; point < route.point_count; ++point) {
        const auto& a = input.candidate_points_nm[base + point - 1U];
        const auto& b = input.candidate_points_nm[base + point];
        RoutingComputeSegment segment{quantize(a.first, input.quantization_nm),
                                      quantize(a.second, input.quantization_nm),
                                      quantize(b.first, input.quantization_nm),
                                      quantize(b.second, input.quantization_nm),
                                      route.radius_nm,
                                      route.layer,
                                      route.owner};
        score.length_nm += static_cast<std::uint64_t>(std::llround(
            std::hypotl(static_cast<long double>(segment.second_x_nm - segment.first_x_nm),
                        static_cast<long double>(segment.second_y_nm - segment.first_y_nm))));
        segments.push_back(segment);
      }
    }
    for (std::size_t first = 0; first < segments.size(); ++first) {
      for (std::size_t second = first + 1U; second < segments.size(); ++second) {
        if (segments[first].layer != segments[second].layer ||
            segments[first].owner == segments[second].owner)
          continue;
        const auto distance = segment_distance(segments[first], segments[second]);
        const auto required = static_cast<long double>(segments[first].radius_nm) +
                              segments[second].radius_nm + input.clearance_nm +
                              input.numerical_guard_nm;
        if (distance < required) {
          ++score.clearance_violations;
          if (distance <= 0.0L)
            ++score.crossings;
          score.total_deficit_nm += static_cast<std::uint64_t>(std::ceil(required - distance));
        }
      }
    }
    ranking.push_back(score);
  }
  std::stable_sort(ranking.begin(), ranking.end(), [](const auto& a, const auto& b) {
    if (a.clearance_violations != b.clearance_violations)
      return a.clearance_violations < b.clearance_violations;
    if (a.crossings != b.crossings)
      return a.crossings < b.crossings;
    if (a.total_deficit_nm != b.total_deficit_nm)
      return a.total_deficit_nm < b.total_deficit_nm;
    if (a.length_nm != b.length_nm)
      return a.length_nm < b.length_nm;
    return a.candidate < b.candidate;
  });
  result.assessed_candidates = ranking.size();
  ranking.resize(std::min(input.top_k, ranking.size()));
  result.shortlist = std::move(ranking);
  return result;
}

[[nodiscard]] RoutingComputeAssignmentResult
assignments_cpu(const RoutingComputeAssignmentInput& input) {
  RoutingComputeAssignmentResult result;
  std::size_t states = 1U;
  for (const auto& domain : input.variable_domains) {
    if (domain.empty())
      return result;
    if (states > input.maximum_states / domain.size()) {
      states = input.maximum_states;
      result.state_space_truncated = true;
      break;
    }
    states *= domain.size();
  }
  states = std::min(states, input.maximum_states);
  for (std::size_t dense = 0; dense < states; ++dense) {
    auto code = dense;
    RoutingComputeAssignmentCandidate candidate;
    candidate.dense_state = dense;
    candidate.options.resize(input.variable_domains.size());
    for (std::size_t variable = 0; variable < input.variable_domains.size(); ++variable) {
      const auto& domain = input.variable_domains[variable];
      candidate.options[variable] = domain[code % domain.size()];
      code /= domain.size();
      if (candidate.options[variable] < input.option_costs.size())
        candidate.score += input.option_costs[candidate.options[variable]];
    }
    bool legal = true;
    for (const auto [first, second] : input.incompatible_options) {
      const auto has_first = std::find(candidate.options.begin(), candidate.options.end(), first) !=
                             candidate.options.end();
      const auto has_second = std::find(candidate.options.begin(), candidate.options.end(),
                                        second) != candidate.options.end();
      if (has_first && has_second) {
        legal = false;
        break;
      }
    }
    if (legal)
      result.shortlist.push_back(std::move(candidate));
  }
  result.assessed_states = states;
  std::stable_sort(result.shortlist.begin(), result.shortlist.end(),
                   [](const auto& a, const auto& b) {
                     return a.score != b.score ? a.score < b.score : a.dense_state < b.dense_state;
                   });
  result.shortlist.resize(std::min(input.top_k, result.shortlist.size()));
  return result;
}

[[nodiscard]] RoutingComputeArcCandidateResult
arc_candidates_cpu(const std::vector<RoutingComputeArcCandidate>& candidates,
                   const std::vector<RoutingComputeSegment>& obstacles, std::int64_t clearance,
                   std::size_t top_k) {
  RoutingComputeArcCandidateResult result;
  for (std::size_t index = 0U; index < candidates.size(); ++index) {
    const auto& candidate = candidates[index];
    RoutingComputeArcCandidateScore score;
    score.candidate = index;
    const auto& p = candidate.points_nm;
    const auto tangent_error = [](const auto& a, const auto& b, const auto& c) {
      const auto ux = static_cast<long double>(b.first - a.first);
      const auto uy = static_cast<long double>(b.second - a.second);
      const auto vx = static_cast<long double>(c.first - b.first);
      const auto vy = static_cast<long double>(c.second - b.second);
      return std::abs(std::atan2(ux * vy - uy * vx, ux * vx + uy * vy));
    };
    score.tangent_error = tangent_error(p[0], p[1], p[2]) + tangent_error(p[2], p[3], p[4]);
    const std::array<RoutingComputeSegment, 2U> chords{
        RoutingComputeSegment{p[1].first, p[1].second, p[2].first, p[2].second,
                              candidate.copper_radius_nm, candidate.layer, candidate.owner},
        RoutingComputeSegment{p[2].first, p[2].second, p[3].first, p[3].second,
                              candidate.copper_radius_nm, candidate.layer, candidate.owner}};
    score.approximate_length_nm = static_cast<std::uint64_t>(
        std::llround(std::hypotl(static_cast<long double>(p[2].first - p[1].first),
                                 static_cast<long double>(p[2].second - p[1].second)) +
                     std::hypotl(static_cast<long double>(p[3].first - p[2].first),
                                 static_cast<long double>(p[3].second - p[2].second))));
    for (const auto& chord : chords)
      for (const auto& obstacle : obstacles)
        if (chord.layer == obstacle.layer && chord.owner != obstacle.owner &&
            segment_distance(chord, obstacle) < chord.radius_nm + obstacle.radius_nm + clearance)
          ++score.approximate_conflicts;
    result.shortlist.push_back(score);
  }
  std::stable_sort(result.shortlist.begin(), result.shortlist.end(),
                   [](const auto& a, const auto& b) {
                     if (a.approximate_conflicts != b.approximate_conflicts)
                       return a.approximate_conflicts < b.approximate_conflicts;
                     if (a.tangent_error != b.tangent_error)
                       return a.tangent_error < b.tangent_error;
                     return a.approximate_length_nm != b.approximate_length_nm
                                ? a.approximate_length_nm < b.approximate_length_nm
                                : a.candidate < b.candidate;
                   });
  result.shortlist.resize(std::min(top_k, result.shortlist.size()));
  return result;
}

[[nodiscard]] RoutingComputeLaneOrderResult lane_order_cpu(const RoutingComputeLaneOrderInput& in) {
  RoutingComputeLaneOrderResult result;
  const auto count = in.widths_nm.size();
  if (count == 0U || count > 12U)
    return result;
  const auto states = std::size_t{1U} << count;
  const auto slots = states * count;
  std::vector<std::uint64_t> widths(slots, std::numeric_limits<std::uint64_t>::max());
  std::vector<std::uint32_t> inversions(slots, std::numeric_limits<std::uint32_t>::max());
  std::vector<std::int32_t> parent(slots, -1);
  std::uint64_t coupled_mask = 0U;
  for (std::size_t i = 0; i < count; ++i)
    if (in.coupled_rank[i] != std::numeric_limits<std::uint32_t>::max())
      coupled_mask |= std::uint64_t{1U} << i;
  const auto coupled_legal = [&](std::uint64_t mask, std::size_t next) {
    return in.coupled_rank[next] == std::numeric_limits<std::uint32_t>::max() ||
           in.coupled_rank[next] == static_cast<std::uint32_t>(std::popcount(mask & coupled_mask));
  };
  for (std::size_t i = 0; i < count; ++i) {
    if (!coupled_legal(0U, i))
      continue;
    const auto slot = (std::size_t{1U} << i) * count + i;
    widths[slot] = static_cast<std::uint64_t>(in.clearances_nm[i]) + in.widths_nm[i];
    inversions[slot] = 0U;
  }
  for (std::size_t mask = 1U; mask < states; ++mask) {
    for (std::size_t last = 0; last < count; ++last) {
      const auto slot = mask * count + last;
      if (widths[slot] == std::numeric_limits<std::uint64_t>::max())
        continue;
      ++result.assessed_states;
      for (std::size_t next = 0; next < count; ++next) {
        if ((mask & (std::size_t{1U} << next)) != 0U || !coupled_legal(mask, next))
          continue;
        const auto next_mask = mask | (std::size_t{1U} << next);
        const auto next_slot = next_mask * count + next;
        const auto width = widths[slot] + std::max(in.clearances_nm[last], in.clearances_nm[next]) +
                           in.widths_nm[next];
        std::uint32_t added = 0U;
        for (std::size_t prior = 0; prior < count; ++prior)
          if ((mask & (std::size_t{1U} << prior)) != 0U &&
              in.reference_position[prior] > in.reference_position[next])
            ++added;
        const auto inv = inversions[slot] + added;
        if (width < widths[next_slot] ||
            (width == widths[next_slot] && inv < inversions[next_slot])) {
          widths[next_slot] = width;
          inversions[next_slot] = inv;
          parent[next_slot] = static_cast<std::int32_t>(last);
        }
      }
    }
  }
  const auto full = states - 1U;
  std::size_t best = count;
  for (std::size_t last = 0; last < count; ++last) {
    const auto slot = full * count + last;
    const auto final_width = widths[slot] == std::numeric_limits<std::uint64_t>::max()
                                 ? widths[slot]
                                 : widths[slot] + in.clearances_nm[last];
    if (final_width <= in.usable_width_nm &&
        (best == count || inversions[slot] < inversions[full * count + best] ||
         (inversions[slot] == inversions[full * count + best] &&
          final_width < result.packed_width_nm))) {
      best = last;
      result.packed_width_nm = final_width;
    }
  }
  if (best == count)
    return result;
  result.physically_fits = true;
  result.inversions = inversions[full * count + best];
  auto mask = full;
  auto last = best;
  while (mask != 0U) {
    result.order.push_back(static_cast<std::uint32_t>(last));
    const auto prior = parent[mask * count + last];
    mask &= ~(std::size_t{1U} << last);
    if (prior < 0)
      break;
    last = static_cast<std::size_t>(prior);
  }
  std::reverse(result.order.begin(), result.order.end());
  return result;
}

[[nodiscard]] RoutingComputeOrientationResult
orientation_cpu(const RoutingComputeOrientationInput& in) {
  RoutingComputeOrientationResult result;
  if (in.bit_count > 20U)
    return result;
  const auto states = std::size_t{1U} << in.bit_count;
  std::vector<std::pair<std::uint64_t, std::uint32_t>> valid;
  for (std::uint32_t state = 0; state < states; ++state) {
    bool legal = true;
    for (const auto [mask, value] : in.forbidden_clauses)
      if ((state & mask) == value) {
        legal = false;
        break;
      }
    if (!legal)
      continue;
    std::uint64_t score = 0U;
    for (std::size_t bit = 0; bit < in.bit_count; ++bit)
      if ((state & (1U << bit)) != 0U && bit < in.bit_flip_costs.size())
        score += in.bit_flip_costs[bit];
    valid.emplace_back(score, state);
  }
  result.assessed_states = states;
  std::stable_sort(valid.begin(), valid.end());
  valid.resize(std::min(in.top_k, valid.size()));
  for (const auto& [score, state] : valid) {
    static_cast<void>(score);
    result.states.push_back(state);
  }
  return result;
}

[[nodiscard]] RoutingComputeCopperNarrowphaseResult
copper_cpu(const std::vector<RoutingComputeCopperPrimitive>& primitives,
           const std::vector<std::pair<std::uint32_t, std::uint32_t>>& pairs,
           std::int64_t clearance, std::int64_t guard) {
  RoutingComputeCopperNarrowphaseResult result;
  result.pairs.resize(pairs.size());
  for (std::size_t index = 0; index < pairs.size(); ++index) {
    const auto [ia, ib] = pairs[index];
    if (ia >= primitives.size() || ib >= primitives.size()) {
      result.diagnostic = "simple-copper pair is out of range";
      return result;
    }
    const auto& a = primitives[ia];
    const auto& b = primitives[ib];
    if (a.layer != b.layer || a.owner == b.owner)
      continue;
    long double distance = 0.0L;
    if (a.kind == RoutingComputeCopperKind::circle && b.kind == RoutingComputeCopperKind::circle)
      distance = std::hypotl(static_cast<long double>(a.first_x_nm - b.first_x_nm),
                             static_cast<long double>(a.first_y_nm - b.first_y_nm));
    else if (a.kind == RoutingComputeCopperKind::circle)
      distance = point_segment_distance(a.first_x_nm, a.first_y_nm, b.first_x_nm, b.first_y_nm,
                                        b.second_x_nm, b.second_y_nm);
    else if (b.kind == RoutingComputeCopperKind::circle)
      distance = point_segment_distance(b.first_x_nm, b.first_y_nm, a.first_x_nm, a.first_y_nm,
                                        a.second_x_nm, a.second_y_nm);
    else
      distance = segment_distance(
          {a.first_x_nm, a.first_y_nm, a.second_x_nm, a.second_y_nm, a.radius_nm, a.layer, a.owner},
          {b.first_x_nm, b.first_y_nm, b.second_x_nm, b.second_y_nm, b.radius_nm, b.layer,
           b.owner});
    const auto required = static_cast<long double>(a.radius_nm + b.radius_nm + clearance + guard);
    if (distance < required) {
      auto& out = result.pairs[index];
      out.possible_conflict = 1U;
      out.approximate_deficit_nm = static_cast<std::int64_t>(std::ceil(required - distance));
      long double dx = static_cast<long double>(a.first_x_nm - b.first_x_nm);
      long double dy = static_cast<long double>(a.first_y_nm - b.first_y_nm);
      const auto norm = std::hypotl(dx, dy);
      if (norm > 0.0L) {
        out.normal_x = dx / norm;
        out.normal_y = dy / norm;
      }
    }
  }
  return result;
}

} // namespace

#if FLUX_HAS_VULKAN_HYDRAULIC_PATHS
RoutingComputeQuantizedCandidateResult
score_quantized_routing_candidates_gpu(const RoutingComputeQuantizedCandidateInput&);
RoutingComputeAssignmentResult
evaluate_routing_assignments_gpu(const RoutingComputeAssignmentInput&);
RoutingComputeLaneOrderResult solve_routing_lane_order_gpu(const RoutingComputeLaneOrderInput&);
RoutingComputeOrientationResult
evaluate_routing_orientations_gpu(const RoutingComputeOrientationInput&);
RoutingComputeCopperNarrowphaseResult
assess_simple_copper_pairs_gpu(const std::vector<RoutingComputeCopperPrimitive>&,
                               const std::vector<std::pair<std::uint32_t, std::uint32_t>>&,
                               std::int64_t, std::int64_t);
RoutingComputeBatchedGraphWavefrontResult
compute_batched_routing_graph_wavefront_gpu(const RoutingComputeBatchedGraphWavefrontInput&);
RoutingComputeNegotiatedResourceResult
reduce_negotiated_routing_resources_gpu(const std::vector<RoutingComputeNegotiatedResource>&);
RoutingComputeArcCandidateResult
score_routing_arc_candidates_gpu(const std::vector<RoutingComputeArcCandidate>&,
                                 const std::vector<RoutingComputeSegment>&, std::int64_t,
                                 std::size_t);
#endif

RoutingComputeQuantizedCandidateResult
score_quantized_routing_candidates(const RoutingComputeQuantizedCandidateInput& input,
                                   bool prefer_gpu) {
  if (input.candidate_count == 0U || input.points_per_candidate == 0U || input.top_k == 0U ||
      input.quantization_nm <= 0 || input.clearance_nm < 0 || input.numerical_guard_nm < 0 ||
      input.candidate_points_nm.size() != input.candidate_count * input.points_per_candidate)
    return {{}, 0U, false, false, "invalid quantized routing candidate batch"};
#if FLUX_HAS_VULKAN_HYDRAULIC_PATHS
  if (prefer_gpu && input.candidate_count >= 16U) {
    auto gpu = score_quantized_routing_candidates_gpu(input);
    if (gpu.ok())
      return gpu;
    auto cpu = quantized_candidates_cpu(input);
    cpu.cpu_fallback = true;
    return cpu;
  }
#else
  static_cast<void>(prefer_gpu);
#endif
  return quantized_candidates_cpu(input);
}

RoutingComputeArcCandidateResult
score_routing_arc_candidates(const std::vector<RoutingComputeArcCandidate>& candidates,
                             const std::vector<RoutingComputeSegment>& obstacles,
                             std::int64_t clearance, std::size_t top_k, bool prefer_gpu) {
  if (candidates.empty() || top_k == 0U || clearance < 0)
    return {{}, false, false, "invalid circular-arc candidate batch"};
#if FLUX_HAS_VULKAN_HYDRAULIC_PATHS
  if (prefer_gpu && candidates.size() >= 4U) {
    auto gpu = score_routing_arc_candidates_gpu(candidates, obstacles, clearance, top_k);
    if (gpu.ok())
      return gpu;
    auto cpu = arc_candidates_cpu(candidates, obstacles, clearance, top_k);
    cpu.cpu_fallback = true;
    return cpu;
  }
#else
  static_cast<void>(prefer_gpu);
#endif
  return arc_candidates_cpu(candidates, obstacles, clearance, top_k);
}

RoutingComputeAssignmentResult evaluate_routing_assignments(const RoutingComputeAssignmentInput& in,
                                                             bool prefer_gpu) {
  if (in.variable_domains.empty() || in.maximum_states == 0U || in.top_k == 0U)
    return {{}, 0U, false, false, false, "invalid regional assignment batch"};
  std::size_t dense_states = 1U;
  for (const auto& domain : in.variable_domains) {
    if (domain.empty())
      return {{}, 0U, false, false, false, "empty regional assignment domain"};
    if (dense_states > in.maximum_states / domain.size()) {
      dense_states = in.maximum_states;
      break;
    }
    dense_states *= domain.size();
  }
#if FLUX_HAS_VULKAN_HYDRAULIC_PATHS
  if (prefer_gpu && dense_states >= 4096U) {
    auto gpu = evaluate_routing_assignments_gpu(in);
    if (gpu.ok())
      return gpu;
    auto cpu = assignments_cpu(in);
    cpu.cpu_fallback = true;
    return cpu;
  }
#else
  static_cast<void>(prefer_gpu);
#endif
  return assignments_cpu(in);
}

RoutingComputeLaneOrderResult solve_routing_lane_order(const RoutingComputeLaneOrderInput& in,
                                                       bool prefer_gpu) {
  if (in.widths_nm.empty() || in.widths_nm.size() > 12U ||
      in.clearances_nm.size() != in.widths_nm.size() ||
      in.reference_position.size() != in.widths_nm.size() ||
      in.coupled_rank.size() != in.widths_nm.size() || in.usable_width_nm == 0U)
    return {{}, 0U, 0U, 0U, false, false, false, "invalid lane-order subset DP"};
#if FLUX_HAS_VULKAN_HYDRAULIC_PATHS
  if (prefer_gpu && in.widths_nm.size() >= 7U) {
    auto gpu = solve_routing_lane_order_gpu(in);
    if (gpu.ok())
      return gpu;
    auto cpu = lane_order_cpu(in);
    cpu.cpu_fallback = true;
    return cpu;
  }
#else
  static_cast<void>(prefer_gpu);
#endif
  return lane_order_cpu(in);
}

RoutingComputeOrientationResult
evaluate_routing_orientations(const RoutingComputeOrientationInput& in, bool prefer_gpu) {
  if (in.bit_count > 20U || in.top_k == 0U)
    return {{}, 0U, false, false, "invalid orientation enumeration"};
#if FLUX_HAS_VULKAN_HYDRAULIC_PATHS
  if (prefer_gpu && in.bit_count >= 8U) {
    auto gpu = evaluate_routing_orientations_gpu(in);
    if (gpu.ok())
      return gpu;
    auto cpu = orientation_cpu(in);
    cpu.cpu_fallback = true;
    return cpu;
  }
#else
  static_cast<void>(prefer_gpu);
#endif
  return orientation_cpu(in);
}

RoutingComputeCopperNarrowphaseResult
assess_simple_copper_pairs(const std::vector<RoutingComputeCopperPrimitive>& primitives,
                           const std::vector<std::pair<std::uint32_t, std::uint32_t>>& pairs,
                           std::int64_t clearance, std::int64_t guard, bool prefer_gpu) {
  if (clearance < 0 || guard < 0)
    return {{}, false, false, "invalid simple-copper clearance"};
#if FLUX_HAS_VULKAN_HYDRAULIC_PATHS
  if (prefer_gpu && pairs.size() >= 256U) {
    auto gpu = assess_simple_copper_pairs_gpu(primitives, pairs, clearance, guard);
    if (gpu.ok())
      return gpu;
    auto cpu = copper_cpu(primitives, pairs, clearance, guard);
    cpu.cpu_fallback = true;
    return cpu;
  }
#else
  static_cast<void>(prefer_gpu);
#endif
  return copper_cpu(primitives, pairs, clearance, guard);
}

RoutingComputeBatchedGraphWavefrontResult
compute_batched_routing_graph_wavefront(const RoutingComputeBatchedGraphWavefrontInput& in,
                                        bool prefer_gpu) {
  RoutingComputeBatchedGraphWavefrontResult result;
  if (in.node_count == 0U || in.offsets.size() != in.node_count + 1U || in.source_groups.empty() ||
      in.maximum_passes == 0U) {
    result.diagnostic = "invalid batched graph wavefront";
    return result;
  }
#if FLUX_HAS_VULKAN_HYDRAULIC_PATHS
  if (prefer_gpu && in.node_count * in.source_groups.size() >= 4096U) {
    auto gpu = compute_batched_routing_graph_wavefront_gpu(in);
    if (gpu.ok())
      return gpu;
    result.cpu_fallback = true;
  }
#else
  static_cast<void>(prefer_gpu);
#endif
  result.distance.reserve(in.source_groups.size());
  for (const auto& sources : in.source_groups) {
    RoutingComputeGraphWavefrontInput one{in.node_count, in.offsets, in.edges, sources,
                                          in.maximum_passes};
    const auto solved = compute_routing_graph_wavefront(one, false);
    if (!solved.ok()) {
      result.diagnostic = solved.diagnostic;
      result.distance.clear();
      return result;
    }
    result.distance.push_back(solved.distance);
    result.passes += solved.passes;
    result.budget_exhausted = result.budget_exhausted || solved.budget_exhausted;
  }
  return result;
}

RoutingComputeNegotiatedResourceResult
reduce_negotiated_routing_resources(const std::vector<RoutingComputeNegotiatedResource>& resources,
                                    bool prefer_gpu) {
  RoutingComputeNegotiatedResourceResult result;
#if FLUX_HAS_VULKAN_HYDRAULIC_PATHS
  if (prefer_gpu && resources.size() >= 256U) {
    auto gpu = reduce_negotiated_routing_resources_gpu(resources);
    if (gpu.ok())
      return gpu;
    result.cpu_fallback = true;
  }
#else
  static_cast<void>(prefer_gpu);
#endif
  result.overflow.resize(resources.size(), 0U);
  result.updated_history.resize(resources.size(), 0U);
  for (std::size_t index = 0U; index < resources.size(); ++index) {
    const auto& resource = resources[index];
    const auto overflow =
        resource.demand > resource.capacity ? resource.demand - resource.capacity : 0U;
    result.overflow[index] = overflow;
    const auto increment = static_cast<std::uint64_t>(overflow) * resource.history_scale;
    result.updated_history[index] = static_cast<std::uint32_t>(std::min<std::uint64_t>(
        std::numeric_limits<std::uint32_t>::max(), resource.history + increment));
    result.total_overflow += overflow;
    result.peak_overflow = std::max(result.peak_overflow, overflow);
  }
  return result;
}

} // namespace flux
