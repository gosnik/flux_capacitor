#include "flux/core/routing_compute.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <numeric>
#include <optional>
#include <utility>
#include <vector>

namespace flux {

#if FLUX_HAS_VULKAN_HYDRAULIC_PATHS
[[nodiscard]] RoutingXpbdResult
relax_routing_xpbd_gpu(const std::vector<RoutingXpbdParticle>& particles,
                       const std::vector<RoutingXpbdSegment>& segments,
                       const std::vector<RoutingXpbdObstacleSegment>& obstacles,
                       const RoutingXpbdTerrainField& terrain, const RoutingXpbdConfig& config);
#endif

namespace {

using SteadyClock = std::chrono::steady_clock;

[[nodiscard]] std::uint64_t elapsed_us(SteadyClock::time_point start,
                                       SteadyClock::time_point finish) {
  return static_cast<std::uint64_t>(
      std::chrono::duration_cast<std::chrono::microseconds>(finish - start).count());
}

struct Vec2 {
  long double x{0.0L};
  long double y{0.0L};
};

[[nodiscard]] Vec2 operator+(Vec2 first, Vec2 second) {
  return {first.x + second.x, first.y + second.y};
}
[[nodiscard]] Vec2 operator-(Vec2 first, Vec2 second) {
  return {first.x - second.x, first.y - second.y};
}
[[nodiscard]] Vec2 operator*(Vec2 value, long double scale) {
  return {value.x * scale, value.y * scale};
}
Vec2& operator+=(Vec2& first, Vec2 second) {
  first.x += second.x;
  first.y += second.y;
  return first;
}
[[nodiscard]] long double dot(Vec2 first, Vec2 second) {
  return first.x * second.x + first.y * second.y;
}
[[nodiscard]] long double length(Vec2 value) { return std::hypotl(value.x, value.y); }
[[nodiscard]] Vec2 normalized(Vec2 value, Vec2 fallback = {1.0L, 0.0L}) {
  const auto magnitude = length(value);
  return magnitude > 1.0e-12L ? value * (1.0L / magnitude) : fallback;
}

struct ClosestSegments {
  long double first_parameter{0.0L};
  long double second_parameter{0.0L};
  Vec2 first;
  Vec2 second;
  long double distance{0.0L};
};

[[nodiscard]] ClosestSegments closest_segments(Vec2 p1, Vec2 q1, Vec2 p2, Vec2 q2) {
  // Ericson's bounded closest-points construction, specialized to 2D.
  constexpr long double epsilon = 1.0e-18L;
  const auto d1 = q1 - p1;
  const auto d2 = q2 - p2;
  const auto r = p1 - p2;
  const auto a = dot(d1, d1);
  const auto e = dot(d2, d2);
  const auto f = dot(d2, r);
  long double s = 0.0L;
  long double t = 0.0L;
  if (a <= epsilon && e <= epsilon) {
    return {0.0L, 0.0L, p1, p2, length(p1 - p2)};
  }
  if (a <= epsilon) {
    t = std::clamp(f / e, 0.0L, 1.0L);
  } else {
    const auto c = dot(d1, r);
    if (e <= epsilon) {
      s = std::clamp(-c / a, 0.0L, 1.0L);
    } else {
      const auto b = dot(d1, d2);
      const auto denominator = a * e - b * b;
      if (std::abs(denominator) > epsilon) {
        s = std::clamp((b * f - c * e) / denominator, 0.0L, 1.0L);
      }
      t = (b * s + f) / e;
      if (t < 0.0L) {
        t = 0.0L;
        s = std::clamp(-c / a, 0.0L, 1.0L);
      } else if (t > 1.0L) {
        t = 1.0L;
        s = std::clamp((b - c) / a, 0.0L, 1.0L);
      }
    }
  }
  const auto first = p1 + d1 * s;
  const auto second = p2 + d2 * t;
  return {s, t, first, second, length(first - second)};
}

[[nodiscard]] long double compliance_weight(long double compliance) {
  if (!(compliance > 0.0L))
    return 1.0L;
  return 1.0L / (1.0L + compliance * 10'000.0L);
}

struct CandidatePair {
  std::uint32_t first{0U};
  std::uint32_t second{0U};
};

[[nodiscard]] std::int64_t particle_motion_bound(const RoutingXpbdParticle& particle,
                                                 const RoutingXpbdConfig& config) {
  if (!(particle.inverse_mass > 0.0L))
    return 0;
  const auto iterative =
      static_cast<long double>(std::max<std::int64_t>(0, config.maximum_step_nm)) *
      static_cast<long double>(config.iterations);
  const auto corridor = particle.corridor_radius_nm > 0
                            ? static_cast<long double>(particle.corridor_radius_nm)
                            : iterative;
  return static_cast<std::int64_t>(
      std::min<long double>(std::min(iterative, corridor),
                            static_cast<long double>(std::numeric_limits<std::int64_t>::max())));
}

[[nodiscard]] std::vector<CandidatePair>
build_candidate_pairs(const std::vector<RoutingXpbdParticle>& particles,
                      const std::vector<RoutingXpbdSegment>& segments,
                      const RoutingXpbdConfig& config, bool& exhausted,
                      std::size_t& occupied_cells, std::size_t& pair_emissions) {
  occupied_cells = 0U;
  pair_emissions = 0U;
  exhausted = false;
  struct ExpandedBounds {
    long double minimum_x{0.0L};
    long double minimum_y{0.0L};
    long double maximum_x{0.0L};
    long double maximum_y{0.0L};
  };
  std::vector<ExpandedBounds> bounds;
  bounds.reserve(segments.size());
  auto domain_minimum_x = std::numeric_limits<long double>::max();
  auto domain_minimum_y = std::numeric_limits<long double>::max();
  auto domain_maximum_x = std::numeric_limits<long double>::lowest();
  auto domain_maximum_y = std::numeric_limits<long double>::lowest();
  auto maximum_extent = 1.0L;
  for (const auto& segment : segments) {
    if (segment.first_particle >= particles.size() || segment.second_particle >= particles.size()) {
      bounds.push_back({});
      continue;
    }
    const auto motion = std::max(particle_motion_bound(particles[segment.first_particle], config),
                                 particle_motion_bound(particles[segment.second_particle], config));
    const auto expansion = static_cast<long double>(
        std::max<std::int64_t>(0, segment.radius_nm) +
        std::max<std::int64_t>(0, segment.clearance_nm) + motion +
        std::max<std::int64_t>(0, config.numerical_guard_nm));
    ExpandedBounds expanded{
        static_cast<long double>(std::min(particles[segment.first_particle].x_nm,
                                          particles[segment.second_particle].x_nm)) -
            expansion,
        static_cast<long double>(std::min(particles[segment.first_particle].y_nm,
                                          particles[segment.second_particle].y_nm)) -
            expansion,
        static_cast<long double>(std::max(particles[segment.first_particle].x_nm,
                                          particles[segment.second_particle].x_nm)) +
            expansion,
        static_cast<long double>(std::max(particles[segment.first_particle].y_nm,
                                          particles[segment.second_particle].y_nm)) +
            expansion};
    domain_minimum_x = std::min(domain_minimum_x, expanded.minimum_x);
    domain_minimum_y = std::min(domain_minimum_y, expanded.minimum_y);
    domain_maximum_x = std::max(domain_maximum_x, expanded.maximum_x);
    domain_maximum_y = std::max(domain_maximum_y, expanded.maximum_y);
    maximum_extent = std::max(maximum_extent, std::max(expanded.maximum_x - expanded.minimum_x,
                                                       expanded.maximum_y - expanded.minimum_y));
    bounds.push_back(expanded);
  }
  if (segments.empty() || !std::isfinite(domain_minimum_x) || !std::isfinite(domain_minimum_y))
    return {};
  constexpr std::size_t maximum_dimension = 128U;
  auto pitch = std::max<long double>(1.0L, std::ceil(maximum_extent / 4.0L));
  const auto width_nm = std::max<long double>(1.0L, domain_maximum_x - domain_minimum_x);
  const auto height_nm = std::max<long double>(1.0L, domain_maximum_y - domain_minimum_y);
  pitch = std::max(pitch, std::ceil(std::max(width_nm, height_nm) /
                                    static_cast<long double>(maximum_dimension - 1U)));
  const auto dimension = [&](long double extent) {
    return static_cast<std::size_t>(std::clamp<long double>(
        std::ceil(extent / pitch) + 1.0L, 1.0L, static_cast<long double>(maximum_dimension)));
  };
  const auto width = dimension(width_nm);
  const auto height = dimension(height_nm);
  std::vector<std::vector<std::uint32_t>> bins(width * height);
  for (std::size_t segment = 0U; segment < segments.size(); ++segment) {
    const auto cell = [&](long double coordinate, long double origin, std::size_t limit) {
      return static_cast<std::size_t>(std::clamp<long double>(
          std::floor((coordinate - origin) / pitch), 0.0L,
          static_cast<long double>(limit - 1U)));
    };
    const auto minimum_x = cell(bounds[segment].minimum_x, domain_minimum_x, width);
    const auto maximum_x = cell(bounds[segment].maximum_x, domain_minimum_x, width);
    const auto minimum_y = cell(bounds[segment].minimum_y, domain_minimum_y, height);
    const auto maximum_y = cell(bounds[segment].maximum_y, domain_minimum_y, height);
    for (auto y = minimum_y; y <= maximum_y; ++y) {
      for (auto x = minimum_x; x <= maximum_x; ++x)
        bins[y * width + x].push_back(static_cast<std::uint32_t>(segment));
    }
  }
  std::vector<std::uint64_t> keys;
  const auto emission_limit =
      config.maximum_candidate_pairs > std::numeric_limits<std::size_t>::max() / 16U
          ? std::numeric_limits<std::size_t>::max()
          : config.maximum_candidate_pairs * 16U;
  for (const auto& bin : bins) {
    if (bin.empty())
      continue;
    ++occupied_cells;
    for (std::size_t first = 0U; first < bin.size(); ++first) {
      for (std::size_t second = first + 1U; second < bin.size(); ++second) {
        const auto a = std::min(bin[first], bin[second]);
        const auto b = std::max(bin[first], bin[second]);
        if (a == b || segments[a].route == segments[b].route ||
            segments[a].layer != segments[b].layer)
          continue;
        if (keys.size() == emission_limit) {
          exhausted = true;
          break;
        }
        keys.push_back((static_cast<std::uint64_t>(a) << 32U) | b);
      }
      if (exhausted)
        break;
    }
    if (exhausted)
      break;
  }
  pair_emissions = keys.size();
  std::sort(keys.begin(), keys.end());
  keys.erase(std::unique(keys.begin(), keys.end()), keys.end());
  if (keys.size() > config.maximum_candidate_pairs) {
    keys.resize(config.maximum_candidate_pairs);
    exhausted = true;
  }
  std::vector<CandidatePair> result;
  result.reserve(keys.size());
  for (const auto key : keys)
    result.push_back({static_cast<std::uint32_t>(key >> 32U), static_cast<std::uint32_t>(key)});
  return result;
}

[[nodiscard]] std::optional<std::size_t> checked_field_cells(std::size_t width,
                                                             std::size_t height,
                                                             std::size_t layers) {
  if (width == 0U || height == 0U || layers == 0U ||
      width > std::numeric_limits<std::size_t>::max() / height)
    return std::nullopt;
  const auto plane = width * height;
  if (plane > std::numeric_limits<std::size_t>::max() / layers)
    return std::nullopt;
  return plane * layers;
}

[[nodiscard]] long double terrain_sample(const RoutingXpbdTerrainField& field, long double x,
                                         long double y, std::uint32_t layer) {
  const auto cells = checked_field_cells(field.width, field.height, field.layer_count);
  if (!cells || field.pitch_nm <= 0 || field.pressure.size() != *cells ||
      layer >= field.layer_count) {
    return 0.0L;
  }
  const auto fx =
      (x - static_cast<long double>(field.origin_x_nm)) / static_cast<long double>(field.pitch_nm);
  const auto fy =
      (y - static_cast<long double>(field.origin_y_nm)) / static_cast<long double>(field.pitch_nm);
  const auto ix = static_cast<std::int64_t>(std::floor(fx));
  const auto iy = static_cast<std::int64_t>(std::floor(fy));
  if (ix < 0 || iy < 0 || ix + 1 >= static_cast<std::int64_t>(field.width) ||
      iy + 1 >= static_cast<std::int64_t>(field.height)) {
    return 0.0L;
  }
  const auto tx = fx - static_cast<long double>(ix);
  const auto ty = fy - static_cast<long double>(iy);
  const auto layer_offset = static_cast<std::size_t>(layer) * field.width * field.height;
  const auto at = [&](std::int64_t px, std::int64_t py) {
    return static_cast<long double>(field.pressure[
        layer_offset + static_cast<std::size_t>(py) * field.width + static_cast<std::size_t>(px)]);
  };
  return (1.0L - tx) * (1.0L - ty) * at(ix, iy) + tx * (1.0L - ty) * at(ix + 1, iy) +
         (1.0L - tx) * ty * at(ix, iy + 1) + tx * ty * at(ix + 1, iy + 1);
}

[[nodiscard]] RoutingXpbdIterationMetrics
measure(const std::vector<Vec2>& positions, const std::vector<RoutingXpbdParticle>& particles,
        const std::vector<RoutingXpbdSegment>& segments,
        const std::vector<RoutingXpbdObstacleSegment>& obstacles,
        const std::vector<CandidatePair>& pairs, const RoutingXpbdConfig& config,
        std::vector<long double>* particle_stress = nullptr,
        std::vector<RoutingXpbdStressBreakdown>* particle_stress_by_type = nullptr,
        RoutingXpbdStressBreakdown* total_stress = nullptr,
        std::size_t* unresolved_trace_pairs = nullptr) {
  enum class StressKind { obstacle, trace_clearance, corridor };
  RoutingXpbdIterationMetrics metrics;
  if (particle_stress != nullptr)
    particle_stress->assign(particles.size(), 0.0L);
  if (particle_stress_by_type != nullptr)
    particle_stress_by_type->assign(particles.size(), {});
  if (total_stress != nullptr)
    *total_stress = {};
  if (unresolved_trace_pairs != nullptr)
    *unresolved_trace_pairs = 0U;
  const auto record = [&](long double residual, std::initializer_list<std::size_t> owners,
                          StressKind kind) {
    // Public geometry is integer nanometres. Sub-half-nanometre floating residue disappears under
    // the mandatory quantization gate and must not report a false unresolved constraint.
    if (!(residual > 0.5L))
      return;
    ++metrics.violations;
    metrics.maximum_residual_nm = std::max(metrics.maximum_residual_nm, residual);
    metrics.total_residual_nm += residual;
    if (particle_stress != nullptr) {
      for (const auto owner : owners) {
        if (owner < particle_stress->size())
          (*particle_stress)[owner] += residual;
      }
    }
    if (particle_stress_by_type != nullptr) {
      for (const auto owner : owners) {
        if (owner >= particle_stress_by_type->size())
          continue;
        auto& stress = (*particle_stress_by_type)[owner];
        if (kind == StressKind::obstacle)
          stress.obstacle_nm += residual;
        else if (kind == StressKind::trace_clearance)
          stress.trace_clearance_nm += residual;
        else
          stress.corridor_nm += residual;
      }
    }
    if (kind == StressKind::trace_clearance && unresolved_trace_pairs != nullptr)
      ++*unresolved_trace_pairs;
  };
  for (const auto& obstacle : obstacles) {
    const auto first = Vec2{static_cast<long double>(obstacle.first_x_nm),
                            static_cast<long double>(obstacle.first_y_nm)};
    const auto second = Vec2{static_cast<long double>(obstacle.second_x_nm),
                             static_cast<long double>(obstacle.second_y_nm)};
    for (const auto& segment : segments) {
      if (segment.layer != obstacle.layer ||
          (obstacle.route != std::numeric_limits<std::uint32_t>::max() &&
           segment.route != obstacle.route))
        continue;
      const auto closest = closest_segments(positions[segment.first_particle],
                                            positions[segment.second_particle], first, second);
      const auto required =
          static_cast<long double>(std::max<std::int64_t>(0, obstacle.radius_nm) +
                                   std::max<std::int64_t>(0, segment.radius_nm) +
                                   std::max<std::int64_t>(0, segment.clearance_nm) +
                                   std::max<std::int64_t>(0, config.numerical_guard_nm));
      record(required - closest.distance, {segment.first_particle, segment.second_particle},
             StressKind::obstacle);
    }
  }
  for (const auto pair : pairs) {
    const auto& first = segments[pair.first];
    const auto& second = segments[pair.second];
    const auto closest =
        closest_segments(positions[first.first_particle], positions[first.second_particle],
                         positions[second.first_particle], positions[second.second_particle]);
    const auto required = static_cast<long double>(
        std::max<std::int64_t>(0, first.radius_nm) + std::max<std::int64_t>(0, second.radius_nm) +
        std::max(std::max<std::int64_t>(0, first.clearance_nm),
                 std::max<std::int64_t>(0, second.clearance_nm)) +
        std::max<std::int64_t>(0, config.numerical_guard_nm));
    record(required - closest.distance, {first.first_particle, first.second_particle,
                                         second.first_particle, second.second_particle},
           StressKind::trace_clearance);
  }
  for (std::size_t particle = 0U; particle < particles.size(); ++particle) {
    if (particles[particle].corridor_radius_nm <= 0)
      continue;
    const auto anchor = Vec2{static_cast<long double>(particles[particle].corridor_anchor_x_nm),
                             static_cast<long double>(particles[particle].corridor_anchor_y_nm)};
    record(length(positions[particle] - anchor) -
               static_cast<long double>(particles[particle].corridor_radius_nm),
           {particle}, StressKind::corridor);
  }
  if (particle_stress_by_type != nullptr) {
    const auto stretch_weight = compliance_weight(config.stretch_compliance);
    const auto bend_weight = compliance_weight(config.bend_compliance);
    std::vector<std::vector<std::size_t>> neighbours(particles.size());
    for (const auto& segment : segments) {
      const auto rest =
          segment.rest_length_nm > 0
              ? static_cast<long double>(segment.rest_length_nm)
              : length(Vec2{static_cast<long double>(particles[segment.second_particle].x_nm -
                                                     particles[segment.first_particle].x_nm),
                            static_cast<long double>(particles[segment.second_particle].y_nm -
                                                     particles[segment.first_particle].y_nm)});
      const auto residual =
          std::abs(length(positions[segment.second_particle] - positions[segment.first_particle]) -
                   rest) *
          stretch_weight;
      (*particle_stress_by_type)[segment.first_particle].stretch_bend_nm += residual;
      (*particle_stress_by_type)[segment.second_particle].stretch_bend_nm += residual;
      neighbours[segment.first_particle].push_back(segment.second_particle);
      neighbours[segment.second_particle].push_back(segment.first_particle);
    }
    for (std::size_t particle = 0U; particle < particles.size(); ++particle) {
      if (neighbours[particle].size() == 2U) {
        const auto target =
            (positions[neighbours[particle][0]] + positions[neighbours[particle][1]]) * 0.5L;
        (*particle_stress_by_type)[particle].stretch_bend_nm +=
            length(target - positions[particle]) * bend_weight;
      }
      auto& stress = (*particle_stress_by_type)[particle];
      if ((particles[particle].flags & routing_xpbd_via) != 0U)
        stress.via_nm = stress.total_nm();
      if (total_stress != nullptr)
        *total_stress += stress;
    }
  }
  return metrics;
}

void build_pressure(RoutingXpbdResult& result, const std::vector<RoutingXpbdParticle>& particles,
                    const RoutingXpbdConfig& config) {
  const auto layer_count = particles.empty()
                               ? 0U
                               : static_cast<std::size_t>(
                                     std::max_element(particles.begin(), particles.end(),
                                                      [](const auto& lhs, const auto& rhs) {
                                                        return lhs.layer < rhs.layer;
                                                      })
                                         ->layer) +
                                     1U;
  const auto cells = checked_field_cells(config.pressure_width, config.pressure_height,
                                         layer_count);
  if (!cells || config.pressure_pitch_nm <= 0) {
    return;
  }
  result.pressure_width = config.pressure_width;
  result.pressure_height = config.pressure_height;
  result.pressure_layer_count = layer_count;
  const auto plane = config.pressure_width * config.pressure_height;
  result.pressure_field.assign(*cells, 0.0F);
  for (std::size_t particle = 0U;
       particle < result.positions_nm.size() && particle < result.particle_stress_nm.size();
       ++particle) {
    const auto x = (result.positions_nm[particle].first - config.pressure_origin_x_nm) /
                   config.pressure_pitch_nm;
    const auto y = (result.positions_nm[particle].second - config.pressure_origin_y_nm) /
                   config.pressure_pitch_nm;
    if (x < 0 || y < 0 || x >= static_cast<std::int64_t>(config.pressure_width) ||
        y >= static_cast<std::int64_t>(config.pressure_height)) {
      continue;
    }
    const auto normalized = static_cast<float>(std::min<long double>(
        result.particle_stress_nm[particle] /
            static_cast<long double>(std::max<std::int64_t>(1, config.pressure_pitch_nm)),
        1'000'000.0L));
    const auto layer_offset = static_cast<std::size_t>(particles[particle].layer) * plane;
    result.pressure_field[layer_offset + static_cast<std::size_t>(y) * config.pressure_width +
                          static_cast<std::size_t>(x)] += normalized;
  }
  std::vector<float> scratch(result.pressure_field.size(), 0.0F);
  const auto diffusion = static_cast<float>(std::clamp(config.pressure_diffusion, 0.0L, 0.25L));
  for (std::size_t pass = 0U; pass < config.pressure_diffusion_passes; ++pass) {
    for (std::size_t layer = 0U; layer < layer_count; ++layer) {
      const auto layer_offset = layer * plane;
      for (std::size_t y = 0U; y < config.pressure_height; ++y) {
        for (std::size_t x = 0U; x < config.pressure_width; ++x) {
          const auto index = layer_offset + y * config.pressure_width + x;
          auto sum = 0.0F;
          auto neighbours = 0U;
          if (x > 0U) {
            sum += result.pressure_field[index - 1U];
            ++neighbours;
          }
          if (x + 1U < config.pressure_width) {
            sum += result.pressure_field[index + 1U];
            ++neighbours;
          }
          if (y > 0U) {
            sum += result.pressure_field[index - config.pressure_width];
            ++neighbours;
          }
          if (y + 1U < config.pressure_height) {
            sum += result.pressure_field[index + config.pressure_width];
            ++neighbours;
          }
          scratch[index] = neighbours == 0U
                               ? result.pressure_field[index]
                               : (1.0F - diffusion * static_cast<float>(neighbours)) *
                                         result.pressure_field[index] +
                                     diffusion * sum;
        }
      }
    }
    result.pressure_field.swap(scratch);
  }
}

[[nodiscard]] RoutingXpbdResult relax_cpu(const std::vector<RoutingXpbdParticle>& particles,
                                          const std::vector<RoutingXpbdSegment>& segments,
                                          const std::vector<RoutingXpbdObstacleSegment>& obstacles,
                                          const RoutingXpbdTerrainField& terrain,
                                          const RoutingXpbdConfig& config) {
  RoutingXpbdResult result;
  constexpr std::uint32_t maximum_routing_layers = 64U;
  if (particles.empty() || segments.empty() || config.iterations == 0U ||
      config.maximum_step_nm <= 0 || config.maximum_candidate_pairs == 0U) {
    result.diagnostic = "invalid XPBD routing input";
    return result;
  }
  if (std::any_of(particles.begin(), particles.end(), [](const auto& particle) {
        return particle.layer >= maximum_routing_layers;
      })) {
    result.diagnostic = "XPBD particle layer exceeds the bounded routing-layer count";
    return result;
  }
  if (!terrain.pressure.empty()) {
    const auto terrain_cells =
        checked_field_cells(terrain.width, terrain.height, terrain.layer_count);
    if (!terrain_cells || terrain.pitch_nm <= 0 || terrain.pressure.size() != *terrain_cells ||
        terrain.layer_count > maximum_routing_layers) {
      result.diagnostic = "invalid layered XPBD terrain field";
      return result;
    }
  }
  for (const auto& segment : segments) {
    if (segment.first_particle >= particles.size() || segment.second_particle >= particles.size() ||
        segment.first_particle == segment.second_particle) {
      result.diagnostic = "XPBD segment references an invalid particle";
      return result;
    }
  }
  const auto broadphase_start = SteadyClock::now();
  bool pair_budget_exhausted = false;
  std::size_t broadphase_cells = 0U;
  std::size_t broadphase_emissions = 0U;
  const auto pairs = build_candidate_pairs(particles, segments, config, pair_budget_exhausted,
                                           broadphase_cells, broadphase_emissions);
  result.timings.broadphase_us = elapsed_us(broadphase_start, SteadyClock::now());
  result.candidate_pairs = pairs.size();
  result.broadphase_cells = broadphase_cells;
  result.broadphase_pair_emissions = broadphase_emissions;
  result.pair_budget_exhausted = pair_budget_exhausted;
  const auto generation_start = SteadyClock::now();
  result.constraints.stretch = segments.size();
  std::vector<std::size_t> degrees(particles.size(), 0U);
  for (const auto& segment : segments) {
    ++degrees[segment.first_particle];
    ++degrees[segment.second_particle];
    result.constraints.obstacle += static_cast<std::size_t>(
        std::count_if(obstacles.begin(), obstacles.end(), [&](const auto& obstacle) {
          return segment.layer == obstacle.layer &&
                 (obstacle.route == std::numeric_limits<std::uint32_t>::max() ||
                  obstacle.route == segment.route);
        }));
  }
  result.constraints.bend = static_cast<std::size_t>(
      std::count(degrees.begin(), degrees.end(), static_cast<std::size_t>(2U)));
  result.constraints.trace_clearance = pairs.size();
  result.constraints.corridor = static_cast<std::size_t>(
      std::count_if(particles.begin(), particles.end(),
                    [](const auto& particle) { return particle.corridor_radius_nm > 0; }));
  std::vector<Vec2> positions;
  positions.reserve(particles.size());
  for (const auto& particle : particles) {
    positions.push_back(
        {static_cast<long double>(particle.x_nm), static_cast<long double>(particle.y_nm)});
  }
  result.initial = measure(positions, particles, segments, obstacles, pairs, config);
  result.timings.constraint_generation_us = elapsed_us(generation_start, SteadyClock::now());
  result.iteration_metrics.reserve(config.iterations);
  const auto stretch_weight = compliance_weight(config.stretch_compliance);
  const auto bend_weight = compliance_weight(config.bend_compliance);
  const auto clearance_weight = compliance_weight(config.clearance_compliance);
  const auto corridor_weight = compliance_weight(config.corridor_compliance);
  const auto solve_start = SteadyClock::now();
  for (std::size_t iteration = 0U; iteration < config.iterations; ++iteration) {
    const auto legality_settling = iteration >= config.iterations - config.iterations / 4U;
    std::vector<Vec2> corrections(particles.size());
    std::vector<long double> weights(particles.size(), 0.0L);
    const auto add = [&](std::size_t particle, Vec2 correction, long double weight) {
      if (particle >= particles.size() || !(particles[particle].inverse_mass > 0.0L) ||
          !(weight > 0.0L)) {
        return;
      }
      corrections[particle] += correction * weight;
      // Average simultaneously proposed Jacobi corrections by constraint count. The compliance
      // weight belongs in the correction magnitude; summing it into the divisor would cancel a
      // lone constraint's compliance completely.
      weights[particle] += 1.0L;
    };
    for (const auto& segment : segments) {
      if (legality_settling)
        break;
      const auto first = segment.first_particle;
      const auto second = segment.second_particle;
      const auto delta = positions[second] - positions[first];
      const auto current_length = length(delta);
      const auto rest =
          segment.rest_length_nm > 0
              ? static_cast<long double>(segment.rest_length_nm)
              : length(
                    Vec2{static_cast<long double>(particles[second].x_nm - particles[first].x_nm),
                         static_cast<long double>(particles[second].y_nm - particles[first].y_nm)});
      if (!(current_length > 0.0L) || !(rest > 0.0L))
        continue;
      const auto direction = delta * (1.0L / current_length);
      const auto residual = current_length - rest;
      const auto total_mass = std::max<long double>(1.0e-12L, particles[first].inverse_mass +
                                                                  particles[second].inverse_mass);
      add(first, direction * (residual * particles[first].inverse_mass / total_mass),
          stretch_weight);
      add(second, direction * (-residual * particles[second].inverse_mass / total_mass),
          stretch_weight);
    }
    // A route-local Laplacian is a stable bending constraint and does not require spline
    // reconstruction inside the solver.
    std::vector<std::vector<std::size_t>> neighbours(particles.size());
    for (const auto& segment : segments) {
      neighbours[segment.first_particle].push_back(segment.second_particle);
      neighbours[segment.second_particle].push_back(segment.first_particle);
    }
    for (std::size_t particle = 0U; particle < particles.size() && !legality_settling; ++particle) {
      if (neighbours[particle].size() != 2U)
        continue;
      const auto target =
          (positions[neighbours[particle][0]] + positions[neighbours[particle][1]]) * 0.5L;
      add(particle, target - positions[particle], bend_weight);
    }
    for (const auto& obstacle : obstacles) {
      const auto first = Vec2{static_cast<long double>(obstacle.first_x_nm),
                              static_cast<long double>(obstacle.first_y_nm)};
      const auto second = Vec2{static_cast<long double>(obstacle.second_x_nm),
                               static_cast<long double>(obstacle.second_y_nm)};
      for (const auto& segment : segments) {
        if (segment.layer != obstacle.layer ||
            (obstacle.route != std::numeric_limits<std::uint32_t>::max() &&
             segment.route != obstacle.route))
          continue;
        const auto closest = closest_segments(positions[segment.first_particle],
                                              positions[segment.second_particle], first, second);
        const auto delta = closest.first - closest.second;
        const auto required =
            static_cast<long double>(std::max<std::int64_t>(0, obstacle.radius_nm) +
                                     std::max<std::int64_t>(0, segment.radius_nm) +
                                     std::max<std::int64_t>(0, segment.clearance_nm) +
                                     std::max<std::int64_t>(0, config.numerical_guard_nm));
        const auto distance = closest.distance;
        if (distance < required) {
          const auto route_delta =
              positions[segment.second_particle] - positions[segment.first_particle];
          auto fallback = normalized({-route_delta.y, route_delta.x},
                                     normalized({second.y - first.y, first.x - second.x}));
          if ((segment.route & 1U) != 0U)
            fallback = fallback * -1.0L;
          const auto outward = normalized(
              Vec2{static_cast<long double>(obstacle.outward_normal_x),
                   static_cast<long double>(obstacle.outward_normal_y)},
              fallback);
          const auto has_outward = obstacle.outward_normal_x != 0 || obstacle.outward_normal_y != 0;
          // A symmetric line obstacle uses the closest-side normal. A polygon boundary carries
          // its forbidden-side certificate, so a point on the material side is projected through
          // the boundary toward free space instead of being pushed deeper into the obstacle.
          const auto closest_normal = normalized(delta, fallback);
          const auto normal = has_outward && distance > 1.0e-12L &&
                                      dot(closest_normal, outward) < 0.0L
                                  ? outward
                                  : closest_normal;
          add(segment.first_particle,
              normal * (required - distance) * (1.0L - closest.first_parameter), clearance_weight);
          add(segment.second_particle, normal * (required - distance) * closest.first_parameter,
              clearance_weight);
        }
      }
    }
    for (const auto pair : pairs) {
      const auto& first = segments[pair.first];
      const auto& second = segments[pair.second];
      const auto closest =
          closest_segments(positions[first.first_particle], positions[first.second_particle],
                           positions[second.first_particle], positions[second.second_particle]);
      const auto required = static_cast<long double>(
          std::max<std::int64_t>(0, first.radius_nm) + std::max<std::int64_t>(0, second.radius_nm) +
          std::max(std::max<std::int64_t>(0, first.clearance_nm),
                   std::max<std::int64_t>(0, second.clearance_nm)) +
          std::max<std::int64_t>(0, config.numerical_guard_nm));
      if (closest.distance >= required)
        continue;
      auto normal = normalized(closest.first - closest.second);
      if (!(closest.distance > 1.0e-12L)) {
        const auto first_delta = positions[first.second_particle] - positions[first.first_particle];
        normal = normalized({-first_delta.y, first_delta.x});
        if (((first.route + second.route) & 1U) != 0U)
          normal = normal * -1.0L;
      }
      const auto correction = required - closest.distance;
      add(first.first_particle, normal * correction * (1.0L - closest.first_parameter),
          clearance_weight);
      add(first.second_particle, normal * correction * closest.first_parameter, clearance_weight);
      add(second.first_particle, normal * -correction * (1.0L - closest.second_parameter),
          clearance_weight);
      add(second.second_particle, normal * -correction * closest.second_parameter,
          clearance_weight);
    }
    for (std::size_t particle = 0U; particle < particles.size(); ++particle) {
      if (particles[particle].corridor_radius_nm > 0) {
        const auto anchor =
            Vec2{static_cast<long double>(particles[particle].corridor_anchor_x_nm),
                 static_cast<long double>(particles[particle].corridor_anchor_y_nm)};
        const auto delta = positions[particle] - anchor;
        const auto distance = length(delta);
        if (distance > static_cast<long double>(particles[particle].corridor_radius_nm)) {
          add(particle,
              normalized(delta) *
                  (static_cast<long double>(particles[particle].corridor_radius_nm) - distance),
              corridor_weight);
        }
      }
      if (!legality_settling && config.terrain_gain > 0.0L && terrain.pitch_nm > 0) {
        const auto probe = static_cast<long double>(terrain.pitch_nm);
        const auto gx =
            terrain_sample(terrain, positions[particle].x + probe, positions[particle].y,
                           particles[particle].layer) -
            terrain_sample(terrain, positions[particle].x - probe, positions[particle].y,
                           particles[particle].layer);
        const auto gy =
            terrain_sample(terrain, positions[particle].x, positions[particle].y + probe,
                           particles[particle].layer) -
            terrain_sample(terrain, positions[particle].x, positions[particle].y - probe,
                           particles[particle].layer);
        add(particle, Vec2{-gx, -gy} * (config.terrain_gain * probe * 0.5L), 1.0L);
      }
    }
    for (std::size_t particle = 0U; particle < particles.size(); ++particle) {
      if (!(particles[particle].inverse_mass > 0.0L) || !(weights[particle] > 0.0L))
        continue;
      auto correction = corrections[particle] * (1.0L / weights[particle]);
      const auto magnitude = length(correction);
      if (magnitude > static_cast<long double>(config.maximum_step_nm)) {
        correction = correction * (static_cast<long double>(config.maximum_step_nm) / magnitude);
      }
      positions[particle] += correction;
    }
    const auto metrics = measure(positions, particles, segments, obstacles, pairs, config);
    result.iteration_metrics.push_back(metrics);
    ++result.iterations_executed;
    if (metrics.violations == 0U) {
      result.converged = true;
      break;
    }
  }
  result.timings.solve_us = elapsed_us(solve_start, SteadyClock::now());
  const auto reduction_start = SteadyClock::now();
  result.final =
      measure(positions, particles, segments, obstacles, pairs, config, &result.particle_stress_nm,
              &result.particle_stress_by_type, &result.stress, &result.unresolved_trace_pairs);
  result.timings.stress_reduction_us = elapsed_us(reduction_start, SteadyClock::now());
  result.converged = result.final.violations == 0U;
  result.stalled = !result.converged &&
                   result.final.total_residual_nm + 0.5L >= result.initial.total_residual_nm;
  result.topology_escape_required = !result.converged;
  result.positions_nm.reserve(positions.size());
  for (const auto position : positions) {
    result.positions_nm.emplace_back(static_cast<std::int64_t>(std::llround(position.x)),
                                     static_cast<std::int64_t>(std::llround(position.y)));
  }
  build_pressure(result, particles, config);
  return result;
}

} // namespace

RoutingXpbdResult relax_routing_xpbd(const std::vector<RoutingXpbdParticle>& particles,
                                     const std::vector<RoutingXpbdSegment>& segments,
                                     const std::vector<RoutingXpbdObstacleSegment>& obstacles,
                                     const RoutingXpbdTerrainField& terrain,
                                     const RoutingXpbdConfig& config) {
#if FLUX_HAS_VULKAN_HYDRAULIC_PATHS
  if (config.prefer_gpu) {
    auto gpu = relax_routing_xpbd_gpu(particles, segments, obstacles, terrain, config);
    if (gpu.ok())
      return gpu;
    auto cpu = relax_cpu(particles, segments, obstacles, terrain, config);
    cpu.cpu_fallback = cpu.ok();
    if (cpu.ok() && !gpu.diagnostic.empty())
      cpu.diagnostic.clear();
    return cpu;
  }
#endif
  return relax_cpu(particles, segments, obstacles, terrain, config);
}

} // namespace flux
