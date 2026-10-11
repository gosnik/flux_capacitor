#include "flux/core/routing_avbd.hpp"

#include "flux/core/cpu_worker_budget.hpp"
#include "flux/core/routing_kernel.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <map>
#include <numeric>
#include <set>
#include <utility>
#include <vector>

namespace flux {
namespace {

struct Vec2 { long double x{0}; long double y{0}; };
[[nodiscard]] Vec2 operator+(Vec2 a, Vec2 b) { return {a.x + b.x, a.y + b.y}; }
[[nodiscard]] Vec2 operator-(Vec2 a, Vec2 b) { return {a.x - b.x, a.y - b.y}; }
[[nodiscard]] Vec2 operator*(Vec2 a, long double b) { return {a.x * b, a.y * b}; }
Vec2& operator+=(Vec2& a, Vec2 b) { a.x += b.x; a.y += b.y; return a; }
[[nodiscard]] long double dot(Vec2 a, Vec2 b) { return a.x * b.x + a.y * b.y; }
[[nodiscard]] long double length(Vec2 a) { return std::hypotl(a.x, a.y); }
[[nodiscard]] Vec2 unit(Vec2 a, Vec2 fallback = {1, 0}) {
  const auto l = length(a); return l > 1.0e-15L ? a * (1.0L / l) : fallback;
}
[[nodiscard]] bool finite(Vec2 a) { return std::isfinite(a.x) && std::isfinite(a.y); }

struct Closest { long double a{0}, b{0}, distance{0}; Vec2 first, second; };
[[nodiscard]] Closest closest(Vec2 p, Vec2 q, Vec2 r, Vec2 s) {
  const auto d1 = q - p, d2 = s - r, w = p - r;
  const auto aa = dot(d1, d1), ee = dot(d2, d2), ff = dot(d2, w);
  long double u = 0, v = 0;
  if (aa < 1.0e-18L && ee < 1.0e-18L) return {0, 0, length(w), p, r};
  if (aa < 1.0e-18L) v = std::clamp(ff / ee, 0.0L, 1.0L);
  else {
    const auto cc = dot(d1, w);
    if (ee < 1.0e-18L) u = std::clamp(-cc / aa, 0.0L, 1.0L);
    else {
      const auto bb = dot(d1, d2), den = aa * ee - bb * bb;
      if (std::abs(den) > 1.0e-18L) u = std::clamp((bb * ff - cc * ee) / den, 0.0L, 1.0L);
      v = (bb * u + ff) / ee;
      if (v < 0) { v = 0; u = std::clamp(-cc / aa, 0.0L, 1.0L); }
      else if (v > 1) { v = 1; u = std::clamp((bb - cc) / aa, 0.0L, 1.0L); }
    }
  }
  const auto first = p + d1 * u, second = r + d2 * v;
  return {u, v, length(first - second), first, second};
}

struct Term { std::size_t owner; long double coefficient; Vec2 direction; };
struct ContactKey {
  unsigned type{0};
  std::size_t first{0}, second{0};
  auto operator<=>(const ContactKey&) const = default;
};
struct Contact {
  ContactKey key{}; std::vector<Term> terms; long double residual{0};
};
struct Dual { long double lambda{0}; long double penalty{0}; };
struct ObstacleIdentity {
  std::int64_t first_x, first_y, second_x, second_y, radius, normal_x, normal_y;
  std::uint32_t layer, route;
  auto operator<=>(const ObstacleIdentity&) const = default;
};
struct Box { long double x0, y0, x1, y1; };
[[nodiscard]] bool overlaps(Box a, Box b) {
  return a.x0 <= b.x1 && b.x0 <= a.x1 && a.y0 <= b.y1 && b.y0 <= a.y1;
}
[[nodiscard]] ContactKey pair_key(std::size_t a, std::size_t b) {
  return {0U, std::min(a, b), std::max(a, b)};
}
[[nodiscard]] ContactKey feature_key(unsigned type, std::size_t first, std::size_t second) {
  return {type, first, second};
}
[[nodiscard]] long double nm(std::int64_t value) { return static_cast<long double>(value); }

[[nodiscard]] RoutingXpbdIterationMetrics metrics(const std::vector<Contact>& contacts,
                                                   const std::vector<Vec2>& pos,
                                                   const std::vector<RoutingXpbdSegment>& segments) {
  RoutingXpbdIterationMetrics out;
  for (const auto& c : contacts) if (c.residual > 0) {
    ++out.violations; out.total_residual_nm += c.residual;
    out.maximum_residual_nm = std::max(out.maximum_residual_nm, c.residual);
  }
  for (const auto& s : segments) {
    const auto rest = s.rest_length_nm > 0 ? static_cast<long double>(s.rest_length_nm) : 0.0L;
    if (rest <= 0) continue;
    const auto r = std::abs(length(pos[s.second_particle] - pos[s.first_particle]) - rest);
    out.total_residual_nm += r;
    out.maximum_residual_nm = std::max(out.maximum_residual_nm, r);
  }
  return out;
}

} // namespace

RoutingAvbdResult relax_routing_avbd(const std::vector<RoutingXpbdParticle>& particles,
                                     const std::vector<RoutingXpbdSegment>& segments,
                                     const std::vector<RoutingXpbdObstacleSegment>& obstacles,
                                     const RoutingXpbdConfig& xpbd, const std::vector<RoutingAvbdRigidGroup>& groups,
                                     const RoutingAvbdConfig& avbd, std::stop_token stop) {
  RoutingAvbdResult out;
  const auto solve_started = std::chrono::steady_clock::now();
  const auto elapsed = [&] { return static_cast<std::uint64_t>(
      std::chrono::duration_cast<std::chrono::microseconds>(
          std::chrono::steady_clock::now() - solve_started).count()); };
  auto fail = [&](const char* text) {
    out.relaxation.diagnostic = text;
    out.relaxation.timings.solve_us = elapsed();
    out.relaxation.positions_nm.clear();
    return out;
  };
  if (particles.empty() || particles.size() > avbd.maximum_particles || segments.empty() ||
      avbd.iterations == 0 || avbd.maximum_contacts == 0 || xpbd.maximum_step_nm <= 0 ||
      !(avbd.initial_penalty > 0) || !(avbd.maximum_penalty >= avbd.initial_penalty) ||
      !(avbd.penalty_growth >= 1) || !std::isfinite(avbd.initial_penalty) ||
      !std::isfinite(avbd.maximum_penalty) || !std::isfinite(avbd.penalty_growth) ||
      avbd.maximum_contacts > std::numeric_limits<std::size_t>::max() / 16U) return fail("invalid or unbounded AVBD input");
  for (const auto& p : particles) {
    if (!std::isfinite(p.inverse_mass) || p.inverse_mass < 0 || p.radius_nm < 0 || p.clearance_nm < 0 || p.corridor_radius_nm < 0)
      return fail("non-finite or invalid AVBD particle");
  }
  if (xpbd.numerical_guard_nm < 0) return fail("AVBD numerical guard is negative");
  for (const auto& s : segments) if (s.first_particle >= particles.size() || s.second_particle >= particles.size() ||
                                      s.first_particle == s.second_particle || s.radius_nm < 0 ||
                                      s.clearance_nm < 0 || s.rest_length_nm < 0) return fail("AVBD segment has invalid index or dimension");
  for (const auto& o : obstacles) if (o.radius_nm < 0) return fail("AVBD obstacle has negative radius");
  if (segments.size() > avbd.maximum_contacts * 16U ||
      obstacles.size() > avbd.maximum_contacts * 16U)
    return fail("AVBD feature preparation budget exhausted");

  std::map<ObstacleIdentity, std::size_t> canonical_obstacle_map;
  std::vector<std::size_t> canonical_obstacles(obstacles.size());
  for (std::size_t index = 0; index < obstacles.size(); ++index) {
    const auto& o = obstacles[index];
    const ObstacleIdentity identity{o.first_x_nm, o.first_y_nm, o.second_x_nm, o.second_y_nm,
                                    o.radius_nm, o.outward_normal_x, o.outward_normal_y, o.layer, o.route};
    canonical_obstacles[index] = canonical_obstacle_map.try_emplace(
        identity, canonical_obstacle_map.size()).first->second;
  }

  std::vector<std::size_t> owner(particles.size());
  std::iota(owner.begin(), owner.end(), 0U);
  std::vector<bool> assigned_to_group(particles.size(), false);
  std::vector<std::vector<std::size_t>> members(particles.size());
  for (std::size_t i = 0; i < particles.size(); ++i) members[i].push_back(i);
  for (const auto& group : groups) {
    if (group.particle_indices.empty()) continue;
    const auto leader = group.particle_indices.front();
    if (leader >= particles.size()) return fail("AVBD rigid group references invalid particle");
    for (const auto index : group.particle_indices) {
      if (index >= particles.size() || assigned_to_group[index])
        return fail("AVBD rigid groups overlap or reference invalid particle");
      assigned_to_group[index] = true;
    }
    members[leader].clear();
    for (const auto index : group.particle_indices) { assigned_to_group[index] = true; owner[index] = leader; members[leader].push_back(index); }
  }
  std::vector<std::size_t> active_owners;
  std::vector<bool> fixed(particles.size(), false);
  for (std::size_t i = 0; i < particles.size(); ++i) if (owner[i] == i) {
    for (const auto member : members[i]) fixed[i] = fixed[i] || !(particles[member].inverse_mass > 0);
    active_owners.push_back(i);
  }
  std::vector<Vec2> initial, pos;
  for (const auto& p : particles) initial.push_back({static_cast<long double>(p.x_nm), static_cast<long double>(p.y_nm)});
  pos = initial;
  std::map<ContactKey, Dual> duals;
  std::vector<long double> stretch_duals(segments.size(), 0.0L);
  long double penalty = avbd.initial_penalty;
  long double previous_residual = std::numeric_limits<long double>::infinity();

  const auto build_contacts = [&](bool& overflow) {
    overflow = false;
    std::vector<Contact> found;
    const auto margin = nm(xpbd.maximum_step_nm) + nm(xpbd.numerical_guard_nm);
    std::vector<Box> boxes;
    boxes.reserve(segments.size());
    for (const auto& s : segments) {
      const auto pad = nm(s.radius_nm) + nm(s.clearance_nm) + margin;
      boxes.push_back({std::min(pos[s.first_particle].x, pos[s.second_particle].x) - pad,
                       std::min(pos[s.first_particle].y, pos[s.second_particle].y) - pad,
                       std::max(pos[s.first_particle].x, pos[s.second_particle].x) + pad,
                       std::max(pos[s.first_particle].y, pos[s.second_particle].y) + pad});
    }
    // A bounded spatial sweep rather than a segment all-pairs pass.  Sorting gives deterministic
    // broadphase order and the explicit emission cap makes an adversarial cell fail closed.
    std::vector<std::size_t> order(segments.size()); std::iota(order.begin(), order.end(), 0U);
    std::sort(order.begin(), order.end(), [&](auto a, auto b) { return boxes[a].x0 < boxes[b].x0 || (boxes[a].x0 == boxes[b].x0 && a < b); });
    std::set<ContactKey> emitted;
    std::size_t candidate_checks = 0U;
    const auto add_pair = [&](std::size_t a, std::size_t b) {
      if (++candidate_checks > avbd.maximum_contacts * 16U || emitted.size() >= avbd.maximum_contacts * 8U) { overflow = true; return; }
      const auto& first = segments[a]; const auto& second = segments[b];
      if (first.route == second.route || first.layer != second.layer) return;
      if (!overlaps(boxes[a], boxes[b])) return;
      const auto c = closest(pos[first.first_particle], pos[first.second_particle], pos[second.first_particle], pos[second.second_particle]);
      const auto required = nm(first.radius_nm) + nm(second.radius_nm) +
          std::max(nm(first.clearance_nm), nm(second.clearance_nm)) + nm(xpbd.numerical_guard_nm);
      if (c.distance >= required + margin) return;
      const auto key = pair_key(a, b); if (!emitted.insert(key).second) { ++out.duplicate_reductions; return; }
      const auto tangent = unit(pos[first.second_particle] - pos[first.first_particle], {1, 0});
      auto fallback = Vec2{-tangent.y, tangent.x};
      if (((first.route + second.route) & 1U) != 0U) fallback = fallback * -1.0L;
      auto n = unit(c.first - c.second, fallback);
      Contact contact{key, {}, required - c.distance};
      const auto add = [&](std::size_t p, long double coef, Vec2 dir) { contact.terms.push_back({owner[p], coef, dir}); };
      add(first.first_particle, 1.0L - c.a, n); add(first.second_particle, c.a, n);
      add(second.first_particle, 1.0L - c.b, n * -1.0L); add(second.second_particle, c.b, n * -1.0L);
      found.push_back(std::move(contact));
    };
    std::vector<std::size_t> active;
    for (const auto a : order) {
      if (overflow) break;
      routing_kernel_checkpoint();
      std::erase_if(active, [&](const auto b) { return boxes[b].x1 < boxes[a].x0; });
      for (const auto b : active) {
        add_pair(a, b);
        if (overflow) break;
      }
      active.push_back(a);
    }
    // A via is a real disk rather than a whole cross-layer trace.  Its layer-local particle
    // copies therefore see copper on that layer only; unrelated cross-layer trace spans remain
    // exempt.  Same-net contacts are omitted because they are one routed conductor.
    for (std::size_t pi = 0; pi < particles.size() && !overflow; ++pi) {
      if ((particles[pi].flags & routing_xpbd_via) == 0U) continue;
      for (std::size_t si = 0; si < segments.size(); ++si) {
        if (++candidate_checks > avbd.maximum_contacts * 16U) { overflow = true; break; }
        const auto& s = segments[si]; if (s.route == particles[pi].route || s.layer != particles[pi].layer ||
            s.first_particle == pi || s.second_particle == pi) continue;
        const auto c = closest(pos[pi], pos[pi], pos[s.first_particle], pos[s.second_particle]);
        const auto required = nm(particles[pi].radius_nm) + nm(s.radius_nm) +
          std::max(nm(particles[pi].clearance_nm), nm(s.clearance_nm)) + nm(xpbd.numerical_guard_nm);
        if (c.distance >= required + margin) continue;
        const auto key = feature_key(2U, pi, si); if (!emitted.insert(key).second) { ++out.duplicate_reductions; continue; }
        const auto n = unit(c.first - c.second, {0, 1});
        found.push_back({key, {{owner[pi], 1.0L, n}, {owner[s.first_particle], 1.0L-c.b, n*-1.0L}, {owner[s.second_particle], c.b, n*-1.0L}}, required-c.distance});
        if (found.size() > avbd.maximum_contacts) { overflow = true; break; }
      }
    }
    // Obstacles use the same bounded sweep gate. Exact duplicate obstacle geometry maps to one
    // physical key, while nearby distinct edges deliberately retain separate contacts.
    for (std::size_t si = 0; si < segments.size() && !overflow; ++si) for (std::size_t oi = 0; oi < obstacles.size(); ++oi) {
      routing_kernel_checkpoint();
      if (++candidate_checks > avbd.maximum_contacts * 16U) { overflow = true; break; }
      const auto& s = segments[si]; const auto& o = obstacles[oi];
      if (s.layer != o.layer || (o.route != std::numeric_limits<std::uint32_t>::max() && o.route != s.route)) continue;
      const auto c = closest(pos[s.first_particle], pos[s.second_particle], {static_cast<long double>(o.first_x_nm), static_cast<long double>(o.first_y_nm)}, {static_cast<long double>(o.second_x_nm), static_cast<long double>(o.second_y_nm)});
      const auto required = nm(s.radius_nm) + nm(s.clearance_nm) + nm(o.radius_nm) + nm(xpbd.numerical_guard_nm);
      if (c.distance >= required + margin) continue;
      const auto key = feature_key(1U, si, canonical_obstacles[oi]); if (!emitted.insert(key).second) { ++out.duplicate_reductions; continue; }
      auto n = unit(c.first - c.second, {static_cast<long double>(o.outward_normal_x), static_cast<long double>(o.outward_normal_y)});
      if (o.outward_normal_x != 0 || o.outward_normal_y != 0) n = unit({static_cast<long double>(o.outward_normal_x), static_cast<long double>(o.outward_normal_y)});
      found.push_back({key, {{owner[s.first_particle], 1.0L - c.a, n}, {owner[s.second_particle], c.a, n}}, required - c.distance});
      if (found.size() > avbd.maximum_contacts) { overflow = true; break; }
    }
    for (std::size_t pi = 0; pi < particles.size() && !overflow; ++pi) {
      if ((particles[pi].flags & routing_xpbd_via) == 0U) continue;
      for (std::size_t oi = 0; oi < obstacles.size(); ++oi) {
        if (++candidate_checks > avbd.maximum_contacts * 16U) { overflow = true; break; }
        const auto& o = obstacles[oi]; if (o.layer != particles[pi].layer ||
            (o.route != std::numeric_limits<std::uint32_t>::max() && o.route != particles[pi].route)) continue;
        const auto c = closest(pos[pi], pos[pi], {static_cast<long double>(o.first_x_nm), static_cast<long double>(o.first_y_nm)}, {static_cast<long double>(o.second_x_nm), static_cast<long double>(o.second_y_nm)});
        const auto required = nm(particles[pi].radius_nm) + nm(o.radius_nm) + nm(particles[pi].clearance_nm) + nm(xpbd.numerical_guard_nm);
        if (c.distance >= required + margin) continue;
        const auto key = feature_key(3U, pi, canonical_obstacles[oi]); if (!emitted.insert(key).second) { ++out.duplicate_reductions; continue; }
        auto n = unit(c.first-c.second, {static_cast<long double>(o.outward_normal_x), static_cast<long double>(o.outward_normal_y)});
        if (o.outward_normal_x != 0 || o.outward_normal_y != 0) n = unit({static_cast<long double>(o.outward_normal_x), static_cast<long double>(o.outward_normal_y)});
        found.push_back({key, {{owner[pi], 1.0L, n}}, required-c.distance});
      }
    }
    for (std::size_t a = 0; a < particles.size() && !overflow; ++a) {
      if ((particles[a].flags & routing_xpbd_via) == 0U) continue;
      for (std::size_t b = 0; b < a; ++b) {
        if (++candidate_checks > avbd.maximum_contacts * 16U) { overflow = true; break; }
        if ((particles[b].flags & routing_xpbd_via) == 0U || particles[a].route == particles[b].route || particles[a].layer != particles[b].layer) continue;
        const auto delta = pos[a]-pos[b]; const auto distance = length(delta);
        const auto required = nm(particles[a].radius_nm) + nm(particles[b].radius_nm) + std::max(nm(particles[a].clearance_nm), nm(particles[b].clearance_nm)) + nm(xpbd.numerical_guard_nm);
        if (distance >= required+margin) continue;
        const auto key = feature_key(4U, a, b); if (!emitted.insert(key).second) { ++out.duplicate_reductions; continue; }
        const auto n = unit(delta, {0, 1}); found.push_back({key, {{owner[a], 1.0L, n}, {owner[b], 1.0L, n*-1.0L}}, required-distance});
      }
    }
    if (found.size() > avbd.maximum_contacts) overflow = true;
    // Sum all endpoint gradients belonging to one rigid owner before forming its Hessian.
    // This preserves cross terms when two endpoints of the same contact move together.
    for (auto& contact : found) {
      std::map<std::size_t, Vec2> gradients;
      for (const auto& term : contact.terms)
        gradients[term.owner] += term.direction * term.coefficient;
      contact.terms.clear();
      for (const auto& [own, gradient] : gradients)
        if (dot(gradient, gradient) > 0.0L) contact.terms.push_back({own, 1.0L, gradient});
    }
    std::sort(found.begin(), found.end(), [](const auto& a, const auto& b) { return a.key < b.key; });
    return found;
  };

  bool first_overflow = false; auto initial_contacts = build_contacts(first_overflow);
  if (first_overflow) return fail("AVBD contact budget exhausted");
  out.relaxation.initial = metrics(initial_contacts, pos, segments);
  auto best_positions = pos;
  auto best_metrics = out.relaxation.initial;
  std::vector<unsigned char> moved_rigid(particles.size(), 0U);
  auto lease = acquire_cpu_workers(avbd.max_workers == 0U ? active_owners.size() : avbd.max_workers,
                                   avbd.max_workers);
  for (std::size_t iteration = 0; iteration < avbd.iterations; ++iteration) {
    if (stop.stop_requested()) { out.cancelled = true; out.relaxation.diagnostic = "AVBD relaxation cancelled"; break; }
    routing_kernel_checkpoint(); bool overflow = false; auto contacts = build_contacts(overflow);
    if (overflow) return fail("AVBD contact budget exhausted");
    out.contacts = std::max(out.contacts, contacts.size());
    const auto color_snapshot = pos;
    std::vector<std::set<std::size_t>> graph(particles.size());
    std::vector<std::size_t> incident_contacts(particles.size(), 0U);
    std::vector<std::vector<std::size_t>> owner_contacts(particles.size());
    for (const auto& c : contacts) { std::set<std::size_t> owners; for (const auto& t : c.terms) owners.insert(t.owner);
      for (auto a = owners.begin(); a != owners.end(); ++a) for (auto b = std::next(a); b != owners.end(); ++b) { graph[*a].insert(*b); graph[*b].insert(*a); }
      for (const auto own : owners) { if (!fixed[own]) ++incident_contacts[own]; owner_contacts[own].push_back(static_cast<std::size_t>(&c - contacts.data())); }
    }
    std::vector<std::vector<std::size_t>> owner_segments(particles.size());
    for (std::size_t si = 0; si < segments.size(); ++si) { const auto& s = segments[si]; const auto a = owner[s.first_particle], b = owner[s.second_particle]; owner_segments[a].push_back(si); if (b != a) { owner_segments[b].push_back(si); graph[a].insert(b); graph[b].insert(a); } }
    std::vector<std::size_t> color(particles.size(), std::numeric_limits<std::size_t>::max()); std::size_t color_count = 0;
    for (const auto own : active_owners) { std::set<std::size_t> used; for (const auto near : graph[own]) if (color[near] != std::numeric_limits<std::size_t>::max()) used.insert(color[near]);
      std::size_t candidate = 0U; while (used.contains(candidate)) ++candidate; color[own] = candidate;
      color_count = std::max(color_count, candidate + 1U); out.maximum_degree = std::max(out.maximum_degree, incident_contacts[own]); }
    out.colors = std::max(out.colors, color_count);
    for (const auto& contact : contacts) {
      auto [state, inserted] = duals.try_emplace(contact.key, Dual{0.0L, penalty});
      if (inserted || state->second.penalty == 0.0L) state->second.penalty = penalty;
    }
    std::vector<long double> contact_weight(contacts.size(), 1.0L);
    for (std::size_t ci = 0; ci < contacts.size(); ++ci) {
      std::size_t degree = 1U;
      for (const auto& term : contacts[ci].terms)
        if (!fixed[term.owner]) degree = std::max(degree, incident_contacts[term.owner]);
      contact_weight[ci] = 1.0L / static_cast<long double>(degree);
    }
    std::atomic<std::size_t> solves_in_color{0U};
    for (std::size_t col = 0; col < color_count; ++col) {
      std::vector<std::size_t> owners; for (const auto own : active_owners) if (color[own] == col && !fixed[own]) owners.push_back(own);
      // Owners of one color never share a contact. run_bounded_routing_tasks therefore has a
      // barrier between colors while retaining deterministic results for the one-owner blocks.
      run_bounded_routing_tasks(owners.size(), lease.workers(), stop, [&](std::size_t index) {
        routing_kernel_checkpoint(); const auto own = owners[index];
        long double tether = 0.0L;
        for (const auto member : members[own]) tether += 1.0e-6L / particles[member].inverse_mass;
        long double hxx = tether, hxy = 0, hyy = tether;
        Vec2 rhs = (initial[own] - pos[own]) * tether;
        for (const auto si : owner_segments[own]) { const auto& s = segments[si];
          if (owner[s.first_particle] == own && owner[s.second_particle] == own) continue;
          const auto delta = pos[s.second_particle] - pos[s.first_particle]; const auto len = length(delta);
          const auto rest = s.rest_length_nm > 0 ? static_cast<long double>(s.rest_length_nm) : length(initial[s.second_particle] - initial[s.first_particle]);
          if (!(len > 1.0e-15L) || !(rest > 0)) continue;
          const auto d = unit(delta); const auto residual = len - rest;
          const auto sign = owner[s.first_particle] == own ? -1.0L : 1.0L;
          const auto force = stretch_duals[si] + penalty * residual;
          rhs += d * (-sign * force); hxx += penalty*d.x*d.x; hxy += penalty*d.x*d.y; hyy += penalty*d.y*d.y;
        }
        const auto contact_residual = [&](const Contact& contact, Vec2 step) {
          auto residual = contact.residual;
          for (const auto& term : contact.terms) {
            const auto representative = members[term.owner].front();
            auto displacement = pos[representative] - color_snapshot[representative];
            if (term.owner == own) displacement += step;
            residual -= term.coefficient * dot(term.direction, displacement);
          }
          return residual;
        };
        for (const auto ci : owner_contacts[own]) for (const auto& term : contacts[ci].terms) if (term.owner == own) {
          const auto& c = contacts[ci]; const auto shared = contact_weight[ci];
          const auto residual = contact_residual(c, {});
          const auto state = duals.find(c.key)->second; const auto force = std::max(0.0L, state.lambda + state.penalty * residual);
          rhs += term.direction * (shared * term.coefficient * force); const auto q = shared * state.penalty * term.coefficient * term.coefficient;
          hxx += q*term.direction.x*term.direction.x; hxy += q*term.direction.x*term.direction.y; hyy += q*term.direction.y*term.direction.y;
        }
        const auto det = hxx*hyy - hxy*hxy; if (!(det > 1.0e-18L) || !std::isfinite(det)) return;
        Vec2 step{(hyy*rhs.x-hxy*rhs.y)/det, (hxx*rhs.y-hxy*rhs.x)/det};
        // The common contact weight belongs to the local block response as well as its
        // augmented terms.  This keeps a high-degree owner from taking a full contact-sized
        // displacement merely because the same scalar appears in both GN sides.
        step = step * (1.0L / static_cast<long double>(std::max<std::size_t>(1U, incident_contacts[own])));
        const auto max_step = static_cast<long double>(xpbd.maximum_step_nm); if (length(step) > max_step) step = unit(step) * max_step;
        long double corridor_scale = 1.0L;
        const auto step_squared = dot(step, step);
        for (const auto member : members[own]) if (particles[member].corridor_radius_nm > 0 && step_squared > 0.0L) {
          const Vec2 anchor{static_cast<long double>(particles[member].corridor_anchor_x_nm), static_cast<long double>(particles[member].corridor_anchor_y_nm)};
          const auto delta = pos[member] - anchor;
          const auto radius = static_cast<long double>(particles[member].corridor_radius_nm);
          const auto c = dot(delta, delta) - radius * radius;
          if (c > 1.0e-9L) { corridor_scale = 0.0L; break; }
          const auto b = 2.0L * dot(delta, step);
          const auto discriminant = b*b - 4.0L*step_squared*c;
          if (discriminant >= 0.0L) {
            const auto exit = (-b + std::sqrt(discriminant)) / (2.0L * step_squared);
            corridor_scale = std::min(corridor_scale, std::clamp(exit, 0.0L, 1.0L));
          }
        }
        step = step * corridor_scale;
        if (!finite(step)) return;
        // The block Hessian linearizes cord length. Limit nonlinear overshoot with a bounded
        // line search on this owner's augmented energy before the next color observes it.
        const auto local_energy = [&](Vec2 displacement) {
          const auto offset = pos[own] + displacement - initial[own];
          auto energy = 0.5L * tether * dot(offset, offset);
          for (const auto si : owner_segments[own]) {
            const auto& s = segments[si];
            if (owner[s.first_particle] == owner[s.second_particle]) continue;
            auto first = pos[s.first_particle], second = pos[s.second_particle];
            if (owner[s.first_particle] == own) first += displacement;
            if (owner[s.second_particle] == own) second += displacement;
            const auto rest = s.rest_length_nm > 0 ? nm(s.rest_length_nm) :
                length(initial[s.second_particle] - initial[s.first_particle]);
            const auto residual = length(second - first) - rest;
            energy += stretch_duals[si] * residual + 0.5L * penalty * residual * residual;
          }
          for (const auto ci : owner_contacts[own]) {
            const auto& c = contacts[ci]; const auto state = duals.find(c.key)->second;
            const auto force = std::max(0.0L, state.lambda + state.penalty * contact_residual(c, displacement));
            energy += contact_weight[ci] * force * force / (2.0L * state.penalty);
          }
          return energy;
        };
        const auto before = local_energy({});
        bool accepted = false;
        for (std::size_t backtrack = 0U; backtrack < 12U; ++backtrack) {
          const auto after = local_energy(step);
          if (std::isfinite(after) && after <= before + 1.0e-12L * std::max(1.0L, std::abs(before))) {
            accepted = true;
            break;
          }
          step = step * 0.5L;
        }
        if (!accepted) step = {};
        for (const auto member : members[own]) pos[member] += step;
        if (length(step) > 0 && assigned_to_group[own]) moved_rigid[own] = 1U;
        ++solves_in_color;
      });
    }
    out.local_block_solves += solves_in_color.load();
    bool post_overflow = false; contacts = build_contacts(post_overflow); if (post_overflow) return fail("AVBD contact budget exhausted");
    std::vector<std::size_t> post_incident(particles.size(), 0U);
    for (const auto& c : contacts) { std::set<std::size_t> owners; for (const auto& t : c.terms) owners.insert(t.owner); for (const auto own : owners) if (!fixed[own]) ++post_incident[own]; }
    contact_weight.assign(contacts.size(), 1.0L);
    for (std::size_t ci = 0; ci < contacts.size(); ++ci) { std::size_t degree = 1U; for (const auto& t : contacts[ci].terms) if (!fixed[t.owner]) degree = std::max(degree, post_incident[t.owner]); contact_weight[ci] = 1.0L / static_cast<long double>(degree); }
    auto current = metrics(contacts, pos, segments); out.relaxation.iteration_metrics.push_back(current); ++out.relaxation.iterations_executed;
    if (current.total_residual_nm < best_metrics.total_residual_nm) {
      best_metrics = current;
      best_positions = pos;
    }
    std::set<ContactKey> live;
    for (std::size_t ci = 0; ci < contacts.size(); ++ci) { const auto& c = contacts[ci]; live.insert(c.key); auto& state = duals[c.key]; if (state.penalty == 0) state.penalty = penalty; state.lambda = std::max(0.0L, state.lambda + contact_weight[ci] * state.penalty*c.residual); ++out.duals; }
    for (std::size_t si = 0; si < segments.size(); ++si) {
      const auto& s = segments[si]; const auto rest = s.rest_length_nm > 0 ? static_cast<long double>(s.rest_length_nm) : length(initial[s.second_particle]-initial[s.first_particle]);
      if (rest > 0) { stretch_duals[si] += penalty * (length(pos[s.second_particle]-pos[s.first_particle])-rest); ++out.duals; }
    }
    for (auto state = duals.begin(); state != duals.end();) state = live.contains(state->first) ? std::next(state) : duals.erase(state);
    if (current.total_residual_nm + 0.5L >= previous_residual && penalty < avbd.maximum_penalty) {
      penalty = std::min(avbd.maximum_penalty, penalty * avbd.penalty_growth);
      for (auto& [key, state] : duals) state.penalty = penalty;
      ++out.penalty_updates;
    }
    previous_residual = current.total_residual_nm;
  }
  if (stop.stop_requested()) {
    out.cancelled = true;
    return fail("AVBD relaxation cancelled");
  }
  // Dual updates can temporarily raise physical residual even while primal energy descends.
  // Publish the best complete iterate; never discard useful progress for the final iterate.
  pos = std::move(best_positions);
  bool final_overflow = false; auto final_contacts = build_contacts(final_overflow); if (final_overflow) return fail("AVBD contact budget exhausted");
  out.relaxation.final = metrics(final_contacts, pos, segments); out.relaxation.converged = out.relaxation.final.violations == 0;
  out.moved_rigid_owners = static_cast<std::size_t>(std::count(moved_rigid.begin(), moved_rigid.end(), 1U));
  out.relaxation.timings.solve_us = elapsed();
  out.relaxation.stalled = !out.relaxation.converged && out.relaxation.final.total_residual_nm + 0.5L >= out.relaxation.initial.total_residual_nm;
  out.relaxation.topology_escape_required = !out.relaxation.converged;
  out.relaxation.positions_nm.reserve(pos.size());
  for (const auto point : pos) {
    constexpr auto limit = static_cast<long double>(std::numeric_limits<std::int64_t>::max() - 1);
    if (!finite(point) || std::abs(point.x) > limit || std::abs(point.y) > limit)
      return fail("AVBD produced non-finite or out-of-range geometry");
    out.relaxation.positions_nm.emplace_back(static_cast<std::int64_t>(std::llround(point.x)), static_cast<std::int64_t>(std::llround(point.y)));
  }
  return out;
}

} // namespace flux
