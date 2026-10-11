#pragma once

#include "flux/core/geometry.hpp"

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace flux {

struct GlobalRouteResult;
struct RegionalRouteResult;
struct DetailedRouteResult;

enum class RoutabilityFailureKind {
  packing_capacity_exceeded,
  terminal_escape_ordering,
  via_site_exhausted,
  curvature_infeasible,
  shove_deadlock,
  quantization_clearance,
  topological_trap,
  layer_transition_conflict,
  packing_unresolved
};

enum class RoutabilityFailureStage {
  global_probe,
  regional_discrete,
  spline_realization,
  local_pns,
  quantization,
  final_validation
};

enum class RoutabilityRelaxation {
  reduce_resource_demand,
  change_terminal_order,
  change_via_site,
  change_lane_order,
  change_layer_sequence,
  change_corridor,
  increase_local_clearance_margin,
  replan_global_topology
};

struct RoutabilityResourceReference {
  std::string stable_id;
  LayerId layer;
  geometry::AxisAlignedBounds bounds;
  std::vector<NetId> nets;

  bool operator==(const RoutabilityResourceReference&) const = default;
};

// Identity within the exact global checkpoint probed by the copper oracle. Original (not
// quantized) endpoints allow consumers to reject stale provenance after topology changes.
struct RoutabilitySourceSegment {
  NetId net;
  std::size_t connection_index{0U};
  std::size_t segment_index{0U};
  LayerId layer;
  Point start;
  Point end;
  bool operator==(const RoutabilitySourceSegment&) const = default;
};

struct RoutabilitySourceSegmentPair {
  RoutabilitySourceSegment first;
  RoutabilitySourceSegment second;
  bool operator==(const RoutabilitySourceSegmentPair&) const = default;
};

struct RoutabilityDiagnosis {
  RoutabilityFailureKind kind{RoutabilityFailureKind::topological_trap};
  RoutabilityFailureStage stage{RoutabilityFailureStage::global_probe};
  RoutabilityResourceReference resource;
  std::uint64_t severity{1U};
  Nanometres clearance_deficit;
  std::optional<std::uint64_t> planned_capacity;
  std::optional<std::uint64_t> observed_capacity;
  RoutabilityRelaxation suggested_relaxation{RoutabilityRelaxation::replan_global_topology};
  std::string detail;
  // A local/regional alternative discharged this observation before upstream escalation.
  bool locally_resolved{false};
  // Some observations are already negotiated by a more specific PathFinder history (for example
  // an exact packed-lane bridge).  Retain them for attribution, telemetry, and affected-cone
  // expansion without charging the same failure a second time through generic physical history.
  bool contributes_to_history{true};
  // Preserve pair associations when spatial diagnoses coalesce, rather than generating a
  // cross-product of nearby same-net branches. Via failures identify their attached
  // planar routing edges as repair owners, not as an assertion that those edges collide.
  // Empty when exact routing ownership is unavailable.
  std::vector<RoutabilitySourceSegmentPair> source_segment_pairs{};

  bool operator==(const RoutabilityDiagnosis&) const = default;
};

struct RoutabilityHistoryEntry {
  RoutabilityFailureKind kind{RoutabilityFailureKind::topological_trap};
  RoutabilityResourceReference resource;
  std::size_t observations{0U};
  std::uint64_t pressure{0U};
  RoutabilityFailureStage stage{RoutabilityFailureStage::global_probe};
  // Retain the corrective action selected by the stage which produced this pressure.  Earlier
  // aggregation discarded it, so a later epoch knew that a region was expensive but not whether
  // to change its lane order, via site, layer sequence, or complete topology.
  RoutabilityRelaxation suggested_relaxation{RoutabilityRelaxation::replan_global_topology};

  bool operator==(const RoutabilityHistoryEntry&) const = default;
};

// Positive evidence is deliberately separate from failure history. It may attract hydraulic
// discovery but never discounts PathFinder congestion, capacity, clearance, or DRC costs.
struct RoutabilitySuccessEntry {
  RoutabilityResourceReference resource;
  Point start;
  Point end;
  std::size_t observations{0U};
  std::uint64_t strength{0U};

  bool operator==(const RoutabilitySuccessEntry&) const = default;
};

struct RoutabilityKnowledge {
  std::vector<RoutabilityDiagnosis> diagnoses;
  std::vector<RoutabilityHistoryEntry> history;
  std::vector<RoutabilitySuccessEntry> success_history;
  std::size_t coalesced_diagnoses{0U};
  std::size_t dropped_diagnoses{0U};
  std::size_t dropped_history_entries{0U};

  [[nodiscard]] bool plausible() const { return diagnoses.empty(); }
  [[nodiscard]] std::uint64_t pressure() const;

  bool operator==(const RoutabilityKnowledge&) const = default;
};

struct RoutabilityProbeConfig {
  Nanometres coalescing_pitch{Nanometres::from_mm(1.0)};
  Nanometres clearance_severity_quantum{Nanometres::from_mm(0.05)};
  std::size_t max_diagnoses{4'096U};
  std::size_t max_history_entries{4'096U};
  // An exact quantized-copper oracle may own physical feedback. In that mode the global shadow
  // witnesses remain in diagnoses for telemetry and repair-scope attribution, but must not also
  // deposit generic regional pressure.
  bool clearance_shadows_contribute_to_history{true};

  bool operator==(const RoutabilityProbeConfig&) const = default;
};

[[nodiscard]] std::string_view to_string(RoutabilityFailureKind kind);
[[nodiscard]] std::string_view to_string(RoutabilityFailureStage stage);
[[nodiscard]] std::string_view to_string(RoutabilityRelaxation relaxation);

// O(selected conflict evidence): no free-space construction, spline fitting, or DRC.
[[nodiscard]] std::vector<RoutabilityDiagnosis>
probe_global_routability(const GlobalRouteResult& global,
                         const RoutabilityProbeConfig& config = {});

// Converts retained regional stop states into the same cross-loop diagnosis vocabulary.
[[nodiscard]] std::vector<RoutabilityDiagnosis>
diagnose_regional_routability(const RegionalRouteResult& regional,
                              const RoutabilityProbeConfig& config = {});

// Classifies the final transactional materialization/connectivity/DRC boundary. These diagnoses
// explain rejection and may seed a later bounded planner pass; they never alter the failed board.
[[nodiscard]] std::vector<RoutabilityDiagnosis>
diagnose_detailed_routability(const DetailedRouteResult& detailed,
                              const RoutabilityProbeConfig& config = {});

// Deterministic, bounded, saturating accumulation. Knowledge is a cost source only.
void accumulate_routability_knowledge(RoutabilityKnowledge& knowledge,
                                      const std::vector<RoutabilityDiagnosis>& diagnoses,
                                      const RoutabilityProbeConfig& config = {});

// Retains bounded positive terrain evidence for candidate copper objects that survived exact
// quantization and represented DRC. Repeated observations deepen the same coalesced prior slowly.
void accumulate_clean_copper_knowledge(RoutabilityKnowledge& knowledge,
                                       const DetailedRouteResult& detailed,
                                       const RoutabilityProbeConfig& config = {});

} // namespace flux
