#pragma once

#include "flux/core/connectivity.hpp"
#include "flux/core/drc.hpp"
#include "flux/core/global_router.hpp"
#include "flux/core/routability_knowledge.hpp"

#include <cstddef>
#include <cstdint>
#include <map>
#include <stop_token>
#include <string>
#include <vector>

namespace flux {

// A deliberately cheap bridge between global topology and regional continuous realization.
// KRT searches directly in a fine quantized copper space; Flux retains its richer NavMesh,
// hydraulic, corridor, and negotiated-resource discovery, then uses this probe to ask the same
// physical question before committing to an expensive regional PH search.  The result is
// diagnostic only: it can add routability pressure, but can never authorize final copper.
struct QuantizedCopperOracleConfig {
  bool enabled{true};
  // Match final native copper's integer-nanometre coordinates by default. A coarser
  // diagnostic lattice can both create static-obstacle conflicts and hide real
  // inter-net conflicts; it must be an explicit experiment, not default feedback.
  Nanometres grid_pitch{1};
  // Exact pad endpoints remain unsnapped.  This preserves the terminal contact selected by the
  // global router. A coincident endpoint via chain is protected as one site. Source-owned
  // reusable vias/plated drills are always immutable anchors, including interior transitions;
  // other interior bends and new vias are snapped together on the probe grid.
  bool preserve_path_endpoints{true};
  Nanometres clearance_severity_quantum{Nanometres::from_mm(0.05)};
  Nanometres diagnosis_coalescing_pitch{Nanometres::from_mm(1.0)};
  std::size_t max_diagnoses{2'048U};

  bool operator==(const QuantizedCopperOracleConfig&) const = default;
};

struct QuantizedCopperOracleResult {
  Nanometres configured_grid_pitch;
  bool evaluated{false};
  bool all_paths_materialized{false};
  bool exact_copper_legal{false};
  std::size_t paths_examined{0U};
  std::size_t paths_materialized{0U};
  std::size_t generated_tracks{0U};
  std::size_t generated_vias{0U};
  std::size_t reused_same_net_transitions{0U};
  std::size_t malformed_paths{0U};
  std::size_t drc_violations{0U};
  // Full raw DRC category counts, independent of coalesced/bounded routing diagnoses.
  std::map<std::string, std::size_t> drc_violation_counts;
  std::size_t open_connections{0U};
  std::size_t unrouted_nets{0U};
  std::uint64_t elapsed_us{0U};
  std::vector<RoutabilityDiagnosis> diagnoses;
  std::string diagnostic;

  [[nodiscard]] bool feasible() const {
    return evaluated && all_paths_materialized && exact_copper_legal && open_connections == 0U;
  }

  bool operator==(const QuantizedCopperOracleResult&) const = default;
};

// Materializes every selected global path as straight, rule-width copper on a fine deterministic
// lattice, represents each complete same-site layer-transition chain as one physical via span,
// and runs Flux's exact dynamic-copper DRC plus exact connectivity.  Unlike clearance shadows,
// the returned diagnoses are based on the same integer track/via primitives ultimately exported
// to KiCad.  PH/rubber-band geometry remains a later realization stage.
[[nodiscard]] QuantizedCopperOracleResult probe_quantized_copper_feasibility(
    const Board& board, const GlobalRouteResult& global,
    const QuantizedCopperOracleConfig& config = {}, std::stop_token stop_token = {});

} // namespace flux
