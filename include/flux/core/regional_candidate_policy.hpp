#pragma once

#include "flux/core/regional_router.hpp"

#include <cstddef>
#include <optional>
#include <span>
#include <vector>

namespace flux {

// A preflight failure is deliberately a proof about immutable candidate evidence.  It is not a
// report of an unsuccessful continuous realization: callers must retain candidates whose routes
// could still move inside their selected topology.
enum class RegionalCandidateInfeasibilityKind : std::uint8_t {
  fixed_endpoint_envelope_overlap,
  cross_section_packing_impossible,
};

struct RegionalCandidateInfeasibility {
  RegionalCandidateInfeasibilityKind kind;
  RegionalRouteFragmentId first;
  RegionalRouteFragmentId second;
  LayerId layer;
  std::optional<std::size_t> channel_id;
  std::optional<std::size_t> cross_section_id;

  bool operator==(const RegionalCandidateInfeasibility&) const = default;
};

struct RegionalCandidatePreflight {
  std::vector<RegionalCandidateInfeasibility> infeasibilities;

  [[nodiscard]] bool definitively_infeasible() const { return !infeasibilities.empty(); }
};

// Determines whether an outcome is a monotonic proof suitable for a hard regional-master
// no-good. Continuous/native realization failures and any bounded or truncated probe remain
// candidate-specific evidence, because another deformation of the same discrete assignment may
// still be legal.
[[nodiscard]] bool regional_assignment_is_no_good_certificate(
    RegionalBundleStopReason stop_reason, bool shape_sample_budget_exhausted = false);

// Performs only cheap, monotonic checks.  In particular, it does not inspect a realization's
// centreline: a crossing between movable routes is not a proof that the selected candidate is
// impossible.  Terminal endpoints, in contrast, are fixed by the canonical board connection.
[[nodiscard]] RegionalCandidatePreflight preflight_regional_candidate(
    const Board& board, const RegionalRouteResult& regional,
    const RegionalBundleAssignment& assignment);

struct RegionalCandidateScheduleEntry {
  std::size_t assignment_index{0U};
  RegionalCandidatePreflight preflight;
  std::size_t known_template_conflicts{0U};
  std::size_t known_residual_conflicts{0U};
  // Present only when the assignment has realizations for every selected fragment.  It counts
  // fragments which have reached all three local legality gates; it is advisory scheduling
  // evidence and never becomes an infeasibility certificate.
  std::optional<std::size_t> conflict_free_realization_progress;
};

// A structural identity for learning from an evaluated candidate without tying the policy to its
// transient position in a shortlist. The IDs are derived regional evidence, not board identity.
struct RegionalCandidateScheduleSignature {
  std::vector<std::size_t> template_ids;
  std::vector<std::size_t> via_choices;
  std::vector<std::vector<RegionalRouteFragmentId>> lane_orders;

  bool operator==(const RegionalCandidateScheduleSignature&) const = default;
};

struct RegionalCandidateEvaluation {
  // The exact post-realization error count for this candidate. This is supplied by the ordered
  // reducer, so parallel completion timing cannot perturb later work ordering.
  std::size_t physical_violations{0U};
  std::size_t conflict_free_realizations{0U};
  bool complete{false};
};

struct RegionalCandidateSchedulingState {
  struct Observation {
    RegionalCandidateScheduleSignature signature;
    RegionalCandidateEvaluation evaluation;
  };
  // Append in deterministic reducer order. The newest matching observation is used for the
  // next batch; callers can discard the state at a regional invocation boundary.
  std::vector<Observation> observations;
};

[[nodiscard]] RegionalCandidateScheduleSignature regional_candidate_schedule_signature(
    const RegionalBundleAssignment& assignment);

void record_regional_candidate_evaluation(RegionalCandidateSchedulingState& state,
                                          RegionalCandidateScheduleSignature signature,
                                          RegionalCandidateEvaluation evaluation);

// Returns a deterministic work order. Definitively impossible candidates are last; otherwise
// known packing/residual/template conflict evidence is minimized. Realization progress breaks
// otherwise equivalent candidates when it is comparably available, followed by input position.
[[nodiscard]] std::vector<RegionalCandidateScheduleEntry> schedule_regional_candidates(
    const Board& board, const RegionalRouteResult& regional,
    std::span<const RegionalBundleAssignment> assignments,
    const RegionalCandidateSchedulingState* state = nullptr);

} // namespace flux
