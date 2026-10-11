#pragma once

#include "flux/core/board.hpp"

#include <cstddef>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace flux {

// Polygonal free-space input for the clean-room allocation skeleton. Boundary segments remain
// authoritative; the quantized Voronoi graph is advisory and must never bypass exact geometry.
struct FreeSpaceDomain {
  Polygon outline;
  std::vector<Polygon> cutouts;
  std::vector<Polygon> obstacles;
  std::vector<CopperZoneFill> filled_obstacles;
};

struct FreeSpaceSkeletonConfig {
  Nanometres minimum_clearance;
  std::size_t maximum_boundary_segments{200'000};
  bool checked_curved_bisector_chords{false};
};

struct FreeSpaceSkeletonEdge {
  std::size_t id{0};
  std::size_t start_node{0};
  std::size_t end_node{0};
  Point start;
  Point end;
  Nanometres minimum_boundary_clearance;
  // A straight, exact-rechecked surrogate joining the endpoints of a parabolic point/segment
  // bisector. It is capacity-bearing only at its recomputed chord clearance.
  bool curved_bisector_chord{false};

  bool operator==(const FreeSpaceSkeletonEdge&) const = default;
};

struct FreeSpaceSkeletonNode {
  std::size_t id{0};
  Point position;
  Nanometres boundary_clearance;
  std::size_t degree{0};

  [[nodiscard]] bool junction() const { return degree > 2U; }
  bool operator==(const FreeSpaceSkeletonNode&) const = default;
};

struct FreeSpaceSkeletonResult {
  std::vector<FreeSpaceSkeletonEdge> edges;
  std::vector<FreeSpaceSkeletonNode> nodes;
  std::size_t junctions{0};
  std::size_t boundary_segments{0};
  std::size_t skipped_infinite_edges{0};
  std::size_t curved_edges_seen{0};
  std::size_t accepted_curved_edge_chords{0};
  std::size_t skipped_curved_edges{0};
  std::size_t rejected_outside_edges{0};
  std::size_t rejected_clearance_edges{0};
  std::string diagnostic;

  [[nodiscard]] bool ok() const { return diagnostic.empty(); }
};

// Constructs finite straight Voronoi edges plus safe straight endpoint-chord surrogates for
// parabolic point/segment bisectors. Every quantized chord is independently checked against the
// represented polygonal free space and clearance; rejected curved proposals remain observable.
[[nodiscard]] FreeSpaceSkeletonResult
build_free_space_skeleton(const FreeSpaceDomain& domain,
                          const FreeSpaceSkeletonConfig& config = {});

// Rechecks an arbitrary quantized chord against the same polygonal-domain semantics used by the
// skeleton builder. Returns the floored minimum boundary clearance only when the entire chord is
// in represented free space. This is useful when a consumer must move a skeleton endpoint onto a
// different quantization lattice without inheriting the source edge's clearance certificate.
[[nodiscard]] std::optional<Nanometres> assess_free_space_chord(const FreeSpaceDomain& domain,
                                                                Point start, Point end);

// Validates the topology accepted by chord assessment without evaluating a chord. Consumers that
// build an immutable prepared query representation use this once before serving many exact
// geometric queries; overlapping ordinary obstacles are accepted as one physical union.
[[nodiscard]] bool validate_free_space_chord_domain(const FreeSpaceDomain& domain,
                                                    std::string* diagnostic = nullptr);

// Batch form of assess_free_space_chord. The domain is validated once and output indices match
// input indices, which keeps large skeleton-to-search-graph promotion bounded in practice.
// Overlapping obstacle contours are treated as one obstacle union for chord assessment; skeleton
// construction retains its stricter disjoint-topology requirement.
[[nodiscard]] std::vector<std::optional<Nanometres>>
assess_free_space_chords(const FreeSpaceDomain& domain,
                         const std::vector<std::pair<Point, Point>>& chords);

// Batch chord assessment for a domain which the caller has already accepted with
// validate_free_space_chord_domain().  Geometry and clearance semantics are identical to
// assess_free_space_chords(); only the invariant polygon validation pass is omitted.  This keeps
// repeated local certificates fail-closed without paying O(boundary^2) validation per batch.
[[nodiscard]] std::vector<std::optional<Nanometres>>
assess_validated_free_space_chords(const FreeSpaceDomain& domain,
                                   const std::vector<std::pair<Point, Point>>& chords);

// Conservative equal-width lane count for an accepted medial edge. The local channel width is
// twice the edge's verified nearest-boundary clearance; each additional lane consumes one trace
// width plus one inter-lane clearance.
[[nodiscard]] std::size_t skeleton_lane_capacity(const FreeSpaceSkeletonEdge& edge,
                                                 Nanometres track_width,
                                                 Nanometres inter_lane_clearance);

} // namespace flux
