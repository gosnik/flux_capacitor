#pragma once

#include "flux/core/free_space_offset.hpp"
#include "flux/core/free_space_skeleton.hpp"
#include "flux/core/geometry.hpp"
#include "flux/core/hydraulic_path.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace flux {

struct NavMeshConfig {
  Nanometres cell_pitch{Nanometres::from_mm(1.0)};
  std::size_t maximum_cells{500'000U};
  std::size_t maximum_search_expansions{500'000U};
  // Optional topology tube used when refining an already allocated route. Empty leaves topology
  // selection to free-space search; a positive radius retains only cells near the supplied path.
  std::vector<Point> topology_guide;
  Nanometres maximum_guide_distance{Nanometres{0}};
  std::size_t corridor_guard_rings{1U};
  // For an unoffset equivalent decomposition, require every retained cell chord and pulled
  // centerline chord to carry at least this source-boundary clearance certificate.
  Nanometres minimum_boundary_clearance{Nanometres{0}};
  // Bounds only the decomposed lattice for a regional routing window. The original free-space
  // contours remain authoritative for every cell and centerline certificate.
  std::optional<geometry::AxisAlignedBounds> search_bounds;
  // A coarse cell whose complete rectangle cannot be certified is not necessarily unusable: a
  // clearance boundary may clip only one corner while leaving a narrow connected passage. Split
  // such boundary cells recursively and retain every independently certified child. A zero
  // minimum pitch derives from the configured refinement depth;
  // maximum_refinement_depth remains the absolute bound.
  bool adaptive_clearance_refinement{true};
  std::size_t maximum_refinement_depth{1U};
  Nanometres minimum_refined_cell_pitch{Nanometres{0}};
};

// Bounded deterministic bidirectional RRT-Connect over the prepared NavMesh cell graph.  The
// low-discrepancy sampler selects topology; the ordinary NavMesh funnel and exact chord
// certificates still construct and validate geometry.  This is an alternative generator, never
// an authority for copper legality.
struct NavMeshRrtConnectConfig {
  std::size_t maximum_iterations{2'048U};
  std::uint64_t deterministic_seed{0U};
  std::size_t opened_guard_rings{0U};
  std::size_t goal_bias_period{8U};
};

// Bounds the LittlePlanets-inspired hydraulic free-space discovery proposal. Discovery rasterizes
// the prepared polygonal domain directly at a finer pitch; the coarse square decomposition is
// used only after discovery to account for/project the selected corridor. Ordinary flood/funnel
// topology and exact chord certification still reconstruct the returned route.
struct NavMeshHydraulicTerrainDeposit {
  geometry::AxisAlignedBounds bounds;
  Point axis_start;
  Point axis_end;
  bool directional{false};
  std::uint32_t pressure{0U};
  std::uint32_t success_prior{0U};

  bool operator==(const NavMeshHydraulicTerrainDeposit&) const = default;
};

struct NavMeshHydraulicConfig {
  HydraulicPathConfig flow;
  std::size_t opened_guard_rings{1U};
  std::size_t discovery_resolution_multiplier{2U};
  std::size_t maximum_discovery_cells{1'000'000U};
  // Already filtered to this net/layer by the caller. Deposits are advisory and cannot alter the
  // polygon-derived walkable mask.
  std::vector<NavMeshHydraulicTerrainDeposit> downstream_feedback;
};

struct NavMeshPortal {
  Point left;
  Point right;

  bool operator==(const NavMeshPortal&) const = default;
};

// A directed portal is the compact topology certificate carried forward from NavMesh search.  It
// records which adjacent cells the route leaves and enters, plus the two immutable portal-boundary
// sites.  The sites are deliberately geometry identities, not inferred KiCad objects: a grid
// portal can be bounded by free-space subdivision as well as by a physical obstacle.  Together,
// the ordered gates distinguish "same free-space union" from "same way around an obstacle".
struct NavMeshTopologyGate {
  std::size_t ordinal{0U};
  std::size_t from_cell{0U};
  std::size_t to_cell{0U};
  Point left_boundary;
  Point right_boundary;
  Point from_center;
  Point to_center;
  std::uint64_t left_boundary_id{0U};
  std::uint64_t right_boundary_id{0U};

  bool operator==(const NavMeshTopologyGate&) const = default;
};

// This is a route-class certificate, not a second authoritative geometry representation.  A
// zero-gate signature is valid for a route contained in one NavMesh cell; normal spline/PNS and
// exact copper gates remain required in either case.
struct NavMeshTopologySignature {
  std::vector<NavMeshTopologyGate> gates;
  std::uint64_t fingerprint{0U};
  bool valid{false};

  bool operator==(const NavMeshTopologySignature&) const = default;
};

enum class NavMeshTopologyCheckReason {
  preserved,
  invalid_signature,
  invalid_polyline,
  missing_gate,
  repeated_gate,
  reversed_gate,
  reordered_gate,
};

// A continuous route need not retain the NavMesh centreline, but it must cross every selected
// portal once, in the selected direction and order.  This is the explicit homotopy guard used by
// spline optimization and push/shove; changing class is a regional-template operation instead.
struct NavMeshTopologyCheck {
  bool preserved{false};
  NavMeshTopologyCheckReason reason{NavMeshTopologyCheckReason::invalid_signature};
  std::size_t gates_crossed{0U};
  std::size_t offending_gate{0U};
  std::string diagnostic;

  bool operator==(const NavMeshTopologyCheck&) const = default;
};

struct NavMeshCell {
  std::size_t id{0};
  std::array<Point, 4U> corners;
  Point center;
  std::size_t refinement_level{0U};

  [[nodiscard]] bool contains(Point point) const {
    return corners.front().x <= point.x && point.x <= corners[2].x &&
           corners.front().y <= point.y && point.y <= corners[2].y;
  }

  // Allocate a canonical polygon only when selected corridor evidence crosses the public
  // regional boundary. The complete prepared lattice remains allocation-free per cell.
  [[nodiscard]] Polygon polygon() const {
    Polygon result;
    result.vertices.assign(corners.begin(), corners.end());
    return result;
  }

  bool operator==(const NavMeshCell&) const = default;
};

// Exact shared-face adjacency for the mixed-resolution rectangular cell complex. One coarse cell
// may have several neighbours along a face when the obstacle side was recursively refined.
struct NavMeshAdjacency {
  std::size_t cell{0U};
  NavMeshPortal portal;
  Nanometres aperture{0};

  bool operator==(const NavMeshAdjacency&) const = default;
};

// Prepared endpoint searches and every accepted alternative share one immutable lattice. Copying a
// route must not copy tens of thousands of cells into each candidate template.
class NavMeshCells {
public:
  using const_iterator = std::vector<NavMeshCell>::const_iterator;

  [[nodiscard]] bool empty() const { return cells_->empty(); }
  [[nodiscard]] std::size_t size() const { return cells_->size(); }
  [[nodiscard]] const NavMeshCell& operator[](std::size_t index) const { return (*cells_)[index]; }
  [[nodiscard]] const_iterator begin() const { return cells_->begin(); }
  [[nodiscard]] const_iterator end() const { return cells_->end(); }

  void push_back(NavMeshCell cell) {
    if (!cells_.unique()) {
      cells_ = std::make_shared<std::vector<NavMeshCell>>(*cells_);
    }
    cells_->push_back(std::move(cell));
  }

  bool operator==(const NavMeshCells& other) const { return *cells_ == *other.cells_; }

private:
  std::shared_ptr<std::vector<NavMeshCell>> cells_{
      std::make_shared<std::vector<NavMeshCell>>()};
};

struct NavMeshCutoutNode {
  geometry::AxisAlignedBounds bounds{
      Point{Nanometres{0}, Nanometres{0}}, Point{Nanometres{0}, Nanometres{0}}};
  std::size_t first{0U};
  std::size_t count{0U};
  std::size_t left{0U};
  std::size_t right{0U};

  [[nodiscard]] bool leaf() const { return count > 0U; }
};

// Endpoint-independent immutable cell decomposition. Alternative topology seeds sharing a
// region/configuration build this once and perform only endpoint binding, graph search and funnel
// work for each route.
struct PreparedNavMeshDomain {
  FreeSpaceDomain free_space;
  geometry::PreparedPolygon prepared_outline;
  std::vector<geometry::PreparedPolygon> prepared_cutouts;
  std::vector<std::size_t> cutout_order;
  std::vector<NavMeshCutoutNode> cutout_bvh;
  Point minimum;
  Nanometres pitch;
  std::vector<Point> topology_guide;
  Nanometres maximum_guide_distance;
  Nanometres minimum_boundary_clearance;
  std::optional<geometry::AxisAlignedBounds> search_bounds;
  std::size_t width{0};
  std::size_t height{0};
  NavMeshCells cells;
  std::vector<std::pair<std::size_t, std::size_t>> coordinate_by_cell;
  std::vector<std::size_t> cell_by_coordinate;
  std::vector<std::vector<std::size_t>> cells_by_coordinate;
  std::vector<std::vector<NavMeshAdjacency>> adjacency_by_cell;
  bool adaptive_clearance_refinement{false};
  std::size_t maximum_refinement_depth{0U};
  Nanometres minimum_refined_cell_pitch{Nanometres{0}};
  std::size_t refined_parent_cells{0U};
  std::size_t refined_leaf_cells{0U};
  std::size_t rejected_leaf_cells{0U};
  std::size_t maximum_realized_refinement_depth{0U};
  bool cell_budget_exhausted{false};
  std::string diagnostic;

  [[nodiscard]] bool ok() const { return diagnostic.empty() && !cells.empty(); }
};

struct PreparedNavMeshPointQuery {
  bool inside{false};
  long double boundary_distance_nm{0.0L};
};

// Shared allocation-free point location and nearest-boundary query over the prepared outline and
// cutout BVHs. Continuous route optimization uses this instead of linearly visiting every hole
// for every position and finite-difference sample.
[[nodiscard]] PreparedNavMeshPointQuery
query_prepared_navmesh_point(const PreparedNavMeshDomain& domain, Point point);

struct NavMeshRouteResult {
  Point start;
  Point goal;
  NavMeshCells cells;
  std::vector<std::size_t> corridor;
  std::vector<std::size_t> topology_guard_cells;
  std::vector<NavMeshPortal> portals;
  NavMeshTopologySignature topology_signature;
  std::vector<Point> centerline;
  Point cell_grid_minimum;
  Nanometres cell_pitch;
  std::size_t cell_grid_width{0};
  std::size_t cell_grid_height{0};
  std::vector<std::uint8_t> topology_guard_coordinate_mask;
  std::size_t search_expansions{0};
  std::size_t flood_fill_cells_visited{0};
  std::size_t adaptive_refined_parent_cells{0U};
  std::size_t adaptive_refined_leaf_cells{0U};
  std::size_t adaptive_rejected_leaf_cells{0U};
  std::size_t adaptive_maximum_refinement_depth{0U};
  std::size_t rrt_connect_iterations{0};
  std::size_t hydraulic_relaxation_passes{0};
  std::size_t hydraulic_droplet_steps{0};
  std::size_t hydraulic_droplets_reaching_target{0};
  Point hydraulic_field_origin;
  Nanometres hydraulic_field_cell_pitch;
  std::size_t hydraulic_discovery_cells{0};
  std::size_t hydraulic_exact_certified_chords{0};
  bool hydraulic_direct_polygon_discovery{false};
  // Bounded diagnostic-only field. Flow/erosion samples are block maxima, terrain is the block
  // valley minimum, and free-space is fractional coverage. Scalar fields are normalized to 16
  // bits against the corresponding maximum below.
  std::size_t hydraulic_field_width{0};
  std::size_t hydraulic_field_height{0};
  std::size_t hydraulic_field_stride{0};
  std::uint32_t hydraulic_maximum_flow{0};
  std::uint32_t hydraulic_maximum_erosion{0};
  std::uint32_t hydraulic_maximum_terrain_height{0};
  std::uint32_t hydraulic_maximum_downstream_pressure{0};
  std::uint32_t hydraulic_maximum_success_prior{0};
  std::vector<std::uint16_t> hydraulic_flow_field;
  std::vector<std::uint16_t> hydraulic_erosion_field;
  // Complete reusable heightmap and free-space coverage. Height is zero at the destination
  // valley and rises away from it; erosion lowers high-flow riverbeds. Free-space coverage keeps
  // pads, vias, keepouts, and board-exterior cells out of the terrain.
  std::vector<std::uint16_t> hydraulic_heightmap;
  std::vector<std::uint16_t> hydraulic_free_space_field;
  std::vector<std::uint16_t> hydraulic_downstream_pressure_field;
  std::vector<std::uint16_t> hydraulic_success_prior_field;
  bool used_flood_fill{false};
  bool used_gpu_wavefront{false};
  bool gpu_wavefront_cpu_fallback{false};
  bool used_rrt_connect{false};
  bool used_hydraulic_erosion{false};
  bool used_hydraulic_gpu{false};
  bool hydraulic_gpu_clearance_cache_hit{false};
  std::size_t hydraulic_gpu_batch_size{0U};
  std::size_t hydraulic_gpu_buffer_reuses{0U};
  std::size_t hydraulic_gpu_buffer_growths{0U};
  bool hydraulic_cpu_fallback{false};
  bool cell_budget_exhausted{false};
  bool search_budget_exhausted{false};
  std::string diagnostic;

  [[nodiscard]] bool ok() const {
    return diagnostic.empty() && centerline.size() >= 2U && !corridor.empty();
  }
};

// Tests membership in the selected closed cell corridor. This is the topology guard used by
// continuous optimization: a legal free-space point outside the selected corridor is still
// rejected so smoothing cannot silently change obstacle side.
[[nodiscard]] bool navmesh_corridor_contains(const NavMeshRouteResult& route, Point point);

// Derives the immutable directed gate sequence from a selected NavMesh cell corridor.  The helper
// is public so regional templates can retain and concatenate certificates without copying a full
// prepared NavMesh domain.
[[nodiscard]] NavMeshTopologySignature
derive_navmesh_topology_signature(const NavMeshRouteResult& route);

// Retains an already selected ordered gate sequence, for example when a regional template joins
// two bounded NavMesh searches at a validated waypoint.  It renumbers only the diagnostic ordinal
// and recomputes the deterministic fingerprint; it never searches or relaxes geometry.
[[nodiscard]] NavMeshTopologySignature
make_navmesh_topology_signature(std::vector<NavMeshTopologyGate> gates);

// Validates a quantized polyline against the directed gate sequence.  The polyline is expected to
// be a spline tessellation or an exact proposed centreline; it is not a DRC substitute.
[[nodiscard]] NavMeshTopologyCheck
validate_navmesh_topology_signature(const NavMeshTopologySignature& signature,
                                    const std::vector<Point>& polyline);

[[nodiscard]] PreparedNavMeshDomain prepare_navmesh_domain(const OffsetFreeSpaceRegion& region,
                                                           const NavMeshConfig& config = {});

// Fast path for a region returned by offset_polygonal_free_space(). That producer has already
// exact-validated polygon simplicity, strict hole containment and pairwise disjointness. The cell
// and chord certificates below remain unchanged; only duplicate source-topology validation is
// omitted.
[[nodiscard]] PreparedNavMeshDomain
prepare_validated_navmesh_domain(const OffsetFreeSpaceRegion& region,
                                 const NavMeshConfig& config = {});

[[nodiscard]] NavMeshRouteResult route_prepared_navmesh(const PreparedNavMeshDomain& domain,
                                                        Point start, Point goal,
                                                        const NavMeshConfig& config = {});

// Produces a deliberately different, potentially longer homotopy candidate.  Failure is explicit
// and does not silently return the shortest flood-fill route, allowing callers to retain the
// ordinary route as an independent fallback family.
[[nodiscard]] NavMeshRouteResult route_prepared_navmesh_rrt_connect(
    const PreparedNavMeshDomain& domain, Point start, Point goal,
    const NavMeshConfig& navmesh_config = {},
    const NavMeshRrtConnectConfig& rrt_config = {});

[[nodiscard]] NavMeshRouteResult route_prepared_navmesh_hydraulic(
    const PreparedNavMeshDomain& domain, Point start, Point goal,
    const NavMeshConfig& navmesh_config = {},
    const NavMeshHydraulicConfig& hydraulic_config = {});

// Runs a bounded reverse multi-source flood over the immutable cell maze, then reconstructs one
// deterministic start-to-goal corridor by following decreasing integer distance labels. This is
// advisory topology evidence; the returned funnel remains subject to all ordinary exact checks.
[[nodiscard]] NavMeshRouteResult
route_prepared_navmesh_flood_fill(const PreparedNavMeshDomain& domain, Point start,
                                  const std::vector<Point>& goals, const NavMeshConfig& config = {},
                                  const std::vector<std::size_t>& dynamically_closed_cells = {});

// Builds a bounded rectilinear polygonal NavMesh inside a represented free-space region, searches
// cell adjacency, then applies the funnel/string-pulling algorithm to its ordered portals. The
// region is normally already clearance-offset; minimum_boundary_clearance provides the equivalent
// source-certificate mode. Every cell edge and diagonal is independently rechecked.
[[nodiscard]] NavMeshRouteResult build_navmesh_funnel_route(const OffsetFreeSpaceRegion& region,
                                                            Point start, Point goal,
                                                            const NavMeshConfig& config = {});

} // namespace flux
