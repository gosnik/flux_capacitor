#pragma once

#include "flux/core/free_space_offset.hpp"
#include "flux/core/free_space_skeleton.hpp"
#include "flux/core/geometry.hpp"

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
  bool cell_budget_exhausted{false};
  std::string diagnostic;

  [[nodiscard]] bool ok() const { return diagnostic.empty() && !cells.empty(); }
};

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
  bool used_flood_fill{false};
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
