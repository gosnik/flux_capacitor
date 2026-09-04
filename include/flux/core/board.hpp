#pragma once

#include "flux/core/ids.hpp"
#include "flux/core/units.hpp"

#include <cstdint>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace flux {

inline constexpr std::uint32_t board_schema_version = 19;
inline constexpr std::int64_t max_abs_coordinate_nm = 1'000'000'000;

struct Point {
  Nanometres x;
  Nanometres y;

  auto operator<=>(const Point&) const = default;
};

[[nodiscard]] constexpr Point operator+(Point lhs, Point rhs) {
  return Point{lhs.x + rhs.x, lhs.y + rhs.y};
}

[[nodiscard]] constexpr Point operator-(Point lhs, Point rhs) {
  return Point{lhs.x - rhs.x, lhs.y - rhs.y};
}

struct Polygon {
  std::vector<Point> vertices;

  bool operator==(const Polygon&) const = default;
};

enum class BoardSide { front, back };
enum class LayerKind { copper, technical };
enum class PadShape { circle, rectangle, oval, round_rectangle, trapezoid, custom, unknown };

struct KeepoutRules {
  bool tracks{false};
  bool vias{false};
  bool pads{false};
  bool copper_pours{false};
  bool footprints{false};

  [[nodiscard]] bool any() const { return tracks || vias || pads || copper_pours || footprints; }
  bool operator==(const KeepoutRules&) const = default;
};

struct Keepout {
  ObjectId id;
  Polygon area;
  std::vector<LayerId> layers;
  KeepoutRules prohibit;
  std::string name;

  bool operator==(const Keepout&) const = default;
};

struct Layer {
  LayerId id;
  std::string name;
  LayerKind kind{LayerKind::copper};
  std::uint32_t stack_order{0};

  bool operator==(const Layer&) const = default;
};

// Properties are independently inherited. Higher numeric priority wins for a property, and
// BoardDesignRules supplies the fallback when no assigned class defines it.
struct NetClass {
  NetClassId id;
  std::string name;
  std::uint32_t priority{0};
  std::optional<Nanometres> clearance;
  std::optional<Nanometres> track_width;
  std::optional<Nanometres> via_diameter;
  std::optional<Nanometres> via_drill;
  std::optional<Nanometres> differential_pair_track_width;
  std::optional<Nanometres> differential_pair_gap;
  std::optional<Nanometres> differential_pair_via_gap;

  bool operator==(const NetClass&) const = default;
};

struct Net {
  NetId id;
  std::string name;
  std::uint32_t priority{0};
  std::vector<NetClassId> classes{};

  bool operator==(const Net&) const = default;
};

// Preferred pair width/gap comes from independently resolved net-class routing defaults.
// These optional values are hard acceptance constraints and must have explicit provenance.
struct DifferentialPair {
  DifferentialPairId id;
  std::string name;
  NetId positive_net;
  NetId negative_net;
  std::optional<Nanometres> minimum_gap;
  std::optional<Nanometres> maximum_skew;

  bool operator==(const DifferentialPair&) const = default;
};

// An explicit physical parallel-routing constraint. Member order is significant and must not be
// inferred from a KiCad schematic bus or a net-name pattern.
struct RoutingBus {
  RoutingBusId id;
  std::string name;
  std::vector<NetId> ordered_nets;
  std::optional<Nanometres> minimum_gap;

  bool operator==(const RoutingBus&) const = default;
};

// A designer-declared rigid placement cluster. Membership is explicit stable-ID policy and must
// never be inferred from connectivity, references, differential pairs, or routing buses.
struct PlacementGroup {
  PlacementGroupId id;
  std::string name;
  std::vector<ObjectId> footprints;

  bool operator==(const PlacementGroup&) const = default;
};

// A designer-declared world-coordinate containment region. Membership is explicit and the full
// transformed authoritative placement envelope, not merely the anchor, must remain inside it.
struct PlacementRegion {
  PlacementRegionId id;
  std::string name;
  Polygon area;
  std::vector<ObjectId> footprints;

  bool operator==(const PlacementRegion&) const = default;
};

// A source-bound inclusive axis-aligned tolerance around one nominal anchor-to-anchor vector.
// Endpoint order defines the nominal vector; pair uniqueness is evaluated without order.
struct RelativePlacementConstraint {
  RelativePlacementConstraintId id;
  std::string name;
  ObjectId first_footprint;
  ObjectId second_footprint;
  Point nominal_offset;
  Nanometres nominal_distance;
  Microdegrees nominal_relative_rotation;
  Nanometres maximum_x_deviation;
  Nanometres maximum_y_deviation;
  std::optional<Nanometres> maximum_distance_deviation;
  std::optional<std::uint64_t> maximum_rotation_deviation_microdegrees;

  bool operator==(const RelativePlacementConstraint&) const = default;
};

struct Pad {
  ObjectId id;
  NetId net;
  Point local_position;
  std::vector<LayerId> layers;
  PadShape shape{PadShape::circle};
  Nanometres width{Nanometres::from_mm(1.0)};
  Nanometres height{Nanometres::from_mm(1.0)};
  Nanometres corner_radius{Nanometres{0}};
  // Footprint-local orientation. KiCad board files serialize an absolute pad angle, so the
  // importer must remove the placed footprint transform before populating this field.
  Microdegrees rotation;
  // Exact pad-local drill geometry. A circular drill has equal dimensions; an oval drill is a
  // capsule aligned with the pad-local X/Y axes before pad and footprint rotation. The offset is
  // measured from the pad anchor in that same local drill frame.
  Nanometres drill_width{Nanometres{0}};
  Nanometres drill_height{Nanometres{0}};
  Point drill_offset;
  bool plated{true};

  bool operator==(const Pad&) const = default;
};

// Empty allowed_rotations means unrestricted. Side flags are explicit because a polarized,
// connectorized, or mechanically constrained component may be legal on only one board side.
struct FootprintOrientationConstraints {
  std::vector<Microdegrees> allowed_rotations;
  bool allow_front{true};
  bool allow_back{true};

  bool operator==(const FootprintOrientationConstraints&) const = default;
};

enum class PlacementEnvelopeSource {
  explicit_courtyard,
  external_constraint,
  approximated_from_pads,
  tentative_from_geometry,
  absent
};

// Exact local geometry for a circular KiCad courtyard primitive. The polygonal courtyard remains
// a conservative search proxy; center and edge preserve the authoritative source circle.
struct CourtyardCircle {
  Point center;
  Point edge;

  bool operator==(const CourtyardCircle&) const = default;
};

struct Footprint {
  ObjectId id;
  std::string reference;
  Point position;
  Microdegrees rotation;
  BoardSide side{BoardSide::front};
  bool locked{false};
  FootprintOrientationConstraints orientation_constraints;
  Polygon courtyard;
  std::vector<CourtyardCircle> courtyard_circles;
  std::vector<Pad> pads;
  std::vector<Keepout> keepouts;
  PlacementEnvelopeSource placement_envelope_source{PlacementEnvelopeSource::explicit_courtyard};

  [[nodiscard]] bool has_exact_placement_envelope() const {
    return placement_envelope_source == PlacementEnvelopeSource::explicit_courtyard ||
           placement_envelope_source == PlacementEnvelopeSource::external_constraint;
  }

  [[nodiscard]] bool has_tentative_placement_envelope() const {
    return placement_envelope_source == PlacementEnvelopeSource::tentative_from_geometry ||
           placement_envelope_source == PlacementEnvelopeSource::approximated_from_pads;
  }

  bool operator==(const Footprint&) const = default;
};

struct TrackSegment {
  ObjectId id;
  NetId net;
  LayerId layer;
  Point start;
  Point end;
  Nanometres width;
  bool locked{false};

  bool operator==(const TrackSegment&) const = default;
};

// A non-degenerate circular centerline arc passing through start, middle, and end in that order.
// The three integer-nanometre points are the authoritative representation used by KiCad.
struct TrackArc {
  ObjectId id;
  NetId net;
  LayerId layer;
  Point start;
  Point middle;
  Point end;
  Nanometres width;
  bool locked{false};

  bool operator==(const TrackArc&) const = default;
};

struct Via {
  ObjectId id;
  NetId net;
  Point position;
  Nanometres diameter;
  Nanometres drill;
  std::vector<LayerId> layers;
  bool locked{false};

  bool operator==(const Via&) const = default;
};

struct CopperZoneFill {
  LayerId layer;
  Polygon area;
  // KiCad fractures a polygon-with-holes into one closed walk by traversing zero-width
  // bridge edges in both directions. Import removes those bridges and represents the same
  // physical copper as one simple outer contour plus simple void contours.
  std::vector<Polygon> holes;

  bool operator==(const CopperZoneFill&) const = default;
};

// Filled areas are immutable physical copper snapshots. Zone outline/refill policy remains owned
// by KiCad until a transactional IPC refill boundary is available.
struct CopperZone {
  ObjectId id;
  NetId net;
  std::string name;
  std::vector<CopperZoneFill> fills;
  // The source contains a zone definition but no saved physical fill. Flux preserves that
  // immutable source fact without inventing copper. Detailed routing may proceed only through
  // the explicitly provisional refill workflow, followed by KiCad refill and strict re-import.
  bool requires_refill{false};
  bool locked{false};

  bool operator==(const CopperZone&) const = default;
};

struct BoardDesignRules {
  Nanometres default_clearance{Nanometres::from_mm(0.2)};
  Nanometres default_track_width{Nanometres::from_mm(0.25)};
  Nanometres default_via_diameter{Nanometres::from_mm(0.6)};
  Nanometres default_via_drill{Nanometres::from_mm(0.3)};
  Nanometres default_differential_pair_track_width{Nanometres::from_mm(0.25)};
  Nanometres default_differential_pair_gap{Nanometres::from_mm(0.2)};
  Nanometres default_differential_pair_via_gap{Nanometres::from_mm(0.2)};
  Nanometres copper_to_edge_clearance{Nanometres::from_mm(0.5)};

  bool operator==(const BoardDesignRules&) const = default;
};

struct Board {
  std::uint32_t schema_version{board_schema_version};
  std::string source_revision;
  BoardDesignRules rules;
  Polygon outline;
  // Disjoint simple contours strictly inside outline. Board material is outline minus cutouts.
  std::vector<Polygon> cutouts;
  std::vector<Layer> layers;
  std::vector<Net> nets;
  std::vector<NetClass> net_classes;
  std::vector<DifferentialPair> differential_pairs;
  std::vector<RoutingBus> routing_buses;
  std::vector<PlacementGroup> placement_groups;
  std::vector<PlacementRegion> placement_regions;
  std::vector<RelativePlacementConstraint> relative_placement_constraints;
  std::vector<Footprint> footprints;
  std::vector<TrackSegment> tracks;
  std::vector<TrackArc> track_arcs;
  std::vector<Via> vias;
  std::vector<CopperZone> zones;
  std::vector<Keepout> keepouts;
};

} // namespace flux
