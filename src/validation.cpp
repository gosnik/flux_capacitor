#include "flux/core/validation.hpp"

#include "flux/core/geometry.hpp"
#include "flux/core/keepout_geometry.hpp"
#include "flux/core/routing_rules.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <set>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

namespace flux {
namespace {

void add_issue(ValidationReport& report, IssueSeverity severity, std::string code,
               std::string message, std::vector<std::string> object_ids = {}) {
  report.issues.push_back(
      ValidationIssue{severity, std::move(code), std::move(message), std::move(object_ids)});
}

bool insert_unique(std::unordered_set<std::string>& values, const std::string& value) {
  return !value.empty() && values.insert(value).second;
}

bool has_value(const std::unordered_set<std::string>& values, const std::string& value) {
  return values.contains(value);
}

[[nodiscard]] std::uint64_t absolute_magnitude(std::int64_t value) {
  return value < 0 ? static_cast<std::uint64_t>(-(value + 1)) + 1U
                   : static_cast<std::uint64_t>(value);
}

[[nodiscard]] std::uint64_t circular_rotation_distance(Microdegrees first, Microdegrees second) {
  constexpr std::uint64_t full_turn = 360'000'000U;
  const auto difference = absolute_magnitude(first.value() - second.value());
  return std::min(difference, full_turn - difference);
}

struct PlacedCourtyard {
  ObjectId id;
  BoardSide side{BoardSide::front};
  Polygon area;
};

} // namespace

std::size_t ValidationReport::error_count() const {
  return static_cast<std::size_t>(
      std::count_if(issues.begin(), issues.end(),
                    [](const auto& issue) { return issue.severity == IssueSeverity::error; }));
}

std::size_t ValidationReport::warning_count() const {
  return static_cast<std::size_t>(
      std::count_if(issues.begin(), issues.end(),
                    [](const auto& issue) { return issue.severity == IssueSeverity::warning; }));
}

namespace {

ValidationReport validate_board_impl(const Board& board, bool reuse_static_zone_geometry) {
  ValidationReport report;
  if (board.schema_version != board_schema_version) {
    add_issue(report, IssueSeverity::error, "unsupported_schema",
              "board schema version is not supported");
  }
  if (!geometry::is_simple_polygon(board.outline)) {
    add_issue(report, IssueSeverity::error, "invalid_board_outline",
              "board outline must be a simple non-zero-area polygon");
    return report;
  }
  bool cutouts_valid = true;
  for (std::size_t index = 0U; index < board.cutouts.size(); ++index) {
    const auto& cutout = board.cutouts[index];
    const bool coordinates_in_range =
        std::all_of(cutout.vertices.begin(), cutout.vertices.end(),
                    [](Point point) { return geometry::coordinate_in_range(point); });
    const bool strictly_inside =
        geometry::is_simple_polygon(cutout) && geometry::polygon_inside(cutout, board.outline) &&
        geometry::polygon_boundary_distance_nm(cutout, board.outline) > 0.0L;
    const bool overlaps_prior = std::any_of(
        board.cutouts.begin(), board.cutouts.begin() + static_cast<std::ptrdiff_t>(index),
        [&cutout](const Polygon& prior) { return geometry::polygons_overlap(cutout, prior); });
    if (!coordinates_in_range || !strictly_inside || overlaps_prior) {
      add_issue(report, IssueSeverity::error, "invalid_board_cutout",
                "board cutouts must be in-range simple, disjoint contours strictly inside the "
                "outer outline",
                {std::to_string(index)});
      cutouts_valid = false;
    }
  }
  if (!cutouts_valid) {
    return report;
  }
  if (board.rules.default_clearance.value() < 0 || board.rules.default_track_width.value() <= 0 ||
      board.rules.default_via_diameter.value() <= 0 || board.rules.default_via_drill.value() < 0 ||
      board.rules.default_via_drill > board.rules.default_via_diameter ||
      board.rules.default_differential_pair_track_width.value() <= 0 ||
      board.rules.default_differential_pair_gap.value() < 0 ||
      board.rules.default_differential_pair_via_gap.value() < 0 ||
      board.rules.minimum_clearance.value() < 0 ||
      board.rules.minimum_track_width.value() < 0 ||
      board.rules.minimum_via_diameter.value() < 0 ||
      board.rules.minimum_via_annular_width.value() < 0 ||
      board.rules.minimum_through_drill.value() < 0 ||
      board.rules.copper_to_edge_clearance.value() < 0 ||
      board.rules.hole_clearance.value() < 0 ||
      board.rules.hole_to_hole_clearance.value() < 0 ||
      board.rules.minimum_via_annular_width.value() >
          (std::numeric_limits<std::int64_t>::max() -
           board.rules.minimum_through_drill.value()) /
              2) {
    add_issue(report, IssueSeverity::error, "invalid_board_design_rules",
              "board design-rule dimensions are inconsistent");
  }

  std::unordered_set<std::string> layer_ids;
  std::unordered_set<std::string> copper_layer_ids;
  for (const auto& layer : board.layers) {
    if (!insert_unique(layer_ids, layer.id.value())) {
      add_issue(report, IssueSeverity::error, "duplicate_or_empty_layer_id",
                "layer IDs must be non-empty and unique", {layer.id.value()});
    }
    if (layer.kind == LayerKind::copper) {
      copper_layer_ids.insert(layer.id.value());
    }
  }

  std::unordered_set<std::string> net_ids;
  std::unordered_set<std::string> net_names;
  std::unordered_set<std::string> net_class_ids;
  std::unordered_set<std::uint32_t> net_class_priorities;
  for (const auto& net_class : board.net_classes) {
    if (!insert_unique(net_class_ids, net_class.id.value())) {
      add_issue(report, IssueSeverity::error, "duplicate_or_empty_net_class_id",
                "net class IDs must be non-empty and unique", {net_class.id.value()});
    }
    if (net_class.name.empty()) {
      add_issue(report, IssueSeverity::error, "empty_net_class_name",
                "net class names must be non-empty", {net_class.id.value()});
    }
    if (!net_class_priorities.insert(net_class.priority).second) {
      add_issue(report, IssueSeverity::error, "duplicate_net_class_priority",
                "net class priorities must be globally unique", {net_class.id.value()});
    }
    const bool invalid_dimensions =
        (net_class.clearance && net_class.clearance->value() < 0) ||
        (net_class.track_width && net_class.track_width->value() <= 0) ||
        (net_class.via_diameter && net_class.via_diameter->value() <= 0) ||
        (net_class.via_drill && net_class.via_drill->value() < 0) ||
        (net_class.via_diameter && net_class.via_drill &&
         *net_class.via_drill > *net_class.via_diameter) ||
        (net_class.differential_pair_track_width &&
         net_class.differential_pair_track_width->value() <= 0) ||
        (net_class.differential_pair_gap && net_class.differential_pair_gap->value() < 0) ||
        (net_class.differential_pair_via_gap && net_class.differential_pair_via_gap->value() < 0);
    if (invalid_dimensions) {
      add_issue(report, IssueSeverity::error, "invalid_net_class_dimensions",
                "net class dimensions are negative, non-positive, or physically inconsistent",
                {net_class.id.value()});
    }
  }
  for (const auto& net : board.nets) {
    if (!insert_unique(net_ids, net.id.value())) {
      add_issue(report, IssueSeverity::error, "duplicate_or_empty_net_id",
                "net IDs must be non-empty and unique", {net.id.value()});
    }
    if (!net.name.empty() && !net_names.insert(net.name).second) {
      add_issue(report, IssueSeverity::error, "duplicate_net_name",
                "named nets must be unique for source policy and differential-pair identity",
                {net.id.value(), net.name});
    }
    std::unordered_set<std::string> memberships;
    for (const auto& class_id : net.classes) {
      if (!memberships.insert(class_id.value()).second) {
        add_issue(report, IssueSeverity::error, "duplicate_net_class_membership",
                  "a net may reference each net class at most once",
                  {net.id.value(), class_id.value()});
      }
      if (!has_value(net_class_ids, class_id.value())) {
        add_issue(report, IssueSeverity::error, "unknown_net_class",
                  "net references an undeclared net class", {net.id.value(), class_id.value()});
      }
    }
    const auto resolved = resolve_net_routing_rules(board, net.id);
    if (resolved.clearance.value() < 0 || resolved.track_width.value() <= 0 ||
        resolved.via_diameter.value() <= 0 || resolved.via_drill.value() < 0 ||
        resolved.via_drill > resolved.via_diameter ||
        resolved.differential_pair_track_width.value() <= 0 ||
        resolved.differential_pair_gap.value() < 0 ||
        resolved.differential_pair_via_gap.value() < 0) {
      add_issue(report, IssueSeverity::error, "invalid_resolved_net_rules",
                "inherited net class dimensions resolve to an inconsistent routing rule set",
                {net.id.value()});
    }
  }

  std::unordered_set<std::string> differential_pair_ids;
  std::unordered_set<std::string> differential_pair_names;
  std::unordered_set<std::string> paired_nets;
  for (const auto& pair : board.differential_pairs) {
    if (!insert_unique(differential_pair_ids, pair.id.value())) {
      add_issue(report, IssueSeverity::error, "duplicate_or_empty_differential_pair_id",
                "differential-pair IDs must be non-empty and unique", {pair.id.value()});
    }
    if (!insert_unique(differential_pair_names, pair.name)) {
      add_issue(report, IssueSeverity::error, "duplicate_or_empty_differential_pair_name",
                "differential-pair names must be non-empty and unique", {pair.id.value()});
    }
    if (pair.positive_net == pair.negative_net || !has_value(net_ids, pair.positive_net.value()) ||
        !has_value(net_ids, pair.negative_net.value())) {
      add_issue(report, IssueSeverity::error, "invalid_differential_pair_nets",
                "a differential pair must reference two distinct declared nets",
                {pair.id.value(), pair.positive_net.value(), pair.negative_net.value()});
    }
    const bool positive_unique = paired_nets.insert(pair.positive_net.value()).second;
    const bool negative_unique = paired_nets.insert(pair.negative_net.value()).second;
    if (!positive_unique || !negative_unique) {
      add_issue(report, IssueSeverity::error, "duplicate_differential_pair_membership",
                "a net may belong to at most one differential pair", {pair.id.value()});
    }
    if ((pair.minimum_gap && pair.minimum_gap->value() < 0) ||
        (pair.maximum_skew && pair.maximum_skew->value() < 0)) {
      add_issue(report, IssueSeverity::error, "invalid_differential_pair_constraints",
                "differential-pair gap and skew constraints must be non-negative",
                {pair.id.value()});
    }
  }

  std::unordered_set<std::string> routing_bus_ids;
  std::unordered_set<std::string> routing_bus_names;
  std::unordered_set<std::string> bus_nets;
  for (const auto& bus : board.routing_buses) {
    if (!insert_unique(routing_bus_ids, bus.id.value())) {
      add_issue(report, IssueSeverity::error, "duplicate_or_empty_routing_bus_id",
                "routing-bus IDs must be non-empty and unique", {bus.id.value()});
    }
    if (!insert_unique(routing_bus_names, bus.name)) {
      add_issue(report, IssueSeverity::error, "duplicate_or_empty_routing_bus_name",
                "routing-bus names must be non-empty and unique", {bus.id.value()});
    }
    if (bus.ordered_nets.size() < 2U) {
      add_issue(report, IssueSeverity::error, "routing_bus_too_small",
                "a routing bus must contain at least two ordered nets", {bus.id.value()});
    }
    if (bus.minimum_gap && bus.minimum_gap->value() < 0) {
      add_issue(report, IssueSeverity::error, "invalid_routing_bus_gap",
                "a routing-bus minimum gap must be non-negative", {bus.id.value()});
    }
    std::unordered_set<std::string> members;
    for (const auto& net : bus.ordered_nets) {
      if (!members.insert(net.value()).second) {
        add_issue(report, IssueSeverity::error, "duplicate_routing_bus_member",
                  "a routing bus may reference each member net at most once",
                  {bus.id.value(), net.value()});
      }
      if (!has_value(net_ids, net.value())) {
        add_issue(report, IssueSeverity::error, "unknown_routing_bus_net",
                  "a routing bus must reference declared nets", {bus.id.value(), net.value()});
      }
      if (!bus_nets.insert(net.value()).second) {
        add_issue(report, IssueSeverity::error, "duplicate_routing_bus_membership",
                  "a net may belong to at most one routing bus", {bus.id.value(), net.value()});
      }
      if (paired_nets.contains(net.value())) {
        add_issue(report, IssueSeverity::error, "overlapping_coupled_routing_membership",
                  "this routing baseline cannot place one net in both a differential pair and a "
                  "routing bus",
                  {bus.id.value(), net.value()});
      }
    }
  }

  std::unordered_set<std::string> object_ids;
  std::vector<PlacedCourtyard> courtyards;
  courtyards.reserve(board.footprints.size());

  const auto validate_keepout = [&](const Keepout& keepout, const Polygon& world_area) {
    bool valid = true;
    if (!insert_unique(object_ids, keepout.id.value())) {
      add_issue(report, IssueSeverity::error, "duplicate_or_empty_object_id",
                "object IDs must be non-empty and globally unique", {keepout.id.value()});
      valid = false;
    }
    if (!geometry::is_simple_polygon(keepout.area)) {
      add_issue(report, IssueSeverity::error, "invalid_keepout_area",
                "keepout area must be a simple non-zero-area polygon", {keepout.id.value()});
      valid = false;
    }
    if (keepout.layers.empty()) {
      add_issue(report, IssueSeverity::error, "keepout_without_layers",
                "keepout must apply to at least one copper layer", {keepout.id.value()});
      valid = false;
    }
    for (const auto& layer : keepout.layers) {
      if (!has_value(copper_layer_ids, layer.value())) {
        add_issue(report, IssueSeverity::error, "unknown_keepout_layer",
                  "keepout references an undeclared or non-copper layer",
                  {keepout.id.value(), layer.value()});
        valid = false;
      }
    }
    if (!keepout.prohibit.any()) {
      add_issue(report, IssueSeverity::error, "empty_keepout_rules",
                "keepout must prohibit at least one object category", {keepout.id.value()});
      valid = false;
    }
    const bool world_in_range =
        std::all_of(world_area.vertices.begin(), world_area.vertices.end(),
                    [](Point point) { return geometry::coordinate_in_range(point); });
    if (!world_in_range || !geometry::polygon_inside_board_area(world_area, board)) {
      add_issue(report, IssueSeverity::error, "keepout_outside_board",
                "keepout area must be fully inside the board outline", {keepout.id.value()});
      valid = false;
    }
    return valid;
  };

  std::vector<WorldKeepout> valid_keepouts;
  valid_keepouts.reserve(board.keepouts.size());
  for (const auto& keepout : board.keepouts) {
    if (validate_keepout(keepout, keepout.area)) {
      valid_keepouts.push_back(
          WorldKeepout{keepout.id, keepout.area, keepout.layers, keepout.prohibit, std::nullopt});
    }
  }

  for (const auto& footprint : board.footprints) {
    if (!insert_unique(object_ids, footprint.id.value())) {
      add_issue(report, IssueSeverity::error, "duplicate_or_empty_object_id",
                "object IDs must be non-empty and globally unique", {footprint.id.value()});
    }
    if (!footprint.orientation_constraints.allow_front &&
        !footprint.orientation_constraints.allow_back) {
      add_issue(report, IssueSeverity::error, "footprint_without_allowed_side",
                "footprint orientation constraints must allow at least one board side",
                {footprint.id.value()});
    }
    const bool current_side_allowed = footprint.side == BoardSide::front
                                          ? footprint.orientation_constraints.allow_front
                                          : footprint.orientation_constraints.allow_back;
    if (!current_side_allowed) {
      add_issue(report, IssueSeverity::error, "forbidden_footprint_side",
                "footprint is placed on a side forbidden by its orientation constraints",
                {footprint.id.value()});
    }
    std::unordered_set<std::int64_t> allowed_rotations;
    for (const auto rotation : footprint.orientation_constraints.allowed_rotations) {
      if (!allowed_rotations.insert(rotation.value()).second) {
        add_issue(report, IssueSeverity::error, "duplicate_allowed_rotation",
                  "allowed footprint rotations must be unique after normalization",
                  {footprint.id.value()});
      }
    }
    if (!allowed_rotations.empty() && !allowed_rotations.contains(footprint.rotation.value())) {
      add_issue(report, IssueSeverity::error, "forbidden_footprint_rotation",
                "footprint rotation is not one of its allowed absolute rotations",
                {footprint.id.value()});
    }
    if (!geometry::coordinate_in_range(footprint.position)) {
      add_issue(report, IssueSeverity::error, "placement_coordinate_out_of_range",
                "footprint position exceeds the supported coordinate range",
                {footprint.id.value()});
      continue;
    }
    for (const auto& circle : footprint.courtyard_circles) {
      if (!geometry::coordinate_in_range(circle.center) ||
          !geometry::coordinate_in_range(circle.edge) || circle.center == circle.edge) {
        add_issue(report, IssueSeverity::error, "invalid_courtyard_circle",
                  "exact courtyard circles require distinct in-range center and edge points",
                  {footprint.id.value()});
      }
    }
    if (!footprint.courtyard_circles.empty() &&
        footprint.placement_envelope_source != PlacementEnvelopeSource::explicit_courtyard) {
      add_issue(report, IssueSeverity::error, "unexpected_courtyard_circle_provenance",
                "exact courtyard circles require explicit source-envelope provenance",
                {footprint.id.value()});
    }
    if (footprint.placement_envelope_source == PlacementEnvelopeSource::absent) {
      if (!footprint.courtyard.vertices.empty() || !footprint.courtyard_circles.empty()) {
        add_issue(report, IssueSeverity::error, "unexpected_absent_courtyard_geometry",
                  "an absent placement envelope must not contain courtyard geometry",
                  {footprint.id.value()});
      } else {
        add_issue(report, IssueSeverity::warning, "missing_placement_envelope",
                  "footprint has no exact placement envelope and cannot participate in "
                  "placement optimization",
                  {footprint.id.value()});
      }
    } else if (!geometry::is_simple_polygon(footprint.courtyard)) {
      add_issue(report, IssueSeverity::error, "invalid_courtyard",
                "represented footprint courtyards must be simple non-zero-area polygons",
                {footprint.id.value()});
    } else {
      const auto courtyard = geometry::transform_polygon(footprint.courtyard, footprint.position,
                                                         footprint.rotation, footprint.side);
      const bool courtyard_in_range =
          std::all_of(courtyard.vertices.begin(), courtyard.vertices.end(),
                      [](Point point) { return geometry::coordinate_in_range(point); });
      if (!courtyard_in_range || !geometry::polygon_inside_board_area(courtyard, board)) {
        add_issue(report, IssueSeverity::error, "placement_outside_board",
                  "footprint courtyard is not fully inside the board outline",
                  {footprint.id.value()});
      }
      if (courtyard_in_range) {
        courtyards.push_back(PlacedCourtyard{footprint.id, footprint.side, courtyard});
      }
    }

    for (const auto& keepout : footprint.keepouts) {
      const auto world_area = geometry::transform_polygon(keepout.area, footprint.position,
                                                          footprint.rotation, footprint.side);
      if (validate_keepout(keepout, world_area)) {
        valid_keepouts.push_back(WorldKeepout{keepout.id, std::move(world_area), keepout.layers,
                                              keepout.prohibit, footprint.id});
      }
    }

    for (const auto& pad : footprint.pads) {
      if (!insert_unique(object_ids, pad.id.value())) {
        add_issue(report, IssueSeverity::error, "duplicate_or_empty_object_id",
                  "object IDs must be non-empty and globally unique", {pad.id.value()});
      }
      if (!has_value(net_ids, pad.net.value())) {
        add_issue(report, IssueSeverity::error, "unknown_pad_net",
                  "pad references an undeclared net", {pad.id.value(), pad.net.value()});
      }
      if (pad.layers.empty()) {
        add_issue(report, IssueSeverity::error, "pad_without_layers",
                  "pad must belong to at least one layer", {pad.id.value()});
      }
      const bool partial_drill = (pad.drill_width.value() == 0) != (pad.drill_height.value() == 0);
      const bool drill_outside_pad_bounds =
          pad.drill_width.value() > 0 && pad.drill_height.value() > 0 &&
          (static_cast<long double>(absolute_magnitude(pad.drill_offset.x.value())) * 2.0L +
                   static_cast<long double>(pad.drill_width.value()) >
               static_cast<long double>(pad.width.value()) ||
           static_cast<long double>(absolute_magnitude(pad.drill_offset.y.value())) * 2.0L +
                   static_cast<long double>(pad.drill_height.value()) >
               static_cast<long double>(pad.height.value()));
      if (pad.width.value() <= 0 || pad.height.value() <= 0 ||
          pad.drill_width.value() < 0 || pad.drill_height.value() < 0 || partial_drill ||
          drill_outside_pad_bounds) {
        add_issue(report, IssueSeverity::error, "invalid_pad_dimensions",
                  "pad dimensions must be positive and contain the complete drill bounds",
                  {pad.id.value()});
      }
      if ((pad.shape == PadShape::circle && pad.width != pad.height) ||
          pad.corner_radius.value() < 0 ||
          (pad.shape == PadShape::round_rectangle &&
           pad.corner_radius > Nanometres{std::min(pad.width.value(), pad.height.value()) / 2}) ||
          (pad.shape != PadShape::round_rectangle && pad.corner_radius.value() != 0)) {
        add_issue(report, IssueSeverity::error, "invalid_pad_shape_geometry",
                  "pad shape dimensions or corner radius are inconsistent", {pad.id.value()});
      }
      for (const auto& layer : pad.layers) {
        if (!has_value(layer_ids, layer.value())) {
          add_issue(report, IssueSeverity::error, "unknown_pad_layer",
                    "pad references an undeclared layer", {pad.id.value(), layer.value()});
        }
      }
      const auto position = geometry::transform_point(pad.local_position, footprint.position,
                                                      footprint.rotation, footprint.side);
      if (!geometry::coordinate_in_range(position) ||
          geometry::locate_point_in_board_area(board, position) ==
              geometry::PointLocation::outside) {
        add_issue(report, IssueSeverity::error, "pad_outside_board",
                  "transformed pad center is outside the board", {pad.id.value()});
      }
    }
  }

  std::unordered_set<std::string> placement_group_ids;
  std::unordered_set<std::string> placement_group_names;
  std::unordered_set<std::string> grouped_footprints;
  std::unordered_set<std::string> footprint_ids;
  for (const auto& footprint : board.footprints) {
    footprint_ids.insert(footprint.id.value());
  }
  for (const auto& group : board.placement_groups) {
    if (!insert_unique(placement_group_ids, group.id.value())) {
      add_issue(report, IssueSeverity::error, "duplicate_or_empty_placement_group_id",
                "placement-group IDs must be non-empty and unique", {group.id.value()});
    }
    if (!insert_unique(placement_group_names, group.name)) {
      add_issue(report, IssueSeverity::error, "duplicate_or_empty_placement_group_name",
                "placement-group names must be non-empty and unique", {group.id.value()});
    }
    if (group.footprints.size() < 2U) {
      add_issue(report, IssueSeverity::error, "placement_group_too_small",
                "a placement group must contain at least two footprints", {group.id.value()});
    }
    std::unordered_set<std::string> members;
    for (const auto& footprint : group.footprints) {
      if (!members.insert(footprint.value()).second) {
        add_issue(report, IssueSeverity::error, "duplicate_placement_group_member",
                  "a placement group may reference each footprint at most once",
                  {group.id.value(), footprint.value()});
      }
      if (!footprint_ids.contains(footprint.value())) {
        add_issue(report, IssueSeverity::error, "unknown_placement_group_footprint",
                  "a placement group must reference declared footprints",
                  {group.id.value(), footprint.value()});
      }
      if (!grouped_footprints.insert(footprint.value()).second) {
        add_issue(report, IssueSeverity::error, "overlapping_placement_group_membership",
                  "a footprint may belong to at most one placement group",
                  {group.id.value(), footprint.value()});
      }
    }
  }

  std::unordered_set<std::string> placement_region_ids;
  std::unordered_set<std::string> placement_region_names;
  std::unordered_set<std::string> region_footprints;
  for (const auto& region : board.placement_regions) {
    if (!insert_unique(placement_region_ids, region.id.value())) {
      add_issue(report, IssueSeverity::error, "duplicate_or_empty_placement_region_id",
                "placement-region IDs must be non-empty and unique", {region.id.value()});
    }
    if (!insert_unique(placement_region_names, region.name)) {
      add_issue(report, IssueSeverity::error, "duplicate_or_empty_placement_region_name",
                "placement-region names must be non-empty and unique", {region.id.value()});
    }
    const bool valid_area = geometry::is_simple_polygon(region.area);
    if (!valid_area) {
      add_issue(report, IssueSeverity::error, "invalid_placement_region_area",
                "a placement region must be a simple non-zero-area polygon", {region.id.value()});
    } else {
      const bool area_in_range =
          std::all_of(region.area.vertices.begin(), region.area.vertices.end(),
                      [](Point point) { return geometry::coordinate_in_range(point); });
      if (!area_in_range || !geometry::polygon_inside_board_area(region.area, board)) {
        add_issue(report, IssueSeverity::error, "placement_region_outside_board",
                  "a placement region must be fully inside the board outline", {region.id.value()});
      }
    }
    if (region.footprints.empty()) {
      add_issue(report, IssueSeverity::error, "empty_placement_region",
                "a placement region must contain at least one footprint", {region.id.value()});
    }
    std::unordered_set<std::string> members;
    for (const auto& member : region.footprints) {
      if (!members.insert(member.value()).second) {
        add_issue(report, IssueSeverity::error, "duplicate_placement_region_member",
                  "a placement region may reference each footprint at most once",
                  {region.id.value(), member.value()});
      }
      if (!footprint_ids.contains(member.value())) {
        add_issue(report, IssueSeverity::error, "unknown_placement_region_footprint",
                  "a placement region must reference declared footprints",
                  {region.id.value(), member.value()});
        continue;
      }
      if (!region_footprints.insert(member.value()).second) {
        add_issue(report, IssueSeverity::error, "overlapping_placement_region_membership",
                  "a footprint may belong to at most one placement region",
                  {region.id.value(), member.value()});
      }
      const auto found =
          std::find_if(board.footprints.begin(), board.footprints.end(),
                       [&member](const Footprint& footprint) { return footprint.id == member; });
      if (found == board.footprints.end()) {
        continue;
      }
      if (!found->has_exact_placement_envelope()) {
        add_issue(report, IssueSeverity::error, "placement_region_requires_exact_envelope",
                  "a region-constrained footprint requires an authoritative placement envelope",
                  {region.id.value(), member.value()});
        continue;
      }
      if (valid_area && geometry::is_simple_polygon(found->courtyard)) {
        const auto courtyard = geometry::transform_polygon(found->courtyard, found->position,
                                                           found->rotation, found->side);
        if (!geometry::polygon_inside(courtyard, region.area)) {
          add_issue(report, IssueSeverity::error, "placement_outside_region",
                    "the complete transformed footprint envelope must remain inside its region",
                    {region.id.value(), member.value()});
        }
      }
    }
  }

  std::unordered_set<std::string> relative_constraint_ids;
  std::unordered_set<std::string> relative_constraint_names;
  std::set<std::pair<std::string, std::string>> relative_pairs;
  for (const auto& constraint : board.relative_placement_constraints) {
    if (!insert_unique(relative_constraint_ids, constraint.id.value())) {
      add_issue(report, IssueSeverity::error, "duplicate_or_empty_relative_placement_id",
                "relative-placement IDs must be non-empty and unique", {constraint.id.value()});
    }
    if (!insert_unique(relative_constraint_names, constraint.name)) {
      add_issue(report, IssueSeverity::error, "duplicate_or_empty_relative_placement_name",
                "relative-placement names must be non-empty and unique", {constraint.id.value()});
    }
    if (constraint.maximum_x_deviation.value() < 0 || constraint.maximum_y_deviation.value() < 0) {
      add_issue(report, IssueSeverity::error, "negative_relative_placement_tolerance",
                "relative-placement tolerances must be non-negative", {constraint.id.value()});
    }
    const auto derived_nominal_distance =
        geometry::segment_length_nm(Point{}, constraint.nominal_offset);
    if (constraint.nominal_distance.value() < 0 ||
        static_cast<std::uint64_t>(constraint.nominal_distance.value()) !=
            derived_nominal_distance) {
      add_issue(report, IssueSeverity::error, "invalid_relative_nominal_distance",
                "relative-placement nominal distance must match the nominal anchor vector",
                {constraint.id.value()});
    }
    if (constraint.maximum_distance_deviation &&
        constraint.maximum_distance_deviation->value() < 0) {
      add_issue(report, IssueSeverity::error, "negative_relative_distance_tolerance",
                "relative distance tolerance must be non-negative", {constraint.id.value()});
    }
    constexpr std::uint64_t half_turn_microdegrees = 180'000'000U;
    if (constraint.maximum_rotation_deviation_microdegrees &&
        *constraint.maximum_rotation_deviation_microdegrees > half_turn_microdegrees) {
      add_issue(report, IssueSeverity::error, "invalid_relative_rotation_tolerance",
                "relative rotation tolerance must not exceed 180 degrees", {constraint.id.value()});
    }
    if (constraint.first_footprint == constraint.second_footprint) {
      add_issue(report, IssueSeverity::error, "relative_placement_self_pair",
                "a relative-placement constraint requires two distinct footprints",
                {constraint.id.value(), constraint.first_footprint.value()});
      continue;
    }
    auto pair =
        std::minmax(constraint.first_footprint.value(), constraint.second_footprint.value());
    if (!relative_pairs.emplace(pair.first, pair.second).second) {
      add_issue(report, IssueSeverity::error, "duplicate_relative_placement_pair",
                "an unordered footprint pair may have at most one relative constraint",
                {constraint.id.value(), pair.first, pair.second});
    }
    const auto first = std::find_if(
        board.footprints.begin(), board.footprints.end(),
        [&](const Footprint& footprint) { return footprint.id == constraint.first_footprint; });
    const auto second = std::find_if(
        board.footprints.begin(), board.footprints.end(),
        [&](const Footprint& footprint) { return footprint.id == constraint.second_footprint; });
    if (first == board.footprints.end() || second == board.footprints.end()) {
      add_issue(report, IssueSeverity::error, "unknown_relative_placement_footprint",
                "relative-placement endpoints must reference declared footprints",
                {constraint.id.value(), constraint.first_footprint.value(),
                 constraint.second_footprint.value()});
      continue;
    }
    const auto actual = second->position - first->position;
    const auto x_deviation =
        absolute_magnitude(actual.x.value() - constraint.nominal_offset.x.value());
    const auto y_deviation =
        absolute_magnitude(actual.y.value() - constraint.nominal_offset.y.value());
    if (constraint.maximum_x_deviation.value() >= 0 &&
        constraint.maximum_y_deviation.value() >= 0 &&
        (x_deviation > static_cast<std::uint64_t>(constraint.maximum_x_deviation.value()) ||
         y_deviation > static_cast<std::uint64_t>(constraint.maximum_y_deviation.value()))) {
      add_issue(report, IssueSeverity::error, "relative_placement_tolerance_exceeded",
                "the current anchor vector exceeds its inclusive source-relative tolerance",
                {constraint.id.value(), constraint.first_footprint.value(),
                 constraint.second_footprint.value()});
    }
    if (constraint.maximum_distance_deviation &&
        constraint.maximum_distance_deviation->value() >= 0 &&
        constraint.nominal_distance.value() >= 0) {
      const auto actual_distance = geometry::segment_length_nm(first->position, second->position);
      const auto distance_deviation = absolute_magnitude(
          static_cast<std::int64_t>(actual_distance) - constraint.nominal_distance.value());
      if (distance_deviation >
          static_cast<std::uint64_t>(constraint.maximum_distance_deviation->value())) {
        add_issue(report, IssueSeverity::error, "relative_distance_tolerance_exceeded",
                  "the current anchor distance exceeds its inclusive source-relative tolerance",
                  {constraint.id.value(), constraint.first_footprint.value(),
                   constraint.second_footprint.value()});
      }
    }
    if (constraint.maximum_rotation_deviation_microdegrees) {
      const auto actual_relative_rotation =
          Microdegrees{second->rotation.value() - first->rotation.value()};
      if (circular_rotation_distance(actual_relative_rotation,
                                     constraint.nominal_relative_rotation) >
          *constraint.maximum_rotation_deviation_microdegrees) {
        add_issue(report, IssueSeverity::error, "relative_rotation_tolerance_exceeded",
                  "the current relative rotation exceeds its inclusive source-relative tolerance",
                  {constraint.id.value(), constraint.first_footprint.value(),
                   constraint.second_footprint.value()});
      }
    }
  }

  for (std::size_t i = 0; i < courtyards.size(); ++i) {
    for (std::size_t j = i + 1U; j < courtyards.size(); ++j) {
      if (geometry::polygons_overlap(courtyards[i].area, courtyards[j].area)) {
        add_issue(report, IssueSeverity::error, "placement_overlap",
                  "footprint courtyards overlap or touch",
                  {courtyards[i].id.value(), courtyards[j].id.value()});
      }
    }
  }
  for (const auto& courtyard : courtyards) {
    const LayerId side_layer{courtyard.side == BoardSide::front ? "F.Cu" : "B.Cu"};
    for (const auto& keepout : valid_keepouts) {
      if (!keepout.prohibit.footprints || keepout.owner_footprint == courtyard.id ||
          !keepout_contains_layer(keepout, side_layer)) {
        continue;
      }
      if (geometry::polygons_overlap(courtyard.area, keepout.area)) {
        add_issue(report, IssueSeverity::error, "placement_keepout",
                  "footprint courtyard overlaps a placement keepout",
                  {courtyard.id.value(), keepout.id.value()});
      }
    }
  }

  for (const auto& track : board.tracks) {
    if (!insert_unique(object_ids, track.id.value())) {
      add_issue(report, IssueSeverity::error, "duplicate_or_empty_object_id",
                "object IDs must be non-empty and globally unique", {track.id.value()});
    }
    if (!has_value(net_ids, track.net.value())) {
      add_issue(report, IssueSeverity::error, "unknown_track_net",
                "track references an undeclared net", {track.id.value(), track.net.value()});
    }
    if (!has_value(layer_ids, track.layer.value())) {
      add_issue(report, IssueSeverity::error, "unknown_track_layer",
                "track references an undeclared layer", {track.id.value(), track.layer.value()});
    }
    if (track.width.value() <= 0) {
      add_issue(report, IssueSeverity::error, "track_nonpositive_width",
                "track width must be positive", {track.id.value()});
    }
    if (track.start == track.end) {
      add_issue(report, IssueSeverity::error, "track_zero_length", "track endpoints must differ",
                {track.id.value()});
    }
    if (!geometry::coordinate_in_range(track.start) || !geometry::coordinate_in_range(track.end) ||
        geometry::locate_point_in_board_area(board, track.start) ==
            geometry::PointLocation::outside ||
        geometry::locate_point_in_board_area(board, track.end) ==
            geometry::PointLocation::outside) {
      add_issue(report, IssueSeverity::error, "track_outside_board",
                "track centerline endpoints must be inside the board", {track.id.value()});
    }
  }

  for (const auto& arc : board.track_arcs) {
    if (!insert_unique(object_ids, arc.id.value())) {
      add_issue(report, IssueSeverity::error, "duplicate_or_empty_object_id",
                "object IDs must be non-empty and globally unique", {arc.id.value()});
    }
    if (!has_value(net_ids, arc.net.value())) {
      add_issue(report, IssueSeverity::error, "unknown_track_arc_net",
                "track arc references an undeclared net", {arc.id.value(), arc.net.value()});
    }
    if (!has_value(layer_ids, arc.layer.value())) {
      add_issue(report, IssueSeverity::error, "unknown_track_arc_layer",
                "track arc references an undeclared layer", {arc.id.value(), arc.layer.value()});
    }
    if (arc.width.value() <= 0) {
      add_issue(report, IssueSeverity::error, "track_arc_nonpositive_width",
                "track arc width must be positive", {arc.id.value()});
    }
    if (!geometry::circular_arc_parameters(arc.start, arc.middle, arc.end)) {
      add_issue(report, IssueSeverity::error, "invalid_track_arc_geometry",
                "track arc requires three distinct non-collinear in-range points",
                {arc.id.value()});
    } else if (!geometry::circular_arc_inside_board_area(arc.start, arc.middle, arc.end, board)) {
      add_issue(report, IssueSeverity::error, "track_arc_outside_board",
                "track arc centerline must remain inside the board", {arc.id.value()});
    }
  }

  for (const auto& via : board.vias) {
    if (!insert_unique(object_ids, via.id.value())) {
      add_issue(report, IssueSeverity::error, "duplicate_or_empty_object_id",
                "object IDs must be non-empty and globally unique", {via.id.value()});
    }
    if (!has_value(net_ids, via.net.value())) {
      add_issue(report, IssueSeverity::error, "unknown_via_net", "via references an undeclared net",
                {via.id.value(), via.net.value()});
    }
    if (via.layers.empty()) {
      add_issue(report, IssueSeverity::error, "via_without_layers",
                "via must connect at least one copper layer", {via.id.value()});
    }
    for (const auto& layer : via.layers) {
      if (!has_value(layer_ids, layer.value())) {
        add_issue(report, IssueSeverity::error, "unknown_via_layer",
                  "via references an undeclared layer", {via.id.value(), layer.value()});
      }
    }
    if (via.diameter.value() <= 0 || via.drill.value() < 0 || via.drill > via.diameter) {
      add_issue(report, IssueSeverity::error, "invalid_via_dimensions",
                "via diameter must be positive and not smaller than its drill", {via.id.value()});
    }
    if (!geometry::coordinate_in_range(via.position) ||
        geometry::locate_point_in_board_area(board, via.position) ==
            geometry::PointLocation::outside) {
      add_issue(report, IssueSeverity::error, "via_outside_board",
                "via center must be inside the board", {via.id.value()});
    }
  }

  for (const auto& zone : board.zones) {
    if (!insert_unique(object_ids, zone.id.value())) {
      add_issue(report, IssueSeverity::error, "duplicate_or_empty_object_id",
                "object IDs must be non-empty and globally unique", {zone.id.value()});
    }
    if (!has_value(net_ids, zone.net.value())) {
      add_issue(report, IssueSeverity::error, "unknown_zone_net",
                "copper zone references an undeclared net", {zone.id.value(), zone.net.value()});
    }
    if (zone.fills.empty() && !zone.requires_refill) {
      add_issue(report, IssueSeverity::error, "zone_without_filled_copper",
                "represented copper zones must contain a saved fill or explicitly require refill",
                {zone.id.value()});
    } else if (!zone.fills.empty() && zone.requires_refill) {
      add_issue(report, IssueSeverity::error, "contradictory_zone_refill_state",
                "a zone cannot contain saved physical fill while also requiring refill",
                {zone.id.value()});
    } else if (zone.requires_refill) {
      add_issue(report, IssueSeverity::warning, "zone_refill_required",
                "zone has no saved physical copper and requires KiCad refill before final routing "
                "acceptance",
                {zone.id.value()});
    }
    for (const auto& fill : zone.fills) {
      if (!has_value(copper_layer_ids, fill.layer.value())) {
        add_issue(report, IssueSeverity::error, "unknown_zone_layer",
                  "zone fill references an undeclared or non-copper layer",
                  {zone.id.value(), fill.layer.value()});
      }
      if (reuse_static_zone_geometry) {
        continue;
      }
      const bool coordinates_in_range =
          std::all_of(fill.area.vertices.begin(), fill.area.vertices.end(),
                      [](Point point) { return geometry::coordinate_in_range(point); });
      if (!coordinates_in_range || !geometry::is_simple_polygon(fill.area) ||
          !geometry::filled_region_inside_board_area(fill, board)) {
        add_issue(report, IssueSeverity::error, "invalid_zone_fill",
                  "zone fill outer contours must be simple polygons fully inside the board "
                  "outline",
                  {zone.id.value(), fill.layer.value()});
      }
      for (std::size_t hole_index = 0; hole_index < fill.holes.size(); ++hole_index) {
        const auto& hole = fill.holes[hole_index];
        const bool hole_coordinates_in_range =
            std::all_of(hole.vertices.begin(), hole.vertices.end(),
                        [](Point point) { return geometry::coordinate_in_range(point); });
        const bool overlaps_prior = std::any_of(
            fill.holes.begin(), fill.holes.begin() + static_cast<std::ptrdiff_t>(hole_index),
            [&hole](const Polygon& prior) { return geometry::polygons_overlap(hole, prior); });
        if (!hole_coordinates_in_range || !geometry::is_simple_polygon(hole) ||
            !geometry::polygon_inside(hole, fill.area) ||
            geometry::polygon_boundary_distance_nm(hole, fill.area) <= 0.0L || overlaps_prior) {
          add_issue(report, IssueSeverity::error, "invalid_zone_fill_hole",
                    "zone fill holes must be simple, disjoint, and strictly inside the outer "
                    "contour",
                    {zone.id.value(), fill.layer.value(), std::to_string(hole_index)});
        }
      }
    }
  }

  return report;
}

[[nodiscard]] bool has_geometry_certification_error(const ValidationReport& validation) {
  static const std::set<std::string> geometry_errors{"invalid_board_outline",
                                                     "invalid_board_cutout", "invalid_zone_fill",
                                                     "invalid_zone_fill_hole"};
  return std::any_of(validation.issues.begin(), validation.issues.end(), [](const auto& issue) {
    return issue.severity == IssueSeverity::error && geometry_errors.contains(issue.code);
  });
}

} // namespace

ValidationReport validate_board(const Board& board) { return validate_board_impl(board, false); }

BoardValidationBaseline make_board_validation_baseline(const Board& source) {
  BoardValidationBaseline result;
  result.source_validation_ = validate_board_impl(source, false);
  result.outline_ = source.outline;
  result.cutouts_ = source.cutouts;
  result.zones_ = source.zones;
  result.static_zone_geometry_certified_ =
      !has_geometry_certification_error(result.source_validation_);
  return result;
}

CandidateValidationResult validate_board_candidate(const Board& candidate,
                                                   const BoardValidationBaseline& baseline) {
  const bool reuse = baseline.static_zone_geometry_certified_ &&
                     candidate.outline == baseline.outline_ &&
                     candidate.cutouts == baseline.cutouts_ && candidate.zones == baseline.zones_;
  return CandidateValidationResult{validate_board_impl(candidate, reuse), reuse};
}

} // namespace flux
