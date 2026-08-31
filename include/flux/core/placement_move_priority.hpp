#pragma once

#include "flux/core/ids.hpp"

#include <string>
#include <string_view>
#include <vector>

namespace flux {

enum class PlacementMovePriorityClass {
  resistor,
  capacitor,
  transistor,
  other,
  misc_ic,
  mcu,
  soc,
  connector
};

[[nodiscard]] constexpr std::string_view
placement_move_priority_class_name(PlacementMovePriorityClass value) {
  switch (value) {
  case PlacementMovePriorityClass::resistor:
    return "resistor";
  case PlacementMovePriorityClass::capacitor:
    return "capacitor";
  case PlacementMovePriorityClass::transistor:
    return "transistor";
  case PlacementMovePriorityClass::other:
    return "other";
  case PlacementMovePriorityClass::misc_ic:
    return "misc_ic";
  case PlacementMovePriorityClass::mcu:
    return "mcu";
  case PlacementMovePriorityClass::soc:
    return "soc";
  case PlacementMovePriorityClass::connector:
    return "connector";
  }
  return "unknown";
}

[[nodiscard]] constexpr std::vector<PlacementMovePriorityClass>
default_placement_move_priority_order() {
  return {PlacementMovePriorityClass::resistor,   PlacementMovePriorityClass::capacitor,
          PlacementMovePriorityClass::transistor, PlacementMovePriorityClass::other,
          PlacementMovePriorityClass::misc_ic,    PlacementMovePriorityClass::mcu,
          PlacementMovePriorityClass::soc,        PlacementMovePriorityClass::connector};
}

struct FootprintMovePriority {
  ObjectId footprint;
  PlacementMovePriorityClass priority_class{PlacementMovePriorityClass::other};

  bool operator==(const FootprintMovePriority&) const = default;
};

struct PlacementMovePriorityPolicy {
  std::string source_revision;
  std::vector<PlacementMovePriorityClass> order;
  std::vector<FootprintMovePriority> footprints;

  bool operator==(const PlacementMovePriorityPolicy&) const = default;
};

} // namespace flux
