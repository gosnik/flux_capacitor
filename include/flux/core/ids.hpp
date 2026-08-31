#pragma once

#include <compare>
#include <string>
#include <utility>

namespace flux {

template <typename Tag> class Id {
public:
  Id() = default;
  explicit Id(std::string value) : value_(std::move(value)) {}

  [[nodiscard]] const std::string& value() const { return value_; }
  [[nodiscard]] bool empty() const { return value_.empty(); }

  auto operator<=>(const Id&) const = default;

private:
  std::string value_;
};

struct ObjectIdTag;
struct NetIdTag;
struct NetClassIdTag;
struct DifferentialPairIdTag;
struct RoutingBusIdTag;
struct PlacementGroupIdTag;
struct PlacementRegionIdTag;
struct RelativePlacementConstraintIdTag;
struct LayerIdTag;

using ObjectId = Id<ObjectIdTag>;
using NetId = Id<NetIdTag>;
using NetClassId = Id<NetClassIdTag>;
using DifferentialPairId = Id<DifferentialPairIdTag>;
using RoutingBusId = Id<RoutingBusIdTag>;
using PlacementGroupId = Id<PlacementGroupIdTag>;
using PlacementRegionId = Id<PlacementRegionIdTag>;
using RelativePlacementConstraintId = Id<RelativePlacementConstraintIdTag>;
using LayerId = Id<LayerIdTag>;

} // namespace flux
