#include "flux/core/global_router.hpp"
#include "flux/core/live_copper_terrain.hpp"

#include <algorithm>
#include <iostream>
#include <limits>
#include <stdexcept>

namespace {
using namespace flux;

Point point(double x, double y) {
  return {Nanometres::from_mm(x), Nanometres::from_mm(y)};
}
void require(bool value, const char* message) {
  if (!value)
    throw std::runtime_error(message);
}
} // namespace

int main() {
  try {
    Board board;
    board.layers = {{LayerId{"F.Cu"}, "F.Cu", LayerKind::copper, 0U},
                    {LayerId{"In1.Cu"}, "In1.Cu", LayerKind::copper, 1U},
                    {LayerId{"B.Cu"}, "B.Cu", LayerKind::copper, 2U}};
    const auto a = NetId{"A"};
    const auto b = NetId{"B"};
    board.tracks.push_back({ObjectId{"line"}, a, LayerId{"F.Cu"}, point(1, 1), point(9, 1),
                            Nanometres::from_mm(.25), false});
    board.track_arcs.push_back({ObjectId{"arc"}, b, LayerId{"F.Cu"}, point(2, 2), point(5, 8),
                                point(8, 2), Nanometres::from_mm(.25), false});
    board.track_beziers.push_back({ObjectId{"bezier"}, b, LayerId{"F.Cu"}, point(1, 3), point(3, 9),
                                   point(7, -3), point(9, 3), Nanometres::from_mm(.25), false});
    board.vias.push_back({ObjectId{"via"}, b, point(5, 5), Nanometres::from_mm(.6),
                          Nanometres::from_mm(.3), {LayerId{"F.Cu"}, LayerId{"B.Cu"}}, false});

    const auto snapshot = make_live_copper_terrain_snapshot(board, false, 4096U);
    require(snapshot.features.size() == 6U, "line, arc, bezier and through-via layers must appear");
    const auto foreign_front = live_copper_terrain_deposits(snapshot, a, LayerId{"F.Cu"});
    require(foreign_front.size() == 3U, "same-net front copper must be excluded");
    const auto foreign_inner = live_copper_terrain_deposits(snapshot, a, LayerId{"In1.Cu"});
    require(foreign_inner.size() == 1U, "via span must pressure inner copper layer");
    require(foreign_front.front().pressure == 4096U, "live copper pressure must be material");
    auto high_clearance = board;
    NetClass wide;
    wide.id = NetClassId{"wide"};
    wide.name = "Wide clearance";
    wide.clearance = Nanometres::from_mm(.8);
    high_clearance.net_classes.push_back(wide);
    high_clearance.nets = {{a, "A", 0U, {}}, {b, "B", 0U, {wide.id}}};
    const auto wide_snapshot = make_live_copper_terrain_snapshot(high_clearance, false, 4096U);
    const auto own_line = std::find_if(wide_snapshot.features.begin(), wide_snapshot.features.end(),
        [&](const auto& feature) { return feature.net == a; });
    require(own_line != wide_snapshot.features.end() &&
                own_line->radius == Nanometres::from_mm(.925) &&
                own_line->bounds.minimum.y == Nanometres::from_mm(.075),
            "querying high-clearance net must be covered by snapshot envelope");
    const auto empty = make_live_copper_terrain_snapshot(Board{}, false, 4096U);
    require(empty != snapshot && live_copper_terrain_deposits(empty, a, LayerId{"F.Cu"}).empty(),
            "replacement snapshot must remove ripped copper pressure");

    OffsetFreeSpaceRegion region;
    region.area = {{point(0, 0), point(10, 0), point(10, 10), point(0, 10)}};
    NavMeshConfig nav;
    nav.cell_pitch = Nanometres::from_mm(.5);
    nav.maximum_cells = 10'000U;
    const auto prepared = prepare_navmesh_domain(region, nav);
    require(prepared.ok(), "pressure fixture must prepare");
    NavMeshHydraulicConfig hydraulic;
    hydraulic.flow.evolve_terrain_height = true;
    hydraulic.flow.prefer_gpu = false;
    hydraulic.downstream_feedback = foreign_front;
    const auto pressured = route_prepared_navmesh_hydraulic(prepared, point(1, 5), point(9, 5),
                                                            nav, hydraulic);
    require(pressured.ok() &&
                std::any_of(pressured.hydraulic_downstream_pressure_field.begin(),
                            pressured.hydraulic_downstream_pressure_field.end(),
                            [](std::uint16_t value) { return value != 0U; }),
            "foreign live copper must affect the next hydraulic terrain field");
    hydraulic.downstream_feedback = live_copper_terrain_deposits(empty, a, LayerId{"F.Cu"});
    const auto ripped = route_prepared_navmesh_hydraulic(prepared, point(1, 5), point(9, 5), nav,
                                                         hydraulic);
    require(ripped.ok() &&
                std::none_of(ripped.hydraulic_downstream_pressure_field.begin(),
                             ripped.hydraulic_downstream_pressure_field.end(),
                             [](std::uint16_t value) { return value != 0U; }),
            "ripped replacement copper must leave no hydraulic pressure");

    GlobalRouterConfig defaults;
    require(defaults.static_seed_hydraulic_flow.evolve_terrain_height &&
                defaults.static_seed_hydraulic_flow.prefer_gpu,
            "API terrain defaults must select GPU height evolution");
    Board numeric;
    numeric.rules.default_clearance = Nanometres{0};
    numeric.tracks.push_back({ObjectId{"odd"}, a, LayerId{"F.Cu"},
                              {Nanometres{std::numeric_limits<std::int64_t>::max()}, Nanometres{0}},
                              {Nanometres{std::numeric_limits<std::int64_t>::max()}, Nanometres{0}},
                              Nanometres{1}, false});
    const auto odd = make_live_copper_terrain_snapshot(numeric, false, 1U);
    require(odd.features.front().radius == Nanometres{1} &&
                odd.features.front().bounds.maximum.x ==
                    Nanometres{std::numeric_limits<std::int64_t>::max()},
            "odd widths round outward and terrain bounds saturate");
    std::cout << "live copper terrain snapshot tests passed\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
