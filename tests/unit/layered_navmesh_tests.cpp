#include "flux/core/layered_navmesh.hpp"

#include <algorithm>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <stop_token>

namespace {
using namespace flux;
[[noreturn]] void fail(const char* message) {
  std::cerr << message << '\n';
  std::exit(1);
}
void require(bool value, const char* message) {
  if (!value)
    fail(message);
}

std::shared_ptr<const PreparedNavMeshDomain> domain(bool block_center = false) {
  OffsetFreeSpaceRegion region;
  region.area = Polygon{{{Nanometres{0}, Nanometres{0}},
                         {Nanometres{100}, Nanometres{0}},
                         {Nanometres{100}, Nanometres{100}},
                         {Nanometres{0}, Nanometres{100}}}};
  if (block_center) {
    region.holes.push_back(Polygon{{{Nanometres{40}, Nanometres{40}},
                                    {Nanometres{60}, Nanometres{40}},
                                    {Nanometres{60}, Nanometres{60}},
                                    {Nanometres{40}, Nanometres{60}}}});
  }
  NavMeshConfig config;
  config.cell_pitch = Nanometres{25};
  config.maximum_cells = 64U;
  return std::make_shared<const PreparedNavMeshDomain>(prepare_navmesh_domain(region, config));
}

PreparedLayeredNavMeshDomain prepared(const LayeredNavMeshViaCertification& certificate) {
  auto first = domain();
  auto middle = domain();
  auto last = domain();
  return prepare_layered_navmesh_domain({{LayerId{"F.Cu"}, 0U, {first}},
                                         {LayerId{"In1.Cu"}, 2U, {middle}},
                                         {LayerId{"B.Cu"}, 4U, {last}}},
                                        {{7U,
                                          {Nanometres{50}, Nanometres{50}},
                                          LayerId{"F.Cu"},
                                          LayerId{"B.Cu"},
                                          LayerId{"F.Cu"},
                                          LayerId{"B.Cu"},
                                          Nanometres{1}}},
                                        certificate);
}

void forced_layer_change_keeps_full_span_and_portals() {
  const auto graph = prepared([](const auto&) { return true; });
  const auto route =
      route_prepared_layered_navmesh(graph, {Nanometres{10}, Nanometres{10}}, LayerId{"F.Cu"},
                                     {Nanometres{90}, Nanometres{90}}, LayerId{"B.Cu"});
  require(route.ok(), "a certified via zone must enable a forced layer change");
  bool transition = false, planar = false;
  for (const auto& edge : route.edges) {
    transition |= edge.kind == LayeredNavMeshEdgeKind::transition;
    planar |= edge.kind == LayeredNavMeshEdgeKind::planar;
    if (edge.kind == LayeredNavMeshEdgeKind::transition)
      require(edge.transition.span ==
                  std::vector<LayerId>{LayerId{"F.Cu"}, LayerId{"In1.Cu"}, LayerId{"B.Cu"}},
              "a via transition must retain its complete physical copper span");
  }
  require(transition && planar, "route must expose both transition and planar graph edges");
}

void electrical_inner_transition_retains_through_barrel_span() {
  const auto first = domain();
  const auto middle = domain();
  const auto last = domain();
  const auto graph = prepare_layered_navmesh_domain({{LayerId{"F.Cu"}, 0U, {first}},
                                                     {LayerId{"In1.Cu"}, 2U, {middle}},
                                                     {LayerId{"B.Cu"}, 4U, {last}}},
                                                    {{9U,
                                                      {Nanometres{50}, Nanometres{50}},
                                                      LayerId{"F.Cu"},
                                                      LayerId{"In1.Cu"},
                                                      LayerId{"F.Cu"},
                                                      LayerId{"B.Cu"},
                                                      Nanometres{1}}},
                                                    [](const auto&) { return true; });
  const auto route =
      route_prepared_layered_navmesh(graph, {Nanometres{10}, Nanometres{10}}, LayerId{"F.Cu"},
                                     {Nanometres{90}, Nanometres{90}}, LayerId{"In1.Cu"});
  require(route.ok(), "through via must connect its electrical endpoints");
  const auto via = std::find_if(route.edges.begin(), route.edges.end(), [](const auto& edge) {
    return edge.kind == LayeredNavMeshEdgeKind::transition;
  });
  require(via != route.edges.end() && via->transition.span.back() == LayerId{"B.Cu"},
          "inner electrical endpoint must not truncate physical through-via occupancy");
}

void intermediate_layer_must_be_certified() {
  const auto first = domain();
  const auto blocked_middle = domain(true);
  const auto last = domain();
  const auto graph = prepare_layered_navmesh_domain({{LayerId{"F.Cu"}, 0U, {first}},
                                                     {LayerId{"In1.Cu"}, 2U, {blocked_middle}},
                                                     {LayerId{"B.Cu"}, 4U, {last}}},
                                                    {{7U,
                                                      {Nanometres{50}, Nanometres{50}},
                                                      LayerId{"F.Cu"},
                                                      LayerId{"B.Cu"},
                                                      LayerId{"F.Cu"},
                                                      LayerId{"B.Cu"},
                                                      Nanometres{1}}},
                                                    [](const auto&) { return true; });
  const auto route =
      route_prepared_layered_navmesh(graph, {Nanometres{10}, Nanometres{10}}, LayerId{"F.Cu"},
                                     {Nanometres{90}, Nanometres{90}}, LayerId{"B.Cu"});
  require(!route.ok() && route.diagnostic.find("disconnected") != std::string::npos,
          "rejecting the full span must remove its through-via transition");
}

void same_layer_route_retains_planar_portals() {
  const auto graph = prepared([](const auto&) { return true; });
  const auto route =
      route_prepared_layered_navmesh(graph, {Nanometres{10}, Nanometres{10}}, LayerId{"F.Cu"},
                                     {Nanometres{90}, Nanometres{90}}, LayerId{"F.Cu"});
  require(route.ok() && !route.edges.empty(), "same-layer cell route must be available");
  require(std::all_of(route.edges.begin(), route.edges.end(),
                      [](const auto& edge) { return edge.kind == LayeredNavMeshEdgeKind::planar; }),
          "same-layer route must preserve planar portal edges without invented vias");
}

void constrained_search_must_choose_via_zones() {
  const auto graph = prepared([](const auto&) { return true; });
  const auto route = route_prepared_layered_navmesh(
      graph, {Nanometres{10}, Nanometres{10}}, LayerId{"F.Cu"}, {Nanometres{90}, Nanometres{90}},
      LayerId{"F.Cu"}, {500'000U, 2U, 2U});
  require(route.ok(), "bounded transition search must find the two-via overpass");
  require(std::count_if(route.edges.begin(), route.edges.end(),
                        [](const auto& edge) {
                          return edge.kind == LayeredNavMeshEdgeKind::transition;
                        }) == 2U,
          "transition constraint must prevent a zero-via original-layer shortcut");
}

void invalid_layers_and_bounded_cancellation_are_explicit() {
  const auto graph = prepared([](const auto&) { return true; });
  const auto unknown =
      route_prepared_layered_navmesh(graph, {Nanometres{10}, Nanometres{10}}, LayerId{"X.Cu"},
                                     {Nanometres{90}, Nanometres{90}}, LayerId{"B.Cu"});
  require(!unknown.ok() && unknown.diagnostic.find("unknown") != std::string::npos,
          "unknown layer must not become a graph vertex");
  const auto duplicate = prepare_layered_navmesh_domain({{LayerId{"F.Cu"}, 0U, {domain()}},
                                                         {LayerId{"In1.Cu"}, 1U, {domain()}},
                                                         {LayerId{"F.Cu"}, 2U, {domain()}}},
                                                        {}, [](const auto&) { return true; });
  require(!duplicate.ok() && duplicate.diagnostic.find("unique") != std::string::npos,
          "non-adjacent duplicate copper identities must not produce ambiguous layer nodes");
  const auto bounded =
      route_prepared_layered_navmesh(graph, {Nanometres{10}, Nanometres{10}}, LayerId{"F.Cu"},
                                     {Nanometres{90}, Nanometres{90}}, LayerId{"B.Cu"}, {0U});
  require(bounded.search_budget_exhausted, "zero search budget must be reported");
  const auto state_bounded = route_prepared_layered_navmesh(
      graph, {Nanometres{10}, Nanometres{10}}, LayerId{"F.Cu"}, {Nanometres{90}, Nanometres{90}},
      LayerId{"B.Cu"}, {500'000U, 0U, 8U, 1U, 16U});
  require(state_bounded.search_budget_exhausted,
          "transition-state allocation bound must be reported before search allocation");
  std::stop_source source;
  source.request_stop();
  const auto cancelled = route_prepared_layered_navmesh(
      graph, {Nanometres{10}, Nanometres{10}}, LayerId{"F.Cu"}, {Nanometres{90}, Nanometres{90}},
      LayerId{"B.Cu"}, {}, source.get_token());
  require(cancelled.cancelled, "requested stop must cancel layered search");
  const auto exhausted = prepare_layered_navmesh_domain(
      {{LayerId{"F.Cu"}, 0U, {domain()}}, {LayerId{"B.Cu"}, 2U, {domain()}}}, {},
      [](const auto&) { return true; }, {1U, 10U, 1U, 1U});
  require(exhausted.preparation_bound_exhausted,
          "node-bound preparation must report incomplete graph");
}

} // namespace
int main() {
  forced_layer_change_keeps_full_span_and_portals();
  electrical_inner_transition_retains_through_barrel_span();
  intermediate_layer_must_be_certified();
  same_layer_route_retains_planar_portals();
  constrained_search_must_choose_via_zones();
  invalid_layers_and_bounded_cancellation_are_explicit();
}
