#include "flux/core/spline_route.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <stop_token>

namespace {
void require(bool ok, const char* message) {
  if (!ok) {
    std::cerr << message << '\n';
    std::exit(1);
  }
}
flux::QuinticPHRoute straight_route() {
  flux::QuinticPHRoute route;
  flux::QuinticPHSegment segment;
  for (std::size_t i = 0; i < 6; ++i)
    segment.bezier[i] = {static_cast<long double>(i) * 1'000'000.0L, 0.0L};
  segment.route_parameter_start = 0.0L;
  segment.route_parameter_end = 1.0L;
  route.segments.push_back(segment);
  return route;
}

flux::QuinticPHRoute curved_route() {
  flux::QuinticPHRoute route;
  flux::QuinticPHSegment segment;
  segment.bezier = {{{0.0L, 0.0L},
                     {500'000.0L, 2'000'000.0L},
                     {1'500'000.0L, 3'000'000.0L},
                     {3'000'000.0L, -2'000'000.0L},
                     {4'000'000.0L, -1'000'000.0L},
                     {5'000'000.0L, 0.0L}}};
  segment.route_parameter_start = 0.0L;
  segment.route_parameter_end = 1.0L;
  route.segments.push_back(segment);
  return route;
}

flux::SplineVector evaluate(const std::array<flux::SplineVector, 6U>& controls, long double t) {
  auto values = controls;
  for (std::size_t count = controls.size(); count > 1U; --count)
    for (std::size_t index = 0U; index + 1U < count; ++index)
      values[index] = {values[index].x + (values[index + 1U].x - values[index].x) * t,
                       values[index].y + (values[index + 1U].y - values[index].y) * t};
  return values.front();
}

flux::SplineVector evaluate_cubic(const flux::SplineFittedPiece& piece, long double t) {
  const std::array<flux::SplineVector, 4U> controls{{
      {static_cast<long double>(piece.start.x.value()), static_cast<long double>(piece.start.y.value())},
      {static_cast<long double>(piece.control1.x.value()), static_cast<long double>(piece.control1.y.value())},
      {static_cast<long double>(piece.control2.x.value()), static_cast<long double>(piece.control2.y.value())},
      {static_cast<long double>(piece.end.x.value()), static_cast<long double>(piece.end.y.value())},
  }};
  auto values = controls;
  for (std::size_t count = controls.size(); count > 1U; --count)
    for (std::size_t index = 0U; index + 1U < count; ++index)
      values[index] = {values[index].x + (values[index + 1U].x - values[index].x) * t,
                       values[index].y + (values[index + 1U].y - values[index].y) * t};
  return values.front();
}
} // namespace

int main() {
  flux::NativeSplineFitConfig config;
  config.fit_tolerance = flux::Nanometres{10};
  config.maximum_depth = 4;
  config.maximum_pieces = 8;
  const auto pieces = flux::fit_quintic_ph_to_cubic_copper(straight_route(), config);
  require(pieces.size() == 1, "a straight quintic must fit as one cubic");
  require(pieces.front().kind == flux::SplineFittedPieceKind::cubic_bezier,
          "the fitter must publish cubic geometry");
  require(pieces.front().start == flux::Point{flux::Nanometres{0}, flux::Nanometres{0}} &&
              pieces.front().end == flux::Point{flux::Nanometres{5'000'000}, flux::Nanometres{0}},
          "cubic endpoints must be preserved");
  std::stop_source stop;
  stop.request_stop();
  require(flux::fit_quintic_ph_to_cubic_copper(straight_route(), config, stop.get_token()).empty(),
          "cancellation must fail closed");
  config.maximum_pieces = 0;
  require(flux::fit_quintic_ph_to_cubic_copper(straight_route(), config).empty(),
          "zero piece budget must fail closed");

  config.fit_tolerance = flux::Nanometres{20'000};
  config.maximum_depth = 10;
  config.maximum_pieces = 64;
  config.maximum_join_angle_radians = 0.1L;
  const auto curved = curved_route();
  const auto curved_pieces = flux::fit_quintic_ph_to_cubic_copper(curved, config);
  require(curved_pieces.size() > 1U && curved_pieces.size() <= config.maximum_pieces,
          "a nontrivial quintic must subdivide into a bounded cubic chain");
  for (std::size_t index = 1U; index < curved_pieces.size(); ++index)
    require(curved_pieces[index - 1U].end == curved_pieces[index].start,
            "adjacent fitted cubics must share an exact quantized endpoint");
  for (std::size_t sample = 0U; sample <= 512U; ++sample) {
    const auto parameter = static_cast<long double>(sample) / 512.0L;
    const auto found = std::find_if(curved_pieces.begin(), curved_pieces.end(),
                                    [parameter](const auto& piece) {
                                      return parameter >= piece.first_parameter - 1e-12L &&
                                             parameter <= piece.last_parameter + 1e-12L;
                                    });
    require(found != curved_pieces.end(), "every quintic sample must map to a fitted cubic span");
    const auto local = (parameter - found->first_parameter) /
                       (found->last_parameter - found->first_parameter);
    const auto expected = evaluate(curved.segments.front().bezier, parameter);
    const auto actual = evaluate_cubic(*found, local);
    require(std::hypotl(expected.x - actual.x, expected.y - actual.y) <= 20'010.0L,
            "dense quintic samples must stay within the cubic fit tolerance");
  }
  auto limited = config;
  limited.maximum_pieces = 1;
  require(flux::fit_quintic_ph_to_cubic_copper(curved, limited).empty(),
          "an insufficient cubic piece budget must fail closed");
  return 0;
}
