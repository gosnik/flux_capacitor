// Replays quantized PH Bezier controls from one recorded regional candidate through the native
// line/arc fitter. This is diagnostic timing input, never an authoritative routing path.
#include "flux/core/spline_route.hpp"
#include "flux/telemetry/event.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <set>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace {

struct RecordedRoute {
  std::string id;
  flux::QuinticPHRoute route;
};

[[nodiscard]] flux::QuinticPHRoute rebuild_recorded_route(const flux::telemetry::Route& recorded) {
  if (recorded.control_points.empty() || recorded.control_points.size() % 6U != 0U)
    throw std::runtime_error("recorded PH controls are not a sequence of quintic Bezier spans");
  flux::QuinticPHRoute route;
  const auto span_count = recorded.control_points.size() / 6U;
  route.segments.reserve(span_count);
  for (std::size_t span = 0U; span < span_count; ++span) {
    flux::QuinticPHSegment segment;
    for (std::size_t control = 0U; control < segment.bezier.size(); ++control) {
      const auto& point = recorded.control_points[span * 6U + control];
      segment.bezier[control] = {static_cast<long double>(point.x_nm),
                                 static_cast<long double>(point.y_nm)};
    }
    segment.route_parameter_start =
        static_cast<long double>(span) / static_cast<long double>(span_count);
    segment.route_parameter_end =
        static_cast<long double>(span + 1U) / static_cast<long double>(span_count);
    route.segments.push_back(std::move(segment));
  }
  route.tangent_continuous = true;
  return route;
}

} // namespace

int main(int argc, char** argv) {
  try {
    if (argc != 4)
      throw std::runtime_error("usage: flux-replay-native-fit TELEMETRY.jsonl WINDOW RANK");
    const std::string window = argv[2];
    const auto rank = static_cast<std::uint64_t>(std::stoull(argv[3]));
    const std::string prefix = "regional-spline:" + window + ":";
    std::ifstream input(argv[1]);
    if (!input)
      throw std::runtime_error("cannot read telemetry");
    std::vector<RecordedRoute> routes;
    std::set<std::string> seen;
    bool candidate_started = false;
    bool candidate_complete = false;
    std::string line;
    while (!candidate_complete && std::getline(input, line)) {
      // Snapshot records dominate diagnostic traces and cannot contain candidate controls or the
      // per-window completion delimiter. Avoid constructing their large property trees.
      if (line.find("\"kind\":\"RouteCandidateSelected\"") == std::string::npos &&
          line.find("\"kind\":\"SearchSummary\"") == std::string::npos)
        continue;
      const auto decoded = flux::telemetry::decode_events(line);
      if (!decoded)
        throw std::runtime_error("malformed telemetry: " + decoded.error);
      for (const auto& event : decoded.events) {
        if (candidate_started && event.type == flux::telemetry::EventType::search_summary &&
            event.active_group == "regional-spline-progress:" + window) {
          candidate_complete = true;
          break;
        }
        if (event.type != flux::telemetry::EventType::route_candidate_selected)
          continue;
        for (const auto& recorded : event.routes) {
          if (!recorded.id.starts_with(prefix) || recorded.geometry != flux::telemetry::RouteGeometry::quintic_ph ||
              recorded.revision != rank + 1U || !seen.insert(recorded.id).second)
            continue;
          routes.push_back({recorded.id, rebuild_recorded_route(recorded)});
          candidate_started = true;
        }
      }
    }
    if (routes.empty())
      throw std::runtime_error("no matching recorded regional PH routes");

    std::uint64_t total_us = 0U;
    std::cout << "{\"authoritative\":false,\"window\":" << window << ",\"rank\":" << rank
              << ",\"routes\":[";
    for (std::size_t index = 0U; index < routes.size(); ++index) {
      flux::NativeSplineFitStats stats;
      const auto started = std::chrono::steady_clock::now();
      const auto pieces = flux::fit_quintic_ph_to_native_copper(routes[index].route, {}, {}, &stats);
      const auto elapsed = static_cast<std::uint64_t>(
          std::chrono::duration_cast<std::chrono::microseconds>(
              std::chrono::steady_clock::now() - started).count());
      total_us += elapsed;
      std::cout << (index ? ",{" : "{") << "\"id\":\"" << routes[index].id
                << "\",\"spans\":" << routes[index].route.segments.size()
                << ",\"pieces\":" << pieces.size() << ",\"elapsed_us\":" << elapsed
                << ",\"attempts\":" << stats.attempts
                << ",\"peak_samples\":" << stats.peak_samples
                << ",\"tessellation_us\":" << stats.tessellation_us
                << ",\"lattice_filter_us\":" << stats.lattice_filter_us
                << ",\"line_arc_fit_us\":" << stats.line_arc_fit_us
                << ",\"tangent_biarc_fit_us\":" << stats.tangent_biarc_fit_us << '}';
    }
    std::cout << "],\"route_count\":" << routes.size() << ",\"total_us\":" << total_us
              << "}\n";
    return 0;
  } catch (const std::exception& exception) {
    std::cerr << exception.what() << '\n';
    return 1;
  }
}
