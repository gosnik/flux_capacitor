#pragma once

#include <chrono>
#include <cstddef>
#include <filesystem>
#include <stop_token>
#include <string>

namespace flux::io {

struct KicadZoneRefillRequest {
  std::filesystem::path python_executable;
  std::filesystem::path helper_script;
  std::filesystem::path provisional_board;
  std::filesystem::path refilled_board;
  std::filesystem::path diagnostic_directory;
  std::chrono::milliseconds timeout{std::chrono::minutes{15}};
  std::size_t maximum_report_bytes{64U * 1024U};
  std::uintmax_t maximum_candidate_bytes{512U * 1024U * 1024U};
};

struct KicadZoneRefillResult {
  bool run{false};
  bool passed{false};
  int exit_code{-1};
  std::string executable;
  std::string diagnostic;
};

// Runs the isolated KiCad Python zone-fill adapter without a command shell. This only certifies
// that the bounded helper transaction produced an artifact; callers must still strict-import the
// board, run exact internal acceptance, and invoke authoritative external KiCad DRC.
[[nodiscard]] KicadZoneRefillResult run_kicad_zone_refill(const KicadZoneRefillRequest& request,
                                                          std::stop_token stop_token = {});

} // namespace flux::io
