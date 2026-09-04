#include "flux/io/kicad_refill.hpp"

#include <boost/json.hpp>
#include <boost/process/v1.hpp>

#include <fstream>
#include <optional>
#include <thread>
#include <vector>

namespace flux::io {
namespace {

[[nodiscard]] std::optional<std::string> read_bounded_text_file(const std::filesystem::path& path,
                                                                std::size_t maximum_bytes) {
  std::error_code error;
  const auto size = std::filesystem::file_size(path, error);
  if (error || size > maximum_bytes) {
    return std::nullopt;
  }
  std::ifstream stream(path, std::ios::binary);
  if (!stream) {
    return std::nullopt;
  }
  std::string contents(static_cast<std::size_t>(size), '\0');
  stream.read(contents.data(), static_cast<std::streamsize>(contents.size()));
  if (!stream && !stream.eof()) {
    return std::nullopt;
  }
  return contents;
}

[[nodiscard]] bool regular_input_file(const std::filesystem::path& path) {
  std::error_code error;
  return std::filesystem::is_regular_file(path, error) && !error;
}

[[nodiscard]] bool regular_output_file(const std::filesystem::path& path) {
  std::error_code error;
  const auto status = std::filesystem::symlink_status(path, error);
  return !error && std::filesystem::is_regular_file(status) && !std::filesystem::is_symlink(status);
}

} // namespace

KicadZoneRefillResult run_kicad_zone_refill(const KicadZoneRefillRequest& request,
                                            std::stop_token stop_token) {
  namespace process = boost::process::v1;
  KicadZoneRefillResult result;
  result.executable = request.python_executable.generic_string();
  if (!regular_input_file(request.python_executable) ||
      !regular_input_file(request.helper_script) ||
      !regular_input_file(request.provisional_board)) {
    result.diagnostic = "zone-refill interpreter, helper, and provisional board must be regular "
                        "files";
    return result;
  }
  std::error_code file_error;
  if (!std::filesystem::is_directory(request.diagnostic_directory, file_error) || file_error ||
      request.refilled_board.parent_path() != request.diagnostic_directory ||
      std::filesystem::exists(request.refilled_board, file_error) || file_error) {
    result.diagnostic = "zone-refill output must be a new file in the diagnostic directory";
    return result;
  }
  if (request.timeout <= std::chrono::milliseconds::zero() || request.maximum_report_bytes == 0U ||
      request.maximum_candidate_bytes == 0U) {
    result.diagnostic = "zone-refill process and artifact bounds must be positive";
    return result;
  }

  const auto standard_output = request.diagnostic_directory / "zone-refill.stdout.log";
  const auto standard_error = request.diagnostic_directory / "zone-refill.stderr";
  const auto report = request.diagnostic_directory / "zone-refill.report.json";
  const std::vector<std::string> arguments{request.helper_script.string(),
                                           "--board",
                                           request.provisional_board.string(),
                                           "--output",
                                           request.refilled_board.string(),
                                           "--report",
                                           report.string()};
  try {
    process::child child(request.python_executable.string(), process::args(arguments),
                         process::std_out > standard_output.string(),
                         process::std_err > standard_error.string());
    result.run = true;
    const auto deadline = std::chrono::steady_clock::now() + request.timeout;
    while (child.running() && !stop_token.stop_requested() &&
           std::chrono::steady_clock::now() < deadline) {
      std::this_thread::sleep_for(std::chrono::milliseconds{25});
    }
    if (child.running()) {
      child.terminate();
      child.wait();
      result.diagnostic = stop_token.stop_requested() ? "KiCad zone refill cancelled"
                                                      : "KiCad zone refill exceeded its bound";
      return result;
    }
    child.wait();
    result.exit_code = child.exit_code();
  } catch (const std::exception& exception) {
    result.diagnostic = "unable to execute KiCad zone refill: " + std::string(exception.what());
    return result;
  }

  const auto stdout_contents =
      read_bounded_text_file(standard_output, request.maximum_report_bytes);
  const auto stderr_contents = read_bounded_text_file(standard_error, request.maximum_report_bytes);
  if (result.exit_code != 0) {
    result.diagnostic = stderr_contents && !stderr_contents->empty()   ? *stderr_contents
                        : stdout_contents && !stdout_contents->empty() ? *stdout_contents
                                                                       : "KiCad zone refill failed";
    return result;
  }
  const auto report_contents = read_bounded_text_file(report, request.maximum_report_bytes);
  boost::system::error_code parse_error;
  const auto parsed =
      report_contents ? boost::json::parse(*report_contents, parse_error) : boost::json::value{};
  if (!report_contents || report_contents->empty() || parse_error || !parsed.is_object()) {
    result.diagnostic = "KiCad zone refill returned success without a valid bounded JSON report";
    return result;
  }
  const auto* ok = parsed.as_object().if_contains("ok");
  if (ok == nullptr || !ok->is_bool() || !ok->as_bool()) {
    result.diagnostic = "KiCad zone refill JSON report did not approve the candidate";
    return result;
  }
  if (!regular_output_file(request.refilled_board)) {
    result.diagnostic = "KiCad zone refill reported success without a regular output board";
    return result;
  }
  const auto candidate_size = std::filesystem::file_size(request.refilled_board, file_error);
  if (file_error || candidate_size == 0U || candidate_size > request.maximum_candidate_bytes) {
    result.diagnostic = "KiCad zone refill output is empty, unreadable, or exceeds its bound";
    return result;
  }
  result.passed = true;
  result.diagnostic = "KiCad zone refill produced a bounded candidate";
  return result;
}

} // namespace flux::io
