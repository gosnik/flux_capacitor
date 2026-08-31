#pragma once

#include "flux/core/board.hpp"
#include "flux/core/evaluate.hpp"

#include <filesystem>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace flux::io {

enum class SpecctraSeverity { warning, error };

struct SpecctraDiagnostic {
  SpecctraSeverity severity{SpecctraSeverity::error};
  std::string code;
  std::string message;
  std::size_t line{1};
  std::size_t column{1};
  std::vector<std::string> object_ids;
};

struct SpecctraDsnExportOptions {
  std::string design_name{"flux-design.dsn"};
};

struct SpecctraDsnExportResult {
  std::optional<std::string> contents;
  std::vector<SpecctraDiagnostic> diagnostics;
  std::size_t exported_layers{0};
  std::size_t exported_pins{0};
  std::size_t protected_tracks{0};
  std::size_t protected_vias{0};

  [[nodiscard]] bool has_errors() const;
  [[nodiscard]] bool ok() const { return contents.has_value() && !has_errors(); }
};

// Produces a deterministic Specctra design at 100-nanometre session resolution. Unsupported
// physical semantics fail closed; no output is returned alongside an error diagnostic.
[[nodiscard]] SpecctraDsnExportResult
export_specctra_dsn(const Board& board, const SpecctraDsnExportOptions& options = {});

// Existing output artifacts are never overwritten.
[[nodiscard]] SpecctraDsnExportResult
export_specctra_dsn_file(const Board& board, const std::filesystem::path& output_path);

struct SpecctraSesImportResult {
  std::optional<Board> candidate;
  std::optional<EvaluationResult> evaluation;
  std::vector<SpecctraDiagnostic> diagnostics;
  std::size_t imported_tracks{0};
  std::size_t imported_vias{0};
  std::size_t verified_protected_tracks{0};
  std::size_t verified_protected_vias{0};

  [[nodiscard]] bool has_errors() const;
  [[nodiscard]] bool ok() const { return candidate.has_value() && !has_errors(); }
};

// The DSN bytes are a provenance artifact: they must be exactly reproducible from source. The
// session may add routing, but may not change placement or omit/replace protected source copper.
[[nodiscard]] SpecctraSesImportResult import_specctra_session(const Board& source,
                                                              std::string_view dsn_contents,
                                                              std::string_view ses_contents);

} // namespace flux::io
