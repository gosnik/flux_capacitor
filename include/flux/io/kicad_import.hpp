#pragma once

#include "flux/core/board.hpp"
#include "flux/core/units.hpp"

#include <filesystem>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace flux::io {

enum class ImportSeverity { warning, error };

struct ImportDiagnostic {
  ImportSeverity severity{ImportSeverity::error};
  std::string code;
  std::string message;
  std::size_t line{1};
  std::size_t column{1};
};

struct KicadImportOptions {
  Nanometres arc_chord_tolerance{Nanometres::from_mm(0.05)};
  bool derive_tentative_placement_envelopes{false};
  // Compatibility alias for callers predating tentative envelope provenance.
  bool approximate_missing_courtyards{false};
};

struct KicadImportResult {
  std::optional<Board> board;
  std::vector<ImportDiagnostic> diagnostics;
  bool lossy{false};
  // File import loads sibling project fabrication minima without changing netclass defaults.
  // This does not claim to resolve custom rules or effective netclass memberships.
  bool project_minimum_rules_applied{false};
  std::string project_rules_fingerprint;

  [[nodiscard]] bool has_errors() const;
  [[nodiscard]] bool ok() const { return board.has_value() && !has_errors(); }
  [[nodiscard]] bool strict() const { return ok() && !lossy; }
};

[[nodiscard]] KicadImportResult import_kicad_pcb(std::string_view contents,
                                                 const KicadImportOptions& options = {});

[[nodiscard]] KicadImportResult import_kicad_pcb_file(const std::filesystem::path& path,
                                                      const KicadImportOptions& options = {});

} // namespace flux::io
