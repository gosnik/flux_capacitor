#pragma once

#include "flux/core/board.hpp"

#include <filesystem>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace flux::io {

enum class NetClassSnapshotSeverity { warning, error };

struct NetClassSnapshotDiagnostic {
  NetClassSnapshotSeverity severity{NetClassSnapshotSeverity::error};
  std::string code;
  std::string message;
  std::size_t line{1};
  std::size_t column{1};
  std::vector<std::string> object_ids;
};

struct KicadNetClassSnapshotResult {
  std::optional<Board> board;
  std::string api_version;
  std::size_t applied_classes{0};
  std::size_t applied_nets{0};
  std::vector<NetClassSnapshotDiagnostic> diagnostics;

  [[nodiscard]] bool has_errors() const;
  [[nodiscard]] bool ok() const { return board.has_value() && !has_errors(); }
};

// Applies the stable output of the thin kicad-python IPC bridge to an immutable board snapshot.
// The input must cover every named net and be bound to the exact imported board revision.
[[nodiscard]] KicadNetClassSnapshotResult apply_kicad_netclass_snapshot(const Board& source,
                                                                        std::string_view contents);

[[nodiscard]] KicadNetClassSnapshotResult
apply_kicad_netclass_snapshot_file(const Board& source, const std::filesystem::path& path);

} // namespace flux::io
