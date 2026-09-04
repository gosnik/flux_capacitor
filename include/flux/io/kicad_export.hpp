#pragma once

#include "flux/core/board.hpp"

#include <filesystem>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace flux::io {

enum class ExportSeverity { warning, error };

struct ExportDiagnostic {
  ExportSeverity severity{ExportSeverity::error};
  std::string code;
  std::string message;
  std::vector<std::string> object_ids;
};

struct ExportedObjectId {
  ObjectId canonical_id;
  std::string kicad_uuid;
};

struct KicadExportOptions {
  std::string generator{"flux_capacitor"};
  std::string generator_version{"0.130.0"};
  bool verify_round_trip{true};
  // Write a review-only copper artifact from a lossy/invalid canonical projection. Source byte
  // identity and supported-design-change checks remain enforced, but strict semantic round-trip
  // qualification is deliberately unavailable in this mode.
  bool allow_lossy_diagnostic_preview{false};
};

struct KicadExportResult {
  std::optional<std::string> contents;
  std::vector<ExportDiagnostic> diagnostics;
  std::vector<ExportedObjectId> generated_id_map;

  [[nodiscard]] bool has_errors() const;
  [[nodiscard]] bool ok() const { return contents.has_value() && !has_errors(); }
};

// Produces a new board artifact by patching only supported footprint positions and
// top-level track/via objects. Unknown source syntax is retained verbatim. The source
// text must match the imported source snapshot, and a result is withheld if strict
// semantic round-trip verification fails.
[[nodiscard]] KicadExportResult export_kicad_candidate(std::string_view source_contents,
                                                       const Board& source, const Board& candidate,
                                                       const KicadExportOptions& options = {});

// Reads source_path and writes output_path only after export validation. Existing
// output files are never overwritten.
[[nodiscard]] KicadExportResult
export_kicad_candidate_file(const std::filesystem::path& source_path,
                            const std::filesystem::path& output_path, const Board& source,
                            const Board& candidate, const KicadExportOptions& options = {});

enum class KicadChangeOperation { create, update, remove };

struct KicadFootprintTransformState {
  Point position;
  Microdegrees rotation;
  BoardSide side{BoardSide::front};

  auto operator<=>(const KicadFootprintTransformState&) const = default;
};

struct KicadFootprintTransformChange {
  ObjectId id;
  std::string reference;
  KicadFootprintTransformState source;
  KicadFootprintTransformState candidate;

  bool operator==(const KicadFootprintTransformChange&) const = default;
};

struct KicadFootprintSnapshot {
  ObjectId id;
  std::string reference;
  KicadFootprintTransformState state;

  bool operator==(const KicadFootprintSnapshot&) const = default;
};

struct KicadTrackChangeState {
  ObjectId id;
  std::string net_name;
  LayerId layer;
  Point start;
  Point end;
  Nanometres width;
  bool locked{false};

  auto operator<=>(const KicadTrackChangeState&) const = default;
};

struct KicadTrackArcChangeState {
  ObjectId id;
  std::string net_name;
  LayerId layer;
  Point start;
  Point middle;
  Point end;
  Nanometres width;
  bool locked{false};

  auto operator<=>(const KicadTrackArcChangeState&) const = default;
};

struct KicadViaChangeState {
  ObjectId id;
  std::string net_name;
  Point position;
  Nanometres diameter;
  Nanometres drill;
  std::vector<LayerId> layers;
  bool locked{false};

  auto operator<=>(const KicadViaChangeState&) const = default;
};

template <typename State> struct KicadRoutingChange {
  KicadChangeOperation operation{KicadChangeOperation::create};
  std::optional<State> source;
  std::optional<State> candidate;

  bool operator==(const KicadRoutingChange&) const = default;
};

struct KicadChangeSet {
  std::uint32_t version{1};
  std::uint32_t board_schema{board_schema_version};
  std::string source_revision;
  std::vector<KicadFootprintSnapshot> source_footprints;
  std::vector<KicadTrackChangeState> source_tracks;
  std::vector<KicadTrackArcChangeState> source_track_arcs;
  std::vector<KicadViaChangeState> source_vias;
  std::vector<KicadFootprintTransformChange> footprint_transforms;
  std::vector<KicadRoutingChange<KicadTrackChangeState>> tracks;
  std::vector<KicadRoutingChange<KicadTrackArcChangeState>> track_arcs;
  std::vector<KicadRoutingChange<KicadViaChangeState>> vias;

  bool operator==(const KicadChangeSet&) const = default;
};

struct KicadChangeSetResult {
  std::optional<KicadChangeSet> change_set;
  std::vector<ExportDiagnostic> diagnostics;

  [[nodiscard]] bool has_errors() const;
  [[nodiscard]] bool ok() const { return change_set.has_value() && !has_errors(); }
};

// Accepts only a strict candidate artifact that the source-preserving exporter reproduces
// byte-for-byte from source_contents. The resulting DTO contains exact live-object preconditions
// and complete supported destinations for an IPC preview or transaction.
[[nodiscard]] KicadChangeSetResult make_kicad_change_set(std::string_view source_contents,
                                                         std::string_view candidate_contents,
                                                         const KicadExportOptions& options = {});

[[nodiscard]] KicadChangeSetResult
make_kicad_change_set_file(const std::filesystem::path& source_path,
                           const std::filesystem::path& candidate_path,
                           const KicadExportOptions& options = {});

} // namespace flux::io
