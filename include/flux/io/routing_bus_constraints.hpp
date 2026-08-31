#pragma once

#include "flux/core/board.hpp"
#include "flux/io/orientation_constraints.hpp"

#include <filesystem>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace flux::io {

struct RoutingBusConstraintResult {
  std::optional<Board> board;
  std::vector<RoutingBusId> loaded_buses;
  std::vector<ConstraintDiagnostic> diagnostics;

  [[nodiscard]] bool has_errors() const;
  [[nodiscard]] bool ok() const { return board.has_value() && !has_errors(); }
};

// Replaces the source board's explicit physical routing buses with a source-revision-bound list.
// Net names are checked against canonical stable IDs; no bus is inferred from naming alone.
[[nodiscard]] RoutingBusConstraintResult apply_routing_bus_constraints(const Board& source,
                                                                       std::string_view contents);

[[nodiscard]] RoutingBusConstraintResult
apply_routing_bus_constraints_file(const Board& source, const std::filesystem::path& path);

// Produces a deterministic empty constraint document. Users add explicit bus entries.
[[nodiscard]] std::string make_routing_bus_constraints_template(const Board& source);

} // namespace flux::io
