#pragma once

#include "flux/core/board.hpp"

#include <cstddef>
#include <map>
#include <vector>

namespace flux {

// Each chord is certified to be within centerline_deviation_nm of the mathematical cubic before
// integer endpoint quantisation, and within quantization_error_nm after it.  Consumers requiring
// an outer copper proxy enlarge its radius by their sum; connectivity consumers reduce the radius
// by their sum.  The latter can miss marginal contact but cannot invent it.
struct BezierFlattenedSegment {
  Point start;
  Point end;
  Nanometres centerline_deviation_nm;
  Nanometres quantization_error_nm;
};

struct BezierFlattening {
  std::vector<BezierFlattenedSegment> segments;
  Nanometres maximum_deviation_nm{Nanometres{0}};
  bool certified{false};
  long double length_lower_nm{0.0L};
  long double length_upper_nm{0.0L};
};

// Uses adaptive de Casteljau subdivision. `tolerance` bounds the unquantised cubic-to-chord
// deviation; integer point rounding is separately reported for every chord.
[[nodiscard]] BezierFlattening tessellate_track_bezier(
    const TrackBezier& track, Nanometres tolerance = Nanometres{1'000},
    std::size_t maximum_depth = 32U);

// An obstacle/planning view. Curves are removed only from the returned copy and replaced by
// widened, uniquely identified straight proxies. `proxy_sources` maps every generated proxy ID
// back to its canonical curve ID so diagnostics can retain source identity.
struct ConservativeBezierBoard {
  Board board;
  std::map<std::string, ObjectId> proxy_sources;
  bool certified{false};
};

[[nodiscard]] ConservativeBezierBoard flatten_board_beziers_conservatively(
    const Board& board, Nanometres tolerance = Nanometres{1'000},
    std::size_t maximum_depth = 32U);

} // namespace flux
