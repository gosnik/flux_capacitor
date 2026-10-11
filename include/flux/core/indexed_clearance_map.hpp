#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace flux {

// An immutable, clearance-oriented view of prepared polygon boundaries.  This deliberately
// consumes polygon *segments*, rather than raw board objects: callers must first apply their
// own board/keepout/pad domain selection and clearance expansion.
struct IndexedClearancePrimitive {
  std::int64_t start_x_nm{0};
  std::int64_t start_y_nm{0};
  std::int64_t end_x_nm{0};
  std::int64_t end_y_nm{0};
  // Owner zero is the board exterior; non-zero owners conventionally identify holes/obstacles.
  std::uint32_t owner_id{0};
  std::optional<std::uint32_t> net_id;
  std::optional<std::uint32_t> layer_id;
  // Physical envelope metadata. owner_id resolves the authoritative owner record (including
  // its net membership and pairwise rules); polygon boundaries normally have radius zero.
  long double copper_radius_nm{0.0L};
};

enum class IndexedClearanceBuildStatus { ok, cancelled, invalid_input, limit_exceeded };

struct IndexedClearanceMapConfig {
  std::int64_t origin_x_nm{0};
  std::int64_t origin_y_nm{0};
  std::int64_t pitch_nm{50'000};
  std::uint32_t width{0};
  std::uint32_t height{0};
  std::uint64_t geometry_revision{0};
  // Empty means materialize every logical cell.  Non-empty means cell_record_indices still has
  // one entry per logical cell, but only these cells receive a rich record.  An invalid index is
  // "not sampled", never "free"; exact_query() remains available for every coordinate.
  std::vector<std::uint32_t> sampled_cells;
  std::uint64_t maximum_cells{4'000'000};
  std::uint64_t maximum_references{64'000'000};
  std::size_t maximum_bytes{512ULL * 1024ULL * 1024ULL};
  std::function<bool()> cancellation_requested;
  // Open copper segments use the same immutable BVH without polygon-parity cell records.
  bool materialize_cells{true};
};

struct IndexedClearanceCellRecord {
  double center_boundary_distance_nm{std::numeric_limits<double>::infinity()};
  double whole_cell_distance_lower_bound_nm{std::numeric_limits<double>::infinity()};
  std::uint32_t overlap_offset{0};
  std::uint32_t overlap_count{0};
  std::uint32_t nearest_primitive_index{std::numeric_limits<std::uint32_t>::max()};
  bool center_inside{false};
  bool boundary_touched{false};
};

struct IndexedClearancePointQuery {
  long double boundary_distance_nm{std::numeric_limits<long double>::infinity()};
  std::uint32_t nearest_primitive_index{std::numeric_limits<std::uint32_t>::max()};
  bool inside{false};
  bool on_boundary{false};
};

struct IndexedClearanceMapBuildResult;

class IndexedClearanceMap {
public:
  static constexpr std::uint32_t invalid_index = std::numeric_limits<std::uint32_t>::max();

  [[nodiscard]] std::uint64_t geometry_revision() const { return geometry_revision_; }
  [[nodiscard]] std::int64_t origin_x_nm() const { return origin_x_nm_; }
  [[nodiscard]] std::int64_t origin_y_nm() const { return origin_y_nm_; }
  [[nodiscard]] std::int64_t pitch_nm() const { return pitch_nm_; }
  [[nodiscard]] std::uint32_t width() const { return width_; }
  [[nodiscard]] std::uint32_t height() const { return height_; }
  [[nodiscard]] const std::vector<IndexedClearancePrimitive>& primitives() const {
    return primitives_;
  }
  [[nodiscard]] const std::vector<std::uint32_t>& cell_record_indices() const {
    return cell_record_indices_;
  }
  [[nodiscard]] const std::vector<IndexedClearanceCellRecord>& records() const { return records_; }
  [[nodiscard]] const std::vector<std::uint32_t>& overlap_primitive_indices() const {
    return overlap_primitive_indices_;
  }
  [[nodiscard]] const IndexedClearanceCellRecord* record_for_cell(std::uint32_t cell_index) const;
  // Exact BVH-backed result.  Net filtering is intentionally absent: ignoring same-net segment
  // boundaries changes even-odd domain semantics and could admit points outside board bounds.
  [[nodiscard]] IndexedClearancePointQuery exact_query(long double x_nm, long double y_nm) const;
  // Conservative broad phase only. Returned primitive IDs are sorted in insertion order.
  // Callers must perform exact segment distance and pairwise net/rule qualification.
  [[nodiscard]] std::vector<std::uint32_t> segment_candidates(
      long double ax, long double ay, long double bx, long double by,
      long double candidate_radius_nm, long double clearance_nm,
      std::optional<std::uint32_t> layer_id = {}) const;
  // Actual committed vector capacity, suitable for a bounded immutable-map cache.
  [[nodiscard]] std::size_t allocated_bytes() const;

public: // exposed only to keep the GPU-friendly storage layout inspectable.
  struct BvhNode {
    std::int64_t min_x{0}, min_y{0}, max_x{0}, max_y{0};
    std::uint32_t first{0}, count{0}, left{invalid_index}, right{invalid_index};
    long double maximum_copper_radius_nm{0.0L};
    [[nodiscard]] bool leaf() const { return count != 0U; }
  };

private:
  friend IndexedClearanceMapBuildResult
      build_indexed_clearance_map(IndexedClearanceMapConfig,
                                  std::vector<IndexedClearancePrimitive>);
  std::int64_t origin_x_nm_{0}, origin_y_nm_{0}, pitch_nm_{0};
  std::uint32_t width_{0}, height_{0};
  std::uint64_t geometry_revision_{0};
  std::vector<IndexedClearancePrimitive> primitives_;
  std::vector<std::uint32_t> primitive_order_;
  std::vector<BvhNode> bvh_;
  std::vector<std::uint32_t> cell_record_indices_;
  std::vector<IndexedClearanceCellRecord> records_;
  std::vector<std::uint32_t> overlap_primitive_indices_;
};

struct IndexedClearanceMapBuildResult {
  IndexedClearanceBuildStatus status{IndexedClearanceBuildStatus::invalid_input};
  std::shared_ptr<const IndexedClearanceMap> map;
  std::string diagnostic;
  std::size_t estimated_bytes{0};
};

[[nodiscard]] IndexedClearanceMapBuildResult
build_indexed_clearance_map(IndexedClearanceMapConfig config,
                            std::vector<IndexedClearancePrimitive> primitives);

} // namespace flux
