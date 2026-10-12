#pragma once

#include <boost/polygon/polygon.hpp>

#include <algorithm>
#include <bit>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <unordered_map>
#include <utility>
#include <vector>

namespace flux::detail {

// Internal immutable Boolean DAG. Stable spatial/hash radix partitions preserve unaffected
// subtrees when a net view omits leaves or copper changes. Exclusions must precede this API.
class FreeSpaceBooleanUnionCache {
 public:
  using Set = boost::polygon::polygon_set_data<std::int32_t>;
  using SetPtr = std::shared_ptr<const Set>;
  struct Input {
    SetPtr shape;  // Normalized before publication; never call a mutating Boost query on it.
    std::uint64_t spatial_key{0};
    std::uint64_t geometry_key{0};
  };
  struct Stats { std::size_t hits{0}; std::size_t misses{0}; std::size_t retained_units{0}; };

  explicit FreeSpaceBooleanUnionCache(std::size_t capacity)
      : capacity_(capacity) {}

  [[nodiscard]] SetPtr unite(std::vector<Input> input) {
    if (input.empty()) return std::make_shared<const Set>();
    std::stable_sort(input.begin(), input.end(), [](const Input& lhs, const Input& rhs) {
      return std::pair{lhs.spatial_key, lhs.geometry_key} <
             std::pair{rhs.spatial_key, rhs.geometry_key};
    });
    return build(input, 0U, input.size()).shape;
  }

  [[nodiscard]] Stats stats() const {
    std::lock_guard lock(mutex_);
    return {stats_.hits, stats_.misses, retained_units_};
  }

  void clear() {
    std::lock_guard lock(mutex_);
    entries_.clear();
    order_.clear();
    retained_units_ = 0U;
    stats_ = {};
  }

 private:
  struct Result {
    SetPtr shape;
    // Includes this cached set and its directly retained operand sets. Operands are value
    // snapshots, not owning DAG nodes, so they cannot retain uncounted descendants.
    std::size_t units{0};
  };
  struct Key {
    const Set* left;
    const Set* right;
    bool operator==(const Key&) const = default;
  };
  struct Hash {
    std::size_t operator()(const Key& key) const {
      const auto a = std::hash<const Set*>{}(key.left);
      const auto b = std::hash<const Set*>{}(key.right);
      return a ^ (b + 0x9e3779b9U + (a << 6U) + (a >> 2U));
    }
  };
  struct Entry { Result left; Result right; Result result; };

  [[nodiscard]] Result build(const std::vector<Input>& input, std::size_t begin,
                              std::size_t end) {
    if (end - begin == 1U) return {input[begin].shape, input[begin].shape->value().size()};
    const auto spatial_difference = input[begin].spatial_key ^ input[end - 1U].spatial_key;
    const auto difference = spatial_difference != 0U ? spatial_difference :
        input[begin].geometry_key ^ input[end - 1U].geometry_key;
    auto middle = begin + (end - begin) / 2U;
    if (difference != 0U) {
      const auto mask = std::bit_floor(difference);
      const auto it = std::partition_point(input.begin() + static_cast<std::ptrdiff_t>(begin),
          input.begin() + static_cast<std::ptrdiff_t>(end), [&](const Input& leaf) {
            return ((spatial_difference != 0U ? leaf.spatial_key : leaf.geometry_key) & mask) == 0U;
          });
      middle = static_cast<std::size_t>(it - input.begin());
    }
    const auto left = build(input, begin, middle);
    const auto right = build(input, middle, end);
    const Key key{left.shape.get(), right.shape.get()};
    {
      std::lock_guard lock(mutex_);
      if (const auto found = entries_.find(key); found != entries_.end()) {
        ++stats_.hits;
        return found->second.result;
      }
    }
    // Boost exposes lazy normalization through some const APIs. Operate on local copies only;
    // cached sets and child operands remain immutable even during concurrent group builds.
    using namespace boost::polygon::operators;
    Set combined = *left.shape;
    const Set right_copy = *right.shape;
    combined += right_copy;
    combined.clean();
    const auto units = left.shape->value().size() + right.shape->value().size() +
                       combined.value().size();
    Result result{std::make_shared<const Set>(std::move(combined)), units};
    std::lock_guard lock(mutex_);
    if (const auto found = entries_.find(key); found != entries_.end()) {
      ++stats_.hits;
      return found->second.result;
    }
    ++stats_.misses;
    if (capacity_ == 0U || units > maximum_retained_units_) return result;
    while (!order_.empty() && (entries_.size() >= capacity_ ||
                              retained_units_ + units > maximum_retained_units_)) {
      const auto oldest = entries_.find(order_.front());
      retained_units_ -= oldest->second.result.units;
      entries_.erase(oldest);
      order_.pop_front();
    }
    entries_.emplace(key, Entry{left, right, result});
    order_.push_back(key);
    retained_units_ += units;
    return result;
  }

  static constexpr std::size_t maximum_retained_units_ = 1'048'576U;
  std::size_t capacity_;
  std::size_t retained_units_{0};
  Stats stats_;
  std::unordered_map<Key, Entry, Hash> entries_;
  std::deque<Key> order_;
  mutable std::mutex mutex_;
};

}  // namespace flux::detail
