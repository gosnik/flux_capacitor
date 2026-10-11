#pragma once

#include <algorithm>
#include <bit>
#include <cstddef>
#include <cstdint>
#include <map>
#include <optional>
#include <tuple>
#include <vector>

namespace flux {

// Immutable, canonical set of flattened grid-cell keys.  Root zero is always the empty set;
// root one is the present leaf when the configured node budget can afford both sentinels.  Every
// non-leaf node is interned by (depth, left, right), so the root denotes the set rather than its
// insertion order.  This is intentionally private to the scaffold search implementation.
class ScaffoldViaHistory {
public:
  ScaffoldViaHistory(std::uint64_t maximum_key, std::size_t maximum_nodes)
      : maximum_key_(maximum_key), maximum_nodes_(maximum_nodes),
        depth_(static_cast<unsigned int>(std::bit_width(maximum_key))) {
    if (maximum_nodes_ >= 1U) nodes_.push_back({}); // empty sentinel, id 0
    if (maximum_nodes_ >= 2U) nodes_.push_back({}); // present leaf, id 1
    peak_nodes_ = nodes_.size();
  }

  [[nodiscard]] std::optional<std::size_t> insert(std::size_t root, std::uint64_t key) {
    if (key > maximum_key_ || root >= nodes_.size()) return std::nullopt;
    if (depth_ == 0U) return root == 1U ? root : present_leaf();
    const auto retained_nodes = nodes_.size();
    const auto result = insert_at(root, key, 0U);
    if (!result) {
      // A failed candidate must not consume the allowance needed by another history. No old
      // immutable node can refer to these newly allocated descendants, so rollback is exact.
      for (auto i = retained_nodes; i < nodes_.size(); ++i)
        interned_.erase(std::tuple{nodes_[i].depth, nodes_[i].left, nodes_[i].right});
      nodes_.resize(retained_nodes);
    }
    return result;
  }

  [[nodiscard]] bool contains(std::size_t root, std::uint64_t key) const {
    if (key > maximum_key_ || root >= nodes_.size()) return false;
    for (unsigned int depth = 0U; depth < depth_; ++depth) {
      if (root == 0U || root == 1U) return false;
      const auto& node = nodes_[root];
      root = bit_at(key, depth) == 0U ? node.left : node.right;
    }
    return root == 1U;
  }

  [[nodiscard]] std::size_t node_count() const { return nodes_.size(); }
  [[nodiscard]] std::size_t peak_node_count() const { return peak_nodes_; }

private:
  struct Node {
    std::size_t left{0U};
    std::size_t right{0U};
    unsigned int depth{0U};
  };

  [[nodiscard]] std::optional<std::size_t> present_leaf() const {
    if (nodes_.size() < 2U) return std::nullopt;
    return std::size_t{1U};
  }

  [[nodiscard]] unsigned int bit_at(std::uint64_t key, unsigned int depth) const {
    return static_cast<unsigned int>((key >> (depth_ - depth - 1U)) & 1U);
  }

  [[nodiscard]] std::optional<std::size_t> intern(unsigned int depth, std::size_t left,
                                                   std::size_t right) {
    const auto tuple = std::tuple{depth, left, right};
    if (const auto found = interned_.find(tuple); found != interned_.end()) return found->second;
    if (nodes_.size() >= maximum_nodes_) return std::nullopt;
    const auto id = nodes_.size();
    nodes_.push_back({left, right, depth});
    peak_nodes_ = std::max(peak_nodes_, nodes_.size());
    interned_.emplace(tuple, id);
    return id;
  }

  [[nodiscard]] std::optional<std::size_t> insert_at(std::size_t root, std::uint64_t key,
                                                      unsigned int depth) {
    if (depth == depth_) return present_leaf();
    if (root == 1U) return std::nullopt; // a leaf is valid only at fixed trie depth.
    const auto left = root == 0U ? 0U : nodes_[root].left;
    const auto right = root == 0U ? 0U : nodes_[root].right;
    const auto choose_left = bit_at(key, depth) == 0U;
    const auto child = insert_at(choose_left ? left : right, key, depth + 1U);
    if (!child) return std::nullopt;
    if (choose_left && *child == left) return root;
    if (!choose_left && *child == right) return root;
    return intern(depth, choose_left ? *child : left, choose_left ? right : *child);
  }

  std::uint64_t maximum_key_;
  std::size_t maximum_nodes_;
  std::size_t peak_nodes_{0U};
  unsigned int depth_;
  std::vector<Node> nodes_;
  std::map<std::tuple<unsigned int, std::size_t, std::size_t>, std::size_t> interned_;
};

} // namespace flux
