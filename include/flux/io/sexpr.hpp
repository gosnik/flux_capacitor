#pragma once

#include <cstddef>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace flux::io {

struct SourceLocation {
  std::size_t offset{0};
  std::size_t line{1};
  std::size_t column{1};
};

enum class SExpressionKind { atom, list };

struct SExpression {
  SExpressionKind kind{SExpressionKind::atom};
  SourceLocation location;
  std::size_t end_offset{0};
  std::string value;
  bool quoted{false};
  std::vector<SExpression> children;

  [[nodiscard]] bool is_atom() const { return kind == SExpressionKind::atom; }
  [[nodiscard]] bool is_list() const { return kind == SExpressionKind::list; }
  [[nodiscard]] std::size_t byte_size() const { return end_offset - location.offset; }
};

struct ParseDiagnostic {
  SourceLocation location;
  std::string code;
  std::string message;
};

struct SExpressionParseOptions {
  std::size_t maximum_depth{512};
  std::size_t maximum_nodes{5'000'000};
};

struct SExpressionParseResult {
  std::optional<SExpression> root;
  std::vector<ParseDiagnostic> diagnostics;

  [[nodiscard]] bool ok() const { return root.has_value() && diagnostics.empty(); }
};

[[nodiscard]] SExpressionParseResult parse_sexpression(std::string_view input,
                                                       const SExpressionParseOptions& options = {});

[[nodiscard]] std::string_view head(const SExpression& expression);
[[nodiscard]] const SExpression* find_child(const SExpression& expression,
                                            std::string_view child_head);
[[nodiscard]] std::vector<const SExpression*> find_children(const SExpression& expression,
                                                            std::string_view child_head);
[[nodiscard]] const SExpression* child_at(const SExpression& expression, std::size_t index);

} // namespace flux::io
