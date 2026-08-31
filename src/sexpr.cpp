#include "flux/io/sexpr.hpp"

#include <cctype>
#include <cstddef>
#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace flux::io {
namespace {

class Parser {
public:
  Parser(std::string_view input, SExpressionParseOptions options)
      : input_(input), options_(options) {}

  [[nodiscard]] SExpressionParseResult parse() {
    skip_ignored();
    if (at_end()) {
      error("empty_document", "expected one S-expression");
      return {std::nullopt, std::move(diagnostics_)};
    }

    auto root = parse_expression(0);
    skip_ignored();
    if (root && !at_end()) {
      error("trailing_expression", "only one root S-expression is allowed");
      root.reset();
    }
    if (!diagnostics_.empty()) {
      root.reset();
    }
    return {std::move(root), std::move(diagnostics_)};
  }

private:
  [[nodiscard]] bool at_end() const { return offset_ >= input_.size(); }

  [[nodiscard]] char current() const { return at_end() ? '\0' : input_[offset_]; }

  [[nodiscard]] SourceLocation location() const { return {offset_, line_, column_}; }

  char take() {
    const auto value = current();
    if (at_end()) {
      return value;
    }
    ++offset_;
    if (value == '\n') {
      ++line_;
      column_ = 1;
    } else {
      ++column_;
    }
    return value;
  }

  void skip_ignored() {
    while (!at_end()) {
      if (std::isspace(static_cast<unsigned char>(current())) != 0) {
        take();
        continue;
      }
      if (current() == ';') {
        while (!at_end() && take() != '\n') {
        }
        continue;
      }
      break;
    }
  }

  void error(std::string code, std::string message, SourceLocation where) {
    diagnostics_.push_back(ParseDiagnostic{where, std::move(code), std::move(message)});
  }

  void error(std::string code, std::string message) {
    error(std::move(code), std::move(message), location());
  }

  [[nodiscard]] bool account_node() {
    ++node_count_;
    if (node_count_ > options_.maximum_nodes) {
      error("node_limit", "S-expression node limit exceeded");
      return false;
    }
    return true;
  }

  [[nodiscard]] std::optional<SExpression> parse_expression(std::size_t depth) {
    skip_ignored();
    if (at_end()) {
      error("unexpected_eof", "expected an atom or list");
      return std::nullopt;
    }
    if (depth > options_.maximum_depth) {
      error("depth_limit", "S-expression nesting depth limit exceeded");
      return std::nullopt;
    }
    if (!account_node()) {
      return std::nullopt;
    }
    if (current() == '(') {
      return parse_list(depth);
    }
    if (current() == ')') {
      error("unexpected_close", "unexpected closing parenthesis");
      return std::nullopt;
    }
    return parse_atom();
  }

  [[nodiscard]] std::optional<SExpression> parse_list(std::size_t depth) {
    const auto start = location();
    take();
    SExpression result{SExpressionKind::list, start, 0, {}, false, {}};
    while (true) {
      skip_ignored();
      if (at_end()) {
        error("unterminated_list", "list is missing a closing parenthesis", start);
        return std::nullopt;
      }
      if (current() == ')') {
        take();
        result.end_offset = offset_;
        return result;
      }
      auto child = parse_expression(depth + 1U);
      if (!child) {
        return std::nullopt;
      }
      result.children.push_back(std::move(*child));
    }
  }

  [[nodiscard]] std::optional<SExpression> parse_atom() {
    const auto start = location();
    if (current() == '"') {
      take();
      std::string value;
      while (!at_end()) {
        const auto character = take();
        if (character == '"') {
          return SExpression{SExpressionKind::atom, start, offset_, std::move(value), true, {}};
        }
        if (character == '\\') {
          if (at_end()) {
            error("unterminated_escape", "quoted atom ends after an escape", start);
            return std::nullopt;
          }
          const auto escaped = take();
          switch (escaped) {
          case 'n':
            value.push_back('\n');
            break;
          case 'r':
            value.push_back('\r');
            break;
          case 't':
            value.push_back('\t');
            break;
          case '"':
          case '\\':
            value.push_back(escaped);
            break;
          default:
            value.push_back(escaped);
            break;
          }
        } else {
          value.push_back(character);
        }
      }
      error("unterminated_string", "quoted atom is missing a closing quote", start);
      return std::nullopt;
    }

    std::string value;
    while (!at_end() && std::isspace(static_cast<unsigned char>(current())) == 0 &&
           current() != '(' && current() != ')' && current() != ';') {
      value.push_back(take());
    }
    if (value.empty()) {
      error("empty_atom", "expected an atom", start);
      return std::nullopt;
    }
    return SExpression{SExpressionKind::atom, start, offset_, std::move(value), false, {}};
  }

  std::string_view input_;
  SExpressionParseOptions options_;
  std::size_t offset_{0};
  std::size_t line_{1};
  std::size_t column_{1};
  std::size_t node_count_{0};
  std::vector<ParseDiagnostic> diagnostics_;
};

} // namespace

SExpressionParseResult parse_sexpression(std::string_view input,
                                         const SExpressionParseOptions& options) {
  return Parser{input, options}.parse();
}

std::string_view head(const SExpression& expression) {
  if (!expression.is_list() || expression.children.empty() ||
      !expression.children.front().is_atom()) {
    return {};
  }
  return expression.children.front().value;
}

const SExpression* find_child(const SExpression& expression, std::string_view child_head) {
  if (!expression.is_list()) {
    return nullptr;
  }
  for (const auto& child : expression.children) {
    if (head(child) == child_head) {
      return &child;
    }
  }
  return nullptr;
}

std::vector<const SExpression*> find_children(const SExpression& expression,
                                              std::string_view child_head) {
  std::vector<const SExpression*> result;
  if (!expression.is_list()) {
    return result;
  }
  for (const auto& child : expression.children) {
    if (head(child) == child_head) {
      result.push_back(&child);
    }
  }
  return result;
}

const SExpression* child_at(const SExpression& expression, std::size_t index) {
  return expression.is_list() && index < expression.children.size() ? &expression.children[index]
                                                                    : nullptr;
}

} // namespace flux::io
