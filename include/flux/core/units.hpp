#pragma once

#include <cmath>
#include <compare>
#include <cstdint>
#include <limits>
#include <stdexcept>

namespace flux {

class Nanometres {
public:
  constexpr Nanometres() = default;
  explicit constexpr Nanometres(std::int64_t value) : value_(value) {}

  [[nodiscard]] static Nanometres from_mm(double value) {
    constexpr double nanometres_per_mm = 1'000'000.0;
    const double scaled = value * nanometres_per_mm;
    if (!std::isfinite(scaled) ||
        scaled > static_cast<double>(std::numeric_limits<std::int64_t>::max()) ||
        scaled < static_cast<double>(std::numeric_limits<std::int64_t>::min())) {
      throw std::out_of_range("millimetre value cannot be represented in nanometres");
    }
    return Nanometres{static_cast<std::int64_t>(std::llround(scaled))};
  }

  [[nodiscard]] constexpr std::int64_t value() const { return value_; }
  [[nodiscard]] constexpr double to_mm() const { return static_cast<double>(value_) / 1'000'000.0; }

  constexpr auto operator<=>(const Nanometres&) const = default;

  constexpr Nanometres& operator+=(Nanometres other) {
    value_ += other.value_;
    return *this;
  }

  constexpr Nanometres& operator-=(Nanometres other) {
    value_ -= other.value_;
    return *this;
  }

private:
  std::int64_t value_{0};
};

[[nodiscard]] constexpr Nanometres operator+(Nanometres lhs, Nanometres rhs) {
  lhs += rhs;
  return lhs;
}

[[nodiscard]] constexpr Nanometres operator-(Nanometres lhs, Nanometres rhs) {
  lhs -= rhs;
  return lhs;
}

[[nodiscard]] constexpr Nanometres operator-(Nanometres value) {
  return Nanometres{-value.value()};
}

class Microdegrees {
public:
  constexpr Microdegrees() = default;
  explicit constexpr Microdegrees(std::int64_t value) : value_(normalize(value)) {}

  [[nodiscard]] static Microdegrees from_degrees(double value) {
    if (!std::isfinite(value)) {
      throw std::out_of_range("angle must be finite");
    }
    return Microdegrees{static_cast<std::int64_t>(std::llround(value * 1'000'000.0))};
  }

  [[nodiscard]] constexpr std::int64_t value() const { return value_; }
  [[nodiscard]] constexpr double to_degrees() const {
    return static_cast<double>(value_) / 1'000'000.0;
  }

  constexpr auto operator<=>(const Microdegrees&) const = default;

private:
  static constexpr std::int64_t full_turn = 360'000'000;

  [[nodiscard]] static constexpr std::int64_t normalize(std::int64_t value) {
    value %= full_turn;
    return value < 0 ? value + full_turn : value;
  }

  std::int64_t value_{0};
};

namespace literals {

[[nodiscard]] constexpr Nanometres operator""_nm(unsigned long long value) {
  return Nanometres{static_cast<std::int64_t>(value)};
}

} // namespace literals

} // namespace flux
