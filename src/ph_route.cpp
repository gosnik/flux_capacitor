#include "flux/core/routing_compute.hpp"
#include "flux/core/spline_route.hpp"

#include "flux/core/geometry.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <complex>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <mutex>
#include <numeric>
#include <optional>
#include <stop_token>
#include <thread>
#include <utility>
#include <vector>

namespace flux {
namespace {

using Complex = std::complex<long double>;
constexpr long double pi = 3.141592653589793238462643383279502884L;

class BoundedWorkerPool {
public:
  explicit BoundedWorkerPool(std::size_t workers) : worker_count_(workers > 1U ? workers : 0U) {
    workers_.reserve(worker_count_);
    for (std::size_t worker = 0U; worker < worker_count_; ++worker)
      workers_.emplace_back([this](std::stop_token stop) { work(stop); });
  }
  ~BoundedWorkerPool() {
    for (auto& worker : workers_)
      worker.request_stop();
    wake_.notify_all();
  }
  BoundedWorkerPool(const BoundedWorkerPool&) = delete;
  BoundedWorkerPool& operator=(const BoundedWorkerPool&) = delete;

  void run(std::size_t count, std::function<void(std::size_t)> task) {
    if (count == 0U)
      return;
    if (workers_.empty()) {
      for (std::size_t index = 0U; index < count; ++index)
        task(index);
      return;
    }
    {
      std::lock_guard lock(mutex_);
      task_ = std::move(task);
      task_count_ = count;
      next_task_ = 0U;
      completed_workers_ = 0U;
      ++generation_;
    }
    wake_.notify_all();
    std::unique_lock lock(mutex_);
    complete_.wait(lock, [&] { return completed_workers_ == worker_count_; });
    task_ = {};
  }

private:
  void work(std::stop_token stop) {
    std::size_t observed_generation = 0U;
    while (!stop.stop_requested()) {
      std::function<void(std::size_t)> task;
      std::size_t count = 0U;
      {
        std::unique_lock lock(mutex_);
        wake_.wait(lock,
                   [&] { return stop.stop_requested() || generation_ != observed_generation; });
        if (stop.stop_requested())
          return;
        observed_generation = generation_;
        task = task_;
        count = task_count_;
      }
      while (true) {
        std::size_t index = 0U;
        {
          std::lock_guard lock(mutex_);
          if (next_task_ >= count)
            break;
          index = next_task_++;
        }
        task(index);
      }
      {
        std::lock_guard lock(mutex_);
        if (++completed_workers_ == worker_count_)
          complete_.notify_one();
      }
    }
  }

  const std::size_t worker_count_{0U};
  std::vector<std::jthread> workers_;
  std::mutex mutex_;
  std::condition_variable wake_;
  std::condition_variable complete_;
  std::function<void(std::size_t)> task_;
  std::size_t task_count_{0U};
  std::size_t next_task_{0U};
  std::size_t completed_workers_{0U};
  std::size_t generation_{0U};
};

[[nodiscard]] Complex complex(SplineVector point) { return {point.x, point.y}; }
[[nodiscard]] SplineVector vector(Complex value) { return {value.real(), value.imag()}; }
[[nodiscard]] SplineVector add(SplineVector a, SplineVector b) { return {a.x + b.x, a.y + b.y}; }
[[nodiscard]] SplineVector subtract(SplineVector a, SplineVector b) {
  return {a.x - b.x, a.y - b.y};
}
[[nodiscard]] SplineVector scale(SplineVector value, long double factor) {
  return {value.x * factor, value.y * factor};
}
[[nodiscard]] long double dot(SplineVector a, SplineVector b) { return a.x * b.x + a.y * b.y; }
[[nodiscard]] long double cross(SplineVector a, SplineVector b) { return a.x * b.y - a.y * b.x; }
[[nodiscard]] long double norm(SplineVector value) { return std::hypotl(value.x, value.y); }
[[nodiscard]] SplineVector normalized(SplineVector value) {
  const auto length = norm(value);
  return length > 0.0L ? scale(value, 1.0L / length) : SplineVector{};
}
[[nodiscard]] SplineVector lerp(SplineVector a, SplineVector b, long double t) {
  return add(scale(a, 1.0L - t), scale(b, t));
}

[[nodiscard]] long double binomial(std::size_t n, std::size_t k) {
  if (k > n)
    return 0.0L;
  k = std::min(k, n - k);
  long double result = 1.0L;
  for (std::size_t i = 1U; i <= k; ++i) {
    result *= static_cast<long double>(n - k + i) / static_cast<long double>(i);
  }
  return result;
}

[[nodiscard]] long double bernstein(std::size_t degree, std::size_t index, long double t) {
  if (index > degree)
    return 0.0L;
  t = std::clamp(t, 0.0L, 1.0L);
  return binomial(degree, index) * std::pow(t, static_cast<long double>(index)) *
         std::pow(1.0L - t, static_cast<long double>(degree - index));
}

template <typename T, std::size_t N>
[[nodiscard]] T evaluate_bezier(const std::array<T, N>& controls, long double parameter) {
  auto work = controls;
  const auto t = std::clamp(parameter, 0.0L, 1.0L);
  for (std::size_t level = 1U; level < N; ++level) {
    for (std::size_t i = 0U; i + level < N; ++i) {
      work[i] = work[i] * (1.0L - t) + work[i + 1U] * t;
    }
  }
  return work[0];
}

template <std::size_t N>
[[nodiscard]] std::array<long double, N>
bernstein_to_power_array(const std::array<long double, N>& coefficients) {
  std::array<long double, N> result{};
  constexpr auto degree = N - 1U;
  for (std::size_t i = 0U; i <= degree; ++i) {
    for (std::size_t j = 0U; j <= degree - i; ++j) {
      const auto k = i + j;
      const auto sign = (j & 1U) == 0U ? 1.0L : -1.0L;
      result[k] += coefficients[i] * binomial(degree, i) * binomial(degree - i, j) * sign;
    }
  }
  return result;
}

template <std::size_t N>
[[nodiscard]] std::array<SplineVector, N>
bernstein_to_power_array(const std::array<SplineVector, N>& coefficients) {
  std::array<SplineVector, N> result{};
  constexpr auto degree = N - 1U;
  for (std::size_t i = 0U; i <= degree; ++i) {
    for (std::size_t j = 0U; j <= degree - i; ++j) {
      const auto k = i + j;
      const auto sign = (j & 1U) == 0U ? 1.0L : -1.0L;
      const auto factor = binomial(degree, i) * binomial(degree - i, j) * sign;
      result[k].x += coefficients[i].x * factor;
      result[k].y += coefficients[i].y * factor;
    }
  }
  return result;
}

template <std::size_t N>
[[nodiscard]] long double evaluate_power_array(const std::array<long double, N>& coefficients,
                                               long double parameter) {
  long double result = 0.0L;
  for (auto it = coefficients.rbegin(); it != coefficients.rend(); ++it)
    result = result * parameter + *it;
  return result;
}

template <std::size_t N>
[[nodiscard]] SplineVector evaluate_power_array(const std::array<SplineVector, N>& coefficients,
                                                long double parameter) {
  SplineVector result;
  for (auto it = coefficients.rbegin(); it != coefficients.rend(); ++it) {
    result.x = result.x * parameter + it->x;
    result.y = result.y * parameter + it->y;
  }
  return result;
}

template <std::size_t N>
[[nodiscard]] SplineVector evaluate_bezier(const std::array<SplineVector, N>& controls,
                                           long double parameter) {
  auto work = controls;
  const auto t = std::clamp(parameter, 0.0L, 1.0L);
  for (std::size_t level = 1U; level < N; ++level) {
    for (std::size_t i = 0U; i + level < N; ++i)
      work[i] = lerp(work[i], work[i + 1U], t);
  }
  return work[0];
}

template <typename T>
[[nodiscard]] std::vector<T> bernstein_product(const std::vector<T>& first,
                                               const std::vector<long double>& second) {
  if (first.empty() || second.empty())
    return {};
  const auto m = first.size() - 1U;
  const auto n = second.size() - 1U;
  std::vector<T> result(m + n + 1U, T{});
  for (std::size_t k = 0U; k <= m + n; ++k) {
    for (std::size_t i = k > n ? k - n : 0U; i <= std::min(m, k); ++i) {
      const auto j = k - i;
      const auto weight = binomial(m, i) * binomial(n, j) / binomial(m + n, k);
      result[k] += first[i] * (second[j] * weight);
    }
  }
  return result;
}

[[nodiscard]] long double evaluate_power(const std::vector<long double>& coefficients,
                                         long double x) {
  long double value = 0.0L;
  for (auto it = coefficients.rbegin(); it != coefficients.rend(); ++it)
    value = value * x + *it;
  return value;
}

[[nodiscard]] std::vector<long double> derivative_power(std::vector<long double> polynomial) {
  if (polynomial.size() <= 1U)
    return {0.0L};
  std::vector<long double> result(polynomial.size() - 1U);
  for (std::size_t i = 1U; i < polynomial.size(); ++i) {
    result[i - 1U] = polynomial[i] * static_cast<long double>(i);
  }
  return result;
}

[[nodiscard]] std::vector<long double> multiply_power(const std::vector<long double>& first,
                                                      const std::vector<long double>& second) {
  std::vector<long double> result(first.size() + second.size() - 1U, 0.0L);
  for (std::size_t i = 0U; i < first.size(); ++i)
    for (std::size_t j = 0U; j < second.size(); ++j)
      result[i + j] += first[i] * second[j];
  return result;
}

void trim_polynomial(std::vector<long double>& polynomial) {
  long double scale_value = 0.0L;
  for (const auto value : polynomial)
    scale_value = std::max(scale_value, std::abs(value));
  const auto tolerance = std::max(1.0e-24L, scale_value * 1.0e-16L);
  while (polynomial.size() > 1U && std::abs(polynomial.back()) <= tolerance)
    polynomial.pop_back();
}

[[nodiscard]] std::vector<long double> real_roots_unit(std::vector<long double> polynomial) {
  trim_polynomial(polynomial);
  if (polynomial.size() <= 1U)
    return {};
  if (polynomial.size() == 2U) {
    const auto root = -polynomial[0] / polynomial[1];
    return root >= -1.0e-12L && root <= 1.0L + 1.0e-12L
               ? std::vector<long double>{std::clamp(root, 0.0L, 1.0L)}
               : std::vector<long double>{};
  }
  auto critical = real_roots_unit(derivative_power(polynomial));
  critical.insert(critical.begin(), 0.0L);
  critical.push_back(1.0L);
  std::sort(critical.begin(), critical.end());
  critical.erase(std::unique(critical.begin(), critical.end(),
                             [](auto a, auto b) { return std::abs(a - b) < 1.0e-12L; }),
                 critical.end());
  std::vector<long double> roots;
  const auto coefficient_scale =
      std::accumulate(polynomial.begin(), polynomial.end(), 0.0L,
                      [](auto sum, auto value) { return sum + std::abs(value); });
  const auto zero_tolerance = std::max(1.0e-20L, coefficient_scale * 1.0e-13L);
  for (const auto x : critical) {
    if (std::abs(evaluate_power(polynomial, x)) <= zero_tolerance)
      roots.push_back(x);
  }
  for (std::size_t interval = 1U; interval < critical.size(); ++interval) {
    auto low = critical[interval - 1U];
    auto high = critical[interval];
    auto low_value = evaluate_power(polynomial, low);
    const auto high_value = evaluate_power(polynomial, high);
    if (low_value == 0.0L || high_value == 0.0L ||
        std::signbit(low_value) == std::signbit(high_value))
      continue;
    for (std::size_t iteration = 0U; iteration < 80U; ++iteration) {
      const auto middle = (low + high) * 0.5L;
      const auto middle_value = evaluate_power(polynomial, middle);
      if (std::signbit(low_value) == std::signbit(middle_value)) {
        low = middle;
        low_value = middle_value;
      } else {
        high = middle;
      }
    }
    roots.push_back((low + high) * 0.5L);
  }
  std::sort(roots.begin(), roots.end());
  roots.erase(std::unique(roots.begin(), roots.end(),
                          [](auto a, auto b) { return std::abs(a - b) < 1.0e-9L; }),
              roots.end());
  return roots;
}

[[nodiscard]] bool forward_collinear(Point first, Point middle, Point last) {
  const auto a = SplineVector{static_cast<long double>(middle.x.value() - first.x.value()),
                              static_cast<long double>(middle.y.value() - first.y.value())};
  const auto b = SplineVector{static_cast<long double>(last.x.value() - middle.x.value()),
                              static_cast<long double>(last.y.value() - middle.y.value())};
  const auto chord = add(a, b);
  // Funnel portals and integer clipping can leave a one-nanometre lateral
  // residue on an otherwise straight run. Preserving that residue as a tiny
  // Hermite span makes w(t) almost vanish and invents a high-curvature cusp.
  // The two-nanometre guard is the same conservative integer-rounding scale
  // used by free-space geometry; it never removes a meaningful obstacle-side
  // or topology decision.
  constexpr long double quantization_guard_nm = 2.0L;
  const auto chord_length = norm(chord);
  return chord_length > 0.0L && std::abs(cross(chord, a)) <= quantization_guard_nm * chord_length &&
         dot(a, b) >= 0.0L;
}

[[nodiscard]] std::vector<Point> canonical_polyline(const std::vector<Point>& input) {
  std::vector<Point> result;
  result.reserve(input.size());
  for (const auto point : input) {
    if (!result.empty() && result.back() == point)
      continue;
    if (result.size() >= 2U &&
        forward_collinear(result[result.size() - 2U], result.back(), point)) {
      result.back() = point;
    } else {
      result.push_back(point);
    }
  }
  return result;
}

[[nodiscard]] Point quantize(SplineVector point) {
  constexpr auto low = static_cast<long double>(std::numeric_limits<std::int64_t>::min());
  constexpr auto high = static_cast<long double>(std::numeric_limits<std::int64_t>::max());
  return Point{Nanometres{static_cast<std::int64_t>(std::llround(std::clamp(point.x, low, high)))},
               Nanometres{static_cast<std::int64_t>(std::llround(std::clamp(point.y, low, high)))}};
}

[[nodiscard]] std::array<long double, 5U>
speed_coefficients(const std::array<SplineVector, 3U>& preimage) {
  const auto w0 = complex(preimage[0]);
  const auto w1 = complex(preimage[1]);
  const auto w2 = complex(preimage[2]);
  return {std::norm(w0), std::real(std::conj(w0) * w1),
          (2.0L * std::norm(w1) + std::real(std::conj(w0) * w2)) / 3.0L,
          std::real(std::conj(w1) * w2), std::norm(w2)};
}

[[nodiscard]] QuinticPHSegment build_segment(SplineVector start, const std::array<Complex, 3U>& w,
                                             std::size_t branch) {
  QuinticPHSegment segment;
  for (std::size_t i = 0U; i < 3U; ++i)
    segment.preimage[i] = vector(w[i]);
  segment.interpolation_branch = branch;
  segment.bezier[0] = start;
  const std::array<Complex, 5U> derivative{w[0] * w[0], w[0] * w[1],
                                           (w[0] * w[2] + 2.0L * w[1] * w[1]) / 3.0L, w[1] * w[2],
                                           w[2] * w[2]};
  for (std::size_t i = 0U; i < 5U; ++i) {
    segment.bezier[i + 1U] = add(segment.bezier[i], scale(vector(derivative[i]), 0.2L));
  }
  segment.speed = speed_coefficients(segment.preimage);
  segment.arc_length[0] = 0.0L;
  for (std::size_t i = 0U; i < 5U; ++i) {
    segment.arc_length[i + 1U] = segment.arc_length[i] + segment.speed[i] / 5.0L;
  }
  segment.preimage_power = bernstein_to_power_array(segment.preimage);
  std::array<SplineVector, 5U> hodograph_bernstein{};
  for (std::size_t i = 0U; i < derivative.size(); ++i)
    hodograph_bernstein[i] = vector(derivative[i]);
  segment.hodograph_power = bernstein_to_power_array(hodograph_bernstein);
  segment.position_power = bernstein_to_power_array(segment.bezier);
  segment.speed_power = bernstein_to_power_array(segment.speed);
  segment.arc_length_power = bernstein_to_power_array(segment.arc_length);
  const std::array<long double, 3U> curvature_numerator_bernstein{
      4.0L * std::imag(std::conj(w[0]) * w[1]), 2.0L * std::imag(std::conj(w[0]) * w[2]),
      4.0L * std::imag(std::conj(w[1]) * w[2])};
  segment.curvature_numerator_power = bernstein_to_power_array(curvature_numerator_bernstein);

  // Cache exact axis-aligned position extrema from roots of the quartic
  // hodograph components. No quintic sampling is needed for broadphase bounds.
  std::vector<long double> position_extrema{0.0L, 1.0L};
  for (const auto x_axis : {true, false}) {
    std::vector<long double> polynomial;
    polynomial.reserve(segment.hodograph_power.size());
    for (const auto coefficient : segment.hodograph_power)
      polynomial.push_back(x_axis ? coefficient.x : coefficient.y);
    const auto roots = real_roots_unit(std::move(polynomial));
    position_extrema.insert(position_extrema.end(), roots.begin(), roots.end());
  }
  segment.bounds_minimum = {std::numeric_limits<long double>::infinity(),
                            std::numeric_limits<long double>::infinity()};
  segment.bounds_maximum = {-std::numeric_limits<long double>::infinity(),
                            -std::numeric_limits<long double>::infinity()};
  for (const auto parameter : position_extrema) {
    const auto point = evaluate_power_array(segment.position_power, parameter);
    segment.bounds_minimum.x = std::min(segment.bounds_minimum.x, point.x);
    segment.bounds_minimum.y = std::min(segment.bounds_minimum.y, point.y);
    segment.bounds_maximum.x = std::max(segment.bounds_maximum.x, point.x);
    segment.bounds_maximum.y = std::max(segment.bounds_maximum.y, point.y);
  }

  const auto append_cusp = [&](Complex candidate) {
    if (segment.cusp_count >= segment.cusp_parameters.size())
      return;
    const auto scale_value = std::max(1.0L, std::abs(candidate.real()));
    if (std::abs(candidate.imag()) <= scale_value * 1.0e-12L && candidate.real() >= -1.0e-12L &&
        candidate.real() <= 1.0L + 1.0e-12L) {
      const auto parameter = std::clamp(candidate.real(), 0.0L, 1.0L);
      if (segment.cusp_count == 0U ||
          std::abs(segment.cusp_parameters[segment.cusp_count - 1U] - parameter) > 1.0e-10L)
        segment.cusp_parameters[segment.cusp_count++] = parameter;
    }
  };
  const auto preimage_a = complex(segment.preimage_power[2]);
  const auto preimage_b = complex(segment.preimage_power[1]);
  const auto preimage_c = complex(segment.preimage_power[0]);
  if (std::abs(preimage_a) > 1.0e-24L) {
    const auto root = std::sqrt(preimage_b * preimage_b - 4.0L * preimage_a * preimage_c);
    auto first = (-preimage_b - root) / (2.0L * preimage_a);
    auto second = (-preimage_b + root) / (2.0L * preimage_a);
    if (first.real() > second.real())
      std::swap(first, second);
    append_cusp(first);
    append_cusp(second);
  } else if (std::abs(preimage_b) > 1.0e-24L) {
    append_cusp(-preimage_c / preimage_b);
  } else if (std::abs(preimage_c) <= 1.0e-24L) {
    segment.cusp_parameters[segment.cusp_count++] = 0.0L;
  }
  segment.regular = segment.cusp_count == 0U;

  const std::vector<long double> curvature_numerator(segment.curvature_numerator_power.begin(),
                                                     segment.curvature_numerator_power.end());
  for (const auto parameter : real_roots_unit(curvature_numerator)) {
    if (parameter > 1.0e-10L && parameter < 1.0L - 1.0e-10L &&
        segment.inflection_count < segment.inflection_parameters.size())
      segment.inflection_parameters[segment.inflection_count++] = parameter;
  }
  const std::vector<long double> sigma(segment.speed_power.begin(), segment.speed_power.end());
  const auto hp = derivative_power(curvature_numerator);
  const auto sigma_p = derivative_power(sigma);
  auto curvature_stationary = multiply_power(sigma, hp);
  auto second_term = multiply_power(curvature_numerator, sigma_p);
  if (curvature_stationary.size() < second_term.size())
    curvature_stationary.resize(second_term.size(), 0.0L);
  if (second_term.size() < curvature_stationary.size())
    second_term.resize(curvature_stationary.size(), 0.0L);
  for (std::size_t i = 0U; i < curvature_stationary.size(); ++i)
    curvature_stationary[i] -= 2.0L * second_term[i];
  std::vector<long double> curvature_parameters{0.0L, 1.0L};
  const auto curvature_roots = real_roots_unit(std::move(curvature_stationary));
  curvature_parameters.insert(curvature_parameters.end(), curvature_roots.begin(),
                              curvature_roots.end());
  std::sort(curvature_parameters.begin(), curvature_parameters.end());
  curvature_parameters.erase(
      std::unique(curvature_parameters.begin(), curvature_parameters.end(),
                  [](auto first, auto second) { return std::abs(first - second) < 1.0e-9L; }),
      curvature_parameters.end());
  for (const auto parameter : curvature_parameters) {
    if (segment.curvature_extremum_count >= segment.curvature_extremum_parameters.size())
      break;
    segment.curvature_extremum_parameters[segment.curvature_extremum_count++] = parameter;
  }
  return segment;
}

struct PHKinematics {
  Complex preimage;
  Complex preimage_derivative;
  Complex tangent;
  Complex second_derivative;
  long double speed{0.0L};
  long double curvature{0.0L};
};

[[nodiscard]] PHKinematics evaluate_preimage_kinematics(const QuinticPHSegment& segment,
                                                        long double parameter) {
  const auto t = std::clamp(parameter, 0.0L, 1.0L);
  const auto w = complex(evaluate_power_array(segment.preimage_power, t));
  const auto wp =
      complex(segment.preimage_power[1]) + 2.0L * t * complex(segment.preimage_power[2]);
  const auto tangent = w * w;
  const auto second = 2.0L * w * wp;
  const auto speed = std::norm(w);
  const auto curvature =
      speed > 1.0e-30L ? 2.0L * std::imag(std::conj(w) * wp) / (speed * speed) : 0.0L;
  return {w, wp, tangent, second, speed, curvature};
}

[[nodiscard]] SplineDifferential evaluate_segment_differential(const QuinticPHSegment& segment,
                                                               long double parameter) {
  const auto t = std::clamp(parameter, 0.0L, 1.0L);
  const auto kinematics = evaluate_preimage_kinematics(segment, t);
  return SplineDifferential{
      evaluate_power_array(segment.position_power, t), vector(kinematics.tangent),
      vector(kinematics.second_derivative), kinematics.curvature,
      std::abs(kinematics.curvature) > 1.0e-30L ? 1.0L / std::abs(kinematics.curvature)
                                                : std::numeric_limits<long double>::infinity()};
}

[[nodiscard]] std::pair<std::size_t, long double> locate_segment(const QuinticPHRoute& route,
                                                                 long double parameter) {
  if (route.segments.empty())
    return {0U, 0.0L};
  const auto t = std::clamp(parameter, 0.0L, 1.0L);
  auto found = std::lower_bound(
      route.segments.begin(), route.segments.end(), t,
      [](const auto& segment, auto value) { return segment.route_parameter_end < value; });
  if (found == route.segments.end())
    found = std::prev(route.segments.end());
  const auto span = found->route_parameter_end - found->route_parameter_start;
  return {static_cast<std::size_t>(std::distance(route.segments.begin(), found)),
          span > 0.0L ? std::clamp((t - found->route_parameter_start) / span, 0.0L, 1.0L) : 0.0L};
}

[[nodiscard]] std::size_t segment_self_intersections(const QuinticPHSegment& segment) {
  constexpr std::size_t samples = 48U;
  std::array<SplineVector, samples + 1U> points;
  for (std::size_t i = 0U; i <= samples; ++i) {
    points[i] = evaluate_power_array(segment.position_power, static_cast<long double>(i) /
                                                                 static_cast<long double>(samples));
  }
  std::size_t count = 0U;
  for (std::size_t i = 1U; i <= samples; ++i) {
    for (std::size_t j = i + 2U; j <= samples; ++j) {
      if (i == 1U && j == samples)
        continue;
      const auto a = points[i - 1U];
      const auto b = points[i];
      const auto c = points[j - 1U];
      const auto d = points[j];
      const auto ab_c = cross(subtract(b, a), subtract(c, a));
      const auto ab_d = cross(subtract(b, a), subtract(d, a));
      const auto cd_a = cross(subtract(d, c), subtract(a, c));
      const auto cd_b = cross(subtract(d, c), subtract(b, c));
      if (((ab_c > 0.0L && ab_d < 0.0L) || (ab_c < 0.0L && ab_d > 0.0L)) &&
          ((cd_a > 0.0L && cd_b < 0.0L) || (cd_a < 0.0L && cd_b > 0.0L)))
        ++count;
    }
  }
  return count;
}

[[nodiscard]] long double absolute_rotation(const QuinticPHSegment& segment) {
  constexpr std::size_t samples = 64U;
  auto prior = vector(evaluate_preimage_kinematics(segment, 0.0L).tangent);
  long double result = 0.0L;
  for (std::size_t i = 1U; i <= samples; ++i) {
    const auto current =
        vector(evaluate_preimage_kinematics(segment, static_cast<long double>(i) /
                                                         static_cast<long double>(samples))
                   .tangent);
    result += std::abs(std::atan2(cross(prior, current), dot(prior, current)));
    prior = current;
  }
  return result;
}

[[nodiscard]] long double segment_branch_score(const QuinticPHSegment& segment,
                                               SplineVector chord) {
  const auto chord_length = std::max(1.0L, norm(chord));
  const auto length = quintic_ph_arc_length(segment);
  const auto inflections = quintic_ph_inflections(segment).size();
  const auto loops = segment_self_intersections(segment);
  long double peak_curvature = 0.0L;
  for (std::size_t i = 0U; i < segment.curvature_extremum_count; ++i) {
    peak_curvature = std::max(
        peak_curvature,
        std::abs(evaluate_preimage_kinematics(segment, segment.curvature_extremum_parameters[i])
                     .curvature));
  }
  long double hull_excess = 0.0L;
  const auto unit = normalized(chord);
  for (const auto point : segment.bezier) {
    const auto relative = subtract(point, segment.bezier.front());
    const auto station = dot(relative, unit);
    hull_excess += std::max(0.0L, -station) + std::max(0.0L, station - chord_length);
  }
  // A zero of the quadratic preimage makes r'(t)=w(t)^2 vanish exactly. Such a cusp is a hard
  // trajectory singularity, not a short-path tradeoff; it must dominate every aesthetic branch
  // term. Keep a finite penalty only so interpolation can still return diagnostic geometry when
  // every algebraic branch is singular.
  return static_cast<long double>(segment.cusp_count) * 1.0e18L +
         static_cast<long double>(loops) * 1.0e12L +
         static_cast<long double>(inflections) * 1.0e8L +
         peak_curvature * chord_length * chord_length * 10.0L +
         absolute_rotation(segment) * chord_length + std::max(0.0L, length - chord_length) * 2.0L +
         hull_excess * 10.0L;
}

[[nodiscard]] bool certify_route_join_continuity(QuinticPHRoute& route) {
  if (route.segments.size() + 1U != route.control_points.size())
    return false;
  route.tangent_continuous = true;
  route.curvature_continuous = true;
  for (std::size_t i = 1U; i < route.segments.size(); ++i) {
    auto left = evaluate_segment_differential(route.segments[i - 1U], 1.0L);
    auto right = evaluate_segment_differential(route.segments[i], 0.0L);
    const auto left_span =
        route.segments[i - 1U].route_parameter_end - route.segments[i - 1U].route_parameter_start;
    const auto right_span =
        route.segments[i].route_parameter_end - route.segments[i].route_parameter_start;
    left.tangent = scale(left.tangent, 1.0L / left_span);
    right.tangent = scale(right.tangent, 1.0L / right_span);
    left.second_derivative = scale(left.second_derivative, 1.0L / (left_span * left_span));
    right.second_derivative = scale(right.second_derivative, 1.0L / (right_span * right_span));
    const auto tangent_scale_value = std::max({1.0L, norm(left.tangent), norm(right.tangent)});
    route.tangent_continuous =
        route.tangent_continuous &&
        norm(subtract(left.tangent, right.tangent)) <= tangent_scale_value * 1.0e-9L;
    route.curvature_continuous =
        route.curvature_continuous &&
        norm(subtract(left.second_derivative, right.second_derivative)) <= 10.0L;
  }
  return route.tangent_continuous;
}

[[nodiscard]] bool build_route_segments_from_derivatives(QuinticPHRoute& route) {
  if (route.control_points.size() < 2U ||
      route.node_derivatives.size() != route.control_points.size() ||
      route.node_parameters.size() != route.control_points.size())
    return false;
  route.segments.clear();
  route.segments.reserve(route.control_points.size() - 1U);
  for (std::size_t i = 1U; i < route.control_points.size(); ++i) {
    const auto parameter_span = route.node_parameters[i] - route.node_parameters[i - 1U];
    if (!(parameter_span > 0.0L))
      return false;
    auto segment =
        interpolate_quintic_ph_segment(route.control_points[i - 1U], route.control_points[i],
                                       scale(route.node_derivatives[i - 1U], parameter_span),
                                       scale(route.node_derivatives[i], parameter_span));
    if (!segment) {
      route.segments.clear();
      route.tangent_continuous = false;
      return false;
    }
    segment->route_parameter_start = route.node_parameters[i - 1U];
    segment->route_parameter_end = route.node_parameters[i];
    route.segments.push_back(*segment);
  }
  return certify_route_join_continuity(route);
}

[[nodiscard]] bool convex_polygon(const Polygon& polygon) {
  if (polygon.vertices.size() < 3U || !geometry::is_simple_polygon(polygon))
    return false;
  int sign = 0;
  for (std::size_t i = 0U; i < polygon.vertices.size(); ++i) {
    const auto& a = polygon.vertices[i];
    const auto& b = polygon.vertices[(i + 1U) % polygon.vertices.size()];
    const auto& c = polygon.vertices[(i + 2U) % polygon.vertices.size()];
    const auto value = static_cast<long double>(b.x.value() - a.x.value()) *
                           static_cast<long double>(c.y.value() - b.y.value()) -
                       static_cast<long double>(b.y.value() - a.y.value()) *
                           static_cast<long double>(c.x.value() - b.x.value());
    if (std::abs(value) <= 0.5L)
      continue;
    const auto current = value > 0.0L ? 1 : -1;
    if (sign != 0 && current != sign)
      return false;
    sign = current;
  }
  return sign != 0;
}

[[nodiscard]] bool point_in_convex(const Polygon& polygon, SplineVector point) {
  return geometry::locate_point(polygon, quantize(point)) != geometry::PointLocation::outside;
}

void clamp_node_to_anchor(QuinticPHRoute& route, std::size_t index, Nanometres maximum) {
  if (index >= route.control_points.size() || index >= route.corridor_anchors.size())
    return;
  auto delta = subtract(route.control_points[index], route.corridor_anchors[index]);
  const auto length = norm(delta);
  if (maximum.value() > 0 && length > static_cast<long double>(maximum.value())) {
    route.control_points[index] =
        add(route.corridor_anchors[index],
            scale(delta, static_cast<long double>(maximum.value()) / length));
  }
  if (route.convex_corridor.empty())
    return;
  if (std::any_of(
          route.convex_corridor.begin(), route.convex_corridor.end(),
          [&](const auto& cell) { return point_in_convex(cell, route.control_points[index]); }))
    return;
  route.control_points[index] = route.corridor_anchors[index];
}

} // namespace

std::vector<long double> select_quintic_ph_parameters(const std::vector<SplineVector>& points,
                                                      long double exponent) {
  if (points.empty())
    return {};
  exponent = std::clamp(exponent, 0.0L, 1.0L);
  std::vector<long double> result(points.size(), 0.0L);
  for (std::size_t i = 1U; i < points.size(); ++i) {
    result[i] = result[i - 1U] + std::pow(norm(subtract(points[i], points[i - 1U])), exponent);
  }
  if (!(result.back() > 0.0L)) {
    for (std::size_t i = 1U; i < result.size(); ++i)
      result[i] = static_cast<long double>(i) / static_cast<long double>(result.size() - 1U);
  } else {
    for (auto& value : result)
      value /= result.back();
  }
  return result;
}

std::optional<QuinticPHSegment> interpolate_quintic_ph_segment(SplineVector start, SplineVector end,
                                                               SplineVector start_derivative,
                                                               SplineVector end_derivative) {
  const auto chord = complex(subtract(end, start));
  if (!(std::abs(chord) > 0.0L) || !(norm(start_derivative) > 0.0L) ||
      !(norm(end_derivative) > 0.0L))
    return std::nullopt;
  const auto root0 = std::sqrt(complex(start_derivative));
  const auto root2 = std::sqrt(complex(end_derivative));
  std::optional<QuinticPHSegment> best;
  auto best_score = std::numeric_limits<long double>::infinity();
  std::size_t branch = 0U;
  for (const auto sign0 : {-1.0L, 1.0L}) {
    for (const auto sign2 : {-1.0L, 1.0L}) {
      const auto w0 = sign0 * root0;
      const auto w2 = sign2 * root2;
      const auto discriminant = 9.0L * (w0 + w2) * (w0 + w2) -
                                8.0L * (3.0L * w0 * w0 + w0 * w2 + 3.0L * w2 * w2 - 15.0L * chord);
      const auto root = std::sqrt(discriminant);
      for (const auto quadratic_sign : {-1.0L, 1.0L}) {
        const auto w1 = (-3.0L * (w0 + w2) + quadratic_sign * root) / 4.0L;
        auto candidate = build_segment(start, {w0, w1, w2}, branch++);
        const auto endpoint_error = norm(subtract(candidate.bezier.back(), end));
        if (!std::isfinite(endpoint_error) ||
            endpoint_error > std::max(1.0e-4L, norm(vector(chord)) * 1.0e-12L))
          continue;
        const auto score = segment_branch_score(candidate, vector(chord));
        if (score < best_score) {
          best_score = score;
          best = candidate;
        }
      }
    }
  }
  return best;
}

[[nodiscard]] bool prepare_route_interpolation_state(QuinticPHRoute& route) {
  if (route.control_points.size() < 2U)
    return false;
  route.node_parameters =
      select_quintic_ph_parameters(route.control_points, route.parameter_exponent);
  route.node_derivatives.assign(route.control_points.size(), {});
  for (std::size_t i = 0U; i < route.control_points.size(); ++i) {
    if (i == 0U) {
      const auto span = route.node_parameters[1U] - route.node_parameters[0U];
      route.node_derivatives[i] = scale(
          subtract(route.control_points[1U], route.control_points[0U]), route.tangent_scale / span);
    } else if (i + 1U == route.control_points.size()) {
      const auto span = route.node_parameters[i] - route.node_parameters[i - 1U];
      route.node_derivatives[i] =
          scale(subtract(route.control_points[i], route.control_points[i - 1U]),
                route.tangent_scale / span);
    } else {
      const auto before = subtract(route.control_points[i], route.control_points[i - 1U]);
      const auto after = subtract(route.control_points[i + 1U], route.control_points[i]);
      const auto direction = normalized(add(normalized(before), normalized(after)));
      const auto before_span = route.node_parameters[i] - route.node_parameters[i - 1U];
      const auto after_span = route.node_parameters[i + 1U] - route.node_parameters[i];
      const auto magnitude = std::min(norm(before) / before_span, norm(after) / after_span);
      route.node_derivatives[i] =
          scale(norm(direction) > 0.0L ? scale(direction, magnitude)
                                       : scale(normalized(after), magnitude),
                route.tangent_scale);
    }
    if (!(norm(route.node_derivatives[i]) > 0.0L))
      return false;
  }
  return true;
}

bool rebuild_quintic_ph_route(QuinticPHRoute& route) {
  if (!prepare_route_interpolation_state(route))
    return false;
  return build_route_segments_from_derivatives(route);
}

bool rebuild_quintic_ph_route_around_control(QuinticPHRoute& route,
                                             std::size_t control_index) {
  if (control_index >= route.control_points.size() ||
      route.segments.size() + 1U != route.control_points.size())
    return rebuild_quintic_ph_route(route);
  if (!prepare_route_interpolation_state(route))
    return false;

  // A node displacement changes its two adjacent chord lengths. The derived tangent at a node
  // depends only on its two adjacent chords, so the exact PH support is bounded to segment indices
  // [control-2, control+1]. Centripetal normalization changes every route parameter by the same
  // factor; it cancels when a global node derivative is scaled into an unaffected local span.
  const auto first_segment = control_index > 1U ? control_index - 2U : 0U;
  const auto last_segment = std::min(route.segments.size() - 1U, control_index + 1U);
  for (std::size_t segment_index = 0U; segment_index < route.segments.size(); ++segment_index) {
    const auto end_node = segment_index + 1U;
    const auto parameter_span =
        route.node_parameters[end_node] - route.node_parameters[segment_index];
    if (!(parameter_span > 0.0L))
      return false;
    if (segment_index >= first_segment && segment_index <= last_segment) {
      auto segment = interpolate_quintic_ph_segment(
          route.control_points[segment_index], route.control_points[end_node],
          scale(route.node_derivatives[segment_index], parameter_span),
          scale(route.node_derivatives[end_node], parameter_span));
      if (!segment)
        return false;
      route.segments[segment_index] = std::move(*segment);
    }
    route.segments[segment_index].route_parameter_start = route.node_parameters[segment_index];
    route.segments[segment_index].route_parameter_end = route.node_parameters[end_node];
  }
  return certify_route_join_continuity(route);
}

namespace {

[[nodiscard]] std::optional<QuinticPHRoute>
make_quintic_ph_route_from_nodes(const std::vector<Point>& nodes,
                                 const PHInterpolationConfig& config) {
  if (nodes.size() < 2U || !(config.tangent_scale > 0.0L) || !std::isfinite(config.tangent_scale) ||
      !std::isfinite(config.parameter_exponent))
    return std::nullopt;
  QuinticPHRoute route;
  route.requested_continuity = config.continuity;
  route.parameter_exponent = std::clamp(config.parameter_exponent, 0.0L, 1.0L);
  route.tangent_scale = config.tangent_scale;
  route.control_points.reserve(nodes.size());
  for (const auto point : nodes) {
    route.control_points.push_back(
        {static_cast<long double>(point.x.value()), static_cast<long double>(point.y.value())});
  }
  route.corridor_anchors = route.control_points;
  if (!rebuild_quintic_ph_route(route))
    return std::nullopt;
  if (config.continuity == PHJoinContinuity::curvature &&
      !enforce_quintic_ph_continuity(route, config.continuity, config.continuity_tolerance_nm,
                                     config.maximum_continuity_iterations))
    return std::nullopt;
  return route;
}

} // namespace

std::optional<QuinticPHRoute> make_quintic_ph_route(const std::vector<Point>& polyline,
                                                    const PHInterpolationConfig& config) {
  return make_quintic_ph_route_from_nodes(canonical_polyline(polyline), config);
}

namespace {

[[nodiscard]] std::optional<QuinticPHRoute>
make_guarded_quintic_ph_route(const std::vector<Point>& polyline, Nanometres corner_guard,
                              const PHInterpolationConfig& config, bool retain_corner) {
  const auto canonical = canonical_polyline(polyline);
  if (canonical.size() < 3U || corner_guard.value() <= 0)
    return make_quintic_ph_route(canonical, config);
  std::vector<Point> guarded;
  guarded.push_back(canonical.front());
  for (std::size_t i = 1U; i + 1U < canonical.size(); ++i) {
    const auto before = SplineVector{static_cast<long double>(canonical[i - 1U].x.value()),
                                     static_cast<long double>(canonical[i - 1U].y.value())};
    const auto corner = SplineVector{static_cast<long double>(canonical[i].x.value()),
                                     static_cast<long double>(canonical[i].y.value())};
    const auto after = SplineVector{static_cast<long double>(canonical[i + 1U].x.value()),
                                    static_cast<long double>(canonical[i + 1U].y.value())};
    const auto trim =
        std::min({static_cast<long double>(corner_guard.value()),
                  norm(subtract(corner, before)) * 0.45L, norm(subtract(after, corner)) * 0.45L});
    guarded.push_back(quantize(add(corner, scale(normalized(subtract(before, corner)), trim))));
    if (retain_corner)
      guarded.push_back(canonical[i]);
    guarded.push_back(quantize(add(corner, scale(normalized(subtract(after, corner)), trim))));
  }
  guarded.push_back(canonical.back());
  // These guards are deliberate topology controls. Passing them through the ordinary canonicalizer
  // would collapse them back into the source corner, allowing a following Hermite span to bow
  // outside a narrow selected portal. Regional seeds retain the apex as an exact obstacle-side
  // witness; finished corner rounding omits it so a C1 curve can bridge the trimmed incident runs.
  return make_quintic_ph_route_from_nodes(guarded, config);
}

} // namespace

std::optional<QuinticPHRoute>
make_topology_preserving_quintic_ph_route(const std::vector<Point>& polyline,
                                          Nanometres corner_guard,
                                          const PHInterpolationConfig& config) {
  return make_guarded_quintic_ph_route(polyline, corner_guard, config, true);
}

std::optional<QuinticPHRoute>
make_corner_rounded_quintic_ph_route(const std::vector<Point>& polyline, Nanometres corner_guard,
                                     const PHInterpolationConfig& config) {
  return make_guarded_quintic_ph_route(polyline, corner_guard, config, false);
}

bool enforce_quintic_ph_continuity(QuinticPHRoute& route, PHJoinContinuity continuity,
                                   long double tolerance_nm, std::size_t maximum_iterations) {
  if (!rebuild_quintic_ph_route(route))
    return false;
  route.requested_continuity = continuity;
  if (continuity == PHJoinContinuity::tangent || route.segments.size() <= 1U)
    return true;
  // Bounded nonlinear relaxation of shared nodal derivatives.  Every
  // interpolation point and C1 join is invariant; only the one derivative
  // shared by the adjacent Hermite segments moves.
  auto best = route;
  auto mismatch = [](const QuinticPHRoute& value) {
    long double result = 0.0L;
    for (std::size_t i = 1U; i < value.segments.size(); ++i) {
      auto left = evaluate_segment_differential(value.segments[i - 1U], 1.0L).second_derivative;
      auto right = evaluate_segment_differential(value.segments[i], 0.0L).second_derivative;
      const auto left_span =
          value.segments[i - 1U].route_parameter_end - value.segments[i - 1U].route_parameter_start;
      const auto right_span =
          value.segments[i].route_parameter_end - value.segments[i].route_parameter_start;
      left = scale(left, 1.0L / (left_span * left_span));
      right = scale(right, 1.0L / (right_span * right_span));
      result = std::max(result, norm(subtract(left, right)));
    }
    return result;
  };
  auto best_mismatch = mismatch(best);
  for (std::size_t iteration = 0U; iteration < maximum_iterations && best_mismatch > tolerance_nm;
       ++iteration) {
    bool improved = false;
    for (std::size_t node = 1U; node + 1U < route.control_points.size(); ++node) {
      const auto original = best.node_derivatives[node];
      const auto local =
          std::min(norm(subtract(route.control_points[node], route.control_points[node - 1U])),
                   norm(subtract(route.control_points[node + 1U], route.control_points[node])));
      const auto step = std::max(1.0L, local * 0.02L / static_cast<long double>(iteration + 1U));
      for (const auto direction :
           std::array<SplineVector, 4U>{{{1, 0}, {-1, 0}, {0, 1}, {0, -1}}}) {
        auto candidate = best;
        candidate.node_derivatives[node] = add(original, scale(direction, step));
        if (!(norm(candidate.node_derivatives[node]) > 0.0L) ||
            !build_route_segments_from_derivatives(candidate))
          continue;
        const auto candidate_mismatch = mismatch(candidate);
        if (candidate_mismatch + 1.0e-9L < best_mismatch) {
          best = std::move(candidate);
          best_mismatch = candidate_mismatch;
          improved = true;
        }
      }
    }
    if (!improved)
      break;
    route = best;
  }
  route = best;
  route.curvature_continuous = best_mismatch <= tolerance_nm;
  return route.curvature_continuous;
}

SplineVector evaluate_quintic_ph(const QuinticPHRoute& route, long double parameter) {
  if (route.segments.empty())
    return route.control_points.empty() ? SplineVector{} : route.control_points.front();
  const auto [segment, local] = locate_segment(route, parameter);
  return evaluate_power_array(route.segments[segment].position_power, local);
}

SplineDifferential evaluate_quintic_ph_differential(const QuinticPHRoute& route,
                                                    long double parameter) {
  if (route.segments.empty())
    return {};
  const auto [segment, local] = locate_segment(route, parameter);
  auto result = evaluate_segment_differential(route.segments[segment], local);
  const auto span =
      route.segments[segment].route_parameter_end - route.segments[segment].route_parameter_start;
  if (span > 0.0L) {
    result.tangent = scale(result.tangent, 1.0L / span);
    result.second_derivative = scale(result.second_derivative, 1.0L / (span * span));
  }
  return result;
}

std::vector<SplineVector> quintic_ph_bezier_controls(const QuinticPHRoute& route) {
  std::vector<SplineVector> result;
  result.reserve(route.segments.size() * 6U);
  for (const auto& segment : route.segments)
    result.insert(result.end(), segment.bezier.begin(), segment.bezier.end());
  return result;
}

std::vector<long double> quintic_ph_bezier_basis(const QuinticPHRoute& route,
                                                 long double parameter) {
  std::vector<long double> result(route.segments.size() * 6U, 0.0L);
  if (route.segments.empty())
    return result;
  const auto [segment, local] = locate_segment(route, parameter);
  for (std::size_t i = 0U; i < 6U; ++i)
    result[segment * 6U + i] = bernstein(5U, i, local);
  return result;
}

long double quintic_ph_arc_length(const QuinticPHSegment& segment, long double parameter) {
  return evaluate_power_array(segment.arc_length_power, std::clamp(parameter, 0.0L, 1.0L));
}

PreparedQuinticPHEvaluator::PreparedQuinticPHEvaluator(const QuinticPHRoute& route,
                                                       std::size_t sample_count)
    : segment_count_(route.segments.size()) {
  if (sample_count < 2U || segment_count_ == 0U)
    return;
  parameters_.reserve(sample_count);
  samples_.resize(sample_count);
  for (std::size_t i = 0U; i < sample_count; ++i)
    parameters_.push_back(static_cast<long double>(i) /
                          static_cast<long double>(sample_count - 1U));
  if (!update(route)) {
    parameters_.clear();
    samples_.clear();
    segment_count_ = 0U;
  }
}

bool PreparedQuinticPHEvaluator::valid() const {
  return segment_count_ > 0U && parameters_.size() >= 2U && parameters_.size() == samples_.size();
}

bool PreparedQuinticPHEvaluator::update(const QuinticPHRoute& route) {
  if (route.segments.size() != segment_count_ || parameters_.size() != samples_.size())
    return false;
  for (std::size_t i = 0U; i < parameters_.size(); ++i)
    samples_[i] = evaluate_quintic_ph_differential(route, parameters_[i]);
  return true;
}

bool insert_quintic_ph_interpolation_node(QuinticPHRoute& route, long double parameter) {
  if (route.segments.empty() || !(parameter > 0.0L && parameter < 1.0L))
    return false;
  if (route.node_parameters.size() != route.control_points.size() ||
      route.node_derivatives.size() != route.control_points.size() ||
      std::any_of(route.node_parameters.begin(), route.node_parameters.end(),
                  [&](auto existing) { return std::abs(existing - parameter) < 1.0e-12L; }))
    return false;
  const auto [segment, local] = locate_segment(route, parameter);
  if (!(local > 1.0e-12L && local < 1.0L - 1.0e-12L))
    return false;
  const auto insert_at = segment + 1U;
  const auto position = evaluate_quintic_ph(route, parameter);
  const auto derivative = evaluate_quintic_ph_differential(route, parameter).tangent;
  if (!(norm(derivative) > 0.0L))
    return false;
  auto candidate = route;
  candidate.control_points.insert(
      candidate.control_points.begin() + static_cast<std::ptrdiff_t>(insert_at), position);
  candidate.node_parameters.insert(
      candidate.node_parameters.begin() + static_cast<std::ptrdiff_t>(insert_at), parameter);
  candidate.node_derivatives.insert(
      candidate.node_derivatives.begin() + static_cast<std::ptrdiff_t>(insert_at), derivative);
  if (candidate.corridor_anchors.size() == route.control_points.size()) {
    candidate.corridor_anchors.insert(
        candidate.corridor_anchors.begin() + static_cast<std::ptrdiff_t>(insert_at), position);
  }
  // Subdivide the quadratic PH preimage exactly. Re-interpolating the two Hermite halves is not
  // shape preserving: a quintic PH Hermite problem has several algebraic branches, and choosing a
  // different branch after insertion can turn one local envelope violation into hundreds. If
  // w(t) is the source preimage, the left and right local preimages are sqrt(u) w(us) and
  // sqrt(1-u) w(u+(1-u)s), respectively. Squaring therefore reproduces the original hodograph
  // under the local parameter changes, while build_segment refreshes every hot power-basis,
  // arc-length, curvature, extrema, and bounds cache once.
  const auto& source = route.segments[segment];
  const auto w0 = complex(source.preimage[0U]);
  const auto w1 = complex(source.preimage[1U]);
  const auto w2 = complex(source.preimage[2U]);
  const auto first = (1.0L - local) * w0 + local * w1;
  const auto second = (1.0L - local) * w1 + local * w2;
  const auto middle = (1.0L - local) * first + local * second;
  const auto left_scale = std::sqrt(local);
  const auto right_scale = std::sqrt(1.0L - local);
  auto left = build_segment(source.bezier.front(),
                            {left_scale * w0, left_scale * first, left_scale * middle},
                            source.interpolation_branch);
  auto right =
      build_segment(position, {right_scale * middle, right_scale * second, right_scale * w2},
                    source.interpolation_branch);
  left.route_parameter_start = source.route_parameter_start;
  left.route_parameter_end = parameter;
  right.route_parameter_start = parameter;
  right.route_parameter_end = source.route_parameter_end;
  candidate.segments.erase(candidate.segments.begin() + static_cast<std::ptrdiff_t>(segment));
  candidate.segments.insert(candidate.segments.begin() + static_cast<std::ptrdiff_t>(segment),
                            {left, right});
  candidate.tangent_continuous = route.tangent_continuous;
  candidate.curvature_continuous = route.curvature_continuous;
  route = std::move(candidate);
  return true;
}

bool set_quintic_ph_corridor(QuinticPHRoute& route, const std::vector<Polygon>& convex_regions) {
  if (convex_regions.empty() ||
      !std::all_of(convex_regions.begin(), convex_regions.end(), convex_polygon))
    return false;
  route.convex_corridor = convex_regions;
  // A corridor is a constraint on an existing route, not a geometric degree of freedom. Every
  // public constructor and deformation path has already prepared the quadratic preimage, power
  // bases, extrema, and bounds. Rebuilding here repeated the eight-branch PH interpolation and
  // exact root certificates for every regional seed portfolio candidate even though no control
  // point changed. Preserve those immutable caches and let the following corridor certificate
  // decide whether the existing curve is admissible.
  return route.segments.size() + 1U == route.control_points.size();
}

std::vector<PHInflection> quintic_ph_inflections(const QuinticPHSegment& segment) {
  std::vector<PHInflection> result;
  result.reserve(segment.inflection_count);
  for (std::size_t i = 0U; i < segment.inflection_count; ++i)
    result.push_back({segment.inflection_parameters[i]});
  return result;
}

std::vector<PHCurvatureExtremum> quintic_ph_curvature_extrema(const QuinticPHSegment& segment) {
  std::vector<PHCurvatureExtremum> result;
  result.reserve(segment.curvature_extremum_count);
  for (std::size_t i = 0U; i < segment.curvature_extremum_count; ++i) {
    const auto parameter = segment.curvature_extremum_parameters[i];
    result.push_back({parameter, evaluate_preimage_kinematics(segment, parameter).curvature});
  }
  return result;
}

PHRationalOffset make_quintic_ph_offset(const QuinticPHSegment& segment,
                                        long double signed_distance_nm) {
  PHRationalOffset result;
  result.distance_nm = signed_distance_nm;
  const std::vector<Complex> curve = [&] {
    std::vector<Complex> values;
    values.reserve(6U);
    for (const auto point : segment.bezier)
      values.push_back(complex(point));
    return values;
  }();
  const std::vector<long double> speed(segment.speed.begin(), segment.speed.end());
  const auto weighted_curve = bernstein_product(curve, speed);
  const auto w0 = complex(segment.preimage[0]);
  const auto w1 = complex(segment.preimage[1]);
  const auto w2 = complex(segment.preimage[2]);
  const std::vector<Complex> derivative{w0 * w0, w0 * w1, (w0 * w2 + 2.0L * w1 * w1) / 3.0L,
                                        w1 * w2, w2 * w2};
  const std::vector<long double> degree_five_one(6U, 1.0L);
  const auto elevated_derivative = bernstein_product(derivative, degree_five_one);
  const auto elevated_speed = bernstein_product(speed, degree_five_one);
  result.regular = weighted_curve.size() == 10U && elevated_derivative.size() == 10U &&
                   elevated_speed.size() == 10U && segment.regular;
  for (std::size_t i = 0U; i < 10U && result.regular; ++i) {
    const auto numerator =
        weighted_curve[i] + Complex{0.0L, signed_distance_nm} * elevated_derivative[i];
    result.weight[i] = elevated_speed[i];
    result.weighted_x[i] = numerator.real();
    result.weighted_y[i] = numerator.imag();
    result.regular = result.regular && std::isfinite(result.weight[i]) &&
                     std::isfinite(result.weighted_x[i]) && std::isfinite(result.weighted_y[i]);
  }
  result.weight_power = bernstein_to_power_array(result.weight);
  result.weighted_x_power = bernstein_to_power_array(result.weighted_x);
  result.weighted_y_power = bernstein_to_power_array(result.weighted_y);
  return result;
}

SplineVector evaluate_quintic_ph_offset(const PHRationalOffset& offset, long double parameter) {
  if (!offset.regular)
    return {};
  const auto t = std::clamp(parameter, 0.0L, 1.0L);
  const auto weight = evaluate_power_array(offset.weight_power, t);
  if (!(weight > 1.0e-30L))
    return {};
  return {evaluate_power_array(offset.weighted_x_power, t) / weight,
          evaluate_power_array(offset.weighted_y_power, t) / weight};
}

namespace {

struct RationalPiece {
  std::array<long double, 10U> weight{};
  std::array<long double, 10U> x{};
  std::array<long double, 10U> y{};
  long double first{0.0L};
  long double last{1.0L};
  std::size_t depth{0U};
};

struct Bounds {
  long double minimum_x{std::numeric_limits<long double>::infinity()};
  long double minimum_y{std::numeric_limits<long double>::infinity()};
  long double maximum_x{-std::numeric_limits<long double>::infinity()};
  long double maximum_y{-std::numeric_limits<long double>::infinity()};
};

[[nodiscard]] Bounds rational_bounds(const RationalPiece& piece) {
  Bounds result;
  for (std::size_t i = 0U; i < 10U; ++i) {
    if (!(piece.weight[i] > 0.0L))
      return {};
    const auto x = piece.x[i] / piece.weight[i];
    const auto y = piece.y[i] / piece.weight[i];
    result.minimum_x = std::min(result.minimum_x, x);
    result.minimum_y = std::min(result.minimum_y, y);
    result.maximum_x = std::max(result.maximum_x, x);
    result.maximum_y = std::max(result.maximum_y, y);
  }
  return result;
}

[[nodiscard]] bool overlap(Bounds a, Bounds b, long double tolerance) {
  return a.minimum_x <= b.maximum_x + tolerance && b.minimum_x <= a.maximum_x + tolerance &&
         a.minimum_y <= b.maximum_y + tolerance && b.minimum_y <= a.maximum_y + tolerance;
}

[[nodiscard]] long double bounds_size(Bounds value) {
  return std::max(value.maximum_x - value.minimum_x, value.maximum_y - value.minimum_y);
}

[[nodiscard]] std::pair<RationalPiece, RationalPiece> split_piece(const RationalPiece& source) {
  RationalPiece left = source;
  RationalPiece right = source;
  std::array<std::array<long double, 10U>, 10U> w{};
  std::array<std::array<long double, 10U>, 10U> x{};
  std::array<std::array<long double, 10U>, 10U> y{};
  w[0] = source.weight;
  x[0] = source.x;
  y[0] = source.y;
  for (std::size_t level = 1U; level < 10U; ++level) {
    for (std::size_t i = 0U; i + level < 10U; ++i) {
      w[level][i] = (w[level - 1U][i] + w[level - 1U][i + 1U]) * 0.5L;
      x[level][i] = (x[level - 1U][i] + x[level - 1U][i + 1U]) * 0.5L;
      y[level][i] = (y[level - 1U][i] + y[level - 1U][i + 1U]) * 0.5L;
    }
  }
  for (std::size_t i = 0U; i < 10U; ++i) {
    left.weight[i] = w[i][0];
    left.x[i] = x[i][0];
    left.y[i] = y[i][0];
    right.weight[9U - i] = w[i][9U - i];
    right.x[9U - i] = x[i][9U - i];
    right.y[9U - i] = y[i][9U - i];
  }
  const auto middle = (source.first + source.last) * 0.5L;
  left.last = middle;
  right.first = middle;
  left.depth = right.depth = source.depth + 1U;
  return {left, right};
}

[[nodiscard]] SplineVector rational_derivative(const PHRationalOffset& offset,
                                               long double parameter) {
  const auto t = std::clamp(parameter, 0.0L, 1.0L);
  const auto evaluate_derivative = [&](const auto& coefficients) {
    long double value = 0.0L;
    for (std::size_t i = coefficients.size() - 1U; i > 0U; --i)
      value = value * t + coefficients[i] * static_cast<long double>(i);
    return value;
  };
  const auto w = evaluate_power_array(offset.weight_power, t);
  const auto x = evaluate_power_array(offset.weighted_x_power, t);
  const auto y = evaluate_power_array(offset.weighted_y_power, t);
  const auto wp = evaluate_derivative(offset.weight_power);
  const auto xp = evaluate_derivative(offset.weighted_x_power);
  const auto yp = evaluate_derivative(offset.weighted_y_power);
  return {(xp * w - x * wp) / (w * w), (yp * w - y * wp) / (w * w)};
}

} // namespace

std::vector<PHOffsetIntersection>
intersect_quintic_ph_offsets(const PHRationalOffset& first, const PHRationalOffset& second,
                             Nanometres tolerance, std::size_t maximum_subdivision_depth,
                             std::size_t maximum_intersections,
                             std::size_t maximum_subdivision_nodes,
                             PHOffsetIntersectionStats* stats) {
  std::vector<PHOffsetIntersection> result;
  if (stats != nullptr)
    *stats = {};
  if (!first.regular || !second.regular || tolerance.value() < 0 || maximum_intersections == 0U ||
      maximum_subdivision_nodes == 0U)
    return result;
  RationalPiece first_root{first.weight, first.weighted_x, first.weighted_y};
  RationalPiece second_root{second.weight, second.weighted_x, second.weighted_y};
  std::vector<std::pair<RationalPiece, RationalPiece>> stack{{first_root, second_root}};
  std::size_t nodes = 0U;
  const auto tolerance_nm = std::max(1.0L, static_cast<long double>(tolerance.value()));
  while (!stack.empty() && result.size() < maximum_intersections &&
         nodes++ < maximum_subdivision_nodes) {
    auto [a, b] = std::move(stack.back());
    stack.pop_back();
    if (stats != nullptr) {
      stats->subdivision_nodes = nodes;
      stats->maximum_depth_reached =
          std::max(stats->maximum_depth_reached, std::max(a.depth, b.depth));
    }
    const auto positive_weights = [](const RationalPiece& piece) {
      return std::all_of(piece.weight.begin(), piece.weight.end(),
                         [](auto value) { return value > 1.0e-24L; });
    };
    if (!positive_weights(a) && a.depth < maximum_subdivision_depth) {
      auto [left, right] = split_piece(a);
      stack.emplace_back(right, b);
      stack.emplace_back(left, std::move(b));
      continue;
    }
    if (!positive_weights(b) && b.depth < maximum_subdivision_depth) {
      auto [left, right] = split_piece(b);
      stack.emplace_back(a, right);
      stack.emplace_back(std::move(a), left);
      continue;
    }
    if (!positive_weights(a) || !positive_weights(b))
      continue;
    const auto a_bounds = rational_bounds(a);
    const auto b_bounds = rational_bounds(b);
    if (!overlap(a_bounds, b_bounds, tolerance_nm))
      continue;
    const auto a_size = bounds_size(a_bounds);
    const auto b_size = bounds_size(b_bounds);
    if ((a_size <= tolerance_nm && b_size <= tolerance_nm) ||
        (a.depth >= maximum_subdivision_depth && b.depth >= maximum_subdivision_depth)) {
      if (stats != nullptr)
        ++stats->isolated_leaf_pairs;
      auto u = (a.first + a.last) * 0.5L;
      auto v = (b.first + b.last) * 0.5L;
      for (std::size_t iteration = 0U; iteration < 16U; ++iteration) {
        const auto pa = evaluate_quintic_ph_offset(first, u);
        const auto pb = evaluate_quintic_ph_offset(second, v);
        const auto residual = subtract(pa, pb);
        const auto du = rational_derivative(first, u);
        const auto dv = rational_derivative(second, v);
        const auto determinant = cross(du, dv);
        if (std::abs(determinant) < 1.0e-24L)
          break;
        const auto delta_u = -cross(residual, dv) / determinant;
        const auto delta_v = -cross(residual, du) / determinant;
        u = std::clamp(u + delta_u, a.first, a.last);
        v = std::clamp(v + delta_v, b.first, b.last);
        if (std::max(std::abs(delta_u), std::abs(delta_v)) < 1.0e-13L)
          break;
      }
      const auto pa = evaluate_quintic_ph_offset(first, u);
      const auto pb = evaluate_quintic_ph_offset(second, v);
      const auto residual = norm(subtract(pa, pb));
      if (residual <= tolerance_nm &&
          std::none_of(result.begin(), result.end(), [&](const auto& existing) {
            return std::abs(existing.first_parameter - u) < 1.0e-7L &&
                   std::abs(existing.second_parameter - v) < 1.0e-7L;
          }))
        result.push_back({u, v, scale(add(pa, pb), 0.5L), residual});
      continue;
    }
    if ((a_size >= b_size && a.depth < maximum_subdivision_depth) ||
        b.depth >= maximum_subdivision_depth) {
      auto [left, right] = split_piece(a);
      stack.emplace_back(right, b);
      stack.emplace_back(left, std::move(b));
    } else {
      auto [left, right] = split_piece(b);
      stack.emplace_back(a, right);
      stack.emplace_back(std::move(a), left);
    }
  }
  if (stats != nullptr)
    stats->node_budget_exhausted = !stack.empty() && nodes >= maximum_subdivision_nodes;
  std::sort(result.begin(), result.end(), [](const auto& a, const auto& b) {
    return std::tie(a.first_parameter, a.second_parameter) <
           std::tie(b.first_parameter, b.second_parameter);
  });
  return result;
}

std::vector<PHOffsetSubdivisionBounds>
subdivide_quintic_ph_offset_bounds(const PHRationalOffset& offset,
                                   std::size_t subdivision_depth, Nanometres outward_guard) {
  std::vector<PHOffsetSubdivisionBounds> result;
  if (!offset.regular || outward_guard.value() < 0 || subdivision_depth > 12U)
    return result;
  std::vector<RationalPiece> stack{{offset.weight, offset.weighted_x, offset.weighted_y}};
  while (!stack.empty()) {
    auto piece = std::move(stack.back());
    stack.pop_back();
    if (piece.depth < subdivision_depth) {
      auto [left, right] = split_piece(piece);
      stack.push_back(std::move(right));
      stack.push_back(std::move(left));
      continue;
    }
    if (!std::all_of(piece.weight.begin(), piece.weight.end(),
                     [](auto value) { return value > 1.0e-24L; }))
      return {};
    const auto bounds = rational_bounds(piece);
    const auto guard = static_cast<long double>(outward_guard.value());
    const auto clamp_i64 = [](long double value) {
      return static_cast<std::int64_t>(std::clamp(
          value, static_cast<long double>(std::numeric_limits<std::int64_t>::min()),
          static_cast<long double>(std::numeric_limits<std::int64_t>::max())));
    };
    result.push_back(PHOffsetSubdivisionBounds{
        clamp_i64(std::floor(bounds.minimum_x - guard)),
        clamp_i64(std::floor(bounds.minimum_y - guard)),
        clamp_i64(std::ceil(bounds.maximum_x + guard)),
        clamp_i64(std::ceil(bounds.maximum_y + guard))});
  }
  return result;
}

namespace {

[[nodiscard]] long double point_line_distance(SplineVector point, SplineVector first,
                                              SplineVector second) {
  const auto chord = subtract(second, first);
  const auto length = norm(chord);
  return length > 0.0L ? std::abs(cross(chord, subtract(point, first))) / length
                       : norm(subtract(point, first));
}

[[nodiscard]] std::pair<std::array<SplineVector, 6U>, std::array<SplineVector, 6U>>
split_bezier(const std::array<SplineVector, 6U>& source) {
  std::array<std::array<SplineVector, 6U>, 6U> triangle{};
  triangle[0] = source;
  for (std::size_t level = 1U; level < 6U; ++level)
    for (std::size_t i = 0U; i + level < 6U; ++i)
      triangle[level][i] = lerp(triangle[level - 1U][i], triangle[level - 1U][i + 1U], 0.5L);
  std::array<SplineVector, 6U> left{};
  std::array<SplineVector, 6U> right{};
  for (std::size_t i = 0U; i < 6U; ++i) {
    left[i] = triangle[i][0];
    right[5U - i] = triangle[i][5U - i];
  }
  return {left, right};
}

[[nodiscard]] long double bezier_flatness(const std::array<SplineVector, 6U>& controls) {
  long double result = 0.0L;
  for (std::size_t i = 1U; i < 5U; ++i)
    result = std::max(result, point_line_distance(controls[i], controls.front(), controls.back()));
  return result;
}

struct TessellationNode {
  std::array<SplineVector, 6U> controls;
  long double first{0.0L};
  long double last{1.0L};
  std::size_t depth{0U};
};

[[nodiscard]] bool proper_segment_intersection(SplineVector a, SplineVector b, SplineVector c,
                                               SplineVector d) {
  const auto ab_c = cross(subtract(b, a), subtract(c, a));
  const auto ab_d = cross(subtract(b, a), subtract(d, a));
  const auto cd_a = cross(subtract(d, c), subtract(a, c));
  const auto cd_b = cross(subtract(d, c), subtract(b, c));
  return ((ab_c > 0.0L && ab_d < 0.0L) || (ab_c < 0.0L && ab_d > 0.0L)) &&
         ((cd_a > 0.0L && cd_b < 0.0L) || (cd_a < 0.0L && cd_b > 0.0L));
}

struct DirectionAssessment {
  long double total_turn{0.0L};
  std::size_t inflections{0U};
  std::size_t turns{0U};
};

[[nodiscard]] DirectionAssessment assess_directions(const std::vector<SplineVector>& directions) {
  DirectionAssessment result;
  std::optional<SplineVector> prior;
  int prior_sign = 0;
  for (const auto direction : directions) {
    if (!(norm(direction) > 0.0L))
      continue;
    const auto unit = normalized(direction);
    if (prior) {
      const auto turn = std::atan2(cross(*prior, unit), dot(*prior, unit));
      if (std::abs(turn) > 1.0e-7L) {
        result.total_turn += std::abs(turn);
        ++result.turns;
        const auto sign = turn > 0.0L ? 1 : -1;
        if (prior_sign != 0 && sign != prior_sign)
          ++result.inflections;
        prior_sign = sign;
      }
    }
    prior = unit;
  }
  return result;
}

[[nodiscard]] bool circle_through(SplineVector a, SplineVector b, SplineVector c,
                                  SplineVector& center, long double& radius) {
  const auto determinant = 2.0L * (a.x * (b.y - c.y) + b.x * (c.y - a.y) + c.x * (a.y - b.y));
  if (std::abs(determinant) < 1.0e-12L)
    return false;
  const auto aa = dot(a, a);
  const auto bb = dot(b, b);
  const auto cc = dot(c, c);
  center.x = (aa * (b.y - c.y) + bb * (c.y - a.y) + cc * (a.y - b.y)) / determinant;
  center.y = (aa * (c.x - b.x) + bb * (a.x - c.x) + cc * (b.x - a.x)) / determinant;
  radius = norm(subtract(a, center));
  return std::isfinite(radius) && radius > 0.0L;
}

[[nodiscard]] bool polygon_contains_all(const Polygon& polygon,
                                        const std::array<SplineVector, 6U>& controls) {
  return std::all_of(controls.begin(), controls.end(),
                     [&](const auto point) { return point_in_convex(polygon, point); });
}

struct ContinuousRectangle {
  long double minimum_x{0.0L};
  long double minimum_y{0.0L};
  long double maximum_x{0.0L};
  long double maximum_y{0.0L};
};

[[nodiscard]] std::optional<ContinuousRectangle> axis_aligned_rectangle(const Polygon& polygon) {
  if (polygon.vertices.size() != 4U)
    return std::nullopt;
  ContinuousRectangle result{static_cast<long double>(polygon.vertices.front().x.value()),
                             static_cast<long double>(polygon.vertices.front().y.value()),
                             static_cast<long double>(polygon.vertices.front().x.value()),
                             static_cast<long double>(polygon.vertices.front().y.value())};
  for (const auto point : polygon.vertices) {
    result.minimum_x = std::min(result.minimum_x, static_cast<long double>(point.x.value()));
    result.minimum_y = std::min(result.minimum_y, static_cast<long double>(point.y.value()));
    result.maximum_x = std::max(result.maximum_x, static_cast<long double>(point.x.value()));
    result.maximum_y = std::max(result.maximum_y, static_cast<long double>(point.y.value()));
  }
  if (!(result.maximum_x > result.minimum_x && result.maximum_y > result.minimum_y))
    return std::nullopt;
  for (const auto point : polygon.vertices) {
    const auto x = static_cast<long double>(point.x.value());
    const auto y = static_cast<long double>(point.y.value());
    if ((x != result.minimum_x && x != result.maximum_x) ||
        (y != result.minimum_y && y != result.maximum_y))
      return std::nullopt;
  }
  return result;
}

[[nodiscard]] bool intervals_cover(long double minimum, long double maximum,
                                   std::vector<std::pair<long double, long double>> intervals) {
  if (minimum == maximum) {
    return std::any_of(intervals.begin(), intervals.end(), [&](const auto& interval) {
      return interval.first <= minimum && minimum <= interval.second;
    });
  }
  std::sort(intervals.begin(), intervals.end());
  auto covered = minimum;
  for (const auto [first, last] : intervals) {
    if (last < covered)
      continue;
    if (first > covered)
      return false;
    covered = std::max(covered, last);
    if (covered >= maximum)
      return true;
  }
  return false;
}

[[nodiscard]] std::vector<SplineVector>
convex_hull_controls(const std::array<SplineVector, 6U>& controls) {
  std::vector<SplineVector> points(controls.begin(), controls.end());
  std::sort(points.begin(), points.end(),
            [](const auto a, const auto b) { return std::tie(a.x, a.y) < std::tie(b.x, b.y); });
  points.erase(std::unique(points.begin(), points.end()), points.end());
  if (points.size() <= 2U)
    return points;
  std::vector<SplineVector> hull;
  hull.reserve(points.size() * 2U);
  for (const auto point : points) {
    while (hull.size() >= 2U && cross(subtract(hull.back(), hull[hull.size() - 2U]),
                                      subtract(point, hull.back())) <= 0.0L)
      hull.pop_back();
    hull.push_back(point);
  }
  const auto lower_size = hull.size();
  for (auto it = points.rbegin() + 1; it != points.rend(); ++it) {
    while (hull.size() > lower_size &&
           cross(subtract(hull.back(), hull[hull.size() - 2U]), subtract(*it, hull.back())) <= 0.0L)
      hull.pop_back();
    hull.push_back(*it);
  }
  if (hull.size() > 1U)
    hull.pop_back();
  return hull;
}

[[nodiscard]] std::optional<std::pair<long double, long double>>
hull_vertical_interval(const std::vector<SplineVector>& hull, long double x) {
  if (hull.empty())
    return std::nullopt;
  long double minimum = std::numeric_limits<long double>::infinity();
  long double maximum = -std::numeric_limits<long double>::infinity();
  for (std::size_t edge = 0U; edge < hull.size(); ++edge) {
    const auto first = hull[edge];
    const auto second = hull[(edge + 1U) % hull.size()];
    if (x < std::min(first.x, second.x) || x > std::max(first.x, second.x))
      continue;
    if (first.x == second.x) {
      minimum = std::min({minimum, first.y, second.y});
      maximum = std::max({maximum, first.y, second.y});
    } else {
      const auto parameter = (x - first.x) / (second.x - first.x);
      const auto y = first.y + parameter * (second.y - first.y);
      minimum = std::min(minimum, y);
      maximum = std::max(maximum, y);
    }
  }
  if (!std::isfinite(minimum) || !std::isfinite(maximum))
    return std::nullopt;
  return std::pair{minimum, maximum};
}

// A NavMesh corridor is a union of certified axis-aligned cells. Requiring one cell to contain a
// complete subdivided hull incorrectly rejects every ordinary crossing of a shared cell edge.
// Partition the actual convex Bezier control hull at every hull/cell X event. Within a slab the
// hull's upper and lower boundaries are linear and the cell cross section is constant, so covering
// the endpoint vertical ranges proves the complete hull slab. This avoids the false empty corners
// introduced by a whole-hull AABB while remaining a conservative continuous certificate.
[[nodiscard]] bool corridor_covers_bezier_hull(const std::vector<Polygon>& corridor,
                                               const std::array<SplineVector, 6U>& controls) {
  if (std::any_of(corridor.begin(), corridor.end(),
                  [&](const auto& polygon) { return polygon_contains_all(polygon, controls); }))
    return true;
  std::vector<ContinuousRectangle> rectangles;
  rectangles.reserve(corridor.size());
  for (const auto& polygon : corridor) {
    const auto rectangle = axis_aligned_rectangle(polygon);
    if (!rectangle)
      return false;
    rectangles.push_back(*rectangle);
  }
  const auto hull = convex_hull_controls(controls);
  if (hull.empty())
    return false;
  const auto [minimum_x, maximum_x] = std::minmax_element(
      hull.begin(), hull.end(), [](const auto a, const auto b) { return a.x < b.x; });
  if (minimum_x->x == maximum_x->x) {
    const auto vertical = hull_vertical_interval(hull, minimum_x->x);
    if (!vertical)
      return false;
    std::vector<std::pair<long double, long double>> intervals;
    for (const auto& rectangle : rectangles) {
      if (rectangle.minimum_x <= minimum_x->x && minimum_x->x <= rectangle.maximum_x)
        intervals.emplace_back(rectangle.minimum_y, rectangle.maximum_y);
    }
    return intervals_cover(vertical->first, vertical->second, std::move(intervals));
  }
  std::vector<long double> x_breaks;
  x_breaks.reserve(hull.size() + rectangles.size() * 2U);
  for (const auto point : hull)
    x_breaks.push_back(point.x);
  for (const auto& rectangle : rectangles) {
    if (rectangle.minimum_x > minimum_x->x && rectangle.minimum_x < maximum_x->x)
      x_breaks.push_back(rectangle.minimum_x);
    if (rectangle.maximum_x > minimum_x->x && rectangle.maximum_x < maximum_x->x)
      x_breaks.push_back(rectangle.maximum_x);
  }
  std::sort(x_breaks.begin(), x_breaks.end());
  x_breaks.erase(std::unique(x_breaks.begin(), x_breaks.end()), x_breaks.end());
  for (std::size_t slab = 1U; slab < x_breaks.size(); ++slab) {
    const auto left_x = x_breaks[slab - 1U];
    const auto right_x = x_breaks[slab];
    const auto x = (left_x + right_x) * 0.5L;
    const auto left = hull_vertical_interval(hull, left_x);
    const auto right = hull_vertical_interval(hull, right_x);
    if (!left || !right)
      return false;
    const auto slab_minimum = std::min(left->first, right->first);
    const auto slab_maximum = std::max(left->second, right->second);
    std::vector<std::pair<long double, long double>> intervals;
    for (const auto& rectangle : rectangles) {
      if (rectangle.minimum_x <= x && x <= rectangle.maximum_x)
        intervals.emplace_back(rectangle.minimum_y, rectangle.maximum_y);
    }
    if (!intervals_cover(slab_minimum, slab_maximum, std::move(intervals)))
      return false;
  }
  return true;
}

struct TrajectoryAssessment {
  std::size_t violations{std::numeric_limits<std::size_t>::max()};
  std::size_t shape_violations{std::numeric_limits<std::size_t>::max()};
  std::size_t bend_radius_violations{0U};
  std::size_t sampled_clearance_violations{0U};
  long double worst_bend_parameter{0.5L};
  long double objective{std::numeric_limits<long double>::infinity()};
};

[[nodiscard]] long double tangent_penalty(SplineVector tangent,
                                          const std::optional<SplineVector>& preferred) {
  if (!preferred || !(norm(*preferred) > 0.0L) || !(norm(tangent) > 0.0L))
    return 0.0L;
  return 1.0L - std::clamp(dot(normalized(tangent), normalized(*preferred)), -1.0L, 1.0L);
}

[[nodiscard]] TrajectoryAssessment
assess_trajectory(const QuinticPHRoute& route, const SplineTrajectoryOptimizationConfig& config,
                  const SplineClearanceEvaluator& clearance_evaluator) {
  TrajectoryAssessment result;
  if (route.segments.empty() || config.samples < 3U)
    return result;
  result.violations = 0U;
  auto shape_config = config.shape;
  shape_config.minimum_bend_radius = config.minimum_bend_radius;
  const auto shape = certify_quintic_ph_shape(route, shape_config);
  result.shape_violations = shape.violations();
  result.bend_radius_violations = shape.bend_radius_violations;
  result.worst_bend_parameter = shape.worst_bend_parameter;
  result.violations += result.shape_violations;
  result.objective = 0.0L;
  constexpr long double nm_per_mm = 1'000'000.0L;
  auto prior = evaluate_quintic_ph_differential(route, 0.0L);
  for (std::size_t i = 0U; i < config.samples; ++i) {
    const auto parameter =
        static_cast<long double>(i) / static_cast<long double>(config.samples - 1U);
    const auto sample = evaluate_quintic_ph_differential(route, parameter);
    long double step_mm = 0.0L;
    if (i > 0U) {
      step_mm = norm(subtract(sample.position, prior.position)) / nm_per_mm;
      result.objective += config.weights.length * step_mm;
      const auto change = (sample.curvature_per_nm - prior.curvature_per_nm) * nm_per_mm;
      result.objective +=
          config.weights.curvature_change * change * change / std::max(1.0e-9L, step_mm);
    }
    const auto curvature = sample.curvature_per_nm * nm_per_mm;
    result.objective += config.weights.curvature * curvature * curvature * step_mm;
    if (config.minimum_bend_radius.value() > 0 &&
        sample.bend_radius_nm < static_cast<long double>(config.minimum_bend_radius.value())) {
      const auto deficit =
          (static_cast<long double>(config.minimum_bend_radius.value()) - sample.bend_radius_nm) /
          nm_per_mm;
      result.objective += config.weights.bend_radius * deficit * deficit;
    }
    if (clearance_evaluator) {
      const auto probe = clearance_evaluator(sample.position);
      const auto deficit =
          static_cast<long double>(config.minimum_clearance.value()) - probe.clearance_nm;
      if (!probe.represented_legal || deficit > 0.0L) {
        result.objective +=
            config.weights.clearance * std::pow(std::max(0.0L, deficit) / nm_per_mm, 2.0L);
        ++result.violations;
        ++result.sampled_clearance_violations;
      } else {
        const auto influence = static_cast<long double>(
            std::max<std::int64_t>(0, config.clearance_influence_distance.value()));
        if (influence > 0.0L && probe.clearance_nm < influence) {
          result.objective += config.weights.clearance *
                              std::pow((influence - probe.clearance_nm) / influence, 2.0L);
        }
      }
    }
    prior = sample;
  }
  const auto start = evaluate_quintic_ph_differential(route, 0.0L);
  const auto end = evaluate_quintic_ph_differential(route, 1.0L);
  result.objective +=
      config.weights.tangent * (tangent_penalty(start.tangent, config.preferred_start_tangent) +
                                tangent_penalty(end.tangent, config.preferred_end_tangent));
  for (std::size_t i = 1U; i + 1U < route.control_points.size(); ++i) {
    if (i < route.corridor_anchors.size()) {
      const auto displacement =
          norm(subtract(route.control_points[i], route.corridor_anchors[i])) / nm_per_mm;
      result.objective += config.weights.elastic_anchor * displacement * displacement;
    }
    const auto second =
        add(subtract(route.control_points[i + 1U], scale(route.control_points[i], 2.0L)),
            route.control_points[i - 1U]);
    result.objective +=
        config.weights.control_complexity * dot(second, second) / (nm_per_mm * nm_per_mm);
    // Adaptive nodes are deliberately inserted at continuous-envelope failure
    // parameters.  Probing them here gives the cheap candidate loop awareness
    // of an inter-sample obstacle without recursively certifying every trial.
    if (clearance_evaluator) {
      const auto probe = clearance_evaluator(route.control_points[i]);
      const auto deficit =
          static_cast<long double>(config.minimum_clearance.value()) - probe.clearance_nm;
      if (!probe.represented_legal || deficit > 0.0L) {
        result.objective +=
            config.weights.clearance * std::pow(std::max(0.0L, deficit) / nm_per_mm, 2.0L);
        ++result.violations;
        ++result.sampled_clearance_violations;
      }
    }
  }
  return result;
}

} // namespace

std::vector<SplineSample> adaptive_tessellate_quintic_ph(const QuinticPHRoute& route,
                                                         Nanometres tolerance,
                                                         std::size_t maximum_depth,
                                                         std::size_t maximum_samples) {
  std::vector<SplineSample> result;
  if (route.segments.empty() || maximum_samples < 2U)
    return result;
  result.push_back({0.0L, route.segments.front().bezier.front()});
  const auto threshold = std::max(1.0L, static_cast<long double>(tolerance.value()));
  for (const auto& segment : route.segments) {
    std::vector<TessellationNode> stack{
        {segment.bezier, segment.route_parameter_start, segment.route_parameter_end, 0U}};
    while (!stack.empty()) {
      auto node = stack.back();
      stack.pop_back();
      if (bezier_flatness(node.controls) <= threshold || node.depth >= maximum_depth) {
        if (result.size() >= maximum_samples)
          return {};
        result.push_back({node.last, node.controls.back()});
        continue;
      }
      const auto [left_controls, right_controls] = split_bezier(node.controls);
      const auto middle = (node.first + node.last) * 0.5L;
      stack.push_back({right_controls, middle, node.last, node.depth + 1U});
      stack.push_back({left_controls, node.first, middle, node.depth + 1U});
    }
  }
  return result;
}

std::vector<SplineFittedPiece>
fit_spline_samples_to_lines_and_arcs(const std::vector<SplineSample>& samples, Nanometres tolerance,
                                     std::size_t maximum_arc_samples, std::size_t maximum_pieces) {
  std::vector<SplineFittedPiece> result;
  if (samples.size() < 2U || maximum_pieces == 0U)
    return result;
  const auto allowed = std::max(1.0L, static_cast<long double>(tolerance.value()));
  std::size_t first = 0U;
  while (first + 1U < samples.size() && result.size() < maximum_pieces) {
    std::size_t accepted_line = first + 1U;
    std::size_t accepted_arc = first;
    const auto last_limit = samples.size() - 1U;
    const auto arc_last_limit =
        std::min(samples.size() - 1U, first + std::max<std::size_t>(2U, maximum_arc_samples));
    for (std::size_t last = first + 2U; last <= last_limit; ++last) {
      const auto chord = subtract(samples[last].position, samples[first].position);
      const auto chord_length = norm(chord);
      bool line_legal = chord_length > 0.0L;
      long double prior_projection = 0.0L;
      for (std::size_t sample = first + 1U; line_legal && sample < last; ++sample) {
        const auto offset = subtract(samples[sample].position, samples[first].position);
        const auto projection = dot(offset, chord) / chord_length;
        const auto distance = std::abs(cross(chord, offset)) / chord_length;
        line_legal = distance <= allowed && projection + allowed >= prior_projection &&
                     projection >= -allowed && projection <= chord_length + allowed;
        prior_projection = std::max(prior_projection, projection);
      }
      if (line_legal)
        accepted_line = last;

      if (last > arc_last_limit)
        continue;
      const auto middle = first + (last - first) / 2U;
      SplineVector center;
      long double radius = 0.0L;
      if (!circle_through(samples[first].position, samples[middle].position, samples[last].position,
                          center, radius))
        continue;
      bool legal = true;
      int turn_sign = 0;
      for (std::size_t i = first; i <= last; ++i) {
        if (std::abs(norm(subtract(samples[i].position, center)) - radius) > allowed) {
          legal = false;
          break;
        }
        if (i > first && i < last) {
          const auto sign_value = cross(subtract(samples[i].position, samples[i - 1U].position),
                                        subtract(samples[i + 1U].position, samples[i].position));
          if (std::abs(sign_value) > 1.0L) {
            const auto current = sign_value > 0.0L ? 1 : -1;
            if (turn_sign != 0 && turn_sign != current)
              legal = false;
            turn_sign = current;
          }
        }
      }
      if (legal && turn_sign != 0) {
        accepted_arc = last;
      }
    }
    // A PH curve must not degrade back into a chain of differently directed straight chords.
    // Prefer every certified circular approximation in a genuinely turning span; retain a line
    // only where the samples are locally collinear and no three-point circle was accepted. The
    // former farthest-span comparison favored a longer line within tolerance, then emitted a
    // differently directed line for the following span, recreating visible polygonal bends after
    // successful continuous optimization.
    const auto use_arc = accepted_arc > first + 1U && accepted_line + 1U < samples.size();
    const auto accepted = use_arc ? accepted_arc : accepted_line;
    if (use_arc) {
      const auto middle = first + (accepted_arc - first) / 2U;
      result.push_back({SplineFittedPieceKind::circular_arc, quantize(samples[first].position),
                        quantize(samples[middle].position),
                        quantize(samples[accepted_arc].position), samples[first].parameter,
                        samples[accepted_arc].parameter});
    } else {
      result.push_back({SplineFittedPieceKind::segment,
                        quantize(samples[first].position),
                        {},
                        quantize(samples[accepted].position),
                        samples[first].parameter,
                        samples[accepted].parameter});
    }
    first = accepted;
  }
  return first + 1U == samples.size() ? result : std::vector<SplineFittedPiece>{};
}

SplineShapeCertificate certify_quintic_ph_shape(const QuinticPHRoute& route,
                                                const SplineShapeCertificationConfig& config) {
  SplineShapeCertificate result;
  if (route.segments.empty() || !route.tangent_continuous)
    return result;
  const auto samples =
      adaptive_tessellate_quintic_ph(route, config.tessellation_tolerance,
                                     config.maximum_tessellation_depth, config.maximum_samples);
  if (samples.size() < 2U) {
    result.sample_budget_exhausted = true;
    return result;
  }
  result.samples = samples.size();
  for (std::size_t i = 1U; i < samples.size(); ++i) {
    for (std::size_t j = i + 2U; j < samples.size(); ++j) {
      if (i == 1U && j + 1U == samples.size())
        continue;
      if (proper_segment_intersection(samples[i - 1U].position, samples[i].position,
                                      samples[j - 1U].position, samples[j].position))
        ++result.self_intersections;
    }
  }
  std::vector<SplineVector> sample_directions;
  sample_directions.reserve(samples.size() - 1U);
  for (std::size_t i = 1U; i < samples.size(); ++i)
    sample_directions.push_back(subtract(samples[i].position, samples[i - 1U].position));
  std::vector<SplineVector> anchor_directions;
  for (std::size_t i = 1U; i < route.corridor_anchors.size(); ++i)
    anchor_directions.push_back(
        subtract(route.corridor_anchors[i], route.corridor_anchors[i - 1U]));
  const auto sampled_shape = assess_directions(sample_directions);
  const auto anchor_shape = assess_directions(anchor_directions);
  result.total_turn_radians = sampled_shape.total_turn;
  result.anchor_turn_radians = anchor_shape.total_turn;
  // One bounded obstacle-avoidance bulge may require an enter/leave curvature
  // sign pair even when the source chord is straight.  More sign changes than
  // that are a genuine PH branch wiggle and fail closed.
  result.unintended_inflections = sampled_shape.inflections > anchor_shape.inflections + 2U
                                      ? sampled_shape.inflections - anchor_shape.inflections - 2U
                                      : 0U;
  const auto allowed_extra_turn =
      std::max(0.0L, config.maximum_extra_turn_radians) +
      std::max(0.0L, config.maximum_extra_turn_per_anchor_corner_radians) *
          static_cast<long double>(anchor_shape.turns);
  if (sampled_shape.total_turn > anchor_shape.total_turn + allowed_extra_turn)
    ++result.excess_turn_violations;
  result.minimum_tangent_speed_nm = std::numeric_limits<long double>::infinity();
  for (const auto& segment : route.segments) {
    // Zeros of the complex quadratic preimage are exact cusps. This includes
    // degenerate linear preimages and deduplicates a repeated quadratic root.
    result.cusps += segment.cusp_count;
    for (const auto extremum : quintic_ph_curvature_extrema(segment)) {
      const auto differential = evaluate_segment_differential(segment, extremum.parameter);
      result.minimum_tangent_speed_nm =
          std::min(result.minimum_tangent_speed_nm, norm(differential.tangent));
      if (differential.bend_radius_nm < result.minimum_bend_radius_nm) {
        result.minimum_bend_radius_nm = differential.bend_radius_nm;
        result.worst_bend_parameter =
            segment.route_parameter_start +
            extremum.parameter * (segment.route_parameter_end - segment.route_parameter_start);
      }
      if (config.minimum_bend_radius.value() > 0 &&
          differential.bend_radius_nm <
              static_cast<long double>(config.minimum_bend_radius.value()))
        ++result.bend_radius_violations;
    }
  }
  const auto station_on_anchor = [&](SplineVector point) {
    if (route.corridor_anchors.size() < 2U)
      return 0.0L;
    long double cumulative = 0.0L;
    long double best_distance = std::numeric_limits<long double>::infinity();
    long double best_station = 0.0L;
    for (std::size_t i = 1U; i < route.corridor_anchors.size(); ++i) {
      const auto first = route.corridor_anchors[i - 1U];
      const auto edge = subtract(route.corridor_anchors[i], first);
      const auto length = norm(edge);
      const auto local =
          length > 0.0L
              ? std::clamp(dot(subtract(point, first), edge) / (length * length), 0.0L, 1.0L)
              : 0.0L;
      const auto distance = norm(subtract(point, add(first, scale(edge, local))));
      if (distance < best_distance) {
        best_distance = distance;
        best_station = cumulative + local * length;
      }
      cumulative += length;
    }
    return best_station;
  };
  long double prior_control_station = -std::numeric_limits<long double>::infinity();
  for (const auto control : route.control_points) {
    const auto station = station_on_anchor(control);
    if (station + static_cast<long double>(config.station_regression_tolerance.value()) <
        prior_control_station)
      ++result.control_station_reversals;
    prior_control_station = std::max(prior_control_station, station);
  }
  long double prior_station = -std::numeric_limits<long double>::infinity();
  for (const auto& sample : samples) {
    const auto station = station_on_anchor(sample.position);
    if (station + static_cast<long double>(config.station_regression_tolerance.value()) <
        prior_station)
      ++result.station_reversals;
    prior_station = std::max(prior_station, station);
  }
  result.legal = result.violations() == 0U;
  return result;
}

SplineContinuousEnvelopeCertificate
certify_quintic_ph_continuous_envelope(const QuinticPHRoute& route, Nanometres minimum_clearance,
                                       const SplineClearanceEvaluator& clearance_evaluator,
                                       const SplineContinuousEnvelopeConfig& config) {
  SplineContinuousEnvelopeCertificate result;
  if (route.segments.empty())
    return result;
  const auto tolerance = std::max(1.0L, static_cast<long double>(config.hull_tolerance.value()));
  for (const auto& segment : route.segments) {
    std::vector<TessellationNode> stack{
        {segment.bezier, segment.route_parameter_start, segment.route_parameter_end, 0U}};
    while (!stack.empty()) {
      auto node = stack.back();
      stack.pop_back();
      if (++result.hulls_checked > config.maximum_hulls) {
        result.budget_exhausted = true;
        return result;
      }
      const auto corridor_legal = route.convex_corridor.empty() ||
                                  corridor_covers_bezier_hull(route.convex_corridor, node.controls);
      if (!corridor_legal && node.depth >= config.maximum_subdivision_depth) {
        result.worst_parameter = (node.first + node.last) * 0.5L;
        ++result.corridor_violations;
        continue;
      }
      const auto diameter = [&] {
        long double value = 0.0L;
        for (const auto a : node.controls)
          for (const auto b : node.controls)
            value = std::max(value, norm(subtract(a, b)));
        return value;
      }();
      bool clearance_legal = true;
      if (clearance_evaluator && !config.corridor_certifies_clearance) {
        const auto probe = clearance_evaluator(evaluate_bezier(node.controls, 0.5L));
        if (probe.clearance_nm < result.worst_clearance_nm) {
          result.worst_clearance_nm = probe.clearance_nm;
          result.worst_parameter = (node.first + node.last) * 0.5L;
          result.worst_normal = probe.obstacle_normal;
        }
        clearance_legal =
            probe.represented_legal &&
            probe.clearance_nm - diameter >= static_cast<long double>(minimum_clearance.value());
      }
      if (corridor_legal && clearance_legal)
        continue;
      if (node.depth >= config.maximum_subdivision_depth || diameter <= tolerance) {
        if (!corridor_legal) {
          result.worst_parameter = (node.first + node.last) * 0.5L;
          ++result.corridor_violations;
        }
        if (!clearance_legal)
          ++result.clearance_violations;
        continue;
      }
      const auto [left, right] = split_bezier(node.controls);
      const auto middle = (node.first + node.last) * 0.5L;
      stack.push_back({right, middle, node.last, node.depth + 1U});
      stack.push_back({left, node.first, middle, node.depth + 1U});
    }
  }
  result.legal = result.violations() == 0U;
  return result;
}

long double quintic_ph_curvature_objective(const QuinticPHRoute& route, std::size_t samples) {
  if (route.segments.empty() || samples < 2U)
    return std::numeric_limits<long double>::infinity();
  long double result = 0.0L;
  auto prior = evaluate_quintic_ph_differential(route, 0.0L);
  for (std::size_t i = 1U; i < samples; ++i) {
    const auto current = evaluate_quintic_ph_differential(
        route, static_cast<long double>(i) / static_cast<long double>(samples - 1U));
    const auto ds = norm(subtract(current.position, prior.position));
    result += 0.5L *
              (current.curvature_per_nm * current.curvature_per_nm +
               prior.curvature_per_nm * prior.curvature_per_nm) *
              ds;
    prior = current;
  }
  return result;
}

std::size_t shove_quintic_ph_interpolation_corridor(QuinticPHRoute& route, long double parameter,
                                                    SplineVector normal_value, Nanometres distance,
                                                    Nanometres maximum_corridor_displacement) {
  if (route.control_points.size() < 3U || !(norm(normal_value) > 0.0L) || distance.value() == 0)
    return 0U;
  normal_value = normalized(normal_value);
  if (route.node_parameters.size() != route.control_points.size())
    route.node_parameters = select_quintic_ph_parameters(route.control_points);
  std::size_t moved = 0U;
  const auto t = std::clamp(parameter, 0.0L, 1.0L);
  for (std::size_t i = 1U; i + 1U < route.control_points.size(); ++i) {
    const auto influence =
        std::max(0.0L, 1.0L - std::abs(route.node_parameters[i] - t) *
                                  static_cast<long double>(route.control_points.size() - 1U));
    if (!(influence > 0.0L))
      continue;
    route.control_points[i] = add(
        route.control_points[i],
        scale(normal_value, static_cast<long double>(distance.value()) * influence * influence));
    clamp_node_to_anchor(route, i, maximum_corridor_displacement);
    ++moved;
  }
  if (moved > 0U && !rebuild_quintic_ph_route(route))
    return 0U;
  return moved;
}

SplineTrajectoryOptimizationResult
optimize_quintic_ph_trajectory(QuinticPHRoute& route,
                               const SplineTrajectoryOptimizationConfig& config,
                               const SplineClearanceEvaluator& clearance_evaluator) {
  SplineTrajectoryOptimizationResult result;
  if (!rebuild_quintic_ph_route(route) || route.control_points.size() < 2U ||
      config.maximum_candidate_checks == 0U)
    return result;
  auto best = route;
  auto best_assessment = assess_trajectory(best, config, clearance_evaluator);
  result.initial_violations = best_assessment.violations;
  result.initial_shape_violations = best_assessment.shape_violations;
  result.initial_bend_radius_violations = best_assessment.bend_radius_violations;
  result.initial_sampled_clearance_violations = best_assessment.sampled_clearance_violations;
  result.initial_objective = best_assessment.objective;
  const auto initial_envelope = certify_quintic_ph_continuous_envelope(
      best, config.minimum_clearance, clearance_evaluator, config.continuous_envelope);
  result.initial_continuous_violations = initial_envelope.violations();
  if (config.prefer_gpu_preconditioner && config.gpu_precondition_iterations > 0U &&
      best.control_points.size() >= 3U &&
      best.control_points.size() <= std::numeric_limits<std::uint32_t>::max()) {
    result.gpu_precondition_attempted = true;
    std::vector<RoutingXpbdParticle> particles;
    std::vector<RoutingXpbdSegment> segments;
    particles.reserve(best.control_points.size());
    segments.reserve(best.control_points.size() - 1U);
    for (std::size_t point = 0U; point < best.control_points.size(); ++point) {
      const auto endpoint = point == 0U || point + 1U == best.control_points.size();
      const auto anchor = point < best.corridor_anchors.size() ? best.corridor_anchors[point]
                                                               : best.control_points[point];
      particles.push_back(RoutingXpbdParticle{
          static_cast<std::int64_t>(std::llround(best.control_points[point].x)),
          static_cast<std::int64_t>(std::llround(best.control_points[point].y)), 0,
          endpoint ? 0.0L : 1.0L, 0U, 0U, endpoint ? routing_xpbd_endpoint : 0U,
          static_cast<std::int64_t>(std::llround(anchor.x)),
          static_cast<std::int64_t>(std::llround(anchor.y)),
          std::max<std::int64_t>(1, config.maximum_corridor_displacement.value())});
      if (point > 0U) {
        // A rubber band is under tension; preserving every sampled zigzag span exactly gives the
        // bend solver no route-shortening degree of freedom. A bounded contracted rest length
        // supplies that tension while corridor clamps and the CPU comparator reject shortcuts
        // that change topology or lose clearance.
        const auto rest = static_cast<std::int64_t>(std::llround(
            norm(subtract(best.control_points[point], best.control_points[point - 1U])) * 0.85L));
        segments.push_back(RoutingXpbdSegment{static_cast<std::uint32_t>(point - 1U),
                                              static_cast<std::uint32_t>(point), 0U, 0U, 0,
                                              std::max<std::int64_t>(1, rest), 0});
      }
    }
    RoutingXpbdConfig xpbd_config;
    xpbd_config.iterations = config.gpu_precondition_iterations;
    // Weak stretch plus strong bending acts as an elastic band: preserve useful station spacing
    // while allowing the full chain to pull a sampled staircase taut in one resident dispatch.
    xpbd_config.stretch_compliance = 1.0e-4L;
    xpbd_config.bend_compliance = 1.0e-6L;
    xpbd_config.clearance_compliance = 1.0L;
    xpbd_config.corridor_compliance = 1.0e-7L;
    xpbd_config.maximum_step_nm = std::max<std::int64_t>(1, config.control_step.value());
    xpbd_config.numerical_guard_nm = 0;
    xpbd_config.maximum_candidate_pairs = 1U;
    xpbd_config.record_iteration_metrics = false;
    xpbd_config.prefer_gpu = true;
    const auto preconditioned = relax_routing_xpbd(particles, segments, {}, {}, xpbd_config);
    result.gpu_precondition_used_gpu = preconditioned.used_gpu;
    result.gpu_precondition_cpu_fallback = preconditioned.cpu_fallback;
    result.gpu_precondition_solve_us = preconditioned.timings.solve_us;
    if (preconditioned.ok() && preconditioned.positions_nm.size() == best.control_points.size()) {
      auto candidate = best;
      for (std::size_t point = 1U; point + 1U < candidate.control_points.size(); ++point) {
        candidate.control_points[point] = {
            static_cast<long double>(preconditioned.positions_nm[point].first),
            static_cast<long double>(preconditioned.positions_nm[point].second)};
        clamp_node_to_anchor(candidate, point, config.maximum_corridor_displacement);
      }
      if (rebuild_quintic_ph_route(candidate)) {
        const auto assessment = assess_trajectory(candidate, config, clearance_evaluator);
        const auto envelope = certify_quintic_ph_continuous_envelope(
            candidate, config.minimum_clearance, clearance_evaluator, config.continuous_envelope);
        const auto candidate_violations = assessment.violations + envelope.violations();
        const auto best_violations = best_assessment.violations + initial_envelope.violations();
        if (candidate_violations < best_violations ||
            (candidate_violations == best_violations &&
             assessment.objective + 1.0e-12L < best_assessment.objective)) {
          best = std::move(candidate);
          best_assessment = assessment;
          result.gpu_precondition_accepted = true;
          result.gpu_precondition_legal = candidate_violations == 0U;
        }
      }
    }
  }
  // Parameterization and tangent magnitude are part of PH interpolation, not
  // cosmetic post-processing. Explore a small deterministic portfolio before
  // adding degrees of freedom: a longer tangent generally relaxes a tight
  // bend, while chord-length parameters redistribute freedom on unequal spans.
  struct InterpolationStart {
    long double exponent;
    long double tangent_scale;
  };
  const std::array<InterpolationStart, 4U> interpolation_starts{
      {{best.parameter_exponent, best.tangent_scale * 1.5L},
       {1.0L, best.tangent_scale},
       {best.parameter_exponent, best.tangent_scale * 2.0L},
       {1.0L, best.tangent_scale * 2.5L}}};
  for (const auto start : interpolation_starts) {
    if (result.multistart_attempts >= config.maximum_multistart_attempts)
      break;
    ++result.multistart_attempts;
    auto candidate = route;
    candidate.parameter_exponent = start.exponent;
    candidate.tangent_scale = start.tangent_scale;
    if (!rebuild_quintic_ph_route(candidate))
      continue;
    const auto assessment = assess_trajectory(candidate, config, clearance_evaluator);
    const auto envelope = certify_quintic_ph_continuous_envelope(
        candidate, config.minimum_clearance, clearance_evaluator, config.continuous_envelope);
    const auto candidate_violations = assessment.violations + envelope.violations();
    const auto best_envelope = certify_quintic_ph_continuous_envelope(
        best, config.minimum_clearance, clearance_evaluator, config.continuous_envelope);
    const auto best_violations = best_assessment.violations + best_envelope.violations();
    if (candidate_violations < best_violations ||
        (candidate_violations == best_violations &&
         assessment.objective + 1.0e-12L < best_assessment.objective)) {
      if (best_violations > 0U && candidate_violations == 0U)
        ++result.multistart_recoveries;
      best = std::move(candidate);
      best_assessment = assessment;
    }
  }
  // A PH segment's worst bend is known analytically from the cached roots of
  // its curvature-stationary polynomial. Add local interpolation freedom at
  // that precise route parameter before coordinate descent; do not wait for a
  // tessellation sample to happen to land near the extremum.
  while (best_assessment.bend_radius_violations > 0U &&
         result.adaptive_controls_inserted < config.maximum_adaptive_control_insertions) {
    auto parameter = std::clamp(best_assessment.worst_bend_parameter, 0.05L, 0.95L);
    if (std::any_of(best.node_parameters.begin(), best.node_parameters.end(),
                    [&](auto existing) { return std::abs(existing - parameter) < 0.025L; })) {
      parameter = std::clamp(parameter + (parameter < 0.5L ? 0.075L : -0.075L), 0.05L, 0.95L);
    }
    auto candidate = best;
    if (!insert_quintic_ph_interpolation_node(candidate, parameter))
      break;
    const auto candidate_assessment = assess_trajectory(candidate, config, clearance_evaluator);
    ++result.adaptive_controls_inserted;
    if (candidate_assessment.violations < best_assessment.violations ||
        (candidate_assessment.violations == best_assessment.violations &&
         candidate_assessment.objective < best_assessment.objective)) {
      best = std::move(candidate);
      best_assessment = candidate_assessment;
    } else {
      break;
    }
  }
  auto adaptive_envelope = certify_quintic_ph_continuous_envelope(
      best, config.minimum_clearance, clearance_evaluator, config.continuous_envelope);
  while (adaptive_envelope.violations() > 0U &&
         result.adaptive_controls_inserted < config.maximum_adaptive_control_insertions) {
    auto parameter = std::clamp(adaptive_envelope.worst_parameter, 0.05L, 0.95L);
    if (std::any_of(best.node_parameters.begin(), best.node_parameters.end(),
                    [&](auto existing) { return std::abs(existing - parameter) < 0.025L; })) {
      parameter = std::clamp(parameter + (parameter < 0.5L ? 0.1L : -0.1L), 0.05L, 0.95L);
    }
    if (!insert_quintic_ph_interpolation_node(best, parameter))
      break;
    ++result.adaptive_controls_inserted;
    best_assessment = assess_trajectory(best, config, clearance_evaluator);
    adaptive_envelope = certify_quintic_ph_continuous_envelope(
        best, config.minimum_clearance, clearance_evaluator, config.continuous_envelope);
  }
  auto best_continuous_envelope = adaptive_envelope;
  auto step = std::max<long double>(1.0L, static_cast<long double>(config.control_step.value()));
  // A legal GPU elastic solution needs only bounded CPU objective polishing. An improved but
  // still illegal proposal retains the full CPU recovery budget.
  const auto refinement_iterations = result.gpu_precondition_legal
                                         ? std::min<std::size_t>(config.maximum_iterations, 8U)
                                         : config.maximum_iterations;
  BoundedWorkerPool exact_candidate_pool(std::max<std::size_t>(1U, config.exact_candidate_workers));
  RoutingComputeSplineScoreInput score_template;
  if (config.prefer_gpu_polishing) {
    long double minimum_x = best.control_points.front().x;
    long double maximum_x = minimum_x;
    long double minimum_y = best.control_points.front().y;
    long double maximum_y = minimum_y;
    const auto include = [&](SplineVector point) {
      minimum_x = std::min(minimum_x, point.x);
      maximum_x = std::max(maximum_x, point.x);
      minimum_y = std::min(minimum_y, point.y);
      maximum_y = std::max(maximum_y, point.y);
    };
    for (const auto point : best.control_points)
      include(point);
    for (const auto& polygon : best.convex_corridor)
      for (const auto point : polygon.vertices)
        include(SplineVector{static_cast<long double>(point.x.value()),
                             static_cast<long double>(point.y.value())});
    const auto margin = static_cast<long double>(
        std::max(config.maximum_corridor_displacement, config.clearance_influence_distance)
            .value());
    minimum_x -= margin;
    minimum_y -= margin;
    maximum_x += margin;
    maximum_y += margin;
    constexpr std::size_t maximum_field_dimension = 128U;
    const auto pitch = std::max<long double>(
        10'000.0L, std::ceil(std::max(maximum_x - minimum_x, maximum_y - minimum_y) /
                             static_cast<long double>(maximum_field_dimension - 1U)));
    score_template.field_pitch_nm = static_cast<std::int64_t>(pitch);
    score_template.field_origin_x_nm = static_cast<std::int64_t>(std::floor(minimum_x));
    score_template.field_origin_y_nm = static_cast<std::int64_t>(std::floor(minimum_y));
    score_template.field_width = std::max<std::size_t>(
        1U, std::min(maximum_field_dimension,
                     static_cast<std::size_t>(std::ceil((maximum_x - minimum_x) / pitch)) + 1U));
    score_template.field_height = std::max<std::size_t>(
        1U, std::min(maximum_field_dimension,
                     static_cast<std::size_t>(std::ceil((maximum_y - minimum_y) / pitch)) + 1U));
    const auto field_cells = score_template.field_width * score_template.field_height;
    score_template.signed_clearance_nm.resize(field_cells);
    score_template.corridor_mask.resize(field_cells, best.convex_corridor.empty() ? 1U : 0U);
    for (std::size_t cell = 0U; cell < field_cells; ++cell) {
      const auto x = cell % score_template.field_width;
      const auto y = cell / score_template.field_width;
      const auto point = SplineVector{
          static_cast<long double>(score_template.field_origin_x_nm) + (x + 0.5L) * pitch,
          static_cast<long double>(score_template.field_origin_y_nm) + (y + 0.5L) * pitch};
      if (clearance_evaluator) {
        const auto probe = clearance_evaluator(point);
        score_template.signed_clearance_nm[cell] = static_cast<float>(
            probe.represented_legal ? probe.clearance_nm : -std::abs(probe.clearance_nm));
      } else {
        score_template.signed_clearance_nm[cell] = std::numeric_limits<float>::max();
      }
      if (!best.convex_corridor.empty()) {
        score_template.corridor_mask[cell] =
            std::any_of(best.convex_corridor.begin(), best.convex_corridor.end(),
                        [&](const auto& polygon) { return point_in_convex(polygon, point); })
                ? 1U
                : 0U;
      }
    }
    score_template.minimum_clearance_nm = config.minimum_clearance.value();
    score_template.minimum_bend_radius_nm = config.minimum_bend_radius.value();
    score_template.clearance_influence_nm = config.clearance_influence_distance.value();
    score_template.objective_weights = {static_cast<float>(config.weights.length),
                                        static_cast<float>(config.weights.curvature),
                                        static_cast<float>(config.weights.curvature_change),
                                        static_cast<float>(config.weights.bend_radius),
                                        static_cast<float>(config.weights.clearance)};
  }
  for (std::size_t iteration = 0U; iteration < refinement_iterations; ++iteration) {
    bool improved = false;
    ++result.iterations;
    std::vector<QuinticPHRoute> candidate_routes;
    std::vector<std::size_t> candidate_control_nodes;
    for (std::size_t node = 1U; node + 1U < best.control_points.size(); ++node) {
      struct MoveProposal {
        SplineVector direction;
        long double step_scale{1.0L};
      };
      std::vector<MoveProposal> proposals{{{1, 0}}, {{-1, 0}}, {{0, 1}},  {{0, -1}},
                                          {{1, 1}}, {{1, -1}}, {{-1, 1}}, {{-1, -1}}};
      const auto current = best.control_points[node];
      const auto previous = best.control_points[node - 1U];
      const auto next = best.control_points[node + 1U];
      // These are the actual rubber-band moves. Axis probes alone can polish a control point but
      // are poor at removing a lattice staircase: the shortening gradient pulls the two incident
      // spans taut, while the Laplacian and biharmonic directions reduce curvature and curvature
      // change across several controls. The configured preconditioners affect proposal distance,
      // not the authoritative objective or legality comparator.
      const auto shortening =
          add(normalized(subtract(previous, current)), normalized(subtract(next, current)));
      if (norm(shortening) > 1.0e-12L)
        proposals.push_back({shortening, 1.5L});
      const auto laplacian = subtract(scale(add(previous, next), 0.5L), current);
      if (norm(laplacian) > 1.0e-12L) {
        proposals.push_back(
            {laplacian, std::clamp(config.second_difference_preconditioner / 8.0L, 0.25L, 2.0L)});
      }
      if (node >= 2U && node + 2U < best.control_points.size()) {
        const auto biharmonic = scale(
            add(add(scale(best.control_points[node - 1U], 4.0L),
                    scale(best.control_points[node + 1U], 4.0L)),
                scale(add(best.control_points[node - 2U], best.control_points[node + 2U]), -1.0L)),
            1.0L / 6.0L);
        const auto fairing = subtract(biharmonic, current);
        if (norm(fairing) > 1.0e-12L) {
          proposals.push_back(
              {fairing, std::clamp(config.third_difference_preconditioner / 2.0L, 0.25L, 1.5L)});
        }
      }
      if (clearance_evaluator) {
        const auto probe = clearance_evaluator(best.control_points[node]);
        if (norm(probe.obstacle_normal) > 0.0L)
          proposals.push_back({normalized(probe.obstacle_normal), 1.0L});
      }
      for (const auto& proposal : proposals) {
        if (result.candidate_checks++ >= config.maximum_candidate_checks) {
          result.budget_exhausted = true;
          break;
        }
        auto candidate = best;
        candidate.control_points[node] =
            add(candidate.control_points[node],
                scale(normalized(proposal.direction), step * proposal.step_scale));
        clamp_node_to_anchor(candidate, node, config.maximum_corridor_displacement);
        // The GPU ranker needs a geometrically responsive proxy, not the exact algebraic caches.
        // Deform only the two adjacent Bézier spans here. Rebuilding PH interpolation branches,
        // curvature extrema and bounds for every rejected proposal was the dominant CPU cost.
        const auto displacement = subtract(candidate.control_points[node], current);
        if (node - 1U < candidate.segments.size()) {
          auto& segment = candidate.segments[node - 1U];
          for (std::size_t control = 0U; control < segment.bezier.size(); ++control)
            segment.bezier[control] =
                add(segment.bezier[control],
                    scale(displacement, static_cast<long double>(control) /
                                            static_cast<long double>(segment.bezier.size() - 1U)));
        }
        if (node < candidate.segments.size()) {
          auto& segment = candidate.segments[node];
          for (std::size_t control = 0U; control < segment.bezier.size(); ++control)
            segment.bezier[control] =
                add(segment.bezier[control],
                    scale(displacement,
                          1.0L - static_cast<long double>(control) /
                                     static_cast<long double>(segment.bezier.size() - 1U)));
        }
        candidate_routes.push_back(std::move(candidate));
        candidate_control_nodes.push_back(node);
      }
      if (result.budget_exhausted)
        break;
    }
    std::vector<std::size_t> exact_candidates(candidate_routes.size());
    std::iota(exact_candidates.begin(), exact_candidates.end(), 0U);
    if (config.prefer_gpu_polishing && config.gpu_polish_shortlist > 0U &&
        candidate_routes.size() > config.gpu_polish_shortlist && score_template.field_width > 0U) {
      ++result.gpu_polish_batches;
      result.gpu_polish_candidates += candidate_routes.size();
      auto batch = score_template;
      batch.sample_count = config.samples;
      batch.candidate_count = candidate_routes.size();
      batch.top_k = std::min<std::size_t>(candidate_routes.size(),
                                          std::max<std::size_t>(config.gpu_polish_shortlist, 8U));
      const auto controls_per_candidate =
          quintic_ph_bezier_controls(candidate_routes.front()).size();
      bool batch_valid = controls_per_candidate > 0U;
      batch.control_points_nm.reserve(controls_per_candidate * candidate_routes.size());
      batch.basis_coefficients.reserve(controls_per_candidate * config.samples *
                                       candidate_routes.size());
      for (const auto& candidate : candidate_routes) {
        const auto controls = quintic_ph_bezier_controls(candidate);
        if (controls.size() != controls_per_candidate) {
          batch_valid = false;
          break;
        }
        for (const auto control : controls) {
          batch.control_points_nm.emplace_back(static_cast<std::int64_t>(std::llround(control.x)),
                                               static_cast<std::int64_t>(std::llround(control.y)));
        }
        for (std::size_t sample = 0U; sample < config.samples; ++sample) {
          const auto parameter =
              static_cast<long double>(sample) / static_cast<long double>(config.samples - 1U);
          const auto basis = quintic_ph_bezier_basis(candidate, parameter);
          if (basis.size() != controls_per_candidate) {
            batch_valid = false;
            break;
          }
          batch.basis_coefficients.insert(batch.basis_coefficients.end(), basis.begin(),
                                          basis.end());
        }
        if (!batch_valid)
          break;
      }
      const auto evaluated = batch_valid ? score_routing_spline_candidates(batch, true)
                                         : RoutingComputeSplineScoreResult{};
      if (evaluated.ok()) {
        ++result.gpu_polish_dispatches;
        exact_candidates.clear();
        exact_candidates.reserve(evaluated.shortlist.size());
        for (const auto& ranked : evaluated.shortlist)
          exact_candidates.push_back(ranked.candidate);
        result.gpu_polish_exact_shortlist_checks += exact_candidates.size();
      } else if (batch_valid) {
        ++result.gpu_polish_cpu_fallbacks;
      }
    }
    struct ExactCandidateEvaluation {
      TrajectoryAssessment assessment;
      SplineContinuousEnvelopeCertificate continuous_envelope;
    };
    std::vector<std::optional<ExactCandidateEvaluation>> exact_evaluations(exact_candidates.size());
    if (config.exact_candidate_workers > 1U && exact_candidates.size() > 1U) {
      ++result.parallel_exact_waves;
      result.parallel_exact_checks += exact_candidates.size();
    }
    exact_candidate_pool.run(exact_candidates.size(), [&](std::size_t slot) {
      const auto candidate_index = exact_candidates[slot];
      auto& candidate = candidate_routes[candidate_index];
      if (!rebuild_quintic_ph_route_around_control(
              candidate, candidate_control_nodes[candidate_index]))
        return;
      exact_evaluations[slot] = ExactCandidateEvaluation{
          assess_trajectory(candidate, config, clearance_evaluator),
          certify_quintic_ph_continuous_envelope(candidate, config.minimum_clearance,
                                                 clearance_evaluator, config.continuous_envelope)};
    });
    std::optional<std::size_t> selected_slot;
    for (std::size_t slot = 0U; slot < exact_candidates.size(); ++slot) {
      if (!exact_evaluations[slot])
        continue;
      const auto& assessment = exact_evaluations[slot]->assessment;
      const auto& continuous_envelope = exact_evaluations[slot]->continuous_envelope;
      const auto candidate_violations = assessment.violations + continuous_envelope.violations();
      const auto best_violations =
          best_assessment.violations + best_continuous_envelope.violations();
      const auto better = candidate_violations < best_violations ||
                          (candidate_violations == best_violations &&
                           assessment.objective + 1.0e-12L < best_assessment.objective);
      if (!better)
        continue;
      if (!selected_slot ||
          assessment.violations + continuous_envelope.violations() <
              exact_evaluations[*selected_slot]->assessment.violations +
                  exact_evaluations[*selected_slot]->continuous_envelope.violations() ||
          (assessment.violations + continuous_envelope.violations() ==
               exact_evaluations[*selected_slot]->assessment.violations +
                   exact_evaluations[*selected_slot]->continuous_envelope.violations() &&
           assessment.objective < exact_evaluations[*selected_slot]->assessment.objective))
        selected_slot = slot;
    }
    if (selected_slot) {
      best = std::move(candidate_routes[exact_candidates[*selected_slot]]);
      best_assessment = exact_evaluations[*selected_slot]->assessment;
      best_continuous_envelope = exact_evaluations[*selected_slot]->continuous_envelope;
      ++result.accepted_control_moves;
      improved = true;
    }
    if (result.budget_exhausted)
      break;
    if (!improved) {
      step *= 0.5L;
      if (step < 1.0L)
        break;
    }
  }
  route = best;
  auto final_shape_config = config.shape;
  final_shape_config.minimum_bend_radius = config.minimum_bend_radius;
  const auto final_shape = certify_quintic_ph_shape(route, final_shape_config);
  const auto final_envelope = certify_quintic_ph_continuous_envelope(
      route, config.minimum_clearance, clearance_evaluator, config.continuous_envelope);
  result.final_violations = best_assessment.violations + final_envelope.violations();
  result.final_shape_violations = final_shape.violations();
  result.final_self_intersections = final_shape.self_intersections;
  result.final_cusps = final_shape.cusps;
  result.final_control_station_reversals = final_shape.control_station_reversals;
  result.final_station_reversals = final_shape.station_reversals;
  result.final_excess_turn_violations = final_shape.excess_turn_violations;
  result.final_unintended_inflections = final_shape.unintended_inflections;
  result.final_bend_radius_violations = final_shape.bend_radius_violations;
  result.final_sampled_clearance_violations = best_assessment.sampled_clearance_violations;
  result.final_continuous_violations = final_envelope.violations();
  result.final_continuous_corridor_violations = final_envelope.corridor_violations;
  result.final_continuous_clearance_violations = final_envelope.clearance_violations;
  result.continuous_hulls_checked = final_envelope.hulls_checked;
  result.continuous_budget_exhausted = final_envelope.budget_exhausted;
  result.final_objective = best_assessment.objective;
  result.legal = best_assessment.violations == 0U && final_shape.legal && final_envelope.legal;
  result.accepted = result.legal;
  return result;
}

SplineTrajectoryOptimizationResult
deform_quintic_ph_elastic(QuinticPHRoute& route, SplineVector new_start, SplineVector new_end,
                          const SplineTrajectoryOptimizationConfig& config,
                          const SplineClearanceEvaluator& clearance_evaluator) {
  SplineTrajectoryOptimizationResult result;
  if (route.control_points.size() < 2U)
    return result;
  const auto old_start = route.control_points.front();
  const auto old_end = route.control_points.back();
  const auto start_delta = subtract(new_start, old_start);
  const auto end_delta = subtract(new_end, old_end);
  for (std::size_t i = 0U; i < route.control_points.size(); ++i) {
    const auto t =
        static_cast<long double>(i) / static_cast<long double>(route.control_points.size() - 1U);
    const auto displacement =
        add(scale(start_delta, std::pow(1.0L - t, 3.0L)), scale(end_delta, std::pow(t, 3.0L)));
    route.control_points[i] = add(route.control_points[i], displacement);
    if (norm(displacement) > 0.0L)
      ++result.elastic_predeformed_controls;
  }
  route.control_points.front() = new_start;
  route.control_points.back() = new_end;
  if (!route.corridor_anchors.empty()) {
    route.corridor_anchors.front() = new_start;
    route.corridor_anchors.back() = new_end;
  }
  if (!rebuild_quintic_ph_route(route))
    return {};
  auto optimized = optimize_quintic_ph_trajectory(route, config, clearance_evaluator);
  optimized.elastic_predeformed_controls += result.elastic_predeformed_controls;
  return optimized;
}

SplineTrajectoryOptimizationResult
deform_quintic_ph_elastic_at(QuinticPHRoute& route, long double collision_parameter,
                             SplineVector obstacle_normal, Nanometres displacement,
                             const SplineTrajectoryOptimizationConfig& config,
                             const SplineClearanceEvaluator& clearance_evaluator) {
  const auto moved =
      shove_quintic_ph_interpolation_corridor(route, collision_parameter, obstacle_normal,
                                              displacement, config.maximum_corridor_displacement);
  if (moved == 0U)
    return {};
  auto result = optimize_quintic_ph_trajectory(route, config, clearance_evaluator);
  result.elastic_predeformed_controls += moved;
  return result;
}

} // namespace flux
