#include "flux/core/routing_compute.hpp"
#include "flux/core/spline_route.hpp"

#include "flux/core/cpu_worker_budget.hpp"
#include "flux/core/geometry.hpp"
#include "flux/core/routing_kernel.hpp"

#include <boost/multiprecision/cpp_dec_float.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <complex>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <mutex>
#include <map>
#include <set>
#include <tuple>
#include <numeric>
#include <optional>
#include <stop_token>
#include <thread>
#include <utility>
#include <vector>

#if __has_include(<experimental/simd>)
#include <experimental/simd>
#define FLUX_HAS_EXPERIMENTAL_SIMD 1
#else
#define FLUX_HAS_EXPERIMENTAL_SIMD 0
#endif

namespace flux {

struct PreparedSplineCorridor {
  struct Bounds {
    long double minimum_x{0.0L};
    long double minimum_y{0.0L};
    long double maximum_x{0.0L};
    long double maximum_y{0.0L};
  };
  std::vector<Polygon> regions;
  std::vector<Bounds> bounds;
};

bool QuinticPHRoute::operator==(const QuinticPHRoute& other) const {
  const auto corridors_equal = [&] {
    if (prepared_corridor == other.prepared_corridor)
      return true;
    if (!prepared_corridor || !other.prepared_corridor)
      return false;
    // Bounds are a deterministic cache derived from regions.  Comparing only the source regions
    // avoids pointer identity becoming part of a route's semantic state.
    return prepared_corridor->regions == other.prepared_corridor->regions;
  };
  return control_points == other.control_points && corridor_anchors == other.corridor_anchors &&
         node_parameters == other.node_parameters && node_derivatives == other.node_derivatives &&
         segments == other.segments && corridors_equal() &&
         parameter_exponent == other.parameter_exponent && tangent_scale == other.tangent_scale &&
         requested_continuity == other.requested_continuity &&
         tangent_continuous == other.tangent_continuous &&
         curvature_continuous == other.curvature_continuous &&
         node_tangent_directions == other.node_tangent_directions;
}

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
    {
      // Serialize the stop predicate with wait's check/unlock transition. Otherwise the
      // notification can precede a worker entering wait and leave shutdown blocked forever.
      std::lock_guard lock(mutex_);
      for (auto& worker : workers_)
        worker.request_stop();
    }
    wake_.notify_all();
    // Members are destroyed in reverse declaration order: explicitly join before the
    // condition variables, task storage and mutex that the workers still reference.
    workers_.clear();
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
  // Beautiful routing is a fairness problem, not merely an endpoint-interpolation problem.
  // Prefer PH branches whose curvature changes monotonically (a spiral) or has at most the one
  // interior extremum needed by a fair C-shaped transition. The scale-normalized total variation
  // also distinguishes equally regular algebraic branches without changing any hard certificate.
  constexpr std::size_t fairness_samples = 24U;
  auto prior_curvature = evaluate_preimage_kinematics(segment, 0.0L).curvature;
  long double curvature_variation = 0.0L;
  for (std::size_t sample = 1U; sample <= fairness_samples; ++sample) {
    const auto curvature =
        evaluate_preimage_kinematics(segment, static_cast<long double>(sample) /
                                                  static_cast<long double>(fairness_samples))
            .curvature;
    curvature_variation += std::abs(curvature - prior_curvature);
    prior_curvature = curvature;
  }
  const auto interior_extrema = static_cast<std::size_t>(std::count_if(
      segment.curvature_extremum_parameters.begin(),
      segment.curvature_extremum_parameters.begin() + segment.curvature_extremum_count,
      [](long double parameter) { return parameter > 0.0L && parameter < 1.0L; }));
  const auto excess_curvature_extrema = interior_extrema > 1U ? interior_extrema - 1U : 0U;
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
         static_cast<long double>(excess_curvature_extrema) * chord_length * 20.0L +
         curvature_variation * chord_length * chord_length * 4.0L +
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
    const auto tangent_scale_value = std::max({1.0L, norm(left.tangent), norm(right.tangent)});
    route.tangent_continuous =
        route.tangent_continuous &&
        norm(subtract(left.tangent, right.tangent)) <= tangent_scale_value * 1.0e-9L;
    route.curvature_continuous =
        route.curvature_continuous &&
        std::abs(left.curvature_per_nm - right.curvature_per_nm) <= 1.0e-10L;
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
  if (!route.prepared_corridor)
    return;
  if (std::any_of(
          route.prepared_corridor->regions.begin(), route.prepared_corridor->regions.end(),
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
  if (route.control_points.size() < 2U ||
      (!route.node_tangent_directions.empty() &&
       route.node_tangent_directions.size() != route.control_points.size()))
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
    if (!route.node_tangent_directions.empty()) {
      const auto direction = route.node_tangent_directions[i];
      if (!std::isfinite(direction.x) || !std::isfinite(direction.y))
        return false;
      if (norm(direction) > 0.0L)
        route.node_derivatives[i] = scale(normalized(direction), norm(route.node_derivatives[i]));
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

bool rebuild_quintic_ph_route_around_control(QuinticPHRoute& route, std::size_t control_index) {
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
                                 const PHInterpolationConfig& config,
                                 std::vector<SplineVector> tangent_directions = {}) {
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
  route.node_tangent_directions = std::move(tangent_directions);
  if (!rebuild_quintic_ph_route(route))
    return std::nullopt;
  if (config.continuity == PHJoinContinuity::curvature &&
      !enforce_quintic_ph_continuity(route, config.continuity,
                                     config.curvature_tolerance_per_nm,
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
                              const PHInterpolationConfig& config, bool retain_corner,
                              bool align_runs = false) {
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
  if (!align_runs)
    return make_quintic_ph_route_from_nodes(guarded, config);
  std::vector<SplineVector> tangent_directions;
  tangent_directions.reserve(guarded.size());
  const auto direction = [](Point a, Point b) {
    return normalized(SplineVector{static_cast<long double>(b.x.value() - a.x.value()),
                                    static_cast<long double>(b.y.value() - a.y.value())});
  };
  tangent_directions.push_back(direction(canonical[0], canonical[1]));
  for (std::size_t i = 1U; i + 1U < canonical.size(); ++i) {
    tangent_directions.push_back(direction(canonical[i - 1U], canonical[i]));
    if (retain_corner)
      tangent_directions.push_back({});
    tangent_directions.push_back(direction(canonical[i], canonical[i + 1U]));
  }
  tangent_directions.push_back(
      direction(canonical[canonical.size() - 2U], canonical.back()));
  return make_quintic_ph_route_from_nodes(guarded, config, std::move(tangent_directions));
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

std::optional<QuinticPHRoute>
make_run_aligned_quintic_ph_route(const std::vector<Point>& polyline, Nanometres corner_guard,
                                 const PHInterpolationConfig& config) {
  return make_guarded_quintic_ph_route(polyline, corner_guard, config, false, true);
}

long double quintic_ph_max_curvature_jump(const QuinticPHRoute& route) {
  long double result = 0.0L;
  for (std::size_t i = 1U; i < route.segments.size(); ++i) {
    const auto left = evaluate_segment_differential(route.segments[i - 1U], 1.0L).curvature_per_nm;
    const auto right = evaluate_segment_differential(route.segments[i], 0.0L).curvature_per_nm;
    if (!std::isfinite(left) || !std::isfinite(right))
      return std::numeric_limits<long double>::infinity();
    result = std::max(result, std::abs(left - right));
  }
  return result;
}

bool enforce_quintic_ph_continuity(QuinticPHRoute& output, PHJoinContinuity continuity,
                                   long double curvature_tolerance_per_nm,
                                   std::size_t maximum_iterations, std::stop_token stop_token) {
  // Promotion is optional. Bound work independently of a caller's route/sample budget and
  // leave the original candidate intact on cancellation or an oversized request.
  if (stop_token.stop_requested() || output.control_points.size() > 130U ||
      !std::isfinite(curvature_tolerance_per_nm))
    return false;
  auto route = output;
  if (!rebuild_quintic_ph_route(route))
    return false;
  route.requested_continuity = continuity;
  if (continuity == PHJoinContinuity::tangent || route.segments.size() <= 1U) {
    output = std::move(route);
    return true;
  }
  // Bounded nonlinear relaxation of shared nodal derivatives. Every interpolation point and G1
  // join is invariant; only the derivative shared by adjacent Hermite segments moves. G2 is a
  // geometric condition, so minimize the scalar curvature jump—not the parameter-dependent
  // second-derivative vector that would incorrectly demand full parametric C2 continuity.
  auto best = route;
  const auto mismatch = quintic_ph_max_curvature_jump;
  auto best_mismatch = mismatch(best);
  const auto tolerance = std::max(0.0L, curvature_tolerance_per_nm);
  const auto residuals = [](const QuinticPHRoute& value) {
    std::vector<long double> values;
    values.reserve(value.segments.size() > 0U ? value.segments.size() - 1U : 0U);
    for (std::size_t join = 1U; join < value.segments.size(); ++join) {
      values.push_back(
          evaluate_segment_differential(value.segments[join - 1U], 1.0L).curvature_per_nm -
          evaluate_segment_differential(value.segments[join], 0.0L).curvature_per_nm);
    }
    return values;
  };
  const auto solve = [](std::vector<std::array<long double, 3U>> matrix,
                        std::vector<long double> rhs) -> std::optional<std::vector<long double>> {
    const auto size = rhs.size();
    // Each residual involves two spans and three neighboring nodal magnitudes. Use a
    // tridiagonal solve, declining ill-conditioned pivots rather than amplifying noise.
    for (std::size_t row = 0U; row < size; ++row) {
      if (row > 0U) {
        const auto factor = matrix[row][0U] / matrix[row - 1U][1U];
        matrix[row][1U] -= factor * matrix[row - 1U][2U];
        rhs[row] -= factor * rhs[row - 1U];
      }
      if (!std::isfinite(matrix[row][1U]) || std::abs(matrix[row][1U]) <= 1.0e-30L)
        return std::nullopt;
    }
    for (std::size_t row = size; row-- > 0U;) {
      if (row + 1U < size)
        rhs[row] -= matrix[row][2U] * rhs[row + 1U];
      rhs[row] /= matrix[row][1U];
      if (!std::isfinite(rhs[row]))
        return std::nullopt;
    }
    return rhs;
  };
  const auto joins = route.segments.size() - 1U;
  for (std::size_t iteration = 0U;
       iteration < std::min<std::size_t>(maximum_iterations, 24U) && best_mismatch > tolerance;
       ++iteration) {
    routing_kernel_checkpoint();
    if (stop_token.stop_requested())
      return false;
    const auto base_residuals = residuals(best);
    std::vector<SplineVector> directions(joins);
    std::vector<long double> magnitudes(joins);
    std::vector<std::array<long double, 3U>> jacobian(joins);
    bool jacobian_valid = true;
    auto candidate = best;
    for (std::size_t variable = 0U; variable < joins; ++variable) {
      if (stop_token.stop_requested())
        return false;
      const auto node = variable + 1U;
      magnitudes[variable] = norm(best.node_derivatives[node]);
      directions[variable] = normalized(best.node_derivatives[node]);
      const auto probe = std::max(1.0L, magnitudes[variable] * 1.0e-4L);
      candidate.node_derivatives[node] = scale(directions[variable], magnitudes[variable] + probe);
      for (const auto span_index : {node - 1U, node}) {
        const auto interval =
            best.node_parameters[span_index + 1U] - best.node_parameters[span_index];
        auto span = interpolate_quintic_ph_segment(
            best.control_points[span_index], best.control_points[span_index + 1U],
            scale(candidate.node_derivatives[span_index], interval),
            scale(candidate.node_derivatives[span_index + 1U], interval));
        if (!span) {
          jacobian_valid = false;
          break;
        }
        span->route_parameter_start = best.node_parameters[span_index];
        span->route_parameter_end = best.node_parameters[span_index + 1U];
        candidate.segments[span_index] = *span;
      }
      if (!jacobian_valid)
        break;
      for (std::size_t equation = variable > 0U ? variable - 1U : 0U;
           equation < std::min(joins, variable + 2U); ++equation) {
        const auto residual =
            evaluate_segment_differential(candidate.segments[equation], 1.0L).curvature_per_nm -
            evaluate_segment_differential(candidate.segments[equation + 1U], 0.0L).curvature_per_nm;
        jacobian[equation][variable + 1U - equation] =
            (residual - base_residuals[equation]) / probe;
      }
      candidate.node_derivatives[node] = best.node_derivatives[node];
      candidate.segments[node - 1U] = best.segments[node - 1U];
      candidate.segments[node] = best.segments[node];
    }
    if (!jacobian_valid)
      break;
    std::vector<long double> rhs(joins);
    std::transform(base_residuals.begin(), base_residuals.end(), rhs.begin(),
                   [](long double value) { return -value; });
    const auto delta = solve(std::move(jacobian), std::move(rhs));
    if (!delta)
      break;
    bool improved = false;
    for (const auto damping : {1.0L, 0.5L, 0.25L, 0.125L, 0.0625L}) {
      auto candidate = best;
      for (std::size_t variable = 0U; variable < joins; ++variable) {
        const auto bounded_delta = std::clamp((*delta)[variable], -magnitudes[variable] * 0.5L,
                                              magnitudes[variable] * 0.5L);
        candidate.node_derivatives[variable + 1U] = scale(
            directions[variable], std::max(1.0L, magnitudes[variable] + damping * bounded_delta));
      }
      if (!build_route_segments_from_derivatives(candidate))
        continue;
      const auto candidate_mismatch = mismatch(candidate);
      if (candidate_mismatch + 1.0e-18L < best_mismatch) {
        best = std::move(candidate);
        best_mismatch = candidate_mismatch;
        improved = true;
        break;
      }
    }
    if (!improved)
      break;
  }
  route = best;
  route.curvature_continuous = best_mismatch <= tolerance;
  if (stop_token.stop_requested()) return false;
  output = std::move(route);
  return output.curvature_continuous;
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
      (!route.node_tangent_directions.empty() &&
       route.node_tangent_directions.size() != route.control_points.size()) ||
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
  if (!candidate.node_tangent_directions.empty()) {
    candidate.node_tangent_directions.insert(
        candidate.node_tangent_directions.begin() + static_cast<std::ptrdiff_t>(insert_at),
        normalized(derivative));
  }
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
  auto prepared = std::make_shared<PreparedSplineCorridor>();
  prepared->regions = convex_regions;
  prepared->bounds.reserve(convex_regions.size());
  for (const auto& polygon : convex_regions) {
    PreparedSplineCorridor::Bounds bounds{
        static_cast<long double>(polygon.vertices.front().x.value()),
        static_cast<long double>(polygon.vertices.front().y.value()),
        static_cast<long double>(polygon.vertices.front().x.value()),
        static_cast<long double>(polygon.vertices.front().y.value())};
    for (const auto point : polygon.vertices) {
      bounds.minimum_x = std::min(bounds.minimum_x, static_cast<long double>(point.x.value()));
      bounds.minimum_y = std::min(bounds.minimum_y, static_cast<long double>(point.y.value()));
      bounds.maximum_x = std::max(bounds.maximum_x, static_cast<long double>(point.x.value()));
      bounds.maximum_y = std::max(bounds.maximum_y, static_cast<long double>(point.y.value()));
    }
    prepared->bounds.push_back(bounds);
  }
  route.prepared_corridor = std::move(prepared);
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

std::vector<PHOffsetIntersection> intersect_quintic_ph_offsets(
    const PHRationalOffset& first, const PHRationalOffset& second, Nanometres tolerance,
    std::size_t maximum_subdivision_depth, std::size_t maximum_intersections,
    std::size_t maximum_subdivision_nodes, PHOffsetIntersectionStats* stats) {
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
subdivide_quintic_ph_offset_bounds(const PHRationalOffset& offset, std::size_t subdivision_depth,
                                   Nanometres outward_guard) {
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
      return static_cast<std::int64_t>(
          std::clamp(value, static_cast<long double>(std::numeric_limits<std::int64_t>::min()),
                     static_cast<long double>(std::numeric_limits<std::int64_t>::max())));
    };
    result.push_back(PHOffsetSubdivisionBounds{clamp_i64(std::floor(bounds.minimum_x - guard)),
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
[[nodiscard]] bool corridor_covers_bezier_hull(const PreparedSplineCorridor& corridor,
                                               const std::array<SplineVector, 6U>& controls) {
  const auto hull = convex_hull_controls(controls);
  if (hull.empty())
    return false;
  auto hull_minimum_x = hull.front().x;
  auto hull_minimum_y = hull.front().y;
  auto hull_maximum_x = hull.front().x;
  auto hull_maximum_y = hull.front().y;
  for (const auto point : hull) {
    hull_minimum_x = std::min(hull_minimum_x, point.x);
    hull_minimum_y = std::min(hull_minimum_y, point.y);
    hull_maximum_x = std::max(hull_maximum_x, point.x);
    hull_maximum_y = std::max(hull_maximum_y, point.y);
  }
  // The guard union may contain thousands of cells, while one recursively subdivided Bézier hull
  // normally intersects only a handful. Filter by immutable cached AABBs before constructing
  // interval events; otherwise every candidate repeatedly sorts every unrelated cell boundary.
  std::vector<ContinuousRectangle> rectangles;
  rectangles.reserve(std::min<std::size_t>(corridor.regions.size(), 64U));
  for (std::size_t index = 0U; index < corridor.regions.size(); ++index) {
    const auto& bounds = corridor.bounds[index];
    if (bounds.maximum_x < hull_minimum_x || bounds.minimum_x > hull_maximum_x ||
        bounds.maximum_y < hull_minimum_y || bounds.minimum_y > hull_maximum_y) {
      continue;
    }
    const auto& polygon = corridor.regions[index];
    if (polygon_contains_all(polygon, controls))
      return true;
    const auto rectangle = axis_aligned_rectangle(polygon);
    if (!rectangle)
      return false;
    rectangles.push_back(*rectangle);
  }
  if (rectangles.empty())
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

[[nodiscard]] long double corridor_distance_to_point(const PreparedSplineCorridor& corridor,
                                                     SplineVector point) {
  auto result = std::numeric_limits<long double>::infinity();
  for (const auto& bounds : corridor.bounds) {
    const auto dx = std::max({bounds.minimum_x - point.x, 0.0L, point.x - bounds.maximum_x});
    const auto dy = std::max({bounds.minimum_y - point.y, 0.0L, point.y - bounds.maximum_y});
    result = std::min(result, std::hypot(dx, dy));
    if (result == 0.0L)
      break;
  }
  return result;
}

struct TrajectoryAssessment {
  std::size_t violations{std::numeric_limits<std::size_t>::max()};
  std::size_t shape_violations{std::numeric_limits<std::size_t>::max()};
  std::size_t bend_radius_violations{0U};
  std::size_t sampled_clearance_violations{0U};
  long double worst_bend_parameter{0.5L};
  long double objective{std::numeric_limits<long double>::infinity()};
  long double regularity{0.0L};
};

[[nodiscard]] long double exact_route_arc_length(const QuinticPHRoute& route) {
  return std::accumulate(route.segments.begin(), route.segments.end(), 0.0L,
                         [](long double total, const QuinticPHSegment& segment) {
                           return total + quintic_ph_arc_length(segment);
                         });
}

// Returns a very small, clearance-field-derived portfolio for a shortcut that crosses an
// obstacle.  The two new controls are interpolation stations (never Bezier handles): they sit
// just before and after the sampled blocked interval and are displaced together to the same side
// of the shortcut.  Exact shape/envelope checks remain the acceptance authority.
[[nodiscard]] std::vector<QuinticPHRoute>
paired_obstacle_support_proposals(const QuinticPHRoute& blocked,
                                  const QuinticPHRoute* reference,
                                  const SplineTrajectoryOptimizationConfig& config,
                                  const SplineClearanceEvaluator& clearance_evaluator,
                                  std::optional<long double> collision_hint = std::nullopt) {
  std::vector<QuinticPHRoute> proposals;
  if (!clearance_evaluator || blocked.segments.empty() ||
      !config.enable_paired_obstacle_supports || config.maximum_paired_obstacle_probes < 8U)
    return proposals;
  const auto probes = std::min<std::size_t>(128U, config.maximum_paired_obstacle_probes);
  const auto required = static_cast<long double>(config.minimum_clearance.value());
  long double first_bad = std::numeric_limits<long double>::infinity();
  long double last_bad = -std::numeric_limits<long double>::infinity();
  for (std::size_t sample = 1U; sample + 1U < probes; ++sample) {
    routing_kernel_checkpoint();
    const auto parameter = static_cast<long double>(sample) / static_cast<long double>(probes - 1U);
    const auto probe = clearance_evaluator(evaluate_quintic_ph(blocked, parameter));
    if (!probe.represented_legal || probe.clearance_nm < required) {
      first_bad = std::min(first_bad, parameter);
      last_bad = std::max(last_bad, parameter);
    }
  }
  // The continuous certificate can find an obstacle thinner than a normal optimisation sample.
  // Seed the bracket from that exact subdivision witness, then expand it by one bounded probe.
  if (collision_hint) {
    const auto parameter = std::clamp(*collision_hint, 0.001L, 0.999L);
    const auto probe = clearance_evaluator(evaluate_quintic_ph(blocked, parameter));
    if (!probe.represented_legal || probe.clearance_nm < required) {
      // Deliberately bracket just this witnessed obstacle.  A single pair must not bridge two
      // disconnected collision islands that happen to lie on the same shortcut.
      first_bad = parameter;
      last_bad = parameter;
      const auto stride = 1.0L / static_cast<long double>(probes - 1U);
      for (std::size_t step = 1U; step < probes / 2U; ++step) {
        const auto left = parameter - stride * static_cast<long double>(step);
        if (left <= 0.0L) break;
        const auto left_probe = clearance_evaluator(evaluate_quintic_ph(blocked, left));
        if (left_probe.represented_legal && left_probe.clearance_nm >= required) break;
        first_bad = left;
      }
      for (std::size_t step = 1U; step < probes / 2U; ++step) {
        const auto right = parameter + stride * static_cast<long double>(step);
        if (right >= 1.0L) break;
        const auto right_probe = clearance_evaluator(evaluate_quintic_ph(blocked, right));
        if (right_probe.represented_legal && right_probe.clearance_nm >= required) break;
        last_bad = right;
      }
    }
  }
  if (!std::isfinite(first_bad) || !std::isfinite(last_bad))
    return proposals;
  const auto denominator = static_cast<long double>(probes - 1U);
  const auto probe_span = 1.0L / denominator;
  const auto approach = std::max(0.02L, first_bad - probe_span);
  const auto departure = std::min(0.98L, last_bad + probe_span);
  if (!(approach + 0.02L < departure))
    return proposals;
  const auto middle = (approach + departure) * 0.5L;
  auto tangent = evaluate_quintic_ph_differential(blocked, middle).tangent;
  if (!(norm(tangent) > 1.0e-9L))
    tangent = subtract(blocked.control_points.back(), blocked.control_points.front());
  if (!(norm(tangent) > 1.0e-9L))
    return proposals;
  const auto side = normalized(SplineVector{-tangent.y, tangent.x});
  // Tightening may replace a much larger existing detour.  Its old, already topology-approved
  // excursion is a safe search bound; treating the ordinary local optimisation displacement as
  // a hard cap would incorrectly make a 20 mm hump unable to become a 5 mm skirt.
  const auto inherited_excursion = reference && !reference->segments.empty()
                                       ? norm(subtract(evaluate_quintic_ph(*reference, middle),
                                                       evaluate_quintic_ph(blocked, middle)))
                                       : 0.0L;
  const auto max_distance = std::max({1'000.0L,
                                      static_cast<long double>(config.maximum_corridor_displacement.value()),
                                      inherited_excursion});
  const auto legal_offset = [&](SplineVector origin, SplineVector direction) -> std::optional<long double> {
    long double low = 0.0L;
    long double high = std::max<long double>(1'000.0L,
        static_cast<long double>(std::max(config.control_step, config.continuous_envelope.hull_tolerance).value()));
    while (high <= max_distance) {
      const auto probe = clearance_evaluator(add(origin, scale(direction, high)));
      if (probe.represented_legal && probe.clearance_nm >= required) {
        for (std::size_t iteration = 0U; iteration < 6U; ++iteration) {
          const auto test = (low + high) * 0.5L;
          const auto refined = clearance_evaluator(add(origin, scale(direction, test)));
          if (refined.represented_legal && refined.clearance_nm >= required)
            high = test;
          else
            low = test;
        }
        return high + static_cast<long double>(config.continuous_envelope.hull_tolerance.value());
      }
      high *= 2.0L;
    }
    return std::nullopt;
  };
  // Prefer the side occupied by the old legal detour, then also try its mirror.  The latter is
  // useful for recovery where no legal predecessor exists.
  const auto base_middle = evaluate_quintic_ph(blocked, middle);
  const auto preferred_sign = reference && !reference->segments.empty() &&
                                      dot(subtract(evaluate_quintic_ph(*reference, middle), base_middle), side) < 0.0L
                                  ? -1.0L : 1.0L;
  for (const auto sign : {preferred_sign, -preferred_sign}) {
    routing_kernel_checkpoint();
    const auto direction = scale(side, sign);
    const auto approach_point = evaluate_quintic_ph(blocked, approach);
    const auto departure_point = evaluate_quintic_ph(blocked, departure);
    // Lift the two stations as one bridge.  Testing only their local clearance would put both
    // just outside the entry/exit boundary and let the PH span sag through the obstacle.
    const auto middle_offset = legal_offset(base_middle, direction);
    const auto approach_offset = legal_offset(approach_point, direction);
    const auto departure_offset = legal_offset(departure_point, direction);
    if (!middle_offset || !approach_offset || !departure_offset)
      continue;
    const auto shared_offset = std::max({*middle_offset, *approach_offset, *departure_offset});
    auto candidate = blocked;
    // Insert in descending parameter order so both parameters refer to the unchanged shortcut.
    if (!insert_quintic_ph_interpolation_node(candidate, departure) ||
        !insert_quintic_ph_interpolation_node(candidate, approach))
      continue;
    const auto closest = [&](SplineVector point) {
      std::size_t index = 1U;
      long double distance = std::numeric_limits<long double>::infinity();
      for (std::size_t i = 1U; i + 1U < candidate.control_points.size(); ++i) {
        const auto candidate_distance = norm(subtract(candidate.control_points[i], point));
        if (candidate_distance < distance) { distance = candidate_distance; index = i; }
      }
      return index;
    };
    const auto approach_index = closest(approach_point);
    const auto departure_index = closest(departure_point);
    if (approach_index == departure_index)
      continue;
    // The field probe supplies a conservative escape height.  Try a tiny coupled contraction
    // portfolio as well; only the full continuous envelope may admit one, and callers rank legal
    // variants by exact PH length.  Moving both stations together preserves the local skirt
    // topology instead of independently creating two arbitrary handles.
    for (const auto factor : {1.0L, 0.9L, 0.8L, 0.7L}) {
      routing_kernel_checkpoint();
      auto tightened = candidate;
      tightened.control_points[approach_index] =
          add(approach_point, scale(direction, shared_offset * factor));
      tightened.control_points[departure_index] =
          add(departure_point, scale(direction, shared_offset * factor));
      if (tightened.corridor_anchors.size() == tightened.control_points.size()) {
        tightened.corridor_anchors[approach_index] = tightened.control_points[approach_index];
        tightened.corridor_anchors[departure_index] = tightened.control_points[departure_index];
      }
      if (rebuild_quintic_ph_route(tightened))
        proposals.push_back(std::move(tightened));
    }
  }
  return proposals;
}

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
  // Exact PH arc length is the primary feasible objective. Curvature terms below are only a
  // secondary regularity tie-breaker; clearance contributes solely to violation detection.
  result.objective = config.weights.length * exact_route_arc_length(route) / 1'000'000.0L;
  constexpr long double nm_per_mm = 1'000'000.0L;
  auto prior = evaluate_quintic_ph_differential(route, 0.0L);
  for (std::size_t i = 0U; i < config.samples; ++i) {
    const auto parameter =
        static_cast<long double>(i) / static_cast<long double>(config.samples - 1U);
    const auto sample = evaluate_quintic_ph_differential(route, parameter);
    long double step_mm = 0.0L;
    if (i > 0U) {
      step_mm = norm(subtract(sample.position, prior.position)) / nm_per_mm;
      const auto change = (sample.curvature_per_nm - prior.curvature_per_nm) * nm_per_mm;
      const auto variation = config.weights.curvature_change * change * change /
                             std::max(1.0e-9L, step_mm);
      result.objective += variation;
      result.regularity += variation;
    }
    const auto curvature = sample.curvature_per_nm * nm_per_mm;
    result.objective += config.weights.curvature * curvature * curvature * step_mm;
    result.regularity += config.weights.curvature * curvature * curvature * step_mm;
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

PreparedPHSegmentCertificate
prepare_quintic_ph_segment_certificate(const QuinticPHSegment& segment,
                                       std::size_t subdivision_depth) {
  PreparedPHSegmentCertificate result;
  subdivision_depth = std::min<std::size_t>(subdivision_depth, 10U);
  const auto hash_mix = [&](long double value) {
    const auto component = static_cast<std::uint64_t>(std::hash<long double>{}(value));
    result.geometry_fingerprint ^=
        component + 0x9e3779b97f4a7c15ULL + (result.geometry_fingerprint << 6U) +
        (result.geometry_fingerprint >> 2U);
  };
  for (const auto control : segment.bezier) {
    hash_mix(control.x);
    hash_mix(control.y);
  }
  const auto make_node = [](const std::array<SplineVector, 6U>& controls, long double first,
                            long double last, std::size_t depth) {
    PHBezierCertificateNode node;
    node.bezier = controls;
    node.first_parameter = first;
    node.last_parameter = last;
    node.depth = depth;
    long double minimum_x = controls.front().x;
    long double minimum_y = controls.front().y;
    long double maximum_x = controls.front().x;
    long double maximum_y = controls.front().y;
    for (const auto point : controls) {
      minimum_x = std::min(minimum_x, point.x);
      minimum_y = std::min(minimum_y, point.y);
      maximum_x = std::max(maximum_x, point.x);
      maximum_y = std::max(maximum_y, point.y);
    }
    // Conversion can round inward. One extra representable double in the outward direction
    // restores containment and makes the subsequent SIMD-friendly AABB test conservative.
    node.outward_bounds = {
        std::nextafter(static_cast<double>(minimum_x), -std::numeric_limits<double>::infinity()),
        std::nextafter(static_cast<double>(minimum_y), -std::numeric_limits<double>::infinity()),
        std::nextafter(static_cast<double>(maximum_x), std::numeric_limits<double>::infinity()),
        std::nextafter(static_cast<double>(maximum_y), std::numeric_limits<double>::infinity())};
    return node;
  };
  result.nodes.push_back(make_node(segment.bezier, 0.0L, 1.0L, 0U));
  for (std::size_t node_index = 0U; node_index < result.nodes.size(); ++node_index) {
    const auto depth = result.nodes[node_index].depth;
    if (depth >= subdivision_depth)
      continue;
    const auto controls = result.nodes[node_index].bezier;
    const auto first_parameter = result.nodes[node_index].first_parameter;
    const auto last_parameter = result.nodes[node_index].last_parameter;
    const auto [left, right] = split_bezier(controls);
    const auto middle = (first_parameter + last_parameter) * 0.5L;
    const auto first_child = result.nodes.size();
    result.nodes.push_back(make_node(left, first_parameter, middle, depth + 1U));
    const auto second_child = result.nodes.size();
    result.nodes.push_back(make_node(right, middle, last_parameter, depth + 1U));
    result.nodes[node_index].first_child = first_child;
    result.nodes[node_index].second_child = second_child;
  }
  return result;
}

PHCenterlineClearanceVerdict certify_quintic_ph_segment_clearance(
    const PreparedPHSegmentCertificate& first, const PreparedPHSegmentCertificate& second,
    long double required_separation_nm, Nanometres numerical_guard,
    std::size_t maximum_node_pairs, PHCenterlineClearanceStats* stats) {
  if (stats != nullptr)
    *stats = {};
  if (first.nodes.empty() || second.nodes.empty() || !(required_separation_nm >= 0.0L) ||
      maximum_node_pairs == 0U)
    return PHCenterlineClearanceVerdict::ambiguous;
  const auto guard = std::max(0.0L, static_cast<long double>(numerical_guard.value()));
  const auto midpoint = [](const std::array<SplineVector, 6U>& controls) {
    auto work = controls;
    for (std::size_t level = 1U; level < work.size(); ++level)
      for (std::size_t index = 0U; index + level < work.size(); ++index)
        work[index] = lerp(work[index], work[index + 1U], 0.5L);
    return work.front();
  };
  const auto size_squared = [](const auto& bounds) {
    const auto dx = bounds[2] - bounds[0];
    const auto dy = bounds[3] - bounds[1];
    return dx * dx + dy * dy;
  };
  std::vector<std::pair<std::size_t, std::size_t>> stack{{0U, 0U}};
  bool ambiguous = false;
  std::size_t visited = 0U;
  const auto collision_threshold = std::max(0.0L, required_separation_nm - guard);
  const auto sample_collision = [&](SplineVector a, SplineVector b) {
    const auto dx = a.x - b.x;
    const auto dy = a.y - b.y;
    const auto distance_squared = dx * dx + dy * dy;
    const auto threshold_squared = collision_threshold * collision_threshold;
    const auto scale = std::max({1.0L, std::abs(distance_squared), std::abs(threshold_squared)});
    if (std::abs(distance_squared - threshold_squared) >
        64.0L * std::numeric_limits<long double>::epsilon() * scale)
      return distance_squared < threshold_squared;

    // This path is deliberately restricted to numerically degenerate, near-tangent samples.
    // It makes the final contact predicate independent of the platform's long-double width. A
    // sample miss is never used to prove separation: unresolved leaves still fall through to the
    // rational-offset isolator.
    using HighPrecision = boost::multiprecision::cpp_dec_float_50;
    if (stats != nullptr)
      ++stats->multiprecision_comparisons;
    const auto high_dx = HighPrecision{dx};
    const auto high_dy = HighPrecision{dy};
    const auto high_threshold = HighPrecision{collision_threshold};
    return high_dx * high_dx + high_dy * high_dy < high_threshold * high_threshold;
  };

#if FLUX_HAS_EXPERIMENTAL_SIMD
  using CertificateSimd = std::experimental::native_simd<double>;
  constexpr auto batch_width = CertificateSimd::size();
#else
  constexpr std::size_t batch_width = 1U;
#endif
  const auto outward_double_separated = [&](const auto& a, const auto& b) {
    // The stored interval endpoints already enclose the long-double hull. Subtraction,
    // multiplication, and addition must also be rounded outward: an upward-rounded nominal SIMD
    // sum is not by itself a valid lower distance bound. This scalar confirmation is needed only
    // for lanes which the vector screen considers separable.
    const auto downward = [](double value) {
      return value > 0.0 ? std::nextafter(value, 0.0) : 0.0;
    };
    const auto dx = std::max(
        {0.0, downward(a[0] - b[2]), downward(b[0] - a[2])});
    const auto dy = std::max(
        {0.0, downward(a[1] - b[3]), downward(b[1] - a[3])});
    const auto dx_squared = downward(dx * dx);
    const auto dy_squared = downward(dy * dy);
    const auto lower_distance_squared = downward(dx_squared + dy_squared);
    const auto upper_threshold = std::nextafter(
        static_cast<double>(required_separation_nm + guard),
        std::numeric_limits<double>::infinity());
    const auto upper_threshold_squared =
        std::nextafter(upper_threshold * upper_threshold,
                       std::numeric_limits<double>::infinity());
    return lower_distance_squared > upper_threshold_squared;
  };
  while (!stack.empty() && visited < maximum_node_pairs) {
    const auto count = std::min({batch_width, stack.size(), maximum_node_pairs - visited});
    std::array<std::pair<std::size_t, std::size_t>, batch_width> pairs{};
    for (std::size_t lane = 0U; lane < count; ++lane) {
      pairs[lane] = stack.back();
      stack.pop_back();
    }
    std::array<bool, batch_width> separated{};
#if FLUX_HAS_EXPERIMENTAL_SIMD
    std::array<double, batch_width> a_minimum_x{};
    std::array<double, batch_width> a_minimum_y{};
    std::array<double, batch_width> a_maximum_x{};
    std::array<double, batch_width> a_maximum_y{};
    std::array<double, batch_width> b_minimum_x{};
    std::array<double, batch_width> b_minimum_y{};
    std::array<double, batch_width> b_maximum_x{};
    std::array<double, batch_width> b_maximum_y{};
    for (std::size_t lane = 0U; lane < count; ++lane) {
      const auto& a = first.nodes[pairs[lane].first].outward_bounds;
      const auto& b = second.nodes[pairs[lane].second].outward_bounds;
      a_minimum_x[lane] = a[0];
      a_minimum_y[lane] = a[1];
      a_maximum_x[lane] = a[2];
      a_maximum_y[lane] = a[3];
      b_minimum_x[lane] = b[0];
      b_minimum_y[lane] = b[1];
      b_maximum_x[lane] = b[2];
      b_maximum_y[lane] = b[3];
    }
    const CertificateSimd zero{0.0};
    const CertificateSimd a_min_x{a_minimum_x.data(), std::experimental::element_aligned};
    const CertificateSimd a_min_y{a_minimum_y.data(), std::experimental::element_aligned};
    const CertificateSimd a_max_x{a_maximum_x.data(), std::experimental::element_aligned};
    const CertificateSimd a_max_y{a_maximum_y.data(), std::experimental::element_aligned};
    const CertificateSimd b_min_x{b_minimum_x.data(), std::experimental::element_aligned};
    const CertificateSimd b_min_y{b_minimum_y.data(), std::experimental::element_aligned};
    const CertificateSimd b_max_x{b_maximum_x.data(), std::experimental::element_aligned};
    const CertificateSimd b_max_y{b_maximum_y.data(), std::experimental::element_aligned};
    const auto dx = std::experimental::max(
        zero, std::experimental::max(a_min_x - b_max_x, b_min_x - a_max_x));
    const auto dy = std::experimental::max(
        zero, std::experimental::max(a_min_y - b_max_y, b_min_y - a_max_y));
    const auto outward_threshold = std::nextafter(
        static_cast<double>(required_separation_nm + guard),
        std::numeric_limits<double>::infinity());
    const auto outward_threshold_squared =
        std::nextafter(outward_threshold * outward_threshold,
                       std::numeric_limits<double>::infinity());
    const auto separated_mask = dx * dx + dy * dy > CertificateSimd{outward_threshold_squared};
    for (std::size_t lane = 0U; lane < count; ++lane) {
      separated[lane] =
          separated_mask[lane] &&
          outward_double_separated(first.nodes[pairs[lane].first].outward_bounds,
                                   second.nodes[pairs[lane].second].outward_bounds);
    }
    if (stats != nullptr) {
      ++stats->simd_batches;
      stats->simd_lanes += count;
    }
#else
    for (std::size_t lane = 0U; lane < count; ++lane) {
      const auto& a = first.nodes[pairs[lane].first].outward_bounds;
      const auto& b = second.nodes[pairs[lane].second].outward_bounds;
      separated[lane] = outward_double_separated(a, b);
    }
#endif
    visited += count;
    if (stats != nullptr)
      stats->node_pairs = visited;

    for (std::size_t lane = 0U; lane < count; ++lane) {
      const auto [first_index, second_index] = pairs[lane];
      const auto& a = first.nodes[first_index];
      const auto& b = second.nodes[second_index];
      if (separated[lane]) {
        if (stats != nullptr)
          ++stats->double_separation_proofs;
        continue;
      }
      const std::array<SplineVector, 3U> a_probes{a.bezier.front(), midpoint(a.bezier),
                                                  a.bezier.back()};
      const std::array<SplineVector, 3U> b_probes{b.bezier.front(), midpoint(b.bezier),
                                                  b.bezier.back()};
      bool collision = false;
      for (const auto pa : a_probes)
        for (const auto pb : b_probes)
          collision = collision || sample_collision(pa, pb);
      if (collision) {
        if (stats != nullptr)
          ++stats->long_double_collision_proofs;
        return PHCenterlineClearanceVerdict::colliding;
      }
      const auto a_leaf = a.first_child == std::numeric_limits<std::size_t>::max();
      const auto b_leaf = b.first_child == std::numeric_limits<std::size_t>::max();
      if (a_leaf && b_leaf) {
        ambiguous = true;
        if (stats != nullptr)
          ++stats->ambiguous_leaf_pairs;
        continue;
      }
      if (!a_leaf &&
          (b_leaf || size_squared(a.outward_bounds) >= size_squared(b.outward_bounds))) {
        stack.emplace_back(a.second_child, second_index);
        stack.emplace_back(a.first_child, second_index);
      } else {
        stack.emplace_back(first_index, b.second_child);
        stack.emplace_back(first_index, b.first_child);
      }
    }
  }
  if (!stack.empty()) {
    ambiguous = true;
    if (stats != nullptr)
      stats->node_budget_exhausted = true;
  }
  return ambiguous ? PHCenterlineClearanceVerdict::ambiguous
                   : PHCenterlineClearanceVerdict::separated;
}

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
    // A valid floating-point circle can become collinear on the integer copper lattice.
    // Do not publish that invalid primitive and then retry at ever finer sampling density.
    const auto use_arc = accepted_arc > first + 1U && accepted_line + 1U < samples.size() &&
        geometry::circular_arc_parameters(quantize(samples[first].position),
            quantize(samples[first + (accepted_arc - first) / 2U].position),
            quantize(samples[accepted_arc].position)).has_value();
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

long double native_spline_max_join_angle(const std::vector<SplineFittedPiece>& pieces) {
  const auto invalid = std::numeric_limits<long double>::infinity();
  if (pieces.empty())
    return invalid;
  std::optional<SplineVector> prior_tangent;
  std::optional<Point> prior_end;
  long double maximum = 0.0L;
  for (const auto& piece : pieces) {
    if (piece.start == piece.end || (prior_end && *prior_end != piece.start))
      return invalid;
    SplineVector start_tangent{
        static_cast<long double>(piece.end.x.value()) - piece.start.x.value(),
        static_cast<long double>(piece.end.y.value()) - piece.start.y.value()};
    auto end_tangent = start_tangent;
    if (piece.kind == SplineFittedPieceKind::circular_arc) {
      const auto arc = geometry::circular_arc_parameters(piece.start, piece.middle, piece.end);
      if (!arc)
        return invalid;
      const auto sign = arc->sweep_radians > 0.0L ? 1.0L : -1.0L;
      const auto tangent = [&](Point point) {
        return SplineVector{-sign * (static_cast<long double>(point.y.value()) - arc->center_y_nm),
                            sign * (static_cast<long double>(point.x.value()) - arc->center_x_nm)};
      };
      start_tangent = tangent(piece.start);
      end_tangent = tangent(piece.end);
    }
    if (piece.kind == SplineFittedPieceKind::cubic_bezier) {
      start_tangent = {static_cast<long double>(piece.control1.x.value()) - piece.start.x.value(),
                       static_cast<long double>(piece.control1.y.value()) - piece.start.y.value()};
      end_tangent = {static_cast<long double>(piece.end.x.value()) - piece.control2.x.value(),
                     static_cast<long double>(piece.end.y.value()) - piece.control2.y.value()};
      if (norm(start_tangent) <= 0.0L || norm(end_tangent) <= 0.0L) return invalid;
    }
    if (prior_tangent) {
      const auto angle = std::abs(
          std::atan2(cross(*prior_tangent, start_tangent), dot(*prior_tangent, start_tangent)));
      if (!std::isfinite(angle))
        return invalid;
      maximum = std::max(maximum, angle);
    }
    prior_tangent = end_tangent;
    prior_end = piece.end;
  }
  return maximum;
}

namespace {

[[nodiscard]] SplineVector native_piece_tangent(const SplineFittedPiece& piece, bool end) {
  SplineVector direction{static_cast<long double>(piece.end.x.value()) - piece.start.x.value(),
                         static_cast<long double>(piece.end.y.value()) - piece.start.y.value()};
  if (piece.kind == SplineFittedPieceKind::circular_arc) {
    const auto arc = geometry::circular_arc_parameters(piece.start, piece.middle, piece.end);
    if (!arc)
      return {};
    const auto point = end ? piece.end : piece.start;
    const auto sign = arc->sweep_radians > 0.0L ? 1.0L : -1.0L;
    direction = {-sign * (point.y.value() - arc->center_y_nm),
                 sign * (point.x.value() - arc->center_x_nm)};
  }
  return normalized(direction);
}

// Equal-distance planar Hermite biarc. The connection solves
// 2(1-t0.t1)d^2 + 2(v.(t0+t1))d - v.v = 0. Use its stable positive root;
// reject reversing/major-arc cases so the caller subdivides rather than creating a loop.
[[nodiscard]] std::vector<SplineFittedPiece> tangent_biarc(const SplineSample& first,
                                                           const SplineSample& last,
                                                           SplineVector t0, SplineVector t1,
                                                           long double maximum_angle) {
  const auto v = subtract(last.position, first.position);
  if (norm(v) < 1.0L || norm(t0) == 0.0L || norm(t1) == 0.0L || dot(v, t0) < 0.0L ||
      dot(v, t1) < 0.0L)
    return {};
  const auto projection = dot(v, add(t0, t1));
  const auto a = 2.0L * std::max(0.0L, 1.0L - dot(t0, t1));
  const auto denominator = projection + std::sqrt(projection * projection + a * dot(v, v));
  if (!(denominator > 0.0L))
    return {};
  const auto d = dot(v, v) / denominator;
  const auto joint =
      scale(add(add(first.position, last.position), scale(subtract(t0, t1), d)), 0.5L);
  const auto arc_from_tangent = [](SplineVector p, SplineVector tangent,
                                   SplineVector q) -> std::optional<SplineFittedPiece> {
    const auto chord = subtract(q, p);
    if (quantize(p) == quantize(q))
      return std::nullopt;
    const auto normal_projection = cross(tangent, chord);
    SplineFittedPiece piece{
        SplineFittedPieceKind::segment, quantize(p), {}, quantize(q), 0.0L, 1.0L};
    if (std::abs(normal_projection) < 1.0L)
      return piece;
    const auto radius = dot(chord, chord) / (2.0L * normal_projection);
    const auto center = add(p, scale(SplineVector{-tangent.y, tangent.x}, radius));
    const auto radial0 = subtract(p, center);
    const auto radial1 = subtract(q, center);
    auto sweep = std::atan2(cross(radial0, radial1), dot(radial0, radial1));
    const auto pi = std::acos(-1.0L);
    if (sweep * radius < 0.0L)
      sweep += radius > 0.0L ? 2.0L * pi : -2.0L * pi;
    if (!std::isfinite(sweep) || std::abs(sweep) > pi)
      return std::nullopt;
    const auto c = std::cos(sweep / 2.0L);
    const auto s = std::sin(sweep / 2.0L);
    piece.middle = quantize(
        add(center, SplineVector{radial0.x * c - radial0.y * s, radial0.x * s + radial0.y * c}));
    if (geometry::circular_arc_parameters(piece.start, piece.middle, piece.end))
      piece.kind = SplineFittedPieceKind::circular_arc;
    return piece;
  };
  auto left = arc_from_tangent(first.position, t0, joint);
  auto right = arc_from_tangent(last.position, scale(t1, -1.0L), joint);
  if (!left || !right)
    return {};
  std::swap(right->start, right->end);
  const auto parameter = (first.parameter + last.parameter) / 2.0L;
  left->first_parameter = first.parameter;
  left->last_parameter = parameter;
  right->first_parameter = parameter;
  right->last_parameter = last.parameter;
  std::vector<SplineFittedPiece> pieces{*left, *right};
  const auto angle = [](SplineVector a, SplineVector b) {
    return std::abs(std::atan2(cross(a, b), dot(a, b)));
  };
  // Endpoint errors add at neighboring biarcs. Reserve half the budget for that join.
  if (native_spline_max_join_angle(pieces) > maximum_angle ||
      angle(native_piece_tangent(*left, false), t0) > maximum_angle / 4.0L ||
      angle(native_piece_tangent(*right, true), t1) > maximum_angle / 4.0L)
    return {};
  return pieces;
}

[[nodiscard]] std::vector<SplineFittedPiece>
fit_tangent_biarcs(const QuinticPHRoute& route, const std::vector<SplineSample>& samples,
                   const NativeSplineFitConfig& config, Nanometres tolerance,
                   std::stop_token stop_token) {
  if (samples.size() < 2U)
    return {};
  std::vector<SplineVector> tangents;
  tangents.reserve(samples.size());
  for (const auto& sample : samples) {
    const auto [span, local] = locate_segment(route, sample.parameter);
    std::array<SplineVector, 5U> derivative;
    for (std::size_t i = 0U; i < derivative.size(); ++i)
      derivative[i] = subtract(route.segments[span].bezier[i + 1U], route.segments[span].bezier[i]);
    tangents.push_back(normalized(evaluate_bezier(derivative, local)));
  }
  std::vector<SplineFittedPiece> result;
  for (std::size_t first = 0U; first + 1U < samples.size();) {
    if (stop_token.stop_requested())
      return {};
    std::vector<SplineFittedPiece> best;
    std::size_t accepted = first;
    const auto limit = first + std::min(samples.size() - first - 1U,
                                        std::max<std::size_t>(1U, config.maximum_arc_samples));
    for (std::size_t last = first + 1U; last <= limit; ++last) {
      auto pieces = tangent_biarc(samples[first], samples[last], tangents[first], tangents[last],
                                  config.maximum_join_angle_radians);
      if (pieces.empty())
        continue;
      const auto fits_point = [&](Point point) {
        auto distance = std::numeric_limits<long double>::infinity();
        for (const auto& piece : pieces)
          distance = std::min(
              distance, piece.kind == SplineFittedPieceKind::circular_arc
                            ? geometry::point_circular_arc_distance_nm(point, piece.start,
                                                                       piece.middle, piece.end)
                            : geometry::point_segment_distance_nm(point, piece.start, piece.end));
        return distance <= static_cast<long double>(tolerance.value());
      };
      bool fits = true;
      for (std::size_t sample = first; fits && sample <= last; ++sample)
        fits = fits_point(quantize(samples[sample].position));
      // Endpoint tangent constraints can bow a biarc away from a nearly flat PH interval.
      // Probe inside even a two-sample interval; endpoint agreement alone is insufficient.
      for (const auto fraction : {0.25L, 0.5L, 0.75L}) {
        if (!fits)
          break;
        const auto parameter = samples[first].parameter +
                               fraction * (samples[last].parameter - samples[first].parameter);
        const auto [span, local] = locate_segment(route, parameter);
        fits = fits_point(quantize(evaluate_bezier(route.segments[span].bezier, local)));
      }
      if (fits) {
        best = std::move(pieces);
        accepted = last;
      }
    }
    if (best.empty() ||
        best.size() > config.maximum_pieces - std::min(config.maximum_pieces, result.size()))
      return {};
    result.insert(result.end(), best.begin(), best.end());
    first = accepted;
  }
  return native_spline_max_join_angle(result) <= config.maximum_join_angle_radians
             ? result
             : std::vector<SplineFittedPiece>{};
}

} // namespace

std::vector<SplineFittedPiece> fit_quintic_ph_to_native_copper(const QuinticPHRoute& route,
                                                               const NativeSplineFitConfig& config,
                                                               std::stop_token stop_token,
                                                               NativeSplineFitStats* stats) {
  if (stats != nullptr)
    *stats = {};
  const auto timed_us = [](auto started) {
    return static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::steady_clock::now() - started).count());
  };
  if (!std::isfinite(config.maximum_join_angle_radians) ||
      config.maximum_join_angle_radians < 0.0L || config.fit_tolerance.value() <= 0 ||
      config.tessellation_tolerance.value() <= 0)
    return {};
  auto tolerance = config.tessellation_tolerance;
  auto fit_tolerance = config.fit_tolerance;
  for (std::size_t attempt = 0U;
       attempt < std::min<std::size_t>(8U, config.maximum_refinement_attempts) &&
       !stop_token.stop_requested();
       ++attempt) {
    if (stats != nullptr)
      ++stats->attempts;
    const auto tessellation_started = std::chrono::steady_clock::now();
    const auto samples = adaptive_tessellate_quintic_ph(route, tolerance, config.maximum_depth,
                                                        config.maximum_samples);
    if (stats != nullptr) {
      stats->tessellation_us += timed_us(tessellation_started);
      stats->peak_samples = std::max(stats->peak_samples, samples.size());
    }
    if (samples.empty())
      return {};
    const auto lattice_started = std::chrono::steady_clock::now();
    std::vector<SplineSample> lattice_samples;
    lattice_samples.reserve(samples.size());
    for (const auto& sample : samples) {
      if (lattice_samples.empty() ||
          quantize(lattice_samples.back().position) != quantize(sample.position))
        lattice_samples.push_back(sample);
      else if (sample.parameter == samples.back().parameter && lattice_samples.size() > 1U)
        lattice_samples.back() = sample;
    }
    if (stats != nullptr)
      stats->lattice_filter_us += timed_us(lattice_started);
    const auto line_arc_started = std::chrono::steady_clock::now();
    auto pieces = fit_spline_samples_to_lines_and_arcs(
        lattice_samples, fit_tolerance, config.maximum_arc_samples, config.maximum_pieces);
    if (stats != nullptr)
      stats->line_arc_fit_us += timed_us(line_arc_started);
    if (!pieces.empty() &&
        native_spline_max_join_angle(pieces) <= config.maximum_join_angle_radians)
      return pieces;
    const auto biarc_started = std::chrono::steady_clock::now();
    pieces = fit_tangent_biarcs(route, lattice_samples, config, fit_tolerance, stop_token);
    if (stats != nullptr)
      stats->tangent_biarc_fit_us += timed_us(biarc_started);
    if (!pieces.empty())
      return pieces;
    if (tolerance.value() == 1 && fit_tolerance.value() == 1)
      break;
    tolerance = Nanometres{std::max<std::int64_t>(1, tolerance.value() / 2)};
    fit_tolerance = Nanometres{std::max<std::int64_t>(1, fit_tolerance.value() / 2)};
  }
  return {};
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
      // Only a genuinely closed route shares its first and last endpoints. Open
      // copper routes must also reject a crossing between those two spans.
      if (i == 1U && j + 1U == samples.size() &&
          samples.front().position == samples.back().position)
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

[[nodiscard]] std::vector<SplineEnvelopeChord>
make_quintic_ph_envelope_capsules(const QuinticPHRoute& subject, Nanometres minimum_clearance) {
  std::vector<SplineEnvelopeChord> capsules;
  capsules.reserve(subject.segments.size());
  for (const auto& segment : subject.segments) {
    if (segment.bezier.size() != 6U)
      return {};
    if (std::any_of(segment.bezier.begin(), segment.bezier.end(), [](const auto point) {
          return !std::isfinite(point.x) || !std::isfinite(point.y) ||
                 point.x <= static_cast<long double>(std::numeric_limits<std::int64_t>::min()) ||
                 point.x >= static_cast<long double>(std::numeric_limits<std::int64_t>::max()) ||
                 point.y <= static_cast<long double>(std::numeric_limits<std::int64_t>::min()) ||
                 point.y >= static_cast<long double>(std::numeric_limits<std::int64_t>::max());
        })) return {};
    const auto first = segment.bezier.front(), last = segment.bezier.back();
    const auto chord = subtract(last, first);
    const auto squared = chord.x * chord.x + chord.y * chord.y;
    long double radius = 0.0L;
    for (const auto control : segment.bezier) {
      const auto offset = subtract(control, first);
      const auto fraction = squared > 0.0L ? std::clamp((offset.x * chord.x + offset.y * chord.y) / squared, 0.0L, 1.0L) : 0.0L;
      radius = std::max(radius, norm(subtract(control, add(first, scale(chord, fraction)))));
    }
    const auto expanded = radius + std::max<std::int64_t>(0, minimum_clearance.value());
    if (!std::isfinite(expanded) ||
        expanded > static_cast<long double>(std::numeric_limits<std::int64_t>::max()))
      return {};
    capsules.push_back({first, last, Nanometres{static_cast<std::int64_t>(std::ceil(
        expanded))}});
  }
  return capsules;
}

SplineContinuousEnvelopeCertificate
certify_quintic_ph_continuous_envelope(const QuinticPHRoute& route, Nanometres minimum_clearance,
                                       const SplineClearanceEvaluator& clearance_evaluator,
                                       const SplineContinuousEnvelopeConfig& config,
                                       const SplineEnvelopeScreenEvaluator& envelope_screen_evaluator) {
  SplineContinuousEnvelopeCertificate result;
  if (route.segments.empty())
    return result;
  // A certificate must reject malformed cached geometry before delegating to either an advisory
  // capsule proof or a caller-provided scalar field. `bezier` is a fixed six-control array, but
  // retain the size check so this remains fail-closed if its representation changes.
  for (const auto& segment : route.segments) {
    if (segment.bezier.size() != 6U ||
        std::any_of(segment.bezier.begin(), segment.bezier.end(), [](const auto point) {
          return !std::isfinite(point.x) || !std::isfinite(point.y) ||
                 point.x <= static_cast<long double>(std::numeric_limits<std::int64_t>::min()) ||
                 point.x >= static_cast<long double>(std::numeric_limits<std::int64_t>::max()) ||
                 point.y <= static_cast<long double>(std::numeric_limits<std::int64_t>::min()) ||
                 point.y >= static_cast<long double>(std::numeric_limits<std::int64_t>::max());
        })) {
      ++result.clearance_violations;
      return result;
    }
  }
  std::vector<std::uint8_t> span_clear(route.segments.size(), 0U);
  if (envelope_screen_evaluator && clearance_evaluator) {
    const auto capsules = make_quintic_ph_envelope_capsules(route, minimum_clearance);
    // Do not treat an empty or incomplete capsule list as an all-clear proof.
    if (capsules.size() == route.segments.size()) {
      const auto screened = envelope_screen_evaluator(capsules);
      if (screened.definitely_clear.size() == route.segments.size())
        span_clear = screened.definitely_clear;
    }
  }
  const auto tolerance = std::max(1.0L, static_cast<long double>(config.hull_tolerance.value()));
  for (std::size_t segment_index = 0U; segment_index < route.segments.size(); ++segment_index) {
    const auto& segment = route.segments[segment_index];
    std::vector<TessellationNode> stack{
        {segment.bezier, segment.route_parameter_start, segment.route_parameter_end, 0U}};
    while (!stack.empty()) {
      auto node = stack.back();
      stack.pop_back();
      if (++result.hulls_checked > config.maximum_hulls) {
        result.budget_exhausted = true;
        return result;
      }
      auto corridor_legal = !route.prepared_corridor ||
                            corridor_covers_bezier_hull(*route.prepared_corridor, node.controls);
      const auto diameter = [&] {
        long double value = 0.0L;
        for (const auto a : node.controls)
          for (const auto b : node.controls)
            value = std::max(value, norm(subtract(a, b)));
        return value;
      }();
      bool clearance_legal = true;
      // A conservative NavMesh contains only cells whose complete rectangle is legal. Clearance
      // boundaries therefore leave legal sub-cell slivers outside the guard-cell union. Once a
      // subdivided Bézier hull is smaller than its exact point-to-boundary certificate, prove that
      // hull directly against the prepared offset domain. The ordered topology gates and winding
      // check still prevent a homotopy change, while the final quantized copper DRC remains the
      // transaction authority.
      if (!corridor_legal && clearance_evaluator) {
        const auto midpoint = evaluate_bezier(node.controls, 0.5L);
        const auto near_selected_corridor =
            route.prepared_corridor &&
            corridor_distance_to_point(*route.prepared_corridor, midpoint) <= diameter + tolerance;
        if (span_clear[segment_index]) {
          // Capsule clearance was proved against the actual offset boundary. Retain the exact
          // selected-corridor proximity condition so this acceleration cannot change homotopy.
          corridor_legal = near_selected_corridor;
        } else {
          const auto probe = clearance_evaluator(midpoint);
          if (probe.clearance_nm < result.worst_clearance_nm) {
            result.worst_clearance_nm = probe.clearance_nm;
            result.worst_parameter = (node.first + node.last) * 0.5L;
            result.worst_normal = probe.obstacle_normal;
          }
          corridor_legal =
              near_selected_corridor && probe.represented_legal &&
              probe.clearance_nm - diameter >= static_cast<long double>(minimum_clearance.value());
        }
      }
      if (clearance_evaluator && !config.corridor_certifies_clearance && !span_clear[segment_index]) {
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

namespace {
// Scoped to one optimizer invocation: its clearance field/rules are immutable. Corridor identity
// and the complete Bezier hull are part of each key. Keep corridor ownership to prevent address
// reuse, and store parameters in local span coordinates so an unchanged span survives global
// route reparameterization. Only complete legal certificates are reusable.
class SpanCertificateCache {
  using Key = std::tuple<std::array<long double, 12U>, std::uintptr_t, std::int64_t,
                         std::int64_t, std::size_t, bool>;
  struct Entry {
    SplineContinuousEnvelopeCertificate certificate;
    std::shared_ptr<const PreparedSplineCorridor> corridor;
  };
  std::map<Key, Entry> entries_;
  std::mutex mutex_;
  std::size_t hits_{0U}, misses_{0U};
  static Key key_for(const QuinticPHRoute& route, const QuinticPHSegment& segment,
                     Nanometres clearance, const SplineContinuousEnvelopeConfig& config) {
    std::array<long double, 12U> hull{};
    for (std::size_t c = 0U; c < segment.bezier.size(); ++c) {
      hull[c * 2U] = segment.bezier[c].x; hull[c * 2U + 1U] = segment.bezier[c].y;
    }
    return {hull, reinterpret_cast<std::uintptr_t>(route.prepared_corridor.get()),
            clearance.value(), config.hull_tolerance.value(), config.maximum_subdivision_depth,
            config.corridor_certifies_clearance};
  }
public:
  bool contains(const QuinticPHRoute& route, std::size_t index, Nanometres clearance,
                const SplineContinuousEnvelopeConfig& config) {
    std::scoped_lock lock(mutex_);
    return entries_.contains(key_for(route, route.segments[index], clearance, config));
  }
  std::pair<std::size_t, std::size_t> counts() {
    std::scoped_lock lock(mutex_); return {hits_, misses_};
  }
  SplineContinuousEnvelopeCertificate certify(
      const QuinticPHRoute& route, Nanometres clearance, const SplineClearanceEvaluator& evaluator,
      const SplineContinuousEnvelopeConfig& config,
      const SplineEnvelopeScreenEvaluator& screen = {}) {
    if (route.segments.empty()) return {};
    for (const auto& segment : route.segments)
      for (const auto point : segment.bezier)
        if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
            point.x <= static_cast<long double>(std::numeric_limits<std::int64_t>::min()) ||
            point.x >= static_cast<long double>(std::numeric_limits<std::int64_t>::max()) ||
            point.y <= static_cast<long double>(std::numeric_limits<std::int64_t>::min()) ||
            point.y >= static_cast<long double>(std::numeric_limits<std::int64_t>::max()))
          return certify_quintic_ph_continuous_envelope(route, clearance, evaluator, config, screen);
    std::vector<std::uint8_t> flags(route.segments.size(), 0U);
    if (screen && evaluator) {
      const auto capsules = make_quintic_ph_envelope_capsules(route, clearance);
      if (capsules.size() == flags.size()) {
        const auto screened = screen(capsules);
        if (screened.definitely_clear.size() == flags.size()) flags = screened.definitely_clear;
      }
    }
    const SplineEnvelopeScreenEvaluator retained_screen = [flags](const auto& capsules) {
      SplineEnvelopeScreenResult result;
      if (capsules.size() == flags.size()) result.definitely_clear = flags;
      return result;
    };
    SplineContinuousEnvelopeCertificate result;
    for (std::size_t i = 0U; i < route.segments.size(); ++i) {
      const auto& segment = route.segments[i];
      const auto key = key_for(route, segment, clearance, config);
      std::optional<SplineContinuousEnvelopeCertificate> certificate;
      {
        std::scoped_lock lock(mutex_);
        if (const auto found = entries_.find(key); found != entries_.end()) {
          certificate = found->second.certificate; ++hits_;
        } else ++misses_;
      }
      if (!certificate) {
        QuinticPHRoute span; span.prepared_corridor = route.prepared_corridor;
        span.segments.push_back(segment);
        span.segments.front().route_parameter_start = 0.0L;
        span.segments.front().route_parameter_end = 1.0L;
        const SplineEnvelopeScreenEvaluator span_screen = [flag = flags[i]](const auto& capsules) {
          SplineEnvelopeScreenResult result;
          if (capsules.size() == 1U) result.definitely_clear = {flag};
          return result;
        };
        certificate = certify_quintic_ph_continuous_envelope(span, clearance, evaluator, config, span_screen);
        if (certificate->legal && !certificate->budget_exhausted) {
          std::scoped_lock lock(mutex_);
          constexpr std::size_t limit = 2048U;
          if (entries_.size() >= limit) entries_.erase(entries_.begin());
          entries_.insert_or_assign(key, Entry{*certificate, route.prepared_corridor});
        }
      }
      // Preserve whole-route subdivision budgets and failure diagnostics exactly. Partial or
      // rejected span evidence never licenses acceptance or changes recovery steering.
      if (!certificate->legal || certificate->hulls_checked > config.maximum_hulls -
          std::min(config.maximum_hulls, result.hulls_checked))
        return certify_quintic_ph_continuous_envelope(route, clearance, evaluator, config, retained_screen);
      result.hulls_checked += certificate->hulls_checked;
      if (certificate->worst_clearance_nm < result.worst_clearance_nm) {
        result.worst_clearance_nm = certificate->worst_clearance_nm;
        result.worst_parameter = segment.route_parameter_start + certificate->worst_parameter *
            (segment.route_parameter_end - segment.route_parameter_start);
        result.worst_normal = certificate->worst_normal;
      }
    }
    result.legal = true;
    return result;
  }
};
} // namespace

SplineTrajectoryOptimizationResult
optimize_quintic_ph_trajectory(QuinticPHRoute& route,
                               const SplineTrajectoryOptimizationConfig& config,
                               const SplineClearanceEvaluator& clearance_evaluator,
                               const SplineRouteConstraintEvaluator& route_constraint,
                               const SplineClearanceBatchEvaluator& clearance_batch_evaluator,
                               const SplineEnvelopeScreenEvaluator& envelope_screen_evaluator) {
  SplineTrajectoryOptimizationResult result;
  if (!rebuild_quintic_ph_route(route) || route.control_points.size() < 2U ||
      config.maximum_candidate_checks == 0U || (route_constraint && !route_constraint(route)))
    return result;
  SpanCertificateCache span_cache;
  const auto certify_envelope = [&](const QuinticPHRoute& candidate, Nanometres clearance,
                                    const SplineClearanceEvaluator& evaluator,
                                    const SplineContinuousEnvelopeConfig& envelope_config,
                                    const SplineEnvelopeScreenEvaluator& screen = {}) {
    return config.reuse_span_certificates
        ? span_cache.certify(candidate, clearance, evaluator, envelope_config, screen)
        : certify_quintic_ph_continuous_envelope(candidate, clearance, evaluator, envelope_config, screen);
  };
  auto best = route;
  auto best_assessment = assess_trajectory(best, config, clearance_evaluator);
  const auto original_assessment = best_assessment;
  // Every proposal family uses the same feasible objective, including XPBD and multistart.
  const auto better_candidate = [&](const QuinticPHRoute& candidate,
                                    const TrajectoryAssessment& assessment,
                                    const SplineContinuousEnvelopeCertificate& envelope,
                                    const SplineContinuousEnvelopeCertificate& incumbent_envelope) {
    const auto violations = assessment.violations + envelope.violations();
    const auto incumbent_violations = best_assessment.violations + incumbent_envelope.violations();
    const bool legal = violations == 0U && envelope.legal;
    const bool incumbent_legal = incumbent_violations == 0U && incumbent_envelope.legal;
    if (legal != incumbent_legal) return legal;
    if (legal) {
      const auto length = exact_route_arc_length(candidate);
      const auto incumbent_length = exact_route_arc_length(best);
      return length + 1.0e-9L < incumbent_length ||
             (std::abs(length - incumbent_length) <= 1.0e-9L &&
              assessment.regularity < best_assessment.regularity);
    }
    return violations < incumbent_violations ||
           (violations == incumbent_violations &&
            (envelope.worst_clearance_nm > incumbent_envelope.worst_clearance_nm + 0.5L ||
             (envelope.worst_clearance_nm == incumbent_envelope.worst_clearance_nm &&
              assessment.objective < best_assessment.objective)));
  };
  result.initial_violations = best_assessment.violations;
  result.initial_shape_violations = best_assessment.shape_violations;
  result.initial_bend_radius_violations = best_assessment.bend_radius_violations;
  result.initial_sampled_clearance_violations = best_assessment.sampled_clearance_violations;
  result.initial_arc_length_nm = exact_route_arc_length(best);
  result.initial_regularity = best_assessment.regularity;
  result.initial_objective = best_assessment.objective;
  const auto initial_envelope = certify_envelope(
      best, config.minimum_clearance, clearance_evaluator, config.continuous_envelope);
  result.initial_continuous_violations = initial_envelope.violations();
  // A collision has two boundaries.  Before adding a single control at its worst sample (which
  // tends to make one broad hump), try a bounded approach/departure pair derived from those
  // boundaries.  This path is also used for genuinely infeasible input, where it may reduce the
  // violation set before ordinary coordinate descent begins.
  if ((best_assessment.violations + initial_envelope.violations()) > 0U &&
      config.enable_paired_obstacle_supports &&
      config.maximum_paired_obstacle_support_pairs > 0U &&
      result.adaptive_controls_inserted + 2U <= config.maximum_adaptive_control_insertions) {
    ++result.paired_obstacle_support_attempts;
    const auto paired = paired_obstacle_support_proposals(
        best, &route, config, clearance_evaluator, initial_envelope.worst_parameter);
    bool selected_pair = false;
    bool recovered_legal = false;
    for (auto candidate : paired) {
      if (result.candidate_checks >= config.maximum_candidate_checks) {
        result.budget_exhausted = true;
        break;
      }
      ++result.candidate_checks;
      routing_kernel_checkpoint();
      if (route_constraint && !route_constraint(candidate))
        continue;
      const auto assessment = assess_trajectory(candidate, config, clearance_evaluator);
      const auto envelope = certify_envelope(
          candidate, config.minimum_clearance, clearance_evaluator, config.continuous_envelope);
      const auto incumbent_envelope = certify_envelope(
          best, config.minimum_clearance, clearance_evaluator, config.continuous_envelope);
      if (!better_candidate(candidate, assessment, envelope, incumbent_envelope))
        continue;
      const auto became_legal = assessment.violations == 0U && envelope.legal;
      best = std::move(candidate);
      best_assessment = assessment;
      selected_pair = true;
      recovered_legal = recovered_legal || became_legal;
    }
    if (selected_pair) {
      result.adaptive_controls_inserted += 2U;
      result.paired_obstacle_supports_inserted += 2U;
      if (recovered_legal)
        ++result.paired_obstacle_support_recoveries;
    }
  }
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
      if (rebuild_quintic_ph_route(candidate) &&
          (!route_constraint || route_constraint(candidate))) {
        const auto assessment = assess_trajectory(candidate, config, clearance_evaluator);
        const auto envelope = certify_envelope(
            candidate, config.minimum_clearance, clearance_evaluator, config.continuous_envelope);
        const auto candidate_violations = assessment.violations + envelope.violations();
        if (better_candidate(candidate, assessment, envelope, initial_envelope)) {
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
  // adding degrees of freedom. Shorter tangents suppress algebraic branch
  // overshoot/extra inflections on closely spaced staircase controls, while a
  // longer tangent can relax a genuinely tight bend. Chord-length parameters
  // redistribute freedom on unequal spans. Keep the low-tension alternatives
  // first because the bounded default examines only four starts.
  struct InterpolationStart {
    long double exponent;
    long double tangent_scale;
  };
  const std::array<InterpolationStart, 6U> interpolation_starts{
      {{best.parameter_exponent, best.tangent_scale * 0.5L},
       {best.parameter_exponent, best.tangent_scale * 0.75L},
       {1.0L, best.tangent_scale * 0.75L},
       {best.parameter_exponent, best.tangent_scale * 1.5L},
       {1.0L, best.tangent_scale},
       {best.parameter_exponent, best.tangent_scale * 2.0L}}};
  for (const auto start : interpolation_starts) {
    if (result.multistart_attempts >= config.maximum_multistart_attempts)
      break;
    ++result.multistart_attempts;
    auto candidate = route;
    candidate.parameter_exponent = start.exponent;
    candidate.tangent_scale = start.tangent_scale;
    if (!rebuild_quintic_ph_route(candidate))
      continue;
    if (route_constraint && !route_constraint(candidate))
      continue;
    const auto assessment = assess_trajectory(candidate, config, clearance_evaluator);
    const auto envelope = certify_envelope(
        candidate, config.minimum_clearance, clearance_evaluator, config.continuous_envelope);
    const auto candidate_violations = assessment.violations + envelope.violations();
    const auto best_envelope = certify_envelope(
        best, config.minimum_clearance, clearance_evaluator, config.continuous_envelope);
    const auto best_violations = best_assessment.violations + best_envelope.violations();
    if (better_candidate(candidate, assessment, envelope, best_envelope)) {
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
    if (route_constraint && !route_constraint(candidate))
      break;
    const auto candidate_assessment = assess_trajectory(candidate, config, clearance_evaluator);
    ++result.adaptive_controls_inserted;
    const auto envelope = certify_envelope(
        candidate, config.minimum_clearance, clearance_evaluator, config.continuous_envelope);
    const auto incumbent_envelope = certify_envelope(
        best, config.minimum_clearance, clearance_evaluator, config.continuous_envelope);
    if (better_candidate(candidate, candidate_assessment, envelope, incumbent_envelope)) {
      best = std::move(candidate);
      best_assessment = candidate_assessment;
    } else {
      break;
    }
  }
  auto adaptive_envelope = certify_envelope(
      best, config.minimum_clearance, clearance_evaluator, config.continuous_envelope);
  while (adaptive_envelope.violations() > 0U &&
         result.adaptive_controls_inserted < config.maximum_adaptive_control_insertions) {
    auto parameter = std::clamp(adaptive_envelope.worst_parameter, 0.05L, 0.95L);
    if (std::any_of(best.node_parameters.begin(), best.node_parameters.end(),
                    [&](auto existing) { return std::abs(existing - parameter) < 0.025L; })) {
      parameter = std::clamp(parameter + (parameter < 0.5L ? 0.1L : -0.1L), 0.05L, 0.95L);
    }
    auto candidate = best;
    if (!insert_quintic_ph_interpolation_node(candidate, parameter))
      break;
    if (route_constraint && !route_constraint(candidate))
      break;
    ++result.adaptive_controls_inserted;
    const auto assessment = assess_trajectory(candidate, config, clearance_evaluator);
    const auto envelope = certify_envelope(
        candidate, config.minimum_clearance, clearance_evaluator, config.continuous_envelope);
    // An infeasible curve may need another interpolation degree of freedom before any
    // shortening/clearance move becomes available. This bounded recovery proposal is never
    // published as legal, and does not apply to feasible inputs.
    if (assessment.shape_violations > best_assessment.shape_violations) break;
    best = std::move(candidate);
    best_assessment = assessment;
    adaptive_envelope = envelope;
  }
  auto best_continuous_envelope = adaptive_envelope;
  auto step = std::max<long double>(1.0L, static_cast<long double>(config.control_step.value()));
  // A legal GPU elastic solution needs only bounded CPU objective polishing. An improved but
  // still illegal proposal retains the full CPU recovery budget.
  const auto refinement_iterations = result.gpu_precondition_legal
                                         ? std::min<std::size_t>(config.maximum_iterations, 8U)
                                         : config.maximum_iterations;
  // Concurrent routing phases share this lease. If another phase owns the CPU budget, exact
  // evaluation remains serial on this caller instead of creating nested worker pools.
  auto exact_candidate_lease = acquire_cpu_workers(
      config.exact_candidate_workers > 1U ? config.exact_candidate_workers : 0U);
  BoundedWorkerPool exact_candidate_pool(exact_candidate_lease.workers());
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
    if (best.prepared_corridor) {
      for (const auto& polygon : best.prepared_corridor->regions)
        for (const auto point : polygon.vertices)
          include(SplineVector{static_cast<long double>(point.x.value()),
                               static_cast<long double>(point.y.value())});
    }
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
    score_template.corridor_mask.resize(field_cells, best.prepared_corridor ? 0U : 1U);
    std::vector<SplineVector> field_points;
    field_points.reserve(field_cells);
    for (std::size_t cell = 0U; cell < field_cells; ++cell) {
      const auto x = cell % score_template.field_width;
      const auto y = cell / score_template.field_width;
      field_points.push_back(SplineVector{
          static_cast<long double>(score_template.field_origin_x_nm) + (x + 0.5L) * pitch,
          static_cast<long double>(score_template.field_origin_y_nm) + (y + 0.5L) * pitch});
    }
    // The batch callback is intentionally restricted to this lossy ranking field.  Its output
    // never reaches a hard acceptance or certificate, and malformed output falls back to the
    // existing scalar predicate rather than changing routing behaviour.
    const auto batch_probes = clearance_batch_evaluator ? clearance_batch_evaluator(field_points)
                                                        : std::vector<SplineClearanceProbe>{};
    const auto have_batch_probes = batch_probes.size() == field_points.size();
    for (std::size_t cell = 0U; cell < field_cells; ++cell) {
      const auto point = field_points[cell];
      if (clearance_evaluator) {
        const auto probe = have_batch_probes ? batch_probes[cell] : clearance_evaluator(point);
        score_template.signed_clearance_nm[cell] = static_cast<float>(
            probe.represented_legal ? probe.clearance_nm : -std::abs(probe.clearance_nm));
      } else {
        score_template.signed_clearance_nm[cell] = std::numeric_limits<float>::max();
      }
      if (best.prepared_corridor) {
        score_template.corridor_mask[cell] =
            std::any_of(best.prepared_corridor->regions.begin(),
                        best.prepared_corridor->regions.end(),
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
      if (best_continuous_envelope.violations() > 0U &&
          norm(best_continuous_envelope.worst_normal) > 0.0L &&
          node < best.node_parameters.size()) {
        const auto before_span = best.node_parameters[node] - best.node_parameters[node - 1U];
        const auto after_span = best.node_parameters[node + 1U] - best.node_parameters[node];
        const auto support = 2.0L * std::max(before_span, after_span);
        const auto distance =
            std::abs(best.node_parameters[node] - best_continuous_envelope.worst_parameter);
        const auto influence = support > 0.0L ? std::max(0.0L, 1.0L - distance / support) : 0.0L;
        if (influence > 0.0L) {
          // Project an exact continuous collision normal back onto nearby interpolation nodes.
          // This is the PH-chain equivalent of elastic-band/PNS basis weighting: the node whose
          // support overlaps the collision moves most, while distant controls remain frozen.
          proposals.push_back(
              {normalized(best_continuous_envelope.worst_normal), 0.5L + influence});
        }
      }
      for (const auto& proposal : proposals) {
        if (result.candidate_checks >= config.maximum_candidate_checks) {
          result.budget_exhausted = true;
          break;
        }
        ++result.candidate_checks;
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
      if (best_assessment.violations == 0U && best_continuous_envelope.legal) {
        // The advisory GPU shortlist must not prune taut moves in favour of extra clearance
        // or lower bending energy. Its hard violation counts remain active; the CPU then
        // compares exact PH length and curvature regularity for the shortlisted candidates.
        batch.objective_weights = {1.0F, 0.0F, 0.0F, 0.0F, 0.0F};
        batch.clearance_influence_nm = 0;
      }
      batch.sample_count = config.samples;
      batch.candidate_count = candidate_routes.size();
      batch.top_k = std::min<std::size_t>(candidate_routes.size(),
                                          std::max<std::size_t>(config.gpu_polish_shortlist, 8U));
      // The resident scorer evaluates quintic Bernstein bases itself. Supplying the cached PH
      // span controls and their normalized route intervals avoids materialising O(C*S*6N) CPU
      // basis rows and the equally dense sampled positions.  Rebuilding remains reserved for
      // the exact long-double candidates selected after advisory ranking.
      const auto spans_per_candidate = candidate_routes.front().segments.size();
      bool batch_valid = spans_per_candidate > 0U;
      batch.quintic_spans_per_candidate = spans_per_candidate;
      batch.quintic_bezier_controls_nm.reserve(spans_per_candidate * 6U * candidate_routes.size());
      batch.quintic_span_intervals.reserve(spans_per_candidate * candidate_routes.size());
      for (const auto& candidate : candidate_routes) {
        if (candidate.segments.size() != spans_per_candidate ||
            candidate.node_parameters.size() != spans_per_candidate + 1U) {
          batch_valid = false;
          break;
        }
        for (std::size_t span = 0U; span < spans_per_candidate; ++span) {
          const auto& segment = candidate.segments[span];
          const auto begin = candidate.node_parameters[span];
          const auto end = candidate.node_parameters[span + 1U];
          if (segment.bezier.size() != 6U || !std::isfinite(begin) || !std::isfinite(end) ||
              begin < 0.0L || end > 1.0L || !(end > begin)) {
            batch_valid = false;
            break;
          }
          batch.quintic_span_intervals.emplace_back(static_cast<float>(begin),
                                                    static_cast<float>(end));
          for (const auto control : segment.bezier) {
            batch.quintic_bezier_controls_nm.emplace_back(
                static_cast<std::int64_t>(std::llround(control.x)),
                static_cast<std::int64_t>(std::llround(control.y)));
          }
        }
        if (!batch_valid)
          break;
      }
      const auto evaluated = batch_valid ? score_routing_spline_candidates(batch, true)
                                         : RoutingComputeSplineScoreResult{};
      if (evaluated.ok()) {
        // A successful CPU implementation is still useful, but must not be reported as a GPU
        // dispatch.  The backend returns both facts independently when a GPU attempt falls back.
        if (evaluated.used_gpu)
          result.gpu_polish_dispatches += evaluated.dispatches;
        if (evaluated.cpu_fallback)
          ++result.gpu_polish_cpu_fallbacks;
        result.gpu_polish_solve_us += evaluated.elapsed_us;
        exact_candidates.clear();
        exact_candidates.reserve(evaluated.shortlist.size());
        for (const auto& ranked : evaluated.shortlist)
          exact_candidates.push_back(ranked.candidate);
        result.gpu_polish_exact_shortlist_checks += exact_candidates.size();
      } else if (batch_valid && evaluated.cpu_fallback) {
        ++result.gpu_polish_cpu_fallbacks;
      }
    }
    struct ExactCandidateEvaluation {
      TrajectoryAssessment assessment;
      SplineContinuousEnvelopeCertificate continuous_envelope;
    };
    // Rebuild and topology-vet deterministically before parallel exact work, so one bounded
    // capsule query covers the complete wave instead of issuing a GPU/CPU boundary query per
    // certificate. Each worker receives only its immutable slice of clear-span flags.
    std::vector<std::uint8_t> exact_ready(exact_candidates.size(), 0U);
    std::vector<std::vector<std::uint8_t>> cached_span_clear(exact_candidates.size());
    std::vector<SplineEnvelopeChord> wave_capsules;
    std::vector<std::pair<std::size_t, std::size_t>> capsule_owners;
    std::set<std::tuple<std::size_t, long double, long double>> proposed_positions;
    for (std::size_t slot = 0U; slot < exact_candidates.size(); ++slot) {
      auto& candidate = candidate_routes[exact_candidates[slot]];
      const auto node = candidate_control_nodes[exact_candidates[slot]];
      const auto point = candidate.control_points[node];
      // Preserve the advisory ranker's original population and shortlist. Removing duplicates
      // before ranking changes which distinct candidates receive exact checks and can lose a
      // downstream-realizable bend. Only suppress equivalent work after that stable selection.
      if (point == best.control_points[node] ||
          !proposed_positions.emplace(node, point.x, point.y).second) {
        ++result.equivalent_candidates_skipped;
        continue;
      }
      if (!rebuild_quintic_ph_route_around_control(candidate,
                                                   candidate_control_nodes[exact_candidates[slot]]) ||
          (route_constraint && !route_constraint(candidate)))
        continue;
      if (best_assessment.violations == 0U && best_continuous_envelope.legal &&
          exact_route_arc_length(candidate) > exact_route_arc_length(best) + 1.0e-9L) {
        ++result.non_improving_candidates_skipped;
        continue;
      }
      exact_ready[slot] = 1U;
      if (!envelope_screen_evaluator)
        continue;
      const auto capsules = make_quintic_ph_envelope_capsules(candidate, config.minimum_clearance);
      cached_span_clear[slot].assign(capsules.size(), 0U);
      for (std::size_t span = 0U; span < capsules.size(); ++span) {
        if (config.reuse_span_certificates &&
            span_cache.contains(candidate, span, config.minimum_clearance, config.continuous_envelope))
          continue;
        capsule_owners.emplace_back(slot, span);
        wave_capsules.push_back(capsules[span]);
      }
    }
    if (envelope_screen_evaluator && !wave_capsules.empty()) {
      const auto screened = envelope_screen_evaluator(wave_capsules);
      if (screened.definitely_clear.size() == wave_capsules.size()) {
        for (std::size_t item = 0U; item < capsule_owners.size(); ++item) {
          const auto [slot, span] = capsule_owners[item];
          cached_span_clear[slot][span] = screened.definitely_clear[item];
        }
      }
    }
    std::vector<std::optional<ExactCandidateEvaluation>> exact_evaluations(exact_candidates.size());
    const auto ready_checks = static_cast<std::size_t>(std::count(exact_ready.begin(), exact_ready.end(), 1U));
    if (exact_candidate_lease.workers() > 1U && ready_checks > 1U) {
      ++result.parallel_exact_waves;
      result.parallel_exact_checks += ready_checks;
    }
    exact_candidate_pool.run(exact_candidates.size(), [&](std::size_t slot) {
      const auto candidate_index = exact_candidates[slot];
      auto& candidate = candidate_routes[candidate_index];
      if (!exact_ready[slot])
        return;
      const SplineEnvelopeScreenEvaluator cached_screen = [flags = cached_span_clear[slot]](
                                                       const std::vector<SplineEnvelopeChord>& capsules) {
        SplineEnvelopeScreenResult screened;
        if (flags.size() == capsules.size())
          screened.definitely_clear = flags;
        return screened;
      };
      exact_evaluations[slot] = ExactCandidateEvaluation{
          assess_trajectory(candidate, config, clearance_evaluator),
          certify_envelope(candidate, config.minimum_clearance,
                                                 clearance_evaluator, config.continuous_envelope,
                                                 envelope_screen_evaluator ? cached_screen
                                                                          : SplineEnvelopeScreenEvaluator{})};
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
      const bool candidate_legal = candidate_violations == 0U && continuous_envelope.legal;
      const bool best_legal = best_violations == 0U && best_continuous_envelope.legal;
      const auto candidate_length = exact_route_arc_length(candidate_routes[exact_candidates[slot]]);
      const auto best_length = exact_route_arc_length(best);
      const auto better = candidate_legal != best_legal ? candidate_legal :
          candidate_legal ? (candidate_length + 1.0e-9L < best_length ||
                             (std::abs(candidate_length - best_length) <= 1.0e-9L &&
                              assessment.regularity + 1.0e-12L < best_assessment.regularity)) :
          (candidate_violations < best_violations ||
           (candidate_violations == best_violations &&
            (continuous_envelope.worst_clearance_nm > best_continuous_envelope.worst_clearance_nm + 0.5L ||
             (continuous_envelope.worst_clearance_nm == best_continuous_envelope.worst_clearance_nm &&
              assessment.objective + 1.0e-12L < best_assessment.objective))));
      if (!better)
        continue;
      const auto selected_violations = [&] {
        if (!selected_slot)
          return std::numeric_limits<std::size_t>::max();
        return exact_evaluations[*selected_slot]->assessment.violations +
               exact_evaluations[*selected_slot]->continuous_envelope.violations();
      }();
      const bool selected_legal = selected_slot && selected_violations == 0U &&
                                  exact_evaluations[*selected_slot]->continuous_envelope.legal;
      const auto selected_length = selected_slot
          ? exact_route_arc_length(candidate_routes[exact_candidates[*selected_slot]])
          : std::numeric_limits<long double>::infinity();
      const bool selected_better = !selected_slot ? true :
          candidate_legal != selected_legal ? candidate_legal :
          candidate_legal ? (candidate_length < selected_length ||
                             (candidate_length == selected_length && assessment.regularity <
                              exact_evaluations[*selected_slot]->assessment.regularity)) :
          (candidate_violations < selected_violations ||
           (candidate_violations == selected_violations &&
            (continuous_envelope.worst_clearance_nm >
                 exact_evaluations[*selected_slot]->continuous_envelope.worst_clearance_nm + 0.5L ||
             (continuous_envelope.worst_clearance_nm ==
                  exact_evaluations[*selected_slot]->continuous_envelope.worst_clearance_nm &&
              assessment.objective < exact_evaluations[*selected_slot]->assessment.objective))));
      if (selected_better)
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
  // The normal engine path reaches this optimiser after materialising PH geometry, rather than
  // through the public tautening API.  Give an already legal broad detour one bounded shortcut
  // attempt here so paired obstacle supports are available in that pipeline too.  The helper is
  // non-recursive and its exact shape/envelope/topology gates remain authoritative.
  if (config.enable_paired_obstacle_supports && clearance_evaluator &&
      best_assessment.violations == 0U && best_continuous_envelope.legal &&
      result.candidate_checks < config.maximum_candidate_checks) {
    auto tightening_config = config;
    tightening_config.maximum_candidate_checks -= result.candidate_checks;
    tightening_config.maximum_paired_obstacle_support_pairs =
        config.maximum_paired_obstacle_support_pairs > result.paired_obstacle_supports_inserted / 2U
            ? 1U : 0U;
    auto tightened = best;
    PHRouteTighteningStatistics tightening_statistics;
    const bool shortened = tauten_quintic_ph_route(
        tightened, tightening_config, clearance_evaluator, route_constraint, 1U,
        &tightening_statistics);
    result.candidate_checks += tightening_statistics.candidate_checks;
    result.paired_obstacle_support_attempts += tightening_statistics.paired_attempts;
    if (shortened) {
      const auto tightened_assessment = assess_trajectory(tightened, config, clearance_evaluator);
      const auto tightened_envelope = certify_envelope(
          tightened, config.minimum_clearance, clearance_evaluator, config.continuous_envelope);
      if (tightened_assessment.violations == 0U && tightened_envelope.legal &&
          (!config.postpolish_route_constraint || config.postpolish_route_constraint(tightened)) &&
          exact_route_arc_length(tightened) + 1.0e-9L < exact_route_arc_length(best)) {
        result.paired_obstacle_supports_inserted +=
            tightening_statistics.paired_supports_inserted;
        best = std::move(tightened);
        best_assessment = tightened_assessment;
        best_continuous_envelope = tightened_envelope;
      }
    }
  }
  // Preserve a legal input across all recovery/preconditioning paths.
  if (original_assessment.violations == 0U && initial_envelope.legal &&
      (best_assessment.violations != 0U || !best_continuous_envelope.legal ||
       exact_route_arc_length(best) > exact_route_arc_length(route) + 1.0e-9L)) {
    best = route;
    best_assessment = original_assessment;
  }
  route = best;
  auto final_shape_config = config.shape;
  final_shape_config.minimum_bend_radius = config.minimum_bend_radius;
  const auto final_shape = certify_quintic_ph_shape(route, final_shape_config);
  const auto final_envelope = certify_envelope(
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
  result.final_arc_length_nm = exact_route_arc_length(route);
  result.final_regularity = best_assessment.regularity;
  result.final_objective = best_assessment.objective;
  result.legal = best_assessment.violations == 0U && final_shape.legal && final_envelope.legal &&
                 (!route_constraint || route_constraint(route));
  result.accepted = result.legal;
  const auto [span_hits, span_misses] = span_cache.counts();
  result.span_certificate_cache_hits = span_hits;
  result.span_certificate_cache_misses = span_misses;
  return result;
}

bool tauten_quintic_ph_route(QuinticPHRoute& output,
                             const SplineTrajectoryOptimizationConfig& config,
                             const SplineClearanceEvaluator& clearance_evaluator,
                             const SplineRouteConstraintEvaluator& route_constraint,
                             std::size_t maximum_removals,
                             PHRouteTighteningStatistics* statistics) {
  PHRouteTighteningStatistics local_statistics;
  auto& stats = statistics ? *statistics : local_statistics;
  stats = {};
  if (maximum_removals == 0U || config.maximum_candidate_checks == 0U ||
      output.control_points.size() <= 2U)
    return false;
  auto route = output;
  if (!rebuild_quintic_ph_route(route)) return false;
  const auto legal = [&](const QuinticPHRoute& candidate) {
    auto shape_config = config.shape;
    shape_config.minimum_bend_radius = config.minimum_bend_radius;
    return (!route_constraint || route_constraint(candidate)) &&
           certify_quintic_ph_shape(candidate, shape_config).legal &&
           certify_quintic_ph_continuous_envelope(candidate, config.minimum_clearance,
                                                  clearance_evaluator,
                                                  config.continuous_envelope).legal;
  };
  if (!legal(route)) return false;
  bool shortened = false;
  auto& checks = stats.candidate_checks;
  std::size_t paired_pairs_used = 0U;
  for (std::size_t removed = 0U; removed < maximum_removals && route.control_points.size() > 2U;
       ++removed) {
    const auto baseline = exact_route_arc_length(route);
    std::optional<QuinticPHRoute> best;
    long double best_length = baseline;
    bool best_is_pair = false;
    for (std::size_t node = 1U; node + 1U < route.control_points.size() &&
         checks < config.maximum_candidate_checks; ++node) {
      routing_kernel_checkpoint();
      ++checks;
      if (node < route.node_tangent_directions.size() &&
          norm(route.node_tangent_directions[node]) > 0.0L)
        continue; // An explicitly constrained tangent is a protected interpolation station.
      auto candidate = route;
      candidate.control_points.erase(candidate.control_points.begin() +
                                     static_cast<std::ptrdiff_t>(node));
      if (candidate.corridor_anchors.size() == route.control_points.size())
        candidate.corridor_anchors.erase(candidate.corridor_anchors.begin() +
                                         static_cast<std::ptrdiff_t>(node));
      if (candidate.node_tangent_directions.size() == route.control_points.size())
        candidate.node_tangent_directions.erase(candidate.node_tangent_directions.begin() +
                                                static_cast<std::ptrdiff_t>(node));
      if (!rebuild_quintic_ph_route(candidate)) continue;
      if (legal(candidate)) {
        const auto length = exact_route_arc_length(candidate);
        if (length + 1.0e-9L < best_length) {
          best = std::move(candidate);
          best_length = length;
          best_is_pair = false;
        }
        continue;
      }
      // Removing a broad, legal hump can expose a blocked shortcut.  Replace it with a bounded
      // approach/departure pair only when the complete exact certificate says it is a strict
      // shortening.  The original route remains untouched when a topology/corridor callback
      // vetoes either support placement.
      if (!config.enable_paired_obstacle_supports ||
          paired_pairs_used >= config.maximum_paired_obstacle_support_pairs ||
          checks >= config.maximum_candidate_checks)
        continue;
      const auto blocked_envelope = certify_quintic_ph_continuous_envelope(
          candidate, config.minimum_clearance, clearance_evaluator, config.continuous_envelope);
      ++stats.paired_attempts;
      const auto paired = paired_obstacle_support_proposals(candidate, &route, config,
                                                             clearance_evaluator,
                                                             blocked_envelope.worst_parameter);
      for (auto support_candidate : paired) {
        if (checks >= config.maximum_candidate_checks)
          break;
        ++checks;
        routing_kernel_checkpoint();
        if (!legal(support_candidate))
          continue;
        const auto length = exact_route_arc_length(support_candidate);
        if (length + 1.0e-9L < best_length) {
          best = std::move(support_candidate);
          best_length = length;
          best_is_pair = true;
        }
      }
    }
    if (!best) break;
    if (best_is_pair) {
      ++paired_pairs_used;
      stats.paired_supports_inserted += 2U;
    }
    route = std::move(*best);
    shortened = true;
  }
  if (shortened) output = std::move(route);
  return shortened;
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
