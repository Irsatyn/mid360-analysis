#pragma once
#include <array>
#include <cstdint>
#include "mid360_analysis/running_stats.hpp"

namespace mid360_analysis {
using Vec3 = std::array<double, 3>;
inline bool finite(const Vec3 &v) {
  return std::isfinite(v[0]) && std::isfinite(v[1]) && std::isfinite(v[2]);
}
inline double norm(const Vec3 &v) { return std::hypot(v[0], v[1], v[2]); }
inline Vec3 subtract(const Vec3 &a, const Vec3 &b) {
  return {a[0]-b[0], a[1]-b[1], a[2]-b[2]};
}
inline void require_positive(double v, const char *name) {
  if (!std::isfinite(v) || v <= 0.) throw std::invalid_argument(name);
}
struct ImuConfig {
  std::size_t window_size{200};
  double nominal_rate_hz{200.}, drop_factor{1.5};
  bool acc_unit_is_g{true};
  double static_acc_tol{0.05}, static_gyro_tol{0.02}, static_std_tol{0.02};
};
class ImuAnalysis {
public:
  explicit ImuAnalysis(ImuConfig cfg = {})
  : dt_stats(cfg.window_size), acc_norm(cfg.window_size), gyro_norm(cfg.window_size),
    acc_axes{RunningStats(cfg.window_size), RunningStats(cfg.window_size), RunningStats(cfg.window_size)},
    gyro_axes{RunningStats(cfg.window_size), RunningStats(cfg.window_size), RunningStats(cfg.window_size)}, cfg_(cfg) {
    require_positive(cfg.nominal_rate_hz, "nominal_rate_hz must be positive");
    require_positive(cfg.drop_factor, "drop_factor must be positive");
    require_positive(cfg.static_acc_tol, "static_acc_tol must be positive");
    require_positive(cfg.static_gyro_tol, "static_gyro_tol must be positive");
    require_positive(cfg.static_std_tol, "static_std_tol must be positive");
  }
  bool update(double time, Vec3 acc, const Vec3 &gyro) {
    segment_reset = false;
    if (!std::isfinite(time) || !finite(acc) || !finite(gyro)) return false;
    if (!cfg_.acc_unit_is_g) for (auto &v : acc) v /= 9.80665;
    const double an = norm(acc), gn = norm(gyro);
    if (!std::isfinite(an) || !std::isfinite(gn)) return false;
    if (has_previous_) {
      const double dt = time - previous_time_;
      if (dt <= 0.) {
        ++non_monotonic_count;
        if (dt == 0.) return false;
        reset(); segment_reset = true;
      } else {
        if (dt > cfg_.drop_factor / cfg_.nominal_rate_hz) ++drop_count;
        dt_stats.push(dt);
      }
    }
    previous_time_ = time; has_previous_ = true;
    acc_norm.push(an); gyro_norm.push(gn);
    for (std::size_t i=0; i<3; ++i) { acc_axes[i].push(acc[i]); gyro_axes[i].push(gyro[i]); }
    static_ = acc_norm.full() && std::abs(acc_norm.mean()-1.) < cfg_.static_acc_tol &&
      gyro_norm.mean() < cfg_.static_gyro_tol && acc_norm.stddev() < cfg_.static_std_tol;
    if (static_) {
      for (std::size_t i=0; i<3; ++i) bias_[i] = gyro_axes[i].mean();
      bias_valid_ = true;
    }
    return true;
  }
  void reset() {
    dt_stats.reset(); acc_norm.reset(); gyro_norm.reset();
    for (auto &s : acc_axes) s.reset();
    for (auto &s : gyro_axes) s.reset();
    has_previous_ = false; static_ = false; bias_ = {}; bias_valid_ = false;
  }
  double rate() const { return dt_stats.mean() > 0. ? 1. / dt_stats.mean() : 0.; }
  bool is_static() const { return static_; }
  bool bias_valid() const { return bias_valid_; }
  const Vec3 &bias() const { return bias_; }
  RunningStats dt_stats, acc_norm, gyro_norm;
  std::array<RunningStats, 3> acc_axes, gyro_axes;
  std::uint64_t drop_count{0}, non_monotonic_count{0};
  bool segment_reset{false};
private:
  ImuConfig cfg_;
  bool has_previous_{false}, static_{false}, bias_valid_{false};
  double previous_time_{0.};
  Vec3 bias_{};
};
}  // namespace mid360_analysis
