#pragma once
#include "mid360_analysis/imu_analysis.hpp"

namespace mid360_analysis {
struct OdomConfig { double min_step_m{0.005}; };
class OdomAnalysis {
public:
  explicit OdomAnalysis(OdomConfig cfg = {}) : cfg_(cfg) {
    if (!std::isfinite(cfg.min_step_m) || cfg.min_step_m < 0.)
      throw std::invalid_argument("min_step_m must be finite and nonnegative");
  }
  bool update(double time, const Vec3 &position) {
    segment_reset = false;
    if (!std::isfinite(time) || !finite(position)) return false;
    if (has_previous_ && time == previous_time_) { ++non_monotonic_count; return false; }
    if (has_previous_ && time < previous_time_) {
      ++non_monotonic_count; reset(); segment_reset = true;
    }
    if (!has_previous_) { first_ = anchor_ = position; }
    else {
      const double dt = time - previous_time_;
      velocity = subtract(position, previous_);
      for (auto &v : velocity) v /= dt;
      speed = norm(velocity);
      if (!finite(velocity) || !std::isfinite(speed)) return false;
      max_speed = std::max(max_speed, speed);
      dt_stats.push(dt);
      // Keep the reference until the threshold is crossed; do not lose slow motion.
      const double step = norm(subtract(position, anchor_));
      if (step > cfg_.min_step_m) { distance += step; anchor_ = position; }
    }
    displacement = norm(subtract(position, first_));
    previous_ = position; previous_time_ = time; has_previous_ = true;
    return true;
  }
  void reset() {
    has_previous_ = false; distance = speed = max_speed = displacement = 0.;
    velocity = {}; dt_stats.reset();
  }
  double rate() const { return dt_stats.mean() > 0. ? 1. / dt_stats.mean() : 0.; }
  double distance{0.}, speed{0.}, max_speed{0.}, displacement{0.};
  Vec3 velocity{};
  RunningStats dt_stats{50};
  std::uint64_t non_monotonic_count{0};
  bool segment_reset{false};
private:
  OdomConfig cfg_;
  bool has_previous_{false};
  Vec3 first_{}, previous_{}, anchor_{};
  double previous_time_{0.};
};
}  // namespace mid360_analysis
