#pragma once
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <deque>
#include <stdexcept>

namespace mid360_analysis {
// Population standard deviation. Welford updates avoid sum-of-squares cancellation.
class RunningStats {
public:
  explicit RunningStats(std::size_t capacity = 200) : capacity_(capacity) {
    if (!capacity) throw std::invalid_argument("window_size must be positive");
  }
  void push(double value) {
    if (!std::isfinite(value)) throw std::invalid_argument("sample must be finite");
    if (values_.size() == capacity_) {
      const long double old = values_.front();
      values_.pop_front();
      if (values_.empty()) { mean_ = 0; m2_ = 0; }
      else {
        const long double next_mean = mean_ + (mean_ - old) / values_.size();
        m2_ = std::max(0.L, m2_ - (old - mean_) * (old - next_mean));
        mean_ = next_mean;
      }
    }
    values_.push_back(value);
    const long double delta = value - mean_;
    mean_ += delta / values_.size();
    m2_ += delta * (value - mean_);
  }
  double mean() const { return static_cast<double>(mean_); }
  double stddev() const {
    return values_.empty() ? 0. : static_cast<double>(std::sqrt(std::max(0.L, m2_) / values_.size()));
  }
  std::size_t size() const { return values_.size(); }
  bool full() const { return size() == capacity_; }
  void reset() { values_.clear(); mean_ = 0; m2_ = 0; }
private:
  std::size_t capacity_;
  std::deque<double> values_;
  long double mean_{0}, m2_{0};
};
}  // namespace mid360_analysis
