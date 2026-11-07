#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>

// Estimate magnitude from A-edge periods. Short electrical glitches are
// discarded, but the threshold is deliberately independent of the 100 RPM
// operational limit so genuine overspeed remains observable. A median rejects
// isolated extra edges without turning them into artificial RPM samples.
class EncoderSpeedEstimator {
 public:
  static constexpr size_t kPeriods = 5;
  static constexpr size_t kMinimumPeriods = 3;
  // Observed false edges are tens of microseconds apart; 200 us still permits
  // pulses at >460 RPM, well above this wheel's configured physical limit.
  static constexpr uint32_t kMinimumElectricalSpacingUs = 200;
  static constexpr uint32_t kStaleAfterUs = 150000;

  bool record(uint32_t edge_time_us) {
    if (!has_last_edge_) {
      last_edge_us_ = edge_time_us;
      has_last_edge_ = true;
      return true;
    }

    const uint32_t interval = edge_time_us - last_edge_us_;
    if (interval > kStaleAfterUs) {
      reset_periods();
      last_edge_us_ = edge_time_us;
      return true;
    }
    if (interval < kMinimumElectricalSpacingUs) return false;

    periods_[next_] = interval;
    next_ = (next_ + 1) % periods_.size();
    if (count_ < periods_.size()) ++count_;
    last_edge_us_ = edge_time_us;
    return true;
  }

  bool ready(uint32_t now_us) const {
    return count_ >= kMinimumPeriods &&
           static_cast<uint32_t>(now_us - last_edge_us_) <= kStaleAfterUs;
  }

  float rpm(uint32_t now_us, float pulses_per_revolution) const {
    if (!ready(now_us) || pulses_per_revolution <= 0.0f) return 0.0f;
    auto sorted = periods_;
    std::sort(sorted.begin(), sorted.begin() + count_);
    float median_us = sorted[count_ / 2];
    if (count_ % 2 == 0)
      median_us = (sorted[count_ / 2 - 1] + median_us) * 0.5f;
    if (median_us <= 0.0f) return 0.0f;
    return 60000000.0f / (pulses_per_revolution * median_us);
  }

  void reset() {
    reset_periods();
    has_last_edge_ = false;
    last_edge_us_ = 0;
  }

 private:
  void reset_periods() {
    periods_.fill(0);
    next_ = 0;
    count_ = 0;
  }

  std::array<uint32_t, kPeriods> periods_{};
  uint32_t last_edge_us_ = 0;
  size_t next_ = 0;
  size_t count_ = 0;
  bool has_last_edge_ = false;
};
