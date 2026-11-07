#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>

// Reject impossible encoder samples from odometry/PID and latch repeated
// anomalies. The window is rolling; each control interval counts once even if
// both wheels are invalid.
class EncoderFeedbackGuard {
 public:
  static constexpr float kMaxRpm = 100.0f;
  static constexpr int64_t kWindowMs = 1000;
  static constexpr size_t kToleratedSamples = 3;

  struct Result {
    bool left_invalid;
    bool right_invalid;
    bool latched;
    size_t anomalies_in_window;
  };

  Result observe(int64_t now_ms, float left_rpm, float right_rpm) {
    const bool left_invalid = !std::isfinite(left_rpm) || fabsf(left_rpm) > kMaxRpm;
    const bool right_invalid = !std::isfinite(right_rpm) || fabsf(right_rpm) > kMaxRpm;
    if (latched_) return {left_invalid, right_invalid, true, count_recent(now_ms)};

    if (left_invalid || right_invalid) {
      prune(now_ms);
      if (count_ < timestamps_.size()) {
        timestamps_[(head_ + count_) % timestamps_.size()] = now_ms;
        ++count_;
      }
      if (count_ > kToleratedSamples) latched_ = true;
    } else {
      prune(now_ms);
    }
    return {left_invalid, right_invalid, latched_, count_recent(now_ms)};
  }

  bool latched() const { return latched_; }

 private:
  void prune(int64_t now_ms) {
    while (count_ > 0 && now_ms - timestamps_[head_] > kWindowMs) {
      head_ = (head_ + 1) % timestamps_.size();
      --count_;
    }
  }

  size_t count_recent(int64_t now_ms) {
    prune(now_ms);
    return count_;
  }

  std::array<int64_t, kToleratedSamples + 1> timestamps_{};
  size_t head_ = 0;
  size_t count_ = 0;
  bool latched_ = false;
};
