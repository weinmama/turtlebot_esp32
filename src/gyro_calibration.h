#pragma once
#include <cmath>
#include <cstddef>

// Startup-only gyro bias estimate. Motion resets the whole sample window;
// never change an established bias while the robot is operating.
class GyroCalibration {
 public:
  static constexpr unsigned kSamples = 100;  // 5 seconds at 20 Hz
  void sample(float x, float y, float z, bool stationary) {
    if (ready_) return;
    const float values[3] = {x, y, z};
    if (!stationary) { reset(); return; }
    for (float value : values) {
      if (!std::isfinite(value) || std::fabs(value) > 0.15f) {
        reset();
        return;
      }
    }
    ++count_;
    for (unsigned axis = 0; axis < 3; ++axis) {
      const float delta = values[axis] - mean_[axis];
      mean_[axis] += delta / count_;
      m2_[axis] += delta * (values[axis] - mean_[axis]);
    }
    if (count_ == kSamples) {
      for (float variance_sum : m2_) {
        if (variance_sum / (count_ - 1) > 0.0001f) { reset(); return; }
      }
      ready_ = true;
    }
  }
  bool ready() const { return ready_; }
  float bias(unsigned axis) const { return ready_ && axis < 3 ? mean_[axis] : 0.0f; }
 private:
  void reset() {
    count_ = 0;
    for (unsigned axis = 0; axis < 3; ++axis) mean_[axis] = m2_[axis] = 0.0f;
  }
  unsigned count_ = 0;
  float mean_[3] = {}, m2_[3] = {};
  bool ready_ = false;
};
