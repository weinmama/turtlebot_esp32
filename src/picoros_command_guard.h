#pragma once

#include <cmath>
#include <cstdint>

// Middleware-independent safety gate for the future Pico-ROS adapter.
// Times are monotonic MCU milliseconds except command_age_ns, which is
// computed from a validated, synchronized ROS epoch by the transport layer.
class PicoRosCommandGuard {
 public:
  enum class Mode { blocked, idle, mapping, navigation };
  enum class Source { manual, navigation };
  static constexpr int64_t kHeartbeatTimeoutMs = 500;
  static constexpr int64_t kCommandTimeoutMs = 500;
  static constexpr int64_t kMaxCommandAgeNs = 300000000;
  static constexpr int64_t kFutureToleranceNs = 200000000;

  void heartbeat(Mode mode, int64_t now_ms) {
    if (now_ms < 0) { invalidate(); return; }
    if (mode != mode_) has_command_ = false;
    mode_ = mode;
    heartbeat_ms_ = now_ms;
    has_heartbeat_ = true;
  }

  bool accept(Source source, float linear, float angular, int64_t command_age_ns,
              int64_t now_ms, bool clock_valid, bool sensors_ready) {
    if (!clock_valid || !sensors_ready || !has_heartbeat_ || now_ms < heartbeat_ms_ ||
        now_ms - heartbeat_ms_ > kHeartbeatTimeoutMs || !std::isfinite(linear) ||
        !std::isfinite(angular) || command_age_ns < -kFutureToleranceNs ||
        command_age_ns > kMaxCommandAgeNs) return false;
    const bool permitted = source == Source::navigation ? mode_ == Mode::navigation :
                           (mode_ == Mode::mapping || mode_ == Mode::idle);
    if (!permitted) return false;
    command_ms_ = now_ms;
    has_command_ = true;
    return true;
  }

  bool should_stop(int64_t now_ms, bool clock_valid, bool sensors_ready) const {
    return !clock_valid || !sensors_ready || !has_heartbeat_ || !has_command_ ||
           mode_ == Mode::blocked || now_ms < heartbeat_ms_ || now_ms < command_ms_ ||
           now_ms - heartbeat_ms_ > kHeartbeatTimeoutMs ||
           now_ms - command_ms_ > kCommandTimeoutMs;
  }

  void invalidate() { mode_ = Mode::blocked; has_heartbeat_ = has_command_ = false; }

 private:
  Mode mode_ = Mode::blocked;
  int64_t heartbeat_ms_ = 0, command_ms_ = 0;
  bool has_heartbeat_ = false, has_command_ = false;
};
