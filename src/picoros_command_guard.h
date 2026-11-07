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
  enum class RejectReason : uint8_t {
    none,
    clock_invalid,
    sensors_not_ready,
    heartbeat_missing,
    heartbeat_from_future,
    heartbeat_stale,
    nonfinite_command,
    command_from_future,
    command_stale,
    receive_from_future,
    delivery_stale,
    mode_mismatch,
    count
  };
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

  RejectReason rejection_reason(Source source, float linear, float angular,
                                int64_t command_age_ns, int64_t now_ms,
                                bool clock_valid, bool sensors_ready,
                                int64_t received_ms = -1) const {
    if (received_ms < 0) received_ms = now_ms;
    if (!clock_valid) return RejectReason::clock_invalid;
    if (!sensors_ready) return RejectReason::sensors_not_ready;
    if (!has_heartbeat_) return RejectReason::heartbeat_missing;
    if (now_ms < heartbeat_ms_) return RejectReason::heartbeat_from_future;
    if (now_ms - heartbeat_ms_ > kHeartbeatTimeoutMs) return RejectReason::heartbeat_stale;
    if (!std::isfinite(linear) || !std::isfinite(angular))
      return RejectReason::nonfinite_command;
    if (command_age_ns < -kFutureToleranceNs) return RejectReason::command_from_future;
    if (command_age_ns > kMaxCommandAgeNs) return RejectReason::command_stale;
    if (received_ms > now_ms) return RejectReason::receive_from_future;
    if (now_ms - received_ms > 200) return RejectReason::delivery_stale;
    const bool permitted = source == Source::navigation ? mode_ == Mode::navigation :
                           (mode_ == Mode::mapping || mode_ == Mode::idle);
    if (!permitted) return RejectReason::mode_mismatch;
    return RejectReason::none;
  }

  bool accept(Source source, float linear, float angular, int64_t command_age_ns,
              int64_t now_ms, bool clock_valid, bool sensors_ready,
              int64_t received_ms = -1, RejectReason* rejection = nullptr) {
    const RejectReason reason = rejection_reason(source, linear, angular, command_age_ns,
                                                 now_ms, clock_valid, sensors_ready,
                                                 received_ms);
    if (rejection) *rejection = reason;
    if (reason != RejectReason::none)
      return false;
    command_ms_ = received_ms;
    if (command_ms_ < 0) command_ms_ = now_ms;
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
  void clear_command() { has_command_ = false; }

 private:
  Mode mode_ = Mode::blocked;
  int64_t heartbeat_ms_ = 0, command_ms_ = 0;
  bool has_heartbeat_ = false, has_command_ = false;
};
