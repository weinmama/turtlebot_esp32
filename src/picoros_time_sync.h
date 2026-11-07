#pragma once

#include <cstdint>
#include <limits>

// Monotonic MCU timestamps are mapped to the Pi's ROS epoch by a bounded-RTT
// exchange. No timestamp is valid until a successful exchange has completed.
class PicoRosTimeSync {
 public:
  enum class UpdateResult : int64_t { accepted = 0, invalid = 1, round_trip = 2, nonmonotonic = 3, clock_step = 4 };
  static constexpr int64_t kMaxRoundTripNs = 100000000;  // 100 ms
  // Midpoint uncertainty is bounded by half the observed RTT. During holdover
  // budget a deliberately conservative 1000 ppm oscillator drift. Time is
  // usable only while their sum remains below 100 ms (below command-age and
  // future-tolerance limits); this is not an arbitrary fixed age extension.
  static constexpr int64_t kMaxUncertaintyNs = 100000000LL;
  static constexpr int64_t kMaxDriftPpb = 1000000LL;
  static constexpr int64_t kMaxCorrectionNs = 200000000;  // reject clock steps
  static constexpr int64_t kSlewDivisor = 200;  // at most 5 ms correction / second

  // The Pi records its epoch immediately on receipt of a request. The MCU
  // estimates the request arrival at the midpoint of its send/receive times.
  bool update(int64_t sent_ns, int64_t received_ns, int64_t host_epoch_ns,
              UpdateResult* result = nullptr) {
    const auto reject = [result](UpdateResult reason) {
      if (result) *result = reason;
      return false;
    };
    if (sent_ns < 0 || received_ns < sent_ns || host_epoch_ns <= 0) return reject(UpdateResult::invalid);
    const int64_t rtt = received_ns - sent_ns;
    if (rtt > kMaxRoundTripNs) return reject(UpdateResult::round_trip);
    const int64_t midpoint = sent_ns + rtt / 2;
    const int64_t target = host_epoch_ns - midpoint;
    if (synced_) {
      if (received_ns <= last_sync_ns_) return reject(UpdateResult::nonmonotonic);
      const int64_t current = offset_at(received_ns);
      // An isolated bad exchange must not move the clock or refresh liveness.
      // A sustained host clock step expires the clock and triggers the normal
      // stopped reconnect path, rather than publishing backwards sensor time.
      const uint64_t distance = target >= current
          ? uint64_t(target) - uint64_t(current) : uint64_t(current) - uint64_t(target);
      if (distance > uint64_t(kMaxCorrectionNs))
        return reject(UpdateResult::clock_step);
      offset_ns_ = current;
    } else {
      offset_ns_ = target;
    }
    target_offset_ns_ = target;
    last_sync_ns_ = received_ns;
    last_rtt_ns_ = rtt;
    synced_ = true;
    if (result) *result = UpdateResult::accepted;
    return true;
  }

  bool epoch(int64_t monotonic_ns, int64_t& result) const {
    if (!synced_ || monotonic_ns < last_sync_ns_) return false;
    if (uncertainty_ns(monotonic_ns) < 0) return false;
    const int64_t offset = offset_at(monotonic_ns);
    if (offset > 0 && monotonic_ns > std::numeric_limits<int64_t>::max() - offset) return false;
    result = monotonic_ns + offset;
    return result > 0;
  }

  void invalidate() { synced_ = false; }

  int64_t uncertainty_ns(int64_t monotonic_ns) const {
    if (!synced_ || monotonic_ns < last_sync_ns_) return -1;
    const int64_t age_ns = monotonic_ns - last_sync_ns_;
    const int64_t drift_ns = age_ns / 1000000000LL * kMaxDriftPpb +
        (age_ns % 1000000000LL) * kMaxDriftPpb / 1000000000LL;
    if (last_rtt_ns_ / 2 > kMaxUncertaintyNs - drift_ns) return -1;
    return last_rtt_ns_ / 2 + drift_ns;
  }

 private:
  int64_t offset_at(int64_t now) const {
    const int64_t limit = (now - last_sync_ns_) / kSlewDivisor;
    const int64_t correction = target_offset_ns_ - offset_ns_;
    return offset_ns_ + (correction > limit ? limit :
                         (correction < -limit ? -limit : correction));
  }
  bool synced_ = false;
  int64_t offset_ns_ = 0;
  int64_t target_offset_ns_ = 0;
  int64_t last_sync_ns_ = 0;
  int64_t last_rtt_ns_ = 0;
};
