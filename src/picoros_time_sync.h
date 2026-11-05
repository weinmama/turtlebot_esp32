#pragma once

#include <cstdint>
#include <limits>

// Monotonic MCU timestamps are mapped to the Pi's ROS epoch by a bounded-RTT
// exchange. No timestamp is valid until a successful exchange has completed.
class PicoRosTimeSync {
 public:
  static constexpr int64_t kMaxRoundTripNs = 100000000;  // 100 ms
  static constexpr int64_t kMaxAgeNs = 120000000000LL;  // 120 s

  // The Pi records its epoch immediately on receipt of a request. The MCU
  // estimates the request arrival at the midpoint of its send/receive times.
  bool update(int64_t sent_ns, int64_t received_ns, int64_t host_epoch_ns) {
    if (sent_ns < 0 || received_ns < sent_ns || host_epoch_ns <= 0) return false;
    const int64_t rtt = received_ns - sent_ns;
    if (rtt > kMaxRoundTripNs) return false;
    const int64_t midpoint = sent_ns + rtt / 2;
    offset_ns_ = host_epoch_ns - midpoint;
    last_sync_ns_ = received_ns;
    synced_ = true;
    return true;
  }

  bool epoch(int64_t monotonic_ns, int64_t& result) const {
    if (!synced_ || monotonic_ns < last_sync_ns_ ||
        monotonic_ns - last_sync_ns_ > kMaxAgeNs ||
        (offset_ns_ > 0 && monotonic_ns > std::numeric_limits<int64_t>::max() - offset_ns_)) return false;
    result = monotonic_ns + offset_ns_;
    return result > 0;
  }

  void invalidate() { synced_ = false; }

 private:
  bool synced_ = false;
  int64_t offset_ns_ = 0;
  int64_t last_sync_ns_ = 0;
};
