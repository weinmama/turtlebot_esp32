#pragma once

#include <cstdint>

// Time-based rejection of edges that would imply a wheel rate above the
// configured physical/control maximum. Unsigned subtraction handles micros()
// rollover. Rejected edges do not move the accepted-edge reference time.
class EncoderPulseFilter {
 public:
  explicit constexpr EncoderPulseFilter(uint32_t minimum_interval_us)
      : minimum_interval_us_(minimum_interval_us) {}

  bool accept(uint32_t edge_time_us) {
    if (has_accepted_edge_ &&
        static_cast<uint32_t>(edge_time_us - last_accepted_edge_us_) <
            minimum_interval_us_) {
      return false;
    }
    last_accepted_edge_us_ = edge_time_us;
    has_accepted_edge_ = true;
    return true;
  }

 private:
  uint32_t minimum_interval_us_;
  uint32_t last_accepted_edge_us_ = 0;
  bool has_accepted_edge_ = false;
};
