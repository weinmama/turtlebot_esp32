#include "encoder_pulse_filter.h"

#include <cassert>
#include <cstdint>

int main() {
  constexpr uint32_t minimum_interval_us = 1166;
  EncoderPulseFilter filter(minimum_interval_us);
  assert(filter.accept(1000));
  assert(!filter.accept(1004));
  assert(!filter.accept(2165));  // Still only 1165 us from accepted edge.
  assert(filter.accept(2166));
  assert(!filter.accept(3000));  // 834 us after last accepted edge.
  assert(filter.accept(3332));   // Rejected edges did not reset the interval.

  EncoderPulseFilter rollover(minimum_interval_us);
  const uint32_t before_wrap = UINT32_MAX - 500;
  assert(rollover.accept(before_wrap));
  assert(!rollover.accept(before_wrap + 1165U));
  assert(rollover.accept(before_wrap + 1166U));
  return 0;
}
