#include "encoder_speed_estimator.h"
#include "encoder_pulse_filter.h"

#include <cassert>
#include <cmath>
#include <cstdint>

int main() {
  constexpr float ppr = 514.8f;
  EncoderSpeedEstimator estimator;
  uint32_t edge = 100000;
  for (int i = 0; i < 5; ++i) {
    assert(estimator.record(edge));
    edge += 1166;
  }
  assert(estimator.record(edge));
  assert(estimator.ready(edge));
  const float at_100_rpm = estimator.rpm(edge, ppr);
  assert(at_100_rpm > 99.8f && at_100_rpm < 100.1f);

  // Narrow electrical spikes do not enter the period history. A single
  // plausible-width extra edge cannot dominate the regular-period median.
  const uint32_t next_real_edge = edge + 1166;
  assert(!estimator.record(edge + 10));
  assert(!estimator.record(edge + 100));
  assert(estimator.record(edge + 500));
  assert(estimator.record(next_real_edge));
  assert(estimator.rpm(next_real_edge, ppr) < 101.0f);

  // Sustained faster periods remain visible instead of being capped at 100 RPM.
  EncoderSpeedEstimator overspeed;
  edge = 200000;
  for (int i = 0; i < 8; ++i) {
    assert(overspeed.record(edge));
    edge += 1100;
  }
  assert(overspeed.rpm(edge - 1100, ppr) > 100.0f);

  // A burst of plausible-width extra edges can occupy the complete five-period
  // window. The median alone cannot distinguish this from genuine overspeed:
  // keep that ambiguity visible to the existing feedback safety guard.
  EncoderSpeedEstimator burst;
  edge = 300000;
  for (int i = 0; i < 6; ++i) {
    assert(burst.record(edge));
    edge += 1166;
  }
  for (int i = 0; i < 5; ++i) {
    assert(burst.record(edge));
    edge += 600;
  }
  const float burst_rpm = burst.rpm(edge - 600, ppr);
  assert(burst_rpm > 190.0f && burst_rpm < 200.0f);
  for (int i = 0; i < 6; ++i) {
    edge += 1166;
    assert(burst.record(edge));
  }
  assert(burst.rpm(edge, ppr) > 99.0f && burst.rpm(edge, ppr) < 101.0f);

  // Match the firmware ISR path: only edges accepted by the physical-rate
  // filter reach the period estimator. Rejected raw edges may be separated by
  // >200 us and would corrupt the estimator if recorded before this filter.
  EncoderSpeedEstimator filtered_estimator;
  EncoderPulseFilter pulse_filter(825);
  edge = 400000;
  assert(pulse_filter.accept(edge));
  filtered_estimator.record(edge);
  for (int i = 0; i < 8; ++i) {
    edge += 1166;
    assert(pulse_filter.accept(edge));
    filtered_estimator.record(edge);
    const uint32_t glitch = edge + 600;
    assert(!pulse_filter.accept(glitch));
    // Intentional: rejected edges are not passed to filtered_estimator.
  }
  assert(filtered_estimator.ready(edge));
  const float filtered_rpm = filtered_estimator.rpm(edge, ppr);
  assert(filtered_rpm > 99.8f && filtered_rpm < 100.1f);

  // Timestamps and interval subtraction remain correct across micros wrap.
  EncoderSpeedEstimator rollover;
  edge = UINT32_MAX - 3000;
  for (int i = 0; i < 6; ++i) {
    assert(rollover.record(edge));
    edge += 1166;
  }
  assert(std::fabs(rollover.rpm(edge - 1166, ppr) - at_100_rpm) < 0.1f);

  // Stale data and a later restart of edge flow must not reuse the old periods.
  assert(rollover.rpm(edge + EncoderSpeedEstimator::kStaleAfterUs + 1, ppr) == 0.0f);
  assert(rollover.record(edge + EncoderSpeedEstimator::kStaleAfterUs + 1000));
  assert(!rollover.ready(edge + EncoderSpeedEstimator::kStaleAfterUs + 1000));
  rollover.reset();
  assert(!rollover.ready(edge));
  return 0;
}
