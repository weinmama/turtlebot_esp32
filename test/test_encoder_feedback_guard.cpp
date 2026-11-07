#include "encoder_feedback_guard.h"

#include <cassert>
#include <cmath>

int main() {
  EncoderFeedbackGuard guard;
  auto result = guard.observe(0, 14.0f, 0.0f);
  assert(!result.left_invalid && !result.right_invalid && !result.latched);

  // Three isolated samples beyond the 140 RPM guard are excluded but tolerated.
  result = guard.observe(100, 209.0f, 0.0f);
  assert(result.left_invalid && !result.latched && result.anomalies_in_window == 1);
  result = guard.observe(300, 14.0f, 0.0f);
  assert(!result.left_invalid && !result.latched && result.anomalies_in_window == 1);
  result = guard.observe(500, 0.0f, -141.0f);
  assert(result.right_invalid && !result.latched && result.anomalies_in_window == 2);
  result = guard.observe(900, 180.0f, 0.0f);
  assert(result.left_invalid && !result.latched && result.anomalies_in_window == 3);

  // Fourth anomaly within the rolling second latches, regardless of wheel.
  result = guard.observe(1000, 0.0f, 141.0f);
  assert(result.right_invalid && result.latched && result.anomalies_in_window == 4);
  result = guard.observe(1001, 14.0f, 0.0f);
  assert(result.latched);

  // A separate guard prunes old events and accepts exactly three in a window.
  EncoderFeedbackGuard rolling;
  rolling.observe(0, NAN, 0.0f);
  rolling.observe(100, 141.0f, 0.0f);
  rolling.observe(200, 0.0f, 141.0f);
  result = rolling.observe(1001, 141.0f, 0.0f);
  assert(!result.latched && result.anomalies_in_window == 3);
  return 0;
}
