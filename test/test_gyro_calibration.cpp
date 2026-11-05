#include "../src/gyro_calibration.h"
#include <cassert>
#include <cmath>
#include <limits>

int main() {
  GyroCalibration c;
  for (unsigned i = 0; i < 99; ++i) c.sample(0.02f, -0.03f, -0.01f, true);
  assert(!c.ready());
  c.sample(0.02f, -0.03f, -0.01f, false); // motion invalidates the entire window
  for (unsigned i = 0; i < 99; ++i) c.sample(0.02f, -0.03f, -0.01f, true);
  assert(!c.ready());
  c.sample(std::numeric_limits<float>::quiet_NaN(), 0.0f, 0.0f, true);
  for (unsigned i = 0; i < 100; ++i) c.sample(0.02f, -0.03f, -0.01f, true);
  assert(c.ready());
  assert(std::fabs(c.bias(0) - 0.02f) < 0.00001f);
  assert(std::fabs(c.bias(1) + 0.03f) < 0.00001f);
  assert(std::fabs(c.bias(2) + 0.01f) < 0.00001f);
  c.sample(0.14f, 0.0f, 0.0f, false); // never recalibrate an established bias in motion
  assert(std::fabs(c.bias(2) + 0.01f) < 0.00001f);

  GyroCalibration noisy;
  for (unsigned i = 0; i < 100; ++i) noisy.sample(0.0f, i % 2 ? 0.04f : -0.04f, 0.0f, true);
  assert(!noisy.ready());
  for (unsigned i = 0; i < 100; ++i) noisy.sample(0.02f, 0.0f, 0.0f, true);
  assert(noisy.ready());
}
