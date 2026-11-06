#pragma once

#include <stdint.h>

// Cast before multiplication: ESP32's time_t can be a 32-bit long.
static inline int64_t picoros_clock_ns(int64_t seconds, int64_t nanoseconds) {
  return seconds * INT64_C(1000000000) + nanoseconds;
}
