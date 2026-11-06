#include "picoros_clock_ns.h"
#include <assert.h>
#include <stdint.h>

int main(void) {
  int32_t seconds = 71;
  int32_t nanoseconds = 123456789;
  assert(picoros_clock_ns(seconds, nanoseconds) == INT64_C(71123456789));
  assert(picoros_clock_ns(INT32_MAX, 999999999) == INT64_C(2147483647999999999));
  return 0;
}
