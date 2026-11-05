#include "../src/picoros_time_sync.h"
#include <cassert>
#include <cstdint>

int main() {
  PicoRosTimeSync clock;
  int64_t epoch = 0;
  assert(!clock.epoch(100, epoch));
  assert(!clock.update(10, 9, 1000000));
  assert(!clock.update(0, 100000001, 1000000));
  assert(!clock.update(0, 10, 0));
  assert(clock.update(1000000000, 1020000000, 1700000000000000000LL));
  assert(clock.epoch(1020000000, epoch));
  assert(epoch == 1700000000010000000LL);
  assert(!clock.epoch(1019999999, epoch));
  assert(!clock.epoch(1020000000LL + PicoRosTimeSync::kMaxAgeNs + 1, epoch));
  clock.invalidate();
  assert(!clock.epoch(1020000000, epoch));
}
