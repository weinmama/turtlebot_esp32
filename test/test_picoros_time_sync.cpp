#include "../src/picoros_time_sync.h"
#include <cassert>
#include <cstdint>

int main() {
  PicoRosTimeSync clock;
  int64_t epoch = 0;
  assert(!clock.epoch(100, epoch));
  assert(!clock.update(10, 9, 1000000));
  assert(!clock.update(0, 100000001, 1000000));
  PicoRosTimeSync::UpdateResult reason;
  assert(!clock.update(0, 100000001, 1000000, &reason));
  assert(reason == PicoRosTimeSync::UpdateResult::round_trip);
  assert(!clock.update(0, 10, 0));
  assert(clock.update(1000000000, 1020000000, 1700000000000000000LL));
  assert(!clock.update(1020000000, 1020000000, 1700000000000000000LL, &reason));
  assert(reason == PicoRosTimeSync::UpdateResult::nonmonotonic);
  assert(clock.epoch(1020000000, epoch));
  assert(epoch == 1700000000010000000LL);
  assert(!clock.epoch(1019999999, epoch));
  // Holdover is allowed only while RTT/2 + conservative oscillator drift is
  // within the explicit 100 ms uncertainty budget.
  assert(clock.epoch(1020000000LL + 5000000000LL, epoch));
  assert(clock.uncertainty_ns(1020000000LL + 5000000000LL) == 15000000LL);
  assert(!clock.epoch(1020000000LL + 91000000000LL, epoch));
  clock.invalidate();
  assert(!clock.epoch(1020000000, epoch));

  // Ten simulated minutes, 100 ppm MCU drift and alternating serial jitter.
  // Query immediately before and after every correction: the old direct
  // offset replacement produces backwards sensor stamps here.
  constexpr int64_t base = 1700000000000000000LL;
  int64_t previous = 0;
  for (int64_t ms = 1000; ms <= 601000; ++ms) {
    const int64_t now = ms * 1000000;
    if (ms % 1000 == 0) {
      const int64_t rtt = (ms % 2000 == 0) ? 80000000 : 10000000;
      const int64_t host = base + now - rtt / 2 + now / 10000;
      assert(clock.update(now - rtt, now, host + (ms % 2000 ? 10000000 : -10000000)));
    }
    assert(clock.epoch(now, epoch));
    assert(epoch > previous);
    assert(epoch > base + now - 30000000 && epoch < base + now + 100000000);
    previous = epoch;
  }
  // Delayed replies, duplicate updates and large host steps cannot keep a
  // failed clock alive. Sensor publication and command gating expire together.
  assert(!clock.update(601000000000LL, 601100000001LL, base + 601050000000LL));
  assert(!clock.update(600990000000LL, 601000000000LL, base + 601000000000LL));
  assert(!clock.update(601990000000LL, 602000000000LL, base + 603000000000LL));
  assert(clock.epoch(606000000000LL, epoch));
  assert(!clock.epoch(697000000000LL, epoch));
  // Recover on the same clock instance, without invalidation or a reboot.
  assert(clock.update(697000000000LL, 697020000000LL,
                      base + 697010000000LL));
  assert(clock.epoch(697020000000LL, epoch));
}
