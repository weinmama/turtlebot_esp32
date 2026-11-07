#include "../src/picoros_command_guard.h"
#include <cassert>
#include <limits>

int main() {
  using Guard = PicoRosCommandGuard;
  Guard g;
  assert(g.should_stop(0, true, true));
  assert(!g.accept(Guard::Source::manual, 0.1f, 0.0f, 0, 0, true, true));
  assert(g.rejection_reason(Guard::Source::manual, 0.1f, 0.0f, 0, 0, true, true) ==
         Guard::RejectReason::heartbeat_missing);
  g.heartbeat(Guard::Mode::mapping, 100);
  assert(g.rejection_reason(Guard::Source::navigation, 0.1f, 0.0f, 0, 100, true, true) ==
         Guard::RejectReason::mode_mismatch);
  assert(!g.accept(Guard::Source::navigation, 0.1f, 0.0f, 0, 100, true, true));
  assert(g.rejection_reason(Guard::Source::manual, 0.1f, 0.0f, 0, 100, true, false) ==
         Guard::RejectReason::sensors_not_ready);
  assert(!g.accept(Guard::Source::manual, std::numeric_limits<float>::quiet_NaN(), 0.0f, 0, 100, true, true));
  assert(g.rejection_reason(Guard::Source::manual, std::numeric_limits<float>::quiet_NaN(), 0.0f,
                            0, 100, true, true) == Guard::RejectReason::nonfinite_command);
  assert(!g.accept(Guard::Source::manual, 0.1f, 0.0f, Guard::kMaxCommandAgeNs + 1, 100, true, true));
  assert(g.rejection_reason(Guard::Source::manual, 0.1f, 0.0f, Guard::kMaxCommandAgeNs + 1,
                            100, true, true) == Guard::RejectReason::command_stale);
  assert(g.rejection_reason(Guard::Source::manual, 0.1f, 0.0f, -Guard::kFutureToleranceNs - 1,
                            100, true, true) == Guard::RejectReason::command_from_future);
  assert(!g.accept(Guard::Source::manual, 0.1f, 0.0f, 0, 100, false, true));
  assert(g.rejection_reason(Guard::Source::manual, 0.1f, 0.0f, 0, 100, false, true) ==
         Guard::RejectReason::clock_invalid);
  assert(g.rejection_reason(Guard::Source::manual, 0.1f, 0.0f, 0, 100, true, true, 101) ==
         Guard::RejectReason::receive_from_future);
  g.heartbeat(Guard::Mode::mapping, 300);
  assert(g.rejection_reason(Guard::Source::manual, 0.1f, 0.0f, 0, 301, true, true, 100) ==
         Guard::RejectReason::delivery_stale);
  assert(g.rejection_reason(Guard::Source::manual, 0.1f, 0.0f, 0, 299, true, true) ==
         Guard::RejectReason::heartbeat_from_future);
  g.heartbeat(Guard::Mode::mapping, 100);
  assert(g.accept(Guard::Source::manual, 0.1f, 0.0f, 0, 100, true, true));
  assert(g.rejection_reason(Guard::Source::manual, 0.1f, 0.0f, 0, 601, true, true) ==
         Guard::RejectReason::heartbeat_stale);
  assert(!g.should_stop(600, true, true));
  assert(g.should_stop(601, true, true));
  assert(g.should_stop(101, false, true));
  assert(g.should_stop(101, true, false));
  g.heartbeat(Guard::Mode::navigation, 700);
  assert(g.should_stop(700, true, true)); // old manual command must not survive a mode switch
  assert(!g.accept(Guard::Source::manual, 0.1f, 0.0f, 0, 700, true, true));
  assert(g.accept(Guard::Source::navigation, 0.1f, 0.0f, 0, 700, true, true));
  g.heartbeat(Guard::Mode::blocked, 701);
  assert(g.should_stop(701, true, true));
  g.invalidate();
  assert(g.should_stop(701, true, true));
  // A heartbeat can arrive after the command but before the control task runs.
  // Validate heartbeat against processing time, timeout against receive time.
  g.heartbeat(Guard::Mode::mapping, 1005);
  assert(g.accept(Guard::Source::manual, 0.1f, 0, 0, 1010, true, true, 1000));
  g.heartbeat(Guard::Mode::mapping, 1490);
  assert(!g.should_stop(1500, true, true));
  assert(g.should_stop(1501, true, true));
}
