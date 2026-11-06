#include "../src/picoros_command_guard.h"
#include <cassert>
#include <limits>

int main() {
  using Guard = PicoRosCommandGuard;
  Guard g;
  assert(g.should_stop(0, true, true));
  assert(!g.accept(Guard::Source::manual, 0.1f, 0.0f, 0, 0, true, true));
  g.heartbeat(Guard::Mode::mapping, 100);
  assert(!g.accept(Guard::Source::navigation, 0.1f, 0.0f, 0, 100, true, true));
  assert(!g.accept(Guard::Source::manual, std::numeric_limits<float>::quiet_NaN(), 0.0f, 0, 100, true, true));
  assert(!g.accept(Guard::Source::manual, 0.1f, 0.0f, Guard::kMaxCommandAgeNs + 1, 100, true, true));
  assert(!g.accept(Guard::Source::manual, 0.1f, 0.0f, 0, 100, false, true));
  assert(g.accept(Guard::Source::manual, 0.1f, 0.0f, 0, 100, true, true));
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
