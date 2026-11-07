# Pico-ROS firmware

Branch: `feature/pico-ros-zenoh`, based on micro-ROS firmware `380f754`.
The default `esp32` PlatformIO target now builds **Pico-ROS over serial Zenoh**,
not micro-ROS. Arduino, board/pins, motor drivers, encoder ISRs, wheel geometry,
PID/feedforward, acceleration and wheel-speed limits, gyro calibration and
sensor/control rates are retained. No Wi-Fi configuration is needed.

## Build and flash

```sh
git submodule update --init --recursive
pio run -e esp32 -t firmware_bundle
# Local USB flashing (stop the serial router first):
pio run -e esp32 -t upload --upload-port /dev/ttyUSB0
```

Inside the backend checkout, the existing desktop build command also works:

```sh
mise run build:esp32
```

The bundle remains `.pio/build/esp32/flash-bundle/manifest.json`, with the same
bootloader/partition/application addresses consumed by the existing Pi flasher.
The manifest additionally identifies `middleware: picoros`. For remote flashing
after the one-time Pi migration below, use this branch's wrapper:

```sh
bash src/hardware/turtlebot_esp32/scripts/deploy.sh   # from the backend root
```

It builds on the desktop, stops the Pico-ROS routers/clock, delegates transfer and
verified flashing to `scripts/esp32/deploy.sh`, and restarts them after a
successful flash. It accepts the existing `PI_HOST`, `PI_SSH_PORT`,
`ESP32_DEVICE`, `ESP32_UPLOAD_SPEED` and `PI_SUDO_NONINTERACTIVE` variables.
The unwrapped old deploy command does not stop a Zenoh serial router.

## One-time Pi migration

The serial port must have one owner: the Zenoh router replaces the micro-ROS
agent. The old DDS/Zenoh bridge also stops; firmware now implements its command
arbitration and publishes wheel odometry directly into the Zenoh ROS graph.
Stop Mapping/Navigation in the UI before switching the firmware/services.

Copy this branch's `host/` files from the desktop:

```sh
ssh pi@robot-pi.local 'mkdir -p "$HOME/.local/share/turtlebox/picoros"'
scp host/{compose.yaml,router.json5,serial-router.json5,wait-router.sh,time_sync.py} \
  pi@robot-pi.local:.local/share/turtlebox/picoros/
```

On the Pi, using the desktop-built backend image already installed there:

```sh
sudo systemctl disable --now turtlebox-hardware@microros-agent.service \
  turtlebox-hardware@bridge.service
cd "$HOME/.local/share/turtlebox/picoros"
docker-compose config --quiet
# Flash from the desktop using the wrapper above; it starts these services.
# For an already-flashed MCU:
docker-compose -p turtlebot-picoros up -d
```

The default image is `turtlebox-backend:lyrical`; set `PICOROS_HOST_IMAGE` in a
`.env` file alongside the compose file if your image has another name. All
services reuse that ARM64 image, with no Pi compilation. The central router
listens on TCP 7447; a separate serial router connects to it and owns UART
`/dev/esp32` at **250000 baud**. The split isolates serial reception from the
central router's ROS routing load. Communication was tested on the Pi with a
10-minute sensor run and repeated router/ESP32 restarts. Reconnection can take
tens of seconds; motor operation was not tested in that communication check.
Multicast discovery lets the
existing Pi Zenoh peers find it. Alternatively, configure peers explicitly
with `connect/endpoints=["tcp/127.0.0.1:7447"]`. ROS domain remains **1**.

`time_sync.py` provides a one-request-per-second clock exchange only; no sensor
or motor command forwarding is involved. Keep the backend running for its
existing `/hardware/control_mode` heartbeat. Do not run the old agent or bridge
alongside this firmware. UART0 carries binary Zenoh exclusively: a serial
monitor or text logging on that port interferes with transport.

For rollback, stop these compose services, flash the `main` branch's micro-ROS
firmware with the original deploy tool, then re-enable the two old services.

## ROS interface

| Topic | Type | Direction / rate |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/msg/TwistStamped` | manual command input |
| `/hardware/nav_cmd_vel` | `geometry_msgs/msg/TwistStamped` | navigation command input |
| `/hardware/control_mode` | `std_msgs/msg/String` | `idle`, `mapping`, `navigation`, `blocked` heartbeat |
| `/wheel/odom` | `nav_msgs/msg/Odometry` | wheel odometry, 10 Hz |
| `/imu/data` | `sensor_msgs/msg/Imu` | calibrated IMU, 20 Hz |
| `/motor/state` | `std_msgs/msg/Float32MultiArray` | original seven diagnostics, 10 Hz |
| `/pid_tune` | `std_msgs/msg/Float32MultiArray` | three shared or six independent PID gains |
| `/rosout` | `rcl_interfaces/msg/Log` | low-rate latest log, volatile |
| `/hardware/time/request`, `/hardware/time/reply` | `std_msgs/msg/Int64MultiArray` | clock exchange, 1 Hz |
| `/hardware/reset_diagnostic` | `std_msgs/msg/Int64MultiArray` | boot ID, ESP-IDF reset reason, intentional restart marker; every 10 s |
| `/hardware/clock_diagnostic` | `std_msgs/msg/Int64MultiArray` | clock exchange counters, timing and rejection reasons; 1 Hz |

The clock diagnostic values are `[boot_id, requests, matched, accepted,
unmatched, last_rtt_ns, max_rtt_ns, last_result, valid, invalid_count,
rtt_rejected, nonmonotonic_count, step_rejected, last_reject]`. Result codes
are 0 accepted, 1 invalid, 2 round-trip above 100 ms, 3 nonmonotonic, and 4
clock correction above 200 ms; -1 means no result/rejection yet. The counters
are since the current boot. They report MCU-observed timing, not an independent
measurement of the Pi clock's correctness. Clock thresholds and watchdogs are
unchanged. This diagnostic adds one low-rate message per second and must be
evaluated for its own effect on serial load.

The reset diagnostic is a best-effort observation, not a safety control. Its
three values are `[boot_id, esp_reset_reason, software_marker]`; boot ID matches
the clock exchange. ESP-IDF reason values include 1 power-on, 3 software,
4 panic, 5 interrupt watchdog, 6 task watchdog, 7 other watchdog and 9 brownout.
The software marker is only meaningful for reason 3: `0x54520101` IMU init,
`0x54520102` transport init, `0x54520103` TX task creation, and `0x54520104`
clock invalidation. Zero or an unknown marker is inconclusive. It is stored in
RTC no-init memory before a deliberate `esp_restart` and cleared at startup; power loss
may erase it. No serial-console output is needed. A newly flashed diagnostic
build is required to observe these fields; it cannot identify earlier resets.

Transport diagnostics are **disabled by default** to avoid extra serial traffic
and formatting work while driving. For investigation, add
`-DROBOT_TRANSPORT_DIAGNOSTICS=1` to PlatformIO `build_flags` and rebuild; this
emits a once-per-second `/rosout` `cmd_diag` summary after command or transport
activity. For each source (`m` manual, `n` navigation), the slash-separated
counts are received/accepted/invalid/overwritten/unselected/locally-stale/replayed;
`r` lists guard rejections in this order: clock, sensors, missing heartbeat,
future heartbeat, stale heartbeat, non-finite command, future command stamp,
stale command stamp, future receive time, stale delivery, mode mismatch. `a`
is last/max calculated command age in milliseconds. `odom` and `imu` each show
publish-call success/failure, stale samples dropped, and mailbox overwrites.
`svc` shows the maximum interval between TX-service calls and maximum service
execution time in microseconds. `pub` gives maximum synchronous publisher-call
duration in microseconds, ordered odom/imu/motor-state/rosout/time-request.
Publish success means accepted by the Pico-ROS API, not confirmed at the Pi.

The former DDS-side `/odom` becomes `/wheel/odom` directly, matching the old
bridge output and existing EKF input. `/odom` remains the EKF output. For an
independent setup, `ROBOT_ODOM_TOPIC` is a compile-time override. Frames remain
`odom` → `base_link` and `imu_link`; covariance values are preserved.

Commands need a fresh ROS epoch stamp, synchronized clock, calibrated/fresh
sensors and a current mode heartbeat. For example, with the backend in idle or
mapping mode, using ROS with `RMW_IMPLEMENTATION=rmw_zenoh_cpp`, domain 1:

```sh
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/TwistStamped \
  '{header: auto, twist: {linear: {x: 0.05}, angular: {z: 0.0}}}'
ros2 topic echo /wheel/odom
```

The first command moves the robot. Startup calibration requires about five
stationary seconds before commands are enabled. Stopping command publication
brakes after 500 ms; sending zero brakes at the next control update.

For standalone use **without the backend**, run `host/time_sync.py
--standalone-manual` instead of the normal clock process. This explicitly
provides the otherwise absent idle heartbeat. Never run this mode alongside
the backend's mode publisher. Manual commands are blocked in navigation mode;
navigation commands require navigation mode. Mode changes invalidate old commands.

## Implementation and checks

### Guarded movement probe

Run a single bounded probe from the desktop against the backend container on
the Pi (the script performs a 10 s passive preflight, streams JSONL diagnostics,
sends zero commands on completion or abort, then observes the stop for 10 s):

```sh
ssh pi@robot-pi.local \
  'docker exec -i turtlebox-hardware-backend /hardware-entrypoint.sh python3 - \
    --linear-x 0.025 --angular-z -0.25 --duration 2' \
  < host/robot_validation_probe.py \
  > /tmp/opencode/robot-validation-left-wheel.jsonl
```

That example requests only the left wheel in the current differential-drive
convention. The tool rejects inputs outside ±0.05 m/s, ±0.3 rad/s and 5 s per
probe. Constant probes remain limited to 5 s. For longer guarded sequences,
`--profile mixed --duration 30` runs six 5 s phases: straight forward, left and
right curves, opposite-direction wheel spins, and reverse, each followed by a
0.25 s zero-command pause. The same profile accepts durations up to 600 s and
varies its speed scale every 30 s. It aborts on stale sensors/motor feedback, mode changes, foreign command
echoes, more than three encoder samples over 140 RPM in a rolling second, PWM
over 255, or 500 ms of requested wheel speed without encoder response. The
firmware excludes each >140 RPM sample from odometry and PID feedback, holding
the last valid measured RPM for PID; the requested RPM is never substituted as
feedback. A fourth outlier in one second brakes both motors and latches motion
disabled until an operator restarts the MCU. Raw RPM, pulse counts, ISR counts
and minimum inter-edge intervals remain visible in telemetry/diagnostic logs.
The GPIO diagnostic path rejects edges closer than 165 µs, derived from the
configured 514.8 pulses/revolution and 140 RPM encoder guard. Raw, accepted and
rejected edges are counted separately; rejected edges do not reset the
last-accepted-edge timer. ESP32 PCNT units 0/1 count each A rising edge and
sample B in hardware for direction, with the maximum 1023-cycle (12.8 µs)
hardware pulse-width filter. PCNT signed count differences drive wheel
odometry; the units remain running between reads and are cleared only near
their counter limits. GPIO A-edge timestamps provide a median period estimate
for PID and the 140 RPM feedback guard. It rejects only
sub-200-µs electrical glitches, independently of the 130 RPM target limit,
and does not convert rejected edges into synthetic RPM. PCNT count rate is
used during estimator warm-up. `encoder_trace` reports PCNT/GPIO counts and
estimated RPM.

Firmware publishes `/hardware/encoder/status` (`ready` or a latched fault).
A missing, stale or non-ready status blocks host hardware readiness and
guarded validation.

The host probe's `rpm_sweep` profile is a guarded 180-second straight-wheel
test with six 30-second stages at 10, 25, 40, 60, 80 and 100 RPM, including a
0.5-second zero-command pause at the end of each stage. It retains all normal
freshness, feedback, target, PWM and stop-confirmation checks.
The `maneuver_sweep` profile is exactly 180 seconds and ramps through 40, 60,
80 and 100 wheel RPM forward and reverse, then exercises forward/reverse curves
and both in-place rotation directions. Its wheel targets remain at or below
100 RPM; the usual 130 RPM command ceiling, 140 RPM encoder guard, sensor and
motor feedback checks, abort behavior, zero-command cleanup and stop confirmation
remain active. This is a moving robot test: run it only with a clear test area
and immediately reachable emergency stop.
The `rpm_hold` profile allows a bounded 0.1–5 second hold up to the 130 RPM
target limit. The encoder feedback guard remains 140 RPM; communication and
no-feedback watchdogs are unchanged. The probe reports whether stillstand was confirmed; missing telemetry is never
counted as a confirmed stop. Host tests exercise these guards without
connecting to the robot.

Example invocations (run one at a time and inspect the complete JSONL log
before proceeding to a longer test):

```sh
ssh pi@robot-pi.local \
  'docker exec -i turtlebox-hardware-backend /hardware-entrypoint.sh python3 - \
    --profile mixed --duration 30' \
  < host/robot_validation_probe.py \
  > /tmp/opencode/robot-validation-mixed-30s.jsonl

# Only after reviewing a clean 30 s result; use 600 for a 10-minute supervised soak:
ssh pi@robot-pi.local \
  'docker exec -i turtlebox-hardware-backend /hardware-entrypoint.sh python3 - \
     --profile mixed --duration 600' \
  < host/robot_validation_probe.py \
  > /tmp/opencode/robot-validation-mixed-10min.jsonl

# Supervised three-minute speed and maneuver test (moves the robot):
ssh pi@robot-pi.local \
  'docker exec -i turtlebox-hardware-backend /hardware-entrypoint.sh python3 - \
     --profile maneuver_sweep --duration 180' \
  < host/robot_validation_probe.py \
  > /tmp/opencode/robot-validation-maneuver-sweep.jsonl
```

* Pico-ROS and its Zenoh-Pico/Micro-CDR dependencies are pinned submodules.
  `scripts/generate_types.py` derives message layouts and RIHS hashes from
  installed Lyrical type descriptions. Generated types are committed.
* `scripts/patch_zenoh.py` fixes the pinned dependency's handling of adjacent
  transport frames in a batch (e.g. mixed Best-Effort/Reliable traffic). Both
  builds use the same patched source without modifying the submodule. A
  dedicated regression test verifies the frame boundary.
* Motor/sensor work stays on the Arduino task. A separate TX task and bounded
  latest-value mailboxes keep serial backpressure away from motor regulation.
* `scripts/patch_arduino_uart.py` replaces the Arduino UART data path with
  `include/picoros_uart.h`. Arduino-ESP32 2.0.17 holds the same HAL UART mutex
  in `uartWriteBuf`, `uartAvailable` and `uartReadBytes`. With synchronous TX,
  an outgoing telemetry frame therefore blocks incoming commands/heartbeats;
  the upstream 256-byte RX buffer also cannot absorb a full discovery frame.
  The adapter retains HardwareSerial initialization/teardown but uses IDF's
  independent RX/TX synchronization for transfers and a 4096-byte RX buffer.
  TX remains synchronous without an added queue. Short writes/errors propagate
  to Zenoh's serial protocol; UART0 remains exclusively binary Zenoh.
* Commands reject non-finite components, malformed CDR, invalid timestamps,
  timestamps older than 300 ms or more than 200 ms in the future, repeated
  timestamps, and queued delivery older than 200 ms. Heartbeat and command
  watchdogs are each 500 ms. PID arrays are bounded and validated.
* Time requests contain a random boot ID, increasing request ID and monotonic
  send time. Replies must match the pending request exactly and arrive within
  100 ms. Matching replies are consumed once. Sensor stamps use the estimated
  midpoint offset; clock loss stops command acceptance after five seconds.
* The two-second task watchdog covers startup handshakes too. Lost host clock
  service/session causes braking and a restart/reconnection attempt. Invalid
  IMU data is suppressed, and stale sensors prevent motor commands.

Run all host checks in an isolated container (never on the robot):

```sh
bash scripts/test.sh
```

`PICOROS_TEST_IMAGE` can select another Lyrical image containing CMake, a C/C++
compiler, rclpy, rmw_zenoh_cpp and standard message packages. Default:
`turtlebot-backend:test`. Checks cover gyro calibration, clock/command guards,
malformed CDR, batched transport frames, real ROS discovery and bidirectional messages, command routing,
timeout/replay/future/NaN rejection, PID tuning and clock recovery. The same
transport source runs against the real ROS Zenoh router over both TCP and a
serial PTY pair, including COBS framing and serial handshakes. A third run uses
separate TCP and serial routers, matching the deployed topology.

The ESP32 image and flash bundle are cross-built. Host integration uses
synthetic sensor data and no motor drivers. PTYs do not model physical UART
timing (the upstream POSIX driver uses 230400 in tests; the ESP32/Pi setting is
250000). Hardware communication tests do not validate motor operation; neither
physical sensor timing nor motor operation are implied by host tests.

Validated locally: seven CTest checks, TCP and serial-PTY integration suites,
the `esp32` firmware build/flash bundle, and the Compose configuration. The
firmware uses about 418 kB flash and 28 kB static RAM (runtime heap/stacks are
additional).

Pi deployment note: the Zenoh router container must receive the serial device
mapping `/dev/esp32:/dev/esp32`; without it, the router starts but cannot open
the ESP32 link and the hardware monitor correctly reports stale sensors.

### UART contention regression (2026-11-06 UTC)

Before the duplex fix, changing IMU Zenoh transport reliability to Best Effort
alone did not fix the failure: the 3-cycle forward/reverse test aborted on its
fourth leg at 302.8 ms odometry age, with TX service up to 148.2 ms, COBS errors,
and a subsequent telemetry outage. The reliability option controls the Zenoh
transport; it does not change Pico-ROS's separately advertised ROS graph QoS.

After deploying the UART adapter, two complete runs at 0.05 m/s each finished
all six legs (target one encoder-derived revolution per wheel per leg):

| Metric | Run 1 | Run 2 |
|---|---:|---:|
| Maximum received odometry stamp age | 72.02 ms | 61.27 ms |
| Maximum received IMU stamp age | 75.58 ms | 67.19 ms |
| Maximum TX-service duration | 50.652 ms | 49.500 ms |
| Maximum command age reported by firmware | 26.5 ms | 24.0 ms |
| Sensor publish failures/stale drops/mailbox overwrites | 0/0/0 | 0/0/0 |

The final motor targets, measured RPM and PWM were zero. Encoder-derived
travel includes stopping overshoot (about 1.02–1.13 revolutions per wheel in
these runs); it is not an independent measurement of ground travel. The
300 ms test feedback limit and firmware watchdogs were not relaxed.
Synchronous odometry publication still takes about 29–35 ms at 250000 baud;
the fix lets reception proceed concurrently rather than hiding this wire time.

The new host UART regression test keeps a simulated IDF TX call blocked while
RX receives fragmented binary data, and checks error/short-write propagation.
It verifies the adapter contract, not the physical UART. Hardware captures are
under `/tmp/opencode/uart-duplex-*`, including the two `three-roundtrips` logs.

### Timebase and damaged-frame recovery

Clock offset corrections are slewed rather than stepped, so periodic Pi clock
updates cannot make ROS sensor stamps move backwards. A reply with an RTT above
100 ms, a delayed/replayed exchange, or an offset jump above 200 ms is ignored;
it does not refresh clock liveness. The mapped clock expires after five seconds
without a valid exchange, and the existing command/heartbeat watchdogs remain
at 500 ms. These are intentional fail-safe limits, not values to widen to mask
link loss.

The firmware's Pico-ROS serial reader drains a malformed/oversized COBS frame to
its delimiter, checks the framing and CRC, and resumes at the next valid frame.
This prevents one damaged inbound frame from permanently ending command
reception. A damaged or missing command still expires through the unchanged
500-ms watchdog. Its host test injects malformed, truncated, oversized and
single-byte-corrupted frames before a valid frame. The isolated integration
soak can additionally run with `PICOROS_SOAK_SECONDS=600 bash scripts/test.sh`;
it rate-limits the PTY relay to approximate 250000-baud 8N1 and exercises ten
minutes of fresh command/telemetry exchange with periodic damaged frames. PTYs
do not reproduce electrical noise or establish physical motor behavior.

Transport diagnostics are off in normal builds to avoid adding a periodic
`/rosout` frame while controlling. Enable them only for a specific investigation.
