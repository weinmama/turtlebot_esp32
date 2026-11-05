# Pico-ROS port (work in progress, not safe to flash)

Branch `feature/pico-ros-zenoh` begins at the existing micro-ROS firmware.
The existing PlatformIO target and `src/main.cpp` remain micro-ROS; the
Pico-ROS motor-control port is **not implemented** and must not be used to
control hardware yet.

The intended transport is the existing ESP32 serial connection to a Zenoh
router on the Pi. The upstream Pico-ROS ESP32 serial example targets ESP32-S3
and ESP-IDF; serial support and pins for this project's ESP32-D0WDQ6,
Arduino/PlatformIO and 460800 baud still need to be established and tested.
Do not assume binary compatibility between Jazzy Pico-ROS examples and the
robot's Lyrical `rmw_zenoh`.

The current firmware uses micro-ROS time synchronization. The standalone
`src/picoros_time_sync.h` provides the beginning of a bounded-RTT monotonic to
epoch conversion, but it is not connected to Pico-ROS or a Pi responder. The
request/reply protocol must include a unique request ID and reject unmatched,
duplicate and stale responses; transport, clock error, drift and reconnects
must be tested before timestamps or commands are trusted. No sensor data or
motion commands may be enabled on an expired or unverified clock.

Before claiming parity, the port must preserve and test:

* `sensor_msgs/Imu`, `nav_msgs/Odometry`, motor diagnostics, PID tuning and
  `geometry_msgs/TwistStamped` topic names, type hashes, frames, covariance,
  QoS and time stamps, including the existing gyro calibration.
* The 500 ms command timeout, watchdog, motor braking, wheel speed limits,
  invalid-command rejection and safe behavior on stalls and disconnects.
* The current host bridge's manual/navigation arbitration, heartbeat and
  command age checks. These must not disappear merely because the DDS/Zenoh
  bridge is removed.
* ROS domain, reconnection, packet fragmentation, serial bandwidth, scheduler
  deadlines, and CPU and message-age benchmarks compared with micro-ROS.

Host-only regression test (does not touch the robot):

```sh
g++ -std=c++17 -Wall -Wextra -Werror test/test_picoros_time_sync.cpp -o /tmp/test_picoros_time_sync
/tmp/test_picoros_time_sync
```

Reference: https://github.com/Pico-ROS/Pico-ROS-software and
https://github.com/Pico-ROS/picoros-examples/tree/master/esp32-joystick
