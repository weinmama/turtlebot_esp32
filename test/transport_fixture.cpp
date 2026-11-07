// Runs the exact firmware transport on Linux. No motor/robot hardware is opened.
#include "robot_transport.h"
#include <chrono>
#include <cstdio>
#include <thread>
#include <cstdlib>

int main(int argc, char** argv) {
  setvbuf(stdout, nullptr, _IONBF, 0);
  if (argc != 2) return 2;
  // Deliberately process-lifetime: network callbacks are active until _Exit.
  auto* transport = new RobotTransport;
  if (!transport->begin(argv[1])) return 3;
  std::thread([transport] {
    for (;;) { transport->service(); std::this_thread::sleep_for(std::chrono::milliseconds(5)); }
  }).detach();
  ros_Odometry odom{};
  odom.header.frame_id = const_cast<char*>("odom");
  odom.child_frame_id = const_cast<char*>("base_link");
  odom.pose.pose.position.x = 1.25;
  odom.pose.pose.orientation.w = 1;
  odom.twist.covariance[0] = 0.0025;
  ros_Imu imu{};
  imu.header.frame_id = const_cast<char*>("imu_link");
  imu.orientation_covariance[0] = -1;
  imu.angular_velocity.z = -0.001;
  imu.linear_acceleration.z = 9.81;
  float state[7] = {};
  ros_Float32MultiArray diagnostics{};
  diagnostics.data.data = state; diagnostics.data.n_elements = 7;
  transport->publish_encoder_status("ready");
  bool was_stopped = true;
  for (int tick = 0; tick < 12000; ++tick) {
    ros_TwistStamped command{};
    if (transport->command(command, true)) {
      printf("COMMAND %.6f %.6f\n", command.twist.linear.x, command.twist.angular.z);
      fflush(stdout);
    }
    const bool stopped = transport->should_stop(true);
    if (stopped != was_stopped) { printf("STOP %d\n", stopped); fflush(stdout); was_stopped = stopped; }
    float gains[6]; size_t count;
    if (transport->tuning(gains, count)) { printf("TUNING %zu %.3f\n", count, gains[0]); fflush(stdout); }
    int64_t stamp;
    if (tick % 10 == 0 && transport->epoch(stamp)) {
      odom.header.stamp = {int32_t(stamp / 1000000000), uint32_t(stamp % 1000000000)};
      transport->publish(odom);
    }
    if (tick % 5 == 0 && transport->epoch(stamp)) {
      imu.header.stamp = {int32_t(stamp / 1000000000), uint32_t(stamp % 1000000000)};
      transport->publish(imu);
      state[0] = stopped ? 1 : 0;
      transport->publish(diagnostics);
      transport->publish_encoder_status("ready");
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  std::_Exit(0);
}
