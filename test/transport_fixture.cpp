// Runs the exact firmware transport on Linux. No motor/robot hardware is opened.
#include "robot_transport.h"
#include "picoros_reply_reannounce.h"
#include <chrono>
#include <cstdio>
#include <thread>
#include <cstdlib>
#include <cstring>

// Test-only fault injection: remove the router's interest but retain the MCU
// callback. The host can still observe replies, exactly the failure boundary
// the firmware's wire-interest recovery is meant to repair.
struct RobotTransportTestAccess {
  static bool withdraw_time_interest(RobotTransport& transport) {
    const auto* sub = z_subscriber_loan(&transport.time_sub_.zsub);
    auto session = _z_session_weak_upgrade_if_open(&sub->_zn);
    if (_Z_RC_IS_NULL(&session)) return false;
    _z_network_message_t message;
    auto declaration = _z_make_undecl_subscriber(sub->_entity_id, nullptr);
    _z_n_msg_make_declare(&message, declaration, _z_optional_id_make_none());
    const auto result = _z_send_undeclare(_Z_RC_IN_VAL(&session), &message);
    _z_n_msg_clear(&message);
    _z_session_rc_drop(&session);
    return result == _Z_RES_OK;
  }
};

int main(int argc, char** argv) {
  setvbuf(stdout, nullptr, _IONBF, 0);
  if (argc != 2) return 2;
  // Deliberately process-lifetime: network callbacks are active until _Exit.
  auto* transport = new RobotTransport;
  transport->set_reset_diagnostic(7, 0x54520104);
  if (!transport->begin(argv[1])) return 3;
  std::thread([transport] {
    char input[64];
    while (fgets(input, sizeof(input), stdin)) {
      if (!strcmp(input, "WITHDRAW_TIME_INTEREST\n"))
        printf("CLOCK_WIRE_WITHDRAWN %d\n", RobotTransportTestAccess::withdraw_time_interest(*transport));
    }
  }).detach();
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
  for (uint64_t tick = 0; ; ++tick) {
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
      state[0] = stopped ? 1 : 0;
      transport->publish(diagnostics);
      transport->publish_encoder_status("ready");
    }
    if (tick % 5 == 0 && transport->epoch(stamp)) {
      imu.header.stamp = {int32_t(stamp / 1000000000), uint32_t(stamp % 1000000000)};
      transport->publish(imu);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  std::_Exit(0);
}
