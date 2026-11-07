#pragma once

#include "picoros.h"
#include "robot_cdr.h"
#include "picoros_time_sync.h"
#include "picoros_command_guard.h"
#include <cstddef>
#include <cstdint>
#include <mutex>

// Transport timing diagnostics generate serial traffic and are for investigation only.
#ifndef ROBOT_TRANSPORT_DIAGNOSTICS
#define ROBOT_TRANSPORT_DIAGNOSTICS 0
#endif

#ifndef ROBOT_ZENOH_LOCATOR
#define ROBOT_ZENOH_LOCATOR "serial/UART_0#baudrate=250000"
#endif
#ifndef ROBOT_ODOM_TOPIC
#define ROBOT_ODOM_TOPIC "wheel/odom"
#endif

// One instance per process (Pico-ROS itself has one session). RX callbacks and
// service() run on communication tasks; only command()/should_stop() run on the
// motor task. Latest-value mailboxes keep serial backpressure out of that task.
class RobotTransport {
 public:
  bool begin(const char* locator = ROBOT_ZENOH_LOCATOR);
  void service();
  bool command(ros_TwistStamped& result, bool sensors_ready);
  bool tuning(float (&values)[6], size_t& count);
  bool should_stop(bool sensors_ready);
  bool epoch(int64_t& stamp);
  bool alive();
  void publish(const ros_Odometry& message);
  void publish(const ros_Imu& message);
  void publish(const ros_Float32MultiArray& message);
  void publish_encoder_status(const char* status);
  void set_reset_diagnostic(int64_t reset_reason, int64_t software_reason);
  void log(uint8_t level, const char* message);
  static int64_t monotonic_ns();

 private:
  struct Command {
    ros_TwistStamped message{};
    int64_t received_ns = 0;
    bool pending = false, valid = false;
  };
#if ROBOT_TRANSPORT_DIAGNOSTICS
  struct CommandStats {
    uint32_t received = 0, invalid = 0, overwritten = 0, unselected = 0;
    uint32_t accepted = 0, local_stale = 0, replayed = 0;
    uint32_t rejected[static_cast<size_t>(PicoRosCommandGuard::RejectReason::count)]{};
    int64_t last_age_ns = 0, max_age_ns = 0;
  };
#endif
  static RobotTransport* instance_;
  static void receive_manual(uint8_t*, size_t);
  static void receive_navigation(uint8_t*, size_t);
  static void receive_mode(uint8_t*, size_t);
  static void receive_tuning(uint8_t*, size_t);
  static void receive_time(uint8_t*, size_t);
  void receive_command(unsigned source, uint8_t*, size_t);
  bool send(picoros_publisher_t&, uint8_t*, size_t);
  std::mutex mutex_;
  PicoRosTimeSync clock_;
  PicoRosCommandGuard guard_;
  picoros_node_t node_{};
  picoros_interface_t interface_{};
  picoros_publisher_t odom_pub_{}, imu_pub_{}, state_pub_{}, encoder_status_pub_{}, log_pub_{}, time_pub_{}, reset_pub_{}, clock_diag_pub_{};
  picoros_subscriber_t manual_sub_{}, nav_sub_{}, mode_sub_{}, tuning_sub_{}, time_sub_{};
  Command commands_[2];
#if ROBOT_TRANSPORT_DIAGNOSTICS
  CommandStats command_stats_[2];
#endif
  PicoRosCommandGuard::Mode mode_ = PicoRosCommandGuard::Mode::blocked;
  int64_t mode_received_ns_ = 0;
  bool mode_pending_ = false;
  int64_t last_command_stamp_[2] = {};
  float tuning_[6] = {};
  size_t tuning_count_ = 0;
  bool tuning_pending_ = false;
  int64_t boot_id_ = 0, request_id_ = 0, request_sent_ns_ = 0;
  int64_t last_sync_ns_ = 0;
#if ROBOT_TRANSPORT_DIAGNOSTICS
  int64_t last_diagnostic_report_ns_ = 0;
  int64_t last_service_start_ns_ = 0;
  uint32_t service_gap_max_us_ = 0, service_run_max_us_ = 0;
  // send() is called only by service(); each entry is the longest synchronous
  // publisher call since the last cmd_diag report: odom, imu, state, log, time.
  uint32_t publish_max_us_[5] = {};
#endif
  bool request_pending_ = false, started_ = false;
  int64_t clock_requests_ = 0, clock_matched_ = 0, clock_accepted_ = 0, clock_unmatched_ = 0;
  int64_t clock_last_rtt_ns_ = -1, clock_max_rtt_ns_ = 0, clock_last_result_ = -1;
  int64_t clock_rejected_[4] = {};
  int64_t clock_last_reject_ = -1;
  int64_t clock_diag_sent_ns_ = 0;
  int64_t reset_reason_ = 0, software_reset_reason_ = 0, reset_diagnostic_sent_ns_ = 0;
  bool reset_diagnostic_available_ = false;
  ros_Odometry odom_{};
  ros_Imu imu_{};
  float state_[7] = {};
  char encoder_status_[64] = "starting";
  char log_text_[256] = {};
  uint8_t log_level_ = 20;
  bool odom_pending_ = false, imu_pending_ = false, state_pending_ = false;
  bool encoder_status_pending_ = false, log_pending_ = false;
#if ROBOT_TRANSPORT_DIAGNOSTICS
  uint32_t odom_overwritten_ = 0, imu_overwritten_ = 0;
  uint32_t odom_publish_ok_ = 0, odom_publish_fail_ = 0, odom_stale_ = 0;
  uint32_t imu_publish_ok_ = 0, imu_publish_fail_ = 0, imu_stale_ = 0;
#endif
};
