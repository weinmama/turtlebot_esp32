#pragma once

#include "picoros.h"
#include "robot_cdr.h"
#include "picoros_time_sync.h"
#include "picoros_command_guard.h"
#include <mutex>

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
  void log(uint8_t level, const char* message);
  static int64_t monotonic_ns();

 private:
  struct Command {
    ros_TwistStamped message{};
    int64_t received_ns = 0;
    bool pending = false, valid = false;
  };
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
  picoros_publisher_t odom_pub_{}, imu_pub_{}, state_pub_{}, log_pub_{}, time_pub_{};
  picoros_subscriber_t manual_sub_{}, nav_sub_{}, mode_sub_{}, tuning_sub_{}, time_sub_{};
  Command commands_[2];
  PicoRosCommandGuard::Mode mode_ = PicoRosCommandGuard::Mode::blocked;
  int64_t mode_received_ns_ = 0;
  bool mode_pending_ = false;
  int64_t last_command_stamp_[2] = {};
  float tuning_[6] = {};
  size_t tuning_count_ = 0;
  bool tuning_pending_ = false;
  int64_t boot_id_ = 0, request_id_ = 0, request_sent_ns_ = 0;
  int64_t last_sync_ns_ = 0;
  bool request_pending_ = false, started_ = false;
  ros_Odometry odom_{};
  ros_Imu imu_{};
  float state_[7] = {};
  char log_text_[256] = {};
  uint8_t log_level_ = 20;
  bool odom_pending_ = false, imu_pending_ = false, state_pending_ = false, log_pending_ = false;
};
