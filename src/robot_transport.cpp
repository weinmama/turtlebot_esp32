#include "robot_transport.h"
#include <cmath>
#include <cstring>
#include <cstdio>
#ifdef ARDUINO
#include <esp_timer.h>
#else
#include <chrono>
#endif

RobotTransport* RobotTransport::instance_ = nullptr;

int64_t RobotTransport::monotonic_ns() {
#ifdef ARDUINO
  return esp_timer_get_time() * 1000;
#else
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
#endif
}

bool RobotTransport::begin(const char* locator) {
  instance_ = this;
  interface_.mode = const_cast<char*>("client");
  interface_.locator = const_cast<char*>(locator);
  if (picoros_interface_init(&interface_) != PICOROS_OK) return false;
  node_.name = "turtlebot_lowlevel_node";
  node_.domain_id = 1;
  if (picoros_node_init(&node_) != PICOROS_OK) return false;

#define PUB(member, topic_name, type) \
  member.topic = {topic_name, ROSTYPE_NAME(type), ROSTYPE_HASH(type)}; \
  z_publisher_options_default(&member.opts); \
  member.opts.congestion_control = Z_CONGESTION_CONTROL_DROP; \
  if (picoros_publisher_declare(&node_, &member) != PICOROS_OK) return false
#define SUB(member, topic_name, type, callback) \
  member.topic = {topic_name, ROSTYPE_NAME(type), ROSTYPE_HASH(type)}; \
  member.user_callback = callback; \
  if (picoros_subscriber_declare(&node_, &member) != PICOROS_OK) return false
  PUB(odom_pub_, ROBOT_ODOM_TOPIC, ros_Odometry);
  PUB(imu_pub_, "imu/data", ros_Imu);
  PUB(state_pub_, "motor/state", ros_Float32MultiArray);
  PUB(log_pub_, "rosout", ros_Log);
  PUB(time_pub_, "hardware/time/request", ros_Int64MultiArray);
  SUB(manual_sub_, "cmd_vel", ros_TwistStamped, receive_manual);
  SUB(nav_sub_, "hardware/nav_cmd_vel", ros_TwistStamped, receive_navigation);
  SUB(mode_sub_, "hardware/control_mode", ros_String, receive_mode);
  SUB(tuning_sub_, "pid_tune", ros_Float32MultiArray, receive_tuning);
  SUB(time_sub_, "hardware/time/reply", ros_Int64MultiArray, receive_time);
#undef PUB
#undef SUB
  boot_id_ = static_cast<int64_t>(z_random_u64() & INT64_MAX);
  started_ = true;
  return true;
}

void RobotTransport::receive_manual(uint8_t* bytes, size_t size) { instance_->receive_command(0, bytes, size); }
void RobotTransport::receive_navigation(uint8_t* bytes, size_t size) { instance_->receive_command(1, bytes, size); }

void RobotTransport::receive_command(unsigned source, uint8_t* bytes, size_t size) {
  Command incoming;
  incoming.received_ns = monotonic_ns();
  incoming.pending = true;
  incoming.valid = size <= 256 && robot_decode(bytes, size, incoming.message, ps_des_ros_TwistStamped);
  if (incoming.valid) {
    const auto& t = incoming.message.twist;
    incoming.valid = std::isfinite(t.linear.x) && std::isfinite(t.linear.y) && std::isfinite(t.linear.z) &&
        std::isfinite(t.angular.x) && std::isfinite(t.angular.y) && std::isfinite(t.angular.z) &&
        incoming.message.header.stamp.sec > 0 && incoming.message.header.stamp.nanosec < 1000000000;
  }
  // Deserialized strings alias the callback buffer; commands only need numbers.
  incoming.message.header.frame_id = nullptr;
  std::lock_guard<std::mutex> lock(mutex_);
  commands_[source] = incoming;
}

void RobotTransport::receive_mode(uint8_t* bytes, size_t size) {
  ros_String message{};
  if (size > 64 || !robot_decode(bytes, size, message, ps_des_ros_String)) return;
  using Mode = PicoRosCommandGuard::Mode;
  Mode mode = Mode::blocked;
  if (!strcmp(message.data, "idle")) mode = Mode::idle;
  else if (!strcmp(message.data, "mapping")) mode = Mode::mapping;
  else if (!strcmp(message.data, "navigation")) mode = Mode::navigation;
  auto& self = *instance_;
  std::lock_guard<std::mutex> lock(self.mutex_);
  // Invalidate immediately even if multiple mode changes occur before a poll.
  if (mode != self.mode_) {
    self.guard_.invalidate();
    self.commands_[0].pending = self.commands_[1].pending = false;
  }
  self.mode_ = mode;
  self.mode_received_ns_ = monotonic_ns();
  self.mode_pending_ = true;
}

void RobotTransport::receive_tuning(uint8_t* bytes, size_t size) {
  float values[6]{};
  ros_Float32MultiArray message{};
  message.data.data = values;
  message.data.n_elements = 6;
  if (size > 128 || !robot_decode(bytes, size, message, ps_des_ros_Float32MultiArray)) return;
  const size_t count = message.data.n_deserialized;
  if (count != 3 && count != 6) return;
  for (size_t i = 0; i < count; ++i) if (!std::isfinite(values[i]) || values[i] < 0) return;
  auto& self = *instance_;
  std::lock_guard<std::mutex> lock(self.mutex_);
  memcpy(self.tuning_, values, count * sizeof(float));
  self.tuning_count_ = count;
  self.tuning_pending_ = true;
}

void RobotTransport::receive_time(uint8_t* bytes, size_t size) {
  const int64_t received = monotonic_ns();
  int64_t values[4]{};
  ros_Int64MultiArray message{};
  message.data.data = values;
  message.data.n_elements = 4;
  if (size > 64 || !robot_decode(bytes, size, message, ps_des_ros_Int64MultiArray) ||
      message.data.n_deserialized != 4) return;
  auto& self = *instance_;
  std::lock_guard<std::mutex> lock(self.mutex_);
  if (!self.request_pending_ || values[0] != self.boot_id_ || values[1] != self.request_id_ ||
      values[2] != self.request_sent_ns_) return;
  self.request_pending_ = false; // A matching reply may only be consumed once.
  if (self.clock_.update(self.request_sent_ns_, received, values[3])) self.last_sync_ns_ = received;
}

bool RobotTransport::command(ros_TwistStamped& result, bool sensors_ready) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (mode_pending_) {
    guard_.heartbeat(mode_, mode_received_ns_ / 1000000);
    mode_pending_ = false;
  }
  int64_t epoch_ns = 0;
  const int64_t now = monotonic_ns();
  const bool clock_valid = clock_.epoch(now, epoch_ns) && now - last_sync_ns_ <= 5000000000LL;
  const unsigned source = mode_ == PicoRosCommandGuard::Mode::navigation ? 1 : 0;
  Command current = commands_[source];
  commands_[0].pending = commands_[1].pending = false;
  if (!current.pending) return false;
  if (!current.valid || now - current.received_ns > 200000000) {
    guard_.clear_command();
    return false;
  }
  const int64_t stamp = int64_t(current.message.header.stamp.sec) * 1000000000 +
                        current.message.header.stamp.nanosec;
  if (stamp <= last_command_stamp_[source]) return false;
  if (!guard_.accept(source ? PicoRosCommandGuard::Source::navigation : PicoRosCommandGuard::Source::manual,
                     current.message.twist.linear.x, current.message.twist.angular.z,
                     epoch_ns - stamp, now / 1000000, clock_valid, sensors_ready,
                     current.received_ns / 1000000)) return false;
  last_command_stamp_[source] = stamp;
  result = current.message;
  return true;
}

bool RobotTransport::tuning(float (&values)[6], size_t& count) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!tuning_pending_) return false;
  memcpy(values, tuning_, sizeof(tuning_));
  count = tuning_count_;
  tuning_pending_ = false;
  return true;
}

bool RobotTransport::should_stop(bool sensors_ready) {
  std::lock_guard<std::mutex> lock(mutex_);
  const int64_t now = monotonic_ns();
  int64_t stamp;
  return guard_.should_stop(now / 1000000,
      clock_.epoch(now, stamp) && now - last_sync_ns_ <= 5000000000LL, sensors_ready);
}

bool RobotTransport::epoch(int64_t& stamp) {
  std::lock_guard<std::mutex> lock(mutex_);
  return clock_.epoch(monotonic_ns(), stamp);
}

bool RobotTransport::alive() {
  std::lock_guard<std::mutex> lock(mutex_);
  return last_sync_ns_ && monotonic_ns() - last_sync_ns_ <= 5000000000LL;
}

void RobotTransport::publish(const ros_Odometry& message) {
  std::lock_guard<std::mutex> lock(mutex_);
  odom_ = message; odom_pending_ = true;
}
void RobotTransport::publish(const ros_Imu& message) {
  std::lock_guard<std::mutex> lock(mutex_);
  imu_ = message; imu_pending_ = true;
}
void RobotTransport::publish(const ros_Float32MultiArray& message) {
  if (message.data.n_elements != 7) return;
  std::lock_guard<std::mutex> lock(mutex_);
  memcpy(state_, message.data.data, sizeof(state_)); state_pending_ = true;
}
void RobotTransport::log(uint8_t level, const char* message) {
  std::lock_guard<std::mutex> lock(mutex_);
  snprintf(log_text_, sizeof(log_text_), "%s", message);
  log_level_ = level; log_pending_ = true;
}

bool RobotTransport::send(picoros_publisher_t& pub, uint8_t* bytes, size_t size) {
  return size && picoros_publish(&pub, bytes, size) == PICOROS_OK;
}

void RobotTransport::service() {
  if (!started_) return;
  uint8_t bytes[1024];
  int64_t time_values[3];
  bool request = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const int64_t now = monotonic_ns();
    if (now - request_sent_ns_ >= 1000000000LL) {
      request_sent_ns_ = now;
      request_pending_ = request = true;
      time_values[0] = boot_id_; time_values[1] = ++request_id_; time_values[2] = now;
    }
  }
  if (request) {
    ros_Int64MultiArray message{};
    message.data.data = time_values; message.data.n_elements = 3;
    send(time_pub_, bytes, robot_encode(bytes, sizeof(bytes), message, ps_ser_ros_Int64MultiArray));
  }
  // Copy under lock; never hold the control-task mutex during transport I/O.
  ros_Odometry odom;
  ros_Imu imu;
  float state[7];
  char text[256];
  uint8_t level;
  bool have_odom, have_imu, have_state, have_log;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    have_odom = odom_pending_; have_imu = imu_pending_; have_state = state_pending_; have_log = log_pending_;
    odom = odom_; imu = imu_; memcpy(state, state_, sizeof(state)); memcpy(text, log_text_, sizeof(text));
    level = log_level_;
    odom_pending_ = imu_pending_ = state_pending_ = log_pending_ = false;
  }
  int64_t stamp;
  if (epoch(stamp)) {
    if (have_odom && stamp - (int64_t(odom.header.stamp.sec) * 1000000000 + odom.header.stamp.nanosec) < 200000000)
      send(odom_pub_, bytes, robot_encode(bytes, sizeof(bytes), odom, ps_ser_ros_Odometry));
    if (have_imu && stamp - (int64_t(imu.header.stamp.sec) * 1000000000 + imu.header.stamp.nanosec) < 200000000)
      send(imu_pub_, bytes, robot_encode(bytes, sizeof(bytes), imu, ps_ser_ros_Imu));
    if (have_log) {
      ros_Log message{};
      message.stamp.sec = stamp / 1000000000; message.stamp.nanosec = stamp % 1000000000;
      message.level = level; message.name = const_cast<char*>("turtlebot_lowlevel_node");
      message.msg = text; message.file = message.function = const_cast<char*>("");
      send(log_pub_, bytes, robot_encode(bytes, sizeof(bytes), message, ps_ser_ros_Log));
    }
  }
  if (have_state) {
    ros_Float32MultiArray message{};
    message.data.data = state; message.data.n_elements = 7;
    send(state_pub_, bytes, robot_encode(bytes, sizeof(bytes), message, ps_ser_ros_Float32MultiArray));
  }
}
