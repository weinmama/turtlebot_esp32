#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Arduino.h>
#include <Wire.h>

#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rcutils/logging_macros.h>
#include <rmw_microros/rmw_microros.h>

#include <geometry_msgs/msg/transform_stamped.h>
#include <geometry_msgs/msg/twist_stamped.h>
#include <nav_msgs/msg/odometry.h>
#include <rcl_interfaces/msg/log.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <std_msgs/msg/string.h>

#include "esp_task_wdt.h"
#include <TB6612_ESP32.h>

// ---------------- Definitions ----------------
#define WDT_TIMEOUT 2              // Secsonds
#define AGENT_DEAD_TIMEOUT_MS 5000 // Milliseconds
#define AGENT_ALIVE_CHECK_MS 100   // Milliseconds
#define MOTOR_CONTROL_RATE_MS 100  // Milliseconds
#define EXECUTOR_SPIN_RATE_MS 5    // Milliseconds
#define IMU_UPDATE_RATE_MS 20      // Milliseconds
#define COM_RATE 115200
#define ROS_DOMAIN_ID 1

// High-rate /rosout logging from the motor control loop can stall the serial
// transport and break timing. Enable only while tuning.
#define MOTOR_DEBUG_LOGS 0
#define MOTOR_DEBUG_LOG_PERIOD_MS 250
#define MOTOR_TUNE_LOGS 0
#define MOTOR_TUNE_LOG_PERIOD_MS 500

// TB6012FNG H-Bridge Connection PINs
#define AIN1 26 // Motor A direction pin 1
#define AIN2 25 // Motor A direction pin 2
#define PWMA 27 // Motor A PWM speed control
#define BIN1 33 // Motor B direction pin 1
#define BIN2 32 // Motor B direction pin 2
#define PWMB 14 // Motor B PWM speed control
#define STBY 12 // Standby pin (must be HIGH to enable motors)

// Wheel Encoders Connection PINs
#define right_encoder_phaseA 19 // Interrupt
#define right_encoder_phaseB 18
#define left_encoder_phaseA 5
#define left_encoder_phaseB 17

#define TIMER_COUNT 3
#define SUBSCRIPTION_COUNT 2

/************** Pins **************/
// left wheel
#define ENC_A_L 5
#define ENC_B_L 17

// right wheel
#define ENC_A_R 19
#define ENC_B_R 18

// IMU I2C Pins
#define SDA_PIN 21
#define SCL_PIN 22

// -------------------- IMU --------------------
Adafruit_MPU6050 imu;

// ---------------- Globals ----------------
rclc_support_t support;
rcl_node_t node;
rcl_timer_t imu_timer;
rcl_timer_t motor_timer;
rcl_timer_t agent_alive_timer;
rclc_executor_t executor;

rcl_publisher_t rosout_pub;
rcl_publisher_t imu_pub;
rcl_publisher_t odom_pub;

rcl_subscription_t cmd_vel_sub;
rcl_subscription_t pid_tune_sub;

geometry_msgs__msg__TwistStamped cmd_vel_msg;
std_msgs__msg__Float32MultiArray pid_tune_msg;
std_msgs__msg__Float32MultiArray imu_msg;
nav_msgs__msg__Odometry odom_msg;

rcl_interfaces__msg__Log log_msg;
bool rosout_pub_initialized = false;

volatile long ticks_L = 0;
volatile long ticks_R = 0;

/************** Robot Parameter **************/
const float WHEEL_RADIUS = 0.034f;
const float WHEEL_BASE = 0.20f;
const int PULSES_PER_REV = 614;
const float WHEEL_CIRC = 2.0f * PI * WHEEL_RADIUS;

const float MIN_SPEED_X = 0.01f;
const float MIN_ANGULAR_SPEED_Z = 0.01f;

// If odometry or wheel direction is flipped, set one or both to -1.
const int ENC_DIR_L = 1;
const int ENC_DIR_R = 1;

// Thresholds to treat commands as zero (avoid float equality).
const float TARGET_RPM_EPS = 0.01f;

/************** L298 **************/
const int PWM_DEADBAND = 0;
const int PWM_MAX = 255;

// Motor capability (used for saturation + feedforward). Adjust to match your
// measured max RPM at PWM_MAX.
const float MAX_MOTOR_RPM = 100.0f;
const float MAX_WHEEL_SPEED_MPS = (MAX_MOTOR_RPM / 60.0f) * WHEEL_CIRC;
const float RPM_TO_PWM = (float)PWM_MAX / MAX_MOTOR_RPM;

// Static friction compensation (feedforward offset). If curves feel like
// "straight then suddenly turn", increase these a bit (typical 15-60).
// If low speeds feel too jumpy, decrease.
const int PWM_STATIC_L = 30;
const int PWM_STATIC_R = 30;

/************** Ramps **************/
const float A_MAX = 0.5f;

/************** Control **************/
static float v_cmd = 0.0f;
static float w_cmd = 0.0f;

static int64_t last_cmd_vel_ms = 0;
static const int64_t CMD_VEL_TIMEOUT_MS = 500;

static float v = 0.0f;
static float w = 0.0f;

float x = 0.0f, y = 0.0f, theta = 0.0f;
float v_odom = 0.0f, w_odom = 0.0f;

float Kp_L = 1.0f;
float Ki_L = 0.6f;
float Kd_L = 0.0f;
float Kp_R = 1.0f;
float Ki_R = 0.6f;
float Kd_R = 0.0f;

float int_L = 0.0f, prev_error_L = 0.0f;
float p_term_L = 0.0f, i_term_L = 0.0f, d_term_L = 0.0f;
float int_R = 0.0f, prev_error_R = 0.0f;
float p_term_R = 0.0f, i_term_R = 0.0f, d_term_R = 0.0f;

int64_t last_agent_ok_ms = 0;

Motor motor_left(BIN1, BIN2, PWMB, -1, STBY, 20000, 8, 2);
Motor motor_right(AIN1, AIN2, PWMA, -1, STBY, 20000, 8, 2);

// forward declarations - Motor helpers
int motorPID(float target_rpm, float actual_rpm, float &integral,
             float &prev_error, float dt, float Kp, float Ki, float Kd,
             float &p_term, float &i_term, float &d_term, int pwm_ff);

void log_message(uint8_t level, const char *name, const char *msg) {
  if (!rosout_pub_initialized) {
    return;
  }

  log_msg.level = level;

  log_msg.name.data = (char *)name;
  log_msg.name.size = strlen(name);
  log_msg.name.capacity = strlen(name) + 1;

  log_msg.msg.data = (char *)msg;
  log_msg.msg.size = strlen(msg);
  log_msg.msg.capacity = strlen(msg) + 1;

  log_msg.function.data = (char *)"";
  log_msg.function.size = 0;
  log_msg.function.capacity = 1;

  log_msg.file.data = (char *)"";
  log_msg.file.size = 0;
  log_msg.file.capacity = 1;

  log_msg.line = 0;

  rcl_ret_t ret = rcl_publish(&rosout_pub, &log_msg, NULL);
  (void)ret;
}

void log_info(const char *msg) {
  log_message(rcl_interfaces__msg__Log__INFO, "turtlebot_lowlevel_node", msg);
}

void log_warn(const char *msg) {
  log_message(rcl_interfaces__msg__Log__WARN, "turtlebot_lowlevel_node", msg);
}

void log_error(const char *msg) {
  log_message(rcl_interfaces__msg__Log__ERROR, "turtlebot_lowlevel_node", msg);
}

void log_debug(const char *msg) {
  log_message(rcl_interfaces__msg__Log__DEBUG, "turtlebot_lowlevel_node", msg);
}

// ===== Encoder ISRs =====

void isrLeft() {
  ticks_L += ENC_DIR_L * ((digitalRead(ENC_B_L) == HIGH) ? 1 : -1);
}
void isrRight() {
  ticks_R += ENC_DIR_R * ((digitalRead(ENC_B_R) == HIGH) ? 1 : -1);
}

// ===== Timer callback (50 Hz) =====
void imu_timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
  (void)last_call_time;
  if (timer == NULL)
    return;

  sensors_event_t a, g, temp;
  imu.getEvent(&a, &g, &temp);

  imu_msg.data.data[0] = a.acceleration.x;
  imu_msg.data.data[1] = a.acceleration.y;
  imu_msg.data.data[2] = a.acceleration.z;
  imu_msg.data.data[3] = g.gyro.x;
  imu_msg.data.data[4] = g.gyro.y;
  imu_msg.data.data[5] = g.gyro.z;

  rcl_ret_t ret = rcl_publish(&imu_pub, &imu_msg, NULL);
  (void)ret; // Suppress unused warning
}

static int64_t now_ms(void) { return (esp_timer_get_time() / 1000); }

static float normalize_angle(float a) {
  while (a > PI)
    a -= 2.0f * PI;
  while (a < -PI)
    a += 2.0f * PI;
  return a;
}

void agent_alive_timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
  (void)last_call_time;
  if (timer == NULL)
    return;

  if (rmw_uros_ping_agent(100, 1) == RMW_RET_OK) {
    last_agent_ok_ms = now_ms();
  }
}

// PID tuning callback - receives array [Kp_L, Ki_L, Kd_L, Kp_R, Ki_R, Kd_R]
void pid_tune_callback(const void *msgin) {
  const std_msgs__msg__Float32MultiArray *msg =
      (const std_msgs__msg__Float32MultiArray *)msgin;

  if (msg->data.size == 3) {
    Kp_L = msg->data.data[0];
    Ki_L = msg->data.data[1];
    Kd_L = msg->data.data[2];

    Kp_R = Kp_L;
    Ki_R = Ki_L;
    Kd_R = Kd_L;
    char log_buf[100];
    snprintf(log_buf, sizeof(log_buf), "Left PID updated: Kp=%.2f Ki=%.2f Kd=%.2f",
             Kp_L, Ki_L, Kd_L);
    log_info(log_buf);
    return;
  }

  if (msg->data.size >= 6) {
    Kp_L = msg->data.data[0];
    Ki_L = msg->data.data[1];
    Kd_L = msg->data.data[2];
    Kp_R = msg->data.data[3];
    Ki_R = msg->data.data[4];
    Kd_R = msg->data.data[5];

    char log_buf[100];
    snprintf(log_buf, sizeof(log_buf), "PID updated: Kp=%.2f Ki=%.2f Kd=%.2f",
             Kp_L, Ki_L, Kd_L);
    log_info(log_buf);
  }
}

void motor_timer_callback(rcl_timer_t *timer, int64_t last_call_time) {

  (void)last_call_time;

  if (timer == NULL)
    return;

  // Compute dt from a monotonic clock. The rclc `last_call_time` argument is
  // not guaranteed to be a period; treating it as dt can break odometry.
  static int64_t last_motor_us = 0;
  const int64_t now_us = esp_timer_get_time();
  if (last_motor_us == 0) {
    last_motor_us = now_us;
    return;
  }

  const float dt = (now_us - last_motor_us) / 1e6f;
  last_motor_us = now_us;

  if (dt <= 0.0f || dt > 0.5f) {
    return;
  }

  v = v_cmd;
  w = w_cmd;

  // If cmd_vel stops arriving (e.g., transport congestion), don't keep driving
  // forever on the last command.
  if (last_cmd_vel_ms != 0 && (now_ms() - last_cmd_vel_ms) > CMD_VEL_TIMEOUT_MS) {
    v = 0.0f;
    w = 0.0f;
    v_cmd = 0.0f;
    w_cmd = 0.0f;
  }

  // -------- Diff Drive --------
  float vL = v - w * WHEEL_BASE / 2.0f;
  float vR = v + w * WHEEL_BASE / 2.0f;

  // Saturate wheel speeds to what the motors can actually deliver. This keeps
  // cmd_vel in m/s, but prevents requesting impossible RPM.
  const float max_abs_wheel = fmaxf(fabsf(vL), fabsf(vR));
  if (max_abs_wheel > MAX_WHEEL_SPEED_MPS && max_abs_wheel > 1e-6f) {
    const float s = MAX_WHEEL_SPEED_MPS / max_abs_wheel;
    vL *= s;
    vR *= s;
  }

  float rpmL_target = vL / WHEEL_CIRC * 60.0f;
  float rpmR_target = vR / WHEEL_CIRC * 60.0f;

  // -------- Encoder --------
  noInterrupts();
  long dL = ticks_L;
  ticks_L = 0;
  long dR = ticks_R;
  ticks_R = 0;
  interrupts();

  float dsL = (float)dL / PULSES_PER_REV * WHEEL_CIRC;
  float dsR = (float)dR / PULSES_PER_REV * WHEEL_CIRC;

  // -------- Odometry --------
  float ds = (dsR + dsL) * 0.5f;
  float dtheta = (dsR - dsL) / WHEEL_BASE;

  x += ds * cos(theta + dtheta * 0.5f);
  y += ds * sin(theta + dtheta * 0.5f);
  theta += dtheta;
  theta = normalize_angle(theta);

  // Calculate velocities for odometry message
  v_odom = ds / dt;
  w_odom = dtheta / dt;

  // -------- Publish Odometry --------
  // Get current time in nanoseconds
  int64_t time_ns = rmw_uros_epoch_nanos();

  // Populate odometry message
  odom_msg.header.stamp.sec = time_ns / 1000000000;
  odom_msg.header.stamp.nanosec = time_ns % 1000000000;

  // Position
  odom_msg.pose.pose.position.x = x;
  odom_msg.pose.pose.position.y = y;
  odom_msg.pose.pose.position.z = 0.0;

  // Orientation (quaternion from theta)
  float half_theta = theta * 0.5f;
  odom_msg.pose.pose.orientation.x = 0.0;
  odom_msg.pose.pose.orientation.y = 0.0;
  odom_msg.pose.pose.orientation.z = sin(half_theta);
  odom_msg.pose.pose.orientation.w = cos(half_theta);

  // Velocity in robot frame
  odom_msg.twist.twist.linear.x = v_odom;
  odom_msg.twist.twist.linear.y = 0.0;
  odom_msg.twist.twist.linear.z = 0.0;
  odom_msg.twist.twist.angular.x = 0.0;
  odom_msg.twist.twist.angular.y = 0.0;
  odom_msg.twist.twist.angular.z = w_odom;

  // Publish odometry
  rcl_ret_t odom_ret = rcl_publish(&odom_pub, &odom_msg, NULL);
  (void)odom_ret; // Suppress unused warning

  // -------- RPM --------
  float rpmL = (dsL / WHEEL_CIRC) / dt * 60.0f;
  float rpmR = (dsR / WHEEL_CIRC) / dt * 60.0f;

  // Calculate actual velocities from encoder measurements
  float vL_actual = dsL / dt;
  float vR_actual = dsR / dt;

  unsigned long now = millis();

  // -------- Control --------
  int pwm_L_out = 0, pwm_R_out = 0;

  // Feedforward PWM from desired RPM, including a static offset to overcome
  // stiction so small steering differentials result in smooth curves.
  auto pwm_ff_from_rpm = [](float rpm_target, int pwm_static) -> int {
    if (fabs(rpm_target) < TARGET_RPM_EPS)
      return 0;

    float pwm = rpm_target * RPM_TO_PWM;
    if (pwm > 0) {
      pwm += pwm_static;
    } else {
      pwm -= pwm_static;
    }
    return (int)constrain((int)pwm, -PWM_MAX, PWM_MAX);
  };

  const int pwmL_ff = pwm_ff_from_rpm(rpmL_target, PWM_STATIC_L);
  const int pwmR_ff = pwm_ff_from_rpm(rpmR_target, PWM_STATIC_R);

  if (fabs(rpmL_target) < TARGET_RPM_EPS) {
    motor_left.brake();
    int_L = 0;
    prev_error_L = 0;
    p_term_L = i_term_L = d_term_L = 0;
  } else {
    pwm_L_out = motorPID(rpmL_target, rpmL, int_L, prev_error_L, dt, Kp_L, Ki_L,
                         Kd_L, p_term_L, i_term_L, d_term_L, pwmL_ff);
    motor_left.drive(pwm_L_out);
  }

  if (fabs(rpmR_target) < TARGET_RPM_EPS) {
    motor_right.brake();
    int_R = 0;
    prev_error_R = 0;
    p_term_R = i_term_R = d_term_R = 0;
  } else {
    pwm_R_out = motorPID(rpmR_target, rpmR, int_R, prev_error_R, dt, Kp_R, Ki_R,
                         Kd_R, p_term_R, i_term_R, d_term_R, pwmR_ff);
    motor_right.drive(pwm_R_out);
  }

  // -------- Debug Logging --------
#if MOTOR_DEBUG_LOGS
  static unsigned long last_log_ms = 0;
  if (now - last_log_ms >= MOTOR_DEBUG_LOG_PERIOD_MS) {
    char log_buffer[220];
    snprintf(log_buffer, sizeof(log_buffer),
             "cmd:%.3f v:%.3f | vL:%.3f(%.3f) vR:%.3f(%.3f) | "
             "rpmL:%.1f(%.1f) rpmR:%.1f(%.1f) | "
             "pwmL:%d(ff:%d) pwmR:%d(ff:%d)",
             v_cmd, v, vL, vL_actual, vR, vR_actual, rpmL_target, rpmL,
             rpmR_target, rpmR, pwm_L_out, pwmL_ff, pwm_R_out, pwmR_ff);
    log_debug(log_buffer);
    last_log_ms = now;
  }
#endif

  // Detailed tuning logs (separate cadence from the main kinematics log)
#if MOTOR_TUNE_LOGS
  static unsigned long last_tune_log_ms = 0;
  if (now - last_tune_log_ms >= MOTOR_TUNE_LOG_PERIOD_MS) {
    char tune_buffer[128];
    snprintf(tune_buffer, sizeof(tune_buffer),
             "L: P=%.1f I=%.1f D=%.1f OUT=%d FF=%d | R: P=%.1f I=%.1f D=%.1f OUT=%d FF=%d",
             p_term_L, i_term_L, d_term_L, pwm_L_out, pwmL_ff, p_term_R,
             i_term_R, d_term_R, pwm_R_out, pwmR_ff);
    log_debug(tune_buffer);
    last_tune_log_ms = now;
  }
#endif
}

void cmd_vel_callback(const void *msgin) {
  const geometry_msgs__msg__TwistStamped *msg =
      (const geometry_msgs__msg__TwistStamped *)msgin;
  v_cmd = fabs(msg->twist.linear.x) < MIN_SPEED_X ? 0.0f : msg->twist.linear.x;
  w_cmd = fabs(msg->twist.angular.z) < MIN_ANGULAR_SPEED_Z
              ? 0.0f
              : msg->twist.angular.z;

  last_cmd_vel_ms = now_ms();
}

// ---------------- Arduino setup ----------------
void setup() {

  // activate standby
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);

  pinMode(ENC_A_L, INPUT_PULLUP);
  pinMode(ENC_B_L, INPUT_PULLUP);
  pinMode(ENC_A_R, INPUT_PULLUP);
  pinMode(ENC_B_R, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENC_A_L), isrLeft, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_A_R), isrRight, RISING);
  Serial.begin(COM_RATE);

  // Initialize micro-ROS serial transport
  set_microros_serial_transports(Serial);

  // wait for agent
  while (rmw_uros_ping_agent(100, 1) != RMW_RET_OK) {
    delay(1000);
  }

  // start watchdog
  esp_task_wdt_init(WDT_TIMEOUT, true);
  esp_task_wdt_add(NULL);

  // try to initialize IMU
  Wire.begin(SDA_PIN, SCL_PIN);
  if (!imu.begin()) {
    RCUTILS_LOG_ERROR("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }

  // Reset Watchdog to be save - don't know if it helps
  esp_task_wdt_reset();

  // Allocator & support
  rcl_allocator_t allocator = rcl_get_default_allocator();

  rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
  rcl_ret_t ret = rcl_init_options_init(&init_options, allocator);
  (void)ret;
  ret = rcl_init_options_set_domain_id(&init_options, ROS_DOMAIN_ID);
  (void)ret;
  rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator);

  // Create node
  rclc_node_init_default(&node, "turtlebot_lowlevel_node", "", &support);

  // Initialize rosout publisher with system default QoS (transient local,
  // reliable)
  rmw_qos_profile_t rosout_qos = rmw_qos_profile_system_default;
  rosout_qos.depth = 10;
  rosout_qos.durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
  rosout_qos.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;

  rcl_publisher_options_t rosout_pub_options =
      rcl_publisher_get_default_options();
  rosout_pub_options.qos = rosout_qos;

  ret = rcl_publisher_init(
      &rosout_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(rcl_interfaces, msg, Log),
      "/rosout", &rosout_pub_options);
  (void)ret;

  // Initialize log message
  rcl_interfaces__msg__Log__init(&log_msg);
  rosout_pub_initialized = true;

  log_info("Node initialized");

  // IMU publisher
    rmw_qos_profile_t sensor_qos = rmw_qos_profile_sensor_data; // best-effort
    sensor_qos.depth = 5;
    rcl_publisher_options_t sensor_pub_options =
      rcl_publisher_get_default_options();
    sensor_pub_options.qos = sensor_qos;

    ret = rcl_publisher_init(
      &imu_pub, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
      "imu/data", &sensor_pub_options);
    (void)ret;

  imu_msg.data.capacity = 6;
  imu_msg.data.size = 6;
  imu_msg.data.data = (float *)malloc(6 * sizeof(float));

  log_info("IMU publisher initialized");

  // Odometry publisher
    ret = rcl_publisher_init(
      &odom_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
      "odom", &sensor_pub_options);
    (void)ret;

  // Initialize odometry message
  nav_msgs__msg__Odometry__init(&odom_msg);

  // Set frame IDs (manually allocate and copy strings)
  odom_msg.header.frame_id.data = (char *)malloc(5);
  strcpy(odom_msg.header.frame_id.data, "odom");
  odom_msg.header.frame_id.size = 4;
  odom_msg.header.frame_id.capacity = 5;

  odom_msg.child_frame_id.data = (char *)malloc(19);
  strcpy(odom_msg.child_frame_id.data, "base_footprint");
  odom_msg.child_frame_id.size = 14;
  odom_msg.child_frame_id.capacity = 19;

  log_info("Odometry publisher initialized");

  rclc_timer_init_default2(&imu_timer, &support,
                           RCL_MS_TO_NS(IMU_UPDATE_RATE_MS), imu_timer_callback,
                           true);

  rclc_timer_init_default2(&agent_alive_timer, &support,
                           RCL_MS_TO_NS(AGENT_ALIVE_CHECK_MS),
                           agent_alive_timer_callback, true);
  rclc_timer_init_default2(&motor_timer, &support,
                           RCL_MS_TO_NS(MOTOR_CONTROL_RATE_MS),
                           motor_timer_callback, true);

  // Reset Watchdog to be save - don't know if it helps
  esp_task_wdt_reset();

  // Initialize PID tuning message
  pid_tune_msg.data.capacity = 6;
  pid_tune_msg.data.size = 0;
  pid_tune_msg.data.data = (float *)malloc(6 * sizeof(float));

  rclc_executor_init(&executor, &support.context, TIMER_COUNT + SUBSCRIPTION_COUNT, &allocator);
  rclc_executor_add_timer(&executor, &agent_alive_timer);
  rclc_executor_add_timer(&executor, &imu_timer);
  rclc_executor_add_timer(&executor, &motor_timer);

  // cmd_vel subscriber
    rmw_qos_profile_t cmd_qos = rmw_qos_profile_sensor_data; // best-effort
    cmd_qos.depth = 1;
    rcl_subscription_options_t cmd_sub_options =
      rcl_subscription_get_default_options();
    cmd_sub_options.qos = cmd_qos;
    ret = rcl_subscription_init(
      &cmd_vel_sub, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, TwistStamped), "cmd_vel",
      &cmd_sub_options);
    (void)ret;
  rclc_executor_add_subscription(&executor, &cmd_vel_sub, &cmd_vel_msg,
                                 &cmd_vel_callback, ON_NEW_DATA);

  // PID tuning subscriber
  rclc_subscription_init_default(
      &pid_tune_sub, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
      "pid_tune");
  rclc_executor_add_subscription(&executor, &pid_tune_sub, &pid_tune_msg,
                                 &pid_tune_callback, ON_NEW_DATA);

  log_info("System initialization complete");
  log_info("Ready to receive commands");
}

// ---------------- Arduino loop ----------------
void loop() {
  if (rclc_executor_spin_some(&executor, RCL_MS_TO_NS(EXECUTOR_SPIN_RATE_MS)) !=

      RCL_RET_OK) {
    delay(1);
  }

  if (now_ms() - last_agent_ok_ms > AGENT_DEAD_TIMEOUT_MS) {
    log_error("Agent connection lost - restarting");
    delay(100); // Give time for log to be sent
    esp_restart();
  } else {
    esp_task_wdt_reset();
  }
}

// // ===================== Motor helpers =====================
int motorPID(float target_rpm, float actual_rpm, float &integral,
             float &prev_error, float dt, float Kp, float Ki, float Kd,
             float &p_term, float &i_term, float &d_term, int pwm_ff) {
  if (fabs(target_rpm) < TARGET_RPM_EPS) {
    integral = 0;
    prev_error = 0;
    p_term = i_term = d_term = 0;
    return 0;
  }

  float error = target_rpm - actual_rpm;

  // Calculate derivative (rate of error change)
  float derivative = 0;
  if (dt > 0.001f) {
    derivative = (error - prev_error) / dt;
    // Simple low-pass filter on derivative to reduce noise
    static float filtered_deriv_L = 0;
    static float filtered_deriv_R = 0;
    float alpha = 0.8f; // filter coefficient
    if (&prev_error == &prev_error_L) {
      filtered_deriv_L = alpha * derivative + (1 - alpha) * filtered_deriv_L;
      derivative = filtered_deriv_L;
    } else {
      filtered_deriv_R = alpha * derivative + (1 - alpha) * filtered_deriv_R;
      derivative = filtered_deriv_R;
    }
  }

  // Predict integral first, then apply a simple anti-windup decision based on
  // the final (feedforward + PID) PWM.
  float integral_candidate = integral + error * dt;
  integral_candidate = constrain(integral_candidate, -100, 100);

  const float p_candidate = Kp * error;
  const float d_candidate = Kd * derivative;
  const float i_candidate = Ki * integral_candidate;
  const float u_candidate = (float)pwm_ff + p_candidate + i_candidate + d_candidate;
  const int pwm_candidate = (int)u_candidate;

  const bool saturated = (pwm_candidate >= PWM_MAX) || (pwm_candidate <= -PWM_MAX);
  if (!saturated || (error * integral < 0)) {
    integral = integral_candidate;
  }

  p_term = Kp * error;
  i_term = Ki * integral;
  d_term = Kd * derivative;

  float u = (float)pwm_ff + p_term + i_term + d_term;
  int pwm = (int)u;

  // Store error for next iteration
  prev_error = error;

  if (pwm > 0) {
    return constrain(pwm, PWM_DEADBAND, PWM_MAX);
  } else {
    return constrain(pwm, -PWM_MAX, -PWM_DEADBAND);
  }
}
