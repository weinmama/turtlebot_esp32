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

#include <geometry_msgs/msg/twist_stamped.h>
#include <nav_msgs/msg/odometry.h>
#include <sensor_msgs/msg/imu.h>
#include <rosidl_runtime_c/string_functions.h>
#include <rcl_interfaces/msg/log.h>
#include <std_msgs/msg/float32_multi_array.h>

#include <esp_timer.h>
#include "esp_task_wdt.h"
#include <PID_v1.h>
#include <TB6612_ESP32.h>
#include "gyro_calibration.h"

// ---------------- Definitions ----------------
#define WDT_TIMEOUT 2              // Secsonds
#define AGENT_DEAD_TIMEOUT_MS 5000 // Milliseconds
#define AGENT_ALIVE_CHECK_MS 500   // Milliseconds
#define MOTOR_CONTROL_RATE_MS 100  // Milliseconds
#define EXECUTOR_SPIN_RATE_MS 5    // Milliseconds
#define IMU_UPDATE_RATE_MS 50      // Milliseconds (20 Hz)
#define COM_RATE 460800
#define ROS_DOMAIN_ID 1

// High-rate /rosout logging from the motor control loop can stall the serial
// transport and break timing. Enable only while tuning.
#define MOTOR_DEBUG_LOGS 0
#define MOTOR_DEBUG_LOG_PERIOD_MS 250

// TB6012FNG H-Bridge Connection PINs
#define AIN1 26 // Motor A direction pin 1
#define AIN2 25 // Motor A direction pin 2
#define PWMA 27 // Motor A PWM speed control
#define BIN1 33 // Motor B direction pin 1
#define BIN2 32 // Motor B direction pin 2
#define PWMB 14 // Motor B PWM speed control
#define STBY 12 // Standby pin (must be HIGH to enable motors)

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
rcl_publisher_t motor_state_pub;

rcl_subscription_t cmd_vel_sub;
rcl_subscription_t pid_tune_sub;

geometry_msgs__msg__TwistStamped cmd_vel_msg;
std_msgs__msg__Float32MultiArray pid_tune_msg;
sensor_msgs__msg__Imu imu_msg;
nav_msgs__msg__Odometry odom_msg;
std_msgs__msg__Float32MultiArray motor_state_msg;
float motor_state_data[7];

rcl_interfaces__msg__Log log_msg;
bool rosout_pub_initialized = false;
int64_t last_time_sync_ms = 0;

volatile long ticks_L = 0;
volatile long ticks_R = 0;
volatile unsigned long encoder_events = 0;
GyroCalibration gyro_calibration;

/************** Robot Parameter **************/
const float WHEEL_RADIUS = 0.034f;
const float WHEEL_BASE = 0.20f;

#define PULSES_PER_MOTOR_REV 11.0f
#define GEAR_RATIO 46.8f
const float PULSES_PER_REV = PULSES_PER_MOTOR_REV * GEAR_RATIO;
const float WHEEL_CIRC = 2.0f * PI * WHEEL_RADIUS;

// Maximum angular velocity safety limit
const float MAX_ANGULAR_SPEED_Z = 4.0f;

// If odometry or wheel direction is flipped, set one or both to -1.
const int ENC_DIR_L = 1;
const int ENC_DIR_R = 1;

// Thresholds to treat commands as zero (avoid float equality).
// Small value to detect true zero while allowing small differential speeds
const float TARGET_RPM_EPS = 0.5f;

const int PWM_MAX = 255;

// Motor capability (used for speed saturation). Adjust to match your measured
// max RPM at PWM_MAX.
const float MAX_MOTOR_RPM = 100.0f;
const float MAX_WHEEL_SPEED_MPS = (MAX_MOTOR_RPM / 60.0f) * WHEEL_CIRC;

// Feedforward mapping fitted to unloaded wheel measurements. Recheck under load.
const float RPM_TO_PWM = 2.0f;
// Feedforward intercept, not a guaranteed minimum PWM under load.
const int PWM_STATIC_L = 16;
const int PWM_STATIC_R = 16;

// Slew-rate limit for wheel RPM targets to avoid a jump in velocity when
// starting to move (especially with feedforward + static friction offset).
// Units: wheel RPM per second.
const float TARGET_RPM_RAMP_PER_S = 200.0f;

static int64_t last_cmd_vel_ms = 0;
static const int64_t CMD_VEL_TIMEOUT_MS = 500;

float x = 0.0f, y = 0.0f, theta = 0.0f;
float v_odom = 0.0f, w_odom = 0.0f;

double left_measured_rpm = 0.0;
double right_measured_rpm = 0.0;
double left_cmd = 0.0;
double right_cmd = 0.0;
double left_target_rpm_req = 0.0;
double right_target_rpm_req = 0.0;
double left_target_rpm = 0.0;
double right_target_rpm = 0.0;
double left_ramped_rpm = 0.0;
double right_ramped_rpm = 0.0;

float left_direction_coeff = 1.0f;
float right_direction_coeff = 1.0f;

#define Kp_L 1.0f
#define Ki_L 3.0f
#define Kd_L 0.0f

#define Kp_R 1.0f
#define Ki_R 3.0f
#define Kd_R 0.0f

PID motor_left_pid(&left_measured_rpm, &left_cmd, &left_target_rpm, Kp_L, Ki_L,
                   Kd_L, DIRECT);
PID motor_right_pid(&right_measured_rpm, &right_cmd, &right_target_rpm, Kp_R,
                    Ki_R, Kd_R, DIRECT);

Motor motor_left(BIN1, BIN2, PWMB, -1, STBY, 20000, 8, 2);
Motor motor_right(AIN1, AIN2, PWMA, -1, STBY, 20000, 8, 1);

int64_t last_agent_ok_ms = 0;

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
  ++encoder_events;
}
void isrRight() {
  ticks_R += ENC_DIR_R * ((digitalRead(ENC_B_R) == HIGH) ? 1 : -1);
  ++encoder_events;
}

// ===== Timer callback (50 Hz) =====
void imu_timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
  (void)last_call_time;
  if (timer == NULL)
    return;

  sensors_event_t a, g, temp;
  imu.getEvent(&a, &g, &temp);

  static unsigned long previous_encoder_events = 0;
  noInterrupts();
  const unsigned long current_encoder_events = encoder_events;
  interrupts();
  const float acceleration = sqrtf(a.acceleration.x * a.acceleration.x +
      a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z);
  const bool stationary = current_encoder_events == previous_encoder_events &&
      fabs(left_target_rpm_req) < TARGET_RPM_EPS && fabs(right_target_rpm_req) < TARGET_RPM_EPS &&
      std::isfinite(acceleration) && fabsf(acceleration - 9.81f) < 1.0f;
  previous_encoder_events = current_encoder_events;
  gyro_calibration.sample(g.gyro.x, g.gyro.y, g.gyro.z, stationary);
  // Do not let the EKF or hardware readiness consume uncalibrated gyro data.
  if (!gyro_calibration.ready()) return;

  if (!rmw_uros_epoch_synchronized() ||
      esp_timer_get_time() / 1000 - last_time_sync_ms > 120000) return;
  const int64_t stamp = rmw_uros_epoch_nanos();
  imu_msg.header.stamp.sec = stamp / 1000000000;
  imu_msg.header.stamp.nanosec = stamp % 1000000000;
  imu_msg.linear_acceleration.x = a.acceleration.x;
  imu_msg.linear_acceleration.y = a.acceleration.y;
  imu_msg.linear_acceleration.z = a.acceleration.z;
  imu_msg.angular_velocity.x = g.gyro.x - gyro_calibration.bias(0);
  imu_msg.angular_velocity.y = g.gyro.y - gyro_calibration.bias(1);
  imu_msg.angular_velocity.z = g.gyro.z - gyro_calibration.bias(2);

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
    const double kp = msg->data.data[0];
    const double ki = msg->data.data[1];
    const double kd = msg->data.data[2];
    motor_left_pid.SetTunings(kp, ki, kd);
    motor_right_pid.SetTunings(kp, ki, kd);

    char log_buf[128];
    snprintf(log_buf, sizeof(log_buf),
             "PID updated (both): Kp=%.3f Ki=%.3f Kd=%.3f", kp, ki, kd);
    log_info(log_buf);
    return;
  }

  if (msg->data.size >= 6) {
    const double kpL = msg->data.data[0];
    const double kiL = msg->data.data[1];
    const double kdL = msg->data.data[2];
    const double kpR = msg->data.data[3];
    const double kiR = msg->data.data[4];
    const double kdR = msg->data.data[5];

    motor_left_pid.SetTunings(kpL, kiL, kdL);
    motor_right_pid.SetTunings(kpR, kiR, kdR);

    char log_buf[192];
    snprintf(log_buf, sizeof(log_buf),
             "PID updated: L(Kp=%.3f Ki=%.3f Kd=%.3f) R(Kp=%.3f Ki=%.3f Kd=%.3f)",
             kpL, kiL, kdL, kpR, kiR, kdR);
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

  if (dt <= 0.0f) {
    return;
  }

  // A delayed executor must not leave the previous motor output running.
  if (dt > 0.5f) {
    motor_left.brake();
    motor_right.brake();
    motor_left_pid.SetMode(MANUAL);
    motor_right_pid.SetMode(MANUAL);
    left_cmd = right_cmd = 0.0;
    left_ramped_rpm = right_ramped_rpm = 0.0;
  }

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
  rcl_ret_t odom_ret = RCL_RET_OK;
  if (rmw_uros_epoch_synchronized() && now_ms() - last_time_sync_ms <= 120000)
    odom_ret = rcl_publish(&odom_pub, &odom_msg, NULL);
  (void)odom_ret; // Suppress unused warning

  // -------- RPM --------
  left_measured_rpm = fabs((dsL / WHEEL_CIRC) / dt * 60.0f);
  right_measured_rpm = fabs((dsR / WHEEL_CIRC) / dt * 60.0f);

  // Decide whether to stop based on commanded targets (not measured odometry).
  // Using odometry here can keep the robot braked forever at standstill.
  const bool cmd_timed_out =
      (last_cmd_vel_ms != 0 &&
       (now_ms() - last_cmd_vel_ms) > CMD_VEL_TIMEOUT_MS);
  const bool targets_zero = ((fabs(left_target_rpm_req) < TARGET_RPM_EPS) &&
                             (fabs(right_target_rpm_req) < TARGET_RPM_EPS));
  const bool robot_stopped = cmd_timed_out || targets_zero || dt > 0.5f;

  // Slew-limit the actual PID setpoints towards the requested targets.
  const float max_delta_rpm = TARGET_RPM_RAMP_PER_S * dt;
  const double previous_left_rpm = left_ramped_rpm;
  const double previous_right_rpm = right_ramped_rpm;
  if (robot_stopped) {
    left_ramped_rpm = right_ramped_rpm = 0.0;
  } else {
    double dl = left_direction_coeff * left_target_rpm_req - left_ramped_rpm;
    if (dl > (double)max_delta_rpm) {
      dl = (double)max_delta_rpm;
    } else if (dl < -(double)max_delta_rpm) {
      dl = -(double)max_delta_rpm;
    }
    left_ramped_rpm += dl;

    double dr = right_direction_coeff * right_target_rpm_req - right_ramped_rpm;
    if (dr > (double)max_delta_rpm) {
      dr = (double)max_delta_rpm;
    } else if (dr < -(double)max_delta_rpm) {
      dr = -(double)max_delta_rpm;
    }
    right_ramped_rpm += dr;
  }
  left_target_rpm = fabs(left_ramped_rpm);
  right_target_rpm = fabs(right_ramped_rpm);
  if (previous_left_rpm * left_ramped_rpm < 0.0) {
    motor_left_pid.SetMode(MANUAL);
    left_cmd = 0.0;
  }
  if (previous_right_rpm * right_ramped_rpm < 0.0) {
    motor_right_pid.SetMode(MANUAL);
    right_cmd = 0.0;
  }

  int pwm_l = 0;
  int pwm_r = 0;

  if (robot_stopped) {
    // Stop everything
    motor_left.brake();
    motor_right.brake();
    left_cmd = 0.0;
    right_cmd = 0.0;
    if (motor_left_pid.GetMode() != MANUAL) {
      motor_left_pid.SetMode(MANUAL);
    }
    if (motor_right_pid.GetMode() != MANUAL) {
      motor_right_pid.SetMode(MANUAL);
    }
  } else {

    if (motor_left_pid.GetMode() != AUTOMATIC) {
      motor_left_pid.SetMode(AUTOMATIC);
    }
    if (motor_right_pid.GetMode() != AUTOMATIC) {
      motor_right_pid.SetMode(AUTOMATIC);
    }

    const int sign_l = (left_ramped_rpm >= 0.0) ? 1 : -1;
    const int sign_r = (right_ramped_rpm >= 0.0) ? 1 : -1;

    int pwm_ff_l = (int)lround(fabs(left_target_rpm) * RPM_TO_PWM);
    int pwm_ff_r = (int)lround(fabs(right_target_rpm) * RPM_TO_PWM);
    pwm_ff_l = constrain(pwm_ff_l, 0, PWM_MAX);
    pwm_ff_r = constrain(pwm_ff_r, 0, PWM_MAX);
    if (pwm_ff_l > 0) {
      pwm_ff_l = min(PWM_MAX, pwm_ff_l + PWM_STATIC_L);
    }
    if (pwm_ff_r > 0) {
      pwm_ff_r = min(PWM_MAX, pwm_ff_r + PWM_STATIC_R);
    }

    // Limit the integral and correction to the actuator headroom.
    motor_left_pid.SetOutputLimits(-pwm_ff_l, PWM_MAX - pwm_ff_l);
    motor_right_pid.SetOutputLimits(-pwm_ff_r, PWM_MAX - pwm_ff_r);
    motor_left_pid.Compute();
    motor_right_pid.Compute();

    const int pwm_mag_l = constrain(pwm_ff_l + (int)lround(left_cmd), 0, PWM_MAX);
    const int pwm_mag_r = constrain(pwm_ff_r + (int)lround(right_cmd), 0, PWM_MAX);

    pwm_l = sign_l * pwm_mag_l;
    pwm_r = sign_r * pwm_mag_r;
    if (left_target_rpm < TARGET_RPM_EPS) {
      motor_left.brake();
      motor_left_pid.SetMode(MANUAL);
      left_cmd = 0.0;
    } else {
      motor_left.drive(pwm_l);
    }
    if (right_target_rpm < TARGET_RPM_EPS) {
      motor_right.brake();
      motor_right_pid.SetMode(MANUAL);
      right_cmd = 0.0;
    } else {
      motor_right.drive(pwm_r);
    }
  }

#if MOTOR_DEBUG_LOGS
  static int64_t last_motor_log_ms = 0;
  const int64_t now_log_ms = now_ms();
  if (now_log_ms - last_motor_log_ms >= MOTOR_DEBUG_LOG_PERIOD_MS) {
    char log_buf[220];
    snprintf(log_buf, sizeof(log_buf),
             "motor: tgt_rpm L=%.1f R=%.1f | meas_rpm L=%.1f R=%.1f | pwm L=%d R=%d | dt=%.3f",
             (double)left_target_rpm, (double)right_target_rpm,
             (double)left_measured_rpm, (double)right_measured_rpm, pwm_l,
             pwm_r, dt);
    log_debug(log_buf);
    last_motor_log_ms = now_log_ms;
  }
#endif

  // Compact best-effort diagnostics: signed target and encoder RPM per wheel,
  // applied PWM per wheel, and actual control interval in seconds.
  motor_state_data[0] = left_ramped_rpm;
  motor_state_data[1] = right_ramped_rpm;
  motor_state_data[2] = (float)dL / PULSES_PER_REV / dt * 60.0f;
  motor_state_data[3] = (float)dR / PULSES_PER_REV / dt * 60.0f;
  motor_state_data[4] = pwm_l;
  motor_state_data[5] = pwm_r;
  motor_state_data[6] = dt;
  rcl_ret_t motor_state_ret = rcl_publish(&motor_state_pub, &motor_state_msg, NULL);
  (void)motor_state_ret;
}

void cmd_vel_callback(const void *msgin) {
  const geometry_msgs__msg__TwistStamped *msg =
      (const geometry_msgs__msg__TwistStamped *)msgin;

  float v = msg->twist.linear.x;
  if (!isfinite(v) || !isfinite(msg->twist.angular.z)) {
    left_target_rpm_req = right_target_rpm_req = 0.0;
    return;
  }
  float w = constrain(msg->twist.angular.z, -MAX_ANGULAR_SPEED_Z,
                      MAX_ANGULAR_SPEED_Z);

  // -------- Diff Drive --------
  float vL = v - w * WHEEL_BASE / 2.0f;
  float vR = v + w * WHEEL_BASE / 2.0f;

  // Saturate wheel speeds individually - clamp each wheel to max
  // This preserves turning at the cost of forward speed if needed
  if (fabsf(vL) > MAX_WHEEL_SPEED_MPS) {
    vL = (vL > 0) ? MAX_WHEEL_SPEED_MPS : -MAX_WHEEL_SPEED_MPS;
  }
  if (fabsf(vR) > MAX_WHEEL_SPEED_MPS) {
    vR = (vR > 0) ? MAX_WHEEL_SPEED_MPS : -MAX_WHEEL_SPEED_MPS;
  }

  left_direction_coeff =
      (vL >= 0.0f) ? 1.0f : -1.0f; // Update direction coefficient
  right_direction_coeff = (vR >= 0.0f) ? 1.0f : -1.0f;

  left_target_rpm_req = fabs(vL / WHEEL_CIRC * 60.0f);
  right_target_rpm_req = fabs(vR / WHEEL_CIRC * 60.0f);

  last_cmd_vel_ms = now_ms();

#if MOTOR_DEBUG_LOGS
  static int64_t last_cmd_log_ms = 0;
  const int64_t now_log_ms = last_cmd_vel_ms;
  if (now_log_ms - last_cmd_log_ms >= MOTOR_DEBUG_LOG_PERIOD_MS) {
    char log_buf[220];
    snprintf(log_buf, sizeof(log_buf),
             "cmd_vel: v=%.3f w=%.3f | vL=%.3f vR=%.3f | tgt_rpm L=%.1f R=%.1f | dir L=%.0f R=%.0f | timeout=%d",
             v, w, vL, vR, (double)left_target_rpm, (double)right_target_rpm,
             (double)left_direction_coeff, (double)right_direction_coeff,
             0);
    log_debug(log_buf);
    last_cmd_log_ms = now_log_ms;
  }
#endif
}

// ---------------- Arduino setup ----------------
void setup() {

  // activate standby
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);
  motor_left.brake();
  motor_right.brake();

  pinMode(ENC_A_L, INPUT_PULLUP);
  pinMode(ENC_B_L, INPUT_PULLUP);
  pinMode(ENC_A_R, INPUT_PULLUP);
  pinMode(ENC_B_R, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENC_A_L), isrLeft, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_A_R), isrRight, RISING);

  // PID outputs are used as a signed correction term around a feedforward PWM.
  motor_left_pid.SetOutputLimits(-PWM_MAX, PWM_MAX);
  motor_right_pid.SetOutputLimits(-PWM_MAX, PWM_MAX);
  motor_left_pid.SetSampleTime(MOTOR_CONTROL_RATE_MS);
  motor_right_pid.SetSampleTime(MOTOR_CONTROL_RATE_MS);

  motor_left_pid.SetMode(AUTOMATIC);
  motor_right_pid.SetMode(AUTOMATIC);

  Serial.begin(COM_RATE);

  // Initialize micro-ROS serial transport
  set_microros_serial_transports(Serial);

  // wait for agent
  while (rmw_uros_ping_agent(100, 1) != RMW_RET_OK) {
    delay(1000);
  }
  last_agent_ok_ms = now_ms();

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

  // Synchronize the MCU clock with the agent on the Pi, before publishing data.
  while (rmw_uros_sync_session(100) != RMW_RET_OK) {
    esp_task_wdt_reset();
    delay(100);
  }
  last_time_sync_ms = now_ms();

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

  rcl_publisher_options_t imu_options = sensor_pub_options;
  // Imu exceeds the best-effort XRCE MTU; reliable streams allow fragmentation.
  imu_options.qos.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
  ret = rcl_publisher_init(
      &imu_pub, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), "imu/data",
      &imu_options);
  (void)ret;

  rmw_uros_set_publisher_session_timeout(rcl_publisher_get_rmw_handle(&imu_pub), 0);
  sensor_msgs__msg__Imu__init(&imu_msg);
  rosidl_runtime_c__String__assign(&imu_msg.header.frame_id, "imu_link");
  imu_msg.orientation_covariance[0] = -1.0; // MPU6050 has no absolute heading.
  for (int i = 0; i < 9; i += 4) {
    imu_msg.angular_velocity_covariance[i] = 0.0004; // (0.02 rad/s)^2
    imu_msg.linear_acceleration_covariance[i] = 0.04; // (0.2 m/s^2)^2
  }

  log_info("IMU publisher initialized");

  ret = rcl_publisher_init(
      &motor_state_pub, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
      "motor/state", &sensor_pub_options);
  (void)ret;
  motor_state_msg.data.data = motor_state_data;
  motor_state_msg.data.size = 7;
  motor_state_msg.data.capacity = 7;

  // Odometry publisher
  // Odometry exceeds the best-effort transport MTU; reliable XRCE streams
  // support fragmentation. Do not wait for acknowledgements in the control loop.
  rcl_publisher_options_t odom_pub_options = sensor_pub_options;
  odom_pub_options.qos.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
  ret = rcl_publisher_init(&odom_pub, &node,
                           ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
                           "odom", &odom_pub_options);
  (void)ret;
  rmw_uros_set_publisher_session_timeout(rcl_publisher_get_rmw_handle(&odom_pub), 0);

  // Initialize odometry message
  nav_msgs__msg__Odometry__init(&odom_msg);

  rosidl_runtime_c__String__assign(&odom_msg.header.frame_id, "odom");
  rosidl_runtime_c__String__assign(&odom_msg.child_frame_id, "base_link");
  // EKF fuses velocities only; these are initial tuning values, not calibration.
  odom_msg.twist.covariance[0] = 0.0025;
  odom_msg.twist.covariance[7] = 0.0025;
  odom_msg.twist.covariance[35] = 0.01;

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

  rclc_executor_init(&executor, &support.context,
                     TIMER_COUNT + SUBSCRIPTION_COUNT, &allocator);
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
  static int64_t last_sync_attempt_ms = 0;
  if (now_ms() - last_sync_attempt_ms > 30000) {
    last_sync_attempt_ms = now_ms();
    if (rmw_uros_sync_session(10) == RMW_RET_OK) last_time_sync_ms = now_ms();
  }
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
