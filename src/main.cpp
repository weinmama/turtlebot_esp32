#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Arduino.h>
#include <Wire.h>

#include "robot_transport.h"

#include <esp_timer.h>
#include "esp_task_wdt.h"
#include <driver/gpio.h>
#include <driver/pcnt.h>
#include "freertos/FreeRTOS.h"
#include <PID_v1.h>
#include <TB6612_ESP32.h>
#include "gyro_calibration.h"
#include "encoder_feedback_guard.h"
#include "encoder_pulse_filter.h"
#include "encoder_speed_estimator.h"

// ---------------- Definitions ----------------
#define WDT_TIMEOUT 2              // Secsonds
#define MOTOR_CONTROL_RATE_MS 100  // Milliseconds
#define IMU_UPDATE_RATE_MS 50      // Milliseconds (20 Hz)
#define PULSES_PER_MOTOR_REV 11.0f
#define GEAR_RATIO 46.8f
static constexpr float PULSES_PER_REV = PULSES_PER_MOTOR_REV * GEAR_RATIO;
static constexpr float MAX_MOTOR_RPM = 130.0f;
static constexpr float ENCODER_GUARD_RPM = 140.0f;
static constexpr uint32_t MIN_VALID_ENCODER_EDGE_US =
    static_cast<uint32_t>(60000000.0f / (PULSES_PER_REV * ENCODER_GUARD_RPM) + 0.999f);

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
RobotTransport transport;
ros_TwistStamped cmd_vel_msg{};
ros_Imu imu_msg{};
ros_Odometry odom_msg{};
ros_Float32MultiArray motor_state_msg{};
float motor_state_data[7];

volatile long ticks_L = 0;
volatile long ticks_R = 0;
volatile unsigned long encoder_events = 0;
volatile unsigned long encoder_events_L = 0;
volatile unsigned long encoder_events_R = 0;
volatile unsigned long encoder_raw_edges_L = 0;
volatile unsigned long encoder_raw_edges_R = 0;
volatile unsigned long encoder_rejected_edges_L = 0;
volatile unsigned long encoder_rejected_edges_R = 0;
static portMUX_TYPE encoder_mux = portMUX_INITIALIZER_UNLOCKED;
volatile uint32_t encoder_last_edge_us_L = 0;
volatile uint32_t encoder_last_edge_us_R = 0;
volatile uint32_t encoder_min_edge_interval_us_L = UINT32_MAX;
volatile uint32_t encoder_min_edge_interval_us_R = UINT32_MAX;
static EncoderPulseFilter encoder_pulse_filter_L(MIN_VALID_ENCODER_EDGE_US);
static EncoderPulseFilter encoder_pulse_filter_R(MIN_VALID_ENCODER_EDGE_US);
static EncoderSpeedEstimator encoder_speed_estimator_L;
static EncoderSpeedEstimator encoder_speed_estimator_R;
static int16_t pcnt_previous_L = 0;
static int16_t pcnt_previous_R = 0;
static bool pcnt_previous_valid_L = false;
static bool pcnt_previous_valid_R = false;
static int encoder_direction_L = 0;
static int encoder_direction_R = 0;
static bool pcnt_encoder_ready = false;
static esp_err_t pcnt_left_setup_result = ESP_FAIL;
static esp_err_t pcnt_right_setup_result = ESP_FAIL;
static EncoderFeedbackGuard encoder_feedback_guard;
static bool encoder_fault_latched = false;
static const char *encoder_fault_reason = nullptr;
static float last_valid_left_rpm = 0.0f;
static float last_valid_right_rpm = 0.0f;
static bool encoder_interrupts_ready = false;
static esp_err_t encoder_isr_service_result = ESP_FAIL;
static esp_err_t encoder_left_intr_type_result = ESP_FAIL;
static esp_err_t encoder_right_intr_type_result = ESP_FAIL;
static esp_err_t encoder_left_isr_result = ESP_FAIL;
static esp_err_t encoder_right_isr_result = ESP_FAIL;
GyroCalibration gyro_calibration;
int64_t last_imu_valid_ms = 0, last_odom_valid_ms = 0;

/************** Robot Parameter **************/
const float WHEEL_RADIUS = 0.034f;
const float WHEEL_BASE = 0.20f;

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

void log_info(const char *msg) {
  transport.log(20, msg);
}

void log_warn(const char *msg) {
  transport.log(30, msg);
}

void log_error(const char *msg) {
  transport.log(40, msg);
}

void log_debug(const char *msg) {
  transport.log(10, msg);
}

// ===== Encoder ISRs =====

void isrLeft(void*) {
  const int direction = ENC_DIR_L * ((digitalRead(ENC_B_L) == HIGH) ? 1 : -1);
  const uint32_t edge_us = static_cast<uint32_t>(esp_timer_get_time());
  portENTER_CRITICAL_ISR(&encoder_mux);
  ++encoder_raw_edges_L;
  if (encoder_last_edge_us_L != 0) {
    const uint32_t interval = edge_us - encoder_last_edge_us_L;
    if (interval < encoder_min_edge_interval_us_L)
      encoder_min_edge_interval_us_L = interval;
  }
  encoder_last_edge_us_L = edge_us;
  encoder_speed_estimator_L.record(edge_us);
  if (encoder_pulse_filter_L.accept(edge_us)) {
    ticks_L += direction;
    ++encoder_events_L;
    ++encoder_events;
  } else {
    ++encoder_rejected_edges_L;
  }
  portEXIT_CRITICAL_ISR(&encoder_mux);
}
void isrRight(void*) {
  const int direction = ENC_DIR_R * ((digitalRead(ENC_B_R) == HIGH) ? 1 : -1);
  const uint32_t edge_us = static_cast<uint32_t>(esp_timer_get_time());
  portENTER_CRITICAL_ISR(&encoder_mux);
  ++encoder_raw_edges_R;
  if (encoder_last_edge_us_R != 0) {
    const uint32_t interval = edge_us - encoder_last_edge_us_R;
    if (interval < encoder_min_edge_interval_us_R)
      encoder_min_edge_interval_us_R = interval;
  }
  encoder_last_edge_us_R = edge_us;
  encoder_speed_estimator_R.record(edge_us);
  if (encoder_pulse_filter_R.accept(edge_us)) {
    ticks_R += direction;
    ++encoder_events_R;
    ++encoder_events;
  } else {
    ++encoder_rejected_edges_R;
  }
  portEXIT_CRITICAL_ISR(&encoder_mux);
}

// ===== IMU sampling (20 Hz) =====
void imu_timer_callback() {

  sensors_event_t a, g, temp;
  imu.getEvent(&a, &g, &temp);

  static unsigned long previous_encoder_events = 0;
  portENTER_CRITICAL(&encoder_mux);
  const unsigned long current_encoder_events = encoder_events;
  portEXIT_CRITICAL(&encoder_mux);
  const float acceleration = sqrtf(a.acceleration.x * a.acceleration.x +
      a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z);
  const bool stationary = current_encoder_events == previous_encoder_events &&
      fabs(left_target_rpm_req) < TARGET_RPM_EPS && fabs(right_target_rpm_req) < TARGET_RPM_EPS &&
      std::isfinite(acceleration) && fabsf(acceleration - 9.81f) < 1.0f;
  previous_encoder_events = current_encoder_events;
  gyro_calibration.sample(g.gyro.x, g.gyro.y, g.gyro.z, stationary);
  if (!std::isfinite(acceleration) || !std::isfinite(g.gyro.x) ||
      !std::isfinite(g.gyro.y) || !std::isfinite(g.gyro.z)) return;
  // Do not let the EKF or hardware readiness consume uncalibrated gyro data.
  if (!gyro_calibration.ready()) return;

  int64_t stamp;
  if (!transport.epoch(stamp)) return;
  imu_msg.header.stamp.sec = stamp / 1000000000;
  imu_msg.header.stamp.nanosec = stamp % 1000000000;
  imu_msg.linear_acceleration.x = a.acceleration.x;
  imu_msg.linear_acceleration.y = a.acceleration.y;
  imu_msg.linear_acceleration.z = a.acceleration.z;
  imu_msg.angular_velocity.x = g.gyro.x - gyro_calibration.bias(0);
  imu_msg.angular_velocity.y = g.gyro.y - gyro_calibration.bias(1);
  imu_msg.angular_velocity.z = g.gyro.z - gyro_calibration.bias(2);

  last_imu_valid_ms = esp_timer_get_time() / 1000;
  transport.publish(imu_msg);
}

static int64_t now_ms(void) { return (esp_timer_get_time() / 1000); }

static bool sensors_ready() {
  const int64_t now = now_ms();
  return encoder_interrupts_ready && gyro_calibration.ready() &&
         !encoder_fault_latched &&
         last_imu_valid_ms && last_odom_valid_ms &&
         now - last_imu_valid_ms <= 1000 && now - last_odom_valid_ms <= 1000;
}

static esp_err_t configure_pcnt_encoder(pcnt_unit_t unit, int pulse_pin,
                                        int direction_pin, int direction_sign) {
  pcnt_config_t config{};
  config.pulse_gpio_num = pulse_pin;
  config.ctrl_gpio_num = direction_pin;
  // Match the legacy ISR convention: B high on A rising is positive.
  config.lctrl_mode = direction_sign > 0 ? PCNT_MODE_REVERSE : PCNT_MODE_KEEP;
  config.hctrl_mode = direction_sign > 0 ? PCNT_MODE_KEEP : PCNT_MODE_REVERSE;
  config.pos_mode = PCNT_COUNT_INC;
  config.neg_mode = PCNT_COUNT_DIS;
  config.counter_h_lim = 30000;
  config.counter_l_lim = -30000;
  config.unit = unit;
  config.channel = PCNT_CHANNEL_0;

  esp_err_t result = pcnt_unit_config(&config);
  if (result != ESP_OK) return result;
  result = pcnt_set_filter_value(unit, 1023);
  if (result != ESP_OK) return result;
  result = pcnt_filter_enable(unit);
  if (result != ESP_OK) return result;
  result = pcnt_counter_pause(unit);
  if (result != ESP_OK) return result;
  result = pcnt_counter_clear(unit);
  if (result != ESP_OK) return result;
  return pcnt_counter_resume(unit);
}

static esp_err_t read_pcnt_delta(pcnt_unit_t unit, int16_t& previous,
                                 bool& previous_valid, int16_t& delta) {
  int16_t current = 0;
  esp_err_t result = pcnt_get_counter_value(unit, &current);
  if (result != ESP_OK) return result;
  // PCNT is cleared during setup, so the first read must include all edges
  // observed since boot rather than dropping that initial interval.
  delta = previous_valid ? static_cast<int16_t>(current - previous) : current;
  previous = current;
  previous_valid = true;

  // Keep ample distance from the configured hardware limits during long runs.
  // Clearing does not pause the PCNT unit; only a vanishingly small race at
  // this maintenance point can lose an edge, rather than every control tick.
  if (current >= 20000 || current <= -20000) {
    result = pcnt_counter_clear(unit);
    if (result != ESP_OK) return result;
    previous = 0;
  }
  return ESP_OK;
}

static float normalize_angle(float a) {
  while (a > PI)
    a -= 2.0f * PI;
  while (a < -PI)
    a += 2.0f * PI;
  return a;
}

// PID tuning callback - receives array [Kp_L, Ki_L, Kd_L, Kp_R, Ki_R, Kd_R]
void pid_tune_callback(const ros_Float32MultiArray *msg) {

  if (msg->data.n_elements == 3) {
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

  if (msg->data.n_elements >= 6) {
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

void motor_timer_callback() {
  // Compute dt from actual monotonic elapsed time, including scheduling jitter.
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
  portENTER_CRITICAL(&encoder_mux);
  const uint32_t encoder_now_us = static_cast<uint32_t>(esp_timer_get_time());
  bool left_period_ready = encoder_speed_estimator_L.ready(encoder_now_us);
  bool right_period_ready = encoder_speed_estimator_R.ready(encoder_now_us);
  const float estimated_left_magnitude_rpm = encoder_speed_estimator_L.rpm(
      encoder_now_us, PULSES_PER_REV);
  const float estimated_right_magnitude_rpm = encoder_speed_estimator_R.rpm(
      encoder_now_us, PULSES_PER_REV);
  const long gpio_delta_l = ticks_L;
  ticks_L = 0;
  const long gpio_delta_r = ticks_R;
  ticks_R = 0;
  const unsigned long encoder_events_l = encoder_events_L;
  const unsigned long encoder_events_r = encoder_events_R;
  const unsigned long encoder_raw_edges_l = encoder_raw_edges_L;
  const unsigned long encoder_raw_edges_r = encoder_raw_edges_R;
  const unsigned long encoder_rejected_edges_l = encoder_rejected_edges_L;
  const unsigned long encoder_rejected_edges_r = encoder_rejected_edges_R;
  const uint32_t encoder_min_edge_us_l = encoder_min_edge_interval_us_L;
  const uint32_t encoder_min_edge_us_r = encoder_min_edge_interval_us_R;
  encoder_events_L = 0;
  encoder_events_R = 0;
  encoder_raw_edges_L = 0;
  encoder_raw_edges_R = 0;
  encoder_rejected_edges_L = 0;
  encoder_rejected_edges_R = 0;
  encoder_min_edge_interval_us_L = UINT32_MAX;
  encoder_min_edge_interval_us_R = UINT32_MAX;
  portEXIT_CRITICAL(&encoder_mux);

  int16_t pcnt_delta_l = 0;
  int16_t pcnt_delta_r = 0;
  const esp_err_t pcnt_read_l = read_pcnt_delta(
      PCNT_UNIT_0, pcnt_previous_L, pcnt_previous_valid_L, pcnt_delta_l);
  const esp_err_t pcnt_read_r = read_pcnt_delta(
      PCNT_UNIT_1, pcnt_previous_R, pcnt_previous_valid_R, pcnt_delta_r);
  const bool pcnt_read_ok = pcnt_read_l == ESP_OK && pcnt_read_r == ESP_OK;
  if (!pcnt_read_ok && !encoder_fault_latched) {
    encoder_fault_latched = true;
    encoder_fault_reason = "fault:pcnt_read";
    left_target_rpm_req = right_target_rpm_req = 0.0;
    left_ramped_rpm = right_ramped_rpm = 0.0;
    left_cmd = right_cmd = 0.0;
    motor_left.brake();
    motor_right.brake();
    motor_left_pid.SetMode(MANUAL);
    motor_right_pid.SetMode(MANUAL);
    char pcnt_error[112];
    snprintf(pcnt_error, sizeof(pcnt_error),
        "PCNT read failed left=%d right=%d; motion latched off until MCU restart",
        static_cast<int>(pcnt_read_l), static_cast<int>(pcnt_read_r));
    log_error(pcnt_error);
  }
  // PCNT supplies signed hardware-decoded counts for pose integration and
  // direction. Median GPIO edge periods supply an uncapped speed magnitude;
  // PCNT count rate is a startup fallback until enough periods are observed.
  long dL = pcnt_delta_l;
  long dR = pcnt_delta_r;
  const int new_direction_l = pcnt_delta_l > 0 ? 1 : (pcnt_delta_l < 0 ? -1 : 0);
  const int new_direction_r = pcnt_delta_r > 0 ? 1 : (pcnt_delta_r < 0 ? -1 : 0);
  if (new_direction_l != 0 && encoder_direction_L != 0 &&
      new_direction_l != encoder_direction_L) {
    portENTER_CRITICAL(&encoder_mux);
    encoder_speed_estimator_L.reset();
    portEXIT_CRITICAL(&encoder_mux);
    left_period_ready = false;
  }
  if (new_direction_r != 0 && encoder_direction_R != 0 &&
      new_direction_r != encoder_direction_R) {
    portENTER_CRITICAL(&encoder_mux);
    encoder_speed_estimator_R.reset();
    portEXIT_CRITICAL(&encoder_mux);
    right_period_ready = false;
  }
  if (new_direction_l != 0) encoder_direction_L = new_direction_l;
  if (new_direction_r != 0) encoder_direction_R = new_direction_r;
  const float count_left_rpm = (float)pcnt_delta_l / PULSES_PER_REV / dt * 60.0f;
  const float count_right_rpm = (float)pcnt_delta_r / PULSES_PER_REV / dt * 60.0f;
  const float left_magnitude_rpm = left_period_ready
      ? estimated_left_magnitude_rpm : fabsf(count_left_rpm);
  const float right_magnitude_rpm = right_period_ready
      ? estimated_right_magnitude_rpm : fabsf(count_right_rpm);
  const float raw_left_rpm = encoder_direction_L == 0 ? 0.0f
      : encoder_direction_L * left_magnitude_rpm;
  const float raw_right_rpm = encoder_direction_R == 0 ? 0.0f
      : encoder_direction_R * right_magnitude_rpm;
  const EncoderFeedbackGuard::Result encoder_result =
      encoder_feedback_guard.observe(now_ms(), raw_left_rpm, raw_right_rpm);
  // Odometry is a two-wheel estimate; suppress the whole interval rather than
  // inventing a one-wheel-only pose update when either side is quarantined.
  if (encoder_result.left_invalid || encoder_result.right_invalid) {
    dL = 0;
    dR = 0;
  }
  if (!encoder_result.left_invalid) {
    last_valid_left_rpm = raw_left_rpm;
    left_measured_rpm = fabsf(raw_left_rpm);
  } else {
    // Never feed an impossible sample to PID. Hold only the last valid
    // feedback briefly; do not synthesize feedback from the requested target.
    left_measured_rpm = fabsf(last_valid_left_rpm);
  }
  if (!encoder_result.right_invalid) {
    last_valid_right_rpm = raw_right_rpm;
    right_measured_rpm = fabsf(raw_right_rpm);
  } else {
    right_measured_rpm = fabsf(last_valid_right_rpm);
  }
  if (encoder_result.latched && !encoder_fault_latched) {
    encoder_fault_latched = true;
    encoder_fault_reason = "fault:encoder_overspeed";
    left_target_rpm_req = right_target_rpm_req = 0.0;
    left_ramped_rpm = right_ramped_rpm = 0.0;
    left_cmd = right_cmd = 0.0;
    motor_left.brake();
    motor_right.brake();
    motor_left_pid.SetMode(MANUAL);
    motor_right_pid.SetMode(MANUAL);
    log_error("encoder feedback fault latched after repeated >140 RPM samples; motion disabled until MCU restart");
  }
  const char *status = encoder_fault_reason ? encoder_fault_reason
      : (encoder_interrupts_ready ? "ready" : "fault:capture_setup");
  transport.publish_encoder_status(status);
  if (encoder_result.left_invalid || encoder_result.right_invalid) {
    static int64_t last_encoder_fault_log_ms = 0;
    const int64_t now = now_ms();
    if (encoder_result.latched || now - last_encoder_fault_log_ms >= 100) {
    char encoder_log[256];
    snprintf(encoder_log, sizeof(encoder_log),
        "encoder_invalid pcnt=%ld,%ld gpio=%ld,%ld raw=%lu,%lu reject=%lu,%lu accepted=%lu,%lu edge_min_us=%lu,%lu dt_ms=%.1f rpm=%.1f,%.1f tolerated=%u/3 latch=%d",
        pcnt_delta_l, pcnt_delta_r, gpio_delta_l, gpio_delta_r,
        encoder_raw_edges_l, encoder_raw_edges_r,
        encoder_rejected_edges_l, encoder_rejected_edges_r,
        encoder_events_l, encoder_events_r,
        encoder_min_edge_us_l == UINT32_MAX ? 0UL : (unsigned long)encoder_min_edge_us_l,
        encoder_min_edge_us_r == UINT32_MAX ? 0UL : (unsigned long)encoder_min_edge_us_r,
          dt * 1000.0f, raw_left_rpm, raw_right_rpm,
          static_cast<unsigned>(encoder_result.anomalies_in_window),
          encoder_result.latched ? 1 : 0);
      transport.log(30, encoder_log);
      last_encoder_fault_log_ms = now;
    }
  }

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
  int64_t time_ns = 0;
  const bool synchronized = transport.epoch(time_ns);

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
  if (synchronized && std::isfinite(v_odom) && std::isfinite(w_odom) &&
      std::isfinite(x) && std::isfinite(y) && std::isfinite(theta)) {
    last_odom_valid_ms = now_ms();
    transport.publish(odom_msg);
  }

  // -------- RPM --------
  // Decide whether to stop based on commanded targets (not measured odometry).
  // Using odometry here can keep the robot braked forever at standstill.
  const bool cmd_timed_out =
      (last_cmd_vel_ms != 0 &&
       (now_ms() - last_cmd_vel_ms) > CMD_VEL_TIMEOUT_MS);
  const bool targets_zero = ((fabs(left_target_rpm_req) < TARGET_RPM_EPS) &&
                             (fabs(right_target_rpm_req) < TARGET_RPM_EPS));
  const bool robot_stopped = cmd_timed_out || targets_zero || dt > 0.5f ||
                             encoder_fault_latched ||
                             transport.should_stop(sensors_ready());

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
  // Keep raw measurements visible to diagnostics/host guards. Invalid raw
  // samples above were excluded from odometry and PID input.
  motor_state_data[2] = raw_left_rpm;
  motor_state_data[3] = raw_right_rpm;
  motor_state_data[4] = pwm_l;
  motor_state_data[5] = pwm_r;
  motor_state_data[6] = dt;
#if ROBOT_TRANSPORT_DIAGNOSTICS
  // Encoder outliers and commanded motion without ticks are otherwise hard
  // to distinguish from transport failures using the compact motor topic.
  // Include ISR event counts and rate-limit to avoid adding serial load.
  static int64_t last_encoder_outlier_diagnostic_ms = 0;
  static int64_t last_encoder_missing_diagnostic_ms = 0;
  const int64_t diagnostic_now_ms = now_ms();
  const bool encoder_outlier = fabsf(motor_state_data[2]) > ENCODER_GUARD_RPM ||
                               fabsf(motor_state_data[3]) > ENCODER_GUARD_RPM;
  const bool left_feedback_missing = fabs(left_ramped_rpm) >= 5.0 && dL == 0;
  const bool right_feedback_missing = fabs(right_ramped_rpm) >= 5.0 && dR == 0;
  static int64_t last_encoder_trace_ms = 0;
  if ((fabs(left_ramped_rpm) >= 5.0 || fabs(right_ramped_rpm) >= 5.0) &&
      diagnostic_now_ms - last_encoder_trace_ms >= 1000) {
    char encoder_log[256];
    snprintf(encoder_log, sizeof(encoder_log),
        "encoder_trace pcnt=%ld,%ld gpio=%ld,%ld raw=%lu,%lu reject=%lu,%lu accepted=%lu,%lu edge_min_us=%lu,%lu dt_ms=%.1f rpm=%.1f,%.1f target=%.1f,%.1f pwm=%d,%d",
        pcnt_delta_l, pcnt_delta_r, gpio_delta_l, gpio_delta_r,
        encoder_raw_edges_l, encoder_raw_edges_r,
        encoder_rejected_edges_l, encoder_rejected_edges_r,
        encoder_events_l, encoder_events_r,
        encoder_min_edge_us_l == UINT32_MAX ? 0UL : (unsigned long)encoder_min_edge_us_l,
        encoder_min_edge_us_r == UINT32_MAX ? 0UL : (unsigned long)encoder_min_edge_us_r,
        dt * 1000.0f, raw_left_rpm, raw_right_rpm,
        left_ramped_rpm, right_ramped_rpm, pwm_l, pwm_r);
    transport.log(20, encoder_log);
    last_encoder_trace_ms = diagnostic_now_ms;
  }
  if (encoder_outlier &&
      diagnostic_now_ms - last_encoder_outlier_diagnostic_ms >= 1000) {
    char encoder_log[192];
    snprintf(encoder_log, sizeof(encoder_log),
        "encoder_diag pcnt=%ld,%ld gpio=%ld,%ld raw=%lu,%lu reject=%lu,%lu accepted=%lu,%lu dt_ms=%.1f rpm=%.1f,%.1f target=%.1f,%.1f pwm=%d,%d",
        pcnt_delta_l, pcnt_delta_r, gpio_delta_l, gpio_delta_r,
        encoder_raw_edges_l, encoder_raw_edges_r,
        encoder_rejected_edges_l, encoder_rejected_edges_r,
        encoder_events_l, encoder_events_r, dt * 1000.0f,
        motor_state_data[2], motor_state_data[3],
        left_ramped_rpm, right_ramped_rpm, pwm_l, pwm_r);
    transport.log(30, encoder_log);
    last_encoder_outlier_diagnostic_ms = diagnostic_now_ms;
  }
  if (!encoder_outlier && (left_feedback_missing || right_feedback_missing) &&
      diagnostic_now_ms - last_encoder_missing_diagnostic_ms >= 1000) {
    char encoder_log[192];
    snprintf(encoder_log, sizeof(encoder_log),
        "encoder_no_feedback delta=%ld,%ld isr=%lu,%lu dt_ms=%.1f target=%.1f,%.1f pwm=%d,%d",
        dL, dR, encoder_events_l, encoder_events_r, dt * 1000.0f,
        left_ramped_rpm, right_ramped_rpm, pwm_l, pwm_r);
    transport.log(30, encoder_log);
    last_encoder_missing_diagnostic_ms = diagnostic_now_ms;
  }
#endif
  transport.publish(motor_state_msg);
}

void cmd_vel_callback(const ros_TwistStamped *msg) {

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

  left_target_rpm_req = fminf(MAX_MOTOR_RPM, fabsf(vL / WHEEL_CIRC * 60.0f));
  right_target_rpm_req = fminf(MAX_MOTOR_RPM, fabsf(vR / WHEEL_CIRC * 60.0f));

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

  // PCNT captures A-edge counts and samples B in hardware for direction;
  // GPIO remains a filtered comparison/edge-period diagnostic channel.
  pcnt_left_setup_result = configure_pcnt_encoder(
      PCNT_UNIT_0, ENC_A_L, ENC_B_L, ENC_DIR_L);
  pcnt_right_setup_result = configure_pcnt_encoder(
      PCNT_UNIT_1, ENC_A_R, ENC_B_R, ENC_DIR_R);
  pcnt_encoder_ready = pcnt_left_setup_result == ESP_OK &&
                       pcnt_right_setup_result == ESP_OK;

  // Do not rely on Arduino attachInterrupt(), which discards the result of
  // gpio_isr_handler_add(). A silently rejected left registration would look
  // exactly like persistent zero wheel speed and let the PID wind up.
  encoder_isr_service_result = gpio_install_isr_service(0);
  const bool isr_service_available = encoder_isr_service_result == ESP_OK ||
                                     encoder_isr_service_result == ESP_ERR_INVALID_STATE;
  if (isr_service_available) {
    encoder_left_intr_type_result = gpio_set_intr_type(
        static_cast<gpio_num_t>(ENC_A_L), GPIO_INTR_POSEDGE);
    encoder_right_intr_type_result = gpio_set_intr_type(
        static_cast<gpio_num_t>(ENC_A_R), GPIO_INTR_POSEDGE);
    if (encoder_left_intr_type_result == ESP_OK) {
      encoder_left_isr_result = gpio_isr_handler_add(
          static_cast<gpio_num_t>(ENC_A_L), isrLeft, nullptr);
    }
    if (encoder_right_intr_type_result == ESP_OK) {
      encoder_right_isr_result = gpio_isr_handler_add(
          static_cast<gpio_num_t>(ENC_A_R), isrRight, nullptr);
    }
  }
  encoder_interrupts_ready = isr_service_available &&
      pcnt_encoder_ready &&
      encoder_left_intr_type_result == ESP_OK &&
      encoder_right_intr_type_result == ESP_OK &&
      encoder_left_isr_result == ESP_OK && encoder_right_isr_result == ESP_OK;

  // PID outputs are used as a signed correction term around a feedforward PWM.
  motor_left_pid.SetOutputLimits(-PWM_MAX, PWM_MAX);
  motor_right_pid.SetOutputLimits(-PWM_MAX, PWM_MAX);
  motor_left_pid.SetSampleTime(MOTOR_CONTROL_RATE_MS);
  motor_right_pid.SetSampleTime(MOTOR_CONTROL_RATE_MS);

  motor_left_pid.SetMode(AUTOMATIC);
  motor_right_pid.SetMode(AUTOMATIC);

  // try to initialize IMU
  Wire.begin(SDA_PIN, SCL_PIN);
  if (!imu.begin()) {
    digitalWrite(STBY, LOW);
    delay(1000);
    esp_restart();
  }

  imu_msg.header.frame_id = const_cast<char*>("imu_link");
  imu_msg.orientation_covariance[0] = -1.0; // MPU6050 has no absolute heading.
  for (int i = 0; i < 9; i += 4) {
    imu_msg.angular_velocity_covariance[i] = 0.0004; // (0.02 rad/s)^2
    imu_msg.linear_acceleration_covariance[i] = 0.04; // (0.2 m/s^2)^2
  }

  motor_state_msg.data.data = motor_state_data;
  motor_state_msg.data.n_elements = 7;
  odom_msg.header.frame_id = const_cast<char*>("odom");
  odom_msg.child_frame_id = const_cast<char*>("base_link");
  // EKF fuses velocities only; these are initial tuning values, not calibration.
  odom_msg.twist.covariance[0] = 0.0025;
  odom_msg.twist.covariance[7] = 0.0025;
  odom_msg.twist.covariance[35] = 0.01;

  // UART0 is exclusively owned by Zenoh; never print text to Serial here.
  // The upstream serial handshake blocks if the Pi is absent. Include startup
  // in the watchdog so a lost INIT cannot leave the MCU waiting indefinitely.
  esp_task_wdt_init(WDT_TIMEOUT, true);
  esp_task_wdt_add(NULL);
  // Prime Arduino's UART0 driver before zenoh-pico creates its own
  // HardwareSerial(0). On a cold boot the latter can otherwise accept writes
  // without transmitting them. No text is emitted on the binary Zenoh UART.
  Serial.begin(250000, SERIAL_8N1, 3, 1);
  Serial.end();
  if (!transport.begin()) {
    digitalWrite(STBY, LOW);
    delay(1000);
    esp_restart();
  }
  if (!encoder_interrupts_ready) {
    char encoder_error[224];
    snprintf(encoder_error, sizeof(encoder_error),
        "encoder capture setup failed pcnt=%d,%d service=%d edge=%d,%d handler=%d,%d; motion disabled",
        static_cast<int>(pcnt_left_setup_result),
        static_cast<int>(pcnt_right_setup_result),
        static_cast<int>(encoder_isr_service_result),
        static_cast<int>(encoder_left_intr_type_result),
        static_cast<int>(encoder_right_intr_type_result),
        static_cast<int>(encoder_left_isr_result),
        static_cast<int>(encoder_right_isr_result));
    log_error(encoder_error);
  }
  char encoder_filter_log[128];
  snprintf(encoder_filter_log, sizeof(encoder_filter_log),
      "encoder capture=pcnt0,pcnt1 hw_glitch_filter_cycles=1023 gpio_compare_min_interval_us=%lu speed_glitch_min_us=%lu max_rpm=%.1f",
      (unsigned long)MIN_VALID_ENCODER_EDGE_US,
      (unsigned long)EncoderSpeedEstimator::kMinimumElectricalSpacingUs,
      MAX_MOTOR_RPM);
  log_info(encoder_filter_log);
  // A separate communication task prevents a congested serial connection from
  // delaying motor regulation or its 500 ms command watchdog.
  if (xTaskCreate([](void*) {
        for (;;) { transport.service(); vTaskDelay(pdMS_TO_TICKS(5)); }
      }, "picoros_tx", 8192, nullptr, 1, nullptr) != pdPASS) {
    digitalWrite(STBY, LOW);
    esp_restart();
  }
  esp_task_wdt_reset();
  log_info("Pico-ROS initialized; waiting for clock, mode heartbeat and gyro calibration");
}

// ---------------- Arduino loop ----------------
void loop() {
  static int64_t last_imu_ms = 0, last_motor_ms = 0;
  static int64_t started_ms = now_ms();
  if (transport.command(cmd_vel_msg, sensors_ready())) cmd_vel_callback(&cmd_vel_msg);
  if (transport.should_stop(sensors_ready())) {
    left_target_rpm_req = right_target_rpm_req = 0.0;
    motor_left.brake();
    motor_right.brake();
  }
  float gains[6];
  size_t count;
  if (transport.tuning(gains, count)) {
    ros_Float32MultiArray msg{};
    msg.data.data = gains; msg.data.n_elements = count;
    pid_tune_callback(&msg);
  }
  const int64_t now = now_ms();
  if (now - last_motor_ms >= MOTOR_CONTROL_RATE_MS) {
    last_motor_ms = now;
    motor_timer_callback();
  }
  if (now - last_imu_ms >= IMU_UPDATE_RATE_MS) {
    last_imu_ms = now;
    imu_timer_callback();
  }
  if (now - started_ms > 10000 && !transport.alive()) {
    motor_left.brake();
    motor_right.brake();
    digitalWrite(STBY, LOW);
    esp_restart();
  }
  esp_task_wdt_reset();
  delay(1);
}
