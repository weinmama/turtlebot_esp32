#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Arduino.h>
#include <Wire.h>

#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rclc_parameter/rclc_parameter.h>
#include <rcutils/logging_macros.h>
#include <rmw_microros/rmw_microros.h>

#include <geometry_msgs/msg/twist_stamped.h>
#include <rcl_interfaces/msg/log.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <std_msgs/msg/string.h>

#include <TB6612_ESP32.h>
#include "esp_task_wdt.h"


// ---------------- Definitions ----------------
#define WDT_TIMEOUT 2              // Secsonds
#define AGENT_DEAD_TIMEOUT_MS 5000 // Milliseconds
#define AGENT_ALIVE_CHECK_MS 100   // Milliseconds
#define MOTOR_CONTROL_RATE_MS 100  // Milliseconds
#define EXECUTOR_SPIN_RATE_MS 5    // Milliseconds
#define IMU_UPDATE_RATE_MS 20      // Milliseconds
#define COM_RATE 115200
#define ROS_DOMAIN_ID 1

// TB6012FNG H-Bridge Connection PINs
#define AIN1 26  // Motor A direction pin 1
#define AIN2 25  // Motor A direction pin 2
#define PWMA 27  // Motor A PWM speed control
#define BIN1 33  // Motor B direction pin 1
#define BIN2 32  // Motor B direction pin 2
#define PWMB 14  // Motor B PWM speed control
#define STBY 12  // Standby pin (must be HIGH to enable motors)

// Wheel Encoders Connection PINs
#define right_encoder_phaseA 19 // Interrupt
#define right_encoder_phaseB 18
#define left_encoder_phaseA 5
#define left_encoder_phaseB 17

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
rcl_timer_t agent_alive_timer;
rcl_timer_t motor_timer;
rclc_executor_t executor;
rclc_parameter_server_t param_server;

rcl_publisher_t rosout_pub;
rcl_publisher_t imu_pub;
bool rosout_pub_initialized = false;

/************** Robot Parameter **************/
const float WHEEL_RADIUS = 0.034; // m
const float WHEEL_BASE = 0.20;    // m
const float WHEEL_CIRC = 2 * PI * WHEEL_RADIUS;
const int PULSES_PER_REV = 614;
// Motor-Objekte für TB6012FNG (Offset für Motor Direction)
const int offsetA = -1;  // Invertiert für Motor rechts
const int offsetB = -1;  // Invertiert für Motor links
// Motor(In1pin, In2pin, PWMpin, offset, STBYpin, freq, resolution, channel_pin)
Motor motor_right= Motor(AIN1, AIN2, PWMA, offsetA, STBY, 5000, 8, 1);
Motor motor_left = Motor(BIN1, BIN2, PWMB, offsetB, STBY, 5000, 8, 2);

/************** L298 **************/
const int PWM_DEADBAND = 0;
const int PWM_MAX = 255;

/************** Ramps **************/
const float A_MAX = 0.5;     // m/s²
const float ALPHA_MAX = 2.0; // rad/s²

/************** Control **************/
static float v_cmd = 0.0; // velocity [m/s]
static float w_cmd = 0.0; // angular velocity [rad/s]

static float v = 0.0; // ramped velocity
static float w = 0.0;

float Kp_L = 5.8;
float Ki_L = 2.9;
float Kd_L = 0.04;
float Kp_R = Kp_L;
float Ki_R = Ki_L;
float Kd_R = Kd_L;

const float MIN_SPEED_X = 0.01;
const float MIN_ANGULAR_SPEED_Z = 0.0001;

/************** Encoder **************/
volatile long ticks_L = 0;
volatile long ticks_R = 0;

/************** Motor **************/
int pwm_L = 0, pwm_R = 0;
float int_L = 0, int_R = 0;
float prev_error_L = 0, prev_error_R = 0;

// For tuning/debugging
float p_term_L = 0, i_term_L = 0, d_term_L = 0;
float p_term_R = 0, i_term_R = 0, d_term_R = 0;

/************** Odometry **************/ 
float x = 0.0, y = 0.0, theta = 0.0;

// cmd_vel subscriber
rcl_subscription_t cmd_vel_sub;
geometry_msgs__msg__TwistStamped cmd_vel_msg;
void cmd_vel_callback(const void *msgin);

// Alive check
static int64_t last_agent_ok_ms = 0;

// -------------------- Messages --------------------
std_msgs__msg__Float32MultiArray imu_msg;
rcl_interfaces__msg__Log log_msg;

// -------------------- Logging Helpers --------------------
void log_message(uint8_t level, const char* name, const char* msg) {
  if (!rosout_pub_initialized) return;
  
  log_msg.stamp.sec = rmw_uros_epoch_millis() / 1000;
  log_msg.stamp.nanosec = (rmw_uros_epoch_millis() % 1000) * 1000000;
  log_msg.level = level;
  
  // Set name - manually copy string
  log_msg.name.data = (char*)name;
  log_msg.name.size = strlen(name);
  log_msg.name.capacity = strlen(name) + 1;
  
  // Set message - manually copy string
  log_msg.msg.data = (char*)msg;
  log_msg.msg.size = strlen(msg);
  log_msg.msg.capacity = strlen(msg) + 1;
  
  // Set function, file, line (optional)
  log_msg.function.data = (char*)"";
  log_msg.function.size = 0;
  log_msg.function.capacity = 1;
  
  log_msg.file.data = (char*)"";
  log_msg.file.size = 0;
  log_msg.file.capacity = 1;
  
  log_msg.line = 0;
  
  rcl_ret_t ret = rcl_publish(&rosout_pub, &log_msg, NULL);
  (void)ret; // Suppress unused warning
}

void log_info(const char* msg) {
  log_message(rcl_interfaces__msg__Log__INFO, "turtlebot_lowlevel_node", msg);
}

void log_warn(const char* msg) {
  log_message(rcl_interfaces__msg__Log__WARN, "turtlebot_lowlevel_node", msg);
}

void log_error(const char* msg) {
  log_message(rcl_interfaces__msg__Log__ERROR, "turtlebot_lowlevel_node", msg);
}

void log_debug(const char* msg) {
  log_message(rcl_interfaces__msg__Log__DEBUG, "turtlebot_lowlevel_node", msg);
}

// forward declarations - Motor helpers
int motorPID(float target_rpm, float actual_rpm, float &integral, float &prev_error, float dt, float Kp, float Ki, float Kd, float &p_term, float &i_term, float &d_term);

// ===== Encoder ISRs =====

void isrLeft() { ticks_L += (digitalRead(ENC_B_L) == HIGH) ? 1 : -1; }
void isrRight() { ticks_R += (digitalRead(ENC_B_R) == HIGH) ? 1 : -1; }

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

void agent_alive_timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
  (void)last_call_time;
  if (timer == NULL)
    return;

  if (rmw_uros_ping_agent(100, 1) == RMW_RET_OK) {
    last_agent_ok_ms = now_ms();
    esp_task_wdt_reset();
  }

  
  // // Sync parameters from parameter server (only every 1 second to avoid blocking)
  // static unsigned long last_param_sync = 0;
  // unsigned long now = millis();
  // if (now - last_param_sync >= 1000) {
  //   double kp_l_val, ki_l_val, kd_l_val, kp_r_val, ki_r_val, kd_r_val;
  //   if (rclc_parameter_get_double(&param_server, "Kp_L", &kp_l_val) == RCL_RET_OK) {
  //     Kp_L = (float)kp_l_val;
  //   }
  //   if (rclc_parameter_get_double(&param_server, "Ki_L", &ki_l_val) == RCL_RET_OK) {
  //     Ki_L = (float)ki_l_val;
  //   }
  //   if (rclc_parameter_get_double(&param_server, "Kd_L", &kd_l_val) == RCL_RET_OK) {
  //     Kd_L = (float)kd_l_val;
  //   }
  //   if (rclc_parameter_get_double(&param_server, "Kp_R", &kp_r_val) == RCL_RET_OK) {
  //     Kp_R = (float)kp_r_val;
  //   }
  //   if (rclc_parameter_get_double(&param_server, "Ki_R", &ki_r_val) == RCL_RET_OK) {
  //     Ki_R = (float)ki_r_val;
  //   }
  //   if (rclc_parameter_get_double(&param_server, "Kd_R", &kd_r_val) == RCL_RET_OK) {
  //     Kd_R = (float)kd_r_val;
  //   }
  //   last_param_sync = now;
  // }
}
static int64_t last_time_ns = 0;

void motor_timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
  
  if (timer == NULL)
    return;

  float dt = last_call_time / 1e9f; // interval ns in secs


  if (dt <= 0.0f || dt > 0.5f)
    return;

  // -------- Ramp --------
  v += constrain(v_cmd - v, -A_MAX * dt, A_MAX * dt);
  w += constrain(w_cmd - w, -ALPHA_MAX * dt, ALPHA_MAX * dt);

  // -------- Diff Drive --------
  float vL = v - w * WHEEL_BASE / 2.0f;
  float vR = v + w * WHEEL_BASE / 2.0f;

  float rpmL_target = vL / WHEEL_CIRC * 60.0f;
  float rpmR_target = vR / WHEEL_CIRC * 60.0f;

  // -------- Encoder --------
  noInterrupts();
  long dL = ticks_L; ticks_L = 0;
  long dR = ticks_R; ticks_R = 0;
  interrupts();

  float dsL = (float)dL / PULSES_PER_REV * WHEEL_CIRC;
  float dsR = (float)dR / PULSES_PER_REV * WHEEL_CIRC;

  // -------- Odometry --------
  float ds = (dsR + dsL) * 0.5f;
  float dtheta = (dsR - dsL) / WHEEL_BASE;

  x += ds * cos(theta + dtheta * 0.5f);
  y += ds * sin(theta + dtheta * 0.5f);
  theta += dtheta;

  // -------- RPM --------
  float rpmL = (dsL / WHEEL_CIRC) / dt * 60.0f;
  float rpmR = (dsR / WHEEL_CIRC) / dt * 60.0f;

  // Calculate actual velocities from encoder measurements
  float vL_actual = dsL / dt;
  float vR_actual = dsR / dt;

  // -------- Debug Logging --------
  static unsigned long last_log_ms = 0;
  unsigned long now = millis();
  if (1) { // Log every 100ms for tuning
    char log_buffer[160];
    snprintf(log_buffer, sizeof(log_buffer), 
             "cmd:%.3f v:%.3f | vL:%.3f(%.3f) vR:%.3f(%.3f) | rpmL:%.1f(%.1f) rpmR:%.1f(%.1f)",
             v_cmd, v, vL, vL_actual, vR, vR_actual, rpmL_target, rpmL, rpmR_target, rpmR);
    log_debug(log_buffer);
    last_log_ms = now;
  }

  // -------- Control --------
  int pwm_L_out = 0, pwm_R_out = 0;
  
  if (vL == 0.0f) {
    motor_left.brake();
    int_L = 0;
    prev_error_L = 0;
    p_term_L = i_term_L = d_term_L = 0;
  } else {
    pwm_L_out = motorPID(rpmL_target, rpmL, int_L, prev_error_L, dt, Kp_L, Ki_L, Kd_L, p_term_L, i_term_L, d_term_L);
    motor_left.drive(pwm_L_out);
  }

  if (vR == 0.0f) {
    motor_right.brake();
    int_R = 0;
    prev_error_R = 0;
    p_term_R = i_term_R = d_term_R = 0;
  } else {
    pwm_R_out = motorPID(rpmR_target, rpmR, int_R, prev_error_R, dt, Kp_R, Ki_R, Kd_R, p_term_R, i_term_R, d_term_R);
    motor_right.drive(pwm_R_out);
  }
  
  // Detailed tuning logs
  if (now - last_log_ms > 100) {
    char tune_buffer[128];
    snprintf(tune_buffer, sizeof(tune_buffer), 
             "L: P=%.1f I=%.1f D=%.1f PWM=%d | R: P=%.1f I=%.1f D=%.1f PWM=%d",
             p_term_L, i_term_L, d_term_L, pwm_L_out,
             p_term_R, i_term_R, d_term_R, pwm_R_out);
    log_debug(tune_buffer);
  }
}

void cmd_vel_callback(const void *msgin) {
  const geometry_msgs__msg__TwistStamped *msg =
      (const geometry_msgs__msg__TwistStamped *)msgin;
  v_cmd = fabs(msg->twist.linear.x) < MIN_SPEED_X ? 0.0f : msg->twist.linear.x;
  w_cmd = fabs(msg->twist.angular.z) < MIN_ANGULAR_SPEED_Z ? 0.0f : msg->twist.angular.z;
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
  
    //Reset Watchdog to be save - don't know if it helps
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
  
  // Initialize rosout publisher with system default QoS (transient local, reliable)
  rmw_qos_profile_t rosout_qos = rmw_qos_profile_system_default;
  rosout_qos.depth = 10;
  rosout_qos.durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
  rosout_qos.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
  
  rcl_publisher_options_t rosout_pub_options = rcl_publisher_get_default_options();
  rosout_pub_options.qos = rosout_qos;
  
  ret = rcl_publisher_init(
      &rosout_pub, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(rcl_interfaces, msg, Log),
      "/rosout", &rosout_pub_options);
  (void)ret;
  
  // Initialize log message
  rcl_interfaces__msg__Log__init(&log_msg);
  rosout_pub_initialized = true;
  
  log_info("Node initialized");

  // IMU publisher
  rclc_publisher_init_default(
      &imu_pub, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
      "imu/data");

  imu_msg.data.capacity = 6;
  imu_msg.data.size = 6;
  imu_msg.data.data = (float *)malloc(6 * sizeof(float));
  
  log_info("IMU publisher initialized");

  rclc_timer_init_default2(&imu_timer, &support,
                           RCL_MS_TO_NS(IMU_UPDATE_RATE_MS), imu_timer_callback,
                           true);

  rclc_timer_init_default2(&agent_alive_timer, &support,
                           RCL_MS_TO_NS(AGENT_ALIVE_CHECK_MS),
                           agent_alive_timer_callback, true);
  rclc_timer_init_default2(&motor_timer, &support,
                           RCL_MS_TO_NS(MOTOR_CONTROL_RATE_MS),
                           motor_timer_callback, true);

  //Reset Watchdog to be save - don't know if it helps
  esp_task_wdt_reset();

  // Initialize parameter server (6 parameters: Kp_L, Ki_L, Kd_L, Kp_R, Ki_R, Kd_R)
  rclc_parameter_server_init_default(&param_server, &node);
  esp_task_wdt_reset();
  
  // Add PID parameters
  rclc_add_parameter(&param_server, "Kp_L", RCLC_PARAMETER_DOUBLE);
  rclc_add_parameter(&param_server, "Ki_L", RCLC_PARAMETER_DOUBLE);
  rclc_add_parameter(&param_server, "Kd_L", RCLC_PARAMETER_DOUBLE);
  esp_task_wdt_reset();
  rclc_add_parameter(&param_server, "Kp_R", RCLC_PARAMETER_DOUBLE);
  rclc_add_parameter(&param_server, "Ki_R", RCLC_PARAMETER_DOUBLE);
  rclc_add_parameter(&param_server, "Kd_R", RCLC_PARAMETER_DOUBLE);
  esp_task_wdt_reset();
  
  // Set initial parameter values
  rclc_parameter_set_double(&param_server, "Kp_L", Kp_L);
  rclc_parameter_set_double(&param_server, "Ki_L", Ki_L);
  rclc_parameter_set_double(&param_server, "Kd_L", Kd_L);
  rclc_parameter_set_double(&param_server, "Kp_R", Kp_R);
  rclc_parameter_set_double(&param_server, "Ki_R", Ki_R);
  rclc_parameter_set_double(&param_server, "Kd_R", Kd_R);
  esp_task_wdt_reset();
  
  log_info("Parameter server initialized");

  rclc_executor_init(&executor, &support.context, 5, &allocator);
  rclc_executor_add_timer(&executor, &agent_alive_timer);
  rclc_executor_add_timer(&executor, &imu_timer);
  rclc_executor_add_timer(&executor, &motor_timer);
  rclc_executor_add_parameter_server(&executor, &param_server, NULL);

  // cmd_vel subscriber
  rclc_subscription_init_default(
      &cmd_vel_sub, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, TwistStamped), "cmd_vel");
  rclc_executor_add_subscription(&executor, &cmd_vel_sub, &cmd_vel_msg,
                                 &cmd_vel_callback, ON_NEW_DATA);
  
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
int motorPID(float target_rpm, float actual_rpm, float &integral, float &prev_error, float dt, float Kp, float Ki, float Kd, float &p_term, float &i_term, float &d_term) {
  if (fabs(target_rpm) < 0.01f) {
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
    float alpha = 0.5; // filter coefficient
    if (&prev_error == &prev_error_L) {
      filtered_deriv_L = alpha * derivative + (1 - alpha) * filtered_deriv_L;
      derivative = filtered_deriv_L;
    } else {
      filtered_deriv_R = alpha * derivative + (1 - alpha) * filtered_deriv_R;
      derivative = filtered_deriv_R;
    }
  }
  
  // Calculate individual terms
  p_term = Kp * error;
  i_term = Ki * integral;
  d_term = Kd * derivative;
  
  float u = p_term + i_term + d_term;
  int pwm = (int)u;
  
  // Anti-windup: only integrate if not saturated
  if ((pwm < PWM_MAX && pwm > -PWM_MAX) || (error * integral < 0)) {
    integral += error * dt;
    integral = constrain(integral, -100, 100);
  }
  
  // Store error for next iteration
  prev_error = error;
  
  if (pwm > 0) {
    return constrain(pwm, PWM_DEADBAND, PWM_MAX);
  } else {
    return constrain(pwm, -PWM_MAX, -PWM_DEADBAND);
  } 
}
