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
#include <rcl_interfaces/msg/log.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <std_msgs/msg/string.h>

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

// L298N H-Bridge Connection PINs
#define L298N_enB 14 // PWM
#define L298N_enA 27 // PWM
#define L298N_in2 25 // Dir Motor B
#define L298N_in1 26 // Dir Motor B
#define L298N_in4 32 // Dir Motor A
#define L298N_in3 33 // Dir Motor A

// Wheel Encoders Connection PINs
#define right_encoder_phaseA 19 // Interrupt
#define right_encoder_phaseB 18
#define left_encoder_phaseA 5
#define left_encoder_phaseB 17

/************** Pins **************/
// left wheel
#define ENA_L 27
#define IN1_L 33
#define IN2_L 32
#define ENC_A_L 5
#define ENC_B_L 17

// right wheel
#define ENA_R 14
#define IN1_R 26
#define IN2_R 25
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

rcl_publisher_t rosout_pub;
rcl_publisher_t imu_pub;

/************** Robot Parameter **************/
const float WHEEL_RADIUS = 0.034; // m
const float WHEEL_BASE = 0.20;    // m
const float WHEEL_CIRC = 2 * PI * WHEEL_RADIUS;
const int PULSES_PER_REV = 614;

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

float Kp_L = 3.0;
float Ki_L = 4.6;
float Kp_R = 2.0;
float Ki_R = 0.6;


const float MIN_SPEED_X = 0.01;
const float MIN_ANGULAR_SPEED_Z = 0.0001;

/************** Encoder **************/
volatile long ticks_L = 0;
volatile long ticks_R = 0;

/************** Motor **************/
int pwm_L = 0, pwm_R = 0;
float int_L = 0, int_R = 0;

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

// forward declarations - Motor helpers
int motorPI(float target_rpm, float actual_rpm, float &integral, float dt, float Kp, float Ki);
void setDir(float v, int in1, int in2);
void brakeMotor(int in1, int in2);

// ===== Encoder ISRs =====

void isrLeft() { ticks_L += (digitalRead(ENC_B_L) == HIGH) ? 1 : -1; }
void isrRight() { ticks_R -= (digitalRead(ENC_B_R) == HIGH) ? 1 : -1; }

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

  rcl_publish(&imu_pub, &imu_msg, NULL);
}

static int64_t now_ms(void) { return (esp_timer_get_time() / 1000); }

void agent_alive_timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
  (void)last_call_time;
  if (timer == NULL)
    return;

  if (rmw_uros_ping_agent(100, 1) == RMW_RET_OK) {
    last_agent_ok_ms = now_ms();
  }
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

  // -------- Control --------
  if (vL == 0.0f) {
    brakeMotor(IN1_L, IN2_L);
    analogWrite(ENA_L, 0);
    int_L = 0;
  } else {
    setDir(vL, IN1_L, IN2_L);
    int pwm = motorPI(rpmL_target, rpmL, int_L, dt, Kp_L, Ki_L);
    analogWrite(ENA_L, pwm);
  }

  if (vR == 0.0f) {
    brakeMotor(IN1_R, IN2_R);
    analogWrite(ENA_R, 0);
    int_R = 0;
  } else {
    setDir(vR, IN1_R, IN2_R);
    int pwm = motorPI(rpmR_target, rpmR, int_R, dt, Kp_R, Ki_R);
    analogWrite(ENA_R, pwm);
  }
}

void cmd_vel_callback(const void *msgin) {
  const geometry_msgs__msg__TwistStamped *msg =
      (const geometry_msgs__msg__TwistStamped *)msgin;
  v_cmd = fabs(msg->twist.linear.x) < MIN_SPEED_X ? 0.0f : msg->twist.linear.x;
  w_cmd = fabs(msg->twist.angular.z) < MIN_ANGULAR_SPEED_Z ? 0.0f : -msg->twist.angular.z;
}

// ---------------- Arduino setup ----------------
void setup() {

  pinMode(ENA_L, OUTPUT);
  pinMode(IN1_L, OUTPUT);
  pinMode(IN2_L, OUTPUT);
  pinMode(ENA_R, OUTPUT);
  pinMode(IN1_R, OUTPUT);
  pinMode(IN2_R, OUTPUT);

  pinMode(ENC_A_L, INPUT_PULLUP);
  pinMode(ENC_B_L, INPUT_PULLUP);
  pinMode(ENC_A_R, INPUT_PULLUP);
  pinMode(ENC_B_R, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENC_A_L), isrLeft, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_A_R), isrRight, RISING);
  Serial.begin(COM_RATE);

  // Initialize micro-ROS serial transport
  set_microros_serial_transports(Serial);
  // delay(2000); // wait for agent
 
  while (rmw_uros_ping_agent(100, 1) != RMW_RET_OK) {
    delay(1000);
  }

  // wait for agent
  esp_task_wdt_init(WDT_TIMEOUT, true);
  esp_task_wdt_add(NULL); 
  
  Wire.begin(SDA_PIN, SCL_PIN);
  if (!imu.begin()) {
    RCUTILS_LOG_ERROR("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }

  // Allocator & support
  rcl_allocator_t allocator = rcl_get_default_allocator();

  rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
  rcl_init_options_init(&init_options, allocator);
  rcl_init_options_set_domain_id(&init_options, ROS_DOMAIN_ID);  // Dein Domain ID
  rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator);

  // Create node
  rclc_node_init_default(&node, "turtlebot_lowlevel_node", "", &support);

  RCUTILS_LOG_INFO(node.get_logger(), "Turtlebot LowLevel Node initialized.");

  // IMU publisher
  rclc_publisher_init_default(
      &imu_pub, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
      "imu/data");

  imu_msg.data.capacity = 6;
  imu_msg.data.size = 6;
  imu_msg.data.data = (float *)malloc(6 * sizeof(float));

  rclc_timer_init_default2(&imu_timer, &support,
                           RCL_MS_TO_NS(IMU_UPDATE_RATE_MS), imu_timer_callback,
                           true);

  rclc_timer_init_default2(&agent_alive_timer, &support,
                           RCL_MS_TO_NS(AGENT_ALIVE_CHECK_MS),
                           agent_alive_timer_callback, true);
  rclc_timer_init_default2(&motor_timer, &support,
                           RCL_MS_TO_NS(MOTOR_CONTROL_RATE_MS),
                           motor_timer_callback, true);
  rclc_executor_init(&executor, &support.context, 4, &allocator);
  rclc_executor_add_timer(&executor, &agent_alive_timer);
  rclc_executor_add_timer(&executor, &imu_timer);
  rclc_executor_add_timer(&executor, &motor_timer);

  // cmd_vel subscriber
  rclc_subscription_init_default(
      &cmd_vel_sub, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, TwistStamped), "cmd_vel");
  rclc_executor_add_subscription(&executor, &cmd_vel_sub, &cmd_vel_msg,
                                 &cmd_vel_callback, ON_NEW_DATA);

  RCUTILS_LOG_INFO(node.get_logger(), "Turtlebot LowLevel Node started");

}

// ---------------- Arduino loop ----------------
void loop() {
  if (rclc_executor_spin_some(&executor, RCL_MS_TO_NS(EXECUTOR_SPIN_RATE_MS)) !=

      RCL_RET_OK) {
    delay(1);
  }

  if (now_ms() - last_agent_ok_ms > AGENT_DEAD_TIMEOUT_MS) {
    esp_restart();
  } else {
    esp_task_wdt_reset();
  }
}

// ===================== Motor helpers =====================
void setDir(float v, int in1, int in2) {
  if (v >= 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  }
}

void brakeMotor(int in1, int in2) {
  digitalWrite(in1, HIGH);
  digitalWrite(in2, HIGH);
}

int motorPI(float target_rpm, float actual_rpm, float &integral, float dt, float Kp, float Ki) {
  if (fabs(target_rpm) < 0.01f) {
    integral = 0;
    return 0;
  }
  
  // // Deadband auf TARGET-Geschwindigkeit!
  // if (fabs(target_rpm) < 20.0f) {  // 10 RPM = ~0.02 m/s
  //   integral = 0;
  //   return 0;
  // }

  float error = target_rpm - actual_rpm;
  integral += error * dt;
  integral = constrain(integral, -50, 50);
  
  float u = Kp * error + Ki * integral;
  int pwm = abs((int)u);
  return constrain(pwm, PWM_DEADBAND, PWM_MAX);  // Mindestens Deadband
}
