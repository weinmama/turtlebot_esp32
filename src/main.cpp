#include <Arduino.h>
#include <PID_v1.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rcutils/logging_macros.h>
#include <rmw_microros/rmw_microros.h>

#include <rcl_interfaces/msg/log.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <std_msgs/msg/string.h>
#include <geometry_msgs/msg/twist.h>

#include "esp_task_wdt.h"

// ---------------- Definitions ----------------
#define WDT_TIMEOUT 2              // Secsonds
#define AGENT_DEAD_TIMEOUT_MS 5000 // Milliseconds
#define AGENT_ALIVE_CHECK_MS 100   // Milliseconds
#define COM_RATE 115200

// L298N H-Bridge Connection PINs
#define L298N_enB 14 // PWM
#define L298N_enA 27 // PWM
#define L298N_in2 25 // Dir Motor B
#define L298N_in1 26 // Dir Motor B
#define L298N_in4 32 // Dir Motor A
#define L298N_in3 33 // Dir Motor A

// Wheel Encoders Connection PINs
#define right_encoder_phaseA 4 // Interrupt
#define right_encoder_phaseB 0
#define left_encoder_phaseA 2 // Interrupt
#define left_encoder_phaseB 15
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

// config for wheels
#define MOTOR_CONTROL_MS 100 // ms
double right_wheel_radius = 0.034; // meters
double left_wheel_radius = 0.034;  // meters
double wheel_base = 0.16; // meters (distance between wheels)

double right_max_rpm = 100.0; // revolutions per minute
double left_max_rpm = 100.0;  // revolutions per minute

// cmd_vel subscriber
rcl_subscription_t cmd_vel_sub;
geometry_msgs__msg__Twist cmd_vel_msg;
void cmd_vel_callback(const void *msgin);

// PID parameters
// Setpoint - Desired
double right_wheel_cmd_vel = 0.0;      // rad/s
double last_right_wheel_cmd_vel = 0.0; // rad/s
double left_wheel_cmd_vel = 0.0;       // rad/s
double last_left_wheel_cmd_vel = 0.0;  // rad/s

// Input - Measurement
double right_wheel_meas_vel = 0.0; // rad/s
double left_wheel_meas_vel = 0.0;  // rad/s
// Output - Command
double right_wheel_cmd = 0.0; // 0-255
double left_wheel_cmd = 0.0;  // 0-255
// Tuning
double Kp_r = 11.5;
double Ki_r = 7.5;
double Kd_r = 0.1;
double Kp_l = 12.8;
double Ki_l = 8.3;
double Kd_l = 0.1;
// Controller
PID rightMotor(&right_wheel_meas_vel, &right_wheel_cmd, &right_wheel_cmd_vel,
               Kp_r, Ki_r, Kd_r, DIRECT);
PID leftMotor(&left_wheel_meas_vel, &left_wheel_cmd, &left_wheel_cmd_vel, Kp_l,
              Ki_l, Kd_l, DIRECT);

// forward declarations
void leftEncoderCallback();
void rightEncoderCallback();

enum wheel_sign_t { POSITIVE, NEGATIVE };

// Encoders
uint64_t right_encoder_counter = 0;
uint64_t left_encoder_counter = 0;
wheel_sign_t right_wheel_sign = POSITIVE; 
wheel_sign_t left_wheel_sign = POSITIVE;  
uint64_t last_millis = 0;
const uint64_t interval = 100;
// Alive check
static int64_t last_agent_ok_ms = 0;

// -------------------- Messages --------------------
std_msgs__msg__Float32MultiArray imu_msg;
rcl_interfaces__msg__Log log_msg;

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

void motor_timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
  if (timer == NULL)
    return;

  uint64_t interval = last_call_time / 1000000; // interval in ms from ns

  // Right Wheel
  if (right_wheel_sign == wheel_sign_t::POSITIVE) {
    right_wheel_meas_vel =
        (right_encoder_counter * 2.0 * 3.1416) / (interval / 1000.0);
  } else {
    right_wheel_meas_vel =
        -(right_encoder_counter * 2.0 * 3.1416) / (interval / 1000.0);
  }
  right_encoder_counter = 0;

  // Left Wheel
  if (left_wheel_sign == wheel_sign_t::POSITIVE) {
    left_wheel_meas_vel =
        (left_encoder_counter * 2.0 * 3.1416) / (interval / 1000.0);
  } else {
    left_wheel_meas_vel =
        -(left_encoder_counter * 2.0 * 3.1416) / (interval / 1000.0);
  }

  left_encoder_counter = 0;
  right_encoder_counter = 0;

  // Compute new PID output
  rightMotor.Compute();
  leftMotor.Compute();

  // Ignore commands smaller than inertia
  if (right_wheel_cmd_vel == 0.0) {
    right_wheel_cmd = 0.0;
  }
  if (left_wheel_cmd_vel == 0.0) {
    left_wheel_cmd = 0.0;
  }

  right_wheel_cmd = 160;
  left_wheel_cmd = 160;
  // Set Motor PWM using LEDC
  // Ensure duty is 0-255 and convert to int for ledcWrite
  ledcWrite(0, (int)constrain(abs(right_wheel_cmd), 0, 255));
  ledcWrite(1, (int)constrain(abs(left_wheel_cmd), 0, 255));
}

void cmd_vel_callback(const void *msgin) {
  const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;
  double v = msg->linear.x;
  double w = msg->angular.z;

  double right = (v + (w * wheel_base / 2.0)) / right_wheel_radius;
  double left = (v - (w * wheel_base / 2.0)) / left_wheel_radius;

  double right_max_rad_s = (right_max_rpm * 2.0 * 3.141592653589793) / 60.0;
  double left_max_rad_s = (left_max_rpm * 2.0 * 3.141592653589793) / 60.0;

  if (right > right_max_rad_s) right = right_max_rad_s;
  if (right < -right_max_rad_s) right = -right_max_rad_s;
  if (left > left_max_rad_s) left = left_max_rad_s;
  if (left < -left_max_rad_s) left = -left_max_rad_s;

  right_wheel_cmd_vel = right;
  left_wheel_cmd_vel = left;
}

// ---------------- Arduino setup ----------------
void setup() {

  // Init L298N H-Bridge Connection PINs
  pinMode(L298N_enA, OUTPUT);
  pinMode(L298N_enB, OUTPUT);
  pinMode(L298N_in1, OUTPUT);
  pinMode(L298N_in2, OUTPUT);
  pinMode(L298N_in3, OUTPUT);
  pinMode(L298N_in4, OUTPUT);

  // Set Motor Rotation Direction
  digitalWrite(L298N_in1, HIGH);
  digitalWrite(L298N_in2, LOW);
  digitalWrite(L298N_in3, HIGH);
  digitalWrite(L298N_in4, LOW);

  rightMotor.SetMode(AUTOMATIC);
  leftMotor.SetMode(AUTOMATIC);

  // Configure LEDC PWM for motor enable pins
  const int pwmFreq = 20000; // 20 kHz
  const int pwmResolution = 8; // 8-bit (0-255)
  // Channel 0 -> Right motor (L298N_enA)
  // Channel 1 -> Left motor  (L298N_enB)
  ledcSetup(0, pwmFreq, pwmResolution);
  ledcAttachPin(L298N_enA, 0);
  ledcSetup(1, pwmFreq, pwmResolution);
  ledcAttachPin(L298N_enB, 1);

  // Init encoders
  pinMode(right_encoder_phaseB, INPUT);
  pinMode(left_encoder_phaseB, INPUT);
  // Set Callback for Wheel Encoders Pulse
  attachInterrupt(digitalPinToInterrupt(right_encoder_phaseA),
                  rightEncoderCallback, RISING);
  attachInterrupt(digitalPinToInterrupt(left_encoder_phaseA),
                  leftEncoderCallback, RISING);

  Serial.begin(COM_RATE);
  delay(2000);

  // Initialize micro-ROS serial transport
  set_microros_serial_transports(Serial);
  delay(2000); // wait for agent

  esp_task_wdt_init(WDT_TIMEOUT, true);
  esp_task_wdt_add(NULL); // loop task

  // wait for agent
  while (rmw_uros_ping_agent(100, 1) != RMW_RET_OK) {
    delay(1000);
    esp_task_wdt_reset();
  }

  Wire.begin(SDA_PIN, SCL_PIN);
  if (!imu.begin()) {
    RCUTILS_LOG_ERROR("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }

  // Allocator & support
  rcl_allocator_t allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);

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

  const uint64_t timer_period_ns = RCL_MS_TO_NS(20);
  rclc_timer_init_default2(&imu_timer, &support, timer_period_ns,
                           imu_timer_callback, true);

  rclc_timer_init_default2(&agent_alive_timer, &support,
                           RCL_MS_TO_NS(AGENT_ALIVE_CHECK_MS),
                           agent_alive_timer_callback, true);
  rclc_timer_init_default2(&motor_timer, &support,
                           RCL_MS_TO_NS(MOTOR_CONTROL_MS), motor_timer_callback,
                           true);

  rclc_executor_init(&executor, &support.context, 3, &allocator);
  rclc_executor_add_timer(&executor, &agent_alive_timer);
  rclc_executor_add_timer(&executor, &imu_timer);
  rclc_executor_add_timer(&executor, &motor_timer);

  // cmd_vel subscriber
  rclc_subscription_init_default(&cmd_vel_sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel");
  rclc_executor_add_subscription(&executor, &cmd_vel_sub, &cmd_vel_msg, &cmd_vel_callback, ON_NEW_DATA);

  RCUTILS_LOG_INFO(node.get_logger(), "Turtlebot LowLevel Node started");
}

// ---------------- Arduino loop ----------------
void loop() {
  if (rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5)) != RCL_RET_OK) {
    delay(1);
  }

  if (now_ms() - last_agent_ok_ms > AGENT_DEAD_TIMEOUT_MS) {
    esp_restart();
  } else {
    esp_task_wdt_reset();
  }
}

// New pulse from Right Wheel Encoder
void rightEncoderCallback() {
  if (digitalRead(right_encoder_phaseB) == HIGH) {
    right_wheel_sign = wheel_sign_t::POSITIVE;
  } else {
    right_wheel_sign = wheel_sign_t::NEGATIVE;
  }
  right_encoder_counter++;
}

// New pulse from Left Wheel Encoder
void leftEncoderCallback() {
  if (digitalRead(left_encoder_phaseB) == HIGH) {
    left_wheel_sign = wheel_sign_t::POSITIVE;
  } else {
    left_wheel_sign = wheel_sign_t::NEGATIVE;
  }
  left_encoder_counter++;
}