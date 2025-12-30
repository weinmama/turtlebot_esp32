#include <Arduino.h>

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

#include "esp_task_wdt.h"

#define WDT_TIMEOUT 5 // Secsonds
#define SDA_PIN 21
#define SCL_PIN 22

// -------------------- IMU --------------------
Adafruit_MPU6050 imu;

// ---------------- Globals ----------------
rclc_support_t support;
rcl_node_t node;
rcl_timer_t imu_timer;
rclc_executor_t executor;

rcl_publisher_t rosout_pub;
rcl_publisher_t imu_pub;

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

// ---------------- Arduino setup ----------------
void setup() {
  Serial.begin(115200);
  delay(2000);

  // Initialize micro-ROS serial transport
  set_microros_serial_transports(Serial);
  delay(2000); // wait for agent

  esp_task_wdt_init(WDT_TIMEOUT, true);
  esp_task_wdt_add(NULL); // loop task

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
  rclc_timer_init_default(&imu_timer, &support, timer_period_ns,
                          imu_timer_callback);

  rclc_executor_init(&executor, &support.context, 1, &allocator);
  rclc_executor_add_timer(&executor, &imu_timer);

  // Initial l
  RCUTILS_LOG_INFO(node.get_logger(), "Turtlebot LowLevel Node started");
}

// ---------------- Arduino loop ----------------
void loop() {
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5));
  esp_task_wdt_reset();
}
