#include "robot_cdr.h"
#include <cassert>
#include <cstring>

int main() {
  uint8_t bytes[1024];
  ros_TwistStamped command{};
  command.header.stamp = {1700000000, 123456789};
  command.header.frame_id = const_cast<char*>("base_link");
  command.twist.linear.x = 0.12;
  command.twist.angular.z = -0.34;
  const size_t size = robot_encode(bytes, sizeof(bytes), command, ps_ser_ros_TwistStamped);
  assert(size > 4);
  ros_TwistStamped result{};
  assert(robot_decode(bytes, size, result, ps_des_ros_TwistStamped));
  assert(result.header.stamp.nanosec == 123456789 && result.twist.linear.x == 0.12);
  for (size_t truncated = 0; truncated < size; ++truncated)
    assert(!robot_decode(bytes, truncated, result, ps_des_ros_TwistStamped));
  bytes[1] = 3; // XCDR2 is not supported by this firmware.
  assert(!robot_decode(bytes, size, result, ps_des_ros_TwistStamped));
  bytes[1] = 1;
  memset(bytes + 12, 0xff, 4); // hostile frame_id string length
  assert(!robot_decode(bytes, size, result, ps_des_ros_TwistStamped));
  assert(robot_encode(bytes, 8, command, ps_ser_ros_TwistStamped) == 0);

  float input[7] = {1, 2, 3, 4, 5, 6, 7}, output[6] = {};
  ros_Float32MultiArray tuning{}, decoded{};
  tuning.data.data = input; tuning.data.n_elements = 7;
  decoded.data.data = output; decoded.data.n_elements = 6;
  const size_t tune_size = robot_encode(bytes, sizeof(bytes), tuning, ps_ser_ros_Float32MultiArray);
  assert(!robot_decode(bytes, tune_size, decoded, ps_des_ros_Float32MultiArray));
}
