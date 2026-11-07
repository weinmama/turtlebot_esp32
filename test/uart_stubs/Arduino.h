#pragma once
#include <cstddef>
#include <cstdint>
// Deliberately no read/write/available API: transfer must bypass the HAL mutex.
class HardwareSerial {
 public:
  explicit HardwareSerial(uint8_t port) : _uart_nr(port) {}
 protected:
  uint8_t _uart_nr;
};
