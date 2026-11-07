#pragma once

#include <Arduino.h>
#include <driver/uart.h>

// HardwareSerial owns initialization/teardown, but its Arduino 2.x HAL uses
// one mutex for read AND blocking write. Use IDF's independent RX/TX locks for
// data transfer so receiving heartbeats cannot be starved by telemetry TX.
class PicorosUart : public HardwareSerial {
 public:
  explicit PicorosUart(uint8_t port) : HardwareSerial(port) {}

  size_t receive(uint8_t* data, size_t size) {
    size_t total = 0;
    while (total < size) {
      const int count = uart_read_bytes(static_cast<uart_port_t>(_uart_nr),
          data + total, size - total, portMAX_DELAY);
      if (count <= 0) return SIZE_MAX;
      total += static_cast<size_t>(count);
    }
    return total;
  }

  size_t transmit(const uint8_t* data, size_t size) {
    if (!size) return 0;
    const int count = uart_write_bytes(static_cast<uart_port_t>(_uart_nr), data, size);
    return count > 0 ? static_cast<size_t>(count) : SIZE_MAX;
  }
};
