#include "picoros_uart.h"
#include <algorithm>
#include <cassert>
#include <chrono>
#include <condition_variable>
#include <cstring>
#include <future>
#include <mutex>

static std::mutex mutex;
static std::condition_variable cv;
static bool writing = false, release_write = false, fail_read = false;
static int write_result = 2;
static size_t received = 0;
static const uint8_t input[] = {0x01, 0xff, 0x00, 0x7f};

int uart_read_bytes(uart_port_t port, void* data, size_t size, uint32_t timeout) {
  assert(port == 2 && timeout == portMAX_DELAY);
  if (fail_read) return -1;
  // Exercise fragmented reads, including binary NULs.
  size_t count = std::min(size, size_t{2});
  assert(received + count <= sizeof(input));
  memcpy(data, input + received, count);
  received += count;
  return static_cast<int>(count);
}

int uart_write_bytes(uart_port_t port, const void* data, size_t size) {
  assert(port == 2 && size == sizeof(input));
  assert(memcmp(data, input, size) == 0);
  std::unique_lock<std::mutex> lock(mutex);
  writing = true;
  cv.notify_all();
  cv.wait(lock, [] { return release_write; });
  return write_result;
}

int main() {
  PicorosUart uart(2);
  auto tx = std::async(std::launch::async, [&] { return uart.transmit(input, sizeof(input)); });
  {
    std::unique_lock<std::mutex> lock(mutex);
    assert(cv.wait_for(lock, std::chrono::seconds(1), [] { return writing; }));
  }
  uint8_t output[sizeof(input)]{};
  auto rx = std::async(std::launch::async, [&] { return uart.receive(output, sizeof(output)); });
  // RX must complete while TX is still blocked in the driver.
  assert(rx.wait_for(std::chrono::seconds(1)) == std::future_status::ready);
  assert(rx.get() == sizeof(input) && memcmp(input, output, sizeof(input)) == 0);
  {
    std::lock_guard<std::mutex> lock(mutex);
    release_write = true;
  }
  cv.notify_all();
  assert(tx.get() == 2); // Preserve short writes for serial_protocol's write-all loop.
  fail_read = true;
  assert(uart.receive(output, 1) == SIZE_MAX);
  assert(uart.receive(output, 0) == 0);
  for (int result : {0, -1}) {
    write_result = result;
    assert(uart.transmit(input, sizeof(input)) == SIZE_MAX);
  }
  assert(uart.transmit(input, 0) == 0);
}
