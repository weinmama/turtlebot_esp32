#include <cassert>
#include <cstdint>
#include <cstring>
#include <vector>
extern "C" {
#include "zenoh-pico/link/transport/serial_protocol.h"
#include "zenoh-pico/protocol/codec/serial.h"
}

static std::vector<uint8_t> input;
static size_t consumed = 0;
static size_t read_byte(_z_sys_net_socket_t, uint8_t* bytes, size_t size) {
  assert(size == 1);
  if (consumed == input.size()) return SIZE_MAX;
  *bytes = input[consumed++];
  return 1;
}
#define _z_serial_read read_byte
#include "picoros_serial_rx.h"
#undef _z_serial_read

static std::vector<uint8_t> frame(const std::vector<uint8_t>& payload) {
  uint8_t raw[_Z_SERIAL_MAX_COBS_BUF_SIZE], tmp[_Z_SERIAL_MFS_SIZE];
  size_t size = _z_serial_msg_serialize(raw, sizeof(raw), payload.data(),
      payload.size(), 0, tmp, sizeof(tmp));
  assert(size != SIZE_MAX);
  return {raw, raw + size};
}

int main() {
  const std::vector<uint8_t> payload = {1, 2, 0, 3, 4, 5};
  const auto good = frame(payload);
  std::vector<std::vector<uint8_t>> damaged = {
      {0}, {0, 0}, {255, 1, 2, 0}, {2, 0},
      std::vector<uint8_t>(_Z_SERIAL_MAX_COBS_BUF_SIZE * 3, 42)};
  damaged.back().push_back(0);
  auto bad_crc = good;
  bad_crc[4] ^= 1;
  damaged.push_back(bad_crc);
  auto missing_delimiter = good;
  missing_delimiter.pop_back();
  missing_delimiter.insert(missing_delimiter.end(), good.begin(), good.end());
  damaged.push_back(missing_delimiter);
  // Every possible single-byte corruption must be discarded; the next valid
  // frame must survive, including when corruption creates an early delimiter.
  for (size_t i = 0; i < good.size(); ++i) {
    for (unsigned value = 0; value < 256; ++value) {
      if (value == good[i]) continue;
      auto corrupt = good;
      corrupt[i] = value;
      if (i == good.size() - 1) corrupt.push_back(0);
      damaged.push_back(corrupt);
    }
  }
  for (const auto& bad : damaged) {
    input = bad;
    input.insert(input.end(), good.begin(), good.end());
    consumed = 0;
    uint8_t output[1500], header = 255;
    assert(_z_read_serial_internal({}, &header, output, sizeof(output)) == payload.size());
    assert(header == 0 && memcmp(output, payload.data(), payload.size()) == 0);
    assert(consumed == input.size());
    assert(_z_read_serial_internal({}, &header, output, sizeof(output)) == SIZE_MAX);
  }
  // Full MTU with maximal COBS overhead used to exceed decoder scratch size.
  const std::vector<uint8_t> large(1500, 0x55);
  input = frame(large);
  consumed = 0;
  uint8_t output[1500], header;
  assert(_z_read_serial_internal({}, &header, output, sizeof(output)) == large.size());
  assert(memcmp(output, large.data(), large.size()) == 0);
}
