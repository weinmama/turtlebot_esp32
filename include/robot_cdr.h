#pragma once
#include "picoserdes.h"
#include <cstddef>
#include <cstdint>

// Explicit wrappers avoid the unchecked encapsulation/size handling in the
// upstream generic macros. No heap allocation is needed for our message types.
template <typename T>
size_t robot_encode(uint8_t* data, size_t capacity, T& message,
                    bool (*serialize)(ucdrBuffer*, T*)) {
  if (capacity < 4) return 0;
  data[0] = 0; data[1] = 1; data[2] = 0; data[3] = 0;
  ucdrBuffer buffer;
  ucdr_init_buffer(&buffer, data + 4, capacity - 4);
  if (!serialize(&buffer, &message) || buffer.error) return 0;
  return ucdr_buffer_length(&buffer) + 4;
}

template <typename T>
bool robot_decode(uint8_t* data, size_t size, T& message,
                  bool (*deserialize)(ucdrBuffer*, T*)) {
  if (size < 4 || data[0] != 0 || data[1] > 1 || data[2] != 0 || data[3] != 0) return false;
  ucdrBuffer buffer;
  ucdr_init_buffer_origin_offset_endian(&buffer, data + 4, size - 4, 0, 0,
                          data[1] ? UCDR_LITTLE_ENDIANNESS : UCDR_BIG_ENDIANNESS);
  return deserialize(&buffer, &message) && !buffer.error && ucdr_buffer_remaining(&buffer) == 0;
}
