#pragma once
#include <cstddef>
#include <cstdint>
using uart_port_t = int;
constexpr uint32_t portMAX_DELAY = UINT32_MAX;
int uart_read_bytes(uart_port_t port, void* data, size_t size, uint32_t timeout);
int uart_write_bytes(uart_port_t port, const void* data, size_t size);
