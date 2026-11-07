#pragma once

// Included in the pinned serial_protocol.c (and in its regression test).
// A serial link is a stream of delimiter-terminated, CRC-protected datagrams.
// Losing a datagram must not terminate the reader and lose all later commands.
// No retries/replays: only a complete, validated next frame is delivered.
static bool picoros_cobs_valid(const uint8_t *bytes, size_t size) {
    if (size < 2 || bytes[size - 1] != 0) return false;
    const size_t end = size - 1;
    size_t pos = 0;
    while (pos < end) {
        const uint8_t code = bytes[pos++];
        if (code == 0 || (size_t)(code - 1) > end - pos) return false;
        for (size_t i = 1; i < code; ++i) {
            if (bytes[pos++] == 0) return false;
        }
    }
    return pos == end;
}

static size_t _z_read_serial_internal(const _z_sys_net_socket_t sock, uint8_t *header,
                                     uint8_t *ptr, size_t len) {
    uint8_t *raw = (uint8_t *)z_malloc(_Z_SERIAL_MAX_COBS_BUF_SIZE);
    // Upstream's decoder requires scratch space >= encoded length, including
    // COBS overhead, even though the decoded data are smaller.
    uint8_t *tmp = (uint8_t *)z_malloc(_Z_SERIAL_MAX_COBS_BUF_SIZE);
    if (raw == NULL || tmp == NULL) {
        z_free(raw);
        z_free(tmp);
        return SIZE_MAX;
    }
    size_t used = 0;
    bool overflow = false;
    size_t result = SIZE_MAX;
    for (;;) {
        uint8_t byte;
        if (_z_serial_read(sock, &byte, 1) != 1) break;
        if (used < _Z_SERIAL_MAX_COBS_BUF_SIZE) raw[used++] = byte;
        else overflow = true;
        if (byte != 0) continue;
        if (!overflow && picoros_cobs_valid(raw, used)) {
            result = _z_serial_msg_deserialize(raw, used, ptr, len, header,
                                               tmp, _Z_SERIAL_MAX_COBS_BUF_SIZE);
            // Control handshakes carry no payload; CRC covers payload only.
            if (result != SIZE_MAX && (*header == 0 || result == 0)) break;
            result = SIZE_MAX;
        }
        // Drain oversized/truncated/CRC-damaged frames through their delimiter.
        // The motor task's independent watchdog still expires after 500 ms.
        used = 0;
        overflow = false;
    }
    z_free(raw);
    z_free(tmp);
    return result;
}
