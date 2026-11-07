"""Use the same bounded serial resynchronization in ESP32 and native tests."""
from pathlib import Path
import sys


def patch(source, output):
    text = Path(source).read_text()
    start = text.index("static size_t _z_read_serial_internal(")
    end = text.index("static size_t _z_send_serial_internal(", start)
    # Fail closed if the pinned implementation changes.
    expected = '''static size_t _z_read_serial_internal(const _z_sys_net_socket_t sock, uint8_t *header, uint8_t *ptr, size_t len) {
    uint8_t *raw_buf = (uint8_t *)z_malloc(_Z_SERIAL_MAX_COBS_BUF_SIZE);
    if (raw_buf == NULL) {
        _Z_ERROR("Failed to allocate serial COBS buffer");
        return SIZE_MAX;
    }

    size_t rb = 0;
    while (rb < _Z_SERIAL_MAX_COBS_BUF_SIZE) {
        size_t chunk = _z_serial_read(sock, &raw_buf[rb], 1);
        if (chunk != 1) {
            z_free(raw_buf);
            return SIZE_MAX;
        }

        rb++;
        if (raw_buf[rb - 1] == (uint8_t)0x00) {
            break;
        }
    }

    uint8_t *tmp_buf = (uint8_t *)z_malloc(_Z_SERIAL_MFS_SIZE);
    if (tmp_buf == NULL) {
        z_free(raw_buf);
        _Z_ERROR("Failed to allocate serial MFS buffer");
        return SIZE_MAX;
    }

    size_t ret = _z_serial_msg_deserialize(raw_buf, rb, ptr, len, header, tmp_buf, _Z_SERIAL_MFS_SIZE);
    z_free(raw_buf);
    z_free(tmp_buf);
    return ret;
}

'''
    if text[start:end] != expected:
        raise RuntimeError("Pinned serial reader changed; review resynchronization patch")
    text = text[:start] + '#include "picoros_serial_rx.h"\n\n' + text[end:]
    output = Path(output)
    output.parent.mkdir(parents=True, exist_ok=True)
    if not output.exists() or output.read_text() != text:
        output.write_text(text)


if __name__ == "__main__":
    patch(*sys.argv[1:])
