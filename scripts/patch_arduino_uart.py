"""Keep the pinned UART driver, replacing its half-duplex Arduino HAL I/O."""
from pathlib import Path
import hashlib
import sys


def patch(source, output):
    text = Path(source).read_text()
    replacements = [
        ('#include <Arduino.h>', '#include <Arduino.h>\n#include "picoros_uart.h"'),
        ('new HardwareSerial(uart)', 'new PicorosUart(uart)'),
        ('    sock->_serial->begin(baudrate);', '''    // Room for a complete incoming serial frame plus discovery bursts.
    // TX remains synchronous/bounded: do not hide congestion in a TX queue.
    if (sock->_serial->setRxBufferSize(4096) != 4096) {
        delete sock->_serial;
        sock->_serial = NULL;
        return _Z_ERR_GENERIC;
    }
    sock->_serial->begin(baudrate, SERIAL_8N1, rxpin, txpin);'''),
    ]
    start = text.index('static size_t _z_uart_arduino_esp32_read(')
    end = text.index('z_result_t _z_serial_open_from_pins(', start)
    # Do not silently replace changed upstream I/O semantics on a dependency update.
    expected = '531004fb03de88caaa12cf2c9dd6b01c5665daf8a861097ea1e5de1c23223c9f'
    if hashlib.sha256(text[start:end].encode()).hexdigest() != expected:
        raise RuntimeError("Pinned Arduino UART I/O changed; review duplex patch")
    replacements.append((text[start:end], '''static size_t _z_uart_arduino_esp32_read(_z_sys_net_socket_t sock, uint8_t *ptr, size_t len) {
    return static_cast<PicorosUart*>(sock._serial)->receive(ptr, len);
}

static size_t _z_uart_arduino_esp32_write(_z_sys_net_socket_t sock, const uint8_t *ptr, size_t len) {
    return static_cast<PicorosUart*>(sock._serial)->transmit(ptr, len);
}

'''))
    for before, after in replacements:
        if text.count(before) != 1:
            raise RuntimeError("Pinned Arduino UART source changed; review duplex patch")
        text = text.replace(before, after)
    output = Path(output)
    output.parent.mkdir(parents=True, exist_ok=True)
    if not output.exists() or output.read_text() != text:
        output.write_text(text)


if __name__ == '__main__':
    patch(*sys.argv[1:])
