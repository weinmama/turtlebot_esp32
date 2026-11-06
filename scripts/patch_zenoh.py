"""Apply the pinned Zenoh-Pico batched-frame fix into a build directory.

The submodule stays clean. Both native tests and the firmware compile the same
patched source. A changed upstream context fails instead of silently skipping.
"""
from pathlib import Path
import sys


def patched_rx(source, output):
    original = Path(source).read_text()
    context = "    while (_z_zbuf_len(msg->_payload) > 0) {\n"
    replacement = context + """        // Network IDs occupy 0x19..0x1f; transport IDs occupy 0x00..0x0f.
        // A batch may switch reliability/priority by starting another Frame.
        // Leave that header unread for the outer transport decoder instead of
        // treating it as a malformed network message and closing the session.
        if (_Z_MID(*_z_zbuf_get_rptr(msg->_payload)) < 0x10) {
            return _Z_RES_OK;
        }
"""
    if original.count(context) != 1:
        raise RuntimeError("Pinned Zenoh-Pico RX source changed; review batched-frame patch")
    output = Path(output)
    output.parent.mkdir(parents=True, exist_ok=True)
    text = original.replace(context, replacement)
    if not output.exists() or output.read_text() != text:
        output.write_text(text)


if __name__ == "__main__":
    patched_rx(*sys.argv[1:])
