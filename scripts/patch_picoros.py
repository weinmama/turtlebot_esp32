"""Fix 32-bit clock overflow without modifying the pinned Pico-ROS submodule."""
from pathlib import Path
import sys


def patch(source, destination):
    text = Path(source).read_text()
    include = '#include "picoros.h"\n'
    expression = '1000000000 * z_clock_now().tv_sec + z_clock_now().tv_nsec'
    marker = '/* Private functions ---------------------------------------------------------*/\n'
    if text.count(include) != 1 or text.count(expression) != 2 or text.count(marker) != 1:
        raise RuntimeError('Pinned Pico-ROS clock implementation changed; review patch')
    text = text.replace(include, include + '#include "picoros_clock_ns.h"\n')
    text = text.replace(marker, marker + '''static int64_t picoros_clock_now_ns(void) {
    const z_clock_t now = z_clock_now();
    return picoros_clock_ns((int64_t)now.tv_sec, (int64_t)now.tv_nsec);
}

''')
    text = text.replace(expression, 'picoros_clock_now_ns()')
    output = Path(destination)
    output.parent.mkdir(parents=True, exist_ok=True)
    if not output.exists() or output.read_text() != text:
        output.write_text(text)


if __name__ == '__main__':
    patch(*sys.argv[1:])
