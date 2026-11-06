#!/usr/bin/env bash
# Isolate all test commands from the real robot and desktop ROS sessions.
set -euo pipefail
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
docker run --rm --network none \
    -u "$(id -u):$(id -g)" -e HOME=/tmp -e PICOROS_TEST_ISOLATED=1 \
    -v "$root:/project" --entrypoint bash \
    "${PICOROS_TEST_IMAGE:-turtlebot-backend:test}" -lc '
        set -e
        source /opt/ros/lyrical/setup.bash
        cmake -S /project -B /project/.pio/native -DCMAKE_BUILD_TYPE=Debug -DZENOH_LOG=
        cmake --build /project/.pio/native -j4
        ctest --test-dir /project/.pio/native --output-on-failure
        python3 /project/test/integration.py
        python3 /project/test/integration.py --serial
    '
