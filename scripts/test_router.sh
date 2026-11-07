#!/usr/bin/env bash
# Regression for the real Rust router's TX pipeline. No real robot/network.
set -euo pipefail
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
image="turtlebot-picoros-router:congestion-test-amd64"
docker buildx build --platform linux/amd64 --load -t "$image" "$root/host/router"
mkdir -p "$root/.pio/host-router"
container="$(docker create --platform linux/amd64 "$image")"
trap 'docker rm "$container" >/dev/null' EXIT
docker cp "$container:/usr/local/bin/picoros-zenohd" "$root/.pio/host-router/zenohd"
docker run --rm --network none -u "$(id -u):$(id -g)" -e HOME=/tmp \
    -e PICOROS_TEST_ISOLATED=1 -e PICOROS_TEST_SERIAL_ROUTER=/project/.pio/host-router/zenohd \
    -v "$root:/project" --entrypoint bash \
    "${PICOROS_TEST_IMAGE:-turtlebot-backend:test}" -lc '
        set -e
        source /opt/ros/lyrical/setup.bash
        for run in 1 2 3; do
            python3 /project/test/integration.py --serial --split-router --separate-clock --congest-serial
        done
    '
