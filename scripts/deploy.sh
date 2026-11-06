#!/usr/bin/env bash
# Desktop build + the existing Pi flash tooling, with Pico-ROS serial ownership.
set -euo pipefail
project="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
backend="$(cd "$project/../../.." && pwd)"
host="${PI_HOST:-pi@robot-pi.local}"
ssh_args=(-o BatchMode=yes -o ConnectTimeout=10)
if [[ -n "${PI_SSH_PORT:-}" ]]; then ssh_args+=(-p "$PI_SSH_PORT"); fi
[[ -f "$backend/scripts/esp32/deploy.sh" ]] || {
    echo 'This wrapper requires the ESP32 checkout inside the turtlebot backend repository.' >&2
    exit 1
}
# Check/build before interrupting the serial service.
bash "$backend/scripts/esp32/build.sh"
ssh "${ssh_args[@]}" "$host" 'bash -s' <<'REMOTE'
set -euo pipefail
if docker compose version >/dev/null 2>&1; then
    compose() { docker compose -p turtlebot-picoros "$@"; }
elif command -v docker-compose >/dev/null 2>&1; then
    compose() { docker-compose -p turtlebot-picoros "$@"; }
else
    echo 'Neither docker compose nor docker-compose is installed' >&2
    exit 1
fi
for component in microros-agent bridge; do
    if systemctl is-active --quiet "turtlebox-hardware@$component.service"; then
        echo "Migrate the Pi services first (see PICOROS_PORT.md): $component is still active" >&2
        exit 1
    fi
done
cd "$HOME/.local/share/turtlebox/picoros"
compose config --quiet
# Refuse to flash if the Pi still has the former single-router deployment.
if ! compose config --services | grep -Fx serial >/dev/null ||
   [[ ! -f serial-router.json5 || ! -f wait-router.sh ]] ||
   ! grep -Fq 'baudrate=250000' serial-router.json5; then
    echo 'Install the current Pico-ROS host files on the Pi before flashing (see PICOROS_PORT.md).' >&2
    exit 1
fi
compose stop
REMOTE
# Parent deploy preserves the flash manifest, esptool validation and flash lock.
# On failure leave the serial services stopped so the flash can be retried.
bash "$backend/scripts/esp32/deploy.sh"
ssh "${ssh_args[@]}" "$host" 'bash -s' <<'REMOTE'
set -euo pipefail
if docker compose version >/dev/null 2>&1; then compose=(docker compose -p turtlebot-picoros)
elif command -v docker-compose >/dev/null 2>&1; then compose=(docker-compose -p turtlebot-picoros)
else exit 1; fi
cd "$HOME/.local/share/turtlebox/picoros"
"${compose[@]}" up -d router
# depends_on only waits for container start; rmw_zenohd can take several
# seconds before its TCP listener exists. Do not start the clock too early.
python3 - <<'PY'
import socket, time
deadline = time.monotonic() + 30
while True:
    try:
        with socket.create_connection(('127.0.0.1', 7447), timeout=1):
            break
    except OSError:
        if time.monotonic() >= deadline:
            raise SystemExit('Zenoh router did not listen within 30 seconds')
        time.sleep(0.5)
PY
"${compose[@]}" up -d serial clock
REMOTE
