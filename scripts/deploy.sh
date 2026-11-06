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
for component in microros-agent bridge; do
    if systemctl is-active --quiet "turtlebox-hardware@$component.service"; then
        echo "Migrate the Pi services first (see PICOROS_PORT.md): $component is still active" >&2
        exit 1
    fi
done
cd "$HOME/.local/share/turtlebox/picoros"
docker compose config --quiet
docker compose stop
REMOTE
# Parent deploy preserves the flash manifest, esptool validation and flash lock.
# On failure leave the serial services stopped so the flash can be retried.
bash "$backend/scripts/esp32/deploy.sh"
ssh "${ssh_args[@]}" "$host" 'cd "$HOME/.local/share/turtlebox/picoros" && docker compose up -d'
