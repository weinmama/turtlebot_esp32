#!/usr/bin/env bash
# Compose depends_on does not guarantee that rmw_zenohd has opened TCP 7447.
set -euo pipefail
until python3 -c 'import socket; socket.create_connection(("127.0.0.1", 7447), timeout=1).close()' 2>/dev/null; do
    sleep 1
done
exec "$@"
