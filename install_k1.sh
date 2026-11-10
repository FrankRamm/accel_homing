#!/bin/sh
# Install accel_homing on a Creality K1 (runs as root, BusyBox sh, no bash/sudo/git needed).
# Usage: sh /usr/data/accel_homing/install_k1.sh [--dry-run] [--no-restart] [--yes] ...
set -eu
source_dir="$(CDPATH='' cd -- "$(dirname -- "$0")" && pwd)"
py=''
for candidate in python3 /usr/bin/python3 /opt/bin/python3 /usr/data/python/bin/python3; do
    if command -v "$candidate" >/dev/null 2>&1; then py="$candidate"; break; fi
done
if [ -z "$py" ]; then
    echo 'python3 not found. Klipper needs it, so check that the K1 firmware/helper script provides it.' >&2
    exit 1
fi
exec "$py" "$source_dir/k1_installer.py" "$@"
