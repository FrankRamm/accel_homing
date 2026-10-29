#!/usr/bin/env bash
# Install from the complete Accelerometer Homing checkout.
set -euo pipefail
source_dir="$(CDPATH='' cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if ! command -v python3 >/dev/null 2>&1; then
    printf '%s\n' 'Python 3.8 or newer is required. Install python3 and retry.' >&2
    exit 1
fi
if [[ ! -f "$source_dir/installer.py" ]]; then
    printf '%s\n' 'Use the complete checkout: git clone https://github.com/FrankRamm/accel_homing.git ~/accel_homing' >&2
    exit 1
fi
exec python3 "$source_dir/installer.py" "$@"
