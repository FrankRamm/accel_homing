#!/usr/bin/env bash
# Compatibility entry point for a complete local checkout. No downloads.
set -euo pipefail
if [[ -z "${BASH_SOURCE[0]-}" || ! -f "${BASH_SOURCE[0]}" ]]; then
    printf '%s\n' 'Clone https://github.com/FrankRamm/accel_homing.git into ~/accel_homing and run ~/accel_homing/install.sh.' >&2
    exit 1
fi
source_dir="$(CDPATH='' cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if [[ ! -f "$source_dir/install.sh" || ! -f "$source_dir/installer.py" ]]; then
    printf '%s\n' 'bootstrap.sh requires the complete local accel_homing checkout; run ~/accel_homing/install.sh.' >&2
    exit 1
fi
exec bash "$source_dir/install.sh" "$@"
