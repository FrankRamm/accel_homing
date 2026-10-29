#!/usr/bin/env bash
# Restore owned installation transactions from the complete local checkout.
set -euo pipefail
source_dir="$(CDPATH='' cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if ! command -v python3 >/dev/null 2>&1; then
    printf '%s\n' 'Python 3.8 or newer is required. Install python3 and retry.' >&2
    exit 1
fi
for name in installer.py uninstaller.py accel_homing_updater.py accel_homing.py beacon_accel_homing.py accel_homing_backends.py accel_homing_workflows.py accel_homing_model.py accel_homing_charts.py LICENSE NOTICE; do
    if [[ ! -f "$source_dir/$name" || -L "$source_dir/$name" ]]; then
        printf '%s\n' 'uninstall.sh requires the complete local accel_homing checkout. Preserve it until uninstall succeeds.' >&2
        exit 1
    fi
done
exec python3 "$source_dir/installer.py" --uninstall "$@"
