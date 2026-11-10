#!/bin/sh
# Remove accel_homing from a Creality K1 (reverts the tagged config edits and module links).
set -eu
exec sh "$(CDPATH='' cd -- "$(dirname -- "$0")" && pwd)/install_k1.sh" --uninstall "$@"
