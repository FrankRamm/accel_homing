#!/usr/bin/env python3
"""Compatibility entry point for a complete local Accelerometer Homing checkout.

The standalone project installs directly from Git. Downloaded archive bootstrap
scripts are no longer an installation source; this wrapper never uses a network.
"""
from pathlib import Path
import subprocess
import sys


FILES = ("install.sh", "uninstall.sh", "installer.py", "uninstaller.py",
         "accel_homing_updater.py", "accel_homing.py", "beacon_accel_homing.py",
         "accel_homing_backends.py", "accel_homing_workflows.py",
         "accel_homing_model.py", "accel_homing_charts.py", "LICENSE", "NOTICE")


def main(argv=None):
    source = Path(__file__).resolve().parent
    if any(not (source / name).is_file() or (source / name).is_symlink() for name in FILES):
        raise ValueError("Use the complete local checkout: git clone "
                         "https://github.com/FrankRamm/accel_homing.git ~/accel_homing; "
                         "then run ~/accel_homing/install.sh")
    return subprocess.run([sys.executable, str(source / "installer.py")]
                          + list(sys.argv[1:] if argv is None else argv)).returncode


if __name__ == "__main__":
    try:
        sys.exit(main())
    except (OSError, ValueError) as error:
        print("Bootstrap aborted before installation: %s" % error, file=sys.stderr)
        sys.exit(1)
