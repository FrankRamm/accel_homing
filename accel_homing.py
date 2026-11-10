# Generic configuration entrypoint.  The old Beacon module remains supported.
# This file may be distributed under the terms of the GNU GPLv3 license.
from .beacon_accel_homing import BeaconAccelHoming


def load_config(config):
    return BeaconAccelHoming(config)
