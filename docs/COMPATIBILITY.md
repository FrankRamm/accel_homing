# Supported sensors and limits

This is an experimental host-side impact endstop. Driver/API compatibility and
synthetic test results do not establish physical contact detection, repeatability
or stopping distance for your printer. Supervise first homes and workflows.
Historical reports from the original Beacon plugin do not validate every adapter
or workflow added here.

## Sensors

The sensor must be firmly mounted and move with **each axis being tested**.
A toolhead sensor on a moving-bed Cartesian printer does not measure Y bed-stop
impacts reliably just because it measures X. There is one selected sensor,
without automatic dual-sensor routing. One-axis installation is available;
TEST requires both axes, while CHARTS/ACCURACY can select an installed axis.

| Sensor/object | Adapter and evidence | Standard configured range |
| --- | --- | --- |
| Beacon | Native raw accelerometer stream; inherited detector plus offline fixtures. | Determined from native stream when available. |
| ADXL345 | Configured timestamped `batch_bulk` or verified legacy `api_dump` data. | 16 g |
| LIS2DW / LIS3DH | Standard configured accelerometer driver contract. | 16 g |
| MPU9250 | Standard configured accelerometer driver contract. | 8 g |
| ICM20948 | Standard configured accelerometer driver contract. | 16 g |
| Cartographer | Its separately configured ADXL345/LIS2DW object through the standard adapter. | Range of that driver. |
| Other compatible object | Explicit full `accel_chip`, actual `sensor_range`, and runtime API checks. | Must be declared correctly. |

A product name, connector or probe section does not establish a usable
accelerometer interface. Configure the sensor normally first. `accel_chip: auto`
requires exactly one recognized configured section; otherwise choose its full
section name, preserving suffix case. Saved/model sections or ambiguous sensor
inventory can require an explicit selection instead of auto.

Generic input is timestamped machine-mapped acceleration in mm/s², converted
to g. The simple calibration/vector model uses three-axis magnitude, which
does not require a homing-axis channel/sign mapping. Correct `axes_map` remains
important for other features and the legacy selected-axis detector.

## Host contracts and evidence

The plugin needs endstop chip registration, normal homing events, MCU
`TriggerDispatch`/trsync support and a supported accelerometer delivery contract.
It checks capabilities rather than declaring support from a checkout name.
A legacy `api_dump` adapter alone does not establish compatibility with every
Klipper 0.12 installation or downstream fork.

These pinned source contracts informed the adapters and workflow handling.
They are inspection references, not a claim that all later versions or all
hardware using them have been validated:

| Project | Pinned reference |
| --- | --- |
| Klipper | [ee2c6fb6b00b2b5d097ef70d0868df7d74943b51](https://github.com/Klipper3d/klipper/tree/ee2c6fb6b00b2b5d097ef70d0868df7d74943b51): MCU, homing, bulk-sensor and standard accelerometer drivers. |
| Kalico | [0057b6a153170807e99080360c1ef15a2078f672](https://github.com/KalicoCrew/kalico/tree/0057b6a153170807e99080360c1ef15a2078f672): corresponding MCU/homing/bulk APIs. |
| Beacon | [3eb0134607664734ccf5cd94cb51840e1c8b27b3](https://github.com/beacon3d/beacon_klipper/blob/3eb0134607664734ccf5cd94cb51840e1c8b27b3/beacon.py): raw helper, scale and axes map. |
| Cartographer configuration | [2e7c69641fb106cf9c59d70f3987008a1117c887](https://github.com/Cartographer3D/docs/blob/2e7c69641fb106cf9c59d70f3987008a1117c887/cartographer-probe/installation-and-setup/installation/klipper-configuation.md): separate standard accelerometer object. |
| Built-in G28 wrappers | Klipper [safe_z_home](https://github.com/Klipper3d/klipper/blob/ee2c6fb6b00b2b5d097ef70d0868df7d74943b51/klippy/extras/safe_z_home.py) and [homing_override](https://github.com/Klipper3d/klipper/blob/ee2c6fb6b00b2b5d097ef70d0868df7d74943b51/klippy/extras/homing_override.py). |

See [offline testing](../TESTING.md) for the boundary between source checks,
executed synthetic tests and physical validation.

## Workflow restrictions

CALIBRATE, TEST, CHARTS, ACCURACY and BETA require:

- Cartesian or CoreXY kinematics with Z independent of XY; no dual carriage.
- Separate usable XY and Z enable lines, with the required kinematic limits APIs.
- Both XY axes initially unhomed and manually centered, with supported Z and
  nozzle/bed clearance established by the operator.
- The requested axes using this plugin's endstops, single-pass
  `homing_retract_dist: 0`, and `min_home_dist` absent/zero.
- Correct finite rail geometry, endstop direction and sufficient
  midpoint-to-endstop travel. Calibration needs more than 20 mm.
- Idle, unpaused operation, an empty motion queue and no competing homing/session.
- A verified stock G28 handler, directly or beneath exactly one recognized
  built-in `safe_z_home` or `homing_override` wrapper.

The two built-in wrappers cannot coexist. Workflows reject G28 macros,
replaced handlers, nested/unknown wrappers and an override already running its
script. The installer's `--acknowledge-homing-override` acknowledges installation
review only; it does not enable unsupported workflow handlers.

For workflow homes the plugin verifies the installed built-in handler chain and
calls stock single-axis homing directly. This skips safe Z hops and all override
G-code/forced `set_position_*` setup, including `SYNC_MOTORS`, current hooks and
Beacon Z actions. Prepare XY currents/enables and nozzle clearance separately.
The workflow itself never requests a Z move; stock fatal-homing handling can
still disable motors, so Z must remain supported.

Ordinary public `G28` continues through its configured wrapper/script. A normal
`G28 X` may still execute a safe Z hop before axis selection or a later X/Y
refusal. Workflow preflight does not intercept ordinary G28 or alter Z/probe
configuration.

## Detection and transport limits

Simple calibration and its newly saved XYZ models use a fresh resting offset
and raw three-axis magnitude. They do not use a quietness gate or an
acceleration-derived startup-motion proof. Ordinary motion or an obstruction
can cross the same threshold as an intended endstop.

Manual config thresholds and older models retain the legacy selected-axis
detector and its preparation/motion rules. Generic legacy homing rejects
unproved startup with `no_motion`; it does not accept a stationary generic
sensor as a successful already-pressed stop. Native Beacon retains a separate
legacy pressed-start path. See [detector details](DETAILS.md#detector-paths).

Generic delivery checks include freshness, timestamps, rate, error counters,
malformed/frozen/clipped data and active range. Native Beacon follows its
separate raw-stream timing policy. These checks do not make the paths identical
or prove that every delayed native packet or every transport failure is caught.

No hard end-to-end latency or physical stopping bound is provided. Python host
scheduling, USB/CAN transport, MCU dispatch, motor response, belt slip and
mechanics still matter. Optional travel policies are nominal command
calculations. Commanded/firmware step counts are not encoder feedback.

Calibration's current criterion is ten qualified validation homes with
max-minus-min travel **at most 2.00 mm**, after a separate discovery. It is not
a physical accuracy specification. A wrong obstruction, skipped steps or a
repeatedly wrong center estimate can still produce plausible results.

Plots use bounded retained data, not every raw sample. Displayed traces/peaks
can differ from the full stream; they are diagnostic views, not independent
evidence that all physical transients were captured. Existing hardware,
native-stream and plotting limitations remain relevant after repository
migration. No new hardware validation is implied by a clean installation or
a passing offline suite.
