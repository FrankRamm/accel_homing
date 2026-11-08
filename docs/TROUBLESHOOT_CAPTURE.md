# Per-home diagnostic capture

`ACCEL_HOMING_TROUBLESHOOT` records the evidence needed to investigate intermittent
impact detection, delayed stopping and a changed return before the next home.
It uses the active detector and homing settings. Ordinary G28, calibration,
ACCURACY and other existing commands keep their existing behavior.

## Update and run

Update the existing checkout while the printer is idle. An existing managed
installation can use **accel_homing -> Update** in Moonraker's frontend. The
diagnostic module also loads from the same resolved source checkout when an
older installation has not yet linked this newly added file into `extras`.
No installer rerun is required solely for that link.

For a manual source update:

```bash
cd ~/accel_homing
git switch main
git pull --ff-only origin main
```

Then restart the selected Klipper/Kalico **host service**, using its actual
service name or the frontend's service controls. A console RESTART alone does
not reload imported Python modules. For installation, dependencies or other
configuration changes, use the normal [installer procedure](INSTALLATION.md).

Follow the [moving-workflow preparation](COMMANDS.md#prepare-every-moving-workflow):
both XY axes unhomed, manually centered, Z supported, nozzle clear, suitable
motor currents and an accessible emergency stop. In the printer console:

```gcode
ACCEL_HOMING_TROUBLESHOOT AXIS=Y SAMPLES=50 CENTERED=1
```

`AXIS` accepts `X`, `Y` or `XY` (default `XY`). `SAMPLES` accepts 2 through 50
(default 50) measured homes per axis. Each axis also has one discovery home,
which is retained but excluded from repeatability statistics. `CENTERED=1`
confirms manual preparation; it does not move the toolhead to the center.

The command retains the configured speed, homing acceleration, active threshold,
sensor stream policy, midpoint returns, settle time and normal successful
terminal release. It does not select a new threshold, calibrate, save a model
or restart. `SAVE_CONFIG` is unsupported. A rejected or failed home stops this
diagnostic; it does not start ACCURACY's early-trigger recovery calibration.
`ACCEL_HOMING_CANCEL` requests cancellation when the command can be handled;
it is not an emergency stop.

## Evidence to share

The console and `ACCEL_HOMING_STATUS` report the output folder and bundle paths.
Output is under `<charts_directory>/troubleshoot/<run_id>/`, with the shareable
ZIP beside it at `<charts_directory>/troubleshoot/<run_id>.zip`. The default
charts directory is `accel_homing_charts` beside the primary printer config.
Share the complete ZIP or folder. It contains:

- Per-home records with discovery/measurement identity, outcome, active
  parameters, reference/baseline and sensor quality information.
- Packet sample counts and times, host callback times, sample age estimates,
  trigger and stop-dispatch events, faults and acquisition limits.
- Commanded stepper trigger/halt positions and logical coordinates before and
  after home/return, so each home can be associated with its predecessor.
- Retained raw accelerometer samples before plotting reduction, plus diagnostic
  plots of travel, packet timing, impact windows and successive-home behavior.
- Relevant effective configuration, runtime/firmware and source identity,
  available MCU statistics and a bounded log excerpt. Unavailable fields are
  reported as unavailable instead of inferred.
- Capture limits, dropped/omitted data and export/render errors. An incomplete
  run remains labelled incomplete; a plotting failure does not discard JSON
  and raw evidence already written.

The main files are `manifest.json`, `packets.jsonl.gz`, one `home-NNN-x.csv.gz`
or `home-NNN-y.csv.gz` and PNG per attempted home, `summary.png`, `README.txt`
and `renderer.log`. When the host log is accessible, the bundle also includes
`klippy-run.log` and selected startup metadata in `klippy-startup.txt`.

Collection is bounded to avoid unlimited memory use. The manifest records
retention limits and omissions; "raw" means the retained samples have not
been reduced to a plotting envelope, not that hardware delivered every sample.
Files are generated after the motion sequence and cleanup. A process crash or
power loss before export can prevent creation of the bundle.

## Reading the timing and plots

Sensor print time and host monotonic time are different clock domains. The
metadata labels each domain and any estimated conversion. For configured
accelerometers, callback entry is when the plugin receives a processed batch;
it is not the hardware interrupt or original CAN/USB packet arrival. A host
timestamp after the stop call or at home completion is not an MCU halt timestamp.

Trigger travel, halt travel and returns come from commanded steps and logical
coordinates. They do not independently verify carriage position, a mechanical
stop, belt deflection or skipped steps. A completed run does not certify physical
homing accuracy, and outliers remain in the report.

The diagnostic observes existing behavior but adds bounded collection overhead.
`callback_duration_s` covers processing before the recorder; it excludes the
recorder's own cost and is not total callback occupancy or transport latency.
Use that distinction and the omission fields when interpreting timing. Disk
writes, hashing, compression and plot rendering are kept outside the measured
motion sequence. To compare settings, preserve one bundle for each run and
change one operating setting at a time under operator supervision.
