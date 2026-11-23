# Troubleshooting

Start with the exact console/installer error, the selected instance and
`ACCEL_HOMING_STATUS`. Keep the printer idle while fixing installation or
configuration. A failed workflow does not establish that the toolhead remains
centered or that SAVE_CONFIG succeeded.

## Installation and updater

| Symptom | Check and next action |
| --- | --- |
| More than one Klipper instance/sensor found | Use the reported full paths and `--config`, `--service`, `--klippy-socket`, `--klipper-path` and/or `--accel-chip`. Explicit selections must describe the same instance. |
| Source/origin/branch/upstream rejected | Use the complete clean `~/accel_homing` native clone with canonical HTTPS origin, `main` and `origin/main`. Review local changes; do not force-reset them away. |
| Old backup section changes or changed Moonraker lines block installation | Update to the current standalone installer and run it directly. Installation backups are deprecated and ignored; existing backup folders remain untouched. The old MISC uninstaller is not required. See [migration](INSTALLATION.md#migrate-from-the-misc-plugin). |
| Shared checkout/consumer conflict | Keep the source and every affected instance intact. Do not replace shared links while another instance uses them; use a supported dedicated installation. |
| Service restart cannot be verified | Review the exact unit, checkout, config and socket. For an unsupported manager use `--no-restart`, then activate the selected processes manually while idle. |
| Install succeeded but old code seems loaded | A full Klipper host-process restart is needed after source/link changes; console RESTART alone does not reload imported modules. |
| Update entry missing after manual edit | Ensure one `[update_manager accel_homing]` across includes, correct standalone path/origin/main, and restart the matching Moonraker process. |
| Update controls disabled or version unknown | Inspect Moonraker logs and its repository/tag/version status. See [official Git-repo configuration](https://moonraker.readthedocs.io/en/latest/configuration/#git-repo-configuration). Do not point it at the historical MISC subtree. |
| Plot dependency/render preflight failed | Rerun the installer with the exact selected Klipper interpreter if needed. Check connectivity, supported binary wheels and output permissions; global package pins are not a repair shortcut. |

Do not delete `~/accel_homing` while extras links or a Moonraker updater entry
still reference it. Uninstall removes the matching current plugin registration
and preserves unrelated updater sections.

### Import failure after the first troubleshooting update

Revision `cdfd898` eagerly imported the newly added diagnostic module. A native
Moonraker Git update could leave that module without an `extras` link. Nested
import fallbacks then reported `No module named 'accel_homing_workflows'`, even
when the workflow link existed. Later `main` revisions load diagnostics only
when requested and can load the file from the same resolved source checkout.

Refresh the updater and update `accel_homing` to current `main`, then ensure the
selected Klipper/Kalico host service restarts. Console `RESTART` or
`FIRMWARE_RESTART` alone does not reload imported Python code. This repair does
not require deleting configuration, recalibrating or reinstalling the plugin.

If the complete source checkout or an original module link is actually missing,
preserve the full traceback and inspect that separate installation problem.
The installer refuses writes while a running Klipper reports an error state;
`--no-restart` does not bypass that check. Do not disable the guard to repair a
missing file. See [service activation and instance selection](INSTALLATION.md).

## Configuration and workflow admission

| Symptom | Check and next action |
| --- | --- |
| `CENTERED=1` required | Physically center XY, leave both axes unhomed, establish Z/nozzle clearance and suitable XY currents, then issue the intended command once with CENTERED=1. |
| XY already homed | Follow your printer's appropriate procedure to release/unhome XY while supporting Z, then manually center. Do not assume a generic motor-off command affects only XY. |
| Custom G28 setup did not run | Workflows call native selected-axis homing and bypass public G28 macros/wrappers. Prepare XY currents/enables and nozzle clearance independently; ordinary G28 keeps its configured sequence. |
| Invalid geometry/insufficient travel | Correct the actual rail limits, endstop position/direction and midpoint distance. Calibration needs more than 20 mm from configured midpoint to stop. |
| Threshold exceeds range/headroom | Check actual configured/active sensor range. Use a justified value within the 80% software policy. Native clients can change the stream range. |
| Saved model mismatch or unexpected active threshold | Inspect selected sensor/model and manual X/Y/shared threshold precedence. Comment manual values to use a calibrated model; do not edit generated model metadata. |
| Saved model is stale after changing `axes_map`, geometry or homing speed | Klipper still starts, but homing that axis is refused (`does not match current ...`) until it is recalibrated with `ACCEL_HOMING_CALIBRATE AXIS=X CENTERED=1`, or an explicit `x_threshold` is set in the regular `[accel_homing]` section. A threshold that calibration wrote to the SAVE_CONFIG block does not count as an override. After an `axes_map` or sensor change, a model from automatic/mapped calibration must be recalibrated, because a manual threshold does not replace the saved axis mapping. The stale axes are listed as `stale_model_axes` in `ACCEL_HOMING_STATUS`. |
| Printing/paused/busy or competing sensor client | Finish or cancel the competing operation normally and wait for a genuinely idle, stationary machine. Do not bypass the check. |

## Data and homing faults

Missing valid deliveries, future/invalid/nonmonotonic timestamps, frozen or
non-finite data requires fixing
the sensor/transport/configuration issue. Inspect `BEACON_ACCEL_HOMING_STATUS`
for retained preparation/timing details. A reported `ok` at the end of one
reference does not prove sustained stream health; inspect rejection counts,
sample/batch ages and failure context.

For generic sensors, an old current-window timestamp alone is accepted,
including 80 ms or more of delivery delay. `max_sample_age` is a nominal
planning input and no longer caps that age. Check monotonic receipt intervals
against the independent `stream_timeout`, future timestamps, measurement
cutoffs and continuity instead. A pre-arm impact still cannot hit, and
pre-arm-only callbacks do not renew armed liveness. Native Beacon retains its
separate age checks. Actual silence can still abort a delayed stream.

Recognized finite sensor rails and native Beacon clipping markers are accepted
as bounded RAW observations, with range provenance and unknown excess
amplitude. Beacon markers are endpoint estimates with unknown true sign/peak.
They do not by themselves establish invalid transport or a measured impact
peak. Threshold headroom and other acquisition/data-fault safeguards still
apply; inspect capture and resting-reference notes separately.

A legacy `no_motion` means the required conservative proof was unavailable
within its deadline. It does not establish that the motor physically stayed
still or that raising a threshold or receipt timeout is the correct fix. New
simple mapped-axis models use a different detector and omit that integrated proof during homing.
Do not compare thresholds between those metrics as if they were identical.

Unexpected early impact can come from motion noise, an obstruction, binding or
true stop contact from a wrong starting point. Normal calibration reverses the
current recorded halt displacement and retries only within its documented
bounds. Above-band travel, missing impact, a sensor fault or failed return
aborts; inspect the machine and manually prepare XY before a new command.
Commanded return completion is not physical arrival feedback.

### `homed after ... mm, less than min_home_dist ... times`

The full error is `accel_homing: <AXIS> homed after <t1>/<t2>/<t3> mm, less than
min_home_dist <N> mm, 3 times. The stop was not confirmed, homing stopped`.

The min-home-distance check fired on three consecutive attempts before the
physical stop was reached. Each short home backed off and homed again, and the
command failed with homing stopped. The trigger repeatedly fired before the
physical stop, which points to stepper desync, a loose belt, or a motor that
snapped to a stop early.

Checks:

- Belts and motor synchronization: look for slipping, loose belts, or a stepper
  that skips steps.
- Threshold too low: a low accelerometer threshold lets noise or vibration
  trigger an early stop. Raise it only after the belts and motors check out.
- `min_home_dist` (or `<axis>_min_home_dist`) larger than the normal distance
  from the back-off position to the physical stop: the trigger fires on a valid
  home. Lower the value to match the real travel.

### Automatic mapping interruption

Automatic mapping runs only when the selected sensor has no explicit
`accel_axes_map` (Beacon) or `axes_map` (other accelerometers). With an explicit
setting, calibration uses the already mapped X/Y channels and skips these
strokes. Fresh noise and threshold qualification still run and can report their
own acquisition faults. Only configure a map that matches the sensor's mounting;
the configured path trusts that orientation rather than measuring it.

For example, a Beacon mounted so positive machine X corresponds to positive
sensor Y, positive machine Y to negative sensor X, and machine Z to sensor Z
uses `accel_axes_map: y, -x, z` in its `[beacon]` section. After changing
the map, reload the configuration and calibrate X and Y for that mounting.
Thresholds are specific to the printer and axis.

The earlier message `automatic axis mapping was interrupted or its sensor
context changed` also covered sensor acquisition faults; it did not establish
that the configured axes map had changed. A retained fault from that failed,
closed stream could also reject the next calibration before fresh acquisition
started.

Mapping now reports the phase and specific interruption or sensor cause.
After an abort, collect the console output and `ACCEL_HOMING_STATUS` before
starting another command. Its `last_result.axis_mapping_diagnostics` retains
the first failure, including available sample-timing and window-fault details
before capture cleanup. A new calibration may start a fresh capture after a
closed-stream fault, but faults in its own reference or moving capture still
abort it. There is no automatic retry of a mapping movement. Manually prepare
and center XY again before retrying; an aborted stroke does not establish that
the toolhead returned to the center.

On Beacon, a small change in the sample period estimated from consecutive
packets can occur even when the next packet arrives on time. Calibration
allows a period change of up to 25% of the current packet's sample period,
with a 3 us minimum tolerance. A gap longer than five current sample periods
plus 3 us still invalidates the window. For a `sample_gap`, compare `gap_s`
with `gap_limit_s` and the two sample periods with `interval_change_limit_s`
when those fields are present.

If both strokes were captured but inference rejects the mapping, for example
`Mapping cruise response is unstable`, calibration exports an evidence bundle
after successful cleanup. The console prints its ZIP path, and STATUS retains
it under `last_result.axis_mapping_diagnostics.paths`. Share the ZIP from the
`axis_mapping_X_*` or `axis_mapping_Y_*` folder inside the charts directory.
No separate diagnostic command or extra movement is needed.

The bundle contains both full signed configured-stream XYZ captures in g,
stationary baselines, actual planner segments, sensor context and available
inference metrics in `mapping.json`, plus `mapping.png`. These are processed
sensor-stream samples, not native ADC counts. The plot shows acceleration and
an integrated velocity estimate; it does not verify physical speed or position.
JSON and the ZIP remain available if plotting fails; a ZIP failure leaves the
JSON path available. A cleanup failure skips export and reports that failure.

Automatic mapping checks the steady-speed part of each stroke. It averages
the signed XYZ velocity estimates in windows of up to 20 ms. The median of
those vectors must have a magnitude between 50% and 150% of commanded speed.
Both strokes must identify the same sensor channel and sign once their
opposite travel directions are accounted for.

Version 3 inference reports distinguish two ways a stroke can fail.

- `raw_coherent_cruise_time_fraction` measures how often the individual
  velocity estimates point along the selected channel with the expected sign.
  That channel must supply at least 85% of the vector magnitude for at least
  95% of cruise time. A low fraction points to conflicting direction evidence.
- `pooled_stable_cruise_fraction` measures how much cruise time the window
  averages spend aligned with that signed channel and within the speed band.
  The forward component must be 50% to 150% of the median speed for a window
  to count. At least 95% of cruise time must qualify, and every window must
  show at least 25% of the median speed in the forward direction. Compare
  `minimum_pooled_projection_mm_s`
  with `minimum_local_motion_mm_s` to identify a local slowdown or reversal.

Earlier reports counted individual samples meeting both direction and speed
criteria. That count alone cannot distinguish speed ripple from an ambiguous
direction. The signed captures in `mapping.json` allow the same stroke to be
evaluated with the updated checks.

The requirement that both X and Y be unhomed still applies. Running `G28`
before calibration homes those axes and therefore causes that separate
precondition error. The mapping routine only moves the selected X or Y axis.

### Noise on another accelerometer axis

A Y move can excite several configured sensor channels through the toolhead
and mount. New calibration uses an explicit sensor map or, if absent, learns
physical Y from gentle motion in both directions. It does not select the largest
endstop-impact peak. Subsequent Y homing compares only that mapped component
with its threshold. X does the same
with its own mapping. Other channels remain in diagnostics but cannot trigger
the mapped-axis detector. A large channel labelled X in a raw stream does not,
by itself, establish a physical X response or an `axes_map` error.

If mapping is ambiguous, check mounting, sensor configuration and the motion
response before recalibrating. Changing the sensor or axes map invalidates its
saved context. Older XYZ-magnitude models require a fresh calibration; do not
copy their threshold into a mapped model.

Compare raw XYZ during startup, steady travel and the impact in a complete
troubleshooting bundle from the affected axis. Stationary noise alone cannot
characterize vibration while moving. Keep speed, acceleration, sensor settings
and threshold recorded with the comparison. A filter or component weighting
changes the detector: it can attenuate an impact or delay the threshold crossing,
and the existing calibrated threshold would need qualification again. There is
currently no configurable filter for the ordinary mapped-axis detector.

## Accuracy recovery and save errors

An early ACCURACY result does not always authorize an automatic return or
calibration. Current capture/halt evidence, unchanged motor/position context
and the confirmed center must survive through final PNG publication and
temporary cleanup. A failure at any of those stages stops recovery.

When recovery proceeds, its first candidate is the actual failed threshold
plus 0.25 g. Subsequent candidate changes use configured calibration_step.
It must complete normal calibration before saving. If native SAVE_CONFIG
reports a duplicate/include/write conflict, the result is `save_failed`;
persistence and restart are not confirmed. Resolve that exact save conflict
and preserve current files instead of editing the DO NOT EDIT block manually.

After a successful save/restart, prepare XY again and rerun the printed
accuracy command. Recovery never reruns accuracy automatically.

## Plots and cancellation

No inline preview does not necessarily mean no PNG: read the saved path.
Use formatted Mainsail output or the frontend file browser. Remote/header-only
authentication may not authorize a browser image GET. See [plot access](PLOTS.md).

Standard chart workflows save no partial PNG on failure, rejection or cancellation. Preserve the
console/status diagnostics, including the failed window, retries and cleanup
results. PNG export failures and unsupported output filesystem operations are
reported separately. An optional preview/link error after a successful saved
measurement is reported as `display_error`; its valid PNG remains available.

`ACCEL_HOMING_CANCEL` is not an immediate emergency stop. Use the accessible
emergency stop for unexpected active movement. A shutdown/homing fault can
disable motors, including Z. Re-establish support and clearance before any
later operation.

## Uninstall or installation conflict

Installation and uninstall use current files, not backup history. There is no
need to repair or delete `.accel_homing_backups/` to proceed. Update the checkout
if an older installer still reports an installation-backup validation error.
Historical `--rollback` is no longer supported and exits without writes.

A current conflict can still indicate an ambiguous instance, a foreign module
link or file, a shared consumer, or a file edited while the operation was being
prepared. Inspect the specific path and resolve that current conflict before
retrying. Do not delete source checkouts to silence a link-ownership error.

Uninstall removes plugin virtual endstop options without restoring original
pins or homing settings. Configure physical X/Y endstops and review their
settings before restarting Klipper. See [uninstall](INSTALLATION.md#uninstall).

## Reporting a problem

For intermittent homing or repeatability problems, use the supervised
[`ACCEL_HOMING_TROUBLESHOOT`](TROUBLESHOOT_CAPTURE.md) command after the usual
manual preparation. It collects per-home timing, position/return metadata,
raw acceleration, environment information and graphs into one bundle. Preserve
the complete bundle even if a home fails or a graph cannot render. Diagnostic
collection does not recalibrate or save thresholds.

Include the plugin commit (`git -C ~/accel_homing rev-parse HEAD`), host/fork
version, selected sensor and mounting, relevant configuration with secrets
removed, exact command/error and STATUS output. For BETA, include the same
command's complete `Beta diagnostic` output and PNG. State what you physically
observed and whether a run was cancelled or interrupted; firmware step counts
alone do not prove carriage movement.
