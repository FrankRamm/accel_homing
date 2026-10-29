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
| Legacy MISC ownership found | Preserve thresholds/models/config/backups and successfully uninstall with the old installation's uninstaller before standalone installation. See [migration](INSTALLATION.md#migrate-from-the-misc-plugin). |
| Shared checkout/consumer conflict | Keep the source and every affected instance intact. Do not replace shared links while another instance uses them; use a supported dedicated installation. |
| Service restart cannot be verified | Review the exact unit, checkout, config and socket. For an unsupported manager use `--no-restart`, then activate the selected processes manually while idle. |
| Install succeeded but old code seems loaded | A full Klipper host-process restart is needed after source/link changes; console RESTART alone does not reload imported modules. |
| Update entry missing after manual edit | Ensure one `[update_manager accel_homing]` across includes, correct standalone path/origin/main, and restart the matching Moonraker process. |
| Update controls disabled or version unknown | Inspect Moonraker logs and its repository/tag/version status. See [official Git-repo configuration](https://moonraker.readthedocs.io/en/latest/configuration/#git-repo-configuration). Do not point it at the historical MISC subtree. |
| Plot dependency/render preflight failed | Rerun the installer with the exact selected Klipper interpreter if needed. Check connectivity, supported binary wheels and output permissions; global package pins are not a repair shortcut. |

Do not delete `~/accel_homing` while extras, restored/shared links or a retained
manual Moonraker updater entry still reference it. The uninstaller removes
installer-created registration, not a pre-existing manual entry it does not own.

## Configuration and workflow admission

| Symptom | Check and next action |
| --- | --- |
| `CENTERED=1` required | Physically center XY, leave both axes unhomed, establish Z/nozzle clearance and suitable XY currents, then issue the intended command once with CENTERED=1. |
| XY already homed | Follow your printer's appropriate procedure to release/unhome XY while supporting Z, then manually center. Do not assume a generic motor-off command affects only XY. |
| G28 handler/wrapper rejected | Stock G28 or one verified built-in safe_z_home/homing_override wrapper is required. Macros and unknown/nested handlers remain unsupported; the installation acknowledgement flag does not enable them. |
| Invalid geometry/insufficient travel | Correct the actual rail limits, endstop position/direction and midpoint distance. Calibration needs more than 20 mm from configured midpoint to stop. |
| Threshold exceeds range/headroom | Check actual configured/active sensor range. Use a justified value within the 80% software policy. Native clients can change the stream range. |
| Saved model mismatch or unexpected active threshold | Inspect selected sensor/model and manual X/Y/shared threshold precedence. Comment manual values to use a calibrated model; do not edit generated model metadata. |
| Printing/paused/busy or competing sensor client | Finish or cancel the competing operation normally and wait for a genuinely idle, stationary machine. Do not bypass the check. |

## Data and homing faults

Missing, stale, invalid, nonmonotonic, frozen or saturated data requires fixing
the sensor/transport/configuration issue. Inspect `BEACON_ACCEL_HOMING_STATUS`
for retained preparation/timing details. A reported `ok` at the end of one
reference does not prove sustained stream health; inspect rejection counts,
sample/batch ages and failure context.

A legacy `no_motion` means the required conservative proof was unavailable
within its deadline. It does not establish that the motor physically stayed
still or that raising a threshold/freshness limit is the correct fix. New
simple vector models use a different detector and omit that integrated proof.
Do not compare thresholds between those metrics as if they were identical.

Unexpected early impact can come from motion noise, an obstruction, binding or
true stop contact from a wrong starting point. Normal calibration reverses the
current recorded halt displacement and retries only within its documented
bounds. Above-band travel, missing impact, a sensor fault or failed return
aborts; inspect the machine and manually prepare XY before a new command.
Commanded return completion is not physical arrival feedback.

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

An incomplete PNG preserves available diagnostics; it is not a successful
measurement. PNG export failures, temporary-capture cleanup faults and
unsupported output filesystem operations are reported separately. Preserve
those errors and any final partial PNG.

`ACCEL_HOMING_CANCEL` is not an immediate emergency stop. Use the accessible
emergency stop for unexpected active movement. A shutdown/homing fault can
disable motors, including Z. Re-establish support and clearance before any
later operation.

## Uninstall or rollback conflict

Keep current configuration, old/new sources and the complete
`.accel_homing_backups/` history. The reported manifest identifies original
and installed states. Reconcile only the managed conflict while retaining
unrelated later edits and other saved models.

Missing original targets, ambiguous shared links or incomplete old baselines
cannot be reconstructed safely from a guessed directory. Recover the original
source/baseline before retrying. Do not delete links/history to silence an
ownership error. See [uninstall and recovery](INSTALLATION.md#uninstall).

## Reporting a problem

Include the plugin commit (`git -C ~/accel_homing rev-parse HEAD`), host/fork
version, selected sensor and mounting, relevant configuration with secrets
removed, exact command/error and STATUS output. For BETA, include the same
command's complete `Beta diagnostic` output and PNG. State what you physically
observed and whether a run was cancelled or interrupted; firmware step counts
alone do not prove carriage movement.
