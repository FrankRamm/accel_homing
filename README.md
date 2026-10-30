# Accelerometer Homing

**Development / experimental: `codex/accuracy-filters`.** This branch adds an
opt-in causal `FILTERED` detector, explicit RAW controls and experimental
`HYBRID` position timing. HYBRID keeps FILTERED confirmation and uses an
associated RAW XYZ threshold crossing as the reported position timestamp when
that crossing is unambiguous. Software tests and
synthetic plots do not establish a physical improvement. Existing RAW models
remain RAW until an explicit FILTERED calibration succeeds and is saved.
Use the [filter comparison procedure](docs/FILTERS.md) to collect a RAW baseline,
calibrate FILTERED, inspect the same-trace diagnostic and compare independent
completed accuracy runs across the calibration restart.
Existing compatible RAW and FILTERED profiles can also be used directly for
[independent 30-sample RAW, pure FILTERED and HYBRID runs](docs/ACCURACY.md#compare-30-raw-pure-filtered-and-hybrid-homes),
without another calibration.

Use a configured accelerometer as an X/Y impact endstop in Klipper or Kalico.
This experimental plugin deliberately contacts mechanical stops. Supervise
initial homing and calibration with an accessible emergency stop.

## Install

Run in the Linux printer host's shell as the existing Klipper service user,
with the printer idle:

```bash
cd ~
git clone --branch codex/accuracy-filters https://github.com/FrankRamm/accel_homing.git
~/accel_homing/install.sh --branch codex/accuracy-filters
```

This branch requires the explicit development branch argument. The installer
backs up its changes, configures X/Y homing and Moonraker
updates, and restarts the verified services. It may ask for your
Linux sudo password. It does not home the printer.

Read the [first calibration instructions](docs/SIMPLE_CALIBRATION.md) before
moving the printer. Existing MISC installations must follow the
[migration instructions](docs/INSTALLATION.md#migrate-from-the-misc-plugin) first.

For later updates, use **accel_homing → Update** in Mainsail or Fluidd while idle.

## Moonraker configuration

The default installer already adds this entry. **Do not add a duplicate** in
`moonraker.conf` or its includes. For manual setup only, add this if missing:

```ini
[update_manager accel_homing]
type: git_repo
channel: dev
path: ~/accel_homing
origin: https://github.com/FrankRamm/accel_homing.git
primary_branch: codex/accuracy-filters
is_system_service: False
managed_services: klipper
```

Add the base `[update_manager]` section once if absent. After manual edits,
restart the matching Moonraker service while idle. `~` is Moonraker's service
user's home; use the printed absolute path if needed. A pre-existing manual
entry survives uninstall.
[Update details](docs/INSTALLATION.md#moonraker-updates) ·
[Moonraker reference](https://moonraker.readthedocs.io/en/latest/configuration/#git-repo-configuration).

## Printer console commands

Enter these in the **Mainsail/Fluidd printer console**, not the host shell.
Before every moving workflow, keep the printer idle and unpaused, leave
**both X and Y unhomed**, physically center XY by hand, support Z and ensure
nozzle/bed clearance. Prepare suitable XY currents/enables and supervise with
an accessible emergency stop. **`CENTERED=1` authorizes movement; it does not
center the toolhead.** Workflows skip safe Z hops and G28 override scripts.

| Command | What it does / how to use it |
| --- | --- |
| `ACCEL_HOMING_STATUS` | Show settings, model, workflow progress, results and PNG paths. |
| `ACCEL_HOMING_CALIBRATE AXIS=X CENTERED=1` | Calibrate X; use `AXIS=Y` for Y. Measures fresh noise, then qualifies a threshold. |
| `ACCEL_HOMING_CALIBRATE_NOISE` | Measure three stationary one-second noise windows and export a PNG; no movement or threshold change. |
| `ACCEL_HOMING_TEST CENTERED=1` | Test X then Y with active thresholds; both axes must use this plugin. |
| `ACCEL_HOMING_CHARTS_GENERATE AXIS=X CENTERED=1` | Home/chart X; use `AXIS=Y` or omit AXIS for both. |
| `ACCEL_HOMING_FILTER_CHARTS AXIS=X CENTERED=1` | Experimental raw/causal-filter comparison of the same valid home; shows actual control, shadow replay, thresholds, noise and censored timing. |
| `ACCEL_HOMING_ACCURACY AXIS=X SAMPLES=10 CENTERED=1` | Measure repeated commanded-travel variation and save an accuracy PNG; X, Y or XY, 2–50 samples per axis. |
| `ACCEL_HOMING_BETA_CALIBRATE AXIS=X CENTERED=1` | Experimental moving-noise calibration proposal; use `AXIS=Y` for Y. Beta PNG and diagnostics only, no saved threshold/model or restart. |
| `ACCEL_HOMING_CANCEL` | Cancel workflow work when handled and restore temporary settings; **not an emergency stop**. |
| `SET_ACCEL_HOMING AXIS=X THRESHOLD=1` | Set X's threshold in g until restart; use `AXIS=Y` for Y. Does not save a model. |
| `BEACON_ACCEL_HOMING_STATUS` | Show detector/preparation diagnostics; retained legacy command. |
| `SET_BEACON_ACCEL_HOMING AXIS=X THRESHOLD=1` | Legacy alias of `SET_ACCEL_HOMING`. |

**CALIBRATE saves the selected axis and restarts Klipper by default.** It adds
0.5 g once to the candidate after ten checks pass; those homes test the candidate,
not the higher saved value. The other axis is unchanged. Prepare
XY again after restart before calibrating the other axis. Explicit X/Y/shared
manual thresholds override ordinary saved models; comment them out and issue
`RESTART` while idle to use calibration. See
[configuration](docs/CONFIGURATION.md#thresholds-and-saved-models).

An early ACCURACY trigger with valid current capture/halt/center evidence can
reverse its recorded commanded halt travel, report the failure in the console, then start
normal calibration at the actual failed threshold **plus 0.25 g**. Successful
recovery calibration saves and restarts Klipper. Prepare XY again and rerun the
exact accuracy command printed in the console; it does not rerun automatically.
[Recovery details](docs/ACCURACY.md#early-trigger-automatic-recalibration).

Charts are saved only after the requested workflow and its required cleanup/save
succeed. Failed, rejected or cancelled runs keep console diagnostics without
saving a partial PNG, including temporary per-home images.

On this development branch, CALIBRATE, TEST, CHARTS, FILTER_CHARTS and ACCURACY
accept `DETECTOR=RAW|FILTERED`. Explicit mode selection uses that mode's own
saved threshold. FILTERED must be calibrated separately; a RAW threshold is not
silently reused. `ACCURACY ... BASELINE=<completed_raw_run_id>` compares two
actual independent completed runs. The compact baseline receipt is inside the
successful PNG and survives restart; no JSON sidecar is required. Reprepare XY
before **every** moving command. See [the exact sequence](docs/FILTERS.md#operator-sequence).

TEST, ordinary CHARTS and ACCURACY also accept explicit `DETECTOR=HYBRID`.
This requires the native Beacon packet path and both compatible saved profiles
before movement. It accepts no temporary threshold override, preserves the
selected model and uses the
saved FILTERED profile to confirm a stop. Its position timestamp uses the
associated RAW crossing or an explicitly reported FILTERED fallback. The
accuracy PNG and JSON distinguish HYBRID from pure FILTERED and count selected
RAW positions and fallback reasons. HYBRID early-quality failure aborts for
manual inspection/recentering; it does not start automatic return or calibration.
See [HYBRID limits and diagnostics](docs/FILTERS.md#hybrid-position-timing).

Read the [command reference](docs/COMMANDS.md) for options and restrictions.
After physically validating both axes, use the printer's normal homing controls.

## Uninstall

Run while idle:

```bash
~/accel_homing/uninstall.sh
```

After uninstallation succeeds, restart the services listed in its output.
Only after those restarts, and after checking that no remaining plugin links
or manual updater entries need this checkout, remove the source if desired:

```bash
rm -rf ~/accel_homing
```

Keep the printed backup paths if you may need to undo the uninstall.
[Uninstall and recovery details](docs/INSTALLATION.md#uninstall).

## Documentation

[Installation and migration](docs/INSTALLATION.md) ·
[Supported sensors and limits](docs/COMPATIBILITY.md) ·
[Configuration](docs/CONFIGURATION.md) ·
[Console commands](docs/COMMANDS.md) ·
[Calibration](docs/SIMPLE_CALIBRATION.md) ·
[Accuracy](docs/ACCURACY.md) ·
[Plots](docs/PLOTS.md) ·
[Experimental filters and A/B comparison](docs/FILTERS.md) ·
[Beta experiment](docs/BETA.md) ·
[Troubleshooting](docs/TROUBLESHOOTING.md) ·
[Technical reference](docs/DETAILS.md) ·
[Offline tests](TESTING.md).

## Credits and thanks

**[CloakedWayne](https://github.com/CloakedWayne) created the original Beacon
Accelerometer Homing plugin and its impact-detection algorithm** in
[Monolith3D/MISC](https://github.com/Monolith3D/MISC/tree/main/Beacon_Accel_Homing).
This project builds on that work and retains its GPLv3 license and attribution.

Thank you to everyone who has tested the plugin, shared results and helped
improve it. Special thanks to **Ingstromentalist**, **Acke**, and **mc-red** for
their time, patience and continued testing. Your reports make this work possible.

Maintained by [FrankRamm](https://github.com/FrankRamm).
[Full attribution and provenance](NOTICE) · [GPLv3](LICENSE).
