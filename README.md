# Accelerometer Homing

Use a configured accelerometer as an X/Y impact endstop in Klipper or Kalico.
This experimental plugin deliberately contacts mechanical stops. Supervise
initial homing and calibration with an accessible emergency stop.

## Install

This `dev` branch includes the experimental
[center calibration workflow](docs/CENTER.md) and an optional
[notch filter](docs/NOTCH.md), disabled by default. It is separate from `main`.

Run in the Linux printer host's shell as the existing Klipper service user,
with the printer idle:

```bash
cd ~
git clone --branch dev https://github.com/FrankRamm/accel_homing.git
~/accel_homing/install.sh --branch dev
```

The installer backs up its changes, configures X/Y homing and Moonraker
updates, and restarts the verified services. It may ask for your
Linux sudo password. It does not home the printer.

Read the [first calibration instructions](docs/SIMPLE_CALIBRATION.md) before
moving the printer. Existing MISC installations must follow the
[migration instructions](docs/INSTALLATION.md#migrate-from-the-misc-plugin) first.

For an existing clean native `~/accel_homing` installation, select `dev` explicitly:

```bash
git -C ~/accel_homing fetch origin
git -C ~/accel_homing switch dev
git -C ~/accel_homing pull --ff-only
~/accel_homing/install.sh --branch dev
```

With only `origin/dev` present, `git switch dev` creates its tracking branch.
Keep the checkout, links and backup history in place. The installer updates
its owned Moonraker registration to follow `dev`; it refuses local changes or
a conflicting manual registration. See [branch switching and recovery](docs/INSTALLATION.md#test-the-dev-branch)
before returning to the older `main` installer.

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
primary_branch: dev
is_system_service: False
managed_services: klipper
```

Add the base `[update_manager]` section once if absent. After manual edits,
restart the matching Moonraker service while idle. `~` is Moonraker's service
user's home; use the printed absolute path if needed. A pre-existing manual
entry survives uninstall.
`channel: dev` selects Moonraker's Git update mode; `primary_branch: dev`
selects this repository's branch. For a `main` installation use `primary_branch: main`.
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
| `ACCEL_HOMING_TRAVEL` | Dev: open/reopen the guided X/Y endpoint and bed-zero wizard. Configured-speed searches, slow dedicated nozzle-alignment jogs, review and optional combined save. See [travel/origin wizard](docs/TRAVEL.md). |
| `ACCEL_HOMING_CALIBRATE AXIS=X CENTERED=1` | Calibrate X; use `AXIS=Y` for Y. Measures fresh noise, then qualifies a threshold. |
| `ACCEL_HOMING_CALIBRATE_CENTER AXIS=ALL CENTERED=1` | Dev: one excluded initial home, then ten alternating direct end-to-end traverses per selected axis, with no backoff/retract moves. Reports geometry; `SAVE_CONFIG=1` explicitly requests saving. See [center calibration](docs/CENTER.md). |
| `ACCEL_HOMING_CALIBRATE_NOISE` | Measure three stationary one-second noise windows and export a PNG; no movement or threshold change. |
| `ACCEL_HOMING_TEST CENTERED=1` | Test X then Y with active thresholds; both axes must use this plugin. |
| `ACCEL_HOMING_CHARTS_GENERATE AXIS=X CENTERED=1` | Home/chart X; use `AXIS=Y` or omit AXIS for both. |
| `ACCEL_HOMING_ACCURACY AXIS=X SAMPLES=10 CENTERED=1` | Measure repeated commanded-travel variation and save an accuracy PNG; X, Y or XY, 2–50 samples per axis. |
| `ACCEL_HOMING_TROUBLESHOOT AXIS=Y SAMPLES=50 CENTERED=1` | Record repeated homes, packet/stop/return timing, raw samples and diagnostic plots in a shareable bundle; fixed active settings, no recalibration or save. |
| `ACCEL_HOMING_CANCEL` | Cancel workflow work when handled and restore temporary settings; **not an emergency stop**. |
| `SET_ACCEL_HOMING AXIS=X THRESHOLD=1` | Set X's threshold in g until restart; use `AXIS=Y` for Y. Does not save a model. |

**ACCEL_HOMING_CALIBRATE saves the selected axis and restarts Klipper by default.** It adds
0.5 g once to the candidate after ten checks pass; those homes test the candidate,
not the higher saved value. The other axis is unchanged. Prepare
XY again after restart before calibrating the other axis. Explicit X/Y/shared
manual thresholds override ordinary saved models; comment them out and issue
`RESTART` while idle to use calibration. See
[configuration](docs/CONFIGURATION.md#thresholds-and-saved-models).

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
[Notch filter (dev)](docs/NOTCH.md) ·
[Console commands](docs/COMMANDS.md) ·
[Calibration](docs/SIMPLE_CALIBRATION.md) ·
[Center calibration (dev)](docs/CENTER.md) ·
[Travel and bed-origin wizard (dev)](docs/TRAVEL.md) ·
[Accuracy](docs/ACCURACY.md) ·
[Per-home diagnostic capture](docs/TROUBLESHOOT_CAPTURE.md) ·
[Plots](docs/PLOTS.md) ·
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
their time, patience and continued testing. Your reports make this work possible. This is a Vibecoded app, still in experimental release.
