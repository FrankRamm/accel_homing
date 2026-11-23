# Accelerometer Homing

Use a configured accelerometer as an X/Y impact endstop in Klipper or Kalico.
This experimental plugin deliberately contacts mechanical stops. Supervise
initial homing and calibration with an accessible emergency stop.

## Install

To test this development branch on an existing printer, follow
[Switch an existing tester installation to dev](docs/INSTALLATION.md#switch-an-existing-tester-installation-to-dev).
The install block below selects the stable branch.

Run in the Linux printer host's shell as the existing Klipper service user,
with the printer idle:

```bash
(
  set -e
  url=https://github.com/FrankRamm/accel_homing.git
  repo="$HOME/accel_homing"
  if [ -e "$repo/.git" ]; then
    cd "$repo"
    if [ "$(git remote get-url origin)" != "$url" ] || [ -n "$(git status --porcelain)" ]; then
      echo "Expected a clean accel_homing checkout with origin $url." >&2
      exit 1
    fi
    git fetch origin refs/heads/main:refs/remotes/origin/main
    git switch main
    git merge --ff-only origin/main
    if [ "$(git rev-parse HEAD)" != "$(git rev-parse origin/main)" ]; then
      echo "Local main has unpublished commits. Review them before installing." >&2
      exit 1
    fi
    git branch --set-upstream-to=origin/main main
  else
    git clone --branch main "$url" "$repo"
  fi
  bash "$repo/install.sh"
)
```

The installer configures X/Y homing and Moonraker updates using the current
configuration, without creating or checking installation backups. It restarts
the verified services and may ask for your Linux sudo password. It does not
home the printer.

Read the [first calibration instructions](docs/SIMPLE_CALIBRATION.md) before
moving the printer. Existing MISC installations can run this installer directly;
the old uninstaller is not required. See the
[migration details](docs/INSTALLATION.md#migrate-from-the-misc-plugin).

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
primary_branch: main
is_system_service: False
managed_services: klipper
```

Add the base `[update_manager]` section once if absent. After manual edits,
restart the matching Moonraker service while idle. `~` is Moonraker's service
user's home; use the printed absolute path if needed. Uninstall removes the
matching plugin entry while preserving unrelated updater configuration.
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
| `ACCEL_HOMING_CALIBRATE AXIS=X CENTERED=1` | Calibrate X. Use `AXIS=Y` for Y. Uses the sensor's explicit axes map or identifies the axis with central strokes, then measures fresh noise and qualifies the threshold. |
| `ACCEL_HOMING_CALIBRATE_NOISE` | Measure three stationary one-second noise windows and export a PNG; no movement or threshold change. |
| `ACCEL_HOMING_TEST CENTERED=1` | Test X then Y with active thresholds; both axes must use this plugin. |
| `ACCEL_HOMING_CHARTS_GENERATE AXIS=X CENTERED=1` | Home/chart X; use `AXIS=Y` or omit AXIS for both. |
| `ACCEL_HOMING_ACCURACY AXIS=X SAMPLES=10 CENTERED=1` | Measure repeated commanded-travel variation and save an accuracy PNG; X, Y or XY, 2–50 samples per axis. |
| `ACCEL_HOMING_TROUBLESHOOT AXIS=Y SAMPLES=50 CENTERED=1` | Record repeated homes, packet/stop/return timing, raw samples and diagnostic plots in a shareable bundle; fixed active settings, no recalibration or save. |
| `ACCEL_HOMING_EXPERIMENT AXIS=X CENTERED=1 MODE=COMPARE SAMPLES=2` | On a saved mapped axis, run four homes in two normal/shorter-delivery pairs at the calibrated threshold and export a ZIP. The historical XYZ comparison has 11 additional reference/discovery homes. See [experimental workflow](docs/EXPERIMENT.md). |
| `ACCEL_HOMING_CANCEL` | Cancel workflow work when handled and restore temporary settings; **not an emergency stop**. |
| `SET_ACCEL_HOMING AXIS=X THRESHOLD=1` | Set X's threshold in g until restart; use `AXIS=Y` for Y. Does not save a model. |

**CALIBRATE saves the selected axis and restarts Klipper by default.** It adds
0.5 g once to the candidate after ten checks pass in the default mode. Gentle
homing saves the exact threshold those ten homes tested. Prepare
XY again after restart before calibrating the other axis. Successful calibration
saves the selected axis's runtime threshold together with its model through
`SAVE_CONFIG`, replacing its previous manual value. A shared threshold remains
available to the other axis. Later manual overrides still take precedence. See
[configuration](docs/CONFIGURATION.md#thresholds-and-saved-models).

`NOTCH=AUTO` defaults to reporting only and tests the exact threshold it
would save. To report a tester run, send the ZIP printed by each command
(`MODE=COMPARE` and `NOTCH=AUTO`). Include the axis and how the individual
homes felt. Each ZIP contains the available measurements, host log for that
command, and diagnostic context. The report identifies missing evidence.

### Gentle homing on dev

For a printer already using this plugin for its selected XY endstops, run this
in the printer's SSH terminal while idle:

```bash
cd ~/accel_homing && git fetch origin dev && git switch dev && git merge --ff-only origin/dev && bash ./install.sh --gentle-homing
```

The installer enables `gentle_homing: True`, caps the existing plugin axes at
40 mm/s and their homing acceleration at 1000 mm/s², and preserves lower
positive settings. Driver currents retain their configured values. Review the
printed installation plan if a rail uses a second pass or a separate acceleration
override. Each axis needs a new calibration at its effective speed and acceleration
before ordinary homing.

Wait for Klipper to report ready. Run `ACCEL_HOMING_STATUS` in the printer
console and confirm `"gentle_homing": true` before the first contact. If
installation used `--no-restart`, first run the host service restart command
printed by the installer to load the updated Python modules.

**First test one contact.** Leave both axes unhomed, support Z, and use a ruler
or a center mark to place the toolhead within 2 mm of the configured XY midpoint.
With the tester at the printer, run the following in the printer console:

```text
ACCEL_HOMING_CALIBRATE AXIS=X CENTERED=1 NOTCH=AUTO CONTACT_ONLY=1
```

The command measures three free strokes and stationary noise on the mapped
channel, derives an initial threshold, and makes at most one approach to the
endstop. It prints a diagnostic ZIP and leaves the threshold and filter unsaved.
After a smooth X contact, release XY and manually center again for the same
command with `AXIS=Y`. If either axis hits hard, skips steps, or aborts, stop
there and share that ZIP. It contains the motion captures, contact samples when
available, trigger and halt step positions, effective settings, and host log.

The single-contact ZIP also has `contact-timing.json` and `contact-timing.svg`
at its root. The SVG shows the full seek and a zoom around scheduled braking,
with acceleration, cruise and braking shaded separately. Raw and diagnostic
200 Hz / Q=5 causal-notch replay traces use absolute acceleration in g;
timestamps use MCU `print_time` seconds. The JSON reports each phase's peak and
the largest filtered sample within 100 ms on either side of scheduled braking.
`peak_minus_brake_ms` is negative when the peak came before braking and positive
when it came after. If the detector requested a stop, a second window compares
the signal with the host time observed just after that request. The filter runs
offline. These timings describe commands and sensor readings. Physical contact
time remains unverified. The report identifies missing evidence on aborted runs.

Gentle supervised approaches command at most the midpoint-to-endstop distance
plus 2 mm. An impact outside the midpoint tolerance, or more than 0.5 mm of
commanded travel from trigger to halt, aborts the sequence. Ordinary G28 also
checks that trigger-to-halt distance. These distances come from step commands.
The physical toolhead position and motor slip require observation.

After each axis feels acceptable, run the ordinary calibration command above
for that axis, starting from the same manually confirmed midpoint each time.
The single-contact check alone does not qualify a model for G28.

Read the [command reference](docs/COMMANDS.md) for options and restrictions.
After physically validating both axes, use the printer's normal homing controls.

New calibration uses the selected sensor's explicit `accel_axes_map` (Beacon)
or `axes_map` (other accelerometers). With that setting, X homing uses the
configured stream's X component and Y uses Y, without automatic mapping strokes.
An explicitly written `x,y,z` counts; an omitted option with an internal identity
default does not. Without the setting, calibration learns the channel and sign
from gentle central strokes. Sensor configuration is preserved and its map is
applied only once. Ambiguous automatic mappings abort before threshold qualification. Older
XYZ-magnitude models require per-axis recalibration; their thresholds are not
silently reused as mapped-axis thresholds.

## Uninstall

Run while idle:

```bash
~/accel_homing/uninstall.sh
```

After uninstallation succeeds, configure the physical X/Y endstops and review
their homing settings before restarting the services listed in its output.
Previous settings are not restored from backups.
Only after those restarts, and after checking that no remaining plugin links
or manual updater entries need this checkout, remove the source if desired:

```bash
rm -rf ~/accel_homing
```

[Uninstall and recovery details](docs/INSTALLATION.md#uninstall).

## Documentation

[Installation and migration](docs/INSTALLATION.md) ·
[Supported sensors and limits](docs/COMPATIBILITY.md) ·
[Configuration](docs/CONFIGURATION.md) ·
[Notch filter (dev)](docs/NOTCH.md) ·
[Console commands](docs/COMMANDS.md) ·
[Calibration](docs/SIMPLE_CALIBRATION.md) ·
[Accuracy](docs/ACCURACY.md) ·
[Per-home diagnostic capture](docs/TROUBLESHOOT_CAPTURE.md) ·
[Experimental detector comparison](docs/EXPERIMENT.md) ·
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
