# Accelerometer Homing

Use a configured accelerometer as an X/Y impact endstop in Klipper or Kalico.
It replaces physical endstop switches for X and Y by detecting the toolhead's
impact on the mechanical stop. See [compatibility](docs/guide/compatibility.md)
for supported sensors and limits.

> [!WARNING]
> This plugin is experimental and deliberately contacts mechanical stops.
> Supervise the first homing and every calibration, and keep an emergency stop
> within reach.

## Quick start

1. **Check compatibility.** Confirm your printer and accelerometer in
   [compatibility](docs/guide/compatibility.md).
2. **Install.** Run in the Linux printer host's shell as the existing Klipper
   service user, with the printer idle:

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

   To test the development branch on an existing printer, see
   [Switch an existing tester installation to dev](docs/guide/installation.md#switch-an-existing-tester-installation-to-dev).
   The block above selects the stable branch. The installer configures X/Y
   homing and Moonraker updates, restarts the verified services and may ask for
   your Linux sudo password. It does not home the printer. Existing MISC installations can run this installer directly; see [migrating from MISC](docs/guide/installation.md#migrate-from-the-misc-plugin).
3. **Calibrate before first homing.** Follow the
   [calibration instructions](docs/guide/calibration.md) before moving the
   printer. `ACCEL_HOMING_CALIBRATE` moves the axis into its stop, then saves and restarts Klipper on success. Calibrate X, then Y; re-center XY by hand and keep both axes unhomed before each one.

Updating: use **accel_homing → Update** in Mainsail or Fluidd while idle.

## Everyday commands

Enter these in the Mainsail/Fluidd printer console, not the host shell.

| Command | What it does |
| --- | --- |
| `ACCEL_HOMING_STATUS` | Show settings, model, workflow progress, results and PNG paths. |
| `ACCEL_HOMING_CALIBRATE AXIS=X CENTERED=1` | Calibrate X. Use `AXIS=Y` for Y. |
| `ACCEL_HOMING_CALIBRATE_NOISE` | Measure stationary noise and export a PNG; no movement or threshold change. |
| `ACCEL_HOMING_TEST CENTERED=1` | Test X then Y with active thresholds. |
| `ACCEL_HOMING_ACCURACY AXIS=X SAMPLES=10 CENTERED=1` | Measure repeated commanded-travel variation and save an accuracy PNG. |
| `ACCEL_HOMING_TROUBLESHOOT AXIS=Y SAMPLES=50 CENTERED=1` | Record repeated homes and diagnostics in a shareable bundle. |
| `ACCEL_HOMING_CANCEL` | Cancel workflow work when handled; **not an emergency stop**. |
| `SET_ACCEL_HOMING AXIS=X THRESHOLD=1` | Set X's threshold in g until restart; does not save a model. |

Moving commands need both axes unhomed and the toolhead physically centered by hand, with Z supported. **`CENTERED=1` authorizes movement; it does not center the toolhead.**
See the [command reference](docs/guide/commands.md) for all commands and options.

## Documentation

Start at the [documentation index](docs/README.md). The guide pages are
[installation](docs/guide/installation.md),
[compatibility](docs/guide/compatibility.md),
[configuration](docs/guide/configuration.md),
[commands](docs/guide/commands.md),
[calibration](docs/guide/calibration.md) and
[troubleshooting](docs/guide/troubleshooting.md).

## Get help

Ask in the [Discord channel](https://discord.com/channels/1227971059764953230/1564459292113899632).
Report bugs with the GitHub issue form.

## Contributing

See [CONTRIBUTING.md](CONTRIBUTING.md).

## Uninstall

Run `~/accel_homing/uninstall.sh` while idle. See
[uninstall and recovery details](docs/guide/installation.md#uninstall).

## Credits and thanks

**[CloakedWayne](https://github.com/CloakedWayne) created the original Beacon
Accelerometer Homing plugin and its impact-detection algorithm** in
[Monolith3D/MISC](https://github.com/Monolith3D/MISC/tree/main/Beacon_Accel_Homing).
This project builds on that work and retains its GPLv3 license and attribution.

Thank you to everyone who has tested the plugin, shared results and helped
improve it. Special thanks to **Ingstromentalist**, **Acke**, and **mc-red** for
their time, patience and continued testing. Your reports make this work possible. This is a Vibecoded app, still in experimental release.
