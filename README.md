# Accelerometer Homing

Use a configured accelerometer as an X/Y impact endstop in Klipper or Kalico.
This experimental plugin deliberately contacts mechanical stops. Supervise
initial homing and calibration with an accessible emergency stop.

## Install

Run in the Linux printer host's shell as the existing Klipper service user,
with the printer idle:

```bash
cd ~
git clone https://github.com/FrankRamm/accel_homing.git
~/accel_homing/install.sh
```

The installer backs up its changes, configures X/Y homing and Moonraker
updates, and restarts the verified services. It may ask for your
Linux sudo password. It does not home the printer.

Read the [first calibration instructions](docs/SIMPLE_CALIBRATION.md) before
moving the printer. Existing MISC installations must follow the
[migration instructions](docs/INSTALLATION.md#migrate-from-the-misc-plugin) first.

For later updates, use **accel_homing → Update** in Mainsail or Fluidd while idle.

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
