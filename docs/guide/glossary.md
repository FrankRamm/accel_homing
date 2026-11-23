# Glossary

[← Documentation](../README.md)

Plain-language meanings of the words used in these docs.

## SSH

SSH lets you type commands on another computer over your network. You use it to
reach the computer that runs your printer from your own PC or laptop. Recent Windows (10 and newer),
macOS and Linux all have an `ssh` command built in. See
[Open a terminal on your printer](installation.md#open-a-terminal-on-your-printer).

## Terminal

A terminal is a window where you type commands and press Enter. When these docs
say "a terminal on your printer", they mean a terminal running on the computer
that runs Klipper, usually opened over [SSH](#ssh). It is not the Mainsail or
Fluidd console, which only accepts G-code and Klipper commands.

## Printer host

The printer host is the computer that runs Klipper. It is often a Raspberry Pi
or the printer's built-in board. Mainsail or Fluidd runs on it too. Your
printer's control board (the "MCU") is a separate, smaller board that the host
talks to.

## printer.cfg

`printer.cfg` is the main Klipper configuration file. You can open and edit it
in the Mainsail or Fluidd "Machine" or "Configuration" page. Klipper reads it
when it starts or restarts. The plugin adds its settings to it or to a file it
includes.

## Moonraker

Moonraker is the program that connects Klipper to Mainsail and Fluidd. It also
handles software updates shown there. It is separate from both Klipper and the
web interface.

## Stable and test versions

The stable version is the released code on the `main` branch. The test version
is the `dev` branch. It has newer features but may have bugs. The installer
installs the stable version unless you choose to
[switch to the test version](installation.md#switch-to-the-test-version).

## Homing

Homing is when the printer moves an axis until it finds a known position, so it
knows where the toolhead is. It normally happens when you send `G28` or press
the home buttons. With this plugin, X and Y find that position by sensing the
motion with an accelerometer instead of a switch.

## Endstop

An endstop is the sensor that tells the printer an axis has reached its end.
Usually it is a small switch. With this plugin, the accelerometer replaces the
physical X and Y endstops for homing.

## Accelerometer

An accelerometer is a small sensor that measures vibration and sudden
movement. Klipper users usually have one, such as an ADXL345, for input shaping.
The plugin uses it to notice when an axis reaches its end during homing.

## Threshold

The threshold is how strong a bump must be before the plugin treats it as the
axis reaching its end. Too low and noise or vibration can stop homing early.
Calibration finds a suitable value for you.

## Calibration

Calibration is the supervised test run that measures your printer and sets the
threshold for each axis. You watch the printer and are ready to stop it. Do it
before you home normally with the plugin.

## Emergency stop

The emergency stop is the command `M112`, or the red button in Mainsail or
Fluidd (it may be labelled EMERGENCY STOP). It shuts Klipper down immediately. Use it if the printer does something
unsafe. Afterwards Klipper needs a firmware restart (`FIRMWARE_RESTART`, or the restart button in Mainsail or Fluidd) before you can print again.
