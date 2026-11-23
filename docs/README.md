# Documentation

Start with the guide, in order. The advanced and development pages are for specific needs.

## Guide

1. [Supported sensors and limits](guide/compatibility.md): read this before installing, to check your sensor, firmware and machine are supported.
2. [Installation, updates and recovery](guide/installation.md): read this when installing, updating, migrating or uninstalling.
3. [Simple supervised calibration](guide/calibration.md): read this before the first homing, to calibrate each axis.
4. [Configuration](guide/configuration.md): read this when you need to change the installer-created settings.
5. [Printer console commands](guide/commands.md): read this when you need the exact syntax of a console command.
6. [Troubleshooting](guide/troubleshooting.md): read this when installation, calibration or homing misbehaves.

## Advanced

- [Accuracy workflow](advanced/accuracy.md): measure variation in commanded homing travel and recover from early triggers.
- [Notch filter](advanced/notch.md): attenuate a narrow frequency band during homing, set manually or with `NOTCH=AUTO`.
- [PNG plots and stationary noise](advanced/plots.md): generate and read the chart images.
- [Per-home diagnostic capture](advanced/diagnostic-capture.md): record evidence for intermittent detection, delayed stopping or a changed return.
- [Homing comparison experiment](advanced/experiment.md): compare normal and shorter batch-delivery intervals.
- [Beta moving-noise calibration](advanced/beta.md): a separate supervised experiment that proposes a threshold from moving noise.
- [Technical reference](advanced/technical-reference.md): detector, preparation and transport policies behind the options.

## Development

- [Offline verification](development/testing.md): run the test suite.
- [Historical verification notes](development/historical-testing.md): archived testing narrative with original dates.
- [Contributing](../CONTRIBUTING.md): how to propose changes.
