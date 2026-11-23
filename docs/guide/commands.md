# Printer console commands

[← Documentation](../README.md)

Enter these commands in the Mainsail/Fluidd **printer console**. Installation
commands belong in the Linux host shell instead.

## Prepare every moving workflow

Before CALIBRATE, TEST, CHARTS, ACCURACY, TROUBLESHOOT, EXPERIMENT or BETA:

1. Keep the printer idle and unpaused, with no queued motion or other active
   homing workflow.
2. Support Z and establish nozzle/bed clearance for the complete XY travel.
3. Use your printer's appropriate controls to leave **both X and Y unhomed**,
   then physically center XY by hand.
4. Prepare suitable XY currents/enables separately if your normal G28 override
   does this. Workflow homes skip that override and its Z hops.
5. Supervise with an accessible emergency stop. `CENTERED=1` confirms the
   preparation and authorizes movement without a second confirmation.

Do not assume a motor-off command affects only XY; use a procedure appropriate
for your machine and independent Z support. See the complete
[workflow restrictions](compatibility.md#workflow-restrictions).

## Command reference

| Command | Options and behavior |
| --- | --- |
| `ACCEL_HOMING_STATUS` | Read-only workflow/model/settings, result and plot paths. |
| `BEACON_ACCEL_HOMING_STATUS` | Read-only detector and preparation diagnostics; retained legacy name. |
| `ACCEL_HOMING_CALIBRATE AXIS=X CENTERED=1` | One axis, required `AXIS=X` or `Y`; optional `START_THRESHOLD`, `STEP`, `SAVE_CONFIG=0|1`; default saves/restarts on success. With `gentle_homing: True`, selects and freezes a profile before the first contact. |
| `ACCEL_HOMING_CALIBRATE AXIS=X CENTERED=1 NOTCH=AUTO SAVE_CONFIG=0` | Select and qualify an exact filter/threshold pair. Explicit AUTO defaults to report only; `SAVE_CONFIG=1` repeats the run and saves on success. |
| `ACCEL_HOMING_CALIBRATE AXIS=X CENTERED=1 NOTCH=AUTO CONTACT_ONLY=1` | Gentle mode only: three free-travel controls, stationary checks, then at most one contact with the selected profile. Exports a ZIP; never qualifies, stages or saves a model. |
| `ACCEL_HOMING_CALIBRATE_NOISE` | Three stationary one-second noise windows and PNG; no movement, model save or restart. `SAVE_CONFIG` is unsupported. |
| `ACCEL_HOMING_TEST CENTERED=1` | Test X then Y with active thresholds; both must use this plugin. Does not calibrate or save. |
| `ACCEL_HOMING_EXPERIMENT AXIS=X CENTERED=1 MODE=COMPARE SAMPLES=2` | Test normal and shorter report intervals over two pairs. With a saved mapped-axis calibration this means four homes using its channel, threshold and fixed filter. Exports a comparison ZIP. See [EXPERIMENT](../advanced/experiment.md). |
| `ACCEL_HOMING_EXPERIMENT AXIS=X SAMPLES=10 CENTERED=1 MODE=SHADOW` | One-axis diagnostic retest: discovery, ten reference homes, then raw homes with frozen-profile replay after motion. No shadow profile controls a home. Existing `MODE=LIVE` remains available. No save or restart. See [EXPERIMENT](../advanced/experiment.md). |
| `ACCEL_HOMING_CHARTS_GENERATE CENTERED=1` | Home/chart X then Y. Optional `AXIS=X` or `Y` selects one; omit for both. |
| `ACCEL_HOMING_ACCURACY AXIS=X SAMPLES=10 CENTERED=1` | Repeated travel measurement. `AXIS=X|Y|XY`, default `XY`; integer `SAMPLES=2..50`, default 10. Recovery can start a saving calibration; see below. |
| `ACCEL_HOMING_TROUBLESHOOT AXIS=Y SAMPLES=50 CENTERED=1` | Per-home diagnostic capture. `AXIS=X|Y|XY`, default `XY`; integer `SAMPLES=2..50`, default 50. Keeps active homing settings; no calibration, save or restart. Preserves available evidence on failure. |
| `ACCEL_HOMING_BETA_CALIBRATE AXIS=X CENTERED=1` | Experimental moving-noise calibration proposal for X or Y. Optional `START_THRESHOLD`, `STEP`, `SAVE_CONFIG=0`; rejects `SAVE_CONFIG=1`. |
| `ACCEL_HOMING_CANCEL` | Cancel owned workflow/pending preparation and restore temporary settings when handled. Not an emergency stop. |
| `SET_ACCEL_HOMING AXIS=X THRESHOLD=1` | Temporary threshold for X or Y; optional compatibility `IMPACT_SIGN` accepts `-1`, `0`, `1`; until restart. |
| `SET_BEACON_ACCEL_HOMING AXIS=X THRESHOLD=1` | Retained alias of `SET_ACCEL_HOMING`. |

`IMPACT_SIGN` does not select RAW detector polarity. A SET containing only
`IMPACT_SIGN` does not create an explicit manual threshold or requalify an old
selected-axis model; supply `THRESHOLD` or complete fresh calibration if the
model's qualification is refused.

No separate centering-confirmation command or popup prompt is required.
The optional [notch filter](../advanced/notch.md) has independent X/Y configuration overrides.
`NOTCH=AUTO` on CALIBRATE measures moving noise and selects a profile automatically.
Gentle calibration also selects a profile when `NOTCH` is omitted. Ordinary G28,
ACCURACY, TEST and CHARTS use their existing detector; they do not select filters.
After either tester command, send the ZIP path printed in the console and
identify the axis and any unusually hard homes. The ZIP includes available
captures, the Klippy log interval, and the effective settings for that run.
Beta calibration refuses an axis with an enabled notch before movement; use
ordinary calibration for that axis.
Only one workflow may own the sensor/motion session at a time. A new command
during an active run is rejected; wait for completion or cancel it. Gentle
calibration ends on the first rejected contact without an automatic retry.

## Calibration

```gcode
ACCEL_HOMING_CALIBRATE AXIS=X CENTERED=1
```

Successful default calibration saves X and restarts Klipper. Repeat manual
preparation after that restart before calibrating Y:

```gcode
ACCEL_HOMING_CALIBRATE AXIS=Y CENTERED=1
```

With gentle mode disabled and `NOTCH` omitted, the saved threshold is the
candidate that passed ten checks **plus 0.5 g**, applied once after qualification.
Those checks do not test the higher saved value. Gentle calibration and explicit
AUTO instead save the **exact tested threshold and filter**, with no later margin.
The other axis is unchanged.

Both commands use an explicit `accel_axes_map` (Beacon) or `axes_map` from the
selected sensor's configuration. Without that setting, they identify the
selected physical axis with gentle bidirectional strokes around the manual
center. Both paths measure fresh stationary noise on the selected component.
Ambiguous automatic mapping aborts before homing. For ordinary calibration, the
default start grid is 1 g with 0.25 g steps. `START_THRESHOLD` chooses its first
candidate while retaining the measurements; `STEP` changes grid spacing. In
gentle calibration, `START_THRESHOLD` is a minimum for the pre-contact selection;
the measured controls may require a higher value. `STEP` never permits an
increase after that candidate is frozen. Starts are finite and at least 0.5 g;
step is finite 0.1–3 g; known-range headroom still applies.

To defer saving:

```gcode
ACCEL_HOMING_CALIBRATE AXIS=X CENTERED=1 SAVE_CONFIG=0
```

This stages the model and selected-axis runtime threshold, plus the selected
filter in gentle mode, without writing or
restarting; it does not activate them for ordinary homing. Issue `SAVE_CONFIG`
to persist/reload both and comment the replaced selected-axis setting. The
other axis and any shared fallback remain unchanged. Later manual overrides
still take precedence. Read the
[complete calibration sequence](calibration.md). Its early-hit automatic
returns apply to ordinary calibration, not gentle mode. Both require ten
validation homes with at most 2 mm of commanded travel spread after discovery.

### Automatic profile selection and gentle calibration (dev)

After manual preparation, to measure and report without staging or saving:

```gcode
ACCEL_HOMING_CALIBRATE AXIS=Y CENTERED=1 NOTCH=AUTO SAVE_CONFIG=0
```

With `gentle_homing: True`, three independent central free-travel strokes and
three stationary windows precede all contact attempts. Two moving controls
select a frequency/Q pair or a disabled filter and threshold; the third can
reject that frozen choice. Startup, acceleration and cruise are checked in the
candidate's signal domain. Braking is retained as a diagnostic. A failed control
does not cause a search for another profile or a higher threshold.

The first home already uses the selected live detector. Full calibration needs
one discovery and ten validation homes using that exact profile, mapped
component and threshold. It does not require a previously successful raw trigger
or eleven unfiltered reference approaches. Any rejected contact ends the run.
When gentle mode is disabled, explicit AUTO retains its two-stage sequence:
three central strokes, raw discovery plus ten reference homes, offline selection,
then fresh discovery plus ten fixed-profile validation homes.

The commands preserve the configured homing speed and acceleration. Gentle
mode requires compatible settings at or below 40 mm/s and 1000 mm/s², and refuses
incompatible geometry or limits; it does not lower them to obtain a passing
result. The installer's `--gentle-homing` option prepares those configuration
caps separately. Load the new modules using its printed host-service restart
command if installation used `--no-restart`, then wait for Klipper to be ready
and confirm `gentle_homing` in `ACCEL_HOMING_STATUS`.

For a supervised first contact, prepare XY within 2 mm of its configured midpoint:

```gcode
ACCEL_HOMING_CALIBRATE AXIS=X CENTERED=1 NOTCH=AUTO CONTACT_ONLY=1
```

This makes at most one approach and requires `SAVE_CONFIG=0` (the default for
explicit AUTO). It saves a diagnostic ZIP, restores the previous runtime
threshold/filter, and does not qualify a G28 model. After inspecting the result,
manually prepare XY again for a full calibration. A centered gentle approach
commands at most midpoint-to-endstop distance plus 2 mm. A trigger outside the
center-travel tolerance or more than 0.5 mm of commanded trigger-to-halt travel
ends the sequence. Ordinary gentle G28 also checks trigger-to-halt travel.
Qualification also requires the original detector onset to fall within a
matched native acceleration or cruise interval. A crossing during planned
braking, outside a known approach interval, or without matching native motion
evidence is rejected before another approach or saving.

Saving behavior depends on whether `NOTCH=AUTO` was written explicitly:

| Command form | Successful result |
| --- | --- |
| Bare CALIBRATE, `SAVE_CONFIG` omitted | Save the qualified model/settings and request a Klipper restart. |
| Bare CALIBRATE, `SAVE_CONFIG=0` | Stage the model/settings for a later `SAVE_CONFIG`; restore the previous runtime settings. |
| Explicit `NOTCH=AUTO`, `SAVE_CONFIG` omitted or `0` | Report only; stage and save nothing. |
| Explicit `NOTCH=AUTO SAVE_CONFIG=1` | Repeat measurements and qualification, save the model/settings, then request a restart. |

Gentle saving can replace the selected axis's explicit threshold. The other axis
and shared fallback remain unchanged. Gentle/AUTO saving rejects unrelated pending
configuration changes. With gentle mode disabled, explicit AUTO still requires
removing the selected-axis/shared manual threshold override before saving.

Raw samples remain in the ZIP. `contact-summary.json` lists each approach's
threshold, recorded profile, and replayed peaks before, during and after planned
braking, including failed no-trigger approaches. The main `contact-timing.json`
and `.svg` replay the recorded candidate; full calibration writes numbered
`contact-0001-timing` files for each approach. Historical 200 Hz/Q=5 replay is
kept separately in `historical_signal` and an explicitly named historical SVG.
Offline replays never select or control a home. A replay peak, scheduled braking time, or plausible
step travel does not establish the instant of physical contact. A missing trigger
does not establish that no physical contact occurred. Record the operator's
observations separately: free-travel margins alone do not prove sensitivity to a
soft stop, and step counts do not measure motor slip or physical toolhead position.
See [automatic selection and evidence](../advanced/notch.md#automatic-selection).

## Accuracy and early-trigger recovery

```gcode
ACCEL_HOMING_ACCURACY AXIS=X SAMPLES=10 CENTERED=1
```

This measures repeated commanded travel at the active threshold, filter and speed.
In gentle mode, it requires a matching qualified model and stops on a rejected
contact without raising the threshold or starting recovery calibration.

With gentle mode disabled, an early trigger with valid current capture/halt/center
evidence reverses its
recorded commanded halt travel, reports failure in the console without an
accuracy PNG and starts one
normal calibration of the failing axis. Its first candidate is the actual
threshold used by the failed attempt **plus 0.25 g**; later steps use configured
`calibration_step`. Successful recovery calibration saves its qualified
candidate plus the normal 0.5 g final margin and restarts Klipper.
The console then tells you to prepare XY again and rerun the exact accuracy
command; it does not rerun accuracy automatically.

`SAVE_CONFIG` is not an ACCURACY option: even `SAVE_CONFIG=0` is rejected.
Read [accuracy and its recovery limits](../advanced/accuracy.md) before running it.

## Per-home diagnostic capture

```gcode
ACCEL_HOMING_TROUBLESHOOT AXIS=Y SAMPLES=50 CENTERED=1
```

This moving diagnostic uses the existing detector, native selected-axis home,
midpoint return and settle policy. It records discovery plus the requested
measured homes, including packet timing, trigger/halt positions, return
coordinates and full retained accelerometer samples. It does not change
thresholds, speed or acceleration settings, calibrate a new model, save
configuration or restart. `SAVE_CONFIG` is unsupported.

A rejected home ends this diagnostic instead of starting ACCURACY's automatic
recovery calibration. Available rejected, cancelled and failed-run evidence is
retained. Files and graphs are produced after motion cleanup; console and
STATUS report the bundle paths. Share the complete bundle, not only a PNG.
See [diagnostic contents and interpretation](../advanced/diagnostic-capture.md).

## Stationary noise and charts

```gcode
ACCEL_HOMING_CALIBRATE_NOISE
```

NOISE leaves motor enable/current and fan state unchanged. Keep the printer
stationary; it needs an idle, unpaused machine, an empty motion queue and an
exclusive usable sensor stream. Its configured-stream component hints do not configure
ordinary homing and are not reused by later calibration.

```gcode
ACCEL_HOMING_CHARTS_GENERATE AXIS=X CENTERED=1
```

CHARTS moves the selected axis; omitting AXIS moves X then Y. It uses the active
detector/threshold and does not calibrate them. See [plot output](../advanced/plots.md).

## Temporary threshold changes

```gcode
SET_ACCEL_HOMING AXIS=X THRESHOLD=1
```

This changes the active threshold until restart, without saving a model.
Thresholds must be finite and positive and satisfy known sensor-range limits.
The current model/manual detector context remains relevant; the command does
not turn a legacy threshold into a newly qualified mapped-axis model.
Cancel an active workflow before changing settings.

For persistent manual changes, edit the existing plugin section and issue
`RESTART` while idle. See [threshold precedence](configuration.md#thresholds-and-saved-models).

## Cancellation and failures

`ACCEL_HOMING_CANCEL` is a normal console command. Frontend/console scheduling
can delay it while another command is executing. Use the accessible emergency
stop for movement that must stop immediately.

On an abort, inspect the message and STATUS. Do not assume the toolhead remains
centered, that a model was saved or that a final release occurred. Correct the
reported fault and manually prepare XY before a new moving command. Shutdown
or stock homing fault handling can turn off motors, including Z.

Next: [Troubleshooting](troubleshooting.md)
