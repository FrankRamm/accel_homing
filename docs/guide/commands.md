# Printer console commands

[← Documentation](../README.md)

> **What you'll do:** look up a command and what it does, then run it in the
> printer console.
>
> **What you need:** the plugin installed and set up (see
> [installation](installation.md)). For commands that move the printer, you
> also need to be at the printer.
>
> **Time:** a few minutes to read. Calibration itself takes longer.

Enter these commands in the Mainsail/Fluidd **printer console**. Installation
commands belong in [a terminal on your printer](glossary.md#terminal) instead
(the Linux host shell, usually reached over [SSH](glossary.md#ssh)).

## Prepare every moving workflow

Some commands make the printer move on its own. Do these steps first, every
time. Skipping them can crash the toolhead.

Before CALIBRATE, TEST, CHARTS, ACCURACY, TROUBLESHOOT, EXPERIMENT or BETA
(in plain words: before any command below that homes or moves the printer):

1. Keep the printer idle and unpaused, with no queued motion or other active
   homing workflow. (Nothing is printing, nothing is waiting to move, and no
   other homing command from this plugin is running.)
2. Support Z and establish nozzle/bed clearance for the complete XY travel.
   (Make sure the nozzle cannot hit the bed anywhere the toolhead can go in X
   and Y, and that the bed or gantry cannot drop.)
3. Use your printer's appropriate controls to leave **both X and Y unhomed**,
   then physically center XY by hand.
   (Unhome X and Y in the way that suits your machine. Read the warning below
   about motor-off commands first. Then move the toolhead to the middle by
   hand.)
4. Prepare suitable XY currents/enables separately if your normal G28 override
   does this. Workflow homes skip that override and its Z hops. (If your
   homing macro normally switches motor current or enables the motors, do that
   yourself first, because these commands do not run that macro.)
5. Supervise with an accessible [emergency stop](glossary.md#emergency-stop).
   `CENTERED=1` confirms the preparation and authorizes movement without a
   second confirmation. (Adding `CENTERED=1` tells the plugin you finished
   steps 1 to 4. Only add it when that is true.)

Do not assume a motor-off command affects only XY; use a procedure appropriate
for your machine and independent Z support. See the complete
[workflow restrictions](compatibility.md#workflow-restrictions).

## Command reference

This table lists every command. The first column is what you type. The last
column says in plain words what it is for. Commands marked as read-only do
not move anything.

| Command | Options and behavior | What it's for |
| --- | --- | --- |
| `ACCEL_HOMING_STATUS` | Read-only workflow/model/settings, result and plot paths. | Shows the plugin's current state. Read-only: it does not move anything. |
| `BEACON_ACCEL_HOMING_STATUS` | Read-only detector and preparation diagnostics; retained legacy name. One line per recent homing move, with post-contact travel and stop-request delay when known ([lost steps](troubleshooting.md#the-axis-loses-position-or-skips-steps-after-homing)). | Same idea for the sensor, kept under its old name. Read-only: it does not move anything. |
| `ACCEL_HOMING_CALIBRATE AXIS=X CENTERED=1` | One axis, required `AXIS=X` or `Y`; optional `START_THRESHOLD`, `STEP`, `SAVE_CONFIG=0|1`; default saves/restarts on success. With `gentle_homing: True`, selects and freezes a profile before the first contact. | Finds the [threshold](glossary.md#threshold) for one axis (the main [calibration](glossary.md#calibration) command). |
| `ACCEL_HOMING_CALIBRATE AXIS=X CENTERED=1 NOTCH=AUTO SAVE_CONFIG=0` | Select and qualify an exact filter/threshold pair. Explicit AUTO defaults to report only; `SAVE_CONFIG=1` repeats the run and saves on success. | Calibrates and lets the plugin pick a noise filter too, without saving anything. |
| `ACCEL_HOMING_CALIBRATE AXIS=X CENTERED=1 NOTCH=AUTO CONTACT_ONLY=1` | Gentle mode only: three free-travel controls, stationary checks, then at most one contact with the selected profile. Exports a ZIP; never qualifies, stages or saves a model. | A careful first test: at most one gentle touch of the [endstop](glossary.md#endstop), nothing saved. |
| `ACCEL_HOMING_CALIBRATE_NOISE` | Three stationary one-second noise windows and PNG; no movement, model save or restart. `SAVE_CONFIG` is unsupported. | Measures how much the sensor picks up while the printer sits still. The printer does not move. |
| `ACCEL_HOMING_TEST CENTERED=1` | Test X then Y with active thresholds; both must use this plugin. Does not calibrate or save. | Checks that homing works with the current settings, X then Y. |
| `ACCEL_HOMING_EXPERIMENT AXIS=X CENTERED=1 MODE=COMPARE SAMPLES=2` | Test normal and shorter report intervals over two pairs. With a saved mapped-axis calibration this means four homes using its channel, threshold and fixed filter. Exports a comparison ZIP. See [EXPERIMENT](../advanced/experiment.md). | Diagnostic runs; the tester ZIP can be sent to the developers. |
| `ACCEL_HOMING_EXPERIMENT AXIS=X SAMPLES=10 CENTERED=1 MODE=SHADOW` | One-axis diagnostic retest: discovery, ten reference homes, then raw homes with frozen-profile replay after motion. No shadow profile controls a home. Existing `MODE=LIVE` remains available. No save or restart. See [EXPERIMENT](../advanced/experiment.md). | Diagnostic runs; the tester ZIP can be sent to the developers. |
| `ACCEL_HOMING_CHARTS_GENERATE CENTERED=1` | Home/chart X then Y. Optional `AXIS=X` or `Y` selects one; omit for both. | Homes and draws charts of what the sensor saw. |
| `ACCEL_HOMING_ACCURACY AXIS=X SAMPLES=10 CENTERED=1` | Repeated travel measurement. `AXIS=X|Y|XY`, default `XY`; integer `SAMPLES=2..50`, default 10. Recovery can start a saving calibration; see below. | Checks how repeatable your homing is. |
| `ACCEL_HOMING_TROUBLESHOOT AXIS=Y SAMPLES=50 CENTERED=1` | Per-home diagnostic capture. `AXIS=X|Y|XY`, default `XY`; integer `SAMPLES=2..50`, default 50. Keeps active homing settings; no calibration, save or restart. Preserves available evidence on failure. | Records detailed data about each home, to find out why homing misbehaves. |
| `ACCEL_HOMING_BETA_CALIBRATE AXIS=X CENTERED=1` | Experimental moving-noise calibration proposal for X or Y. Optional `START_THRESHOLD`, `STEP`, `SAVE_CONFIG=0`; rejects `SAVE_CONFIG=1`. | Experimental calibration. |
| `ACCEL_HOMING_CANCEL` | Cancel owned workflow/pending preparation and restore temporary settings when handled. Not an emergency stop. | Stops a running plugin command. It is not an emergency stop. |
| `SET_ACCEL_HOMING AXIS=X THRESHOLD=1` | Temporary threshold for X or Y; optional compatibility `IMPACT_SIGN` accepts `-1`, `0`, `1`; until restart. | Tries a different threshold until the next restart, without saving it. |
| `SET_BEACON_ACCEL_HOMING AXIS=X THRESHOLD=1` | Retained alias of `SET_ACCEL_HOMING`. | Another name for the command above. |

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

Calibration teaches the plugin how strongly the sensor must feel a bump
before it counts as the endstop being reached. Do it for each axis you use, and again after changing the sensor setup. Start
with the checklist above, then run:

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
The margin is reduced if needed so the saved value stays at or below 80% of the
weakest impact those homes measured, never below the tested candidate.
Those checks do not test a higher saved value. Gentle calibration and explicit
AUTO instead save the **exact tested threshold and filter**, with no later margin.
The other axis is unchanged.

Ordinary calibration then derives a lower
[cruise threshold](calibration.md#cruise-threshold) from the vibration of steady
travel and tests it with five more homes. It is saved with the model only if all
five pass; otherwise the console says why and the normal threshold is still saved.

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
restarting; it does not activate them for ordinary homing, which keeps the
previous threshold until `SAVE_CONFIG` and the restart. Issue `SAVE_CONFIG`
to persist/reload both and comment the replaced selected-axis setting. The
other axis and any shared fallback remain unchanged. Later manual overrides
still take precedence. Read the
[complete calibration sequence](calibration.md). Its early-hit automatic
returns apply to ordinary calibration, not gentle mode. Both require ten
validation homes with at most 2 mm of commanded travel spread after discovery.

### Automatic profile selection and gentle calibration (dev)

This part is only in the [test version](glossary.md#stable-and-test-versions)
(newer features, may have bugs). It lets the plugin choose a noise filter for
you and touch the endstop more gently.

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
braking, including failed no-trigger approaches. If the detector never fires,
`commanded_trigger_travel_mm_signed` is `null`. Use `commanded_halt_travel_mm_signed`
for the commanded distance at the end of that approach. For a `klipper_batch` trigger,
`trigger_batch_timing` gives three intervals in milliseconds from the triggering
sample to the last sample of its batch, from that sample to the adapter's host
timestamp after row validation, and from there to the host's stop-request
timestamp. These last two timestamps are estimates in the main MCU print-time
clock. Hardware stop receipt and physical stopping distance remain unmeasured.
For host and bus context, `debug/context.json` includes the board model when
Linux exposes it, the logical CPU count, and available Klipper system and CAN
statistics at the start and end of the run. The archived Klippy log contains
periodic statistics when logging is enabled. Short scheduling delays can occur
between observations.
The main `contact-timing.json`
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

This checks how repeatable your [homing](glossary.md#homing) is by homing
many times and measuring the differences. If the printer stops too early,
recovery can start a calibration that saves a new threshold and restarts
Klipper (see below). Run accuracy after calibrating:

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

Use this when homing misbehaves and you want to record what happens on each
home, for example to send to the developers:

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

Noise measurement checks how much the [accelerometer](glossary.md#accelerometer)
picks up while the printer sits still. Charts draw what the sensor saw while
homing.

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

Use this to try a different [threshold](glossary.md#threshold) without
changing your saved settings. It is forgotten when Klipper restarts.

```gcode
SET_ACCEL_HOMING AXIS=X THRESHOLD=1
```

This changes the active threshold until restart, without saving a model.
Thresholds must be finite and positive and satisfy known sensor-range limits.
The current model/manual detector context remains relevant; the command does
not turn a legacy threshold into a newly qualified mapped-axis model.
Cancel an active workflow before changing settings.
A value below twice the homing acceleration ramp prints one extra advice line
after the normal reply; the value is still applied. See
[low manual thresholds](configuration.md#thresholds-and-saved-models).

For persistent manual changes, edit the existing plugin section and issue
`RESTART` while idle. See [threshold precedence](configuration.md#thresholds-and-saved-models).

## Cancellation and failures

If a plugin command is running and you want it to stop, you can cancel it.
Cancelling is not instant, so it is not a substitute for the emergency stop.

`ACCEL_HOMING_CANCEL` is a normal console command. Frontend/console scheduling
can delay it while another command is executing. Use the accessible emergency
stop for movement that must stop immediately.

On an abort, inspect the message and STATUS. Do not assume the toolhead remains
centered, that a model was saved or that a final release occurred. Correct the
reported fault and manually prepare XY before a new moving command. Shutdown
or stock homing fault handling can turn off motors, including Z.

## Gentle homing (dev)

Gentle homing is a feature of the [test version](glossary.md#stable-and-test-versions)
(newer features, may have bugs). It makes the printer touch the endstop with
less force. Stay at the printer for the whole test.

Settings and axes-map behavior are in [Gentle homing (dev)](configuration.md#gentle-homing-dev).

After mapping, a gentle X run records and filters only physical X. A Y run
does the same for physical Y. The resting reference, moving-noise threshold,
contact trigger and ZIP traces all use that one component. For example, if
the console reports X mapped to sensor +Y, the X run reads sensor Y. Without
an explicit sensor map, the initial free strokes compare the three sensor
channels to identify the requested axis. These strokes move only in XY.

`NOTCH=AUTO` defaults to reporting only and tests the exact threshold it
would save. To report a tester run, send the ZIP printed by each command
(`MODE=COMPARE` and `NOTCH=AUTO`). Include the axis and how the individual
homes felt. Each ZIP contains the available measurements, host log for that
command, and diagnostic context. The report identifies missing evidence.

For a printer already using this plugin for its selected XY endstops, run this
in the printer's SSH terminal (see [how to open a terminal on your printer](installation.md#open-a-terminal-on-your-printer)) while idle:

```bash
cd ~/accel_homing && git fetch origin dev && git switch dev && git merge --ff-only origin/dev && bash ./install.sh --gentle-homing
```

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

The command uses two free strokes to select a mapped-axis filter and threshold.
A third independent stroke and three stationary windows must clear the same
threshold before the toolhead approaches the endstop. The detector can retain
the unfiltered signal when a notch gives no benefit. It makes at most one
contact attempt, prints a diagnostic ZIP and leaves the settings unsaved.
After a smooth X contact, release XY and manually center again for the same
command with `AXIS=Y`. If either axis hits hard, skips steps, or aborts, stop
there and share that ZIP. It contains the motion captures, contact samples when
available, trigger and halt step positions, effective settings, and host log.

The single-contact ZIP also has `contact-timing.json` and `contact-timing.svg`
at its root. The SVG shows the full seek and a zoom around scheduled braking,
with acceleration, cruise and braking shaded separately. The raw signal and
the replay of the selected detector use absolute acceleration in g;
timestamps use MCU `print_time` seconds. The JSON reports each phase's peak and
the largest filtered sample within 100 ms on either side of scheduled braking.
`peak_minus_brake_ms` is negative when the peak came before braking and positive
when it came after. If the detector requested a stop, a second window compares
the signal with the host time observed just after that request. The fixed
200 Hz / Q=5 replay is a separate historical comparison. These timings describe
commands and sensor readings. The report identifies missing evidence on aborted
runs. A soft contact can produce a signal below the selected threshold.

Each new gentle diagnostic ZIP includes its own run at the root. Under
`history/` it also includes original captures, per-run Klippy logs, reports and
plots from up to four recent same-axis contact archives, including failed runs.
`history/index.json` identifies the source runs and any read or size omissions.
The limit for included history is 256 MiB of original ZIP member data.

To collect up to five existing X runs into a single ZIP **without moving the
printer or repeating calibration**, update the plugin and run:

```text
ACCEL_HOMING_REPORT AXIS=X
```

Use `AXIS=Y` for Y. The console prints the report path. Existing runs retain
the evidence recorded by the installed version at the time. On new missed
seeks, the collector tries to retain samples through 100 ms past the native
move end, for up to 250 ms after the dispatcher stops. The timing JSON records
the last retained sample at disarm, tail coverage and the threshold crossing
separately from peaks after a stop request. Tail samples are diagnostic only.

Gentle supervised approaches command at most the midpoint-to-endstop distance
plus 2 mm. An impact outside the midpoint tolerance, or more than 0.5 mm of
commanded travel from trigger to halt, aborts the sequence. Ordinary G28 also
checks that trigger-to-halt distance. These distances come from step commands.
The physical toolhead position and motor slip require observation.

Full gentle calibration uses the same selection procedure. Starting with XY
unhomed and physically centered, run `ACCEL_HOMING_CALIBRATE AXIS=X CENTERED=1`
for X or use `AXIS=Y` for Y. One discovery and ten repeat contacts must qualify
with the exact selected filter and threshold before the command saves that
axis. Each onset must precede commanded braking, fall within the center-travel
tolerance and meet the trigger-to-halt limit. Ordinary G28 checks the saved
model and the phase of its own trigger. A single-contact check qualifies no model.

Read the [command reference](#command-reference) for options and restrictions.
After physically validating both axes, use the printer's normal homing controls.

Next: [Troubleshooting](troubleshooting.md)
