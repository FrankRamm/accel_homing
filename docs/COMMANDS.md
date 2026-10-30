# Printer console commands

Enter these commands in the Mainsail/Fluidd **printer console**. Installation
commands belong in the Linux host shell instead.

## Prepare every moving workflow

Before CALIBRATE, TEST, CHARTS, FILTER_CHARTS, ACCURACY or BETA:

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
[workflow restrictions](COMPATIBILITY.md#workflow-restrictions).

## Command reference

| Command | Options and behavior |
| --- | --- |
| `ACCEL_HOMING_STATUS` | Read-only workflow/model/settings, result and plot paths. |
| `BEACON_ACCEL_HOMING_STATUS` | Read-only detector and preparation diagnostics; retained legacy name. |
| `ACCEL_HOMING_CALIBRATE AXIS=X CENTERED=1` | One axis, required `AXIS=X` or `Y`; optional `DETECTOR=RAW|FILTERED`, `START_THRESHOLD`, `STEP`, `SAVE_CONFIG=0|1`; default saves/restarts on success. |
| `ACCEL_HOMING_CALIBRATE_NOISE` | Three stationary one-second noise windows and PNG; no movement, model save or restart. `SAVE_CONFIG` is unsupported. |
| `ACCEL_HOMING_TEST CENTERED=1` | Test X then Y with active thresholds; both must use this plugin. Does not calibrate or save. |
| `ACCEL_HOMING_CHARTS_GENERATE CENTERED=1` | Home/chart X then Y. Optional `AXIS=X` or `Y` selects one; omit for both. |
| `ACCEL_HOMING_FILTER_CHARTS AXIS=X CENTERED=1` | Development comparison: one actual home plus same-trace causal replay; `AXIS=X|Y|XY`, optional `DETECTOR=RAW|FILTERED`, explicit `RAW_THRESHOLD`/`FILTERED_THRESHOLD` in g when no suitable calibrated profile exists. |
| `ACCEL_HOMING_ACCURACY AXIS=X SAMPLES=10 CENTERED=1` | Repeated travel measurement. `AXIS=X|Y|XY`, default `XY`; integer `SAMPLES=2..50`, default 10; optional `DETECTOR=RAW|FILTERED`, `BASELINE=<completed_run_id>`. Recovery can start a saving calibration; see below. |
| `ACCEL_HOMING_BETA_CALIBRATE AXIS=X CENTERED=1` | Experimental moving-noise calibration proposal for X or Y. Optional `START_THRESHOLD`, `STEP`, `SAVE_CONFIG=0`; rejects `SAVE_CONFIG=1`. |
| `ACCEL_HOMING_CANCEL` | Cancel owned workflow/pending preparation and restore temporary settings when handled. Not an emergency stop. |
| `SET_ACCEL_HOMING AXIS=X THRESHOLD=1` | Temporary threshold for X or Y; optional `IMPACT_SIGN=-1|0|1`; until restart. |
| `SET_BEACON_ACCEL_HOMING AXIS=X THRESHOLD=1` | Retained alias of `SET_ACCEL_HOMING`. |

No separate centering-confirmation command or popup prompt is required.
Only one workflow may own the sensor/motion session at a time. A new command
during an active run is rejected; do not reissue it while automatic retries run.

On `codex/accuracy-filters`, TEST and ordinary CHARTS also accept explicit
`DETECTOR=RAW|FILTERED`. The selected mode uses its own saved threshold; test/chart
commands can use an explicit mode-specific `RAW_THRESHOLD` or
`FILTERED_THRESHOLD` instead. These are temporary experimental settings and are
not saved or qualified. Calibration uses `START_THRESHOLD` instead. A RAW
threshold is not silently reused for FILTERED. Omitted DETECTOR retains the
selected model/manual mode. See the [supervised A/B sequence](FILTERS.md#operator-sequence).

## Calibration

```gcode
ACCEL_HOMING_CALIBRATE AXIS=X CENTERED=1
```

Successful default calibration saves X and restarts Klipper. Repeat manual
preparation after that restart before calibrating Y:

```gcode
ACCEL_HOMING_CALIBRATE AXIS=Y CENTERED=1
```

The saved threshold is the candidate that passed ten checks **plus 0.5 g**,
applied once after qualification. Those checks test the candidate; they do not
test the higher saved value. The other axis is unchanged.

Both commands first measure fresh stationary noise. The default start grid is
1 g with 0.25 g steps. `START_THRESHOLD` explicitly chooses the first candidate
but still performs the measurement; `STEP` alone changes grid spacing while
retaining noise-based selection. Starts are finite and at least 0.5 g; step is
finite 0.1–3 g; known-range headroom still applies.

To defer saving:

```gcode
ACCEL_HOMING_CALIBRATE AXIS=X CENTERED=1 SAVE_CONFIG=0
```

This stages a model without writing or restarting; it does not activate it for
ordinary homing. Issue `SAVE_CONFIG` to persist/reload it. Explicit config
thresholds take precedence over ordinary saved models. Read the
[complete calibration sequence](SIMPLE_CALIBRATION.md), including early-hit
automatic returns and the ten-home, at-most-2-mm spread rule.

## Accuracy and early-trigger recovery

```gcode
ACCEL_HOMING_ACCURACY AXIS=X SAMPLES=10 CENTERED=1
```

This measures repeated commanded travel at the active threshold and speed.
An early trigger with valid current capture/halt/center evidence reverses its
recorded commanded halt travel, reports failure in the console without an
accuracy PNG and starts one
normal calibration of the failing axis. Its first candidate is the actual
threshold used by the failed attempt **plus 0.25 g**; later steps use configured
`calibration_step`. Successful recovery calibration saves its qualified
candidate plus the normal 0.5 g final margin and restarts Klipper.
The console then tells you to prepare XY again and rerun the exact accuracy
command; it does not rerun accuracy automatically.

`SAVE_CONFIG` is not an ACCURACY option: even `SAVE_CONFIG=0` is rejected.
Read [accuracy and its recovery limits](ACCURACY.md) before running it.

## Stationary noise and charts

```gcode
ACCEL_HOMING_CALIBRATE_NOISE
```

NOISE leaves motor enable/current and fan state unchanged. Keep the printer
stationary; it needs an idle, unpaused machine, an empty motion queue and an
exclusive usable sensor stream. Its mapped component hints do not configure
ordinary homing and are not reused by later calibration.

```gcode
ACCEL_HOMING_CHARTS_GENERATE AXIS=X CENTERED=1
```

CHARTS moves the selected axis; omitting AXIS moves X then Y. It uses the active
detector/threshold and does not calibrate them. See [plot output](PLOTS.md).

## Temporary threshold changes

```gcode
SET_ACCEL_HOMING AXIS=X THRESHOLD=1
```

This changes the active threshold until restart, without saving a model.
Thresholds must be finite and positive and satisfy known sensor-range limits.
The current model/manual detector context remains relevant; the command does
not turn a legacy threshold into a newly qualified XYZ model.
Cancel an active workflow before changing settings.

For persistent manual changes, edit the existing plugin section and issue
`RESTART` while idle. See [threshold precedence](CONFIGURATION.md#thresholds-and-saved-models).

## Cancellation and failures

`ACCEL_HOMING_CANCEL` is a normal console command. Frontend/console scheduling
can delay it while another command is executing. Use the accessible emergency
stop for movement that must stop immediately.

On an abort, inspect the message and STATUS. Do not assume the toolhead remains
centered, that a model was saved or that a final release occurred. Correct the
reported fault and manually prepare XY before a new moving command. Shutdown
or stock homing fault handling can turn off motors, including Z.
