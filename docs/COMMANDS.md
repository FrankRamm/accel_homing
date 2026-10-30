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
| `ACCEL_HOMING_CALIBRATE AXIS=X CENTERED=1` | One axis, required `AXIS=X` or `Y`; optional `DETECTOR=RAW|FILTERED|HYBRID`, default HYBRID; `START_THRESHOLD`, `STEP`, `SAVE_CONFIG=0|1`. HYBRID refreshes RAW with five valid checks and FILTERED with five; one final save/restart after both stages. Pure RAW/FILTERED retain ten checks. |
| `ACCEL_HOMING_CALIBRATE_NOISE` | Three stationary one-second noise windows and PNG; no movement, model save or restart. `SAVE_CONFIG` is unsupported. |
| `ACCEL_HOMING_TEST CENTERED=1` | Test X then Y with active thresholds; both must use this plugin. Does not calibrate or save. |
| `ACCEL_HOMING_CHARTS_GENERATE CENTERED=1` | Home/chart X then Y. Optional `AXIS=X` or `Y` selects one; omit for both. |
| `ACCEL_HOMING_FILTER_CHARTS AXIS=X CENTERED=1` | Development comparison: one actual home plus same-trace causal replay; `AXIS=X|Y|XY`, optional `DETECTOR=RAW|FILTERED`, explicit `RAW_THRESHOLD`/`FILTERED_THRESHOLD` in g when no suitable calibrated profile exists. |
| `ACCEL_HOMING_ACCURACY AXIS=X SAMPLES=10 CENTERED=1` | Repeated travel measurement. `AXIS=X|Y|XY`, default `XY`; integer `SAMPLES=2..50`, default 10; optional `DETECTOR=RAW|FILTERED|HYBRID`, default HYBRID; optional `BASELINE=<completed_run_id>`. RAW/FILTERED early-trigger recovery can start a saving calibration; HYBRID aborts for manual review. |
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
threshold is not silently reused for FILTERED. Omitted DETECTOR uses HYBRID on
CALIBRATE and ACCURACY; TEST, CHARTS and FILTER_CHARTS retain their selected
model/manual-mode defaults. Ordinary G28 is unchanged. No global detector
default configuration key is added. See the [supervised A/B sequence](FILTERS.md#operator-sequence).

TEST, ordinary CHARTS and ACCURACY additionally accept `DETECTOR=HYBRID`.
HYBRID requires the native Beacon packet path and validates both saved
compatible profiles for every requested axis before movement. FILTERED controls
confirmation; RAW supplies its own XYZ threshold
and independent stationary reference for position timing. It rejects
`RAW_THRESHOLD`/`FILTERED_THRESHOLD` overrides and unsupported tuning parameters.
These saved-profile preconditions apply to HYBRID measurement. Default or
explicit HYBRID CALIBRATE instead obtains a fresh qualified pair as described
below. There is no persistent HYBRID ordinary-homing model selection.
CALIBRATE_NOISE and FILTER_CHARTS reject HYBRID. TEST/ordinary CHARTS require
explicit `DETECTOR=HYBRID`; ACCURACY selects it by default.
TEST always requests X then Y, so HYBRID TEST needs both saved profiles on both
axes. Use explicit `AXIS=X` with CHARTS or ACCURACY for an X-only experiment.
Read [the HYBRID selection/fallback rules](FILTERS.md#hybrid-position-timing) and
[copyable 30-sample commands](ACCURACY.md#compare-30-raw-pure-filtered-and-hybrid-homes).

## Calibration

```gcode
ACCEL_HOMING_CALIBRATE AXIS=X CENTERED=1
```

Successful default calibration saves X and restarts Klipper. Repeat manual
preparation after that restart before calibrating Y:

```gcode
ACCEL_HOMING_CALIBRATE AXIS=Y CENTERED=1
```

The default is native-Beacon paired HYBRID calibration. Other supported sensor
backends must request pure RAW or FILTERED explicitly. HYBRID uses RAW discovery plus five valid RAW
checks, automatic recentering, then FILTERED discovery plus five valid FILTERED
checks. Each stage uses its actual controlling detector; there is no third
HYBRID-position qualification stage or manual transition between series.
Discovery, rejected impacts and retries
are excluded from each five-check result. The first profile stays private until
both stages and cleanup succeed; default saving then writes the complete pair
and restarts once. Paired HYBRID preserves the other axis and ordinary G28's
selected RAW/FILTERED type; explicit pure calibration selects its requested mode.

A recoverable early hit or excessive spread in either series discards both
series and private results, resets RAW/FILTERED counts to zero, and restarts
from RAW after increasing only the failing detector's candidate by STEP.
The global attempt count is not reset. Fatal stream, context, geometry and
cancellation errors abort. No partial-success profile is reused or saved.

Each saved threshold is its stage's qualified candidate **plus 0.5 g**, applied
once. Those checks test the candidate, not the higher saved value. Explicit
`DETECTOR=RAW` or `FILTERED` qualifies ten checks for that one mode. HYBRID uses
the existing shared attempt budget, normally 40 homes. A clean pair needs at
least 12 monitored homes, and every whole-pair retry requires at least 12
remaining attempts. The global counter never resets; discovery/retries add
homes beyond the ten valid checks.

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
automatic returns and each stage's at-most-2-mm spread rule.

## Accuracy and early-trigger recovery

```gcode
ACCEL_HOMING_ACCURACY AXIS=X SAMPLES=10 CENTERED=1
```

This measures repeated commanded travel at the active threshold and speed.
For RAW or pure FILTERED, an early trigger with valid current
capture/halt/center evidence reverses its
recorded commanded halt travel, reports failure in the console without an
accuracy PNG and starts one
normal calibration of the failing axis. Its first candidate is the actual
threshold used by the failed attempt **plus 0.25 g**; later steps use configured
`calibration_step`. Successful recovery calibration saves its qualified
candidate plus the normal 0.5 g final margin and restarts Klipper.
The console then tells you to prepare XY again and rerun the exact accuracy
command; it does not rerun accuracy automatically.

HYBRID ACCURACY early-quality failure instead aborts with diagnostics and requires
operator inspection and manual XY preparation. It performs no automatic return
or recovery calibration, so existing profiles are not recalibrated by that path.

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
Native Beacon ignores IMPACT_SIGN as an impact-direction gate and uses
absolute centered acceleration/XYZ magnitude; original signed protocol counts
remain unchanged. Generic-backend sign policies are unchanged.
A successful native RAW SET with a valid THRESHOLD explicitly declares that
exact current value as an unqualified temporary manual threshold. It preserves
the selected detector type and saved profiles rather than claiming that an old
selected-axis model was requalified. An IMPACT_SIGN-only command or invalid
THRESHOLD does not authorize this temporary path. Native signed-v1 FILTERED
remains rejected; SET never converts it to rectified v2.
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
