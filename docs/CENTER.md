# Development coordinate calibration

`ACCEL_HOMING_CALIBRATE_CENTER` estimates X/Y travel, proposes consistent axis
coordinates and reports the inferred midpoint. It is available on the development
branch. This command does not run the existing threshold calibration.

```gcode
ACCEL_HOMING_CALIBRATE_CENTER AXIS=X CENTERED=1
ACCEL_HOMING_CALIBRATE_CENTER AXIS=Y CENTERED=1
ACCEL_HOMING_CALIBRATE_CENTER AXIS=ALL CENTERED=1
```

Omitting `AXIS` selects X and Y, in that order. `AXIS=XY` is an alias for `ALL`.
The printer must be idle, both X and Y must be unhomed, and the operator must
manually place XY approximately at the configured center before passing
`CENTERED=1`. Z and extruder coordinates are retained. Existing workflow
restrictions apply: Cartesian or CoreXY with independent Z, no dual carriage,
independently controllable XY enable lines, selected rails using this module's
accelerometer endstop, `homing_retract_dist: 0`, and `min_home_dist: 0` where the
firmware exposes that option.

Native motion integration requires the `Homing(printer)` and
`HomingMove(printer, endstops)` constructor forms. Kalico `main` revision
[`991e1823`](https://github.com/KalicoCrew/kalico/blob/991e1823e983a31d26b9264d2b6305fd64e3a09a/klippy/extras/homing.py)
requires additional constructor arguments and is not supported by this workflow.
Compatibility with its config-file save API alone does not establish motion API
compatibility. TRAVEL uses the same native motion integration.

The command performs physical motion only when the operator executes it. No
physical testing is implied by the development implementation or its source
regressions.

## Exactly ten direct traverses

For each selected axis:

1. Home once toward its normal configured endstop to establish the starting
   endpoint. This initial positioning is excluded from the ten measurements.
   An initial impact after 20 mm or less is rejected.
2. Reverse directly toward the opposite end and stop on an accelerometer impact.
   Continue alternating directions until ten one-way traverses are measured:
   five positive and five negative. Ten traverses means five complete cycles.
3. Finish at the final stop. No additional center move is performed.

There are **zero separate backoff, release or retract moves** before, between or
after the traverses, including initial positioning and final completion. The
module's ordinary per-axis back-off values (`back_off_dist`, `x_back_off_dist`, `y_back_off_dist`) are suppressed only for this workflow and
restored during cleanup. `BACKOFF` is not a supported parameter. A stationary
dwell and a new accelerometer reference precede the next search. Final contact
with the stop is intentional; an error does not initiate a return or recovery
move. A subsequent axis begins without repositioning the completed axis.

Searches use the existing configured homing speed, accelerometer threshold and
detector metric. Existing selected-axis absolute detection or saved-model XYZ
magnitude detection remains in use in both directions. The existing absolute
reference-subtracted polarity policy accepts either impact direction; configured
impact-sign values are preserved. Available native homing-current and rail
homing-acceleration helpers are used, together with the module's normal
acceleration cap. No threshold search or automatic speed reduction occurs.

The measured axes are left **unhomed** after cleanup. Ordinary limits and rail
objects are restored; the proposed range is not activated in the current runtime.
Normal homing is required before using those axes again.

## Finite search allowance

Every initial home and traverse has a finite commanded search distance:

```text
search distance = configured position_max - configured position_min + SEARCH_MARGIN
```

`SEARCH_MARGIN` defaults to **20 mm** and must be finite and nonnegative. This is
an explicit, reported commanded-search allowance beyond the configured span;
it is never increased automatically. `SEARCH_MARGIN=0` permits no extra search.
The distance
is announced before the axis starts. The native homing move ends with an error if
no impact occurs within it; existing sensor and watchdog checks remain active.
If the configured span plus allowance is smaller than the travel needed to reach
a trigger, the search fails. An operator can choose a different allowance, for
example:

```gcode
ACCEL_HOMING_CALIBRATE_CENTER AXIS=X CENTERED=1 SEARCH_MARGIN=5
```

That permits five additional commanded millimeters for each search, replacing
the 20 mm default. It does not
establish that the extra distance is physically safe. The workflow never derives
an unverified physical boundary from step counts or silently uses a 1.5x span.
Accepted full traverses must exceed 20 mm and be at least
`configured_span * (1 - calibration_travel_tolerance)`; the existing tolerance is
used as a plausibility check, without changing its setting.

## Measurement and coordinate convention

The primary sample is the **commanded step displacement from the stationary
departure halt to the next accelerometer trigger**. Each sample includes its
axis, direction, signed start-to-trigger and start-to-halt displacement, current
trigger-to-halt overshoot, previous overshoot, and the finite search distance.
The alternate commanded trigger-to-trigger sum is retained separately for
diagnosis. Previous overshoot is **not subtracted from the primary estimate**:
steps commanded after contacting a hard stop can load belts or be skipped.

The sensor detects impacts; step counters are not position encoders. Trigger
latency, impact threshold, belt loading, lost steps and hard-stop compliance can
bias these estimates. The report explicitly includes
`physical_position_verified: false`. Neither the primary estimate nor the
alternate diagnostic establishes physical metrology or physical overshoot.

The final report includes all ten samples, mean, minimum, maximum, spread,
sample standard deviation, five-sample statistics for each direction, and the
difference between directional means. The overall spread must not exceed 2 mm;
otherwise the entire request fails without a coordinate proposal. Repeated
results within 2 mm establish commanded repeatability, not physical accuracy.

The coordinate convention anchors the detected negative endpoint at the
**configured `position_min`**, including nonzero or negative minima:

```text
estimated travel = mean of ten primary samples
proposed position_min = configured position_min  # preserved, never written
proposed position_max = position_min + estimated travel
inferred midpoint = (position_min + proposed position_max) / 2
```

For normal positive-direction homing, the proposed `position_endstop` equals
the proposed `position_max`. For negative-direction homing, it equals the
preserved `position_min`, even if the prior endstop coordinate was offset from
that minimum. Both homing signs use the same minimum-anchored convention.
The midpoint is the inferred midpoint of the axis travel; it does not establish
the center of the print bed.

Per-traverse findings and final per-axis proposals appear in the console.
`ACCEL_HOMING_STATUS` provides the structured report in `last_result`, including
all samples and proposed settings. No PNG or plotting operation is required.

## Report first; explicit persistence

`SAVE_CONFIG` defaults to `0`: no setting is staged or saved, including no model
metadata. To repeat the measurements and request persistence explicitly:

```gcode
ACCEL_HOMING_CALIBRATE_CENTER AXIS=ALL CENTERED=1 SAVE_CONFIG=1
```

There is no command that applies a previous report. `SAVE_CONFIG=1` performs a
fresh complete request. Only after every selected axis and all sensor, current,
acceleration, rail and coordinate-limit cleanup succeeds does it stage the
selected axes' changed `position_max` and `position_endstop`. No partial-axis
proposal is published or saved if another selected axis fails. Thresholds,
configured speeds, Z, bed macros and ordinary calibration behavior are preserved.

A saved model whose threshold will become active on restart also needs its
exact geometry context updated. The command stages only that selected axis's
model JSON metadata: new geometry and coordinate-measurement provenance. It
retains the original calibration geometry and other audit fields, threshold and
success count. A temporary `SET_ACCEL_HOMING` override does not exempt the saved
model because that override disappears at restart. Provenance records the
threshold and metric used for this coordinate measurement and states that the
original threshold calibration was not rerun. The existing exact geometry
validation on startup remains enabled.

Persistence preflight refuses unrelated pending `SAVE_CONFIG` changes, and
checks again immediately before staging. Supported native config parsing is
required. Nested and wildcard includes, split sections and repeated options
are resolved in native order. Config changes detected during measurement,
including include comments, new wildcard matches or symlink targets, prevent
automatic saving.

For `SAVE_CONFIG=1`, changed selected coordinates/model metadata originally
defined in includes are migrated to the main native save block: every duplicate
of those selected keys is commented in its included file. Other settings and
comments remain intact, and Kalico's global `autosave_includes` setting is not
enabled. No included file is modified in report-only mode or during measurement.

The command invokes the native configfile `SAVE_CONFIG` implementation directly,
without executing a registered `SAVE_CONFIG` macro. The firmware writes the
normal save block/backups and requests a restart. Include migration first backs
up the original main/include bytes in a hidden `.accel-homing-save-*` directory
beside the main config; its manifest and the reported result identify the files.
Included symlinks are preserved. For include migration, a symlinked main config
or hard-linked target file requires report-only/manual persistence.

Staging errors restore prior pending settings. A reported native failure also
attempts to restore original files; an unrecognized concurrent edit is preserved
and reported as an incomplete rollback. Backups remain available for recovery.
Individual include replacements are atomic, but the multi-file save is not a
power-loss/process-crash transaction. Any failure is reported as `save_failed`,
with restored or unconfirmed disk state; inspect that result before proceeding.

`ACCEL_HOMING_CANCEL`, disconnect, shutdown, sensor faults, pressed starts,
missing or incomplete stepper evidence, implausible travel and failed
repeatability checks abort the workflow. An armed search receives a host stop
request and native move cleanup runs before settings are restored; cancellation
does not issue a backoff or another homing attempt.
