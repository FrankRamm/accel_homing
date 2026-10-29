# Accuracy workflow and early-trigger recovery

ACCURACY measures variation in **commanded homing travel** at the currently
active thresholds and configured homing speed. It does not measure carriage
position with an encoder or establish absolute physical accuracy.

## Run

Follow the [manual preparation](COMMANDS.md#prepare-every-moving-workflow):
idle and unpaused, both XY axes unhomed and physically centered, Z supported,
nozzle clearance, and suitable XY currents/enables. Supervise the entire run.

```gcode
ACCEL_HOMING_ACCURACY AXIS=X SAMPLES=10 CENTERED=1
```

Use `AXIS=Y` for Y, or `AXIS=XY`/omit AXIS for X then Y. Each requested axis
must use this plugin's endstop. `SAMPLES` is an integer from 2 through 50,
default 10, per axis. `CENTERED=1` authorizes movement without another prompt.

Do not add `SAVE_CONFIG`: ACCURACY rejects the parameter, including zero.
A successful measurement alone does not save/restart. An early-trigger recovery
can start a separate normal calibration that saves and restarts on success.

## Sequence and statistics

For each axis:

1. Qualify one discovery home from the manually confirmed center. Exclude it
   from measured statistics.
2. Return that qualified axis to the configured midpoint.
3. Record the requested number of additional homes at the unchanged active
   threshold and homing speed. Return only that axis to midpoint when another
   home/axis is needed.
4. After the final qualified home, perform the bounded configured final release
   and publish the accuracy summary.

Midpoint returns use ordinary acceleration restored after homing and the same
`calibration_settle_time` wait as CALIBRATE. Homing speed/cap are not lowered
to make the measurement appear more repeatable.

The report includes each accepted home's commanded travel, actual trigger
threshold, measured onset and peak acceleration. It reports minimum, maximum,
mean, range, population standard deviation in mm and population variance in mm².
Range is max minus min; population variance divides by N. Fewer than two
accepted measured homes have no standard deviation/variance. An interrupted run
reports incomplete statistics in the console without saving a chart.

Travel is reported to 0.001 mm from commanded/firmware steps. Small reported
variation does not prove correct-stop contact, no missed steps or a physically
accurate return. This measurement is not a calibration pass criterion. Current
CALIBRATE requires ten qualified checks with range at most 2.00 mm; an existing
1.5 mm advisory in ACCURACY output refers to older qualification behavior.

## Early-trigger automatic recalibration

Recovery is limited to an early trigger with complete **current** capture,
signed trigger/halt step evidence and an intact confirmed starting reference.
A sensor fault, missing/mismatched evidence, lost motor/position state,
cancellation or invalid candidate prevents recovery.

When that evidence is valid, the same command:

1. Reverses the selected axis's recorded signed **start-to-halt** displacement,
   including commanded travel after the trigger. It waits for completion and
   settling. A valid zero-step halt needs no reverse move.
2. Invalidates the rejected XY homing assignment without disabling the motors
   or inventing a new physical center coordinate.
3. Reports the failed accuracy run in the console and discards its bounded
   in-memory capture traces. It saves no accuracy PNG. Required cleanup must
   succeed before the handoff.
4. Starts one standard calibration of the failing axis at the actual threshold
   used by the failed ACCURACY attempt **plus exactly 0.25 g**. This increment
   is fixed; later candidate changes use configured `calibration_step`.
5. Performs normal fresh stationary-noise checks, discovery and ten qualified
   validation homes. It then adds the normal **0.5 g final margin** to the
   qualified candidate and saves only after that calibration succeeds.

For example, a failed attempt using 1.75 g starts recovery calibration at
2.00 g. If that candidate passes, the saved threshold is 2.50 g; those ten homes
tested 2.00 g, not 2.50 g. A temporary SET threshold counts as the effective
threshold; recovery does not assume the configured default or the model's older value.
The 0.5 g calibration minimum, known sensor-range headroom and finite-number
requirements still apply to both candidate and saved threshold. Either value
outside those limits aborts without clamping or saving a replacement.

If a manual shared or axis setting would override the new model after reload,
recovery also stages the failing axis's replacement threshold through native
SAVE_CONFIG. The other axis/shared setting is preserved. Include/save conflicts
are reported as `save_failed`; a passed calibration does not imply persistence
or restart succeeded when SAVE_CONFIG reports an error.

After a successful save and Klipper restart, the console tells you to release
and manually center XY again, then rerun the exact ACCURACY command it prints.
There is no popup confirmation and no automatic accuracy rerun after restart.
Do not issue another accuracy command while recovery is active.

The return uses commanded steps and assumes no slip, skipped steps or blocked
travel. Its recorded completion does not independently verify physical center.
Unexpected physical behavior requires the emergency stop and manual review.

## PNG and cleanup

A successful ACCURACY run keeps one final overview PNG per requested axis in
the configured chart directory. It includes accepted traces and travel/statistics,
with the actual detector threshold. Any failed, rejected or cancelled run saves
no accuracy PNG, including when another requested axis already completed.

Per-home traces and their current-run provenance stay bounded in memory; no
per-home PNG or scratch capture file is written. Required cleanup and every
axis's rendering complete before final publication starts. A later publication
error attempts to remove only this run's new files. Existing chart-directory
outputs are preserved. Partial statistics remain available in console/status.

See [plot locations and access](PLOTS.md) and
[troubleshooting](TROUBLESHOOTING.md). `ACCEL_HOMING_CANCEL` is not an immediate
emergency stop for a running console command.
