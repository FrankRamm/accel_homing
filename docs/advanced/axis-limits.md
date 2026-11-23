# Axis limits and bed-origin calibration

[← Documentation](../README.md)

`ACCEL_HOMING_CALIBRATE_LIMITS` opens a supervised X/Y wizard. It finds the
negative and positive end of one axis with the accelerometer homing detector,
lets you jog the nozzle to your chosen bed-zero line, and proposes
`position_min`, `position_max` and `position_endstop`. Repeat for the other
axis and save both together with one Klipper restart.

This is experimental and has not been tested on a printer. Motor step counts
are not encoder measurements. A detected vibration can be a false impact, and
missed steps, belt loading and trigger latency change the result. The output
marks physical position as unverified.

## Install

The wizard lives in `accel_homing_axis_limits.py`. A new installation links it
automatically. On an existing installation, rerun `install.sh` after updating,
then restart Klipper as the installer says. If you only update the source,
the other commands still work and the wizard asks you to rerun the installer.
No configuration section is needed. The optional
[macro file](../../examples/axis_limits_calibration.cfg) only adds buttons.

## Prepare

1. Keep the printer idle and unpaused, with Cartesian or CoreXY kinematics,
   an independent Z axis, and this plugin's accelerometer endstop on the
   selected axis.
2. Make the nozzle safe and support Z. Release XY, leave **both axes
   unhomed**, and place **both axes roughly midway by hand**. Exact center is
   not needed. Do not use a general motor-off command without supporting Z.
3. Set suitable driver currents and enables. The wizard bypasses G28 macros,
   homing overrides and safe Z hops, and never moves Z.
4. Each search is bounded by the configured axis length plus `SEARCH_MARGIN`
   (default 20 mm, 0 to 100 mm). The total must not exceed 1000 mm. This
   limits the command; it does not prove the path is clear.
5. Keep an emergency stop in reach and watch the whole sequence. The
   accelerometer cannot tell that the carriage already rests on a stop. There
   is no clearance move first. If either stop is not clearly free, stop and
   prepare again.

Searches use the configured homing speed, acceleration and threshold. They
make no retract or back-off moves. Each end is measured with a single pass
at `homing_speed`. If the rail uses `homing_retract_dist` (a second pass at
`second_homing_speed`) or `min_home_dist`, ordinary G28 can trigger slightly
differently. Only the alignment jogs are slow:
`JOG_SPEED` 1 to 20 mm/s, default 10, with acceleration capped at 500 mm/s².
The wizard does not calibrate a threshold; set that up first with
[calibration](../guide/calibration.md).

## Use

```gcode
ACCEL_HOMING_CALIBRATE_LIMITS
```

Mainsail and Fluidd show one dialog that stays open and updates in place at
each step; everything also appears in the console. `PROMPT=0` turns the
dialog off. Closing it does not cancel. Run the bare command again, or
`ACTION=STATUS`, to reopen the current step without moving. A rejected
command (bad parameter, or a button from an earlier step) moves nothing and
leaves the session open.

1. Select X or Y and confirm approximate center and clearance. This allows
   **two searches, negative first, then positive**. The first impact must come
   after more than 20 mm. A missing, early or invalid impact aborts without a
   recovery move. The measured length must fit the plausible range
   (`calibration_travel_tolerance`) and the search allowance. A wrong
   configured size can reject a real length; check it and restart.
2. After the searches the carriage is at the positive stop. Keep motors on and
   use only the wizard's jogs to line the nozzle up with your bed-zero line.
   **Do not push the carriage by hand or use normal movement controls.** Each
   jog is at most 100 mm and must end at least 1 mm inside the measured ends.
   The dialog shows the current coordinate and step buttons -100, -10, -1,
   -0.1 and +0.1, +1, +10, +100 (buttons that would leave the measured range
   are hidden). **Accept bed zero** and **Abort** are in the footer.
3. Accept the origin and review the proposal. Save this axis alone or choose
   **Center X; select Y** (or the reverse). That recenters the measured axis
   in segments of at most 10 mm. The other axis must still be at its
   prepared center; confirm clearance again before its search.
4. After the second origin, save both axes together.

The origin must be at least 1 mm inside the measured length, so `position_min`
is negative and zero cannot sit exactly on a stop. The wizard does not find
bed size, probe reach or mesh area and does not change parking, purge or mesh
macros. Check those against the new coordinates.

## Console sequence

The distances are examples to choose by eye, not a script to run blindly.

```gcode
ACCEL_HOMING_CALIBRATE_LIMITS PROMPT=0 JOG_SPEED=10 SEARCH_MARGIN=20
ACCEL_HOMING_CALIBRATE_LIMITS ACTION=SELECT AXIS=X
ACCEL_HOMING_CALIBRATE_LIMITS ACTION=MEASURE CENTERED=1 CLEAR=1
ACCEL_HOMING_CALIBRATE_LIMITS ACTION=JOG DISTANCE=-10
# Continue valid +/-10, +/-1 or +/-0.1 mm jogs to your X bed zero.
ACCEL_HOMING_CALIBRATE_LIMITS ACTION=ORIGIN CONFIRM=1
ACCEL_HOMING_CALIBRATE_LIMITS ACTION=NEXT
ACCEL_HOMING_CALIBRATE_LIMITS ACTION=MEASURE CENTERED=1 CLEAR=1
ACCEL_HOMING_CALIBRATE_LIMITS ACTION=JOG DISTANCE=-10
# Align Y bed zero with valid jogs.
ACCEL_HOMING_CALIBRATE_LIMITS ACTION=ORIGIN CONFIRM=1
# Review both proposals, then choose one:
ACCEL_HOMING_CALIBRATE_LIMITS ACTION=SAVE CONFIRM=1
# ACTION=DONE ends without staging or saving anything.
```

`CENTERED=1 CLEAR=1` is your statement that the carriage is centered and
clear. It is not a measurement and does not center anything. Parameters
cannot change once a session starts; cancel and start over to change
`JOG_SPEED`, `SEARCH_MARGIN` or detector settings.

## Coordinates and saving

The wizard works in a temporary frame where the negative **trigger** is zero.
The negative trigger-to-halt overshoot and the following positive
start-to-trigger travel put the positive trigger in the same frame. Jogs are
checked against all motors using the printer's own kinematics (both A and B
motors on CoreXY).

With negative trigger `n`, positive trigger `p` and chosen origin `o`:

```text
position_min     = n - o
position_max     = p - o
position_endstop = position_max  (positive homing direction)
                 = position_min  (negative homing direction)
```

Trigger and halt are separate commanded-step positions. Adding overshoot does
not prove the carriage moved after impact. Both raw search records stay in
the result. Each end is searched once, so there is no repeatability check.

Nothing is staged until `ACTION=SAVE CONFIRM=1`. Saving writes the accepted
axes' three values together through native `SAVE_CONFIG` and restarts Klipper.
If an axis has a saved threshold model, its geometry is rebased and the
original calibration record is kept; the threshold is not recalibrated.

Before saving, the wizard refuses unrelated pending `SAVE_CONFIG` changes,
config files that changed since the start, and keys that `SAVE_CONFIG` cannot
replace because they come from an included file. Move those keys into
`printer.cfg` and start again. A native disk-write error is reported as
unconfirmed, never as a successful save. A refused or failed save ends the
session but keeps the full proposal in the result and prints it, so the
measurements are not lost; apply the values by hand or measure again. Both
newer and older Klipper/Kalico configuration layouts are supported.

`ACTION=DONE` writes and stages nothing. After any finish or abort, the
measured axes need normal homing. Check the coordinates and macros before
using the printer.

## Cancel and invalidation

Use the dialog's **Abort** button or `ACCEL_HOMING_CANCEL`. This is not an
emergency stop. Klipper may run it only after the current command returns,
so it takes effect after the current jog move finishes (up to 100 mm, at
`JOG_SPEED`, which is at most 20 mm/s). During a search it runs only after
both searches have finished. Use the printer's emergency stop to halt a
search or in any immediate danger.

The wait expires after `calibration_confirm_timeout` (default 300 seconds).
Disabling any motor, shutdown, disconnect, external homing or any G-code error
discards the result. At every step the wizard also checks toolhead
position, all motor step counts, homed limits, motor enables, detector and
model, and live motion settings. An outside `G1`, `FORCE_MOVE` or offset
change cannot keep a stale reference. Nothing detects a hand-moved carriage
while step counts stay unchanged, so do not touch it.

Do not run G28 macros during the wizard; some request a Z hop first. Machine
coordinates, not G-code offsets, define the origin.

Prompt protocol: [Mainsail macro prompts](https://github.com/mainsail-crew/gb-docs/blob/main/overview/features/macro-prompts.md),
[Fluidd macros](https://docs.fluidd.xyz/features/macros).
