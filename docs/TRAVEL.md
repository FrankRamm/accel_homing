# Guided travel and bed-origin calibration (dev)

`ACCEL_HOMING_TRAVEL` opens a supervised X/Y wizard. It detects the negative
and positive endpoints of one axis using the existing accelerometer homing
detector, lets you align the nozzle with your chosen bed-zero line using slow
jogs, and proposes `position_min`, `position_max` and `position_endstop`.
Repeat for the other axis and save both together with one Klipper restart.
The existing threshold-calibration and CENTER commands keep their behavior.

This is an experimental, offline-tested workflow. Physical impacts, clearance,
fit and accuracy have not been validated on a printer by this change. Motor
step counts are not encoder measurements. A detected vibration can be a false
impact, and missed steps, belt loading and trigger latency affect the result.
The output explicitly marks physical position as unverified.

## Install and prepare

Install/update the complete `dev` checkout with `install.sh --branch dev`,
including its new `accel_homing_travel.py` extras link, then restart Klipper as
instructed by the installer. A source-only Git/Moonraker update of an older
installation may lack that link: existing commands still load, but the wizard
asks you to rerun the installer. No configuration section is required for the
wizard; the optional [macro file](../examples/travel_calibration.cfg) adds a
named entry button only.

1. Keep the printer idle and unpaused. Use Cartesian/CoreXY kinematics with
   independent Z and this module's accelerometer endstop on the selected axis.
   `homing_retract_dist` and `min_home_dist` must be zero.
2. Establish safe nozzle clearance and support Z. Release XY appropriately for
   your machine, leave **both axes unhomed**, and place **both axes approximately
   midway by hand before starting**. Exact midpoint is unnecessary. Do not use
   a general motor-off command without accounting for Z support.
3. Prepare suitable driver currents and enable settings. The wizard bypasses
   G28 wrappers, homing overrides and safe Z hops; it never requests a Z move.
4. Confirm the configured travel is a reasonable estimate. Each search is
   bounded by the configured span plus `SEARCH_MARGIN` (default 20 mm, allowed
   0–100 mm); total per-search allowance must not exceed 1000 mm. This bound
   limits the command; it does not prove that the path is physically clear.
5. Keep an emergency stop accessible and supervise the entire sequence. An
   accelerometer cannot determine that the carriage is already pressed against
   a hard stop. There is **no blind initial clearance move**. Stop and prepare
   again if the carriage is not clearly free of both stops.

**Searches retain the configured homing speed, acceleration and threshold.**
Changing search speed would invalidate the existing impact-detection context;
very slow impacts can be too weak to detect. Only alignment/recenter jogs are
slow: default 10 mm/s, configurable with `JOG_SPEED=1..20`. Jog acceleration is
at most 500 mm/s² and never exceeds the prior live limit. The wizard neither
calibrates a threshold nor claims that any configured threshold is physically
qualified; establish appropriate detector settings separately before use.

## Use the wizard

In the printer console:

```gcode
ACCEL_HOMING_TRAVEL
```

On Mainsail/Fluidd versions supporting `action:prompt_*`, a dialog presents the
current step and buttons. The module emits the protocol directly; `[respond]`
is not required. All instructions and equivalent commands also appear in the
console. Use `PROMPT=0` to suppress dialogs. The dialog's close/X control only
hides it; it does not cancel. Run the same bare command again, or use
`ACCEL_HOMING_TRAVEL ACTION=STATUS`, to reopen the current step without moving.

Select X or Y, then confirm approximate center and clearance. This confirmation
authorizes **two searches: negative first, then positive**. The initial negative
impact must occur after more than 20 mm; a premature/missing/invalid impact
aborts immediately without an automatic recovery move. The inferred total
span must fit the configured plausibility band (`calibration_travel_tolerance`)
and explicit search allowance. A mistaken configured size can reject a real
travel; inspect it and restart instead of treating that rejection as proof of
the mechanical limit.

After both searches, the carriage is at the positive stop. The first offered
jog moves inward. Keep motors energized and use the wizard's selected-axis
jogs until the nozzle aligns with your desired bed-zero line. **Do not move
the carriage by hand or use the ordinary movement controls after measurement.**
The wizard keeps XY unhomed between commands and grants temporary motion limits
only for its own bounded jog. Each jog is at most 10 mm and must end at least
1 mm inside the estimated trigger endpoints. This 1 mm command margin is not
physically verified clearance.

Accept the origin and review the proposed values. You can save this axis alone,
or choose **Center X; select Y** (or the reverse). This recenters only the axis
just measured, in completed segments no larger than 10 mm, preserving its
accepted proposal and keeping motors enabled. The other axis must still be at
its initially prepared approximate center; confirm clearance again before its
search. After the second origin is accepted, save both axes together.

This version requires the chosen zero to be at least 1 mm inside the estimated
travel. It cannot place zero exactly at a mechanical endpoint, and the proposed
`position_min` is negative. It does not discover bed size, probe reach or mesh
area, and does not modify parking, purge, mesh or other movement macros. Review
those separately against the resulting coordinates.

## Equivalent console sequence

The example distances below are commands to choose interactively, not a script
to run blindly. Repeat a jog only while observing the nozzle and remaining
estimated range.

```gcode
ACCEL_HOMING_TRAVEL PROMPT=0 JOG_SPEED=10 SEARCH_MARGIN=20
ACCEL_HOMING_TRAVEL ACTION=SELECT AXIS=X
ACCEL_HOMING_TRAVEL ACTION=MEASURE CENTERED=1 CLEAR=1
ACCEL_HOMING_TRAVEL ACTION=JOG DISTANCE=-10
# Continue valid +/-10, +/-1 or +/-0.1 mm jogs to your chosen X bed zero.
ACCEL_HOMING_TRAVEL ACTION=ORIGIN CONFIRM=1
ACCEL_HOMING_TRAVEL ACTION=NEXT
ACCEL_HOMING_TRAVEL ACTION=MEASURE CENTERED=1 CLEAR=1
ACCEL_HOMING_TRAVEL ACTION=JOG DISTANCE=-10
# Align Y bed zero using valid selected-axis jogs.
ACCEL_HOMING_TRAVEL ACTION=ORIGIN CONFIRM=1
# Review both displayed stepper proposals, then choose one:
ACCEL_HOMING_TRAVEL ACTION=SAVE CONFIRM=1
# Alternatively, ACTION=DONE ends without staging or saving settings.
```

`CENTERED=1 CLEAR=1` is an operator assertion, not a measurement or automatic
centering action. Once a session starts, parameters cannot be changed; cancel
and start over to change `JOG_SPEED`, `SEARCH_MARGIN` or detector settings.

## Coordinates and persistence

The temporary measurement frame anchors the negative **trigger** at zero.
The negative trigger-to-halt commanded overshoot and subsequent positive
start-to-trigger travel establish the positive trigger in that same frame.
The positive halt coordinate includes its own trigger-to-halt commanded
overshoot. Dedicated jogs are checked against all steppers using the native
kinematics conversion (both A/B motors for CoreXY).

For negative trigger `n`, positive trigger `p`, and operator-selected origin
coordinate `o`, the proposal is:

```text
position_min     = n - o
position_max     = p - o
position_endstop = position_max  (configured positive homing direction)
                = position_min  (configured negative homing direction)
```

Trigger and halt are distinct commanded-step coordinates; adding overshoot
does not prove that the carriage physically moved after impact. Both raw
search records and the alternative halt-to-positive-trigger distance remain
in the result. Unlike CENTER's ten-traverse halt-to-trigger estimator, this
wizard uses a common trigger-coordinate frame and only one search per endpoint.
There is no repeatability qualification of the travel result.

Nothing is staged until `ACTION=SAVE CONFIRM=1`. Saving uses native
`SAVE_CONFIG` to write the accepted axes' three values together and restart.
An active saved threshold model has its geometry metadata rebased while
retaining original calibration provenance; this does not rerun or requalify
threshold calibration. Existing manual thresholds retain precedence.

Before saving, the wizard rejects unrelated pending `SAVE_CONFIG` settings,
changed main/include/autosave contents, and changed include symlink targets.
Native parsing resolves nested and wildcard includes in their configured order.
When selected coordinate or model-metadata keys occur in included files, the
explicit save comments every selected duplicate in those files and writes the
accepted values to the main native `SAVE_CONFIG` block. Other settings,
comments and include ordering are retained. This does
not enable Kalico's global `autosave_includes` setting.

Before any include edit, original main/include bytes are backed up in a hidden
`.accel-homing-save-*` directory beside the main config, with a manifest mapping
backup files to their original paths. The result reports this directory and the
modified include files. Native `SAVE_CONFIG` also creates its usual main-config
backup. Included symlinks are preserved; automatic include migration refuses a
symlinked main config or hard-linked target files.

Staging errors restore previous pending settings. A reported native save failure
also attempts to restore the original files, without overwriting an unrecognized
concurrent edit. Recovery failures are reported as persistence unconfirmed, with
details and retained backups. Individual replacements are atomic; the entire
multi-file save is not atomic across a power loss or process crash. Inspect the
reported backup directory if an interrupted save leaves configuration incomplete.

`ACTION=DONE` closes the wizard without writing or staging configuration.
After any finish/abort, measured axes require normal homing. Inspect the
coordinates and affected macros before subsequently operating the printer.

## Cancellation and invalidation

Use the dialog's explicit **Cancel** button or `ACCEL_HOMING_CANCEL`. Cancellation
is handled at firmware execution boundaries and **is not an emergency stop**.
The ordinary G-code queue may defer a console/button Cancel until the current
command returns. Asynchronous motor/disconnect/reactor interrupts are checked
between recenter segments: the current ordinary jog may finish its segment
(at most 10 mm), and no further segment is submitted once the interruption is
observed. Native searches use their existing sensor/MCU stop lifecycle. Use the
printer's emergency stop for immediate risk.

The wait expires after `calibration_confirm_timeout` (default 300 seconds).
Motor disable (including an individual X/Y/Z stepper), shutdown/disconnect,
external homing or any G-code command error invalidates the pending result.
At every continuation, the wizard also checks toolhead coordinates, all
kinematic MCU step counts, homed limits, motor enables, detector/model and live
motion settings. External `G1`, `FORCE_MOVE`, offsets that change the machine
position, or failed jogs cannot silently preserve a stale reference. No encoder
can detect manual slipping while the controller's step counts remain unchanged;
do not physically disturb the carriage.

Ordinary G28 wrappers can request a Z hop before emitting a native homing event;
the wizard does not rewrite public G28 or other macros. Do not invoke them in
an active wizard. Machine/toolhead coordinates, not G-code offsets, define the
chosen origin. A malformed/out-of-stage wizard command also aborts; reopen with
the bare command or `ACTION=STATUS` only when the session is still active.

Prompt protocol references:
[Mainsail macro prompts](https://github.com/mainsail-crew/gb-docs/blob/main/overview/features/macro-prompts.md)
and [Fluidd macros](https://docs.fluidd.xyz/features/macros).
