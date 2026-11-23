# Simple supervised calibration

[← Documentation](../README.md)

> **What you'll do:** let the plugin learn the right [threshold](glossary.md#threshold) for each axis, one axis at a time, while you watch the printer.
>
> **What you need:** the plugin installed and set up (see [Configuration](configuration.md)), the [accelerometer](glossary.md#accelerometer) set up, Z supported, the printer idle, and your hand on the [emergency stop](glossary.md#emergency-stop).
>
> **Time:** about 10 minutes per axis (rough guide; it depends on how many tries the plugin needs).

[Calibration](glossary.md#calibration) is how the plugin finds the threshold. The threshold is the strength of knock at which the [accelerometer](glossary.md#accelerometer) tells the printer it hit the [endstop](glossary.md#endstop). You stay at the printer the whole time.

## Steps

Use one axis per invocation. `AXIS` selects the printer direction to move,
not an accelerometer channel. The commands are shown in the steps below.

Prepare the printer first. Skipping this can crash the toolhead. The full
checklist is in [commands](commands.md#prepare-every-moving-workflow).

1. **Keep the printer idle and unpaused**, with no queued motion or other
   active homing workflow. (Nothing is printing, nothing is waiting to move,
   and no other homing command from this plugin is running.)
2. **Support Z and establish nozzle/bed clearance for the complete XY travel.**
   (Make sure the nozzle cannot hit the bed anywhere the toolhead can go in X
   and Y, and that the bed or gantry cannot drop.) Do not assume a motor-off
   command affects only XY; use independent Z support.
3. **Leave both X and Y unhomed.** Use your printer's controls to unhome X and Y in the way that suits your machine. Read the warning about motor-off commands first (see the [checklist](commands.md#prepare-every-moving-workflow)). Then move the toolhead to the middle by hand.
4. **Prepare XY currents/enables yourself if your G28 override normally does
   this.** These commands skip that override and its Z hops. (If your homing
   macro normally switches motor current or enables the motors, do that first.)
   Supervise with an accessible [emergency stop](glossary.md#emergency-stop).
5. **Calibrate X.** Type this into the Console in Mainsail or Fluidd and send it:

   ```gcode
   ACCEL_HOMING_CALIBRATE AXIS=X CENTERED=1
   ```

   `CENTERED=1` authorizes automatic mapping strokes when needed, as well as
   subsequent homing. Automatic mapping strokes are gentle, short movements
   (at most 15 mm each) that let the plugin work out which sensor channel
   matches this axis. Only add `CENTERED=1` when steps 1 to 4 are done.
6. **Watch what happens.** The plugin may first make small mapping strokes
   (at most 15 mm; see Details). Then the axis makes a discovery approach into
   its stop, followed by ten validation trips. When it succeeds, it saves the
   result and restarts Klipper (this is the default).
7. **Repeat the preparation (steps 1 to 4) after that restart**, including
   centering the toolhead by hand again. Then calibrate Y:

   ```gcode
   ACCEL_HOMING_CALIBRATE AXIS=Y CENTERED=1
   ```
8. **Test homing.** Repeat the preparation (steps 1 to 4) first. A restart
   only happens with the default saving, so wait for Klipper to be ready if it
   restarted. Then run:

   ```gcode
   ACCEL_HOMING_TEST CENTERED=1
   ```

   This tests X then Y with the active thresholds. It does not calibrate or
   save. See the [command reference](commands.md).

## If something goes wrong

- **Stop immediately** with the emergency stop if the toolhead does anything unexpected.
- Calibration stopped with an error message: see [Troubleshooting](troubleshooting.md).
- After an abort, physically center XY again, then run the command again (see "Travel plausibility before midpoint returns" under Details).

## Details

The sections below are the full technical description.

### Automatic physical-axis mapping

The plugin has to know which accelerometer channel matches which printer direction.

Calibration first checks the **selected sensor's loaded configuration**,
including included files and named sensor sections. An explicit Beacon
`accel_axes_map` or other accelerometer's `axes_map` skips automatic mapping.
The configured stream's X channel is then used for physical X and Y for
physical Y. An explicit `x,y,z` counts as configured; the driver's identity
default when the option is omitted does not. The declared map must be a valid
signed permutation and agree with the map applied by the driver. This path
trusts the user's orientation setting; it does not measure or verify mounting.

When the selected sensor has no explicit map, calibration makes gentle, short
central strokes in both directions on the selected machine axis. It uses the signed motion
response to identify one configured-stream sensor channel and its sign.
Each outward stroke is at most 15 mm, with a commanded return to the manual
center between directions; speed is at most 20 mm/s and acceleration at most
500 mm/s², respecting lower live limits. Configured homing speed is unchanged.
This measurement is separate from endstop impacts. An absent, weak, ambiguous
or inconsistent response aborts before noise-based threshold selection or homing;
there is no fallback to whichever channel has the largest impact peak.

The mapping acts on the sensor stream **after** the sensor's configured
`accel_axes_map` or `axes_map`; the plugin does not rewrite that configuration
or apply it twice.
The result is stored with the selected axis model and its sensor context.
Repeat calibration after changing the sensor, orientation, axes map or other
reported context. Automatic mapping checks a consistent measured response to
commanded motion; it does not independently verify physical position or clearance.
Neither mapping path moves machine Z. Fresh noise measurement and threshold
qualification still run when an explicit configuration skips the mapping strokes.

Recalibrate both axes after remounting the sensor. If a successful calibration
detects that the previous orientation changed or conflicts with the other
axis's saved channel, it invalidates that other profile until it is recalibrated.
It retains the old evidence and threshold, but ordinary homing cannot use the
invalidated profile, including with a manual threshold override. A failed
calibration does not stage this invalidation. With `SAVE_CONFIG=0`, new mappings
remain pending; run `SAVE_CONFIG` and restart before ordinary homing. Successive
per-axis calibrations consult the pending pair so the second calibration does
not overwrite the first one's newly qualified mapping.

### Stationary noise after mapping

The plugin first listens to the resting printer to measure its background vibration.

After mapping and settling, the command acquires three disjoint one-second
stationary-noise windows. This runs once before the
first candidate home, not before each threshold retry or same-run continuation.
X and Y each get a new measurement, including after a save/restart. A previous
standalone `ACCEL_HOMING_CALIBRATE_NOISE` result is never reused.

The measurement requires an idle, unpaused printer, an empty motion queue and
an exclusive sensor stream. It leaves motor enable/current and fan state as
they are. A failed measurement, changed context or sensor fault aborts calibration
before threshold qualification; it does not fall back to a default threshold. Transient sample
gaps and fresh-data/acquisition timeouts permit only the bounded complete-window retries described in
[the noise policy](../advanced/plots.md#stationary-noise).

The noise summary stays in memory until the calibration qualifies. Rendering
must succeed before the model is staged/saved, and the PNG is published only
after required cleanup and saving succeed. An aborted calibration does not
publish that qualified noise PNG. A mapping inference rejection instead exports
its own diagnostic JSON/PNG/ZIP after successful cleanup; see
[mapping failure evidence](troubleshooting.md#automatic-mapping-interruption).

Without an explicit `START_THRESHOLD`, fresh noise selects the initial
candidate using the same baseline-subtracted mapped component as the detector.
The configured grid starts at 1.00 g with 0.25 g steps by default. Existing
`calibration_start_threshold` and `calibration_step` settings remain the grid
origin and spacing. An explicit `STEP` changes that spacing and still permits
noise-based selection. For an advanced initial-value override:

```gcode
ACCEL_HOMING_CALIBRATE AXIS=X CENTERED=1 START_THRESHOLD=1 STEP=0.25
```

This still performs the fresh noise measurement, but starts at the supplied
1.00 g and then uses 0.25 g increments. Command parameters are fixed for the
run. The noise-selected threshold is an initial candidate, not a calibrated
impact threshold or a certified minimum. Stationary measurements do not cover
motion, motor enabling, hold-to-run current changes or first-step transients.
See the [noise-selection policy and limits](../advanced/plots.md#stationary-noise).

### Detector

The detector is the part that decides when the accelerometer has felt an impact.

Each move acquires a fresh resting XYZ offset (normally 100 ms, at least
50 contiguous samples), separate from the initial three noise windows.
It does not test stationary peak-to-peak quietness. For each sample it evaluates
`abs(sign * (a[source_axis] - baseline[source_axis]))` in g, with the
selected channel/sign and the resting offset fixed for the move. X homing uses
only physical X; Y homing uses only physical Y. Other stream channels remain
available for diagnostics and do not contribute to the ordinary impact metric.
The first sample at or above the candidate threshold requests an immediate
stop. The later recorded impact peak does not delay stopping. The selected sign
preserves physical-axis orientation; taking the absolute value makes the
trigger independent of polarity. Ordinary motion can also cross the
threshold: it is not an independent classifier of mechanical contact.

With the optional [notch filter](../advanced/notch.md) enabled for this homing axis, the
mapped component is filtered after subtracting its stationary reference. Its
absolute value is compared with the threshold. The first filtered crossing requests the stop, without
an additional confirmation window. Raw samples are retained for diagnosis;
filter settings are recorded with the qualified model. A different filter
configuration requires a new calibration for affected saved thresholds.

There is no acceleration-derived velocity integration, noise-envelope motion
proof or startup-motion deadline in this detector. Missing valid deliveries,
future/invalid timestamps, gapped/frozen streams and the full-move travel/time limit still stop with an
error. Host-delivered sensor data does not provide a hard real-time stop bound.
Generic current-window data is not rejected merely for arriving 80 ms or more
after measurement. Its reference and moving detector use receipt liveness,
while cutoffs exclude earlier windows/pre-arm impacts. Native Beacon's age
policy is unchanged. See [timing details](../advanced/technical-reference.md#timing-and-optional-travel-policies).

### Threshold validity and homing attempts

This section lists the allowed threshold values and what happens with repeated attempts.

Calibration starts and saved model thresholds must be finite and at least
0.5 g. `STEP` remains a finite value from 0.1 to 3 g. Calibration has no
configurable g maximum or fixed 15 g maximum. The legacy
`calibration_max_threshold` option is accepted but ignored, with a deprecation
warning, so an existing configuration can still load.

When the sensor range is known, the candidate and final saved threshold must
remain at or below
`0.8 * sensor_full_scale_g`. This existing 80% rule is a conservative software
headroom policy, not the physical saturation point. The active stream range is
checked again before arming. Recognized finite rails and native Beacon clipping
markers remain accepted bounded observations with separate range provenance;
excess amplitude is unknown, as are native markers' true sign and peak.
Invalid/non-finite samples and acquisition/timing faults still abort. An
unknown range prevents automatic noise-based
selection; an explicit, otherwise valid `START_THRESHOLD` can still be used.
Neither value is clamped downward to fit a limit. If the qualified candidate
plus the final 0.5 g margin exceeds that ceiling, calibration aborts without
staging a model, saving configuration or publishing its noise PNG.
Before arming, the derived raw trigger representation must be finite and
positive. This validates numeric
representation rather than adding a configurable g maximum.

Ordinary and Beta calibration have no total homing-attempt limit. The former
`calibration_max_attempts` option is accepted without numeric validation and
ignored, with a deprecation warning. Discovery, validation and retries can
continue beyond 40 homes; every candidate still needs a fresh discovery plus
ten passing validation homes. Cancellation, disconnects, sensor/motion faults
and invalid thresholds still stop the workflow. There is no recovery-count
cap or accumulated recovery-distance budget. The separate bounded stationary
sensor acquisition retries are unchanged.

### Impact discovery and validation

The plugin finds a first impact, then confirms it ten times.

1. Home the selected printer axis toward its mechanical endstop, watching
   acceleration throughout. **Reaching 20 mm alone does not stop the move or
   select a candidate.** Discovery requires a threshold-crossing impact after
   **more than 20 mm** of commanded travel that also passes the travel
   plausibility check below.
2. An impact at **20 mm or less**, or below the expected-distance band defined
   below, stops immediately and is premature. Reverse the selected axis's
   recorded signed start-to-halt displacement, including travel after the
   trigger, toward its **estimated starting point**. After the commanded return
   and settling complete, advance one threshold step and restart discovery with
   the previous candidate and measurements cleared. Exactly 20 mm is still an
   early hit; no further command or manual recentering is needed for an accepted
   recovery.
3. The first plausible impact **beyond 20 mm** selects the candidate. Stock
   axis homing assigns the endstop coordinate before its result is checked.
   After the travel check passes, move **only that printer axis** to its
   configured midpoint `(position_min + position_max) / 2`. The discovery
   distance is excluded from repeatability because its start was manual.
4. At the unchanged threshold, run **ten** center-to-impact trips, returning
   only the calibrated axis to that same configured midpoint between trips.
   Record commanded step distance at each first acceleration threshold crossing
   and check its travel plausibility before another automatic move.
   A premature validation impact rejects the candidate and follows the same
   return and new discovery in step 2. A sensor fault or missing impact
   aborts without a return or an increase in g.
5. After **each** accepted validation impact, calculate the running **maximum
   distance minus minimum distance**, not statistical variance or just
   consecutive differences. If the spread is **greater than 2.00 mm**, reject
   the candidate immediately without waiting for ten checks. Advance one
   threshold step, discard the candidate and its values, recenter the calibrated
   axis, and restart **impact discovery** with an empty window. Passing still
   requires **ten** validation impacts with a spread **at most 2.00 mm**;
   exactly 2.00 mm passes. The new discovery impact also stays outside its
   following ten measurements. All cycles retain the sensor-range headroom
   policy described above.

Discovery and validation use the same native selected-axis homing path,
independently of public G28 macros or wrappers.
There is no standalone 20 mm probe move or successful no-contact endpoint.
Only returns after qualifying impacts target the configured midpoint.
CALIBRATE and ACCURACY share the helper for midpoint returns. These returns use
ordinary acceleration restored after homing, then wait `calibration_settle_time`
(default 0.25 s) before the next acquisition. They do not apply the homing
acceleration cap to midpoint returns. Homing speed and its acceleration cap
are unchanged. No return command changes the other XY coordinate or Z.
Cancellation/failure restores settings; native homing's fatal error handling may
turn off all motors, including Z. Keep Z supported as for ordinary homing faults.

### Travel plausibility before midpoint returns

The plugin checks that each impact happened about as far from the center as expected.

To qualify, discovery and every validation impact beyond 20 mm must agree with the
configured midpoint-to-endstop distance. Let `expected` be the absolute
distance from `(position_min + position_max) / 2` to `position_endstop`.
The recorded commanded trigger travel must fall inside the inclusive band
`expected * (1 - calibration_travel_tolerance)` through
`expected * (1 + calibration_travel_tolerance)`. The existing tolerance defaults
to `0.20` (20%); this is separate from the 2 mm repeatability limit.

The same classification applies to discovery and every validation impact:

- At or below 20 mm, or strictly below the lower edge of the band: premature;
  reverse the recorded start-to-halt displacement and retry as described below.
  Exactly 20 mm never qualifies.
- Beyond 20 mm and inside the inclusive band: plausible; use the existing
  configured-midpoint return and qualification cycle.
- Beyond the upper edge of the band, unless it is at or below 20 mm: abort
  without a return, threshold increase or model write.

For example, `expected = 109.25 mm` and tolerance `0.20` give an inclusive band
of `87.4` through `131.1 mm`. A `51.175 mm` trigger is premature even though it
exceeds 20 mm; the return reverses that attempt's recorded start-to-halt travel,
including any movement after the trigger. Exactly `87.4 mm` is plausible,
whereas a trigger above `131.1 mm` aborts.

An aborted suspect impact invalidates only the selected axis's homed state;
the other XY axis and Z are preserved. Physically center XY and prepare a new
command after an abort. The midpoint return after a plausible impact is
unchanged; recovery does not introduce an independently measured physical
reference.

This is a commanded-travel plausibility check, not a measurement of the manual
starting position or proof of contact with the intended stop. Skipped steps,
slip or an obstruction can still produce plausible recorded travel.

### Automatic early-impact returns

If an impact comes too early, the plugin moves the axis back and tries again with a higher threshold.

For a premature impact, take the selected axis's current coordinate after native
homing and subtract its recorded signed start-to-halt displacement. Command one
move to that target. The displacement includes movement between the threshold
crossing and the halt; it is not just the trigger distance. If 2 mm was
commanded before the halt, command 2 mm in the opposite direction.

The return uses the completed home record from the current attempt and its
finite recorded displacement. The current coordinate and calculated target
must also be finite. It does not compare the return with a predicted endstop
clearance, impose a trigger-to-halt allowance or accumulate a distance budget.
There is no additional test of whether the carriage physically arrived.
Ordinary firmware movement limits and errors still apply to the commanded move.

Before returning, the next threshold must satisfy the existing sensor headroom
policy and at least one monitored-home attempt must remain.
After the return's queued movement and `calibration_settle_time` wait (default
0.25 s) complete, record the return, raise the threshold by STEP, clear the
candidate and its measurements, and restart discovery. Early reversals retain
their homing acceleration cap and existing return speed. They command only the
selected axis, without changing the other XY coordinate or Z.

After returning, clear only the selected axis's homed state before retrying.
The return does not reset coordinates or accept the premature impact as the
endstop. Zero recorded displacement needs no reverse move, but still settles
and advances the candidate. Its triggering home has already used an attempt.

Cancellation, shutdown, a sensor/transport fault, a stale or missing current
home record, or a firmware movement error aborts. A failed or interrupted return
does not increment completed-return telemetry or advance the threshold. There
is no same-run manual-recentering fallback after an abort.

These are commanded steps, not encoder measurements. Skipped steps, slip or a
blocked carriage can make the estimated starting point wrong. Completion of
the queued move does not measure physical arrival.

### Saving and ordinary homing

This section explains what is saved after a successful calibration and how normal homing uses it.

The following saving/margin rules describe ordinary calibration without
`NOTCH=AUTO`. The test-version [automatic-notch mode](../advanced/notch.md#automatic-selection)
first measures independent moving noise and unfiltered reference homes, selects
a fixed filter/threshold, then requires new validation homes. AUTO defaults to
report-only with no staged configuration and saves exactly the tested threshold
only with explicit `SAVE_CONFIG=1`; it adds no untested post-validation margin.

A clean cycle has one discovery impact plus ten endstop checks. `SAVE_CONFIG=1` is still
the default: successful calibration stages the selected axis model and its
`x_threshold` or `y_threshold`, saves both, and restarts Klipper. Existing
selected-axis threshold lines are commented at save time, including those in
included files, and the new setting goes in the generated SAVE_CONFIG block.
The shared `threshold` is preserved for the other axis; the new per-axis value
overrides it for the calibrated axis. The other axis's threshold and saved
model are not changed, except for the documented orientation-conflict invalidation.
`SAVE_CONFIG=0` only stages the values in memory until an explicit save; it does
not comment or write configuration files early. Sequential X/Y calibrations
retain both pending results. Failed qualification stages no replacement.

After the ten checks pass, add **0.5 g once** to the qualified candidate and
stage that value as the selected axis's threshold. For example, ten checks at
1.25 g produce a saved threshold of 1.75 g. Discovery, retries and all ten
checks still use the candidate without this final margin; the higher saved
value was **not tested by those ten homes**. A later `SAVE_CONFIG`, restart or
model reload does not add the margin again. Failed qualification stages no
replacement threshold. Beta proposals are unchanged.

Console output and saved metadata distinguish `tested_threshold_g`,
`saved_threshold_g` and `threshold_margin_g` (0.5), with
`saved_threshold_tested: false`. The final result's `threshold_g` is the saved
value; `candidate_g` remains the tested candidate. Stationary-noise charts
continue to describe their measurement and initial-value hints.

New models record `impact_metric: mapped_axis` and `axis_mapping` with version,
machine axis, source channel, sign, `coordinate_frame: configured_sensor_stream`,
and its selection method. `bidirectional_velocity_v1` records measured confidence
and diagnostics; `configured_axes_map_v1` instead records the explicit sensor
section, option and normalized map, without claiming measured confidence.
Configured-map models require the same explicit declaration after reload.
Previously inferred models remain valid when their sensor context is unchanged.
After reload, ordinary G28, TEST, CHARTS, ACCURACY and TROUBLESHOOT use that axis's mapped
component and offset-only reference. Charts identify the physical axis and
keep the other configured-stream channels visible as diagnostics.
Older XYZ-magnitude qualifications require fresh calibration and cannot supply
a mapped threshold. Successful calibration replaces the selected axis's previous
manual threshold. A later user-written override still takes precedence;
thresholds qualified for different metrics are not interchangeable. The paired
generated setting retains the qualified model's context checks after reload.

New calibration metadata also records `travel_plausibility_required: true`,
`recenter_acceleration_policy: restored_normal_limit` and the configured
settling interval. `physical_position_verified` remains false.

`ACCEL_HOMING_CALIBRATE_NOISE` remains a separate stationary diagnostic. Its
configured-stream X/Y component hints do not select a later calibration's initial
value; that calibration learns its axis and measures fresh mapped-axis noise.
`ACCEL_HOMING_STATUS` reports the discovery/validation phase, threshold,
restart count and current ten-check spread. Its `stationary_noise` summary
identifies the fresh measurement, mapped-axis peak, initial-value source and PNG.
The XYZ-magnitude noise peak is retained as diagnostic evidence.
The nested `recovery` object in active calibration status, the final result and
new saved model metadata contains only completed-return telemetry:

- `count`: completed commanded returns, including a zero-displacement return.
- `last_return_mm_signed`: the last completed signed commanded return, or `null`
  before any return completes.
- `accounting`: `completed_commanded_returns`.
- `physical_position_verified`: `false`.

There is no accumulated distance, count/distance limit or remaining budget,
nominal clearance or margin in this object. Old saved recovery metadata remains
readable as historical metadata; it is not used to limit a new calibration.
Active calibration status and final results keep the diagnostic `attempts`
count. The compatibility fields `attempt_limit` and `attempts_remaining` are
`null`, meaning no total limit or countdown. `candidate_attempts_required`
(11) describes one discovery plus ten validation homes. A final result retains
`candidate_g` and `restarts` if the next candidate is denied.

Beta retains only the newest 16 home captures for its PNG and console
diagnostics, including the final discovery and all ten qualifying checks.
This bounds display storage independently of the number of homing attempts;
it does not discard trial records used to qualify a candidate. Status and final
results expose `beta_home_history`; console diagnostics and the PNG report the
total, retained and omitted capture counts and their actual attempt ranges.
Attempt labels retain their original numbers. Ordinary calibration keeps its
existing trial history.

### Limits of this method

This section lists what the method cannot check.

The operator supplies the initial physical center. There is no encoder feedback:
step travel measures issued motor movement, not confirmed carriage movement.
Returns therefore assume no skipped steps, slipping belts or blocked motion.
Repeatable contact with the wrong obstruction can pass. A plausible impact
beyond 20 mm is a candidate,
not independent proof that the intended endstop was reached. Assigning an
endstop coordinate and returning to its midpoint assumes genuine contact;
supervise actual travel and stop on unexpected behavior. A premature trigger
may arise from vibration, a real obstruction or binding; the workflow does not
automatically diagnose which occurred. The automatic return does not verify
that the carriage physically returned. Evidence must be tied to the exact
revision and environment; see [offline verification](../development/testing.md). Historical
passing tests and publication dates do not establish hardware validation.

Next: [Configuration](configuration.md)
