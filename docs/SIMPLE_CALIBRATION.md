# Simple supervised calibration

Use one automatic command for the selected axis:

```gcode
ACCEL_HOMING_CALIBRATE AXIS=X CENTERED=1
```

Use `AXIS=Y` for Y in a separate invocation, with normal preparation after any
saving restart. Use one axis per invocation. `AXIS` selects the printer direction to move,
not an accelerometer channel. Physically center XY, leave both axes unhomed,
keep Z supported/clear, and supervise with an accessible emergency stop.
`CENTERED=1` authorizes movement after stationary preparation succeeds.

## Default HYBRID pair and explicit overrides

Omitted `DETECTOR` now means HYBRID on CALIBRATE. Explicit
`DETECTOR=RAW|FILTERED|HYBRID` remains available. HYBRID calibration always
refreshes both profiles in the same invocation; existing profiles do not skip
any qualification stage.

HYBRID currently requires the native Beacon backend. Other supported sensors
must explicitly request RAW or FILTERED; an unsupported default is rejected
before movement.

The automatic series are:

| Series | Actual controlling detector | Excluded discovery | Required valid checks |
| --- | --- | --- | --- |
| First | RAW XYZ magnitude | One fresh accepted discovery | Five |
| Second | FILTERED | One fresh accepted discovery | Five |

The transition and recentering are automatic. The operator performs no manual
action between these series. There is no third HYBRID-position qualification
series. Discovery, rejected homes and retries are outside each five-check
result. A clean pair therefore requires at least 12 monitored homes.

If either series has a recoverable early hit or excessive spread, discard both
series and their private results, reset both valid counts to zero, increase
only the failing detector's candidate by STEP and restart from RAW. A completed
RAW series cannot be reused after a FILTERED failure. The shared attempt budget
never resets; insufficient remaining budget aborts. Fatal stream, context,
geometry and cancellation errors abort instead of starting this retry.

Only both qualified series and successful cleanup permit the final pair to be
staged/saved. No partial-success series is saved. Each final threshold receives
the normal +0.5 g margin once. Paired HYBRID preserves the pre-calibration ordinary G28 selected
RAW/FILTERED type and the other axis are preserved. Explicit pure RAW or FILTERED
calibration retains one discovery plus ten valid checks for that mode.
ACCURACY also defaults to HYBRID; other command defaults and ordinary G28 do not
gain a HYBRID default or a new global config key.

## Stationary noise before movement

Every new per-axis calibration command first acquires three disjoint one-second
stationary-noise windows. This runs once before the
first candidate home, not before each threshold retry or same-run continuation.
X and Y each get a new measurement, including after a save/restart. A previous
standalone `ACCEL_HOMING_CALIBRATE_NOISE` result is never reused.

The measurement requires an idle, unpaused printer, an empty motion queue and
an exclusive sensor stream. It leaves motor enable/current and fan state as
they are. A failed measurement, changed context or sensor fault aborts calibration
before movement; it does not fall back to a default threshold. Transient sample
gaps and fresh-data/acquisition timeouts permit only the bounded complete-window retries described in
[the noise policy](PLOTS.md#stationary-noise).

The noise summary stays in memory until the calibration qualifies. Rendering
must succeed before the model is staged/saved, and the PNG is published only
after required cleanup and saving succeed. An aborted calibration saves no PNG.

Without an explicit `START_THRESHOLD`, fresh noise selects the initial
candidate for each detector's metric. Paired HYBRID uses the same initial
stationary capture to calculate independent RAW and FILTERED starting hints;
the metrics and thresholds are not interchangeable.
The configured grid starts at 1.00 g with 0.25 g steps by default. Existing
`calibration_start_threshold` and `calibration_step` settings remain the grid
origin and spacing. An explicit `STEP` changes that spacing and still permits
noise-based selection. For an advanced initial-value override:

```gcode
ACCEL_HOMING_CALIBRATE AXIS=X CENTERED=1 START_THRESHOLD=1 STEP=0.25
```

This still performs the fresh noise measurement, but starts each requested
mode's search at the supplied 1.00 g and uses 0.25 g increments for a failing
candidate. Paired mode keeps the two candidate values independent. Command parameters are fixed for the
run. The noise-selected threshold is an initial candidate, not a calibrated
impact threshold or a certified minimum. Stationary measurements do not cover
motion, motor enabling, hold-to-run current changes or first-step transients.
See the [noise-selection policy and limits](PLOTS.md#stationary-noise).

## Detector

RAW stage moves acquire a fresh resting XYZ offset (normally 100 ms, at least
50 contiguous samples), separate from the initial three noise windows.
It does not test stationary peak-to-peak quietness. For each sample it evaluates
`sqrt((ax-bx)^2 + (ay-by)^2 + (az-bz)^2)` in g, with the offset fixed for the move.
This is sign-independent: each axis's centered acceleration contributes its
absolute magnitude, regardless of the encoded sign. The stationary reference
is subtracted before taking absolute values; uncentered gravity is not rectified
into an impact. Original signed native samples remain unchanged for decoding,
reference calculation and provenance. Native marker estimates remain labelled;
they do not recover the original amplitude or true sign.
The first sample at or above the candidate threshold requests an immediate
stop. The later recorded impact peak does not delay stopping. Channel
permutations and sign reversals do not change this metric; no homing-axis
mapping or impact-sign choice is needed. Ordinary motion can also cross the
threshold: it is not an independent classifier of mechanical contact.

FILTERED stage moves use their actual causal rectified-v2 detector, robust
stationary reference, noise weights and temporal/energy confirmation. Their
per-axis impact inputs are `abs(a - reference)` before the causal filter; new
paired RAW/FILTERED decisions do not use an impact-direction sign gate. Their
position reference is the accepted FILTERED threshold crossing. These are
FILTERED-controlled qualification homes, not HYBRID-position measurements.
See [the detector metrics](FILTERS.md#what-changes). The future HYBRID accuracy
measurement combines the two saved profiles under its separate association
policy; paired calibration does not claim every qualification home used that
position policy.

There is no acceleration-derived velocity integration, noise-envelope motion
proof or startup-motion deadline in this detector. Missing, stale, invalid,
gapped/frozen streams and the full-move travel/time limit still stop with an
error. Host-delivered sensor data does not provide a hard real-time stop bound.

## Threshold and attempt bounds

Calibration starts and saved model thresholds must be finite and at least
0.5 g. `STEP` remains a finite value from 0.1 to 3 g. Calibration has no
configurable g maximum or fixed 15 g maximum. The legacy
`calibration_max_threshold` option is accepted but ignored, with a deprecation
warning, so an existing configuration can still load.

When the sensor range is known, the candidate and final saved threshold must
remain at or below
`0.8 * sensor_full_scale_g`. This existing 80% rule is a conservative software
headroom policy, not the physical saturation point. The active stream range is
checked again before arming; sensor saturation, invalid data and other fault
checks remain separate. An unknown range prevents automatic noise-based
selection; an explicit, otherwise valid `START_THRESHOLD` can still be used.
Neither value is clamped downward to fit a limit. If the qualified candidate
plus the final 0.5 g margin exceeds that ceiling, calibration aborts without
staging a model, saving configuration or publishing its noise PNG.
Before arming, the derived raw trigger limit and its squared value used by the
XYZ-magnitude detector must both be finite and positive. This validates numeric
representation rather than adding a configurable g maximum.

The existing monitored-home budget, 40 attempts by default, includes discovery,
validation and all retries across the entire command. It never resets between
series or pair retries. A new paired HYBRID sequence/retry requires at least
**12 attempts remaining**: discovery plus five valid homes for RAW and the same
for FILTERED. The RAW series reserves the minimum future FILTERED budget.
Explicit pure RAW/FILTERED candidate retries require **11 attempts remaining**:
one fresh discovery plus ten validation homes.
This is checked before a recovery return, threshold increase or recentering.
There is no separate recovery-count cap or accumulated recovery-distance
budget; the monitored-home attempt budget still bounds retries.

## Impact discovery and validation

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
4. At the unchanged threshold, run the current series's required number of
   center-to-impact trips: **five** in paired HYBRID, **ten** in explicit pure
   RAW/FILTERED. Return
   only the calibrated axis to that same configured midpoint between trips.
   Record commanded step distance at each accepted detector position timestamp
   and check its travel plausibility before another automatic move.
   A premature validation impact rejects the candidate and follows the same
   return and new discovery in step 2. A sensor fault or missing impact
   aborts without a return or an increase in g.
5. After **each** accepted validation impact, calculate the running **maximum
   distance minus minimum distance**, not statistical variance or just
   consecutive differences. If the spread is **greater than 2.00 mm**, reject
   the candidate immediately without waiting for all required checks. Advance one
   threshold step, discard the candidate and its values, recenter the calibrated
   axis, and restart **impact discovery** with an empty window. Passing still
   requires the series's **five or ten** validation impacts with a spread **at most 2.00 mm**;
   exactly 2.00 mm passes. The new discovery impact also stays outside its
   following measured series. In paired HYBRID, any recoverable early-hit/spread
   rejection discards both series and starts again from RAW with both counts at
   zero; only the failing detector's candidate increases. All cycles share the attempt budget and
   sensor-range headroom policy described above.
6. In paired HYBRID, a qualified RAW series is retained privately while the
   workflow automatically recenters and performs FILTERED discovery plus five
   valid checks. No user action or intermediate save/restart occurs. Both
   final series must belong to the same successful pair before publication/save.

Discovery and validation use the same verified stock single-axis G28 path.
There is no standalone 20 mm probe move or successful no-contact endpoint.
Only returns after qualifying impacts target the configured midpoint.
CALIBRATE and ACCURACY share the helper for midpoint returns. These returns use
ordinary acceleration restored after homing, then wait `calibration_settle_time`
(default 0.25 s) before the next acquisition. They do not apply the homing
acceleration cap to midpoint returns. Homing speed and its acceleration cap
are unchanged. No return command changes the other XY coordinate or Z.
Cancellation/failure restores settings; stock G28's fatal error handling may
turn off all motors, including Z. Keep Z supported as for ordinary homing faults.

## Travel plausibility before midpoint returns

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

## Automatic early-impact returns

For a premature impact, take the selected axis's current coordinate after stock
G28 and subtract its recorded signed start-to-halt displacement. Command one
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
policy. A new paired HYBRID retry needs at least 12 attempts for both fresh
series; a pure-mode retry needs at least 11 for discovery and ten checks.
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

## Saving and ordinary homing

A clean paired HYBRID cycle has RAW discovery plus five checks and FILTERED
discovery plus five checks; explicit pure RAW/FILTERED has one discovery plus
ten checks. `SAVE_CONFIG=1` is still the default: only a complete qualified
result and successful cleanup stage the selected axis model, save it, and
restart Klipper once. `SAVE_CONFIG=0` only stages the complete result until an explicit save.
The other axis's saved model is not changed.

After the required checks pass, add **0.5 g once** to each final qualified
candidate. For example, a final candidate tested at 1.25 g produces a saved
threshold of 1.75 g. Discovery, retries and all qualification checks use the
candidate without this final margin; the higher saved value was **not tested
by those qualification homes**. The margin is derived from the final candidates,
not accumulated on pair restarts. A later `SAVE_CONFIG`, restart or
model reload does not add the margin again. Failed qualification stages no
replacement threshold. Beta proposals are unchanged.

Console output and saved metadata distinguish `tested_threshold_g`,
`saved_threshold_g` and `threshold_margin_g` (0.5), with
`saved_threshold_tested: false`. The final result's `threshold_g` is the saved
value; `candidate_g` remains the tested candidate. Stationary-noise charts
continue to describe their measurement and initial-value hints.

Paired results identify `requested_detector=hybrid` and
`calibration_mode=paired_profiles`, with `qualification_counts={raw: 5,
filtered: 5}`, `selected_profile_successes=5` and `successes=10` for the pair
total. The total does not claim ten checks for either individual profile.
`profiles` retains each actual detector's threshold/metadata; `pair_id` and
`paired_restarts` identify the successful paired qualification and whole-pair
retries. `SAVE_CONFIG=0` stages only a complete pair; there is no first-series
stage/publication for the operator to save separately.

Paired active status and aborted/final results expose
`requested_detector=hybrid`, `actual_series_detector` (RAW or FILTERED),
`qualification_counts`, `required_checks_per_detector=5`, `pair_id`,
`paired_restarts` and the independent `paired_candidates`. These identify
actual series controls rather than claiming HYBRID-position homes. Both counts
return to zero on a recoverable whole-pair restart. A successful result's
`candidate_g` follows the selected profile's `tested_threshold_g`;
`actual_series_candidate_g` separately identifies the final FILTERED candidate.

Schema 3 metadata stores each `paired_qualification` proof and its corresponding
`paired_calibrations` certificate, tying RAW five and FILTERED five to the same
pair, actual stage identities, context and once-only margins. A later explicit
pure ten-check calibration may replace one profile while retaining a certificate
needed by an archived five-check profile. Such mixed lifecycle evidence remains
schema 3; it is not relabelled as a legacy ten-check pair. See
[older-source compatibility](FILTERS.md#returning-to-stable-code).

New profiles record `impact_metric: xyz_magnitude` in their JSON metadata, and
new native calibration records
`native_polarity_policy: absolute_after_reference_subtraction_v1`.
Paired HYBRID stores honest five-check-per-profile and ten-total qualification
evidence in model schema 3. Existing schema 1/2 models remain readable without
automatic rewriting. After paired calibration reload, ordinary G28 retains
the pre-calibration selected RAW/FILTERED type; configured manual RAW thresholds
are allowed and preserved and retain priority over the new model thresholds.
TEST/CHARTS retain their existing selected-mode default. ACCURACY defaults to
HYBRID and uses both compatible profiles. Explicit pure calibration continues
to select its requested mode. Charts identify the actual detector and metric.
Native saved selected-axis RAW profiles with legacy/unknown polarity provenance
require recalibration before live use, including when impact_sign was zero.
Explicit native manual thresholds remain unqualified centered-absolute
selected-axis values; generic legacy policies are unchanged. Leave manual overrides commented out
to use the new calibrated model; thresholds from different metrics are not
interchangeable. Existing axes_map remains useful for other Klipper features.

New calibration metadata also records `travel_plausibility_required: true`,
`recenter_acceleration_policy: restored_normal_limit` and the configured
settling interval. `physical_position_verified` remains false.

`ACCEL_HOMING_CALIBRATE_NOISE` remains a separate stationary diagnostic. Its
mapped X/Y component hints do not select a later calibration's initial value;
that calibration uses its own fresh XYZ-magnitude measurement.
`ACCEL_HOMING_STATUS` reports the discovery/validation phase, threshold,
restart count and the current series's required-count spread. Its `stationary_noise` summary
identifies the fresh measurement, magnitude peak, initial-value source and PNG.
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
Active calibration status and final results also expose `attempt_limit`,
`attempts_remaining` and the required candidate-attempt budget. A final result
retains `candidate_g` and `restarts` if the next candidate is denied.

## Limits of this method

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
revision and environment; see [offline verification](../TESTING.md). Historical
passing tests and publication dates do not establish hardware validation.
