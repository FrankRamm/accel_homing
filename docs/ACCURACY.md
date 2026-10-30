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
A successful measurement alone does not save/restart. RAW/FILTERED early-trigger
recovery can start a separate normal calibration that saves and restarts on
success. HYBRID early-quality failures abort for operator review instead.

## Compare 30 RAW, pure FILTERED and HYBRID homes

When both compatible calibrated profiles already exist, compare them directly;
another calibration is not required to collect these measurements. Otherwise
follow the [profile preparation sequence](FILTERS.md#operator-sequence) first. Explicit
`DETECTOR` selects that mode's own saved threshold for the command. A successful
ACCURACY run restores the previously selected mode instead of changing it.

These are three different modes. RAW uses its own detector. Pure FILTERED uses
the accepted FILTERED threshold crossing for its position timestamp. HYBRID
keeps that FILTERED control/confirmation and selects an associated RAW XYZ
threshold crossing for position, with an explicit FILTERED fallback when RAW
selection is unavailable or ambiguous. The saved RAW and FILTERED thresholds
remain usable without recalibration. HYBRID must be requested explicitly and
does not become the persisted selected mode.

Manually prepare XY before the fresh RAW command:

```gcode
ACCEL_HOMING_ACCURACY AXIS=X DETECTOR=RAW SAMPLES=30 CENTERED=1
```

Keep its successful original PNG and JSON, and copy the console's **Completed
accuracy run ID**. Manually prepare XY again before the pure FILTERED command:

```gcode
ACCEL_HOMING_ACCURACY AXIS=X DETECTOR=FILTERED SAMPLES=30 CENTERED=1 BASELINE=ID_RAW
```

Replace `ID_RAW` with the completed RAW run ID. Keep the pure FILTERED outputs
and copy its completed run ID. Manually prepare XY again before HYBRID:

```gcode
ACCEL_HOMING_ACCURACY AXIS=X DETECTOR=HYBRID SAMPLES=30 CENTERED=1 BASELINE=ID_FILTERED
```

Replace `ID_FILTERED` with the completed pure FILTERED run ID. `BASELINE` is
optional for **each** command; omit the entire parameter for a standalone
report. To compare HYBRID with RAW instead, use that completed actual RAW run
in the same chart directory:

```gcode
ACCEL_HOMING_ACCURACY AXIS=X DETECTOR=HYBRID SAMPLES=30 CENTERED=1 BASELINE=ID_RAW
```

Run IDs contain 32 hexadecimal characters. One command accepts one baseline;
the preserved independent reports can be inspected separately without another
physical series just to change the baseline plot. Choose either the RAW or
FILTERED baseline for the HYBRID command in the three-series comparison.

Each command has one excluded discovery home followed by 30 measured homes.
Use the same sensor/mount/axes map, geometry, homing speed/acceleration, currents,
fan state and physical setup. Repeat separately for Y only when both compatible
saved Y profiles exist, using that axis's baseline.
If a RAW/FILTERED early trigger starts automatic recovery, the accuracy run is
incomplete; the separate recovery calibration can save and restart. Reprepare after that
restart and follow the printed rerun command; see
[early-trigger recovery](#early-trigger-automatic-recalibration).
HYBRID instead aborts for manual inspection and recentering on an early-quality
failure; it does not start the automatic return/recalibration path.

Both saved detector profiles and the compact PNG baseline receipt survive a
saving calibration/restart. The report shows both actual series, counts, ranges,
population standard deviations, threshold/mode/config differences and required
context mismatches. It does not pool trials, infer paired physical measurements
or use shadow replay as a baseline. A baseline from another axis, an incomplete
run or a synthetic figure is refused. Existing PNG baselines with a valid compact
receipt remain usable without a JSON companion and retain their historical schema.
A PNG without a valid completed-run receipt still requires a new baseline.

For a before/after pure FILTERED comparison, its command can select the verified
original completed FILTERED PNG as its alternative baseline:

```gcode
ACCEL_HOMING_ACCURACY AXIS=X DETECTOR=FILTERED SAMPLES=30 CENTERED=1 BASELINE=ID_PREVIOUS_FILTERED
```

Replace `ID_PREVIOUS_FILTERED` with that original run's actual 32-character ID.
Use one baseline ID per command. This compares the new pure FILTERED series to
the old pure FILTERED series. A historical FILTERED receipt without
an explicit position policy retains `filtered_lookback_v1`; new FILTERED receipts
use version 4 with `accepted_threshold_crossing_v1`. The report lists this policy
change as a treatment difference. Preserve the old files and interpret their
position statistics separately instead of pooling or relabelling them.

HYBRID receipts identify filtered control separately from
`requested_detector=hybrid` and
`position_policy=associated_raw_threshold_crossing_v1`. Their measured positions
can include RAW selections and FILTERED fallbacks; inspect those counts and
reasons before comparing aggregate spread or standard deviation. A different
timestamp policy is a treatment difference, not proof of improved physical
contact or unchanged end-to-end stop latency.
RAW-selection counts describe eligibility. Even every selected RAW timestamp
can coincide with the FILTERED crossing; a high selection count alone is no
timing or accuracy improvement. Per-home `raw_minus_filtered_time_s` describes
the sampled timestamp difference, not detector reaction or physical stop latency.
HYBRID is available only on the native Beacon packet path; other backends fail
preflight before movement. TEST's default XY selection needs both profiles on
both axes; the commands above deliberately select X.

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

For RAW and pure FILTERED, recovery is limited to an early trigger with complete
**current** capture, signed trigger/halt step evidence and an intact confirmed
starting reference.
A sensor fault, missing/mismatched evidence, lost motor/position state,
cancellation or invalid candidate prevents recovery.
HYBRID does not enter this recovery procedure; an early-quality failure aborts
with diagnostics for operator review and manual XY preparation.

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

## Per-home JSON diagnostics

Every successful RAW, pure FILTERED or HYBRID ACCURACY run automatically writes
a diagnostic JSON beside each axis's overview PNG. No new option or calibration is needed,
and diagnostics do not change thresholds, filtering, returned trigger decisions
or the number of homes. The console prints the JSON path as `Accuracy telemetry
X: ... (schema 1; discovery + N measured homes)`.
Workflow/status keeps `summary_paths` PNG-only and reports JSON paths separately
in `telemetry_paths` as `x_json`/`y_json`; it does not dump sample rows repeatedly.

For example, the normal output names share a run/axis/local-timestamp prefix:

```text
Accel Homing Accuracy X - Run <id> - <timestamp>.png
Accel Homing Accuracy X - Run <id> - <timestamp>.json
```

Collision suffixes can differ between files. Use the announced paths and the
JSON's `png_sha256` to associate the exact PNG, and preserve the original files
and PNG metadata. The JSON has its own version 1 schema and
`kind=completed_accuracy_telemetry`; it does not replace or change the compact
baseline receipt embedded in the PNG. New pure FILTERED PNG receipts use version
4 to identify the accepted-crossing position policy; HYBRID uses receipt version
5 for its distinct requested mode, position configuration and selection counts.
The separate JSON schema remains version 1.
Historical PNG receipts remain readable without rewriting them. JSON cannot
recreate diagnostics for older runs.
The header also distinguishes actual runs (`simulated=false`) from synthetic QA.

The JSON separates `discovery` from the measured `trials` and retains run/axis,
threshold/configuration, sensor/geometry context and statistics. Each home's
native event window contains at most **50 consecutive original rows**, beginning
up to 16 rows before the recorded onset. It is not a downsampled plot trace or a
full capture export. A FILTERED window starts no earlier than its first armed
sample; unavailable pre-onset context is flagged as edge-truncated. RAW can retain
original pre-arm rows without claiming an IIR state. The window labels omitted
rows and events outside its bounds. Unsupported/generic capture paths mark native data unavailable
with a reason instead of manufacturing sensor counts or packet identities.
Diagnostics are bounded to 128 KiB per home and 8 MiB per axis, with at most one
discovery and 50 measured homes.

Retained native rows keep signed sensor-frame counts, packet/sample identities,
per-packet sample intervals, marker flags and bounded decoded endpoint estimates.
Original acquisition-snapshot times and origins are separate from capture/remapped
times. Event records retain the actual recorded onset/crossing/confirmation times
and the exact trigger time returned to Klipper. In RAW, disarm-time conversions,
acquisition snapshots and the returned trigger value can differ numerically;
use their labelled origins and sample identities rather than silently treating
them as one clock snapshot. Unrecorded legacy event times remain `null` with an
unavailable reason even when their original sample identities are retained.
`returned_trigger_time_s` is the timestamp value returned by `home_wait`, not
the host time when that function finished.

For FILTERED and HYBRID, diagnostics include the frozen reference offsets/noise
weights, the effective onset gate and actual candidate/rejection counters. When native
provenance is available, `state_before_window` is
**post-stop deterministic replay** of the retained exact prefix, verified against
the actual original event decisions. It is explicitly not an online state
snapshot. Replay counters and whether they match the actual counters are labelled
separately. RAW marks filter state unavailable; an unsupported FILTERED capture
reports missing native proof as its unavailable reason. Rows identify whether
the filter processed them; post-confirmation values are not additional online
filtering.

HYBRID additionally retains its independent frozen RAW arithmetic reference,
RAW XYZ threshold and the bounded selection evidence. Each home's
`position_selection` identifies `actual_source` as `raw_threshold_crossing` or
`filtered_threshold_crossing_fallback`, with an explicit `fallback_reason` when
applicable, `raw_crossing_count`, association interval and selected time/sample.
The [association interval and exact reasons](FILTERS.md#hybrid-position-timing)
are bounded to the accepted candidate through its confirmation sample; later
response rows cannot supply the selected crossing.
`raw_crossing_count` counts every provable discrete edge, including a marker
edge that remains ineligible for position. A native marker cannot supply the
selected timestamp or RAW amplitude; a proven continuation-above code state can
preserve an earlier finite crossing. Diagnostics retain
`marker_rows_with_proven_above_code_state`,
`marker_crossings_ineligible_for_position` and `native_clip_values`. These are
RAW code-metric evidence, not recovered physical amplitudes.
The RAW reference and FILTERED reference use the same admitted stationary
pre-arm window but keep their distinct arithmetic offsets and filter weights;
FILTERED noise weights are not applied to RAW XYZ magnitude.

The completed report's `hybrid_position_summary` records measured
`raw_selected_count`, `fallback_count` and `fallback_reasons`; the PNG displays
the corresponding counts. Keep discovery separate from measured statistics.
Fallback is reported as fallback rather than silently counted as a RAW-selected
position. A HYBRID run is not a pure FILTERED run merely because its stop
confirmation uses FILTERED.

Planned movement-start and first-step timestamps are explicitly `null` with an
unavailable reason: the existing homing interface does not expose them. Do not
call onset-minus-arm detector reaction latency or substitute a rounded console
value for a missing event time. The host stop-request interval bounds dispatch,
not MCU halt or physical contact. Trigger travel and signed start-to-halt travel
are separate commanded-step measurements; neither independently measures carriage
position. The JSON retains `physical_position_verified=false`.

Rows label their relation to the host stop-request bounds. Only original times
strictly before the lower bound are proven before dispatch; later response data
cannot validate a new pre-stop decision. Native markers retain their estimate
and unknown true sign/peak semantics; a numeric endpoint is not a recovered
beyond-range amplitude. These limits apply even when the JSON contains more
precision than the plot or console.

## PNG, JSON and cleanup

A successful ACCURACY run keeps one final overview PNG and diagnostic JSON per
requested axis in the configured chart directory. Failed, rejected, aborted or
cancelled runs publish neither accuracy artifact, including when another
requested axis already completed.

Full per-home traces and current-run provenance stay bounded in memory; only the
diagnostic window above is serialized. No per-home PNG or scratch capture file is
written. Required cleanup, every axis's rendering and JSON validation complete
before publication starts. Handled publication errors/cancellation attempt to
remove only this run's newly owned files; existing outputs are preserved. This
group rollback is not crash-atomic or a guarantee against power loss. Partial
statistics remain available in console/status, without recurring sample dumps.

See [plot locations and access](PLOTS.md) and
[troubleshooting](TROUBLESHOOTING.md). `ACCEL_HOMING_CANCEL` is not an immediate
emergency stop for a running console command.
