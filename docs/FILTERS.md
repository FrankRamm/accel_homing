# Development filters and actual-run comparison

This is **development / experimental** behavior on `codex/accuracy-filters`.
It is accelerometer-only: no encoder or separate contact sensor is added.
Filtering may suppress interference, attenuate a real narrow contact and delay
the stop request. A smoother curve is not a measured accuracy improvement.
The operator performs every physical home/calibration and checks the result.

## What changes

`RAW` retains the existing detector and its threshold semantics. `FILTERED`
uses a causal first-order low-pass filter on signed mapped XYZ, a frozen robust
stationary median/MAD reference, bounded XYZ noise weights, and a short
confirmation window with excess-energy and multiple raw-support requirements.
It retains transverse components; it does not project onto one axis or use
future samples. Moving-noise characterization is observation only and never
raises the threshold or changes weights after arming.

The versioned initial configuration uses a 0.2 ms time constant, confirmation
between 0.3 and 1.5 ms, and a minimum XYZ weight of 0.75. These are development
defaults, not measured optimal values. Timing, sample age, sensor health,
overflow, saturation and cleanup checks remain required. No new Python
dependency is added to the armed detector.

Thresholds belong to their detector metric. A RAW component/XYZ threshold is
not automatically a FILTERED weighted-level threshold. CALIBRATE qualifies ten
homes at the candidate and adds the existing **0.5 g once** only when staging
the successful result. Those ten homes do not test the higher saved threshold.

## Operator sequence

Before **each** moving command below, follow the
[manual preparation](COMMANDS.md#prepare-every-moving-workflow): printer idle
and unpaused, both XY axes unhomed and physically centered, Z supported,
nozzle/bed clearance, suitable motor enables/currents, accessible emergency
stop. `CENTERED=1` authorizes movement and does not center the toolhead.
Repeat preparation after every successful saving calibration/restart.

1. Collect the RAW control at its own calibrated threshold:

   ```gcode
   ACCEL_HOMING_ACCURACY AXIS=X DETECTOR=RAW SAMPLES=10 CENTERED=1
   ```

   Keep the successful PNG on the host. Copy its **Completed accuracy run ID**
   from the console/status. A failed or incomplete run is not a baseline.
   If the RAW model still uses the legacy selected-axis metric, first run
   `ACCEL_HOMING_CALIBRATE AXIS=X DETECTOR=RAW CENTERED=1`, then reprepare and
   collect the baseline. FILTER_CHARTS requires a RAW XYZ-magnitude profile
   rather than treating a legacy component threshold as a magnitude threshold.

2. Reprepare and calibrate FILTERED explicitly:

   ```gcode
   ACCEL_HOMING_CALIBRATE AXIS=X DETECTOR=FILTERED CENTERED=1
   ```

   This measures fresh stationary noise, qualifies the FILTERED candidate,
   saves the selected axis and restarts Klipper by default. Explicit manual
   `threshold`/axis threshold config retains RAW semantics; remove those
   overrides and restart while idle before FILTERED calibration. The command
   refuses that conflict before motion. `SAVE_CONFIG=0` stages only; it does
   not activate the staged model for ordinary homing.

3. Reprepare and inspect the comparison:

   ```gcode
   ACCEL_HOMING_FILTER_CHARTS AXIS=X CENTERED=1
   ```

   The saved selected mode controls the real home. Use `DETECTOR=RAW` for an
   explicit control using the archived RAW profile, or `DETECTOR=FILTERED` for
   explicit filtered control. The other detector is replayed on the same
   input; the replay does not operate a second physical home.

4. Reprepare and compare a new FILTERED series to the saved RAW baseline:

   ```gcode
   ACCEL_HOMING_ACCURACY AXIS=X DETECTOR=FILTERED SAMPLES=10 CENTERED=1 BASELINE=<raw_run_id>
   ```

   Replace `<raw_run_id>` with the actual 32-character ID printed for step 1.
   Do not include angle brackets. Use the same sensor/mount/axes map, rail
   geometry, speed, homing acceleration, motor current, fan state and physical
   setup. Repeat the sequence separately for Y; use that axis's baseline.

Both detector profiles coexist per axis in the saved model. A FILTERED
calibration preserves the prior RAW threshold and metadata across restart;
`DETECTOR=RAW` can select that archived profile without changing the saved
selected mode. Reloading/staging does not add the 0.5 g margin again. A later
successful calibration of a mode replaces that mode's profile. An omitted
DETECTOR uses the selected model/manual mode; choose it explicitly for A/B work.
The X/Y config does not gain an independent persistent detector switch.

## Read the same-trace chart

The **Development / Experimental** chart identifies the actual controlling
mode, filter version/configuration, run ID and both thresholds in g. It shows
raw and causal filtered signed XYZ and magnitudes at matched sample times;
the weighted filtered decision level is separate from the unweighted norm.
Candidates, estimated onset, confirmation and the actual host stop-request
interval are distinct. Estimated onset is not independently measured contact.

Only samples strictly before the conservative lower bound of the actual
stop-request interval can validate a replay decision. If native saturation
truncates the response earlier, the first invalid sample is the replay cutoff.
The chart names the active cutoff and its time. A detector that has not
confirmed before that cutoff is **censored**; the chart cannot infer its
additional delay, rejection of a false contact, or what another physical home
would have done. Gray after-cutoff curves show valid captured response only.
An amber region marks an omitted saturated tail and contains no reconstructed
curve. Neither region proves contact or mechanical stopping.

Noise RMS and robust P99 use the same validated stationary pre-arm reference
interval, after filter warmup, with each interval mean removed. They exclude
the candidate/contact/after-stop response. The PSD uses an unreduced contiguous
reference segment only: at least 256 samples spanning 50 ms and at most 10%
sample-period irregularity. Otherwise the spectrum says unavailable. A lower
RMS/PSD, different crossing count or rejected candidate is not a labelled false
positive or proof of better contact accuracy.

A single comparison home has no repeatability spread/standard deviation.
Use actual repeated ACCURACY runs for that measurement. Ordinary filtered
capture/accuracy component curves remain raw measured response; their causal
weighted threshold is stated separately rather than drawn over the raw norm.

## Read the actual A/B report

`BASELINE` selects one explicit successful run ID from the same chart root and
axis. It never chooses the newest run, scans for a convenient control, pools
trials or combines a replay with real homes. Both series are independent;
their accepted-home indices are not paired physical measurements.

The report compares count, actual commanded travel, range and population
standard deviation. It separately counts censored responses: a qualified
commanded-travel trial can remain valid while its full response peak is
unavailable. Only each trial's validated finite prefix appears in the response
overlays; a truncated endpoint is marked. It lists detector, metric,
configuration and threshold differences as the treatment. Sensor, geometry,
speeds and homing acceleration must be present and match for reported-context
comparability. Missing/different
conditions are shown prominently and prohibit attributing the change to
filtering. Even matching recorded conditions establish only the variation
observed in those runs, not absolute position, correct contact, lack of missed
steps or a causal physical improvement. Repeat controlled series and inspect
the machine before drawing a hardware conclusion.

The compact validated receipt is embedded in the completed PNG. Keep its
original filename and metadata on the host; image editors/recompression can
strip that metadata. Old PNGs without this development receipt cannot serve
as a baseline. Receipts with censored responses use version 2 so older code
cannot read them as ordinary version 1 runs. Existing valid version 1 RAW
baselines remain readable; absent censor metadata is displayed as unknown
legacy evidence, not zero censored responses. Lookup accepts bounded safe IDs,
never filesystem paths; rejects symlinks, duplicate IDs,
malformed/incomplete/oversized receipts, another axis,
and synthetic/actual mixing. Baseline selection is validated before movement
and again before publication. A directory with more than 4096 entries requires
operator housekeeping; no old outputs are deleted automatically.

Only full successful workflows publish final PNGs. The existing bounded
in-memory capture and all-images-before-publication transaction apply to the
new comparison too. A failure/cancellation does not leave a partial PNG or raw
scratch capture. Successful existing baselines are retained.

## Offline examples

Run on a development machine with the existing matplotlib stack:

```bash
python3 -B tests/render_filter_examples.py --output /tmp/accel-homing-synthetic-plots
```

Every generated figure is prominently **SYNTHETIC / SIMULATED / NOT PRINTER
MEASUREMENT**. The generated pulse, noise and independent travel series are
visual/software fixtures, not printer measurements or results. Native callback
examples also exercise post-confirmation saturation, its missing response tail,
and a censored-response count alongside independently computed travel statistics.

## Sample-fault diagnostics

### Native saturation after FILTERED confirmation

Only a native Beacon `FILTERED` home with a fully qualified confirmation
strictly before the first clipped/full-scale sample can retain a censored
response. This comparison is chronological sample order, including rows in
one packet; it does not assume the clipping happened after the host requested
a stop. Saturation at or before confirmation still stops and rejects the
attempt. `RAW` retains its existing clipping handling.

The shared validator checks the finite contiguous prefix against the original
native packets, frozen stationary reference, detector configuration and exact
confirmation identities for this attempt. It replays the actual detector to
verify that confirmation. Whole-capture `raw_samples_validated` remains
`false`. Any malformed/nonfinite data, timing failure, invalid reference or
other integrity fault still rejects the capture, including a fault later in
the unusable tail. Rows after the first saturation are never skipped and
joined to earlier rows.

The exported samples contain only that validated prefix. Public
`response_censoring` metadata identifies the first invalid original packet
row and time, original/prefix/censored sample counts, confirmation, and the
separately named `observed_valid_prefix_peak_g`. The full `impact_peak_g` is
`null`: a prefix maximum is not a complete impact peak, bound on the missing
response, or evidence that contact was correct. Charts state **SATURATED /
RESPONSE TRUNCATED / FULL PEAK UNAVAILABLE** and omit the unknown tail.
Original native packets remain bounded private validation evidence in memory;
they are not serialized to a PNG or raw scratch file.

Native `FILTERED` capture starts at the exact accepted stationary-reference
row and ends at host stream close or explicit capture seal. Earlier enable/
settle data and later flush packets belong outside that scope. A partial first
packet retains its original row position, count and clock interval. The scope identifies host
callback admission, not a firmware stream acknowledgment. Already admitted
callbacks finish before capture is sealed; a bounded drain also completes
before trigger dispatch starts. A timeout aborts with `capture_inflight_timeout`
and no output. Gaps, repeats, malformed data and other faults within the
accepted reference and response still reject the workflow. An empty native
packet contributes no rows and refreshes no reference or watchdog.

A failed or incomplete workflow still publishes no chart. This exception does
not establish that an earlier failed calibration would have passed: without
its original valid prefix and confirmation evidence, that conclusion cannot
be reconstructed.

### Rejection details

`invalid_data` is a general stop result, not proof of a repeated timestamp.
For the instrumented native Beacon homing rejection paths, the same command
error and `ACCEL_HOMING_STATUS` now include the first bounded `sample_fault`
record. Generic backend and Beta acquisition faults may still provide only
their broader fault result. `native_clip_sentinel` means Beacon reported `0x7FFF`;
`axis_full_scale` means an individual raw sensor axis reached its range limit.
An XYZ vector magnitude of 14–16 g, or even above the individual-axis range,
does not itself prove clipping. `nonmonotonic_packet_clock`,
`filter_nonmonotonic_time` and `filter_sample_gap` identify different timestamp
rejections. The record includes the phase, packet clocks/count, and relevant
sample position, axis, value or time difference; it does not retain raw history.
An in-scope prefix timing rejection also reports its packet index, previous
and next timestamps, measured gap and limit. These numbers identify the actual
rejected pair; an error from an older run cannot establish that pair.

Stop dispatch occurs before optional diagnostic conversion or logging. Apart
from the validated post-confirmation FILTERED saturation case above, the
rejection rules, watchdogs and clipping protections are unchanged. A failed
workflow still publishes no PNG or raw scratch capture. Copy the exact command
error or status record for diagnosis; an earlier run without this detail cannot
establish which rejection occurred.

Stationary acquisition retains its strict cadence guard and bounded complete
window retries. Its `sample_gap` diagnostic now distinguishes `interval_change`
from `actual_gap`, with both periods and their numerical limits. A changed
packet span alone does not establish benign timing variation or internal sample
loss. These stationary observations do not establish the cause of a later
homing fault.

## Returning to stable code

**A source-only switch to main is incompatible after a FILTERED model was
saved.** A model with an active/staged FILTERED selection uses schema version
2. Current stable main supports version 1 and rejects that model at startup;
this prevents it from silently using a filtered threshold with RAW detection.

While still on development code, use normal operator-supervised
`ACCEL_HOMING_CALIBRATE AXIS=X DETECTOR=RAW CENTERED=1` for each affected axis.
Reprepare before every command and after each successful saving restart.
Repeat with Y if Y was filtered; verify all selected models are RAW and the
saved schema has returned to version 1 after restart. This persists compatible
RAW values rather than merely running a temporary RAW test. While still on
development code, use its reviewed history-aware uninstall (or the exact
unchanged transaction rollback) to restore the original module/updater/model
baseline and remove active development ownership. Only then select stable main
and restart the verified services after source/links/configuration agree.
An uninstall restores the original plugin model baseline and may remove a
first-time plugin installation; review its dry run. Old main has no `--branch`
option and cannot process an active development journal. If main needs to be
installed again, use its ordinary `install.sh` without `--branch` after the
development history has been restored. See the [concrete host procedure](INSTALLATION.md#explicit-development-branch).
Keep valid A/B baseline PNGs as evidence of their
original mode/settings; they are not automatically recast as a new control.

If FILTERED was never saved, the original RAW model was not converted, so the
recorded installation rollback can restore the original modules/updater
registration while idle. Preserve later unrelated configuration edits; do not
overwrite a whole current `printer.cfg` with an old backup.
