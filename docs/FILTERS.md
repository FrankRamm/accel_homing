# Development filters and actual-run comparison

This is **development / experimental** behavior on `codex/accuracy-filters`.
It is accelerometer-only: no encoder or separate contact sensor is added.
Filtering may suppress interference, attenuate a real narrow contact and delay
the stop request. A smoother curve is not a measured accuracy improvement.
The operator performs every physical home/calibration and checks the result.

## What changes

`RAW` retains the existing detector and its threshold semantics. New
`FILTERED` calibrations use detector version 2, `signal_mode=rectified`:
subtract the frozen signed stationary reference from each sensor axis, take
the absolute value of each centered axis, then apply the causal first-order
low-pass filter before the weighted XYZ decision norm. Original signed samples
are retained. Absolute value is applied before filtering, not to a filtered
signed signal or to the raw sensor reading before gravity/offset removal.

Saved version 1 FILTERED profiles retain their original signed-axis filtering
until a new FILTERED calibration replaces that profile. Version 1 serialized
configurations remain unchanged and contain no `signal_mode` field. Both
versions use a frozen robust stationary median/MAD reference, bounded XYZ
noise weights, and a short confirmation window with excess-energy and multiple
raw-support requirements.
It retains transverse components; it does not project onto one axis or use
future samples. Moving-noise characterization is observation only and never
raises the threshold or changes weights after arming.

The versioned initial configuration uses a 0.2 ms time constant, confirmation
between 0.3 and 1.5 ms, and a minimum XYZ weight of 0.75. These are development
defaults, not measured optimal values. Timing, sample age, sensor health,
overflow, sample-integrity and cleanup checks remain required. No new Python
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

   This measures fresh stationary noise, qualifies the rectified-v2 FILTERED candidate,
   saves the selected axis and restarts Klipper by default. Explicit manual
   `threshold`/axis threshold config retains RAW semantics; remove those
   overrides and restart while idle before FILTERED calibration. The command
   refuses that conflict before motion. `SAVE_CONFIG=0` stages only; it does
   not activate the staged model for ordinary homing. A successful calibration
   publishes its stationary-noise summary; it does not automatically publish
   the signed-versus-rectified comparison below.

3. Reprepare and inspect the comparison:

   ```gcode
   ACCEL_HOMING_FILTER_CHARTS AXIS=X DETECTOR=FILTERED CENTERED=1
   ```

   This performs **another real home**, requiring fresh manual preparation and
   a physically centered toolhead after calibration/restart. The saved
   FILTERED version controls that home; both filter versions are compared on
   its same original capture. Use `DETECTOR=RAW` for an
   explicit control using the archived RAW profile, or `DETECTOR=FILTERED` for
   explicit filtered control. Nonactive algorithms are diagnostic shadow
   replays and do not operate another physical home.

   If no compatible saved RAW XYZ profile is available, an explicit RAW
   comparison threshold is supported. For an operator-chosen 4.25 g example:

   ```gcode
   ACCEL_HOMING_FILTER_CHARTS AXIS=X DETECTOR=FILTERED RAW_THRESHOLD=4.25 CENTERED=1
   ```

   This changes only the RAW shadow's numerical comparison threshold; it does
   not change the controlling FILTERED threshold or convert an old component
   threshold into a calibrated XYZ threshold. FILTERED calibration preserves
   an existing RAW model profile, but a config-only default need not have an
   archived XYZ profile.

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
mode, exact filter version/configuration, run ID and thresholds in g. It shows
the original centered signed XYZ, the prior signed-v1 filtered axes, and the
new rectified-v2 per-axis envelopes at matched sample times. Rectified envelopes
are reordered to machine-axis names without reintroducing `axes_map` signs;
they are not signed acceleration or measured motion direction. Both variants
use the same capture, frozen reference, noise weights, other filter parameters
and numerical filtered threshold. The nonactive variant's threshold is a
diagnostic comparison setting, not an independent calibration for that metric.
Each weighted decision level is separate from its unweighted norm.
Candidates, estimated onset, confirmation and the actual host stop-request
interval are distinct for each version. Confirmation-minus-RAW-crossing values
are algorithm-time comparisons only and remain unavailable for censored
decisions. Estimated onset is not independently measured contact.
Original acquisition timestamps determine decision eligibility. Retimed plot
positions, including display-only interpolation within a packet gap, do not
change which confirmations were observed before the real cutoff.

Only samples strictly before the conservative lower bound of the actual
stop-request interval can validate a replay decision. For a historical censored
capture, the earlier first invalid sample remains the replay cutoff.
The chart names the active cutoff and its time. A detector that has not
confirmed before that cutoff is **censored**; the chart cannot infer its
additional delay, rejection of a false contact, or what another physical home
would have done. Gray after-cutoff curves show valid captured response only.
An amber region marks an omitted saturated tail and contains no reconstructed
curve. Neither region proves contact or mechanical stopping.

Noise RMS and robust P99 use the same validated stationary pre-arm reference
interval, after filter warmup, with each interval mean removed. They exclude
the candidate/contact/after-stop response. Separately reported decision-level
mean, RMS and P99 keep the mean intact, exposing the positive baseline introduced
by rectification. A small mean-removed fluctuation is not a small decision
level. The PSD uses an unreduced contiguous
reference segment only: at least 256 samples spanning 50 ms and at most 10%
sample-period irregularity. Otherwise the spectrum says unavailable. A lower
RMS/PSD, different crossing count or rejected candidate is not a labelled false
positive or proof of better contact accuracy.
Absolute value is nonlinear: spectral peaks can move or double, so differences
between these PSDs are not a linear attenuation measurement.

A single comparison home has no repeatability spread/standard deviation.
Use actual repeated ACCURACY runs for that measurement. Ordinary filtered
capture/accuracy component curves show sensor input, including explicitly
labelled bounded endpoint estimates when native markers occur; their causal
weighted threshold is stated separately rather than drawn over the raw norm.

## Read the actual A/B report

Acceleration components in every plot use absolute display values, including
the signed v1 filter output. The v1 detector still filters signed centered XYZ;
only its plotted Y coordinates are folded. Stored display rows retain their
signs, and original timing, confirmation, RMS/P99 and genuine PSD calculations
are unchanged. Relative time remains signed. See [PLOTS.md](PLOTS.md) for display
and directional-threshold details.

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
as a baseline. Every rectified-v2 actual run uses receipt version 3, with or
without censored responses, so older code cannot accept it as a signed run.
Historical signed-filter receipt versions 1 and 2 remain readable with their
exact original detector configuration; version 2 records censored responses.
Existing valid version 1 RAW baselines remain readable. A/B labels distinguish
FILTERED v1 signed from FILTERED v2 rectified, and list algorithm/configuration
differences explicitly. Absent censor metadata is displayed as unknown
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

### Finite range observations and explicit native clipping

Finite signed axis values at or above the configured per-axis range now enter
the existing reference, noise, homing and replay pipelines. They do not imply
an impact: `FILTERED` still requires its existing temporal/energy confirmation,
and all timestamp, frozen-stream, quiet-reference and other health checks still
apply. Threshold headroom and scale-metadata validation are unchanged.
`RAW`, signed `FILTERED` v1 and rectified v2 retain their signal calculations;
v2 still subtracts the frozen signed reference before taking absolute values.

`amplitude_limited` and `excess_amplitude_unknown` are informational: at least
one finite original axis value was at/above the configured range. The numeric
`impact_peak_g` for accepted finite `FILTERED` values is an **observed code-derived magnitude**, not proof of the
true physical maximum or any excess beyond that range. These finite values are
not clamped or reconstructed. `RAW` retains `reported_sensor_derived` peak semantics
and its existing legacy sentinel handling. The existing `saturated_axis_values` counter describes range
observations, not proven physical clipping; `range_limit_axis_values` excludes
the explicit native sentinel and nonfinite values. A finite range observation
alone does not invalidate `raw_samples_validated`, truncate a response, or
prevent a successful workflow/chart. Historical `axis_full_scale` receipts
remain readable, but the current detector no longer emits that rejection.

The known native `0x7FFF` marker now uses the configured Beacon endpoints and
its previous decoded **sensor-axis** sign: nonnegative selects the positive
endpoint, negative selects the negative endpoint. A new host stream starts
with the explicit zero seed, so an initial marker selects the positive end.
This follows Beacon's sign-selection convention; it does not claim to share
the upstream helper's separate queued history. Endpoints come from the actual
helper and active scale. Missing or invalid endpoints fail before `FILTERED`
motion/acquisition; the former `32767` fallback is never a new decoder limit.

Decoded rows retain their original timestamps and identities. Subtraction of
the frozen reference comes next, followed by the selected signed v1 or
rectified v2 filter. The unchanged support/energy gates now count qualifying
input rows under this decoder policy, which can include endpoint estimates.
A marker alone is not an unconditional home, and acceptance is not proof of
its true sign/amplitude or physical contact. No thresholds, speeds, retries,
calibration rules or save margins are changed by this input policy.

`native_clip_decode` names `beacon_previous_sensor_sign_v1`, configured limits,
scale, marker-axis count and first/last source identities. Marker-derived
traces and peaks are bounded endpoint estimates with inferred sign and unknown
true peak, not original signed measurements. Reference and moving-noise data
use the same convention. Native `RAW` control retains its legacy decoder; its
armed chart rows use the frozen seed that that controller actually used.
Generic numerical `32767` values are ordinary numerical data, not Beacon markers.

The original bytes remain bounded private proof data. Native replay checks
the admitted stream seed, chained packet seeds/endpoints, decoded rows,
frozen reference and actual confirmation. Processing before confirmation is
chronological; tail conversion/state advancement follows stop dispatch.
Nonfinite data, timestamp/order/gap/freshness failures, overflows, frozen
streams and changes of scale/endpoints remain errors. No row is skipped.

### Historical censored native responses

Older native Beacon `FILTERED` captures with a fully qualified confirmation
strictly before the first explicit native clipped sample can retain a censored
response. This comparison is chronological sample order, including rows in
one packet; it does not assume the clipping happened after the host requested
a stop. These historical censored captures/PNG receipts remain readable.
New known-marker captures retain all decoded rows and are not truncated or
aborted solely because of the marker. `RAW` retains its existing control behavior.

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
their broader fault result. Historical `native_clip_sentinel` means Beacon reported `0x7FFF`;
Historical `axis_full_scale` means a raw sensor axis reached its range limit;
that magnitude-only rejection has been removed for finite values.
An XYZ vector magnitude of 14–16 g, or even above the individual-axis range,
does not itself prove clipping. `nonmonotonic_packet_clock`,
`filter_nonmonotonic_time` and `filter_sample_gap` identify different timestamp
rejections. The record includes the phase, packet clocks/count, and relevant
sample position, axis, value or time difference; it does not retain raw history.
An in-scope prefix timing rejection also reports its packet index, previous
and next timestamps, measured gap and limit. These numbers identify the actual
rejected pair; an error from an older run cannot establish that pair.

Stop dispatch occurs before optional diagnostic conversion or logging.
Finite range readings and known Beacon markers now enter the decoder/filter
as described above. Other rejection rules and watchdogs are unchanged. A failed
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

Older development code from before rectified v2 rejects a v2 FILTERED profile,
including one archived inside an otherwise RAW-selected model. Selecting or
calibrating RAW alone does not make that model readable by such an older
development version. Restore a verified compatible pre-v2 model/configuration
backup when returning to that target. The stable-main procedure below is a
different compatibility target; do not assume their model readers are identical.

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
