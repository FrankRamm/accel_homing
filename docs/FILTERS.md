# Development filters and actual-run comparison

This is **development / experimental** behavior on `codex/accuracy-filters`.
It is accelerometer-only: no encoder or separate contact sensor is added.
Filtering may suppress interference, attenuate a real narrow contact and delay
the stop request. A smoother curve is not a measured accuracy improvement.
The operator performs every physical home/calibration and checks the result.

## What changes

Native Beacon live RAW decisions use `abs(a - reference)` for a selected-axis
metric or the centered XYZ magnitude, independently of sign. Configured
`impact_sign`/automatic direction no longer gate native impacts. Original signed
protocol counts remain untouched for decoding, references, JSON and provenance;
absolute value is applied after stationary-reference subtraction.

New
`FILTERED` calibrations use detector version 2, `signal_mode=rectified`:
subtract the frozen signed stationary reference from each sensor axis, take
the absolute value of each centered axis, then apply the causal first-order
low-pass filter before the weighted XYZ decision norm. Original signed samples
are retained. Absolute value is applied before filtering, not to a filtered
signed signal or to the raw sensor reading before gravity/offset removal.

Live native Beacon FILTERED/HYBRID requires rectified version 2. A saved signed
version-1 profile remains readable as historical/offline evidence but is
rejected before live movement with recalibration guidance. Its serialized
configuration remains unchanged and contains no `signal_mode` field. Historical
v1 and current v2 diagnostics use a frozen robust stationary median/MAD reference, bounded XYZ
noise weights, and a short confirmation window with excess-energy and multiple
raw-support requirements.
It retains transverse components; it does not project onto one axis or use
future samples. Moving-noise characterization is observation only and never
raises the threshold or changes weights after arming.

For live pure FILTERED rectified-v2 control, the position timestamp
uses the existing threshold-crossing sample of the candidate that actually
confirms. Its policy is
`accepted_threshold_crossing_v1`; onset time, level and sample identity come from
that crossing. The earlier lower-level lookback no longer supplies the returned
position timestamp. Temporal confirmation, candidate rejection and the host
stop-request dispatch are unchanged: crossing alone does not authorize a stop.
The original lookback still anchors the existing 15 ms impact-response window.
This changes the position reference, not the filter coefficients, response
capture window or calibrated threshold. Compatible sign-independent RAW XYZ and
rectified-v2 profiles remain usable without recalibration or a model rewrite.
Historical signed-v1 receipts remain readable without being promoted to live v2.

### Native legacy-profile migration

Native signed-v1 FILTERED profiles need a new rectified-v2 calibration before
live FILTERED/HYBRID operation. Saved selected-axis RAW profiles with legacy or
unknown native polarity provenance are likewise rejected, even when their
configured impact_sign is zero; that value cannot prove compatible historical
qualification. Existing RAW XYZ profiles already use a sign-independent norm
and remain compatible. New calibration records
`native_polarity_policy=absolute_after_reference_subtraction_v1`. After normal manual
preparation, one `ACCEL_HOMING_CALIBRATE AXIS=X CENTERED=1` refreshes both
profiles automatically. Use explicit `DETECTOR=RAW` or `FILTERED` for a pure
ten-check replacement when that individual mode is intended. Do not edit saved
metadata to declare an old profile newly qualified. Current compatible XYZ/v2
profiles require no recalibration merely to install this update. Other backend
policies and original encoded sample signs are outside this native-only change.
Explicit configured or valid temporary native RAW manual thresholds are
unqualified values, not inherited saved-model qualification. Temporary SET
provenance belongs only to its successfully supplied current threshold and
preserves detector selection; IMPACT_SIGN-only/invalid SET cannot authorize
it, and it never bypasses native signed-v1 FILTERED rejection.

The versioned initial configuration uses a 0.2 ms time constant, confirmation
between 0.3 and 1.5 ms, and a minimum XYZ weight of 0.75. These are development
defaults, not measured optimal values. Timing, sample age, sensor health,
overflow, sample-integrity and cleanup checks remain required. No new Python
dependency is added to the armed detector.

Thresholds belong to their detector metric. A RAW component/XYZ threshold is
not automatically a FILTERED weighted-level threshold. Explicit pure RAW or
FILTERED calibration qualifies ten homes. Default HYBRID calibration obtains
five valid RAW checks and five valid FILTERED checks in one automatic command,
with separate discovery and actual mode-specific control. Each final candidate
receives **0.5 g once** when the complete result is staged. Qualification homes
do not test the higher saved thresholds.

## HYBRID position timing

HYBRID is an experimental position policy for TEST, ordinary CHARTS and
ACCURACY. ACCURACY uses it when DETECTOR is omitted; TEST and ordinary CHARTS
require an explicit HYBRID request. The saved FILTERED detector still confirms the
impact and requests stopping. When the bounded evidence identifies one eligible
RAW XYZ crossing associated with that accepted FILTERED candidate, HYBRID uses
the RAW crossing's original sample time and identity as its reported position.
It does not use an earlier unconfirmed RAW event to request a stop.
The selected RAW sample may occur before, at or after the FILTERED crossing
within the association interval. Complete RAW selection coverage can therefore
produce exactly the same timestamps as pure FILTERED. Coverage alone establishes
neither an earlier position timestamp nor improved measured or physical accuracy.

HYBRID measurement requires the native Beacon packet path; unsupported backends are rejected
before movement. Both saved profiles must be present and compatible for every
requested axis before movement. The RAW profile must use XYZ magnitude, with
matching sensor,
backend, axes map, geometry, speed, acceleration and impact-sign context.
FILTERED uses its own saved threshold/configuration. RAW uses its own saved
threshold and an independent frozen arithmetic stationary reference. That
reference is computed from the same admitted original pre-arm window as
FILTERED; FILTERED's robust reference and noise weights remain separate.
Its reference metadata records `standalone_raw_lifecycle_equivalent=false`
because this shared-window admission belongs to FILTERED. The RAW metric and
arithmetic reference do not imply that a separate RAW-controlled home would have
the same preparation lifecycle or physical result.
Threshold overrides and unsupported tuning options are rejected. A missing or
incompatible saved profile is a preflight error, not permission to borrow the
other mode's threshold. Existing compatible calibrations need no rewrite or
replacement.

The association interval begins at the latest of the arm time, the accepted
candidate's original lookback onset (its continuous above-gate region), the
accepted FILTERED threshold-crossing time minus configured `onset_lookback_s`,
and the last rejected FILTERED candidate's time. The last-rejection boundary is
exclusive: an event at or before it cannot be reused. The interval ends at the
actual FILTERED confirmation sample. Later response samples cannot supply an
earlier-looking position event.

RAW must have exactly one provable discrete below-to-above threshold edge inside
that interval, and the selected crossing must be an original finite row. Its
selected time is that original sample time; there is no threshold interpolation.
An already-high first available sample does not prove a crossing. Multiple
finite or marker edges are ambiguous rather than permission to choose the
earliest one.

A native marker cannot supply the selected position time or amplitude. A later
marker can preserve an earlier finite crossing's continuation-above state only
when the native decoded endpoint-code lower bound about the frozen RAW baseline
is at least the RAW threshold. This establishes threshold state in the declared
RAW code metric, not a measured acceleration, true clipping sign or mechanical
amplitude bound. An unknown state still requires fallback. A unique edge first
reached on a marker is ineligible for RAW position even when its above-threshold
state can be proved.

The selection records these explicit reasons when RAW selection is unavailable:

| Reason | Meaning |
| --- | --- |
| `raw_reference_contains_marker_estimates` | The stationary RAW reference contains native marker estimates, so its arithmetic baseline cannot justify a RAW timestamp. |
| `raw_association_history_truncated` | The bounded observer no longer retains all required association evidence. |
| `raw_window_has_unverifiable_samples` | The association window contains a sample whose RAW threshold state cannot be verified. |
| `multiple_raw_threshold_crossings` | More than one provable below-to-above edge is present, including finite and marker edges. |
| `no_associated_raw_threshold_crossing` | No provable below-to-above edge is present in the association interval. |
| `raw_crossing_has_unverifiable_amplitude` | The unique provable edge occurs on a marker, which cannot supply the RAW position timestamp or amplitude. |
| `filtered_not_accepted` | No FILTERED event was accepted; this diagnostic cannot authorize a successful home or completed accuracy output. |

For an accepted FILTERED event, a RAW selection failure uses that accepted
FILTERED threshold crossing as the explicit fallback position. It never uses
an unconfirmed event or a row beyond confirmation. Missing/incompatible profiles
and native backend requirements remain preflight errors rather than fallbacks.

HYBRID retains FILTERED candidate rejection, confirmation, stop-request dispatch
and the existing lookback anchor for the 15 ms impact-response window. This
software policy does not measure physical contact time, MCU halt or end-to-end
stop latency. RAW, pure FILTERED and HYBRID reports retain separate identities
and position policies; do not pool their samples or relabel them as equivalent.

HYBRID's `position_policy` is
`associated_raw_threshold_crossing_v1`. Its control detector remains FILTERED
while `requested_detector=hybrid` identifies the requested measurement mode.
Each home's `position_selection` records the actual RAW selection or explicit
`filtered_threshold_crossing_fallback`, the reason, number of RAW crossings,
association interval and selected sample/time. The completed
`hybrid_position_summary` counts `raw_selected_count`, `fallback_count` and
`fallback_reasons`; the accuracy PNG displays those counts. Discovery remains
separate from the measured series. HYBRID successful PNG receipts use version 5;
pure FILTERED retains version 4 and its accepted-crossing policy. The separate
bounded diagnostic JSON retains schema version 1. Old valid receipts retain
their original schema, policy and interpretation.
`raw_crossing_count` includes provable marker edges even though those edges
cannot be selected for position. Per-home evidence also records
`marker_rows_with_proven_above_code_state`,
`marker_crossings_ineligible_for_position` and the native decoded clip endpoints
used for the code-state bound.

HYBRID measurement does not change the selected ordinary detector after a
completed workflow. CALIBRATE_NOISE and FILTER_CHARTS reject HYBRID.
An early-quality HYBRID ACCURACY failure
aborts with diagnostics and requires manual inspection/recentering; automatic
return and recovery calibration are disabled for that path.

## Default paired HYBRID calibration

CALIBRATE and ACCURACY use HYBRID when `DETECTOR` is omitted. Explicit
`DETECTOR=RAW|FILTERED|HYBRID` remains available; no global default config key is
added. Ordinary G28 and the defaults of TEST, CHARTS, FILTER_CHARTS, NOISE and
Beta keep their existing behavior.

One default or explicit HYBRID CALIBRATE always obtains a fresh RAW discovery
plus five valid RAW checks, automatically recenters, then obtains a fresh
FILTERED discovery plus five valid FILTERED checks. No existing RAW profile is
reused to skip qualification, and no user action is required between series.
Actual controls are RAW for the first series and FILTERED for the second;
there is no third HYBRID-position validation series. Discovery, rejected homes
and retries do not contribute to either five-check result.

A recoverable early hit or excessive spread in either series discards both
series, resets both counts to zero, increases only the failing detector's
candidate and restarts from RAW. Fatal stream/context/geometry/cancellation
errors abort. The same 40-attempt default budget covers all series and retries;
it never resets, and a complete new pair needs at least 12 remaining attempts.
The first qualified profile remains private. Only both qualified series and
successful cleanup permit one final save/restart, with +0.5 g applied once to
each final candidate. No partial-success series is saved or reused.

This creates paired qualification evidence in model schema 3 while preserving
the pre-calibration ordinary G28 selected RAW/FILTERED type. Existing schema
1/2 profiles remain loadable and are not automatically rewritten by an update.
HYBRID ACCURACY can use compatible existing profiles without a new calibration.
See [the complete calibration algorithm](SIMPLE_CALIBRATION.md) for candidate,
attempt, saving and failure details.

Use the [three independent 30-sample commands](ACCURACY.md#compare-30-raw-pure-filtered-and-hybrid-homes)
for the full RAW/pure FILTERED/HYBRID comparison. Each command can optionally
select one completed actual baseline. A chart replay remains diagnostic and
cannot substitute for an independent physical series.

## Operator sequence

If both compatible saved profiles are already available, use the
[30-home RAW, pure FILTERED and HYBRID comparison](ACCURACY.md#compare-30-raw-pure-filtered-and-hybrid-homes)
without recalibrating just for that comparison. A single automatic paired
command can create or refresh both profiles. The numbered pure-mode sequence
is an alternative for individual profile preparation and a separate
same-capture diagnostic, rather than manual stages required by HYBRID.

Before **each** moving command below, follow the
[manual preparation](COMMANDS.md#prepare-every-moving-workflow): printer idle
and unpaused, both XY axes unhomed and physically centered, Z supported,
nozzle/bed clearance, suitable motor enables/currents, accessible emergency
stop. `CENTERED=1` authorizes movement and does not center the toolhead.
Repeat preparation after every successful saving calibration/restart.

To create or refresh both profiles in one command:

```gcode
ACCEL_HOMING_CALIBRATE AXIS=X CENTERED=1
```

It performs RAW five and FILTERED five automatically, with separate discovery,
whole-pair retry and one final save/restart. Manually prepare again after the
restart. Skip calibration when compatible saved profiles already exist and no
refresh is intended.

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

   RAW-controlled captures retain legacy lookback attribution for their FILTERED
   diagnostic shadows, labelled `Estimated filtered lookback onset`; actual
   FILTERED control uses the accepted threshold-crossing position policy.

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
DETECTOR uses HYBRID on CALIBRATE/ACCURACY and the selected model/manual mode on
other commands; choose it explicitly for A/B work.
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

Successful RAW, pure FILTERED and HYBRID ACCURACY commands automatically publish
a per-axis JSON diagnostic beside the existing PNG. `discovery` is separate from
the measured `trials`; a 30-home report therefore retains one discovery record
and 30 measured records. The JSON does not change detector/control parameters,
calibrated thresholds or statistics. The PNG position-policy receipt versions
are described below. Existing
valid PNG baselines remain usable without JSON. Failed workflows publish neither
accuracy artifact. See [the files and timing limits](ACCURACY.md#per-home-json-diagnostics).

Each native event window keeps at most 50 consecutive original signed sensor
rows, with packet identities, original acquisition times, separately labelled
capture times and marker/endpoint-estimate flags. It is not the chart's reduced
display trace. FILTERED diagnostics retain its reference offsets/noise weights,
effective onset gate and actual candidate/rejection counters; replay counters
and their match to the actual counts are reported separately. Its window starts
no earlier than the first armed sample. The bounded
pre-window FILTERED state is reconstructed after stopping and replay-verified
against the original decisions, not captured online. Planned movement-start and
first-step times remain explicitly unavailable. Host stop-request bounds,
returned trigger time and commanded trigger/halt travel do not measure physical
contact or MCU halt. After-cutoff rows and marker estimates retain the same
interpretation limits as the comparison chart.

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
as a baseline. New pure FILTERED actual runs use receipt version 4 and explicitly
record `context.position_policy=accepted_threshold_crossing_v1`, separately from
the numerical detector configuration. Historical receipt versions 1 through 3
remain readable with their original identities and detector configurations;
an absent FILTERED position policy means `filtered_lookback_v1`. Existing valid
RAW baselines remain readable. A/B labels distinguish FILTERED v1 signed from
FILTERED v2 rectified, distinguish HYBRID requested timing from its FILTERED
control, and report position-policy differences as a treatment
difference. Old and new FILTERED series may be compared, but remain separate
measurements; their position references must not be treated as equivalent.
HYBRID uses receipt version 5 for its separate requested mode, raw position
configuration and measured selection/fallback counts. The FILTERED control
identity alone does not make its position policy equivalent to pure FILTERED.
Absent censor metadata is displayed as unknown
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
The successful Accuracy JSON retains only its explicitly bounded event rows,
not that full packet history.

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

New paired HYBRID calibration writes model schema 3. Older code without that
schema cannot load it, even if ordinary homing is selected RAW. A later pure
ten-check calibration can replace one profile while retaining the original
paired certificate for an archived five-check profile; it does not establish a
schema downgrade. Do not relabel five checks as ten or edit the version number.
Before selecting an older target, restore a verified model/configuration
baseline compatible with that target using the reviewed development
uninstall/transaction recovery procedure, preserving later unrelated edits.
See [the host procedure](INSTALLATION.md#explicit-development-branch). The
RAW-recalibration route below applies only to legacy schema 1/2 models that
have never used schema 3.

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

For those legacy schema 1/2 models, while still on development code, use normal operator-supervised
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
