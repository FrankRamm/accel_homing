# Technical reference

[← Documentation](../README.md)

Start with [configuration](../guide/configuration.md) and [console commands](../guide/commands.md)
for operation. This page explains detector, preparation and transport policies.
The [calibration sequence](../guide/calibration.md) and
[accuracy recovery](accuracy.md) are documented separately.

## Detector paths

The core registers host-triggered X/Y endstop aliases and uses the stepper
MCU's trigger dispatch to stop a homing move. Acceleration reaches Python on
the host before it requests that stop. This is not an MCU-local contact switch
or a hard real-time endstop.

| Context | Metric and preparation |
| --- | --- |
| New simple calibration | Explicit sensor map, or gentle bidirectional central mapping if absent; fresh XYZ resting offset; absolute selected physical-axis component at each sample; no quiet-range gate or integrated homing-motion proof. |
| New saved model with `impact_metric: mapped_axis` | Same mapped detector during ordinary homing, TEST, CHARTS, ACCURACY and TROUBLESHOOT after reload; persisted mapping is tied to sensor context. |
| Older saved `xyz_magnitude` qualification | Requires fresh per-axis calibration; cannot be silently reused as mapped-axis qualification. |
| Manual threshold with a valid mapped model | Manual scalar threshold with the selected physical-axis orientation and mapped reference policy. |
| Unmapped manual thresholds / accepted legacy selected-axis models | Absolute configured-channel deviation about a fresh quiet reference, with legacy motion/pressed policies. |
| Beta calibration | Isolated moving-noise experiment and proposal; does not replace or save the ordinary model. |

The mapped detector evaluates
`abs(sign * (a[source_axis] - baseline[source_axis]))` in g about a fixed fresh
reference. The selected source channel is in the already configured sensor
stream, after driver `axes_map`; raw XYZ capture rows are not reordered.
The first sample reaching the threshold requests stopping; a later peak does
not delay that request. Other channels do not contribute to this metric.
Motion, vibration, binding and intended stop contact can all produce
the same threshold crossing. The full-move deadline and data-fault checks remain.
Beta, historical captures and explicit XYZ experiments retain vector support.

The selected-axis RAW detector similarly compares `abs(a-b)` with its
threshold; homing direction does not select an acceleration polarity.
Reference subtraction always precedes the absolute value. Source protocol
values, reference offsets and the signed evidence used for motion proof and
recovery retain their signs. PNG component curves apply the same absolute
display convention; travel and relative times keep their existing meaning.

Known finite sensor rails remain valid bounded observations. Native Beacon's
clipping sentinel has no measured sign or amplitude; it is decoded to a
bounded endpoint estimate using prior sensor sign. Diagnostics distinguish
these markers from finite rail values, disclose unknown excess amplitude and
retain separate capture and resting-reference scopes. Other invalid or
non-finite data still faults. These observations do not relax threshold
headroom or establish a physical impact peak.

Changing a threshold through temporary SET does not save a new detector/model.
Persistent manual overrides take precedence over saved model values on reload.
Successful calibration replaces its selected-axis override through SAVE_CONFIG;
the generated paired threshold retains the qualified model's context checks.
See [threshold precedence](../guide/configuration.md#thresholds-and-saved-models).

## Fresh reference and legacy quiet policies

Every pass completes queued setup/release/backoff, enables the required motors
(CoreXY needs both drives), waits `enable_settle_time` after newly enabling them,
and acquires a fresh contiguous reference. The default `baseline_time` is
0.1 s with at least 50 samples. Earlier buffered impacts cannot supply that
fresh reference. Preparation can time out if no usable window arrives.

Simple mapped-axis references estimate only a fresh resting offset; the quiet gates
below do not apply to them. They still need valid, fresh, contiguous data.

For the **generic legacy detector**, each sensor axis's stationary peak-to-peak
limit is:

```text
max(0.20 * active_threshold_g, baseline_noise_floor)
```

The default floor is 0.7 g, configurable from 0 to 0.7. Zero restores the old
20% rule. This does not increase the impact threshold. Native Beacon has its
own bounded policy:

```text
max(0.20 * active_threshold_g, min(0.5, active_threshold_g))
```

For references admitted above the older 20% gate, legacy motion proof discounts
the observed per-axis noise bounds. Generic proof uses the maximum absolute
deviation from the accepted mean. Signed integration supplies motion-proof
evidence and does not select impact polarity: RAW impacts always use absolute
deviation from the reference. Expanded native references retain bounded
motion/pressed checks.
Ambiguous startup fails with `no_motion`; wider admission does not prove that
a low threshold or every acceleration is usable.

These observed bounds do not guarantee future noise. Preparation does not
measure physical motor current, prevent a hold-to-run transient, or prove that
a carriage moved. Data validity, delivery continuity and raw impacts remain separate.

## Timing and optional travel policies

Generic defaults request a 5 ms report interval. `max_sample_age` remains a
70 ms nominal planning assumption. Its existing numeric bounds and
`stream_interval < max_sample_age` configuration rule remain for compatibility.
It is not a past-timestamp acceptance limit or warmup trimming boundary.
Valid current-window samples can arrive 80 ms or more after measurement,
including during preparation, homing, stationary acquisition and Beta.
The adapter temporarily shortens driver batching and restores it afterward.

Generic liveness uses monotonic receipt of validated relevant rows and the
independent `stream_timeout`, default 50 ms. Pre-arm-only callbacks cannot
keep an armed move alive. Reference arming and stationary/Beta windows also
check receipt silence independently of sensor timestamp age.
Stationary acquisition must complete within the window duration plus 0.5 s.
This existing deadline also allows an initial delivery delay. The Beta
response tail retains its separate bounded wait. Stream/reference/window cutoffs, the
5 ms future guard and continuity/rate/error/overflow/frozen-data safeguards
remain. Native Beacon retains its separate timestamp-age policy.

Explicit reporting/startup distance policies still use the configured nominal
age. A higher value can tighten their speed limits. The selected-axis startup
deadline also includes this assumption when optional travel budgets are absent.
Regular delayed receipts can satisfy `stream_timeout` while the carriage
continues travelling or pushing until the host receives and acts on an older
contact sample. Nominal travel settings cannot cap that transport delay or
the resulting physical distance.

There is no default 50 mm/s cap, reporting-distance cap or startup-distance
cap. Configured finite positive homing speed is retained, subject to normal
live toolhead limits, numeric validity, sensor delivery and any explicit
policies. `homing_accel` caps live acceleration without raising a lower value.

Optional **generic** settings are independent:

| Policy | Accepted value | Test |
| --- | --- | --- |
| `max_report_travel` | 0.1–2.0 mm | Effective speed ≤ limit / `max_sample_age`. |
| `max_startup_travel` | 0.1–5.0 mm | Nominal travel before the host startup-stop request ≤ limit. |

Leave either absent to disable that policy. They never silently choose a speed
or change timing/acceleration. Installer flags of the same names opt in
explicitly and are rejected for native Beacon.

For legacy startup proof, let `v` be effective speed in mm/s, `a` effective
acceleration in mm/s², `age` the configured nominal age assumption and `interval` the
watchdog/report interval supplied by that path. The shared helper calculates:

```text
proof_window = max(0.004, min(0.050, 0.050 / v))
startup_timeout = v/a + 0.015 + age
                  + max(0.010, 2*interval, proof_window)
nominal_startup_request_travel = 0.5*v*(v/a)
                  + v*(0.018 + age
                       + max(0.010, 2*interval, proof_window))
```

The travel expression includes one 3 ms startup watchdog allowance.
It excludes host stalls, transport/MCU stopping delay and mechanical response.
A policy refusal describes a numerically planned command, not a measured
stopping distance. Runtime checks use current live limits, including changes
made by other macros. Simple mapped-axis detection omits the integrated startup
proof, while applicable generic plan/admission checks remain in place.

Receipt silence, future/invalid/nonmonotonic timestamps, rate/error/range faults and
the full-move deadline are not relaxed to admit a requested speed. Finite input
and finite derived raw trigger limits/deadlines are required.

## Calibration qualification and saving

Current simple calibration uses the selected sensor's explicit map, or learns
a physical-axis channel/sign from gentle central bidirectional motion when
the option is absent, rejecting ambiguous measured results. It then uses
fresh three-window stationary noise on that mapped component to select
an initial candidate, then requires an impact beyond 20 mm inside the
midpoint-to-endstop travel band. It excludes discovery and requires ten fresh
validation homes at the candidate threshold with travel range at most 2 mm.
After qualification, the saved threshold is that candidate plus 0.5 g once,
subject to sensor headroom. Metadata distinguishes the tested candidate from
the higher saved value, which those homes did not test.

Premature impacts reverse recorded signed **start-to-halt** command travel,
not merely trigger travel; the workflow then settles, increases the candidate
and starts a new discovery. A running spread above 2 mm likewise rejects the
candidate. Ordinary and Beta calibration have no total monitored-home limit;
the deprecated `calibration_max_attempts` option is accepted without numeric
validation and ignored with a warning. Qualification still requires discovery
plus ten checks. Cancellation, disconnects and sensor/motion faults still abort.
No encoder verifies those returns or the intended stop.

SAVE_CONFIG stores a named per-axis model and normal calibration defaults to
automatic save/restart. Explicit manual thresholds retain priority.
ACCURACY early-trigger recovery additionally stages the failing axis's explicit
override when necessary so its saved replacement is effective after reload;
native save/include conflicts are reported instead of editing includes by hand.

See [the full algorithm and exact boundaries](../guide/calibration.md) and
[accuracy recovery](accuracy.md).

## Output and diagnostics

STATUS retains active workflow phase/candidate/attempts, model and last result,
including bounded preparation and legacy motion-proof summaries when present.
Diagnostics may report current/best quiet candidate spans, per-axis peak-to-peak
g, sample/batch ages, rejection details and cleanup state. Those ages are
snapshots at the reported event, not continuously updated live measurements.

`home_wait` returns the onset sample time, so Klipper takes the trigger
position at that sample and the halt position where the steppers stopped.
Every homing-move record therefore carries `post_trigger_mm`, the absolute
difference of `halt_travel_mm_signed` and `trigger_travel_mm_signed`: travel
commanded into the stop after contact, roughly speed times sensor delivery,
host, stop-request and MCU stop delay. It is `null` without both step
positions. `stop_request_after_onset_ms` is the main-MCU print-time estimate
taken right after the stop request was sent, minus the onset sample time; both
are print times. It is `null` for stops other than an impact, without a clock
estimate, and for ACCEL_HOMING_EXPERIMENT decisions, whose recorded time is a
host decision time rather than a sample time. After an impact with more than
1 mm of post-trigger travel, one console notice per homing move explains the
push. Neither field nor the notice changes detection, motion or the homed
position. See [lost steps after homing](../guide/troubleshooting.md#the-axis-loses-position-or-skips-steps-after-homing).

`start_position_mm` and `expected_travel_mm` describe the forced start position
Klipper applies during homing (1.5 times the rail range). The record also
carries `axis_was_homed` and, for the first pass of a homed axis,
`believed_distance_to_stop_mm` from the commanded position at
`homing:home_rails_begin`, before the forced position. After the move,
`believed_minus_trigger_travel_mm` is that distance minus the measured trigger
travel. These are diagnostic only.

`min_home_dist` judges the first pass of each G28 (the long approach). With
`homing_retract_dist` above zero the second touch only travels the retract;
acceptance, the final release and record identity still use the final pass.

With a notch enabled, every armed packet is checked against the filter rate.
For decoded batches, the packet period must be within 2%, and the interval
across a packet boundary within 2% of the packet period. As in the offline
gentle analysis of decoded batches, one boundary interval may instead deviate
up to 4% from the filter period when its average with the previous and with
the current packet period is within 2% of it; a boundary after a packet with a
single eligible sample gets no such allowance. Native Beacon packet periods and
boundary intervals use the 25% Beacon packet tolerance, and the span from the
previous packet's first sample to the current packet's last must be within 2%
of the filter rate. A lost sample or a sustained rate change still stops the
move with `filter_rate`. On Beacon a sustained change below 25% is found at
the latest one packet later, when the two-packet span covers it.

Charts are PNG-only bounded diagnostic traces published only for successful
workflows. Per-home ACCURACY traces remain in memory; failures retain console
statistics without a chart. A successful Beta proposal emits a labelled PNG;
its same-command console diagnostics also cover failures. See [plots](plots.md) and
[Beta](beta.md).

Displayed acceleration components are nonnegative after signed-reference
subtraction. Bounded envelopes preserve signed extrema and absolute troughs;
they are not a full-resolution export. Range notes aggregate all displayed
homes/windows and report limited references separately without double counting.

The source/API references and unresolved physical limits are collected in
[compatibility](../guide/compatibility.md). Test evidence belongs to its exact revision
and environment; see [offline verification](../development/testing.md) rather than treating
old passing counts as current hardware evidence.
