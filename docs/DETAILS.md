# Technical reference

Start with [configuration](CONFIGURATION.md) and [console commands](COMMANDS.md)
for operation. This page explains detector, preparation and transport policies.
The [calibration sequence](SIMPLE_CALIBRATION.md) and
[accuracy recovery](ACCURACY.md) are documented separately.

## Detector paths

The core registers host-triggered X/Y endstop aliases and uses the stepper
MCU's trigger dispatch to stop a homing move. Acceleration reaches Python on
the host before it requests that stop. This is not an MCU-local contact switch
or a hard real-time endstop.

| Context | Metric and preparation |
| --- | --- |
| New simple calibration | Fresh XYZ resting offset; magnitude at each sample; no quiet-range gate or integrated motion proof. |
| New saved model with `impact_metric: xyz_magnitude` | Same vector detector during ordinary homing, TEST, CHARTS and ACCURACY after reload, unless explicit config thresholds take precedence. |
| Development FILTERED model | Versioned causal XYZ filtering and temporal confirmation; the accepted candidate's threshold crossing supplies the position timestamp. |
| Explicit development HYBRID workflow | FILTERED confirmation with an associated RAW XYZ threshold crossing for position; explicit FILTERED fallback when RAW selection is unavailable or ambiguous. Both compatible saved profiles are required. |
| Explicit config thresholds / older models | Legacy machine-axis detector with fresh quiet reference and its motion/pressed policies. |
| Beta calibration | Isolated moving-noise experiment and proposal; does not replace or save the ordinary model. |

The vector detector evaluates
`sqrt((ax-bx)^2 + (ay-by)^2 + (az-bz)^2)` in g about a fixed fresh reference.
The first sample reaching the threshold requests stopping; a later peak does
not delay that request. Channel permutations/sign reversals do not change the
magnitude. Motion, vibration, binding and intended stop contact can all produce
the same threshold crossing. The full-move deadline and data-fault checks remain.

Development FILTERED v1/v2 models use the existing threshold crossing of the
candidate that actually confirms as onset time/level/sample identity under
`accepted_threshold_crossing_v1`. Confirmation, rejection, stop-request dispatch
and the original lookback-anchored 15 ms response window retain their behavior.
Saved calibrated thresholds and both profiles remain compatible without
recalibration. Historical FILTERED measurements retain `filtered_lookback_v1`
when their receipt has no explicit position policy; comparisons name that
difference. See [filter policies and baseline receipts](FILTERS.md).

Explicit HYBRID TEST/ordinary CHARTS/ACCURACY uses
`associated_raw_threshold_crossing_v1` for position selection while retaining
FILTERED confirmation and the original response-window anchor. A separate
bounded RAW observer uses the saved XYZ threshold and its own arithmetic
reference from the same stationary window. It supplies a timestamp only after
the FILTERED candidate confirms; it never independently requests a stop.
Selection provenance, fallback reasons and measured RAW/fallback counts remain
visible in accuracy JSON/PNG. This does not establish physical stop latency.
See [HYBRID requirements and limits](FILTERS.md#hybrid-position-timing).

Changing a threshold through temporary SET does not save a new detector/model.
Persistent manual overrides take precedence over saved model values on reload.
See [threshold precedence](CONFIGURATION.md#thresholds-and-saved-models).

## Fresh reference and legacy quiet policies

Every pass completes queued setup/release/backoff, enables the required motors
(CoreXY needs both drives), waits `enable_settle_time` after newly enabling them,
and acquires a fresh contiguous reference. The default `baseline_time` is
0.1 s with at least 50 samples. Earlier buffered impacts cannot supply that
fresh reference. Preparation can time out if no usable window arrives.

Simple vector references estimate only a fresh resting offset; the quiet gates
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
deviation from the accepted mean; uncertain signed motion keeps raw impacts
bidirectional. Expanded native references retain bounded motion/pressed checks.
Ambiguous startup fails with `no_motion`; wider admission does not prove that
a low threshold or every acceleration is usable.

These observed bounds do not guarantee future noise. Preparation does not
measure physical motor current, prevent a hold-to-run transient, or prove that
a carriage moved. Data validity, freshness and raw impacts remain separate.

## Timing and optional travel policies

Generic defaults request a 5 ms report interval and permit sample age up to
50 ms. `stream_interval` must remain strictly less than `max_sample_age`.
The adapter temporarily shortens driver batching and restores the prior
interval afterward. A configured lower stream timeout/freshness setting is
preserved. Native Beacon uses its distinct raw path.

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
acceleration in mm/s², `age` the backend freshness bound and `interval` the
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
made by other macros. Simple vector detection omits the integrated startup
proof, while applicable generic plan/admission checks remain in place.

Missing/stale data, invalid/nonmonotonic timestamps, rate/error/range faults and
the full-move deadline are not relaxed to admit a requested speed. Finite input
and finite derived raw trigger limits/deadlines are required.

## Calibration qualification and saving

Current simple calibration uses fresh three-window stationary noise to select
an initial candidate, then requires an impact beyond 20 mm inside the
midpoint-to-endstop travel band. It excludes discovery and requires ten fresh
validation homes at the candidate threshold with travel range at most 2 mm.
After qualification, the saved threshold is that candidate plus 0.5 g once,
subject to sensor headroom. Metadata distinguishes the tested candidate from
the higher saved value, which those homes did not test.

Premature impacts reverse recorded signed **start-to-halt** command travel,
not merely trigger travel; the workflow then settles, increases the candidate
and starts a new discovery. A running spread above 2 mm likewise rejects the
candidate. A new candidate needs eleven remaining monitored homes.
No encoder verifies those returns or the intended stop.

SAVE_CONFIG stores a named per-axis model and normal calibration defaults to
automatic save/restart. Explicit manual thresholds retain priority.
ACCURACY early-trigger recovery additionally stages the failing axis's explicit
override when necessary so its saved replacement is effective after reload;
native save/include conflicts are reported instead of editing includes by hand.

See [the full algorithm and exact boundaries](SIMPLE_CALIBRATION.md) and
[accuracy recovery](ACCURACY.md).

## Output and diagnostics

STATUS retains active workflow phase/candidate/attempts, model and last result,
including bounded preparation and legacy motion-proof summaries when present.
Diagnostics may report current/best quiet candidate spans, per-axis peak-to-peak
g, sample/batch ages, rejection details and cleanup state. Those ages are
snapshots at the reported event, not continuously updated live measurements.

Charts are PNG-only bounded diagnostic traces published only for successful
workflows. Per-home ACCURACY traces remain in memory; failures retain console
statistics without a chart. A successful Beta proposal emits a labelled PNG;
its same-command console diagnostics also cover failures. See [plots](PLOTS.md) and
[Beta](BETA.md).

The source/API references and unresolved physical limits are collected in
[compatibility](COMPATIBILITY.md). Test evidence belongs to its exact revision
and environment; see [offline verification](../TESTING.md) rather than treating
old passing counts as current hardware evidence.
