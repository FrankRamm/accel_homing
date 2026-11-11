# Configuration

Use the installer-created configuration as your starting point. Do not paste a
second `[accel_homing]` section or duplicate stepper sections. The annotated
[complete example](../examples/accel_homing.cfg) lists the same options.

## Sensor and endstops

A minimal plugin section is:

```ini
[accel_homing]
accel_chip: auto
```

`auto` requires exactly one recognized configured sensor. With several sensor
sections, select the full section name, preserving its suffix case:

```ini
[accel_homing]
accel_chip: adxl345 ToolHead
```

Configure the sensor's normal MCU, bus, wiring and driver separately. For
Cartographer, select its separate standard `[adxl345 ...]` or `[lis2dw ...]`
object; `[scanner]`/probe distance data is not an accelerometer.

The installed axes use these values in their **existing** stepper sections:

```ini
# In the existing [stepper_x]:
endstop_pin: accel_homing:x
homing_retract_dist: 0

# In the existing [stepper_y]:
endstop_pin: accel_homing:y
homing_retract_dist: 0
```

Keep the machine's correct `position_min`, `position_max`, `position_endstop`,
`homing_positive_dir` and suitable homing speed. Do not copy another printer's
geometry or current. Workflows also require `min_home_dist` absent/zero.
Installation preserves speed unless `--homing-speed` explicitly changes it.

## Thresholds and saved models

Threshold units are **g**. For each axis, restart-time precedence is:

1. Explicit `x_threshold` or `y_threshold`.
2. Explicit legacy shared `threshold`.
3. The selected saved model's threshold.
4. Sensor-aware defaults: normally 8 g for the known 16 g generic drivers,
   4 g for the known 8 g MPU9250 driver, and the native Beacon default.

For example, manual overrides belong in the existing plugin section:

```ini
x_threshold: 0.5
y_threshold: 0.5
```

Manual values must be finite and positive. They have no fixed 0.5/3 g minimum
or 15 g maximum. When the sensor range is known, a threshold must be at most
80% of that full-scale range; the active stream is checked again before arming.
The raw trigger representation must also remain finite and positive. Recognized
sensor rails/Beacon clipping estimates are accepted as bounded observations
with unknown excess amplitude; they do not relax threshold headroom or other
data-fault checks. Do not paste an 8 g threshold for
an 8 g MPU9250 sensor.

To use calibration, comment out both axis overrides **and** the shared
`threshold`, then issue console `RESTART` while idle. An explicit manual
threshold retains the legacy selected-axis detector when its notch is disabled;
a newly calibrated model uses XYZ magnitude. An enabled notch explicitly selects
filtered XYZ magnitude, including for a manual threshold. Values calibrated for
different metrics are not interchangeable. The detector and filter in use are
part of the model/status context.
Both RAW metrics ignore acceleration polarity after subtracting their fresh
signed reference; selected-axis detection uses `abs(a-b)`. Homing direction
still controls commanded travel and does not change the source sensor signs.
Compatible saved XYZ-magnitude profiles retain their thresholds and
qualification. A saved selected-axis profile with legacy/unknown sign-gated
qualification is refused before homing unless it records the current polarity
policy (including the recognized native policy). Recalibrate that axis or
choose an explicit manual threshold; the plugin does not rewrite old models.

`model: default` selects `[accel_homing_model default]` in the generated
SAVE_CONFIG block. Leave the generated **DO NOT EDIT** block alone. Models keep
each axis independently, ten-success qualification and sensor/geometry metadata.
Changed context can prevent a model from being accepted; recalibrate after
reviewing the reported mismatch rather than editing generated metadata.

Successful normal calibration defaults to `SAVE_CONFIG=1`: it writes the
selected axis model and restarts Klipper. `SAVE_CONFIG=0` stages the model only.
Its threshold is the qualified candidate plus a fixed 0.5 g final margin; the
ten qualification homes test the candidate before that addition. Both values
must satisfy known-range headroom. Saving or reloading never adds the margin again.
The former active thresholds are restored when that workflow ends; saving and
reloading are still required to activate the staged model. An ordinary restart
discards unsaved staged values. See [calibration](SIMPLE_CALIBRATION.md).

## Common plugin options

Defaults and bounds below describe the runtime. All numeric values must be
finite. Sensor timing options do not make host-delivered stopping real time.

| Option | Default | Meaning and accepted range |
| --- | --- | --- |
| `accel_chip` | `auto` | Full configured sensor object; exactly one recognized object for auto. |
| `model` | `default` | Saved model name; letters, numbers, underscore and hyphen only. |
| `x_threshold`, `y_threshold` | Model/sensor default | Manual independent thresholds in g; finite positive with known-range headroom. |
| `threshold` | Native default 8 g | Historical shared manual fallback; omit when using saved models. |
| `x_impact_sign`, `y_impact_sign` | `0` | Accepted compatibility values: `-1`, `0`, `1`. RAW impact detection remains independent of acceleration polarity. |
| `homing_accel` | `5000` | Homing/release/early-return acceleration cap in mm/s²; nonnegative; `0` leaves the live limit unchanged. Never raises a lower live acceleration. |
| `back_off_dist` | `5` | Shared fallback for the final successful XY release and cross-axis release in mm; at least 0, no maximum, zero disables. Not a second touch. |
| `x_back_off_dist`, `y_back_off_dist` | `back_off_dist` | Per-axis override of the release distance in mm; at least 0, no maximum, zero disables that axis. There is no Z back-off option. |
| `enable_settle_time` | `0.25` | Seconds after enabling disabled homing motors; 0–2. |
| `baseline_time` | `0.1` | Fresh reference duration in seconds; 0.05–1, with at least 50 contiguous samples. |
| `baseline_batches` | `25` | Retained baseline batch capacity; integer 10–200. |
| `baseline_noise_floor` | `0.7` | Generic legacy quiet-reference floor in g; 0–0.7. Does not apply to the simple vector detector or change native Beacon's separate policy. |
| `stream_timeout` | `0.05` | Independent stream receipt watchdog in seconds (0.01–0.5). Generic data renews it through validated relevant rows. Native Beacon retains its timing policy. |
| `travel_margin` | `1.1` | Full-move travel/time margin multiplier; 1–2. |
| `watchdog_margin` | `0.15` | Additional full-move watchdog allowance in seconds; 0–2. |
| `charts_directory` | `accel_homing_charts/` beside main config | PNG output directory; use a writable path with hard-link support. |

## Experimental notch filter (dev)

| Option | Default | Meaning and accepted range |
| --- | --- | --- |
| `notch_frequency` | `0` | Shared notch frequency in Hz. Finite, nonnegative; zero disables. Enabled frequency must be below half the sensor sample rate. |
| `notch_q` | `10` | Shared notch quality factor; finite, 0.5–1000. Higher Q rejects a narrower band. |
| `x_notch_frequency`, `y_notch_frequency` | `notch_frequency` | Per-homing-axis frequency override. Zero explicitly disables filtering for that machine axis. |
| `x_notch_q`, `y_notch_q` | `notch_q` | Per-homing-axis quality-factor override, with the same accepted range. |

Choose frequency from moving captures; no frequency is selected automatically.
All three signed sensor components use the selected machine-axis profile before
the XYZ magnitude is compared to the threshold. Filter settings belong to the
calibration: changing them requires requalification of affected saved models.
See [configuration, calibration and comparison](NOTCH.md).

## Compatibility and release settings

The compatibility sign options do not select detector polarity. An
`IMPACT_SIGN`-only SET changes the retained setting but does not register an
explicit manual threshold or requalify an old selected-axis model. Supply an
explicit `THRESHOLD` or complete fresh calibration when that model's
qualification is refused.

The back-off distances are not clamped: a target outside the rail limits fails
G28 like Kalico's own retract ("Move out of range"). They respect the existing
firmware release and do not repeat an already sufficient release. It applies to ordinary
successful homes and the final qualified axis of a workflow. Suspect,
failed or cancelled trials do not receive a speculative final release.
No plugin final release requests a Z move.
The status field `back_off_dist` (`get_status()`, as read by macros through
the plugin's printer object) is a per-axis dict such as
`{"x": 5.0, "y": 50.0}`; `ACCEL_HOMING_STATUS` prints both values.

## Calibration options

| Option | Default | Meaning and accepted range |
| --- | --- | --- |
| `calibration_start_threshold` | `1.0` | Noise-selection grid origin in g; at least 0.5. |
| `calibration_step` | `0.25` | Candidate increment in g; 0.1–3. |
| `calibration_max_attempts` | Ignored | Deprecated compatibility option; accepted without numeric validation, with a warning. |
| `calibration_confirm_timeout` | `300` | Pending calibration confirmation timeout in seconds; 30–1800. |
| `calibration_travel_tolerance` | `0.20` | Midpoint-to-endstop plausibility fraction; 0.05–0.40. |
| `workflow_reposition_speed` | `50` | Return-to-center speed in mm/s; 1–150. |
| `calibration_settle_time` | `0.25` | Extra seconds after completed CALIBRATE/ACCURACY midpoint returns and early reversals; 0–2. |

There is no configurable calibration g ceiling or fixed 15 g maximum.
`calibration_max_threshold` is deprecated: it is accepted but ignored with a
warning. Ordinary and Beta calibration have no total homing-attempt limit;
`calibration_max_attempts` is also deprecated and ignored. The sensor-range
headroom and finite representation checks still apply. Calibration/model
thresholds have a 0.5 g minimum.

## Generic sensor options

These apply to configured batch accelerometers rather than the native Beacon
raw adapter. Known standard driver ranges are listed in
[compatibility](COMPATIBILITY.md#sensors).

| Option | Default | Meaning and accepted range |
| --- | --- | --- |
| `stream_interval` | `0.005` | Requested reporting interval in seconds; 0.001–0.020 and strictly less than `max_sample_age`. Restored afterward. |
| `max_sample_age` | `0.070` | Nominal reporting and selected-axis startup-deadline assumption in seconds (0.005–0.070). Existing lower values remain planning inputs. |
| `min_sample_rate` | `1000` | Minimum delivered/configured rate in Hz; 800–10000. |
| `sensor_range` | Known driver range | Actual full-scale g range; positive, at most 1000; required for an unknown compatible object. |
| `max_report_travel` | Disabled when absent | Optional nominal reporting-distance policy, 0.1–2.0 mm. |
| `max_startup_travel` | Disabled when absent | Optional nominal startup-request-distance policy, 0.1–5.0 mm. |

Generic samples from the current stream/reference/measurement window remain
eligible when they arrive 80 ms or more after their sensor timestamp. A fixed
stream-start cutoff excludes prior-generation warmup; measurement cutoffs
still exclude earlier reference/window data, and pre-arm impacts cannot hit.
The 5 ms future guard, continuity, rate, error/overflow, frozen-data checks and
full-move deadlines remain. Receipt silence is independent: only validated
relevant rows renew `stream_timeout`. Existing initial acquisition allowances
apply before a window receives its first eligible samples.

Generic status reports `sample_age_limit_enforced: false` and null actual
`max_sample_age_s` / reference `max_sample_age_ms` limits. The retained nominal
value is labelled `configured_max_sample_age_s` (or `_ms` in reference status).
Receipt timing has its own `last_receipt_time_s` / `receipt_timeout_ms` fields.
The selected-axis startup deadline still includes the nominal age assumption
when optional travel budgets are absent. `stream_timeout` remains independent
of that assumption.
Native Beacon's timestamp-age protection is unchanged.

Leave optional limits absent to disable them; do not write `None`. No default
50 mm/s cap or startup-distance cap is added. Explicit limits can reject a
configured plan, including after a macro lowers live acceleration. They do
not automatically edit speed, acceleration or receipt timeouts. They describe nominal
command travel before a host request, not physical stopping distance. See
[the formulas and limitations](DETAILS.md#timing-and-optional-travel-policies).

## Historical Beacon names

`[beacon_accel_homing]`, `beacon: beacon`, `beacon_accel:x/y`,
`BEACON_ACCEL_HOMING_STATUS` and `SET_BEACON_ACCEL_HOMING` remain supported.
Load only one of `[accel_homing]` and `[beacon_accel_homing]`.
The [legacy example](../examples/beacon_accel_homing.cfg) is for an existing
configuration, not an additional section to paste beside a new installation.
