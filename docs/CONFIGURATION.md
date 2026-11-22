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

Successful calibration replaces the selected axis's old manual value by saving
`x_threshold` or `y_threshold` alongside its qualified model. You do not need to
comment out those overrides before calibrating. Existing selected-axis entries
are commented when `SAVE_CONFIG` writes the new setting, including entries in
included configuration files. The shared `threshold` is retained for the other
axis; the new per-axis value takes precedence for the calibrated axis.
In the main `printer.cfg`, keep section headers and threshold option names
unindented so the native SAVE_CONFIG serializer can comment replaced entries.
An unsupported main-file layout is reported before included files are modified.

A later explicit manual threshold changes the numeric threshold. When a valid mapped-axis model exists,
its configured or learned orientation and mapped detector remain active with that manual
value. Without a mapped model, manual thresholds retain the legacy configured
channel behavior. RAW detection subtracts its fresh signed reference before
the absolute value; homing direction controls commanded travel, not polarity.
The detector and mapping in use are part of the model/status context.
Older XYZ-magnitude profiles require fresh per-axis calibration before ordinary
homing, including when a manual threshold is present. Their qualification is
not transferable to the mapped metric. A saved selected-axis profile with legacy/unknown sign-gated
qualification is refused before homing unless it records the current polarity
policy (including the recognized native policy). Recalibrate that axis or
choose an explicit manual threshold; the plugin does not rewrite old models.

`model: default` selects `[accel_homing_model default]` in the generated
SAVE_CONFIG block. Leave the generated **DO NOT EDIT** block alone. Models keep
each axis independently, ten-success qualification and sensor/geometry metadata.
New calibration stores `impact_metric: mapped_axis` and a versioned
`axis_mapping` with machine axis, source channel, sign and selection method.
An explicit `accel_axes_map` in the selected Beacon section, or `axes_map` in
the selected other sensor section, uses method `configured_axes_map_v1` and
records that declaration. Included settings and explicit `x,y,z` are recognized;
omitted identity defaults are not. Without an explicit declaration, automatic
mapping uses `bidirectional_velocity_v1` with measured confidence and diagnostics.
Its coordinate frame is the configured sensor stream, after the driver's map.
The plugin does not edit or apply that setting twice. X/Y homing
uses only its own selected physical component; raw XYZ remain available for
diagnosis. A changed sensor/orientation/axes map requires recalibration.
Changed context can prevent a model from being accepted; recalibrate after
reviewing the reported mismatch rather than editing generated metadata.

Successful normal calibration defaults to `SAVE_CONFIG=1`: it writes the
selected axis model and runtime threshold, then restarts Klipper.
`SAVE_CONFIG=0` stages both in memory, without changing configuration files.
Its threshold is the qualified candidate plus a fixed 0.5 g final margin; the
ten qualification homes test the candidate before that addition. Both values
must satisfy known-range headroom. Saving or reloading never adds the margin again.
The generated threshold is tied to its qualified model and keeps the model's
sensor, geometry and speed checks. A later user-written override remains a
manual override, even when its numeric value matches the calibration.
The former active thresholds are restored when that workflow ends; saving and
reloading are still required to activate the staged values. An ordinary restart
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
| `back_off_dist` | `5` | Final successful XY release and cross-axis release in mm; 0–20, zero disables. Not a second touch. |
| `enable_settle_time` | `0.25` | Seconds after enabling disabled homing motors; 0–2. |
| `baseline_time` | `0.1` | Fresh reference duration in seconds; 0.05–1, with at least 50 contiguous samples. |
| `baseline_batches` | `25` | Retained baseline batch capacity; integer 10–200. |
| `baseline_noise_floor` | `0.7` | Generic legacy quiet-reference floor in g; 0–0.7. Does not apply to the simple mapped-axis detector or change native Beacon's separate policy. |
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

Choose frequency from moving captures, or use
`ACCEL_HOMING_CALIBRATE AXIS=Y CENTERED=1 NOTCH=AUTO` for automatic selection.
AUTO defaults to report-only and stages nothing; explicit `SAVE_CONFIG=1`
measures, qualifies and saves the chosen per-axis profile and model together.
For a mapped-axis calibration, the stop decision uses the filtered component
for that machine axis. Filter settings belong to the
calibration: changing them requires requalification of affected saved models.
See [configuration, calibration and comparison](NOTCH.md).

## Compatibility and release settings

The compatibility sign options do not select detector polarity. An
`IMPACT_SIGN`-only SET changes the retained setting but does not register an
explicit manual threshold or requalify an old selected-axis model. Supply an
explicit `THRESHOLD` or complete fresh calibration when that model's
qualification is refused.

`back_off_dist` is bounded by the rail limits and existing firmware release;
it does not repeat an already sufficient release. It applies to ordinary
successful homes and the final qualified axis of a workflow. Suspect,
failed or cancelled trials do not receive a speculative final release.
No plugin final release requests a Z move.

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
