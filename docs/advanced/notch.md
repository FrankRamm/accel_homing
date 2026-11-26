# Experimental notch filter (dev)

[← Documentation](../README.md)

The optional notch attenuates a narrow frequency band during accelerometer
homing. It is disabled by default. `NOTCH=AUTO` can select frequency and Q
during calibration; manual configuration remains available. Profiles apply to
the measured machine, sensor, speed and acceleration.

A notch can also attenuate an impact, produce ringing or change the threshold
crossing time. Its usefulness must be established by comparing complete runs;
a smoother acceleration trace does not establish better physical homing.

## Automatic selection

After the [manual preparation](../guide/commands.md#prepare-every-moving-workflow), run:

```gcode
ACCEL_HOMING_CALIBRATE AXIS=Y CENTERED=1 NOTCH=AUTO SAVE_CONFIG=0
```

Use `AXIS=X` for X. This command moves the selected axis. AUTO defaults to
`SAVE_CONFIG=0`: it reports its result, restores the prior filter/threshold
settings and stages no configuration. A later bare `SAVE_CONFIG` therefore
does not apply this report. To measure, qualify and save in one run, prepare
XY again and explicitly request:

```gcode
ACCEL_HOMING_CALIBRATE AXIS=Y CENTERED=1 NOTCH=AUTO SAVE_CONFIG=1
```

For saving, remove explicit shared or selected-axis manual thresholds so the
saved model controls subsequent homing. A matching threshold/model pair created
by ordinary calibration and native SAVE_CONFIG is recognized automatically; it
does not need manual removal. The command rejects conflicting manual
overrides and unrelated pending configuration changes before movement.
Successful saving writes the selected axis filter and model together through
native configuration saving and restarts Klipper. It also saves the selected
axis runtime threshold, so the qualified model controls subsequent homes. The
other axis is preserved unless the fresh mapping proves its saved mapping is
stale or selects the same sensor channel; that mapping is invalidated and must
be recalibrated.

The sequence is:

1. Establish a fresh signed sensor mapping with the ordinary bidirectional
   mapping strokes, or use the sensor's explicitly configured `axes_map`
   (`accel_axes_map` for Beacon). A previous saved inferred mapping is not reused.
2. Record three independent central strokes at the existing effective homing
   speed, acceleration and homing-current conditions. The stroke stops halfway
   from the assumed center to the configured directional endstop and returns
   to the assumed center. Actual planner segments identify acceleration,
   cruise and braking; the speed is not reduced to make a short stroke fit.
3. With the selected notch temporarily disabled, obtain a discovery and ten
   qualifying reference homes using the ordinary monitored calibration loop.
   Return to the assumed center and settle before offline analysis.
4. Derive persistent frequencies from the first two central-stroke cruise
   recordings. Evaluate a bounded set of frequencies and Q values with the
   same causal filter used during homing. The first six reference homes are
   training data; the third stroke and last four homes are held out.
5. Freeze the selected profile and threshold, then test that choice against
   the held-out recordings. These recordings can reject the selection, not
   retune it. Leaving the notch disabled is allowed only if the original raw
   threshold also passes all independent noise and reference-stop checks.
6. Require a new discovery and ten new qualifying homes with that fixed
   profile and threshold. A failed final check aborts instead of changing the
   selected threshold. AUTO saves exactly the tested threshold, with no
   additional untested 0.5 g margin.

A clean run therefore includes the mapping strokes when needed, three
independent central strokes and 22 homes. The raw
reference phase can need additional attempts under the existing retry policy,
including the calibration stops for a repeatable premature contact and the
third spread rejection ([threshold validity](../guide/calibration.md#threshold-validity-and-homing-attempts)).
AUTO needs usable unfiltered reference homes; it cannot recover every case
in which raw homing fails. Missing data, insufficient cruise duration, clipping
or rate changes are reported as unusable evidence. Native Beacon reports one
time span per packet, so its sample intervals alternate by several percent
between packets. For gentle selection on a native Beacon, each interval must
stay within the 25% packet-period tolerance the runtime stream checks already
use (a lost sample doubles one interval and is rejected), the capture's mean
rate must be within 2% of its reference and the acquisition receipt must report
no gap, drop or fault. Decoded drivers keep their stricter interval checks.
A rejected notch may fall
back to the already-tested disabled control; if that control also fails, the
run is rejected without selecting or saving a profile.

Analysis uses the signed mapped sensor component for spectra and its absolute
value for thresholds, matching the live detector. In gentle mode, post-mapping
captures retain time and signed acceleration for that machine axis. The ZIP
records the source sensor channel and sign so the trace can be replayed.
All recordings must have the same validated mapping and sensor context. It excludes braking from the noise score
and retains acceleration/startup checks so filtering cannot hide a new early
trigger. Reference-home replay ends at the original raw trigger sample;
post-trigger braking caused by that original stop is not evidence that a
filtered home would have stopped. These are plausible reference stop events,
not independent measurements of physical contact. Fresh-home repeatability
also remains commanded-travel repeatability.

Each idle AUTO command announces its diagnostic directory immediately and
finishes with one shareable ZIP beside that directory. The ZIP contains:

- the selection report and comparison PNG when those phases completed;
- full raw captures for every acquired discovery, reference, retry, rejected
  and fresh qualification home, plus mapping, moving-noise and stationary
  captures, independently of which captures qualify for analysis;
- the command arguments, timestamped phases, exception traceback, cleanup and
  sensor/context diagnostics in `run.json` and `debug/context.json`;
- the available Klippy log interval, effective relevant configuration,
  firmware/runtime identity and installed module fingerprints;
- analysis-worker logs and interpreter/dependency metadata when the worker ran.

A caught failure or cancellation still produces a ZIP with the available partial
evidence. Idle preflight rejections also produce a report. A command rejected
while another workflow, print or queued motion is active does not start a new
AUTO run or perform diagnostic file work. Unavailable data, capture omissions
and export errors are recorded explicitly. Raw diagnostic storage is bounded to
1 GiB of compressed captures per run, with 64 MiB of JSON per capture. Keep the
whole ZIP, including rejected candidate reasons and failed-run evidence.

For `SAVE_CONFIG=1`, the complete ZIP must exist before native saving is called.
That ZIP is marked `before_native_save`: it includes the finished qualification
and runtime cleanup, while the save/restart result was still pending. A small
`completion.json` in the run directory records the returned native-save result without
waiting on a reactor that may already be stopping. On a save failure, the ZIP is
refreshed with the error where possible; a failed refresh preserves the previous
valid ZIP and reports that its snapshot is older. A power loss, process kill or
unrecoverable storage failure cannot guarantee a final ZIP; the announced
partial directory contains the captures already written.

Diagnostic serialization and ZIP creation yield in bounded chunks after motion
stops. They recheck that no homing, printing or queued motion has started. This
evidence does not independently verify physical contact or stopping accuracy.

## Manual configuration

Add options to the existing `[accel_homing]` or `[beacon_accel_homing]` section,
not a second section. These values leave filtering disabled:

```ini
notch_frequency: 0
notch_q: 10
x_notch_frequency: 0
y_notch_frequency: 0
```

Set `y_notch_frequency` to the measured frequency in Hz to filter Y homes only.
`x_notch_frequency: 0` explicitly keeps X unfiltered, including when the shared
`notch_frequency` is positive. An omitted axis option inherits the shared value.
`x_notch_q` and `y_notch_q` can likewise override the shared `notch_q`.

The frequency must be finite and nonnegative. Zero disables that axis's filter;
an enabled frequency must be below half the sensor sampling rate. `notch_q`
must be finite and between 0.5 and 1000. A higher Q rejects a narrower band;
a lower Q affects more nearby frequencies. The default Q is a starting setting,
not a measured optimum for a printer. Numerically degenerate coefficient sets
are rejected even if the entered frequency is nominally below Nyquist.

X/Y options select the **machine axis being homed**. Each enabled profile applies
identical independent filters to the signed sensor components, then selects
only the calibrated component for the homing threshold. For example, a Y home
whose fresh map selects sensor X uses the absolute filtered X component. The
other channels are retained for diagnostics and do not trigger this mapped home.

## Calibrate after changing the filter

After updating the plugin, follow the normal service-update instructions.
Later configuration-only filter changes need a Klipper `RESTART` while idle.
Enabling, disabling or changing a notch changes the meaning of the threshold.
Recalibrate each affected axis before using its saved model:

```gcode
ACCEL_HOMING_CALIBRATE AXIS=Y CENTERED=1
```

Follow the [manual preparation](../guide/commands.md#prepare-every-moving-workflow)
before running this moving command. It saves and restarts on success by default;
`SAVE_CONFIG=0` stages only. Reprepare after restart before another moving test.

The saved model identifies the filter settings used for qualification. An
incompatible saved threshold is refused before affected homing; the printer
can still load and perform fresh calibration. Explicit manual thresholds are
interpreted in the configured detector's metric and do not establish calibration.
Remove manual overrides when intending to use a saved model.

Filtered models use model version 2 so older code cannot silently load
their thresholds as unfiltered thresholds. Returning to `main` may require
restoring a compatible unfiltered model/configuration. Preserve the configuration
backup; do not relabel a filtered model as an older version.

The initial stationary-noise measurement remains unfiltered. It selects a
starting candidate, not a bound on filtered peaks: filter transients can exceed
the corresponding raw peak. Discovery and the ten validation homes evaluate
the configured detector. A new filter still needs its own qualified threshold.

`ACCEL_HOMING_BETA_CALIBRATE` is not supported on an axis with a notch enabled
and refuses before movement. Use ordinary `ACCEL_HOMING_CALIBRATE` for that axis.
Beta on an axis with its notch disabled retains the existing behavior.

## Compare the result

After calibration and manual preparation, collect a diagnostic run:

```gcode
ACCEL_HOMING_TROUBLESHOOT AXIS=Y SAMPLES=50 CENTERED=1
```

Keep a complete unfiltered control run and a complete filtered run with otherwise
matched operating settings. Inspect raw and filtered traces, early/missed
triggers, trigger/stop timing, and travel variation. Retain the complete bundle,
including failed homes. Raw traces remain available and are not overwritten by
the filtered signal. Sensor delivery faults remain errors; a notch does not
repair missing samples or delayed packet delivery.

## Detector semantics

In gentle mode the detector selects the mapped component, subtracts its resting
reference, applies the selected causal notch if enabled, and compares the
absolute result to that axis's threshold. Before a map is known, free mapping
strokes read the configured sensor channels to identify the requested axis.
Explicit XYZ diagnostics use their own detector. The filter has bounded
per-home state, retained across packets and reset for
the next home. State starts at zero at the first eligible armed sample; there
is no hidden warmup or interval in which initial impacts are ignored.
Coefficients use the measured resting-reference sensor cadence, not host report
cadence. A rate change exceeding 2% relative to the filter's rate is rejected;
saved filtered models also require a compatible measured rate within 2%.

The first filtered threshold crossing requests the stop. There is no additional
multi-sample confirmation or retrospective contact-time correction. The filter
itself can shift the crossing, and host/MCU delivery adds separate latency.
Sample-integrity, stream-liveness and movement limits still apply to the original
data. With frequency zero, the existing detector path remains unchanged.

The coefficients use the RBJ notch formula with `alpha = sin(omega)/(2*Q)`;
see the [W3C Audio EQ Cookbook](https://www.w3.org/TR/audio-eq-cookbook/).
The armed implementation does not require SciPy.
