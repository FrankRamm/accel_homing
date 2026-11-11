# Experimental notch filter (dev)

The optional notch attenuates a narrow frequency band during accelerometer
homing. It is disabled by default. There is no automatic frequency selection:
choose the frequency from moving-noise and impact captures for the affected
machine, sensor, speed and acceleration.

A notch can also attenuate an impact, produce ringing or change the threshold
crossing time. Its usefulness must be established by comparing complete runs;
a smoother acceleration trace does not establish better physical homing.

## Configuration

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
the same notch to all three signed accelerometer components before combining
them. For example, a Y home can filter vibration measured predominantly by the
sensor's X channel without discarding that channel's impact information.

## Calibrate after changing the filter

After updating the plugin, follow the normal service-update instructions.
Later configuration-only filter changes need a Klipper `RESTART` while idle.
Enabling, disabling or changing a notch changes the meaning of the threshold.
Recalibrate each affected axis before using its saved model:

```gcode
ACCEL_HOMING_CALIBRATE AXIS=Y CENTERED=1
```

Follow the [manual preparation](COMMANDS.md#prepare-every-moving-workflow)
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

Processing is: subtract the fixed resting XYZ reference, apply three identical
causal second-order notch filters, then compare the XYZ magnitude to the active
threshold. The filter operates on signed components, before absolute value or
magnitude. It has bounded per-home state, retained across packets and reset for
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
