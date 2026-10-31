# PNG plots and stationary noise

All newly generated plots are completed **PNG** images. There are no CSV,
JSON, SVG or HTML chart sidecars and no full-resolution raw-sample export.
Only successful workflows publish plots. Rejected, failed, aborted or cancelled
measurements retain console diagnostics without saving a partial PNG anywhere.

## Where files are saved

By default, `accel_homing_charts/` is beside the main `printer.cfg`. Set
`charts_directory` in the existing plugin section to choose a different writable
directory. The output filesystem needs hard-link support for atomic publication.

Console output and `ACCEL_HOMING_STATUS` report saved paths. Files use the host's
local timestamp in both title and filename, for example:

```text
Accel Homing Accuracy X - 2026-10-27 23:59:42.png
```

PNG metadata retains UTC provenance and the local timezone offset.
Rename to a colon-free filename when downloading to Windows. Existing images
are retained; the plugin does not rotate or delete old final chart outputs.

## Which command produces which plot

| Command | Motion | Output |
| --- | --- | --- |
| `ACCEL_HOMING_CALIBRATE_NOISE` | None | Three-window stationary-noise overview. |
| `ACCEL_HOMING_CALIBRATE AXIS=X CENTERED=1` | Yes | Fresh stationary-noise summary after calibration, cleanup and required save succeed; its hints exclude the final 0.5 g saving margin. |
| `ACCEL_HOMING_CHARTS_GENERATE AXIS=X CENTERED=1` | Yes | Measured acceleration capture for that home; omit AXIS for X then Y. |
| `ACCEL_HOMING_ACCURACY AXIS=X SAMPLES=10 CENTERED=1` | Yes | One final accuracy overview per requested axis after every axis succeeds. |
| `ACCEL_HOMING_BETA_CALIBRATE AXIS=X CENTERED=1` | Yes | Combined **Beta** moving-noise/trial diagnostic and suggestion report. |

Acceleration curves show `abs(ax-bx)`, `abs(ay-by)` and `abs(az-bz)` in g.
The fresh signed resting reference is subtracted **before** taking absolute
values. Acceleration axes, threshold lines and peak markers are nonnegative.
New vector-model captures also label XYZ magnitude; selected-axis captures
retain their detector context. The measured onset/peak can differ from the
threshold used to request stopping. Signed source samples, reference offsets,
travel statistics and times are retained internally. Relative event times can
be negative; ACCURACY retains its precise travel scale in mm.

Traces retain bounded summaries, signed extrema and absolute-value troughs
instead of every sample. New capture envelopes have at most 4096 points;
legacy published-capture receipts retain at most 512. Display reduction does
not change the signed samples used for detection or statistics.
A diagnostic chart cannot establish that all vector peaks or physical
transients were preserved, nor measure end-to-end stopping latency. Synthetic
fixtures are marked **SIMULATED / NOT PRINTER MEASUREMENT**.

Native Beacon clipping markers are decoded as bounded sensor-endpoint
estimates using prior sensor sign. Their true sign and amplitude are unknown;
an estimated plotted peak is not a recovered impact peak. Finite sensor-rail
values are reported separately, with unknown excess amplitude. Recognized
markers or rails do not by themselves invalidate otherwise valid RAW data.
Threshold headroom, acquisition and other data-fault checks still apply.

Range notes aggregate every measured ACCURACY home (excluding the qualifier),
all three noise windows, and every Beta motion/home in their respective
groups. Counts are axis values, not a count of samples. Capture/window/motion
provenance and resting-reference provenance are disclosed separately; their
counts can overlap and are never added together. A limited reference can
make the baseline uncertain even when the plotted capture has no markers.
Older receipts without range metadata remain usable and are labelled as
having unavailable provenance. Mixed native decode conventions are reported.

## Stationary noise

Run without moving or disturbing the printer:

```gcode
ACCEL_HOMING_CALIBRATE_NOISE
```

It acquires three disjoint one-second windows with the current motor/fan state
and an exclusive sensor stream. It does not enable/release motors, home,
change thresholds, save configuration or restart Klipper.

A transient sample gap, fresh-data wait timeout or acquisition timeout discards
the entire affected window and starts a fresh one-second acquisition. A window permits at most two such retries,
with at most three shared across all three windows. Console diagnostics identify
retries and exhaustion. Partial windows are never joined or used for statistics.
Sensor errors, future/invalid timestamps, changed context and cancellation
abort without this recovery. Generic current-window rows are accepted despite
old sensor timestamps. Only validated relevant receipts renew the independent
stream watchdog. Native Beacon retains its age policy. Homing/Beta do not opt
into stationary-window retries.

Mapped X/Y hints are diagnostic. The numerical candidate is the measured peak
plus the larger of 20% or 0.1 g, rounded upward onto the configured start/step
grid. Unknown physical range or a result above known sensor headroom makes
an automatic hint unavailable. A displayed hint is not a calibrated impact
threshold or a measured minimum.

Each new CALIBRATE measures its own fresh **XYZ-magnitude** noise before movement.
A previous standalone NOISE result is not reused. Fresh acquisition is required
even when START_THRESHOLD is explicit. Its summary is rendered only after the
calibration qualifies and published after required cleanup and saving succeed.
Stationary measurement does not cover motor enabling, first-step
transients, acceleration or moving vibration. See [calibration](SIMPLE_CALIBRATION.md).

## Opening plots in Mainsail or Fluidd

A verified same-origin Moonraker file route can provide a console image link
or inline preview. Mainsail needs formatted console output; raw-output mode
shows text. An ordinary browser image GET must already be authorized, for
example through a trusted LAN or a suitable session cookie. Header-token-only
authentication or a remote frontend may require opening the PNG through the
frontend file browser. No tokens are embedded in plot links.

Automatic route discovery requires one same-user Moonraker process matching
the selected Klippy socket. Unresolved includes/templates, relative socket
paths or ambiguous instances can prevent a verified inline URL. The saved
path remains usable; lack of an inline preview alone does not invalidate an
otherwise completed measurement.

Successful measurement/publication is committed before optional console preview
announcement. A later preview/link error is reported as `display_error` and
keeps the valid saved PNG; it does not convert that measurement into a failure.

## Plot dependencies and errors

Installation uses the selected Klipper interpreter to test a real render.
A working existing stack is reused; otherwise binary plotting wheels are
prepared in the ignored checkout-local `.plot-deps/` cache. Matplotlib/Agg
renders in a separate process so Klippy can keep servicing its reactor.
Global Python pins are unchanged.

A small real render is checked before CHARTS/ACCURACY motion or stationary
noise acquisition. Missing dependencies or an unusable output directory
produce an explicit failure. A calibration summary-rendering failure prevents
model staging/saving. Accepted per-home traces remain bounded in-memory data;
they are not saved as intermediate PNGs. Every required image is rendered before
the first output of a multi-axis report is published. If a later publication
fails, cleanup attempts to remove only new files owned by that run; previously
saved user files are preserved.

Rerun the installer while idle if a Python environment change or Hard Recovery
invalidated the cache. Use the exact `--klipper-python` for the selected
instance when automatic discovery cannot establish it; do not install packages
into a guessed global Python. See [installation](INSTALLATION.md).
