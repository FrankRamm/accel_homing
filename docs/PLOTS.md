# PNG plots and stationary noise

All newly generated plots are completed **PNG** images. Successful ACCURACY
also publishes a per-axis bounded JSON diagnostic; other plot workflows do not
gain a JSON sidecar. There are no CSV, SVG or HTML charts or full-capture
raw-sample exports. Only successful workflows publish measurement artifacts.
Rejected, failed, aborted or cancelled runs retain console diagnostics without
saving partial accuracy PNG/JSON outputs.

All acceleration-component curves are displayed as absolute values after their
existing processing: `|X|`, `|Y|`, `|Z|`, and the magnitude of signed v1 output.
This includes home/capture, Accuracy overlays, filter comparisons, Beta and
stationary-noise plots. Component peaks, truncated endpoints and directional
threshold lines use positive display magnitudes. Generic/historical directional
RAW diagnostics retain their labelled polarity; native Beacon live decisions
use sign-independent centered absolute acceleration/XYZ magnitude. Native
impact_sign/automatic direction no longer gates impacts. Amplitude and variation panels start at zero. Travel panels
retain their useful positive zoom, and relative time keeps its sign.

Display folding preserves original signed captures, encoded JSON samples,
saved metadata, Accuracy receipts and baseline statistics. The separate native
live absolute-decision policy is described in [FILTERS.md](FILTERS.md#what-changes).
RMS/P99 and other metrics use the original processed samples; PSD
uses original raw/signed-v1 or existing rectified-v2 outputs, never the absolute
display curves. V2 input rectification and display folding are separate steps.
Display reduction retains signed extrema and per-component minimum absolute
values within the existing point limit, so near-zero troughs remain visible.
Comparison text reports a positive change magnitude with its direction.

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
| `ACCEL_HOMING_ACCURACY AXIS=X SAMPLES=10 CENTERED=1` | Yes | One final overview PNG and bounded per-home JSON per requested axis after every axis succeeds; optional explicit `BASELINE=<run_id>` adds independent actual-run A/B statistics. |
| `ACCEL_HOMING_FILTER_CHARTS AXIS=X CENTERED=1` | Yes | Same original signed capture compared with signed-v1 and rectified-v2 filtering; actual control, diagnostic shadows, thresholds, timing, noise/DC and censoring are labelled separately. |
| `ACCEL_HOMING_BETA_CALIBRATE AXIS=X CENTERED=1` | Yes | Combined **Beta** moving-noise/trial diagnostic and suggestion report. |

The plots show baseline-relative acceleration, the configured detector
threshold and detected onset when available. New vector-model captures label
XYZ magnitude; legacy captures retain their detector context. The measured
onset/peak can differ from the threshold used to request stopping.

Pure FILTERED control uses onset markers at the threshold crossing of
the candidate that actually confirms (`accepted_threshold_crossing_v1`). RAW
control retains legacy lookback markers for its FILTERED diagnostic shadows,
labelled `Estimated filtered lookback onset`. Confirmation and the host
stop-request interval remain distinct and unchanged, and the original lookback
still anchors the existing 15 ms impact-response window. Historical FILTERED
baselines without a policy retain `filtered_lookback_v1`; new pure FILTERED PNG
receipts use version 4 and list policy differences in actual-run comparisons. Existing
images are not rewritten or reinterpreted as new-policy measurements.

Explicit HYBRID ordinary CHARTS/ACCURACY retains FILTERED confirmation and
labels its position policy separately as
`associated_raw_threshold_crossing_v1`. Its position marker uses the associated
RAW XYZ threshold crossing when selected, with an explicit FILTERED-crossing
fallback when selection fails. Accuracy figures identify the requested HYBRID
mode, FILTERED control, both thresholds and measured RAW-selection/fallback
counts and reasons. The paired JSON keeps each home's selected sample/time and
selection evidence. HYBRID's embedded receipt uses version 5; its bounded JSON
retains schema version 1. Read those counts before interpreting aggregate variation;
a HYBRID report can include both position sources. Confirmation, returned
position and the host stop-request interval remain different events, and none
measures physical contact or MCU halt. See [HYBRID timing](FILTERS.md#hybrid-position-timing).

Traces retain bounded summaries and signed extrema instead of every sample.
A diagnostic chart cannot establish that all vector peaks or physical
transients were preserved, nor measure end-to-end stopping latency. Synthetic
fixtures are marked **SIMULATED / NOT PRINTER MEASUREMENT**.

Development accuracy PNGs include a bounded compact successful-run receipt in
PNG text metadata and the run ID in the filename. It survives calibration and
restart so a later explicit BASELINE can compare actual independent series.
The separate [per-home diagnostic JSON](ACCURACY.md#per-home-json-diagnostics)
links to the exact PNG by SHA256; it does not replace or change that embedded
receipt. Existing valid baselines need no JSON companion. Keep the original
host filenames, contents and PNG metadata.
Missing sensor/geometry/speed/acceleration context or a mismatch prevents an
attribution to filtering. A replay trace is never used as a physical baseline.
Read [the comparison procedure and interpretation](FILTERS.md).

The filter figure's noise/PSD use only the validated stationary pre-arm
reference, after filter warmup. No spectrum is drawn from a gapped, undersized
or display-reduced trace. The signed-v1 and rectified-v2 variants share the
same source/reference/weights and numerical comparison threshold; only the
declared actual version controlled that home. Nonactive variants are diagnostic
and do not have an independently calibrated threshold in this capture. Original
signed XYZ remain retained; their plotted components use absolute display values,
as do the rectified per-axis envelopes.
Known native Beacon marker rows remain at their original times but use bounded
configured-endpoint estimates selected by prior sensor sign. Their trace,
marker count and unknown true sign/peak are labelled explicitly; `32767` is
never plotted as an acceleration. Generic numerical values are unchanged.
Mean-removed XYZ noise metrics are separate from decision-level mean/RMS/P99
that retain rectification's DC. Absolute value is nonlinear, so spectral peaks
can move/double; PSD differences do not establish physical vibration removal.
Confirmations at/after the actual stop-request cutoff are censored;
after-cutoff curves are labelled response only. Historical censored captures
still omit their unavailable clipped tail. Original acquisition times determine
eligibility; displayed cutoff markers may be interpolated. Ordinary FILTERED
accuracy/capture plots show sensor input (labelled estimates for marker rows), with the causal weighted
threshold stated separately rather than overlaid on the raw magnitude.

Successful `ACCEL_HOMING_CALIBRATE AXIS=X DETECTOR=FILTERED CENTERED=1` publishes
its fresh stationary-noise summary, not this comparison. After saving/restart,
reprepare and physically recenter before
`ACCEL_HOMING_FILTER_CHARTS AXIS=X DETECTOR=FILTERED CENTERED=1`: it performs
another real home. A compatible RAW XYZ profile or explicitly chosen
`RAW_THRESHOLD` is required for the RAW shadow. Failed/incomplete workflows
still publish no comparison PNG. Historical rectified-v2 accuracy receipts can
use version 3; new pure FILTERED uses 4 and HYBRID uses 5. Historical RAW and signed-filter receipts retain their
original version/configuration and remain usable under the documented checks.

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
Sensor errors, old/future timestamps, invalid data, changed context and
cancellation abort without this recovery. Homing and Beta moving-noise checks
retain their existing strict policy.

Mapped X/Y hints are diagnostic. The numerical candidate is the measured peak
plus the larger of 20% or 0.1 g, rounded upward onto the configured start/step
grid. Unknown physical range or a result above known sensor headroom makes
an automatic hint unavailable. A displayed hint is not a calibrated impact
threshold or a measured minimum.

Each new CALIBRATE acquires its own three-window stationary capture before
movement. Paired HYBRID computes independent RAW and FILTERED starting hints
from that initial capture; pure mode uses its actual detector metric. A previous
standalone NOISE result is not reused. Fresh acquisition is required even when
START_THRESHOLD is explicit. Its summary is rendered only after the complete
calibration qualifies, including both five-check series for paired HYBRID, and
published after required cleanup and saving succeed.
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
keeps the valid saved outputs; it does not convert that measurement into a failure.

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
