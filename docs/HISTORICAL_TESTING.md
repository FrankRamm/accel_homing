# Historical verification notes

This archive preserves the pre-standalone testing narrative and its original
dates, counts and limitations. Statements such as "current" below refer to
their original revisions. Old directory names and bootstrap commands are
historical examples, not installation or test instructions for this repository.
Do not use their passing counts to validate the standalone revision.

Use [current testing instructions](../TESTING.md), [current calibration](SIMPLE_CALIBRATION.md)
and [current installation](INSTALLATION.md) for this checkout.
The immutable older [report](../validation/VALIDATION.json) and
[log](../validation/tests.txt) remain unchanged.

---

# Offline verification

## Simple early-impact return — 2026-10-28

Validation for this change is limited to source inspection and static text/diff
checks. Regression sources were updated but **not executed or imported**;
no plugin import, compilation, rendering, printer connection or printer command
was performed. Historical passing counts below do not verify this revision.
Earlier sections and the archived `validation/` reports describe their original
revisions. Their recovery-distance/clearance policies, reserved-distance
metadata, threshold ceilings and recovery-count limits are historical.

Regression sources: `tests/test_calibration_auto_return.py`,
`tests/test_calibration_midtravel_recovery.py`, `tests/test_calibration_limits.py`
and the new `tests/test_calibration_simple_return.py`.
The intended cases cover premature impacts at <=20 mm and below the lower
expected-distance band, including the reported 51.175 mm travel against an
expected 109.25 mm and lower bound 87.4 mm. Both discovery and validation use
the same classification in both axes and homing directions. Exactly the lower
and upper band boundaries remain plausible when beyond 20 mm; above the upper
bound aborts without return, threshold increase or model write.

Recovery reverses the selected axis's recorded signed start-to-halt commanded
displacement, including post-trigger travel. For 2 mm commanded before the halt,
the return command is 2 mm in the opposite direction. There is no accumulated
recovery distance or separate recovery-count/distance cap. The return has no
overshoot/nominal-clearance gate or extra test of physical arrival. The current
home record and finite displacement/coordinates
are still required to calculate the command. Ordinary firmware movement limits
and errors apply, along with existing sensor/fault/cancellation checks.

After the queued move and settling complete, record the return, increase the
threshold by STEP and retry discovery. A failed/interrupted return does not
advance the threshold or completed-return count. Zero displacement needs no
reverse move but still settles and retries. A new discovery and ten fresh
validation impacts are still required; >2 mm running spread still rejects
immediately and exactly 2 mm still passes after all ten checks.

The new `recovery` object in active status, final results and saved metadata has
only `count`, `last_return_mm_signed`, `accounting: completed_commanded_returns`
and `physical_position_verified: false`. Count includes only completed commanded
returns; the last signed return is `null` until one completes. There are no
accumulated-distance, limit/remaining, clearance or margin fields. Old saved
recovery metadata remains readable as opaque historical data and does not
restrict a new run.

The existing monitored-home budget, 40 attempts by default, includes discovery,
validation and retries. Every candidate change requires at least 11 future
attempts for a fresh discovery and ten checks before any recovery return,
threshold increase or recentering. Active status and
final results expose `attempt_limit`, `attempts_remaining` and
`candidate_attempts_required` (11). Denial makes no return or threshold increase
and invalidates only the suspect axis's homed state. More than eight recoveries
may proceed within that attempt budget; there is no separate count cap.

The configurable calibration maximum and fixed 15 g calibration/model maximum
are removed from initial parameters, noise selection, candidate increases and
model save/load. `calibration_max_threshold` remains accepted but ignored with a
deprecation warning. Calibration/model thresholds retain their finite 0.5 g
minimum and STEP retains its finite 0.1–3 g range. The existing 80%-of-known-range
headroom policy remains, including active-stream rechecks; it is a conservative
software policy, separate from physical saturation and sensor-fault checks.
Unknown range still prevents an automatic noise seed, while an otherwise valid
explicit start may be used. No candidate is clamped downward. Thresholds are
therefore still bounded when the sensor range is known.
Before arming, the derived raw trigger limit and its squared value for the
XYZ-magnitude detector must remain finite and positive; this is numeric
representation validation, separate from a configurable g maximum.
Huge/tiny finite inputs that make that square overflow to infinity or underflow
to zero are rejected. The guard and its regression sources were not executed.

Faults, a missing current home record, missing impacts and firmware movement
errors still abort without a threshold increase. These source cases do not
establish physical return, correct endstop contact or hardware performance.
There is no encoder feedback or automatic binding diagnosis. See the [current
sequence, exact limits and metadata](SIMPLE_CALIBRATION.md).

## Running spread, travel plausibility and shared returns — 2026-10-28

This change was reviewed from source only. Regression sources were updated but
**not executed or imported**; no plugin import, compilation, rendering, printer
connection or printer command was performed. `git diff --check` is the only
executed check for this revision. Historical test counts below do not verify it.

Regression sources: `tests/test_calibration_validation.py`,
`tests/test_simple_calibration.py` and `tests/test_workflows.py`.
They cover rejecting a running max-minus-min spread greater
than 2 mm as soon as it appears, without waiting for ten validation impacts;
exactly 2 mm remains accepted, but success still needs ten passing checks after
a separate discovery. A retry clears the entire window and requires a fresh
discovery, within the existing attempt and threshold ceilings.

They also cover inclusive midpoint-to-endstop travel plausibility at the
existing `calibration_travel_tolerance` (default 20%), for discovery and every
validation impact beyond 20 mm, on both axes and homing directions. An
implausible >20 mm hit aborts without reverse/midpoint return, threshold
increase or model write, invalidates only the suspect axis's homed state and
requires manual XY centering before a new command. Bounded automatic reversal
and retry for impacts at or below 20 mm are retained. These are commanded-step
plausibility checks, not contact or physical-position proof.

CALIBRATE and ACCURACY midpoint return sources use ordinary acceleration and
the same explicit `calibration_settle_time` wait. Bounded early reversals keep
their existing cap and wait; homing speed, its acceleration cap and
TEST/CHARTS returns remain unchanged. Fresh three-window stationary-noise
preflight and summary export remain mandatory before calibration movement.
Earlier descriptions of the 1.5 mm rule, waiting for complete windows and capped
midpoint returns describe historical revisions, not current calibration.
Current behavior is documented in [SIMPLE_CALIBRATION.md](SIMPLE_CALIBRATION.md).

## Stationary-noise preflight before calibration — 2026-10-28

This change was reviewed from source only. Regression sources were updated but
**not executed or imported**; no plugin import, compilation, rendering, printer
connection or printer command was performed. Historical test counts below do
not verify this revision.

Regression sources: `tests/test_calibration_noise_preflight.py`,
`tests/test_retract_noise.py`, `tests/test_accuracy_charts.py` and the shared
synthetic fixture in `tests/test_workflows.py`.
The intended cases cover three fresh stationary windows and completed PNG
export before each new per-axis calibration's first movement; no remeasurement
on candidate retries; and no reuse of an older standalone diagnostic result.
Automatic initial selection uses mean-removed per-sample XYZ magnitude,
including multi-channel noise, rather than mapped X/Y component hints.
An explicit `START_THRESHOLD` retains its value while still requiring the
measurement; an explicit `STEP` supplies the selection grid spacing.
Unknown sensor range and over-ceiling automatic candidates abort without
clamping or fallback. Acquisition/context/export faults, cancellation and
shutdown must abort before movement and must not publish a successful seed.
Status/model metadata distinguish the measurement from an automatic seed and
preserve the current measurement's identity and magnitude provenance.

The >20 mm discovery rule, ten subsequent validation homes, <=2 mm complete-set
spread and existing attempt/threshold limits are unchanged. Stationary noise
does not qualify motor-start or travel transients. These are synthetic source
cases and an initial-value heuristic, not executed verification, a certified
minimum threshold or hardware performance evidence.

## Automatic early-trigger calibration return — 2026-10-28

This change was reviewed from source only. Regression sources were updated but
**not executed or imported**; no plugin import, compilation, rendering, printer
connection or printer command was performed. Historical test counts below do
not verify this revision.

Regression sources: `tests/test_calibration_auto_return.py`,
`tests/test_simple_calibration.py` and `tests/test_workflows.py`.
The intended cases cover early impacts in discovery and validation,
both axes and homing directions, signed start-to-halt reversal including
post-trigger travel, settling before retry, selected-axis-only return, fresh
discovery and ten-check windows, and the existing attempt/threshold ceilings.
Return checks require current, complete evidence and conservative per-return
travel bounds. Faults, uncertain evidence, cancellation, shutdown and motor
release must abort without automatic recovery. The >20 mm discovery rule and
the <=2 mm spread across ten validation impacts are unchanged. These are
commanded-step checks, not physical-position or hardware validation.

## Generic 50 ms sample-age default — 2026-10-28

This change was reviewed from source only. Regression sources were updated but
**not executed or imported**; no plugin import, compilation, rendering, printer
connection or printer command was performed. Historical test counts below do
not verify this revision.

The sources cover the 50 ms generic default during stationary preparation and
armed homing, rejection beyond that limit, preserved explicit sample-age and
shorter no-data timeouts, and matching installer policy. Earlier numeric policy
fixtures explicitly select 20 ms where needed to retain their original cases.
The report/startup distance policies and future-timestamp, gap and disconnect
checks remain enabled. With sample age omitted, the existing legacy startup
formula gains 30 ms and `speed * 0.030` mm; a 1 mm reporting limit now permits
at most 20 mm/s. No distance budget is increased to admit a faster plan.

## Temporary accuracy captures and colon filenames — 2026-10-28

This change was reviewed from source only. Regression sources were updated but
**not executed or imported**; no plugin import, compilation, rendering, printer
connection or printer command was performed. Historical test counts below do
not verify this revision.

The regression sources cover per-run temporary captures outside the watched
chart/config roots, including a redirected temporary root; cleanup after final
export, abort/cancel and setup/export failure; original motion-error priority;
separate capture-source receipt/hash checks; public metadata without removed
paths; prior-chart preservation; and already-published X paths when Y export
fails. The maximum XY source fixture uses 102 temporary per-home PNGs, then keeps
only two bounded final summaries with 100 accepted traces. Filename sources
expect local `HH:MM:SS`, including collision suffixes and encoded inline URLs.
Filesystem chart fixtures target the Linux Klipper host because Windows filenames
do not support colons.

## Accuracy statistics and export errors — 2026-10-28

This correction has been reviewed from source only. Its added regression
tests have **not been executed**; no plugin or test module was imported,
and no printer or printer service was accessed. The 708-test result in the
next section belongs to the preceding revision and does not verify this change.

The new regressions cover shared statistics from unrounded commanded travel,
fallback for older records, matching chart travel, rejection of malformed exact
measurements or altered statistics, controlled validation/export command errors,
and propagation of motion and restoration failures. Calibration's existing
repeatability calculation and acceptance rules are unchanged.

## Impact-after-20 mm discovery and selected-axis centering

The updated package passed **708 offline tests** with no failures, errors or
skips using `python -B -m unittest discover -s Beacon_Accel_Homing/tests -v`.
`git diff --check` passed. No printer hardware or service was accessed.

This revision corrects the preceding screening design: crossing 20 mm is
never success. Both discovery and validation require a threshold-crossing
impact with commanded travel strictly greater than 20 mm. The initial hit
is excluded; only the selected X/Y axis returns to its configured midpoint,
then ten following impacts define the max-minus-min spread. Exactly 2 mm
passes; a larger completed set raises the threshold by STEP and requires a
new discovery impact and ten new measurements.

Regressions cover both movement axes and both homing directions; full G28
rather than a bounded no-contact screen; a live sensor stream beyond 20 mm
without a trigger; no-impact rejection; the exact 20 mm boundary using unrounded
travel; exclusion of an off-center discovery; unchanged other-axis/Z return
coordinates; failed-set restart requiring another impact; unchanged command
syntax and resume grid; rejecting obsolete screen-only records at save time;
and existing vector detection, sensor faults, ceilings and cleanup. Stock
G28's fatal all-motor-off handling is preserved, including on discovery errors.

Historical sections below describe their own revisions, not current behavior.

## Simple vector calibration with 20 mm screening — 2026-10-27

The integrated package passed **704 offline tests**, including **29 new**
vector/screening/return/model/chart regressions:

```bash
python -B -m unittest discover -s Beacon_Accel_Homing/tests -v
```

The local run completed in 16.091 seconds with no failures, errors or skips.
`git diff --check` passed. No printer or printer service was accessed. All
measurement fixtures are synthetic; these results do not certify mechanical
endstop contact, physical travel, repeatability or stopping latency.

The current workflow is documented in [SIMPLE_CALIBRATION.md](SIMPLE_CALIBRATION.md).
It replaces calibration discovery collisions with a bounded 20 mm screening
move, keeps all ten validation impacts separate, accepts an exactly 2 mm spread,
and retries a failed complete set at the next candidate with a new screen and
an empty measurement window. Early hits still stop and request manual recentering.
The command syntax, optional start/step overrides, save behavior, selected X/Y
motion, attempt ceiling, threshold ceiling and data fault cleanup are exercised.

Detector tests exercise the real prepared generic backend, noisy offset
references, all sensor channels/signs, diagonal vectors, exact threshold
crossing, original onset sample identity, separate post-trigger peaks, legacy
startup-deadline bypass, and retained sensor-data/travel watchdogs. Native raw
vector and sample-gap cases are included. Legacy native/generic estimator tests
remain for old models and manual thresholds; those paths are not silently
converted to vector detection.

The screening adapter is exercised with a fake HomingMove implementing the
verified `probe_pos=True, check_triggered=False` contract: both axes and both
travel directions, endpoint step quantization, early impacts, no stock G28
screen dispatch, missing API and missing halted-distance failures. This is not
a full firmware/MCU replay. Return tests reverse signed halted step travel,
including overshoot, rather than a falsely assumed physical endstop coordinate.
Cancellation/cleanup and Z-state preservation are covered by the workflow suite.

Saved vector models are tested through ordinary-home preparation to retain
the same detection metric and omit quietness/motion-proof checks after reload.
Charts show the XYZ magnitude and its positive threshold; old scalar plots
remain unchanged. Spread calculations retain step precision rather than the
console's three-decimal rounding, with only binary representation noise removed.
Stationary-noise hints are diagnostic and no longer seed this calibration.

Earlier sections below describe the historical policies of their revisions.

## Generic startup evidence and calibration return settling — 2026-10-27

The package passed **675 tests** with:

```bash
python -B -m unittest discover -s Beacon_Accel_Homing/tests -v
```

The integrated run completed in 9.603 seconds with no failures, errors or
skips. `git diff --check` also passed. No printer or printer service was
accessed.

The generic motion regression replays the same timestamped ramp under two
report phases. Previously one phase lost its conservative early speed evidence
before the commanded-speed comparison became eligible. Both now prove motion
using a retained lower-bound peak, subject to the existing command eligibility
and half-full-speed requirement. Samples dated after the original startup
deadline cannot create that historical peak. Stationary noise, bounded bias,
insufficient-margin ramps, and a peak with no eligible command still fail.
Native motion, first-sample contact and calibration data-fault tests also pass.

Calibration return tests cover both selected axes, the existing acceleration
cap without increasing a lower live setting, the default 0.25-second dwell,
zero dwell, zero cap, configuration bounds, status and saved metadata. Errors
and cancellation at each yielding stage restore acceleration and prevent the
next trial. A paired synthetic decaying-vibration replay fails immediately
after the return and proves motion after the dwell; persistent vibration still
fails with or without the dwell.

New diagnostic tests retain coherent raw/lower/upper speed measurements,
immutable mapped noise bounds, command eligibility, peak timing and stop-request
timing through cleanup and status reporting. Generic diagnostics do not imply
the presence of native first-step history.

These checks establish the reproduced software defect and the behavior of the
correction. They do not establish the exact cause of the reported printer
failure or physical homing success. The settling interval can reduce transient
return vibration, but persistent noise can still prevent movement from being
verified. Calibration remains at a 1.0 g default start and 0.25 g default step.

## Calibration references and selected threshold grids — 2026-10-27

The package passed **665 tests** with:

```bash
python -B -m unittest discover -s Beacon_Accel_Homing/tests -v
```

The integrated run completed in 9.653 seconds with no failures, errors or
skips. `git diff --check` also passed. No printer or printer service was
accessed. Earlier sections describe their historical revisions and defaults.

Calibration now skips only the peak-to-peak quietness gate during an active
trial for the selected axis. Synthetic generic traces with XYZ ranges of
1.313/0.604/0.281 g retain a complete 100 ms reference, its actual mean and
maximum-deviation envelope. The envelope remains active at higher candidates;
stationary noise or bounded bias cannot become proof of movement merely by
raising the impact threshold. Native and generic first-sample contact tests
retain raw threshold detection without a blanking interval.

New regressions cover full duration/count and capacity limits; missing, stale,
future, duplicated, gapped, invalid, clipped, saturated and scale-changed data;
other workflow and ordinary-homing quietness; and rejection of an expired or
out-of-scope calibration reference before arming. Status distinguishes a fresh
calibration reference from a quiet-qualified reference.

Workflow defaults are 1.0 g start and 0.25 g step. Selected-axis fixtures check
1.0 → 1.25 → 1.5 g progression, unchanged configuration, explicit command-grid
precedence over noise seeds, immutable resume parameters, value/ceiling checks
before effects, and cleanup on failure. The existing 40-attempt limit,
threshold ceilings and ten-home qualification are unchanged. These tests
verify software behavior; they do not establish physical homing success on
the user's printer.

## Rolling quiet references and warmup continuity — 2026-10-27

The package passed **647 tests** with:

```bash
python -B -m unittest discover -s Beacon_Accel_Homing/tests -v
```

The final run completed in 10.732 seconds with no failures, errors or skips.
`git diff --check` also passed. No printer or printer service was accessed.

Ten rolling-reference tests include a reproduced overlapping-window failure,
2,550 per-sample comparisons with an independent suffix-search oracle, tied
extrema, retained boundary samples, count/duration limits, mean and noise-bound
membership, capacity failure, and stale/new-noise invalidation before arming.
Both supported generic batch interfaces retain first-sample raw impact
detection. A 65,536-sample capacity overflow is latched and reported before
trigger dispatch; it never silently shortens the required reference.

Five new warmup tests reproduce continuous 3200 Hz data falsely reported as
gapped after its stale prefix was intentionally discarded. They cover both
helper interfaces, obsolete pending singleton data, entirely old batches,
fresh-history and retained-row gaps, malformed rows, driver errors, and
previously latched faults. Actual measured windows and armed moves retain
their original gap and freshness checks.

Existing baseline fixtures now evaluate the exact retained window's mean and
maximum deviations, and use the existing 1 ns duration tolerance at a trimmed
boundary. The default 0.7 g floor, 100 ms duration, impact thresholds and
motion-proof policy are unchanged. These regressions establish software
defects and their correction, not the precise cause of every printer log or
successful physical homing under arbitrary vibration.

## Generic quiet-baseline floor of 0.7 g — 2026-10-27

At commit `d58a18c`, the package passed **632 tests** with:

```bash
python -m unittest discover -s Beacon_Accel_Homing/tests -v
```

The run completed in 7.494 seconds with no failures, errors or skips.
`git diff --check` also passed. Five new regression methods, with
parameterized subcases, exercise the requested wider generic baseline floor.
Constructed XYZ ranges of 0.581/0.392/0.258 g qualify for the full 100 ms at
0.5 g and 1 g impact thresholds. Above-floor variation still prevents arming,
stationary bias cannot prove motion, raw impacts retain their thresholds,
and a ramp with sufficient signal margin can prove movement. Existing
explicit lower floors and native Beacon behavior remain covered.

These are synthetic software checks. They do not replay a raw printer
recording or establish that every admitted noise trace can prove movement.
The existing motion checks can still abort with `no_motion` after the wider
baseline qualifies. No printer or printer service was accessed.

## Noise-window timing correction — 2026-10-27

At commit `62295b1`, the package passed **627 tests** with CPython 3.12.14 on Linux:

```bash
python -m unittest discover -s Beacon_Accel_Homing/tests -v
```

The final run completed in 7.209 seconds with no failures, errors or skips.
`git diff --check` also passed. All sensor, motion, service and installation
operations in this suite use synthetic objects, mocked services or disposable
local files; no printer or printer service was accessed.

Eight new decoded-backend regressions exercise the reproduced boundary case:
a sample preceding the new noise window ages to 20.5 ms during a callback,
while every relevant sample remains younger than 11 ms. Both modern and
legacy helpers now exclude the old row before freshness/continuity checks.
Coverage also includes exact-cutoff/all-old batches, pending singleton rows,
between-window draining and continued rejection of genuine current-window
stale/future/duplicate/gapped data and malformed/error/overflow reports.

Four new noise diagnostics tests exercise native and decoded second-window
failures, signed ages and stale/future classification, first-fault retention,
cleanup and next-stream reset, preservation of completed captures and prior
homing diagnostics, the actual shorter noise-wait timeout, and refusal to
attach a previous run's fault to a pre-stream failure. Existing baseline and
status-copy tests also verify the retained trial threshold after restoring
ordinary thresholds and independent copies of timing evidence.

The integrated run exposed an unrelated existing updater fixture with a
malformed simulated `ExecStart`. Its literal command now includes systemctl's
`path=... ; argv[]=...` structure. The production updater is unchanged.

That revision left quiet-baseline limits, impact thresholds, calibration-candidate
policy, homing freshness limits and motor behavior unchanged. The suite verifies the
software boundary correction and diagnostics; it does not establish that a
particular printer's physical noise or delivery timing is acceptable.

## Earlier verification history

The following notes record validation at earlier revisions. The historical
`validation/VALIDATION.json` and `validation/tests.txt` are unchanged and do not
describe the newer runs above.

**Earlier installation fixes: source review only at the time.** The separation of automatic
link ownership validation from uninstall restoration, live Moonraker path-default
discovery, and their added regression source were reviewed without executing
tests, installers or printer actions.
The **558-test executed run recorded below predates these fixes**. Its log and
source digests remain historical evidence; they do not certify the changed
installation implementation. No current test pass is claimed.

Path discovery follows the captured installed Moonraker argument parser:
command-line paths override `MOONRAKER_DATA_PATH`/`MOONRAKER_CONFIG_PATH`, with
the data default expanded using the verified process home. Only those path
keys and `HOME` are retained from bounded same-user process environment reads.
Four unexecuted regression methods cover path precedence/defaults, bounded
environment filtering, socket/instance identity and loaded-service path reuse.
The changed modules were not imported, compiled or run for this source review.

Standard `[homing_override]` installation/workflow support and its added or
adjusted regression source were reviewed without execution. Installer fixtures
cover preserving the complete section/script without acknowledgement, refusing
duplicate/conflicting built-in sections and retaining custom G28 safeguards.
Workflow fixtures cover the verified built-in-to-stock path and its refusal
guards. No tests, imports, compilation, plugin execution or printer actions
were performed for this revision. The historical 558-test run does not validate it.

The subsequent zero-match discovery correction is also source-reviewed only.
Its unexecuted regression source covers Moonraker's omitted `/tmp/klippy_uds`
default, explicit empty/relative socket paths, literal `{data_path}` expansion,
working-directory identity, and bounded rejection diagnostics that omit raw
environment/configuration and exception payloads. No affected-host cause or
universal installation compatibility is established by this source review.

The later runtime PNG-root discovery correction is also source-reviewed only.
Six added discovery fixture methods cover environment-only module launches,
argv/environment/home/CWD precedence, passwd fallback, the official omitted
socket default, bounded allowlisted environment reads, UID/entrypoint guards
and ambiguous matching PIDs. The older discovery fixture now states its socket
explicitly instead of depending on the former incorrect `data/comms` fallback.
No tests, imports, compilation, rendering or printer actions were performed for
this correction. A saved-path preview advisory is separate from an accuracy
capture-validation failure; no affected host's current route was verified.

The companion impact-capture correction is source-reviewed only. Its regression
source covers exact original packet/index proof, a unique retained onset row,
separate original-trigger timing, and refusal of missing/altered/ambiguous proof.
The copied report uses the actual retained row time without a tolerance or
synthetic sample. No workflow/unit tests, imports, compilation, rendering or
printer actions were executed for this revision; the historical suite below
does not validate these changes or establish physical accuracy.

Optional CHARTS `AXIS=X`/`AXIS=Y` selection was reviewed as source. Its five new
workflow regression methods were not imported, compiled or executed. The
fixtures cover early invalid-axis rejection, selected-axis capture/final release,
single-axis installations, unchanged manual XY preparation and failure cleanup.
Omitting `AXIS` retains the existing X-then-Y path. The historical executed
counts below do not validate this change. The current PNG layout, local display
timestamps and descriptive filenames also remain outside that executed run.

Separately authorized PC runs executed the isolated chart renderer with
simulated data. Final capture, accuracy and three-window noise PNGs were
visually checked. These are layout previews: workflow/unit regressions remained
unexecuted, no printer actions were performed and no physical accuracy is
established.

All recorded verification uses synthetic Klipper objects, fake sensor
batches, temporary config files and mocked status/download responses. No
printer connection, SSH session, deployment, service restart, physical home,
calibration or hardware measurement was performed.

Run from the repository root with Python 3:

```bash
python -m unittest discover -s Beacon_Accel_Homing/tests -v
```

For shell-entry-point changes, also run:

```bash
bash -n Beacon_Accel_Homing/install.sh
bash -n Beacon_Accel_Homing/bootstrap.sh
bash -n Beacon_Accel_Homing/uninstall.sh
```

Focused installer/bootstrap/uninstall and workflow/homing-path checks:

```bash
python -m unittest discover -s Beacon_Accel_Homing/tests -p test_installer.py -v
python -m unittest discover -s Beacon_Accel_Homing/tests -p test_workflows.py -v
python -m unittest discover -s Beacon_Accel_Homing/tests -p test_accuracy_workflow.py -v
python -m unittest discover -s Beacon_Accel_Homing/tests -p test_accuracy_charts.py -v
python -m unittest discover -s Beacon_Accel_Homing/tests -p test_bootstrap_short.py -v
```

`test_bootstrap_short` uses fake shell `curl` and `python3` functions. Its nine
offline fixtures verify that a failed download cannot start Python even after
printing a full valid script, successful truncated executable prefixes fail
the final-marker guard, complete string/file launches preserve optional flags,
and unsupported direct piping is refused. A comment/function-definitions-only
prefix is a no-op and never invokes installation. No real network, installer,
printer API, package manager or service operation is executed by those fixtures.
The original pinned Git-blob/full-payload/cache tests remain in `BootstrapFixture`.

Native-updater fixtures cover source anchoring/publication, isolated ownership,
Moonraker include conflicts and service identity, activation order/retries,
legacy-mode boundaries and restoration advice. One fixture uses real Git only
inside disposable local repositories to verify sparse tracked publication,
private-cache retention and rollback. It uses no public network or user repo;
all Moonraker/printer/service/pip behavior remains synthetic or mocked.

The chart module's focused command includes the stationary-noise fixtures.
Those fixtures check three separate one-second windows, independent XYZ means,
signed raw-minus-mean traces and exact absolute peak/RMS/peak-to-peak statistics.
A 2 g X peak produces a distinct 2.5 g initial
candidate on the default grid; Y is independent and Z gravity stays in raw
data while its window mean is removed from the plotted trace. Incomplete or
faulted data cannot recommend a seed. Over-ceiling and unknown-range cases
retain the complete three-panel PNG with unavailable candidates. Same-run
identity, chronological non-overlap, sensor/map/scale consistency, immutable
receipts, PNG path/hash/CRC, bounded extrema-preserving traces and collision-safe
publication are checked. Chart directories contain PNG files only; there are
no CSV/JSON/SVG/HTML sidecars or full-resolution raw-data exports.

Delayed-watcher fixtures verify that only the durable chart root becomes a
watched directory; completed PNGs are published directly beneath it. Nine
regressions cover delayed directory-watch registration, a racing filename,
cancellation before/after publication, unsupported hard links, bounded
collision exhaustion and the final filename's create/close-write events.
Completion errors remove only owned files and preserve foreign replacements.
Temporary staging is a same-directory regular file removed on success/failure.
No live Moonraker watcher was exercised.

The representative three-window chart retained outside this source package is
**SIMULATED / NOT PRINTER MEASUREMENT**. Matplotlib/Agg rendered it in an
isolated worker. The three signed XYZ panels share a vertical scale; the
2/1/0.5 g peaks and separate 2.5/1.5 g starting candidates stay distinct.
Additional capture/accuracy PNGs check threshold/onset/peak and travel layout.
Those historical PNG filenames/titles share a timezone-aware UTC instant.
This is layout verification, not a measurement of a physical sensor or printer.

The chart fixtures also run a real tiny worker render and a bounded worker
timeout with temporary-file cleanup. Matplotlib/NumPy/Pillow do not appear in
the parent process's module set. Other synthetic workflow fixtures inject a
small valid-PNG renderer; they verify orchestration/provenance rather than
repeating the real plotting dependency startup for every fake home. The
historical maximum XY accuracy fixture retained 102 per-home PNGs and two bounded overviews,
with 100 accepted traces and population statistics. Validation yields between
PNG files and can cancel before publication. PNG checks reject bad CRCs,
duplicate headers, truncation/trailing bytes and oversized dimensions.

`test_plotting_setup` covers selected lexical venv/ABI identification, existing
renderer reuse, guarded private binary-wheel preparation/verification, dry-run
behavior and dependency failure before printer writes/restart. Integration
fixtures cover same-origin inline display only with correlated Moonraker paths,
path fallback, formatted-console limits and safe URL handling. No real package
installation or printer/frontend service operation is performed by these tests.

Focused motor-enable and fresh-reference fixtures, run from
`Beacon_Accel_Homing/tests/`:

```bash
python -m unittest test_session.EnablePreparationTest test_backends.GenericPreparationTest -v
```

The recorded combined run includes **133 installer/bootstrap/uninstall tests:
132 passed and one real POSIX symlink test was skipped** because the Windows
host does not permit symlink creation. Temporary regular-file fixtures cover
installation, config idempotence, backup/rollback, full-history restore,
failed-write recovery, concurrent-edit rejection and later-edit protection.
Full restore/link-history checks run with isolated link-state emulation.
Actual symlink creation must still be exercised on a Linux host before
claiming a host integration test. All three shell entry points retain their source-base bytes and passed
bash -n without execution. No native Linux systemd/Unix-socket
integration run was performed here.

The need for a full host-process restart follows inspected upstream
import/restart source, documented in
[current activation reference](INSTALLATION.md#service-activation).
Installer activation fixtures use mocked systemd/process/privilege operations.
They cover default install/update restart, `--no-restart`, selected-unit and
live-process identity checks, a fresh idle/pause check before activation,
authorization before writes, unchanged-package activation, and retained
installation/backups on restart failure. Dry run, uninstall and rollback do
not automatically restart. No real service restart or live module reload was
tested. Configuration-only edits retain console `RESTART` and normal
calibration `SAVE_CONFIG` behavior.

The restart coverage includes **15 `test_installer.RestartFixture` tests**.
The final combined offline verification run discovered **558 tests: 557 passed,
zero failures/errors, and the same one Windows symlink skip**. The portable
machine-readable report and complete referenced log are together under
[`validation/VALIDATION.json`](../validation/VALIDATION.json) and
[`validation/tests.txt`](../validation/tests.txt).

Historical executed installer fixtures cover main/include discovery,
stopped/running/multiple instances, process command and service parsing, configured sensor selection,
mixed-case names, generic speed limits, legacy Beacon settings, wildcard include
deduplication, unchanged saved blocks/macros, required-file conflicts, custom
G28 safeguards and the former blanket homing_override refusal, standard
safe_z_home preservation, read-only dry run, wrong-socket identity, printing refusal, immutable-ref downloads, pinned
Git blob integrity, valid-Python-prefix
truncation, shell invocation truncation, cache integrity, and aborted downloads.
Publication fixtures also execute both bootstrap entry points with mocked
downloads to verify their FrankRamm testing-fork defaults, immutable revision
pinning and forwarded installer arguments, and verify that the real source
package retains its GPLv3 license and original CloakedWayne credit/provenance.
Installer speed fixtures verify preservation of configured homing speeds and
finite effective plans before writes. Explicit `--homing-speed` edits remain
opt-in; a targeted X=75/Y=50 fixture uses no travel-budget flags. Absent
`max_report_travel` and `max_startup_travel` settings disable those optional
admission policies. There is no default plugin 50 mm/s ceiling or nominal
startup-distance cap. Firmware velocity limits, actual accelerometer
freshness/rate/fault checks and the existing `no_motion` deadline remain.
If a user explicitly supplies either travel setting, that policy is checked
independently. Acceleration, report interval and sample-age settings are never
automatically changed to admit a speed. Updating also does not infer a
historical speed from backups.
Prepared 75 and 100 mm/s synthetic plans exercise both directions through
fresh-reference acquisition, arming and watchdog handling. Moving traces
establish motion; parked varying traces still fail with `no_motion` at the
unchanged calculated deadline. Separate advanced-policy fixtures exercise
explicit 1 mm/3 mm constraints and constructed faster reporting. These
signals do not qualify real hardware timing or stopping distances.

Full-uninstall fixtures cover multiple upgrades back to the original
activation/config state, restored original links and sensorless settings,
unrelated later edits and saved Beacon/bed-mesh data, removal of introduced
plugin models, restoration of original plugin models, and strict undo of
the uninstall transaction. They also cover repeated uninstall, CRLF files,
SAVE_CONFIG-commented original model options, incomplete history, missing
original sources, malformed/conflicting records and unowned plugin references.
Shared-checkout fixtures reject module-link changes when another discovered
config uses the checkout, including an unreadable peer config or a new
consumer found after planning. Hosted shell fixtures use mocked downloads
and exercise verified offline-cache dispatch and download/refusal paths.

Core/backend and workflow suites separately cover trigger/session behavior,
mapped generic data and units, timing/rate/error guards, staged/independent
models, explicit `CENTERED=1` consent, verified stock G28 requirements,
suspect travel without automatic returns, and measured chart export. Synthetic
chart fixtures are explicitly marked as simulated.

Historical safe Z compatibility fixtures verify the recognized built-in handler over
stock `G28`, including a nonzero configured `z_hop`. CALIBRATE, TEST, CHARTS and ACCURACY
use only the verified stock X/Y handler, with no safe Z hop or change to the
public `G28` registration. Handler validation repeats before each trial.
Unknown/custom wrappers, nested handler chains, mismatched registrations and
G28 macros remain refused. That historical revision also refused `homing_override`;
the current built-in override support is source-reviewed only, as noted above.
These synthetic checks do not
validate nozzle clearance, Cartographer probing or physical homing behavior.

Centered-calibration fixtures check `ACCEL_HOMING_CALIBRATE AXIS=X CENTERED=1`
as the start/resume command, with both XY axes unhomed before it can move or
change workflow state. The first home tests the 0.5 g default candidate from
the operator's manual center; a clean run has one qualifying discovery and ten
validation homes. Early triggers increase the default candidate to 1.0 g,
release/invalidate XY and wait for manual recentering followed by the same
calibration command, preserving the pending candidate and attempt count.
Automatic recentering follows only accepted homes. TEST/CHARTS start directly
with `ACCEL_HOMING_TEST CENTERED=1` and
`ACCEL_HOMING_CHARTS_GENERATE CENTERED=1`, from an existing manual XY center
with both axes unhomed. They do not release XY before the first home. Missing
consent or a homed XY axis fails before movement or a workflow-state change.
These checks do not detect or prove physical centering.

Repeatability fixtures require **ten distinct consecutive geometry-qualified
validation homes** after the separate discovery at the same candidate, with
`max(travel_mm) - min(travel_mm) < 1.5 mm`. They cover equality rejection,
decimal endpoint subtraction across binary exponent boundaries, just-below
acceptance, either-direction outliers, discovery/manual-center exclusion,
independent X/Y windows and recorded range/limit metadata. The previously
reported ten-distance example spans 3.625 mm and is rejected. A range failure
aborts before another automatic return/home, candidate increase, model staging,
config write or SAVE_CONFIG dispatch. Existing early hits, including a result
below the broad geometry tolerance, retain retry behavior while clearing all
validation evidence and requiring a fresh discovery. The save-path fixtures
also reject incomplete, reused, foreign-axis/candidate/run or malformed windows
and recompute the range before writes. Legacy saved models remain accepted;
their old distances do not acquire the new qualification on loading.

Accuracy workflow/report fixtures are in `test_accuracy_workflow` and
`test_accuracy_charts`. `ACCEL_HOMING_ACCURACY` accepts X, Y or XY (default),
with 2–50 measured homes per axis and a default of ten. Each axis first needs
a separate geometry-qualified discovery home from the manually confirmed
center. That home is excluded from the measured statistics. Qualified homes
authorize the following return to center; the final X return occurs before Y
discovery. Missing motion, stale or invalid captures and sensor faults stop the
run. Early triggers with complete current halt/capture and center-reference
evidence can hand off to one standard calibration, as described below. The
maximum successful XY accuracy case makes 102 synthetic homes independently of
the calibration attempt limit. Measurements keep fixed thresholds and configured
homing speeds; no workflow requests a Z move.

Known-distance fixtures verify independent X/Y count, minimum, maximum, mean,
max-minus-min range, population standard deviation and population variance
(divide by the measured count). Discovery/manual centering stays excluded.
Qualified observations remain in the report when their range is at or above
1.5 mm; this command does not apply calibration's acceptance rule. Failed or
cancelled runs retain measured observations and any published final summaries,
remove temporary per-home PNGs, and report their measured/requested counts as
incomplete. Fewer than two measured homes leave standard deviation
and variance unavailable. Tests cover cancellation and shutdown during waits,
recenter/export errors and a final restoration error; none can publish a
successful run after that failure.

Each bounded raw capture is bound to its current detector attempt and source
record. Tests reject a previous pass's baseline, reused records, nonadvancing
arm times, multiple detector attempts, absent pre-arm samples and incomplete
impact data. Exact detector threshold/onset/peak values survive a 0.001 g
configured threshold; negative/permuted native sensor mappings retain the
machine-axis sign. Capture conversion happens after trigger handling, and
raw sample arrays are released after each trial and excluded from status.
Temporary per-home PNGs are validated before the final per-axis overview PNGs
preserve measured-acceleration plots, precise signed trigger thresholds and
commanded-travel statistics. Only the final overviews are retained. Raw arrays
are released after bounded trace/metric preparation; private immutable receipts
are excluded from status JSON. No image pixels or sidecars are used to invent
sensor samples. Synthetic visual checks do not validate physical centering,
nozzle clearance, impact behavior or mechanical accuracy.

`test_accuracy_recovery.py` adds 26 synthetic regression methods for automatic
recalibration after an early accuracy trigger. Parameterized cases cover X/Y,
both homing directions, Cartesian/CoreXY, discovery and later measured failures,
and X/Y failures within an XY run. They verify a signed halt return including
overshoot, zero-step pressed/impact handling, an unchanged other axis/Z/extruder,
no coordinate rebasing or motor release, exact effective threshold +0.25 g and
the normal configured candidate step. The standard calibration still measures
fresh noise and requires discovery plus ten qualified validation homes, including
candidate resets for excessive spread.

Failure cases cover stale/missing/wrong-sign step evidence, changed motor counts
or coordinates, unavailable initial references, native/capture sensor faults,
threshold bounds, cancellation during return/summary/cleanup/handoff, real
printing/pause/SD state, and fresh-noise failures. The owned recovery transaction
admits its own `idle_timeout: Printing` state after motion while retaining actual
printing, queue, sensor and cancellation checks. Summaries and temporary cleanup
must finish before calibration. Reload fixtures verify model and manual shared,
axis and legacy-section threshold precedence; save errors report `save_failed`.

On 2026-10-29, the new 26 methods passed with WSL Ubuntu / CPython 3.12. A bounded
147-method accuracy/calibration run retained two pre-existing failures: the stock
G28 motor-off fixture's Z expectation and a plotting test without installed
matplotlib. The remaining 145 methods passed. These tests use synthetic motion
and sensor objects only; no printer was accessed or physically tested.

Direct-command fixtures check that no separate centering-confirmation command
is registered and no `action:prompt_*` messages are emitted during successful
calibration/test/chart runs, early-trigger recentering, cancellation, timeout
or failure. Early calibration hits retain console instructions, the same
calibration resume command, candidate/attempt state and pending timeout.

Save-and-reload fixtures run the real named-model loader for both X and Y,
including a clean 0.5 g calibration and a 1.0 g calibration after an early trigger.
They check that the saved threshold and ten successes survive restart loading,
with the same sensor/geometry checks and no extra homing request. Both axes
reject saved-model thresholds below 0.5 g and nonfinite values; previous models
at 1 g and 2 g remain accepted. Calibration/model bounds remain 0.5–15 g.
Manual configuration and temporary SET commands accept finite positive values
without a fixed 3 g minimum or 15 g maximum; known sensor range remains limited
to 80% of full-scale and the actual session scale is rechecked before arming.
The `SAVE_CONFIG=1` default retains automatic write/restart dispatch.
`SAVE_CONFIG=0` fixtures check model staging without write/restart dispatch
and restoration of the original active thresholds; saving and reloading are
still required to activate that model. All save/restart operations in these
fixtures are simulated.

Manual-threshold fixtures add **11 tests** in
`test_manual_thresholds.ManualThresholdRangeTests`, with backend, current/legacy
section, X/Y, shared-fallback and both SET-command alias subcases. They accept
0.5 g and smaller positive values, including 0.001 g and the minimum positive
IEEE-754 double, and values above 15 g when the actual sensor range permits.
They reject zero, negative and nonfinite values; accept exactly 80% of a known
range and reject the immediately following representable value; recheck a
narrower active Beacon stream for SET and before arming; and reject raw-limit
overflow/underflow before dispatch without a new decimal floor. Unknown native
Beacon clip ranges keep their existing behavior. Manual SET/status feedback
retains 0.001 g without displaying zero. Two installer fixtures preserve
existing shared/X/Y values below 3 g through initial installation and update
in either section. The next-float fixture uses portable struct operations;
Python 3.8 runtime compatibility was not exercised.

The recorded run used CPython 3.13.5 on Windows: **558 tests in 14.285 seconds**
(14.56137 seconds subprocess wall time). The full UTF-8 unittest output is
retained in `validation/tests.txt` as its exact binary-captured bytes, without
newline conversion: **95,452 bytes and 563 CRLF pairs**. The JSON report hashes
that stored log and all **25 Python sources both before and after** the run;
the two source snapshots match. All **40 package files** were also identical
between the snapshots taken during the suite; this summary, the new log and
the validation report were written afterward. The package inventory contains **40 files**,
with **23 immutable bootstrap payload files**; no private user log
or raw hardware samples are included.

Motor-enable and fresh-reference checks include **26 tests** in
`test_session.EnablePreparationTest` (16) and
`test_backends.GenericPreparationTest` (10), with parameterized subcases. They
cover disabled, partly enabled and already enabled CoreXY motors; actual
completion of queued settling/backoff/retract before the cutoff; a fresh
reference for the 0.5 g candidate and every pass; exclusion of old/buffered
impacts; ringdown and noise on all sensor axes; and rejection of invalid,
stale, future, nonmonotonic, gapped, clipped or scale-changed references.
They also exercise bounded failure with persistent noise or no data,
long references independent of retained-batch capacity, cancellation during
queue waits and acquisition, Beacon clock conversion/wrap, pending-sample
freshness, preservation of an immediate real impact in a batch crossing
arming, and other accelerometer clients surviving success/failure cleanup.

The diagnostic fixtures exercise ADXL-style 3200 Hz timestamps rounded to
microseconds, a 100 ms reference with a 10-batch retained deque, and a separate
print-clock epoch. They distinguish acquisition timeout from expiry,
invalidation or a missing reference before arming. Retained summaries include
current and best quiet candidate counts/spans, X/Y/Z peak-to-peak variation in
g, reset/pre-cutoff/accepted counts, and sample/batch ages in ms. Ages are
snapshots at the reported gate, not live readings after cleanup. Native Beacon
fixtures verify machine-axis mapping, status-copy isolation and rejection of
stale oldest data even when the actual newest batch sample is fresh.

Configured baseline-floor checks in
`test_backends.BaselineFloorTest` and `test_backends.RaisedBaselineFloorTest`
use parameterized subcases. The finite
`baseline_noise_floor` accepts 0–0.7 g and defaults to 0.7 g for configured
batch backends. Their effective quiet limit is the larger of that floor and
20% of the active impact threshold. A zero floor restores the prior rule;
explicit lower floors remain active, and thresholds at least 3.5 g retain
their prior limit. Native Beacon independently
uses `max(0.20 * threshold, min(0.5 g, threshold))`; the generic setting does
not control it. At a 0.5 g candidate, native preparation accepts at most
0.5 g peak-to-peak variation on every sensor axis while the actual impact
threshold remains 0.5 g. Neither policy reduces the required reference
sample count, duration or freshness.

The wider generic-floor fixtures use constructed 3200 Hz samples with
X/Y/Z peak-to-peak ranges of 0.581/0.392/0.258 g, matching a reported summary.
They are synthetic regression inputs, not a recording from the printer.
They check full-duration reference admission, all-axis rejection above 0.7 g,
unchanged raw impact thresholds, retained measured-noise bounds, and rejection
of stationary bias as movement. A separate ramp with sufficient signal margin
checks that motion can still be proved; a wider admitted reference does not
guarantee that every speed, acceleration or noise trace will prove motion.

**15 tests** in `test_motion_policy.NativeExpandedReferenceTests` use constructed native
Beacon samples with X/Y/Z peak-to-peak ranges of approximately
0.367/0.430/0.422 g. Those values are regression inputs based on a reported
summary, not a replay of raw printer data or a new measurement. The fixtures
check reference admission at 0.5 g, the cap at lower manual thresholds,
all-axis rejection above the floor, balanced/biased/asymmetric parked noise,
and motion/contact behavior across X/Y directions and batch/watchdog phases.
Movement uses conservative lower velocity bounds; a retained early peak must
reach half the full effective speed and still needs the later commanded
history. Pressed detection uses the current upper bound, so ambiguous
low-acceleration evidence fails with `no_motion`.

Additional cases retain valid motion across sensor-clock wrap, reject an
exact-end duplicate across that wrap, and reject nonfinite derived deadlines
before arming. A changing parked bias cannot select an impact direction or
hide either first-contact sign. A sufficiently small current upper bound
still allows a justified expanded-reference pressed start after the usual
two distinct checks; the wider-noise parked fixtures remain `no_motion`.

The full 75 mm/s, 5000 mm/s² ramp fixtures use a 1 g detector threshold to
observe motion without the ramp itself exceeding the trigger. A separate
0.5 g fixture verifies that real startup acceleration above 0.5 g still
triggers immediately and is rejected by workflow qualification. Increasing
quiet admission does not make a 0.5 g candidate qualify at every acceleration.
Both first-contact directions remain eligible, including a single retained
post-arm sample after an old pre-arm transient. Expanded-native stale,
future, duplicate, reversed, zero-interval and nonfinite packets are refused;
gaps before motion proof cannot establish it, and frozen startup traces fail
closed. Their finite startup
calculation is 90 ms at 75 mm/s, 5000 mm/s² and a 50 ms native stream timeout,
bounded also by the existing travel deadline. These are synthetic software
requests, not measured stopping latency or a hardware guarantee. Quiet native
references retain the legacy pressed-start and stream watchdog path.

**14 tests** in `test_motion_policy.NativeStepEpochTests` exercise the added
native-expanded interval proof. Synthetic step histories include queued
startup delay separately from sensor/step skew and vary watchdog phase.
The interval uses the first recorded commanded step, a coherent X/Y/Z noise
bound and the existing commanded-history check, while retaining the original
arm-relative deadline. Tests do not reconstruct a ramp that occurred before
arming or accept stationary bias, later jolts or subthreshold ringing as
startup proof. First and later raw contact samples remain eligible.

History is limited to 4096 prefix entries, including its initial entry;
overflow requests a host stop with `proof_overflow` rather than evicting
earlier evidence. Each actual watchdog callback shares one budget of at most
1024 interval comparisons. Fixtures check this bound, prohibit step-history
queries from the sensor callback, cover clock wrap and attempt isolation,
and retain failure diagnostics through session/capture cleanup. Diagnostic
failures after dispatch cannot prevent the stop request. A step-history query
fault cannot fall through to cached full-arm proof. A later sensor disconnect
still fails after verified startup.

The retained scalar `last.<axis>.motion_proof` summary reports planned
effective speed/acceleration, step and sensor timing, full-arm/window bounds,
command eligibility, request timing and bounded-history state. It separates
the bounds' evaluated sample time from a newer accepted sample time and
contains no raw sample array. The synthetic reproduction establishes a
software timing failure mode; it does not prove every live failure's cause
or physical motor travel. A separate broadened matched step/sensor ramp still
fails conservatively, so these tests do not qualify arbitrary physical ramp
shapes. No quiet-floor, impact-threshold, speed or timeout relaxation is part
of this correction.

For a configured reference whose actual accepted variation exceeds the old
20% gate, movement proof uses a conservative lower bound. Each axis freezes
`b = max(abs(low - mean), abs(high - mean))`; it subtracts `N*b` from the
absolute sum of baseline-relative readings, clamps at zero, then uses the
existing physical velocity conversion and vector norm. One atomic snapshot
contains sums, sample count, sample interval and final sample clock; `N`
includes the first accumulated sample. Automatic impact direction requires
the corresponding signed velocity interval to clear +30 or -30 mm/s;
otherwise raw impacts remain bidirectional. No raw post-arm sample or
integral is filtered, and raw first-impact detection and startup deadlines
remain unchanged. References admitted by the old gate retain their old path.
The status summary exposes `baseline.motion_proof_noise_bound_g` when this
correction applies. Statistics use constant-size summaries.

A floor-only experiment reproduced false movement proof with a parked,
changing-mean signal. The final fixtures reject balanced, slow-block and
asymmetric-reference bias on one or three axes, using the maximum deviation
from the mean rather than half the peak-to-peak range. Actual synthetic host
stop requests meet the unchanged startup timeout plus one 3 ms watchdog
interval; the modeled 50 mm/s request is also below 3 mm without establishing
a default distance limit. True 50 mm/s ramps at
4000 mm/s² prove movement in both directions and across pseudo-clock wrap;
that ideal acceleration stays below the unchanged 0.5 g impact threshold.
Separate first-sample ±0.51 g contacts still trigger, including after uncertain
raw integrated speed exceeds 30 mm/s. A real 1 mm/s synthetic ramp with too
little evidence under the expanded noise envelope fails with `no_motion`;
no speed or acceleration is automatically changed. Repeated 0.21 g excursions
and stale, future or pre-cutoff-only data still abort before arming. Modeled
0.164 g ranges are constructed signals, not a replay of a user's raw data.

The noise envelope is an observation from preparation, not a guaranteed bound
on future disturbances. These fixtures establish software behavior only and
do not certify hardware noise, physical contact or stopping distance.

These fixtures advance fake host/print/sensor clocks and check ordering before
trigger dispatch is armed. They do not measure mechanical settling, qualify
the configured floor or native bounded floor on hardware, or prevent a driver's
first-step hold-to-run current transient. No physical homing or motor-current measurement was made.
The 0.5 g starting value does not establish a physical fix for skipped steps
or inconsistent homing. Commanded/firmware step travel is not an encoder
measurement. The default 20% center-to-stop geometry check and new fixed
less-than-1.5 mm validation range check do not establish precise physical
repeatability, correct-stop contact or absence of skipped steps.

`test_motion_policy.py` adds offline coverage for relative motion proof at low
speeds, preservation of quiet native Beacon pressed-start behavior, generic
`no_motion` failures, startup deadline/budget calculations, and rejection of
plans above an explicitly configured 3 mm nominal limit before enabling/backoff.
Additional fixtures cover read-only requested-axis preflight from an unhomed
workflow, live limits lowered after configuration, invalid limits, zero and
nonzero plugin caps, and rejection before workflow effects. Diagnostics include
effective speed/acceleration and, when a travel policy is enabled, its speed
ceiling. With unchanged timing, 50 mm/s calculates about 3.233333 mm at live
1500 mm/s², compared with 2.65 mm at 5000 mm/s²; these distances do not create
a default admission limit. Repeated checks protect later preparation. Normal public G28 and
its safe Z wrapper remain unchanged; these tests do not claim a workflow
preflight can prevent that public wrapper's configured Z hop.
Those checks establish software behavior only; no physical stop travel or
end-to-end host/MCU latency was measured.

Optional policy coverage includes **10 tests** in
`test_motion_policy.ConfigurableStartupBudgetTests`, **9 tests** in
`test_motion_policy.OptionalTravelPolicyTests`, and **31 additional installer
and uninstaller tests**. Both travel settings default to absent/disabled.
Explicit `max_startup_travel` values must be finite and within 0.1–5.0 mm;
explicit `max_report_travel` values retain their 0.1–2.0 mm range. Each opt-in
is enforced independently, with strict `required <= configured` comparison.
An advanced example with 1.5 mm reporting and 4.5 mm startup limits admits a
75 mm/s plan at effective 5000 mm/s² with the original 20 ms sample-age and
5 ms report settings: nominal startup travel is 4.1625 mm and the unchanged
startup timeout is 60 ms. At live 1500 mm/s² that example needs 5.475 mm and
fails its explicit 4.5 mm policy. Without that opt-in, no distance policy
rejects the otherwise numerically valid plan.

Default 75 and 100 mm/s cases pass connect, workflow preflight and real
prepare/arm callbacks without editing speed, freshness, intervals or
acceleration. Real moving and parked traces verify the existing movement
proof and `no_motion` deadline. Missing, stale and frozen streams still
request failure. Absent-policy status uses JSON null and console text
`disabled`, including replay fallbacks with no prepared effective speed.
Finite-positive parameters and finite derived startup timeout, nominal
travel and homing deadline remain mandatory; numeric extremes cannot leak
infinity/NaN into status or create an unbounded deadline. The test harness
returns an absent numeric default of `None` only when the option is absent;
explicit invalid `None` values are not treated as an opt-out.

Installer fixtures exercise the X=75/Y=50 target without budget flags,
default preservation, explicit `--max-report-travel`/`--max-startup-travel`
opt-ins, selected-axis-only speed edits, invalid-value and native-Beacon
refusals, and pre-write validation of retained plugin axes affected by shared
policy edits. Retained active second-pass speeds, retract/rehome distances
and finite positive rail acceleration overrides are checked. Selected rails
use their planned single-pass settings after installer-owned overrides are
removed; unselected rails keep their settings. Uninstall validates owned
opt-in values while retaining later-edit and unrelated-setting protections.

The calibration regression fixtures preserve the strict
**max-minus-min travel <1.5 mm across ten qualified validation homes**, separate
for each calibrated axis. The workflow, core capture and chart source changed
for ACCURACY; preserved behavior is established by the regression fixtures,
not by byte identity for those files. This is a repeatability rule, not an
impact threshold, latency distance or physical stopping guarantee. Optional
travel limits describe nominal command travel before a host request; their
engineering ranges do not certify hardware safety. No current printer
settings were changed and no private user log or raw measurements were
included in this package.

All **25 production and test Python files** listed in
`validation/VALIDATION.json` passed Python 3.8
grammar parsing on the current Python 3.13 host. No Python 3.8 runtime was
executed; syntax parsing alone does not establish complete runtime compatibility.
