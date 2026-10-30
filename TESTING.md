# Offline verification

Run checks from the standalone repository root on a development machine.
The suite uses synthetic sensor/motion/service objects and temporary fixture
directories; it is not a request to operate a printer.

```bash
cd ~/accel_homing
python3 -B -m unittest discover -s tests -v
git diff --check
```

Use a development environment with the plotting dependencies required by the
rendering tests. A missing matplotlib stack, platform-specific symlink
permission or unavailable POSIX shell can affect results; record those facts
rather than presenting an unexecuted/skipped case as passing. Do not run the
installer against a real printer merely to satisfy an offline test.

## Focused checks

```bash
python3 -B -m unittest discover -s tests -p 'test_installer.py' -v
python3 -B -m unittest discover -s tests -p 'test_updater.py' -v
python3 -B -m unittest discover -s tests -p 'test_accuracy_recovery.py' -v
python3 -B -m unittest discover -s tests -p 'test_simple_calibration.py' -v
python3 -B -m unittest discover -s tests -p 'test_calibration_margin.py' -v
python3 -B -m unittest discover -s tests -p 'test_beta_calibration.py' -v
python3 -B -m unittest discover -s tests -p 'test_noise_recovery.py' -v
python3 -B -m unittest discover -s tests -p 'test_failed_chart_publication.py' -v
python3 -B -m unittest discover -s tests -p 'test_filter_charts.py' -v
python3 -B -m unittest discover -s tests -p 'test_filter_packaging.py' -v
python3 -B -m unittest discover -s tests -p 'test_updater_branches.py' -v
```

The tests cover installation/ownership/rollback, adapter contracts and faults,
fresh references, legacy/vector detector behavior, single-pass homing/release,
noise selection, calibration geometry/early returns/repeatability, accuracy
capture/recovery/statistics, PNG rendering and the isolated Beta experiment.
A source test file's presence is not proof that it executed successfully.

For real matplotlib visual QA of the new development figures, on the
development machine only:

```bash
python3 -B tests/render_filter_examples.py --output /tmp/accel-homing-synthetic-plots
```

This renders a RAW-controlled same-trace filter example with a censored
FILTERED confirmation, a successful synthetic RAW accuracy baseline and an
independent synthetic FILTERED A/B report reusing the embedded PNG receipt.
The prominent SYNTHETIC labels are required; none of the data are physical
results. Inspect event markers/units, actual-vs-replay labels, pre-arm noise
interval and spectrum availability, censoring, actual run counts/statistics,
mode/threshold/settings differences and legible scientific tick labels.

For a reproducible report, retain the exact commit, command/working directory,
Python/platform/dependency versions, exit status, full test output and any
failures/skips. Grammar parsing, compilation, imports, real PNG rendering and a
full suite run establish different things; identify which were actually done.

## Current behavior to verify

- Calibration discovers an impact after more than 20 mm inside the configured
  midpoint-to-stop band and excludes discovery. Explicit pure RAW/FILTERED
  needs ten validation homes; default/explicit HYBRID needs five RAW and five
  FILTERED valid checks in one automatic command, with separate discovery and
  no manual transition. Each series requires max-minus-min travel at most
  2.00 mm. A running spread above that rejects the
  candidate immediately. Only after qualification, the staged/saved threshold
  receives a single 0.5 g margin. Trial thresholds stay unchanged; the final
  value must fit sensor headroom without clamping, and metadata reports that
  the higher value was not tested by those homes.
- A paired HYBRID recoverable early-hit/spread failure in either series discards
  both private results, resets both valid counts to zero and restarts from RAW,
  adjusting only the failing detector's candidate. Global attempts never reset;
  a new complete pair needs at least twelve remaining attempts. No incomplete
  first-stage result is reused/saved. Fatal context/stream/geometry/cancellation
  faults abort. Both successful series and cleanup precede the only final save.
  Schema 3 retains honest five-per-profile and ten-total evidence; legacy schema
  1/2 models remain readable without rewriting. Ordinary G28's selected type is
  preserved, while explicit pure calibration keeps its ten-check contract.
- Premature calibration returns reverse current signed start-to-halt commanded
  travel before the next candidate; faults/failed returns do not count as
  completed recovery.
- ACCURACY uses the active threshold and bounded in-memory capture receipts.
  Only a complete workflow publishes final summaries. RAW/FILTERED early-trigger recovery
  requires current evidence, reports failure without a PNG and completes cleanup,
  then starts standard calibration at the failed
  threshold plus exactly 0.25 g. Only successful calibration saves/restarts.
- Stationary noise retries discard complete rejected windows, respect per-window
  and whole-session limits, and keep homing/Beta policies unchanged.
- Failed measurements do not invoke final PNG publication. Every multi-axis
  image is rendered before publication; rollback removes only newly owned files.
- Beta produces a labelled PNG only for a successful restored proposal; failures
  retain bounded console diagnostics without staging/saving thresholds/models
  or restarting.
- Standalone installation uses a complete canonical native Git clone, root
  entrypoints and explicit matching main/origin/main or development branch
  updater configuration. Legacy MISC ownership
  is not silently adopted; uninstall leaves source deletion to the operator.
- FILTERED uses the actual frozen validated stationary reference and actual
  armed sample period. Same-trace replay initializes that exact profile, with
  independently labelled warmed pre-arm diagnostics; pre-arm zeros must never
  masquerade as noise attenuation. Shadow decisions stop strictly before the
  lower bound of actual stop dispatch. Candidate/onset/confirmation differ.
- Explicit RAW/FILTERED calibrated profiles coexist across SAVE_CONFIG/restart;
  RAW thresholds are never implicitly converted. Accuracy BASELINE reads a
  bounded validated successful PNG receipt from the same root/axis and keeps
  actual independent-run statistics separate from replay. Required missing or
  mismatched acceleration/speed/geometry/sensor context prevents attribution.
- CALIBRATE and ACCURACY default to HYBRID only when DETECTOR is omitted;
  explicit RAW/FILTERED/HYBRID overrides remain. SAMPLES and every other command
  default remain unchanged; no global default setting is added.
- Native live RAW uses sign-independent centered absolute acceleration or XYZ
  norm; configured impact_sign/automatic direction does not gate impacts. Live
  native FILTERED/HYBRID requires rectified v2. Legacy/unknown saved selected-axis
  RAW qualification is rejected even with impact_sign zero; compatible XYZ/v2
  remains usable. Original signed native counts/JSON and historical signed-v1
  replay/receipts remain unchanged and labelled. Generic behavior is unchanged.
- HYBRID measurement preflights both saved compatible profiles for every requested
  axis before movement. FILTERED confirmation/rejection and the response anchor
  retain their behavior; an independent RAW XYZ observer supplies only the
  associated position timestamp. Missing/ambiguous RAW evidence falls back with
  explicit provenance and measured counts. Rejected candidates cannot donate a
  timestamp, unsupported commands/overrides fail, and HYBRID ACCURACY early-quality
  failure cannot enter automatic return/recalibration. Historical RAW/FILTERED
  receipts remain readable with their recorded position policy.
- Combined packaging regression installs six modules on main, upgrades the
  seventh module plus development updater branch in the same journal, validates
  complete ownership/migration, and uninstalls while retaining unrelated later
  configuration edits. Arbitrary historical module subsets are not accepted.

These are software contracts, not physical measurements. A passing synthetic
suite cannot establish correct-stop contact, actual motor current, carriage
position, skipped-step absence or end-to-end stopping latency.

## Historical evidence

[Historical verification notes](docs/HISTORICAL_TESTING.md) preserve the earlier
dated narrative. The immutable [validation report](validation/VALIDATION.json)
and [test log](validation/tests.txt) describe an older package snapshot:
2026-10-27 UTC, Windows/CPython 3.13.5, 558 tests with one symlink-permission
skip. The report explicitly has `final_validation: false`.

Those source hashes, counts and timings do not validate this standalone
conversion or subsequent calibration/accuracy changes. Historical Python 3.8
grammar parsing is not execution on a Python 3.8 interpreter. Historical Beacon
printer reports do not validate the generic adapters or new workflows.

Repository migration itself does not add hardware validation. Any physical
qualification must be performed separately by an operator supervising the
printer and documented with the exact revision/configuration used.
