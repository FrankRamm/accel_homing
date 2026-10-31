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
python3 -B -m unittest discover -s tests -p 'test_absolute_charts.py' -v
python3 -B -m unittest discover -s tests -p 'test_sample_age*.py' -v
python3 -B -m unittest discover -s tests -p 'test_warmup_timing.py' -v
```

The tests cover installation/ownership/rollback, adapter contracts and faults,
fresh references, legacy/vector detector behavior, single-pass homing/release,
noise selection, calibration geometry/early returns/repeatability, accuracy
capture/recovery/statistics, PNG rendering and the isolated Beta experiment.
A source test file's presence is not proof that it executed successfully.

For a reproducible report, retain the exact commit, command/working directory,
Python/platform/dependency versions, exit status, full test output and any
failures/skips. Grammar parsing, compilation, imports, real PNG rendering and a
full suite run establish different things; identify which were actually done.

## Current behavior to verify

- Calibration discovers an impact after more than 20 mm inside the configured
  midpoint-to-stop band, excludes discovery and needs ten validation homes with
  max-minus-min travel at most 2.00 mm. A running spread above that rejects the
  candidate immediately. Only after qualification, the staged/saved threshold
  receives a single 0.5 g margin. Trial thresholds stay unchanged; the final
  value must fit sensor headroom without clamping, and metadata reports that
  the higher value was not tested by those homes.
- Premature calibration returns reverse current signed start-to-halt commanded
  travel before the next candidate; faults/failed returns do not count as
  completed recovery.
- ACCURACY uses the active threshold and bounded in-memory capture receipts.
  Only a complete workflow publishes final summaries. Early-trigger recovery
  requires current evidence, reports failure without a PNG and completes cleanup,
  then starts standard calibration at the failed
  threshold plus exactly 0.25 g. Only successful calibration saves/restarts.
- Stationary noise retries discard complete rejected windows, respect per-window
  and whole-session limits, and keep homing/Beta policies unchanged.
- Generic valid current-window data is accepted despite old sensor timestamps,
  including the reported 71.432/8.943 ms batch and 80 ms or longer delays.
  Tests distinguish advancing delayed receipts from real silence, exercise
  watchdog polls and a late post-arm hit, exclude pre-arm-only data from
  hit/liveness evidence, and retain future/continuity/error/overflow/frozen/rate
  guards. Reference arming, stationary windows and Beta use their receipt
  liveness; native Beacon's age protection remains. `max_sample_age` stays a
  nominal planning input, independently of the valid-receipt timeout.
- Failed measurements do not invoke final PNG publication. Every multi-axis
  image is rendered before publication; rollback removes only newly owned files.
- Capture, ACCURACY, stationary/post-calibration noise and Beta PNGs display
  absolute components after signed-reference subtraction. Acceleration limits,
  threshold lines and peak markers are nonnegative and include the full plotted
  range after autoscaling. Raw/reference signs, negative relative event times
  and the precise travel scale/statistics remain intact. Dense bipolar
  envelopes preserve signed extrema and absolute troughs within bounded
  primary/receipt sizes. Range notes count all displayed homes/windows, keep
  native marker and finite-rail totals separate, and disclose uncertain
  references without double counting; legacy provenance can be unavailable.
- Beta produces a labelled PNG only for a successful restored proposal; failures
  retain bounded console diagnostics without staging/saving thresholds/models
  or restarting.
- Standalone installation uses a complete canonical native Git clone, root
  entrypoints and main/origin/main updater configuration. Legacy MISC ownership
  is not silently adopted; uninstall leaves source deletion to the operator.

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
