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
python3 -B -m unittest discover -s tests -p 'test_beta_calibration.py' -v
python3 -B -m unittest discover -s tests -p 'test_noise_recovery.py' -v
python3 -B -m unittest discover -s tests -p 'test_failed_chart_publication.py' -v
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
  candidate immediately.
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
- Failed measurements do not invoke final PNG publication. Every multi-axis
  image is rendered before publication; rollback removes only newly owned files.
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
