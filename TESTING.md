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

Select a group by behavior while developing. With `PYTHONPATH=tests`, unittest
loads only the named test modules; their fixture imports do not run the other
modules' test cases. These POSIX shell commands also work in WSL:

```bash
# Per-axis release and interactive center/travel behavior.
PYTHONPATH=tests python3 -B -m unittest -v \
  test_back_off_per_axis test_center_calibration \
  test_travel_calibration test_travel_native_integration

# Sensor motion proof, sample timing and stream guards.
PYTHONPATH=tests python3 -B -m unittest -v \
  test_backends test_motion_policy test_sample_age70 test_sample_age_removal \
  test_warmup_timing test_trigger

# Optional notch response, lifecycle, model identity and upgrade compatibility.
PYTHONPATH=tests python3 -B -m unittest -v test_notch

# Workflow return, cancellation and Beta capture contracts.
PYTHONPATH=tests python3 -B -m unittest -v \
  test_workflows test_calibration_auto_return test_calibration_limits \
  test_beta_capture_charts

# Installation/update transactions in disposable local directories.
PYTHONPATH=tests python3 -B -m unittest -v \
  test_installer test_updater test_dev_installation test_bootstrap_short
```

Use the full discovery command above before reporting the integrated result.
Keep stdout, stderr and exit status together when redirecting a run. For example:

```bash
python3 -B -m unittest discover -s tests -v > tests.log 2>&1
test_status=$?
cat tests.log
exit "$test_status"
```

Individual-file commands remain useful for a narrower change:

```bash
python3 -B -m unittest discover -s tests -p 'test_installer.py' -v
python3 -B -m unittest discover -s tests -p 'test_travel*.py' -v
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
python3 -B -m unittest discover -s tests -p 'test_troubleshoot*.py' -v
```

The tests cover installation/ownership/rollback, adapter contracts and faults,
fresh references, legacy/vector detector behavior, single-pass homing/release,
noise selection, calibration geometry/early returns/repeatability, accuracy
capture/recovery/statistics, PNG rendering and the isolated Beta experiment.
TROUBLESHOOT adds fixed-setting motion equivalence, passive stop ordering,
partial evidence retention, bounded collection and post-cleanup export checks.
A source test file's presence is not proof that it executed successfully.

The optional dev notch needs independent signal-response and integration checks:
target-band attenuation with retained off-band response, first-crossing timing,
packet partition invariance, fresh state per home, raw sensor-fault handling,
rate/Nyquist validation, saved-model filter identity and mixed-axis persistence.
Disabled profiles must retain the existing detector, while filtered diagnostics
must retain original samples and reproduce the causal decision. These software
checks do not select a frequency or establish physical impact discrimination.

Travel-wizard tests cover negative/positive trigger-versus-halt accounting,
CoreXY/Cartesian conversion, supervised setup, selected-axis bounded jogs,
X-to-Y continuation, cancellation during recenter segments, individual X/Y/Z
motor disable/re-enable, external movement/settings and command failures,
prompt reopening, source/include/pending/model conflicts, combined coordinate
saves, autosave rollback and compatibility with older installed module links.
They simulate motion and do not establish physical sensor/stop behavior.

For a reproducible report, retain the exact commit, command/working directory,
Python/platform/dependency versions, exit status, full test output and any
failures/skips. Grammar parsing, compilation, imports, real PNG rendering and a
full suite run establish different things; identify which were actually done.

The 2026-11-10 baseline at `872ac70d7ed138d7386271355c9f331dc8901a2f`
discovered 1,098 test cases: 1,098 unique test IDs and 1,098 unique test method
definitions, with no repeated imported or inherited cases. Its 18 reported
failure/error records came from eight methods (subtests are reported separately):

| Records | Cause in the synthetic tests | Correction |
| --- | --- | --- |
| 6 | Baseline-noise fixtures assumed the former 20 ms / 3 mm startup settings while using the current 70 ms / opt-in-limit defaults. | Select the original policy explicitly; retain the original time/travel assertions. Current-default coverage remains in `test_motion_policy.py`. |
| 1 | Beta moving-capture success fixture supplied 10 samples; the existing minimum is 50. | Supply 50 changing samples over two receipts and retain the end-time, offset and cleanup checks. |
| 3 | Fake native homing dereferenced a cleared workflow session after cancellation/disconnect/shutdown. | Retain that home's session context so production cancellation and cleanup can complete. |
| 6 | Direct recenter fixtures omitted the accuracy metadata normally recorded by `_home`. | Provide the home-start and generation context; retain acceleration, movement and settle assertions. |
| 1 | Fake `manual_move` indexed Z after receiving only XY coordinates from safe-Z homing. | Treat omitted coordinates as unchanged, as the native call does. |
| 1 | A stock homing error was expected to preserve Z despite the fixture modeling stock `motor_off`. | Assert all motors disabled/unhomed for the stock error, and Z preserved for a failed plugin return. |

These corrections do not remove test methods or change runtime behavior. The
baseline count is a dated inventory, not a target for future suites. The real
absolute-renderer check also requires all four JSON result groups before
checking axis limits; a successful subprocess with truncated results cannot
silently skip a plot kind.

## Current behavior to verify

- Calibration discovers an impact after more than 20 mm inside the configured
  midpoint-to-stop band, excludes discovery and needs ten validation homes with
  max-minus-min travel at most 2.00 mm. A running spread above that rejects the
  candidate immediately. Only after qualification, the staged/saved threshold
  receives a single 0.5 g margin. Trial thresholds stay unchanged; the final
  value must fit sensor headroom without clamping, and metadata reports that
  the higher value was not tested by those homes.
- Ordinary and Beta calibration have no total homing-attempt limit. Discovery
  and all ten validation checks can complete after more than 40 homes; the
  deprecated `calibration_max_attempts` option is consumed without numeric
  validation and ignored. Attempt counts remain diagnostic, with null limit
  and remaining fields. Cancellation, disconnects and faults still stop
  progression without partial qualification.
- Premature calibration returns reverse current signed start-to-halt commanded
  travel before the next candidate; faults/failed returns do not count as
  completed recovery.
- ACCURACY uses the active threshold and bounded in-memory capture receipts.
  Only a complete workflow publishes final summaries. Early-trigger recovery
  requires current evidence, reports failure without a PNG and completes cleanup,
  then starts standard calibration at the failed
  threshold plus exactly 0.25 g. Only successful calibration saves/restarts.
- TROUBLESHOOT keeps the active threshold, speed and acceleration fixed. It
  uses the same native homes and successful return/release sequence as ACCURACY,
  stops on a rejected home without recovery calibration, and retains available
  partial evidence. Diagnostic failures cannot suppress the stop request.
  Files and PNGs are exported after motion cleanup; rendering failure preserves
  the JSON and raw data. Retention limits and omissions are explicit.
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
- Failed standard chart measurements do not invoke final PNG publication. Every multi-axis
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
  or restarting. Its newest 16 diagnostic home captures retain real attempt
  numbers and report omitted/retained counts and ranges, including after more
  than 100 homes; qualification still uses every required trial record.
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
