# Minimum home distance with automatic re-home — design

Date: 2026-11-22. Status: draft, awaiting approval.

## Intent

A user saw wrong homes caused by stepper unsync: the accelerometer triggered after
only a few mm, well away from the physical stop, and Klipper accepted that position
as home. We want accel_homing to behave like Klipper/Kalico homing with a
"minimum home distance": a home that travelled less than `min_home_dist` is not
trusted. The plugin backs off and homes again, and only accepts a home that proves
it reached the physical stop. A position that was never confirmed against the stop
must never be accepted.

## Goals

- Configurable minimum home distance per XY axis, in the plugin section.
- Same behaviour on Klipper and Kalico.
- An early false trigger is recovered automatically (the re-home continues to the
  real stop) instead of only failing.
- Repeated short homes end in a clear error, never in an accepted position.
- Existing `[stepper_*] min_home_dist` values are carried over by the installer.

## Non-goals

- No confirmation touch after long homes (a false trigger mid-travel is out of scope;
  the observed failure stops after a few mm).
- No change to calibration/travel/center workflows; they keep bypassing ordinary G28
  homing and keep requiring native `homing_retract_dist: 0` / `min_home_dist: 0`.
- No re-enabling of native `homing_retract_dist` / Kalico `min_home_dist`.
- No Z axis (the plugin homes X/Y only, `AXES = {"x": 0, "y": 1}`).

## Anti-goals

- Accepting a home whose travel was below the threshold without a successful re-home.
- Double back-off/re-home from Kalico's native logic running alongside ours.
- A slower second touch (soft impacts are harder to detect).

## Approaches considered

- **A — plugin-owned check (chosen).** After an ordinary G28 home the plugin knows the
  stepper travel. If it is below `min_home_dist`, back off by `min_home_dist` and home
  that axis again at `homing_speed` with the full travel allowance. Recovers from
  early false triggers, identical on both firmwares, no conflict with installer or
  workflows.
- **B — native firmware options.** Re-enable Kalico `min_home_dist` / Klipper
  `homing_retract_dist` plus the existing second-touch check. Rejected: Klipper always
  does a slower second touch, early false triggers end in errors rather than recovery,
  and it conflicts with the installer and workflow requirements.
- Reading the value from `[stepper_*]` was rejected: Kalico would act on it natively
  (double re-home) and on Klipper the key would mean something different.

## Decided

- **Config** (plugin section `[beacon_accel_homing]` / the managed `accel_homing.cfg`),
  following the `back_off_dist` pattern (`beacon_accel_homing.py:574`):
  - `min_home_dist` shared value, `x_min_home_dist` / `y_min_home_dist` overrides,
    mm, `minval=0`, `0` disables.
  - Exposed in `get_status()` / scalar options like `back_off_dist`.
- **Behaviour per axis, ordinary G28 only** (inside the existing `kin.home` hook
  `_home_with_retract`, before the final back-off `_retract_completed_home`):
  1. Home as today. Read the accepted impact travel along the axis (`_travel`, already
     computed at the trigger record).
  2. If `travel >= min_home_dist`: accept.
  3. Else: move away from the stop by `min_home_dist` (range-checked, never clamped,
     like `_check_back_off_range`), then home that axis again at `homing_speed`
     with the normal first-pass travel allowance.
  4. A re-home is accepted when `travel >= min_home_dist - SECOND_TOUCH_TOLERANCE`
     (1.5 mm). A genuine first home therefore passes with travel ≈ `min_home_dist`;
     a false first trigger is replaced by a re-home that runs to the real stop.
  5. Otherwise retry again, at most 2 re-homes; then raise a `command_error` naming
     axis, the travel of each attempt and the threshold. Axis must be left unhomed
     (no accepted position).
- **Installer migration:** when the installer disables a selected rail's
  `min_home_dist` (`installer.py:681`), it writes the former numeric value as
  `<axis>_min_home_dist` into the managed plugin section. Invalid/non-numeric values
  follow the existing "No files changed" error style. Rails without the key get nothing.
- **Docs:** CONFIGURATION.md option table, TROUBLESHOOTING.md entry for the new error,
  INSTALLATION.md note on migration.
- **Tests:** Linux-only unittest (Docker), covering: accept long home; short genuine
  home re-homed and accepted; early false trigger recovered; repeated short homes
  error and leave axis unhomed; disabled when 0; per-axis override; workflows
  unaffected; installer migration incl. invalid value; Klipper and Kalico homing
  shapes.

- **Re-home geometry:** after the short touch the axis position is `position_endstop`
  (true or false). The back-off moves `min_home_dist` away from there; the re-home then
  starts from that point with the same travel allowance/deadline as a first pass
  (`(high - low) * travel_margin`). Travel is measured from the back-off point, so a
  genuine stop gives ≈ `min_home_dist` and a false first trigger gives more. The
  allowance is only a safety deadline; motion ends at the trigger.
- **Re-home record:** the re-home is a fresh first-pass-style home (pass numbering
  restarts), so the native `pass_no >= 2` second-touch check (`beacon_accel_homing.py:1623`)
  does not apply. Every attempt is logged; the accepted re-home record replaces the
  short one in `context["rails"]`, and only it drives the final back-off.
- **Failure:** raise `command_error` from the kinematics hook before the final
  back-off. G28's error path turns motors off, which clears homed state; the builder
  verifies this on both Klipper and Kalico and clears homed state explicitly if not.
- **Coupled kinematics:** `_travel` already converts stepper travel to axis travel via
  `kin.calc_position`, so CoreXY works. Back-off and re-home move only the short
  axis; other already-homed axes stay homed.

## My calls (correct if wrong)

- Default `min_home_dist: 0` (off) so existing setups don't change; the installer
  migration and docs turn it on where users had it.
- Max 2 re-homes, fixed constant, not configurable.
- Re-home uses `homing_speed`, not `second_homing_speed`.
- Tolerance reuses `SECOND_TOUCH_TOLERANCE` (1.5 mm).

## Left to the builder

- The exact mechanism to re-home a single rail through the same plugin path (records,
  thresholds, trigger checks) on both Klipper's and Kalico's `Homing` objects when
  G28 homes several axes, and how the re-home record replaces the short one in
  `context["rails"]` so the final back-off and `_homed` state stay correct.
- How the short-home travel is passed from the trigger record to the hook.
