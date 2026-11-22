# Minimum Home Distance Re-home Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Ordinary G28 homes that travel less than a configured `min_home_dist` are backed off and re-homed automatically; repeated short homes fail instead of being accepted.

**Architecture:** New per-axis options in the plugin section. The existing kinematics hook `BeaconAccelHoming._home_with_retract` checks each accepted rail record's `travel_mm_exact` after `kinematics_home(homing)` returns and before `_retract_completed_home`; short axes are backed off and re-homed through `homing.home_rails([rail], forcepos, homepos)` so the normal plugin session/events/records run again. The installer copies a converted rail's `min_home_dist` into the plugin section.

**Tech Stack:** Python 3, Klipper/Kalico extras API, `unittest` with the fakes in `tests/harness.py`, `tests/test_session.py`, `tests/test_retract_noise.py`.

**Spec:** `docs/superpowers/specs/2026-11-22-min-home-dist-design.md`

## Global Constraints

- Axes: X and Y only (`AXES = {"x": 0, "y": 1}`).
- Options: `min_home_dist` (shared), `x_min_home_dist`, `y_min_home_dist`; mm, `minval=0.0`, default `0.0` = disabled; must be finite.
- Re-home acceptance: `travel >= min_home_dist - SECOND_TOUCH_TOLERANCE` (1.5 mm). First home acceptance: `travel >= min_home_dist`.
- At most 2 re-homes per axis per G28 (`MAX_SHORT_HOME_RETRIES = 2`), then `command_error`.
- Re-home runs at `homing_speed` with the first-pass travel allowance (fresh session, `passes` restarts at 1).
- Back-off target outside rail range raises (never clamp), via `_check_back_off_range`.
- Workflows (`workflows.session`, `workflows.in_home`, `noise_session`) never re-home: they already set `context["valid"] = False`.
- Tests run on Linux only: `docker run --rm -v "D:/Workspace/beattune-core/accel_homing:/src" -w /src python:3.11-slim sh -c "pip install -q numpy matplotlib && python3 -B -m unittest discover -s tests"` (single file: append e.g. `-p test_min_home_dist.py`). Never use Windows results.
- Commit messages end with `Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>`.

## Review Focus

1. Record with `travel_mm_exact is None` (travel calculation failed): cannot prove distance → treat as short (re-home), never accept silently. Test in Task 1.
2. Pressed start / `kept_origin` record (axis already at stop, travel ≈ 0) with `min_home_dist > 0`: re-homes like any short home. Test in Task 1.
3. Multi-axis G28 where only Y is short: X stays homed, not re-homed, final back-off still applies to both with the replaced Y record. Test in Task 1.
4. `min_home_dist` larger than the space between `position_endstop` and the far range limit: raises "out of range" before moving. Test in Task 1.
5. Installer re-run on an already converted config (stepper key already commented out): no duplicate/overwritten plugin option. Test in Task 2.

---

### Task 1: Plugin options and short-home re-home

**Suggested model:** sonnet. **Depends on:** none. **Wave:** 1.

**Files:**
- Modify: `beacon_accel_homing.py` (constants near line 58; options near lines 574-593; `_home_with_retract` lines 769-790; new helper next to `_check_back_off_range` ~line 846)
- Create: `tests/test_min_home_dist.py`

**Interfaces:**
- Produces: `BeaconAccelHoming.min_home_dist: dict[str, float]` keyed `"x"`, `"y"`; `get_status()["min_home_dist"]` = that dict (same shape as `back_off_dist`); module constant `MAX_SHORT_HOME_RETRIES = 2`; method `_rehome_short_axes(self, context) -> None`.
- Error text (Task 3 documents it verbatim): `"accel_homing: %s homed after %s mm, less than min_home_dist %.1f mm, %d times. The stop was not confirmed, homing stopped"` where `%s` axis upper, travel list formatted `"/".join("%.2f" % t ...)` (`"?"` for None).

- [ ] **Step 1: Write failing tests in `tests/test_min_home_dist.py`**

Reuse `test_retract_noise.FinalRetractTests.machine`-style fakes (copy its `machine` into a `MinHomeDistTests.machine` that lets each scripted home set `record["travel_mm_exact"]` from a per-axis list, and give the fake `homing` object a `home_rails(rails, forcepos, homepos)` that records calls, runs one scripted home for that rail and calls `obj._handle_rails_end(homing, rails)`). Tests:
- `test_options_fall_back_to_shared_and_default_off`: `{}` → `{"x":0.0,"y":0.0}`; `{"min_home_dist":"10","y_min_home_dist":"4"}` → `{"x":10.0,"y":4.0}`; `{"x_min_home_dist":"-1"}` raises `ConfigError`; status `min_home_dist` equals the dict.
- `test_long_home_is_accepted_without_rehome`: min 10, travel 150 → `home_rails` not called; final back-off move unchanged.
- `test_short_genuine_home_rehomes_and_accepts`: min 10, travels `[3.0, 9.0]` (9.0 ≥ 10-1.5) → one back-off move to `position_endstop ± 10`, one `home_rails` call with the Y rail only, accepted; final back-off uses the re-home record.
- `test_false_early_trigger_recovered_by_long_rehome`: travels `[3.0, 140.0]` → accepted after one re-home.
- `test_repeated_short_homes_raise_and_never_back_off`: travels `[3.0, 2.0, 2.5]` → `CommandError` matching `"less than min_home_dist 10.0 mm, 3 times"`, exactly 2 `home_rails` calls, no final back-off move.
- `test_zero_disables`: min 0, travel 0.5 → no re-home.
- `test_missing_travel_counts_as_short`: travels `[None, 9.5]` → re-homed and accepted.
- `test_pressed_kept_origin_record_rehomes`: record with `reason="pressed"`, `kept_origin=True`, travel 0.2, min 10 → re-home happens.
- `test_only_short_axis_rehomed_in_xy_home`: axes "xy", X travel 150, Y travels `[2.0, 10.0]` → `home_rails` called once with Y rail; both final back-offs applied.
- `test_back_off_out_of_range_raises_before_move`: bounds (0, 8), min 10 → `CommandError` "out of range", no moves.
- `test_workflow_session_never_rehomes`: set `obj.workflows.session = object()` (or `in_home=True`) → short travel accepted path untouched, no `home_rails`.

- [ ] **Step 2: Run, expect failures** (`-p test_min_home_dist.py` in Docker): AttributeError `min_home_dist` / missing re-home.

- [ ] **Step 3: Parse options** beside `back_off_dist` (line 574): `shared_min_home = config.getfloat("min_home_dist", 0.0, minval=0.0)`, per-axis dict `self.min_home_dist`; add shared and per-axis entries to `scalar_options` (finite check); add to `get_status` next to `back_off_dist`.

- [ ] **Step 4: Implement `_rehome_short_axes(context)`** and call it in `_home_with_retract` between `kinematics_home(homing)` and `_retract_completed_home(context)`, only when `context["valid"]`.

Algorithm per `(axis, (rail, record))` in `list(context["rails"].items())`, skipping axes with `self.min_home_dist[axis] == 0`:
```
minimum = self.min_home_dist[axis]
travels = [record.get("travel_mm_exact")]
limit = minimum                       # first home: strict
while travels[-1] is None or travels[-1] < limit:
    if len(travels) > MAX_SHORT_HOME_RETRIES: raise command_error(<error text>)
    info = rail.get_homing_info(); low, high = rail.get_range()
    away = -1.0 if info.positive_dir else 1.0
    target = info.position_endstop + away * minimum
    self._check_back_off_range(axis, target, low, high)
    coord = [None, None, None]; coord[AXES[axis]] = target
    self.toolhead.manual_move(coord, getattr(info, "retract_speed", None) or info.speed)
    homepos = [None] * 4; homepos[AXES[axis]] = info.position_endstop
    forcepos = list(homepos)  # same full-range force position as Klipper's _home_axis:
    forcepos[AXES[axis]] += -1.5 * (info.position_endstop - low) if info.positive_dir \
                            else 1.5 * (high - info.position_endstop)
    context["rails"].pop(axis, None)
    context["homing"].home_rails([rail], forcepos, homepos)
    entry = context["rails"].get(axis)   # filled by _handle_rails_end
    if entry is None: raise command_error("accel_homing: %s re-home was not accepted" % axis.upper())
    rail, record = entry
    travels.append(record.get("travel_mm_exact"))
    limit = minimum - SECOND_TOUCH_TOLERANCE
logging.info(...) each re-home with axis and travels.
```
Wrap the back-off move with the existing `_limit_accel()` / `_restore_accel()` pair, as `_retract_completed_home` does.

- [ ] **Step 5: Run the new file, then the full suite in Docker.** Expected: all pass (baseline 1349 OK, 9 skipped + new tests).

- [ ] **Step 6: Commit** `beacon_accel_homing.py tests/test_min_home_dist.py` — "Re-home axes whose G28 travel is below min_home_dist".

---

### Task 2: Installer migrates stepper `min_home_dist`

**Suggested model:** sonnet. **Depends on:** none (option names fixed above). **Wave:** 1.

**Files:**
- Modify: `installer.py` (collect before the plugin-section write ~line 644-673; disabling stays at line 681)
- Test: `tests/test_installer.py`

**Interfaces:**
- Consumes: option names `x_min_home_dist` / `y_min_home_dist` (Task 1).
- Produces: managed `[accel_homing]` section (new) or `existing` section (re-run) gains `<axis>_min_home_dist: <value>` for each selected axis whose stepper section has an active numeric `min_home_dist` > 0.

- [ ] **Step 1: Failing tests in `tests/test_installer.py`** (reuse the fixture around line 50 / 811 that has `min_home_dist: 10`):
  - `test_min_home_dist_migrates_into_plugin_section`: planned managed file contains `y_min_home_dist: 10` (axis per fixture) and stepper still shows `# min_home_dist: 10`.
  - `test_min_home_dist_zero_or_absent_not_migrated`: no `_min_home_dist:` line.
  - `test_min_home_dist_invalid_value_changes_nothing`: `min_home_dist: abc` → `InstallError` containing `"No files changed"`.
  - `test_min_home_dist_rerun_does_not_duplicate`: running the installer on its own output yields exactly one `_min_home_dist:` line per axis, value unchanged.
  - `test_min_home_dist_existing_section_gets_value`: existing plugin section path (`existing.replace`) receives the value.

- [ ] **Step 2: Run, expect failures.**

- [ ] **Step 3: Implement.** For each selected axis rail read `rail.get("min_home_dist")` before it is disabled; parse float, require finite and ≥ 0 (else `InstallError("%s min_home_dist must be a finite nonnegative number. No files changed." % axis.upper())`); keep values > 0 in a `migrated` dict `{"<axis>_min_home_dist": "%.12g" % value}`. Write `migrated` exactly like `overrides` into `existing` / new `content`, but not into the "Explicit generic policy overrides" message. Append message `"Moved stepper min_home_dist into [accel_homing] as %s (accel_homing re-homes short homes)."`.

- [ ] **Step 4: Run installer tests then full suite in Docker; expect pass.**

- [ ] **Step 5: Commit** `installer.py tests/test_installer.py` — "Migrate stepper min_home_dist into the plugin section".

---

### Task 3: Documentation

**Suggested model:** haiku. **Depends on:** none (names/text fixed above). **Wave:** 1.

**Files:**
- Modify: `docs/CONFIGURATION.md` (option table after `x_back_off_dist` row, line ~142)
- Modify: `docs/TROUBLESHOOTING.md` (new entry for the error text from Task 1)
- Modify: `docs/INSTALLATION.md` (line ~90 migration note)

- [ ] **Step 1: CONFIGURATION.md rows:**
  - `| \`min_home_dist\` | \`0\` | Minimum G28 travel in mm before a home is trusted; a shorter home backs off this distance and homes again (up to 2 times, at homing_speed). 0 disables. Not used by calibration workflows. |`
  - `| \`x_min_home_dist\`, \`y_min_home_dist\` | \`min_home_dist\` | Per-axis override. |`
  - Add one sentence: the stepper-section `min_home_dist` stays disabled; this plugin option replaces it on Klipper and Kalico.
- [ ] **Step 2: TROUBLESHOOTING.md entry** headed by the error text "homed after … mm, less than min_home_dist … times": meaning (the trigger repeatedly fired before the physical stop, e.g. stepper desync, loose belt, motor snap), checks (belts/motor sync, threshold too low, `min_home_dist` larger than the normal distance from the back-off position).
- [ ] **Step 3: INSTALLATION.md**: after the sentence about commented `min_home_dist`, add that its value is copied into `accel_homing.cfg` as `<axis>_min_home_dist`.
- [ ] **Step 4: Commit** docs — "Document min_home_dist re-home".

---

### Final

- Merge wave 1 (three disjoint file sets) into `dev` feature branch, full Docker suite, then one whole-branch review (sonnet; branch is small and not concurrency-heavy).
