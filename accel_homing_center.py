# Supervised bidirectional coordinate calibration for accelerometer homing. GPLv3.
"""Development workflow. Step counts estimate travel; they are not an encoder."""
import copy
import datetime
import json
import math
import statistics
import types


AXES = {"x": 0, "y": 1}
TRAVERSES = 10
REPEATABILITY_MM = 2.0
MIN_INITIAL_TRAVEL_MM = 20.0
DEFAULT_SEARCH_MARGIN_MM = 20.0
POLARITY_POLICY = "absolute_after_reference_subtraction_v1"


def _statistics(values):
    return {"count": len(values), "mean_mm": statistics.mean(values),
            "minimum_mm": min(values), "maximum_mm": max(values),
            "spread_mm": max(values) - min(values),
            "standard_deviation_mm": statistics.stdev(values) if len(values) > 1 else 0.0}


class _HomingInfo:
    def __init__(self, original, direction, target):
        self.original = original
        self.positive_dir, self.position_endstop = direction > 0, target
        self.retract_dist = self.min_home_dist = 0.0

    def __getattr__(self, name):
        return getattr(self.original, name)


class _Rail:
    """Supply this pass's direction and finite deadline without editing a rail."""
    def __init__(self, original, direction, distance):
        self.original, self.target = original, direction * distance
        self.info = _HomingInfo(original.get_homing_info(), direction, self.target)

    def get_homing_info(self):
        return self.info

    def get_range(self):
        return min(0.0, self.target), max(0.0, self.target)

    def __getattr__(self, name):
        return getattr(self.original, name)


class CenterCalibration:
    def __init__(self, workflows):
        self.workflows, self.owner = workflows, workflows.owner

    def _error(self, message):
        return self.workflows._error("center calibration: " + message)

    def _check(self, session):
        self.workflows._check_running(session)
        if session.get("center_interrupted"):
            raise self._error(session["center_interrupted"])
        if self.owner.operation_generation != session["center_generation"]:
            raise self._error("motors, connection or operation changed")
        if (self.owner.threshold_g != session["saved_thresholds"]
                or self.owner.impact_sign != session["center_impact_sign"]):
            raise self._error("homing thresholds or impact settings changed")
        if getattr(self.owner, "sensor_fault", None) is not None:
            raise self._error("accelerometer fault: %s" % self.owner.sensor_fault)
        if self.owner.toolhead.get_max_velocity()[0] != session["center_original_motion"][0]:
            raise self._error("live velocity limit changed during measurement")

    def _restore_live_accel(self, accel):
        if self.owner.toolhead.get_max_velocity()[1] != accel:
            self.owner._set_accel(accel)
        if self.owner.toolhead.get_max_velocity()[1] != accel:
            raise self._error("could not restore the original live acceleration limit")

    def interrupt(self, result):
        """Stop an armed search immediately; let its native move unwind first."""
        session = self.workflows.session
        session["center_interrupted"] = result.get("reason", "workflow interrupted")
        session["center_ending"] = dict(result)
        self.owner._sensor_fault("cancelled")
        self.owner._end_session()

    def _native_classes(self):
        native = self.workflows._native_g28()
        namespace = getattr(getattr(native, "__func__", native), "__globals__", {})
        state, move = namespace.get("Homing"), namespace.get("HomingMove")
        if (not isinstance(state, type) or not isinstance(move, type)
                or not callable(getattr(move, "homing_move", None))):
            raise self._error("native Homing/HomingMove APIs are unavailable before motion")
        return state, move

    def _expected_steppers(self, axis):
        kin = self.owner.toolhead.get_kinematics()
        active = [s for s in kin.get_steppers() if s.is_active_axis(axis)]
        attached = self.owner.endstops[axis].get_steppers()
        if not active or {id(s) for s in active} != {id(s) for s in attached}:
            raise self._error("%s endstop lacks complete selected-axis stepper coverage" % axis.upper())
        if len({s.get_name() for s in active}) != len(active):
            raise self._error("ambiguous selected-axis stepper names")
        for stepper in active:
            distance = stepper.get_step_dist()
            if not math.isfinite(distance) or distance <= 0:
                raise self._error("step distance must be finite and positive")
        return active

    def _plan(self, axes, margin):
        state_class, move_class = self._native_classes()
        plans = {}
        for axis in axes:
            rail = self.owner.rails_by_axis[axis]
            low, high = rail.get_range()
            info = rail.get_homing_info()
            span, budget = high - low, high - low + margin
            if (not all(math.isfinite(value) for value in (span, budget))
                    or budget <= MIN_INITIAL_TRAVEL_MM):
                raise self._error("finite search travel must exceed 20 mm")
            endstops = rail.get_endstops()
            if len(endstops) != 1 or endstops[0][0] is not self.owner.endstops[axis]:
                raise self._error("each selected rail must have exactly this module's endstop")
            expected = self._expected_steppers(axis)
            state = state_class(self.workflows.printer)
            state.set_axes([AXES[axis]])
            current, current_context = self.workflows._beta_current_context(axis, purpose="Center calibration")
            rail_accel = getattr(info, "accel", None)
            if rail_accel is not None and (not math.isfinite(rail_accel) or rail_accel <= 0
                    or not callable(getattr(state, "_set_homing_accel", None))):
                raise self._error("configured rail homing acceleration has no supported native helper")
            metric = "xyz_magnitude" if self.owner._uses_vector_detector(axis) else "selected_axis"
            plans[axis] = {"rail": rail, "minimum": low, "maximum": high,
                           "endstop": info.position_endstop, "span": span,
                           "normal_direction": 1 if info.positive_dir else -1,
                           "budget": budget, "margin": margin, "speed": info.speed,
                           "rail_accel": rail_accel, "state": state, "move_class": move_class,
                           "current_helper": current, "current_context": current_context,
                           "steppers": expected, "metric": metric,
                           "minimum_span": max(MIN_INITIAL_TRAVEL_MM,
                                               span * (1.0 - self.workflows.travel_tolerance))}
        return plans

    def _delta(self, before, after, axis):
        kin = self.owner.toolhead.get_kinematics()
        moved = {s.get_name(): (after[s.get_name()] - before[s.get_name()]) * s.get_step_dist()
                 for s in kin.get_steppers()}
        delta = kin.calc_position(moved)
        tolerance = max(0.000001, max(s.get_step_dist() for s in kin.get_steppers()) * 2.0)
        if (len(delta) < 3 or not all(math.isfinite(value) for value in delta[:3])
                or any(abs(value) > tolerance for index, value in enumerate(delta[:3])
                       if index != AXES[axis])):
            raise self._error("stepper evidence contains unexpected cross-axis displacement")
        return delta[AXES[axis]], tolerance

    def _positions(self):
        return {s.get_name(): s.get_mcu_position()
                for s in self.owner.toolhead.get_kinematics().get_steppers()}

    def _measure(self, session, plan, direction, distance, phase):
        self._check(session)
        axis, w, owner = session["axis"], self.workflows, self.owner
        toolhead, kin = owner.toolhead, owner.toolhead.get_kinematics()
        original, limits = owner.rails_by_axis[axis], kin.limits[AXES[axis]]
        proxy = _Rail(original, direction, distance)
        old_record = owner.last.get(axis)
        current_started = accel_started = False
        session["attempts"] += 1
        session["phase"] = phase
        session["center_touched"].add(axis)
        w.in_home, w.home_axis = True, axis
        hmove, operation_error = None, None
        try:
            owner._handle_rails_begin(plan["state"], [proxy])
            self._check(session)
            if not owner.homing_active:
                raise self._error("homing preparation did not remain active")
            if plan["current_helper"] is not None:
                current_started = True
                plan["current_helper"]([AXES[axis]], True)
                self._check(session)
            if plan["rail_accel"] is not None:
                accel_started = True
                plan["state"]._set_homing_accel(plan["rail_accel"], pre_homing=True)
                self._check(session)
            position = toolhead.get_position()
            position[AXES[axis]] = 0.0
            toolhead.set_position(position)
            self._check(session)
            kin.limits[AXES[axis]] = proxy.get_range()
            target = list(position)
            target[AXES[axis]] = direction * distance
            hmove = plan["move_class"](w.printer, proxy.get_endstops())
            # HomingMove's own begin/end events retain the baseline, sensor
            # faults, trsync and trigger/halt bookkeeping of ordinary homing.
            self._check(session)
            hmove.homing_move(target, plan["speed"], probe_pos=True)
            toolhead.wait_moves()
            self._check(session)
            record = owner.last.get(axis)
            if (not isinstance(record, dict) or record is old_record
                    or record.get("axis") != axis or record.get("pass") != 1
                    or record.get("result") != "impact"
                    or record.get("impact_metric") != plan["metric"]
                    or record.get("polarity_policy") != POLARITY_POLICY):
                raise self._error("%s returned no new valid single impact; pressed/early/no-trigger "
                                  "results cannot establish an endpoint" % axis.upper())
            positions = hmove.stepper_positions
            expected = {id(s) for s in plan["steppers"]}
            if (len(positions) != len(expected) or {id(sp.stepper) for sp in positions} != expected
                    or any(type(value) is not int for sp in positions
                           for value in (sp.start_pos, sp.trig_pos, sp.halt_pos))):
                raise self._error("native move returned incomplete trigger/halt stepper evidence")
            zero = self._positions()
            starts, triggers, halts = dict(zero), dict(zero), dict(zero)
            for sp in positions:
                starts[sp.stepper_name], triggers[sp.stepper_name], halts[sp.stepper_name] = (
                    sp.start_pos, sp.trig_pos, sp.halt_pos)
            trigger, tolerance = self._delta(starts, triggers, axis)
            halt, _ = self._delta(starts, halts, axis)
            if (not 0 < direction * trigger <= distance + tolerance
                    or direction * (halt - trigger) < -tolerance):
                raise self._error("trigger/halt evidence has wrong sign or exceeds the explicit search")
            for key, value in (("trigger_travel_mm_signed", trigger), ("halt_travel_mm_signed", halt)):
                reported = record.get(key)
                if type(reported) not in (int, float) or not math.isfinite(reported) \
                        or abs(reported - value) > tolerance:
                    raise self._error("detector and native move disagree on signed step displacement")
            # Assign only a temporary selected-axis coordinate. Homed state is
            # never certified by this workflow and is invalidated on cleanup.
            position = toolhead.get_position()
            position[AXES[axis]] = (plan["maximum"] if direction > 0 else plan["minimum"]) + halt - trigger
            toolhead.set_position(position)
            self._check(session)
            return {"phase": phase, "direction": "+" if direction > 0 else "-",
                    "direction_sign": direction, "search_distance_mm": distance,
                    "trigger_travel_mm_signed": trigger, "halt_travel_mm_signed": halt,
                    "overshoot_mm_signed": halt - trigger,
                    "impact_metric": record["impact_metric"], "threshold_g": owner.threshold_g[axis],
                    "onset_g": record.get("onset_g"), "impact_peak_g": record.get("impact_peak_g"),
                    "physical_position_verified": False}
        except Exception as exc:
            operation_error = exc
            raise
        finally:
            cleanup = []
            if accel_started:
                cleanup.append(("native rail acceleration", lambda: plan["state"]._set_homing_accel(
                    plan["rail_accel"], pre_homing=False)))
            if current_started:
                cleanup.append(("native homing current", lambda: plan["current_helper"]([AXES[axis]], False)))
            cleanup.extend((("sensor session", owner._end_session),
                            ("live acceleration", lambda: self._restore_live_accel(session["center_original_motion"][1]))))
            errors = []
            for name, callback in cleanup:
                try:
                    callback()
                except Exception as exc:
                    errors.append("%s cleanup: %s" % (name, exc))
            owner.rails_by_axis[axis] = original
            kin.limits[AXES[axis]] = limits
            w.in_home, w.home_axis = False, None
            if errors:
                session["center_cleanup_errors"].extend(errors)
                if operation_error is None:
                    raise self._error("; ".join(errors))

    def _axis(self, session, plan):
        direction = plan["normal_direction"]
        initial = self._measure(session, plan, direction, plan["budget"], "initial_positioning")
        if abs(initial["trigger_travel_mm_signed"]) <= MIN_INITIAL_TRAVEL_MM:
            raise self._error("initial positioning impact after <=20 mm; manually recenter and inspect thresholds")
        previous = initial
        samples = []
        for number in range(1, TRAVERSES + 1):
            direction *= -1
            self.owner.toolhead.dwell(self.workflows.calibration_settle_time)
            self.owner.toolhead.wait_moves()
            self._check(session)
            sample = self._measure(session, plan, direction, plan["budget"], "traverse_%d" % number)
            # At a hard stop, commanded trigger->halt steps can stretch a belt
            # or be skipped. Do not subtract them as a physical correction.
            # Primary estimate is the direct stationary halt->next-trigger
            # command distance. Retain the alternate step-coordinate sum only
            # as a diagnostic with its separate uncertainty.
            span = direction * sample["trigger_travel_mm_signed"]
            if span <= MIN_INITIAL_TRAVEL_MM or not plan["minimum_span"] <= span <= plan["budget"] + 0.000001:
                raise self._error("%s traverse %d estimated %.6f mm outside the declared plausible range "
                                  "%.6f..%.6f mm; inspect stops/thresholds and SEARCH_MARGIN" % (
                                      session["axis"].upper(), number, span,
                                      plan["minimum_span"], plan["budget"]))
            sample.update(sample=number, end_to_end_mm=span,
                          commanded_halt_to_trigger_mm=span,
                          previous_overshoot_mm_signed=previous["overshoot_mm_signed"],
                          alternate_commanded_trigger_to_trigger_mm=direction * (
                              previous["overshoot_mm_signed"] + sample["trigger_travel_mm_signed"]))
            samples.append(sample)
            session["center_completed"][session["axis"]] = len(samples)
            self.workflows._info("%s traverse %d/10 %s: estimated halt-to-next-trigger %.6f mm; "
                                 "prior overshoot %+.6f, start-trigger %+.6f, "
                                 "halt %+.6f mm" % (session["axis"].upper(), number,
                                    sample["direction"], span, sample["previous_overshoot_mm_signed"],
                                    sample["trigger_travel_mm_signed"], sample["halt_travel_mm_signed"]))
            self._check(session)
            previous = sample
        stats = _statistics([sample["end_to_end_mm"] for sample in samples])
        directions = {sign: _statistics([sample["end_to_end_mm"] for sample in samples
                                        if sample["direction"] == sign]) for sign in ("+", "-")}
        if stats["spread_mm"] > REPEATABILITY_MM:
            raise self._error("%s ten-traverse spread %.6f mm exceeds %.2f mm; no coordinates proposed" % (
                session["axis"].upper(), stats["spread_mm"], REPEATABILITY_MM))
        proposed_max = plan["minimum"] + stats["mean_mm"]
        proposed_endstop = proposed_max if plan["normal_direction"] > 0 else plan["minimum"]
        return {"samples": samples, "initial_positioning": initial,
                "statistics": stats, "directions": directions,
                "direction_mean_difference_mm": directions["+"]["mean_mm"] - directions["-"]["mean_mm"],
                "repeatability_limit_mm": REPEATABILITY_MM,
                "configured": {"position_min": plan["minimum"], "position_max": plan["maximum"],
                               "position_endstop": plan["endstop"], "homing_positive_dir": plan["normal_direction"] > 0},
                "proposed": {"position_min": plan["minimum"], "position_max": proposed_max,
                             "position_endstop": proposed_endstop,
                             "midpoint": (plan["minimum"] + proposed_max) * 0.5},
                "search": {"configured_span_mm": plan["span"], "extra_allowance_mm": plan["margin"],
                           "per_move_search_budget_mm": plan["budget"],
                           "minimum_accepted_span_mm": plan["minimum_span"]},
                "span_estimator": "commanded_stationary_halt_to_next_trigger_no_overshoot_subtraction",
                "motion": {"homing_speed_mm_s": plan["speed"], "separate_backoff_moves": 0,
                           "rail_homing_accel_mm_s2": plan["rail_accel"],
                           "threshold_g": session["saved_thresholds"][session["axis"]],
                           "impact_metric": plan["metric"], "homing_current": plan["current_context"]},
                "physical_position_verified": False}

    def _active_model(self, axis):
        w = self.workflows
        # SET_ACCEL_HOMING's temporary override disappears at restart. Rebase
        # any selected saved model that will supply its threshold afterwards.
        return w.model is not None and axis in w.model.thresholds and axis not in w.manual_axes

    def _save_keys(self, axes):
        keys = [("stepper_" + axis, name) for axis in axes
                for name in ("position_max", "position_endstop")]
        keys.extend((self.workflows.model_section, axis + "_metadata") for axis in axes if self._active_model(axis))
        return keys

    def _save_snapshot(self, keys):
        """Read-only native include check and snapshot; never write config files."""
        w = self.workflows
        configfile = w.printer.lookup_object("configfile")
        required = ("autosave", "status_save_pending", "save_config_pending", "_read_config_file",
                    "_find_autosave_data", "_strip_duplicates", "_build_config_wrapper", "cmd_SAVE_CONFIG")
        if any(not hasattr(configfile, name) for name in required):
            raise self._error("SAVE_CONFIG=1 needs supported native autosave snapshot/include APIs; use report-only")
        if configfile.save_config_pending or configfile.status_save_pending:
            raise self._error("unrelated SAVE_CONFIG changes are pending; save or discard them separately first")
        parser = configfile.autosave.fileconfig
        filename = w.printer.get_start_args()["config_file"]
        data = configfile._read_config_file(filename)
        regular, _ = configfile._find_autosave_data(data)
        proposed = copy.deepcopy(parser)
        for section, option in keys:
            if not proposed.has_section(section):
                proposed.add_section(section)
            proposed.set(section, option, "center_calibration_include_preflight")
        stripped = configfile._strip_duplicates(regular, types.SimpleNamespace(fileconfig=proposed))
        included = configfile._build_config_wrapper(stripped, filename).fileconfig
        conflicts = ["[%s] %s" % key for key in keys if included.has_option(*key)]
        if conflicts:
            raise self._error("SAVE_CONFIG=1 cannot safely replace included options: %s. "
                              "Move these options into the main printer.cfg first, or use report-only" % ", ".join(conflicts))
        native_save = getattr(type(configfile), "cmd_SAVE_CONFIG", None)
        if not callable(native_save):
            raise self._error("native SAVE_CONFIG implementation is unavailable before motion")
        return {"configfile": configfile, "parser": copy.deepcopy(parser),
                "pending": copy.deepcopy(configfile.status_save_pending),
                "flag": configfile.save_config_pending, "source": data,
                "native_save": types.MethodType(native_save, configfile)}

    @staticmethod
    def _parser_values(parser):
        return [(section, list(parser.items(section, raw=True))) for section in parser.sections()]

    def _edits(self, session, results):
        w, edits = self.workflows, []
        for axis, result in results.items():
            proposed, configured = result["proposed"], result["configured"]
            for name in ("position_max", "position_endstop"):
                if proposed[name] != configured[name]:
                    edits.append(("stepper_" + axis, name, "%.17g" % proposed[name]))
            if self._active_model(axis):
                metadata = copy.deepcopy(w.model.metadata[axis])
                geometry = {"minimum": proposed["position_min"], "maximum": proposed["position_max"],
                            "endstop": proposed["position_endstop"],
                            "positive_dir": configured["homing_positive_dir"], "center": proposed["midpoint"]}
                if metadata["geometry"] != geometry:
                    previous = metadata["geometry"]
                    metadata.setdefault("calibration_geometry", copy.deepcopy(previous))
                    metadata["geometry"] = geometry
                    metadata["center_coordinate_rebase"] = {
                        "run_id": session["run_id"], "previous_geometry": previous,
                        "created_utc": datetime.datetime.now(datetime.timezone.utc).isoformat(),
                        "traverses": TRAVERSES, "travel_statistics": result["statistics"],
                        "measurement_threshold_g": session["saved_thresholds"][axis],
                        "measurement_impact_metric": result["motion"]["impact_metric"],
                        "threshold_calibration_rerun": False, "physical_position_verified": False}
                    edits.append((w.model_section, axis + "_metadata",
                                  json.dumps(metadata, separators=(",", ":"), allow_nan=False)))
        return edits

    def _save(self, snapshot, keys, edits, result, session):
        if not edits:
            self._check_complete(session)
            result["persistence"] = "no_coordinate_changes"
            return
        configfile = snapshot["configfile"]
        command = self.workflows.gcode.create_gcode_command("SAVE_CONFIG", "SAVE_CONFIG", {})
        self.workflows._info("Saving measured coordinate proposal through native SAVE_CONFIG; Klipper will restart")
        fresh = self._save_snapshot(keys)
        if (fresh["configfile"] is not configfile or fresh["source"] != snapshot["source"]
                or self._parser_values(fresh["parser"]) != self._parser_values(snapshot["parser"])):
            raise self._error("configuration changed during measurement; rerun before saving")
        self._check_complete(session)
        try:
            for section, option, value in edits:
                configfile.set(section, option, value)
            snapshot["native_save"](command)
        except Exception:
            # Staging is rolled back as one in-memory transaction. The native
            # firmware owns disk I/O; an I/O failure can leave persistence
            # uncertain and must never be described as a successful save.
            configfile.autosave.fileconfig = snapshot["parser"]
            configfile.status_save_pending = snapshot["pending"]
            configfile.save_config_pending = snapshot["flag"]
            result.update(result="save_failed", save_config=False,
                          persistence="native_save_failed_disk_state_unconfirmed")
            raise
        result.update(save_config=True, persistence="native_save_requested",
                      model_metadata_rebased=list(result["model_metadata_rebase_proposed"]))

    def _check_complete(self, session):
        w = self.workflows
        if (w.session is not None or self.owner.operation_generation != session["center_generation"]
                or w.preflight_generation != session["center_preflight_generation"]
                or (hasattr(w.printer, "is_shutdown") and w.printer.is_shutdown())
                or w._printing() or self.owner.homing_active or self.owner.attempt is not None
                or self.owner.threshold_g != session["saved_thresholds"]
                or self.owner.impact_sign != session["center_impact_sign"]
                or self.owner.back_off_dist != session["saved_back_off"]
                or tuple(self.owner.toolhead.get_max_velocity()) != session["center_original_motion"]):
            raise self._error("calibration context changed after cleanup; no automatic save")

    def _cleanup(self, session, plans):
        errors = list(session["center_cleanup_errors"])
        try:
            self.owner._end_session()
        except Exception as exc:
            errors.append("sensor/acceleration cleanup: %s" % exc)
        try:
            self.owner._restore_accel()
        except Exception as exc:
            errors.append("acceleration restoration: %s" % exc)
        try:
            self._restore_live_accel(session["center_original_motion"][1])
        except Exception as exc:
            errors.append("original live acceleration restoration: %s" % exc)
        try:
            self.owner.toolhead.wait_moves()
        except Exception as exc:
            errors.append("motion completion: %s" % exc)
        finally:
            kin = self.owner.toolhead.get_kinematics()
            for axis in session["center_touched"]:
                self.owner.rails_by_axis[axis] = plans[axis]["rail"]
                kin.limits[AXES[axis]] = (1.0, -1.0)
            self.owner.threshold_g.update(session["saved_thresholds"])
            self.owner.impact_sign.update(session["center_impact_sign"])
            self.owner.back_off_dist = session["saved_back_off"]
            self.workflows.in_home, self.workflows.home_axis = False, None
        if errors:
            raise self._error("; ".join(errors))

    def command(self, gcmd):
        w = self.workflows
        requested = str(gcmd.get("AXIS", "ALL")).strip().lower()
        if requested not in ("all", "xy", "x", "y"):
            raise gcmd.error("AXIS must be X, Y, XY or ALL (default both)")
        axes = ("x", "y") if requested in ("all", "xy") else (requested,)
        save = bool(gcmd.get_int("SAVE_CONFIG", 0, minval=0, maxval=1))
        margin = gcmd.get_float("SEARCH_MARGIN", DEFAULT_SEARCH_MARGIN_MM, minval=0.0)
        if not math.isfinite(margin):
            raise gcmd.error("SEARCH_MARGIN must be finite")
        if gcmd.get("BACKOFF", None) is not None:
            raise gcmd.error("center calibration has no BACKOFF parameter or separate retract moves")
        w._require_centered(gcmd)
        if w.session is not None:
            raise gcmd.error("A workflow is already active; use ACCEL_HOMING_CANCEL first")
        w._preflight(axes)
        w._require_unhomed_xy()
        plans = self._plan(axes, margin)
        keys = self._save_keys(axes)
        snapshot = self._save_snapshot(keys) if save else None
        w._begin(gcmd, "center", axes=axes)
        session = w.session
        session.update(state="running", axes=axes, save_config=save,
                       center_generation=self.owner.operation_generation,
                       center_preflight_generation=w.preflight_generation,
                       center_original_motion=tuple(self.owner.toolhead.get_max_velocity()),
                       center_cleanup_errors=[],
                       center_impact_sign=dict(self.owner.impact_sign), center_touched=set(),
                       center_completed={axis: 0 for axis in axes}, phase="preflight")
        self.owner.back_off_dist = 0.0
        self.owner.sensor_fault = None
        results, operation_error, cleanup_error = {}, None, None
        result = {"result": "aborted", "mode": "center", "run_id": session["run_id"],
                  "axes": list(axes), "save_requested": save, "save_config": False,
                  "traverses_per_axis": TRAVERSES, "traverses_per_direction": TRAVERSES // 2,
                  "initial_positioning_included": False, "physical_position_verified": False,
                  "coordinate_convention": "detected_negative_trigger_anchored_at_configured_position_min",
                  "measurement_basis": "accelerometer_triggered_commanded_step_displacement",
                  "span_estimator": "commanded_stationary_halt_to_next_trigger_no_overshoot_subtraction",
                  "separate_backoff_moves": 0,
                  "physical_uncertainty": "trigger latency, belt loading, skipped steps and post-trigger "
                                          "commanded overshoot are not measured physical displacement",
                  "persistence": "report_only"}
        try:
            for axis in axes:
                session["axis"] = axis
                plan = plans[axis]
                self._check(session)
                w._info("%s: ten alternating one-way traverses, initial positioning excluded; "
                        "configured span %.6f mm + explicit SEARCH_MARGIN %.6f mm = %.6f mm "
                        "per-move search budget; no separate backoff/retract; speed %.6f mm/s, metric %s. "
                        "Step counts estimate travel assuming no slip or skipped steps" % (
                            axis.upper(), plan["span"], margin, plan["budget"],
                            plan["speed"], plan["metric"]))
                results[axis] = self._axis(session, plan)
            self._check(session)
            edits = self._edits(session, results)
        except Exception as exc:
            operation_error = exc
        try:
            self._cleanup(session, plans)
            if operation_error is None:
                self._check(session)
        except Exception as exc:
            cleanup_error = exc
        finally:
            w.session = None
            w.reactor.update_timer(w.timer, w.reactor.NEVER)
        if operation_error is not None or cleanup_error is not None:
            ending = session.get("center_ending", {})
            result.update(result=ending.get("result", "aborted"),
                          reason=str(operation_error or cleanup_error),
                          failed_axis=session.get("axis"), phase=session.get("phase"),
                          completed_traverses=dict(session["center_completed"]),
                          axis_results={}, proposed_config={})
            if cleanup_error is not None:
                result["cleanup_error"] = str(cleanup_error)
            w.last_result = result
            raise operation_error or cleanup_error
        result.update(result="center_report", axis_results=results,
                      proposed_config={"stepper_" + axis: {key: value for key, value in item["proposed"].items()
                                                          if key != "midpoint"}
                                       for axis, item in results.items()},
                      model_metadata_rebase_proposed=[option[:-9] for section, option, value in edits
                                                     if section == w.model_section])
        w.last_result = result
        try:
            for axis, item in results.items():
                proposed, stats = item["proposed"], item["statistics"]
                w._info("%s: estimated travel %.6f mm, spread %.6f mm, standard deviation %.6f mm; "
                        "direction means + %.6f / - %.6f mm. Proposal [stepper_%s]: "
                        "position_min %.6f (preserved), position_max %.6f, position_endstop %.6f; "
                        "midpoint %.6f. No center move was requested; measured axes require normal homing" % (
                            axis.upper(), stats["mean_mm"], stats["spread_mm"], stats["standard_deviation_mm"],
                            item["directions"]["+"]["mean_mm"], item["directions"]["-"]["mean_mm"], axis,
                            proposed["position_min"], proposed["position_max"], proposed["position_endstop"],
                            proposed["midpoint"]))
            if save:
                self._save(snapshot, keys, edits, result, session)
            else:
                w._info("Report only: no config settings staged or saved. SAVE_CONFIG=1 reruns the measurements "
                        "and saves only after all selected axes and cleanup succeed")
        except Exception as exc:
            if result["result"] != "save_failed":
                result.update(result="save_failed" if save else "report_failed", save_config=False,
                              reason=str(exc))
            else:
                result["reason"] = str(exc)
            raise
