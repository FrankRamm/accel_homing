# Guided X/Y axis limits calibration for accelerometer homing. GPLv3.
"""Operator-established origin in a measured, temporary commanded-step frame.

No encoder is available: trigger latency, belt loading and skipped steps remain
physical uncertainties. Nothing in this workflow certifies physical position.
"""
import copy
import datetime
import json
import math
import types


AXES = {"x": 0, "y": 1}
MIN_INITIAL_TRAVEL_MM = 20.0
POLARITY_POLICY = "absolute_after_reference_subtraction_v1"


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


class AxisLimitsCalibration:

    def __init__(self, workflows):
        self.workflows, self.owner = workflows, workflows.owner
        self.watched_enable_lines = set()
        p = workflows.printer
        p.register_event_handler("stepper_enable:motor_off", self._motor_off)
        p.register_event_handler("homing:home_rails_begin", self._external_home)
        p.register_event_handler("gcode:command_error", self._command_error)


    def _error(self, message):
        return self.workflows._error("axis limits calibration: " + message)


    def _session(self):
        session = self.workflows.session
        return session if session is not None and session["mode"] == "axis_limits" else None


    def _motor_off(self, *args):
        if self._session() is not None:
            self.interrupt({"result": "aborted", "reason": "motor disabled; measured reference discarded"})


    def _enable_changed(self, print_time, enabled):
        if not enabled:
            self._motor_off()


    def _external_home(self, *args):
        session = self._session()
        if session is not None and not self.workflows.in_home:
            self.interrupt({"result": "aborted", "reason": "external homing invalidated the wizard"})
            raise self._error("wizard cancelled by external homing; restart it after manual preparation")


    def _command_error(self, *args):
        if self._session() is not None:
            self.interrupt({"result": "aborted", "reason": "G-code command failed; reference discarded"})


    def _watch_motors(self):
        manager = self.workflows.printer.lookup_object("stepper_enable")
        for stepper in self.owner.toolhead.get_kinematics().get_steppers():
            line = manager.lookup_enable(stepper.get_name())
            if not callable(getattr(line, "register_state_callback", None)):
                raise self._error("firmware lacks individual kinematic motor-disable notifications")
            if id(line) not in self.watched_enable_lines:
                line.register_state_callback(self._enable_changed)
                self.watched_enable_lines.add(id(line))


    def _settings(self):
        return (dict(self.owner.threshold_g), dict(self.owner.impact_sign),
                self.owner.homing_accel, tuple(sorted(self.owner.back_off_dist.items())),
                tuple(self.owner.toolhead.get_max_velocity()),
                {axis: self.workflows._model_context(axis) for axis in self.owner.rails_by_axis
                 if axis in AXES},
                None if self.workflows.model is None else
                (self.workflows.model.thresholds, self.workflows.model.metadata))


    def _position_reference(self):
        toolhead = self.owner.toolhead
        manager = self.workflows.printer.lookup_object("stepper_enable")
        return (tuple(toolhead.get_position()), self._positions(),
                tuple(tuple(bounds) for bounds in toolhead.get_kinematics().limits),
                tuple((s.get_name(), manager.lookup_enable(s.get_name()).is_motor_enabled())
                      for s in toolhead.get_kinematics().get_steppers()))


    def _check(self, session):
        self.workflows._check_running(session)
        if session.get("limits_interrupted"):
            raise self._error(session["limits_interrupted"])
        if self.owner.operation_generation != session["limits_generation"]:
            raise self._error("motors, connection or operation changed")
        if (self.owner.threshold_g != session["saved_thresholds"]
                or self.owner.impact_sign != session["limits_impact_sign"]):
            raise self._error("homing thresholds or impact settings changed")
        if getattr(self.owner, "sensor_fault", None) is not None:
            raise self._error("accelerometer fault: %s" % self.owner.sensor_fault)
        if self.owner.toolhead.get_max_velocity()[0] != session["limits_original_motion"][0]:
            raise self._error("live velocity limit changed during measurement")
        if self.workflows.preflight_generation != session["limits_preflight_generation"]:
            raise self._error("workflow context changed")


    def _check_complete(self, session):
        w = self.workflows
        if (w.session is not None or self.owner.operation_generation != session["limits_generation"]
                or w.preflight_generation != session["limits_preflight_generation"]
                or (hasattr(w.printer, "is_shutdown") and w.printer.is_shutdown())
                or w._printing() or self.owner.homing_active or self.owner.attempt is not None
                or self.owner.threshold_g != session["saved_thresholds"]
                or self.owner.impact_sign != session["limits_impact_sign"]
                or self.owner.back_off_dist != session["saved_back_off"]
                or tuple(self.owner.toolhead.get_max_velocity()) != session["limits_original_motion"]):
            raise self._error("calibration context changed after cleanup; no automatic save")
        if session.get("limits_interrupted"):
            raise self._error(session["limits_interrupted"])


    def _continue(self, session):
        self._check(session)
        if self.workflows.reactor.monotonic() >= session["expires"]:
            raise self._error("wizard wait expired")
        self.workflows._preflight([session["axis"]] if session["axis"] else [], continuation=True)
        if self._settings() != session["settings"]:
            raise self._error("homing, sensor, geometry or live motion settings changed")
        if self._position_reference() != session["reference"]:
            raise self._error("position, stepper counts, homing state or motor enables changed outside the wizard")


    def _wait(self, session, stage):
        self._check(session)
        session.update(state="waiting", stage=stage,
                       expires=self.workflows.reactor.monotonic() + self.workflows.confirm_timeout)
        session["settings"] = copy.deepcopy(self._settings())
        session["reference"] = self._position_reference()
        self.workflows.reactor.update_timer(self.workflows.timer, session["expires"])
        self._show(session)


    def _prompt(self, session, title, lines, buttons, groups=(), footer=None):
        w = self.workflows
        if footer is None:
            footer = [("Abort", "ACCEL_HOMING_CANCEL", "error")]
        groups = [group for group in groups if group]
        for line in lines:
            w._info(line)
        for label, command in list(buttons) + [b for group in groups for b in group]:
            w._info("%s: %s" % (label, command))
        for label, command, _style in footer:
            w._info("%s: %s" % (label, command))
        if not session["prompt"]:
            return
        raw = w.gcode.respond_raw
        # No prompt_end: a new prompt_begin replaces an open dialog in place.
        raw("// action:prompt_begin " + title)
        for line in lines:
            raw("// action:prompt_text " + line)
        for label, command in buttons:
            raw("// action:prompt_button %s|%s|primary" % (label, command))
        for group in groups:
            raw("// action:prompt_button_group_start")
            for label, command in group:
                raw("// action:prompt_button %s|%s|primary" % (label, command))
            raw("// action:prompt_button_group_end")
        for label, command, style in footer:
            raw("// action:prompt_footer_button %s|%s|%s" % (label, command, style))
        raw("// action:prompt_show")


    def _show(self, session):
        stage, axis = session["stage"], session["axis"]
        base = "ACCEL_HOMING_CALIBRATE_LIMITS ACTION="
        if stage == "select":
            self._prompt(session, "Axis limits calibration: choose axis", [
                "Before measuring: manually place BOTH unhomed XY axes approximately midway, "
                "with nozzle/Z clear and motion paths unobstructed. Exact midpoint is unnecessary.",
                "After measurement keep motors enabled; use only wizard jogs. No Z move is requested."],
                [("Select X", base + "SELECT AXIS=X"), ("Select Y", base + "SELECT AXIS=Y")])
        elif stage == "prepare":
            plan = session["plans"][axis]
            self._prompt(session, "Axis limits calibration: " + axis.upper(), [
                "%s: search negative then positive, at configured homing speed %.6g mm/s, "
                "threshold %.6g g. Each search is bounded to %.6g mm; missed impacts can still hit the stop." %
                (axis.upper(), plan["speed"], self.owner.threshold_g[axis], plan["budget"]),
                "Confirm approximate center, physical clearance and supervised operation. "
                "Keep emergency stop accessible; the accelerometer cannot detect an already pressed stop."],
                [("Confirm clear center; measure", base + "MEASURE CENTERED=1 CLEAR=1")])
        elif stage == "origin":
            item = session["measurement"]
            coordinate, span = item["coordinate"], item["span"]
            groups = [[("%+g" % distance, base + "JOG DISTANCE=%s" % distance)
                       for distance in steps if 1.0 <= coordinate + distance <= span - 1.0]
                      for steps in ((-100, -10, -1, -0.1), (0.1, 1, 10, 100))]
            footer = [("Abort", "ACCEL_HOMING_CANCEL", "error")]
            if 1.0 <= coordinate <= span - 1.0:
                footer.insert(0, ("Accept bed zero", base + "ORIGIN CONFIRM=1", "primary"))
            self._prompt(session, "Align nozzle: " + axis.upper() + " bed zero", [
                "%s %.2f mm (measured span 0 -> %.2f mm)" % (axis.upper(), coordinate, span),
                "Keep motors enabled; never push the carriage by hand. "
                "Jog this axis until the nozzle is at your chosen bed-zero line; then accept origin. "
                "Only the selected axis moves, at %.6g mm/s." % session["jog_speed"]],
                [], groups, footer)
        else:
            lines = []
            for name, result in session["results"].items():
                p = result["proposed"]
                lines.append("[stepper_%s] position_min: %.6f; position_max: %.6f; "
                             "position_endstop: %.6f" % (name, p["position_min"],
                                                        p["position_max"], p["position_endstop"]))
            lines.append("Review these estimated coordinates. SAVE writes them together and restarts Klipper. "
                         "Bed size, mesh/probe areas and parking macros are not updated.")
            buttons = [("Save and restart", base + "SAVE CONFIRM=1"), ("Finish without saving", base + "DONE")]
            other = "y" if axis == "x" else "x"
            if other not in session["results"]:
                buttons.insert(0, ("Center %s; select %s" % (axis.upper(), other.upper()), base + "NEXT"))
            self._prompt(session, "Review axis limits", lines, buttons)


    @staticmethod
    def _legacy_layout(configfile):
        """Retained PrinterConfig layout: pending state and readers on configfile."""
        return hasattr(configfile, "status_save_pending")


    def _newer_files(self, configfile, filename):
        """Read the newer layout through the helper's native reader; read-only."""
        module = self.workflows.config_module
        reader = module._NativeReader(configfile)
        root, files, _ = module.CalibrationConfigSave(
            self.workflows.printer, "accel_homing")._configuration(reader, filename)
        return reader, root, files


    def _configuration_state(self):
        w = self.workflows
        config = w.printer.lookup_object("configfile")
        try:
            filename = w.printer.get_start_args()["config_file"]
            if not self._legacy_layout(config):
                _reader, _root, files = self._newer_files(config, filename)
                state = config.autosave
                return (tuple(sorted((path, item["data"]) for path, item in files.items())), [],
                        self._parser_values(state.fileconfig), copy.deepcopy(state.status_save_pending),
                        state.save_config_pending)
            source = config._read_config_file(filename)
            regular, _ = config._find_autosave_data(source)
            resolved = config._build_config_wrapper(regular, filename).fileconfig
            return (source, self._parser_values(resolved), self._parser_values(config.autosave.fileconfig),
                    copy.deepcopy(config.status_save_pending), config.save_config_pending)
        except (AttributeError, KeyError, ValueError, OSError):
            return None


    def _start(self, gcmd):
        w = self.workflows
        if w.session is not None:
            raise self._error("a workflow is active; cancel it first")
        speed = gcmd.get_float("JOG_SPEED", 10.0, minval=1.0, maxval=20.0)
        margin = gcmd.get_float("SEARCH_MARGIN", 20.0, minval=0.0, maxval=100.0)
        if not all(math.isfinite(v) for v in (speed, margin)):
            raise self._error("JOG_SPEED and SEARCH_MARGIN must be finite")
        prompt = bool(gcmd.get_int("PROMPT", 1, minval=0, maxval=1))
        prompt = prompt and callable(getattr(w.gcode, "respond_raw", None))
        # No motion, motor release or forced coordinate assignment at START.
        w._preflight([])
        w._require_unhomed_xy()
        self._watch_motors()
        configuration = self._configuration_state()
        w._begin(gcmd, "axis_limits", axes=[])
        session = w.session
        session.update(axis=None, axes=[], plans={}, results={}, stage="select", prompt=prompt,
                       jog_speed=speed, search_margin=margin, configuration=configuration,
                       limits_generation=self.owner.operation_generation,
                       limits_preflight_generation=w.preflight_generation,
                       limits_original_motion=tuple(self.owner.toolhead.get_max_velocity()),
                       limits_cleanup_errors=[], limits_impact_sign=dict(self.owner.impact_sign),
                       limits_touched=set(), limits_completed={}, phase="prepare", busy=False)
        self.owner.back_off_dist = dict.fromkeys(self.owner.back_off_dist, 0.0)
        self.owner.sensor_fault = None
        self._wait(session, "select")


    def _select(self, session, axis):
        if axis not in AXES:
            raise self._error("AXIS must be X or Y")
        if axis in session["results"]:
            raise self._error("axis already accepted; cancel and start again to repeat it")
        self.workflows._preflight([axis], continuation=True)
        self.workflows._require_unhomed_xy()
        plan = self._plan([axis], session["search_margin"])[axis]
        if plan["budget"] > 1000.0:
            raise self._error("configured span plus SEARCH_MARGIN exceeds the 1000 mm search bound")
        session["plans"][axis] = plan
        session["axis"] = axis
        self._wait(session, "prepare")


    def _measure_axis(self, session, gcmd):
        if session["stage"] != "prepare":
            raise self._error("select an axis before measuring")
        self.workflows._require_centered(gcmd)
        if gcmd.get_int("CLEAR", 0, minval=0, maxval=1) != 1:
            raise self._error("CLEAR=1 must confirm physical nozzle and path clearance")
        axis, plan = session["axis"], session["plans"][session["axis"]]
        negative = self._measure(session, plan, -1, plan["budget"], "negative_search")
        if abs(negative["trigger_travel_mm_signed"]) <= 20.0:
            raise self._error("negative impact after <=20 mm; inspect and manually recenter before restarting")
        self.owner.toolhead.dwell(self.workflows.calibration_settle_time)
        self.owner.toolhead.wait_moves()
        self._check(session)
        positive = self._measure(session, plan, 1, plan["budget"], "positive_search")
        # Common commanded-step frame: negative TRIGGER is zero. The next
        # search starts at negative HALT, not at that trigger coordinate.
        span = negative["overshoot_mm_signed"] + positive["trigger_travel_mm_signed"]
        if not math.isfinite(span) or not plan["minimum_span"] <= span <= plan["budget"]:
            raise self._error("estimated trigger span is outside the declared plausible range")
        coordinate = span + positive["overshoot_mm_signed"]
        session["measurement"] = {"negative": negative, "positive": positive, "span": span,
                                  "coordinate": coordinate, "origin": None}
        position = self.owner.toolhead.get_position()
        position[AXES[axis]] = coordinate
        self.owner.toolhead.set_position(position)
        self.owner.toolhead.get_kinematics().limits[AXES[axis]] = (1.0, -1.0)
        self._wait(session, "origin")


    def _jog_to(self, session, target):
        self._check(session)
        axis, item = session["axis"], session["measurement"]
        # Stay at least one millimetre inside estimated trigger endpoints. The
        # first move after positive detection must move away from that stop.
        if not math.isfinite(target) or not 1.0 <= target <= item["span"] - 1.0:
            raise self._error("jog target must stay at least 1 mm inside estimated trigger endpoints")
        toolhead, kin = self.owner.toolhead, self.owner.toolhead.get_kinematics()
        index = AXES[axis]
        previous_limits = kin.limits[index]
        before = self._positions()
        start = item["coordinate"]
        original_accel = toolhead.get_max_velocity()[1]
        try:
            self.owner._set_accel(min(original_accel, 500.0))
            self._check(session)
            kin.limits[index] = (min(0.0, start), max(item["span"], start))
            coords = [None, None, None]
            coords[index] = target
            toolhead.manual_move(coords, session["jog_speed"])
            toolhead.wait_moves()
            self._check(session)
            delta, tolerance = self._delta(before, self._positions(), axis)
            if abs(delta - (target - start)) > tolerance:
                raise self._error("completed jog stepper displacement disagrees with target")
            if abs(toolhead.get_position()[index] - target) > tolerance:
                raise self._error("completed jog coordinate disagrees with target")
            item["coordinate"] = target
        finally:
            kin.limits[index] = previous_limits
            self._restore_live_accel(original_accel)


    def _origin(self, session, gcmd):
        if session["stage"] != "origin" or gcmd.get_int("CONFIRM", 0, minval=0, maxval=1) != 1:
            raise self._error("align the nozzle using JOG, then ORIGIN CONFIRM=1")
        axis, item = session["axis"], session["measurement"]
        origin, span = item["coordinate"], item["span"]
        if not 1.0 <= origin <= span - 1.0:
            raise self._error("origin must be at least 1 mm inside estimated trigger endpoints")
        plan = session["plans"][axis]
        proposed = {"position_min": -origin, "position_max": span - origin}
        proposed["position_endstop"] = proposed["position_max"] if plan["normal_direction"] > 0 else proposed["position_min"]
        proposed["midpoint"] = (proposed["position_min"] + proposed["position_max"]) / 2.0
        if not all(math.isfinite(v) for v in proposed.values()) or not proposed["position_min"] < 0 < proposed["position_max"]:
            raise self._error("invalid origin/soft-limit proposal")
        item["origin"] = origin
        session["results"][axis] = {
            "proposed": proposed, "origin_in_measurement_frame_mm": origin,
            "negative": copy.deepcopy(item["negative"]), "positive": copy.deepcopy(item["positive"]),
            "trigger_span_mm": span,
            "halt_to_positive_trigger_mm": item["positive"]["trigger_travel_mm_signed"],
            "configured": {"position_min": plan["minimum"], "position_max": plan["maximum"],
                           "position_endstop": plan["endstop"], "homing_positive_dir": plan["normal_direction"] > 0},
            "homing_speed_mm_s": plan["speed"], "threshold_g": session["saved_thresholds"][axis],
            "physical_position_verified": False}
        self._wait(session, "review")


    def _restore_live_accel(self, accel):
        if self.owner.toolhead.get_max_velocity()[1] != accel:
            self.owner._set_accel(accel)
        if self.owner.toolhead.get_max_velocity()[1] != accel:
            raise self._error("could not restore the original live acceleration limit")


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
            current, current_context = self.workflows._beta_current_context(axis, purpose="Axis limits calibration")
            rail_accel = getattr(info, "accel", None)
            if rail_accel is not None and (not math.isfinite(rail_accel) or rail_accel <= 0
                    or not callable(getattr(state, "_set_homing_accel", None))):
                raise self._error("configured rail homing acceleration has no supported native helper")
            # Mirror the detector's own record: a saved or configured axis
            # mapping reports "mapped_axis", which the single-impact check needs.
            if self.workflows.detection_axis_mapping(axis) is not None:
                metric = "mapped_axis"
            else:
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
        session["limits_touched"].add(axis)
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
                    # Ordinary homing accepts a selected_axis impact only with
                    # proof of motion; vector and mapped metrics need none.
                    or (record["impact_metric"] == "selected_axis"
                        and not (record.get("moving") or record.get("motion_seen")))
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
                            ("live acceleration", lambda: self._restore_live_accel(session["limits_original_motion"][1]))))
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
                session["limits_cleanup_errors"].extend(errors)
                if operation_error is None:
                    raise self._error("; ".join(errors))


    def _active_model(self, axis):
        w = self.workflows
        # SET_ACCEL_HOMING's temporary override disappears at restart. Rebase
        # any selected saved model that will supply its threshold afterwards.
        return w.model is not None and axis in w.model.thresholds and axis not in w.manual_axes


    def _save_snapshot(self, keys):
        """Read-only native include check and snapshot; never write config files."""
        w = self.workflows
        configfile = w.printer.lookup_object("configfile")
        legacy = self._legacy_layout(configfile)
        # Newer hosts keep pending state and autosave helpers on configfile.autosave.
        state = configfile if legacy else getattr(configfile, "autosave", None)
        legacy_required = ("autosave", "_read_config_file", "_find_autosave_data",
                           "_strip_duplicates", "_build_config_wrapper")
        if (not hasattr(getattr(configfile, "autosave", None), "fileconfig")
                or not hasattr(state, "status_save_pending") or not hasattr(state, "save_config_pending")
                or (legacy and any(not hasattr(configfile, name) for name in legacy_required))):
            raise self._error("SAVE_CONFIG=1 needs supported native autosave snapshot/include APIs; use report-only")
        if state.save_config_pending or state.status_save_pending:
            raise self._error("unrelated SAVE_CONFIG changes are pending; save or discard them separately first")
        parser = configfile.autosave.fileconfig
        filename = w.printer.get_start_args()["config_file"]
        if legacy:
            data = configfile._read_config_file(filename)
            regular, _ = configfile._find_autosave_data(data)
            proposed = copy.deepcopy(parser)
            for section, option in keys:
                if not proposed.has_section(section):
                    proposed.add_section(section)
                proposed.set(section, option, "axis_limits_include_preflight")
            stripped = configfile._strip_duplicates(regular, types.SimpleNamespace(fileconfig=proposed))
            included = configfile._build_config_wrapper(stripped, filename).fileconfig
            conflicts = ["[%s] %s" % key for key in keys if included.has_option(*key)]
        else:
            module = w.config_module
            targets = {key: {"value": "axis_limits_include_preflight"} for key in keys}
            try:
                reader, root, files = self._newer_files(configfile, filename)
                reader.check_root_targets(root["data"], targets)
            except (ValueError, OSError) as exc:
                raise self._error("SAVE_CONFIG=1 cannot read the configuration safely: %s; use report-only" % exc)
            data = tuple(sorted((path, item["data"]) for path, item in files.items()))
            conflicts = ["[%s] %s" % key for key in keys
                         if any(path != root["path"] and module._comment_selected(
                                    item["data"], {key: targets[key]}) != item["data"]
                                for path, item in files.items())]
        if conflicts:
            raise self._error("SAVE_CONFIG=1 cannot safely replace included options: %s. "
                              "Move these options into the main printer.cfg first, or use report-only" % ", ".join(conflicts))
        native_save = getattr(type(configfile), "cmd_SAVE_CONFIG", None)
        if not callable(native_save) and not legacy:
            native_save = getattr(type(configfile.autosave), "cmd_SAVE_CONFIG", None)
            owner = configfile.autosave
        else:
            owner = configfile
        if not callable(native_save):
            raise self._error("native SAVE_CONFIG implementation is unavailable before motion")
        return {"configfile": configfile, "state": state, "parser": copy.deepcopy(parser),
                "pending": copy.deepcopy(state.status_save_pending),
                "flag": state.save_config_pending, "source": data,
                "native_save": types.MethodType(native_save, owner)}


    @staticmethod
    def _parser_values(parser):
        return [(section, list(parser.items(section, raw=True))) for section in parser.sections()]


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
            snapshot["state"].status_save_pending = snapshot["pending"]
            snapshot["state"].save_config_pending = snapshot["flag"]
            result.update(result="save_failed", save_config=False,
                          persistence="native_save_failed_disk_state_unconfirmed")
            raise
        result.update(save_config=True, persistence="native_save_requested",
                      model_metadata_rebased=list(result["model_metadata_rebase_proposed"]))


    def _cleanup(self, session, plans):
        errors = list(session["limits_cleanup_errors"])
        try:
            self.owner._end_session()
        except Exception as exc:
            errors.append("sensor/acceleration cleanup: %s" % exc)
        try:
            self.owner._restore_accel()
        except Exception as exc:
            errors.append("acceleration restoration: %s" % exc)
        try:
            self._restore_live_accel(session["limits_original_motion"][1])
        except Exception as exc:
            errors.append("original live acceleration restoration: %s" % exc)
        try:
            self.owner.toolhead.wait_moves()
        except Exception as exc:
            errors.append("motion completion: %s" % exc)
        finally:
            kin = self.owner.toolhead.get_kinematics()
            for axis in session["limits_touched"]:
                self.owner.rails_by_axis[axis] = plans[axis]["rail"]
                kin.limits[AXES[axis]] = (1.0, -1.0)
            self.owner.threshold_g.update(session["saved_thresholds"])
            self.owner.impact_sign.update(session["limits_impact_sign"])
            self.owner.back_off_dist = dict(session["saved_back_off"])
            self.workflows.in_home, self.workflows.home_axis = False, None
        if errors:
            raise self._error("; ".join(errors))


    def _save_keys(self, axes):
        keys = [("stepper_" + axis, name) for axis in axes
                for name in ("position_min", "position_max", "position_endstop")]
        keys.extend((self.workflows.model_section, axis + "_metadata") for axis in axes if self._active_model(axis))
        return keys

    def _limits_edits(self, session):
        edits, w = [], self.workflows
        for axis, result in session["results"].items():
            p = result["proposed"]
            for key in ("position_min", "position_max", "position_endstop"):
                edits.append(("stepper_" + axis, key, "%.17g" % p[key]))
            if self._active_model(axis):
                metadata = copy.deepcopy(w.model.metadata[axis])
                previous = copy.deepcopy(metadata["geometry"])
                metadata.setdefault("calibration_geometry", previous)
                metadata["geometry"] = {"minimum": p["position_min"], "maximum": p["position_max"],
                                        "endstop": p["position_endstop"], "positive_dir": result["configured"]["homing_positive_dir"],
                                        "center": p["midpoint"]}
                metadata["axis_limits_rebase"] = {
                    "run_id": session["run_id"], "previous_geometry": previous,
                    "created_utc": datetime.datetime.now(datetime.timezone.utc).isoformat(),
                    "origin_in_measurement_frame_mm": result["origin_in_measurement_frame_mm"],
                    "trigger_span_mm": result["trigger_span_mm"],
                    "threshold_calibration_rerun": False, "physical_position_verified": False}
                edits.append((w.model_section, axis + "_metadata",
                              json.dumps(metadata, separators=(",", ":"), allow_nan=False)))
        return edits


    def _result(self, session, result="axis_limits_report", reason=None):
        complete = result == "axis_limits_report"
        return {"mode": "axis_limits", "run_id": session["run_id"], "result": result, "reason": reason,
                "axis_results": copy.deepcopy(session["results"]) if complete else {},
                "proposed_config": {"stepper_" + axis: {k: v for k, v in item["proposed"].items()
                                                         if k != "midpoint"}
                                    for axis, item in session["results"].items()} if complete else {},
                "measurement_basis": "commanded_step_trigger_frame_with_separate_halt_coordinates",
                "physical_position_verified": False,
                "model_metadata_rebase_proposed": [axis for axis in session["results"] if self._active_model(axis)] if complete else [],
                "physical_uncertainty": "impact latency, belt loading, skipped steps and commanded overshoot "
                                        "are not measured physical displacement",
                "save_config": False, "persistence": "report_only"}


    def _close(self, session, result):
        w = self.workflows
        # Cleanup waits may yield to another disable/shutdown callback. Keep
        # those callbacks from recursively entering this same cleanup routine.
        session["busy"] = True
        try:
            self._cleanup(session, session["plans"])
        finally:
            w.session = None
            w.reactor.update_timer(w.timer, w.reactor.NEVER)
            w.last_result = result
            if session["prompt"]:
                w.gcode.respond_raw("// action:prompt_end")


    def interrupt(self, result):
        session = self._session()
        if session is None:
            return
        session["limits_interrupted"] = result.get("reason", "workflow interrupted")
        session["limits_ending"] = dict(result)
        if session["busy"]:
            # Native homing must unwind before its rail/current/limits cleanup.
            self.owner._sensor_fault("cancelled")
            self.owner._end_session()
            return
        self._close(session, self._result(session, result.get("result", "aborted"), result.get("reason")))


    def timeout(self, eventtime):
        session = self._session()
        if session is None or session["state"] != "waiting":
            return self.workflows.reactor.NEVER
        if eventtime >= session["expires"]:
            self.interrupt({"result": "cancelled", "reason": "axis limits wizard wait expired"})
            return self.workflows.reactor.NEVER
        # Also invalidate individual state changes that have no global event.
        try:
            self._continue(session)
        except Exception as exc:
            self.interrupt({"result": "aborted", "reason": str(exc)})
            return self.workflows.reactor.NEVER
        return session["expires"]


    def command(self, gcmd):
        action = str(gcmd.get("ACTION", "START")).strip().upper()
        if action == "START":
            if self._session() is not None:
                action = "STATUS"
            else:
                try:
                    self._start(gcmd)
                except Exception as exc:
                    session = self._session()
                    if session is not None:
                        self._close(session, self._result(session, "aborted", str(exc)))
                    raise
                return
        session = self._session()
        if session is None:
            raise self._error("start the wizard with ACCEL_HOMING_CALIBRATE_LIMITS")
        if session["busy"]:
            raise self._error("wizard operation is still running")
        try:
            self._validate(session, action, gcmd)
        except Exception:
            # A rejected command changes nothing: keep the session, stage and
            # measurements, and show the current dialog again.
            if session["state"] == "waiting":
                self._show(session)
            raise
        report = None
        try:
            self._continue(session)
            session["busy"] = True
            session["state"] = "running"
            self.workflows.reactor.update_timer(self.workflows.timer, self.workflows.reactor.NEVER)
            if action == "STATUS":
                self._wait(session, session["stage"])
            elif action == "SELECT":
                self._select(session, str(gcmd.get("AXIS", "")).strip().lower())
            elif action == "MEASURE":
                self._measure_axis(session, gcmd)
            elif action == "JOG":
                distance = gcmd.get_float("DISTANCE", minval=-100.0, maxval=100.0)
                self._jog_to(session, session["measurement"]["coordinate"] + distance)
                self._wait(session, "origin")
            elif action == "ORIGIN":
                self._origin(session, gcmd)
            elif action == "NEXT":
                other = "y" if session["axis"] == "x" else "x"
                midpoint = session["measurement"]["span"] * 0.5
                while abs(session["measurement"]["coordinate"] - midpoint) > 0.000001:
                    current = session["measurement"]["coordinate"]
                    target = current + max(-10.0, min(10.0, midpoint - current))
                    self._jog_to(session, target)
                self._select(session, other)
            elif action in ("SAVE", "DONE"):
                result = report = self._result(session)
                if action == "SAVE":
                    keys = self._save_keys(session["results"])
                    snapshot = self._save_snapshot(keys)
                    if session["configuration"] is None or self._configuration_state() != session["configuration"]:
                        raise self._error("configuration or pending settings changed since START; restart wizard")
                    edits = self._limits_edits(session)
                self._close(session, result)
                self._check_complete(session)
                if action == "SAVE":
                    if self._configuration_state() != session["configuration"]:
                        raise self._error("configuration changed during cleanup; no save")
                    self._save(snapshot, keys, edits, result, session)
                else:
                    self.workflows._info("Coordinates reported only; no settings staged. XY remain unhomed.")
            else:
                raise self._error("action is invalid for the current wizard stage")
            if self.workflows.session is session:
                self._check(session)
        except Exception as exc:
            ending = session.get("limits_ending", {})
            if report is not None and action == "SAVE":
                # The proposal was complete; a refused or failed save must not
                # lose the measurements. Keep the full report and print it.
                result = report
                result["reason"] = str(exc)
                if result["result"] != "save_failed":
                    result.update(persistence="save_refused",
                                  result=ending.get("result", "axis_limits_report"))
                self._print_report(result)
            else:
                result = self._result(session, ending.get("result", "aborted"), str(exc))
            if self.workflows.session is session:
                self._close(session, result)
            elif report is not None and action == "SAVE":
                self.workflows.last_result = result
            elif self.workflows.last_result is not None and self.workflows.last_result.get("result") != "save_failed":
                self.workflows.last_result = result
            raise
        finally:
            session["busy"] = False


    def _validate(self, session, action, gcmd):
        """Check stage and parameters before anything can change or close."""
        stage = session["stage"]
        if action == "STATUS":
            return
        required = {"SELECT": "select", "MEASURE": "prepare", "JOG": "origin", "ORIGIN": "origin",
                    "NEXT": "review", "SAVE": "review", "DONE": "review"}
        if required.get(action) != stage:
            raise self._error("action is invalid for the current wizard stage")
        if action == "SELECT":
            axis = str(gcmd.get("AXIS", "")).strip().lower()
            if axis not in AXES:
                raise self._error("AXIS must be X or Y")
            if axis in session["results"]:
                raise self._error("axis already accepted; cancel and start again to repeat it")
        elif action == "MEASURE":
            self.workflows._require_centered(gcmd)
            if gcmd.get_int("CLEAR", 0, minval=0, maxval=1) != 1:
                raise self._error("CLEAR=1 must confirm physical nozzle and path clearance")
        elif action == "JOG":
            distance = gcmd.get_float("DISTANCE", minval=-100.0, maxval=100.0)
            if not math.isfinite(distance) or distance == 0:
                raise self._error("DISTANCE must be finite, nonzero and at most 100 mm")
            item = session["measurement"]
            if not 1.0 <= item["coordinate"] + distance <= item["span"] - 1.0:
                raise self._error("jog target must stay at least 1 mm inside estimated trigger endpoints")
        elif action == "ORIGIN":
            if gcmd.get_int("CONFIRM", 0, minval=0, maxval=1) != 1:
                raise self._error("align the nozzle using JOG, then ORIGIN CONFIRM=1")
            item = session["measurement"]
            if not 1.0 <= item["coordinate"] <= item["span"] - 1.0:
                raise self._error("origin must be at least 1 mm inside estimated trigger endpoints")
        elif action == "NEXT":
            if ("y" if session["axis"] == "x" else "x") in session["results"]:
                raise self._error("both axes are already accepted; review and SAVE or DONE")
        elif action == "SAVE":
            if gcmd.get_int("CONFIRM", 0, minval=0, maxval=1) != 1:
                raise self._error("SAVE CONFIRM=1 is required after reviewing coordinates")


    def _print_report(self, result):
        w = self.workflows
        status = "persistence unconfirmed" if result["result"] == "save_failed" else "not saved"
        w._info("Axis limits proposal %s (%s). Measurements are kept; apply by hand if wanted:"
                % (status, result["reason"]))
        for name, p in result["proposed_config"].items():
            w._info("[%s] position_min: %.6f; position_max: %.6f; position_endstop: %.6f"
                    % (name, p["position_min"], p["position_max"], p["position_endstop"]))
