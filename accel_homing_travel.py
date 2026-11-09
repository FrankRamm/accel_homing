# Guided travel/origin calibration for accelerometer homing. GPLv3.
"""Operator-established origin in a measured, temporary commanded-step frame.

No encoder is available: trigger latency, belt loading and skipped steps remain
physical uncertainties. Nothing in this workflow certifies physical position.
"""
import copy
import datetime
import json
import math

if __package__:
    from .accel_homing_center import AXES, CenterCalibration
else:  # Standalone offline analysis; do not mask package dependency errors.
    from accel_homing_center import AXES, CenterCalibration


class TravelCalibration(CenterCalibration):
    def __init__(self, workflows):
        super().__init__(workflows)
        self.watched_enable_lines = set()
        p = workflows.printer
        p.register_event_handler("stepper_enable:motor_off", self._motor_off)
        p.register_event_handler("homing:home_rails_begin", self._external_home)
        p.register_event_handler("gcode:command_error", self._command_error)

    def _error(self, message):
        return self.workflows._error("travel calibration: " + message)

    def _session(self):
        session = self.workflows.session
        return session if session is not None and session["mode"] == "travel" else None

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
                self.owner.homing_accel, self.owner.back_off_dist,
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
        super()._check(session)
        if self.workflows.preflight_generation != session["center_preflight_generation"]:
            raise self._error("workflow context changed")

    def _check_complete(self, session):
        super()._check_complete(session)
        if session.get("center_interrupted"):
            raise self._error(session["center_interrupted"])

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

    def _prompt(self, session, title, lines, buttons):
        w = self.workflows
        for line in lines:
            w._info(line)
        for label, command in buttons:
            w._info("%s: %s" % (label, command))
        if not session["prompt"]:
            return
        raw = w.gcode.respond_raw
        raw("// action:prompt_end")
        raw("// action:prompt_begin " + title)
        for line in lines:
            raw("// action:prompt_text " + line)
        for label, command in buttons:
            raw("// action:prompt_button %s|%s|primary" % (label, command))
        raw("// action:prompt_footer_button Cancel|ACCEL_HOMING_CANCEL|error")
        raw("// action:prompt_show")

    def _show(self, session):
        stage, axis = session["stage"], session["axis"]
        base = "ACCEL_HOMING_TRAVEL ACTION="
        if stage == "select":
            self._prompt(session, "Travel calibration: choose axis", [
                "Before measuring: manually place BOTH unhomed XY axes approximately midway, "
                "with nozzle/Z clear and motion paths unobstructed. Exact midpoint is unnecessary.",
                "After measurement keep motors enabled; use only wizard jogs. No Z move is requested."],
                [("Select X", base + "SELECT AXIS=X"), ("Select Y", base + "SELECT AXIS=Y")])
        elif stage == "prepare":
            plan = session["plans"][axis]
            self._prompt(session, "Travel calibration: " + axis.upper(), [
                "%s: search negative then positive, at configured homing speed %.6g mm/s, "
                "threshold %.6g g. Each search is bounded to %.6g mm; missed impacts can still hit the stop." %
                (axis.upper(), plan["speed"], self.owner.threshold_g[axis], plan["budget"]),
                "Confirm approximate center, physical clearance and supervised operation. "
                "Keep emergency stop accessible; the accelerometer cannot detect an already pressed stop."],
                [("Confirm clear center; measure", base + "MEASURE CENTERED=1 CLEAR=1")])
        elif stage == "origin":
            item = session["measurement"]
            jogs = [("%+.1f mm" % distance, base + "JOG DISTANCE=%s" % distance)
                    for distance in (-10, -1, -.1, .1, 1, 10)
                    if 1.0 <= item["coordinate"] + distance <= item["span"] - 1.0]
            if 1.0 <= item["coordinate"] <= item["span"] - 1.0:
                jogs.append(("Accept bed zero", base + "ORIGIN CONFIRM=1"))
            self._prompt(session, "Align nozzle: " + axis.upper() + " bed zero", [
                "Estimated trigger span %.6f mm. Current temporary coordinate %.6f mm. "
                "Keep motors enabled; never push the carriage by hand." % (item["span"], item["coordinate"]),
                "Jog this axis until the nozzle is at your chosen bed-zero line; then accept origin. "
                "Only the selected axis moves, at %.6g mm/s." % session["jog_speed"]],
                jogs)
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
            self._prompt(session, "Review travel coordinates", lines, buttons)

    def _configuration_state(self):
        config = self.workflows.printer.lookup_object("configfile")
        try:
            filename = self.workflows.printer.get_start_args()["config_file"]
            source = config._read_config_file(filename)
            regular, _ = config._find_autosave_data(source)
            resolved = config._build_config_wrapper(regular, filename).fileconfig
            return (source, self._parser_values(resolved), self._parser_values(config.autosave.fileconfig),
                    copy.deepcopy(config.status_save_pending), config.save_config_pending)
        except (AttributeError, KeyError):
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
        w._begin(gcmd, "travel", axes=[])
        session = w.session
        session.update(axis=None, axes=[], plans={}, results={}, stage="select", prompt=prompt,
                       jog_speed=speed, search_margin=margin, configuration=configuration,
                       center_generation=self.owner.operation_generation,
                       center_preflight_generation=w.preflight_generation,
                       center_original_motion=tuple(self.owner.toolhead.get_max_velocity()),
                       center_cleanup_errors=[], center_impact_sign=dict(self.owner.impact_sign),
                       center_touched=set(), center_completed={}, phase="prepare", busy=False)
        self.owner.back_off_dist = 0.0
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

    def _save_keys(self, axes):
        keys = super()._save_keys(axes)
        return [("stepper_" + axis, "position_min") for axis in axes] + keys

    def _travel_edits(self, session):
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
                metadata["travel_coordinate_rebase"] = {
                    "run_id": session["run_id"], "previous_geometry": previous,
                    "created_utc": datetime.datetime.now(datetime.timezone.utc).isoformat(),
                    "origin_in_measurement_frame_mm": result["origin_in_measurement_frame_mm"],
                    "trigger_span_mm": result["trigger_span_mm"],
                    "threshold_calibration_rerun": False, "physical_position_verified": False}
                edits.append((w.model_section, axis + "_metadata",
                              json.dumps(metadata, separators=(",", ":"), allow_nan=False)))
        return edits

    def _result(self, session, result="travel_report", reason=None):
        complete = result == "travel_report"
        return {"mode": "travel", "run_id": session["run_id"], "result": result, "reason": reason,
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
        session["center_interrupted"] = result.get("reason", "workflow interrupted")
        session["center_ending"] = dict(result)
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
            self.interrupt({"result": "cancelled", "reason": "travel wizard wait expired"})
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
            raise self._error("start the wizard with ACCEL_HOMING_TRAVEL")
        if session["busy"]:
            raise self._error("wizard operation is still running")
        try:
            self._continue(session)
            session["busy"] = True
            session["state"] = "running"
            self.workflows.reactor.update_timer(self.workflows.timer, self.workflows.reactor.NEVER)
            if action == "STATUS":
                self._wait(session, session["stage"])
            elif action == "SELECT" and session["stage"] == "select":
                self._select(session, str(gcmd.get("AXIS", "")).strip().lower())
            elif action == "MEASURE":
                self._measure_axis(session, gcmd)
            elif action == "JOG" and session["stage"] == "origin":
                distance = gcmd.get_float("DISTANCE", minval=-10.0, maxval=10.0)
                if not math.isfinite(distance) or distance == 0:
                    raise self._error("DISTANCE must be finite, nonzero and at most 10 mm")
                self._jog_to(session, session["measurement"]["coordinate"] + distance)
                self._wait(session, "origin")
            elif action == "ORIGIN":
                self._origin(session, gcmd)
            elif action == "NEXT" and session["stage"] == "review":
                other = "y" if session["axis"] == "x" else "x"
                if other in session["results"]:
                    raise self._error("both axes are already accepted; review and SAVE or DONE")
                midpoint = session["measurement"]["span"] * 0.5
                while abs(session["measurement"]["coordinate"] - midpoint) > 0.000001:
                    current = session["measurement"]["coordinate"]
                    target = current + max(-10.0, min(10.0, midpoint - current))
                    self._jog_to(session, target)
                self._select(session, other)
            elif action in ("SAVE", "DONE") and session["stage"] == "review":
                result = self._result(session)
                if action == "SAVE":
                    if gcmd.get_int("CONFIRM", 0, minval=0, maxval=1) != 1:
                        raise self._error("SAVE CONFIRM=1 is required after reviewing coordinates")
                    keys = self._save_keys(session["results"])
                    snapshot = self._save_snapshot(keys)
                    if session["configuration"] is None or self._configuration_state() != session["configuration"]:
                        raise self._error("configuration or pending settings changed since START; restart wizard")
                    edits = self._travel_edits(session)
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
            ending = session.get("center_ending", {})
            result = self._result(session, ending.get("result", "aborted"), str(exc))
            if self.workflows.session is session:
                self._close(session, result)
            elif self.workflows.last_result is not None and self.workflows.last_result.get("result") != "save_failed":
                self.workflows.last_result = result
            raise
        finally:
            session["busy"] = False
