# Supervised automatic notch qualification and persistence. GPLv3.
"""Freeze offline detector proposals before monitored qualification; never tune an armed filter."""
import copy
import gzip
import json
import logging
import math
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import time
import traceback
import uuid

if __name__ == "__main__" and len(sys.argv) == 5 and sys.argv[1] == "--analysis-worker":
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    if Path(sys.argv[4]).is_dir():
        sys.path.insert(0, sys.argv[4])

if __package__:
    from .accel_homing_backends import load_companion_module
else:
    from accel_homing_backends import load_companion_module

MAX_RETAINED_SAMPLES = 500000
MAX_ATTEMPT_RECORDS = 128
MAX_ANALYSIS_INPUT = 64 * 1024 * 1024
MAX_ANALYSIS_OUTPUT = 16 * 1024 * 1024
MAX_DEBUG_RAW_BYTES = 1024 * 1024 * 1024


def _checkpoint(renderer, check_running):
    check_running()
    if renderer.reactor is not None:
        renderer.reactor.pause(renderer.reactor.monotonic() + .001)
    check_running()


def _write_json(stream, value, cooperate, limit=None):
    """Bound synchronous encoding/compression work without materializing JSON."""
    parts, size = [], 0
    encoder = json.JSONEncoder(allow_nan=False, separators=(",", ":"))
    for piece in encoder.iterencode(value):
        parts.append(piece)
        if len(parts) >= 8192:
            block = "".join(parts)
            size += len(block.encode("utf-8"))
            if limit is not None and size > limit:
                raise ValueError("AUTO JSON exceeds its bounded capacity")
            stream.write(block)
            parts.clear()
            cooperate()
    block = "".join(parts)
    size += len(block.encode("utf-8"))
    if limit is not None and size > limit:
        raise ValueError("AUTO JSON exceeds its bounded capacity")
    stream.write(block)
    cooperate()


def run_analysis(payload, renderer, check_running, debug_directory=None):
    """Use exactly the configured plotting interpreter/private dependencies, off the reactor."""
    check_running()
    with tempfile.TemporaryDirectory(prefix="accel_homing_auto_worker_") as temporary:
        folder = Path(temporary)
        source, target = folder / "input.json", folder / "output.json"
        with source.open("w", encoding="utf-8") as stream:
            _write_json(stream, payload, lambda: _checkpoint(renderer, check_running), MAX_ANALYSIS_INPUT)
        with open(folder / "worker.log", "wb") as log:
            process = subprocess.Popen([renderer.python_executable, "-B", "-I", str(Path(__file__).resolve()),
                "--analysis-worker", str(source), str(target), renderer.dependency_directory],
                stdin=subprocess.DEVNULL, stdout=log, stderr=log)
            deadline = time.monotonic() + 90.0
            try:
                while process.poll() is None:
                    check_running()
                    if os.fstat(log.fileno()).st_size > 1024 * 1024 or (target.exists() and target.stat().st_size > MAX_ANALYSIS_OUTPUT):
                        raise ValueError("AUTO analysis exceeded its output/log capacity")
                    if time.monotonic() >= deadline:
                        raise ValueError("AUTO analysis exceeded its 90-second deadline")
                    if renderer.reactor is None:
                        time.sleep(.02)
                    else:
                        renderer.reactor.pause(renderer.reactor.monotonic() + .02)
                check_running()
                if process.returncode:
                    log.flush()
                    detail = (folder / "worker.log").read_bytes()[:8000].decode("utf-8", "replace")
                    raise ValueError("AUTO analysis failed in selected Klipper Python: " + detail.strip())
                if not target.is_file() or target.stat().st_size > MAX_ANALYSIS_OUTPUT:
                    raise ValueError("AUTO analysis did not produce a bounded completed report")
                result = json.loads(target.read_text(encoding="utf-8"))
                json.dumps(result, allow_nan=False)
                return result
            finally:
                if process.poll() is None:
                    process.kill()
                process.wait(timeout=2.0)
                if debug_directory is not None:
                    try:
                        destination = Path(debug_directory)
                        destination.mkdir(parents=True, exist_ok=True)
                        log.flush()
                        (destination / "worker.log").write_bytes((folder / "worker.log").read_bytes()[:1024 * 1024])
                        (destination / "worker.json").write_text(json.dumps({
                            "python_executable": renderer.python_executable,
                            "dependency_directory": renderer.dependency_directory,
                            "returncode": process.returncode,
                            "input_bytes": source.stat().st_size,
                            "output_bytes": target.stat().st_size if target.exists() else None,
                            "preflight": bool(payload.get("preflight"))}, indent=2), encoding="utf-8")
                    except Exception:
                        logging.exception("accel_homing AUTO worker diagnostic export failed")


def _contact_seek_segments(extractor, arm, home_end, axis, direction, horizon):
    """Read one native seek before set_position can trim its scheduled tail."""
    if not (math.isfinite(arm) and math.isfinite(home_end) and arm < home_end):
        raise ValueError("invalid native homing time interval")
    # On a trigger Klipper can retire the unexecuted remainder to history.
    # Keep the original schedule, including ramps beyond home_end, while no
    # subsequent move has been queued. home_end is not a physical halt time.
    moves, backing = extractor(arm, horizon)
    segments = []
    for move in moves:
        start, duration = move.print_time, move.move_t
        velocity, accel = move.start_v, move.accel
        vector = (move.x_r, move.y_r, move.z_r)
        if not all(math.isfinite(v) for v in (start, duration, velocity, accel) + vector) or duration < 0:
            raise ValueError("nonfinite or invalid trapq segment")
        if not duration or (not velocity and not accel) or start + duration <= arm:
            continue
        if (start < arm - 1e-6 or any(abs(v - expected) > 1e-8 for v, expected in
                zip(vector, (direction, 0., 0.) if axis == "x" else (0., direction, 0.)))):
            raise ValueError("trapq motion does not belong to this directional seek")
        phase = "acceleration" if accel > 0 else "braking" if accel < 0 else "cruise"
        segments.append(dict(phase=phase, start_time_s=start, end_time_s=start + duration,
                             start_velocity_mm_s=velocity, acceleration_mm_s2=accel))
    del backing  # move structs reference these CFFI allocations
    segments.sort(key=lambda item: item["start_time_s"])
    if not segments or segments[0]["start_time_s"] > home_end:
        raise ValueError("no matching trapq seek in the native homing interval")
    order = ["acceleration", "cruise", "braking"]
    if (len(segments) > 3 or any(order.index(left["phase"]) >= order.index(right["phase"])
            or abs(left["end_time_s"] - right["start_time_s"]) > 1e-6
            for left, right in zip(segments, segments[1:]))):
        raise ValueError("ambiguous or discontinuous native seek profile")
    return segments


def _contact_phase_receipt(planner, *, axis, pass_number, arm_time, onset_time):
    """Bind an original detector onset to an observed native approach interval."""
    receipt = {"policy": "prebrake_v1", "accepted": False, "phase": None,
               "trigger_onset_time_s": onset_time, "arm_time_s": arm_time,
               "brake_start_time_s": planner.get("brake_start_time_s") if isinstance(planner, dict) else None,
               "physical_contact_verified": False}
    finite = lambda value: type(value) in (int, float) and math.isfinite(value)
    if (not isinstance(planner, dict) or planner.get("verified_native_seek") is not True
            or planner.get("clock_domain") != "main_mcu_print_time"
            or planner.get("axis") != axis or planner.get("pass_number") != pass_number
            or pass_number != 1 or not finite(arm_time) or planner.get("arm_time_s") != arm_time
            or not finite(onset_time) or onset_time < arm_time
            or not finite(planner.get("native_home_end_time_s"))
            or not arm_time < planner["native_home_end_time_s"] or onset_time > planner["native_home_end_time_s"]
            or planner.get("observation_count", 1) != 1):
        receipt["reason"] = "missing or mismatched native seek/arm/onset evidence"
        return receipt
    segments = planner.get("segments")
    order, previous = ("acceleration", "cruise", "braking"), None
    if not isinstance(segments, (list, tuple)) or not 1 <= len(segments) <= 3:
        receipt["reason"] = "missing native approach phase intervals"
        return receipt
    for segment in segments:
        if not isinstance(segment, dict):
            receipt["reason"] = "invalid native phase interval"
            return receipt
        phase, left, right = (segment.get(key) for key in ("phase", "start_time_s", "end_time_s"))
        if (phase not in order or not finite(left) or not finite(right) or not arm_time <= left < right
                or previous is not None and (order.index(phase) <= order.index(previous[0])
                                            or abs(left - previous[1]) > 1e-6)):
            receipt["reason"] = "ambiguous or discontinuous native approach phases"
            return receipt
        previous = (phase, right)
        if left <= onset_time < right:
            receipt.update(phase=phase, phase_start_time_s=left, phase_end_time_s=right)
    brake = receipt["brake_start_time_s"]
    if finite(brake):
        receipt["onset_minus_brake_ms"] = (onset_time - brake) * 1000.
    receipt["accepted"] = (receipt["phase"] in ("acceleration", "cruise")
                           and (brake is None or finite(brake) and onset_time < brake))
    receipt["reason"] = ("recorded onset lies within a verified native approach phase" if receipt["accepted"] else
                         "recorded onset is in planned braking or outside a verified pre-braking phase")
    return receipt


class AutoNotchCalibration:
    def __init__(self, workflows):
        self.w = workflows
        self.analysis = None
        self.snapshot = None
        self.folder = None
        self.outcome = None
        self.exported_sources = {}
        self.debug_errors = []
        self.debug_events = []
        self.debug_capture_files = []
        self.debug_raw_bytes = 0
        self.debug_failure = None
        self.debug_session = None
        self.debug_context = {}
        self.log_cursor = None
        self.contact_only = False
        self.stage_only = False
        self.contact_planner = None
        self.contact_capture = None
        self.contact_result = None
        self.contact_plans = {}
        self.contact_data = {}
        self.contact_summaries = {}
        self.contact_observer_installed = False
        self.previous_contact_observer = None

    def error(self, text):
        return self.w._error("AUTO notch: " + text)

    def check(self, session):
        self.w._beta_check(session)
        expected = ({"type": "disabled"} if session["auto_notch_phase"] == "raw_reference" else
                    session["auto_notch_analysis"]["profile"])
        mapping = session.get("auto_notch_axis_mapping")
        if mapping is not None and (
                self.w._validate_current_axis_mapping(session.get("axis_mapping"), session["axis"]) != mapping
                or self.w.owner.noise_sensor_context() != session["axis_mapping_context"]):
            raise self.error("mapped detector or sensor context changed during calibration")
        if self.w._filter_settings(session["axis"]) != expected:
            raise self.error("active selected-axis filter changed during calibration")
        frozen = session.get("gentle_frozen_candidate")
        if frozen is not None and (
                session.get("candidate") != frozen["threshold_g"]
                or session.get("start_threshold_g") != frozen["threshold_g"]
                or expected != frozen["profile"]
                or session["auto_notch_analysis"].get("threshold_g") != frozen["threshold_g"]):
            raise self.error("gentle candidate changed after pre-contact selection; restart calibration")
        if self.contact_observer_installed and getattr(self.w.owner, "contact_timing_observer", None) != self._observe_contact_plan:
            raise self.error("gentle native-seek observer changed during qualification")

    def restore(self, session):
        self._remove_contact_observer()
        self.w.owner.notch_profiles[session["axis"]] = dict(session["auto_notch_original_profile"])
        if session["auto_notch_had_staged_axes"]:
            self.w.staged_filter_axes = set(session["auto_notch_original_staged_axes"])
        elif hasattr(self.w, "staged_filter_axes"):
            del self.w.staged_filter_axes

    def summary(self, session):
        analysis = session.get("auto_notch_analysis") or {}
        return {"phase": session.get("auto_notch_phase"), "decision": analysis.get("decision"),
                "profile": analysis.get("profile"), "threshold_g": analysis.get("threshold_g"),
                "analysis_status": analysis.get("status"),
                "report_only": not session["save_config"] and not self.stage_only,
                "staging_requested": self.stage_only,
                "selection_source": analysis.get("selection_source"),
                "candidate_frozen_before_contact": bool(session.get("gentle_frozen_candidate")),
                "impact_metric": "mapped_axis", "axis_mapping": session.get("axis_mapping"),
                "reference_homes": len(session.get("auto_notch_homes", [])),
                "fresh_homes": len(session.get("auto_notch_fresh", [])),
                "retained_sample_limit": MAX_RETAINED_SAMPLES,
                "superseded_reference_captures": session.get("auto_notch_omitted", 0),
                "retained_attempt_record_limit": MAX_ATTEMPT_RECORDS,
                "omitted_attempt_records": session.get("auto_notch_record_offset", 0),
                "artifact_directory": str(self.folder) if self.folder else None,
                "physical_position_verified": False}

    def _keys(self, axis):
        w = self.w
        return [(w.config.get_name(), axis + suffix) for suffix in ("_threshold", "_notch_frequency", "_notch_q")] + [
            (w.model_section, key) for key in ("version", axis + "_threshold", axis + "_successes", axis + "_metadata")]

    def event(self, phase, session=None):
        self.debug_phase = phase
        self.debug_events.append({"time_unix_s": time.time(), "phase": phase,
                                  "attempt": (session or {}).get("attempts")})

    @staticmethod
    def _exception(exc, phase):
        return {"type": type(exc).__name__, "message": str(exc), "phase": phase,
                "traceback": "".join(traceback.format_exception(type(exc), exc, exc.__traceback__))[-32768:]}

    def run(self, gcmd):
        """Keep diagnostic finalization outside all motion/preflight failure paths."""
        if not self.w._experiment_export_idle():
            raise self.error("requires an idle printer with no active workflow; no AUTO run was started")
        self.debug_run_id = uuid.uuid4().hex
        axis = str(gcmd.get("AXIS", "")).lower()
        self.debug_session = {"run_id": self.debug_run_id, "axis": axis,
            "mode": "diagnostic_export", "state": "exporting",
            "save_config": False, "auto_notch_phase": "preflight", "auto_notch_homes": [],
            "auto_notch_fresh": [], "beta_motion_iterations": [], "validation_records": [],
            "records": [], "auto_notch_record_offset": 0, "auto_notch_original_profile": None}
        self.debug_context = {"workflow": "auto_notch", "run_id": self.debug_run_id,
                              "axis": axis, "command": "ACCEL_HOMING_CALIBRATE",
                              "arguments": {key: value for key, value in gcmd.get_command_parameters().items()
                                            if key in ("AXIS", "CENTERED", "NOTCH", "SAVE_CONFIG", "START_THRESHOLD", "STEP", "CONTACT_ONLY")}}
        self.event("preflight", self.debug_session)
        try:
            diagnostics = load_companion_module("accel_homing_troubleshoot")
            self.log_cursor = diagnostics.debug_log_cursor(self.w.printer)
        except Exception as exc:
            self.debug_errors.append("log cursor: " + str(exc))
        try:
            self.folder = Path(self.w.charts_directory).expanduser().resolve() / "notch-auto" / self.debug_run_id
            self.folder.mkdir(parents=True, exist_ok=False)
        except Exception as exc:
            self.debug_errors.append("configured artifact directory: " + str(exc))
            self.folder = Path(tempfile.mkdtemp(prefix="accel_homing_auto_debug_"))
        self.w._info("AUTO diagnostic directory: %s; completed or caught-failure ZIP will be written beside it" % self.folder)
        self._write_debug_json(self.folder / "run.json", dict(self.debug_context, phase="preflight", status="starting"))
        try:
            return self._run(gcmd)
        except Exception as exc:
            if self.debug_failure is None:
                self.debug_failure = self._exception(exc, self.debug_phase)
            raise
        finally:
            session = self.debug_session
            result = self.w.last_result
            if not isinstance(result, dict) or result.get("run_id") != session["run_id"]:
                result = dict(self.outcome or {}, result="aborted", run_id=session["run_id"], axis=axis,
                              reason=(self.debug_failure or {}).get("message", "AUTO run did not finish"),
                              save_config=False, model_staged=False, persistence="not_saved")
                if self.w.session is None or self.w.session is session:
                    self.w.last_result = result
            if result.get("persistence") == "native_save_requested":
                # The verified ZIP predates restart. Do not yield or rebuild it
                # after native SAVE_CONFIG may have stopped the reactor.
                self._write_debug_json(self.folder / "completion.json", {
                    "run_id": session["run_id"], "result": result,
                    "archive_snapshot": "before_native_save", "finished_unix_s": time.time()})
            else:
                self.finalize_debug(session, result)

    def _run(self, gcmd):
        w = self.w
        axis = str(gcmd.get("AXIS", "")).lower()
        if axis not in ("x", "y"):
            raise gcmd.error("AXIS must be X or Y")
        w._require_centered(gcmd)
        if w.session is not None:
            raise self.error("cancel the active workflow before starting a new AUTO run")
        implicit_gentle = bool(w.gentle_homing and gcmd.get("NOTCH", None) is None)
        default_save = 1 if implicit_gentle else 0
        save = bool(gcmd.get_int("SAVE_CONFIG", default_save, minval=0, maxval=1))
        self.stage_only = implicit_gentle and not save
        contact_only = bool(gcmd.get_int("CONTACT_ONLY", 0, minval=0, maxval=1))
        self.contact_only = contact_only
        if contact_only and (not w.gentle_homing or save):
            raise self.error("CONTACT_ONLY=1 requires gentle_homing and SAVE_CONFIG=0")
        if save and axis in w.manual_axes and not w.gentle_homing:
            raise self.error("SAVE_CONFIG=1 requires removing the selected axis/shared explicit threshold first; "
                             "use SAVE_CONFIG=0 for a report-only run")
        limit = w._threshold_limit()
        if not isinstance(limit, (int, float)) or not math.isfinite(limit) or limit < .5:
            raise self.error("a known finite sensor range/threshold headroom is required before AUTO motion")
        self.analysis = load_companion_module("accel_homing_auto_notch")
        self.persistence = load_companion_module("accel_homing_config")
        if save or self.stage_only:
            if w.calibration_config_save is None:
                w.calibration_config_save = self.persistence.CalibrationConfigSave(w.printer, w.config.get_name())
            self.config_helper = w.calibration_config_save
            self.snapshot = self._save_snapshot()
            allowed = set(self._keys(axis))
            if save and any((section, option) not in allowed for section, values in self.snapshot["pending"].items()
                   for option in values):
                raise self.error("SAVE_CONFIG=1 would also save unrelated pending configuration; "
                                 "resolve those changes first or use SAVE_CONFIG=0")
        grid = w._calibration_grid(gcmd)
        w._begin(gcmd, "calibration", axis, calibration_grid=grid, save_config_default=default_save)
        session = w.session
        session["run_id"] = self.debug_run_id
        self.debug_session = session
        session.update(auto_notch=True, auto_notch_controller=self, auto_notch_phase="raw_reference",
                       auto_notch_original_profile=w._filter_settings(axis),
                       auto_notch_original_staged_axes=set(getattr(w, "staged_filter_axes", set())),
                       auto_notch_had_staged_axes=hasattr(w, "staged_filter_axes"),
                       auto_notch_homes=[], auto_notch_fresh=[], auto_notch_analysis=None,
                       beta_generation=w.owner.operation_generation, beta_motion_iterations=[],
                       auto_notch_omitted=0, auto_notch_record_offset=0)
        w.owner.notch_profiles[axis] = {"type": "disabled"}
        try:
            self.event("dependency_preflight", session)
            w._chart_preflight()
            session["auto_notch_dependency_preflight"] = run_analysis({"preflight": True}, w.renderer,
                lambda: self.check(session), debug_directory=self.folder / "worker-preflight")
            with tempfile.TemporaryFile(dir=self.folder) as stream:
                stream.write(b"AUTO artifact preflight")
            self.check(session)
            plan = w._beta_motion_plan(session)
            cruise = plan["distance_mm"] / plan["effective_speed_mm_s"] - plan["effective_speed_mm_s"] / plan["effective_accel_mm_s2"]
            if cruise < self.analysis.MIN_CRUISE_SECONDS:
                raise self.error("central stroke cannot provide the required %.3f s cruise at the existing speed; "
                                 "no motion was started" % self.analysis.MIN_CRUISE_SECONDS)
            session["beta_motion_plan"] = {k: v for k, v in plan.items() if k not in ("extractor", "current_helper")}
            if contact_only:
                w._info("Gentle %s contact check: three central moving-noise captures and one centered "
                        "contact attempt with a profile selected before contact. The diagnostic ZIP records the result." % axis.upper())
            elif w.gentle_homing:
                w._info("Gentle AUTO %s: three independent central moving-noise captures select one fixed "
                        "profile before contact, followed by discovery + ten checks; save=%s" % (axis.upper(), save))
            else:
                w._info("AUTO %s: three independent central moving-noise captures, raw discovery + ten centered "
                        "reference homes, then fresh fixed-profile discovery + ten checks; save=%s" % (axis.upper(), save))
            # Never reuse a prior saved map: qualify this run before collecting
            # evidence that will choose a detector profile.
            self.event("axis_mapping", session)
            w._calibration_axis_mapping(session)
            session["auto_notch_axis_mapping"] = copy.deepcopy(session["axis_mapping"])
            self.check(session)
            self.event("moving_noise", session)
            w._beta_motion_noise(session, plan)
            session["beta_phase"] = "raw_reference"
            for item in session["beta_motion_iterations"]:
                item["record"].update(physical_position_verified=False, impact_metric="mapped_axis",
                                      axis_mapping=copy.deepcopy(session["axis_mapping"]),
                                      axis_mapping_context=copy.deepcopy(session["axis_mapping_context"]))
                item["capture"].update(axis_mapping=copy.deepcopy(session["axis_mapping"]),
                                       coordinate_frame="configured_sensor_stream")
                if item["capture"].get("sensor_context") != session["axis_mapping_context"]:
                    raise self.error("moving capture sensor context changed after axis mapping")
            self.check(session)
            self._bound(session)
            w._require_unhomed_xy()
            self.event("stationary_noise", session)
            w._calibration_noise_preflight(session)
            self.check(session)
            if w.gentle_homing:
                self.select_gentle(session)
                self.previous_contact_observer = getattr(w.owner, "contact_timing_observer", None)
                w.owner.contact_timing_observer = self._observe_contact_plan
                self.contact_observer_installed = True
            if contact_only:
                self.event("single_contact", session)
                record, _capture = w._home(axis, session["candidate"])
                quality, reason = w._quality(axis, record)
                if quality == "implausible":
                    reason = w._mark_implausible_calibration(axis, session["terminal_home"][2], reason)
                if quality != "good":
                    raise self.error(reason)
                self.validate_contact(session, record, _capture)
                w._release_terminal_home(axis, record)
                result = {"result": "contact_check_complete", "axis": axis, "run_id": session["run_id"],
                          "contact": record, "commanded_post_trigger_mm": abs(
                              record["halt_travel_mm_signed"] - record["trigger_travel_mm_signed"]),
                          "candidate_threshold_g": session["candidate"],
                          "signal_filter": copy.deepcopy(session["auto_notch_analysis"]["profile"]),
                          "candidate_frozen_before_contact": True,
                          "qualification_complete": False,
                          "save_config": False, "model_staged": False, "persistence": "report_only",
                          "physical_position_verified": False}
                w._finish(result)
                self.event("single_contact_complete", session)
                return
            self.event("fixed_profile_homes" if w.gentle_homing else "raw_reference_homes", session)
            w._calibrate_trials()
        except Exception as exc:
            self.debug_failure = self._exception(exc, self.debug_phase)
            if w.session is session:
                result = dict(self.outcome or {})
                if result.get("result") != "save_failed" and result.get("persistence") != "native_save_requested":
                    result.update(result="aborted", save_config=False, model_staged=False,
                                  saved_threshold_g=None, persistence="not_saved")
                result.setdefault("run_id", session["run_id"])
                result.setdefault("axis", axis)
                result.setdefault("reason", str(exc))
                try:
                    w._finish(result, release=bool(session.get("axis_touched")
                        and not session.get("early_return_cleanup") and not session.get("axis_only_cleanup")))
                except Exception as cleanup:
                    self.debug_errors.append("cleanup: " + str(cleanup))
            raise
        finally:
            self._remove_contact_observer()

    @staticmethod
    def _debug_clean(value, depth=0, cooperate=None, count=None):
        # Keep all raw sample rows; unlike a status snapshot this is replay data.
        count = [0] if count is None else count
        count[0] += 1
        if cooperate is not None and count[0] % 8192 == 0:
            cooperate()
        if value is None or type(value) in (str, bool, int):
            return value
        if isinstance(value, float):
            return value if math.isfinite(value) else {"nonfinite": str(value)}
        if depth >= 20:
            return {"unavailable": "debug metadata depth limit"}
        if isinstance(value, dict):
            return {str(key): AutoNotchCalibration._debug_clean(item, depth + 1, cooperate, count)
                    for key, item in value.items() if key != "_record"}
        if isinstance(value, (list, tuple)):
            return [AutoNotchCalibration._debug_clean(item, depth + 1, cooperate, count) for item in value]
        return {"unavailable_type": type(value).__name__}

    def _require_debug_idle(self, session):
        owner = self.w.owner
        queued, estimated, empty = owner.toolhead.check_busy(self.w.reactor.monotonic())
        if (owner.homing_active or getattr(owner, "attempt", None) is not None or self.w._printing()
                or not empty or queued > estimated + .1
                or (self.w.session is not None and self.w.session is not session)):
            raise ValueError("diagnostic I/O stopped: printer or another workflow is active")

    def _debug_cooperate(self, session):
        self._require_debug_idle(session)
        self.w.reactor.pause(self.w.reactor.monotonic() + .001)
        self._require_debug_idle(session)

    def _write_debug_json(self, path, value, cooperate=None):
        temporary = path.with_name("." + path.name + ".tmp")
        try:
            with temporary.open("w", encoding="utf-8") as stream:
                _write_json(stream, self._debug_clean(value, cooperate=cooperate), cooperate or (lambda: None), MAX_ANALYSIS_OUTPUT)
            temporary.replace(path)
            return True
        except Exception as exc:
            self.debug_errors.append("%s: %s" % (path.name, exc))
            return False
        finally:
            try:
                if temporary.exists():
                    temporary.unlink()
            except OSError:
                pass

    def _remove_contact_observer(self):
        if not self.contact_observer_installed:
            return
        if getattr(self.w.owner, "contact_timing_observer", None) == self._observe_contact_plan:
            if self.previous_contact_observer is None:
                del self.w.owner.contact_timing_observer
            else:
                self.w.owner.contact_timing_observer = self.previous_contact_observer
        self.contact_observer_installed = False

    def _observe_contact_plan(self, attempt, home_end):
        session = self.debug_session
        if self.w.session is not session:
            return
        plan = session["beta_motion_plan"]
        number = session["attempts"]
        if number in self.contact_plans:
            self.contact_plans[number]["observation_count"] += 1
            self.contact_plans[number].pop("trigger_batch_timing", None)
            self.contact_plans[number]["reason"] = "more than one native seek observation for this attempt"
            return
        observed = self.contact_planner = dict(
            source="motion_report.toolhead.extract_trapq", clock_domain="main_mcu_print_time",
            run_id=session["run_id"], workflow_attempt=number, observation_count=1,
            axis=attempt.axis, pass_number=attempt.pass_no, arm_time_s=attempt.arm_time,
            native_home_end_time_s=home_end, segments=[], brake_start_time_s=None,
            stop_request_observed_time_s=attempt.request_time, stop_request_reason=attempt.reason,
            stop_request_time_source="host main-MCU print-time estimate immediately after trigger dispatch",
            status="unavailable", verified_native_seek=False)
        self.contact_plans[number] = observed
        batch = getattr(attempt, "trigger_batch_times", None)
        if batch is not None and attempt.reason == "impact":
            onset, last_sample, adapter = batch
            stop = attempt.request_time
            timing = dict(source="klipper_batch", clock_domain="main_mcu_print_time",
                          onset_sample_time_s=onset, batch_last_sample_time_s=last_sample,
                          adapter_estimated_print_time_s=adapter, stop_request_observed_time_s=stop,
                          status="unavailable")
            observed["trigger_batch_timing"] = timing
            if stop is not None and math.isfinite(stop) and onset <= last_sample <= adapter <= stop:
                timing.update(status="complete", onset_to_batch_last_ms=(last_sample - onset) * 1000.,
                              batch_last_to_adapter_ms=(adapter - last_sample) * 1000.,
                              adapter_to_stop_request_ms=(stop - adapter) * 1000.)
            else:
                timing["reason"] = "missing dispatch estimate or inconsistent clock order"
        try:
            direction = 1. if plan["selected_endstop_mm"] > plan["center_mm"] else -1.
            segments = _contact_seek_segments(self.w._beta_extractor(), attempt.arm_time, home_end,
                                              attempt.axis, direction, self.w.reactor.NEVER)
        except Exception as exc:
            observed["reason"] = str(exc)
            return
        observed["segments"] = segments
        brake = next((s["start_time_s"] for s in segments if s["phase"] == "braking"), None)
        observed.update(brake_start_time_s=brake,
                        verified_native_seek=True,
                        status="complete" if segments[0]["phase"] == "acceleration" and brake is not None else "partial")
        if brake is not None and attempt.request_time is not None:
            observed["stop_request_minus_brake_ms"] = (attempt.request_time - brake) * 1000.
        if self.previous_contact_observer is not None:
            self.previous_contact_observer(attempt, home_end)

    def collect_missed_contact_tail(self, attempt, home_end):
        """After native stop/disarm, retain pending batches through the end of a missed seek."""
        session = self.debug_session
        plan = self.contact_plans.get(session["attempts"])
        owner = self.w.owner
        if (not plan or plan["arm_time_s"] != attempt.arm_time or plan["axis"] != attempt.axis
                or not math.isfinite(home_end)):
            return
        target = home_end + .100
        tail = plan["missed_contact_tail"] = {"target_time_s": target, "complete": False,
            "last_sample_at_disarm_s": getattr(attempt, "last_capture_time_at_disarm", None),
            "boundary": "native_home_end", "used_for_detection": False}
        generation = owner.operation_generation
        deadline = self.w.reactor.monotonic() + .250
        while True:
            self.w._check_running(session)
            if owner.operation_generation != generation or owner.sensor_fault:
                tail["reason"] = "sensor context changed"
                return
            last = owner.last_captured_sample_time(attempt)
            tail["last_sample_time_s"] = last
            if last is None:
                tail["reason"] = "capture unavailable"
                return
            if last >= target:
                tail["complete"] = True
                return
            now = self.w.reactor.monotonic()
            if now >= deadline:
                tail["reason"] = "capture deadline reached"
                return
            self.w.reactor.pause(min(deadline, now + .005))

    def validate_contact(self, session, record, capture):
        """Reject braking-correlated qualification before any return or saving."""
        number = record.get("workflow_attempt")
        planner = self.contact_plans.get(number)
        detector = capture.get("detector") or {}
        onset = record.get("trigger_onset_time_s")
        receipt = _contact_phase_receipt(planner, axis=session["axis"],
            pass_number=record.get("pass"), arm_time=record.get("arm_time_s"), onset_time=onset)
        if (planner is None or planner.get("run_id") != session["run_id"]
                or planner.get("workflow_attempt") != number
                or detector.get("axis") != session["axis"] or detector.get("attempt_count") != 1
                or detector.get("arm_time_s") != record.get("arm_time_s")
                or detector.get("trigger_onset_time_s") != onset):
            receipt.update(accepted=False, reason="capture does not match the current native seek and original detector onset")
        record["gentle_contact_timing"] = receipt
        if planner is not None:
            planner["qualification"] = copy.deepcopy(receipt)
        if not receipt["accepted"]:
            reason = self.w._mark_implausible_calibration(
                session["axis"], session["terminal_home"][2], receipt["reason"])
            raise self.error(reason)
        return receipt

    def _export_contact_timing(self, session, result, cooperate, number=None):
        source = self.contact_data.get(number, {}) if number is not None else {}
        capture = (source.get("capture") if number is not None else self.contact_capture) or {}
        detector = capture.get("detector") or {}
        observed = self.contact_plans.get(number) if number is not None else self.contact_planner
        planner = observed or dict(status="unavailable", segments=[], brake_start_time_s=None,
                                  reason="no native contact seek was observed")
        matched = bool(observed and detector.get("axis") == planner["axis"]
                       and detector.get("arm_time_s") == planner["arm_time_s"]
                       and detector.get("attempt_count") == 1)
        # Preserve independent evidence even if the capture cannot be paired.
        alignment = planner if matched else {"status": "unavailable", "segments": [], "brake_start_time_s": None}
        candidate = session.get("gentle_frozen_candidate") or {}
        if capture and candidate.get("profile") is not None:
            signal, rows = self.analysis.describe_contact_timing(
                capture, alignment, check_running=cooperate, settings=candidate["profile"])
            historical, historical_rows = self.analysis.describe_contact_timing(
                capture, alignment, check_running=cooperate)
        else:
            signal, rows = {"status": "insufficient_data", "reason": "no retained candidate acquisition"}, []
            historical, historical_rows = dict(signal), []
        signal["analysis_role"] = ("actual_candidate_replay" if signal.get("matches_recorded_live_filter") is True
                                   else "candidate_profile_unverified")
        signal["used_for_live_detection"] = False
        historical.update(analysis_role="historical_fixed_200Hz_Q5_diagnostic_only", used_for_live_detection=False)
        phases = signal.get("phases") or []
        def phase_peak(names):
            values = [item.get("filtered_peak_g") for item in phases if item.get("phase") in names]
            return max((value for value in values if type(value) in (int, float) and math.isfinite(value)), default=None)
        record = source.get("record") or {}
        contact_result = record.get("result") if number is not None else self.contact_result
        native_error = (source.get("error") or {}).get("message")
        missed = "no trigger on %s after full movement" % session["axis"]
        if contact_result is None and native_error and missed in native_error.lower():
            contact_result = "no_trigger"
        qualification = copy.deepcopy(planner.get("qualification"))
        if qualification is None:
            reason = None
            rejection = session.get("axis_only_cleanup")
            if contact_result == "no_trigger":
                reason = "native seek ended without a detector trigger"
            elif isinstance(rejection, dict) and rejection.get("attempt") == number:
                reason = rejection.get("reason")
            if reason:
                qualification = {"accepted": False, "reason": reason}
        attempt_summary = {"workflow_attempt": number if number is not None else session.get("attempts"),
            "contact_result": contact_result, "native_error": native_error,
            "commanded_trigger_travel_mm_signed": record.get("trigger_travel_mm_signed"),
            "commanded_halt_travel_mm_signed": record.get("halt_travel_mm_signed"),
            "detector_onset_g": detector.get("onset_g"),
            "threshold_g": detector.get("threshold_g"), "profile": copy.deepcopy(candidate.get("profile")),
            "replay_status": signal.get("status"), "replay_matches_live_filter": signal.get("matches_recorded_live_filter"),
            "planner_status": planner.get("status"), "capture_matches_plan": matched,
            "brake_start_time_s": planner.get("brake_start_time_s"),
            "original_trigger_onset_time_s": detector.get("trigger_onset_time_s"),
            "peak_before_braking_g": phase_peak(("armed_before_motion", "acceleration", "cruise")),
            "peak_during_braking_g": phase_peak(("braking",)),
            "peak_from_braking_through_tail_g": phase_peak(("braking", "tail")),
            "stop_request_observed_time_s": alignment.get("stop_request_observed_time_s"),
            "trigger_batch_timing": alignment.get("trigger_batch_timing"),
            "peak_before_stop_request_g": (signal.get("before_stop_request") or {}).get("filtered_peak_g"),
            "peak_at_or_after_stop_request_g": (signal.get("after_stop_request") or {}).get("filtered_peak_g"),
            "peak_before_braking_basis": "scheduled planner phases",
            "phases": copy.deepcopy(phases), "qualification": qualification,
            "physical_contact_verified": False}
        self.contact_summaries[attempt_summary["workflow_attempt"]] = attempt_summary
        payload = dict(schema="accel_homing_contact_timing/v2", run_id=session["run_id"], axis=session["axis"],
                       workflow_attempt=number if number is not None else session.get("attempts"),
                       clock_domain="main_mcu_print_time", capture_matches_plan=matched,
                       contact_result=contact_result,
                       detector_threshold_g=detector.get("threshold_g"), detector_onset_time_s=detector.get("onset_time_s"),
                       original_trigger_onset_time_s=detector.get("trigger_onset_time_s"),
                       active_detector={"profile": detector.get("signal_filter"),
                                        "threshold_g": detector.get("threshold_g"),
                                        "candidate": session.get("gentle_frozen_candidate"),
                                        "source": "recorded_live_detector"},
                       planner=planner, qualification=qualification,
                       summary=attempt_summary, signal=signal, historical_signal=historical, physical_position_verified=False)
        stem = "contact-timing" if self.contact_only else "contact-%04d-timing" % number
        paths = {}
        if self._write_debug_json(self.folder / (stem + ".json"), payload, cooperate):
            paths["json"] = str(self.folder / (stem + ".json"))
        path = self.folder / (stem + ".svg")
        charts = load_companion_module("accel_homing_charts")
        charts.write_contact_timing_plot(
            path, rows, alignment if rows else planner, signal, cooperate=cooperate)
        paths["candidate_plot"] = str(path)
        historical_path = self.folder / (stem + "-historical-200Hz-Q5.svg")
        charts.write_contact_timing_plot(historical_path, historical_rows,
            alignment if historical_rows else planner, historical, cooperate=cooperate)
        paths["historical_plot"] = str(historical_path)
        if self.contact_only:
            result["artifacts"].update(contact_timing=paths.get("json"), contact_timing_plot=str(path),
                                       contact_historical_timing_plot=str(historical_path))
        else:
            result["artifacts"].setdefault("contact_timings", {})[str(number)] = paths

    def _export_capture(self, session, kind, capture, cooperate):
        if (not capture or not getattr(self.w, "gentle_homing", False)
                or kind in ("mapping", "mapping-partial")):
            return capture
        project = load_companion_module("accel_homing_axes").mapped_axis_capture
        return project(capture, session.get("axis_mapping"), cooperate)

    def retain_debug_capture(self, session, kind, number, capture, record=None, error=None):
        """Persist stopped acquisitions independently from the accepted training set."""
        if self.folder is None:
            return
        name = "raw/%s-%04d.json.gz" % (kind, number)
        path = self.folder / name
        if path.exists():
            return
        temporary = path.with_name("." + path.name + ".tmp")
        try:
            self._require_debug_idle(session)
            cooperate = lambda: self._debug_cooperate(session)
            capture = self._export_capture(session, kind, capture, cooperate)
            if self.contact_only and kind == "home":
                self.contact_capture = capture
                self.contact_result = (record or {}).get("result")
            if session.get("gentle_frozen_candidate") and kind == "home":
                self.contact_data[number] = {"capture": capture, "record": record, "error": error}
            if self.debug_raw_bytes >= MAX_DEBUG_RAW_BYTES:
                raise ValueError("raw diagnostic budget exhausted (1 GiB compressed per run)")
            path.parent.mkdir(exist_ok=True)
            payload = {"kind": kind, "number": number, "phase": self.debug_phase,
                       "time_unix_s": time.time(), "run_id": session["run_id"],
                       "axis": session.get("axis"), "candidate_g": session.get("candidate"),
                       "native_seek": self.contact_plans.get(number) if kind == "home" else None,
                       "record": record, "capture": capture, "error": error}
            if kind in ("mapping", "mapping-partial"):
                payload["acquisition_purpose"] = "axis_identification"
            with gzip.open(temporary, "wt", encoding="utf-8") as stream:
                _write_json(stream, self._debug_clean(payload, cooperate=cooperate), cooperate, MAX_ANALYSIS_INPUT)
            size = temporary.stat().st_size
            if self.debug_raw_bytes + size > MAX_DEBUG_RAW_BYTES:
                raise ValueError("raw diagnostic budget exceeded (1 GiB compressed per run)")
            temporary.replace(path)
            self.debug_raw_bytes += size
            self.debug_capture_files.append({"file": name, "kind": kind, "number": number,
                                             "compressed_bytes": size,
                                             "sample_count": len((capture or {}).get("samples", ()))})
        except Exception as exc:
            if len(self.debug_errors) < 128:
                self.debug_errors.append("%s: %s" % (name, exc))
        finally:
            try:
                if temporary.exists():
                    temporary.unlink()
            except OSError:
                pass

    def finalize_debug(self, session, result):
        """Cooperative export after motion cleanup; preserve any previous ZIP."""
        if self.folder is None:
            return False
        archive = self.folder.with_suffix(".zip")
        previous_session = self.w.session
        previous_finishing = session.get("finishing")
        reserved = False
        try:
            self._require_debug_idle(session)
            owner = self.w.owner
            # Retain ownership while yielding during export. Another workflow
            # or ordinary home must not start midway through diagnostic I/O.
            self.w.session = session
            session["finishing"] = True
            reserved = True
            cooperate = lambda: self._debug_cooperate(session)
            self.debug_cooperate = cooperate
            failed_home = session.pop("auto_notch_failed_home", None)
            if failed_home is not None:
                self.retain_debug_capture(session, "home", **failed_home)
            result.setdefault("artifacts", {}).update(directory=str(self.folder),
                report=str(self.folder / "report.json"), archive=str(archive),
                archive_snapshot=self.debug_phase)
            if self.contact_only or session.get("gentle_frozen_candidate"):
                try:
                    if self.contact_only:
                        self._export_contact_timing(session, result, cooperate, session.get("attempts"))
                    else:
                        for number in sorted(set(self.contact_data) | set(self.contact_plans)):
                            self._export_contact_timing(session, result, cooperate, number)
                except Exception as exc:
                    self.debug_errors.append("contact timing: " + str(exc))
                finally:
                    self.contact_capture = None
                summaries = [self.contact_summaries[number] for number in sorted(self.contact_summaries)]
                if self._write_debug_json(self.folder / "contact-summary.json", {
                        "schema": "accel_homing_contact_summary/v1", "run_id": session["run_id"],
                        "axis": session.get("axis"), "attempts": summaries,
                        "last_attempt": summaries[-1] if summaries else None,
                        "interpretation": "Peaks replay the recorded candidate profile on retained samples. "
                                          "Planned braking and detector onset are not independent physical contact measurements."}, cooperate):
                    result["artifacts"]["contact_summary"] = str(self.folder / "contact-summary.json")
            self.debug_finalizing = True
            try:
                self._write_report(session, result)
            except Exception as exc:
                result["artifact_report_error"] = str(exc)
                self.debug_errors.append("report: " + str(exc))
                if not (self.folder / "report.json").exists():
                    if not self._write_debug_json(self.folder / "report.json", {
                            "schema": "accel_homing_auto_notch/v2", "run_id": session["run_id"],
                            "axis": session.get("axis"), "outcome": result, "report_error": str(exc)}, cooperate):
                        raise ValueError("could not write the diagnostic report")
            details = dict(self.debug_context, phase=self.debug_phase, log_cursor=self.log_cursor,
                error=self.debug_failure, events=self.debug_events, result=result,
                axis_mapping=session.get("axis_mapping"), axis_mapping_context=session.get("axis_mapping_context"),
                mapping_diagnostics=session.get("axis_mapping_diagnostics"),
                motion_diagnostics=session.get("beta_current_motion"),
                acquisition_faults=session.get("noise_acquisition_faults"),
                sensor=getattr(owner, "sensor_name", None), sensor_fault=getattr(owner, "sensor_fault", None))
            diagnostics = load_companion_module("accel_homing_troubleshoot")
            result["debug_context"] = diagnostics.write_debug_context(self.folder, self.w.printer, details=details)
            if not self._write_debug_json(self.folder / "run.json", dict(self.debug_context,
                    schema="accel_homing_auto_debug/v1", phase=self.debug_phase,
                    exception=self.debug_failure, events=self.debug_events,
                    captures=self.debug_capture_files, errors=self.debug_errors,
                    raw_compressed_bytes=self.debug_raw_bytes, raw_byte_limit=MAX_DEBUG_RAW_BYTES,
                    mapping_diagnostics=session.get("axis_mapping_diagnostics"),
                    motion_diagnostics=session.get("beta_current_motion"),
                    noise_acquisition_faults=session.get("noise_acquisition_faults"),
                    outcome=result), cooperate):
                raise ValueError("could not write the diagnostic run receipt")
            if self.w.gentle_homing and session.get("axis") in ("x", "y"):
                diagnostics.write_run_archive(self.folder, archive, cooperate=cooperate,
                    append=lambda bundle: diagnostics.append_gentle_history(
                        bundle, self.folder.parent, session["axis"], session["run_id"], cooperate))
            else:
                diagnostics.write_run_archive(self.folder, archive, cooperate=cooperate)
            self.w._info("AUTO diagnostic ZIP: %s" % archive)
            return True
        except Exception as exc:
            result["debug_export_error"] = str(exc)
            if archive.is_file():
                result.setdefault("artifacts", {})["archive"] = str(archive)
                result["artifacts"]["archive_snapshot"] = "previous_completed_snapshot"
                message = "AUTO diagnostic ZIP retained from prior snapshot: %s; refresh failed: %s" % (archive, exc)
            else:
                result.get("artifacts", {}).pop("archive", None)
                message = "AUTO diagnostic ZIP unavailable: %s; retained directory %s" % (exc, self.folder)
            logging.warning("accel_homing AUTO diagnostic finalization failed: %s", exc)
            try:
                self.w._info(message)
            except Exception:
                pass
            return False
        finally:
            self.debug_finalizing = False
            if reserved:
                if self.w.session is session:
                    self.w.session = previous_session
                if previous_finishing is None:
                    session.pop("finishing", None)
                else:
                    session["finishing"] = previous_finishing

    def _bound(self, session):
        items = session["beta_motion_iterations"] + session["auto_notch_homes"] + session["auto_notch_fresh"]
        total = sum(len(item["capture"].get("samples", ())) for item in items)
        if total > MAX_RETAINED_SAMPLES:
            raise self.error("complete raw evidence exceeds the bounded sample budget; no truncated analysis is allowed")

    def retain_home(self, session, record, capture, discovery):
        self.check(session)
        if session.get("gentle_frozen_candidate"):
            self.validate_contact(session, record, capture)
        if session["auto_notch_phase"] == "raw_reference" and discovery:
            return
        target = "auto_notch_homes" if session["auto_notch_phase"] == "raw_reference" else "auto_notch_fresh"
        record["axis_mapping_context"] = copy.deepcopy(session["axis_mapping_context"])
        session[target].append({"record": record, "capture": capture})
        self._bound(session)

    def bound_records(self, session):
        omitted = max(0, len(session["records"]) - MAX_ATTEMPT_RECORDS)
        if omitted:
            del session["records"][:omitted]
            session["auto_notch_record_offset"] += omitted

    def select(self, session):
        w = self.w
        self.check(session)
        w._qualified_validation_records()
        w._recenter(session["axis"])
        self.check(session)
        self.event("offline_selection", session)
        result = run_analysis({"motions": session["beta_motion_iterations"], "homes": session["auto_notch_homes"],
            "raw_threshold_g": session["candidate"], "threshold_min_g": 0.5,
            "threshold_max_g": w._threshold_limit()}, w.renderer, lambda: self.check(session),
            debug_directory=self.folder / "worker-selection")
        json.dumps(result, allow_nan=False)
        session["auto_notch_analysis"] = result
        if result.get("decision") not in ("selected", "disabled") or result.get("status") == "insufficient_data":
            raise self.error("insufficient or ambiguous evidence: %s" % "; ".join(result.get("reasons", ())))
        settings = result.get("profile")
        filt = load_companion_module("accel_homing_filter")
        if settings != filt.DISABLED and (not isinstance(settings, dict) or
                filt.profile(settings.get("frequency_hz", float("nan")), settings.get("q", float("nan"))) != settings):
            raise self.error("analyzer returned an invalid fixed filter profile")
        threshold = result.get("threshold_g")
        if type(threshold) not in (int, float):
            raise self.error("analyzer did not return a finite fixed threshold")
        w._validate_calibration_threshold(threshold, "AUTO fixed threshold")
        self.check(session)
        w.owner.notch_profiles[session["axis"]] = dict(settings)
        self.event("fixed_profile_homes", session)
        session.update(auto_notch_phase="qualification", candidate=threshold, successes=0,
                       candidate_qualified=False, candidate_attempt=None, validation_records=[],
                       repeatability_spread_mm=None)
        # Final qualification is fixed by independent analysis. Do not call the
        # stationary-noise seed again after changing the profile.
        w._clear_noise_seeds()
        w._info("AUTO frozen %s profile %s at %.12g g; fresh discovery + ten checks required; "
                "this exact threshold will be saved, with no later margin" %
                (result["decision"], json.dumps(settings, sort_keys=True), threshold))

    def select_gentle(self, session):
        """Choose from free-travel controls, with no prerequisite raw contact."""
        w = self.w
        self.check(session)
        if (not w.gentle_homing or session["attempts"] != 0
                or session["auto_notch_homes"] or session["auto_notch_fresh"]
                or session.get("gentle_frozen_candidate") is not None):
            raise self.error("gentle selection requires an untouched pre-contact run")
        minimum = session.get("gentle_candidate_min_g")
        if type(minimum) not in (int, float) or not math.isfinite(minimum):
            raise self.error("gentle selection lacks its stationary-noise threshold bound")
        w._validate_calibration_threshold(minimum, "gentle candidate minimum")
        self.event("gentle_precontact_selection", session)
        result = run_analysis({"gentle_candidate": True, "motions": session["beta_motion_iterations"],
            "threshold_min_g": minimum, "threshold_max_g": w._threshold_limit()},
            w.renderer, lambda: self.check(session), debug_directory=self.folder / "worker-selection")
        json.dumps(result, allow_nan=False)
        session["auto_notch_analysis"] = result
        if (result.get("status") not in ("selected", "no_benefit")
                or result.get("decision") not in ("selected", "disabled")):
            raise self.error("gentle pre-contact candidate rejected: %s" %
                             "; ".join(result.get("reasons", ())))
        settings = result.get("profile")
        filt = load_companion_module("accel_homing_filter")
        if settings != filt.DISABLED and (not isinstance(settings, dict) or
                filt.profile(settings.get("frequency_hz", float("nan")), settings.get("q", float("nan"))) != settings):
            raise self.error("gentle analyzer returned an invalid fixed filter profile")
        threshold = result.get("threshold_g")
        if type(threshold) not in (int, float) or not math.isfinite(threshold) or threshold < minimum:
            raise self.error("gentle analyzer returned a threshold below its pre-contact bound")
        w._validate_calibration_threshold(threshold, "gentle fixed threshold")
        self.check(session)
        result["selection_source"] = "three_independent_motion_controls_before_first_contact"
        w.owner.notch_profiles[session["axis"]] = dict(settings)
        session.update(auto_notch_phase="qualification", candidate=threshold, start_threshold_g=threshold,
                       successes=0, candidate_qualified=False, candidate_attempt=None, validation_records=[],
                       repeatability_spread_mm=None, attempt_limit=1 if self.contact_only else 11,
                       gentle_frozen_candidate={"profile": copy.deepcopy(settings), "threshold_g": threshold,
                                                "source": result["selection_source"], "contact_inputs": 0})
        w._gentle_freeze_candidate(session, result)
        w._clear_noise_seeds()
        self.check(session)
        self.event("gentle_candidate_frozen", session)
        w._info("Gentle candidate frozen before contact: %s at %.12g g; %s. "
                "No threshold increase or profile change is allowed during this run." %
                (json.dumps(settings, sort_keys=True), threshold,
                 "one contact only" if self.contact_only else "discovery + ten same-profile checks required"))

    def complete(self, session, metadata, qualified, noise_png):
        w = self.w
        self.check(session)
        self.event("qualification_report", session)
        analysis = session["auto_notch_analysis"]
        if (session["candidate"] != analysis["threshold_g"] or metadata["signal_filter"] != analysis["profile"]
                or len(session["auto_notch_fresh"]) != 11):
            raise self.error("fresh homes do not match the frozen analyzed profile and threshold")
        gentle = bool(session.get("gentle_frozen_candidate"))
        if gentle:
            for home in session["auto_notch_fresh"]:
                self.validate_contact(session, home["record"], home["capture"])
            metadata["gentle_contact_phase_policy"] = "prebrake_v1"
            metadata["gentle_contact_timing"] = [copy.deepcopy(home["record"]["gentle_contact_timing"])
                                               for home in session["auto_notch_fresh"]]
        metadata["auto_notch_selection"] = {"decision": analysis["decision"], "run_id": session["run_id"],
            "report": str(self.folder / "report.json"), "raw_reference_homes": 0 if gentle else 10,
            "fresh_homes": 11, "candidate_frozen_before_contact": gentle,
            "threshold_source": ("independent_motion_train_then_holdout_before_contact" if gentle else
                                 "frozen_train_selection_then_holdout_validation"),
            "physical_position_verified": False}
        result = {"result": "qualification_complete_pending_finalization",
            "axis": session["axis"], "run_id": session["run_id"], "threshold_g": session["candidate"],
            "tested_threshold_g": session["candidate"], "saved_threshold_g": None,
            "threshold_margin_g": 0.0, "saved_threshold_tested": True, "signal_filter": analysis["profile"],
            "successes": 10, "save_config": False, "model_staged": False,
            "physical_position_verified": False, "persistence": "report_only"}
        self.outcome = result
        session["auto_notch_proposed_model"] = metadata
        self._write_report(session, result)
        charts = load_companion_module("accel_homing_charts")
        png = charts.write_notch_auto(self.folder, session["run_id"], session["axis"], analysis,
                                     renderer=w.renderer, check_running=lambda: self.check(session))
        result["artifacts"] = {"report": str(self.folder / "report.json"), **png}
        self.check(session)
        w._qualified_validation_records()
        # Finish stream/acceleration/profile restoration before persistence.
        session["auto_notch_restore_requested"] = True
        w._restore()
        if session["save_config"] or self.stage_only:
            self.event("before_native_save" if session["save_config"] else "before_staging", session)
            if not self.finalize_debug(session, result):
                raise self.error("diagnostic ZIP could not be completed; configuration was not staged or saved")
            if session.get("finish_interrupted"):
                raise self.error(session["finish_interrupted"])
            self._save(session, metadata, result, native_save=session["save_config"])
        else:
            result["result"] = "auto_notch_proposed"
        w._finish(result)
        self.event("complete", session)
        w._announce_pngs(png, "AUTO notch selection", w.last_result)
        w._info("AUTO %s: %s, fixed threshold %.12g g; %s. Report %s" %
                (session["axis"].upper(), analysis["decision"], session["candidate"],
                 "native SAVE_CONFIG requested" if session["save_config"] else
                 "model staged for SAVE_CONFIG; original runtime settings restored" if self.stage_only else
                 "report only; original runtime settings restored",
                 self.folder / "report.json"))

    def _save_snapshot(self):
        configfile = self.w.printer.lookup_object("configfile")
        reader = self.persistence._NativeReader(configfile)
        filename = self.w.printer.get_start_args()["config_file"]
        root, files, edges = self.config_helper._configuration(reader, filename)
        self.config_helper._verify_configuration(reader, files, edges)
        autosave = configfile.autosave
        state = configfile if hasattr(configfile, "status_save_pending") else autosave
        return {"configfile": configfile, "root": root, "files": files, "edges": edges,
                "parser": copy.deepcopy(autosave.fileconfig), "state": state,
                "pending": copy.deepcopy(state.status_save_pending), "flag": state.save_config_pending}

    def _save(self, session, metadata, result, native_save=True):
        w, axis = self.w, session["axis"]
        fresh, previous = self._save_snapshot(), self.snapshot
        parser_values = lambda parser: {section: dict(parser.items(section, raw=True)) for section in parser.sections()}
        if (fresh["configfile"] is not previous["configfile"]
                or fresh["files"] != previous["files"] or fresh["edges"] != previous["edges"]
                or parser_values(fresh["parser"]) != parser_values(previous["parser"])
                or fresh["pending"] != previous["pending"] or fresh["flag"] != previous["flag"]):
            raise self.error("configuration changed during AUTO measurement; nothing was saved")
        w._check_running(session)
        w._qualified_validation_records()
        if w._filter_settings(axis) != session["auto_notch_original_profile"]:
            raise self.error("runtime filter cleanup did not complete before saving")
        settings = metadata["signal_filter"]
        filtered = {a for a in (w.model.thresholds if w.model else ())
                    if a != axis and w.model.metadata[a].get("signal_filter", {}).get("type", "disabled") != "disabled"}
        if settings["type"] != "disabled":
            filtered.add(axis)
        edits = [(w.config.get_name(), axis + "_notch_frequency", str(settings.get("frequency_hz", 0.))),
                 (w.config.get_name(), axis + "_notch_q", str(settings.get("q", 10.))),
                 (w.model_section, "version", "2" if filtered else "1"),
                 (w.model_section, axis + "_threshold", "%.17g" % session["candidate"]),
                 (w.model_section, axis + "_successes", "10"),
                 (w.model_section, axis + "_metadata", json.dumps(metadata, separators=(",", ":"), allow_nan=False))]
        edits.append((w.config.get_name(), axis + "_threshold", "%.17g" % session["candidate"]))
        staged_metadata = {axis: metadata}
        conflict = session.get("axis_mapping_peer_conflict")
        if conflict is not None:
            other = conflict["axis"]
            if w._staged_or_saved_metadata(other) != conflict["metadata"]:
                raise self.error("other-axis model changed during automatic mapping")
            invalidated = copy.deepcopy(conflict["metadata"])
            invalidated["axis_mapping_invalidated"] = {
                "reason": "sensor orientation changed or channel reassigned by qualified opposite-axis calibration",
                "by_axis": axis, "by_run_id": session["run_id"]}
            staged_metadata[other] = invalidated
            edits.append((w.model_section, other + "_metadata", json.dumps(invalidated, separators=(",", ":"), allow_nan=False)))
        rollback = None
        try:
            rollback = self.config_helper.register_settings(axis, edits, w.model_section)
            w._stage_model_settings(previous["configfile"], edits)
            if native_save:
                command = w.gcode.create_gcode_command("SAVE_CONFIG", "SAVE_CONFIG", {})
                self.config_helper.cmd_SAVE_CONFIG(command)
        except Exception:
            unchanged = self.persistence._matches(previous["root"])
            if unchanged:
                previous["configfile"].autosave.fileconfig = previous["parser"]
                previous["state"].status_save_pending = previous["pending"]
                previous["state"].save_config_pending = previous["flag"]
                if rollback is not None:
                    rollback()
            result.update(result="save_failed", save_config=False, model_staged=False,
                          persistence="save_failed_restored_pending" if unchanged else "persistence_restart_unconfirmed")
            raise
        w.pending_model_metadata.update(copy.deepcopy(staged_metadata))
        result.update(result="calibrated", save_config=native_save, model_staged=True,
                      saved_threshold_g=session["candidate"],
                      persistence="native_save_requested" if native_save else "staged_not_saved")

    def _write_report(self, session, result):
        guarded = (self.w.session is session and not session.get("finishing")
                   and not session.get("auto_notch_restore_requested"))
        check = (lambda: self.check(session)) if guarded else (lambda: None)
        cooperate = self.debug_cooperate if getattr(self, "debug_finalizing", False) else \
            (lambda: _checkpoint(self.w.renderer, check))
        cooperate()
        sources = []
        groups = (("motion", session["beta_motion_iterations"], 2),
                  ("reference", session["auto_notch_homes"], 6),
                  ("fresh", session["auto_notch_fresh"], 0))
        for kind, items, split in groups:
            for index, item in enumerate(items):
                name = "%s-%02d.json.gz" % (kind, index + 1)
                if self.exported_sources.get(name) != id(item):
                    temporary = self.folder / ("." + name + ".tmp")
                    try:
                        capture = self._export_capture(session, kind, item["capture"], cooperate)
                        with gzip.open(temporary, "wt", encoding="utf-8") as stream:
                            _write_json(stream, dict(item, capture=capture), cooperate, MAX_ANALYSIS_INPUT)
                        temporary.replace(self.folder / name)
                        self.exported_sources[name] = id(item)
                    finally:
                        if temporary.exists():
                            temporary.unlink()
                identity_kind = "home" if kind == "reference" else kind
                number = item["record"].get("iteration" if kind == "motion" else "workflow_attempt", index + 1)
                sources.append({"capture_id": "%s:%s:%s:%d" % (session["run_id"], session["axis"], identity_kind, number), "file": name,
                                "role": "fresh_qualification" if kind == "fresh" else "train" if index < split else "holdout",
                                "sample_count": len(item["capture"]["samples"])})
        report = {"schema": "accel_homing_auto_notch/v2", "run_id": session["run_id"],
                  "axis": session["axis"], "impact_metric": "mapped_axis",
                  "axis_mapping": session.get("axis_mapping"), "analysis": session.get("auto_notch_analysis"),
                  "summary": self.summary(session), "sources": sources,
                  "motion_plan": session.get("beta_motion_plan"), "motion_cleanup": session.get("beta_restore"),
                  "gentle_motion_noise": session.get("gentle_motion_noise"),
                  "gentle_frozen_candidate": session.get("gentle_frozen_candidate"),
                  "gentle_contact_plans": [self.contact_plans[number] for number in sorted(self.contact_plans)],
                  "gentle_contact_summary": [self.contact_summaries[number] for number in sorted(self.contact_summaries)],
                  "proposed_model": session.get("auto_notch_proposed_model"),
                  "qualification_records": session["validation_records"], "retained_attempt_records": session["records"],
                  "omitted_attempt_records": session["auto_notch_record_offset"],
                  "original_profile": session["auto_notch_original_profile"], "outcome": result,
                  "interpretation": ("Candidate selected from independent free-travel controls before contact; "
                                     "fresh homes validate the exact frozen mapped detector. Commanded travel "
                                     "and trigger-to-halt limits do not independently prove physical contact."
                                     if session.get("gentle_frozen_candidate") else
                                     "Commanded/plausible reference stops, not verified physical contact. "
                                     "Candidate reference replay ends at the original raw trigger; post-stop tail is not evidence of a later impact.")}
        temporary = self.folder / ".report.json.tmp"
        try:
            with temporary.open("w", encoding="utf-8") as stream:
                _write_json(stream, report, cooperate, MAX_ANALYSIS_OUTPUT)
                stream.write("\n")
            temporary.replace(self.folder / "report.json")
        finally:
            if temporary.exists():
                temporary.unlink()


if __name__ == "__main__":
    if len(sys.argv) != 5 or sys.argv[1] != "--analysis-worker":
        raise SystemExit("Internal AUTO analysis worker only")
    module = load_companion_module("accel_homing_auto_notch")
    source, target = Path(sys.argv[2]), Path(sys.argv[3])
    if source.stat().st_size > MAX_ANALYSIS_INPUT:
        raise ValueError("AUTO worker input exceeds its capacity")
    payload = json.loads(source.read_text(encoding="utf-8"))
    if payload.get("preflight"):
        result = module.preflight()
    elif payload.pop("gentle_candidate", False):
        result = module.analyze_gentle(**payload)
    else:
        result = module.analyze(**payload)
    encoded = json.dumps(result, allow_nan=False, separators=(",", ":")).encode("utf-8")
    if len(encoded) > MAX_ANALYSIS_OUTPUT:
        raise ValueError("AUTO worker output exceeds its capacity")
    target.write_bytes(encoded)
