# Supervised automatic notch qualification and persistence. GPLv3.
"""Offline selection between two monitored calibration phases; never auto-tune an armed filter."""
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


def run_analysis(payload, renderer, check_running):
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


class AutoNotchCalibration:
    def __init__(self, workflows):
        self.w = workflows
        self.analysis = None
        self.snapshot = None
        self.folder = None
        self.outcome = None
        self.exported_sources = {}

    def error(self, text):
        return self.w._error("AUTO notch: " + text)

    def check(self, session):
        self.w._beta_check(session)
        expected = ({"type": "disabled"} if session["auto_notch_phase"] == "raw_reference" else
                    session["auto_notch_analysis"]["profile"])
        if self.w._filter_settings(session["axis"]) != expected:
            raise self.error("active selected-axis filter changed during calibration")

    def restore(self, session):
        self.w.owner.notch_profiles[session["axis"]] = dict(session["auto_notch_original_profile"])
        if session["auto_notch_had_staged_axes"]:
            self.w.staged_filter_axes = set(session["auto_notch_original_staged_axes"])
        elif hasattr(self.w, "staged_filter_axes"):
            del self.w.staged_filter_axes

    def summary(self, session):
        analysis = session.get("auto_notch_analysis") or {}
        return {"phase": session.get("auto_notch_phase"), "decision": analysis.get("decision"),
                "profile": analysis.get("profile"), "threshold_g": analysis.get("threshold_g"),
                "analysis_status": analysis.get("status"), "report_only": not session["save_config"],
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
        return [(w.config.get_name(), axis + suffix) for suffix in ("_notch_frequency", "_notch_q")] + [
            (w.model_section, key) for key in ("version", axis + "_threshold", axis + "_successes", axis + "_metadata")]

    def run(self, gcmd):
        w = self.w
        axis = str(gcmd.get("AXIS", "")).lower()
        if axis not in ("x", "y"):
            raise gcmd.error("AXIS must be X or Y")
        w._require_centered(gcmd)
        if w.session is not None:
            raise self.error("cancel the active workflow before starting a new AUTO run")
        save = bool(gcmd.get_int("SAVE_CONFIG", 0, minval=0, maxval=1))
        if save and axis in w.manual_axes:
            raise self.error("SAVE_CONFIG=1 requires removing the selected axis/shared explicit threshold first; "
                             "use SAVE_CONFIG=0 for a report-only run")
        limit = w._threshold_limit()
        if not isinstance(limit, (int, float)) or not math.isfinite(limit) or limit < .5:
            raise self.error("a known finite sensor range/threshold headroom is required before AUTO motion")
        self.analysis = load_companion_module("accel_homing_auto_notch")
        self.persistence = load_companion_module("accel_homing_center")
        self.config_helper = self.persistence.CenterCalibration(w)
        if save:
            self.snapshot = self.config_helper._save_snapshot(self._keys(axis))
        grid = w._calibration_grid(gcmd)
        w._begin(gcmd, "calibration", axis, calibration_grid=grid, save_config_default=0)
        session = w.session
        session.update(auto_notch=True, auto_notch_controller=self, auto_notch_phase="raw_reference",
                       auto_notch_original_profile=w._filter_settings(axis),
                       auto_notch_original_staged_axes=set(getattr(w, "staged_filter_axes", set())),
                       auto_notch_had_staged_axes=hasattr(w, "staged_filter_axes"),
                       auto_notch_homes=[], auto_notch_fresh=[], auto_notch_analysis=None,
                       beta_generation=w.owner.operation_generation, beta_motion_iterations=[],
                       auto_notch_omitted=0, auto_notch_record_offset=0)
        w.owner.notch_profiles[axis] = {"type": "disabled"}
        try:
            w._chart_preflight()
            session["auto_notch_dependency_preflight"] = run_analysis({"preflight": True}, w.renderer,
                                                                       lambda: self.check(session))
            self.folder = Path(w.charts_directory).expanduser().resolve() / "notch-auto" / session["run_id"]
            self.folder.mkdir(parents=True, exist_ok=False)
            with tempfile.TemporaryFile(dir=self.folder) as stream:
                stream.write(b"AUTO artifact preflight")
            self.check(session)
            plan = w._beta_motion_plan(session)
            cruise = plan["distance_mm"] / plan["effective_speed_mm_s"] - plan["effective_speed_mm_s"] / plan["effective_accel_mm_s2"]
            if cruise < self.analysis.MIN_CRUISE_SECONDS:
                raise self.error("central stroke cannot provide the required %.3f s cruise at the existing speed; "
                                 "no motion was started" % self.analysis.MIN_CRUISE_SECONDS)
            session["beta_motion_plan"] = {k: v for k, v in plan.items() if k not in ("extractor", "current_helper")}
            w._info("AUTO %s: three independent central moving-noise captures, raw discovery + ten centered "
                    "reference homes, then fresh fixed-profile discovery + ten checks; save=%s" % (axis.upper(), save))
            w._beta_motion_noise(session, plan)
            session["beta_phase"] = "raw_reference"
            for item in session["beta_motion_iterations"]:
                item["record"]["physical_position_verified"] = False
            self.check(session)
            self._bound(session)
            w._require_unhomed_xy()
            w._calibration_noise_preflight(session)
            self.check(session)
            w._calibrate_trials()
        except Exception as exc:
            if w.session is session:
                result = dict(self.outcome or {})
                if result.get("result") != "save_failed" and result.get("persistence") != "native_save_requested":
                    result.update(result="aborted", save_config=False, model_staged=False,
                                  saved_threshold_g=None, persistence="not_saved")
                result.setdefault("reason", str(exc))
                w._finish(result, release=bool(session.get("axis_touched")
                    and not session.get("early_return_cleanup") and not session.get("axis_only_cleanup")))
            if self.folder is not None:
                try:
                    self._write_report(session, w.last_result or {"result": "aborted", "reason": str(exc)})
                except Exception:
                    pass  # Preserve the original motion/cleanup/persistence exception.
            raise

    def _bound(self, session):
        items = session["beta_motion_iterations"] + session["auto_notch_homes"] + session["auto_notch_fresh"]
        total = sum(len(item["capture"].get("samples", ())) for item in items)
        if total > MAX_RETAINED_SAMPLES:
            raise self.error("complete raw evidence exceeds the bounded sample budget; no truncated analysis is allowed")

    def retain_home(self, session, record, capture, discovery):
        self.check(session)
        if session["auto_notch_phase"] == "raw_reference" and discovery:
            return
        target = "auto_notch_homes" if session["auto_notch_phase"] == "raw_reference" else "auto_notch_fresh"
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
        result = run_analysis({"motions": session["beta_motion_iterations"], "homes": session["auto_notch_homes"],
            "raw_threshold_g": session["candidate"], "threshold_min_g": 0.5,
            "threshold_max_g": w._threshold_limit()}, w.renderer, lambda: self.check(session))
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
        session.update(auto_notch_phase="qualification", candidate=threshold, successes=0,
                       candidate_qualified=False, candidate_attempt=None, validation_records=[],
                       repeatability_spread_mm=None)
        # Final qualification is fixed by independent analysis. Do not call the
        # stationary-noise seed again after changing the profile.
        w._clear_noise_seeds()
        w._info("AUTO frozen %s profile %s at %.12g g; fresh discovery + ten checks required; "
                "this exact threshold will be saved, with no later margin" %
                (result["decision"], json.dumps(settings, sort_keys=True), threshold))

    def complete(self, session, metadata, qualified, noise_png):
        w = self.w
        self.check(session)
        analysis = session["auto_notch_analysis"]
        if (session["candidate"] != analysis["threshold_g"] or metadata["signal_filter"] != analysis["profile"]
                or len(session["auto_notch_fresh"]) != 11):
            raise self.error("fresh homes do not match the frozen analyzed profile and threshold")
        metadata["auto_notch_selection"] = {"decision": analysis["decision"], "run_id": session["run_id"],
            "report": str(self.folder / "report.json"), "raw_reference_homes": 10, "fresh_homes": 11,
            "threshold_source": "frozen_train_selection_then_holdout_validation", "physical_position_verified": False}
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
        if session["save_config"]:
            self._save(session, metadata, result)
        else:
            result["result"] = "auto_notch_proposed"
        w._finish(result)
        try:
            self._write_report(session, w.last_result)
        except Exception as exc:
            # Native SAVE_CONFIG may already have written files and requested
            # restart. A report refresh cannot undo or misrepresent that result.
            w.last_result["artifact_report_error"] = str(exc)
            logging.warning("accel_homing AUTO final report refresh failed: %s", exc)
        w._announce_pngs(png, "AUTO notch selection", w.last_result)
        w._info("AUTO %s: %s, fixed threshold %.12g g; %s. Report %s" %
                (session["axis"].upper(), analysis["decision"], session["candidate"],
                 "native SAVE_CONFIG requested" if session["save_config"] else "report only; original runtime settings restored",
                 self.folder / "report.json"))

    def _save(self, session, metadata, result):
        w, axis = self.w, session["axis"]
        keys = self._keys(axis)
        fresh = self.config_helper._save_snapshot(keys)
        previous = self.snapshot
        if (fresh["configfile"] is not previous["configfile"] or any(fresh[key] != previous[key]
                for key in ("source", "files", "resolved", "paths"))
                or self.config_helper._parser_values(fresh["parser"]) != self.config_helper._parser_values(previous["parser"])):
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
        configfile, access = previous["configfile"], previous["access"]
        transaction, native_started = None, False
        try:
            for section, option, value in edits:
                configfile.set(section, option, value)
            replacements = access.include_edits(previous["filename"], previous["source"], previous["files"],
                                                access.parser, set(keys))
            selected = copy.deepcopy(access.parser)
            for section in selected.sections():
                for option in selected.options(section):
                    if (section, option) not in keys:
                        selected.remove_option(section, option)
            previous["selected_parser"] = selected
            transaction = self.persistence._IncludeSaveTransaction(previous, replacements)
            if transaction.backup_dir:
                result["config_backup_directory"] = transaction.backup_dir
            transaction.apply()
            command = w.gcode.create_gcode_command("SAVE_CONFIG", "SAVE_CONFIG", {})
            native_started = True
            previous["native_save"](command)
        except Exception:
            errors = transaction.rollback(native_started) if transaction else []
            configfile.autosave.fileconfig = previous["parser"]
            access.save_owner.status_save_pending = previous["pending"]
            access.save_owner.save_config_pending = previous["flag"]
            result.update(result="save_failed", save_config=False, model_staged=False,
                          persistence="save_failed_restored_original_files" if not errors else "rollback_incomplete")
            if errors:
                result["config_rollback_errors"] = errors
            raise
        result.update(result="calibrated", save_config=True, model_staged=True,
                      saved_threshold_g=session["candidate"], persistence="native_save_requested")

    def _write_report(self, session, result):
        guarded = (self.w.session is session and not session.get("finishing")
                   and not session.get("auto_notch_restore_requested"))
        check = (lambda: self.check(session)) if guarded else (lambda: None)
        cooperate = lambda: _checkpoint(self.w.renderer, check)
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
                        with gzip.open(temporary, "wt", encoding="utf-8") as stream:
                            _write_json(stream, item, cooperate, MAX_ANALYSIS_INPUT)
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
        report = {"schema": "accel_homing_auto_notch/v1", "run_id": session["run_id"],
                  "axis": session["axis"], "analysis": session.get("auto_notch_analysis"),
                  "summary": self.summary(session), "sources": sources,
                  "motion_plan": session.get("beta_motion_plan"), "motion_cleanup": session.get("beta_restore"),
                  "proposed_model": session.get("auto_notch_proposed_model"),
                  "qualification_records": session["validation_records"], "retained_attempt_records": session["records"],
                  "omitted_attempt_records": session["auto_notch_record_offset"],
                  "original_profile": session["auto_notch_original_profile"], "outcome": result,
                  "interpretation": "Commanded/plausible reference stops, not verified physical contact. "
                                    "Candidate reference replay ends at the original raw trigger; post-stop tail is not evidence of a later impact."}
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
    result = module.preflight() if payload.get("preflight") else module.analyze(**payload)
    encoded = json.dumps(result, allow_nan=False, separators=(",", ":")).encode("utf-8")
    if len(encoded) > MAX_ANALYSIS_OUTPUT:
        raise ValueError("AUTO worker output exceeds its capacity")
    target.write_bytes(encoded)
