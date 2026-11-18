"""Offline Moonraker registration/source transaction fixtures; never a host install."""
import base64
import copy
import contextlib
import hashlib
import json
import io
import os
from pathlib import Path
import shlex
import shutil
import subprocess
import tempfile
import types
import unittest
from unittest import mock

import test_installer as installer_tests

installer = installer_tests.installer
updater = installer_tests.load("accel_homing_updater")


class UpdaterFixture(unittest.TestCase):
    plan = installer_tests.HostFixture.plan

    def setUp(self):
        installer_tests.HostFixture.setUp(self)
        installer.load_updater.return_value = updater
        native_source = self.home / "accel_homing"
        self.source.rename(native_source)
        self.source = native_source
        identity = mock.patch.object(updater, "repo_identity", side_effect=lambda source, api: dict(
            commit=self.commit, origin=updater.ORIGIN, branch=updater.BRANCH,
            upstream="origin/" + updater.BRANCH))
        identity.start()
        self.addCleanup(identity.stop)
        source = mock.patch.object(updater, "source_commit", side_effect=lambda source, api: self.commit)
        source.start()
        self.addCleanup(source.stop)
        home = mock.patch.object(updater.Path, "home", return_value=self.home)
        home.start()
        self.addCleanup(home.stop)
        self.selected["socket"] = self.home / "klippy.sock"
        self.data = self.home / "printer_data_moon"
        self.moon_config = self.data / "config/moonraker.conf"
        self.moon_config.parent.mkdir(parents=True)
        self.moon_config.write_text("[server]\nklippy_uds_address: %s\n[update_manager]\n" % self.selected["socket"])
        self.commit = "a" * 40
        self.marker()
        self.refresh()
        self.instance_discovery = updater.moonraker_instances
        instances = mock.patch.object(updater, "moonraker_instances", side_effect=lambda api: [self.moon])
        instances.start()
        self.addCleanup(instances.stop)

    def marker(self):
        # Configuration/transaction fixtures mock the native Git identity. The
        # isolated Git tests below validate actual clone content and commands.
        pass

    def refresh(self):
        self.moon = updater._moon_command(
            [str(self.home / "moon-env/bin/python"), str(self.home / "moonraker/moonraker.py"), "-d", str(self.data)],
            self.home, installer.uninstall_api(), pid=77, service="moonraker-test.service", scope="system", start="100")

    def updater_plan(self):
        return updater.make_plan(self.source, self.selected, installer.arguments([]), installer.uninstall_api())

    def install_config(self, plan):
        changes = {path: value for path, value in self.plan()[0].items() if value["kind"] != "symlink"}
        changes.update(plan["changes"])
        backup = installer.apply_plan(changes, self.config, selected=self.selected, updater=plan)
        self.refresh()
        return backup


    def test_live_moonraker_paths_follow_argv_environment_and_verified_owner_home(self):
        owner = self.home / "moonraker-owner"
        default_data = owner / "printer_data"
        default_config = default_data / "config/moonraker.conf"
        default_config.parent.mkdir(parents=True)
        default_config.write_bytes(self.moon_config.read_bytes())
        alternate = self.home / "alternate-data"
        alternate.mkdir()
        custom = self.home / "custom-moonraker.conf"
        custom.write_bytes(self.moon_config.read_bytes())
        argv = [str(self.home / "moon-env/bin/python"), "-m", "moonraker"]
        cases = (
            ([], {"HOME": str(owner), "MOONRAKER_DATA_PATH": str(self.data)}, self.data, self.moon_config),
            ([], {"HOME": str(owner)}, default_data, default_config),
            ([], {}, default_data, default_config),
            ([], {"HOME": str(owner), "MOONRAKER_DATA_PATH": "~/printer_data"}, default_data, default_config),
            (["-d", str(alternate), "--datapath", str(self.data)],
             {"HOME": str(owner), "MOONRAKER_DATA_PATH": str(alternate)}, self.data, self.moon_config),
            (["--datapath="], {"HOME": str(owner), "MOONRAKER_DATA_PATH": str(self.data)}, default_data, default_config),
            (["-d", str(self.data)], {"HOME": str(owner), "MOONRAKER_CONFIG_PATH": str(custom)}, self.data, custom),
            (["-d", str(self.data), "--configfile", str(self.moon_config)],
             {"HOME": str(owner), "MOONRAKER_CONFIG_PATH": str(custom)}, self.data, self.moon_config),
        )
        for flags, environment, data, config in cases:
            with self.subTest(flags=flags, environment_keys=sorted(environment)):
                found = updater._moon_command(argv + flags, self.home, installer.uninstall_api(), pid=77,
                                              path_environment=environment, owner_home=str(owner))
                self.assertIsNotNone(found)
                self.assertEqual(found["data"], data)
                self.assertEqual(found["config"], config)
        # Moonraker treats an empty config as a supplied path, not its default.
        self.assertIsNone(updater._moon_command(argv, self.home, installer.uninstall_api(), pid=77,
                                               path_environment={"HOME": str(owner), "MOONRAKER_CONFIG_PATH": ""},
                                               owner_home=str(owner)))
        # No live environment proof must not create an assumed default instance.
        self.assertIsNone(updater._moon_command(argv, self.home, installer.uninstall_api(), pid=77))

    def test_moonraker_environment_reader_retains_only_bounded_path_inputs(self):
        process = self.home / "synthetic-process"
        process.mkdir()
        path = process / "environ"
        path.write_bytes(b"HOME=/verified/home\0MOONRAKER_DATA_PATH=/verified/data\0"
                         b"MOONRAKER_CONFIG_PATH=/verified/moonraker.conf\0"
                         b"SECRET_TOKEN=must-not-be-retained\0MOONRAKER_LOG_PATH=/ignored/log\0")
        self.assertEqual(updater._path_environment(process), {
            "HOME": "/verified/home", "MOONRAKER_DATA_PATH": "/verified/data",
            "MOONRAKER_CONFIG_PATH": "/verified/moonraker.conf"})
        path.write_bytes(b"HOME=/one\0HOME=/two\0")
        with self.assertRaisesRegex(ValueError, "ambiguous"):
            updater._path_environment(process)
        path.write_bytes(b"IGNORED=" + b"x" * 65536)
        with self.assertRaisesRegex(ValueError, "bound"):
            updater._path_environment(process)

    def test_omitted_klippy_socket_uses_moonraker_tmp_default_without_alternate_probing(self):
        self.moon_config.write_text("[server]\n[update_manager]\n")
        self.refresh()
        expected = installer.canonical("/tmp/klippy_uds", self.home)
        self.assertEqual(self.moon["socket"], expected)
        selected = {**self.selected, "socket": expected}
        self.assertIs(updater.discover_moonraker(selected, installer.uninstall_api(), instances=[self.moon]), self.moon)
        explicit = self.data / "comms/custom-klippy.sock"
        self.moon_config.write_text("[server]\nklippy_uds_address: %s\n" % explicit)
        self.refresh()
        self.assertEqual(self.moon["socket"], explicit)
        with self.assertRaisesRegex(installer.InstallError, "socket mismatch"):
            updater.discover_moonraker(selected, installer.uninstall_api(), instances=[self.moon])

    def test_configured_socket_paths_resolve_cwd_and_literal_data_path_without_defaulting_empty(self):
        cwd = self.home / "moonraker-working-directory"
        cwd.mkdir()
        argv = [str(self.home / "moon-env/bin/python"), "-m", "moonraker", "-d", str(self.data)]
        cases = (("comms/klippy.sock", cwd / "comms/klippy.sock"),
                 ("{data_path}/comms/klippy.sock", self.data / "comms/klippy.sock"),
                 ("{ data_path }/comms/klippy.sock", self.data / "comms/klippy.sock"),
                 ("", cwd))
        for configured, expected in cases:
            with self.subTest(configured=configured):
                self.moon_config.write_text("[server]\nklippy_uds_address: %s\n" % configured)
                found = updater._moon_command(argv, cwd, installer.uninstall_api(), pid=77,
                                              path_environment={"HOME": str(self.home)}, owner_home=str(self.home))
                self.assertIsNotNone(found)
                self.assertEqual(found["socket"], expected)

    def test_no_match_diagnostics_identify_parser_template_and_socket_stages_without_payloads(self):
        argv = [str(self.home / "moon-env/bin/python"), "-m", "moonraker", "-d", str(self.data)]
        for body, reason in (("[server]\nklippy_uds_address: {secrets.private_socket}\n", "socket expression is unsupported"),
                             ("[server]\n[include missing.cfg]\n", "include pattern has no matches")):
            with self.subTest(reason=reason):
                self.moon_config.write_text(body)
                notes = []
                self.assertIsNone(updater._moon_command(argv, self.home, installer.uninstall_api(), pid=77,
                                                       diagnostics=notes))
                self.assertIn(reason, "\n".join(notes))
                self.assertNotIn("secrets.private_socket", "\n".join(notes))
        self.moon_config.write_text("[server]\nklippy_uds_address: %s\n" % self.selected["socket"])
        self.refresh()
        selected = {**self.selected, "socket": self.home / "wanted\nsocket"}
        with self.assertRaises(installer.InstallError) as stopped:
            updater.discover_moonraker(selected, installer.uninstall_api(), instances=[self.moon])
        message = str(stopped.exception)
        self.assertIn("Selected Klipper config=", message)
        self.assertIn("PID=77", message)
        self.assertIn("socket mismatch", message)
        self.assertIn("wanted\\nsocket", message)
        self.assertNotIn("wanted\nsocket", message)

    def test_proc_rejections_report_unreadable_environment_or_wrong_uid_without_secret_errors(self):
        proc = self.home / "synthetic-proc"
        process = proc / "77"
        (process / "cwd").mkdir(parents=True)
        units = self.home / "empty-system-units"
        units.mkdir()
        (process / "cmdline").write_bytes(b"python\0-m\0moonraker\0-d\0" + str(self.data).encode() + b"\0")
        uid = process.stat().st_uid
        with mock.patch.object(updater.os, "getuid", return_value=uid, create=True), \
                mock.patch.object(updater, "_path_environment", side_effect=PermissionError("SECRET_VALUE must not escape")):
            found = self.instance_discovery(installer.uninstall_api(), proc, [units])
        with self.assertRaises(installer.InstallError) as stopped:
            updater.discover_moonraker(self.selected, installer.uninstall_api(), instances=found)
        self.assertIn("bounded path environment unavailable (PermissionError)", str(stopped.exception))
        self.assertNotIn("SECRET_VALUE", str(stopped.exception))
        with mock.patch.object(updater.os, "getuid", return_value=uid + 1, create=True), \
                mock.patch.object(Path, "read_bytes", side_effect=AssertionError("foreign process arguments must not be read")), \
                mock.patch.object(updater, "_path_environment") as environment:
            found = self.instance_discovery(installer.uninstall_api(), proc, [units])
            environment.assert_not_called()
        with self.assertRaises(installer.InstallError) as stopped:
            updater.discover_moonraker(self.selected, installer.uninstall_api(), instances=found)
        self.assertIn("skipped 1 processes owned by other UIDs", str(stopped.exception))

    def test_environment_based_instances_still_require_matching_socket_unique_pid_and_stable_inputs(self):
        argv = [str(self.home / "moon-env/bin/python"), "-m", "moonraker"]
        environment = {"HOME": str(self.home), "MOONRAKER_DATA_PATH": str(self.data)}
        found = updater._moon_command(argv, self.home, installer.uninstall_api(), pid=77,
                                      service="moonraker-test.service", scope="system", start="100",
                                      path_environment=environment, owner_home=str(self.home))
        other = {**found, "pid": 78, "socket": self.home / "other-klippy.sock"}
        self.assertIs(updater.discover_moonraker(self.selected, installer.uninstall_api(), instances=[found, other]), found)
        with self.assertRaisesRegex(installer.InstallError, "multiple live"):
            updater.discover_moonraker(self.selected, installer.uninstall_api(), instances=[found, {**other, "socket": found["socket"]}])
        with self.assertRaises(installer.InstallError):
            updater.discover_moonraker(self.selected, installer.uninstall_api(), self.home / "unrelated.conf", [found])
        changes = ({**found, "path_environment": {**environment, "HOME": str(self.home / "changed-home")}},
                   {**found, "cwd": str(self.home / "changed-working-directory")})
        for changed in changes:
            with mock.patch.object(updater, "discover_moonraker", return_value=changed):
                with self.assertRaisesRegex(installer.InstallError, "changed"):
                    updater.assert_identity(found, self.selected, installer.uninstall_api())

    def test_literal_loaded_service_reuses_verified_live_environment_path_context(self):
        argv = [str(self.home / "moon-env/bin/python"), "-m", "moonraker"]
        found = updater._moon_command(argv, self.home, installer.uninstall_api(), pid=77,
                                      service="moonraker-test.service", scope="system", start="100",
                                      path_environment={"HOME": str(self.home), "MOONRAKER_DATA_PATH": str(self.data)},
                                      owner_home=str(self.home))
        properties = dict(ActiveState="active", SubState="running", MainPID="77", User="fixture",
                          WorkingDirectory=str(self.home),
                          ExecStart="{ path=%s ; argv[]=%s ; ignore_errors=no ; }" % (
                              argv[0], shlex.join(argv)))
        with mock.patch.object(updater, "discover_moonraker", return_value=found), \
                mock.patch.object(updater, "_current_users", return_value={"fixture"}), \
                mock.patch.object(installer, "read_service_unit", return_value=properties):
            identity = updater.service_identity(found, self.selected, installer.uninstall_api(), "systemctl")
        self.assertEqual(identity["instance"]["config"], str(self.moon_config))
        self.assertEqual(identity["instance"]["path_environment"], found["path_environment"])


    def test_new_direct_registration_adds_one_missing_base_and_preserves_original_bytes(self):
        original = self.moon_config.read_bytes().replace(b"[update_manager]\r\n", b"").replace(b"[update_manager]\n", b"")
        original += b"[unrelated]\nkeep: yes\n# operator comment\n"
        self.moon_config.write_bytes(original)
        self.refresh()
        plan = self.updater_plan()
        rendered = plan["changes"][self.moon_config]["data"]
        self.assertEqual(rendered, original + updater.direct_suffix(plan["repo"], True, True))
        self.assertEqual(rendered.count(b"[update_manager]\n"), 1)
        self.assertEqual(rendered.count(b"[update_manager accel_homing]\n"), 1)


    def test_new_direct_registration_does_not_create_a_file_matched_by_existing_wildcards(self):
        self.moon_config.write_text(self.moon_config.read_text() + "[include *update.cfg]\n[include accel_*.cfg]\n")
        (self.moon_config.parent / "otherupdate.cfg").write_text("[existing]\n")
        (self.moon_config.parent / "accel_other.cfg").write_text("[another]\n")
        self.refresh()
        plan = self.updater_plan()
        self.assertIn(self.moon_config, plan["changes"])
        self.assertNotIn(self.moon_config.parent / updater.INCLUDE, plan["changes"])
        self.assertNotIn(b"[include accel_homing_update.cfg]", plan["changes"][self.moon_config]["data"])

    def test_concurrent_moonraker_main_edit_is_not_adopted_or_overwritten(self):
        plan = self.updater_plan()
        current = self.moon_config.read_bytes() + b"[operator]\nkeep: yes\n"
        self.moon_config.write_bytes(current)
        with self.assertRaisesRegex(installer.InstallError, "changed after planning"):
            installer.apply_plan(plan["changes"], self.config, selected=self.selected, updater=plan)
        self.assertEqual(self.moon_config.read_bytes(), current)
        self.assertFalse((self.config.parent / ".accel_homing_backups").exists())


    def test_no_moonraker_ambiguous_and_wrong_explicit_instance_do_not_guess(self):
        for instances, explicit in (([], None), ([self.moon, {**self.moon, "pid": 78}], None),
                                     ([self.moon], self.home / "different.conf")):
            with self.subTest(instances=len(instances), explicit=explicit):
                with self.assertRaises(installer.InstallError):
                    updater.discover_moonraker(self.selected, installer.uninstall_api(), explicit, instances)

    def test_combined_transaction_failure_restores_printer_and_moonraker_files(self):
        plan = self.updater_plan()
        printer, moon = self.config.read_bytes(), self.moon_config.read_bytes()
        def fail(path, state):
            if path == self.config.parent / updater.RECEIPT and state["kind"] == "file":
                raise OSError("synthetic updater receipt write failure")
            installer.write_state(path, state)
        changes = {path: value for path, value in self.plan()[0].items() if value["kind"] != "symlink"}
        changes.update(plan["changes"])
        with self.assertRaisesRegex(installer.InstallError, "restored"):
            installer.apply_plan(changes, self.config, writer=fail, selected=self.selected, updater=plan)
        self.assertEqual(self.config.read_bytes(), printer)
        self.assertEqual(self.moon_config.read_bytes(), moon)
        self.assertFalse((self.moon_config.parent / updater.INCLUDE).exists())


    def test_loaded_wrong_instance_and_wrong_user_service_are_rejected(self):
        properties = dict(ActiveState="inactive", SubState="dead", MainPID="0", User="fixture",
                          ExecStart="{ argv[]=%s %s -d %s ; ignore_errors=no ; }" % (
                              self.moon["executable"], self.home / "moonraker/moonraker.py", self.home / "other-data"))
        stopped = {**self.moon, "pid": None, "start_time": None}
        with mock.patch.object(updater, "discover_moonraker", return_value=stopped), \
                mock.patch.object(updater, "_current_users", return_value={"fixture"}), \
                mock.patch.object(installer, "read_service_unit", return_value=properties):
            with self.assertRaisesRegex(installer.InstallError, "effective"):
                updater.service_identity(stopped, self.selected, installer.uninstall_api(), "systemctl")
        with mock.patch.object(updater, "_current_users", return_value={"actual-user"}), \
                mock.patch.object(installer, "read_service_unit", return_value=properties):
            with self.assertRaisesRegex(installer.InstallError, "not owned"):
                updater.service_identity(stopped, self.selected, installer.uninstall_api(), "systemctl")

    @contextlib.contextmanager
    def main_fixture(self, activation_error=False, transaction_error=False):
        shutil.copyfile(installer_tests.DIRECTORY / "uninstaller.py", self.source / "uninstaller.py")
        events, links = [], {}
        real_apply, real_snapshot, real_write = installer.apply_plan, installer.snapshot, installer.write_state
        def module(path):
            return Path(path).parent == self.extras and Path(path).name in installer.MODULES
        def snapshot(path):
            return dict(links.get(Path(path), {"kind": "absent"})) if module(path) else real_snapshot(path)
        def write(path, state):
            if module(path):
                links[Path(path)] = dict(state)
            else:
                real_write(path, state)
        def apply(*args, **kwargs):
            events.append("transaction")
            if transaction_error:
                raise installer.InstallError("synthetic combined transaction failure")
            kwargs["writer"] = write
            return real_apply(*args, **kwargs)
        def source(plan, api, guard):
            guard()
            events.append("candidate")
            plan["candidate"] = self.source
            return self.source
        def publish(plan, api, guard):
            guard()
            events.append("publish")
        def activate(*args):
            events.append("moonraker")
            if activation_error:
                raise installer.InstallError("files installed; Moonraker activation pending")
        with mock.patch.object(installer, "__file__", str(self.source / "installer.py")), \
                mock.patch.object(installer.os, "geteuid", return_value=1000, create=True), \
                mock.patch.object(installer, "discover_target", return_value=dict(self.selected)), \
                mock.patch.object(installer, "proc_instances", return_value=[]), \
                mock.patch.object(installer, "service_instances", return_value=[]), \
                mock.patch.object(installer, "prepare_restart", return_value={"klippy": "guarded"}), \
                mock.patch.object(installer, "assert_restart_safe"), \
                mock.patch.object(installer, "prepare_plotting", side_effect=lambda *a, **k: events.append("plotting")), \
                mock.patch.object(installer, "activate_install", side_effect=lambda *a: events.append("klipper")), \
                mock.patch.object(installer, "apply_plan", side_effect=apply), \
                mock.patch.object(installer, "snapshot", side_effect=snapshot), \
                mock.patch.object(updater, "prepare_restart", return_value={"moon": "guarded"}), \
                mock.patch.object(updater, "assert_restart_safe"), \
                mock.patch.object(updater, "prepare_source", side_effect=source), \
                mock.patch.object(updater, "publish_source", side_effect=publish), \
                mock.patch.object(updater, "restore_publication", side_effect=lambda *a: events.append("restore-source")), \
                mock.patch.object(updater, "activate", side_effect=activate), \
                contextlib.redirect_stdout(io.StringIO()):
            yield events, links

    def test_main_validates_candidate_then_publishes_and_activates_exactly_moonraker_before_klipper(self):
        with self.main_fixture() as (events, links):
            self.assertEqual(installer.main([]), 0)
        self.assertEqual(events, ["candidate", "plotting", "publish", "transaction", "moonraker", "klipper"])
        for name in installer.MODULES:
            self.assertEqual(links[self.extras / name]["target"], str(updater.managed_path(self.selected) / name))
        self.assertIn("[update_manager accel_homing]", self.moon_config.read_text())
        self.assertFalse((self.moon_config.parent / updater.INCLUDE).exists())

    def test_main_moonraker_activation_failure_keeps_installed_files_and_never_activates_klipper(self):
        with self.main_fixture(activation_error=True) as (events, links):
            with self.assertRaisesRegex(installer.InstallError, "Moonraker activation pending"):
                installer.main([])
        self.assertEqual(events, ["candidate", "plotting", "publish", "transaction", "moonraker"])
        self.assertEqual(len(links), len(installer.MODULES))
        self.assertIn("[update_manager accel_homing]", self.moon_config.read_text())
        self.assertFalse((self.moon_config.parent / updater.INCLUDE).exists())

    def test_main_transaction_failure_restores_source_before_any_activation(self):
        with self.main_fixture(transaction_error=True) as (events, links):
            with self.assertRaisesRegex(installer.InstallError, "combined transaction failure"):
                installer.main([])
        self.assertEqual(events, ["candidate", "plotting", "publish", "transaction", "restore-source"])
        self.assertEqual(links, {})


    @contextlib.contextmanager
    def migration_links(self):
        """Keep migration fixtures independent of native symlink privileges."""
        links = {}
        real_snapshot, real_write = installer.snapshot, installer.write_state
        real_exists, real_symlink, real_readlink = Path.exists, Path.is_symlink, os.readlink
        real_canonical = installer.canonical
        def module(path):
            return Path(path).parent == self.extras and Path(path).name in installer.MODULES
        def snapshot(path):
            return dict(links.get(Path(path), {"kind": "absent"})) if module(path) else real_snapshot(path)
        def write(path, state):
            if module(path):
                links[Path(path)] = dict(state)
            else:
                real_write(path, state)
        def exists(path):
            return links.get(Path(path), {}).get("kind", "absent") != "absent" if module(path) else real_exists(path)
        def is_symlink(path):
            return links.get(Path(path), {}).get("kind") == "symlink" if module(path) else real_symlink(path)
        def readlink(path, *args, **kwargs):
            if module(path) and links.get(Path(path), {}).get("kind") == "symlink":
                return links[Path(path)]["target"]
            return real_readlink(path, *args, **kwargs)
        def canonical(path, base=None):
            if module(path) and links.get(Path(path), {}).get("kind") == "symlink":
                return real_canonical(links[Path(path)]["target"], Path(path).parent)
            return real_canonical(path, base)
        with mock.patch.object(installer, "snapshot", side_effect=snapshot), \
                mock.patch.object(installer, "canonical", side_effect=canonical), \
                mock.patch.object(Path, "exists", new=exists), \
                mock.patch.object(Path, "is_symlink", new=is_symlink), \
                mock.patch.object(os, "readlink", side_effect=readlink), \
                mock.patch.object(installer, "proc_instances", return_value=[]), \
                mock.patch.object(installer, "service_instances", return_value=[]):
            yield links, write


    def migration_upgrade(self, write):
        previous = self.source
        self.source = self.home / "next-snapshot"
        shutil.copytree(previous, self.source)
        return installer.apply_plan(self.plan("--replace-existing-links")[0], self.config,
                                    writer=write, selected=self.selected)


    def test_install_and_activation_write_only_current_metadata_without_backups(self):
        plan = self.updater_plan()
        self.assertIsNone(self.install_config(plan))
        receipt = json.loads((self.config.parent / updater.RECEIPT).read_text())
        self.assertEqual(receipt["version"], 2)
        self.assertEqual(receipt["commit"], self.commit)
        self.assertFalse({"append", "before", "after", "changes", "history", "migration_history"} & set(receipt))
        path = updater.record_activation(plan, self.selected, installer.uninstall_api(), None, "skipped")
        self.assertEqual(path, self.config.parent / updater.ACTIVATION_RECEIPT)
        self.assertEqual(json.loads(path.read_text())["status"], "skipped")
        self.assertFalse((self.config.parent / ".accel_homing_backups").exists())
        self.assertFalse(any(self.home.rglob("manifest.json")))

    def test_obsolete_and_malformed_history_and_receipt_do_not_block_current_install(self):
        old = self.config.parent / ".accel_homing_backups" / "broken"
        old.mkdir(parents=True)
        manifest = old / "manifest.json"
        manifest.write_text("not a transaction")
        receipt = self.config.parent / updater.RECEIPT
        for obsolete in ("broken JSON", json.dumps({"origin": updater.LEGACY_ORIGIN, "append": "obsolete"})):
            with self.subTest(receipt=obsolete):
                receipt.write_text(obsolete)
                with mock.patch.object(installer, "load_uninstaller", side_effect=AssertionError("history was read")):
                    plan = self.updater_plan()
                    args = updater.migration_args(self.selected, installer.arguments([]), installer.uninstall_api())
                    self.assertFalse(getattr(args, "_legacy_migration", False))
                self.assertFalse(plan["legacy_migration"])
                self.assertIn(receipt, plan["changes"])
                self.assertEqual(manifest.read_text(), "not a transaction")

    def test_rerun_preserves_operator_stanza_and_base_edits(self):
        self.install_config(self.updater_plan())
        text = self.moon_config.read_text().replace("# Managed by", "# Operator edited; originally managed by")
        text = text.replace("channel: dev", "channel: stable\nrefresh_interval: 73")
        text = text.replace("primary_branch: main", "primary_branch: operator-branch")
        text = text.replace("[update_manager]\n", "[update_manager]\nenable_auto_refresh: False\n")
        self.moon_config.write_text(text)
        self.refresh()
        plan = self.updater_plan()
        self.assertNotIn(self.moon_config, plan["changes"])
        self.install_config(plan)
        self.assertEqual(self.moon_config.read_text(), text)

    def test_rerun_uses_current_custom_include_layout_and_semantic_path(self):
        self.install_config(self.updater_plan())
        include = self.moon_config.parent / "operator-updates.cfg"
        custom = updater.stanza(self.source).replace(
            ("path: %s" % self.source).encode(), b"path: ~/accel_homing/../accel_homing")
        custom += b"# keep this included comment\n[history]\nenabled: True\n"
        include.write_bytes(custom)
        main = updater.remove_registration(self.moon_config.read_bytes(), self.moon_config, installer.uninstall_api())
        self.moon_config.write_bytes(main + b"\n[include operator-updates.cfg]\n")
        self.refresh()
        self.moon["path_environment"] = {"HOME": str(self.home)}
        plan = self.updater_plan()
        self.assertNotIn(include, plan["changes"])
        self.assertNotIn(self.moon_config, plan["changes"])
        self.assertEqual(plan["context"]["registration_path"], str(include))
        self.assertEqual(include.read_bytes(), custom)

    def test_semantically_equal_literal_path_and_origin_are_preserved(self):
        literal = self.source / ".." / self.source.name
        custom = updater.stanza(self.source).replace(str(self.source).encode(), str(literal).encode())
        custom = custom.replace(updater.ORIGIN.encode(), updater.ORIGIN[:-4].encode() + b"/")
        self.moon_config.write_bytes(self.moon_config.read_bytes() + custom)
        self.refresh()
        self.assertNotIn(self.moon_config, self.updater_plan()["changes"])

    def test_foreign_or_custom_registration_is_never_replaced_even_with_legacy_links(self):
        original = self.moon_config.read_bytes()
        for old, new in ((str(self.source), str(self.home / "custom-clone")),
                         (updater.ORIGIN, "https://example.invalid/custom.git"),
                         ("type: git_repo", "type: web")):
            with self.subTest(value=new), self.migration_links() as (links, _write):
                links[self.extras / installer.MODULES[0]] = dict(
                    kind="symlink", target=str(self.home / "MISC/Beacon_Accel_Homing" / installer.MODULES[0]))
                self.moon_config.write_bytes(original + updater.stanza(self.source).replace(old.encode(), new.encode()))
                self.refresh()
                with self.assertRaisesRegex(installer.InstallError, "conflicting existing"):
                    self.updater_plan()
                self.assertFalse((self.config.parent / ".accel_homing_backups").exists())

    def test_tilde_registration_requires_verified_matching_moonraker_home(self):
        self.moon_config.write_bytes(self.moon_config.read_bytes() + updater.stanza(self.source).replace(
            str(self.source).encode(), b"~/accel_homing"))
        self.refresh()
        with self.assertRaisesRegex(installer.InstallError, "live home is unavailable"):
            self.updater_plan()
        self.moon["path_environment"] = {"HOME": str(self.home / "someone-else")}
        with self.assertRaisesRegex(installer.InstallError, "does not identify"):
            self.updater_plan()
        self.moon["path_environment"] = {}
        self.moon["owner_home"] = str(self.home)
        self.assertNotIn(self.moon_config, self.updater_plan()["changes"])

    def test_missing_registration_is_recreated_without_old_text_or_receipt_checks(self):
        self.install_config(self.updater_plan())
        current = updater.remove_registration(self.moon_config.read_bytes(), self.moon_config, installer.uninstall_api())
        self.moon_config.write_bytes(current + b"# operator removed the old registration\n")
        self.refresh()
        self.install_config(self.updater_plan())
        self.assertEqual(self.moon_config.read_text().count("[update_manager accel_homing]"), 1)
        self.assertIn("# operator removed the old registration", self.moon_config.read_text())

    def test_current_misc_registration_migrates_in_place_and_preserves_operator_options(self):
        include = self.moon_config.parent / "custom-update.cfg"
        legacy = updater.stanza(self.home / "old-MISC").replace(updater.ORIGIN.encode(), updater.LEGACY_ORIGIN.encode())
        legacy = legacy.replace(b"primary_branch: main", ("primary_branch: " + updater.LEGACY_BRANCH).encode())
        legacy = legacy.replace(b"channel: dev", b"channel: stable\n# user note\nrefresh_interval: 72")
        legacy += b"\n[history]\nenabled: True\n"
        include.write_bytes(legacy)
        self.moon_config.write_bytes(self.moon_config.read_bytes() + b"[include custom-update.cfg]\n")
        self.refresh()
        args = updater.migration_args(self.selected, installer.arguments([]), installer.uninstall_api())
        self.assertTrue(args._legacy_migration)
        plan = self.updater_plan()
        self.assertTrue(plan["legacy_migration"])
        self.assertNotIn(self.moon_config, plan["changes"])
        data = plan["changes"][include]["data"]
        self.assertIn(updater.ORIGIN.encode(), data)
        self.assertIn(("path: " + str(self.source)).encode(), data)
        self.assertIn(b"channel: stable\n# user note\nrefresh_interval: 72", data)
        self.assertIn(b"[history]\nenabled: True", data)
        self.assertNotIn(b"accel_homing previous:", data)
        self.install_config(plan)
        self.assertFalse(self.updater_plan()["legacy_migration"])

    def test_current_legacy_link_is_detected_without_receipt_or_history(self):
        with self.migration_links() as (links, _write):
            path = self.extras / installer.MODULES[0]
            links[path] = dict(kind="symlink", target=str(self.home / "missing/Beacon_Accel_Homing" / path.name))
            args = updater.migration_args(self.selected, installer.arguments([]), installer.uninstall_api())
            self.assertTrue(args.replace_existing_links)
            self.assertEqual(args._migration_expected[path], links[path])
            self.assertTrue(self.updater_plan()["legacy_migration"])

    def test_uninstall_removes_current_edited_registration_only_and_keeps_include(self):
        include = self.moon_config.parent / "operator-update.cfg"
        custom = (b"# operator header\n" + updater.stanza(self.source).replace(b"channel: dev", b"channel: stable")
                  + b"# keep comment\n[history]\nenabled: True\n")
        include.write_bytes(custom)
        main = self.moon_config.read_bytes() + b"[include operator-update.cfg]\n[unrelated]\nkeep: yes\n"
        self.moon_config.write_bytes(main)
        self.refresh()
        self.install_config(self.updater_plan())
        updater.record_activation(self.updater_plan(), self.selected, installer.uninstall_api(), None, "skipped")
        old = self.config.parent / ".accel_homing_backups"
        old.mkdir()
        (old / "garbage").write_text("untouched")
        with mock.patch.object(installer, "load_uninstaller", side_effect=AssertionError("history was read")):
            plan = updater.make_uninstall_plan(self.selected, installer.uninstall_api())
        self.assertEqual(set(plan["expected"]), {str(path) for path in (
            self.moon_config, include, self.config.parent / updater.RECEIPT,
            self.config.parent / updater.ACTIVATION_RECEIPT)})
        installer.apply_plan(plan["changes"], self.config, selected=self.selected)
        self.assertEqual(self.moon_config.read_bytes(), main)
        self.assertNotIn(b"[update_manager accel_homing]", include.read_bytes())
        self.assertIn(b"# keep comment\n[history]\nenabled: True", include.read_bytes())
        self.assertTrue(include.exists())
        self.assertFalse((self.config.parent / updater.RECEIPT).exists())
        self.assertFalse((self.config.parent / updater.ACTIVATION_RECEIPT).exists())
        self.assertEqual((old / "garbage").read_text(), "untouched")

    def test_uninstall_refuses_concurrent_registration_edit(self):
        self.install_config(self.updater_plan())
        plan = updater.make_uninstall_plan(self.selected, installer.uninstall_api())
        edited = self.moon_config.read_bytes() + b"# concurrent operator edit\n"
        self.moon_config.write_bytes(edited)
        with self.assertRaisesRegex(installer.InstallError, "changed"):
            installer.apply_plan(plan["changes"], self.config, selected=self.selected)
        self.assertEqual(self.moon_config.read_bytes(), edited)

    def test_edit_after_identity_check_cannot_be_adopted_as_planning_baseline(self):
        self.install_config(self.updater_plan())
        original = self.moon_config.read_bytes()
        assert_identity = updater.assert_identity
        def edit_after_check(*args, **kwargs):
            assert_identity(*args, **kwargs)
            self.moon_config.write_bytes(original + b"# concurrent edit after identity check\n")
        for operation in (self.updater_plan,
                          lambda: updater.make_uninstall_plan(self.selected, installer.uninstall_api())):
            with self.subTest(operation=operation):
                self.moon_config.write_bytes(original)
                self.refresh()
                with mock.patch.object(updater, "assert_identity", side_effect=edit_after_check), \
                        self.assertRaisesRegex(installer.InstallError, "changed after planning"):
                    operation()
                self.assertEqual(self.moon_config.read_bytes(), original + b"# concurrent edit after identity check\n")

    def test_uninstall_without_history_preserves_shared_base_and_is_idempotent(self):
        self.moon_config.write_text(self.moon_config.read_text().replace(
            "[update_manager]\n", "[update_manager]\nrefresh_interval: 123\n"))
        self.refresh()
        self.install_config(self.updater_plan())
        plan = updater.make_uninstall_plan(self.selected, installer.uninstall_api())
        installer.apply_plan(plan["changes"], self.config, selected=self.selected)
        self.refresh()
        self.assertIn("[update_manager]\nrefresh_interval: 123", self.moon_config.read_text())
        self.assertEqual(updater.make_uninstall_plan(self.selected, installer.uninstall_api())["changes"], {})
        self.assertFalse((self.config.parent / ".accel_homing_backups").exists())

    def test_activation_retries_pending_failed_and_skipped_without_transaction(self):
        plan = self.updater_plan()
        self.install_config(plan)
        for status in ("skipped", "pending", "failed"):
            with self.subTest(status=status):
                updater.record_activation(plan, self.selected, installer.uninstall_api(), None, status)
                repeated = self.updater_plan()
                self.assertEqual(repeated["changes"], {})
                self.assertTrue(repeated["restart_needed"])
        updater.record_activation(plan, self.selected, installer.uninstall_api(), None, "restarted")
        self.assertFalse(self.updater_plan()["restart_needed"])
        self.moon["start_time"] = "300"
        self.assertTrue(self.updater_plan()["restart_needed"])

    def test_actual_activation_writes_fixed_receipt_and_does_not_repeat_restart(self):
        plan = self.updater_plan()
        self.install_config(plan)
        old = updater._fingerprint(plan["moon"])
        restart = dict(sudo="sudo", systemctl="systemctl", identity={"instance": old},
                       service_command=["systemctl", "--system", "--no-ask-password", "restart", "--", "moonraker-test.service"])
        new = {"instance": {**old, "pid": 88, "start_time": "200"}, "unit": {}}
        with mock.patch.object(updater, "assert_restart_safe"), \
                mock.patch.object(updater, "service_identity", return_value=new), \
                mock.patch.object(installer, "service_command", return_value=types.SimpleNamespace(returncode=0)) as command, \
                contextlib.redirect_stdout(io.StringIO()):
            updater.activate(plan, self.selected, restart, installer.uninstall_api(), None)
        self.assertEqual(command.call_args.args[0], ["sudo", "-n", "-u", "root", "--"] + restart["service_command"])
        self.moon.update(pid=88, start_time="200")
        receipt = json.loads((self.config.parent / updater.ACTIVATION_RECEIPT).read_text())
        self.assertEqual(receipt["status"], "restarted")
        self.assertNotIn("identity", receipt)
        self.assertFalse(self.updater_plan()["restart_needed"])
        self.assertFalse((self.config.parent / ".accel_homing_backups").exists())

    def test_real_restart_preparation_keeps_exact_service_permission_check(self):
        identity = {"instance": updater._fingerprint(self.moon)}
        with mock.patch.object(updater.shutil, "which", side_effect=lambda name: "/usr/bin/" + name), \
                mock.patch.object(updater, "service_identity", return_value=identity), \
                mock.patch.object(installer, "restart_permission") as permission:
            plan = updater.prepare_restart(self.moon, self.selected, installer.uninstall_api())
        self.assertEqual(plan["service_command"], ["/usr/bin/systemctl", "--system", "--no-ask-password",
                                                   "restart", "--", "moonraker-test.service"])
        permission.assert_called_once_with(plan, acquire=True)


class IsolatedGitSourceTests(unittest.TestCase):
    """Real Git writes only inside disposable synthetic repositories."""
    plan = installer_tests.HostFixture.plan
    refresh = UpdaterFixture.refresh
    migration_links = UpdaterFixture.migration_links

    def run_git(self, path, *args):
        result = subprocess.run(["git", "-C", str(path), *args], capture_output=True, text=True,
                                env={**os.environ, "GIT_TERMINAL_PROMPT": "0"}, timeout=20)
        if result.returncode:
            raise AssertionError(result.stderr)
        return result.stdout.strip()

    def setUp(self):
        if shutil.which("git") is None:
            self.skipTest("git is unavailable")
        installer_tests.HostFixture.setUp(self)
        installer.load_updater.return_value = updater
        home = mock.patch.object(updater.Path, "home", return_value=self.home)
        home.start()
        self.addCleanup(home.stop)
        for function in ("proc_instances", "service_instances"):
            patch = mock.patch.object(installer, function, return_value=[])
            patch.start()
            self.addCleanup(patch.stop)
        self.selected["socket"] = self.home / "klippy.sock"
        self.data = self.home / "printer_data_moon"
        self.moon_config = self.data / "config/moonraker.conf"
        self.moon_config.parent.mkdir(parents=True)
        self.moon_config.write_text("[server]\nklippy_uds_address: %s\n[update_manager]\n" % self.selected["socket"])
        self.refresh()
        moon = mock.patch.object(updater, "moonraker_instances", side_effect=lambda api: [self.moon])
        moon.start()
        self.addCleanup(moon.stop)
        self.remote = self.home / "synthetic-remote"
        self.remote.mkdir()
        self.run_git(self.remote, "init", "-b", "main")
        self.run_git(self.remote, "config", "user.name", "Synthetic Test")
        self.run_git(self.remote, "config", "user.email", "fixture@example.invalid")
        self.run_git(self.remote, "config", "core.autocrlf", "false")
        required = tuple(installer.MODULES) + ("install.sh", "uninstall.sh", "installer.py", "uninstaller.py",
                                               "accel_homing_updater.py", "LICENSE", "NOTICE")
        for name in required:
            (self.remote / name).write_bytes((installer_tests.DIRECTORY / name).read_bytes())
        (self.remote / ".gitignore").write_text(".plot-deps/\n__pycache__/\n*.py[cod]\n")
        self.run_git(self.remote, "add", ".")
        self.run_git(self.remote, "commit", "-m", "synthetic initial source")
        self.source = updater.managed_path(self.selected)
        # Clone solely from a disposable file:// repository, then set the exact
        # public origin. Installer operations below never fetch that origin.
        self.run_git(self.home, "-c", "core.autocrlf=false", "clone", "--branch", "main", self.remote.as_uri(), str(self.source))
        self.run_git(self.source, "config", "core.autocrlf", "false")
        self.run_git(self.source, "config", "remote.origin.url", updater.ORIGIN)
        self.commit = self.run_git(self.source, "rev-parse", "HEAD")
        self.api = installer.uninstall_api()

    def updater_plan(self):
        return updater.make_plan(self.source, self.selected, installer.arguments([]), installer.uninstall_api())

    def advance_clone(self):
        (self.remote / installer.MODULES[0]).write_bytes(
            (self.remote / installer.MODULES[0]).read_bytes() + b"\n# synthetic next Git revision\n")
        self.run_git(self.remote, "add", ".")
        self.run_git(self.remote, "commit", "-m", "synthetic update")
        second = self.run_git(self.remote, "rev-parse", "HEAD")
        self.run_git(self.source, "fetch", self.remote.as_uri(), "main:refs/remotes/origin/main")
        self.run_git(self.source, "merge", "--ff-only", "origin/main")
        return second

    def install(self, write):
        plan = self.updater_plan()
        args = updater.migration_args(self.selected, installer.arguments([]), installer.uninstall_api())
        changes = installer.make_plan(self.source, self.selected, args)[0]
        changes.update(plan["changes"])
        backup = installer.apply_plan(changes, self.config, writer=write, selected=self.selected, updater=plan)
        self.refresh()
        return plan, backup

    def test_native_clone_without_manifest_is_reused_without_git_mutations_or_source_backups(self):
        plan = self.updater_plan()
        self.assertFalse((self.source / ".source.json").exists())
        self.assertEqual(updater.source_commit(self.source, self.api), self.commit)
        before = {str(path.relative_to(self.source)): path.read_bytes()
                  for path in self.source.rglob("*") if path.is_file()}
        calls = []
        real_git = updater.git
        def record(repo, arguments, api, timeout=120):
            calls.append(arguments[0])
            return real_git(repo, arguments, api, timeout)
        with mock.patch.object(updater, "git", side_effect=record):
            self.assertEqual(updater.prepare_source(plan, self.api, lambda: None), self.source)
            updater.publish_source(plan, self.api, lambda: None)
            updater.restore_publication(plan, self.api)
        self.assertFalse(set(calls) & {"clone", "fetch", "checkout", "reset", "stash", "pull"})
        self.assertEqual(before, {str(path.relative_to(self.source)): path.read_bytes()
                                  for path in self.source.rglob("*") if path.is_file()})
        self.assertFalse(any(path.name.startswith(("source-backup-", ".candidate-", "failed-source-"))
                             for path in self.home.iterdir()))

    def test_ignored_python_and_plotting_caches_do_not_dirty_native_source(self):
        for directory, name in ((".plot-deps", "cache.txt"), ("__pycache__", "installer.cpython-313.pyc")):
            (self.source / directory).mkdir()
            (self.source / directory / name).write_bytes(b"private local cache")
        self.assertEqual(updater.source_commit(self.source, self.api), self.commit)
        self.assertEqual(self.run_git(self.source, "status", "--porcelain"), "")

    def test_reinstall_after_git_pull_updates_metadata_and_preserves_checkout_and_links(self):
        with self.migration_links() as (links, write):
            first, result = self.install(write)
            self.assertIsNone(result)
            original_links = copy.deepcopy(links)
            original_config = self.config.read_bytes(), self.moon_config.read_bytes()
            second = self.advance_clone()
            changes = self.updater_plan()["changes"]
            self.assertEqual(set(changes), {self.config.parent / updater.RECEIPT})
            plan, result = self.install(write)
            self.assertIsNone(result)
            self.assertEqual(links, original_links)
            self.assertEqual((self.config.read_bytes(), self.moon_config.read_bytes()), original_config)
            receipt = json.loads((self.config.parent / updater.RECEIPT).read_text())
            self.assertEqual(receipt["commit"], second)
            self.assertEqual(first["context"], plan["context"])
            self.assertEqual(self.run_git(self.source, "rev-parse", "HEAD"), second)
            self.assertFalse((self.config.parent / ".accel_homing_backups").exists())

    def test_current_uninstall_keeps_caller_checkout_after_git_update(self):
        with self.migration_links() as (_links, write):
            self.install(write)
            second = self.advance_clone()
            before = {str(path.relative_to(self.source)): path.read_bytes()
                      for path in self.source.rglob("*") if path.is_file()}
            plan = updater.make_uninstall_plan(self.selected, self.api)
            installer.apply_plan(plan["changes"], self.config, writer=write, selected=self.selected)
            self.assertEqual(before, {str(path.relative_to(self.source)): path.read_bytes()
                                      for path in self.source.rglob("*") if path.is_file()})
            self.assertEqual(self.run_git(self.source, "rev-parse", "HEAD"), second)
            self.assertNotIn("[update_manager accel_homing]", self.moon_config.read_text())
            self.assertFalse((self.config.parent / ".accel_homing_backups").exists())


    def test_dirty_or_foreign_checkout_refuses_before_printer_or_moonraker_writes(self):
        original = self.config.read_bytes(), self.axes.read_bytes(), self.moon_config.read_bytes()
        module = self.source / installer.MODULES[0]
        contents = module.read_bytes()
        for damage in ("tracked_edit", "untracked_file", "origin", "branch", "upstream", "lock"):
            with self.subTest(damage=damage):
                if damage == "tracked_edit":
                    module.write_bytes(contents + b"\n# user edit\n")
                elif damage == "untracked_file":
                    (self.source / "operator.txt").write_text("preserve this file")
                elif damage == "origin":
                    self.run_git(self.source, "config", "remote.origin.url", "https://example.invalid/other.git")
                elif damage == "branch":
                    self.run_git(self.source, "checkout", "-b", "operator-branch")
                elif damage == "upstream":
                    self.run_git(self.source, "branch", "--unset-upstream")
                else:
                    (self.source / ".git/index.lock").write_bytes(b"owned by another Git command")
                try:
                    with self.assertRaises(installer.InstallError):
                        self.updater_plan()
                    self.assertEqual((self.config.read_bytes(), self.axes.read_bytes(), self.moon_config.read_bytes()), original)
                    self.assertFalse((self.config.parent / ".accel_homing_backups").exists())
                finally:
                    if damage == "tracked_edit":
                        module.write_bytes(contents)
                    elif damage == "untracked_file":
                        (self.source / "operator.txt").unlink()
                    elif damage == "origin":
                        self.run_git(self.source, "config", "remote.origin.url", updater.ORIGIN)
                    elif damage == "branch":
                        self.run_git(self.source, "checkout", "main")
                    elif damage == "upstream":
                        self.run_git(self.source, "branch", "--set-upstream-to", "origin/main", "main")
                    else:
                        (self.source / ".git/index.lock").unlink()

    def test_clean_unpushed_source_commit_is_not_adopted_or_reset(self):
        self.run_git(self.source, "config", "user.name", "Synthetic Operator")
        self.run_git(self.source, "config", "user.email", "fixture@example.invalid")
        (self.source / "operator.txt").write_text("preserve this committed customization")
        self.run_git(self.source, "add", ".")
        self.run_git(self.source, "commit", "-m", "operator customization")
        custom = self.run_git(self.source, "rev-parse", "HEAD")
        with self.assertRaises(installer.InstallError):
            self.updater_plan()
        self.assertEqual(self.run_git(self.source, "rev-parse", "HEAD"), custom)
        self.assertEqual((self.source / "operator.txt").read_text(), "preserve this committed customization")

    def test_source_commit_change_after_planning_refuses_without_rolling_back_git(self):
        plan = self.updater_plan()
        second = self.advance_clone()
        with self.assertRaisesRegex(installer.InstallError, "changed after planning"):
            updater.prepare_source(plan, self.api, lambda: None)
        self.assertEqual(self.run_git(self.source, "rev-parse", "HEAD"), second)
        self.assertFalse((self.config.parent / ".accel_homing_backups").exists())

    def test_archive_manifest_is_refused_before_any_native_registration(self):
        marker = self.source / ".source.json"
        marker.write_text(json.dumps(dict(repo="FrankRamm/MISC", commit=self.commit, sha256={})))
        with self.assertRaisesRegex(installer.InstallError, "archive source manifests"):
            self.updater_plan()
        self.assertTrue(marker.exists())
        self.assertFalse((self.config.parent / ".accel_homing_backups").exists())

    def test_other_detected_repo_using_same_clone_is_rejected_without_touching_it(self):
        peer = {**self.selected, "repo": self.home / "another-klipper", "config": self.home / "another.cfg"}
        linked = peer["repo"] / "klippy/extras" / installer.MODULES[0]
        original = Path.is_symlink
        def is_symlink(path):
            return True if path == linked else original(path)
        real_canonical = installer.canonical
        def canonical(path, base=None):
            return self.source / linked.name if path == linked else real_canonical(path, base)
        with mock.patch.object(installer, "proc_instances", return_value=[peer]), \
                mock.patch.object(Path, "is_symlink", new=is_symlink), \
                mock.patch.object(installer, "canonical", side_effect=canonical):
            with self.assertRaisesRegex(installer.InstallError, "another detected Klipper instance"):
                self.updater_plan()
        self.assertFalse(peer["repo"].exists())
        self.assertFalse((self.config.parent / ".accel_homing_backups").exists())


if __name__ == "__main__":
    unittest.main()
