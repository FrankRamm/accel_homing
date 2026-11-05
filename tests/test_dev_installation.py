"""Unexecuted regression sources for managed main/dev transitions; no live host."""
import base64
import contextlib
import copy
import hashlib
import json
import unittest
from unittest import mock

import test_updater as updater_tests


installer = updater_tests.installer
updater = updater_tests.updater


class DevRegistrationTests(unittest.TestCase):
    setUp = updater_tests.UpdaterFixture.setUp
    plan = updater_tests.UpdaterFixture.plan
    marker = updater_tests.UpdaterFixture.marker
    refresh = updater_tests.UpdaterFixture.refresh
    updater_plan = updater_tests.UpdaterFixture.updater_plan
    install_config = updater_tests.UpdaterFixture.install_config
    legacy_plan = updater_tests.UpdaterFixture.legacy_plan

    @contextlib.contextmanager
    def branch(self, branch):
        with mock.patch.object(updater, "repo_identity", side_effect=lambda source, api: dict(
                commit=self.commit, origin=updater.ORIGIN, branch=branch, upstream="origin/" + branch)):
            yield

    def uninstall(self):
        with mock.patch.object(installer, "proc_instances", return_value=[]), \
                mock.patch.object(installer, "service_instances", return_value=[]):
            plan = installer.make_uninstall_plan(self.selected)
            return installer.apply_uninstall_plan(plan, self.selected)

    def test_default_main_and_explicit_dev_select_matching_updater_branches(self):
        self.assertEqual(self.updater_plan()["context"]["branch"], "main")
        with self.branch("dev"):
            plan = updater.make_plan(self.source, self.selected, installer.arguments(["--branch", "dev"]),
                                     installer.uninstall_api())
            self.assertEqual(plan["context"]["branch"], "dev")
            self.assertIn(b"channel: dev\n", plan["changes"][self.moon_config]["data"])
            self.assertIn(b"primary_branch: dev\n", plan["changes"][self.moon_config]["data"])
            with self.assertRaisesRegex(installer.InstallError, "does not match"):
                updater.make_plan(self.source, self.selected, installer.arguments(["--branch", "main"]),
                                  installer.uninstall_api())
        self.assertEqual(self.updater_plan()["context"]["branch"], "main")

    def test_branch_argument_refuses_unmanaged_recovery_and_arbitrary_branches(self):
        for flags in (("--branch", "feature"), ("--branch", "dev", "--uninstall"),
                      ("--branch", "dev", "--rollback", "/fixture"),
                      ("--branch", "dev", "--no-update-manager")):
            with self.subTest(flags=flags), self.assertRaises(SystemExit):
                installer.arguments(list(flags))

    def test_owned_direct_main_dev_update_uninstall_preserves_unrelated_settings(self):
        original = self.moon_config.read_bytes()
        self.install_config(self.updater_plan())
        installed_config = self.config.read_bytes(), self.axes.read_bytes()
        unrelated = b"\n[operator]\nkeep: yes\n"
        self.moon_config.write_bytes(self.moon_config.read_bytes() + unrelated)
        self.refresh()
        with self.branch("dev"):
            plan = self.updater_plan()
            self.assertEqual(plan["context"]["previous_branch"], "main")
            self.assertEqual(set(plan["changes"]), {self.moon_config, plan["receipt"]})
            backup = self.install_config(plan)
            self.assertEqual((self.config.read_bytes(), self.axes.read_bytes()), installed_config)
            self.assertEqual(self.updater_plan()["changes"], {})
            updater.record_activation(plan, self.selected, installer.uninstall_api(), backup, "restarted")
            activation = json.loads((backup / "moonraker-activation.json").read_text())
            self.assertEqual(activation["registration_sha256"], hashlib.sha256(updater.stanza(self.source, "dev")).hexdigest())
            self.assertFalse(self.updater_plan()["restart_needed"])
            self.commit = "b" * 40
            repeated = self.updater_plan()
            self.assertEqual(set(repeated["changes"]), {repeated["receipt"]})
            self.install_config(repeated)
            undo = self.uninstall()
            uninstall_manifest = json.loads((undo / "manifest.json").read_text())
            updater.validate_record(uninstall_manifest, self.selected, installer.uninstall_api())
            installer.rollback(undo, selected=self.selected)
            self.refresh()
            self.assertIn(b"primary_branch: dev\n", self.moon_config.read_bytes())
            self.uninstall()
        self.assertEqual(self.moon_config.read_bytes(), original + unrelated)
        self.assertFalse((self.config.parent / updater.RECEIPT).exists())

    def test_owned_include_main_dev_update_and_uninstall_keep_historical_layout(self):
        original = self.moon_config.read_bytes()
        self.install_config(self.legacy_plan())
        installed = self.moon_config.read_bytes()
        include = self.moon_config.parent / updater.INCLUDE
        with self.branch("dev"):
            plan = self.updater_plan()
            self.assertEqual(set(plan["changes"]), {include, plan["receipt"]})
            self.install_config(plan)
            self.assertEqual(self.moon_config.read_bytes(), installed)
            self.assertEqual(include.read_bytes(), updater.stanza(self.source, "dev"))
            self.commit = "b" * 40
            self.install_config(self.updater_plan())
            undo = self.uninstall()
            uninstall_manifest = json.loads((undo / "manifest.json").read_text())
            updater.validate_record(uninstall_manifest, self.selected, installer.uninstall_api())
            installer.rollback(undo, selected=self.selected)
            self.refresh()
            self.assertEqual(include.read_bytes(), updater.stanza(self.source, "dev"))
            self.uninstall()
        self.assertEqual(self.moon_config.read_bytes(), original)
        self.assertFalse(include.exists())

    def test_dev_transition_rollback_restores_main_registration_and_receipt(self):
        self.install_config(self.updater_plan())
        main = self.moon_config.read_bytes(), (self.config.parent / updater.RECEIPT).read_bytes()
        with self.branch("dev"):
            backup = self.install_config(self.updater_plan())
            installer.rollback(backup, selected=self.selected)
        self.assertEqual((self.moon_config.read_bytes(), (self.config.parent / updater.RECEIPT).read_bytes()), main)

    def test_branch_switch_does_not_adopt_preexisting_manual_direct_entry(self):
        original = self.moon_config.read_bytes()
        manual = original + updater.stanza(self.source, "main")
        self.moon_config.write_bytes(manual)
        self.refresh()
        self.install_config(self.updater_plan())
        receipt = (self.config.parent / updater.RECEIPT).read_bytes()
        with self.branch("dev"), self.assertRaisesRegex(installer.InstallError, "pre-existing manual"):
            self.updater_plan()
        self.assertEqual(self.moon_config.read_bytes(), manual)
        self.assertEqual((self.config.parent / updater.RECEIPT).read_bytes(), receipt)

    def test_branch_switch_does_not_adopt_preexisting_manual_include_entry(self):
        include = self.moon_config.parent / updater.INCLUDE
        include.write_bytes(updater.stanza(self.source, "main"))
        self.moon_config.write_bytes(self.moon_config.read_bytes() + updater.main_suffix(False, True))
        self.refresh()
        self.install_config(self.updater_plan())
        original = self.moon_config.read_bytes(), include.read_bytes()
        with self.branch("dev"), self.assertRaisesRegex(installer.InstallError, "pre-existing manual"):
            self.updater_plan()
        self.assertEqual((self.moon_config.read_bytes(), include.read_bytes()), original)

    def test_historical_exclude_compatibility_rejects_an_arbitrary_module_subset(self):
        backup = self.install_config(self.updater_plan())
        path = backup / "manifest.json"
        manifest = json.loads(path.read_text())
        entry = next(change for change in manifest["changes"]
                     if change["path"] == str(self.repo / ".git/info/exclude"))
        changed = base64.b64decode(entry["after"]["data"]).replace(b"klippy/extras/accel_homing_charts.py\n", b"")
        entry["after"]["data"] = base64.b64encode(changed).decode()
        path.write_text(json.dumps(manifest))
        with self.assertRaisesRegex(installer.InstallError, "exclude backup contains unowned"):
            installer.load_uninstaller().validate_installation_history(self.selected, installer.uninstall_api())

    def test_forged_transition_or_unrelated_main_edit_is_rejected(self):
        self.install_config(self.updater_plan())
        with self.branch("dev"):
            backup = self.install_config(self.updater_plan())
        manifest = json.loads((backup / "manifest.json").read_text())
        for branch in ("feature", "dev", ["main"]):
            tampered = copy.deepcopy(manifest)
            tampered["updater"]["previous_branch"] = branch
            with self.subTest(branch=branch), self.assertRaises(installer.InstallError):
                updater.validate_record(tampered, self.selected, installer.uninstall_api())
        tampered = copy.deepcopy(manifest)
        entry = next(change for change in tampered["changes"] if change["path"] == str(self.moon_config))
        entry["after"]["data"] = base64.b64encode(base64.b64decode(entry["after"]["data"]) +
                                                   b"\n[unowned]\nunsafe: yes\n").decode()
        with self.assertRaisesRegex(installer.InstallError, "unrelated Moonraker"):
            updater.validate_record(tampered, self.selected, installer.uninstall_api())
        for state, branch in (("before", "dev"), ("after", "main")):
            tampered = copy.deepcopy(manifest)
            entry = next(change for change in tampered["changes"]
                         if change["path"] == str(self.config.parent / updater.RECEIPT))
            receipt = json.loads(base64.b64decode(entry[state]["data"]))
            receipt.pop("previous_branch", None)
            receipt["branch"] = branch
            suffix = updater.direct_suffix(self.source, False, True, branch)
            receipt["append"] = base64.b64encode(suffix).decode()
            entry[state]["data"] = base64.b64encode(json.dumps(receipt).encode()).decode()
            with self.subTest(state=state), self.assertRaisesRegex(installer.InstallError, "unowned updater receipt"):
                updater.validate_record(tampered, self.selected, installer.uninstall_api())


class IsolatedDevGitTests(unittest.TestCase):
    setUp = updater_tests.IsolatedGitSourceTests.setUp
    run_git = updater_tests.IsolatedGitSourceTests.run_git
    refresh = updater_tests.IsolatedGitSourceTests.refresh
    migration_links = updater_tests.IsolatedGitSourceTests.migration_links
    plan = updater_tests.IsolatedGitSourceTests.plan
    updater_plan = updater_tests.IsolatedGitSourceTests.updater_plan
    install = updater_tests.IsolatedGitSourceTests.install

    def select_dev(self):
        self.run_git(self.remote, "branch", "dev")
        self.run_git(self.source, "fetch", self.remote.as_uri(), "dev:refs/remotes/origin/dev")
        self.run_git(self.source, "checkout", "--track", "origin/dev")

    def test_native_dev_identity_requires_matching_remote_tracking_and_keeps_user_edits(self):
        self.select_dev()
        identity = updater.repo_identity(self.source, self.api)
        self.assertEqual((identity["branch"], identity["upstream"]), ("dev", "origin/dev"))
        self.assertEqual(self.updater_plan()["context"]["branch"], "dev")
        self.run_git(self.source, "branch", "--set-upstream-to", "origin/main", "dev")
        with self.assertRaises(installer.InstallError):
            self.updater_plan()
        self.run_git(self.source, "branch", "--set-upstream-to", "origin/dev", "dev")
        customized = self.source / installer.MODULES[0]
        contents = customized.read_bytes() + b"\n# preserve operator customization\n"
        customized.write_bytes(contents)
        with self.assertRaisesRegex(installer.InstallError, "dirty"):
            self.updater_plan()
        self.assertEqual(customized.read_bytes(), contents)
        self.assertEqual(self.run_git(self.source, "branch", "--show-current"), "dev")

    def test_six_module_main_install_upgrades_and_rolls_back_without_switching_git(self):
        modules = installer.MODULES
        self.assertEqual(modules[-1], "accel_homing_center.py")
        with self.migration_links() as (links, write):
            with mock.patch.object(installer, "MODULES", modules[:-1]):
                _main, _first = self.install(write)
            main_links = copy.deepcopy(links)
            main_files = {path: path.read_bytes() for path in (
                self.config, self.axes, self.moon_config, self.config.parent / updater.RECEIPT,
                self.repo / ".git/info/exclude")}
            self.select_dev()
            plan, backup = self.install(write)
            self.assertEqual(plan["context"]["branch"], "dev")
            for path, state in main_links.items():
                self.assertEqual(links[path], state)
            center = self.extras / modules[-1]
            self.assertEqual(links[center]["target"], str(self.source / modules[-1]))
            self.assertIn(b"primary_branch: dev\n", self.moon_config.read_bytes())
            self.assertEqual((self.config.read_bytes(), self.axes.read_bytes()),
                             (main_files[self.config], main_files[self.axes]))
            with mock.patch.object(installer, "write_state", side_effect=write):
                installer.rollback(backup, selected=self.selected)
            self.assertEqual(links[center]["kind"], "absent")
            self.assertEqual({path: path.read_bytes() for path in main_files}, main_files)
            self.assertEqual(self.run_git(self.source, "branch", "--show-current"), "dev")
            self.assertEqual(self.run_git(self.source, "rev-parse", "HEAD"), self.commit)


if __name__ == "__main__":
    unittest.main()
