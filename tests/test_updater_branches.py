"""Explicit development branch registration; all hosts and Git repos are fixtures."""
import base64
import copy
import json
from pathlib import Path
import unittest
from unittest import mock

import test_updater as fixtures

installer, updater = fixtures.installer, fixtures.updater
DEV_BRANCH = "codex/accuracy-filters"


class BranchUpdaterTests(unittest.TestCase):
    plan = fixtures.UpdaterFixture.plan
    marker = fixtures.UpdaterFixture.marker
    refresh = fixtures.UpdaterFixture.refresh
    install_config = fixtures.UpdaterFixture.install_config
    legacy_plan = fixtures.UpdaterFixture.legacy_plan

    def setUp(self):
        fixtures.UpdaterFixture.setUp(self)
        # Use the target Linux host's line endings for native config fixtures.
        self.moon_config.write_bytes(self.moon_config.read_bytes().replace(b"\r\n", b"\n"))
        self.refresh()

    def updater_plan(self, branch="main"):
        args = installer.arguments(["--branch", branch])
        return updater.make_plan(self.source, self.selected, args, installer.uninstall_api())

    def uninstall(self):
        with mock.patch.object(installer, "proc_instances", return_value=[]), \
                mock.patch.object(installer, "service_instances", return_value=[]):
            plan = installer.make_uninstall_plan(self.selected)
            installer.apply_uninstall_plan(plan, self.selected)

    def test_explicit_dev_plan_is_read_only_and_records_selected_branch(self):
        original = self.config.read_bytes(), self.moon_config.read_bytes()
        plan = self.updater_plan(DEV_BRANCH)
        text = plan["changes"][self.moon_config]["data"]
        self.assertIn(("primary_branch: %s\n" % DEV_BRANCH).encode(), text)
        self.assertEqual(plan["context"]["branch"], DEV_BRANCH)
        self.assertEqual(plan["source_identity"]["branch"], DEV_BRANCH)
        self.assertEqual((self.config.read_bytes(), self.moon_config.read_bytes()), original)
        self.assertFalse((self.config.parent / ".accel_homing_backups").exists())
        self.assertFalse(plan["receipt"].exists())

    def test_owned_main_dev_main_history_uninstalls_to_original_and_keeps_later_edits(self):
        original = self.moon_config.read_bytes()
        first = self.updater_plan()
        first_backup = self.install_config(first)
        updater.record_activation(first, self.selected, installer.uninstall_api(), first_backup, "restarted")
        main = self.moon_config.read_bytes()
        development = self.updater_plan(DEV_BRANCH)
        self.assertTrue(development["restart_needed"])
        self.assertEqual(development["changes"][self.moon_config]["data"], main.replace(
            b"primary_branch: main\n", ("primary_branch: %s\n" % DEV_BRANCH).encode(), 1))
        dev_backup = self.install_config(development)
        updater.record_activation(development, self.selected, installer.uninstall_api(), dev_backup, "restarted")
        self.assertFalse(self.updater_plan(DEV_BRANCH)["restart_needed"])
        repeated = self.updater_plan()
        self.assertTrue(repeated["restart_needed"])
        self.install_config(repeated)
        self.assertEqual(self.moon_config.read_bytes(), main)
        unrelated = b"\n[operator]\nkeep: yes\n"
        self.moon_config.write_bytes(main + unrelated)
        self.refresh()
        self.uninstall()
        self.assertEqual(self.moon_config.read_bytes(), original + unrelated)
        self.assertFalse(first["receipt"].exists())

    def test_preexisting_native_stanza_without_receipt_gets_reversible_branch_only_edit(self):
        original = self.moon_config.read_bytes() + updater.stanza(self.source)
        self.moon_config.write_bytes(original)
        self.refresh()
        plan = self.updater_plan(DEV_BRANCH)
        self.assertEqual(plan["context"]["append"], "")
        self.assertEqual(plan["changes"][self.moon_config]["data"], original.replace(
            b"primary_branch: main\n", ("primary_branch: %s\n" % DEV_BRANCH).encode(), 1))
        backup = self.install_config(plan)
        manifest = json.loads((backup / "manifest.json").read_text())
        updater.validate_record(manifest, self.selected, installer.uninstall_api())
        later = b"\n[operator]\nkeep: yes\n"
        self.moon_config.write_bytes(self.moon_config.read_bytes() + later)
        self.refresh()
        self.uninstall()
        self.assertEqual(self.moon_config.read_bytes(), original + later)
        self.assertFalse(plan["receipt"].exists())

    def test_branch_transaction_rollback_restores_manual_stanza_and_receipt_absence(self):
        original = self.moon_config.read_bytes() + updater.stanza(self.source)
        self.moon_config.write_bytes(original)
        self.refresh()
        plan = self.updater_plan(DEV_BRANCH)
        backup = self.install_config(plan)
        installer.rollback(backup, selected=self.selected)
        self.assertEqual(self.moon_config.read_bytes(), original)
        self.assertFalse(plan["receipt"].exists())

    def test_manual_branch_switch_and_missing_base_are_one_reversible_transaction(self):
        original = self.moon_config.read_bytes().replace(b"[update_manager]\n", b"")
        original += updater.stanza(self.source)
        self.moon_config.write_bytes(original)
        self.refresh()
        plan = self.updater_plan(DEV_BRANCH)
        self.install_config(plan)
        self.assertEqual(self.moon_config.read_bytes().count(b"[update_manager]\n"), 1)
        self.uninstall()
        self.assertEqual(self.moon_config.read_bytes(), original)

    def test_legacy_include_switch_keeps_layout_and_restores_full_history(self):
        original = self.moon_config.read_bytes()
        first = self.legacy_plan()
        self.install_config(first)
        registered = self.moon_config.read_bytes()
        include = self.moon_config.parent / updater.INCLUDE
        plan = self.updater_plan(DEV_BRANCH)
        self.assertNotIn(self.moon_config, plan["changes"])
        self.assertNotIn("layout", plan["context"])
        self.assertEqual(plan["changes"][include]["data"], updater.stanza(self.source, DEV_BRANCH))
        self.install_config(plan)
        self.assertEqual(self.moon_config.read_bytes(), registered)
        self.assertEqual(include.read_bytes(), updater.stanza(self.source, DEV_BRANCH))
        self.uninstall()
        self.assertEqual(self.moon_config.read_bytes(), original)
        self.assertFalse(include.exists())

    def test_branch_receipt_and_installed_stanza_must_agree_before_update(self):
        self.install_config(self.updater_plan())
        changed = self.moon_config.read_bytes().replace(
            b"primary_branch: main\n", ("primary_branch: %s\n" % DEV_BRANCH).encode())
        self.moon_config.write_bytes(changed)
        self.refresh()
        with self.assertRaisesRegex(installer.InstallError, "changed/missing|conflicting existing"):
            self.updater_plan(DEV_BRANCH)
        self.assertEqual(self.moon_config.read_bytes(), changed)

    def test_record_validator_rejects_unrelated_change_or_foreign_branch(self):
        original = self.moon_config.read_bytes() + updater.stanza(self.source)
        self.moon_config.write_bytes(original)
        self.refresh()
        backup = self.install_config(self.updater_plan(DEV_BRANCH))
        manifest = json.loads((backup / "manifest.json").read_text())
        for attack in ("unrelated", "branch"):
            damaged = copy.deepcopy(manifest)
            if attack == "branch":
                damaged["updater"]["branch"] = "operator-branch"
            else:
                entry = next(item for item in damaged["changes"] if item["path"] == str(self.moon_config))
                text = base64.b64decode(entry["after"]["data"]) + b"\n[unowned]\nvalue: changed\n"
                entry["after"]["data"] = base64.b64encode(text).decode()
            with self.subTest(attack=attack), self.assertRaises(installer.InstallError):
                updater.validate_record(damaged, self.selected, installer.uninstall_api())

    def test_invalid_or_unrecorded_branch_refuses_before_files_change(self):
        original = self.moon_config.read_bytes()
        for branch in ("operator-branch", "main\ninstall_script: malicious", None, ["main"]):
            args = installer.arguments([])
            args.branch = branch
            with self.subTest(branch=branch), self.assertRaisesRegex(installer.InstallError, "unsupported managed branch"):
                updater.make_plan(self.source, self.selected, args, installer.uninstall_api())
        self.assertEqual(self.moon_config.read_bytes(), original)
        self.assertFalse((self.config.parent / updater.RECEIPT).exists())
        self.assertFalse((self.config.parent / ".accel_homing_backups").exists())


class BranchGitIdentityTests(unittest.TestCase):
    setUp = fixtures.IsolatedGitSourceTests.setUp
    plan = fixtures.IsolatedGitSourceTests.plan
    refresh = fixtures.IsolatedGitSourceTests.refresh
    run_git = fixtures.IsolatedGitSourceTests.run_git

    def select_development(self):
        self.run_git(self.remote, "branch", DEV_BRANCH)
        self.run_git(self.source, "fetch", self.remote.as_uri(),
                     DEV_BRANCH + ":refs/remotes/origin/" + DEV_BRANCH)
        self.run_git(self.source, "checkout", "-b", DEV_BRANCH, "--track", "origin/" + DEV_BRANCH)

    def test_actual_dev_checkout_requires_explicit_branch_and_matching_upstream(self):
        self.select_development()
        original = self.config.read_bytes(), self.moon_config.read_bytes()
        with self.assertRaisesRegex(installer.InstallError, "origin/branch/upstream differs"):
            updater.source_commit(self.source, self.api)
        self.assertEqual(updater.source_commit(self.source, self.api, DEV_BRANCH), self.commit)
        plan = updater.make_plan(self.source, self.selected,
                                 installer.arguments(["--branch", DEV_BRANCH]), self.api)
        self.assertEqual(plan["source_identity"]["upstream"], "origin/" + DEV_BRANCH)
        updater.prepare_source(plan, self.api, lambda: None)
        self.assertEqual((self.config.read_bytes(), self.moon_config.read_bytes()), original)
        self.run_git(self.source, "branch", "--set-upstream-to", "origin/main")
        with self.assertRaisesRegex(installer.InstallError, "origin/branch/upstream differs"):
            updater.source_commit(self.source, self.api, DEV_BRANCH)
        self.assertEqual(self.run_git(self.source, "branch", "--show-current"), DEV_BRANCH)

    def test_branch_change_after_planning_is_refused_without_resetting_checkout(self):
        self.select_development()
        plan = updater.make_plan(self.source, self.selected,
                                 installer.arguments(["--branch", DEV_BRANCH]), self.api)
        self.run_git(self.source, "checkout", "main")
        with self.assertRaisesRegex(installer.InstallError, "origin/branch/upstream differs"):
            updater.prepare_source(plan, self.api, lambda: None)
        self.assertEqual(self.run_git(self.source, "branch", "--show-current"), "main")
        self.assertFalse(plan["receipt"].exists())


if __name__ == "__main__":
    unittest.main()
