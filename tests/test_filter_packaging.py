"""Offline six-module/main -> seven-module/development transaction regression."""
import json
from pathlib import Path
import unittest
from unittest import mock

import test_updater as fixtures
import test_updater_branches as branch_fixtures

installer, updater = fixtures.installer, fixtures.updater
DEV_BRANCH = branch_fixtures.DEV_BRANCH


class FilterPackageArgumentTests(unittest.TestCase):
    def test_branch_defaults_and_explicit_development_are_supported(self):
        self.assertEqual(installer.arguments([]).branch, "main")
        self.assertEqual(installer.arguments(["--branch", DEV_BRANCH]).branch, DEV_BRANCH)
        self.assertIn("accel_homing_filters.py", installer.MODULES)
        self.assertEqual(len(installer.LEGACY_MODULES), 6)

    def test_branch_is_not_an_uninstall_rollback_or_unmanaged_option(self):
        for flags in (["--uninstall"], ["--rollback", "/fixture/backup"], ["--no-update-manager"]):
            for branch in ("main", DEV_BRANCH):
                with self.subTest(flags=flags, branch=branch), mock.patch("sys.stderr"), self.assertRaises(SystemExit):
                    installer.arguments(flags + ["--branch", branch])


class FilterPackageHistoryTests(unittest.TestCase):
    setUp = branch_fixtures.BranchUpdaterTests.setUp
    plan = fixtures.UpdaterFixture.plan
    marker = fixtures.UpdaterFixture.marker
    refresh = fixtures.UpdaterFixture.refresh
    main_fixture = fixtures.UpdaterFixture.main_fixture

    def test_unjournaled_native_six_main_adopts_exact_baseline_and_remains_repeatable(self):
        # This matches the supported native layout before any managed journal:
        # canonical source links + a manual main stanza, never MISC/foreign code.
        self.moon_config.write_bytes(self.moon_config.read_bytes() + updater.stanza(self.source))
        exclude = self.repo / ".git/info/exclude"
        exclude.write_bytes(exclude.read_bytes() + ("\n".join(
            "klippy/extras/" + name for name in installer.LEGACY_MODULES) + "\n").encode())
        original = {path: path.read_bytes() for path in (self.config, self.axes, self.moon_config, exclude)}
        self.refresh()
        with self.main_fixture() as (_events, links):
            for name in installer.LEGACY_MODULES:
                links[self.extras / name] = {"kind": "symlink", "target": str(self.source / name)}
            prior_links = {path: dict(state) for path, state in links.items()}
            self.assertEqual(installer.main(["--branch", DEV_BRANCH]), 0)
            history = installer.load_uninstaller().validate_installation_history(self.selected, installer.uninstall_api())
            self.assertEqual(len(history["active"]), 1)
            first = history["active"][0]
            baseline_entries = [entry for entry in first["manifest"]["changes"]
                                if Path(entry["path"]) in prior_links]
            self.assertEqual(len(baseline_entries), 6)
            self.assertTrue(all(entry["before"] == entry["after"] == prior_links[Path(entry["path"])]
                                for entry in baseline_entries))
            self.refresh()
            self.assertEqual(installer.main(["--branch", DEV_BRANCH, "--dry-run"]), 0)
            self.assertEqual(installer.main(["--branch", DEV_BRANCH]), 0)
            self.assertEqual(len(installer.load_uninstaller().validate_migration(self.selected, installer.uninstall_api())), 8)
            real_write = installer.write_state
            def restore(path, state):
                if Path(path).parent == self.extras and Path(path).name in installer.MODULES:
                    links[Path(path)] = dict(state)
                else:
                    real_write(path, state)
            with mock.patch.object(installer, "write_state", side_effect=restore):
                installer.rollback(first["directory"], selected=self.selected)
            for path, state in prior_links.items():
                self.assertEqual(links[path], state)
            self.assertEqual(links[self.extras / "accel_homing_filters.py"], {"kind": "absent"})
            self.assertEqual({path: path.read_bytes() for path in original}, original)
            self.refresh()
            self.assertEqual(installer.main(["--branch", DEV_BRANCH]), 0)
            axis_note, moon_note = b"\n# later native operator axis note\n", b"\n[operator]\nkeep: yes\n"
            self.axes.write_bytes(self.axes.read_bytes() + axis_note)
            self.moon_config.write_bytes(self.moon_config.read_bytes() + moon_note)
            self.refresh()
            plan = installer.make_uninstall_plan(self.selected)
            installer.apply_uninstall_plan(plan, self.selected, writer=restore)
            for path, state in prior_links.items():
                self.assertEqual(links[path], state)
            self.assertEqual(links[self.extras / "accel_homing_filters.py"], {"kind": "absent"})
        self.assertEqual(self.config.read_bytes(), original[self.config])
        self.assertEqual(self.axes.read_bytes(), original[self.axes] + axis_note)
        self.assertEqual(self.moon_config.read_bytes(), original[self.moon_config] + moon_note)
        self.assertEqual(exclude.read_bytes(), original[exclude])
        self.assertFalse((self.config.parent / updater.RECEIPT).exists())

    def test_six_module_main_to_seven_module_dev_then_uninstall_keeps_later_edits(self):
        paths = (self.config, self.axes, self.moon_config, self.repo / ".git/info/exclude")
        original = {path: path.read_bytes() for path in paths}
        receipt = self.config.parent / updater.RECEIPT
        with self.main_fixture() as (_events, links):
            with mock.patch.object(installer, "MODULES", installer.LEGACY_MODULES):
                self.assertEqual(installer.main([]), 0)
            self.assertEqual(len(links), 6)
            self.assertEqual(json.loads(receipt.read_text())["branch"], "main")
            self.refresh()
            migration = installer.load_uninstaller().validate_migration(self.selected, installer.uninstall_api())
            extra = self.extras / "accel_homing_filters.py"
            self.assertEqual(migration[extra], {"kind": "absent"})
            self.assertEqual(installer.main(["--branch", DEV_BRANCH]), 0)
            self.assertEqual(len(links), 7)
            self.assertEqual(links[extra]["kind"], "symlink")
            self.assertEqual(json.loads(receipt.read_text())["branch"], DEV_BRANCH)
            self.assertIn(("primary_branch: %s\n" % DEV_BRANCH).encode(), self.moon_config.read_bytes())
            checked = installer.load_uninstaller().validate_installation_history(self.selected, installer.uninstall_api())
            self.assertEqual(len(checked["active"]), 2)
            self.assertEqual(len(installer.load_uninstaller().validate_migration(self.selected, installer.uninstall_api())), 8)
            axis_note, moon_note = b"\n# operator retained axis note\n", b"\n[operator]\nkeep: yes\n"
            self.axes.write_bytes(self.axes.read_bytes() + axis_note)
            self.moon_config.write_bytes(self.moon_config.read_bytes() + moon_note)
            self.refresh()
            plan = installer.make_uninstall_plan(self.selected)
            def restore(path, state):
                if Path(path).parent == self.extras and Path(path).name in installer.MODULES:
                    links[Path(path)] = dict(state)
                else:
                    installer.write_state(path, state)
            installer.apply_uninstall_plan(plan, self.selected, writer=restore)
            self.assertTrue(all(state["kind"] == "absent" for state in links.values()))
            self.assertFalse(receipt.exists())
        self.assertEqual(self.config.read_bytes(), original[self.config])
        self.assertEqual(self.axes.read_bytes(), original[self.axes] + axis_note)
        self.assertEqual(self.moon_config.read_bytes(), original[self.moon_config] + moon_note)
        self.assertEqual((self.repo / ".git/info/exclude").read_bytes(), original[self.repo / ".git/info/exclude"])


if __name__ == "__main__":
    unittest.main()
