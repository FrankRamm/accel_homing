# Instructions for AI coding agents

These rules apply to every AI assistant working in this repository (ChatGPT,
Codex, Claude, Copilot, Cursor, …). Read [CONTRIBUTING.md](CONTRIBUTING.md)
too; it applies to you as well. Where the two disagree, CONTRIBUTING wins.

## Safety first

This plugin deliberately drives printer axes into their mechanical stops.

- Never send G-code, console commands or Moonraker requests to a real printer,
  and never edit a live printer's config, without a human confirming each
  action. Hardware testing is always supervised with an emergency stop in
  reach.
- Do not weaken safety checks (preparation, `CENTERED=1`, thresholds, contact
  qualification, cancellation) to make a test or workflow pass. If a check
  is in the way, stop and ask.
- Changes to homing or detection behaviour need a hardware test by a human
  before merge. Say so in the PR if you could not run one.

## Tests

- Only Linux runs count. Klipper never runs on Windows; Windows runs produce
  many unrelated failures. On a Windows machine, use Docker:

  ```bash
  docker run --rm -v "$(pwd):/src" -w /src python:3.11-slim \
    sh -c "pip install -q numpy matplotlib && python3 -B -m unittest discover -s tests"
  ```

  In Git Bash, prefix with `MSYS_NO_PATHCONV=1` and use `$(pwd -W)`.
- The suite uses synthetic sensors and fake printers; it never touches
  hardware. Run it and `git diff --check` before every commit, and report
  the result honestly. CI runs the same suite on Linux.
- `tests/test_docs_links.py` checks every relative link and `#anchor` in the
  Markdown files. Keep it green when you move or rename headings or files.

## Git

- Branch from `dev`, open PRs against `dev`. Rebase on `dev`; never merge
  `dev` into your branch. Do not push to `main`.
- One topic per PR. Fill in the PR template, including how it was tested.
- Commit messages: imperative summary line, like the existing history.
  Your tool's own AI-attribution trailer is fine.

## Where things live

- `accel_homing.py` — generic entry point; `beacon_accel_homing.py` — the
  original Beacon endstop module, still supported.
- `accel_homing_*.py` — one concern each: `backends` (sensor adapters),
  `workflows` (supervised calibration), `axes` (axis identification),
  `model` / `config` (saved models and thresholds), `filter` / `auto_notch`
  / `auto` (notch filter), `charts`, `troubleshoot`, `experiment`, `updater`
  (Moonraker updates).
- `install.sh`, `installer.py`, `bootstrap.*`, `uninstall*` — installation.
- `tests/harness.py` — shared fake printer and sensor fixtures; reuse them
  in new tests.
- `docs/guide/` user guides, `docs/advanced/` advanced features,
  `docs/development/` testing and history. Index: `docs/README.md`.
- Historical records — do not edit: `validation/`,
  `docs/development/historical-testing.md`.
- Never commit plans, design specs, session notes or other working
  documents. Keep them local (`docs/superpowers/` and `.superpowers/` are
  git-ignored) and put the reasoning in the PR description and commit
  messages.

## Documentation

- When you change a console command, a config option or user-visible
  behaviour, update the matching page in `docs/guide/` in the same PR.
- Keep the README short: it is the quick start, not the reference.
- Write plainly and tersely, matching the existing docs. No marketing.
