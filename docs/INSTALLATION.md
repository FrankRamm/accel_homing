# Installation, updates and recovery

Run shell commands on the Linux printer host as the existing Klipper service
user. Keep the printer idle and unpaused throughout installation, updating,
uninstallation and service restarts. Installation does not request motion.

## Prerequisites

- A working Klipper or Kalico installation, Bash, Git, and Python 3.8 or newer.
- A configured, functioning accelerometer supported by the
  [sensor adapter](COMPATIBILITY.md). Configure its MCU, wiring, firmware and
  driver before installing this plugin.
- Write access to the selected printer configuration and Klipper extras
  directory. Use the normal service user; the installer requests sudo for
  verified system service activation when needed.
- A native Git clone for Moonraker updates. A ZIP/archive, copied subtree or
  old MISC checkout is not the standalone update source.
- Internet access for the clone and, when needed, binary plotting dependencies.

The installer discovers the Klipper checkout, main configuration, API socket,
service and configured sensor. Multiple possible instances or sensors require
explicit selection. It does not infer electrical wiring or validate mechanics.

## Install

```bash
cd ~
git clone https://github.com/FrankRamm/accel_homing.git
~/accel_homing/install.sh
```

Keep this checkout in place: installed extras are links to its Python modules.
Moving or deleting the checkout breaks those links. A second `git clone` into
an existing directory is unnecessary; use the update procedure below.

The default managed source must be the complete clean `~/accel_homing` Git
checkout of `main`, with exact
origin `https://github.com/FrankRamm/accel_homing.git` and upstream `origin/main`.
The installer refuses conflicting source identity or ownership; it does not
reset or stash local changes to make them disappear. One managed Klipper
instance may use this per-user clone; a discovered second instance using its
extras links is rejected. Do not share this clone across printer installations.

To inspect a plan without writes or automatic restarts:

```bash
~/accel_homing/install.sh --dry-run
```

A dry run reports missing plotting dependencies but does not install them.

## Explicit development branch

`codex/accuracy-filters` is **development / experimental**. It adds an opt-in
causal detector and comparison plots; installation does not home or calibrate
the printer and does not convert saved RAW models.

For a first installation of that branch:

```bash
git clone --branch codex/accuracy-filters https://github.com/FrankRamm/accel_homing.git ~/accel_homing
~/accel_homing/install.sh --branch codex/accuracy-filters --dry-run
~/accel_homing/install.sh --branch codex/accuracy-filters
```

For an existing clean native clone, inspect `git -C ~/accel_homing status
--short` first and resolve local changes explicitly. Do not reset/stash them
as an installation shortcut. On the first switch to the development branch:

```bash
git -C ~/accel_homing fetch origin
git -C ~/accel_homing switch --track -c codex/accuracy-filters origin/codex/accuracy-filters
~/accel_homing/install.sh --branch codex/accuracy-filters --dry-run
~/accel_homing/install.sh --branch codex/accuracy-filters
```

If that local branch already exists, use `git -C ~/accel_homing switch
codex/accuracy-filters` instead of recreating it. The source branch/upstream and
the explicitly selected installer branch must agree. The matching Moonraker
entry uses `primary_branch: codex/accuracy-filters`; do not leave it targeting
main while loading development code. Only `main` and `codex/accuracy-filters`
are accepted. `--branch` is a managed install/update option and is rejected
with uninstall, rollback or `--no-update-manager`.

The seventh `accel_homing_filters.py` module and its Git exclude entry are
recorded transactionally. Existing six-module history and updater receipts
remain valid; a later uninstall restores the actual prior owned configuration
across both generations while preserving unrelated later edits. Retain the
complete backup journal. Matching existing native manual Moonraker stanzas can
receive a reversible branch-only edit; conflicting ownership is refused.

Default HYBRID CALIBRATE writes paired model schema 3. Installing/updating this
branch alone preserves existing schema 1/2 models; only a later operator-run
paired calibration creates schema 3. Older code without schema-3 support cannot
load it, even when ordinary homing is selected RAW. A later explicit pure
ten-check calibration can retain the archived five-check profile's paired
certificate and schema 3. Do not infer compatibility from the selected mode,
edit the schema number or relabel its qualification count. Restore a verified
compatible pre-pair baseline through the reviewed development uninstall or
exact unchanged transaction rollback below before switching to an older target.
The RAW-recalibration route in the next paragraph applies only to legacy schema
1/2 models that have never used schema 3.

For those legacy models, before returning to main **after saving any FILTERED axis**, keep the
development code active and restore a valid selected RAW model for every
affected axis. The supported operator route is normal supervised calibration:
reprepare XY, run `ACCEL_HOMING_CALIBRATE AXIS=X DETECTOR=RAW CENTERED=1`, let it
save/restart, then repeat manual preparation and RAW calibration for Y if Y
was filtered. Confirm all affected selected models are RAW after restart and
the saved model has returned to schema version 1. A model containing FILTERED
uses schema version 2; current stable main rejects it at startup rather than
reinterpreting a filtered threshold as RAW. Do not switch code first and leave
an incompatible model, or restore an entire old `printer.cfg` over later edits.

While **still running the development installer**, preview and restore the
owned development installation history before switching source code:

```bash
~/accel_homing/uninstall.sh --dry-run
~/accel_homing/uninstall.sh
```

This restores the journal's original activation and plugin-model baseline,
including pre-existing native six-module links/main updater when present;
a first-time install may be removed completely. Review that concrete plan.
An exact unchanged installation transaction can instead use its printed
`--rollback` path. After later calibration/config edits, rollback may refuse
changed files; use the reviewed history-aware uninstall rather than forcing
old file bytes over current edits.

Verify the pre-development module/updater state is restored and no active
development ownership remains. Then select the clean tracked main checkout.
Defer service restart until source and restored links/configuration agree.
The older main installer does **not** accept `--branch` and cannot validate an
active development journal; if reinstalling main is needed after restoration,
use its `install.sh --dry-run` and `install.sh` without `--branch`.
If no FILTERED model was saved, the original RAW model was not converted and
the exact recorded transaction rollback is sufficient when unchanged.
See [the model rollback limits](FILTERS.md#returning-to-stable-code).

## What installation changes

The transaction records original file bytes, modes and link targets under
`.accel_homing_backups/` beside the selected main `printer.cfg` before applying
changes. Preserve the complete backup history, not just the latest directory.

Installation links the plugin modules into the selected `klippy/extras`, adds
one plugin section/include when needed, and sets the requested X/Y endstop pins
to `accel_homing:x` and `accel_homing:y` with `homing_retract_dist: 0`. Historical
Beacon sections retain their aliases. Only the selected axes are changed.

Associated X/Y TMC `diag_pin`, `diag0_pin`, `diag1_pin`, `home_current` and rail
`homing_accel`/`min_home_dist` settings are commented with their former values.
Existing `run_current`, thresholds, Z/probe settings, unrelated macros and other
axes are preserved. Default installation preserves configured homing speeds;
`--homing-speed` explicitly requests a speed change. An update does not infer
and restore a speed written by an earlier installer.

Existing regular Python files at managed link paths are not overwritten.
Links to another source require review and `--replace-existing-links`; this
flag does not adopt an old MISC installation's backup/update ownership.
Duplicate sections/options/includes, ambiguous instances and unsupported
configuration structures fail with a reported conflict.

The installer checks a real PNG render using the selected Klipper interpreter.
It reuses a working plotting stack or prepares binary wheels in the checkout's
private, ignored `.plot-deps/` cache. It does not change global Python package
pins. A dependency/preflight failure stops before printer configuration writes.

## Service activation

Default installation/update restarts the exact verified Klipper systemd service
and activates the correlated Moonraker service when required. It verifies the
selected unit/process identity and rechecks idle state before activation. It
may ask for your Linux sudo password before writing files. A process restart
does not itself establish that the printer reached Ready or that homing works.

For a process manager the installer cannot verify, or to defer activation:

```bash
~/accel_homing/install.sh --no-restart
```

Then restart the selected host processes manually while idle using the printed
commands. `--no-restart` also opts out of automatic Moonraker activation.
Uninstall and explicit rollback always leave service restarts to the operator.

Console `RESTART`/`FIRMWARE_RESTART` rebuild printer objects inside the same
Python process; they do not reload already imported modules after code/link
changes. Use a full host process restart for code installation, updates,
uninstall and module-link rollback. Configuration-only edits and calibration's
`SAVE_CONFIG` use the normal console restart behavior.

Restarting also discards unsaved configuration/model changes. Preserve any
pending work before installing or updating.

## Options and explicit instance selection

```bash
~/accel_homing/install.sh --help
```

| Option | Purpose |
| --- | --- |
| `--klipper-path PATH` | Select the Klipper/Kalico checkout. |
| `--config PATH` | Select the main `printer.cfg`. |
| `--service NAME` | Select its exact systemd service. |
| `--klippy-socket PATH` | Select its read-only Klipper API socket. |
| `--klipper-python PATH` | Select its exact venv Python when discovery cannot establish it. |
| `--moonraker-config PATH` | Select the Moonraker config correlated with that Klippy socket. |
| `--accel-chip 'adxl345 toolhead'` | Select a full configured sensor section; preserve suffix case. |
| `--generic-names` | Rename a legacy plugin section and both X/Y pin aliases transactionally; preserve calibration and reject active legacy macro/object/command references. Installation/update only. |
| `--axes x,y`, `--axes x`, `--axes y` | Install both axes by default, or only the selected axis. |
| `--homing-speed N` | Explicitly replace selected-axis homing speeds in mm/s. |
| `--max-report-travel N` | Opt into the generic reporting policy, finite 0.1–2.0 mm. |
| `--max-startup-travel N` | Opt into the generic startup policy, finite 0.1–5.0 mm. |
| `--no-update-manager` | Skip managed source/Moonraker registration for offline or development use. |
| `--no-restart` | Defer service activation to the operator. |
| `--replace-existing-links` | Back up and replace reviewed links from another source. |
| `--acknowledge-homing-override` | Acknowledge reviewed `gcode_macro G28` behavior for installation only. |
| `--dry-run` | Validate and print a plan without writes. |
| `--rollback BACKUP_DIRECTORY` | Restore one unchanged transaction. |
| `--uninstall` or `-u` | Perform full managed uninstallation. |

`KLIPPER_PATH`, `PRINTER_CONFIG`, `KLIPPER_SERVICE` and `KLIPPY_SOCKET` supply
defaults for the corresponding selection options. Explicit selections must
still agree with the selected instance; they do not bypass identity checks.

For example, replace every path/service below with the actual instance:

```bash
~/accel_homing/install.sh --dry-run \
  --klipper-path /home/pi/klipper \
  --config /home/pi/printer_data/config/printer.cfg \
  --service klipper.service \
  --klippy-socket /home/pi/printer_data/comms/klippy.sock \
  --accel-chip 'adxl345 toolhead'
```

Generic travel flags are optional and rejected for native Beacon. Absent flags
preserve existing explicit limits or leave them disabled. They do not raise
acceleration or automatically choose a slower speed. See
[timing and optional travel policies](DETAILS.md#timing-and-optional-travel-policies).

Built-in `[safe_z_home]` or `[homing_override]` needs no acknowledgement flag.
The plugin preserves their ordinary G28 behavior. Workflows bypass their scripts
and need independent XY preparation; custom G28 macros remain unsupported by
workflows even if installation was acknowledged. See
[workflow restrictions](COMPATIBILITY.md#workflow-restrictions).

Development opt-outs are not a shared-instance migration mechanism. In
particular, do not replace links in a shared Klipper checkout while another
printer instance using them is active.

## Moonraker updates

Default installation registers the correlated Moonraker instance automatically.
The source path is the complete per-user `~/accel_homing` checkout, such as
`/home/YOUR_USER/accel_homing`, not a subtree or download cache. A manual entry
has this form; use the exact absolute path printed by your installation:

```ini
[update_manager accel_homing]
type: git_repo
channel: dev
path: /home/YOUR_USER/accel_homing
origin: https://github.com/FrankRamm/accel_homing.git
primary_branch: main
is_system_service: False
managed_services: klipper
```

Add `[update_manager]` once if absent. Keep exactly one `accel_homing` entry
across all includes. `managed_services: klipper` is Moonraker's supported alias
for its associated Klippy service; do not replace it with an arbitrary unit
name. Manual configuration edits require restarting that Moonraker service.

Use **accel_homing → Update** in Mainsail/Fluidd while idle. Native Git updates
do not execute the installer as a post-update hook. Rerun `install.sh` when a
release requires configuration/dependency changes, or after Hard Recovery if
the private plotting cache needs to be rebuilt. Frontend update controls and
version detection depend on Moonraker/frontend versions and repository tags;
see the [official Git-repo configuration](https://moonraker.readthedocs.io/en/latest/configuration/#git-repo-configuration).
The `dev` channel does not make this experimental plugin a stable release.

For a manual update of an established clean `main` checkout:

```bash
git -C ~/accel_homing pull --ff-only
~/accel_homing/install.sh
```

If Git or the installer reports local changes, preserve and review them. Do not
use a forced reset to conceal an ownership or configuration conflict.
`--no-update-manager` installs do not configure this native update flow.

## Migrate from the MISC plugin

The standalone repository has a different source and update identity. It does
not automatically take ownership of an installation from
`FrankRamm/MISC` or `Monolith3D/MISC`.

1. Keep the printer idle. Back up manual thresholds and the SAVE_CONFIG models
   before uninstall: full uninstall restores the recorded baseline and can
   remove models introduced by that installation. Preserve current configuration,
   plots and the complete `.accel_homing_backups/` history. Keep the old source
   checkout/cache available.
2. Run the **old installation's own uninstaller**, with that installation's
   instance options. Use the exact existing path rather than a guessed MISC
   directory. Review its output and require a successful result.
3. Restart every service it lists while idle. Verify that the previous endstop
   configuration was restored. An uninstall conflict or incomplete backup is a
   recovery task; do not continue by deleting links, updater entries or history.
4. Install this standalone repository using the commands above. Review the new
   plan and confirm its selected sensor and restored homing settings.
5. Prepare and supervise new calibration/testing. Historical models and results
   do not validate the standalone revision or the current mechanics.

Do not use `--replace-existing-links` as a shortcut around legacy ownership.
If the old source is missing, recover the version associated with its recorded
backup before attempting restoration. Keep old sources until no restored links
or retained recovery procedure requires them.

## Uninstall

Preview or perform the operation from the complete checkout:

```bash
~/accel_homing/uninstall.sh --dry-run
~/accel_homing/uninstall.sh
```

Selection options such as `--config` and `--service` work here too.
`uninstall.sh` performs full uninstall and rejects `--rollback`; use
`install.sh --rollback` for a single transaction.

Full uninstall follows the active installation/upgrade history back to its
recorded baseline. It restores original endstops, retract/speed settings,
sensorless options and plugin links; removes owned includes/configuration,
installer-created updater registration and newly introduced plugin models;
and restores any pre-existing plugin state. Unrelated edits and other SAVE_CONFIG
data, such as Beacon/bed-mesh models, are preserved when ownership is clear.

An intact history and recognizable managed edits are required. Changed links or
settings, unrelated additions to an installer-created file, missing original
targets, shared consumers or incomplete backups cause a conflict before writes.
There is no force-uninstall option that overwrites later edits.

Successful uninstall prints a current-state backup and an **Undo this uninstall**
command. Save both. Then manually restart the selected Klipper and any listed
Moonraker service while idle. Uninstall does not restart services or move the
printer. The old Python module and updater may remain loaded until those
processes restart.

Charts and transaction backups remain on disk. The checkout and its
`.plot-deps/` cache also remain until you remove them yourself. **Only after
uninstallation succeeds and the listed services have restarted**, check that
no restored or shared plugin links or pre-existing manual updater entries
still need this checkout. Remove/reconcile any manual updater entry separately;
the uninstaller preserves pre-existing registration it does not own. Then, if desired:

```bash
rm -rf ~/accel_homing
```

Do not run that deletion after a failed uninstall. Deleting the checkout also
deletes its local undo entrypoint and plotting cache. To use a retained
transaction backup later, recover or re-clone a compatible source at the path
required by its recorded links, then use the saved rollback command. Retain
the source instead if you want the printed undo command to remain immediately
usable.

## Transaction rollback and conflicts

`--rollback` restores one selected transaction, only if the affected files and
links still match that transaction's recorded result. Use the **actual path
printed by the installer**, for example:

```bash
~/accel_homing/install.sh --rollback \
  "$HOME/printer_data/config/.accel_homing_backups/ACTUAL-BACKUP-DIRECTORY"
```

You can add `--dry-run` first. This restores the recorded configuration and
module-link states, not the caller-owned Git checkout revision. It does not
undo a Git pull or Moonraker code update, and it does not perform full uninstall.
Recover the intended compatible source revision separately when required.
Restart the printed host services manually afterward.

On a conflict, preserve current files and all history. Inspect the reported
`manifest.json` and compare its `before`/`after` states with the current file or
link. Reconcile managed edits without replacing a whole current `printer.cfg`
with an old copy. A missing baseline or removed original source cannot be
reconstructed by the uninstaller. Ordinary write failures are rolled back;
power loss or a terminated process with a `prepared` manifest can require manual
recovery. See [troubleshooting](TROUBLESHOOTING.md).
