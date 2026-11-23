# Installation, updates and recovery

[← Documentation](../README.md)

Run shell commands on the Linux printer host as the existing Klipper service
user. Keep the printer idle and unpaused throughout installation, updating,
uninstallation and service restarts. Installation does not request motion.

## Prerequisites

- A working Klipper or Kalico installation, Bash, Git, and Python 3.8 or newer.
- A configured, functioning accelerometer supported by the
  [sensor adapter](compatibility.md). Configure its MCU, wiring, firmware and
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
(
  set -e
  url=https://github.com/FrankRamm/accel_homing.git
  repo="$HOME/accel_homing"
  if [ -e "$repo/.git" ]; then
    cd "$repo"
    if [ "$(git remote get-url origin)" != "$url" ] || [ -n "$(git status --porcelain)" ]; then
      echo "Expected a clean accel_homing checkout with origin $url." >&2
      exit 1
    fi
    git fetch origin refs/heads/main:refs/remotes/origin/main
    git switch main
    git merge --ff-only origin/main
    if [ "$(git rev-parse HEAD)" != "$(git rev-parse origin/main)" ]; then
      echo "Local main has unpublished commits. Review them before installing." >&2
      exit 1
    fi
    git branch --set-upstream-to=origin/main main
  else
    git clone --branch main "$url" "$repo"
  fi
  bash "$repo/install.sh"
)
```

Use the same block for a fresh installation or a reinstall. It updates an
existing clean checkout to the published `main` before running its installer.
If Git reports an error, the block stops. Keep the checkout in place because
the installed extras link to its Python modules.

The managed source must be a complete clean `~/accel_homing` Git checkout.
The stable branch tracks `origin/main`. The tester procedure below tracks
`origin/dev`. Both use origin `https://github.com/FrankRamm/accel_homing.git`.
The installer refuses conflicting source identity or ownership; it does not
reset or stash local changes to make them disappear. One managed Klipper
instance may use this per-user clone; a discovered second instance using its
extras links is rejected. Do not share this clone across printer installations.

To inspect a plan without writes or automatic restarts:

```bash
~/accel_homing/install.sh --dry-run
```

A dry run reports missing plotting dependencies but does not install them.

### Switch an existing tester installation to dev

With the printer idle, run these commands over SSH as the Klipper service user.
The installer links the additional modules and reloads the host services.

```bash
(
  set -e
  cd ~/accel_homing
  test "$(git remote get-url origin)" = https://github.com/FrankRamm/accel_homing.git
  if [ -n "$(git status --porcelain)" ]; then
    echo "Preserve your local plugin edits before switching branches." >&2
    exit 1
  fi
  git fetch origin
  if git show-ref --verify --quiet refs/heads/dev &&
     ! git merge-base --is-ancestor dev origin/dev; then
    git branch -m dev "dev-before-sync-$(git rev-parse --short dev)"
  fi
  git switch dev
  git branch --set-upstream-to=origin/dev
  git merge --ff-only origin/dev
  bash ./install.sh
)
```

The branch rename preserves any local commits from the old dev history. If
this is a single-branch clone, run `git remote set-branches --add origin dev`
inside the checkout before the block. The dev installer updates an existing
standard `primary_branch: main` registration to `dev`, including in an
included Moonraker configuration. Subsequent frontend updates follow dev.
Verify the checkout with `git -C ~/accel_homing status -sb`; it should show
`dev...origin/dev`.

## What installation changes

Installation uses the current configuration and does not create or read
installation backups. Existing `.accel_homing_backups/` directories are left
untouched, even if their contents are incomplete or no longer match your files.
Current user edits are preserved outside the settings the installation needs
to change.

Installation links the plugin modules into the selected `klippy/extras`, adds
one plugin section/include when needed, and sets the requested X/Y endstop pins
to `accel_homing:x` and `accel_homing:y` with `homing_retract_dist: 0`. Historical
Beacon sections retain their aliases. Only the selected axes are changed.

Associated X/Y TMC `diag_pin`, `diag0_pin`, `diag1_pin`, `home_current` and rail
`homing_accel`/`min_home_dist` settings are commented with their former values.
A stepper-section `min_home_dist` value is copied into `accel_homing.cfg` as
`<axis>_min_home_dist`.
Existing `run_current`, thresholds, Z/probe settings, unrelated macros and other
axes are preserved. Default installation preserves configured homing speeds;
`--homing-speed` explicitly requests a speed change. An update does not infer
and restore a speed written by an earlier installer.

Existing regular Python files at managed link paths are not overwritten.
Recognized MISC module links are replaced automatically. Other
source links require review and `--replace-existing-links`.
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
may ask for your Linux sudo password before writing files when a controlling
terminal is available. A process restart
does not itself establish that the printer reached Ready or that homing works.

Restart authorization accepts validated sudo credentials or an explicit
`NOPASSWD` rule for the exact command executed as root:
`/absolute/path/to/systemctl --system --no-ask-password restart -- SERVICE`.
Permission to list commands or run an unrelated command does not establish
permission for this restart. Authorization is checked again before writing and
activation; the restart itself always uses noninteractive sudo.

For sudo older than 1.9.15, passwordless detection supports local `sudoers`
rules with literal commands or `ALL`, including an exact `NOPASSWD` rule after
a general password-required rule. Complex patterns, bound defaults, ambiguous
selectors or other policy sources require validated credentials or
`--no-restart`. A noninteractive SSH session never opens a password prompt;
if authorization cannot be verified, the installer stops before file writes.

For a process manager the installer cannot verify, or to defer activation:

```bash
~/accel_homing/install.sh --no-restart
```

Then restart the selected host processes manually while idle using the printed
commands. `--no-restart` also opts out of automatic Moonraker activation.
Uninstall leaves service restarts to the operator, after physical endstop
configuration has been restored manually.

Console `RESTART`/`FIRMWARE_RESTART` rebuild printer objects inside the same
Python process; they do not reload already imported modules after code/link
changes. Use a full host process restart for code installation, updates,
and uninstall. Configuration-only edits and calibration's
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
| `--replace-existing-links` | Replace reviewed links from another source. |
| `--acknowledge-homing-override` | Acknowledge reviewed `gcode_macro G28` behavior for installation only. |
| `--dry-run` | Validate and print a plan without writes. |
| `--rollback BACKUP_DIRECTORY` | Deprecated; exits without writes. Historical restoration is no longer supported. |
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
[timing and optional travel policies](../advanced/technical-reference.md#timing-and-optional-travel-policies).

Built-in `[safe_z_home]` or `[homing_override]` needs no acknowledgement flag.
The plugin preserves their ordinary G28 behavior. Custom G28 macros still need
`--acknowledge-homing-override` for installation after reviewing their normal
homing sequence. Workflows use native selected-axis homing and bypass public
G28 wrappers/macros, so prepare XY currents/enables and clearance independently.
See
[workflow restrictions](compatibility.md#workflow-restrictions).

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

The troubleshooting command loads its optional module from the installed
package or the same resolved source checkout. Existing installations with the
original six module links therefore support Git-only updates adding this
diagnostic. A missing or broken diagnostic module produces an error only when
that command is invoked; it does not prevent ordinary plugin startup.

For a manual update, run the [install commands](#install) above.

If Git or the installer reports local changes, preserve and review them. Do not
use a forced reset to conceal an ownership or configuration conflict.
`--no-update-manager` installs do not configure this native update flow.

## Moonraker update manager

The default installer already adds this entry. **Do not add a duplicate** in
`moonraker.conf` or its includes. For manual setup only, add this if missing:

```ini
[update_manager accel_homing]
type: git_repo
channel: dev
path: ~/accel_homing
origin: https://github.com/FrankRamm/accel_homing.git
primary_branch: main
is_system_service: False
managed_services: klipper
```

Add the base `[update_manager]` section once if absent. After manual edits,
restart the matching Moonraker service while idle. `~` is Moonraker's service
user's home; use the printed absolute path if needed. Uninstall removes the
matching plugin entry while preserving unrelated updater configuration.
[Update details](#moonraker-updates) ·
[Moonraker reference](https://moonraker.readthedocs.io/en/latest/configuration/#git-repo-configuration).

## Migrate from the MISC plugin

Run the normal standalone installer above while idle. Installations from
`FrankRamm/MISC` or `Monolith3D/MISC` no longer require the old uninstaller or
`--replace-existing-links`. The deleted MISC repository is not contacted.
An edited or removed old Moonraker registration does not block installation.

The installer preserves current thresholds and SAVE_CONFIG models and points
recognized plugin links at `~/accel_homing`. It replaces an identified MISC
`[update_manager accel_homing]` section, including one in an included file, or
creates the standalone entry if absent. Other Moonraker sections and comments
are preserved. An unused old `accel_homing_update.cfg` is left in place.

No backup history or original configuration is required. Existing backup
folders are not read, changed or deleted. Old source directories are left in
place. Current instance selection, source ownership, non-symlink Python files,
concurrent edits, and idle/restart checks still apply.

Uninstall removes the current plugin setup; it does not restore the previous
MISC installation or its original physical endstop settings. Installation does
not test homing; supervise subsequent physical testing.

## Uninstall

Preview or perform the operation from the complete checkout:

```bash
~/accel_homing/uninstall.sh --dry-run
~/accel_homing/uninstall.sh
```

Selection options such as `--config` and `--service` work here too.
Uninstall uses the current configuration and recognized plugin module links.
It removes the plugin sections, saved plugin models, plugin virtual endstop
options and matching Moonraker updater registration. Other settings, comments,
includes, files and unrelated SAVE_CONFIG models remain in place. An empty
plugin include file may remain so existing include directives stay valid.

Original endstop pins, speeds, retract distances and sensorless settings are
not restored. **Configure the physical X/Y endstops and review their homing
settings before restarting Klipper.** Existing commented-out values can be
reviewed manually; they are never automatically re-enabled.

Uninstall checks current ownership and refuses ambiguous or foreign module
links, regular Python files at module paths, shared consumers and concurrent
changes. It does not compare your files with old installation snapshots.

Uninstall does not restart services or move the printer. Once the configuration
is ready, manually restart the selected Klipper and any listed Moonraker
service while idle. The old Python module and updater may remain loaded until
those processes restart.

Charts, old backups, source checkouts and the `.plot-deps/` cache remain on disk.
No new backup or undo command is created. Only remove the source after uninstall
succeeds, the configuration is ready, services have restarted and no remaining
links or updater entries reference that checkout. Do not delete the checkout
to work around a failed uninstall.

Only after those restarts, and after checking that no remaining plugin links
or manual updater entries need this checkout, remove the source if desired:

```bash
rm -rf ~/accel_homing
```

## Backup deprecation and interrupted installation

Persistent installation backups and historical rollback are deprecated.
`install.sh --rollback` exits without changing files. Installation, migration,
updates, activation and uninstall ignore `.accel_homing_backups/`; no old
manifest can block a new installation.

The installer retains the current operation's before-state only in memory
while applying changes. If a write fails, it attempts to restore changes from
that operation without overwriting a later concurrent edit. These temporary
states are not saved as a backup elsewhere. Activation receipts contain service
and source metadata, not copies of configuration contents.

A terminated process or power loss has no persistent automatic restore point.
Inspect the current files and rerun the installer after resolving any partial
write or current ownership conflict. See [troubleshooting](troubleshooting.md).

Next: [Simple supervised calibration](calibration.md)
