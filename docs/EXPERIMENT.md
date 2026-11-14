# Experimental homing comparison

`ACCEL_HOMING_EXPERIMENT` compares the active raw XYZ detector with a learned
weighted-XYZ detector on the printer, collecting evidence for every attempted
home. It is an opt-in experiment on `main`; ordinary G28, CALIBRATE, the legacy
TEST command and TROUBLESHOOT retain their policies outside this command.

## Run

Update the existing `main` installation with the normal Moonraker updater or
the [installation procedure](INSTALLATION.md), then restart the selected
Klipper/Kalico **host service** to reload Python. A console RESTART alone does
not reload imported modules. The optional experiment module supports existing
symlink installations by loading its exact sibling in the resolved checkout.

Follow [moving-workflow preparation](COMMANDS.md#prepare-every-moving-workflow):
printer idle, both XY axes unhomed and manually centered, supported Z and clear
nozzle, appropriate motor state and operator supervision. `CENTERED=1` confirms
that preparation; it does not measure position or automatically center the head.

```gcode
ACCEL_HOMING_EXPERIMENT AXIS=X SAMPLES=10 CENTERED=1
```

Use `AXIS=Y` in a separately prepared run for Y. The command requires one axis
at a time. Share the complete ZIP reported in the console and status.

| Parameter | Default | Meaning |
|---|---|---|
| `AXIS` | Required | `X` or `Y`; no combined XY run. |
| `SAMPLES` | `10` | Fresh candidate homes, from 2 to 40; the ten reference homes and discovery are additional. |
| `CENTERED` | Required `1` | Operator confirms the preparation above. |
| `GC_GUARD` | `1` | Suppress automatic and supported reactor GC during each native home. `0` leaves the existing collection policy unchanged. Unsupported guarded reactors reject before movement. |
| `MAX_SAMPLE_AGE` | `0.040` | Candidate age limit in seconds for the oldest eligible sample in a delivered batch. Valid range: 0.001 to 0.5 seconds. |

The sequence is:

1. One discovery home, retained but excluded from fitting and statistics.
2. Ten reference homes at the active threshold and configured speed. Six train
   the candidate; four independently check the frozen candidate.
3. If the candidate passes replay checks, `SAMPLES` fresh experimental homes.
   The default is ten, for 21 homes total on a fully accepted run.
4. Restore temporary state and export the result, captures and plots.

Successful homes return to the configured midpoint between attempts. A failed
home ends the run without recovery calibration or a blind automatic return.
An unsuccessful fit prevents experimental homes; the reference evidence and
rejection reasons remain available. `ACCEL_HOMING_CANCEL` requests cancellation
when the host can handle it; it is not an emergency stop.

## What changes during this command

The experimental score weights the three reference-subtracted acceleration
components separately. Weights and the threshold are frozen before testing;
they are not retuned against failed validation homes. The weighted route uses
short temporal confirmation, while an instantaneous raw XYZ crossing retains
the original impact threshold as a stop path. The decision sample, supporting
sample and host dispatch are distinct events in the evidence.
The raw stop path also retains its susceptibility to noise above that original
threshold. This comparison does not claim to eliminate every false trigger.

Reference captures include startup and moving vibration. The recorded raw
trigger is a software reference, not independently measured physical contact.
Agreement with those timestamps is necessary for this experiment, but is not
proof of better physical accuracy. All candidate decisions must pass the full
prefix replay checks, including checks for significantly earlier triggers.

The scoped memory-collection guard suppresses automatic collection and the
supported reactor's explicit collection check during each native home. Original
GC/reactor state is restored even when the home fails. Both phases use the same
selected guard policy, so the comparison does not isolate the effect of GC.
Existing diagnostic runs can provide context, but a separate controlled test
is necessary to attribute changes to GC alone.

Experimental sample-age checking rejects stale data rather than declaring a
successful home. This is a host-side check, not a guarantee on stopping time.
The 40 ms default is a conservative experiment envelope, not a universal sensor
transport guarantee. Reference packet ages are checked before activating the
candidate; an incompatible envelope prevents candidate movement. The command
does not increase the limit automatically. Increasing it manually permits older
data and does not reduce transport delay. The diagnostic itself still adds work
and memory usage.

Speed, acceleration and motor currents are not automatically searched. A SPEED
different from the configured speed is rejected before movement: a threshold
qualified at one speed must not be blindly reused at another. This experiment
does not save a model, write printer.cfg, change position limits, restart, or
enable its detector for subsequent ordinary homes. SAVE_CONFIG is unsupported.

## Evidence and interpretation

The diagnostic bundle retains per-home raw CSV data, packet and timing records,
baseline and saturation details, commanded trigger/halt/return positions,
effective settings, source/firmware identity and available host/MCU statistics.
It also reports the fitted profile, train/holdout evidence, reference versus
experimental phases, GC guard state and partial failures. See
[diagnostic capture](TROUBLESHOOT_CAPTURE.md) for file formats, clock domains,
bounded retention, plots and export limitations.
Results are written below `charts_directory/experiment/<run_id>/`, including
`experiment.json`, the manifest and plots. The console reports the ZIP path.

Travel and post-trigger travel describe commanded steps. They do not measure
actual carriage displacement, motor rotation, contact force or missed steps.
Completion means the requested workflow finished; it does not certify physical
repeatability or universal reliability.

## Remaining limits

- MCU-local accelerometer triggering and a sensor-freshness watchdog independent
  of Python require supported firmware/protocols. This command does not add them.
- It does not infer that a stationary head is already pressing a stop. Manual
  centering is required; a failed run requires renewed preparation.
- Automatic speed/current search and automatic notch selection are not part of
  this command. The separate notch work on `dev` is not silently activated here.
- Beacon and configured Cartographer accelerometers share the experimental
  detector, while retaining their own data conversion and transport. Real
  validation must cover each sensor, transport, axis and machine condition.
- Independent position/contact measurements and trials with different fan,
  temperature and host-load conditions are needed to assess physical results.
