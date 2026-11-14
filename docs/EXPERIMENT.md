# Experimental homing comparison

`ACCEL_HOMING_EXPERIMENT MODE=COMPARE` compares normal and shorter batch-delivery
intervals using the original raw XYZ detector for every home. It also evaluates
a frozen weighted profile offline, with startup and cruise noise margins.
The existing `MODE=SHADOW`
keeps every home on the raw detector and evaluates the frozen weighted profile
after each home. `MODE=LIVE` retains the original experiment's stricter checks
before allowing the weighted profile to control candidate homes. `LIVE` remains
the default; specify `COMPARE` for the paired delivery test below.
It is an opt-in experiment on `main`; ordinary G28, CALIBRATE, the legacy
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
ACCEL_HOMING_EXPERIMENT AXIS=X CENTERED=1 MODE=COMPARE SAMPLES=10
```

Use `AXIS=Y` in a separately prepared run for Y. The command requires one axis
at a time. Share the complete ZIP reported in the console and status, plus
whether the announced CONTROL and FAST homes felt different. No manual
measurement or separate fan/temperature/load adjustment is required for this run.

| Parameter | Default | Meaning |
|---|---|---|
| `AXIS` | Required | `X` or `Y`; no combined XY run. |
| `MODE` | `LIVE` | `COMPARE` pairs normal/shorter delivery using RAW throughout. `SHADOW` records raw homes and replays the frozen profile afterward. `LIVE` preserves the original live candidate workflow and its +/-1 ms compatibility requirement. |
| `SAMPLES` | `10` | In COMPARE, number of pairs: 20 comparison homes at the default. In SHADOW/LIVE, number of fresh homes. Range 2–40; ten reference homes and discovery are additional. |
| `CENTERED` | Required `1` | Operator confirms the preparation above. |
| `GC_GUARD` | `1` | Suppress automatic and supported reactor GC during each native home. `0` leaves the existing collection policy unchanged. Unsupported guarded reactors reject before movement. |
| `MAX_SAMPLE_AGE` | `0.040` | Candidate age limit in seconds for the oldest eligible sample in a delivered batch. Valid range: 0.001 to 0.5 seconds. COMPARE uses this for shadow qualification, without changing the original RAW detector's freshness policy. |

## Paired RAW delivery comparison

COMPARE runs one discovery home followed by ten reference homes at the original
threshold and configured, qualified speed. Six reference homes train the shadow
profile; four qualify that frozen profile without retuning it. Discovery is
excluded from training and statistics. It then runs `SAMPLES` pairs:

1. First pair: CONTROL, then FAST.
2. Second pair: FAST, then CONTROL.
3. Repeat that alternating order for the remaining pairs.

The default ten pairs give **31 homes total**: 1 discovery + 10 references + 20
comparison homes. Forty pairs give 91 homes. Each pair contains both variants;
with an even pair count, each variant also appears first equally often. The
console announces each variant before movement. The original threshold,
configured speed, capture detail and selected GC guard are the same throughout.
No weighted profile controls any COMPARE home, including when a previous live
profile existed before the command.

For the known Klipper batch backend, CONTROL requests the normal configured
report interval; FAST requests `min(normal, 0.002)` seconds. The existing backend
applies the smaller of that request and the driver's current interval, including
its existing timer update. The workflow changes the request only with a stopped,
clean stream and restores it after every home, before returns, and during error
or cancellation cleanup. It checks the actual active backend/helper interval
observed in callbacks. Missing, conflicting or un-restored interval evidence
aborts before further movement. External interval changes are preserved and
reported as a failed restoration, rather than overwritten.

This intervention changes host batch reporting, **not sensor sampling rate**.
Configured intervals above 5 ms are unsupported for this delivery-only change:
the same backend setting also affects startup safety budgets above that boundary.
Intervals already at or below 2 ms, including a faster interval owned by another
driver client, provide no shortening. Native Beacon RAW has no adjustable batch
delivery interval. These known unsupported/no-effect cases still collect the
reference homes and paired RAW captures, labelled `unavailable`; they do not
claim a FAST intervention or delivery improvement. An unknown driver interface,
invalid interval or dirty stream state is recorded and rejected before motion.

COMPARE can finish its RAW comparison when the shadow fit is rejected. The
bundle retains the rejection, and an unusable frozen profile is not replayed on
fresh homes. A valid shadow profile is replayed after each completed home; if a
fresh replay or noise margin fails, the shadow result becomes invalid while the
qualified RAW comparison can continue. No rejected evidence retunes the profile
or authorizes live use. A raw-home quality failure, stream failure or cancellation
still stops without an automatic return.

## Existing SHADOW and LIVE sequence

These modes retain their previous behavior:

1. One discovery home, retained but excluded from fitting and statistics.
2. Ten reference homes at the active threshold and configured speed. Six train
   the candidate; four independently check the frozen candidate.
3. If the candidate passes its mode's replay checks, `SAMPLES` fresh homes.
   In SHADOW these still use the original raw detector; the frozen profile is
   evaluated only after the complete capture. In LIVE the profile controls the
   fresh homes with the original raw crossing retained as a stop path.
   The default is ten, for 21 homes total on a fully accepted run.
4. Restore temporary state and export the result, captures and plots.

Successful homes return to the configured midpoint between attempts. A failed
home ends the run without recovery calibration or a blind automatic return.
In SHADOW and LIVE, an unsuccessful fit prevents additional homes; the reference evidence and
rejection reasons remain available. A shadow replay outside its diagnostic
window stops further attempts without an automatic return, even though that
home itself was controlled by the raw detector.
`ACCEL_HOMING_CANCEL` requests cancellation
when the host can handle it; it is not an emergency stop.

## Weighted profile and original RAW stop path

The experimental score weights the three reference-subtracted acceleration
components separately. Weights and the threshold are frozen before testing;
they are not retuned against failed validation homes. In LIVE the weighted route uses
short temporal confirmation, while an instantaneous raw XYZ crossing retains
the original impact threshold as a stop path. The decision sample, supporting
sample and host dispatch are distinct events in the evidence.
The raw stop path also retains its susceptibility to noise above that original
threshold. This comparison does not claim to eliminate every false trigger.

Reference captures include startup and moving vibration. The recorded raw
trigger is a software reference, not independently measured physical contact.
In LIVE, agreement within +/-1 ms remains necessary and is not proof of better
physical accuracy. SHADOW separately studies earlier signals within the final
20 ms before the raw crossing; this window is a diagnostic region, not a claim
that contact occurred there. Neither mode ignores startup or cruise samples.

## Shadow selection and rejection diagnostics

Six training captures select one profile. Selection requires at least three
weighted detections and no decisions before the final 20 ms region. Within
that constraint it prefers more weighted detections, then earlier summed
sample-time decisions and stronger separation from training motion noise.
The four heldout captures can only accept or reject that frozen profile; they
never change its weights or threshold. At least one heldout weighted detection
and no detections outside the diagnostic region are required for fresh shadow
homes. Full capture validation, sample provenance and cadence checks still apply.

Shadow profiles are explicitly marked as diagnostic-only and the live profile
validator rejects them. A ready shadow profile does not make a rejected live
fit acceptable. The original +/-1 ms live acceptance limit is unchanged.

The fit report distinguishes insufficient weighted detections, incompatible
software-reference timing, startup/cruise decisions and invalid evidence. It
includes candidate counts and bounded examples rather than attributing every
rejection to inseparable noise. Early near-event signals may be contact onset
or preceding vibration; an independent contact/position measurement is needed
to decide. Replay timestamps do not predict actual future stop dispatch or
physical halt, since delivery and scheduling can differ.

The shadow experiment does not reduce the current homing force or speed. It
collects evidence for a later change without using an unverified early signal
to set the home coordinate.

COMPARE additionally requires noise separation independently in startup and
cruise, for both the weighted score and the original RAW bypass. The weighted
noise measure is the largest minimum of adjacent samples that could satisfy
the frozen confirmation interval. The RAW measure is the largest individual
sample. Each threshold must exceed its region's noise measure by at least the
larger of **10% of the threshold or 0.5 g**. Candidate selection uses training
data only; heldout and fresh captures qualify or reject the frozen values.
Reports retain isolated weighted peaks, confirmable weighted pairs, RAW maxima,
measured/required margins, supporting sample indices and distinct rejection reasons.
An isolated weighted spike may exceed its threshold without confirming a stop;
the separate RAW bypass margin still applies to every individual sample.
Near-event evidence is excluded from this motion-noise
margin. A weighted margin cannot compensate for inadequate RAW bypass separation.
This extra margin policy is specific to COMPARE; SHADOW and LIVE defaults remain
unchanged.

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
candidate; an incompatible envelope prevents SHADOW/LIVE additional homes.
In COMPARE it rejects shadow qualification while qualified RAW comparison can continue.
The live candidate enforces its age limit during detection. SHADOW uses raw
detection throughout and reports the observed timing; it does not install a
different live freshness policy. The command
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

For COMPARE, `experiment.delivery_comparison` records the planned pair order,
supported/effective intervention, original and requested intervals, and restoration
results. Each home records its variant, pair/order and callback-observed interval;
reference phases remain separate from `delivery_control`, `delivery_fast` and
`delivery_unavailable`. Shadow-fit status and margin policy are independent of
RAW completion. Paired timing summaries retain unknown components rather than
turning missing evidence into zero delay. Existing fan, temperature and host-load
state is recorded where available; the workflow does not change those settings.

For SHADOW and accepted COMPARE shadow profiles, each replayed home's plot compares raw XYZ and the frozen weighted
score, with separate original and simulated decision markers. Raw CSV and
per-home replay records retain the exact samples behind those markers.
`delivery.json` and `delivery.csv` separate onset-to-batch-end, batch-end-to-adapter,
adapter-to-detector and detector-to-stop-request timing. The per-home timing
plot shows the same decomposition. Missing or ambiguous evidence stays unknown;
sample-to-host terms use clock estimates, and the delivery/driver term is not
pure transport latency. No MCU receipt or physical halt time is inferred.

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
  A Beacon COMPARE run has no delivery-interval intervention.
- Independent position/contact measurements and trials with different fan,
  temperature and host-load conditions are needed to assess physical results.
