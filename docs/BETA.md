# Beta moving-noise calibration experiment

BETA is a separate supervised experiment. A successful proposal produces a
**Beta-labelled PNG** and threshold suggestion, with a diagnostic report in the same command's
console output. It does not stage/save a threshold or model, call SAVE_CONFIG
or restart Klipper. Its suggestion is not hardware qualification.

## Run

Use the same [manual XY preparation](COMMANDS.md#prepare-every-moving-workflow)
as calibration, with an idle unpaused printer, both axes unhomed and centered,
supported Z, nozzle clearance, suitable XY current/enable behavior, and an
accessible emergency stop.

```gcode
ACCEL_HOMING_BETA_CALIBRATE AXIS=X CENTERED=1
```

Use `AXIS=Y` for Y. Only one axis is accepted per command.
`CENTERED=1` authorizes movement without a second prompt.
Optional `START_THRESHOLD` and `STEP` follow normal calibration numeric and
known-range limits. `SAVE_CONFIG=0` is accepted; `SAVE_CONFIG=1` is rejected.

## What it does

1. Establishes the existing effective homing speed/acceleration, usable central
   stroke, sensor/capture APIs and exposed current context.
2. Records three independent central forward moves and their return behavior.
   The selected stroke remains before the configured directional endstop.
3. Checks actual planner acceleration, cruise and braking segments and that
   the central stroke reached the existing effective homing speed. It refuses
   insufficient stroke instead of silently lowering speed.
4. Uses startup/acceleration/cruise XYZ-magnitude noise to suggest an initial
   candidate. Braking/return peaks remain diagnostic and do not seed it.
   An explicit START_THRESHOLD overrides selection but not acquisition.
5. Performs discovery and repeatability trials with rejected-hit recovery,
   within the normal candidate/attempt/sensor limits. A successful proposal
   requires discovery plus ten qualified checks with range at most 2.00 mm.
6. Restores temporary state and reports the outcome, including failures or
   incomplete trials.

The noise margin is the larger of 20% or 0.1 g above the observed forward
peak, rounded upward onto the selected grid. Automatic selection requires a
known sensor range. Physical current is not measured; a changed homing-current
setup that cannot be matched through the installed native helper is refused.

Unlike normal calibration, BETA does not make its proposed value the saved
ordinary-homing threshold. Read the report, preserve the PNG/console output
and decide on a separate supervised validation procedure.

## Diagnostic output

The same command emits bounded `Beta diagnostic` sections on success and on
an abort when reporting remains available. They include:

- Source hashes, firmware/sensor context and timing/sample-age policies.
- Planned and observed motion phases, driver/current context and noise passes.
- Candidate thresholds, discovery/validation trials, spread and rejection reasons.
- Cleanup/restoration state, partial-output errors and the final proposal/result.
- `physical_position_verified: false`.

One combined Beta PNG records a successful proposal's motion/noise/trial
information after required cleanup succeeds. Failed, incomplete or cancelled
experiments save no PNG. Bounded console diagnostics still explain the abort,
including available trials and cleanup results. A rendering/reporting failure
is itself reported when possible.

The PNG shows absolute components after signed resting-offset subtraction;
negative times before each exact crossing remain visible. Full-sample event
statistics retain their existing definitions. Range notes cover every motion
and captured home, including finite rails without a native sentinel. Native
markers are bounded endpoint estimates with unknown true sign/peak; finite
rails have unknown excess amplitude. Reference observations are disclosed
separately and can overlap capture counts. See [plots](PLOTS.md).

## Limits and cancellation

There is no encoder or independent contact/return verification. The central
coordinate/stroke assumes the operator supplied the correct physical center;
rejected returns reverse commanded halt travel and assume no slip or missed
steps. Planner/current metadata does not measure actual mechanical motion.

Normal commands and previously saved models retain their ordinary behavior.
An experiment does not certify a sensor, printer, current, threshold or stopping
distance. The implementation's offline/source evidence is described in
[TESTING](../TESTING.md); historical Beta publication is not a physical test.

`ACCEL_HOMING_CANCEL` is a normal queued console command, not an emergency stop.
Use the accessible emergency stop for unexpected active movement. After an
abort, inspect diagnostics and manually prepare XY before a new command.
