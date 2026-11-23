# Contributing

accel_homing is an experimental, vibecoded Klipper plugin. It deliberately
drives axes into their mechanical stops, so changes need care. Contributions
are welcome.

## Getting help

Questions and setup help go to Discord:
https://discord.com/channels/1227971059764953230/1564459292113899632

GitHub issues are for confirmed bugs. Use the bug form.

## Branches

`dev` is the integration branch. `main` is stable.

Branch from `dev` and open pull requests against `dev`. Keep your branch
rebased on `dev`; do not merge `dev` into it.

## Running the tests

Tests only count on Linux. Klipper never runs on Windows, and Windows runs
give unrelated failures.

With Docker, from the repository root:

```bash
docker run --rm -v "$(pwd):/src" -w /src python:3.11-slim sh -c "pip install -q numpy matplotlib && python3 -B -m unittest discover -s tests"
```

On a Linux host:

```bash
python3 -B -m unittest discover -s tests -v
git diff --check
```

See [TESTING.md](TESTING.md) for focused checks and details.
`validation/VALIDATION.json` is a historical record; its paths and hashes are
not kept current.

## Testing on a printer

Supervise every printer action with an e-stop in reach.

In the pull request, state the printer, kinematics, accelerometer or probe,
and whether you run Klipper or Kalico. Changes to homing or detection
behaviour need a hardware test.

## Commits and pull requests

Write the summary line in the imperative, for example:

- Fix calibration on Kalico and tolerate coherent Beacon cruise ripple
- Support Kalico's command-aware Homing state during calibration
- Validate signed cruise direction when mapping noisy accelerometers

One topic per pull request. Fill in the pull request template. Update the
docs when behaviour or commands change.
