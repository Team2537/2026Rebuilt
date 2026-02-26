# Auto Aim Diagnostics Runbook

This runbook is for diagnosing moving-shot auto aim accuracy in sim, with AKit logging.

## What this covers

- Runs `Along Alliance Moving Shot` in simulation.
- Measures shot accuracy at `z=1.83m` on **descent**.
- Reports miss distance to hub center and detailed decomposition metrics.
- Writes both a text summary and `.wpilog`.

## Baseline command

```bash
./scripts/run_along_alliance_auto_aim_diag.sh
```

The script:

- runs `AlongAllianceMovingShotAutoSimTest`
- writes artifacts under `logs/auto-aim/<timestamp>/`
- prints summary + latest `.wpilog`
- returns nonzero if thresholds fail

## Useful environment overrides

```bash
AUTO_AIM_OUT_DIR=/tmp/auto-aim-run \
AUTO_AIM_ASSERT_DESCENT_P90_METERS=0.50 \
AUTO_AIM_ASSERT_P90_DEG=20 \
AUTO_AIM_ASSERT_MAX_DEG=30 \
./scripts/run_along_alliance_auto_aim_diag.sh
```

Use relaxed thresholds when you want data collection without test failure.

Available knobs:

- `AUTO_AIM_OUT_DIR`
- `AUTO_AIM_MIN_DISTANCE_M`
- `AUTO_AIM_MIN_MOVING_FEED_SAMPLES`
- `AUTO_AIM_MIN_MOVING_FEED_FRACTION`
- `AUTO_AIM_ASSERT_P90_DEG`
- `AUTO_AIM_ASSERT_MAX_DEG`
- `AUTO_AIM_MIN_DESCENT_SAMPLES`
- `AUTO_AIM_IDEAL_DESCENT_MISS_METERS`
- `AUTO_AIM_ASSERT_DESCENT_P90_METERS`
- `AUTO_AIM_POST_AUTO_OBSERVATION_SEC`
- `SHOOTER_SIM_MOTION_COMP_TIME_SCALE`
- `SHOOTER_SIM_MOTION_COMP_DISTANCE_TIME_SCALE`

## Output files

- Summary text:
  - `along_alliance_moving_shot_summary_*.txt`
- AKit log:
  - `*.wpilog`

## Primary metrics to compare between agents

- `p90_descent_miss_in`
- `max_descent_miss_in`
- `descent_within_ideal_fraction`
- `p90_moving_aim_error_deg`
- `mean_signed_descent_miss_in`

Target:

- `p90_descent_miss_in <= 6.0`

## Interpreting miss decomposition

From summary:

- `mean_along_velocity_miss_m`
- `mean_abs_cross_velocity_miss_m`
- `p90_abs_cross_velocity_miss_m`
- `descent_sample_details`

Interpretation:

- Large `along` with small `cross`: mostly range/flight-time bias.
- Small `along` with large `cross`: mostly heading lead / lateral bias.
- Large both: coupled model issue.

`descent_sample_details` format (semicolon-separated samples):

`timestamp_sec, miss_in, along_in, cross_in, velocity_to_target_angle_deg`

## AKit keys worth plotting

- `AutoAimDiagnostics/MovingAbsAimErrorDeg`
- `AutoAimDiagnostics/CompensatedDistanceMeters`
- `AutoAimDiagnostics/CompensatedHeadingDeg`
- `AutoAimDiagnostics/DescentCrossing/LastMissDistanceMeters`
- `AutoAimDiagnostics/DescentCrossing/LastAlongVelocityMissMeters`
- `AutoAimDiagnostics/DescentCrossing/LastCrossVelocityMissMeters`
- `AutoAimDiagnostics/Summary/*`
- `sim/FUEL/accuracy/*`

## Multi-worktree workflow

1. Each agent runs with a unique output directory:

```bash
AUTO_AIM_OUT_DIR=/tmp/agent_A ./scripts/run_along_alliance_auto_aim_diag.sh
```

2. Share:
- exact commit hash
- summary file
- `.wpilog` path
- what changed

3. Compare first on:
- `p90_descent_miss_in`
- `mean_signed_descent_miss_in`
- `mean_along_velocity_miss_m`
- `mean_abs_cross_velocity_miss_m`

4. Prefer fixes that improve all four, not just one path timing.
