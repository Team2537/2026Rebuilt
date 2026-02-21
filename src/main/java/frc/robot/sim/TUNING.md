# Team 2537 - 2026 REBUILT Sim Notes

This project includes a shooter tuning mode and simulated FUEL flight visualization.

## What was added

- Live shooter tuning from dashboard values (hood + left/right RPM + kicker feed).
- Trigger-based tuning control on the **driver controller**.
- Simulated FUEL poses published as a pool of **50** objects.
- Ballistic shot motion (gravity only), with respawn/reset when `z < 0`.

## Enable required subsystems

Shooter logic only runs if shooter is enabled.

Edit `src/main/java/frc/robot/Constants.java` and include shooter in:

- `ENABLED_MECHANISMS`

Example:

```java
public static final EnumSet<Mechanism> ENABLED_MECHANISMS =
        EnumSet.of(Mechanism.DRIVE, Mechanism.VISION, Mechanism.SHOOTER, Mechanism.INTAKE, Mechanism.TRANSFER);
```

## Driver controls

- `driverController.rightTrigger`:
  - If `Shooter/Tuning/Enabled = false`: normal distance-based shoot command.
  - If `Shooter/Tuning/Enabled = true`: dashboard tuning command.
- `driverController.leftTrigger`: normal aim-for-distance behavior (when not shooting).

## Dashboard keys for tuning

Use SmartDashboard/NetworkTables keys:

- `Shooter/Tuning/Enabled` (bool)
- `Shooter/Tuning/LeftRPM` (number)
- `Shooter/Tuning/RightRPM` (number)
- `Shooter/Tuning/HoodDeg` (number)
- `Shooter/Tuning/FeedKicker` (bool)
- `Shooter/Tuning/KickerTorqueAmps` (number)

## How to live tune in sim

1. Start sim (`./gradlew simulateJava`).
2. Set `Shooter/Tuning/Enabled = true`.
3. Set initial values for `LeftRPM`, `RightRPM`, and `HoodDeg`.
4. Hold `driverController.rightTrigger` to run tuning.
5. If you want to fire while tuning, set `Shooter/Tuning/FeedKicker = true`.
6. Adjust RPM/hood until shots land in the hub from your test distance.
7. Record final values and copy them into `ShooterConstants` shot map arrays:
   - `SHOT_MAP_DISTANCE_METERS`
   - `SHOT_MAP_LEFT_RPM`
   - `SHOT_MAP_RIGHT_RPM`
   - `SHOT_MAP_HOOD_ANGLE_DEG`
8. Set `Shooter/Tuning/Enabled = false` to return right trigger to normal shoot mode.

## FUEL simulation outputs

AdvantageKit logs:

- `sim/FUEL/poses` (`Pose3d[50]`)
- `sim/FUEL/activeCount`
- `sim/FUEL/diameterMeters`

Behavior:

- New shots spawn while shooter is actively firing.
- If a FUEL piece drops below field height (`z < 0`), it resets to `(0,0,0)` and is reused.
