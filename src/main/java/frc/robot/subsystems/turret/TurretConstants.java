package frc.robot.subsystems.turret;

/** Constants for turret control and simulation. */
public final class TurretConstants {
    public static final double KP = 30.0;
    public static final double KI = 0.0;
    public static final double KD = 0.2;

    public static final double KS = 0.2;
    public static final double KV = 1.0;

    public static final double MAX_VELOCITY_RAD_PER_SEC = 15.0;
    public static final double MAX_ACCEL_RAD_PER_SEC_SQ = 30.0;
    public static final double POSITION_TOLERANCE_RAD = Math.toRadians(1.5);
    public static final double VELOCITY_TOLERANCE_RAD_PER_SEC = Math.toRadians(5.0);

    public static final double MAX_VOLTAGE = 12.0;

    public static final double GEAR_RATIO = 100.0;
    public static final double MOMENT_OF_INERTIA_KG_M2 = 0.004;

    private TurretConstants() {
    }
}
