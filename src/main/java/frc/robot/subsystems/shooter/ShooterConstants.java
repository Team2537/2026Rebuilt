package frc.robot.subsystems.shooter;

import edu.wpi.first.math.util.Units;

/** Constants for the shooter hardware implementation. */
public final class ShooterConstants {
    // TODO: Set these IDs to match your wiring.
    public static final int LEFT_SHOOTER_MOTOR_ID = 10;
    public static final int RIGHT_SHOOTER_MOTOR_ID = 11;
    public static final int HOOD_MOTOR_ID = 12;
    public static final int KICKER_MOTOR_ID = 13;

    public static final boolean LEFT_SHOOTER_INVERTED = false;
    public static final boolean RIGHT_SHOOTER_INVERTED = true;
    public static final boolean HOOD_INVERTED = false;
    public static final boolean KICKER_INVERTED = false;

    public static final double SHOOTER_SENSOR_TO_MECHANISM_RATIO = 1.125;
    public static final double HOOD_SENSOR_TO_MECHANISM_RATIO = 72.0;
    public static final double KICKER_SENSOR_TO_MECHANISM_RATIO = 2.667;

    public static final double SHOOTER_MAX_RPM = 6200.0;
    public static final double SLOW_SHOOTER_RPM = 1000.0;
    public static final double KICKER_MAX_TORQUE_CURRENT_AMPS = 70.0;
    public static final double MAX_OUTPUT_VOLTS = 12.0;

    public static final double HOOD_MIN_ANGLE_RAD = Units.degreesToRadians(0.0);
    public static final double HOOD_MAX_ANGLE_RAD = Units.degreesToRadians(90.0);

    public static final double STATUS_UPDATE_HZ = 50.0;

    public static final double SHOOTER_RPM_TOLERANCE = 125.0;
    public static final double HOOD_ANGLE_TOLERANCE_RAD = Units.degreesToRadians(3.0);
    public static final double DEFAULT_KICKER_TORQUE_AMPS = 45.0;
    public static final int HUB_TAG_ID = 26;
    public static final double HUB_TARGET_X_OFFSET_METERS = Units.inchesToMeters(26.0);

    /**
     * Seed shot map for interpolation.
     * Replace with measured values from characterization once available.
     */
    public static final double[] SHOT_MAP_DISTANCE_METERS = {2.0, 3.0, 4.0, 5.0, 7.0, 9.0};
    public static final double[] SHOT_MAP_LEFT_RPM = {1400.0, 1600.0, 1600.0, 1800.0, 1950.0, 2200.0};
    public static final double[] SHOT_MAP_RIGHT_RPM = { 1400.0, 1600.0, 1600.0, 1800.0, 1950.0, 2200.0 };
    public static final double[] SHOT_MAP_HOOD_ANGLE_DEG = {80.0, 72.0, 64.0, 65.0, 53.0, 52.0};
    public static final double[] SHOT_TIME_IN_AIR_SECONDS = {1.55, 1.66, 1.78, 1.93, 2.21, 2.55};

    // Closed-loop gains (starting points, tune on robot).
    public static final double SHOOTER_KP = 10.0;
    public static final double SHOOTER_KI = 0.0;
    public static final double SHOOTER_KD = 0.0;
    public static final double SHOOTER_KS = 0.0;
    public static final double SHOOTER_KV = 0.12;

    public static final double HOOD_KP = 30.0;
    public static final double HOOD_KI = 0.0;
    public static final double HOOD_KD = 0.0;
    public static final double HOOD_KS = 0.0;
    public static final double HOOD_KV = 0.0;

    public static final double SHOOTER_STATOR_CURRENT_LIMIT_AMPS = 120.0;
    public static final double SHOOTER_SUPPLY_CURRENT_LIMIT_AMPS = 70.0;
    public static final double HOOD_STATOR_CURRENT_LIMIT_AMPS = 60.0;
    public static final double HOOD_SUPPLY_CURRENT_LIMIT_AMPS = 40.0;
    public static final double KICKER_STATOR_CURRENT_LIMIT_AMPS = 70.0;
    public static final double KICKER_SUPPLY_CURRENT_LIMIT_AMPS = 45.0;

    private ShooterConstants() {}
}
