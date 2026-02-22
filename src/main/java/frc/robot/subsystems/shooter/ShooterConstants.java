package frc.robot.subsystems.shooter;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;

/** Constants for the shooter hardware implementation. */
public final class ShooterConstants {
    private static final boolean isReal = RobotBase.isReal();

    // CAN IDs for the shooter assembly.
    public static final int LEFT_SHOOTER_MOTOR_ID = 10;
    public static final int RIGHT_SHOOTER_MOTOR_ID = 11;
    public static final int HOOD_MOTOR_ID = 12;
    public static final int KICKER_MOTOR_ID = 13;

    public static final boolean LEFT_SHOOTER_INVERTED = false;
    public static final boolean RIGHT_SHOOTER_INVERTED = true;
    public static final boolean HOOD_INVERTED = true;
    public static final boolean KICKER_INVERTED = true;

    public static final double SHOOTER_SENSOR_TO_MECHANISM_RATIO = 1.125;
    public static final double HOOD_SENSOR_TO_MECHANISM_RATIO = 72.0;
    public static final double KICKER_SENSOR_TO_MECHANISM_RATIO = 2.667;

    public static final double SHOOTER_MAX_RPM = 6200.0;
    public static final double SLOW_SHOOTER_RPM = 4000.0;
    public static final double KICKER_MAX_TORQUE_CURRENT_AMPS = 70.0;
    public static final double MAX_OUTPUT_VOLTS = 12.0;

    public static final double HOOD_MIN_ANGLE_RAD = Units.degreesToRadians(0.0);
    public static final double HOOD_MAX_ANGLE_RAD = Units.degreesToRadians(90.0);

    public static final double STATUS_UPDATE_HZ = 50.0;

    public static final double SHOOTER_RPM_TOLERANCE = 125.0;
    public static final double HOOD_ANGLE_TOLERANCE_RAD = Units.degreesToRadians(3.0);
    public static final double DEFAULT_KICKER_TORQUE_AMPS = 65.0;

    /**
     * Seed shot map for interpolation.
     * Replace with measured values from characterization once available.
     */

    private static final double[] SHOT_MAP_DISTANCE_METERS_SIM = {0.0, 2.0, 3.0, 4.0, 5.0, 7.0, 9.0};
    private static final double[] SHOT_MAP_LEFT_RPM_SIM = { 1400.0, 1400.0, 1600.0, 1600.0, 1800.0, 1950.0, 2200.0};
    private static final double[] SHOT_MAP_RIGHT_RPM_SIM = { 1400.0, 1400.0, 1600.0, 1600.0, 1800.0, 1950.0, 2200.0 };
    private static final double[] SHOT_MAP_HOOD_ANGLE_DEG_SIM = {88.0, 80.0, 72.0, 64.0, 65.0, 53.0, 52.0};
    private static final double[] SHOT_TIME_IN_AIR_SECONDS_SIM = {1.55, 1.55, 1.66, 1.78, 1.93, 2.21, 2.55};

    private static final double[] SHOT_MAP_DISTANCE_METERS_REAL = {0.0, 20.0};
    private static final double[] SHOT_MAP_LEFT_RPM_REAL = {4000.0, 4000.0};
    private static final double[] SHOT_MAP_RIGHT_RPM_REAL = { 4000.0, 4000.0 };
    private static final double[] SHOT_MAP_HOOD_ANGLE_DEG_REAL = {0.0, 0.0};
    private static final double[] SHOT_TIME_IN_AIR_SECONDS_REAL = {1.55, 1.55};

    public static final double[] SHOT_MAP_DISTANCE_METERS =
        isReal ? SHOT_MAP_DISTANCE_METERS_REAL : SHOT_MAP_DISTANCE_METERS_SIM;
    public static final double[] SHOT_MAP_LEFT_RPM = isReal ? SHOT_MAP_LEFT_RPM_REAL : SHOT_MAP_LEFT_RPM_SIM;
    public static final double[] SHOT_MAP_RIGHT_RPM = isReal ? SHOT_MAP_RIGHT_RPM_REAL : SHOT_MAP_RIGHT_RPM_SIM;
    public static final double[] SHOT_MAP_HOOD_ANGLE_DEG =
        isReal ? SHOT_MAP_HOOD_ANGLE_DEG_REAL : SHOT_MAP_HOOD_ANGLE_DEG_SIM;
    public static final double[] SHOT_TIME_IN_AIR_SECONDS =
        isReal ? SHOT_TIME_IN_AIR_SECONDS_REAL : SHOT_TIME_IN_AIR_SECONDS_SIM;

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
