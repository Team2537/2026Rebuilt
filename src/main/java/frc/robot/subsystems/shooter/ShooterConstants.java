package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Translation2d;
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
    public static final double SLOW_SHOOTER_RPM = isReal ? 2000.0 : 500.0;
    public static final double KICKER_MAX_TORQUE_CURRENT_AMPS = 70.0;
    public static final double MAX_OUTPUT_VOLTS = 12.0;
    public static final double SHOOTER_SYSID_STEP_VOLTAGE_VOLTS = 6.0;
    public static final double SHOOTER_SYSID_TIMEOUT_SEC = 10.0;

    public static final double HOOD_MIN_ANGLE_RAD = Units.degreesToRadians(0.0);
    public static final double HOOD_MAX_ANGLE_RAD = Units.degreesToRadians(90.0);

    public static final double STATUS_UPDATE_HZ = 50.0;

    public static final double SHOOTER_RPM_TOLERANCE = 200.0;
    public static final double HOOD_ANGLE_TOLERANCE_RAD = Units.degreesToRadians(1.0);
    public static final double DEFAULT_KICKER_TORQUE_AMPS = 65.0;
    public static final double MOTION_COMP_TIME_SCALE = 1.0;

    /**
     * Shooter position relative to robot center, in robot coordinates (x=forward, y=left).
     * Negative X because the shooter fires backward (muzzle is behind robot center).
     * Used for velocity transform (accounting for angular velocity at the shooter)
     * and for 3D pose visualization in AdvantageKit.
     */
    public static final Translation2d ROBOT_TO_SHOOTER_OFFSET = new Translation2d(-0.32, 0.0);

    /** Height of the shooter above the ground (meters), for 3D pose visualization. */
    public static final double SHOOTER_HEIGHT_METERS = 0.6;

    /**
     * Phase delay in seconds to compensate for system latency (sensor processing,
     * network transmission, actuator response). Applied before the iterative
     * time-distance convergence loop to project the robot pose forward.
     */
    public static final double PHASE_DELAY_SEC = 0.03;

    /**
     * Seed shot map for interpolation.
     * Replace with measured values from characterization once available.
     */

    private static final double[] SHOT_MAP_DISTANCE_METERS_SIM = {0.0, 2.0, 3.0, 4.0, 5.0, 7.0, 9.0};
    private static final double[] SHOT_MAP_LEFT_RPM_SIM = { 1400.0, 1400.0, 1600.0, 1600.0, 1800.0, 1975.0, 2200.0};
    private static final double[] SHOT_MAP_RIGHT_RPM_SIM = { 1400.0, 1400.0, 1600.0, 1600.0, 1800.0, 1975.0, 2200.0 };
    private static final double[] SHOT_MAP_HOOD_ANGLE_DEG_SIM = {88.0, 78.5, 72.2, 62.2, 64.1, 55.7, 53.9};
    // Physically computed from sim ballistic physics:
    // t_descent = (vz + sqrt(vz^2 - 2*g*(1.83 - 0.68))) / g
    // where vz = muzzle_speed * sin(hood_angle), muzzle_speed = RPM * 2pi/60 * 0.0508m * 0.85
    private static final double[] SHOT_TIME_IN_AIR_SECONDS_SIM = {1.24, 1.204, 1.211, 1.09, 1.315, 1.328, 1.481};

    private static final double[] SHOT_MAP_DISTANCE_METERS_REAL = { 1.4, 2.04, 3.0, 4.0, 5.0, 6.0, 7.0 };
    private static final double[] SHOT_MAP_LEFT_RPM_REAL = { 3250.0, 3250.0, 3400.0, 3650.0, 3950.0, 4200.0, 4550.0 };
    private static final double[] SHOT_MAP_RIGHT_RPM_REAL = { 3250.0, 3250.0, 3400.0, 3650.0, 3950.0, 4200.0, 4550.0 };
    private static final double[] SHOT_MAP_HOOD_ANGLE_DEG_REAL = {6.0, 12.0, 15.5, 19.0, 22.0, 25.0, 28.0};
    private static final double[] SHOT_TIME_IN_AIR_SECONDS_REAL = {1.24, 1.14, 1.13, 1.20, 1.24, 1.36, 1.41};

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
    public static final double SHOOTER_KS = 0.1876;
    public static final double SHOOTER_KV = 0.002;

    public static final double HOOD_KP = 200.0;
    public static final double HOOD_KI = 0.0;
    public static final double HOOD_KD = 5.0;
    public static final double HOOD_KS = 0.34;
    public static final double HOOD_KV = 7.0;
    public static final double HOOD_KG = 0.04;

    public static final double SHOOTER_STATOR_CURRENT_LIMIT_AMPS = 120.0;
    public static final double SHOOTER_SUPPLY_CURRENT_LIMIT_AMPS = 70.0;
    public static final double HOOD_STATOR_CURRENT_LIMIT_AMPS = 60.0;
    public static final double HOOD_SUPPLY_CURRENT_LIMIT_AMPS = 40.0;
    public static final double KICKER_STATOR_CURRENT_LIMIT_AMPS = 70.0;
    public static final double KICKER_SUPPLY_CURRENT_LIMIT_AMPS = 45.0;

    public static final double HOMING_CURRENT_THRESHOLD_AMPS = 25.0;
    public static final double HOMING_WAIT_TIMEOUT_SEC = 3.0;


    private ShooterConstants() {}
}
