package frc.robot.subsystems.intake;

public final class IntakeConstants {
    private IntakeConstants() {}

    public static final int ROLLER_MOTOR_ID = 25;
    public static final int LEFT_MOTOR_ID = 23;
    public static final int RIGHT_MOTOR_ID = 24;

    public static final double ROLLER_RPM = 3000.0;
    public static final double SLOW_ROLLER_RPM = 1000.0;
    public static final double ROLLER_MAX_RPM = 6200.0;
    public static final double STATUS_UPDATE_HZ = 50.0;

    public static final double ROLLER_SENSOR_TO_MECHANISM_RATIO = 2.667;
    public static final double ROLLER_KP = 10.0;
    public static final double ROLLER_KI = 0.0;
    public static final double ROLLER_KD = 0.0;
    public static final double ROLLER_KS = 0.0;
    public static final double ROLLER_KV = 0.12;

    public static final double INTAKE_KP = 2.0;
    public static final double INTAKE_KI = 0.0;
    public static final double INTAKE_KD = 0.0;
    public static final double INTAKE_KS = 0.0;
    public static final double INTAKE_KV = 0.0;

    public static final boolean LEFT_INTAKE_INVERTED = true;
    public static final boolean RIGHT_OPPOSES_LEFT = true;
    public static final boolean ROLLER_INVERTED = false;

    public static final double ROLLER_STATOR_CURRENT_LIMIT_AMPS = 70.0;
    public static final double ROLLER_SUPPLY_CURRENT_LIMIT_AMPS = 70.0;
    public static final double INTAKE_STATOR_CURRENT_LIMIT_AMPS = 70.0;
    public static final double INTAKE_SUPPLY_CURRENT_LIMIT_AMPS = 70.0;

    public static final double INTAKE_VELOCITY = 400.0;
    public static final double INTAKE_ACCELERATION = 600.0;

    public static final double SLOW_INTAKE_VELOCITY = 200.0;
    public static final double SLOW_INTAKE_ACCELERATION = 300.0;

    public static final double TRAVEL_IN_PER_MOTOR_REV = 0.2326;
    public static final double MOTOR_REV_PER_IN = 1.0 / TRAVEL_IN_PER_MOTOR_REV;
    public static final double FULL_TRAVEL_IN = 10.65;
    public static final double RETRACTED_POSITION_ROT = 0.0;
    public static final double EXTENDED_POSITION_ROT = FULL_TRAVEL_IN * MOTOR_REV_PER_IN;

    public static final double HOMING_CURRENT_THRESHOLD_AMPS = 25.0;
}
