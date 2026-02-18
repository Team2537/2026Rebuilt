package frc.robot.subsystems.intake;

public final class IntakeConstants {
    private IntakeConstants() {}

    public static final int ROLLER_MOTOR_ID = 0;
    public static final int LEFT_MOTOR_ID = 0;
    public static final int RIGHT_MOTOR_ID = 0;

    public static final double ROLLER_RPM = 0.0;
    public static final double STATUS_UPDATE_HZ = 50.0;

    public static final double INTAKE_KP = 8.0;
    public static final double INTAKE_KI = 0.0;
    public static final double INTAKE_KD = 0.0;
    public static final double INTAKE_KS = 0.0;
    public static final double INTAKE_KV = 0.0;

    public static final boolean LEFT_INTAKE_INVERTED = false;
    public static final boolean RIGHT_OPPOSES_LEFT = false;
    public static final boolean ROLLER_INVERTED = false;

    public static final double ROLLER_STATOR_CURRENT_LIMIT_AMPS = 70.0;
    public static final double ROLLER_SUPPLY_CURRENT_LIMIT_AMPS = 70.0;
    public static final double INTAKE_STATOR_CURRENT_LIMIT_AMPS = 70.0;
    public static final double INTAKE_SUPPLY_CURRENT_LIMIT_AMPS = 70.0;

    public static final double INTAKE_VELOCITY = 20.0;
    public static final double INTAKE_ACCELERATION = 80.0;

    public static final double TRAVEL_IN_PER_MOTOR_REV = 0.2326;
    public static final double MOTOR_REV_PER_IN = 1.0 / TRAVEL_IN_PER_MOTOR_REV;
    public static final double FULL_TRAVEL_IN = 16.65;
    public static final double RETRACTED_POSITION_ROT = 0.0;
    public static final double EXTENDED_POSITION_ROT = FULL_TRAVEL_IN * MOTOR_REV_PER_IN;
}
