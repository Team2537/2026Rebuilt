package frc.robot.subsystems.intake;

public final class IntakeConstants {
    private IntakeConstants() {}

    public static final int ROLLER_MOTOR_ID = 25;
    public static final int LEFT_MOTOR_ID = 23;
    public static final int RIGHT_MOTOR_ID = 24;

    public static final double ROLLER_RPM = 4000.0;
    public static final double SLOW_ROLLER_RPM = 1500.0;
    public static final double ROLLER_MAX_RPM = 6200.0;
    public static final double STATUS_UPDATE_HZ = 50.0;

    public static final double ROLLER_SENSOR_TO_MECHANISM_RATIO = 2.667;
    public static final double ROLLER_KP = 10.0;
    public static final double ROLLER_KI = 0.0;
    public static final double ROLLER_KD = 0.0;
    public static final double ROLLER_KS = 0.0;
    public static final double ROLLER_KV = 0.12;

    public static final double INTAKE_KP = 4.0;
    public static final double INTAKE_KI = 0.5;
    public static final double INTAKE_KD = 0.0;
    public static final double INTAKE_KS = 0.0;
    public static final double INTAKE_KV = 0.0;

    public static final boolean LEFT_INTAKE_INVERTED = true;
    public static final boolean RIGHT_OPPOSES_LEFT = true;
    public static final boolean ROLLER_INVERTED = false;

    public static final double ROLLER_STATOR_CURRENT_LIMIT_AMPS = 120.0;
    public static final double ROLLER_SUPPLY_CURRENT_LIMIT_AMPS = 120.0;
    public static final double INTAKE_STATOR_CURRENT_LIMIT_AMPS = 70.0;
    public static final double INTAKE_SUPPLY_CURRENT_LIMIT_AMPS = 70.0;

    public static final double INTAKE_STATOR_CURRENT_LIMIT_AMPS_LOW = 15.0;
    public static final double INTAKE_SUPPLY_CURRENT_LIMIT_AMPS_LOW = 15.0;

    public static final double INTAKE_VELOCITY = 100.0;
    public static final double INTAKE_ACCELERATION = 400.0;

    public static final double SLOW_INTAKE_VELOCITY = 20.0;
    public static final double SLOW_INTAKE_ACCELERATION = 50.0;

    public static final double TRAVEL_IN_PER_MOTOR_REV = 0.5233;
    public static final double MOTOR_REV_PER_IN = 1.0 / TRAVEL_IN_PER_MOTOR_REV;
    public static final double FULL_TRAVEL_IN = 11.0;
    public static final double RETRACTED_POSITION_ROT = 6.0;
    public static final double EXTENDED_POSITION_ROT = FULL_TRAVEL_IN * MOTOR_REV_PER_IN;
    public static final double DRIVER_TRIGGER_WIGGLE_BASELINE_IN = FULL_TRAVEL_IN;
    public static final double DRIVER_TRIGGER_WIGGLE_PEAK_IN = 11.4;
    public static final double DRIVER_TRIGGER_WIGGLE_BASELINE_ROT =
            DRIVER_TRIGGER_WIGGLE_BASELINE_IN * MOTOR_REV_PER_IN;
    public static final double DRIVER_TRIGGER_WIGGLE_PEAK_ROT =
            DRIVER_TRIGGER_WIGGLE_PEAK_IN * MOTOR_REV_PER_IN;
    public static final double DRIVER_TRIGGER_WIGGLE_SWITCH_INTERVAL_SEC = 0.5;
    public static final double POSITION_TOLERANCE_ROT = 0.25;
    public static final double MOVE_TIMEOUT_SEC = 1.5;
    public static final double INTAKE_MAX_VOLTS = 12.0;
    public static final double SLOW_RETRACT_MAX_VOLTS = 4.0;

    public static final double HOMING_CURRENT_THRESHOLD_AMPS = 20.0;
    public static final double HOMING_VOLTAGE = -1.5;
    public static final double HOMING_WAIT_TIMEOUT_SEC = 4.0;

    // Smart-retract will only drive intake as far as this value (unless regular retract is used).
    public static final double SMART_RETRACT_RETRACTED_POSITION_ROT = 2.0;
    public static final double SMART_RETRACT_HALF_RETRACT_POSITION_ROT = 10.0;
    public static final double SMART_RETRACT_CURRENT_FILTER_ALPHA = 0.20;
    public static final int SMART_RETRACT_FEED_ENGAGE_CYCLES = 2;
    public static final double SMART_RETRACT_FEED_START_DELAY_SEC = 0.2;

    public static final double SMART_RETRACT_NIBBLE_CURRENT_DELTA_AMPS = 2.0;
    public static final int SMART_RETRACT_NIBBLE_DETECT_CYCLES = 3;
    public static final double SMART_RETRACT_NIBBLE_STEP_ROT = 0.40;
    public static final double SMART_RETRACT_NIBBLE_BACKOFF_ROT = 6.0;
    public static final double SMART_RETRACT_NIBBLE_BACKOFF_DWELL_SEC = 0.3;
}
