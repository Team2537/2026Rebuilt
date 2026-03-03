package frc.robot.subsystems.transfer;

/** Constants for the transfer hardware implementation. */
public final class TransferConstants {
    // CAN ID for the transfer motor.
    public static final int TRANSFER_MOTOR_ID = 14;

    public static final boolean TRANSFER_INVERTED = true;
    public static final double SENSOR_TO_MECHANISM_RATIO = 1.333;

    public static final double RUN_TRANSFER_PERCENT = 0.35;

    public static final double STATUS_UPDATE_HZ = 50.0;
    public static final double STATOR_CURRENT_LIMIT_AMPS = 80.0;
    public static final double SUPPLY_CURRENT_LIMIT_AMPS = 50.0;
    public static final double JAM_CURRENT_THRESHOLD_AMPS = 40.0;
    public static final int JAM_DETECT_CYCLES = 8;
    public static final double JAM_NOTIFY_COOLDOWN_SEC = 1.5;

    private TransferConstants() {}
}
