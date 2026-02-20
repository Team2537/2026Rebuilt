package frc.robot.subsystems.transfer;

/** Constants for the transfer hardware implementation. */
public final class TransferConstants {
    // TODO: Set to match wiring.
    public static final int TRANSFER_MOTOR_ID = 14;

    public static final boolean TRANSFER_INVERTED = false;
    public static final double SENSOR_TO_MECHANISM_RATIO = 1.333;

    public static final double DEFAULT_TRANSFER_PERCENT = 0.0;
    public static final double RUN_TRANSFER_PERCENT = 0.35;

    public static final double STATUS_UPDATE_HZ = 50.0;
    public static final double STATOR_CURRENT_LIMIT_AMPS = 80.0;
    public static final double SUPPLY_CURRENT_LIMIT_AMPS = 50.0;

    private TransferConstants() {}
}
