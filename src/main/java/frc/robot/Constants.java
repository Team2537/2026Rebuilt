package frc.robot;

/** Robot-wide constants expressed in standard units. */
public final class Constants {
    public static final String DRIVETRAIN_CAN_BUS = "canivore";
    public static final String MECHANISM_CAN_BUS = "rio";

    private Constants() {}

    /** Enables decimation for expensive per-loop telemetry/logging paths. */
    public static final boolean ENABLE_PERF_LOG_DECIMATION = false;
    /** Must stay a power of two when used with bitmask-based cycle checks. */
    public static final int PERF_LOG_DECIMATION_CYCLES = 8;
}
