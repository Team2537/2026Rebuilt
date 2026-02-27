package frc.robot;

import java.util.EnumSet;

/** Robot-wide constants expressed in standard units. */
public final class Constants {
    public static final String DRIVETRAIN_CAN_BUS = "canivore";
    public static final String MECHANISM_CAN_BUS = "rio";

    /** Selects when the feed path is allowed to open during shooting. */
    public enum FeedGateMode {
        IMMEDIATE,
        SHOOTER_AT_SETPOINT,
        SHOOTER_AND_AIM
    }

    public enum Mechanism {
        DRIVE,
        VISION,
        SHOOTER,
        TRANSFER,
        INTAKE
    }

    private Constants() {}

    /**
     * Enable/disable robot mechanisms at init time.
     */
    public static final EnumSet<Mechanism> ENABLED_MECHANISMS =
            EnumSet.of(Mechanism.DRIVE, Mechanism.SHOOTER, Mechanism.TRANSFER, Mechanism.VISION, Mechanism.INTAKE);

    public static final FeedGateMode SHOOTING_FEED_GATE_MODE = FeedGateMode.SHOOTER_AND_AIM;
    public static final double SHOOTING_AIM_TOLERANCE_RAD = Math.toRadians(3.0);
    public static final double SHOOTING_AIM_RELEASE_TOLERANCE_RAD = Math.toRadians(4.0);
    public static final double SHOOTING_AIM_TARGET_HOLD_SEC = 0.12;

    /** Enables decimation for expensive per-loop telemetry/logging paths. */
    public static final boolean ENABLE_PERF_LOG_DECIMATION = true;
    /** Must stay a power of two when used with bitmask-based cycle checks. */
    public static final int PERF_LOG_DECIMATION_CYCLES = 8;

    public static boolean isMechanismEnabled(Mechanism mechanism) {
        return ENABLED_MECHANISMS.contains(mechanism);
    }
}
