package frc.robot;

import java.util.EnumSet;

/** Robot-wide constants expressed in standard units. */
public final class Constants {
    public static final String DRIVETRAIN_CAN_BUS = "canivore";
    public static final String MECHANISM_CAN_BUS = "rio";

    /** Selects how teleop auto-align computes heading. */
    public enum AutoAlignMode {
        /** Use full-field pose estimate + motion-compensated hub heading. */
        POSE_ODOMETRY,
        /** Use robot pose estimated from hub tags only. */
        HUB_TAGS_ONLY
    }

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

    /** Toggleable auto-align mode for right-bumper aiming in teleop. */
    // public static final AutoAlignMode TELEOP_AUTO_ALIGN_MODE = AutoAlignMode.POSE_ODOMETRY;
    public static final AutoAlignMode TELEOP_AUTO_ALIGN_MODE = AutoAlignMode.HUB_TAGS_ONLY;
    public static final FeedGateMode SHOOTING_FEED_GATE_MODE = FeedGateMode.SHOOTER_AND_AIM;
    public static final double SHOOTING_AIM_TOLERANCE_RAD = Math.toRadians(1.0);

    public static boolean isMechanismEnabled(Mechanism mechanism) {
        return ENABLED_MECHANISMS.contains(mechanism);
    }
}
