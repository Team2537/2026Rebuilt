package frc.robot.util;

import edu.wpi.first.math.trajectory.TrapezoidProfile;

/** Shared heading-alignment tuning used by teleop auto-align and autonomous path overrides. */
public final class AutoAimHeadingConfig {
    private static final double DEFAULT_AIM_TOLERANCE_DEG = 1.5;
    private static final double DEFAULT_AIM_RELEASE_TOLERANCE_DEG = 2.5;
    private static final double DEFAULT_SHOT_ON_MOVE_AIM_TOLERANCE_DEG = 3.0;
    private static final double DEFAULT_SHOT_ON_MOVE_AIM_RELEASE_TOLERANCE_DEG = 5.0;
    private static final double DEFAULT_PASS_AIM_TOLERANCE_DEG = 8.0;
    private static final double DEFAULT_PASS_AIM_RELEASE_TOLERANCE_DEG = 12.0;
    private static final double DEFAULT_TARGET_HOLD_SEC = 0.12;

    public static final double HEADING_PROFILE_MAX_VELOCITY_RAD_PER_SEC = Math.toRadians(
            readPositiveDouble("autoAim.headingProfile.maxVelocityDegPerSec", 360.0));
    public static final double HEADING_PROFILE_MAX_ACCELERATION_RAD_PER_SEC2 = Math.toRadians(
            readPositiveDouble("autoAim.headingProfile.maxAccelerationDegPerSec2", 1200.0));

    public static TrapezoidProfile.Constraints createHeadingProfileConstraints() {
        return new TrapezoidProfile.Constraints(
                HEADING_PROFILE_MAX_VELOCITY_RAD_PER_SEC,
                HEADING_PROFILE_MAX_ACCELERATION_RAD_PER_SEC2);
    }

    public static final double AIM_TOLERANCE_RAD = Math.toRadians(
            readPositiveDouble("autoAim.aimToleranceDeg", DEFAULT_AIM_TOLERANCE_DEG));
    public static final double AIM_RELEASE_TOLERANCE_RAD = Math.toRadians(readPositiveDouble(
            "autoAim.aimReleaseToleranceDeg",
            DEFAULT_AIM_RELEASE_TOLERANCE_DEG));
    public static final double SHOT_ON_MOVE_AIM_TOLERANCE_RAD = Math.toRadians(readPositiveDouble(
            "autoAim.shotOnMoveAimToleranceDeg",
            DEFAULT_SHOT_ON_MOVE_AIM_TOLERANCE_DEG));
    public static final double SHOT_ON_MOVE_AIM_RELEASE_TOLERANCE_RAD = Math.toRadians(readPositiveDouble(
            "autoAim.shotOnMoveAimReleaseToleranceDeg",
            DEFAULT_SHOT_ON_MOVE_AIM_RELEASE_TOLERANCE_DEG));
    public static final double PASS_AIM_TOLERANCE_RAD = Math.toRadians(readPositiveDouble(
            "autoAim.passAimToleranceDeg",
            DEFAULT_PASS_AIM_TOLERANCE_DEG));
    public static final double PASS_AIM_RELEASE_TOLERANCE_RAD = Math.toRadians(readPositiveDouble(
            "autoAim.passAimReleaseToleranceDeg",
            DEFAULT_PASS_AIM_RELEASE_TOLERANCE_DEG));
    public static final double TARGET_HOLD_SEC = readPositiveDouble("autoAim.targetHoldSec", DEFAULT_TARGET_HOLD_SEC);

    private static double readPositiveDouble(String propertyKey, double defaultValue) {
        String raw = System.getProperty(propertyKey);
        if (raw == null || raw.isBlank()) {
            return defaultValue;
        }
        try {
            double parsed = Double.parseDouble(raw);
            if (!Double.isFinite(parsed) || parsed <= 0.0) {
                throw new IllegalArgumentException(
                        "Expected " + propertyKey + " > 0 and finite, got " + raw);
            }
            return parsed;
        } catch (NumberFormatException exception) {
            throw new IllegalArgumentException(
                    "Expected numeric value for " + propertyKey + ", got " + raw,
                    exception);
        }
    }

    private AutoAimHeadingConfig() {}
}
