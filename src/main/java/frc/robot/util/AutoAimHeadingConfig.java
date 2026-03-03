package frc.robot.util;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.Constants;

/** Shared heading-alignment tuning used by teleop auto-align and autonomous path overrides. */
public final class AutoAimHeadingConfig {
    public static final double HEADING_PROFILE_MAX_VELOCITY_RAD_PER_SEC = Math.toRadians(
            readPositiveDouble("autoAim.headingProfile.maxVelocityDegPerSec", 540.0));
    public static final double HEADING_PROFILE_MAX_ACCELERATION_RAD_PER_SEC2 = Math.toRadians(
            readPositiveDouble("autoAim.headingProfile.maxAccelerationDegPerSec2", 2160.0));

    public static TrapezoidProfile.Constraints createHeadingProfileConstraints() {
        return new TrapezoidProfile.Constraints(
                HEADING_PROFILE_MAX_VELOCITY_RAD_PER_SEC,
                HEADING_PROFILE_MAX_ACCELERATION_RAD_PER_SEC2);
    }

    public static final double AIM_TOLERANCE_RAD = Math.toRadians(
            readPositiveDouble("autoAim.aimToleranceDeg", Math.toDegrees(Constants.SHOOTING_AIM_TOLERANCE_RAD)));
    public static final double AIM_RELEASE_TOLERANCE_RAD = Math.toRadians(readPositiveDouble(
            "autoAim.aimReleaseToleranceDeg",
            Math.toDegrees(Constants.SHOOTING_AIM_RELEASE_TOLERANCE_RAD)));
    public static final double TARGET_HOLD_SEC = Constants.SHOOTING_AIM_TARGET_HOLD_SEC;

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
