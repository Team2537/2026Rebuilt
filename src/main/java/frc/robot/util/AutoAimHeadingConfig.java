package frc.robot.util;

import edu.wpi.first.math.trajectory.TrapezoidProfile;

/** Shared heading-alignment tuning used by teleop auto-align and autonomous path overrides. */
public final class AutoAimHeadingConfig {
    private static final LoggedTunableNumber headingProfileMaxVelocityDegPerSec =
            new LoggedTunableNumber("AutoAim/HeadingProfileMaxVelocityDegPerSec", 270.0);
    private static final LoggedTunableNumber headingProfileMaxAccelerationDegPerSec2 =
            new LoggedTunableNumber("AutoAim/HeadingProfileMaxAccelerationDegPerSec2", 900.0);
    private static final LoggedTunableNumber aimToleranceDeg =
            new LoggedTunableNumber("AutoAim/AimToleranceDeg", 1.5);
    private static final LoggedTunableNumber aimReleaseToleranceDeg =
            new LoggedTunableNumber("AutoAim/AimReleaseToleranceDeg", 2.5);
    private static final LoggedTunableNumber shotOnMoveAimToleranceDeg =
            new LoggedTunableNumber("AutoAim/MovingAimToleranceDeg", 4.0);
    private static final LoggedTunableNumber shotOnMoveAimReleaseToleranceDeg =
            new LoggedTunableNumber("AutoAim/MovingAimReleaseToleranceDeg", 5.0);
    private static final LoggedTunableNumber passAimToleranceDeg =
            new LoggedTunableNumber("AutoAim/PassAimToleranceDeg", 8.0);
    private static final LoggedTunableNumber passAimReleaseToleranceDeg =
            new LoggedTunableNumber("AutoAim/PassAimReleaseToleranceDeg", 12.0);

    public static double headingProfileMaxVelocityRadPerSec() {
        return Math.toRadians(headingProfileMaxVelocityDegPerSec.get());
    }

    public static double headingProfileMaxAccelerationRadPerSec2() {
        return Math.toRadians(headingProfileMaxAccelerationDegPerSec2.get());
    }

    public static TrapezoidProfile.Constraints createHeadingProfileConstraints() {
        return new TrapezoidProfile.Constraints(
                headingProfileMaxVelocityRadPerSec(),
                headingProfileMaxAccelerationRadPerSec2());
    }

    public static double aimToleranceRad() {
        return Math.toRadians(aimToleranceDeg.get());
    }

    public static double aimReleaseToleranceRad() {
        return Math.toRadians(aimReleaseToleranceDeg.get());
    }

    public static double movingAimToleranceRad() {
        return Math.toRadians(shotOnMoveAimToleranceDeg.get());
    }

    public static double movingAimReleaseToleranceRad() {
        return Math.toRadians(shotOnMoveAimReleaseToleranceDeg.get());
    }

    public static double passAimToleranceRad() {
        return Math.toRadians(passAimToleranceDeg.get());
    }

    public static double passAimReleaseToleranceRad() {
        return Math.toRadians(passAimReleaseToleranceDeg.get());
    }

    private AutoAimHeadingConfig() {}
}
