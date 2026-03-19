package frc.robot.coordination.shooting;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.shooter.Shooter;

/** Single-source shot solution shared by drive aiming, shooter tracking, and feed gating. */
public record ShotSolution(
        boolean valid,
        ShotIntent intent,
        boolean movingShot,
        Pose2d targetPose,
        double distanceMeters,
        Rotation2d targetHeading,
        Rotation2d desiredRobotHeading,
        double desiredHeadingRateRadPerSec,
        double headingToleranceRad,
        double headingReleaseToleranceRad,
        double shooterRpmTolerance,
        Shooter.ShotSetpoint shooterSetpoint) {
    public static ShotSolution invalid(Pose2d targetPose) {
        return new ShotSolution(
                false,
                ShotIntent.NONE,
                false,
                targetPose != null ? targetPose : new Pose2d(),
                Double.NaN,
                null,
                null,
                0.0,
                Double.NaN,
                Double.NaN,
                Double.NaN,
                null);
    }
}
