package frc.robot.coordination.shooting;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.shooter.LaunchCalculator;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.util.FieldConstants;

/** Builds unified shot solutions from score/pass target geometry and motion compensation. */
public final class ShotSolutionCalculator {
    private final Shooter shooter;

    public ShotSolutionCalculator(Shooter shooter) {
        this.shooter = shooter;
    }

    public HeadingRateTracker createHeadingRateTracker() {
        return new HeadingRateTracker();
    }

    public ShotSolution createHubScoreSolution(
            LaunchCalculator.MotionCompensation compensation,
            boolean movingShot,
            double headingToleranceRad,
            double headingReleaseToleranceRad,
            double shooterRpmTolerance) {
        Pose2d targetPose = new Pose2d(FieldConstants.getHubTargetTranslation(), Rotation2d.kZero);
        if (compensation == null) {
            return ShotSolution.invalid(targetPose);
        }
        return createScoreLikeSolution(
                ShotIntent.SCORE,
                movingShot,
                targetPose,
                compensation.compensatedDistanceMeters(),
                compensation.compensatedHeading(),
                compensation.desiredRobotHeading(),
                compensation.desiredHeadingRateRadPerSec(),
                headingToleranceRad,
                headingReleaseToleranceRad,
                shooterRpmTolerance);
    }

    public ShotSolution createScoreSolution(
            Pose2d targetPose,
            double distanceMeters,
            Rotation2d targetHeading,
            HeadingRateTracker headingRateTracker,
            boolean movingShot,
            double headingToleranceRad,
            double headingReleaseToleranceRad,
            double shooterRpmTolerance) {
        return createScoreLikeSolution(
                ShotIntent.SCORE,
                movingShot,
                targetPose,
                distanceMeters,
                targetHeading,
                targetHeading != null ? targetHeading.plus(Rotation2d.kPi) : null,
                headingRateTracker.update(targetHeading != null ? targetHeading.plus(Rotation2d.kPi) : null),
                headingToleranceRad,
                headingReleaseToleranceRad,
                shooterRpmTolerance);
    }

    public ShotSolution createPassSolution(
            Pose2d robotPose,
            HeadingRateTracker headingRateTracker,
            double headingToleranceRad,
            double headingReleaseToleranceRad,
            double shooterRpmTolerance) {
        Pose2d targetPose = FieldConstants.getPassTargetPose(robotPose);
        double distanceMeters = getShooterDistanceToTarget(robotPose, targetPose.getTranslation());
        Rotation2d targetHeading = FieldConstants.getHeadingToTarget(robotPose, targetPose.getTranslation());
        return createScoreLikeSolution(
                ShotIntent.PASS,
                false,
                targetPose,
                distanceMeters,
                targetHeading,
                targetHeading != null ? targetHeading.plus(Rotation2d.kPi) : null,
                headingRateTracker.update(targetHeading != null ? targetHeading.plus(Rotation2d.kPi) : null),
                headingToleranceRad,
                headingReleaseToleranceRad,
                shooterRpmTolerance);
    }

    private ShotSolution createScoreLikeSolution(
            ShotIntent intent,
            boolean movingShot,
            Pose2d targetPose,
            double distanceMeters,
            Rotation2d targetHeading,
            Rotation2d desiredRobotHeading,
            double desiredHeadingRateRadPerSec,
            double headingToleranceRad,
            double headingReleaseToleranceRad,
            double shooterRpmTolerance) {
        if (!Double.isFinite(distanceMeters) || targetHeading == null || desiredRobotHeading == null) {
            return ShotSolution.invalid(targetPose);
        }

        Shooter.ShotSetpoint shooterSetpoint = shooter.calculateSetpointForDistance(distanceMeters);
        if (shooterSetpoint == null
                || !Double.isFinite(shooterSetpoint.leftRpm())
                || !Double.isFinite(shooterSetpoint.rightRpm())
                || !Double.isFinite(shooterSetpoint.hoodAngleRad())) {
            return ShotSolution.invalid(targetPose);
        }

        return new ShotSolution(
                true,
                intent,
                movingShot,
                targetPose != null ? targetPose : new Pose2d(),
                distanceMeters,
                targetHeading,
                desiredRobotHeading,
                desiredHeadingRateRadPerSec,
                headingToleranceRad,
                headingReleaseToleranceRad,
                shooterRpmTolerance,
                shooterSetpoint);
    }

    public static double getShooterDistanceToTarget(Pose2d robotPose, Translation2d targetTranslation) {
        if (robotPose == null || targetTranslation == null) {
            return Double.NaN;
        }
        Translation2d shooterFieldPosition = robotPose.getTranslation().plus(
                ShooterConstants.ROBOT_TO_SHOOTER_OFFSET.rotateBy(robotPose.getRotation()));
        return shooterFieldPosition.getDistance(targetTranslation);
    }

    /** Tracks heading rate from successive desired-heading samples. */
    public static final class HeadingRateTracker {
        private LinearFilter headingRateFilter = LinearFilter.movingAverage(LaunchCalculator.currentHeadingRateFilterTaps());
        private int lastFilterTaps = LaunchCalculator.currentHeadingRateFilterTaps();
        private Rotation2d lastHeading = null;
        private double lastTimestampSec = Double.NaN;

        public double update(Rotation2d heading) {
            if (heading == null) {
                reset();
                return 0.0;
            }
            int desiredFilterTaps = LaunchCalculator.currentHeadingRateFilterTaps();
            if (desiredFilterTaps != lastFilterTaps) {
                headingRateFilter = LinearFilter.movingAverage(desiredFilterTaps);
                lastFilterTaps = desiredFilterTaps;
            }
            double nowSec = Timer.getFPGATimestamp();
            double headingRateRadPerSec = 0.0;
            if (lastHeading != null && Double.isFinite(lastTimestampSec)) {
                double dtSec = nowSec - lastTimestampSec;
                if (dtSec > 1e-6 && dtSec < 0.1) {
                    double rawRateRadPerSec = MathUtil.angleModulus(heading.minus(lastHeading).getRadians()) / dtSec;
                    headingRateRadPerSec = MathUtil.clamp(
                            headingRateFilter.calculate(rawRateRadPerSec),
                            -LaunchCalculator.currentMaxHeadingRateRadPerSec(),
                            LaunchCalculator.currentMaxHeadingRateRadPerSec());
                }
            } else {
                headingRateFilter.reset();
            }
            lastHeading = heading;
            lastTimestampSec = nowSec;
            return headingRateRadPerSec;
        }

        public void reset() {
            headingRateFilter.reset();
            lastHeading = null;
            lastTimestampSec = Double.NaN;
        }
    }
}
