package frc.robot.coordination.shooting;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.subsystems.shooter.LaunchCalculator;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.util.FieldConstants;

/** Builds unified shot solutions from score/pass target geometry and motion compensation. */
public final class ShotSolutionCalculator {
    private static final LoggedTunableNumber maxHeadingRateDegPerSec =
            new LoggedTunableNumber("ShotSolution/MaxHeadingRateDegPerSec", 540.0);
    private static final LoggedTunableNumber headingRateFilterTaps =
            new LoggedTunableNumber("ShotSolution/HeadingRateFilterTaps", 3.0);

    private final Shooter shooter;

    public ShotSolutionCalculator(Shooter shooter) {
        this.shooter = shooter;
    }

    public HeadingRateTracker createHeadingRateTracker() {
        return new HeadingRateTracker();
    }

    public ShotSolution createHubScoreSolution(
            LaunchCalculator.MotionCompensation compensation,
            HeadingRateTracker headingRateTracker,
            boolean movingShot,
            double headingToleranceRad,
            double headingReleaseToleranceRad,
            double shooterRpmTolerance) {
        Pose2d targetPose = new Pose2d(FieldConstants.getHubTargetTranslation(), Rotation2d.kZero);
        if (compensation == null) {
            headingRateTracker.reset();
            return ShotSolution.invalid(targetPose);
        }
        return createScoreLikeSolution(
                ShotIntent.SCORE,
                movingShot,
                targetPose,
                compensation.compensatedDistanceMeters(),
                compensation.compensatedHeading(),
                headingRateTracker,
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
                headingRateTracker,
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
                headingRateTracker,
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
            HeadingRateTracker headingRateTracker,
            double headingToleranceRad,
            double headingReleaseToleranceRad,
            double shooterRpmTolerance) {
        if (!Double.isFinite(distanceMeters) || targetHeading == null) {
            headingRateTracker.reset();
            return ShotSolution.invalid(targetPose);
        }

        Shooter.ShotSetpoint shooterSetpoint = shooter.calculateSetpointForDistance(distanceMeters);
        if (shooterSetpoint == null
                || !Double.isFinite(shooterSetpoint.leftRpm())
                || !Double.isFinite(shooterSetpoint.rightRpm())
                || !Double.isFinite(shooterSetpoint.hoodAngleRad())) {
            headingRateTracker.reset();
            return ShotSolution.invalid(targetPose);
        }

        Rotation2d desiredRobotHeading = targetHeading.plus(Rotation2d.kPi);
        double desiredHeadingRateRadPerSec = headingRateTracker.update(desiredRobotHeading);
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
        private LinearFilter headingRateFilter = LinearFilter.movingAverage(currentHeadingRateFilterTaps());
        private int lastFilterTaps = currentHeadingRateFilterTaps();
        private Rotation2d lastHeading = null;
        private double lastTimestampSec = Double.NaN;

        public double update(Rotation2d heading) {
            if (heading == null) {
                reset();
                return 0.0;
            }
            int desiredFilterTaps = currentHeadingRateFilterTaps();
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
                    double maxHeadingRateRadPerSec = Math.toRadians(maxHeadingRateDegPerSec.get());
                    headingRateRadPerSec = MathUtil.clamp(
                            headingRateFilter.calculate(rawRateRadPerSec),
                            -maxHeadingRateRadPerSec,
                            maxHeadingRateRadPerSec);
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

    private static int currentHeadingRateFilterTaps() {
        return Math.max(1, (int) Math.round(headingRateFilterTaps.get()));
    }
}
