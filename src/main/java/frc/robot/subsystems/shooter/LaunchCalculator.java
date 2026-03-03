package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.util.FieldConstants;

/**
 * Computes motion-compensated launch parameters for shooting while moving.
 *
 * <p>Given the robot's current pose and velocity, this class predicts where the robot
 * will be when the ball reaches the target and computes the compensated distance and
 * heading. Uses an iterative convergence loop because flight time depends on distance,
 * and distance depends on the predicted future position (which depends on flight time).
 */
public final class LaunchCalculator {

    /** Result of a motion compensation calculation. */
    public record MotionCompensation(
            double rawDistanceMeters,
            double compensatedDistanceMeters,
            double timeInAirSec,
            double velocityTowardHubMps,
            double velocityPerpendicularHubMps,
            Rotation2d compensatedHeading) {}

    /**
     * Maximum number of iterations for the time/distance convergence loop.
     * 3 iterations is sufficient -- typically converges in 2.
     */
    private static final int MAX_ITERATIONS = 3;
    private static final double TIME_CONVERGENCE_THRESHOLD_SEC = 0.001;

    private final InterpolatingDoubleTreeMap timeInAirSecondsByDistance;
    private final Translation2d shooterOffset;
    private final double phaseDelaySec;
    private final double motionCompTimeScale;
    private final double motionCompDistanceTimeScale;

    public LaunchCalculator(
            InterpolatingDoubleTreeMap timeInAirSecondsByDistance,
            Translation2d shooterOffset,
            double phaseDelaySec,
            double motionCompTimeScale,
            double motionCompDistanceTimeScale) {
        this.timeInAirSecondsByDistance = timeInAirSecondsByDistance;
        this.shooterOffset = shooterOffset;
        this.phaseDelaySec = phaseDelaySec;
        this.motionCompTimeScale = motionCompTimeScale;
        this.motionCompDistanceTimeScale = motionCompDistanceTimeScale;
    }

    /**
     * Computes motion-compensated distance and heading to the hub target.
     *
     * @param robotPose              current robot pose on the field
     * @param robotRelativeSpeeds    robot-relative chassis speeds
     * @return compensation result with raw/compensated distance, heading, and diagnostics
     */
    public MotionCompensation calculate(Pose2d robotPose, ChassisSpeeds robotRelativeSpeeds) {
        Translation2d hubTarget = FieldConstants.getHubTargetTranslation();
        if (hubTarget == null || robotPose == null || robotRelativeSpeeds == null || shooterOffset == null) {
            return invalid();
        }

        Translation2d shooterFieldPosition = getShooterFieldPosition(robotPose);
        double rawDistanceMeters = shooterFieldPosition != null
                ? shooterFieldPosition.getDistance(hubTarget)
                : Double.NaN;
        Translation2d toHubVector = shooterFieldPosition != null ? hubTarget.minus(shooterFieldPosition) : null;
        if (toHubVector == null) {
            return invalid();
        }
        double rangeMeters = toHubVector.getNorm();
        if (!Double.isFinite(rawDistanceMeters) || !Double.isFinite(rangeMeters) || rangeMeters <= 1e-6) {
            return new MotionCompensation(rawDistanceMeters, rawDistanceMeters, 0.0, 0.0, 0.0, null);
        }

        // Transform robot velocity to shooter velocity to account for angular velocity.
        // v_shooter = v_center + omega x r (2D cross: omega_z x (x,y) = (-omega*y, omega*x))
        double shooterVxRobot = robotRelativeSpeeds.vxMetersPerSecond
                - robotRelativeSpeeds.omegaRadiansPerSecond * shooterOffset.getY();
        double shooterVyRobot = robotRelativeSpeeds.vyMetersPerSecond
                + robotRelativeSpeeds.omegaRadiansPerSecond * shooterOffset.getX();
        ChassisSpeeds shooterFieldSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(
                new ChassisSpeeds(shooterVxRobot, shooterVyRobot, robotRelativeSpeeds.omegaRadiansPerSecond),
                robotPose.getRotation());
        Translation2d shooterFieldVelocity = new Translation2d(
                shooterFieldSpeeds.vxMetersPerSecond, shooterFieldSpeeds.vyMetersPerSecond);

        // Velocity decomposition for diagnostics
        Translation2d towardUnit = new Translation2d(toHubVector.getX() / rangeMeters, toHubVector.getY() / rangeMeters);
        Translation2d perpendicularUnit = new Translation2d(-towardUnit.getY(), towardUnit.getX());
        double velocityTowardHub =
                shooterFieldVelocity.getX() * towardUnit.getX() + shooterFieldVelocity.getY() * towardUnit.getY();
        double velocityPerpendicularHub =
                shooterFieldVelocity.getX() * perpendicularUnit.getX()
                        + shooterFieldVelocity.getY() * perpendicularUnit.getY();

        // Apply phase delay to compensate for system latency.
        Translation2d phaseDelayedTranslation = shooterFieldPosition.plus(
                shooterFieldVelocity.times(phaseDelaySec));

        // Iterative convergence: time-in-air depends on distance, distance depends on
        // predicted position (which depends on time-in-air).
        double timeInAirSec = lookupTimeInAir(rawDistanceMeters);
        double effectiveTimeSec = timeInAirSec * motionCompTimeScale;
        Translation2d predictedRobotTranslation;
        Translation2d compensatedVector = toHubVector;
        double compensatedDistanceMeters = rawDistanceMeters;

        for (int i = 0; i < MAX_ITERATIONS; i++) {
            predictedRobotTranslation = phaseDelayedTranslation.plus(shooterFieldVelocity.times(effectiveTimeSec));
            compensatedVector = hubTarget.minus(predictedRobotTranslation);
            compensatedDistanceMeters = compensatedVector.getNorm();
            if (!Double.isFinite(compensatedDistanceMeters) || compensatedDistanceMeters < 1e-6) {
                compensatedDistanceMeters = rawDistanceMeters;
                compensatedVector = toHubVector;
                break;
            }
            double newTimeInAirSec = lookupTimeInAir(compensatedDistanceMeters);
            if (!Double.isFinite(newTimeInAirSec) || newTimeInAirSec < 0.0) {
                break;
            }
            double newEffectiveTimeSec = newTimeInAirSec * motionCompTimeScale;
            if (Math.abs(newEffectiveTimeSec - effectiveTimeSec) < TIME_CONVERGENCE_THRESHOLD_SEC) {
                timeInAirSec = newTimeInAirSec;
                effectiveTimeSec = newEffectiveTimeSec;
                break;
            }
            timeInAirSec = newTimeInAirSec;
            effectiveTimeSec = newEffectiveTimeSec;
        }

        // Final prediction with converged time
        predictedRobotTranslation = phaseDelayedTranslation.plus(shooterFieldVelocity.times(effectiveTimeSec));
        compensatedVector = hubTarget.minus(predictedRobotTranslation);
        compensatedDistanceMeters = compensatedVector.getNorm();
        if (!Double.isFinite(compensatedDistanceMeters) || compensatedDistanceMeters < 1e-6) {
            compensatedDistanceMeters = rawDistanceMeters;
            compensatedVector = toHubVector;
        }

        Rotation2d rawHeading = toHubVector.getAngle();
        Rotation2d compensatedHeading =
                compensatedVector.getNorm() > 1e-6 ? compensatedVector.getAngle() : rawHeading;

        return new MotionCompensation(
                rawDistanceMeters,
                compensatedDistanceMeters,
                timeInAirSec,
                velocityTowardHub,
                velocityPerpendicularHub,
                compensatedHeading);
    }

    private Translation2d getShooterFieldPosition(Pose2d robotPose) {
        if (robotPose == null || shooterOffset == null) {
            return null;
        }
        return robotPose.getTranslation().plus(shooterOffset.rotateBy(robotPose.getRotation()));
    }

    private double lookupTimeInAir(double distanceMeters) {
        double scaledDistanceMeters = distanceMeters * motionCompDistanceTimeScale;
        double time = timeInAirSecondsByDistance.get(scaledDistanceMeters);
        return Double.isFinite(time) && time >= 0.0 ? time : 0.0;
    }

    /** Returns the raw distance from the shooter position to the hub target. */
    public static double getHubDistanceMeters(Pose2d robotPose) {
        Translation2d hubTarget = FieldConstants.getHubTargetTranslation();
        if (hubTarget == null || robotPose == null) {
            return Double.NaN;
        }
        Translation2d shooterFieldPosition = robotPose
                .getTranslation()
                .plus(ShooterConstants.ROBOT_TO_SHOOTER_OFFSET.rotateBy(robotPose.getRotation()));
        return shooterFieldPosition.getDistance(hubTarget);
    }

    private static MotionCompensation invalid() {
        return new MotionCompensation(Double.NaN, Double.NaN, Double.NaN, Double.NaN, Double.NaN, null);
    }
}
