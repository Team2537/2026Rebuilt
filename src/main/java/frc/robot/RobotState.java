package frc.robot;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.subsystems.drive.Drive;

/**
 * Centralised read/write access to robot pose and velocity state.
 *
 * <p>Delegates to {@link Drive}'s pose estimator so every consumer reads from a
 * single source of truth without needing a direct reference to the drive
 * subsystem. Vision, commands, and autonomous routines should use this class
 * instead of calling Drive directly for pose-related queries.
 */
public final class RobotState {
    private static RobotState instance;

    private final Drive drive;

    private RobotState(Drive drive) {
        this.drive = drive;
    }

    /** Initialises the singleton. Replaces any previous instance. */
    public static void initialize(Drive drive) {
        instance = new RobotState(drive);
    }

    /** Returns the singleton instance. Throws if not yet initialised. */
    public static RobotState getInstance() {
        if (instance == null) {
            throw new IllegalStateException("RobotState has not been initialized");
        }
        return instance;
    }

    /** Returns the current odometry pose. */
    public Pose2d getPose() {
        return drive.getPose();
    }

    /** Returns the current robot rotation (live gyro on real hardware). */
    public Rotation2d getRotation() {
        return drive.getRotation();
    }

    /** Returns measured chassis speeds in robot-relative coordinates. */
    public ChassisSpeeds getMeasuredChassisSpeeds() {
        return drive.getMeasuredChassisSpeeds();
    }

    /** Returns the latest commanded chassis-speed setpoint in robot-relative coordinates. */
    public ChassisSpeeds getSetpointChassisSpeeds() {
        return drive.getSetpointChassisSpeeds();
    }

    /**
     * Returns the estimator pose sampled at a historical timestamp.
     * Falls back to current pose if the buffer does not contain that sample.
     */
    public Pose2d getPoseAtTimestamp(double timestampSeconds) {
        return drive.getPoseAtTimestamp(timestampSeconds);
    }

    /**
     * Returns the sim ground truth pose (pure odometry, no vision corrections).
     * Falls back to the fused estimator pose when not in simulation.
     */
    public Pose2d getSimGroundTruthPose() {
        return drive.getSimGroundTruthPose();
    }

    /** Resets the odometry pose. */
    public void setPose(Pose2d pose) {
        drive.setPose(pose);
    }

    /** Adds a timestamped vision measurement to the pose estimator. */
    public void addVisionMeasurement(
            Pose2d visionRobotPoseMeters,
            double timestampSeconds,
            Matrix<N3, N1> visionMeasurementStdDevs) {
        drive.addVisionMeasurement(visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
    }
}
