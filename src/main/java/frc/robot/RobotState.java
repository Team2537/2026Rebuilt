package frc.robot;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.VisionConsensus;
import org.littletonrobotics.junction.Logger;

/**
 * Centralized read/write access to robot pose and velocity state.
 *
 * <p>Drive owns dead reckoning. Vision owns camera observations and consensus.
 * RobotState owns the fused field pose that commands and autonomous routines use.
 */
public final class RobotState {
    /** High-level pose-estimation state exposed for dashboards and debugging. */
    public enum PoseMode {
        NOMINAL,
        SLIP_SUSPECTED,
        RECOVERING,
        VISION_UNAVAILABLE
    }

    private static final double ESTIMATOR_ANGULAR_STD_DEV_RAD = 999.0;
    private static final double MIN_VISION_CONFIDENCE = 0.15;
    private static final double RECOVERY_MIN_CONFIDENCE = 0.60;
    private static final double SLIP_SUSPECTED_THRESHOLD = 0.45;
    private static final double RECOVERY_TRANSLATION_DISAGREEMENT_METERS = 0.75;
    private static final double RECOVERY_DIRECTION_DOT_THRESHOLD = 0.60;
    private static final int RECOVERY_CONSISTENT_FRAMES_REQUIRED = 3;
    private static final double RECOVERY_HOLD_SECONDS = 0.50;
    private static final double MAX_VISION_AGE_SECONDS = 0.75;
    private static final double MIN_TRANSLATION_STD_DEV_METERS = 0.02;
    private static final double SLIP_TRUST_SCALE = 0.60;
    private static final double RECOVERY_TRUST_SCALE = 0.30;

    private static RobotState instance;

    private final Drive drive;
    private final SwerveDrivePoseEstimator fusedPoseEstimator;

    private Pose2d latestDeadReckonedPose = Pose2d.kZero;
    private VisionConsensus latestVisionConsensus = null;
    private double latestSlipScore = 0.0;
    private PoseMode poseMode = PoseMode.VISION_UNAVAILABLE;
    private Translation2d lastRecoveryDirection = null;
    private int consistentRecoveryFrameCount = 0;
    private double recoveryUntilTimestampSeconds = Double.NEGATIVE_INFINITY;

    private RobotState(Drive drive) {
        this.drive = drive;
        this.latestDeadReckonedPose = drive.getDeadReckonedPose();
        this.fusedPoseEstimator = new SwerveDrivePoseEstimator(
                drive.getKinematics(),
                drive.getRotation(),
                drive.getModulePositionsSnapshot(),
                latestDeadReckonedPose,
                VecBuilder.fill(0.1, 0.1, 0.1),
                VecBuilder.fill(1.3, 1.3, 1.3));
        logState();
    }

    /** Initializes the singleton. Replaces any previous instance. */
    public static void initialize(Drive drive) {
        instance = new RobotState(drive);
        drive.attachRobotState(instance);
    }

    /** Returns the singleton instance. Throws if not yet initialized. */
    public static RobotState getInstance() {
        if (instance == null) {
            throw new IllegalStateException("RobotState has not been initialized");
        }
        return instance;
    }

    /** Returns the current fused pose used by the rest of the robot. */
    public Pose2d getPose() {
        return fusedPoseEstimator.getEstimatedPosition();
    }

    /** Returns the latest dead-reckoned pose published by the drive subsystem. */
    public Pose2d getDeadReckonedPose() {
        return latestDeadReckonedPose;
    }

    /** Returns the most recent usable vision consensus, or null when unavailable/stale. */
    public VisionConsensus getLatestVisionConsensus() {
        return latestVisionConsensus != null
                && latestVisionConsensus.isUsable()
                && isFresh(latestVisionConsensus.timestampSeconds(), MAX_VISION_AGE_SECONDS)
                ? latestVisionConsensus
                : null;
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

    /** Returns the latest slip score from the drive subsystem. */
    public double getSlipScore() {
        return latestSlipScore;
    }

    /** Returns the current pose fusion mode. */
    public PoseMode getPoseMode() {
        return poseMode;
    }

    /**
     * Returns the fused pose sampled at a historical timestamp.
     * Falls back to current pose if the buffer does not contain that sample.
     */
    public Pose2d getPoseAtTimestamp(double timestampSeconds) {
        if (!Double.isFinite(timestampSeconds)) {
            return getPose();
        }
        return fusedPoseEstimator.sampleAt(timestampSeconds).orElseGet(this::getPose);
    }

    /** Returns the sim ground-truth pose used by vision simulation. */
    public Pose2d getSimGroundTruthPose() {
        return drive.getSimGroundTruthPose();
    }

    /** Resets both the fused estimator and drive dead reckoning to the supplied pose. */
    public void setPose(Pose2d pose) {
        drive.resetDeadReckoningPose(pose);
        latestDeadReckonedPose = pose;
        fusedPoseEstimator.resetPosition(drive.getRotation(), drive.getModulePositionsSnapshot(), pose);
        clearRecoveryTracking();
        updatePoseMode();
        logState();
    }

    /** Updates the fused estimator from a dead-reckoning sample published by Drive. */
    public void addDriveSample(
            double timestampSeconds,
            Rotation2d gyroYaw,
            SwerveModulePosition[] modulePositions,
            Pose2d deadReckonedPose,
            double slipScore) {
        latestDeadReckonedPose = deadReckonedPose;
        latestSlipScore = clamp01(slipScore);
        fusedPoseEstimator.updateWithTime(timestampSeconds, gyroYaw, modulePositions);
        updatePoseMode();
        logState();
    }

    /**
     * Applies the latest vision consensus to the fused estimator when it is usable.
     *
     * <p>Heading remains gyro-owned; only translation is corrected by vision.
     * Recovery mode temporarily increases vision authority after repeated,
     * high-confidence disagreement in the same direction.
     */
    public void addVisionConsensus(VisionConsensus consensus) {
        latestVisionConsensus = consensus != null && consensus.isUsable() ? consensus : null;
        if (latestVisionConsensus == null || latestVisionConsensus.confidence() < MIN_VISION_CONFIDENCE) {
            clearRecoveryTracking();
            updatePoseMode();
            logState();
            return;
        }

        Pose2d referencePose = getPoseAtTimestamp(latestVisionConsensus.timestampSeconds());
        Translation2d disagreement = latestVisionConsensus.pose()
                .getTranslation()
                .minus(referencePose.getTranslation());
        double disagreementMeters = disagreement.getNorm();
        Translation2d disagreementDirection = disagreementMeters > 1e-6
                ? disagreement.div(disagreementMeters)
                : null;

        boolean sameDirection = disagreementDirection != null
                && (lastRecoveryDirection == null
                        || disagreementDirection.getX() * lastRecoveryDirection.getX()
                                + disagreementDirection.getY() * lastRecoveryDirection.getY()
                                >= RECOVERY_DIRECTION_DOT_THRESHOLD);
        boolean recoveryCandidate = latestVisionConsensus.confidence() >= RECOVERY_MIN_CONFIDENCE
                && disagreementMeters >= RECOVERY_TRANSLATION_DISAGREEMENT_METERS
                && sameDirection;
        boolean continuingRecoverySequence = consistentRecoveryFrameCount > 0
                && latestVisionConsensus.confidence() >= RECOVERY_MIN_CONFIDENCE
                && disagreementMeters >= RECOVERY_TRANSLATION_DISAGREEMENT_METERS * 0.33
                && sameDirection;

        updateRecoveryTracking(disagreementDirection, recoveryCandidate || continuingRecoverySequence);

        boolean shouldHoldForRecovery = disagreementMeters >= RECOVERY_TRANSLATION_DISAGREEMENT_METERS
                && latestSlipScore < SLIP_SUSPECTED_THRESHOLD
                && !isRecoveryActive()
                && consistentRecoveryFrameCount < RECOVERY_CONSISTENT_FRAMES_REQUIRED;
        if (shouldHoldForRecovery) {
            updatePoseMode();
            logState();
            return;
        }

        double translationStdDev = adjustedVisionTranslationStdDev(latestVisionConsensus.translationStdDevMeters());

        Pose2d estimatorMeasurementPose = new Pose2d(
                latestVisionConsensus.pose().getTranslation(),
                referencePose.getRotation());
        fusedPoseEstimator.addVisionMeasurement(
                estimatorMeasurementPose,
                latestVisionConsensus.timestampSeconds(),
                VecBuilder.fill(
                        translationStdDev,
                        translationStdDev,
                        ESTIMATOR_ANGULAR_STD_DEV_RAD));

        updatePoseMode();
        logState();
    }

    private void updatePoseMode() {
        if (isRecoveryActive()) {
            poseMode = PoseMode.RECOVERING;
            return;
        }
        if (latestSlipScore >= SLIP_SUSPECTED_THRESHOLD) {
            poseMode = PoseMode.SLIP_SUSPECTED;
            return;
        }
        poseMode = getLatestVisionConsensus() != null ? PoseMode.NOMINAL : PoseMode.VISION_UNAVAILABLE;
    }

    private void logState() {
        VisionConsensus usableVision = getLatestVisionConsensus();
        Pose2d fusedPose = getPose();
        double visionAgeSeconds = usableVision != null
                ? Timer.getFPGATimestamp() - usableVision.timestampSeconds()
                : Double.NaN;
        Logger.recordOutput("RobotState/FusedPose", fusedPose);
        Logger.recordOutput("RobotState/DeadReckonedPose", latestDeadReckonedPose);
        Logger.recordOutput("RobotState/VisionConsensusPose", usableVision != null ? usableVision.pose() : new Pose2d());
        Logger.recordOutput("RobotState/VisionConsensusValid", usableVision != null);
        Logger.recordOutput("RobotState/VisionConfidence", usableVision != null ? usableVision.confidence() : Double.NaN);
        Logger.recordOutput("RobotState/VisionConsensusAgeSeconds", visionAgeSeconds);
        Logger.recordOutput(
                "RobotState/VisionConsensusTranslationStdDevMeters",
                usableVision != null ? usableVision.translationStdDevMeters() : Double.NaN);
        Logger.recordOutput(
                "RobotState/VisionConsensusRotationStdDevRadians",
                usableVision != null ? usableVision.rotationStdDevRadians() : Double.NaN);
        Logger.recordOutput(
                "RobotState/VisionConsensusContributingCameraCount",
                usableVision != null ? usableVision.contributingCameraCount() : 0);
        Logger.recordOutput(
                "RobotState/VisionConsensusTagCount",
                usableVision != null ? usableVision.tagCount() : 0);
        Logger.recordOutput("RobotState/SlipScore", latestSlipScore);
        Logger.recordOutput("RobotState/PoseMode", poseMode.name());
        Logger.recordOutput("RobotState/RecoveryActive", poseMode == PoseMode.RECOVERING);
        Logger.recordOutput("RobotState/RecoveryConsistentFrameCount", consistentRecoveryFrameCount);
        Logger.recordOutput("RobotState/RecoveryUntilTimestampSeconds", recoveryUntilTimestampSeconds);
        Logger.recordOutput(
                "RobotState/VisionDisagreementMeters",
                usableVision != null
                        ? usableVision.pose().getTranslation().getDistance(fusedPose.getTranslation())
                        : Double.NaN);
    }

    private void updateRecoveryTracking(Translation2d disagreementDirection, boolean recoveryCandidate) {
        if (recoveryCandidate) {
            consistentRecoveryFrameCount++;
            lastRecoveryDirection = disagreementDirection;
            if (consistentRecoveryFrameCount >= RECOVERY_CONSISTENT_FRAMES_REQUIRED) {
                recoveryUntilTimestampSeconds = Math.max(
                        recoveryUntilTimestampSeconds,
                        Math.max(Timer.getFPGATimestamp(), latestVisionConsensus.timestampSeconds()) + RECOVERY_HOLD_SECONDS);
            }
            return;
        }

        consistentRecoveryFrameCount = 0;
        lastRecoveryDirection = disagreementDirection;
    }

    private void clearRecoveryTracking() {
        consistentRecoveryFrameCount = 0;
        lastRecoveryDirection = null;
        recoveryUntilTimestampSeconds = Double.NEGATIVE_INFINITY;
    }

    private double adjustedVisionTranslationStdDev(double baseStdDev) {
        double translationStdDev = baseStdDev;
        if (latestSlipScore >= SLIP_SUSPECTED_THRESHOLD) {
            translationStdDev *= SLIP_TRUST_SCALE;
        }
        if (isRecoveryActive()) {
            translationStdDev *= RECOVERY_TRUST_SCALE;
        }
        return Math.max(MIN_TRANSLATION_STD_DEV_METERS, translationStdDev);
    }

    private static double clamp01(double value) {
        if (!Double.isFinite(value)) {
            return 0.0;
        }
        return Math.max(0.0, Math.min(1.0, value));
    }

    private static boolean isFresh(double timestampSeconds, double maxAgeSeconds) {
        if (!Double.isFinite(timestampSeconds)) {
            return false;
        }
        double ageSeconds = Timer.getFPGATimestamp() - timestampSeconds;
        return Double.isFinite(ageSeconds) && ageSeconds <= maxAgeSeconds;
    }

    private boolean isRecoveryActive() {
        return Double.isFinite(recoveryUntilTimestampSeconds)
                && recoveryUntilTimestampSeconds >= Timer.getFPGATimestamp();
    }
}
