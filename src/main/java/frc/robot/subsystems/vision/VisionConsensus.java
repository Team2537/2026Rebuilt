package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;

/**
 * Aggregated field pose estimate built from the latest valid camera observations.
 *
 * <p>Confidence is normalized to {@code [0, 1]}. Standard deviations describe the
 * expected quality of the consensus measurement before any estimator-specific
 * trust adjustments are applied.
 */
public record VisionConsensus(
        double timestampSeconds,
        Pose2d pose,
        double confidence,
        double translationStdDevMeters,
        double rotationStdDevRadians,
        int contributingCameraCount,
        int tagCount,
        boolean multiTag) {
    /** Returns whether this consensus object contains a complete, usable estimate. */
    public boolean isUsable() {
        return pose != null
                && Double.isFinite(timestampSeconds)
                && Double.isFinite(confidence)
                && confidence > 0.0
                && Double.isFinite(translationStdDevMeters)
                && translationStdDevMeters > 0.0
                && Double.isFinite(rotationStdDevRadians)
                && rotationStdDevRadians > 0.0
                && contributingCameraCount > 0
                && tagCount > 0;
    }
}
