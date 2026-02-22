package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {
    @AutoLog
    class VisionIOInputs {
        public boolean isConnected = false;
        public TargetObservation latestTargetObservation =
                new TargetObservation(Rotation2d.kZero, Rotation2d.kZero);
        public PoseObservation[] poseObservations = new PoseObservation[0];
        public int[] tagIds = new int[0];
        public TargetTransform[] targetTransforms = new TargetTransform[0];

        public void clearFrameData() {
            latestTargetObservation = new TargetObservation(Rotation2d.kZero, Rotation2d.kZero);
            poseObservations = new PoseObservation[0];
            tagIds = new int[0];
            targetTransforms = new TargetTransform[0];
        }
    }

    record TargetObservation(Rotation2d yaw, Rotation2d pitch) {}

    record PoseObservation(
            double timestampSeconds, Pose3d pose, double ambiguity, int tagCount, double averageTagDistance) {}

    record TargetTransform(
            double timestampSeconds,
            int fiducialId,
            int cameraIndex,
            Transform3d cameraToTarget,
            double ambiguity,
            double distanceMeters) {}

    default void updateInputs(VisionIOInputs inputs) {}
}
