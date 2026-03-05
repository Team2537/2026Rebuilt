package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.util.FieldConstants;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

/** Real/sim shared PhotonVision implementation. */
public class VisionIOPhotonVision implements VisionIO {
    private static final PoseObservation[] EMPTY_POSE_OBSERVATIONS = new PoseObservation[0];
    private static final int[] EMPTY_TAG_IDS = new int[0];
    private static final TargetTransform[] EMPTY_TARGET_TRANSFORMS = new TargetTransform[0];

    protected final PhotonCamera camera;
    protected final Transform3d robotToCamera;
    private final int cameraIndex;
    private TargetObservation latestTargetObservation = new TargetObservation(Rotation2d.kZero, Rotation2d.kZero);

    public VisionIOPhotonVision(String name, Transform3d robotToCamera, int cameraIndex) {
        this.camera = new PhotonCamera(name);
        this.robotToCamera = robotToCamera;
        this.cameraIndex = cameraIndex;
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        inputs.isConnected = camera.isConnected();
        inputs.latestTargetObservation = latestTargetObservation;

        List<PhotonPipelineResult> unreadResults = camera.getAllUnreadResults();
        if (unreadResults.isEmpty()) {
            clearFrameOutputs(inputs);
            return;
        }

        PhotonPipelineResult result = unreadResults.get(unreadResults.size() - 1);
        if (result == null || !result.hasTargets()) {
            clearFrameOutputs(inputs);
            return;
        }
        double resultTimestampSeconds = result.getTimestampSeconds();

        Set<Integer> tagIds = new HashSet<>();
        List<PoseObservation> poseObservations = new ArrayList<>();
        List<TargetTransform> targetTransforms = new ArrayList<>();
        List<PhotonTrackedTarget> targets = result.getTargets();

        PhotonTrackedTarget bestTarget = result.getBestTarget();
        latestTargetObservation =
                new TargetObservation(
                        Rotation2d.fromDegrees(bestTarget.getYaw()),
                        Rotation2d.fromDegrees(bestTarget.getPitch()));
        inputs.latestTargetObservation = latestTargetObservation;

        for (PhotonTrackedTarget target : targets) {
            Transform3d cameraToTarget = target.getBestCameraToTarget();
            double distance = cameraToTarget.getTranslation().getNorm();
            targetTransforms.add(
                    new TargetTransform(
                            resultTimestampSeconds,
                            target.getFiducialId(),
                            cameraIndex,
                            cameraToTarget,
                            target.getPoseAmbiguity(),
                            distance));
        }

        result
                .getMultiTagResult()
                .ifPresentOrElse(
                        multitagResult -> {
                            Transform3d fieldToCamera = multitagResult.estimatedPose.best;
                            Transform3d fieldToRobot = fieldToCamera.plus(robotToCamera.inverse());
                            Pose3d robotPose = new Pose3d(fieldToRobot.getTranslation(), fieldToRobot.getRotation());

                            double totalTagDistance = 0.0;
                            for (PhotonTrackedTarget target : targets) {
                                totalTagDistance += target.getBestCameraToTarget().getTranslation().getNorm();
                            }

                            for (short id : multitagResult.fiducialIDsUsed) {
                                tagIds.add((int) id);
                            }

                            poseObservations.add(
                                    new PoseObservation(
                                            resultTimestampSeconds,
                                            robotPose,
                                            multitagResult.estimatedPose.ambiguity,
                                            multitagResult.fiducialIDsUsed.size(),
                                            targets.isEmpty()
                                                    ? 0.0
                                                    : totalTagDistance / targets.size()));
                        },
                        () -> {
                            addSingleTagObservations(
                                    targets,
                                    resultTimestampSeconds,
                                    robotToCamera,
                                    tagIds,
                                    poseObservations);
                        });
        inputs.poseObservations = poseObservations.toArray(new PoseObservation[0]);
        int[] tagIdArray = new int[tagIds.size()];
        int tagIdx = 0;
        for (int tagId : tagIds) {
            tagIdArray[tagIdx++] = tagId;
        }
        inputs.tagIds = tagIdArray;
        inputs.targetTransforms = targetTransforms.toArray(new TargetTransform[0]);
    }

    private static void clearFrameOutputs(VisionIOInputs inputs) {
        inputs.poseObservations = EMPTY_POSE_OBSERVATIONS;
        inputs.tagIds = EMPTY_TAG_IDS;
        inputs.targetTransforms = EMPTY_TARGET_TRANSFORMS;
    }

    static void addSingleTagObservations(
            List<PhotonTrackedTarget> targets,
            double resultTimestampSeconds,
            Transform3d robotToCamera,
            Set<Integer> tagIds,
            List<PoseObservation> poseObservations) {
        for (PhotonTrackedTarget target : targets) {
            FieldConstants.TAG_LAYOUT.getTagPose(target.getFiducialId()).ifPresent(tagPose -> {
                Transform3d fieldToTarget =
                        new Transform3d(tagPose.getTranslation(), tagPose.getRotation());
                Transform3d cameraToTarget = target.getBestCameraToTarget();
                Transform3d fieldToCamera = fieldToTarget.plus(cameraToTarget.inverse());
                Transform3d fieldToRobot = fieldToCamera.plus(robotToCamera.inverse());
                Pose3d robotPose =
                        new Pose3d(fieldToRobot.getTranslation(), fieldToRobot.getRotation());

                tagIds.add(target.getFiducialId());
                poseObservations.add(
                        new PoseObservation(
                                resultTimestampSeconds,
                                robotPose,
                                target.getPoseAmbiguity(),
                                1,
                                cameraToTarget.getTranslation().getNorm()));
            });
        }
    }
}
