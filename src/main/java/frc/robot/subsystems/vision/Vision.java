package frc.robot.subsystems.vision;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotType;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.util.FieldConstants;
import frc.robot.subsystems.vision.VisionIO.PoseObservation;
import frc.robot.subsystems.vision.VisionIO.TargetTransform;
import java.util.List;
import java.util.Set;
import java.util.stream.Collectors;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/** Vision subsystem responsible for pose updates from PhotonVision. */
public final class Vision extends SubsystemBase {
    private static final List<String> CAMERA_NAMES = List.of("Front Right", "Front Left");
    private static final List<Transform3d> ROBOT_TO_CAMERAS = List.of(
            new Transform3d(
                    new Translation3d(-0.325145, 0.301625, 0.254330),
                    new Rotation3d(0.0, Units.degreesToRadians(-27.5), Units.degreesToRadians(212.0))),
            new Transform3d(
                    new Translation3d(-0.325145, -0.301625, 0.254330),
                    new Rotation3d(0.0, Units.degreesToRadians(-30.0), Units.degreesToRadians(153.0))));

    private static final double MAX_AMBIGUITY = 0.3;
    private static final double MAX_Z_ERROR = 0.75;
    private static final double LINEAR_STD_DEV_BASELINE = 0.08;
    private static final double ANGULAR_STD_DEV_BASELINE = 0.18;
    private static final double HUB_TAG_CLUSTER_RADIUS_METERS = Units.inchesToMeters(30.0);
    private static final int HUB_TAG_ID = ShooterConstants.HUB_TAG_ID;
    private static final Set<Integer> HUB_TAG_IDS = determineHubTagIds();

    private final Drive drive;
    private final Supplier<Pose2d> robotPoseSupplier;
    private final List<VisionIO> ios;
    private final List<VisionIOInputsAutoLogged> inputs;
    private double hubYawRad = Double.NaN;

    public Vision(Drive drive) {
        super("vision");
        this.drive = drive;
        this.robotPoseSupplier = drive::getPose;
        this.ios = createIOs();
        this.inputs = ios.stream().map(io -> new VisionIOInputsAutoLogged()).toList();
    }

    public double getHubYawRad() {
        return hubYawRad;
    }

    @Override
    public void periodic() {
        Pose2d currentPose = robotPoseSupplier.get();

        for (int index = 0; index < ios.size(); index++) {
            VisionIO io = ios.get(index);
            VisionIOInputsAutoLogged input = inputs.get(index);
            io.updateInputs(input);
            Logger.processInputs(getName() + "/Camera" + index, input);

            PoseObservation bestObservation = selectBestObservation(input.poseObservations);
            if (bestObservation != null) {
                double stdDevFactor = Math.pow(bestObservation.averageTagDistance(), 2) / bestObservation.tagCount();
                double clampedFactor = Math.max(1.0, stdDevFactor);
                double linearStdDev = LINEAR_STD_DEV_BASELINE * clampedFactor;
                double angularStdDev = ANGULAR_STD_DEV_BASELINE * clampedFactor;

                drive.addVisionMeasurement(
                        bestObservation.pose().toPose2d(),
                        bestObservation.timestampSeconds(),
                        VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev));
            }
        }

        Logger.recordOutput(
                "vision/cameraPoses",
                ROBOT_TO_CAMERAS.stream()
                        .map(transform -> new Pose3d(currentPose).transformBy(transform))
                        .toArray(Pose3d[]::new));

        updateHubYawFromTags();
    }

    private List<VisionIO> createIOs() {
        return switch (RobotType.MODE) {
            case REAL ->
                    List.of(
                            new VisionIOPhotonVision(CAMERA_NAMES.get(0), ROBOT_TO_CAMERAS.get(0), 0),
                            new VisionIOPhotonVision(CAMERA_NAMES.get(1), ROBOT_TO_CAMERAS.get(1), 1));
            case SIMULATION ->
                    List.of(
                            new VisionIOPhotonVisionSim(
                                    CAMERA_NAMES.get(0), ROBOT_TO_CAMERAS.get(0), 0, robotPoseSupplier),
                            new VisionIOPhotonVisionSim(
                                    CAMERA_NAMES.get(1), ROBOT_TO_CAMERAS.get(1), 1, robotPoseSupplier));
            case REPLAY -> List.of(new NullVisionIO(), new NullVisionIO());
        };
    }

    private PoseObservation selectBestObservation(PoseObservation[] observations) {
        PoseObservation best = null;
        int bestTagCount = -1;
        double bestAmbiguity = Double.POSITIVE_INFINITY;
        double bestAvgDist = Double.POSITIVE_INFINITY;

        for (PoseObservation observation : observations) {
            if (!isPoseValid(observation)) {
                continue;
            }

            boolean better = false;
            if (observation.tagCount() > bestTagCount) {
                better = true;
            } else if (observation.tagCount() == bestTagCount) {
                if (observation.ambiguity() < bestAmbiguity) {
                    better = true;
                } else if (observation.ambiguity() == bestAmbiguity
                        && observation.averageTagDistance() < bestAvgDist) {
                    better = true;
                }
            }

            if (better) {
                best = observation;
                bestTagCount = observation.tagCount();
                bestAmbiguity = observation.ambiguity();
                bestAvgDist = observation.averageTagDistance();
            }
        }

        return best;
    }

    private boolean isPoseValid(PoseObservation observation) {
        if (observation.tagCount() == 0) {
            return false;
        }
        if (observation.tagCount() == 1 && observation.ambiguity() > MAX_AMBIGUITY) {
            return false;
        }
        if (Math.abs(observation.pose().getZ()) > MAX_Z_ERROR) {
            return false;
        }
        double x = observation.pose().getX();
        double y = observation.pose().getY();
        if (x < 0.0 || x > FieldConstants.FIELD_LENGTH_METERS) {
            return false;
        }
        if (y < 0.0 || y > FieldConstants.FIELD_WIDTH_METERS) {
            return false;
        }
        return true;
    }

    private void updateHubYawFromTags() {
        TargetTransform bestHubTarget = null;
        for (VisionIOInputsAutoLogged input : inputs) {
            for (TargetTransform target : input.targetTransforms) {
                if (!HUB_TAG_IDS.contains(target.fiducialId())) {
                    continue;
                }
                if (bestHubTarget == null || target.distanceMeters() < bestHubTarget.distanceMeters()) {
                    bestHubTarget = target;
                }
            }
        }

        if (bestHubTarget == null
                || bestHubTarget.cameraIndex() < 0
                || bestHubTarget.cameraIndex() >= ROBOT_TO_CAMERAS.size()) {
            hubYawRad = Double.NaN;
            Logger.recordOutput("vision/hubYawRobotRad", Double.NaN);
            Logger.recordOutput("vision/hubYawTagId", -1);
            Logger.recordOutput("vision/hubRelativePose", new Pose3d());
            return;
        }

        Transform3d robotToTarget = ROBOT_TO_CAMERAS.get(bestHubTarget.cameraIndex()).plus(bestHubTarget.cameraToTarget());
        hubYawRad = Math.atan2(robotToTarget.getY(), robotToTarget.getX());
        Logger.recordOutput("vision/hubYawRobotRad", hubYawRad);
        Logger.recordOutput("vision/hubYawTagId", bestHubTarget.fiducialId());
        Logger.recordOutput(
                "vision/hubRelativePose",
                new Pose3d(robotToTarget.getTranslation(), new Rotation3d()));
    }

    private static Set<Integer> determineHubTagIds() {
        return FieldConstants.TAG_LAYOUT.getTagPose(HUB_TAG_ID)
                .map(referenceTagPose -> FieldConstants.TAG_LAYOUT.getTags().stream()
                        .filter(tag -> tag.pose.getTranslation()
                                .getDistance(referenceTagPose.getTranslation()) <= HUB_TAG_CLUSTER_RADIUS_METERS)
                        .map(tag -> tag.ID)
                        .collect(Collectors.toUnmodifiableSet()))
                .orElse(Set.of(HUB_TAG_ID));
    }

    private static final class NullVisionIO implements VisionIO {
    }
}
