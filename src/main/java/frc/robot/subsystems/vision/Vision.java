package frc.robot.subsystems.vision;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotType;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.FieldConstants;
import frc.robot.subsystems.vision.VisionIO.PoseObservation;
import frc.robot.subsystems.vision.VisionIO.TargetTransform;
import java.util.Arrays;
import java.util.function.Supplier;
import java.util.List;
import java.util.Set;
import java.util.stream.Collectors;
import org.littletonrobotics.junction.Logger;

/** Vision subsystem responsible for pose updates from PhotonVision. */
public final class Vision extends SubsystemBase {
    private static final List<String> CAMERA_NAMES = List.of("Front Right", "Front Left");
    private static final List<Transform3d> ROBOT_TO_CAMERAS = List.of(
            new Transform3d(
                    new Translation3d(-0.325145, -0.299705, 0.254330),
                    new Rotation3d(0.0, Units.degreesToRadians(-45.0), Units.degreesToRadians(160.0))),
            new Transform3d(
                    new Translation3d(-0.325145, 0.299705, 0.254330),
                    new Rotation3d(0.0, Units.degreesToRadians(-20.0), Units.degreesToRadians(200.0))));

    private static final double MAX_AMBIGUITY = 0.3;
    private static final double MAX_Z_ERROR = 0.75;
    private static final double LINEAR_STD_DEV_BASELINE = 0.08;
    private static final double ANGULAR_STD_DEV_BASELINE = 0.18;
    private static final double HUB_TAG_CLUSTER_RADIUS_METERS = Units.inchesToMeters(30.0);
    private static final double HUB_YAW_MAX_AMBIGUITY = MAX_AMBIGUITY;
    private static final double HUB_YAW_MAX_DISTANCE_METERS = 6.0;
    private static final double HUB_YAW_MAX_AGE_SECONDS = 0.25;
    private static final int HUB_TAG_ID = FieldConstants.HUB_TAG_ID;
    private static final Set<Integer> HUB_TAG_IDS = determineHubTagIds();
    private static final double MAX_VISION_TRANSLATION_DELTA_METERS = 1.0;
    private static final double MAX_VISION_HEADING_DELTA_DEGREES = 35.0;
    private static final double VISION_JUMP_TRANSLATION_THRESHOLD_METERS = 0.5;
    private static final double VISION_JUMP_HEADING_THRESHOLD_DEGREES = 20.0;
    private static final double UNIFIED_POSE_MAX_AGE_SECONDS = 0.5;
    private static final boolean ENABLE_VISION_EVENT_LOGS = false;

    private final Drive drive;
    private final Supplier<Pose2d> robotPoseSupplier;
    private final List<VisionIO> ios;
    private final List<VisionIOInputsAutoLogged> inputs;
    private double hubYawRad = Double.NaN;
    private Pose2d hubTagRobotPose = null;
    private Pose2d unifiedRobotPose = null;
    private double unifiedRobotPoseTimestampSeconds = Double.NaN;

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

    /**
     * Returns robot pose estimated only from currently visible/reliable hub tags.
     * Returns null when unavailable or stale.
     */
    public Pose2d getHubTagRobotPose() {
        return hubTagRobotPose;
    }

    /**
     * Returns the most recent best vision-only robot pose from all cameras, if fresh.
     * Returns null when unavailable or stale.
     */
    public Pose2d getUnifiedRobotPose() {
        return unifiedRobotPose;
    }

    @Override
    public void periodic() {
        Pose2d currentPose = robotPoseSupplier.get();
        PoseObservation bestUnifiedObservation = null;

        for (int index = 0; index < ios.size(); index++) {
            VisionIO io = ios.get(index);
            VisionIOInputsAutoLogged input = inputs.get(index);
            input.clearFrameData();
            io.updateInputs(input);
            Logger.processInputs(getName() + "/Camera" + index, input);

            if (currentPose == null) {
                continue;
            }

            PoseObservation bestObservation = selectBestObservation(input.poseObservations);
            if (bestObservation != null) {
                if (isBetterObservation(bestObservation, bestUnifiedObservation)) {
                    bestUnifiedObservation = bestObservation;
                }

                double stdDevFactor = Math.pow(bestObservation.averageTagDistance(), 2) / bestObservation.tagCount();
                double clampedFactor = Math.max(1.0, stdDevFactor);
                double linearStdDev = LINEAR_STD_DEV_BASELINE * clampedFactor;
                double angularStdDev = ANGULAR_STD_DEV_BASELINE * clampedFactor;
                Pose2d measuredPose = bestObservation.pose().toPose2d();
                PoseDelta innovation = calculatePoseDelta(measuredPose, currentPose);
                if (!isVisionMeasurementConsistent(innovation)) {
                    logVisionRejection(
                            bestObservation.timestampSeconds(),
                            measuredPose,
                            currentPose,
                            innovation,
                            bestObservation.tagCount(),
                            input.tagIds,
                            bestObservation.ambiguity(),
                            bestObservation.averageTagDistance(),
                            linearStdDev,
                            angularStdDev,
                            index);
                    continue;
                }

                drive.addVisionMeasurement(
                        measuredPose,
                        bestObservation.timestampSeconds(),
                        VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev));

                logVisionJumpIfLarge(
                        bestObservation.timestampSeconds(),
                        measuredPose,
                        currentPose,
                        innovation,
                        bestObservation.tagCount(),
                        input.tagIds,
                        bestObservation.ambiguity(),
                        bestObservation.averageTagDistance(),
                        linearStdDev,
                        angularStdDev,
                        index);
            }
        }

        updateUnifiedPose(bestUnifiedObservation);
        Logger.recordOutput(
                "vision/unifiedRobotPose",
                unifiedRobotPose != null ? unifiedRobotPose : new Pose2d());
        Logger.recordOutput("vision/unifiedRobotPoseValid", unifiedRobotPose != null);

        Logger.recordOutput(
                "vision/cameraPoses",
                currentPose == null
                        ? new Pose3d[0]
                        : ROBOT_TO_CAMERAS.stream()
                                .map(transform -> new Pose3d(currentPose).transformBy(transform))
                                .toArray(Pose3d[]::new));

        updateHubTagTracking();
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

        for (PoseObservation observation : observations) {
            if (!isPoseValid(observation)) {
                continue;
            }

            if (isBetterObservation(observation, best)) {
                best = observation;
            }
        }

        return best;
    }

    private static boolean isBetterObservation(PoseObservation candidate, PoseObservation currentBest) {
        if (candidate == null) {
            return false;
        }
        if (currentBest == null) {
            return true;
        }
        if (candidate.tagCount() != currentBest.tagCount()) {
            return candidate.tagCount() > currentBest.tagCount();
        }
        if (candidate.ambiguity() != currentBest.ambiguity()) {
            return candidate.ambiguity() < currentBest.ambiguity();
        }
        return candidate.averageTagDistance() < currentBest.averageTagDistance();
    }

    private void updateUnifiedPose(PoseObservation bestUnifiedObservation) {
        if (bestUnifiedObservation != null) {
            unifiedRobotPose = bestUnifiedObservation.pose().toPose2d();
            unifiedRobotPoseTimestampSeconds = bestUnifiedObservation.timestampSeconds();
            return;
        }

        if (unifiedRobotPose == null) {
            return;
        }

        double ageSeconds = Timer.getFPGATimestamp() - unifiedRobotPoseTimestampSeconds;
        if (!Double.isFinite(ageSeconds) || ageSeconds > UNIFIED_POSE_MAX_AGE_SECONDS) {
            unifiedRobotPose = null;
            unifiedRobotPoseTimestampSeconds = Double.NaN;
        }
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

    private void updateHubTagTracking() {
        double nowSeconds = Timer.getFPGATimestamp();
        TargetTransform bestHubTarget = findBestHubTarget(nowSeconds);

        if (bestHubTarget == null
                || bestHubTarget.cameraIndex() < 0
                || bestHubTarget.cameraIndex() >= ROBOT_TO_CAMERAS.size()) {
            hubYawRad = Double.NaN;
            hubTagRobotPose = null;
            Logger.recordOutput("vision/hubYawRobotRad", Double.NaN);
            Logger.recordOutput("vision/hubYawTagId", -1);
            Logger.recordOutput("vision/hubRelativePose", new Pose3d());
            Logger.recordOutput("vision/hubTagRobotPose", new Pose2d());
            return;
        }

        Transform3d robotToTarget = ROBOT_TO_CAMERAS.get(bestHubTarget.cameraIndex()).plus(bestHubTarget.cameraToTarget());
        hubYawRad = Math.atan2(robotToTarget.getY(), robotToTarget.getX());
        hubTagRobotPose = estimateRobotPoseFromHubTarget(bestHubTarget, robotToTarget);
        Logger.recordOutput("vision/hubYawRobotRad", hubYawRad);
        Logger.recordOutput("vision/hubYawTagId", bestHubTarget.fiducialId());
        Logger.recordOutput(
                "vision/hubRelativePose",
                new Pose3d(robotToTarget.getTranslation(), new Rotation3d()));
        Logger.recordOutput(
                "vision/hubTagRobotPose",
                hubTagRobotPose != null ? hubTagRobotPose : new Pose2d());
    }

    private Pose2d estimateRobotPoseFromHubTarget(TargetTransform target, Transform3d robotToTarget) {
        return FieldConstants.TAG_LAYOUT.getTagPose(target.fiducialId())
                .map(fieldToTarget -> fieldToTarget.transformBy(robotToTarget.inverse()).toPose2d())
                .orElse(null);
    }

    private TargetTransform findBestHubTarget(double nowSeconds) {
        TargetTransform bestHubTarget = null;
        for (VisionIOInputsAutoLogged input : inputs) {
            for (TargetTransform target : input.targetTransforms) {
                if (!HUB_TAG_IDS.contains(target.fiducialId())) {
                    continue;
                }
                if (!isHubTargetReliable(target, nowSeconds)) {
                    continue;
                }
                if (bestHubTarget == null || target.distanceMeters() < bestHubTarget.distanceMeters()) {
                    bestHubTarget = target;
                }
            }
        }
        return bestHubTarget;
    }

    private boolean isHubTargetReliable(TargetTransform target, double nowSeconds) {
        if (!Double.isFinite(target.distanceMeters()) || target.distanceMeters() > HUB_YAW_MAX_DISTANCE_METERS) {
            return false;
        }
        if (!Double.isFinite(target.ambiguity()) || target.ambiguity() > HUB_YAW_MAX_AMBIGUITY) {
            return false;
        }

        double ageSeconds = nowSeconds - target.timestampSeconds();
        return Double.isFinite(ageSeconds) && ageSeconds >= 0.0 && ageSeconds <= HUB_YAW_MAX_AGE_SECONDS;
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

    private boolean isVisionMeasurementConsistent(PoseDelta innovation) {
        return innovation.translationMeters() <= MAX_VISION_TRANSLATION_DELTA_METERS
                && innovation.headingDegrees() <= MAX_VISION_HEADING_DELTA_DEGREES;
    }

    private static PoseDelta calculatePoseDelta(Pose2d measuredPose, Pose2d odometryPose) {
        Translation2d deltaTranslation = measuredPose.getTranslation().minus(odometryPose.getTranslation());
        double translationDelta = deltaTranslation.getNorm();
        double headingDeltaDeg = calculateHeadingDeltaDegrees(measuredPose.getRotation(), odometryPose.getRotation());
        return new PoseDelta(translationDelta, headingDeltaDeg);
    }

    private static double calculateHeadingDeltaDegrees(Rotation2d first, Rotation2d second) {
        return Units.radiansToDegrees(
                Math.abs(Math.IEEEremainder(first.minus(second).getRadians(), 2.0 * Math.PI)));
    }

    private void logVisionRejection(
            double timestampSeconds,
            Pose2d measuredPose,
            Pose2d odometryPose,
            PoseDelta innovation,
            int tagCount,
            int[] tagIds,
            double ambiguity,
            double avgDistance,
            double linearStdDev,
            double angularStdDev,
            int cameraIndex) {
        logVisionEvent(
                "reject",
                timestampSeconds,
                measuredPose,
                odometryPose,
                innovation,
                tagCount,
                tagIds,
                ambiguity,
                avgDistance,
                linearStdDev,
                angularStdDev,
                cameraIndex);
    }

    private void logVisionJumpIfLarge(
            double timestampSeconds,
            Pose2d measuredPose,
            Pose2d odometryPose,
            PoseDelta innovation,
            int tagCount,
            int[] tagIds,
            double ambiguity,
            double avgDistance,
            double linearStdDev,
            double angularStdDev,
            int cameraIndex) {
        if (innovation.translationMeters() <= VISION_JUMP_TRANSLATION_THRESHOLD_METERS
                && innovation.headingDegrees() <= VISION_JUMP_HEADING_THRESHOLD_DEGREES) {
            return;
        }

        logVisionEvent(
                "jump",
                timestampSeconds,
                measuredPose,
                odometryPose,
                innovation,
                tagCount,
                tagIds,
                ambiguity,
                avgDistance,
                linearStdDev,
                angularStdDev,
                cameraIndex);
    }

    private void logVisionEvent(
            String eventType,
            double timestampSeconds,
            Pose2d measuredPose,
            Pose2d odometryPose,
            PoseDelta innovation,
            int tagCount,
            int[] tagIds,
            double ambiguity,
            double avgDistance,
            double linearStdDev,
            double angularStdDev,
            int cameraIndex) {
        if (!ENABLE_VISION_EVENT_LOGS) {
            return;
        }
        String tagList = tagIds == null ? "[]" : Arrays.toString(tagIds);
        System.out.printf(
                "Vision %s @%.3f s cam=%d pose=%s odom=%s dPos=%.3fm dYawDeg=%.1f tags=%d %s amb=%.3f avgDist=%.3fm stdDev=[%.3f,%.3f]%n",
                eventType,
                timestampSeconds,
                cameraIndex,
                measuredPose,
                odometryPose,
                innovation.translationMeters(),
                innovation.headingDegrees(),
                tagCount,
                tagList,
                ambiguity,
                avgDistance,
                linearStdDev,
                angularStdDev);
    }

    private record PoseDelta(double translationMeters, double headingDegrees) {
    }
}
