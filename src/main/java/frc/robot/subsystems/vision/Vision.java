package frc.robot.subsystems.vision;

import edu.wpi.first.math.MathUtil;
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
import frc.robot.subsystems.vision.VisionIO.PoseObservation;
import frc.robot.subsystems.vision.VisionIO.TargetTransform;
import frc.robot.util.FieldConstants;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Set;
import java.util.function.Supplier;
import java.util.stream.Collectors;
import org.littletonrobotics.junction.Logger;

/** Vision subsystem responsible for pose updates from PhotonVision. */
public final class Vision extends SubsystemBase {
    private static final List<String> CAMERA_NAMES = List.of("Front Right", "Front Left");
    private static final List<Transform3d> ROBOT_TO_CAMERAS = List.of(
            new Transform3d(
                    new Translation3d(-0.325145, -0.299705, 0.13633),
                    new Rotation3d(Units.degreesToRadians(0.5), Units.degreesToRadians(-45.0), Units.degreesToRadians(160.0))),
            new Transform3d(
                    new Translation3d(-0.325145, 0.299705, 0.13633),
                    new Rotation3d(Units.degreesToRadians(2.0), Units.degreesToRadians(-23.0), Units.degreesToRadians(197.0))));

    private static final double MAX_AMBIGUITY = 0.3;
    private static final double MAX_Z_ERROR = 0.75;
    private static final double LINEAR_STD_DEV_BASELINE = 0.08;
    private static final double ANGULAR_STD_DEV_BASELINE = 0.18;
    private static final double ESTIMATOR_ANGULAR_STD_DEV_RAD = 999.0;
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
    private static final double UNIFIED_RAW_POSE_MAX_AGE_SECONDS = 0.25;
    private static final double UNIFIED_POSE_MAX_AGE_SECONDS = 0.5;
    private static final double UNIFIED_POSE_SMOOTH_ALPHA = 0.35;
    private static final double UNIFIED_POSE_MAX_STEP_METERS = 0.35;
    private static final double UNIFIED_POSE_MAX_STEP_DEGREES = 10.0;
    private static final boolean ENABLE_VISION_EVENT_LOGS = false;
    private static final boolean ENABLE_VERBOSE_VISION_DIAGNOSTICS = false;

    private final Drive drive;
    private final Supplier<Pose2d> robotPoseSupplier;
    private final List<VisionIO> ios;
    private final List<VisionIOInputsAutoLogged> inputs;
    private final int[] visionRejectEventCountByCamera = new int[CAMERA_NAMES.size()];
    private final int[] visionJumpEventCountByCamera = new int[CAMERA_NAMES.size()];

    private double hubYawRad = Double.NaN;
    private Pose2d hubTagRobotPose = null;
    private Pose2d unifiedRobotPoseRaw = null;
    private double unifiedRobotPoseRawTimestampSeconds = Double.NaN;
    private Pose2d unifiedRobotPose = null;
    private double unifiedRobotPoseTimestampSeconds = Double.NaN;
    private int visionEventSequence = 0;
    private int visionRejectEventCount = 0;
    private int visionJumpEventCount = 0;

    public Vision(Drive drive) {
        super("vision");
        this.drive = drive;
        this.robotPoseSupplier = drive::getPose;
        this.ios = createIOs();
        this.inputs = ios.stream().map(io -> new VisionIOInputsAutoLogged()).toList();

        Logger.recordOutput("vision/events/sequence", visionEventSequence);
        Logger.recordOutput("vision/events/rejectCount", visionRejectEventCount);
        Logger.recordOutput("vision/events/jumpCount", visionJumpEventCount);
        Logger.recordOutput("vision/events/last/type", "none");
        Logger.recordOutput("vision/events/last/cameraIndex", -1);
        Logger.recordOutput("vision/events/last/tagIds", new int[0]);
        if (ENABLE_VERBOSE_VISION_DIAGNOSTICS) {
            Logger.recordOutput("vision/events/rejectCountByCamera", Arrays.copyOf(visionRejectEventCountByCamera, visionRejectEventCountByCamera.length));
            Logger.recordOutput("vision/events/jumpCountByCamera", Arrays.copyOf(visionJumpEventCountByCamera, visionJumpEventCountByCamera.length));
            resetFrameEventOutputs();
            resetJumpDiagnosticsOutputs();
        }
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
     * Returns the most recent filtered vision pose that passed consistency checks.
     * Returns null when unavailable or stale.
     */
    public Pose2d getUnifiedRobotPose() {
        return unifiedRobotPose;
    }

    /**
     * Returns the most recent raw unified vision pose before consistency checks.
     * Returns null when unavailable or stale.
     */
    public Pose2d getUnifiedRobotPoseRaw() {
        return unifiedRobotPoseRaw;
    }

    @Override
    public void periodic() {
        Pose2d currentPose = robotPoseSupplier.get();
        PoseObservation bestRawObservation = null;
        PoseObservation bestConsistentObservation = null;
        Pose2d bestConsistentMeasuredPose = null;
        List<CandidateObservationDiagnostics> candidateDiagnostics =
                ENABLE_VERBOSE_VISION_DIAGNOSTICS ? new ArrayList<>() : null;

        if (ENABLE_VERBOSE_VISION_DIAGNOSTICS) {
            resetFrameEventOutputs();
            resetJumpDiagnosticsOutputs();
        }

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
            if (bestObservation == null) {
                continue;
            }

            Pose2d measuredPose = bestObservation.pose().toPose2d();
            if (isBetterObservation(bestObservation, bestRawObservation)) {
                bestRawObservation = bestObservation;
            }

            double stdDevFactor = Math.pow(bestObservation.averageTagDistance(), 2) / bestObservation.tagCount();
            double clampedFactor = Math.max(1.0, stdDevFactor);
            double linearStdDev = LINEAR_STD_DEV_BASELINE * clampedFactor;
            double angularStdDev = ANGULAR_STD_DEV_BASELINE * clampedFactor;
            PoseDelta innovation = calculatePoseDelta(measuredPose, currentPose);
            boolean consistent = isVisionMeasurementConsistent(innovation);

            if (candidateDiagnostics != null) {
                candidateDiagnostics.add(new CandidateObservationDiagnostics(
                        measuredPose,
                        innovation,
                        index,
                        bestObservation.tagCount(),
                        bestObservation.ambiguity(),
                        bestObservation.averageTagDistance(),
                        consistent));
            }

            if (!consistent) {
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

            // Let vision correct field translation while keeping heading anchored to gyro.
            Pose2d estimatorMeasurementPose = new Pose2d(measuredPose.getTranslation(), currentPose.getRotation());
            drive.addVisionMeasurement(
                    estimatorMeasurementPose,
                    bestObservation.timestampSeconds(),
                    VecBuilder.fill(linearStdDev, linearStdDev, ESTIMATOR_ANGULAR_STD_DEV_RAD));

            if (isBetterObservation(bestObservation, bestConsistentObservation)) {
                bestConsistentObservation = bestObservation;
                bestConsistentMeasuredPose = measuredPose;
            }

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

        updateUnifiedRawPose(bestRawObservation);
        updateUnifiedPose(
                bestConsistentMeasuredPose,
                bestConsistentObservation != null ? bestConsistentObservation.timestampSeconds() : Double.NaN);

        if (ENABLE_VERBOSE_VISION_DIAGNOSTICS) {
            logCandidateDiagnostics(candidateDiagnostics);
            logUnifiedPoseDiagnostics(currentPose);
        }

        Logger.recordOutput("vision/unifiedRobotPoseRaw", unifiedRobotPoseRaw != null ? unifiedRobotPoseRaw : new Pose2d());
        Logger.recordOutput("vision/unifiedRobotPoseRawValid", unifiedRobotPoseRaw != null);
        Logger.recordOutput("vision/unifiedRobotPose", unifiedRobotPose != null ? unifiedRobotPose : new Pose2d());
        Logger.recordOutput("vision/unifiedRobotPoseValid", unifiedRobotPose != null);

        if (ENABLE_VERBOSE_VISION_DIAGNOSTICS) {
            Logger.recordOutput(
                    "vision/cameraPoses",
                    currentPose == null
                            ? new Pose3d[0]
                            : ROBOT_TO_CAMERAS.stream()
                                    .map(transform -> new Pose3d(currentPose).transformBy(transform))
                                    .toArray(Pose3d[]::new));
        }

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

    private void updateUnifiedRawPose(PoseObservation bestRawObservation) {
        if (bestRawObservation != null) {
            Pose2d newRawPose = bestRawObservation.pose().toPose2d();
            if (ENABLE_VERBOSE_VISION_DIAGNOSTICS) {
                logUnifiedRawStepDiagnostics(newRawPose);
            }
            unifiedRobotPoseRaw = newRawPose;
            unifiedRobotPoseRawTimestampSeconds = bestRawObservation.timestampSeconds();
            return;
        }

        if (unifiedRobotPoseRaw == null) {
            return;
        }

        double ageSeconds = Timer.getFPGATimestamp() - unifiedRobotPoseRawTimestampSeconds;
        if (!Double.isFinite(ageSeconds) || ageSeconds > UNIFIED_RAW_POSE_MAX_AGE_SECONDS) {
            unifiedRobotPoseRaw = null;
            unifiedRobotPoseRawTimestampSeconds = Double.NaN;
        }
    }

    private void updateUnifiedPose(Pose2d bestConsistentPose, double bestConsistentTimestampSeconds) {
        if (bestConsistentPose != null && Double.isFinite(bestConsistentTimestampSeconds)) {
            Pose2d previousPose = unifiedRobotPose;
            unifiedRobotPose = previousPose == null
                    ? bestConsistentPose
                    : smoothPose(previousPose, bestConsistentPose);
            unifiedRobotPoseTimestampSeconds = bestConsistentTimestampSeconds;
            if (ENABLE_VERBOSE_VISION_DIAGNOSTICS) {
                logUnifiedFilteredStepDiagnostics(previousPose, unifiedRobotPose);
            }
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

    private static Pose2d smoothPose(Pose2d previous, Pose2d measured) {
        Translation2d delta = measured.getTranslation().minus(previous.getTranslation());
        double deltaNorm = delta.getNorm();
        double requestedTranslationStep = deltaNorm * UNIFIED_POSE_SMOOTH_ALPHA;
        double clampedTranslationStep = Math.min(requestedTranslationStep, UNIFIED_POSE_MAX_STEP_METERS);
        Translation2d newTranslation = deltaNorm < 1e-9
                ? previous.getTranslation()
                : previous.getTranslation().plus(delta.times(clampedTranslationStep / deltaNorm));

        double headingDeltaRad =
                MathUtil.angleModulus(measured.getRotation().minus(previous.getRotation()).getRadians());
        double requestedHeadingStepRad = headingDeltaRad * UNIFIED_POSE_SMOOTH_ALPHA;
        double clampedHeadingStepRad = MathUtil.clamp(
                requestedHeadingStepRad,
                -Units.degreesToRadians(UNIFIED_POSE_MAX_STEP_DEGREES),
                Units.degreesToRadians(UNIFIED_POSE_MAX_STEP_DEGREES));
        Rotation2d newRotation = previous.getRotation().plus(new Rotation2d(clampedHeadingStepRad));
        return new Pose2d(newTranslation, newRotation);
    }

    private static PoseDelta calculatePoseDelta(Pose2d measuredPose, Pose2d referencePose) {
        Translation2d deltaTranslation = measuredPose.getTranslation().minus(referencePose.getTranslation());
        double translationDelta = deltaTranslation.getNorm();
        double headingDeltaDeg = calculateHeadingDeltaDegrees(measuredPose.getRotation(), referencePose.getRotation());
        return new PoseDelta(translationDelta, headingDeltaDeg);
    }

    private static double calculateHeadingDeltaDegrees(Rotation2d first, Rotation2d second) {
        return Units.radiansToDegrees(
                Math.abs(Math.IEEEremainder(first.minus(second).getRadians(), 2.0 * Math.PI)));
    }

    private void logUnifiedRawStepDiagnostics(Pose2d newRawPose) {
        if (unifiedRobotPoseRaw == null) {
            return;
        }
        PoseDelta step = calculatePoseDelta(newRawPose, unifiedRobotPoseRaw);
        Logger.recordOutput("vision/jump/rawStepTranslationMeters", step.translationMeters());
        Logger.recordOutput("vision/jump/rawStepHeadingDegrees", step.headingDegrees());
        Logger.recordOutput(
                "vision/jump/rawStepIsLarge",
                step.translationMeters() > VISION_JUMP_TRANSLATION_THRESHOLD_METERS
                        || step.headingDegrees() > VISION_JUMP_HEADING_THRESHOLD_DEGREES);
    }

    private void logUnifiedFilteredStepDiagnostics(Pose2d previousPose, Pose2d newPose) {
        if (previousPose == null || newPose == null) {
            return;
        }
        PoseDelta step = calculatePoseDelta(newPose, previousPose);
        Logger.recordOutput("vision/jump/filteredStepTranslationMeters", step.translationMeters());
        Logger.recordOutput("vision/jump/filteredStepHeadingDegrees", step.headingDegrees());
        Logger.recordOutput(
                "vision/jump/filteredStepIsLarge",
                step.translationMeters() > VISION_JUMP_TRANSLATION_THRESHOLD_METERS
                        || step.headingDegrees() > VISION_JUMP_HEADING_THRESHOLD_DEGREES);
    }

    private void logUnifiedPoseDiagnostics(Pose2d currentPose) {
        if (currentPose == null) {
            Logger.recordOutput("vision/jump/rawToOdometryTranslationMeters", Double.NaN);
            Logger.recordOutput("vision/jump/rawToOdometryHeadingDegrees", Double.NaN);
            Logger.recordOutput("vision/jump/filteredToOdometryTranslationMeters", Double.NaN);
            Logger.recordOutput("vision/jump/filteredToOdometryHeadingDegrees", Double.NaN);
            return;
        }

        if (unifiedRobotPoseRaw != null) {
            PoseDelta rawDelta = calculatePoseDelta(unifiedRobotPoseRaw, currentPose);
            Logger.recordOutput("vision/jump/rawToOdometryTranslationMeters", rawDelta.translationMeters());
            Logger.recordOutput("vision/jump/rawToOdometryHeadingDegrees", rawDelta.headingDegrees());
        } else {
            Logger.recordOutput("vision/jump/rawToOdometryTranslationMeters", Double.NaN);
            Logger.recordOutput("vision/jump/rawToOdometryHeadingDegrees", Double.NaN);
        }

        if (unifiedRobotPose != null) {
            PoseDelta filteredDelta = calculatePoseDelta(unifiedRobotPose, currentPose);
            Logger.recordOutput("vision/jump/filteredToOdometryTranslationMeters", filteredDelta.translationMeters());
            Logger.recordOutput("vision/jump/filteredToOdometryHeadingDegrees", filteredDelta.headingDegrees());
        } else {
            Logger.recordOutput("vision/jump/filteredToOdometryTranslationMeters", Double.NaN);
            Logger.recordOutput("vision/jump/filteredToOdometryHeadingDegrees", Double.NaN);
        }
    }

    private void logCandidateDiagnostics(List<CandidateObservationDiagnostics> candidates) {
        Pose2d[] allPoses = new Pose2d[candidates.size()];
        Pose2d[] acceptedPoses = candidates.stream()
                .filter(CandidateObservationDiagnostics::accepted)
                .map(CandidateObservationDiagnostics::measuredPose)
                .toArray(Pose2d[]::new);
        Pose2d[] rejectedPoses = candidates.stream()
                .filter(candidate -> !candidate.accepted())
                .map(CandidateObservationDiagnostics::measuredPose)
                .toArray(Pose2d[]::new);

        double[] allTranslationDeltas = new double[candidates.size()];
        double[] allHeadingDeltas = new double[candidates.size()];
        double[] allAmbiguities = new double[candidates.size()];
        double[] allAvgDistances = new double[candidates.size()];
        int[] allCameraIndices = new int[candidates.size()];
        int[] allTagCounts = new int[candidates.size()];
        int[] allAcceptedFlags = new int[candidates.size()];

        double maxTranslation = Double.NaN;
        double maxHeading = Double.NaN;
        for (int i = 0; i < candidates.size(); i++) {
            CandidateObservationDiagnostics candidate = candidates.get(i);
            allPoses[i] = candidate.measuredPose();
            allTranslationDeltas[i] = candidate.innovation().translationMeters();
            allHeadingDeltas[i] = candidate.innovation().headingDegrees();
            allAmbiguities[i] = candidate.ambiguity();
            allAvgDistances[i] = candidate.averageTagDistance();
            allCameraIndices[i] = candidate.cameraIndex();
            allTagCounts[i] = candidate.tagCount();
            allAcceptedFlags[i] = candidate.accepted() ? 1 : 0;

            if (!Double.isFinite(maxTranslation) || allTranslationDeltas[i] > maxTranslation) {
                maxTranslation = allTranslationDeltas[i];
            }
            if (!Double.isFinite(maxHeading) || allHeadingDeltas[i] > maxHeading) {
                maxHeading = allHeadingDeltas[i];
            }
        }

        Logger.recordOutput("vision/candidates/count", candidates.size());
        Logger.recordOutput("vision/candidates/poses", allPoses);
        Logger.recordOutput("vision/candidates/acceptedPoses", acceptedPoses);
        Logger.recordOutput("vision/candidates/rejectedPoses", rejectedPoses);
        Logger.recordOutput("vision/candidates/innovationTranslationMeters", allTranslationDeltas);
        Logger.recordOutput("vision/candidates/innovationHeadingDegrees", allHeadingDeltas);
        Logger.recordOutput("vision/candidates/ambiguity", allAmbiguities);
        Logger.recordOutput("vision/candidates/averageDistanceMeters", allAvgDistances);
        Logger.recordOutput("vision/candidates/cameraIndex", allCameraIndices);
        Logger.recordOutput("vision/candidates/tagCount", allTagCounts);
        Logger.recordOutput("vision/candidates/acceptedFlag", allAcceptedFlags);
        Logger.recordOutput("vision/candidates/maxInnovationTranslationMeters", maxTranslation);
        Logger.recordOutput("vision/candidates/maxInnovationHeadingDegrees", maxHeading);
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
        int[] safeTagIds = tagIds == null ? new int[0] : Arrays.copyOf(tagIds, tagIds.length);
        visionEventSequence++;
        if ("reject".equals(eventType)) {
            visionRejectEventCount++;
            if (cameraIndex >= 0 && cameraIndex < visionRejectEventCountByCamera.length) {
                visionRejectEventCountByCamera[cameraIndex]++;
            }
        } else if ("jump".equals(eventType)) {
            visionJumpEventCount++;
            if (cameraIndex >= 0 && cameraIndex < visionJumpEventCountByCamera.length) {
                visionJumpEventCountByCamera[cameraIndex]++;
            }
        }

        Logger.recordOutput("vision/events/sequence", visionEventSequence);
        Logger.recordOutput("vision/events/rejectCount", visionRejectEventCount);
        Logger.recordOutput("vision/events/jumpCount", visionJumpEventCount);

        recordVisionEventDetails("vision/events/last", eventType, timestampSeconds, measuredPose, odometryPose, innovation,
                tagCount, safeTagIds, ambiguity, avgDistance, linearStdDev, angularStdDev, cameraIndex);
        if (ENABLE_VERBOSE_VISION_DIAGNOSTICS) {
            Logger.recordOutput("vision/events/rejectCountByCamera", Arrays.copyOf(visionRejectEventCountByCamera, visionRejectEventCountByCamera.length));
            Logger.recordOutput("vision/events/jumpCountByCamera", Arrays.copyOf(visionJumpEventCountByCamera, visionJumpEventCountByCamera.length));
            recordVisionEventDetails("vision/events/frame", eventType, timestampSeconds, measuredPose, odometryPose, innovation,
                    tagCount, safeTagIds, ambiguity, avgDistance, linearStdDev, angularStdDev, cameraIndex);
        }

        if (!ENABLE_VISION_EVENT_LOGS) {
            return;
        }
        String tagList = Arrays.toString(safeTagIds);
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

    private void recordVisionEventDetails(
            String keyPrefix,
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
        Logger.recordOutput(keyPrefix + "/type", eventType);
        Logger.recordOutput(keyPrefix + "/timestampSeconds", timestampSeconds);
        Logger.recordOutput(keyPrefix + "/cameraIndex", cameraIndex);
        Logger.recordOutput(keyPrefix + "/measuredPose", measuredPose);
        Logger.recordOutput(keyPrefix + "/odometryPose", odometryPose);
        Logger.recordOutput(keyPrefix + "/translationDeltaMeters", innovation.translationMeters());
        Logger.recordOutput(keyPrefix + "/headingDeltaDegrees", innovation.headingDegrees());
        Logger.recordOutput(keyPrefix + "/tagCount", tagCount);
        Logger.recordOutput(keyPrefix + "/tagIds", tagIds);
        Logger.recordOutput(keyPrefix + "/ambiguity", ambiguity);
        Logger.recordOutput(keyPrefix + "/avgDistanceMeters", avgDistance);
        Logger.recordOutput(keyPrefix + "/linearStdDev", linearStdDev);
        Logger.recordOutput(keyPrefix + "/angularStdDev", angularStdDev);
    }

    private void resetFrameEventOutputs() {
        Logger.recordOutput("vision/events/frame/type", "none");
        Logger.recordOutput("vision/events/frame/timestampSeconds", Double.NaN);
        Logger.recordOutput("vision/events/frame/cameraIndex", -1);
        Logger.recordOutput("vision/events/frame/measuredPose", new Pose2d());
        Logger.recordOutput("vision/events/frame/odometryPose", new Pose2d());
        Logger.recordOutput("vision/events/frame/translationDeltaMeters", Double.NaN);
        Logger.recordOutput("vision/events/frame/headingDeltaDegrees", Double.NaN);
        Logger.recordOutput("vision/events/frame/tagCount", 0);
        Logger.recordOutput("vision/events/frame/tagIds", new int[0]);
        Logger.recordOutput("vision/events/frame/ambiguity", Double.NaN);
        Logger.recordOutput("vision/events/frame/avgDistanceMeters", Double.NaN);
        Logger.recordOutput("vision/events/frame/linearStdDev", Double.NaN);
        Logger.recordOutput("vision/events/frame/angularStdDev", Double.NaN);
    }

    private void resetJumpDiagnosticsOutputs() {
        Logger.recordOutput("vision/jump/rawStepTranslationMeters", Double.NaN);
        Logger.recordOutput("vision/jump/rawStepHeadingDegrees", Double.NaN);
        Logger.recordOutput("vision/jump/rawStepIsLarge", false);
        Logger.recordOutput("vision/jump/filteredStepTranslationMeters", Double.NaN);
        Logger.recordOutput("vision/jump/filteredStepHeadingDegrees", Double.NaN);
        Logger.recordOutput("vision/jump/filteredStepIsLarge", false);
    }

    private record PoseDelta(double translationMeters, double headingDegrees) {
    }

    private record CandidateObservationDiagnostics(
            Pose2d measuredPose,
            PoseDelta innovation,
            int cameraIndex,
            int tagCount,
            double ambiguity,
            double averageTagDistance,
            boolean accepted) {
    }
}
