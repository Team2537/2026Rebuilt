package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import frc.robot.RobotType;
import frc.robot.subsystems.vision.VisionIO.PoseObservation;
import frc.robot.subsystems.vision.VisionIO.TargetTransform;
import frc.robot.util.ElasticNotifications;
import frc.robot.util.FieldConstants;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.Comparator;
import java.util.function.Supplier;
import java.util.stream.Collectors;
import java.util.stream.IntStream;
import org.littletonrobotics.junction.Logger;

/**
 * Vision subsystem responsible for camera IO, per-frame filtering, and the
 * latest consensus vision pose used by {@link RobotState}.
 */
public final class Vision extends SubsystemBase {
    private static final double VISIBLE_TAG_HOLD_SECONDS = 0.25;

    private final RobotState robotState;
    private final Supplier<Pose2d> robotPoseSupplier;
    private final List<VisionIO> ios;
    private final List<VisionIOInputsAutoLogged> inputs;
    private final List<String> cameraLogKeys;
    private final int[] visionRejectEventCountByCamera = new int[VisionConstants.CAMERA_CONFIGS.size()];
    private final int[] visionJumpEventCountByCamera = new int[VisionConstants.CAMERA_CONFIGS.size()];

    private double hubYawRad = Double.NaN;
    private Pose2d hubTagRobotPose = null;
    private Pose2d latestRawPose = null;
    private double latestRawPoseTimestampSeconds = Double.NaN;
    private VisionConsensus latestConsensus = null;
    private int visionEventSequence = 0;
    private int visionRejectEventCount = 0;
    private int visionJumpEventCount = 0;
    private boolean lastAllCamerasConnected = true;
    private boolean dashboardDisabled = false;
    private int[] latestVisibleTagIds = new int[0];
    private final HashMap<Integer, Double> lastSeenTagTimestampsSec = new HashMap<>();

    public Vision(RobotState robotState) {
        super("Vision");
        this.robotState = robotState;
        this.robotPoseSupplier = robotState::getPose;
        if (RobotType.MODE == RobotType.Mode.SIMULATION) {
            VisionIOPhotonVisionSim.resetSharedVisionSim();
        }
        this.ios = createIOs();
        this.inputs = ios.stream().map(io -> new VisionIOInputsAutoLogged()).toList();
        this.cameraLogKeys = new ArrayList<>(ios.size());
        for (int i = 0; i < ios.size(); i++) {
            cameraLogKeys.add(getName() + "/Camera" + i);
        }

        Logger.recordOutput("Vision/events/sequence", visionEventSequence);
        Logger.recordOutput("Vision/events/rejectCount", visionRejectEventCount);
        Logger.recordOutput("Vision/events/jumpCount", visionJumpEventCount);
        Logger.recordOutput("Vision/events/last/type", "none");
        Logger.recordOutput("Vision/events/last/cameraIndex", -1);
        Logger.recordOutput("Vision/events/last/tagIds", new int[0]);
        if (VisionConstants.ENABLE_VERBOSE_VISION_DIAGNOSTICS) {
            Logger.recordOutput("Vision/events/rejectCountByCamera", Arrays.copyOf(visionRejectEventCountByCamera, visionRejectEventCountByCamera.length));
            Logger.recordOutput("Vision/events/jumpCountByCamera", Arrays.copyOf(visionJumpEventCountByCamera, visionJumpEventCountByCamera.length));
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

    /** Returns the most recent consensus pose, or null when unavailable or stale. */
    public Pose2d getConsensusPose() {
        VisionConsensus consensus = getLatestConsensus();
        return consensus != null ? consensus.pose() : null;
    }

    /** Returns the most recent consensus object used for fusion and diagnostics. */
    public VisionConsensus getLatestConsensus() {
        return latestConsensus != null
                        && latestConsensus.isUsable()
                        && isFresh(latestConsensus.timestampSeconds(), VisionConstants.CONSENSUS_POSE_MAX_AGE_SECONDS)
                ? latestConsensus
                : null;
    }

    /** Returns the most recent raw vision pose before consensus filtering. */
    public Pose2d getRawPose() {
        return isFresh(latestRawPoseTimestampSeconds, VisionConstants.RAW_POSE_MAX_AGE_SECONDS)
                ? latestRawPose
                : null;
    }

    /** Returns AprilTag IDs currently visible from active camera results. */
    public int[] getVisibleAprilTagIds() {
        return Arrays.copyOf(latestVisibleTagIds, latestVisibleTagIds.length);
    }

    /** Allows the dashboard to disable vision pose updates at runtime. */
    public void setDashboardDisabled(boolean disabled) {
        this.dashboardDisabled = disabled;
    }

    @Override
    public void periodic() {
        Logger.recordOutput("RobotState/VisionDisabledOverride", dashboardDisabled);

        Pose2d currentPose = robotPoseSupplier.get();
        List<CandidateObservationDiagnostics> candidateDiagnostics =
                VisionConstants.ENABLE_VERBOSE_VISION_DIAGNOSTICS ? new ArrayList<>() : null;

        if (VisionConstants.ENABLE_VERBOSE_VISION_DIAGNOSTICS) {
            resetFrameEventOutputs();
            resetJumpDiagnosticsOutputs();
        }

        VisionCycleResult cycleResult = processVisionCycle(currentPose, candidateDiagnostics);
        updateLatestVisibleAprilTagIds();
        updateLatestRawPose(cycleResult.bestRawObservation());
        updateLatestConsensus(cycleResult.consensus());
        seedVisionStateFromRobotPoseInSimulation(currentPose);
        robotState.addVisionConsensus(dashboardDisabled ? null : getLatestConsensus());

        if (VisionConstants.ENABLE_VERBOSE_VISION_DIAGNOSTICS) {
            logCandidateDiagnostics(candidateDiagnostics);
            logConsensusDiagnostics(currentPose);
        }

        publishPoseOutputs();
        updateCameraConnectivityAndDiagnostics(currentPose);
        updateHubTagTracking();
    }

    private VisionCycleResult processVisionCycle(
            Pose2d currentPose,
            List<CandidateObservationDiagnostics> candidateDiagnostics) {
        PoseObservation bestRawObservation = null;
        List<ConsensusObservation> consensusCandidates = new ArrayList<>();

        for (int index = 0; index < ios.size(); index++) {
            VisionIO io = ios.get(index);
            VisionIOInputsAutoLogged input = inputs.get(index);
            input.clearFrameData();
            io.updateInputs(input);
            Logger.processInputs(cameraLogKeys.get(index), input);

            if (currentPose == null) {
                continue;
            }

            PoseObservation bestObservation = selectBestObservation(input.poseObservations);
            if (bestObservation == null) {
                continue;
            }
            if (isBetterObservation(bestObservation, bestRawObservation)) {
                bestRawObservation = bestObservation;
            }

            Pose2d measuredPose = bestObservation.pose().toPose2d();
            Pose2d referencePose = robotState.getPoseAtTimestamp(bestObservation.timestampSeconds());
            ObservationStdDevs stdDevs = computeObservationStdDevs(bestObservation);
            PoseDelta innovation = calculatePoseDelta(measuredPose, referencePose);
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
                        referencePose,
                        innovation,
                        bestObservation.tagCount(),
                        input.tagIds,
                        bestObservation.ambiguity(),
                        bestObservation.averageTagDistance(),
                        stdDevs.linearStdDev(),
                        stdDevs.angularStdDev(),
                        index);
            } else {
                logVisionJumpIfLarge(
                        bestObservation.timestampSeconds(),
                        measuredPose,
                        referencePose,
                        innovation,
                        bestObservation.tagCount(),
                        input.tagIds,
                        bestObservation.ambiguity(),
                        bestObservation.averageTagDistance(),
                        stdDevs.linearStdDev(),
                        stdDevs.angularStdDev(),
                        index);
            }

            consensusCandidates.add(new ConsensusObservation(bestObservation, measuredPose, stdDevs));
        }

        return new VisionCycleResult(
                bestRawObservation,
                buildVisionConsensus(consensusCandidates));
    }

    private static ObservationStdDevs computeObservationStdDevs(PoseObservation bestObservation) {
        double stdDevFactor = Math.pow(bestObservation.averageTagDistance(), 2) / bestObservation.tagCount();
        double clampedFactor = Math.max(1.0, stdDevFactor);
        return new ObservationStdDevs(
                VisionConstants.LINEAR_STD_DEV_BASELINE * clampedFactor,
                VisionConstants.ANGULAR_STD_DEV_BASELINE * clampedFactor);
    }

    private VisionConsensus buildVisionConsensus(List<ConsensusObservation> observations) {
        if (observations.isEmpty()) {
            return null;
        }

        ConsensusObservation seed = observations.stream()
                .max(Comparator.comparingDouble(ConsensusObservation::weight))
                .orElse(null);
        if (seed == null) {
            return null;
        }

        List<ConsensusObservation> cluster = observations.stream()
                .filter(observation -> isConsensusCompatible(seed.measuredPose(), observation.measuredPose()))
                .toList();
        if (cluster.isEmpty()) {
            return null;
        }

        double totalWeight = 0.0;
        double sumX = 0.0;
        double sumY = 0.0;
        double sumCos = 0.0;
        double sumSin = 0.0;
        double weightedLinearStdDev = 0.0;
        double weightedAngularStdDev = 0.0;
        double weightedAmbiguity = 0.0;
        double weightedDistance = 0.0;
        double timestampSeconds = Double.NEGATIVE_INFINITY;
        int totalTagCount = 0;
        boolean multiTag = false;

        for (ConsensusObservation observation : cluster) {
            double weight = observation.weight();
            totalWeight += weight;
            Pose2d pose = observation.measuredPose();
            sumX += pose.getX() * weight;
            sumY += pose.getY() * weight;
            sumCos += pose.getRotation().getCos() * weight;
            sumSin += pose.getRotation().getSin() * weight;
            weightedLinearStdDev += observation.stdDevs().linearStdDev() * weight;
            weightedAngularStdDev += observation.stdDevs().angularStdDev() * weight;
            weightedAmbiguity += observation.observation().ambiguity() * weight;
            weightedDistance += observation.observation().averageTagDistance() * weight;
            timestampSeconds = Math.max(timestampSeconds, observation.observation().timestampSeconds());
            totalTagCount += observation.observation().tagCount();
            multiTag |= observation.observation().tagCount() > 1;
        }

        if (!(totalWeight > 0.0) || !Double.isFinite(timestampSeconds)) {
            return null;
        }

        Pose2d consensusPose = new Pose2d(
                sumX / totalWeight,
                sumY / totalWeight,
                new Rotation2d(sumCos, sumSin));
        double translationStdDev = (weightedLinearStdDev / totalWeight) / Math.sqrt(cluster.size());
        double rotationStdDev = (weightedAngularStdDev / totalWeight) / Math.sqrt(cluster.size());
        double averageAmbiguity = weightedAmbiguity / totalWeight;
        double averageDistance = weightedDistance / totalWeight;
        double confidence = computeConsensusConfidence(
                cluster.size(),
                totalTagCount,
                averageAmbiguity,
                averageDistance,
                multiTag);

        return new VisionConsensus(
                timestampSeconds,
                consensusPose,
                confidence,
                Math.max(0.01, translationStdDev),
                Math.max(Units.degreesToRadians(2.0), rotationStdDev),
                cluster.size(),
                totalTagCount,
                multiTag);
    }

    private static boolean isConsensusCompatible(Pose2d seedPose, Pose2d candidatePose) {
        PoseDelta delta = calculatePoseDelta(seedPose, candidatePose);
        return delta.translationMeters() <= VisionConstants.MAX_CONSENSUS_TRANSLATION_DELTA_METERS
                && delta.headingDegrees() <= VisionConstants.MAX_CONSENSUS_HEADING_DELTA_DEGREES;
    }

    private static double computeConsensusConfidence(
            int contributingCameraCount,
            int totalTagCount,
            double averageAmbiguity,
            double averageDistance,
            boolean multiTag) {
        double cameraFactor = Math.min(1.0, contributingCameraCount / (double) Math.max(1, VisionConstants.CAMERA_CONFIGS.size()));
        double tagFactor = Math.min(1.0, totalTagCount / 4.0);
        double ambiguityFactor = 1.0 - Math.min(1.0, Math.max(0.0, averageAmbiguity));
        double distanceFactor = Math.min(1.0, 1.0 / Math.max(1.0, averageDistance));
        double confidence = (0.30 * cameraFactor)
                + (0.35 * tagFactor)
                + (0.20 * ambiguityFactor)
                + (0.15 * distanceFactor);
        if (multiTag) {
            confidence += 0.10;
        }
        return Math.max(0.0, Math.min(1.0, confidence));
    }

    private void publishPoseOutputs() {
        VisionConsensus consensus = getLatestConsensus();
        Pose2d rawPose = getRawPose();
        Logger.recordOutput("Vision/rawPose", rawPose != null ? rawPose : new Pose2d());
        Logger.recordOutput("Vision/rawPoseValid", rawPose != null);
        Logger.recordOutput("Vision/consensus/pose", consensus != null ? consensus.pose() : new Pose2d());
        Logger.recordOutput("Vision/consensus/valid", consensus != null);
        Logger.recordOutput("Vision/consensus/confidence", consensus != null ? consensus.confidence() : Double.NaN);
        Logger.recordOutput(
                "Vision/consensus/translationStdDevMeters",
                consensus != null ? consensus.translationStdDevMeters() : Double.NaN);
        Logger.recordOutput(
                "Vision/consensus/rotationStdDevRadians",
                consensus != null ? consensus.rotationStdDevRadians() : Double.NaN);
        Logger.recordOutput(
                "Vision/consensus/contributingCameraCount",
                consensus != null ? consensus.contributingCameraCount() : 0);
        Logger.recordOutput("Vision/consensus/tagCount", consensus != null ? consensus.tagCount() : 0);
        Logger.recordOutput("Vision/consensus/multiTag", consensus != null && consensus.multiTag());
    }

    private void updateCameraConnectivityAndDiagnostics(Pose2d currentPose) {
        boolean allCamerasConnected = true;
        for (int i = 0; i < inputs.size(); i++) {
            boolean connected = inputs.get(i).isConnected;
            Logger.recordOutput("RobotState/Vision/Camera" + i + "Connected", connected);
            allCamerasConnected = allCamerasConnected && connected;
        }
        Logger.recordOutput("RobotState/VisionConnected", allCamerasConnected);
        if (!allCamerasConnected && lastAllCamerasConnected) {
            ElasticNotifications.sendWarning("Vision", "Camera disconnected");
        }
        lastAllCamerasConnected = allCamerasConnected;

        if (VisionConstants.ENABLE_VERBOSE_VISION_DIAGNOSTICS) {
            Logger.recordOutput(
                    "Vision/cameraPoses",
                    currentPose == null
                            ? new Pose3d[0]
                            : VisionConstants.CAMERA_CONFIGS.stream()
                                    .map(VisionConstants.CameraConfig::robotToCamera)
                                    .map(transform -> new Pose3d(currentPose).transformBy(transform))
                                    .toArray(Pose3d[]::new));
        }
    }

    private void updateLatestVisibleAprilTagIds() {
        double nowSec = Timer.getFPGATimestamp();
        for (VisionIOInputsAutoLogged input : inputs) {
            for (TargetTransform targetTransform : input.targetTransforms) {
                int tagId = targetTransform.fiducialId();
                if (tagId > 0) {
                    lastSeenTagTimestampsSec.put(tagId, nowSec);
                }
            }
            for (int tagId : input.tagIds) {
                if (tagId > 0) {
                    lastSeenTagTimestampsSec.put(tagId, nowSec);
                }
            }
        }

        lastSeenTagTimestampsSec.entrySet().removeIf(entry -> nowSec - entry.getValue() > VISIBLE_TAG_HOLD_SECONDS);

        int[] tagIds = new int[lastSeenTagTimestampsSec.size()];
        int index = 0;
        for (Map.Entry<Integer, Double> entry : lastSeenTagTimestampsSec.entrySet()) {
            Integer tagId = entry.getKey();
            tagIds[index++] = tagId;
        }
        Arrays.sort(tagIds);
        latestVisibleTagIds = tagIds;
        Logger.recordOutput("Vision/visibleTagIds", latestVisibleTagIds);
    }

    private List<VisionIO> createIOs() {
        return switch (RobotType.MODE) {
            case REAL -> IntStream.range(0, VisionConstants.CAMERA_CONFIGS.size())
                    .mapToObj(index -> (VisionIO) new VisionIOPhotonVision(
                            VisionConstants.CAMERA_CONFIGS.get(index).name(),
                            VisionConstants.CAMERA_CONFIGS.get(index).robotToCamera(),
                            index))
                    .toList();
            case SIMULATION -> {
                // Use the ground truth pose (pure odometry, no vision corrections) so the
                // sim cameras see the real robot position and can correct estimator drift.
                Supplier<Pose2d> simTruthPoseSupplier = robotState::getSimGroundTruthPose;
                yield IntStream.range(0, VisionConstants.CAMERA_CONFIGS.size())
                        .mapToObj(index -> (VisionIO) new VisionIOPhotonVisionSim(
                                VisionConstants.CAMERA_CONFIGS.get(index).name(),
                                VisionConstants.CAMERA_CONFIGS.get(index).robotToCamera(),
                                index,
                                simTruthPoseSupplier))
                        .toList();
            }
            case REPLAY -> IntStream.range(0, VisionConstants.CAMERA_CONFIGS.size())
                    .mapToObj(index -> (VisionIO) new NullVisionIO())
                    .toList();
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

    private void updateLatestRawPose(PoseObservation bestRawObservation) {
        if (bestRawObservation != null) {
            latestRawPose = bestRawObservation.pose().toPose2d();
            latestRawPoseTimestampSeconds = bestRawObservation.timestampSeconds();
            return;
        }

        if (latestRawPose == null) {
            return;
        }

        if (!isFresh(latestRawPoseTimestampSeconds, VisionConstants.RAW_POSE_MAX_AGE_SECONDS)) {
            latestRawPose = null;
            latestRawPoseTimestampSeconds = Double.NaN;
        }
    }

    private void updateLatestConsensus(VisionConsensus consensus) {
        if (consensus != null && consensus.isUsable()) {
            latestConsensus = consensus;
            return;
        }

        if (latestConsensus == null) {
            return;
        }

        if (!isFresh(latestConsensus.timestampSeconds(), VisionConstants.CONSENSUS_POSE_MAX_AGE_SECONDS)) {
            latestConsensus = null;
        }
    }

    private void seedVisionStateFromRobotPoseInSimulation(Pose2d currentPose) {
        if (RobotType.MODE != RobotType.Mode.SIMULATION || currentPose == null) {
            return;
        }
        double nowSec = Timer.getFPGATimestamp();
        if (latestRawPose == null) {
            latestRawPose = currentPose;
            latestRawPoseTimestampSeconds = nowSec;
        }
        if (latestConsensus == null) {
            latestConsensus = new VisionConsensus(
                    nowSec,
                    currentPose,
                    1.0,
                    VisionConstants.LINEAR_STD_DEV_BASELINE,
                    VisionConstants.ANGULAR_STD_DEV_BASELINE,
                    1,
                    1,
                    true);
        }
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

    private static boolean isFresh(double timestampSeconds, double maxAgeSeconds) {
        if (!Double.isFinite(timestampSeconds)) {
            return false;
        }
        double ageSeconds = Timer.getFPGATimestamp() - timestampSeconds;
        return Double.isFinite(ageSeconds) && ageSeconds <= maxAgeSeconds;
    }

    private void logConsensusDiagnostics(Pose2d currentPose) {
        Pose2d consensusPose = getConsensusPose();
        if (currentPose == null || consensusPose == null) {
            Logger.recordOutput("Vision/jump/consensusToOdometryTranslationMeters", Double.NaN);
            Logger.recordOutput("Vision/jump/consensusToOdometryHeadingDegrees", Double.NaN);
            return;
        }

        PoseDelta delta = calculatePoseDelta(consensusPose, currentPose);
        Logger.recordOutput("Vision/jump/consensusToOdometryTranslationMeters", delta.translationMeters());
        Logger.recordOutput("Vision/jump/consensusToOdometryHeadingDegrees", delta.headingDegrees());
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

        Logger.recordOutput("Vision/candidates/count", candidates.size());
        Logger.recordOutput("Vision/candidates/poses", allPoses);
        Logger.recordOutput("Vision/candidates/acceptedPoses", acceptedPoses);
        Logger.recordOutput("Vision/candidates/rejectedPoses", rejectedPoses);
        Logger.recordOutput("Vision/candidates/innovationTranslationMeters", allTranslationDeltas);
        Logger.recordOutput("Vision/candidates/innovationHeadingDegrees", allHeadingDeltas);
        Logger.recordOutput("Vision/candidates/ambiguity", allAmbiguities);
        Logger.recordOutput("Vision/candidates/averageDistanceMeters", allAvgDistances);
        Logger.recordOutput("Vision/candidates/cameraIndex", allCameraIndices);
        Logger.recordOutput("Vision/candidates/tagCount", allTagCounts);
        Logger.recordOutput("Vision/candidates/acceptedFlag", allAcceptedFlags);
        Logger.recordOutput("Vision/candidates/maxInnovationTranslationMeters", maxTranslation);
        Logger.recordOutput("Vision/candidates/maxInnovationHeadingDegrees", maxHeading);
    }

    private boolean isPoseValid(PoseObservation observation) {
        if (observation == null || !Double.isFinite(observation.ambiguity())) {
            return false;
        }
        if (!isFinitePose(observation.pose().toPose2d()) || !isFinitePose(observation.pose())) {
            return false;
        }
        if (observation.tagCount() == 0) {
            return false;
        }
        if (observation.tagCount() == 1 && observation.ambiguity() > VisionConstants.MAX_AMBIGUITY) {
            return false;
        }
        if (Math.abs(observation.pose().getZ()) > VisionConstants.MAX_Z_ERROR) {
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

    private static boolean isFinitePose(Pose2d pose) {
        return pose != null
                && Double.isFinite(pose.getX())
                && Double.isFinite(pose.getY())
                && Double.isFinite(pose.getRotation().getRadians());
    }

    private static boolean isFinitePose(Pose3d pose) {
        return pose != null
                && Double.isFinite(pose.getX())
                && Double.isFinite(pose.getY())
                && Double.isFinite(pose.getZ())
                && Double.isFinite(pose.getRotation().getX())
                && Double.isFinite(pose.getRotation().getY())
                && Double.isFinite(pose.getRotation().getZ());
    }

    private void updateHubTagTracking() {
        double nowSeconds = Timer.getFPGATimestamp();
        TargetTransform bestHubTarget = findBestHubTarget(nowSeconds);

        if (bestHubTarget == null
                || bestHubTarget.cameraIndex() < 0
                || bestHubTarget.cameraIndex() >= VisionConstants.CAMERA_CONFIGS.size()) {
            hubYawRad = Double.NaN;
            hubTagRobotPose = null;
            Logger.recordOutput("Vision/hubYawRobotRad", Double.NaN);
            Logger.recordOutput("Vision/hubYawTagId", -1);
            Logger.recordOutput("Vision/hubRelativePose", new Pose3d());
            Logger.recordOutput("Vision/hubTagRobotPose", new Pose2d());
            return;
        }

        Transform3d robotToTarget = VisionConstants.CAMERA_CONFIGS
                .get(bestHubTarget.cameraIndex())
                .robotToCamera()
                .plus(bestHubTarget.cameraToTarget());
        hubYawRad = Math.atan2(robotToTarget.getY(), robotToTarget.getX());
        hubTagRobotPose = estimateRobotPoseFromHubTarget(bestHubTarget, robotToTarget);
        Logger.recordOutput("Vision/hubYawRobotRad", hubYawRad);
        Logger.recordOutput("Vision/hubYawTagId", bestHubTarget.fiducialId());
        Logger.recordOutput(
                "Vision/hubRelativePose",
                new Pose3d(robotToTarget.getTranslation(), new Rotation3d()));
        Logger.recordOutput(
                "Vision/hubTagRobotPose",
                hubTagRobotPose != null ? hubTagRobotPose : new Pose2d());
    }

    private Pose2d estimateRobotPoseFromHubTarget(TargetTransform target, Transform3d robotToTarget) {
        return FieldConstants.TAG_LAYOUT.getTagPose(target.fiducialId())
                .map(fieldToTarget -> fieldToTarget.transformBy(robotToTarget.inverse()).toPose2d())
                .orElse(null);
    }

    private TargetTransform findBestHubTarget(double nowSeconds) {
        TargetTransform bestHubTarget = null;
        Set<Integer> hubTagIds = determineHubTagIds(FieldConstants.getAllianceHubTagId());
        for (VisionIOInputsAutoLogged input : inputs) {
            for (TargetTransform target : input.targetTransforms) {
                if (!hubTagIds.contains(target.fiducialId())) {
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
        if (!Double.isFinite(target.distanceMeters()) || target.distanceMeters() > VisionConstants.HUB_YAW_MAX_DISTANCE_METERS) {
            return false;
        }
        if (!Double.isFinite(target.ambiguity()) || target.ambiguity() > VisionConstants.HUB_YAW_MAX_AMBIGUITY) {
            return false;
        }

        double ageSeconds = nowSeconds - target.timestampSeconds();
        return Double.isFinite(ageSeconds) && ageSeconds >= 0.0 && ageSeconds <= VisionConstants.HUB_YAW_MAX_AGE_SECONDS;
    }

    private static Set<Integer> determineHubTagIds(int referenceTagId) {
        return FieldConstants.TAG_LAYOUT.getTagPose(referenceTagId)
                .map(referenceTagPose -> FieldConstants.TAG_LAYOUT.getTags().stream()
                        .filter(tag -> tag.pose.getTranslation()
                                .getDistance(referenceTagPose.getTranslation()) <= VisionConstants.HUB_TAG_CLUSTER_RADIUS_METERS)
                        .map(tag -> tag.ID)
                        .collect(Collectors.toUnmodifiableSet()))
                .orElse(Set.of(referenceTagId));
    }

    private static final class NullVisionIO implements VisionIO {
    }

    private boolean isVisionMeasurementConsistent(PoseDelta innovation) {
        return innovation.translationMeters() <= VisionConstants.MAX_VISION_TRANSLATION_DELTA_METERS
                && innovation.headingDegrees() <= VisionConstants.MAX_VISION_HEADING_DELTA_DEGREES;
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
        if (innovation.translationMeters() <= VisionConstants.VISION_JUMP_TRANSLATION_THRESHOLD_METERS
                && innovation.headingDegrees() <= VisionConstants.VISION_JUMP_HEADING_THRESHOLD_DEGREES) {
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

        Logger.recordOutput("Vision/events/sequence", visionEventSequence);
        Logger.recordOutput("Vision/events/rejectCount", visionRejectEventCount);
        Logger.recordOutput("Vision/events/jumpCount", visionJumpEventCount);

        recordVisionEventDetails("Vision/events/last", eventType, timestampSeconds, measuredPose, odometryPose, innovation,
                tagCount, safeTagIds, ambiguity, avgDistance, linearStdDev, angularStdDev, cameraIndex);
        if (VisionConstants.ENABLE_VERBOSE_VISION_DIAGNOSTICS) {
            Logger.recordOutput("Vision/events/rejectCountByCamera", Arrays.copyOf(visionRejectEventCountByCamera, visionRejectEventCountByCamera.length));
            Logger.recordOutput("Vision/events/jumpCountByCamera", Arrays.copyOf(visionJumpEventCountByCamera, visionJumpEventCountByCamera.length));
            recordVisionEventDetails("Vision/events/frame", eventType, timestampSeconds, measuredPose, odometryPose, innovation,
                    tagCount, safeTagIds, ambiguity, avgDistance, linearStdDev, angularStdDev, cameraIndex);
        }

        if (!VisionConstants.ENABLE_VISION_EVENT_LOGS) {
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
        Logger.recordOutput("Vision/events/frame/type", "none");
        Logger.recordOutput("Vision/events/frame/timestampSeconds", Double.NaN);
        Logger.recordOutput("Vision/events/frame/cameraIndex", -1);
        Logger.recordOutput("Vision/events/frame/measuredPose", new Pose2d());
        Logger.recordOutput("Vision/events/frame/odometryPose", new Pose2d());
        Logger.recordOutput("Vision/events/frame/translationDeltaMeters", Double.NaN);
        Logger.recordOutput("Vision/events/frame/headingDeltaDegrees", Double.NaN);
        Logger.recordOutput("Vision/events/frame/tagCount", 0);
        Logger.recordOutput("Vision/events/frame/tagIds", new int[0]);
        Logger.recordOutput("Vision/events/frame/ambiguity", Double.NaN);
        Logger.recordOutput("Vision/events/frame/avgDistanceMeters", Double.NaN);
        Logger.recordOutput("Vision/events/frame/linearStdDev", Double.NaN);
        Logger.recordOutput("Vision/events/frame/angularStdDev", Double.NaN);
    }

    private void resetJumpDiagnosticsOutputs() {
        Logger.recordOutput("Vision/jump/consensusToOdometryTranslationMeters", Double.NaN);
        Logger.recordOutput("Vision/jump/consensusToOdometryHeadingDegrees", Double.NaN);
    }

    private record PoseDelta(double translationMeters, double headingDegrees) {
    }

    private record ObservationStdDevs(double linearStdDev, double angularStdDev) {
    }

    private record VisionCycleResult(
            PoseObservation bestRawObservation,
            VisionConsensus consensus) {
    }

    private record ConsensusObservation(
            PoseObservation observation,
            Pose2d measuredPose,
            ObservationStdDevs stdDevs) {
        private double weight() {
            double ambiguityFactor = 1.0 - Math.min(1.0, Math.max(0.0, observation.ambiguity()));
            double distanceFactor = 1.0 / Math.max(0.5, observation.averageTagDistance());
            return Math.max(1e-6, observation.tagCount() * (0.25 + ambiguityFactor) * distanceFactor);
        }
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
