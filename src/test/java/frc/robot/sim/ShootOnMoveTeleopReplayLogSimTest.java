package frc.robot.sim;

import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.datalog.DataLogReader;
import edu.wpi.first.util.datalog.DataLogRecord;
import edu.wpi.first.util.struct.StructBuffer;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;
import java.util.Locale;
import org.junit.jupiter.api.Test;

final class ShootOnMoveTeleopReplayLogSimTest {
    private static final Path LOG_PATH = Path.of("logs/akit_26-03-18_22-40-46.wpilog").toAbsolutePath();
    private static final double LOOP_PERIOD_SEC = FullFunctionalityHarness.LOOP_PERIOD_SEC;
    private static final double STATIONARY_STICK_THRESHOLD = 0.05;
    private static final double TRANSLATION_ONSET_STICK_THRESHOLD = 0.40;
    private static final double MIN_STATIONARY_SHOOT_TIME_SEC = 1.5;
    private static final double REPLAY_PRE_ROLL_SEC = 0.40;
    private static final double REPLAY_POST_ONSET_SEC = 2.20;
    private static final double POST_REPLAY_OBSERVATION_SEC = 2.00;
    private static final double MIN_POST_ONSET_TRANSLATION_METERS = 1.0;
    private static final int MIN_MOVING_DESCENT_SAMPLES = 2;

    @Test
    void replayLoggedStationaryThenMoveShootSequenceScoresWithoutMiss() throws Exception {
        Result result = runReplay();

        assertTrue(
                result.postOnsetTranslationMeters() >= MIN_POST_ONSET_TRANSLATION_METERS,
                String.format(
                        Locale.US,
                        "Expected replay to translate after logged joystick onset. moved=%.3fm min=%.3fm",
                        result.postOnsetTranslationMeters(),
                        MIN_POST_ONSET_TRANSLATION_METERS));
        assertTrue(
                result.sawMovingFeed(),
                "Expected logged replay to feed while translating after the stationary-to-moving transition.");
        assertTrue(
                result.movingDescentSamples() >= MIN_MOVING_DESCENT_SAMPLES,
                String.format(
                        Locale.US,
                        "Expected moving replay to produce descent samples. samples=%d min=%d",
                        result.movingDescentSamples(),
                        MIN_MOVING_DESCENT_SAMPLES));
        assertTrue(
                Double.isFinite(result.maxMovingDescentMissMeters())
                        && result.maxMovingDescentMissMeters() <= FuelSim.IDEAL_DESCENT_MISS_DISTANCE_METERS,
                String.format(
                        Locale.US,
                        "Expected replayed moving shots to stay within ideal miss distance. max=%.4fm (%.2fin) ideal=%.4fm (%.2fin)",
                        result.maxMovingDescentMissMeters(),
                        result.maxMovingDescentMissMeters() * 39.37007874,
                        FuelSim.IDEAL_DESCENT_MISS_DISTANCE_METERS,
                        FuelSim.IDEAL_DESCENT_MISS_DISTANCE_METERS * 39.37007874));
    }

    static Result runReplay() throws Exception {
        ReplayWindow replayWindow = ReplayWindow.fromLog(LOG_PATH);
        System.out.printf(
                Locale.US,
                "[ShootOnMoveReplay] steps=%d onset=%.3fs initialPose=(%.2f, %.2f, %.1fdeg)%n",
                replayWindow.steps().size(),
                replayWindow.translationOnsetSec(),
                replayWindow.initialPose().getX(),
                replayWindow.initialPose().getY(),
                replayWindow.initialPose().getRotation().getDegrees());

        try (FullFunctionalityHarness.Context context = new FullFunctionalityHarness.Context(false)) {
            context.setTeleopEnabled();
            context.container.teleopInit();
            context.clearControllerState();
            context.runCycles(220);
            context.drive.setPose(replayWindow.initialPose());
            context.clearControllerState();
            context.runCycles(8);

            FuelSim movingFuelSim = null;
            int movingDescentSamples = 0;
            double maxMovingDescentMissMeters = Double.NaN;
            Pose2d postOnsetStartPose = null;
            boolean sawMovingFeed = false;

            for (ReplayStep step : replayWindow.steps()) {
                context.driverControllerSim.setLeftX(step.leftX());
                context.driverControllerSim.setLeftY(step.leftY());
                context.driverControllerSim.setRightX(step.rightX());
                context.driverControllerSim.setRightTriggerAxis(step.rightTrigger());
                context.driverControllerSim.setRightBumperButton(step.rightBumper());
                context.runCycles(1);

                if (step.relativeTimeSec() >= replayWindow.translationOnsetSec() && movingFuelSim == null) {
                    movingFuelSim = new FuelSim();
                    postOnsetStartPose = context.drive.getPose();
                }
                if (movingFuelSim == null) {
                    continue;
                }

                movingFuelSim.update(
                        context.drive.getPose(),
                        context.shooter.getMeasuredAverageShooterRpm(),
                        context.shooter.getTargetHoodAngleRad(),
                        context.shooter.isKickerActive(),
                        LOOP_PERIOD_SEC);
                List<FuelSim.DescentCrossingSample> descentSamples = movingFuelSim.drainDescentCrossingSamples();
                movingDescentSamples += descentSamples.size();
                for (FuelSim.DescentCrossingSample descentSample : descentSamples) {
                    System.out.printf(
                            Locale.US,
                            "[ShootOnMoveReplay] moving descent t=%.3f miss=%.4fm along=%.4fm cross=%.4fm angle=%.2fdeg%n",
                            descentSample.timestampSec(),
                            descentSample.missDistanceMeters(),
                            descentSample.alongVelocityMissMeters(),
                            descentSample.crossVelocityMissMeters(),
                            descentSample.velocityToTargetAngleDeg());
                    if (!Double.isFinite(maxMovingDescentMissMeters)
                            || descentSample.missDistanceMeters() > maxMovingDescentMissMeters) {
                        maxMovingDescentMissMeters = descentSample.missDistanceMeters();
                    }
                }

                ChassisSpeeds speeds = context.drive.getMeasuredChassisSpeeds();
                double translationSpeedMps = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
                sawMovingFeed |= translationSpeedMps >= 0.25
                        && context.shooter.isKickerActive()
                        && context.transferInputs.appliedVolts > 1.0;
            }

            context.clearControllerState();
            int postReplayObservationCycles = (int) Math.ceil(POST_REPLAY_OBSERVATION_SEC / LOOP_PERIOD_SEC);
            for (int i = 0; i < postReplayObservationCycles && movingFuelSim != null; i++) {
                context.runCycles(1);
                movingFuelSim.update(
                        context.drive.getPose(),
                        context.shooter.getMeasuredAverageShooterRpm(),
                        context.shooter.getTargetHoodAngleRad(),
                        context.shooter.isKickerActive(),
                        LOOP_PERIOD_SEC);
                List<FuelSim.DescentCrossingSample> descentSamples = movingFuelSim.drainDescentCrossingSamples();
                movingDescentSamples += descentSamples.size();
                for (FuelSim.DescentCrossingSample descentSample : descentSamples) {
                    System.out.printf(
                            Locale.US,
                            "[ShootOnMoveReplay] moving descent t=%.3f miss=%.4fm along=%.4fm cross=%.4fm angle=%.2fdeg%n",
                            descentSample.timestampSec(),
                            descentSample.missDistanceMeters(),
                            descentSample.alongVelocityMissMeters(),
                            descentSample.crossVelocityMissMeters(),
                            descentSample.velocityToTargetAngleDeg());
                    if (!Double.isFinite(maxMovingDescentMissMeters)
                            || descentSample.missDistanceMeters() > maxMovingDescentMissMeters) {
                        maxMovingDescentMissMeters = descentSample.missDistanceMeters();
                    }
                }
            }

            Pose2d finalPose = context.drive.getPose();
            double postOnsetTranslationMeters = postOnsetStartPose != null
                    ? finalPose.getTranslation().getDistance(postOnsetStartPose.getTranslation())
                    : 0.0;
            System.out.printf(
                    Locale.US,
                    "[ShootOnMoveReplay] translated=%.3fm movingFeed=%s descentSamples=%d maxMiss=%.4fm%n",
                    postOnsetTranslationMeters,
                    sawMovingFeed,
                    movingDescentSamples,
                    maxMovingDescentMissMeters);
            return new Result(postOnsetTranslationMeters, sawMovingFeed, movingDescentSamples, maxMovingDescentMissMeters);
        }
    }

    record Result(
            double postOnsetTranslationMeters,
            boolean sawMovingFeed,
            int movingDescentSamples,
            double maxMovingDescentMissMeters) {}

    private record ReplayStep(
            double relativeTimeSec,
            float leftX,
            float leftY,
            float rightX,
            float rightTrigger,
            boolean rightBumper) {}

    private record ReplaySnapshot(
            double timeSec,
            float leftX,
            float leftY,
            float rightX,
            float rightTrigger,
            boolean rightBumper,
            String rightTriggerMode,
            Pose2d pose) {
        double translationStickMagnitude() {
            return Math.hypot(leftX, leftY);
        }
    }

    private record ReplayWindow(Pose2d initialPose, double translationOnsetSec, List<ReplayStep> steps) {
        static ReplayWindow fromLog(Path logPath) {
            List<ReplaySnapshot> snapshots = readSnapshots(logPath);
            ShotSequenceWindow sequenceWindow = findStationaryThenMoveScoreWindow(snapshots);
            List<ReplayStep> steps = resampleSteps(snapshots, sequenceWindow.startSec(), sequenceWindow.endSec());
            return new ReplayWindow(sequenceWindow.startPose(), sequenceWindow.translationOnsetSec() - sequenceWindow.startSec(), steps);
        }

        private static List<ReplaySnapshot> readSnapshots(Path logPath) {
            DataLogReader reader;
            try {
                reader = new DataLogReader(logPath.toString());
            } catch (java.io.IOException exception) {
                throw new IllegalStateException("Failed to open log: " + logPath, exception);
            }
            StructBuffer<Pose2d> poseBuffer = StructBuffer.create(Pose2d.struct);

            int axisEntry = -1;
            int buttonEntry = -1;
            int poseEntry = -1;
            int rightTriggerModeEntry = -1;

            for (DataLogRecord record : recordsUntilFailure(reader)) {
                if (!record.isStart()) {
                    continue;
                }
                DataLogRecord.StartRecordData start;
                try {
                    start = record.getStartData();
                } catch (IllegalArgumentException ignored) {
                    continue;
                }
                switch (start.name) {
                    case "/DriverStation/Joystick0/AxisValues" -> axisEntry = start.entry;
                    case "/DriverStation/Joystick0/ButtonValues" -> buttonEntry = start.entry;
                    case "/RealOutputs/Odometry/Robot" -> poseEntry = start.entry;
                    case "/RealOutputs/Shooting/RightTriggerMode" -> rightTriggerModeEntry = start.entry;
                    default -> {}
                }
            }

            float[] axes = new float[0];
            long buttons = 0L;
            Pose2d pose = new Pose2d();
            String rightTriggerMode = "";
            List<ReplaySnapshot> snapshots = new ArrayList<>();
            DataLogReader replayReader;
            try {
                replayReader = new DataLogReader(logPath.toString());
            } catch (java.io.IOException exception) {
                throw new IllegalStateException("Failed to reopen log: " + logPath, exception);
            }
            for (DataLogRecord record : recordsUntilFailure(replayReader)) {
                if (record.isStart() || record.isControl()) {
                    continue;
                }
                int entry = record.getEntry();
                if (entry == axisEntry) {
                    axes = record.getFloatArray();
                } else if (entry == buttonEntry) {
                    buttons = record.getInteger();
                } else if (entry == poseEntry) {
                    pose = poseBuffer.read(record.getRaw());
                } else if (entry == rightTriggerModeEntry) {
                    rightTriggerMode = record.getString();
                } else {
                    continue;
                }

                float leftX = axes.length > 0 ? axes[0] : 0.0f;
                float leftY = axes.length > 1 ? axes[1] : 0.0f;
                float rightTrigger = axes.length > 3 ? axes[3] : 0.0f;
                float rightX = axes.length > 4 ? axes[4] : 0.0f;
                boolean rightBumper = (buttons & (1L << 5)) != 0;
                snapshots.add(new ReplaySnapshot(
                        record.getTimestamp() / 1_000_000.0,
                        leftX,
                        leftY,
                        rightX,
                        rightTrigger,
                        rightBumper,
                        rightTriggerMode,
                        pose));
            }
            return snapshots;
        }

        private static ShotSequenceWindow findStationaryThenMoveScoreWindow(List<ReplaySnapshot> snapshots) {
            ShotSequenceWindow bestWindow = null;
            double bestPostOnsetAverageStick = Double.NEGATIVE_INFINITY;
            for (int i = 1; i < snapshots.size(); i++) {
                ReplaySnapshot previous = snapshots.get(i - 1);
                ReplaySnapshot current = snapshots.get(i);
                if (previous.rightTrigger() > 0.5f || current.rightTrigger() <= 0.5f || !"SHOOT".equals(current.rightTriggerMode())) {
                    continue;
                }

                int stationaryIndex = -1;
                for (int j = i; j < snapshots.size(); j++) {
                    ReplaySnapshot candidate = snapshots.get(j);
                    if (candidate.rightTrigger() <= 0.5f) {
                        break;
                    }
                    if (candidate.translationStickMagnitude() <= STATIONARY_STICK_THRESHOLD) {
                        stationaryIndex = j;
                        break;
                    }
                }
                if (stationaryIndex < 0) {
                    continue;
                }

                int onsetIndex = -1;
                for (int j = stationaryIndex; j < snapshots.size(); j++) {
                    ReplaySnapshot candidate = snapshots.get(j);
                    if (candidate.rightTrigger() <= 0.5f) {
                        break;
                    }
                    if (candidate.timeSec() - snapshots.get(stationaryIndex).timeSec() < MIN_STATIONARY_SHOOT_TIME_SEC) {
                        continue;
                    }
                    if (candidate.translationStickMagnitude() >= TRANSLATION_ONSET_STICK_THRESHOLD) {
                        onsetIndex = j;
                        break;
                    }
                }
                if (onsetIndex < 0) {
                    continue;
                }

                double startSec = Math.max(0.0, current.timeSec() - REPLAY_PRE_ROLL_SEC);
                double endSec = snapshots.get(onsetIndex).timeSec() + REPLAY_POST_ONSET_SEC;
                Pose2d startPose = samplePoseAtOrBefore(snapshots, startSec);
                double postOnsetAverageStick = averageStickMagnitude(
                        snapshots,
                        snapshots.get(onsetIndex).timeSec(),
                        snapshots.get(onsetIndex).timeSec() + 0.75);
                boolean anyRightBumperAfterOnset = anyRightBumper(
                        snapshots,
                        snapshots.get(onsetIndex).timeSec(),
                        snapshots.get(onsetIndex).timeSec() + 0.75);
                if (!anyRightBumperAfterOnset && postOnsetAverageStick > bestPostOnsetAverageStick) {
                    bestPostOnsetAverageStick = postOnsetAverageStick;
                    bestWindow = new ShotSequenceWindow(startSec, endSec, snapshots.get(onsetIndex).timeSec(), startPose);
                }
            }
            if (bestWindow != null) {
                return bestWindow;
            }
            throw new IllegalStateException("Could not find a stationary-then-move SHOOT replay window in " + LOG_PATH);
        }

        private static Pose2d samplePoseAtOrBefore(List<ReplaySnapshot> snapshots, double timeSec) {
            Pose2d pose = snapshots.get(0).pose();
            for (ReplaySnapshot snapshot : snapshots) {
                if (snapshot.timeSec() > timeSec) {
                    break;
                }
                pose = snapshot.pose();
            }
            return pose;
        }

        private static List<ReplayStep> resampleSteps(List<ReplaySnapshot> snapshots, double startSec, double endSec) {
            List<ReplayStep> steps = new ArrayList<>();
            int snapshotIndex = 0;
            ReplaySnapshot current = snapshots.get(0);
            for (double t = startSec; t <= endSec + 1e-9; t += LOOP_PERIOD_SEC) {
                while (snapshotIndex + 1 < snapshots.size() && snapshots.get(snapshotIndex + 1).timeSec() <= t) {
                    snapshotIndex++;
                    current = snapshots.get(snapshotIndex);
                }
                steps.add(new ReplayStep(
                        t - startSec,
                        current.leftX(),
                        current.leftY(),
                        current.rightX(),
                        current.rightTrigger(),
                        current.rightBumper()));
            }
            return steps;
        }

        private static double averageStickMagnitude(List<ReplaySnapshot> snapshots, double startSec, double endSec) {
            double sum = 0.0;
            int count = 0;
            for (ReplaySnapshot snapshot : snapshots) {
                if (snapshot.timeSec() < startSec || snapshot.timeSec() > endSec || snapshot.rightTrigger() <= 0.5f) {
                    continue;
                }
                sum += snapshot.translationStickMagnitude();
                count++;
            }
            return count > 0 ? sum / count : Double.NEGATIVE_INFINITY;
        }

        private static boolean anyRightBumper(List<ReplaySnapshot> snapshots, double startSec, double endSec) {
            for (ReplaySnapshot snapshot : snapshots) {
                if (snapshot.timeSec() < startSec || snapshot.timeSec() > endSec) {
                    continue;
                }
                if (snapshot.rightBumper()) {
                    return true;
                }
            }
            return false;
        }
    }

    private record ShotSequenceWindow(double startSec, double endSec, double translationOnsetSec, Pose2d startPose) {}

    private static List<DataLogRecord> recordsUntilFailure(DataLogReader reader) {
        List<DataLogRecord> records = new ArrayList<>();
        Iterator<DataLogRecord> iterator = reader.iterator();
        while (true) {
            try {
                if (!iterator.hasNext()) {
                    break;
                }
                records.add(iterator.next());
            } catch (IllegalArgumentException exception) {
                break;
            }
        }
        return records;
    }
}
