package frc.robot.sim;

import static edu.wpi.first.math.MathUtil.angleModulus;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.datalog.DataLogReader;
import edu.wpi.first.util.datalog.DataLogRecord;
import edu.wpi.first.util.struct.StructBuffer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.coordination.shooting.ShootingTeleopController;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;
import java.util.Locale;
import org.junit.jupiter.api.Test;

class LoggedAimTuningSweepSimTest {
    private static final Path LOG_PATH = Path.of("logs/akit_26-03-18_23-28-36.wpilog").toAbsolutePath();
    private static final double PRE_ROLL_SEC = 0.60;
    private static final double GOOD_ERROR_DEG = 4.0;

    @Test
    void sweepShotYawTuningsAgainstLoggedAimWindows() throws Exception {
        List<Window> windows = List.of(
                new Window(22.0, 23.30),
                new Window(30.30, 31.70),
                new Window(38.60, 40.20));
        List<Config> configs = List.of(
                new Config("baseline", 8.0, 0.5, 3, 540.0),
                new Config("kd0.20_only", 8.0, 0.20, 3, 540.0),
                new Config("taps5_only", 8.0, 0.5, 5, 540.0),
                new Config("rate360_only", 8.0, 0.5, 3, 360.0),
                new Config("kd0.20_taps5_rate360", 8.0, 0.20, 5, 360.0),
                new Config("kd0.15_taps7_rate300", 8.0, 0.15, 7, 300.0),
                new Config("kd0.10_taps7_rate300", 8.0, 0.10, 7, 300.0),
                new Config("kd0.00_taps7_rate300", 8.0, 0.00, 7, 300.0),
                new Config("kp6.5_kd0.20_taps5_rate360", 6.5, 0.20, 5, 360.0));

        for (Config config : configs) {
            Aggregate aggregate = new Aggregate();
            for (Window window : windows) {
                Result result = runWindow(config, window);
                aggregate.add(result);
                System.out.printf(
                        Locale.US,
                        "%s window %.2f-%.2f meanErr=%.2f maxErr=%.2f under4=%.1f%% settle=%.3fs\n",
                        config.name(),
                        window.startSec(),
                        window.endSec(),
                        result.meanAbsErrorDeg(),
                        result.maxAbsErrorDeg(),
                        100.0 * result.goodFraction(),
                        result.timeToGoodSec());
            }
            System.out.printf(
                    Locale.US,
                    "%s aggregate meanErr=%.2f maxErr=%.2f under4=%.1f%% avgSettle=%.3fs\n\n",
                    config.name(),
                    aggregate.meanAbsErrorDeg(),
                    aggregate.maxAbsErrorDeg(),
                    100.0 * aggregate.goodFraction(),
                    aggregate.avgSettleSec());
        }
    }

    private static Result runWindow(Config config, Window window) throws Exception {
        setConfig(config);
        ReplayWindow replay = ReplayWindow.fromLog(LOG_PATH, window.startSec() - PRE_ROLL_SEC, window.endSec());
        try (FullFunctionalityHarness.Context context = new FullFunctionalityHarness.Context(false)) {
            context.setTeleopEnabled();
            context.container.teleopInit();
            context.clearControllerState();
            context.runCycles(220);
            context.drive.setPose(replay.initialPose());
            context.clearControllerState();
            context.runCycles(8);

            ShootingTeleopController controller =
                    FullFunctionalityHarness.getPrivateField(context.container, "shootingTeleopController", ShootingTeleopController.class);
            ReplayState replayState = new ReplayState();
            ShootingTeleopController.AimingContext aimingContext = controller.createAimingContext();
            Command command = controller.createSelectedAimCommand(
                    () -> replayState.leftY,
                    () -> replayState.leftX,
                    () -> replayState.rightX,
                    aimingContext.hubDistanceSupplier(),
                    aimingContext.hubHeadingSupplier());
            command.schedule();

            double sumAbsErrorDeg = 0.0;
            int samples = 0;
            double maxAbsErrorDeg = 0.0;
            int goodSamples = 0;
            double timeToGoodSec = Double.NaN;

            for (ReplayStep step : replay.steps()) {
                replayState.leftX = step.leftX();
                replayState.leftY = step.leftY();
                replayState.rightX = step.rightX();
                context.runCycles(1);

                if (step.relativeTimeSec() < PRE_ROLL_SEC) {
                    continue;
                }
                Rotation2d targetHeading = aimingContext.hubHeadingSupplier().get();
                if (targetHeading == null) {
                    continue;
                }
                Rotation2d desiredRobotHeading = targetHeading.plus(Rotation2d.kPi);
                double errorDeg = Math.toDegrees(angleModulus(
                        desiredRobotHeading.minus(context.drive.getPose().getRotation()).getRadians()));
                double absErrorDeg = Math.abs(errorDeg);
                sumAbsErrorDeg += absErrorDeg;
                samples++;
                maxAbsErrorDeg = Math.max(maxAbsErrorDeg, absErrorDeg);
                if (absErrorDeg <= GOOD_ERROR_DEG) {
                    goodSamples++;
                    if (!Double.isFinite(timeToGoodSec)) {
                        timeToGoodSec = step.relativeTimeSec() - PRE_ROLL_SEC;
                    }
                }
            }
            command.cancel();
            context.runCycles(2);
            return new Result(
                    samples > 0 ? sumAbsErrorDeg / samples : Double.NaN,
                    maxAbsErrorDeg,
                    samples > 0 ? (double) goodSamples / samples : Double.NaN,
                    timeToGoodSec);
        }
    }

    private static void setConfig(Config config) {
        SmartDashboard.putNumber("ShotYaw/kP", config.kP());
        SmartDashboard.putNumber("ShotYaw/kD", config.kD());
        SmartDashboard.putNumber("ShotSolution/HeadingRateFilterTaps", config.headingRateFilterTaps());
        SmartDashboard.putNumber("ShotSolution/MaxHeadingRateDegPerSec", config.maxHeadingRateDegPerSec());
    }

    private record Config(String name, double kP, double kD, int headingRateFilterTaps, double maxHeadingRateDegPerSec) {}

    private record Window(double startSec, double endSec) {}

    private record Result(double meanAbsErrorDeg, double maxAbsErrorDeg, double goodFraction, double timeToGoodSec) {}

    private static final class Aggregate {
        double sumMeanErr = 0.0;
        int count = 0;
        double maxErr = 0.0;
        double sumGoodFraction = 0.0;
        double sumSettle = 0.0;
        int settleCount = 0;

        void add(Result result) {
            sumMeanErr += result.meanAbsErrorDeg();
            sumGoodFraction += result.goodFraction();
            maxErr = Math.max(maxErr, result.maxAbsErrorDeg());
            count++;
            if (Double.isFinite(result.timeToGoodSec())) {
                sumSettle += result.timeToGoodSec();
                settleCount++;
            }
        }

        double meanAbsErrorDeg() { return count > 0 ? sumMeanErr / count : Double.NaN; }
        double maxAbsErrorDeg() { return maxErr; }
        double goodFraction() { return count > 0 ? sumGoodFraction / count : Double.NaN; }
        double avgSettleSec() { return settleCount > 0 ? sumSettle / settleCount : Double.NaN; }
    }

    private static final class ReplayState {
        double leftX;
        double leftY;
        double rightX;
    }

    private record ReplayStep(double relativeTimeSec, float leftX, float leftY, float rightX) {}

    private record ReplaySnapshot(double timeSec, float leftX, float leftY, float rightX, Pose2d pose) {}

    private record ReplayWindow(Pose2d initialPose, List<ReplayStep> steps) {
        static ReplayWindow fromLog(Path logPath, double startSec, double endSec) throws Exception {
            List<ReplaySnapshot> snapshots = readSnapshots(logPath);
            Pose2d initialPose = samplePoseAtOrBefore(snapshots, startSec);
            List<ReplayStep> steps = resampleSteps(snapshots, startSec, endSec);
            return new ReplayWindow(initialPose, steps);
        }

        private static List<ReplaySnapshot> readSnapshots(Path logPath) throws Exception {
            DataLogReader reader = new DataLogReader(logPath.toString());
            StructBuffer<Pose2d> poseBuffer = StructBuffer.create(Pose2d.struct);
            int axisEntry=-1, poseEntry=-1;
            for (DataLogRecord record : recordsUntilFailure(reader)) {
                if (!record.isStart()) continue;
                try {
                    var start = record.getStartData();
                    switch (start.name) {
                        case "/DriverStation/Joystick0/AxisValues" -> axisEntry = start.entry;
                        case "/RealOutputs/Odometry/Robot" -> poseEntry = start.entry;
                        default -> {}
                    }
                } catch (IllegalArgumentException ignored) {}
            }
            float[] axes = new float[0];
            Pose2d pose = new Pose2d();
            List<ReplaySnapshot> snapshots = new ArrayList<>();
            for (DataLogRecord record : recordsUntilFailure(new DataLogReader(logPath.toString()))) {
                if (record.isStart() || record.isControl()) continue;
                int entry = record.getEntry();
                if (entry == axisEntry) {
                    axes = record.getFloatArray();
                } else if (entry == poseEntry) {
                    pose = poseBuffer.read(record.getRaw());
                } else {
                    continue;
                }
                float leftX = axes.length > 0 ? axes[0] : 0.0f;
                float leftY = axes.length > 1 ? axes[1] : 0.0f;
                float rightX = axes.length > 4 ? axes[4] : 0.0f;
                snapshots.add(new ReplaySnapshot(record.getTimestamp() / 1_000_000.0, leftX, leftY, rightX, pose));
            }
            return snapshots;
        }

        private static Pose2d samplePoseAtOrBefore(List<ReplaySnapshot> snapshots, double timeSec) {
            Pose2d pose = snapshots.get(0).pose();
            for (ReplaySnapshot snapshot : snapshots) {
                if (snapshot.timeSec() > timeSec) break;
                pose = snapshot.pose();
            }
            return pose;
        }

        private static List<ReplayStep> resampleSteps(List<ReplaySnapshot> snapshots, double startSec, double endSec) {
            List<ReplayStep> steps = new ArrayList<>();
            int index = 0;
            ReplaySnapshot current = snapshots.get(0);
            for (double t = startSec; t <= endSec + 1e-9; t += FullFunctionalityHarness.LOOP_PERIOD_SEC) {
                while (index + 1 < snapshots.size() && snapshots.get(index + 1).timeSec() <= t) {
                    index++;
                    current = snapshots.get(index);
                }
                steps.add(new ReplayStep(t - startSec, current.leftX(), current.leftY(), current.rightX()));
            }
            return steps;
        }
    }

    private static List<DataLogRecord> recordsUntilFailure(DataLogReader reader) {
        List<DataLogRecord> records = new ArrayList<>();
        Iterator<DataLogRecord> iterator = reader.iterator();
        while (true) {
            try {
                if (!iterator.hasNext()) break;
                records.add(iterator.next());
            } catch (IllegalArgumentException exception) {
                break;
            }
        }
        return records;
    }
}
