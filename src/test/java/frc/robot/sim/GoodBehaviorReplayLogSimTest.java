package frc.robot.sim;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.util.datalog.DataLogReader;
import edu.wpi.first.util.datalog.DataLogRecord;
import edu.wpi.first.util.struct.StructBuffer;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;
import java.util.Locale;
import org.junit.jupiter.api.Test;

class GoodBehaviorReplayLogSimTest {
    private static final Path LOG_PATH = Path.of("logs/akit_26-03-19_00-08-50.wpilog").toAbsolutePath();
    private static final double LOOP_PERIOD_SEC = FullFunctionalityHarness.LOOP_PERIOD_SEC;

    @Test
    void replaySelectedGoodMovingWindows() throws Exception {
        runWindow("goodA", 160.80, 166.80);
        runWindow("goodB", 168.40, 176.90);
    }

    private static void runWindow(String name, double startSec, double endSec) throws Exception {
        ReplayWindow replay = ReplayWindow.fromLog(LOG_PATH, startSec, endSec);
        System.out.printf(Locale.US,
                "\n[%s] steps=%d initialPose=(%.2f,%.2f,%.1fdeg) start=%.3f end=%.3f%n",
                name,
                replay.steps().size(),
                replay.initialPose().getX(),
                replay.initialPose().getY(),
                replay.initialPose().getRotation().getDegrees(),
                startSec,
                endSec);
        try (FullFunctionalityHarness.Context context = new FullFunctionalityHarness.Context(false)) {
            context.setTeleopEnabled();
            context.container.teleopInit();
            context.clearControllerState();
            context.runCycles(220);
            context.drive.setPose(replay.initialPose());
            context.clearControllerState();
            context.runCycles(8);

            FuelSim fuelSim = new FuelSim();
            int descentSamples = 0;
            double maxMiss = Double.NaN;
            int movingFeedSamples = 0;
            for (ReplayStep step : replay.steps()) {
                context.driverControllerSim.setLeftX(step.leftX());
                context.driverControllerSim.setLeftY(step.leftY());
                context.driverControllerSim.setRightX(step.rightX());
                context.driverControllerSim.setRightTriggerAxis(step.rightTrigger());
                context.driverControllerSim.setRightBumperButton(step.rightBumper());
                context.runCycles(1);

                fuelSim.update(
                        context.drive.getPose(),
                        context.shooter.getMeasuredAverageShooterRpm(),
                        context.shooter.getTargetHoodAngleRad(),
                        context.shooter.isKickerActive(),
                        LOOP_PERIOD_SEC);
                var samples = fuelSim.drainDescentCrossingSamples();
                descentSamples += samples.size();
                for (var sample : samples) {
                    if (!Double.isFinite(maxMiss) || sample.missDistanceMeters() > maxMiss) {
                        maxMiss = sample.missDistanceMeters();
                    }
                    System.out.printf(Locale.US,
                            "[%s] descent t=%.3f miss=%.4fm along=%.4fm cross=%.4fm angle=%.2fdeg%n",
                            name,
                            sample.timestampSec(),
                            sample.missDistanceMeters(),
                            sample.alongVelocityMissMeters(),
                            sample.crossVelocityMissMeters(),
                            sample.velocityToTargetAngleDeg());
                }
                var speeds = context.drive.getMeasuredChassisSpeeds();
                double speed = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
                if (speed >= 0.25 && context.shooter.isKickerActive() && context.transferInputs.appliedVolts > 1.0) {
                    movingFeedSamples++;
                }
            }
            context.clearControllerState();
            context.runCycles(40);
            fuelSim.update(
                    context.drive.getPose(),
                    context.shooter.getMeasuredAverageShooterRpm(),
                    context.shooter.getTargetHoodAngleRad(),
                    context.shooter.isKickerActive(),
                    LOOP_PERIOD_SEC);
            var tail = fuelSim.drainDescentCrossingSamples();
            descentSamples += tail.size();
            for (var sample : tail) {
                if (!Double.isFinite(maxMiss) || sample.missDistanceMeters() > maxMiss) {
                    maxMiss = sample.missDistanceMeters();
                }
                System.out.printf(Locale.US,
                        "[%s] descent t=%.3f miss=%.4fm along=%.4fm cross=%.4fm angle=%.2fdeg%n",
                        name,
                        sample.timestampSec(),
                        sample.missDistanceMeters(),
                        sample.alongVelocityMissMeters(),
                        sample.crossVelocityMissMeters(),
                        sample.velocityToTargetAngleDeg());
            }

            System.out.printf(Locale.US,
                    "[%s] movingFeedSamples=%d descentSamples=%d maxMiss=%.4fm%n",
                    name,
                    movingFeedSamples,
                    descentSamples,
                    maxMiss);
        }
    }

    private record ReplayStep(double relativeTimeSec, float leftX, float leftY, float rightX, float rightTrigger, boolean rightBumper) {}

    private record ReplaySnapshot(double timeSec, float leftX, float leftY, float rightX, float rightTrigger, boolean rightBumper, Pose2d pose) {}

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
            int axisEntry=-1, buttonEntry=-1, poseEntry=-1;
            for (DataLogRecord record : recordsUntilFailure(reader)) {
                if (!record.isStart()) continue;
                try {
                    var start = record.getStartData();
                    switch (start.name) {
                        case "/DriverStation/Joystick0/AxisValues" -> axisEntry = start.entry;
                        case "/DriverStation/Joystick0/ButtonValues" -> buttonEntry = start.entry;
                        case "/RealOutputs/Odometry/Robot" -> poseEntry = start.entry;
                        default -> {}
                    }
                } catch (IllegalArgumentException ignored) {}
            }
            float[] axes = new float[0];
            long buttons = 0L;
            Pose2d pose = new Pose2d();
            List<ReplaySnapshot> snapshots = new ArrayList<>();
            for (DataLogRecord record : recordsUntilFailure(new DataLogReader(logPath.toString()))) {
                if (record.isStart() || record.isControl()) continue;
                int entry = record.getEntry();
                if (entry == axisEntry) axes = record.getFloatArray();
                else if (entry == buttonEntry) buttons = record.getInteger();
                else if (entry == poseEntry) pose = poseBuffer.read(record.getRaw());
                else continue;
                float leftX = axes.length > 0 ? axes[0] : 0.0f;
                float leftY = axes.length > 1 ? axes[1] : 0.0f;
                float rightTrigger = axes.length > 3 ? axes[3] : 0.0f;
                float rightX = axes.length > 4 ? axes[4] : 0.0f;
                boolean rightBumper = (buttons & (1L << 5)) != 0;
                snapshots.add(new ReplaySnapshot(record.getTimestamp() / 1_000_000.0, leftX, leftY, rightX, rightTrigger, rightBumper, pose));
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
            for (double t = startSec; t <= endSec + 1e-9; t += LOOP_PERIOD_SEC) {
                while (index + 1 < snapshots.size() && snapshots.get(index + 1).timeSec() <= t) {
                    index++;
                    current = snapshots.get(index);
                }
                steps.add(new ReplayStep(t - startSec, current.leftX(), current.leftY(), current.rightX(), current.rightTrigger(), current.rightBumper()));
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
