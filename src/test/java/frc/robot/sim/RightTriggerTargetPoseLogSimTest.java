package frc.robot.sim;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.datalog.DataLogReader;
import edu.wpi.first.util.datalog.DataLogRecord;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.util.FieldConstants;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.Iterator;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

final class RightTriggerTargetPoseLogSimTest {
    private static final double POSE_EPSILON_METERS = 1e-6;

    private Path outputDir;

    @BeforeEach
    void setUp() throws IOException {
        HAL.initialize(500, 0);
        outputDir = Path.of("build/right-trigger-target-log-sim").toAbsolutePath();
        Files.createDirectories(outputDir);
    }

    @Test
    void rightTriggerTargetPoseIsPublishedToAdvantageKit() throws IOException {
        Pose2d expectedPassTarget;
        try (FullFunctionalityHarness.Context context = new FullFunctionalityHarness.Context(false)) {
            context.setTeleopEnabled();
            context.container.teleopInit();
            context.runCycles(20);

            Pose2d outsideBlueZonePose = new Pose2d(
                    FieldConstants.getAllianceZoneBoundaryX() + 0.8,
                    FieldConstants.getHubTargetTranslation().getY() - 0.7,
                    Rotation2d.kZero);
            context.drive.setPose(outsideBlueZonePose);
            expectedPassTarget = FieldConstants.getPassTargetPose(outsideBlueZonePose);
            context.runCycles(6);

            Logger.end();
            Logger.AdvancedHooks.disableRobotBaseCheck();
            Logger.disableConsoleCapture();
            Logger.recordMetadata("TestName", getClass().getSimpleName());
            Logger.addDataReceiver(new WPILOGWriter(outputDir.toString()));
            Logger.start();

            for (int i = 0; i < 20; i++) {
                runLoggedHarnessCycle(context);
            }

            Logger.end();
        }

        TargetPoseLogSample sample = extractLatestTargetPose(findLatestWpiLog(outputDir));
        assertEquals("PASS", sample.mode());
        assertEquals(expectedPassTarget.getX(), sample.targetX(), POSE_EPSILON_METERS);
        assertEquals(expectedPassTarget.getY(), sample.targetY(), POSE_EPSILON_METERS);
    }

    private static void runLoggedHarnessCycle(FullFunctionalityHarness.Context context) {
        SimHooks.stepTiming(FullFunctionalityHarness.LOOP_PERIOD_SEC);
        DriverStationSim.notifyNewData();
        long periodicBeforeStartUs = RobotController.getFPGATime();
        Logger.AdvancedHooks.invokePeriodicBeforeUser();
        long periodicBeforeEndUs = RobotController.getFPGATime();

        CommandScheduler.getInstance().run();
        context.container.robotPeriodic();
        context.container.simulationPeriodic();

        long userCodeEndUs = RobotController.getFPGATime();
        Logger.AdvancedHooks.invokePeriodicAfterUser(
                userCodeEndUs - periodicBeforeEndUs,
                periodicBeforeEndUs - periodicBeforeStartUs);
    }

    private static Path findLatestWpiLog(Path directory) throws IOException {
        try (var paths = Files.list(directory)) {
            return paths
                    .filter(path -> path.getFileName().toString().endsWith(".wpilog"))
                    .max((a, b) -> Long.compare(a.toFile().lastModified(), b.toFile().lastModified()))
                    .orElseThrow(() -> new IllegalStateException("No .wpilog files found in " + directory));
        }
    }

    private static TargetPoseLogSample extractLatestTargetPose(Path wpilog) throws IOException {
        DataLogReader reader = new DataLogReader(wpilog.toString());
        assertTrue(reader.isValid(), "Invalid WPILOG file: " + wpilog);

        int targetXEntry = -1;
        int targetYEntry = -1;
        int modeEntry = -1;
        Iterator<DataLogRecord> startIterator = reader.iterator();
        while (startIterator.hasNext()) {
            DataLogRecord record = startIterator.next();
            if (!record.isStart()) {
                continue;
            }
            var start = record.getStartData();
            if (start.name.equals("/AdvantageKit/RealOutputs/Shooting/TargetPoseX")
                    || start.name.equals("/AdvantageKit/Shooting/TargetPoseX")
                    || start.name.equals("/RealOutputs/Shooting/TargetPoseX")) {
                targetXEntry = start.entry;
            } else if (start.name.equals("/AdvantageKit/RealOutputs/Shooting/TargetPoseY")
                    || start.name.equals("/AdvantageKit/Shooting/TargetPoseY")
                    || start.name.equals("/RealOutputs/Shooting/TargetPoseY")) {
                targetYEntry = start.entry;
            } else if (start.name.equals("/AdvantageKit/RealOutputs/Shooting/RightTriggerMode")
                    || start.name.equals("/AdvantageKit/Shooting/RightTriggerMode")
                    || start.name.equals("/RealOutputs/Shooting/RightTriggerMode")) {
                modeEntry = start.entry;
            }
        }

        double latestTargetX = Double.NaN;
        double latestTargetY = Double.NaN;
        String latestMode = "";
        Iterator<DataLogRecord> dataIterator = reader.iterator();
        while (dataIterator.hasNext()) {
            DataLogRecord record = dataIterator.next();
            if (record.isStart() || record.isControl()) {
                continue;
            }
            if (record.getEntry() == targetXEntry) {
                latestTargetX = record.getDouble();
            } else if (record.getEntry() == targetYEntry) {
                latestTargetY = record.getDouble();
            } else if (record.getEntry() == modeEntry) {
                latestMode = record.getString();
            }
        }

        return new TargetPoseLogSample(latestTargetX, latestTargetY, latestMode);
    }

    private record TargetPoseLogSample(double targetX, double targetY, String mode) {}
}
