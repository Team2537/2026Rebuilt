package frc.robot.autos;

import static org.junit.jupiter.api.Assertions.assertTrue;

import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.util.datalog.DataLogReader;
import edu.wpi.first.util.datalog.DataLogRecord;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.coordination.shooting.ShootCoordinator;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIOSim;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.transfer.Transfer;
import frc.robot.subsystems.transfer.TransferIOSim;
import frc.robot.subsystems.vision.Vision;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.stream.Collectors;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

class BragAutoVisionSimTest {
    private static final String AUTO_NAME = "brag";
    private static final String TIMING_KEY = "AutoSim/SimTimeSec";
    private static final double LOOP_PERIOD_SEC = 0.02;
    private static final double MAX_SIM_TIME_SEC = 45.0;

    private Path outputDir;
    private boolean loggerClosed = false;

    @BeforeEach
    void setUp() throws IOException {
        HAL.initialize(500, 0);
        SimHooks.pauseTiming();
        DriverStationSim.setDsAttached(true);
        DriverStationSim.setAutonomous(true);
        DriverStationSim.setTest(false);
        DriverStationSim.setEStop(false);
        DriverStationSim.setEnabled(true);
        DriverStationSim.notifyNewData();
        CommandScheduler.getInstance().cancelAll();

        outputDir = resolveOutputDir();
        Files.createDirectories(outputDir);

        Logger.end();
        Logger.AdvancedHooks.disableRobotBaseCheck();
        Logger.disableConsoleCapture();
        Logger.recordMetadata("TestName", getClass().getSimpleName());
        Logger.recordMetadata("AutoName", AUTO_NAME);
        Logger.recordMetadata("OutputDir", outputDir.toString());
        Logger.addDataReceiver(new WPILOGWriter(outputDir.toString()));
        Logger.start();
        loggerClosed = false;
    }

    @AfterEach
    void tearDown() {
        CommandScheduler.getInstance().cancelAll();
        DriverStationSim.setEnabled(false);
        DriverStationSim.notifyNewData();
        if (!loggerClosed) {
            Logger.end();
            loggerClosed = true;
        }
        SimHooks.resumeTiming();
    }

    @Test
    void bragAutoLogsVisionJumpEventsToWpilog() throws IOException {
        GyroIOSim gyro = new GyroIOSim(Drive.getModuleTranslations());
        Drive drive = new Drive(
                gyro,
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));
        gyro.setModulePositionsSupplier(drive::getModulePositionsForSim);

        Shooter shooter = new Shooter(new ShooterIOSim());
        Transfer transfer = new Transfer(new TransferIOSim());
        Intake intake = new Intake(new IntakeIOSim());
        ShootCoordinator shootCoordinator = new ShootCoordinator(shooter, transfer);
        frc.robot.RobotState.initialize(drive);
        @SuppressWarnings("unused")
        Vision vision = new Vision(frc.robot.RobotState.getInstance());

        AutoNamedCommands.registerAll(drive, shooter, transfer, intake, shootCoordinator);
        assertTrue(
                AutoRoutines.create(drive).stream()
                        .map(AutoRoutines.AutoRoutine::name)
                        .collect(Collectors.toSet())
                        .contains("pp/" + AUTO_NAME),
                "Auto selector routines did not include PathPlanner auto: " + AUTO_NAME);

        Command auto = new PathPlannerAuto(AUTO_NAME).withName("TestAuto_" + AUTO_NAME);
        CommandScheduler.getInstance().schedule(auto);

        Pose2d previousPose = drive.getPose();
        double distanceTraveledMeters = 0.0;
        boolean finished = false;
        double elapsedSec = 0.0;

        int maxIterations = (int) Math.ceil(MAX_SIM_TIME_SEC / LOOP_PERIOD_SEC);
        for (int i = 0; i < maxIterations; i++) {
            SimHooks.stepTiming(LOOP_PERIOD_SEC);
            DriverStationSim.notifyNewData();
            runLoggedSchedulerCycle();

            double simTimeSec = (i + 1) * LOOP_PERIOD_SEC;
            Pose2d currentPose = drive.getPose();
            distanceTraveledMeters +=
                    currentPose.getTranslation().getDistance(previousPose.getTranslation());
            previousPose = currentPose;

            Logger.recordOutput(TIMING_KEY, simTimeSec);
            Logger.recordOutput("AutoSim/DistanceTraveledMeters", distanceTraveledMeters);
            Logger.recordOutput("AutoSim/AutoScheduled", CommandScheduler.getInstance().isScheduled(auto));

            if (!CommandScheduler.getInstance().isScheduled(auto)) {
                finished = true;
                elapsedSec = simTimeSec;
                break;
            }
        }

        assertTrue(finished, "PathPlanner auto did not finish in simulation.");
        assertTrue(distanceTraveledMeters > 2.0,
                "Drive did not appear to follow paths. distanceTraveledMeters=" + distanceTraveledMeters);

        Logger.end();
        loggerClosed = true;

        Path wpilog = findLatestWpiLog(outputDir);
        VisionJumpSummary visionSummary = extractVisionJumpSummary(wpilog);
        ShooterGateSummary shooterSummary = extractShooterGateSummary(wpilog);

        Path summaryFile = outputDir.resolve("brag_vision_jump_summary.txt");
        Files.writeString(summaryFile, formatSummary(wpilog, elapsedSec, distanceTraveledMeters, visionSummary));
        System.out.println("Brag vision jump summary: " + summaryFile.toAbsolutePath());

        Path shooterSummaryFile = outputDir.resolve("brag_shooter_gate_summary.txt");
        Files.writeString(shooterSummaryFile, formatShooterSummary(wpilog, shooterSummary));
        System.out.println("Brag shooter gate summary: " + shooterSummaryFile.toAbsolutePath());
    }

    private static void runLoggedSchedulerCycle() {
        long periodicBeforeStartUs = RobotController.getFPGATime();
        Logger.AdvancedHooks.invokePeriodicBeforeUser();
        long periodicBeforeEndUs = RobotController.getFPGATime();

        CommandScheduler.getInstance().run();

        long userCodeEndUs = RobotController.getFPGATime();
        Logger.AdvancedHooks.invokePeriodicAfterUser(
                userCodeEndUs - periodicBeforeEndUs,
                periodicBeforeEndUs - periodicBeforeStartUs);
    }

    private static Path resolveOutputDir() {
        String configured = System.getProperty("bragVision.sim.outDir");
        if (configured == null || configured.isBlank()) {
            configured = System.getenv("BRAG_VISION_SIM_OUT_DIR");
        }
        if (configured == null || configured.isBlank()) {
            configured = "build/brag-vision-sim";
        }
        return Path.of(configured).toAbsolutePath();
    }

    private static Path findLatestWpiLog(Path directory) throws IOException {
        try (var paths = Files.list(directory)) {
            return paths
                    .filter(path -> path.getFileName().toString().endsWith(".wpilog"))
                    .max((a, b) -> Long.compare(a.toFile().lastModified(), b.toFile().lastModified()))
                    .orElseThrow(() -> new IllegalStateException("No .wpilog files found in " + directory));
        }
    }

    private static VisionJumpSummary extractVisionJumpSummary(Path wpilog) throws IOException {
        DataLogReader reader = new DataLogReader(wpilog.toString());
        if (!reader.isValid()) {
            throw new IllegalStateException("Invalid WPILOG file: " + wpilog);
        }

        int sequenceEntry = -1;
        int jumpCountEntry = -1;
        int rejectCountEntry = -1;
        int eventTypeEntry = -1;
        int eventTimestampEntry = -1;
        int eventCameraEntry = -1;
        int eventTranslationEntry = -1;
        int eventHeadingEntry = -1;
        int eventTagCountEntry = -1;
        int eventAmbiguityEntry = -1;
        int eventAvgDistanceEntry = -1;

        MutableEvent event = null;
        long activeSequence = Long.MIN_VALUE;
        int sequenceSamples = 0;
        long jumpCount = 0;
        long rejectCount = 0;
        List<JumpEvent> jumpEvents = new ArrayList<>();

        for (DataLogRecord record : reader) {
            if (record.isStart()) {
                var start = record.getStartData();
                String name = start.name;
                if (name.contains("vision/events/sequence")) {
                    sequenceEntry = start.entry;
                } else if (name.contains("vision/events/jumpCount")) {
                    jumpCountEntry = start.entry;
                } else if (name.contains("vision/events/rejectCount")) {
                    rejectCountEntry = start.entry;
                } else if (name.contains("vision/events/last/type")) {
                    eventTypeEntry = start.entry;
                } else if (name.contains("vision/events/last/timestampSeconds")) {
                    eventTimestampEntry = start.entry;
                } else if (name.contains("vision/events/last/cameraIndex")) {
                    eventCameraEntry = start.entry;
                } else if (name.contains("vision/events/last/translationDeltaMeters")) {
                    eventTranslationEntry = start.entry;
                } else if (name.contains("vision/events/last/headingDeltaDegrees")) {
                    eventHeadingEntry = start.entry;
                } else if (name.contains("vision/events/last/tagCount")) {
                    eventTagCountEntry = start.entry;
                } else if (name.contains("vision/events/last/ambiguity")) {
                    eventAmbiguityEntry = start.entry;
                } else if (name.contains("vision/events/last/avgDistanceMeters")) {
                    eventAvgDistanceEntry = start.entry;
                }
                continue;
            }
            if (record.isControl()) {
                continue;
            }

            int entry = record.getEntry();
            if (entry == sequenceEntry) {
                long nextSequence = record.getInteger();
                sequenceSamples++;
                if (event != null && activeSequence != Long.MIN_VALUE && activeSequence != nextSequence) {
                    maybeCaptureJump(event, jumpEvents);
                }
                if (event == null) {
                    event = new MutableEvent(nextSequence);
                } else if (activeSequence == Long.MIN_VALUE) {
                    event.sequence = nextSequence;
                } else if (activeSequence != nextSequence) {
                    event = new MutableEvent(nextSequence, event);
                }
                activeSequence = nextSequence;
                continue;
            }
            if (entry == jumpCountEntry) {
                jumpCount = record.getInteger();
                continue;
            }
            if (entry == rejectCountEntry) {
                rejectCount = record.getInteger();
                continue;
            }
            if (event == null) {
                continue;
            }
            if (entry == eventTypeEntry) {
                event.type = record.getString();
            } else if (entry == eventTimestampEntry) {
                event.timestampSeconds = record.getDouble();
            } else if (entry == eventCameraEntry) {
                event.cameraIndex = (int) record.getInteger();
            } else if (entry == eventTranslationEntry) {
                event.translationDeltaMeters = record.getDouble();
            } else if (entry == eventHeadingEntry) {
                event.headingDeltaDegrees = record.getDouble();
            } else if (entry == eventTagCountEntry) {
                event.tagCount = (int) record.getInteger();
            } else if (entry == eventAmbiguityEntry) {
                event.ambiguity = record.getDouble();
            } else if (entry == eventAvgDistanceEntry) {
                event.avgDistanceMeters = record.getDouble();
            }
        }

        if (event != null) {
            maybeCaptureJump(event, jumpEvents);
        }

        double maxTranslation = 0.0;
        double maxHeading = 0.0;
        for (JumpEvent jumpEvent : jumpEvents) {
            maxTranslation = Math.max(maxTranslation, jumpEvent.translationDeltaMeters());
            maxHeading = Math.max(maxHeading, jumpEvent.headingDeltaDegrees());
        }

        return new VisionJumpSummary(
                sequenceSamples,
                jumpCount,
                rejectCount,
                maxTranslation,
                maxHeading,
                jumpEvents);
    }

    private static void maybeCaptureJump(MutableEvent event, List<JumpEvent> jumpEvents) {
        if (event.sequence <= 0) {
            return;
        }
        if (!"jump".equals(event.type)) {
            return;
        }
        jumpEvents.add(new JumpEvent(
                event.sequence,
                event.timestampSeconds,
                event.cameraIndex,
                event.translationDeltaMeters,
                event.headingDeltaDegrees,
                event.tagCount,
                event.ambiguity,
                event.avgDistanceMeters));
    }

    private static String formatSummary(
            Path wpilog,
            double elapsedSec,
            double distanceTraveledMeters,
            VisionJumpSummary summary) {
        StringBuilder jumpLines = new StringBuilder();
        for (JumpEvent event : summary.jumpEvents()) {
            jumpLines.append(String.format(
                    Locale.US,
                    "  seq=%d t=%.3f cam=%d dPos=%.3f dYaw=%.3f tags=%d amb=%.3f avgDist=%.3f%n",
                    event.sequence(),
                    event.timestampSeconds(),
                    event.cameraIndex(),
                    event.translationDeltaMeters(),
                    event.headingDeltaDegrees(),
                    event.tagCount(),
                    event.ambiguity(),
                    event.avgDistanceMeters()));
        }
        if (jumpLines.length() == 0) {
            jumpLines.append("  (none)\n");
        }

        return String.format(
                Locale.US,
                "auto=%s%n"
                        + "wpilog=%s%n"
                        + "elapsed_sec=%.3f%n"
                        + "distance_traveled_m=%.3f%n"
                        + "vision_event_sequence_samples=%d%n"
                        + "vision_jump_count=%d%n"
                        + "vision_reject_count=%d%n"
                        + "vision_jump_event_records=%d%n"
                        + "vision_jump_max_translation_m=%.3f%n"
                        + "vision_jump_max_heading_deg=%.3f%n"
                        + "%n"
                        + "jump_events=%n"
                        + "%s",
                AUTO_NAME,
                wpilog.toAbsolutePath(),
                elapsedSec,
                distanceTraveledMeters,
                summary.sequenceSamples(),
                summary.jumpCount(),
                summary.rejectCount(),
                summary.jumpEvents().size(),
                summary.maxTranslationDeltaMeters(),
                summary.maxHeadingDeltaDegrees(),
                jumpLines.toString());
    }

    private static ShooterGateSummary extractShooterGateSummary(Path wpilog) throws IOException {
        DataLogReader reader = new DataLogReader(wpilog.toString());
        if (!reader.isValid()) {
            throw new IllegalStateException("Invalid WPILOG file: " + wpilog);
        }

        int distanceValidEntry = -1;
        int gateOpenEntry = -1;
        int blockReasonEntry = -1;
        int shooterAtSetpointEntry = -1;
        int aimReadyEntry = -1;
        int leftVelocityErrorEntry = -1;
        int rightVelocityErrorEntry = -1;
        int hoodAngleErrorEntry = -1;
        int shooterTargetLeftEntry = -1;
        int shooterTargetRightEntry = -1;
        int kickerTorqueEntry = -1;

        for (DataLogRecord record : reader) {
            if (!record.isStart()) {
                continue;
            }
            var start = record.getStartData();
            String name = start.name;
            if (name.contains("Shooting/DistanceValid")) {
                distanceValidEntry = start.entry;
            } else if (name.contains("Shooting/GateOpen")) {
                gateOpenEntry = start.entry;
            } else if (name.contains("Shooting/BlockReason")) {
                blockReasonEntry = start.entry;
            } else if (name.contains("Shooting/ShooterAtSetpoint")) {
                shooterAtSetpointEntry = start.entry;
            } else if (name.contains("Shooting/AimReady")) {
                aimReadyEntry = start.entry;
            } else if (name.contains("Shooter/Readiness/LeftVelocityErrorRpm")) {
                leftVelocityErrorEntry = start.entry;
            } else if (name.contains("Shooter/Readiness/RightVelocityErrorRpm")) {
                rightVelocityErrorEntry = start.entry;
            } else if (name.contains("Shooter/Readiness/HoodAngleErrorDeg")) {
                hoodAngleErrorEntry = start.entry;
            } else if (name.contains("Shooter/TargetLeftRpm")) {
                shooterTargetLeftEntry = start.entry;
            } else if (name.contains("Shooter/TargetRightRpm")) {
                shooterTargetRightEntry = start.entry;
            } else if (name.contains("Shooter/KickerTorqueAmps")) {
                kickerTorqueEntry = start.entry;
            }
        }

        boolean distanceValid = false;
        boolean gateOpen = false;
        String blockReason = "";
        boolean shooterAtSetpoint = false;
        boolean aimReady = false;
        double leftVelocityErrorRpm = Double.NaN;
        double rightVelocityErrorRpm = Double.NaN;
        double hoodAngleErrorDeg = Double.NaN;
        double shooterTargetLeftRpm = Double.NaN;
        double shooterTargetRightRpm = Double.NaN;
        double kickerTorqueAmps = 0.0;

        long firstDistanceValidUs = Long.MIN_VALUE;
        long firstGateOpenUs = Long.MIN_VALUE;
        long firstKickerActiveUs = Long.MIN_VALUE;
        Map<String, Integer> preOpenBlockReasonCounts = new LinkedHashMap<>();
        List<String> preOpenTransitions = new ArrayList<>();

        for (DataLogRecord record : reader) {
            if (record.isStart() || record.isControl()) {
                continue;
            }

            int entry = record.getEntry();
            long timestampUs = record.getTimestamp();
            boolean relevantUpdate = false;

            if (entry == distanceValidEntry) {
                distanceValid = record.getBoolean();
                relevantUpdate = true;
                if (distanceValid && firstDistanceValidUs == Long.MIN_VALUE) {
                    firstDistanceValidUs = timestampUs;
                }
            } else if (entry == gateOpenEntry) {
                gateOpen = record.getBoolean();
                relevantUpdate = true;
                if (gateOpen
                        && firstGateOpenUs == Long.MIN_VALUE
                        && firstDistanceValidUs != Long.MIN_VALUE) {
                    firstGateOpenUs = timestampUs;
                }
            } else if (entry == blockReasonEntry) {
                blockReason = record.getString();
                relevantUpdate = true;
                if (firstDistanceValidUs != Long.MIN_VALUE && firstGateOpenUs == Long.MIN_VALUE) {
                    if (blockReason != null && !blockReason.isBlank()) {
                        preOpenBlockReasonCounts.merge(blockReason, 1, Integer::sum);
                    }
                }
            } else if (entry == shooterAtSetpointEntry) {
                shooterAtSetpoint = record.getBoolean();
                relevantUpdate = true;
            } else if (entry == aimReadyEntry) {
                aimReady = record.getBoolean();
                relevantUpdate = true;
            } else if (entry == leftVelocityErrorEntry) {
                leftVelocityErrorRpm = record.getDouble();
                relevantUpdate = true;
            } else if (entry == rightVelocityErrorEntry) {
                rightVelocityErrorRpm = record.getDouble();
                relevantUpdate = true;
            } else if (entry == hoodAngleErrorEntry) {
                hoodAngleErrorDeg = record.getDouble();
                relevantUpdate = true;
            } else if (entry == shooterTargetLeftEntry) {
                shooterTargetLeftRpm = record.getDouble();
                relevantUpdate = true;
            } else if (entry == shooterTargetRightEntry) {
                shooterTargetRightRpm = record.getDouble();
                relevantUpdate = true;
            } else if (entry == kickerTorqueEntry) {
                kickerTorqueAmps = record.getDouble();
                relevantUpdate = true;
                if (Math.abs(kickerTorqueAmps) > 1e-6 && firstKickerActiveUs == Long.MIN_VALUE) {
                    firstKickerActiveUs = timestampUs;
                }
            }

            if (!relevantUpdate) {
                continue;
            }

            if (firstDistanceValidUs != Long.MIN_VALUE && firstGateOpenUs == Long.MIN_VALUE) {
                preOpenTransitions.add(String.format(
                        Locale.US,
                        "  t=%.3f gateOpen=%s blockReason=%s shooterAtSetpoint=%s aimReady=%s "
                                + "leftErr=%.1f rightErr=%.1f hoodErrDeg=%.2f targetL=%.1f targetR=%.1f",
                        timestampUs / 1_000_000.0,
                        gateOpen,
                        blockReason == null || blockReason.isBlank() ? "<none>" : blockReason,
                        shooterAtSetpoint,
                        aimReady,
                        leftVelocityErrorRpm,
                        rightVelocityErrorRpm,
                        hoodAngleErrorDeg,
                        shooterTargetLeftRpm,
                        shooterTargetRightRpm));
            }
        }

        double distanceToOpenSec = Double.NaN;
        if (firstDistanceValidUs != Long.MIN_VALUE && firstGateOpenUs != Long.MIN_VALUE) {
            distanceToOpenSec = (firstGateOpenUs - firstDistanceValidUs) / 1_000_000.0;
        }
        double distanceToKickerSec = Double.NaN;
        if (firstDistanceValidUs != Long.MIN_VALUE && firstKickerActiveUs != Long.MIN_VALUE) {
            distanceToKickerSec = (firstKickerActiveUs - firstDistanceValidUs) / 1_000_000.0;
        }

        return new ShooterGateSummary(
                firstDistanceValidUs,
                firstGateOpenUs,
                firstKickerActiveUs,
                distanceToOpenSec,
                distanceToKickerSec,
                preOpenBlockReasonCounts,
                preOpenTransitions);
    }

    private static String formatShooterSummary(Path wpilog, ShooterGateSummary summary) {
        StringBuilder blockCounts = new StringBuilder();
        if (summary.preOpenBlockReasonCounts().isEmpty()) {
            blockCounts.append("  (none)\n");
        } else {
            for (var entry : summary.preOpenBlockReasonCounts().entrySet()) {
                blockCounts.append(
                        String.format(Locale.US, "  %s: %d%n", entry.getKey(), entry.getValue()));
            }
        }

        StringBuilder transitions = new StringBuilder();
        if (summary.preOpenTransitions().isEmpty()) {
            transitions.append("  (none)\n");
        } else {
            for (String line : summary.preOpenTransitions()) {
                transitions.append(line).append(System.lineSeparator());
            }
        }

        return String.format(
                Locale.US,
                "auto=%s%n"
                        + "wpilog=%s%n"
                        + "first_distance_valid_us=%d%n"
                        + "first_gate_open_us=%d%n"
                        + "first_kicker_active_us=%d%n"
                        + "distance_to_gate_open_sec=%.3f%n"
                        + "distance_to_kicker_active_sec=%.3f%n"
                        + "%n"
                        + "pre_open_block_reason_counts=%n"
                        + "%s"
                        + "%n"
                        + "pre_open_transitions=%n"
                        + "%s",
                AUTO_NAME,
                wpilog.toAbsolutePath(),
                summary.firstDistanceValidUs(),
                summary.firstGateOpenUs(),
                summary.firstKickerActiveUs(),
                summary.distanceValidToGateOpenSec(),
                summary.distanceValidToKickerActiveSec(),
                blockCounts.toString(),
                transitions.toString());
    }

    private static final class MutableEvent {
        private long sequence;
        private String type = "none";
        private double timestampSeconds = Double.NaN;
        private int cameraIndex = -1;
        private double translationDeltaMeters = Double.NaN;
        private double headingDeltaDegrees = Double.NaN;
        private int tagCount = 0;
        private double ambiguity = Double.NaN;
        private double avgDistanceMeters = Double.NaN;

        private MutableEvent(long sequence) {
            this.sequence = sequence;
        }

        private MutableEvent(long sequence, MutableEvent previous) {
            this.sequence = sequence;
            this.type = previous.type;
            this.timestampSeconds = previous.timestampSeconds;
            this.cameraIndex = previous.cameraIndex;
            this.translationDeltaMeters = previous.translationDeltaMeters;
            this.headingDeltaDegrees = previous.headingDeltaDegrees;
            this.tagCount = previous.tagCount;
            this.ambiguity = previous.ambiguity;
            this.avgDistanceMeters = previous.avgDistanceMeters;
        }
    }

    private record JumpEvent(
            long sequence,
            double timestampSeconds,
            int cameraIndex,
            double translationDeltaMeters,
            double headingDeltaDegrees,
            int tagCount,
            double ambiguity,
            double avgDistanceMeters) {
    }

    private record VisionJumpSummary(
            int sequenceSamples,
            long jumpCount,
            long rejectCount,
            double maxTranslationDeltaMeters,
            double maxHeadingDeltaDegrees,
            List<JumpEvent> jumpEvents) {
    }

    private record ShooterGateSummary(
            long firstDistanceValidUs,
            long firstGateOpenUs,
            long firstKickerActiveUs,
            double distanceValidToGateOpenSec,
            double distanceValidToKickerActiveSec,
            Map<String, Integer> preOpenBlockReasonCounts,
            List<String> preOpenTransitions) {
    }
}
