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
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.IdentityHashMap;
import java.util.List;
import java.util.Map;
import java.util.stream.Collectors;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

class RightMidCycleAutoSimTest {
    private static final String AUTO_NAME = "right mid cycle";
    private static final String TIMING_KEY = "AutoSim/SimTimeSec";
    private static final double LOOP_PERIOD_SEC = 0.02;
    private static final double MAX_SIM_TIME_SEC = 45.0;
    private static final String PATH_RIGHT_TO_MIDFIELD = "right to midfield";
    private static final String PATH_RIGHT_MIDFIELD_INTAKE = "right midfield intake";
    private static final String PATH_RIGHT_AFTER_INTAKE_TO_TRENCH = "right after intake to trench";
    private static final double FIRST_SHOOT_DEADLINE_WAIT_SEC = 2.5;
    private static final double FINAL_SHOOT_DEADLINE_WAIT_SEC = 4.0;

    private Path outputDir;
    private boolean commandCaptureEnabled = false;
    private double callbackSimTimeSec = 0.0;
    private final Map<Command, CommandStart> activeCommands = new IdentityHashMap<>();
    private final List<CommandWindow> completedCommands = new ArrayList<>();
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
        installSchedulerHooksIfNeeded();
        activeCommands.clear();
        completedCommands.clear();
        commandCaptureEnabled = true;
        callbackSimTimeSec = 0.0;

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
        commandCaptureEnabled = false;
        if (!loggerClosed) {
            Logger.end();
            loggerClosed = true;
        }
        SimHooks.resumeTiming();
    }

    @Test
    void rightMidCycleAutoWritesTimingSamplesToWpilog() throws IOException {
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
        List<PathWindow> pathWindows = new ArrayList<>();
        String activePathName = "";
        double activePathStartSec = Double.NaN;

        int maxIterations = (int) Math.ceil(MAX_SIM_TIME_SEC / LOOP_PERIOD_SEC);
        for (int i = 0; i < maxIterations; i++) {
            SimHooks.stepTiming(LOOP_PERIOD_SEC);
            DriverStationSim.notifyNewData();
            callbackSimTimeSec = (i + 1) * LOOP_PERIOD_SEC;
            runLoggedSchedulerCycle();

            double simTimeSec = callbackSimTimeSec;
            String currentPathName = PathPlannerAuto.currentPathName == null ? "" : PathPlannerAuto.currentPathName;
            if (!currentPathName.equals(activePathName)) {
                if (!activePathName.isEmpty() && Double.isFinite(activePathStartSec)) {
                    pathWindows.add(new PathWindow(activePathName, activePathStartSec, simTimeSec));
                }
                activePathName = currentPathName;
                activePathStartSec = activePathName.isEmpty() ? Double.NaN : simTimeSec;
            }

            Pose2d currentPose = drive.getPose();
            distanceTraveledMeters +=
                    currentPose.getTranslation().getDistance(previousPose.getTranslation());
            previousPose = currentPose;

            Logger.recordOutput(TIMING_KEY, simTimeSec);
            Logger.recordOutput("AutoSim/DistanceTraveledMeters", distanceTraveledMeters);
            Logger.recordOutput("AutoSim/AutoScheduled", CommandScheduler.getInstance().isScheduled(auto));
            Logger.recordOutput("AutoSim/ActivePathName", currentPathName);

            if (!CommandScheduler.getInstance().isScheduled(auto)) {
                finished = true;
                elapsedSec = simTimeSec;
                break;
            }
        }
        if (!activePathName.isEmpty() && Double.isFinite(activePathStartSec) && elapsedSec > activePathStartSec) {
            pathWindows.add(new PathWindow(activePathName, activePathStartSec, elapsedSec));
        }

        assertTrue(finished, "PathPlanner auto did not finish in simulation.");
        assertTrue(distanceTraveledMeters > 2.0,
                "Drive did not appear to follow paths. distanceTraveledMeters=" + distanceTraveledMeters);
        SectionTimingSummary sectionTimings = summarizeSections(pathWindows, elapsedSec);
        List<CommandWindow> commandWindows = buildSortedCommandWindows(elapsedSec);

        Logger.end();
        loggerClosed = true;
        Path wpilog = findLatestWpiLog(outputDir);
        TimingStats timingStats = extractTimingStats(wpilog, TIMING_KEY);
        assertTrue(timingStats.sampleCount() > 30,
                "Insufficient timing samples in WPILOG. sampleCount=" + timingStats.sampleCount());
        assertTrue(timingStats.monotonic(),
                "Timing key timestamps were not monotonic.");
        assertTrue(Math.abs(timingStats.meanPeriodSec() - LOOP_PERIOD_SEC) <= 0.01,
                "Unexpected mean timing period from WPILOG. meanPeriodSec=" + timingStats.meanPeriodSec());

        Path summaryFile = outputDir.resolve("right_mid_cycle_timing_summary.txt");
        Files.writeString(summaryFile, String.format(
                "auto=%s%n"
                        + "wpilog=%s%n"
                        + "elapsed_sec=%.3f%n"
                        + "distance_traveled_m=%.3f%n"
                        + "timing_key=%s%n"
                        + "sample_count=%d%n"
                        + "first_timestamp_us=%d%n"
                        + "last_timestamp_us=%d%n"
                        + "mean_period_sec=%.6f%n"
                        + "first_value_sec=%.3f%n"
                        + "last_value_sec=%.3f%n"
                        + "%n"
                        + "section_homing_parallel_sec=%.3f%n"
                        + "section_initial_shoot_deadline_sec=%.3f%n"
                        + "section_path_right_to_midfield_sec=%.3f%n"
                        + "section_intake_extend_sec=%.3f%n"
                        + "section_path_right_midfield_intake_sec=%.3f%n"
                        + "section_intake_stop_retract_sec=%.3f%n"
                        + "section_path_right_after_intake_to_trench_sec=%.3f%n"
                        + "section_final_shoot_deadline_sec=%.3f%n"
                        + "%n"
                        + "path_windows=%n%s%n"
                        + "%n"
                        + "command_windows=%n%s%n",
                AUTO_NAME,
                wpilog.toAbsolutePath(),
                elapsedSec,
                distanceTraveledMeters,
                TIMING_KEY,
                timingStats.sampleCount(),
                timingStats.firstTimestampUs(),
                timingStats.lastTimestampUs(),
                timingStats.meanPeriodSec(),
                timingStats.firstValueSec(),
                timingStats.lastValueSec(),
                sectionTimings.homingParallelSec(),
                sectionTimings.initialShootDeadlineSec(),
                sectionTimings.pathRightToMidfieldSec(),
                sectionTimings.intakeExtendSec(),
                sectionTimings.pathRightMidfieldIntakeSec(),
                sectionTimings.intakeStopRetractSec(),
                sectionTimings.pathRightAfterIntakeToTrenchSec(),
                sectionTimings.finalShootDeadlineSec(),
                formatPathWindows(pathWindows),
                formatCommandWindows(commandWindows)));
        System.out.println("Right mid cycle timing summary: " + summaryFile.toAbsolutePath());
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

    private void installSchedulerHooksIfNeeded() {
        CommandScheduler scheduler = CommandScheduler.getInstance();
        scheduler.onCommandInitialize(command -> {
            if (!commandCaptureEnabled) {
                return;
            }
            activeCommands.put(command, new CommandStart(callbackSimTimeSec, describeRequirements(command)));
        });
        scheduler.onCommandFinish(command -> {
            if (!commandCaptureEnabled) {
                return;
            }
            completeCommand(command, false);
        });
        scheduler.onCommandInterrupt(command -> {
            if (!commandCaptureEnabled) {
                return;
            }
            completeCommand(command, true);
        });
    }

    private void completeCommand(Command command, boolean interrupted) {
        CommandStart start = activeCommands.remove(command);
        if (start == null) {
            return;
        }
        completedCommands.add(new CommandWindow(
                command.getName(),
                start.requirements(),
                start.startSec(),
                callbackSimTimeSec,
                interrupted));
    }

    private static String describeRequirements(Command command) {
        return command.getRequirements().stream()
                .map(subsystem -> subsystem.getName())
                .sorted()
                .collect(Collectors.joining(","));
    }

    private List<CommandWindow> buildSortedCommandWindows(double elapsedSec) {
        List<CommandWindow> windows = new ArrayList<>(completedCommands);
        for (var entry : activeCommands.entrySet()) {
            Command command = entry.getKey();
            CommandStart start = entry.getValue();
            windows.add(new CommandWindow(
                    command.getName(),
                    start.requirements(),
                    start.startSec(),
                    elapsedSec,
                    true));
        }
        windows.sort(Comparator
                .comparingDouble(CommandWindow::startSec)
                .thenComparing(CommandWindow::name)
                .thenComparing(CommandWindow::requirements));
        return windows;
    }

    private static String formatCommandWindows(List<CommandWindow> windows) {
        return windows.stream()
                .map(window -> String.format(
                        "  %.3f-%.3f (%.3fs) interrupted=%s name=%s req=[%s]",
                        window.startSec(),
                        window.endSec(),
                        window.durationSec(),
                        window.interrupted(),
                        window.name(),
                        window.requirements()))
                .collect(Collectors.joining(System.lineSeparator()));
    }

    private static String formatPathWindows(List<PathWindow> windows) {
        return windows.stream()
                .sorted(Comparator.comparingDouble(PathWindow::startSec))
                .map(window -> String.format(
                        "  %.3f-%.3f (%.3fs) path=%s",
                        window.startSec(),
                        window.endSec(),
                        window.durationSec(),
                        window.pathName()))
                .collect(Collectors.joining(System.lineSeparator()));
    }

    private static Path resolveOutputDir() {
        String configured = System.getProperty("rightMidCycle.sim.outDir");
        if (configured == null || configured.isBlank()) {
            configured = System.getenv("RIGHT_MID_CYCLE_SIM_OUT_DIR");
        }
        if (configured == null || configured.isBlank()) {
            configured = "build/right-mid-cycle-sim";
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

    private static TimingStats extractTimingStats(Path wpilog, String keyContains) throws IOException {
        DataLogReader reader = new DataLogReader(wpilog.toString());
        if (!reader.isValid()) {
            throw new IllegalStateException("Invalid WPILOG file: " + wpilog);
        }

        int timingEntry = -1;
        long firstTimestampUs = Long.MIN_VALUE;
        long lastTimestampUs = Long.MIN_VALUE;
        long previousTimestampUs = Long.MIN_VALUE;
        double firstValueSec = Double.NaN;
        double lastValueSec = Double.NaN;
        int sampleCount = 0;
        boolean monotonic = true;

        for (DataLogRecord record : reader) {
            if (record.isStart()) {
                var start = record.getStartData();
                if (start.name.contains(keyContains)) {
                    timingEntry = start.entry;
                }
                continue;
            }
            if (record.isControl() || record.getEntry() != timingEntry) {
                continue;
            }

            long timestampUs = record.getTimestamp();
            if (sampleCount == 0) {
                firstTimestampUs = timestampUs;
                firstValueSec = record.getDouble();
            } else if (timestampUs < previousTimestampUs) {
                monotonic = false;
            }
            previousTimestampUs = timestampUs;
            lastTimestampUs = timestampUs;
            lastValueSec = record.getDouble();
            sampleCount++;
        }

        if (timingEntry == -1) {
            throw new IllegalStateException("Did not find timing entry containing '" + keyContains + "' in " + wpilog);
        }
        if (sampleCount == 0) {
            throw new IllegalStateException("Found timing entry but no samples for '" + keyContains + "' in " + wpilog);
        }

        double meanPeriodSec = sampleCount <= 1
                ? Double.NaN
                : ((lastTimestampUs - firstTimestampUs) / 1_000_000.0) / (sampleCount - 1);
        return new TimingStats(
                sampleCount,
                firstTimestampUs,
                lastTimestampUs,
                meanPeriodSec,
                firstValueSec,
                lastValueSec,
                monotonic);
    }

    private static SectionTimingSummary summarizeSections(List<PathWindow> pathWindows, double elapsedSec) {
        PathWindow path1 = findPathWindow(pathWindows, PATH_RIGHT_TO_MIDFIELD);
        PathWindow path2 = findPathWindow(pathWindows, PATH_RIGHT_MIDFIELD_INTAKE);
        PathWindow path3 = findPathWindow(pathWindows, PATH_RIGHT_AFTER_INTAKE_TO_TRENCH);

        double homingParallelSec = clampNonNegative(path1.startSec() - FIRST_SHOOT_DEADLINE_WAIT_SEC);
        double intakeExtendSec = clampNonNegative(path2.startSec() - path1.endSec());
        double intakeStopRetractSec = clampNonNegative(path3.startSec() - path2.endSec());
        double finalShootDeadlineSec = clampNonNegative(elapsedSec - path3.endSec());

        return new SectionTimingSummary(
                homingParallelSec,
                FIRST_SHOOT_DEADLINE_WAIT_SEC,
                path1.durationSec(),
                intakeExtendSec,
                path2.durationSec(),
                intakeStopRetractSec,
                path3.durationSec(),
                finalShootDeadlineSec);
    }

    private static PathWindow findPathWindow(List<PathWindow> pathWindows, String pathName) {
        return pathWindows.stream()
                .filter(window -> window.pathName().equals(pathName))
                .findFirst()
                .orElseThrow(() -> new IllegalStateException(
                        "Did not observe path '" + pathName + "' in timeline. Observed=" + pathWindows));
    }

    private static double clampNonNegative(double value) {
        return value < 0.0 ? 0.0 : value;
    }

    private record PathWindow(String pathName, double startSec, double endSec) {
        private double durationSec() {
            return endSec - startSec;
        }
    }

    private record CommandStart(double startSec, String requirements) {}

    private record CommandWindow(
            String name,
            String requirements,
            double startSec,
            double endSec,
            boolean interrupted) {
        private double durationSec() {
            return endSec - startSec;
        }
    }

    private record TimingStats(
            int sampleCount,
            long firstTimestampUs,
            long lastTimestampUs,
            double meanPeriodSec,
            double firstValueSec,
            double lastValueSec,
            boolean monotonic) {}

    private record SectionTimingSummary(
            double homingParallelSec,
            double initialShootDeadlineSec,
            double pathRightToMidfieldSec,
            double intakeExtendSec,
            double pathRightMidfieldIntakeSec,
            double intakeStopRetractSec,
            double pathRightAfterIntakeToTrenchSec,
            double finalShootDeadlineSec) {
    }
}
