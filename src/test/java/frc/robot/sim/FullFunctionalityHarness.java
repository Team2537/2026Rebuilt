package frc.robot.sim;

import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj.simulation.XboxControllerSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.transfer.Transfer;
import frc.robot.subsystems.transfer.TransferIO;
import java.io.IOException;
import java.lang.reflect.Field;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.IdentityHashMap;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.Set;
import java.util.concurrent.atomic.AtomicReference;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

final class FullFunctionalityHarness {
    static final double LOOP_PERIOD_SEC = 0.02;

    private static final Pattern AUTO_NAMED_COMMAND_PATTERN =
            Pattern.compile("\\\"name\\\"\\s*:\\s*\\\"([^\\\"]+)\\\"");
    private static final Pattern AUTO_PATH_PATTERN =
            Pattern.compile("\\\"pathName\\\"\\s*:\\s*\\\"([^\\\"]+)\\\"");

    private static final AtomicReference<CommandLifecycleRecorder> ACTIVE_RECORDER = new AtomicReference<>();
    private static boolean schedulerHooksInstalled = false;

    private FullFunctionalityHarness() {}

    static String formatExpectedVsActual(String expectation, Object expected, Object actual) {
        return "Expected: " + expectation + " | expected=" + expected + " | actual=" + actual;
    }

    static Command namedCommand(String name) {
        Command command = NamedCommands.getCommand(name);
        assertNotNull(command, "Expected NamedCommands registry to contain: " + name);
        return command;
    }

    static Command getDashboardCommand(String key) {
        Sendable sendable = SmartDashboard.getData(key);
        assertTrue(
                sendable instanceof Command,
                "Expected SmartDashboard key '" + key + "' to be a Command sendable.");
        return (Command) sendable;
    }

    static Set<String> parseDeclaredNamedCommands(Path autoFile) throws IOException {
        if (!Files.exists(autoFile)) {
            return Collections.emptySet();
        }
        String json = Files.readString(autoFile);
        Matcher matcher = AUTO_NAMED_COMMAND_PATTERN.matcher(json);
        Set<String> names = new LinkedHashSet<>();
        while (matcher.find()) {
            names.add(matcher.group(1));
        }
        return names;
    }

    static List<String> parseDeclaredPathNames(Path autoFile) throws IOException {
        if (!Files.exists(autoFile)) {
            return List.of();
        }
        String json = Files.readString(autoFile);
        Matcher matcher = AUTO_PATH_PATTERN.matcher(json);
        List<String> pathNames = new ArrayList<>();
        while (matcher.find()) {
            pathNames.add(matcher.group(1));
        }
        return pathNames;
    }

    static synchronized void installSchedulerHooksIfNeeded() {
        if (schedulerHooksInstalled) {
            return;
        }
        CommandScheduler scheduler = CommandScheduler.getInstance();
        scheduler.onCommandInitialize(command -> {
            CommandLifecycleRecorder recorder = ACTIVE_RECORDER.get();
            if (recorder != null) {
                recorder.onInitialize(command);
            }
        });
        scheduler.onCommandFinish(command -> {
            CommandLifecycleRecorder recorder = ACTIVE_RECORDER.get();
            if (recorder != null) {
                recorder.onFinish(command);
            }
        });
        scheduler.onCommandInterrupt(command -> {
            CommandLifecycleRecorder recorder = ACTIVE_RECORDER.get();
            if (recorder != null) {
                recorder.onInterrupt(command);
            }
        });
        schedulerHooksInstalled = true;
    }

    static <T> T getPrivateField(Object target, String fieldName, Class<T> type) {
        try {
            Field field = target.getClass().getDeclaredField(fieldName);
            field.setAccessible(true);
            Object value = field.get(target);
            return type.cast(value);
        } catch (ReflectiveOperationException exception) {
            throw new IllegalStateException(
                    "Unable to read field '" + fieldName + "' from " + target.getClass().getName(),
                    exception);
        }
    }

    record CommandCounts(int starts, int finishes, int interrupts) {
        CommandCounts minus(CommandCounts other) {
            return new CommandCounts(
                    starts - other.starts,
                    finishes - other.finishes,
                    interrupts - other.interrupts);
        }

        @Override
        public String toString() {
            return String.format(Locale.US, "starts=%d finishes=%d interrupts=%d", starts, finishes, interrupts);
        }
    }

    record CommandRun(double startSec, double endSec, boolean interrupted) {
        double durationSec() {
            return endSec - startSec;
        }
    }

    static final class CommandLifecycleRecorder {
        private final Map<String, MutableCounts> byName = new HashMap<>();
        private final Map<String, List<CommandRun>> runsByName = new HashMap<>();
        private final Map<Command, ActiveRun> activeRuns = new IdentityHashMap<>();

        synchronized void onInitialize(Command command) {
            String name = commandName(command);
            byName.computeIfAbsent(name, ignored -> new MutableCounts()).starts++;
            activeRuns.put(command, new ActiveRun(name, Timer.getFPGATimestamp()));
        }

        synchronized void onFinish(Command command) {
            endRun(command, false);
        }

        synchronized void onInterrupt(Command command) {
            endRun(command, true);
        }

        synchronized CommandCounts getCounts(String commandName) {
            MutableCounts counts = byName.get(commandName);
            if (counts == null) {
                return new CommandCounts(0, 0, 0);
            }
            return new CommandCounts(counts.starts, counts.finishes, counts.interrupts);
        }

        synchronized int runningCount(String commandName) {
            int running = 0;
            for (ActiveRun run : activeRuns.values()) {
                if (run.commandName.equals(commandName)) {
                    running++;
                }
            }
            return running;
        }

        synchronized List<CommandRun> getRuns(String commandName) {
            return List.copyOf(runsByName.getOrDefault(commandName, List.of()));
        }

        synchronized CommandRun latestRun(String commandName) {
            List<CommandRun> runs = runsByName.getOrDefault(commandName, List.of());
            return runs.isEmpty() ? null : runs.get(runs.size() - 1);
        }

        private void endRun(Command command, boolean interrupted) {
            ActiveRun active = activeRuns.remove(command);
            String name = commandName(command);
            MutableCounts counts = byName.computeIfAbsent(name, ignored -> new MutableCounts());
            if (interrupted) {
                counts.interrupts++;
            } else {
                counts.finishes++;
            }
            if (active != null) {
                runsByName
                        .computeIfAbsent(active.commandName, ignored -> new ArrayList<>())
                        .add(new CommandRun(active.startSec, Timer.getFPGATimestamp(), interrupted));
            }
        }

        private static String commandName(Command command) {
            if (command == null) {
                return "UnnamedCommand";
            }
            String name = command.getName();
            if (name == null || name.isBlank()) {
                String className = command.getClass().getSimpleName();
                return className == null || className.isBlank() ? "UnnamedCommand" : className;
            }
            return name;
        }

        private static final class MutableCounts {
            int starts;
            int finishes;
            int interrupts;
        }

        private record ActiveRun(String commandName, double startSec) {}
    }

    static final class Context implements AutoCloseable {
        final RobotContainer container;
        final Drive drive;
        final Shooter shooter;
        final Intake intake;
        final Transfer transfer;

        final ShooterIO.ShooterIOInputs shooterInputs;
        final IntakeIO.IntakeIOInputs intakeInputs;
        final TransferIO.TransferIOInputs transferInputs;

        final CommandLifecycleRecorder recorder;
        final XboxControllerSim driverControllerSim;
        final XboxControllerSim godControllerSim;

        Context(boolean connectGodController) {
            HAL.initialize(500, 0);
            SimHooks.pauseTiming();
            DriverStationSim.resetData();
            AutoBuilder.resetForTesting();
            NamedCommands.clearAll();
            FullFunctionalityHarness.installSchedulerHooksIfNeeded();

            CommandScheduler scheduler = CommandScheduler.getInstance();
            scheduler.cancelAll();
            scheduler.getDefaultButtonLoop().clear();
            scheduler.getActiveButtonLoop().clear();
            scheduler.setActiveButtonLoop(scheduler.getDefaultButtonLoop());
            scheduler.clearComposedCommands();
            scheduler.unregisterAllSubsystems();

            DriverStationSim.setDsAttached(true);
            DriverStationSim.setEStop(false);
            DriverStationSim.setFmsAttached(false);
            DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
            configureJoystickPort(0, true);
            configureJoystickPort(5, connectGodController);
            DriverStationSim.setAutonomous(false);
            DriverStationSim.setTest(false);
            DriverStationSim.setEnabled(false);
            DriverStationSim.notifyNewData();

            recorder = new CommandLifecycleRecorder();
            ACTIVE_RECORDER.set(recorder);

            container = new RobotContainer();
            drive = getPrivateField(container, "drive", Drive.class);
            shooter = getPrivateField(container, "shooter", Shooter.class);
            intake = getPrivateField(container, "intake", Intake.class);
            transfer = getPrivateField(container, "transfer", Transfer.class);

            shooterInputs = (ShooterIO.ShooterIOInputs) getPrivateField(shooter, "inputs", Object.class);
            intakeInputs = (IntakeIO.IntakeIOInputs) getPrivateField(intake, "inputs", Object.class);
            transferInputs = (TransferIO.TransferIOInputs) getPrivateField(transfer, "inputs", Object.class);

            driverControllerSim = new XboxControllerSim(0);
            godControllerSim = connectGodController ? new XboxControllerSim(5) : null;
            clearControllerState();
            runCycles(6);
        }

        void setDisabled() {
            DriverStationSim.setAutonomous(false);
            DriverStationSim.setTest(false);
            DriverStationSim.setEnabled(false);
            DriverStationSim.notifyNewData();
        }

        void setTeleopEnabled() {
            DriverStationSim.setAutonomous(false);
            DriverStationSim.setTest(false);
            DriverStationSim.setEnabled(true);
            DriverStationSim.notifyNewData();
        }

        void setAutonomousEnabled() {
            DriverStationSim.setAutonomous(true);
            DriverStationSim.setTest(false);
            DriverStationSim.setEnabled(true);
            DriverStationSim.notifyNewData();
        }

        void runCycles(int cycles) {
            for (int i = 0; i < cycles; i++) {
                SimHooks.stepTiming(LOOP_PERIOD_SEC);
                DriverStationSim.notifyNewData();
                CommandScheduler.getInstance().run();
                container.robotPeriodic();
                container.simulationPeriodic();
            }
        }

        boolean runUntil(BooleanCondition condition, int maxCycles) {
            for (int i = 0; i < maxCycles; i++) {
                if (condition.getAsBoolean()) {
                    return true;
                }
                runCycles(1);
            }
            return condition.getAsBoolean();
        }

        void clearControllerState() {
            driverControllerSim.setLeftX(0.0);
            driverControllerSim.setLeftY(0.0);
            driverControllerSim.setRightX(0.0);
            driverControllerSim.setRightY(0.0);
            driverControllerSim.setLeftTriggerAxis(0.0);
            driverControllerSim.setRightTriggerAxis(0.0);
            driverControllerSim.setAButton(false);
            driverControllerSim.setBButton(false);
            driverControllerSim.setXButton(false);
            driverControllerSim.setYButton(false);
            driverControllerSim.setLeftBumperButton(false);
            driverControllerSim.setRightBumperButton(false);
            driverControllerSim.setBackButton(false);
            driverControllerSim.setStartButton(false);
            driverControllerSim.setLeftStickButton(false);
            driverControllerSim.setRightStickButton(false);
            driverControllerSim.setPOV(-1);
            if (godControllerSim != null) {
                godControllerSim.setLeftX(0.0);
                godControllerSim.setLeftY(0.0);
                godControllerSim.setRightX(0.0);
                godControllerSim.setRightY(0.0);
                godControllerSim.setLeftTriggerAxis(0.0);
                godControllerSim.setRightTriggerAxis(0.0);
                godControllerSim.setAButton(false);
                godControllerSim.setBButton(false);
                godControllerSim.setXButton(false);
                godControllerSim.setYButton(false);
                godControllerSim.setLeftBumperButton(false);
                godControllerSim.setRightBumperButton(false);
                godControllerSim.setBackButton(false);
                godControllerSim.setStartButton(false);
                godControllerSim.setLeftStickButton(false);
                godControllerSim.setRightStickButton(false);
                godControllerSim.setPOV(-1);
            }
            DriverStationSim.notifyNewData();
        }

        void tapDriverButton(java.util.function.Consumer<Boolean> setter) {
            setter.accept(true);
            runCycles(4);
            setter.accept(false);
            runCycles(4);
        }

        void tapDriverPov(int angle) {
            driverControllerSim.setPOV(angle);
            runCycles(4);
            driverControllerSim.setPOV(-1);
            runCycles(4);
        }

        void tapGodButton(java.util.function.Consumer<Boolean> setter) {
            if (godControllerSim == null) {
                throw new IllegalStateException("God controller was not connected for this context.");
            }
            setter.accept(true);
            runCycles(4);
            setter.accept(false);
            runCycles(4);
        }

        void tapGodPov(int angle) {
            if (godControllerSim == null) {
                throw new IllegalStateException("God controller was not connected for this context.");
            }
            godControllerSim.setPOV(angle);
            runCycles(4);
            godControllerSim.setPOV(-1);
            runCycles(4);
        }

        @Override
        public void close() {
            clearControllerState();
            CommandScheduler scheduler = CommandScheduler.getInstance();
            scheduler.cancelAll();
            scheduler.getDefaultButtonLoop().clear();
            scheduler.getActiveButtonLoop().clear();
            scheduler.setActiveButtonLoop(scheduler.getDefaultButtonLoop());
            scheduler.clearComposedCommands();
            scheduler.unregisterAllSubsystems();
            DriverStationSim.setEnabled(false);
            DriverStationSim.notifyNewData();
            runCycles(2);
            ACTIVE_RECORDER.compareAndSet(recorder, null);
            SimHooks.resumeTiming();
        }

        private static void configureJoystickPort(int port, boolean connected) {
            if (connected) {
                DriverStationSim.setJoystickAxisCount(port, 6);
                DriverStationSim.setJoystickButtonCount(port, 12);
                DriverStationSim.setJoystickPOVCount(port, 1);
                DriverStationSim.setJoystickIsXbox(port, true);
            } else {
                DriverStationSim.setJoystickAxisCount(port, 0);
                DriverStationSim.setJoystickButtonCount(port, 0);
                DriverStationSim.setJoystickPOVCount(port, 0);
                DriverStationSim.setJoystickIsXbox(port, false);
            }
        }
    }

    @FunctionalInterface
    interface BooleanCondition {
        boolean getAsBoolean();
    }
}
