package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.autos.AutoNamedCommands;
import frc.robot.autos.AutoRoutines;
import frc.robot.autos.AutoSelector;
import frc.robot.commands.DriveCommands;
import frc.robot.coordination.shooting.ShootCoordinator;
import frc.robot.generated.TunerConstants;
import frc.robot.sim.FuelSim;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.GyroIOSim;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOReal;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOReal;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.transfer.Transfer;
import frc.robot.subsystems.transfer.TransferIO;
import frc.robot.subsystems.transfer.TransferIOReal;
import frc.robot.subsystems.transfer.TransferIOSim;
import frc.robot.subsystems.vision.Vision;
import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.IdentityHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.WeakHashMap;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/** Owns subsystems, bindings, and autonomous orchestration. */
public final class RobotContainer {
    private static final int GOD_CONTROLLER_PORT = 5;
    private static final int COMMAND_EVENT_HISTORY_LIMIT = 40;
    private static final String UNKNOWN_COMMAND_SOURCE = "unknown";

    private final Drive drive;
    private final Vision vision;
    private final Shooter shooter;
    private final Transfer transfer;
    private final Intake intake;
    private final ShootCoordinator shootCoordinator;
    private final AutoSelector autoSelector;

    private final CommandXboxController driverController = new CommandXboxController(0);
    private final CommandXboxController godController = new CommandXboxController(GOD_CONTROLLER_PORT);

    private FuelSim fuelSim;
    private final Map<Command, String> commandSources = new WeakHashMap<>();
    private final Map<Command, Double> commandStartTimesSec = new IdentityHashMap<>();
    private final Map<Command, Integer> commandRunIds = new IdentityHashMap<>();
    private final ArrayDeque<String> recentCommandEvents = new ArrayDeque<>();
    private int nextCommandRunId = 1;

    public RobotContainer() {
        drive = createDrive();
        vision = Constants.isMechanismEnabled(Constants.Mechanism.VISION) ? new Vision(drive) : null;
        shooter = createShooter();
        transfer = createTransfer();
        intake = createIntake();
        shootCoordinator = new ShootCoordinator(shooter, transfer);
        Logger.recordOutput("mechanismPoses", new Pose3d[0]);

        AutoNamedCommands.registerAll(drive, shooter, transfer, intake, shootCoordinator);
        autoSelector = new AutoSelector(AutoRoutines.create(drive));

        configureCommandTelemetry();
        configureBindings();
    }

    public Command getAutonomousCommand() {
        return autoSelector.getSelectedCommand();
    }

    public void robotPeriodic() {
        updateCommandLoggingOutputs(Timer.getFPGATimestamp());
    }

    public void disabledInit() {
        schedule("mode.disabledInit.stopManipulators", stopManipulatorsCommand());
    }

    public void autonomousInit() {
        schedule("mode.autonomousInit.selectedAuto", getAutonomousCommand());
        scheduleHomingAndBackground("mode.autonomousInit");
    }

    public void teleopInit() {
        cancelAllCommands("mode.teleopInit");
        scheduleHomingAndBackground("mode.teleopInit");
    }

    public void teleopExit() {
        schedule("mode.teleopExit.stopManipulators", stopManipulatorsCommand());
    }

    public void testInit() {
        cancelAllCommands("mode.testInit");
    }

    public void simulationPeriodic() {
        if (fuelSim == null) {
            fuelSim = new FuelSim();
        }
        fuelSim.update(
                drive.getPose(),
                shooter.getMeasuredAverageShooterRpm(),
                shooter.getTargetHoodAngleRad(),
                shooter.isKickerActive(),
                0.02);
    }

    private void scheduleHomingAndBackground(String source) {
        schedule(source + ".intakeHomeThenBackground",
                intake.homeCommand().andThen(intake.backgroundCommand()).withName("IntakeHomeThenBackground"));
        schedule(source + ".shooterHomeThenBackground",
                shooter.homeCommand().andThen(shooter.backgroundCommand()).withName("ShooterHomeThenBackground"));
    }

    private void configureCommandTelemetry() {
        CommandScheduler scheduler = CommandScheduler.getInstance();
        scheduler.onCommandInitialize(this::handleCommandInitialize);
        scheduler.onCommandFinish(this::handleCommandFinish);
        scheduler.onCommandInterrupt(this::handleCommandInterrupt);
        updateCommandLoggingOutputs(Timer.getFPGATimestamp());
    }

    private void bindOnTrue(Trigger trigger, String source, Command command) {
        trigger.onTrue(withCommandSource(source, command));
    }

    private void bindWhileTrue(Trigger trigger, String source, Command command) {
        trigger.whileTrue(withCommandSource(source, command));
    }

    private Command withCommandSource(String source, Command command) {
        commandSources.put(command, normalizeCommandSource(source));
        return command;
    }

    private void schedule(String source, Command... commands) {
        String normalizedSource = normalizeCommandSource(source);
        List<Command> commandsToSchedule = new ArrayList<>();
        List<String> commandNames = new ArrayList<>();

        for (Command command : commands) {
            if (command == null) {
                continue;
            }
            Command trackedCommand = withCommandSource(normalizedSource, command);
            commandsToSchedule.add(trackedCommand);
            commandNames.add(normalizeCommandName(trackedCommand));
        }

        if (commandsToSchedule.isEmpty()) {
            return;
        }

        recordCommandEvent("REQUEST source=" + normalizedSource + " commands=" + String.join(", ", commandNames));
        Logger.recordOutput("commands/lastSchedule/source", normalizedSource);
        Logger.recordOutput("commands/lastSchedule/commands", commandNames.toArray(String[]::new));
        CommandScheduler.getInstance().schedule(commandsToSchedule.toArray(Command[]::new));
    }

    private void cancelAllCommands(String source) {
        String normalizedSource = normalizeCommandSource(source);
        recordCommandEvent("CANCEL_ALL source=" + normalizedSource);
        Logger.recordOutput("commands/lastCancelAllSource", normalizedSource);
        CommandScheduler.getInstance().cancelAll();
    }

    private void handleCommandInitialize(Command command) {
        double nowSec = Timer.getFPGATimestamp();
        int runId = nextCommandRunId++;
        commandStartTimesSec.put(command, nowSec);
        commandRunIds.put(command, runId);

        String commandName = normalizeCommandName(command);
        String source = commandSources.getOrDefault(command, UNKNOWN_COMMAND_SOURCE);
        String requirements = getRequirementsSummary(command);
        String commandKey = sanitizeCommandKey(commandName);

        Logger.recordOutput("commands/" + commandName, true);
        Logger.recordOutput("commands/byName/" + commandKey + "/running", true);
        Logger.recordOutput("commands/byName/" + commandKey + "/activeInstances", countRunningInstances(commandName));
        Logger.recordOutput("commands/lastStarted/name", commandName);
        Logger.recordOutput("commands/lastStarted/source", source);
        Logger.recordOutput("commands/lastStarted/runId", runId);
        Logger.recordOutput("commands/lastStarted/requirements", requirements);

        recordCommandEvent(String.format(
                Locale.US,
                "START run=%d name=%s source=%s requirements=%s",
                runId,
                commandName,
                source,
                requirements));
        updateCommandLoggingOutputs(nowSec);
    }

    private void handleCommandFinish(Command command) {
        handleCommandEnd(command, false);
    }

    private void handleCommandInterrupt(Command command) {
        handleCommandEnd(command, true);
    }

    private void handleCommandEnd(Command command, boolean interrupted) {
        double nowSec = Timer.getFPGATimestamp();
        Double startTimeSec = commandStartTimesSec.remove(command);
        Integer runId = commandRunIds.remove(command);

        String commandName = normalizeCommandName(command);
        String source = commandSources.getOrDefault(command, UNKNOWN_COMMAND_SOURCE);
        String commandKey = sanitizeCommandKey(commandName);
        double durationSec = startTimeSec == null ? 0.0 : nowSec - startTimeSec;
        boolean stillRunning = isCommandNameRunning(commandName);

        Logger.recordOutput("commands/" + commandName, stillRunning);
        Logger.recordOutput("commands/byName/" + commandKey + "/running", stillRunning);
        Logger.recordOutput("commands/byName/" + commandKey + "/activeInstances", countRunningInstances(commandName));
        Logger.recordOutput("commands/lastEnded/name", commandName);
        Logger.recordOutput("commands/lastEnded/source", source);
        Logger.recordOutput("commands/lastEnded/runId", runId == null ? -1 : runId);
        Logger.recordOutput("commands/lastEnded/durationSec", durationSec);
        Logger.recordOutput("commands/lastEnded/interrupted", interrupted);

        recordCommandEvent(String.format(
                Locale.US,
                "%s run=%d name=%s source=%s duration=%.3fs",
                interrupted ? "INTERRUPT" : "FINISH",
                runId == null ? -1 : runId,
                commandName,
                source,
                durationSec));
        updateCommandLoggingOutputs(nowSec);
    }

    private void updateCommandLoggingOutputs(double nowSec) {
        List<Map.Entry<Command, Double>> runningEntries = new ArrayList<>(commandStartTimesSec.entrySet());
        runningEntries.sort(Map.Entry.comparingByValue(Comparator.naturalOrder()));

        List<String> runningNames = new ArrayList<>();
        List<String> runningInstances = new ArrayList<>();

        for (Map.Entry<Command, Double> entry : runningEntries) {
            Command command = entry.getKey();
            String name = normalizeCommandName(command);
            if (!runningNames.contains(name)) {
                runningNames.add(name);
            }
            runningInstances.add(String.format(
                    Locale.US,
                    "run=%d name=%s source=%s elapsed=%.3fs requirements=%s",
                    commandRunIds.getOrDefault(command, -1),
                    name,
                    commandSources.getOrDefault(command, UNKNOWN_COMMAND_SOURCE),
                    nowSec - entry.getValue(),
                    getRequirementsSummary(command)));
        }

        Logger.recordOutput("commands/running/count", runningEntries.size());
        Logger.recordOutput("commands/running/names", runningNames.toArray(String[]::new));
        Logger.recordOutput("commands/running/instances", runningInstances.toArray(String[]::new));
        Logger.recordOutput("commands/recentEvents", recentCommandEvents.toArray(String[]::new));
    }

    private int countRunningInstances(String commandName) {
        int count = 0;
        for (Command runningCommand : commandStartTimesSec.keySet()) {
            if (commandName.equals(normalizeCommandName(runningCommand))) {
                count++;
            }
        }
        return count;
    }

    private boolean isCommandNameRunning(String commandName) {
        return countRunningInstances(commandName) > 0;
    }

    private static String getRequirementsSummary(Command command) {
        List<String> requirementNames = new ArrayList<>();
        for (var requirement : command.getRequirements()) {
            String requirementName = requirement.getName();
            if (requirementName == null || requirementName.isBlank()) {
                requirementName = requirement.getClass().getSimpleName();
            }
            requirementNames.add(requirementName);
        }
        requirementNames.sort(String::compareTo);
        return requirementNames.isEmpty() ? "[]" : "[" + String.join(", ", requirementNames) + "]";
    }

    private void recordCommandEvent(String event) {
        String eventWithTimestamp = String.format(Locale.US, "%.3fs %s", Timer.getFPGATimestamp(), event);
        recentCommandEvents.addLast(eventWithTimestamp);
        while (recentCommandEvents.size() > COMMAND_EVENT_HISTORY_LIMIT) {
            recentCommandEvents.removeFirst();
        }
        Logger.recordOutput("commands/lastEvent", eventWithTimestamp);
    }

    private static String normalizeCommandName(Command command) {
        if (command == null) {
            return "UnnamedCommand";
        }
        String name = command.getName();
        if (name == null || name.isBlank()) {
            String fallbackName = command.getClass().getSimpleName();
            if (fallbackName == null || fallbackName.isBlank()) {
                return "UnnamedCommand";
            }
            return fallbackName;
        }
        return name;
    }

    private static String normalizeCommandSource(String source) {
        return source == null || source.isBlank() ? UNKNOWN_COMMAND_SOURCE : source;
    }

    private static String sanitizeCommandKey(String key) {
        StringBuilder sanitized = new StringBuilder(key.length());
        for (int i = 0; i < key.length(); i++) {
            char character = key.charAt(i);
            if (Character.isLetterOrDigit(character) || character == '_' || character == '-') {
                sanitized.append(character);
            } else {
                sanitized.append('_');
            }
        }
        return sanitized.isEmpty() ? "unnamed" : sanitized.toString();
    }

    private static Drive createDrive() {
        boolean enabled = Constants.isMechanismEnabled(Constants.Mechanism.DRIVE);
        return switch (RobotType.MODE) {
            case REAL -> enabled
                    ? new Drive(
                            new GyroIOPigeon2(),
                            new ModuleIOTalonFX(TunerConstants.FrontLeft),
                            new ModuleIOTalonFX(TunerConstants.FrontRight),
                            new ModuleIOTalonFX(TunerConstants.BackLeft),
                            new ModuleIOTalonFX(TunerConstants.BackRight))
                    : new Drive(
                            new GyroIO() {},
                            new ModuleIO() {}, new ModuleIO() {},
                            new ModuleIO() {}, new ModuleIO() {});
            case SIMULATION -> {
                GyroIOSim gyroIOSim = new GyroIOSim(Drive.getModuleTranslations());
                Drive d = new Drive(
                        gyroIOSim,
                        new ModuleIOSim(TunerConstants.FrontLeft),
                        new ModuleIOSim(TunerConstants.FrontRight),
                        new ModuleIOSim(TunerConstants.BackLeft),
                        new ModuleIOSim(TunerConstants.BackRight));
                gyroIOSim.setModulePositionsSupplier(d::getModulePositionsForSim);
                yield d;
            }
            case REPLAY -> new Drive(
                    new GyroIO() {},
                    new ModuleIO() {}, new ModuleIO() {},
                    new ModuleIO() {}, new ModuleIO() {});
        };
    }

    private static Shooter createShooter() {
        boolean enabled = Constants.isMechanismEnabled(Constants.Mechanism.SHOOTER);
        return new Shooter(switch (RobotType.MODE) {
            case REAL -> enabled ? new ShooterIOReal() : new ShooterIO() {};
            case SIMULATION -> new ShooterIOSim();
            case REPLAY -> new ShooterIO() {};
        });
    }

    private static Transfer createTransfer() {
        boolean enabled = Constants.isMechanismEnabled(Constants.Mechanism.TRANSFER);
        return new Transfer(switch (RobotType.MODE) {
            case REAL -> enabled ? new TransferIOReal() : new TransferIO() {};
            case SIMULATION -> new TransferIOSim();
            case REPLAY -> new TransferIO() {};
        });
    }

    private static Intake createIntake() {
        boolean enabled = Constants.isMechanismEnabled(Constants.Mechanism.INTAKE);
        return new Intake(switch (RobotType.MODE) {
            case REAL -> enabled ? new IntakeIOReal() : new IntakeIO() {};
            case SIMULATION -> new IntakeIOSim();
            case REPLAY -> new IntakeIO() {};
        });
    }

    private void configureBindings() {
        drive.setDefaultCommand(
                withCommandSource("default.driveJoystick", DriveCommands.joystickDrive(
                        drive,
                        () -> -driverController.getLeftY(),
                        () -> -driverController.getLeftX(),
                        () -> -driverController.getRightX())
                        .withName("DriveJoystickDefault")));
        shooter.setDefaultCommand(withCommandSource("default.shooterBackground", shooter.backgroundCommand()));
        intake.setDefaultCommand(withCommandSource("default.intakeBackground", intake.backgroundCommand()));

        bindOnTrue(driverController.leftBumper(), "driver.leftBumper.onTrue",
                drive.toggleSlowMode().withName("DriveToggleSlowMode"));
        bindOnTrue(driverController.back(), "driver.back.onTrue",
                DriveCommands.toggleFieldOriented(drive).withName("DriveToggleFieldOriented"));
        bindOnTrue(driverController.start(), "driver.start.onTrue",
                DriveCommands.resetOdometryAndHeading(drive).withName("DriveResetOdometryAndHeading"));

        Trigger reverseTransferTrigger = driverController.y();
        bindWhileTrue(reverseTransferTrigger, "driver.y.whileTrue", transfer.reverseCommand());

        bindOnTrue(driverController.b(), "driver.b.onTrue", intake.toggleExtendedCommand());
        Trigger intakeRollerTrigger = driverController.leftTrigger();
        bindWhileTrue(intakeRollerTrigger, "driver.leftTrigger.whileTrue", intake.spinRoller());

        bindOnTrue(
                driverController.povUp(),
                "driver.povUp.onTrue",
                Commands.runOnce(
                                () -> schedule(
                                        "driver.povUp.shooterBackground",
                                        shooter.backgroundCommand()))
                        .withName("ScheduleShooterBackground"));
        bindOnTrue(driverController.povDown(), "driver.povDown.onTrue", stopManipulatorsCommand());
        bindOnTrue(driverController.povLeft(), "driver.povLeft.onTrue", Commands.runOnce(() -> {
            if (vision == null) {
                DriverStation.reportWarning(
                        "Cannot set odometry from unified vision pose: vision is disabled.",
                        false);
                return;
            }
            Pose2d unifiedVisionPose = vision.getUnifiedRobotPoseRaw();
            if (unifiedVisionPose != null) {
                Logger.recordOutput("vision/odometryOverrideSource", "rawUnifiedVisionPose");
            } else {
                unifiedVisionPose = vision.getUnifiedRobotPose();
                Logger.recordOutput("vision/odometryOverrideSource", "filteredUnifiedVisionPose");
            }

            if (unifiedVisionPose == null) {
                DriverStation.reportWarning(
                        "Cannot set odometry from unified vision pose: no recent vision pose is available.",
                        false);
                return;
            }
            drive.setPose(unifiedVisionPose);
        }, drive).withName("DriveSetOdometryFromUnifiedVision"));

        Supplier<Pose2d> shootingPoseSupplier = createShootingPoseSupplier();
        Supplier<Shooter.MotionCompensation> motionCompensationSupplier =
                createTeleopMotionCompensationSupplier(shootingPoseSupplier);
        DoubleSupplier hubDistanceSupplier =
                () -> motionCompensationSupplier.get().compensatedDistanceMeters();
        Supplier<Rotation2d> hubHeadingSupplier =
                () -> motionCompensationSupplier.get().compensatedHeading();
        Supplier<Rotation2d> teleopAutoAlignHeadingSupplier = hubHeadingSupplier;
        BooleanSupplier teleopAimReadySupplier =
                createTeleopAimReadySupplier(teleopAutoAlignHeadingSupplier);

        Trigger rightTriggerPressed = driverController.rightTrigger();
        Trigger dashboardTuneTrigger = rightTriggerPressed.and(new Trigger(shooter::isDashboardTuningEnabled));
        Trigger dashboardTransferTuneTrigger = dashboardTuneTrigger.and(new Trigger(shooter::isDashboardFeedKickerEnabled));
        Trigger shootTrigger = rightTriggerPressed.and(new Trigger(() -> !shooter.isDashboardTuningEnabled()));
        Trigger aimTrigger = driverController.rightBumper();
        Trigger aimOnlyTrigger = aimTrigger.and(shootTrigger.negate());
        bindWhileTrue(shootTrigger, "driver.shoot.whileTrue", createTeleopShootCommand(
                () -> -driverController.getLeftY(),
                () -> -driverController.getLeftX(),
                () -> -driverController.getRightX(),
                hubDistanceSupplier,
                teleopAutoAlignHeadingSupplier,
                teleopAimReadySupplier));
        bindWhileTrue(aimOnlyTrigger, "driver.aimOnly.whileTrue", Commands.parallel(
                createTeleopAutoAlignCommand(
                        () -> -driverController.getLeftY(),
                        () -> -driverController.getLeftX(),
                        () -> -driverController.getRightX(),
                        teleopAutoAlignHeadingSupplier),
                shootCoordinator.aimForDistance(hubDistanceSupplier))
                .withName("ShooterAimOnly"));
        bindWhileTrue(
                dashboardTuneTrigger,
                "driver.dashboardTune.whileTrue",
                shooter.dashboardTuneCommand().withName("ShooterDashboardTune"));
        bindWhileTrue(
                dashboardTransferTuneTrigger,
                "driver.dashboardTuneTransfer.whileTrue",
                transfer.runCommand().withName("TransferDashboardTune"));

        bindOnTrue(godController.leftBumper(), "god.leftBumper.onTrue",
                drive.toggleSlowMode().withName("DriveToggleSlowMode"));
        bindOnTrue(godController.povDown(), "god.povDown.onTrue",
                DriveCommands.resetOdometryAndHeading(drive).withName("DriveResetOdometryAndHeading"));
        bindWhileTrue(godController.a(), "god.a.whileTrue",
                drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward).withName("DriveSysIdQuasistaticForward"));
        bindWhileTrue(godController.b(), "god.b.whileTrue",
                drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse).withName("DriveSysIdQuasistaticReverse"));
        bindWhileTrue(godController.x(), "god.x.whileTrue",
                drive.sysIdDynamic(SysIdRoutine.Direction.kForward).withName("DriveSysIdDynamicForward"));
        bindWhileTrue(godController.y(), "god.y.whileTrue",
                drive.sysIdDynamic(SysIdRoutine.Direction.kReverse).withName("DriveSysIdDynamicReverse"));
    }

    private Supplier<Pose2d> createShootingPoseSupplier() {
        // Single-source aiming pose: estimator translation + live gyro heading.
        // This avoids frame discontinuities from switching between tag and odometry heading sources.
        return () -> new Pose2d(drive.getPose().getTranslation(), drive.getRotation());
    }

    private Supplier<Shooter.MotionCompensation> createTeleopMotionCompensationSupplier(
            Supplier<Pose2d> shootingPoseSupplier) {
        final long[] cachedLoop = new long[] {Long.MIN_VALUE};
        final Shooter.MotionCompensation[] cachedCompensation = new Shooter.MotionCompensation[] {
                new Shooter.MotionCompensation(Double.NaN, Double.NaN, Double.NaN, Double.NaN, Double.NaN, null)
        };

        return () -> {
            long loop = RobotController.getFPGATime() / 20_000L;
            if (loop != cachedLoop[0]) {
                cachedLoop[0] = loop;
                cachedCompensation[0] = shooter.getMotionCompensationToHub(
                        shootingPoseSupplier.get(),
                        drive.getMeasuredChassisSpeeds());
            }
            return cachedCompensation[0];
        };
    }

    private BooleanSupplier createTeleopAimReadySupplier(
            Supplier<Rotation2d> targetHeadingSupplier) {
        final boolean[] aimReadyLatched = new boolean[] {false};
        final Rotation2d[] lastValidTargetHeading = new Rotation2d[] {null};
        final double[] lastValidTargetTimestampSec = new double[] {Double.NaN};
        return () -> {
            Rotation2d targetHeading = targetHeadingSupplier.get();
            double nowSec = Timer.getFPGATimestamp();
            if (targetHeading != null) {
                lastValidTargetHeading[0] = targetHeading;
                lastValidTargetTimestampSec[0] = nowSec;
            } else if (lastValidTargetHeading[0] != null) {
                double targetAgeSec = nowSec - lastValidTargetTimestampSec[0];
                if (Double.isFinite(targetAgeSec)
                        && targetAgeSec <= Constants.SHOOTING_AIM_TARGET_HOLD_SEC) {
                    targetHeading = lastValidTargetHeading[0];
                }
            }

            boolean targetAvailable = targetHeading != null;
            Rotation2d robotHeading = drive.getRotation();
            Logger.recordOutput("Shooting/AimTargetAvailable", targetAvailable);
            Logger.recordOutput("Shooting/RobotHeadingDeg", robotHeading.getDegrees());
            if (targetHeading == null) {
                Logger.recordOutput("Shooting/TargetHeadingDeg", Double.NaN);
                Logger.recordOutput("Shooting/DesiredRobotHeadingDeg", Double.NaN);
                Logger.recordOutput("Shooting/AimErrorRad", Double.NaN);
                Logger.recordOutput("Shooting/AimErrorDeg", Double.NaN);
                Logger.recordOutput("Shooting/AimReadyLatched", false);
                aimReadyLatched[0] = false;
                return false;
            }

            Rotation2d desiredRobotHeading = targetHeading.plus(Rotation2d.kPi);
            double headingErrorRad = MathUtil.angleModulus(
                    desiredRobotHeading.minus(robotHeading).getRadians());
            Logger.recordOutput("Shooting/TargetHeadingDeg", targetHeading.getDegrees());
            Logger.recordOutput("Shooting/DesiredRobotHeadingDeg", desiredRobotHeading.getDegrees());
            Logger.recordOutput("Shooting/AimErrorRad", headingErrorRad);
            Logger.recordOutput("Shooting/AimErrorDeg", Math.toDegrees(headingErrorRad));
            double absHeadingErrorRad = Math.abs(headingErrorRad);
            if (aimReadyLatched[0]) {
                aimReadyLatched[0] = absHeadingErrorRad <= Constants.SHOOTING_AIM_RELEASE_TOLERANCE_RAD;
            } else {
                aimReadyLatched[0] = absHeadingErrorRad <= Constants.SHOOTING_AIM_TOLERANCE_RAD;
            }
            Logger.recordOutput("Shooting/AimReadyLatched", aimReadyLatched[0]);
            return aimReadyLatched[0];
        };
    }

    private Command createTeleopAutoAlignCommand(
            DoubleSupplier xSupplier,
            DoubleSupplier ySupplier,
            DoubleSupplier omegaFallbackSupplier,
            Supplier<Rotation2d> targetHeadingSupplier) {
        return DriveCommands.autoAlignToHubPose(
                drive,
                xSupplier,
                ySupplier,
                omegaFallbackSupplier,
                targetHeadingSupplier);
    }

    private Command createTeleopShootCommand(
            DoubleSupplier xSupplier,
            DoubleSupplier ySupplier,
            DoubleSupplier omegaFallbackSupplier,
            DoubleSupplier distanceMetersSupplier,
            Supplier<Rotation2d> targetHeadingSupplier,
            BooleanSupplier aimReadySupplier) {
        return Commands.parallel(
                createTeleopAutoAlignCommand(
                        xSupplier,
                        ySupplier,
                        omegaFallbackSupplier,
                        targetHeadingSupplier),
                shootCoordinator.shootForDistance(distanceMetersSupplier, aimReadySupplier))
                .withName("ShooterTriggerAimAndShoot");
    }

    private Command stopManipulatorsCommand() {
        return Commands.parallel(shooter.stopCommand(), transfer.stopCommand(), intake.stopAndRetractCommand())
                .withName("StopManipulators");
    }

}
