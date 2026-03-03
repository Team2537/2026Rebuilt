package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
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
import frc.robot.subsystems.drive.DriveConstants;
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
import frc.robot.subsystems.shooter.LaunchCalculator;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOReal;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.transfer.Transfer;
import frc.robot.subsystems.transfer.TransferIO;
import frc.robot.subsystems.transfer.TransferIOReal;
import frc.robot.subsystems.transfer.TransferIOSim;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.AutoAimHeadingConfig;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/** Owns subsystems, bindings, and autonomous orchestration. */
public final class RobotContainer {
    private static final int GOD_CONTROLLER_PORT = 5;
    private static final String DASHBOARD_ACTIONS_PREFIX = "Actions/";

    private final Drive drive;
    private final Vision vision;
    private final Shooter shooter;
    private final Transfer transfer;
    private final Intake intake;
    private final ShootCoordinator shootCoordinator;
    private final AutoSelector autoSelector;

    private final CommandXboxController driverController = new CommandXboxController(0);
    private final CommandXboxController godController;

    private final CommandTelemetry commandTelemetry = new CommandTelemetry();
    private final DashboardOverrides dashboardOverrides = new DashboardOverrides();

    private FuelSim fuelSim;

    public RobotContainer() {
        godController =
                DriverStation.isJoystickConnected(GOD_CONTROLLER_PORT)
                        ? new CommandXboxController(GOD_CONTROLLER_PORT)
                        : null;

        drive = createDrive();
        RobotState.initialize(drive);
        vision = Constants.isMechanismEnabled(Constants.Mechanism.VISION) ? new Vision(RobotState.getInstance()) : null;
        shooter = createShooter();
        transfer = createTransfer();
        intake = createIntake();
        shootCoordinator = new ShootCoordinator(shooter, transfer);
        Logger.recordOutput("mechanismPoses", new Pose3d[0]);

        AutoNamedCommands.registerAll(drive, shooter, transfer, intake, shootCoordinator);
        autoSelector = new AutoSelector(AutoRoutines.create(drive));

        commandTelemetry.configure();
        configureBindings();
        dashboardOverrides.init();
        publishDashboardSysIdCommands();
        publishDashboardActionCommands();
        dashboardOverrides.publishCommands(commandTelemetry);
    }

    public Command getAutonomousCommand() {
        return autoSelector.getSelectedCommand();
    }

    public void robotPeriodic() {
        commandTelemetry.periodic();
        dashboardOverrides.periodic(vision);
    }

    public void disabledInit() {
        commandTelemetry.schedule("mode.disabledInit.stopManipulators", stopManipulatorsCommand());
    }

    public void autonomousInit() {
        Command selectedAuto = getAutonomousCommand();
        commandTelemetry.schedule("mode.autonomousInit.selectedAuto", selectedAuto);
        if (!commandUsesIntakeOrShooter(selectedAuto)) {
            scheduleHomingAndBackground("mode.autonomousInit");
        }
    }

    public void teleopInit() {
        commandTelemetry.cancelAllCommands("mode.teleopInit");
        scheduleHomingAndBackground("mode.teleopInit");
    }

    public void teleopExit() {
        commandTelemetry.schedule("mode.teleopExit.stopManipulators", stopManipulatorsCommand());
    }

    public void testInit() {
        commandTelemetry.cancelAllCommands("mode.testInit");
    }

    public void simulationPeriodic() {
        if (fuelSim == null) {
            fuelSim = new FuelSim();
        }
        fuelSim.update(
                RobotState.getInstance().getPose(),
                shooter.getMeasuredAverageShooterRpm(),
                shooter.getTargetHoodAngleRad(),
                shooter.isKickerActive(),
                0.02);
    }

    private void scheduleHomingAndBackground(String source) {
        commandTelemetry.schedule(source + ".intakeHomeThenBackground",
                intake.homeCommand().andThen(intake.backgroundCommand()).withName("IntakeHomeThenBackground"));
        commandTelemetry.schedule(source + ".shooterHomeThenBackground",
                shooter.homeCommand().andThen(shooter.backgroundCommand()).withName("ShooterHomeThenBackground"));
    }

    private boolean commandUsesIntakeOrShooter(Command command) {
        if (command == null) {
            return false;
        }
        return command.getRequirements().contains(intake) || command.getRequirements().contains(shooter);
    }

    private void bindOnTrue(Trigger trigger, String source, Command command) {
        trigger.onTrue(commandTelemetry.withCommandSource(source, command));
    }

    private void bindWhileTrue(Trigger trigger, String source, Command command) {
        trigger.whileTrue(commandTelemetry.withCommandSource(source, command));
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
                commandTelemetry.withCommandSource("default.driveJoystick", DriveCommands.joystickDrive(
                        drive,
                        () -> driverController.getLeftY(),
                        () -> driverController.getLeftX(),
                        () -> -driverController.getRightX())
                        .withName("DriveJoystickDefault")));
        intake.setDefaultCommand(commandTelemetry.withCommandSource("default.intakeBackground", intake.backgroundCommand()));

        bindOnTrue(driverController.leftBumper(), "driver.leftBumper.onTrue",
                drive.toggleSlowMode().withName("DriveToggleSlowMode"));
        bindOnTrue(driverController.back(), "driver.back.onTrue",
                DriveCommands.toggleFieldOriented(drive).withName("DriveToggleFieldOriented"));
        bindOnTrue(driverController.start(), "driver.start.onTrue",
                DriveCommands.resetOdometryAndHeading(drive).withName("DriveResetOdometryAndHeading"));
        bindOnTrue(driverController.rightStick(), "driver.rightStick.onTrue",
                DriveCommands.headingSnap(drive).withName("DriveHeadingSnap"));

        Trigger reverseTransferTrigger = driverController.y();
        bindWhileTrue(reverseTransferTrigger, "driver.y.whileTrue", transfer.reverseCommand());

        bindOnTrue(driverController.b(), "driver.b.onTrue", intake.toggleExtendedCommand());
        Trigger slowRetractTrigger = driverController.x();
        bindWhileTrue(slowRetractTrigger, "driver.x.whileTrue", intake.slowRetractCommand());
        Trigger intakeRollerTrigger = driverController.leftTrigger();
        bindWhileTrue(intakeRollerTrigger, "driver.leftTrigger.whileTrue", intake.spinRoller());

        bindOnTrue(
                driverController.povUp(),
                "driver.povUp.onTrue",
                Commands.runOnce(
                                () -> commandTelemetry.schedule(
                                        "driver.povUp.shooterBackground",
                                        shooter.backgroundCommand()))
                        .withName("ScheduleShooterBackground"));
        bindOnTrue(driverController.povDown(), "driver.povDown.onTrue", stopManipulatorsCommand());
        bindOnTrue(
                driverController.povLeft(),
                "driver.povLeft.onTrue",
                createSetOdometryFromUnifiedVisionCommand());

        Supplier<Pose2d> shootingPoseSupplier = createShootingPoseSupplier();
        Supplier<LaunchCalculator.MotionCompensation> motionCompensationSupplier =
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
        bindWhileTrue(shootTrigger, "driver.shoot.whileTrue", withConstraintProfile(
                createTeleopSelectedShootCommand(
                        () -> driverController.getLeftY(),
                        () -> driverController.getLeftX(),
                        () -> -driverController.getRightX(),
                        hubDistanceSupplier,
                        teleopAutoAlignHeadingSupplier,
                        teleopAimReadySupplier),
                DriveConstants.ConstraintProfile.SHOOTING_ON_MOVE).withName("ShooterTriggerSelectedMode"));
        bindWhileTrue(aimOnlyTrigger, "driver.aimOnly.whileTrue", createTeleopSelectedAimOnlyCommand(
                () -> driverController.getLeftY(),
                () -> driverController.getLeftX(),
                () -> -driverController.getRightX(),
                hubDistanceSupplier,
                teleopAutoAlignHeadingSupplier));
        bindWhileTrue(
                dashboardTuneTrigger,
                "driver.dashboardTune.whileTrue",
                shooter.dashboardTuneCommand().withName("ShooterDashboardTune"));
        bindWhileTrue(
                dashboardTransferTuneTrigger,
                "driver.dashboardTuneTransfer.whileTrue",
                transfer.runCommand().withName("TransferDashboardTune"));

        if (godController != null) {
            bindOnTrue(godController.leftBumper(), "god.leftBumper.onTrue",
                    drive.toggleSlowMode().withName("DriveToggleSlowMode"));
            bindOnTrue(godController.povDown(), "god.povDown.onTrue",
                    DriveCommands.resetOdometryAndHeading(drive).withName("DriveResetOdometryAndHeading"));
            bindWhileTrue(godController.a(), "god.a.whileTrue",
                    drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward)
                            .withName("DriveSysIdQuasistaticForward"));
            bindWhileTrue(godController.b(), "god.b.whileTrue",
                    drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse)
                            .withName("DriveSysIdQuasistaticReverse"));
            bindWhileTrue(godController.x(), "god.x.whileTrue",
                    drive.sysIdDynamic(SysIdRoutine.Direction.kForward).withName("DriveSysIdDynamicForward"));
            bindWhileTrue(godController.y(), "god.y.whileTrue",
                    drive.sysIdDynamic(SysIdRoutine.Direction.kReverse).withName("DriveSysIdDynamicReverse"));
        }
    }

    private void publishDashboardSysIdCommands() {
        SmartDashboard.putData(
                "Drive/SysId/WheelRadius",
                commandTelemetry.withCommandSource(
                        "dashboard.driveSysId.wheelRadius",
                        DriveCommands.wheelRadiusCharacterization(drive)
                                .withName("DriveWheelRadiusCharacterization")));
        SmartDashboard.putData(
                "Shooter/SysId/QuasistaticForward",
                commandTelemetry.withCommandSource(
                        "dashboard.shooterSysId.quasistaticForward",
                        shooter.sysIdQuasistatic(SysIdRoutine.Direction.kForward)
                                .withName("ShooterSysIdQuasistaticForward")));
        SmartDashboard.putData(
                "Shooter/SysId/QuasistaticReverse",
                commandTelemetry.withCommandSource(
                        "dashboard.shooterSysId.quasistaticReverse",
                        shooter.sysIdQuasistatic(SysIdRoutine.Direction.kReverse)
                                .withName("ShooterSysIdQuasistaticReverse")));
        SmartDashboard.putData(
                "Shooter/SysId/DynamicForward",
                commandTelemetry.withCommandSource(
                        "dashboard.shooterSysId.dynamicForward",
                        shooter.sysIdDynamic(SysIdRoutine.Direction.kForward)
                                .withName("ShooterSysIdDynamicForward")));
        SmartDashboard.putData(
                "Shooter/SysId/DynamicReverse",
                commandTelemetry.withCommandSource(
                        "dashboard.shooterSysId.dynamicReverse",
                        shooter.sysIdDynamic(SysIdRoutine.Direction.kReverse)
                                .withName("ShooterSysIdDynamicReverse")));
    }

    private void publishDashboardActionCommands() {
        putDashboardCommand(
                DASHBOARD_ACTIONS_PREFIX + "IntakeHome",
                "dashboard.actions.intakeHome",
                intake.homeCommand().withName("DashboardIntakeHome"));
        putDashboardCommand(
                DASHBOARD_ACTIONS_PREFIX + "ShooterHome",
                "dashboard.actions.shooterHome",
                shooter.homeCommand().withName("DashboardShooterHome"));
        putDashboardCommand(
                DASHBOARD_ACTIONS_PREFIX + "StopManipulators",
                "dashboard.actions.stopManipulators",
                stopManipulatorsCommand().withName("DashboardStopManipulators"));
        putDashboardCommand(
                DASHBOARD_ACTIONS_PREFIX + "TransferRun",
                "dashboard.actions.transferRun",
                transfer.runCommand().withName("DashboardTransferRun"));
        putDashboardCommand(
                DASHBOARD_ACTIONS_PREFIX + "ShooterStop",
                "dashboard.actions.shooterStop",
                shooter.stopCommand().withName("DashboardShooterStop"));
        putDashboardCommand(
                DASHBOARD_ACTIONS_PREFIX + "DriveStopWithX",
                "dashboard.actions.driveStopWithX",
                Commands.runOnce(drive::stopWithX, drive).withName("DashboardDriveStopWithX"));
        putDashboardCommand(
                DASHBOARD_ACTIONS_PREFIX + "DriveResetOdometryAndHeading",
                "dashboard.actions.driveResetOdometryAndHeading",
                DriveCommands.resetOdometryAndHeading(drive).withName("DashboardDriveResetOdometryAndHeading"));
        putDashboardCommand(
                DASHBOARD_ACTIONS_PREFIX + "DriveSetOdometryFromUnifiedVision",
                "dashboard.actions.driveSetOdometryFromUnifiedVision",
                createSetOdometryFromUnifiedVisionCommand().withName("DashboardDriveSetOdometryFromUnifiedVision"));
    }

    private void putDashboardCommand(String dashboardKey, String source, Command command) {
        SmartDashboard.putData(dashboardKey, commandTelemetry.withCommandSource(source, command));
    }

    private Supplier<Pose2d> createShootingPoseSupplier() {
        // Single-source aiming pose: estimator translation + live gyro heading.
        // This avoids frame discontinuities from switching between tag and odometry heading sources.
        RobotState robotState = RobotState.getInstance();
        return () -> new Pose2d(robotState.getPose().getTranslation(), robotState.getRotation());
    }

    private Supplier<LaunchCalculator.MotionCompensation> createTeleopMotionCompensationSupplier(
            Supplier<Pose2d> shootingPoseSupplier) {
        // Use the periodic cycle counter instead of FPGA time division so the cache
        // stays correct even if loop timing jitters.
        final int[] cachedCycle = new int[] {Integer.MIN_VALUE};
        final LaunchCalculator.MotionCompensation[] cachedCompensation = new LaunchCalculator.MotionCompensation[] {
                new LaunchCalculator.MotionCompensation(Double.NaN, Double.NaN, Double.NaN, Double.NaN, Double.NaN, null)
        };

        return () -> {
            if (commandTelemetry.getCycle() != cachedCycle[0]) {
                cachedCycle[0] = commandTelemetry.getCycle();
                cachedCompensation[0] = shooter.getMotionCompensationToHub(
                        shootingPoseSupplier.get(),
                        RobotState.getInstance().getMeasuredChassisSpeeds());
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
                        && targetAgeSec <= AutoAimHeadingConfig.TARGET_HOLD_SEC) {
                    targetHeading = lastValidTargetHeading[0];
                }
            }

            boolean targetAvailable = targetHeading != null;
            Rotation2d robotHeading = RobotState.getInstance().getRotation();
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
                aimReadyLatched[0] = absHeadingErrorRad <= AutoAimHeadingConfig.AIM_RELEASE_TOLERANCE_RAD;
            } else {
                aimReadyLatched[0] = absHeadingErrorRad <= AutoAimHeadingConfig.AIM_TOLERANCE_RAD;
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
                shootCoordinator.shootForDistance(distanceMetersSupplier, aimReadySupplier),
                intake.smartRetractDuringShootCommand(shootCoordinator::isActivelyFeeding))
                .withName("ShooterTriggerAimAndShoot");
    }

    private Command createTeleopSelectedShootCommand(
            DoubleSupplier xSupplier,
            DoubleSupplier ySupplier,
            DoubleSupplier omegaFallbackSupplier,
            DoubleSupplier distanceMetersSupplier,
            Supplier<Rotation2d> targetHeadingSupplier,
            BooleanSupplier aimReadySupplier) {
        return Commands.either(
                createTeleopOverrideAutoAimShootCommand(xSupplier, ySupplier, omegaFallbackSupplier),
                createTeleopShootCommand(
                        xSupplier,
                        ySupplier,
                        omegaFallbackSupplier,
                        distanceMetersSupplier,
                        targetHeadingSupplier,
                        aimReadySupplier),
                dashboardOverrides::isAutoAimEnabled)
                .withName("ShooterTriggerSelectedMode");
    }

    private Command createTeleopOverrideAutoAimShootCommand(
            DoubleSupplier xSupplier,
            DoubleSupplier ySupplier,
            DoubleSupplier omegaSupplier) {
        return Commands.parallel(
                DriveCommands.joystickDrive(
                        drive,
                        xSupplier,
                        ySupplier,
                        omegaSupplier)
                        .withName("DriveJoystickOverrideAutoAim"),
                shootCoordinator.shootForDistance(dashboardOverrides::getAimDistanceMeters, () -> true)
                        .withName("ShooterShootOverrideDistance"),
                intake.smartRetractDuringShootCommand(shootCoordinator::isActivelyFeeding))
                .withName("ShooterTriggerOverrideAutoAimShoot");
    }

    private Command createTeleopSelectedAimOnlyCommand(
            DoubleSupplier xSupplier,
            DoubleSupplier ySupplier,
            DoubleSupplier omegaFallbackSupplier,
            DoubleSupplier distanceMetersSupplier,
            Supplier<Rotation2d> targetHeadingSupplier) {
        Command normalAimOnly = Commands.parallel(
                createTeleopAutoAlignCommand(
                        xSupplier,
                        ySupplier,
                        omegaFallbackSupplier,
                        targetHeadingSupplier),
                shootCoordinator.aimForDistance(distanceMetersSupplier))
                .withName("ShooterAimOnly");
        Command overrideAimOnly = shootCoordinator.aimForDistance(dashboardOverrides::getAimDistanceMeters)
                .withName("ShooterAimOnlyOverrideDistance");
        return Commands.either(overrideAimOnly, normalAimOnly, dashboardOverrides::isAutoAimEnabled)
                .withName("ShooterAimOnlySelectedMode");
    }

    private Command withConstraintProfile(Command command, DriveConstants.ConstraintProfile profile) {
        return command
                .beforeStarting(() -> drive.setConstraintProfileActive(profile, true))
                .finallyDo((interrupted) -> drive.setConstraintProfileActive(profile, false));
    }

    private Command stopManipulatorsCommand() {
        return Commands.parallel(shooter.stopCommand(), transfer.stopCommand(), intake.stopAndRetractCommand())
                .withName("StopManipulators");
    }

    private Command createSetOdometryFromUnifiedVisionCommand() {
        return Commands.runOnce(() -> {
            if (vision == null) {
                DriverStation.reportWarning(
                        "Cannot set odometry from unified vision pose: vision is disabled.",
                        false);
                return;
            }
            Pose2d unifiedVisionPose = vision.getUnifiedRobotPose();
            String source = "unifiedVisionPose";
            if (unifiedVisionPose == null) {
                unifiedVisionPose = vision.getUnifiedRobotPoseRaw();
                source = "rawUnifiedVisionPoseFallback";
            }
            if (unifiedVisionPose == null) {
                DriverStation.reportWarning(
                        "Cannot set odometry from unified vision pose: no recent vision pose is available.",
                        false);
                return;
            }

            Pose2d poseToApply = unifiedVisionPose;
            String sourceTag = source;
            CommandScheduler.getInstance().schedule(
                    Commands.runOnce(() -> {
                        Logger.recordOutput("vision/odometryOverrideSource", sourceTag);
                        RobotState.getInstance().setPose(poseToApply);
                    }, drive).withName("DriveSetOdometryFromUnifiedVision"));
        }).withName("DriveSetOdometryFromUnifiedVisionDispatch");
    }
}
