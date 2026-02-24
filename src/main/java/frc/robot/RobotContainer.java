package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
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
import frc.robot.util.FieldConstants;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

/** Owns subsystems, bindings, and autonomous orchestration. */
public final class RobotContainer {
    private static final int GOD_CONTROLLER_PORT = 5;

    private final Drive drive;
    private final Vision vision;
    private final Shooter shooter;
    private final Transfer transfer;
    private final Intake intake;
    private final AutoSelector autoSelector;

    private final CommandXboxController driverController = new CommandXboxController(0);
    private final CommandXboxController godController = new CommandXboxController(GOD_CONTROLLER_PORT);

    private FuelSim fuelSim;

    public RobotContainer() {
        drive = createDrive();
        vision = Constants.isMechanismEnabled(Constants.Mechanism.VISION) ? new Vision(drive) : null;
        shooter = createShooter();
        transfer = createTransfer();
        intake = createIntake();

        AutoNamedCommands.registerAll(drive, shooter, transfer, intake);
        autoSelector = new AutoSelector(AutoRoutines.create(drive));

        configureBindings();
    }

    public Command getAutonomousCommand() {
        return autoSelector.getSelectedCommand();
    }

    public void robotPeriodic() {
        shooter.getMotionCompensationToHub(drive.getPose(), drive.getMeasuredChassisSpeeds());
        MechanismVisualizer.updatePoses();
    }

    public void disabledInit() {
        schedule(stopManipulatorsCommand());
    }

    public void autonomousInit() {
        schedule(getAutonomousCommand());
        scheduleHomingAndBackground();
    }

    public void teleopInit() {
        CommandScheduler.getInstance().cancelAll();
        scheduleHomingAndBackground();
    }

    public void teleopExit() {
        schedule(stopManipulatorsCommand());
    }

    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    public void simulationPeriodic() {
        if (fuelSim == null) {
            fuelSim = new FuelSim();
        }
        fuelSim.update(
                drive.getPose(),
                shooter.getTargetAverageShooterRpm(),
                shooter.getTargetHoodAngleRad(),
                shooter.isKickerActive(),
                0.02);
    }

    private void scheduleHomingAndBackground() {
        schedule(intake.homeCommand().andThen(intake.backgroundCommand()).withName("IntakeHomeThenBackground"));
        schedule(shooter.homeCommand().andThen(shooter.backgroundCommand()).withName("ShooterHomeThenBackground"));
        schedule(transfer.backgroundCommand());
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
                DriveCommands.joystickDrive(
                        drive,
                        () -> driverController.getLeftY(),
                        driverController::getLeftX,
                        () -> -driverController.getRightX())
                        .withName("DriveJoystickDefault"));
        shooter.setDefaultCommand(shooter.idleCommand());
        intake.setDefaultCommand(intake.backgroundCommand());

        driverController.leftBumper().onTrue(drive.toggleSlowMode().withName("DriveToggleSlowMode"));
        driverController.back().onTrue(DriveCommands.toggleFieldOriented(drive).withName("DriveToggleFieldOriented"));
        driverController.start().onTrue(
                DriveCommands.resetOdometryAndHeading(drive).withName("DriveResetOdometryAndHeading"));

        Trigger reverseTransferTrigger = driverController.y();
        reverseTransferTrigger.whileTrue(transfer.reverseCommand());
        reverseTransferTrigger.whileFalse(transfer.backgroundCommand());

        driverController.b().onTrue(intake.toggleExtendedCommand());
        Trigger intakeRollerTrigger = driverController.leftTrigger();
        intakeRollerTrigger.onTrue(intake.spinRoller());
        intakeRollerTrigger.onFalse(intake.backgroundCommand());

        driverController.povUp()
                .onTrue(Commands.runOnce(this::scheduleBackgroundManipulators).withName("ScheduleBackgroundManipulators"));
        driverController.povDown().onTrue(stopManipulatorsCommand());
        driverController.povLeft().onTrue(Commands.runOnce(() -> {
            if (vision == null) {
                DriverStation.reportWarning(
                        "Cannot set odometry from unified vision pose: vision is disabled.",
                        false);
                return;
            }
            Pose2d unifiedVisionPose = vision.getUnifiedRobotPose();
            if (unifiedVisionPose == null) {
                DriverStation.reportWarning(
                        "Cannot set odometry from unified vision pose: no recent vision pose is available.",
                        false);
                return;
            }
            drive.setPose(unifiedVisionPose);
        }, drive).withName("DriveSetOdometryFromUnifiedVision"));

        DoubleSupplier hubDistanceSupplier = () -> shooter.getMotionCompensatedHubDistanceMeters(
                drive.getPose(), drive.getMeasuredChassisSpeeds());
        Supplier<Rotation2d> hubHeadingSupplier = () -> shooter.getMotionCompensatedHubHeading(
                drive.getPose(), drive.getMeasuredChassisSpeeds());
        Supplier<Rotation2d> hubTagOnlyHeadingSupplier = () -> getHubFacingHeading(
                vision != null ? vision.getHubTagRobotPose() : null);

        Trigger rightTriggerPressed = driverController.rightTrigger();
        Trigger dashboardTuneTrigger = rightTriggerPressed.and(new Trigger(shooter::isDashboardTuningEnabled));
        Trigger shootTrigger = rightTriggerPressed.and(new Trigger(() -> !shooter.isDashboardTuningEnabled()));
        Trigger aimTrigger = driverController.rightBumper();
        Trigger aimOnlyTrigger = aimTrigger.and(shootTrigger.negate());
        Trigger shootOnlyTrigger = shootTrigger.and(aimTrigger.negate());
        Trigger aimAndShootTrigger = aimTrigger.and(shootTrigger);
        Trigger shooterControlsReleased = aimTrigger.or(rightTriggerPressed).negate();

        shootOnlyTrigger.whileTrue(shooter.shoot(hubDistanceSupplier));
        aimOnlyTrigger.whileTrue(Commands.parallel(
                createTeleopAutoAlignCommand(
                        () -> driverController.getLeftY(),
                        driverController::getLeftX,
                        () -> -driverController.getRightX(),
                        hubHeadingSupplier,
                        hubTagOnlyHeadingSupplier),
                shooter.aimForDistance(hubDistanceSupplier))
                .withName("ShooterAimOnly"));
        aimAndShootTrigger.whileTrue(Commands.parallel(
                createTeleopAutoAlignCommand(
                        () -> driverController.getLeftY(),
                        driverController::getLeftX,
                        () -> -driverController.getRightX(),
                        hubHeadingSupplier,
                        hubTagOnlyHeadingSupplier),
                shooter.shoot(hubDistanceSupplier))
                .withName("ShooterAimAndShoot"));
        dashboardTuneTrigger.whileTrue(shooter.dashboardTuneCommand().withName("ShooterDashboardTune"));
        shooterControlsReleased.onTrue(
                Commands.runOnce(this::scheduleShooterBackgroundIfIdle).withName("ScheduleShooterBackgroundIfIdle"));

        godController.leftBumper().onTrue(drive.toggleSlowMode().withName("DriveToggleSlowMode"));
        godController.povDown().onTrue(DriveCommands.resetOdometryAndHeading(drive).withName("DriveResetOdometryAndHeading"));
        godController.a().whileTrue(
                drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward).withName("DriveSysIdQuasistaticForward"));
        godController.b().whileTrue(
                drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse).withName("DriveSysIdQuasistaticReverse"));
        godController.x().whileTrue(
                drive.sysIdDynamic(SysIdRoutine.Direction.kForward).withName("DriveSysIdDynamicForward"));
        godController.y().whileTrue(
                drive.sysIdDynamic(SysIdRoutine.Direction.kReverse).withName("DriveSysIdDynamicReverse"));
    }

    private Command createTeleopAutoAlignCommand(
            DoubleSupplier xSupplier,
            DoubleSupplier ySupplier,
            DoubleSupplier omegaFallbackSupplier,
            Supplier<Rotation2d> hubHeadingSupplier,
            Supplier<Rotation2d> hubTagOnlyHeadingSupplier) {
        return switch (Constants.TELEOP_AUTO_ALIGN_MODE) {
            case HUB_TAGS_ONLY -> {
                if (vision == null) {
                    throw new IllegalStateException(
                            "TELEOP_AUTO_ALIGN_MODE is HUB_TAGS_ONLY but VISION mechanism is disabled.");
                }
                yield DriveCommands.autoAlignToHubPose(
                        drive, xSupplier, ySupplier, omegaFallbackSupplier, hubTagOnlyHeadingSupplier);
            }
            case POSE_ODOMETRY -> DriveCommands.autoAlignToHubPose(
                    drive, xSupplier, ySupplier, omegaFallbackSupplier, hubHeadingSupplier);
        };
    }

    private static Rotation2d getHubFacingHeading(Pose2d robotPose) {
        if (robotPose == null) {
            return null;
        }
        return FieldConstants.TAG_LAYOUT.getTagPose(FieldConstants.HUB_TAG_ID)
                .map(tagPose -> {
                    Translation2d hubTarget = new Translation2d(
                            tagPose.getX() + FieldConstants.HUB_TARGET_X_OFFSET_METERS,
                            tagPose.getY());
                    Translation2d toHub = hubTarget.minus(robotPose.getTranslation());
                    if (toHub.getNorm() <= 1e-6) {
                        return null;
                    }
                    return toHub.getAngle();
                })
                .orElse(null);
    }

    private Command stopManipulatorsCommand() {
        return Commands.parallel(shooter.stopCommand(), transfer.stopCommand(), intake.stopAndRetractCommand())
                .withName("StopManipulators");
    }

    private void scheduleBackgroundManipulators() {
        schedule(transfer.backgroundCommand());
        schedule(shooter.backgroundCommand());
        schedule(intake.backgroundCommand());
    }

    private void scheduleShooterBackgroundIfIdle() {
        if (!DriverStation.isEnabled()) {
            return;
        }
        Command currentShooterCommand = shooter.getCurrentCommand();
        if (currentShooterCommand == null || "ShooterIdle".equals(currentShooterCommand.getName())) {
            schedule(shooter.backgroundCommand());
        }
    }

    private static void schedule(Command... commands) {
        CommandScheduler.getInstance().schedule(commands);
    }
}
