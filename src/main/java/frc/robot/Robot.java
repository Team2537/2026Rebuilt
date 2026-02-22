package frc.robot;

import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.hal.HALUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.util.WPILibVersion;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.Autos;
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
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

public final class Robot extends LoggedRobot {
    private static final int GOD_CONTROLLER_PORT = 5;

    private final Drive drive;
    private final Vision vision;
    private final Shooter shooter;
    private final Transfer transfer;
    private final Intake intake;
    private final Autos autos;

    private final CommandXboxController driverController = new CommandXboxController(0);
    private final CommandXboxController godController = new CommandXboxController(GOD_CONTROLLER_PORT);

    private FuelSim fuelSim;

    public Robot() {
        HAL.report(tResourceType.kResourceType_Language, tInstances.kLanguage_Java, 0, WPILibVersion.Version);

        Logger.recordMetadata("Type", RobotType.TYPE.toString());
        Logger.recordMetadata("Serial Number", HALUtil.getSerialNumber());
        Logger.recordOutput("Git Dirty", BuildConstants.DIRTY == 1 ? "DIRTY" : "CLEAN");
        Logger.recordOutput("Git Branch", BuildConstants.GIT_BRANCH);
        Logger.recordOutput("Git SHA", BuildConstants.GIT_SHA);
        Logger.recordOutput("Git Date", BuildConstants.GIT_DATE);

        switch (RobotType.MODE) {
            case REAL -> {
                Logger.addDataReceiver(new NT4Publisher());
                Logger.addDataReceiver(createWpiLogWriter());
                new PowerDistribution(1, PowerDistribution.ModuleType.kRev);
            }
            case SIMULATION -> {
                Logger.addDataReceiver(new NT4Publisher());
                Logger.addDataReceiver(createWpiLogWriter());
            }
            case REPLAY -> {
                setUseTiming(false);
                String logFile = LogFileUtil.findReplayLog();
                Logger.setReplaySource(new WPILOGReader(logFile));
                Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logFile, "_replayed")));
            }
        }

        Logger.start();

        CommandScheduler.getInstance()
                .onCommandInitialize(command -> Logger.recordOutput("commands/" + command.getName(), true));
        CommandScheduler.getInstance()
                .onCommandFinish(command -> Logger.recordOutput("commands/" + command.getName(), false));

        drive = createDrive();
        vision = Constants.isMechanismEnabled(Constants.Mechanism.VISION)
                ? new Vision(drive)
                : null;
        shooter = createShooter();
        transfer = createTransfer();
        intake = createIntake();
        autos = new Autos(drive);

        configureBindings();
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

    private static WPILOGWriter createWpiLogWriter() {
        if (RobotType.MODE != RobotType.Mode.REAL) {
            return new WPILOGWriter();
        }

        Path usbLogDir = Path.of("/U/logs");
        if (Files.isDirectory(usbLogDir) && Files.isWritable(usbLogDir)) {
            return new WPILOGWriter(usbLogDir.toString());
        }

        Path onboardLogDir = Path.of("/home/lvuser/logs");
        try {
            Files.createDirectories(onboardLogDir);
        } catch (IOException ignored) {
        }
        return new WPILOGWriter(onboardLogDir.toString());
    }

    private void configureBindings() {
        drive.setDefaultCommand(
                DriveCommands.joystickDrive(
                        drive, () -> driverController.getLeftY(), driverController::getLeftX,
                        () -> -driverController.getRightX()));
        shooter.setDefaultCommand(shooter.idleCommand());
        intake.setDefaultCommand(intake.backgroundCommand());

        driverController.leftBumper().onTrue(drive.toggleSlowMode());
        driverController.back().onTrue(DriveCommands.toggleFieldOriented(drive));
        driverController.start().onTrue(DriveCommands.resetOdometryAndHeading(drive));

        driverController.y().whileTrue(transfer.reverseCommand()).whileFalse(transfer.backgroundCommand());

        driverController.b()
                .onTrue(Commands.either(
                        intake.retractCommand().andThen(intake.stopCommand()),
                        intake.extendCommand().andThen(intake.backgroundCommand()),
                        intake::isExtended));
        driverController.leftTrigger().onTrue(intake.spinRoller()).onFalse(intake.backgroundCommand());

        driverController.povUp().onTrue(Commands.runOnce(this::scheduleBackgroundManipulators));
        driverController.povDown().onTrue(stopManipulatorsCommand());

        DoubleSupplier hubDistanceSupplier = () -> shooter.getMotionCompensatedHubDistanceMeters(
                drive.getPose(), drive.getMeasuredChassisSpeeds());
        Supplier<Rotation2d> hubHeadingSupplier = () -> shooter.getMotionCompensatedHubHeading(
                drive.getPose(), drive.getMeasuredChassisSpeeds());

        Trigger dashboardTuneTrigger = driverController.rightTrigger()
                .and(new Trigger(shooter::isDashboardTuningEnabled));
        Trigger shootTrigger = driverController.rightTrigger()
                .and(new Trigger(() -> !shooter.isDashboardTuningEnabled()));
        Trigger aimTrigger = driverController.rightBumper();
        Trigger aimOnlyTrigger = aimTrigger.and(shootTrigger.negate());
        Trigger shootOnlyTrigger = shootTrigger.and(aimTrigger.negate());
        Trigger aimAndShootTrigger = aimTrigger.and(shootTrigger);

        shootOnlyTrigger.whileTrue(shooter.shoot(hubDistanceSupplier));
        aimOnlyTrigger.whileTrue(Commands.parallel(
                DriveCommands.autoAlignToHubPose(
                        drive,
                        () -> driverController.getLeftY(),
                        driverController::getLeftX,
                        () -> -driverController.getRightX(),
                        hubHeadingSupplier),
                shooter.aimForDistance(hubDistanceSupplier)));
        aimAndShootTrigger.whileTrue(Commands.parallel(
                DriveCommands.autoAlignToHubPose(
                        drive,
                        () -> driverController.getLeftY(),
                        driverController::getLeftX,
                        () -> -driverController.getRightX(),
                        hubHeadingSupplier),
                shooter.shoot(hubDistanceSupplier)));
        dashboardTuneTrigger.whileTrue(shooter.dashboardTuneCommand());

        godController.leftBumper().onTrue(drive.toggleSlowMode());
        godController.povDown().onTrue(DriveCommands.resetOdometryAndHeading(drive));
        godController.a().whileTrue(drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        godController.b().whileTrue(drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        godController.x().whileTrue(drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
        godController.y().whileTrue(drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        shooter.getMotionCompensationToHub(drive.getPose(), drive.getMeasuredChassisSpeeds());
        MechanismVisualizer.updatePoses();
    }

    @Override
    public void disabledInit() {
        scheduleManipulatorStop();
    }

    @Override
    public void disabledPeriodic() {}

    @Override
    public void disabledExit() {}

    @Override
    public void autonomousInit() {
        autos.getSelectedRoutine().schedule();
        CommandScheduler.getInstance().schedule(intake.homeCommand());
        scheduleBackgroundManipulators();
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {}

    @Override
    public void teleopInit() {
        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().schedule(intake.homeCommand());
        scheduleBackgroundManipulators();
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void teleopExit() {
        scheduleManipulatorStop();
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {}

    @Override
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

    private Command stopManipulatorsCommand() {
        return Commands.parallel(
                shooter.stopCommand(),
                transfer.stopCommand(),
                intake.stopCommand())
                .withName("StopManipulators");
    }

    private void scheduleManipulatorStop() {
        CommandScheduler.getInstance().schedule(stopManipulatorsCommand());
    }

    private void scheduleBackgroundManipulators() {
        CommandScheduler.getInstance().schedule(
                transfer.backgroundCommand(),
                shooter.backgroundCommand(),
                intake.backgroundCommand());
    }
}
