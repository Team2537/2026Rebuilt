package frc.robot;

import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.hal.HALUtil;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.util.WPILibVersion;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.Autos;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.AlignmentState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.GyroIOSim;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.drive.ModuleIOSim;
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
import frc.robot.sim.FuelSim;
import lib.controllers.CommandButtonBoard;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import edu.wpi.first.math.geometry.Rotation2d;

public final class Robot extends LoggedRobot {
    public static final double UPDATE_RATE_SECONDS = 0.02;

    private static Drive drive;
    private static AlignmentState alignmentState;
    private static Vision vision;
    private static Shooter shooter;
    private static Transfer transfer;
    private static Intake intake;

    // Controller setup from 2025Reefscape
    private final CommandXboxController driverController = new CommandXboxController(0);
    // private final CommandButtonBoard operatorController = new CommandButtonBoard(1, 2);
    private final CommandXboxController godController = new CommandXboxController(5);
    private final FuelSim fuelSim = new FuelSim();

    private Autos autos;

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

        boolean enableDrive = Constants.isMechanismEnabled(Constants.Mechanism.DRIVE);
        boolean enableVision = Constants.isMechanismEnabled(Constants.Mechanism.VISION);
        boolean enableShooter = Constants.isMechanismEnabled(Constants.Mechanism.SHOOTER);
        boolean enableTransfer = Constants.isMechanismEnabled(Constants.Mechanism.TRANSFER);
        boolean enableIntake = Constants.isMechanismEnabled(Constants.Mechanism.INTAKE);

        if (enableDrive) {
            // Initialize drive subsystem
            switch (RobotType.MODE) {
                case REAL ->
                        drive = new Drive(
                                new GyroIOPigeon2(),
                                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                                new ModuleIOTalonFX(TunerConstants.FrontRight),
                                new ModuleIOTalonFX(TunerConstants.BackLeft),
                                new ModuleIOTalonFX(TunerConstants.BackRight));
                case SIMULATION -> {
                    GyroIOSim gyroIOSim = new GyroIOSim(Drive.getModuleTranslations());
                    drive = new Drive(
                            gyroIOSim,
                            new ModuleIOSim(TunerConstants.FrontLeft),
                            new ModuleIOSim(TunerConstants.FrontRight),
                            new ModuleIOSim(TunerConstants.BackLeft),
                            new ModuleIOSim(TunerConstants.BackRight));
                    gyroIOSim.setModulePositionsSupplier(drive::getModulePositionsForSim);
                }
                default ->
                        drive = new Drive(
                                new GyroIO() {},
                                new ModuleIO() {},
                                new ModuleIO() {},
                                new ModuleIO() {},
                                new ModuleIO() {});
            }
        }

        alignmentState = new AlignmentState();
        vision = enableDrive && enableVision ? new Vision(drive) : null;
        autos = enableDrive ? new Autos(drive) : null;

        if (enableShooter) {
            switch (RobotType.MODE) {
                case REAL -> shooter = new Shooter(new ShooterIOReal());
                case SIMULATION -> shooter = new Shooter(new ShooterIOSim());
                case REPLAY -> shooter = new Shooter(new ShooterIO() {
                });
            }
        }

        if (enableTransfer) {
            switch (RobotType.MODE) {
                case REAL -> transfer = new Transfer(new TransferIOReal());
                case SIMULATION -> transfer = new Transfer(new TransferIOSim());
                case REPLAY -> transfer = new Transfer(new TransferIO() {
                });
            }
        }

        if (enableIntake) {
            switch (RobotType.MODE) {
                case REAL -> intake = new Intake(new IntakeIOReal());
                case SIMULATION -> intake = new Intake(new IntakeIOSim());
                case REPLAY -> intake = new Intake(new IntakeIO() {
                });
            }
        }
        configureBindings();
    }

    private static WPILOGWriter createWpiLogWriter() {
        if (RobotType.MODE != RobotType.MODE.REAL) {
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
            // Writer creation below will report an explicit AdvantageKit error if this still fails.
        }
        return new WPILOGWriter(onboardLogDir.toString());
    }

    private void configureBindings() {
        if (drive != null) {
            // Default drive command
            drive.setDefaultCommand(
                    DriveCommands.joystickDrive(
                            drive, () -> driverController.getLeftY(), driverController::getLeftX, () -> -driverController.getRightX()));
        }

        // Driver controller bindings
        if (drive != null) {
            driverController.leftBumper().onTrue(drive.toggleSlowMode());
        }
        if (transfer != null) {
            driverController.rightBumper().onTrue(transfer.toggleCommand());
        }
        if (intake != null) {
            driverController.x().onTrue(intake.extendCommand());
            driverController.y().onTrue(intake.retractCommand());
            driverController.a().whileTrue(intake.spinRoller());
        }
        if (drive != null) {
            driverController.leftStick().onTrue(DriveCommands.toggleFieldOriented(drive));
            driverController.povDown().onTrue(DriveCommands.resetOdometryAndHeading(drive));
        }

        if (shooter != null) {
            shooter.setDefaultCommand(shooter.idleCommand());
            Trigger dashboardTuneTrigger = driverController.rightTrigger().and(new Trigger(shooter::isDashboardTuningEnabled));
            if (drive != null) {
                DoubleSupplier hubDistanceSupplier =
                        () -> shooter.getMotionCompensatedHubDistanceMeters(
                                drive.getPose(),
                                drive.getMeasuredChassisSpeeds());
                Supplier<Rotation2d> hubHeadingSupplier =
                        () -> shooter.getMotionCompensatedHubHeading(
                                drive.getPose(),
                                drive.getMeasuredChassisSpeeds());
                Trigger shootTrigger = driverController.rightTrigger().and(new Trigger(() -> !shooter.isDashboardTuningEnabled()));
                Trigger aimTrigger = driverController.leftTrigger().and(shootTrigger.negate());

                var shootCommand = shooter.shoot(hubDistanceSupplier);
                var aimCommand = shooter.aimForDistance(hubDistanceSupplier);
                var alignToHubPose = DriveCommands.autoAlignToHubPose(
                        drive,
                        () -> driverController.getLeftY(),
                        driverController::getLeftX,
                        () -> -driverController.getRightX(),
                        hubHeadingSupplier);
                aimCommand = Commands.parallel(alignToHubPose, aimCommand);
                shootCommand = Commands.parallel(
                        DriveCommands.autoAlignToHubPose(
                                drive,
                                () -> driverController.getLeftY(),
                                driverController::getLeftX,
                                () -> -driverController.getRightX(),
                                hubHeadingSupplier),
                        shootCommand);
                shootTrigger.whileTrue(shootCommand);
                aimTrigger.whileTrue(aimCommand);
            }
            dashboardTuneTrigger.whileTrue(shooter.dashboardTuneCommand());

            // Operator panel: action fires, stow stops shooter outputs.
            // operatorController.getActionButton().whileTrue(shooter.shoot(hubDistanceSupplier));
            // operatorController.getStowButton().onTrue(shooter.stopCommand());
        }

        // God controller can also toggle slow mode and reset heading
        if (drive != null) {
            godController.leftBumper().onTrue(drive.toggleSlowMode());
            godController.povDown().onTrue(DriveCommands.resetOdometryAndHeading(drive));
            godController.a().whileTrue(drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
            godController.b().whileTrue(drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
            godController.x().whileTrue(drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
            godController.y().whileTrue(drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
        }
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        if (shooter != null && drive != null) {
            shooter.getMotionCompensationToHub(drive.getPose(), drive.getMeasuredChassisSpeeds());
        }
        MechanismVisualizer.updatePoses();
    }

    @Override
    public void disabledInit() {
        stopManipulators();
    }

    @Override
    public void disabledPeriodic() {
    }

    @Override
    public void disabledExit() {
    }

    @Override
    public void autonomousInit() {
        if (autos != null) {
            autos.getSelectedRoutine().schedule();
        }
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void autonomousExit() {
    }

    @Override
    public void teleopInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void teleopExit() {
        stopManipulators();
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {
    }

    @Override
    public void testExit() {
    }

    @Override
    public void simulationPeriodic() {
        if (drive == null) {
            return;
        }

        boolean shooterActive = shooter != null && shooter.isKickerActive();
        double shooterRpm = shooter != null ? shooter.getTargetAverageShooterRpm() : 0.0;
        double hoodAngleRad = shooter != null ? shooter.getTargetHoodAngleRad() : 0.0;
        fuelSim.update(drive.getPose(), shooterRpm, hoodAngleRad, shooterActive, UPDATE_RATE_SECONDS);
    }

    public static double getUpdateRateSec() {
        return UPDATE_RATE_SECONDS;
    }

    public static Drive getDrive() {
        return drive;
    }

    public static AlignmentState getAlignmentState() {
        return alignmentState;
    }

    public static Vision getVision() {
        return vision;
    }

    public static Shooter getShooter() {
        return shooter;
    }

    public static Transfer getTransfer() {
        return transfer;
    }

    public static Intake getIntake() {
        return intake;
    }

    private void stopManipulators() {
        if (shooter != null) {
            shooter.stopAll();
        }
        if (transfer != null) {
            transfer.stopAll();
        }
        if (intake != null) {
            intake.stopAll();
        }
    }
}
