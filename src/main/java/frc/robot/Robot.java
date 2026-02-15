package frc.robot;

import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.hal.HALUtil;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.util.WPILibVersion;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.Autos;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.AlignmentState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.GyroIONavX;
import frc.robot.subsystems.drive.GyroIOSim;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOHybridFXS;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOReal;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.TurretIO;
import frc.robot.subsystems.turret.TurretIOSim;
import frc.robot.subsystems.vision.Vision;
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

public final class Robot extends LoggedRobot {
    public static final double UPDATE_RATE_SECONDS = 0.02;

    private static Drive drive;
    private static AlignmentState alignmentState;
    private static Vision vision;
    private static Turret turret;
    private static Shooter shooter;

    // Controller setup from 2025Reefscape
    private final CommandXboxController driverController = new CommandXboxController(0);
    private final CommandButtonBoard operatorController = new CommandButtonBoard(1, 2);
    private final CommandXboxController godController = new CommandXboxController(5);

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

        boolean visionOnlyMode = Constants.VISION_ONLY_MODE;

        // Initialize drive subsystem
        if (visionOnlyMode) {
            drive = new Drive(
                    new GyroIO() {},
                    new ModuleIO() {},
                    new ModuleIO() {},
                    new ModuleIO() {},
                    new ModuleIO() {});
        } else {
            switch (RobotType.MODE) {
                case REAL ->
                        drive = new Drive(
                                new GyroIONavX(),
                                new ModuleIOHybridFXS(TunerConstants.FrontLeft),
                                new ModuleIOHybridFXS(TunerConstants.FrontRight),
                                new ModuleIOHybridFXS(TunerConstants.BackLeft),
                                new ModuleIOHybridFXS(TunerConstants.BackRight));
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
        vision = new Vision(drive);

        // Initialize shooter subsystem
        if (!visionOnlyMode) {
            switch (RobotType.MODE) {
                case REAL -> shooter = new Shooter(new ShooterIOReal());
                case SIMULATION, REPLAY -> shooter = new Shooter(new ShooterIO() {});
            }
        }

        if (!visionOnlyMode) {
            // Initialize turret subsystem
            switch (RobotType.MODE) {
                case SIMULATION -> turret = new Turret(new TurretIOSim(), vision::getHubYawRad);
                case REAL, REPLAY -> turret = new Turret(new TurretIO() {}, vision::getHubYawRad);
            }

            autos = new Autos(drive);

            configureBindings();
        }
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
        // Default drive command
        drive.setDefaultCommand(
                DriveCommands.joystickDrive(
                        drive, driverController::getLeftY, driverController::getLeftX, () -> -driverController.getRightX()));

        // Driver controller bindings
        driverController.leftBumper().onTrue(drive.toggleSlowMode());
        driverController.leftStick().onTrue(DriveCommands.toggleFieldOriented(drive));
        driverController.povDown().onTrue(DriveCommands.resetOdometryAndHeading(drive));

        if (shooter != null) {
            DoubleSupplier hubDistanceSupplier = Shooter.hubDistanceMetersSupplier(drive::getPose);

            // Driver hold-to-shoot while driving.
            driverController.rightTrigger().whileTrue(shooter.shoot(hubDistanceSupplier));
            driverController.leftTrigger().whileTrue(shooter.aimForDistance(hubDistanceSupplier));

            // Operator panel: action fires, stow stops shooter outputs.
            operatorController.getActionButton().whileTrue(shooter.shoot(hubDistanceSupplier));
            operatorController.getStowButton().onTrue(shooter.stopCommand());
        }

        // God controller can also toggle slow mode and reset heading
        godController.leftBumper().onTrue(drive.toggleSlowMode());
        godController.povDown().onTrue(DriveCommands.resetOdometryAndHeading(drive));
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        MechanismVisualizer.updatePoses();
    }

    @Override
    public void disabledInit() {
        if (shooter != null) {
            shooter.stopAll();
        }
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
        if (shooter != null) {
            shooter.stopAll();
        }
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

    public static Turret getTurret() {
        return turret;
    }

    public static Shooter getShooter() {
        return shooter;
    }
}
