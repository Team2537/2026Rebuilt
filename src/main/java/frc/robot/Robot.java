package frc.robot;

import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.hal.HALUtil;
import edu.wpi.first.net.WebServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.util.WPILibVersion;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.util.ElasticNotifications;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.Locale;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.pathplanner.lib.commands.FollowPathCommand;

public final class Robot extends LoggedRobot {
    private static final int POWER_DISTRIBUTION_CAN_ID = 1;
    private static final double LOW_BATTERY_VOLTAGE = 11.5;
    private final RobotContainer robotContainer;
    private PowerDistribution powerDistribution = null;
    private boolean lowBatteryNotified = false;

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
                powerDistribution = new PowerDistribution(POWER_DISTRIBUTION_CAN_ID, PowerDistribution.ModuleType.kRev);
            }
            case SIMULATION -> {
                Logger.addDataReceiver(new NT4Publisher());
                Logger.addDataReceiver(createWpiLogWriter());
                powerDistribution = null;
            }
            case REPLAY -> {
                setUseTiming(false);
                String logFile = LogFileUtil.findReplayLog();
                Logger.setReplaySource(new WPILOGReader(logFile));
                Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logFile, "_replayed")));
                powerDistribution = null;
            }
        }

        Logger.start();
        WebServer.start(5800, Filesystem.getDeployDirectory().getPath());

        FollowPathCommand.warmupCommand().schedule();

        robotContainer = new RobotContainer();
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

    @Override
    public void robotPeriodic() {
        logRobotState();
        logPowerDistribution();
        CommandScheduler.getInstance().run();
        robotContainer.robotPeriodic();
    }

    private void logPowerDistribution() {
        if (powerDistribution == null) {
            return;
        }

        double voltage = powerDistribution.getVoltage();
        Logger.recordOutput("PowerDistribution/totalVoltageVolts", voltage);
        Logger.recordOutput("PowerDistribution/totalCurrentAmps", powerDistribution.getTotalCurrent());
        Logger.recordOutput("PowerDistribution/totalPowerWatts", powerDistribution.getTotalPower());
        Logger.recordOutput("PowerDistribution/temperatureCelsius", powerDistribution.getTemperature());
        boolean batteryLow = voltage < LOW_BATTERY_VOLTAGE;
        Logger.recordOutput("RobotState/BatteryVoltage", voltage);
        Logger.recordOutput("RobotState/BatteryLow", batteryLow);
    }

    private void logRobotState() {
        String mode;
        if (!DriverStation.isEnabled()) {
            mode = DriverStation.isEStopped() ? "ESTOPPED" : "DISABLED";
        } else if (DriverStation.isAutonomous()) {
            mode = "AUTO";
        } else if (DriverStation.isTeleop()) {
            mode = "TELEOP";
        } else {
            mode = "TEST";
        }
        Logger.recordOutput("RobotState/Mode", mode);
        Logger.recordOutput("RobotState/Enabled", DriverStation.isEnabled());
        Logger.recordOutput("RobotState/MatchTime", DriverStation.getMatchTime());
        Logger.recordOutput("RobotState/Alliance",
                DriverStation.getAlliance().map(Enum::name).orElse("Unknown"));
        Logger.recordOutput("RobotState/FMSAttached", DriverStation.isFMSAttached());
    }

    @Override
    public void disabledInit() {
        robotContainer.disabledInit();
    }

    @Override
    public void disabledPeriodic() {}

    @Override
    public void disabledExit() {}

    @Override
    public void autonomousInit() {
        robotContainer.autonomousInit();
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {}

    @Override
    public void teleopInit() {
        robotContainer.teleopInit();
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void teleopExit() {
        robotContainer.teleopExit();
    }

    @Override
    public void testInit() {
        robotContainer.testInit();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {}

    @Override
    public void simulationPeriodic() {
        robotContainer.simulationPeriodic();
    }
}
