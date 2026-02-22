package frc.robot.subsystems.shooter;

import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class ShooterIOSimTest {
    @BeforeEach
    void setUp() {
        HAL.initialize(500, 0);
        DriverStationSim.setDsAttached(true);
        DriverStationSim.setAutonomous(false);
        DriverStationSim.setTest(false);
        DriverStationSim.setEStop(false);
        DriverStationSim.setEnabled(true);
        DriverStationSim.notifyNewData();
        CommandScheduler.getInstance().cancelAll();
    }

    @AfterEach
    void tearDown() {
        CommandScheduler.getInstance().cancelAll();
        DriverStationSim.setEnabled(false);
        DriverStationSim.notifyNewData();
    }

    @Test
    void shootCommandEventuallyActivatesKickerInSim() {
        Shooter shooter = new Shooter(new ShooterIOSim());
        Command shoot = shooter.shoot(() -> 4.0);
        CommandScheduler.getInstance().schedule(shoot);

        boolean kickerEverActive = false;
        boolean everEnabled = false;
        boolean everAtSetpoint = false;
        for (int i = 0; i < 500; i++) {
            DriverStationSim.notifyNewData();
            CommandScheduler.getInstance().run();
            everEnabled |= DriverStation.isEnabled();
            everAtSetpoint |= shooter.atSetpoint();
            if (shooter.isKickerActive()) {
                kickerEverActive = true;
                break;
            }
        }

        assertTrue(kickerEverActive,
                "Kicker never became active in simulation. everEnabled=" + everEnabled + " everAtSetpoint=" + everAtSetpoint);
    }

    @Test
    void simIoCanReachShooterAndHoodTargets() {
        ShooterIOSim io = new ShooterIOSim();
        ShooterIO.ShooterIOInputs inputs = new ShooterIO.ShooterIOInputs();
        double targetRpm = 1600.0;
        double targetHoodRad = Math.toRadians(64.0);

        io.setLeftVelocity(targetRpm);
        io.setRightVelocity(targetRpm);
        io.setHoodAngle(targetHoodRad);

        for (int i = 0; i < 500; i++) {
            io.updateInputs(inputs);
        }

        double leftError = Math.abs(inputs.shooterLeftVelocityRpm - targetRpm);
        double rightError = Math.abs(inputs.shooterRightVelocityRpm - targetRpm);
        double hoodErrorRad = Math.abs(inputs.hoodPositionRad - targetHoodRad);

        assertTrue(
                leftError <= ShooterConstants.SHOOTER_RPM_TOLERANCE
                        && rightError <= ShooterConstants.SHOOTER_RPM_TOLERANCE
                        && hoodErrorRad <= ShooterConstants.HOOD_ANGLE_TOLERANCE_RAD,
                "Targets not reached: leftErrorRpm=" + leftError
                        + " rightErrorRpm=" + rightError
                        + " hoodErrorDeg=" + Math.toDegrees(hoodErrorRad)
                        + " finalLeftRpm=" + inputs.shooterLeftVelocityRpm
                        + " finalRightRpm=" + inputs.shooterRightVelocityRpm
                        + " finalHoodDeg=" + Math.toDegrees(inputs.hoodPositionRad));
    }
}
