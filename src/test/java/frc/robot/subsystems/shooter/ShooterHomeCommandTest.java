package frc.robot.subsystems.shooter;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class ShooterHomeCommandTest {
    @BeforeEach
    void setUp() {
        HAL.initialize(500, 0);
        CommandScheduler.getInstance().cancelAll();

        DriverStationSim.setDsAttached(true);
        DriverStationSim.setAutonomous(false);
        DriverStationSim.setTest(false);
        DriverStationSim.setEStop(false);
        DriverStationSim.setEnabled(true);
        DriverStationSim.notifyNewData();
    }

    @AfterEach
    void tearDown() {
        CommandScheduler.getInstance().cancelAll();
        DriverStationSim.setEnabled(false);
        DriverStationSim.notifyNewData();
    }

    @Test
    void transientHomingCurrentStillLatchesSuccessfulReset() {
        TransientHomingShooterIO io = new TransientHomingShooterIO();
        Shooter shooter = new Shooter(io);
        Command home = shooter.homeCommand();
        CommandScheduler.getInstance().schedule(home);

        runSchedulerCycles(12);

        assertFalse(CommandScheduler.getInstance().isScheduled(home), "Shooter home should finish.");
        assertEquals(1, io.resetCount, "Shooter home should latch the stop hit and reset the encoder once.");
    }

    private static void runSchedulerCycles(int cycles) {
        for (int i = 0; i < cycles; i++) {
            DriverStationSim.notifyNewData();
            CommandScheduler.getInstance().run();
        }
    }

    private static final class TransientHomingShooterIO implements ShooterIO {
        private int cycle = 0;
        private boolean hoodVoltageCommanded = false;
        private boolean targetAngleCommanded = false;
        private double targetHoodAngleRad = ShooterConstants.HOOD_MIN_ANGLE_RAD;
        private int resetCount = 0;

        @Override
        public void updateInputs(ShooterIOInputs inputs) {
            cycle++;
            inputs.shooterLeftVelocityRpm = 0.0;
            inputs.shooterRightVelocityRpm = 0.0;
            inputs.hoodPositionRad = targetAngleCommanded ? targetHoodAngleRad : ShooterConstants.HOOD_MIN_ANGLE_RAD;
            inputs.hoodStatorCurrentAmps =
                    hoodVoltageCommanded && cycle == 3
                            ? ShooterConstants.HOMING_CURRENT_THRESHOLD_AMPS + 2.0
                            : 0.0;
        }

        @Override
        public void setHoodVoltage(double volts) {
            hoodVoltageCommanded = volts < 0.0;
            targetAngleCommanded = false;
        }

        @Override
        public void resetHoodEncoder() {
            resetCount++;
        }

        @Override
        public void setHoodAngle(double angleRad) {
            targetAngleCommanded = true;
            targetHoodAngleRad = angleRad;
        }

        @Override
        public void stop() {
            hoodVoltageCommanded = false;
        }
    }
}
