package frc.robot.coordination.shooting;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.transfer.Transfer;
import frc.robot.subsystems.transfer.TransferConstants;
import frc.robot.subsystems.transfer.TransferIO;
import java.util.concurrent.atomic.AtomicReference;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class ShootCoordinatorTest {
    private static final double EPSILON = 1e-9;

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
    void openPolicyEnablesKickerAndTransfer() {
        TestShooterIO shooterIO = new TestShooterIO();
        TestTransferIO transferIO = new TestTransferIO();
        Shooter shooter = new Shooter(shooterIO);
        Transfer transfer = new Transfer(transferIO);
        ShootCoordinator coordinator =
                new ShootCoordinator(shooter, transfer, Constants.FeedGateMode.IMMEDIATE);

        Command command = coordinator.shootForDistance(() -> 4.0, () -> false);
        CommandScheduler.getInstance().schedule(command);

        runSchedulerCycles(3);

        assertTrue(shooter.isKickerActive());
        assertEquals(TransferConstants.RUN_TRANSFER_PERCENT, transferIO.percent, EPSILON);
    }

    @Test
    void blockedPolicyDisablesKickerAndTransfer() {
        TestShooterIO shooterIO = new TestShooterIO();
        TestTransferIO transferIO = new TestTransferIO();
        Shooter shooter = new Shooter(shooterIO);
        Transfer transfer = new Transfer(transferIO);
        ShootCoordinator coordinator =
                new ShootCoordinator(shooter, transfer, Constants.FeedGateMode.SHOOTER_AND_AIM);

        Command command = coordinator.shootForDistance(() -> 4.0, () -> false);
        CommandScheduler.getInstance().schedule(command);

        runSchedulerCycles(3);

        assertFalse(shooter.isKickerActive());
        assertEquals(0.0, transferIO.percent, EPSILON);
    }

    @Test
    void invalidDistanceAlwaysBlocksFeed() {
        TestShooterIO shooterIO = new TestShooterIO();
        TestTransferIO transferIO = new TestTransferIO();
        Shooter shooter = new Shooter(shooterIO);
        Transfer transfer = new Transfer(transferIO);
        ShootCoordinator coordinator =
                new ShootCoordinator(shooter, transfer, Constants.FeedGateMode.IMMEDIATE);
        AtomicReference<Double> distanceMeters = new AtomicReference<>(4.0);

        Command command = coordinator.shootForDistance(distanceMeters::get, () -> true);
        CommandScheduler.getInstance().schedule(command);

        runSchedulerCycles(3);
        assertTrue(shooter.isKickerActive());
        assertEquals(TransferConstants.RUN_TRANSFER_PERCENT, transferIO.percent, EPSILON);

        distanceMeters.set(Double.NaN);
        runSchedulerCycles(2);

        assertFalse(shooter.isKickerActive());
        assertEquals(0.0, transferIO.percent, EPSILON);
    }

    @Test
    void cancelStopsShooterAndTransferOutputs() {
        TestShooterIO shooterIO = new TestShooterIO();
        TestTransferIO transferIO = new TestTransferIO();
        Shooter shooter = new Shooter(shooterIO);
        Transfer transfer = new Transfer(transferIO);
        ShootCoordinator coordinator =
                new ShootCoordinator(shooter, transfer, Constants.FeedGateMode.IMMEDIATE);

        Command command = coordinator.shootForDistance(() -> 4.0, () -> true);
        CommandScheduler.getInstance().schedule(command);
        runSchedulerCycles(3);
        assertTrue(shooter.isKickerActive());

        command.cancel();
        runSchedulerCycles(2);

        assertFalse(shooter.isKickerActive());
        assertEquals(0.0, transferIO.percent, EPSILON);
        assertEquals(0.0, shooter.getTargetAverageShooterRpm(), EPSILON);
    }

    private static void runSchedulerCycles(int cycles) {
        for (int i = 0; i < cycles; i++) {
            DriverStationSim.notifyNewData();
            CommandScheduler.getInstance().run();
        }
    }

    private static final class TestShooterIO implements ShooterIO {
        private double leftRpm;
        private double rightRpm;
        private double hoodRad;

        @Override
        public void updateInputs(ShooterIOInputs inputs) {
            inputs.shooterLeftVelocityRpm = leftRpm;
            inputs.shooterRightVelocityRpm = rightRpm;
            inputs.hoodPositionRad = hoodRad;
        }

        @Override
        public void setLeftVelocity(double rpm) {
            leftRpm = rpm;
        }

        @Override
        public void setRightVelocity(double rpm) {
            rightRpm = rpm;
        }

        @Override
        public void setHoodAngle(double angle) {
            hoodRad = angle;
        }
    }

    private static final class TestTransferIO implements TransferIO {
        private double percent = 0.0;

        @Override
        public void setPercent(double percent) {
            this.percent = percent;
        }

        @Override
        public void stop() {
            percent = 0.0;
        }
    }
}
