package frc.robot.coordination.shooting;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Shooter.ReadinessMode;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterConstants;
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
                new ShootCoordinator(shooter, transfer, ShootCoordinatorConstants.FeedGateMode.IMMEDIATE);

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
                new ShootCoordinator(shooter, transfer, ShootCoordinatorConstants.FeedGateMode.SHOOTER_AND_AIM);

        Command command = coordinator.shootForDistance(() -> 4.0, () -> false);
        CommandScheduler.getInstance().schedule(command);

        runSchedulerCycles(3);

        assertFalse(shooter.isKickerActive());
        assertEquals(0.0, transferIO.percent, EPSILON);
    }

    @Test
    void shooterAndAimPolicyDebouncesBeforeFeeding() {
        TestShooterIO shooterIO = new TestShooterIO();
        TestTransferIO transferIO = new TestTransferIO();
        Shooter shooter = new Shooter(shooterIO);
        Transfer transfer = new Transfer(transferIO);
        ShootCoordinator coordinator =
                new ShootCoordinator(shooter, transfer, ShootCoordinatorConstants.FeedGateMode.SHOOTER_AND_AIM);

        Command command = coordinator.shootForDistance(() -> 4.0, () -> true);
        CommandScheduler.getInstance().schedule(command);

        runSchedulerCycles(2);
        assertFalse(shooter.isKickerActive());
        assertEquals(0.0, transferIO.percent, EPSILON);

        runSchedulerCycles(1);
        assertTrue(shooter.isKickerActive());
        assertEquals(TransferConstants.RUN_TRANSFER_PERCENT, transferIO.percent, EPSILON);
    }

    @Test
    void invalidDistanceAlwaysBlocksFeedAndRecoveryReopensGate() {
        TestShooterIO shooterIO = new TestShooterIO();
        TestTransferIO transferIO = new TestTransferIO();
        Shooter shooter = new Shooter(shooterIO);
        Transfer transfer = new Transfer(transferIO);
        ShootCoordinator coordinator =
                new ShootCoordinator(shooter, transfer, ShootCoordinatorConstants.FeedGateMode.IMMEDIATE);
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

        distanceMeters.set(5.0);
        runSchedulerCycles(2);

        assertTrue(shooter.isKickerActive());
        assertEquals(TransferConstants.RUN_TRANSFER_PERCENT, transferIO.percent, EPSILON);
    }

    @Test
    void infiniteDistanceIsRejectedAsInvalid() {
        TestShooterIO shooterIO = new TestShooterIO();
        TestTransferIO transferIO = new TestTransferIO();
        Shooter shooter = new Shooter(shooterIO);
        Transfer transfer = new Transfer(transferIO);
        ShootCoordinator coordinator =
                new ShootCoordinator(shooter, transfer, ShootCoordinatorConstants.FeedGateMode.IMMEDIATE);
        AtomicReference<Double> distanceMeters = new AtomicReference<>(4.0);

        Command command = coordinator.shootForDistance(distanceMeters::get, () -> true);
        CommandScheduler.getInstance().schedule(command);
        runSchedulerCycles(3);
        assertTrue(shooter.isKickerActive());

        distanceMeters.set(Double.POSITIVE_INFINITY);
        runSchedulerCycles(2);
        assertFalse(shooter.isKickerActive());
        assertEquals(0.0, transferIO.percent, EPSILON);

        distanceMeters.set(Double.NEGATIVE_INFINITY);
        runSchedulerCycles(2);
        assertFalse(shooter.isKickerActive());
        assertEquals(0.0, transferIO.percent, EPSILON);
    }

    @Test
    void modeSwitchFromImmediateToStrictClosesGate() {
        TestShooterIO shooterIO = new TestShooterIO();
        TestTransferIO transferIO = new TestTransferIO();
        Shooter shooter = new Shooter(shooterIO);
        Transfer transfer = new Transfer(transferIO);
        ShootCoordinator immediate =
                new ShootCoordinator(shooter, transfer, ShootCoordinatorConstants.FeedGateMode.IMMEDIATE);
        ShootCoordinator strict =
                new ShootCoordinator(shooter, transfer, ShootCoordinatorConstants.FeedGateMode.SHOOTER_AND_AIM);

        Command immediateCommand = immediate.shootForDistance(() -> 4.0, () -> true);
        CommandScheduler.getInstance().schedule(immediateCommand);
        runSchedulerCycles(2);

        assertTrue(shooter.isKickerActive());

        immediateCommand.cancel();
        runSchedulerCycles(1);

        Command strictCommand = strict.shootForDistance(() -> 4.0, () -> false);
        CommandScheduler.getInstance().schedule(strictCommand);
        runSchedulerCycles(3);

        assertFalse(shooter.isKickerActive());
        assertEquals(0.0, transferIO.percent, EPSILON);
    }

    @Test
    void aimDropResetsDebounceAndRequiresFreshStableCycles() {
        TestShooterIO shooterIO = new TestShooterIO();
        TestTransferIO transferIO = new TestTransferIO();
        Shooter shooter = new Shooter(shooterIO);
        Transfer transfer = new Transfer(transferIO);
        ShootCoordinator coordinator =
                new ShootCoordinator(shooter, transfer, ShootCoordinatorConstants.FeedGateMode.SHOOTER_AND_AIM);
        AtomicReference<Boolean> aimReady = new AtomicReference<>(true);

        Command command = coordinator.shootForDistance(() -> 4.0, aimReady::get);
        CommandScheduler.getInstance().schedule(command);

        runSchedulerCycles(3);
        assertTrue(shooter.isKickerActive());

        aimReady.set(false);
        runSchedulerCycles(1);
        assertFalse(shooter.isKickerActive());
        assertEquals(0.0, transferIO.percent, EPSILON);

        aimReady.set(true);
        runSchedulerCycles(1);
        assertFalse(shooter.isKickerActive(), "Debounce should require a second stable cycle after aim drop.");
        runSchedulerCycles(1);
        assertTrue(shooter.isKickerActive());
        assertEquals(TransferConstants.RUN_TRANSFER_PERCENT, transferIO.percent, EPSILON);
    }

    @Test
    void shotOnMoveReadinessDropDebounceKeepsFeedAliveForOneFlicker() {
        TestShooterIO shooterIO = new TestShooterIO();
        TestTransferIO transferIO = new TestTransferIO();
        Shooter shooter = new Shooter(shooterIO);
        Transfer transfer = new Transfer(transferIO);
        ShootCoordinator coordinator =
                new ShootCoordinator(shooter, transfer, ShootCoordinatorConstants.FeedGateMode.SHOOTER_AND_AIM);
        AtomicReference<Boolean> aimReady = new AtomicReference<>(true);

        Command command = coordinator.shootForDistance(() -> 4.0, aimReady::get, () -> true, () -> false, () -> true);
        CommandScheduler.getInstance().schedule(command);

        runSchedulerCycles(3);
        assertTrue(shooter.isKickerActive());

        aimReady.set(false);
        runSchedulerCycles(1);
        assertTrue(shooter.isKickerActive(), "Shot-on-move drop debounce should mask a one-cycle readiness flicker.");
        assertEquals(TransferConstants.RUN_TRANSFER_PERCENT, transferIO.percent, EPSILON);

        runSchedulerCycles(1);
        assertFalse(shooter.isKickerActive(), "A sustained readiness loss should still close the gate.");
        assertEquals(0.0, transferIO.percent, EPSILON);
    }

    @Test
    void shotOnMoveToleranceIsLooserThanStationaryTolerance() {
        double rpmError = (ShooterConstants.STATIONARY_SHOOTER_RPM_TOLERANCE
                + ShooterConstants.SHOT_ON_MOVE_SHOOTER_RPM_TOLERANCE) / 2.0;
        OffsetShooterIO shooterIO = new OffsetShooterIO(rpmError);
        Shooter shooter = new Shooter(shooterIO);

        shooter.setTargetsForDistance(4.0);
        runSchedulerCycles(1);

        var stationaryReadiness = shooter.getReadinessDiagnosticsNow(ReadinessMode.STATIONARY);
        var shotOnMoveReadiness = shooter.getReadinessDiagnosticsNow(ReadinessMode.SHOT_ON_MOVE);

        assertFalse(stationaryReadiness.leftVelocityAtSetpoint());
        assertFalse(stationaryReadiness.rightVelocityAtSetpoint());
        assertTrue(shotOnMoveReadiness.leftVelocityAtSetpoint());
        assertTrue(shotOnMoveReadiness.rightVelocityAtSetpoint());
    }

    @Test
    void hoodNotAtSetpointBlocksShooterAtSetpointPolicy() {
        LaggingHoodShooterIO shooterIO = new LaggingHoodShooterIO();
        TestTransferIO transferIO = new TestTransferIO();
        Shooter shooter = new Shooter(shooterIO);
        Transfer transfer = new Transfer(transferIO);
        ShootCoordinator coordinator =
                new ShootCoordinator(shooter, transfer, ShootCoordinatorConstants.FeedGateMode.SHOOTER_AT_SETPOINT);

        Command command = coordinator.shootForDistance(() -> 4.0, () -> true);
        CommandScheduler.getInstance().schedule(command);

        runSchedulerCycles(10);

        assertFalse(shooter.isKickerActive());
        assertEquals(0.0, transferIO.percent, EPSILON);
    }

    @Test
    void aimForDistanceClearsTargetsWhenDistanceBecomesInvalid() {
        TestShooterIO shooterIO = new TestShooterIO();
        Shooter shooter = new Shooter(shooterIO);
        Transfer transfer = new Transfer(new TestTransferIO());
        ShootCoordinator coordinator = new ShootCoordinator(shooter, transfer);
        AtomicReference<Double> distanceMeters = new AtomicReference<>(4.0);

        Command command = coordinator.aimForDistance(distanceMeters::get);
        CommandScheduler.getInstance().schedule(command);

        runSchedulerCycles(2);
        assertTrue(shooter.getTargetAverageShooterRpm() > 0.0);

        distanceMeters.set(Double.NaN);
        runSchedulerCycles(2);

        assertEquals(0.0, shooter.getTargetAverageShooterRpm(), EPSILON);
    }

    @Test
    void cancelStopsShooterAndTransferOutputs() {
        TestShooterIO shooterIO = new TestShooterIO();
        TestTransferIO transferIO = new TestTransferIO();
        Shooter shooter = new Shooter(shooterIO);
        Transfer transfer = new Transfer(transferIO);
        ShootCoordinator coordinator =
                new ShootCoordinator(shooter, transfer, ShootCoordinatorConstants.FeedGateMode.IMMEDIATE);

        Command command = coordinator.shootForDistance(() -> 4.0, () -> true);
        CommandScheduler.getInstance().schedule(command);
        runSchedulerCycles(3);
        assertTrue(shooter.isKickerActive());

        command.cancel();
        runSchedulerCycles(2);

        assertFalse(shooter.isKickerActive());
        assertFalse(coordinator.isActivelyFeeding());
        assertEquals(0.0, transferIO.percent, EPSILON);
        assertEquals(0.0, shooter.getTargetAverageShooterRpm(), EPSILON);
    }

    private static void runSchedulerCycles(int cycles) {
        for (int i = 0; i < cycles; i++) {
            DriverStationSim.notifyNewData();
            CommandScheduler.getInstance().run();
        }
    }

    private static class TestShooterIO implements ShooterIO {
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

    private static final class OffsetShooterIO extends TestShooterIO {
        private final double rpmError;

        private OffsetShooterIO(double rpmError) {
            this.rpmError = rpmError;
        }

        @Override
        public void setLeftVelocity(double rpm) {
            super.setLeftVelocity(rpm - rpmError);
        }

        @Override
        public void setRightVelocity(double rpm) {
            super.setRightVelocity(rpm - rpmError);
        }
    }

    private static final class LaggingHoodShooterIO extends TestShooterIO {
        @Override
        public void setHoodAngle(double angle) {
            // Simulate hood lag/failure: commanded angle never reflects in measured position.
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
