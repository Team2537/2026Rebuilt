package frc.robot.subsystems.shooter;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import java.util.concurrent.atomic.AtomicReference;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class ShooterIOSimTest {
    private static final double EPSILON = 1e-6;
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

        assertTrue(
                kickerEverActive,
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

    @Test
    void dynamicDistanceChangesMidShotRemainStable() {
        Shooter shooter = new Shooter(new ShooterIOSim());
        double minDistance = ShooterConstants.SHOT_MAP_DISTANCE_METERS[0];
        double maxDistance = ShooterConstants.SHOT_MAP_DISTANCE_METERS[
                ShooterConstants.SHOT_MAP_DISTANCE_METERS.length - 1];
        AtomicReference<Double> distanceMeters = new AtomicReference<>(minDistance);

        Command shoot = shooter.shoot(distanceMeters::get);
        CommandScheduler.getInstance().schedule(shoot);

        boolean atSetpointBeforeSwitch = false;
        boolean atSetpointAfterSwitch = false;
        for (int i = 0; i < 800; i++) {
            if (i == 220) {
                distanceMeters.set(maxDistance);
            }
            if (i == 440) {
                distanceMeters.set((minDistance + maxDistance) * 0.5);
            }

            DriverStationSim.notifyNewData();
            CommandScheduler.getInstance().run();
            if (i < 220 && shooter.atSetpoint()) {
                atSetpointBeforeSwitch = true;
            }
            if (i > 520 && shooter.atSetpoint()) {
                atSetpointAfterSwitch = true;
            }
        }

        assertTrue(atSetpointBeforeSwitch, "Shooter never reached setpoint before distance switch.");
        assertTrue(atSetpointAfterSwitch, "Shooter never recovered setpoint after distance switches.");
    }

    @Test
    void shotMapBoundaryQueriesReturnFiniteTargets() {
        Shooter shooter = new Shooter(new ShooterIOSim());
        double minDistance = ShooterConstants.SHOT_MAP_DISTANCE_METERS[0];
        double maxDistance = ShooterConstants.SHOT_MAP_DISTANCE_METERS[
                ShooterConstants.SHOT_MAP_DISTANCE_METERS.length - 1];

        Shooter.ShotSetpoint belowMin = shooter.calculateSetpointForDistance(minDistance - 1.0);
        Shooter.ShotSetpoint aboveMax = shooter.calculateSetpointForDistance(maxDistance + 1.0);

        assertTrue(
                Double.isFinite(belowMin.leftRpm())
                        && Double.isFinite(belowMin.rightRpm())
                        && Double.isFinite(belowMin.hoodAngleRad())
                        && Double.isFinite(aboveMax.leftRpm())
                        && Double.isFinite(aboveMax.rightRpm())
                        && Double.isFinite(aboveMax.hoodAngleRad()),
                "Shot map interpolation returned non-finite boundary values.");
    }

    @Test
    void simIoKickerModesAndClampBehavior() {
        ShooterIOSim io = new ShooterIOSim();
        ShooterIO.ShooterIOInputs inputs = new ShooterIO.ShooterIOInputs();

        io.setKickerTorque(ShooterConstants.KICKER_MAX_TORQUE_CURRENT_AMPS * 2.0);
        io.updateInputs(inputs);
        assertEquals(
                ShooterConstants.MAX_OUTPUT_VOLTS,
                Math.abs(inputs.kickerAppliedVolts),
                1e-3,
                "Kicker torque mode should clamp to max output volts");

        io.setKickerVoltage(4.0);
        io.updateInputs(inputs);
        assertEquals(4.0, inputs.kickerAppliedVolts, 1e-3);

        io.stop();
        io.updateInputs(inputs);
        assertEquals(0.0, inputs.kickerAppliedVolts, EPSILON);
    }

    @Test
    void hoodHomingCurrentAppearsOnlyAfterSimulatedHardStop() {
        ShooterIOSim io = new ShooterIOSim();
        ShooterIO.ShooterIOInputs inputs = new ShooterIO.ShooterIOInputs();

        io.setHoodVoltage(-12.0);

        boolean homingCurrentObserved = false;
        for (int i = 0; i < 250; i++) {
            io.updateInputs(inputs);
            if (inputs.hoodStatorCurrentAmps > ShooterConstants.HOMING_CURRENT_THRESHOLD_AMPS) {
                homingCurrentObserved = true;
                break;
            }
        }

        assertTrue(
                homingCurrentObserved,
                "Expected simulated hood homing current once hood reached hard stop");

        io.setHoodVoltage(0.0);
        io.updateInputs(inputs);
        assertFalse(
                inputs.hoodStatorCurrentAmps > ShooterConstants.HOMING_CURRENT_THRESHOLD_AMPS,
                "Homing current should clear once homing voltage is removed");
    }

    @Test
    void atSetpointDebouncedForBriefShooterRpmDrop() {
        TestInputsShooterIO shooterIO = new TestInputsShooterIO();
        shooterIO.leftRpm = 2200.0;
        shooterIO.rightRpm = 2200.0;
        shooterIO.hoodPositionRad = 0.0;
        Shooter shooter = new Shooter(shooterIO);
        shooter.setTargets(2200.0, 2200.0, 0.0);

        shooter.periodic();
        assertTrue(shooter.atSetpoint());

        shooterIO.leftRpm = 1000.0;
        shooterIO.rightRpm = 1000.0;
        shooter.periodic();
        Timer.delay(0.10);
        shooter.periodic();
        assertTrue(shooter.atSetpoint(), "A shooter-RPM dip shorter than 0.2s should not clear atSetpoint.");
    }

    @Test
    void atSetpointDropsAfterSustainedShooterRpmDrop() {
        TestInputsShooterIO shooterIO = new TestInputsShooterIO();
        shooterIO.leftRpm = 2200.0;
        shooterIO.rightRpm = 2200.0;
        shooterIO.hoodPositionRad = 0.0;
        Shooter shooter = new Shooter(shooterIO);
        shooter.setTargets(2200.0, 2200.0, 0.0);

        shooter.periodic();
        assertTrue(shooter.atSetpoint());

        shooterIO.leftRpm = 1000.0;
        shooterIO.rightRpm = 1000.0;
        shooter.periodic();
        Timer.delay(0.25);
        shooter.periodic();
        assertFalse(shooter.atSetpoint(), "A shooter-RPM dip longer than 0.2s should clear atSetpoint.");
    }

    @Test
    void stopClearsClosedLoopOutputs() {
        ShooterIOSim io = new ShooterIOSim();
        ShooterIO.ShooterIOInputs inputs = new ShooterIO.ShooterIOInputs();

        io.setLeftVelocity(2500.0);
        io.setRightVelocity(2500.0);
        io.setHoodAngle(Math.toRadians(35.0));
        io.setKickerVoltage(6.0);
        io.updateInputs(inputs);
        assertTrue(Math.abs(inputs.shooterLeftAppliedVolts) > 0.0);
        assertTrue(Math.abs(inputs.kickerAppliedVolts) > 0.0);

        io.stop();
        io.updateInputs(inputs);

        assertEquals(0.0, inputs.shooterLeftAppliedVolts, EPSILON);
        assertEquals(0.0, inputs.shooterRightAppliedVolts, EPSILON);
        assertEquals(0.0, inputs.hoodAppliedVolts, EPSILON);
        assertEquals(0.0, inputs.kickerAppliedVolts, EPSILON);
    }

    private static final class TestInputsShooterIO implements ShooterIO {
        private double leftRpm = 0.0;
        private double rightRpm = 0.0;
        private double hoodPositionRad = 0.0;

        @Override
        public void updateInputs(ShooterIOInputs inputs) {
            inputs.shooterLeftVelocityRpm = leftRpm;
            inputs.shooterRightVelocityRpm = rightRpm;
            inputs.hoodPositionRad = hoodPositionRad;
        }
    }
}
