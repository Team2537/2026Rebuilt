package frc.robot.sim;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.shooter.ShooterIO;
import java.util.Locale;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

final class FullFunctionalityFaultInjectionDisconnects {
    private static final double LOOP_PERIOD_SEC = 0.02;

    @BeforeEach
    void setUp() {
        HAL.initialize(500, 0);
        SimHooks.pauseTiming();
        DriverStationSim.resetData();

        CommandScheduler scheduler = CommandScheduler.getInstance();
        scheduler.cancelAll();
        scheduler.getDefaultButtonLoop().clear();
        scheduler.getActiveButtonLoop().clear();
        scheduler.setActiveButtonLoop(scheduler.getDefaultButtonLoop());
        scheduler.clearComposedCommands();
        scheduler.unregisterAllSubsystems();

        DriverStationSim.setDsAttached(true);
        DriverStationSim.setEStop(false);
        DriverStationSim.setAutonomous(false);
        DriverStationSim.setTest(false);
        DriverStationSim.setEnabled(true);
        DriverStationSim.notifyNewData();

        runCycles(4);
    }

    @AfterEach
    void tearDown() {
        CommandScheduler scheduler = CommandScheduler.getInstance();
        scheduler.cancelAll();
        scheduler.getDefaultButtonLoop().clear();
        scheduler.getActiveButtonLoop().clear();
        scheduler.setActiveButtonLoop(scheduler.getDefaultButtonLoop());
        scheduler.clearComposedCommands();
        scheduler.unregisterAllSubsystems();

        DriverStationSim.setEnabled(false);
        DriverStationSim.notifyNewData();
        runCycles(2);
        SimHooks.resumeTiming();
    }

    @Test
    void intakeExtendShouldNotClaimExtendedWhenDisconnected() {
        IntakeDisconnectedIO io = new IntakeDisconnectedIO();
        Intake intake = new Intake(io);
        runCycles(5);

        Command extend = intake.extendCommand().withName("FF_Fault_IntakeExtendDisconnected");
        CommandScheduler.getInstance().schedule(extend);

        boolean finished = runUntil(
                () -> !CommandScheduler.getInstance().isScheduled(extend),
                secondsToCycles(IntakeConstants.MOVE_TIMEOUT_SEC + 0.5));
        assertTrue(
                finished,
                FullFunctionalityHarness.formatExpectedVsActual(
                        "Fault-injected intake extend command should time out and finish in bounded sim time",
                        "finished=true",
                        "finished=" + finished));

        assertFalse(
                intake.isExtended(),
                FullFunctionalityHarness.formatExpectedVsActual(
                        "Disconnected intake must not claim logical extended=true when mechanism never reaches target",
                        "intake.isExtended()=false",
                        String.format(
                                Locale.US,
                                "intake.isExtended()=%s leftPosRot=%.3f rightPosRot=%.3f leftCmdVolts=%.2f rightCmdVolts=%.2f",
                                intake.isExtended(),
                                Units.radiansToRotations(io.getLeftPositionRad()),
                                Units.radiansToRotations(io.getRightPositionRad()),
                                io.getLeftAppliedVolts(),
                                io.getRightAppliedVolts())));
    }

    @Test
    void intakeHomeShouldNotClaimRetractedWhenHomingNeverReachesStop() {
        IntakeDisconnectedIO io = new IntakeDisconnectedIO();
        Intake intake = new Intake(io);
        runCycles(5);

        intake.setExtended(true);
        runCycles(8);
        assertTrue(intake.isExtended(), "Precondition failed: intake should start this fault test marked extended.");

        Command home = intake.homeCommand().withName("FF_Fault_IntakeHomeDisconnected");
        CommandScheduler.getInstance().schedule(home);

        boolean finished = runUntil(
                () -> !CommandScheduler.getInstance().isScheduled(home),
                secondsToCycles(IntakeConstants.HOMING_WAIT_TIMEOUT_SEC + 0.5));
        assertTrue(
                finished,
                FullFunctionalityHarness.formatExpectedVsActual(
                        "Fault-injected intake home should reach timeout path in bounded sim time",
                        "finished=true",
                        "finished=" + finished));

        assertTrue(
                intake.isExtended() && io.getResetEncodersCalls() == 0,
                FullFunctionalityHarness.formatExpectedVsActual(
                        "Intake home with no homing current should not claim retract success or reset encoders",
                        "intake.isExtended()=true and resetEncodersCalls=0",
                        String.format(
                                Locale.US,
                                "intake.isExtended()=%s resetEncodersCalls=%d maxObservedStatorCurrent=%.2fA threshold=%.2fA",
                                intake.isExtended(),
                                io.getResetEncodersCalls(),
                                io.getMaxObservedStatorCurrentAmps(),
                                IntakeConstants.HOMING_CURRENT_THRESHOLD_AMPS)));
    }

    @Test
    void shooterHomeShouldNotFinishSuccessfullyWhenHomingNeverReachesStop() {
        ShooterDisconnectedIO io = new ShooterDisconnectedIO();
        Shooter shooter = new Shooter(io);
        runCycles(5);

        shooter.setTargets(2200.0, 2200.0, Units.degreesToRadians(72.0));
        runCycles(20);

        Command home = shooter.homeCommand().withName("FF_Fault_ShooterHomeDisconnected");
        CommandScheduler.getInstance().schedule(home);

        boolean finished = runUntil(
                () -> !CommandScheduler.getInstance().isScheduled(home),
                secondsToCycles(1.6));

        assertFalse(
                finished,
                FullFunctionalityHarness.formatExpectedVsActual(
                        "Shooter home should not report finished/successful homing when hood current threshold is never reached",
                        "finished=false",
                        String.format(
                                Locale.US,
                                "finished=%s resetHoodEncoderCalls=%d maxObservedHoodCurrent=%.2fA threshold=%.2fA hoodPosDeg=%.2f",
                                finished,
                                io.getResetHoodEncoderCalls(),
                                io.getMaxObservedHoodStatorCurrentAmps(),
                                ShooterConstants.HOMING_CURRENT_THRESHOLD_AMPS,
                                Math.toDegrees(io.getHoodPositionRad()))));
    }

    private void runCycles(int cycles) {
        for (int i = 0; i < cycles; i++) {
            SimHooks.stepTiming(LOOP_PERIOD_SEC);
            DriverStationSim.notifyNewData();
            CommandScheduler.getInstance().run();
        }
    }

    private boolean runUntil(BooleanCondition condition, int maxCycles) {
        for (int i = 0; i < maxCycles; i++) {
            if (condition.getAsBoolean()) {
                return true;
            }
            runCycles(1);
        }
        return condition.getAsBoolean();
    }

    private static int secondsToCycles(double seconds) {
        return (int) Math.ceil(seconds / LOOP_PERIOD_SEC);
    }

    @FunctionalInterface
    private interface BooleanCondition {
        boolean getAsBoolean();
    }

    private static final class IntakeDisconnectedIO implements IntakeIO {
        private final double leftPositionRad = Units.rotationsToRadians(IntakeConstants.RETRACTED_POSITION_ROT);
        private final double rightPositionRad = Units.rotationsToRadians(
                IntakeConstants.RIGHT_OPPOSES_LEFT
                        ? -IntakeConstants.RETRACTED_POSITION_ROT
                        : IntakeConstants.RETRACTED_POSITION_ROT);

        private double leftAppliedVolts = 0.0;
        private double rightAppliedVolts = 0.0;
        private double rollerAppliedVolts = 0.0;
        private int resetEncodersCalls = 0;
        private double maxObservedStatorCurrentAmps = 0.0;

        @Override
        public void updateInputs(IntakeIOInputs inputs) {
            double statorCurrentAmps = 0.0;
            maxObservedStatorCurrentAmps = Math.max(maxObservedStatorCurrentAmps, statorCurrentAmps);

            inputs.leftAppliedVolts = leftAppliedVolts;
            inputs.leftPositionRad = leftPositionRad;
            inputs.leftSupplyCurrentAmps = 0.0;
            inputs.leftStatorCurrentAmps = statorCurrentAmps;
            inputs.leftVelocityRpm = 0.0;

            inputs.rightAppliedVolts = rightAppliedVolts;
            inputs.rightPositionRad = rightPositionRad;
            inputs.rightSupplyCurrentAmps = 0.0;
            inputs.rightStatorCurrentAmps = statorCurrentAmps;
            inputs.rightVelocityRpm = 0.0;

            inputs.rollerAppliedVolts = rollerAppliedVolts;
            inputs.rollerPositionRad = 0.0;
            inputs.rollerSupplyCurrentAmps = 0.0;
            inputs.rollerStatorCurrentAmps = 0.0;
            inputs.rollerVelocityRpm = 0.0;
        }

        @Override
        public void setRollerRpm(double rpm) {
            rollerAppliedVolts = Math.signum(rpm) * 2.0;
        }

        @Override
        public void setIntakePosition(
                double leftTargetRot,
                double velocityRotPerSec,
                double accelerationRotPerSecSq,
                double maxVolts) {
            double clampedVolts = Math.min(Math.abs(maxVolts), IntakeConstants.INTAKE_MAX_VOLTS);
            double leftErrorRot = leftTargetRot - Units.radiansToRotations(leftPositionRad);
            double rightTargetRot = IntakeConstants.RIGHT_OPPOSES_LEFT ? -leftTargetRot : leftTargetRot;
            double rightErrorRot = rightTargetRot - Units.radiansToRotations(rightPositionRad);
            leftAppliedVolts = Math.signum(leftErrorRot) * clampedVolts;
            rightAppliedVolts = Math.signum(rightErrorRot) * clampedVolts;
        }

        @Override
        public void setLeftIntakeVoltage(double volts) {
            leftAppliedVolts = volts;
        }

        @Override
        public void setRightIntakeVoltage(double volts) {
            rightAppliedVolts = volts;
        }

        @Override
        public void resetEncoders() {
            resetEncodersCalls++;
        }

        @Override
        public void stopLeftIntake() {
            leftAppliedVolts = 0.0;
        }

        @Override
        public void stopRightIntake() {
            rightAppliedVolts = 0.0;
        }

        @Override
        public void stopRoller() {
            rollerAppliedVolts = 0.0;
        }

        double getLeftPositionRad() {
            return leftPositionRad;
        }

        double getRightPositionRad() {
            return rightPositionRad;
        }

        double getLeftAppliedVolts() {
            return leftAppliedVolts;
        }

        double getRightAppliedVolts() {
            return rightAppliedVolts;
        }

        int getResetEncodersCalls() {
            return resetEncodersCalls;
        }

        double getMaxObservedStatorCurrentAmps() {
            return maxObservedStatorCurrentAmps;
        }
    }

    private static final class ShooterDisconnectedIO implements ShooterIO {
        private final double hoodPositionRad = Units.degreesToRadians(58.0);

        private double leftVelocitySetpointRpm = 0.0;
        private double rightVelocitySetpointRpm = 0.0;
        private double hoodAppliedVolts = 0.0;
        private double hoodStatorCurrentAmps = 0.0;
        private int resetHoodEncoderCalls = 0;
        private double maxObservedHoodStatorCurrentAmps = 0.0;

        @Override
        public void updateInputs(ShooterIOInputs inputs) {
            maxObservedHoodStatorCurrentAmps = Math.max(maxObservedHoodStatorCurrentAmps, hoodStatorCurrentAmps);

            inputs.shooterLeftPositionRad = 0.0;
            inputs.shooterLeftVelocityRpm = leftVelocitySetpointRpm * 0.2;
            inputs.shooterLeftAppliedVolts = 0.0;
            inputs.shooterLeftSupplyCurrentAmps = 0.0;
            inputs.shooterLeftTempCelsius = 25.0;

            inputs.shooterRightPositionRad = 0.0;
            inputs.shooterRightVelocityRpm = rightVelocitySetpointRpm * 0.2;
            inputs.shooterRightAppliedVolts = 0.0;
            inputs.shooterRightSupplyCurrentAmps = 0.0;
            inputs.shooterRightTempCelsius = 25.0;

            inputs.hoodPositionRad = hoodPositionRad;
            inputs.hoodVelocityRpm = 0.0;
            inputs.hoodAppliedVolts = hoodAppliedVolts;
            inputs.hoodSupplyCurrentAmps = hoodStatorCurrentAmps;
            inputs.hoodStatorCurrentAmps = hoodStatorCurrentAmps;
            inputs.hoodTempCelsius = 25.0;

            inputs.kickerVelocityRpm = 0.0;
            inputs.kickerPositionRad = 0.0;
            inputs.kickerAppliedVolts = 0.0;
            inputs.kickerSupplyCurrentAmps = 0.0;
            inputs.kickerTempCelsius = 25.0;
        }

        @Override
        public void setLeftVelocity(double rpm) {
            leftVelocitySetpointRpm = rpm;
        }

        @Override
        public void setRightVelocity(double rpm) {
            rightVelocitySetpointRpm = rpm;
        }

        @Override
        public void setHoodAngle(double angle) {
            hoodAppliedVolts = Math.signum(angle - hoodPositionRad) * 2.0;
        }

        @Override
        public void setHoodVoltage(double volts) {
            hoodAppliedVolts = volts;
            hoodStatorCurrentAmps = 0.0;
        }

        @Override
        public void resetHoodEncoder() {
            resetHoodEncoderCalls++;
        }

        @Override
        public void stop() {
            hoodAppliedVolts = 0.0;
            leftVelocitySetpointRpm = 0.0;
            rightVelocitySetpointRpm = 0.0;
        }

        int getResetHoodEncoderCalls() {
            return resetHoodEncoderCalls;
        }

        double getMaxObservedHoodStatorCurrentAmps() {
            return maxObservedHoodStatorCurrentAmps;
        }

        double getHoodPositionRad() {
            return hoodPositionRad;
        }
    }
}
