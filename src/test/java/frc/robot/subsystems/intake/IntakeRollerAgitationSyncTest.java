package frc.robot.subsystems.intake;

import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import java.util.Locale;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class IntakeRollerAgitationSyncTest {
    private static final double LOOP_PERIOD_SEC = 0.02;
    private static final double TARGET_EPSILON_ROT = 1e-9;

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
        DriverStationSim.setAutonomous(false);
        DriverStationSim.setTest(false);
        DriverStationSim.setEStop(false);
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
    void wiggleWaitsForLaggingSideBeforeAdvancingBackToBaseline() {
        LaggyAgitationIntakeIO io = new LaggyAgitationIntakeIO();
        Intake intake = new Intake(io);
        runCycles(5);

        intake.setExtended(true);
        assertTrue(runUntil(intake::isExtended, 400), "Precondition failed: intake should settle extended.");

        Command wiggle = intake.driverTriggerSpinRollerCommand();
        CommandScheduler.getInstance().schedule(wiggle);

        assertTrue(
                runUntil(
                        () -> Math.abs(io.getLastLeftTargetRot() - IntakeConstants.DRIVER_TRIGGER_WIGGLE_PEAK_ROT)
                                <= TARGET_EPSILON_ROT,
                        120),
                "Expected wiggle to eventually command the peak target.");

        double expectedRightPeakRot = IntakeConstants.RIGHT_OPPOSES_LEFT
                ? -IntakeConstants.DRIVER_TRIGGER_WIGGLE_PEAK_ROT
                : IntakeConstants.DRIVER_TRIGGER_WIGGLE_PEAK_ROT;
        double rightErrorAtPeak = Math.abs(io.getRightPositionRot() - expectedRightPeakRot);
        assertTrue(
                rightErrorAtPeak > IntakeConstants.POSITION_TOLERANCE_ROT + 0.5,
                String.format(
                        Locale.US,
                        "Precondition failed: lagging side should still be well away from peak when the first peak command is issued. rightError=%.3f",
                        rightErrorAtPeak));

        runCycles(secondsToCycles(IntakeConstants.DRIVER_TRIGGER_WIGGLE_SWITCH_INTERVAL_SEC + 0.06));

        assertTrue(
                Math.abs(io.getLastLeftTargetRot() - IntakeConstants.DRIVER_TRIGGER_WIGGLE_PEAK_ROT) <= TARGET_EPSILON_ROT,
                String.format(
                        Locale.US,
                        "Expected wiggle to hold the peak until both sides are in phase. lastTarget=%.3f peak=%.3f rightPos=%.3f",
                        io.getLastLeftTargetRot(),
                        IntakeConstants.DRIVER_TRIGGER_WIGGLE_PEAK_ROT,
                        io.getRightPositionRot()));

        assertTrue(
                runUntil(
                        () -> Math.abs(io.getLastLeftTargetRot() - IntakeConstants.DRIVER_TRIGGER_WIGGLE_BASELINE_ROT)
                                <= TARGET_EPSILON_ROT,
                        220),
                "Expected wiggle to return to baseline after the lagging side catches up.");

        CommandScheduler.getInstance().cancel(wiggle);
        runCycles(2);
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

    private static final class LaggyAgitationIntakeIO implements IntakeIO {
        private static final double LEFT_MAX_STEP_ROT = 0.90;
        private static final double RIGHT_MAX_STEP_ROT = 0.12;

        private double leftPositionRot = IntakeConstants.RETRACTED_POSITION_ROT;
        private double rightPositionRot = mirrored(IntakeConstants.RETRACTED_POSITION_ROT);
        private double leftTargetRot = IntakeConstants.RETRACTED_POSITION_ROT;
        private double rightTargetRot = mirrored(IntakeConstants.RETRACTED_POSITION_ROT);
        private double lastLeftTargetRot = IntakeConstants.RETRACTED_POSITION_ROT;
        private double rollerAppliedVolts = 0.0;

        @Override
        public void updateInputs(IntakeIOInputs inputs) {
            leftPositionRot = advanceTowards(leftPositionRot, leftTargetRot, LEFT_MAX_STEP_ROT);
            rightPositionRot = advanceTowards(rightPositionRot, rightTargetRot, RIGHT_MAX_STEP_ROT);

            inputs.leftAppliedVolts = 0.0;
            inputs.leftPositionRad = Units.rotationsToRadians(leftPositionRot);
            inputs.leftSupplyCurrentAmps = 0.0;
            inputs.leftStatorCurrentAmps = 0.0;
            inputs.leftVelocityRpm = 0.0;

            inputs.rightAppliedVolts = 0.0;
            inputs.rightPositionRad = Units.rotationsToRadians(rightPositionRot);
            inputs.rightSupplyCurrentAmps = 0.0;
            inputs.rightStatorCurrentAmps = 0.0;
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
            this.leftTargetRot = leftTargetRot;
            this.rightTargetRot = mirrored(leftTargetRot);
            this.lastLeftTargetRot = leftTargetRot;
        }

        @Override
        public void stopRoller() {
            rollerAppliedVolts = 0.0;
        }

        double getLastLeftTargetRot() {
            return lastLeftTargetRot;
        }

        double getRightPositionRot() {
            return rightPositionRot;
        }

        private static double mirrored(double leftTargetRot) {
            return IntakeConstants.RIGHT_OPPOSES_LEFT ? -leftTargetRot : leftTargetRot;
        }

        private static double advanceTowards(double current, double target, double maxStep) {
            double error = target - current;
            if (Math.abs(error) <= maxStep) {
                return target;
            }
            return current + Math.copySign(MathUtil.clamp(Math.abs(error), 0.0, maxStep), error);
        }
    }
}
