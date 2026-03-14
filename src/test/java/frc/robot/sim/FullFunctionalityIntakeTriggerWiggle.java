package frc.robot.sim;

import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.intake.IntakeConstants;
import java.util.Locale;
import org.junit.jupiter.api.Test;

final class FullFunctionalityIntakeTriggerWiggle {
    private static final double POSITION_TOLERANCE_IN = 0.18;

    @Test
    void leftTriggerHoldWigglesExtendedIntakeAtHalfSecondCadence() {
        try (FullFunctionalityHarness.Context context = new FullFunctionalityHarness.Context(false)) {
            context.setTeleopEnabled();
            context.container.teleopInit();
            context.runCycles(60);

            extendAndSettleAtBaseline(context);

            context.driverControllerSim.setLeftTriggerAxis(1.0);
            context.runCycles(15);
            double lowPhaseTargetIn = commandedLeftTargetInches(context);
            context.runCycles(25);
            double highPhaseTargetIn = commandedLeftTargetInches(context);
            context.runCycles(25);
            double returnLowPhaseTargetIn = commandedLeftTargetInches(context);

            assertNearInches(
                    lowPhaseTargetIn,
                    IntakeConstants.DRIVER_TRIGGER_WIGGLE_BASELINE_IN,
                    "Expected the initial left-trigger wiggle phase to stay at the 11.0 inch baseline.");
            assertNearInches(
                    highPhaseTargetIn,
                    IntakeConstants.DRIVER_TRIGGER_WIGGLE_PEAK_IN,
                    "Expected the second half-second of left-trigger hold to reach the 11.4 inch wiggle peak.");
            assertNearInches(
                    returnLowPhaseTargetIn,
                    IntakeConstants.DRIVER_TRIGGER_WIGGLE_BASELINE_IN,
                    "Expected the third half-second of left-trigger hold to return to the 11.0 inch baseline.");
            assertTrue(
                    highPhaseTargetIn > lowPhaseTargetIn + 0.22 && highPhaseTargetIn > returnLowPhaseTargetIn + 0.22,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "Left-trigger wiggle should create a visible extension swing above baseline",
                            "highPhase > both low phases by >0.22 in",
                            String.format(
                                    Locale.US,
                                    "low1=%.3f high=%.3f low2=%.3f",
                                    lowPhaseTargetIn,
                                    highPhaseTargetIn,
                                    returnLowPhaseTargetIn)));
            assertTrue(
                    Math.abs(context.intakeInputs.rollerAppliedVolts) > 0.5,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "Left-trigger hold should keep the intake roller actively driven during wiggle",
                            "abs(rollerAppliedVolts)>0.5",
                            String.format(Locale.US, "rollerAppliedVolts=%.3f", context.intakeInputs.rollerAppliedVolts)));
        }
    }

    @Test
    void releasingLeftTriggerReturnsExtendedIntakeToBaseline() {
        try (FullFunctionalityHarness.Context context = new FullFunctionalityHarness.Context(false)) {
            context.setTeleopEnabled();
            context.container.teleopInit();
            context.runCycles(60);

            extendAndSettleAtBaseline(context);

            context.driverControllerSim.setLeftTriggerAxis(1.0);
            context.runCycles(40);
            double peakBeforeReleaseIn = commandedLeftTargetInches(context);
            assertTrue(
                    peakBeforeReleaseIn >= IntakeConstants.DRIVER_TRIGGER_WIGGLE_PEAK_IN - POSITION_TOLERANCE_IN,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "Release recovery test should start from the wiggle peak",
                            String.format(
                                    Locale.US,
                                    "leftInches>=%.3f",
                                    IntakeConstants.DRIVER_TRIGGER_WIGGLE_PEAK_IN - POSITION_TOLERANCE_IN),
                            String.format(Locale.US, "leftInches=%.3f", peakBeforeReleaseIn)));

            context.driverControllerSim.setLeftTriggerAxis(0.0);
            boolean returnedToBaseline = context.runUntil(
                    () -> Math.abs(leftPositionInches(context) - IntakeConstants.DRIVER_TRIGGER_WIGGLE_BASELINE_IN)
                            <= POSITION_TOLERANCE_IN,
                    120);

            assertTrue(returnedToBaseline, "Expected left-trigger release to return the intake to the 11.0 inch baseline.");
            assertTrue(
                    Math.abs(commandedLeftTargetInches(context) - IntakeConstants.DRIVER_TRIGGER_WIGGLE_BASELINE_IN)
                            <= POSITION_TOLERANCE_IN,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "Releasing left trigger from the wiggle should restore the 11.0 inch extension target",
                            String.format(
                                    Locale.US,
                                    "commandedTargetInches=%.3f±%.3f",
                                    IntakeConstants.DRIVER_TRIGGER_WIGGLE_BASELINE_IN,
                                    POSITION_TOLERANCE_IN),
                            String.format(Locale.US, "commandedTargetInches=%.3f", commandedLeftTargetInches(context))));
            assertTrue(
                    context.recorder.runningCount("IntakeSpinRoller") == 0,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "After left-trigger release, the fast roller hold command should no longer be running",
                            "IntakeSpinRoller runningCount=0",
                            "IntakeSpinRoller runningCount=" + context.recorder.runningCount("IntakeSpinRoller")));
        }
    }

    @Test
    void directSpinRollerCommandDoesNotWiggleExtension() {
        try (FullFunctionalityHarness.Context context = new FullFunctionalityHarness.Context(false)) {
            context.setTeleopEnabled();
            context.container.teleopInit();
            context.runCycles(60);

            extendAndSettleAtBaseline(context);

            Command directSpinRoller = context.intake.spinRoller().withName("FF_DirectIntakeSpinRoller");
            CommandScheduler.getInstance().schedule(directSpinRoller);

            double minTargetInches = Double.POSITIVE_INFINITY;
            double maxTargetInches = Double.NEGATIVE_INFINITY;
            for (int i = 0; i < 90; i++) {
                context.runCycles(1);
                double targetInches = commandedLeftTargetInches(context);
                minTargetInches = Math.min(minTargetInches, targetInches);
                maxTargetInches = Math.max(maxTargetInches, targetInches);
            }

            CommandScheduler.getInstance().cancel(directSpinRoller);
            context.runCycles(10);

            assertTrue(
                    maxTargetInches - minTargetInches <= 0.02,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "Direct spinRoller command should not inherit the driver-only wiggle behavior",
                            "maxTargetInches-minTargetInches<=0.02",
                            String.format(
                                    Locale.US,
                                    "minTargetInches=%.3f maxTargetInches=%.3f",
                                    minTargetInches,
                                    maxTargetInches)));
            assertTrue(
                    minTargetInches >= IntakeConstants.DRIVER_TRIGGER_WIGGLE_BASELINE_IN - 0.02
                            && maxTargetInches <= IntakeConstants.DRIVER_TRIGGER_WIGGLE_BASELINE_IN + 0.02,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "Direct spinRoller should keep the intake parked at the 11.0 inch baseline extension",
                            String.format(
                                    Locale.US,
                                    "%.3f±%.3f in",
                                    IntakeConstants.DRIVER_TRIGGER_WIGGLE_BASELINE_IN,
                                    0.02),
                            String.format(
                                    Locale.US,
                                    "minTargetInches=%.3f maxTargetInches=%.3f",
                                    minTargetInches,
                                    maxTargetInches)));
        }
    }

    private static void extendAndSettleAtBaseline(FullFunctionalityHarness.Context context) {
        context.intake.setExtended(true);
        boolean settled = context.runUntil(
                () -> Math.abs(leftPositionInches(context) - IntakeConstants.DRIVER_TRIGGER_WIGGLE_BASELINE_IN)
                        <= POSITION_TOLERANCE_IN,
                220);
        assertTrue(settled, "Expected intake to settle at the 11.0 inch extended baseline before wiggle testing.");
        context.runCycles(8);
    }

    private static double leftPositionInches(FullFunctionalityHarness.Context context) {
        return Units.radiansToRotations(context.intakeInputs.leftPositionRad) / IntakeConstants.MOTOR_REV_PER_IN;
    }

    private static double commandedLeftTargetInches(FullFunctionalityHarness.Context context) {
        Double targetRot = FullFunctionalityHarness.getPrivateField(context.intake, "commandedLeftTargetRot", Double.class);
        return targetRot / IntakeConstants.MOTOR_REV_PER_IN;
    }

    private static void assertNearInches(double actualInches, double expectedInches, String message) {
        assertTrue(
                Math.abs(actualInches - expectedInches) <= POSITION_TOLERANCE_IN,
                FullFunctionalityHarness.formatExpectedVsActual(
                        message,
                        String.format(Locale.US, "%.3f±%.3f in", expectedInches, POSITION_TOLERANCE_IN),
                        String.format(Locale.US, "%.3f in", actualInches)));
    }
}
