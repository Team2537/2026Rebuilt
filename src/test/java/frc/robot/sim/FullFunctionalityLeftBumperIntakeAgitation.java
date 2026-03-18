package frc.robot.sim;

import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.intake.IntakeConstants;
import java.util.Locale;
import org.junit.jupiter.api.Test;

final class FullFunctionalityLeftBumperIntakeAgitation {
    private static final double POSITION_TOLERANCE_IN = 0.18;

    @Test
    void leftBumperHoldWigglesExtendedIntakeAtConfiguredCadence() {
        try (FullFunctionalityHarness.Context context = new FullFunctionalityHarness.Context(false)) {
            context.setTeleopEnabled();
            context.container.teleopInit();
            context.runCycles(60);

            extendAndSettleAtBaseline(context);

            context.driverControllerSim.setLeftBumperButton(true);
            boolean sawInitialBaseline = context.runUntil(
                    () -> Math.abs(commandedLeftTargetInches(context) - IntakeConstants.DRIVER_AGITATION_BASELINE_IN)
                            <= POSITION_TOLERANCE_IN,
                    20);
            double lowPhaseTargetIn = commandedLeftTargetInches(context);
            boolean reachedPeak = context.runUntil(
                    () -> Math.abs(commandedLeftTargetInches(context) - IntakeConstants.DRIVER_AGITATION_PEAK_IN)
                            <= POSITION_TOLERANCE_IN,
                    30);
            double highPhaseTargetIn = commandedLeftTargetInches(context);
            boolean returnedLow = context.runUntil(
                    () -> Math.abs(commandedLeftTargetInches(context) - IntakeConstants.DRIVER_AGITATION_BASELINE_IN)
                            <= POSITION_TOLERANCE_IN,
                    30);
            double returnLowPhaseTargetIn = commandedLeftTargetInches(context);

            assertTrue(sawInitialBaseline, "Expected left-bumper agitation to begin from the baseline extension target.");
            assertTrue(reachedPeak, "Expected left-bumper agitation to reach the configured peak target.");
            assertTrue(returnedLow, "Expected left-bumper agitation to return to the baseline target after the peak.");

            assertNearInches(
                    lowPhaseTargetIn,
                    IntakeConstants.DRIVER_AGITATION_BASELINE_IN,
                    "Expected the initial left-bumper agitation phase to stay at the configured baseline.");
            assertNearInches(
                    highPhaseTargetIn,
                    IntakeConstants.DRIVER_AGITATION_PEAK_IN,
                    "Expected the second left-bumper agitation phase to reach the configured peak.");
            assertNearInches(
                    returnLowPhaseTargetIn,
                    IntakeConstants.DRIVER_AGITATION_BASELINE_IN,
                    "Expected the third left-bumper agitation phase to return to the configured baseline.");
            assertTrue(
                    highPhaseTargetIn > lowPhaseTargetIn + 0.22 && highPhaseTargetIn > returnLowPhaseTargetIn + 0.22,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "Left-bumper agitation should create a visible extension swing above baseline",
                            "highPhase > both low phases by >0.22 in",
                            String.format(
                                    Locale.US,
                                    "low1=%.3f high=%.3f low2=%.3f",
                                    lowPhaseTargetIn,
                                    highPhaseTargetIn,
                                    returnLowPhaseTargetIn)));
            assertTrue(
                    context.recorder.runningCount("IntakeAgitate") >= 1,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "Expected left bumper hold to keep IntakeAgitate running",
                            "IntakeAgitate runningCount>=1",
                            "IntakeAgitate runningCount=" + context.recorder.runningCount("IntakeAgitate")));
            assertTrue(
                    Math.abs(context.intakeInputs.rollerAppliedVolts) > 0.5,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "Expected left bumper hold to command nonzero roller voltage during agitation",
                            "abs(rollerAppliedVolts)>0.5",
                            String.format(Locale.US, "rollerAppliedVolts=%.3f", context.intakeInputs.rollerAppliedVolts)));
        }
    }

    @Test
    void releasingLeftBumperReturnsExtendedIntakeToConfiguredBaseline() {
        try (FullFunctionalityHarness.Context context = new FullFunctionalityHarness.Context(false)) {
            context.setTeleopEnabled();
            context.container.teleopInit();
            context.runCycles(60);

            extendAndSettleAtBaseline(context);

            context.driverControllerSim.setLeftBumperButton(true);
            context.runCycles(40);
            double peakBeforeReleaseIn = commandedLeftTargetInches(context);
            assertTrue(
                    peakBeforeReleaseIn >= IntakeConstants.DRIVER_AGITATION_PEAK_IN - POSITION_TOLERANCE_IN,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "Release recovery test should start from the left-bumper agitation peak",
                            String.format(
                                    Locale.US,
                                    "leftInches>=%.3f",
                                    IntakeConstants.DRIVER_AGITATION_PEAK_IN - POSITION_TOLERANCE_IN),
                            String.format(Locale.US, "leftInches=%.3f", peakBeforeReleaseIn)));

            context.driverControllerSim.setLeftBumperButton(false);
            boolean returnedToBaseline = context.runUntil(
                    () -> Math.abs(leftPositionInches(context) - IntakeConstants.DRIVER_AGITATION_BASELINE_IN)
                            <= POSITION_TOLERANCE_IN,
                    120);

            assertTrue(
                    returnedToBaseline,
                    "Expected left-bumper release to return the intake to the configured baseline extension.");
            assertTrue(
                    Math.abs(commandedLeftTargetInches(context) - IntakeConstants.DRIVER_AGITATION_BASELINE_IN)
                            <= POSITION_TOLERANCE_IN,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "Releasing left bumper from agitation should restore the configured baseline target",
                            String.format(
                                    Locale.US,
                                    "commandedTargetInches=%.3f±%.3f",
                                    IntakeConstants.DRIVER_AGITATION_BASELINE_IN,
                                    POSITION_TOLERANCE_IN),
                            String.format(Locale.US, "commandedTargetInches=%.3f", commandedLeftTargetInches(context))));
            assertTrue(
                    context.recorder.runningCount("IntakeAgitate") == 0,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "After left-bumper release, the agitation hold command should no longer be running",
                            "IntakeAgitate runningCount=0",
                            "IntakeAgitate runningCount=" + context.recorder.runningCount("IntakeAgitate")));
        }
    }

    private static void extendAndSettleAtBaseline(FullFunctionalityHarness.Context context) {
        context.intake.setExtended(true);
        boolean settled = context.runUntil(
                () -> Math.abs(leftPositionInches(context) - IntakeConstants.DRIVER_AGITATION_BASELINE_IN)
                        <= POSITION_TOLERANCE_IN,
                220);
        assertTrue(settled, "Expected intake to settle at the configured left-bumper baseline before agitation testing.");
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
