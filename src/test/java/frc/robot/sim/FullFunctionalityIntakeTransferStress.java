package frc.robot.sim;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.intake.IntakeConstants;
import java.util.List;
import java.util.Locale;
import org.junit.jupiter.api.Test;

final class FullFunctionalityIntakeTransferStress {
    @Test
    void conflictingIntakeCommandsCloseLifecycleAndEndRetractedWithProgress() {
        try (FullFunctionalityHarness.Context context = new FullFunctionalityHarness.Context(false)) {
            context.setTeleopEnabled();
            context.container.teleopInit();
            context.runCycles(80);

            context.intake.setExtended(false);
            context.runCycles(12);

            Command extend = context.intake.extendCommand().withName("FF_StressIntakeExtendOverlap");
            Command retract = context.intake.retractCommand().withName("FF_StressIntakeRetractOverlap");
            Command slowRetract = context.intake.slowRetractCommand().withName("FF_StressIntakeSlowRetractOverlap");

            FullFunctionalityHarness.CommandCounts extendBefore =
                    context.recorder.getCounts("FF_StressIntakeExtendOverlap");
            FullFunctionalityHarness.CommandCounts retractBefore =
                    context.recorder.getCounts("FF_StressIntakeRetractOverlap");
            FullFunctionalityHarness.CommandCounts slowRetractBefore =
                    context.recorder.getCounts("FF_StressIntakeSlowRetractOverlap");

            double initialLeftRot = leftPositionRot(context);
            double peakLeftRot = initialLeftRot;

            CommandScheduler.getInstance().schedule(extend);
            for (int i = 0; i < 20; i++) {
                context.runCycles(1);
                peakLeftRot = Math.max(peakLeftRot, leftPositionRot(context));
            }

            CommandScheduler.getInstance().schedule(retract);
            for (int i = 0; i < 8; i++) {
                context.runCycles(1);
                peakLeftRot = Math.max(peakLeftRot, leftPositionRot(context));
            }

            CommandScheduler.getInstance().schedule(slowRetract);
            boolean slowRetractFinished =
                    context.runUntil(() -> !CommandScheduler.getInstance().isScheduled(slowRetract), 360);
            assertTrue(
                    slowRetractFinished,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "Slow retract should finish after extend/retract overlap contention",
                            "slowRetract finished",
                            "slowRetract still scheduled"));
            context.runCycles(20);

            FullFunctionalityHarness.CommandCounts extendDelta =
                    context.recorder.getCounts("FF_StressIntakeExtendOverlap").minus(extendBefore);
            FullFunctionalityHarness.CommandCounts retractDelta =
                    context.recorder.getCounts("FF_StressIntakeRetractOverlap").minus(retractBefore);
            FullFunctionalityHarness.CommandCounts slowRetractDelta =
                    context.recorder.getCounts("FF_StressIntakeSlowRetractOverlap").minus(slowRetractBefore);

            assertTrue(
                    extendDelta.starts() >= 1,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "Overlap scenario should start extend",
                            "starts>=1",
                            extendDelta));
            assertTrue(
                    extendDelta.interrupts() >= 1,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "Retract scheduling should interrupt extend",
                            "interrupts>=1",
                            extendDelta));
            assertTrue(
                    retractDelta.starts() >= 1,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "Overlap scenario should start retract",
                            "starts>=1",
                            retractDelta));
            assertTrue(
                    retractDelta.interrupts() >= 1,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "Slow retract scheduling should interrupt retract",
                            "interrupts>=1",
                            retractDelta));
            assertTrue(
                    slowRetractDelta.starts() >= 1 && slowRetractDelta.finishes() >= 1,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "Slow retract should start and finish after winning arbitration",
                            "starts>=1 and finishes>=1",
                            slowRetractDelta));

            assertLifecycleClosed("FF_StressIntakeExtendOverlap", extendDelta);
            assertLifecycleClosed("FF_StressIntakeRetractOverlap", retractDelta);
            assertLifecycleClosed("FF_StressIntakeSlowRetractOverlap", slowRetractDelta);

            assertHasLatestRun(context, "FF_StressIntakeExtendOverlap");
            assertHasLatestRun(context, "FF_StressIntakeRetractOverlap");
            assertHasLatestRun(context, "FF_StressIntakeSlowRetractOverlap");

            double finalLeftRot = leftPositionRot(context);
            double finalRightRot = rightPositionRot(context);
            double expectedRightRetractRot =
                    IntakeConstants.RIGHT_OPPOSES_LEFT ? -IntakeConstants.RETRACTED_POSITION_ROT : IntakeConstants.RETRACTED_POSITION_ROT;

            assertTrue(
                    peakLeftRot > initialLeftRot + 0.8,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "Conflicting scenario should show measurable extend progress before retraction wins",
                            "peakLeft > initialLeft+0.8 rot",
                            String.format(Locale.US, "initial=%.3f peak=%.3f", initialLeftRot, peakLeftRot)));
            assertTrue(
                    finalLeftRot < peakLeftRot - 0.8,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "Final state should make retraction progress from the peak extended position",
                            "finalLeft < peakLeft-0.8 rot",
                            String.format(Locale.US, "peak=%.3f final=%.3f", peakLeftRot, finalLeftRot)));
            assertTrue(
                    Math.abs(finalLeftRot - IntakeConstants.RETRACTED_POSITION_ROT) <= 1.0,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "Final left intake position should settle near retract target",
                            IntakeConstants.RETRACTED_POSITION_ROT + "±1.0 rot",
                            finalLeftRot));
            assertTrue(
                    Math.abs(finalRightRot - expectedRightRetractRot) <= 1.1,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "Final right intake position should settle near retract target",
                            expectedRightRetractRot + "±1.1 rot",
                            finalRightRot));
            assertTrue(
                    !context.intake.isExtended(),
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "Conflicting intake commands should end with intake marked retracted",
                            "intake.isExtended()=false",
                            context.intake.isExtended()));
        }
    }

    @Test
    void rapidSpamOfBXYAndLeftTriggerEndsInDeterministicState() {
        try (FullFunctionalityHarness.Context context = new FullFunctionalityHarness.Context(false)) {
            context.setTeleopEnabled();
            context.container.teleopInit();
            context.runCycles(70);

            context.intake.setExtended(true);
            context.runCycles(12);

            FullFunctionalityHarness.CommandCounts snapBefore = context.recorder.getCounts("DriveHeadingSnap");
            FullFunctionalityHarness.CommandCounts rollerBefore = context.recorder.getCounts("IntakeSpinRoller");

            final int spamRounds = 7;
            for (int i = 0; i < spamRounds; i++) {
                context.driverControllerSim.setBButton(true);
                context.driverControllerSim.setXButton(true);
                context.driverControllerSim.setLeftTriggerAxis(1.0);
                context.driverControllerSim.setYButton(true);
                context.runCycles(2);

                context.driverControllerSim.setBButton(false);
                context.driverControllerSim.setXButton(false);
                context.driverControllerSim.setLeftTriggerAxis(0.0);
                context.driverControllerSim.setYButton(false);
                context.runCycles(2);
            }

            context.runCycles(24);
            context.driverControllerSim.setLeftTriggerAxis(1.0);
            context.runCycles(4);
            context.driverControllerSim.setLeftTriggerAxis(0.0);
            context.runCycles(4);
            context.driverControllerSim.setLeftTriggerAxis(1.0);
            context.runCycles(4);
            context.driverControllerSim.setLeftTriggerAxis(0.0);
            boolean finalRetracted = context.runUntil(() -> !context.intake.isExtended(), 240);
            assertTrue(finalRetracted, "Expected final left-trigger double tap to retract the intake.");
            context.clearControllerState();
            context.runCycles(24);

            FullFunctionalityHarness.CommandCounts snapDelta =
                    context.recorder.getCounts("DriveHeadingSnap").minus(snapBefore);
            FullFunctionalityHarness.CommandCounts rollerDelta =
                    context.recorder.getCounts("IntakeSpinRoller").minus(rollerBefore);

            assertTrue(
                    snapDelta.starts() >= spamRounds,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "B/X/Y spam should trigger many heading snaps",
                            "starts>=" + spamRounds,
                            snapDelta));
            assertTrue(
                    rollerDelta.starts() >= spamRounds && rollerDelta.interrupts() >= spamRounds,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "Left-trigger spam should repeatedly start and interrupt intake roller",
                            "starts>=" + spamRounds + " and interrupts>=" + spamRounds,
                            rollerDelta));

            assertLifecycleClosed("DriveHeadingSnap", snapDelta);
            assertLifecycleClosed("IntakeSpinRoller", rollerDelta);

            assertTrue(
                    context.recorder.runningCount("DriveHeadingSnap") == 0
                            && context.recorder.runningCount("IntakeSpinRoller") == 0,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "No spammed intake/transfer commands should remain running after release",
                            "runningCount=0 for snap/roller",
                            String.format(
                                    Locale.US,
                                    "snap=%d roller=%d",
                                    context.recorder.runningCount("DriveHeadingSnap"),
                                    context.recorder.runningCount("IntakeSpinRoller"))));
            assertTrue(
                    !context.intake.isExtended(),
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "Rapid B/X/LT spam sequence should end retracted after final left-trigger double tap",
                            "intake.isExtended()=false",
                            context.intake.isExtended()));
            assertTrue(
                    Math.abs(context.intakeInputs.rollerAppliedVolts) < 1e-6,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "After left-trigger spam release, intake roller voltage should be zero",
                            "rollerAppliedVolts=0",
                            String.format(Locale.US, "rollerAppliedVolts=%.3f", context.intakeInputs.rollerAppliedVolts)));
            assertTrue(
                    Math.abs(context.transferInputs.appliedVolts) < 1e-6,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "After Y spam release, transfer voltage should be zero",
                            "transferAppliedVolts=0",
                            String.format(Locale.US, "transferAppliedVolts=%.3f", context.transferInputs.appliedVolts)));
        }
    }

    @Test
    void transferRunReverseStopArbitrateUnderRepeatedSchedulingAndCancel() {
        try (FullFunctionalityHarness.Context context = new FullFunctionalityHarness.Context(false)) {
            context.setTeleopEnabled();
            context.container.teleopInit();
            context.runCycles(40);

            Command transferRun = context.transfer.runCommand().withName("FF_StressTransferRun");
            Command transferReverse = context.transfer.reverseCommand().withName("FF_StressTransferReverse");
            Command transferStop = context.transfer.stopCommand().withName("FF_StressTransferStop");

            FullFunctionalityHarness.CommandCounts runBefore = context.recorder.getCounts("FF_StressTransferRun");
            FullFunctionalityHarness.CommandCounts reverseBefore = context.recorder.getCounts("FF_StressTransferReverse");
            FullFunctionalityHarness.CommandCounts stopBefore = context.recorder.getCounts("FF_StressTransferStop");

            double maxForwardVolts = Double.NEGATIVE_INFINITY;
            double minReverseVolts = Double.POSITIVE_INFINITY;
            double maxAbsVoltsWhenStopped = 0.0;
            int rounds = 6;
            for (int i = 0; i < rounds; i++) {
                CommandScheduler.getInstance().schedule(transferRun);
                context.runCycles(4);
                maxForwardVolts = Math.max(maxForwardVolts, context.transferInputs.appliedVolts);

                CommandScheduler.getInstance().schedule(transferReverse);
                context.runCycles(4);
                minReverseVolts = Math.min(minReverseVolts, context.transferInputs.appliedVolts);

                CommandScheduler.getInstance().schedule(transferStop);
                context.runCycles(2);
                maxAbsVoltsWhenStopped = Math.max(maxAbsVoltsWhenStopped, Math.abs(context.transferInputs.appliedVolts));

                CommandScheduler.getInstance().schedule(transferRun);
                context.runCycles(3);
                CommandScheduler.getInstance().cancel(transferRun);
                context.runCycles(2);
                maxAbsVoltsWhenStopped = Math.max(maxAbsVoltsWhenStopped, Math.abs(context.transferInputs.appliedVolts));
            }

            context.runCycles(8);

            FullFunctionalityHarness.CommandCounts runDelta =
                    context.recorder.getCounts("FF_StressTransferRun").minus(runBefore);
            FullFunctionalityHarness.CommandCounts reverseDelta =
                    context.recorder.getCounts("FF_StressTransferReverse").minus(reverseBefore);
            FullFunctionalityHarness.CommandCounts stopDelta =
                    context.recorder.getCounts("FF_StressTransferStop").minus(stopBefore);

            assertEquals(
                    rounds * 2,
                    runDelta.starts(),
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "TransferRun should start twice per arbitration round",
                            "starts=" + (rounds * 2),
                            runDelta));
            assertEquals(
                    rounds * 2,
                    runDelta.interrupts(),
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "Each TransferRun instance should end by interruption (reverse takeover or explicit cancel)",
                            "interrupts=" + (rounds * 2),
                            runDelta));
            assertEquals(
                    rounds,
                    reverseDelta.starts(),
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "TransferReverse should start once per arbitration round",
                            "starts=" + rounds,
                            reverseDelta));
            assertEquals(
                    rounds,
                    reverseDelta.interrupts(),
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "TransferReverse should be interrupted by TransferStop once per round",
                            "interrupts=" + rounds,
                            reverseDelta));
            assertEquals(
                    rounds,
                    stopDelta.starts(),
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "TransferStop should start once per arbitration round",
                            "starts=" + rounds,
                            stopDelta));
            assertEquals(
                    rounds,
                    stopDelta.finishes(),
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "TransferStop should finish once per arbitration round",
                            "finishes=" + rounds,
                            stopDelta));

            assertLifecycleClosed("FF_StressTransferRun", runDelta);
            assertLifecycleClosed("FF_StressTransferReverse", reverseDelta);
            assertLifecycleClosed("FF_StressTransferStop", stopDelta);

            assertHasLatestRun(context, "FF_StressTransferRun");
            assertHasLatestRun(context, "FF_StressTransferReverse");
            assertHasLatestRun(context, "FF_StressTransferStop");

            assertTrue(
                    maxForwardVolts > 1.0,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "TransferRun arbitration rounds should drive positive transfer voltage",
                            "maxForwardVolts>1.0",
                            String.format(Locale.US, "maxForwardVolts=%.3f", maxForwardVolts)));
            assertTrue(
                    minReverseVolts < -1.0,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "TransferReverse arbitration rounds should drive negative transfer voltage",
                            "minReverseVolts<-1.0",
                            String.format(Locale.US, "minReverseVolts=%.3f", minReverseVolts)));
            assertTrue(
                    maxAbsVoltsWhenStopped < 1e-6,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "TransferStop/cancel phases should hold transfer voltage at zero",
                            "maxAbsVoltsWhenStopped=0",
                            String.format(Locale.US, "maxAbsVoltsWhenStopped=%.6f", maxAbsVoltsWhenStopped)));
            assertTrue(
                    Math.abs(context.transferInputs.appliedVolts) < 1e-6,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "Transfer arbitration stress should end with transfer output zero",
                            "transferAppliedVolts=0",
                            String.format(Locale.US, "transferAppliedVolts=%.3f", context.transferInputs.appliedVolts)));
        }
    }

    @Test
    void disableWhileIntakeAndTransferDrivenForcesAndKeepsOutputsZero() {
        try (FullFunctionalityHarness.Context context = new FullFunctionalityHarness.Context(false)) {
            context.setTeleopEnabled();
            context.container.teleopInit();
            context.runCycles(60);

            context.intake.setExtended(true);
            context.runCycles(10);

            Command intakeActive = context.intake.spinRoller().withName("FF_DisableIntakeSpinRoller");
            Command transferActive = context.transfer.runCommand().withName("FF_DisableTransferRun");

            FullFunctionalityHarness.CommandCounts intakeBefore =
                    context.recorder.getCounts("FF_DisableIntakeSpinRoller");
            FullFunctionalityHarness.CommandCounts transferBefore =
                    context.recorder.getCounts("FF_DisableTransferRun");

            CommandScheduler.getInstance().schedule(intakeActive);
            CommandScheduler.getInstance().schedule(transferActive);
            context.runCycles(10);

            double preDisableMaxIntakeAbsVolts = Math.max(
                    Math.abs(context.intakeInputs.leftAppliedVolts),
                    Math.max(
                            Math.abs(context.intakeInputs.rightAppliedVolts),
                            Math.abs(context.intakeInputs.rollerAppliedVolts)));
            assertTrue(
                    context.transferInputs.appliedVolts > 1.0,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "Pre-disable stress precondition: transfer should be actively driven",
                            "transferAppliedVolts>1.0",
                            String.format(Locale.US, "transferAppliedVolts=%.3f", context.transferInputs.appliedVolts)));
            assertTrue(
                    preDisableMaxIntakeAbsVolts > 0.5,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "Pre-disable stress precondition: intake should be actively driven",
                            "maxIntakeAbsVolts>0.5",
                            String.format(Locale.US, "maxIntakeAbsVolts=%.3f", preDisableMaxIntakeAbsVolts)));

            context.setDisabled();
            context.container.disabledInit();
            context.runCycles(1);
            context.runCycles(1);

            double maxTransferAbsAfterDisable = Math.abs(context.transferInputs.appliedVolts);
            double maxIntakeAbsAfterDisable = Math.max(
                    Math.abs(context.intakeInputs.leftAppliedVolts),
                    Math.max(
                            Math.abs(context.intakeInputs.rightAppliedVolts),
                            Math.abs(context.intakeInputs.rollerAppliedVolts)));
            for (int i = 0; i < 80; i++) {
                context.runCycles(1);
                maxTransferAbsAfterDisable = Math.max(maxTransferAbsAfterDisable, Math.abs(context.transferInputs.appliedVolts));
                maxIntakeAbsAfterDisable = Math.max(
                        maxIntakeAbsAfterDisable,
                        Math.max(
                                Math.abs(context.intakeInputs.leftAppliedVolts),
                                Math.max(
                                        Math.abs(context.intakeInputs.rightAppliedVolts),
                                        Math.abs(context.intakeInputs.rollerAppliedVolts))));
            }

            FullFunctionalityHarness.CommandCounts intakeDelta =
                    context.recorder.getCounts("FF_DisableIntakeSpinRoller").minus(intakeBefore);
            FullFunctionalityHarness.CommandCounts transferDelta =
                    context.recorder.getCounts("FF_DisableTransferRun").minus(transferBefore);

            assertTrue(
                    intakeDelta.starts() >= 1,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "Disable stress should start intake command before disable",
                            "starts>=1",
                            intakeDelta));
            assertTrue(
                    transferDelta.starts() >= 1,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "Disable stress should start transfer command before disable",
                            "starts>=1",
                            transferDelta));
            assertTrue(
                    intakeDelta.interrupts() >= 1,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "Disabling should interrupt active intake command",
                            "interrupts>=1",
                            intakeDelta));
            assertTrue(
                    transferDelta.interrupts() >= 1,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "Disabling should interrupt active transfer command",
                            "interrupts>=1",
                            transferDelta));

            assertLifecycleClosed("FF_DisableIntakeSpinRoller", intakeDelta);
            assertLifecycleClosed("FF_DisableTransferRun", transferDelta);

            assertHasLatestRun(context, "FF_DisableIntakeSpinRoller");
            assertHasLatestRun(context, "FF_DisableTransferRun");

            assertTrue(
                    maxTransferAbsAfterDisable < 1e-6,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "Disable should force transfer voltage to zero by the first full disabled loop and keep it zero",
                            "maxAbsTransferAfterDisable=0",
                            String.format(Locale.US, "maxAbsTransferAfterDisable=%.6f", maxTransferAbsAfterDisable)));
            assertTrue(
                    maxIntakeAbsAfterDisable < 1e-6,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "Disable should force all intake voltages to zero by the first full disabled loop and keep them zero",
                            "maxAbsIntakeAfterDisable=0",
                            String.format(Locale.US, "maxAbsIntakeAfterDisable=%.6f", maxIntakeAbsAfterDisable)));
            assertTrue(
                    !CommandScheduler.getInstance().isScheduled(intakeActive)
                            && !CommandScheduler.getInstance().isScheduled(transferActive),
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "Disable stress should leave active intake/transfer commands unscheduled",
                            "isScheduled=false for both",
                            String.format(
                                    Locale.US,
                                    "intakeScheduled=%s transferScheduled=%s",
                                    CommandScheduler.getInstance().isScheduled(intakeActive),
                                    CommandScheduler.getInstance().isScheduled(transferActive))));
        }
    }

    private static void assertLifecycleClosed(String commandName, FullFunctionalityHarness.CommandCounts delta) {
        int ends = delta.finishes() + delta.interrupts();
        assertEquals(
                delta.starts(),
                ends,
                FullFunctionalityHarness.formatExpectedVsActual(
                        commandName + " lifecycle should close",
                        "starts==finishes+interrupts",
                        String.format(Locale.US, "%s ends=%d", delta, ends)));
    }

    private static void assertHasLatestRun(FullFunctionalityHarness.Context context, String commandName) {
        FullFunctionalityHarness.CommandRun latest = context.recorder.latestRun(commandName);
        assertNotNull(
                latest,
                FullFunctionalityHarness.formatExpectedVsActual(
                        "Expected recorded run window for command",
                        commandName + " latestRun!=null",
                        "latestRun=null"));
    }

    private static double leftPositionRot(FullFunctionalityHarness.Context context) {
        return Units.radiansToRotations(context.intakeInputs.leftPositionRad);
    }

    private static double rightPositionRot(FullFunctionalityHarness.Context context) {
        return Units.radiansToRotations(context.intakeInputs.rightPositionRad);
    }
}
