package frc.robot.sim;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.intake.IntakeConstants;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import org.junit.jupiter.api.Test;

final class FullFunctionalityChaosMonkeyDivergences {
    private static final int CHAOS_PHASES = 20;
    private static final int TELEOP_CHAOS_CYCLES_PER_PHASE = 52;
    private static final int FINAL_SLOW_RETRACT_HOLD_CYCLES = 220;
    private static final int POST_RELEASE_SETTLE_CYCLES = 80;
    private static final int MANY_SLOW_RETRACT_STARTS = 12;

    private static final double RETRACT_TOLERANCE_LEFT_ROT = 0.55;
    private static final double RETRACT_TOLERANCE_RIGHT_ROT = 0.60;
    private static final double STRICT_FINAL_RETRACT_LEFT_ROT = 0.40;
    private static final double STRICT_FINAL_RETRACT_RIGHT_ROT = 0.45;

    private static final List<String> TRACKED_COMMANDS = List.of(
            "IntakeToggleExtended",
            "IntakeSlowRetract",
            "IntakeSpinRoller",
            "TransferReverse",
            "DriveToggleFieldOriented",
            "DriveResetOdometryAndHeading",
            "DriverShooterHome",
            "StopManipulators",
            "DriveSetOdometryFromUnifiedVision",
            "ShooterTriggerSelectedMode",
            "ShooterTriggerAimAndShoot",
            "ShooterTriggerOverrideAutoAimShoot",
            "ShooterAimOnlySelectedMode",
            "DriveSysIdQuasistaticForward",
            "DriveSysIdQuasistaticReverse",
            "DriveSysIdDynamicForward",
            "DriveSysIdDynamicReverse");

    private static final List<String> HELD_COMMANDS = List.of(
            "IntakeSlowRetract",
            "IntakeSpinRoller",
            "TransferReverse",
            "ShooterTriggerSelectedMode",
            "ShooterTriggerAimAndShoot",
            "ShooterTriggerOverrideAutoAimShoot",
            "ShooterAimOnlySelectedMode",
            "ShooterAimOnly",
            "ShooterAimOnlyOverrideDistance",
            "DriveSysIdQuasistaticForward",
            "DriveSysIdQuasistaticReverse",
            "DriveSysIdDynamicForward",
            "DriveSysIdDynamicReverse");

    @Test
    void chaosMonkeySpamAndModeFlipsExposeExpectedVsActualDivergences() {
        try (FullFunctionalityHarness.Context context = new FullFunctionalityHarness.Context(true)) {
            context.setTeleopEnabled();
            context.container.teleopInit();
            context.runCycles(40);

            Map<String, FullFunctionalityHarness.CommandCounts> before = snapshotCounts(context, TRACKED_COMMANDS);

            int chaosCycles = 0;
            int modeFlipCount = 0;
            for (int phase = 0; phase < CHAOS_PHASES; phase++) {
                chaosCycles += runTeleopChaosBurst(context, phase, TELEOP_CHAOS_CYCLES_PER_PHASE);
                chaosCycles += holdInputsAcrossModeFlip(context, phase);

                context.setDisabled();
                context.container.disabledInit();
                int disabledCycles = 4 + (phase % 3);
                context.runCycles(disabledCycles);
                chaosCycles += disabledCycles;
                modeFlipCount++;

                context.setAutonomousEnabled();
                context.container.autonomousInit();
                int autoCycles = 6 + (phase % 5);
                context.runCycles(autoCycles);
                chaosCycles += autoCycles;
                modeFlipCount++;

                context.setTeleopEnabled();
                context.container.teleopInit();
                int teleopRecoveryCycles = 6 + (phase % 4);
                context.runCycles(teleopRecoveryCycles);
                chaosCycles += teleopRecoveryCycles;
                modeFlipCount++;
            }

            context.setTeleopEnabled();
            context.container.teleopInit();
            context.driverControllerSim.setXButton(true);
            context.driverControllerSim.setLeftTriggerAxis(1.0);
            context.driverControllerSim.setYButton(true);
            context.runCycles(12);
            chaosCycles += 12;
            context.driverControllerSim.setLeftTriggerAxis(0.0);
            context.driverControllerSim.setYButton(false);
            context.runCycles(4);
            chaosCycles += 4;
            // Force a clean whileTrue edge so IntakeSlowRetract is rescheduled for the final long hold.
            context.driverControllerSim.setXButton(false);
            context.runCycles(2);
            chaosCycles += 2;
            context.driverControllerSim.setXButton(true);
            context.runCycles(FINAL_SLOW_RETRACT_HOLD_CYCLES);
            chaosCycles += FINAL_SLOW_RETRACT_HOLD_CYCLES;
            context.driverControllerSim.setXButton(false);

            double chaosDurationSec = chaosCycles * FullFunctionalityHarness.LOOP_PERIOD_SEC;
            assertTrue(
                    chaosDurationSec >= 20.0 && chaosDurationSec <= 40.0,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "Deterministic chaos runtime should stay in [20s, 40s]",
                            "20.0<=chaosDurationSec<=40.0",
                            String.format(Locale.US, "chaosDurationSec=%.3f", chaosDurationSec)));
            assertTrue(
                    modeFlipCount >= CHAOS_PHASES * 3,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "Each chaos phase should flip through disabled->auto->teleop",
                            "modeFlipCount>=" + (CHAOS_PHASES * 3),
                            modeFlipCount));

            context.clearControllerState();
            context.runCycles(POST_RELEASE_SETTLE_CYCLES);

            Map<String, FullFunctionalityHarness.CommandCounts> deltas = deltaCounts(context, before, TRACKED_COMMANDS);
            FullFunctionalityHarness.CommandCounts slowRetractDelta = deltas.get("IntakeSlowRetract");
            FullFunctionalityHarness.CommandCounts toggleDelta = deltas.get("IntakeToggleExtended");
            FullFunctionalityHarness.CommandCounts rollerDelta = deltas.get("IntakeSpinRoller");
            FullFunctionalityHarness.CommandCounts reverseDelta = deltas.get("TransferReverse");
            FullFunctionalityHarness.CommandCounts fieldToggleDelta = deltas.get("DriveToggleFieldOriented");
            FullFunctionalityHarness.CommandCounts resetDelta = deltas.get("DriveResetOdometryAndHeading");

            assertTrue(
                    toggleDelta.starts() >= 20,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "Chaos B spam should trigger many intake toggles",
                            "starts>=20",
                            toggleDelta));
            assertTrue(
                    slowRetractDelta.starts() >= MANY_SLOW_RETRACT_STARTS,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "Chaos X spam should trigger many slow retract starts",
                            "starts>=" + MANY_SLOW_RETRACT_STARTS,
                            slowRetractDelta));
            assertTrue(
                    rollerDelta.starts() >= 20,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "Chaos left-trigger spam should repeatedly start intake roller",
                            "starts>=20",
                            rollerDelta));
            assertTrue(
                    reverseDelta.starts() >= 20,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "Chaos Y spam should repeatedly start transfer reverse",
                            "starts>=20",
                            reverseDelta));
            assertTrue(
                    fieldToggleDelta.starts() >= 12,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "Chaos back-button spam should toggle field mode frequently",
                            "starts>=12",
                            fieldToggleDelta));
            assertTrue(
                    resetDelta.starts() >= 12,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "Chaos start-button spam should reset heading frequently",
                            "starts>=12",
                            resetDelta));

            for (String commandName : TRACKED_COMMANDS) {
                FullFunctionalityHarness.CommandCounts delta = deltas.get(commandName);
                if (delta.starts() == 0) {
                    continue;
                }
                assertLifecycleClosed(commandName, delta);
            }

            for (String commandName : HELD_COMMANDS) {
                assertEquals(
                        0,
                        context.recorder.runningCount(commandName),
                        FullFunctionalityHarness.formatExpectedVsActual(
                                "After chaos release + settle, held command must not remain running",
                                commandName + " runningCount=0",
                                commandName + " runningCount=" + context.recorder.runningCount(commandName)));
            }

            assertTrue(
                    Math.abs(context.intakeInputs.rollerAppliedVolts) < 1e-6,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "After chaos release + settle, intake roller output should be zero",
                            "rollerAppliedVolts=0",
                            String.format(Locale.US, "rollerAppliedVolts=%.3f", context.intakeInputs.rollerAppliedVolts)));
            assertTrue(
                    Math.abs(context.transferInputs.appliedVolts) < 1e-6,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "After chaos release + settle, transfer output should be zero",
                            "transferAppliedVolts=0",
                            String.format(Locale.US, "transferAppliedVolts=%.3f", context.transferInputs.appliedVolts)));

            double leftRot = leftPositionRot(context);
            double rightRot = rightPositionRot(context);
            double expectedRightRetractRot =
                    IntakeConstants.RIGHT_OPPOSES_LEFT ? -IntakeConstants.RETRACTED_POSITION_ROT : IntakeConstants.RETRACTED_POSITION_ROT;

            assertTrue(
                    !context.intake.isExtended(),
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "Final intake logical state should be retracted after chaos",
                            "intake.isExtended()=false",
                            context.intake.isExtended()));
            assertTrue(
                    Math.abs(leftRot - IntakeConstants.RETRACTED_POSITION_ROT) <= RETRACT_TOLERANCE_LEFT_ROT,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "If intake is marked retracted, left physical position should match retract target",
                            IntakeConstants.RETRACTED_POSITION_ROT + "±" + RETRACT_TOLERANCE_LEFT_ROT + " rot",
                            String.format(Locale.US, "leftRot=%.3f", leftRot)));
            assertTrue(
                    Math.abs(rightRot - expectedRightRetractRot) <= RETRACT_TOLERANCE_RIGHT_ROT,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "If intake is marked retracted, right physical position should match retract target",
                            expectedRightRetractRot + "±" + RETRACT_TOLERANCE_RIGHT_ROT + " rot",
                            String.format(Locale.US, "rightRot=%.3f", rightRot)));

            assertTrue(
                    slowRetractDelta.finishes() >= 1,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "Slow retract should finish at least once during chaos and final hold",
                            "finishes>=1",
                            slowRetractDelta));
            assertTrue(
                    Math.abs(leftRot - IntakeConstants.RETRACTED_POSITION_ROT) <= STRICT_FINAL_RETRACT_LEFT_ROT
                            && Math.abs(rightRot - expectedRightRetractRot) <= STRICT_FINAL_RETRACT_RIGHT_ROT,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "After many slow retract triggers, final retract should complete physically (not just logical flag)",
                            "left/right within strict retract tolerance",
                            String.format(
                                    Locale.US,
                                    "slowRetractDelta=%s leftRot=%.3f rightRot=%.3f intakeExtended=%s",
                                    slowRetractDelta,
                                    leftRot,
                                    rightRot,
                                    context.intake.isExtended())));
        }
    }

    private static Map<String, FullFunctionalityHarness.CommandCounts> snapshotCounts(
            FullFunctionalityHarness.Context context,
            List<String> commandNames) {
        Map<String, FullFunctionalityHarness.CommandCounts> byName = new LinkedHashMap<>();
        for (String commandName : commandNames) {
            byName.put(commandName, context.recorder.getCounts(commandName));
        }
        return byName;
    }

    private static Map<String, FullFunctionalityHarness.CommandCounts> deltaCounts(
            FullFunctionalityHarness.Context context,
            Map<String, FullFunctionalityHarness.CommandCounts> before,
            List<String> commandNames) {
        Map<String, FullFunctionalityHarness.CommandCounts> byName = new LinkedHashMap<>();
        for (String commandName : commandNames) {
            byName.put(commandName, context.recorder.getCounts(commandName).minus(before.get(commandName)));
        }
        return byName;
    }

    private static void assertLifecycleClosed(String commandName, FullFunctionalityHarness.CommandCounts delta) {
        int ends = delta.finishes() + delta.interrupts();
        assertEquals(
                delta.starts(),
                ends,
                FullFunctionalityHarness.formatExpectedVsActual(
                        commandName + " lifecycle should close under chaos",
                        "starts==finishes+interrupts",
                        String.format(Locale.US, "%s ends=%d", delta, ends)));
    }

    private static int runTeleopChaosBurst(
            FullFunctionalityHarness.Context context,
            int phase,
            int cycles) {
        for (int i = 0; i < cycles; i++) {
            int globalCycle = phase * cycles + i;
            int pattern = globalCycle % 24;

            context.driverControllerSim.setBButton(pattern <= 1 || pattern == 13);
            context.driverControllerSim.setXButton(pattern >= 2 && pattern <= 6);
            context.driverControllerSim.setLeftTriggerAxis(pattern >= 4 && pattern <= 9 ? 1.0 : 0.0);
            context.driverControllerSim.setRightTriggerAxis(pattern >= 8 && pattern <= 14 ? 1.0 : 0.0);
            context.driverControllerSim.setRightBumperButton(pattern >= 15 && pattern <= 20);
            context.driverControllerSim.setYButton(pattern >= 10 && pattern <= 13);
            context.driverControllerSim.setBackButton(pattern == 3 || pattern == 11 || pattern == 19);
            context.driverControllerSim.setStartButton(pattern == 5 || pattern == 17 || pattern == 23);
            context.driverControllerSim.setPOV(switch (pattern % 4) {
                case 0 -> 0;
                case 1 -> 180;
                case 2 -> 270;
                default -> -1;
            });

            context.driverControllerSim.setLeftX((((pattern % 6) - 2.5) / 2.5) * 0.70);
            context.driverControllerSim.setLeftY(-((((pattern + 2) % 6) - 2.5) / 2.5) * 0.60);
            context.driverControllerSim.setRightX(((((pattern + 4) % 6) - 2.5) / 2.5) * 0.50);
            context.driverControllerSim.setRightY(0.0);

            applyGodControllerChaos(context, globalCycle);
            context.runCycles(1);
        }
        return cycles;
    }

    private static int holdInputsAcrossModeFlip(
            FullFunctionalityHarness.Context context,
            int phase) {
        context.driverControllerSim.setBButton((phase % 2) == 0);
        context.driverControllerSim.setXButton(true);
        context.driverControllerSim.setLeftTriggerAxis(1.0);
        context.driverControllerSim.setRightTriggerAxis(1.0);
        context.driverControllerSim.setRightBumperButton((phase % 3) != 0);
        context.driverControllerSim.setYButton(true);
        context.driverControllerSim.setBackButton((phase % 2) == 1);
        context.driverControllerSim.setStartButton((phase % 4) == 0);
        context.driverControllerSim.setPOV((phase % 3) == 0 ? 0 : (phase % 3) == 1 ? 180 : 270);

        if (context.godControllerSim != null) {
            context.godControllerSim.setAButton((phase % 4) == 0);
            context.godControllerSim.setBButton((phase % 4) == 1);
            context.godControllerSim.setXButton((phase % 4) == 2);
            context.godControllerSim.setYButton((phase % 4) == 3);
            context.godControllerSim.setLeftBumperButton((phase % 5) == 0);
            context.godControllerSim.setPOV((phase % 2) == 0 ? 180 : -1);
        }

        int holdCycles = 3;
        context.runCycles(holdCycles);
        return holdCycles;
    }

    private static void applyGodControllerChaos(
            FullFunctionalityHarness.Context context,
            int globalCycle) {
        if (context.godControllerSim == null) {
            return;
        }

        int godPattern = globalCycle % 160;
        context.godControllerSim.setAButton(godPattern >= 8 && godPattern < 20);
        context.godControllerSim.setBButton(godPattern >= 40 && godPattern < 52);
        context.godControllerSim.setXButton(godPattern >= 72 && godPattern < 84);
        context.godControllerSim.setYButton(godPattern >= 104 && godPattern < 116);
        context.godControllerSim.setLeftBumperButton(godPattern == 24 || godPattern == 120);
        context.godControllerSim.setPOV((godPattern == 60 || godPattern == 140) ? 180 : -1);
    }

    private static double leftPositionRot(FullFunctionalityHarness.Context context) {
        return Units.radiansToRotations(context.intakeInputs.leftPositionRad);
    }

    private static double rightPositionRot(FullFunctionalityHarness.Context context) {
        return Units.radiansToRotations(context.intakeInputs.rightPositionRad);
    }
}
