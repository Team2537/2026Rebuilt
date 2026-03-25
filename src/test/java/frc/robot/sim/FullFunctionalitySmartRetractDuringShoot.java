package frc.robot.sim;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.util.FieldConstants;
import java.util.Locale;
import org.junit.jupiter.api.Test;

final class FullFunctionalitySmartRetractDuringShoot {

    @Test
    void smartRetractRestoresExtendedIfShootReleasedEarly() {
        try (FullFunctionalityHarness.Context context = new FullFunctionalityHarness.Context(false)) {
            context.setTeleopEnabled();
            context.container.teleopInit();
            context.runCycles(50);

            SmartDashboard.putBoolean("Overrides/OverrideAutoAim", true);
            SmartDashboard.putNumber("Overrides/AimDistanceMeters", 3.0);
            SmartDashboard.putBoolean("Shooter/Tuning/Enabled", false);
            settleSmartRetractConfig(context);

            context.intake.setExtended(true);
            context.runCycles(180);
            double extendedStartRot = Units.radiansToRotations(context.intakeInputs.leftPositionRad);
            assertTrue(
                    Math.abs(extendedStartRot - IntakeConstants.EXTENDED_POSITION_ROT) <= 1.20,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "Intake should be physically near extended before smart retract test",
                            IntakeConstants.EXTENDED_POSITION_ROT + "±1.20 rot",
                            String.format(Locale.US, "extendedStartRot=%.3f", extendedStartRot)));

            context.driverControllerSim.setRightTriggerAxis(1.0);
            context.runCycles(100);

            context.driverControllerSim.setRightTriggerAxis(0.0);
            context.runCycles(100);
            double afterReleaseRot = Units.radiansToRotations(context.intakeInputs.leftPositionRad);
            assertTrue(
                    context.intake.isExtended(),
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "Releasing shoot early should return intake to extended",
                            "intake.isExtended()=true",
                            context.intake.isExtended()));
            assertTrue(
                    Math.abs(afterReleaseRot - IntakeConstants.EXTENDED_POSITION_ROT) <= 1.20,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "After early release, intake should settle near extended position",
                            IntakeConstants.EXTENDED_POSITION_ROT + "±1.20 rot",
                            String.format(Locale.US, "afterReleaseRot=%.3f", afterReleaseRot)));
        }
    }

    @Test
    void longShootWithoutReachingSmartRetractTargetRestoresExtendedOnRelease() {
        try (FullFunctionalityHarness.Context context = new FullFunctionalityHarness.Context(false)) {
            context.setTeleopEnabled();
            context.container.teleopInit();
            context.runCycles(50);

            SmartDashboard.putBoolean("Overrides/OverrideAutoAim", true);
            SmartDashboard.putNumber("Overrides/AimDistanceMeters", 3.0);
            SmartDashboard.putBoolean("Shooter/Tuning/Enabled", false);
            settleSmartRetractConfig(context);

            context.intake.setExtended(true);
            context.runCycles(30);

            context.driverControllerSim.setRightTriggerAxis(1.0);
            boolean reachedSmartRetractTarget = context.runUntil(
                    () -> Units.radiansToRotations(context.intakeInputs.leftPositionRad)
                            <= IntakeConstants.SMART_RETRACT_RETRACTED_POSITION_ROT + 1.20,
                    500);
            assertTrue(
                    !reachedSmartRetractTarget,
                    "Expected this scenario to keep smart retract above the inward target.");
            context.driverControllerSim.setRightTriggerAxis(0.0);
            context.runCycles(100);

            double afterReleaseRot = Units.radiansToRotations(context.intakeInputs.leftPositionRad);
            assertTrue(
                    context.intake.isExtended(),
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "If smart retract never fully reaches its inward target, releasing shoot should restore extended",
                            "intake.isExtended()=true",
                            context.intake.isExtended()));
            assertTrue(
                    Math.abs(afterReleaseRot - IntakeConstants.EXTENDED_POSITION_ROT) <= 1.20,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "After release without reaching smart retract target, intake should settle back near extended position",
                            IntakeConstants.EXTENDED_POSITION_ROT + "±1.20 rot",
                            String.format(Locale.US, "afterReleaseRot=%.3f", afterReleaseRot)));
        }
    }

    @Test
    void autonomousShooterShootHubNamedCommandLeavesIntakeExtendedOnCancel() {
        try (FullFunctionalityHarness.Context context = new FullFunctionalityHarness.Context(false)) {
            context.setAutonomousEnabled();
            context.runCycles(40);
            settleSmartRetractConfig(context);

            context.intake.setExtended(true);
            context.runCycles(180);
            context.drive.setPose(new Pose2d(
                    FieldConstants.getAllianceZoneBoundaryX() - 0.2,
                    FieldConstants.getHubTargetTranslation().getY(),
                    Rotation2d.kPi));
            context.runCycles(20);
            double extendedStartRot = Units.radiansToRotations(context.intakeInputs.leftPositionRad);
            assertTrue(
                    Math.abs(extendedStartRot - IntakeConstants.EXTENDED_POSITION_ROT) <= 1.20,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "Intake should be physically near extended before autonomous smart retract test",
                            IntakeConstants.EXTENDED_POSITION_ROT + "±1.20 rot",
                            String.format(Locale.US, "extendedStartRot=%.3f", extendedStartRot)));

            Command shootHub = FullFunctionalityHarness.namedCommand("ShooterShootHub")
                    .withName("FF_AutoShooterShootHubSmartRetract");
            CommandScheduler.getInstance().schedule(shootHub);
            context.runCycles(120);

            CommandScheduler.getInstance().cancel(shootHub);
            boolean restoredExtended = context.runUntil(
                    () -> context.intake.isExtended()
                            && Math.abs(Units.radiansToRotations(context.intakeInputs.leftPositionRad)
                                            - IntakeConstants.EXTENDED_POSITION_ROT)
                                    <= 1.20,
                    220);
            assertTrue(
                    restoredExtended,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "Canceling autonomous ShooterShootHub should restore the intake to extended after smart retract",
                            "intake restored near extended",
                            String.format(
                                    Locale.US,
                                    "isExtended=%s leftRot=%.3f",
                                    context.intake.isExtended(),
                                    Units.radiansToRotations(context.intakeInputs.leftPositionRad))));
        }
    }

    @Test
    void autonomousShooterShootHubNoSmartRetractLeavesIntakeExtended() {
        try (FullFunctionalityHarness.Context context = new FullFunctionalityHarness.Context(false)) {
            context.setAutonomousEnabled();
            context.runCycles(40);
            settleSmartRetractConfig(context);

            context.intake.setExtended(true);
            context.runCycles(180);
            double extendedStartRot = Units.radiansToRotations(context.intakeInputs.leftPositionRad);
            assertTrue(
                    Math.abs(extendedStartRot - IntakeConstants.EXTENDED_POSITION_ROT) <= 1.20,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "Intake should be physically near extended before no-smart-retract autonomous shot test",
                            IntakeConstants.EXTENDED_POSITION_ROT + "±1.20 rot",
                            String.format(Locale.US, "extendedStartRot=%.3f", extendedStartRot)));

            Command shootHubNoSmartRetract = FullFunctionalityHarness.namedCommand("ShooterShootHubNoSmartRetract")
                    .withName("FF_AutoShooterShootHubNoSmartRetract");
            CommandScheduler.getInstance().schedule(shootHubNoSmartRetract);
            context.runCycles(160);

            double minAllowedTargetRot = IntakeConstants.EXTENDED_POSITION_ROT - 0.10;
            double commandedTargetRot = commandedLeftTargetRot(context);
            assertFalse(
                    commandedTargetRot < minAllowedTargetRot,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "Autonomous ShooterShootHubNoSmartRetract should not command intake inward",
                            String.format(Locale.US, "commandedLeftTargetRot>=%.3f", minAllowedTargetRot),
                            String.format(Locale.US, "commandedLeftTargetRot=%.3f", commandedTargetRot)));
            assertTrue(
                    context.intake.isExtended(),
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "Autonomous ShooterShootHubNoSmartRetract should leave intake marked extended",
                            "intake.isExtended()=true",
                            context.intake.isExtended()));

            CommandScheduler.getInstance().cancel(shootHubNoSmartRetract);
            context.runCycles(20);
        }
    }

    @Test
    void autonomousShooterShootHubOnMoveNoSmartRetractLeavesIntakeExtended() {
        try (FullFunctionalityHarness.Context context = new FullFunctionalityHarness.Context(false)) {
            context.setAutonomousEnabled();
            context.runCycles(40);
            settleSmartRetractConfig(context);

            context.intake.setExtended(true);
            context.runCycles(180);
            double extendedStartRot = Units.radiansToRotations(context.intakeInputs.leftPositionRad);
            assertTrue(
                    Math.abs(extendedStartRot - IntakeConstants.EXTENDED_POSITION_ROT) <= 1.20,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "Intake should be physically near extended before no-smart-retract on-move autonomous shot test",
                            IntakeConstants.EXTENDED_POSITION_ROT + "±1.20 rot",
                            String.format(Locale.US, "extendedStartRot=%.3f", extendedStartRot)));

            Command shootHubOnMoveNoSmartRetract =
                    FullFunctionalityHarness.namedCommand("ShooterShootHubOnMoveNoSmartRetract")
                            .withName("FF_AutoShooterShootHubOnMoveNoSmartRetract");
            CommandScheduler.getInstance().schedule(shootHubOnMoveNoSmartRetract);
            context.runCycles(160);

            double minAllowedTargetRot = IntakeConstants.EXTENDED_POSITION_ROT - 0.10;
            double commandedTargetRot = commandedLeftTargetRot(context);
            assertFalse(
                    commandedTargetRot < minAllowedTargetRot,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "Autonomous ShooterShootHubOnMoveNoSmartRetract should not command intake inward",
                            String.format(Locale.US, "commandedLeftTargetRot>=%.3f", minAllowedTargetRot),
                            String.format(Locale.US, "commandedLeftTargetRot=%.3f", commandedTargetRot)));
            assertTrue(
                    context.intake.isExtended(),
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "Autonomous ShooterShootHubOnMoveNoSmartRetract should leave intake marked extended",
                            "intake.isExtended()=true",
                            context.intake.isExtended()));

            CommandScheduler.getInstance().cancel(shootHubOnMoveNoSmartRetract);
            context.runCycles(20);
        }
    }

    @Test
    void directSmartRetractSessionWigglesAfterFullRetraction() {
        try (FullFunctionalityHarness.Context context = new FullFunctionalityHarness.Context(false)) {
            context.setTeleopEnabled();
            context.container.teleopInit();
            context.runCycles(50);

            settleSmartRetractConfig(context);
            extendAndSettle(context);

            boolean[] feeding = {true};
            Command smartRetract = context.intake.smartRetractDuringShootCommand(() -> feeding[0], () -> true)
                    .withName("FF_DirectSmartRetractWiggle");
            CommandScheduler.getInstance().schedule(smartRetract);

            double retractBaselineRot = expectedSmartRetractBaselineRot();
            double retractPeakRot = expectedSmartRetractPeakRot();
            boolean reachedFullRetract = context.runUntil(
                    () -> leftPositionRot(context) <= retractBaselineRot + 0.30,
                    400);
            assertTrue(
                    reachedFullRetract,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "Smart retract should physically reach the tuned full-retract target before wiggle validation",
                            String.format(Locale.US, "leftRot<=%.3f", retractBaselineRot + 0.30),
                            String.format(Locale.US, "leftRot=%.3f", leftPositionRot(context))));

            double minTargetRot = Double.POSITIVE_INFINITY;
            double maxTargetRot = Double.NEGATIVE_INFINITY;
            for (int i = 0; i < 40; i++) {
                context.runCycles(1);
                double commandedTargetRot = commandedLeftTargetRot(context);
                minTargetRot = Math.min(minTargetRot, commandedTargetRot);
                maxTargetRot = Math.max(maxTargetRot, commandedTargetRot);
            }

            assertTrue(
                    minTargetRot <= retractBaselineRot + 0.15,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "Smart retract wiggle should return to the retract baseline after full retraction",
                            String.format(Locale.US, "minTargetRot<=%.3f", retractBaselineRot + 0.15),
                            String.format(Locale.US, "minTargetRot=%.3f", minTargetRot)));
            assertTrue(
                    maxTargetRot >= retractPeakRot - 0.15,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "Smart retract wiggle should reach its configured outward peak after full retraction",
                            String.format(Locale.US, "maxTargetRot>=%.3f", retractPeakRot - 0.15),
                            String.format(Locale.US, "maxTargetRot=%.3f", maxTargetRot)));

            CommandScheduler.getInstance().cancel(smartRetract);
            context.runCycles(10);
        }
    }

    @Test
    void cancelingAfterRetractWigglePeakDoesNotRestoreExtended() {
        try (FullFunctionalityHarness.Context context = new FullFunctionalityHarness.Context(false)) {
            context.setTeleopEnabled();
            context.container.teleopInit();
            context.runCycles(50);

            settleSmartRetractConfig(context);
            extendAndSettle(context);

            boolean[] feeding = {true};
            Command smartRetract = context.intake.smartRetractDuringShootCommand(() -> feeding[0], () -> true)
                    .withName("FF_DirectSmartRetractCancelAfterWiggle");
            CommandScheduler.getInstance().schedule(smartRetract);

            double retractBaselineRot = expectedSmartRetractBaselineRot();
            double retractPeakRot = expectedSmartRetractPeakRot();
            boolean reachedFullRetract = context.runUntil(
                    () -> leftPositionRot(context) <= retractBaselineRot + 0.30,
                    400);
            assertTrue(reachedFullRetract, "Expected smart retract to reach its inward target before cancellation edge-case validation.");

            boolean reachedWigglePeak = context.runUntil(
                    () -> commandedLeftTargetRot(context) >= retractPeakRot - 0.15,
                    120);
            assertTrue(
                    reachedWigglePeak,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "Smart retract should enter the post-retract wiggle before cancellation",
                            String.format(Locale.US, "commandedTargetRot>=%.3f", retractPeakRot - 0.15),
                            String.format(Locale.US, "commandedTargetRot=%.3f", commandedLeftTargetRot(context))));

            CommandScheduler.getInstance().cancel(smartRetract);
            context.runCycles(140);

            assertFalse(
                    context.intake.isExtended(),
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "Ending smart retract after full retract wiggle must not restore extension",
                            "intake.isExtended()=false",
                            context.intake.isExtended()));
            assertTrue(
                    leftPositionRot(context) <= retractBaselineRot + 0.55,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "After cancellation, the intake should settle back near the smart retract baseline instead of extending",
                            String.format(Locale.US, "leftRot<=%.3f", retractBaselineRot + 0.55),
                            String.format(Locale.US, "leftRot=%.3f", leftPositionRot(context))));
        }
    }

    @Test
    void pausingFeedDoesNotCollapseRetractWiggle() {
        try (FullFunctionalityHarness.Context context = new FullFunctionalityHarness.Context(false)) {
            context.setTeleopEnabled();
            context.container.teleopInit();
            context.runCycles(50);

            settleSmartRetractConfig(context);
            extendAndSettle(context);

            boolean[] feeding = {true};
            Command smartRetract = context.intake.smartRetractDuringShootCommand(() -> feeding[0], () -> true)
                    .withName("FF_DirectSmartRetractPauseResumeWiggle");
            CommandScheduler.getInstance().schedule(smartRetract);

            double retractBaselineRot = expectedSmartRetractBaselineRot();
            double retractPeakRot = expectedSmartRetractPeakRot();
            boolean reachedFullRetract = context.runUntil(
                    () -> leftPositionRot(context) <= retractBaselineRot + 0.30,
                    400);
            assertTrue(reachedFullRetract, "Expected smart retract to reach full retract before pause/resume validation.");

            boolean reachedInitialWigglePeak = context.runUntil(
                    () -> commandedLeftTargetRot(context) >= retractPeakRot - 0.15,
                    120);
            assertTrue(reachedInitialWigglePeak, "Expected smart retract to enter wiggle before feed pause.");

            feeding[0] = false;
            double pausedMinTargetRot = Double.POSITIVE_INFINITY;
            double pausedMaxTargetRot = Double.NEGATIVE_INFINITY;
            for (int i = 0; i < 20; i++) {
                context.runCycles(1);
                double commandedTargetRot = commandedLeftTargetRot(context);
                pausedMinTargetRot = Math.min(pausedMinTargetRot, commandedTargetRot);
                pausedMaxTargetRot = Math.max(pausedMaxTargetRot, commandedTargetRot);
            }

            assertTrue(
                    pausedMaxTargetRot - pausedMinTargetRot >= IntakeConstants.smartRetractWiggleOutRot() * 0.60,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "When feed drops, smart retract should keep working the fully retracted fuel stack instead of freezing",
                            String.format(Locale.US, "pausedRange>=%.3f", IntakeConstants.smartRetractWiggleOutRot() * 0.60),
                            String.format(
                                    Locale.US,
                                    "pausedRange=%.3f (min=%.3f max=%.3f)",
                                    pausedMaxTargetRot - pausedMinTargetRot,
                                    pausedMinTargetRot,
                                    pausedMaxTargetRot)));

            feeding[0] = true;
            boolean reachedResumedWigglePeak = context.runUntil(
                    () -> commandedLeftTargetRot(context) >= retractPeakRot - 0.15,
                    120);
            assertTrue(
                    reachedResumedWigglePeak,
                    "Expected smart retract wiggle to still be available after feed resumes.");

            CommandScheduler.getInstance().cancel(smartRetract);
            context.runCycles(10);
        }
    }

    private static void settleSmartRetractConfig(FullFunctionalityHarness.Context context) {
        context.runCycles(8);
    }

    private static double expectedSmartRetractBaselineRot() {
        return IntakeConstants.smartRetractRetractedPositionRot();
    }

    private static double expectedSmartRetractPeakRot() {
        return IntakeConstants.smartRetractRetractedPositionRot() + IntakeConstants.smartRetractWiggleOutRot();
    }

    private static void extendAndSettle(FullFunctionalityHarness.Context context) {
        context.intake.setExtended(true);
        boolean settled = context.runUntil(
                () -> Math.abs(leftPositionRot(context) - IntakeConstants.EXTENDED_POSITION_ROT) <= 1.10,
                220);
        assertTrue(settled, "Expected intake to settle near extended before direct smart retract testing.");
        context.runCycles(8);
    }

    private static double leftPositionRot(FullFunctionalityHarness.Context context) {
        return Units.radiansToRotations(context.intakeInputs.leftPositionRad);
    }

    private static double commandedLeftTargetRot(FullFunctionalityHarness.Context context) {
        return FullFunctionalityHarness.getPrivateField(context.intake, "commandedLeftTargetRot", Double.class);
    }
}
