package frc.robot.sim;

import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assertions.assertFalse;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.intake.IntakeConstants;
import java.util.Locale;
import org.junit.jupiter.api.Test;

final class FullFunctionalitySmartRetractDuringShoot {

    @Test
    void nibbleModeRestoresExtendedIfShootReleasedEarly() {
        try (FullFunctionalityHarness.Context context = new FullFunctionalityHarness.Context(false)) {
            context.setTeleopEnabled();
            context.container.teleopInit();
            context.runCycles(50);

            SmartDashboard.putBoolean("Intake/SmartRetract/EnableNibble", true);
            SmartDashboard.putBoolean("Intake/SmartRetract/EnableHalfRetractReturn", false);
            SmartDashboard.putBoolean("Overrides/OverrideAutoAim", true);
            SmartDashboard.putNumber("Overrides/AimDistanceMeters", 3.0);
            SmartDashboard.putBoolean("Shooter/Tuning/Enabled", false);
            context.runCycles(8);

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
    void halfRetractReturnModeRestoresExtendedIfShootReleasedEarly() {
        try (FullFunctionalityHarness.Context context = new FullFunctionalityHarness.Context(false)) {
            context.setTeleopEnabled();
            context.container.teleopInit();
            context.runCycles(50);

            SmartDashboard.putBoolean("Intake/SmartRetract/EnableNibble", false);
            SmartDashboard.putBoolean("Intake/SmartRetract/EnableHalfRetractReturn", true);
            SmartDashboard.putBoolean("Overrides/OverrideAutoAim", true);
            SmartDashboard.putNumber("Overrides/AimDistanceMeters", 3.0);
            SmartDashboard.putBoolean("Shooter/Tuning/Enabled", false);
            context.runCycles(8);

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

            SmartDashboard.putBoolean("Intake/SmartRetract/EnableNibble", true);
            SmartDashboard.putBoolean("Intake/SmartRetract/EnableHalfRetractReturn", false);
            SmartDashboard.putBoolean("Overrides/OverrideAutoAim", true);
            SmartDashboard.putNumber("Overrides/AimDistanceMeters", 3.0);
            SmartDashboard.putBoolean("Shooter/Tuning/Enabled", false);
            context.runCycles(8);

            context.intake.setExtended(true);
            context.runCycles(30);

            context.driverControllerSim.setRightTriggerAxis(1.0);
            boolean reachedSmartRetractTarget = context.runUntil(
                    () -> Units.radiansToRotations(context.intakeInputs.leftPositionRad)
                            <= IntakeConstants.SMART_RETRACT_RETRACTED_POSITION_ROT + 1.20,
                    500);
            assertTrue(
                    !reachedSmartRetractTarget,
                    "Expected this scenario to keep nibble smart retract above the inward target.");
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
    void autonomousShooterShootHubNamedCommandRunsSmartRetractAndRestoresExtendedOnCancel() {
        try (FullFunctionalityHarness.Context context = new FullFunctionalityHarness.Context(false)) {
            context.setAutonomousEnabled();
            context.runCycles(40);

            SmartDashboard.putBoolean("Intake/SmartRetract/EnableNibble", true);
            SmartDashboard.putBoolean("Intake/SmartRetract/EnableHalfRetractReturn", false);
            context.runCycles(8);

            context.intake.setExtended(true);
            context.runCycles(180);
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
            boolean commandedInward = context.runUntil(
                    () -> commandedLeftTargetRot(context)
                            <= IntakeConstants.EXTENDED_POSITION_ROT - 0.25,
                    400);
            assertTrue(
                    commandedInward,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "Autonomous ShooterShootHub should command intake smart retract inward",
                            String.format(
                                    Locale.US,
                                    "commandedLeftTargetRot<=%.3f",
                                    IntakeConstants.EXTENDED_POSITION_ROT - 0.25),
                            String.format(
                                    Locale.US,
                                    "commandedLeftTargetRot=%.3f",
                                    commandedLeftTargetRot(context))));

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

            SmartDashboard.putBoolean("Intake/SmartRetract/EnableNibble", true);
            SmartDashboard.putBoolean("Intake/SmartRetract/EnableHalfRetractReturn", false);
            context.runCycles(8);

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

            SmartDashboard.putBoolean("Intake/SmartRetract/EnableNibble", true);
            SmartDashboard.putBoolean("Intake/SmartRetract/EnableHalfRetractReturn", false);
            context.runCycles(8);

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

    private static double commandedLeftTargetRot(FullFunctionalityHarness.Context context) {
        return FullFunctionalityHarness.getPrivateField(context.intake, "commandedLeftTargetRot", Double.class);
    }
}
