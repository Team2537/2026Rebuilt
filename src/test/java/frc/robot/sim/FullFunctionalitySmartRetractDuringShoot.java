package frc.robot.sim;

import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.util.Units;
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
            SmartDashboard.putBoolean("Intake/SmartRetract/EnableCurrentHold", false);
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
            context.runCycles(40);

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
    void currentHoldModeRestoresExtendedIfShootReleasedEarly() {
        try (FullFunctionalityHarness.Context context = new FullFunctionalityHarness.Context(false)) {
            context.setTeleopEnabled();
            context.container.teleopInit();
            context.runCycles(50);

            SmartDashboard.putBoolean("Intake/SmartRetract/EnableNibble", false);
            SmartDashboard.putBoolean("Intake/SmartRetract/EnableCurrentHold", true);
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
            context.runCycles(40);

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
    void doesNotReextendIfFullyRetractedBeforeShootRelease() {
        try (FullFunctionalityHarness.Context context = new FullFunctionalityHarness.Context(false)) {
            context.setTeleopEnabled();
            context.container.teleopInit();
            context.runCycles(50);

            SmartDashboard.putBoolean("Intake/SmartRetract/EnableNibble", false);
            SmartDashboard.putBoolean("Intake/SmartRetract/EnableCurrentHold", true);
            SmartDashboard.putBoolean("Overrides/OverrideAutoAim", true);
            SmartDashboard.putNumber("Overrides/AimDistanceMeters", 3.0);
            SmartDashboard.putBoolean("Shooter/Tuning/Enabled", false);
            context.runCycles(8);

            context.intake.setExtended(true);
            context.runCycles(30);

            context.driverControllerSim.setRightTriggerAxis(1.0);
            context.runCycles(320);
            context.driverControllerSim.setRightTriggerAxis(0.0);
            context.runCycles(40);

            double leftRot = Units.radiansToRotations(context.intakeInputs.leftPositionRad);
            assertTrue(
                    !context.intake.isExtended(),
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "If shoot is released after full retract, intake should stay retracted",
                            "intake.isExtended()=false",
                            context.intake.isExtended()));
            assertTrue(
                    Math.abs(leftRot - IntakeConstants.RETRACTED_POSITION_ROT) <= 0.80,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "After full retract, intake should stay near retracted setpoint",
                            IntakeConstants.RETRACTED_POSITION_ROT + "±0.80 rot",
                            String.format(Locale.US, "leftRot=%.3f", leftRot)));
        }
    }
}
