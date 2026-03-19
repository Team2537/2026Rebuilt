package frc.robot.sim;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.shooter.ShooterConstants;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.Set;
import java.util.function.Consumer;
import org.junit.jupiter.api.Test;

final class FullFunctionalityBindings {
    @Test
    void intakeEdgeCasesExtendedThenImmediateRetractAndToggleWhileRunning() {
        try (FullFunctionalityHarness.Context context = new FullFunctionalityHarness.Context(false)) {
            context.setTeleopEnabled();
            context.container.teleopInit();
            context.runCycles(20);

            Command extend = context.intake.extendCommand().withName("FF_IntakeExtendImmediate");
            Command retract = context.intake.retractCommand().withName("FF_IntakeRetractImmediate");
            FullFunctionalityHarness.CommandCounts extendBefore =
                    context.recorder.getCounts("FF_IntakeExtendImmediate");
            FullFunctionalityHarness.CommandCounts retractBefore =
                    context.recorder.getCounts("FF_IntakeRetractImmediate");

            CommandScheduler.getInstance().schedule(extend);
            context.runCycles(6);
            CommandScheduler.getInstance().schedule(retract);
            boolean retractFinished = context.runUntil(() -> !CommandScheduler.getInstance().isScheduled(retract), 300);
            assertTrue(retractFinished, "Immediate retract scenario should complete retract command.");

            FullFunctionalityHarness.CommandCounts extendDelta =
                    context.recorder.getCounts("FF_IntakeExtendImmediate").minus(extendBefore);
            FullFunctionalityHarness.CommandCounts retractDelta =
                    context.recorder.getCounts("FF_IntakeRetractImmediate").minus(retractBefore);
            assertTrue(
                    extendDelta.starts() >= 1,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "Immediate retract case should start extend", "starts>=1", extendDelta));
            assertTrue(
                    extendDelta.interrupts() >= 1,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "Immediate retract case should interrupt extend", "interrupts>=1", extendDelta));
            assertTrue(
                    retractDelta.starts() >= 1 && retractDelta.finishes() >= 1,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "Immediate retract case should finish retract",
                            "starts>=1 and finishes>=1",
                            retractDelta));
            assertTrue(!context.intake.isExtended(), "Immediate retract case should end retracted.");

            double leftImmediateRetract = Units.radiansToRotations(context.intakeInputs.leftPositionRad);
            double rightImmediateRetract = Units.radiansToRotations(context.intakeInputs.rightPositionRad);
            double expectedRightRetractRot =
                    IntakeConstants.RIGHT_OPPOSES_LEFT ? -IntakeConstants.RETRACTED_POSITION_ROT : IntakeConstants.RETRACTED_POSITION_ROT;
            assertTrue(
                    Math.abs(leftImmediateRetract - IntakeConstants.RETRACTED_POSITION_ROT) <= 0.7,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "Immediate retract should land left intake near retract target",
                            IntakeConstants.RETRACTED_POSITION_ROT + "±0.7 rot",
                            leftImmediateRetract));
            assertTrue(
                    Math.abs(rightImmediateRetract - expectedRightRetractRot) <= 0.7,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "Immediate retract should land right intake near retract target",
                            expectedRightRetractRot + "±0.7 rot",
                            rightImmediateRetract));

            Command rollerHold = context.intake.spinRoller().withName("FF_IntakeRollerHold");
            FullFunctionalityHarness.CommandCounts rollerBefore = context.recorder.getCounts("FF_IntakeRollerHold");
            CommandScheduler.getInstance().schedule(rollerHold);
            context.runCycles(12);
            assertTrue(
                    context.recorder.runningCount("FF_IntakeRollerHold") >= 1,
                    "Roller hold should be running before toggle conflict test.");

            Command toggleWhileRoller =
                    context.intake.toggleExtendedCommand().withName("FF_IntakeToggleWhileRoller");
            FullFunctionalityHarness.CommandCounts toggleWhileRollerBefore =
                    context.recorder.getCounts("FF_IntakeToggleWhileRoller");
            CommandScheduler.getInstance().schedule(toggleWhileRoller);
            boolean toggleWhileRollerFinished =
                    context.runUntil(() -> !CommandScheduler.getInstance().isScheduled(toggleWhileRoller), 300);
            assertTrue(toggleWhileRollerFinished, "Toggle while roller-running should complete.");

            FullFunctionalityHarness.CommandCounts rollerDelta =
                    context.recorder.getCounts("FF_IntakeRollerHold").minus(rollerBefore);
            FullFunctionalityHarness.CommandCounts toggleWhileRollerDelta =
                    context.recorder.getCounts("FF_IntakeToggleWhileRoller").minus(toggleWhileRollerBefore);
            assertTrue(
                    rollerDelta.interrupts() >= 1,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "Toggle while running should interrupt roller command",
                            "interrupts>=1",
                            rollerDelta));
            assertTrue(
                    toggleWhileRollerDelta.starts() >= 1 && toggleWhileRollerDelta.finishes() >= 1,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "Toggle while running should start and finish toggle",
                            "starts>=1 and finishes>=1",
                            toggleWhileRollerDelta));
            assertTrue(
                    Math.abs(context.intakeInputs.rollerAppliedVolts) < 1e-6,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "Toggle while roller-running should leave roller output stopped after interruption",
                            "rollerAppliedVolts=0",
                            String.format(Locale.US, "rollerAppliedVolts=%.3f", context.intakeInputs.rollerAppliedVolts)));

            context.intake.setExtended(false);
            context.runCycles(10);
            Command toggle1 = context.intake.toggleExtendedCommand().withName("FF_IntakeToggle1");
            Command toggle2 = context.intake.toggleExtendedCommand().withName("FF_IntakeToggle2");
            FullFunctionalityHarness.CommandCounts toggle1Before = context.recorder.getCounts("FF_IntakeToggle1");
            FullFunctionalityHarness.CommandCounts toggle2Before = context.recorder.getCounts("FF_IntakeToggle2");

            CommandScheduler.getInstance().schedule(toggle1);
            context.runCycles(3);
            CommandScheduler.getInstance().schedule(toggle2);
            boolean toggle2Finished = context.runUntil(() -> !CommandScheduler.getInstance().isScheduled(toggle2), 300);
            assertTrue(toggle2Finished, "Second toggle in rapid toggle scenario should finish.");

            FullFunctionalityHarness.CommandCounts toggle1Delta =
                    context.recorder.getCounts("FF_IntakeToggle1").minus(toggle1Before);
            FullFunctionalityHarness.CommandCounts toggle2Delta =
                    context.recorder.getCounts("FF_IntakeToggle2").minus(toggle2Before);
            assertTrue(
                    toggle1Delta.starts() >= 1,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "Rapid toggle case should start first toggle", "starts>=1", toggle1Delta));
            assertTrue(
                    toggle1Delta.finishes() >= 1,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "Rapid toggle case should finish first toggle", "finishes>=1", toggle1Delta));
            assertTrue(
                    toggle2Delta.starts() >= 1 && toggle2Delta.finishes() >= 1,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "Rapid toggle case should start and finish second toggle",
                            "starts>=1 and finishes>=1",
                            toggle2Delta));
            boolean rapidToggleSettledRetract = context.runUntil(
                    () -> Math.abs(Units.radiansToRotations(context.intakeInputs.leftPositionRad)
                            - IntakeConstants.RETRACTED_POSITION_ROT) <= 0.8
                            && Math.abs(Units.radiansToRotations(context.intakeInputs.rightPositionRad)
                            - expectedRightRetractRot) <= 0.8
                            && !context.intake.isExtended(),
                    320);
            assertTrue(
                    rapidToggleSettledRetract,
                    "Rapid toggle case should settle retracted after the second toggle command event.");

            double leftRapidToggle = Units.radiansToRotations(context.intakeInputs.leftPositionRad);
            double rightRapidToggle = Units.radiansToRotations(context.intakeInputs.rightPositionRad);
            assertTrue(
                    Math.abs(leftRapidToggle - IntakeConstants.RETRACTED_POSITION_ROT) <= 0.8,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "Rapid toggle case should finish with left intake near retract target",
                            IntakeConstants.RETRACTED_POSITION_ROT + "±0.8 rot",
                            leftRapidToggle));
            assertTrue(
                    Math.abs(rightRapidToggle - expectedRightRetractRot) <= 0.8,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "Rapid toggle case should finish with right intake near retract target",
                            expectedRightRetractRot + "±0.8 rot",
                            rightRapidToggle));

            context.intake.setExtended(false);
            context.runCycles(10);
            Command repeatedToggle =
                    context.intake.toggleExtendedCommand().withName("FF_IntakeToggleRepeatedBinding");
            FullFunctionalityHarness.CommandCounts repeatedToggleBefore =
                    context.recorder.getCounts("FF_IntakeToggleRepeatedBinding");
            double initialRepeatedLeftRot = Units.radiansToRotations(context.intakeInputs.leftPositionRad);

            CommandScheduler.getInstance().schedule(repeatedToggle);
            context.runCycles(8);
            double leftAfterFirstRepeatedToggle = Units.radiansToRotations(context.intakeInputs.leftPositionRad);
            assertTrue(
                    leftAfterFirstRepeatedToggle > initialRepeatedLeftRot + 0.35,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "First press of repeated-toggle binding should begin extension progress",
                            "leftAfterFirst > initial+0.35 rot",
                            String.format(
                                    Locale.US,
                                    "initial=%.3f leftAfterFirst=%.3f",
                                    initialRepeatedLeftRot,
                                    leftAfterFirstRepeatedToggle)));

            CommandScheduler.getInstance().schedule(repeatedToggle);
            boolean repeatedToggleReturnedToRetract = context.runUntil(
                    () -> Math.abs(Units.radiansToRotations(context.intakeInputs.leftPositionRad)
                            - IntakeConstants.RETRACTED_POSITION_ROT) <= 0.8
                            && !context.intake.isExtended(),
                    320);
            assertTrue(
                    repeatedToggleReturnedToRetract,
                    "Second press of repeated-toggle binding should interrupt/reverse motion and settle retracted.");
            double finalRepeatedLeftRot = Units.radiansToRotations(context.intakeInputs.leftPositionRad);
            assertTrue(
                    finalRepeatedLeftRot < leftAfterFirstRepeatedToggle - 0.25,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "Second repeated-toggle press should reverse extension direction, not ignore until completion",
                            "finalLeft < leftAfterFirst-0.25 rot",
                            String.format(
                                    Locale.US,
                                    "leftAfterFirst=%.3f finalLeft=%.3f",
                                    leftAfterFirstRepeatedToggle,
                                    finalRepeatedLeftRot)));
            FullFunctionalityHarness.CommandCounts repeatedToggleDelta =
                    context.recorder.getCounts("FF_IntakeToggleRepeatedBinding").minus(repeatedToggleBefore);
            assertEquals(
                    2,
                    repeatedToggleDelta.starts(),
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "Repeated-toggle binding should schedule exactly twice",
                            "starts=2",
                            repeatedToggleDelta));
            assertEquals(
                    2,
                    repeatedToggleDelta.finishes(),
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "Repeated-toggle binding should finish both presses",
                            "finishes=2",
                            repeatedToggleDelta));

            context.intake.setExtended(false);
            context.runCycles(10);
            Command extendRace = context.intake.extendCommand().withName("FF_IntakeExtendRace");
            Command toggleRace = context.intake.toggleExtendedCommand().withName("FF_IntakeToggleRace");
            FullFunctionalityHarness.CommandCounts extendRaceBefore = context.recorder.getCounts("FF_IntakeExtendRace");
            FullFunctionalityHarness.CommandCounts toggleRaceBefore = context.recorder.getCounts("FF_IntakeToggleRace");

            CommandScheduler.getInstance().schedule(extendRace);
            context.runCycles(2);
            CommandScheduler.getInstance().schedule(toggleRace);
            boolean toggleRaceFinished = context.runUntil(() -> !CommandScheduler.getInstance().isScheduled(toggleRace), 300);
            assertTrue(toggleRaceFinished, "Toggle-over-extend race should finish toggle command.");

            FullFunctionalityHarness.CommandCounts extendRaceDelta =
                    context.recorder.getCounts("FF_IntakeExtendRace").minus(extendRaceBefore);
            FullFunctionalityHarness.CommandCounts toggleRaceDelta =
                    context.recorder.getCounts("FF_IntakeToggleRace").minus(toggleRaceBefore);
            assertTrue(
                    extendRaceDelta.interrupts() >= 1,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "Toggle-over-extend race should interrupt extending command",
                            "interrupts>=1",
                            extendRaceDelta));
            assertTrue(
                    toggleRaceDelta.starts() >= 1 && toggleRaceDelta.finishes() >= 1,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "Toggle-over-extend race should run to completion",
                            "starts>=1 and finishes>=1",
                            toggleRaceDelta));
            assertTrue(!context.intake.isExtended(), "Toggle-over-extend race should end retracted.");
        }
    }

    @Test
    void slowModeToggleKeepsDefaultDriveActiveAndRobotMoving() {
        try (FullFunctionalityHarness.Context context = new FullFunctionalityHarness.Context(false)) {
            context.setTeleopEnabled();
            context.container.teleopInit();
            context.runCycles(20);

            Pose2d startingPose = context.drive.getPose();
            context.tapDriverButton(context.driverControllerSim::setLeftStickButton);
            context.driverControllerSim.setLeftY(0.55);
            context.runCycles(30);

            assertTrue(
                    context.drive.isConstraintProfileActive(frc.robot.subsystems.drive.DriveConstants.ConstraintProfile.SLOW_MODE),
                    "Expected left stick hold to enable slow mode while driving.");
            assertTrue(
                    context.recorder.runningCount("DriveJoystickDefault") >= 1,
                    "Expected DriveJoystickDefault to remain running while slow mode is held.");
            assertTrue(
                    context.drive.getPose().getTranslation().getDistance(startingPose.getTranslation()) > 0.05,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "Slow mode toggle should still allow translational motion",
                            "distanceMeters>0.05",
                            String.format(
                                    Locale.US,
                                    "distanceMeters=%.4f",
                                    context.drive.getPose().getTranslation().getDistance(startingPose.getTranslation()))));
        }
    }

    @Test
    void driverButtonsWorkAcrossMultipleSituations() {
        try (FullFunctionalityHarness.Context context = new FullFunctionalityHarness.Context(false)) {
            context.setTeleopEnabled();
            context.container.teleopInit();
            context.runCycles(20);

            assertTrue(
                    context.recorder.getCounts("DriveJoystickDefault").starts() >= 1,
                    "Expected DriveJoystickDefault to start as default command in teleop.");
            assertTrue(
                    context.recorder.getCounts("ShooterBackground").starts() >= 1
                            || context.recorder.getCounts("ShooterHome").starts() >= 1,
                    "Expected either ShooterBackground or ShooterHome to start in teleop.");
            assertTrue(
                    context.recorder.getCounts("IntakeBackground").starts() >= 1
                            || context.recorder.getCounts("IntakeHome").starts() >= 1,
                    "Expected either IntakeBackground or IntakeHome to start in teleop.");

            FullFunctionalityHarness.CommandCounts slowModeBefore = context.recorder.getCounts("DriveToggleSlowMode");
            context.tapDriverButton(context.driverControllerSim::setLeftStickButton);
            context.runCycles(10);
            assertTrue(
                    context.drive.isConstraintProfileActive(frc.robot.subsystems.drive.DriveConstants.ConstraintProfile.SLOW_MODE),
                    "Expected left stick press to toggle slow mode on.");
            context.tapDriverButton(context.driverControllerSim::setLeftStickButton);
            context.runCycles(10);
            assertTrue(
                    !context.drive.isConstraintProfileActive(frc.robot.subsystems.drive.DriveConstants.ConstraintProfile.SLOW_MODE),
                    "Expected a second left stick press to toggle slow mode off.");
            FullFunctionalityHarness.CommandCounts slowModeDelta =
                    context.recorder.getCounts("DriveToggleSlowMode").minus(slowModeBefore);
            assertEquals(
                    2,
                    slowModeDelta.starts(),
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "Left stick should schedule the slow-mode toggle twice",
                            "starts=2",
                            slowModeDelta));
            assertEquals(
                    2,
                    slowModeDelta.finishes(),
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "Slow-mode toggle should complete both presses as one-shot commands",
                            "finishes=2",
                            slowModeDelta));
            assertEquals(
                    0,
                    slowModeDelta.interrupts(),
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "Slow-mode toggle should not rely on interruption to turn off",
                            "interrupts=0",
                            slowModeDelta));

            FullFunctionalityHarness.CommandCounts fieldModeBefore =
                    context.recorder.getCounts("DriveToggleFieldOriented");
            context.tapDriverPov(90);
            assertTrue(!context.drive.isFieldOriented(), "Expected POV right to toggle field orientation off.");
            context.tapDriverPov(90);
            assertTrue(context.drive.isFieldOriented(), "Expected POV right to toggle field orientation back on.");
            FullFunctionalityHarness.CommandCounts fieldModeDelta =
                    context.recorder.getCounts("DriveToggleFieldOriented").minus(fieldModeBefore);
            assertEquals(
                    2,
                    fieldModeDelta.starts(),
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "POV right should toggle field-oriented mode twice", "starts=2", fieldModeDelta));

            context.drive.setPose(new Pose2d(context.drive.getPose().getTranslation(), Rotation2d.fromRadians(0.9)));
            FullFunctionalityHarness.CommandCounts resetBefore =
                    context.recorder.getCounts("DriveResetOdometryAndHeading");
            context.driverControllerSim.setLeftStickButton(true);
            context.runCycles(4);
            context.driverControllerSim.setRightStickButton(true);
            context.runCycles(4);
            assertTrue(
                    Math.abs(context.drive.getPose().getRotation().getRadians()) <= 1e-3,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "Left and right stick press combo should reset heading to zero radians",
                            "headingRad≈0",
                            String.format(Locale.US, "headingRad=%.6f", context.drive.getPose().getRotation().getRadians())));
            context.driverControllerSim.setRightStickButton(false);
            context.driverControllerSim.setLeftStickButton(false);
            context.runCycles(4);
            FullFunctionalityHarness.CommandCounts resetDelta =
                    context.recorder.getCounts("DriveResetOdometryAndHeading").minus(resetBefore);
            assertEquals(
                    1,
                    resetDelta.starts(),
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "Stick-press combo should schedule reset once", "starts=1", resetDelta));

            context.drive.setPose(new Pose2d(context.drive.getPose().getTranslation(), Rotation2d.fromRadians(0.75)));
            FullFunctionalityHarness.CommandCounts snapBefore = context.recorder.getCounts("DriveHeadingSnap");
            assertDriverHeadingSnap(
                    context,
                    context.driverControllerSim::setBButton,
                    -90.0,
                    "B should snap intake to face field right.");
            assertDriverHeadingSnap(
                    context,
                    context.driverControllerSim::setAButton,
                    180.0,
                    "A should snap intake to face field back.");
            assertDriverHeadingSnap(
                    context,
                    context.driverControllerSim::setXButton,
                    90.0,
                    "X should snap intake to face field left.");
            assertDriverHeadingSnap(
                    context,
                    context.driverControllerSim::setYButton,
                    0.0,
                    "Y should snap intake to face field forward.");
            FullFunctionalityHarness.CommandCounts snapDelta =
                    context.recorder.getCounts("DriveHeadingSnap").minus(snapBefore);
            assertTrue(
                    snapDelta.starts() >= 4,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "Face-button heading snap should schedule once per mapping", "starts>=4", snapDelta));
            assertTrue(
                    snapDelta.finishes() >= 4,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "Each face-button heading snap should finish in teleop", "finishes>=4", snapDelta));

            FullFunctionalityHarness.CommandCounts intakeTriggerBefore =
                    context.recorder.getCounts("DriverIntakeTriggerPress");
            FullFunctionalityHarness.CommandCounts rollerBefore = context.recorder.getCounts("IntakeSpinRoller");
            double leftBeforeTriggerRot = Units.radiansToRotations(context.intakeInputs.leftPositionRad);
            context.driverControllerSim.setLeftTriggerAxis(1.0);
            context.runCycles(40);
            assertTrue(
                    context.recorder.runningCount("IntakeSpinRoller") >= 1,
                    "Expected left trigger hold to keep IntakeSpinRoller running.");
            assertTrue(
                    Units.radiansToRotations(context.intakeInputs.leftPositionRad) > leftBeforeTriggerRot + 0.35,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "Left trigger press should drive the intake toward extension even if it has not fully settled yet",
                            "leftPositionRot > initial+0.35",
                            String.format(
                                    Locale.US,
                                    "initial=%.3f leftPositionRot=%.3f",
                                    leftBeforeTriggerRot,
                                    Units.radiansToRotations(context.intakeInputs.leftPositionRad))));
            assertTrue(
                    Math.abs(context.intakeInputs.rollerAppliedVolts) > 0.5,
                    "Expected left trigger hold to command nonzero roller voltage.");
            context.driverControllerSim.setLeftTriggerAxis(0.0);
            context.runCycles(10);
            FullFunctionalityHarness.CommandCounts rollerDelta =
                    context.recorder.getCounts("IntakeSpinRoller").minus(rollerBefore);
            assertTrue(
                    rollerDelta.starts() >= 1,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "Left trigger should schedule IntakeSpinRoller", "starts>=1", rollerDelta));
            assertTrue(
                    rollerDelta.interrupts() >= 1,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "Releasing left trigger should interrupt IntakeSpinRoller",
                            "interrupts>=1",
                            rollerDelta));
            FullFunctionalityHarness.CommandCounts intakeTriggerDelta =
                    context.recorder.getCounts("DriverIntakeTriggerPress").minus(intakeTriggerBefore);
            assertTrue(
                    intakeTriggerDelta.starts() >= 1,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "Left trigger press should schedule the intake deploy/double-press handler",
                            "starts>=1",
                            intakeTriggerDelta));

            context.driverControllerSim.setLeftTriggerAxis(1.0);
            context.runCycles(4);
            context.driverControllerSim.setLeftTriggerAxis(0.0);
            context.runCycles(4);
            context.driverControllerSim.setLeftTriggerAxis(1.0);
            context.runCycles(4);
            context.driverControllerSim.setLeftTriggerAxis(0.0);
            boolean leftTriggerDoubleTapRetracted = context.runUntil(() -> !context.intake.isExtended(), 260);
            assertTrue(leftTriggerDoubleTapRetracted, "Expected left-trigger double press to retract the intake.");

            FullFunctionalityHarness.CommandCounts scheduleBackgroundBefore =
                    context.recorder.getCounts("DriverIntakeHome");
            context.tapDriverButton(context.driverControllerSim::setStartButton);
            FullFunctionalityHarness.CommandCounts scheduleBackgroundDelta =
                    context.recorder.getCounts("DriverIntakeHome").minus(scheduleBackgroundBefore);
            assertEquals(
                    1,
                    scheduleBackgroundDelta.starts(),
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "Start should schedule DriverIntakeHome once",
                            "starts=1",
                            scheduleBackgroundDelta));

            SmartDashboard.putBoolean("Overrides/OverrideAutoAim", true);
            SmartDashboard.putNumber("Overrides/AimDistanceMeters", 3.0);
            SmartDashboard.putBoolean("Shooter/Tuning/Enabled", false);
            SmartDashboard.putBoolean("Shooter/Tuning/FeedKicker", false);
            context.runCycles(6);

            FullFunctionalityHarness.CommandCounts aimOnlyBefore =
                    context.recorder.getCounts("ShooterDriverAim");
            FullFunctionalityHarness.CommandCounts shootBefore =
                    context.recorder.getCounts("ShooterTriggerSelectedMode");
            context.driverControllerSim.setRightTriggerAxis(1.0);
            context.runCycles(80);
            assertTrue(
                    !context.shooter.isKickerActive(),
                    "Expected right trigger aim-only mode to block automatic feeding.");
            assertTrue(
                    Math.abs(context.transferInputs.appliedVolts) < 1e-6,
                    "Expected right trigger aim-only mode to keep transfer stopped.");
            context.driverControllerSim.setRightTriggerAxis(0.0);
            context.runCycles(10);
            FullFunctionalityHarness.CommandCounts aimOnlyDelta =
                    context.recorder.getCounts("ShooterDriverAim").minus(aimOnlyBefore);
            FullFunctionalityHarness.CommandCounts shootDelta =
                    context.recorder.getCounts("ShooterTriggerSelectedMode").minus(shootBefore);
            assertTrue(
                    aimOnlyDelta.starts() >= 1,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "Right trigger should schedule ShooterDriverAim when tuning disabled",
                            "starts>=1",
                            aimOnlyDelta));
            assertTrue(
                    aimOnlyDelta.interrupts() >= 1,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "Releasing right trigger should interrupt ShooterDriverAim",
                            "interrupts>=1",
                            aimOnlyDelta));
            assertEquals(
                    0,
                    shootDelta.starts(),
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "Right trigger aim-only mode should not start ShooterTriggerSelectedMode",
                            "starts=0",
                            shootDelta));
            assertTrue(
                    context.recorder.runningCount("ShooterBackground") >= 1,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "Shooter background should resume automatically after shoot trigger is released",
                            "ShooterBackground running>=1",
                            "ShooterBackground running=" + context.recorder.runningCount("ShooterBackground")));
            assertTrue(
                    Math.abs(context.shooter.getTargetAverageShooterRpm() - ShooterConstants.SLOW_SHOOTER_RPM) <= 1e-6,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "Shooter background should restore slow shooter target RPM after shoot release",
                            "targetAverageRpm=SLOW_SHOOTER_RPM",
                            String.format(
                                    Locale.US,
                                    "targetAverageRpm=%.3f slowRpm=%.3f",
                                    context.shooter.getTargetAverageShooterRpm(),
                                    ShooterConstants.SLOW_SHOOTER_RPM)));

            SmartDashboard.putBoolean("Overrides/DisableFeeding", true);
            context.runCycles(6);
            FullFunctionalityHarness.CommandCounts feedDisabledShootBefore =
                    context.recorder.getCounts("ShooterTriggerSelectedMode");
            context.driverControllerSim.setRightBumperButton(true);
            context.runCycles(140);
            assertTrue(
                    context.recorder.runningCount("ShooterTriggerSelectedMode") >= 1,
                    "Expected right bumper auto-feed mode to keep aiming/spinning while feed disable override is enabled.");
            assertTrue(
                    !context.shooter.isKickerActive(),
                    "Expected feed disable override to block automatic kicker feed.");
            assertTrue(
                    Math.abs(context.transferInputs.appliedVolts) < 1e-6,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "Feed disable override should keep transfer stopped until manual override is used",
                            "transferAppliedVolts=0",
                            String.format(Locale.US, "transferAppliedVolts=%.3f", context.transferInputs.appliedVolts)));
            context.driverControllerSim.setRightBumperButton(false);
            context.runCycles(10);
            FullFunctionalityHarness.CommandCounts feedDisabledShootDelta =
                    context.recorder.getCounts("ShooterTriggerSelectedMode").minus(feedDisabledShootBefore);
            assertTrue(
                    feedDisabledShootDelta.starts() >= 1,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "Feed disable override should still run the right-bumper auto-feed command wrapper",
                            "starts>=1",
                            feedDisabledShootDelta));
            SmartDashboard.putBoolean("Overrides/DisableFeeding", false);
            context.runCycles(4);

            SmartDashboard.putBoolean("Shooter/Tuning/Enabled", false);
            context.runCycles(4);
            FullFunctionalityHarness.CommandCounts autoFeedBefore =
                    context.recorder.getCounts("ShooterTriggerSelectedMode");
            context.driverControllerSim.setRightBumperButton(true);
            boolean shootSpunShooter =
                    context.runUntil(
                            () -> context.shooter.getTargetAverageShooterRpm() > ShooterConstants.SLOW_SHOOTER_RPM + 100.0,
                            240);
            assertTrue(
                    shootSpunShooter,
                    "Expected right bumper auto-feed mode to spin the shooter above its background target.");
            boolean bumperFed = context.runUntil(
                    () -> context.shooter.isKickerActive() && context.transferInputs.appliedVolts > 1.0,
                    260);
            assertTrue(bumperFed, "Right bumper auto-feed should eventually activate kicker and transfer.");
            assertTrue(
                    context.transferInputs.appliedVolts > 1.0,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "Right bumper auto-feed should drive the transfer",
                            "transferAppliedVolts>1.0",
                            String.format(Locale.US, "transferAppliedVolts=%.3f", context.transferInputs.appliedVolts)));
            context.driverControllerSim.setRightBumperButton(false);
            context.runCycles(10);
            FullFunctionalityHarness.CommandCounts autoFeedDelta =
                    context.recorder.getCounts("ShooterTriggerSelectedMode").minus(autoFeedBefore);
            assertTrue(
                    autoFeedDelta.starts() >= 1,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "Right bumper should schedule auto-feed shoot mode", "starts>=1", autoFeedDelta));
            assertTrue(
                    autoFeedDelta.interrupts() >= 1,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "Releasing right bumper should interrupt auto-feed shoot mode",
                            "interrupts>=1",
                            autoFeedDelta));

            FullFunctionalityHarness.CommandCounts aimBeforeCombined =
                    context.recorder.getCounts("ShooterDriverAim");
            FullFunctionalityHarness.CommandCounts shootBeforeCombined =
                    context.recorder.getCounts("ShooterTriggerSelectedMode");
            context.driverControllerSim.setRightBumperButton(true);
            context.driverControllerSim.setRightTriggerAxis(1.0);
            context.runCycles(30);
            context.driverControllerSim.setRightBumperButton(false);
            context.driverControllerSim.setRightTriggerAxis(0.0);
            context.runCycles(10);
            FullFunctionalityHarness.CommandCounts aimCombinedDelta =
                    context.recorder.getCounts("ShooterDriverAim").minus(aimBeforeCombined);
            FullFunctionalityHarness.CommandCounts shootCombinedDelta =
                    context.recorder.getCounts("ShooterTriggerSelectedMode").minus(shootBeforeCombined);
            assertEquals(
                    0,
                    aimCombinedDelta.starts(),
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "Right bumper + right trigger together should prioritize right bumper auto-feed over aim-only",
                            "starts=0",
                            aimCombinedDelta));
            assertTrue(
                    shootCombinedDelta.starts() >= 1,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "Right bumper + right trigger together should start ShooterTriggerSelectedMode",
                            "starts>=1",
                            shootCombinedDelta));

            FullFunctionalityHarness.CommandCounts aimTransitionBefore =
                    context.recorder.getCounts("ShooterDriverAim");
            FullFunctionalityHarness.CommandCounts shootTransitionBefore =
                    context.recorder.getCounts("ShooterTriggerSelectedMode");
            context.driverControllerSim.setRightTriggerAxis(1.0);
            boolean aimStarted = context.runUntil(() -> context.recorder.runningCount("ShooterDriverAim") >= 1, 120);
            assertTrue(aimStarted, "Right trigger should start aim-only before bumper is added.");
            context.driverControllerSim.setRightBumperButton(true);
            boolean shootTookOver = context.runUntil(
                    () -> context.recorder.runningCount("ShooterTriggerSelectedMode") >= 1
                            && context.recorder.runningCount("ShooterDriverAim") == 0,
                    120);
            assertTrue(shootTookOver, "Pressing right bumper while right trigger is held should switch into auto-feed mode.");
            context.driverControllerSim.setRightBumperButton(false);
            boolean aimResumed = context.runUntil(
                    () -> context.recorder.runningCount("ShooterDriverAim") >= 1
                            && context.recorder.runningCount("ShooterTriggerSelectedMode") == 0,
                    120);
            assertTrue(aimResumed, "Releasing right bumper while right trigger stays held should fall back to aim-only mode.");
            context.driverControllerSim.setRightTriggerAxis(0.0);
            context.runCycles(10);
            FullFunctionalityHarness.CommandCounts aimTransitionDelta =
                    context.recorder.getCounts("ShooterDriverAim").minus(aimTransitionBefore);
            FullFunctionalityHarness.CommandCounts shootTransitionDelta =
                    context.recorder.getCounts("ShooterTriggerSelectedMode").minus(shootTransitionBefore);
            assertTrue(aimTransitionDelta.starts() >= 2,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "Aim-only should start before bumper takeover and again after bumper release",
                            "starts>=2",
                            aimTransitionDelta));
            assertTrue(shootTransitionDelta.starts() >= 1,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "Auto-feed should start when bumper is added during an aim-only hold",
                            "starts>=1",
                            shootTransitionDelta));

            FullFunctionalityHarness.CommandCounts tuneBefore = context.recorder.getCounts("ShooterDashboardTune");
            FullFunctionalityHarness.CommandCounts tuneTransferBefore =
                    context.recorder.getCounts("TransferDashboardTune");
            FullFunctionalityHarness.CommandCounts shootDuringTuneBefore =
                    context.recorder.getCounts("ShooterTriggerSelectedMode");
            SmartDashboard.putBoolean("Shooter/Tuning/Enabled", true);
            SmartDashboard.putBoolean("Shooter/Tuning/FeedKicker", true);
            context.runCycles(8);
            context.driverControllerSim.setRightBumperButton(true);
            context.runCycles(60);
            assertTrue(
                    context.recorder.runningCount("ShooterDashboardTune") >= 1,
                    "Expected right bumper to run ShooterDashboardTune when dashboard tuning is enabled.");
            assertTrue(
                    context.recorder.runningCount("TransferDashboardTune") >= 1,
                    "Expected right bumper + dashboard feed enabled to run TransferDashboardTune.");
            context.driverControllerSim.setRightBumperButton(false);
            context.runCycles(10);
            FullFunctionalityHarness.CommandCounts tuneDelta =
                    context.recorder.getCounts("ShooterDashboardTune").minus(tuneBefore);
            FullFunctionalityHarness.CommandCounts tuneTransferDelta =
                    context.recorder.getCounts("TransferDashboardTune").minus(tuneTransferBefore);
            FullFunctionalityHarness.CommandCounts shootDuringTuneDelta =
                    context.recorder.getCounts("ShooterTriggerSelectedMode").minus(shootDuringTuneBefore);
            assertTrue(
                    tuneDelta.starts() >= 1,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "Dashboard tune mode should schedule ShooterDashboardTune", "starts>=1", tuneDelta));
            assertTrue(
                    tuneTransferDelta.starts() >= 1,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "Dashboard tune mode with feed enabled should schedule TransferDashboardTune",
                            "starts>=1",
                            tuneTransferDelta));
            assertEquals(
                    0,
                    shootDuringTuneDelta.starts(),
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "ShooterTriggerSelectedMode should not start while dashboard tuning is enabled",
                            "starts=0",
                            shootDuringTuneDelta));

            context.drive.setPose(new Pose2d(
                    context.drive.getPose().getTranslation().plus(new edu.wpi.first.math.geometry.Translation2d(1.0, 0.0)),
                    context.drive.getPose().getRotation()));
            context.runCycles(40);
            FullFunctionalityHarness.CommandCounts povLeftBefore =
                    context.recorder.getCounts("DriveSetOdometryFromUnifiedVision");
            context.tapDriverPov(270);
            FullFunctionalityHarness.CommandCounts povLeftDelta =
                    context.recorder.getCounts("DriveSetOdometryFromUnifiedVision").minus(povLeftBefore);
            assertTrue(
                    povLeftDelta.starts() >= 1,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "POV left should schedule DriveSetOdometryFromUnifiedVision",
                            "starts>=1",
                            povLeftDelta));

            SmartDashboard.putBoolean("Shooter/Tuning/Enabled", false);
            context.runCycles(6);
            context.intake.setExtended(true);
            context.runCycles(8);
            context.driverControllerSim.setRightTriggerAxis(1.0);
            context.runCycles(80);
            context.driverControllerSim.setRightTriggerAxis(0.0);
            context.runCycles(10);
            FullFunctionalityHarness.CommandCounts stopBefore = context.recorder.getCounts("StopManipulators");
            context.tapDriverPov(180);
            context.runCycles(20);
            FullFunctionalityHarness.CommandCounts stopDelta =
                    context.recorder.getCounts("StopManipulators").minus(stopBefore);
            assertTrue(
                    stopDelta.starts() >= 1,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "POV down should schedule StopManipulators", "starts>=1", stopDelta));
            assertTrue(!context.intake.isExtended(), "Expected StopManipulators to retract intake.");
            assertTrue(
                    Math.abs(context.shooter.getTargetAverageShooterRpm()) <= 1e-6,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "Expected StopManipulators to leave shooter targets cleared",
                            "targetAverageRpm=0",
                            String.format(
                                    Locale.US,
                                    "targetAverageRpm=%.3f",
                                    context.shooter.getTargetAverageShooterRpm())));
            assertTrue(
                    Math.abs(context.transferInputs.appliedVolts) < 1e-6,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "Expected StopManipulators to stop transfer",
                            "transferAppliedVolts=0",
                            String.format(Locale.US, "transferAppliedVolts=%.3f", context.transferInputs.appliedVolts)));

            Set<String> requiredDriverCommandNames = Set.of(
                    "DriveToggleSlowMode",
                    "DriveToggleFieldOriented",
                    "DriveResetOdometryAndHeading",
                    "DriveHeadingSnap",
                    "DriverIntakeTriggerPress",
                    "IntakeSpinRoller",
                    "DriverIntakeHome",
                    "StopManipulators",
                    "DriveSetOdometryFromUnifiedVision",
                    "ShooterTriggerSelectedMode",
                    "ShooterDriverAim",
                    "ShooterDashboardTune",
                    "TransferDashboardTune");
            List<String> missing = new ArrayList<>();
            for (String name : requiredDriverCommandNames) {
                if (context.recorder.getCounts(name).starts() == 0) {
                    missing.add(name);
                }
            }
            assertTrue(
                    missing.isEmpty(),
                    "Expected all driver binding commands to start at least once. Missing=" + missing);
        }
    }

    @Test
    void godControllerButtonsWorkWhenPresent() {
        try (FullFunctionalityHarness.Context context = new FullFunctionalityHarness.Context(true)) {
            context.setTeleopEnabled();
            context.container.teleopInit();
            context.runCycles(10);
            assertNotNull(context.godControllerSim, "Expected god controller sim to exist when port 5 is connected.");

            FullFunctionalityHarness.CommandCounts slowModeBefore = context.recorder.getCounts("DriveToggleSlowMode");
            context.tapGodButton(context.godControllerSim::setLeftBumperButton);
            FullFunctionalityHarness.CommandCounts slowModeDelta =
                    context.recorder.getCounts("DriveToggleSlowMode").minus(slowModeBefore);
            assertTrue(
                    slowModeDelta.starts() >= 1,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "God left bumper should schedule DriveToggleSlowMode", "starts>=1", slowModeDelta));

            context.drive.setPose(new Pose2d(context.drive.getPose().getTranslation(), Rotation2d.fromRadians(1.1)));
            FullFunctionalityHarness.CommandCounts resetBefore =
                    context.recorder.getCounts("DriveResetOdometryAndHeading");
            context.tapGodPov(180);
            FullFunctionalityHarness.CommandCounts resetDelta =
                    context.recorder.getCounts("DriveResetOdometryAndHeading").minus(resetBefore);
            assertTrue(
                    resetDelta.starts() >= 1,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "God POV down should schedule DriveResetOdometryAndHeading",
                            "starts>=1",
                            resetDelta));
            assertTrue(
                    Math.abs(context.drive.getPose().getRotation().getRadians()) <= 1e-3,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "God POV down should reset heading to zero",
                            "headingRad≈0",
                            String.format(Locale.US, "headingRad=%.6f", context.drive.getPose().getRotation().getRadians())));

            assertHeldCommandInterruptsOnRelease(
                    context,
                    "DriveSysIdQuasistaticForward",
                    () -> context.godControllerSim.setAButton(true),
                    () -> context.godControllerSim.setAButton(false));
            assertHeldCommandInterruptsOnRelease(
                    context,
                    "DriveSysIdQuasistaticReverse",
                    () -> context.godControllerSim.setBButton(true),
                    () -> context.godControllerSim.setBButton(false));
            assertHeldCommandInterruptsOnRelease(
                    context,
                    "DriveSysIdDynamicForward",
                    () -> context.godControllerSim.setXButton(true),
                    () -> context.godControllerSim.setXButton(false));
            assertHeldCommandInterruptsOnRelease(
                    context,
                    "DriveSysIdDynamicReverse",
                    () -> context.godControllerSim.setYButton(true),
                    () -> context.godControllerSim.setYButton(false));
        }
    }

    private static void assertHeldCommandInterruptsOnRelease(
            FullFunctionalityHarness.Context context,
            String commandName,
            Runnable press,
            Runnable release) {
        FullFunctionalityHarness.CommandCounts before = context.recorder.getCounts(commandName);
        press.run();
        context.runCycles(60);
        release.run();
        context.runCycles(10);
        FullFunctionalityHarness.CommandCounts delta = context.recorder.getCounts(commandName).minus(before);
        assertTrue(
                delta.starts() >= 1,
                FullFunctionalityHarness.formatExpectedVsActual(
                        "Expected held command to start while button is pressed",
                        commandName + " starts>=1",
                        delta));
        assertTrue(
                delta.interrupts() >= 1,
                    FullFunctionalityHarness.formatExpectedVsActual(
                        "Expected held command to interrupt on button release",
                        commandName + " interrupts>=1",
                        delta));
    }

    private static void assertDriverHeadingSnap(
            FullFunctionalityHarness.Context context,
            Consumer<Boolean> buttonSetter,
            double expectedHeadingDeg,
            String message) {
        FullFunctionalityHarness.CommandCounts before = context.recorder.getCounts("DriveHeadingSnap");
        context.tapDriverButton(buttonSetter);
        boolean snapFinished = context.runUntil(
                () -> context.recorder.getCounts("DriveHeadingSnap").minus(before).finishes() >= 1,
                260);
        assertTrue(snapFinished, message + " Command did not finish.");
        double headingErrorRad = MathUtil.angleModulus(
                Rotation2d.fromDegrees(expectedHeadingDeg)
                        .minus(context.drive.getPose().getRotation())
                        .getRadians());
        assertTrue(
                Math.abs(headingErrorRad) <= Math.toRadians(3.0),
                FullFunctionalityHarness.formatExpectedVsActual(
                        message,
                        "headingErrorDeg<=3.0",
                        String.format(Locale.US, "headingErrorDeg=%.3f", Math.toDegrees(headingErrorRad))));
    }
}
