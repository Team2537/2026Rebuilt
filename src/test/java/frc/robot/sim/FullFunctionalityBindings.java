package frc.robot.sim;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

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
            context.tapDriverButton(context.driverControllerSim::setLeftBumperButton);
            assertTrue(
                    context.drive.isConstraintProfileActive(frc.robot.subsystems.drive.DriveConstants.ConstraintProfile.SLOW_MODE),
                    "Expected left bumper to enable slow mode on first press.");
            context.tapDriverButton(context.driverControllerSim::setLeftBumperButton);
            assertTrue(
                    !context.drive.isConstraintProfileActive(frc.robot.subsystems.drive.DriveConstants.ConstraintProfile.SLOW_MODE),
                    "Expected left bumper to disable slow mode on second press.");
            FullFunctionalityHarness.CommandCounts slowModeDelta =
                    context.recorder.getCounts("DriveToggleSlowMode").minus(slowModeBefore);
            assertEquals(
                    2,
                    slowModeDelta.starts(),
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "Left bumper should toggle slow mode twice", "starts=2", slowModeDelta));

            FullFunctionalityHarness.CommandCounts fieldModeBefore =
                    context.recorder.getCounts("DriveToggleFieldOriented");
            context.tapDriverButton(context.driverControllerSim::setBackButton);
            assertTrue(!context.drive.isFieldOriented(), "Expected back button to toggle field orientation off.");
            context.tapDriverButton(context.driverControllerSim::setBackButton);
            assertTrue(context.drive.isFieldOriented(), "Expected back button to toggle field orientation back on.");
            FullFunctionalityHarness.CommandCounts fieldModeDelta =
                    context.recorder.getCounts("DriveToggleFieldOriented").minus(fieldModeBefore);
            assertEquals(
                    2,
                    fieldModeDelta.starts(),
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "Back button should toggle field-oriented mode twice", "starts=2", fieldModeDelta));

            context.drive.setPose(new Pose2d(context.drive.getPose().getTranslation(), Rotation2d.fromRadians(0.9)));
            FullFunctionalityHarness.CommandCounts resetBefore =
                    context.recorder.getCounts("DriveResetOdometryAndHeading");
            context.tapDriverButton(context.driverControllerSim::setStartButton);
            assertTrue(
                    Math.abs(context.drive.getPose().getRotation().getRadians()) <= 1e-3,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "Start button should reset heading to zero radians",
                            "headingRad≈0",
                            String.format(Locale.US, "headingRad=%.6f", context.drive.getPose().getRotation().getRadians())));
            FullFunctionalityHarness.CommandCounts resetDelta =
                    context.recorder.getCounts("DriveResetOdometryAndHeading").minus(resetBefore);
            assertEquals(
                    1,
                    resetDelta.starts(),
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "Start button should schedule reset once", "starts=1", resetDelta));

            context.drive.setPose(new Pose2d(context.drive.getPose().getTranslation(), Rotation2d.fromRadians(0.75)));
            FullFunctionalityHarness.CommandCounts snapBefore = context.recorder.getCounts("DriveHeadingSnap");
            context.tapDriverButton(context.driverControllerSim::setRightStickButton);
            context.runCycles(250);
            FullFunctionalityHarness.CommandCounts snapDelta =
                    context.recorder.getCounts("DriveHeadingSnap").minus(snapBefore);
            assertTrue(
                    snapDelta.starts() >= 1,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "Right stick press should schedule heading snap", "starts>=1", snapDelta));
            assertTrue(
                    snapDelta.finishes() >= 1,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "Heading snap should finish in teleop", "finishes>=1", snapDelta));

            FullFunctionalityHarness.CommandCounts intakeToggleBefore =
                    context.recorder.getCounts("IntakeToggleExtended");
            context.tapDriverButton(context.driverControllerSim::setBButton);
            context.runCycles(200);
            assertTrue(context.intake.isExtended(), "Expected B button to extend intake.");
            context.tapDriverButton(context.driverControllerSim::setBButton);
            context.runCycles(200);
            assertTrue(!context.intake.isExtended(), "Expected second B button press to retract intake.");
            FullFunctionalityHarness.CommandCounts intakeToggleDelta =
                    context.recorder.getCounts("IntakeToggleExtended").minus(intakeToggleBefore);
            assertEquals(
                    2,
                    intakeToggleDelta.starts(),
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "B button should trigger intake toggle twice", "starts=2", intakeToggleDelta));

            FullFunctionalityHarness.CommandCounts rollerBefore = context.recorder.getCounts("IntakeSpinRoller");
            context.driverControllerSim.setLeftTriggerAxis(1.0);
            context.runCycles(40);
            assertTrue(
                    context.recorder.runningCount("IntakeSpinRoller") >= 1,
                    "Expected left trigger hold to keep IntakeSpinRoller running.");
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

            FullFunctionalityHarness.CommandCounts reverseBefore = context.recorder.getCounts("TransferReverse");
            context.driverControllerSim.setYButton(true);
            context.runCycles(40);
            assertTrue(
                    context.transferInputs.appliedVolts < -1.0,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "Holding Y should command reverse transfer voltage",
                            "appliedVolts< -1.0",
                            String.format(Locale.US, "appliedVolts=%.3f", context.transferInputs.appliedVolts)));
            context.driverControllerSim.setYButton(false);
            context.runCycles(10);
            assertTrue(
                    Math.abs(context.transferInputs.appliedVolts) < 1e-6,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "Releasing Y should stop transfer",
                            "appliedVolts=0",
                            String.format(Locale.US, "appliedVolts=%.3f", context.transferInputs.appliedVolts)));
            FullFunctionalityHarness.CommandCounts reverseDelta =
                    context.recorder.getCounts("TransferReverse").minus(reverseBefore);
            assertTrue(
                    reverseDelta.starts() >= 1,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "Y should schedule TransferReverse", "starts>=1", reverseDelta));
            assertTrue(
                    reverseDelta.interrupts() >= 1,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "Releasing Y should interrupt TransferReverse", "interrupts>=1", reverseDelta));

            context.intake.setExtended(true);
            context.runCycles(20);
            FullFunctionalityHarness.CommandCounts slowRetractBefore = context.recorder.getCounts("IntakeSlowRetract");
            context.driverControllerSim.setXButton(true);
            context.runCycles(220);
            context.driverControllerSim.setXButton(false);
            context.runCycles(20);
            assertTrue(!context.intake.isExtended(), "Expected X hold to complete slow retract and clear extended state.");
            FullFunctionalityHarness.CommandCounts slowRetractDelta =
                    context.recorder.getCounts("IntakeSlowRetract").minus(slowRetractBefore);
            assertTrue(
                    slowRetractDelta.starts() >= 1,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "X should schedule IntakeSlowRetract", "starts>=1", slowRetractDelta));
            assertTrue(
                    slowRetractDelta.finishes() >= 1,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "Holding X should allow at least one IntakeSlowRetract run to finish",
                            "finishes>=1",
                            slowRetractDelta));
            List<FullFunctionalityHarness.CommandRun> slowRetractRuns = context.recorder.getRuns("IntakeSlowRetract");
            assertTrue(!slowRetractRuns.isEmpty(), "Expected to record IntakeSlowRetract run durations.");
            double longestSlowRetractSec =
                    slowRetractRuns.stream().mapToDouble(FullFunctionalityHarness.CommandRun::durationSec).max().orElse(0.0);
            assertTrue(
                    longestSlowRetractSec >= 1.20,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "At least one slow retract run should persist long enough to hit its timeout or setpoint wait",
                            "longestRun>=1.20s",
                            String.format(Locale.US, "longestRun=%.3fs", longestSlowRetractSec)));
            double leftAfterSlowRetract = Units.radiansToRotations(context.intakeInputs.leftPositionRad);
            assertTrue(
                    leftAfterSlowRetract < IntakeConstants.EXTENDED_POSITION_ROT - 4.0,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "Slow retract should make significant progress toward retracting left intake",
                            "leftPosition < (extended-4.0) rot",
                            leftAfterSlowRetract));

            FullFunctionalityHarness.CommandCounts scheduleBackgroundBefore =
                    context.recorder.getCounts("DriverShooterHome");
            context.tapDriverPov(0);
            FullFunctionalityHarness.CommandCounts scheduleBackgroundDelta =
                    context.recorder.getCounts("DriverShooterHome").minus(scheduleBackgroundBefore);
            assertEquals(
                    1,
                    scheduleBackgroundDelta.starts(),
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "POV up should schedule DriverShooterHome once",
                            "starts=1",
                            scheduleBackgroundDelta));

            SmartDashboard.putBoolean("Overrides/OverrideAutoAim", true);
            SmartDashboard.putNumber("Overrides/AimDistanceMeters", 3.0);
            SmartDashboard.putBoolean("Shooter/Tuning/Enabled", false);
            SmartDashboard.putBoolean("Shooter/Tuning/FeedKicker", false);
            context.runCycles(6);

            FullFunctionalityHarness.CommandCounts shootBefore =
                    context.recorder.getCounts("ShooterTriggerSelectedMode");
            context.driverControllerSim.setRightTriggerAxis(1.0);
            boolean kickerActivated = context.runUntil(context.shooter::isKickerActive, 420);
            assertTrue(
                    kickerActivated,
                    "Expected right trigger shoot mode with auto-aim override enabled to eventually activate kicker.");
            context.driverControllerSim.setRightTriggerAxis(0.0);
            context.runCycles(10);
            FullFunctionalityHarness.CommandCounts shootDelta =
                    context.recorder.getCounts("ShooterTriggerSelectedMode").minus(shootBefore);
            assertTrue(
                    shootDelta.starts() >= 1,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "Right trigger should schedule ShooterTriggerSelectedMode when tuning disabled",
                            "starts>=1",
                            shootDelta));
            assertTrue(
                    shootDelta.interrupts() >= 1,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "Releasing right trigger should interrupt ShooterTriggerSelectedMode",
                            "interrupts>=1",
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

            SmartDashboard.putBoolean("Shooter/Tuning/Enabled", false);
            context.runCycles(4);
            FullFunctionalityHarness.CommandCounts aimOnlyBefore =
                    context.recorder.getCounts("ShooterAimOnlySelectedMode");
            context.driverControllerSim.setRightBumperButton(true);
            context.runCycles(40);
            assertTrue(
                    context.recorder.runningCount("ShooterAimOnlySelectedMode") >= 1,
                    "Expected right bumper alone to run aim-only mode.");
            context.driverControllerSim.setRightBumperButton(false);
            context.runCycles(10);
            FullFunctionalityHarness.CommandCounts aimOnlyDelta =
                    context.recorder.getCounts("ShooterAimOnlySelectedMode").minus(aimOnlyBefore);
            assertTrue(
                    aimOnlyDelta.starts() >= 1,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "Right bumper should schedule aim-only mode", "starts>=1", aimOnlyDelta));
            assertTrue(
                    aimOnlyDelta.interrupts() >= 1,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "Releasing right bumper should interrupt aim-only mode",
                            "interrupts>=1",
                            aimOnlyDelta));

            FullFunctionalityHarness.CommandCounts aimOnlyBeforeCombined =
                    context.recorder.getCounts("ShooterAimOnlySelectedMode");
            context.driverControllerSim.setRightBumperButton(true);
            context.driverControllerSim.setRightTriggerAxis(1.0);
            context.runCycles(30);
            context.driverControllerSim.setRightBumperButton(false);
            context.driverControllerSim.setRightTriggerAxis(0.0);
            context.runCycles(10);
            FullFunctionalityHarness.CommandCounts aimOnlyCombinedDelta =
                    context.recorder.getCounts("ShooterAimOnlySelectedMode").minus(aimOnlyBeforeCombined);
            assertEquals(
                    0,
                    aimOnlyCombinedDelta.starts(),
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "Right bumper + right trigger together should not enter aim-only mode",
                            "starts=0",
                            aimOnlyCombinedDelta));

            FullFunctionalityHarness.CommandCounts tuneBefore = context.recorder.getCounts("ShooterDashboardTune");
            FullFunctionalityHarness.CommandCounts tuneTransferBefore =
                    context.recorder.getCounts("TransferDashboardTune");
            FullFunctionalityHarness.CommandCounts shootDuringTuneBefore =
                    context.recorder.getCounts("ShooterTriggerSelectedMode");
            SmartDashboard.putBoolean("Shooter/Tuning/Enabled", true);
            SmartDashboard.putBoolean("Shooter/Tuning/FeedKicker", true);
            context.runCycles(8);
            context.driverControllerSim.setRightTriggerAxis(1.0);
            context.runCycles(60);
            assertTrue(
                    context.recorder.runningCount("ShooterDashboardTune") >= 1,
                    "Expected right trigger to run ShooterDashboardTune when dashboard tuning enabled.");
            assertTrue(
                    context.recorder.runningCount("TransferDashboardTune") >= 1,
                    "Expected right trigger + dashboard feed enabled to run TransferDashboardTune.");
            context.driverControllerSim.setRightTriggerAxis(0.0);
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
                    Math.abs(context.shooter.getTargetAverageShooterRpm()) < 1e-6,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "Expected StopManipulators to zero shooter target",
                            "targetAverageRpm=0",
                            String.format(Locale.US, "targetAverageRpm=%.3f", context.shooter.getTargetAverageShooterRpm())));
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
                    "TransferReverse",
                    "IntakeToggleExtended",
                    "IntakeSlowRetract",
                    "IntakeSpinRoller",
                    "DriverShooterHome",
                    "StopManipulators",
                    "DriveSetOdometryFromUnifiedVision",
                    "ShooterTriggerSelectedMode",
                    "ShooterAimOnlySelectedMode",
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
}
