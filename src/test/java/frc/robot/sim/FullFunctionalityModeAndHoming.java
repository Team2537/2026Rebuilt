package frc.robot.sim;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.shooter.ShooterConstants;
import java.util.Locale;
import org.junit.jupiter.api.Test;

final class FullFunctionalityModeAndHoming {
    @Test
    void autonomousInitStartsHomingImmediatelyWhenAutoDoesNotUseManipulators() {
        try (FullFunctionalityHarness.Context context = new FullFunctionalityHarness.Context(false)) {
            context.setAutonomousEnabled();
            context.runCycles(4);

            FullFunctionalityHarness.CommandCounts intakeHomeBefore =
                    context.recorder.getCounts("IntakeHomeThenBackground");
            FullFunctionalityHarness.CommandCounts shooterHomeBefore =
                    context.recorder.getCounts("ShooterHomeThenBackground");

            context.container.autonomousInit();

            FullFunctionalityHarness.CommandCounts intakeImmediateDelta =
                    context.recorder.getCounts("IntakeHomeThenBackground").minus(intakeHomeBefore);
            FullFunctionalityHarness.CommandCounts shooterImmediateDelta =
                    context.recorder.getCounts("ShooterHomeThenBackground").minus(shooterHomeBefore);

            assertTrue(
                    intakeImmediateDelta.starts() >= 1,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "autonomousInit should start IntakeHomeThenBackground immediately (without waiting for scheduler cycles)",
                            "starts>=1 immediately",
                            intakeImmediateDelta));
            assertTrue(
                    shooterImmediateDelta.starts() >= 1,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "autonomousInit should start ShooterHomeThenBackground immediately (without waiting for scheduler cycles)",
                            "starts>=1 immediately",
                            shooterImmediateDelta));
            assertTrue(
                    context.recorder.runningCount("IntakeHomeThenBackground") >= 1,
                    "Expected IntakeHomeThenBackground to be running immediately after autonomousInit.");
            assertTrue(
                    context.recorder.runningCount("ShooterHomeThenBackground") >= 1,
                    "Expected ShooterHomeThenBackground to be running immediately after autonomousInit.");
        }
    }

    @Test
    void modeTransitionsScheduleAndCancelExpectedCommands() {
        try (FullFunctionalityHarness.Context context = new FullFunctionalityHarness.Context(false)) {
            context.setTeleopEnabled();
            context.container.teleopInit();
            context.runCycles(30);

            context.intake.setExtended(true);
            context.shooter.setTargets(1800.0, 1800.0, Units.degreesToRadians(65.0));
            Command preDisabledTransfer = context.transfer.runCommand().withName("FF_PreDisabledTransferRun");
            CommandScheduler.getInstance().schedule(preDisabledTransfer);
            context.runCycles(20);
            assertTrue(
                    context.transferInputs.appliedVolts > 1.0,
                    "Precondition failed: transfer should be actively running before entering disabled.");

            context.setDisabled();
            FullFunctionalityHarness.CommandCounts stopBeforeDisabled =
                    context.recorder.getCounts("StopManipulators");
            context.container.disabledInit();
            context.runCycles(8);
            FullFunctionalityHarness.CommandCounts stopAfterDisabled =
                    context.recorder.getCounts("StopManipulators");
            FullFunctionalityHarness.CommandCounts stopDisabledDelta = stopAfterDisabled.minus(stopBeforeDisabled);

            assertEquals(
                    0,
                    stopDisabledDelta.starts(),
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "disabledInit StopManipulators should not initialize while robot is disabled",
                            "starts=0",
                            stopDisabledDelta));
            assertTrue(
                    Math.abs(context.transferInputs.appliedVolts) < 1e-6,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "Disabled should force transfer voltage to zero immediately",
                            "transferAppliedVolts=0",
                            String.format(Locale.US, "transferAppliedVolts=%.3f", context.transferInputs.appliedVolts)));
            assertTrue(
                    Math.abs(context.shooter.getTargetAverageShooterRpm()) < 1e-6,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "Disabled should force shooter target rpm to zero",
                            "targetAverageRpm=0",
                            String.format(Locale.US, "targetAverageRpm=%.3f", context.shooter.getTargetAverageShooterRpm())));
            assertTrue(
                    Math.abs(context.intakeInputs.leftAppliedVolts) < 1e-6
                            && Math.abs(context.intakeInputs.rightAppliedVolts) < 1e-6,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "Disabled should force intake arm voltages to zero",
                            "leftAppliedVolts=0 and rightAppliedVolts=0",
                            String.format(
                                    Locale.US,
                                    "leftAppliedVolts=%.3f rightAppliedVolts=%.3f",
                                    context.intakeInputs.leftAppliedVolts,
                                    context.intakeInputs.rightAppliedVolts)));

            context.setTeleopEnabled();
            FullFunctionalityHarness.CommandCounts intakeHomeBefore =
                    context.recorder.getCounts("IntakeHomeThenBackground");
            FullFunctionalityHarness.CommandCounts shooterHomeBefore =
                    context.recorder.getCounts("ShooterHomeThenBackground");
            context.container.teleopInit();
            context.runCycles(80);
            FullFunctionalityHarness.CommandCounts intakeHomeDelta =
                    context.recorder.getCounts("IntakeHomeThenBackground").minus(intakeHomeBefore);
            FullFunctionalityHarness.CommandCounts shooterHomeDelta =
                    context.recorder.getCounts("ShooterHomeThenBackground").minus(shooterHomeBefore);

            assertTrue(
                    intakeHomeDelta.starts() >= 1,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "teleopInit should schedule IntakeHomeThenBackground",
                            "starts>=1",
                            intakeHomeDelta));
            assertTrue(
                    shooterHomeDelta.starts() >= 1,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "teleopInit should schedule ShooterHomeThenBackground",
                            "starts>=1",
                            shooterHomeDelta));
            assertTrue(
                    context.recorder.runningCount("IntakeHomeThenBackground") >= 1,
                    "Expected IntakeHomeThenBackground to remain active as background command in teleop.");
            assertTrue(
                    context.recorder.runningCount("ShooterHomeThenBackground") >= 1,
                    "Expected ShooterHomeThenBackground to remain active as background command in teleop.");

            FullFunctionalityHarness.CommandCounts intakeHomeBeforeCancel =
                    context.recorder.getCounts("IntakeHomeThenBackground");
            FullFunctionalityHarness.CommandCounts shooterHomeBeforeCancel =
                    context.recorder.getCounts("ShooterHomeThenBackground");
            context.container.testInit();
            context.runCycles(6);
            FullFunctionalityHarness.CommandCounts intakeHomeCancelDelta =
                    context.recorder.getCounts("IntakeHomeThenBackground").minus(intakeHomeBeforeCancel);
            FullFunctionalityHarness.CommandCounts shooterHomeCancelDelta =
                    context.recorder.getCounts("ShooterHomeThenBackground").minus(shooterHomeBeforeCancel);

            assertTrue(
                    intakeHomeCancelDelta.interrupts() >= 1,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "testInit should interrupt active IntakeHomeThenBackground",
                            "interrupts>=1",
                            intakeHomeCancelDelta));
            assertTrue(
                    shooterHomeCancelDelta.interrupts() >= 1,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "testInit should interrupt active ShooterHomeThenBackground",
                            "interrupts>=1",
                            shooterHomeCancelDelta));

            context.setAutonomousEnabled();
            FullFunctionalityHarness.CommandCounts autoNoneBefore = context.recorder.getCounts("AutoNone");
            FullFunctionalityHarness.CommandCounts intakeAutoBefore =
                    context.recorder.getCounts("IntakeHomeThenBackground");
            FullFunctionalityHarness.CommandCounts shooterAutoBefore =
                    context.recorder.getCounts("ShooterHomeThenBackground");
            context.container.autonomousInit();
            context.runCycles(20);
            FullFunctionalityHarness.CommandCounts autoNoneDelta =
                    context.recorder.getCounts("AutoNone").minus(autoNoneBefore);
            FullFunctionalityHarness.CommandCounts intakeAutoDelta =
                    context.recorder.getCounts("IntakeHomeThenBackground").minus(intakeAutoBefore);
            FullFunctionalityHarness.CommandCounts shooterAutoDelta =
                    context.recorder.getCounts("ShooterHomeThenBackground").minus(shooterAutoBefore);

            assertTrue(
                    autoNoneDelta.starts() >= 1,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "autonomousInit with default selector should schedule AutoNone",
                            "starts>=1",
                            autoNoneDelta));
            assertTrue(
                    autoNoneDelta.finishes() >= 1,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "AutoNone should finish immediately",
                            "finishes>=1",
                            autoNoneDelta));
            assertTrue(
                    intakeAutoDelta.starts() >= 1,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "autonomousInit should schedule IntakeHomeThenBackground when selected auto does not require intake/shooter",
                            "starts>=1",
                            intakeAutoDelta));
            assertTrue(
                    shooterAutoDelta.starts() >= 1,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "autonomousInit should schedule ShooterHomeThenBackground when selected auto does not require intake/shooter",
                            "starts>=1",
                            shooterAutoDelta));
        }
    }

    @Test
    void homingStartsFinishesInReasonableTimeAndUpdatesEncoderPositions() {
        try (FullFunctionalityHarness.Context context = new FullFunctionalityHarness.Context(false)) {
            context.setTeleopEnabled();
            context.container.teleopInit();
            context.runCycles(20);

            Command prepExtend = context.intake.extendCommand().withName("FF_PrepIntakeExtend");
            CommandScheduler.getInstance().schedule(prepExtend);
            boolean prepFinished = context.runUntil(() -> !CommandScheduler.getInstance().isScheduled(prepExtend), 300);
            assertTrue(prepFinished, "Preparation intake extend should finish before homing test.");

            Command intakeHome = context.intake.homeCommand().withName("FF_IntakeHomeDirect");
            FullFunctionalityHarness.CommandCounts intakeHomeBefore =
                    context.recorder.getCounts("FF_IntakeHomeDirect");
            CommandScheduler.getInstance().schedule(intakeHome);

            boolean sawLeftNegativeHomingVolts = false;
            boolean sawRightNegativeHomingVolts = false;
            boolean sawLeftHomingCurrentThreshold = false;
            boolean sawRightHomingCurrentThreshold = false;
            boolean intakeHomeFinished = false;
            for (int i = 0; i < 250; i++) {
                context.runCycles(1);
                sawLeftNegativeHomingVolts |= context.intakeInputs.leftAppliedVolts < -0.5;
                sawRightNegativeHomingVolts |= context.intakeInputs.rightAppliedVolts < -0.5;
                sawLeftHomingCurrentThreshold |=
                        Math.abs(context.intakeInputs.leftStatorCurrentAmps) > IntakeConstants.HOMING_CURRENT_THRESHOLD_AMPS;
                sawRightHomingCurrentThreshold |=
                        Math.abs(context.intakeInputs.rightStatorCurrentAmps) > IntakeConstants.HOMING_CURRENT_THRESHOLD_AMPS;
                if (!CommandScheduler.getInstance().isScheduled(intakeHome)) {
                    intakeHomeFinished = true;
                    break;
                }
            }
            assertTrue(intakeHomeFinished, "Intake home should finish in simulation.");

            FullFunctionalityHarness.CommandCounts intakeHomeDelta =
                    context.recorder.getCounts("FF_IntakeHomeDirect").minus(intakeHomeBefore);
            FullFunctionalityHarness.CommandRun intakeRun = context.recorder.latestRun("FF_IntakeHomeDirect");
            assertNotNull(intakeRun, "Expected a recorded run window for intake home.");
            assertEquals(
                    1,
                    intakeHomeDelta.starts(),
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "Intake home should start once",
                            "starts=1",
                            intakeHomeDelta));
            assertEquals(
                    1,
                    intakeHomeDelta.finishes(),
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "Intake home should finish once",
                            "finishes=1",
                            intakeHomeDelta));
            assertEquals(
                    0,
                    intakeHomeDelta.interrupts(),
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "Intake home should not be interrupted",
                            "interrupts=0",
                            intakeHomeDelta));
            assertTrue(
                    intakeRun.durationSec() >= 0.05 && intakeRun.durationSec() <= 1.40,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "Intake home duration should be reasonable in sim",
                            "0.05s <= duration <= 1.40s",
                            String.format(Locale.US, "duration=%.3fs", intakeRun.durationSec())));
            assertTrue(sawLeftNegativeHomingVolts, "Intake home should drive left intake negative during homing.");
            assertTrue(sawRightNegativeHomingVolts, "Intake home should drive right intake negative during homing.");
            assertTrue(
                    sawLeftHomingCurrentThreshold,
                    "Intake home should detect left homing current threshold before completing.");
            assertTrue(
                    sawRightHomingCurrentThreshold,
                    "Intake home should detect right homing current threshold before completing.");

            double leftRotReset = Units.radiansToRotations(context.intakeInputs.leftPositionRad);
            double rightRotReset = Units.radiansToRotations(context.intakeInputs.rightPositionRad);
            assertTrue(
                    Math.abs(leftRotReset) <= 0.5,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "Intake home should reset left encoder reference near zero before retract setpoint",
                            "0.0±0.5 rot",
                            leftRotReset));
            assertTrue(
                    Math.abs(rightRotReset) <= 0.6,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "Intake home should reset right encoder reference near zero before retract setpoint",
                            "0.0±0.6 rot",
                            rightRotReset));

            context.runCycles(160);
            double leftRotSettled = Units.radiansToRotations(context.intakeInputs.leftPositionRad);
            double rightRotSettled = Units.radiansToRotations(context.intakeInputs.rightPositionRad);
            double expectedRightRot = IntakeConstants.RIGHT_OPPOSES_LEFT
                    ? -IntakeConstants.RETRACTED_POSITION_ROT
                    : IntakeConstants.RETRACTED_POSITION_ROT;
            assertTrue(
                    Math.abs(leftRotSettled - IntakeConstants.RETRACTED_POSITION_ROT) <= 0.80,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "Intake home should drive left encoder to retracted target after settle time",
                            IntakeConstants.RETRACTED_POSITION_ROT + "±0.80 rot",
                            leftRotSettled));
            assertTrue(
                    Math.abs(rightRotSettled - expectedRightRot) <= 0.50,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "Intake home should drive right encoder to retracted target after settle time",
                            expectedRightRot + "±0.50 rot",
                            rightRotSettled));
            assertTrue(!context.intake.isExtended(), "Intake home should end with intake not extended.");

            context.shooter.setTargets(2200.0, 2200.0, Units.degreesToRadians(70.0));
            context.runCycles(120);
            assertTrue(
                    context.shooterInputs.hoodPositionRad > Units.degreesToRadians(40.0),
                    "Shooter prep should move hood away from home before homing.");

            Command shooterHome = context.shooter.homeCommand().withName("FF_ShooterHomeDirect");
            FullFunctionalityHarness.CommandCounts shooterHomeBefore =
                    context.recorder.getCounts("FF_ShooterHomeDirect");
            CommandScheduler.getInstance().schedule(shooterHome);

            boolean sawNegativeHoodVolts = false;
            boolean sawHoodHomingCurrentThreshold = false;
            boolean shooterHomeFinished = false;
            for (int i = 0; i < 300; i++) {
                context.runCycles(1);
                sawNegativeHoodVolts |= context.shooterInputs.hoodAppliedVolts < -8.0;
                sawHoodHomingCurrentThreshold |=
                        Math.abs(context.shooterInputs.hoodStatorCurrentAmps) > ShooterConstants.HOMING_CURRENT_THRESHOLD_AMPS;
                if (!CommandScheduler.getInstance().isScheduled(shooterHome)) {
                    shooterHomeFinished = true;
                    break;
                }
            }
            assertTrue(shooterHomeFinished, "Shooter home should finish in simulation.");
            assertTrue(sawNegativeHoodVolts, "Shooter home should apply negative hood homing voltage.");
            assertTrue(
                    sawHoodHomingCurrentThreshold,
                    "Shooter home should detect hood homing current threshold before completing.");

            FullFunctionalityHarness.CommandCounts shooterHomeDelta =
                    context.recorder.getCounts("FF_ShooterHomeDirect").minus(shooterHomeBefore);
            FullFunctionalityHarness.CommandRun shooterRun = context.recorder.latestRun("FF_ShooterHomeDirect");
            assertNotNull(shooterRun, "Expected a recorded run window for shooter home.");
            assertEquals(
                    1,
                    shooterHomeDelta.starts(),
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "Shooter home should start once",
                            "starts=1",
                            shooterHomeDelta));
            assertEquals(
                    1,
                    shooterHomeDelta.finishes(),
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "Shooter home should finish once",
                            "finishes=1",
                            shooterHomeDelta));
            assertEquals(
                    0,
                    shooterHomeDelta.interrupts(),
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "Shooter home should not be interrupted",
                            "interrupts=0",
                            shooterHomeDelta));
            assertTrue(
                    shooterRun.durationSec() >= 0.05 && shooterRun.durationSec() <= 1.30,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "Shooter home duration should be reasonable in sim",
                            "0.05s <= duration <= 1.30s",
                            String.format(Locale.US, "duration=%.3fs", shooterRun.durationSec())));

            double preAimRad = ShooterConstants.SHOT_MAP_HOOD_ANGLE_DEG.length > 0
                    ? Units.degreesToRadians(ShooterConstants.SHOT_MAP_HOOD_ANGLE_DEG[0])
                    : ShooterConstants.HOOD_MIN_ANGLE_RAD;
            assertTrue(
                    Math.abs(context.shooterInputs.hoodPositionRad - preAimRad) <= Units.degreesToRadians(2.0),
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "Shooter home should reset hood and settle near pre-aim angle",
                            Math.toDegrees(preAimRad) + "deg ±2.0deg",
                            Math.toDegrees(context.shooterInputs.hoodPositionRad) + "deg"));
        }
    }
}
