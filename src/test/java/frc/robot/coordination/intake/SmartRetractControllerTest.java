package frc.robot.coordination.intake;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.intake.IntakeConstants;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class SmartRetractControllerTest {
    private static final double LOOP_PERIOD_SEC = 1.0 / IntakeConstants.STATUS_UPDATE_HZ;
    private static final double MID_POSITION =
            (IntakeConstants.RETRACTED_POSITION_ROT + IntakeConstants.EXTENDED_POSITION_ROT) / 2.0;

    private SmartRetractController controller;
    private SmartRetractController.Session session;
    private double nowSec;

    @BeforeEach
    void setUp() {
        SmartDashboard.putNumber(
                "Intake/SmartRetract/RetractedPositionRot",
                IntakeConstants.SMART_RETRACT_RETRACTED_POSITION_ROT);
        SmartDashboard.putNumber(
                "Intake/SmartRetract/CurrentFilterAlpha",
                IntakeConstants.SMART_RETRACT_CURRENT_FILTER_ALPHA);
        SmartDashboard.putNumber(
                "Intake/SmartRetract/FeedEngageCycles",
                IntakeConstants.SMART_RETRACT_FEED_ENGAGE_CYCLES);
        SmartDashboard.putNumber(
                "Intake/SmartRetract/FeedStartDelaySec",
                IntakeConstants.SMART_RETRACT_FEED_START_DELAY_SEC);
        SmartDashboard.putNumber(
                "Intake/SmartRetract/StepRot",
                IntakeConstants.SMART_RETRACT_STEP_ROT);
        SmartDashboard.putNumber(
                "Intake/SmartRetract/RollerRpm",
                IntakeConstants.SMART_RETRACT_ROLLER_RPM);
        SmartDashboard.putNumber(
                "Intake/SmartRetract/JamBackoffCurrentThresholdAmps",
                IntakeConstants.SMART_RETRACT_JAM_BACKOFF_CURRENT_THRESHOLD_AMPS);
        SmartDashboard.putNumber(
                "Intake/SmartRetract/JamBackoffDetectCycles",
                IntakeConstants.SMART_RETRACT_JAM_BACKOFF_DETECT_CYCLES);
        SmartDashboard.putNumber(
                "Intake/SmartRetract/JamFirstShotTimeoutSec",
                IntakeConstants.SMART_RETRACT_JAM_FIRST_SHOT_TIMEOUT_SEC);
        SmartDashboard.putNumber(
                "Intake/SmartRetract/JamInterShotTimeoutSec",
                IntakeConstants.SMART_RETRACT_JAM_INTER_SHOT_TIMEOUT_SEC);
        SmartDashboard.putNumber(
                "Intake/SmartRetract/JamRecoveryExtendPositionRot",
                IntakeConstants.SMART_RETRACT_JAM_RECOVERY_EXTEND_POSITION_ROT);
        SmartDashboard.putNumber(
                "Intake/SmartRetract/InnerStallRecoveryExtendPositionRot",
                IntakeConstants.SMART_RETRACT_INNER_STALL_RECOVERY_EXTEND_POSITION_ROT);
        SmartDashboard.putNumber(
                "Intake/SmartRetract/TailDrainGraceSec",
                IntakeConstants.SMART_RETRACT_TAIL_DRAIN_GRACE_SEC);
        SmartDashboard.putNumber(
                "Intake/SmartRetract/WiggleOutRot",
                IntakeConstants.SMART_RETRACT_WIGGLE_OUT_ROT);
        SmartDashboard.putNumber(
                "Intake/SmartRetract/WiggleSwitchIntervalSec",
                IntakeConstants.SMART_RETRACT_WIGGLE_SWITCH_INTERVAL_SEC);

        controller = new SmartRetractController();
        session = new SmartRetractController.Session();
        nowSec = 0.0;
    }

    @Test
    void sessionInactiveWhenNotStartedExtended() {
        initialize(false, MID_POSITION, 0.0);
        SmartRetractController.Update update = update(false, false, MID_POSITION, 0.0);
        assertFalse(session.active());
        assertEquals(SmartRetractController.Phase.INACTIVE, session.phase());
        assertFalse(update.commandTarget());
    }

    @Test
    void retractStartsImmediatelyBeforeFeed() {
        initialize(true, MID_POSITION, 0.0);
        SmartRetractController.Update update = update(false, false, MID_POSITION, 0.0);
        assertTrue(update.commandTarget());
        assertEquals(MID_POSITION - IntakeConstants.smartRetractStepRot(), update.commandedLeftTargetRot(), 1e-9);
        assertEquals(SmartRetractController.Phase.SEEKING_FLOW, session.phase());
    }

    @Test
    void droppingFeedDoesNotHoldPosition() {
        initialize(true, MID_POSITION, 0.0);
        SmartRetractController.Update first = update(true, false, MID_POSITION, 0.0);
        double currentTarget = first.commandedLeftTargetRot();

        SmartRetractController.Update noFeed = update(false, false, MID_POSITION, 0.0);
        assertTrue(noFeed.commandedLeftTargetRot() < currentTarget);
        assertEquals(SmartRetractController.Phase.SEEKING_FLOW, session.phase());
    }

    @Test
    void firstPulseGetsBearingsAfterFeedLatch() {
        initialize(true, MID_POSITION, 0.0);
        latchFeed(MID_POSITION, 0.0);
        assertTrue(session.feedLatched());

        update(true, true, MID_POSITION, 0.0);
        assertTrue(session.sawShotPulse());
        assertEquals(SmartRetractController.Phase.FLOWING, session.phase());
    }

    @Test
    void outerJamRecoveryTriggersOutsideInnerRegion() {
        initialize(true, IntakeConstants.EXTENDED_POSITION_ROT, 0.0);

        SmartRetractController.Update update = null;
        for (int i = 0; i < IntakeConstants.smartRetractJamBackoffDetectCycles(); i++) {
            update = update(false, false, IntakeConstants.EXTENDED_POSITION_ROT, 20.0);
        }

        assertEquals(SmartRetractController.Phase.OUTER_JAM_RECOVERY, session.phase());
        assertTrue(update != null);
        assertEquals(IntakeConstants.smartRetractJamRecoveryExtendPositionRot(), update.commandedLeftTargetRot(), 1e-9);
    }

    @Test
    void noFirstPulseNearRetractedStartsInnerRecovery() {
        SmartDashboard.putNumber("Intake/SmartRetract/JamFirstShotTimeoutSec", 0.04);
        initialize(true, IntakeConstants.smartRetractInnerStallRecoveryExtendPositionRot() - 0.5, 0.0);
        latchFeed(IntakeConstants.smartRetractInnerStallRecoveryExtendPositionRot() - 0.5, 0.0);

        update(true, false, IntakeConstants.smartRetractInnerStallRecoveryExtendPositionRot() - 0.5, 0.0);
        SmartRetractController.Update timeoutUpdate =
                update(true, false, IntakeConstants.smartRetractInnerStallRecoveryExtendPositionRot() - 0.5, 0.0);

        assertEquals(SmartRetractController.Phase.INNER_STALL_RECOVERY, session.phase());
        assertEquals(IntakeConstants.smartRetractInnerStallRecoveryExtendPositionRot(), timeoutUpdate.commandedLeftTargetRot(), 1e-9);
    }

    @Test
    void fullRetractKeepsWiggleActiveEvenBeforePulse() {
        initialize(true, MID_POSITION, 0.0);
        SmartRetractController.Update update = null;
        double position = MID_POSITION;
        for (int i = 0; i < 100; i++) {
            position = Math.max(IntakeConstants.smartRetractRetractedPositionRot(), position - 0.5);
            update = update(false, false, position, 0.0);
            if (session.fullRetractReached()) {
                break;
            }
        }
        assertTrue(session.fullRetractReached());
        assertTrue(update != null && update.wiggleActive());
    }

    @Test
    void lowCurrentPulseGapEntersTailDrainBeforeRecovering() {
        SmartDashboard.putNumber("Intake/SmartRetract/JamInterShotTimeoutSec", 0.04);
        SmartDashboard.putNumber("Intake/SmartRetract/TailDrainGraceSec", 0.06);
        double nearRetracted = IntakeConstants.smartRetractRetractedPositionRot() + 0.6;
        initialize(true, nearRetracted, 0.0);
        latchFeed(nearRetracted, 0.0);
        update(true, false, IntakeConstants.smartRetractRetractedPositionRot(), 0.0);
        update(true, true, IntakeConstants.smartRetractRetractedPositionRot(), 0.0);
        assertTrue(session.fullRetractReached());

        update(true, false, IntakeConstants.smartRetractRetractedPositionRot(), 0.0);
        update(true, false, IntakeConstants.smartRetractRetractedPositionRot(), 0.0);
        assertEquals(SmartRetractController.Phase.TAIL_DRAIN, session.phase());

        update(true, false, IntakeConstants.smartRetractRetractedPositionRot(), 0.0);
        update(true, false, IntakeConstants.smartRetractRetractedPositionRot(), 0.0);
        update(true, false, IntakeConstants.smartRetractRetractedPositionRot(), 0.0);
        assertEquals(SmartRetractController.Phase.INNER_STALL_RECOVERY, session.phase());
    }

    @Test
    void feedPauseDisarmsShotTimeoutUntilFeedReturns() {
        SmartDashboard.putNumber("Intake/SmartRetract/JamInterShotTimeoutSec", 0.04);
        double nearRetracted = IntakeConstants.smartRetractRetractedPositionRot() + 0.6;
        initialize(true, nearRetracted, 0.0);
        latchFeed(nearRetracted, 0.0);
        update(true, false, IntakeConstants.smartRetractRetractedPositionRot(), 0.0);
        update(true, true, IntakeConstants.smartRetractRetractedPositionRot(), 0.0);

        update(false, false, IntakeConstants.smartRetractRetractedPositionRot(), 0.0);
        update(false, false, IntakeConstants.smartRetractRetractedPositionRot(), 0.0);
        update(false, false, IntakeConstants.smartRetractRetractedPositionRot(), 0.0);
        assertFalse(session.jamRecoveryActive());

        latchFeed(IntakeConstants.smartRetractRetractedPositionRot(), 0.0);
        update(true, false, IntakeConstants.smartRetractRetractedPositionRot(), 0.0);
        update(true, false, IntakeConstants.smartRetractRetractedPositionRot(), 0.0);
        assertFalse(session.jamRecoveryActive());
    }

    @Test
    void shouldRestoreExtendedOnExitOnlyBeforeFullRetract() {
        initialize(true, MID_POSITION, 0.0);
        assertTrue(controller.shouldRestoreExtendedOnExit(session, false, false));

        session = new SmartRetractController.Session();
        initialize(true, MID_POSITION, 0.0);
        update(false, false, IntakeConstants.smartRetractRetractedPositionRot(), 0.0);
        assertTrue(session.fullRetractReached());
        assertFalse(controller.shouldRestoreExtendedOnExit(session, false, false));
    }

    private void initialize(boolean startedExtended, double initialLeftPositionRot, double initialSignalCurrentAmps) {
        controller.initialize(session, startedExtended, initialLeftPositionRot, initialSignalCurrentAmps);
    }

    private SmartRetractController.Update update(
            boolean activelyFeeding,
            boolean shotPulseDetectedThisCycle,
            double leftPositionRot,
            double rawSignalCurrentAmps) {
        nowSec += LOOP_PERIOD_SEC;
        return controller.update(
                session,
                new SmartRetractController.Inputs(
                        activelyFeeding,
                        shotPulseDetectedThisCycle,
                        leftPositionRot,
                        rawSignalCurrentAmps),
                nowSec);
    }

    private void latchFeed(double leftPositionRot, double rawSignalCurrentAmps) {
        for (int i = 0; i < feedStartDelayCycles(); i++) {
            update(true, false, leftPositionRot, rawSignalCurrentAmps);
        }
    }

    private static int feedStartDelayCycles() {
        int configuredDelayCycles =
                (int) Math.ceil(IntakeConstants.smartRetractFeedStartDelaySec() * IntakeConstants.STATUS_UPDATE_HZ);
        return Math.max(IntakeConstants.smartRetractFeedEngageCycles(), Math.max(1, configuredDelayCycles));
    }
}
