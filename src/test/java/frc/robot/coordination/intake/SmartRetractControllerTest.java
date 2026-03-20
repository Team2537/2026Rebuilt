package frc.robot.coordination.intake;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.intake.IntakeConstants;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class SmartRetractControllerTest {
    private SmartRetractController controller;
    private SmartRetractController.Session session;

    private static final double MID_POSITION =
            (IntakeConstants.RETRACTED_POSITION_ROT + IntakeConstants.EXTENDED_POSITION_ROT) / 2.0;

    @BeforeEach
    void setUp() {
        SmartDashboard.putNumber(
                "Intake/SmartRetract/RetractedPositionRot",
                IntakeConstants.SMART_RETRACT_RETRACTED_POSITION_ROT);
        SmartDashboard.putNumber(
                "Intake/SmartRetract/HalfRetractPositionRot",
                IntakeConstants.SMART_RETRACT_HALF_RETRACT_POSITION_ROT);
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
                "Intake/SmartRetract/NibbleCurrentThresholdAmps",
                IntakeConstants.SMART_RETRACT_NIBBLE_CURRENT_THRESHOLD_AMPS);
        SmartDashboard.putNumber(
                "Intake/SmartRetract/NibbleDetectCycles",
                IntakeConstants.SMART_RETRACT_NIBBLE_DETECT_CYCLES);
        SmartDashboard.putNumber(
                "Intake/SmartRetract/NibbleStepRot",
                IntakeConstants.SMART_RETRACT_NIBBLE_STEP_ROT);
        SmartDashboard.putNumber(
                "Intake/SmartRetract/NibbleBackoffRot",
                IntakeConstants.SMART_RETRACT_NIBBLE_BACKOFF_ROT);
        SmartDashboard.putNumber(
                "Intake/SmartRetract/NibbleBackoffDwellSec",
                IntakeConstants.SMART_RETRACT_NIBBLE_BACKOFF_DWELL_SEC);
        controller = new SmartRetractController();
        session = new SmartRetractController.Session();
    }

    @Test
    void disabledModeIsNotActive() {
        controller.initialize(session, SmartRetractController.Mode.DISABLED, true, MID_POSITION, 5.0);
        assertFalse(session.active());
        assertEquals(SmartRetractController.Mode.DISABLED, session.mode());
    }

    @Test
    void disabledModeDoesNotCommandRetract() {
        controller.initialize(session, SmartRetractController.Mode.DISABLED, true, MID_POSITION, 5.0);
        SmartRetractController.Update update =
                controller.update(session, true, MID_POSITION, 5.0, true, false);
        assertFalse(update.commandRetractTarget());
    }

    @Test
    void nibbleModeActivatesWhenStartedExtended() {
        controller.initialize(session, SmartRetractController.Mode.NIBBLE, true, MID_POSITION, 5.0);
        assertTrue(session.active());
        assertEquals(SmartRetractController.Mode.NIBBLE, session.mode());
    }

    @Test
    void nibbleModeNotActiveWhenNotStartedExtended() {
        controller.initialize(session, SmartRetractController.Mode.NIBBLE, false, MID_POSITION, 5.0);
        assertFalse(session.active());
    }

    @Test
    void feedLatchEngagesAfterConfiguredStartDelay() {
        controller.initialize(session, SmartRetractController.Mode.NIBBLE, true, MID_POSITION, 5.0);
        int startDelayCycles = feedStartDelayCycles();

        for (int i = 0; i < startDelayCycles - 1; i++) {
            SmartRetractController.Update update =
                    controller.update(session, true, MID_POSITION, 5.0, true, false);
            assertFalse(update.commandRetractTarget(), "Should not start smart retract before delay elapses");
        }

        SmartRetractController.Update update =
                controller.update(session, true, MID_POSITION, 5.0, true, false);
        assertTrue(update.commandRetractTarget(), "Should start smart retract after configured feed delay");
        assertTrue(session.feedLatched());
    }

    @Test
    void smartRetractPausesMovementWhenFeedDropsAndResumesWithoutReset() {
        controller.initialize(
                session,
                SmartRetractController.Mode.HALF_RETRACT_RETURN,
                true,
                IntakeConstants.EXTENDED_POSITION_ROT,
                5.0);

        for (int i = 0; i < feedStartDelayCycles(); i++) {
            controller.update(session, true, IntakeConstants.EXTENDED_POSITION_ROT, 5.0, true, false);
        }

        SmartRetractController.Update startUpdate =
                controller.update(session, true, IntakeConstants.EXTENDED_POSITION_ROT, 5.0, true, false);
        assertTrue(startUpdate.commandRetractTarget());
        assertEquals(
                IntakeConstants.SMART_RETRACT_HALF_RETRACT_POSITION_ROT,
                startUpdate.commandedLeftTargetRot(),
                1e-9,
                "Half-retract mode should command the midway target once started");

        double pausedPosition =
                (IntakeConstants.EXTENDED_POSITION_ROT + IntakeConstants.SMART_RETRACT_HALF_RETRACT_POSITION_ROT)
                        / 2.0;
        SmartRetractController.Update pausedUpdate =
                controller.update(session, false, pausedPosition, 5.0, true, false);
        assertTrue(pausedUpdate.commandRetractTarget());
        assertEquals(
                pausedPosition,
                pausedUpdate.commandedLeftTargetRot(),
                1e-9,
                "When feed is inactive, smart retract should hold at current position");

        SmartRetractController.Update resumedUpdate =
                controller.update(session, true, pausedPosition, 5.0, true, false);
        assertTrue(resumedUpdate.commandRetractTarget());
        assertEquals(
                IntakeConstants.SMART_RETRACT_HALF_RETRACT_POSITION_ROT,
                resumedUpdate.commandedLeftTargetRot(),
                1e-9,
                "When feed resumes, smart retract should continue the existing profile without reset");
    }

    @Test
    void halfRetractReturnCommandsExtendedAfterHalfwayReached() {
        controller.initialize(
                session,
                SmartRetractController.Mode.HALF_RETRACT_RETURN,
                true,
                IntakeConstants.EXTENDED_POSITION_ROT,
                5.0);

        for (int i = 0; i < feedStartDelayCycles(); i++) {
            controller.update(session, true, IntakeConstants.EXTENDED_POSITION_ROT, 5.0, true, false);
        }

        controller.update(session, true, IntakeConstants.EXTENDED_POSITION_ROT, 5.0, true, false);
        SmartRetractController.Update atHalfwayUpdate = controller.update(
                session,
                true,
                IntakeConstants.SMART_RETRACT_HALF_RETRACT_POSITION_ROT,
                5.0,
                true,
                false);
        assertEquals(
                IntakeConstants.SMART_RETRACT_HALF_RETRACT_POSITION_ROT,
                atHalfwayUpdate.commandedLeftTargetRot(),
                1e-9,
                "First halfway hit cycle still commands halfway target");

        SmartRetractController.Update returnUpdate = controller.update(
                session,
                true,
                IntakeConstants.SMART_RETRACT_HALF_RETRACT_POSITION_ROT,
                5.0,
                true,
                false);
        assertEquals(
                IntakeConstants.EXTENDED_POSITION_ROT,
                returnUpdate.commandedLeftTargetRot(),
                1e-9,
                "After halfway is reached, mode should command extension back out");
    }

    @Test
    void positionClampedToRetractedAndExtended() {
        controller.initialize(
                session, SmartRetractController.Mode.NIBBLE, true, IntakeConstants.RETRACTED_POSITION_ROT, 5.0);

        for (int i = 0; i < feedStartDelayCycles(); i++) {
            controller.update(
                    session,
                    true,
                    IntakeConstants.RETRACTED_POSITION_ROT,
                    5.0,
                    true,
                    false);
        }

        for (int i = 0; i < 500; i++) {
            controller.update(
                    session,
                    true,
                    IntakeConstants.RETRACTED_POSITION_ROT,
                    5.0,
                    true,
                    false);
        }
        assertTrue(
                session.commandedLeftTargetRot() >= IntakeConstants.SMART_RETRACT_RETRACTED_POSITION_ROT,
                "Position should not go below smart-retract retracted limit");
    }

    @Test
    void shouldRestoreExtendedOnExitWhenActiveAndNotRetracted() {
        controller.initialize(session, SmartRetractController.Mode.NIBBLE, true, MID_POSITION, 5.0);
        assertTrue(controller.shouldRestoreExtendedOnExit(session, false, false));
    }

    @Test
    void shouldNotRestoreOnExitWhenDisabled() {
        controller.initialize(session, SmartRetractController.Mode.NIBBLE, true, MID_POSITION, 5.0);
        assertFalse(controller.shouldRestoreExtendedOnExit(session, true, false));
    }

    @Test
    void shouldNotRestoreOnExitWhenAtRetractedTarget() {
        controller.initialize(session, SmartRetractController.Mode.NIBBLE, true, MID_POSITION, 5.0);
        assertFalse(controller.shouldRestoreExtendedOnExit(session, false, true));
    }

    @Test
    void currentFilteringAppliesExponentialSmoothing() {
        controller.initialize(session, SmartRetractController.Mode.NIBBLE, true, MID_POSITION, 0.0);

        for (int i = 0; i < feedStartDelayCycles(); i++) {
            controller.update(session, true, MID_POSITION, 50.0, true, false);
        }

        double filtered = session.filteredSignalCurrentAmps();
        assertTrue(filtered > 0.0, "Filtered current should be positive after high-current inputs");
        assertTrue(filtered < 50.0, "Filtered current should lag behind raw current");
    }

    @Test
    void nibbleUsesConstantCurrentThresholdInsteadOfBaselineDelta() {
        controller.initialize(session, SmartRetractController.Mode.NIBBLE, true, MID_POSITION, 20.0);
        assertEquals(
                IntakeConstants.SMART_RETRACT_NIBBLE_CURRENT_THRESHOLD_AMPS,
                session.nibbleCurrentThresholdAmps(),
                1e-9,
                "Nibble threshold should come from the fixed threshold constant");

        for (int i = 0; i < feedStartDelayCycles(); i++) {
            controller.update(session, true, MID_POSITION, 0.0, true, false);
        }

        boolean sawBackoff = false;
        for (int i = 0; i < 20; i++) {
            controller.update(session, true, MID_POSITION, 11.0, true, false);
            if (session.nibbleBackoffActive()) {
                sawBackoff = true;
                break;
            }
        }

        assertTrue(
                sawBackoff,
                "Fixed threshold should still trigger backoff even if the session started with a much higher initial current.");
    }

    @Test
    void nibbleDoesNotSpikeBelowFixedThreshold() {
        controller.initialize(session, SmartRetractController.Mode.NIBBLE, true, MID_POSITION, 0.0);

        for (int i = 0; i < feedStartDelayCycles(); i++) {
            controller.update(session, true, MID_POSITION, 0.0, true, false);
        }

        for (int i = 0; i < 20; i++) {
            controller.update(
                    session,
                    true,
                    MID_POSITION,
                    IntakeConstants.SMART_RETRACT_NIBBLE_CURRENT_THRESHOLD_AMPS - 1.0,
                    true,
                    false);
        }

        assertFalse(session.nibbleBackoffActive(), "Currents below the fixed threshold should not trigger nibble backoff.");
    }

    @Test
    void nibbleSettingsCanBeTunedFromDashboard() {
        SmartDashboard.putNumber("Intake/SmartRetract/RetractedPositionRot", 3.5);
        SmartDashboard.putNumber("Intake/SmartRetract/HalfRetractPositionRot", 12.0);
        SmartDashboard.putNumber("Intake/SmartRetract/CurrentFilterAlpha", 1.0);
        SmartDashboard.putNumber("Intake/SmartRetract/FeedEngageCycles", 5.0);
        SmartDashboard.putNumber("Intake/SmartRetract/FeedStartDelaySec", 0.02);
        SmartDashboard.putNumber("Intake/SmartRetract/NibbleCurrentThresholdAmps", 8.5);
        SmartDashboard.putNumber("Intake/SmartRetract/NibbleDetectCycles", 3.0);
        SmartDashboard.putNumber("Intake/SmartRetract/NibbleStepRot", 1.25);
        SmartDashboard.putNumber("Intake/SmartRetract/NibbleBackoffRot", 2.5);
        SmartDashboard.putNumber("Intake/SmartRetract/NibbleBackoffDwellSec", 0.6);

        controller.initialize(session, SmartRetractController.Mode.NIBBLE, true, 0.0, 0.0);
        assertEquals(8.5, session.nibbleCurrentThresholdAmps(), 1e-9);
        assertEquals(3.5, session.commandedLeftTargetRot(), 1e-9);

        controller.initialize(session, SmartRetractController.Mode.NIBBLE, true, MID_POSITION, 0.0);
        for (int i = 0; i < 4; i++) {
            SmartRetractController.Update preLatchUpdate =
                    controller.update(session, true, MID_POSITION, 50.0, true, false);
            assertFalse(preLatchUpdate.commandRetractTarget(), "Feed engage cycles tuning should delay latching.");
        }

        SmartRetractController.Update stepUpdate =
                controller.update(session, true, MID_POSITION, 0.0, true, false);
        assertEquals(MID_POSITION - 1.25, stepUpdate.commandedLeftTargetRot(), 1e-9);
        assertEquals(0.0, session.filteredSignalCurrentAmps(), 1e-9);

        controller.update(session, true, MID_POSITION, 50.0, true, false);
        controller.update(session, true, MID_POSITION, 50.0, true, false);
        assertFalse(session.nibbleBackoffActive(), "Detect cycles tuning should delay backoff until the configured count.");

        SmartRetractController.Update backoffUpdate =
                controller.update(session, true, MID_POSITION, 50.0, true, false);
        assertTrue(session.nibbleBackoffActive(), "Third spike should trigger backoff when detect cycles is tuned to 3.");
        assertEquals(MID_POSITION + 2.5, backoffUpdate.commandedLeftTargetRot(), 1e-9);

        controller.initialize(
                session,
                SmartRetractController.Mode.HALF_RETRACT_RETURN,
                true,
                IntakeConstants.EXTENDED_POSITION_ROT,
                0.0);
        for (int i = 0; i < 4; i++) {
            controller.update(session, true, IntakeConstants.EXTENDED_POSITION_ROT, 0.0, true, false);
        }
        SmartRetractController.Update halfRetractUpdate =
                controller.update(session, true, IntakeConstants.EXTENDED_POSITION_ROT, 0.0, true, false);
        assertEquals(12.0, halfRetractUpdate.commandedLeftTargetRot(), 1e-9);
    }

    private static int feedStartDelayCycles() {
        int configuredDelayCycles =
                (int) Math.ceil(IntakeConstants.smartRetractFeedStartDelaySec() * IntakeConstants.STATUS_UPDATE_HZ);
        return Math.max(IntakeConstants.smartRetractFeedEngageCycles(), Math.max(1, configuredDelayCycles));
    }
}
