package frc.robot.coordination.intake;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

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
    void feedLatchEngagesAfterConfiguredTrueCycles() {
        controller.initialize(session, SmartRetractController.Mode.NIBBLE, true, MID_POSITION, 5.0);
        int engageCycles = IntakeConstants.SMART_RETRACT_FEED_ENGAGE_CYCLES;

        for (int i = 0; i < engageCycles - 1; i++) {
            SmartRetractController.Update update =
                    controller.update(session, true, MID_POSITION, 5.0, true, false);
            assertFalse(update.commandRetractTarget(), "Should not latch before engage cycles");
        }

        SmartRetractController.Update update =
                controller.update(session, true, MID_POSITION, 5.0, true, false);
        assertTrue(update.commandRetractTarget(), "Should latch after enough true cycles");
        assertTrue(session.feedLatched());
    }

    @Test
    void feedLatchReleasesAfterConfiguredFalseCycles() {
        controller.initialize(session, SmartRetractController.Mode.NIBBLE, true, MID_POSITION, 5.0);
        int engageCycles = IntakeConstants.SMART_RETRACT_FEED_ENGAGE_CYCLES;
        int releaseCycles = IntakeConstants.SMART_RETRACT_FEED_RELEASE_CYCLES;

        // Engage the latch
        for (int i = 0; i < engageCycles; i++) {
            controller.update(session, true, MID_POSITION, 5.0, true, false);
        }
        assertTrue(session.feedLatched());

        // Release: need releaseCycles false cycles
        for (int i = 0; i < releaseCycles - 1; i++) {
            controller.update(session, false, MID_POSITION, 5.0, true, false);
            assertTrue(session.feedLatched(), "Should not release before enough false cycles");
        }
        controller.update(session, false, MID_POSITION, 5.0, true, false);
        assertFalse(session.feedLatched(), "Should release after enough false cycles");
    }

    @Test
    void currentHoldStepsTowardTargetCurrent() {
        controller.initialize(session, SmartRetractController.Mode.CURRENT_HOLD, true, MID_POSITION, 5.0);

        // Engage feed latch
        for (int i = 0; i < IntakeConstants.SMART_RETRACT_FEED_ENGAGE_CYCLES; i++) {
            controller.update(session, true, MID_POSITION, 5.0, true, false);
        }

        double initialTarget = session.commandedLeftTargetRot();
        // Low current → should step inward (decrease position)
        SmartRetractController.Update update =
                controller.update(session, true, MID_POSITION, 5.0, true, false);
        assertTrue(update.commandRetractTarget());
        assertTrue(
                session.commandedLeftTargetRot() < initialTarget,
                "Should step inward (retract) when below target current");
    }

    @Test
    void currentHoldBacksOffWhenCurrentTooHigh() {
        double highCurrent =
                IntakeConstants.SMART_RETRACT_HOLD_TARGET_CURRENT_AMPS
                        + IntakeConstants.SMART_RETRACT_HOLD_DEADBAND_AMPS
                        + 5.0;

        controller.initialize(
                session, SmartRetractController.Mode.CURRENT_HOLD, true, MID_POSITION, highCurrent);

        // Engage feed latch
        for (int i = 0; i < IntakeConstants.SMART_RETRACT_FEED_ENGAGE_CYCLES; i++) {
            controller.update(session, true, MID_POSITION, highCurrent, true, false);
        }

        double beforeBackoff = session.commandedLeftTargetRot();
        controller.update(session, true, MID_POSITION, highCurrent, true, false);
        assertTrue(
                session.commandedLeftTargetRot() > beforeBackoff,
                "Should back off (extend) when current too high");
    }

    @Test
    void positionClampedToRetractedAndExtended() {
        controller.initialize(
                session, SmartRetractController.Mode.CURRENT_HOLD, true, MID_POSITION, 5.0);

        // Engage feed latch
        for (int i = 0; i < IntakeConstants.SMART_RETRACT_FEED_ENGAGE_CYCLES; i++) {
            controller.update(session, true, MID_POSITION, 5.0, true, false);
        }

        // Repeatedly step inward to hit retracted limit
        for (int i = 0; i < 500; i++) {
            controller.update(session, true, MID_POSITION, 5.0, true, false);
        }
        assertTrue(
                session.commandedLeftTargetRot() >= IntakeConstants.RETRACTED_POSITION_ROT,
                "Position should not go below retracted limit");
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

        // Engage feed latch
        for (int i = 0; i < IntakeConstants.SMART_RETRACT_FEED_ENGAGE_CYCLES; i++) {
            controller.update(session, true, MID_POSITION, 50.0, true, false);
        }

        double filtered = session.filteredSignalCurrentAmps();
        // With alpha=0.20, after several cycles of 50A from 0A baseline, filtered should
        // be less than raw but greater than initial (0)
        assertTrue(filtered > 0.0, "Filtered current should be positive after high-current inputs");
        assertTrue(filtered < 50.0, "Filtered current should lag behind raw current");
    }
}
