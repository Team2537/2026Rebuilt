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
        SmartDashboard.putNumber(
                "Intake/SmartRetract/RollerRpm",
                IntakeConstants.SMART_RETRACT_ROLLER_RPM);
        SmartDashboard.putNumber(
                "Intake/SmartRetract/JamCurrentThresholdAmps",
                IntakeConstants.SMART_RETRACT_JAM_CURRENT_THRESHOLD_AMPS);
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
                "Intake/SmartRetract/WiggleOutRot",
                IntakeConstants.SMART_RETRACT_WIGGLE_OUT_ROT);
        SmartDashboard.putNumber(
                "Intake/SmartRetract/WiggleSwitchIntervalSec",
                IntakeConstants.SMART_RETRACT_WIGGLE_SWITCH_INTERVAL_SEC);
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
    void reachingFullSmartRetractLatchesCompletionAndSuppressesRestore() {
        controller.initialize(session, SmartRetractController.Mode.NIBBLE, true, MID_POSITION, 0.0);

        for (int i = 0; i < feedStartDelayCycles(); i++) {
            controller.update(session, true, MID_POSITION, 0.0, true, false);
        }

        SmartRetractController.Update update = controller.update(
                session,
                true,
                IntakeConstants.smartRetractRetractedPositionRot(),
                0.0,
                false,
                false);

        assertTrue(session.fullRetractReached(), "Expected full smart retract to latch once the inward target is reached.");
        assertEquals(
                IntakeConstants.smartRetractRetractedPositionRot(),
                update.commandedLeftTargetRot(),
                1e-9,
                "Once full smart retract is reached, the controller should park its base target at the retract baseline.");
        assertFalse(
                controller.shouldRestoreExtendedOnExit(session, false, false),
                "A session that already completed full smart retract must not restore extension on exit.");
    }

    @Test
    void regularRetractedPositionDoesNotCountAsFullSmartRetractCompletion() {
        controller.initialize(session, SmartRetractController.Mode.NIBBLE, true, MID_POSITION, 0.0);

        for (int i = 0; i < feedStartDelayCycles(); i++) {
            controller.update(session, true, MID_POSITION, 0.0, true, false);
        }

        controller.update(
                session,
                true,
                IntakeConstants.RETRACTED_POSITION_ROT,
                0.0,
                false,
                true);

        assertFalse(
                session.fullRetractReached(),
                "Crossing only the normal retract target must not be treated as completing smart retract.");
    }

    @Test
    void startingInsideSmartRetractThresholdDoesNotLatchCompletionWithoutCrossingIntoIt() {
        double smartRetractTargetRot = IntakeConstants.smartRetractRetractedPositionRot();
        controller.initialize(
                session,
                SmartRetractController.Mode.NIBBLE,
                true,
                smartRetractTargetRot,
                0.0);

        for (int i = 0; i < feedStartDelayCycles(); i++) {
            controller.update(session, true, smartRetractTargetRot, 0.0, true, false);
        }

        SmartRetractController.Update holdUpdate = controller.update(
                session,
                true,
                smartRetractTargetRot,
                0.0,
                true,
                false);

        assertFalse(
                session.fullRetractReached(),
                "Starting a session already inside the smart-retract threshold should not count as completing full retract.");
        assertTrue(
                controller.shouldRestoreExtendedOnExit(session, false, false),
                "If this session never crossed into the smart-retract threshold from above, it should still restore extension on exit.");
        assertEquals(
                smartRetractTargetRot,
                holdUpdate.commandedLeftTargetRot(),
                1e-9,
                "Without crossing the threshold from above, the controller should hold the baseline target instead of latching completion.");
    }

    @Test
    void currentFilteringAppliesExponentialSmoothing() {
        controller.initialize(session, SmartRetractController.Mode.NIBBLE, true, MID_POSITION, 0.0);

        for (int i = 0; i < feedStartDelayCycles(); i++) {
            controller.update(session, true, true, MID_POSITION, 50.0, true, false);
        }

        double filtered = session.filteredSignalCurrentAmps();
        assertTrue(filtered > 0.0, "Filtered current should be positive after high-current inputs");
        assertTrue(filtered < 50.0, "Filtered current should lag behind raw current");
    }

    @Test
    void jamRecoveryTriggersWhenNoFirstShotPulseArrives() {
        SmartDashboard.putNumber("Intake/SmartRetract/JamCurrentThresholdAmps", 3.0);
        SmartDashboard.putNumber("Intake/SmartRetract/JamBackoffCurrentThresholdAmps", 100.0);
        SmartDashboard.putNumber("Intake/SmartRetract/JamFirstShotTimeoutSec", 0.04);
        SmartDashboard.putNumber("Intake/SmartRetract/JamRecoveryExtendPositionRot", IntakeConstants.EXTENDED_POSITION_ROT);

        controller.initialize(session, SmartRetractController.Mode.NIBBLE, true, MID_POSITION, 20.0);

        for (int i = 0; i < feedStartDelayCycles(); i++) {
            controller.update(session, true, false, MID_POSITION, 20.0, true, false);
            sleepMs(20);
        }

        controller.update(session, true, false, MID_POSITION, 20.0, true, false);
        sleepMs(20);
        SmartRetractController.Update jamUpdate =
                controller.update(session, true, false, MID_POSITION, 20.0, true, false);

        assertTrue(session.jamRecoveryActive(), "Missing the first shot pulse should trigger jam recovery.");
        assertEquals(1, session.jamRecoveryCount(), "Jam recovery should increment its recovery counter.");
        assertTrue(session.jamDetectionCurrentMet(), "Jam recovery should only arm once the current threshold is met.");
        assertEquals(
                IntakeConstants.EXTENDED_POSITION_ROT,
                jamUpdate.commandedLeftTargetRot(),
                1e-9,
                "Jam recovery should command the configured extend position before restarting retract.");
    }

    @Test
    void shotPulsePreventsJamRecoveryUntilInterShotTimeoutExpires() {
        SmartDashboard.putNumber("Intake/SmartRetract/JamCurrentThresholdAmps", 3.0);
        SmartDashboard.putNumber("Intake/SmartRetract/JamBackoffCurrentThresholdAmps", 100.0);
        SmartDashboard.putNumber("Intake/SmartRetract/JamFirstShotTimeoutSec", 0.04);
        SmartDashboard.putNumber("Intake/SmartRetract/JamInterShotTimeoutSec", 0.04);

        controller.initialize(session, SmartRetractController.Mode.NIBBLE, true, MID_POSITION, 20.0);

        for (int i = 0; i < feedStartDelayCycles(); i++) {
            controller.update(session, true, false, MID_POSITION, 20.0, true, false);
            sleepMs(20);
        }

        controller.update(session, true, true, MID_POSITION, 20.0, true, false);
        sleepMs(20);
        controller.update(session, true, false, MID_POSITION, 20.0, true, false);
        assertFalse(session.jamRecoveryActive(), "A recent shot pulse should reset the jam timer.");

        sleepMs(50);
        controller.update(session, true, false, MID_POSITION, 20.0, true, false);
        assertTrue(session.jamRecoveryActive(), "After the inter-shot timeout expires, jam recovery should trigger.");
    }

    @Test
    void lowCurrentDoesNotTriggerJamRecoveryWhenIntakeMayBeEmpty() {
        SmartDashboard.putNumber("Intake/SmartRetract/JamCurrentThresholdAmps", 3.0);
        SmartDashboard.putNumber("Intake/SmartRetract/JamBackoffCurrentThresholdAmps", 2.5);
        SmartDashboard.putNumber("Intake/SmartRetract/JamFirstShotTimeoutSec", 0.04);

        controller.initialize(session, SmartRetractController.Mode.NIBBLE, true, MID_POSITION, 0.0);

        for (int i = 0; i < feedStartDelayCycles(); i++) {
            controller.update(session, true, false, MID_POSITION, 0.5, true, false);
            sleepMs(20);
        }

        sleepMs(60);
        controller.update(session, true, false, MID_POSITION, 0.5, true, false);

        assertFalse(session.jamDetectionCurrentMet(), "Low current should leave jam detection disarmed.");
        assertFalse(session.jamRecoveryActive(), "Low-current empty-intake feeding should not trigger jam recovery.");
    }

    @Test
    void sustainedBackoffCurrentTriggersJamRecoveryWithoutWaitingForShotTimeout() {
        SmartDashboard.putNumber("Intake/SmartRetract/JamBackoffCurrentThresholdAmps", 2.5);
        SmartDashboard.putNumber("Intake/SmartRetract/JamBackoffDetectCycles", 2.0);
        SmartDashboard.putNumber("Intake/SmartRetract/JamFirstShotTimeoutSec", 10.0);

        controller.initialize(session, SmartRetractController.Mode.NIBBLE, true, MID_POSITION, 0.0);

        for (int i = 0; i < feedStartDelayCycles(); i++) {
            controller.update(session, true, false, MID_POSITION, 0.0, true, false);
        }

        controller.update(session, true, false, MID_POSITION, 20.0, true, false);
        assertEquals(1, session.jamBackoffCurrentCycles(), "First overcurrent cycle should arm the backoff counter.");

        SmartRetractController.Update jamUpdate =
                controller.update(session, true, false, MID_POSITION, 20.0, true, false);
        assertTrue(session.jamRecoveryActive(), "Sustained overcurrent should trigger jam recovery immediately.");
        assertEquals(0, session.jamBackoffCurrentCycles(), "Jam recovery should clear the overcurrent counter.");
        assertEquals(
                IntakeConstants.smartRetractJamRecoveryExtendPositionRot(),
                jamUpdate.commandedLeftTargetRot(),
                1e-9,
                "Current-based backoff should use the jam recovery extend target.");
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
        SmartDashboard.putNumber("Intake/SmartRetract/JamCurrentThresholdAmps", 6.0);
        SmartDashboard.putNumber("Intake/SmartRetract/JamBackoffCurrentThresholdAmps", 2.5);
        SmartDashboard.putNumber("Intake/SmartRetract/JamBackoffDetectCycles", 4.0);
        SmartDashboard.putNumber("Intake/SmartRetract/JamFirstShotTimeoutSec", 0.04);
        SmartDashboard.putNumber("Intake/SmartRetract/JamInterShotTimeoutSec", 0.06);
        SmartDashboard.putNumber("Intake/SmartRetract/JamRecoveryExtendPositionRot", 14.0);

        controller.initialize(session, SmartRetractController.Mode.NIBBLE, true, 0.0, 0.0);
        assertEquals(8.5, session.nibbleCurrentThresholdAmps(), 1e-9);
        assertEquals(6.0, session.jamCurrentThresholdAmps(), 1e-9);
        assertEquals(2.5, session.jamBackoffCurrentThresholdAmps(), 1e-9);
        assertEquals(4, session.jamBackoffDetectCycles());
        assertEquals(3.5, session.commandedLeftTargetRot(), 1e-9);

        controller.initialize(session, SmartRetractController.Mode.NIBBLE, true, MID_POSITION, 0.0);
        for (int i = 0; i < 4; i++) {
            SmartRetractController.Update preLatchUpdate =
                    controller.update(session, true, false, MID_POSITION, 50.0, true, false);
            assertFalse(preLatchUpdate.commandRetractTarget(), "Feed engage cycles tuning should delay latching.");
            sleepMs(20);
        }

        SmartRetractController.Update stepUpdate =
                controller.update(session, true, false, MID_POSITION, 0.0, true, false);
        assertEquals(MID_POSITION - 1.25, stepUpdate.commandedLeftTargetRot(), 1e-9);
        assertEquals(0.0, session.filteredSignalCurrentAmps(), 1e-9);

        sleepMs(20);
        controller.update(session, true, false, MID_POSITION, 50.0, true, false);
        assertFalse(session.jamRecoveryActive(), "Jam recovery should wait until the tuned timeout expires.");

        sleepMs(30);
        SmartRetractController.Update jamUpdate =
                controller.update(session, true, false, MID_POSITION, 50.0, true, false);
        assertTrue(session.jamRecoveryActive(), "Jam recovery should trigger once the tuned timeout expires.");
        assertEquals(14.0, jamUpdate.commandedLeftTargetRot(), 1e-9);

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

    private static void sleepMs(long millis) {
        try {
            Thread.sleep(millis);
        } catch (InterruptedException ex) {
            Thread.currentThread().interrupt();
            throw new RuntimeException(ex);
        }
    }
}
