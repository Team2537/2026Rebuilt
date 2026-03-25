package frc.robot.coordination.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.intake.IntakeConstants;

/** Stateful smart-retract controller extracted from Intake for reuse and testing. */
public final class SmartRetractController {
    public enum Mode {
        DISABLED,
        NIBBLE,
        HALF_RETRACT_RETURN
    }

    public static final class Session {
        private Mode mode = Mode.DISABLED;
        private boolean startedExtended = false;
        private boolean active = false;
        private boolean nibbleBackoffActive = false;
        private int nibbleSpikeCycles = 0;
        private double nibbleBackoffUntilSec = Double.NaN;
        private double commandedLeftTargetRot = Double.NaN;
        private double filteredSignalCurrentAmps = 0.0;
        private double nibbleCurrentThresholdAmps = 0.0;
        private boolean feedLatched = false;
        private int feedTrueCycles = 0;
        private int feedFalseCycles = 0;
        private boolean halfRetractReached = false;
        private boolean fullRetractReached = false;
        private boolean seenAboveSmartRetractThreshold = false;
        private boolean sawShotPulse = false;
        private double jamCurrentThresholdAmps = 0.0;
        private boolean jamDetectionCurrentMet = false;
        private double jamBackoffCurrentThresholdAmps = 0.0;
        private int jamBackoffDetectCycles = 0;
        private int jamBackoffCurrentCycles = 0;
        private double lastShotOrFeedTimestampSec = Double.NaN;
        private int jamRecoveryCount = 0;

        public Mode mode() {
            return mode;
        }

        public boolean active() {
            return active;
        }

        public double commandedLeftTargetRot() {
            return commandedLeftTargetRot;
        }

        public double filteredSignalCurrentAmps() {
            return filteredSignalCurrentAmps;
        }

        public double nibbleCurrentThresholdAmps() {
            return nibbleCurrentThresholdAmps;
        }

        public boolean feedLatched() {
            return feedLatched;
        }

        public int feedTrueCycles() {
            return feedTrueCycles;
        }

        public int feedFalseCycles() {
            return feedFalseCycles;
        }

        public int nibbleSpikeCycles() {
            return nibbleSpikeCycles;
        }

        public boolean nibbleBackoffActive() {
            return nibbleBackoffActive;
        }

        public boolean fullRetractReached() {
            return fullRetractReached;
        }

        public boolean sawShotPulse() {
            return sawShotPulse;
        }

        public double jamCurrentThresholdAmps() {
            return jamCurrentThresholdAmps;
        }

        public boolean jamDetectionCurrentMet() {
            return jamDetectionCurrentMet;
        }

        public double jamBackoffCurrentThresholdAmps() {
            return jamBackoffCurrentThresholdAmps;
        }

        public int jamBackoffDetectCycles() {
            return jamBackoffDetectCycles;
        }

        public int jamBackoffCurrentCycles() {
            return jamBackoffCurrentCycles;
        }

        public double lastShotOrFeedTimestampSec() {
            return lastShotOrFeedTimestampSec;
        }

        public boolean jamRecoveryActive() {
            return nibbleBackoffActive;
        }

        public int jamRecoveryCount() {
            return jamRecoveryCount;
        }
    }

    public record Update(
            boolean spinRoller,
            boolean commandRetractTarget,
            double commandedLeftTargetRot,
            double rawSignalCurrentAmps) {}

    public void initialize(
            Session session,
            Mode mode,
            boolean startedExtended,
            double initialLeftPositionRot,
            double initialSignalCurrentAmps) {
        session.mode = mode;
        session.startedExtended = startedExtended;
        session.active = session.startedExtended && session.mode != Mode.DISABLED;
        session.commandedLeftTargetRot = clampSmartRetractTargetRot(initialLeftPositionRot);
        session.filteredSignalCurrentAmps = initialSignalCurrentAmps;
        session.nibbleCurrentThresholdAmps = IntakeConstants.smartRetractNibbleCurrentThresholdAmps();
        session.nibbleBackoffActive = false;
        session.nibbleSpikeCycles = 0;
        session.nibbleBackoffUntilSec = Double.NaN;
        session.feedLatched = false;
        session.feedTrueCycles = 0;
        session.feedFalseCycles = 0;
        session.halfRetractReached = false;
        session.fullRetractReached = false;
        session.sawShotPulse = false;
        session.jamCurrentThresholdAmps = IntakeConstants.smartRetractJamCurrentThresholdAmps();
        session.jamDetectionCurrentMet = false;
        session.jamBackoffCurrentThresholdAmps = IntakeConstants.smartRetractJamBackoffCurrentThresholdAmps();
        session.jamBackoffDetectCycles = IntakeConstants.smartRetractJamBackoffDetectCycles();
        session.jamBackoffCurrentCycles = 0;
        session.lastShotOrFeedTimestampSec = Double.NaN;
        session.jamRecoveryCount = 0;
        session.seenAboveSmartRetractThreshold =
                initialLeftPositionRot > smartRetractCompletionThresholdRot();
    }

    public Update update(
            Session session,
            boolean activelyFeeding,
            boolean shotPulseDetectedThisCycle,
            double leftPositionRot,
            double rawSignalCurrentAmps,
            boolean goalExtended,
            boolean atRetractedTarget) {
        double nowSec = Timer.getFPGATimestamp();
        session.nibbleCurrentThresholdAmps = IntakeConstants.smartRetractNibbleCurrentThresholdAmps();
        session.jamCurrentThresholdAmps = IntakeConstants.smartRetractJamCurrentThresholdAmps();
        session.jamBackoffCurrentThresholdAmps = IntakeConstants.smartRetractJamBackoffCurrentThresholdAmps();
        session.jamBackoffDetectCycles = IntakeConstants.smartRetractJamBackoffDetectCycles();
        boolean shouldSpinRoller = activelyFeeding;
        if (!session.active) {
            return new Update(shouldSpinRoller, false, session.commandedLeftTargetRot, rawSignalCurrentAmps);
        }

        session.filteredSignalCurrentAmps = filteredCurrent(
                session.filteredSignalCurrentAmps,
                rawSignalCurrentAmps,
                IntakeConstants.smartRetractCurrentFilterAlpha());
        session.jamDetectionCurrentMet = session.filteredSignalCurrentAmps >= session.jamCurrentThresholdAmps;
        boolean wasFeedLatched = session.feedLatched;
        boolean feedLatched = updateFeedLatch(session, activelyFeeding);
        if (feedLatched && !wasFeedLatched) {
            resetShotProgressWindow(session, nowSec, false);
        }
        if (!feedLatched) {
            session.jamBackoffCurrentCycles = 0;
            return new Update(shouldSpinRoller, false, session.commandedLeftTargetRot, rawSignalCurrentAmps);
        }

        if (!activelyFeeding) {
            session.commandedLeftTargetRot = clampSmartRetractTargetRot(leftPositionRot);
            session.lastShotOrFeedTimestampSec = nowSec;
            session.jamBackoffCurrentCycles = 0;
            return new Update(shouldSpinRoller, true, session.commandedLeftTargetRot, rawSignalCurrentAmps);
        }

        if (shotPulseDetectedThisCycle) {
            session.sawShotPulse = true;
            session.lastShotOrFeedTimestampSec = nowSec;
        }

        if (session.mode == Mode.NIBBLE && session.nibbleBackoffActive) {
            runJamRecovery(session, leftPositionRot, nowSec);
            return new Update(shouldSpinRoller, true, session.commandedLeftTargetRot, rawSignalCurrentAmps);
        }

        updateJamBackoffCurrentCycles(session);
        if (session.mode == Mode.NIBBLE
                && session.jamBackoffCurrentCycles >= session.jamBackoffDetectCycles) {
            startJamRecovery(session);
            return new Update(shouldSpinRoller, true, session.commandedLeftTargetRot, rawSignalCurrentAmps);
        }

        if (session.mode == Mode.NIBBLE && !session.jamDetectionCurrentMet) {
            resetShotProgressWindow(session, nowSec, false);
        }

        if (session.mode == Mode.NIBBLE && shouldStartJamRecovery(session, nowSec)) {
            startJamRecovery(session);
            return new Update(shouldSpinRoller, true, session.commandedLeftTargetRot, rawSignalCurrentAmps);
        }

        boolean atSmartRetractTarget = leftPositionRot
                <= smartRetractCompletionThresholdRot();

        if (!atSmartRetractTarget) {
            session.seenAboveSmartRetractThreshold = true;
        }

        if (session.mode == Mode.NIBBLE
                && atSmartRetractTarget
                && session.seenAboveSmartRetractThreshold) {
            session.fullRetractReached = true;
            session.commandedLeftTargetRot = IntakeConstants.smartRetractRetractedPositionRot();
            return new Update(shouldSpinRoller, true, session.commandedLeftTargetRot, rawSignalCurrentAmps);
        }

        switch (session.mode) {
            case NIBBLE -> runNibbleSmartRetract(session, leftPositionRot);
            case HALF_RETRACT_RETURN -> runHalfRetractReturnSmartRetract(session, leftPositionRot);
            case DISABLED -> {
                // Session is marked active only for non-disabled modes.
            }
        }

        return new Update(shouldSpinRoller, true, session.commandedLeftTargetRot, rawSignalCurrentAmps);
    }

    public Update update(
            Session session,
            boolean activelyFeeding,
            double leftPositionRot,
            double rawSignalCurrentAmps,
            boolean goalExtended,
            boolean atRetractedTarget) {
        return update(
                session,
                activelyFeeding,
                true,
                leftPositionRot,
                rawSignalCurrentAmps,
                goalExtended,
                atRetractedTarget);
    }

    public boolean shouldRestoreExtendedOnExit(
            Session session,
            boolean disabled,
            boolean atRetractedTarget) {
        if (!session.active || disabled) {
            return false;
        }
        if (session.fullRetractReached) {
            return false;
        }
        return session.startedExtended && !atRetractedTarget;
    }

    private static boolean shouldStartJamRecovery(Session session, double nowSec) {
        if (!session.jamDetectionCurrentMet || !Double.isFinite(session.lastShotOrFeedTimestampSec)) {
            return false;
        }
        double timeoutSec = session.sawShotPulse
                ? IntakeConstants.smartRetractJamInterShotTimeoutSec()
                : IntakeConstants.smartRetractJamFirstShotTimeoutSec();
        return timeoutSec > 0.0 && nowSec - session.lastShotOrFeedTimestampSec >= timeoutSec;
    }

    private static void startJamRecovery(Session session) {
        session.nibbleBackoffActive = true;
        session.nibbleSpikeCycles = 0;
        session.nibbleBackoffUntilSec = Double.NaN;
        session.fullRetractReached = false;
        session.jamBackoffCurrentCycles = 0;
        session.jamRecoveryCount++;
        session.commandedLeftTargetRot = clampSmartRetractTargetRot(
                IntakeConstants.smartRetractJamRecoveryExtendPositionRot());
    }

    private static void runJamRecovery(Session session, double leftPositionRot, double nowSec) {
        double extendTargetRot = IntakeConstants.smartRetractJamRecoveryExtendPositionRot();
        session.commandedLeftTargetRot = clampSmartRetractTargetRot(extendTargetRot);
        if (leftPositionRot + IntakeConstants.POSITION_TOLERANCE_ROT < extendTargetRot) {
            return;
        }

        session.nibbleBackoffActive = false;
        session.seenAboveSmartRetractThreshold = true;
        resetShotProgressWindow(session, nowSec, false);
    }

    private static void resetShotProgressWindow(Session session, double nowSec, boolean sawShotPulse) {
        session.sawShotPulse = sawShotPulse;
        session.lastShotOrFeedTimestampSec = nowSec;
    }

    private static void updateJamBackoffCurrentCycles(Session session) {
        if (session.filteredSignalCurrentAmps >= session.jamBackoffCurrentThresholdAmps) {
            session.jamBackoffCurrentCycles++;
            return;
        }
        session.jamBackoffCurrentCycles = 0;
    }

    private void runNibbleSmartRetract(Session session, double leftPositionRot) {
        session.commandedLeftTargetRot = clampSmartRetractTargetRot(
                session.commandedLeftTargetRot - IntakeConstants.smartRetractNibbleStepRot());
    }

    private static void runHalfRetractReturnSmartRetract(Session session, double leftPositionRot) {
        double halfRetractTargetRot = IntakeConstants.smartRetractHalfRetractPositionRot();
        if (!session.halfRetractReached) {
            session.commandedLeftTargetRot = clampSmartRetractTargetRot(halfRetractTargetRot);
            if (leftPositionRot <= halfRetractTargetRot
                    + IntakeConstants.POSITION_TOLERANCE_ROT) {
                session.halfRetractReached = true;
            }
            return;
        }

        session.commandedLeftTargetRot = IntakeConstants.EXTENDED_POSITION_ROT;
    }

    private static boolean updateFeedLatch(Session session, boolean activelyFeeding) {
        if (activelyFeeding) {
            session.feedTrueCycles++;
            session.feedFalseCycles = 0;
        } else {
            session.feedFalseCycles++;
        }

        if (!session.feedLatched && activelyFeeding
                && session.feedTrueCycles >= feedStartDelayCycles()) {
            session.feedLatched = true;
        } else if (!session.feedLatched && !activelyFeeding) {
            session.feedTrueCycles = 0;
        }
        return session.feedLatched;
    }

    private static int feedStartDelayCycles() {
        int configuredDelayCycles = (int) Math.ceil(
                IntakeConstants.smartRetractFeedStartDelaySec() * IntakeConstants.STATUS_UPDATE_HZ);
        return Math.max(IntakeConstants.smartRetractFeedEngageCycles(), Math.max(1, configuredDelayCycles));
    }

    private static double filteredCurrent(double previous, double current, double alpha) {
        double clampedAlpha = MathUtil.clamp(alpha, 0.0, 1.0);
        return previous + clampedAlpha * (current - previous);
    }

    private static double clampSmartRetractTargetRot(double targetRot) {
        return MathUtil.clamp(
                targetRot,
                IntakeConstants.smartRetractRetractedPositionRot(),
                IntakeConstants.EXTENDED_POSITION_ROT);
    }

    private static double smartRetractCompletionThresholdRot() {
        return IntakeConstants.smartRetractRetractedPositionRot() + IntakeConstants.POSITION_TOLERANCE_ROT;
    }
}
