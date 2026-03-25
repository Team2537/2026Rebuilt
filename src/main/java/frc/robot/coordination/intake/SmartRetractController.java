package frc.robot.coordination.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.intake.IntakeConstants;

/** Stateful smart-retract coordinator with jam-aware retract-first behavior. */
public final class SmartRetractController {
    public enum Phase {
        INACTIVE,
        SEEKING_FLOW,
        FLOWING,
        TAIL_DRAIN,
        OUTER_JAM_RECOVERY,
        INNER_STALL_RECOVERY
    }

    public static final class Session {
        private Phase phase = Phase.INACTIVE;
        private boolean startedExtended = false;
        private boolean active = false;
        private double commandedLeftTargetRot = Double.NaN;
        private double filteredSignalCurrentAmps = 0.0;
        private boolean feedLatched = false;
        private int feedTrueCycles = 0;
        private int feedFalseCycles = 0;
        private boolean fullRetractReached = false;
        private boolean seenAboveSmartRetractThreshold = false;
        private boolean sawShotPulse = false;
        private double jamBackoffCurrentThresholdAmps = 0.0;
        private int jamBackoffDetectCycles = 0;
        private int jamBackoffCurrentCycles = 0;
        private double lastShotOrFeedTimestampSec = Double.NaN;
        private double tailDrainUntilSec = Double.NaN;
        private int jamRecoveryCount = 0;

        public Phase phase() {
            return phase;
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

        public boolean feedLatched() {
            return feedLatched;
        }

        public int feedTrueCycles() {
            return feedTrueCycles;
        }

        public int feedFalseCycles() {
            return feedFalseCycles;
        }

        public boolean fullRetractReached() {
            return fullRetractReached;
        }

        public boolean sawShotPulse() {
            return sawShotPulse;
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
            return phase == Phase.OUTER_JAM_RECOVERY || phase == Phase.INNER_STALL_RECOVERY;
        }

        public int jamRecoveryCount() {
            return jamRecoveryCount;
        }
    }

    public record Inputs(
            boolean activelyFeeding,
            boolean shotPulseDetectedThisCycle,
            double leftPositionRot,
            double rawSignalCurrentAmps) {}

    public record Update(
            boolean spinRoller,
            boolean commandTarget,
            double commandedLeftTargetRot,
            boolean wiggleActive,
            double rawSignalCurrentAmps) {}

    public void initialize(
            Session session,
            boolean startedExtended,
            double initialLeftPositionRot,
            double initialSignalCurrentAmps) {
        session.startedExtended = startedExtended;
        session.active = startedExtended;
        session.phase = session.active ? Phase.SEEKING_FLOW : Phase.INACTIVE;
        session.commandedLeftTargetRot = clampSmartRetractTargetRot(initialLeftPositionRot);
        session.filteredSignalCurrentAmps = initialSignalCurrentAmps;
        session.feedLatched = false;
        session.feedTrueCycles = 0;
        session.feedFalseCycles = 0;
        session.fullRetractReached = false;
        session.sawShotPulse = false;
        session.jamBackoffCurrentCycles = 0;
        session.lastShotOrFeedTimestampSec = Double.NaN;
        session.tailDrainUntilSec = Double.NaN;
        session.jamRecoveryCount = 0;
        session.seenAboveSmartRetractThreshold = initialLeftPositionRot > smartRetractCompletionThresholdRot();
        refreshThresholds(session);
    }

    public Update update(Session session, Inputs inputs) {
        return update(session, inputs, Timer.getFPGATimestamp());
    }

    Update update(Session session, Inputs inputs, double nowSec) {
        boolean shouldSpinRoller = inputs.activelyFeeding();
        if (!session.active) {
            session.phase = Phase.INACTIVE;
            return new Update(
                    shouldSpinRoller,
                    false,
                    session.commandedLeftTargetRot,
                    false,
                    inputs.rawSignalCurrentAmps());
        }

        refreshThresholds(session);
        session.filteredSignalCurrentAmps = filteredCurrent(
                session.filteredSignalCurrentAmps,
                inputs.rawSignalCurrentAmps(),
                IntakeConstants.smartRetractCurrentFilterAlpha());

        boolean feedWindowOpened = updateFeedLatch(session, inputs.activelyFeeding(), nowSec);
        if (feedWindowOpened) {
            session.phase = session.fullRetractReached ? Phase.FLOWING : Phase.SEEKING_FLOW;
        }

        if (inputs.shotPulseDetectedThisCycle()) {
            session.sawShotPulse = true;
            session.lastShotOrFeedTimestampSec = nowSec;
            session.tailDrainUntilSec = Double.NaN;
            session.phase = Phase.FLOWING;
        }

        if (session.phase == Phase.OUTER_JAM_RECOVERY) {
            runOuterJamRecovery(session, inputs.leftPositionRot(), nowSec);
            return buildUpdate(session, shouldSpinRoller, inputs.rawSignalCurrentAmps());
        }
        if (session.phase == Phase.INNER_STALL_RECOVERY) {
            runInnerStallRecovery(session, inputs.leftPositionRot(), nowSec);
            return buildUpdate(session, shouldSpinRoller, inputs.rawSignalCurrentAmps());
        }

        boolean inInnerRegion = inputs.leftPositionRot() <= innerStallArmThresholdRot();
        updateFullRetractReached(session, inputs.leftPositionRot());

        if (!inInnerRegion) {
            updateJamBackoffCurrentCycles(session);
            if (session.jamBackoffCurrentCycles >= session.jamBackoffDetectCycles) {
                startOuterJamRecovery(session);
                return buildUpdate(session, shouldSpinRoller, inputs.rawSignalCurrentAmps());
            }
        } else {
            session.jamBackoffCurrentCycles = 0;
        }

        if (shouldStartInnerStallHandling(session, inputs.activelyFeeding(), inInnerRegion, nowSec)) {
            if (shouldEnterTailDrain(session)) {
                if (session.phase != Phase.TAIL_DRAIN) {
                    session.phase = Phase.TAIL_DRAIN;
                    session.tailDrainUntilSec = nowSec + IntakeConstants.smartRetractTailDrainGraceSec();
                } else if (Double.isFinite(session.tailDrainUntilSec) && nowSec >= session.tailDrainUntilSec) {
                    startInnerStallRecovery(session);
                }
            } else {
                startInnerStallRecovery(session);
            }
            return buildUpdate(session, shouldSpinRoller, inputs.rawSignalCurrentAmps());
        }

        if (session.phase == Phase.TAIL_DRAIN
                && (!inputs.activelyFeeding() || !session.sawShotPulse || !session.fullRetractReached)) {
            session.phase = session.sawShotPulse ? Phase.FLOWING : Phase.SEEKING_FLOW;
            session.tailDrainUntilSec = Double.NaN;
        }

        commandNormalRetract(session);
        return buildUpdate(session, shouldSpinRoller, inputs.rawSignalCurrentAmps());
    }

    public boolean shouldRestoreExtendedOnExit(Session session, boolean disabled, boolean atRetractedTarget) {
        if (!session.active || disabled) {
            return false;
        }
        if (session.fullRetractReached) {
            return false;
        }
        return session.startedExtended && !atRetractedTarget;
    }

    private static Update buildUpdate(Session session, boolean spinRoller, double rawSignalCurrentAmps) {
        return new Update(
                spinRoller,
                true,
                session.commandedLeftTargetRot,
                session.fullRetractReached && !session.jamRecoveryActive(),
                rawSignalCurrentAmps);
    }

    private static void refreshThresholds(Session session) {
        session.jamBackoffCurrentThresholdAmps = IntakeConstants.smartRetractJamBackoffCurrentThresholdAmps();
        session.jamBackoffDetectCycles = IntakeConstants.smartRetractJamBackoffDetectCycles();
    }

    private static boolean updateFeedLatch(Session session, boolean activelyFeeding, double nowSec) {
        boolean wasLatched = session.feedLatched;
        if (activelyFeeding) {
            session.feedTrueCycles++;
            session.feedFalseCycles = 0;
        } else {
            session.feedFalseCycles++;
            session.feedTrueCycles = 0;
            session.feedLatched = false;
            return false;
        }

        if (session.feedTrueCycles >= feedStartDelayCycles()) {
            session.feedLatched = true;
        }
        if (session.feedLatched && !wasLatched) {
            resetShotProgressWindow(session, nowSec, false);
            return true;
        }
        return false;
    }

    private static boolean shouldStartInnerStallHandling(
            Session session,
            boolean activelyFeeding,
            boolean inInnerRegion,
            double nowSec) {
        if (!activelyFeeding || !session.feedLatched || !inInnerRegion || !Double.isFinite(session.lastShotOrFeedTimestampSec)) {
            return false;
        }
        double timeoutSec = session.sawShotPulse
                ? IntakeConstants.smartRetractJamInterShotTimeoutSec()
                : IntakeConstants.smartRetractJamFirstShotTimeoutSec();
        return timeoutSec > 0.0 && nowSec - session.lastShotOrFeedTimestampSec >= timeoutSec;
    }

    private static boolean shouldEnterTailDrain(Session session) {
        return session.sawShotPulse
                && session.fullRetractReached
                && session.filteredSignalCurrentAmps < session.jamBackoffCurrentThresholdAmps;
    }

    private static void startOuterJamRecovery(Session session) {
        startRecovery(
                session,
                Phase.OUTER_JAM_RECOVERY,
                IntakeConstants.smartRetractJamRecoveryExtendPositionRot());
    }

    private static void startInnerStallRecovery(Session session) {
        startRecovery(
                session,
                Phase.INNER_STALL_RECOVERY,
                IntakeConstants.smartRetractInnerStallRecoveryExtendPositionRot());
    }

    private static void runOuterJamRecovery(Session session, double leftPositionRot, double nowSec) {
        runRecovery(session, leftPositionRot, IntakeConstants.smartRetractJamRecoveryExtendPositionRot(), nowSec);
    }

    private static void runInnerStallRecovery(Session session, double leftPositionRot, double nowSec) {
        runRecovery(
                session,
                leftPositionRot,
                IntakeConstants.smartRetractInnerStallRecoveryExtendPositionRot(),
                nowSec);
    }

    private static void startRecovery(Session session, Phase phase, double extendTargetRot) {
        session.phase = phase;
        session.fullRetractReached = false;
        session.jamBackoffCurrentCycles = 0;
        session.tailDrainUntilSec = Double.NaN;
        session.jamRecoveryCount++;
        session.commandedLeftTargetRot = clampSmartRetractTargetRot(extendTargetRot);
    }

    private static void runRecovery(Session session, double leftPositionRot, double extendTargetRot, double nowSec) {
        session.commandedLeftTargetRot = clampSmartRetractTargetRot(extendTargetRot);
        if (leftPositionRot + IntakeConstants.POSITION_TOLERANCE_ROT < extendTargetRot) {
            return;
        }

        session.phase = Phase.SEEKING_FLOW;
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

    private static void updateFullRetractReached(Session session, double leftPositionRot) {
        boolean atSmartRetractTarget = leftPositionRot <= smartRetractCompletionThresholdRot();
        if (!atSmartRetractTarget) {
            session.seenAboveSmartRetractThreshold = true;
        }
        if (atSmartRetractTarget && session.seenAboveSmartRetractThreshold) {
            session.fullRetractReached = true;
        }
    }

    private static void commandNormalRetract(Session session) {
        if (session.fullRetractReached) {
            session.commandedLeftTargetRot = IntakeConstants.smartRetractRetractedPositionRot();
            return;
        }
        session.commandedLeftTargetRot = clampSmartRetractTargetRot(
                session.commandedLeftTargetRot - IntakeConstants.smartRetractStepRot());
    }

    private static int feedStartDelayCycles() {
        int configuredDelayCycles =
                (int) Math.ceil(IntakeConstants.smartRetractFeedStartDelaySec() * IntakeConstants.STATUS_UPDATE_HZ);
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

    private static double innerStallArmThresholdRot() {
        return IntakeConstants.smartRetractInnerStallRecoveryExtendPositionRot();
    }
}
