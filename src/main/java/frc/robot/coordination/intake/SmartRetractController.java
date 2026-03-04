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
        private double baselineSignalCurrentAmps = 0.0;
        private boolean feedLatched = false;
        private int feedTrueCycles = 0;
        private int feedFalseCycles = 0;
        private boolean halfRetractReached = false;

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

        public double baselineSignalCurrentAmps() {
            return baselineSignalCurrentAmps;
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
        session.baselineSignalCurrentAmps = initialSignalCurrentAmps;
        session.nibbleBackoffActive = false;
        session.nibbleSpikeCycles = 0;
        session.nibbleBackoffUntilSec = Double.NaN;
        session.feedLatched = false;
        session.feedTrueCycles = 0;
        session.feedFalseCycles = 0;
        session.halfRetractReached = false;
    }

    public Update update(
            Session session,
            boolean activelyFeeding,
            double leftPositionRot,
            double rawSignalCurrentAmps,
            boolean goalExtended,
            boolean atRetractedTarget) {
        boolean shouldSpinRoller = session.startedExtended && (goalExtended || !atRetractedTarget);
        if (!session.active) {
            return new Update(shouldSpinRoller, false, session.commandedLeftTargetRot, rawSignalCurrentAmps);
        }

        session.filteredSignalCurrentAmps = filteredCurrent(
                session.filteredSignalCurrentAmps,
                rawSignalCurrentAmps,
                IntakeConstants.SMART_RETRACT_CURRENT_FILTER_ALPHA);
        boolean feedLatched = updateFeedLatch(session, activelyFeeding);
        if (!feedLatched) {
            return new Update(shouldSpinRoller, false, session.commandedLeftTargetRot, rawSignalCurrentAmps);
        }

        if (!activelyFeeding) {
            session.commandedLeftTargetRot = clampSmartRetractTargetRot(leftPositionRot);
            return new Update(shouldSpinRoller, true, session.commandedLeftTargetRot, rawSignalCurrentAmps);
        }

        if (session.mode == Mode.NIBBLE && atRetractedTarget) {
            session.commandedLeftTargetRot = IntakeConstants.SMART_RETRACT_RETRACTED_POSITION_ROT;
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

    public boolean shouldRestoreExtendedOnExit(
            Session session,
            boolean disabled,
            boolean atRetractedTarget) {
        if (!session.active || disabled) {
            return false;
        }
        return session.startedExtended && !atRetractedTarget;
    }

    private void runNibbleSmartRetract(Session session, double leftPositionRot) {
        double nowSec = Timer.getFPGATimestamp();
        if (session.nibbleBackoffActive) {
            if (nowSec < session.nibbleBackoffUntilSec) {
                return;
            }
            session.nibbleBackoffActive = false;
            session.nibbleSpikeCycles = 0;
            session.baselineSignalCurrentAmps = session.filteredSignalCurrentAmps;
        }

        double thresholdAmps =
                session.baselineSignalCurrentAmps + IntakeConstants.SMART_RETRACT_NIBBLE_CURRENT_DELTA_AMPS;
        if (session.filteredSignalCurrentAmps >= thresholdAmps) {
            session.nibbleSpikeCycles++;
        } else {
            session.nibbleSpikeCycles = 0;
            session.baselineSignalCurrentAmps = filteredCurrent(
                    session.baselineSignalCurrentAmps,
                    session.filteredSignalCurrentAmps,
                    0.05);
        }

        if (session.nibbleSpikeCycles >= IntakeConstants.SMART_RETRACT_NIBBLE_DETECT_CYCLES) {
            session.commandedLeftTargetRot = clampSmartRetractTargetRot(
                    leftPositionRot + IntakeConstants.SMART_RETRACT_NIBBLE_BACKOFF_ROT);
            session.nibbleBackoffActive = true;
            session.nibbleBackoffUntilSec =
                    nowSec + IntakeConstants.SMART_RETRACT_NIBBLE_BACKOFF_DWELL_SEC;
            session.nibbleSpikeCycles = 0;
            return;
        }

        session.commandedLeftTargetRot = clampSmartRetractTargetRot(
                session.commandedLeftTargetRot - IntakeConstants.SMART_RETRACT_NIBBLE_STEP_ROT);
    }

    private static void runHalfRetractReturnSmartRetract(Session session, double leftPositionRot) {
        if (!session.halfRetractReached) {
            session.commandedLeftTargetRot =
                    clampSmartRetractTargetRot(IntakeConstants.SMART_RETRACT_HALF_RETRACT_POSITION_ROT);
            if (leftPositionRot <= IntakeConstants.SMART_RETRACT_HALF_RETRACT_POSITION_ROT
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
                IntakeConstants.SMART_RETRACT_FEED_START_DELAY_SEC * IntakeConstants.STATUS_UPDATE_HZ);
        return Math.max(IntakeConstants.SMART_RETRACT_FEED_ENGAGE_CYCLES, Math.max(1, configuredDelayCycles));
    }

    private static double filteredCurrent(double previous, double current, double alpha) {
        double clampedAlpha = MathUtil.clamp(alpha, 0.0, 1.0);
        return previous + clampedAlpha * (current - previous);
    }

    private static double clampSmartRetractTargetRot(double targetRot) {
        return MathUtil.clamp(
                targetRot,
                IntakeConstants.SMART_RETRACT_RETRACTED_POSITION_ROT,
                IntakeConstants.EXTENDED_POSITION_ROT);
    }
}
