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
    }

    public Update update(
            Session session,
            boolean activelyFeeding,
            double leftPositionRot,
            double rawSignalCurrentAmps,
            boolean goalExtended,
            boolean atRetractedTarget) {
        session.nibbleCurrentThresholdAmps = IntakeConstants.smartRetractNibbleCurrentThresholdAmps();
        boolean shouldSpinRoller = session.startedExtended && (goalExtended || !atRetractedTarget);
        if (!session.active) {
            return new Update(shouldSpinRoller, false, session.commandedLeftTargetRot, rawSignalCurrentAmps);
        }

        session.filteredSignalCurrentAmps = filteredCurrent(
                session.filteredSignalCurrentAmps,
                rawSignalCurrentAmps,
                IntakeConstants.smartRetractCurrentFilterAlpha());
        boolean feedLatched = updateFeedLatch(session, activelyFeeding);
        if (!feedLatched) {
            return new Update(shouldSpinRoller, false, session.commandedLeftTargetRot, rawSignalCurrentAmps);
        }

        if (!activelyFeeding) {
            session.commandedLeftTargetRot = clampSmartRetractTargetRot(leftPositionRot);
            return new Update(shouldSpinRoller, true, session.commandedLeftTargetRot, rawSignalCurrentAmps);
        }

        boolean atSmartRetractTarget = leftPositionRot
                <= IntakeConstants.smartRetractRetractedPositionRot() + IntakeConstants.POSITION_TOLERANCE_ROT;

        if (session.mode == Mode.NIBBLE && atSmartRetractTarget) {
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

    private void runNibbleSmartRetract(Session session, double leftPositionRot) {
        double nowSec = Timer.getFPGATimestamp();
        if (session.nibbleBackoffActive) {
            if (nowSec < session.nibbleBackoffUntilSec) {
                return;
            }
            session.nibbleBackoffActive = false;
            session.nibbleSpikeCycles = 0;
        }

        if (session.filteredSignalCurrentAmps >= session.nibbleCurrentThresholdAmps) {
            session.nibbleSpikeCycles++;
        } else {
            session.nibbleSpikeCycles = 0;
        }

        if (session.nibbleSpikeCycles >= IntakeConstants.smartRetractNibbleDetectCycles()) {
            session.commandedLeftTargetRot = clampSmartRetractTargetRot(
                    leftPositionRot + IntakeConstants.smartRetractNibbleBackoffRot());
            session.nibbleBackoffActive = true;
            session.nibbleBackoffUntilSec =
                    nowSec + IntakeConstants.smartRetractNibbleBackoffDwellSec();
            session.nibbleSpikeCycles = 0;
            return;
        }

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
}
