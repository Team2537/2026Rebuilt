package frc.robot.coordination.shooting;

/** Simple feed gate with readiness debounce. */
public final class ShotGate {
    private final ShootCoordinatorConstants.FeedGateMode feedGateMode;
    private int readyStableCycles = 0;
    private int readyDropStableCycles = 0;
    private boolean gateOpen = false;

    public ShotGate(ShootCoordinatorConstants.FeedGateMode feedGateMode) {
        this.feedGateMode = feedGateMode;
    }

    public GateDecision update(
            boolean solutionValid,
            boolean movingShot,
            boolean shooterAtSetpoint,
            boolean aimReady,
            boolean manualFeedOverride,
            boolean automaticFeedEnabled) {
        if (!solutionValid) {
            reset();
            return new GateDecision(false, "NoValidShotSolution");
        }
        if (manualFeedOverride) {
            reset();
            gateOpen = true;
            return new GateDecision(true, "ManualFeedOverride");
        }
        if (!automaticFeedEnabled) {
            reset();
            return new GateDecision(false, "FeedingDisabledOverride");
        }

        if (feedGateMode == ShootCoordinatorConstants.FeedGateMode.IMMEDIATE) {
            readyStableCycles = 0;
            gateOpen = true;
            return new GateDecision(true, "");
        }

        boolean ready = switch (feedGateMode) {
            case IMMEDIATE -> true;
            case SHOOTER_AT_SETPOINT -> shooterAtSetpoint;
            case SHOOTER_AND_AIM -> shooterAtSetpoint && aimReady;
        };
        if (ready) {
            readyStableCycles++;
            readyDropStableCycles = 0;
            gateOpen = readyStableCycles >= ShootCoordinatorConstants.gateReadyDebounceCycles();
            return new GateDecision(gateOpen, gateOpen ? "" : "ReadinessDebounce");
        }

        readyStableCycles = 0;
        if (movingShot && gateOpen) {
            readyDropStableCycles++;
            if (readyDropStableCycles < ShootCoordinatorConstants.movingGateDropDebounceCycles()) {
                return new GateDecision(true, "MovingShotReadinessDropDebounce");
            }
        }

        readyDropStableCycles = 0;
        gateOpen = false;
        if (!shooterAtSetpoint && !aimReady) {
            return new GateDecision(false, "ShooterNotAtSetpoint+AimNotReady");
        }
        if (!shooterAtSetpoint) {
            return new GateDecision(false, "ShooterNotAtSetpoint");
        }
        return new GateDecision(false, "AimNotReady");
    }

    public int getReadyStableCycles() {
        return readyStableCycles;
    }

    public int getReadyDropStableCycles() {
        return readyDropStableCycles;
    }

    public boolean isGateOpen() {
        return gateOpen;
    }

    public void reset() {
        readyStableCycles = 0;
        readyDropStableCycles = 0;
        gateOpen = false;
    }

    public record GateDecision(boolean gateOpen, String blockReason) {}
}
