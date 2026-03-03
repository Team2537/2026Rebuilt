package frc.robot.coordination.shooting;

/** Constants scoped to shoot coordination/gating policy. */
public final class ShootCoordinatorConstants {
    public enum FeedGateMode {
        IMMEDIATE,
        SHOOTER_AT_SETPOINT,
        SHOOTER_AND_AIM
    }

    public static final FeedGateMode DEFAULT_FEED_GATE_MODE = FeedGateMode.SHOOTER_AND_AIM;
    public static final int GATE_READY_DEBOUNCE_CYCLES = 2;

    private ShootCoordinatorConstants() {}
}
