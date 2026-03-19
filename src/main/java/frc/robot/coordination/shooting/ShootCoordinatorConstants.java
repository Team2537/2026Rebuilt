package frc.robot.coordination.shooting;

import frc.robot.util.LoggedTunableNumber;

/** Constants scoped to shoot coordination/gating policy. */
public final class ShootCoordinatorConstants {
    public enum FeedGateMode {
        IMMEDIATE,
        SHOOTER_AT_SETPOINT,
        SHOOTER_AND_AIM
    }

    public static final FeedGateMode DEFAULT_FEED_GATE_MODE = FeedGateMode.SHOOTER_AND_AIM;
    private static final LoggedTunableNumber gateReadyDebounceCycles =
            new LoggedTunableNumber("Shooting/GateReadyDebounceCycles", 4);
    private static final LoggedTunableNumber movingGateDropDebounceCycles =
            new LoggedTunableNumber("Shooting/MovingGateDropDebounceCycles", 2);

    public static int gateReadyDebounceCycles() {
        return Math.max(1, (int) Math.round(gateReadyDebounceCycles.get()));
    }

    public static int movingGateDropDebounceCycles() {
        return Math.max(1, (int) Math.round(movingGateDropDebounceCycles.get()));
    }

    private ShootCoordinatorConstants() {}
}
