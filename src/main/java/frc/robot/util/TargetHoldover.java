package frc.robot.util;

import edu.wpi.first.wpilibj.Timer;
import java.util.function.DoubleSupplier;

/**
 * Holds the most recent non-null target for a bounded window when new samples are missing.
 */
public final class TargetHoldover<T> {
    private final double holdWindowSec;
    private final DoubleSupplier nowSecSupplier;
    private T lastValue;
    private double lastValueTimestampSec = Double.NaN;

    public TargetHoldover(double holdWindowSec) {
        this(holdWindowSec, Timer::getFPGATimestamp);
    }

    public TargetHoldover(double holdWindowSec, DoubleSupplier nowSecSupplier) {
        this.holdWindowSec = holdWindowSec;
        this.nowSecSupplier = nowSecSupplier;
    }

    public HoldResult<T> apply(T target) {
        double nowSec = nowSecSupplier.getAsDouble();
        if (target != null) {
            lastValue = target;
            lastValueTimestampSec = nowSec;
            return new HoldResult<>(target, false, 0.0);
        }

        if (lastValue == null || !Double.isFinite(lastValueTimestampSec)) {
            return new HoldResult<>(null, false, Double.NaN);
        }

        double ageSec = nowSec - lastValueTimestampSec;
        if (!Double.isFinite(ageSec) || ageSec < 0.0 || ageSec > holdWindowSec) {
            return new HoldResult<>(null, false, ageSec);
        }
        return new HoldResult<>(lastValue, true, ageSec);
    }

    public void reset() {
        lastValue = null;
        lastValueTimestampSec = Double.NaN;
    }

    public void clear() {
        reset();
    }

    public record HoldResult<T>(T value, boolean held, double ageSec) {}
}
