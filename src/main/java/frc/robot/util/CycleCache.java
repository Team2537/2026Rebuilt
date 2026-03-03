package frc.robot.util;

import java.util.function.Supplier;

/** Caches a computed value for a loop key and recomputes only when the key changes. */
public final class CycleCache<T> {
    private long lastCycleKey = Long.MIN_VALUE;
    private boolean hasValue = false;
    private T cachedValue;

    public T get(long cycleKey, Supplier<T> compute) {
        if (!hasValue || cycleKey != lastCycleKey) {
            lastCycleKey = cycleKey;
            cachedValue = compute.get();
            hasValue = true;
        }
        return cachedValue;
    }

    public void invalidate() {
        hasValue = false;
        lastCycleKey = Long.MIN_VALUE;
        cachedValue = null;
    }
}
