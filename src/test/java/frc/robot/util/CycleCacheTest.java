package frc.robot.util;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNull;

import java.util.concurrent.atomic.AtomicInteger;
import org.junit.jupiter.api.Test;

class CycleCacheTest {
    @Test
    void firstCallComputes() {
        CycleCache<String> cache = new CycleCache<>();
        AtomicInteger calls = new AtomicInteger();
        String result = cache.get(1L, () -> {
            calls.incrementAndGet();
            return "hello";
        });
        assertEquals("hello", result);
        assertEquals(1, calls.get());
    }

    @Test
    void sameKeyReturnsCachedValue() {
        CycleCache<String> cache = new CycleCache<>();
        AtomicInteger calls = new AtomicInteger();
        cache.get(1L, () -> {
            calls.incrementAndGet();
            return "first";
        });
        String result = cache.get(1L, () -> {
            calls.incrementAndGet();
            return "second";
        });
        assertEquals("first", result);
        assertEquals(1, calls.get());
    }

    @Test
    void differentKeyRecomputes() {
        CycleCache<String> cache = new CycleCache<>();
        AtomicInteger calls = new AtomicInteger();
        cache.get(1L, () -> {
            calls.incrementAndGet();
            return "first";
        });
        String result = cache.get(2L, () -> {
            calls.incrementAndGet();
            return "second";
        });
        assertEquals("second", result);
        assertEquals(2, calls.get());
    }

    @Test
    void nullValueIsCached() {
        CycleCache<String> cache = new CycleCache<>();
        AtomicInteger calls = new AtomicInteger();
        cache.get(1L, () -> {
            calls.incrementAndGet();
            return null;
        });
        String result = cache.get(1L, () -> {
            calls.incrementAndGet();
            return "should not compute";
        });
        assertNull(result);
        assertEquals(1, calls.get());
    }

    @Test
    void invalidateForcesRecomputation() {
        CycleCache<String> cache = new CycleCache<>();
        AtomicInteger calls = new AtomicInteger();
        cache.get(1L, () -> {
            calls.incrementAndGet();
            return "first";
        });
        cache.invalidate();
        String result = cache.get(1L, () -> {
            calls.incrementAndGet();
            return "recomputed";
        });
        assertEquals("recomputed", result);
        assertEquals(2, calls.get());
    }
}
