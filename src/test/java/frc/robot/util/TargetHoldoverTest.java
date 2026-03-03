package frc.robot.util;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

class TargetHoldoverTest {
    private static final double EPSILON = 1e-9;

    @Test
    void freshNonNullTargetReturnsImmediately() {
        double[] time = {0.0};
        TargetHoldover<String> holdover = new TargetHoldover<>(1.0, () -> time[0]);

        TargetHoldover.HoldResult<String> result = holdover.apply("target");
        assertEquals("target", result.value());
        assertFalse(result.held());
        assertEquals(0.0, result.ageSec(), EPSILON);
    }

    @Test
    void nullWithinHoldWindowReturnsLastValue() {
        double[] time = {0.0};
        TargetHoldover<String> holdover = new TargetHoldover<>(1.0, () -> time[0]);

        holdover.apply("target");
        time[0] = 0.5;
        TargetHoldover.HoldResult<String> result = holdover.apply(null);

        assertEquals("target", result.value());
        assertTrue(result.held());
        assertEquals(0.5, result.ageSec(), EPSILON);
    }

    @Test
    void nullAfterHoldWindowReturnsNull() {
        double[] time = {0.0};
        TargetHoldover<String> holdover = new TargetHoldover<>(1.0, () -> time[0]);

        holdover.apply("target");
        time[0] = 1.5;
        TargetHoldover.HoldResult<String> result = holdover.apply(null);

        assertNull(result.value());
        assertFalse(result.held());
    }

    @Test
    void sequentialNonNullInputsUpdateHeldValue() {
        double[] time = {0.0};
        TargetHoldover<String> holdover = new TargetHoldover<>(1.0, () -> time[0]);

        holdover.apply("first");
        time[0] = 0.1;
        holdover.apply("second");
        time[0] = 0.3;
        TargetHoldover.HoldResult<String> result = holdover.apply(null);

        assertEquals("second", result.value());
        assertTrue(result.held());
        assertEquals(0.2, result.ageSec(), EPSILON);
    }

    @Test
    void resetClearsHeldState() {
        double[] time = {0.0};
        TargetHoldover<String> holdover = new TargetHoldover<>(1.0, () -> time[0]);

        holdover.apply("target");
        holdover.reset();
        time[0] = 0.1;
        TargetHoldover.HoldResult<String> result = holdover.apply(null);

        assertNull(result.value());
        assertFalse(result.held());
    }

    @Test
    void clearAliasesReset() {
        double[] time = {0.0};
        TargetHoldover<String> holdover = new TargetHoldover<>(1.0, () -> time[0]);

        holdover.apply("target");
        holdover.clear();
        time[0] = 0.1;
        TargetHoldover.HoldResult<String> result = holdover.apply(null);

        assertNull(result.value());
    }

    @Test
    void initialNullInputReturnsNull() {
        double[] time = {0.0};
        TargetHoldover<String> holdover = new TargetHoldover<>(1.0, () -> time[0]);

        TargetHoldover.HoldResult<String> result = holdover.apply(null);
        assertNull(result.value());
        assertFalse(result.held());
    }

    @Test
    void exactlyAtHoldWindowBoundaryReturnsHeldValue() {
        double[] time = {0.0};
        TargetHoldover<String> holdover = new TargetHoldover<>(1.0, () -> time[0]);

        holdover.apply("target");
        time[0] = 1.0;
        TargetHoldover.HoldResult<String> result = holdover.apply(null);

        assertEquals("target", result.value());
        assertTrue(result.held());
    }
}
