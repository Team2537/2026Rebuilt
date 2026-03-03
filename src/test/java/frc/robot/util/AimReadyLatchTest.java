package frc.robot.util;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

class AimReadyLatchTest {
    private static final double ENGAGE = 0.05;
    private static final double RELEASE = 0.10;

    @Test
    void engagesWhenErrorBelowEngageTolerance() {
        AimReadyLatch latch = new AimReadyLatch(ENGAGE, RELEASE);
        assertTrue(latch.update(0.03));
    }

    @Test
    void doesNotEngageAboveEngageTolerance() {
        AimReadyLatch latch = new AimReadyLatch(ENGAGE, RELEASE);
        assertFalse(latch.update(0.07));
    }

    @Test
    void staysLatchedInHysteresisBand() {
        AimReadyLatch latch = new AimReadyLatch(ENGAGE, RELEASE);
        latch.update(0.03); // engage
        assertTrue(latch.update(0.08)); // between engage and release — stays latched
    }

    @Test
    void releasesAboveReleaseTolerance() {
        AimReadyLatch latch = new AimReadyLatch(ENGAGE, RELEASE);
        latch.update(0.03); // engage
        assertFalse(latch.update(0.15)); // above release tolerance
    }

    @Test
    void resetClearsLatch() {
        AimReadyLatch latch = new AimReadyLatch(ENGAGE, RELEASE);
        latch.update(0.03); // engage
        latch.reset();
        // After reset, error in hysteresis band should not latch (since engage requires <= ENGAGE)
        assertFalse(latch.update(0.08));
    }

    @Test
    void reEngagesAfterRelease() {
        AimReadyLatch latch = new AimReadyLatch(ENGAGE, RELEASE);
        latch.update(0.03);  // engage
        latch.update(0.15);  // release
        assertFalse(latch.update(0.07)); // in band but not engaged yet
        assertTrue(latch.update(0.04));  // back below engage
    }

    @Test
    void exactlyAtEngageToleranceEngages() {
        AimReadyLatch latch = new AimReadyLatch(ENGAGE, RELEASE);
        assertTrue(latch.update(ENGAGE));
    }

    @Test
    void exactlyAtReleaseToleranceStaysLatched() {
        AimReadyLatch latch = new AimReadyLatch(ENGAGE, RELEASE);
        latch.update(0.03); // engage
        assertTrue(latch.update(RELEASE)); // exactly at release — stays latched (<=)
    }
}
