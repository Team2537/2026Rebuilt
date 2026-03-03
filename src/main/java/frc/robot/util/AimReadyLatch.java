package frc.robot.util;

/** Hysteresis latch for heading error based aim readiness. */
public final class AimReadyLatch {
    private final double engageToleranceRad;
    private final double releaseToleranceRad;
    private boolean latched = false;

    public AimReadyLatch(double engageToleranceRad, double releaseToleranceRad) {
        this.engageToleranceRad = engageToleranceRad;
        this.releaseToleranceRad = releaseToleranceRad;
    }

    public boolean update(double absoluteHeadingErrorRad) {
        if (latched) {
            latched = absoluteHeadingErrorRad <= releaseToleranceRad;
        } else {
            latched = absoluteHeadingErrorRad <= engageToleranceRad;
        }
        return latched;
    }

    public void reset() {
        latched = false;
    }
}
