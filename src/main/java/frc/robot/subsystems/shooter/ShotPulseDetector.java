package frc.robot.subsystems.shooter;

import frc.robot.util.LoggedTunableNumber;
import java.util.ArrayDeque;
import java.util.Deque;

/**
 * Detects individual shot pulses from raw left-shooter velocity notches relative to the current
 * target RPM, without smoothing.
 */
public final class ShotPulseDetector {
    private static final LoggedTunableNumber minTargetRpm =
            new LoggedTunableNumber("Shooter/ShotDetection/MinTargetRpm", 2800.0);
    private static final LoggedTunableNumber recentMaxWindowSec =
            new LoggedTunableNumber("Shooter/ShotDetection/RecentMaxWindowSec", 0.12);
    private static final LoggedTunableNumber maxTargetRangeRpm =
            new LoggedTunableNumber("Shooter/ShotDetection/MaxTargetRangeRpm", 120.0);
    private static final LoggedTunableNumber minNotchDepthRpm =
            new LoggedTunableNumber("Shooter/ShotDetection/MinNotchDepthRpm", 250.0);
    private static final LoggedTunableNumber minErrorRpm =
            new LoggedTunableNumber("Shooter/ShotDetection/MinErrorRpm", 180.0);
    private static final LoggedTunableNumber minImmediateStepRpm =
            new LoggedTunableNumber("Shooter/ShotDetection/MinImmediateStepRpm", 20.0);
    private static final LoggedTunableNumber minShotSeparationSec =
            new LoggedTunableNumber("Shooter/ShotDetection/MinShotSeparationSec", 0.05);

    private final Deque<Sample> recentSamples = new ArrayDeque<>();
    private Sample previousSample;
    private Sample previousPreviousSample;
    private double lastShotTimestampSec = Double.NEGATIVE_INFINITY;

    /** Result of one detector update. */
    public record UpdateResult(
            boolean detected,
            boolean armed,
            double candidateTimestampSec,
            double candidateTargetRpm,
            double candidateMeasuredRpm,
            double candidateErrorRpm,
            double notchDepthRpm,
            double leftStepRpm,
            double rightStepRpm,
            double recentMaxRpm,
            double targetRangeRpm) {
        private static UpdateResult empty(boolean armed) {
            return new UpdateResult(
                    false,
                    armed,
                    Double.NaN,
                    Double.NaN,
                    Double.NaN,
                    Double.NaN,
                    Double.NaN,
                    Double.NaN,
                    Double.NaN,
                    Double.NaN,
                    Double.NaN);
        }
    }

    private record Sample(
            double timestampSec,
            double targetRpm,
            double measuredRpm,
            double errorRpm,
            boolean armed) {}

    public UpdateResult update(double timestampSec, double targetRpm, double measuredRpm, boolean armed) {
        double activeMinTargetRpm = minTargetRpm.get();
        double activeRecentMaxWindowSec = recentMaxWindowSec.get();
        double activeMaxTargetRangeRpm = maxTargetRangeRpm.get();
        double activeMinNotchDepthRpm = minNotchDepthRpm.get();
        double activeMinErrorRpm = minErrorRpm.get();
        double activeMinImmediateStepRpm = minImmediateStepRpm.get();
        double activeMinShotSeparationSec = minShotSeparationSec.get();

        Sample current = new Sample(timestampSec, targetRpm, measuredRpm, targetRpm - measuredRpm, armed);
        while (!recentSamples.isEmpty()
                && timestampSec - recentSamples.peekFirst().timestampSec() > activeRecentMaxWindowSec) {
            recentSamples.removeFirst();
        }

        UpdateResult result = UpdateResult.empty(armed);
        if (previousPreviousSample != null && previousSample != null) {
            Sample candidate = previousSample;
            double recentMaxRpm = candidate.measuredRpm();
            double targetMinRpm = candidate.targetRpm();
            double targetMaxRpm = candidate.targetRpm();

            for (Sample sample : recentSamples) {
                if (sample.timestampSec() >= candidate.timestampSec()) {
                    continue;
                }
                recentMaxRpm = Math.max(recentMaxRpm, sample.measuredRpm());
                targetMinRpm = Math.min(targetMinRpm, sample.targetRpm());
                targetMaxRpm = Math.max(targetMaxRpm, sample.targetRpm());
            }
            targetMinRpm = Math.min(targetMinRpm, current.targetRpm());
            targetMaxRpm = Math.max(targetMaxRpm, current.targetRpm());

            double notchDepthRpm = recentMaxRpm - candidate.measuredRpm();
            double leftStepRpm = previousPreviousSample.measuredRpm() - candidate.measuredRpm();
            double rightStepRpm = current.measuredRpm() - candidate.measuredRpm();
            double targetRangeRpm = targetMaxRpm - targetMinRpm;

            boolean localMinimum = previousPreviousSample.measuredRpm() > candidate.measuredRpm()
                    && current.measuredRpm() >= candidate.measuredRpm();
            boolean enoughTimeSinceLastShot =
                    candidate.timestampSec() - lastShotTimestampSec >= activeMinShotSeparationSec;
            boolean passes = candidate.armed()
                    && candidate.targetRpm() >= activeMinTargetRpm
                    && localMinimum
                    && notchDepthRpm >= activeMinNotchDepthRpm
                    && candidate.errorRpm() >= activeMinErrorRpm
                    && leftStepRpm >= activeMinImmediateStepRpm
                    && rightStepRpm >= activeMinImmediateStepRpm
                    && targetRangeRpm <= activeMaxTargetRangeRpm
                    && enoughTimeSinceLastShot;

            if (passes) {
                lastShotTimestampSec = candidate.timestampSec();
            }
            result = new UpdateResult(
                    passes,
                    candidate.armed(),
                    candidate.timestampSec(),
                    candidate.targetRpm(),
                    candidate.measuredRpm(),
                    candidate.errorRpm(),
                    notchDepthRpm,
                    leftStepRpm,
                    rightStepRpm,
                    recentMaxRpm,
                    targetRangeRpm);
        }

        recentSamples.addLast(current);
        previousPreviousSample = previousSample;
        previousSample = current;
        return result;
    }

    public void reset() {
        recentSamples.clear();
        previousSample = null;
        previousPreviousSample = null;
        lastShotTimestampSec = Double.NEGATIVE_INFINITY;
    }
}
