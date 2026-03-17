package frc.robot.sim;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import frc.robot.commands.HubAlignController;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

final class HubAlignControllerResponseSimTest {
    private static final double DT_SEC = 0.02;
    private static final double PLANT_TIME_CONSTANT_SEC = 0.09;
    private static final double PLANT_MAX_ANGULAR_ACCEL_RAD_PER_SEC_SQ = 30.0;
    private static final double STATIC_SETTLE_ERROR_DEG = 2.0;
    private static final double STATIC_POST_SETTLE_MAX_ERROR_DEG = 2.2;

    @BeforeEach
    void setUp() {
        HAL.initialize(500, 0);
        SimHooks.pauseTiming();
        DriverStationSim.resetData();
    }

    @AfterEach
    void tearDown() {
        SimHooks.resumeTiming();
    }

    @Test
    void staticStepResponseSettlesQuicklyWithoutOscillationOrSpikes() {
        HubAlignController controller = new HubAlignController();
        Rotation2d target = Rotation2d.fromDegrees(35.0);
        SimState state = new SimState(0.0, 0.0);
        controller.reset(state.headingRad, target);

        int settleCycles = 0;
        boolean settled = false;
        double settledAtSec = Double.NaN;
        double previousErrorSign = 0.0;
        boolean crossedOnce = false;
        int postCrossOscillationCount = 0;
        double maxCommandAccel = 0.0;
        double previousCommandOmega = 0.0;
        boolean havePreviousCommand = false;
        double maxAbsErrorAfterSettleDeg = 0.0;

        for (int i = 0; i < 180; i++) {
            SimHooks.stepTiming(DT_SEC);
            double commandOmega = controller.calculate(state.headingRad, state.omegaRadPerSec, target, 0.0);
            if (havePreviousCommand) {
                maxCommandAccel = Math.max(maxCommandAccel, Math.abs(commandOmega - previousCommandOmega) / DT_SEC);
            }
            previousCommandOmega = commandOmega;
            havePreviousCommand = true;

            state = advancePlant(state, commandOmega);
            double errorRad = MathUtil.angleModulus(target.getRadians() - state.headingRad);
            double absErrorDeg = Math.abs(Units.radiansToDegrees(errorRad));

            if (Math.abs(errorRad) > Units.degreesToRadians(0.6)) {
                double sign = Math.signum(errorRad);
                if (previousErrorSign != 0.0 && sign != previousErrorSign) {
                    if (crossedOnce) {
                        postCrossOscillationCount++;
                    } else {
                        crossedOnce = true;
                    }
                }
                previousErrorSign = sign;
            }

            if (absErrorDeg <= STATIC_SETTLE_ERROR_DEG && Math.abs(state.omegaRadPerSec) <= 0.35) {
                settleCycles++;
                if (!settled && settleCycles >= 15) {
                    settled = true;
                    settledAtSec = (i + 1) * DT_SEC;
                }
            } else {
                settleCycles = 0;
            }

            if (settled) {
                maxAbsErrorAfterSettleDeg = Math.max(maxAbsErrorAfterSettleDeg, absErrorDeg);
            }
        }

        assertTrue(
                settled,
                FullFunctionalityHarness.formatExpectedVsActual(
                        "HubAlign should settle from a 35-degree step",
                        "settled=true",
                        "settled=false"));
        assertTrue(
                settledAtSec <= 1.2,
                FullFunctionalityHarness.formatExpectedVsActual(
                        "HubAlign should settle quickly without sluggishness",
                        "<= 1.20s",
                        String.format(Locale.US, "%.3fs", settledAtSec)));
        assertTrue(
                postCrossOscillationCount <= 1,
                FullFunctionalityHarness.formatExpectedVsActual(
                        "HubAlign should not oscillate repeatedly after first crossing",
                        "postCrossOscillationCount<=1",
                        "postCrossOscillationCount=" + postCrossOscillationCount));
        assertTrue(
                maxCommandAccel <= 26.0,
                FullFunctionalityHarness.formatExpectedVsActual(
                        "HubAlign command acceleration should stay smooth",
                        "<= 26.0 rad/s^2",
                        String.format(Locale.US, "%.3f rad/s^2", maxCommandAccel)));
        assertTrue(
                maxAbsErrorAfterSettleDeg <= STATIC_POST_SETTLE_MAX_ERROR_DEG,
                FullFunctionalityHarness.formatExpectedVsActual(
                        "HubAlign should remain stable after settling",
                        String.format(Locale.US, "<= %.1f deg", STATIC_POST_SETTLE_MAX_ERROR_DEG),
                        String.format(Locale.US, "%.3f deg", maxAbsErrorAfterSettleDeg)));
    }

    @Test
    void movingTargetTrackingStaysSmoothWithLowChatter() {
        HubAlignController controller = new HubAlignController();
        SimState state = new SimState(0.0, 0.0);
        double initialTargetRad = movingTargetHeadingRad(0.0);
        controller.reset(state.headingRad, Rotation2d.fromRadians(initialTargetRad));

        List<Double> absoluteErrorDegSamples = new ArrayList<>();
        int lowErrorSignFlips = 0;
        double previousErrorSign = 0.0;
        double maxCommandAccel = 0.0;
        double previousCommandOmega = 0.0;
        boolean havePreviousCommand = false;

        for (int i = 0; i < 260; i++) {
            double tSec = (i + 1) * DT_SEC;
            Rotation2d target = Rotation2d.fromRadians(movingTargetHeadingRad(tSec));

            SimHooks.stepTiming(DT_SEC);
            double commandOmega = controller.calculate(state.headingRad, state.omegaRadPerSec, target, 0.0);
            if (havePreviousCommand) {
                maxCommandAccel = Math.max(maxCommandAccel, Math.abs(commandOmega - previousCommandOmega) / DT_SEC);
            }
            previousCommandOmega = commandOmega;
            havePreviousCommand = true;

            state = advancePlant(state, commandOmega);
            double errorRad = MathUtil.angleModulus(target.getRadians() - state.headingRad);
            double absErrorDeg = Math.abs(Units.radiansToDegrees(errorRad));
            absoluteErrorDegSamples.add(absErrorDeg);

            if (absErrorDeg <= 2.0) {
                double sign = Math.signum(errorRad);
                if (previousErrorSign != 0.0 && sign != 0.0 && sign != previousErrorSign) {
                    lowErrorSignFlips++;
                }
                if (sign != 0.0) {
                    previousErrorSign = sign;
                }
            }
        }

        assertFalse(
                absoluteErrorDegSamples.isEmpty(),
                FullFunctionalityHarness.formatExpectedVsActual(
                        "Moving-target simulation should produce heading error samples",
                        "sampleCount>0",
                        "sampleCount=0"));
        double p95ErrorDeg = percentile(absoluteErrorDegSamples, 0.95);

        assertTrue(
                p95ErrorDeg <= 7.5,
                FullFunctionalityHarness.formatExpectedVsActual(
                        "HubAlign should keep moving-target heading error bounded",
                        "<= 7.5 deg",
                        String.format(Locale.US, "%.3f deg", p95ErrorDeg)));
        assertTrue(
                maxCommandAccel <= 26.0,
                FullFunctionalityHarness.formatExpectedVsActual(
                        "HubAlign moving-target output should avoid sharp spikes",
                        "<= 26.0 rad/s^2",
                        String.format(Locale.US, "%.3f rad/s^2", maxCommandAccel)));
        assertTrue(
                lowErrorSignFlips <= 10,
                FullFunctionalityHarness.formatExpectedVsActual(
                        "HubAlign should not chatter near the target during tracking",
                        "lowErrorSignFlips<=10",
                        "lowErrorSignFlips=" + lowErrorSignFlips));
    }

    private static SimState advancePlant(SimState state, double commandedOmegaRadPerSec) {
        double angularAccelerationRadPerSecSq = MathUtil.clamp(
                (commandedOmegaRadPerSec - state.omegaRadPerSec) / PLANT_TIME_CONSTANT_SEC,
                -PLANT_MAX_ANGULAR_ACCEL_RAD_PER_SEC_SQ,
                PLANT_MAX_ANGULAR_ACCEL_RAD_PER_SEC_SQ);
        double nextOmegaRadPerSec = state.omegaRadPerSec + angularAccelerationRadPerSecSq * DT_SEC;
        double nextHeadingRad = MathUtil.angleModulus(state.headingRad + nextOmegaRadPerSec * DT_SEC);
        return new SimState(nextHeadingRad, nextOmegaRadPerSec);
    }

    private static double movingTargetHeadingRad(double tSec) {
        return Units.degreesToRadians(24.0) * Math.sin(2.0 * Math.PI * 0.35 * tSec);
    }

    private static double percentile(List<Double> samples, double percentile) {
        if (samples.isEmpty()) {
            return Double.NaN;
        }
        List<Double> sorted = new ArrayList<>(samples);
        sorted.sort(Double::compareTo);
        int index = (int) Math.round(MathUtil.clamp(percentile, 0.0, 1.0) * (sorted.size() - 1));
        return sorted.get(index);
    }

    private record SimState(double headingRad, double omegaRadPerSec) {}
}
