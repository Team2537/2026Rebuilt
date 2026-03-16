package frc.robot.commands;

import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import java.util.Locale;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class HubAlignControllerJitterGuardTest {
    private static final double DT_SEC = 0.02;
    private static final double POST_SETTLE_STATIC_REACQUIRE_MAX_RAD_PER_SEC = Units.degreesToRadians(8.0);

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
    void staticTargetReacquireIsSofterAfterControllerHasAlreadySettledOnce() {
        Rotation2d target = Rotation2d.kZero;
        double reacquireHeadingRad = Units.degreesToRadians(-2.3);

        HubAlignController unarmedController = new HubAlignController();
        unarmedController.reset(0.0, target);
        SimHooks.stepTiming(DT_SEC);
        double unarmedOmega = Math.abs(unarmedController.calculate(reacquireHeadingRad, target, 0.0));

        HubAlignController armedController = new HubAlignController();
        armedController.reset(0.0, target);
        SimHooks.stepTiming(DT_SEC);
        armedController.calculate(Units.degreesToRadians(-0.5), target, 0.0);
        SimHooks.stepTiming(DT_SEC);
        double armedOmega = Math.abs(armedController.calculate(reacquireHeadingRad, target, 0.0));

        assertTrue(
                armedOmega <= POST_SETTLE_STATIC_REACQUIRE_MAX_RAD_PER_SEC,
                String.format(
                        Locale.US,
                        "Expected post-settle static reacquire omega <= %.3f rad/s, got %.3f rad/s",
                        POST_SETTLE_STATIC_REACQUIRE_MAX_RAD_PER_SEC,
                        armedOmega));
        assertTrue(
                armedOmega < unarmedOmega * 0.5,
                String.format(
                        Locale.US,
                        "Expected post-settle static reacquire to be < 50%% of initial command, armed=%.3f unarmed=%.3f",
                        armedOmega,
                        unarmedOmega));
    }
}
