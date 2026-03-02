package frc.robot.sim;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants;
import frc.robot.DashboardOverrides;
import frc.robot.commands.HubAlignController;
import frc.robot.coordination.shooting.ShootCoordinator;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.transfer.Transfer;
import frc.robot.subsystems.transfer.TransferConstants;
import frc.robot.subsystems.transfer.TransferIO;
import java.lang.reflect.Constructor;
import java.lang.reflect.Method;
import java.util.Locale;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

final class FullFunctionalityUncoveredPathExploration {
    private static final double EPSILON = 1e-9;

    private static final String AUTO_AIM_KEY = "Overrides/OverrideAutoAim";
    private static final String AIM_DISTANCE_METERS_KEY = "Overrides/AimDistanceMeters";
    private static final String DISABLE_VISION_KEY = "Overrides/DisableVision";
    private static final double DEFAULT_AIM_DISTANCE_METERS = 1.5;

    @BeforeEach
    void setUp() {
        HAL.initialize(500, 0);
        SimHooks.pauseTiming();
        DriverStationSim.resetData();

        CommandScheduler scheduler = CommandScheduler.getInstance();
        scheduler.cancelAll();
        scheduler.getDefaultButtonLoop().clear();
        scheduler.getActiveButtonLoop().clear();
        scheduler.setActiveButtonLoop(scheduler.getDefaultButtonLoop());

        DriverStationSim.setDsAttached(true);
        DriverStationSim.setAutonomous(false);
        DriverStationSim.setTest(false);
        DriverStationSim.setEStop(false);
        DriverStationSim.setEnabled(true);
        DriverStationSim.notifyNewData();
    }

    @AfterEach
    void tearDown() {
        CommandScheduler.getInstance().cancelAll();
        DriverStationSim.setEnabled(false);
        DriverStationSim.notifyNewData();
        SimHooks.resumeTiming();
    }

    @Test
    void hubAlignTargetLossAssumesImmediateFallbackAndFlagsDivergenceIfHeld() {
        HubAlignController controller = new HubAlignController();
        controller.reset(0.0, Rotation2d.fromDegrees(35.0));

        SimHooks.stepTiming(0.02);
        double fallbackOmega = 0.0;
        double actualOmega = controller.calculate(0.0, null, fallbackOmega);

        assertEquals(
                fallbackOmega,
                actualOmega,
                1e-6,
                FullFunctionalityHarness.formatExpectedVsActual(
                        "HubAlign should immediately fall back on first null target sample (assumption)",
                        formatOmega(fallbackOmega),
                        formatOmega(actualOmega)));
    }

    @Test
    void hubAlignHoldsThenFallsBackThenReacquiresTarget() {
        HubAlignController controller = new HubAlignController();
        controller.reset(0.0, Rotation2d.fromDegrees(30.0));

        SimHooks.stepTiming(0.02);
        double fallbackOmega = 0.4;
        double heldOmega = controller.calculate(0.0, null, fallbackOmega);
        assertTrue(
                Math.abs(heldOmega - fallbackOmega) > 0.05,
                FullFunctionalityHarness.formatExpectedVsActual(
                        "Within hold window, held-target output should diverge from fallback",
                        "|heldOmega - fallbackOmega| > 0.05",
                        String.format(
                                Locale.US,
                                "heldOmega=%.6f fallbackOmega=%.6f |diff|=%.6f",
                                heldOmega,
                                fallbackOmega,
                                Math.abs(heldOmega - fallbackOmega))));

        SimHooks.stepTiming(0.14);
        double postHoldOmega = controller.calculate(0.0, null, fallbackOmega);
        assertEquals(
                fallbackOmega,
                postHoldOmega,
                1e-6,
                FullFunctionalityHarness.formatExpectedVsActual(
                        "After hold expiry, controller should output fallback omega",
                        formatOmega(fallbackOmega),
                        formatOmega(postHoldOmega)));

        SimHooks.stepTiming(0.02);
        double reacquiredOmega = controller.calculate(0.0, Rotation2d.fromDegrees(18.0), fallbackOmega);
        assertTrue(
                Math.abs(reacquiredOmega - fallbackOmega) > 1e-3,
                FullFunctionalityHarness.formatExpectedVsActual(
                        "Target reacquisition should move command off fallback omega",
                        "|reacquiredOmega - fallbackOmega| > 0.001",
                        String.format(
                                Locale.US,
                                "reacquiredOmega=%.6f fallbackOmega=%.6f |diff|=%.6f",
                                reacquiredOmega,
                                fallbackOmega,
                                Math.abs(reacquiredOmega - fallbackOmega))));
    }

    @Test
    void dashboardOverridesInvalidDistanceInputsFallbackToDefault() {
        DashboardOverrides overrides = new DashboardOverrides();
        overrides.init();

        assertDashboardDistanceEquals(
                "Fresh dashboard should report default aim distance",
                DEFAULT_AIM_DISTANCE_METERS,
                overrides.getAimDistanceMeters());

        SmartDashboard.putNumber(AIM_DISTANCE_METERS_KEY, Double.NaN);
        assertDashboardDistanceEquals(
                "NaN aim distance should fall back to default",
                DEFAULT_AIM_DISTANCE_METERS,
                overrides.getAimDistanceMeters());

        SmartDashboard.putNumber(AIM_DISTANCE_METERS_KEY, -0.5);
        assertDashboardDistanceEquals(
                "Negative aim distance should fall back to default",
                DEFAULT_AIM_DISTANCE_METERS,
                overrides.getAimDistanceMeters());

        SmartDashboard.putNumber(AIM_DISTANCE_METERS_KEY, 0.0);
        assertDashboardDistanceEquals(
                "Zero aim distance should fall back to default",
                DEFAULT_AIM_DISTANCE_METERS,
                overrides.getAimDistanceMeters());

        SmartDashboard.putNumber(AIM_DISTANCE_METERS_KEY, 2.75);
        assertDashboardDistanceEquals(
                "Positive finite aim distance should pass through",
                2.75,
                overrides.getAimDistanceMeters());
    }

    @Test
    void dashboardClearAllRemainsStableUnderRapidToggling() {
        DashboardOverrides overrides = new DashboardOverrides();
        overrides.init();

        for (int cycle = 0; cycle < 25; cycle++) {
            SmartDashboard.putBoolean(AUTO_AIM_KEY, cycle % 2 == 0);
            SmartDashboard.putNumber(AIM_DISTANCE_METERS_KEY, cycle % 3 == 0 ? Double.NaN : cycle + 0.25);
            SmartDashboard.putBoolean(DISABLE_VISION_KEY, cycle % 2 != 0);

            CommandScheduler.getInstance().schedule(overrides.clearAllCommand());
            runSchedulerCycles(1);

            boolean autoAimEnabled = SmartDashboard.getBoolean(AUTO_AIM_KEY, true);
            double distanceMeters = SmartDashboard.getNumber(AIM_DISTANCE_METERS_KEY, -999.0);
            boolean visionDisabled = SmartDashboard.getBoolean(DISABLE_VISION_KEY, true);

            assertFalse(
                    autoAimEnabled,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "Clear-all should reset OverrideAutoAim each rapid cycle",
                            "cycle=" + cycle + " autoAim=false",
                            "cycle=" + cycle + " autoAim=" + autoAimEnabled));
            assertEquals(
                    DEFAULT_AIM_DISTANCE_METERS,
                    distanceMeters,
                    EPSILON,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "Clear-all should reset AimDistanceMeters each rapid cycle",
                            String.format(Locale.US, "cycle=%d distance=%.3f", cycle, DEFAULT_AIM_DISTANCE_METERS),
                            String.format(Locale.US, "cycle=%d distance=%.3f", cycle, distanceMeters)));
            assertFalse(
                    visionDisabled,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "Clear-all should reset DisableVision each rapid cycle",
                            "cycle=" + cycle + " disableVision=false",
                            "cycle=" + cycle + " disableVision=" + visionDisabled));
        }
    }

    @Test
    void shootCoordinatorImmediateModeKeepsGateOpenExceptOnInvalidDistance() {
        RecordingShooter shooter = new RecordingShooter();
        RecordingTransfer transfer = new RecordingTransfer();
        ShootCoordinator coordinator = new ShootCoordinator(shooter, transfer, Constants.FeedGateMode.IMMEDIATE);

        AtomicReference<Double> distanceMeters = new AtomicReference<>(4.0);
        AtomicBoolean aimReady = new AtomicBoolean(false);

        Command command = coordinator.shootForDistance(distanceMeters::get, aimReady::get);
        CommandScheduler.getInstance().schedule(command);

        runSchedulerCycles(2);
        assertEquals(
                2,
                shooter.setTargetsForDistanceCalls,
                FullFunctionalityHarness.formatExpectedVsActual(
                        "IMMEDIATE mode should update targets every valid cycle",
                        "setTargetsForDistanceCalls=2",
                        "setTargetsForDistanceCalls=" + shooter.setTargetsForDistanceCalls));
        assertEquals(
                2,
                shooter.setKickerTorqueCalls,
                FullFunctionalityHarness.formatExpectedVsActual(
                        "IMMEDIATE mode should open gate each valid cycle",
                        "setKickerTorqueCalls=2",
                        "setKickerTorqueCalls=" + shooter.setKickerTorqueCalls));
        assertEquals(
                0,
                shooter.stopKickerCalls,
                FullFunctionalityHarness.formatExpectedVsActual(
                        "IMMEDIATE mode should not close gate while target valid",
                        "stopKickerCalls=0",
                        "stopKickerCalls=" + shooter.stopKickerCalls));
        assertEquals(
                2,
                transfer.setPercentCalls,
                FullFunctionalityHarness.formatExpectedVsActual(
                        "IMMEDIATE mode should run transfer each valid cycle",
                        "setPercentCalls=2",
                        "setPercentCalls=" + transfer.setPercentCalls));
        assertEquals(
                ShooterConstants.DEFAULT_KICKER_TORQUE_AMPS,
                shooter.lastKickerTorqueAmps,
                EPSILON,
                FullFunctionalityHarness.formatExpectedVsActual(
                        "Coordinator should apply default kicker torque when gate opens",
                        ShooterConstants.DEFAULT_KICKER_TORQUE_AMPS,
                        shooter.lastKickerTorqueAmps));
        assertEquals(
                TransferConstants.RUN_TRANSFER_PERCENT,
                transfer.lastPercent,
                EPSILON,
                FullFunctionalityHarness.formatExpectedVsActual(
                        "Coordinator should apply run transfer percent when gate opens",
                        TransferConstants.RUN_TRANSFER_PERCENT,
                        transfer.lastPercent));

        distanceMeters.set(Double.NaN);
        runSchedulerCycles(1);
        assertEquals(
                2,
                shooter.setTargetsForDistanceCalls,
                FullFunctionalityHarness.formatExpectedVsActual(
                        "Invalid distance should skip target updates",
                        "setTargetsForDistanceCalls=2",
                        "setTargetsForDistanceCalls=" + shooter.setTargetsForDistanceCalls));
        assertEquals(
                1,
                shooter.stopKickerCalls,
                FullFunctionalityHarness.formatExpectedVsActual(
                        "Invalid distance should close gate immediately",
                        "stopKickerCalls=1",
                        "stopKickerCalls=" + shooter.stopKickerCalls));
        assertEquals(
                1,
                transfer.stopAllCalls,
                FullFunctionalityHarness.formatExpectedVsActual(
                        "Invalid distance should stop transfer immediately",
                        "stopAllCalls=1",
                        "stopAllCalls=" + transfer.stopAllCalls));

        distanceMeters.set(4.5);
        runSchedulerCycles(1);
        assertEquals(
                3,
                shooter.setTargetsForDistanceCalls,
                FullFunctionalityHarness.formatExpectedVsActual(
                        "Returning to valid distance should resume target updates",
                        "setTargetsForDistanceCalls=3",
                        "setTargetsForDistanceCalls=" + shooter.setTargetsForDistanceCalls));
        assertEquals(
                3,
                shooter.setKickerTorqueCalls,
                FullFunctionalityHarness.formatExpectedVsActual(
                        "Returning to valid distance should reopen gate immediately in IMMEDIATE mode",
                        "setKickerTorqueCalls=3",
                        "setKickerTorqueCalls=" + shooter.setKickerTorqueCalls));

        command.cancel();
        runSchedulerCycles(1);
        assertTrue(
                shooter.stopAllCalls >= 1,
                FullFunctionalityHarness.formatExpectedVsActual(
                        "Canceling command should stop shooter outputs",
                        "stopAllCalls>=1",
                        "stopAllCalls=" + shooter.stopAllCalls));
        assertTrue(
                transfer.stopAllCalls >= 2,
                FullFunctionalityHarness.formatExpectedVsActual(
                        "Canceling command should stop transfer outputs",
                        "stopAllCalls>=2",
                        "stopAllCalls=" + transfer.stopAllCalls));
    }

    @Test
    void shootCoordinatorShooterAtSetpointModeDebouncesAndResetsInOrder() {
        RecordingShooter shooter = new RecordingShooter();
        RecordingTransfer transfer = new RecordingTransfer();
        ShootCoordinator coordinator = new ShootCoordinator(shooter, transfer, Constants.FeedGateMode.SHOOTER_AT_SETPOINT);
        AtomicReference<Double> distanceMeters = new AtomicReference<>(4.0);

        Command command = coordinator.shootForDistance(distanceMeters::get, () -> false);
        CommandScheduler.getInstance().schedule(command);

        shooter.atSetpoint = false;
        runSchedulerCycles(1);
        assertEquals(
                0,
                shooter.setKickerTorqueCalls,
                FullFunctionalityHarness.formatExpectedVsActual(
                        "Cycle 1: gate must remain closed when shooter not at setpoint",
                        "setKickerTorqueCalls=0",
                        "setKickerTorqueCalls=" + shooter.setKickerTorqueCalls));

        shooter.atSetpoint = true;
        runSchedulerCycles(1);
        assertEquals(
                0,
                shooter.setKickerTorqueCalls,
                FullFunctionalityHarness.formatExpectedVsActual(
                        "Cycle 2: first ready cycle should still debounce closed",
                        "setKickerTorqueCalls=0",
                        "setKickerTorqueCalls=" + shooter.setKickerTorqueCalls));

        runSchedulerCycles(1);
        assertEquals(
                1,
                shooter.setKickerTorqueCalls,
                FullFunctionalityHarness.formatExpectedVsActual(
                        "Cycle 3: second consecutive ready cycle should open gate",
                        "setKickerTorqueCalls=1",
                        "setKickerTorqueCalls=" + shooter.setKickerTorqueCalls));

        shooter.atSetpoint = false;
        runSchedulerCycles(1);
        assertEquals(
                1,
                shooter.setKickerTorqueCalls,
                FullFunctionalityHarness.formatExpectedVsActual(
                        "Cycle 4: readiness drop should close gate and reset debounce",
                        "setKickerTorqueCalls=1",
                        "setKickerTorqueCalls=" + shooter.setKickerTorqueCalls));

        shooter.atSetpoint = true;
        runSchedulerCycles(1);
        assertEquals(
                1,
                shooter.setKickerTorqueCalls,
                FullFunctionalityHarness.formatExpectedVsActual(
                        "Cycle 5: after reset, first ready cycle should stay closed",
                        "setKickerTorqueCalls=1",
                        "setKickerTorqueCalls=" + shooter.setKickerTorqueCalls));

        runSchedulerCycles(1);
        assertEquals(
                2,
                shooter.setKickerTorqueCalls,
                FullFunctionalityHarness.formatExpectedVsActual(
                        "Cycle 6: second ready cycle after reset should reopen gate",
                        "setKickerTorqueCalls=2",
                        "setKickerTorqueCalls=" + shooter.setKickerTorqueCalls));

        distanceMeters.set(Double.NaN);
        runSchedulerCycles(1);
        assertEquals(
                6,
                shooter.setTargetsForDistanceCalls,
                FullFunctionalityHarness.formatExpectedVsActual(
                        "Cycle 7: invalid distance should skip target update and reset debounce",
                        "setTargetsForDistanceCalls=6",
                        "setTargetsForDistanceCalls=" + shooter.setTargetsForDistanceCalls));

        distanceMeters.set(4.0);
        runSchedulerCycles(1);
        assertEquals(
                2,
                shooter.setKickerTorqueCalls,
                FullFunctionalityHarness.formatExpectedVsActual(
                        "Cycle 8: post-invalid first ready cycle should remain closed",
                        "setKickerTorqueCalls=2",
                        "setKickerTorqueCalls=" + shooter.setKickerTorqueCalls));

        runSchedulerCycles(1);
        assertEquals(
                3,
                shooter.setKickerTorqueCalls,
                FullFunctionalityHarness.formatExpectedVsActual(
                        "Cycle 9: post-invalid second ready cycle should reopen gate",
                        "setKickerTorqueCalls=3",
                        "setKickerTorqueCalls=" + shooter.setKickerTorqueCalls));

        assertEquals(
                8,
                shooter.setTargetsForDistanceCalls,
                FullFunctionalityHarness.formatExpectedVsActual(
                        "Only valid distance cycles should update shooter targets",
                        "setTargetsForDistanceCalls=8",
                        "setTargetsForDistanceCalls=" + shooter.setTargetsForDistanceCalls));
        assertEquals(
                6,
                shooter.stopKickerCalls,
                FullFunctionalityHarness.formatExpectedVsActual(
                        "Closed-gate cycles should stop kicker exactly 6 times",
                        "stopKickerCalls=6",
                        "stopKickerCalls=" + shooter.stopKickerCalls));
        assertEquals(
                3,
                transfer.setPercentCalls,
                FullFunctionalityHarness.formatExpectedVsActual(
                        "Gate-open cycles should run transfer exactly 3 times",
                        "setPercentCalls=3",
                        "setPercentCalls=" + transfer.setPercentCalls));
        assertEquals(
                6,
                transfer.stopAllCalls,
                FullFunctionalityHarness.formatExpectedVsActual(
                        "Gate-closed cycles should stop transfer exactly 6 times",
                        "stopAllCalls=6",
                        "stopAllCalls=" + transfer.stopAllCalls));

        command.cancel();
        runSchedulerCycles(1);
    }

    @Test
    void shootCoordinatorShooterAndAimModeEnforcesOrderingAndDebounce() {
        RecordingShooter shooter = new RecordingShooter();
        RecordingTransfer transfer = new RecordingTransfer();
        ShootCoordinator coordinator = new ShootCoordinator(shooter, transfer, Constants.FeedGateMode.SHOOTER_AND_AIM);
        AtomicReference<Double> distanceMeters = new AtomicReference<>(4.0);
        AtomicBoolean aimReady = new AtomicBoolean(false);

        Command command = coordinator.shootForDistance(distanceMeters::get, aimReady::get);
        CommandScheduler.getInstance().schedule(command);

        shooter.atSetpoint = false;
        aimReady.set(false);
        runSchedulerCycles(1);
        assertEquals(
                0,
                shooter.setKickerTorqueCalls,
                FullFunctionalityHarness.formatExpectedVsActual(
                        "Cycle 1: shooter+aim false should keep gate closed",
                        "setKickerTorqueCalls=0",
                        "setKickerTorqueCalls=" + shooter.setKickerTorqueCalls));

        shooter.atSetpoint = true;
        aimReady.set(false);
        runSchedulerCycles(1);
        assertEquals(
                0,
                shooter.setKickerTorqueCalls,
                FullFunctionalityHarness.formatExpectedVsActual(
                        "Cycle 2: shooter ready but aim not ready should keep gate closed",
                        "setKickerTorqueCalls=0",
                        "setKickerTorqueCalls=" + shooter.setKickerTorqueCalls));

        aimReady.set(true);
        runSchedulerCycles(1);
        assertEquals(
                0,
                shooter.setKickerTorqueCalls,
                FullFunctionalityHarness.formatExpectedVsActual(
                        "Cycle 3: first dual-ready cycle should debounce closed",
                        "setKickerTorqueCalls=0",
                        "setKickerTorqueCalls=" + shooter.setKickerTorqueCalls));

        runSchedulerCycles(1);
        assertEquals(
                1,
                shooter.setKickerTorqueCalls,
                FullFunctionalityHarness.formatExpectedVsActual(
                        "Cycle 4: second dual-ready cycle should open gate",
                        "setKickerTorqueCalls=1",
                        "setKickerTorqueCalls=" + shooter.setKickerTorqueCalls));

        aimReady.set(false);
        runSchedulerCycles(1);
        assertEquals(
                1,
                shooter.setKickerTorqueCalls,
                FullFunctionalityHarness.formatExpectedVsActual(
                        "Cycle 5: aim drop should close gate immediately and reset debounce",
                        "setKickerTorqueCalls=1",
                        "setKickerTorqueCalls=" + shooter.setKickerTorqueCalls));

        aimReady.set(true);
        runSchedulerCycles(1);
        assertEquals(
                1,
                shooter.setKickerTorqueCalls,
                FullFunctionalityHarness.formatExpectedVsActual(
                        "Cycle 6: first dual-ready cycle after reset should remain closed",
                        "setKickerTorqueCalls=1",
                        "setKickerTorqueCalls=" + shooter.setKickerTorqueCalls));

        runSchedulerCycles(1);
        assertEquals(
                2,
                shooter.setKickerTorqueCalls,
                FullFunctionalityHarness.formatExpectedVsActual(
                        "Cycle 7: second dual-ready cycle after reset should reopen gate",
                        "setKickerTorqueCalls=2",
                        "setKickerTorqueCalls=" + shooter.setKickerTorqueCalls));

        shooter.atSetpoint = false;
        runSchedulerCycles(1);
        assertEquals(
                2,
                shooter.setKickerTorqueCalls,
                FullFunctionalityHarness.formatExpectedVsActual(
                        "Cycle 8: shooter drop should close gate and reset debounce",
                        "setKickerTorqueCalls=2",
                        "setKickerTorqueCalls=" + shooter.setKickerTorqueCalls));

        shooter.atSetpoint = true;
        runSchedulerCycles(1);
        assertEquals(
                2,
                shooter.setKickerTorqueCalls,
                FullFunctionalityHarness.formatExpectedVsActual(
                        "Cycle 9: first dual-ready cycle after shooter recovery should remain closed",
                        "setKickerTorqueCalls=2",
                        "setKickerTorqueCalls=" + shooter.setKickerTorqueCalls));

        runSchedulerCycles(1);
        assertEquals(
                3,
                shooter.setKickerTorqueCalls,
                FullFunctionalityHarness.formatExpectedVsActual(
                        "Cycle 10: second dual-ready cycle after shooter recovery should open gate",
                        "setKickerTorqueCalls=3",
                        "setKickerTorqueCalls=" + shooter.setKickerTorqueCalls));

        assertEquals(
                10,
                shooter.setTargetsForDistanceCalls,
                FullFunctionalityHarness.formatExpectedVsActual(
                        "All 10 valid cycles should update shooter targets",
                        "setTargetsForDistanceCalls=10",
                        "setTargetsForDistanceCalls=" + shooter.setTargetsForDistanceCalls));
        assertEquals(
                7,
                shooter.stopKickerCalls,
                FullFunctionalityHarness.formatExpectedVsActual(
                        "Seven gate-closed cycles should stop kicker",
                        "stopKickerCalls=7",
                        "stopKickerCalls=" + shooter.stopKickerCalls));
        assertEquals(
                3,
                transfer.setPercentCalls,
                FullFunctionalityHarness.formatExpectedVsActual(
                        "Three gate-open cycles should run transfer",
                        "setPercentCalls=3",
                        "setPercentCalls=" + transfer.setPercentCalls));
        assertEquals(
                7,
                transfer.stopAllCalls,
                FullFunctionalityHarness.formatExpectedVsActual(
                        "Seven gate-closed cycles should stop transfer",
                        "stopAllCalls=7",
                        "stopAllCalls=" + transfer.stopAllCalls));

        command.cancel();
        runSchedulerCycles(1);
    }

    @Test
    void autoCommandsProfiledHeadingHandlesNullGoalTransitionsWithoutStateCorruption() throws Exception {
        Object profiledHeadingTarget = createProfiledHeadingTarget();

        Rotation2d initialNull = invokeProfiledCalculate(profiledHeadingTarget, null);
        assertNull(
                initialNull,
                FullFunctionalityHarness.formatExpectedVsActual(
                        "Null goal before initialization should return null",
                        "null",
                        String.valueOf(initialNull)));

        Rotation2d firstGoal = Rotation2d.fromDegrees(90.0);
        Rotation2d initialized = invokeProfiledCalculate(profiledHeadingTarget, firstGoal);
        assertNotNull(
                initialized,
                FullFunctionalityHarness.formatExpectedVsActual(
                        "First non-null goal should initialize profiled heading",
                        formatRotation(firstGoal),
                        String.valueOf(initialized)));
        assertEquals(
                firstGoal.getRadians(),
                initialized.getRadians(),
                1e-9,
                FullFunctionalityHarness.formatExpectedVsActual(
                        "First non-null goal should be returned exactly on initialization",
                        formatRotation(firstGoal),
                        formatRotation(initialized)));

        SimHooks.stepTiming(0.02);
        Rotation2d goalAfterInit = Rotation2d.fromDegrees(-120.0);
        Rotation2d profiledStep = invokeProfiledCalculate(profiledHeadingTarget, goalAfterInit);
        assertNotNull(
                profiledStep,
                FullFunctionalityHarness.formatExpectedVsActual(
                        "Profiled heading with initialized state and non-null goal should return a heading",
                        "non-null",
                        String.valueOf(profiledStep)));

        SimHooks.stepTiming(0.02);
        Rotation2d nullAfterInit = invokeProfiledCalculate(profiledHeadingTarget, null);
        assertNull(
                nullAfterInit,
                FullFunctionalityHarness.formatExpectedVsActual(
                        "Null goal after initialization should return null",
                        "null",
                        String.valueOf(nullAfterInit)));
        double velocityAfterNull = invokeProfiledVelocity(profiledHeadingTarget);
        assertEquals(
                0.0,
                velocityAfterNull,
                1e-9,
                FullFunctionalityHarness.formatExpectedVsActual(
                        "Null goal should zero profile velocity",
                        "velocityRadPerSec=0.000000",
                        String.format(Locale.US, "velocityRadPerSec=%.6f", velocityAfterNull)));

        Rotation2d immediateAfterNull = invokeProfiledCalculate(profiledHeadingTarget, goalAfterInit);
        assertNotNull(
                immediateAfterNull,
                FullFunctionalityHarness.formatExpectedVsActual(
                        "Immediate goal reappearance should still return a heading",
                        "non-null",
                        String.valueOf(immediateAfterNull)));
        assertEquals(
                profiledStep.getRadians(),
                immediateAfterNull.getRadians(),
                1e-6,
                FullFunctionalityHarness.formatExpectedVsActual(
                        "With ~0 dt after null transition, heading should hold previous profile state",
                        formatRotation(profiledStep),
                        formatRotation(immediateAfterNull)));

        SimHooks.stepTiming(0.02);
        Rotation2d resumedStep = invokeProfiledCalculate(profiledHeadingTarget, goalAfterInit);
        double resumedDeltaRad = Math.abs(MathUtil.angleModulus(resumedStep.minus(immediateAfterNull).getRadians()));
        assertTrue(
                resumedDeltaRad > 1e-4,
                FullFunctionalityHarness.formatExpectedVsActual(
                        "After time advances, profiled heading should resume moving toward goal",
                        "|deltaRad|>0.0001",
                        String.format(Locale.US, "|deltaRad|=%.6f", resumedDeltaRad)));
    }

    private static void runSchedulerCycles(int cycles) {
        for (int i = 0; i < cycles; i++) {
            SimHooks.stepTiming(0.02);
            DriverStationSim.notifyNewData();
            CommandScheduler.getInstance().run();
        }
    }

    private static void assertDashboardDistanceEquals(String expectation, double expected, double actual) {
        assertEquals(
                expected,
                actual,
                EPSILON,
                FullFunctionalityHarness.formatExpectedVsActual(
                        expectation,
                        String.format(Locale.US, "distanceMeters=%.3f", expected),
                        String.format(Locale.US, "distanceMeters=%.3f", actual)));
    }

    private static String formatOmega(double omega) {
        return String.format(Locale.US, "omegaRadPerSec=%.6f", omega);
    }

    private static String formatRotation(Rotation2d rotation) {
        return rotation == null
                ? "null"
                : String.format(Locale.US, "deg=%.6f", rotation.getDegrees());
    }

    private static Object createProfiledHeadingTarget() throws Exception {
        Class<?> cls = Class.forName("frc.robot.autos.AutoCommands$ProfiledHeadingTarget");
        Constructor<?> constructor = cls.getDeclaredConstructor(TrapezoidProfile.Constraints.class);
        constructor.setAccessible(true);
        return constructor.newInstance(new TrapezoidProfile.Constraints(Math.toRadians(540.0), Math.toRadians(2160.0)));
    }

    private static Rotation2d invokeProfiledCalculate(Object profiledHeadingTarget, Rotation2d goal) throws Exception {
        Method calculate = profiledHeadingTarget.getClass().getDeclaredMethod("calculate", Rotation2d.class);
        calculate.setAccessible(true);
        return (Rotation2d) calculate.invoke(profiledHeadingTarget, new Object[] {goal});
    }

    private static double invokeProfiledVelocity(Object profiledHeadingTarget) throws Exception {
        Method getVelocity = profiledHeadingTarget.getClass().getDeclaredMethod("getVelocityRadPerSec");
        getVelocity.setAccessible(true);
        return (double) getVelocity.invoke(profiledHeadingTarget);
    }

    private static final class RecordingShooter extends Shooter {
        boolean atSetpoint = false;
        int setTargetsForDistanceCalls = 0;
        double lastDistanceMeters = Double.NaN;
        int setKickerTorqueCalls = 0;
        double lastKickerTorqueAmps = Double.NaN;
        int stopKickerCalls = 0;
        int stopAllCalls = 0;

        RecordingShooter() {
            super(new ShooterIO() {});
        }

        @Override
        public void periodic() {
            // Deterministic no-op for gating-path exploration.
        }

        @Override
        public void setTargetsForDistance(double distanceMeters) {
            setTargetsForDistanceCalls++;
            lastDistanceMeters = distanceMeters;
        }

        @Override
        public ReadinessDiagnostics getReadinessDiagnosticsNow() {
            return new ReadinessDiagnostics(
                    0.0,
                    0.0,
                    0.0,
                    atSetpoint,
                    atSetpoint,
                    atSetpoint,
                    atSetpoint,
                    atSetpoint);
        }

        @Override
        public void setKickerTorqueAmps(double torqueAmps) {
            setKickerTorqueCalls++;
            lastKickerTorqueAmps = torqueAmps;
        }

        @Override
        public void stopKicker() {
            stopKickerCalls++;
        }

        @Override
        public void stopAll() {
            stopAllCalls++;
        }
    }

    private static final class RecordingTransfer extends Transfer {
        int setPercentCalls = 0;
        double lastPercent = Double.NaN;
        int stopAllCalls = 0;

        RecordingTransfer() {
            super(new TransferIO() {});
        }

        @Override
        public void periodic() {
            // Deterministic no-op for gating-path exploration.
        }

        @Override
        public void setPercent(double percent) {
            setPercentCalls++;
            lastPercent = percent;
        }

        @Override
        public void stopAll() {
            stopAllCalls++;
            lastPercent = 0.0;
        }
    }
}
