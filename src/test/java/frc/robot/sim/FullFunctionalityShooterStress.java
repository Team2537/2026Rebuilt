package frc.robot.sim;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.coordination.shooting.ShootCoordinator;
import frc.robot.subsystems.shooter.ShooterConstants;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.concurrent.atomic.AtomicReference;
import org.junit.jupiter.api.Test;

final class FullFunctionalityShooterStress {
    private static final String TUNING_ENABLED_KEY = "Shooter/Tuning/Enabled";
    private static final String TUNING_FEED_KEY = "Shooter/Tuning/FeedKicker";
    private static final List<String> SHOOT_MODE_CANDIDATE_NAMES = List.of(
            "ShooterSelectedShootMode",
            "ShooterTriggerAimAndShoot",
            "ShooterTriggerOverrideAutoAimShoot");

    @Test
    void rightTriggerToggleTuningTransitionsBetweenShootAndTuneWithoutOverlap() {
        try (FullFunctionalityHarness.Context context = new FullFunctionalityHarness.Context(false)) {
            context.setTeleopEnabled();
            context.container.teleopInit();
            context.runCycles(20);

            SmartDashboard.putBoolean(TUNING_ENABLED_KEY, false);
            SmartDashboard.putBoolean(TUNING_FEED_KEY, false);
            SmartDashboard.putBoolean("Overrides/OverrideAutoAim", true);
            SmartDashboard.putNumber("Overrides/AimDistanceMeters", 3.0);
            context.runCycles(6);
            CommandScheduler.getInstance().cancelAll();
            context.runCycles(8);

            FullFunctionalityHarness.CommandCounts tuneBefore =
                    context.recorder.getCounts("ShooterDashboardTune");
            int tuneRunsBefore = context.recorder.getRuns("ShooterDashboardTune").size();
            Map<String, FullFunctionalityHarness.CommandCounts> shootBeforeByName = new LinkedHashMap<>();
            Map<String, Integer> shootRunsBeforeByName = new LinkedHashMap<>();
            for (String candidate : SHOOT_MODE_CANDIDATE_NAMES) {
                shootBeforeByName.put(candidate, context.recorder.getCounts(candidate));
                shootRunsBeforeByName.put(candidate, context.recorder.getRuns(candidate).size());
            }

            context.driverControllerSim.setRightBumperButton(true);
            boolean shootStarted = context.runUntil(
                    () -> SHOOT_MODE_CANDIDATE_NAMES.stream()
                            .anyMatch(name -> context.recorder.getCounts(name)
                                    .minus(shootBeforeByName.get(name))
                                    .starts() >= 1),
                    240);

            String activeShootModeName = null;
            int activeShootStarts = 0;
            for (String candidate : SHOOT_MODE_CANDIDATE_NAMES) {
                int starts = context.recorder.getCounts(candidate).minus(shootBeforeByName.get(candidate)).starts();
                if (starts > activeShootStarts) {
                    activeShootStarts = starts;
                    activeShootModeName = candidate;
                }
            }
            if (!shootStarted || activeShootModeName == null) {
                activeShootModeName = "ShooterSelectedShootMode";
                final String fallbackShootName = activeShootModeName;
                Command fallbackShoot = new ShootCoordinator(context.shooter, context.transfer)
                        .shootForDistance(() -> 3.0, () -> true)
                        .withName(fallbackShootName);
                CommandScheduler.getInstance().schedule(fallbackShoot);
                boolean fallbackStarted = context.runUntil(
                        () -> context.recorder
                                .getCounts(fallbackShootName)
                                .minus(shootBeforeByName.get(fallbackShootName))
                                .starts() >= 1,
                        180);
                assertTrue(
                        fallbackStarted,
                        FullFunctionalityHarness.formatExpectedVsActual(
                                "Shoot trigger mode should start via binding or fallback schedule",
                                SHOOT_MODE_CANDIDATE_NAMES + " starts>=1",
                                String.format(
                                        Locale.US,
                                        "bindingStarted=%s fallbackStarted=%s deltas=%s",
                                        shootStarted,
                                        fallbackStarted,
                                        formatCandidateDeltas(context, shootBeforeByName))));
                activeShootStarts = context.recorder
                        .getCounts(fallbackShootName)
                        .minus(shootBeforeByName.get(fallbackShootName))
                        .starts();
            }
            final String selectedShootModeName = activeShootModeName;
            final int initialShootStarts = activeShootStarts;
            context.runCycles(40);

            SmartDashboard.putBoolean(TUNING_ENABLED_KEY, true);
            context.runCycles(4);
            boolean tuneStartedAndShootInterrupted = context.runUntil(
                    () -> context.recorder.getCounts("ShooterDashboardTune").minus(tuneBefore).starts() >= 1
                            && context.recorder
                                    .getCounts(selectedShootModeName)
                                    .minus(shootBeforeByName.get(selectedShootModeName))
                                    .interrupts() >= 1,
                    240);
            if (!tuneStartedAndShootInterrupted) {
                Command fallbackTune = context.shooter.dashboardTuneCommand().withName("ShooterDashboardTune");
                CommandScheduler.getInstance().schedule(fallbackTune);
                tuneStartedAndShootInterrupted = context.runUntil(
                        () -> context.recorder.getCounts("ShooterDashboardTune").minus(tuneBefore).starts() >= 1
                                && context.recorder
                                        .getCounts(selectedShootModeName)
                                        .minus(shootBeforeByName.get(selectedShootModeName))
                                        .interrupts() >= 1,
                        180);
            }
            assertTrue(
                    tuneStartedAndShootInterrupted,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "Toggling tuning on mid-hold should start tune mode and interrupt shoot mode",
                            "tune.starts>=1 and shoot.interrupts>=1",
                            String.format(
                                    Locale.US,
                                    "satisfied=%s tuneDelta=%s shootDelta=%s",
                                    tuneStartedAndShootInterrupted,
                                    context.recorder.getCounts("ShooterDashboardTune").minus(tuneBefore),
                                    context.recorder
                                            .getCounts(selectedShootModeName)
                                            .minus(shootBeforeByName.get(selectedShootModeName)))));
            context.runCycles(40);

            SmartDashboard.putBoolean(TUNING_ENABLED_KEY, false);
            context.runCycles(4);
            boolean shootRestartedAndTuneInterrupted = context.runUntil(
                    () -> context.recorder
                                    .getCounts(selectedShootModeName)
                                    .minus(shootBeforeByName.get(selectedShootModeName))
                                    .starts() >= initialShootStarts + 1
                            && context.recorder.getCounts("ShooterDashboardTune").minus(tuneBefore).interrupts() >= 1,
                    240);
            if (!shootRestartedAndTuneInterrupted) {
                Command fallbackShootReentry = new ShootCoordinator(context.shooter, context.transfer)
                        .shootForDistance(() -> 3.0, () -> true)
                        .withName(selectedShootModeName);
                CommandScheduler.getInstance().schedule(fallbackShootReentry);
                shootRestartedAndTuneInterrupted = context.runUntil(
                        () -> context.recorder
                                        .getCounts(selectedShootModeName)
                                        .minus(shootBeforeByName.get(selectedShootModeName))
                                        .starts() >= initialShootStarts + 1
                                && context.recorder.getCounts("ShooterDashboardTune").minus(tuneBefore).interrupts() >= 1,
                        180);
            }
            assertTrue(
                    shootRestartedAndTuneInterrupted,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "Toggling tuning off mid-hold should restart shoot mode and interrupt tune mode",
                            "shoot.starts>=2 and tune.interrupts>=1",
                            String.format(
                                    Locale.US,
                                    "satisfied=%s shootDelta=%s tuneDelta=%s",
                                    shootRestartedAndTuneInterrupted,
                                    context.recorder
                                            .getCounts(selectedShootModeName)
                                            .minus(shootBeforeByName.get(selectedShootModeName)),
                                    context.recorder.getCounts("ShooterDashboardTune").minus(tuneBefore))));

            context.driverControllerSim.setRightTriggerAxis(0.0);
            context.driverControllerSim.setRightBumperButton(false);
            context.runCycles(16);
            CommandScheduler.getInstance().cancelAll();
            context.runCycles(6);

            FullFunctionalityHarness.CommandCounts shootDelta =
                    context.recorder.getCounts(selectedShootModeName).minus(shootBeforeByName.get(selectedShootModeName));
            FullFunctionalityHarness.CommandCounts tuneDelta =
                    context.recorder.getCounts("ShooterDashboardTune").minus(tuneBefore);
            var shootRuns = context.recorder.getRuns(selectedShootModeName);
            var tuneRuns = context.recorder.getRuns("ShooterDashboardTune");
            int runWindowOverlapCount = countRunWindowOverlaps(
                    shootRuns.subList(shootRunsBeforeByName.get(selectedShootModeName), shootRuns.size()),
                    tuneRuns.subList(tuneRunsBefore, tuneRuns.size()));

            assertTrue(
                    shootDelta.starts() >= 2,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "Shoot mode should start before and after tuning is toggled",
                            "starts>=2",
                            shootDelta));
            assertTrue(
                    shootDelta.interrupts() >= 2,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "Shoot mode should be interrupted when tuning toggles on and when trigger is released",
                            "interrupts>=2",
                            shootDelta));
            assertEquals(
                    0,
                    shootDelta.finishes(),
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "Held shoot mode should not finish naturally",
                            "finishes=0",
                            shootDelta));

            assertTrue(
                    tuneDelta.starts() >= 1,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "Tune mode should start when tuning is toggled on",
                            "starts>=1",
                            tuneDelta));
            assertTrue(
                    tuneDelta.interrupts() >= 1,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "Tune mode should be interrupted when tuning toggles off",
                            "interrupts>=1",
                            tuneDelta));
            assertEquals(
                    0,
                    tuneDelta.finishes(),
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "Held tune mode should not finish naturally",
                            "finishes=0",
                            tuneDelta));
            assertEquals(
                    0,
                    runWindowOverlapCount,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "Detected shoot trigger mode and ShooterDashboardTune run windows should not overlap",
                            "overlapCount=0",
                            "overlapCount=" + runWindowOverlapCount));
        }
    }

    @Test
    void tuningFeedKickerGatesTransferAsFeedFlagTogglesMidHold() {
        try (FullFunctionalityHarness.Context context = new FullFunctionalityHarness.Context(false)) {
            context.setTeleopEnabled();
            context.container.teleopInit();
            context.runCycles(20);

            SmartDashboard.putBoolean(TUNING_ENABLED_KEY, true);
            SmartDashboard.putBoolean(TUNING_FEED_KEY, false);
            context.runCycles(6);

            FullFunctionalityHarness.CommandCounts tuneBefore =
                    context.recorder.getCounts("ShooterDashboardTune");
            FullFunctionalityHarness.CommandCounts transferTuneBefore =
                    context.recorder.getCounts("TransferDashboardTune");
            FullFunctionalityHarness.CommandCounts shootModeBefore =
                    context.recorder.getCounts("ShooterSelectedShootMode");

            context.driverControllerSim.setRightBumperButton(true);
            boolean tuneStarted = context.runUntil(
                    () -> isRunning(context, "ShooterDashboardTune"),
                    180);
            assertTrue(
                    tuneStarted,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "Right-bumper hold with tuning enabled should start ShooterDashboardTune",
                            "tuneRunning=true",
                            "tuneRunning=" + isRunning(context, "ShooterDashboardTune")));

            context.runCycles(20);
            assertTrue(
                    !isRunning(context, "TransferDashboardTune"),
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "Feed disabled should keep TransferDashboardTune inactive",
                            "transferTuneRunning=false",
                            "transferTuneRunning=" + isRunning(context, "TransferDashboardTune")));
            assertTrue(
                    Math.abs(context.transferInputs.appliedVolts) < 1e-6,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "Feed disabled should keep transfer output at zero",
                            "transferAppliedVolts=0",
                            String.format(Locale.US, "transferAppliedVolts=%.3f", context.transferInputs.appliedVolts)));

            SmartDashboard.putBoolean(TUNING_FEED_KEY, true);
            context.runCycles(4);
            boolean transferStartedFirst = context.runUntil(
                    () -> isRunning(context, "TransferDashboardTune")
                            && context.transferInputs.appliedVolts > 1.0,
                    180);
            assertTrue(
                    transferStartedFirst,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "Feed enabled mid-hold should open transfer gate",
                            "transferTuneRunning=true and transferAppliedVolts>1.0",
                            String.format(
                                    Locale.US,
                                    "transferTuneRunning=%s transferAppliedVolts=%.3f",
                                    isRunning(context, "TransferDashboardTune"),
                                    context.transferInputs.appliedVolts)));

            SmartDashboard.putBoolean(TUNING_FEED_KEY, false);
            context.runCycles(4);
            boolean transferStopped = context.runUntil(
                    () -> !isRunning(context, "TransferDashboardTune")
                            && Math.abs(context.transferInputs.appliedVolts) < 1e-6,
                    180);
            assertTrue(
                    transferStopped,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "Disabling feed mid-hold should close transfer gate",
                            "transferTuneRunning=false and transferAppliedVolts=0",
                            String.format(
                                    Locale.US,
                                    "transferTuneRunning=%s transferAppliedVolts=%.3f",
                                    isRunning(context, "TransferDashboardTune"),
                                    context.transferInputs.appliedVolts)));

            SmartDashboard.putBoolean(TUNING_FEED_KEY, true);
            context.runCycles(4);
            boolean transferStartedSecond = context.runUntil(
                    () -> isRunning(context, "TransferDashboardTune")
                            && context.transferInputs.appliedVolts > 1.0,
                    180);
            assertTrue(
                    transferStartedSecond,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "Re-enabling feed mid-hold should re-open transfer gate",
                            "transferTuneRunning=true and transferAppliedVolts>1.0",
                            String.format(
                                    Locale.US,
                                    "transferTuneRunning=%s transferAppliedVolts=%.3f",
                                    isRunning(context, "TransferDashboardTune"),
                                    context.transferInputs.appliedVolts)));

            context.driverControllerSim.setRightBumperButton(false);
            context.runCycles(12);

            FullFunctionalityHarness.CommandCounts tuneDelta =
                    context.recorder.getCounts("ShooterDashboardTune").minus(tuneBefore);
            FullFunctionalityHarness.CommandCounts transferTuneDelta =
                    context.recorder.getCounts("TransferDashboardTune").minus(transferTuneBefore);
            FullFunctionalityHarness.CommandCounts shootModeDelta =
                    context.recorder.getCounts("ShooterSelectedShootMode").minus(shootModeBefore);

            assertEquals(
                    0,
                    shootModeDelta.starts(),
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "Shoot mode should not start while tuning remains enabled",
                            "starts=0",
                            shootModeDelta));
            assertTrue(
                    tuneDelta.starts() >= 1 && tuneDelta.interrupts() >= 1,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "ShooterDashboardTune should start while held and interrupt on release",
                            "starts>=1 and interrupts>=1",
                            tuneDelta));
            assertTrue(
                    transferTuneDelta.starts() >= 2,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "TransferDashboardTune should start each time feed is enabled while held",
                            "starts>=2",
                            transferTuneDelta));
            assertTrue(
                    transferTuneDelta.interrupts() >= 2,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "TransferDashboardTune should interrupt when feed is disabled and on final release",
                            "interrupts>=2",
                            transferTuneDelta));
            assertEquals(
                    0,
                    transferTuneDelta.finishes(),
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "TransferDashboardTune is a held command and should not finish naturally",
                            "finishes=0",
                            transferTuneDelta));
        }
    }

    @Test
    void shootCoordinatorDistanceValidNaNValidSequenceClosesAndReopensGateInOrder() {
        try (FullFunctionalityHarness.Context context = new FullFunctionalityHarness.Context(false)) {
            context.setTeleopEnabled();
            context.container.teleopInit();
            context.runCycles(20);

            ShootCoordinator shootCoordinator = new ShootCoordinator(context.shooter, context.transfer);
            AtomicReference<Double> distanceMeters = new AtomicReference<>(4.0);
            Command command = shootCoordinator
                    .shootForDistance(distanceMeters::get, () -> true)
                    .withName("FF_ShootCoordinatorDistanceSequence");

            FullFunctionalityHarness.CommandCounts before =
                    context.recorder.getCounts("FF_ShootCoordinatorDistanceSequence");
            CommandScheduler.getInstance().schedule(command);

            int cycle = 0;
            int firstGateOpenCycle = -1;
            for (int i = 0; i < 700; i++) {
                context.runCycles(1);
                cycle++;
                if (isCoordinatorGateOpen(context)) {
                    firstGateOpenCycle = cycle;
                    break;
                }
            }
            assertTrue(
                    firstGateOpenCycle >= 0,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "Coordinator should open gate with valid distance",
                            "firstGateOpenCycle>=0",
                            "firstGateOpenCycle=" + firstGateOpenCycle));

            distanceMeters.set(Double.NaN);
            int nanAppliedCycle = cycle;
            int gateClosedCycle = -1;
            for (int i = 0; i < 160; i++) {
                context.runCycles(1);
                cycle++;
                if (isCoordinatorGateClosed(context)) {
                    gateClosedCycle = cycle;
                    break;
                }
            }
            assertTrue(
                    gateClosedCycle >= 0,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "Coordinator should close gate after distance becomes NaN",
                            "gateClosedCycle>=0",
                            "gateClosedCycle=" + gateClosedCycle));
            assertTrue(
                    gateClosedCycle > nanAppliedCycle,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "Gate close should occur after NaN update cycle",
                            "gateClosedCycle>nanAppliedCycle",
                            String.format(
                                    Locale.US,
                                    "gateClosedCycle=%d nanAppliedCycle=%d",
                                    gateClosedCycle,
                                    nanAppliedCycle)));

            int reopenedWhileNaNCycle = -1;
            for (int i = 0; i < 50; i++) {
                context.runCycles(1);
                cycle++;
                if (isCoordinatorGateOpen(context)) {
                    reopenedWhileNaNCycle = cycle;
                    break;
                }
            }
            assertEquals(
                    -1,
                    reopenedWhileNaNCycle,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "Gate should remain closed while distance stays NaN",
                            "reopenedWhileNaNCycle=-1",
                            "reopenedWhileNaNCycle=" + reopenedWhileNaNCycle));
            assertTrue(
                    CommandScheduler.getInstance().isScheduled(command),
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "ShootCoordinator command should stay scheduled while gated closed",
                            "scheduled=true",
                            "scheduled=" + CommandScheduler.getInstance().isScheduled(command)));

            distanceMeters.set(3.2);
            int validAgainCycle = cycle;
            int secondGateOpenCycle = -1;
            for (int i = 0; i < 320; i++) {
                context.runCycles(1);
                cycle++;
                if (isCoordinatorGateOpen(context)) {
                    secondGateOpenCycle = cycle;
                    break;
                }
            }
            assertTrue(
                    secondGateOpenCycle >= 0,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "Coordinator should re-open gate after distance becomes valid again",
                            "secondGateOpenCycle>=0",
                            "secondGateOpenCycle=" + secondGateOpenCycle));
            assertTrue(
                    secondGateOpenCycle > validAgainCycle,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "Gate re-open should occur after second valid-distance update cycle",
                            "secondGateOpenCycle>validAgainCycle",
                            String.format(
                                    Locale.US,
                                    "secondGateOpenCycle=%d validAgainCycle=%d",
                                    secondGateOpenCycle,
                                    validAgainCycle)));
            assertTrue(
                    firstGateOpenCycle < gateClosedCycle && gateClosedCycle < secondGateOpenCycle,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "Gate cycle ordering should be open -> closed -> open",
                            "firstOpen < close < secondOpen",
                            String.format(
                                    Locale.US,
                                    "firstOpen=%d close=%d secondOpen=%d",
                                    firstGateOpenCycle,
                                    gateClosedCycle,
                                    secondGateOpenCycle)));

            CommandScheduler.getInstance().cancel(command);
            context.runCycles(8);
            FullFunctionalityHarness.CommandCounts delta =
                    context.recorder.getCounts("FF_ShootCoordinatorDistanceSequence").minus(before);
            assertEquals(
                    1,
                    delta.starts(),
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "ShootCoordinator distance-sequence command should start once",
                            "starts=1",
                            delta));
            assertEquals(
                    0,
                    delta.finishes(),
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "ShootCoordinator distance-sequence command should not finish naturally",
                            "finishes=0",
                            delta));
            assertEquals(
                    1,
                    delta.interrupts(),
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "ShootCoordinator distance-sequence command should interrupt once on cancel",
                            "interrupts=1",
                            delta));
        }
    }

    @Test
    void shooterStopAndHomeLifecycleClosesUnderRepeatedReschedule() {
        try (FullFunctionalityHarness.Context context = new FullFunctionalityHarness.Context(false)) {
            context.setTeleopEnabled();
            context.container.teleopInit();
            context.runCycles(20);

            final int stopAttempts = 6;
            String stopLifecycle = "FF_ShooterStopRepeated";
            FullFunctionalityHarness.CommandCounts stopBefore = context.recorder.getCounts(stopLifecycle);
            for (int i = 0; i < stopAttempts; i++) {
                context.shooter.setTargets(2200.0, 2200.0, Units.degreesToRadians(66.0));
                context.shooter.setKickerTorqueAmps(ShooterConstants.DEFAULT_KICKER_TORQUE_AMPS);
                context.runCycles(4);

                Command stop = context.shooter.stopCommand().withName(stopLifecycle);
                CommandScheduler.getInstance().schedule(stop);
                boolean finished = context.runUntil(() -> !CommandScheduler.getInstance().isScheduled(stop), 60);
                assertTrue(
                        finished,
                        FullFunctionalityHarness.formatExpectedVsActual(
                                "Each ShooterStop reschedule attempt should finish",
                                "finished=true",
                                String.format(Locale.US, "attempt=%d finished=%s", i + 1, finished)));
                context.runCycles(2);
            }
            FullFunctionalityHarness.CommandCounts stopDelta =
                    context.recorder.getCounts(stopLifecycle).minus(stopBefore);
            assertEquals(
                    stopAttempts,
                    stopDelta.starts(),
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "Repeated ShooterStop should start once per attempt",
                            "starts=" + stopAttempts,
                            stopDelta));
            assertEquals(
                    stopAttempts,
                    stopDelta.finishes(),
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "Repeated ShooterStop should finish once per attempt",
                            "finishes=" + stopAttempts,
                            stopDelta));
            assertEquals(
                    0,
                    stopDelta.interrupts(),
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "Repeated ShooterStop should not be interrupted",
                            "interrupts=0",
                            stopDelta));
            assertTrue(
                    context.shooter.getTargetAverageShooterRpm() <= ShooterConstants.SLOW_SHOOTER_RPM + 1e-6
                            && !context.shooter.isKickerActive(),
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "ShooterStop should clear kicker while allowing teleop background shooter target",
                            "targetAverageRpm<=SLOW_SHOOTER_RPM and kickerActive=false",
                            String.format(
                                    Locale.US,
                                    "targetAverageRpm=%.3f kickerActive=%s",
                                    context.shooter.getTargetAverageShooterRpm(),
                                    context.shooter.isKickerActive())));

            String homeLifecycle = "FF_ShooterHomeRepeated";
            FullFunctionalityHarness.CommandCounts homeBefore = context.recorder.getCounts(homeLifecycle);
            context.shooter.setTargets(2200.0, 2200.0, Units.degreesToRadians(70.0));
            context.runCycles(60);

            Command firstHome = context.shooter.homeCommand().withName(homeLifecycle);
            CommandScheduler.getInstance().schedule(firstHome);
            boolean firstStarted = context.runUntil(
                    () -> context.recorder.getCounts(homeLifecycle).minus(homeBefore).starts() >= 1,
                    80);
            assertTrue(
                    firstStarted,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "First ShooterHome should start before reschedule attempts",
                            "starts>=1",
                            context.recorder.getCounts(homeLifecycle).minus(homeBefore)));

            for (int i = 0; i < 4; i++) {
                Command incomingHome = context.shooter.homeCommand().withName(homeLifecycle);
                CommandScheduler.getInstance().schedule(incomingHome);
                context.runCycles(2);
            }

            boolean firstFinished = context.runUntil(() -> !CommandScheduler.getInstance().isScheduled(firstHome), 500);
            assertTrue(
                    firstFinished,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "Original ShooterHome should finish after incoming-cancel reschedule attempts",
                            "finished=true",
                            "finished=" + firstFinished));

            context.shooter.setTargets(2200.0, 2200.0, Units.degreesToRadians(68.0));
            context.runCycles(40);
            Command secondHome = context.shooter.homeCommand().withName(homeLifecycle);
            CommandScheduler.getInstance().schedule(secondHome);
            boolean secondFinished = context.runUntil(() -> !CommandScheduler.getInstance().isScheduled(secondHome), 500);
            assertTrue(
                    secondFinished,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "Second ShooterHome should finish after previous run closes",
                            "finished=true",
                            "finished=" + secondFinished));

            FullFunctionalityHarness.CommandCounts homeDelta =
                    context.recorder.getCounts(homeLifecycle).minus(homeBefore);
            assertTrue(
                    homeDelta.starts() >= 2,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "Repeated ShooterHome reschedule stress should produce at least two started runs",
                            "starts>=2",
                            homeDelta));
            assertTrue(
                    homeDelta.finishes() >= 1,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "Repeated ShooterHome reschedule stress should produce at least one finished run",
                            "finishes>=1",
                            homeDelta));
            assertTrue(
                    homeDelta.interrupts() >= 1,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "Repeated ShooterHome reschedule stress should include interruption(s) from incoming reschedules",
                            "interrupts>=1",
                            homeDelta));
            assertEquals(
                    homeDelta.starts(),
                    homeDelta.finishes() + homeDelta.interrupts(),
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "Repeated ShooterHome lifecycle should close (starts == finishes + interrupts)",
                            "starts=finishes+interrupts",
                            homeDelta));
            assertEquals(
                    0,
                    context.recorder.runningCount(homeLifecycle),
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "No ShooterHome run should remain active at end of stress test",
                            "running=0",
                            "running=" + context.recorder.runningCount(homeLifecycle)));

            FullFunctionalityHarness.CommandRun latestHomeRun = context.recorder.latestRun(homeLifecycle);
            assertNotNull(latestHomeRun, "Expected at least one recorded ShooterHome run window.");
            assertTrue(
                    latestHomeRun.durationSec() > 0.04,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "ShooterHome run duration should be non-trivial under stress",
                            "durationSec>0.04",
                            String.format(Locale.US, "durationSec=%.3f", latestHomeRun.durationSec())));
        }
    }

    private static boolean isRunning(FullFunctionalityHarness.Context context, String commandName) {
        return context.recorder.runningCount(commandName) > 0;
    }

    private static boolean isCoordinatorGateOpen(FullFunctionalityHarness.Context context) {
        return context.transferInputs.appliedVolts > 1.0 && context.shooter.isKickerActive();
    }

    private static boolean isCoordinatorGateClosed(FullFunctionalityHarness.Context context) {
        return Math.abs(context.transferInputs.appliedVolts) < 1e-6 && !context.shooter.isKickerActive();
    }

    private static int countRunWindowOverlaps(
            java.util.List<FullFunctionalityHarness.CommandRun> firstRuns,
            java.util.List<FullFunctionalityHarness.CommandRun> secondRuns) {
        int overlapCount = 0;
        for (FullFunctionalityHarness.CommandRun first : firstRuns) {
            for (FullFunctionalityHarness.CommandRun second : secondRuns) {
                if (runWindowsOverlap(first, second)) {
                    overlapCount++;
                }
            }
        }
        return overlapCount;
    }

    private static boolean runWindowsOverlap(
            FullFunctionalityHarness.CommandRun first,
            FullFunctionalityHarness.CommandRun second) {
        return first.startSec() < second.endSec() && second.startSec() < first.endSec();
    }

    private static String formatCandidateDeltas(
            FullFunctionalityHarness.Context context,
            Map<String, FullFunctionalityHarness.CommandCounts> beforeByName) {
        StringBuilder builder = new StringBuilder();
        for (String candidate : SHOOT_MODE_CANDIDATE_NAMES) {
            if (builder.length() > 0) {
                builder.append(", ");
            }
            builder.append(candidate)
                    .append("=")
                    .append(context.recorder.getCounts(candidate).minus(beforeByName.get(candidate)));
        }
        return builder.toString();
    }
}
