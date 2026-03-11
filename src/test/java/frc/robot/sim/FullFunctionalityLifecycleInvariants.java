package frc.robot.sim;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import java.util.HashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import org.junit.jupiter.api.Test;

final class FullFunctionalityLifecycleInvariants {
    private static final double MAX_REASONABLE_RUN_SEC = 8.0;

    @Test
    void broadScenarioMaintainsCommandLifecycleInvariants() {
        try (FullFunctionalityHarness.Context context = new FullFunctionalityHarness.Context(true)) {
            context.setTeleopEnabled();
            context.container.teleopInit();
            context.runCycles(24);

            List<String> keyCommands = List.of(
                    "DriveHoldSlowMode",
                    "DriveToggleFieldOriented",
                    "DriveResetOdometryAndHeading",
                    "DriveHeadingSnap",
                    "IntakeToggleExtended",
                    "DriverIntakeTriggerPress",
                    "IntakeSpinRoller",
                    "DriverManualFeed",
                    "TransferReverse",
                    "DriverIntakeHome",
                    "StopManipulators",
                    "DriveSetOdometryFromUnifiedVision",
                    "DriveSysIdQuasistaticForward",
                    "DriveSysIdQuasistaticReverse",
                    "DriveSysIdDynamicForward",
                    "DriveSysIdDynamicReverse",
                    "DashboardIntakeHome",
                    "DashboardShooterHome",
                    "DashboardStopManipulators",
                    "DashboardDriveResetOdometryAndHeading",
                    "DashboardDriveSetOdometryFromUnifiedVision",
                    "DashboardTransferRun",
                    "AutoNone");

            Map<String, FullFunctionalityHarness.CommandCounts> before = snapshotCounts(context, keyCommands);

            context.driverControllerSim.setLeftStickButton(true);
            context.runCycles(24);
            context.driverControllerSim.setLeftStickButton(false);
            context.runCycles(8);
            context.tapDriverPov(90);
            context.tapDriverButton(context.driverControllerSim::setStartButton);
            context.driverControllerSim.setLeftStickButton(true);
            context.runCycles(4);
            context.driverControllerSim.setRightStickButton(true);
            context.runCycles(4);
            context.driverControllerSim.setRightStickButton(false);
            context.driverControllerSim.setLeftStickButton(false);
            context.runCycles(4);

            context.drive.setPose(new Pose2d(context.drive.getPose().getTranslation(), Rotation2d.fromRadians(0.78)));
            FullFunctionalityHarness.CommandCounts snapBefore = before.get("DriveHeadingSnap");
            context.tapDriverPov(0);
            boolean headingSnapFinished = context.runUntil(
                    () -> context.recorder.getCounts("DriveHeadingSnap").minus(snapBefore).finishes() >= 1,
                    420);
            assertTrue(
                    headingSnapFinished,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "Driver heading-snap sequence should finish within scripted scenario",
                            "finished=true",
                            String.format(
                                    Locale.US,
                                    "finished=%s delta=%s",
                                    headingSnapFinished,
                                    context.recorder.getCounts("DriveHeadingSnap").minus(snapBefore))));

            context.tapDriverButton(context.driverControllerSim::setBButton);
            context.runCycles(120);
            context.tapDriverButton(context.driverControllerSim::setBButton);
            context.runCycles(120);

            context.driverControllerSim.setXButton(true);
            context.runCycles(60);
            context.driverControllerSim.setXButton(false);
            context.runCycles(20);

            context.driverControllerSim.setLeftTriggerAxis(1.0);
            context.runCycles(36);
            context.driverControllerSim.setLeftTriggerAxis(0.0);
            context.runCycles(10);
            context.driverControllerSim.setLeftTriggerAxis(1.0);
            context.runCycles(4);
            context.driverControllerSim.setLeftTriggerAxis(0.0);
            context.runCycles(4);
            context.driverControllerSim.setLeftTriggerAxis(1.0);
            context.runCycles(4);
            context.driverControllerSim.setLeftTriggerAxis(0.0);
            context.runCycles(20);

            context.driverControllerSim.setYButton(true);
            context.runCycles(36);
            context.driverControllerSim.setYButton(false);
            context.runCycles(10);

            context.tapDriverPov(180);
            context.tapDriverPov(270);

            context.tapGodButton(context.godControllerSim::setLeftBumperButton);
            context.tapGodPov(180);
            holdGodButton(context, context.godControllerSim::setAButton, 40, 8);
            holdGodButton(context, context.godControllerSim::setBButton, 40, 8);
            holdGodButton(context, context.godControllerSim::setXButton, 40, 8);
            holdGodButton(context, context.godControllerSim::setYButton, 40, 8);

            runDashboardOneShot(context, "Actions/IntakeHome", "DashboardIntakeHome");
            runDashboardOneShot(context, "Actions/ShooterHome", "DashboardShooterHome");
            runDashboardOneShot(context, "Actions/StopManipulators", "DashboardStopManipulators");
            runDashboardOneShot(
                    context,
                    "Actions/DriveResetOdometryAndHeading",
                    "DashboardDriveResetOdometryAndHeading");
            runDashboardOneShot(
                    context,
                    "Actions/DriveSetOdometryFromUnifiedVision",
                    "DashboardDriveSetOdometryFromUnifiedVision");
            runDashboardHeldAndCancel(context, "Actions/TransferRun", "DashboardTransferRun", 40);

            context.setAutonomousEnabled();
            context.container.autonomousInit();
            context.runCycles(30);

            context.clearControllerState();
            context.runCycles(20);

            List<String> finiteMustFinish = List.of(
                    "DriveToggleFieldOriented",
                    "DriveResetOdometryAndHeading",
                    "DriveHeadingSnap",
                    "IntakeToggleExtended",
                    "DriveSetOdometryFromUnifiedVision",
                    "DashboardIntakeHome",
                    "DashboardShooterHome",
                    "DashboardDriveResetOdometryAndHeading",
                    "DashboardDriveSetOdometryFromUnifiedVision",
                    "AutoNone");

            List<String> heldMustInterrupt = List.of(
                    "DriveHoldSlowMode",
                    "IntakeSpinRoller",
                    "DriverManualFeed",
                    "TransferReverse",
                    "DriveSysIdQuasistaticForward",
                    "DriveSysIdQuasistaticReverse",
                    "DriveSysIdDynamicForward",
                    "DriveSysIdDynamicReverse",
                    "DashboardTransferRun");

            for (String name : keyCommands) {
                FullFunctionalityHarness.CommandCounts delta = context.recorder.getCounts(name).minus(before.get(name));
                if (delta.starts() == 0) {
                    continue;
                }
                assertLifecycleClosed(name, delta);
            }

            for (String name : finiteMustFinish) {
                FullFunctionalityHarness.CommandCounts delta = context.recorder.getCounts(name).minus(before.get(name));
                assertFiniteCompletes(name, delta);
            }

            for (String name : heldMustInterrupt) {
                FullFunctionalityHarness.CommandCounts delta = context.recorder.getCounts(name).minus(before.get(name));
                assertHeldInterrupts(name, delta);
            }

            for (String name : keyCommands) {
                FullFunctionalityHarness.CommandCounts delta = context.recorder.getCounts(name).minus(before.get(name));
                if (delta.starts() == 0) {
                    continue;
                }
                assertRunDurationsReasonable(context, name, MAX_REASONABLE_RUN_SEC);
            }
        }
    }

    private static Map<String, FullFunctionalityHarness.CommandCounts> snapshotCounts(
            FullFunctionalityHarness.Context context,
            List<String> commandNames) {
        Map<String, FullFunctionalityHarness.CommandCounts> byName = new HashMap<>();
        for (String name : commandNames) {
            byName.put(name, context.recorder.getCounts(name));
        }
        return byName;
    }

    private static void holdGodButton(
            FullFunctionalityHarness.Context context,
            java.util.function.Consumer<Boolean> buttonSetter,
            int holdCycles,
            int settleCyclesAfterRelease) {
        buttonSetter.accept(true);
        context.runCycles(holdCycles);
        buttonSetter.accept(false);
        context.runCycles(settleCyclesAfterRelease);
    }

    private static void runDashboardOneShot(
            FullFunctionalityHarness.Context context,
            String dashboardKey,
            String lifecycleName) {
        Command command = FullFunctionalityHarness.getDashboardCommand(dashboardKey);
        FullFunctionalityHarness.CommandCounts before = context.recorder.getCounts(lifecycleName);
        CommandScheduler.getInstance().schedule(command);
        boolean finished = context.runUntil(() -> !CommandScheduler.getInstance().isScheduled(command), 700);
        if (!finished) {
            CommandScheduler.getInstance().cancel(command);
            context.runCycles(8);
        }

        FullFunctionalityHarness.CommandCounts delta = context.recorder.getCounts(lifecycleName).minus(before);
        assertTrue(
                delta.starts() >= 1,
                FullFunctionalityHarness.formatExpectedVsActual(
                        "Dashboard one-shot should start",
                        lifecycleName + " starts>=1",
                        delta));
        assertTrue(
                delta.finishes() >= 1,
                FullFunctionalityHarness.formatExpectedVsActual(
                        "Dashboard one-shot should finish",
                        lifecycleName + " finishes>=1",
                        delta));
        assertEquals(
                0,
                delta.interrupts(),
                FullFunctionalityHarness.formatExpectedVsActual(
                        "Dashboard one-shot should not interrupt",
                        lifecycleName + " interrupts=0",
                        delta));
    }

    private static void runDashboardHeldAndCancel(
            FullFunctionalityHarness.Context context,
            String dashboardKey,
            String lifecycleName,
            int holdCycles) {
        Command command = FullFunctionalityHarness.getDashboardCommand(dashboardKey);
        FullFunctionalityHarness.CommandCounts before = context.recorder.getCounts(lifecycleName);
        CommandScheduler.getInstance().schedule(command);
        context.runCycles(holdCycles);
        CommandScheduler.getInstance().cancel(command);
        context.runCycles(10);

        FullFunctionalityHarness.CommandCounts delta = context.recorder.getCounts(lifecycleName).minus(before);
        assertTrue(
                delta.starts() >= 1,
                FullFunctionalityHarness.formatExpectedVsActual(
                        "Dashboard held command should start",
                        lifecycleName + " starts>=1",
                        delta));
        assertTrue(
                delta.interrupts() >= 1,
                FullFunctionalityHarness.formatExpectedVsActual(
                        "Dashboard held command should interrupt on cancel",
                        lifecycleName + " interrupts>=1",
                        delta));
    }

    private static void assertLifecycleClosed(String commandName, FullFunctionalityHarness.CommandCounts delta) {
        int ends = delta.finishes() + delta.interrupts();
        assertEquals(
                delta.starts(),
                ends,
                FullFunctionalityHarness.formatExpectedVsActual(
                        "Lifecycle should close for key command",
                        commandName + " starts==finishes+interrupts",
                        String.format(Locale.US, "%s ends=%d", delta, ends)));
    }

    private static void assertFiniteCompletes(String commandName, FullFunctionalityHarness.CommandCounts delta) {
        assertTrue(
                delta.starts() >= 1,
                FullFunctionalityHarness.formatExpectedVsActual(
                        "Finite command should be exercised in scenario",
                        commandName + " starts>=1",
                        delta));
        assertTrue(
                delta.finishes() >= 1,
                FullFunctionalityHarness.formatExpectedVsActual(
                        "Finite command expected to complete should finish at least once",
                        commandName + " finishes>=1",
                        delta));
    }

    private static void assertHeldInterrupts(String commandName, FullFunctionalityHarness.CommandCounts delta) {
        assertTrue(
                delta.starts() >= 1,
                FullFunctionalityHarness.formatExpectedVsActual(
                        "Held command should be exercised in scenario",
                        commandName + " starts>=1",
                        delta));
        assertTrue(
                delta.interrupts() >= 1,
                FullFunctionalityHarness.formatExpectedVsActual(
                        "Held command should interrupt at least once on release/cancel",
                        commandName + " interrupts>=1",
                        delta));
    }

    private static void assertRunDurationsReasonable(
            FullFunctionalityHarness.Context context,
            String commandName,
            double maxReasonableRunSec) {
        List<FullFunctionalityHarness.CommandRun> runs = context.recorder.getRuns(commandName);
        assertTrue(
                !runs.isEmpty(),
                FullFunctionalityHarness.formatExpectedVsActual(
                        "Command with starts should record run windows",
                        commandName + " runs.size()>0",
                        "runs.size()=" + runs.size()));

        double maxObservedDurationSec = 0.0;
        for (FullFunctionalityHarness.CommandRun run : runs) {
            maxObservedDurationSec = Math.max(maxObservedDurationSec, run.durationSec());
            assertTrue(
                    run.durationSec() >= -1e-9,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "Run durations should be non-negative",
                            commandName + " duration>=0",
                            String.format(Locale.US, "duration=%.6fs", run.durationSec())));
        }

        assertTrue(
                maxObservedDurationSec <= maxReasonableRunSec,
                FullFunctionalityHarness.formatExpectedVsActual(
                        "No command run should be absurdly long in this short scripted scenario",
                        String.format(Locale.US, "%s maxDuration<=%.3fs", commandName, maxReasonableRunSec),
                        String.format(Locale.US, "%s maxDuration=%.3fs", commandName, maxObservedDurationSec)));
    }
}
