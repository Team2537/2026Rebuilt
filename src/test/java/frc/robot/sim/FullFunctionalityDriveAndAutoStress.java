package frc.robot.sim;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.autos.AutoRoutines;
import frc.robot.subsystems.drive.DriveConstants;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import org.junit.jupiter.api.Test;

final class FullFunctionalityDriveAndAutoStress {
    private static final int AUTO_MAX_CYCLES = 4500;
    private static final double MAX_ELAPSED_DRIFT_SEC = 0.16;
    private static final double MAX_FINAL_TRANSLATION_DRIFT_METERS = 0.03;
    private static final double MAX_FINAL_HEADING_DRIFT_DEG = 6.0;
    private static final double MAX_DISTANCE_DRIFT_METERS = 0.12;

    @Test
    void defaultDriveCommandRecoversAfterHeadingSnapAndStopWithXInterruption() {
        try (FullFunctionalityHarness.Context context = new FullFunctionalityHarness.Context(false)) {
            context.setTeleopEnabled();
            context.container.teleopInit();
            context.runCycles(20);

            assertTrue(
                    context.recorder.runningCount("DriveJoystickDefault") >= 1,
                    "Expected DriveJoystickDefault to be active before interruption stress test.");

            context.driverControllerSim.setLeftY(0.35);
            context.driverControllerSim.setLeftX(-0.2);
            context.runCycles(12);

            context.drive.setPose(new Pose2d(context.drive.getPose().getTranslation(), Rotation2d.fromRadians(0.72)));
            FullFunctionalityHarness.CommandCounts snapBefore = context.recorder.getCounts("DriveHeadingSnap");
            FullFunctionalityHarness.CommandCounts driveBeforeSnap = context.recorder.getCounts("DriveJoystickDefault");

            context.tapDriverPov(0);
            boolean snapFinished = context.runUntil(
                    () -> context.recorder.getCounts("DriveHeadingSnap").minus(snapBefore).finishes() >= 1,
                    360);
            assertTrue(
                    snapFinished,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "DriveHeadingSnap should finish during interruption stress test",
                            "finished=true",
                            String.format(
                                    Locale.US,
                                    "finished=%s delta=%s",
                                    snapFinished,
                                    context.recorder.getCounts("DriveHeadingSnap").minus(snapBefore))));

            context.runCycles(8);
            FullFunctionalityHarness.CommandCounts snapDelta =
                    context.recorder.getCounts("DriveHeadingSnap").minus(snapBefore);
            FullFunctionalityHarness.CommandCounts driveSnapDelta =
                    context.recorder.getCounts("DriveJoystickDefault").minus(driveBeforeSnap);

            assertTrue(
                    snapDelta.starts() >= 1,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "Heading snap should initialize at least once",
                            "starts>=1",
                            snapDelta));
            assertTrue(
                    snapDelta.finishes() >= 1,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "Heading snap should finish at least once",
                            "finishes>=1",
                            snapDelta));
            assertTrue(
                    driveSnapDelta.interrupts() >= 1,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "Drive default should be interrupted by heading snap",
                            "interrupts>=1",
                            driveSnapDelta));
            assertTrue(
                    driveSnapDelta.starts() >= 1,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "Drive default should restart after heading snap",
                            "starts>=1",
                            driveSnapDelta));
            assertTrue(
                    context.recorder.runningCount("DriveJoystickDefault") >= 1,
                    "Expected DriveJoystickDefault to recover and run after heading snap.");

            context.clearControllerState();
            context.runCycles(10);

            String stopLifecycleName = "FF_DriveStopWithXStress";
            Command stopWithX = FullFunctionalityHarness.namedCommand("DriveStopWithX").withName(stopLifecycleName);
            FullFunctionalityHarness.CommandCounts stopBefore = context.recorder.getCounts(stopLifecycleName);
            FullFunctionalityHarness.CommandCounts driveBeforeStop = context.recorder.getCounts("DriveJoystickDefault");

            CommandScheduler.getInstance().schedule(stopWithX);
            boolean stopFinished = context.runUntil(() -> !CommandScheduler.getInstance().isScheduled(stopWithX), 80);
            assertTrue(
                    stopFinished,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "DriveStopWithX should finish as one-shot command",
                            "finished=true",
                            String.format(Locale.US, "finished=%s", stopFinished)));
            context.runCycles(8);

            FullFunctionalityHarness.CommandCounts stopDelta =
                    context.recorder.getCounts(stopLifecycleName).minus(stopBefore);
            FullFunctionalityHarness.CommandCounts driveStopDelta =
                    context.recorder.getCounts("DriveJoystickDefault").minus(driveBeforeStop);

            assertEquals(
                    1,
                    stopDelta.starts(),
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "DriveStopWithX wrapper should start exactly once",
                            "starts=1",
                            stopDelta));
            assertEquals(
                    1,
                    stopDelta.finishes(),
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "DriveStopWithX wrapper should finish exactly once",
                            "finishes=1",
                            stopDelta));
            assertEquals(
                    0,
                    stopDelta.interrupts(),
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "DriveStopWithX wrapper should not be interrupted",
                            "interrupts=0",
                            stopDelta));
            assertTrue(
                    driveStopDelta.interrupts() >= 1,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "Drive default should be interrupted by DriveStopWithX",
                            "interrupts>=1",
                            driveStopDelta));
            assertTrue(
                    driveStopDelta.starts() >= 1,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "Drive default should restart after DriveStopWithX",
                            "starts>=1",
                            driveStopDelta));
            assertTrue(
                    context.recorder.runningCount("DriveJoystickDefault") >= 1,
                    "Expected DriveJoystickDefault to recover and run after DriveStopWithX.");

            ChassisSpeeds settledSpeeds = context.drive.getMeasuredChassisSpeeds();
            double settledLinearNorm = Math.hypot(settledSpeeds.vxMetersPerSecond, settledSpeeds.vyMetersPerSecond);
            assertTrue(
                    settledLinearNorm <= 0.25 && Math.abs(settledSpeeds.omegaRadiansPerSecond) <= 0.45,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "DriveStopWithX stress case should leave chassis near zero measured speed",
                            "linear<=0.25 m/s and |omega|<=0.45 rad/s",
                            String.format(
                                    Locale.US,
                                    "linear=%.3f omega=%.3f",
                                    settledLinearNorm,
                                    settledSpeeds.omegaRadiansPerSecond)));
        }
    }

    @Test
    void headingSnapFinishesWhenAlreadyInsideControllerToleranceBand() {
        try (FullFunctionalityHarness.Context context = new FullFunctionalityHarness.Context(false)) {
            context.setTeleopEnabled();
            context.container.teleopInit();
            context.runCycles(20);

            context.drive.setPose(new Pose2d(
                    context.drive.getPose().getTranslation(),
                    Rotation2d.fromDegrees(1.5)));
            FullFunctionalityHarness.CommandCounts snapBefore = context.recorder.getCounts("DriveHeadingSnap");

            context.tapDriverPov(0);
            boolean snapFinished = context.runUntil(
                    () -> context.recorder.getCounts("DriveHeadingSnap").minus(snapBefore).finishes() >= 1,
                    20);

            assertTrue(
                    snapFinished,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "DriveHeadingSnap should finish when already within controller tolerance",
                            "finished=true",
                            String.format(
                                    Locale.US,
                                    "finished=%s delta=%s",
                                    snapFinished,
                                    context.recorder.getCounts("DriveHeadingSnap").minus(snapBefore))));
            assertTrue(
                    context.recorder.runningCount("DriveJoystickDefault") >= 1,
                    "Expected DriveJoystickDefault to be running after a near-cardinal heading snap.");
        }
    }

    @Test
    void trackedPathPlannerAutosRemainRepeatableAcrossTwoRunsPerAuto() {
        try (FullFunctionalityHarness.Context context = new FullFunctionalityHarness.Context(false)) {
            context.setAutonomousEnabled();
            context.runCycles(6);

            List<String> autoNames = AutoRoutines.create(context.drive).stream()
                    .map(AutoRoutines.AutoRoutine::name)
                    .filter(name -> name.startsWith("pp/"))
                    .map(name -> name.substring("pp/".length()))
                    .filter(FullFunctionalityAutoGoldens::isTrackedAuto)
                    .sorted()
                    .toList();
            assertTrue(!autoNames.isEmpty(), "Expected at least one PathPlanner auto routine.");

            List<String> failures = new ArrayList<>();
            for (String autoName : autoNames) {
                if (FullFunctionalityAutoGoldens.isUntrackedCompAuto(autoName)) {
                    continue;
                }
                AutoRunMetrics runOne = runPathPlannerAutoOnce(context, autoName, 1);
                AutoRunMetrics runTwo = runPathPlannerAutoOnce(context, autoName, 2);

                if (!runOne.finished()) {
                    failures.add(FullFunctionalityHarness.formatExpectedVsActual(
                            "PathPlanner auto run #1 should finish",
                            autoName + " finished=true",
                            autoName + " finished=false"));
                }
                if (!runTwo.finished()) {
                    failures.add(FullFunctionalityHarness.formatExpectedVsActual(
                            "PathPlanner auto run #2 should finish",
                            autoName + " finished=true",
                            autoName + " finished=false"));
                }
                if (!runOne.lifecycleClosedCleanly()) {
                    failures.add(FullFunctionalityHarness.formatExpectedVsActual(
                            "PathPlanner auto run #1 lifecycle should close cleanly",
                            "starts=1 finishes=1 interrupts=0",
                            autoName + " " + runOne.deltaCounts()));
                }
                if (!runTwo.lifecycleClosedCleanly()) {
                    failures.add(FullFunctionalityHarness.formatExpectedVsActual(
                            "PathPlanner auto run #2 lifecycle should close cleanly",
                            "starts=1 finishes=1 interrupts=0",
                            autoName + " " + runTwo.deltaCounts()));
                }

                double elapsedDriftSec = Math.abs(runOne.elapsedSec() - runTwo.elapsedSec());
                double finalTranslationDriftMeters = runOne.finalPose()
                        .getTranslation()
                        .getDistance(runTwo.finalPose().getTranslation());
                double finalHeadingDriftDeg = Math.abs(Math.toDegrees(MathUtil.angleModulus(
                        runOne.finalPose()
                                .getRotation()
                                .minus(runTwo.finalPose().getRotation())
                                .getRadians())));
                double distanceDriftMeters = Math.abs(runOne.distanceMeters() - runTwo.distanceMeters());

                if (elapsedDriftSec > MAX_ELAPSED_DRIFT_SEC) {
                    failures.add(FullFunctionalityHarness.formatExpectedVsActual(
                            "Auto elapsed-time drift between run #1 and run #2 should be very small",
                            String.format(Locale.US, "|drift|<=%.3fs", MAX_ELAPSED_DRIFT_SEC),
                            String.format(Locale.US, "auto=%s drift=%.3fs", autoName, elapsedDriftSec)));
                }
                if (finalTranslationDriftMeters > MAX_FINAL_TRANSLATION_DRIFT_METERS) {
                    failures.add(FullFunctionalityHarness.formatExpectedVsActual(
                            "Auto final-translation drift between run #1 and run #2 should be very small",
                            String.format(Locale.US, "|drift|<=%.3fm", MAX_FINAL_TRANSLATION_DRIFT_METERS),
                            String.format(
                                    Locale.US,
                                    "auto=%s drift=%.3fm",
                                    autoName,
                                    finalTranslationDriftMeters)));
                }
                if (finalHeadingDriftDeg > MAX_FINAL_HEADING_DRIFT_DEG) {
                    failures.add(FullFunctionalityHarness.formatExpectedVsActual(
                            "Auto final-heading drift between run #1 and run #2 should be very small",
                            String.format(Locale.US, "|drift|<=%.2fdeg", MAX_FINAL_HEADING_DRIFT_DEG),
                            String.format(Locale.US, "auto=%s drift=%.3fdeg", autoName, finalHeadingDriftDeg)));
                }
                if (distanceDriftMeters > MAX_DISTANCE_DRIFT_METERS) {
                    failures.add(FullFunctionalityHarness.formatExpectedVsActual(
                            "Auto integrated-distance drift between run #1 and run #2 should be very small",
                            String.format(Locale.US, "|drift|<=%.3fm", MAX_DISTANCE_DRIFT_METERS),
                            String.format(Locale.US, "auto=%s drift=%.3fm", autoName, distanceDriftMeters)));
                }
            }

            assertTrue(
                    failures.isEmpty(),
                    "PathPlanner repeatability stress mismatches:\n" + String.join("\n", failures));
        }
    }

    @Test
    void teleopInitInterruptsActiveAutoAndRestoresTeleopDefaultsAndHoming() {
        try (FullFunctionalityHarness.Context context = new FullFunctionalityHarness.Context(false)) {
            context.setAutonomousEnabled();
            context.container.autonomousInit();
            context.runCycles(10);

            List<String> autoNames = AutoRoutines.create(context.drive).stream()
                    .map(AutoRoutines.AutoRoutine::name)
                    .filter(name -> name.startsWith("pp/"))
                    .map(name -> name.substring("pp/".length()))
                    .sorted()
                    .toList();
            assertTrue(!autoNames.isEmpty(), "Expected at least one PathPlanner auto routine.");

            ActiveAuto activeAuto = scheduleActivePathPlannerAuto(context, autoNames);
            assertNotNull(activeAuto, "Expected at least one PathPlanner auto to remain active for transition stress.");
            assertTrue(
                    CommandScheduler.getInstance().isScheduled(activeAuto.command()),
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "Transition stress precondition should hold active auto before teleopInit",
                            "scheduled=true",
                            String.format(
                                    Locale.US,
                                    "auto=%s scheduled=%s",
                                    activeAuto.autoName(),
                                    CommandScheduler.getInstance().isScheduled(activeAuto.command()))));

            FullFunctionalityHarness.CommandCounts autoBefore =
                    context.recorder.getCounts(activeAuto.lifecycleName());
            FullFunctionalityHarness.CommandCounts driveBefore = context.recorder.getCounts("DriveJoystickDefault");
            FullFunctionalityHarness.CommandCounts intakeHomeBefore =
                    context.recorder.getCounts("IntakeHome");
            FullFunctionalityHarness.CommandCounts shooterHomeBefore =
                    context.recorder.getCounts("ShooterHome");

            context.setTeleopEnabled();
            context.container.teleopInit();
            context.runCycles(30);

            FullFunctionalityHarness.CommandCounts autoDelta =
                    context.recorder.getCounts(activeAuto.lifecycleName()).minus(autoBefore);
            FullFunctionalityHarness.CommandCounts driveDelta =
                    context.recorder.getCounts("DriveJoystickDefault").minus(driveBefore);
            FullFunctionalityHarness.CommandCounts intakeHomeDelta =
                    context.recorder.getCounts("IntakeHome").minus(intakeHomeBefore);
            FullFunctionalityHarness.CommandCounts shooterHomeDelta =
                    context.recorder.getCounts("ShooterHome").minus(shooterHomeBefore);

            assertTrue(
                    autoDelta.interrupts() >= 1,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "teleopInit should interrupt an active auto command",
                            "interrupts>=1",
                            String.format(Locale.US, "auto=%s %s", activeAuto.autoName(), autoDelta)));
            assertTrue(
                    !CommandScheduler.getInstance().isScheduled(activeAuto.command()),
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "Active auto command should be unscheduled after teleopInit",
                            "scheduled=false",
                            String.format(
                                    Locale.US,
                                    "auto=%s scheduled=%s",
                                    activeAuto.autoName(),
                                    CommandScheduler.getInstance().isScheduled(activeAuto.command()))));
            assertTrue(
                    driveDelta.starts() >= 1,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "teleopInit should restart DriveJoystickDefault",
                            "starts>=1",
                            driveDelta));
            assertTrue(
                    context.recorder.runningCount("DriveJoystickDefault") >= 1,
                    "Expected DriveJoystickDefault to be running after teleop transition.");
            assertTrue(
                    intakeHomeDelta.starts() >= 1,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "teleopInit should schedule IntakeHome while transitioning from auto",
                            "starts>=1",
                            intakeHomeDelta));
            assertTrue(
                    shooterHomeDelta.starts() >= 1,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "teleopInit should schedule ShooterHome while transitioning from auto",
                            "starts>=1",
                            shooterHomeDelta));
            assertTrue(
                    context.recorder.runningCount("IntakeHome") >= 1
                            || context.recorder.runningCount("IntakeBackground") >= 1,
                    "Expected intake to be active via IntakeHome or IntakeBackground after teleop transition.");
            assertTrue(
                    context.recorder.runningCount("ShooterHome") >= 1
                            || context.recorder.runningCount("ShooterBackground") >= 1,
                    "Expected shooter to be active via ShooterHome or ShooterBackground after teleop transition.");
        }
    }

    @Test
    void fieldOrientedPersistsButSlowModeClearsAcrossRapidDisableEnableCycles() {
        try (FullFunctionalityHarness.Context context = new FullFunctionalityHarness.Context(false)) {
            context.setTeleopEnabled();
            context.container.teleopInit();
            context.runCycles(20);

            assertTrue(context.drive.isFieldOriented(), "Expected field-oriented mode to start enabled.");
            assertTrue(
                    !context.drive.isConstraintProfileActive(DriveConstants.ConstraintProfile.SLOW_MODE),
                    "Expected slow mode to start disabled.");

            context.tapDriverPov(90);
            context.driverControllerSim.setLeftStickButton(true);
            context.runCycles(10);
            assertTrue(
                    !context.drive.isFieldOriented(),
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "POV right precondition should set field-oriented mode false",
                            "fieldOriented=false",
                            "fieldOriented=" + context.drive.isFieldOriented()));
            assertTrue(
                    context.drive.isConstraintProfileActive(DriveConstants.ConstraintProfile.SLOW_MODE),
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "Left stick hold precondition should set slow mode true",
                            "slowMode=true",
                            "slowMode=" + context.drive.isConstraintProfileActive(
                                    DriveConstants.ConstraintProfile.SLOW_MODE)));

            FullFunctionalityHarness.CommandCounts fieldToggleBefore =
                    context.recorder.getCounts("DriveToggleFieldOriented");

            for (int i = 0; i < 8; i++) {
                context.setDisabled();
                context.container.disabledInit();
                context.runCycles(6);
                assertTrue(
                        !context.drive.isFieldOriented(),
                        FullFunctionalityHarness.formatExpectedVsActual(
                                "Field-oriented setting should persist through disabled cycle",
                                "fieldOriented=false",
                                String.format(
                                        Locale.US,
                                        "cycle=%d fieldOriented=%s",
                                        i,
                                        context.drive.isFieldOriented())));
                context.setTeleopEnabled();
                context.container.teleopInit();
                context.runCycles(10);
                assertTrue(
                        !context.drive.isFieldOriented(),
                        FullFunctionalityHarness.formatExpectedVsActual(
                                "Field-oriented setting should persist through rapid re-enable",
                                "fieldOriented=false",
                                String.format(
                                        Locale.US,
                                        "cycle=%d fieldOriented=%s",
                                        i,
                                        context.drive.isFieldOriented())));
                assertTrue(
                        !context.drive.isConstraintProfileActive(DriveConstants.ConstraintProfile.SLOW_MODE),
                        FullFunctionalityHarness.formatExpectedVsActual(
                                "teleopInit should clear slow mode even if left stick remains held through re-enable",
                                "slowMode=false",
                                String.format(
                                        Locale.US,
                                        "cycle=%d slowMode=%s leftStickHeld=true",
                                        i,
                                        context.drive.isConstraintProfileActive(
                                                DriveConstants.ConstraintProfile.SLOW_MODE))));
                assertTrue(
                        context.recorder.runningCount("DriveJoystickDefault") >= 1,
                        "Expected DriveJoystickDefault to be running after each re-enable.");
                assertTrue(
                        context.recorder.runningCount("IntakeHome") >= 1
                                || context.recorder.runningCount("IntakeBackground") >= 1,
                        "Expected intake to be running via IntakeHome or IntakeBackground after each re-enable.");
                assertTrue(
                        context.recorder.runningCount("ShooterHome") >= 1
                                || context.recorder.runningCount("ShooterBackground") >= 1,
                        "Expected shooter to be running via ShooterHome or ShooterBackground after each re-enable.");
            }

            FullFunctionalityHarness.CommandCounts fieldToggleDelta =
                    context.recorder.getCounts("DriveToggleFieldOriented").minus(fieldToggleBefore);

            assertEquals(
                    0,
                    fieldToggleDelta.starts(),
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "Disable/enable cycling should not implicitly toggle field-oriented state",
                            "starts=0",
                            fieldToggleDelta));
            context.driverControllerSim.setLeftStickButton(false);
            context.runCycles(8);
            assertTrue(
                    !context.drive.isConstraintProfileActive(DriveConstants.ConstraintProfile.SLOW_MODE),
                    "Expected releasing left stick after the cycle stress test to disable slow mode.");
        }
    }

    private static AutoRunMetrics runPathPlannerAutoOnce(
            FullFunctionalityHarness.Context context,
            String autoName,
            int runIndex) {
        context.clearControllerState();
        context.drive.setPose(Pose2d.kZero);
        CommandScheduler.getInstance().cancelAll();
        PathPlannerAuto.currentPathName = null;
        context.runCycles(8);

        String lifecycleName = "FF_Repeatability_" + sanitizeForLifecycle(autoName) + "_Run" + runIndex;
        Command auto = new PathPlannerAuto(autoName).withName(lifecycleName);
        FullFunctionalityHarness.CommandCounts before = context.recorder.getCounts(lifecycleName);

        CommandScheduler.getInstance().schedule(auto);
        Pose2d previousPose = context.drive.getPose();
        double distanceMeters = 0.0;
        int completedCycles = 0;
        boolean finished = false;

        for (int i = 0; i < AUTO_MAX_CYCLES; i++) {
            context.runCycles(1);
            completedCycles = i + 1;

            Pose2d currentPose = context.drive.getPose();
            distanceMeters += currentPose.getTranslation().getDistance(previousPose.getTranslation());
            previousPose = currentPose;

            if (!CommandScheduler.getInstance().isScheduled(auto)) {
                finished = true;
                break;
            }
        }
        if (!finished) {
            CommandScheduler.getInstance().cancel(auto);
            context.runCycles(10);
        }

        return new AutoRunMetrics(
                autoName,
                finished,
                completedCycles * FullFunctionalityHarness.LOOP_PERIOD_SEC,
                distanceMeters,
                context.drive.getPose(),
                context.recorder.getCounts(lifecycleName).minus(before));
    }

    private static ActiveAuto scheduleActivePathPlannerAuto(
            FullFunctionalityHarness.Context context,
            List<String> autoNames) {
        CommandScheduler scheduler = CommandScheduler.getInstance();
        for (String autoName : autoNames) {
            scheduler.cancelAll();
            context.runCycles(6);

            String lifecycleName = "FF_TeleopTransition_" + sanitizeForLifecycle(autoName);
            Command auto = new PathPlannerAuto(autoName).withName(lifecycleName);
            scheduler.schedule(auto);
            context.runCycles(6);
            if (scheduler.isScheduled(auto)) {
                return new ActiveAuto(autoName, lifecycleName, auto);
            }
        }
        return null;
    }

    private static String sanitizeForLifecycle(String raw) {
        return raw.replaceAll("[^A-Za-z0-9]+", "_");
    }

    private record AutoRunMetrics(
            String autoName,
            boolean finished,
            double elapsedSec,
            double distanceMeters,
            Pose2d finalPose,
            FullFunctionalityHarness.CommandCounts deltaCounts) {
        boolean lifecycleClosedCleanly() {
            return deltaCounts.starts() == 1 && deltaCounts.finishes() == 1 && deltaCounts.interrupts() == 0;
        }
    }

    private record ActiveAuto(String autoName, String lifecycleName, Command command) {}
}
