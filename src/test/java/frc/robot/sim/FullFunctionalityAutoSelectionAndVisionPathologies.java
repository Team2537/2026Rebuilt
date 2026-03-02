package frc.robot.sim;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import java.io.IOException;
import java.lang.reflect.Field;
import java.lang.reflect.Method;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.Set;
import java.util.function.Predicate;
import org.junit.jupiter.api.Test;

final class FullFunctionalityAutoSelectionAndVisionPathologies {
    @Test
    void autonomousInitShouldNotScheduleHomingWhenForcedAutoDeclaresIntakeAndShooterCommands() throws IOException {
        try (FullFunctionalityHarness.Context context = new FullFunctionalityHarness.Context(false)) {
            context.setAutonomousEnabled();
            context.runCycles(8);

            SelectedPathPlannerAuto selectedAuto = forcePathPlannerSelection(
                    context,
                    declaredNames -> containsNamedCommandWithPrefix(declaredNames, "Intake")
                            && containsNamedCommandWithPrefix(declaredNames, "Shooter"));

            assertTrue(
                    containsNamedCommandWithPrefix(selectedAuto.declaredNamedCommands(), "Intake")
                            && containsNamedCommandWithPrefix(selectedAuto.declaredNamedCommands(), "Shooter"),
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "Forced pp/* routine should declare both intake and shooter named commands",
                            "declared contains Intake* and Shooter*",
                            String.format(
                                    Locale.US,
                                    "%s declared=%s",
                                    selectedAuto.routineName(),
                                    selectedAuto.declaredNamedCommands())));

            String selectedAutoLifecycleName = "AutoPathPlanner_" + selectedAuto.autoName();
            FullFunctionalityHarness.CommandCounts selectedAutoBefore =
                    context.recorder.getCounts(selectedAutoLifecycleName);
            FullFunctionalityHarness.CommandCounts intakeHomeBefore =
                    context.recorder.getCounts("IntakeHomeThenBackground");
            FullFunctionalityHarness.CommandCounts shooterHomeBefore =
                    context.recorder.getCounts("ShooterHomeThenBackground");

            context.container.autonomousInit();
            context.runCycles(40);

            FullFunctionalityHarness.CommandCounts selectedAutoDelta =
                    context.recorder.getCounts(selectedAutoLifecycleName).minus(selectedAutoBefore);
            FullFunctionalityHarness.CommandCounts intakeHomeDelta =
                    context.recorder.getCounts("IntakeHomeThenBackground").minus(intakeHomeBefore);
            FullFunctionalityHarness.CommandCounts shooterHomeDelta =
                    context.recorder.getCounts("ShooterHomeThenBackground").minus(shooterHomeBefore);

            assertTrue(
                    selectedAutoDelta.starts() >= 1,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "Forced selected PathPlanner auto should start during autonomousInit",
                            selectedAutoLifecycleName + " starts>=1",
                            selectedAutoDelta));
            assertEquals(
                    0,
                    intakeHomeDelta.starts(),
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "autonomousInit SHOULD NOT schedule IntakeHomeThenBackground when selected auto itself uses intake/shooter",
                            "starts=0",
                            intakeHomeDelta));
            assertEquals(
                    0,
                    shooterHomeDelta.starts(),
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "autonomousInit SHOULD NOT schedule ShooterHomeThenBackground when selected auto itself uses intake/shooter",
                            "starts=0",
                            shooterHomeDelta));
        }
    }

    @Test
    void driveSetOdometryFromUnifiedVisionShouldNotReportSuccessWhenNoPoseAvailable() {
        try (FullFunctionalityHarness.Context context = new FullFunctionalityHarness.Context(false)) {
            context.setAutonomousEnabled();
            context.container.autonomousInit();
            context.runCycles(12);

            Object vision = FullFunctionalityHarness.getPrivateField(context.container, "vision", Object.class);
            assertTrue(vision != null, "Expected vision subsystem to be present in simulation context.");

            FullFunctionalityHarness.CommandCounts before =
                    context.recorder.getCounts("DriveSetOdometryFromUnifiedVision");

            for (int i = 0; i < 8; i++) {
                clearUnifiedVisionPoseState(vision);
                Command command = invokeCreateSetOdometryFromUnifiedVisionCommand(context.container);
                CommandScheduler.getInstance().schedule(command);
                context.runCycles(2);
            }

            FullFunctionalityHarness.CommandCounts delta =
                    context.recorder.getCounts("DriveSetOdometryFromUnifiedVision").minus(before);

            assertEquals(
                    0,
                    delta.starts(),
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "DriveSetOdometryFromUnifiedVision should not report successful action start when no unified vision pose is available",
                            "starts=0",
                            delta));
            assertEquals(
                    0,
                    delta.finishes(),
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "DriveSetOdometryFromUnifiedVision should not report successful action finish when no unified vision pose is available",
                            "finishes=0",
                            delta));
        }
    }

    @Test
    void disableVisionOverrideTogglingDuringAutoHeadingAlignmentShouldKeepOdometryBounded() throws IOException {
        try (FullFunctionalityHarness.Context context = new FullFunctionalityHarness.Context(false)) {
            context.setAutonomousEnabled();
            context.drive.setPose(Pose2d.kZero);
            context.runCycles(8);

            SelectedPathPlannerAuto selectedAuto = forcePathPlannerSelection(
                    context,
                    Set::isEmpty);

            String selectedAutoLifecycleName = "AutoPathPlanner_" + selectedAuto.autoName();
            String headingAlignmentLifecycleName = "FF_StressAutoHeadingAlignment";
            FullFunctionalityHarness.CommandCounts selectedAutoBefore =
                    context.recorder.getCounts(selectedAutoLifecycleName);
            FullFunctionalityHarness.CommandCounts headingAlignmentBefore =
                    context.recorder.getCounts(headingAlignmentLifecycleName);

            context.container.autonomousInit();
            Command headingAlignment =
                    FullFunctionalityHarness.namedCommand("ShooterShootHubOnMove").withName(headingAlignmentLifecycleName);
            CommandScheduler.getInstance().schedule(headingAlignment);

            Pose2d startPose = context.drive.getPose();
            Pose2d previousPose = startPose;
            double maxStepTranslationMeters = 0.0;
            double maxStepHeadingDeg = 0.0;
            double maxDistanceFromStartMeters = 0.0;
            boolean autoWasRunning = false;
            boolean autoAndHeadingAlignmentOverlapped = false;

            for (int cycle = 0; cycle < 1200; cycle++) {
                SmartDashboard.putBoolean("Overrides/DisableVision", (cycle & 1) == 0);
                context.runCycles(1);

                Pose2d pose = context.drive.getPose();
                maxStepTranslationMeters = Math.max(
                        maxStepTranslationMeters,
                        pose.getTranslation().getDistance(previousPose.getTranslation()));
                maxStepHeadingDeg = Math.max(
                        maxStepHeadingDeg,
                        Math.abs(Math.toDegrees(MathUtil.angleModulus(
                                pose.getRotation().minus(previousPose.getRotation()).getRadians()))));
                maxDistanceFromStartMeters = Math.max(
                        maxDistanceFromStartMeters,
                        pose.getTranslation().getDistance(startPose.getTranslation()));
                previousPose = pose;

                boolean currentlyRunning = context.recorder.runningCount(selectedAutoLifecycleName) > 0;
                boolean headingAlignmentRunning = CommandScheduler.getInstance().isScheduled(headingAlignment);
                autoWasRunning |= currentlyRunning;
                autoAndHeadingAlignmentOverlapped |= currentlyRunning && headingAlignmentRunning;
                if (autoWasRunning && !currentlyRunning && cycle > 120) {
                    break;
                }
            }

            CommandScheduler.getInstance().cancel(headingAlignment);
            SmartDashboard.putBoolean("Overrides/DisableVision", false);
            context.runCycles(6);

            FullFunctionalityHarness.CommandCounts selectedAutoDelta =
                    context.recorder.getCounts(selectedAutoLifecycleName).minus(selectedAutoBefore);
            FullFunctionalityHarness.CommandCounts headingAlignmentDelta =
                    context.recorder.getCounts(headingAlignmentLifecycleName).minus(headingAlignmentBefore);

            assertTrue(
                    selectedAutoDelta.starts() >= 1,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "Selected auto should start while stress-toggling DisableVision",
                            selectedAutoLifecycleName + " starts>=1",
                            selectedAutoDelta));
            assertTrue(
                    autoAndHeadingAlignmentOverlapped,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "Heading alignment stress command should overlap with active auto",
                            "overlap=true",
                            String.format(Locale.US, "overlap=%s", autoAndHeadingAlignmentOverlapped)));
            assertTrue(
                    headingAlignmentDelta.starts() >= 1,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "Selected auto should exercise heading alignment command while toggling DisableVision",
                            headingAlignmentLifecycleName + " starts>=1",
                            headingAlignmentDelta));
            assertTrue(
                    maxStepTranslationMeters <= 0.85,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "Odometry translation step should remain bounded during rapid DisableVision toggling",
                            "maxStepTranslation<=0.85m",
                            String.format(Locale.US, "maxStepTranslation=%.3fm", maxStepTranslationMeters)));
            assertTrue(
                    maxStepHeadingDeg <= 55.0,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "Odometry heading step should remain bounded during rapid DisableVision toggling",
                            "maxStepHeading<=55deg",
                            String.format(Locale.US, "maxStepHeading=%.3fdeg", maxStepHeadingDeg)));
            assertTrue(
                    maxDistanceFromStartMeters <= 20.0,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "Odometry should remain globally bounded during stress scenario",
                            "maxDistanceFromStart<=20m",
                            String.format(Locale.US, "maxDistanceFromStart=%.3fm", maxDistanceFromStartMeters)));
        }
    }

    private static boolean containsNamedCommandWithPrefix(Set<String> declaredNames, String prefix) {
        return declaredNames.stream().anyMatch(name -> name.startsWith(prefix));
    }

    private static SelectedPathPlannerAuto forcePathPlannerSelection(
            FullFunctionalityHarness.Context context,
            Predicate<Set<String>> namedCommandPredicate)
            throws IOException {
        Object autoSelector = FullFunctionalityHarness.getPrivateField(context.container, "autoSelector", Object.class);
        Object chooser = getPrivateField(autoSelector, "chooser");

        @SuppressWarnings("unchecked")
        Map<String, Object> options = (Map<String, Object>) getPrivateField(chooser, "options");

        Path autosDir = Filesystem.getDeployDirectory().toPath().resolve("pathplanner").resolve("autos");
        List<String> candidateFailures = new ArrayList<>();

        for (String routineName : options.keySet().stream().filter(name -> name.startsWith("pp/")).sorted().toList()) {
            String autoName = routineName.substring("pp/".length());
            Set<String> declaredNames =
                    FullFunctionalityHarness.parseDeclaredNamedCommands(autosDir.resolve(autoName + ".auto"));
            if (namedCommandPredicate.test(declaredNames)) {
                setPrivateField(chooser, "selectedValue", routineName);
                setPrivateField(chooser, "previousValue", routineName);
                return new SelectedPathPlannerAuto(routineName, autoName, declaredNames);
            }
            candidateFailures.add(routineName + " declared=" + declaredNames);
        }

        throw new AssertionError(
                "Unable to force a pp/* routine matching required named commands. Candidates: "
                        + String.join(" | ", candidateFailures));
    }

    private static Command invokeCreateSetOdometryFromUnifiedVisionCommand(Object container) {
        try {
            Method method = container.getClass().getDeclaredMethod("createSetOdometryFromUnifiedVisionCommand");
            method.setAccessible(true);
            return (Command) method.invoke(container);
        } catch (ReflectiveOperationException exception) {
            throw new IllegalStateException(
                    "Unable to invoke RobotContainer#createSetOdometryFromUnifiedVisionCommand via reflection",
                    exception);
        }
    }

    private static void clearUnifiedVisionPoseState(Object vision) {
        setPrivateField(vision, "unifiedRobotPose", null);
        setPrivateField(vision, "unifiedRobotPoseRaw", null);
        setPrivateDoubleField(vision, "unifiedRobotPoseTimestampSeconds", Double.NaN);
        setPrivateDoubleField(vision, "unifiedRobotPoseRawTimestampSeconds", Double.NaN);
    }

    private static Object getPrivateField(Object target, String fieldName) {
        try {
            Field field = target.getClass().getDeclaredField(fieldName);
            field.setAccessible(true);
            return field.get(target);
        } catch (ReflectiveOperationException exception) {
            throw new IllegalStateException(
                    "Unable to read field '" + fieldName + "' from " + target.getClass().getName(),
                    exception);
        }
    }

    private static void setPrivateField(Object target, String fieldName, Object value) {
        try {
            Field field = target.getClass().getDeclaredField(fieldName);
            field.setAccessible(true);
            field.set(target, value);
        } catch (ReflectiveOperationException exception) {
            throw new IllegalStateException(
                    "Unable to set field '" + fieldName + "' on " + target.getClass().getName(),
                    exception);
        }
    }

    private static void setPrivateDoubleField(Object target, String fieldName, double value) {
        try {
            Field field = target.getClass().getDeclaredField(fieldName);
            field.setAccessible(true);
            field.setDouble(target, value);
        } catch (ReflectiveOperationException exception) {
            throw new IllegalStateException(
                    "Unable to set double field '" + fieldName + "' on " + target.getClass().getName(),
                    exception);
        }
    }

    private record SelectedPathPlannerAuto(
            String routineName,
            String autoName,
            Set<String> declaredNamedCommands) {
    }
}
