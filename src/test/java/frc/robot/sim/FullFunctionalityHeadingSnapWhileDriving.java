package frc.robot.sim;

import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import org.junit.jupiter.api.Test;

final class FullFunctionalityHeadingSnapWhileDriving {
    private static final double SNAP_HEADING_TOLERANCE_DEG = 3.0;

    @Test
    void headingSnapKeepsTranslatingWhileTurningToCardinal() {
        try (FullFunctionalityHarness.Context context = new FullFunctionalityHarness.Context(false)) {
            context.setTeleopEnabled();
            context.container.teleopInit();
            context.runCycles(20);

            context.drive.setPose(new Pose2d(
                    context.drive.getPose().getTranslation(),
                    Rotation2d.fromRadians(0.90)));
            context.driverControllerSim.setLeftY(0.45);
            context.runCycles(10);

            Pose2d translationStartPose = context.drive.getPose();
            FullFunctionalityHarness.CommandCounts snapBefore = context.recorder.getCounts("DriveHeadingSnap");

            context.tapDriverPov(0);

            boolean sawSnapRunning = false;
            boolean translatedWhileSnapRunning = false;
            boolean snapFinished = false;
            for (int i = 0; i < 260; i++) {
                if (context.recorder.runningCount("DriveHeadingSnap") >= 1) {
                    sawSnapRunning = true;
                    translatedWhileSnapRunning |=
                            context.drive.getPose().getTranslation().getDistance(translationStartPose.getTranslation()) > 0.20;
                }
                if (context.recorder.getCounts("DriveHeadingSnap").minus(snapBefore).finishes() >= 1) {
                    snapFinished = true;
                    break;
                }
                context.runCycles(1);
            }

            assertTrue(sawSnapRunning, "Expected DriveHeadingSnap to run during the POV-up scenario.");
            assertTrue(
                    translatedWhileSnapRunning,
                    "Heading snap should preserve translational driving while it rotates to cardinal.");
            assertTrue(snapFinished, "Heading snap should finish once it reaches the heading tolerance.");
            assertTrue(
                    Math.abs(MathUtil.angleModulus(
                            Rotation2d.fromDegrees(90.0).minus(context.drive.getPose().getRotation()).getRadians()))
                            <= Math.toRadians(SNAP_HEADING_TOLERANCE_DEG),
                    "Heading snap should finish near the nearest right angle.");

            context.driverControllerSim.setLeftY(0.0);
            context.runCycles(10);
        }
    }

    @Test
    void headingSnapInterruptsAsSoonAsTurnStickReturns() {
        try (FullFunctionalityHarness.Context context = new FullFunctionalityHarness.Context(false)) {
            context.setTeleopEnabled();
            context.container.teleopInit();
            context.runCycles(20);

            context.drive.setPose(new Pose2d(
                    context.drive.getPose().getTranslation(),
                    Rotation2d.fromRadians(0.90)));
            context.driverControllerSim.setLeftY(0.35);
            context.runCycles(10);

            FullFunctionalityHarness.CommandCounts snapBefore = context.recorder.getCounts("DriveHeadingSnap");
            context.driverControllerSim.setPOV(0);
            context.runCycles(4);
            context.driverControllerSim.setPOV(-1);

            boolean snapStarted = context.runUntil(
                    () -> context.recorder.runningCount("DriveHeadingSnap") >= 1,
                    40);
            assertTrue(snapStarted, "Expected heading snap to start before reintroducing turn-stick input.");

            context.driverControllerSim.setRightX(0.65);
            boolean snapStopped = context.runUntil(
                    () -> context.recorder.runningCount("DriveHeadingSnap") == 0,
                    20);
            assertTrue(snapStopped, "Heading snap should stop immediately when the driver moves the turn stick.");
            assertTrue(
                    context.recorder.runningCount("DriveJoystickDefault") >= 1,
                    "Drive default should resume immediately after heading snap interruption.");

            context.driverControllerSim.setRightX(0.0);
            context.driverControllerSim.setLeftY(0.0);
            context.runCycles(10);
        }
    }
}
