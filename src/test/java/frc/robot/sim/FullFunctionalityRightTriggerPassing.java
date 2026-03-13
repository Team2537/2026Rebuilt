package frc.robot.sim;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.util.FieldConstants;
import org.junit.jupiter.api.Test;

final class FullFunctionalityRightTriggerPassing {
    private static final String RIGHT_TRIGGER_TARGET_OBJECT_NAME = "Right Trigger Target";
    private static final double POSE_EPSILON_METERS = 1e-6;

    @Test
    void blueShootButtonPassesInNeutralUpperLane() {
        try (FullFunctionalityHarness.Context context = new FullFunctionalityHarness.Context(false)) {
            context.setTeleopEnabled();
            context.container.teleopInit();
            context.runCycles(20);

            Pose2d passPose = new Pose2d(
                    8.0,
                    FieldConstants.getHubBackBlockUpperY() + 0.2,
                    Rotation2d.kZero);
            context.drive.setPose(passPose);
            context.runCycles(10);

            Field2d dashboardField =
                    FullFunctionalityHarness.getPrivateField(context.container, "dashboardField", Field2d.class);
            Pose2d expectedPassTarget = FieldConstants.getPassTargetPose(passPose);

            context.driverControllerSim.setRightBumperButton(true);
            boolean kickerActivated = context.runUntil(context.shooter::isKickerActive, 420);
            assertTrue(kickerActivated, "Blue neutral-lane shoot button should eventually pass.");

            Pose2d publishedTarget = dashboardField.getObject(RIGHT_TRIGGER_TARGET_OBJECT_NAME).getPose();
            assertEquals(expectedPassTarget.getX(), publishedTarget.getX(), POSE_EPSILON_METERS);
            assertEquals(expectedPassTarget.getY(), publishedTarget.getY(), POSE_EPSILON_METERS);
            assertTrue(
                    publishedTarget.getX() < FieldConstants.getAllianceZoneBoundaryX(),
                    "Blue pass target should stay on the blue side of the field.");
            assertTrue(
                    publishedTarget.getY() > FieldConstants.getHubTargetTranslation().getY(),
                    "Blue pass target should avoid the hub lane.");

            context.driverControllerSim.setRightBumperButton(false);
            context.runCycles(10);
        }
    }

    @Test
    void redShootButtonPassesInNeutralLowerLane() {
        try (FullFunctionalityHarness.Context context = new FullFunctionalityHarness.Context(false)) {
            DriverStationSim.setAllianceStationId(AllianceStationID.Red1);
            DriverStationSim.notifyNewData();
            context.runCycles(6);
            context.setTeleopEnabled();
            context.container.teleopInit();
            context.runCycles(20);

            Pose2d passPose = new Pose2d(
                    8.0,
                    FieldConstants.getHubBackBlockLowerY() - 0.2,
                    Rotation2d.kZero);
            context.drive.setPose(passPose);
            context.runCycles(10);

            Field2d dashboardField =
                    FullFunctionalityHarness.getPrivateField(context.container, "dashboardField", Field2d.class);
            Pose2d expectedPassTarget = FieldConstants.getPassTargetPose(passPose);

            context.driverControllerSim.setRightBumperButton(true);
            boolean kickerActivated = context.runUntil(context.shooter::isKickerActive, 420);
            assertTrue(kickerActivated, "Red neutral-lane shoot button should eventually pass.");

            Pose2d publishedTarget = dashboardField.getObject(RIGHT_TRIGGER_TARGET_OBJECT_NAME).getPose();
            assertEquals(expectedPassTarget.getX(), publishedTarget.getX(), POSE_EPSILON_METERS);
            assertEquals(expectedPassTarget.getY(), publishedTarget.getY(), POSE_EPSILON_METERS);
            assertTrue(
                    publishedTarget.getX() > FieldConstants.getAllianceZoneBoundaryX(),
                    "Red pass target should stay on the red side of the field.");
            assertTrue(
                    publishedTarget.getY() < FieldConstants.getHubTargetTranslation().getY(),
                    "Red pass target should avoid the hub lane.");

            context.driverControllerSim.setRightBumperButton(false);
            context.runCycles(10);
        }
    }
}
