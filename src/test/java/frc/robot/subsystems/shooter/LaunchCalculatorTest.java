package frc.robot.subsystems.shooter;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class LaunchCalculatorTest {
    private static final double EPSILON = 0.01;
    private static final double BLUE_HUB_Y_METERS = 4.0213534;

    private LaunchCalculator calculator;

    @BeforeEach
    void setUp() {
        HAL.initialize(500, 0);

        InterpolatingDoubleTreeMap timeMap = new InterpolatingDoubleTreeMap();
        double[] distances = ShooterConstants.SHOT_MAP_DISTANCE_METERS;
        double[] times = ShooterConstants.SHOT_TIME_IN_AIR_SECONDS;
        for (int i = 0; i < distances.length; i++) {
            timeMap.put(distances[i], times[i]);
        }

        calculator = new LaunchCalculator(
                timeMap::get,
                ShooterConstants.ROBOT_TO_SHOOTER_OFFSET,
                ShooterConstants.PHASE_DELAY_SEC,
                ShooterConstants.MOTION_COMP_TIME_SCALE,
                ShooterConstants.MOTION_COMP_DISTANCE_TIME_SCALE);
    }

    @Test
    void stationaryRobotCompensatedDistanceMatchesRaw() {
        Pose2d pose = new Pose2d(2.0, 4.0, Rotation2d.kZero);
        ChassisSpeeds speeds = new ChassisSpeeds(0, 0, 0);

        LaunchCalculator.MotionCompensation result = calculator.calculate(pose, speeds);

        assertTrue(Double.isFinite(result.rawDistanceMeters()));
        assertTrue(Double.isFinite(result.compensatedDistanceMeters()));
        assertEquals(result.rawDistanceMeters(), result.compensatedDistanceMeters(), 0.1);
    }

    @Test
    void movingTowardHubReducesCompensatedDistance() {
        // Robot directly below hub, facing hub (positive X toward hub at 4.6, 4.0213534)
        Pose2d pose = new Pose2d(1.0, BLUE_HUB_Y_METERS, Rotation2d.kZero);
        ChassisSpeeds speeds = new ChassisSpeeds(3.0, 0, 0); // moving toward hub

        LaunchCalculator.MotionCompensation result = calculator.calculate(pose, speeds);

        assertTrue(Double.isFinite(result.compensatedDistanceMeters()));
        assertTrue(
                result.compensatedDistanceMeters() < result.rawDistanceMeters(),
                "Compensated distance should be less when moving toward hub");
    }

    @Test
    void movingAwayFromHubIncreasesCompensatedDistance() {
        Pose2d pose = new Pose2d(1.0, BLUE_HUB_Y_METERS, Rotation2d.kZero);
        ChassisSpeeds speeds = new ChassisSpeeds(-3.0, 0, 0); // moving away from hub

        LaunchCalculator.MotionCompensation result = calculator.calculate(pose, speeds);

        assertTrue(Double.isFinite(result.compensatedDistanceMeters()));
        assertTrue(
                result.compensatedDistanceMeters() > result.rawDistanceMeters(),
                "Compensated distance should be greater when moving away from hub");
    }

    @Test
    void lateralMovementShiftsCompensatedHeading() {
        Pose2d pose = new Pose2d(2.0, 4.0, Rotation2d.kZero);
        ChassisSpeeds stationary = new ChassisSpeeds(0, 0, 0);
        ChassisSpeeds lateral = new ChassisSpeeds(0, 2.0, 0); // moving left

        LaunchCalculator.MotionCompensation stationaryResult = calculator.calculate(pose, stationary);
        LaunchCalculator.MotionCompensation lateralResult = calculator.calculate(pose, lateral);

        assertNotNull(stationaryResult.compensatedHeading());
        assertNotNull(lateralResult.compensatedHeading());
        double headingDiffRad = lateralResult
                .compensatedHeading()
                .minus(stationaryResult.compensatedHeading())
                .getRadians();
        assertTrue(
                Math.abs(headingDiffRad) > 0.001,
                "Lateral movement should shift compensated heading");
    }

    @Test
    void nullPoseReturnsInvalid() {
        ChassisSpeeds speeds = new ChassisSpeeds(0, 0, 0);
        LaunchCalculator.MotionCompensation result = calculator.calculate(null, speeds);

        assertTrue(Double.isNaN(result.rawDistanceMeters()));
        assertTrue(Double.isNaN(result.compensatedDistanceMeters()));
        assertNull(result.compensatedHeading());
    }

    @Test
    void nullSpeedsReturnsInvalid() {
        Pose2d pose = new Pose2d(2.0, 4.0, Rotation2d.kZero);
        LaunchCalculator.MotionCompensation result = calculator.calculate(pose, null);

        assertTrue(Double.isNaN(result.rawDistanceMeters()));
        assertNull(result.compensatedHeading());
    }

    @Test
    void resultFieldsAreFiniteForReasonableInputs() {
        Pose2d pose = new Pose2d(2.0, 2.0, Rotation2d.fromDegrees(45));
        ChassisSpeeds speeds = new ChassisSpeeds(1.0, -0.5, 0.3);

        LaunchCalculator.MotionCompensation result = calculator.calculate(pose, speeds);

        assertTrue(Double.isFinite(result.rawDistanceMeters()));
        assertTrue(Double.isFinite(result.compensatedDistanceMeters()));
        assertTrue(Double.isFinite(result.timeInAirSec()));
        assertTrue(Double.isFinite(result.velocityTowardHubMps()));
        assertTrue(Double.isFinite(result.velocityPerpendicularHubMps()));
        assertNotNull(result.compensatedHeading());
    }

    @Test
    void staticGetHubDistanceMetersWithKnownPose() {
        // Hub is at (4.6, 4.0213534) for blue alliance; shooter offset is (-0.1524, 0)
        // Robot at (4.6, 4.0213534) facing 0 deg -> shooter at (4.4476, 4.0213534)
        // Distance from (4.4476, 4.0213534) to (4.6, 4.0213534) = 0.1524
        Pose2d atHub = new Pose2d(4.6, BLUE_HUB_Y_METERS, Rotation2d.kZero);
        double distance = LaunchCalculator.getHubDistanceMeters(atHub);
        assertTrue(Double.isFinite(distance));
        assertEquals(0.1524, distance, EPSILON);
    }

    @Test
    void staticGetHubDistanceMetersWithNullReturnsNaN() {
        assertTrue(Double.isNaN(LaunchCalculator.getHubDistanceMeters(null)));
    }
}
