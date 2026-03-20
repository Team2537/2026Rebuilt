package frc.robot.subsystems.shooter;

import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class ShooterMotionCompensationTest {
    private static final double BLUE_HUB_Y_METERS = 4.0213534;

    @BeforeEach
    void setUp() {
        HAL.initialize(500, 0);
        SmartDashboard.putNumber("Shooter/MotionCompTimeScale", 1.0);
        SmartDashboard.putNumber("Shooter/MotionCompDistanceTimeScale", 1.0);
    }

    @Test
    void outOfRangeDistanceStillProducesHeadingAndDistance() {
        Shooter shooter = new Shooter(new NoopShooterIO());
        Pose2d robotPose = new Pose2d(-3.0, -3.0, Rotation2d.kZero);
        ChassisSpeeds robotSpeeds = new ChassisSpeeds(3.5, -1.2, 0.0);

        LaunchCalculator.MotionCompensation compensation =
                shooter.getMotionCompensationToHub(robotPose, robotSpeeds);

        assertTrue(Double.isFinite(compensation.rawDistanceMeters()));
        assertTrue(Double.isFinite(compensation.compensatedDistanceMeters()));
        assertNotNull(compensation.compensatedHeading());
    }

    @Test
    void motionCompTimeScaleUpdatesFromDashboardAfterPeriodic() {
        Shooter shooter = new Shooter(new NoopShooterIO());
        Pose2d robotPose = new Pose2d(1.0, BLUE_HUB_Y_METERS, Rotation2d.kZero);
        ChassisSpeeds robotSpeeds = new ChassisSpeeds(3.0, 0.0, 0.0);

        LaunchCalculator.MotionCompensation baseline = shooter.getMotionCompensationToHub(robotPose, robotSpeeds);

        SmartDashboard.putNumber("Shooter/MotionCompTimeScale", 0.5);
        shooter.periodic();

        LaunchCalculator.MotionCompensation updated = shooter.getMotionCompensationToHub(robotPose, robotSpeeds);

        assertTrue(Double.isFinite(baseline.compensatedDistanceMeters()));
        assertTrue(Double.isFinite(updated.compensatedDistanceMeters()));
        assertTrue(
                updated.compensatedDistanceMeters() > baseline.compensatedDistanceMeters(),
                "Reducing MotionCompTimeScale on the dashboard should reduce the amount of lead.");
    }

    @Test
    void motionCompDistanceTimeScaleUpdatesFromDashboardAfterPeriodic() {
        Shooter shooter = new Shooter(new NoopShooterIO());
        Pose2d robotPose = new Pose2d(1.0, BLUE_HUB_Y_METERS, Rotation2d.kZero);
        ChassisSpeeds robotSpeeds = new ChassisSpeeds(3.0, 0.0, 0.0);

        LaunchCalculator.MotionCompensation baseline = shooter.getMotionCompensationToHub(robotPose, robotSpeeds);

        SmartDashboard.putNumber("Shooter/MotionCompDistanceTimeScale", 0.5);
        shooter.periodic();

        LaunchCalculator.MotionCompensation updated = shooter.getMotionCompensationToHub(robotPose, robotSpeeds);

        assertTrue(Double.isFinite(baseline.compensatedDistanceMeters()));
        assertTrue(Double.isFinite(updated.compensatedDistanceMeters()));
        assertTrue(
                Math.abs(updated.compensatedDistanceMeters() - baseline.compensatedDistanceMeters()) > 1e-6,
                "Changing MotionCompDistanceTimeScale on the dashboard should change the compensation result.");
    }

    private static final class NoopShooterIO implements ShooterIO {}
}
