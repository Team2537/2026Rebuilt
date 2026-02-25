package frc.robot.subsystems.shooter;

import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class ShooterMotionCompensationTest {
    @BeforeEach
    void setUp() {
        HAL.initialize(500, 0);
    }

    @Test
    void outOfRangeDistanceStillProducesHeadingAndDistance() {
        Shooter shooter = new Shooter(new NoopShooterIO());
        Pose2d robotPose = new Pose2d(-3.0, -3.0, Rotation2d.kZero);
        ChassisSpeeds robotSpeeds = new ChassisSpeeds(3.5, -1.2, 0.0);

        Shooter.MotionCompensation compensation =
                shooter.getMotionCompensationToHub(robotPose, robotSpeeds);

        assertTrue(Double.isFinite(compensation.rawDistanceMeters()));
        assertTrue(Double.isFinite(compensation.compensatedDistanceMeters()));
        assertNotNull(compensation.compensatedHeading());
    }

    private static final class NoopShooterIO implements ShooterIO {}
}
