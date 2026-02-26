package frc.robot.subsystems.drive;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertSame;

import com.pathplanner.lib.path.PathConstraints;
import java.util.List;
import org.junit.jupiter.api.Test;

class DriveConstantsTest {
    private static final double EPSILON = 1e-9;

    @Test
    void mergeConstraintsUsesStrictestValues() {
        PathConstraints base = new PathConstraints(4.0, 14.0, 9.0, 30.0);
        PathConstraints overlayA = new PathConstraints(3.0, 10.0, 8.5, 25.0);
        PathConstraints overlayB = new PathConstraints(3.5, 6.0, 7.0, 15.0);

        PathConstraints merged = DriveConstants.mergeConstraints(base, List.of(overlayA, overlayB));

        assertEquals(3.0, merged.maxVelocityMPS(), EPSILON);
        assertEquals(6.0, merged.maxAccelerationMPSSq(), EPSILON);
        assertEquals(7.0, merged.maxAngularVelocityRadPerSec(), EPSILON);
        assertEquals(15.0, merged.maxAngularAccelerationRadPerSecSq(), EPSILON);
    }

    @Test
    void profileConstraintMappingIsStable() {
        assertSame(
                DriveConstants.SLOW_MODE_LIMITS,
                DriveConstants.constraintsForProfile(DriveConstants.ConstraintProfile.SLOW_MODE));
        assertSame(
                DriveConstants.SHOOTING_ON_MOVE_LIMITS,
                DriveConstants.constraintsForProfile(DriveConstants.ConstraintProfile.SHOOTING_ON_MOVE));
    }
}
