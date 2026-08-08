package frc.robot.subsystems.drive;

import com.pathplanner.lib.path.PathConstraints;
import static edu.wpi.first.units.Units.*;

import java.util.Collection;

/** Constants for drive subsystem configuration. */
public final class DriveConstants {
    public enum ConstraintProfile {
        SLOW_MODE,
        SHOOTING_ON_MOVE
    }

    private static final double DRIVER_MAX_LINEAR_VELOCITY_MPS = 1.0;
    private static final double DRIVER_MAX_LINEAR_ACCELERATION_MPSSQ = 10.0;
    private static final double DRIVEBASE_RADIUS_METERS = Drive.DRIVE_BASE_RADIUS;
    private static final double DRIVER_MAX_ANGULAR_VELOCITY_RAD_PER_SEC =
            DRIVER_MAX_LINEAR_VELOCITY_MPS / DRIVEBASE_RADIUS_METERS;
    private static final double DRIVER_MAX_ANGULAR_ACCELERATION_RAD_PER_SEC_SQ =
            DegreesPerSecondPerSecond.of(700.0).in(RadiansPerSecondPerSecond);

    private static final double SLOW_MODE_MAX_LINEAR_VELOCITY_MPS = 2.0;
    private static final double SLOW_MODE_MAX_LINEAR_ACCELERATION_MPSSQ = 35.0;
    private static final double SLOW_MODE_MAX_ANGULAR_VELOCITY_RAD_PER_SEC =
            SLOW_MODE_MAX_LINEAR_VELOCITY_MPS / DRIVEBASE_RADIUS_METERS;
    private static final double SLOW_MODE_MAX_ANGULAR_ACCELERATION_RAD_PER_SEC_SQ =
            DegreesPerSecondPerSecond.of(1000.0).in(RadiansPerSecondPerSecond);

    private static final double SHOOTING_MAX_LINEAR_VELOCITY_MPS = 2.0;
    private static final double SHOOTING_MAX_LINEAR_ACCELERATION_MPSSQ = 35.0;
    private static final double SHOOTING_MAX_ANGULAR_VELOCITY_RAD_PER_SEC = SLOW_MODE_MAX_LINEAR_VELOCITY_MPS
                    / DRIVEBASE_RADIUS_METERS;
    private static final double SHOOTING_MAX_ANGULAR_ACCELERATION_RAD_PER_SEC_SQ =
            DegreesPerSecondPerSecond.of(1000.0).in(RadiansPerSecondPerSecond);

    public static final double MAX_STEER_VELOCITY_RAD_PER_SEC = DegreesPerSecond.of(1500.0).in(RadiansPerSecond);

    /** Robot physical constants for PathPlanner configuration. */
    public static final double ROBOT_MASS_KG = 74.088;
    public static final double ROBOT_MOI = 6.883;
    public static final double WHEEL_COF = 1.2;

    public static final PathConstraints DRIVER_DEFAULT_LIMITS = new PathConstraints(
            MetersPerSecond.of(DRIVER_MAX_LINEAR_VELOCITY_MPS),
            MetersPerSecondPerSecond.of(DRIVER_MAX_LINEAR_ACCELERATION_MPSSQ),
            RadiansPerSecond.of(DRIVER_MAX_ANGULAR_VELOCITY_RAD_PER_SEC),
            RadiansPerSecondPerSecond.of(DRIVER_MAX_ANGULAR_ACCELERATION_RAD_PER_SEC_SQ));

    public static final PathConstraints SLOW_MODE_LIMITS = new PathConstraints(
            MetersPerSecond.of(SLOW_MODE_MAX_LINEAR_VELOCITY_MPS),
            MetersPerSecondPerSecond.of(SLOW_MODE_MAX_LINEAR_ACCELERATION_MPSSQ),
            RadiansPerSecond.of(SLOW_MODE_MAX_ANGULAR_VELOCITY_RAD_PER_SEC),
            RadiansPerSecondPerSecond.of(SLOW_MODE_MAX_ANGULAR_ACCELERATION_RAD_PER_SEC_SQ));

    public static final PathConstraints SHOOTING_ON_MOVE_LIMITS = new PathConstraints(
            MetersPerSecond.of(SHOOTING_MAX_LINEAR_VELOCITY_MPS),
            MetersPerSecondPerSecond.of(SHOOTING_MAX_LINEAR_ACCELERATION_MPSSQ),
            RadiansPerSecond.of(SHOOTING_MAX_ANGULAR_VELOCITY_RAD_PER_SEC),
            RadiansPerSecondPerSecond.of(SHOOTING_MAX_ANGULAR_ACCELERATION_RAD_PER_SEC_SQ));

    public static final PathConstraints AUTO_LIMITS = new PathConstraints(
            MetersPerSecond.of(3.0),
            MetersPerSecondPerSecond.of(14.5),
            DegreesPerSecond.of(540.0),
            DegreesPerSecondPerSecond.of(720.0));

    private DriveConstants() {}

    public static PathConstraints constraintsForProfile(ConstraintProfile profile) {
        return switch (profile) {
            case SLOW_MODE -> SLOW_MODE_LIMITS;
            case SHOOTING_ON_MOVE -> SHOOTING_ON_MOVE_LIMITS;
        };
    }

    public static PathConstraints mergeConstraints(PathConstraints base, Collection<PathConstraints> overlays) {
        double maxVelocityMps = base.maxVelocityMPS();
        double maxAccelerationMpsSq = base.maxAccelerationMPSSq();
        double maxAngularVelocityRadPerSec = base.maxAngularVelocityRadPerSec();
        double maxAngularAccelerationRadPerSecSq = base.maxAngularAccelerationRadPerSecSq();

        for (PathConstraints overlay : overlays) {
            if (overlay == null) {
                continue;
            }
            maxVelocityMps = Math.min(maxVelocityMps, overlay.maxVelocityMPS());
            maxAccelerationMpsSq = Math.min(maxAccelerationMpsSq, overlay.maxAccelerationMPSSq());
            maxAngularVelocityRadPerSec =
                    Math.min(maxAngularVelocityRadPerSec, overlay.maxAngularVelocityRadPerSec());
            maxAngularAccelerationRadPerSecSq =
                    Math.min(maxAngularAccelerationRadPerSecSq, overlay.maxAngularAccelerationRadPerSecSq());
        }

        return new PathConstraints(
                maxVelocityMps,
                maxAccelerationMpsSq,
                maxAngularVelocityRadPerSec,
                maxAngularAccelerationRadPerSecSq);
    }
}
