package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;
import java.util.function.DoubleSupplier;

/** Reusable autonomous command primitives intended for composition in PathPlanner. */
public final class AutoCommands {
    private AutoCommands() {}

    private static DoubleSupplier hubDistanceMeters(Drive drive, Shooter shooter) {
        return () -> shooter.getMotionCompensatedHubDistanceMeters(
                drive.getPose(), drive.getMeasuredChassisSpeeds());
    }

    /**
     * Continuously tracks the hub shot target and spins the shooter without feeding.
     * Runs until interrupted.
     */
    public static Command aimForHub(Drive drive, Shooter shooter) {
        DoubleSupplier hubDistanceMeters = hubDistanceMeters(drive, shooter);
        return Commands.run(() -> {
            shooter.setTargetsForDistance(hubDistanceMeters.getAsDouble());
            shooter.stopKicker();
        }, shooter)
                .withName("AutoAimForHub");
    }

    /**
     * Continuously tracks the hub shot target and enables kicker once at setpoint.
     * Runs until interrupted.
     */
    public static Command shootHub(Drive drive, Shooter shooter) {
        DoubleSupplier hubDistanceMeters = hubDistanceMeters(drive, shooter);
        return Commands.run(() -> {
            shooter.setTargetsForDistance(hubDistanceMeters.getAsDouble());
            if (shooter.atSetpoint()) {
                shooter.setKickerTorqueAmps(ShooterConstants.DEFAULT_KICKER_TORQUE_AMPS);
            } else {
                shooter.stopKicker();
            }
        }, shooter)
                .finallyDo(interrupted -> shooter.stopKicker())
                .withName("AutoShootHub");
    }
}
