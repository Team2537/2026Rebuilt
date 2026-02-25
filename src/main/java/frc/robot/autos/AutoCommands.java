package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.coordination.shooting.ShootCoordinator;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.Shooter;
import java.util.function.DoubleSupplier;

/** Reusable autonomous command primitives intended for composition in PathPlanner. */
public final class AutoCommands {
    private AutoCommands() {}

    private static DoubleSupplier createHubDistanceSupplier(Drive drive, Shooter shooter) {
        return () -> shooter.getMotionCompensatedHubDistanceMeters(
                drive.getPose(), drive.getMeasuredChassisSpeeds());
    }

    /**
     * Continuously tracks the hub shot target and spins the shooter without feeding.
     * Runs until interrupted.
     */
    public static Command aimForHub(
            Drive drive, Shooter shooter, ShootCoordinator shootCoordinator) {
        return shootCoordinator.aimForDistance(createHubDistanceSupplier(drive, shooter))
                .withName("AutoAimForHub");
    }

    /**
     * Continuously tracks the hub shot target and enables kicker/transfer once at setpoint.
     * Runs until interrupted.
     */
    public static Command shootHub(
            Drive drive, Shooter shooter, ShootCoordinator shootCoordinator) {
        return shootCoordinator.shootForDistance(createHubDistanceSupplier(drive, shooter))
                .withName("AutoShootHub");
    }
}
