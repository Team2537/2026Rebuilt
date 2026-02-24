package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.transfer.Transfer;
import java.util.function.DoubleSupplier;

/**
 * Multi-subsystem shoot command compositions shared between teleop and autonomous.
 */
public final class ShootCommands {
    private ShootCommands() {}

    /**
     * Continuously aims the shooter at the supplied distance and feeds via the transfer
     * once the shooter reports ready to fire. Runs until interrupted.
     */
    public static Command shootWithFeed(
            Shooter shooter, Transfer transfer, DoubleSupplier distanceMeters) {
        return Commands.parallel(
                shooter.shoot(distanceMeters),
                transfer.runWhen(shooter::readyToFire))
                .withName("ShootWithFeed");
    }
}
