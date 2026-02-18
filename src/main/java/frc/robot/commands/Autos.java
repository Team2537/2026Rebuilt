package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import java.util.Optional;
import java.util.function.Supplier;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/** Collection of autonomous routine factories. */
public final class Autos {
    private static final Supplier<Command> DEFAULT_ROUTINE = Commands::none;
    private final LoggedDashboardChooser<Supplier<Command>> chooser =
            new LoggedDashboardChooser<>("auto");

    public Autos(Drive drive) {
        chooser.addDefaultOption("none", DEFAULT_ROUTINE);
        chooser.addOption("stopWithX", () -> Commands.runOnce(drive::stopWithX, drive));
    }

    public Command getSelectedRoutine() {
        return Optional.ofNullable(chooser.get())
                .map(Supplier::get)
                .orElseGet(DEFAULT_ROUTINE);
    }
}
