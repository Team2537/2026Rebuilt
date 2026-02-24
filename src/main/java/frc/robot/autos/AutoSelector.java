package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.List;
import java.util.Optional;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/** Dashboard-backed autonomous routine selector. */
public final class AutoSelector {
    private static final AutoRoutines.AutoRoutine DEFAULT_ROUTINE =
            new AutoRoutines.AutoRoutine("none", () -> Commands.none().withName("AutoNone"));

    private final LoggedDashboardChooser<AutoRoutines.AutoRoutine> chooser =
            new LoggedDashboardChooser<>("auto");

    public AutoSelector(List<AutoRoutines.AutoRoutine> routines) {
        chooser.addDefaultOption(DEFAULT_ROUTINE.name(), DEFAULT_ROUTINE);
        for (AutoRoutines.AutoRoutine routine : routines) {
            if (DEFAULT_ROUTINE.name().equals(routine.name())) {
                continue;
            }
            chooser.addOption(routine.name(), routine);
        }
    }

    public Command getSelectedCommand() {
        return Optional.ofNullable(chooser.get())
                .orElse(DEFAULT_ROUTINE)
                .createCommand();
    }
}
