package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/** Dashboard-backed autonomous routine selector. */
public final class AutoSelector {
    private static final AutoRoutines.AutoRoutine DEFAULT_ROUTINE =
            new AutoRoutines.AutoRoutine("none", () -> Commands.none().withName("AutoNone"));

    private final LoggedDashboardChooser<AutoRoutines.AutoRoutine> chooser =
            new LoggedDashboardChooser<>("auto");

    public AutoSelector(List<AutoRoutines.AutoRoutine> routines) {
        List<String> availableRoutineNames = new ArrayList<>();
        chooser.addDefaultOption(DEFAULT_ROUTINE.name(), DEFAULT_ROUTINE);
        availableRoutineNames.add(DEFAULT_ROUTINE.name());
        for (AutoRoutines.AutoRoutine routine : routines) {
            if (DEFAULT_ROUTINE.name().equals(routine.name())) {
                continue;
            }
            chooser.addOption(routine.name(), routine);
            availableRoutineNames.add(routine.name());
        }
        String[] routineNames = availableRoutineNames.toArray(String[]::new);
        Logger.recordOutput("auto/selector/options", routineNames);
        Logger.recordOutput("auto/selector/optionsCount", routineNames.length);
    }

    public Command getSelectedCommand() {
        AutoRoutines.AutoRoutine dashboardSelection = chooser.get();
        AutoRoutines.AutoRoutine selectedRoutine =
                dashboardSelection != null ? dashboardSelection : DEFAULT_ROUTINE;
        Command selectedCommand = selectedRoutine.createCommand();

        Logger.recordOutput("auto/selector/selectedRoutineName", selectedRoutine.name());
        Logger.recordOutput(
                "auto/selector/selectedRoutineIsDefault",
                DEFAULT_ROUTINE.name().equals(selectedRoutine.name()));
        Logger.recordOutput("auto/selector/hasDashboardSelection", dashboardSelection != null);
        Logger.recordOutput("auto/selector/selectedCommandName", getCommandName(selectedCommand));

        return selectedCommand;
    }

    private static String getCommandName(Command command) {
        if (command == null) {
            return "UnnamedCommand";
        }
        String name = command.getName();
        if (name == null || name.isBlank()) {
            String className = command.getClass().getSimpleName();
            return className == null || className.isBlank() ? "UnnamedCommand" : className;
        }
        return name;
    }
}
