package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.List;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/** Dashboard-backed autonomous routine selector. */
public final class AutoSelector {
    private static final AutoRoutines.AutoRoutine DEFAULT_ROUTINE =
            new AutoRoutines.AutoRoutine("none", () -> Commands.none().withName("AutoNone"));

    private final LoggedDashboardChooser<AutoRoutines.AutoRoutine> chooser =
            new LoggedDashboardChooser<>("auto");
    private final Map<String, Command> routinesByName = new LinkedHashMap<>();

    public AutoSelector(List<AutoRoutines.AutoRoutine> routines) {
        List<String> availableRoutineNames = new ArrayList<>();
        chooser.addDefaultOption(DEFAULT_ROUTINE.name(), DEFAULT_ROUTINE);
        routinesByName.put(DEFAULT_ROUTINE.name(), DEFAULT_ROUTINE.createCommand());
        availableRoutineNames.add(DEFAULT_ROUTINE.name());
        for (AutoRoutines.AutoRoutine routine : routines) {
            if (DEFAULT_ROUTINE.name().equals(routine.name())) {
                continue;
            }
            chooser.addOption(routine.name(), routine);
            availableRoutineNames.add(routine.name());
            routinesByName.put(routine.name(), routine.createCommand());
        }
        String[] routineNames = availableRoutineNames.toArray(String[]::new);
        Logger.recordOutput("Auto/selector/options", routineNames);
        Logger.recordOutput("Auto/selector/optionsCount", routineNames.length);
    }

    public Command getSelectedCommand() {
        AutoRoutines.AutoRoutine selectedRoutine = getSelectedRoutine();
        Command selectedCommand =
                routinesByName.getOrDefault(selectedRoutine.name(), routinesByName.get(DEFAULT_ROUTINE.name()));
        if (selectedCommand == null) {
            selectedCommand = DEFAULT_ROUTINE.createCommand();
            routinesByName.put(DEFAULT_ROUTINE.name(), selectedCommand);
        }

        Logger.recordOutput("Auto/selector/selectedRoutineName", selectedRoutine.name());
        Logger.recordOutput(
                "Auto/selector/selectedRoutineIsDefault",
                DEFAULT_ROUTINE.name().equals(selectedRoutine.name()));
        Logger.recordOutput("Auto/selector/hasDashboardSelection", chooser.get() != null);
        Logger.recordOutput("Auto/selector/selectedCommandName", getCommandName(selectedCommand));

        return selectedCommand;
    }

    public AutoRoutines.AutoRoutine getSelectedRoutine() {
        AutoRoutines.AutoRoutine dashboardSelection = chooser.get();
        AutoRoutines.AutoRoutine selectedRoutine =
                dashboardSelection != null ? dashboardSelection : DEFAULT_ROUTINE;
        return selectedRoutine;
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
