package frc.robot.autos;

import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.function.Supplier;

/** Builds the list of autonomous routine factories. */
public final class AutoRoutines {
    private AutoRoutines() {}

    public record AutoRoutine(String name, Supplier<Command> commandFactory) {
        public Command createCommand() {
            Command command = commandFactory.get();
            return command != null ? command : Commands.none().withName("AutoNone");
        }
    }

    public static List<AutoRoutine> create(Drive drive) {
        List<AutoRoutine> routines = new ArrayList<>();
        routines.add(new AutoRoutine("none", () -> Commands.none().withName("AutoNone")));
        routines.add(new AutoRoutine(
                "stopWithX",
                () -> Commands.runOnce(drive::stopWithX, drive).withName("AutoStopWithX")));
        routines.addAll(loadPathPlannerRoutines());
        return routines;
    }

    private static List<AutoRoutine> loadPathPlannerRoutines() {
        Path autosDir = Filesystem.getDeployDirectory().toPath().resolve("pathplanner").resolve("autos");
        if (!Files.isDirectory(autosDir)) {
            return List.of();
        }

        List<AutoRoutine> routines = new ArrayList<>();
        try (var autoFiles = Files.list(autosDir)) {
            autoFiles
                    .filter(path -> path.getFileName().toString().endsWith(".auto"))
                    .sorted(Comparator.comparing(path -> path.getFileName().toString()))
                    .forEach(path -> {
                        String fileName = path.getFileName().toString();
                        String autoName = fileName.substring(0, fileName.length() - ".auto".length());
                        routines.add(new AutoRoutine(
                                "pp/" + autoName,
                                () -> new PathPlannerAuto(autoName).withName("AutoPathPlanner_" + autoName)));
                    });
        } catch (IOException ignored) {
            return List.of();
        }
        return routines;
    }
}
