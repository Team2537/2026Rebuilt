package frc.robot.autos;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.List;
import org.junit.jupiter.api.Test;

class PathPlannerEventMarkerCommandIntegrityTest {
    private static final ObjectMapper MAPPER = new ObjectMapper();
    private static final Path PATHPLANNER_PATHS_DIR =
            Path.of("src/main/deploy/pathplanner/paths").toAbsolutePath();
    private static final List<String> COMMAND_MARKER_PREFIXES = List.of("Intake", "Shooter", "Transfer", "Drive");

    @Test
    void manipulatorLikeEventMarkersMustCarryMatchingNamedCommands() throws IOException {
        assertTrue(Files.isDirectory(PATHPLANNER_PATHS_DIR), "Missing PathPlanner paths dir: " + PATHPLANNER_PATHS_DIR);

        try (var paths = Files.list(PATHPLANNER_PATHS_DIR)) {
            List<Path> pathFiles = paths
                    .filter(path -> path.getFileName().toString().endsWith(".path"))
                    .sorted()
                    .toList();
            assertTrue(!pathFiles.isEmpty(), "No PathPlanner .path files found in " + PATHPLANNER_PATHS_DIR);

            for (Path pathFile : pathFiles) {
                JsonNode root = MAPPER.readTree(pathFile.toFile());
                for (JsonNode marker : root.path("eventMarkers")) {
                    String markerName = marker.path("name").asText();
                    if (!isManipulatorLikeMarker(markerName)) {
                        continue;
                    }

                    JsonNode command = marker.get("command");
                    assertNotNull(command, "Marker is missing command field: " + pathFile + " marker=" + markerName);
                    assertTrue(!command.isNull(), "Marker command is null: " + pathFile + " marker=" + markerName);
                    assertEquals(
                            "named",
                            command.path("type").asText(),
                            "Marker should use a named command: " + pathFile + " marker=" + markerName);
                    assertEquals(
                            markerName,
                            command.path("data").path("name").asText(),
                            "Marker name must match referenced named command: " + pathFile + " marker=" + markerName);
                }
            }
        }
    }

    private static boolean isManipulatorLikeMarker(String markerName) {
        for (String prefix : COMMAND_MARKER_PREFIXES) {
            if (markerName.startsWith(prefix)) {
                return true;
            }
        }
        return false;
    }
}
