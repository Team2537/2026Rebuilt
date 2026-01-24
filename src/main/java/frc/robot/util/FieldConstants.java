package frc.robot.util;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.wpilibj.Filesystem;
import java.io.IOException;
import java.nio.file.Path;
import java.util.Locale;
import java.util.stream.Stream;

/** Field-specific constants used for vision and simulation. */
public final class FieldConstants {
  private FieldConstants() {}

  public static final AprilTagFieldLayout TAG_LAYOUT = loadRebuiltLayout();
  public static final double FIELD_LENGTH_METERS = TAG_LAYOUT.getFieldLength();
  public static final double FIELD_WIDTH_METERS = TAG_LAYOUT.getFieldWidth();

  private static AprilTagFieldLayout loadRebuiltLayout() {
    Path deployDir = Filesystem.getDeployDirectory().toPath();
    try (Stream<Path> files = java.nio.file.Files.list(deployDir)) {
      Path layoutPath =
          files
              .filter(path -> path.getFileName().toString().toLowerCase(Locale.ROOT).contains("rebuilt"))
              .filter(path -> path.getFileName().toString().toLowerCase(Locale.ROOT).endsWith(".json"))
              .findFirst()
              .orElseThrow(
                  () ->
                      new IllegalStateException(
                          "Rebuilt field layout JSON not found in deploy directory: " + deployDir));
      return new AprilTagFieldLayout(layoutPath);
    } catch (IOException ex) {
      throw new IllegalStateException("Failed to load Rebuilt AprilTag layout", ex);
    }
  }
}
