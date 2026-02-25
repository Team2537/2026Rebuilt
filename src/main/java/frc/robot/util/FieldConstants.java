package frc.robot.util;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.List;
import java.util.Locale;
import java.util.Optional;
import java.util.stream.Stream;

/** Field-specific constants used for vision and simulation. */
public final class FieldConstants {
  private FieldConstants() {}

  // Hub / scoring target constants (game-specific, centralised here)
  public static final int HUB_TAG_ID = 26;
  public static final double HUB_TARGET_X_OFFSET_METERS = 1.0;

  private static final String REBUILT_LAYOUT_TOKEN = "rebuilt";
  private static final List<Path> LAYOUT_SEARCH_DIRECTORIES =
      List.of(
          Filesystem.getDeployDirectory().toPath(),
          Path.of("src", "main", "deploy"),
          Path.of("build", "deploy"));

  public static final AprilTagFieldLayout TAG_LAYOUT = loadRebuiltLayout();
  public static final double FIELD_LENGTH_METERS = TAG_LAYOUT.getFieldLength();
  public static final double FIELD_WIDTH_METERS = TAG_LAYOUT.getFieldWidth();

  /** Returns the hub scoring target position on the field, or null if the tag layout lacks the hub tag. */
  public static Translation2d getHubTargetTranslation() {
    return TAG_LAYOUT.getTagPose(HUB_TAG_ID)
        .map(tagPose -> new Translation2d(
            tagPose.getX() + HUB_TARGET_X_OFFSET_METERS,
            tagPose.getY()))
        .orElse(null);
  }

  /** Returns the field heading from the robot pose toward the hub target, or null if unavailable. */
  public static Rotation2d getHubFacingHeading(Pose2d robotPose) {
    if (robotPose == null) {
      return null;
    }
    Translation2d hubTarget = getHubTargetTranslation();
    if (hubTarget == null) {
      return null;
    }
    Translation2d toHub = hubTarget.minus(robotPose.getTranslation());
    if (toHub.getNorm() <= 1e-6) {
      return null;
    }
    return toHub.getAngle();
  }

  private static AprilTagFieldLayout loadRebuiltLayout() {
    for (Path directory : LAYOUT_SEARCH_DIRECTORIES) {
      Optional<Path> candidate = findRebuiltLayoutFile(directory);
      if (candidate.isPresent()) {
        try {
          return new AprilTagFieldLayout(candidate.get());
        } catch (IOException ex) {
          throw new IllegalStateException(
              "Failed to load rebuilt AprilTag layout from " + candidate.get(), ex);
        }
      }
    }

    DriverStation.reportError(
        "Rebuilt field layout JSON not found. Searched: " + LAYOUT_SEARCH_DIRECTORIES, false);
    return AprilTagFieldLayout.loadField(edu.wpi.first.apriltag.AprilTagFields.kDefaultField);
  }

  private static Optional<Path> findRebuiltLayoutFile(Path directory) {
    if (!Files.isDirectory(directory)) {
      return Optional.empty();
    }

    try (Stream<Path> files = Files.list(directory)) {
      return files.filter(FieldConstants::isRebuiltLayout).sorted().findFirst();
    } catch (IOException ex) {
      throw new IllegalStateException("Failed to scan for rebuilt field layouts in " + directory, ex);
    }
  }

  private static boolean isRebuiltLayout(Path path) {
    if (!Files.isRegularFile(path)) {
      return false;
    }
    String lower = path.getFileName().toString().toLowerCase(Locale.ROOT);
    return lower.endsWith(".json") && lower.contains(REBUILT_LAYOUT_TOKEN);
  }
}
