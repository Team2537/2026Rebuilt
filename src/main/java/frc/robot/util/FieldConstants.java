package frc.robot.util;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
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

  public static final int BLUE_HUB_TAG_ID = 26;
  public static final int RED_HUB_TAG_ID = 10;

  private static final String REBUILT_LAYOUT_TOKEN = "rebuilt";
  private static final List<Path> LAYOUT_SEARCH_DIRECTORIES =
      List.of(
          Filesystem.getDeployDirectory().toPath(),
          Path.of("src", "main", "deploy"),
          Path.of("build", "deploy"));

  public static final AprilTagFieldLayout TAG_LAYOUT = loadRebuiltLayout();
  public static final double FIELD_LENGTH_METERS = TAG_LAYOUT.getFieldLength();
  public static final double FIELD_WIDTH_METERS = TAG_LAYOUT.getFieldWidth();
  private static final double HUB_TARGET_X_METERS = 4.6;
  private static final double HUB_TARGET_Y_METERS = getTagY(BLUE_HUB_TAG_ID, 4.0213534);
  private static final Translation2d BLUE_HUB_TARGET_TRANSLATION =
      new Translation2d(HUB_TARGET_X_METERS, HUB_TARGET_Y_METERS);
  private static final Translation2d RED_HUB_TARGET_TRANSLATION =
      new Translation2d(
          FIELD_LENGTH_METERS - HUB_TARGET_X_METERS,
          HUB_TARGET_Y_METERS);

  /** Returns the alliance-specific hub scoring target position on the field. */
  public static Translation2d getHubTargetTranslation() {
    return isRedAlliance() ? RED_HUB_TARGET_TRANSLATION : BLUE_HUB_TARGET_TRANSLATION;
  }

  /** Returns the alliance-specific hub reference tag ID. */
  public static int getAllianceHubTagId() {
    return isRedAlliance() ? RED_HUB_TAG_ID : BLUE_HUB_TAG_ID;
  }

  /** Returns the field heading from the robot pose toward the hub target, or null if unavailable. */
  public static Rotation2d getHubFacingHeading(Pose2d robotPose) {
    if (robotPose == null) {
      return null;
    }
    Translation2d hubTarget = getHubTargetTranslation();
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

  private static double getTagY(int tagId, double defaultY) {
    return TAG_LAYOUT.getTagPose(tagId)
        .map(tagPose -> tagPose.getY())
        .orElse(defaultY);
  }

  private static boolean isRedAlliance() {
    return DriverStation.getAlliance().map(alliance -> alliance == DriverStation.Alliance.Red).orElse(false);
  }
}
