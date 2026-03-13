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
  public static final double ALLIANCE_ZONE_DEPTH_METERS = Units.inchesToMeters(158.6);
  public static final double ROBOT_BUMPER_LENGTH_METERS = 0.9;
  public static final int BLUE_HUB_BACK_BLOCK_UPPER_TAG_ID = 24;
  public static final int BLUE_HUB_BACK_BLOCK_LOWER_TAG_ID = 27;
  private static final double HUB_TARGET_X_METERS = 4.6;
  private static final double HUB_TARGET_Y_METERS = getTagY(BLUE_HUB_TAG_ID, 4.0213534);
  private static final double PASS_TARGET_X_FROM_ALLIANCE_WALL_METERS = ALLIANCE_ZONE_DEPTH_METERS * 0.5;
  private static final double PASS_TARGET_WALL_MARGIN_METERS = 0.8;
  private static final double PASS_TARGET_HUB_CLEARANCE_METERS = 1.6;
  private static final double HUB_BACK_BLOCK_UPPER_Y_METERS =
      getTagY(BLUE_HUB_BACK_BLOCK_UPPER_TAG_ID, 4.6247558);
  private static final double HUB_BACK_BLOCK_LOWER_Y_METERS =
      getTagY(BLUE_HUB_BACK_BLOCK_LOWER_TAG_ID, 3.417951);
  private static final Translation2d BLUE_HUB_TARGET_TRANSLATION =
      new Translation2d(HUB_TARGET_X_METERS, HUB_TARGET_Y_METERS);
  private static final Translation2d RED_HUB_TARGET_TRANSLATION =
      new Translation2d(
          FIELD_LENGTH_METERS - HUB_TARGET_X_METERS,
          HUB_TARGET_Y_METERS);
  private static final double PASS_TARGET_LOWER_Y_METERS =
      clamp(
          HUB_TARGET_Y_METERS - PASS_TARGET_HUB_CLEARANCE_METERS,
          PASS_TARGET_WALL_MARGIN_METERS,
          FIELD_WIDTH_METERS - PASS_TARGET_WALL_MARGIN_METERS);
  private static final double PASS_TARGET_UPPER_Y_METERS =
      clamp(
          HUB_TARGET_Y_METERS + PASS_TARGET_HUB_CLEARANCE_METERS,
          PASS_TARGET_WALL_MARGIN_METERS,
          FIELD_WIDTH_METERS - PASS_TARGET_WALL_MARGIN_METERS);

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
    return getHeadingToTarget(robotPose, getHubTargetTranslation());
  }

  /** Returns whether the robot is allowed to launch into its alliance hub. */
  public static boolean isInAllianceZone(Pose2d robotPose) {
    if (robotPose == null) {
      return false;
    }
    double halfRobotLengthMeters = ROBOT_BUMPER_LENGTH_METERS * 0.5;
    double allianceZoneBoundaryX = getAllianceZoneBoundaryX();
    return isRedAlliance()
        ? robotPose.getX() + halfRobotLengthMeters >= allianceZoneBoundaryX
        : robotPose.getX() - halfRobotLengthMeters <= allianceZoneBoundaryX;
  }

  /** Returns the field X coordinate of the alliance-zone boundary for the current alliance. */
  public static double getAllianceZoneBoundaryX() {
    return isRedAlliance()
        ? FIELD_LENGTH_METERS - ALLIANCE_ZONE_DEPTH_METERS
        : ALLIANCE_ZONE_DEPTH_METERS;
  }

  /** Returns the field X coordinate of the opponent alliance-zone boundary. */
  public static double getOpponentAllianceZoneBoundaryX() {
    return isRedAlliance()
        ? ALLIANCE_ZONE_DEPTH_METERS
        : FIELD_LENGTH_METERS - ALLIANCE_ZONE_DEPTH_METERS;
  }

  /** Returns whether the robot overlaps the opponent alliance zone. */
  public static boolean isInOpponentAllianceZone(Pose2d robotPose) {
    if (robotPose == null) {
      return false;
    }
    double halfRobotLengthMeters = ROBOT_BUMPER_LENGTH_METERS * 0.5;
    double opponentAllianceZoneBoundaryX = getOpponentAllianceZoneBoundaryX();
    return isRedAlliance()
        ? robotPose.getX() - halfRobotLengthMeters <= opponentAllianceZoneBoundaryX
        : robotPose.getX() + halfRobotLengthMeters >= opponentAllianceZoneBoundaryX;
  }

  /** Returns whether the robot is in the neutral X band between the two alliance zones. */
  public static boolean isInNeutralField(Pose2d robotPose) {
    return robotPose != null
        && !isInAllianceZone(robotPose)
        && !isInOpponentAllianceZone(robotPose);
  }

  /** Returns whether the robot is in the no-pass band behind the hub while in neutral field. */
  public static boolean isInHubBackBlockedNeutralBand(Pose2d robotPose) {
    return robotPose != null
        && isInNeutralField(robotPose)
        && robotPose.getY() >= HUB_BACK_BLOCK_LOWER_Y_METERS
        && robotPose.getY() <= HUB_BACK_BLOCK_UPPER_Y_METERS;
  }

  /** Returns the lower Y bound of the neutral no-pass band behind the hub. */
  public static double getHubBackBlockLowerY() {
    return HUB_BACK_BLOCK_LOWER_Y_METERS;
  }

  /** Returns the upper Y bound of the neutral no-pass band behind the hub. */
  public static double getHubBackBlockUpperY() {
    return HUB_BACK_BLOCK_UPPER_Y_METERS;
  }

  /** Returns the alliance-relative pass target translation used when right-trigger passing. */
  public static Translation2d getPassTargetTranslation(Pose2d robotPose) {
    double targetY =
        robotPose != null && robotPose.getY() >= HUB_TARGET_Y_METERS
            ? PASS_TARGET_UPPER_Y_METERS
            : PASS_TARGET_LOWER_Y_METERS;
    double targetX =
        isRedAlliance()
            ? FIELD_LENGTH_METERS - PASS_TARGET_X_FROM_ALLIANCE_WALL_METERS
            : PASS_TARGET_X_FROM_ALLIANCE_WALL_METERS;
    return new Translation2d(targetX, targetY);
  }

  /** Returns the pass target pose used for visualization/logging. */
  public static Pose2d getPassTargetPose(Pose2d robotPose) {
    Translation2d translation = getPassTargetTranslation(robotPose);
    return new Pose2d(translation, Rotation2d.kZero);
  }

  /** Returns the field heading from the robot pose toward a translation, or null if unavailable. */
  public static Rotation2d getHeadingToTarget(Pose2d robotPose, Translation2d targetTranslation) {
    if (robotPose == null || targetTranslation == null) {
      return null;
    }
    Translation2d toTarget = targetTranslation.minus(robotPose.getTranslation());
    if (toTarget.getNorm() <= 1e-6) {
      return null;
    }
    return toTarget.getAngle();
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

  private static double clamp(double value, double minInclusive, double maxInclusive) {
    return Math.max(minInclusive, Math.min(maxInclusive, value));
  }

  private static boolean isRedAlliance() {
    return DriverStation.getAlliance().map(alliance -> alliance == DriverStation.Alliance.Red).orElse(false);
  }
}
