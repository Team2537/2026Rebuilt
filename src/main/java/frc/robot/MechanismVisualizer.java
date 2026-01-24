package frc.robot;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import org.littletonrobotics.junction.Logger;

/** Helper for visualizing component poses in AdvantageScope. */
public final class MechanismVisualizer {
  private static final int COMPONENT_COUNT = 3;
  private static final List<Integer> TURRET_COMPONENT_INDICES = List.of(0);
  private static final List<Integer> INTAKE_COMPONENT_INDICES = List.of(1, 2);
  private static final double TURRET_POSE_Z_METERS = 1.0;
  private static final double INTAKE_POSE_Z_METERS = 1.0;
  private static final List<Pose3d> MECHANISM_POSES =
      new ArrayList<>(Collections.nCopies(COMPONENT_COUNT, new Pose3d()));

  private MechanismVisualizer() {}

  public static void setIntakeYaw(Rotation2d yaw) {
    Pose3d pose =
        new Pose3d(
            new Translation3d(0.0, 0.0, INTAKE_POSE_Z_METERS),
            new Rotation3d(0.0, 0.0, yaw.getRadians()));
    for (int index : INTAKE_COMPONENT_INDICES) {
      if (index >= 0 && index < MECHANISM_POSES.size()) {
        MECHANISM_POSES.set(index, pose);
      }
    }
  }

  public static void setTurretYaw(Rotation2d yaw) {
    Pose3d pose =
        new Pose3d(
            new Translation3d(0.0, 0.0, TURRET_POSE_Z_METERS),
            new Rotation3d(0.0, 0.0, yaw.getRadians()));
    for (int index : TURRET_COMPONENT_INDICES) {
      if (index >= 0 && index < MECHANISM_POSES.size()) {
        MECHANISM_POSES.set(index, pose);
      }
    }
  }

  public static void updatePoses() {
    Logger.recordOutput("mechanismPoses", MECHANISM_POSES.toArray(Pose3d[]::new));
  }
}
