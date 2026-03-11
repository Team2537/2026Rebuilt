package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.TimedRobot;
import java.util.List;

/** Constants for vision filtering, camera placement, and simulation camera modeling. */
public final class VisionConstants {
    public record CameraConfig(String name, Transform3d robotToCamera) {}

    public static final List<CameraConfig> CAMERA_CONFIGS = List.of(
            new CameraConfig(
                    "Front Right",
                    new Transform3d(
                            new Translation3d(-0.325145, -0.299705, 0.13633),
                            new Rotation3d(
                                    Units.degreesToRadians(0.5),
                                    Units.degreesToRadians(-45.0),
                                    Units.degreesToRadians(160.0)))),
            new CameraConfig(
                    "Front Left",
                    new Transform3d(
                            new Translation3d(-0.325145, 0.299705, 0.13633),
                            new Rotation3d(
                                    Units.degreesToRadians(2.0),
                                    Units.degreesToRadians(-23.0),
                                    Units.degreesToRadians(197.0)))));

    public static final double MAX_AMBIGUITY = 0.3;
    public static final double MAX_Z_ERROR = 0.75;
    public static final double LINEAR_STD_DEV_BASELINE = 0.08;
    public static final double ANGULAR_STD_DEV_BASELINE = 0.18;
    public static final double ESTIMATOR_ANGULAR_STD_DEV_RAD = 999.0;
    public static final double HUB_TAG_CLUSTER_RADIUS_METERS = Units.inchesToMeters(60.0);
    public static final double HUB_YAW_MAX_AMBIGUITY = MAX_AMBIGUITY;
    public static final double HUB_YAW_MAX_DISTANCE_METERS = 6.0;
    public static final double HUB_YAW_MAX_AGE_SECONDS = 0.25;
    public static final double MAX_VISION_TRANSLATION_DELTA_METERS = 0.5;
    public static final double MAX_VISION_HEADING_DELTA_DEGREES = 20.0;
    public static final double VISION_JUMP_TRANSLATION_THRESHOLD_METERS = 0.5;
    public static final double VISION_JUMP_HEADING_THRESHOLD_DEGREES = 20.0;
    public static final double UNIFIED_RAW_POSE_MAX_AGE_SECONDS = 0.25;
    public static final double UNIFIED_POSE_MAX_AGE_SECONDS = 0.5;
    public static final boolean ENABLE_VISION_EVENT_LOGS = false;
    public static final boolean ENABLE_VERBOSE_VISION_DIAGNOSTICS = false;

    public static final int SIM_CAMERA_WIDTH = 640;
    public static final int SIM_CAMERA_HEIGHT = 480;
    public static final double[] SIM_CAMERA_INTRINSICS = {
            546.947202769191, 0.0, 317.2326216443899,
            0.0, 546.8805910328873, 256.54365866088693,
            0.0, 0.0, 1.0
    };
    public static final double[] SIM_CAMERA_DISTORTION = {
            0.04575330800554877,
            -0.06764388351284431,
            -0.0002716072733319333,
            -0.0008388770763606548,
            0.007416750430854674,
            -0.0016856447818708886,
            0.0027323859252879066,
            -0.00029965498573175946
    };
    public static final double SIM_CALIB_ERROR_PIXELS = 0.0;
    public static final double SIM_UPDATE_PERIOD_SEC = TimedRobot.kDefaultPeriod;
    public static final String SIM_VISION_SYSTEM_NAME = "main";

    private VisionConstants() {}
}
