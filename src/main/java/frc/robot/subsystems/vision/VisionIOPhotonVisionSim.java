package frc.robot.subsystems.vision;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.util.FieldConstants;
import java.util.function.Supplier;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

/** Simulation implementation of the PhotonVision IO layer. */
public final class VisionIOPhotonVisionSim extends VisionIOPhotonVision {
    private static VisionSystemSim sharedVisionSim;
    private static double lastVisionSimUpdateTimestampSec = Double.NEGATIVE_INFINITY;

    private final Supplier<Pose2d> poseSupplier;
    private final PhotonCameraSim cameraSim;

    public VisionIOPhotonVisionSim(
            String name, Transform3d robotToCamera, int cameraIndex, Supplier<Pose2d> poseSupplier) {
        super(name, robotToCamera, cameraIndex);
        this.poseSupplier = poseSupplier;

        SimCameraProperties cameraProperties = new SimCameraProperties();
        cameraProperties.setCalibration(
                VisionConstants.SIM_CAMERA_WIDTH,
                VisionConstants.SIM_CAMERA_HEIGHT,
                MatBuilder.fill(
                        Nat.N3(),
                        Nat.N3(),
                        VisionConstants.SIM_CAMERA_INTRINSICS[0],
                        VisionConstants.SIM_CAMERA_INTRINSICS[1],
                        VisionConstants.SIM_CAMERA_INTRINSICS[2],
                        VisionConstants.SIM_CAMERA_INTRINSICS[3],
                        VisionConstants.SIM_CAMERA_INTRINSICS[4],
                        VisionConstants.SIM_CAMERA_INTRINSICS[5],
                        VisionConstants.SIM_CAMERA_INTRINSICS[6],
                        VisionConstants.SIM_CAMERA_INTRINSICS[7],
                        VisionConstants.SIM_CAMERA_INTRINSICS[8]),
                MatBuilder.fill(
                        Nat.N8(),
                        Nat.N1(),
                        VisionConstants.SIM_CAMERA_DISTORTION[0],
                        VisionConstants.SIM_CAMERA_DISTORTION[1],
                        VisionConstants.SIM_CAMERA_DISTORTION[2],
                        VisionConstants.SIM_CAMERA_DISTORTION[3],
                        VisionConstants.SIM_CAMERA_DISTORTION[4],
                        VisionConstants.SIM_CAMERA_DISTORTION[5],
                        VisionConstants.SIM_CAMERA_DISTORTION[6],
                        VisionConstants.SIM_CAMERA_DISTORTION[7]));
        // Remove simulated calibration error to reduce pose jitter in sim
        cameraProperties.setCalibError(
                VisionConstants.SIM_CALIB_ERROR_PIXELS,
                VisionConstants.SIM_CALIB_ERROR_PIXELS);

        this.cameraSim = new PhotonCameraSim(camera, cameraProperties);
        getOrCreateVisionSim().addCamera(cameraSim, robotToCamera);
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        updateVisionSimIfNeeded(poseSupplier.get());
        super.updateInputs(inputs);
    }

    private static synchronized void updateVisionSimIfNeeded(Pose2d pose) {
        if (pose == null) {
            return;
        }
        double nowSec = Timer.getFPGATimestamp();
        if (nowSec - lastVisionSimUpdateTimestampSec < VisionConstants.SIM_UPDATE_PERIOD_SEC * 0.5) {
            return;
        }

        getOrCreateVisionSim().update(pose);
        lastVisionSimUpdateTimestampSec = nowSec;
    }

    private static synchronized VisionSystemSim getOrCreateVisionSim() {
        if (sharedVisionSim == null) {
            sharedVisionSim = new VisionSystemSim(VisionConstants.SIM_VISION_SYSTEM_NAME);
            sharedVisionSim.addAprilTags(FieldConstants.TAG_LAYOUT);
        }
        return sharedVisionSim;
    }

    static synchronized void resetSharedVisionSim() {
        sharedVisionSim = null;
        lastVisionSimUpdateTimestampSec = Double.NEGATIVE_INFINITY;
    }
}
