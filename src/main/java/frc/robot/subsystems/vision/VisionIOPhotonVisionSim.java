package frc.robot.subsystems.vision;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.util.FieldConstants;
import java.util.function.Supplier;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

/** Simulation implementation of the PhotonVision IO layer. */
public final class VisionIOPhotonVisionSim extends VisionIOPhotonVision {
    private static VisionSystemSim sharedVisionSim;

    private final Supplier<Pose2d> poseSupplier;
    private final PhotonCameraSim cameraSim;

    public VisionIOPhotonVisionSim(
            String name, Transform3d robotToCamera, int cameraIndex, Supplier<Pose2d> poseSupplier) {
        super(name, robotToCamera, cameraIndex);
        this.poseSupplier = poseSupplier;

        SimCameraProperties cameraProperties = new SimCameraProperties();
        cameraProperties.setCalibration(
                640,
                480,
                MatBuilder.fill(
                        Nat.N3(),
                        Nat.N3(),
                        546.947202769191,
                        0.0,
                        317.2326216443899,
                        0.0,
                        546.8805910328873,
                        256.54365866088693,
                        0.0,
                        0.0,
                        1.0),
                MatBuilder.fill(
                        Nat.N8(),
                        Nat.N1(),
                        0.04575330800554877,
                        -0.06764388351284431,
                        -0.0002716072733319333,
                        -0.0008388770763606548,
                        0.007416750430854674,
                        -0.0016856447818708886,
                        0.0027323859252879066,
                        -0.00029965498573175946));
        // Remove simulated calibration error to reduce pose jitter in sim
        cameraProperties.setCalibError(0.0, 0.0);

        this.cameraSim = new PhotonCameraSim(camera, cameraProperties);
        getOrCreateVisionSim().addCamera(cameraSim, robotToCamera);
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        getOrCreateVisionSim().update(poseSupplier.get());
        super.updateInputs(inputs);
    }

    private static synchronized VisionSystemSim getOrCreateVisionSim() {
        if (sharedVisionSim == null) {
            sharedVisionSim = new VisionSystemSim("main");
            sharedVisionSim.addAprilTags(FieldConstants.TAG_LAYOUT);
        }
        return sharedVisionSim;
    }
}
