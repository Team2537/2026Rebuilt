package frc.robot.subsystems.vision;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import org.junit.jupiter.api.Test;
import org.photonvision.targeting.PhotonTrackedTarget;

class VisionIOPhotonVisionTest {
    @Test
    void singleTagFallbackPreservesAllUsableTargets() {
        List<PhotonTrackedTarget> targets = List.of(
                createTarget(1, 4.0, 0.20),
                createTarget(2, 1.5, 0.05));
        Set<Integer> tagIds = new HashSet<>();
        List<VisionIO.PoseObservation> observations = new ArrayList<>();

        VisionIOPhotonVision.addSingleTagObservations(
                targets,
                1.25,
                new Transform3d(),
                tagIds,
                observations);

        assertEquals(2, observations.size(), "Expected a pose observation for each usable single-tag target.");
        assertEquals(Set.of(1, 2), tagIds);
        assertTrue(
                observations.stream().anyMatch(observation -> observation.averageTagDistance() < 2.0),
                "Expected the closer second target to be preserved in fallback observations.");
    }

    private static PhotonTrackedTarget createTarget(int fiducialId, double distanceMeters, double ambiguity) {
        PhotonTrackedTarget target = new PhotonTrackedTarget();
        target.fiducialId = fiducialId;
        target.bestCameraToTarget =
                new Transform3d(new Translation3d(distanceMeters, 0.0, 0.0), new Rotation3d());
        target.altCameraToTarget = new Transform3d();
        target.poseAmbiguity = ambiguity;
        target.detectedCorners = List.of();
        target.minAreaRectCorners = List.of();
        return target;
    }
}
