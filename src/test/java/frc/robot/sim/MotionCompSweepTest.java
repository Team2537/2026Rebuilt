package frc.robot.sim;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.Locale;
import org.junit.jupiter.api.Test;

class MotionCompSweepTest {
    @Test
    void sweepTeleopMotionCompBlend() throws Exception {
        double[] blends = {0.0, 0.25, 0.5, 0.75, 1.0};
        for (double blend : blends) {
            SmartDashboard.putNumber("Shooting/TeleopMotionCompSetpointBlend", blend);
            ShootOnMoveTeleopReplayLogSimTest.Result result = ShootOnMoveTeleopReplayLogSimTest.runReplay();
            System.out.printf(
                    Locale.US,
                    "blend=%.2f moved=%.3f feed=%s samples=%d maxMiss=%.4f\n",
                    blend,
                    result.postOnsetTranslationMeters(),
                    result.sawMovingFeed(),
                    result.movingDescentSamples(),
                    result.maxMovingDescentMissMeters());
        }
    }
}
