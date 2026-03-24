package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIOSim;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.vision.VisionConsensus;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class RobotStateTest {
    @BeforeEach
    void setUp() {
        HAL.initialize(500, 0);
        AutoBuilder.resetForTesting();
        DriverStationSim.setDsAttached(true);
        DriverStationSim.setAutonomous(false);
        DriverStationSim.setTest(false);
        DriverStationSim.setEStop(false);
        DriverStationSim.setEnabled(true);
        DriverStationSim.notifyNewData();
    }

    @Test
    void fusedPoseTracksVisionConsensusWhileDriveStillExposesDeadReckoningSeparately() {
        Drive drive = createSimDrive();
        RobotState.initialize(drive);
        RobotState robotState = RobotState.getInstance();
        robotState.setPose(Pose2d.kZero);

        for (int i = 1; i <= 4; i++) {
            double timestampSeconds = i * 0.02;
            robotState.addDriveSample(
                    timestampSeconds,
                    Rotation2d.kZero,
                    drive.getModulePositionsSnapshot(),
                    Pose2d.kZero,
                    0.0);
            robotState.addVisionConsensus(new VisionConsensus(
                    timestampSeconds,
                    new Pose2d(1.2, 0.0, Rotation2d.kZero),
                    0.9,
                    0.05,
                    0.10,
                    1,
                    2,
                    true));
        }

        assertTrue(robotState.getPose().getX() > 0.6,
                "Expected fused pose to move meaningfully toward good vision consensus.");
        assertEquals(0.0, robotState.getDeadReckonedPose().getX(), 1e-9,
                "Dead reckoning should remain separate from fused pose corrections.");
        assertEquals(robotState.getPose().getX(), drive.getPose().getX(), 1e-9,
                "Drive#getPose should expose the fused pose once RobotState is attached.");
        assertEquals(robotState.getDeadReckonedPose().getX(), drive.getDeadReckonedPose().getX(), 1e-9,
                "Drive should still expose dead reckoning directly for diagnostics/sim.");
    }

    @Test
    void repeatedHighConfidenceDisagreementTriggersRecoveryMode() {
        Drive drive = createSimDrive();
        RobotState.initialize(drive);
        RobotState robotState = RobotState.getInstance();
        robotState.setPose(Pose2d.kZero);

        for (int i = 1; i <= 6; i++) {
            double timestampSeconds = i * 0.02;
            robotState.addDriveSample(
                    timestampSeconds,
                    Rotation2d.kZero,
                    drive.getModulePositionsSnapshot(),
                    Pose2d.kZero,
                    0.85);
            robotState.addVisionConsensus(new VisionConsensus(
                    timestampSeconds,
                    new Pose2d(3.5, 0.0, Rotation2d.kZero),
                    0.95,
                    0.50,
                    0.10,
                    2,
                    4,
                    true));
        }

        assertEquals(RobotState.PoseMode.RECOVERING, robotState.getPoseMode());
        assertTrue(robotState.getPose().getX() > 1.5,
                "Recovery mode should let strong vision pull fused translation aggressively.");
        assertTrue(robotState.getSlipScore() >= 0.8,
                "RobotState should surface the latest drive slip score for consumers.");
    }

    private static Drive createSimDrive() {
        GyroIOSim gyro = new GyroIOSim(Drive.getModuleTranslations());
        Drive drive = new Drive(
                gyro,
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));
        gyro.setModulePositionsSupplier(drive::getModulePositionsForSim);
        return drive;
    }
}
