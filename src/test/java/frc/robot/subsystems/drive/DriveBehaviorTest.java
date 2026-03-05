package frc.robot.subsystems.drive;

import static org.junit.jupiter.api.Assertions.assertDoesNotThrow;
import static org.junit.jupiter.api.Assertions.assertEquals;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.DriveFeedforwards;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import frc.robot.generated.TunerConstants;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class DriveBehaviorTest {
    private static final double EPSILON = 1e-9;

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

    @AfterEach
    void tearDown() {
        DriverStationSim.setEnabled(false);
        DriverStationSim.notifyNewData();
    }

    @Test
    void periodicToleratesUnevenOdometrySampleCounts() {
        Drive drive = new Drive(
                new FixedGyroIO(
                        true,
                        Rotation2d.fromDegrees(15.0),
                        new double[] {0.10},
                        new Rotation2d[] {Rotation2d.fromDegrees(15.0)}),
                new FixedModuleIO(new double[] {0.10, 0.12}, new double[] {0.0, 0.0}),
                new FixedModuleIO(new double[] {0.10}, new double[] {0.0}),
                new FixedModuleIO(new double[] {0.10}, new double[] {0.0}),
                new FixedModuleIO(new double[] {0.10}, new double[] {0.0}));

        assertDoesNotThrow(drive::periodic);
    }

    @Test
    void headingResetKeepsEstimatorGyroAndSimTruthAligned() {
        GyroIOSim gyro = new GyroIOSim(Drive.getModuleTranslations());
        Drive drive = new Drive(
                gyro,
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));
        gyro.setModulePositionsSupplier(drive::getModulePositionsForSim);

        Pose2d initialPose = new Pose2d(1.2, -0.4, Rotation2d.fromDegrees(67.0));
        drive.setPose(initialPose);
        drive.resetOdometryAndHeadingToZero();

        assertEquals(0.0, drive.getPose().getRotation().getDegrees(), EPSILON);
        assertEquals(0.0, drive.getRotation().getDegrees(), EPSILON);
        assertEquals(0.0, drive.getSimGroundTruthPose().getRotation().getDegrees(), EPSILON);
        assertEquals(initialPose.getX(), drive.getPose().getX(), EPSILON);
        assertEquals(initialPose.getY(), drive.getPose().getY(), EPSILON);
    }

    @Test
    void externalFeedforwardAccelerationReachesModuleIo() {
        FixedGyroIO gyro = new FixedGyroIO(false, Rotation2d.kZero, new double[0], new Rotation2d[0]);
        FixedModuleIO fl = new FixedModuleIO(new double[] {0.10}, new double[] {0.0});
        Drive drive = new Drive(
                gyro,
                fl,
                new FixedModuleIO(new double[] {0.10}, new double[] {0.0}),
                new FixedModuleIO(new double[] {0.10}, new double[] {0.0}),
                new FixedModuleIO(new double[] {0.10}, new double[] {0.0}));

        DriveFeedforwards feedforwards = new DriveFeedforwards(
                new double[] {1.5, 0.0, 0.0, 0.0},
                new double[] {0.0, 0.0, 0.0, 0.0},
                new double[] {0.0, 0.0, 0.0, 0.0},
                new double[] {0.0, 0.0, 0.0, 0.0},
                new double[] {0.0, 0.0, 0.0, 0.0});

        drive.runVelocity(new ChassisSpeeds(), feedforwards);

        assertEquals(1.5 / TunerConstants.FrontLeft.WheelRadius, fl.lastAccelerationRadPerSecSq, EPSILON);
    }

    private static final class FixedGyroIO implements GyroIO {
        private final boolean connected;
        private final Rotation2d yawPosition;
        private final double[] timestamps;
        private final Rotation2d[] odometryYawPositions;

        private FixedGyroIO(
                boolean connected,
                Rotation2d yawPosition,
                double[] timestamps,
                Rotation2d[] odometryYawPositions) {
            this.connected = connected;
            this.yawPosition = yawPosition;
            this.timestamps = timestamps;
            this.odometryYawPositions = odometryYawPositions;
        }

        @Override
        public void updateInputs(GyroIOInputs inputs) {
            inputs.connected = connected;
            inputs.yawPosition = yawPosition;
            inputs.odometryYawTimestamps = timestamps;
            inputs.odometryYawPositions = odometryYawPositions;
        }
    }

    private static final class FixedModuleIO implements ModuleIO {
        private final double[] timestamps;
        private final double[] drivePositionsRad;
        private double lastAccelerationRadPerSecSq = 0.0;

        private FixedModuleIO(double[] timestamps, double[] drivePositionsRad) {
            this.timestamps = timestamps;
            this.drivePositionsRad = drivePositionsRad;
        }

        @Override
        public void updateInputs(ModuleIOInputs inputs) {
            inputs.driveConnected = true;
            inputs.turnConnected = true;
            inputs.turnEncoderConnected = true;
            inputs.turnPosition = Rotation2d.kZero;
            inputs.odometryTimestamps = timestamps;
            inputs.odometryDrivePositionsRad = drivePositionsRad;
            inputs.odometryTurnPositions = new Rotation2d[drivePositionsRad.length];
            for (int i = 0; i < drivePositionsRad.length; i++) {
                inputs.odometryTurnPositions[i] = Rotation2d.kZero;
            }
        }

        @Override
        public void setDriveVelocity(
                double velocityRadPerSec,
                double accelerationRadPerSecSq,
                double arbitraryFeedforward) {
            lastAccelerationRadPerSecSq = accelerationRadPerSecSq;
        }
    }
}
