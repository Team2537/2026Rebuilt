package frc.robot.coordination.shooting;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.CommandTelemetry;
import frc.robot.DashboardOverrides;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.transfer.Transfer;
import frc.robot.subsystems.transfer.TransferIO;
import frc.robot.util.FieldConstants;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class ShootingTeleopControllerXLockTest {
    private static final double X_LOCK_ANGLE_TOLERANCE_RAD = Math.toRadians(3.0);
    private static Fixture fixture;

    @BeforeEach
    void setUp() {
        HAL.initialize(500, 0);
        CommandScheduler.getInstance().cancelAll();

        DriverStationSim.setDsAttached(true);
        DriverStationSim.setAutonomous(false);
        DriverStationSim.setTest(false);
        DriverStationSim.setEStop(false);
        DriverStationSim.setEnabled(true);
        DriverStationSim.notifyNewData();

        if (fixture == null) {
            fixture = new Fixture();
        } else {
            fixture.resetForTest();
        }
    }

    @AfterEach
    void tearDown() {
        CommandScheduler.getInstance().cancelAll();
        DriverStationSim.setEnabled(false);
        DriverStationSim.notifyNewData();
    }

    @Test
    void locksWithXByTheTimeFeedingBeginsWhenIdle() {
        AtomicBoolean aimReady = new AtomicBoolean(true);

        Command command = fixture.createShootCommand(0.0, 0.0, 0.0, aimReady::get);
        CommandScheduler.getInstance().schedule(command);

        boolean sawFeedStart = false;
        boolean sawLockWhenFeeding = false;
        StringBuilder trace = new StringBuilder();
        for (int i = 0; i < 8; i++) {
            fixture.runSchedulerCycles(1);
            boolean xLock = fixture.isDriveInXLock();
            boolean feeding = fixture.coordinator.isActivelyFeeding();
            trace.append(String.format("c%d[x=%s,f=%s] ", i + 1, xLock, feeding));
            if (feeding) {
                sawFeedStart = true;
                sawLockWhenFeeding = xLock;
                break;
            }
        }

        assertTrue(
                sawFeedStart,
                "Expected feeding to begin during the observation window. Trace: " + trace);
        assertTrue(
                sawLockWhenFeeding,
                "Drive should be X-locked by the time feeding begins when idle. Trace: " + trace);
    }

    @Test
    void doesNotLockWithXWhenAimIsNotReady() {
        AtomicBoolean aimReady = new AtomicBoolean(false);

        Command command = fixture.createShootCommand(0.0, 0.0, 0.0, aimReady::get);
        CommandScheduler.getInstance().schedule(command);

        fixture.runSchedulerCycles(4);

        assertFalse(fixture.coordinator.isActivelyFeeding());
        assertFalse(fixture.isDriveInXLock(), "Aim not ready should block X-lock.");
    }

    @Test
    void doesNotLockWithXWhenDriverCommandsTranslation() {
        AtomicBoolean aimReady = new AtomicBoolean(true);

        Command command = fixture.createShootCommand(0.45, 0.0, 0.0, aimReady::get);
        CommandScheduler.getInstance().schedule(command);

        fixture.runSchedulerCycles(4);

        assertFalse(
                fixture.isDriveInXLock(),
                "Nonzero translation input should prevent X-lock even when ready.");
    }

    @Test
    void transitionsOutOfXLockWhenDriverInputResumes() {
        AtomicBoolean aimReady = new AtomicBoolean(true);
        AtomicReference<Double> xInput = new AtomicReference<>(0.0);

        Command command = fixture.createShootCommandSupplier(xInput::get, () -> 0.0, () -> 0.0, aimReady::get);
        CommandScheduler.getInstance().schedule(command);

        boolean locked = false;
        for (int i = 0; i < 8; i++) {
            fixture.runSchedulerCycles(1);
            if (fixture.isDriveInXLock()) {
                locked = true;
                break;
            }
        }
        assertTrue(locked, "Expected X-lock before driver movement resumes.");

        xInput.set(0.5);
        boolean unlocked = false;
        for (int i = 0; i < 6; i++) {
            fixture.runSchedulerCycles(1);
            if (!fixture.isDriveInXLock()) {
                unlocked = true;
                break;
            }
        }
        assertTrue(unlocked, "Expected X-lock to release after driver input resumes.");
    }

    @Test
    void rightTriggerPublishesPassTargetOutsideBlueAllianceZone() {
        fixture.setAlliance(AllianceStationID.Blue1);
        fixture.setPose(new Pose2d(
                FieldConstants.getAllianceZoneBoundaryX() + 0.7,
                FieldConstants.getHubTargetTranslation().getY() - 0.9,
                Rotation2d.kZero));

        ShootingTeleopController.TargetSelection selection = fixture.publishRightTriggerTargetTelemetry();

        assertTrue(selection.mode() == ShootingTeleopController.RightTriggerMode.PASS);
        assertTrue(
                selection.targetPose().getX() < FieldConstants.getAllianceZoneBoundaryX(),
                "Blue pass target should stay on the blue side of the field.");
        assertTrue(
                selection.targetPose().getY() < FieldConstants.getHubTargetTranslation().getY(),
                "Blue pass target should move away from the hub lane.");
    }

    @Test
    void rightTriggerPublishesPassTargetOutsideRedAllianceZone() {
        fixture.setAlliance(AllianceStationID.Red1);
        fixture.setPose(new Pose2d(
                FieldConstants.getAllianceZoneBoundaryX() - 0.7,
                FieldConstants.getHubTargetTranslation().getY() + 0.9,
                Rotation2d.kZero));

        ShootingTeleopController.TargetSelection selection = fixture.publishRightTriggerTargetTelemetry();

        assertTrue(selection.mode() == ShootingTeleopController.RightTriggerMode.PASS);
        assertTrue(
                selection.targetPose().getX() > FieldConstants.getAllianceZoneBoundaryX(),
                "Red pass target should stay on the red side of the field.");
        assertTrue(
                selection.targetPose().getY() > FieldConstants.getHubTargetTranslation().getY(),
                "Red pass target should move away from the hub lane.");
    }

    private static final class Fixture {
        private final TestModuleIO[] moduleIOs = new TestModuleIO[] {
                new TestModuleIO(),
                new TestModuleIO(),
                new TestModuleIO(),
                new TestModuleIO()
        };
        private final Drive drive = new Drive(
                new TestGyroIO(),
                moduleIOs[0], moduleIOs[1], moduleIOs[2], moduleIOs[3]);
        private final Shooter shooter = new Shooter(new TestShooterIO());
        private final Transfer transfer = new Transfer(new TestTransferIO());
        private final Intake intake = new Intake(new IntakeIO() {});
        private final ShootCoordinator coordinator = new ShootCoordinator(shooter, transfer);
        private final DashboardOverrides dashboardOverrides = new DashboardOverrides();
        private final CommandTelemetry telemetry = new CommandTelemetry();
        private final ShootingTeleopController controller = new ShootingTeleopController(
                drive,
                shooter,
                intake,
                coordinator,
                dashboardOverrides,
                telemetry);

        private Fixture() {
            RobotState.initialize(drive);
            telemetry.configure();
            dashboardOverrides.init();
            resetForTest();
        }

        private void resetForTest() {
            CommandScheduler.getInstance().cancelAll();
            SmartDashboard.putBoolean("Overrides/OverrideAutoAim", false);
            drive.setFieldOriented(false);
            drive.runDriverVelocity(new ChassisSpeeds(1.0, 0.0, 0.0));
            runSchedulerCycles(2);
        }

        private Command createShootCommand(
                double x,
                double y,
                double omega,
                java.util.function.BooleanSupplier aimReadySupplier) {
            return createShootCommandSupplier(() -> x, () -> y, () -> omega, aimReadySupplier);
        }

        private Command createShootCommandSupplier(
                java.util.function.DoubleSupplier xSupplier,
                java.util.function.DoubleSupplier ySupplier,
                java.util.function.DoubleSupplier omegaSupplier,
                java.util.function.BooleanSupplier aimReadySupplier) {
            return controller.createSelectedShootCommand(
                    xSupplier,
                    ySupplier,
                    omegaSupplier,
                    () -> 4.0,
                    () -> Rotation2d.kZero,
                    aimReadySupplier,
                    () -> false);
        }

        private void runSchedulerCycles(int cycles) {
            for (int i = 0; i < cycles; i++) {
                DriverStationSim.notifyNewData();
                CommandScheduler.getInstance().run();
                telemetry.periodic();
            }
        }

        private void setPose(Pose2d pose) {
            drive.setPose(pose);
            runSchedulerCycles(2);
        }

        private void setAlliance(AllianceStationID allianceStationID) {
            DriverStationSim.setAllianceStationId(allianceStationID);
            DriverStationSim.notifyNewData();
            runSchedulerCycles(2);
        }

        private ShootingTeleopController.TargetSelection publishRightTriggerTargetTelemetry() {
            return controller.publishRightTriggerTargetTelemetry();
        }

        private boolean isDriveInXLock() {
            Translation2d[] translations = Drive.getModuleTranslations();
            for (int i = 0; i < moduleIOs.length; i++) {
                double expectedRad = translations[i].getAngle().getRadians();
                double actualRad = moduleIOs[i].lastTurnSetpoint.getRadians();
                double lineErrorRad = Math.abs(MathUtil.angleModulus(actualRad - expectedRad));
                lineErrorRad = Math.min(lineErrorRad, Math.abs(Math.PI - lineErrorRad));
                if (lineErrorRad > X_LOCK_ANGLE_TOLERANCE_RAD) {
                    return false;
                }
            }
            return true;
        }
    }

    private static final class TestGyroIO implements GyroIO {
        private Rotation2d yaw = Rotation2d.kZero;

        @Override
        public void updateInputs(GyroIOInputs inputs) {
            inputs.connected = true;
            inputs.yawPosition = yaw;
            inputs.yawVelocityRadPerSec = 0.0;
            inputs.odometryYawTimestamps = new double[] { Timer.getFPGATimestamp() };
            inputs.odometryYawPositions = new Rotation2d[] { yaw };
        }

        @Override
        public void setYaw(Rotation2d yaw) {
            this.yaw = yaw;
        }
    }

    private static final class TestModuleIO implements ModuleIO {
        private Rotation2d lastTurnSetpoint = Rotation2d.kZero;
        private double driveVelocityRadPerSec = 0.0;

        @Override
        public void updateInputs(ModuleIOInputs inputs) {
            inputs.driveConnected = true;
            inputs.turnConnected = true;
            inputs.turnEncoderConnected = true;
            inputs.driveVelocityRadPerSec = driveVelocityRadPerSec;
            inputs.turnPosition = lastTurnSetpoint;
            inputs.turnAbsolutePosition = lastTurnSetpoint;
            inputs.odometryTimestamps = new double[] { Timer.getFPGATimestamp() };
            inputs.odometryDrivePositionsRad = new double[] { 0.0 };
            inputs.odometryTurnPositions = new Rotation2d[] { lastTurnSetpoint };
        }

        @Override
        public void setDriveVelocity(double velocityRadPerSec) {
            driveVelocityRadPerSec = velocityRadPerSec;
        }

        @Override
        public void setTurnPosition(Rotation2d rotation) {
            lastTurnSetpoint = rotation;
        }
    }

    private static final class TestShooterIO implements ShooterIO {
        private double leftRpm;
        private double rightRpm;
        private double hoodRad;

        @Override
        public void updateInputs(ShooterIOInputs inputs) {
            inputs.shooterLeftVelocityRpm = leftRpm;
            inputs.shooterRightVelocityRpm = rightRpm;
            inputs.hoodPositionRad = hoodRad;
        }

        @Override
        public void setLeftVelocity(double rpm) {
            leftRpm = rpm;
        }

        @Override
        public void setRightVelocity(double rpm) {
            rightRpm = rpm;
        }

        @Override
        public void setHoodAngle(double angle) {
            hoodRad = angle;
        }

        @Override
        public void stop() {
            leftRpm = 0.0;
            rightRpm = 0.0;
        }
    }

    private static final class TestTransferIO implements TransferIO {
        @Override
        public void setPercent(double percent) {}

        @Override
        public void stop() {}
    }
}
