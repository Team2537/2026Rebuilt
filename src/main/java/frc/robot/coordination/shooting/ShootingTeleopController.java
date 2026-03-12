package frc.robot.coordination.shooting;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.CommandTelemetry;
import frc.robot.DashboardOverrides;
import frc.robot.RobotState;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.LaunchCalculator;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Shooter.ReadinessMode;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.util.AimReadyLatch;
import frc.robot.util.AutoAimHeadingConfig;
import frc.robot.util.CycleCache;
import frc.robot.util.TargetHoldover;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/** Encapsulates teleop shooting/aim logic and per-loop cached suppliers. */
public final class ShootingTeleopController {
    private static final double SHOT_ON_MOVE_LINEAR_ENGAGE_MPS = 0.35;
    private static final double SHOT_ON_MOVE_LINEAR_RELEASE_MPS = 0.20;
    private static final double SHOT_ON_MOVE_ANGULAR_ENGAGE_RAD_PER_SEC = Math.toRadians(35.0);
    private static final double SHOT_ON_MOVE_ANGULAR_RELEASE_RAD_PER_SEC = Math.toRadians(20.0);

    private final Drive drive;
    private final Shooter shooter;
    private final Intake intake;
    private final ShootCoordinator shootCoordinator;
    private final DashboardOverrides dashboardOverrides;
    private final CommandTelemetry commandTelemetry;

    public record AimingContext(
            DoubleSupplier hubDistanceSupplier,
            Supplier<Rotation2d> hubHeadingSupplier,
            BooleanSupplier aimReadySupplier) {}

    public ShootingTeleopController(
            Drive drive,
            Shooter shooter,
            Intake intake,
            ShootCoordinator shootCoordinator,
            DashboardOverrides dashboardOverrides,
            CommandTelemetry commandTelemetry) {
        this.drive = drive;
        this.shooter = shooter;
        this.intake = intake;
        this.shootCoordinator = shootCoordinator;
        this.dashboardOverrides = dashboardOverrides;
        this.commandTelemetry = commandTelemetry;
    }

    public AimingContext createAimingContext() {
        Supplier<Pose2d> shootingPoseSupplier = createShootingPoseSupplier();
        Supplier<LaunchCalculator.MotionCompensation> motionCompensationSupplier =
                createTeleopMotionCompensationSupplier(shootingPoseSupplier);
        DoubleSupplier hubDistanceSupplier =
                () -> motionCompensationSupplier.get().compensatedDistanceMeters();
        Supplier<Rotation2d> hubHeadingSupplier =
                () -> motionCompensationSupplier.get().compensatedHeading();
        return new AimingContext(
                hubDistanceSupplier,
                hubHeadingSupplier,
                createTeleopAimReadySupplier(hubHeadingSupplier));
    }

    public Command createSelectedShootCommand(
            DoubleSupplier xSupplier,
            DoubleSupplier ySupplier,
            DoubleSupplier omegaFallbackSupplier,
            DoubleSupplier distanceMetersSupplier,
            Supplier<Rotation2d> targetHeadingSupplier,
            BooleanSupplier aimReadySupplier,
            BooleanSupplier manualFeedOverrideSupplier) {
        return withShootingConstraintProfile(
                Commands.either(
                        createOverrideAutoAimShootCommand(
                                xSupplier,
                                ySupplier,
                                omegaFallbackSupplier,
                                manualFeedOverrideSupplier),
                        createShootCommand(
                                xSupplier,
                                ySupplier,
                                omegaFallbackSupplier,
                                distanceMetersSupplier,
                                targetHeadingSupplier,
                                aimReadySupplier,
                                manualFeedOverrideSupplier),
                        dashboardOverrides::isAutoAimEnabled)
                        .withName("ShooterTriggerSelectedMode"));
    }

    public Command createHubShotCommand(
            DoubleSupplier xSupplier,
            DoubleSupplier ySupplier,
            DoubleSupplier omegaFallbackSupplier,
            Supplier<Rotation2d> targetHeadingSupplier,
            BooleanSupplier aimReadySupplier,
            BooleanSupplier manualFeedOverrideSupplier) {
        return withShootingConstraintProfile(
                createShootCommand(
                        xSupplier,
                        ySupplier,
                        omegaFallbackSupplier,
                        () -> ShooterConstants.HUB_SHOT_DISTANCE_METERS,
                        targetHeadingSupplier,
                        aimReadySupplier,
                        manualFeedOverrideSupplier)
                        .withName("ShooterHubShot"));
    }

    private Supplier<Pose2d> createShootingPoseSupplier() {
        // Single-source aiming pose: estimator translation + live gyro heading.
        RobotState robotState = RobotState.getInstance();
        return () -> new Pose2d(robotState.getPose().getTranslation(), robotState.getRotation());
    }

    private Supplier<LaunchCalculator.MotionCompensation> createTeleopMotionCompensationSupplier(
            Supplier<Pose2d> shootingPoseSupplier) {
        CycleCache<LaunchCalculator.MotionCompensation> cycleCache = new CycleCache<>();
        LaunchCalculator.MotionCompensation emptyCompensation = new LaunchCalculator.MotionCompensation(
                Double.NaN,
                Double.NaN,
                Double.NaN,
                Double.NaN,
                Double.NaN,
                null);

        return () -> cycleCache.get(commandTelemetry.getCycle(), () -> {
            LaunchCalculator.MotionCompensation compensation = shooter.getMotionCompensationToHub(
                    shootingPoseSupplier.get(),
                    RobotState.getInstance().getMeasuredChassisSpeeds());
            return compensation != null ? compensation : emptyCompensation;
        });
    }

    private BooleanSupplier createTeleopAimReadySupplier(
            Supplier<Rotation2d> targetHeadingSupplier) {
        TargetHoldover<Rotation2d> targetHoldover =
                new TargetHoldover<>(AutoAimHeadingConfig.TARGET_HOLD_SEC, Timer::getFPGATimestamp);
        AimReadyLatch aimReadyLatch = new AimReadyLatch(
                AutoAimHeadingConfig.AIM_TOLERANCE_RAD,
                AutoAimHeadingConfig.AIM_RELEASE_TOLERANCE_RAD);
        return () -> {
            TargetHoldover.HoldResult<Rotation2d> holdoverResult =
                    targetHoldover.apply(targetHeadingSupplier.get());
            Rotation2d targetHeading = holdoverResult.value();
            Rotation2d robotHeading = RobotState.getInstance().getRotation();

            Logger.recordOutput("Shooting/AimTargetAvailable", targetHeading != null);
            Logger.recordOutput("Shooting/TargetHeld", holdoverResult.held());
            Logger.recordOutput("Shooting/TargetAgeSec", holdoverResult.ageSec());
            Logger.recordOutput("Shooting/RobotHeadingDeg", robotHeading.getDegrees());
            if (targetHeading == null) {
                Logger.recordOutput("Shooting/TargetHeadingDeg", Double.NaN);
                Logger.recordOutput("Shooting/DesiredRobotHeadingDeg", Double.NaN);
                Logger.recordOutput("Shooting/AimErrorRad", Double.NaN);
                Logger.recordOutput("Shooting/AimErrorDeg", Double.NaN);
                Logger.recordOutput("Shooting/AimReadyLatched", false);
                aimReadyLatch.reset();
                return false;
            }

            Rotation2d desiredRobotHeading = targetHeading.plus(Rotation2d.kPi);
            double headingErrorRad = MathUtil.angleModulus(
                    desiredRobotHeading.minus(robotHeading).getRadians());
            Logger.recordOutput("Shooting/TargetHeadingDeg", targetHeading.getDegrees());
            Logger.recordOutput("Shooting/DesiredRobotHeadingDeg", desiredRobotHeading.getDegrees());
            Logger.recordOutput("Shooting/AimErrorRad", headingErrorRad);
            Logger.recordOutput("Shooting/AimErrorDeg", Math.toDegrees(headingErrorRad));
            boolean aimReady = aimReadyLatch.update(Math.abs(headingErrorRad));
            Logger.recordOutput("Shooting/AimReadyLatched", aimReady);
            return aimReady;
        };
    }

    private Command createAutoAlignCommand(
            DoubleSupplier xSupplier,
            DoubleSupplier ySupplier,
            DoubleSupplier omegaFallbackSupplier,
            Supplier<Rotation2d> targetHeadingSupplier) {
        return createAutoAlignCommand(
                xSupplier,
                ySupplier,
                omegaFallbackSupplier,
                targetHeadingSupplier,
                () -> false);
    }

    private Command createAutoAlignCommand(
            DoubleSupplier xSupplier,
            DoubleSupplier ySupplier,
            DoubleSupplier omegaFallbackSupplier,
            Supplier<Rotation2d> targetHeadingSupplier,
            BooleanSupplier xLockConditionSupplier) {
        return DriveCommands.autoAlignToHubPose(
                drive,
                xSupplier,
                ySupplier,
                omegaFallbackSupplier,
                targetHeadingSupplier,
                xLockConditionSupplier);
    }

    private Command createShootCommand(
            DoubleSupplier xSupplier,
            DoubleSupplier ySupplier,
            DoubleSupplier omegaFallbackSupplier,
            DoubleSupplier distanceMetersSupplier,
            Supplier<Rotation2d> targetHeadingSupplier,
            BooleanSupplier aimReadySupplier,
            BooleanSupplier manualFeedOverrideSupplier) {
        CycleCache<Boolean> aimReadyCycleCache = new CycleCache<>();
        BooleanSupplier shotOnMoveSupplier = createTeleopShotOnMoveSupplier();
        CycleCache<Boolean> shotOnMoveCycleCache = new CycleCache<>();
        BooleanSupplier cachedAimReadySupplier =
                () -> aimReadyCycleCache.get(commandTelemetry.getCycle(), aimReadySupplier::getAsBoolean);
        BooleanSupplier cachedShotOnMoveSupplier =
                () -> shotOnMoveCycleCache.get(commandTelemetry.getCycle(), shotOnMoveSupplier::getAsBoolean);
        BooleanSupplier readyToFeedSupplier =
                () -> shooter.getReadinessDiagnosticsNow(
                                cachedShotOnMoveSupplier.getAsBoolean()
                                        ? ReadinessMode.SHOT_ON_MOVE
                                        : ReadinessMode.STATIONARY)
                        .atSetpoint()
                        && cachedAimReadySupplier.getAsBoolean();

        return Commands.parallel(
                createAutoAlignCommand(
                        xSupplier,
                        ySupplier,
                        omegaFallbackSupplier,
                        targetHeadingSupplier,
                        readyToFeedSupplier),
                shootCoordinator.shootForDistance(
                        distanceMetersSupplier,
                        cachedAimReadySupplier,
                        cachedShotOnMoveSupplier,
                        manualFeedOverrideSupplier,
                        () -> !dashboardOverrides.isFeedingDisabled()),
                intake.smartRetractDuringShootCommand(shootCoordinator::isActivelyFeeding))
                .withName("ShooterTriggerAimAndShoot");
    }

    private Command createOverrideAutoAimShootCommand(
            DoubleSupplier xSupplier,
            DoubleSupplier ySupplier,
            DoubleSupplier omegaSupplier,
            BooleanSupplier manualFeedOverrideSupplier) {
        return Commands.parallel(
                DriveCommands.joystickDrive(
                        drive,
                        xSupplier,
                        ySupplier,
                        omegaSupplier)
                        .withName("DriveJoystickOverrideAutoAim"),
                shootCoordinator.shootForDistance(
                                dashboardOverrides::getAimDistanceMeters,
                                () -> true,
                                manualFeedOverrideSupplier,
                                () -> !dashboardOverrides.isFeedingDisabled())
                        .withName("ShooterShootOverrideDistance"),
                intake.smartRetractDuringShootCommand(shootCoordinator::isActivelyFeeding))
                .withName("ShooterTriggerOverrideAutoAimShoot");
    }

    private Command withShootingConstraintProfile(Command command) {
        return command
                .beforeStarting(() -> drive.setConstraintProfileActive(DriveConstants.ConstraintProfile.SHOOTING_ON_MOVE, true))
                .finallyDo(interrupted -> drive.setConstraintProfileActive(DriveConstants.ConstraintProfile.SHOOTING_ON_MOVE, false));
    }

    private BooleanSupplier createTeleopShotOnMoveSupplier() {
        MotionModeLatch shotOnMoveLatch = new MotionModeLatch(
                SHOT_ON_MOVE_LINEAR_ENGAGE_MPS,
                SHOT_ON_MOVE_LINEAR_RELEASE_MPS,
                SHOT_ON_MOVE_ANGULAR_ENGAGE_RAD_PER_SEC,
                SHOT_ON_MOVE_ANGULAR_RELEASE_RAD_PER_SEC);
        return () -> {
            ChassisSpeeds speeds = RobotState.getInstance().getMeasuredChassisSpeeds();
            double linearSpeedMps = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
            double angularSpeedRadPerSec = Math.abs(speeds.omegaRadiansPerSecond);
            boolean shotOnMove = shotOnMoveLatch.update(linearSpeedMps, angularSpeedRadPerSec);

            Logger.recordOutput("Shooting/TeleopLinearSpeedMps", linearSpeedMps);
            Logger.recordOutput("Shooting/TeleopAngularSpeedRadPerSec", angularSpeedRadPerSec);
            Logger.recordOutput("Shooting/TeleopShotOnMoveLatched", shotOnMove);
            return shotOnMove;
        };
    }

    private static final class MotionModeLatch {
        private final double linearEngageMps;
        private final double linearReleaseMps;
        private final double angularEngageRadPerSec;
        private final double angularReleaseRadPerSec;
        private boolean latched = false;

        private MotionModeLatch(
                double linearEngageMps,
                double linearReleaseMps,
                double angularEngageRadPerSec,
                double angularReleaseRadPerSec) {
            this.linearEngageMps = linearEngageMps;
            this.linearReleaseMps = linearReleaseMps;
            this.angularEngageRadPerSec = angularEngageRadPerSec;
            this.angularReleaseRadPerSec = angularReleaseRadPerSec;
        }

        private boolean update(double linearSpeedMps, double angularSpeedRadPerSec) {
            if (latched) {
                latched = linearSpeedMps >= linearReleaseMps || angularSpeedRadPerSec >= angularReleaseRadPerSec;
            } else {
                latched = linearSpeedMps >= linearEngageMps || angularSpeedRadPerSec >= angularEngageRadPerSec;
            }
            return latched;
        }
    }
}
