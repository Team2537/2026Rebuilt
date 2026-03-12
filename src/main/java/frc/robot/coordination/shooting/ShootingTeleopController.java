package frc.robot.coordination.shooting;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
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
import frc.robot.util.FieldConstants;
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

    public enum RightTriggerMode {
        SHOOT,
        PASS
    }

    public record TargetSelection(
            RightTriggerMode mode,
            Pose2d targetPose,
            double distanceMeters,
            Rotation2d targetHeading,
            ReadinessMode readinessMode) {}

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

    /** Publishes the current right-trigger target selection for driver verification. */
    public TargetSelection publishRightTriggerTargetTelemetry() {
        TargetSelection selection = buildTelemetryTargetSelection();
        logTargetSelection(selection);
        return selection;
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
                        createZoneAwareShootCommand(
                                xSupplier,
                                ySupplier,
                                omegaFallbackSupplier,
                                distanceMetersSupplier,
                                targetHeadingSupplier,
                                aimReadySupplier,
                                manualFeedOverrideSupplier),
                        () -> dashboardOverrides.isAutoAimEnabled()
                                && FieldConstants.isInAllianceZone(RobotState.getInstance().getPose()))
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
                        () -> ReadinessMode.STATIONARY,
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

    private TargetSelection buildTelemetryTargetSelection() {
        Pose2d robotPose = RobotState.getInstance().getPose();
        if (FieldConstants.isInAllianceZone(robotPose)) {
            return new TargetSelection(
                    RightTriggerMode.SHOOT,
                    new Pose2d(FieldConstants.getHubTargetTranslation(), Rotation2d.kZero),
                    LaunchCalculator.getHubDistanceMeters(robotPose),
                    FieldConstants.getHubFacingHeading(robotPose),
                    ReadinessMode.STATIONARY);
        }

        Pose2d passTargetPose = FieldConstants.getPassTargetPose(robotPose);
        return new TargetSelection(
                RightTriggerMode.PASS,
                passTargetPose,
                getShooterDistanceToTarget(robotPose, passTargetPose.getTranslation()),
                FieldConstants.getHeadingToTarget(robotPose, passTargetPose.getTranslation()),
                ReadinessMode.PASSING);
    }

    private Supplier<TargetSelection> createRightTriggerTargetSelectionSupplier(
            DoubleSupplier hubDistanceMetersSupplier,
            Supplier<Rotation2d> hubTargetHeadingSupplier) {
        CycleCache<TargetSelection> cycleCache = new CycleCache<>();
        return () -> cycleCache.get(commandTelemetry.getCycle(), () -> {
            Pose2d robotPose = RobotState.getInstance().getPose();
            TargetSelection selection;
            if (FieldConstants.isInAllianceZone(robotPose)) {
                selection = new TargetSelection(
                        RightTriggerMode.SHOOT,
                        new Pose2d(FieldConstants.getHubTargetTranslation(), Rotation2d.kZero),
                        hubDistanceMetersSupplier.getAsDouble(),
                        hubTargetHeadingSupplier.get(),
                        ReadinessMode.STATIONARY);
            } else {
                Pose2d passTargetPose = FieldConstants.getPassTargetPose(robotPose);
                selection = new TargetSelection(
                        RightTriggerMode.PASS,
                        passTargetPose,
                        getShooterDistanceToTarget(robotPose, passTargetPose.getTranslation()),
                        FieldConstants.getHeadingToTarget(robotPose, passTargetPose.getTranslation()),
                        ReadinessMode.PASSING);
            }
            logTargetSelection(selection);
            return selection;
        });
    }

    private BooleanSupplier createTeleopAimReadySupplier(
            Supplier<Rotation2d> targetHeadingSupplier) {
        return createTeleopAimReadySupplier(targetHeadingSupplier, () -> false);
    }

    private BooleanSupplier createTeleopAimReadySupplier(
            Supplier<Rotation2d> targetHeadingSupplier,
            BooleanSupplier passModeSupplier) {
        TargetHoldover<Rotation2d> targetHoldover =
                new TargetHoldover<>(AutoAimHeadingConfig.TARGET_HOLD_SEC, Timer::getFPGATimestamp);
        AimReadyLatch shootAimReadyLatch = new AimReadyLatch(
                AutoAimHeadingConfig.AIM_TOLERANCE_RAD,
                AutoAimHeadingConfig.AIM_RELEASE_TOLERANCE_RAD);
        AimReadyLatch passAimReadyLatch = new AimReadyLatch(
                AutoAimHeadingConfig.PASS_AIM_TOLERANCE_RAD,
                AutoAimHeadingConfig.PASS_AIM_RELEASE_TOLERANCE_RAD);
        final boolean[] lastPassMode = new boolean[] {passModeSupplier.getAsBoolean()};
        return () -> {
            boolean passMode = passModeSupplier.getAsBoolean();
            if (passMode != lastPassMode[0]) {
                shootAimReadyLatch.reset();
                passAimReadyLatch.reset();
                lastPassMode[0] = passMode;
            }
            TargetHoldover.HoldResult<Rotation2d> holdoverResult =
                    targetHoldover.apply(targetHeadingSupplier.get());
            Rotation2d targetHeading = holdoverResult.value();
            Rotation2d robotHeading = RobotState.getInstance().getRotation();
            AimReadyLatch activeAimReadyLatch = passMode ? passAimReadyLatch : shootAimReadyLatch;
            double activeAimToleranceRad = passMode
                    ? AutoAimHeadingConfig.PASS_AIM_TOLERANCE_RAD
                    : AutoAimHeadingConfig.AIM_TOLERANCE_RAD;
            double activeAimReleaseToleranceRad = passMode
                    ? AutoAimHeadingConfig.PASS_AIM_RELEASE_TOLERANCE_RAD
                    : AutoAimHeadingConfig.AIM_RELEASE_TOLERANCE_RAD;

            Logger.recordOutput("Shooting/AimTargetAvailable", targetHeading != null);
            Logger.recordOutput("Shooting/TargetHeld", holdoverResult.held());
            Logger.recordOutput("Shooting/TargetAgeSec", holdoverResult.ageSec());
            Logger.recordOutput("Shooting/RobotHeadingDeg", robotHeading.getDegrees());
            Logger.recordOutput("Shooting/PassAimMode", passMode);
            Logger.recordOutput("Shooting/ActiveAimToleranceRad", activeAimToleranceRad);
            Logger.recordOutput("Shooting/ActiveAimReleaseToleranceRad", activeAimReleaseToleranceRad);
            if (targetHeading == null) {
                Logger.recordOutput("Shooting/TargetHeadingDeg", Double.NaN);
                Logger.recordOutput("Shooting/DesiredRobotHeadingDeg", Double.NaN);
                Logger.recordOutput("Shooting/AimErrorRad", Double.NaN);
                Logger.recordOutput("Shooting/AimErrorDeg", Double.NaN);
                Logger.recordOutput("Shooting/AimReadyLatched", false);
                activeAimReadyLatch.reset();
                return false;
            }

            Rotation2d desiredRobotHeading = targetHeading.plus(Rotation2d.kPi);
            double headingErrorRad = MathUtil.angleModulus(
                    desiredRobotHeading.minus(robotHeading).getRadians());
            Logger.recordOutput("Shooting/TargetHeadingDeg", targetHeading.getDegrees());
            Logger.recordOutput("Shooting/DesiredRobotHeadingDeg", desiredRobotHeading.getDegrees());
            Logger.recordOutput("Shooting/AimErrorRad", headingErrorRad);
            Logger.recordOutput("Shooting/AimErrorDeg", Math.toDegrees(headingErrorRad));
            boolean aimReady = activeAimReadyLatch.update(Math.abs(headingErrorRad));
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

    private Command createZoneAwareShootCommand(
            DoubleSupplier xSupplier,
            DoubleSupplier ySupplier,
            DoubleSupplier omegaFallbackSupplier,
            DoubleSupplier hubDistanceMetersSupplier,
            Supplier<Rotation2d> hubTargetHeadingSupplier,
            BooleanSupplier shootAimReadySupplier,
            BooleanSupplier manualFeedOverrideSupplier) {
        Supplier<TargetSelection> targetSelectionSupplier =
                createRightTriggerTargetSelectionSupplier(hubDistanceMetersSupplier, hubTargetHeadingSupplier);
        BooleanSupplier passModeSupplier =
                () -> targetSelectionSupplier.get().mode() == RightTriggerMode.PASS;
        BooleanSupplier passAimReadySupplier =
                createTeleopAimReadySupplier(
                        () -> targetSelectionSupplier.get().targetHeading(),
                        passModeSupplier);
        BooleanSupplier aimReadySupplier =
                () -> passModeSupplier.getAsBoolean()
                        ? passAimReadySupplier.getAsBoolean()
                        : shootAimReadySupplier.getAsBoolean();
        return createShootCommand(
                xSupplier,
                ySupplier,
                omegaFallbackSupplier,
                () -> targetSelectionSupplier.get().distanceMeters(),
                () -> targetSelectionSupplier.get().targetHeading(),
                aimReadySupplier,
                () -> targetSelectionSupplier.get().readinessMode(),
                manualFeedOverrideSupplier);
    }

    private Command createShootCommand(
            DoubleSupplier xSupplier,
            DoubleSupplier ySupplier,
            DoubleSupplier omegaFallbackSupplier,
            DoubleSupplier distanceMetersSupplier,
            Supplier<Rotation2d> targetHeadingSupplier,
            BooleanSupplier aimReadySupplier,
            Supplier<ReadinessMode> readinessModeSupplier,
            BooleanSupplier manualFeedOverrideSupplier) {
        CycleCache<Boolean> aimReadyCycleCache = new CycleCache<>();
        BooleanSupplier shotOnMoveSupplier = createTeleopShotOnMoveSupplier();
        CycleCache<ReadinessMode> readinessModeCycleCache = new CycleCache<>();
        BooleanSupplier cachedAimReadySupplier =
                () -> aimReadyCycleCache.get(commandTelemetry.getCycle(), aimReadySupplier::getAsBoolean);
        Supplier<ReadinessMode> cachedReadinessModeSupplier =
                () -> readinessModeCycleCache.get(commandTelemetry.getCycle(), () -> {
                    ReadinessMode requestedMode = readinessModeSupplier.get();
                    if (requestedMode == ReadinessMode.PASSING) {
                        return ReadinessMode.PASSING;
                    }
                    return shotOnMoveSupplier.getAsBoolean()
                            ? ReadinessMode.SHOT_ON_MOVE
                            : ReadinessMode.STATIONARY;
                });
        BooleanSupplier readyToFeedSupplier =
                () -> shooter.getReadinessDiagnosticsNow(cachedReadinessModeSupplier.get())
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
                        cachedReadinessModeSupplier,
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

    private void logTargetSelection(TargetSelection selection) {
        Logger.recordOutput("Shooting/RightTriggerMode", selection.mode().name());
        Logger.recordOutput("Shooting/InAllianceZone", selection.mode() == RightTriggerMode.SHOOT);
        Logger.recordOutput("Shooting/TargetPose", selection.targetPose());
        Logger.recordOutput("Shooting/TargetPoseX", selection.targetPose().getX());
        Logger.recordOutput("Shooting/TargetPoseY", selection.targetPose().getY());
        Logger.recordOutput("Shooting/TargetDistanceMeters", selection.distanceMeters());
        Logger.recordOutput(
                "Shooting/TargetHeadingForPoseDeg",
                selection.targetHeading() != null ? selection.targetHeading().getDegrees() : Double.NaN);
        Logger.recordOutput("Shooting/TargetReadinessMode", selection.readinessMode().name());
        Logger.recordOutput("Shooting/AllianceZoneBoundaryX", FieldConstants.getAllianceZoneBoundaryX());
    }

    private static double getShooterDistanceToTarget(Pose2d robotPose, Translation2d targetTranslation) {
        if (robotPose == null || targetTranslation == null) {
            return Double.NaN;
        }
        Translation2d shooterFieldPosition = robotPose.getTranslation().plus(
                ShooterConstants.ROBOT_TO_SHOOTER_OFFSET.rotateBy(robotPose.getRotation()));
        return shooterFieldPosition.getDistance(targetTranslation);
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
