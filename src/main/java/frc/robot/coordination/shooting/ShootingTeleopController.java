package frc.robot.coordination.shooting;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
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
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.util.AutoAimHeadingConfig;
import frc.robot.util.CycleCache;
import frc.robot.util.FieldConstants;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/** Encapsulates teleop shooting/aim logic around a unified {@link ShotSolution}. */
public final class ShootingTeleopController {
    private static final LoggedTunableNumber movingShotMinTranslationSpeedMps =
            new LoggedTunableNumber("Shooting/MovingShotMinTranslationSpeedMps", 0.25);

    private final Drive drive;
    private final Shooter shooter;
    private final Intake intake;
    private final ShootCoordinator shootCoordinator;
    private final DashboardOverrides dashboardOverrides;
    private final CommandTelemetry commandTelemetry;
    private final ShotSolutionCalculator shotSolutionCalculator;

    public record AimingContext(
            DoubleSupplier hubDistanceSupplier,
            Supplier<Rotation2d> hubHeadingSupplier,
            BooleanSupplier aimReadySupplier,
            Supplier<ShotSolution> hubShotSolutionSupplier,
            Supplier<TargetSelection> shootTargetSelectionSupplier) {}

    public enum ShootTargetMode {
        SHOOT,
        PASS,
        MANUAL_DISTANCE,
        NONE
    }

    public record TargetSelection(
            ShootTargetMode mode,
            boolean hasFieldTarget,
            Pose2d targetPose,
            double distanceMeters,
            Rotation2d targetHeading,
            ShotIntent intent) {}

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
        this.shotSolutionCalculator = new ShotSolutionCalculator(shooter);
    }

    public AimingContext createAimingContext() {
        Supplier<ShotSolution> scoreSolutionSupplier = createTeleopHubShotSolutionSupplier();
        Supplier<TargetSelection> shootTargetSelectionSupplier =
                createShootTargetSelectionSupplier(scoreSolutionSupplier);
        return new AimingContext(
                () -> scoreSolutionSupplier.get().distanceMeters(),
                () -> scoreSolutionSupplier.get().targetHeading(),
                createAimReadySupplier(scoreSolutionSupplier),
                scoreSolutionSupplier,
                shootTargetSelectionSupplier);
    }

    public Command createSelectedShootCommand(
            DoubleSupplier xSupplier,
            DoubleSupplier ySupplier,
            DoubleSupplier omegaFallbackSupplier,
            Supplier<ShotSolution> hubShotSolutionSupplier,
            BooleanSupplier manualFeedOverrideSupplier) {
        return createSelectedShootCommand(
                xSupplier,
                ySupplier,
                omegaFallbackSupplier,
                null,
                null,
                hubShotSolutionSupplier,
                null,
                manualFeedOverrideSupplier);
    }

    /** Publishes the current shoot-target selection for driver verification. */
    public TargetSelection publishShootTargetTelemetry(Supplier<TargetSelection> targetSelectionSupplier) {
        TargetSelection selection = targetSelectionSupplier.get();
        logTargetSelection(selection);
        return selection;
    }

    public Command createSelectedShootCommand(
            DoubleSupplier xSupplier,
            DoubleSupplier ySupplier,
            DoubleSupplier omegaFallbackSupplier,
            DoubleSupplier distanceMetersSupplier,
            Supplier<Rotation2d> targetHeadingSupplier,
            BooleanSupplier manualFeedOverrideSupplier) {
        return createSelectedShootCommand(
                xSupplier,
                ySupplier,
                omegaFallbackSupplier,
                distanceMetersSupplier,
                targetHeadingSupplier,
                null,
                null,
                manualFeedOverrideSupplier);
    }

    Command createSelectedShootCommand(
            DoubleSupplier xSupplier,
            DoubleSupplier ySupplier,
            DoubleSupplier omegaFallbackSupplier,
            DoubleSupplier distanceMetersSupplier,
            Supplier<Rotation2d> targetHeadingSupplier,
            BooleanSupplier aimReadyOverrideSupplier,
            BooleanSupplier manualFeedOverrideSupplier) {
        return createSelectedShootCommand(
                xSupplier,
                ySupplier,
                omegaFallbackSupplier,
                distanceMetersSupplier,
                targetHeadingSupplier,
                null,
                aimReadyOverrideSupplier,
                manualFeedOverrideSupplier);
    }

    Command createSelectedShootCommand(
            DoubleSupplier xSupplier,
            DoubleSupplier ySupplier,
            DoubleSupplier omegaFallbackSupplier,
            DoubleSupplier distanceMetersSupplier,
            Supplier<Rotation2d> targetHeadingSupplier,
            Supplier<ShotSolution> hubShotSolutionSupplier,
            BooleanSupplier aimReadyOverrideSupplier,
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
                                hubShotSolutionSupplier,
                                aimReadyOverrideSupplier,
                                manualFeedOverrideSupplier,
                                () -> !dashboardOverrides.isFeedingDisabled()),
                        () -> dashboardOverrides.isAutoAimEnabled()
                                && FieldConstants.isInAllianceZone(RobotState.getInstance().getPose()))
                        .withName("ShooterSelectedShootMode"));
    }

    public Command createSelectedAimCommand(
            DoubleSupplier xSupplier,
            DoubleSupplier ySupplier,
            DoubleSupplier omegaFallbackSupplier,
            Supplier<ShotSolution> hubShotSolutionSupplier) {
        return withShootingConstraintProfile(
                Commands.either(
                        createOverrideAutoAimCommand(
                                xSupplier,
                                ySupplier,
                                omegaFallbackSupplier),
                        createZoneAwareAimCommand(
                                xSupplier,
                                ySupplier,
                                omegaFallbackSupplier,
                                null,
                                null,
                                hubShotSolutionSupplier),
                        () -> dashboardOverrides.isAutoAimEnabled()
                                && FieldConstants.isInAllianceZone(RobotState.getInstance().getPose()))
                        .withName("ShooterDriverAim"));
    }

    public Command createSelectedAimCommand(
            DoubleSupplier xSupplier,
            DoubleSupplier ySupplier,
            DoubleSupplier omegaFallbackSupplier,
            DoubleSupplier hubDistanceMetersSupplier,
            Supplier<Rotation2d> hubTargetHeadingSupplier) {
        return withShootingConstraintProfile(
                Commands.either(
                        createOverrideAutoAimCommand(
                                xSupplier,
                                ySupplier,
                                omegaFallbackSupplier),
                        createZoneAwareAimCommand(
                                xSupplier,
                                ySupplier,
                                omegaFallbackSupplier,
                                hubDistanceMetersSupplier,
                                hubTargetHeadingSupplier,
                                null),
                        () -> dashboardOverrides.isAutoAimEnabled()
                                && FieldConstants.isInAllianceZone(RobotState.getInstance().getPose()))
                        .withName("ShooterDriverAim"));
    }

    private Supplier<Pose2d> createShootingPoseSupplier() {
        RobotState robotState = RobotState.getInstance();
        return () -> new Pose2d(robotState.getPose().getTranslation(), robotState.getRotation());
    }

    private Supplier<LaunchCalculator.MotionCompensation> createTeleopMotionCompensationSupplier(
            Supplier<Pose2d> shootingPoseSupplier) {
        CycleCache<LaunchCalculator.MotionCompensation> cycleCache = new CycleCache<>();
        LaunchCalculator.CompensationTracker compensationTracker =
                new LaunchCalculator.CompensationTracker();
        LaunchCalculator.MotionCompensation emptyCompensation = new LaunchCalculator.MotionCompensation(
                Double.NaN,
                Double.NaN,
                Double.NaN,
                Double.NaN,
                Double.NaN,
                null,
                null,
                0.0);

        return () -> cycleCache.get(commandTelemetry.getCycle(), () -> {
            ChassisSpeeds compensationSpeeds = RobotState.getInstance().getSetpointChassisSpeeds();
            LaunchCalculator.MotionCompensation compensation = shooter.getMotionCompensationToHub(
                    shootingPoseSupplier.get(),
                    compensationSpeeds,
                    compensationTracker);
            return compensation != null ? compensation : emptyCompensation;
        });
    }

    private Supplier<ShotSolution> createTeleopHubShotSolutionSupplier() {
        Supplier<Pose2d> shootingPoseSupplier = createShootingPoseSupplier();
        Supplier<LaunchCalculator.MotionCompensation> motionCompensationSupplier =
                createTeleopMotionCompensationSupplier(shootingPoseSupplier);
        CycleCache<ShotSolution> cycleCache = new CycleCache<>();
        return () -> cycleCache.get(commandTelemetry.getCycle(), () -> {
            boolean movingShot = isMovingShot();
            return shotSolutionCalculator.createHubScoreSolution(
                    motionCompensationSupplier.get(),
                    movingShot,
                    movingShot ? AutoAimHeadingConfig.movingAimToleranceRad() : AutoAimHeadingConfig.aimToleranceRad(),
                    movingShot
                            ? AutoAimHeadingConfig.movingAimReleaseToleranceRad()
                            : AutoAimHeadingConfig.aimReleaseToleranceRad(),
                    movingShot
                            ? ShooterConstants.movingShooterRpmTolerance()
                            : ShooterConstants.scoreShooterRpmTolerance());
        });
    }

    private Supplier<TargetSelection> createShootTargetSelectionSupplier(
            Supplier<ShotSolution> allianceZoneHubShotSolutionSupplier) {
        Supplier<ShotSolution> selectedShotSolutionSupplier = createSelectedShotSolutionSupplier(
                null,
                null,
                AutoAimHeadingConfig.aimToleranceRad(),
                AutoAimHeadingConfig.aimReleaseToleranceRad(),
                ShooterConstants.scoreShooterRpmTolerance(),
                allianceZoneHubShotSolutionSupplier);
        CycleCache<TargetSelection> cycleCache = new CycleCache<>();
        return () -> cycleCache.get(commandTelemetry.getCycle(), () -> {
            Pose2d robotPose = RobotState.getInstance().getPose();
            if (dashboardOverrides.isAutoAimEnabled() && FieldConstants.isInAllianceZone(robotPose)) {
                return createManualDistanceTargetSelection(dashboardOverrides.getAimDistanceMeters());
            }
            return toTargetSelection(selectedShotSolutionSupplier.get());
        });
    }

    private Supplier<ShotSolution> createSelectedShotSolutionSupplier(
            DoubleSupplier scoreDistanceMetersSupplier,
            Supplier<Rotation2d> scoreTargetHeadingSupplier,
            double scoreHeadingToleranceRad,
            double scoreHeadingReleaseToleranceRad,
            double scoreShooterRpmTolerance,
            Supplier<ShotSolution> allianceZoneScoreSolutionSupplier) {
        ShotSolutionCalculator.HeadingRateTracker scoreTracker = shotSolutionCalculator.createHeadingRateTracker();
        ShotSolutionCalculator.HeadingRateTracker passTracker = shotSolutionCalculator.createHeadingRateTracker();
        CycleCache<ShotSolution> cycleCache = new CycleCache<>();
        return () -> cycleCache.get(commandTelemetry.getCycle(), () -> {
            Pose2d robotPose = RobotState.getInstance().getPose();
            if (FieldConstants.isInAllianceZone(robotPose)) {
                ShotSolution solution;
                if (allianceZoneScoreSolutionSupplier != null) {
                    solution = allianceZoneScoreSolutionSupplier.get();
                } else {
                    boolean movingShot = isMovingShot();
                    double headingToleranceRad = movingShot
                            ? AutoAimHeadingConfig.movingAimToleranceRad()
                            : scoreHeadingToleranceRad;
                    double headingReleaseToleranceRad = movingShot
                            ? AutoAimHeadingConfig.movingAimReleaseToleranceRad()
                            : scoreHeadingReleaseToleranceRad;
                    double shooterRpmTolerance = movingShot
                            ? ShooterConstants.movingShooterRpmTolerance()
                            : scoreShooterRpmTolerance;
                    solution = shotSolutionCalculator.createScoreSolution(
                            new Pose2d(FieldConstants.getHubTargetTranslation(), Rotation2d.kZero),
                            scoreDistanceMetersSupplier != null ? scoreDistanceMetersSupplier.getAsDouble() : Double.NaN,
                            scoreTargetHeadingSupplier != null ? scoreTargetHeadingSupplier.get() : null,
                            scoreTracker,
                            movingShot,
                            headingToleranceRad,
                            headingReleaseToleranceRad,
                            shooterRpmTolerance);
                }
                return solution;
            }

            scoreTracker.reset();
            if (FieldConstants.isInHubBackBlockedNeutralBand(robotPose)) {
                passTracker.reset();
                return ShotSolution.invalid(robotPose != null ? robotPose : new Pose2d());
            }

            return shotSolutionCalculator.createPassSolution(
                    robotPose,
                    passTracker,
                    AutoAimHeadingConfig.passAimToleranceRad(),
                    AutoAimHeadingConfig.passAimReleaseToleranceRad(),
                    ShooterConstants.passingShooterRpmTolerance());
        });
    }

    private static boolean isMovingShot() {
        ChassisSpeeds measuredSpeeds = RobotState.getInstance().getMeasuredChassisSpeeds();
        if (measuredSpeeds == null) {
            return false;
        }
        return Math.hypot(measuredSpeeds.vxMetersPerSecond, measuredSpeeds.vyMetersPerSecond)
                >= movingShotMinTranslationSpeedMps.get();
    }

    private BooleanSupplier createAimReadySupplier(Supplier<ShotSolution> shotSolutionSupplier) {
        ShotAimReadiness aimReadiness = new ShotAimReadiness();
        return () -> aimReadiness.update(shotSolutionSupplier.get(), RobotState.getInstance().getRotation());
    }

    private Command createZoneAwareShootCommand(
            DoubleSupplier xSupplier,
            DoubleSupplier ySupplier,
            DoubleSupplier omegaSupplier,
            DoubleSupplier hubDistanceMetersSupplier,
            Supplier<Rotation2d> hubTargetHeadingSupplier,
            Supplier<ShotSolution> hubShotSolutionSupplier,
            BooleanSupplier aimReadyOverrideSupplier,
            BooleanSupplier manualFeedOverrideSupplier,
            BooleanSupplier automaticFeedEnabledSupplier) {
        Supplier<ShotSolution> shotSolutionSupplier = createSelectedShotSolutionSupplier(
                hubDistanceMetersSupplier,
                hubTargetHeadingSupplier,
                AutoAimHeadingConfig.aimToleranceRad(),
                AutoAimHeadingConfig.aimReleaseToleranceRad(),
                ShooterConstants.scoreShooterRpmTolerance(),
                hubShotSolutionSupplier);
        return Commands.either(
                createBlockedShootDriveCommand(xSupplier, ySupplier, omegaSupplier),
                createShootCommand(
                        xSupplier,
                        ySupplier,
                        omegaSupplier,
                        shotSolutionSupplier,
                        aimReadyOverrideSupplier,
                        manualFeedOverrideSupplier,
                        automaticFeedEnabledSupplier),
                () -> shotSolutionSupplier.get().intent() == ShotIntent.NONE);
    }

    private Command createZoneAwareAimCommand(
            DoubleSupplier xSupplier,
            DoubleSupplier ySupplier,
            DoubleSupplier omegaSupplier,
            DoubleSupplier hubDistanceMetersSupplier,
            Supplier<Rotation2d> hubTargetHeadingSupplier,
            Supplier<ShotSolution> hubShotSolutionSupplier) {
        Supplier<ShotSolution> shotSolutionSupplier = createSelectedShotSolutionSupplier(
                hubDistanceMetersSupplier,
                hubTargetHeadingSupplier,
                AutoAimHeadingConfig.aimToleranceRad(),
                AutoAimHeadingConfig.aimReleaseToleranceRad(),
                ShooterConstants.scoreShooterRpmTolerance(),
                hubShotSolutionSupplier);
        return Commands.either(
                createBlockedShootDriveCommand(xSupplier, ySupplier, omegaSupplier),
                createAimCommand(xSupplier, ySupplier, omegaSupplier, shotSolutionSupplier),
                () -> shotSolutionSupplier.get().intent() == ShotIntent.NONE);
    }

    private Command createAimCommand(
            DoubleSupplier xSupplier,
            DoubleSupplier ySupplier,
            DoubleSupplier omegaSupplier,
            Supplier<ShotSolution> shotSolutionSupplier) {
        return Commands.parallel(
                createDriveWhileAimingCommand(xSupplier, ySupplier, omegaSupplier, shotSolutionSupplier, () -> false),
                shootCoordinator.aimForShot(shotSolutionSupplier))
                .withName("ShooterTriggerAimOnly");
    }

    private Command createBlockedShootDriveCommand(
            DoubleSupplier xSupplier,
            DoubleSupplier ySupplier,
            DoubleSupplier omegaSupplier) {
        return DriveCommands.joystickDrive(drive, xSupplier, ySupplier, omegaSupplier)
                .withName("DriveJoystickBlockedShoot");
    }

    private Command createShootCommand(
            DoubleSupplier xSupplier,
            DoubleSupplier ySupplier,
            DoubleSupplier omegaSupplier,
            Supplier<ShotSolution> shotSolutionSupplier,
            BooleanSupplier aimReadyOverrideSupplier,
            BooleanSupplier manualFeedOverrideSupplier,
            BooleanSupplier automaticFeedEnabledSupplier) {
        BooleanSupplier cachedAimReadySupplier =
                aimReadyOverrideSupplier != null ? aimReadyOverrideSupplier : createAimReadySupplier(shotSolutionSupplier);
        BooleanSupplier readyToFeedSupplier =
                () -> {
                    ShotSolution solution = shotSolutionSupplier.get();
                    return solution.valid()
                            && shooter.getReadinessDiagnosticsNow(solution.shooterRpmTolerance()).atSetpoint()
                            && cachedAimReadySupplier.getAsBoolean();
                };

        return Commands.parallel(
                createDriveWhileAimingCommand(xSupplier, ySupplier, omegaSupplier, shotSolutionSupplier, readyToFeedSupplier),
                shootCoordinator.shootForShot(
                        shotSolutionSupplier,
                        cachedAimReadySupplier,
                        manualFeedOverrideSupplier,
                        automaticFeedEnabledSupplier),
                intake.smartRetractDuringShootCommand(shootCoordinator::isActivelyFeeding))
                .withName("ShooterTriggerAimAndShoot");
    }

    private Command createDriveWhileAimingCommand(
            DoubleSupplier xSupplier,
            DoubleSupplier ySupplier,
            DoubleSupplier omegaSupplier,
            Supplier<ShotSolution> shotSolutionSupplier,
            BooleanSupplier xLockConditionSupplier) {
        return DriveCommands.driveWhileAiming(
                drive,
                xSupplier,
                ySupplier,
                omegaSupplier,
                () -> shotSolutionSupplier.get().desiredRobotHeading(),
                () -> shotSolutionSupplier.get().desiredHeadingRateRadPerSec(),
                xLockConditionSupplier);
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

    private Command createOverrideAutoAimCommand(
            DoubleSupplier xSupplier,
            DoubleSupplier ySupplier,
            DoubleSupplier omegaSupplier) {
        return Commands.parallel(
                DriveCommands.joystickDrive(
                        drive,
                        xSupplier,
                        ySupplier,
                        omegaSupplier)
                        .withName("DriveJoystickOverrideAutoAim"),
                shootCoordinator.aimForDistance(dashboardOverrides::getAimDistanceMeters)
                        .withName("ShooterAimOverrideDistance"))
                .withName("ShooterTriggerOverrideAutoAim");
    }

    private Command withShootingConstraintProfile(Command command) {
        return command
                .beforeStarting(() -> drive.setConstraintProfileActive(DriveConstants.ConstraintProfile.SHOOTING_ON_MOVE, true))
                .finallyDo(interrupted -> drive.setConstraintProfileActive(DriveConstants.ConstraintProfile.SHOOTING_ON_MOVE, false));
    }

    private void logTargetSelection(TargetSelection selection) {
        Logger.recordOutput("Shooting/ShootTargetMode", selection.mode().name());
        Logger.recordOutput("Shooting/InAllianceZone", selection.mode() == ShootTargetMode.SHOOT);
        Logger.recordOutput("Shooting/InOpponentAllianceZone",
                FieldConstants.isInOpponentAllianceZone(RobotState.getInstance().getPose()));
        Logger.recordOutput("Shooting/InNeutralBlockedBand",
                FieldConstants.isInHubBackBlockedNeutralBand(RobotState.getInstance().getPose()));
        Logger.recordOutput("Shooting/TargetHasFieldPose", selection.hasFieldTarget());
        Logger.recordOutput("Shooting/TargetPose", selection.hasFieldTarget() ? selection.targetPose() : new Pose2d());
        Logger.recordOutput("Shooting/TargetPoseX", selection.hasFieldTarget() ? selection.targetPose().getX() : Double.NaN);
        Logger.recordOutput("Shooting/TargetPoseY", selection.hasFieldTarget() ? selection.targetPose().getY() : Double.NaN);
        Logger.recordOutput("Shooting/TargetDistanceMeters", selection.distanceMeters());
        Logger.recordOutput(
                "Shooting/TargetHeadingForPoseDeg",
                selection.targetHeading() != null ? selection.targetHeading().getDegrees() : Double.NaN);
        Logger.recordOutput("Shooting/TargetIntent", selection.intent().name());
        Logger.recordOutput("Shooting/AllianceZoneBoundaryX", FieldConstants.getAllianceZoneBoundaryX());
        Logger.recordOutput("Shooting/OpponentAllianceZoneBoundaryX", FieldConstants.getOpponentAllianceZoneBoundaryX());
        Logger.recordOutput("Shooting/HubBackBlockLowerY", FieldConstants.getHubBackBlockLowerY());
        Logger.recordOutput("Shooting/HubBackBlockUpperY", FieldConstants.getHubBackBlockUpperY());
    }

    private static TargetSelection createManualDistanceTargetSelection(double distanceMeters) {
        return new TargetSelection(
                ShootTargetMode.MANUAL_DISTANCE,
                false,
                new Pose2d(),
                distanceMeters,
                null,
                ShotIntent.NONE);
    }

    private static TargetSelection toTargetSelection(ShotSolution solution) {
        if (solution == null || solution.intent() == ShotIntent.NONE || !solution.valid()) {
            Pose2d pose = solution != null ? solution.targetPose() : new Pose2d();
            return new TargetSelection(ShootTargetMode.NONE, false, pose, Double.NaN, null, ShotIntent.NONE);
        }
        return new TargetSelection(
                solution.intent() == ShotIntent.PASS ? ShootTargetMode.PASS : ShootTargetMode.SHOOT,
                true,
                solution.targetPose(),
                solution.distanceMeters(),
                solution.targetHeading(),
                solution.intent());
    }
}
