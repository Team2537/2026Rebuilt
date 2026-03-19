package frc.robot.coordination.shooting;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
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
            Supplier<ShotSolution> hubShotSolutionSupplier) {}

    public enum RightTriggerMode {
        SHOOT,
        PASS,
        NOTHING
    }

    public record TargetSelection(
            RightTriggerMode mode,
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
        return new AimingContext(
                () -> scoreSolutionSupplier.get().distanceMeters(),
                () -> scoreSolutionSupplier.get().targetHeading(),
                createAimReadySupplier(scoreSolutionSupplier),
                scoreSolutionSupplier);
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
                        .withName("ShooterTriggerSelectedMode"));
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

    public Command createSelectedShootWithoutAimCommand(
            DoubleSupplier xSupplier,
            DoubleSupplier ySupplier,
            DoubleSupplier omegaSupplier,
            DoubleSupplier hubDistanceMetersSupplier,
            Supplier<Rotation2d> hubTargetHeadingSupplier,
            BooleanSupplier manualFeedOverrideSupplier) {
        return withShootingConstraintProfile(
                Commands.either(
                        createOverrideAutoAimShootWithoutAimCommand(
                                xSupplier,
                                ySupplier,
                                omegaSupplier,
                                manualFeedOverrideSupplier),
                        createZoneAwareShootWithoutAimCommand(
                                xSupplier,
                                ySupplier,
                                omegaSupplier,
                                hubDistanceMetersSupplier,
                                hubTargetHeadingSupplier,
                                manualFeedOverrideSupplier),
                        () -> dashboardOverrides.isAutoAimEnabled()
                                && FieldConstants.isInAllianceZone(RobotState.getInstance().getPose()))
                        .withName("ShooterTriggerSelectedMode"));
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

    private TargetSelection buildTelemetryTargetSelection() {
        Pose2d robotPose = RobotState.getInstance().getPose();
        return buildTargetSelection(
                robotPose,
                LaunchCalculator.getHubDistanceMeters(robotPose),
                FieldConstants.getHubFacingHeading(robotPose));
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
                logTargetSelection(toTargetSelection(solution));
                return solution;
            }

            scoreTracker.reset();
            if (FieldConstants.isInHubBackBlockedNeutralBand(robotPose)) {
                passTracker.reset();
                ShotSolution invalid = ShotSolution.invalid(robotPose != null ? robotPose : new Pose2d());
                logTargetSelection(toTargetSelection(invalid));
                return invalid;
            }

            ShotSolution passSolution = shotSolutionCalculator.createPassSolution(
                    robotPose,
                    passTracker,
                    AutoAimHeadingConfig.passAimToleranceRad(),
                    AutoAimHeadingConfig.passAimReleaseToleranceRad(),
                    ShooterConstants.passingShooterRpmTolerance());
            logTargetSelection(toTargetSelection(passSolution));
            return passSolution;
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
                createBlockedRightTriggerCommand(xSupplier, ySupplier, omegaSupplier),
                createShootCommand(
                        xSupplier,
                        ySupplier,
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
                createBlockedRightTriggerCommand(xSupplier, ySupplier, omegaSupplier),
                createAimCommand(xSupplier, ySupplier, shotSolutionSupplier),
                () -> shotSolutionSupplier.get().intent() == ShotIntent.NONE);
    }

    private Command createAimCommand(
            DoubleSupplier xSupplier,
            DoubleSupplier ySupplier,
            Supplier<ShotSolution> shotSolutionSupplier) {
        return Commands.parallel(
                createDriveWhileAimingCommand(xSupplier, ySupplier, shotSolutionSupplier, () -> false),
                shootCoordinator.aimForShot(shotSolutionSupplier))
                .withName("ShooterTriggerAimOnly");
    }

    private Command createZoneAwareShootWithoutAimCommand(
            DoubleSupplier xSupplier,
            DoubleSupplier ySupplier,
            DoubleSupplier omegaSupplier,
            DoubleSupplier hubDistanceMetersSupplier,
            Supplier<Rotation2d> hubTargetHeadingSupplier,
            BooleanSupplier manualFeedOverrideSupplier) {
        Supplier<ShotSolution> shotSolutionSupplier = createSelectedShotSolutionSupplier(
                hubDistanceMetersSupplier,
                hubTargetHeadingSupplier,
                AutoAimHeadingConfig.aimToleranceRad(),
                AutoAimHeadingConfig.aimReleaseToleranceRad(),
                ShooterConstants.scoreShooterRpmTolerance(),
                null);
        return Commands.either(
                createBlockedRightTriggerCommand(xSupplier, ySupplier, omegaSupplier),
                createShootWithoutAimCommand(
                        xSupplier,
                        ySupplier,
                        omegaSupplier,
                        shotSolutionSupplier,
                        manualFeedOverrideSupplier),
                () -> shotSolutionSupplier.get().intent() == ShotIntent.NONE);
    }

    private Command createBlockedRightTriggerCommand(
            DoubleSupplier xSupplier,
            DoubleSupplier ySupplier,
            DoubleSupplier omegaSupplier) {
        return DriveCommands.joystickDrive(drive, xSupplier, ySupplier, omegaSupplier)
                .withName("DriveJoystickBlockedRightTrigger");
    }

    private Command createShootWithoutAimCommand(
            DoubleSupplier xSupplier,
            DoubleSupplier ySupplier,
            DoubleSupplier omegaSupplier,
            Supplier<ShotSolution> shotSolutionSupplier,
            BooleanSupplier manualFeedOverrideSupplier) {
        return Commands.parallel(
                DriveCommands.joystickDrive(
                        drive,
                        xSupplier,
                        ySupplier,
                        omegaSupplier)
                        .withName("DriveJoystickShootNoAim"),
                shootCoordinator.shootForShot(
                                shotSolutionSupplier,
                                () -> true,
                                manualFeedOverrideSupplier,
                                () -> !dashboardOverrides.isFeedingDisabled())
                        .withName("ShooterShootWithoutAim"),
                intake.smartRetractDuringShootCommand(shootCoordinator::isActivelyFeeding))
                .withName("ShooterTriggerFeedAndShootNoAim");
    }

    private Command createShootCommand(
            DoubleSupplier xSupplier,
            DoubleSupplier ySupplier,
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
                createDriveWhileAimingCommand(xSupplier, ySupplier, shotSolutionSupplier, readyToFeedSupplier),
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
            Supplier<ShotSolution> shotSolutionSupplier,
            BooleanSupplier xLockConditionSupplier) {
        return DriveCommands.driveWhileAiming(
                drive,
                xSupplier,
                ySupplier,
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

    private Command createOverrideAutoAimShootWithoutAimCommand(
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

    private void logTargetSelection(TargetSelection selection) {
        Logger.recordOutput("Shooting/RightTriggerMode", selection.mode().name());
        Logger.recordOutput("Shooting/InAllianceZone", selection.mode() == RightTriggerMode.SHOOT);
        Logger.recordOutput("Shooting/InOpponentAllianceZone",
                FieldConstants.isInOpponentAllianceZone(RobotState.getInstance().getPose()));
        Logger.recordOutput("Shooting/InNeutralBlockedBand",
                FieldConstants.isInHubBackBlockedNeutralBand(RobotState.getInstance().getPose()));
        Logger.recordOutput("Shooting/TargetPose", selection.targetPose());
        Logger.recordOutput("Shooting/TargetPoseX", selection.targetPose().getX());
        Logger.recordOutput("Shooting/TargetPoseY", selection.targetPose().getY());
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

    private static TargetSelection buildTargetSelection(
            Pose2d robotPose,
            double hubDistanceMeters,
            Rotation2d hubTargetHeading) {
        if (FieldConstants.isInAllianceZone(robotPose)) {
            return new TargetSelection(
                    RightTriggerMode.SHOOT,
                    new Pose2d(FieldConstants.getHubTargetTranslation(), Rotation2d.kZero),
                    hubDistanceMeters,
                    hubTargetHeading,
                    ShotIntent.SCORE);
        }

        if (FieldConstants.isInHubBackBlockedNeutralBand(robotPose)) {
            Pose2d blockedPose = robotPose != null ? robotPose : new Pose2d();
            return new TargetSelection(
                    RightTriggerMode.NOTHING,
                    blockedPose,
                    Double.NaN,
                    null,
                    ShotIntent.NONE);
        }

        Pose2d passTargetPose = FieldConstants.getPassTargetPose(robotPose);
        return new TargetSelection(
                RightTriggerMode.PASS,
                passTargetPose,
                ShotSolutionCalculator.getShooterDistanceToTarget(robotPose, passTargetPose.getTranslation()),
                FieldConstants.getHeadingToTarget(robotPose, passTargetPose.getTranslation()),
                ShotIntent.PASS);
    }

    private static TargetSelection toTargetSelection(ShotSolution solution) {
        if (solution == null || solution.intent() == ShotIntent.NONE || !solution.valid()) {
            Pose2d pose = solution != null ? solution.targetPose() : new Pose2d();
            return new TargetSelection(RightTriggerMode.NOTHING, pose, Double.NaN, null, ShotIntent.NONE);
        }
        return new TargetSelection(
                solution.intent() == ShotIntent.PASS ? RightTriggerMode.PASS : RightTriggerMode.SHOOT,
                solution.targetPose(),
                solution.distanceMeters(),
                solution.targetHeading(),
                solution.intent());
    }
}
