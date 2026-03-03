package frc.robot.coordination.shooting;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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
            BooleanSupplier aimReadySupplier) {
        return withShootingConstraintProfile(
                Commands.either(
                        createOverrideAutoAimShootCommand(xSupplier, ySupplier, omegaFallbackSupplier),
                        createShootCommand(
                                xSupplier,
                                ySupplier,
                                omegaFallbackSupplier,
                                distanceMetersSupplier,
                                targetHeadingSupplier,
                                aimReadySupplier),
                        dashboardOverrides::isAutoAimEnabled)
                        .withName("ShooterTriggerSelectedMode"));
    }

    public Command createSelectedAimOnlyCommand(
            DoubleSupplier xSupplier,
            DoubleSupplier ySupplier,
            DoubleSupplier omegaFallbackSupplier,
            DoubleSupplier distanceMetersSupplier,
            Supplier<Rotation2d> targetHeadingSupplier) {
        Command normalAimOnly = Commands.parallel(
                createAutoAlignCommand(
                        xSupplier,
                        ySupplier,
                        omegaFallbackSupplier,
                        targetHeadingSupplier),
                shootCoordinator.aimForDistance(distanceMetersSupplier))
                .withName("ShooterAimOnly");
        Command overrideAimOnly = shootCoordinator.aimForDistance(dashboardOverrides::getAimDistanceMeters)
                .withName("ShooterAimOnlyOverrideDistance");
        return Commands.either(overrideAimOnly, normalAimOnly, dashboardOverrides::isAutoAimEnabled)
                .withName("ShooterAimOnlySelectedMode");
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
        return DriveCommands.autoAlignToHubPose(
                drive,
                xSupplier,
                ySupplier,
                omegaFallbackSupplier,
                targetHeadingSupplier);
    }

    private Command createShootCommand(
            DoubleSupplier xSupplier,
            DoubleSupplier ySupplier,
            DoubleSupplier omegaFallbackSupplier,
            DoubleSupplier distanceMetersSupplier,
            Supplier<Rotation2d> targetHeadingSupplier,
            BooleanSupplier aimReadySupplier) {
        return Commands.parallel(
                createAutoAlignCommand(
                        xSupplier,
                        ySupplier,
                        omegaFallbackSupplier,
                        targetHeadingSupplier),
                shootCoordinator.shootForDistance(distanceMetersSupplier, aimReadySupplier),
                intake.smartRetractDuringShootCommand(shootCoordinator::isActivelyFeeding))
                .withName("ShooterTriggerAimAndShoot");
    }

    private Command createOverrideAutoAimShootCommand(
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
                shootCoordinator.shootForDistance(dashboardOverrides::getAimDistanceMeters, () -> true)
                        .withName("ShooterShootOverrideDistance"),
                intake.smartRetractDuringShootCommand(shootCoordinator::isActivelyFeeding))
                .withName("ShooterTriggerOverrideAutoAimShoot");
    }

    private Command withShootingConstraintProfile(Command command) {
        return command
                .beforeStarting(() -> drive.setConstraintProfileActive(DriveConstants.ConstraintProfile.SHOOTING_ON_MOVE, true))
                .finallyDo(interrupted -> drive.setConstraintProfileActive(DriveConstants.ConstraintProfile.SHOOTING_ON_MOVE, false));
    }
}
