package frc.robot.autos;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotState;
import frc.robot.commands.DriveCommands;
import frc.robot.coordination.shooting.ShootCoordinator;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.LaunchCalculator;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.util.AimReadyLatch;
import frc.robot.util.AutoAimHeadingConfig;
import frc.robot.util.CycleCache;
import frc.robot.util.TargetHoldover;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/** Reusable autonomous command primitives intended for composition in PathPlanner. */
public final class AutoCommands {
    private static final long CONTROL_LOOP_PERIOD_US =
            Math.round(TimedRobot.kDefaultPeriod * 1_000_000.0);

    private AutoCommands() {}

    private static DoubleSupplier createHubDistanceSupplier(Shooter shooter) {
        RobotState robotState = RobotState.getInstance();
        return () -> shooter.getMotionCompensatedHubDistanceMeters(
                robotState.getPose(), robotState.getMeasuredChassisSpeeds());
    }

    /**
     * Continuously tracks the hub shot target and spins the shooter without feeding.
     * Runs until interrupted.
     */
    public static Command aimForHub(
            Drive drive, Shooter shooter, Intake intake, ShootCoordinator shootCoordinator) {
        return shootCoordinator.aimForDistance(createHubDistanceSupplier(shooter))
                .withName("AutoAimForHub");
    }

    /**
     * Continuously tracks the hub shot target and enables kicker/transfer once at setpoint.
     * Runs until interrupted.
     */
    public static Command shootHub(
            Drive drive, Shooter shooter, Intake intake, ShootCoordinator shootCoordinator) {
        return withSmartRetractDuringShoot(
                        createHubShootCommand(drive, shooter, shootCoordinator),
                        intake,
                        shootCoordinator)
                .withName("AutoShootHub");
    }

    /**
     * Continuously tracks the hub shot target and enables kicker/transfer once at setpoint.
     * Runs until interrupted and intentionally leaves intake smart retract disabled.
     */
    public static Command shootHubWithoutSmartRetract(
            Drive drive, Shooter shooter, Intake intake, ShootCoordinator shootCoordinator) {
        return createHubShootCommand(drive, shooter, shootCoordinator)
                .withName("AutoShootHubNoSmartRetract");
    }

    private static Command createHubShootCommand(
            Drive drive, Shooter shooter, ShootCoordinator shootCoordinator) {
        Supplier<LaunchCalculator.MotionCompensation> compensationSupplier =
                createHubMotionCompensationSupplier(shooter);
        DoubleSupplier distanceSupplier =
                () -> compensationSupplier.get().compensatedDistanceMeters();
        Supplier<Rotation2d> targetHeadingSupplier =
                () -> compensationSupplier.get().compensatedHeading();
        Supplier<Rotation2d> rawDesiredRobotHeadingSupplier =
                createCycleCachedRotationSupplier(() -> {
                    Rotation2d targetHeading = targetHeadingSupplier.get();
                    return targetHeading == null ? null : targetHeading.plus(Rotation2d.kPi);
                });
        BooleanSupplier aimReadySupplier = createAimReadySupplier(
                rawDesiredRobotHeadingSupplier,
                AutoAimHeadingConfig.AIM_TOLERANCE_RAD,
                AutoAimHeadingConfig.AIM_RELEASE_TOLERANCE_RAD);

        return Commands.parallel(
                shootCoordinator.shootForDistance(distanceSupplier, aimReadySupplier),
                DriveCommands.autoAlignToHubPose(
                        drive,
                        () -> 0.0,
                        () -> 0.0,
                        () -> 0.0,
                        targetHeadingSupplier));
    }

    /**
     * Tracks and shoots at the hub while another command is following a path.
     * Uses motion-compensated heading to override PathPlanner's rotation target.
     */
    public static Command shootHubOnMove(
            Drive drive, Shooter shooter, Intake intake, ShootCoordinator shootCoordinator) {
        return withSmartRetractDuringShoot(
                        createHubShootOnMoveCommand(drive, shooter, shootCoordinator),
                        intake,
                        shootCoordinator)
                .withName("AutoShootHubOnMove");
    }

    /**
     * Tracks and shoots at the hub while another command is following a path.
     * Uses motion-compensated heading to override PathPlanner's rotation target and leaves smart retract disabled.
     */
    public static Command shootHubOnMoveWithoutSmartRetract(
            Drive drive, Shooter shooter, Intake intake, ShootCoordinator shootCoordinator) {
        return createHubShootOnMoveCommand(drive, shooter, shootCoordinator)
                .withName("AutoShootHubOnMoveNoSmartRetract");
    }

    private static Command createHubShootOnMoveCommand(
            Drive drive, Shooter shooter, ShootCoordinator shootCoordinator) {
        Supplier<LaunchCalculator.MotionCompensation> compensationSupplier =
                createHubMotionCompensationSupplier(shooter);
        DoubleSupplier distanceSupplier =
                () -> compensationSupplier.get().compensatedDistanceMeters();
        Supplier<Rotation2d> targetHeadingSupplier =
                () -> compensationSupplier.get().compensatedHeading();
        Supplier<Rotation2d> rawDesiredRobotHeadingSupplier =
                createCycleCachedRotationSupplier(() -> {
                    Rotation2d targetHeading = targetHeadingSupplier.get();
                    return targetHeading == null ? null : targetHeading.plus(Rotation2d.kPi);
                });
        ProfiledHeadingTarget profiledHeadingTarget =
                new ProfiledHeadingTarget(AutoAimHeadingConfig.createHeadingProfileConstraints());
        Supplier<Rotation2d> profiledDesiredRobotHeadingSupplier =
                createCycleCachedRotationSupplier(
                        () -> profiledHeadingTarget.calculate(rawDesiredRobotHeadingSupplier.get()));
        BooleanSupplier aimReadySupplier = createAimReadySupplier(
                profiledDesiredRobotHeadingSupplier,
                AutoAimHeadingConfig.SHOT_ON_MOVE_AIM_TOLERANCE_RAD,
                AutoAimHeadingConfig.SHOT_ON_MOVE_AIM_RELEASE_TOLERANCE_RAD);

        return withPathRotationOverride(
                shootCoordinator.shootForDistance(
                        distanceSupplier,
                        aimReadySupplier,
                        () -> true,
                        () -> false,
                        () -> true),
                rawDesiredRobotHeadingSupplier,
                profiledDesiredRobotHeadingSupplier,
                profiledHeadingTarget);
    }

    private static Command withSmartRetractDuringShoot(
            Command shootingCommand, Intake intake, ShootCoordinator shootCoordinator) {
        return Commands.parallel(
                shootingCommand,
                intake.smartRetractDuringShootCommand(shootCoordinator::isActivelyFeeding));
    }

    private static Supplier<LaunchCalculator.MotionCompensation> createHubMotionCompensationSupplier(
            Shooter shooter) {
        RobotState robotState = RobotState.getInstance();
        CycleCache<LaunchCalculator.MotionCompensation> cache = new CycleCache<>();

        return () -> cache.get(loopCycleKey(), () -> shooter.getMotionCompensationToHub(
                robotState.getPose(),
                robotState.getMeasuredChassisSpeeds()));
    }

    private static BooleanSupplier createAimReadySupplier(
            Supplier<Rotation2d> desiredRobotHeadingSupplier) {
        return createAimReadySupplier(
                desiredRobotHeadingSupplier,
                AutoAimHeadingConfig.AIM_TOLERANCE_RAD,
                AutoAimHeadingConfig.AIM_RELEASE_TOLERANCE_RAD);
    }

    private static BooleanSupplier createAimReadySupplier(
            Supplier<Rotation2d> desiredRobotHeadingSupplier,
            double aimToleranceRad,
            double aimReleaseToleranceRad) {
        TargetHoldover<Rotation2d> holdover =
                new TargetHoldover<>(AutoAimHeadingConfig.TARGET_HOLD_SEC, Timer::getFPGATimestamp);
        AimReadyLatch latch = new AimReadyLatch(
                aimToleranceRad,
                aimReleaseToleranceRad);

        return () -> {
            TargetHoldover.HoldResult<Rotation2d> holdResult =
                    holdover.apply(desiredRobotHeadingSupplier.get());
            Rotation2d desiredRobotHeading = holdResult.value();
            Rotation2d robotHeading = RobotState.getInstance().getRotation();
            Logger.recordOutput("AutoAim/RobotHeadingDeg", robotHeading.getDegrees());
            Logger.recordOutput("AutoAim/TargetHeld", holdResult.held());
            Logger.recordOutput("AutoAim/TargetAvailable", desiredRobotHeading != null);
            Logger.recordOutput("AutoAim/TargetAgeSec", holdResult.ageSec());

            if (desiredRobotHeading == null) {
                Logger.recordOutput("AutoAim/TargetHeadingDeg", Double.NaN);
                Logger.recordOutput("AutoAim/DesiredRobotHeadingDeg", Double.NaN);
                Logger.recordOutput("AutoAim/AimErrorRad", Double.NaN);
                Logger.recordOutput("AutoAim/AimErrorDeg", Double.NaN);
                Logger.recordOutput("AutoAim/AimReadyLatched", false);
                latch.reset();
                return false;
            }

            Rotation2d targetHeading = desiredRobotHeading.minus(Rotation2d.kPi);
            double headingErrorRad = MathUtil.angleModulus(
                    desiredRobotHeading.minus(robotHeading).getRadians());
            double absHeadingErrorRad = Math.abs(headingErrorRad);
            boolean aimReady = latch.update(absHeadingErrorRad);

            Logger.recordOutput("AutoAim/TargetHeadingDeg", targetHeading.getDegrees());
            Logger.recordOutput("AutoAim/DesiredRobotHeadingDeg", desiredRobotHeading.getDegrees());
            Logger.recordOutput("AutoAim/AimErrorRad", headingErrorRad);
            Logger.recordOutput("AutoAim/AimErrorDeg", Math.toDegrees(headingErrorRad));
            Logger.recordOutput("AutoAim/AimReadyLatched", aimReady);
            return aimReady;
        };
    }

    @SuppressWarnings("deprecation")
    private static Command withPathRotationOverride(
            Command command,
            Supplier<Rotation2d> rawDesiredRobotHeadingSupplier,
            Supplier<Rotation2d> desiredRobotHeadingSupplier,
            ProfiledHeadingTarget profiledHeadingTarget) {
        Supplier<Optional<Rotation2d>> rotationOverrideSupplier = () -> {
            Rotation2d rawDesiredHeading = rawDesiredRobotHeadingSupplier.get();
            Rotation2d desiredHeading = desiredRobotHeadingSupplier.get();
            Logger.recordOutput(
                    "AutoAim/PathRotationOverrideRawTargetDeg",
                    rawDesiredHeading != null ? rawDesiredHeading.getDegrees() : Double.NaN);
            Logger.recordOutput(
                    "AutoAim/PathRotationOverrideTargetDeg",
                    desiredHeading != null ? desiredHeading.getDegrees() : Double.NaN);
            Logger.recordOutput(
                    "AutoAim/PathRotationOverrideProfileVelocityDegPerSec",
                    desiredHeading != null
                            ? Math.toDegrees(profiledHeadingTarget.getVelocityRadPerSec())
                            : Double.NaN);
            return Optional.ofNullable(desiredHeading);
        };

        return command
                .beforeStarting(() -> {
                    profiledHeadingTarget.reset(RobotState.getInstance().getRotation());
                    PPHolonomicDriveController.setRotationTargetOverride(rotationOverrideSupplier);
                    Logger.recordOutput("AutoAim/PathRotationOverrideEnabled", true);
                })
                .finallyDo(interrupted -> {
                    PPHolonomicDriveController.setRotationTargetOverride(null);
                    Logger.recordOutput("AutoAim/PathRotationOverrideEnabled", false);
                    Logger.recordOutput("AutoAim/PathRotationOverrideTargetDeg", Double.NaN);
                    Logger.recordOutput("AutoAim/PathRotationOverrideRawTargetDeg", Double.NaN);
                    Logger.recordOutput("AutoAim/PathRotationOverrideProfileVelocityDegPerSec", Double.NaN);
                });
    }

    private static Supplier<Rotation2d> createCycleCachedRotationSupplier(
            Supplier<Rotation2d> sourceSupplier) {
        CycleCache<Rotation2d> cache = new CycleCache<>();
        return () -> cache.get(loopCycleKey(), sourceSupplier);
    }

    private static long loopCycleKey() {
        if (CONTROL_LOOP_PERIOD_US <= 0L) {
            return RobotController.getFPGATime();
        }
        return RobotController.getFPGATime() / CONTROL_LOOP_PERIOD_US;
    }

    private static final class ProfiledHeadingTarget {
        private final ProfiledPIDController controller;
        private boolean initialized = false;

        ProfiledHeadingTarget(TrapezoidProfile.Constraints constraints) {
            controller = new ProfiledPIDController(0, 0, 0, constraints, TimedRobot.kDefaultPeriod);
            controller.enableContinuousInput(-Math.PI, Math.PI);
        }

        void reset(Rotation2d currentHeading) {
            double headingRad = currentHeading != null ? currentHeading.getRadians() : 0.0;
            controller.reset(headingRad);
            initialized = true;
        }

        Rotation2d calculate(Rotation2d goalHeading) {
            if (goalHeading == null) {
                controller.reset(controller.getSetpoint().position);
                return null;
            }
            if (!initialized) {
                reset(goalHeading);
            }
            controller.calculate(controller.getSetpoint().position, goalHeading.getRadians());
            return Rotation2d.fromRadians(controller.getSetpoint().position);
        }

        double getVelocityRadPerSec() {
            return controller.getSetpoint().velocity;
        }
    }
}
