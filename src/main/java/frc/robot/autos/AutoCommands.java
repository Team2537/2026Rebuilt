package frc.robot.autos;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotState;
import frc.robot.commands.DriveCommands;
import frc.robot.coordination.shooting.ShootCoordinator;
import frc.robot.coordination.shooting.ShotAimReadiness;
import frc.robot.coordination.shooting.ShotSolution;
import frc.robot.coordination.shooting.ShotSolutionCalculator;
import frc.robot.coordination.shooting.ShotYawController;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.LaunchCalculator;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.util.AutoAimHeadingConfig;
import frc.robot.util.CycleCache;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/** Reusable autonomous command primitives intended for composition in PathPlanner. */
public final class AutoCommands {
    private static final long CONTROL_LOOP_PERIOD_US =
            Math.round(TimedRobot.kDefaultPeriod * 1_000_000.0);

    private AutoCommands() {}

    /**
     * Continuously tracks the hub shot target and spins the shooter without feeding.
     * Runs until interrupted.
     */
    public static Command aimForHub(
            Drive drive, Shooter shooter, Intake intake, ShootCoordinator shootCoordinator) {
        Supplier<ShotSolution> shotSolutionSupplier = createHubShotSolutionSupplier(
                shooter,
                false,
                AutoAimHeadingConfig.aimToleranceRad(),
                AutoAimHeadingConfig.aimReleaseToleranceRad(),
                ShooterConstants.scoreShooterRpmTolerance());
        return shootCoordinator.aimForShot(shotSolutionSupplier).withName("AutoAimForHub");
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
        Supplier<ShotSolution> shotSolutionSupplier = createHubShotSolutionSupplier(
                shooter,
                false,
                AutoAimHeadingConfig.aimToleranceRad(),
                AutoAimHeadingConfig.aimReleaseToleranceRad(),
                ShooterConstants.scoreShooterRpmTolerance());
        ShotAimReadiness aimReadiness = new ShotAimReadiness();
        BooleanSupplier aimReadySupplier =
                () -> aimReadiness.update(shotSolutionSupplier.get(), RobotState.getInstance().getRotation());

        return Commands.parallel(
                shootCoordinator.shootForShot(shotSolutionSupplier, aimReadySupplier, () -> false, () -> true),
                DriveCommands.driveWhileAiming(
                        drive,
                        () -> 0.0,
                        () -> 0.0,
                        () -> shotSolutionSupplier.get().desiredRobotHeading(),
                        () -> shotSolutionSupplier.get().desiredHeadingRateRadPerSec(),
                        () -> false));
    }

    /**
     * Tracks and shoots at the hub while another command is following a path.
     * Uses motion-compensated heading to override PathPlanner's rotation feedback.
     */
    public static Command shootHubOnMove(
            Drive drive, Shooter shooter, Intake intake, ShootCoordinator shootCoordinator) {
        return withSmartRetractDuringShoot(
                        createHubShootOnMoveCommand(shooter, shootCoordinator),
                        intake,
                        shootCoordinator)
                .withName("AutoShootHubOnMove");
    }

    /**
     * Tracks and shoots at the hub while another command is following a path.
     * Uses motion-compensated heading to override PathPlanner's rotation feedback and leaves smart retract disabled.
     */
    public static Command shootHubOnMoveWithoutSmartRetract(
            Drive drive, Shooter shooter, Intake intake, ShootCoordinator shootCoordinator) {
        return createHubShootOnMoveCommand(shooter, shootCoordinator)
                .withName("AutoShootHubOnMoveNoSmartRetract");
    }

    private static Command createHubShootOnMoveCommand(
            Shooter shooter, ShootCoordinator shootCoordinator) {
        Supplier<ShotSolution> shotSolutionSupplier = createHubShotSolutionSupplier(
                shooter,
                true,
                AutoAimHeadingConfig.movingAimToleranceRad(),
                AutoAimHeadingConfig.movingAimReleaseToleranceRad(),
                ShooterConstants.movingShooterRpmTolerance());
        ShotAimReadiness aimReadiness = new ShotAimReadiness();
        BooleanSupplier aimReadySupplier =
                () -> aimReadiness.update(shotSolutionSupplier.get(), RobotState.getInstance().getRotation());

        return withPathRotationFeedbackOverride(
                shootCoordinator.shootForShot(shotSolutionSupplier, aimReadySupplier, () -> false, () -> true),
                shotSolutionSupplier);
    }

    private static Command withSmartRetractDuringShoot(
            Command shootingCommand, Intake intake, ShootCoordinator shootCoordinator) {
        return Commands.parallel(
                shootingCommand,
                intake.smartRetractDuringShootCommand(shootCoordinator::isActivelyFeeding));
    }

    private static Supplier<ShotSolution> createHubShotSolutionSupplier(
            Shooter shooter,
            boolean movingShot,
            double headingToleranceRad,
            double headingReleaseToleranceRad,
            double shooterRpmTolerance) {
        RobotState robotState = RobotState.getInstance();
        ShotSolutionCalculator shotSolutionCalculator = new ShotSolutionCalculator(shooter);
        ShotSolutionCalculator.HeadingRateTracker headingRateTracker = shotSolutionCalculator.createHeadingRateTracker();
        CycleCache<LaunchCalculator.MotionCompensation> compensationCache = new CycleCache<>();
        CycleCache<ShotSolution> shotSolutionCache = new CycleCache<>();

        return () -> shotSolutionCache.get(loopCycleKey(), () -> {
            LaunchCalculator.MotionCompensation compensation = compensationCache.get(loopCycleKey(), () ->
                    shooter.getMotionCompensationToHub(
                            robotState.getPose(),
                            robotState.getMeasuredChassisSpeeds()));
            return shotSolutionCalculator.createHubScoreSolution(
                    compensation,
                    headingRateTracker,
                    movingShot,
                    headingToleranceRad,
                    headingReleaseToleranceRad,
                    shooterRpmTolerance);
        });
    }

    private static Command withPathRotationFeedbackOverride(
            Command command,
            Supplier<ShotSolution> shotSolutionSupplier) {
        ShotYawController yawController = new ShotYawController();
        return command
                .beforeStarting(() -> {
                    PPHolonomicDriveController.overrideRotationFeedback(() -> {
                        ShotSolution shotSolution = shotSolutionSupplier.get();
                        Logger.recordOutput(
                                "AutoAim/PathRotationOverrideTargetDeg",
                                shotSolution.valid() && shotSolution.desiredRobotHeading() != null
                                        ? shotSolution.desiredRobotHeading().getDegrees()
                                        : Double.NaN);
                        Logger.recordOutput(
                                "AutoAim/PathRotationOverrideTargetRateDegPerSec",
                                shotSolution.valid()
                                        ? Math.toDegrees(shotSolution.desiredHeadingRateRadPerSec())
                                        : Double.NaN);
                        if (!shotSolution.valid() || shotSolution.desiredRobotHeading() == null) {
                            return 0.0;
                        }
                        return yawController.calculateFeedback(
                                RobotState.getInstance().getRotation(),
                                RobotState.getInstance().getMeasuredChassisSpeeds().omegaRadiansPerSecond,
                                shotSolution.desiredRobotHeading(),
                                shotSolution.desiredHeadingRateRadPerSec(),
                                AutoAimHeadingConfig.headingProfileMaxVelocityRadPerSec());
                    });
                    Logger.recordOutput("AutoAim/PathRotationOverrideEnabled", true);
                })
                .finallyDo(interrupted -> {
                    PPHolonomicDriveController.clearRotationFeedbackOverride();
                    Logger.recordOutput("AutoAim/PathRotationOverrideEnabled", false);
                    Logger.recordOutput("AutoAim/PathRotationOverrideTargetDeg", Double.NaN);
                    Logger.recordOutput("AutoAim/PathRotationOverrideTargetRateDegPerSec", Double.NaN);
                });
    }

    private static long loopCycleKey() {
        if (CONTROL_LOOP_PERIOD_US <= 0L) {
            return RobotController.getFPGATime();
        }
        return RobotController.getFPGATime() / CONTROL_LOOP_PERIOD_US;
    }
}
