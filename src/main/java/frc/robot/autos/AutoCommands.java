package frc.robot.autos;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.coordination.shooting.ShootCoordinator;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.Shooter;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/** Reusable autonomous command primitives intended for composition in PathPlanner. */
public final class AutoCommands {
    private static final double AUTO_AIM_TOLERANCE_RAD = Math.toRadians(3.0);
    private static final double AUTO_AIM_RELEASE_TOLERANCE_RAD = Math.toRadians(4.0);

    private AutoCommands() {}

    private static DoubleSupplier createHubDistanceSupplier(Drive drive, Shooter shooter) {
        return () -> shooter.getMotionCompensatedHubDistanceMeters(
                drive.getPose(), drive.getMeasuredChassisSpeeds());
    }

    /**
     * Continuously tracks the hub shot target and spins the shooter without feeding.
     * Runs until interrupted.
     */
    public static Command aimForHub(
            Drive drive, Shooter shooter, ShootCoordinator shootCoordinator) {
        return shootCoordinator.aimForDistance(createHubDistanceSupplier(drive, shooter))
                .withName("AutoAimForHub");
    }

    /**
     * Continuously tracks the hub shot target and enables kicker/transfer once at setpoint.
     * Runs until interrupted.
     */
    public static Command shootHub(
            Drive drive, Shooter shooter, ShootCoordinator shootCoordinator) {
        return Commands.parallel(
                        shootCoordinator.shootForDistance(createHubDistanceSupplier(drive, shooter)),
                        Commands.run(drive::stopWithX, drive))
                .withName("AutoShootHub");
    }

    /**
     * Tracks and shoots at the hub while another command is following a path.
     * Uses motion-compensated heading to override PathPlanner's rotation target.
     */
    public static Command shootHubOnMove(
            Drive drive, Shooter shooter, ShootCoordinator shootCoordinator) {
        Supplier<Shooter.MotionCompensation> compensationSupplier =
                createHubMotionCompensationSupplier(drive, shooter);
        DoubleSupplier distanceSupplier =
                () -> compensationSupplier.get().compensatedDistanceMeters();
        Supplier<Rotation2d> targetHeadingSupplier =
                () -> compensationSupplier.get().compensatedHeading();
        Supplier<Rotation2d> desiredRobotHeadingSupplier =
                () -> {
                    Rotation2d targetHeading = targetHeadingSupplier.get();
                    return targetHeading == null ? null : targetHeading.plus(Rotation2d.kPi);
                };
        BooleanSupplier aimReadySupplier = createAimReadySupplier(drive, targetHeadingSupplier);

        return withPathRotationOverride(
                        shootCoordinator.shootForDistance(distanceSupplier, aimReadySupplier),
                        desiredRobotHeadingSupplier)
                .withName("AutoShootHubOnMove");
    }

    private static Supplier<Shooter.MotionCompensation> createHubMotionCompensationSupplier(
            Drive drive, Shooter shooter) {
        final long[] cachedCycleTimestampUs = new long[] {Long.MIN_VALUE};
        final Shooter.MotionCompensation[] cachedCompensation =
                new Shooter.MotionCompensation[] {
                    new Shooter.MotionCompensation(
                            Double.NaN,
                            Double.NaN,
                            Double.NaN,
                            Double.NaN,
                            Double.NaN,
                            null)
                };

        return () -> {
            long timestampUs = RobotController.getFPGATime();
            if (timestampUs != cachedCycleTimestampUs[0]) {
                cachedCycleTimestampUs[0] = timestampUs;
                cachedCompensation[0] = shooter.getMotionCompensationToHub(
                        drive.getPose(),
                        drive.getMeasuredChassisSpeeds());
            }
            return cachedCompensation[0];
        };
    }

    private static BooleanSupplier createAimReadySupplier(
            Drive drive, Supplier<Rotation2d> targetHeadingSupplier) {
        final boolean[] aimReadyLatched = new boolean[] {false};
        final Rotation2d[] lastValidTargetHeading = new Rotation2d[] {null};
        final double[] lastValidTargetTimestampSec = new double[] {Double.NaN};

        return () -> {
            Rotation2d targetHeading = targetHeadingSupplier.get();
            double nowSec = Timer.getFPGATimestamp();
            boolean targetHeld = false;

            if (targetHeading != null) {
                lastValidTargetHeading[0] = targetHeading;
                lastValidTargetTimestampSec[0] = nowSec;
            } else if (lastValidTargetHeading[0] != null) {
                double targetAgeSec = nowSec - lastValidTargetTimestampSec[0];
                if (Double.isFinite(targetAgeSec)
                        && targetAgeSec <= Constants.SHOOTING_AIM_TARGET_HOLD_SEC) {
                    targetHeading = lastValidTargetHeading[0];
                    targetHeld = true;
                }
            }

            Rotation2d robotHeading = drive.getRotation();
            Logger.recordOutput("AutoAim/RobotHeadingDeg", robotHeading.getDegrees());
            Logger.recordOutput("AutoAim/TargetHeld", targetHeld);
            Logger.recordOutput("AutoAim/TargetAvailable", targetHeading != null);
            Logger.recordOutput(
                    "AutoAim/TargetAgeSec",
                    Double.isFinite(lastValidTargetTimestampSec[0])
                            ? nowSec - lastValidTargetTimestampSec[0]
                            : Double.NaN);

            if (targetHeading == null) {
                Logger.recordOutput("AutoAim/TargetHeadingDeg", Double.NaN);
                Logger.recordOutput("AutoAim/DesiredRobotHeadingDeg", Double.NaN);
                Logger.recordOutput("AutoAim/AimErrorRad", Double.NaN);
                Logger.recordOutput("AutoAim/AimErrorDeg", Double.NaN);
                Logger.recordOutput("AutoAim/AimReadyLatched", false);
                aimReadyLatched[0] = false;
                return false;
            }

            Rotation2d desiredRobotHeading = targetHeading.plus(Rotation2d.kPi);
            double headingErrorRad = MathUtil.angleModulus(
                    desiredRobotHeading.minus(robotHeading).getRadians());
            double absHeadingErrorRad = Math.abs(headingErrorRad);

            if (aimReadyLatched[0]) {
                aimReadyLatched[0] =
                        absHeadingErrorRad <= AUTO_AIM_RELEASE_TOLERANCE_RAD;
            } else {
                aimReadyLatched[0] =
                        absHeadingErrorRad <= AUTO_AIM_TOLERANCE_RAD;
            }

            Logger.recordOutput("AutoAim/TargetHeadingDeg", targetHeading.getDegrees());
            Logger.recordOutput("AutoAim/DesiredRobotHeadingDeg", desiredRobotHeading.getDegrees());
            Logger.recordOutput("AutoAim/AimErrorRad", headingErrorRad);
            Logger.recordOutput("AutoAim/AimErrorDeg", Math.toDegrees(headingErrorRad));
            Logger.recordOutput("AutoAim/AimReadyLatched", aimReadyLatched[0]);
            return aimReadyLatched[0];
        };
    }

    @SuppressWarnings("deprecation")
    private static Command withPathRotationOverride(
            Command command,
            Supplier<Rotation2d> desiredRobotHeadingSupplier) {
        Supplier<Optional<Rotation2d>> rotationOverrideSupplier = () -> {
            Rotation2d desiredHeading = desiredRobotHeadingSupplier.get();
            Logger.recordOutput(
                    "AutoAim/PathRotationOverrideTargetDeg",
                    desiredHeading != null ? desiredHeading.getDegrees() : Double.NaN);
            return Optional.ofNullable(desiredHeading);
        };

        return command
                .beforeStarting(() -> {
                    PPHolonomicDriveController.setRotationTargetOverride(rotationOverrideSupplier);
                    Logger.recordOutput("AutoAim/PathRotationOverrideEnabled", true);
                })
                .finallyDo(interrupted -> {
                    PPHolonomicDriveController.setRotationTargetOverride(null);
                    Logger.recordOutput("AutoAim/PathRotationOverrideEnabled", false);
                    Logger.recordOutput("AutoAim/PathRotationOverrideTargetDeg", Double.NaN);
                });
    }
}
