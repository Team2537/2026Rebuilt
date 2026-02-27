package frc.robot.autos;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
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
    private static final long CONTROL_LOOP_PERIOD_US = 20_000L;
    private static final double AUTO_AIM_HEADING_PROFILE_MAX_VELOCITY_RAD_PER_SEC = Math.toRadians(540.0);
    private static final double AUTO_AIM_HEADING_PROFILE_MAX_ACCELERATION_RAD_PER_SEC2 = Math.toRadians(2160.0);
    private static final TrapezoidProfile.Constraints AUTO_AIM_HEADING_PROFILE_CONSTRAINTS =
            new TrapezoidProfile.Constraints(
                    AUTO_AIM_HEADING_PROFILE_MAX_VELOCITY_RAD_PER_SEC,
                    AUTO_AIM_HEADING_PROFILE_MAX_ACCELERATION_RAD_PER_SEC2);

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
        Supplier<Rotation2d> rawDesiredRobotHeadingSupplier =
                createCycleCachedRotationSupplier(() -> {
                    Rotation2d targetHeading = targetHeadingSupplier.get();
                    return targetHeading == null ? null : targetHeading.plus(Rotation2d.kPi);
                });
        ProfiledHeadingTarget profiledHeadingTarget =
                new ProfiledHeadingTarget(AUTO_AIM_HEADING_PROFILE_CONSTRAINTS);
        Supplier<Rotation2d> profiledDesiredRobotHeadingSupplier =
                createCycleCachedRotationSupplier(
                        () -> profiledHeadingTarget.calculate(rawDesiredRobotHeadingSupplier.get()));
        BooleanSupplier aimReadySupplier =
                createAimReadySupplier(drive, profiledDesiredRobotHeadingSupplier);

        return withPathRotationOverride(
                        shootCoordinator.shootForDistance(distanceSupplier, aimReadySupplier),
                        drive,
                        rawDesiredRobotHeadingSupplier,
                        profiledDesiredRobotHeadingSupplier,
                        profiledHeadingTarget)
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
            Drive drive, Supplier<Rotation2d> desiredRobotHeadingSupplier) {
        final boolean[] aimReadyLatched = new boolean[] {false};
        final Rotation2d[] lastValidDesiredRobotHeading = new Rotation2d[] {null};
        final double[] lastValidTargetTimestampSec = new double[] {Double.NaN};

        return () -> {
            Rotation2d desiredRobotHeading = desiredRobotHeadingSupplier.get();
            double nowSec = Timer.getFPGATimestamp();
            boolean targetHeld = false;

            if (desiredRobotHeading != null) {
                lastValidDesiredRobotHeading[0] = desiredRobotHeading;
                lastValidTargetTimestampSec[0] = nowSec;
            } else if (lastValidDesiredRobotHeading[0] != null) {
                double targetAgeSec = nowSec - lastValidTargetTimestampSec[0];
                if (Double.isFinite(targetAgeSec)
                        && targetAgeSec <= Constants.SHOOTING_AIM_TARGET_HOLD_SEC) {
                    desiredRobotHeading = lastValidDesiredRobotHeading[0];
                    targetHeld = true;
                }
            }

            Rotation2d robotHeading = drive.getRotation();
            Logger.recordOutput("AutoAim/RobotHeadingDeg", robotHeading.getDegrees());
            Logger.recordOutput("AutoAim/TargetHeld", targetHeld);
            Logger.recordOutput("AutoAim/TargetAvailable", desiredRobotHeading != null);
            Logger.recordOutput(
                    "AutoAim/TargetAgeSec",
                    Double.isFinite(lastValidTargetTimestampSec[0])
                            ? nowSec - lastValidTargetTimestampSec[0]
                            : Double.NaN);

            if (desiredRobotHeading == null) {
                Logger.recordOutput("AutoAim/TargetHeadingDeg", Double.NaN);
                Logger.recordOutput("AutoAim/DesiredRobotHeadingDeg", Double.NaN);
                Logger.recordOutput("AutoAim/AimErrorRad", Double.NaN);
                Logger.recordOutput("AutoAim/AimErrorDeg", Double.NaN);
                Logger.recordOutput("AutoAim/AimReadyLatched", false);
                aimReadyLatched[0] = false;
                return false;
            }

            Rotation2d targetHeading = desiredRobotHeading.minus(Rotation2d.kPi);
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
            Drive drive,
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
                    profiledHeadingTarget.reset(drive.getRotation());
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
        final long[] lastLoop = new long[] {Long.MIN_VALUE};
        final Rotation2d[] cachedValue = new Rotation2d[] {null};
        return () -> {
            long currentLoop = RobotController.getFPGATime() / CONTROL_LOOP_PERIOD_US;
            if (currentLoop != lastLoop[0]) {
                lastLoop[0] = currentLoop;
                cachedValue[0] = sourceSupplier.get();
            }
            return cachedValue[0];
        };
    }

    private static final class ProfiledHeadingTarget {
        private final TrapezoidProfile.Constraints constraints;
        private TrapezoidProfile.State state = new TrapezoidProfile.State();
        private double lastTimestampSec = Double.NaN;
        private boolean initialized = false;

        ProfiledHeadingTarget(TrapezoidProfile.Constraints constraints) {
            this.constraints = constraints;
        }

        void reset(Rotation2d currentHeading) {
            double headingRad = currentHeading != null ? currentHeading.getRadians() : 0.0;
            state = new TrapezoidProfile.State(headingRad, 0.0);
            lastTimestampSec = Timer.getFPGATimestamp();
            initialized = true;
        }

        Rotation2d calculate(Rotation2d goalHeading) {
            double nowSec = Timer.getFPGATimestamp();
            if (goalHeading == null) {
                state = new TrapezoidProfile.State(state.position, 0.0);
                lastTimestampSec = nowSec;
                return null;
            }

            if (!initialized) {
                state = new TrapezoidProfile.State(goalHeading.getRadians(), 0.0);
                lastTimestampSec = nowSec;
                initialized = true;
                return goalHeading;
            }

            double dtSec = Double.isFinite(lastTimestampSec)
                    ? MathUtil.clamp(nowSec - lastTimestampSec, 0.0, 0.1)
                    : 0.02;
            lastTimestampSec = nowSec;

            if (dtSec <= 1e-6) {
                return Rotation2d.fromRadians(MathUtil.angleModulus(state.position));
            }

            double goalRad = unwrapAngleNear(goalHeading.getRadians(), state.position);
            TrapezoidProfile profile = new TrapezoidProfile(constraints);
            state = profile.calculate(dtSec, state, new TrapezoidProfile.State(goalRad, 0.0));
            return Rotation2d.fromRadians(MathUtil.angleModulus(state.position));
        }

        double getVelocityRadPerSec() {
            return state.velocity;
        }

        private static double unwrapAngleNear(double angleRad, double referenceRad) {
            return referenceRad + MathUtil.angleModulus(angleRad - referenceRad);
        }
    }
}
