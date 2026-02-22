// Copyright (c) 2021-2025 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.FieldConstants;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.LinkedList;
import java.util.List;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class DriveCommands {
    private static final double DEADBAND = 0.1;
    private static final double ANGLE_KP = 5.0;
    private static final double ANGLE_KD = 0.4;
    private static final double ANGLE_MAX_VELOCITY = 8.0;
    private static final double ANGLE_MAX_ACCELERATION = 20.0;
    private static final double HUB_ALIGN_KP = 5.0;
    private static final double HUB_ALIGN_KD = 0.4;
    private static final double HUB_ALIGN_MAX_VELOCITY = 8.0;
    private static final double HUB_ALIGN_MAX_ACCELERATION = 20.0;
    private static final double HUB_POSE_ALIGN_KP = 2.4;
    private static final double HUB_POSE_ALIGN_KD = 0.16;
    private static final double HUB_POSE_ALIGN_MAX_VELOCITY = 5.2;
    private static final double HUB_POSE_ALIGN_MAX_ACCELERATION = 14.0;
    private static final double HUB_POSE_ALIGN_TOLERANCE_RAD = Units.degreesToRadians(1.25);
    private static final double HUB_POSE_ALIGN_HOLD_RELEASE_TOLERANCE_RAD = Units.degreesToRadians(1.7);
    private static final double HUB_POSE_TARGET_FILTER_ALPHA = 0.2;
    private static final double HUB_POSE_TARGET_FILTER_MAX_STEP_RAD = Units.degreesToRadians(4.0);
    private static final double HUB_POSE_MAX_OMEGA_RAD_PER_SEC = 4.5;
    private static final double HUB_POSE_OMEGA_SLEW_RATE_RAD_PER_SEC_SQ = 36.0;
    private static final double HUB_POSE_TARGET_RATE_FILTER_ALPHA = 0.35;
    private static final double HUB_POSE_TARGET_RATE_MAX_RAD_PER_SEC = 6.0;
    private static final double HUB_POSE_TARGET_RATE_FF = 0.9;
    private static final double HUB_POSE_MOVING_TARGET_RATE_THRESHOLD_RAD_PER_SEC = 0.35;
    private static final double HUB_POSE_TARGET_RATE_DT_MIN_SEC = 1e-3;
    private static final double HUB_POSE_TARGET_RATE_DT_MAX_SEC = 0.1;
    private static final double FF_START_DELAY = 2.0; // Secs
    private static final double FF_RAMP_RATE = 0.1; // Volts/Sec
    private static final double WHEEL_RADIUS_MAX_VELOCITY = 0.25; // Rad/Sec
    private static final double WHEEL_RADIUS_RAMP_RATE = 0.05; // Rad/Sec^2

    private DriveCommands() {
    }

    private static Translation2d getLinearVelocityFromJoysticks(double x, double y) {
        // Apply deadband
        double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), DEADBAND);
        Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));

        // Square magnitude for more precise control
        linearMagnitude = linearMagnitude * linearMagnitude;

        // Return new linear velocity
        return new Pose2d(Translation2d.kZero, linearDirection)
                .transformBy(new Transform2d(linearMagnitude, 0.0, Rotation2d.kZero))
                .getTranslation();
    }

    /**
     * Field relative drive command using two joysticks (controlling linear and
     * angular velocities).
     */
    public static Command joystickDrive(
            Drive drive,
            DoubleSupplier xSupplier,
            DoubleSupplier ySupplier,
            DoubleSupplier omegaSupplier) {
        return Commands.run(
                () -> {
                    // Get linear velocity
                    Translation2d linearVelocity = getLinearVelocityFromJoysticks(-xSupplier.getAsDouble(),
                            -ySupplier.getAsDouble());

                    // Apply rotation deadband
                    double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);

                    // Square rotation value for more precise control
                    omega = Math.copySign(omega * omega, omega);

                    // Compute desired chassis speeds from joystick input
                    ChassisSpeeds speeds = new ChassisSpeeds(
                            linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                            linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                            omega * drive.getMaxAngularSpeedRadPerSec());

                    boolean isFlipped = DriverStation.getAlliance().isPresent()
                            && DriverStation.getAlliance().get() == Alliance.Red;

                    Rotation2d heading = isFlipped
                            ? drive.getRotation().plus(new Rotation2d(Math.PI))
                            : drive.getRotation();

                    ChassisSpeeds commandSpeeds = drive.isFieldOriented()
                            ? ChassisSpeeds.fromFieldRelativeSpeeds(speeds, heading)
                            : speeds;

                    drive.runVelocity(commandSpeeds);
                },
                drive);
    }

    /** Returns a command that toggles field-oriented vs robot-oriented driving. */
    public static Command toggleFieldOriented(Drive drive) {
        return Commands.runOnce(drive::toggleFieldOriented, drive);
    }

    /** Returns a command that resets odometry and sets heading to 0 radians. */
    public static Command resetOdometryAndHeading(Drive drive) {
        return Commands.runOnce(drive::resetOdometryAndHeadingToZero, drive);
    }

    /**
     * Field relative drive command using joystick for linear control and PID for
     * angular control.
     * Possible use cases include snapping to an angle, aiming at a vision target,
     * or controlling
     * absolute rotation with a joystick.
     */
    public static Command joystickDriveAtAngle(
            Drive drive,
            DoubleSupplier xSupplier,
            DoubleSupplier ySupplier,
            Supplier<Rotation2d> rotationSupplier) {

        // Create PID controller
        ProfiledPIDController angleController = new ProfiledPIDController(
                ANGLE_KP,
                0.0,
                ANGLE_KD,
                new TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION));
        angleController.enableContinuousInput(-Math.PI, Math.PI);

        // Construct command
        return Commands.run(
                () -> {
                    // Get linear velocity
                    Translation2d linearVelocity = getLinearVelocityFromJoysticks(xSupplier.getAsDouble(),
                            ySupplier.getAsDouble());

                    // Calculate angular speed
                    double omega = angleController.calculate(
                            drive.getRotation().getRadians(), rotationSupplier.get().getRadians());

                    // Convert to field relative speeds & send command
                    ChassisSpeeds speeds = new ChassisSpeeds(
                            linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                            linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                            omega);
                    boolean isFlipped = DriverStation.getAlliance().isPresent()
                            && DriverStation.getAlliance().get() == Alliance.Red;
                    drive.runVelocity(
                            ChassisSpeeds.fromFieldRelativeSpeeds(
                                    speeds,
                                    isFlipped
                                            ? drive.getRotation().plus(new Rotation2d(Math.PI))
                                            : drive.getRotation()));
                },
                drive)

                // Reset PID controller when command starts
                .beforeStarting(() -> angleController.reset(drive.getRotation().getRadians()));
    }

    /**
     * Drive command that keeps translational joystick control and auto-rotates to
     * face the hub using only observed hub AprilTag transforms.
     */
    public static Command autoAlignToHub(
            Drive drive,
            Vision vision,
            DoubleSupplier xSupplier,
            DoubleSupplier ySupplier,
            DoubleSupplier omegaFallbackSupplier) {
        ProfiledPIDController hubYawController = new ProfiledPIDController(
                HUB_ALIGN_KP,
                0.0,
                HUB_ALIGN_KD,
                new TrapezoidProfile.Constraints(HUB_ALIGN_MAX_VELOCITY, HUB_ALIGN_MAX_ACCELERATION));
        hubYawController.enableContinuousInput(-Math.PI, Math.PI);

        return Commands.run(
                () -> {
                    Translation2d linearVelocity = getLinearVelocityFromJoysticks(-xSupplier.getAsDouble(),
                            -ySupplier.getAsDouble());

                    double hubYawRad = vision.getHubYawRad();
                    double omega;
                    if (Double.isFinite(hubYawRad)) {
                        omega = hubYawController.calculate(-hubYawRad, 0.0);
                    } else {
                        double fallbackOmega = MathUtil.applyDeadband(omegaFallbackSupplier.getAsDouble(), DEADBAND);
                        fallbackOmega = Math.copySign(fallbackOmega * fallbackOmega, fallbackOmega);
                        omega = fallbackOmega * drive.getMaxAngularSpeedRadPerSec();
                        hubYawController.reset(0.0);
                    }

                    ChassisSpeeds speeds = new ChassisSpeeds(
                            linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                            linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                            omega);

                    boolean isFlipped = DriverStation.getAlliance().isPresent()
                            && DriverStation.getAlliance().get() == Alliance.Red;
                    Rotation2d heading = isFlipped
                            ? drive.getRotation().plus(new Rotation2d(Math.PI))
                            : drive.getRotation();

                    ChassisSpeeds commandSpeeds = drive.isFieldOriented()
                            ? ChassisSpeeds.fromFieldRelativeSpeeds(speeds, heading)
                            : speeds;

                    drive.runVelocity(commandSpeeds);
                },
                drive).beforeStarting(() -> {
                    double initialHubYaw = vision.getHubYawRad();
                    hubYawController.reset(Double.isFinite(initialHubYaw) ? -initialHubYaw : 0.0);
                })
                .withName("DriveAutoAlignToHub");
    }

    /**
     * Drive command that keeps translational joystick control and auto-rotates to
     * face the hub using odometry + field layout.
     */
    public static Command autoAlignToHubPose(
            Drive drive,
            DoubleSupplier xSupplier,
            DoubleSupplier ySupplier,
            DoubleSupplier omegaFallbackSupplier) {
        return autoAlignToHubPose(
                drive,
                xSupplier,
                ySupplier,
                omegaFallbackSupplier,
                () -> getHubFacingHeading(drive.getPose()));
    }

    /**
     * Drive command that keeps translational joystick control and auto-rotates to
     * a supplied field heading.
     */
    public static Command autoAlignToHubPose(
            Drive drive,
            DoubleSupplier xSupplier,
            DoubleSupplier ySupplier,
            DoubleSupplier omegaFallbackSupplier,
            Supplier<Rotation2d> headingSupplier) {
        HubPoseAlignState state = new HubPoseAlignState();
        SlewRateLimiter omegaLimiter = new SlewRateLimiter(HUB_POSE_OMEGA_SLEW_RATE_RAD_PER_SEC_SQ);

        ProfiledPIDController angleController = new ProfiledPIDController(
                HUB_POSE_ALIGN_KP,
                0.0,
                HUB_POSE_ALIGN_KD,
                new TrapezoidProfile.Constraints(HUB_POSE_ALIGN_MAX_VELOCITY, HUB_POSE_ALIGN_MAX_ACCELERATION));
        angleController.enableContinuousInput(-Math.PI, Math.PI);
        angleController.setTolerance(HUB_POSE_ALIGN_TOLERANCE_RAD);

        return Commands.run(
                () -> {
                    Translation2d linearVelocity = getLinearVelocityFromJoysticks(-xSupplier.getAsDouble(),
                            -ySupplier.getAsDouble());

                    Rotation2d targetHeading = headingSupplier.get();
                    double omega;
                    if (targetHeading != null) {
                        if (state.filteredTargetHeading == null) {
                            state.initializeTargetTracking(targetHeading, Timer.getFPGATimestamp());
                            angleController.reset(drive.getRotation().getRadians());
                        } else {
                            state.filteredTargetHeading = filterTargetHeading(
                                    state.filteredTargetHeading,
                                    targetHeading);
                        }

                        double targetHeadingRateRadPerSec = updateTargetHeadingRateRadPerSec(
                                state,
                                Timer.getFPGATimestamp());

                        double currentHeadingRad = drive.getRotation().getRadians();
                        double filteredTargetHeadingRad = state.filteredTargetHeading.getRadians();
                        double headingErrorRad = MathUtil.angleModulus(filteredTargetHeadingRad - currentHeadingRad);
                        boolean targetMoving = Math.abs(targetHeadingRateRadPerSec)
                                >= HUB_POSE_MOVING_TARGET_RATE_THRESHOLD_RAD_PER_SEC;

                        if (state.holdAtTarget
                                && !targetMoving
                                && Math.abs(headingErrorRad) <= HUB_POSE_ALIGN_HOLD_RELEASE_TOLERANCE_RAD) {
                            omega = 0.0;
                            angleController.reset(currentHeadingRad);
                        } else if (!targetMoving && Math.abs(headingErrorRad) <= HUB_POSE_ALIGN_TOLERANCE_RAD) {
                            state.holdAtTarget = true;
                            omega = 0.0;
                            angleController.reset(currentHeadingRad);
                        } else {
                            state.holdAtTarget = false;
                            double feedbackOmega = angleController.calculate(currentHeadingRad, filteredTargetHeadingRad);
                            double feedforwardOmega = targetHeadingRateRadPerSec * HUB_POSE_TARGET_RATE_FF;
                            omega = MathUtil.clamp(
                                    feedbackOmega + feedforwardOmega,
                                    -HUB_POSE_MAX_OMEGA_RAD_PER_SEC,
                                    HUB_POSE_MAX_OMEGA_RAD_PER_SEC);
                        }
                        omega = omegaLimiter.calculate(omega);
                    } else {
                        state.clearTargetTracking();
                        double fallbackOmega = MathUtil.applyDeadband(omegaFallbackSupplier.getAsDouble(), DEADBAND);
                        fallbackOmega = Math.copySign(fallbackOmega * fallbackOmega, fallbackOmega);
                        omega = fallbackOmega * drive.getMaxAngularSpeedRadPerSec();
                        omegaLimiter.reset(omega);
                        angleController.reset(drive.getRotation().getRadians());
                    }

                    ChassisSpeeds speeds = new ChassisSpeeds(
                            linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                            linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                            omega);

                    boolean isFlipped = DriverStation.getAlliance().isPresent()
                            && DriverStation.getAlliance().get() == Alliance.Red;
                    Rotation2d heading = isFlipped
                            ? drive.getRotation().plus(new Rotation2d(Math.PI))
                            : drive.getRotation();

                    ChassisSpeeds commandSpeeds = drive.isFieldOriented()
                            ? ChassisSpeeds.fromFieldRelativeSpeeds(speeds, heading)
                            : speeds;

                    drive.runVelocity(commandSpeeds);
                },
                drive)
                .beforeStarting(() -> {
                    Rotation2d initialTargetHeading = headingSupplier.get();
                    if (initialTargetHeading != null) {
                        state.initializeTargetTracking(initialTargetHeading, Timer.getFPGATimestamp());
                    } else {
                        state.clearTargetTracking();
                    }
                    omegaLimiter.reset(0.0);
                    angleController.reset(drive.getRotation().getRadians());
                })
                .withName("DriveAutoAlignToHubPose");
    }

    private static Rotation2d filterTargetHeading(Rotation2d filteredTargetHeading, Rotation2d targetHeading) {
        double targetDelta = MathUtil.angleModulus(targetHeading.getRadians() - filteredTargetHeading.getRadians());
        double filteredDeltaStep = MathUtil.clamp(
                targetDelta * HUB_POSE_TARGET_FILTER_ALPHA,
                -HUB_POSE_TARGET_FILTER_MAX_STEP_RAD,
                HUB_POSE_TARGET_FILTER_MAX_STEP_RAD);
        return new Rotation2d(filteredTargetHeading.getRadians() + filteredDeltaStep);
    }

    private static double updateTargetHeadingRateRadPerSec(HubPoseAlignState state, double nowSec) {
        if (state.lastFilteredTargetHeading == null || !Double.isFinite(state.lastFilteredTargetHeadingTimestampSec)) {
            state.lastFilteredTargetHeading = state.filteredTargetHeading;
            state.lastFilteredTargetHeadingTimestampSec = nowSec;
            state.filteredTargetHeadingRateRadPerSec = 0.0;
            return 0.0;
        }

        double dtSec = MathUtil.clamp(
                nowSec - state.lastFilteredTargetHeadingTimestampSec,
                HUB_POSE_TARGET_RATE_DT_MIN_SEC,
                HUB_POSE_TARGET_RATE_DT_MAX_SEC);
        double targetHeadingDelta = MathUtil.angleModulus(
                state.filteredTargetHeading.getRadians()
                        - state.lastFilteredTargetHeading.getRadians());
        double rawTargetHeadingRateRadPerSec = MathUtil.clamp(
                targetHeadingDelta / dtSec,
                -HUB_POSE_TARGET_RATE_MAX_RAD_PER_SEC,
                HUB_POSE_TARGET_RATE_MAX_RAD_PER_SEC);
        state.filteredTargetHeadingRateRadPerSec += HUB_POSE_TARGET_RATE_FILTER_ALPHA
                * (rawTargetHeadingRateRadPerSec - state.filteredTargetHeadingRateRadPerSec);
        state.lastFilteredTargetHeading = state.filteredTargetHeading;
        state.lastFilteredTargetHeadingTimestampSec = nowSec;
        return state.filteredTargetHeadingRateRadPerSec;
    }

    private static Rotation2d getHubFacingHeading(Pose2d robotPose) {
        if (robotPose == null) {
            return null;
        }
        return FieldConstants.TAG_LAYOUT.getTagPose(ShooterConstants.HUB_TAG_ID)
                .map(tagPose -> {
                    Translation2d hubTarget = new Translation2d(
                            tagPose.getX() + ShooterConstants.HUB_TARGET_X_OFFSET_METERS,
                            tagPose.getY());
                    Translation2d toHub = hubTarget.minus(robotPose.getTranslation());
                    if (toHub.getNorm() <= 1e-6) {
                        return null;
                    }
                    return toHub.getAngle();
                })
                .orElse(null);
    }

    /**
     * Measures the velocity feedforward constants for the drive motors.
     *
     * <p>
     * This command should only be used in voltage control mode.
     */
    public static Command feedforwardCharacterization(Drive drive) {
        List<Double> velocitySamples = new LinkedList<>();
        List<Double> voltageSamples = new LinkedList<>();
        Timer timer = new Timer();

        return Commands.sequence(
                // Reset data
                Commands.runOnce(
                        () -> {
                            velocitySamples.clear();
                            voltageSamples.clear();
                        }),

                // Allow modules to orient
                Commands.run(
                        () -> {
                            drive.runCharacterization(0.0);
                        },
                        drive)
                        .withTimeout(FF_START_DELAY),

                // Start timer
                Commands.runOnce(timer::restart),

                // Accelerate and gather data
                Commands.run(
                        () -> {
                            double voltage = timer.get() * FF_RAMP_RATE;
                            drive.runCharacterization(voltage);
                            velocitySamples.add(drive.getFFCharacterizationVelocity());
                            voltageSamples.add(voltage);
                        },
                        drive)

                        // When cancelled, calculate and print results
                        .finallyDo(
                                () -> {
                                    int n = velocitySamples.size();
                                    double sumX = 0.0;
                                    double sumY = 0.0;
                                    double sumXY = 0.0;
                                    double sumX2 = 0.0;
                                    for (int i = 0; i < n; i++) {
                                        sumX += velocitySamples.get(i);
                                        sumY += voltageSamples.get(i);
                                        sumXY += velocitySamples.get(i) * voltageSamples.get(i);
                                        sumX2 += velocitySamples.get(i) * velocitySamples.get(i);
                                    }
                                    double kS = (sumY * sumX2 - sumX * sumXY) / (n * sumX2 - sumX * sumX);
                                    double kV = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);

                                    NumberFormat formatter = new DecimalFormat("#0.00000");
                                    System.out.println("********** Drive FF Characterization Results **********");
                                    System.out.println("\tkS: " + formatter.format(kS));
                                    System.out.println("\tkV: " + formatter.format(kV));
                                }));
    }

    /** Measures the robot's wheel radius by spinning in a circle. */
    public static Command wheelRadiusCharacterization(Drive drive) {
        SlewRateLimiter limiter = new SlewRateLimiter(WHEEL_RADIUS_RAMP_RATE);
        WheelRadiusCharacterizationState state = new WheelRadiusCharacterizationState();

        return Commands.parallel(
                // Drive control sequence
                Commands.sequence(
                        // Reset acceleration limiter
                        Commands.runOnce(
                                () -> {
                                    limiter.reset(0.0);
                                }),

                        // Turn in place, accelerating up to full speed
                        Commands.run(
                                () -> {
                                    double speed = limiter.calculate(WHEEL_RADIUS_MAX_VELOCITY);
                                    drive.runVelocity(new ChassisSpeeds(0.0, 0.0, speed));
                                },
                                drive)),

                // Measurement sequence
                Commands.sequence(
                        // Wait for modules to fully orient before starting measurement
                        Commands.waitSeconds(1.0),

                        // Record starting measurement
                        Commands.runOnce(
                                () -> {
                                    state.positions = drive.getWheelRadiusCharacterizationPositions();
                                    state.lastAngle = drive.getRotation();
                                    state.gyroDelta = 0.0;
                                }),

                        // Update gyro delta
                        Commands.run(
                                () -> {
                                    var rotation = drive.getRotation();
                                    state.gyroDelta += Math.abs(rotation.minus(state.lastAngle).getRadians());
                                    state.lastAngle = rotation;
                                })

                                // When cancelled, calculate and print results
                                .finallyDo(
                                        () -> {
                                            double[] positions = drive.getWheelRadiusCharacterizationPositions();
                                            double wheelDelta = 0.0;
                                            for (int i = 0; i < 4; i++) {
                                                wheelDelta += Math.abs(positions[i] - state.positions[i]) / 4.0;
                                            }
                                            double wheelRadius = (state.gyroDelta * Drive.DRIVE_BASE_RADIUS)
                                                    / wheelDelta;

                                            NumberFormat formatter = new DecimalFormat("#0.000");
                                            System.out.println(
                                                    "********** Wheel Radius Characterization Results **********");
                                            System.out.println(
                                                    "\tWheel Delta: " + formatter.format(wheelDelta) + " radians");
                                            System.out.println(
                                                    "\tGyro Delta: " + formatter.format(state.gyroDelta) + " radians");
                                            System.out.println(
                                                    "\tWheel Radius: "
                                                            + formatter.format(wheelRadius)
                                                            + " meters, "
                                                            + formatter.format(Units.metersToInches(wheelRadius))
                                                            + " inches");
                                        })));
    }

    private static class HubPoseAlignState {
        Rotation2d filteredTargetHeading = null;
        boolean holdAtTarget = false;
        Rotation2d lastFilteredTargetHeading = null;
        double lastFilteredTargetHeadingTimestampSec = Double.NaN;
        double filteredTargetHeadingRateRadPerSec = 0.0;

        void initializeTargetTracking(Rotation2d initialHeading, double nowSec) {
            filteredTargetHeading = initialHeading;
            holdAtTarget = false;
            lastFilteredTargetHeading = initialHeading;
            lastFilteredTargetHeadingTimestampSec = nowSec;
            filteredTargetHeadingRateRadPerSec = 0.0;
        }

        void clearTargetTracking() {
            filteredTargetHeading = null;
            holdAtTarget = false;
            lastFilteredTargetHeading = null;
            lastFilteredTargetHeadingTimestampSec = Double.NaN;
            filteredTargetHeadingRateRadPerSec = 0.0;
        }
    }

    private static class WheelRadiusCharacterizationState {
        double[] positions = new double[4];
        Rotation2d lastAngle = Rotation2d.kZero;
        double gyroDelta = 0.0;
    }
}
