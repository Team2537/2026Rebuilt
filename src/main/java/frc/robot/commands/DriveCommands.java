// Copyright (c) 2021-2025 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.FieldConstants;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public final class DriveCommands {
    private static final double DEADBAND = 0.1;
    private static final Rotation2d HUB_AUTO_ALIGN_HEADING_OFFSET = Rotation2d.kPi;
    private static final double HEADING_SNAP_TOLERANCE_RAD = Math.toRadians(1.2);
    private static final double WHEEL_RADIUS_MAX_VELOCITY = 1.0; // Rad/Sec
    private static final double WHEEL_RADIUS_RAMP_RATE = 0.50; // Rad/Sec^2

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

    private static Rotation2d getAllianceAdjustedFieldHeading() {
        Rotation2d rotation = RobotState.getInstance().getRotation();
        boolean isFlipped = DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == Alliance.Red;
        return isFlipped ? rotation.plus(Rotation2d.kPi) : rotation;
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

                    ChassisSpeeds commandSpeeds = drive.isFieldOriented()
                            ? ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getAllianceAdjustedFieldHeading())
                            : speeds;

                    drive.runDriverVelocity(commandSpeeds);
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

    /** Returns a command that snaps heading to the nearest cardinal direction. */
    public static Command headingSnap(Drive drive) {
        HubAlignController alignController = new HubAlignController();
        HeadingSnapState state = new HeadingSnapState();

        return Commands.run(
                () -> {
                    double omega = alignController.calculate(
                            RobotState.getInstance().getRotation().getRadians(),
                            state.targetHeading,
                            0.0);
                    drive.runDriverVelocity(new ChassisSpeeds(0.0, 0.0, omega));
                },
                drive)
                .beforeStarting(() -> {
                    Rotation2d snappedHeading = snapToNearestCardinal(RobotState.getInstance().getRotation());
                    state.targetHeading = snappedHeading;
                    alignController.reset(RobotState.getInstance().getRotation().getRadians(), snappedHeading);
                })
                .until(() -> Math.abs(MathUtil.angleModulus(
                        state.targetHeading.minus(RobotState.getInstance().getRotation()).getRadians()))
                        <= HEADING_SNAP_TOLERANCE_RAD)
                .andThen(Commands.runOnce(drive::stop, drive))
                .withName("DriveHeadingSnap");
    }

    /** Measures the robot wheel radius by spinning in place and integrating gyro angle. */
    public static Command wheelRadiusCharacterization(Drive drive) {
        SlewRateLimiter limiter = new SlewRateLimiter(WHEEL_RADIUS_RAMP_RATE);
        WheelRadiusCharacterizationState state = new WheelRadiusCharacterizationState();

        return Commands.parallel(
                Commands.sequence(
                        Commands.runOnce(() -> limiter.reset(0.0)),
                        Commands.run(
                                () -> {
                                    double speed = limiter.calculate(WHEEL_RADIUS_MAX_VELOCITY);
                                    drive.runVelocity(new ChassisSpeeds(0.0, 0.0, speed));
                                },
                                drive)),
                Commands.sequence(
                        Commands.waitSeconds(1.0),
                        Commands.runOnce(() -> {
                            state.positions = drive.getWheelRadiusCharacterizationPositions();
                            state.lastAngle = drive.getRotation();
                            state.gyroDelta = 0.0;
                        }),
                        Commands.run(() -> {
                            Rotation2d rotation = drive.getRotation();
                            state.gyroDelta += Math.abs(rotation.minus(state.lastAngle).getRadians());
                            state.lastAngle = rotation;
                        }).finallyDo(() -> {
                            double[] positions = drive.getWheelRadiusCharacterizationPositions();
                            double wheelDelta = 0.0;
                            for (int i = 0; i < 4; i++) {
                                wheelDelta += Math.abs(positions[i] - state.positions[i]) / 4.0;
                            }

                            double wheelRadius = (state.gyroDelta * Drive.DRIVE_BASE_RADIUS) / wheelDelta;
                            NumberFormat formatter = new DecimalFormat("#0.000");
                            System.out.println("********** Wheel Radius Characterization Results **********");
                            System.out.println("\tWheel Delta: " + formatter.format(wheelDelta) + " radians");
                            System.out.println("\tGyro Delta: " + formatter.format(state.gyroDelta) + " radians");
                            System.out.println(
                                    "\tWheel Radius: "
                                            + formatter.format(wheelRadius)
                                            + " meters, "
                                            + formatter.format(Units.metersToInches(wheelRadius))
                                            + " inches");
                        })))
                .finallyDo(() -> drive.stop())
                .withName("DriveWheelRadiusCharacterization");
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
                () -> FieldConstants.getHubFacingHeading(RobotState.getInstance().getPose()));
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
        return autoAlignToHubPose(
                drive,
                xSupplier,
                ySupplier,
                omegaFallbackSupplier,
                headingSupplier,
                () -> false);
    }

    /**
     * Drive command that keeps translational joystick control and auto-rotates to
     * a supplied field heading.
     * Optionally turns the modules to an X arrangement when the robot is idle and
     * the supplied lock condition is true.
     */
    public static Command autoAlignToHubPose(
            Drive drive,
            DoubleSupplier xSupplier,
            DoubleSupplier ySupplier,
            DoubleSupplier omegaFallbackSupplier,
            Supplier<Rotation2d> headingSupplier,
            BooleanSupplier xLockConditionSupplier) {
        HubAlignController alignController = new HubAlignController();

        return Commands.run(
                () -> {
                    Translation2d linearVelocity = getLinearVelocityFromJoysticks(
                            -xSupplier.getAsDouble(), -ySupplier.getAsDouble());

                    Rotation2d targetHeading = applyHubAutoAlignHeadingOffset(headingSupplier.get());

                    double fallbackOmegaInput = MathUtil.applyDeadband(
                            omegaFallbackSupplier.getAsDouble(), DEADBAND);
                    double fallbackOmega = Math.copySign(
                            fallbackOmegaInput * fallbackOmegaInput,
                            fallbackOmegaInput);
                    fallbackOmega *= drive.getMaxAngularSpeedRadPerSec();

                    double omega = alignController.calculate(
                            RobotState.getInstance().getRotation().getRadians(), targetHeading, fallbackOmega);

                    boolean noDriverInput = linearVelocity.getNorm() <= 1e-6
                            && Math.abs(fallbackOmegaInput) <= 1e-6;
                    if (noDriverInput && xLockConditionSupplier.getAsBoolean()) {
                        drive.stopWithX();
                        return;
                    }

                    ChassisSpeeds speeds = new ChassisSpeeds(
                            linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                            linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                            omega);

                    ChassisSpeeds commandSpeeds = drive.isFieldOriented()
                            ? ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getAllianceAdjustedFieldHeading())
                            : speeds;

                    drive.runDriverVelocity(commandSpeeds);
                },
                drive)
                .beforeStarting(() -> {
                    Rotation2d initialTarget = applyHubAutoAlignHeadingOffset(headingSupplier.get());
                    alignController.reset(RobotState.getInstance().getRotation().getRadians(), initialTarget);
                })
                .withName("DriveAutoAlignToHubPose");
    }

    private static Rotation2d applyHubAutoAlignHeadingOffset(Rotation2d heading) {
        return heading == null ? null : heading.plus(HUB_AUTO_ALIGN_HEADING_OFFSET);
    }

    private static Rotation2d snapToNearestCardinal(Rotation2d heading) {
        double cardinalStepRad = Math.PI / 2.0;
        double snappedRad = Math.round(heading.getRadians() / cardinalStepRad) * cardinalStepRad;
        return Rotation2d.fromRadians(MathUtil.angleModulus(snappedRad));
    }

    private static class WheelRadiusCharacterizationState {
        double[] positions = new double[4];
        Rotation2d lastAngle = Rotation2d.kZero;
        double gyroDelta = 0.0;
    }

    private static class HeadingSnapState {
        Rotation2d targetHeading = Rotation2d.kZero;
    }

}
