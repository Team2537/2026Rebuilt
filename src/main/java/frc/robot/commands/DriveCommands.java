// Copyright (c) 2021-2025 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.FieldConstants;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public final class DriveCommands {
    private static final double DEADBAND = 0.1;
    private static final Rotation2d HUB_AUTO_ALIGN_HEADING_OFFSET = Rotation2d.kPi;

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

    private static Rotation2d getAllianceAdjustedFieldHeading(Drive drive) {
        boolean isFlipped = DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == Alliance.Red;
        return isFlipped ? drive.getRotation().plus(Rotation2d.kPi) : drive.getRotation();
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
                            ? ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getAllianceAdjustedFieldHeading(drive))
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
                () -> FieldConstants.getHubFacingHeading(drive.getPose()));
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
        HubAlignController alignController = new HubAlignController();

        return Commands.run(
                () -> {
                    Translation2d linearVelocity = getLinearVelocityFromJoysticks(
                            -xSupplier.getAsDouble(), -ySupplier.getAsDouble());

                    Rotation2d targetHeading = applyHubAutoAlignHeadingOffset(headingSupplier.get());

                    double fallbackOmega = MathUtil.applyDeadband(
                            omegaFallbackSupplier.getAsDouble(), DEADBAND);
                    fallbackOmega = Math.copySign(fallbackOmega * fallbackOmega, fallbackOmega);
                    fallbackOmega *= drive.getMaxAngularSpeedRadPerSec();

                    double omega = alignController.calculate(
                            drive.getRotation().getRadians(), targetHeading, fallbackOmega);

                    ChassisSpeeds speeds = new ChassisSpeeds(
                            linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                            linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                            omega);

                    ChassisSpeeds commandSpeeds = drive.isFieldOriented()
                            ? ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getAllianceAdjustedFieldHeading(drive))
                            : speeds;

                    drive.runVelocity(commandSpeeds);
                },
                drive)
                .beforeStarting(() -> {
                    Rotation2d initialTarget = applyHubAutoAlignHeadingOffset(headingSupplier.get());
                    alignController.reset(drive.getRotation().getRadians(), initialTarget);
                })
                .withName("DriveAutoAlignToHubPose");
    }

    private static Rotation2d applyHubAutoAlignHeadingOffset(Rotation2d heading) {
        return heading == null ? null : heading.plus(HUB_AUTO_ALIGN_HEADING_OFFSET);
    }

}
