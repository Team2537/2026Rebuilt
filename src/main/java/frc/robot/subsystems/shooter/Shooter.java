package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.util.FieldConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
    public record ShotSetpoint(double leftRpm, double rightRpm, double hoodAngleRad) {}

    private enum KickerControlMode {
        OFF,
        TORQUE,
        VOLTAGE
    }

    private final ShooterIO io;
    private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

    private final InterpolatingDoubleTreeMap leftRpmByDistance = new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap rightRpmByDistance = new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap hoodAngleRadByDistance = new InterpolatingDoubleTreeMap();

    private double targetLeftRpm = 0.0;
    private double targetRightRpm = 0.0;
    private double targetHoodAngleRad = ShooterConstants.HOOD_MIN_ANGLE_RAD;
    private double kickerOutput = 0.0;
    private KickerControlMode kickerControlMode = KickerControlMode.OFF;

    public Shooter(ShooterIO io) {
        super("shooter");
        this.io = io;
        loadShotMapFromConstants();
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Shooter", inputs);

        if (DriverStation.isDisabled()) {
            io.stop();
            stopKicker();
        } else {
            applyShooterTargets();
            applyKickerOutput();
        }

        Logger.recordOutput("Shooter/TargetLeftRpm", targetLeftRpm);
        Logger.recordOutput("Shooter/TargetRightRpm", targetRightRpm);
        Logger.recordOutput("Shooter/TargetHoodDeg", Units.radiansToDegrees(targetHoodAngleRad));
        Logger.recordOutput("Shooter/KickerTorqueAmps", kickerControlMode == KickerControlMode.TORQUE ? kickerOutput : 0.0);
        Logger.recordOutput("Shooter/KickerVoltage", kickerControlMode == KickerControlMode.VOLTAGE ? kickerOutput : 0.0);
        Logger.recordOutput("Shooter/AtSetpoint", atSetpoint());
        Logger.recordOutput("Shooter/ReadyToFire", readyToFire());
    }

    private void applyShooterTargets() {
        io.setLeftVelocity(targetLeftRpm);
        io.setRightVelocity(targetRightRpm);
        io.setHoodAngle(targetHoodAngleRad);
    }

    private void applyKickerOutput() {
        switch (kickerControlMode) {
            case TORQUE -> io.setKickerTorque(kickerOutput);
            case VOLTAGE -> io.setKickerVoltage(kickerOutput);
            case OFF -> io.setKickerTorque(0.0);
        }
    }

    public void setTargets(ShotSetpoint setpoint) {
        setTargets(setpoint.leftRpm(), setpoint.rightRpm(), setpoint.hoodAngleRad());
    }

    public void setTargets(double leftRpm, double rightRpm, double hoodAngleRad) {
        targetLeftRpm = MathUtil.clamp(leftRpm, -ShooterConstants.SHOOTER_MAX_RPM, ShooterConstants.SHOOTER_MAX_RPM);
        targetRightRpm = MathUtil.clamp(rightRpm, -ShooterConstants.SHOOTER_MAX_RPM, ShooterConstants.SHOOTER_MAX_RPM);
        targetHoodAngleRad = MathUtil.clamp(
                hoodAngleRad,
                ShooterConstants.HOOD_MIN_ANGLE_RAD,
                ShooterConstants.HOOD_MAX_ANGLE_RAD);
    }

    public void setTargetsForDistance(double distanceMeters) {
        if (!Double.isFinite(distanceMeters)) {
            return;
        }
        setTargets(calculateSetpointForDistance(distanceMeters));
    }

    public ShotSetpoint calculateSetpointForDistance(double distanceMeters) {
        return new ShotSetpoint(
                leftRpmByDistance.get(distanceMeters),
                rightRpmByDistance.get(distanceMeters),
                hoodAngleRadByDistance.get(distanceMeters));
    }

    public void setKickerTorqueAmps(double torqueAmps) {
        kickerOutput = MathUtil.clamp(
                torqueAmps,
                -ShooterConstants.KICKER_MAX_TORQUE_CURRENT_AMPS,
                ShooterConstants.KICKER_MAX_TORQUE_CURRENT_AMPS);
        kickerControlMode = KickerControlMode.TORQUE;
    }

    public void setKickerVoltage(double voltage) {
        kickerOutput = MathUtil.clamp(voltage, -ShooterConstants.MAX_OUTPUT_VOLTS, ShooterConstants.MAX_OUTPUT_VOLTS);
        kickerControlMode = KickerControlMode.VOLTAGE;
    }

    public void stopKicker() {
        kickerControlMode = KickerControlMode.OFF;
        kickerOutput = 0.0;
    }

    public void stopAll() {
        setTargets(0.0, 0.0, ShooterConstants.HOOD_MIN_ANGLE_RAD);
        stopKicker();
    }

    public boolean atSetpoint() {
        return Math.abs(inputs.shooterLeftVelocityRpm - targetLeftRpm) <= ShooterConstants.SHOOTER_RPM_TOLERANCE
                && Math.abs(inputs.shooterRightVelocityRpm - targetRightRpm) <= ShooterConstants.SHOOTER_RPM_TOLERANCE
                && Math.abs(inputs.hoodPositionRad - targetHoodAngleRad) <= ShooterConstants.HOOD_ANGLE_TOLERANCE_RAD;
    }

    public boolean readyToFire() {
        return atSetpoint() && (Math.abs(targetLeftRpm) > 1.0 || Math.abs(targetRightRpm) > 1.0);
    }

    private Command runCommandWithCleanup(Runnable action, Runnable cleanup, String name) {
        return Commands.run(action, this).finallyDo(cleanup).withName(name);
    }

    private Command runTargetingCommand(Runnable action, String name) {
        return runCommandWithCleanup(action, this::stopAll, name);
    }

    public Command aimForDistance(DoubleSupplier distanceMetersSupplier) {
        return runTargetingCommand(
                () -> setTargetsForDistance(distanceMetersSupplier.getAsDouble()),
                "ShooterAimForDistance");
    }

    public Command shoot(
            DoubleSupplier distanceMetersSupplier, DoubleSupplier kickerTorqueAmpsSupplier) {
        return runCommandWithCleanup(
                        () -> {
                            setTargetsForDistance(distanceMetersSupplier.getAsDouble());
                            if (readyToFire()) {
                                setKickerTorqueAmps(kickerTorqueAmpsSupplier.getAsDouble());
                            } else {
                                stopKicker();
                            }
                        },
                        this::stopAll,
                        "ShooterShoot");
    }

    public Command shoot(DoubleSupplier distanceMetersSupplier) {
        return shoot(distanceMetersSupplier, () -> ShooterConstants.DEFAULT_KICKER_TORQUE_AMPS)
                .withName("ShooterShootDefaultFeed");
    }

    public Command runSetpoint(
            DoubleSupplier leftRpmSupplier, DoubleSupplier rightRpmSupplier, DoubleSupplier hoodAngleRadSupplier) {
        return runTargetingCommand(
                () -> setTargets(
                        leftRpmSupplier.getAsDouble(),
                        rightRpmSupplier.getAsDouble(),
                        hoodAngleRadSupplier.getAsDouble()),
                "ShooterRunSetpoint");
    }

    public Command runKickerTorque(DoubleSupplier kickerTorqueAmpsSupplier) {
        return runCommandWithCleanup(
                () -> setKickerTorqueAmps(kickerTorqueAmpsSupplier.getAsDouble()),
                this::stopKicker,
                "ShooterRunKickerTorque");
    }

    public Command runKickerVoltage(DoubleSupplier kickerVoltageSupplier) {
        return runCommandWithCleanup(
                () -> setKickerVoltage(kickerVoltageSupplier.getAsDouble()),
                this::stopKicker,
                "ShooterRunKickerVoltage");
    }

    public Command stopCommand() {
        return Commands.runOnce(this::stopAll, this).withName("ShooterStop");
    }

    public Command idleCommand() {
        return Commands.run(this::stopAll, this).withName("ShooterIdle");
    }

    public void setShotMapPoint(double distanceMeters, double leftRpm, double rightRpm, double hoodAngleDeg) {
        leftRpmByDistance.put(distanceMeters, leftRpm);
        rightRpmByDistance.put(distanceMeters, rightRpm);
        hoodAngleRadByDistance.put(distanceMeters, Units.degreesToRadians(hoodAngleDeg));
    }

    public void setShotMap(
            double[] distancesMeters, double[] leftRpm, double[] rightRpm, double[] hoodAngleDeg) {
        if (distancesMeters.length < 2) {
            throw new IllegalArgumentException("Shot map requires at least 2 points.");
        }
        if (distancesMeters.length != leftRpm.length
                || distancesMeters.length != rightRpm.length
                || distancesMeters.length != hoodAngleDeg.length) {
            throw new IllegalArgumentException("Shot map arrays must have equal length.");
        }

        leftRpmByDistance.clear();
        rightRpmByDistance.clear();
        hoodAngleRadByDistance.clear();

        for (int i = 0; i < distancesMeters.length; i++) {
            setShotMapPoint(distancesMeters[i], leftRpm[i], rightRpm[i], hoodAngleDeg[i]);
        }
    }

    private void loadShotMapFromConstants() {
        setShotMap(
                ShooterConstants.SHOT_MAP_DISTANCE_METERS,
                ShooterConstants.SHOT_MAP_LEFT_RPM,
                ShooterConstants.SHOT_MAP_RIGHT_RPM,
                ShooterConstants.SHOT_MAP_HOOD_ANGLE_DEG);
    }

    public static DoubleSupplier hubDistanceMetersSupplier(Supplier<Pose2d> robotPoseSupplier) {
        return () -> getHubDistanceMeters(robotPoseSupplier.get());
    }

    public static double getHubDistanceMeters(Pose2d robotPose) {
        return FieldConstants.TAG_LAYOUT
                .getTagPose(ShooterConstants.HUB_TAG_ID)
                .map(tagPose -> {
                    Translation2d target = new Translation2d(
                            tagPose.getX() + ShooterConstants.HUB_TARGET_X_OFFSET_METERS,
                            tagPose.getY());
                    return robotPose.getTranslation().getDistance(target);
                })
                .orElse(Double.NaN);
    }
}
