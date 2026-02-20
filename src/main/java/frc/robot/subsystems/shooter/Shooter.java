package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.util.FieldConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
    private static final String LOG_COMP_ROBOT_POSE_KEY = "Shooter/Comp/RobotPose";
    private static final String LOG_COMP_TARGET_POSE_KEY = "Shooter/Comp/TargetPose";
    private static final String LOG_COMP_FAILURE_KEY = "Shooter/Comp/LastFailure";
    private static final String DASHBOARD_ENABLE_KEY = "Shooter/Tuning/Enabled";
    private static final String DASHBOARD_LEFT_RPM_KEY = "Shooter/Tuning/LeftRPM";
    private static final String DASHBOARD_RIGHT_RPM_KEY = "Shooter/Tuning/RightRPM";
    private static final String DASHBOARD_HOOD_DEG_KEY = "Shooter/Tuning/HoodDeg";
    private static final String DASHBOARD_FEED_KEY = "Shooter/Tuning/FeedKicker";
    private static final String DASHBOARD_KICKER_TORQUE_KEY = "Shooter/Tuning/KickerTorqueAmps";
    private static final String DASHBOARD_RAW_DISTANCE_KEY = "Shooter/Comp/RawHubDistanceMeters";
    private static final String DASHBOARD_COMP_DISTANCE_KEY = "Shooter/Comp/CompensatedDistanceMeters";
    private static final String DASHBOARD_VELOCITY_TOWARD_HUB_KEY = "Shooter/Comp/VelocityTowardHubMps";
    private static final String DASHBOARD_VELOCITY_PERP_HUB_KEY = "Shooter/Comp/VelocityPerpendicularHubMps";
    private static final String DASHBOARD_TIME_IN_AIR_KEY = "Shooter/Comp/TimeInAirSec";
    private static final String DASHBOARD_COMP_HEADING_DEG_KEY = "Shooter/Comp/CompensatedHeadingDeg";
    private static final String DASHBOARD_COMP_TARGET_X_KEY = "Shooter/Comp/TargetX";
    private static final String DASHBOARD_COMP_TARGET_Y_KEY = "Shooter/Comp/TargetY";
    private static final String DASHBOARD_COMP_TARGET_HEADING_KEY = "Shooter/Comp/TargetHeadingDeg";
    private static final String DASHBOARD_COMP_TARGET_VALID_KEY = "Shooter/Comp/TargetValid";
    private static final String DASHBOARD_MAP_DISTANCE_KEY = "Shooter/Comp/MapDistanceMeters";
    private static final String DASHBOARD_MAP_TIME_KEY = "Shooter/Comp/MapTimeInAirSec";

    public record ShotSetpoint(double leftRpm, double rightRpm, double hoodAngleRad) {}
    public record MotionCompensation(
            double rawDistanceMeters,
            double compensatedDistanceMeters,
            double timeInAirSec,
            double velocityTowardHubMps,
            double velocityPerpendicularHubMps,
            Rotation2d compensatedHeading) {}

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
    private final InterpolatingDoubleTreeMap timeInAirSecondsByDistance = new InterpolatingDoubleTreeMap();
    private double shotMapMinDistanceMeters = Double.NaN;
    private double shotMapMaxDistanceMeters = Double.NaN;

    private double targetLeftRpm = 0.0;
    private double targetRightRpm = 0.0;
    private double targetHoodAngleRad = ShooterConstants.HOOD_MIN_ANGLE_RAD;
    private double kickerOutput = 0.0;
    private KickerControlMode kickerControlMode = KickerControlMode.OFF;

    public Shooter(ShooterIO io) {
        super("shooter");
        this.io = io;
        loadShotMapFromConstants();
        loadTimeInAirMapFromConstants();
        initDashboardTuningEntries();
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
        Logger.recordOutput("Shooter/TuningEnabled", isDashboardTuningEnabled());
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
        if (setpoint == null) {
            throw new IllegalArgumentException("setpoint must not be null");
        }
        setTargets(setpoint.leftRpm(), setpoint.rightRpm(), setpoint.hoodAngleRad());
    }

    public void setTargets(double leftRpm, double rightRpm, double hoodAngleRad) {
        targetLeftRpm = requireInRange(
                leftRpm, -ShooterConstants.SHOOTER_MAX_RPM, ShooterConstants.SHOOTER_MAX_RPM, "leftRpm");
        targetRightRpm = requireInRange(
                rightRpm, -ShooterConstants.SHOOTER_MAX_RPM, ShooterConstants.SHOOTER_MAX_RPM, "rightRpm");
        targetHoodAngleRad = requireInRange(
                hoodAngleRad, ShooterConstants.HOOD_MIN_ANGLE_RAD, ShooterConstants.HOOD_MAX_ANGLE_RAD, "hoodAngleRad");
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
        kickerOutput = requireInRange(
                torqueAmps,
                -ShooterConstants.KICKER_MAX_TORQUE_CURRENT_AMPS,
                ShooterConstants.KICKER_MAX_TORQUE_CURRENT_AMPS,
                "torqueAmps");
        kickerControlMode = KickerControlMode.TORQUE;
    }

    public void setKickerVoltage(double voltage) {
        kickerOutput = requireInRange(
                voltage,
                -ShooterConstants.MAX_OUTPUT_VOLTS,
                ShooterConstants.MAX_OUTPUT_VOLTS,
                "voltage");
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

    public boolean isActivelyShooting() {
        return readyToFire() && isKickerActive();
    }

    public boolean isKickerActive() {
        return kickerControlMode != KickerControlMode.OFF && Math.abs(kickerOutput) > 1e-6;
    }

    public double getAverageShooterVelocityRpm() {
        return (inputs.shooterLeftVelocityRpm + inputs.shooterRightVelocityRpm) / 2.0;
    }

    public double getTargetAverageShooterRpm() {
        return (targetLeftRpm + targetRightRpm) / 2.0;
    }

    public double getCurrentHoodAngleRad() {
        return inputs.hoodPositionRad;
    }

    public double getTargetHoodAngleRad() {
        return targetHoodAngleRad;
    }

    public double getMotionCompensatedHubDistanceMeters(Pose2d robotPose, ChassisSpeeds robotRelativeSpeeds) {
        return getMotionCompensationToHub(robotPose, robotRelativeSpeeds).compensatedDistanceMeters();
    }

    public Rotation2d getMotionCompensatedHubHeading(Pose2d robotPose, ChassisSpeeds robotRelativeSpeeds) {
        return getMotionCompensationToHub(robotPose, robotRelativeSpeeds).compensatedHeading();
    }

    public MotionCompensation getMotionCompensationToHub(Pose2d robotPose, ChassisSpeeds robotRelativeSpeeds) {
        double rawDistanceMeters = getHubDistanceMeters(robotPose);
        Translation2d hubTarget = getHubTargetTranslation();
        Logger.recordOutput(LOG_COMP_FAILURE_KEY, "");
        if (!Double.isFinite(rawDistanceMeters) || hubTarget == null || robotPose == null || robotRelativeSpeeds == null) {
            publishCompensationTargetInvalid();
            return publishMotionCompensation(
                    rawDistanceMeters,
                    rawDistanceMeters,
                    Double.NaN,
                    Double.NaN,
                    Double.NaN,
                    null);
        }

        Translation2d toHubVector = hubTarget.minus(robotPose.getTranslation());
        double rangeMeters = toHubVector.getNorm();
        if (rangeMeters <= 1e-6) {
            publishCompensationTargetInvalid();
            return publishMotionCompensation(
                    rawDistanceMeters,
                    rawDistanceMeters,
                    0.0,
                    0.0,
                    0.0,
                    null);
        }

        ChassisSpeeds fieldRelativeSpeeds =
                ChassisSpeeds.fromRobotRelativeSpeeds(robotRelativeSpeeds, robotPose.getRotation());
        Translation2d robotFieldVelocity =
                new Translation2d(fieldRelativeSpeeds.vxMetersPerSecond, fieldRelativeSpeeds.vyMetersPerSecond);
        Translation2d towardUnit = new Translation2d(toHubVector.getX() / rangeMeters, toHubVector.getY() / rangeMeters);
        Translation2d perpendicularUnit = new Translation2d(-towardUnit.getY(), towardUnit.getX());
        double velocityTowardHubMetersPerSec =
                robotFieldVelocity.getX() * towardUnit.getX() + robotFieldVelocity.getY() * towardUnit.getY();
        double velocityPerpendicularHubMetersPerSec =
                robotFieldVelocity.getX() * perpendicularUnit.getX()
                        + robotFieldVelocity.getY() * perpendicularUnit.getY();

        if (!isDistanceWithinShotMapRange(rawDistanceMeters)) {
            reportCompensationFailure(String.format(
                    "Raw distance %.3f m out of shot map range [%.3f, %.3f]",
                    rawDistanceMeters,
                    shotMapMinDistanceMeters,
                    shotMapMaxDistanceMeters));
            publishCompensationTargetInvalid();
            return publishMotionCompensation(
                    rawDistanceMeters,
                    Double.NaN,
                    Double.NaN,
                    velocityTowardHubMetersPerSec,
                    velocityPerpendicularHubMetersPerSec,
                    null);
        }

        double firstPassTimeSec = timeInAirSecondsByDistance.get(rawDistanceMeters);
        if (!Double.isFinite(firstPassTimeSec)) {
            reportCompensationFailure(String.format(
                    "Time-in-air lookup invalid for raw distance %.3f m",
                    rawDistanceMeters));
            publishCompensationTargetInvalid();
            return publishMotionCompensation(
                    rawDistanceMeters,
                    Double.NaN,
                    Double.NaN,
                    velocityTowardHubMetersPerSec,
                    velocityPerpendicularHubMetersPerSec,
                    null);
        }
        Translation2d firstPassVector = toHubVector
                .minus(robotFieldVelocity.times(firstPassTimeSec));
        double firstPassDistance = firstPassVector.getNorm();
        if (!isDistanceWithinShotMapRange(firstPassDistance)) {
            reportCompensationFailure(String.format(
                    "First-pass distance %.3f m out of shot map range [%.3f, %.3f]",
                    firstPassDistance,
                    shotMapMinDistanceMeters,
                    shotMapMaxDistanceMeters));
            publishCompensationTargetInvalid();
            return publishMotionCompensation(
                    rawDistanceMeters,
                    Double.NaN,
                    Double.NaN,
                    velocityTowardHubMetersPerSec,
                    velocityPerpendicularHubMetersPerSec,
                    null);
        }

        double timeInAirSec = timeInAirSecondsByDistance.get(firstPassDistance);
        if (!Double.isFinite(timeInAirSec)) {
            reportCompensationFailure(String.format(
                    "Time-in-air lookup invalid for first-pass distance %.3f m",
                    firstPassDistance));
            publishCompensationTargetInvalid();
            return publishMotionCompensation(
                    rawDistanceMeters,
                    Double.NaN,
                    Double.NaN,
                    velocityTowardHubMetersPerSec,
                    velocityPerpendicularHubMetersPerSec,
                    null);
        }
        Translation2d compensatedVector = toHubVector
                .minus(robotFieldVelocity.times(timeInAirSec));
        double compensatedDistanceMeters = compensatedVector.getNorm();
        if (!isDistanceWithinShotMapRange(compensatedDistanceMeters)) {
            reportCompensationFailure(String.format(
                    "Compensated distance %.3f m out of shot map range [%.3f, %.3f]",
                    compensatedDistanceMeters,
                    shotMapMinDistanceMeters,
                    shotMapMaxDistanceMeters));
            publishCompensationTargetInvalid();
            return publishMotionCompensation(
                    rawDistanceMeters,
                    Double.NaN,
                    timeInAirSec,
                    velocityTowardHubMetersPerSec,
                    velocityPerpendicularHubMetersPerSec,
                    null);
        }

        Rotation2d compensatedHeading = compensatedVector.getNorm() > 1e-6 ? compensatedVector.getAngle() : null;
        Translation2d compensatedRobot = robotPose.getTranslation()
                .plus(robotFieldVelocity.times(timeInAirSec));
        Rotation2d compensatedRobotHeading =
                new Rotation2d(robotPose.getRotation().getRadians() + robotRelativeSpeeds.omegaRadiansPerSecond * timeInAirSec);
        boolean hasValidTarget = compensatedHeading != null;
        publishCompensationTarget(compensatedRobot, compensatedRobotHeading, hasValidTarget);

        return publishMotionCompensation(
                rawDistanceMeters,
                compensatedDistanceMeters,
                timeInAirSec,
                velocityTowardHubMetersPerSec,
                velocityPerpendicularHubMetersPerSec,
                compensatedHeading);
    }

    private MotionCompensation publishMotionCompensation(
            double rawDistanceMeters,
            double compensatedDistanceMeters,
            double timeInAirSec,
            double velocityTowardHubMps,
            double velocityPerpendicularHubMps,
            Rotation2d compensatedHeading) {
        double headingDeg = compensatedHeading != null ? compensatedHeading.getDegrees() : Double.NaN;

        Logger.recordOutput("Shooter/RawHubDistanceMeters", rawDistanceMeters);
        Logger.recordOutput("Shooter/CompensatedHubDistanceMeters", compensatedDistanceMeters);
        Logger.recordOutput("Shooter/VelocityTowardHubMps", velocityTowardHubMps);
        Logger.recordOutput("Shooter/VelocityPerpendicularHubMps", velocityPerpendicularHubMps);
        Logger.recordOutput("Shooter/TimeInAirSec", timeInAirSec);
        Logger.recordOutput("Shooter/CompensatedHubHeadingDeg", headingDeg);

        SmartDashboard.putNumber(DASHBOARD_RAW_DISTANCE_KEY, rawDistanceMeters);
        SmartDashboard.putNumber(DASHBOARD_COMP_DISTANCE_KEY, compensatedDistanceMeters);
        SmartDashboard.putNumber(DASHBOARD_VELOCITY_TOWARD_HUB_KEY, velocityTowardHubMps);
        SmartDashboard.putNumber(DASHBOARD_VELOCITY_PERP_HUB_KEY, velocityPerpendicularHubMps);
        SmartDashboard.putNumber(DASHBOARD_TIME_IN_AIR_KEY, timeInAirSec);
        SmartDashboard.putNumber(DASHBOARD_COMP_HEADING_DEG_KEY, headingDeg);

        return new MotionCompensation(
                rawDistanceMeters,
                compensatedDistanceMeters,
                timeInAirSec,
                velocityTowardHubMps,
                velocityPerpendicularHubMps,
                compensatedHeading);
    }

    private boolean isDistanceWithinShotMapRange(double distanceMeters) {
        return Double.isFinite(distanceMeters)
                && Double.isFinite(shotMapMinDistanceMeters)
                && Double.isFinite(shotMapMaxDistanceMeters)
                && distanceMeters >= shotMapMinDistanceMeters
                && distanceMeters <= shotMapMaxDistanceMeters;
    }

    private void publishCompensationTarget(Translation2d compensatedRobot, Rotation2d compensatedRobotHeading, boolean hasValidTarget) {
        Pose3d robotPose = new Pose3d(
                compensatedRobot.getX(),
                compensatedRobot.getY(),
                0.0,
                new Rotation3d(0.0, 0.0, compensatedRobotHeading.getRadians()));
        Logger.recordOutput(LOG_COMP_ROBOT_POSE_KEY, robotPose);
        // Keep legacy key for existing widgets; it mirrors the compensated robot pose.
        Logger.recordOutput(LOG_COMP_TARGET_POSE_KEY, robotPose);
        SmartDashboard.putNumber(DASHBOARD_COMP_TARGET_X_KEY, compensatedRobot.getX());
        SmartDashboard.putNumber(DASHBOARD_COMP_TARGET_Y_KEY, compensatedRobot.getY());
        SmartDashboard.putNumber(DASHBOARD_COMP_TARGET_HEADING_KEY, compensatedRobotHeading.getDegrees());
        SmartDashboard.putBoolean(DASHBOARD_COMP_TARGET_VALID_KEY, hasValidTarget);
    }

    private void publishCompensationTargetInvalid() {
        Logger.recordOutput(LOG_COMP_ROBOT_POSE_KEY, new Pose3d());
        Logger.recordOutput(LOG_COMP_TARGET_POSE_KEY, new Pose3d());
        SmartDashboard.putNumber(DASHBOARD_COMP_TARGET_X_KEY, Double.NaN);
        SmartDashboard.putNumber(DASHBOARD_COMP_TARGET_Y_KEY, Double.NaN);
        SmartDashboard.putNumber(DASHBOARD_COMP_TARGET_HEADING_KEY, Double.NaN);
        SmartDashboard.putBoolean(DASHBOARD_COMP_TARGET_VALID_KEY, false);
    }

    private void reportCompensationFailure(String reason) {
        Logger.recordOutput(LOG_COMP_FAILURE_KEY, reason);
        DriverStation.reportError("[Shooter] Compensation invalid: " + reason, false);
    }

    private static double requireInRange(double value, double minInclusive, double maxInclusive, String name) {
        if (!Double.isFinite(value)) {
            throw new IllegalArgumentException(name + " must be finite, got " + value);
        }
        if (value < minInclusive || value > maxInclusive) {
            throw new IllegalArgumentException(
                    String.format(
                            "%s out of range: %.3f not in [%.3f, %.3f]",
                            name,
                            value,
                            minInclusive,
                            maxInclusive));
        }
        return value;
    }

    public Command dashboardTuneCommand() {
        return runCommandWithCleanup(
                () -> {
                    setTargets(getDashboardShotSetpoint());
                    if (SmartDashboard.getBoolean(DASHBOARD_FEED_KEY, false)) {
                        setKickerTorqueAmps(SmartDashboard.getNumber(
                                DASHBOARD_KICKER_TORQUE_KEY,
                                ShooterConstants.DEFAULT_KICKER_TORQUE_AMPS));
                    } else {
                        stopKicker();
                    }
                },
                this::stopAll,
                "ShooterDashboardTune");
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
        if (!Double.isFinite(distanceMeters)) {
            throw new IllegalArgumentException("distanceMeters must be finite");
        }
        requireInRange(leftRpm, -ShooterConstants.SHOOTER_MAX_RPM, ShooterConstants.SHOOTER_MAX_RPM, "leftRpm");
        requireInRange(rightRpm, -ShooterConstants.SHOOTER_MAX_RPM, ShooterConstants.SHOOTER_MAX_RPM, "rightRpm");
        double hoodAngleRad = Units.degreesToRadians(hoodAngleDeg);
        requireInRange(
                hoodAngleRad,
                ShooterConstants.HOOD_MIN_ANGLE_RAD,
                ShooterConstants.HOOD_MAX_ANGLE_RAD,
                "hoodAngleRad");
        leftRpmByDistance.put(distanceMeters, leftRpm);
        rightRpmByDistance.put(distanceMeters, rightRpm);
        hoodAngleRadByDistance.put(distanceMeters, hoodAngleRad);
        if (!Double.isFinite(shotMapMinDistanceMeters) || distanceMeters < shotMapMinDistanceMeters) {
            shotMapMinDistanceMeters = distanceMeters;
        }
        if (!Double.isFinite(shotMapMaxDistanceMeters) || distanceMeters > shotMapMaxDistanceMeters) {
            shotMapMaxDistanceMeters = distanceMeters;
        }
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
        validateDistancesStrictlyIncreasing(distancesMeters, "Shot map");

        leftRpmByDistance.clear();
        rightRpmByDistance.clear();
        hoodAngleRadByDistance.clear();
        shotMapMinDistanceMeters = Double.NaN;
        shotMapMaxDistanceMeters = Double.NaN;

        for (int i = 0; i < distancesMeters.length; i++) {
            setShotMapPoint(distancesMeters[i], leftRpm[i], rightRpm[i], hoodAngleDeg[i]);
        }
    }

    public void setTimeInAirMap(double[] distancesMeters, double[] timeInAirSec) {
        if (distancesMeters.length < 2) {
            throw new IllegalArgumentException("Time-in-air map requires at least 2 points.");
        }
        if (distancesMeters.length != timeInAirSec.length) {
            throw new IllegalArgumentException("Time-in-air map arrays must have equal length.");
        }
        validateDistancesStrictlyIncreasing(distancesMeters, "Time-in-air map");

        timeInAirSecondsByDistance.clear();
        for (int i = 0; i < distancesMeters.length; i++) {
            if (!Double.isFinite(timeInAirSec[i]) || timeInAirSec[i] < 0.0) {
                throw new IllegalArgumentException(
                        String.format("Time-in-air map value at index %d must be finite and >= 0.0, got %.3f", i, timeInAirSec[i]));
            }
            timeInAirSecondsByDistance.put(distancesMeters[i], timeInAirSec[i]);
        }
    }

    private static void validateDistancesStrictlyIncreasing(double[] distancesMeters, String mapName) {
        for (int i = 0; i < distancesMeters.length; i++) {
            if (!Double.isFinite(distancesMeters[i])) {
                throw new IllegalArgumentException(
                        String.format("%s distance at index %d must be finite, got %s", mapName, i, distancesMeters[i]));
            }
            if (i > 0 && distancesMeters[i] <= distancesMeters[i - 1]) {
                throw new IllegalArgumentException(
                        String.format(
                                "%s distances must be strictly increasing: index %d value %.3f <= previous %.3f",
                                mapName,
                                i,
                                distancesMeters[i],
                                distancesMeters[i - 1]));
            }
        }
    }

    private void loadShotMapFromConstants() {
        setShotMap(
                ShooterConstants.SHOT_MAP_DISTANCE_METERS,
                ShooterConstants.SHOT_MAP_LEFT_RPM,
                ShooterConstants.SHOT_MAP_RIGHT_RPM,
                ShooterConstants.SHOT_MAP_HOOD_ANGLE_DEG);
    }

    private void loadTimeInAirMapFromConstants() {
        if (ShooterConstants.SHOT_MAP_DISTANCE_METERS.length
                != ShooterConstants.SHOT_TIME_IN_AIR_SECONDS.length) {
            throw new IllegalArgumentException("SHOT_TIME_IN_AIR_SECONDS must match SHOT_MAP_DISTANCE_METERS length.");
        }
        setTimeInAirMap(
                ShooterConstants.SHOT_MAP_DISTANCE_METERS,
                ShooterConstants.SHOT_TIME_IN_AIR_SECONDS);
    }

    public boolean isDashboardTuningEnabled() {
        return SmartDashboard.getBoolean(DASHBOARD_ENABLE_KEY, false);
    }

    private ShotSetpoint getDashboardShotSetpoint() {
        return new ShotSetpoint(
                SmartDashboard.getNumber(DASHBOARD_LEFT_RPM_KEY, ShooterConstants.SHOT_MAP_LEFT_RPM[0]),
                SmartDashboard.getNumber(DASHBOARD_RIGHT_RPM_KEY, ShooterConstants.SHOT_MAP_RIGHT_RPM[0]),
                Units.degreesToRadians(SmartDashboard.getNumber(DASHBOARD_HOOD_DEG_KEY, ShooterConstants.SHOT_MAP_HOOD_ANGLE_DEG[0])));
    }

    private static void initDashboardTuningEntries() {
        SmartDashboard.setDefaultBoolean(DASHBOARD_ENABLE_KEY, false);
        SmartDashboard.setDefaultNumber(DASHBOARD_LEFT_RPM_KEY, ShooterConstants.SHOT_MAP_LEFT_RPM[0]);
        SmartDashboard.setDefaultNumber(DASHBOARD_RIGHT_RPM_KEY, ShooterConstants.SHOT_MAP_RIGHT_RPM[0]);
        SmartDashboard.setDefaultNumber(DASHBOARD_HOOD_DEG_KEY, ShooterConstants.SHOT_MAP_HOOD_ANGLE_DEG[0]);
        SmartDashboard.setDefaultBoolean(DASHBOARD_FEED_KEY, false);
        SmartDashboard.setDefaultNumber(DASHBOARD_KICKER_TORQUE_KEY, ShooterConstants.DEFAULT_KICKER_TORQUE_AMPS);
        SmartDashboard.setDefaultNumber(DASHBOARD_RAW_DISTANCE_KEY, Double.NaN);
        SmartDashboard.setDefaultNumber(DASHBOARD_COMP_DISTANCE_KEY, Double.NaN);
        SmartDashboard.setDefaultNumber(DASHBOARD_VELOCITY_TOWARD_HUB_KEY, 0.0);
        SmartDashboard.setDefaultNumber(DASHBOARD_VELOCITY_PERP_HUB_KEY, 0.0);
        SmartDashboard.setDefaultNumber(DASHBOARD_TIME_IN_AIR_KEY, Double.NaN);
        SmartDashboard.setDefaultNumber(DASHBOARD_COMP_HEADING_DEG_KEY, Double.NaN);
        SmartDashboard.setDefaultNumber(DASHBOARD_COMP_TARGET_X_KEY, Double.NaN);
        SmartDashboard.setDefaultNumber(DASHBOARD_COMP_TARGET_Y_KEY, Double.NaN);
        SmartDashboard.setDefaultNumber(DASHBOARD_COMP_TARGET_HEADING_KEY, Double.NaN);
        SmartDashboard.setDefaultBoolean(DASHBOARD_COMP_TARGET_VALID_KEY, false);
        SmartDashboard.putNumberArray(DASHBOARD_MAP_DISTANCE_KEY, ShooterConstants.SHOT_MAP_DISTANCE_METERS);
        SmartDashboard.putNumberArray(DASHBOARD_MAP_TIME_KEY, ShooterConstants.SHOT_TIME_IN_AIR_SECONDS);
    }

    public static DoubleSupplier hubDistanceMetersSupplier(Supplier<Pose2d> robotPoseSupplier) {
        return () -> getHubDistanceMeters(robotPoseSupplier.get());
    }

    public static double getHubDistanceMeters(Pose2d robotPose) {
        Translation2d hubTarget = getHubTargetTranslation();
        if (hubTarget == null || robotPose == null) {
            return Double.NaN;
        }
        return robotPose.getTranslation().getDistance(hubTarget);
    }

    private static Translation2d getHubTargetTranslation() {
        return FieldConstants.TAG_LAYOUT
                .getTagPose(ShooterConstants.HUB_TAG_ID)
                .map(tagPose -> new Translation2d(
                        tagPose.getX() + ShooterConstants.HUB_TARGET_X_OFFSET_METERS,
                        tagPose.getY()))
                .orElse(null);
    }
}
