package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.util.FieldConstants;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
    private static final String LOG_COMP_ROBOT_POSE_KEY = "Shooter/Comp/RobotPose";
    private static final String LOG_COMP_TARGET_POSE_KEY = "Shooter/Comp/TargetPose";
    private static final String LOG_COMP_FAILURE_KEY = "Shooter/Comp/LastFailure";
    private static final String LOG_COMP_TIME_LOOKUP_DISTANCE_KEY = "Shooter/Comp/TimeLookupDistanceMeters";
    private static final String LOG_COMP_TIME_LOOKUP_CLAMPED_KEY = "Shooter/Comp/TimeLookupDistanceClamped";
    private static final String LOG_COMP_RAW_ROBOT_POSE_KEY = "Shooter/Comp/RawRobotPose";
    private static final String LOG_COMP_HEADING_PREDICTED_POSE_KEY = "Shooter/Comp/HeadingPredictedRobotPose";
    private static final String LOG_COMP_DISTANCE_PREDICTED_POSE_KEY = "Shooter/Comp/DistancePredictedRobotPose";
    private static final String LOG_COMP_RAW_AIM_LINE_KEY = "Shooter/Comp/RawAimLine";
    private static final String LOG_COMP_HEADING_AIM_LINE_KEY = "Shooter/Comp/HeadingAimLine";
    private static final String LOG_COMP_DISTANCE_AIM_LINE_KEY = "Shooter/Comp/DistanceAimLine";
    private static final String LOG_COMP_RAW_HEADING_DEG_KEY = "Shooter/Comp/RawHubHeadingDeg";
    private static final String LOG_COMP_UNCLAMPED_HEADING_DEG_KEY = "Shooter/Comp/UnclampedCompensatedHubHeadingDeg";
    private static final String LOG_COMP_LEAD_DEG_UNCLAMPED_KEY = "Shooter/Comp/LeadAngleDegUnclamped";
    private static final String LOG_COMP_LEAD_DEG_CLAMPED_KEY = "Shooter/Comp/LeadAngleDegClamped";
    private static final String LOG_COMP_APPLIED_TIME_SEC_KEY = "Shooter/Comp/AppliedTimeInAirSec";
    private static final String LOG_COMP_CLAMPED_KEY = "Shooter/Comp/CompensationClamped";
    private static final String LOG_COMP_TOWARD_DISP_UNCLAMPED_KEY = "Shooter/Comp/TowardDisplacementMetersUnclamped";
    private static final String LOG_COMP_TOWARD_DISP_CLAMPED_KEY = "Shooter/Comp/TowardDisplacementMetersClamped";
    private static final String LOG_COMP_PERP_DISP_UNCLAMPED_KEY = "Shooter/Comp/PerpendicularDisplacementMetersUnclamped";
    private static final String LOG_COMP_PERP_DISP_CLAMPED_KEY = "Shooter/Comp/PerpendicularDisplacementMetersClamped";
    private static final String LOG_COMP_FORWARD_COMPONENT_METERS_KEY = "Shooter/Comp/ForwardComponentMeters";
    private static final String DASHBOARD_ENABLE_KEY = "Shooter/Tuning/Enabled";
    private static final String DASHBOARD_LEFT_RPM_KEY = "Shooter/Tuning/LeftRPM";
    private static final String DASHBOARD_RIGHT_RPM_KEY = "Shooter/Tuning/RightRPM";
    private static final String DASHBOARD_HOOD_DEG_KEY = "Shooter/Tuning/HoodDeg";
    private static final String DASHBOARD_FEED_KEY = "Shooter/Tuning/FeedKicker";
    private static final String DASHBOARD_KICKER_TORQUE_KEY = "Shooter/Tuning/KickerTorqueAmps";
    private static final String DASHBOARD_SYSID_ENABLE_KEY = "Shooter/SysId/Enabled";

    public record ShotSetpoint(double leftRpm, double rightRpm, double hoodAngleRad) {}
    public record ReadinessDiagnostics(
            double leftVelocityErrorRpm,
            double rightVelocityErrorRpm,
            double hoodAngleErrorRad,
            boolean leftVelocityAtSetpoint,
            boolean rightVelocityAtSetpoint,
            boolean hoodAngleAtSetpoint,
            boolean atSetpoint,
            boolean readyToFire) {}
    public record MotionCompensation(
            double rawDistanceMeters,
            double compensatedDistanceMeters,
            double timeInAirSec,
            double velocityTowardHubMps,
            double velocityPerpendicularHubMps,
            Rotation2d compensatedHeading) {}

    private static final boolean ENABLE_VERBOSE_COMPENSATION_LOGS = false;

    private enum KickerControlMode {
        OFF,
        TORQUE,
        VOLTAGE
    }

    private final ShooterIO io;
    private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
    private final SysIdRoutine sysId;
    private StringLogEntry sysIdStateLogEntry;

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
    private ReadinessDiagnostics cachedReadiness = new ReadinessDiagnostics(0, 0, 0, false, false, false, false, false);
    private boolean cachedDashboardTuningEnabled = false;
    private boolean cachedDashboardFeedKickerEnabled = false;
    private boolean cachedDashboardSysIdEnabled = false;

    public Shooter(ShooterIO io) {
        super("shooter");
        this.io = io;
        loadShotMapFromConstants();
        loadTimeInAirMapFromConstants();
        initDashboardTuningEntries();
        sysId = new SysIdRoutine(
                new SysIdRoutine.Config(
                        null,
                        Volts.of(ShooterConstants.SHOOTER_SYSID_STEP_VOLTAGE_VOLTS),
                        Seconds.of(ShooterConstants.SHOOTER_SYSID_TIMEOUT_SEC),
                        (state) -> {
                            String stateString = state.toString();
                            if (sysIdStateLogEntry == null) {
                                sysIdStateLogEntry =
                                        new StringLogEntry(DataLogManager.getLog(), "sysid-test-state-" + getName());
                            }
                            sysIdStateLogEntry.append(stateString);
                            Logger.recordOutput("Shooter/SysIdState", stateString);
                            System.out.println("[Shooter SysId] " + stateString);
                        }),
                new SysIdRoutine.Mechanism(
                        (voltage) -> runShooterCharacterization(voltage.in(Volts)),
                        (log) -> {
                            log.motor("shooter-left")
                                    .voltage(Volts.of(inputs.shooterLeftAppliedVolts))
                                    .angularPosition(Radians.of(inputs.shooterLeftPositionRad))
                                    .angularVelocity(RotationsPerSecond.of(inputs.shooterLeftVelocityRpm / 60.0));
                            log.motor("shooter-right")
                                    .voltage(Volts.of(inputs.shooterRightAppliedVolts))
                                    .angularPosition(Radians.of(inputs.shooterRightPositionRad))
                                    .angularVelocity(RotationsPerSecond.of(inputs.shooterRightVelocityRpm / 60.0));
                        },
                        this));
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Shooter", inputs);

        if (DriverStation.isDisabled()) {
            setIdleTargets();
            stopKicker();
            io.stop();
        } else {
            applyKickerOutput();
        }

        Logger.recordOutput("Shooter/TargetLeftRpm", targetLeftRpm);
        Logger.recordOutput("Shooter/TargetRightRpm", targetRightRpm);
        Logger.recordOutput("Shooter/TargetHoodDeg", Units.radiansToDegrees(targetHoodAngleRad));
        Logger.recordOutput("Shooter/KickerTorqueAmps", kickerControlMode == KickerControlMode.TORQUE ? kickerOutput : 0.0);
        Logger.recordOutput("Shooter/KickerVoltage", kickerControlMode == KickerControlMode.VOLTAGE ? kickerOutput : 0.0);
        cachedReadiness = computeReadinessDiagnostics();
        logReadinessOutputs(cachedReadiness);
        // Cache SmartDashboard reads once per cycle to avoid repeated NT lookups
        cachedDashboardTuningEnabled = SmartDashboard.getBoolean(DASHBOARD_ENABLE_KEY, false);
        cachedDashboardFeedKickerEnabled = SmartDashboard.getBoolean(DASHBOARD_FEED_KEY, false);
        cachedDashboardSysIdEnabled = SmartDashboard.getBoolean(DASHBOARD_SYSID_ENABLE_KEY, false);
        Logger.recordOutput("Shooter/TuningEnabled", cachedDashboardTuningEnabled);
        Logger.recordOutput("Shooter/SysIdEnabled", cachedDashboardSysIdEnabled);
    }

    private static void logReadinessOutputs(ReadinessDiagnostics readiness) {
        Logger.recordOutput("Shooter/Readiness/LeftVelocityErrorRpm", readiness.leftVelocityErrorRpm());
        Logger.recordOutput("Shooter/Readiness/RightVelocityErrorRpm", readiness.rightVelocityErrorRpm());
        Logger.recordOutput("Shooter/Readiness/HoodAngleErrorDeg", Units.radiansToDegrees(readiness.hoodAngleErrorRad()));
        Logger.recordOutput("Shooter/Readiness/LeftVelocityAtSetpoint", readiness.leftVelocityAtSetpoint());
        Logger.recordOutput("Shooter/Readiness/RightVelocityAtSetpoint", readiness.rightVelocityAtSetpoint());
        Logger.recordOutput("Shooter/Readiness/HoodAngleAtSetpoint", readiness.hoodAngleAtSetpoint());
        Logger.recordOutput("Shooter/AtSetpoint", readiness.atSetpoint());
        Logger.recordOutput("Shooter/ReadyToFire", readiness.readyToFire());
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
        io.setLeftVelocity(targetLeftRpm);
        targetRightRpm = MathUtil.clamp(rightRpm, -ShooterConstants.SHOOTER_MAX_RPM, ShooterConstants.SHOOTER_MAX_RPM);
        io.setRightVelocity(targetRightRpm);
        targetHoodAngleRad = MathUtil.clamp(hoodAngleRad, ShooterConstants.HOOD_MIN_ANGLE_RAD, ShooterConstants.HOOD_MAX_ANGLE_RAD);
        io.setHoodAngle(targetHoodAngleRad);
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
        kickerOutput = MathUtil.clamp(torqueAmps,
                -ShooterConstants.KICKER_MAX_TORQUE_CURRENT_AMPS,
                ShooterConstants.KICKER_MAX_TORQUE_CURRENT_AMPS);
        kickerControlMode = KickerControlMode.TORQUE;
    }

    public void setKickerVoltage(double voltage) {
        kickerOutput = MathUtil.clamp(voltage,
                -ShooterConstants.MAX_OUTPUT_VOLTS,
                ShooterConstants.MAX_OUTPUT_VOLTS);
        kickerControlMode = KickerControlMode.VOLTAGE;
    }

    public void stopKicker() {
        kickerControlMode = KickerControlMode.OFF;
        kickerOutput = 0.0;
    }

    public void stopAll() {
        setIdleTargets();
        stopKicker();
        io.stop();
    }

    private void setIdleTargets() {
        targetLeftRpm = 0.0;
        targetRightRpm = 0.0;
        targetHoodAngleRad = ShooterConstants.HOOD_MIN_ANGLE_RAD;
    }

    public boolean atSetpoint() {
        return cachedReadiness.atSetpoint();
    }

    public boolean readyToFire() {
        return cachedReadiness.readyToFire();
    }

    /** Returns the cached readiness diagnostics computed in periodic(). */
    public ReadinessDiagnostics getReadinessDiagnostics() {
        return cachedReadiness;
    }

    /**
     * Returns readiness diagnostics computed immediately from current inputs/targets.
     * Use this when command logic changes targets and needs same-cycle gate decisions.
     */
    public ReadinessDiagnostics getReadinessDiagnosticsNow() {
        return computeReadinessDiagnostics();
    }

    private ReadinessDiagnostics computeReadinessDiagnostics() {
        double leftVelocityErrorRpm = targetLeftRpm - inputs.shooterLeftVelocityRpm;
        double rightVelocityErrorRpm = targetRightRpm - inputs.shooterRightVelocityRpm;
        double hoodAngleErrorRad = targetHoodAngleRad - inputs.hoodPositionRad;

        boolean leftVelocityAtSetpoint =
                Math.abs(leftVelocityErrorRpm) <= ShooterConstants.SHOOTER_RPM_TOLERANCE;
        boolean rightVelocityAtSetpoint =
                Math.abs(rightVelocityErrorRpm) <= ShooterConstants.SHOOTER_RPM_TOLERANCE;
        boolean hoodAngleAtSetpoint =
                Math.abs(hoodAngleErrorRad) <= ShooterConstants.HOOD_ANGLE_TOLERANCE_RAD;

        boolean atSetpoint = leftVelocityAtSetpoint && rightVelocityAtSetpoint && hoodAngleAtSetpoint;
        boolean readyToFire = atSetpoint && Math.abs(targetLeftRpm) > 1.0 && Math.abs(targetRightRpm) > 1.0;

        return new ReadinessDiagnostics(
                leftVelocityErrorRpm,
                rightVelocityErrorRpm,
                hoodAngleErrorRad,
                leftVelocityAtSetpoint,
                rightVelocityAtSetpoint,
                hoodAngleAtSetpoint,
                atSetpoint,
                readyToFire);
    }

    public boolean isKickerActive() {
        return kickerControlMode != KickerControlMode.OFF && Math.abs(kickerOutput) > 1e-6;
    }

    public double getTargetAverageShooterRpm() {
        return (targetLeftRpm + targetRightRpm) / 2.0;
    }

    public double getMeasuredAverageShooterRpm() {
        return (inputs.shooterLeftVelocityRpm + inputs.shooterRightVelocityRpm) / 2.0;
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

    /**
     * Maximum number of iterations for the time↔distance convergence loop.
     * The predicted distance determines the flight time, which determines the predicted distance.
     * 3 iterations is sufficient — typically converges in 2.
     */
    private static final int MOTION_COMP_ITERATIONS = 3;
    private static final double MOTION_COMP_TIME_CONVERGENCE_SEC = 0.001;

    public MotionCompensation getMotionCompensationToHub(Pose2d robotPose, ChassisSpeeds robotRelativeSpeeds) {
        Translation2d hubTarget = FieldConstants.getHubTargetTranslation();
        if (ENABLE_VERBOSE_COMPENSATION_LOGS) {
            Logger.recordOutput(LOG_COMP_FAILURE_KEY, "");
        }
        if (hubTarget == null || robotPose == null || robotRelativeSpeeds == null) {
            if (ENABLE_VERBOSE_COMPENSATION_LOGS) {
                publishCompensationTargetInvalid();
                clearCompensationDiagnosticsOutputs();
            }
            return publishMotionCompensation(
                    Double.NaN,
                    Double.NaN,
                    Double.NaN,
                    Double.NaN,
                    Double.NaN,
                    null);
        }

        double rawDistanceMeters = getHubDistanceMeters(robotPose);
        Translation2d toHubVector = hubTarget.minus(robotPose.getTranslation());
        double rangeMeters = toHubVector.getNorm();
        if (!Double.isFinite(rawDistanceMeters) || !Double.isFinite(rangeMeters) || rangeMeters <= 1e-6) {
            if (ENABLE_VERBOSE_COMPENSATION_LOGS) {
                publishCompensationTargetInvalid();
                clearCompensationDiagnosticsOutputs();
            }
            return publishMotionCompensation(
                    rawDistanceMeters,
                    rawDistanceMeters,
                    0.0,
                    0.0,
                    0.0,
                    null);
        }

        // Transform robot velocity to shooter velocity to account for angular velocity effects.
        // The ball inherits the shooter's velocity at launch, not the robot center's velocity.
        // v_shooter = v_center + omega x r (2D cross product: omega_z x (x,y) = (-omega*y, omega*x))
        Translation2d shooterOffset = ShooterConstants.ROBOT_TO_SHOOTER_OFFSET;
        double shooterVxRobot = robotRelativeSpeeds.vxMetersPerSecond
                - robotRelativeSpeeds.omegaRadiansPerSecond * shooterOffset.getY();
        double shooterVyRobot = robotRelativeSpeeds.vyMetersPerSecond
                + robotRelativeSpeeds.omegaRadiansPerSecond * shooterOffset.getX();
        ChassisSpeeds shooterFieldSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(
                new ChassisSpeeds(shooterVxRobot, shooterVyRobot, robotRelativeSpeeds.omegaRadiansPerSecond),
                robotPose.getRotation());
        Translation2d shooterFieldVelocity = new Translation2d(
                shooterFieldSpeeds.vxMetersPerSecond, shooterFieldSpeeds.vyMetersPerSecond);

        // Velocity decomposition for diagnostics
        Translation2d towardUnit = new Translation2d(toHubVector.getX() / rangeMeters, toHubVector.getY() / rangeMeters);
        Translation2d perpendicularUnit = new Translation2d(-towardUnit.getY(), towardUnit.getX());
        double velocityTowardHubMetersPerSec =
                shooterFieldVelocity.getX() * towardUnit.getX() + shooterFieldVelocity.getY() * towardUnit.getY();
        double velocityPerpendicularHubMetersPerSec =
                shooterFieldVelocity.getX() * perpendicularUnit.getX()
                        + shooterFieldVelocity.getY() * perpendicularUnit.getY();

        // Iterative "aim from predicted pose" model.
        // The ball inherits the shooter's velocity at launch, so we must aim as if shooting from
        // the position the robot will occupy when the ball arrives at the target.
        // Time-in-air depends on the compensated distance (which determines shot parameters),
        // and compensated distance depends on the prediction time — iterate to converge.
        Translation2d rawRobotTranslation = robotPose.getTranslation();
        Rotation2d rawRobotHeading = robotPose.getRotation();

        // Apply phase delay to compensate for system latency (sensors, network, actuators).
        // Project the starting position forward so the iterative loop starts from where
        // the robot will be when the command actually takes effect.
        Translation2d phaseDelayedTranslation = rawRobotTranslation.plus(
                shooterFieldVelocity.times(ShooterConstants.PHASE_DELAY_SEC));

        double timeInAirSec = lookupTimeInAir(rawDistanceMeters);
        double effectiveTimeSec = timeInAirSec * ShooterConstants.MOTION_COMP_TIME_SCALE;
        Translation2d predictedRobotTranslation = phaseDelayedTranslation;
        Translation2d compensatedVector = toHubVector;
        double compensatedDistanceMeters = rawDistanceMeters;

        for (int i = 0; i < MOTION_COMP_ITERATIONS; i++) {
            predictedRobotTranslation = phaseDelayedTranslation.plus(shooterFieldVelocity.times(effectiveTimeSec));
            compensatedVector = hubTarget.minus(predictedRobotTranslation);
            compensatedDistanceMeters = compensatedVector.getNorm();
            if (!Double.isFinite(compensatedDistanceMeters) || compensatedDistanceMeters < 1e-6) {
                compensatedDistanceMeters = rawDistanceMeters;
                compensatedVector = toHubVector;
                break;
            }
            double newTimeInAirSec = lookupTimeInAir(compensatedDistanceMeters);
            if (!Double.isFinite(newTimeInAirSec) || newTimeInAirSec < 0.0) {
                break;
            }
            double newEffectiveTimeSec = newTimeInAirSec * ShooterConstants.MOTION_COMP_TIME_SCALE;
            if (Math.abs(newEffectiveTimeSec - effectiveTimeSec) < MOTION_COMP_TIME_CONVERGENCE_SEC) {
                timeInAirSec = newTimeInAirSec;
                effectiveTimeSec = newEffectiveTimeSec;
                break;
            }
            timeInAirSec = newTimeInAirSec;
            effectiveTimeSec = newEffectiveTimeSec;
        }

        // Final prediction with converged time — single pose for both heading and distance
        predictedRobotTranslation = phaseDelayedTranslation.plus(shooterFieldVelocity.times(effectiveTimeSec));
        compensatedVector = hubTarget.minus(predictedRobotTranslation);
        compensatedDistanceMeters = compensatedVector.getNorm();
        if (!Double.isFinite(compensatedDistanceMeters) || compensatedDistanceMeters < 1e-6) {
            compensatedDistanceMeters = rawDistanceMeters;
            compensatedVector = toHubVector;
        }

        Rotation2d rawHeading = toHubVector.getAngle();
        Rotation2d compensatedHeading =
                compensatedVector.getNorm() > 1e-6 ? compensatedVector.getAngle() : rawHeading;

        // NOTE: Shooter offset from robot center is accounted for in the shot map calibration —
        // the shot map RPM/hood values are tuned so that the ball, launched from the shooter,
        // crosses z=1.83m at the correct horizontal distance.

        if (ENABLE_VERBOSE_COMPENSATION_LOGS) {
            Rotation2d predictedRobotHeading = new Rotation2d(
                    rawRobotHeading.getRadians() + robotRelativeSpeeds.omegaRadiansPerSecond * effectiveTimeSec);
            double unclampedLeadRad = MathUtil.angleModulus(compensatedHeading.minus(rawHeading).getRadians());
            double towardDisplacementMeters = velocityTowardHubMetersPerSec * effectiveTimeSec;
            double perpendicularDisplacementMeters = velocityPerpendicularHubMetersPerSec * effectiveTimeSec;
            Logger.recordOutput(LOG_COMP_RAW_HEADING_DEG_KEY, rawHeading.getDegrees());
            Logger.recordOutput(LOG_COMP_UNCLAMPED_HEADING_DEG_KEY, compensatedHeading.getDegrees());
            Logger.recordOutput(LOG_COMP_LEAD_DEG_UNCLAMPED_KEY, Units.radiansToDegrees(unclampedLeadRad));
            Logger.recordOutput(LOG_COMP_LEAD_DEG_CLAMPED_KEY, Units.radiansToDegrees(unclampedLeadRad));
            Logger.recordOutput(LOG_COMP_APPLIED_TIME_SEC_KEY, effectiveTimeSec);
            Logger.recordOutput(LOG_COMP_CLAMPED_KEY, false);
            Logger.recordOutput(LOG_COMP_TOWARD_DISP_UNCLAMPED_KEY, towardDisplacementMeters);
            Logger.recordOutput(LOG_COMP_TOWARD_DISP_CLAMPED_KEY, towardDisplacementMeters);
            Logger.recordOutput(LOG_COMP_PERP_DISP_UNCLAMPED_KEY, perpendicularDisplacementMeters);
            Logger.recordOutput(LOG_COMP_PERP_DISP_CLAMPED_KEY, perpendicularDisplacementMeters);
            Logger.recordOutput(LOG_COMP_FORWARD_COMPONENT_METERS_KEY, rangeMeters - towardDisplacementMeters);
            publishCompensationVisualization(
                    rawRobotTranslation,
                    rawRobotHeading,
                    predictedRobotTranslation,
                    predictedRobotHeading,
                    predictedRobotTranslation,
                    predictedRobotHeading,
                    hubTarget,
                    true);
        }

        // Publish shooter pose for AdvantageKit visualization (verify offset is correct)
        Translation2d shooterFieldPos = rawRobotTranslation.plus(
                shooterOffset.rotateBy(robotPose.getRotation()));
        Logger.recordOutput("Shooter/ShooterPose3d", new Pose3d(
                shooterFieldPos.getX(),
                shooterFieldPos.getY(),
                ShooterConstants.SHOOTER_HEIGHT_METERS,
                new Rotation3d(0.0, 0.0, robotPose.getRotation().getRadians())));

        return publishMotionCompensation(
                rawDistanceMeters,
                compensatedDistanceMeters,
                timeInAirSec,
                velocityTowardHubMetersPerSec,
                velocityPerpendicularHubMetersPerSec,
                compensatedHeading);
    }

    private double lookupTimeInAir(double distanceMeters) {
        double clamped = clampDistanceToShotMap(distanceMeters);
        double time = timeInAirSecondsByDistance.get(clamped);
        return Double.isFinite(time) && time >= 0.0 ? time : 0.0;
    }

    private MotionCompensation publishMotionCompensation(
            double rawDistanceMeters,
            double compensatedDistanceMeters,
            double timeInAirSec,
            double velocityTowardHubMps,
            double velocityPerpendicularHubMps,
            Rotation2d compensatedHeading) {
        Logger.recordOutput("Shooter/RawHubDistanceMeters", rawDistanceMeters);
        Logger.recordOutput("Shooter/CompensatedHubDistanceMeters", compensatedDistanceMeters);
        Logger.recordOutput("Shooter/VelocityTowardHubMps", velocityTowardHubMps);
        Logger.recordOutput("Shooter/VelocityPerpendicularHubMps", velocityPerpendicularHubMps);
        Logger.recordOutput("Shooter/TimeInAirSec", timeInAirSec);
        Logger.recordOutput("Shooter/CompensatedHubHeadingDeg",
                compensatedHeading != null ? compensatedHeading.getDegrees() : Double.NaN);

        return new MotionCompensation(
                rawDistanceMeters,
                compensatedDistanceMeters,
                timeInAirSec,
                velocityTowardHubMps,
                velocityPerpendicularHubMps,
                compensatedHeading);
    }

    private double clampDistanceToShotMap(double distanceMeters) {
        if (!Double.isFinite(distanceMeters)) {
            return distanceMeters;
        }
        if (!Double.isFinite(shotMapMinDistanceMeters) || !Double.isFinite(shotMapMaxDistanceMeters)) {
            return distanceMeters;
        }
        return MathUtil.clamp(distanceMeters, shotMapMinDistanceMeters, shotMapMaxDistanceMeters);
    }

    private void publishCompensationVisualization(
            Translation2d rawRobot,
            Rotation2d rawRobotHeading,
            Translation2d headingPredictedRobot,
            Rotation2d headingPredictedRobotHeading,
            Translation2d distancePredictedRobot,
            Rotation2d distancePredictedRobotHeading,
            Translation2d targetTranslation,
            boolean hasValidTarget) {
        Pose3d rawRobotPose = toPose3d(rawRobot, rawRobotHeading);
        Pose3d headingPredictedPose = toPose3d(headingPredictedRobot, headingPredictedRobotHeading);
        Pose3d distancePredictedPose = toPose3d(distancePredictedRobot, distancePredictedRobotHeading);
        Pose3d targetPose = targetTranslation != null
                ? new Pose3d(targetTranslation.getX(), targetTranslation.getY(), 0.0, new Rotation3d())
                : new Pose3d();

        Logger.recordOutput(LOG_COMP_RAW_ROBOT_POSE_KEY, rawRobotPose);
        Logger.recordOutput(LOG_COMP_HEADING_PREDICTED_POSE_KEY, headingPredictedPose);
        Logger.recordOutput(LOG_COMP_DISTANCE_PREDICTED_POSE_KEY, distancePredictedPose);
        Logger.recordOutput(LOG_COMP_ROBOT_POSE_KEY, headingPredictedPose); // legacy key for dashboards
        Logger.recordOutput(LOG_COMP_TARGET_POSE_KEY, targetPose);
        Logger.recordOutput(LOG_COMP_RAW_AIM_LINE_KEY, createAimLine(rawRobot, targetTranslation));
        Logger.recordOutput(LOG_COMP_HEADING_AIM_LINE_KEY, createAimLine(headingPredictedRobot, targetTranslation));
        Logger.recordOutput(LOG_COMP_DISTANCE_AIM_LINE_KEY, createAimLine(distancePredictedRobot, targetTranslation));
        Logger.recordOutput("Shooter/Comp/TargetValid", hasValidTarget);
    }

    private void publishCompensationTargetInvalid() {
        Logger.recordOutput(LOG_COMP_RAW_ROBOT_POSE_KEY, new Pose3d());
        Logger.recordOutput(LOG_COMP_HEADING_PREDICTED_POSE_KEY, new Pose3d());
        Logger.recordOutput(LOG_COMP_DISTANCE_PREDICTED_POSE_KEY, new Pose3d());
        Logger.recordOutput(LOG_COMP_ROBOT_POSE_KEY, new Pose3d());
        Logger.recordOutput(LOG_COMP_TARGET_POSE_KEY, new Pose3d());
        Logger.recordOutput(LOG_COMP_RAW_AIM_LINE_KEY, new Pose3d[0]);
        Logger.recordOutput(LOG_COMP_HEADING_AIM_LINE_KEY, new Pose3d[0]);
        Logger.recordOutput(LOG_COMP_DISTANCE_AIM_LINE_KEY, new Pose3d[0]);
        Logger.recordOutput("Shooter/Comp/TargetValid", false);
    }

    private static Pose3d toPose3d(Translation2d translation, Rotation2d heading) {
        return new Pose3d(
                translation.getX(),
                translation.getY(),
                0.0,
                new Rotation3d(0.0, 0.0, heading.getRadians()));
    }

    private static Pose3d[] createAimLine(Translation2d fromTranslation, Translation2d targetTranslation) {
        if (fromTranslation == null || targetTranslation == null) {
            return new Pose3d[0];
        }
        return new Pose3d[] {
                new Pose3d(fromTranslation.getX(), fromTranslation.getY(), 0.05, new Rotation3d()),
                new Pose3d(targetTranslation.getX(), targetTranslation.getY(), 0.05, new Rotation3d())
        };
    }

    private static void clearCompensationDiagnosticsOutputs() {
        Logger.recordOutput(LOG_COMP_TIME_LOOKUP_DISTANCE_KEY, Double.NaN);
        Logger.recordOutput(LOG_COMP_TIME_LOOKUP_CLAMPED_KEY, false);
        Logger.recordOutput(LOG_COMP_RAW_HEADING_DEG_KEY, Double.NaN);
        Logger.recordOutput(LOG_COMP_UNCLAMPED_HEADING_DEG_KEY, Double.NaN);
        Logger.recordOutput(LOG_COMP_LEAD_DEG_UNCLAMPED_KEY, Double.NaN);
        Logger.recordOutput(LOG_COMP_LEAD_DEG_CLAMPED_KEY, Double.NaN);
        Logger.recordOutput(LOG_COMP_APPLIED_TIME_SEC_KEY, Double.NaN);
        Logger.recordOutput(LOG_COMP_CLAMPED_KEY, false);
        Logger.recordOutput(LOG_COMP_TOWARD_DISP_UNCLAMPED_KEY, Double.NaN);
        Logger.recordOutput(LOG_COMP_TOWARD_DISP_CLAMPED_KEY, Double.NaN);
        Logger.recordOutput(LOG_COMP_PERP_DISP_UNCLAMPED_KEY, Double.NaN);
        Logger.recordOutput(LOG_COMP_PERP_DISP_CLAMPED_KEY, Double.NaN);
        Logger.recordOutput(LOG_COMP_FORWARD_COMPONENT_METERS_KEY, Double.NaN);
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
                    if (isDashboardFeedKickerEnabled()) {
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

    public Command aimForDistance(DoubleSupplier distanceMetersSupplier) {
        return runCommandWithCleanup(
                () -> setAimTargets(distanceMetersSupplier.getAsDouble()),
                this::stopAll,
                "ShooterAimForDistance");
    }

    private void setAimTargets(double distanceMeters) {
        if (isDashboardTuningEnabled()) {
            setTargets(getDashboardShotSetpoint());
        } else {
            setTargetsForDistance(distanceMeters);
        }
    }

    public Command shoot(
            DoubleSupplier distanceMetersSupplier, DoubleSupplier kickerTorqueAmpsSupplier) {
        return runCommandWithCleanup(
                        () -> {
                            setTargetsForDistance(distanceMetersSupplier.getAsDouble());
                            if (atSetpoint()) {
                                setKickerTorqueAmps(kickerTorqueAmpsSupplier.getAsDouble());
                            } else {
                                stopKicker();
                            }
                        },
                        this::stopAll,
                        "ShooterShoot");
    }

    public Command slowShooterMotorsCommand() {
        return Commands.run(
                () -> {
                    setTargets(
                            ShooterConstants.SLOW_SHOOTER_RPM,
                            ShooterConstants.SLOW_SHOOTER_RPM,
                            ShooterConstants.HOOD_MIN_ANGLE_RAD);
                    stopKicker();
                },
                this)
                .withName("ShooterSlowShooterMotors");
    }

    public Command backgroundCommand() {
        return slowShooterMotorsCommand().withName("ShooterBackground");
    }

    public Command shoot(DoubleSupplier distanceMetersSupplier) {
        return shoot(distanceMetersSupplier, () -> ShooterConstants.DEFAULT_KICKER_TORQUE_AMPS)
                .withName("ShooterShootDefaultFeed");
    }

    public Command stopCommand() {
        return Commands.runOnce(this::stopAll, this).withName("ShooterStop");
    }

    private void runShooterCharacterization(double volts) {
        double clampedVolts = MathUtil.clamp(volts, -ShooterConstants.MAX_OUTPUT_VOLTS, ShooterConstants.MAX_OUTPUT_VOLTS);
        setIdleTargets();
        stopKicker();
        io.setHoodAngle(targetHoodAngleRad);
        io.setLeftVoltage(clampedVolts);
        io.setRightVoltage(clampedVolts);
    }

    private Command gateSysIdOnDashboardEnable(Command sysIdCommand, String commandName) {
        return Commands.either(
                        sysIdCommand,
                        Commands.runOnce(
                                () -> DriverStation.reportWarning(
                                        "Shooter SysId blocked. Set " + DASHBOARD_SYSID_ENABLE_KEY + " = true to run.",
                                        false)),
                        this::isDashboardSysIdEnabled)
                .withName(commandName);
    }

    /** Returns a command to run a shooter quasistatic SysId test in the specified direction. */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        String directionLabel = direction == SysIdRoutine.Direction.kForward ? "Forward" : "Reverse";
        return gateSysIdOnDashboardEnable(
                Commands.run(() -> runShooterCharacterization(0.0), this)
                        .withTimeout(0.5)
                        .andThen(sysId.quasistatic(direction))
                        .finallyDo((interrupted) -> stopAll())
                        .withName("ShooterSysIdQuasistatic" + directionLabel),
                "ShooterSysIdQuasistatic" + directionLabel + "Gated");
    }

    /** Returns a command to run a shooter dynamic SysId test in the specified direction. */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        String directionLabel = direction == SysIdRoutine.Direction.kForward ? "Forward" : "Reverse";
        return gateSysIdOnDashboardEnable(
                Commands.run(() -> runShooterCharacterization(0.0), this)
                        .withTimeout(0.5)
                        .andThen(sysId.dynamic(direction))
                        .finallyDo((interrupted) -> stopAll())
                        .withName("ShooterSysIdDynamic" + directionLabel),
                "ShooterSysIdDynamic" + directionLabel + "Gated");
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
        return cachedDashboardTuningEnabled;
    }

    public boolean isDashboardFeedKickerEnabled() {
        return cachedDashboardFeedKickerEnabled;
    }

    public boolean isDashboardSysIdEnabled() {
        return cachedDashboardSysIdEnabled;
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
        SmartDashboard.setDefaultBoolean(DASHBOARD_SYSID_ENABLE_KEY, false);
    }

    public static double getHubDistanceMeters(Pose2d robotPose) {
        Translation2d hubTarget = FieldConstants.getHubTargetTranslation();
        if (hubTarget == null || robotPose == null) {
            return Double.NaN;
        }
        return robotPose.getTranslation().getDistance(hubTarget);
    }

    public Command homeCommand() {
        BooleanSupplier atHomingStop =
                () -> Math.abs(inputs.hoodStatorCurrentAmps) > ShooterConstants.HOMING_CURRENT_THRESHOLD_AMPS;
        return Commands.sequence(
            Commands.runOnce(() -> io.setHoodVoltage(homingVoltage()), this),
            Commands.waitUntil(atHomingStop)
                    .withTimeout(homingWaitTimeoutSec())
                    .withName("HoodHomeWaitUntil"),
            Commands.runOnce(() -> io.stop(), this),
            Commands.either(
                    Commands.sequence(
                    Commands.runOnce(() -> io.resetHoodEncoder(), this),
                    Commands.runOnce(() -> io.setHoodAngle(initialHoodPreAimAngleRad()), this)),
                Commands.runOnce(
                        () -> {
                            DriverStation.reportWarning(
                                    "Hood homing timed out before current threshold; skipping encoder reset/retract.",
                                    false);
                        },
                        this),
                atHomingStop))
                .withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming)
                .finallyDo(interrupted -> io.stop())
                .withName("HoodHome");
    }

    private static double homingVoltage() {
        return RobotBase.isSimulation() ? -12.0 : -2.0;
    }

    private static double homingWaitTimeoutSec() {
        return RobotBase.isReal() ? ShooterConstants.HOMING_WAIT_TIMEOUT_SEC : 1.0;
    }

    private static double initialHoodPreAimAngleRad() {
        if (ShooterConstants.SHOT_MAP_HOOD_ANGLE_DEG.length == 0) {
            return ShooterConstants.HOOD_MIN_ANGLE_RAD;
        }
        double preAimRad = Units.degreesToRadians(ShooterConstants.SHOT_MAP_HOOD_ANGLE_DEG[0]);
        return MathUtil.clamp(preAimRad, ShooterConstants.HOOD_MIN_ANGLE_RAD, ShooterConstants.HOOD_MAX_ANGLE_RAD);
    }

}
