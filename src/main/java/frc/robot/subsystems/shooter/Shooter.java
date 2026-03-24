package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.util.ElasticNotifications;
import java.util.Objects;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
    private static final double AT_SETPOINT_FALLING_DEBOUNCE_SECONDS = 0.2;
    private static final String DASHBOARD_ENABLE_KEY = "Shooter/Tuning/Enabled";
    private static final String DASHBOARD_LEFT_RPM_KEY = "Shooter/Tuning/LeftRPM";
    private static final String DASHBOARD_RIGHT_RPM_KEY = "Shooter/Tuning/RightRPM";
    private static final String DASHBOARD_HOOD_DEG_KEY = "Shooter/Tuning/HoodDeg";
    private static final String DASHBOARD_FEED_KEY = "Shooter/Tuning/FeedKicker";
    private static final String DASHBOARD_KICKER_TORQUE_KEY = "Shooter/Tuning/KickerTorqueAmps";
    private static final String DASHBOARD_SYSID_ENABLE_KEY = "Shooter/SysId/Enabled";
    private static final String DASHBOARD_SHOT_DETECTION_ARMED_KEY = "Shooter/ShotDetection/Armed";
    private static final String DASHBOARD_SHOT_DETECTED_KEY = "Shooter/ShotDetection/ShotDetected";
    private static final String DASHBOARD_SHOTS_SINCE_ENABLE_KEY = "Shooter/ShotDetection/ShotsSinceEnable";
    private static final String DASHBOARD_SHOTS_IN_ACTIVE_SHOOT_COMMAND_KEY = "Shooter/ShotDetection/ShotsInActiveShootCommand";
    private static final String DASHBOARD_LAST_SHOOT_COMMAND_SHOTS_KEY = "Shooter/ShotDetection/LastShootCommandShots";
    private static final String DASHBOARD_LAST_SHOT_TIME_KEY = "Shooter/ShotDetection/LastShotTimestampSec";
    private static final String DASHBOARD_LAST_SHOT_ERROR_KEY = "Shooter/ShotDetection/LastShotErrorRpm";
    private static final String DASHBOARD_LAST_SHOT_DEPTH_KEY = "Shooter/ShotDetection/LastShotDepthRpm";
    private static final String DASHBOARD_LEFT_SHOT_DETECTED_KEY = "Shooter/ShotDetection/Left/ShotDetected";
    private static final String DASHBOARD_RIGHT_SHOT_DETECTED_KEY = "Shooter/ShotDetection/Right/ShotDetected";
    private static final String DASHBOARD_LEFT_SHOTS_SINCE_ENABLE_KEY = "Shooter/ShotDetection/Left/ShotsSinceEnable";
    private static final String DASHBOARD_RIGHT_SHOTS_SINCE_ENABLE_KEY = "Shooter/ShotDetection/Right/ShotsSinceEnable";
    private static final String DASHBOARD_LEFT_SHOTS_IN_ACTIVE_SHOOT_COMMAND_KEY = "Shooter/ShotDetection/Left/ShotsInActiveShootCommand";
    private static final String DASHBOARD_RIGHT_SHOTS_IN_ACTIVE_SHOOT_COMMAND_KEY = "Shooter/ShotDetection/Right/ShotsInActiveShootCommand";
    private static final String DASHBOARD_LAST_LEFT_SHOOT_COMMAND_SHOTS_KEY = "Shooter/ShotDetection/Left/LastShootCommandShots";
    private static final String DASHBOARD_LAST_RIGHT_SHOOT_COMMAND_SHOTS_KEY = "Shooter/ShotDetection/Right/LastShootCommandShots";

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

    private enum KickerControlMode {
        OFF,
        TORQUE,
        VOLTAGE
    }

    private final ShooterIO io;
    private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
    private final ShotPulseDetector leftShotPulseDetector = new ShotPulseDetector();
    private final ShotPulseDetector rightShotPulseDetector = new ShotPulseDetector();
    private final SysIdRoutine sysId;
    private StringLogEntry sysIdStateLogEntry;
    private Debouncer atSetpointDropDebouncer =
            new Debouncer(AT_SETPOINT_FALLING_DEBOUNCE_SECONDS, Debouncer.DebounceType.kFalling);

    private final InterpolatingDoubleTreeMap leftRpmByDistance = new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap rightRpmByDistance = new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap hoodAngleRadByDistance = new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap timeInAirSecondsByDistance = new InterpolatingDoubleTreeMap();
    private final int tunableId = System.identityHashCode(this);
    private LaunchCalculator launchCalculator;
    private double shotMapMinDistanceMeters = Double.NaN;
    private double shotMapMaxDistanceMeters = Double.NaN;

    private double targetLeftRpm = 0.0;
    private double targetRightRpm = 0.0;
    private double targetHoodAngleRad = ShooterConstants.HOOD_MIN_ANGLE_RAD;
    private boolean shooterTargetRequested = false;
    private double kickerOutput = 0.0;
    private KickerControlMode kickerControlMode = KickerControlMode.OFF;
    private ReadinessDiagnostics cachedReadiness = new ReadinessDiagnostics(0, 0, 0, false, false, false, false, false);
    private double activeReadinessTolerance = ShooterConstants.scoreShooterRpmTolerance();
    private String activeReadinessLabel = "SCORE";
    private boolean cachedDashboardTuningEnabled = false;
    private boolean cachedDashboardFeedKickerEnabled = false;
    private boolean cachedDashboardSysIdEnabled = false;
    private boolean wasEnabledLastCycle = false;
    private boolean shotTrackingShootCommandActive = false;
    private boolean leftShotDetectedThisCycle = false;
    private boolean rightShotDetectedThisCycle = false;
    private boolean shotDetectedThisCycle = false;
    private int leftShotsSinceEnable = 0;
    private int rightShotsSinceEnable = 0;
    private int totalShotsSinceEnable = 0;
    private int leftShotsInActiveShootCommand = 0;
    private int rightShotsInActiveShootCommand = 0;
    private int totalShotsInActiveShootCommand = 0;
    private int lastLeftShootCommandShotCount = 0;
    private int lastRightShootCommandShotCount = 0;
    private int lastShootCommandShotCount = 0;
    private double lastShotTimestampSec = Double.NaN;
    private double lastShotErrorRpm = Double.NaN;
    private double lastShotDepthRpm = Double.NaN;

    public Shooter(ShooterIO io) {
        super("shooter");
        this.io = io;
        loadShotMapFromConstants();
        loadTimeInAirMapFromConstants();
        rebuildLaunchCalculator();
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

        updateShotDetectionEnabledState();

        if (ShooterConstants.motionCompTimeScaleHasChanged(tunableId)
                || ShooterConstants.motionCompDistanceTimeScaleHasChanged(tunableId)) {
            rebuildLaunchCalculator();
        }

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
        cachedReadiness = computeReadinessDiagnostics(activeReadinessTolerance);
        logReadinessOutputs(cachedReadiness, activeReadinessTolerance, activeReadinessLabel);
        updateShotDetection();
        // Cache SmartDashboard reads once per cycle to avoid repeated NT lookups
        cachedDashboardTuningEnabled = SmartDashboard.getBoolean(DASHBOARD_ENABLE_KEY, false);
        cachedDashboardFeedKickerEnabled = SmartDashboard.getBoolean(DASHBOARD_FEED_KEY, false);
        cachedDashboardSysIdEnabled = SmartDashboard.getBoolean(DASHBOARD_SYSID_ENABLE_KEY, false);
        Logger.recordOutput("Shooter/TuningEnabled", cachedDashboardTuningEnabled);
        Logger.recordOutput("Shooter/SysIdEnabled", cachedDashboardSysIdEnabled);
    }

    private void updateShotDetectionEnabledState() {
        boolean enabled = DriverStation.isEnabled();
        if (enabled && !wasEnabledLastCycle) {
            leftShotsSinceEnable = 0;
            rightShotsSinceEnable = 0;
            totalShotsSinceEnable = 0;
            leftShotsInActiveShootCommand = 0;
            rightShotsInActiveShootCommand = 0;
            totalShotsInActiveShootCommand = 0;
            lastLeftShootCommandShotCount = 0;
            lastRightShootCommandShotCount = 0;
            lastShootCommandShotCount = 0;
            leftShotDetectedThisCycle = false;
            rightShotDetectedThisCycle = false;
            lastShotTimestampSec = Double.NaN;
            lastShotErrorRpm = Double.NaN;
            lastShotDepthRpm = Double.NaN;
            shotDetectedThisCycle = false;
            leftShotPulseDetector.reset();
            rightShotPulseDetector.reset();
        } else if (!enabled && wasEnabledLastCycle) {
            shotTrackingShootCommandActive = false;
            leftShotsInActiveShootCommand = 0;
            rightShotsInActiveShootCommand = 0;
            totalShotsInActiveShootCommand = 0;
            leftShotDetectedThisCycle = false;
            rightShotDetectedThisCycle = false;
            shotDetectedThisCycle = false;
            leftShotPulseDetector.reset();
            rightShotPulseDetector.reset();
        }
        wasEnabledLastCycle = enabled;
    }

    private void updateShotDetection() {
        boolean armed = DriverStation.isEnabled()
                && shooterTargetRequested
                && isKickerActive();
        ShotPulseDetector.UpdateResult leftResult = leftShotPulseDetector.update(
                Timer.getFPGATimestamp(),
                targetLeftRpm,
                inputs.shooterLeftVelocityRpm,
                armed);
        ShotPulseDetector.UpdateResult rightResult = rightShotPulseDetector.update(
                Timer.getFPGATimestamp(),
                targetRightRpm,
                inputs.shooterRightVelocityRpm,
                armed);
        leftShotDetectedThisCycle = leftResult.detected();
        rightShotDetectedThisCycle = rightResult.detected();
        shotDetectedThisCycle = leftShotDetectedThisCycle || rightShotDetectedThisCycle;

        if (leftShotDetectedThisCycle) {
            leftShotsSinceEnable++;
            totalShotsSinceEnable++;
            if (shotTrackingShootCommandActive) {
                leftShotsInActiveShootCommand++;
                totalShotsInActiveShootCommand++;
            }
        }
        if (rightShotDetectedThisCycle) {
            rightShotsSinceEnable++;
            totalShotsSinceEnable++;
            if (shotTrackingShootCommandActive) {
                rightShotsInActiveShootCommand++;
                totalShotsInActiveShootCommand++;
            }
        }
        if (leftShotDetectedThisCycle && rightShotDetectedThisCycle) {
            if (leftResult.candidateTimestampSec() >= rightResult.candidateTimestampSec()) {
                lastShotTimestampSec = leftResult.candidateTimestampSec();
                lastShotErrorRpm = leftResult.candidateErrorRpm();
                lastShotDepthRpm = leftResult.notchDepthRpm();
            } else {
                lastShotTimestampSec = rightResult.candidateTimestampSec();
                lastShotErrorRpm = rightResult.candidateErrorRpm();
                lastShotDepthRpm = rightResult.notchDepthRpm();
            }
        } else if (leftShotDetectedThisCycle) {
            lastShotTimestampSec = leftResult.candidateTimestampSec();
            lastShotErrorRpm = leftResult.candidateErrorRpm();
            lastShotDepthRpm = leftResult.notchDepthRpm();
        } else if (rightShotDetectedThisCycle) {
            lastShotTimestampSec = rightResult.candidateTimestampSec();
            lastShotErrorRpm = rightResult.candidateErrorRpm();
            lastShotDepthRpm = rightResult.notchDepthRpm();
        }

        Logger.recordOutput("Shooter/ShotDetection/Armed", armed);
        Logger.recordOutput("Shooter/ShotDetection/ShotDetected", shotDetectedThisCycle);
        Logger.recordOutput("Shooter/ShotDetection/ShotsSinceEnable", totalShotsSinceEnable);
        Logger.recordOutput("Shooter/ShotDetection/ShotsInActiveShootCommand", totalShotsInActiveShootCommand);
        Logger.recordOutput("Shooter/ShotDetection/LastShootCommandShots", lastShootCommandShotCount);
        Logger.recordOutput("Shooter/ShotDetection/Left/ShotDetected", leftShotDetectedThisCycle);
        Logger.recordOutput("Shooter/ShotDetection/Right/ShotDetected", rightShotDetectedThisCycle);
        Logger.recordOutput("Shooter/ShotDetection/Left/ShotsSinceEnable", leftShotsSinceEnable);
        Logger.recordOutput("Shooter/ShotDetection/Right/ShotsSinceEnable", rightShotsSinceEnable);
        Logger.recordOutput("Shooter/ShotDetection/Left/ShotsInActiveShootCommand", leftShotsInActiveShootCommand);
        Logger.recordOutput("Shooter/ShotDetection/Right/ShotsInActiveShootCommand", rightShotsInActiveShootCommand);
        Logger.recordOutput("Shooter/ShotDetection/Left/LastShootCommandShots", lastLeftShootCommandShotCount);
        Logger.recordOutput("Shooter/ShotDetection/Right/LastShootCommandShots", lastRightShootCommandShotCount);
        Logger.recordOutput("Shooter/ShotDetection/Left/CandidateTimestampSec", leftResult.candidateTimestampSec());
        Logger.recordOutput("Shooter/ShotDetection/Left/CandidateTargetRpm", leftResult.candidateTargetRpm());
        Logger.recordOutput("Shooter/ShotDetection/Left/CandidateMeasuredRpm", leftResult.candidateMeasuredRpm());
        Logger.recordOutput("Shooter/ShotDetection/Left/CandidateErrorRpm", leftResult.candidateErrorRpm());
        Logger.recordOutput("Shooter/ShotDetection/Left/CandidateNotchDepthRpm", leftResult.notchDepthRpm());
        Logger.recordOutput("Shooter/ShotDetection/Left/CandidateLeftStepRpm", leftResult.leftStepRpm());
        Logger.recordOutput("Shooter/ShotDetection/Left/CandidateRightStepRpm", leftResult.rightStepRpm());
        Logger.recordOutput("Shooter/ShotDetection/Left/CandidateRecentMaxRpm", leftResult.recentMaxRpm());
        Logger.recordOutput("Shooter/ShotDetection/Left/CandidateTargetRangeRpm", leftResult.targetRangeRpm());
        Logger.recordOutput("Shooter/ShotDetection/Right/CandidateTimestampSec", rightResult.candidateTimestampSec());
        Logger.recordOutput("Shooter/ShotDetection/Right/CandidateTargetRpm", rightResult.candidateTargetRpm());
        Logger.recordOutput("Shooter/ShotDetection/Right/CandidateMeasuredRpm", rightResult.candidateMeasuredRpm());
        Logger.recordOutput("Shooter/ShotDetection/Right/CandidateErrorRpm", rightResult.candidateErrorRpm());
        Logger.recordOutput("Shooter/ShotDetection/Right/CandidateNotchDepthRpm", rightResult.notchDepthRpm());
        Logger.recordOutput("Shooter/ShotDetection/Right/CandidateLeftStepRpm", rightResult.leftStepRpm());
        Logger.recordOutput("Shooter/ShotDetection/Right/CandidateRightStepRpm", rightResult.rightStepRpm());
        Logger.recordOutput("Shooter/ShotDetection/Right/CandidateRecentMaxRpm", rightResult.recentMaxRpm());
        Logger.recordOutput("Shooter/ShotDetection/Right/CandidateTargetRangeRpm", rightResult.targetRangeRpm());
        Logger.recordOutput("Shooter/ShotDetection/LastShotTimestampSec", lastShotTimestampSec);
        Logger.recordOutput("Shooter/ShotDetection/LastShotErrorRpm", lastShotErrorRpm);
        Logger.recordOutput("Shooter/ShotDetection/LastShotDepthRpm", lastShotDepthRpm);

        SmartDashboard.putBoolean(DASHBOARD_SHOT_DETECTION_ARMED_KEY, armed);
        SmartDashboard.putBoolean(DASHBOARD_SHOT_DETECTED_KEY, shotDetectedThisCycle);
        SmartDashboard.putBoolean(DASHBOARD_LEFT_SHOT_DETECTED_KEY, leftShotDetectedThisCycle);
        SmartDashboard.putBoolean(DASHBOARD_RIGHT_SHOT_DETECTED_KEY, rightShotDetectedThisCycle);
        SmartDashboard.putNumber(DASHBOARD_SHOTS_SINCE_ENABLE_KEY, totalShotsSinceEnable);
        SmartDashboard.putNumber(DASHBOARD_LEFT_SHOTS_SINCE_ENABLE_KEY, leftShotsSinceEnable);
        SmartDashboard.putNumber(DASHBOARD_RIGHT_SHOTS_SINCE_ENABLE_KEY, rightShotsSinceEnable);
        SmartDashboard.putNumber(DASHBOARD_SHOTS_IN_ACTIVE_SHOOT_COMMAND_KEY, totalShotsInActiveShootCommand);
        SmartDashboard.putNumber(DASHBOARD_LEFT_SHOTS_IN_ACTIVE_SHOOT_COMMAND_KEY, leftShotsInActiveShootCommand);
        SmartDashboard.putNumber(DASHBOARD_RIGHT_SHOTS_IN_ACTIVE_SHOOT_COMMAND_KEY, rightShotsInActiveShootCommand);
        SmartDashboard.putNumber(DASHBOARD_LAST_SHOOT_COMMAND_SHOTS_KEY, lastShootCommandShotCount);
        SmartDashboard.putNumber(DASHBOARD_LAST_LEFT_SHOOT_COMMAND_SHOTS_KEY, lastLeftShootCommandShotCount);
        SmartDashboard.putNumber(DASHBOARD_LAST_RIGHT_SHOOT_COMMAND_SHOTS_KEY, lastRightShootCommandShotCount);
        SmartDashboard.putNumber(DASHBOARD_LAST_SHOT_TIME_KEY, lastShotTimestampSec);
        SmartDashboard.putNumber(DASHBOARD_LAST_SHOT_ERROR_KEY, lastShotErrorRpm);
        SmartDashboard.putNumber(DASHBOARD_LAST_SHOT_DEPTH_KEY, lastShotDepthRpm);
    }

    private void beginShotTrackingForShootCommand() {
        shotTrackingShootCommandActive = true;
        leftShotsInActiveShootCommand = 0;
        rightShotsInActiveShootCommand = 0;
        totalShotsInActiveShootCommand = 0;
        leftShotDetectedThisCycle = false;
        rightShotDetectedThisCycle = false;
        shotDetectedThisCycle = false;
        leftShotPulseDetector.reset();
        rightShotPulseDetector.reset();
    }

    private void endShotTrackingForShootCommand() {
        lastLeftShootCommandShotCount = leftShotsInActiveShootCommand;
        lastRightShootCommandShotCount = rightShotsInActiveShootCommand;
        lastShootCommandShotCount = totalShotsInActiveShootCommand;
        leftShotsInActiveShootCommand = 0;
        rightShotsInActiveShootCommand = 0;
        totalShotsInActiveShootCommand = 0;
        shotTrackingShootCommandActive = false;
        leftShotDetectedThisCycle = false;
        rightShotDetectedThisCycle = false;
        shotDetectedThisCycle = false;
        leftShotPulseDetector.reset();
        rightShotPulseDetector.reset();
    }

    private static void logReadinessOutputs(
            ReadinessDiagnostics readiness, double shooterRpmTolerance, String readinessLabel) {
        Logger.recordOutput("Shooter/Readiness/LeftVelocityErrorRpm", readiness.leftVelocityErrorRpm());
        Logger.recordOutput("Shooter/Readiness/RightVelocityErrorRpm", readiness.rightVelocityErrorRpm());
        Logger.recordOutput("Shooter/Readiness/HoodAngleErrorDeg", Units.radiansToDegrees(readiness.hoodAngleErrorRad()));
        Logger.recordOutput("Shooter/Readiness/LeftVelocityAtSetpoint", readiness.leftVelocityAtSetpoint());
        Logger.recordOutput("Shooter/Readiness/RightVelocityAtSetpoint", readiness.rightVelocityAtSetpoint());
        Logger.recordOutput("Shooter/Readiness/HoodAngleAtSetpoint", readiness.hoodAngleAtSetpoint());
        Logger.recordOutput("Shooter/AtSetpoint", readiness.atSetpoint());
        Logger.recordOutput("Shooter/ReadyToFire", readiness.readyToFire());
        Logger.recordOutput("Shooter/Readiness/Mode", readinessLabel);
        Logger.recordOutput("Shooter/Readiness/RpmTolerance", shooterRpmTolerance);
    }

    private void applyKickerOutput() {
        switch (kickerControlMode) {
            case TORQUE -> io.setKickerTorque(kickerOutput);
            case VOLTAGE -> io.setKickerVoltage(kickerOutput);
            case OFF -> io.setKickerTorque(0.0);
        }
    }

    public void setTargets(ShotSetpoint setpoint) {
        setTargetsInternal(setpoint.leftRpm(), setpoint.rightRpm(), setpoint.hoodAngleRad(), true);
    }

    public void setTargets(double leftRpm, double rightRpm, double hoodAngleRad) {
        setTargetsInternal(leftRpm, rightRpm, hoodAngleRad, true);
    }

    private void setTargetsInternal(
            double leftRpm,
            double rightRpm,
            double hoodAngleRad,
            boolean allowBackgroundHoldoff) {
        if (allowBackgroundHoldoff && RobotBase.isSimulation()) {
            cancelBackgroundShooterCommandsIfOnlyBackgroundIsActive();
        }
        targetLeftRpm = MathUtil.clamp(leftRpm, -ShooterConstants.SHOOTER_MAX_RPM, ShooterConstants.SHOOTER_MAX_RPM);
        io.setLeftVelocity(targetLeftRpm);
        targetRightRpm = MathUtil.clamp(rightRpm, -ShooterConstants.SHOOTER_MAX_RPM, ShooterConstants.SHOOTER_MAX_RPM);
        io.setRightVelocity(targetRightRpm);
        targetHoodAngleRad = MathUtil.clamp(hoodAngleRad, ShooterConstants.HOOD_MIN_ANGLE_RAD, ShooterConstants.HOOD_MAX_ANGLE_RAD);
        io.setHoodAngle(targetHoodAngleRad);
        shooterTargetRequested = Math.abs(targetLeftRpm) > 1e-6 || Math.abs(targetRightRpm) > 1e-6;
    }

    public void setTargetsForDistance(double distanceMeters) {
        if (!Double.isFinite(distanceMeters)) {
            stopAll();
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
        resetActiveReadinessProfile();
        setIdleTargets();
        stopKicker();
        io.stop();
    }

    private void setIdleTargets() {
        targetLeftRpm = 0.0;
        targetRightRpm = 0.0;
        targetHoodAngleRad = ShooterConstants.HOOD_MIN_ANGLE_RAD;
        shooterTargetRequested = false;
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
        return computeReadinessDiagnostics(ShooterConstants.scoreShooterRpmTolerance());
    }

    public ReadinessDiagnostics getReadinessDiagnosticsNow(double shooterRpmTolerance) {
        return computeReadinessDiagnostics(shooterRpmTolerance);
    }

    public void setActiveReadinessProfile(double shooterRpmTolerance, String readinessLabel) {
        String normalizedLabel = normalizeReadinessLabel(readinessLabel);
        if (Double.compare(activeReadinessTolerance, shooterRpmTolerance) == 0
                && Objects.equals(activeReadinessLabel, normalizedLabel)) {
            return;
        }
        activeReadinessTolerance = shooterRpmTolerance;
        activeReadinessLabel = normalizedLabel;
        atSetpointDropDebouncer =
                new Debouncer(AT_SETPOINT_FALLING_DEBOUNCE_SECONDS, Debouncer.DebounceType.kFalling);
    }

    public void resetActiveReadinessProfile() {
        setActiveReadinessProfile(ShooterConstants.scoreShooterRpmTolerance(), "SCORE");
    }

    public void publishActiveReadiness(
            ReadinessDiagnostics readiness, double shooterRpmTolerance, String readinessLabel) {
        setActiveReadinessProfile(shooterRpmTolerance, readinessLabel);
        cachedReadiness = readiness;
        logReadinessOutputs(cachedReadiness, activeReadinessTolerance, activeReadinessLabel);
    }

    private ReadinessDiagnostics computeReadinessDiagnostics(double shooterRpmTolerance) {
        double leftVelocityErrorRpm = targetLeftRpm - inputs.shooterLeftVelocityRpm;
        double rightVelocityErrorRpm = targetRightRpm - inputs.shooterRightVelocityRpm;
        double hoodAngleErrorRad = targetHoodAngleRad - inputs.hoodPositionRad;

        boolean leftVelocityAtSetpoint =
                Math.abs(leftVelocityErrorRpm) <= shooterRpmTolerance;
        boolean rightVelocityAtSetpoint =
                Math.abs(rightVelocityErrorRpm) <= shooterRpmTolerance;
        boolean hoodAngleAtSetpoint =
                Math.abs(hoodAngleErrorRad) <= ShooterConstants.HOOD_ANGLE_TOLERANCE_RAD;

        boolean shooterRpmAtSetpoint = leftVelocityAtSetpoint && rightVelocityAtSetpoint;
        boolean debouncedShooterRpmAtSetpoint = atSetpointDropDebouncer.calculate(shooterRpmAtSetpoint);
        boolean atSetpoint = debouncedShooterRpmAtSetpoint && hoodAngleAtSetpoint;
        boolean readyToFire = atSetpoint && shooterTargetRequested;

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

    private static String normalizeReadinessLabel(String readinessLabel) {
        return readinessLabel == null || readinessLabel.isBlank() ? "SCORE" : readinessLabel;
    }

    public boolean isKickerActive() {
        return kickerControlMode != KickerControlMode.OFF && Math.abs(kickerOutput) > 1e-6;
    }

    public boolean didDetectShotPulseThisCycle() {
        return shotDetectedThisCycle;
    }

    public int getShotsSinceEnable() {
        return totalShotsSinceEnable;
    }

    public int getLeftShotsSinceEnable() {
        return leftShotsSinceEnable;
    }

    public int getRightShotsSinceEnable() {
        return rightShotsSinceEnable;
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

    public LaunchCalculator.MotionCompensation getMotionCompensationToHub(
            Pose2d robotPose, ChassisSpeeds robotRelativeSpeeds) {
        LaunchCalculator.MotionCompensation result = launchCalculator.calculate(robotPose, robotRelativeSpeeds);
        logMotionCompensation(result, robotPose);
        return result;
    }

    public LaunchCalculator.MotionCompensation getMotionCompensationToHub(
            Pose2d robotPose,
            ChassisSpeeds robotRelativeSpeeds,
            LaunchCalculator.CompensationTracker compensationTracker) {
        LaunchCalculator.MotionCompensation result =
                launchCalculator.calculate(robotPose, robotRelativeSpeeds, compensationTracker);
        logMotionCompensation(result, robotPose);
        return result;
    }

    private void logMotionCompensation(LaunchCalculator.MotionCompensation result, Pose2d robotPose) {
        Logger.recordOutput("Shooter/MotionCompTimeScale", ShooterConstants.motionCompTimeScale());
        Logger.recordOutput(
                "Shooter/MotionCompDistanceTimeScale",
                ShooterConstants.motionCompDistanceTimeScale());
        Logger.recordOutput("Shooter/RawHubDistanceMeters", result.rawDistanceMeters());
        Logger.recordOutput("Shooter/CompensatedHubDistanceMeters", result.compensatedDistanceMeters());
        Logger.recordOutput("Shooter/VelocityTowardHubMps", result.velocityTowardHubMps());
        Logger.recordOutput("Shooter/VelocityPerpendicularHubMps", result.velocityPerpendicularHubMps());
        Logger.recordOutput("Shooter/TimeInAirSec", result.timeInAirSec());
        Logger.recordOutput("Shooter/CompensatedHubHeadingDeg",
                result.compensatedHeading() != null ? result.compensatedHeading().getDegrees() : Double.NaN);
        Logger.recordOutput("Shooter/DesiredRobotHeadingDeg",
                result.desiredRobotHeading() != null ? result.desiredRobotHeading().getDegrees() : Double.NaN);
        Logger.recordOutput("Shooter/DesiredRobotHeadingRateRadPerSec", result.desiredHeadingRateRadPerSec());

        // Publish shooter pose for AdvantageKit visualization
        if (robotPose != null) {
            Translation2d shooterFieldPos = robotPose.getTranslation().plus(
                    ShooterConstants.ROBOT_TO_SHOOTER_OFFSET.rotateBy(robotPose.getRotation()));
            Logger.recordOutput("Shooter/ShooterPose3d", new Pose3d(
                    shooterFieldPos.getX(),
                    shooterFieldPos.getY(),
                    ShooterConstants.SHOOTER_HEIGHT_METERS,
                    new Rotation3d(0.0, 0.0, robotPose.getRotation().getRadians())));
        }
    }

    /** Rebuilds the LaunchCalculator after shot map or time-in-air map changes. */
    private void rebuildLaunchCalculator() {
        launchCalculator = new LaunchCalculator(
                this::getTimeInAirSecondsForDistance,
                ShooterConstants.ROBOT_TO_SHOOTER_OFFSET,
                ShooterConstants.PHASE_DELAY_SEC,
                ShooterConstants.motionCompTimeScale(),
                ShooterConstants.motionCompDistanceTimeScale());
    }

    private double getTimeInAirSecondsForDistance(double distanceMeters) {
        if (!Double.isFinite(distanceMeters) || distanceMeters < 0.0) {
            return 0.0;
        }
        double time = timeInAirSecondsByDistance.get(distanceMeters);
        return Double.isFinite(time) && time >= 0.0 ? time : 0.0;
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
        } else if (!Double.isFinite(distanceMeters)) {
            stopAll();
        } else {
            setTargetsForDistance(distanceMeters);
        }
    }

    public Command shoot(
            DoubleSupplier distanceMetersSupplier, DoubleSupplier kickerTorqueAmpsSupplier) {
        return Commands.run(
                        () -> {
                            setTargetsForDistance(distanceMetersSupplier.getAsDouble());
                            if (getReadinessDiagnosticsNow().atSetpoint()) {
                                setKickerTorqueAmps(kickerTorqueAmpsSupplier.getAsDouble());
                            } else {
                                stopKicker();
                            }
                        },
                        this)
                .beforeStarting(this::beginShotTrackingForShootCommand)
                .finallyDo(interrupted -> {
                    endShotTrackingForShootCommand();
                    stopAll();
                })
                .withName("ShooterShoot");
    }

    public Command slowShooterMotorsCommand() {
        return Commands.run(
                () -> {
                    setTargetsInternal(
                            ShooterConstants.SLOW_SHOOTER_RPM,
                            ShooterConstants.SLOW_SHOOTER_RPM,
                            ShooterConstants.HOOD_MIN_ANGLE_RAD,
                            false);
                    stopKicker();
                },
                this)
                .withName("ShooterSlowShooterMotors");
    }

    private void cancelBackgroundShooterCommandsIfOnlyBackgroundIsActive() {
        Command current = getCurrentCommand();
        if (current != null && isBackgroundLikeShooterCommand(current.getName())) {
            CommandScheduler.getInstance().cancel(current);
        }
    }

    private static boolean isBackgroundLikeShooterCommand(String commandName) {
        return "ShooterBackground".equals(commandName)
                || "ShooterSlowShooterMotors".equals(commandName);
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
        rebuildLaunchCalculator();
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
        SmartDashboard.setDefaultBoolean(DASHBOARD_SHOT_DETECTION_ARMED_KEY, false);
        SmartDashboard.setDefaultBoolean(DASHBOARD_SHOT_DETECTED_KEY, false);
        SmartDashboard.setDefaultBoolean(DASHBOARD_LEFT_SHOT_DETECTED_KEY, false);
        SmartDashboard.setDefaultBoolean(DASHBOARD_RIGHT_SHOT_DETECTED_KEY, false);
        SmartDashboard.setDefaultNumber(DASHBOARD_SHOTS_SINCE_ENABLE_KEY, 0.0);
        SmartDashboard.setDefaultNumber(DASHBOARD_LEFT_SHOTS_SINCE_ENABLE_KEY, 0.0);
        SmartDashboard.setDefaultNumber(DASHBOARD_RIGHT_SHOTS_SINCE_ENABLE_KEY, 0.0);
        SmartDashboard.setDefaultNumber(DASHBOARD_SHOTS_IN_ACTIVE_SHOOT_COMMAND_KEY, 0.0);
        SmartDashboard.setDefaultNumber(DASHBOARD_LEFT_SHOTS_IN_ACTIVE_SHOOT_COMMAND_KEY, 0.0);
        SmartDashboard.setDefaultNumber(DASHBOARD_RIGHT_SHOTS_IN_ACTIVE_SHOOT_COMMAND_KEY, 0.0);
        SmartDashboard.setDefaultNumber(DASHBOARD_LAST_SHOOT_COMMAND_SHOTS_KEY, 0.0);
        SmartDashboard.setDefaultNumber(DASHBOARD_LAST_LEFT_SHOOT_COMMAND_SHOTS_KEY, 0.0);
        SmartDashboard.setDefaultNumber(DASHBOARD_LAST_RIGHT_SHOOT_COMMAND_SHOTS_KEY, 0.0);
        SmartDashboard.setDefaultNumber(DASHBOARD_LAST_SHOT_TIME_KEY, Double.NaN);
        SmartDashboard.setDefaultNumber(DASHBOARD_LAST_SHOT_ERROR_KEY, Double.NaN);
        SmartDashboard.setDefaultNumber(DASHBOARD_LAST_SHOT_DEPTH_KEY, Double.NaN);
    }

    public static double getHubDistanceMeters(Pose2d robotPose) {
        return LaunchCalculator.getHubDistanceMeters(robotPose);
    }

    public Command homeCommand() {
        AtomicBoolean homingReached = new AtomicBoolean(false);
        BooleanSupplier atHomingStop =
                () -> Math.abs(inputs.hoodStatorCurrentAmps) > ShooterConstants.HOMING_CURRENT_THRESHOLD_AMPS;
        BooleanSupplier latchHomingStop = () -> {
            boolean reached = atHomingStop.getAsBoolean();
            if (reached) {
                homingReached.set(true);
            }
            return reached;
        };
        double preAimAngleRad = initialHoodPreAimAngleRad();
        return Commands.sequence(
            Commands.runOnce(() -> {
                        homingReached.set(false);
                        io.setHoodVoltage(homingVoltage());
                    }, this),
            Commands.waitUntil(latchHomingStop)
                    .withTimeout(homingWaitTimeoutSec())
                    .withName("HoodHomeWaitUntil"),
            Commands.either(
                    Commands.sequence(
                            Commands.runOnce(io::stop, this),
                            Commands.runOnce(io::resetHoodEncoder, this),
                            Commands.runOnce(() -> io.setHoodAngle(preAimAngleRad), this),
                            Commands.waitUntil(() -> Math.abs(inputs.hoodPositionRad - preAimAngleRad)
                                            <= ShooterConstants.HOOD_ANGLE_TOLERANCE_RAD)
                                    .withTimeout(homePreAimSettleTimeoutSec())
                                    .withName("HoodHomePreAimWaitUntil")),
                    Commands.sequence(
                            Commands.runOnce(io::stop, this),
                            Commands.runOnce(
                                    () -> DriverStation.reportWarning(
                                            "Hood homing timed out before current threshold; skipping encoder reset/retract.",
                                            false),
                                    this),
                            Commands.runOnce(
                                    () -> ElasticNotifications.sendWarning(
                                            "Shooter",
                                            "Hood homing timed out; using current hood position until re-homed."),
                                    this)),
                    homingReached::get))
                .finallyDo(interrupted -> {
                    if (interrupted) {
                        io.stop();
                    }
                })
                .withName("ShooterHome");
    }

    private static double homingVoltage() {
        return RobotBase.isSimulation() ? -12.0 : -2.0;
    }

    private static double homingWaitTimeoutSec() {
        return ShooterConstants.HOMING_WAIT_TIMEOUT_SEC;
    }

    private static double homePreAimSettleTimeoutSec() {
        return RobotBase.isReal() ? 0.8 : 1.2;
    }

    private static double initialHoodPreAimAngleRad() {
        if (ShooterConstants.SHOT_MAP_HOOD_ANGLE_DEG.length == 0) {
            return ShooterConstants.HOOD_MIN_ANGLE_RAD;
        }
        double preAimRad = Units.degreesToRadians(ShooterConstants.SHOT_MAP_HOOD_ANGLE_DEG[0]);
        return MathUtil.clamp(preAimRad, ShooterConstants.HOOD_MIN_ANGLE_RAD, ShooterConstants.HOOD_MAX_ANGLE_RAD);
    }

}
